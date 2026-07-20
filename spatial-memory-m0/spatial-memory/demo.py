"""
demo.py — 端到端演示:一条完整的空间记忆生命周期

场景脚本:
  Day 1  机器人首次巡逻          → 全部 ADD,记忆建立
  Day 2  环境变化:咖啡杯被人从会议室带到办公室;走廊的门被关上;
         会议室一把椅子被搬走
         机器人再次巡逻          → UPDATE / CONFIRM / DECAY 各就各位
  Day 3-4 继续巡逻               → 消失的椅子连续未见 → RETIRE
  查询验证:
    ① 语义检索: "红色灭火器在哪?"
    ② 房间清单: 办公室 B 现在有什么?
    ③ 时间旅行: 咖啡杯 Day 1 的时候在哪?
    ④ 事件审计: 那把消失的椅子经历了什么?
    ⑤ 人工修正: 把 TV 屏幕改名并 pin 保护
    ⑥ 增量同步: 另一台机器人按版本号拉取变更

运行:  python demo.py
"""
import time

from spatial_memory import build_system
from spatial_memory.schema import Pose
from spatial_memory.synthetic import SimulatedRobot, SyntheticWorld

DAY_US = 24 * 3600 * 10**6


def banner(title):
    print(f"\n{'=' * 62}\n  {title}\n{'=' * 62}")


def show_events(evs):
    from collections import Counter
    c = Counter(e.event_type.value for e in evs)
    print("  事件统计:", dict(c))


def main():
    sys = build_system(":memory:")
    world = SyntheticWorld()
    robot = SimulatedRobot(world)

    # 静态结构入库(首建管线的产物,M0 直接由合成世界给出)
    sys.entities.upsert_submap(world.submap)
    for p in world.places:
        sys.entities.upsert_place(p)

    t0 = int(time.time() * 1e6)

    # ---------------- Day 1: 首次巡逻,记忆建立 ----------------
    banner("Day 1 — 首次巡逻(冷启动建图)")
    all_evs = []
    for obs in robot.patrol(timestamp_us=t0):
        all_evs += sys.consolidator.ingest(obs)
    show_events(all_evs)
    for pid in ["meeting_a", "corridor", "office_b"]:
        objs = sys.query.objects_in_place(pid)
        print(f"  {pid:10s}: {[o.class_label for o in objs]}")

    # ---------------- Day 2: 环境变化 ----------------
    banner("Day 2 — 环境发生变化后巡逻")
    # 变化1: 咖啡杯在会议室内被挪动约 1 米(M0 可重识别 → UPDATE)
    #        注: 跨房间移动的重识别是 M2 课题,见 ROADMAP
    world.move_object("coffee_cup", Pose(4.4, 5.5, 0.8), "meeting_a")
    # 变化2: 走廊的门被关上(属性变化 → UPDATE)
    world.set_attribute("door", "state", "closed")
    # 变化3: 会议室一把椅子被搬走(负观测 → DECAY → RETIRE)
    world.remove_object("office_chair")   # remove 会移除同名全部,再加回一把
    from spatial_memory.synthetic import WorldObject
    world.objects.append(WorldObject("office_chair", Pose(6, 7, 0.3), "meeting_a"))

    all_evs = []
    for obs in robot.patrol(timestamp_us=t0 + 1 * DAY_US):
        all_evs += sys.consolidator.ingest(obs)
    show_events(all_evs)
    for ev in all_evs:
        if ev.event_type.value in ("update", "decay"):
            e = sys.entities.get_object(ev.entity_uuid)
            print(f"  [{ev.event_type.value.upper():6s}] {e.class_label:18s} {ev.payload}")

    # ---------------- Day 3-4: 持续巡逻,消失实体退场 ----------------
    banner("Day 3-4 — 持续巡逻,负观测累积")
    for day in (2, 3):
        all_evs = []
        for obs in robot.patrol(timestamp_us=t0 + day * DAY_US):
            all_evs += sys.consolidator.ingest(obs)
        show_events(all_evs)

    # ---------------- 查询验证 ----------------
    banner("查询 ① 语义检索: '红色灭火器在哪?'")
    for e, score in sys.query.semantic_search("red fire extinguisher", top_k=3):
        place = sys.entities.get_place(e.place_id)
        print(f"  {score:.3f}  {e.class_label:20s} @ {place.name} "
              f"({e.pose.x:.1f},{e.pose.y:.1f}) conf={e.confidence:.2f}")

    banner("查询 ② 房间清单: 办公室 B 现在有什么?")
    for e in sys.query.objects_in_place("office_b"):
        print(f"  {e.class_label:20s} conf={e.confidence:.2f} "
              f"obs={e.observation_count} by={e.last_updated_by}")

    banner("查询 ③ 时间旅行: 咖啡杯 Day 1 时在哪?")
    cup = [e for e in sys.query.objects_in_place("meeting_a")
           if e.class_label == "coffee_cup"][0]
    past = sys.query.where_was(cup.uuid, at_us=t0 + 12 * 3600 * 10**6)
    print(f"  Day1 位置: pose=({past['pose']['x']:.1f},{past['pose']['y']:.1f})"
          f"  place={past['place_id']}")
    print(f"  当前位置: pose=({cup.pose.x:.1f},{cup.pose.y:.1f})"
          f"  (Day2 被挪动,事件回放可精确还原历史位姿)")

    banner("查询 ④ 事件审计: 一把被搬走的椅子经历了什么?")
    retired = [r for r in sys.entities.objects_in_place("meeting_a", include_retired=True)
               if r.status.value == "retired"]
    if retired:
        for ev in sys.query.entity_history(retired[0].uuid):
            print(f"  v{ev.entity_version} {ev.event_type.value:8s} "
                  f"src={ev.source:9s} {ev.payload}")

    banner("查询 ⑤ 人工修正 + pin 保护")
    tv = [e for e in sys.query.objects_in_place("meeting_a")
          if e.class_label == "tv_screen"][0]
    sys.consolidator.human_correct(
        tv.uuid, {"class_label": "conference_display",
                  "aliases": ["会议大屏"]}, user_id="alice")
    tv2 = sys.entities.get_object(tv.uuid)
    print(f"  修正后: label={tv2.class_label} aliases={tv2.aliases} "
          f"pinned={tv2.pinned_until_us > 0} by={tv2.last_updated_by}")

    banner("查询 ⑥ 增量同步: 机器人 B 拉取 version>1 的变更")
    delta = sys.query.changes_since("submap_f3_a", after_version=1)
    print(f"  需同步事件数: {len(delta)}")
    for ev in delta[:5]:
        print(f"  {ev.event_type.value:14s} v{ev.entity_version} {ev.payload}")
    print("  ...")

    banner("演示结束 — 完整生命周期: ADD→CONFIRM→UPDATE→DECAY→RETIRE→CORRECT→SYNC")


if __name__ == "__main__":
    main()
