"""
demo_eval.py — 空间实体验证演示:记忆与真值的对账闭环

流程:
  ① 冷启动审计     首次巡逻后,记忆 vs 真值做 L1/L2/L3 对账
  ② 注入缺陷       人为制造一个"幻觉实体"(FP)+ 删除一条记忆(FN),
                    验证审计器能否精确点名问题实体
  ③ 时序审计(L4)  世界变化后,测记忆需要几轮巡逻才收敛到新真值
                    —— 这就是"记忆新鲜度"指标
  ④ 检索审计(R)   自然语言查询集的 Recall@3

运行:  python demo_eval.py
"""
import time

from spatial_memory import build_system
from spatial_memory.eval import EntityValidator, gt_from_world
from spatial_memory.schema import (Detection, ObservationEvent, Pose,
                                   EntityStatus)
from spatial_memory.synthetic import (SimulatedRobot, SyntheticWorld,
                                      WorldObject, toy_embed)

DAY_US = 24 * 3600 * 10**6


def banner(t):
    print(f"\n{'=' * 62}\n  {t}\n{'=' * 62}")


def main():
    sys = build_system(":memory:")
    world = SyntheticWorld()
    robot = SimulatedRobot(world)
    validator = EntityValidator(sys.entities, sys.query)

    sys.entities.upsert_submap(world.submap)
    for p in world.places:
        sys.entities.upsert_place(p)

    t0 = int(time.time() * 1e6)
    day = [0]

    def patrol_and_ingest():
        for obs in robot.patrol(timestamp_us=t0 + day[0] * DAY_US):
            sys.consolidator.ingest(obs)
        day[0] += 1

    # ---------------- ① 冷启动审计 ----------------
    banner("① 冷启动审计:首次巡逻后的 L1/L2/L3 对账")
    patrol_and_ingest()
    report = validator.audit_snapshot(gt_from_world(world))
    print(report.pretty())

    # ---------------- ② 注入缺陷,验证审计器灵敏度 ----------------
    banner("② 缺陷注入测试:审计器能否点名问题实体?")
    # 制造幻觉实体:直接给整合器喂一条不存在的检测
    fake = ObservationEvent(
        submap_id=world.SUBMAP_ID, place_id="corridor",
        robot_id="robot_01",
        detections=[Detection("vending_machine", Pose(17, 6, 0.9),
                              embedding=toy_embed("vending_machine"),
                              score=0.9)],
        view_center=Pose(17, 6, 1.0), view_radius=1.5,
        timestamp_us=t0 + day[0] * DAY_US)
    sys.consolidator.ingest(fake)
    # 制造遗忘:手工把白板置为 retired(模拟错误的 RETIRE 决策)
    wb = [e for e in sys.query.objects_in_place("office_b")
          if e.class_label == "whiteboard"][0]
    wb.status = EntityStatus.RETIRED
    sys.entities.upsert_object(wb)

    report = validator.audit_snapshot(gt_from_world(world))
    print(report.pretty())
    # 恢复现场(撤销注入的缺陷)
    wb.status = EntityStatus.ACTIVE
    sys.entities.upsert_object(wb)

    # ---------------- ③ L4 时序审计:变化收敛轮数 ----------------
    banner("③ 时序审计:环境变化后,记忆几轮巡逻收敛?")
    world.move_object("coffee_cup", Pose(4.4, 5.5, 0.8), "meeting_a")
    world.set_attribute("door", "state", "closed")
    world.remove_object("potted_plant")
    world.objects.append(WorldObject("water_dispenser",
                                     Pose(18, 5, 0.6), "corridor"))
    print("  世界变化: 杯子挪动 / 门关闭 / 盆栽移除 / 新增饮水机")
    rounds, final = validator.audit_change(
        gt_from_world(world), patrol_and_ingest, max_rounds=6)
    if rounds:
        print(f"  ✓ 记忆在 {rounds} 轮巡逻后与新真值完全对账")
    else:
        print("  ✗ 未在限定轮数内收敛,最终状态:")
    print(final.pretty())
    print("  说明: 收敛需要多轮是符合预期的 —— 消失实体的 RETIRE")
    print("  依赖负观测置信度累积(semi_static 需 3 次未见),这是")
    print("  '抗噪稳健性' 与 '变化响应速度' 之间的刻意权衡,")
    print("  decay_step 参数即调节旋钮。")

    # ---------------- ④ 检索审计 ----------------
    banner("④ 检索审计: 自然语言查询 Recall@3")
    cases = [
        ("fire extinguisher", "fire_extinguisher"),
        ("red safety equipment", "fire_extinguisher"),
        ("water dispenser", "water_dispenser"),
        ("whiteboard for writing", "whiteboard"),
        ("conference table", "conference_table"),
        ("standing desk", "standing_desk"),
    ]
    recall = validator.audit_retrieval(cases, top_k=3)
    print(f"  Recall@3 = {recall:.3f}  ({len(cases)} 条查询)")
    print("  注: M0 玩具嵌入只有字面相似性;M1 换 CLIP/SigLIP 后")
    print("  本查询集应扩充真实用户问法(含中文)并作为回归基线。")

    banner("验证闭环建立完成 — 每次改动整合器/感知,跑本脚本即知优劣")


if __name__ == "__main__":
    main()
