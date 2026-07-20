"""
Phase5 sim validation, step 1b -- T1-T11 scripted scenarios (T12, the
occlusion trap, needs real mesh raycasting and lives in
p5sim_occlusion_check.py instead, per work/p5_sim_validation/PLAN.md).

Each test case is a plain function that drives a live SpatialMemorySystem
through a sequence of consolidator.ingest() / consolidator.human_correct()
calls and returns a CaseResult recording what we EXPECT to have happened.
p5sim_run_1b.py runs every case, then separately pulls the ACTUAL event
history per entity_uuid from the store and diffs it against these
expectations -- this file only encodes intent, it never reads results back.

All test objects use synthetic per-test class_labels (t1_widget, t2_widget,
...) placed far from the real scene_04 instances and from each other, so
cross-test/cross-object matching interference is structurally impossible
regardless of exact coordinates -- no need to carefully avoid geometric
overlap between unrelated test cases.

Every ObservationEvent/human_correct timestamp uses real wall-clock
`now_us()` (offset by whole seconds), NOT a synthetic logical clock starting
at 0 -- required for T9, where `pinned_until_us` (set from real now_us() by
human_correct()) is compared directly against `obs.timestamp_us`. Mixing
clock domains across tests would make that comparison meaningless, so every
test uses the same real-clock convention for consistency even where it
doesn't strictly matter (T1-T8, T10-T11).

Pre-registered predictions for T3/T7/T9/T10 are written up BEFORE this file
was executed, in PLAN.md §2.5 -- read that section for the reasoning; this
file only implements the mechanics.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from spatial_memory.schema import (Detection, EventType, Mobility, ObservationEvent,
                                    Pose, SpatialEvent, new_uuid, now_us)

SUBMAP_ID = "sim_submap_scene04_1b"


@dataclass
class CaseResult:
    id: str
    description: str
    keys: list[str]                      # synthetic object keys, in creation order
    expected_events: dict[str, list[str]] = field(default_factory=dict)
    expected_final: dict[str, dict] = field(default_factory=dict)  # key -> {field: value}
    uuid_of: dict[str, str] = field(default_factory=dict)          # filled during run
    prediction_note: str = ""


def _one_det(label, x, y, z=0.0, yaw=0.0, score=1.0, attrs=None):
    return Detection(class_label=label, pose=Pose(x, y, z, yaw),
                      embedding=None, attributes=dict(attrs or {}), score=score)


def _obs(place_id, dets, cx, cy, radius, ts, robot="sim_1b"):
    return ObservationEvent(submap_id=SUBMAP_ID, place_id=place_id, robot_id=robot,
                             detections=dets, view_center=Pose(cx, cy, 0.0, 0.0),
                             view_radius=radius, timestamp_us=ts)


def _patch_mobility(sys_, uuid, mobility):
    e = sys_.entities.get_object(uuid)
    e.mobility = mobility
    sys_.entities.upsert_object(e)


# ---------------------------------------------------------------- T1
def t1_repeat_no_move(sys_):
    """不动,观测3次: ADD -> CONFIRM -> CONFIRM."""
    label, place = "t1_widget", "apartment_unit"
    x, y = 500.0, 500.0
    t0 = now_us()
    r1 = sys_.consolidator.ingest(_obs(place, [_one_det(label, x, y)], x, y, 2.0, t0))
    uuid = r1[0].entity_uuid
    _patch_mobility(sys_, uuid, Mobility.SEMI_STATIC)
    sys_.consolidator.ingest(_obs(place, [_one_det(label, x, y)], x, y, 2.0, t0 + 1_000_000))
    sys_.consolidator.ingest(_obs(place, [_one_det(label, x, y)], x, y, 2.0, t0 + 2_000_000))
    return CaseResult(
        id="T1", description="原地重复,观测3次",
        keys=["t1"], uuid_of={"t1": uuid},
        expected_events={"t1": ["add", "confirm", "confirm"]},
        expected_final={"t1": {"version": 1, "confidence": 1.0, "observation_count": 3}},
        prediction_note=("方案文档写 conf=1.2,但 _do_confirm 用 min(1.0, conf+0.1) 封顶,"
                          "1.0+0.1+0.1 会被两次封顶在 1.0,不可能到 1.2 —— 这是方案文档"
                          "算漏了 clamp,不是实现问题,预期值按代码实际公式改成 1.0。"))


# ---------------------------------------------------------------- T2
def t2_large_displacement(sys_):
    """第2次观测前移动0.5m(>0.30m 阈值): ADD -> UPDATE."""
    label, place = "t2_widget", "apartment_unit"
    x, y = 520.0, 500.0
    t0 = now_us()
    r1 = sys_.consolidator.ingest(_obs(place, [_one_det(label, x, y)], x, y, 2.0, t0))
    uuid = r1[0].entity_uuid
    _patch_mobility(sys_, uuid, Mobility.SEMI_STATIC)
    new_x = x + 0.5
    sys_.consolidator.ingest(_obs(place, [_one_det(label, new_x, y)], new_x, y, 2.0, t0 + 1_000_000))
    return CaseResult(
        id="T2", description="第2次观测前移动0.5m(>阈值)",
        keys=["t2"], uuid_of={"t2": uuid},
        expected_events={"t2": ["add", "update"]},
        expected_final={"t2": {"version": 2, "confidence": 1.0,
                                "pose": {"x": new_x, "y": y, "z": 0.0, "yaw": 0.0}}},
        prediction_note="同 T1,方案文档 conf=1.2 应为 min(1.0, 1.0+0.2)=1.0(clamp 疏漏)。")


# ---------------------------------------------------------------- T3
def t3_small_displacement(sys_):
    """移动0.2m(<0.30m 阈值): ADD -> CONFIRM,库内位姿保持旧值。"""
    label, place = "t3_widget", "apartment_unit"
    x, y = 540.0, 500.0
    t0 = now_us()
    r1 = sys_.consolidator.ingest(_obs(place, [_one_det(label, x, y)], x, y, 2.0, t0))
    uuid = r1[0].entity_uuid
    _patch_mobility(sys_, uuid, Mobility.SEMI_STATIC)
    new_x = x + 0.2
    sys_.consolidator.ingest(_obs(place, [_one_det(label, new_x, y)], new_x, y, 2.0, t0 + 1_000_000))
    return CaseResult(
        id="T3", description="移动0.2m(<阈值),验证小位移被吸收、位姿不更新",
        keys=["t3"], uuid_of={"t3": uuid},
        expected_events={"t3": ["add", "confirm"]},
        expected_final={"t3": {"pose": {"x": x, "y": y, "z": 0.0, "yaw": 0.0}}},
        prediction_note="预注册(方案原文):规格内行为,位姿应保持 ADD 时的旧值。")


# ---------------------------------------------------------------- T4
def t4_semi_static_vanish(sys_):
    """semi_static 移除,站点仍覆盖原位置,继续观测3次(空检测):
    ADD -> DECAY(0.70) -> DECAY(0.40) -> RETIRE(0.10<0.25,第3次触发)."""
    label, place = "t4_widget", "apartment_unit"
    x, y = 560.0, 500.0
    t0 = now_us()
    r1 = sys_.consolidator.ingest(_obs(place, [_one_det(label, x, y)], x, y, 2.0, t0))
    uuid = r1[0].entity_uuid
    _patch_mobility(sys_, uuid, Mobility.SEMI_STATIC)
    for i in range(1, 4):
        sys_.consolidator.ingest(_obs(place, [], x, y, 2.0, t0 + i * 1_000_000))
    return CaseResult(
        id="T4", description="semi_static 消失,视野仍覆盖,连续3次空检测",
        keys=["t4"], uuid_of={"t4": uuid},
        expected_events={"t4": ["add", "decay", "decay", "retire"]},
        expected_final={"t4": {"status": "retired"}},
        prediction_note=("衰减算术 1.0->0.70->0.40->0.10<0.25,第3次是单个 RETIRE 事件"
                          "(代码里 DECAY/RETIRE 是同一处产生的互斥事件,不是"
                          "'DECAY+RETIRE'两个事件)。"))


# ---------------------------------------------------------------- T5
def t5_static_vanish(sys_):
    """static 消失,只验前3次衰减斜率(-0.02/次): ADD -> DECAY x3, conf≈0.94, active."""
    label, place = "t5_widget", "apartment_unit"
    x, y = 580.0, 500.0
    t0 = now_us()
    r1 = sys_.consolidator.ingest(_obs(place, [_one_det(label, x, y)], x, y, 2.0, t0))
    uuid = r1[0].entity_uuid
    _patch_mobility(sys_, uuid, Mobility.STATIC)
    for i in range(1, 4):
        sys_.consolidator.ingest(_obs(place, [], x, y, 2.0, t0 + i * 1_000_000))
    return CaseResult(
        id="T5", description="static 消失,只验前3次衰减斜率",
        keys=["t5"], uuid_of={"t5": uuid},
        expected_events={"t5": ["add", "decay", "decay", "decay"]},
        expected_final={"t5": {"status": "active", "confidence_approx": 0.94}})


# ---------------------------------------------------------------- T6
def t6_out_of_view_vanish(sys_):
    """移除,但后续站点半径不覆盖原位置: ADD 后再无事件(负观测判定不越界)."""
    label, place = "t6_widget", "apartment_unit"
    x, y = 600.0, 500.0
    t0 = now_us()
    r1 = sys_.consolidator.ingest(_obs(place, [_one_det(label, x, y)], x, y, 2.0, t0))
    uuid = r1[0].entity_uuid
    _patch_mobility(sys_, uuid, Mobility.SEMI_STATIC)
    # 后续三次"巡逻"站点搬到别处,view_radius 明确不覆盖 (x,y)
    far_x, far_y = x + 50.0, y + 50.0
    for i in range(1, 4):
        sys_.consolidator.ingest(_obs(place, [], far_x, far_y, 2.0, t0 + i * 1_000_000))
    return CaseResult(
        id="T6", description="移除但站点半径不覆盖原位置",
        keys=["t6"], uuid_of={"t6": uuid},
        expected_events={"t6": ["add"]},
        expected_final={"t6": {"status": "active", "confidence": 1.0}})


# ---------------------------------------------------------------- T7
def t7_dynamic_detection(sys_):
    """DYNAMIC 类对象入观测: 预注册预测 -- 没有拦截分支,会被正常 ADD 并留库。"""
    label, place = "t7_person", "apartment_unit"
    x, y = 620.0, 500.0
    t0 = now_us()
    r1 = sys_.consolidator.ingest(_obs(place, [_one_det(label, x, y)], x, y, 2.0, t0))
    uuid = r1[0].entity_uuid
    _patch_mobility(sys_, uuid, Mobility.DYNAMIC)
    return CaseResult(
        id="T7", description="DYNAMIC 类对象入观测,检查是否真的落长期库",
        keys=["t7"], uuid_of={"t7": uuid},
        expected_events={"t7": ["add"]},
        expected_final={"t7": {"status": "active", "mobility": "dynamic",
                                "_check": "persisted_in_store"}},
        prediction_note=("预注册:consolidator._do_add 不检查 mobility,预测会被正常 ADD 并"
                          "留在 objects 表里,与 docs/phase5_schema.md 'DYNAMIC 只进工作记忆"
                          "不落长期库' 的注释矛盾——如果实测确认,是规格/实现偏差,需要报告。"))


# ---------------------------------------------------------------- T8
def t8_cross_place(sys_):
    """对象在 place-A,之后一条空检测事件标 place-B(半径覆盖该对象):
    预期不匹配也不衰减(候选集按 place_id 门控)。"""
    label = "t8_widget"
    home_place, wrong_place = "apartment_unit", "courtyard"
    x, y = 640.0, 500.0
    t0 = now_us()
    r1 = sys_.consolidator.ingest(_obs(home_place, [_one_det(label, x, y)], x, y, 2.0, t0))
    uuid = r1[0].entity_uuid
    _patch_mobility(sys_, uuid, Mobility.SEMI_STATIC)
    # 空检测,但打了错误的 place_id,view_center/radius 几何上确实盖住了 (x,y)
    sys_.consolidator.ingest(_obs(wrong_place, [], x, y, 2.0, t0 + 1_000_000))
    return CaseResult(
        id="T8", description="观测事件误标 place,几何上覆盖但 place 不同",
        keys=["t8"], uuid_of={"t8": uuid},
        expected_events={"t8": ["add"]},  # 误标事件不应在该实体历史里产生任何事件
        expected_final={"t8": {"status": "active", "confidence": 1.0,
                                "place_id": home_place}})


def _manual_human_correct_pose(sys_, uuid, new_pose: Pose, user_id: str, pin_seconds: int):
    """Stand-in for consolidator.human_correct() -- that method cannot
    currently correct `pose` at all (see PLAN.md §3.5):
      - patch={"pose": Pose(...)} crashes in events.append() (json.dumps
        doesn't know how to serialize a Pose dataclass instance).
      - patch={"pose": {...}} (plain dict, to dodge the above) crashes one
        line earlier in entities.upsert_object(), which reads obj.pose.x/.y/
        .z/.yaw directly for the flattened SQLite columns -- but by then
        obj.pose IS the plain dict from setattr(), so `.x` doesn't exist.
    Both call shapes are broken, so this function does manually what
    human_correct() should do: proper Pose object on the entity (so the
    flattened-column upsert works), JSON-safe dict in the event payload (so
    the event log write works). This is scaffolding to let T9 test the
    actually-interesting question (does pin protection hold), not a claim
    that this is how the real API should be used -- it currently can't be."""
    e = sys_.entities.get_object(uuid)
    e.pose = new_pose
    e.version += 1
    e.pinned_until_us = now_us() + pin_seconds * 10**6
    e.confidence = 1.0
    e.last_updated_by = user_id
    sys_.entities.upsert_object(e)
    ev = SpatialEvent(new_uuid(), e.uuid, EventType.HUMAN_CORRECT,
                       {"pose": {"x": new_pose.x, "y": new_pose.y,
                                 "z": new_pose.z, "yaw": new_pose.yaw}},
                       user_id, now_us(), e.version)
    sys_.events.append(ev)
    return ev


# ---------------------------------------------------------------- T9
def t9_human_correct_pin(sys_):
    """人工修正位姿+pin(1天),之后机器观测报旧位姿(位移0.5m,超 moved_dist):
    预注册预测 -- 正观测路径不检查 pinned_until_us,UPDATE 会把人工修正覆盖回去。
    注:human_correct() 本身无法修正 pose(两种调用都崩,见 PLAN.md §3.5),
    这里用 _manual_human_correct_pose() 手工模拟它"应该"做的事,只是为了能继续
    测下游的 pin 保护问题——human_correct() 该 bug 本身已经单独记录,不会因为
    这里绕过去了就被这份报告忽略。"""
    label, place = "t9_widget", "apartment_unit"
    x, y = 660.0, 500.0
    t0 = now_us()
    r1 = sys_.consolidator.ingest(_obs(place, [_one_det(label, x, y)], x, y, 2.0, t0))
    uuid = r1[0].entity_uuid
    _patch_mobility(sys_, uuid, Mobility.SEMI_STATIC)

    corrected_x = x + 0.5
    _manual_human_correct_pose(sys_, uuid, Pose(corrected_x, y, 0.0, 0.0),
                                user_id="fred_manual", pin_seconds=24 * 3600)

    # 机器观测:旧位姿 (x,y),与人工修正后的位姿相差 0.5m(> moved_dist 0.30m)
    sys_.consolidator.ingest(_obs(place, [_one_det(label, x, y)], x, y, 2.0, t0 + 2_000_000))
    return CaseResult(
        id="T9", description="人工修正位姿+pin后,机器观测报旧位姿",
        keys=["t9"], uuid_of={"t9": uuid},
        expected_events={"t9": ["add", "human_correct", "update"]},
        expected_final={"t9": {"pose": {"x": x, "y": y, "z": 0.0, "yaw": 0.0}}},
        prediction_note=("预注册:ingest() 里 pinned_until_us 只在负观测(DECAY)分支检查"
                          "(`if entity.pinned_until_us > obs.timestamp_us: continue`,只出现"
                          "在负观测 for 循环里),正观测匹配到后走 _do_update 完全不检查保护期。"
                          "预测这次机器观测会成功触发 UPDATE,把位姿从人工修正值(corrected_x)"
                          "覆盖回机器的旧读数(x)——如果实测确认,这是一个该报告的真实 bug:"
                          "'pin 保护'只挡负观测衰减,挡不住看见了但看错的正观测覆盖。"
                          "另外 human_correct() 本身完全无法修正 pose 字段,见 PLAN.md §3.5,"
                          "这是独立于本条预测的另一个更严重的发现。"))


# ---------------------------------------------------------------- T10 / T11
def t10_close_pair_same_batch(sys_):
    """两个同 label 对象相距1.2m(<1.5m门控),同批(同一个 ObservationEvent)观测:
    预注册预测 -- 候选集在循环前就已快照,批内不会互相匹配,应该 ADD x2。"""
    label, place = "t10_widget", "apartment_unit"
    xa, ya = 680.0, 500.0
    xb, yb = xa + 1.2, ya  # 1.2m 门控内
    t0 = now_us()
    dets = [_one_det(label, xa, ya), _one_det(label, xb, yb)]
    cx, cy = (xa + xb) / 2, ya
    events = sys_.consolidator.ingest(_obs(place, dets, cx, cy, 3.0, t0))
    uuid_a, uuid_b = events[0].entity_uuid, events[1].entity_uuid
    _patch_mobility(sys_, uuid_a, Mobility.SEMI_STATIC)
    _patch_mobility(sys_, uuid_b, Mobility.SEMI_STATIC)
    return CaseResult(
        id="T10", description="两个同label对象相距1.2m,同批观测",
        keys=["t10_a", "t10_b"], uuid_of={"t10_a": uuid_a, "t10_b": uuid_b},
        expected_events={"t10_a": ["add"], "t10_b": ["add"]},
        expected_final={"t10_a": {}, "t10_b": {}, "_n_distinct_entities": 2},
        prediction_note=("预注册:consolidator.ingest() 的 unmatched_entities 字典在"
                          "for-det 循环开始前就已固定快照,批内新 ADD 的实体不会被塞回这个"
                          "字典,所以第二个 detection 不可能撞上第一个刚建的实体。预测方案"
                          "担心的'批内链式合并'不会发生,ADD x2。"))


def t11_far_pair_control(sys_):
    """对照组:相距2.0m(>1.5m门控),同批观测,ADD x2(平凡成立)。"""
    label, place = "t11_widget", "apartment_unit"
    xa, ya = 700.0, 500.0
    xb, yb = xa + 2.0, ya
    t0 = now_us()
    dets = [_one_det(label, xa, ya), _one_det(label, xb, yb)]
    cx, cy = (xa + xb) / 2, ya
    events = sys_.consolidator.ingest(_obs(place, dets, cx, cy, 3.0, t0))
    uuid_a, uuid_b = events[0].entity_uuid, events[1].entity_uuid
    _patch_mobility(sys_, uuid_a, Mobility.SEMI_STATIC)
    _patch_mobility(sys_, uuid_b, Mobility.SEMI_STATIC)
    return CaseResult(
        id="T11", description="对照组:相距2.0m(超出门控),同批观测",
        keys=["t11_a", "t11_b"], uuid_of={"t11_a": uuid_a, "t11_b": uuid_b},
        expected_events={"t11_a": ["add"], "t11_b": ["add"]},
        expected_final={"t11_a": {}, "t11_b": {}, "_n_distinct_entities": 2})


ALL_CASES = [
    t1_repeat_no_move, t2_large_displacement, t3_small_displacement,
    t4_semi_static_vanish, t5_static_vanish, t6_out_of_view_vanish,
    t7_dynamic_detection, t8_cross_place, t9_human_correct_pin,
    t10_close_pair_same_batch, t11_far_pair_control,
]
