"""
consolidator.py — 记忆整合器(写入路径的大脑)

借鉴 Mem0 的 提取→比对→{ADD/UPDATE/NOOP/DELETE} 写入闭环,
落到空间域变成五种决策:

  观测到检测 D:
    与库中实例匹配成功 且 位姿/属性一致  → CONFIRM (刷新 last_seen)
    与库中实例匹配成功 但 位姿/属性变化  → UPDATE
    无法匹配任何实例                     → ADD (新实例)
  视锥内"应见而未见"的实例 E:
    E 的置信度扣减                        → DECAY
    连续扣减跌破阈值                      → RETIRE

M0 的匹配算法刻意简单(几何近邻 + 标签/嵌入相似,贪心分配),
但接口 (match / decide / apply) 已经切好 —— M2 在这里替换为
匈牙利分配 + 多假设跟踪,上层无感知。

这是整个系统最值得持续自研迭代的模块。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .schema import (Detection, EntityStatus, EventType, Mobility,
                     ObjectInstance, ObservationEvent, SpatialEvent,
                     new_uuid, now_us)
from .store import EntityStore, EventLog, VectorIndex


@dataclass
class ConsolidatorConfig:
    # ---- 匹配双阈值 ----
    match_max_dist: float = 1.5        # 超过此距离不认为是同一实例(米)
    match_min_sim: float = 0.55        # 嵌入余弦相似度下限
    # ---- 变化判定 ----
    moved_dist: float = 0.30           # 位移超过此值判定 UPDATE(米)
    # ---- 负观测衰减 ----
    decay_step: dict = None            # 每次"应见未见"扣减量,按 mobility 区分
    retire_threshold: float = 0.25     # 置信度跌破即 RETIRE

    def __post_init__(self):
        if self.decay_step is None:
            self.decay_step = {
                Mobility.STATIC: 0.02,       # 墙不见了大概率是感知问题
                Mobility.SEMI_STATIC: 0.30,  # 椅子不见了三次就该信了
                Mobility.DYNAMIC: 1.0,       # 动态物即刻退场
            }


class Consolidator:
    def __init__(self, entities: EntityStore, events: EventLog,
                 vindex: VectorIndex,
                 config: ConsolidatorConfig | None = None):
        self.entities = entities
        self.events = events
        self.vindex = vindex
        self.cfg = config or ConsolidatorConfig()

    # ------------------------------------------------------------ 主入口
    def ingest(self, obs: ObservationEvent) -> list[SpatialEvent]:
        """处理一条机器人观测事件,返回产生的所有空间事件(供日志/调试)。"""
        produced: list[SpatialEvent] = []

        # 候选集:视野范围内的活跃实例。
        # M0 可见性模型 = "同一 place 内、半径 r 以内"(不能隔墙看/隔墙衰减)。
        # M2 升级为真实视锥 + mesh 遮挡剔除。
        candidates = [
            e for e in self.entities.objects_near(
                obs.view_center, obs.submap_id, obs.view_radius)
            if e.place_id == obs.place_id
        ]
        unmatched_entities = {e.uuid: e for e in candidates}

        # ---- 正观测:逐个检测做匹配与决策 ----
        for det in obs.detections:
            match = self._match(det, list(unmatched_entities.values()))
            if match is None:
                produced.append(self._do_add(det, obs))
            else:
                unmatched_entities.pop(match.uuid)
                if self._changed(det, match):
                    produced.append(self._do_update(det, match, obs))
                else:
                    produced.append(self._do_confirm(match, obs))

        # ---- 负观测:视野内应见未见 → 衰减 ----
        for entity in unmatched_entities.values():
            if entity.pinned_until_us > obs.timestamp_us:
                continue  # 人工修正保护期内不衰减
            ev = self._do_decay(entity, obs)
            produced.append(ev)
            if ev.event_type == EventType.RETIRE:
                pass  # RETIRE 已在 _do_decay 内落库

        return produced

    # ------------------------------------------------------------ 人工通道
    def human_correct(self, entity_uuid: str, patch: dict, user_id: str,
                      pin_seconds: int = 7 * 24 * 3600) -> SpatialEvent:
        """人工修正走同一事件通道,但带 pin 保护期,机器观测不可覆盖。
        patch 示例: {"class_label": "fire_hydrant", "aliases": ["消防栓"]}
        """
        e = self.entities.get_object(entity_uuid)
        assert e is not None, f"entity {entity_uuid} not found"
        for k, v in patch.items():
            setattr(e, k, v)
        e.version += 1
        e.pinned_until_us = now_us() + pin_seconds * 10**6
        e.confidence = 1.0
        e.last_updated_by = user_id
        self.entities.upsert_object(e)
        ev = SpatialEvent(new_uuid(), e.uuid, EventType.HUMAN_CORRECT,
                          patch, user_id, now_us(), e.version)
        self.events.append(ev)
        return ev

    # ------------------------------------------------------------ 匹配
    def _match(self, det: Detection,
               candidates: list[ObjectInstance]) -> ObjectInstance | None:
        """M0: 几何门控 + 相似度贪心。M2 → 匈牙利分配 + 多假设。"""
        best, best_score = None, -1.0
        for e in candidates:
            dist = det.pose.distance_to(e.pose)
            if dist > self.cfg.match_max_dist:
                continue
            sim = self._similarity(det, e)
            if sim < self.cfg.match_min_sim:
                continue
            score = sim - 0.1 * dist          # 距离作轻惩罚
            if score > best_score:
                best, best_score = e, score
        return best

    def _similarity(self, det: Detection, e: ObjectInstance) -> float:
        if det.embedding is not None and e.uuid in getattr(self.vindex, "_vecs", {}):
            v1 = det.embedding / (np.linalg.norm(det.embedding) + 1e-9)
            v2 = self.vindex._vecs[e.uuid]
            return float(v1 @ v2)
        # 嵌入缺失时退化为标签精确匹配
        return 1.0 if det.class_label == e.class_label else 0.0

    def _changed(self, det: Detection, e: ObjectInstance) -> bool:
        if det.pose.distance_to(e.pose) > self.cfg.moved_dist:
            return True
        for k, v in det.attributes.items():
            if e.attributes.get(k) != v:
                return True   # 如 door state: open → closed
        return False

    # ------------------------------------------------------------ 五种写入
    def _do_add(self, det: Detection, obs: ObservationEvent) -> SpatialEvent:
        e = ObjectInstance(
            uuid=new_uuid(), class_label=det.class_label, pose=det.pose,
            submap_id=obs.submap_id, place_id=obs.place_id,
            attributes=dict(det.attributes),
            confidence=min(1.0, det.score),
            last_updated_by=obs.robot_id,
            first_seen_us=obs.timestamp_us, last_seen_us=obs.timestamp_us)
        self.entities.upsert_object(e)
        if det.embedding is not None:
            self.vindex.upsert(e.uuid, det.embedding)
        ev = SpatialEvent(new_uuid(), e.uuid, EventType.ADD,
                          {"class_label": e.class_label,
                           "pose": vars(det.pose), "place_id": obs.place_id},
                          obs.robot_id, obs.timestamp_us, e.version)
        self.events.append(ev)
        return ev

    def _do_update(self, det: Detection, e: ObjectInstance,
                   obs: ObservationEvent) -> SpatialEvent:
        diff = {}
        if det.pose.distance_to(e.pose) > self.cfg.moved_dist:
            diff["pose"] = {"from": vars(e.pose), "to": vars(det.pose)}
            e.pose = det.pose
        for k, v in det.attributes.items():
            if e.attributes.get(k) != v:
                diff.setdefault("attributes", {})[k] = \
                    {"from": e.attributes.get(k), "to": v}
                e.attributes[k] = v
        if obs.place_id != e.place_id:
            diff["place_id"] = {"from": e.place_id, "to": obs.place_id}
            e.place_id = obs.place_id
            e.submap_id = obs.submap_id
        e.version += 1
        e.confidence = min(1.0, e.confidence + 0.2)
        e.last_seen_us = obs.timestamp_us
        e.observation_count += 1
        e.last_updated_by = obs.robot_id
        self.entities.upsert_object(e)
        ev = SpatialEvent(new_uuid(), e.uuid, EventType.UPDATE, diff,
                          obs.robot_id, obs.timestamp_us, e.version)
        self.events.append(ev)
        return ev

    def _do_confirm(self, e: ObjectInstance,
                    obs: ObservationEvent) -> SpatialEvent:
        e.confidence = min(1.0, e.confidence + 0.1)
        e.last_seen_us = obs.timestamp_us
        e.observation_count += 1
        # CONFIRM 不递增 version:它不改变任何"事实",只强化记忆
        self.entities.upsert_object(e)
        ev = SpatialEvent(new_uuid(), e.uuid, EventType.CONFIRM, {},
                          obs.robot_id, obs.timestamp_us, e.version)
        self.events.append(ev)
        return ev

    def _do_decay(self, e: ObjectInstance,
                  obs: ObservationEvent) -> SpatialEvent:
        step = self.cfg.decay_step[e.mobility]
        e.confidence = max(0.0, e.confidence - step)
        if e.confidence < self.cfg.retire_threshold:
            e.status = EntityStatus.RETIRED
            e.version += 1
            self.entities.upsert_object(e)
            self.vindex.remove(e.uuid)
            ev = SpatialEvent(new_uuid(), e.uuid, EventType.RETIRE,
                              {"final_confidence": e.confidence},
                              obs.robot_id, obs.timestamp_us, e.version)
        else:
            self.entities.upsert_object(e)
            ev = SpatialEvent(new_uuid(), e.uuid, EventType.DECAY,
                              {"confidence": round(e.confidence, 3)},
                              obs.robot_id, obs.timestamp_us, e.version)
        self.events.append(ev)
        return ev
