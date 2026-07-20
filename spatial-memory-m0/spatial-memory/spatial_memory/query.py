"""
query.py — SpatialQuery 门面:上层唯一的读取入口

四类原语,对应四种上层需求:
  结构化   objects_in_place / objects_near / place_graph   → 任务规划、导航
  语义     semantic_search                                  → 自然语言落地
  时序     entity_history / where_was                       → "上周它在哪"
  同步     changes_since                                    → 机器人增量拉取

M0 与 M2 的差别只在内部路由(SQLite → PG+pgvector+Redis),
方法签名从现在起冻结 —— 这就是引擎层可以放心依赖的契约。
"""
from __future__ import annotations

from typing import Optional

import numpy as np

from .schema import AABB, EventType, ObjectInstance, Place, Pose, SpatialEvent
from .store import EntityStore, EventLog, VectorIndex


class SpatialQuery:
    def __init__(self, entities: EntityStore, events: EventLog,
                 vindex: VectorIndex, embed_fn=None):
        self.entities = entities
        self.events = events
        self.vindex = vindex
        self.embed_fn = embed_fn   # text -> np.ndarray,与写入侧同一模型!

    # ---------------------------------------------------- 结构化
    def objects_in_place(self, place_id: str) -> list[ObjectInstance]:
        return self.entities.objects_in_place(place_id)

    def objects_near(self, pose: Pose, submap_id: str,
                     radius: float) -> list[ObjectInstance]:
        return self.entities.objects_near(pose, submap_id, radius)

    def place_graph(self) -> dict[str, list[str]]:
        """拓扑邻接表,导航规划的输入。"""
        return {p.place_id: p.connected_to for p in self.entities.all_places()}

    def find_place_by_name(self, name: str) -> Optional[Place]:
        for p in self.entities.all_places():
            if name in p.name or p.name in name:
                return p
        return None

    # ---------------------------------------------------- 语义
    def semantic_search(self, text: str, top_k: int = 5,
                        place_id: str | None = None
                        ) -> list[tuple[ObjectInstance, float]]:
        """自然语言 → 实例。可选 place 过滤(先粗筛再精排的雏形)。"""
        assert self.embed_fn is not None, "需要注入 embed_fn"
        q = self.embed_fn(text)
        hits = self.vindex.search(q, top_k=top_k * 4)  # 过采样后过滤
        out = []
        for uuid, score in hits:
            e = self.entities.get_object(uuid)
            if e is None or e.status.value != "active":
                continue
            if place_id and e.place_id != place_id:
                continue
            out.append((e, score))
            if len(out) >= top_k:
                break
        return out

    # ---------------------------------------------------- 时序
    def entity_history(self, uuid: str,
                       t_start_us: int = 0,
                       t_end_us: int = 2**62) -> list[SpatialEvent]:
        return self.events.history(uuid, t_start_us, t_end_us)

    def where_was(self, uuid: str, at_us: int) -> Optional[dict]:
        """时间旅行查询:回放事件流,重建 at_us 时刻该实体的位姿/位置。
        M0 全量回放;M3 加周级物化切片后改为 切片+增量回放。"""
        pose, place = None, None
        for ev in self.events.history(uuid, 0, at_us):
            if ev.event_type == EventType.ADD:
                pose = ev.payload.get("pose")
                place = ev.payload.get("place_id")
            elif ev.event_type == EventType.UPDATE:
                if "pose" in ev.payload:
                    pose = ev.payload["pose"]["to"]
                if "place_id" in ev.payload:
                    place = ev.payload["place_id"]["to"]
        if pose is None:
            return None
        return {"pose": pose, "place_id": place, "as_of_us": at_us}

    # ---------------------------------------------------- 同步
    def changes_since(self, submap_id: str,
                      after_version: int) -> list[SpatialEvent]:
        return self.events.changes_since(submap_id, after_version,
                                         self.entities)
