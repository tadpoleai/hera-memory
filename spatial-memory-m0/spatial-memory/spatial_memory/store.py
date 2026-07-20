"""
store.py — 存储抽象层:接口冻结,实现可换

这是整个框架"可持续探索"的关键:上层(整合器/查询)只依赖四个抽象接口,
底层实现按里程碑替换,替换时上层代码零改动。

  接口            M0 实现(本文件)          M2+ 生产实现
  -------------  ------------------------  --------------------------------
  EntityStore    SQLite 表                  PostgreSQL (+ PostGIS 空间索引)
  EventLog       SQLite 追加表              PG 分区表 → Kafka + Parquet 冷备
  VectorIndex    numpy 暴力检索             pgvector HNSW / Faiss / Milvus
  BlobStore      本地文件目录               MinIO / S3

M0 刻意选 SQLite 而非纯内存:让"重启后记忆还在"从第一天就成立,
这会倒逼所有序列化问题提早暴露。
"""
from __future__ import annotations

import json
import sqlite3
from abc import ABC, abstractmethod
from dataclasses import asdict
from pathlib import Path
from typing import Optional

import numpy as np

from .schema import (AABB, EntityStatus, EventType, Mobility, ObjectInstance,
                     Place, Pose, SpatialEvent, Submap, new_uuid)


# ================================================================ 抽象接口

class EntityStore(ABC):
    """L1-L3 实体的快照存储(永远反映当前最新状态)。"""

    @abstractmethod
    def upsert_object(self, obj: ObjectInstance) -> None: ...
    @abstractmethod
    def get_object(self, uuid: str) -> Optional[ObjectInstance]: ...
    @abstractmethod
    def objects_in_place(self, place_id: str,
                         include_retired: bool = False) -> list[ObjectInstance]: ...
    @abstractmethod
    def objects_in_region(self, bounds: AABB, submap_id: str) -> list[ObjectInstance]: ...
    @abstractmethod
    def objects_near(self, pose: Pose, submap_id: str,
                     radius: float) -> list[ObjectInstance]: ...
    @abstractmethod
    def upsert_place(self, place: Place) -> None: ...
    @abstractmethod
    def get_place(self, place_id: str) -> Optional[Place]: ...
    @abstractmethod
    def all_places(self) -> list[Place]: ...
    @abstractmethod
    def upsert_submap(self, submap: Submap) -> None: ...
    @abstractmethod
    def get_submap(self, submap_id: str) -> Optional[Submap]: ...


class EventLog(ABC):
    """追加式事件日志。时序查询、审计、回滚、增量同步都建立在它之上。"""

    @abstractmethod
    def append(self, event: SpatialEvent) -> None: ...
    @abstractmethod
    def history(self, entity_uuid: str,
                t_start_us: int = 0, t_end_us: int = 2**62) -> list[SpatialEvent]: ...
    @abstractmethod
    def changes_since(self, submap_id: str, after_version: int,
                      entity_store: "EntityStore") -> list[SpatialEvent]: ...


class VectorIndex(ABC):
    """语义嵌入的 ANN 检索。key = entity uuid。"""

    @abstractmethod
    def upsert(self, uuid: str, embedding: np.ndarray) -> None: ...
    @abstractmethod
    def remove(self, uuid: str) -> None: ...
    @abstractmethod
    def search(self, query: np.ndarray, top_k: int = 5) -> list[tuple[str, float]]: ...


class BlobStore(ABC):
    """L0 大对象(mesh/TSDF/关键帧)。M0 仅占位,M2 接 MinIO。"""

    @abstractmethod
    def put(self, key: str, data: bytes) -> str: ...
    @abstractmethod
    def get(self, key: str) -> bytes: ...


# ================================================================ M0 实现

_SCHEMA_SQL = """
CREATE TABLE IF NOT EXISTS objects (
    uuid TEXT PRIMARY KEY,
    class_label TEXT NOT NULL,
    place_id TEXT NOT NULL,
    submap_id TEXT NOT NULL,
    status TEXT NOT NULL,
    confidence REAL NOT NULL,
    x REAL, y REAL, z REAL, yaw REAL,
    last_seen_us INTEGER NOT NULL,
    version INTEGER NOT NULL,
    doc TEXT NOT NULL                -- 完整 JSON,避免 M0 频繁改列
);
CREATE INDEX IF NOT EXISTS idx_obj_place ON objects(place_id, status);
CREATE INDEX IF NOT EXISTS idx_obj_submap ON objects(submap_id, status);
CREATE INDEX IF NOT EXISTS idx_obj_seen ON objects(last_seen_us);

CREATE TABLE IF NOT EXISTS places (place_id TEXT PRIMARY KEY, doc TEXT NOT NULL);
CREATE TABLE IF NOT EXISTS submaps (submap_id TEXT PRIMARY KEY, doc TEXT NOT NULL);

CREATE TABLE IF NOT EXISTS events (
    seq INTEGER PRIMARY KEY AUTOINCREMENT,   -- 全局单调,同步游标
    event_id TEXT NOT NULL,
    entity_uuid TEXT NOT NULL,
    event_type TEXT NOT NULL,
    source TEXT NOT NULL,
    timestamp_us INTEGER NOT NULL,
    entity_version INTEGER NOT NULL,
    payload TEXT NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_evt_entity ON events(entity_uuid, timestamp_us);
"""


def _obj_to_json(obj: ObjectInstance) -> str:
    d = asdict(obj)
    d["embedding"] = None
    d["mobility"] = obj.mobility.value
    d["status"] = obj.status.value
    return json.dumps(d, ensure_ascii=False)


def _obj_from_json(doc: str) -> ObjectInstance:
    d = json.loads(doc)
    d["pose"] = Pose(**d["pose"])
    d["mobility"] = Mobility(d["mobility"])
    d["status"] = EntityStatus(d["status"])
    d.pop("embedding", None)
    return ObjectInstance(embedding=None, **d)


class SqliteStore(EntityStore, EventLog):
    """M0: 实体快照 + 事件日志合并在一个 SQLite 文件里,单进程够用。"""

    def __init__(self, path: str | Path = "spatial_memory.db"):
        self.conn = sqlite3.connect(str(path))
        self.conn.executescript(_SCHEMA_SQL)
        self.conn.commit()

    # ---- EntityStore ----
    def upsert_object(self, obj: ObjectInstance) -> None:
        self.conn.execute(
            """INSERT INTO objects
               (uuid,class_label,place_id,submap_id,status,confidence,
                x,y,z,yaw,last_seen_us,version,doc)
               VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)
               ON CONFLICT(uuid) DO UPDATE SET
                 class_label=excluded.class_label, place_id=excluded.place_id,
                 submap_id=excluded.submap_id, status=excluded.status,
                 confidence=excluded.confidence, x=excluded.x, y=excluded.y,
                 z=excluded.z, yaw=excluded.yaw,
                 last_seen_us=excluded.last_seen_us,
                 version=excluded.version, doc=excluded.doc""",
            (obj.uuid, obj.class_label, obj.place_id, obj.submap_id,
             obj.status.value, obj.confidence,
             obj.pose.x, obj.pose.y, obj.pose.z, obj.pose.yaw,
             obj.last_seen_us, obj.version, _obj_to_json(obj)))
        self.conn.commit()

    def get_object(self, uuid: str) -> Optional[ObjectInstance]:
        row = self.conn.execute(
            "SELECT doc FROM objects WHERE uuid=?", (uuid,)).fetchone()
        return _obj_from_json(row[0]) if row else None

    def objects_in_place(self, place_id: str,
                         include_retired: bool = False) -> list[ObjectInstance]:
        q = "SELECT doc FROM objects WHERE place_id=?"
        if not include_retired:
            q += " AND status='active'"
        return [_obj_from_json(r[0])
                for r in self.conn.execute(q, (place_id,)).fetchall()]

    def objects_in_region(self, bounds: AABB, submap_id: str) -> list[ObjectInstance]:
        rows = self.conn.execute(
            """SELECT doc FROM objects WHERE submap_id=? AND status='active'
               AND x BETWEEN ? AND ? AND y BETWEEN ? AND ?""",
            (submap_id, bounds.min_x, bounds.max_x,
             bounds.min_y, bounds.max_y)).fetchall()
        return [_obj_from_json(r[0]) for r in rows]

    def objects_near(self, pose: Pose, submap_id: str,
                     radius: float) -> list[ObjectInstance]:
        box = AABB(pose.x - radius, pose.y - radius, -1e9,
                   pose.x + radius, pose.y + radius, 1e9)
        return [o for o in self.objects_in_region(box, submap_id)
                if o.pose.distance_to(pose) <= radius]

    def upsert_place(self, place: Place) -> None:
        self.conn.execute(
            "INSERT OR REPLACE INTO places VALUES (?,?)",
            (place.place_id, json.dumps(asdict(place), ensure_ascii=False)))
        self.conn.commit()

    def get_place(self, place_id: str) -> Optional[Place]:
        row = self.conn.execute(
            "SELECT doc FROM places WHERE place_id=?", (place_id,)).fetchone()
        if not row:
            return None
        d = json.loads(row[0]); d["bounds"] = AABB(**d["bounds"])
        return Place(**d)

    def all_places(self) -> list[Place]:
        out = []
        for (doc,) in self.conn.execute("SELECT doc FROM places").fetchall():
            d = json.loads(doc); d["bounds"] = AABB(**d["bounds"])
            out.append(Place(**d))
        return out

    def upsert_submap(self, submap: Submap) -> None:
        self.conn.execute(
            "INSERT OR REPLACE INTO submaps VALUES (?,?)",
            (submap.submap_id, json.dumps(asdict(submap), ensure_ascii=False)))
        self.conn.commit()

    def get_submap(self, submap_id: str) -> Optional[Submap]:
        row = self.conn.execute(
            "SELECT doc FROM submaps WHERE submap_id=?", (submap_id,)).fetchone()
        if not row:
            return None
        d = json.loads(row[0])
        d["anchor_pose_world"] = Pose(**d["anchor_pose_world"])
        d["bounds"] = AABB(**d["bounds"])
        return Submap(**d)

    # ---- EventLog ----
    def append(self, event: SpatialEvent) -> None:
        self.conn.execute(
            """INSERT INTO events
               (event_id,entity_uuid,event_type,source,timestamp_us,
                entity_version,payload) VALUES (?,?,?,?,?,?,?)""",
            (event.event_id, event.entity_uuid, event.event_type.value,
             event.source, event.timestamp_us, event.entity_version,
             json.dumps(event.payload, ensure_ascii=False)))
        self.conn.commit()

    def history(self, entity_uuid: str, t_start_us: int = 0,
                t_end_us: int = 2**62) -> list[SpatialEvent]:
        rows = self.conn.execute(
            """SELECT event_id,entity_uuid,event_type,source,timestamp_us,
                      entity_version,payload FROM events
               WHERE entity_uuid=? AND timestamp_us BETWEEN ? AND ?
               ORDER BY timestamp_us""",
            (entity_uuid, t_start_us, t_end_us)).fetchall()
        return [SpatialEvent(r[0], r[1], EventType(r[2]), json.loads(r[6]),
                             r[3], r[4], r[5]) for r in rows]

    def changes_since(self, submap_id: str, after_version: int,
                      entity_store: EntityStore) -> list[SpatialEvent]:
        """机器人增量同步:拉取某子图下、版本号之后的所有事件。
        M0 简化实现:按实体过滤;M2 事件表直接带 submap_id 列。"""
        rows = self.conn.execute(
            """SELECT e.event_id,e.entity_uuid,e.event_type,e.source,
                      e.timestamp_us,e.entity_version,e.payload
               FROM events e JOIN objects o ON e.entity_uuid=o.uuid
               WHERE o.submap_id=? AND e.entity_version>?
               ORDER BY e.seq""",
            (submap_id, after_version)).fetchall()
        return [SpatialEvent(r[0], r[1], EventType(r[2]), json.loads(r[6]),
                             r[3], r[4], r[5]) for r in rows]


class NumpyVectorIndex(VectorIndex):
    """M0: 暴力余弦检索。<1e4 实体时毫秒级,完全够探索期使用。
    M2 换 pgvector/Faiss 时,本类保留作为召回率对照基线。"""

    def __init__(self):
        self._vecs: dict[str, np.ndarray] = {}

    def upsert(self, uuid: str, embedding: np.ndarray) -> None:
        v = embedding.astype(np.float32)
        n = np.linalg.norm(v)
        self._vecs[uuid] = v / n if n > 0 else v

    def remove(self, uuid: str) -> None:
        self._vecs.pop(uuid, None)

    def search(self, query: np.ndarray, top_k: int = 5) -> list[tuple[str, float]]:
        if not self._vecs:
            return []
        q = query.astype(np.float32)
        n = np.linalg.norm(q)
        q = q / n if n > 0 else q
        ids = list(self._vecs.keys())
        mat = np.stack([self._vecs[i] for i in ids])
        sims = mat @ q
        order = np.argsort(-sims)[:top_k]
        return [(ids[i], float(sims[i])) for i in order]


class LocalBlobStore(BlobStore):
    """M0: 本地目录模拟对象存储,接口与 S3 对齐(put/get by key)。"""

    def __init__(self, root: str | Path = "./blobs"):
        self.root = Path(root); self.root.mkdir(parents=True, exist_ok=True)

    def put(self, key: str, data: bytes) -> str:
        p = self.root / key
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(data)
        return f"file://{p.resolve()}"

    def get(self, key: str) -> bytes:
        return (self.root / key).read_bytes()
