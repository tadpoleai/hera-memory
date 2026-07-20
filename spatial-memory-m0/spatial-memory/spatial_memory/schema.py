"""
schema.py — 空间记忆的核心数据契约(M0 版本)

设计原则:
  1. 这是全系统唯一的事实来源。存储、整合器、查询、机器人端都只依赖这里。
  2. M0 用 dataclass 起步,便于快速迭代;M1 冻结后迁移到 Protobuf
     (字段编号已在注释中预留,迁移时一一对应)。
  3. 位姿一律相对子图(submap),不存世界系 —— 回环修正只改子图锚点。

升级路径(见 ROADMAP.md):
  M0: dataclass + JSON 序列化(当前)
  M1: Protobuf 定义,codegen 出 Python/C++/TS 三端
"""
from __future__ import annotations

import time
import uuid
from dataclasses import dataclass, field, asdict
from enum import Enum
from typing import Optional

import numpy as np


def now_us() -> int:
    return int(time.time() * 1e6)


def new_uuid() -> str:
    return str(uuid.uuid4())


# ---------------------------------------------------------------- 几何基元

@dataclass
class Pose:
    """SE(3) 位姿。M0 只用平移 + yaw,四元数字段预留。"""
    x: float = 0.0          # proto field 1
    y: float = 0.0          # proto field 2
    z: float = 0.0          # proto field 3
    yaw: float = 0.0        # proto field 4 (M1 换成四元数 qx,qy,qz,qw)

    def distance_to(self, other: "Pose") -> float:
        return float(np.linalg.norm(
            [self.x - other.x, self.y - other.y, self.z - other.z]))


@dataclass
class AABB:
    """轴对齐包围盒(M0 简化)。M2 升级为 OBB(有向包围盒)。"""
    min_x: float; min_y: float; min_z: float
    max_x: float; max_y: float; max_z: float

    def contains_point(self, x: float, y: float, z: float = 0.0) -> bool:
        return (self.min_x <= x <= self.max_x
                and self.min_y <= y <= self.max_y
                and self.min_z <= z <= self.max_z)


# ---------------------------------------------------------------- 枚举

class Mobility(str, Enum):
    """决定置信度衰减速率与是否进入长期记忆。"""
    STATIC = "static"            # 墙、门框:几乎不衰减
    SEMI_STATIC = "semi_static"  # 桌椅、设备:按天衰减
    DYNAMIC = "dynamic"          # 人、推车:只进工作记忆,不落长期库


class EventType(str, Enum):
    ADD = "add"                   # 新实例入库
    UPDATE = "update"             # 位姿/属性/状态变化
    CONFIRM = "confirm"           # 观测一致,仅刷新 last_seen(= Mem0 的 NOOP)
    DECAY = "decay"               # 负观测(应见未见),置信度扣减
    RETIRE = "retire"             # 置信度跌破阈值,标记消失
    HUMAN_CORRECT = "human_correct"  # 人工修正,带 pin 保护


class EntityStatus(str, Enum):
    ACTIVE = "active"
    RETIRED = "retired"


# ---------------------------------------------------------------- 核心实体

@dataclass
class Submap:
    """物理组织单元:L0 数据、实体归属、增量同步都以它为最小粒度。"""
    submap_id: str
    anchor_pose_world: Pose            # 回环修正只改这一条
    bounds: AABB
    anchor_version: int = 0
    mesh_blob_uri: str = ""            # M2 起指向对象存储
    status: str = "ACTIVE"


@dataclass
class Place:
    """L2 地点节点:房间/走廊/功能区。拓扑导航与人机对话的锚点。"""
    place_id: str
    name: str                          # "3F 会议室 A"
    floor_id: str
    bounds: AABB                       # M0 用 AABB 代替多边形
    connected_to: list[str] = field(default_factory=list)  # 连通的 place_id


@dataclass
class ObjectInstance:
    """L1 物体实例 —— 空间记忆的基本记账单位。"""
    uuid: str
    class_label: str                   # 开放词汇主标签
    pose: Pose                         # 相对 submap_id 的位姿!
    submap_id: str
    place_id: str                      # 冗余外键,加速 "房间里有什么"
    embedding: Optional[np.ndarray] = None   # 语义特征向量
    embedding_model: str = "toy-trigram@v0"  # 换模型时必须区分版本
    aliases: list[str] = field(default_factory=list)
    attributes: dict[str, str] = field(default_factory=dict)  # {"color":"red","state":"door_open"}
    mobility: Mobility = Mobility.SEMI_STATIC
    confidence: float = 1.0
    status: EntityStatus = EntityStatus.ACTIVE
    first_seen_us: int = field(default_factory=now_us)
    last_seen_us: int = field(default_factory=now_us)
    observation_count: int = 1
    last_updated_by: str = ""          # provenance
    version: int = 1                   # 乐观锁
    pinned_until_us: int = 0           # 人工修正保护期

    def to_row(self) -> dict:
        d = asdict(self)
        d["embedding"] = None  # 向量单独存索引,不进快照行
        return d


# ---------------------------------------------------------------- 事件与观测

@dataclass
class SpatialEvent:
    """追加式事件日志的记录单元。实体表 = 事件流的物化快照。"""
    event_id: str
    entity_uuid: str
    event_type: EventType
    payload: dict                      # diff 内容,如 {"pose": {...}, "confidence": 0.8}
    source: str                        # robot_id / human_user_id / pipeline_id
    timestamp_us: int
    entity_version: int                # 该事件产生后实体的版本号


@dataclass
class Detection:
    """机器人端单个检测结果(已在边缘侧完成蒸馏)。"""
    class_label: str
    pose: Pose                         # 相对 submap
    embedding: Optional[np.ndarray] = None
    attributes: dict[str, str] = field(default_factory=dict)
    score: float = 1.0


@dataclass
class ObservationEvent:
    """机器人上传的最小单元(< 5KB)。带视锥以支持负观测。"""
    submap_id: str
    place_id: str
    robot_id: str
    detections: list[Detection]
    view_center: Pose                  # M0 简化: 用观测中心+半径近似视锥
    view_radius: float
    timestamp_us: int = field(default_factory=now_us)
