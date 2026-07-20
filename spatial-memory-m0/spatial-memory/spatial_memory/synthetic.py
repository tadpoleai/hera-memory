"""
synthetic.py — 合成世界与模拟机器人(M0 的"传感器")

作用:让整条 观测→整合→查询 链路在没有任何硬件和 SLAM 的情况下
先跑起来。真实感知接入(M1)时,只需让真实管线产出同样的
ObservationEvent,本文件即可整体退役 —— 这就是它存在的意义。

toy_embed: 字符 trigram 哈希嵌入。仅为演示嵌入接口而存在,
中英文标签间有朴素的相似性(共享字符片段)。M1 替换为
CLIP/SigLIP 文本-图像双塔,embed_fn 签名不变。
"""
from __future__ import annotations

import hashlib
from dataclasses import dataclass, field

import numpy as np

from .schema import (AABB, Detection, Mobility, ObservationEvent, Place,
                     Pose, Submap, now_us)

EMBED_DIM = 64


def toy_embed(text: str) -> np.ndarray:
    """确定性 trigram 哈希嵌入。同类词汇共享片段 → 相似向量。"""
    text = f"##{text.lower()}##"
    v = np.zeros(EMBED_DIM, dtype=np.float32)
    for i in range(len(text) - 2):
        tri = text[i:i + 3]
        h = int(hashlib.md5(tri.encode()).hexdigest(), 16)
        v[h % EMBED_DIM] += 1.0
        v[(h // EMBED_DIM) % EMBED_DIM] += 0.5
    n = np.linalg.norm(v)
    return v / n if n > 0 else v


# ---------------------------------------------------------------- 合成世界

@dataclass
class WorldObject:
    label: str
    pose: Pose
    place_id: str
    attributes: dict = field(default_factory=dict)
    mobility: Mobility = Mobility.SEMI_STATIC


class SyntheticWorld:
    """一层楼:走廊连接会议室与办公室,内置若干物体。
    世界状态可被外部修改(移动/移除物体)来模拟环境变化。"""

    SUBMAP_ID = "submap_f3_a"

    def __init__(self):
        self.submap = Submap(
            submap_id=self.SUBMAP_ID,
            anchor_pose_world=Pose(0, 0, 0, 0),
            bounds=AABB(0, 0, 0, 30, 12, 3))
        self.places = [
            Place("meeting_a", "3F 会议室 A", "F3",
                  AABB(0, 0, 0, 10, 12, 3), ["corridor"]),
            Place("corridor", "3F 走廊", "F3",
                  AABB(10, 4, 0, 20, 8, 3), ["meeting_a", "office_b"]),
            Place("office_b", "3F 办公室 B", "F3",
                  AABB(20, 0, 0, 30, 12, 3), ["corridor"]),
        ]
        self.objects: list[WorldObject] = [
            WorldObject("conference_table", Pose(5, 6, 0.4), "meeting_a"),
            WorldObject("office_chair", Pose(4, 5, 0.3), "meeting_a"),
            WorldObject("office_chair", Pose(6, 7, 0.3), "meeting_a"),
            WorldObject("tv_screen", Pose(0.5, 6, 1.5), "meeting_a"),
            WorldObject("coffee_cup", Pose(5.2, 6.1, 0.8), "meeting_a"),
            WorldObject("fire_extinguisher", Pose(15, 7.8, 0.5), "corridor",
                        {"color": "red"}, Mobility.STATIC),
            WorldObject("door", Pose(10, 6, 1.0), "corridor",
                        {"state": "open"}, Mobility.STATIC),
            WorldObject("standing_desk", Pose(25, 6, 0.7), "office_b"),
            WorldObject("whiteboard", Pose(29.5, 6, 1.4), "office_b"),
            WorldObject("potted_plant", Pose(21, 2, 0.4), "office_b"),
        ]

    # ---- 世界演化算子(演示环境变化用) ----
    def move_object(self, label: str, new_pose: Pose, new_place: str):
        for o in self.objects:
            if o.label == label:
                o.pose, o.place_id = new_pose, new_place
                return
        raise KeyError(label)

    def remove_object(self, label: str):
        self.objects = [o for o in self.objects if o.label != label]

    def set_attribute(self, label: str, key: str, value: str):
        for o in self.objects:
            if o.label == label:
                o.attributes[key] = value
                return
        raise KeyError(label)


class SimulatedRobot:
    """模拟巡逻:依次访问每个房间中心,观测半径内的世界物体,
    产出与真实边缘端完全同构的 ObservationEvent。"""

    def __init__(self, world: SyntheticWorld, robot_id: str = "robot_01",
                 view_radius: float = 8.0, noise_std: float = 0.05,
                 seed: int = 42):
        self.world = world
        self.robot_id = robot_id
        self.view_radius = view_radius
        self.rng = np.random.default_rng(seed)
        self.noise_std = noise_std

    def patrol(self, timestamp_us: int | None = None) -> list[ObservationEvent]:
        ts = timestamp_us or now_us()
        events = []
        for place in self.world.places:
            b = place.bounds
            center = Pose((b.min_x + b.max_x) / 2, (b.min_y + b.max_y) / 2, 1.0)
            dets = []
            for o in self.world.objects:
                if o.place_id != place.place_id:
                    continue
                if o.pose.distance_to(center) > self.view_radius:
                    continue
                noisy = Pose(
                    o.pose.x + float(self.rng.normal(0, self.noise_std)),
                    o.pose.y + float(self.rng.normal(0, self.noise_std)),
                    o.pose.z + float(self.rng.normal(0, self.noise_std)),
                    o.pose.yaw)
                dets.append(Detection(
                    class_label=o.label, pose=noisy,
                    embedding=toy_embed(o.label),
                    attributes=dict(o.attributes), score=0.95))
            events.append(ObservationEvent(
                submap_id=self.world.SUBMAP_ID, place_id=place.place_id,
                robot_id=self.robot_id, detections=dets,
                view_center=center, view_radius=self.view_radius,
                timestamp_us=ts))
            ts += 60 * 10**6   # 每个房间间隔 1 分钟
        return events
