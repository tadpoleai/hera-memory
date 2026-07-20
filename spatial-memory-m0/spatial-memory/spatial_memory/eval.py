"""
eval.py — 空间实体验证器(记忆 vs 真值的对账系统)

验证的四个层次(自下而上,逐级依赖):

  L1 存在性  记忆里的实体是否真实存在?真实物体是否都被记住了?
             → Precision / Recall / F1(带几何+语义双重匹配门控)
  L2 属性    位姿准不准?标签对不对?attributes 对不对?
             → 位姿 MAE / 标签准确率 / 属性准确率(只在 TP 上计算)
  L3 归属    place_id 挂对房间了吗?
             → 归属准确率
  L4 时序    环境变化后,记忆多快、多准地反映了变化?
             → 变化检测召回 + 检测延迟(以巡逻轮数计)

外加一项面向应用的端到端指标:
  R  检索    自然语言查询能否召回正确实体? → Recall@K

真值来源的演进(与 ROADMAP 对齐):
  M0: SyntheticWorld 即真值(本文件直接消费它,零标注成本)
  M1: 真实楼层人工标注真值集(标注工具产出同样的 GroundTruth 结构)
  M3: 在线抽样审计(无全量真值,人工抽检 + 多机交叉验证)
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .query import SpatialQuery
from .schema import ObjectInstance, Pose
from .store import EntityStore
from .synthetic import SyntheticWorld


# ---------------------------------------------------------------- 真值结构

@dataclass
class GroundTruthEntity:
    label: str
    pose: Pose
    place_id: str
    attributes: dict = field(default_factory=dict)


def gt_from_world(world: SyntheticWorld) -> list[GroundTruthEntity]:
    """M0 真值适配器。M1 换成'从标注文件加载',本函数签名即标注格式规范。"""
    return [GroundTruthEntity(o.label, o.pose, o.place_id, dict(o.attributes))
            for o in world.objects]


# ---------------------------------------------------------------- 匹配

@dataclass
class EvalConfig:
    match_max_dist: float = 0.75   # 真值匹配门控:比整合器更严(0.75m)
    require_label: bool = True     # M0 标签精确匹配;M1 可放宽为嵌入相似


def _match_gt_to_memory(gts: list[GroundTruthEntity],
                        mems: list[ObjectInstance],
                        cfg: EvalConfig) -> list[tuple[int, int, float]]:
    """贪心一对一分配(按距离升序)。返回 [(gt_idx, mem_idx, dist)]。
    M2 升级为匈牙利算法,本函数是唯一替换点。"""
    pairs = []
    for gi, gt in enumerate(gts):
        for mi, m in enumerate(mems):
            if cfg.require_label and gt.label != m.class_label \
                    and gt.label not in m.aliases:
                continue
            d = gt.pose.distance_to(m.pose)
            if d <= cfg.match_max_dist:
                pairs.append((d, gi, mi))
    pairs.sort()
    used_gt, used_mem, out = set(), set(), []
    for d, gi, mi in pairs:
        if gi in used_gt or mi in used_mem:
            continue
        used_gt.add(gi); used_mem.add(mi)
        out.append((gi, mi, d))
    return out


# ---------------------------------------------------------------- 报告结构

@dataclass
class EntityReport:
    n_gt: int; n_mem: int
    tp: int; fp: int; fn: int
    precision: float; recall: float; f1: float
    pose_mae: float                    # L2: TP 上的平均位姿误差(米)
    attr_accuracy: float               # L2: TP 上的属性正确率
    place_accuracy: float              # L3: TP 上的房间归属正确率
    fp_labels: list = field(default_factory=list)   # 误报清单(幻觉实体)
    fn_labels: list = field(default_factory=list)   # 漏报清单(遗忘实体)

    def pretty(self) -> str:
        lines = [
            f"  真值 {self.n_gt} | 记忆 {self.n_mem} | "
            f"TP {self.tp}  FP {self.fp}  FN {self.fn}",
            f"  L1 存在性  P={self.precision:.3f}  R={self.recall:.3f}  "
            f"F1={self.f1:.3f}",
            f"  L2 属性    位姿MAE={self.pose_mae:.3f}m  "
            f"属性准确率={self.attr_accuracy:.3f}",
            f"  L3 归属    房间归属准确率={self.place_accuracy:.3f}",
        ]
        if self.fp_labels:
            lines.append(f"  ⚠ 幻觉实体(FP): {self.fp_labels}")
        if self.fn_labels:
            lines.append(f"  ⚠ 遗忘实体(FN): {self.fn_labels}")
        return "\n".join(lines)


# ---------------------------------------------------------------- 验证器

class EntityValidator:
    def __init__(self, entities: EntityStore, query: SpatialQuery,
                 cfg: EvalConfig | None = None):
        self.entities = entities
        self.query = query
        self.cfg = cfg or EvalConfig()

    # ---- L1/L2/L3: 静态快照对账 ----
    def audit_snapshot(self, gts: list[GroundTruthEntity]) -> EntityReport:
        mems = [e for p in self.entities.all_places()
                for e in self.entities.objects_in_place(p.place_id)]
        matches = _match_gt_to_memory(gts, mems, self.cfg)

        tp = len(matches)
        fp = len(mems) - tp
        fn = len(gts) - tp
        prec = tp / max(1, len(mems))
        rec = tp / max(1, len(gts))
        f1 = 2 * prec * rec / max(1e-9, prec + rec)

        pose_errs, attr_ok, attr_total, place_ok = [], 0, 0, 0
        for gi, mi, d in matches:
            gt, m = gts[gi], mems[mi]
            pose_errs.append(d)
            place_ok += int(gt.place_id == m.place_id)
            for k, v in gt.attributes.items():
                attr_total += 1
                attr_ok += int(m.attributes.get(k) == v)

        matched_mem = {mi for _, mi, _ in matches}
        matched_gt = {gi for gi, _, _ in matches}
        return EntityReport(
            n_gt=len(gts), n_mem=len(mems), tp=tp, fp=fp, fn=fn,
            precision=prec, recall=rec, f1=f1,
            pose_mae=float(np.mean(pose_errs)) if pose_errs else 0.0,
            attr_accuracy=attr_ok / max(1, attr_total),
            place_accuracy=place_ok / max(1, tp),
            fp_labels=[m.class_label for i, m in enumerate(mems)
                       if i not in matched_mem],
            fn_labels=[g.label for i, g in enumerate(gts)
                       if i not in matched_gt])

    # ---- L4: 时序验证(变化检测延迟) ----
    def audit_change(self, gts_after: list[GroundTruthEntity],
                     patrol_and_ingest, max_rounds: int = 5
                     ) -> tuple[int | None, EntityReport]:
        """世界已变化。反复执行 patrol_and_ingest(),测记忆需要几轮
        巡逻才能与新真值对账通过(F1==1 且属性全对)。
        返回 (收敛轮数 | None, 最终报告)。这是'记忆新鲜度'的直接度量。"""
        report = None
        for r in range(1, max_rounds + 1):
            patrol_and_ingest()
            report = self.audit_snapshot(gts_after)
            if report.f1 >= 0.999 and report.attr_accuracy >= 0.999:
                return r, report
        return None, report

    # ---- R: 检索验证 ----
    def audit_retrieval(self, cases: list[tuple[str, str]],
                        top_k: int = 3) -> float:
        """cases: [(自然语言查询, 期望命中的 class_label)]。返回 Recall@K。
        M1 起查询集必须来自真实用户问法采集,而非开发者自造。"""
        hit = 0
        for text, expect_label in cases:
            results = self.query.semantic_search(text, top_k=top_k)
            if any(e.class_label == expect_label or expect_label in e.aliases
                   for e, _ in results):
                hit += 1
        return hit / max(1, len(cases))
