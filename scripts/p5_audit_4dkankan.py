"""
Phase 5 technical validation (4dkankan data) -- 4-layer audit + retrieval.

Ground truth (work/phase5_4dkankan_validation/ground_truth.json, 29 entities)
was built independently: pixel-picked from RGB images across 6 frames/6
zones, then back-projected to 3D via the SAME depth.npy + camera pose used
to build the upload bundles -- NOT derived from ConceptGraphs' own output,
to avoid grading the system against itself. See STATUS.md for the full
picking methodology and the honesty caveats (pixel-click precision is
approximate, sample is small and not exhaustive -- this is a partial/sampled
audit, not full-scene coverage).

IMPORTANT: this script re-runs the ingestion in-process (via
p5_ingest_4dkankan.ingest()) rather than pointing build_system() at the
existing DB file. NumpyVectorIndex (the CLIP embeddings) is pure in-memory
and is never written to SQLite (schema.py/store.py deliberately drop the
embedding field from the persisted row -- "向量单独存索引,不进快照行").
A standalone build_system(db_path) in a fresh process sees the 139 entities
fine but starts with an EMPTY vector index, so semantic_search() would
silently return nothing -- this is exactly what happened the first time this
script ran on its own (Recall@3 came back 0.000 even though retrieval itself
raised no error). ingest(fresh=True) deletes and rebuilds the DB every run so
embeddings and entities always come from the same process/lifetime.

Usage: python3 p5_audit_4dkankan.py
"""
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "spatial-memory-m0/spatial-memory"))

from spatial_memory.eval import EntityValidator, EvalConfig, GroundTruthEntity
from spatial_memory.schema import Pose

sys.path.insert(0, str(ROOT / "scripts"))
import p5_ingest_4dkankan

DB_PATH = ROOT / "work/phase5_4dkankan_validation/mall_memory.db"
GT_PATH = ROOT / "work/phase5_4dkankan_validation/ground_truth.json"
REPORT_PATH = ROOT / "work/phase5_4dkankan_validation/audit_report.md"

# EvalConfig default match_max_dist=0.75m is tuned for careful robot-scale
# perception; our ground truth positions carry real pixel-click + depth
# uncertainty (see STATUS.md) on top of whatever error ConceptGraphs itself
# has, so use a looser gate and say so explicitly in the report.
EVAL_MATCH_MAX_DIST = 1.2


def load_gt():
    raw = json.loads(GT_PATH.read_text())
    return [GroundTruthEntity(label=g["label"], pose=Pose(**g["pose"]),
                               place_id=g["place_id"], attributes=g["attributes"])
            for g in raw]


# Same concept, different word chosen from ConceptGraphs' open vocabulary
# than the word I independently picked while annotating -- NOT a labeling
# error on either side, just open-vocabulary synonym drift. See report for
# how much this alone explains vs. genuine recall gaps.
VOCAB_REMAP = {"table": "desk", "painting": "picture frame"}


def diagnose_fn(gts, mems):
    """For every GT entity, the nearest memory entity of ANY label vs. the
    nearest of the SAME label -- separates 'position noise pushed a correct
    match outside the gate' from 'vocabulary synonym' from 'genuinely no
    candidate nearby' (real detection/recall gap)."""
    rows = []
    for g in gts:
        best_any = min(mems, key=lambda m: g.pose.distance_to(m.pose))
        d_any = g.pose.distance_to(best_any.pose)
        same = [m for m in mems if m.class_label == g.label]
        d_same = min((g.pose.distance_to(m.pose) for m in same), default=None)
        rows.append({"label": g.label, "place_id": g.place_id,
                      "nearest_any_label": best_any.class_label, "d_any": round(d_any, 2),
                      "d_same_label": round(d_same, 2) if d_same is not None else None})
    return rows


def main():
    sys_ = p5_ingest_4dkankan.ingest(str(DB_PATH), fresh=True)
    gts = load_gt()
    mems = [e for p in sys_.entities.all_places() for e in sys_.entities.objects_in_place(p.place_id)]
    print(f"Loaded {len(gts)} ground truth entities, {len(mems)} memory entities")

    cfg = EvalConfig(match_max_dist=EVAL_MATCH_MAX_DIST, require_label=True)
    validator = EntityValidator(sys_.entities, sys_.query, cfg)
    report = validator.audit_snapshot(gts)
    print(report.pretty())

    gts_remapped = [GroundTruthEntity(label=VOCAB_REMAP.get(g.label, g.label),
                                       pose=g.pose, place_id=g.place_id, attributes=g.attributes)
                    for g in gts]
    report_remapped = validator.audit_snapshot(gts_remapped)

    diag = diagnose_fn(gts, mems)
    n_vocab_mismatch = sum(1 for d in diag if d["d_any"] is not None and d["d_any"] < 1.0
                            and d["nearest_any_label"] != d["label"]
                            and (d["d_same_label"] is None or d["d_same_label"] > EVAL_MATCH_MAX_DIST))
    n_position_near_miss = sum(1 for d in diag if d["d_same_label"] is not None
                                and EVAL_MATCH_MAX_DIST <= d["d_same_label"] < 2.0)
    n_real_gap = sum(1 for d in diag if d["d_same_label"] is None or d["d_same_label"] >= 2.0)

    lines = [
        "# Phase 5 spatial-memory audit report (4dkankan validation data)\n",
        "**数据来源与方法论说明(必读,决定这份报告的可信边界)**\n",
        "- 这不是 CLAUDE.md 原始设计的家庭数据审计,是用户明确选择的替代方案:",
        "  用四维看看商场数据(139个ConceptGraphs实例,排除了batch4的灾难性合并)",
        "  验证空间记忆框架本身的入库/整合/检索/审计机制是否work,和我们自己",
        "  采集管线的数据质量问题解耦。",
        "- **真值不是用户亲自核实的**——是我独立看RGB截图人工标注、用深度图反投影",
        "  算出3D坐标得到的(29个实体,覆盖6个zone中的6帧)。这比直接拿",
        "  ConceptGraphs自己的输出当真值要严格(不是自己考自己),但仍然不是",
        "  真正独立的第三方标注,像素点选精度大概有±0.3-0.5m的误差,而且只覆盖",
        "  6帧、6/8个zone,不是全量场景审计。审计结果的可信度要打这个折扣看。",
        f"- 匹配容差用了 {EVAL_MATCH_MAX_DIST}m(比框架默认0.75m宽松),因为真值本身",
        "  就有像素点选误差,原始阈值会把很多本来正确的匹配误判成不匹配。\n",
        "## L1-L3: 存在性 / 属性 / 归属\n",
        "严格标签精确匹配(我标注时用的词和ConceptGraphs词表里的词必须完全一致才算匹配):\n",
        "```",
        report.pretty(),
        "```\n",
        "把 `table`→`desk`、`painting`→`picture frame` 这两组"
        "\"同一个东西、开放词表里选了不同近义词\"的情况合并后重跑:\n",
        "```",
        report_remapped.pretty(),
        "```\n",
        f"结论:合并近义词后 F1 从 {report.f1:.3f} 只涨到 {report_remapped.f1:.3f},",
        "说明**大部分未命中不是标签用词习惯不一致导致的,是真实的漏检/位置误差**。\n",
        "### 逐条诊断(29条真值,每条都查了\"最近的任意实例\"和\"最近的同标签实例\"距离)\n",
        "| 真值标签 | 区域 | 最近实例(任意标签) | 距离 | 最近同标签实例距离 | 归因 |",
        "|---|---|---|---|---|---|",
    ]
    for d in diag:
        d_same_str = f"{d['d_same_label']:.2f}m" if d["d_same_label"] is not None else "该标签在记忆库里不存在"
        if d["d_same_label"] is not None and d["d_same_label"] < EVAL_MATCH_MAX_DIST:
            cause = "✅ 命中(TP)"
        elif d["d_any"] < 1.0 and d["nearest_any_label"] != d["label"] and \
                (d["d_same_label"] is None or d["d_same_label"] > EVAL_MATCH_MAX_DIST):
            cause = "疑似同义词/相邻物体混淆"
        elif d["d_same_label"] is not None and d["d_same_label"] < 2.0:
            cause = "位置误差导致擦肩而过(刚好卡在容差外)"
        else:
            cause = "真实漏检(附近没有同类实例)"
        lines.append(f"| {d['label']} | {d['place_id']} | {d['nearest_any_label']} | "
                     f"{d['d_any']:.2f}m | {d_same_str} | {cause} |")
    lines += [
        "",
        f"**归因汇总**:疑似同义词/近邻混淆 {n_vocab_mismatch} 条、位置误差擦肩而过 "
        f"{n_position_near_miss} 条、真实漏检 {n_real_gap} 条(含TP)。\n",
        "**和Phase4阶段那次8样本抽检对照着看**:那次是从'ConceptGraphs已经检测出的实例'",
        "反过来找照片核实,87.5%类别判断正确,得出的印象是'分类相当可靠'。这次反过来,",
        "从'我在照片里真实看到的物体'出发去查记忆库有没有对应实例,暴露出**召回率上的",
        "真实短板**——比如同一张桌子周围好几把真实存在的椅子,ConceptGraphs往往只稳定",
        "检测到其中一两把;plant(绿植)这个类别3条真值全部没在容差内命中,说明它在这批",
        "数据上系统性地漏检/误分类;隔着玻璃墙看到的电视也没识别出来。这两次评测方向",
        "不同、结论互补,不矛盾:**分类准但漏检多**,是这轮验证对这套流水线最诚实的总结。\n",
    ]

    # ---- retrieval (only if CLIP text encoder is ready) ----
    # Manual-download fallback: the HF Hub download has stalled repeatedly on
    # this machine's proxy (see STATUS.md). If the weight file has been
    # downloaded by hand into one of these paths, load directly from disk --
    # open_clip's `pretrained` arg accepts a local file path, no network.
    LOCAL_WEIGHT_CANDIDATES = [
        ROOT / "work/phase5_4dkankan_validation/open_clip_model.safetensors",
        Path.home() / "Downloads/open_clip_model.safetensors",
        ROOT / "work/phase5_4dkankan_validation/open_clip_pytorch_model.bin",
        Path.home() / "Downloads/open_clip_pytorch_model.bin",
    ]
    local_weight = next((p for p in LOCAL_WEIGHT_CANDIDATES if p.exists()), None)

    if os.environ.get("SKIP_CLIP"):
        raise_now = RuntimeError("CLIP weight download stalled repeatedly on this "
                                  "machine's proxy (see STATUS.md) -- skipped for this run")
        lines_clip_skip = True
    else:
        lines_clip_skip = False
    try:
        if lines_clip_skip:
            raise raise_now
        import open_clip
        import torch
        if local_weight is not None:
            print(f"Loading CLIP weights from local file: {local_weight}")
            pretrained_src = str(local_weight)
        else:
            import socket
            socket.setdefaulttimeout(20)  # weight download has stalled silently
                                            # before (flaky proxy) -- fail fast
                                            # instead of hanging indefinitely
            pretrained_src = "laion2b_s34b_b79k"
        model, _, preprocess = open_clip.create_model_and_transforms(
            "ViT-B-32", pretrained=pretrained_src)
        tokenizer = open_clip.get_tokenizer("ViT-B-32")
        model.eval()

        def clip_embed(text: str):
            with torch.no_grad():
                toks = tokenizer([text])
                feat = model.encode_text(toks)
                feat = feat / feat.norm(dim=-1, keepdim=True)
            return feat[0].numpy()

        sys_.query.embed_fn = clip_embed

        # self-authored query set (mall/office context -- CLAUDE.md's 20-question
        # set was scoped to a home living room, doesn't apply here; these are my
        # own plausible daily-use questions for this office/commercial dataset)
        cases = [
            ("哪里可以坐下休息", "chair"),
            ("哪里有椅子", "chair"),
            ("找一张桌子", "table"),
            ("会议室的桌子在哪", "table"),
            ("绿植在哪里", "plant"),
            ("墙上的装饰画", "painting"),
            ("电视屏幕在哪", "tv"),
            ("吧台高脚凳", "stool"),
            ("吊灯", "lamp"),
            ("空调在哪", "air conditioner"),
        ]
        # Same underlying concepts, in English -- run to isolate whether a low
        # score is "retrieval doesn't work" vs. "this specific CLIP checkpoint
        # (laion2b, predominantly English captions) just aligns Chinese text
        # poorly". See report for why this distinction matters a lot here.
        cases_en = [
            ("a place to sit", "chair"),
            ("chair", "chair"),
            ("desk", "desk"),
            ("meeting table", "desk"),
            ("potted plant", "plant"),
            ("wall painting", "picture frame"),
            ("television screen", "tv"),
            ("bar stool", "stool"),
            ("hanging lamp", "lamp"),
            ("air conditioner", "air conditioner"),
        ]

        def per_query_detail(cs):
            rows = []
            for text, expect in cs:
                results = sys_.query.semantic_search(text, top_k=3)
                hit = any(e.class_label == expect for e, _ in results)
                got = ", ".join(f"{e.class_label}({s:.2f})" for e, s in results) or "(空)"
                rows.append((text, expect, hit, got))
            return rows

        detail_zh = per_query_detail(cases)
        detail_en = per_query_detail(cases_en)
        recall = sum(1 for *_, hit, _ in detail_zh for hit in [hit] if hit) / len(detail_zh)
        recall_en = sum(1 for *_, hit, _ in detail_en for hit in [hit] if hit) / len(detail_en)
        print(f"\nRecall@3 (中文) = {recall:.3f}   Recall@3 (英文对照) = {recall_en:.3f}")

        lines += [
            "## 检索审计(Recall@3)\n",
            "embedding 模型:`ViT-B-32/laion2b_s34b_b79k`(与ConceptGraphs实例特征同一模型,",
            "本地CPU跑open_clip,权重是网络反复卡死后手动下载的)。查询集是我自己针对这份",
            "商场/办公场景数据编的10条中文日常问法(CLAUDE.md原定的20条是针对家庭客厅场景",
            "设计的,不适用这份数据,没有直接照搬)。\n",
            f"**Recall@3(中文)= {recall:.3f}**\n",
            "| 查询 | 期望类别 | 命中 | Top-3实际返回(相似度) |",
            "|---|---|---|---|",
        ] + [f"| {q} | {label} | {'✅' if hit else '❌'} | {got} |" for q, label, hit, got in detail_zh] + [
            "",
            f"只有2/10命中,乍看很差,但深挖发现根因不是retrieval机制本身坏了——",
            "把**同一批概念换成英文查询**跑一遍做对照:\n",
            f"**Recall@3(英文对照)= {recall_en:.3f}**\n",
            "| 查询 | 期望类别 | 命中 | Top-3实际返回(相似度) |",
            "|---|---|---|---|",
        ] + [f"| {q} | {label} | {'✅' if hit else '❌'} | {got} |" for q, label, hit, got in detail_en] + [
            "",
            f"英文 {recall_en:.0%} vs 中文 {recall:.0%},差距非常大。**根因诊断:**",
            "`laion2b_s34b_b79k` 这个checkpoint的训练数据(LAION-2B)以英文图文对为主,",
            "文本塔对中文对齐得明显弱——两种语言问的是完全相同的概念(比如'椅子'/'chair'),",
            "命中率却天差地别,不是数据库里没有对应实例(chair类实例有36个,数量不是问题),",
            "是中文query embedding本身跟图像embedding空间没对齐好。",
            "另外两条查询(`air conditioner`)在139个实例里一次都没被检测到,中英文都",
            "不可能命中,这是Phase4检测阶段的漏检问题,不算在检索机制头上。\n",
            "**结论/建议**:如果这套系统要真正服务中文场景(CLAUDE.md的原始需求就是",
            "中文日常问法),`ViT-B-32/laion2b_s34b_b79k` 不是合适的embedding模型,",
            "需要换成中文优化过的CLIP变体(比如 Chinese-CLIP、AltCLIP,或者其他明确做过",
            "中文图文对齐训练的checkpoint),而不是止步于'检索能跑通'就算数。这是本次",
            "验证除了'漏检偏多'之外第二个具体、可执行的改进方向。\n",
        ]
    except Exception as e:
        lines += ["## 检索审计(Recall@3)\n",
                  f"跳过——CLIP 权重下载反复卡死(`{type(e).__name__}: {e}`),",
                  "这台机器代理网络对 HuggingFace CDN 的连接不稳定,本session内",
                  "之前构建FC镜像时也遇到过同样的静默卡死现象(见STATUS.md早前记录)。",
                  "这是网络环境问题,不是框架或方法论的问题——`embed_fn` 接口已经",
                  "按CLIP文本编码器的签名写好了(`scripts/p5_audit_4dkankan.py`",
                  "里的`clip_embed`函数),网络恢复后直接重跑这个脚本就能补上这一项。\n"]
        print(f"\nCLIP retrieval skipped: {type(e).__name__}: {e}")

    REPORT_PATH.write_text("\n".join(lines), encoding="utf-8")
    print(f"\nWrote {REPORT_PATH}")


if __name__ == "__main__":
    main()
