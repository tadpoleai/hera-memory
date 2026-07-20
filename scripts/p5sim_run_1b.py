"""
Phase5 sim validation, step 1b -- runs every case in p5sim_scenario.py
against one fresh, live SpatialMemorySystem (same process, same pitfalls as
p5sim_run_1a.py: vindex not persisted across processes doesn't matter here
since every Detection uses embedding=None, but we still keep everything in
one process for consistency).

For each test object key, pulls the ACTUAL event-type sequence from
`sys_.events.history(uuid)` (uses the idx_evt_entity index, per
docs/phase5_schema.md §2) and diffs it against the case's expected sequence.
Also checks expected_final field values against the entity's final DB state.

Usage: .venv_p5sim\\Scripts\\python.exe scripts\\p5sim_run_1b.py
"""
import json
import sys as _sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
_sys.path.insert(0, str(ROOT / "spatial-memory-m0/spatial-memory"))
_sys.path.insert(0, str(ROOT / "scripts"))

from spatial_memory import build_system
from spatial_memory.schema import AABB, Place, Pose, Submap

import p5sim_scenario as scenario

DB_PATH = ROOT / "work/p5_sim_validation/sim_memory_1b.db"
OUT_PATH = ROOT / "work/p5_sim_validation/run_1b_result.json"
REPORT_PATH = ROOT / "work/p5_sim_validation/run_1b_report.md"

ALL_EVENT_TYPES = {"add", "update", "confirm", "decay", "retire", "human_correct"}


def setup_system():
    if DB_PATH.exists():
        DB_PATH.unlink()
    sys_ = build_system(str(DB_PATH))
    sys_.entities.upsert_submap(Submap(
        submap_id=scenario.SUBMAP_ID, anchor_pose_world=Pose(0, 0, 0, 0),
        bounds=AABB(-10000, -10000, -10000, 10000, 10000, 10000)))
    # 复用 gt_instances.json 里已经定义的两个 place(1b 的合成对象都挂在这两个
    # place 之一,不需要新建 place 定义)
    gt_path = ROOT / "work/p5_sim_validation/gt_instances.json"
    gt = json.loads(gt_path.read_text(encoding="utf-8"))
    for place_id, pdef in gt["places"].items():
        sys_.entities.upsert_place(Place(place_id=place_id, name=pdef["name"],
                                          floor_id="F0", bounds=AABB(*pdef["bounds"])))
    return sys_


def _entity_field(e, field_name):
    if field_name == "pose":
        return {"x": e.pose.x, "y": e.pose.y, "z": e.pose.z, "yaw": e.pose.yaw}
    if field_name == "status":
        return e.status.value
    if field_name == "mobility":
        return e.mobility.value
    return getattr(e, field_name)


def check_final(sys_, case, key, expected):
    uuid = case.uuid_of[key]
    e = sys_.entities.get_object(uuid)
    results = []
    for field_name, exp_val in expected.items():
        if field_name.startswith("_"):
            continue  # special markers, handled by caller
        if field_name == "confidence_approx":
            actual = e.confidence
            ok = abs(actual - exp_val) < 0.01
            results.append((f"confidence≈{exp_val}", ok, actual))
            continue
        actual = _entity_field(e, field_name)
        ok = actual == exp_val
        results.append((field_name, ok, actual))
    return results, e


def main():
    sys_ = setup_system()

    all_results = []
    branch_hits = set()

    for case_fn in scenario.ALL_CASES:
        case = case_fn(sys_)
        case_report = {"id": case.id, "description": case.description,
                        "prediction_note": case.prediction_note, "objects": []}

        for key in case.keys:
            uuid = case.uuid_of[key]
            actual_events = [ev.event_type.value for ev in sys_.events.history(uuid)]
            expected_events = case.expected_events.get(key, [])
            branch_hits.update(actual_events)
            seq_match = actual_events == expected_events

            final_checks, entity = check_final(sys_, case, key, case.expected_final.get(key, {}))

            case_report["objects"].append({
                "key": key, "uuid": uuid,
                "expected_events": expected_events, "actual_events": actual_events,
                "sequence_match": seq_match,
                "final_checks": [{"field": f, "pass": ok, "actual": (
                    val.value if hasattr(val, "value") else val)}
                    for f, ok, val in final_checks],
                "final_status": entity.status.value, "final_version": entity.version,
                "final_confidence": round(entity.confidence, 4),
            })

        # special whole-case marker: top-level key on expected_final itself
        if "_n_distinct_entities" in case.expected_final:
            n_expected = case.expected_final["_n_distinct_entities"]
            uuids = {case.uuid_of[k] for k in case.keys}
            case_report["n_distinct_entities_expected"] = n_expected
            case_report["n_distinct_entities_actual"] = len(uuids)

        all_results.append(case_report)

    # ---- branch coverage ----
    coverage = {et: (et in branch_hits) for et in sorted(ALL_EVENT_TYPES)}
    n_covered = sum(coverage.values())

    # ---- sequence exact-match rate ----
    total_objs = sum(len(c["objects"]) for c in all_results)
    matched_objs = sum(1 for c in all_results for o in c["objects"] if o["sequence_match"])

    # ---- final-state consistency rate (all per-field checks passing) ----
    total_final_checks = sum(len(o["final_checks"]) for c in all_results for o in c["objects"])
    passed_final_checks = sum(1 for c in all_results for o in c["objects"]
                               for fc in o["final_checks"] if fc["pass"])

    summary = {
        "branch_coverage": coverage,
        "branch_coverage_rate": n_covered / len(ALL_EVENT_TYPES),
        "sequence_match_rate": matched_objs / total_objs if total_objs else 0.0,
        "n_objects": total_objs, "n_sequence_matched": matched_objs,
        "final_check_pass_rate": (passed_final_checks / total_final_checks
                                   if total_final_checks else 0.0),
        "n_final_checks": total_final_checks, "n_final_checks_passed": passed_final_checks,
    }

    OUT_PATH.write_text(json.dumps({"summary": summary, "cases": all_results},
                                    ensure_ascii=False, indent=2), encoding="utf-8")

    # ---- console + markdown report ----
    lines = ["# Phase5 sim validation -- 1b 时序分支覆盖结果\n"]
    lines.append(f"分支覆盖率: {n_covered}/{len(ALL_EVENT_TYPES)} "
                 f"({summary['branch_coverage_rate']:.0%})")
    lines.append(f"序列精确匹配率: {matched_objs}/{total_objs} "
                 f"({summary['sequence_match_rate']:.0%})")
    lines.append(f"终态一致率: {passed_final_checks}/{total_final_checks} "
                 f"({summary['final_check_pass_rate']:.0%})\n")
    lines.append("| 事件类型 | 是否命中 |")
    lines.append("|---|---|")
    for et, hit in coverage.items():
        lines.append(f"| {et} | {'✅' if hit else '❌'} |")
    lines.append("")
    print(f"分支覆盖率: {n_covered}/{len(ALL_EVENT_TYPES)}  {coverage}")
    print(f"序列精确匹配率: {matched_objs}/{total_objs}")
    print(f"终态检查通过率: {passed_final_checks}/{total_final_checks}")
    print()

    for c in all_results:
        lines.append(f"## {c['id']} — {c['description']}\n")
        if c["prediction_note"]:
            lines.append(f"**预注册预测**: {c['prediction_note']}\n")
        print(f"=== {c['id']} — {c['description']} ===")
        for o in c["objects"]:
            status = "PASS" if o["sequence_match"] else "FAIL"
            line = (f"  [{status}] {o['key']}: expected={o['expected_events']} "
                    f"actual={o['actual_events']}")
            print(line)
            lines.append(f"- `{o['key']}`: expected `{o['expected_events']}`, "
                         f"actual `{o['actual_events']}` "
                         f"{'✅' if o['sequence_match'] else '❌ MISMATCH'}")
            for fc in o["final_checks"]:
                mark = "OK" if fc["pass"] else "MISMATCH"
                print(f"      final.{fc['field']}: {mark} (actual={fc['actual']})")
                lines.append(f"  - final.{fc['field']}: "
                             f"{'✅' if fc['pass'] else '❌'} (actual={fc['actual']})")
        if "n_distinct_entities_expected" in c:
            ok = c["n_distinct_entities_expected"] == c["n_distinct_entities_actual"]
            print(f"      n_distinct_entities: expected={c['n_distinct_entities_expected']} "
                  f"actual={c['n_distinct_entities_actual']} {'OK' if ok else 'MISMATCH'}")
            lines.append(f"- 实体数量: 期望 {c['n_distinct_entities_expected']}, "
                         f"实际 {c['n_distinct_entities_actual']} {'✅' if ok else '❌'}")
        lines.append("")

    REPORT_PATH.write_text("\n".join(lines), encoding="utf-8")
    print(f"\nWrote {OUT_PATH}")
    print(f"Wrote {REPORT_PATH}")


if __name__ == "__main__":
    main()
