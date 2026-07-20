"""
p5sim_visualize_memory.py — 在 Isaac Sim 中加载真实场景并叠加 Phase5 记忆库状态

用法（用 Isaac Sim 自带 python 跑，不要用项目 venv）:
  ~/isaacsim/python.sh scripts/p5sim_visualize_memory.py \
      --scene  <scene.usd | scene.obj | points.ply> \
      --db     work/p5_sim_validation/sim_memory_1b.db \
      --mode   snapshot            # snapshot | replay | occlusion
      [--gt    work/p5_sim_validation/gt_instances.json]   # 可选: 提供 AABB 尺寸与真值对比
      [--stations stations.json]                           # occlusion 模式: 站点列表
      [--yup]                                              # 场景源数据是 Y-up 时加此开关

三种模式:
  snapshot  — 静态叠加: 每个记忆实体一个半透明盒, 颜色=状态/置信度
  replay    — 按 events 表 seq 逐事件回放 (空格暂停, 方向键步进见控制台提示)
  occlusion — 从站点向候选实体画通视/遮挡光线 (T12 演示, 需 --stations)

设计要点:
  * 算法不在这里跑; 本脚本只读 SQLite, 是纯展示层
  * Y-up→Z-up 用 rotateX=+90° 实现, 数学上等价于修复后的 yup_to_zup:
    Rx(90): (x,y,z)->(x,-z,y)  —— 与 p5_ingest_4dkankan.py 当前正确版一致
  * ObjectInstance 无 bbox 字段, 盒子尺寸优先取 --gt 里同 uuid 的 AABB,
    否则退化为 0.35m 立方体 (只影响观感, 不影响位置正确性)
  * 注意: Isaac Sim 4.5 扩展命名处于 omni.isaac.* → isaacsim.* 过渡期,
    个别 import 若报错, 按本机实际扩展名微调 (标注了 [API-CHECK] 的行)
"""

import argparse
import json
import math
import sqlite3
import sys
from pathlib import Path

# ----------------------------------------------------------------------------
# 参数解析必须在 SimulationApp 启动前完成
# ----------------------------------------------------------------------------
parser = argparse.ArgumentParser()
parser.add_argument("--scene", required=True)
parser.add_argument("--db", required=True)
parser.add_argument("--mode", default="snapshot",
                    choices=["snapshot", "replay", "occlusion"])
parser.add_argument("--gt", default=None)
parser.add_argument("--stations", default=None)
parser.add_argument("--yup", action="store_true",
                    help="场景源数据是 Y-up (如 4dkankan/realsee 的 OBJ)")
parser.add_argument("--headless", action="store_true")
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402
app = SimulationApp({"headless": args.headless})

import omni.usd                                    # noqa: E402
from pxr import Usd, UsdGeom, Gf, Sdf, UsdShade    # noqa: E402

# ----------------------------------------------------------------------------
# 1. 读取记忆库
# ----------------------------------------------------------------------------

def load_memory(db_path):
    con = sqlite3.connect(db_path)
    con.row_factory = sqlite3.Row
    objs = [dict(r) for r in con.execute(
        "SELECT uuid, class_label, place_id, status, confidence,"
        "       x, y, z, yaw, version, doc FROM objects")]
    for o in objs:
        try:
            o["doc"] = json.loads(o["doc"])
        except Exception:
            o["doc"] = {}
    events = [dict(r) for r in con.execute(
        "SELECT seq, event_id, entity_uuid, event_type, source,"
        "       timestamp_us, payload FROM events ORDER BY seq")]
    con.close()
    return objs, events


def load_gt_sizes(gt_path):
    """gt_instances.json -> [(class_label, x, y, z, sx, sy, sz), ...]; 无文件返回空列表。

    gt_instances.json 顶层是 {"instances": [...], "places": {...}, ...} 一个
    dict,不是裸 list -- 之前直接 `for item in json.loads(...)` 会把 dict 的
    key(字符串)当条目遍历,`item.get(...)` 直接崩(实测踩到)。

    真值条目里没有 uuid(uuid 是入库时才生成的,gt_instances.json 和任何
    memory DB 之间没有天然的 join key),所以按 (class_label, 最近位置) 匹配,
    思路和 consolidator 自己的匹配门控类似,但只用来决定可视化盒子尺寸。
    对 sim_memory.db(1a,真实 scene_04 103 个实例)有意义;对
    sim_memory_1b.db(T1-T11 合成测试对象,坐标故意放在场景外很远的地方)
    不会匹配到任何东西,盒子会全部退化成默认 0.35m 立方体 -- 这是正确行为,
    不是 bug,1b 的库本来就不该期待和这份真值对得上。
    """
    if not gt_path:
        return []
    data = json.loads(Path(gt_path).read_text())
    items = data.get("instances", []) if isinstance(data, dict) else data
    out = []
    for item in items:
        bb = item.get("aabb") or item.get("bounds")
        pose = item.get("pose") or {}
        label = item.get("class_label")
        if bb and label and "x" in pose:
            out.append((label, pose["x"], pose["y"], pose.get("z", 0.0),
                        max(0.1, bb["max_x"] - bb["min_x"]),
                        max(0.1, bb["max_y"] - bb["min_y"]),
                        max(0.1, bb["max_z"] - bb["min_z"])))
    return out


GT_MATCH_MAX_DIST = 0.5  # 米,只影响可视化盒子尺寸,不是匹配算法本身


def _lookup_gt_size(gt_list, label, x, y, z):
    best, best_d = None, GT_MATCH_MAX_DIST
    for g_label, gx, gy, gz, sx, sy, sz in gt_list:
        if g_label != label:
            continue
        d = math.dist((x, y, z), (gx, gy, gz))
        if d < best_d:
            best, best_d = (sx, sy, sz), d
    return best or (0.35, 0.35, 0.35)


# ----------------------------------------------------------------------------
# 2. 场景加载 (USD 直开 / OBJ 转换 / PLY 点云)
# ----------------------------------------------------------------------------

def load_scene(stage, scene_path, yup):
    scene_path = str(scene_path)
    root = UsdGeom.Xform.Define(stage, "/World/Scene")
    if yup:
        # Rx(+90°) == 修复后的 yup_to_zup: (x,y,z)->(x,-z,y)
        root.AddRotateXOp().Set(90.0)

    if scene_path.endswith(".usd") or scene_path.endswith(".usdc") \
            or scene_path.endswith(".usda"):
        ref = stage.DefinePrim("/World/Scene/mesh")
        ref.GetReferences().AddReference(scene_path)

    elif scene_path.endswith(".obj"):
        # [API-CHECK] asset converter 是异步扩展; 若此段报错, 先用 GUI:
        #   File > Import 把 OBJ 转成 USD, 再以 --scene xxx.usd 重跑
        import asyncio
        import omni.kit.asset_converter as conv
        out_usd = scene_path.rsplit(".", 1)[0] + "_converted.usd"

        async def _do():
            task = conv.get_instance().create_converter_task(
                scene_path, out_usd, None)
            ok = await task.wait_until_finished()
            if not ok:
                raise RuntimeError(task.get_error_message())
        asyncio.get_event_loop().run_until_complete(_do())
        ref = stage.DefinePrim("/World/Scene/mesh")
        ref.GetReferences().AddReference(out_usd)

    elif scene_path.endswith(".ply"):
        # hera 点云: 直接建 UsdGeomPoints (20 万级无压力)
        import numpy as np
        pts = _read_ply_xyz(scene_path)          # 简易读取, 只取 xyz
        geom = UsdGeom.Points.Define(stage, "/World/Scene/cloud")
        geom.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
        geom.CreateWidthsAttr([0.02] * len(pts))
        geom.CreateDisplayColorAttr([Gf.Vec3f(0.6, 0.6, 0.65)])
    else:
        raise ValueError(f"不认识的场景格式: {scene_path}")


def _read_ply_xyz(path, max_points=400_000):
    import numpy as np
    try:
        import open3d as o3d                     # o3d venv 里跑才有; 无则手解析
        pc = o3d.io.read_point_cloud(path)
        arr = np.asarray(pc.points)
    except ImportError:
        arr = _read_ply_ascii_fallback(path)
    if len(arr) > max_points:                    # 均匀降采样, 保观感
        arr = arr[:: len(arr) // max_points + 1]
    return arr


def _read_ply_ascii_fallback(path):
    import numpy as np
    with open(path, "rb") as f:
        header, n = [], 0
        while True:
            line = f.readline().decode("ascii", "ignore").strip()
            header.append(line)
            if line.startswith("element vertex"):
                n = int(line.split()[-1])
            if line == "end_header":
                break
        if any("binary" in h for h in header):
            raise RuntimeError("二进制 PLY 请装 open3d 或先转 ASCII")
        rows = [f.readline().split()[:3] for _ in range(n)]
    return np.array(rows, dtype=float)


# ----------------------------------------------------------------------------
# 3. 记忆实体 -> 彩色半透明盒
# ----------------------------------------------------------------------------

def status_color(obj):
    """active: 置信度 1.0 绿 -> 0.25 黄橙; retired: 灰; pinned: 蓝。"""
    doc = obj.get("doc", {})
    if doc.get("pinned_until_us", 0) > 0:
        return Gf.Vec3f(0.20, 0.45, 0.95)
    if obj["status"] != "active":
        return Gf.Vec3f(0.45, 0.45, 0.45)
    c = max(0.0, min(1.0, float(obj["confidence"])))
    return Gf.Vec3f(1.0 - 0.8 * c, 0.55 + 0.35 * c, 0.15)


def sanitize(name):
    return "".join(ch if ch.isalnum() else "_" for ch in name)[:48]


def draw_entities(stage, objs, gt_sizes, missed_uuids=frozenset()):
    root = UsdGeom.Xform.Define(stage, "/World/Memory")
    for o in objs:
        label = sanitize(f"{o['class_label']}_{o['uuid'][:8]}")
        prim_path = f"/World/Memory/{label}"
        xf = UsdGeom.Xform.Define(stage, prim_path)
        xf.AddTranslateOp().Set(Gf.Vec3d(o["x"], o["y"], o["z"]))
        xf.AddRotateZOp().Set(math.degrees(o["yaw"] or 0.0))

        cube = UsdGeom.Cube.Define(stage, prim_path + "/box")
        sx, sy, sz = _lookup_gt_size(gt_sizes, o["class_label"], o["x"], o["y"], o["z"])
        cube.AddScaleOp().Set(Gf.Vec3f(sx / 2, sy / 2, sz / 2))  # Cube 边长 2
        color = (Gf.Vec3f(0.95, 0.15, 0.15)      # 真值漏检: 红
                 if o["uuid"] in missed_uuids else status_color(o))
        cube.CreateDisplayColorAttr([color])
        cube.CreateDisplayOpacityAttr([0.45])
        # prim 名即标签: 视口里点选/stage 树可读 class_label + uuid 前缀
    return root


# ----------------------------------------------------------------------------
# 4. replay 模式: 按事件 seq 重建每步状态
#    置信度轨迹确定性重建: CONFIRM +0.1 / UPDATE +0.2 (封顶1.0),
#    DECAY 按 mobility 扣减 (static .02 / semi .30 / dynamic 1.0)
# ----------------------------------------------------------------------------

DECAY_STEP = {"static": 0.02, "semi_static": 0.30, "dynamic": 1.0}


def build_timeline(objs, events):
    base = {o["uuid"]: dict(o) for o in objs}
    # 从终态无法直接回放, 改为从 ADD 正向推演
    state, frames = {}, []
    for ev in events:
        uid, et = ev["entity_uuid"], ev["event_type"].lower()
        payload = {}
        try:
            payload = json.loads(ev["payload"])
        except Exception:
            pass
        if et == "add":
            proto = dict(base.get(uid, {}))
            proto.update({"status": "active", "confidence": 1.0})
            for k in ("x", "y", "z", "yaw"):
                if k in payload:
                    proto[k] = payload[k]
            state[uid] = proto
        elif uid in state:
            s = state[uid]
            mob = (s.get("doc", {}).get("mobility") or "semi_static").lower()
            if et == "confirm":
                s["confidence"] = min(1.0, s["confidence"] + 0.1)
            elif et == "update":
                s["confidence"] = min(1.0, s["confidence"] + 0.2)
                for k in ("x", "y", "z", "yaw"):
                    if k in payload:
                        s[k] = payload[k]
            elif et == "decay":
                s["confidence"] -= DECAY_STEP.get(mob, 0.30)
            elif et == "retire":
                s["status"] = "retired"
            elif et == "human_correct":
                for k in ("x", "y", "z", "yaw"):
                    if k in payload:
                        s[k] = payload[k]
                s.setdefault("doc", {})["pinned_until_us"] = 1
        frames.append((ev["seq"], et, uid,
                       {u: dict(v) for u, v in state.items()}))
    return frames


def run_replay(stage, objs, events, gt_sizes):
    frames = build_timeline(objs, events)
    print(f"[replay] 共 {len(frames)} 个事件帧; 控制台回车步进, q 退出")
    for seq, et, uid, snap in frames:
        # 清掉旧盒重画 (帧数不大, 简单粗暴即可)
        old = stage.GetPrimAtPath("/World/Memory")
        if old.IsValid():
            stage.RemovePrim("/World/Memory")
        draw_entities(stage, list(snap.values()), gt_sizes)
        print(f"  seq={seq:4d}  {et.upper():14s}  entity={uid[:8]}  "
              f"库内active={sum(1 for s in snap.values() if s['status']=='active')}")
        for _ in range(3):
            app.update()
        cmd = input("      [Enter]下一事件 / q 退出 > ").strip().lower()
        if cmd == "q":
            break


# ----------------------------------------------------------------------------
# 5. occlusion 模式: 站点 -> 实体 通视光线 (T12 演示)
#    光线求交结果建议直接复用 t12_occlusion_result.json, 本模式只负责画
# ----------------------------------------------------------------------------

def run_occlusion(stage, objs, stations_path):
    stations = json.loads(Path(stations_path).read_text())
    # stations.json 期望格式:
    # [{"name": "...", "x":.., "y":.., "z":.., "radius": 6.0,
    #   "blocked_uuids": [...], "visible_uuids": [...]}]
    obj_by_uuid = {o["uuid"]: o for o in objs}
    for i, st in enumerate(stations):
        sp = Gf.Vec3f(st["x"], st["y"], st.get("z", 1.0))
        marker = UsdGeom.Sphere.Define(stage, f"/World/Stations/st_{i}")
        marker.AddTranslateOp().Set(Gf.Vec3d(*sp))
        marker.CreateRadiusAttr(0.12)
        marker.CreateDisplayColorAttr([Gf.Vec3f(0.1, 0.3, 1.0)])
        for kind, uuids, col in (
                ("clear", st.get("visible_uuids", []), Gf.Vec3f(0.1, 0.85, 0.2)),
                ("blocked", st.get("blocked_uuids", []), Gf.Vec3f(0.95, 0.1, 0.1))):
            for j, uid in enumerate(uuids):
                o = obj_by_uuid.get(uid)
                if not o:
                    continue
                curve = UsdGeom.BasisCurves.Define(
                    stage, f"/World/Stations/st_{i}/{kind}_{j}")
                curve.CreateTypeAttr("linear")
                curve.CreatePointsAttr([sp, Gf.Vec3f(o["x"], o["y"], o["z"])])
                curve.CreateCurveVertexCountsAttr([2])
                curve.CreateWidthsAttr([0.015, 0.015])
                curve.CreateDisplayColorAttr([col])


# ----------------------------------------------------------------------------
# main
# ----------------------------------------------------------------------------

def main():
    ctx = omni.usd.get_context()
    ctx.new_stage()
    stage = ctx.get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")

    objs, events = load_memory(args.db)
    gt_sizes = load_gt_sizes(args.gt)
    print(f"[load] 实体 {len(objs)} 条 / 事件 {len(events)} 条 "
          f"/ 带尺寸真值 {len(gt_sizes)} 条")

    load_scene(stage, args.scene, args.yup)

    if args.mode == "snapshot":
        draw_entities(stage, objs, gt_sizes)
        # 可选: 把 GT 里库中完全不存在的条目画成红色小盒 (系统性漏检可视化),
        # 需要 gt 与 db 的 uuid 体系一致; 4dkankan 的 29 条独立标注真值
        # 若 uuid 不同源, 改成按 (label, 距离<0.5m) 匹配后取补集即可
    elif args.mode == "replay":
        run_replay(stage, objs, events, gt_sizes)
    elif args.mode == "occlusion":
        if not args.stations:
            sys.exit("occlusion 模式需要 --stations stations.json")
        draw_entities(stage, objs, gt_sizes)
        run_occlusion(stage, objs, args.stations)

    if not args.headless:
        print("[view] 进入交互查看; 关窗口退出")
        while app.is_running():
            app.update()
    app.close()


if __name__ == "__main__":
    main()