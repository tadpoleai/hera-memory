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
parser.add_argument("--place", default=None,
                    help="只画/只对焦某个 place_id 的实体 (snapshot 模式很有用:"
                         "不加这个选项相机会按全部 place 的整体范围取景, 如果各"
                         "place 尺度悬殊 -- 比如一个小房间 vs 一整个庭院 -- 镜头"
                         "会被大的那个拉得很远, 小 place 里的细节完全看不清)")
parser.add_argument("--exclude-labels", default=None,
                    help="逗号分隔的 class_label 列表, 从渲染/取景里排除"
                         "(snapshot 模式很有用: wall/floor/window 这类占满"
                         "整个房间的结构件会把 framing 撑得很开, 家具类小"
                         "物体挤成几个像素, 排除掉结构件才能看清框有没有"
                         "真的贴在家具上, 例如 --exclude-labels wall,floor,"
                         "window,floor_tile)")
parser.add_argument("--stations", default=None)
parser.add_argument("--yup", action="store_true",
                    help="场景源数据是 Y-up (如 4dkankan/realsee 的 OBJ)")
parser.add_argument("--auto", type=float, default=0.0,
                    help="replay 自动播放, 每事件停留秒数; 0=回车步进(默认)")
parser.add_argument("--shots", default=None,
                    help="replay 每事件截图输出目录 (可选, 用于录demo/做GIF)")
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
    """gt_instances.json -> {uuid: (sx, sy, sz)}; 无文件返回空表。

    兼容顶层三种形态:
      1. [ {...}, {...} ]                      # 列表
      2. {"instances": [...]} 等包裹键          # 包裹列表
      3. { "<uuid>": {...}, ... }              # uuid 做键的字典
    兼容 AABB 两种形态:
      a. {"min_x":..,"max_x":..,...}
      b. {"min":[x,y,z], "max":[x,y,z]}  或 6 元素平铺列表
    """
    if not gt_path:
        return {"by_uuid": {}, "entries": []}
    data = json.loads(Path(gt_path).read_text())

    if isinstance(data, dict):
        candidates = []
        for k, v in data.items():
            if isinstance(v, list) and v and isinstance(v[0], dict):
                candidates.append((k, v))
            elif isinstance(v, dict) and v and all(
                    isinstance(x, dict) for x in list(v.values())[:3]):
                candidates.append((k, [dict(x, uuid=x.get("uuid", kk))
                                       for kk, x in v.items()]))
        if candidates:
            # 键名含 instance/object 的优先 (避免误选 places 等旁支键);
            # 同优先级取条目最多的
            def _rank(c):
                name = c[0].lower()
                named = ("instance" in name) or ("object" in name)
                return (0 if named else 1, -len(c[1]))
            candidates.sort(key=_rank)
            if len(candidates) > 1:
                print(f"[gt] 候选键 {[k for k, _ in candidates]}, "
                      f"选用 '{candidates[0][0]}' ({len(candidates[0][1])} 条)")
            data = candidates[0][1]
        else:  # 顶层本身就是 uuid 做键的字典
            data = [dict(v, uuid=v.get("uuid", k))
                    for k, v in data.items() if isinstance(v, dict)]

    by_uuid, entries = {}, []
    for item in data:
        if not isinstance(item, dict):
            continue
        uid = item.get("uuid") or item.get("id")
        label = item.get("class_label") or item.get("label")
        bb = (item.get("aabb") or item.get("bounds")
              or item.get("bbox") or item.get("aabb_world"))
        if bb is None:
            continue
        try:
            if isinstance(bb, dict) and "min_x" in bb:
                lo = (bb["min_x"], bb["min_y"], bb["min_z"])
                hi = (bb["max_x"], bb["max_y"], bb["max_z"])
            elif isinstance(bb, dict) and "min" in bb:
                lo, hi = tuple(bb["min"][:3]), tuple(bb["max"][:3])
            elif isinstance(bb, (list, tuple)) and len(bb) == 6:
                lo, hi = tuple(bb[:3]), tuple(bb[3:])
            else:
                continue
            ext = tuple(max(0.1, abs(hi[i] - lo[i])) for i in range(3))
            ctr = tuple((hi[i] + lo[i]) / 2.0 for i in range(3))
            # label 一起存: 最近邻匹配必须同标签才可信, 见 _size_for 的说明
            # (实测踩过坑: 不按标签过滤时, wall/floor 这类横跨整个房间的
            # 巨大 AABB 会把"离它质心最近"的任何家具类实体的尺寸污染成
            # 房间大小的板子, --exclude-labels 只影响画不画, 不影响这个
            # 匹配池, 两者要分开处理)
            entries.append((label, ctr, ext))
            if uid:
                by_uuid[uid] = ext
        except (KeyError, IndexError, TypeError):
            continue

    if not entries:
        sample = data[0] if isinstance(data, list) and data else data
        print(f"[warn] gt 未解析出任何 AABB, 盒子将用默认尺寸 0.35m。"
              f"首条目结构样例: {str(sample)[:300]}")
    else:
        print(f"[gt] 尺寸解析成功 {len(entries)} 条 "
              f"(uuid 可索引 {len(by_uuid)} 条)")
    return {"by_uuid": by_uuid, "entries": entries}


def _size_for(gt_index, o, max_dist=0.75):
    """uuid 直查; 失配时按 (同 class_label, AABB 中心最近邻) 匹配。
    consolidator ADD 时会生成新 uuid, 与真值提取侧 uuid 不同源,
    所以位置最近邻才是常态匹配路径, uuid 直查只是巧合命中时的捷径。

    必须同 label 才能匹配, 不能只按距离 -- 实测踩过坑: wall/floor 这类
    横跨整个房间的巨大 AABB, 只要质心离某个家具类实体足够近(哪怕差着好几米
    的量级, 房间尺度下"最近"很容易就是它们), 不按 label 过滤就会把家具的
    盒子尺寸污染成房间大小的板子 -- 而且这个污染源和 --exclude-labels 是否
    把 wall/floor 排除在画面外无关, 因为那只影响画不画, 这个匹配池不受影响。
    """
    ext = gt_index["by_uuid"].get(o["uuid"])
    if ext:
        return ext
    label = o["class_label"]
    best, best_d2 = None, max_dist * max_dist
    for g_label, ctr, e in gt_index["entries"]:
        if g_label != label:
            continue
        d2 = ((ctr[0] - o["x"]) ** 2 + (ctr[1] - o["y"]) ** 2
              + (ctr[2] - o["z"]) ** 2)
        if d2 < best_d2:
            best, best_d2 = e, d2
    return best or (0.35, 0.35, 0.35)


# ----------------------------------------------------------------------------
# 2. 场景加载 (USD 直开 / OBJ 转换 / PLY 点云)
# ----------------------------------------------------------------------------

def load_scene(stage, scene_path, yup):
    # reference 相对匿名 stage 解析而非工作目录, 必须转绝对路径
    scene_path = str(Path(scene_path).resolve())
    if not Path(scene_path).exists():
        raise FileNotFoundError(f"场景文件不存在: {scene_path}")
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


def _try_frame(prim_path="/World/Memory"):
    """选中 prim 并让视口相机自动 framing (等效手动选中后按 F)。
    [API-CHECK] 若本机 API 不符只打警告不中断。"""
    try:
        import omni.kit.viewport.utility as vu
        ctx = omni.usd.get_context()
        ctx.get_selection().set_selected_prim_paths([prim_path], True)
        vu.frame_viewport_selection(vu.get_active_viewport())
        app.update()
    except Exception as e:
        print(f"[warn] 自动对准视角失败(可手动: Stage选中后按F): {e}")


def _capture(path):
    """存一张当前视口截图。snapshot/occlusion 模式在 --headless 下如果不
    调用这个, 什么文件都不会落盘 -- 之前只有 replay 模式接了 --shots,
    snapshot/occlusion 在无人值守/纯远程验证时完全没法留证据, 补齐。
    [API-CHECK] 截图 API 位置同 run_replay 里那处。"""
    try:
        from omni.kit.viewport.utility import (
            get_active_viewport, capture_viewport_to_file)
        for _ in range(3):
            app.update()
        capture_viewport_to_file(get_active_viewport(), str(path))
        app.update()
        print(f"[shot] 已存 {path}")
    except Exception as e:
        print(f"[warn] 截图失败: {e}")


def draw_entities(stage, objs, gt_sizes, missed_uuids=frozenset()):
    root = UsdGeom.Xform.Define(stage, "/World/Memory")
    for o in objs:
        label = sanitize(f"{o['class_label']}_{o['uuid'][:8]}")
        prim_path = f"/World/Memory/{label}"
        xf = UsdGeom.Xform.Define(stage, prim_path)
        xf.AddTranslateOp().Set(Gf.Vec3d(o["x"], o["y"], o["z"]))
        xf.AddRotateZOp().Set(math.degrees(o["yaw"] or 0.0))

        cube = UsdGeom.Cube.Define(stage, prim_path + "/box")
        sx, sy, sz = _size_for(gt_sizes, o)
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
    import time
    frames = build_timeline(objs, events)
    shots_dir = Path(args.shots) if args.shots else None
    if shots_dir:
        shots_dir.mkdir(parents=True, exist_ok=True)
    mode_desc = (f"自动播放, 每事件 {args.auto}s" if args.auto > 0
                 else "控制台回车步进, q 退出")
    print(f"[replay] 共 {len(frames)} 个事件帧; {mode_desc}")
    # 先用"终局全体实体分布"画一遍并 framing, 让机位一开始就覆盖全场,
    # 避免对着 seq=1 的单个小盒怼特写、后续实体全部出画
    if frames:
        _, _, _, last_snap = frames[-1]
        draw_entities(stage, list(last_snap.values()), gt_sizes)
        for _ in range(3):
            app.update()
        _try_frame("/World/Memory")
        print("      [提示] 机位已按全体实体范围预置, 现在可手动微调, "
              "调好后回车开始播放" if args.auto == 0 else
              "      [提示] 机位已按全体实体范围预置")
        if args.auto == 0:
            input("      按回车开始 > ")
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
        # 每帧对焦到本次事件实际变化的那个实体, 不是全程焦在整体范围不动。
        # 之前全程用一次性的整体 framing 拍 25 张回放截图, 实测发现构图
        # 几乎一模一样看不出区别 -- T1-T11 的合成对象排成一条约200m长的线,
        # 又都是默认0.35m小方块, 固定远景下每帧只占几个像素, 技术上对但
        # 视觉上没有诊断价值。改成逐帧对焦到 uid 对应的 prim。
        target = snap.get(uid)
        if target is not None:
            label = sanitize(f"{target['class_label']}_{uid[:8]}")
            _try_frame(f"/World/Memory/{label}")
        if shots_dir:
            _capture(shots_dir / f"seq_{seq:04d}_{et}.png")
        if args.auto > 0:
            t0 = time.time()
            while time.time() - t0 < args.auto:
                app.update()
        else:
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
          f"/ 带尺寸真值 {len(gt_sizes['entries'])} 条")

    if args.place:
        before = len(objs)
        objs = [o for o in objs if o["place_id"] == args.place]
        events = [e for e in events if e["entity_uuid"] in {o["uuid"] for o in objs}]
        print(f"[place] 过滤到 place_id={args.place}: {before} -> {len(objs)} 条实体")

    if args.exclude_labels:
        excl = {s.strip() for s in args.exclude_labels.split(",") if s.strip()}
        before = len(objs)
        objs = [o for o in objs if o["class_label"] not in excl]
        events = [e for e in events if e["entity_uuid"] in {o["uuid"] for o in objs}]
        print(f"[exclude] 排除 {excl}: {before} -> {len(objs)} 条实体")

    load_scene(stage, args.scene, args.yup)

    if args.mode == "snapshot":
        draw_entities(stage, objs, gt_sizes)
        _try_frame("/World/Memory")
        if args.shots:
            Path(args.shots).mkdir(parents=True, exist_ok=True)
            _capture(Path(args.shots) / "snapshot.png")
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
        _try_frame("/World/Stations")
        if args.shots:
            Path(args.shots).mkdir(parents=True, exist_ok=True)
            _capture(Path(args.shots) / "occlusion.png")

    if not args.headless:
        print("[view] 进入交互查看; 关窗口退出")
        while app.is_running():
            app.update()
    app.close()


if __name__ == "__main__":
    main()