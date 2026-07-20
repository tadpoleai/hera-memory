# 运行手册:新数据跑通全流程

本文档记录用一份新的采集(`<basename>.hera` + `.insv` + `.session.json`)
从头跑通 Phase1→Phase4 的具体步骤和命令。详细的排错过程、失败尝试、决策
依据见 `STATUS.md`;这里只保留跑得通的路径。

## 前置:一次性环境

- `secrets/aliyun.env`:阿里云 AK/SK + OSS bucket/endpoint(`source` 后用)
- `secrets/acr.env`:ACR 登录凭证(`source` 后 `docker login`)
- `/home/fred/Code/mvp2/hera-desktop/config.toml`:已配置好(`glim_config_dir`
  指向 `~/.cache/hera/glim-recon/config`,`output_dir` 指向本项目
  `work/phase2`)
- FC 端点(已部署,直接调用即可,除非要改代码才需要重新 build/push):
  - 全景拼接:`https://hera-intitch-fc-uqjbgmbswr.cn-shanghai.fcapp.run`
    (镜像 `crpi-wzvoh0tsm7bwb22w.cn-shanghai.personal.cr.aliyuncs.com/glim/hera-insta-stitch-fc:v0.1.2`)
  - 实例提取:`https://conceptraphs-fc-tdfhlsjeui.cn-shanghai.fcapp.run`
    (镜像 `.../glim/concept-graphs:v0.1.4`)

## 新数据到手后要确认的信息(问采集者)

1. LiDAR 是否倒装 → `MOUNTING_RPY`(倒装用 `[0,180,0]`)
2. 房间/停留点数量 → `ROOM_COUNT`
3. 走的路线顺序(哪个停留对应哪个房间)——留着核对 Phase1 的静止段检测

---

## Phase 1:数据完整性验证

```bash
cd /home/fred/Code/spatial-memory
# 1. 软链接新数据(替换指向,不要复制/移动原始文件)
rm -f data/raw && ln -s /path/to/new/capture/dir data/raw

# 2. 改 config.env 的 BASENAME / MOUNTING_RPY / ROOM_COUNT

# 3. 提取 IMU + 点云采样
python3 scripts/p1_extract_imu.py \
  data/raw/<BASENAME>.hera work/phase1/imu.csv work/phase1/points_sample.csv

# 4. 六项自动验收 + 静止段可视化
PYTHONPATH=/home/fred/Code/hera-sdk-python python3 scripts/p1_checks.py
```

看 `work/phase1/static_segments.png`,人工核对静止段数量/时长和实际走的
路线对得上。**注意**:脚本里 `STILL_GYRO_THRESH=0.15`(已从规格的0.02放宽,
原因见 STATUS.md)是经验值,不同采集的手持抖动幅度可能需要重新调。

---

## Phase 2:GLIM 建图(稠密点云,当前最佳方案)

**不要**只用 `hera-run` 默认导出(`export_map_pcd.py`,只保留降采样后的
子图关键帧点,点数少、覆盖不全)。改用逐点插值的稠密导出:

```bash
# 1. GLIM 离线解算(glim-runner:r0.6 镜像自带 glim_offline)
docker run --rm --entrypoint /bin/bash \
  -v /path/to/data/dir:/data \
  crpi-wzvoh0tsm7bwb22w.cn-shanghai.personal.cr.aliyuncs.com/glim/glim-runner:r0.6 \
  -c '
cp -r /opt/glim_offline/share/glim/config /tmp/config
cp /tmp/config/config_headless.json /tmp/config/config.json
glim_offline /data/<BASENAME>.hera -c /tmp/config -o /data/map --window 0.1
'

# 2. 稠密导出:原始点(带时间戳)+ 逐点位姿插值(平移线性+四元数slerp)
#    脚本:/home/fred/Code/glim/scripts/export_hera_dense_pcd.py
docker run --rm --entrypoint /bin/bash \
  -v /path/to/data/dir:/data \
  -v /home/fred/Code/glim/scripts/export_hera_dense_pcd.py:/tmp/export_hera_dense_pcd.py:ro \
  crpi-wzvoh0tsm7bwb22w.cn-shanghai.personal.cr.aliyuncs.com/glim/glim-runner:r0.6 \
  -c 'python3 /tmp/export_hera_dense_pcd.py /data/<BASENAME>.hera /data/map/traj_lidar.txt \
      -o /data/map_export_capped.ply --max-points 1000000'
```

**倒装云台修正的说明**:目前实测发现"逐点插值稠密导出"这条路径,即使
**不加** `mounting_rpy`/`imu_acc_noise` 修正,质量也明显好于"子图刚体拼接
+ 加了修正"的旧路径(见 STATUS.md 的对比记录,原因未完全查清,猜测是
逐点插值对姿态误差更鲁棒)。**先按无修正跑一次,拿侧视图/俯视图肉眼判断
地面天花板是否基本水平**,不平再考虑 `glim_offline` 加 mounting_rpy 参数
(该二进制是否支持见 `docs/README_cn.md` 或直接试 `--help`)。

```bash
# 3. 本地质检
cd /home/fred/Code/spatial-memory
python3 scripts/p2_load_traj.py /path/to/data/dir/map work/phase2/trajectory.csv
python3 scripts/p2_checks.py /path/to/data/dir/map_export_capped.ply work/phase2/trajectory.csv
python3 scripts/p2_render.py /path/to/data/dir/map_export_capped.ply work/phase2/trajectory.csv work/phase2/
# 看 work/phase2/{topdown,sideview,trajectory_overlay}.png
```

若有零散离群点(y或x远超其他点2个数量级),手动过滤(参考 STATUS.md 里
"5个y>10m离群点"那次的处理方式:读PLY、布尔mask、重新写PLY)。

**确认无误后**,复制为正式交付物:
```bash
cp /path/to/data/dir/map_export_capped.ply deliverables/map_<BASENAME>.ply
```

---

## Phase 3:全景拼接 + 位姿绑定

```bash
# 1. 上传 .insv 到 OSS
source secrets/aliyun.env
python3 -c "
import os, oss2
bucket = oss2.Bucket(oss2.Auth(os.environ['ALIBABA_CLOUD_ACCESS_KEY_ID'], os.environ['ALIBABA_CLOUD_ACCESS_KEY_SECRET']), os.environ['OSS_ENDPOINT'], os.environ['OSS_BUCKET'])
bucket.put_object_from_file('slam-inputs/<BASENAME>.insv', 'data/raw/<BASENAME>.insv')
"

# 2. 调用拼接 FC(注意用 /stitch-insv-oss,不是 /stitch-oss ——
#    /stitch-oss 走 .hera 里的 InstaVideoPacket,这批设备的采集配置下
#    视频只进了 .insv,.hera 里没有视频包)
#
#    ⚠️ flowstate 必须是 False,不是 True!2026-07-14 debug 定位到:
#    flowstate=True 会让 Insta360 MediaSDK 对每一帧全景独立做一次基于自身
#    陀螺仪的水平锁定/防抖重定向,导致每帧的"哪个方向是前"都不一样,
#    根本不存在一个能同时适配所有帧的固定外参(表现为:每张 overlay 图
#    需要的yaw修正角度都不一样)。flowstate=False 才是全景内容刚性绑定在
#    相机机身上的版本,才能用 work/phase3/extrinsic.json 里那套固定外参。
python3 -c "
import requests
r = requests.post('https://hera-intitch-fc-uqjbgmbswr.cn-shanghai.fcapp.run/stitch-insv-oss',
    json={'input_bucket':'uploaded-hera','input_key':'slam-inputs/<BASENAME>.insv',
          'output_key':'slam-outputs/<BASENAME>.pano.mp4',
          'stitch_type':'optflow','flowstate':False}, timeout=900)
print(r.status_code, r.text)
"

# 3. 下载拼接结果
python3 -c "
import os, oss2
bucket = oss2.Bucket(oss2.Auth(...), os.environ['OSS_ENDPOINT'], os.environ['OSS_BUCKET'])
bucket.get_object_to_file('slam-outputs/<BASENAME>.pano.mp4', 'work/phase3/pano.mp4')
"

# 4. 按 Phase1 确认的静止段,改 scripts/p3_extract_frames.py 里的
#    SEGMENTS_IMU_REL / IMU_ABS_START(从 work/phase1/imu.csv 第一行取)
#    / SESSION 路径,然后:
python3 scripts/p3_extract_frames.py
python3 scripts/p3_bind_pose.py     # 需要 work/phase2/trajectory.csv 已生成
python3 scripts/p3_project_check.py # 外参投影目视检查,看 work/phase3/overlay_*.png
```

**外参现状(2026-07-14 确认)**:`p3_bind_pose.py` 里已经写死了
`T_LIDAR_CAMERA_ROT = yaw 90deg`(见 work/phase3/extrinsic.json 的完整推导
记录)。这套外参**只对 flowstate=False 拼出来的全景有效**。如果每次采集
相机安装方式不变(同一副支架/同一个相机),理论上不用重新标定,直接跑
`p3_project_check.py` 看 overlay 图确认一下就行;如果看着还是有明显错位
(尤其是不同停留点错位方向/幅度不一致——这是 flowstate 类问题的典型症状),
参考 `scripts/p3_calibrate_extrinsic.py` 的手动 yaw 试值流程重新迭代
(改 yaw 值 → 跑 render_overlay → 目视确认 → 循环),别绕过人工确认这一步。

---

## Phase 4:实例提取(ConceptGraphs on FC GPU)

```bash
# 1. 本地预处理:cubemap切片 + 深度渲染
#    改 scripts/p4_prepare.py 里的 PLY/FRAMES 路径为新数据的
python3 scripts/p4_prepare.py
# 检查 hole_rate 打印,大量面 >20% 是预期的(继承自Phase2点云密度限制)

# 2. 上传 bundle 到 OSS
python3 -c "
import os, oss2
bucket = oss2.Bucket(oss2.Auth(...), os.environ['OSS_ENDPOINT'], os.environ['OSS_BUCKET'])
for root, dirs, files in os.walk('work/phase4/upload_bundle'):
    for name in files:
        local = os.path.join(root, name)
        rel = os.path.relpath(local, 'work/phase4/upload_bundle')
        bucket.put_object_from_file('phase4/upload_bundle/' + rel.replace(os.sep,'/'), local)
"

# 3. 调用实例提取 FC(耗时约3分钟/15帧,GPU Ada 48G实测)
python3 -c "
import requests
r = requests.post('https://conceptraphs-fc-tdfhlsjeui.cn-shanghai.fcapp.run/extract-oss',
    json={'input_bucket':'uploaded-hera','input_prefix':'phase4/upload_bundle/'}, timeout=900)
print(r.status_code, r.text)
"

# 4. 下载结果
python3 -c "
import os, oss2
bucket = oss2.Bucket(oss2.Auth(...), os.environ['OSS_ENDPOINT'], os.environ['OSS_BUCKET'])
bucket.get_object_to_file('phase4/upload_bundle_instances/instances.json', 'work/phase4/instances_result/instances.json')
bucket.get_object_to_file('phase4/upload_bundle_instances/objects_debug.ply', 'work/phase4/instances_result/objects_debug.ply')
"
```

### 如果要改 FC GPU 镜像代码

```bash
cd /home/fred/Code/hera-sdk-python/tools/fc-gpu-conceptgraphs
# 改代码后,版本号+1,构建时记得带代理(本机连GitHub/PyPI要走本地代理):
docker build -t crpi-wzvoh0tsm7bwb22w.cn-shanghai.personal.cr.aliyuncs.com/glim/concept-graphs:v0.1.X \
  --add-host=host.docker.internal:host-gateway \
  --build-arg http_proxy=http://host.docker.internal:7897 \
  --build-arg https_proxy=http://host.docker.internal:7897 .
docker push crpi-wzvoh0tsm7bwb22w.cn-shanghai.personal.cr.aliyuncs.com/glim/concept-graphs:v0.1.X
# 然后去FC控制台更新镜像版本号并重新发布
```

**磁盘空间提醒**:这个镜像构建链条(torch+cuda+open3d等)一次要用 15-20GB
临时空间,本机磁盘经常紧张。构建前检查 `df -h /`,不够就
`docker container prune -f && docker image prune -f`,如果还不够,删掉
本地多余的旧版本 `concept-graphs` 镜像(已推送到ACR的,本地删了不会丢,
`docker images | grep concept-graphs` 找出来 `docker rmi`)。

---

## Phase 5:尚未实现

空间记忆入库(SpatialQuery/consolidator)+ 四层审计还没做,是下一步。
