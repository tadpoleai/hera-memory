# Hera 自采数据规格

> 本文档描述本项目**自有**的 hera 采集数据：硬件组成、传感器数据类型、
> 存储格式、以及目前在管线各阶段的实测质量状况。第三方参考数据（4dkankan、
> realsee）另见 `docs/third_party_reference_data_spec.md`。

---

## 1. 采集硬件

hera 采集端由两颗物理绑定在同一支架上的传感器组成，通过 `recorder` 仓库
（`/home/fred/Code/recorder`）的采集程序统一写入 `.hera` 容器：

| 硬件 | 型号 | 驱动位置 | 提供的数据 |
|---|---|---|---|
| 激光雷达 | **Livox Mid-360**（机械式固态激光雷达，内置 IMU） | `device/plugin/lidar/livox/` | 点云 + IMU |
| 全景相机 | **Insta360**（型号未在代码中显式标注，走 SDK `camera/camera.h`） | `device/plugin/camera/insta/` | 全景视频 + 相机自带陀螺 |

- 本项目实测安装方式为**倒装**（`config.env` 中 `MOUNTING_RPY="[0.0, 180.0, 0.0]"`，
  用户已确认），下游所有位姿/外参计算都要按此安装角修正。
- `session.json` 中 `device_vendor_type: 1057` 是这套"Mid-360 + Insta360"
  组合的设备标识，用于区分未来可能接入的其他硬件组合（如 Hesai/Velodyne 雷达、
  其他品牌 IMU，见 `device/plugin/` 下其余尚未启用的插件）。
- 采集程序、存储层完全在我方掌控内（`recorder` 仓库源码级可读可改），这是
  hera 数据相对第三方数据的核心优势：出问题可以一路 debug 到驱动层。

---

## 2. 原始数据三件套（`data/raw/`，只读）

一次采集产出同一 `basename` 下的三个文件：

```
<basename>.hera           # Livox 点云 + IMU 的二进制容器
<basename>.insv            # Insta360 原始全景视频
<basename>.session.json    # 采集元信息
```

`session.json` 示例字段：

```json
{
  "record_start_host_ns": 1783862663484405760,
  "device_vendor_type": 1057,
  "hera_session_path": "/var/hera/data/<basename>.hera",
  "mp4_files": ["/var/hera/data/insta_insv/<basename>.insv"]
}
```

注意：`hera_session_path` / `mp4_files` 是 Jetson 端的绝对路径，跨机处理前
必须改写成本机路径（已知坑，见 CLAUDE.md §9）。

---

## 3. `.hera` 容器格式

### 3.1 文件头（`StorageDataHeader`，见 `recorder/storage/include/storage_data_header.hpp`）

- **版本**：V3（仅公共信息）/ **V4**（本项目使用，额外含 JSON `extra_info`）
- 头部字段：
  - `timestamp_start` / `timestamp_end` — 录制起止时间
  - `device_message_nums` / `device_data_sizes` / `device_names` — 每路设备的消息数、字节数、名称
  - `extra_info`（nlohmann::json）— 扩展元信息
  - `logs` — 录制期间的日志
  - `indices`（`StorageHeaderTimestampIndex[]`）— 时间戳→文件偏移量的索引，支持随机访问
- 最大头长度 `ReservedLength = 4 << 20`（4MB）

### 3.2 数据包类型（`device/plugin/*/plugin_data.hpp`）

容器内部按 TLV 方式逐包存储，每类包有唯一的 magic id：

| 包类型 | ID | 来源 | 关键字段 |
|---|---|---|---|
| `LivoxPacket` | `0x0521` | Mid-360 点云 | `timestamp_device_ns`, `timestamp_host_ns`, `handle`, `livox_dev_type`, `livox_data_type`, `dot_num`, `payload`（原始点数据） |
| `LivoxPacketUnSynced` | `0x0522` | 未同步点云变体 | 同上（结构预留） |
| `LivoxPacketLocalSynced` | `0x0523` | 本地同步点云变体 | 同上（结构预留） |
| `LivoxImuPacket` | `0x0524` | Mid-360 内置 IMU | `gyro_x/y/z`, `acc_x/y/z`, `timestamp_device_ns`, `timestamp_host_ns` |
| `InstaVideoPacket` | `0x0421` | Insta360 预览码流 | `stream_type`, `stream_index`, `payload`（H264/H265 字节流） |
| `InstaGyroPacket` | `0x0422` | Insta360 自带陀螺 | `sample_count`, `payload`（GyroData 数组），用于跨设备时间同步 |
| `InstaJpegFramePacket` | `0x0423` | 下载 MP4 后解码出的单帧 | `frame_index`, `width`, `height`, `payload`（JPEG 字节），由 `hera-storage-ingest-insta-video` 写入，时间戳 = `record_start_host_ns + frame PTS` |

**已知的时间戳陷阱**（CLAUDE.md §9，务必留意）：
- Insta 陀螺仪 CSV 里的 `timestamp_ns` 是 SDK 相对毫秒时钟，**不是**纳秒也不是绝对时间；
  绝对时间只认各数据包的 `timestamp_host_ns`。
- `.hera` 的 IMU 时间窗与 `.insv` 视频时间窗**不完全重合**——本项目两次采集都实测到这个
  系统性偏差（IMU 窗口比视频窗口窄约 1.6s 左右），判断为该数据链路的固有特征而非单次异常。

---

## 4. 传感器数据的具体规格（实测值）

### 4.1 点云（`points_sample.csv`：`timestamp_host_ns, x, y, z, reflectivity`）

- 坐标：米制，实测 `|x|max ≈ 23.75m`，`|y|max ≈ 19.39m`（室内尺度合理）
- `reflectivity`：Livox 原始反射率，取值 0-255（PLY 导出时字段名为 `intensity`），
  实测非零比例约 61.6%-64.9%
- 单帧密度较稀疏，是下游深度渲染空洞率高（8%-88%）的根源

### 4.2 IMU（`imu.csv`：`timestamp_device_ns, timestamp_host_ns, gx, gy, gz, ax, ay, az`）

- 陀螺：`gx/gy/gz`，单位 rad/s
- 加速度：`ax/ay/az`，**实测单位为 g（不是 m/s²）**——这是 CLAUDE.md 特别要求验证的项，
  两次采集分别实测静止段加速度模长均值 0.9995 / 0.9981，非常接近 1.0，确认单位为 g
- 采样率：实测 **201.6-201.8 Hz**（dt 中位数约 4.96ms），落在 195-205Hz 验收区间内
- 时长覆盖：IMU 覆盖时长与 `.hera` 声明时长差 0.7-0.9s（≤2s 阈值内通过）

### 4.3 全景视频（`.insv` → 拼接 → JPEG 帧）

- 原始为鱼眼双目视频，需软件拼接为等距柱状投影（ERP）全景后才可用于外参标定
- 无 GPU 时走软件拼接，耗时较长（后台运行）
- JPEG 导出只取静止段中点附近 1-2 帧，不做全量导出（`python3 -m hera.cli extract-jpegs`
  是已知的死命令，必须走 `HeraFile.save_jpegs` 的 Python API）

### 4.4 深度图（管线内派生数据，非传感器直出）

- Mid-360 本身不产生深度图，深度只能靠"点云 + GLIM 位姿 + 相机外参"做 z-buffer 投影渲染得到
- 实测空洞率 **8%-88%**（15 个面里 13 个超过 20% 降级阈值）——这是目前 Phase4
  实例提取效果不佳的主要瓶颈来源之一（已通过对照 4dkankan 干净深度数据验证，
  详见 `docs/third_party_reference_data_spec.md`）

---

## 5. 管线各阶段的衍生数据类型

| 阶段 | 数据类型 | 代表文件 | 当前状态 |
|---|---|---|---|
| Phase1 数据完整性 | IMU/点云 CSV、静止段检测可视化 | `work/phase1/imu.csv`, `points_sample.csv`, `static_segments.png` | ✅ 通过（阈值经实测数据修正后） |
| Phase2 GLIM 建图 | SLAM 轨迹 CSV（位置+四元数）、点云 PLY（`x y z intensity`） | `work/phase2/trajectory_v3.csv`, `deliverables/map_*.ply` | ⚠️ 回环漂移未完全达标，多轮调参中（v1→v3） |
| Phase3 全景对齐 | 帧-位姿绑定 JSON、相机-LiDAR 外参、投影叠加图 | `frames_with_pose.json`, `extrinsic.json`, `overlay_stop*.png` | ⚠️ 外参标定多轮迭代中 |
| Phase4 实例提取 | RGB/Depth/Pose/Intrinsics 上传包、ConceptGraphs 实例清单 | `upload_bundle/`, `instances_result/instances.json` | ⚠️ 受 Phase1-3 几何质量拖累，深度空洞率高 |
| Phase5 空间记忆入库 | SQLite 实体库、真值标注、四层审计报告 | `deliverables/home_memory.db` | 🔲 本地数据尚未达到可完整跑通的规模/质量（仅 3-4 房间，实例数不足以支撑 30-60 条真值） |

---

## 6. 采集规模现状

- 已实采 1 次正式数据（`20260712175715_fred_calib`，211.5s，3-4 个房间）+ 1 次早期
  标定/联调数据（`20260712142734_fred_office`，36s，已废弃）
- 检出候选静止段 7 个，其中 ≥10s 的有 5 个（对应 ROOM_COUNT）
- 样本量小，几何质量（点云密度、回环漂移、外参精度）是当前瓶颈，是全项目里
  **唯一能反映真实硬件+算法端到端表现**的数据源，但目前还不足以独立支撑
  Phase4/5 的完整统计意义评估
