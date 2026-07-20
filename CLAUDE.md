# CLAUDE.md — 空间记忆验证任务指挥文档

> 本文档是本工作目录的最高指令。你(Claude Code)的任务是按五个阶段执行并验证
> "hera 采集 → GLIM 建图 → 全景对齐 → 实例提取 → 空间记忆入库审计" 的完整管线。
> 数据背景、格式细节见 `docs/insta_livox_data_spec.md`,GLIM 用法见 `docs/README_cn.md`。

---

## 0. 总原则(违反任何一条即视为任务失败)

1. **门禁制**:每个阶段末尾有"自动验收"和"人工确认点"。自动验收不过 → 停止,
   写失败报告,不得进入下一阶段;人工确认点 → 生成证据文件后**停下来等我确认**,
   不要替我做视觉判断。
2. **不伪造成功**:任何命令失败、指标不达标,如实记录在 `STATUS.md`。
   宁可停在第 1 阶段,不要带着坏数据跑到第 5 阶段。
3. **凭证安全**:阿里云 AK/SK 只从 `secrets/aliyun.env` 读取(source 后用环境变量),
   永远不要把密钥内容打印到终端、日志或 STATUS.md,永远不要写进任何会提交的文件。
   `secrets/` 已在 `.gitignore`,确认它保持在里面。
4. **只读原始数据**:`data/raw/` 下的文件一律不修改、不移动、不重命名。
   所有中间产物写入 `work/`,最终交付物写入 `deliverables/`。
5. **每阶段完成后更新 `STATUS.md`**(格式见 §8),这是我了解进度的唯一渠道。

---

## 1. 目录约定与配置区

```
.
├── CLAUDE.md                  # 本文档
├── STATUS.md                  # 你维护的进度/结果报告
├── config.env                 # ← 任务配置(见下),执行前先读它
├── secrets/aliyun.env         # ALIBABA_CLOUD_ACCESS_KEY_ID / SECRET / OSS_BUCKET 等
├── docs/                      # insta_livox_data_spec.md / README_cn.md(GLIM)
├── data/raw/                  # 采集三件套: <basename>.hera / .insv / .session.json
├── tools/                     # hera-storage-extract-* 等二进制(amd64 构建产物)
├── spatial-memory/            # M0 空间记忆框架(含 demo_eval.py 四层审计)
├── scripts/                   # 你在任务中编写的所有脚本放这里,带注释可复跑
├── work/                      # 中间产物: work/phase1 ... work/phase5
└── deliverables/              # 最终交付: 地图PLY / 审计报告 / 演示问答记录
```

`config.env` 示例(以实际文件为准,缺失项先问我):

```bash
BASENAME=20260712xxxxxx_fred_home      # 本次会话 basename
MOUNTING_RPY="[0.0, 0.0, 0.0]"         # LiDAR 安装角,倒置装为 [180.0, 0.0, 0.0]
OSS_INPUT_PREFIX=hera-input/           # 上传触发工作流的 OSS 前缀
OSS_OUTPUT_PREFIX=results/             # 工作流产物前缀
GPU_ECS_HOST=                          # 第 4 阶段 GPU 机器 ssh 目标(留空则先问我)
ROOM_COUNT=4
```

---

## 2. 阶段 -1:环境自检(开工前必做)

- [ ] `data/raw/` 下三件套齐全且 basename 一致;记录各文件大小
- [ ] `tools/` 下 `hera-storage-extract-mid360`、`hera-storage-extract-insta`、
      `hera-storage-ingest-insta-video`、`multi_source_synchronizer` 存在且可执行
      (`chmod +x`,跑 `--help` 确认不缺动态库;缺库先 `ldd` 诊断并报告)
- [ ] `python3 -c "import numpy"` 通过;`spatial-memory/` 下 `python3 demo.py`
      和 `python3 demo_eval.py` 跑通(这是框架自身的回归基线)
- [ ] `ossutil`(或 aliyun CLI)可用,`source secrets/aliyun.env` 后能 `ls` 到 bucket
- [ ] 磁盘剩余空间 > 原始数据体积的 5 倍

全部通过 → 在 STATUS.md 记录环境快照,进入阶段 1。

---

## 3. 阶段 1:数据完整性验证(本地,~10 分钟)

**目标**:确认这次采集的数据值得往下处理。

执行(产物入 `work/phase1/`):

```bash
tools/hera-storage-extract-mid360 data/raw/$BASENAME.hera \
    --imu work/phase1/imu.csv --points work/phase1/points_sample.csv
```

写脚本 `scripts/p1_checks.py` 完成以下自动验收:

| # | 检查 | 通过标准 |
|---|---|---|
| 1 | IMU 速率 | `timestamp_host_ns` 差分中位数 → 195–205 Hz |
| 2 | IMU 时长 | 与 `.hera` 覆盖时长一致(±2s),且 ≥ 采集计划时长 |
| 3 | **加速度单位** | 自动检测静止段(gyro 模长 < 0.02 rad/s 持续 ≥5s,应有 ≥4 段对应四个房间停留),静止段 acc 模长均值 ∈ [0.95, 1.05] → 单位为 g。若 ∈ [9.3, 10.3] → 单位是 m/s²,**记录并在后续所有换算中适配,不要沉默** |
| 4 | 静止段结构 | 检出的静止段数 ≥ ROOM_COUNT(+起点回环段更好);每段 ≥10s;开头 30s 内存在高动态段(标定晃动) |
| 5 | 时间窗覆盖 | 读 `.session.json` 的 `record_start_hos­t_ns` + 用 ffprobe 读 `.insv` 时长,确认 `.hera` IMU 时间窗完全覆盖视频时间窗(数据规格 §6 的历史坑) |
| 6 | 点云合理性 | 抽样 xyz 均在米制室内尺度(|x|,|y| < 30m),reflectivity 非全零 |

**人工确认点 P1**:把静止段检测结果画成一张时序图(gyro 模长 vs 时间,标出静止段),
存 `work/phase1/static_segments.png`,停下等我确认段数与采集动作对得上。

---

## 4. 阶段 2:GLIM 云上建图 + 几何底座验收

**目标**:拿到度量精确的轨迹与点云,这是全流程唯一几何底座。

执行:

1. `source secrets/aliyun.env`,用 ossutil 上传 `.hera` 到 `$OSS_INPUT_PREFIX`
   触发既有工作流(参考 docs/README_cn.md 阿里云部分;注意工作流返回字段是
   `output_key` 不是 `map_key`)。若工作流不支持传 `mounting_rpy`,而
   `MOUNTING_RPY` 非零 → 停止并报告(这是已知的硬编码问题,不要绕过)。
2. 轮询/等待产物,下载地图压缩包与 PLY 到 `work/phase2/`,PLY 复制到
   `deliverables/map_$BASENAME.ply`。
3. 解包地图目录,确认结构符合 docs/README_cn.md:编号子图目录、每个含
   `data.txt`(T_world_origin + 逐帧位姿)。**写 `scripts/p2_load_traj.py`
   解析所有 data.txt,拼出全局逐帧轨迹 `work/phase2/trajectory.csv`
   (timestamp, x, y, z, qx, qy, qz, qw)——阶段 3 依赖它。**

自动验收(写 `scripts/p2_checks.py`,直接读 PLY,不依赖 GUI):

| # | 检查 | 通过标准 |
|---|---|---|
| 1 | Z 轴朝向 | 点云 Z 直方图:最大密度峰(地面)在 Z ∈ [-0.3, 0.3],次峰(天花板)在 Z ∈ [2.2, 3.2] |
| 2 | 规模合理 | 总点数 > 10⁵;XY 包围盒对得上一套住宅(每边 < 25m) |
| 3 | 轨迹完整 | 轨迹时长 ≈ 采集时长;无 >0.5s 的位姿空洞 |
| 4 | 回环漂移 | 轨迹首末位置差(若按 SOP 回到起点)< 0.15m,记录具体数值 |
| 5 | 墙体质量 | 对最大墙面做 RANSAC 平面拟合,内点 RMS < 0.03m |

**人工确认点 P2**:渲染三张图存 `work/phase2/`:俯视投影图(应能数出 4 个房间)、
侧视图(地面/天花板两条平线)、轨迹叠加俯视图。停下等我确认,并等我提供
2–3 个卷尺实测距离(门宽等),你在点云上量同名距离,误差 ≤3cm 记入 STATUS.md
——这是度量尺度的最终裁决。

---

## 5. 阶段 3:全景拼接 + 时间/空间对齐(本地,拼接较慢可后台挂)

**目标**:让每张全景帧获得"绝对时间 + GLIM 位姿",并粗标相机—LiDAR 外参。

执行:

1. 按数据规格 §5 生成路径改写后的 `work/phase3/insta_session_local.json`
   (改 `hera_session_path` / `mp4_files` 为本机路径),然后:
   ```bash
   tools/hera-storage-ingest-insta-video \
       --session work/phase3/insta_session_local.json \
       --output work/phase3/$BASENAME.pano.hera
   ```
   无 GPU 走软件拼接,慢是预期,后台运行并在 STATUS.md 记录进度。
2. 导出 JPEG:**不要用 `python3 -m hera.cli extract-jpegs`(死命令,规格 §3)**,
   直接调 `hera` Python API(`HeraFile` + `save_jpegs`)。只需导出静止段
   中点附近各 1–2 帧(用阶段 1 的静止段时间窗筛),不要全量导出。
3. 时间偏移:
   ```bash
   tools/multi_source_synchronizer data/raw/$BASENAME.hera data/raw/$BASENAME.hera \
       --OutputOffset work/phase3/offset.json
   ```
   记录 `offset_sec`(定义 t_mid = t_insta + offset)。同时用 JPEG 帧自带的
   `timestamp_host_ns` 交叉核对:两条路径给出的帧绝对时间差应 < 50ms,
   超出则报告"时钟漂移嫌疑",分段处理前先停下问我。
   若报 "Outbound error is too low" → 开头晃动不足,如实报告(时间对齐退化为
   仅依赖 JPEG host 时间戳,标注为降级模式继续)。
4. 位姿绑定:每张选定 ERP 帧,按其绝对时间在 `trajectory.csv` 上球面插值(姿态
   slerp、位置线性),得 `work/phase3/frames_with_pose.json`。
5. 外参粗标定:初值 = 我提供的手工量测(平移 xyz + 安装朝向,没给就停下问我)。
   写 `scripts/p3_project_check.py`:用 位姿×外参 把 GLIM 点云(按深度着色)
   投影到每张 ERP 帧上,输出叠加图 `work/phase3/overlay_room{N}.png`。

自动验收:所有选定帧成功绑定位姿;投影图中有效投影点数 > 10⁴/帧。

**人工确认点 P3**(全流程最重要的人工环节):我看 overlay 图中墙角、门框边缘的
错位,口头给你调整量(如"绕 Z 转 2 度、抬高 5cm"),你改外参重投影,迭代至我说
"对齐了"。最终外参写入 `work/phase3/extrinsic.json` 并在 STATUS.md 记录迭代次数。

---

## 6. 阶段 4:ConceptGraphs 实例提取(阿里云 GPU ECS)

**目标**:每个房间产出带 3D 位置 + 开放词汇特征的实例清单。

执行:

1. 本地预处理(写 `scripts/p4_prepare.py`):每张 ERP 帧切 4–5 面 cubemap 透视
   视图(跳过下半球,避开采集者身体),每面 90° FOV、分辨率 ≥ 640×640;
   为每面用 外参+位姿 从 GLIM 点云渲染深度图(z-buffer 点渲染 + 3×3 空洞填补),
   统计每面深度空洞率并记录(>20% 的面标记降级)。打包
   `work/phase4/upload_bundle/`(rgb/ depth/ poses.json intrinsics.json)。
2. GPU ECS:ssh 到 `$GPU_ECS_HOST`(未配置则停下问我是否现在购买按量实例)。
   在 ECS 上克隆并配置 ConceptGraphs 环境,跑通其 demo 后再喂我们的 bundle。
   环境安装期间遇到版本地狱,优先锁定其 README 指定版本,不要自行升级依赖。
   产物(实例列表:3D bbox/位置、类别、CLIP 特征、置信度)打包拉回
   `work/phase4/instances.json`。**任务完成后提醒我释放 ECS 实例。**
3. 本地验收(写 `scripts/p4_checks.py`):

| # | 检查 | 通过标准 |
|---|---|---|
| 1 | 实例数量 | 每房间 ≥5 个实例(家庭场景的下限合理性) |
| 2 | 位置合理 | 所有实例 3D 位置落在点云包围盒内,Z ∈ [-0.2, 2.5] |
| 3 | 特征完整 | 每实例 CLIP 特征维度一致且非零 |

**人工确认点 P4**:把实例 3D 框投影回对应 RGB 视图出标注图,我核对每房间的
识别质量并口头给你"漏检/误检清单"(这同时是阶段 5 真值标注的起点)。

---

## 7. 阶段 5:空间记忆入库 + 四层审计 + LLM 问答(本地,终点)

**目标**:真实数据流过 M0 框架,产出第一份真实审计报告和三句演示问答。

执行:

1. Place 定义:从阶段 2 俯视图读出四个房间的 AABB(我口头确认边界),写
   `scripts/p5_setup_places.py` 建 Place(单 submap,锚点为世界原点)。
2. 适配器 `scripts/p5_ingest_cg.py`:`instances.json` → 框架的
   `Detection`/`ObservationEvent`(按实例位置分配 place_id;view_center =
   该房间静止采集点,view_radius 覆盖全房间;embedding 直接用 CLIP 特征)。
   同时把框架的 `toy_embed` 换成 CLIP **文本**编码器(与实例特征同一模型,
   在本地 CPU 跑 open_clip 即可,慢无所谓),注意实体的 `embedding_model`
   字段同步改写。
3. `consolidator.ingest()` 入库,SQLite 落 `deliverables/home_memory.db`。
4. 真值标注:基于 P4 的核对清单 + 我补充的漏检物体,生成
   `work/phase5/ground_truth.json`(格式对齐 `spatial_memory/eval.py` 的
   `GroundTruthEntity`)。目标 30–60 个实体。
5. 跑四层审计(复用 `EntityValidator`),报告写
   `deliverables/audit_report.md`:L1 P/R/F1、L2 位姿 MAE、L3 归属、
   检索 Recall@3(查询集:我提供 20 条中文日常问法)。对照验收线
   (F1≥0.85 / MAE≤0.3m / 归属≥0.95 / Recall@3≥0.8)逐项标注达标与否,
   未达标项给出你的归因分析。
6. LLM 问答演示:写 `scripts/p5_llm_demo.py`,把 SpatialQuery 四类原语包成
   工具接入 Anthropic API(key 在 secrets/,同样不许打印),录制三句问答
   ("客厅有什么"/"遥控器在哪"/一条时序问题)到 `deliverables/qa_demo.md`。
   注:L4 时序审计需要第二次采集,本次只验证时序查询接口可用。

**人工确认点 P5**:audit_report + qa_demo 一起交我验收。

---

## 8. STATUS.md 格式约定

每阶段追加一节:

```markdown
## Phase N — <名称>   [状态: 进行中/等待人工确认/通过/失败]
- 开始/结束时间:
- 自动验收: 表格逐项 ✅/❌ + 实测数值
- 人工确认点: 证据文件路径 + 等待事项
- 异常与决策: 遇到什么、你怎么处理的、哪些需要我拍板
- 产物清单: 路径 + 一句话说明
```

## 9. 已知坑速查(来自 docs/,动手前先扫一遍)

- Insta 陀螺仪 CSV 的 `timestamp_ns` 是 SDK 相对毫秒时钟,**不是**纳秒也不是
  绝对时间;绝对时间只认各数据包的 `timestamp_host_ns`
- `python3 -m hera.cli extract-jpegs` 是死命令,必须直接调 Python API
- `session.json` 里是 Jetson 绝对路径,跨机必改副本
- Livox IMU 加速度单位按惯例是 g 不是 m/s²,阶段 1 第 3 项必须实测确认
- 云工作流 `run-slam` 输出字段名是 `output_key`;`mounting_rpy` 目前在 yml 里硬编码
- ROS bag 路线的 `/camera/insta360/imu` 话题时间戳异常会卡播放器——我们走
  `.hera` 离线路线,天然绕开,但若你出于任何原因转 bag,记得 `--topics` 过滤
- Docker 产物目录可能归 root,导出前 `chown` 或写用户可写路径
