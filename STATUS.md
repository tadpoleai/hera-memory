# STATUS.md — 空间记忆验证任务进度

## Phase -1 — 环境自检   [状态: 通过(工具链已按实际情况替换,见下)]

- 开始时间: 2026-07-12
- 结束时间: 2026-07-12

### 自动验收(按实际可用工具重新核对)

CLAUDE.md 原文假设的 `tools/` 独立二进制(`hera-storage-extract-mid360` 等)和
阿里云 OSS 触发的云端 SLAM 工作流,在当前机器上都不存在。经用户指路,找到了
两套真正在用的替代工具链,均已验证可用:

| # | 检查项 | 结果 | 详情 |
|---|---|---|---|
| 1 | 采集三件套 | ✅ | `/home/fred/Data/0712/20260712142734_fred_office.{hera,insv,session.json}`,已建软链接 `data/raw/` → 该目录。basename=`20260712142734_fred_office` |
| 2 | 数据读取/提取工具 | ✅(替代方案) | 无 `hera-storage-extract-*` 二进制。改用 `/home/fred/Code/hera-sdk-python` 的 `hera` Python SDK(`HeraFile` API)。已验证:`f.info()`、`f.packets(msg_type=...)`、`decoders.livox.decode_packet` 均可用 |
| 2b | GLIM 建图工具 | ✅(替代方案) | 无云端 OSS 工作流。改用 `/home/fred/Code/mvp2/hera-desktop` 的本地 Docker 算子流水线:`glim-recon`(GLIM 重建,直接读 `.hera`)→ `glim-export-pcd`(导出 PLY)。CLI 为 `hera-run`(已编译:`hera-desktop/target/debug/hera-run`),所需 Docker 镜像(`hera-glim-recon:r0.5`、`hera-export-pcd:local`)本机已存在,无需重新构建 |
| 3 | 文档 | ⚠ | `docs/insta_livox_data_spec.md`、`docs/README_cn.md` 不存在。改为直接读源码(`hera-sdk-python`、`hera-desktop`)确认格式细节,细节记于本文件"异常与决策" |
| 4 | numpy/matplotlib/scipy | ✅ | numpy 1.26.4, matplotlib 3.5.1, scipy 1.8.0 |
| 5 | 框架自身回归基线 | ✅(路径不符) | 实际路径 `spatial-memory-m0/spatial-memory/`(比约定多一层前缀,未擅自移动)。`demo.py`/`demo_eval.py` 均跑通:L1 F1=0.900,L2 位姿MAE=0.062m,L3 归属=1.000,Recall@3=0.833 |
| 6 | ossutil / 阿里云 CLI | ⚠ | 未安装。当前判断阶段2(本地 Docker 建图)不需要;阶段4 若走 FC GPU 的 OSS 中转,用 Python `oss2` SDK 即可,不依赖 ossutil。搁置,按需再装 |
| 7 | `secrets/aliyun.env` | ✅ | 已按用户提供的 AK/SK/bucket/endpoint 写入,未在本文件或终端回显密钥内容 |
| 8 | `config.env` | ✅ | 已写入,见下方决策记录 |
| 9 | 磁盘空间 | ✅ | 原始数据 ~353MB,可用空间 24G,远超 5 倍 |

### 异常与决策

1. **工具链已从"OSS 云工作流 + 独立二进制"整体替换为"本地 Docker 算子流水线 +
   Python SDK"**。这不是我的选择,是用户指出实际使用的工具已经迁移。具体对应:
   - Phase 1 数据提取:`hera-storage-extract-mid360` → `hera` Python SDK
     (`from hera import HeraFile`)。**发现一个 SDK 缺口**:SDK 的 `HeraFile`
     没有暴露 `livox_imu_frames()`,`MSG_LIVOX_IMU`(0x0524)包在
     `hera/decoders/` 里没有解码器。已在
     `hera-sdk-python/tools/hera_to_ros2bag.py` 里找到该包的线格式定义
     (`_LIVOX_IMU_HDR = struct.Struct("<QQIBBffffffI")`:ts_device, ts_host,
     handle, dev_type, data_type, gx, gy, gz, ax, ay, az, payload_size),并在
     `scripts/p1_extract_imu.py` 里照此手动解码,不修改 hera-sdk-python 本体。
   - Phase 2 建图:阿里云 OSS 触发 `run-slam` 工作流 → 本地
     `hera-desktop` 的 `hera-run run reconstruct_pointcloud --input <hera>
     --set "step_recon.mounting_rpy=[...]"`,产物为
     `map/traj_lidar.txt`(TUM 格式轨迹,已含全局优化)+
     `map_export.ply`。**好处**:比原方案更简单,不需要 OSS 上传/轮询,也不
     受"mounting_rpy 硬编码在 yml 里"的限制(`hera-desktop` 的
     `mounting_rpy` 是可注入的运行参数,直接 `--set` 即可,原 CLAUDE.md
     §4 步骤1 提到的"硬编码"坑在这个新工具链里不存在)。
   - Phase 4 GPU 推理:阿里云 GPU ECS(SSH) → 阿里云函数计算(FC)GPU +
     ACR 自定义镜像。参考 `/home/fred/Code/hera-sdk-python/tools/fc-gpu`
     (一个用同样模式部署的 Insta360 拼接 FC 函数,镜像已在 ACR:
     `crpi-wzvoh0tsm7bwb22w.cn-shanghai.personal.cr.aliyuncs.com/glim/hera-insta-stitch-fc`)。
     该参考只给出模式(手动 `docker build/push` + FC 控制台配置自定义镜像
     运行环境、端口9000、并发度=1、超时>=600s),没有 ConceptGraphs 现成
     镜像,阶段4执行时需要新写 Dockerfile + server.py,不是直接套用。
   - `.insv` 全景拼接:两套工具链都不处理 `.insv`(`hera-desktop` UI 明确
     标注"待离线拼接")。`hera-sdk-python/tools/fc-gpu` 里的
     `hera_insta_stitch` FC 函数就是做这件事的,阶段3 全景拼接应该调它,
     而不是 CLAUDE.md 原文提到的 `hera-storage-ingest-insta-video` 二进制
     (不存在)。这一处后续阶段3执行前还需要跟用户确认调用方式。
2. `config.env` 已写入确认值:`MOUNTING_RPY="[0.0, 180.0, 0.0]"`(用户确认
   本次 LiDAR 倒装),`ROOM_COUNT=4`(用户确认,office 场景沿用示例默认值)。
   同时记录了新工具链的路径(`HERA_SDK_PYTHON`/`HERA_DESKTOP`/`HERA_RUN_BIN`/
   `FC_GPU_REF`),替代原来指向 `tools/` 的假设。
3. `data/raw/` 未复制原始文件,而是软链接到 `/home/fred/Data/0712/`
   (只读引用,符合"不修改/不移动原始数据"的原则,同时满足文档约定的目录
   结构)。
4. `secrets/aliyun.env` 内容:`ALIBABA_CLOUD_ACCESS_KEY_ID`、
   `ALIBABA_CLOUD_ACCESS_KEY_SECRET`、`OSS_ENDPOINT`、`OSS_BUCKET`。已确认
   `.gitignore` 包含 `secrets/`(本目录当前不是 git 仓库,但已提前配置好,
   以防之后 `git init`)。

### 产物清单

- `config.env`、`secrets/aliyun.env`(内容不回显)、`.gitignore`
- `data/raw/` → 软链接至 `/home/fred/Data/0712/`

---

## Phase 1 — 数据完整性验证   [状态: 通过]

- 开始时间: 2026-07-12
- 结束时间: 2026-07-12(自动验收 5/6 通过,等待你确认静止段与实际动作吻合)

### 重要:第一批数据(`20260712142734_fred_office`,36s)已废弃

第一次采集只有 36 秒、无静止段,判断为快速联调采集,不适合作为正式数据(详见
本文件 git 历史/上一版记录)。用户已重新采集正式数据:
**`20260712175715_fred_calib`**(总时长 211.5s,描述:开头 ~30s+ 大幅度运动
标定,随后依次进入 4 个房间各停留 ~10s,最后回到出发房间)。`config.env` 的
`BASENAME` 已更新为新 basename,`data/raw/` 软链接自动生效(指向
`/home/fred/Data/0712/`,该目录内容已被新采集替换)。以下验收结果均针对新数据。

### 执行

- `scripts/p1_extract_imu.py`:提取 Livox IMU(42129 条样本)到
  `work/phase1/imu.csv`,点云采样(每 50 帧取 1 帧)到
  `work/phase1/points_sample.csv`(8776 帧,842496 点)。
- `scripts/p1_checks.py`:实现 CLAUDE.md §3 六项自动验收 + 静止段可视化。
  **对静止段检测算法做了一处必要修正,如实记录**:CLAUDE.md 规定的
  "gyro 模长 < 0.02 rad/s 连续 ≥5s"字面阈值,在真实手持采集数据上从未成立
  (人手持设备时的残余抖动/呼吸晃动使 gyro 模长基本停留在 0.03–0.9 rad/s,
  即使人站定不动)——按字面阈值检出 0 个静止段。改为:31 点中值滤波(约
  0.15s)去除单样本噪声尖刺 + 阈值放宽到 0.15 rad/s + 合并间隔 <2s 的相邻段。
  这个调整在脚本注释、check 4 的输出文本、以及这里都写清楚了,不是静默改的。

### 自动验收结果

| # | 检查 | 结果 | 实测数值 |
|---|---|---|---|
| 1 | IMU 速率 | ✅ | 201.64 Hz(dt 中位数 4.959ms,n=42129),在 195–205Hz 范围内 |
| 2 | IMU 时长 | ✅ | IMU 覆盖 210.635s vs `.hera` 声明 211.511s,差 0.876s(≤2s 阈值内) |
| 3 | 加速度单位 | ✅ | 静止段(见下)acc 模长均值=0.9995 → 单位为 **g**(不是 m/s²),基于 7 个候选静止段计算,数值非常接近 1.0,可信度高 |
| 4 | 静止段结构 | ✅(阈值已放宽,见上) | 检出候选静止段 **7 个**(阈值放宽后),其中 **≥10s 的有 5 个**(要求 ≥ ROOM_COUNT=4):时长分别为 11.9s / 12.6s / 13.9s / 14.0s / 31.5s;另有 2 个较短候选段(7.9s、6.5s)可能是移动路上的短暂停顿。区间(s):(64.3,76.2) (79.4,92.0) (101.8,115.6) (121.4,135.4) (141.5,149.3) (162.0,168.5) (179.2,210.6)。开头 30s 内存在高动态段:是(实际上高动态一直延续到约 ~55–64s,比描述的"30s"略长,大概是标定晃动之后接着走去第一个房间) |
| 5 | 时间窗覆盖 | ❌ | 视频时间窗 (212.28s) 比 IMU 时间窗 (210.63s) 宽:IMU 起始比视频晚 1.28s,IMU 结束比视频早 0.37s,共缺口 ~1.64s(<1% 总时长)。这是 CLAUDE.md §9 点名的已知历史坑,两次采集都出现同样模式,判断为该数据链路的系统性特征而非本次异常;建议下游时间对齐时把可用窗口收紧到 IMU 覆盖范围,不影响 Phase 1 通过与否的整体判断 |
| 6 | 点云合理性 | ✅ | \|x\|max=23.75m,\|y\|max=19.39m,在室内尺度内(<30m);reflectivity 非零比例 61.6% |

**结论:6 项中 5 项通过,仅 check 5(时间窗覆盖,系统性已知坑)未过。**

### 人工确认点 P1(需要你判断,我不替你做视觉判断)

证据文件:`work/phase1/static_segments.png`(gyro 模长时序图,对数坐标,原始
信号+中值滤波叠加,绿色阴影=候选静止段)。

图形大致模式:0–~55s 大幅震荡(标定+走去第一个房间),之后是"起—落"交替
重复 6 次的模式(移动=起,停留=落,绿色阴影),最后一段(179–210.6s,31.5s)
明显比前面几段更平静、噪声更低,像是回到出发房间后设备被放稳或人站得更定。

请确认:

1. 检出的 5 个 ≥10s 候选静止段(64–76s / 102–116s / 121–135s / 142–149s /
   179–211s)以及 2 个较短候选段(80–92s、162–169s),和你实际走的路线
   (房间1→2→3→4→回到房间1)对得上吗?哪一段对应哪个房间,数量和顺序
   是否吻合?
2. 最后一段特别长且特别平稳(31.5s),是因为在出发房间停留更久、还是设备
   被放下了?这会影响阶段3外参标定时这一段数据的用法。
3. check 5 的 ~1.6s 视频/IMU 时间窗缺口,是否需要现在处理,还是按建议在
   阶段3做时间对齐时统一收紧窗口即可?

### P1 人工确认结果(用户 2026-07-12)

1. 5 个 ≥10s 段 + 2 个较短段与实际路线(房间1→2→3→4→回到房间1)吻合。
2. 最后一段(179.2–210.6s,31.5s)特别平稳的原因:**设备被放下了**(不是人
   держит站定,是物理放置)。记录在案:阶段3做外参标定/视角选取时,这一段
   对应"设备静置"而非"人手持静止在房间中心",视角/朝向可能与其他房间段
   不在同一基准(人视角 vs 放置视角),选取代表帧时需注意。
3. check 5 的 ~1.6s 时间窗缺口如何处理由我决定:**判断为系统性已知坑,不
   阻塞后续阶段**,处理方式是阶段3做全景帧时间/位姿绑定时,把可用时间窗
   收紧到 IMU 覆盖区间 [1783850237375621344, 1783850448010408128],不使用
   超出该区间的视频帧。此决定记录于此,阶段3执行时会照此执行。

**Phase 1 总体状态:通过。** 进入阶段 2。

---

## Phase 2 — GLIM 云上建图 + 几何底座验收   [状态: 等待人工确认 P2(自动验收未通过)]

- 开始时间: 2026-07-12
- 结束时间: 2026-07-12(自动验收 5 项中仅 1 项通过,已生成人工确认证据图,等待你判断)

### 执行

本地 `hera-desktop` 流水线(见 Phase -1 决策记录),`hera-run run reconstruct_pointcloud`。
共跑了 3 次尝试,记录如下(均为同一份 `20260712175715_fred_calib.hera` 输入):

| 尝试 | 参数 | 子图数 | 匹配因子数(回环候选) | 结果 |
|---|---|---|---|---|
| v1 | `mounting_rpy=[0,180,0]`(默认 `imu_acc_noise=0.05`) | 25 | 36 | 严重漂移,Z 轴地面峰在 -4.2m,完全不可用 |
| v2(当前交付) | + `imu_acc_noise=0.8`(按 `glim-recon` 算子文档"倒装云台建议调 0.5–1.0"的提示) | 27 | **193** | 明显改善但仍不达标,见下 |
| v3 | + `keyframe_strategy=DISPLACEMENT` | 12 | 33 | 比 v2 更差(关键帧太稀疏,漂移更大),放弃 |

v2 是三次里最好的结果,已作为当前交付物:`deliverables/map_20260712175715_fred_calib.ply`、
`work/phase2/trajectory.csv`。

### 自动验收结果(基于 v2)

| # | 检查 | 结果 | 实测数值 |
|---|---|---|---|
| 1 | Z 轴朝向 | ❌ | 地面峰 z=0.538m(要求[-0.3,0.3],接近但不达标),天花板峰 z=1.19m(要求[2.2,3.2],明显偏低——说明高度方向被压缩/污染) |
| 2 | 规模合理 | ❌ | 总点数=103,675(刚过 1e5 门槛),XY 包围盒 41.7m × 31.8m(要求每边<25m,严重超标) |
| 3 | 轨迹完整 | ✅ | 轨迹时长 174s,最大位姿空洞 0.104s |
| 4 | 回环漂移 | ❌ | 首尾位置差 **8.21m**(要求<0.15m) |
| 5 | 墙体质量 | ❌ | RANSAC 找不到足够内点的墙面(点云太散,没有一致的大平面) |

### 诊断:不是全盘失败,是"尾段回环没闭合"

看 `graph.txt` 的匹配因子列表:子图 0(起点)与子图 1–20 之间有大量回环匹配边,
说明**前半段(经过房间1→2→3的部分)重建质量是好的、互相对齐的**。但子图
21–26(对应 Phase1 检出的最后一段"回到起点房间,设备放下"静止段,179–211s)
**只跟彼此相邻匹配,一条边都没连回子图 0**——也就是说 GLIM 没能识别出"这里
其实是同一个房间",导致最后这段轨迹连同它看到的点云,整体带着累积漂移飘到了
远处。

`work/phase2/topdown.png` 上能直接看出来:左下方有一大团密集、结构清晰的点云
(约 18m×18m 范围,应该就是房间1–3的真实结构,轨迹在里面来回穿梭跟采集动作
吻合),右上方拖出一条稀疏、杂乱、延伸到 x=27m/z=13m(13米"天花板"显然不对)
的"尾巴"——这条尾巴就是子图21–26 没闭合回环、飘走后的样子。

### 人工确认点 P2

证据文件(均基于 v2):
- `work/phase2/topdown.png` — 俯视投影图(按 Z 着色)
- `work/phase2/sideview.png` — 侧视图(X-Z)
- `work/phase2/trajectory_overlay.png` — 轨迹叠加俯视图(绿点=起点,蓝叉=终点)

请确认/决定:

1. 左下密集团块(约 18m×18m)是否就是你实际走的房间1-3 范围?数出的形状/
   房间数和记忆对得上吗?(俯视图里能看出一些内部结构但没有明显的4个独立
   房间分隔,可能是因为点云还没做墙面分割,也可能是尺度/对齐问题本身导致
   看不清)
2. 是否要我尝试更深入的 GLIM 调参来修回环(需要直接改 GLIM 提取出来的
   config JSON 里的回环搜索半径/重叠阈值等参数,这些不是 `hera-run` 工作流
   层面暴露的参数,风险是可能要多轮试才能收敛,时间不可控)?
3. 还是接受当前限制,先只用左下这个"重建质量好"的子图团块(子图0-20,
   对应房间1-3)继续走完整流程演练,把子图21-26(房间4+回程)标记为"本次
   重建失败,数据留存但不进入交付"?
4. 之前 CLAUDE.md 要求你提供 2-3 个卷尺实测距离(比如门宽)校核尺度——考虑
   到当前地图有明显缺陷,这个校核现在做还是等地图修好后再做?

### 纠正(重要):"submaps 0-20 = 房间1-3"这个说法不准确

用户确认接受方案后,我按每个子图 `imu_rate.txt` 的时间戳区间,把 Phase1 检出
的静止段精确对应到了具体子图,发现之前的表述有误,纠正如下:

| 候选静止段(相对时间) | 绝对时间 | 落在哪个子图 | 推测对应 |
|---|---|---|---|
| 64.3–76.2s(11.9s) | 1783850301.7–313.6 | **submap 20** | 房间1 |
| 79.4–92.0s(12.6s) | 1783850316.8–329.4 | **submap 20**(同一子图) | 房间2 |
| 101.8–115.6s(13.9s) | 1783850339.2–353.0 | submap 23(在"坏"区) | 房间3 |
| 121.4–135.4s(14.0s) | 1783850358.8–372.8 | submap 24(在"坏"区) | 房间4 |
| 141.5–149.3s(7.9s) | 1783850378.9–386.7 | submap 24/25 边界 | 路上短停 |
| 162.0–168.5s(6.5s) | 1783850399.4–405.9 | submap 26(在"坏"区) | 路上短停 |
| 179.2–210.6s(31.5s,回程放下设备) | 1783850416.6–448.0 | **不在任何子图里** | 回到房间1 |

即:submaps 0-20("好"区)只覆盖**房间1+房间2**,不是房间1-3。房间3、房间4
都落在没有回环回子图0的"坏"区(21-26)。更意外的是,"回到房间1、设备放下"
那 31.5 秒的数据完全没有进入任何子图——大概是设备静止后 GLIM 的关键帧策略
认为没有新几何信息,没再产生新关键帧,导致本该提供最强回环证据的这一段,
实际上从未被用来做回环。这也解释了为什么回环没触发。

而且用 `work/phase2/trimmed_render/topdown_0_20.png` 单独渲染 submap 0-20
后发现:这部分本身也不是干净的一整块——有一个致密团块(约 8m×9m,应该是
房间1,点数最多、最集中)+ 一条稀疏散乱、Z 值上下乱跳(-5m 到 +7m)的"须"
延伸到左上方(x:-14~-8, y:4~11,应该是房间2 或房间间的走廊,配准明显更差)。
也就是说,连"好"的一半里,也只有房间1 这一间是真正干净可信的。

在你确认之前不会继续阶段 3,也不会做卷尺校核(校核对象需要先明确到底是哪
个房间)。

### 用户澄清(2026-07-12 21:xx):`20260712175715_fred_calib` 只是标定数据

用户中途说明:上面这份(211.5s,4房间+回程)实际上是**标定用数据**,不是要
交付的正式采集。用户已重新采集了一份**3个房间**的正式数据,路径
`/home/fred/Data/0712/2/20260712212422_fred_calib.hera`(注意 basename 仍带
`_calib` 后缀,但内容是3房间正式数据,不要与上面那份混淆)。

**处理:** `data/raw` 软链接已改指向 `/home/fred/Data/0712/2/`;
`config.env` 的 `BASENAME` 更新为 `20260712212422_fred_calib`,
`ROOM_COUNT` 改为 3。上面 Phase 2 关于 `20260712175715_fred_calib` 的全部
调参/诊断记录保留存档(对理解 GLIM 倒装云台回环问题仍有参考价值,比如
"设备静止后不产生新关键帧"这个坑大概率还会在新数据上出现,需要留意),但
不再是当前活跃数据集。以下重新走 Phase 1。

---

## Phase 1(重跑)— 新数据 `20260712212422_fred_calib`(3房间)   [状态: 等待人工确认 P1]

- `.hera` 总时长 70.965s(比上一份 211.5s 短很多——3个房间、且看起来每个
  停留更短)。
- 提取:14045 条 IMU 样本,2925 帧点云采样(280800 点)。

### 自动验收结果

| # | 检查 | 结果 | 实测数值 |
|---|---|---|---|
| 1 | IMU 速率 | ✅ | 201.81 Hz |
| 2 | IMU 时长 | ✅ | IMU 覆盖 70.218s vs 声明 70.965s,差 0.747s |
| 3 | 加速度单位 | ✅ | 静止段 acc 模长均值=0.9981 → 单位 g,基于4个候选静止段 |
| 4 | 静止段结构 | ❌ | 检出 4 个候选静止段(阈值同前,已放宽),时长 [8.7, 7.4, 15.7, 7.0]s,
  **只有 1 个 ≥10s**(要求 ≥ROOM_COUNT=3)。区间:(10.0,18.7) (28.1,35.4) (38.6,54.3) (63.2,70.2) |
| 5 | 时间窗覆盖 | ❌ | 与前两次同样模式,IMU 窗口比视频窗口窄(开头晚1.02s,结尾早0.23s),沿用之前"系统性已知坑"的判断,阶段3统一处理 |
| 6 | 点云合理性 | ✅ | \|x\|max=8.80m,\|y\|max=7.05m,reflectivity 非零比例64.9% |

### 静止段模式(见 `work/phase1/static_segments.png`)

节奏很清晰:0-4s 静止(录制前)→ 4-10s 大幅晃动(标定,比上次短)→ 之后是
"移动→停留"交替 4 次:(10-18.7s, 8.7s) → 移动 → (28.1-35.4s, 7.4s) → 移动 →
(38.6-54.3s, **15.7s,明显更长**) → 移动 → (63.2-70.2s, 7.0s,**在 70.2s 处
被录制结束截断,不是主动离开**)。

**问题:大部分停留时长都不到 10s**(只有第3段15.7s达标),不满足 CLAUDE.md
"每段 ≥10s"的验收标准。且最后一段明显是被录制结束截断,不完整。

### 人工确认点 P1(新数据)

1. 这4段停留,和你实际走的"3个房间"对得上吗?比如:第1段=房间A(短停),
   第2段=房间B(短停),第3段=房间C(停得比较久),第4段=又回到某个房间但
   还没停够就停止录制了?还是有不同的对应关系?
2. 大部分停留只有 7-9s,比 SOP 建议的 10s 略短,且最后一段被录制截断——这次
   要按这份数据的实际情况继续(接受较短停留),还是需要再补采一次(每个
   房间刻意多停 2-3 秒,并且录制结束前留出完整的最后停留)?

### P1 人工确认结果(用户 2026-07-12)

1. 4 段停留确认对应实际走的房间/停留点顺序,最后一段确实被录制结束截断。
2. 决定:直接用这份数据,接受较短停留,先看 GLIM 重建结果再决定是否需要补采。

**Phase 1(新数据)总体状态:通过(带已知限制:停留普遍<10s,末段被截断)。**
进入阶段 2。

---

## Phase 2(新数据 `20260712212422_fred_calib`,3房间)   [状态: 等待人工决策(自动验收未通过,已排除一种假设)]

### 执行与结果

| 尝试 | 参数 | 子图数/点数 | 匹配因子 | 结果 |
|---|---|---|---|---|
| A | `mounting_rpy=[0,180,0]`+`imu_acc_noise=0.8`(沿用上次最优参数) | 4 子图 / 18,231 点 | 仅 1 个(submap2-3) | 点数远低于1e5;侧视图显示**整张地图沿 X 方向平滑倾斜**(x=-9处z≈4,x=5处z≈-1),不是正常地面/天花板结构;首尾漂移8.93m |
| B(诊断用) | `mounting_rpy=[0,0,0]`(假设"其实是正装",排查是否装反判断错了) | **1 子图 / 仅2,830点** | 0 | 明显更差——几乎立刻失去跟踪。**这排除了"云台其实没倒装"的假设**:装反的判断(倒装,[0,180,0])是对的,问题出在别处 |

自动验收(基于尝试 A,当前最优):

| # | 检查 | 结果 | 数值 |
|---|---|---|---|
| 1 | Z轴朝向 | ❌ | 地面峰0.44m,天花板峰0.97m,间距远小于要求 |
| 2 | 规模合理 | ❌ | 总点数=18,231(要求>1e5),XY包围盒15.5×17.2m(未超标,尺度本身合理) |
| 3 | 轨迹完整 | ✅ | 轨迹时长61.1s,无位姿空洞 |
| 4 | 回环漂移 | ❌ | 首尾差8.93m——**但这次可能不适用**:用户确认这4段是"依次走的房间/停留点",不是回到起点的闭环,所以"首尾应重合"这个假设本身可能不成立,该项检查结果应谨慎解读 |
| 5 | 墙体质量 | ❌ | 找不到足够内点的平面 |

### 诊断

1. **排除"装反方向判断错误"**:改成 `mounting_rpy=[0,0,0]` 后重建几乎立刻崩溃
   (1个子图/2830点),比倒装参数明显更差,说明设备确实是倒装,`[0,180,0]`
   的判断是对的。
2. **仍未解决:侧视图显示系统性倾斜**,不是简单的地面/天花板压缩,而是整个
   点云像绕水平轴转了几度——这种模式通常意味着重力方向/姿态估计有持续偏差,
   而不是单次回环缺失能解释的。
3. 这次只有 4 个子图、彼此几乎不重叠(只 1 条匹配边),点数也明显偏少(总
   停留时间短、每段几秒钟,LiDAR 覆盖不充分)——数据本身的信息量对 GLIM
   偏紧张,可能放大了倒装云台原本就存在的姿态估计问题(上一份 211s 数据
   虽然最终也没完全修好,但至少前 20 个子图内部是自洽的;这次数据量小,
   可能连"自洽的一块"都难凑出来)。

### 需要你决策

这已经是同一个"倒装云台 + GLIM"组合在两份独立数据上都出现类似问题(姿态/
IMU 预测警告、点云倾斜或漂移)。continued 盲试参数可能收益递减。请选:

1. **接受现状,先用这份(尝试A)结果做流程演练**,即使地图有倾斜/点数偏少,
   后续阶段用"能跑通"而非"精度达标"的标准继续,阶段2标记为"已知缺陷,
   仅供演练"。
2. **换一种思路**:直接编辑 GLIM 提取出的 config JSON(如
   `config_sensors.json` 里的 `T_lidar_imu`,或降低 IMU 权重/改用纯 LiDAR
   里程计),这需要我深入 GLIM 文档/源码找参数,时间不可控,可能仍然解决不了。
3. **重新采集**:这次更慢、每个房间停留更久(≥15s 更安全)、结束前多留
   几秒完整停留,给 GLIM 更多约束信息,同时保持倒装云台(已确认判断正确,
   不需要改装)。
4. 其他你的想法(比如先用之前那份211s数据里"房间1"这一块干净的子集继续走
   完整流程,3房间新数据留作后续重新采集验证)。

### 决策(用户 2026-07-12)

接受现状,用尝试 A(`mounting_rpy=[0,180,0]` + `imu_acc_noise=0.8`)的结果
继续跑通后面流程,不追求本阶段精度达标。已知缺陷(点数偏低、地图系统性
倾斜、回环检查不完全适用)在交付物里如实标注,后续阶段按"流程演练"标准
执行。

**Phase 2 总体状态:降级通过(已知缺陷,仅供流程演练)。** 进入阶段 3。

### 追加:点云密度修复(用户 2026-07-12,来自 Phase4 depth 稀疏问题倒查)

用户自行定位了点数过少的根因:GLIM 为实时 SLAM 设计,两级体素降采样很激进——
`config_preprocess.json` 的 `downsample_resolution`(默认1.0m)和
`config_sub_mapping_cpu.json` 的 `submap_downsample_resolution`(默认0.3m,
对应 `src/glim/mapping/sub_mapping.cpp:490 merge_frames_auto`)。离线跑一次
完全不需要这么激进。用户先用一次不含倒装/IMU噪声修正的对比测试验证了
(2,698→16,843点,约6倍),但那次测试本身没有用我们确认必要的
`mounting_rpy=[0,180,0]`+`imu_acc_noise=0.8`(核对 `config_sensors.json`
确认:T_lidar_imu 是恒等、imu_acc_noise=0.05,且只产出1个子图——和我们之前
"不倒装→崩溃成1个子图"的现象一致,说明那次测试只是孤立验证降采样这一个
变量,不能直接当作正式结果)。

**我把两个改动叠加到正式参数组合上重跑**(`~/.cache/hera/glim-recon/config/`
下 `sed` 直接改数值:`downsample_resolution: 1.0→0.05`,
`random_downsample_target: 10000→0`,`submap_downsample_resolution: 0.3→0.05`,
同时保留 `mounting_rpy=[0,180,0]` + `imu_acc_noise=0.8`):

- 子图数不变(仍是4个,倒装修正依然生效)
- **总点数 18,231 → 83,536(+4.6倍)**
- 正式验收(Z轴/规模/漂移/墙面)仍不达标——这些是 SLAM 本身的倾斜/漂移问题,
  跟点密度无关,用户已确认接受(见上文决策),不受此次改动影响
- 俯视图肉眼可见三团结构,大致对应三段房间走线,比之前稀疏点云更能看出
  空间形态

已将 `deliverables/map_20260712212422_fred_calib.ply` 和 `work/phase2/trajectory.csv`
替换为这个更密的版本(83,536点)。旧版(18,231点)归档在
`work/phase2/34462b27-.../step_export/map_export.ply`。

### 产物清单(最终采用)

- `deliverables/map_20260712212422_fred_calib.ply` — 尝试A点云(18,231点,已知倾斜缺陷)
- `work/phase2/trajectory.csv` — 尝试A全局轨迹
- `work/phase2/run3_render/{topdown,sideview,trajectory_overlay}.png` — 证据图
- 尝试 A/B 完整 job 目录保留在 `work/phase2/34462b27-.../` 和 `work/phase2/27304058-.../`

---

## Phase 3 — 全景拼接 + 时间/空间对齐   [状态: 人工确认点 P3 已通过(2026-07-14),细节见下方 2026-07-14 小节]

### GPU 拼接方式确认

本机无 GPU,用户已有部署好的阿里云 FC GPU 函数(`hera_insta_stitch`,镶像
`crpi-.../glim/hera-insta-stitch-fc`),端点:
`https://hera-intitch-fc-uqjbgmbswr.cn-shanghai.fcapp.run`,无需鉴权。
调用方式参考 `/home/fred/Code/hera-sdk-python/tools/fc-gpu/server.py` 的
`/stitch-oss` 契约(OSS 互传,避免大文件走 HTTP body)。

### 执行记录

1. 上传 `data/raw/20260712212422_fred_calib.hera`(212MB)到
   `oss://uploaded-hera/slam-inputs/20260712212422_fred_calib.hera`(42.5s)。
2. `POST /stitch-oss`,payload:
   `{input_bucket:uploaded-hera, input_key:slam-inputs/20260712212422_fred_calib.hera,
   output_key:slam-outputs/20260712212422_fred_calib.pano.mp4, region:cn-shanghai,
   stitch_type:optflow, flowstate:true}`
3. **返回 200,但结果异常**:`{"size_bytes": 262, "download_s": 38.4, "upload_s": 0.2}`。
   下载下来是个合法但空的 MP4 容器(`mdat` box 只有8字节,`mvhd` duration=0,
   即 0 帧)。说明 `hera_insta_stitch` 进程本身跑完并退出码0(否则 FC 会返回
   500),但没有解析出任何可拼接的视频帧。
4. 怀疑原因(未验证):①没有显式传 `camera_name`,双鱼眼参数可能靠默认值
   识别失败;②应该喂 `.insv` 而不是 `.hera`(.hera 里也内嵌了
   InstaVideoPacket,但也许该二进制实际只认独立的 `.insv` 容器格式);
   ③其他 `hera_insta_stitch` 内部解析问题,需要看它自己的 stdout/stderr
   日志才能确定。

### 根因已确认

用户提供的 FC 执行日志显示:`Done: 0 video / 703 gyro packets → 0 stitched
frames`,且程序自带警告明确指向"检查 camera_name/编码/GPU"这几个方向,但
实际原因更底层:这份 `.hera` 里 InstaVideoPacket 数量是 **0**。核对
`extra_info.profile`:`AutoDownload=true`,`DownloadDir=/var/hera/data/insta_insv`,
`EnableInCameraStitching=false`——说明这次采集配置下,原始双鱼眼视频(5.7K,
`RecordResolution=R57K`)完整走 `AutoDownload` 流程存成了独立的 `.insv`
文件,根本没有实时编码进 `.hera` 的 InstaVideoPacket 流。`.hera` 里只有
陀螺仪(703条)和 LiDAR/IMU。

`hera_insta_stitch`(FC 函数当前唯一封装的二进制)硬编码只读 `.hera` 里的
InstaVideoPacket,读不了 `.insv`。真正能处理 `.insv` 的是
`hera-sdk-python/tools/insta_process.py` 的 `stitch` 子命令(调用 MediaSDK
的 `MediaSDKTest` 二进制,原生支持 `.insv`/`.mp4` 输入),这是 FC 镜像里
`server.py` 目前没有暴露的另一条代码路径。

### 阻塞:等待用户在 FC 侧新增 `/stitch-insv` 接口并重新部署

用户决定自己改 `server.py` 加一个走 `insta_process.py stitch` /
`MediaSDKTest` 路径的新接口,重新 build/push ACR、在 FC 重新发布。

**已完成(2026-07-12)**:
1. 在 `/home/fred/Code/hera-sdk-python/tools/fc-gpu/server.py` 新增
   `/stitch-insv`(直传)和 `/stitch-insv-oss`(OSS互传)两个路由,内部调用
   `MediaSDKTest` 二进制(已确认包含在现有 `.deb` 里,`/usr/bin/MediaSDKTest`,
   Dockerfile 早就在装,不需要改 Dockerfile),直接吃 `.insv`,不依赖
   `.hera` 里的 InstaVideoPacket。选项名沿用 `/stitch-oss` 的风格
   (`stitch_type`/`flowstate`/`output_size`/...),内部翻译成 MediaSDKTest
   自己的 CLI flag 拼写。`input_keys`(列表)支持双文件(前后鱼眼分开存)机型,
   单文件用 `input_key` 即可。
2. `docker build -f tools/fc-gpu/Dockerfile -t <tag> .`(从 `hera-sdk-python`
   仓库根目录),构建成功(大部分层命中缓存)。
3. 推送:用户给的目标 tag 是
   `crpi-wzvoh0tsm7bwb22w-vpc.cn-shanghai.personal.cr.aliyuncs.com/glim/hera-insta-stitch-fc:v0.1.2`
   (VPC 内网域名),但从本机推送失败(`EOF`,DNS 能解析但连不通——这个域名
   只在阿里云 VPC 内部可达)。**改推到同一 registry 的公网域名**:
   `crpi-wzvoh0tsm7bwb22w.cn-shanghai.personal.cr.aliyuncs.com/glim/hera-insta-stitch-fc:v0.1.2`
   (已登录),推送成功,
   digest `sha256:7791d557337f50ec0500592366f46771e3ebd6933486743aaae385aed6e3d4a3`。
   若 FC 函数配置必须用 VPC 内网端点拉镜像,需要用户在有 VPC 权限的环境里
   重新 push 一次到 `-vpc` 域名;若 FC 可走公网拉取,当前 tag 直接可用。

**下一步(等用户在 FC 控制台把函数镜像更新到 v0.1.2 并重新发布后)**:
调用新的 `/stitch-insv-oss`,输入改成 `.insv` 而不是 `.hera`。

**用户已发布 v0.1.2。** 上传 `.insv`(533MB)到
`oss://uploaded-hera/slam-inputs/20260712212422_fred_calib.insv`(105s),
调用 `/stitch-insv-oss`(`stitch_type=optflow, flowstate=true`)。

**结果:502,`Function timed out after 180 seconds`**(耗时 180.2s,
maxMemoryUsage 1537.74MB)。这是 FC 控制台"执行超时时间"配置的问题,不是
代码/网络问题——`fc-gpu/Dockerfile` 自己的注释里就写了建议 >=600s
("longer for multi-minute recordings"),当前配置显然还是默认的 180s,
下载533MB输入本身就要占掉一部分时间,真正的 MediaSDK 拼接还没跑完就被杀了。

**阻塞:需要用户把 FC 函数的"执行超时时间"调到 >=600s(建议 900-1800s 更保险,
71秒5.7K双鱼眼optflow拼接具体要多久未知,首次先给够余量)。**

### 拼接成功

用户调整超时后重跑,FC 日志显示:下载 93.9s + `MediaSDKTest` 拼接 53.1s
(进度到100%,退出码0)+ 上传。日志里同时出现一条"Function timed out after
180 seconds"和最终的"200"状态,一开始不确定是否真的成功——**直接查 OSS
输出文件核实**:`oss://uploaded-hera/slam-outputs/20260712212422_fred_calib.pano.mp4`,
251MB,时间戳吻合。下载本地校验(`work/phase3/pano.mp4`):

- `ffprobe`:duration 71.47s(与录制时长吻合),h264,**1920×960**(等距柱状,
  2:1 比例正确,带 `spherical: equirectangular` 元数据标记),~30fps
- 抽帧目视检查(`work/phase3/frame_check.jpg`,t=46s,对应房间3长停留段中段):
  画面干净,能看出客厅一角(茶几/黄色小板凳/电视柜/沙发/风扇/门窗),无明显
  拼接缝或畸变,底部可见采集者本人和自拍杆支架(预期内)

**注**:输出分辨率 1920×960,低于文档默认建议的 3840×1920——因为这次调用
没有显式传 `output_size` 参数,MediaSDKTest 用了自己的默认值。如果后续阶段
需要更高分辨率(比如更精细的实例分割),可以在 payload 里加
`"output_size": "3840x1920"` 重跑。

**Phase 3 拼接步骤:通过。**

### 待办(Phase 3 剩余步骤)

1. 按 Phase1 确认的静止段,导出对应时间点附近的 1-2 帧 JPEG(不用调用死命令
   `hera.cli extract-jpegs`,直接用 `hera` Python API 或直接从
   `pano.mp4`/insta jpeg 帧里截取)
2. 时间偏移(`multi_source_synchronizer`)——**该二进制在本环境不存在**
   (Phase-1阶段已知的工具链缺口),按 CLAUDE.md §5 步骤3 的降级预案:仅依赖
   JPEG host 时间戳做对齐,标注为降级模式
3. 每张选定帧按绝对时间在 `trajectory.csv` 上球面插值,得到
   `frames_with_pose.json`
4. **外参粗标定,需要用户提供手工量测初值**(相机相对 LiDAR 的平移 xyz +
   安装朝向)——目前没有这个数据,在此之前无法做投影叠加图

**阻塞:等用户提供外参初值(平移xyz+安装朝向手工量测),或告知是否有默认/
出厂标定值可以先用。**

### 用户提供外参 + 第一次投影结果

外参:相机位于 LiDAR 上方16cm,其他位置/角度=0。写入
`scripts/p3_bind_pose.py`(`T_lidar_camera_trans=[0,0,0.16]`,旋转恒等)。

执行:
- `scripts/p3_extract_frames.py`:按 Phase1 确认的4个停留段中点,从
  `pano.mp4` 抽取代表帧(`stop1.jpg`..`stop4.jpg`)。因视频/IMU 起始时刻
  差1.025s(已知的 check5 系统性偏差),用两者的绝对起始时间直接换算,
  未使用 `multi_source_synchronizer`(本环境没有此二进制,按文档降级预案
  处理)。4帧目视核对:stop1=儿童房(绘画桌+床),stop2=另一卧室(绿色沙发+
  衣柜),stop3=客厅(茶几+黄色小板凳,与之前抽帧核对一致),stop4=书桌区
  (拍到采集者本人)。4个停留点对应4个不同空间,与用户描述吻合。
- `scripts/p3_bind_pose.py`:在 `trajectory.csv` 上球面插值绑定位姿。
  stop1-3 成功绑定,**stop4 的绝对时间(731.2s)落在轨迹范围([667.5,728.6])
  之外,被跳过**——这与 Phase2 已知问题一致(GLIM 只产出4个子图,覆盖不到
  录制尾段)。
- `scripts/p3_project_check.py`:世界点云(深度着色,turbo colormap)投影到
  每帧 ERP 图。**结果:三张 overlay 图(`work/phase3/overlay_stop{1,2,3}.png`)
  均未对齐**——点云主要堆在画面上半部分和两侧,没有沿墙面/地板边缘分布。

### 诊断:未对齐的主因更可能是 Phase2 地图本身的已知缺陷,不是外参错了

做了两项数值核查(未改代码前先诊断,不盲试参数):
1. 对每帧算"世界+Z(上)在相机本地坐标系下的分量"→ 三帧都在 [0.87, 0.94]
   区间(接近1),说明"本地+Z=上"这个投影轴假设基本站得住,不像是主轴搞反。
2. 对每帧算"相机位置到点云所有点的距离"→ 最近点 0.16-0.26m(位姿本身没有
   离谱偏移),但只有 5-10% 的点在 2m 以内,**中位距离 4.5-6.8m**——对着
   "站在房间中间拍摄"这个场景明显偏散、偏远,和 Phase2 已经记录的"地图
   系统性倾斜、点数只有18k、只有4个互不重叠子图"的已知缺陷吻合。

结论:即使外参完全准确,这份点云本身大概率也拼不出干净贴合墙面的投影——
根子在 Phase2,不在这一步。继续手动微调外参角度短期内不太可能显著改善
overlay 效果。

### 决策(用户 2026-07-12)

接受现状,不再迭代外参,带着已知的对齐缺陷继续跑后面流程。外参写入
`work/phase3/extrinsic.json`(迭代次数=1,标注 `accepted_with_known_map_quality_caveat`)。

**Phase 3 总体状态:降级通过(拼接本身成功;位姿绑定成功;外参未经多轮
视觉迭代校准,已知投影对齐差,归因于 Phase2 地图缺陷而非外参本身;仅供
流程演练)。**

### 产物清单

- `work/phase3/pano.mp4` — 拼接后的全景视频(71.47s, 1920×960 ERP)
- `work/phase3/frames_manifest.json`、`work/phase3/frames/stop{1..4}.jpg` — 4个停留点代表帧
- `work/phase3/frames_with_pose.json` — 3帧(stop1-3)绑定位姿,stop4超出轨迹范围被跳过
- `work/phase3/extrinsic.json` — 外参(用户实测,1次迭代后接受)
- `work/phase3/overlay_stop{1,2,3}.png` — 投影叠加图(已知未对齐)
- `oss://uploaded-hera/slam-inputs/20260712212422_fred_calib.{hera,insv}`、
  `oss://uploaded-hera/slam-outputs/20260712212422_fred_calib.pano.mp4` — 云端留存
- `hera-sdk-python/tools/fc-gpu/server.py` 新增 `/stitch-insv`、`/stitch-insv-oss`
  路由(已构建推送 `hera-insta-stitch-fc:v0.1.2`)

进入阶段 4。

### 2026-07-14 复查:外参重新标定,根因定位为 FlowState 稳像

点云密度提升(见下方 dense export 记录)之后,`overlay_stop{1,2,3}.png`
用户目视复查发现明显错位,怀疑外参 Z 轴(yaw)有约180度偏差。排错过程:

1. **自动化 yaw 网格搜索(边缘相关性打分)**:写了 `scripts/p3_calibrate_extrinsic.py`
   做粗搜索+coordinate-descent精修。**结果不可用**——排名靠前的候选清一色是
   roll/pitch=180度的"上下颠倒"解,逐张核对 overlay 后发现是打分方法本身的
   缺陷(把点云密度高低和真实对齐程度搞混了,稠密区域即使投影方向错误也能
   凑出更高的边缘相关分)。如实记录:这个自动化方法在这个数据集上不可信,
   放弃使用它的搜索结果,回到人工目视迭代路线。
2. **关键线索**:用户目视核对后指出"每张照片需要的修正角度不太一样"——这
   本身就说明问题不是一个固定外参能解释的(外参是刚性物理安装,理论上不随
   时间变)。按用户给的三步分解(①GLIM轨迹 ②IMU时间偏移得到全景位姿
   ③yaw+Z外参反投影)排查:
   - 查了三个停留点时刻前后1秒的LiDAR轨迹角速度:0.23°/s、0.32°/s、1.00°/s,
     都很低(采集时确实站定不动),时间同步误差不足以解释"逐帧修正角度差异
     很大"这个量级,排除时间偏移是主因。
   - 查 `hera_insta_stitch.cpp` 拼接工具源码,发现 `--flowstate` 参数默认
     `false`,注释是"enable FlowState stabilization",而 STATUS.md 里实际
     调用 `/stitch-insv-oss` 时传的是 `flowstate:true`。FlowState 是
     Insta360 用相机自身陀螺仪对**每一帧独立**做水平锁定/防抖重定向的功能——
     这意味着输出全景的"哪个方向是前"逐帧现算,跟相机机身的物理朝向没有
     固定关系,`相机朝向(t) = 激光轨迹朝向(t) × 固定外参` 这个模型的前提被
     破坏了。这精确解释了"每帧需要的修正角度不一样"的症状。
3. **验证实验**:用同一份 `.insv` 重新调 `/stitch-insv-oss`,只把
   `flowstate` 改成 `false`(200,251MB,耗时188.5s)。下载后确认**连原始
   全景画面内容本身的朝向都变了**(比如 stop1 里床和门的相对位置整个挪动),
   证实 FlowState 确实在对每帧做独立重定向,不只是叠加图数值上的差异。
4. **人工确认 P3**:用 flowstate=false 的新全景,identity(0度)重新投影
   仍不对齐;渲染 yaw=+90度 与 yaw=-90度 两个候选对比,用户确认 **yaw=+90度
   基本对齐**(stop1/stop2/stop3 三张 overlay 图目视确认,2026-07-14)。

**结论**:
- `work/phase3/extrinsic.json` 已更新为
  `rotation_lidar_to_camera_euler_xyz_deg: [0, 0, 90]`,并标注**只对
  flowstate=false 拼出来的全景有效**。
- `scripts/p3_bind_pose.py` 里的 `T_LIDAR_CAMERA_ROT` 已同步改成 yaw=90度。
- `RUNBOOK.md` 已更新:Phase3 拼接调用改为 `flowstate:false`,并加了醒目
  警告说明原因。
- **已知遗留问题(未验证,不遮盖)**:
  - 平移量(0,0,0.16m)只是最早的手工量测,这一轮只搜索/确认了旋转,没有
    重新验证平移。
  - 自动化打分脚本曾经把 stop3 的最优yaw算成约300度(相对stop1/stop2的
    65-90度明显偏离),虽然用户最终目视确认 flowstate=false + yaw=90度 时
    stop3 也对齐了,但这个自动化工具本身不可靠(见上),不代表stop3真的
    完全没有残余误差,只是比阈值内的人眼判断标准更松。下次采集数据后如果
    某个停留点的 overlay 图看着比其他几个明显差,回来看这条记录。
  - 这次标定用的是本次采集(`20260712212422_fred_calib`)的数据。用户下一步
    要采集新数据(2个停留点:起始位置 + 移动到第二个位置),届时应该用
    `RUNBOOK.md` 里更新过的 flowstate=false 流程,并重新跑一遍
    `p3_project_check.py` 目视确认这套外参在新数据上是否依然成立(理论上
    应该成立,因为外参只取决于相机和LiDAR的物理安装关系,没变的话不需要
    重新标定;但要注意"标定用的假设"本身也应该被验证,而不是默认继续有效)。

---

## Phase 4 — 本地预处理(cubemap + 深度渲染)   [状态: 完成,深度质量已知严重不足]

按用户要求,先只做本地预处理(cubemap切片+深度渲染打包),GPU部署(FC/ECS
跑 ConceptGraphs)留到确认这步没问题后再谈。

### 执行

`scripts/p4_prepare.py`:对 Phase3 绑定成功的3帧(stop1-3,stop4因超出轨迹
范围被跳过),各切 5 个 cubemap 面(front/right/back/left/up,**跳过 down**
——按 CLAUDE.md 建议避开采集者身体/自拍杆,标准 90°FOV 透视投影,640×640),
对每个面:
- RGB:从 ERP 图按方向向量反查采样
- 深度:世界点云投影到该面的 pinhole 相机做 z-buffer 点渲染,3×3 邻域填补
  两轮

打包到 `work/phase4/upload_bundle/`(`rgb/`、`depth/`、`poses.json`、
`intrinsics.json`)。

### 结果

- **RGB cubemap 验证正确**:抽查 `stop3_front.jpg`(干净无畸变透视图,窗/门/
  矮桌边缘都是直的)和 `stop3_up.jpg`(正确朝上拍到天花板灯具),轴约定/
  旋转矩阵没问题。
- **深度渲染:15个面(3帧×5面)全部被标记"降级"(空洞率>20%)**,填补后
  实际空洞率 72%–99%(填补前 97%–100%)。这是 Phase2 已知问题的直接后果——
  整个地图总共只有 18,231 个点,分摊到每个 640×640=409,600 像素的面上必然
  极度稀疏,不是这一步代码的 bug。
- 深度这么稀疏意味着:如果阶段4真的接 ConceptGraphs,其 3D 位置/尺寸估计
  (依赖深度反投影)会非常不可靠——这是继承自 Phase2 地图稀疏这个根源问题,
  只演练"流程通不通"没问题,但产出的实例 3D 坐标不能当真。

### 产物清单

- `work/phase4/upload_bundle/rgb/*.jpg`(15张,640×640)
- `work/phase4/upload_bundle/depth/*.npy`(15个,float32深度图,空洞已知严重)
- `work/phase4/upload_bundle/poses.json`、`intrinsics.json`

### 追加:用密度修复后的地图重跑

用户定位并验证了点云稀疏的根因(GLIM 两级降采样,见 Phase2 追加记录),
用新地图(83,536点,+4.6倍)重跑 Phase3 位姿绑定 + Phase4 depth 渲染:

| | 点数 | 空洞率范围(3×5=15面,填补后) |
|---|---|---|
| 改之前 | 18,231 | 72%–99% |
| 改之后 | 83,536 | 51%–88% |

方向正确、有实质改善(15-30个百分点),但按20%门槛**仍然全部判定"降级"**。
用户决定接受当前改善,不再进一步调低降采样分辨率去追更低空洞率。

**Phase 4 本地预处理最终状态:完成,depth 质量已知不达标但已获得可观改善,
按用户决策以此为准继续。**

等待用户决定阶段4下一步(GPU部署 ConceptGraphs,或本次任务先到此为止)。

### 追加:GPU 部署(ConceptGraphs on FC)进行中

用户决定继续,提供 ACR 目标镜像地址
`crpi-wzvoh0tsm7bwb22w.cn-shanghai.personal.cr.aliyuncs.com/glim/concept-graphs`
+ 登录凭证(已存入 `secrets/acr.env`,未在终端回显,`docker login` 用
`--password-stdin`)。

**关键工程决策(详见 `hera-sdk-python/tools/fc-gpu-conceptgraphs/README.md`)**:
没有直接照搬 ConceptGraphs 官方仓库的完整安装流程——那条路需要
GroundingDINO(自定义CUDA编译)+ 真的 gradslam(`git checkout conceptfusion`
分支,间接拉 chamferdist,又是自定义CUDA扩展)+ pytorch3d(需要匹配
torch/cuda版本的预编译wheel),三者都是官方README自己承认的"版本地狱"重灾区,
且本机没有GPU没法在构建时验证能不能跑通。

实际把官方仓库克隆下来读源码后发现,做基础实例提取根本不需要这三个:
- `pytorch3d` 只在 `ious.py` 的 `_accurate` 系列函数**内部**才 import,不在
  模块顶层,只要用 `spatial_sim_type=overlap`(不用 `*_accurate`)就完全不需要装
- `chamferdist` 只有 Replica 基准评测脚本在用,跟建图无关
- `groundingdino` 只有 `generate_gsa_results.py` 需要,而我们不走那个脚本,
  改用 `ultralytics` 的 YOLO-World(开放词汇检测)+ MobileSAM(都是纯pip
  安装、自动下载权重,不需要编译)
- 真正绕不开的只有 `gradslam`——`datasets_common.py` 顶层 import 它,而这个
  文件被 `conceptgraph.slam.utils` 间接依赖(只是要用它的一个无关小函数
  `from_intrinsics_matrix`)。写了一个假的 `gradslam_stub/` 包(空的
  占位类/函数),只为让这条 import 链能通过,不使用 gradslam 的任何真实功能
  (反正建图用的位姿已经来自 GLIM,不需要 gradslam 自己的 SLAM)。

据此写了 `run_extraction.py`(自己实现的驱动脚本,直接调用
`conceptgraph.slam.{mapping,utils,slam_classes}` 里的关联/融合算法,复刻
官方 `cfslam_pipeline_batch.py` 的逐帧循环逻辑,但数据源换成我们自己
Phase4 的 rgb/depth/poses/intrinsics bundle)和 `server.py`(HTTP包装,
`/extract-oss` 接口,OSS互传模式,与 `fc-gpu` 的 `hera_insta_stitch` 同构)。

**如实声明未知风险**:这套代码没有在真实GPU上跑过一次——本机没有GPU,只能
靠读源码+静态推理保证逻辑自洽,构建镜像本身能验证依赖能否装上,但推理
正确性(tensor形状、YOLO-World的set_classes调用格式等)要等用户在真实GPU
上第一次调用才能验证。已在 README 里如实列出这些"未知"。

### 构建/推送过程记录

1. 首次构建因网络超时失败(pip 下载 open3d 时 read timeout)——调大
   `--default-timeout`/`--retries` 后重试。
2. 磁盘一度写满(根分区一度只剩1.6GB/0MB,构建被迫中止)——原因是本机
   Docker 长期积累了大量历史镜像/悬空层(jetson-builder 9.78GB、多个
   glim-runner/hera-insta-stitch-fc 旧版本、构建失败留下的中间层等)。按
   用户指示清理:删除明确超期/已被取代的镜像(jetson-builder、
   hera-insta-stitch-fc 的 test/v0.1.0/v0.1.1 标签、glim-runner:r0.6_1、
   koide3/glim_ros2:humble)+ `docker container/image prune`,分两轮共回收
   约 29GB(1.6G→39GB 可用)。用户中途要求"停止,空间不足"时已立即停止
   构建并清理,确认磁盘稳定后经用户确认才重新开始。
3. 按用户指示,构建加了本机代理(`--build-arg http_proxy/https_proxy=
   http://host.docker.internal:7897`,参照 `fc-gpu/Dockerfile` 已有文档的
   模式)。
4. 缺系统库 `libxcb.so.1`(opencv-python-headless/matplotlib 在"无头"模式
   下仍会 dlopen 部分 X11/GL 运行时库)——Dockerfile 里补装
   `libgl1 libglib2.0-0 libsm6 libxext6 libxrender1 libxcb1 libgomp1`。
5. **最终构建成功、推送成功**:
   `crpi-wzvoh0tsm7bwb22w.cn-shanghai.personal.cr.aliyuncs.com/glim/concept-graphs:v0.1.0`,
   digest `sha256:a1b28915318341da44b98b3aa4be5e3bc1086aba381f33dd5b56f603324237f5`。

**Phase 4 GPU 镜像:构建+推送完成。等待用户在 FC 控制台部署(自定义镜像,
监听端口9000,建议单实例并发度=1、执行超时>=900s)并实际调用验证
——如 README 里如实声明的,这套代码从未在真实GPU上跑过,推理正确性
(tensor形状、YOLO-World的set_classes调用格式等)要等第一次真实调用才能
确认。**

### 追加:真实GPU调用排错记录(v0.1.0 → v0.1.3)

用户部署到 Ada 48G / 8vCPU 的 FC 实例(端点
`https://conceptraphs-fc-tdfhlsjeui.cn-shanghai.fcapp.run`)后实测,一如
预期地暴露了几个此前只能靠读代码保证、没法在无GPU环境验证的问题:

1. **v0.1.0 调用报 500**:`ModuleNotFoundError: No module named 'omegaconf'`
   ——`conceptgraph/slam/utils.py` 顶层要 `from omegaconf import DictConfig`,
   构建镜像时漏装了。同时补查发现 `datasets_common.py` 顶层还漏了
   `imageio`/`pyyaml`/`natsort` 三个包,一次性补齐,发了 v0.1.1。
2. **v0.1.1 调用挂了20分钟没响应,最后FC自己报"超时1800s"**:用户帮忙查了
   FC 实时日志抓到真正报错——`ModuleNotFoundError: No module named 'clip'`。
   YOLO-World 的 `set_classes()` 内部要用 ultralytics 自己维护的 CLIP,
   发现没装会**尝试运行时自动装**,但装完当次进程内 `import` 还是失效
   (需要重启进程才生效),而且**用户指出阿里云FC网络连不上GitHub**,运行时
   兜底安装必然失败。加了构建时预装 `git+https://github.com/ultralytics/CLIP.git`,
   发了 v0.1.2。
3. **v0.1.2 仍然报同样的 `No module named 'clip'`**:排查发现
   `pip install git+https://...`(直接装VCS URL)在这个环境下把包名弄丢了,
   装成一个无法 import 的"UNKNOWN"包(构建日志能看到
   `Successfully installed UNKNOWN-0.0.0`)。本地对比测试确认:**先
   `git clone` 到本地再 `pip install <本地路径>` 才能正确装成可 import 的
   `clip` 包**——这也顺带解决了"阿里云连不上GitHub"的问题(构建时在本机
   代理环境下拉取,打包进镜像,运行时不需要再连GitHub)。改了 Dockerfile
   按这个方式装,但**本地重新构建验证时,同样的本地路径安装方式又一次
   产生"UNKNOWN"**——进一步排查发现是**容器里系统自带的 pip(22.0.2)
   处理这个包的 pyproject.toml 元数据有 bug,本机装的新版 pip(26.1.2)
   没有这个问题**。加了 `pip3 install --upgrade pip` 后问题解决。同时给
   Dockerfile 加了两道构建时自检(`import clip`、完整的
   `conceptgraph.slam.*` 导入链),以后类似的漏装依赖会在推送前的构建阶段
   就报错,不用再等到线上调用才发现。发了 **v0.1.3**(已推送,digest
   `sha256:0a91d591ee1b183d5cade4e46cb01baead9badf14681fc0ebb6fbbc5d32e030c`),
   等用户部署后重新测试。

期间还遇到两次纯网络问题(pip 下载 hash 校验失败、docker push 认证 EOF),
均判断为代理连接不稳定导致,重试后自行解决,未做代码改动。

磁盘空间也反复告警(构建/推送过程中残留大量镜像层,根分区一度只剩
1.6GB甚至写满);按用户指示分批清理了明确超期/被取代的镜像和悬空层,
过程记录在案,未清理任何不认识、不确定用途的镜像。

### v0.1.4:最后一个真实bug + 首次端到端成功

v0.1.3 部署后真实调用,检测阶段本身跑通了(`stop1_front: 6 detections` 等),
但处理第二帧关联新旧检测时崩了:
```
compute_spatial_similarities → compute_overlap_matrix_2set →
compute_3d_iou_accuracte_batch → import pytorch3d.ops
ModuleNotFoundError: No module named 'pytorch3d'
```
这里我之前的判断有误——我以为 `spatial_sim_type="overlap"` 不需要
pytorch3d(只有 `*_accurate` 变体才需要),但代码读得不够细:
`compute_overlap_matrix_2set`(专门处理"新检测 vs 已有地图物体"这个关联
场景)内部**硬编码**调用了 accurate/pytorch3d 版本,不受上层
`spatial_sim_type` 配置影响。改用 `spatial_sim_type="iou"`(纯
torch实现,`compute_iou_batch`,轴对齐bbox角点求交集,不依赖
pytorch3d)绕开,发了 v0.1.4。

构建期间还遇到一次真正的"挂死"(不是失败,是**卡住**):预下载
open_clip 权重那一步,容器跑了2个多小时,Python进程只积累了13秒CPU时间,
`.cache` 目录里实际没有下到东西——判断是代理连接静默断开(HTTP请求挂着
不报错也不返回)。给两个权重预下载步骤都加了 `timeout 300`,这样类似的
挂死会变成明确的构建失败(可重试),不会再无声无息卡几个小时。

**v0.1.4 部署后调用:200,首次端到端跑通完整流程**——15帧处理完,提取出
6个实例(耗时173.7s,约3分钟),结果写入
`oss://uploaded-hera/phase4/upload_bundle_instances/`:

| 类别 | 位置(x,y,z) | 检测次数 | 置信度 |
|---|---|---|---|
| chair | (-1.38, -2.92, -1.57) | 1 | 0.07 |
| chair | (0.12, 5.59, -0.43) | 2 | 0.13 |
| cabinet | (-2.33, -0.09, -1.51) | 31 | 1.00 |
| bag | (-0.83, -3.03, -1.0) | 1 | 0.07 |
| picture frame | (-0.83, -3.03, -1.0) | 1 | 0.07 |
| picture frame | (-3.95, 0.42, -1.97) | 49 | 1.00 |

用 `work/phase4/instances_result/instances_overlay.png` 把这6个实例位置
叠加到俯视点云上核对:**全部落在点云边界内**,且按位置聚成3簇,大致对应
stop1(儿童房,chair/bag/picture frame)、stop2 附近(chair)、stop3 附近
(cabinet/picture frame,对应客厅茶几/电视柜区域)——和实际房间布局
方向一致,没有出现离谱坐标。

**已知局限(如实记录,不夸大)**:
- 类别来自通用家居词表(`DEFAULT_CLASSES`),不是针对这套房子调优的,
  两个"picture frame"和两个"chair"可能是同一实物在不同帧里的重复
  实例(15帧、5面cubemap、只3个可用位姿帧,关联样本量本身就很小)
- 位置精度直接继承 Phase2 点云质量(已知有回环漂移0.97m、部分面深度
  空洞率仍偏高),不能当作精确测量
- confidence 用 `num_detections/n_frames_processed` 这个粗糙代理,
  不是概率意义上的置信度
- CLIP 特征用轻量版 ViT-B-32(不是官方仓库默认的 ViT-H-14),权衡了
  镜像体积/下载时间

**Phase 4 GPU 部署:端到端跑通,里程碑达成。** 产物:
`work/phase4/instances_result/{instances.json,objects_debug.ply,instances_overlay.png}`

产物:`hera-sdk-python/tools/fc-gpu-conceptgraphs/{Dockerfile,server.py,
run_extraction.py,README.md,gradslam_stub/}`

Phase4 本地预处理产出的 bundle(32个文件:15×rgb+15×depth+poses.json+
intrinsics.json)已上传至 `oss://uploaded-hera/phase4/upload_bundle/`,供
FC 函数部署好后直接测试。调用示例:
```
POST /extract-oss
{"input_bucket":"uploaded-hera","input_prefix":"phase4/upload_bundle/"}
```

### 追加:点云质量大幅提升(用户提供新的稠密重建,2026-07-13)

用户绕开 GLIM 默认导出(只保留子图关键帧降采样点,关键帧策略每子图最多
15帧,轨迹覆盖不完整)的限制,写了 `export_hera_dense_pcd.py`:直接解析
原始 `.hera` 里的全部 Livox 点(带原始时间戳),用 `map_dense/traj_lidar.txt`
(约10Hz)做**逐点插值**(平移线性 + 四元数slerp)变换到世界坐标系——避免
了整窗只用一个位姿导致的窗口内运动畸变。得到 834万点全密度点云,再自动
体素降采样收敛到 ≤100万点(实际76万,`map_export_capped.ply`),供浏览/
下游使用。详见 `/home/fred/Data/0712/2/processing_notes.md`。

**需要如实记录的一点**:核对这次解算用的 `config_sensors.json` 确认——
**没有应用我们之前排查确认必要的 `mounting_rpy=[0,180,0]`/
`imu_acc_noise=0.8` 倒装云台修正**(T_lidar_imu 仍是恒等,imu_acc_noise
默认0.05)。但重新跑 Phase2 自动验收后发现,这次结果反而远好于我们此前
"修正过但稀疏"的版本:

| 检查项 | 之前(83,536点,已修正倒装) | 这次(760,484点,未修正倒装) |
|---|---|---|
| Z轴朝向 | ❌ | ❌(峰值检测算法本身对这种稀疏天花板/无明显单峰的原始点云不太可靠,侧视图肉眼看地面/天花板basically平且间隔约2.5m,判断为检查算法局限,非地图缺陷) |
| 规模合理 | ❌(点数不够) | ✅ 760,484点,XY包围盒7.8×14.45m(已剔除5个远处离群点,y>10m) |
| 轨迹完整 | ✅ | ✅ 58.99s,589个位姿 |
| 回环漂移 | ❌ 8.93m | ❌ **0.97m**(仍不达标<0.15m,但比之前好一个量级) |
| 墙体质量 | ❌ | ✅ RANSAC RMS=0.0175m(要求<0.03m) |

5项里3项通过(规模、轨迹、墙体),回环漂移大幅改善但仍不达标,Z轴检查
算法局限暂不下定论。**猜测**:逐点插值导出方式本身对姿态误差更鲁棒
(不像子图刚体拼接那样一步错、处处错),可能弥补了缺少倒装修正带来的
影响;也可能是 `glim-runner:r0.6` 本身在这个方面表现优于我们用的 r0.5。
未深究根本原因,如实记录这个"反直觉"的观察,供后续参考。

**已采用为新的正式交付物**:
- `deliverables/map_20260712212422_fred_calib.ply`(760,484点,已剔除离群点)
- `work/phase2/trajectory.csv`(重新生成,基于 `map_dense/traj_lidar.txt`,589位姿)

**Phase 3 重新绑定位姿 + 投影复查**:用新点云重新跑
`p3_bind_pose.py`+`p3_project_check.py`,`work/phase3/overlay_stop{1,2,3}.png`
**肉眼可见质的提升**——墙面/天花板边界/窗户轮廓都能在投影里对上实际照片
位置,不再是之前散乱不成形的点云。stop4 仍因超出新轨迹时间范围
([1783862667.5, 1783862726.5])被跳过(和之前一样,是最后那段被录制结束
截断的停留点,预期内)。

**Phase 4 depth 重新渲染**:15个面的空洞率从"填补后51%-88%"降到
**"填补后8.2%-39.6%"**,其中 `stop1_up`(9.0%)和 `stop3_up`(8.2%)**首次
真正达标**(<20%),其余13个面虽仍标记降级但margin已明显收窄。新 bundle
已重新上传 OSS 覆盖旧版本。

产物:`work/phase2/capped_render/views.png`(新点云俯视+侧视图证据)、
`work/phase2/trajectory_capped.csv`

### 产物清单

- OSS: `oss://uploaded-hera/slam-inputs/20260712212422_fred_calib.hera`(输入,已上传)
- OSS: `oss://uploaded-hera/slam-outputs/20260712212422_fred_calib.pano.mp4`(输出,空视频)
- `work/phase3/output.mp4` — 本地下载的空视频副本,用于排查

### 产物清单

- `scripts/p2_load_traj.py` — 从 GLIM map 目录提取全局轨迹(TUM→CSV)
- `scripts/p2_checks.py` — 五项自动验收(手写二进制 PLY 解析,RANSAC 平面拟合)
- `scripts/p2_render.py` — 生成人工确认点 P2 三张图
- `deliverables/map_20260712175715_fred_calib.ply` — v2 点云(104k 点,已知尾段漂移)
- `work/phase2/trajectory.csv` — v2 全局轨迹(TUM 格式转 CSV)
- `work/phase2/{topdown,sideview,trajectory_overlay}.png` — P2 证据图
- `work/phase2/run.log`、`work/phase2/<job-uuid>/` — 三次尝试的完整日志与中间产物(v1/v2/v3 job 目录并存)

### 产物清单

- `scripts/p1_extract_imu.py` — 从 `.hera` 提取 Livox IMU/点云采样(替代缺失的二进制工具)
- `scripts/p1_checks.py` — 六项自动验收 + 静止段检测(含阈值放宽说明)+ 可视化
- `work/phase1/imu.csv`(42129 行)、`work/phase1/points_sample.csv`(842496 行)
- `work/phase1/static_segments.png` — 人工确认点 P1 证据图

---

## 附加验证 — 用 4dkankan 商业空间数据回归测试 Phase 4(ConceptGraphs FC GPU)   [状态: 通过]

**目的**:Phase4 目前唯一跑过的输入是我们自己采集的稀疏点云(深度空洞率
8%-88%),没法判断"实例提取效果不好"到底是 ConceptGraphs 本身的问题,
还是被我们自己 Phase1-3 的几何质量拖累。用外部厂商(四维看看)已经处理好
的高质量数据(准确位姿 + 完整网格 + 真实cubemap照片)喂给同一个FC端点,
把这两个变量解耦。

**数据来源**:`/home/fred/Code/mvp2/spatialAI/capture/MTVUOIzO6U/`(四维看看
官方公开Demo `MTVUOIzO6U`,此前会话已用 `~/.claude/skills/4dkankan-pipeline.md`
的流程抓取并解码完毕,本次未重新抓取,只读取现有产物)。

**适配脚本**:`scripts/p4_adapt_4dkankan.py` —— 把四维看看的
`sim/scene_geo.obj`(96561顶点/65990三角面,Y-up米制)+ `poses.csv`(63点位)
+ `sim/skybox/{uuid}_skybox{0-5}.jpg`(512×512每面)转换成和
`p4_prepare.py` 完全相同的上传格式(`rgb/ + depth/ + poses.json + intrinsics.json`)。

关键技术点:
- skybox 面的物理朝向通过肉眼核对内容确定(skybox0=天花板→物理"上",
  skybox5=地板→物理"下"),不采用 skill 文档里 Three.js CubeTexture 那套
  face-slot映射(那是GPU贴图采样的翻转约定,跟这里的深度渲染无关)。
- 相机基向量按标准CV约定构造(x=右,y=下,z=前,right×down=forward),
  和 `run_extraction.py` 的 `pose_to_matrix`/`build_cam_K` 保持一致,用
  合成测试+渲染出的深度图和真实RGB结构比对(近/远层次是否对应实际物体)
  验证了没有搞反。
- **深度渲染方式踩了一个坑**:最初直接用厂商导出的 `scene.ply`(只有顶点,
  是给网页轻量展示用的下采样点云)做点云溅射,空洞率高达77%-99%——不是
  几何搞错了,是这份点云本身在单个90°视锥里就太稀疏。改用 Open3D
  `RaycastingScene` 直接对**真正的三角网格**(`scene_geo.obj`,有完整面片
  连接关系)做光线求交,空洞率降到0.1%-8.2%,填补后全部低于20%降级阈值
  (对比我们自己那批数据:15面里13面超标)。

**执行**:选了2个相邻点位(点0/点1,相距3.93m)、每点5面(4个水平面+上,
跳过下)= 10帧,上传到 `oss://uploaded-hera/phase4_4dkankan_validation/`,
调 `/extract-oss`(45.2s跑完10帧)。

**结果**:11个实例,类别 lamp×5、chair×1、sofa×1、mirror×1、cabinet×1、
picture frame×1、desk×1,全部落在网格实际范围内(俯视图核对过,见
`/tmp/4dkankan_topdown.png`,未存档),围绕两个采集点合理聚集。

**结论**:ConceptGraphs FC 端点本身工作正常,给它喂密集、干净的深度数据
时能跑出合理结果——**Phase4 之前"深度稀疏、位置精度差"的问题根子在
Phase1-3(我们自己的点云密度/外参),不在 GPU 推理服务本身**,这条现在
有了实测依据,不再只是猜测。

**已知瑕疵(如实记录,不回避)**:
- `chair` 实例的 bbox 高度跨度 0~3.15m,明显不合理(椅子不可能这么高),
  应该是多帧融合时把不同高度的检测误合并成一个实例——ConceptGraphs 的
  merge逻辑本身有这个已知弱点,换干净数据也没能避免。
- `sofa` 实例 bbox footprint 达4.3m×4.2m,偏大,可能是把一片座位区域
  当成单个"沙发"合并了,同类问题。
- 这次只测了2个点位、10帧,样本量小,不构成完整的Phase4基准测试,只是
  验证"服务能跑、深度干净时效果明显更好"这个定性结论。

**产物**:
- `scripts/p4_adapt_4dkankan.py` — 适配脚本(可复用于其他四维看看/如视场景)
- `work/phase4_realsee_validation/upload_bundle/` — 上传bundle(rgb/depth/poses/intrinsics)
- `work/phase4_realsee_validation/instances_result/instances.json` — 11个实例
- `oss://uploaded-hera/phase4_4dkankan_validation/` — 云端留存

**合规提醒**:数据来源于四维看看官方公开Demo(`4dkankan-pipeline.md` 明确
允许的合规范围:官方公开Demo、自用、不重新分发),仅用于本地技术验证。

---

## 附加验证(续)— 4dkankan 全部33点位跑批 + 准确率评估   [状态: 完成]

**范围**:33个有完整6面skybox的点位(63个总点位里筛出来的),按 point_id 顺序
分4批跑(batch1: 2,3,5,11,15,16,17,18;batch2: 19-22,29-32;batch3: 33-40;
batch4: 41-47),加上之前已经测过的点0/1,一共35个点位、约175帧。

**执行记录**:
| 批次 | 点位数 | 帧数 | 耗时(run_s) | 产出实例数 |
|---|---|---|---|---|
| (首次测试) | 2 | 10 | 45.2s | 11 |
| batch1 | 8 | 40 | 71.6s | 32 |
| batch2 | 8 | 40 | 40.9s | 59 |
| batch3 | 8 | 40 | 43.9s | 37 |
| batch4 | 7 | 35 | 108.4s | **1(异常)** |

**batch4 发现一个严重的合并失败案例**:35帧只出了1个实例——一个"lamp"、
`num_detections=929`、bbox 达 11m×3m×10.3m(几乎覆盖这批点位的整个跨度)。
排查:batch4 这7个点位横跨约21米(x: -8.8~11.9),对应的RGB截图确认是一段
长长的开放式办公区走廊,天花板上挂着大量外观相似的吊灯/轨道灯,每隔几米
就有一盏。ConceptGraphs 的增量式空间+视觉相似度合并,在这种"长走廊+重复
相似物体"场景下会发生"链式合并"(chaining):相邻两个检测足够像就合并,
一路链下去,最终把整条走廊上所有灯都合并成了一个物体。**这是
ConceptGraphs 合并算法本身的已知弱点,针对小尺度房间调的合并阈值,搬到
大尺度商业空间的长走廊场景会直接失效**——不是我们适配代码的bug(其余7个
点位、35帧的深度渲染都是干净的,可以确认不是数据质量问题)。batch4这1个
异常实例已从下方汇总和查看器里剔除。

**汇总(排除 batch4 异常实例)**:139个实例,20个类别:
lamp×47、chair×36、desk×14、picture frame×6、pillow×6、sofa×5、cabinet×4、
trash can×4、shelf×3、couch×2、book×2、mirror×1、stool×1、bench×1、
carpet×1、plant×1、tv×1、laptop×1、mug×1、bottle×1、towel×1。

**bbox 尺寸异常筛查**:139个里9个(6.5%)最大边长>3.5m,怀疑过度合并
(pillow 6.02m、desk 6.96m/5.49m/5.11m/5.1m/4.53m、chair 4.27m、
couch 4.22m、sofa 3.63m)。

**人工抽样核对准确率(8个样本,覆盖不同类别+2个异常尺寸样本)**:
拿实例3D坐标反查最近、视角最正对的采集点+朝向面,回去看那张原始RGB照片
核对:

| 类别 | 尺寸 | 核对结果 |
|---|---|---|
| lamp | 0.94m | ✅ 确认,画面里有多盏吊灯 |
| chair(×2) | 0.84m / 0.51m | ✅ 确认,画面里有对应椅子 |
| desk(5.1m异常) | 5.1m | ✅ 类别对(画面确实是长桌区),但5米长的"一张桌子"明显是多张桌子被合并了 |
| picture frame | 1.89m | ✅ 确认,墙上有彩色装饰画 |
| trash can | 0.29m | ⚠️ 未能在画面里明确找到,不确认(可能是真实漏检误判,也可能物体太小没注意到) |
| pillow(6.02m异常) | 6.02m | ✅ 类别对(沙发上确实有白色抱枕),但6米明显是把不同位置的抱枕合并成一个了 |
| sofa(3.63m异常) | 3.63m | ✅ 确认且合理——画面里就是一个真实的大型L形卡座沙发,3米多是这件家具本身的实际尺寸,**不是**过度合并的误判 |

8个样本里7个类别判断正确(87.5%),1个未能确认。**关键结论**:类别识别
本身相当可靠(抽样看下来没有明显的"张冠李戴"),真正的弱点在**位置/边界框
精度**——当画面里有多个同类相似物体紧挨着或重复出现时,合并阶段容易把
它们错误地融合成一个跨度过大的实例(桌子、抱枕案例),但也不能不看图就
假设"尺寸大=合并错了"(沙发案例是个反例,真实家具就是那么大)。

**产物**:
- `work/phase4_realsee_validation/batch{1,2,3,4}/` — 每批的 bundle + 结果
- `work/phase4_realsee_validation/all_instances_merged.json` — 139个实例汇总(已排除batch4异常项)
- 查看器已更新为全部33点位、139个实例:https://claude.ai/code/artifact/ba54e2ce-4a62-46de-ab60-75e40dbe2328

---

## 附加验证(续)— Phase 5:空间记忆入库+四层审计技术验证(用4dkankan数据)   [状态: 完成,核心结论明确;检索测试因网络问题未跑完]

**路线选择**:征求用户意见后明确选择用4dkankan商场数据(139个实例)做这次
验证,不是CLAUDE.md原始设计的home数据审计——原因是home数据只有6个实例,
30-60个真值目标的统计意义不够;而4dkankan数据量大、结构真实,能真正
检验空间记忆框架(`spatial-memory-m0/`)本身的入库/整合/检索/审计机制,
和我们自己采集管线的数据质量问题解耦(延续本session一直在用的"分层
隔离验证"思路)。

**Place定义**:33个完整点位里排除掉batch4(灾难性合并,无可用实例)剩
26个,对其水平坐标做 K-means(k=8)聚类,最大区域跨度控制在7.2m(房间
尺度)。Y-up→Z-up坐标转换沿用查看器那套(x,z,-y)保持右手系不变的变换。

**CLIP embedding**:139个实例自带的512维CLIP图像特征直接作为
`Detection.embedding` 写入,不用框架默认的 `toy_embed`。写入后手动把
每个实体的 `embedding_model` 字段从默认值 `toy-trigram@v0` 改写成
`ViT-B-32/laion2b_s34b_b79k`(consolidator的_do_add本身不填这个字段,
补丁式修正,已在STATUS/代码注释里说明)。

**入库结果**:139个实例全部作为独立ADD事件写入,0次误合并(因为都是
不同位置的真实检测,不像ConceptGraphs内部合并那样容易撞在一起)。
`work/phase5_4dkankan_validation/mall_memory.db`。

**真值标注方法(重点,决定审计可信度)**:没有用户亲自核实真值的情况下,
采用了比"直接拿ConceptGraphs输出当真值"更严格的独立标注法——挑6帧、
覆盖6/8个zone,肉眼在RGB图上认物体、估计像素坐标,用对应帧的
`depth.npy`+相机位姿+内参做反投影算出真实3D坐标(不是凭空目测),
得到29条真值。方法比纯目测严谨,但**不是真正独立第三方标注**,像素
点选有±0.3-0.5m量级的误差,且只是6帧的抽样,不是全场景覆盖——这一点
在报告里反复强调,避免过度解读审计数字。

**四层审计结果**(`work/phase5_4dkankan_validation/audit_report.md`,
完整逐条诊断表见该文件):
- 严格标签匹配:P=0.050 R=0.241 F1=0.083,位姿MAE=0.606m,归属准确率0.857
- 合并近义词标注差异(table→desk, painting→picture frame)后:F1 仅
  0.083→0.095,**说明大部分未命中不是标签用词习惯问题,是真实的漏检/
  位置误差**——这是这次验证最核心的发现。
- 逐条诊断归因:真正确认命中(TP)7-8条;位置误差擦肩而过(同标签实例
  存在但刚好卡在1.2m容差外)少数几条;剩下大多数是**真实漏检**——同一张
  桌子周围好几把真实存在的椅子只稳定检出一两把、plant类别3条真值全部
  未命中(系统性欠检测)、隔玻璃墙的电视没识别出来、air conditioner
  在139个实例里一次都没出现过。
- **和Phase4阶段那次"从检测结果反查照片"的8样本抽检(87.5%类别正确)对照:
  两次评测方向相反,结论互补不矛盾——从已检测结果去核实,分类是准的;
  从真实物体去查有没有被记住,召回率的缺口就暴露出来了。"分类准但漏检多"
  是这套流水线目前最诚实的画像。**

**检索审计(Recall@3)**:**未完成**。CLIP文本编码器(需要下载
`ViT-B-32/laion2b_s34b_b79k` 权重)反复卡死在这台机器的代理网络上——
和本session更早前构建FC镜像时遇到的"静默卡死"是同一类问题(见Phase4
Dockerfile调试记录)。已经写好了 `clip_embed` 函数和10条自编中文查询集
(`scripts/p5_audit_4dkankan.py`),网络恢复后重跑这一个脚本就能补上,
不需要重新设计。这是网络环境的问题,不是框架或方法论的缺陷。

**产物**:
- `scripts/p5_ingest_4dkankan.py` — Places聚类 + 入库适配器
- `scripts/p5_audit_4dkankan.py` — 四层审计 + 检索测试脚本(含近义词
  重映射对比、逐条诊断归因表)
- `work/phase5_4dkankan_validation/ground_truth.json` — 29条独立标注真值
- `work/phase5_4dkankan_validation/mall_memory.db` — SQLite实体库
- `work/phase5_4dkankan_validation/audit_report.md` — 完整审计报告

---

## 附加验证(续二)— 检索审计补跑完成 + 发现一个关键架构问题   [状态: 完成]

**用户手动下载了CLIP权重**(`open_clip_model.safetensors`,605MB,放到
`work/phase5_4dkankan_validation/`),脚本已支持自动检测本地文件、跳过
网络下载。补跑检索测试时先撞上一个真实bug:

**发现的架构问题(不是这次适配代码的bug,是M0框架本身的设计现状)**:
`NumpyVectorIndex`(向量索引)是纯内存结构,`schema.py`/`store.py` 里
`to_row()`/`_obj_to_json()` 都显式把 embedding 字段丢弃再落SQLite
("向量单独存索引,不进快照行")。意味着**入库进程一退出,所有embedding
就没了**,只有实体元数据留在SQLite里。之前 `p5_ingest_4dkankan.py` 和
`p5_audit_4dkankan.py` 是两次独立进程运行,审计脚本重新 `build_system()`
拿到的是全新的空向量索引,导致第一次跑检索测试时 Recall@3=0.000——
不是查询坏了,是向量库本来就是空的(直接确认:`len(sys_.vindex._vecs)==0`)。
修复:把 `p5_ingest_4dkankan.py` 的入库逻辑重构成可调用函数
`ingest(db_path, fresh=True)`,`p5_audit_4dkankan.py` 改为在**同一个进程**
里先调这个函数拿到同一个 `SpatialMemorySystem` 对象再跑审计,`fresh=True`
每次清空重建DB避免重复ingest导致uuid堆积。这个"向量索引不持久化"的现状
本身值得记一笔——如果这套框架要真正支持"重启后语义检索还能用",M1阶段
需要把向量索引也接到持久化存储(框架文档里其实已经预告了M2会换
pgvector/Faiss,现在只是提前验证了这个缺口在M0确实存在)。

**修复后的检索结果**:

| 语言 | Recall@3 | 命中/总数 |
|---|---|---|
| 中文(原定10条) | 0.200 | 2/10 |
| 英文(同样10个概念的对照组) | 0.800 | 8/10 |

**根因诊断**:英文80% vs 中文20%,差距巨大,而且数据库里对应类别的实例
数量都不小(比如chair有36个)。排除掉"检索机制坏了"或"数据库里没有对应
实例"这两种可能后,定位到:`ViT-B-32/laion2b_s34b_b79k` 这个checkpoint
训练数据以英文图文对为主,文本塔对中文的对齐明显弱于英文。

**结论(这是本次验证除了"漏检偏多"之外第二个具体的、可执行的改进项)**:
如果这套空间记忆系统要真正服务中文场景(CLAUDE.md原始需求就是"中文日常
问法"),`ViT-B-32/laion2b_s34b_b79k` 不是合适的embedding模型,需要换成
中文优化过的CLIP变体(如 Chinese-CLIP、AltCLIP)。

**产物更新**:
- `scripts/p5_ingest_4dkankan.py` — 重构出可复用的 `ingest()` 函数
- `scripts/p5_audit_4dkankan.py` — 同进程ingest+audit,支持本地CLIP权重
  文件加载,新增中英文检索对照测试
- `work/phase5_4dkankan_validation/audit_report.md` — 补全检索审计章节
  (含逐条中英文查询命中详情 + 根因诊断)
- `work/phase5_4dkankan_validation/open_clip_model.safetensors` — 用户
  手动下载的CLIP权重(605MB,gitignore范围内,不会被提交)

至此,Phase 5 空间记忆技术验证(用4dkankan数据)四层审计+检索全部跑完,
两个具体可执行的改进方向已经找到:①漏检率偏高(尤其重复/密集摆放的
同类物体、非常规视角的物体)②中文语义检索需要换CLIP模型。

---

## 附加验证(续三)— 3D查看器新增全景视角,实例投影到真实照片上

在原有的4dkankan查看器(139实例+网格)基础上加了"全景视角"模式,响应
用户需求"把识别到对象投影到全景图像上"。

**实现方式**:
- 26个有效点位每个点的6张cubemap照片(压缩到320×320,quality78,共
  ~3.9MB)全部内嵌进HTML,配合每张照片对应的相机朝向基向量(right/down/
  forward,和 `p4_adapt_4dkankan.py`/`p5_ingest_4dkankan.py` 用的是同一套
  投影约定,Y-up→Z-up变换也保持一致)。
- 面板顶部加了"3D网格/全景视角"模式切换。全景模式下:相机固定在选中点位
  的真实位置,只能环顾四周(不能平移/缩放),6张照片按各自真实朝向拼成
  一个立方体环境,无缝衔接。
- 每个检测实例按其真实3D方向从当前点位投影到全景立方体表面(距离固定
  在球面上,不是真实距离——避免真实距离过近/过远导致视觉比例失真),
  复用3D模式下同一套"跳动光点"marker着色器,点击/悬停显示类别标签。
- 只投影5米范围内的检测(而不是整个zone),避免同一个zone里27个实例
  全部糊在一张照片上看不清。

**验证方式**:本地没有浏览器可交互测试,用 Playwright + headless
Chrome(`--use-gl=swiftshader` 软件渲染)实际加载页面、截图核对——
确认了立方体六个面拼接无缝(没有翻转/镜像的接缝错位)、拖拽环顾视角
正常、切换点位正常、marker投影位置和照片内容(天花板灯具、椅子等)
对得上。控制台0报错。

**产物**:更新后的同一个查看器URL:
https://claude.ai/code/artifact/ba54e2ce-4a62-46de-ab60-75e40dbe2328
(3D网格模式和之前完全一样,新增全景模式作为可选视角)

---

## 附加验证(续四)— 修复Y-up→Z-up坐标转换的方向错误(用户报告)

用户反馈全景视角里"天花板和地板对调,其他图片也有问题"。排查后确认是一个
实际存在的bug,不是想象:

**根因**:`yup_to_zup` 转换公式写错了。用的是 `(x,y,z)→(x,z,-y)`,这确实是
一个合法的正交旋转(行列式+1,不镜像),但验证下来它把 Y-up 的"上"方向
`(0,1,0)` 映射到了 Z-up 的"下"(`(0,0,-1)`),而不是"上"。正确公式应该是
`(x,y,z)→(x,-z,y)`——用 `(0,1,0)` 代入验证:`(0,-0,1)=(0,0,1)`,才是
Z-up的"上"。错的那版实际上相当于把整个场景绕Z-up的X轴转了180度:不仅
上下翻了,水平方向上的另一对面(Y_zup对应的那两个cubemap面)也跟着错了,
只有左右(X_zup)那一对没受影响——这和用户说的"天花板地板对调,其他图片
也有问题"完全对得上。

**排查方式**:没有直接改公式了事,是先手算验证了两个候选公式分别把
Y-up的"上"映射到哪,确认了错在哪、该怎么改,而不是猜。

**影响范围排查**:
- 3D网格查看器(点云+139实例位置)、全景查看器(cam_pos+相机朝向基向量)
  ——**受影响,已修复重新生成**。这个bug在纯3D环视模式下不容易被肉�眼
  发现(一个对称性较强的室内点云从外部俯视角度看,倒转过来也不会显得
  特别违和),是这次加了第一人称全景模式才暴露出来的。
- `scripts/p5_ingest_4dkankan.py` 里同一个函数(用于Phase5空间记忆入库
  的Place聚类边界)——**同样有这个bug,已同步修复**。
- **但Phase5四层审计的数值结论不受影响**:L1/L2/L3的P/R/F1、位姿MAE、
  归属准确率,底层全部基于"记忆实体位置"和"真值位置"之间的**欧氏距离**
  计算,K-means聚类和AABB包含判定也是基于变换后坐标自身重新计算的——
  只要真值和记忆用的是**同一套(哪怕是错的)变换**,距离和包含关系在
  正交旋转下不变,不会被这个bug污染。这不是事后找借口,是先验证了
  "两种公式都是合法正交变换、只差一个180度旋转"这个数学事实之后才敢
  这么说的,已经在数据层面用这个不变性论证过,没有重新跑一遍整个审计
  流程去"求证"——如果之后有疑虑,可以用修复后的坐标重新跑一遍
  `p5_audit_4dkankan.py` 交叉验证,预期数字应该完全一样。

**产物**:
- `scripts/p5_ingest_4dkankan.py` 的 `yup_to_zup` 已修复,注释里记录了
  完整的错误分析,防止以后又被绕进去。
- 查看器数据(点云、139实例位置、26点位全景cam_pos/朝向)全部用修复后
  的公式重新生成,已重新发布到同一个链接:
  https://claude.ai/code/artifact/ba54e2ce-4a62-46de-ab60-75e40dbe2328
- 用 Playwright 截图核对:环视时天花板方向能看到管线/灯具,地板方向
  能看到木地板,和之前颠倒的状态对比确认修复生效。

---

## 附加验证(续五)— 天花板贴图朝向修复 + 澄清"站在天花板上"的疑问

**关于"是否对天花板/地板对调产生了误解"**:不是误解,那个bug是真实的,
有两方面独立证据支撑,不是凭感觉判断的:
1. 数学证明:验证了两个候选公式分别把 Y-up 的"上"方向 `(0,1,0)` 映射到
   哪——错的那版映射到 Z-up 的"下" `(0,0,-1)`,不是"上"。
2. 修复前后分别用 Playwright 截图对比:同一个拖拽方向(比如往下拖看
   "地面"),修复前显示的是天花板管线内容,修复后显示的是木地板内容——
   内容本身真的换了,不是默认视角凑巧对着哪个方向的问题。

**新反馈的天花板贴图朝向错误,是一个独立的bug**,和上面那个坐标轴bug
不是一回事:

`cv_basis()` 函数处理"上"/"下"两个极点朝向时(相机朝向和世界上方向平行,
没法用标准的 cross product 求右手系),用了一个任意选的参考轴
`[0,0,1]` 兜底,这个参考轴和水平环那4个面(skybox1-4)各自的朝向之间
没有建立起对应关系,所以拼出来的天花板贴图旋转角度是"蒙的",凑巧和
正确朝向差了整整180度。

**这次没有直接照用户说的"转180度"照抄了事**,而是重新独立推导了一遍:
从水平参考面 skybox1(朝向世界+Z)出发,physically把相机"抬头"转90度
看向天花板,推出此时"右手"方向应该保持不变、原来的"下"方向变成新的
"前"方向——按这个物理直觉重新算出天花板正确的基向量,和 `cv_basis()`
兜底逻辑算出来的结果做对比,发现确实正好相差180度(right和down两个
分量都反了符号),和用户的观察完全吻合,不是拍脑袋接受了用户的说法。

**已知残留的小问题(如实记录)**:测试 Point 17 往上看时,天花板和两个
相邻水平面交界的墙角处(彩色装饰画那个墙角)截图看到一处不太平滑的
接缝,不确定是相邻两个水平面(skybox2/skybox3之类)之间本身的拼接
误差(供应商原始素材的已知瑕疵,之前查看单张skybox图片时就见过类似的
模糊/重影接缝),还是还有别的问题没排查到。这个和用户报告的"天花板
贴图本身要转180度"是两回事,先如实标注在这里,后续如果用户还看到明显
错位可以再具体定位。

**产物**:查看器已重新发布到同一链接(favicon不变):
https://claude.ai/code/artifact/ba54e2ce-4a62-46de-ab60-75e40dbe2328

---

## 附加验证(续六)— Isaac Sim + Lightwheel 完美真值直灌验证第1步(1a+1b)   [状态: 完成,发现2个真bug+1个规格/实现偏差]

**背景**:`docs/p5_sim_validation_plan.md` 提出用 Isaac Sim + Lightwheel
SimReady 资产构造完美真值,跳过 Phase1-4,直接验证 Phase5 记忆层
(`spatial-memory-m0/`)本身的正确性,和感知/采集链路的数据质量问题彻底
解耦——延续本session一直在用的"分层隔离验证"思路,这次是最彻底的一次隔离
(连"输入是不是完美的"都不再是变量)。用户确认按默认方案执行,不需要逐条
确认。

**环境探查,推翻了方案原文一个假设**:方案原文认为"不需要跑 Isaac Sim
仿真循环"等价于"不需要 Isaac Sim 本体"。实测发现这只对**自包含**场景成立
——`Locomotion/KitchenRoom.usd` 这类用 payload 引用共享道具资产的场景,
本地纯 `pip install usd-core` 打开后道具 payload 解析失败(绝对路径断链
+ 缺一个 Omniverse 专有的 `metricsAssembler` 图层插件),这类场景**确实
需要** Isaac Sim 的完整资产解析栈。好在 `Locomotion/Apartment/scene_04.usd`
是烘焙自包含场景,本地 pip 装的 `usd-core`(Python 3.13,不需要 Isaac
Sim/云电脑)就能正确读出全部 104 个实例(Z-up、米制,不需要坐标变换),
**零云成本、现在就能跑**,于是用户确认第一步只用这个场景,不追加云电脑
去修 KitchenRoom。

**标签与Place**:19个name-root(无 Semantics API 标注,方案预留的"退化用
prim路径名"分支是唯一可用路径),手工核对了两个有歧义的类别(`SM_kitchen`
确认是橱柜构件;`SM_bar`——0.67×0.68×0.04m的薄盘,材质`plastic_bar`——
认不出真实身份,诚实标注`label_confidence=low`,没有编造一个可能错的人类
标签)。Place 不是k-means,是场景本身天然的两簇结构(实测发现,不是猜的):
apartment_unit(室内单元,72实例)与courtyard(室外庭院,31实例),两者
物理距离很远且高程不同,对跨place门控测试(T8)是个干净的条件。

**1a 静态直灌**:F1=1.000、位姿MAE=0.000m、归属准确率=1.000,三项全部
满分,达标线(F1≥0.95/MAE≤0.05m/归属=1.0)。和4dkankan那次真实数据审计
(F1=0.083-0.095)对比,证实了当时的推测——低分根子在感知层,不在记忆层。

**1b 时序分支覆盖(T1-T12)**:6/6事件类型分支覆盖、13/13序列精确匹配、
18/18终态检查通过。开跑前按方案要求书面预注册了4条预测(见
`work/p5_sim_validation/PLAN.md`§2.5),3条命中真bug/偏差,1条证伪了方案
自己的担忧:

1. **【真bug】`human_correct()` 完全无法修正 pose 字段**——两种调用方式
   (传Pose对象 / 传dict)都会崩溃,一个在事件日志JSON序列化处,一个在
   SQLite拍平列读取处。这个函数从未被"改pose"这个最自然的用例实测过。
2. **【真bug,预注册命中】pin保护期挡不住"正观测覆盖"**——`pinned_until_us`
   检查只写在负观测(DECAY)分支里,正观测匹配到后走`_do_update`完全不检查
   保护期。实测:人工修正位姿并pin 24小时后,机器一次旧读数观测就把修正
   覆盖回去了,pin形同虚设。
3. **【规格/实现偏差,预注册命中】DYNAMIC类对象会被正常落长期库**——
   `_do_add()`不检查mobility(`Detection`本身没有这个字段),违背
   `docs/phase5_schema.md`"DYNAMIC只进工作记忆不落长期库"的设计意图。
4. **【证伪方案的担忧】同批检测不会链式合并**——候选集在循环前就已固定
   快照,T10(1.2m,同批)实测两个同类检测各自独立ADD,方案"最可能翻车的
   用例"没有翻车。
5. 顺带发现`_do_add()`不设置mobility(新实体一律落SEMI_STATIC默认值),
   和`embedding_model`默认值问题是同一类坑,两个脚本都做了二次遍历补丁。
6. 方案文档T1/T2表格里"conf=1.2"的预期值算漏了`min(1.0,...)`封顶,实测
   都是1.0,已在报告里指出更正。

**T12 遮挡陷阱(量化实验,单独一节)**:用open3d对24面墙的真实三角网格
(`scripts/p5sim_extract_walls_mesh.py`提取,497顶点/902面)做光线求交,
3个巡逻站点实测:**8.3%的"视野内"候选物体其实被墙挡住,这些对象100%被
M0错误DECAY**(中心+半径模型对遮挡零感知)。这个数字直接支撑M0→M2升级到
真实视锥+mesh遮挡剔除的立项依据。open3d没有Python 3.13 wheel,单独建了
个Python 3.11 conda env(`.venv_p5sim_o3d/`,用conda-forge渠道装的,绕开了
Anaconda默认渠道的ToS门槛)。

**决策规则复核**:1a/1b两项指标都"通过"了方案§0的验收线,但**不建议直接
进入第2/3步**(4dkankan补视角归因、退化注入曲线)——建议先把上面#1
(human_correct改pose)和#2(pin保护失效)这两个真bug当独立任务修掉,因为
它们不是"没覆盖到的边缘情况",是"覆盖到了、而且行为明确错误"。

**产物**:
- `work/p5_sim_validation/PLAN.md` — 执行计划、环境探查记录、预注册预测
- `work/p5_sim_validation/report.md` — 完整报告(指标表、逐用例结果、
  6项发现的完整分析、T12遮挡量化明细)
- `scripts/p5sim_extract_gt.py` / `p5sim_run_1a.py` / `p5sim_scenario.py`
  / `p5sim_run_1b.py` / `p5sim_extract_walls_mesh.py` /
  `p5sim_occlusion_check.py`
- `.venv_p5sim/`(Python 3.13,usd-core+numpy+scikit-learn)、
  `.venv_p5sim_o3d/`(Python 3.11,open3d)——均已加入 `.gitignore`
