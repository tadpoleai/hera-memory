# Phase5 仿真完美真值直灌验证方案（第1步）

> 目标：用 Isaac Sim 4.5 + Lightwheel SimReady 资产构造带完美真值的观测流，
> 跳过 Phase1-4，直接验证 Phase5 记忆层（schema/store/consolidator/审计）
> 本身的正确性。对应总验证路线的第 1 步（见前期讨论），分 1a（静态直灌）
> 与 1b（时序分支覆盖）两个子实验。
> 依赖文档：`docs/phase5_schema.md`、`docs/hera_data_spec.md`、
> `docs/third_party_reference_data_spec.md`。

---

## 0. 实验定位与通过标准

| 子实验 | 回答的问题 | 通过标准 |
|---|---|---|
| 1a 静态直灌 | 完美输入下，与 4dkankan 同口径的审计指标是多少？ | 严格标签 F1 ≥ 0.95、位姿 MAE ≤ 0.05m、归属准确率 = 1.0（理论期望全部 ≈ 满分，任何显著偏差即记忆层 bug） |
| 1b 时序分支 | CONFIRM/UPDATE/DECAY/RETIRE/pin 五个从未测过的分支是否按 §3 规格工作？ | 事件层审计：每个实体的实际事件流与期望事件流逐条匹配，分支覆盖率 100% |

**决策规则**：1a 或 1b 不通过 → 暂停 hera 补采与 Phase1-3 调参，先修记忆层；
两者均通过 → Phase5 可行性记为"完美输入下成立"，进入第 2/3 步
（4dkankan 补视角归因、退化注入曲线）。

---

## 1. 为什么第 1 步不需要跑 Isaac Sim 仿真循环

完美真值直灌只消费三样东西：**语义标签、世界位姿、AABB**。这些全部可以用
`pxr`（USD Python API）headless 读取 Lightwheel 场景 USD 得到，不需要渲染、
不需要物理步进。1b 的"物体移动/移除"也只是用 USD API 改写 prim 的 transform
或 active 状态后重新提取一遍真值，同样不需要仿真循环。

Isaac Sim 本体留到第 3 步（退化注入需要渲染 RGB/Depth 跑真实 ConceptGraphs）
和第 4 步（双 session 变化实验若要出感知数据）才真正需要。

好处：第 1 步的全部代码是纯 Python + pxr，可以进 CI，跑一次秒级。

---

## 2. 数据提取与字段映射

### 2.1 前置检查（必做，两个已知类型的坑）

```python
from pxr import Usd, UsdGeom
stage = Usd.Stage.Open(scene_usd_path)
up = UsdGeom.GetStageUpAxis(stage)        # 期望 "Z"（Isaac Sim 默认）
mpu = UsdGeom.GetStageMetersPerUnit(stage) # 期望 1.0
```

- **上轴**：框架约定 Z-up。Isaac Sim 默认 Z-up，但个别资产可能按 USD 默认
  Y-up 制作。若检出 Y-up，套用 `p5_ingest_4dkankan.py` 中**当前正确版**的
  变换 `(x,y,z) → (x,-z,y)`，不要复用早期错误版 `(x,z,-y)`（schema 文档
  §4.2 记录的 bug）。
- **单位**：若 `metersPerUnit ≠ 1.0`（如 0.01 表示厘米），所有平移与 AABB
  乘以该系数归一到米。

### 2.2 真值提取

遍历 stage 上带语义标注的 prim（SimReady 资产带结构化语义 schema；若个别
资产无 Semantics API 标注，退化用 prim 路径名末段做标签，人工核对一遍）：

```python
from pxr import Usd, UsdGeom, Gf
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                          [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
for prim in stage.Traverse():
    label = read_semantic_label(prim)          # Semantics API / 路径名回退
    if label is None: continue
    xf   = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos  = xf.ExtractTranslation()
    quat = xf.ExtractRotationQuat()
    aabb = cache.ComputeWorldBound(prim).ComputeAlignedRange()
```

yaw 提取（Z-up）：`yaw = atan2(2(qw*qz + qx*qy), 1 - 2(qy^2 + qz^2))`。

### 2.3 映射到 Phase5 契约

| 目标字段 | 来源 | 说明 |
|---|---|---|
| `Detection.class_label` | prim 语义标签 | 建一张 `标签→归一词表` 映射（资产标签风格可能与 ConceptGraphs 开放词汇不同，归一后才能与 4dkankan 口径可比） |
| `Detection.pose` | prim 世界位姿 | 单 Submap、锚定世界原点（复用 4dkankan 的做法），故 submap 相对位姿 = 世界位姿 |
| `Detection.embedding` | **Run-1 置 None** | 见 §2.4 |
| `Detection.attributes` | 可选，从资产元数据取颜色/状态 | 非必须，留空不影响 1a/1b |
| `ObjectInstance.mobility` | 按类别查表 | 建 `class→mobility` 字典：墙/门框/橱柜→STATIC，桌椅/家电/餐具→SEMI_STATIC；DYNAMIC 类第 1 步不引入（其"只进工作记忆"的语义单独在 1b-T7 验证） |
| `Place` | 单房间场景 → 手工划 2-3 个 AABB 分区（如厨房操作区/餐区） | 刻意划出多个 place 是为了测跨 place 门控（1b-T8） |
| `ObservationEvent.view_center / view_radius` | 脚本设定的"虚拟机器人"站点 | radius 沿用 4dkankan 的 6.0m 起步 |
| `ObservationEvent.timestamp_us` | 脚本递增的逻辑时钟 | 时序实验的自变量，不必是真实时间 |

### 2.4 embedding 策略：两轮运行

- **Run-1（主实验）**：`embedding=None`，匹配退化为标签精确匹配
  （`_similarity` 的回退路径）。目的：把记忆逻辑与嵌入质量彻底隔离——
  第 1 步验证的是 consolidator 的分支机器，不是 CLIP。
- **Run-2（可选加测）**：给每类资产渲染一张缩略图过 CLIP（ViT-B-32），
  验证 `match_min_sim=0.55` 门控在真实向量下的行为。
  注意 schema 文档 §4.3 的坑：`_do_add` 不透传 `embedding_model`，入库后需
  二次遍历改写，否则库里残留 `"toy-trigram@v0"` 假标注。

---

## 3. 实验 1a：静态直灌（对齐 4dkankan 口径）

流程 = `p5_ingest_4dkankan.py` 的同构复刻，仅数据源换成 USD 真值：

1. 提取全部实例（预期几十~一二百个，取决于所用场景）；
2. 按 place 分组打包 `ObservationEvent`（每 place 一条，一次性导入）；
3. `ingest(db_path, fresh=True)` 入库（**fresh=True 必须**，避免重复运行
   uuid 变化导致事件堆叠——schema 文档 §4.4）；
4. **同一进程内**跑四层审计（vindex 不持久化，跨进程语义检索会静默返回空
   ——§4.4 的第一坑；Run-1 无向量，此坑不触发，但流程上先养成习惯）；
5. 真值 = 提取脚本自身的输出（零标注成本），审计指标与 4dkankan 报告
   逐项对齐：严格标签 F1、位姿 MAE、归属准确率。

**预期与解读**：所有指标应 ≈ 满分。这一轮的真正产出是一句可写进报告的话：
"与 4dkankan 的 0.083-0.095 相比，完美输入下 F1 = X"——X 与 1.0 的差距
（若有）全部归因记忆层，X 与 0.09 的差距全部归因感知层。

---

## 4. 实验 1b：时序分支覆盖（本次主菜）

### 4.1 场景剧本

选一个基准场景状态 S0，脚本生成 T 个逻辑时刻的观测序列。每个受试对象
按剧本操作，期望事件流由 §3 规格直接推出：

| 用例 | 对象操作 | 期望事件流 | 期望终态 | 验证点 |
|---|---|---|---|---|
| T1 原地重复 | 不动，观测 3 次 | ADD → CONFIRM → CONFIRM | version=1, conf=1.2, obs_count=3 | CONFIRM 不涨 version、不重复 ADD |
| T2 大位移 | 第 2 次观测前移动 0.5m（>0.30m） | ADD → UPDATE | version=2, pose=新位置, conf=1.2 | moved_dist 阈值上沿 |
| T3 小位移 | 移动 0.2m（<0.30m） | ADD → CONFIRM | pose=**旧位置** | 阈值下沿；确认"小位移被吸收、库内位姿不更新"确是预期语义（注意：连续多次 0.2m 位移会累积漂出 1.5m 匹配门，属已知设计张力，记录实测行为即可） |
| T4 semi_static 消失 | 移除，站点仍覆盖原位置，继续观测 3 次 | ADD → DECAY → DECAY → DECAY+RETIRE | status=retired | 衰减算术：1.0→0.70→0.40→0.10<0.25，第 3 次 DECAY 触发 RETIRE |
| T5 static 消失 | 同 T4，但对象为 STATIC 类 | ADD → DECAY×3（conf≈0.94） | active | 衰减速率分档（-0.02/次，38 次才退休——只验前 3 次斜率） |
| T6 视野外消失 | 移除，但站点半径**不**覆盖原位置 | ADD（此后无事件） | active, conf=1.0 | 负观测判定不越界：看不见的地方不许衰减 |
| T7 dynamic 检测 | DYNAMIC 类对象入观测 | 按文档"只进工作记忆不落长期库"——实测行为待记录 | 库内应无此实体（若有，即规格与实现不一致，记录之） | Mobility=DYNAMIC 的落库策略从未验证 |
| T8 跨 place | 对象在 place-A，观测事件标 place-B（半径覆盖该对象） | 期望**不**匹配也**不**衰减（候选集按 place_id 门控） | A 中实体不受影响 | place 门控有效性 |
| T9 人工修正+pin | human_correct 改位姿并 pin 7 天 → 机器观测报旧位姿 | HUMAN_CORRECT → （机器观测被拒或降级，实测记录） | pose=人工值不被覆盖 | pinned_until_us 保护期 |
| T10 近距同类对 | 两个同 label 对象相距 1.2m（<1.5m 门控），同批观测 | 期望 ADD×2 | 2 个实体 | 贪心匹配在门控半径内的同类对上是否误合并（对应 4dkankan 链式合并的记忆层版本） |
| T11 远距同类对 | 同 label 相距 2.0m | ADD×2 | 2 个实体 | 对照组 |
| T12 遮挡陷阱 | 对象在站点半径内但被墙/柜体遮挡（同 place），对象未移除但"检测不到"（从 Detection 列表剔除以模拟视线被挡） | M0 简化视锥会误判"应见未见"→ 错误 DECAY | 记录误衰减发生与幅度 | **量化 M0 中心+半径模型的固有误伤率**，作为 M2 真视锥+mesh 遮挡剔除升级的立项依据 |

T12 的真值视线可用 USD mesh raycast 判定（Open3D `RaycastingScene` 即可，
与 4dkankan 深度渲染同一套工具链），不需要 Isaac Sim 渲染。

### 4.2 事件层审计（新增审计维度）

剧本生成时同步产出 `expected_events.json`：
`{对象真值id: [期望 EventType 序列]}`。审计脚本按 `entity_uuid` 从
`events` 表拉实际序列（`idx_evt_entity` 索引现成），逐条比对：

- **分支覆盖率**：ADD/UPDATE/CONFIRM/DECAY/RETIRE/HUMAN_CORRECT 六类事件
  各自至少被一个用例命中且行为正确；
- **序列精确匹配率**：逐实体期望序列 vs 实际序列的完全匹配比例；
- **终态一致率**：`objects` 表终态（status/version/confidence/pose）与
  剧本推演终态的匹配比例——顺带验证"objects 是 events 的物化视图"这一
  §2.1 设计承诺在时序场景下真实成立。

### 4.3 已知实现风险清单（预注册，避免当场争论是 bug 还是 feature）

跑之前先书面预测，跑完对答案：

1. T3 小位移吸收 → 位姿陈旧化：预测为规格内行为，但记录累积漂移隐患；
2. T7 DYNAMIC 落库：`consolidator.ingest()` 是否真的对 DYNAMIC 有特殊
   分支存疑（schema 只在 Mobility 注释里提了一句），预测实现里可能
   **并没有**拦截，会照常 ADD——若如此即规格/实现偏差，记录；
3. T9 保护期内机器观测：被拒后是否产生任何事件（静默丢弃 vs 记录被拒
   事件）规格未写明，实测定义之；
4. T10 同批 ObservationEvent 内两个同类 Detection 先后处理时，第二个是否
   会匹配上第一个刚 ADD 的实体（取决于 `_match` 候选集是否含本批新建实体）
   ——这是最可能翻车的用例。

---

## 5. 产出物

```
work/p5_sim_validation/
├── extract_gt.py              # USD → 真值实例清单（含上轴/单位检查）
├── scenario.py                # 1b 剧本定义 + expected_events.json 生成
├── run_1a.py / run_1b.py      # 入库+同进程审计（复用 ingest() 重构后接口）
├── gt_instances.json          # 1a 真值
├── expected_events.json       # 1b 事件层真值
└── report.md                  # 指标表：1a 三指标对照 4dkankan；1b 覆盖率矩阵 + 风险清单对答案
```

---

## 6. 与后续步骤的接口

- 1a 的提取脚本直接复用于第 3 步（退化注入的"干净基线"就是它的输出）；
- 1b 的剧本机制直接扩展为第 4 步的双 session 变化实验（把逻辑时刻拉长为
  两个 session、变更规模放大即可）；
- T12 的误衰减率是 M0→M2 视锥升级的量化立项依据，写进报告单独成节。
