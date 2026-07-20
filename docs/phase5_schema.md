# Phase5 入库 Schema 细节

> 本文档描述空间记忆框架（`spatial-memory-m0/`）Phase5 入库阶段的数据契约、
> SQLite 落库结构、入库决策逻辑，以及目前唯一有实测记录的入库案例——用
> 4dkankan 商场数据（139个实例）跑通全流程时的真实映射方式与踩过的坑。
> 本地 hera 家庭数据目前只有 6 个可用实例，样本量不足以支撑完整入库验证，
> 尚未真正跑过这条路径（背景见 `docs/hera_data_spec.md` 与
> `docs/third_party_reference_data_spec.md`）。

分四层：**核心数据契约**（`schema.py`）→ **SQLite 落库结构**（`store.py`）
→ **入库决策逻辑**（`consolidator.py`）→ **4dkankan 数据实际落地时踩的坑**
（`scripts/p5_ingest_4dkankan.py`）。

---

## 1. 核心数据契约（`spatial-memory-m0/spatial-memory/spatial_memory/schema.py`）

M0 阶段用 dataclass 起步，字段注释里预留了 M1 迁移到 Protobuf 时对应的
field number，全系统（存储、整合器、查询、机器人端）都只依赖这一份契约。

### 1.1 几何基元

```python
Pose(x, y, z, yaw)
# SE(3) 简化版：M0 只用平移 + yaw，四元数字段预留给 M1

AABB(min_x, min_y, min_z, max_x, max_y, max_z)
# 轴对齐包围盒，M2 才升级成 OBB（有向包围盒）
```

### 1.2 三个层级的空间组织单元

| 层级 | 类型 | 关键字段 |
|---|---|---|
| L0 | `Submap` | `submap_id`, `anchor_pose_world`（回环修正只改这一条）, `bounds`, `anchor_version`, `mesh_blob_uri`, `status` |
| L2 | `Place` | `place_id`, `name`, `floor_id`, `bounds`（AABB代替多边形）, `connected_to[]`（拓扑连通的 place_id） |
| L1 | `ObjectInstance` | 见下（空间记忆的基本记账单位） |

`ObjectInstance` 完整字段：

```python
uuid: str
class_label: str                   # 开放词汇主标签
pose: Pose                         # 注意：相对 submap_id 的位姿，不是世界系！
submap_id: str
place_id: str                      # 冗余外键，加速"房间里有什么"这类查询
embedding: Optional[np.ndarray]    # 语义特征向量
embedding_model: str = "toy-trigram@v0"  # 换模型时必须区分版本，见 §4 的坑
aliases: list[str]
attributes: dict[str, str]         # {"color":"red","state":"door_open"}
mobility: Mobility                 # static / semi_static / dynamic，决定衰减速率
confidence: float = 1.0
status: EntityStatus                # active / retired
first_seen_us, last_seen_us: int
observation_count: int = 1
last_updated_by: str                # provenance
version: int = 1                    # 乐观锁
pinned_until_us: int = 0            # 人工修正保护期，此期间内机器观测不能覆盖
```

`Mobility` 枚举决定负观测衰减速率：`STATIC`（墙/门框，几乎不衰减）、
`SEMI_STATIC`（桌椅/设备，按天衰减）、`DYNAMIC`（人/推车，只进工作记忆，
不落长期库）。

### 1.3 事件与观测

```python
SpatialEvent(event_id, entity_uuid, event_type, payload, source,
             timestamp_us, entity_version)
# 追加式事件日志的记录单元；实体表 = 事件流的物化快照

EventType: ADD | UPDATE | CONFIRM | DECAY | RETIRE | HUMAN_CORRECT

Detection(class_label, pose, embedding, attributes, score)
# 机器人端单个检测结果（已在边缘侧完成蒸馏），pose 相对 submap

ObservationEvent(submap_id, place_id, robot_id, detections: list[Detection],
                  view_center: Pose, view_radius: float, timestamp_us)
# 机器人上传的最小单元（<5KB）；view_center+view_radius 是简化视锥，
# 用于支持"应见而未见"的负观测判定
```

---

## 2. SQLite 落库结构（`store.py::_SCHEMA_SQL`，`SqliteStore` 实现）

```sql
CREATE TABLE objects (
    uuid TEXT PRIMARY KEY,
    class_label TEXT NOT NULL,
    place_id TEXT NOT NULL,
    submap_id TEXT NOT NULL,
    status TEXT NOT NULL,
    confidence REAL NOT NULL,
    x REAL, y REAL, z REAL, yaw REAL,   -- 位姿字段拍平，供空间查询走索引
    last_seen_us INTEGER NOT NULL,
    version INTEGER NOT NULL,
    doc TEXT NOT NULL                    -- 完整对象的 JSON 快照，避免频繁改列
);
CREATE INDEX idx_obj_place  ON objects(place_id, status);
CREATE INDEX idx_obj_submap ON objects(submap_id, status);
CREATE INDEX idx_obj_seen   ON objects(last_seen_us);

CREATE TABLE places  (place_id  TEXT PRIMARY KEY, doc TEXT NOT NULL);
CREATE TABLE submaps (submap_id TEXT PRIMARY KEY, doc TEXT NOT NULL);

CREATE TABLE events (
    seq INTEGER PRIMARY KEY AUTOINCREMENT,  -- 全局单调，增量同步游标
    event_id TEXT NOT NULL,
    entity_uuid TEXT NOT NULL,
    event_type TEXT NOT NULL,
    source TEXT NOT NULL,
    timestamp_us INTEGER NOT NULL,
    entity_version INTEGER NOT NULL,
    payload TEXT NOT NULL
);
CREATE INDEX idx_evt_entity ON events(entity_uuid, timestamp_us);
```

### 2.1 关键设计点

- **快照 + 日志分离但同库**：`objects` 表永远是"当前最新状态"，`events`
  表是不可变追加日志——`objects` 理论上是 `events` 的物化视图。
- **`doc` 列存整个对象的 JSON**：结构化列（x/y/z/yaw/place_id 等）只是为了
  让常用查询（按房间、按子图、按时间）走索引，避免 M0 阶段频繁
  `ALTER TABLE`；完整读写走 `doc` 反序列化。
- **embedding 向量不落 SQLite**：`ObjectInstance.to_row()` 与
  `store.py::_obj_to_json()` 都显式把 `embedding` 设为 `None` 再序列化——
  向量只存在内存态的 `NumpyVectorIndex` 里，**进程重启即丢失**（见 §4 的坑）。

### 2.2 抽象接口与升级路径

| 接口 | M0 实现（本库） | M2+ 生产实现 |
|---|---|---|
| `EntityStore` | SQLite 表 | PostgreSQL(+PostGIS 空间索引) |
| `EventLog` | SQLite 追加表 | PG 分区表 → Kafka + Parquet 冷备 |
| `VectorIndex` | numpy 暴力检索（`NumpyVectorIndex`） | pgvector HNSW / Faiss / Milvus |
| `BlobStore` | 本地文件目录（`LocalBlobStore`） | MinIO / S3 |

M0 刻意选 SQLite 而非纯内存，是为了让"重启后记忆还在"从第一天就成立，倒逼
序列化问题提早暴露。

---

## 3. 入库决策逻辑（`consolidator.py::Consolidator.ingest()`）

每条 `ObservationEvent` 进来后：

1. **取候选集**：同一 `place_id` 内、`view_center ± view_radius` 范围内的
   活跃实例（M0 的"视锥"简化模型 = 观测中心+半径，不能隔墙看；M2 才升级为
   真实视锥 + mesh 遮挡剔除）。
2. **逐个 `Detection` 做匹配**（`_match`，几何门控 + 相似度贪心）：
   - 距离门控：`distance ≤ match_max_dist = 1.5m`
   - 相似度门控：嵌入余弦相似度 `≥ match_min_sim = 0.55`（无嵌入时退化为
     标签精确匹配，见 `_similarity`）
   - 打分：`score = sim − 0.1 × dist`，取候选集中最优
3. **五种写入分支**：

| 情形 | 事件类型 | 行为 |
|---|---|---|
| 匹配不到任何实例 | `ADD` | 新建 `ObjectInstance`，写入 vindex |
| 匹配到但位姿/属性变化超阈值(`moved_dist=0.30m`) | `UPDATE` | 更新位姿/属性，`version+1`，`confidence+0.2` |
| 匹配到且一致 | `CONFIRM` | 只刷新 `last_seen`/`confidence+0.1`，`version` 不变（不算改变事实） |
| 候选集里视野内但没被检测到 | `DECAY` | `confidence` 按 mobility 扣减：static−0.02 / semi_static−0.30 / dynamic−1.0 |
| DECAY 后 `confidence < retire_threshold(0.25)` | `RETIRE` | `status=retired`，从 vindex 移除 |

另有 `human_correct(entity_uuid, patch, user_id, pin_seconds)` 走同一事件
通道，但带 `pinned_until_us` 保护期（默认7天），机器观测在保护期内不能覆盖
人工修正。

---

## 4. 实际落地案例：4dkankan 数据入库（`scripts/p5_ingest_4dkankan.py`）

这是目前唯一有真实数据跑过的入库案例，几个值得记录的实际决策与踩过的坑：

### 4.1 空间组织映射

- **Submap**：单个，世界原点锚定——因为整个场景已经是厂商处理好的全局
  一致 mesh，没有我方自采管线那种子图边界要处理。
- **Place 划分**：对 26 个有效采集点（排除 batch4 灾难性合并案例，见
  `docs/third_party_reference_data_spec.md`）的水平坐标做
  **K-means(k=8)** 聚类；每个 zone 的 AABB = 成员实例 bbox + 1.5m padding，
  最大跨度控制在 7.2m（房间尺度）。

### 4.2 坐标系转换（一个已修复的 bug，值得记录）

4dkankan 原始坐标是 Y-up，框架约定 Z-up，需要保持右手系的正交变换：

```python
# 正确版本（当前）：(x,y,z) Y-up -> (x,-z,y) Z-up
def yup_to_zup(v):
    return np.array([v[0], -v[2], v[1]])
```

早期版本用的是 `(x, z, -y)`——这是另一个"合法"但方向相反的 90° 旋转，会把
Y-up 的"上"映射成 Z-up 的"下"（外加一个水平轴的翻转）。这个错误在 3D 环视
查看器里因为室内点云的对称性看不出来，直到做第一人称全景模式才发现天花板/
地板反了。

**重要说明**：该 bug **不影响** F1/位姿MAE/召回率等 Phase5 审计数字——这些
指标都归结为欧氏距离或 AABB 包含判断，且是在记忆库和真值**同步**做过一致
变换之后计算的，正交变换保距离，只影响人眼可见的可视化朝向。是一个"看似
严重、实则不影响核心指标"的典型案例，记录下来避免以后重复排查。

### 4.3 embedding 与 embedding_model

- 直接用 ConceptGraphs 输出的 **512维 CLIP 图像特征**作为
  `Detection.embedding`，不用框架默认的 `toy_embed`。
- **`embedding_model` 字段的补丁式修正**：`consolidator._do_add()` 建
  `ObjectInstance` 时不会设置这个字段（`Detection` 本身没有该字段），会
  残留默认值 `"toy-trigram@v0"`——在真实 CLIP 向量的语境下这是一句谎言。
  脚本在入库完成后**手动二次遍历**所有实体，把 `embedding_model` 改写成
  `"ViT-B-32/laion2b_s34b_b79k"`，属于补丁式修正，代码注释里已说明原因。

### 4.4 一个更本质的架构坑：vindex 不持久化

`NumpyVectorIndex` 是纯内存结构（`store.py` 里 `_vecs: dict[str, np.ndarray]`），
SQLite 只落实体快照，不落向量（见 §2.1）。这意味着：

> 新进程重新 `build_system()` 指向一个已有 db 文件时，向量索引是空的——
> 语义检索会**静默**返回空结果，不会报错。

`p5_ingest_4dkankan.py` 和 `p5_audit_4dkankan.py` 最初是两次独立进程运行，
第一次就踩了这个坑（数据库里明明有 139 条实体，检索却什么都搜不到）。后来
把入库逻辑重构成可调用的 `ingest(db_path, fresh=True) -> SpatialMemorySystem`
函数，审计脚本改为在**同一进程**里先调用 `ingest()` 拿到活的
`SpatialMemorySystem`（含填好的 vindex），再在其上做检索。

这是 M0 框架本身的设计现状（`store.py` 里 `VectorIndex` 接口本就标注 M2 才
换 pgvector/Faiss），不是这次适配代码的 bug，但对任何后续要"入库 + 检索"
两步都做的脚本都是必须注意的坑。

`ingest()` 函数签名与语义：

```python
def ingest(db_path: str | None = None, fresh: bool = True):
    """fresh=True（默认）会先删除已存在的 DB 文件，避免重复运行时
    uuid 不同导致 upsert_object 无法按内容去重、造成事件堆叠。"""
```

### 4.5 入库结果

- 139 个实例全部作为独立 `ADD` 事件写入，**0 次误合并**——因为都是不同
  位置的真实检测，几何门控（1.5m）自然就把它们分开了，不像 ConceptGraphs
  内部合并那样容易撞在一起。
- 每个 zone 对应一条 `ObservationEvent`（把该 zone 内全部实例的
  `Detection` 一次性打包），`view_center` = zone 聚类中心，`view_radius=6.0`。

---

## 5. 小结：这套 schema 目前验证到了什么程度

- **契约与存储层**（§1、§2）本身经受住了 139 个真实实例、8 个 Place 的入库
  测试，SQLite 快照+事件日志的设计没有暴露问题。
- **入库决策逻辑**（§3）在 4dkankan 数据上 0 误合并，说明几何门控在"不同
  位置真实检测"场景下工作正常；但这套逻辑**没有**在"同一物体多次观测、
  需要正确 CONFIRM/UPDATE 而非重复 ADD"的真实时序场景下测试过——4dkankan
  是一次性全量导入，不含时序重复观测，这部分覆盖率是空的。
- **两个已知的框架级坑**（vindex 不持久化、embedding_model 默认值不同步）
  在真实数据落地时才暴露，均已在适配脚本层打补丁绕过，根治需要等 M1/M2
  升级 `VectorIndex` 实现、或给 `Detection`/`_do_add` 补上
  `embedding_model` 透传。
- hera 自采数据尚未真正走过这条入库路径，一旦 Phase1-3 几何质量达标、
  Phase4 实例数量足够，还需要重新验证：真实时序重复观测下的
  ADD/UPDATE/CONFIRM/DECAY 分支是否符合预期（4dkankan 这次完全没测到）。
