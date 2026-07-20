# spatial-memory — 空间记忆框架 (M0)

一个**可运行的最小空间记忆系统**:麻雀虽小,五脏俱全。
它不是原型玩具,而是生产系统的**接口骨架**——所有模块边界按最终形态切分,
M0 用最简实现填充,后续按 [ROADMAP.md](ROADMAP.md) 逐个替换为生产实现。

## 30 秒跑起来

```bash
pip install numpy
python demo.py
```

演示一条完整的记忆生命周期:机器人巡逻建图 → 环境变化(物体被挪动/门被关上/
椅子被搬走)→ 记忆自动更新与遗忘 → 语义检索 / 时间旅行查询 / 事件审计 /
人工修正 / 多机增量同步。

## 架构

```
                      写路径                          读路径
  机器人/感知 ──ObservationEvent──▶ Consolidator      SpatialQuery ◀── 引擎层/应用层
  (M0: 合成世界)                    (整合器)              (查询门面)
                                      │  ▲                 │
                          五种决策     │  │ 匹配候选         │ 四类原语
                 ADD/UPDATE/CONFIRM   ▼  │                 ▼
                 DECAY/RETIRE   ┌──────────────────────────────┐
                                │ EntityStore   实体快照(现状)  │  M0: SQLite
                                │ EventLog      事件日志(历史)  │  M0: SQLite
                                │ VectorIndex   语义索引        │  M0: numpy
                                │ BlobStore     L0 几何大对象   │  M0: 本地目录
                                └──────────────────────────────┘
```

核心设计决策(与 LLM memory 前沿对齐):

| 决策 | 来源范式 | 落点 |
|---|---|---|
| 写入 = 提取→比对→五种操作 | Mem0 的 ADD/UPDATE/DELETE/NOOP | `consolidator.py` |
| 实体表 = 事件流的物化快照 | 事件溯源 | `store.py` 双表 |
| 负观测衰减 + 按 mobility 分级遗忘 | A-MEM 的记忆过时处理 | `consolidator._do_decay` |
| 人工修正 pin 保护 + provenance | 人机协作记忆 | `human_correct()` |
| 位姿相对子图,回环只改锚点 | SLAM 工程实践 | `schema.Submap` |
| 按 (submap, version) 增量同步 | 分布式日志 | `query.changes_since` |

## 文件导览

```
spatial_memory/
  schema.py        数据契约(全系统唯一事实来源,M1 迁 Protobuf)
  store.py         四个存储接口 + M0 实现(替换点全在这里)
  consolidator.py  记忆整合器 —— 最值得持续自研的模块
  query.py         查询门面:结构化/语义/时序/同步 四类原语
  synthetic.py     合成世界与模拟机器人(真实感知接入后退役)
demo.py            端到端演示 + 回归测试
ROADMAP.md         M0→M4 逐步填充路径
```

## 如何开始探索(给团队的三个入口)

1. **感知同学**:让你的真实管线产出 `ObservationEvent`(见 schema.py),
   直接喂给 `consolidator.ingest()`——这就是 M1 的全部对接工作。
2. **系统同学**:实现 `EntityStore`/`VectorIndex` 的 PG/pgvector 版本,
   跑通 demo.py 即为验收——接口就是规格说明。
3. **算法同学**:改 `Consolidator._match` 和 `ConsolidatorConfig`,
   用 synthetic.py 构造更刁钻的世界演化脚本做对抗测试。
```
