# 第三方参考数据规格（4dkankan / realsee）

> 本文档描述用于**解耦验证**的第三方数据来源：四维看看（4dkankan）与
> 如视/贝壳（realsee）。二者均非本项目采集，而是从厂商公开 Web 端逆向抓取的
> **已加工产物**，不含任何原始传感器数据。用途是把"下游算法（ConceptGraphs
> 实例提取、空间记忆入库审计）本身的能力上限"与"我方 hera 采集链路的几何质量
> 问题"解耦开来判断。自有数据规格见 `docs/hera_data_spec.md`。

---

## 0. 与 hera 数据的本质差异

| | hera（自有，对照组） | 4dkankan / realsee（第三方） |
|---|---|---|
| 数据性质 | 原始传感器数据（点云+IMU+视频） | 厂商云端处理后的**成品**（位姿+网格+贴图） |
| 采集硬件 | Livox Mid-360 + Insta360，我方完全掌控 | 未知专有设备，我方无从获取硬件细节 |
| 获取方式 | 现场实采 | Playwright 拦截网页端网络请求 |
| 位姿 | 需自己跑 SLAM 解算，存在漂移误差 | 厂商已解算好，直接可用、精度高 |
| 深度 | 无深度传感器，靠点云投影渲染，空洞率高 | 无深度传感器，但靠完整三角网格 raycasting 渲染，空洞率极低 |
| 能验证什么 | 硬件采集 + SLAM + 标定的真实链路质量 | 算法（ConceptGraphs / Phase5 审计）在"理想干净输入"下的能力上限 |
| 局限 | 样本量小、几何质量是瓶颈 | 完全绕过传感器层，测不出真实部署会遇到的稀疏/噪声/漂移问题 |

---

## 1. 四维看看（4dkankan）

### 1.1 数据来源与合规范围

- 官方公开 Demo：`https://www.4dkankan.com/smobile.html?m=MTVUOIzO6U&lang=zh`
- 抓取流程固化在 `~/.claude/skills/4dkankan-pipeline.md`
- 本地已解码产物路径：`/home/fred/Code/mvp2/spatialAI/capture/MTVUOIzO6U/`
- **合规范围**：官方公开 Demo、自用、不重新分发（skill 文档中明确的边界）

### 1.2 采集硬件

未知（4dkankan 专有商用扫描设备，型号不掌握）。我方拿到的是设备原始数据经厂商
云端处理后的成品，无法反推硬件规格。

### 1.3 传感器数据种类

| 类型 | 是否提供 | 说明 |
|---|---|---|
| 原始点云 | ❌ | 只有已三角化的 mesh |
| IMU | ❌ | 完全不提供 |
| 位姿 | ✅ | `vision.modeldata`（Protobuf，无官方 .proto，靠字节分析逆向） |
| 全景/RGB | ✅ | `{uuid}_skybox{0-5}.jpg`，512×512/面 |
| 深度图 | ⚙️ 派生 | 无原生深度，靠对 mesh 做 Open3D `RaycastingScene` 光线求交渲染得到 |
| 网格 | ✅ | `.dam` 格式（自定义 protobuf，`binary_mesh` schema 已逆向解出） |

### 1.4 数据格式细节

**位姿（`vision.modeldata`，Protobuf 逆向 schema）**：

```
NavigationInfo { repeated SweepEntry sweeps; }
SweepEntry {
  uuid: string(36字符)
  SweepTransform { quaternion: 4×float32(qx,qy,qz,qw); position: 3×float32(x,y,z, Z-up, z≈1.55m) }
  floor_position: 3×float32(x,y,z_floor≈0)
  floor_id, type: varint
  visibles[]: repeated varint  ← 邻接点位索引，直接编码导航图
}
```

坐标系为 **Z-up**，使用时需转换为 Y-up：`x_yup=x, y_yup=z, z_yup=-y`。

**网格（`.dam`）**：

```protobuf
message binary_mesh {
  repeated chunk_simple    chunk           = 1;
  repeated chunk_quantized quantized_chunk = 2;
}
message vertices_simple { repeated float xyz=1[packed]; repeated float uv=2[packed]; }
message faces_simple    { repeated uint32 faces=1[packed]; }
```

schema 是从主 JS bundle 里的 base64 变量 `damPro` 解码得到，无官方文档。

**全景**：`{uuid}_skybox{0-5}.jpg`，6 面等大小 cubemap，物理朝向需肉眼核对内容确定
（本项目确认 skybox0=天花板/"上"，skybox5=地板/"下"，1-4 为水平环向 90° 间隔）。

**导出为标准 bundle**（`scripts/p4_adapt_4dkankan.py`）：转换成与 hera Phase4
完全相同的 `rgb/ + depth/ + poses.json + intrinsics.json` 格式，以便复用同一个
ConceptGraphs FC 端点。深度改用 mesh raycasting 而非厂商轻量化的 `scene.ply`
点云溅射（后者空洞率高达 77%-99%，因为只是给网页展示用的稀疏下采样点）。

### 1.5 已用规模与实测结果

- `scene_geo.obj`：96,561 顶点 / 65,990 三角面（Y-up，米制）
- `poses.csv`：63 个总点位，其中 33 个含完整 6 面 skybox 可用
- 已跑：33 点位、约 175 帧 → 深度空洞率 **0.1%-8.2%**（远优于 hera 的 8%-88%）
- ConceptGraphs 提取：139 个实例、20 个类别（lamp×47、chair×36、desk×14 等）
- 人工抽样核对类别准确率：**87.5%**（8 样本中 7 个类别判断正确）
- Phase5 四层审计（29 条独立反投影标注真值）：严格标签匹配 F1 仅 **0.083-0.095**，
  位姿 MAE=0.606m，归属准确率 0.857——核心发现是**系统性漏检**（如 plant 类别
  3 条真值全部未命中、玻璃后的电视未识别、air conditioner 从未出现），而非算法
  分类不准
- 已知失败模式：长走廊+大量外观相似物体（如成排吊灯）会触发 ConceptGraphs
  的"链式合并"（chaining），把整条走廊的灯合并成 1 个跨度 11m×3m×10.3m 的异常实例

---

## 2. 如视 / 贝壳（Realsee）

### 2.1 数据来源与现状

- 抓取流程固化在 `~/.claude/skills/realsee-pipeline.md`（与 4dkankan 同一套
  Playwright 拦截方法论）
- **当前状态：尚未实际执行采集，本项目里没有落地的 realsee 数据**。
  `work/phase4_realsee_validation/` 目录名带 "realsee" 但实际内容全部是
  4dkankan 的产物（命名历史遗留，非数据来源错误）——引用时需注意区分。

### 2.2 采集硬件

未知（如视/贝壳专有深度相机设备，型号不掌握），同样是纯 Web 端产物。

### 2.3 传感器数据种类（按 pipeline 脚本设计，尚未验证）

| 类型 | 是否提供 | 说明 |
|---|---|---|
| 原始点云 | ❌ | 只有网格 |
| IMU | ❌ | 完全不提供 |
| 位姿 | ✅ | `model.json`，JSON 明文，比 4dkankan 更易解析 |
| 全景/RGB | ✅ | cubemap JPG，命名 `{capture_index}_{point_id}_{face}.jpg` |
| 深度图 | ⚙️ 派生（同 4dkankan 思路，尚未执行） | 可用 OBJ mesh raycasting 渲染 |
| 网格 | ✅ | **标准格式**：OBJ + MTL + 4 张纹理 JPG，无需逆向解码 |

### 2.4 数据格式细节

**位姿（`model.json`）**：

```python
data["observers"]           # 点位列表
obs["position"]             # {x, y, z}，Y-up，米制，y≈1.5m（相机高度）
obs["quaternion"]           # {x, y, z, w}
obs["floor_index"]          # 楼层 ID
```

导航图无显式邻接列表（不同于 4dkankan 的 `visibles[]`），需按点位间距离 ≤4.5m
自动连边。

**网格**：`scene.obj`（三角形网格，Y-up）+ `scene.mtl`（材质）+
`texture_0~3.jpg`（UV 纹理图集），标准 Wavefront 格式，可直接被 Three.js
`OBJLoader`/`MTLLoader` 或 Open3D 读取，**无需任何逆向工程**——这是相对
4dkankan 的主要优势（4dkankan 的 `.dam` 需要逆向 protobuf schema）。

**全景**：`{capture_index}_{point_id}_{face}.jpg`，face ∈ {f,b,l,r,u,d}。
存储时图像上下翻转，且 Three.js 加载时 d/u 面需要互换（GPU 贴图采样约定，
和"物理朝向"是两回事，做深度渲染时不要套用这个翻转表）。

### 2.5 与 4dkankan 的对比优势

- 网格/纹理是标准格式，解析成本远低于 4dkankan 的自定义 `.dam` protobuf
- 位姿是明文 JSON，字段语义清晰（`floor_index` 直接给出楼层）
- **代价**：目前完全没有落地数据，`scripts/p4_adapt_4dkankan.py` 里预留的
  "可复用于其他四维看看/如视场景"的适配思路尚未在 realsee 上实际验证过

---

## 3. 小结：如何解读用这两类第三方数据跑出来的验证结果

1. **能证明什么**：ConceptGraphs FC 端点本身工作正常，喂给它干净、密集的深度
   数据时类别识别可靠（87.5%），说明 hera 数据链路里"深度稀疏、位置精度差"的
   问题根子在我方 Phase1-3（点云密度、外参标定），不在 GPU 推理服务本身。
2. **不能证明什么**：因为深度是"作弊"般用完整 mesh 渲染出来的，真实机器人
   传感器不会有这么干净的输入，所以这里测出的 F1/召回率数字是**能力上限**，
   不是真实部署下的预期表现；同时也测不出时间同步、IMU 漂移、外参误差这些
   只存在于真实传感器链路里的问题——这些只能靠 hera 数据本身验证。
3. 两次评测方向互补：从 4dkankan 检测结果反查照片核实（分类准确率高）与从
   真值物体反查是否被系统记住（召回率低）得出的结论并不矛盾，合起来是
   "分类准但漏检多"这一诚实画像。
