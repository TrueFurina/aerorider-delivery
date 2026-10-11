# AeroRider-Delivery：骑手-无人机协同即时配送路径优化（论文复现包）

> Code and data for the paper *"Optimization of Rider–UAV Collaborative Instant Delivery with Dynamic Time-Window Adaptability"*.
> 本仓库为论文「基于动态时间窗适配度的无人机-骑手协同即时配送路径优化研究」第5-6章（算法设计与实验分析）的可复现代码与数据。

---

## 1. 研究简介

面向景区与城区即时配送场景，研究**骑手-无人机协同配送**的路径优化问题。系统以 15 min 为滚动调度窗口，对每个订单逐单决策配送模式：

- **直送**：骑手完成取货与送达，不经过无人机；
- **三段式（Courier-Drone-Courier, C2D2C）**：首段骑手取货 → 无人机干线接驳（停机坪到停机坪）→ 末端骑手送达。

混合系统允许两种模式并存，模式由逐单决策确定；对照系统为纯骑手系统（全部订单由骑手地面配送）。核心评价指标为总配送时长、时间窗满足率、延时惩罚成本与成本结构。

数据基于福州市真实 POI（取货点/送达点/9 个候选停机坪）参数化生成，Solomon 基准用于算法对标。

## 2. 目录结构

```
├── code/
│   ├── solve_full_v4.py          # 主实验求解器（滚动时域 + 直送/三段式逐单决策）
│   ├── gen_sim_mix.py            # 混合场景订单生成器（同区/跨区，可调跨区比例）
│   ├── build_road_matrices.py    # 由真实路网（graphml）重建 OD 矩阵（替代通道）
│   ├── build_quality_report.py   # 数据质量报告
│   ├── geo_utils.py              # 坐标转换等工具
│   └── solver/                   # 算法模块（batching / drone_flow / rolling / alns 等）
├── data/
│   ├── 节点索引.csv               # OD 矩阵行列索引（取货点/送达点/停机坪 → 节点号）
│   ├── matrices/                 # 订单↔停机坪路网距离矩阵（handoff 9 坪版）
│   ├── orders_expanded/          # 订单数据集（sim_mix_600 为主实验输入）
│   ├── params/                   # 参数表（停机坪、无人机、骑手、天气/交通/取消系数、禁飞区）
│   └── benchmarks/               # Solomon 基准（R/C/RC 算例）
├── results/                      # 论文引用的全部实验结果 CSV（120/240/360/480/600 五档）
└── docs/                         # 数据字段字典、参数标定依据
```

## 3. 快速开始

```bash
pip install -r requirements.txt

# ① 生成混合场景订单（600 单，跨区比例 0.6；已随仓库提供，可跳过）
python code/gen_sim_mix.py 0.6 600

# ② 混合系统（无人机协同）主实验
python code/solve_full_v4.py 600 "" 1.0 2.0 data/orders_expanded/fz_orders_sim_mix_600.csv sim

# ③ 纯骑手系统（对照）主实验
python code/solve_full_v4.py 600 pure 1.0 2.0 data/orders_expanded/fz_orders_sim_mix_600.csv sim
```

命令行参数：`N`（订单数，默认120）、模式（`pure`=纯骑手，缺省=混合）、`TW_SCALE`（时间窗缩放，默认1.0）、`PENALTY`（延误惩罚 元/min，默认2.0）、订单CSV路径、`sim`（仿真模式）。

输出写入 `results/`，命名 `CD-C_full_{N}单.csv`（混合）与 `纯骑手_full_{N}单.csv`（纯骑手），另含 `routes`（骑手路线明细）、`uav`（无人机架次）、`handoff`（交接明细）、`decision`（逐单模式决策）四类文件。

## 4. OD 距离矩阵说明

`solve_full_v4.py` 默认读取 `data/OD道路距离_米.npy`（约 3.5 GB，浮点二维矩阵，行列与 `data/节点索引.csv` 一一对应），距离由福州市真实路网（OpenStreetMap，约 16.9 万节点/30.7 万边）Dijkstra 预计算，坐标系 GCJ-02。

- 该矩阵体积超过 GitHub 单文件限制，**不随仓库分发**；
- 若缺少该文件，求解器自动回退为「直线距离×1.35」口径（`SIM` 模式，与合成订单一致），论文 §6 两种口径的差异已在正文与数据质量报告中声明；
- 需要完整路网矩阵的读者可运行 `build_road_matrices.py`（输入真实路网 graphml 后重建订单↔停机坪矩阵），或向作者索取（Release 附件）。

## 5. 实验与结果

论文实验在 120/240/360/480/600 五个订单规模上运行，每个规模分别执行混合与纯骑手两种系统，`results/` 已随仓库提供全部 CSV。汇总口径：

| 指标 | 混合系统（CD-C） | 纯骑手系统 |
|---|---|---|
| 时间窗满足率 | 88.2%（600单） | 72.0%（600单） |
| 准时履约率 | 53.92% | 59.93% |
| 含罚总成本（元） | 24572.63 | 27940.67 |

（完整五档结果见 `results/实验结果汇总.csv` 与论文表6-4~6-7。）

## 6. 数据来源与许可

- 订单取/送点、停机坪坐标：福州市真实 POI 与《福州交接点清单》主模型节点（9 个：FZ-TP-001/003/004/005/007/011/012/018/019）；
- 路网：OpenStreetMap 福州区域（WGS-84 转 GCJ-02）；禁飞区按福州机场净空与直升机坪缓冲构建；
- Solomon 基准：SINTEF 官方 C/R/RC 算例；
- 天气/交通/取消系数：文献与行业口径标定，详见 `docs/参数标定依据.md`。

代码采用 MIT 许可；数据仅供学术研究使用，引用请注明论文与数据集来源。

## 7. 复现与引用

如需复现论文全部图表，请先核对 `results/` 与 `03_实验设计/实验结果/` 一致（数字校验脚本见 `docs/`）。论文录用后本仓库将并入作者主页公开仓库。引用格式：

> 张敏杰, 等. 基于动态时间窗适配度的无人机-骑手协同即时配送路径优化研究[J]. （待发表）.
