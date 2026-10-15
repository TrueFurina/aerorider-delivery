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

# ① 重新采集订单已随仓库提供（data/orders_expanded/fz_orders_recollect_600.csv，600 单，可直接使用）

# ② 混合系统（无人机协同）主实验（v5，含消融开关第 9 参数：no_alns / no_ten / none）
python code/solve_full_v5.py 600 cdc 1.0 2.0 data/orders_expanded/fz_orders_recollect_600.csv sim 7 50

# ③ 纯骑手系统（对照）主实验
python code/solve_full_v5.py 600 pure 1.0 2.0 data/orders_expanded/fz_orders_recollect_600.csv sim 7 50
```

命令行参数：`N`（订单数）、模式（`pure`=纯骑手，缺省=混合）、`TW_SCALE`（时间窗缩放，默认1.0）、`PENALTY`（延误惩罚 元/min，默认2.0）、订单CSV路径、`sim`（仿真模式）、`seed`（默认7）、`FIX_W`（ALNS 目标中路线数等效里程权重，实验取值50）、第9参数（消融开关，默认空=完整版）。

输出写入 `results/`，命名 `CD-C2_v5_full_{N}单.csv`（混合）与 `纯骑手_v5_full_{N}单.csv`（纯骑手），另含 `routes`（骑手路线明细）、`uav`（无人机架次）、`decision`（逐单模式决策）等文件。

> **版本对应关系**：`results/` 当前均为 `*_v5_*`（**2026-10-15 重新采集版**，老师指令"重新采集6小时内的600个订单"后的重算结果，CP-SAT 确定性修复后权威值），**论文表 6-4~6-8 引用 v5**；`*_v4_*` 历史版本已移入项目回收站 `回收站_20261016/release_v4旧结果/`（需要时可索取）。v5 特点：① 订单数据 `fz_orders_recollect_600.csv` 坐标全部取自福州市真实 POI 母本（`fz_orders_1550.csv`），限定取送点与任一交接点直线距离 ≤4 km 以保障三段式协同可达；② 下单时刻 660~1019 min（11:00~16:59）按 24 个 15 min 桶均匀分布（每桶 25 单）；③ 全量落水验证 0/1200、5×5 邻域 robust 0；④ 订单已脱敏（REC 前缀）。复现 v5 请使用 `code/solve_full_v5.py` 与 `data/orders_expanded/fz_orders_recollect_600.csv`，停机坪数据源 `data/params/fz_handoff_nodes_master_fixed.csv`（9 坪全陆地）。

## 4. OD 距离矩阵说明

`solve_full_v5.py` 默认读取 `data/matrices/OD道路距离_米.npy`（约 3.5 GB，浮点二维矩阵，行列与 `data/节点索引.csv` 一一对应），距离由福州市真实路网（OpenStreetMap，约 16.9 万节点/30.7 万边）Dijkstra 预计算，坐标系 GCJ-02。

- 该矩阵体积超过 GitHub 单文件限制，**不随仓库分发**（`data/matrices/OD道路距离_米.npy` 需自行放入，或使用 `build_road_matrices.py` 重建，或向作者索取 Release 附件）；
- 论文 §6 各表数字即基于该路网矩阵口径复现；若以「直线距离×1.35」回退口径运行，数值将与论文不同。

## 5. 实验与结果

论文实验在 120/240/360/480/600 五个订单规模上运行，每个规模分别执行混合与纯骑手两种系统，`results/` 已随仓库提供全部 CSV（含消融实验 no_alns / no_ten / 构造式基线）。汇总口径（600 单，v5 重新采集版，SEED=7，CP-SAT 确定性修复后）：

| 指标 | 混合系统（CD-C） | 纯骑手系统 |
|---|---|---|
| 时间窗满足率 | 80.2%（600单） | 67.5%（600单） |
| 平均履约时间 | 57.73 min | 62.62 min |
| 含罚总成本（元） | 29189.15 | 32050.94 |

> **可复现性说明（2026-10-15 晚）**：核验发现 `solve_full_v5.py` 中 CP-SAT 求解器默认多线程且未固定随机种子，导致含 TEN 的实验组存在不可复现漂移；已修复为 `num_search_workers=1` + `random_seed=SEED`（代码第 411-416 行）并双重验证（cdc 600/cdc 120 逐 key 一致）后重算。上表及 `results/` 下 `*_v5_*` 全部 CSV 均为修复后**最终权威值**；论文表 6-4~6-8 与正文同步引用该口径。

（完整五档结果见 `results/` 下 `*_v5_full_*.csv` 与论文表6-4~6-8；`*_v4_*` 为投影修正前的历史版本，保留供追溯。）

## 6. 数据来源与许可

- 订单取/送点、停机坪坐标：福州市真实 POI 与《福州交接点清单》主模型节点（9 个：FZ-TP-001/003/004/005/007/011/012/018/019）；
- 路网：OpenStreetMap 福州区域（WGS-84 转 GCJ-02）；禁飞区按福州机场净空与直升机坪缓冲构建；
- Solomon 基准：SINTEF 官方 C/R/RC 算例；
- 天气/交通/取消系数：文献与行业口径标定，详见 `docs/参数标定依据.md`。

代码采用 MIT 许可；数据仅供学术研究使用，引用请注明论文与数据集来源。

## 7. 复现与引用

如需复现论文全部图表，请先核对 `results/` 与论文实验输出一致。论文录用后本仓库将并入作者主页公开仓库。引用格式（录用后回填作者与出处）：

> 作者. 基于动态时间窗适配度的无人机-骑手协同即时配送路径优化研究[J]. （待发表）.
