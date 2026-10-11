# -*- coding: utf-8 -*-
"""成套实验矩阵（论文图表数据源）：机队 × 骑手 × 天气 × 取消率。

四组实验（静态单周期口径，ALNS vs 贪婪）：
  E1 机队规模: fleet ∈ {6,15,30,60}，骑手 20
  E2 骑手规模: riders ∈ {10,20,40,80}，机队 30
  E3 天气:     fz_weather_factor.csv 的骑手/无人机速度折扣（小雨/中雨/高温/大风）
  E4 取消率:   fz_cancellation.csv 三档（2%/3.5%/5%），事件级剔除后重优化
输出 data/experiments/E{1..4}.csv + summary.md。
服务器 128 核：按 (实验, 配置) 两个字段并行（multiprocessing Pool）。
"""
import csv
import os
import sys
from concurrent.futures import ProcessPoolExecutor

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from problem import load_problem
from network import TimeMatrix
from batching import run_stage1
from alns import alns, set_drone_layer, DRONE_LAYER, PENALTY_PER_MIN
from baseline import greedy_baseline

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ORDERS = os.path.join(BASE, "data", "orders_expanded", "fz_orders_1550.csv")
OUTD = os.path.join(BASE, "data", "experiments")
os.makedirs(OUTD, exist_ok=True)

# 天气折扣（与 data/params/fz_weather_factor.csv 同口径，机器可读副本）
WEATHER = [("晴", 1.00, 1.00), ("小雨", 0.90, 0.85), ("中雨", 0.80, 0.60),
           ("高温35", 0.85, 0.90)]
# 取消率三档（fz_cancellation.csv）
CANCEL = [("低2%", 0.02), ("中3.5%", 0.035), ("高5%", 0.05)]

# 订单子集：控制单次实验规模（1029 适航单全量 ALNS 太慢，取样代表层）
SUBSET = 300
ITERS = 30
SEED = 2025


def load_base():
    """缓存级问题实例（订单全集 + 路网矩阵一次加载，进程内复用）。"""
    p = load_problem(orders_csv=ORDERS)
    tm = TimeMatrix(p, use_road=True)
    return p, tm


def prep_instance(p, tm, seed_shift=0, cancel_rate=0.0,
                  rider_f=1.0, drone_f=1.0, n_riders=None, n_drones=None):
    """按实验因子派生子实例：子集抽样 + 天气折扣 + 取消剔除。

    天气折扣作用于速度：骑手速度 25→25×rider_f，无人机 60→60×drone_f。
    直接改写 problem 的速度参数（Problem 无可变状态字段，用属性注入）。
    """
    import random
    rng = random.Random(SEED + seed_shift)
    orders = list(p.orders)
    rng.shuffle(orders)
    orders = orders[:SUBSET]
    if cancel_rate > 0:                       # 取消：随机剔除（事件级口径）
        orders = [o for o in orders if rng.random() > cancel_rate]
    p2 = load_problem(orders_csv=ORDERS)      # 干净副本再改，避免进程间串扰
    p2.orders = orders
    # 速度折扣：Problem 的时间原语读模块常量 → 注入实例级覆盖
    p2.rider_kmh = 25.0 * rider_f
    p2.drone_kmh = 60.0 * drone_f
    p2.n_riders = n_riders or p.n_riders
    p2.n_drones = n_drones or p.n_drones
    tm2 = TimeMatrix(p2, use_road=False)      # 子实例时间走解析式（速度已折扣）
    tm2.p = p2
    # 覆盖时间原语：直线口径 × 天气折扣（矩阵 ride_min 是基准速度，需缩放，
    # 但查表键的折扣会重复计，故子实验统一直线口径保持因子可比）
    def leg1_min(o, pad):
        from problem import road_km, RIDER_SERVICE_MIN
        return (road_km(o.pu_lon, o.pu_lat, pad.lon, pad.lat)
                / p2.rider_kmh * 60 + RIDER_SERVICE_MIN)

    def leg2_min(pa, pb):
        from problem import haversine_km, PAD_TRANSFER_MIN
        if pa.pad_id == pb.pad_id:
            return 0.0
        return (haversine_km(pa.lon, pa.lat, pb.lon, pb.lat)
                / p2.drone_kmh * 60 + PAD_TRANSFER_MIN)

    def leg3_min(pad, o):
        from problem import road_km, RIDER_SERVICE_MIN
        return (road_km(pad.lon, pad.lat, o.de_lon, o.de_lat)
                / p2.rider_kmh * 60 + RIDER_SERVICE_MIN)

    p2.leg1_min, p2.leg2_min, p2.leg3_min = leg1_min, leg2_min, leg3_min
    return p2, tm2


def run_one(args):
    """单个实验配置（子进程入口）。返回 (实验组, 配置, 指标 dict)。"""
    group, label, kw = args
    p0 = _BASE_P[0]
    tm0 = _BASE_TM[0]
    from batching import bind_pads, bundle_first_leg
    p, tm = prep_instance(p0, tm0, seed_shift=hash(label) % 1000, **kw)
    bind_pads(p, tm)
    orders, bundles = run_stage1(p, tm)
    set_drone_layer(p, tm, bundles)
    gsol = greedy_baseline(p, tm, bundles)
    gcost, gdet = gsol.evaluate()
    bsol, bcost, bdet = alns(p, tm, bundles, iters=ITERS, seed=SEED)
    return {"group": group, "config": label, "orders": len(orders),
            "bundles": len(bundles),
            "greedy_cost": round(gcost, 1), "alns_cost": round(bcost, 1),
            "alns_delay": round(bdet["delay"], 1),
            "improve_pct": round((1 - bcost / gcost) * 100, 1) if gcost else 0.0}


# 进程级缓存（Pool initializer 注入，避免每任务重载 43740 条矩阵）
_BASE_P, _BASE_TM = None, None


def _init():
    global _BASE_P, _BASE_TM
    p, tm = load_base()
    _BASE_P, _BASE_TM = (p,), (tm,)


def main():
    jobs = []
    # E1 机队规模（骑手 20 固定）
    for f in (6, 15, 30, 60):
        jobs.append(("E1_机队", f"fleet{f}", dict(n_drones=f)))
    # E2 骑手规模（机队 30 固定）
    for r in (10, 20, 40, 80):
        jobs.append(("E2_骑手", f"riders{r}", dict(n_riders=r, n_drones=30)))
    # E3 天气（机队 30 / 骑手 20）
    for w, rf, df in WEATHER:
        jobs.append(("E3_天气", w, dict(rider_f=rf, drone_f=df,
                                        n_drones=30)))
    # E4 取消率（机队 30 / 骑手 20）
    for lab, cr in CANCEL:
        jobs.append(("E4_取消", lab, dict(cancel_rate=cr, n_drones=30)))

    with ProcessPoolExecutor(max_workers=8, initializer=_init) as ex:
        results = list(ex.map(run_one, jobs))

    out = os.path.join(OUTD, "experiment_matrix.csv")
    with open(out, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=["group", "config", "orders", "bundles",
                                          "greedy_cost", "alns_cost",
                                          "alns_delay", "improve_pct"])
        w.writeheader()
        w.writerows(results)
    print(f"[OK] {out} ({len(results)} 配置)")
    for r in results:
        print(f"  {r['group']:8s} {r['config']:10s} 订单{r['orders']:4d} "
              f"贪婪{r['greedy_cost']:10.1f} ALNS{r['alns_cost']:10.1f} "
              f"改善{r['improve_pct']:6.1f}%")


if __name__ == "__main__":
    main()
