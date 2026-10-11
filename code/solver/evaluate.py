# -*- coding: utf-8 -*-
"""M4 对比实验：ALNS vs 贪婪基线，同一问题实例、同一指标口径。

指标：总成本 = 延迟惩罚 + 行驶时间成本；另报 延迟分钟 / 超时单数 / 行驶分钟。
用法：python solver/evaluate.py [--iters 200] [--seed 42]
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from problem import load_problem
from network import TimeMatrix
from batching import run_stage1
from alns import alns, set_drone_layer
from baseline import greedy_baseline
from drone_flow import rebalance, concurrency_check


def count_late(problem, sol):
    """超时单数（完成时刻 > due）。"""
    late = 0
    for rid, seq in sol.assign.items():
        rider = sol._rider(rid)
        cur_lon, cur_lat, t = rider.lon, rider.lat, rider.free_min
        for bi in seq:
            b = sol.bundles[bi]
            pad_s = sol.p.pads[b["pad_start"]]
            from alns import _virt
            t += (sol.tm.order_order_km(_virt((cur_lon, cur_lat)), b["orders"][0]) / 25.0 * 60)
            for o in b["orders"]:
                t = max(t, o.ready_min) + 3.0
            t += sol.tm.leg1(b["orders"][0], pad_s) - 3.0
            t1 = t
            for o in b["orders"]:
                pad_e = sol.p.pads[o.pad_end]
                finish = t1 + sol.tm.leg2(pad_s, pad_e) + sol.tm.leg3(pad_e, o)
                if finish > o.due_min + 1e-9:
                    late += 1
            cur_lon, cur_lat = pad_s.lon, pad_s.lat
    return late


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=200)
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    p = load_problem()
    tm = TimeMatrix(p)
    orders, bundles = run_stage1(p, tm)
    print(f"实例：订单 {len(orders)}（适航），bundles {len(bundles)}，骑手 {p.n_riders}，无人机 {p.n_drones}\n")

    # 无人机层：重定位成本 + 机队并发校验（对两种策略是同一常数项，单独列报）
    rb = rebalance(p, tm, bundles)
    cc = concurrency_check(p, tm, bundles)
    set_drone_layer(p, tm, bundles)   # 统一注入总成本口径，贪婪/ALNS 一致
    print(f"无人机重定位：空飞 {rb['cost']:.0f} min，搬运 {len(rb['moves'])} 条；"
          f"并发峰值 {cc['peak']}/{p.n_drones}，顺延 bundle {len(cc['delay_map'])} 个\n")

    t0 = time.time()
    gsol = greedy_baseline(p, tm, bundles)
    gcost, gdet = gsol.evaluate()
    glate = count_late(p, gsol)
    t_greedy = time.time() - t0

    t0 = time.time()
    bsol, bcost, bdet = alns(p, tm, bundles, iters=args.iters, seed=args.seed)
    blate = count_late(p, bsol)
    t_alns = time.time() - t0

    hdr = f"{'策略':<10}{'总成本':>12}{'延迟惩罚':>12}{'行驶min':>10}{'超时单':>8}{'用时s':>8}"
    print(hdr)
    print("-" * len(hdr))
    print(f"{'贪婪基线':<10}{gcost:>12.1f}{gdet['delay']:>12.1f}{gdet['travel_min']:>10.1f}{glate:>8d}{t_greedy:>8.1f}")
    print(f"{'ALNS':<10}{bcost:>12.1f}{bdet['delay']:>12.1f}{bdet['travel_min']:>10.1f}{blate:>8d}{t_alns:>8.1f}")
    imp = (1 - bcost / gcost) * 100 if gcost else 0
    print(f"\nALNS 相对贪婪基线总成本改善：{imp:.1f}%")


if __name__ == "__main__":
    main()
