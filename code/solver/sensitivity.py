# -*- coding: utf-8 -*-
"""机队规模敏感性分析：无人机数量 6/15/30/60 架对总成本的影响。

每次换机队规模必须重置 DRONE_LAYER（顺延惩罚随 fleet 变化）。
输出：机队 × (空飞成本, 顺延惩罚, 峰值并发, 顺延bundle数) 表。
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from problem import load_problem
from network import TimeMatrix
from batching import run_stage1
from drone_flow import rebalance, concurrency_check
import alns as A

FLEETS = [6, 15, 30, 60]


def main():
    p = load_problem()
    tm = TimeMatrix(p)
    orders, bundles = run_stage1(p, tm)
    print(f"实例：订单 {len(orders)}，bundles {len(bundles)}\n")

    hdr = (f"{'机队':>4}{'空飞min':>10}{'空飞成本':>10}{'顺延惩罚':>10}"
           f"{'峰值并发':>10}{'顺延bundle':>12}")
    print(hdr)
    print("-" * len(hdr))
    for f in FLEETS:
        rb = rebalance(p, tm, bundles)
        cc = concurrency_check(p, tm, bundles, fleet=f)
        rc = rb["cost"] * A.TRAVEL_COST_PER_MIN
        dp = sum(cc["delay_map"].values()) * A.PENALTY_PER_MIN
        print(f"{f:>4d}{rb['cost']:>10.0f}{rc:>10.1f}{dp:>10.1f}"
              f"{cc['peak']:>10d}{len(cc['delay_map']):>12d}")
    print("\n注：空飞成本与机队规模无关（重平衡由 bundle 起终点决定）；"
          "顺延惩罚随机队增大而下降，可据此定最小经济机队规模。")


if __name__ == "__main__":
    main()
