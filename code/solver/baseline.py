# -*- coding: utf-8 -*-
"""Baseline：贪婪就近分配（订单到达 → 就近空闲骑手），与 ALNS 同口径评估。

贪婪策略：按 ready 时刻排序 bundle，逐个分给「插入成本最小」的骑手（尾插），
不做任何重优化。这就是文档要求的对照基线。
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from alns import make_riders, _rids, _insert_cost, Solution


def greedy_baseline(problem, tm, bundles):
    make_riders(problem)
    sol = Solution(problem, tm, bundles)
    pending = sorted(sol.unassigned, key=lambda bi: bundles[bi]["orders"][0].ready_min)
    for bi in pending:
        best = min(((_insert_cost(sol, bi, r), r) for r in _rids(sol)),
                   default=(float("inf"), None))
        if best[1] is not None:
            sol.assign[best[1]].append(bi)
            sol.unassigned = [u for u in sol.unassigned if u != bi]
    return sol
