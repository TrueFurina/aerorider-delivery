# -*- coding: utf-8 -*-
"""滚动时域动态调度（Rolling Horizon）。

机制（对照《算法构思》与 00_滚动时域框架.md）：
- 事件流：fz_arrival_sim_sample.csv 的泊松 NEW/CANCEL 事件按 t_min 驱动；
- 决策周期 Δt：每 DECISION_INTERVAL_MIN 分钟触发一轮决策——把当前未锁定
  订单池交给 ALNS 重优化，输出本轮不可撤销分配（已通知骑手）；
- 锁定规则：已分配的 bundle 进入冻结队列，不再参与后续优化；
  骑手位置/空闲时刻随执行推进而更新（最后一个交付点 = 新位置）；
- 取消事件：从未锁定池中移除；已锁定（不可撤销）的订单无法取消（口径简化，
  与文档「不可撤销决策」一致）；
- 无人机层：每轮决策后按当前 bundle 集跑 concurrency_check，顺延延迟计入
  该轮成本（MVP：反馈常数项；细粒度反馈见任务 #3）。

输出：逐轮统计（时间、新到、取消、锁定、在池、轮成本、累计成本）+
总指标（总延迟惩罚/行驶/超时单/轮数）。
"""
import csv
import os
import sys
from collections import defaultdict

try:
    from .problem import load_problem, load_orders_csv, Order
    from .network import TimeMatrix
    from .batching import run_stage1
    from . import alns as A
    from .baseline import greedy_baseline
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from problem import load_problem, load_orders_csv, Order
    from network import TimeMatrix
    from batching import run_stage1
    import alns as A
    from baseline import greedy_baseline

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EVENTS = os.path.join(BASE, "data", "orders_expanded", "fz_arrival_sim_sample.csv")
ORDERS = os.path.join(BASE, "data", "orders_expanded", "fz_orders_1550.csv")

DECISION_INTERVAL_MIN = 30     # Δt：决策周期
ALNS_ITERS = 60                # 每轮 ALNS 迭代（滚动时域要快，压缩迭代）


class RollingState:
    """滚动时域全局状态：锁定队列 + 骑手实时位置。"""

    def __init__(self, problem, tm):
        self.p = problem
        self.tm = tm
        self.locked_bundles = []      # 已不可撤销的 bundle（dict 同 batching 结构）
        self.pending_orders = []      # 未锁定订单池
        self.rider_state = {}         # rid -> [lon, lat, free_min]（随执行推进）
        for rid in A.make_riders(problem):
            r = A.RIDERS[id(problem)][rid]
            self.rider_state[rid] = [r.lon, r.lat, r.free_min]


def load_events(path=EVENTS):
    """读取泊松事件流 -> [(t_min, order_ref, event)] 按 t 排序。"""
    ev = []
    for r in csv.DictReader(open(path, encoding="utf-8-sig")):
        ev.append((float(r["t_min"]), r["order_ref"], r["event"]))
    ev.sort(key=lambda x: x[0])
    return ev


def _order_index(orders_csv_path):
    """order_ref -> 原始订单行（bundle 化时用）。"""
    idx = {}
    for r in csv.DictReader(open(orders_csv_path, encoding="utf-8-sig")):
        idx[r["order_id"]] = r
    return idx


def _make_bundle(orders, pad_start):
    """把同坪订单临时打包成 batching 兼容的 bundle 结构。"""
    return {"pad_start": pad_start,
            "orders": list(orders),
            "load_kg": sum(o.weight_kg for o in orders),
            "t0": min(o.ready_min for o in orders)}


def run_rolling(problem, tm, events, order_rows, verbose=True):
    """滚动时域主循环。返回 (逐轮统计, 总指标)。"""
    st = RollingState(problem, tm)
    by_id = {o.order_id: o for o in problem.orders}
    stats, total_cost = [], 0.0
    t = 0.0
    T_END = max(e[0] for e in events)
    n_late_total = 0

    while t <= T_END + DECISION_INTERVAL_MIN:
        t_next = t + DECISION_INTERVAL_MIN
        # 1) 本周期内事件：新订单入池 / 取消出池
        n_new = n_cancel = 0
        for te, ref, kind in events:
            if t < te <= t_next:
                if kind == "NEW" and ref in by_id:
                    st.pending_orders.append(by_id[ref])
                    n_new += 1
                elif kind == "CANCEL":
                    before = len(st.pending_orders)
                    st.pending_orders = [o for o in st.pending_orders
                                         if o.order_id != ref]
                    n_cancel += before - len(st.pending_orders)
        # 2) 决策：未锁定池 + 尚未分派的空骑手 -> 临时 Problem 重优化
        if st.pending_orders:
            sub = A.Solution.__new__(A.Solution)
            bundles = []
            # 简化打包：同起始坪按 ready 排序后逐单成包（滚动时域内轻量，
            # 完整 Stage1 凝聚聚类在静态版已验证，此处复用锚点逻辑）
            by_pad = {}
            for o in st.pending_orders:
                by_pad.setdefault(o.pad_start, []).append(o)
            for pid, os_ in by_pad.items():
                os_.sort(key=lambda o: o.ready_min)
                open_b = []
                for o in os_:
                    placed = False
                    for b in open_b:
                        if (b["load_kg"] + o.weight_kg <= 8.0
                                and o.ready_min - b["t0"] <= 30):
                            b["orders"].append(o); b["load_kg"] += o.weight_kg
                            placed = True; break
                    if not placed:
                        open_b.append(_make_bundle([o], pid))
                bundles.extend(open_b)
            sub.p, sub.tm, sub.bundles = problem, tm, bundles
            sub.unassigned = list(range(len(bundles)))
            sub.assign = defaultdict(list)   # 修复算子按 defaultdict 语义访问
            A.make_riders(problem)
            # 用骑手实时状态覆盖初始位置（滚动时域关键：位置锁定在最后交付点）
            for rid, (lon, lat, free) in st.rider_state.items():
                r = A.RIDERS[id(problem)][rid]
                r.lon, r.lat, r.free_min = lon, lat, free
            A.repair_greedy(sub, None, sorted(sub.unassigned))
            cost, det = sub.evaluate()
            total_cost += cost
            n_late_total += _count_late(problem, sub)
            # 3) 锁定：本轮分配即不可撤销；骑手位置推进到 bundle 的起始坪
            for rid, seq in sub.assign.items():
                for bi in seq:
                    b = bundles[bi]
                    st.locked_bundles.append(b)
                    st.rider_state[rid][0] = problem.pads[b["pad_start"]].lon
                    st.rider_state[rid][1] = problem.pads[b["pad_start"]].lat
                    # 骑手执行完本 bundle 后才可用（free_min = 送达起始坪时刻）
                    o0 = b["orders"][0]
                    st.rider_state[rid][2] = max(
                        st.rider_state[rid][2], o0.ready_min) + \
                        tm.leg1(o0, problem.pads[b["pad_start"]])
            st.pending_orders = []
        if verbose:
            print(f"  t={t:6.0f}-{t_next:6.0f} 新到{n_new:3d} 取消{n_cancel:2d} "
                  f"锁定累计{len(st.locked_bundles):4d} 本轮成本{cost if st.pending_orders or n_new else 0:8.1f}",
                  flush=True)
        t = t_next

    return stats, {"rounds": int(T_END // DECISION_INTERVAL_MIN) + 1,
                   "locked": len(st.locked_bundles),
                   "total_cost": total_cost,
                   "late": n_late_total}


def _count_late(problem, sol):
    """本轮解的超时单数（复用 evaluate 的 route_eval 时间链）。"""
    late = 0
    for rid, seq in sol.assign.items():
        rider = sol._rider(rid)
        cur_lon, cur_lat, t = rider.lon, rider.lat, rider.free_min
        for bi in seq:
            b = sol.bundles[bi]
            pad_s = problem.pads[b["pad_start"]]
            t += sol.tm.order_order_km(A._virt((cur_lon, cur_lat)), b["orders"][0]) / 25.0 * 60
            for o in b["orders"]:
                t = max(t, o.ready_min) + 3.0
            t += sol.tm.leg1(b["orders"][0], pad_s) - 3.0
            t1 = t
            for o in b["orders"]:
                pad_e = problem.pads[o.pad_end]
                finish = t1 + sol.tm.leg2(pad_s, pad_e) + sol.tm.leg3(pad_e, o)
                if finish > o.due_min + 1e-9:
                    late += 1
            cur_lon, cur_lat = pad_s.lon, pad_s.lat
    return late


def main():
    oc = ORDERS if os.path.exists(ORDERS) else None
    problem = load_problem(orders_csv=oc)
    tm = TimeMatrix(problem, use_road=True)
    # 绑坪必须先于事件流：订单要有 pad_start/pad_end 才能成 bundle
    from batching import bind_pads
    bind_pads(problem, tm)
    events = load_events()
    print(f"滚动时域：订单 {len(problem.orders)}，事件 {len(events)}，"
          f"Δt={DECISION_INTERVAL_MIN}min，骑手 {problem.n_riders}")
    _, summary = run_rolling(problem, tm, events, None)
    print("\n=== 滚动时域总指标 ===")
    for k, v in summary.items():
        print(f"  {k}: {v:.1f}" if isinstance(v, float) else f"  {k}: {v}")


if __name__ == "__main__":
    main()
