# -*- coding: utf-8 -*-
"""Stage 2-1：无人机时空流重定位（MVP：静态重平衡 + 机队并发校验）。

设计文档要求 TEN + MinCostFlow。OR-Tools 在 Python 3.14 无 wheel，
故 MCMF 用纯 Python SSP（SPFA 找增广路）实现——网络经剪枝后规模小，
多项式可解，语义与文档一致：
  - 载货飞行把无人机从起始坪「搬运」到终点坪 → 各坪出现盈余/缺口；
  - 空载重定位 = 盈余坪 → 缺口坪的最小费用运输（费用 = 空飞分钟）；
  - 机队并发校验 = 任一 5min 桶内「在飞无人机数」不得超过机队规模，
    超出的 bundle 顺延（延迟计入惩罚）。

滚动时域版再细化到逐时刻 TEN；本模块是静态近似，接口已按文档输出：
每 bundle 的无人机到达终点坪时刻 + 各坪供需状态。
"""
from collections import defaultdict, deque


class MCMF:
    """最小费用流（SSP，SPFA 增广）。小规模网络够用，纯 Python 无依赖。"""

    def __init__(self, n):
        self.n = n
        self.g = [[] for _ in range(n)]

    def add(self, u, v, cap, cost):
        self.g[u].append([v, cap, cost, len(self.g[v])])
        self.g[v].append([u, 0, -cost, len(self.g[u]) - 1])

    def run(self, s, t):
        flow = cost = 0
        INF = float("inf")
        while True:
            dist = [INF] * self.n
            inq = [False] * self.n
            prev = [(-1, -1)] * self.n
            dist[s] = 0
            dq = deque([s])
            inq[s] = True
            while dq:
                u = dq.popleft()
                inq[u] = False
                for i, e in enumerate(self.g[u]):
                    v, cap, c, _ = e
                    if cap > 0 and dist[u] + c < dist[v]:
                        dist[v] = dist[u] + c
                        prev[v] = (u, i)
                        if not inq[v]:
                            dq.append(v)
                            inq[v] = True
            if dist[t] == INF:
                return flow, cost
            # 沿最短路推满
            push, v = INF, t
            while v != s:
                u, i = prev[v]
                push = min(push, self.g[u][i][1])
                v = u
            v = t
            while v != s:
                u, i = prev[v]
                e = self.g[u][i]
                e[1] -= push
                self.g[e[0]][e[3]][1] += push
                v = u
            flow += push
            cost += push * dist[t]

    def flow_on(self, u, v):
        """正向弧 (u,v) 的实际流量 = 反向边的容量。"""
        for e in self.g[u]:
            if e[0] == v:
                return self.g[v][e[3]][1]
        return 0


def rebalance(problem, tm, bundles):
    """空载重平衡：盈余坪 → 缺口坪最小费用运输。

    返回 dict(cost=空飞分钟, moves=[(from_pad, to_pad, 数量)], balance={pad: 净缺口})
    """
    net = defaultdict(int)          # pad -> 结束时无人机净增量
    for b in bundles:
        ends = {o.pad_end for o in b["orders"]}
        starts = {b["pad_start"]}
        for s in starts:
            net[s] -= 1             # 需要一架无人机在起始坪（被载走）
        for e in ends:
            net[e] += 1             # 送达后无人机停在终点坪

    surplus = [(p, -k) for p, k in net.items() if k < 0]   # 缺口（需要调入）
    deficit = [(p, k) for p, k in net.items() if k > 0]    # 盈余（可调出）
    if not surplus or not deficit:
        return {"cost": 0.0, "moves": [], "balance": dict(net)}

    # 二部最小费用运输：源点=盈余坪，汇点=缺口坪，弧费用=空飞时间
    pad_by_id = problem.pads
    ids = [p for p, _ in deficit] + [p for p, _ in surplus]
    idx = {p: i for i, p in enumerate(ids)}
    n_def, n_sur = len(deficit), len(surplus)
    S, T = len(ids), len(ids) + 1
    mc = MCMF(len(ids) + 2)
    for p, k in deficit:                       # 盈余供给
        mc.add(S, idx[p], k, 0)
    for p, k in surplus:                       # 缺口需求
        mc.add(idx[p], T, k, 0)
    for ps, _ in deficit:
        for pd, _ in surplus:
            pa, pb = pad_by_id[ps], pad_by_id[pd]
            if pa is pb:
                continue
            c = max(1, int(round(tm.leg2(pa, pb))))    # 空飞分钟为费用
            mc.add(idx[ps], idx[pd], 10 ** 6, c)
    _, mc_cost = mc.run(S, T)
    moves = []
    for ps, _ in deficit:
        for pd, _ in surplus:
            f = mc.flow_on(idx[ps], idx[pd])
            if f > 0:
                moves.append((ps, pd, f))
    return {"cost": float(mc_cost), "moves": moves, "balance": dict(net)}


def concurrency_check(problem, tm, bundles, fleet=None, bucket=5.0):
    """机队并发校验：任一 bucket 分钟内「在飞」无人机数 ≤ 机队规模。

    超出的 bundle 顺延到下一个有空位的 bucket（返回每 bundle 的起飞推迟分钟）。
    返回 dict(delay_map={bundle_idx: 推迟min}, peak=峰值在飞数)
    """
    fleet = fleet or problem.n_drones
    # 每 bundle 的在飞区间 [t1, t1+leg2]（近似：leg1 起算即取货）
    spans = []
    for bi, b in enumerate(bundles):
        o = b["orders"][0]
        t1 = max(o.ready_min, 0.0) + 3.0 + 1.0
        t2 = t1 + tm.leg2(problem.pads[b["pad_start"]],
                          problem.pads[o.pad_end])
        spans.append([bi, t1, t2])
    spans.sort(key=lambda x: x[1])
    delay = {}
    # 事件扫描：按时间桶计当前在飞数
    active = []   # (t2, bi)
    peak = 0
    for bi, t1, t2 in spans:
        active = [x for x in active if x[0] > t1 + delay.get(bi, 0)]
        if len(active) >= fleet:
            # 顺延到最早释放时刻
            active.sort()
            t_free = active[0][0]
            d = t_free - t1
            delay[bi] = d
            t2 = t1 + d + (t2 - t1)
        active.append((t2, bi))
        peak = max(peak, len(active))
    return {"delay_map": delay, "peak": peak}
