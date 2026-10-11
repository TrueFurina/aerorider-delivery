# -*- coding: utf-8 -*-
"""Stage 2（MVP 核心）：ALNS 骑手排程 + 三段时间链校验。

MVP 简化口径（按算法构思的假设）：
- 无人机运力充足、快速换电 → 干线飞行不构成瓶颈，订单完成时刻由
  「首段骑手到达起始坪 t1 + leg2」决定干线送达，末端段由终点坪接续。
- 首段与末端共用同一骑手车队：骑手串行执行 bundle 队列，位置锁定在
  最后交付点（滚动时域锁定逻辑，与文档一致）。
- 目标 = 总延迟惩罚 Σ βo·max(0, 完成时刻 - due) + 行驶时间成本。

解表示：bundle -> rider 的分配 + 每个骑手的 bundle 序列。
ALNS：破坏算子（随机/最差延迟/地理聚集）× 修复算子（贪婪/Regret-2），
SA 准则接受劣解；固定种子可复现。
"""
import math
import random
from collections import defaultdict

PENALTY_PER_MIN = 2.0     # βo：超时每分钟惩罚
TRAVEL_COST_PER_MIN = 0.1  # 骑手行驶时间成本权重

# 无人机层（对给定 bundles 是常数项，与骑手分配无关）：
#   rebal_cost = 空载重定位总空飞分钟；delay_sum = 机队并发超限的顺延分钟总和。
# evaluate() 会把它们并入总成本，使 ALNS/基线口径与文档目标函数一致
# （min 运输成本 + 空载重定位成本 + 积压延迟）。
DRONE_LAYER = {}   # id(problem) -> {"rebal_cost": float, "delay_sum": float}


def set_drone_layer(problem, tm, bundles, fleet=None):
    """跑一次 drone_flow，把重定位成本与机队顺延惩罚注入评估口径。"""
    from drone_flow import rebalance, concurrency_check
    rb = rebalance(problem, tm, bundles)
    cc = concurrency_check(problem, tm, bundles, fleet=fleet)
    DRONE_LAYER[id(problem)] = {
        "rebal_cost": rb["cost"] * TRAVEL_COST_PER_MIN,   # 空飞分钟 → 时间成本口径
        "delay_sum": sum(cc["delay_map"].values()) * PENALTY_PER_MIN,
        "raw": {"rebal": rb, "conc": cc},
    }


class Solution:
    def __init__(self, problem, tm, bundles):
        self.p = problem
        self.tm = tm
        self.bundles = bundles
        self.assign = defaultdict(list)   # rider_id -> [bundle 索引（执行序）]
        self.unassigned = list(range(len(bundles)))

    def copy(self):
        s = Solution.__new__(Solution)
        s.p, s.tm, s.bundles = self.p, self.tm, self.bundles
        s.assign = defaultdict(list, {r: list(v) for r, v in self.assign.items()})
        s.unassigned = list(self.unassigned)
        return s

    # ---- 评估：三段衔接时间链 + 延迟惩罚 ----
    def route_eval(self, rid, seq, want_spans=False):
        """单骑手路线成本（延迟 + 行驶），供增量评估复用。

        want_spans=True 时同时返回本路线各 bundle 的无人机在飞区间
        [(t1, t2)]（t1=骑手送达起始坪时刻，t2=t1+leg2），供机队并发惩罚。
        """
        rider = self._rider(rid)
        cur_lon, cur_lat, t = rider.lon, rider.lat, rider.free_min
        delay, travel = 0.0, 0.0
        spans = []
        for bi in seq:
            b = self.bundles[bi]
            pad_s = self.p.pads[b["pad_start"]]
            # 骑手空驶到取货锚点 → 取货 → 到起始坪
            t += (self.tm.order_order_km(_virt((cur_lon, cur_lat)), b["orders"][0])
                  / 25.0 * 60)
            for o in b["orders"]:
                t = max(t, o.ready_min) + 3.0   # 等备 + 取件服务
            t += self.tm.leg1(b["orders"][0], pad_s) - 3.0  # 去掉 leg1 内置的取件服务（已单算）
            t1 = t
            if want_spans:
                # 干线飞行占机队：首个订单的 pad_end 决定在飞时长（同 bundle 共机）
                pad_e0 = self.p.pads[b["orders"][0].pad_end]
                spans.append((t1, t1 + self.tm.leg2(pad_s, pad_e0)))
            # 干线 + 末端：本 bundle 所有单共享干线和终点坪
            for o in b["orders"]:
                pad_e = self.p.pads[o.pad_end]
                arrive_end = t1 + self.tm.leg2(pad_s, pad_e)
                finish = arrive_end + self.tm.leg3(pad_e, o)
                delay += max(0.0, finish - o.due_min) * PENALTY_PER_MIN
            travel += self.tm.leg1(b["orders"][0], pad_s)
            cur_lon, cur_lat = pad_s.lon, pad_s.lat
        if want_spans:
            return delay, travel, spans
        return delay, travel

    def occupancy_penalty(self, spans_all, fleet=None):
        """机队并发惩罚：5min 桶内在飞数超机队的部分 × 惩罚/分钟。

        spans_all = 各骑手 spans 的并集。这是把无人机瓶颈反馈进插入决策的
        核心项：往高峰时段插 bundle 会被加罚，ALNS 自动学会错峰。
        """
        fleet = fleet or self.p.n_drones
        if not spans_all:
            return 0.0
        bucket = defaultdict(int)
        for t1, t2 in spans_all:
            b0, b1 = int(t1 // 5), int(t2 // 5)
            for b in range(b0, b1 + 1):
                bucket[b] += 1
        return sum(max(0, c - fleet) * 5.0 * PENALTY_PER_MIN
                   for c in bucket.values())   # 超限桶按整桶 5min 计

    def evaluate(self):
        """返回 (总成本, 明细)。时间链：t1 = 骑手到达起始坪；干线送达 = t1+leg2；
        完成时刻 = 干线送达 + leg3（末端骑手从终点坪送达，MVP 合并到同车队计）。"""
        total_delay, total_travel = 0.0, 0.0
        all_spans = []
        for rid, seq in self.assign.items():
            d, tr, sp = self.route_eval(rid, seq, want_spans=True)
            total_delay += d
            total_travel += tr
            all_spans.extend(sp)
        cost = (total_delay + total_travel * TRAVEL_COST_PER_MIN
                + self.occupancy_penalty(all_spans))
        dl = DRONE_LAYER.get(id(self.p))
        if dl:  # 无人机层常数项：空载重定位成本（顺延惩罚已由并发项动态接管）
            cost += dl["rebal_cost"]
        return cost, {"delay": total_delay, "travel_min": total_travel}

    def _rider(self, rid):
        return RIDERS[id(self.p)][rid]

    def feasible_insert(self, bi, rid):
        """MVP：插入总是可行（时间软约束走惩罚），无人机运力按假设充足。"""
        return True


# 骑手初始位置：MVP 用固定网格起点（福州市中心），滚动时域再接入真实状态
class _RiderStub:
    def __init__(self, rid, lon, lat):
        self.rider_id, self.lon, self.lat, self.free_min = rid, lon, lat, 0


RIDERS = {}


def make_riders(problem, n=None):
    """生成骑手列表（MVP：均匀撒在订单包围盒内）。"""
    n = n or problem.n_riders
    if not problem.orders:
        RIDERS[id(problem)] = {}
        return []
    xs = [o.pu_lon for o in problem.orders]
    ys = [o.pu_lat for o in problem.orders]
    riders = {}
    for i in range(n):
        lon = min(xs) + (max(xs) - min(xs)) * (i % 5) / 4
        lat = min(ys) + (max(ys) - min(ys)) * (i // 5) / max(1, n // 5)
        riders[f"R{i}"] = _RiderStub(f"R{i}", lon, lat)
    RIDERS[id(problem)] = riders
    return riders


def _virt(pos):
    class _V:  # 让 order_order_km 能算「骑手当前位置→订单」
        pass
    v = _V()
    v.pu_lon, v.pu_lat = pos
    return v


# ---- 破坏算子 ----
def destroy_random(sol, rng, n_remove):
    assigned = [bi for seq in sol.assign.values() for bi in seq]
    if not assigned:
        return []
    out = rng.sample(assigned, min(n_remove, len(assigned)))
    _remove(sol, out)
    return out


def destroy_worst_delay(sol, rng, n_remove):
    """移除延迟成本最高的 bundle（按单 bundle 延迟直接打分，避免逐个全量评估）。"""
    scored = []
    for rid, seq in sol.assign.items():
        for bi in seq:
            b = sol.bundles[bi]
            pad_s = sol.p.pads[b["pad_start"]]
            d = 0.0
            for o in b["orders"]:
                pad_e = sol.p.pads[o.pad_end]
                finish = (sol.tm.leg1(b["orders"][0], pad_s)
                          + sol.tm.leg2(pad_s, pad_e) + sol.tm.leg3(pad_e, o))
                d += max(0.0, finish - o.due_min)
            scored.append((d, bi))
    scored.sort(reverse=True)
    out = [bi for _, bi in scored[:min(n_remove, len(scored))]]
    _remove(sol, out)
    return out


def destroy_geo(sol, rng, n_remove):
    """移除地理关联 bundle（以随机 bundle 的锚点为心，移除其近邻）。"""
    assigned = [(rid, bi) for rid, seq in sol.assign.items() for bi in seq]
    if not assigned:
        return []
    _, seed_bi = rng.choice(assigned)
    seed = sol.bundles[seed_bi]["orders"][0]
    def dist(bi):
        o = sol.bundles[bi]["orders"][0]
        return math.hypot(o.pu_lon - seed.pu_lon, o.pu_lat - seed.pu_lat)
    out = sorted((bi for _, bi in assigned), key=dist)[:n_remove]
    _remove(sol, out)
    return out


def _remove(sol, bis):
    bset = set(bis)
    for rid in list(sol.assign):
        seq = [bi for bi in sol.assign[rid] if bi not in bset]
        if seq:
            sol.assign[rid] = seq
        else:
            del sol.assign[rid]
    sol.unassigned = sorted(set(sol.unassigned) | bset)


# ---- 修复算子 ----
def _insert_cost(sol, bi, rid):
    """把 bundle bi 插到 rid 队列末尾的增量成本（只重算该骑手路线，O(路线长)）。"""
    seq = sol.assign.get(rid, [])
    base_d, base_t = sol.route_eval(rid, seq)
    new_d, new_t = sol.route_eval(rid, seq + [bi])
    base_c = base_d + base_t * TRAVEL_COST_PER_MIN
    new_c = new_d + new_t * TRAVEL_COST_PER_MIN
    return base_c + (new_c - base_c)


def _insert_cost_pos(sol, bi, rid, pos):
    """把 bi 插到 rid 路线第 pos 位（0..len）的增量成本，只重算该骑手。

    含机队并发惩罚差值：插入改变该骑手的在飞时间窗，往高峰插会被
    occupancy_penalty 加罚——这就是「顺延延迟反馈进插入决策」的落地。
    """
    seq = sol.assign.get(rid, [])
    base_d, base_t, base_sp = sol.route_eval(rid, seq, want_spans=True)
    new_d, new_t, new_sp = sol.route_eval(rid, seq[:pos] + [bi] + seq[pos:],
                                          want_spans=True)
    base_c = (base_d + base_t * TRAVEL_COST_PER_MIN
              + sol.occupancy_penalty(base_sp))
    new_c = (new_d + new_t * TRAVEL_COST_PER_MIN
             + sol.occupancy_penalty(new_sp))
    return base_c + (new_c - base_c)


def _best_insert(sol, bi):
    """遍历所有骑手 × 所有插入位，返回 (增量成本, rid, pos)。"""
    best = (float("inf"), None, None)
    for r in _rids(sol):
        seq = sol.assign.get(r, [])
        for pos in range(len(seq) + 1):
            c = _insert_cost_pos(sol, bi, r, pos)
            if c < best[0]:
                best = (c, r, pos)
    return best


def repair_greedy(sol, rng, removed):
    # 全位插入：骑手路线是串行时间链，尾部追加会让搜索退化为无改进空间；
    # 允许任意位插入后 ALNS 才能真正重排（实测改善由此而来）
    for bi in sorted(removed, key=lambda x: sol.bundles[x]["orders"][0].ready_min):
        _, rid, pos = _best_insert(sol, bi)
        if rid is not None:
            sol.assign[rid].insert(pos, bi)
            sol.unassigned = [u for u in sol.unassigned if u != bi]


def repair_regret2(sol, rng, removed):
    """Regret-2：优先插入「次优与最优差值大」的 bundle（不做马上会被坑的单）。
    全位插入：每个 bundle 取每骑手最优位后再算 regret。"""
    pending = sorted(removed, key=lambda x: sol.bundles[x]["orders"][0].ready_min)
    while pending:
        best_bi, best_regret, best_place = None, -1.0, None
        for bi in pending:
            costs = []
            places = {}
            for r in _rids(sol):
                seq = sol.assign.get(r, [])
                c_best = min(
                    (_insert_cost_pos(sol, bi, r, pos), pos)
                    for pos in range(len(seq) + 1))
                costs.append(c_best[0])
                places[r] = c_best
            if not costs:
                continue
            costs.sort()
            regret = (costs[1] - costs[0]) if len(costs) > 1 else float("inf")
            if regret > best_regret:
                best_bi, best_regret = bi, regret
                best_place = min(places.items(), key=lambda kv: kv[1][0])
        if best_bi is None:
            break
        rid, (c, pos) = best_place
        sol.assign[rid].insert(pos, best_bi)
        sol.unassigned = [u for u in sol.unassigned if u != best_bi]
        pending.remove(best_bi)


def _rids(sol):
    return list(RIDERS[id(sol.p)].keys())


def _argmin_insert(sol, bi):
    return min(((_insert_cost(sol, bi, r), r) for r in _rids(sol)))[1]


# ---- 主循环 ----
DESTROY = {"random": destroy_random, "worst": destroy_worst_delay, "geo": destroy_geo}
REPAIR = {"greedy": repair_greedy, "regret2": repair_regret2}


def alns(problem, tm, bundles, iters=200, n_remove=8, seed=42,
         t0=50.0, t1=0.5, verbose=False, fleet=None):
    """ALNS 主循环。返回 (最优解, 成本, 明细)。"""
    rng = random.Random(seed)
    if fleet is not None:
        problem.n_drones = fleet   # occupancy_penalty 读 p.n_drones，必须同步
    make_riders(problem)
    if id(problem) not in DRONE_LAYER:
        set_drone_layer(problem, tm, bundles, fleet=fleet)
    sol = Solution(problem, tm, bundles)
    repair_greedy(sol, rng, sorted(sol.unassigned))
    cur_cost, cur_det = sol.evaluate()
    best, best_cost, best_det = sol.copy(), cur_cost, cur_det

    for it in range(iters):
        T = t0 * (t1 / t0) ** (it / max(1, iters - 1))
        cand = sol.copy()
        op_d = rng.choice(list(DESTROY))
        op_r = rng.choice(list(REPAIR))
        removed = DESTROY[op_d](cand, rng, n_remove)
        REPAIR[op_r](cand, rng, removed)
        c, det = cand.evaluate()
        if c < cur_cost or rng.random() < math.exp(-(c - cur_cost) / max(T, 1e-9)):
            sol, cur_cost, cur_det = cand, c, det
            if c < best_cost:
                best, best_cost, best_det = cand.copy(), c, det
        if verbose and (it + 1) % 50 == 0:
            print(f"  iter {it+1:4d} T={T:5.1f} cur={cur_cost:8.1f} best={best_cost:8.1f}")
    return best, best_cost, best_det
