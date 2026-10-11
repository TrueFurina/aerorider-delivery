# -*- coding: utf-8 -*-
"""Stage 1：订单绑坪（起始/终点停机坪分配）+ 首段订单打包（Order Bundles）。

绑坪：每单的取货点/送达点各自绑定地面距离最近的停机坪（k-d 树不需要，
221 单 × 4313 坪暴力扫一遍毫秒级）。

打包：按「起始坪相同 + 取货点空间邻近 + 时间窗相容」聚簇，簇内订单由同一
骑手一车拉到起始坪（首段共乘），负载上限受骑手载重约束。
实现用「坪内贪心凝聚聚类」：以 time_ready 排序，逐单尝试并入已有簇，
簇的锚点 = 已并入订单取货点均值；距离超 RADIUS_KM 或时间窗不相容或超载则开新簇。
比 k-medoids 更贴合硬约束（负载/共坪），且 O(n) 无需选 k。
"""
from collections import defaultdict

try:                                    # 包内/直接两种导入方式兼容
    from .network import matrix_pad_ids
except ImportError:
    from network import matrix_pad_ids

RADIUS_KM = 1.2        # 同簇订单取货点最大散布（相对锚点）
MAX_LOAD_KG = 8.0      # 首段骑手单车载重上限
TIME_GAP_MIN = 30      # 簇内 ready 时刻最大跨度


def bind_pads(problem, tm):
    """每单绑定起始/终点停机坪（各取最近）。

    候选限定在 A2 路网矩阵覆盖的坪白名单（15 个正式候选坪）：矩阵只对
    这些坪有真实骑行距离，绑到白名单外必然回退直线近似（实测 99.8% 回退
    的假路网模式就是这么来的）。
    """
    allow = matrix_pad_ids()
    pads = [p for p in problem.pads.values() if (not allow or p.pad_id in allow)]
    for o in problem.orders:
        s = min(pads, key=lambda x: tm.pu_pad_km(o, x))
        e = min(pads, key=lambda x: tm.de_pad_km(o, x))
        o.pad_start, o.pad_end = s.pad_id, e.pad_id
    return problem.orders


def bundle_first_leg(problem, tm):
    """首段打包：同起始坪内按 ready 凝聚聚类。返回 bundles 列表。

    bundle = {pad_start, orders:[Order], load_kg, anchor:(lon,lat)}
    空间用 tm.order_order_km（直线），时间窗相容 = ready 差 ≤ TIME_GAP_MIN。
    """
    by_pad = defaultdict(list)
    for o in problem.orders:
        by_pad[o.pad_start].append(o)

    bundles = []
    for pad_id, orders in by_pad.items():
        orders.sort(key=lambda o: o.ready_min)
        open_b = []   # 该坪的开放簇
        for o in orders:
            placed = False
            for b in open_b:
                if (b["load_kg"] + o.weight_kg <= MAX_LOAD_KG
                        and o.ready_min - b["t0"] <= TIME_GAP_MIN
                        and tm.order_order_km(o, _anchor_order(b)) <= RADIUS_KM):
                    b["orders"].append(o)
                    b["load_kg"] += o.weight_kg
                    placed = True
                    break
            if not placed:
                open_b.append({"pad_start": pad_id, "orders": [o],
                               "load_kg": o.weight_kg, "t0": o.ready_min})
        bundles.extend(open_b)
    return bundles


def _anchor_order(b):
    """簇锚点 = 首单（ready 最早），凝聚时以其取货点为空间参照。"""
    return b["orders"][0]


def run_stage1(problem, tm):
    """Stage 1 入口：绑坪 + 打包。返回 (orders, bundles)。"""
    orders = bind_pads(problem, tm)
    bundles = bundle_first_leg(problem, tm)
    return orders, bundles
