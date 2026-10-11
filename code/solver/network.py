# -*- coding: utf-8 -*-
"""距离/时间矩阵：停机坪间无人机干线 + 订单到停机坪的地面段。

规模控制：停机坪 4313 个，全矩阵 4313^2 会浪费内存且大部分永不使用——
按需懒计算并缓存（functools.lru_cache 不适用于实例方法，手写 dict 缓存）。
"""
import csv
import math
import os
try:                                    # 包内导入（from solver import network）
    from .problem import haversine_km, road_km
except ImportError:                     # 直接以 solver/ 为 path 导入（脚本/验证用）
    from problem import haversine_km, road_km

# A2 路网矩阵位置（solver/ 的上级即项目根）
BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAT_DIR = os.path.join(BASE, "data", "matrices")
_MAT_FILES = {
    "pickup": "fz_matrix_pickup_pad.csv",
    "delivery": "fz_matrix_delivery_pad.csv",
}


def load_road_matrix():
    """加载 A2 真实路网骑行矩阵。

    返回 {(src_id, pad_id): (road_km, ride_min)}；文件不存在返回空 dict
    （solver 自动回退直线×1.35 口径，行为兼容）。
    """
    out = {}
    for role, fn in _MAT_FILES.items():
        p = os.path.join(MAT_DIR, fn)
        if not os.path.exists(p):
            return {}
        for r in csv.DictReader(open(p, encoding="utf-8-sig")):
            if r["road_km"] == "-1":
                continue            # 不可达：查不到时走回退
            out[(r["src_id"], r["pad_id"])] = (float(r["road_km"]),
                                               float(r["ride_min"]))
    return out


def matrix_pad_ids():
    """A2 矩阵覆盖的停机坪集合（求解器实验坪白名单 = 15 个正式候选坪）。

    绑坪必须在该白名单内选：矩阵只对这些坪有路网距离，绑到其它坪必然回退直线。
    """
    ids = set()
    for fn in _MAT_FILES.values():
        p = os.path.join(MAT_DIR, fn)
        if not os.path.exists(p):
            return ids
        for r in csv.DictReader(open(p, encoding="utf-8-sig")):
            ids.add(r["pad_id"])
        break                       # 两文件坪集合相同，读一个即可
    return ids


class TimeMatrix:
    """懒计算时间/距离矩阵，统一走 Problem 的时间原语。

    路网模式（load_road_matrix 非空）：leg1/leg3 优先用 A2 真实骑行距离
    （25km/h 基准），查不到（订单节点不在矩阵/该坪不可达）回退直线×1.35。
    pu_pad_km/de_pad_km 聚类几何距离保持直线口径（绑坪用，语义不变）。
    """

    def __init__(self, problem, use_road=True):
        self.p = problem
        self._leg2 = {}     # (pad_a, pad_b) -> 分钟
        self._leg1 = {}     # (order_id, pad_id) -> 分钟
        self._leg3 = {}     # (pad_id, order_id) -> 分钟
        self._pad_air = {}  # (pad_a, pad_b) -> 空中直线 km（评估用）
        self.road = load_road_matrix() if use_road else {}
        self.road_hits = 0   # 路网命中/回退计数（实验报表用）
        self.road_fallback = 0

    def _road_time(self, src_id, pad_id, fallback_min):
        """查路网矩阵；命中返回骑行分钟，否则回退并计数。"""
        v = self.road.get((src_id, pad_id))
        if v is not None:
            self.road_hits += 1
            return v[1]
        self.road_fallback += 1
        return fallback_min

    def leg1(self, o, pad):
        k = (o.order_id, pad.pad_id)
        if k not in self._leg1:
            fallback = self.p.leg1_min(o, pad)
            # 路网模式：取货点→起始坪用真实骑行时间（A2 矩阵按 pickup_id 查），
            # 查不到回退直线
            self._leg1[k] = (self._road_time(o.pu_id, pad.pad_id, fallback)
                             if self.road else fallback)
        return self._leg1[k]

    def leg2(self, pa, pb):
        k = (pa.pad_id, pb.pad_id)
        if k not in self._leg2:
            self._leg2[k] = self.p.leg2_min(pa, pb)
        return self._leg2[k]

    def leg3(self, pad, o):
        k = (pad.pad_id, o.order_id)
        if k not in self._leg3:
            fallback = self.p.leg3_min(pad, o)
            # 路网模式：终点坪→顾客用真实骑行时间（A2 矩阵按 delivery_id 查），
            # 查不到回退直线
            self._leg3[k] = (self._road_time(o.de_id, pad.pad_id, fallback)
                             if self.road else fallback)
        return self._leg3[k]

    # ---- 供聚类/绑坪用的纯几何距离（不带时间语义，避免语义混用）----
    def pu_pad_km(self, o, pad):
        return road_km(o.pu_lon, o.pu_lat, pad.lon, pad.lat)

    def de_pad_km(self, o, pad):
        return road_km(o.de_lon, o.de_lat, pad.lon, pad.lat)

    def order_order_km(self, a, b, end="pu"):
        """订单间距离（默认取货点对取货点），聚类用。"""
        f = (lambda o: (o.pu_lon, o.pu_lat)) if end == "pu" else (lambda o: (o.de_lon, o.de_lat))
        xa, ya = f(a); xb, yb = f(b)
        return haversine_km(xa, ya, xb, yb)
