# -*- coding: utf-8 -*-
"""问题定义：骑手-无人机三段式协同配送（数据类 + 从 fz_dataset.json 加载）。

三段式：首段骑手(取货点→起始停机坪) → 无人机干线(起始坪→终点坪) → 末端骑手(终点坪→顾客)。
坐标系统一 GCJ-02；距离口径沿用 build_dataset：地面段 = 直线×1.35，空中段 = 直线。
"""
import json
import math
import os
from dataclasses import dataclass, field

# solver/ 的上一级即项目根
BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATASET = os.path.join(BASE, "data", "processed", "fz_dataset.json")

# 载具参数（默认值，建模口径，可调）
RIDER_SPEED_KMH = 25.0     # 电动车城市均速
DRONE_SPEED_KMH = 60.0     # 干线巡航
RIDER_SERVICE_MIN = 3.0    # 取/送每单地面服务耗时
PAD_TRANSFER_MIN = 5.0     # 停机坪装卸/换电耗时
DRONE_MAX_KG = 2.5         # 与数据集适航口径一致


def haversine_km(lon1, lat1, lon2, lat2):
    """球面直线距离（km），GCJ-02 下相对误差可忽略（本项目口径）。"""
    r = 6371.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def road_km(lon1, lat1, lon2, lat2):
    """地面绕行距离 = 直线 × 1.35（与 build_dataset 三段拆分同口径）。"""
    return haversine_km(lon1, lat1, lon2, lat2) * 1.35


@dataclass
class Pad:
    """停机坪（接驳点，relay 池筛过适航的点）。"""
    pad_id: str
    name: str
    lon: float
    lat: float


@dataclass
class Order:
    """一张订单：取货点 + 送达点 + 时间窗 + 绑定坪（由 batching 阶段填写）。"""
    order_id: str
    pu_lon: float
    pu_lat: float
    de_lon: float
    de_lat: float
    demand_type: str
    weight_kg: float
    ready_min: int     # 订单预备完成时刻（分钟制，t=0 为当日 00:00）
    due_min: int       # 承诺交付时刻
    pad_start: str = ""    # 起始停机坪（Stage1 绑定）
    pad_end: str = ""      # 终点停机坪
    pu_id: str = ""        # 取货点真实节点 ID（A2 路网矩阵查表键）
    de_id: str = ""        # 送达点真实节点 ID


@dataclass
class Rider:
    """骑手：位置 + 空闲时刻。滚动时域下位置锁定在最后一个交付点。"""
    rider_id: str
    lon: float
    lat: float
    free_min: int = 0
    assigned: list = field(default_factory=list)   # 已排的 bundle 索引


@dataclass
class Problem:
    orders: list
    pads: dict                       # pad_id -> Pad
    n_riders: int
    n_drones: int
    horizon_min: int = 0             # 时间窗跨度

    # ---- 时间/成本原语（全项目统一口径，别在别处另算）----
    def leg1_min(self, o: Order, pad: Pad):
        """首段骑手：取货点 → 起始坪（地面）。含取货服务时间。"""
        return (road_km(o.pu_lon, o.pu_lat, pad.lon, pad.lat)
                / RIDER_SPEED_KMH * 60 + RIDER_SERVICE_MIN)

    def leg2_min(self, pa: Pad, pb: Pad):
        """无人机干线：起始坪 → 终点坪（空中直线）+ 换电/装卸。"""
        if pa.pad_id == pb.pad_id:
            return 0.0
        return (haversine_km(pa.lon, pa.lat, pb.lon, pb.lat)
                / DRONE_SPEED_KMH * 60 + PAD_TRANSFER_MIN)

    def leg3_min(self, pad: Pad, o: Order):
        """末端骑手：终点坪 → 顾客（地面）。含交付服务时间。"""
        return (road_km(pad.lon, pad.lat, o.de_lon, o.de_lat)
                / RIDER_SPEED_KMH * 60 + RIDER_SERVICE_MIN)


def load_orders(raw, min_weight=None, uav_only=True):
    """fz_dataset['orders'] -> Order 列表。uav_only 只保留可上无人机的单。"""
    out = []
    for r in raw:
        if uav_only and not r.get("uav_suitable"):
            continue
        if min_weight is not None and r.get("weight_kg", 0) > DRONE_MAX_KG:
            continue
        # time_window 是档位标签（标准60min / 加急30-45min / 宽松90-120min），
        # 不是时刻区间：ready 取下单小时整点，due = ready + 档位时长上限
        dur = {"标准60min": 60, "加急30-45min": 45, "宽松90-120min": 120}
        tw = r.get("time_window", "标准60min")
        ready = int(r.get("order_hour", 11)) * 60
        out.append(Order(
            order_id=r["order_id"],
            pu_lon=r["pickup_lon"], pu_lat=r["pickup_lat"],
            de_lon=r["delivery_lon"], de_lat=r["delivery_lat"],
            demand_type=r.get("demand_type", ""),
            weight_kg=r.get("weight_kg", 1.0),
            ready_min=ready, due_min=ready + dur.get(tw, 60),
            pu_id=r.get("pickup_id", ""), de_id=r.get("delivery_id", ""),
        ))
    return out


def load_orders_csv(path, uav_only=True):
    """fz_orders_1550.csv（扩容订单，B1）-> Order 列表。

    与 load_orders 同口径，但来源是 CSV（tw_tier 直接给时刻）。
    订单节点 ID 与 A2 路网矩阵 src 同源——路网模式必须用这份数据，
    用 fz_dataset 300 单（FZ 系列）则 98% 查不到矩阵（ID 集合不同源）。
    """
    import csv as _csv
    out = []
    for r in _csv.DictReader(open(path, encoding="utf-8-sig")):
        if uav_only and r.get("uav_suitable") != "1":
            continue
        w = float(r.get("weight_kg", 1.0))
        if w > DRONE_MAX_KG:
            continue
        out.append(Order(
            order_id=r["order_id"],
            pu_lon=float(r["pickup_lon"]), pu_lat=float(r["pickup_lat"]),
            de_lon=float(r["delivery_lon"]), de_lat=float(r["delivery_lat"]),
            demand_type=r.get("demand_type", ""),
            weight_kg=w,
            ready_min=int(r["ready_min"]), due_min=int(r["due_min"]),
            pu_id=r.get("pickup_id", ""), de_id=r.get("delivery_id", ""),
        ))
    return out


def load_problem(path=DATASET, n_riders=20, n_drones=6, uav_only=True,
                 orders_csv=None):
    """加载问题实例：订单 + 适航停机坪池。

    orders_csv 给定时用扩容订单（1550 单，节点与 A2 矩阵同源），
    否则用 fz_dataset 的 300 单（直线口径兼容模式）。
    """
    d = json.load(open(path, encoding="utf-8"))
    if orders_csv:
        orders = load_orders_csv(orders_csv, uav_only=uav_only)
    else:
        orders = load_orders(d["orders"], uav_only=uav_only)
    # relay/pickup/delivery 是 GeoJSON FeatureCollection：坐标在 geometry.coordinates
    pads = {}
    for f in d["relay"]["features"]:
        pr = f["properties"]
        lon, lat = f["geometry"]["coordinates"][:2]
        pads[pr["node_id"]] = Pad(pr["node_id"], pr["name"], lon, lat)
    horizon = max((o.due_min for o in orders), default=0)
    return Problem(orders=orders, pads=pads, n_riders=n_riders,
                   n_drones=n_drones, horizon_min=horizon)
