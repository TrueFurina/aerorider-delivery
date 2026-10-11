# -*- coding: utf-8 -*-
"""A2：路网距离-时间矩阵（骑手实际骑行，替代直线近似）。

输入：data/geo/fz_road_network.graphml（A1）+ fz_dataset（取/送点）+ 预选主力坪。
方法：
  - shapely STRtree 把每个 POI/坪 O(1) 就近吸附到路网节点；
  - 对每个停机坪跑一次多源 Dijkstra，一次拿到它到全部采样点的最短路；
  - 骑行时间 = 路网 km / 25 km/h × 60（C4 基准速度，路况折扣在 A3 系数表另乘）。
输出（CSV UTF-8-BOM，data/matrices/）：
  fz_matrix_pickup_pad.csv   取样500 × 15坪  (km, min)
  fz_matrix_delivery_pad.csv 送样500 × 15坪  (km, min)
  覆盖 ≥1000×15 规模底线。
"""
import csv
import json
import os
import sys
import time

import networkx as nx
from shapely.geometry import Point
from shapely.strtree import STRtree

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GML = os.path.join(BASE, "data", "geo", "fz_road_network.graphml")
DS = os.path.join(BASE, "data", "processed", "fz_dataset.json")
OUTD = os.path.join(BASE, "data", "matrices")
os.makedirs(OUTD, exist_ok=True)

N_SAMPLE = 500      # 取/送各取样
N_PADS = 15
RIDER_KMH = 25.0    # C4 基准速度
SEED = 2025

# 主力坪预选：仅真实站点类接驳点（build_dataset 已剔地下/室内），按业态优先级
PAD_BIZ = ["停车场", "公园广场", "轨道站点", "体育场馆"]


def main():
    t0 = time.time()
    print("[1/4] 加载路网 ...", flush=True)
    G = nx.read_graphml(GML, node_type=int)
    nodes = list(G.nodes(data=True))
    pts = [Point(d["lon"], d["lat"]) for _, d in nodes]
    tree = STRtree(pts)
    print(f"  节点 {len(nodes)}，用时 {time.time()-t0:.0f}s", flush=True)

    def snap(lon, lat):
        i = tree.nearest(Point(lon, lat))
        return nodes[i][0]

    print("[2/4] 选点 ...", flush=True)
    ds = json.load(open(DS, encoding="utf-8"))

    def feats(role):
        return [f["properties"] | {
            "lon": f["geometry"]["coordinates"][0],
            "lat": f["geometry"]["coordinates"][1]}
            for f in ds[role]["features"]]

    # src 节点全集 = 订单 CSV 的唯一 pickup/delivery 节点（核验要求：
    # 订单节点 ⊆ 矩阵节点，覆盖率必须 100%，随机采样会导致 96% 订单查不到）
    orders_csv = os.path.join(BASE, "data", "orders_expanded", "fz_orders_1550.csv")
    orders = list(csv.DictReader(open(orders_csv, encoding="utf-8-sig")))

    def uniq(rows, pre):
        seen, out = set(), []
        for o in orders:
            k = o[f"{pre}_id"]
            if k not in seen:
                seen.add(k)
                out.append({"node_id": k, "name": o[f"{pre}_name"],
                            "lon": float(o[f"{pre}_lon"]), "lat": float(o[f"{pre}_lat"])})
        return out

    pu = uniq(orders, "pickup")
    de = uniq(orders, "delivery")

    # 主力坪：从 C2 正式候选坪 CSV 取前 15（综合得分排序，空域判定剔除禁飞）
    pads_csv = os.path.join(BASE, "data", "params", "fz_pads_candidates.csv")
    nofly_csv = os.path.join(BASE, "data", "params", "fz_pads_nofly_dist.csv")
    verdict = {}
    if os.path.exists(nofly_csv):
        for r in csv.DictReader(open(nofly_csv, encoding="utf-8-sig")):
            verdict[r["pad_id"]] = r["空域判定"]
    pads = []
    for r in csv.DictReader(open(pads_csv, encoding="utf-8-sig")):
        if verdict.get(r["pad_id"]) == "禁飞":
            continue
        pads.append({"node_id": r["pad_id"], "name": r["名称"],
                     "lon": float(r["lon_gcj02"]), "lat": float(r["lat_gcj02"])})
        if len(pads) >= N_PADS:
            break
    print(f"  取样 {len(pu)} / 送样 {len(de)} / 主力坪 {len(pads)}（C2 正式候选，剔除禁飞）", flush=True)

    pu_ids = [snap(r["lon"], r["lat"]) for r in pu]
    de_ids = [snap(r["lon"], r["lat"]) for r in de]
    pad_ids = [snap(r["lon"], r["lat"]) for r in pads]

    def write_matrix(path, srcs, src_meta, label):
        with open(path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            w.writerow(["src_id", "src_name", "src_lon", "src_lat",
                        "pad_id", "pad_name", "pad_lon", "pad_lat",
                        "road_km", "ride_min"])
            for j, (pid, pr) in enumerate(zip(pad_ids, pads)):
                dist = nx.single_source_dijkstra_path_length(G, pid, weight="km")
                hit = miss = 0
                for k, (sid, sr) in enumerate(zip(srcs, src_meta)):
                    if sid in dist:
                        km = dist[sid]
                        hit += 1
                    else:
                        km = -1.0
                        miss += 1
                    w.writerow([sr["node_id"], sr["name"], sr["lon"], sr["lat"],
                                pr["node_id"], pr["name"], pr["lon"], pr["lat"],
                                f"{km:.3f}" if km >= 0 else "-1",
                                f"{km/RIDER_KMH*60:.1f}" if km >= 0 else "-1"])
                print(f"  [{label}] 坪{j+1}/{len(pads)} {pr['name']}: 可达 {hit} 不可达 {miss}", flush=True)

    print("[3/4] 取单矩阵（每坪一次 Dijkstra）...", flush=True)
    write_matrix(os.path.join(OUTD, "fz_matrix_pickup_pad.csv"), pu_ids, pu, "pickup")
    print("[4/4] 送单矩阵 ...", flush=True)
    write_matrix(os.path.join(OUTD, "fz_matrix_delivery_pad.csv"), de_ids, de, "delivery")
    print(f"[OK] 完成，总用时 {time.time()-t0:.0f}s")


if __name__ == "__main__":
    main()
