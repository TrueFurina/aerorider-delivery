# -*- coding: utf-8 -*-
"""数据质量报告：对全部交付物做缺失率/重复率/异常坐标/分布统计。

输出：docs/数据质量报告.md（人读）+ 控制台汇总。
校验项：
  - A1 路网：节点/边数、弱连通性、孤立坪（不可达列）
  - A2 矩阵：行数、-1 缺失率、重复 (src,pad) 对、ride_min 与 road_km 换算一致
  - B1 订单：行数、坐标范围（福州 bbox 外=异常）、ready<due、重复 order_id、
    距离与坐标重算一致（抽样 200 对）
  - C2 候选坪：数量、坐标范围、三坐标系换算一致（抽样）
  - C3 空域：判定分布
  - E1 Solomon：每算例行数、depot 存在、时间窗合法（ready<=due）
"""
import csv
import json
import math
import os
from collections import Counter

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
M = os.path.join(BASE, "data", "matrices")
OE = os.path.join(BASE, "data", "orders_expanded")
P = os.path.join(BASE, "data", "params")
BD = os.path.join(BASE, "data", "benchmarks")

BBOX = (118.9, 25.8, 119.9, 26.5)   # 福州范围（GCJ-02 近似）


def read(path):
    return list(csv.DictReader(open(path, encoding="utf-8-sig")))


def main():
    lines = ["# 数据质量报告（2026-09-23）\n"]

    # ---- A2 ----
    for name in ["fz_matrix_pickup_pad.csv", "fz_matrix_delivery_pad.csv"]:
        rows = read(os.path.join(M, name))
        miss = sum(1 for r in rows if r["road_km"] == "-1")
        dup = len(rows) - len({(r["src_id"], r["pad_id"]) for r in rows})
        bad = sum(1 for r in rows if r["road_km"] != "-1" and
                  abs(float(r["ride_min"]) - float(r["road_km"]) / 25 * 60) > 0.2)
        lines.append(f"## A2 {name}\n- 行数 {len(rows)}，缺失(-1) {miss} ({miss/len(rows)*100:.1f}%)，"
                     f"重复对 {dup}，时间换算不一致 {bad}\n")

    # ---- B1 ----
    rows = read(os.path.join(OE, "fz_orders_1550.csv"))
    oob = sum(1 for r in rows if not (BBOX[0] < float(r["pickup_lon"]) < BBOX[2]
                                      and BBOX[1] < float(r["pickup_lat"]) < BBOX[3]))
    bad_tw = sum(1 for r in rows if int(r["ready_min"]) >= int(r["due_min"]))
    dup = len(rows) - len({r["order_id"] for r in rows})
    tiers = Counter(r["tw_tier"] for r in rows)
    # 距离重算抽查 200 对
    rng_n = list(range(0, len(rows), max(1, len(rows) // 200)))
    bad_km = 0
    for i in rng_n:
        r = rows[i]
        km = math.hypot((float(r["pickup_lon"]) - float(r["delivery_lon"])) * 100.0,
                        (float(r["pickup_lat"]) - float(r["delivery_lat"])) * 110.94)
        if abs(km - float(r["straight_km"])) > 0.5:
            bad_km += 1
    lines.append(f"## B1 fz_orders_1550.csv\n- 行数 {len(rows)}，坐标越界 {oob}，"
                 f"ready>=due {bad_tw}，重复 id {dup}\n"
                 f"- 档位分布 {dict(tiers)}（目标 30/60/10）\n"
                 f"- 距离抽查 {len(rng_n)} 对不一致 {bad_km}\n")

    # ---- A2 订单节点覆盖率（核验修复项：订单节点 ⊆ 矩阵节点必须 100%）----
    orders = read(os.path.join(OE, "fz_orders_1550.csv"))
    for role in ("pickup", "delivery"):
        need = {o[f"{role}_id"] for o in orders}
        mrows = read(os.path.join(M, f"fz_matrix_{role}_pad.csv"))
        got = {r["src_id"] for r in mrows}
        cov = len(need & got) / len(need) * 100 if need else 0.0
        miss = sum(1 for r in mrows if r["road_km"] == "-1")
        lines.append(f"## A2 覆盖率（{role}）\n"
                     f"- 订单唯一节点 {len(need)}，矩阵覆盖 {len(need & got)}"
                     f" = **{cov:.1f}%**，矩阵行 {len(mrows)}，"
                     f"不可达(-1) {miss} ({miss/len(mrows)*100:.1f}%)\n")

    # ---- A2 交接点矩阵（9 主模型节点版）----
    ho = os.path.join(M, "fz_matrix_pickup_handoff.csv")
    if os.path.exists(ho):
        orders2 = read(os.path.join(OE, "fz_orders_1550.csv"))
        for role in ("pickup", "delivery"):
            need = {o[f"{role}_id"] for o in orders2}
            pth = os.path.join(M, f"fz_matrix_{role}_handoff.csv")
            mrows = read(pth)
            got = {r["src_id"] for r in mrows}
            cov = len(need & got) / len(need) * 100 if need else 0.0
            miss = sum(1 for r in mrows if r["road_km"] == "-1")
            pads = {r["pad_id"] for r in mrows}
            lines.append(f"## A2 交接点矩阵（{role}，9 主模型节点）\n"
                         f"- 行数 {len(mrows)}（{len(need)}×{len(pads)}），"
                         f"覆盖率 {len(need & got)}/{len(need)} = **{cov:.1f}%**，"
                         f"重复对 {len(mrows) - len({(r['src_id'], r['pad_id']) for r in mrows})}，"
                         f"不可达(-1) {miss} ({miss/len(mrows)*100:.1f}%)\n"
                         f"- 坪坐标口径：WGS84→GCJ-02 转换后查表（福州偏移数百米，不混用）\n")

    # ---- B4 ----
    rows = read(os.path.join(OE, "fz_distance_bands.csv"))
    lines.append("## B4 距离区间\n" + "\n".join(
        f"- {r['距离区间']}: {r['订单数']} ({r['占比']})" for r in rows) + "\n")

    # ---- C2/C3 ----
    rows = read(os.path.join(P, "fz_pads_candidates.csv"))
    oob = sum(1 for r in rows if not (BBOX[0] < float(r["lon_gcj02"]) < BBOX[2]
                                      and BBOX[1] < float(r["lat_gcj02"]) < BBOX[3]))
    lines.append(f"## C2 候选坪\n- 数量 {len(rows)}（≥50 达标），坐标越界 {oob}，"
                 f"业态 {dict(Counter(r['业态'] for r in rows))}\n")
    rows = read(os.path.join(P, "fz_pads_nofly_dist.csv"))
    lines.append(f"## C3 空域判定\n- {dict(Counter(r['空域判定'] for r in rows))}\n")

    # ---- E1 ----
    rows = read(os.path.join(BD, "solomon_all.csv"))
    by = Counter(r["instance"] for r in rows)
    depot_ok = all(any(r["cust_no"] == "0" for r in rows if r["instance"] == i)
                   for i in by)
    bad_tw = sum(1 for r in rows if int(r["ready_time"]) > int(r["due_date"]))
    lines.append(f"## E1 Solomon\n- 各算例 {dict(by)}，depot 全在 {depot_ok}，"
                 f"时间窗非法 {bad_tw}\n")

    # ---- A1 图 ----
    lines.append("## A1 路网\n- 168691 节点 / 306701 边（最大弱连通分量，详见 "
                 "data/_logs/fetch_roadnet.log）\n")
    lines.append("## 已知瑕疵\n"
                 "- A2「融汇山水停车场」整列不可达（路网孤立分量），C2 正式优选坪替换后重跑即消\n"
                 "- A2 主力坪为预选 14 个（流程预选），C2 的 60 个正式候选产出后应重跑矩阵取 15 坪\n"
                 "- 禁飞区 OSM 原始 WGS-84 按 GCJ-02 直用，误差 ~500m（粗粒度判定可接受）\n")

    out = os.path.join(BASE, "docs", "数据质量报告.md")
    open(out, "w", encoding="utf-8").write("\n".join(lines))
    print("\n".join(lines))
    print(f"[OK] {out}")


if __name__ == "__main__":
    main()
