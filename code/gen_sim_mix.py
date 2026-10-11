# -*- coding: utf-8 -*-
"""混合场景（Solomon 式合成实例）：同区短距 + 跨区三段，按比例生成。
同区单：pickup/delivery 均在仓东北 TP-018 簇 3km 圆内，直线 3-8km（纯骑手直送分支）
跨区单：pickup 东北 TP-018 簇 3km、delivery 西 TP-007 簇 6km，直线 17-23km（C-D-C 三段分支）
窗：同区 50min / 跨区 70min。ready 660-1020 均匀。路网距离=直线×1.35（SIM模式）。
用法: python gen_sim_mix.py [cross_ratio=0.6] [N=600]
"""
import csv, math, random, sys, os

random.seed(2026)
CROSS_RATIO = float(sys.argv[1]) if len(sys.argv) > 1 else 0.6
N = int(sys.argv[2]) if len(sys.argv) > 2 else 600
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data", "orders_expanded", "fz_orders_sim_mix_600.csv")
CX, CY = 119.30, 26.08
PK = (119.30908, 26.066761)   # TP-018 东北
PD = (119.12453, 26.132628)   # TP-007 西郊

def plane(lon, lat):
    return ((float(lon)-CX)*95.0, (float(lat)-CY)*111.0)

def rand_in_circle(cx, cy, r_deg):
    a = random.uniform(0, 2*math.pi)
    d = random.uniform(0.005, r_deg)
    return (cx + d*math.cos(a), cy + d*math.sin(a))

n_cross = int(round(N * CROSS_RATIO))
lines = []
i = 0
jsame = 0
# --- 跨区单（C-D-C 三段分支）---
while len([x for x in lines if x["region"] == "cross"]) < n_cross:
    plon, plat = rand_in_circle(*PK, 0.03)
    while True:
        dlon, dlat = rand_in_circle(*PD, 0.06)
        if 17.0 <= math.hypot(*[plane(dlon,dlat)[k]-plane(plon,plat)[k] for k in (0,1)]) <= 23.0:
            break
    ready = int(660 + (1020-660)*i/n_cross)   # 独立计数：跨区 ready 均匀铺满 660-1020
    i += 1
    w = 1.5 if i % 3 == 0 else (0.8 if i % 3 == 1 else 1.2)
    lines.append(dict(order_id=f"SIM{i:05d}", pickup_id=90000+i, pickup_name=f"SIM-PK-{i:05d}",
                      pickup_lon=f"{plon:.6f}", pickup_lat=f"{plat:.6f}",
                      delivery_id=91000+i, delivery_name=f"SIM-DL-{i:05d}",
                      delivery_lon=f"{dlon:.6f}", delivery_lat=f"{dlat:.6f}",
                      ready_min=ready, due_min=ready+80, tw_tier="跨区80min", time_slot="跨区",
                      straight_km=f"{math.hypot(plane(dlon,dlat)[0]-plane(plon,plat)[0], plane(dlon,dlat)[1]-plane(plon,plat)[1]):.3f}",
                      weight_kg=w, uav_suitable=1, region="cross"))
# --- 同区单（纯骑手直送分支）---
n_same = N - n_cross
while len([x for x in lines if x["region"] == "same"]) < n_same:
    plon, plat = rand_in_circle(*PK, 0.03)
    while True:
        dlon, dlat = rand_in_circle(*PK, 0.03)
        d = math.hypot(plane(dlon,dlat)[0]-plane(plon,plat)[0], plane(dlon,dlat)[1]-plane(plon,plat)[1])
        if 3.0 <= d <= 8.0:
            break
    ready = int(660 + (1020-660)*jsame/n_same)   # 独立计数：同区 ready 亦均匀铺满 660-1020
    jsame += 1
    i += 1
    w = 1.5 if i % 3 == 0 else (0.8 if i % 3 == 1 else 1.2)
    lines.append(dict(order_id=f"SIM{i:05d}", pickup_id=90000+i, pickup_name=f"SIM-PK-{i:05d}",
                      pickup_lon=f"{plon:.6f}", pickup_lat=f"{plat:.6f}",
                      delivery_id=91000+i, delivery_name=f"SIM-DL-{i:05d}",
                      delivery_lon=f"{dlon:.6f}", delivery_lat=f"{dlat:.6f}",
                      ready_min=ready, due_min=ready+50, tw_tier="同区50min", time_slot="同区",
                      straight_km=f"{d:.3f}", weight_kg=w, uav_suitable=0, region="same"))
lines.sort(key=lambda x: int(x["ready_min"]))  # 按 ready 排序：前 N 单天然混合两种模式
with open(OUT, "w", newline="", encoding="utf-8-sig") as f:
    w = csv.DictWriter(f, fieldnames=list(lines[0].keys()))
    w.writeheader(); w.writerows(lines)
nc = len([x for x in lines if x["region"] == "cross"])
ns = len([x for x in lines if x["region"] == "same"])
print(f"写入 {OUT} 共 {len(lines)} 单（跨区{nc} 同区{ns}）")
for reg in ("same", "cross"):
    sk = [float(x["straight_km"]) for x in lines if x["region"] == reg]
    print(f"  {reg}: 直线 min={min(sk):.1f} 均值={sum(sk)/len(sk):.1f} max={max(sk):.1f} km")
