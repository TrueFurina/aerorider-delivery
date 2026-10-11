# -*- coding: utf-8 -*-
"""C-D-C 完整版求解器 v2：时间计算逐字复用 solve_cdc_rolling.py（数字口径一致），
仅增加对齐 0917 result(5).xlsx 的明细输出。
产出(前缀 {TAG}_full_{N}单):
  {TAG}_full_{N}单.csv     汇总指标(18列)
  {TAG}_routes_{N}单.csv   骑手路线明细(骑手编号,单数,延迟,完工时间,里程,路线)
  {TAG}_uav_{N}单.csv      无人机架次(架次,载件数,停靠点数,停靠序列,起飞,返仓,航程)
  {TAG}_handoff_{N}单.csv  交接明细(骑手编号,交接单数,明细)
用法: python solve_full.py [N] [pure]
"""
import numpy as np, csv, math, sys, os, time
from collections import defaultdict

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NP = os.path.join(ROOT, "data", "OD道路距离_米.npy")
IDX = os.path.join(ROOT, "data", "节点索引.csv")
ORD = sys.argv[5] if len(sys.argv) > 5 and os.path.isabs(sys.argv[5]) else os.path.join(ROOT, sys.argv[5] if len(sys.argv) > 5 else os.path.join("data", "orders_expanded", "fz_orders_1550.csv"))
OUTD = os.path.join(ROOT, "results")
DETOUR = 1.35

RIDER_KMH = 25.0
UAV_KMH = 57.6
RIDER_FIXED = 50.0
RIDER_VAR = 0.2
UAV_FIXED = 10.0
UAV_VAR = 0.3
SERVICE_MIN = 3.0
MAX_ORDERS_PER_RIDER = 10
UAV_FLEET = 20          # 无人机机队规模（对齐 0917：20 架共享）
UAV_MAX_LOAD = 5.0      # 无人机最大载重 kg（合并多单上限）
MERGE_WIN = 10.0        # 同航程(po,pd)订单合并时间窗 min

RIDER_KPM = RIDER_KMH/60.0
UAV_KPM = UAV_KMH/60.0

PADS = [("FZ-TP-001",21144),("FZ-TP-003",21146),("FZ-TP-004",21147),("FZ-TP-005",21148),
        ("FZ-TP-007",21150),("FZ-TP-011",21154),("FZ-TP-012",21155),("FZ-TP-018",21161),("FZ-TP-019",21162)]
PAD_IDS = [p[1] for p in PADS]

print("加载 npy...")
A = np.load(NP, mmap_mode="r")
idx = list(csv.DictReader(open(IDX, encoding="utf-8-sig")))
pname2row = {r["NodeName"]: int(r["MatrixID"]) for r in idx if r["Category"]=="pickup"}
dsid2row = {r["SourceID"]: int(r["MatrixID"]) for r in idx if r["Category"]=="delivery"}

orders = list(csv.DictReader(open(ORD, encoding="utf-8-sig")))
sel = [o for o in orders if o["pickup_name"] in pname2row and o["delivery_id"] in dsid2row]
N = int(sys.argv[1]) if len(sys.argv) > 1 else 120
PURE = (len(sys.argv) > 2 and sys.argv[2] == "pure")
TW_SCALE = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0   # 时间窗缩放系数
PENALTY = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0    # 延误惩罚 元/min
SIM = (len(sys.argv) > 6 and sys.argv[6] == "sim")             # 仿真模式：合成ID+直线×1.35距离
if SIM:
    sel = orders[:N]
else:
    sel = [o for o in orders if o["pickup_name"] in pname2row and o["delivery_id"] in dsid2row]
    sel = sel[:N]

PAD_CSV = ROOT + r"\data\params\fz_handoff_nodes_master.csv"
pad_plane = {}
for r in csv.DictReader(open(PAD_CSV, encoding="utf-8-sig")):
    pad_plane[r["node_id"]] = (float(r["plane_x_km"]), float(r["plane_y_km"]))

def plane(lon, lat):
    return ((float(lon)-119.30)*95.0, (float(lat)-26.08)*111.0)

row2xy = {}
if not SIM:
    for o in sel:
        row2xy[pname2row[o["pickup_name"]]] = plane(o["pickup_lon"], o["pickup_lat"])
        row2xy[dsid2row[o["delivery_id"]]] = plane(o["delivery_lon"], o["delivery_lat"])
for pid, rid in PADS:
    if pid in pad_plane:
        row2xy[rid] = pad_plane[pid]
if SIM:
    for o in sel:
        row2xy[int(o["pickup_id"])] = plane(o["pickup_lon"], o["pickup_lat"])
        row2xy[int(o["delivery_id"])] = plane(o["delivery_lon"], o["delivery_lat"])

def line_km(ri, rj):
    xi,yi = row2xy.get(ri,(0,0)); xj,yj = row2xy.get(rj,(0,0))
    return math.hypot(xi-xj, yi-yj)

def road_km(ri, rj):
    if ri < A.shape[0] and rj < A.shape[1]:
        v = float(A[ri, rj])
        if math.isfinite(v) and v > 0:
            return v/1000.0
    return line_km(ri, rj)*DETOUR

def t_min(km): return km/RIDER_KPM

processed = []
for o in sel:
    pr = (int(o["pickup_id"]) if SIM else pname2row[o["pickup_name"]])
    dr = (int(o["delivery_id"]) if SIM else dsid2row[o["delivery_id"]])
    ready = int(float(o["ready_min"]))
    due = int(ready + (float(o["due_min"])-float(o["ready_min"]))*TW_SCALE)
    win = due - ready
    # C-D-C三段式：枚举po/pd坪组合，找最快方案
    best_po, best_pd = PAD_IDS[0], PAD_IDS[0]
    best_tB = 999999
    for po in PAD_IDS:
        for pd in PAD_IDS:
            if po == pd: continue
            rp = road_km(pr, po); rd = road_km(pd, dr); pp = line_km(po, pd)
            if rp < 0 or rd < 0: continue
            tB_raw = (rp+rd)/RIDER_KPM + pp/UAV_KPM + 2*SERVICE_MIN + 3.0
            if tB_raw < best_tB:
                best_tB = tB_raw; best_po = po; best_pd = pd
    # 直送
    direct_km = road_km(pr, dr)
    t_A = direct_km / RIDER_KPM + 2*SERVICE_MIN
    if PURE:
        is_local = True
    else:
        # 逐单决策：比较直送时间t_A与三段式时间t_B，谁快选谁
        is_local = (t_A <= best_tB)
    processed.append(dict(
        oid=o["order_id"], pr=pr, dr=dr,
        pname=o["pickup_name"], dname=o["delivery_name"],
        po=best_po, pd=best_pd,
        weight=float(o["weight_kg"]) if o["weight_kg"] else 1.0,
        same=is_local,
        ready=ready, due=due,
    ))

PERIODS = 4
WINDOW_MIN = 15  # 滚动时域窗口（min）
proc_sorted = sorted(processed, key=lambda x: x["ready"])
batches = []
cur = []
for x in proc_sorted:
    if not cur or x["ready"] <= cur[0]["ready"] + WINDOW_MIN:
        cur.append(x)
    else:
        batches.append(cur); cur = [x]
if cur: batches.append(cur)

def _tw_feasible(path, items, start_node, end_node, t0, mode):
    """串行时间链检查：返回(是否全部按时, 末时间)"""
    t = t0
    cur = start_node
    for it in path:
        if mode == "local":
            t += t_min(road_km(cur, it["pr"])) + SERVICE_MIN
            t += t_min(road_km(it["pr"], it["dr"])) + SERVICE_MIN
            cur = it["dr"]
        elif mode == "first":
            t += t_min(road_km(cur, it["pr"])) + SERVICE_MIN
            cur = it["pr"]
        else:  # last
            t += t_min(road_km(cur, it["dr"])) + SERVICE_MIN
            cur = it["dr"]
        if t > it["due"]: return False, t
    if mode == "first" and end_node is not None:
        t += t_min(road_km(cur, end_node))
    return True, t

def cw_group(items, start_row, end_row, t0, capacity=MAX_ORDERS_PER_RIDER, row_attr="pr", mode="local"):
    n = len(items)
    if n == 0: return []
    rows = [it[row_attr] for it in items]
    sav=[]
    for i in range(n):
        for j in range(i+1,n):
            s = road_km(rows[i], end_row) + road_km(rows[j], end_row) - road_km(rows[i], rows[j])
            sav.append((s,i,j))
    sav.sort(reverse=True)
    paths=[[i] for i in range(n)]
    for s,i,j in sav:
        if s<=0: break
        pi=[k for k,p in enumerate(paths) if i in p]
        pj=[k for k,p in enumerate(paths) if j in p]
        if pi==pj: continue
        ri,rj=pi[0],pj[0]
        if len(paths[ri])+len(paths[rj]) > capacity: continue
        cand=None
        if paths[ri][-1]==i and paths[rj][0]==j:
            cand=paths[ri]+paths[rj]
        elif paths[ri][0]==i and paths[rj][-1]==j:
            cand=paths[rj]+paths[ri]
        if cand is not None:
            feas,_ = _tw_feasible([items[k] for k in cand], items, start_row, end_row, t0, mode)
            if feas:
                paths[ri]=cand; del paths[rj]
    out=[]
    for p in paths:
        km=0.0
        for k in range(len(p)-1): km+=road_km(rows[p[k]],rows[p[k+1]])
        km+=road_km(rows[p[-1]],end_row)
        out.append(([items[i] for i in p], km, len(p)))
    return out

# === 滚动决策（时间计算与 solve_cdc_rolling.py 逐字一致）===
all_routes = []   # (周期, 骑手编号, 类型标签, 坪, item列表, km)
delivered_min = {} # oid -> (送达时间, 是否准时)
oid2info = {x["oid"]: x for x in processed}
first_arrive = {}
uav_seq = []      # (oid, 起飞时间, 飞行min, 航程km, po, pd)
period_cpu = []   # (周期, 耗时s)
rider_pool = {}   # rid -> {node, t_end, used}
rider_idx = 0

def pick_rider(priority_node, period_start, item_list, prefer_node=None):
    """复用比'仓出发'更近的空闲骑手；否则新骑手从仓出发。返回 (rid, st, start_node)"""
    global rider_idx
    t_open = max(period_start, min(i["ready"] for i in item_list))
    best = None
    best_d = road_km(0, priority_node)  # 基准：新骑手从仓出发
    for rid, st in rider_pool.items():
        if st["used"]: continue
        if st["t_end"] > t_open: continue  # 上周期未完工，不可复用
        d = 0.0 if (prefer_node is not None and st["node"] == prefer_node) else road_km(st["node"], priority_node)
        if d < best_d: best_d = d; best = (rid, st)
    if best is not None:
        rid, st = best
        start_node = st["node"]
        st["t_end"] = t_open
    else:
        rider_idx += 1
        rid = f"RD-{rider_idx:03d}"
        st = {"node": 0, "t_end": t_open, "used": False}
        rider_pool[rid] = st
        start_node = 0
    return rid, st, start_node

for p_idx, batch in enumerate(batches):
    if not batch: continue
    t_period_start = time.time()
    period_start = batch[0]["ready"]
    for st in rider_pool.values(): st["used"] = False  # 每周期重置骑手可用性
    same_orders = [x for x in batch if x["same"]]
    cross = [x for x in batch if not x["same"]]
    first_by_po = defaultdict(list)
    last_by_pd = defaultdict(list)
    for x in cross:
        first_by_po[x["po"]].append(x)
        last_by_pd[x["pd"]].append(x)

    # 同区: 骑手位置(复用)->pickup->delivery
    for item_list, km_old, cnt in cw_group(same_orders, 0, 0, period_start, mode="local"):
        rid, st, start_node = pick_rider(item_list[0]["pr"], period_start, item_list)
        seq = [start_node]
        for it in item_list: seq += [it["pr"], it["dr"]]
        km = sum(road_km(seq[i], seq[i+1]) for i in range(len(seq)-1))
        t = st["t_end"]
        parts = []
        sp = 0
        for k, it in enumerate(item_list):
            t = max(t, it["ready"])  # 等待订单就绪
            t += t_min(road_km(seq[sp], seq[sp+1])) + SERVICE_MIN
            parts.append(f"取{it['pname']}({t:.1f})")
            sp += 1
            t += t_min(road_km(seq[sp], seq[sp+1])) + SERVICE_MIN
            ok = t <= it["due"]
            delivered_min[it["oid"]] = (t, ok)
            parts.append(f"送{it['dname']}({t:.1f})")
            sp += 1
        st["node"] = item_list[-1]["dr"]; st["t_end"] = t; st["used"] = True
        all_routes.append((p_idx+1, rid, "纯骑手", "-", item_list, km, " → ".join(parts)))

    # 跨区首段: 骑手位置(复用)->pickup(打包同PO)->坪
    for po, items in first_by_po.items():
        po_name = PADS[PAD_IDS.index(po)][0]
        for item_list, km_old, cnt in cw_group(items, 0, po, period_start, capacity=3, mode="first"):
            rid, st, start_node = pick_rider(item_list[0]["pr"], period_start, item_list)
            seq = [start_node]
            for it in item_list: seq.append(it["pr"])
            seq.append(po)
            km = sum(road_km(seq[i], seq[i+1]) for i in range(len(seq)-1))
            t = st["t_end"]
            parts = []
            sp = 0
            for k, it in enumerate(item_list):
                t = max(t, it["ready"])  # 等待订单就绪
                t += t_min(road_km(seq[sp], seq[sp+1])) + SERVICE_MIN
                parts.append(f"取{it['pname']}({t:.1f})")
                sp += 1
            t += t_min(road_km(seq[sp], seq[sp+1]))  # 送坪
            parts.append(f"交{po_name}({t:.1f})")
            for oid in [i["oid"] for i in item_list]: first_arrive[oid] = t  # 同组统一交坪
            st["node"] = po; st["t_end"] = t; st["used"] = True
            all_routes.append((p_idx+1, rid, f"首段->{po_name}", po_name, item_list, km, " → ".join(parts)))

    # 周期内无人机调度：同航程(po,pd)合并 + 机队20排队（体现「交接点整合订单、某一无人机送」）
    uav_records = {}   # oid -> (takeoff, arrive)
    if cross:
        merge = defaultdict(list)
        for x in cross:
            merge[(x["po"], x["pd"])].append((x["oid"], first_arrive.get(x["oid"], period_start), x["weight"]))
        sched = []
        for (po, pd), lst in merge.items():
            lst.sort(key=lambda v: v[1])
            groups = []
            for oid, t, w in lst:
                if groups and t - groups[-1][1] <= MERGE_WIN and groups[-1][2] + w <= UAV_MAX_LOAD:
                    g = groups[-1]; g[0].append(oid); g[1] = t; g[2] += w
                else:
                    groups.append([[oid], t, w])
            po_name = PADS[PAD_IDS.index(po)][0]; pd_name = PADS[PAD_IDS.index(pd)][0]
            x1, y1 = pad_plane[po_name]; x2, y2 = pad_plane[pd_name]
            fl = math.hypot(x1-x2, y1-y2) / UAV_KPM
            dist = math.hypot(x1-x2, y1-y2)
            for oids, tmax, wsum in groups:
                sched.append((oids, tmax, fl, dist, po_name, pd_name, len(oids)))
        sched.sort(key=lambda s: s[1])
        fleet = [0.0] * UAV_FLEET
        for oids, t_req, fl, dist, po_name, pd_name, cnt in sched:
            idx = fleet.index(min(fleet))
            takeoff = max(t_req, fleet[idx])
            fleet[idx] = takeoff + fl
            for oid in oids:
                uav_records[oid] = (takeoff, takeoff + fl)
            uav_seq.append((tuple(oids), takeoff, fl, dist, po_name, pd_name))

    # 跨区末端: 坪->delivery（打包同PD，复用停在坪的骑手）
    for pd, items in last_by_pd.items():
        pd_name = PADS[PAD_IDS.index(pd)][0]
        for item_list, km_old, cnt in cw_group(items, pd, pd, period_start, capacity=3, row_attr="dr", mode="last"):
            uav_arr = {it["oid"]: uav_records.get(it["oid"], (period_start, period_start))[1] for it in item_list}
            rid, st, start_node = pick_rider(pd, period_start, item_list, prefer_node=pd)
            # 末端骑手在pd坪待命（片区骑手），直接开始送dr
            seq = [pd] + [it["dr"] for it in item_list]
            t = max(st["t_end"], max(uav_arr.values()))
            parts = []
            sp = 0
            for k, it in enumerate(item_list):
                t = max(t, uav_arr[it["oid"]], it["ready"])
                t += t_min(road_km(seq[sp], seq[sp+1])) + SERVICE_MIN
                ok = t <= it["due"]
                delivered_min[it["oid"]] = (t, ok)
                parts.append(f"送{it['dname']}({t:.1f})")
                sp += 1
            st["node"] = item_list[-1]["dr"]; st["t_end"] = t; st["used"] = True
            all_routes.append((p_idx+1, rid, f"末端<-{pd_name}", pd_name, item_list, km_old, " → ".join(parts)))
    period_cpu.append((p_idx+1, time.time()-t_period_start))

# === 汇总（与 solve_cdc_rolling.py 逐字一致）===
TAG = "纯骑手" if PURE else "CD-C"
total = len(processed)
ontime = sum(1 for t,ok in delivered_min.values() if ok)
late = sum(1 for t,ok in delivered_min.values() if not ok)
# 口径对齐 0917：平均履约 = 送达-下单(ready)；完工max = 车次最后送达-该单ready
avg_commit = sum(t - oid2info[oid]["ready"] for oid, (t, ok) in delivered_min.items())/len(delivered_min)
max_finish = max(max(delivered_min[it["oid"]][0] - it["ready"] for it in r[4]) for r in all_routes if r[4])
rider_km = sum(r[5] for r in all_routes)
rider_trips = len(all_routes)
rider_ids = len(set(r[1] for r in all_routes))       # 最优骑手数（去重）
cross_total = sum(1 for x in processed if not x["same"])
uav_km_total = sum(u[3] for u in uav_seq)
uav_trips = len(uav_seq)
uav_used = min(UAV_FLEET, max(uav_trips, 0))         # 最优无人机数（实际使用≤机队）
rider_cost = rider_trips*RIDER_FIXED + rider_km*RIDER_VAR
uav_cost = uav_trips*UAV_FIXED + uav_km_total*UAV_VAR
total_cost = rider_cost + uav_cost
late_penalty = sum((t - oid2info[oid]["due"])*PENALTY for oid, (t, ok) in delivered_min.items() if not ok)
total_cost_w_penalty = total_cost + late_penalty
total_cpu = sum(c for _, c in period_cpu)
period_cpu_str = " ".join(f"{p}:{c:.2f}" for p, c in period_cpu)

print(f"\n=== 完整版 {TAG} ({N}单, 窗宽x{TW_SCALE}, 罚{PENALTY}元/min) ===")
print(f"服务率: {total}/{total}=100% | 时间窗满足{ontime/total*100:.1f}%, 延迟{late}单")
print(f"平均履约(送达-下单): {avg_commit:.1f}min | 完工max: {max_finish:.1f}min")
print(f"骑手: {rider_trips}次出车({rider_ids}骑手), {rider_km:.1f}km | 无人机: {uav_trips}架次, {uav_km_total:.1f}km")
print(f"成本: {total_cost:.1f}元 | 延误惩罚: {late_penalty:.1f}元 | 含罚总成本: {total_cost_w_penalty:.1f}元 | 经坪捎带: {cross_total}单")

# === 输出 CSV（full，21列：含时间窗满足率、延迟罚、含罚总成本）===
with open(OUTD + rf"\{TAG}_v4_full_{N}单.csv","w",newline="",encoding="utf-8-sig") as f:
    w = csv.writer(f)
    w.writerow(["场景","订单量(单)","调度使用骑手数","调度使用无人机数","已服务订单(单)","未服务订单(单)",
                "订单服务率(%)","时间窗满足率(%)","延迟订单数(单)",
                "平均履约时间(送达-下单)(min)","骑手完工时间最大值(min)",
                "运营成本(元)","延迟惩罚(元)","含罚总成本(元)",
                "CPU(s)","每周期CPU(s)","骑手出发次数","骑手里程(km)","无人机里程(km)","无人机架次",
                "经交接点由无人机捎带(单)"])
    w.writerow([f"仿真混合·{'纯骑手' if PURE else '无人机协同'}",N,rider_ids,uav_used,total,0,100,
                f"{ontime/total*100:.1f}",late,
                f"{avg_commit:.2f}",f"{max_finish:.2f}",
                f"{total_cost:.2f}",f"{late_penalty:.2f}",f"{total_cost_w_penalty:.2f}",
                f"{total_cpu:.2f}",period_cpu_str,
                rider_trips,f"{rider_km:.2f}",f"{uav_km_total:.2f}",uav_trips,cross_total])

# 路线明细：骑手编号、已服务、延迟、完工时间（送达-下单）、里程、服务路线
with open(OUTD + rf"\{TAG}_v4_routes_{N}单.csv","w",newline="",encoding="utf-8-sig") as f:
    w = csv.writer(f)
    w.writerow(["骑手编号","已服务订单(单)","延迟订单数(单)","完工时间(min)","骑手里程(km)","服务路线"])
    for r in all_routes:
        items = r[4]
        lt = sum(1 for it in items if not delivered_min[it["oid"]][1])
        finish = max(delivered_min[it["oid"]][0] - it["ready"] for it in items) if items else 0
        w.writerow([r[1], len(items), lt, f"{finish:.2f}", f"{r[5]:.2f}", r[6]])

# 无人机架次（含同航程合并载件数）
with open(OUTD + rf"\{TAG}_v4_uav_{N}单.csv","w",newline="",encoding="utf-8-sig") as f:
    w = csv.writer(f)
    w.writerow(["架次","载件数(单)","停靠点数","停靠序列","起飞(min)","返仓(min)","航程(km)"])
    period_of_oid = {}
    for p_idx, batch in enumerate(batches):
        for x in batch: period_of_oid[x["oid"]] = p_idx+1
    uav_global = 0
    for u in uav_seq:
        uav_global += 1
        oids, t0, fl, dist, po, pd = u
        pid = period_of_oid.get(oids[0], 1)
        w.writerow([f"UAV-{uav_global:02d}(第{pid}周期)", len(oids), 1,
                    f"{po} → {pd}", f"{t0:.1f}", f"{t0+fl:.1f}", f"{dist:.2f}"])

# 交接明细：末端骑手聚合
with open(OUTD + rf"\{TAG}_v4_handoff_{N}单.csv","w",newline="",encoding="utf-8-sig") as f:
    w = csv.writer(f)
    w.writerow(["骑手编号","交接订单数(单)","交接明细"])
    uav_no_of_oid = {}
    uav_global = 0
    for u in uav_seq:
        uav_global += 1
        for oid in u[0]:
            uav_no_of_oid[oid] = uav_global
    rider_handoff = defaultdict(list)
    for r in all_routes:
        if r[2].startswith("末端"):
            pd_name = r[3]
            for it in r[4]:
                uav_n = uav_no_of_oid.get(it["oid"], 0)
                rider_handoff[r[1]].append(f"单{it['oid']} 由无人机UAV-{uav_n:02d} 在 {pd_name} 交接")
    for rid, lst in rider_handoff.items():
        w.writerow([rid, len(lst), "; ".join(lst)])

# 决策分布（老师要求：两种形式决策结果）—— 纯骑手分支 vs C-D-C 分支
with open(OUTD + rf"\{TAG}_v4_decision_{N}单.csv","w",newline="",encoding="utf-8-sig") as f:
    w = csv.writer(f)
    w.writerow(["决策模式","订单数(单)","占比(%)","服务率(%)","时间窗满足率(%)","延迟订单数(单)",
                "平均履约(送达-下单)(min)","运营成本(元)","骑手出车次数","骑手里程(km)","无人机架次","无人机里程(km)"])
    for mode, is_same in [("纯骑手直送", True), ("跨区C-D-C三段", False)]:
        ids = [x["oid"] for x in processed if x["same"] == is_same]
        if not ids:
            w.writerow([mode, 0, 0.0, "-", "-", "-", "-", "-", "-", "-", "-", "-"])
            continue
        oks = sum(1 for oid in ids if delivered_min[oid][1])
        cm = sum(delivered_min[oid][0] - oid2info[oid]["ready"] for oid in ids)/len(ids)
        trips = [r for r in all_routes if (not is_same and r[2].startswith(("首段", "末端"))) or (is_same and r[2] == "纯骑手")]
        tkm = sum(r[5] for r in trips)
        tcost = len(trips)*RIDER_FIXED + tkm*RIDER_VAR
        ukm = 0.0; utrips = 0
        if not is_same:
            utrips = sum(1 for u in uav_seq if any(oid in ids for oid in u[0]))
            ukm = sum(u[3] for u in uav_seq if any(oid in ids for oid in u[0]))
            tcost += utrips*UAV_FIXED + ukm*UAV_VAR
        w.writerow([mode, len(ids), f"{100.0*len(ids)/total:.1f}", "100", f"{100.0*oks/len(ids):.1f}",
                    len(ids)-oks, f"{cm:.2f}", f"{tcost:.2f}", len(trips), f"{tkm:.2f}", utrips, f"{ukm:.2f}"])

print(f"\n输出: {TAG}_v4_full_{N}单.csv / {TAG}_v4_routes_{N}单.csv / {TAG}_v4_uav_{N}单.csv / {TAG}_v4_handoff_{N}单.csv / {TAG}_v4_decision_{N}单.csv")
