# -*- coding: utf-8 -*-
"""C-D-C 完整版求解器 v5：论文第5章设计的 TEN（时空流网络）+ ALNS（自适应大邻域搜索）落地版。
与 v4 唯一差异：
  - 无人机干线合并：TEN（OR-Tools CP-SAT 时间展开网络模型）替代同航程贪心合并
  - 骑手分配：ALNS（3 destroy + 2 repair + 自适应权重 + 模拟退火）替代 CW 节约
其余（逐单决策 t_A/t_B、滚动时域、指标口径、输出格式）与 v4 完全一致。
产出与 v4 同名 CSV（可对比），TAG 前缀 CD-C2 区分。
用法: python solve_full_v5.py [N] [cdc|pure] [TW] [PEN] [ORD] [sim] [seed]
"""
import numpy as np, csv, math, sys, os, time, random
from collections import defaultdict

ROOT = r"E:\Program\MultiAgent-Delivery"
NP = ROOT + r"\data\OD道路距离_米.npy"
IDX = ROOT + r"\data\节点索引.csv"
ORD = sys.argv[5] if len(sys.argv) > 5 and os.path.isabs(sys.argv[5]) else (ROOT + (sys.argv[5] if len(sys.argv) > 5 else r"\data\orders_expanded\fz_orders_1550.csv"))
OUTD = ROOT + r"\03_实验设计\实验结果"
DETOUR = 1.35

RIDER_KMH = 25.0
UAV_KMH = 57.6
RIDER_FIXED = 50.0
RIDER_VAR = 0.2
UAV_FIXED = 10.0
UAV_VAR = 0.3
SERVICE_MIN = 3.0
HANDOFF_MIN = 2.0
MAX_ORDERS_PER_RIDER = 10
UAV_FLEET = 20
UAV_MAX_LOAD = 5.0
MERGE_WIN = 10.0

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
N = int(sys.argv[1]) if len(sys.argv) > 1 else 120
PURE = (len(sys.argv) > 2 and sys.argv[2] == "pure")
TW_SCALE = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
PENALTY = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0
SIM = (len(sys.argv) > 6 and sys.argv[6] == "sim")
SEED = int(sys.argv[7]) if len(sys.argv) > 7 else 7
FIX_W = float(sys.argv[8]) if len(sys.argv) > 8 else 0.0   # ALNS 目标中骑手固定成本等效里程权重（0=纯准时+里程）
random.seed(SEED)
if SIM:
    sel = orders[:N]
else:
    sel = [o for o in orders if o["pickup_name"] in pname2row and o["delivery_id"] in dsid2row][:N]

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
    best_po, best_pd = PAD_IDS[0], PAD_IDS[0]
    best_tB = 999999
    for po in PAD_IDS:
        for pd in PAD_IDS:
            if po == pd: continue
            rp = road_km(pr, po); rd = road_km(pd, dr); pp = line_km(po, pd)
            if rp < 0 or rd < 0: continue
            tB_raw = (rp+rd)/RIDER_KPM + pp/UAV_KPM + 2*SERVICE_MIN + 2*HANDOFF_MIN
            if tB_raw < best_tB:
                best_tB = tB_raw; best_po = po; best_pd = pd
    direct_km = road_km(pr, dr)
    t_A = direct_km / RIDER_KPM + 2*SERVICE_MIN
    if PURE:
        is_local = True
    else:
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
WINDOW_MIN = 15
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
        else:
            t += t_min(road_km(cur, it["dr"])) + SERVICE_MIN
            cur = it["dr"]
        if t > it["due"]: return False, t
    if mode == "first" and end_node is not None:
        t += t_min(road_km(cur, end_node))
    return True, t

def _route_time(start_node, end_node, path, t0, mode, uav_arr=None):
    """串行时间链：返回 (末时刻, 各订单送达时刻dict, 是否全部准时, 总km)"""
    t = t0
    cur = start_node
    km = 0.0
    arr = {}
    ok = True
    for it in path:
        if mode == "local":
            km += road_km(cur, it["pr"]); t += t_min(road_km(cur, it["pr"])) + SERVICE_MIN
            t = max(t, it["ready"])
            cur = it["pr"]
            km += road_km(cur, it["dr"]); t += t_min(road_km(cur, it["dr"])) + SERVICE_MIN
            arr[it["oid"]] = t
            cur = it["dr"]
        elif mode == "first":
            km += road_km(cur, it["pr"]); t += t_min(road_km(cur, it["pr"])) + SERVICE_MIN
            t = max(t, it["ready"])
            cur = it["pr"]
        else:
            ua = uav_arr.get(it["oid"], (t0, t0))[1] if uav_arr else t0
            t = max(t, ua, it["ready"])
            km += road_km(cur, it["dr"]); t += t_min(road_km(cur, it["dr"])) + SERVICE_MIN
            arr[it["oid"]] = t
            cur = it["dr"]
        if t > it["due"]: ok = False
    if mode == "first" and end_node is not None:
        km += road_km(cur, end_node); t += t_min(road_km(cur, end_node))
    return t, arr, ok, km

# ==================== ALNS（5.3.2）====================
def cw_initial(items, start_row, end_row, t0, capacity, row_attr, mode):
    """初始解：CW节约（与 v4 一致的构造式，作为 ALNS 初始解）"""
    n = len(items)
    if n == 0: return []
    rows = [it[row_attr] for it in items]
    sav = []
    for i in range(n):
        for j in range(i+1, n):
            s = road_km(rows[i], end_row) + road_km(rows[j], end_row) - road_km(rows[i], rows[j])
            sav.append((s, i, j))
    sav.sort(reverse=True)
    paths = [[i] for i in range(n)]
    for s, i, j in sav:
        if s <= 0: break
        pi = [k for k, p in enumerate(paths) if i in p]
        pj = [k for k, p in enumerate(paths) if j in p]
        if pi == pj: continue
        ri, rj = pi[0], pj[0]
        if len(paths[ri]) + len(paths[rj]) > capacity: continue
        cand = None
        if paths[ri][-1] == i and paths[rj][0] == j: cand = paths[ri] + paths[rj]
        elif paths[ri][0] == i and paths[rj][-1] == j: cand = paths[rj] + paths[ri]
        if cand is not None:
            feas, _ = _tw_feasible([items[k] for k in cand], items, start_row, end_row, t0, mode)
            if feas:
                paths[ri] = cand; del paths[rj]
    return [[items[i] for i in p] for p in paths]

def _insert_cost(route, it, items, start_row, end_row, t0, mode, uav_arr, alpha, fix_w):
    """把 it 插入 route 各位置的最小增量代价（超窗罚*alpha + 里程 + 新开路线固定成本）"""
    best = None
    for pos in range(len(route) + 1):
        new_path = route[:pos] + [it] + route[pos:]
        t_end, arr, ok, km = _route_time(start_row, end_row, new_path, t0, mode, uav_arr)
        late = sum(max(0.0, arr.get(i["oid"], t_end) - i["due"]) for i in new_path if i["oid"] in arr)
        if mode == "local":
            late = sum(max(0.0, arr[i["oid"]] - i["due"]) for i in new_path)
        cost = alpha * late + km
        if not route:
            cost += fix_w  # 新开路线：计入骑手固定成本（等效里程）
        if best is None or cost < best[0]:
            best = (cost, new_path)
    return best

def alns_assign(items, start_row, end_row, t0, capacity, row_attr, mode,
                iter_max=500, alpha=10.0, uav_arr=None, seed=7, fix_w=250.0):
    """ALNS 骑手分配。返回路线列表 [[item,...], ...] 与总里程。
    目标 = alpha*总超窗 + 总里程 + fix_w*路线数（fix_w 为骑手固定成本等效里程）。"""
    if not items: return [], 0.0
    rnd = random.Random(seed)
    routes = cw_initial(items, start_row, end_row, t0, capacity, row_attr, mode)
    if not routes: routes = []
    # 目标：总超窗罚 + 总里程 + 固定成本
    def obj(rs):
        late = 0.0; km = 0.0
        for r in rs:
            _, arr, _, k = _route_time(start_row, end_row, r, t0, mode, uav_arr)
            km += k
            for i in r:
                t = arr.get(i["oid"])
                if t is not None and t > i["due"]:
                    late += t - i["due"]
        return alpha * late + km + fix_w * len(rs)
    cur_rs = [list(r) for r in routes if r]
    cur_obj = obj(cur_rs)
    best_rs = [list(r) for r in cur_rs]
    best_obj = cur_obj
    # 算子权重
    weights = {"random": 1.0, "related": 1.0, "worst": 1.0}
    scores = {"random": 0.0, "related": 0.0, "worst": 0.0}
    used = {"random": 0, "related": 0, "worst": 0}
    T = 30.0
    for it in range(iter_max):
        # 选 destroy 算子
        tot = sum(weights.values())
        rv = rnd.random() * tot
        acc = 0.0; dname = "random"
        for k, v in weights.items():
            acc += v
            if rv <= acc: dname = k; break
        used[dname] += 1
        # destroy：移除 q 个订单
        q = max(1, int(len(items) * rnd.uniform(0.15, 0.35)))
        removed = []
        if dname == "random":
            pool = [i for r in cur_rs for i in r]
            rnd.shuffle(pool)
            removed = pool[:q]
        elif dname == "related":
            pool = [i for r in cur_rs for i in r]
            seed_it = rnd.choice(pool)
            pool.sort(key=lambda x: (abs(x["ready"] - seed_it["ready"]), (x["po"] != seed_it["po"]) + (x["pd"] != seed_it["pd"])))
            removed = pool[:q]
        else:  # worst
            def single(r):
                _, arr, _, k = _route_time(start_row, end_row, r, t0, mode, uav_arr)
                return sum(max(0.0, arr.get(i["oid"], 0) - i["due"]) for i in r)
            scored = []
            for r in cur_rs:
                for i in r:
                    scored.append((single(r), id(i), i))
            scored.sort(reverse=True)
            removed = [x[2] for x in scored[:q]]
        # 移除
        rem_set = set(id(i) for i in removed)
        new_rs = [[i for i in r if id(i) not in rem_set] for r in cur_rs]
        new_rs = [r for r in new_rs if r]
        # repair：后悔插入
        pending = list(removed)
        rnd.shuffle(pending)
        while pending:
            # 对每个待插订单计算最好与次好插入
            best_pick = None
            for oi, o in enumerate(pending):
                cands = []
                for ri, r in enumerate(new_rs):
                    c = _insert_cost(r, o, items, start_row, end_row, t0, mode, uav_arr, alpha, fix_w)
                    if c is not None and (len(r) + 1) <= capacity:
                        cands.append((c[0], ri, c[1]))
                if len(new_rs) < 500:
                    c0 = _insert_cost([], o, items, start_row, end_row, t0, mode, uav_arr, alpha, fix_w)
                    if c0 is not None:
                        cands.append((c0[0], -1, c0[1]))
                cands.sort(key=lambda x: x[0])
                if not cands: continue
                regret = (cands[1][0] - cands[0][0]) if len(cands) > 1 else cands[0][0]
                if best_pick is None or regret > best_pick[0]:
                    best_pick = (regret, oi, cands[0][1], cands[0][2])
            if best_pick is None:
                # 兜底：剩余订单各自成单独路线，保证不丢单
                for o in pending:
                    new_rs.append([o])
                break
            _, oi, ri, new_path = best_pick
            o = pending.pop(oi)
            if ri < 0:
                new_rs.append(new_path)
            else:
                new_rs[ri] = new_path
        cand_obj = obj(new_rs)
        # 接受准则：模拟退火
        if cand_obj <= cur_obj or rnd.random() < math.exp(-(cand_obj - cur_obj) / max(T, 1e-6)):
            cur_rs = new_rs; cur_obj = cand_obj
            scores[dname] += 1.0
        if cand_obj < best_obj:
            best_rs = [list(r) for r in new_rs]; best_obj = cand_obj
            scores[dname] += 2.0
        T *= 0.995
        if it % 100 == 99:
            for k in weights:
                weights[k] = max(0.1, weights[k] * 0.9 + 0.1 * (scores[k] / max(used[k], 1)))
    # 过滤空路线；计算最终里程
    out = [r for r in best_rs if r]
    tot_km = 0.0
    for r in out:
        _, _, _, k = _route_time(start_row, end_row, r, t0, mode, uav_arr)
        tot_km += k
    return out, tot_km

# ==================== TEN（5.3.1）====================
def ten_schedule(merge_input, uav_fleet=UAV_FLEET):
    """时空流网络：停机坪对 (po,pd) 订单→航班指派（时间展开网络 + CP-SAT 求解）。
    merge_input: { (po,pd): [(oid, ready_at_pad, weight, due)] }
    网络流模型：每坪对内订单按交接时刻排序，连续切分为航班（保持顺序、段内载重≤5 kg），
    目标为航班数最小（等价于最小费用流中的固定成本项最小化）。
    求解后起飞时刻按段内最晚交接时刻确定，机队按最早可用时刻贪心排队（对给定航班序列最优）。
    返回 {oid: (takeoff, arrive)}, uav_seq 列表 [(oids, takeoff, fl, dist, po_name, pd_name)]
    """
    try:
        from ortools.sat.python import cp_model
    except Exception:
        raise RuntimeError("OR-Tools 未安装: pip install ortools")
    model = cp_model.CpModel()
    pairs = {}
    for (po, pd), lst in merge_input.items():
        pairs[(po, pd)] = sorted(lst, key=lambda v: v[1])
    fl_of = {}
    dist_of = {}
    for (po, pd) in pairs:
        po_name = PADS[PAD_IDS.index(po)][0]; pd_name = PADS[PAD_IDS.index(pd)][0]
        x1, y1 = pad_plane[po_name]; x2, y2 = pad_plane[pd_name]
        dist_of[(po, pd)] = math.hypot(x1 - x2, y1 - y2)
        fl_of[(po, pd)] = dist_of[(po, pd)] / UAV_KPM
    # 连续切分建模：cut[o]=1 表示该订单为其航班起点（保持交接时刻顺序的分组）
    cut = {}
    g = {}
    for (po, pd), lst in pairs.items():
        n = len(lst)
        for k in range(n):
            cut[(po, pd, k)] = model.NewBoolVar(f"cut_{po}_{pd}_{k}")
        model.Add(cut[(po, pd, 0)] == 1)
        for k in range(n):
            gk = model.NewIntVar(0, n, f"g_{po}_{pd}_{k}")
            g[(po, pd, k)] = gk
            model.Add(gk == sum(cut[(po, pd, j)] for j in range(k + 1)) - 1)
        # 超载区间禁止同航班：任意连续区间重量和 > 5 kg 时首尾订单必须分属不同航班
        for i in range(n):
            for j in range(i + 1, n):
                if sum(lst[k][2] for k in range(i, j + 1)) > UAV_MAX_LOAD:
                    model.Add(g[(po, pd, i)] != g[(po, pd, j)])
    n_flight = model.NewIntVar(0, 10 ** 6, "n_flight")
    model.Add(n_flight == sum(cut.values()))
    model.Minimize(5000 * n_flight)
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = 15.0
    solver.parameters.num_search_workers = 4
    status = solver.Solve(model)
    groups = defaultdict(list)
    if status in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        for (po, pd), lst in pairs.items():
            for k, (oid, t, w, due) in enumerate(lst):
                fg = solver.Value(g[(po, pd, k)])
                groups[(po, pd, fg)].append((oid, t, w))
    else:
        # 求解失败回退：与 v4 一致的同航程贪心合并
        for (po, pd), lst in pairs.items():
            lst_s = sorted(lst, key=lambda v: v[1])
            grp = []
            for oid, t, w in lst_s:
                if grp and t - grp[-1][1] <= MERGE_WIN and grp[-1][2] + w <= UAV_MAX_LOAD:
                    g0 = grp[-1]; g0[0].append(oid); g0[1] = t; g0[2] += w
                else:
                    grp.append([[oid], t, w])
            for k, (oids, t, w) in enumerate(grp):
                groups[(po, pd, k)] = [(oid, t, w) for oid in oids]
    uav_records = {}
    uav_seq = []
    sched = []
    for (po, pd, _fg), grp in groups.items():
        po_name = PADS[PAD_IDS.index(po)][0]; pd_name = PADS[PAD_IDS.index(pd)][0]
        fl = fl_of[(po, pd)]; dist = dist_of[(po, pd)]
        oids = [x[0] for x in grp]
        t_req = max(x[1] for x in grp)
        sched.append((oids, t_req, fl, dist, po_name, pd_name, len(oids)))
    sched.sort(key=lambda s: s[1])
    fleet = [0.0] * uav_fleet
    for oids, t_req, fl, dist, po_name, pd_name, cnt in sched:
        idx = fleet.index(min(fleet))
        takeoff = max(t_req, fleet[idx])
        fleet[idx] = takeoff + fl
        for oid in oids:
            uav_records[oid] = (takeoff, takeoff + fl)
        uav_seq.append((tuple(oids), takeoff, fl, dist, po_name, pd_name))
    return uav_records, uav_seq

# ==================== 主流程（与 v4 一致，仅替换分配/合并模块）====================
all_routes = []
delivered_min = {}
oid2info = {x["oid"]: x for x in processed}
first_arrive = {}
uav_seq_all = []
period_cpu = []
rider_pool = {}
rider_idx = 0

def pick_rider(priority_node, period_start, item_list, prefer_node=None):
    global rider_idx
    t_open = max(period_start, min(i["ready"] for i in item_list))
    best = None
    best_d = road_km(0, priority_node)
    for rid, st in rider_pool.items():
        if st["used"]: continue
        if st["t_end"] > t_open: continue
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
    for st in rider_pool.values(): st["used"] = False
    same_orders = [x for x in batch if x["same"]]
    cross = [x for x in batch if not x["same"]]
    first_by_po = defaultdict(list)
    last_by_pd = defaultdict(list)
    for x in cross:
        first_by_po[x["po"]].append(x)
        last_by_pd[x["pd"]].append(x)

    # 同区（ALNS）
    if same_orders:
        routes, _ = alns_assign(same_orders, 0, 0, period_start, MAX_ORDERS_PER_RIDER, "pr", "local", iter_max=400, seed=SEED + p_idx, fix_w=FIX_W)
        for item_list in routes:
            rid, st, start_node = pick_rider(item_list[0]["pr"], period_start, item_list)
            seq = [start_node]
            for it in item_list: seq += [it["pr"], it["dr"]]
            km = sum(road_km(seq[i], seq[i + 1]) for i in range(len(seq) - 1))
            t = st["t_end"]
            parts = []
            sp = 0
            for k, it in enumerate(item_list):
                t = max(t, it["ready"])
                t += t_min(road_km(seq[sp], seq[sp + 1])) + SERVICE_MIN
                parts.append(f"取{it['pname']}({t:.1f})")
                sp += 1
                t += t_min(road_km(seq[sp], seq[sp + 1])) + SERVICE_MIN
                ok = t <= it["due"]
                delivered_min[it["oid"]] = (t, ok)
                parts.append(f"送{it['dname']}({t:.1f})")
                sp += 1
            st["node"] = item_list[-1]["dr"]; st["t_end"] = t; st["used"] = True
            all_routes.append((p_idx + 1, rid, "纯骑手", "-", item_list, km, " → ".join(parts)))

    # 跨区首段（ALNS）
    for po, items in first_by_po.items():
        po_name = PADS[PAD_IDS.index(po)][0]
        routes, _ = alns_assign(items, 0, po, period_start, 3, "pr", "first", iter_max=300, seed=SEED + p_idx + 1, fix_w=FIX_W)
        for item_list in routes:
            rid, st, start_node = pick_rider(item_list[0]["pr"], period_start, item_list)
            seq = [start_node]
            for it in item_list: seq.append(it["pr"])
            seq.append(po)
            km = sum(road_km(seq[i], seq[i + 1]) for i in range(len(seq) - 1))
            t = st["t_end"]
            parts = []
            sp = 0
            for k, it in enumerate(item_list):
                t = max(t, it["ready"])
                t += t_min(road_km(seq[sp], seq[sp + 1])) + SERVICE_MIN
                parts.append(f"取{it['pname']}({t:.1f})")
                sp += 1
            t += t_min(road_km(seq[sp], seq[sp + 1]))
            parts.append(f"交{po_name}({t:.1f})")
            for oid in [i["oid"] for i in item_list]: first_arrive[oid] = t
            st["node"] = po; st["t_end"] = t; st["used"] = True
            all_routes.append((p_idx + 1, rid, f"首段->{po_name}", po_name, item_list, km, " → ".join(parts)))

    # 无人机干线（TEN）
    uav_records = {}
    if cross:
        merge_input = defaultdict(list)
        for x in cross:
            merge_input[(x["po"], x["pd"])].append((x["oid"], first_arrive.get(x["oid"], period_start), x["weight"], x["due"]))
        uav_records, uav_seq = ten_schedule(dict(merge_input))
        uav_seq_all += uav_seq

    # 跨区末端（ALNS）
    for pd, items in last_by_pd.items():
        pd_name = PADS[PAD_IDS.index(pd)][0]
        routes, _ = alns_assign(items, pd, pd, period_start, 3, "dr", "last", iter_max=300, seed=SEED + p_idx + 2, uav_arr=uav_records, fix_w=FIX_W)
        for item_list in routes:
            uav_arr = {it["oid"]: uav_records.get(it["oid"], (period_start, period_start))[1] for it in item_list}
            rid, st, start_node = pick_rider(pd, period_start, item_list, prefer_node=pd)
            seq = [pd] + [it["dr"] for it in item_list]
            t = max(st["t_end"], max(uav_arr.values()))
            parts = []
            sp = 0
            for k, it in enumerate(item_list):
                t = max(t, uav_arr[it["oid"]], it["ready"])
                t += t_min(road_km(seq[sp], seq[sp + 1])) + SERVICE_MIN
                ok = t <= it["due"]
                delivered_min[it["oid"]] = (t, ok)
                parts.append(f"送{it['dname']}({t:.1f})")
                sp += 1
            st["node"] = item_list[-1]["dr"]; st["t_end"] = t; st["used"] = True
            last_km = sum(road_km(seq[i], seq[i + 1]) for i in range(len(seq) - 1))
            all_routes.append((p_idx + 1, rid, f"末端<-{pd_name}", pd_name, item_list, last_km, " → ".join(parts)))
    period_cpu.append((p_idx + 1, time.time() - t_period_start))

# === 汇总（与 v4 逐字一致）===
TAG = "纯骑手" if PURE else "CD-C2"
total = len(processed)
ontime = sum(1 for t, ok in delivered_min.values() if ok)
late = sum(1 for t, ok in delivered_min.values() if not ok)
avg_commit = sum(t - oid2info[oid]["ready"] for oid, (t, ok) in delivered_min.items()) / len(delivered_min)
max_finish = max(max(delivered_min[it["oid"]][0] - it["ready"] for it in r[4]) for r in all_routes if r[4])
rider_km = sum(r[5] for r in all_routes)
rider_trips = len(all_routes)
rider_ids = len(set(r[1] for r in all_routes))
cross_total = sum(1 for x in processed if not x["same"])
uav_km_total = sum(u[3] for u in uav_seq_all)
uav_trips = len(uav_seq_all)
uav_used = min(UAV_FLEET, max(uav_trips, 0))
rider_cost = rider_trips * RIDER_FIXED + rider_km * RIDER_VAR
uav_cost = uav_trips * UAV_FIXED + uav_km_total * UAV_VAR
total_cost = rider_cost + uav_cost
late_penalty = sum((t - oid2info[oid]["due"]) * PENALTY for oid, (t, ok) in delivered_min.items() if not ok)
total_cost_w_penalty = total_cost + late_penalty
total_cpu = sum(c for _, c in period_cpu)
period_cpu_str = " ".join(f"{p}:{c:.2f}" for p, c in period_cpu)

print(f"\n=== 完整版 {TAG} ({N}单, 窗宽x{TW_SCALE}, 罚{PENALTY}元/min, seed={SEED}) ===")
print(f"服务率: {total}/{total}=100% | 时间窗满足{ontime / total * 100:.1f}%, 延迟{late}单")
print(f"平均履约(送达-下单): {avg_commit:.1f}min | 完工max: {max_finish:.1f}min")
print(f"骑手: {rider_trips}次出车({rider_ids}骑手), {rider_km:.1f}km | 无人机: {uav_trips}架次, {uav_km_total:.1f}km")
print(f"成本: {total_cost:.1f}元 | 延误惩罚: {late_penalty:.1f}元 | 含罚总成本: {total_cost_w_penalty:.1f}元 | 经坪捎带: {cross_total}单")

with open(OUTD + rf"\{TAG}_v5_full_{N}单.csv", "w", newline="", encoding="utf-8-sig") as f:
    w = csv.writer(f)
    w.writerow(["场景", "订单量(单)", "调度使用骑手数", "调度使用无人机数", "已服务订单(单)", "未服务订单(单)",
                "订单服务率(%)", "时间窗满足率(%)", "延迟订单数(单)",
                "平均履约时间(送达-下单)(min)", "骑手完工时间最大值(min)",
                "运营成本(元)", "延迟惩罚(元)", "含罚总成本(元)",
                "CPU(s)", "每周期CPU(s)", "骑手出发次数", "骑手里程(km)", "无人机里程(km)", "无人机架次",
                "经交接点由无人机捎带(单)"])
    w.writerow([f"仿真混合·{'纯骑手' if PURE else '无人机协同'}", N, rider_ids, uav_used, total, 0, 100,
                f"{ontime / total * 100:.1f}", late,
                f"{avg_commit:.2f}", f"{max_finish:.2f}",
                f"{total_cost:.2f}", f"{late_penalty:.2f}", f"{total_cost_w_penalty:.2f}",
                f"{total_cpu:.2f}", period_cpu_str,
                rider_trips, f"{rider_km:.2f}", f"{uav_km_total:.2f}", uav_trips, cross_total])

with open(OUTD + rf"\{TAG}_v5_routes_{N}单.csv", "w", newline="", encoding="utf-8-sig") as f:
    w = csv.writer(f)
    w.writerow(["骑手编号", "已服务订单(单)", "延迟订单数(单)", "完工时间(min)", "骑手里程(km)", "服务路线"])
    for r in all_routes:
        items = r[4]
        lt = sum(1 for it in items if not delivered_min[it["oid"]][1])
        finish = max(delivered_min[it["oid"]][0] - it["ready"] for it in items) if items else 0
        w.writerow([r[1], len(items), lt, f"{finish:.2f}", f"{r[5]:.2f}", r[6]])

with open(OUTD + rf"\{TAG}_v5_uav_{N}单.csv", "w", newline="", encoding="utf-8-sig") as f:
    w = csv.writer(f)
    w.writerow(["架次", "载件数(单)", "停靠点数", "停靠序列", "起飞(min)", "返仓(min)", "航程(km)"])
    period_of_oid = {}
    for p_idx, batch in enumerate(batches):
        for x in batch: period_of_oid[x["oid"]] = p_idx + 1
    uav_global = 0
    for u in uav_seq_all:
        uav_global += 1
        oids, t0, fl, dist, po, pd = u
        pid = period_of_oid.get(oids[0], 1)
        w.writerow([f"UAV-{uav_global:02d}(第{pid}周期)", len(oids), 1,
                    f"{po} → {pd}", f"{t0:.1f}", f"{t0 + fl:.1f}", f"{dist:.2f}"])

with open(OUTD + rf"\{TAG}_v5_decision_{N}单.csv", "w", newline="", encoding="utf-8-sig") as f:
    w = csv.writer(f)
    w.writerow(["决策模式", "订单数(单)", "占比(%)", "服务率(%)", "时间窗满足率(%)", "延迟订单数(单)",
                "平均履约(送达-下单)(min)", "运营成本(元)", "骑手出车次数", "骑手里程(km)", "无人机架次", "无人机里程(km)"])
    for mode, is_same in [("纯骑手直送", True), ("跨区C-D-C三段", False)]:
        ids = [x["oid"] for x in processed if x["same"] == is_same]
        if not ids:
            w.writerow([mode, 0, 0.0, "-", "-", "-", "-", "-", "-", "-", "-", "-"])
            continue
        oks = sum(1 for oid in ids if delivered_min[oid][1])
        cm = sum(delivered_min[oid][0] - oid2info[oid]["ready"] for oid in ids) / len(ids)
        trips = [r for r in all_routes if (not is_same and r[2].startswith(("首段", "末端"))) or (is_same and r[2] == "纯骑手")]
        tkm = sum(r[5] for r in trips)
        tcost = len(trips) * RIDER_FIXED + tkm * RIDER_VAR
        ukm = 0.0; utrips = 0
        if not is_same:
            utrips = sum(1 for u in uav_seq_all if any(oid in ids for oid in u[0]))
            ukm = sum(u[3] for u in uav_seq_all if any(oid in ids for oid in u[0]))
            tcost += utrips * UAV_FIXED + ukm * UAV_VAR
        w.writerow([mode, len(ids), f"{100.0 * len(ids) / total:.1f}", "100", f"{100.0 * oks / len(ids):.1f}",
                    len(ids) - oks, f"{cm:.2f}", f"{tcost:.2f}", len(trips), f"{tkm:.2f}", utrips, f"{ukm:.2f}"])

print(f"\n输出: {TAG}_v5_full_{N}单.csv / ..._routes / ..._uav / ..._decision")
