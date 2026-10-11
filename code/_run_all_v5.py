# -*- coding: utf-8 -*-
"""全五档 v5 批量运行：120/240/360/480/600，档内 cdc/pure 并行，fix_w=50 定案。
用法: python _run_all_v5.py
输出: 03_实验设计\实验结果\*_v5_full_*.csv（与 v4 文件并存）+ 日志 _run_v5_{mode}_{N}.txt
"""
import subprocess, sys, os, time, concurrent.futures

ROOT = r"E:\Program\MultiAgent-Delivery"
PY = r"D:\miniconda3_new\python.exe"
SCRIPT = os.path.join(ROOT, "scripts", "solve_full_v5.py")
ORD = os.path.join(ROOT, "data", "orders_expanded", "fz_orders_sim_mix_600.csv")
OUT = os.path.join(ROOT, "03_实验设计", "实验结果")

SIZES = [120, 240, 360, 480, 600]
FIX_W = "50"

def run_one(mode, n):
    log = os.path.join(OUT, f"_run_v5_{mode}_{n}.txt")
    t0 = time.time()
    with open(log, "w", encoding="utf-8") as f:
        r = subprocess.run([PY, "-X", "utf8", SCRIPT, str(n), mode, "1.0", "2.0", ORD, "sim", "7", FIX_W],
                           stdout=f, stderr=subprocess.STDOUT)
    dt = time.time() - t0
    return mode, n, r.returncode, dt

t_all = time.time()
for n in SIZES:
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as ex:
        futs = [ex.submit(run_one, m, n) for m in ("cdc", "pure")]
        for f in concurrent.futures.as_completed(futs):
            mode, nn, rc, dt = f.result()
            print(f"v5 {mode} {nn}单: rc={rc} 耗时={dt:.1f}s", flush=True)
print(f"全部完成，总耗时 {time.time() - t_all:.1f}s")
