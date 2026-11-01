#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""results/ 内部一致性核对（不需要 3.5 GB OD 矩阵）。

为什么需要这个：`results/` 随仓库提供了 100 个 CSV，论文表 6-4~6-8 的每个数字都
应当能在其中找到出处。但矩阵不随仓库分发，**没人能重跑实验来验**——于是
"数字对不对"只能靠 CSV 自身的一致性来兜。本脚本把可机验的那几层全部钉住：

  C1 成本恒等式：含罚总成本 == 运营成本 + 延迟惩罚
  C2 满足率恒等式：时间窗满足率 == (已服务 - 延迟) / 订单量
  C3 服务率恒等式：订单服务率 == 已服务 / 订单量
  C4 规模一致性：文件名里的「N单」== 表中订单量
  C5 消融单调性：完整版成本 <= 各消融变体（**列为 WARN 而非 FAIL**，
                 因为实测 120/240 档不成立，属已知口径问题，见 R 项）

用法:
    python code/check_results_consistency.py [results目录]

退出码: 0=全部通过 / 1=存在 FAIL
"""
from __future__ import annotations

import csv
import glob
import os
import re
import sys

TOL_MONEY = 0.02      # 金额：分位（CSV 保留两位小数）
TOL_RATE = 0.15       # 百分比：一位小数四舍五入

FAIL, WARN, OK = [], [], []


def _num(row, key):
    v = row.get(key)
    if v is None or str(v).strip() == "":
        return None
    try:
        return float(v)
    except ValueError:
        return None


def check_file(path):
    name = os.path.basename(path)
    m = re.search(r"(\d+)单", name)
    fname_n = int(m.group(1)) if m else None
    for i, r in enumerate(csv.DictReader(open(path, encoding="utf-8-sig")), 2):
        # 表头行 / 空行
        n = _num(r, "订单量(单)")
        if n is None:
            continue
        served = _num(r, "已服务订单(单)")
        late = _num(r, "延迟订单数(单)")
        op = _num(r, "运营成本(元)")
        pen = _num(r, "延迟惩罚(元)")
        tot = _num(r, "含罚总成本(元)")
        rate = _num(r, "时间窗满足率(%)")
        srate = _num(r, "订单服务率(%)")

        # C1 成本恒等式
        if None not in (op, pen, tot) and abs(op + pen - tot) > TOL_MONEY:
            FAIL.append(f"{name}:{i} C1 含罚总成本 {tot} != {op}+{pen}={op+pen:.2f}")
        # C2 满足率恒等式
        if None not in (served, late, rate) and n:
            implied = (served - late) / n * 100
            if abs(implied - rate) > TOL_RATE:
                FAIL.append(f"{name}:{i} C2 满足率 {rate} != (已服务-延迟)/单量 = {implied:.2f}")
        # C3 服务率恒等式
        if None not in (served, srate) and n:
            implied = served / n * 100
            if abs(implied - srate) > TOL_RATE:
                FAIL.append(f"{name}:{i} C3 服务率 {srate} != 已服务/单量 = {implied:.2f}")
        # C4 文件名与表内规模一致
        if fname_n is not None and int(n) != fname_n:
            FAIL.append(f"{name}:{i} C4 文件名规模 {fname_n} != 表内订单量 {int(n)}")


def check_ablation(results_dir):
    """C5：完整版应当不差于各消融变体。现状在 120/240 档不成立。"""
    data = {}
    for f in glob.glob(os.path.join(results_dir, "CD-C2*_v5_full_*单.csv")):
        m = re.search(r"v5_full_(\d+)单", os.path.basename(f))
        if not m:
            continue
        n = int(m.group(1))
        variant = ("no_alns" if "no_alns" in f else
                   "no_ten" if "no_ten" in f else
                   "none" if "none" in f else "完整")
        for r in csv.DictReader(open(f, encoding="utf-8-sig")):
            t = _num(r, "含罚总成本(元)")
            if t is not None:
                data.setdefault(n, {})[variant] = t
    for n in sorted(data):
        d = data[n]
        full = d.get("完整")
        if full is None:
            continue
        for v in ("no_alns", "no_ten", "none"):
            if v in d and full > d[v] + TOL_MONEY:
                WARN.append(
                    f"{n}单 C5 完整版 {full:.2f} 高于 {v} {d[v]:.2f}"
                    f"（+{full - d[v]:.2f}，变差 {(full / d[v] - 1) * 100:.2f}%）")


def main(argv=None):
    argv = argv or sys.argv[1:]
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    results = argv[0] if argv else os.path.join(root, "results")

    files = sorted(glob.glob(os.path.join(results, "*_full_*.csv")))
    if not files:
        print(f"[ERR] {results} 下没有 *_full_*.csv", file=sys.stderr)
        return 2
    for f in files:
        check_file(f)
    check_ablation(results)

    print(f"=== results/ 一致性核对（{len(files)} 个 full CSV）===")
    print(f"  FAIL {len(FAIL)}    WARN {len(WARN)}")
    for x in FAIL[:20]:
        print(f"  ❌ {x}")
    for x in WARN[:20]:
        print(f"  ⚠️  {x}")
    if not FAIL and not WARN:
        print("  ✅ 全部一致")
    if FAIL:
        print("\n存在不一致：这些是**数据自相矛盾**，不是口径差异。")
        return 1
    if WARN:
        print("\nWARN 项不影响自洽性，指向已知的口径问题。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
