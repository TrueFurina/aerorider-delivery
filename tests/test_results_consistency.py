# -*- coding: utf-8 -*-
"""results/ 一致性的回归锁（不需要 3.5 GB OD 矩阵）。

本仓库是论文复现包：`results/` 下的 100 个 CSV 就是论文表 6-4~6-8 的出处。
但 OD 矩阵不随仓库分发，**任何人都无法重跑实验来验**——因此"这些数字有没有被
改坏"只能靠 CSV 自身的一致性来兜。这条测试把那几层恒等式钉住，
防止后续重新生成结果时悄悄写出自相矛盾的表。
"""
import csv
import os
import sys

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "code"))

import check_results_consistency as chk

RESULTS = os.path.join(ROOT, "results")


def test_checker_reports_no_hard_failures():
    """现有 results/ 必须零 FAIL（恒等式全成立）。"""
    rc = chk.main([RESULTS])
    assert rc == 0, f"存在硬矛盾：{chk.FAIL}"
    assert chk.FAIL == [], f"results/ 自相矛盾：{chk.FAIL[:5]}"


def test_checker_detects_tampered_total(tmp_path):
    """变异验证：篡改「含罚总成本」必须被拦住。

    没有这条，上一条断言可能因为检查器恒返回 0 而空转通过。
    """
    files = [f for f in os.listdir(RESULTS) if f.endswith("_full_600单.csv")]
    assert files, "前提：存在 600 单结果"
    src = os.path.join(RESULTS, files[0])
    dst = tmp_path / files[0]
    rows = list(csv.reader(open(src, encoding="utf-8-sig")))
    rows[1][rows[0].index("含罚总成本(元)")] = "99999.99"
    with open(dst, "w", encoding="utf-8-sig", newline="") as f:
        csv.writer(f).writerows(rows)
    chk.FAIL.clear(); chk.WARN.clear()
    rc = chk.main([str(tmp_path)])
    assert rc == 1, "篡改后检查器仍判通过 —— 检查器失效"
    assert any("C1" in x for x in chk.FAIL)


@pytest.mark.parametrize("field,code", [
    ("时间窗满足率(%)", "C2"),
    ("订单服务率(%)", "C3"),
    ("订单量(单)", "C4"),
])
def test_checker_detects_each_identity(tmp_path, field, code):
    """三条恒等式各自独立可被击穿（避免其中两条互相掩盖）。"""
    name = "CD-C2_v5_full_360单.csv"
    src = os.path.join(RESULTS, name)
    dst = tmp_path / name
    rows = list(csv.reader(open(src, encoding="utf-8-sig")))
    j = rows[0].index(field)
    rows[1][j] = "999.9"
    with open(dst, "w", encoding="utf-8-sig", newline="") as f:
        csv.writer(f).writerows(rows)
    chk.FAIL.clear(); chk.WARN.clear()
    chk.main([str(tmp_path)])
    assert any(code in x for x in chk.FAIL), f"{code} 未被触发：{chk.FAIL}"


def test_ablation_warning_is_not_silent():
    """C5：完整版不优于消融变体时必须是 WARN，不能静默。

    当前 120/240 档确实如此（ALNS 的 alpha=10 与计分 PENALTY=2 口径不一致所致）。
    这条测试锁住"如实报出来"这个行为本身；若将来口径对齐、WARN 消失，
    本测试会失败，届时应当 knowingly 更新它，而不是删掉。
    """
    chk.FAIL.clear(); chk.WARN.clear()
    chk.main([RESULTS])
    assert chk.WARN, "已知的 120/240 档口径问题没有被报出来 —— 告警静默了"
