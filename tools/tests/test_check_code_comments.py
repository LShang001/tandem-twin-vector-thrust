#!/usr/bin/env python3
"""check-code-comments.py 的 R04/R06/R08 回归测试。

@file    tools/tests/test_check_code_comments.py
@module  工具/质量门禁
@brief   锁定注释识别的边界：真违规必须命中，代码不得被误判成注释。
@details 2026-11-05 修正了 comment_run 的注释识别（块外 `*` 开头行是解引用
         代码、`/* 标签 */ code,` 是行内标签、`@code` 示例块豁免），并修正
         R08 无法识别多行函数签名后的 docstring。本测试用七组样本防回退：
         三组真违规必须报（英文注释块 / 注释掉的代码 / 缺 docstring），
         四组正常代码不得误报（解引用赋值 / 字段标签 / Doxygen 示例 /
         多行签名 + docstring）。
@theory  N/A（工程规则回归）
@frame   N/A
@units   N/A

用法（仓库根目录）：
    py -3.12 -m pytest tools/tests/test_check_code_comments.py -q
    py -3.12 tools/tests/test_check_code_comments.py        # 不依赖 pytest 直接跑
"""

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

# 真违规：连续纯英文注释（无中文、≥6 个英文词）→ 应命中 R06
REAL_EN_BLOCK = """\
// Compute the quaternion derivative using the RK4 integration step
// and normalize the result after every stage to keep the norm.
// This block is intentionally written in English for the test.
int add(int a, int b) { return a + b; }
"""

# 真违规：连续注释掉的代码 → 应命中 R04
REAL_COMMENTED_CODE = """\
// x = a + b;
// y = f(x);
// if (x > 0) { return x; }
int alive(void) { return 1; }
"""

# 代码形态：块注释外的 `*` 开头行是解引用赋值，不是注释（曾被误判）
DEREF_CODE = """\
void computeU(double phi_e, double phi_n, double phi_u, double eps_e,
              double eps_n, double eps_u, double *u_e, double *u_n, double *u_u)
{
  *u_e = phi_n * kOmega * std::sin(kLat) - phi_u * kOmega * std::cos(kLat) - eps_e;
  *u_n = -phi_e * kOmega * std::sin(kLat) - eps_n;
  *u_u = phi_e * kOmega * std::cos(kLat) - eps_u;
}
"""

# 代码形态：字段名标签后紧跟初始化代码（曾被误判成英文注释块）
INLINE_LABEL = """\
  /* speed_filter_alpha2    */ { 0.99f, 0.99f, 0.99f },
  /* angle_out_filter_alpha */ { 0.85f, 0.85f, 0.85f },
  /* output_filter_alpha    */ { 0.9f, 0.9f, 0.9f },
  /* inertia_comp_mask      */ 0x03,
"""

# 合法注释：Doxygen @code 示例块里的代码不是「被注释掉的代码」
DOXYGEN_CODE = """\
/**
 * @file demo.cpp
 * @brief 示例头（中文说明）
 * @code
 * CCAEGT_State_t state = {0};
 * void setup() { CCAEGT_Init(&state); }
 * void loop() { CCAEGT_Compute(&state, &output); }
 * @endcode
 */
int demo(void) { return 0; }
"""


# 多行签名 + 函数体内 docstring：正常 Python 写法，不得判缺 docstring
MULTILINE_SIG_WITH_DOC = '''\
def case(profile, load, speed=80.0,
         latency=20.0, outage=None):
    """构造一条用例的参数字典。"""
    return dict(profile=profile, load=load)
'''

# 多行签名且没有 docstring：仍应命中 R08
MULTILINE_SIG_NO_DOC = '''\
def case(profile, load, speed=80.0,
         latency=20.0, outage=None):
    return dict(profile=profile, load=load)
'''


def _load_checker():
    """按文件路径加载被检脚本（文件名含连字符，无法直接 import）。"""
    path = ROOT / "tools" / "check-code-comments.py"
    spec = importlib.util.spec_from_file_location("check_code_comments", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # dataclass 装饰器要求模块已注册
    spec.loader.exec_module(module)
    return module


_CHECKER = None


def _rules_for(name: str, text: str) -> set:
    """返回样本文本命中的 R04/R06/R08 规则集合。"""
    global _CHECKER
    if _CHECKER is None:
        _CHECKER = _load_checker()
    result = _CHECKER.check_file(name, text)
    return {f.rule for f in result.findings if f.rule in ("R04", "R06", "R08")}


def test_real_english_comment_block_hit():
    """纯英文注释块仍应命中 R06。"""
    assert _rules_for("real_en_block.cpp", REAL_EN_BLOCK) == {"R06"}


def test_real_commented_code_hit():
    """被注释掉的代码仍应命中 R04。"""
    assert _rules_for("real_commented_code.cpp", REAL_COMMENTED_CODE) == {"R04"}


def test_deref_code_not_comment():
    """块注释外的 `*` 开头行是代码，不得判成 R04/R06。"""
    assert _rules_for("deref_code.cpp", DEREF_CODE) == set()


def test_inline_label_not_english_block():
    """字段标签行紧跟代码，不得判成英文注释块。"""
    assert _rules_for("inline_label.cpp", INLINE_LABEL) == set()


def test_doxygen_code_block_exempt():
    """Doxygen @code 示例块不得判成注释掉的代码。"""
    assert _rules_for("doxygen_code.cpp", DOXYGEN_CODE) == set()


def test_multiline_signature_docstring_ok():
    """多行函数签名后的 docstring 属正常写法，不得判缺 docstring。"""
    assert "R08" not in _rules_for("multi_sig.py", MULTILINE_SIG_WITH_DOC)


def test_multiline_signature_missing_docstring_hit():
    """多行签名且确实没有 docstring，仍应命中 R08。"""
    assert "R08" in _rules_for("multi_sig_nodoc.py", MULTILINE_SIG_NO_DOC)


if __name__ == "__main__":
    cases = [
        ("真英文注释块命中 R06", test_real_english_comment_block_hit),
        ("注释掉的代码命中 R04", test_real_commented_code_hit),
        ("解引用代码不误报", test_deref_code_not_comment),
        ("字段标签不误报", test_inline_label_not_english_block),
        ("@code 示例块豁免", test_doxygen_code_block_exempt),
        ("多行签名带 docstring 不误报", test_multiline_signature_docstring_ok),
        ("多行签名缺 docstring 命中", test_multiline_signature_missing_docstring_hit),
    ]
    failed = 0
    for name, fn in cases:
        try:
            fn()
            print(f"[PASS] {name}")
        except AssertionError as exc:
            failed += 1
            print(f"[FAIL] {name}: {exc}")
    print("\n结果:", "全部通过" if failed == 0 else f"{failed} 条失败")
    sys.exit(0 if failed == 0 else 1)
