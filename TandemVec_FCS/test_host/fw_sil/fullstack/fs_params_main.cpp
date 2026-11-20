/**
 * @file    fs_params_main.cpp
 * @module  固件/全链路数字孪生（fs：通信层参数页运行期验证）
 * @brief   让真实 `src/ano_params.cpp` 的参数注册表在宿主跑起来，锁定读写与非法值语义
 * @details 被测：`src/ano_params.cpp`（anoParamCount / NameAt / ReadFloat / WriteFloat /
 *          IdByName / anoParamsHandleRx 的参数側），**一行不改**地链接进来。
 *          输入：参数表本身（169 条，指向 `kFlightCtrlParams` 可变实例）；
 *          处理：按名字查 ID → 读默认值 → 写合法值 → 写 NaN/Inf → 越界 ID；
 *          输出：逐项 PASS/FAIL，退出码即结论。
 * @theory  N/A（参数治理逻辑，不是控制/估计算法）
 * @ref     src/ano_params.cpp、docs/00-项目治理/GOV-006（FIX-02 参数包线校验）
 * @frame   N/A
 * @units   N/A（参数值量纲随字段，见 FlightCtrlParams.h）
 * @note    ★本测试同时是「FIX-02 是否已实现」的判据：当前实现只拒绝 NaN/Inf，
 *          没有 min/max 包线——写 `att_roll.kp = 1e6` 会成功。这不是本测试的缺陷，
 *          而是把"包线校验尚未落地"这个事实固化成可执行证据。
 * @test    本文件即测试：退出码 0 = 通过。
 */

#include "Arduino.h"
#include "ano_params.h"
#include "FlightCtrlParams.h"

#include <cmath>
#include <cstdio>
#include <string>

namespace
{
int g_fail = 0;
void check(bool cond, const char *name)
{
  std::printf(cond ? "[PASS] %s\n" : "[FAIL] %s\n", name);
  if (!cond) ++g_fail;
}
}  // namespace

int main()
{
  std::printf("== 真实 ano_params.cpp 参数页验证 ==\n");

  // ---------- 1) 注册表规模 ----------
  const uint16_t n = anoParamCount();
  std::printf("  参数总数 = %u\n", static_cast<unsigned>(n));
  check(n == 169, "参数注册表 169 条（133 + 15 共轴 + 21 单涵道）");

  // ---------- 2) 按名字查找 ----------
  const int16_t id_kp = anoParamIdByName("att_roll.kp");
  std::printf("  att_roll.kp → id=%d\n", static_cast<int>(id_kp));
  check(id_kp >= 0, "按名字找到 att_roll.kp");
  check(anoParamIdByName("no.such.param") < 0, "未知名字返回负值");
  if (id_kp >= 0)
  {
    const char *nm = anoParamNameAt(static_cast<uint16_t>(id_kp));
    check(nm != nullptr && std::string(nm) == "att_roll.kp", "名字反查一致");
  }

  // ---------- 3) 读默认值 ----------
  float v = 0.0f;
  const bool rok = anoParamReadFloat(static_cast<uint16_t>(id_kp), &v);
  std::printf("  默认 att_roll.kp = %.6f（kFlightCtrlParamsDefaults 同名 %.6f）\n",
              static_cast<double>(v),
              static_cast<double>(kFlightCtrlParamsDefaults.att_roll.kp));
  check(rok, "读取 att_roll.kp 成功");
  check(std::fabs(v - kFlightCtrlParamsDefaults.att_roll.kp) < 1e-6f,
        "读到的就是默认值（表指针指向可变实例）");

  // ---------- 4) 写合法值：表与实例同步 ----------
  const float kNew = 3.25f;
  const bool wok = anoParamWriteFloat(static_cast<uint16_t>(id_kp), kNew);
  float back = 0.0f;
  anoParamReadFloat(static_cast<uint16_t>(id_kp), &back);
  check(wok, "写入合法值返回成功");
  check(std::fabs(back - kNew) < 1e-6f, "读回等于写入值");
  check(std::fabs(kFlightCtrlParams.att_roll.kp - kNew) < 1e-6f,
        "kFlightCtrlParams 实例被真实改写（不是影子副本）");

  // ---------- 5) 非法值：NaN / Inf 必须拒绝且不污染 ----------
  const float before = kFlightCtrlParams.att_roll.kp;
  check(!anoParamWriteFloat(static_cast<uint16_t>(id_kp), NAN), "拒绝 NaN");
  check(!anoParamWriteFloat(static_cast<uint16_t>(id_kp), INFINITY), "拒绝 +Inf");
  check(!anoParamWriteFloat(static_cast<uint16_t>(id_kp), -INFINITY), "拒绝 -Inf");
  check(kFlightCtrlParams.att_roll.kp == before, "非法写入未污染原值");

  // ---------- 6) 越界 ID ----------
  check(!anoParamWriteFloat(169, 1.0f), "越界 ID(169) 写入被拒");
  float dummy = 0.0f;
  check(!anoParamReadFloat(169, &dummy), "越界 ID(169) 读取被拒");

  // ---------- 7) FIX-02 现状：没有包线校验 ----------
  // 写一个物理上荒谬的增益应当被拒，但当前实现只判 NaN/Inf。
  const float kAbsurd = 1.0e6f;
  const bool absurd_ok = anoParamWriteFloat(static_cast<uint16_t>(id_kp), kAbsurd);
  std::printf("  写 att_roll.kp=1e6 结果 = %s（FIX-02 包线校验状态）\n",
              absurd_ok ? "接受" : "拒绝");
  if (absurd_ok)
  {
    // 这是已知缺口，不是本测试失败：如实记录当前行为，并恢复原值。
    std::printf("  [NOTE] 当前无 min/max 包线，FIX-02 未落地（GOV-006 P0）\n");
  }
  anoParamWriteFloat(static_cast<uint16_t>(id_kp), kFlightCtrlParamsDefaults.att_roll.kp);

  // ---------- 8) bool/uint8 语义 ----------
  const int16_t id_en = anoParamIdByName("att_roll.enabled");
  if (id_en >= 0)
  {
    anoParamWriteFloat(static_cast<uint16_t>(id_en), 0.0f);
    float b = 1.0f;
    anoParamReadFloat(static_cast<uint16_t>(id_en), &b);
    check(b == 0.0f, "bool 参数写 0 读回 0");
    anoParamWriteFloat(static_cast<uint16_t>(id_en), 1.0f);
    anoParamReadFloat(static_cast<uint16_t>(id_en), &b);
    check(b == 1.0f, "bool 参数写 1 读回 1");
  }

  std::printf("\n%s（失败 %d 项）\n", g_fail == 0 ? "=== 全部通过 ===" : "=== 存在失败 ===",
              g_fail);
  return g_fail == 0 ? 0 : 1;
}