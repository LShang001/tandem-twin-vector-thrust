/**
 * @file    test_deta100_crc_guard.cpp
 * @module  固件/宿主回归测试（fw-tests：DETA100 帧校验防回退）
 * @brief   源码级守卫：DETA100 CRC16 比较不得再被注释掉
 * @details 背景（2026-10-31 审计 P0-1）：`DETA100_module.h` 的 CRC16 分支曾把
 *          `ok = false;` 注释掉，只算不比 —— 任何长度正确的帧都被接受，载荷完整性
 *          无保护；而文件头注释与 FW-INFRA 文档都记为“双 CRC 已核实正常”。
 *          该模块只能在 main.cpp 中包含（依赖串口/主循环），无法在宿主机构造完整
 *          运行时用例，故用源码守卫锁住三条不变量：
 *            ① CRC16 比较存在且失败分支启用（`ok = false;` 未被注释）；
 *            ② 失败帧计入拒帧计数（可观测性，避免静默丢帧）；
 *            ③ 文件头仍声明“双校验”，文档与实现保持一致。
 *          这不是运行时验证的替代品：运行时行为由 `pio run` 编译 + 上板前
 *          DETA100 联调覆盖。
 * @test    T1 校验分支启用；T2 拒帧计数；T3 头注释一致
 * @see     include/DETA100_module.h、docs/04-数学建模/代码理论核查/FW-INFRA-代码理论核查.md
 */
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>

static int g_fail = 0;

static void check(bool cond, const char *what)
{
  if (!cond)
  {
    std::printf("FAIL: %s\n", what);
    ++g_fail;
  }
}

// __FILE__ 可能是绝对或相对路径；取所在目录后拼 ../include/DETA100_module.h。
static std::string headerPath()
{
  std::string f = __FILE__;
  const std::string::size_type slash = f.find_last_of("/\\");
  const std::string dir = (slash == std::string::npos) ? std::string(".") : f.substr(0, slash);
  return dir + "/../include/DETA100_module.h";
}

static std::string stripSpaces(const std::string &s)
{
  std::string out;
  out.reserve(s.size());
  for (char ch : s)
  {
    if (ch != ' ' && ch != '\t')
    {
      out.push_back(ch);
    }
  }
  return out;
}

int main()
{
  std::printf("== DETA100 帧校验防回退守卫 ==\n");

  std::ifstream in(headerPath());
  check(in.good(), "能打开 include/DETA100_module.h");
  if (!in.good())
  {
    std::printf("失败 %d 项\n", g_fail);
    return 1;
  }
  std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  const std::string flat = stripSpaces(text);

  // T1：CRC16 比较与失败分支都必须启用。
  check(flat.find("if(crc16!=recv)") != std::string::npos,
        "必须保留 CRC16 比较");
  check(flat.find("ok=false;") != std::string::npos,
        "CRC16 失败分支必须置 ok=false（不得再被注释）");
  check(flat.find("//ok=false;") == std::string::npos,
        "不得存在被注释的 ok=false（P0-1 的历史缺陷形状）");

  // T2：拒帧必须可观测。
  check(flat.find("++deta100_crc16_reject_count") != std::string::npos,
        "CRC16 失败必须累加拒帧计数");
  check(flat.find("deta100_crc16_reject_count") != std::string::npos,
        "拒帧计数变量必须存在");

  // T3：头注释仍声明双校验（防止文档先于实现失效）。
  check(text.find("CRC8") != std::string::npos && text.find("CRC16") != std::string::npos,
        "文件头必须声明 CRC8 + CRC16 校验");

  if (g_fail == 0)
  {
    std::printf("全部通过\n");
    return 0;
  }
  std::printf("失败 %d 项\n", g_fail);
  return 1;
}
