# vector-thrust-lab — Web 六自由度交互仿真

_纵列双发矢量推力飞行器项目的浏览器端概念动力学仿真子项目_

**作者：LShang**

---

基于 Three.js 的交互式六自由度刚体动力学可视化实验室：纵列双发、正交单轴摆座、差速反扭滚转布局的力/力矩实时解算与三维演示。

> ⚠️ 这是**概念级参数化仿真**：用于布局机理研究、控制通道定性分析与方案演示。模型参数为未经标定的默认值（`MODEL-DEFAULT`），不代表真实飞行器性能。

## 运行

**方式一（零依赖，直接双击）**：`standalone.html` —— 全部代码与样式内联的单文件版，双击即可在浏览器打开，无需服务器、无需 Python。

**方式二（开发模式）**：需 Python 3（仅用于静态文件服务）：

```bat
启动.bat
```

或：

```bash
python -m http.server 8080
# 打开 http://localhost:8080
```

`index.html` 必须通过 HTTP 访问（ES Module / import map），不能直接双击；`standalone.html` 无此限制。

`standalone.html` 为构建产物（由根目录 `tools/build-standalone.py` 从 `index.html` + `src/` + `vendor/` 生成，勿手改）；修改源码后需重新生成，可用 `py -3.12 tools/build-standalone.py --check` 检测漂移。

## 结构

| 路径 | 说明 |
|---|---|
| `index.html` | 页面入口、控制面板、遥测区、方程面板 |
| `standalone.html` | 单文件版（构建产物，双击直开） |
| `src/core/` | 纯计算层：参数、数学、控制、推进、气动、六自由度积分（无 Three.js/DOM 依赖，可在 Node 中测试） |
| `src/browser/` | 浏览器适配层：Three.js 场景、程序化几何、HUD、演示序列、主题 |
| `src/main.js` | 引导与主循环装配 |
| `css/` | 界面样式 |
| `vendor/three-r170/` | 第三方依赖（Three.js r170 + es-module-shims 1.10.0，MIT） |
| `docs/` | 文档发布站（HTML 由根目录 `tools/build-docs.py` 从 `docs/` Markdown 源生成，勿手改） |
| `tests/` | Node 内置测试运行器的单元与回归测试 |

## 测试

```bash
node --test "tests/*.test.mjs"
```

## 参数来源

`src/core/parameters.mjs` 由仓库根目录 `models/aircraft-model.json` 经 `tools/sync-params.py` 生成，**不要手工编辑**。参数修改请改 JSON 源文件并重新同步。

## 模型边界

- 刚体六自由度 + 四元数姿态（右乘形式）+ 线性小迎角气动 + 一阶电机滞后 + SAS 增稳
- **双构型模式**（`S.vtolMode`，UI「悬停模式」按钮切换）：
  - **固定翼巡航**（默认）：28 m/s 四方程配平（v0.2.0，`scripts/trim_solve.py` 求解）：`aTrim=4.122°`、`thrTrim=0.192272`、`dtTrim=-18.702°`、`dfTrim=-1.739°`；摆角滑块为相对配平增量。油门开环下配平点呈弱 phugoid 慢发散（10 s 空速衰减约 12%），为模型固有行为
  - **VTOL 悬停**（机头朝天，x_b 竖直）：进入时**默认关掉机翼**（`aero=false`，气动力忽略 = 无翼构型），可手动再开。控制律为误差四元数级联（姿态外环 `ωdes=−2·vtolAttKp·qe`，qe=qCmd⁻¹⊗q 机体系误差；角速度内环复用 rateKq/rateKr/rateKp，通道效率符号与 sasMode=3 一致），与固件 `flight_control.cpp` 悬停链同构。滑块语义切换为**目标指令**（四轴式）：dw→**航向角速度 ψ̇**（°/s，rate 模式松手回中停转，与固件 RATE_MODE 偏航摇杆一致）、dt→俯仰倾斜角（尾摆）、df→侧倾角（前摆）；油门自动置悬停配平 `thrHover=√(m·g/2kT·wMax²)≈50%`。悬停自稳按钮（b-sas）仅区分开/关；关闭时 dt/df 直通摆角（dw 仍为角速度指令）。悬停模式只做姿态稳定，无位置闭环——倾斜指令会产生持续水平漂移，为模型固有行为
  - **自动定高**（悬停模式内，UI「定高」按钮 + 参考高度滑块 0–20m，拖动实时更新）：串级高度环（外环 P+I → 目标垂直速度 ±1 m/s → 内环油门修正），前馈 `thrHover/√cosγ` 倾角补偿（T∝thr² 故为 1/√cosγ 而非 1/cosγ，稳态 T·cosγ=m·g 精确成立）；独立于姿态自稳（直通模式同样生效），切换时清积分器
- `lockXY` 是删除水平速度的运动学约束，不是悬停动力学或 VTOL 能力证明
- 未包含：失速、非定常气动、螺旋桨滑流、舵机动态、传感器模型、风扰
- 渲染几何为示意性非尺度模型，与动力学力臂无关（见 `docs/04-数学建模/MOD-002`）
- 完整模型规范见根目录 `docs/04-数学建模/MOD-001-六自由度仿真模型规范.md`
