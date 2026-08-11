# 匿名通信协议（AnoCom 官方协议手册）

> **来源**: 官方手册《匿名通信协议.pdf》（C:\Users\12631\Desktop\匿名通信协议.pdf），2026-08-10 经 MinerU v4 (vlm) 转换，共 26 页。
> **用途**: 开发参考——协议帧格式、校验算法、功能码定义以此为准。
> **注意**: ① 本工程自定义帧/占用约定（0xF1 执行器帧、0x0D fc 字段氧压占用）与官方定义**不同**，以 `GCS/server/anocom.py` 注释 + AGENTS.md 踩坑记录为准；② 表格经 MinerU 转 HTML 表格，可读性略有损失，原始 PDF 仍保留在桌面。

## 本工程与官方手册差异对照（2026-08-10 核查，数据归位后）

> 依据: 官方手册 vs `GCS/server/anocom.py`（PC 侧事实源）+ 固件 `lib/AnoComProtocol/`。实机已联调验证的参数链路（0xE0/0xE1/0xE2/0xE3）与官方定义**逐项一致**（CMD 表、PAR_ID u16 LE、PAR_TYPE 枚举、NAME 20B、0x10 0xAA/0xAB 恢复/保存、0x00 校验帧 ID_GET+SC_GET+AC_GET）。
> **2026-08-10 数据归位**：0x0D bat 字段改真实 ADC 采样（原 12.6/16.8 占位已废）、0x20 恢复手册 PWM 语义、0x40 恢复手册遥控帧、执行器帧迁 0xF1（手册灵活格式帧）——官方上位机显示全部恢复正确。

| 功能码 | 官方定义 | 本工程实际 | 说明 |
|--------|----------|------------|------|
| 0x00 | 数据校验帧（回传 ID/SC/AC） | 一致 | 确认帧假阴性 = 2M 链路丢帧，非协议偏差 |
| 0x0D | 电压电流（bat/current） | bat=真实 ADC 电压/电流；**fc 字段占用为 P1/P2 氧压** | 前 2 字段官方上位机显示真实电压（2026-08-10 修复占位常量）；扩展字段官方忽略 |
| 0x20 | PWM 控制量 8×u16（0-10000，0.01%） | **手册语义**：ch3/ch4 电机输出 %×100 | 2026-08-10 归位（原传 raw_rc us）；官方上位机油门条恢复 |
| 0x21 | 姿态控制量 4×int16（±5000） | **alpha_ref×10**（rad/s²）+ thr×10 | 本工程为角加速度指令（四旋翼语义），非官方控制量 |
| 0x30 | GPS 信息（FIX/S_NUM/LNG/LAT/ALT/NED/PDOP/SACC/VACC） | 字段一致；**PDOP/SACC/VACC 缩放 ÷10** | 官方文字写"缩小 100 倍"与表头 ×0.01 矛盾，且 u8 无法装 20000×100——固件 `sendGPSInfo1` 实测 ×10，与 anocom.py 自洽 |
| 0x40 | 遥控器数据 10×int16（ROL/PIT/THR/YAW/AUX1-6，**地面站→飞控**） | **手册语义**：飞控→地面站回传 raw_rc 10 通道 us | 2026-08-10 归位（原为自定义执行器帧）；RC 显示数据源；本工程遥控上行仍走 ELRS/CRSF，0x40 上行不处理 |
| 0x41 | 实时控制帧 7×int16（CTRL_ROL/PIT/THR/YAWDPS/SPD_X/Y/Z） | 未使用 | 如需官方上位机地面控制可复用 |
| 0xE0-0xE3 | 参数读写链路 | **逐项一致** | 见上，已实机 121 参数全链路验证 |
| 0xF1-0xFA | 灵活格式帧（用户自定义） | **0xF1 = 执行器帧 12B**（摆角/电机/差速/饱和） | 2026-08-10 起执行器输出迁入（原 0x40），mix 输出级全模式捕获 |

> **2026-08-11 单位兼容说明**：固件内部速率 PID 已统一输出 `deg/s²`，但 0x21、CAN 和既有 AnoVars `alpha_ref_x/y/z` 仍在物理边界后发送 `rad/s²`，协议字节与缩放不变。GCS 解码同时提供派生字段 `ctrl_roll_dps2/ctrl_pitch_dps2/ctrl_yaw_dps2`；AnoVars 新增 `adps2_x/y/z` 直接观测控制器域值。

## 目录

一、匿名通信协议介绍....4
1. 通信帧格式介绍....4
2. 匿名安全通信协议....6
1) ID: 0x00: 数据校验帧....6
3. 灵活格式帧....7
1) ID: 0xF1\~0xFA: 灵活格式帧....7
二、基本信息类帧....8
1. 飞控相关信息类....8
1) ID: 0x01: 惯性传感器数据....8
2) ID: 0x02: 罗盘、气压、温度传感器数据....8
3) ID: 0x03: 飞控姿态: 欧拉角格式....8
4) ID: 0x05: 高度数据....8
5) ID: 0x06: 飞控运行模式....9
6) ID: 0x07: 飞行速度数据....9
7) ID: 0x08: 位置偏移数据....9
8) ID: 0x09: 风速估计....9
9) ID: 0x0A: 目标姿态数据....9
10) ID: 0x0B: 目标速度数据....9
11) ID: 0x0C: 回航信息....9
12) ID: 0x0D: 电压电流数据....10
13) ID: 0x0E: 外接模块工作状态....10
14) ID: 0x0F: RGB亮度信息输出....10
2. 飞控控制量输出类....11
1) ID: 0x20: PWM控制量....11
2) ID: 0x21: 姿态控制量....11
3. 飞控接收信息类....12
1) ID: 0x30: GPS传感器信息1....12
2) ID: 0x31: 原始光流信息....12
3) ID: 0x32: 通用位置型传感器数据（非捷联载体测量型）....12
4) ID: 0x33: 通用速度型传感器数据（捷联载体测量型）....12
5) ID: 0x34: 通用测距传感器数据（捷联载体测量型）....12
6) ID: 0x35: 通用图像特征点信息帧....13
4. 飞控接收控制指令类....14
1) ID: 0x40: 遥控器数据....14
2) ID: 0x41: 实时控制帧....14
5. 光流信息类....15
1) ID: 0x51: 匿名光流数据....15
6. GPS航点读写帧....16
1) ID: 0x60: 航点读取....16
2) ID: 0x61: 航点写入、航点读取返回....16
三、功能触发类帧....17
1. ID: 0xC0: CMD命令帧....17
2. ID: 0xC1: CMD功能帧....17
3. ID: 0xC2: CMD命令信息帧....17
四、参数读写类帧....19
1. ID: 0xE0: 参数命令....19
2. ID: 0xE1: 参数值写入、参数值读取返回....20

3. ID: 0xE2: 参数信息返回....20
4. ID: 0xE3: 设备信息返回....20
5. 通用校准参数....21
五、固件升级....22
六、其他帧....24
1) ID: 0xA0: LOG 信息输出--字符串....24
2) ID: 0xA1: LOG 信息输出--字符串+数字....24
3) ID: 0xB0: 图像数据....24
4) ID: 0xB1: 基于 IP 组网的数据（格式 1）....24
5) ID: 0xB2: 基于 IP 组网的数据（格式 2）....25
七、数据定义....26
1. 硬件地址定义....26

## 一、 匿名通信协议介绍

## 1. 通信帧格式介绍

为了适应多种数据类型的传输，保证高效的通信效率，所有数据的通信，均需要遵守本通信帧格式。本格式在确保通信高效、源码简单、可移植性高的基础上，实现数据正确性判断，有效避免数据传输过程中出现的错误数据导致的错误解析。

具体帧格式如下：

<table><tr><td>帧头HEAD</td><td>源地址S_ADDR</td><td>目标地址D_ADDR</td><td>功能码ID</td><td colspan="2">数据长度LEN</td><td>数据内容DATA</td><td>和校验SUM CHECK</td><td>附加校验ADD CHECK</td></tr><tr><td>1 Byte</td><td>1 Byte</td><td>1 Byte</td><td>1 Byte</td><td>1 Byte</td><td>1 Byte</td><td>n Byte</td><td>1 Byte</td><td>1 Byte</td></tr></table>

字段定义如下：

<table><tr><td>字段</td><td>长度</td><td>内容</td></tr><tr><td>帧头:HEAD</td><td>1字节</td><td>固定为十六进制的0xAB</td></tr><tr><td>源地址:S_ADDR</td><td>1字节</td><td>表示发送数据的设备ID,定义附后</td></tr><tr><td>目标地址:D_ADDR</td><td>1字节</td><td>表示接收数据的设备ID,定义附后</td></tr><tr><td>功能码:ID</td><td>1字节</td><td>表示本帧的功能识别码,定义附后</td></tr><tr><td>数据长度:LEN</td><td>2字节</td><td>表示数据区(DATA区)长度</td></tr><tr><td>数据内容:DATA</td><td>N字节</td><td>本帧数据携带的数据</td></tr><tr><td>和校验:SUM CHECK</td><td>1字节</td><td>和校验字节,计算方法附后</td></tr><tr><td>附加校验:ADD CHECK</td><td>1字节</td><td>附加校验字节,计算方法附后</td></tr></table>

匿名协议采用小端模式，低字节在前，高字节在后（协议内所有1字节以上的数据类型，比如数据长度LEN 以及DATA数据内容中的数据等，均是低字节在前，比如DATA 长度等于5，那么此时数据长度LEN=5，其对应的小端十六进制为0x0500）。

 和校验 SUM CHECK 计算方法：

从帧头 0xAB 字节开始，一直到 DATA 区结束，对每一字节进行累加操作，只取低 8 位

##  附加校验 ADD CHECK 计算方法：

计算和校验时，每进行一字节的加法运算，同时进行一次SUM CHECK的累加操作，只取低8位。

##  校验计算示例：

假设数据帧缓存为 data\_buf 数组，0xAB 存放于数组起始位置，那么 data\_buf[4]、data\_buf[5]存放的是数据长度，校验程序如下：

```c
◆ 计算校验字节:
uint8_t sumcheck = 0;
uint8_t addcheck = 0;
uint16_tflen = data_buf[4] + data_buf[5] * 256;
For(uint16_t i=0; i < (flen+6); i++)
{
    sumcheck += data_buf[i]; //从帧头开始，对每一字节进行求和，直到DATA区结束
    addcheck += sumcheck; //每一字节的求和操作，进行一次sumcheck的累加
}
//将计算出来的校验数据写入数据帧
data_buf[flen+6] = sumcheck;
data_buf[flen+7] = addcheck;

◆ 校验数据帧:
uint8_t sumcheck = 0;
uint8_t addcheck = 0;
uint16_tflen = data_buf[4] + data_buf[5] * 256;
For(uint16_t i=0; i < (flen+6); i++)
{
    sumcheck += data_buf[i]; //从帧头开始，对每一字节进行求和，直到DATA区结束
    addcheck += sumcheck; //每一字节の求和操作，进行一次sumcheck的累加
}
//如果计算出的sumcheck和addcheck和接收到的check数据相等，代表校验通过，反之数据有误
if(sumcheck == data_buf[flen+6] && addcheck == data_buf[flen+7])
    return true; //校验通过
else
    return false; //校验失败
```

## 2. 匿名安全通信协议

大家在调试时，经常会使用串口数传一类的无线通信模块，这类模块会极大提高调试和通信的便捷程度，可以无线实时监视设备状态、调整参数等。但无线数传相比有线通信，其稳定性大大降低，数据发送出去，并不能保证对侧能百分百接收到，又或者接收到了，但是数据有可能被干扰而接收到错误的数据。这在传输显示数据时没有问题，因为显示数据缺少部分数据并不会影响设备正常运行。但是如果关键敏感的数据丢失或者接收错误，比如控制命令、参数信息等，就会影响设备的正常运行，故必须定义一种安全通信协议。

匿名规定，参数写入类、命令控制类等非显示类帧，均需返回验证，其过程如下：

如发送一个参数 ID 为 10 的参数值给设备，当上位机发送参数后，会等待帧 ID 为 0 的校验帧，校验帧格式如下。只有当上位机收到校验帧，并且校验帧的ID\_GET、SC\_GET、AC\_GET与发送帧相同时，代表本次通信完成，设备已经正确收到了该参数。

若上位机在规定时间内没有收到帧 ID 为 0 的校验帧，或者校验帧数据和发送帧的不同，则上位机认为本次通信出错，会重新尝试发送该参数。

## 1) ID：0x00：数据校验帧

<table><tr><td>帧头HEAD</td><td>源地址S_ADDR</td><td>目标地址D_ADDR</td><td>功能码ID</td><td colspan="2">数据长度LEN</td><td>数据内容DATA</td><td>和校验SC</td><td>附加校验AC</td></tr><tr><td>0xAB</td><td>0xXX</td><td>0xXX</td><td>0x00</td><td>3</td><td>0</td><td>格式如下</td><td>程序计算</td><td>程序计算</td></tr></table>

DATA 区域内容：

<table><tr><td>数据类型</td><td>U8</td><td>U8</td><td>U8</td></tr><tr><td>数据内容</td><td>ID_GET</td><td>SC_GET</td><td>AC_GET</td></tr></table>

ID\_GET：需要校验的帧的帧ID码。  
SC\_GET、AC\_GET：需要校验的帧的和校验SC和附加校验AC。

## 3. 灵活格式帧

灵活格式帧，我们也可以叫做用户自定义帧，也就是用户可以自己定义数据内容格式的数据帧。可能从名字无法很好的理解灵活格式帧有什么用，那么我们举一个简单的例子。

假如我在调试一个自己写的滤波算法，传感器原始数据 A，为 int16 格式，使用滤波算法对 A 进行滤波后，得到滤波后数据 B，B 也是 int16 格式。滤波后数据经过控制算法，输出一个控制量 C，C是 int32格式。那我需要对滤波算法和控制算法进行调试，肯定是需要得到ABC三个数据的波形，根据波形进行数据分析。

那么如何将数据 ABC 发送至上位机进行显示呢，就要用到灵活格式帧了。灵活格式帧共 10 帧，帧 ID 从 0xF1到0xFA，每一帧可以携带N 个数据，每一个数据可以分别设置为U8、S16、U16、S32、Float等格式。

那么我们可以用0xF1 帧，添加 3 个数据，第一个数据为 int16，第二个数据为 int16，第三个数据为 int32。如下图：

<table><tr><td>帧ID:</td><td>0xF1</td><td>数据个数:</td><td>3</td><td>数据长度:</td><td>8</td><td>帧长度:</td><td>16</td><td>接收频率:</td><td>0</td><td></td><td></td></tr><tr><td>数据ID:</td><td>0</td><td colspan="2">数据名称:F1#0:</td><td colspan="2">A</td><td>数据类型:</td><td>Int16</td><td>▼</td><td>数据值:</td><td>0</td><td>×</td></tr><tr><td>数据ID:</td><td>1</td><td colspan="2">数据名称:F1#1:</td><td colspan="2">B</td><td>数据类型:</td><td>Int16</td><td>▼</td><td>数据值:</td><td>0</td><td>×</td></tr><tr><td>数据ID:</td><td>2</td><td colspan="2">数据名称:F1#2:</td><td colspan="2">C</td><td>数据类型:</td><td>Int32</td><td>▼</td><td>数据值:</td><td>0</td><td>×</td></tr><tr><td colspan="12">添加数据</td></tr></table>

数据名称后面的输入框，还可以输入自定义数据的名称，这里以A、B、C为例。

到此，上位机的配置完成，只需要单片机按照如下协议格式将数据发送至上位机，即可观察到对应的数据值开始刷新，并可绘制对应数据波形。

1) ID：0xF1\~0xFA：灵活格式帧

<table><tr><td>帧头HEAD</td><td>源地址S_ADDR</td><td>目标地址D_ADDR</td><td>功能码ID</td><td>数据长度LEN</td><td>数据内容DATA</td><td>和校验SC</td><td>附加校验AC</td></tr><tr><td>0xAB</td><td>0xXX</td><td>0xXX</td><td>0xF1~0xFA</td><td>数据长度</td><td>格式如下</td><td>程序计算</td><td>程序计算</td></tr></table>

DATA 区域内容：

举例说明 DATA 区域格式，例如上文，需要发送 ABC 三个数据，AB 为 int16 型，C 为 int32 型，那么 ABC 三个数据共2+2+4=8字节，那么LEN字节为8，帧ID为0xF1，DATA 区域依次放入ABC三个数据，然后计算SC、AC，完成后将本帧发送至上位机即可。

注意：

数据区使用小端模式，低字节在前。

## 二、 基本信息类帧

根据帧 ID 区分不同的信息，基本的帧格式如下：

<table><tr><td>帧头HEAD</td><td>源地址S_ADDR</td><td>目标地址D_ADDR</td><td>功能码ID</td><td>数据长度LEN</td><td>数据内容DATA</td><td>和校验SC</td><td>附加校验AC</td></tr><tr><td>0xAB</td><td>0xXX</td><td>0xXX</td><td>0xXX</td><td>数据长度</td><td>格式如下</td><td>程序计算</td><td>程序计算</td></tr></table>

比如发送 ID：0x01：惯性传感器数据，那么功能码 ID 就等于 0x01，发送ID：0x02：罗盘、气压、温度传感器数据，那么功能码 ID 就等于 0x02。

## 1. 飞控相关信息类

## 1) ID：0x01：惯性传感器数据

数据长度：13 字节

DATA 区域内容：

<table><tr><td>数据类型</td><td>int16</td><td>int16</td><td>int16</td><td>int16</td><td>int16</td><td>int16</td><td>uint8</td></tr><tr><td>数据内容</td><td>ACC_X</td><td>ACC_Y</td><td>ACC_Z</td><td>GYR_X</td><td>GYR_Y</td><td>GYR_Z</td><td>SHOCK_STA</td></tr></table>

ACC：加速度传感器数据，单位cm/ss。  
GYR：陀螺仪传感器数据，32768 对应 2000deg/s。  
SHOCK\_STA：震动状态。

## 2) ID：0x02：罗盘、温度传感器数据

DATA 区域内容：

<table><tr><td>数据类型</td><td>int16</td><td>int16</td><td>int16</td><td>int16</td><td>uint8</td></tr><tr><td>数据内容</td><td>MAG_X</td><td>MAG_Y</td><td>MAG_Z</td><td>TMP</td><td>MAG_STA</td></tr></table>

MAG：磁罗盘传感器数据。  
TMP: 传感器温度，放大10倍传输，0.1摄氏度。  
MAG\_STA：罗盘状态

## 3) ID：0x03：飞控姿态：欧拉角格式

DATA 区域内容：

<table><tr><td>数据类型</td><td>int16</td><td>int16</td><td>int16</td><td>uint8</td></tr><tr><td>数据内容</td><td>ROL*100</td><td>PIT*100</td><td>YAW*100</td><td>FUSION_STA</td></tr></table>

ROL、PIT、YAW：姿态角，依次为横滚、俯仰、航向，精确到0.01。  
FUSION \_STA：融合状态。

## 4) ID：0x04：飞控姿态：四元数格式

DATA 区域内容：

<table><tr><td>数据类型</td><td>int16</td><td>int16</td><td>int16</td><td>int16</td><td>uint8</td></tr><tr><td>数据内容</td><td>V0*10000</td><td>V1*10000</td><td>V2*10000</td><td>V3*10000</td><td>FUSION_STA</td></tr></table>

V0、V1、V2、V3：四元数，传输时扩大10000倍。  
FUSION \_STA：融合状态。

## 5) ID：0x05：高度数据

DATA 区域内容：

<table><tr><td>数据类型</td><td>Int32</td><td>Int32</td><td>Int32</td><td>Uint8</td></tr><tr><td>数据内容</td><td>ALT_BAR</td><td>ALT_ADD</td><td>ALT_FU</td><td>ALT_STA</td></tr></table>

ALT\_FU：融合后对地高度，单位厘米。

ALT\_BAR：气压计高度，单位 cm。

ALT\_ADD：附加高度传感高度数据，如超声波、激光测距，单位厘米。

ALT\_STA：测距状态。

6) ID：0x06：飞控运行模式

DATA 区域内容：

<table><tr><td>数据类型</td><td>U8</td><td>U8</td><td>U8</td><td>U8</td><td>U8</td></tr><tr><td>数据内容</td><td>MODE</td><td>SFLAG</td><td>CID</td><td>CMD0</td><td>CMD1</td></tr></table>

MODE：飞控模式。

SFLAG：功能标志， 0锁定，1解锁，2已起飞。

CID、CMD0、CMD1：当前飞控执行的指令功能（指示最近的一次，完成后复位为“悬停功能”），对应后边指令表。

7) ID：0x07：飞行速度数据

DATA 区域内容：

<table><tr><td>数据类型</td><td>int16</td><td>int16</td><td>int16</td></tr><tr><td>数据内容</td><td>SPEED_X</td><td>SPEED_Y</td><td>SPEED_Z</td></tr></table>

SPEED\_XYZ：依次为 XYZ 方向上的速度，单位 cm/s。

8) ID：0x08：位置偏移数据

DATA 区域内容：

<table><tr><td>数据类型</td><td>S32</td><td>S32</td><td>S32</td></tr><tr><td>数据内容</td><td>POS_X</td><td>POS_Y</td><td>POS_Z</td></tr></table>

POS\_XYZ：融合后位置数据，相比起飞点的位置偏移量，单位cm。

9) ID：0x09：风速估计

DATA 区域内容：

<table><tr><td>数据类型</td><td>int16</td><td>int16</td></tr><tr><td>数据内容</td><td>WIND_X</td><td>WIND_Y</td></tr></table>

WIND\_XY：风速估计，单位 cm/s。

10) ID：0x0A：目标姿态数据

DATA 区域内容：

<table><tr><td>数据类型</td><td>int16</td><td>int16</td><td>int16</td></tr><tr><td>数据内容</td><td>TAR_ROL</td><td>TAR_PIT</td><td>TAR_YAW</td></tr></table>

TAR\_ROL、TAR\_PIT、TAR\_YAW：依次为横滚、俯仰、航向的目标角度，精确到 0.01。

11) ID：0x0B：目标速度数据

DATA 区域内容：

<table><tr><td>数据类型</td><td>int16</td><td>int16</td><td>int16</td></tr><tr><td>数据内容</td><td>TAR_SPEED_X</td><td>TAR_SPEED_Y</td><td>TAR_SPEED_Z</td></tr></table>

TAR\_SPEED\_XYZ：依次为 3 轴目标速度，单位 cm/s。

12) ID：0x0C：回航信息

DATA 区域内容：

<table><tr><td>数据类型</td><td>Int16</td><td>Uint16</td></tr><tr><td>数据内容</td><td>R_A*10</td><td>R_D</td></tr></table>

R\_A：回航角度，正负180度，传输时扩大10倍变成整数传输。

R\_D：回航距离，单位为米。

## 13) ID：0x0D：电压电流数据

DATA 区域内容：

<table><tr><td>数据类型</td><td>U16</td><td>U16</td></tr><tr><td>数据内容</td><td>VOTAGE*100</td><td>CURRENT*100</td></tr></table>

VOTAGE、CURRENT：依次为电压、电流，传输时扩大100倍。

## 14) ID：0x0E：外接模块工作状态

DATA 区域内容：

<table><tr><td>数据类型</td><td>Uint8</td><td>Uint8</td><td>Uint8</td><td>Uint8</td></tr><tr><td>数据内容</td><td>STA_G_VEL</td><td>STA_G_POS</td><td>STA_GPS</td><td>STA_ALT_ADD</td></tr></table>

STA\_G\_VEL：通用速度传感器状态

STA\_G\_POS：通用位置传感器状态

STA\_GPS：GPS 传感器状态

STA\_ALT\_ADD：附加测高传感器状态

传感器的工作状态，0为无数据，1为有数据但不可用，2为正常，3为良好（3为GPS专用）。

## 15) ID：0x0F：RGB 亮度信息输出

DATA 区域内容：

<table><tr><td>数据类型</td><td>Uint8</td><td>Uint8</td><td>Uint8</td><td>Uint8</td></tr><tr><td>数据内容</td><td>BRI_R</td><td>BRI_G</td><td>BRI_B</td><td>BRI_A</td></tr></table>

BRI\_R、BRI\_G、BRI\_B、BRI\_A：分别为 RGB 指示灯的红、绿、蓝三色的亮度，BRI\_A 为单独 LED 亮度，有效范围 0-20，表示从暗到亮共 21 级亮度，0最暗，20 最亮。

CTRL\_THR 为油门控制量，范围为 0\~10000；

## 2. 飞控控制量输出类

## 1) ID：0x20： PWM 控制量

DATA 区域内容：

<table><tr><td>数据类型</td><td>U16</td><td>U16</td><td>U16</td><td>U16</td><td>U16</td><td>U16</td><td>U16</td><td>U16</td></tr><tr><td>数据内容</td><td>PWM1</td><td>PWM2</td><td>PWM3</td><td>PWM4</td><td>PWM5</td><td>PWM6</td><td>PWM7</td><td>PWM8</td></tr></table>

PWM：PWM 输出信号，范围 0-10000，默认 4 轴，单位0.01%油门。

4轴模式只输出前4通道，6轴模式输出6通道，8轴模式输出8通道。

## 2) ID：0x21：姿态控制量

DATA 区域内容：

<table><tr><td>数据类型</td><td>int16</td><td>int16</td><td>int16</td><td>int16</td></tr><tr><td>数据内容</td><td>CTRL_ROL</td><td>CTRL_PIT</td><td>CTRL_THR</td><td>CTRL_YAW</td></tr></table>

CTRL\_ROL、CTRL\_PIT、CTRL\_YAW：依次为横滚、俯仰、油门、航向控制量，范围统一为+-5000。

用户可以利用以上数据自行增加电机映射矩阵，从而控制6轴/8轴等更高轴数或者异结构飞行器。

## 3. 飞控接收信息类

1) ID：0x30： GPS 传感器信息 1

DATA 区域内容：

<table><tr><td>数据类型</td><td>U8</td><td>U8</td><td>S32</td><td>S32</td><td>S32</td><td>S16</td><td>S16</td><td>S16</td><td>U8</td><td>U8</td><td>U8</td></tr><tr><td>数据内容</td><td>FIX_STA</td><td>S_NUM</td><td>LNG*1e7</td><td>LAT*1e7</td><td>ALT_GPS</td><td>N_SPE</td><td>E_SPE</td><td>D_SPE</td><td>PDOP *0.01</td><td>SACC *0.01</td><td>VACC *0.01</td></tr></table>

FIX\_STA：定位状态，UBX 协议的 FIX\_STA。  
S\_NUM：卫星数量

LNG、LAT：依次为经度、纬度，传输时扩大10000000倍变成整数传输，使用时除以10000000即可。

ALT\_GPS：GPS 模块解算出的高度。

N\_SPE、E\_SPE、D\_SPE：NED 速度（cm/s）。

PDOP：定位精度，，0-20000，20000 表示 GPS 信息不可靠，传输时缩小 100 倍（0-200）

SACC、VACC：依次为速度精度、高度精度，最大值 20000（mm），传输时除以 100。

## 2) ID：0x31： 原始光流信息

DATA 区域内容：

<table><tr><td>数据类型</td><td>U8</td><td>S16</td><td>S16</td><td>U8</td></tr><tr><td>数据内容</td><td>TYPE</td><td>DX</td><td>DY</td><td>QUA</td></tr></table>

TYPE：光流模块型号。

DX、DY：光流移动距离（以像素为单位，一般为光流模块原始数据）。

QUA：光流信息质量，0为无效，越大代表质量越好。

3) ID：0x32：通用位置型传感器数据（非捷联载体测量型）

DATA 区域内容：

<table><tr><td>数据类型</td><td>S32</td><td>S32</td><td>S32</td></tr><tr><td>数据内容</td><td>POS_X</td><td>POS_Y</td><td>POS_Z</td></tr></table>

POS\_XYZ：依次XYZ坐标轴的位置信息，单位 mm，0x80000000表示数据无效。

注意：位置传感器坐标系与飞行器载体匿名坐标系应在飞控解锁时对准，即飞控解锁时，机头朝向位置传感器X 轴正方向，飞机左侧指向定位传感器Y轴正方向。

4) ID：0x33：通用速度型传感器数据（捷联载体测量型）

DATA 区域内容：

<table><tr><td>数据类型</td><td>S16</td><td>S16</td><td>S16</td></tr><tr><td>数据内容</td><td>SPEED_X</td><td>SPEED_Y</td><td>SPEED_Z</td></tr></table>

SPEED\_XYZ：依次XYZ 坐标轴的速度信息，单位 cm/s，0x8000 表示数据无效。

注意：速度型传感器安装时，应保证速度传感器坐标系与飞行器载体匿名坐标系对准，即飞控机头朝向速度传感器X 轴正方向，飞行器左侧指向速度传感器Y轴正方向。

5) ID：0x34：通用测距传感器数据（捷联载体测量型）

DATA 区域内容：

<table><tr><td>数据类型</td><td>U8</td><td>BYTE*n</td></tr><tr><td>数据内容</td><td>TYPE</td><td>DATA</td></tr></table>

TYPE：测距类型定义，见下表。

<table><tr><td>TYPE</td><td>类型</td><td>数据长度</td></tr><tr><td>1</td><td>向前测距</td><td>1+4=5</td></tr><tr><td>2</td><td>向后测距</td><td>1+4=5</td></tr><tr><td>3</td><td>向左测距</td><td>1+4=5</td></tr><tr><td>4</td><td>向右测距</td><td>1+4=5</td></tr><tr><td>5</td><td>向上测距</td><td>1+4=5</td></tr><tr><td>6、0</td><td>向下测距</td><td>1+4=5</td></tr><tr><td>100</td><td>圆周扫描雷达测距</td><td>1+(2+2)*n</td></tr></table>

 DATA 区域内容(TYPE=0,1,2,3,4,5,6)：

<table><tr><td>数据类型</td><td>U8</td><td>S32</td></tr><tr><td>数据内容</td><td>TYPE=0,1,2,3,4,5,6</td><td>DIST</td></tr></table>

TYPE：TYPE=0,1,2,3,4,5,6 时，表示单向测距，方向见上表 TYPE 定义，数据为 Int32 格式，单位 mm。

 DATA 区域内容(TYPE=100)：

<table><tr><td>数据类型</td><td>U8</td><td>U16+U16</td><td>N*4</td></tr><tr><td>数据内容</td><td>TYPE=100</td><td>ANGLE+DIST</td><td>ANGLE+DIST...</td></tr></table>

TYPE：TYPE=100 时，表示圆周扫描雷达测距。

ANGLE：以机头为 0 度，顺时针增加，范围为 0-360 度，精确到 0.01，比如 ANGLE=12345，表示 123.45 度。DIST：距离值，单位为厘米 cm，比如 DIST=12345，表示 123.45 米。

注意：当一帧数据只有1个测距值时，本帧数据长度5字节，一帧数据可发送多个测距值，此时，一个测距值包含2字节的角度和2字节的距离，故此时数据长度为测距点数量N\*4+1字节。

## 6) ID：0x35：通用图像特征点信息帧

DATA 区域内容：

<table><tr><td>数据类型</td><td>U8</td><td>S16</td><td>S16</td><td>U16</td></tr><tr><td>数据内容</td><td>ID</td><td>X</td><td>Y</td><td>ANGLE</td></tr></table>

ID：目标编号，0\~250 有效范围，255 表示未搜索到任何特征点，当 ID=255 时，后续 XY 以及角度信息无意义。

X、Y：目标点对于图像中点的偏移信息，以图像长宽中点为基准点，左侧为x 负半轴，下侧为y 负半轴，量程为±1000。 如果分辨率不是正方形，以长边为基准，长边两端分别对应±100，按分辨率比例，计算短边两个边缘的比例，以 320\*240 分辨率为例，横向分辨率＞纵向分辨率，以图像中心为 0 点，图像最左侧对应 x=-100，图像最右侧对应 x=100，240/320=0.75，则图像最上侧对应 y=75，图像最底侧对应 y=-75。

分辨率：320\*240，目标点像素坐标（100，80）

X=-100/(320/2)\*1000=-625

Y=80/(320/2)\*1000=500

则对应本帧的X、Y分别为-625、500。

ANGLE：角度信息，单位度，以视野正上方为基准，顺时针为正

## 4. 飞控接收控制指令类

## 1) ID：0x40：遥控器数据

DATA 区域内容：

<table><tr><td>数据类型</td><td>int16</td><td>int16</td><td>int16</td><td>int16</td><td>int16</td><td>int16</td><td>int16</td><td>int16</td><td>int16</td><td>int16</td></tr><tr><td>数据内容</td><td>ROL</td><td>PIT</td><td>THR</td><td>YAW</td><td>AUX1</td><td>AUX2</td><td>AUX3</td><td>AUX4</td><td>AUX5</td><td>AUX6</td></tr></table>

THR、YAW、ROL、PIT、AUX：依次为油门、航向、横滚、俯仰、辅助通道值，数据范围 1000-2000。  
数据为0代表没有通信或者失控（与遥控设置有关）。

## 2) ID：0x41：实时控制帧

DATA 区域内容：

<table><tr><td>数据类型</td><td>int16</td><td>int16</td><td>int16</td><td>int16</td><td>int16</td><td>int16</td><td>int16</td></tr><tr><td>数据内容</td><td>CTRL_ROL</td><td>CTRL_PIT</td><td>CTRL_THR</td><td>CTRL_YAWDPS</td><td>CTRL_SPD_X</td><td>CTRL_SPD_Y</td><td>CTRL_SPD_Z</td></tr></table>

CTRL\_ROL、CTRL\_PIT：姿态角实时控制量，角度±90，放大 100 倍传输，精确到 0.01。

CTRL\_THR：油门控制量，0-1000(0.1%) 。

CTRL\_YAW\_DPS：自旋角速度度控制量，+-200度每秒，单位度每秒。

CTRL\_SPD\_X、Y、Z：分别为 XYZ 轴的期望速度值，单位厘米每秒，最大值由飞控参数设定部分定义。

<table><tr><td></td><td colspan="5">0x41 数据有效/无效真值表(1 有效,0 无效)</td></tr><tr><td rowspan="2">数据/模式</td><td rowspan="2">姿态自稳</td><td rowspan="2">自稳+定高</td><td colspan="2">定点飞行</td><td rowspan="2">定点+程控</td></tr><tr><td>摇杆在中位</td><td>摇杆非中位</td></tr><tr><td>CTRL_ROL</td><td>1</td><td>1</td><td>0</td><td>0</td><td>0</td></tr><tr><td>CTRL_PIT</td><td>1</td><td>1</td><td>0</td><td>0</td><td>0</td></tr><tr><td>CTRL_THR</td><td>1</td><td>0</td><td>0</td><td>0</td><td>0</td></tr><tr><td>CTRL_YAWDPS</td><td>1</td><td>1</td><td>1</td><td>0</td><td>0</td></tr><tr><td>CTRL_SPD_X</td><td>0</td><td>0</td><td>1</td><td>0</td><td>0</td></tr><tr><td>CTRL_SPD_Y</td><td>0</td><td>0</td><td>1</td><td>0</td><td>0</td></tr><tr><td>CTRL_SPD_Z</td><td>0</td><td>1</td><td>1</td><td>0</td><td>0</td></tr></table>

## 注意：

1. 程控模式下，飞行控制由指令值设定，飞控全自动控制。

2. 定点模式下，摇杆前4通道任一通道不在中位，将优先响应摇杆通道对应的输入。

3. 定点模式下，摇杆的俯仰横滚中位死区为±40，油门航向中位死区为±80。

## 5. 光流信息类

## 1) ID：0x51：匿名光流数据

DATA 区域内容(MODE 0)：

<table><tr><td>数据类型</td><td>U8</td><td>U8</td><td>S8</td><td>S8</td><td>U8</td></tr><tr><td>数据内容</td><td>MODE=0</td><td>STATE</td><td>DX_0</td><td>DY_0</td><td>QUALITY</td></tr></table>

MODE：0 表示本帧为原始的光流数据

STATE：状态标记位， 0：无效，1：有效。

DX\_0、DY\_0：X、Y 轴的光流信息，对应移动的速度（像素移动速率，单位：像素/20 毫秒）

QUALITY：光流数据质量，数值越大，表示光流数据质量越好（0-255），仅供参考。

DATA 区域内容(MODE 1)：

<table><tr><td>数据类型</td><td>U8</td><td>U8</td><td>S16</td><td>S16</td><td>U8</td></tr><tr><td>数据内容</td><td>MODE=1</td><td>STATE</td><td>DX_1</td><td>DY_1</td><td>QUALITY</td></tr></table>

MODE：1 表示本帧为融合的光流数据

STATE：状态标记位， 0：无效，1：有效。

DX\_1、DY\_1：X、Y 轴的光流信息，对应移动的速度（地面速度,单位：厘米/秒）

QUALITY：光流数据质量，数值越大，表示光流数据质量越好（0-255），仅供参考。

注意：本组数据仅在高度传感器数据有效/陀螺仪无异常时有效。

DATA 区域内容(MODE 2)：

<table><tr><td>数据类型</td><td>U8</td><td>U8</td><td>S16</td><td>S16</td><td>S16</td><td>S16</td><td>S16</td><td>S16</td><td>U8</td></tr><tr><td>数据内容</td><td>MODE=2</td><td>STATE</td><td>DX_2</td><td>DY_2</td><td>DX_FIX</td><td>DY_FIX</td><td>INTEG_X</td><td>INTEG_Y</td><td>QUALITY</td></tr></table>

MODE：2 表示本帧为惯导融合的光流数据

STATE：状态标记位， 0：无效，1：有效。

DX\_2、DY\_2：X、Y 轴的光流信息，对应融合的移动的速度（地面速度,单位：厘米/秒）；

DX\_FIX、DY\_FIX：修正后的X、Y 轴的移动速度，适用于积分计算（地面速度,单位：厘米/秒）；

INTEG\_X、INTEG\_Y：X、Y 轴的速度积分值（单纯积分，仅供参考，单位厘米，-32768\~+32767 循环）。

QUALITY：光流数据质量，数值越大，表示光流数据质量越好（0-255），仅供参考。

注意：本组数据仅在光流融合有效、高度传感器数据有效/陀螺仪无异常时有效。

## 6. GPS 航点读写帧

## 1) ID：0x60：航点读取

DATA 区域内容(MODE 0)：

<table><tr><td>数据类型</td><td>U8</td></tr><tr><td>数据内容</td><td>NUM</td></tr></table>

NUM=0xFF：读取航点数量。

NUM=n $( 0 \leqslant \mathsf { n } \leqslant 0 \mathsf { x } \mathsf { F E } )$ ）：读取第 n 个航点。

注意：

 当读取航点数量时，飞控以本帧格氏也就是帧 ID：0x60 格氏返回航点数量，NUM=航点数量。

 当读取航点数据时，飞控返回帧 ID：0x61的航点内容。

2) ID：0x61：航点写入、航点读取返回

DATA 区域内容(MODE 0)：

<table><tr><td>数据类型</td><td>U8</td><td>S32</td><td>S32</td><td>S32</td><td>U16</td><td>U16</td><td>U8</td><td>U8</td><td>U8</td><td>U8</td><td>U8</td></tr><tr><td>数据内容</td><td>NUM</td><td>LNG</td><td>LAT</td><td>ALT</td><td>SPD</td><td>YAW</td><td>FUN</td><td>CMD1</td><td>CMD2</td><td>CMD3</td><td>CMD4</td></tr></table>

NUM：当前读写的航点编号，从 0 开始，0 号航点表示 HOME 起飞点。

LNG/LAT：航点经度、纬度信息，以 int32格氏通信，传输时乘以 10000000，保留小数点后 7 位。

ALT：航点高度，单位 cm。

SPD：飞行速度，单位 cm/s。

YAW：机头朝向，单位度，有以下几种用法：

 0-359：和地磁北的夹角；

 400：机头朝向目标点；

FUN：航点功能。

CMD1-4：功能参数。

注意：飞控收到本帧后，需返回校验信息，即返回帧 ID=0x00 的校验帧。

## 航点通信过程：

读取飞控：

1、上位机发送： $0 \times A B + A D D R + 0 \times 6 0 + 0 \times 0 1 + 0 \times F F + S C + A C$ ，查询飞控内有多少个航点信息；

2、飞控返回： $0 \times A B + A D D R + 0 \times 6 0 + 0 \times 0 1 + N U M + S C + A C$ ，NUM 为飞控内航点数量；

3、上位机依次发送： $\begin{array} { r } { 0 \times \mathsf { A B } + \mathsf { A D D R } + 0 \times 6 0 + 0 \times 0 1 + \mathsf { N } + \mathsf { S C } + \mathsf { A C } } \end{array}$ ，N=0，读取第一个航点；

4、飞控收到读取命令，返回第 N 个航点的内容，利用帧 ID 为 61 的航点内容帧；

5、上位机收到帧 ID 为 61 的航点内容后，重复执行第 3 步，开始读取下一个航点信息，直到读取完毕。写入飞控：

1、上位机发送： $0 \times A B + A D D R + 0 \times 6 1 + 2 2 + D A T A + S C + A C$ ，DATA 为航点数据，其中 NUM=0，即从第一个航点开始发送；

2、飞控收到帧 ID=61 的航点数据帧后，需返回帧 ID 为0 的校验帧（帧格式见本手册开头部分）；

3、上位机收到帧 ID 为0 的校验帧后，进行校验，如果单位时间内未收到校验帧或者校验出错，上位机会尝试重新发送本航点信息；若校验通过，则重复执行第一步，开始发送 NUM=1 的第二个航点信息，直到发送完毕。

4、

## 三、 功能触发类帧

为了提升上位机的通用性，上位机不再定义各类功能命令的格式，而是定义好功能帧的格式，具体命令的内容，由下位机来定义，这样就实现了一个上位机可以对应各类不同的硬件及软件的调试工作。

功能命令的使用流程：

1. 上位机发送读取命令，读取下位机的命令数量；

2. 上位机根据命令数量，依次读取下位机各条命令的内容以及相关信息；

3. 上位机读取所有命令后，在命令界面显示出所有命令内容，用户点击最后的触发按钮，即可触发对应的命令，同时命令还可以携带参数，参数的定义也在下位机实现。

## 1. ID：0xC0：CMD 命令帧

DATA 区域内容：

<table><tr><td>数据类型</td><td>U8</td><td>U8</td><td>U8</td><td></td><td></td><td></td><td></td><td></td><td></td><td></td><td></td></tr><tr><td>数据内容</td><td>CID0</td><td>CID1</td><td>CID2</td><td>VAL0</td><td>VAL1</td><td>VAL2</td><td>VAL3</td><td>VAL4</td><td>VAL5</td><td>VAL6</td><td>VAL7</td></tr></table>

CID0-2：本 CMD 功能种类，与命令信息帧的 CID 对应

VAL0-7：CMD 功能帧参数，最少 0 个数据，最多 8 个数据，可选项，与命令信息定义的数据个数和格式对应，比如命令信息定义本帧命令有 3 个 int16 型参数，那么 VAL0、1、2 为 int16 格式，后面 VAL3-7 删除不发送；若本帧命令为简单的触发命令，比如触发参数恢复默认、校准传感器等，根据命令信息定义，可只使用 3 字节CID，后面不跟任何 VAL 参数。故本数据帧长度是根据 VAL 的个数及格式可变的。

注意：飞控收到本帧后，需返回校验信息，即返回帧 ID=0x00 的校验帧。

## 2. ID：0xC1：CMD 功能帧

DATA 区域内容：

<table><tr><td>数据类型</td><td>Unt8</td><td>Unt16</td></tr><tr><td>数据内容</td><td>CMD</td><td>VAL</td></tr></table>

目标地址D\_ADDR：读取哪个设备的命令列表，目标地址就应配置为该硬件的地址信息，比如读取飞控命令列表时本字节就应为0x05。

CMD=0：读取命令列表数量，此时 VAL 无意义，等于0 即可，下位机收到后返回 0xC1 帧。返回时 CMD=0，VAL代表参数个数。

CMD=1：读取命令列表内容，此时VAL为功能序号ID，下位机收到读取命令后返回0xC2帧，

## 3. ID：0xC2：CMD 命令信息帧

DATA 区域内容：

<table><tr><td>数据类型</td><td>Uint16</td><td>Uint8*11</td><td>Char*20</td><td>Char*N</td></tr><tr><td>数据内容</td><td>CMD_ID</td><td>CMD</td><td>CMD_NAME</td><td>CMD_INFO</td></tr></table>

CMD\_ID：第几条命令

CMD：依次为 CID0-2、VAL0-7，共 11 字节，其中 VAL0-7 字节，定义本帧命令对应的参数个数和参数数据类型，定义见下表。比如本帧数据不需任何参数，那么 VAL0-7 这 8 字节均为 0；如果本帧数据需要 1 个 uint8型参数，那么 VAL0=1，VAL1-7 为 0。

CMD\_NAME：命令名称

CMD\_INFO：命令信息介绍

参数数据类型定义如下：

<table><tr><td>命令信息帧 VAL 值</td><td>对应的数据类型</td><td>占用字节</td></tr><tr><td>0</td><td>无需参数</td><td>0</td></tr><tr><td>1</td><td>Uint8</td><td>1</td></tr><tr><td>2</td><td>Int8</td><td>1</td></tr><tr><td>3</td><td>Uint16</td><td>2</td></tr><tr><td>4</td><td>Int16</td><td>2</td></tr><tr><td>5</td><td>Uint32</td><td>4</td></tr><tr><td>6</td><td>Int32</td><td>4</td></tr><tr><td>7</td><td>Float</td><td>4</td></tr></table>

举例 1：  
触发校准命令，无参数，那么可以定义如下命令信息：

{{0x01,0xA0,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},"Cali\_Trig1","Cali\_Acc",AnoPTv8CmdFun\_Cali\_T rig1}

其中，{0x01,0xA0,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}为 11 字节的命令 CMD 和 VAL 结构体，CMD0-2 分别为 0x01、0xA0、0x01，这 3字节可随便定义，但不可与其他命令相同。后面 8 字节的 VAL 均为0，表示本帧命令不需要参数。

后面的"Cali\_Trig1","Cali\_Acc",AnoPTv8CmdFun\_Cali\_Trig1，分别为命令的名称，命令的简介，命令对应的 函数指针。

## 举例 2：

一个用户自定义的带有 3 个参数的命令，第一个参数 uint8，第二个参数 int16，第三个参数 float，那么可以定义如下命令信息：

{{0x01,0xB0,0x01,0x01,0x04,0x07,0x00,0x00,0x00,0x00,0x00},"USERCMD1","UserDefCmd1",AnoPTv8CmdFu n\_UserCmd1}

其中，{0x01,0xB0,0x01,0x01,0x04,0x07,0x00,0x00,0x00,0x00,0x00}为 11 字节的命令 CMD 和 VAL 结构体，CMD0-2 分别为 0x01、0xB0、0x01，这 3字节可随便定义，但不可与其他命令相同。后面 8 字节的 VAL 信息，第一字节为 0x01，表示第一个参数为 uint8 格式，第二字节为 0x04，表示第二个参数为 int16 格式，第三字节为 0x07，表示第三个参数为 float 格式。

## 四、 参数读写类帧

为了提升上位机的通用性，上位机不再定义参数的名称以及范围等各类参数信息，而是定义好参数的格式，各类设备按照定义好的格式来传输参数，而具体参数的范围、名称等信息，在上位机进行定义，这样就实现了一个上位机可以对应各类不同的硬件及软件的调试工作。

本协议支持多种格式的参数，包括 Uint8、Int8、Uint16、Int16、Uint32、Int32、Uint64、Int64、Float、Double、String（字符串）。

上位机触发读取命令时，会首先判断是否读取了当前设备的设备信息，因为如果设备信息为空，上位机不知道是要读取哪个设备的参数。若设备信息为空，上位机首先会尝试读取设备信息，读取到设备信息后，上位机开始读取该设备的参数。若未成功读取到设备信息，则上位机以广播地址（0xFF）为目标进行读取。

 参数读取的具体实现流程：

1. 上位机发送读取命令，读取下位机的参数数量；

2. 上位机根据参数数量，依次读取所有参数的信息，包含参数名称、参数格式、简介（参数序号从 0 开始，中间不能间断）；

3. 上位机根据参数数量，依次读取所有参数的数值信息；

 参数写入的具体实现流程：（智能写入：只发送修改过的参数）

1. 上位机判断哪些参数进行过修改，从第一个修改过的参数开始，依次发送；

2. 上位机发送某个修改过的参数值至下位机，并等待校验帧；

3. 下位机收到这个参数后，返回校验帧；

4. 上位机收到校验帧并校验通过后，开始发送下一个修改过的参数，直到发送完成；

5. 若上位机接收校验帧超时或校验出错，则重新发送本参数。

 参数写入的具体实现流程：（全部写入：发送所有参数）

6. 上位机从第一个参数开始，直到最后一个参数，依次发送；

7. 上位机发送某个参数值至下位机，并等待校验帧；

8. 下位机收到这个参数后，返回校验帧；

9. 上位机收到校验帧并校验通过后，开始发送下一个参数，直到发送完成；

10. 若上位机接收校验帧超时或校验出错，则重新发送本参数。

## 1. ID：0xE0：参数命令

DATA 区域内容：

<table><tr><td>数据类型</td><td>Unt8</td><td>Unt16</td></tr><tr><td>数据内容</td><td>CMD</td><td>VAL</td></tr></table>

目标地址D\_ADDR：读取哪个设备的参数，目标地址就应配置为该硬件的地址信息，比如读取飞控参数时本字节就应为 0x05。

参数命令：

<table><tr><td>命令CMD</td><td>内容VAL</td><td>定义</td><td>返回数据</td></tr><tr><td>0x00</td><td>NA</td><td>读取设备信息,此时 VAL 无意义,等于 0 即可</td><td>下位机收到后返回 0xE3 帧</td></tr><tr><td>0x01</td><td>NA</td><td>读取参数个数,此时 VAL 无意义,等于 0 即可</td><td>下位机收到后返回 0xE0 帧,返回时 CMD=1,VAL 代表参数个数</td></tr><tr><td>0x02</td><td>ID</td><td>读取参数值,此时 VAL 为参数 ID</td><td>下位机收到后返回 0xE1 帧</td></tr><tr><td>0x03</td><td>ID</td><td>读取参数信息,此时 VAL 为参数 ID</td><td>下位机收到后返回 0xE2 帧</td></tr><tr><td rowspan="6">0x10</td><td>0xAA</td><td>所有参数恢复默认值</td><td>下位机返回 0x00 校验帧</td></tr><tr><td>0xAB</td><td>保存参数命令</td><td>下位机返回 0x00 校验帧</td></tr><tr><td>0x20</td><td>清除加速度校准数据,校准开始时会自动发送本命令,确保上传的数据为传感器原始数据</td><td>下位机返回 0x00 校验帧</td></tr><tr><td>0x21</td><td>发送完加速度校准数据后发送本命令,用于触发参数保存及启用相关功能</td><td>下位机返回 0x00 校验帧</td></tr><tr><td>0x30</td><td>清除罗盘校准数据,校准开始时会自动发送本命令,确保上传的数据为传感器原始数据</td><td>下位机返回 0x00 校验帧</td></tr><tr><td>0x31</td><td>发送完罗盘校准数据后发送本命令,用于触发参数保存及启用相关功能</td><td>下位机返回 0x00 校验帧</td></tr></table>

## 2. ID：0xE1：参数值写入、参数值读取返回

DATA 区域内容：

<table><tr><td>数据类型</td><td>Unt16</td><td>Byte*n</td></tr><tr><td>数据内容</td><td>PAR_ID</td><td>PAR_VAL</td></tr></table>

目标地址D\_ADDR：向哪个设备写入参数，目标地址就应配置为该硬件的地址信息，比如写入飞控参数时本字节就应为0x05。如果本帧为下位机对上位机读取操作的返回，本字节为上位机地址0xFE。  
PAR\_ID：参数 ID。  
PAR\_VAL：参数值，可变长度，长度由参数类型决定。

注意：飞控收到参数写入帧后，需返回校验信息，即返回帧 ID=0x00 的校验帧。

## 3. ID：0xE2：参数信息返回

DATA 区域内容：

<table><tr><td>数据类型</td><td>Unt16</td><td>Uint8</td><td>Char*20</td><td>Char*N</td></tr><tr><td>数据内容</td><td>PAR_ID</td><td>PAR_TYPE</td><td>PAR_NAME</td><td>PAR_INFO</td></tr></table>

PAR\_ID：本参数信息帧对应的参数ID。  
PAR\_TYPE：参数格式，格式定义如下：

PAR\_NAME：参数名称，字符格式，固定长度为 20 字符，不足部分补 0x00。

PAR\_INFO：参数介绍，字符格式，长度可变。

注意：上位机收到本帧数据后，并不需返回校验。

## 4. ID：0xE3：设备信息返回

DATA 区域内容：

<table><tr><td>PAR_TYPE</td><td>参数格式</td><td>参数长度</td></tr><tr><td>0</td><td>Uint8</td><td>1</td></tr><tr><td>1</td><td>Int8</td><td>1</td></tr><tr><td>2</td><td>Uint16</td><td>2</td></tr><tr><td>3</td><td>Int16</td><td>2</td></tr><tr><td>4</td><td>Uint32</td><td>4</td></tr><tr><td>5</td><td>Int32</td><td>4</td></tr><tr><td>6</td><td>Uint64</td><td>8</td></tr><tr><td>7</td><td>Int64</td><td>8</td></tr><tr><td>8</td><td>Float</td><td>4</td></tr><tr><td>9</td><td>Double</td><td>8</td></tr><tr><td>10</td><td>String</td><td>n</td></tr></table>

<table><tr><td>数据类型</td><td>Unt8</td><td>Int16</td><td>Int16</td><td>Int16</td><td>Int16</td><td>Char*N</td></tr><tr><td>数据内容</td><td>DEV_ID</td><td>HW_VER</td><td>SW_VER</td><td>BL_VER</td><td>PT_VER</td><td>DEV_NAME</td></tr></table>

DEV\_ID：设备ID，与设备地址相同。  
HW\_VER、SW\_VER、BL\_VER、PT\_VER：分别为硬件版本、软件版本、Bootloader 版本、通信协议版本。没有 Bootloader 时，BL\_VER=0 即可。  
DEV\_NAME：设备名称，字符串，最长20字节，N表示字符串长度。  
注意：上位机收到本帧数据后，并不需返回校验。

## 5. 通用校准参数

匿名助手设计有加速度计6面校准和磁罗盘校准功能，上位机会根据传感器采样数据计算出偏移、模长、误差等校准参数，为实现以上参数的自动写入，本协议规定了以上相关参数的名称。

当需要读写校准参数时，上位机会在设备的参数列表中搜索校准参数的名称，从而确定参数的 ID 范围（上位机会搜索第一个校准参数和最后一个校准参数的名称，并计算之间的参数数量，必须名称和数量均与本协议定义相同才会进入读写逻辑）。

加速度六面校准相关参数：

<table><tr><td>Name</td><td>Info</td></tr><tr><td>CAL_AOFF_X</td><td>加速度 Offset: X 轴偏移</td></tr><tr><td>CAL_AOFF_Y</td><td>加速度 Offset: Y 轴偏移</td></tr><tr><td>CAL_AOFF_Z</td><td>加速度 Offset: Z 轴偏移</td></tr><tr><td>CAL_ASEN_X</td><td>加速度 Sensitivity: X 轴感度</td></tr><tr><td>CAL_ASEN_Y</td><td>加速度 Sensitivity: Y 轴感度</td></tr><tr><td>CAL_ASEN_Z</td><td>加速度 Sensitivity: Z 轴感度</td></tr><tr><td>CAL_AEM1_X</td><td>加速度误差矩阵 1: X 轴</td></tr><tr><td>CAL_AEM1_Y</td><td>加速度误差矩阵 1: Y 轴</td></tr><tr><td>CAL_AEM1_Z</td><td>加速度误差矩阵 1: Z 轴</td></tr><tr><td>CAL_AEM2_X</td><td>加速度误差矩阵 2: X 轴</td></tr><tr><td>CAL_AEM2_Y</td><td>加速度误差矩阵 2: Y 轴</td></tr><tr><td>CAL_AEM2_Z</td><td>加速度误差矩阵 2: Z 轴</td></tr><tr><td>CAL_AEM3_X</td><td>加速度误差矩阵 3: X 轴</td></tr><tr><td>CAL_AEM3_Y</td><td>加速度误差矩阵 3: Y 轴</td></tr><tr><td>CAL_AEM3_Z</td><td>加速度误差矩阵 3: Z 轴</td></tr></table>

磁罗盘校准相关参数：

<table><tr><td>Name</td><td>Info</td></tr><tr><td>CAL_MOFF_X</td><td>磁罗盘 Offset: X 轴偏移</td></tr><tr><td>CAL_MOFF_Y</td><td>磁罗盘 Offset: Y 轴偏移</td></tr><tr><td>CAL_MOFF_Z</td><td>磁罗盘 Offset: Z 轴偏移</td></tr><tr><td>CAL_MSEN_X</td><td>磁罗盘 Sensitivity: X 轴感度</td></tr><tr><td>CAL_MSEN_Y</td><td>磁罗盘 Sensitivity: Y 轴感度</td></tr><tr><td>CAL_MSEN_Z</td><td>磁罗盘 Sensitivity: Z 轴感度</td></tr></table>

注意：以上参数，第一个参数的 ID 不做要求，根据用户设备随意存放，但加速度 15 个校准参数或磁罗盘的 6个校准参数，需要依次、连续的存储，中间不能间断或删除。

## 五、 固件升级

本协议提供通用固件升级功能，所有固件升级相关通信帧均使用帧ID：0xF0。

帧 ID：0xF0

DATA 区域内容：

<table><tr><td>数据类型</td><td>Unt8</td><td>Unt8*n</td></tr><tr><td>数据内容</td><td>IAP_CMD</td><td>IAP_DATA</td></tr></table>

固件升级流程：

1、读取设备信息（非必须），一般情况下，建议升级固件前，首先读取设备的信息，使用参数读写类帧里面的读取设备信息功能，首先发送帧ID为0xE0，命令为读取设备信息的参数命令帧，设备返回帧ID为0xE3的设备信息帧，包含设备种类、硬件版本、软件版本等信息。

2、上位机发送固件信息，包含固件总字节长度，每帧固件字节长度，固件校验信息、固件版本、适用硬件版本等信息。

3、下位机收到固件信息后，判断固件是否满足要求，满足则回复固件合法命令；反之，回复固件非法命令。

4、上位机收到固件合法命令后，上位机发送擦除命令，擦除设备指定区域内的flash。

5、下位机擦除完毕后，返回擦除成功命令；若擦除过程出错，返回擦除失败命令。

6、上位机收到擦除成功命令后，开始逐帧发送固件内容。

7、下位机收到每一帧固件后，根据是否校验通过或写入成功，返回成功或失败信息至上位机。

8、上位机收到当前发送固件帧的校验信息并校验通过后，开始发送下一帧；校验失败则重新发送本帧。

9、固件发送完毕后，上位机发送固件传输完成命令。

10、 下位机收到传输完成命令后，开始对固件进行校验，并返回校验结果。

11、 上位机根据下位机返回的校验结果，提示是否升级成功。

<table><tr><td>帧功能</td><td>IAP_CMD</td><td>IAP_DATA</td></tr><tr><td>固件命令</td><td>0x00</td><td>0x0101: 下位机判断固件信息非法0x0102: 下位机判断固件信息合法0x0110: 擦除 flash 命令0x0111: 擦除 falsh 失败0x0112: 擦除 falsh 成功0x1001+BinFNum (uint16): 第 BinFNum 帧写入失败0x1002+BinFNum (uint16): 第 BinFNum 帧写入成功0x1010: 固件发送完成命令0x1011: 下位机固件校验失败0x1012: 下位机固件校验通过</td></tr><tr><td>固件信息</td><td>0x01</td><td>BinLen: uint32: 固件总长度BinFLen: uint16: 每帧数据内固件长度BinCrc: uint16: 固件 CRC 校验数据BinVer: uint16: 固件软件版本(非必须,默认=0)HwTyp: uint8: 固件适用硬件种类(非必须,默认=0)HwVerMin: uint16: 固件适用最小硬件版本(非必须,默认=0)HwVerMax: uint16: 固件适用最大硬件版本(非必须,默认=0)Cmd: uint32: 附加命令(非必须,默认=0)</td></tr><tr><td>固件内容</td><td>0x10</td><td>BinFNum: uint16: 当前帧的序号,从 0 开始,依次递增BinData: 长度为 BinFLen 字节,固件内容数据</td></tr><tr><td></td><td></td><td></td></tr><tr><td></td><td></td><td></td></tr><tr><td></td><td></td><td></td></tr><tr><td></td><td></td><td></td></tr></table>

本功能支持两种格式的固件，bin和bia 格式。

bin：通用固件，一般编译工具可直接生成bin格式的固件文件，上位机可直接加载bin文件。使用bin文件时，固件信息中的 BinVer、HwTyp、HwVerMin、HwVerMax、Cmd 均为默认值 0，不可修改。

bia：匿名定义的一种基于bin格式的固件，在bin固件的文件头，添加100字节的信息，100字节的信息内容定义如下：

Byte0：0x62，字母 b

Byte1：0x69，字母 i

Byte2：0x61，字母 a

Byte3-Byte9：缺省值 0

Byte10-Byte11：固件版本号，uint16 格式，小端模式

Byte12-Byte13：固件 crc 校验

Byte14-Byte19：缺省值 0

Byte20：HwTyp，固件适用硬件种类

Byte21-Byte29：缺省值 0

Byte30-Byte31：HwVerMin，uint16：固件适用最小硬件版本

Byte32-Byte33：HwVerMax，uint16：固件适用最大硬件版本

Byte34-Byte49：缺省值 0

Byte50-Byte53：Cmd，uint32：附加命令

Byte54-Byte99：缺省值 0

## 六、 其他帧

## 1) ID：0xA0：LOG 信息输出--字符串

DATA 区域内容：

<table><tr><td>数据类型</td><td>Uint8</td><td>Uint8*(n-1)</td></tr><tr><td>数据内容</td><td>COLOR</td><td>STR</td></tr></table>

COLOR：颜色，0：默认，1：红色，2：绿色。

STR：需要显示的英文字符串，比如需要显示字符串“ABCDE”，则 STR 长度为 5 字节，依次为 ABCDE 的 ASC 码。

2) ID：0xA1：LOG 信息输出--字符串+数字

DATA 区域内容：

<table><tr><td>数据类型</td><td>Float</td><td>Uint8*(n-4)</td></tr><tr><td>数据内容</td><td>VAL</td><td>STR</td></tr></table>

VAL：数值，int32 格式，4 字节。

STR：需要显示的英文字符串，比如需要显示字符串“ABCDE”，则STR长度为5字节，依次为ABCDE的ASC码。

## 3) ID：0xB0：图像数据

DATA 区域内容：

<table><tr><td>数据类型</td><td>Uint16</td><td>UInt8</td><td>Uint16</td><td>Uint16</td><td>Uint8*10</td><td>Uint8*n</td></tr><tr><td>数据内容</td><td>FLAG</td><td>COLOR</td><td>WIDTH</td><td>HEIGHT</td><td>VAL*10</td><td>PIXDATA</td></tr></table>

FLAG：标志位。

COLOR：本帧图像的颜色，具体颜色对应关系见下表。

WIDTH：图像宽度，比如 320\*240 的分辨率，则 WIDTH=320。

HEIGHT：图像高度，比如 320\*240 的分辨率，则 HEIGHT =240。

VAL：调试参数，用户自定义，比如可以发送对应本帧图像的识别结果等，共 10 字节。

PIXDATA：图像数据，一位代表一个像素，PIX\_DATA 长度应为本帧数据传输的所有图像行数\*WIDTH/8。

颜色定义：

<table><tr><td>COLOR</td><td>颜色</td></tr><tr><td>0x00</td><td>灰度,0-255,1字节一个像素</td></tr><tr><td>0x01</td><td>默认颜色,黑白二值化单色,推荐</td></tr><tr><td>0x02</td><td>红色</td></tr><tr><td>0x03</td><td>绿色</td></tr><tr><td>0x04</td><td>蓝色</td></tr><tr><td>0x05</td><td>黄色</td></tr><tr><td>0x06</td><td>紫色</td></tr></table>

## 4) ID：0xB1：基于IP 组网的数据（格式 1）

DATA 区域内容：

<table><tr><td>数据类型</td><td>UInt8*4</td><td>Uint8*4</td><td>Uint16</td><td>Uint8*(n-10)</td></tr><tr><td>数据内容</td><td>数据源 IP</td><td>数据目标 IP</td><td>目标 Port</td><td>DATA</td></tr></table>

数据源 IP：发送数据设备的 IP地址，比如192.168.0.1，用户发送数据时，数据源 IP的 4 字节均写 0，数传发送时自动填充本机 IP地址到数据源 IP。

数据目标 IP：数据目标设备的 IP地址，比如 192.168.0.2，发送数据时需用户指定目标设备 IP。

目标 Port：数据目标 Socket 端口，0 为数传默认数据端口

DATA：数据内容，若干帧普通数据，数据内容为匿名 V8 协议帧，从帧头 0xAB 开始直至附加校验 AC，一次

可传输一帧或多帧数据。

5) ID：0xB2：基于 IP 组网的数据（格式 2）

DATA 区域内容：

<table><tr><td>数据类型</td><td>UInt8</td><td>Uint8</td><td>Uint16</td><td>Uint8*(n-4)</td></tr><tr><td>数据内容</td><td>数据源 IP 最低字节</td><td>数据目标 IP 最低字节</td><td>目标 Port</td><td>DATA</td></tr></table>

数据源 IP最低字节：发送数据设备的 IP地址的最低字节，比如 192.168.0.80，那么本字节为 80（十进制），用户发送数据时，本字节写 0，数传发送时自动填充本机 IP最低字节。

数据目标 IP最低字节：数据目标设备的 IP地址的最低字节，，发送数据时需用户指定。

DATA：数据内容，若干帧普通数据，数据内容为匿名 V8 协议帧，从帧头 0xAB 开始直至附加校验 AC，一次可传输一帧或多帧数据。

注意：

本帧数据的 IP 地址，自动使用本机 IP 的网段，比如 WIFI 的 DHCP 设置的 192.168.0.X 网段，当数传模块连接到路由器后，假设分配的 IP为 192.168.0.123，那么该数传使用本帧收发数据时，自动使用 192.168.0 前三字节，只需确定 IP地址的最低字节即可。

## 6) ID：0xFB：特殊数据

DATA 区域内容：

<table><tr><td>数据类型</td><td>n</td></tr><tr><td>数据内容</td><td>DATA</td></tr></table>

本协议未定义的特殊数据，比如匿名数传的专用配置信息帧等。

该 ID 的数据帧用户无需关注。

## 七、 数据定义

## 1. 硬件地址（硬件 ID）定义

<table><tr><td>地址 ID 码</td><td>十六进制</td><td>定义</td><td>是否支持修改 ID</td></tr><tr><td>1-199</td><td>0x01-0xC7</td><td>用户自定义 ID</td><td>NA</td></tr><tr><td>1-20</td><td>0x01-0x20</td><td>推荐前 20 个 ID 配置给设备的不同端口</td><td></td></tr><tr><td>200-254</td><td>0xC8-0xFE</td><td>匿名预定义设备 ID</td><td></td></tr><tr><td>210</td><td>0xD2</td><td>匿名协议至 ROS2 系统通信节点(AnoRosBridge)</td><td>不支持 q</td></tr><tr><td>219</td><td>0xDB</td><td>匿名 MCU</td><td>支持</td></tr><tr><td>220</td><td>0xDC</td><td>匿名飞控(拓空者)</td><td>支持</td></tr><tr><td>221</td><td>0xDD</td><td>匿名 IMU</td><td>支持</td></tr><tr><td>222</td><td>0xDE</td><td>匿名光流</td><td>支持</td></tr><tr><td>223</td><td>0xDF</td><td>基于匿名协议的 OPENMV</td><td>支持</td></tr><tr><td>230</td><td>0xE6</td><td>匿名数传 V410 以上</td><td>不支持</td></tr><tr><td>253</td><td>0xFD</td><td>协议模拟器</td><td>支持</td></tr><tr><td>254</td><td>0xFE</td><td>上位机</td><td>不支持</td></tr><tr><td>255</td><td>0xFF</td><td>无特定目标,用于数据广播型输出</td><td>不支持</td></tr></table>

匿名设备会默认使用预先定义的 ID，用户自己的设备，可自行分配 1 到 199 共 199 个 ID，但应注意同一通信系统内，所有设备的ID应互不相同。

推荐1到20共20个ID，配置给设备的不同端口，比如某设备为一个有5个串口的单片机，则可以为串口1到5分别配置为ID1到5，单片机ID配置为21，这样上位机如果数据发给单片机，则数据帧目标ID为21，如果上位机需要向接在单片机串口 1 的设备发送信息，则数据帧目标 ID 为 1，单片机收到目标 ID 为 1 的数据后，通过串口1发送出去即可。

特殊情况：

比如一个飞控上面，通过用户二次开发，接入了两个匿名光流传感器，如果光流传感器都用默认ID，则上位机或其他设备将无法区分两个光流模块。此时，可以修改两个匿名光流的硬件ID为不同的值（在0-199范围内），这样就可以实现识别两个光流的目的。
