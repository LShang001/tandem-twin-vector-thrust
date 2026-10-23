# 第8章 平台式惯导系统

## 第8章 平台式惯导系统

## 8. 1 平台式惯导系统的力学编排

平台式惯导系统的力学编排是指实现正确控制惯性平台和解算导航参数的方案和方程，包括平台指令角速度的计算公式及速度和位置的解算方程。

## 8. 1. 1 指北方位惯导系统的力学编排

## 1.平台的指令角速度

指北方位系统以地理坐标系为导航坐标系，也就是说理想、平台坐标系 即为地理坐标系 $g$ C平台模拟地理坐标系，将三个加速度计的敏感轴定向在当地的东、北、天方位上。所以平台应该跟踪地理坐标系，即

$$
\boldsymbol {\omega} _ {i T} = \boldsymbol {\omega} _ {i g}
$$

而地理坐标系的旋转角速度由两部分组成:跟随地球旋转的角速度 ${ \pmb { \omega } } _ { i e }$ 和由于运载体运动而引起的相对地球的旋转角速度 ${ \pmb \omega } _ { \mathcal { E } }$ ，即

$$
\pmb {\omega} _ {i g} = \pmb {\omega} _ {i e} + \pmb {\omega} _ {e g}
$$

由图 8. 1.

$$
\boldsymbol {\omega} _ {i e} ^ {T} = \boldsymbol {\omega} _ {i e} ^ {g} = \left[ \begin{array}{c} 0 \\ \omega_ {i e} \cos L \\ \omega_ {i e} \sin L \end{array} \right]\tag{8.1.1a}
$$

$$
\boldsymbol {\omega} _ {e T} ^ {T} = \boldsymbol {\omega} _ {e g} ^ {g} = \left[ \begin{array}{c} - \frac {V _ {N}}{R _ {M}} \\ \frac {V _ {E}}{R _ {N} \cos L} \cos L \\ \frac {V _ {E}}{R _ {N} \cos L} \sin L \end{array} \right] = \left[ \begin{array}{c} - \frac {V _ {N}}{R _ {M}} \\ \frac {V _ {E}}{R _ {N}} \\ \frac {V _ {E}}{R _ {N}} \tan L \end{array} \right]\tag{8.1.1b}
$$

![](images/cb88ac5ebebfc96d750cd7ae0fb7e868beef66e2cfb8247cc0288ee5724469e8.jpg)  
(a)

![](images/96fc32c1f693603299f113d79f2f8ff8f860b03e8549d8614f580870b4e77091.jpg)  
(b)

![](images/c81b2438080f54a93b837b73cd6e98b266f76cbca95df66993c1642acb0d561f.jpg)  
(c)  
8. 1. 地理坐标系的旋转角速度

所以

$$
\boldsymbol {\omega} _ {i T} ^ {T} = \boldsymbol {\omega} _ {i g} ^ {g} = \boldsymbol {\omega} _ {i e} ^ {g} + \boldsymbol {\omega} _ {e g} ^ {g} = \left[ \begin{array}{c} - \frac {V _ {N}}{R _ {M}} \\ \omega_ {i e} \cos L + \frac {V _ {E}}{R _ {N}} \\ \omega_ {i e} \sin L + \frac {V _ {E}}{R _ {N}} \tan L \end{array} \right]\tag{8.1.2}
$$

平台的指令角速度为

$$
\omega_ {\mathrm{cmd} x} ^ {T} = - \frac {V _ {N}}{R _ {M}}\tag{8.1.3a}
$$

$$
\omega_ {\mathrm{cmdy}} ^ {T} = \omega_ {i e} \cos L + \frac {V _ {E}}{R _ {N}}\tag{8.1.3b}
$$

$$
\omega_ {\mathrm{cmd} z} ^ {T} = \omega_ {i e} \sin L + \frac {V _ {E}}{R _ {N}} \tan L\tag{8.1.3c}
$$

## 2. 速度方程

将式 (8. 1. 1)代入式 (7.4.7) ，得

$$
\left[ \begin{array}{c} \dot {V} _ {E} \\ \dot {V} _ {N} \\ \dot {V} _ {U} \end{array} \right] = \left[ \begin{array}{c} f _ {E} \\ f _ {N} \\ f _ {U} \end{array} \right] - \left[ \begin{array}{c c c} 0 & - \left(2 \omega_ {i e} \sin L + \frac {V _ {E}}{R _ {N}} \tan L\right) & 2 \omega_ {i e} \cos L + \frac {V _ {E}}{R _ {N}} \\ 2 \omega_ {i e} \sin L + \frac {V _ {E}}{R _ {N}} \tan L & 0 & \frac {V _ {N}}{R _ {M}} \\ - \left(2 \omega_ {i e} \cos L + \frac {V _ {E}}{R _ {N}}\right) & - \frac {V _ {N}}{R _ {M}} & 0 \end{array} \right]
$$

$$
\times \left[ \begin{array}{l} V _ {E} \\ V _ {N} \\ V _ {U} \end{array} \right] + \left[ \begin{array}{c} 0 \\ 0 \\ - g \end{array} \right]
$$

展开后，得

$$
\dot {V} _ {E} = f _ {E} + \left(2 \omega_ {i e} \sin L + \frac {V _ {E}}{R _ {N}} \tan L\right) V _ {N} - \left(2 \omega_ {i e} \cos L + \frac {V _ {E}}{R _ {N}}\right) V _ {U}\tag{8.1.4a}
$$

$$
\dot {V} _ {N} = f _ {N} - \left(2 \omega_ {i e} \sin L + \frac {V _ {E}}{R _ {N}} \tan L\right) V _ {E} - \frac {V _ {N}}{R _ {M}} V _ {U}\tag{8.1.4b}
$$

$$
\dot {V} _ {U} = f _ {U} + \left(2 \omega_ {i e} \cos L + \frac {V _ {E}}{R _ {N}}\right) V _ {E} + \frac {V _ {N} ^ {2}}{R _ {M}} - g\tag{8.1.4c}
$$

对于飞机和舰船，垂直速度远比水平速度小，所以在计算 $V _ { E }$ 和 $V _ { N }$ 时可略去 $V _ { U }$ 的影响，式(8. 1. 4a) (8. 1. 4b)可简化为

$$
\dot {V} _ {E} = f _ {E} + \left(2 \omega_ {i e} \sin L + \frac {V _ {E}}{R _ {N}} \tan L\right) V _ {N}\tag{8.1.5a}
$$

$$
\dot {V} _ {N} = f _ {N} - \left(2 \omega_ {i e} \sin L + \frac {V _ {E}}{R _ {N}} \tan L\right) V _ {E}\tag{8.1.5b}
$$

水平速度为

$$
V = \sqrt {V _ {E} ^ {2} + V _ {N} ^ {2}}
$$

## 3. 经、纬度方程

北向速度分量引起运载体的纬度变化，东向速度分量则引起经度变化，如图 8. 1. 所示。由图得

$$
\dot {L} = \frac {V _ {N}}{R _ {M}}\tag{8.1.6a}
$$

$$
\dot {\lambda} = \frac {V _ {E}}{R _ {N} \cos L}\tag{8.1.6b}
$$

![](images/e7097ca1efb28336a84f6a8819bb715a2c604fe7f106924d3aa4addcbd614fcf.jpg)  
(a)

![](images/304a75026976bb6236faa8271ee04fc3506ba6c4fa2eaed950952bbd5e32f992.jpg)  
(b)  
8. 1. 速度引起的经、纬度变化

## 4. 高度计算

纯惯性高度通道是发散的，可用外来高度参考信息引人阻尼， 7. 节对此作了详细介绍，此处不再重复。

## 5. 指北方位系统的优缺点分析

## 1)优点

由于平台模拟当地的地理坐标系，所以航向角、俯仰角及横滚角可从平台环架轴上直接读取，各导航参数间的关系比较简单，导航解算方程简洁，计算量较小，对计算机要求较低。该系统在惯导系统发展初期计算机技术水平不高的年代是十分合适的选择方案。

## 2) 缺点

由式(8. 1. 3c)知，方位陀螺的指令角速度为 ${ \omega } _ { \mathrm { c m d } z } ^ { T } = { \omega } _ { i e } \mathrm { s i n } L + \frac { V _ { E } } { R _ { N } } \mathrm { t a n } L$ L，随着纬度 的增高，对方位陀螺的施矩电流急剧上升，在极区 $( L { \approx } 9 0 ^ { \circ } )$ 根本无法工作。又由式 (8. 1. 4a) 和式 (8. 1. 4b)知，在水平速度解算中有正切函数 tanL，当 $L { \approx } 9 0 ^ { \circ }$ 时速度中的计算误差被严重放大，甚至产生溢出。所以指北方位系统不能在高纬度地区正常工作，而只适用于中、低纬度地区的导航。

## 8. 1. 2 自由方位惯导系统的力学编排

## 1.自由方位角

指北方位惯导系统之所以不能在高纬度地区正常工作，原因是纬度很高时方位陀螺无法实现正常施矩，为了克服此缺陷，提出了对方位陀螺不施矩的编排方案，而对水平陀螺施矩控制平台始终保持水平，此即自由方位系统。由于对方位陀螺的施矩量为零，所以平台在方位上相对惯性空间稳定，即

$$
\omega_ {i T z} ^ {T} = 0\tag{8.1.7}
$$

而地球在旋转，运载体相对地球运动时地理坐标系相对地球也在旋转，所以平台的水平轴相对地理坐标系存在一个变化着的夹角，此夹角称为自由方位角 $\alpha _ { f } ( t )$ 如图 8. 1. 所示。图中，'1[1'为运载体的航向角，北偏东为正;' $\psi _ { \textit { n } }$ '布为航机角，亦称平台航向，在平台方位环上读取，顺时针为正 ${ \bf \rangle } \alpha _ { f } ( t )$ 为自由方位角，逆时针为正 $; \mathcal { X } _ { h }$ 和 $y _ { h }$ 分别为机体坐标系的$\mathcal { X } _ { b }$ (指向右)和 $y _ { b }$ (指向前)的水平投影。

由图得

$$
\Psi = \Psi_ {T b} - \alpha_ {f} (t)
$$

![](images/bfd000975a2734ca4995b507812283d05bf876ed9461748ffb81e545179e4649.jpg)

(8. 1. 8)

由于

$$
\boldsymbol {\omega} _ {i T} ^ {T} = \boldsymbol {\omega} _ {i e} ^ {T} + \boldsymbol {\omega} _ {e g} ^ {T} + \boldsymbol {\omega} _ {g T} ^ {T} = \mathbf {C} _ {g} ^ {T} (\boldsymbol {\omega} _ {i e} ^ {g} + \boldsymbol {\omega} _ {e g} ^ {g}) + \boldsymbol {\omega} _ {g T} ^ {T}
$$

8. 1. 航向角'[1'，航机角 $\Psi _ { \mathcal { I b } }$ 'Th和自由方位角

\~p

$$
\begin{array}{r l} \left[ \begin{array}{c} \omega_ {i T x} ^ {T} \\ \omega_ {i T y} ^ {T} \\ \omega_ {i T z} ^ {T} \end{array} \right] & = \left[ \begin{array}{c c c} \cos \alpha_ {f} & \sin \alpha_ {f} & 0 \\ - \sin \alpha_ {f} & \cos \alpha_ {f} & 0 \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c} - \frac {V _ {N}}{R _ {M}} \\ \omega_ {i e} \cos L + \frac {V _ {E}}{R _ {N}} \\ \omega_ {i e} \sin L + \frac {V _ {E}}{R _ {N}} \tan L \end{array} \right] + \left[ \begin{array}{c} 0 \\ 0 \\ \dot {\alpha} _ {f} (t) \end{array} \right] \\ & = \left[ \begin{array}{c} - \frac {V _ {N}}{R _ {M}} \cos \alpha_ {f} + (\omega_ {i e} \cos L + \frac {V _ {E}}{R _ {N}}) \sin \alpha_ {f} \\ \frac {V _ {N}}{R _ {M}} \sin \alpha_ {f} + (\omega_ {i e} \cos L + \frac {V _ {E}}{R _ {N}}) \cos \alpha_ {f} \\ \omega_ {i e} \sin L + \frac {V _ {E}}{R _ {N}} \tan L + \dot {\alpha} _ {f} (t) \end{array} \right] \end{array}
$$

由上式和式 (8. 1. 7) ，得

$$
\omega_ {i T z} ^ {T} = \omega_ {i e} \sin L + \frac {V _ {E}}{R _ {N}} \tan L + \dot {\alpha} _ {f} (t) = 0
$$

所以自由方位角服从如下方程:

$$
\dot {\alpha} _ {f} (t) = - \left(\omega_ {i e} \sin L + \frac {V _ {E}}{R _ {N}} \tan L\right)\tag{8.1.9}
$$

## 2. 方向余弦矩阵和定位计算

自由方位系统导航坐标系与地理坐标系只差自由方位角，如果按式(8. 1. 9)解算出自由方位角，并将加速度计获得的比力变换到地理坐标系中，则可按指北方位系统作导航解算，但由于指北方位系统还存在高纬度地区导航解算误差放大和溢出问题，所以必须采用其他方法作定位解算。

## 1)方向余弦阵 $C _ { e } ^ { T }$ 和经、纬度的关系

设运载体所在地 的经、纬度为 ，则 点的地理坐标系 $g$ 可由地球坐标系 $e$ 经兰次基本旋转后确定出，即

$$
x _ {e} y _ {e} z _ {e} \xrightarrow [ \text {旋转} \lambda ]{\text {绕} z _ {e} \text {轴}} x _ {e} ^ {\prime} y _ {e} ^ {\prime} z _ {e} ^ {\prime} \xrightarrow [ \text {旋转} 9 0 ^ {\circ} - L ]{\text {绕} y _ {e} ^ {\prime} \text {轴}} x _ {e} ^ {\prime \prime} y _ {e} ^ {\prime \prime} z _ {e} ^ {\prime \prime} \xrightarrow [ \text {旋转} 9 0 ^ {\circ} ]{\text {绕} z _ {e} ^ {\prime \prime} \text {轴}} x _ {g} y _ {g} z _ {g}
$$

如图 8. 1. 所示。由于 Zg 为垂线方向，所以 $z _ { g }$ $\boldsymbol { x } _ { e } ^ { \prime \prime }$ 和 $y _ { e } ^ { \prime \prime }$ 轴在水平面内， $\boldsymbol { x } _ { e } ^ { \prime \prime }$ 轴在经度为 的子午面内， $, x _ { e } ^ { \prime \prime }$ 又为水平轴，所以 $\boldsymbol { x } _ { e } ^ { \prime \prime }$ 轴指向南， $y _ { e } ^ { \prime \prime }$ 轴指向东，经 $9 0 ^ { \circ }$ 。旋转后， $\boldsymbol { x } _ { e } ^ { \prime \prime \prime }$ 指向东， $y _ { e } ^ { \prime \prime \prime }$ 指向北，因此$x _ { g } y _ { g } z _ { g }$ 即为 点的地理坐标系。

![](images/7d4328c9af6b6083256a8573910b3c0e231363db2f3dfa0dc0afce3c9d9551cb.jpg)  
8. 1. 由地球坐标系 到地理坐标系 $g$ 的三次旋转

由图 8. 1. 得坐标变换阵:

$$
\begin{array}{l} \boldsymbol {C} _ {e ^ {\prime}} = \left[ \begin{array}{c c c} \cos \lambda & \sin \lambda & 0 \\ - \sin \lambda & \cos \lambda & 0 \\ 0 & 0 & 1 \end{array} \right] \\ \boldsymbol {C} _ {e ^ {\prime}} ^ {e ^ {\prime \prime}} = \left[ \begin{array}{c c c} \cos (9 0 ^ {\circ} - L) & 0 & - \sin (9 0 ^ {\circ} - L) \\ 0 & 1 & 0 \\ \sin (9 0 ^ {\circ} - L) & 0 & \cos (9 0 ^ {\circ} - L) \end{array} \right] = \left[ \begin{array}{c c c} \sin L & 0 & - \cos L \\ 0 & 1 & 0 \\ \cos L & 0 & \sin L \end{array} \right] \end{array}
$$

$$
\boldsymbol {C} _ {e ^ {\prime}} ^ {g} = \boldsymbol {C} _ {e ^ {\prime \prime}} ^ {e ^ {\prime \prime \prime}} = \left[ \begin{array}{c c c} 0 & 1 & 0 \\ - 1 & 0 & 0 \\ 0 & 0 & 1 \end{array} \right]
$$

所以

$$
\begin{array}{r l} \boldsymbol {C} _ {e} ^ {\mathrm{g}} & = \boldsymbol {C} _ {e} ^ {\mathrm{g}} \boldsymbol {C} _ {e} ^ {e ^ {\prime \prime}} \boldsymbol {C} _ {e} ^ {e ^ {\prime}} = \left[ \begin{array}{c c c} 0 & 1 & 0 \\ - 1 & 0 & 0 \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c c c} \sin L & 0 & - \cos L \\ 0 & 1 & 0 \\ \cos L & 0 & \sin L \end{array} \right] \left[ \begin{array}{c c c} \cos \lambda & \sin \lambda & 0 \\ - \sin \lambda & \cos \lambda & 0 \\ 0 & 0 & 1 \end{array} \right] \\ & = \left[ \begin{array}{c c c} - \sin \lambda & \cos \lambda & 0 \\ - \sin L \cos \lambda & - \sin L \sin \lambda & \cos L \\ \cos L \cos \lambda & \cos L \sin \lambda & \sin L \end{array} \right] \end{array}\tag{8.1.10}
$$

设平台的自由方位角为 $\alpha _ { f }$ 则由图 8. 1. ，得

$$
\boldsymbol {C} _ {g} ^ {T} = \left[ \begin{array}{c c c} \cos \alpha_ {f} & \sin \alpha_ {f} & 0 \\ - \sin \alpha_ {f} & \cos \alpha_ {f} & 0 \\ 0 & 0 & 1 \end{array} \right]
$$

因此

$$
\begin{array}{r l} \boldsymbol {C} _ {e} ^ {T} & = \boldsymbol {C} _ {g} ^ {T} \boldsymbol {C} _ {e} ^ {g} = \left[ \begin{array}{c c c} \cos \alpha_ {f} & \sin \alpha_ {f} & 0 \\ - \sin \alpha_ {f} & \cos \alpha_ {f} & 0 \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c c c} - \sin \lambda & \cos \lambda & 0 \\ - \sin L \cos \lambda & - \sin L \sin \lambda & \cos L \\ \cos L \cos \lambda & \cos L \sin \lambda & \sin L \end{array} \right] \\ & = \left[ \begin{array}{c c c} - \cos \alpha_ {f} \sin \lambda - \sin \alpha_ {f} \sin L \cos \lambda & \cos \alpha_ {f} \cos \lambda - \sin \alpha_ {f} \sin L \sin \lambda & \sin \alpha_ {f} \cos L \\ \sin \alpha_ {f} \sin \lambda - \cos \alpha_ {f} \sin L \cos \lambda & - \sin \alpha_ {f} \cos \lambda - \cos \alpha_ {f} \sin L \sin \lambda & \cos \alpha_ {f} \cos L \\ \cos L \cos \lambda & \cos L \sin \lambda & \sin L \end{array} \right] \\ & = \left[ \begin{array}{c c c} C _ {1 1} & C _ {1 2} & C _ {1 3} \\ C _ {2 1} & C _ {2 2} & C _ {2 3} \\ C _ {3 1} & C _ {3 2} & C _ {3 3} \end{array} \right] \end{array}\tag{8.1.11}
$$

即有 $C _ { 1 3 } = \sin \alpha _ { f } \cos L , C _ { 2 3 } = \cos \alpha _ { f } \cos L , C _ { 3 3 } = \sin L , C _ { 3 1 } = \cos L \cos \lambda , C _ { 3 2 } = \cos L \sin \lambda$ 因此，若己求得 $C _ { e } ^ { T }$ ，

$$
L = \arcsin C _ {3 3}\tag{8.1.12a}
$$

$$
\lambda_ {\text {主}} = \arctan \frac {C _ {3 2}}{C _ {3 1}}\tag{8.1.12b}
$$

$$
\alpha_ {f \text {主}} = \arctan \frac {C _ {1 3}}{C _ {2 3}}\tag{8.1.12c}
$$

和 $\alpha _ { f }$ 的真值分别按表 8. 1. 和表 8. 1. 确定。获得 $\alpha _ { f }$ 后，根据式(8. 1. 8)可确定出航向角? $\boldsymbol { \Psi } _ { \mathrm { ~ c ~ } }$

8.1.1 的真值确定  
8.1.2 ${ \pmb { \alpha } } _ { f }$ 的真值确定

<table><tr><td> $C_{31}$ \λ主</td><td>-</td><td>+</td></tr><tr><td>-</td><td> $λ = \lambda_{主} + 180^{\circ}$ </td><td> $λ = \lambda_{主} - 180^{\circ}$ </td></tr><tr><td>+</td><td colspan="2"> $λ = \lambda_{主}$ </td></tr></table>

<table><tr><td> $C_{23}^{\alpha_f主}$ </td><td>+</td><td>-</td></tr><tr><td>+</td><td> $\alpha=\alpha_f主$ </td><td> $\alpha_f=\alpha_f主+360^\circ$ </td></tr><tr><td>-</td><td colspan="2"> $\alpha_f=\alpha_f主+180^\circ$ </td></tr></table>

2) 方向余弦阵微分方程及其求解

设自地心至平台支点 弓|的位置矢量为 平台坐标系 相对地球坐标系 的旋转角速度为( ${ \pmb { \omega } } _ { e T }$ 。则

$$
\boldsymbol {R} ^ {e} = \boldsymbol {C} _ {T} ^ {e} \boldsymbol {R} ^ {T}
$$

平台支点 相对地球的速度为

$$
\boldsymbol {V} ^ {e} = \left. \frac {\mathrm{d} \boldsymbol {R}}{\mathrm{d} t} \right| _ {e} ^ {e} = \dot {\boldsymbol {R}} ^ {e} = \dot {\boldsymbol {C}} _ {T} ^ {e} \boldsymbol {R} ^ {T} + \boldsymbol {C} _ {T} ^ {e} \dot {\boldsymbol {R}} ^ {T}
$$

由于 处于垂线位置，平台始终保持水平，所以 始终与平台的方位轴重合 在平台坐标系内保持不变，即 $\dot { \pmb R } ^ { T } = 0$ ，因此

$$
\mathbf {V} ^ {e} = \dot {\mathbf {C}} _ {T} ^ {e} \mathbf {C} _ {e} ^ {T} \mathbf {R} ^ {e}\tag{8.1.13}
$$

又是 的矢端速度

$$
\mathbf {V} = \boldsymbol {\omega} _ {e T} \times \mathbf {R}
$$

上式分别写成地球坐标系 和平台坐标系 内的数学向量形式:

$$
\mathbf {V} ^ {e} = \pmb {\omega} _ {e T} ^ {e} \times \mathbf {R} ^ {e}\tag{8.1.14}
$$

$$
\boldsymbol {V} ^ {T} = \boldsymbol {\omega} _ {e T} ^ {T} \times \boldsymbol {R} ^ {T} = (\boldsymbol {\omega} _ {e T} ^ {T} \times) \boldsymbol {C} _ {e} ^ {T} \boldsymbol {R} ^ {e}\tag{8.1.15}
$$

比较式 (8. 1. 13) 和式 (8. 1. 14) ，得

$$
\dot {\mathbf {C}} _ {T} ^ {e} \mathbf {C} _ {e} ^ {T} = (\boldsymbol {\omega} _ {e T} ^ {e} \times)\tag{8.1.16}
$$

在式 (8. 1. 15) 和式 (8. 1. 16) 中， $( { \pmb { \omega } } _ { e T } ^ { c } \times )$ 是由 ${ \pmb \omega } _ { e T } ^ { c }$ 构造出的叉乘反对称矩阵，即

$$
(\boldsymbol {\omega} _ {e T} ^ {c} \times) = \left[ \begin{array}{c c c} 0 & - \boldsymbol {\omega} _ {e T z} ^ {c c c} & \boldsymbol {\omega} _ {e T y} ^ {c c c} \\ \boldsymbol {\omega} _ {e T z} ^ {c c c} & 0 & - \boldsymbol {\omega} _ {e T x} ^ {c c c} \\ - \boldsymbol {\omega} _ {e T y} ^ {c c c} & \boldsymbol {\omega} _ {e T x} ^ {c c c} & 0 \end{array} \right] \quad c = e, T
$$

又由式 (8. 1. 14) ，得

$$
\boldsymbol {V} ^ {T} = \boldsymbol {C} _ {e} ^ {T} \boldsymbol {V} ^ {e} = \boldsymbol {C} _ {e} ^ {T} (\boldsymbol {\omega} _ {e T} ^ {e} \times) \boldsymbol {R} ^ {e}\tag{8.1.17}
$$

比较式 (8. 1. 15) 和式 (8. 1. 17) ，得

$$
(\pmb {\omega} _ {e T} ^ {T} \times) \pmb {C} _ {e} ^ {T} = \pmb {C} _ {e} ^ {T} (\pmb {\omega} _ {e T} ^ {e} \times)\tag{8.1.18}
$$

将式 (8. 1. 16) 代入上式，得

$$
(\boldsymbol {\omega} _ {e T} ^ {T} \times) \mathbf {C} _ {e} ^ {T} = \mathbf {C} _ {e} ^ {T} \dot {\mathbf {C}} _ {T} ^ {e} \mathbf {C} _ {e} ^ {T}
$$

即

$$
\dot {\pmb {C}} _ {T} ^ {e} = \pmb {C} _ {T} ^ {e} (\pmb {\omega} _ {e T} ^ {T} \times)\tag{8.1.19}
$$

由于 ${ \cal C } _ { e } ^ { T } { \cal C } _ { T } ^ { e } { = } I$ ，所以

$$
\begin{array}{c} \dot {\boldsymbol {C}} _ {e} ^ {T} \boldsymbol {C} _ {T} ^ {e} + \boldsymbol {C} _ {e} ^ {T} \dot {\boldsymbol {C}} _ {T} ^ {e} = \boldsymbol {0} \\ \dot {\boldsymbol {C}} _ {T} ^ {e} = - \boldsymbol {C} _ {T} ^ {e} \dot {\boldsymbol {C}} _ {e} ^ {T} \boldsymbol {C} _ {T} ^ {e} \end{array}
$$

上式代入式 (8. 1. 19) ，得

$$
- \boldsymbol {C} _ {T} ^ {e} \dot {\boldsymbol {C}} _ {e} ^ {T} \boldsymbol {C} _ {T} ^ {e} = \boldsymbol {C} _ {T} ^ {e} (\boldsymbol {\omega} _ {e T} ^ {T} \times)
$$

即

$$
\dot {\mathbf {C}} _ {e} ^ {T} = - (\pmb {\omega} _ {e T} ^ {T} \times) \mathbf {C} _ {e} ^ {T}\tag{8.1.20}
$$

(8. 1. 19) 和式 (8. 1. 20)是方向余弦矩阵微分方程的两种表达式，此处采用式 (8. 1. 20)算方向余弦矩阵

$$
\left[ \begin{array}{c c c} \dot {C} _ {1 1} & \dot {C} _ {1 2} & \dot {C} _ {1 3} \\ \dot {C} _ {2 1} & \dot {C} _ {2 2} & \dot {C} _ {2 3} \\ \dot {C} _ {3 1} & \dot {C} _ {3 2} & \dot {C} _ {3 3} \end{array} \right] = \left[ \begin{array}{c c c} 0 & \boldsymbol {\omega} _ {e T z} ^ {T} & - \boldsymbol {\omega} _ {e T y} ^ {T} \\ - \boldsymbol {\omega} _ {e T z} ^ {T} & 0 & \boldsymbol {\omega} _ {e T x} ^ {T} \\ \boldsymbol {\omega} _ {e T y} ^ {T} & - \boldsymbol {\omega} _ {e T x} ^ {T} & 0 \end{array} \right] \left[ \begin{array}{c c c} C _ {1 1} & C _ {1 2} & C _ {1 3} \\ C _ {2 1} & C _ {2 2} & C _ {2 3} \\ C _ {3 1} & C _ {3 2} & C _ {3 3} \end{array} \right]
$$

由于 $C _ { e } ^ { T }$ 是单位正交矩阵，所以求解上述方程可删去任意一列，而仅需解 个微分方程，再根据单位正交矩阵的逆与转置相等，利用 个代数余子约束方程，即可确定出 $C _ { e } ^ { T }$ 。若删去第一列，则微分方程和代数方程为

$$
\dot {C} _ {1 2} = \omega_ {e T z} ^ {T} C _ {2 2} - \omega_ {e T y} ^ {T} C _ {3 2}\tag{8.1.21a}
$$

$$
\dot {C} _ {1 3} = \omega_ {e T z} ^ {T} C _ {2 3} - \omega_ {e T y} ^ {T} C _ {3 3}\tag{8.1.21b}
$$

$$
\dot {C} _ {2 2} = - \omega_ {e T z} ^ {T} C _ {1 2} + \omega_ {e T x} ^ {T} C _ {3 2}\tag{8.1.21c}
$$

$$
\dot {C} _ {2 3} = - \omega_ {e T z} ^ {T} C _ {1 3} + \omega_ {e T x} ^ {T} C _ {3 3}\tag{8.1.21d}
$$

$$
\dot {C} _ {3 2} = \omega_ {e T y} ^ {T} C _ {1 2} - \omega_ {e T x} ^ {T} C _ {2 2}\tag{8.1.21e}
$$

$$
\dot {C} _ {3 3} = \omega_ {e T y} ^ {T} C _ {1 3} - \omega_ {e T x} ^ {T} C _ {2 3}\tag{8.1.21f}
$$

$$
C _ {1 1} = C _ {2 2} C _ {3 3} - C _ {2 3} C _ {3 2}\tag{8.1.22a}
$$

$$
C _ {2 1} = C _ {1 2} C _ {3 3} - C _ {1 3} C _ {3 2}\tag{8.1.22b}
$$

$$
C _ {3 1} = C _ {1 2} C _ {2 3} - C _ {2 2} C _ {1 3}\tag{8.1.22c}
$$

3) 位置速率 ${ \pmb { \omega } } _ { e T } ^ { T }$ 的确定

由于

$$
\boldsymbol {\omega} _ {i T} ^ {T} = \boldsymbol {\omega} _ {i e} ^ {T} + \boldsymbol {\omega} _ {e T} ^ {T} = \boldsymbol {C} _ {e} ^ {T} \boldsymbol {\omega} _ {i e} ^ {e} + \boldsymbol {\omega} _ {e T} ^ {T}
$$

即

$$
\left[ \begin{array}{l} \boldsymbol {\omega} _ {i T x} ^ {T} \\ \boldsymbol {\omega} _ {i T y} ^ {T} \\ \boldsymbol {\omega} _ {i T z} ^ {T} \end{array} \right] = \left[ \begin{array}{c c c} C _ {1 1} & C _ {1 2} & C _ {1 3} \\ C _ {2 1} & C _ {2 2} & C _ {2 3} \\ C _ {3 1} & C _ {3 2} & C _ {3 3} \end{array} \right] \left[ \begin{array}{l} 0 \\ 0 \\ \boldsymbol {\omega} _ {i e} \end{array} \right] + \left[ \begin{array}{l} \boldsymbol {\omega} _ {e T x} ^ {T} \\ \boldsymbol {\omega} _ {e T y} ^ {T} \\ \boldsymbol {\omega} _ {e T z} ^ {T} \end{array} \right] = \left[ \begin{array}{l} C _ {1 3} \boldsymbol {\omega} _ {i e} + \boldsymbol {\omega} _ {e T x} ^ {T} \\ C _ {2 3} \boldsymbol {\omega} _ {i e} + \boldsymbol {\omega} _ {e T y} ^ {T} \\ C _ {3 3} \boldsymbol {\omega} _ {i e} + \boldsymbol {\omega} _ {e T z} ^ {T} \end{array} \right]\tag{8.1.23}
$$

对于自由方位系统， $\omega _ { i T z } ^ { T } = 0$ ，所以

$$
\omega_ {e T z} ^ {T} = - C _ {3 3} \omega_ {i e}\tag{8.1.24}
$$

下面再确定 $\omega _ { e T x } ^ { T }$ 和 $\omega _ { e T y } ^ { T }$ e 由于

$$
\boldsymbol {\omega} _ {e T} ^ {T} = \boldsymbol {\omega} _ {e g} ^ {T} + \boldsymbol {\omega} _ {g T} ^ {T} = \boldsymbol {C} _ {g} ^ {T} \boldsymbol {\omega} _ {e g} ^ {g} + \boldsymbol {\omega} _ {g T} ^ {T}
$$

\~p

$$
\left[ \begin{array}{c} \boldsymbol {\omega} _ {e T x} ^ {T} \\ \boldsymbol {\omega} _ {e T y} ^ {T} \\ \boldsymbol {\omega} _ {e T z} ^ {T} \end{array} \right] = \left[ \begin{array}{c c c} \cos \alpha_ {f} & \sin \alpha_ {f} & 0 \\ - \sin \alpha_ {f} & \cos \alpha_ {f} & 0 \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c} \boldsymbol {\omega} _ {e g, x} ^ {g} \\ \boldsymbol {\omega} _ {e g y} ^ {g} \\ \boldsymbol {\omega} _ {e g z} ^ {g} \end{array} \right] + \left[ \begin{array}{c} 0 \\ 0 \\ \dot {\alpha} _ {f} \end{array} \right]
$$

$$
\left[ \begin{array}{c} \boldsymbol {\omega} _ {e T x} ^ {T} \\ \boldsymbol {\omega} _ {e T y} ^ {T} \end{array} \right] = \left[ \begin{array}{c c} \cos \alpha_ {f} & \sin \alpha_ {f} \\ - \sin \alpha_ {f} & \cos \alpha_ {f} \end{array} \right] \left[ \begin{array}{c} \boldsymbol {\omega} _ {e g, x} ^ {g} \\ \boldsymbol {\omega} _ {e g, y} ^ {g} \end{array} \right]\tag{8.1.25}
$$

式中

$$
\left[ \begin{array}{c} \omega_ {e g, x} ^ {g} \\ \omega_ {e g, y} ^ {g} \end{array} \right] = \left[ \begin{array}{c} - \frac {V _ {N}}{R _ {M}} \\ \frac {V _ {E}}{R _ {N}} \end{array} \right] = \left[ \begin{array}{c c} - \frac {1}{R _ {M}} & 0 \\ 0 & \frac {1}{R _ {N}} \end{array} \right] \left[ \begin{array}{c} V _ {N} \\ V _ {E} \end{array} \right]\tag{8.1.26}
$$

而 $\mathbf { V } ^ { g } = \pmb { C } _ { T } ^ { g } \mathbf { V } ^ { T }$ ,即

$$
\left[ \begin{array}{c} V _ {E} \\ V _ {N} \\ V _ {U} \end{array} \right] = \left[ \begin{array}{c c c} \cos \alpha_ {f} & - \sin \alpha_ {f} & 0 \\ \sin \alpha_ {f} & \cos \alpha_ {f} & 0 \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c} V _ {x} ^ {T} \\ V _ {y} ^ {T} \\ V _ {z} ^ {T} \end{array} \right] = \left[ \begin{array}{c} V _ {x} ^ {T} \cos \alpha_ {f} - V _ {y} ^ {T} \sin \alpha_ {f} \\ V _ {x} ^ {T} \sin \alpha_ {f} + V _ {y} ^ {T} \cos \alpha_ {f} \\ V _ {z} ^ {T} \end{array} \right]
$$

$$
\left[ \begin{array}{c} V _ {N} \\ V _ {E} \end{array} \right] = \left[ \begin{array}{c c} \sin \alpha_ {f} & \cos \alpha_ {f} \\ \cos \alpha_ {f} & - \sin \alpha_ {f} \end{array} \right] \left[ \begin{array}{c} V _ {x} ^ {T} \\ V _ {y} ^ {T} \end{array} \right]\tag{8.1.27}
$$

将式 (8. 1. 27) 代入式 (8. 1. 26)后再代入式 (8. 1. 25).

$$
\begin{array}{r l} \left[ \begin{array}{c} \omega_ {e T x} ^ {T} \\ \omega_ {e T y} ^ {T} \end{array} \right] & = \left[ \begin{array}{c c} \cos \alpha_ {f} & \sin \alpha_ {f} \\ - \sin \alpha_ {f} & \cos \alpha_ {f} \end{array} \right] \left[ \begin{array}{c c} - \frac {1}{R _ {M}} & 0 \\ 0 & \frac {1}{R _ {N}} \end{array} \right] \left[ \begin{array}{c c} \sin \alpha_ {f} & \cos \alpha_ {f} \\ \cos \alpha_ {f} & - \sin \alpha_ {f} \end{array} \right] \left[ \begin{array}{c} V _ {x} ^ {T} \\ V _ {y} ^ {T} \end{array} \right] \\ & = \left[ \begin{array}{c c} - \frac {1}{\tau_ {f}} & - \frac {1}{R _ {y T}} \\ \frac {1}{R _ {x T}} & \frac {1}{\tau_ {f}} \end{array} \right] \left[ \begin{array}{c} V _ {x} ^ {T} \\ V _ {y} ^ {T} \end{array} \right] \end{array}\tag{8.1.28}
$$

式中

$$
\frac {1}{R _ {x T}} = \frac {\sin^ {2} \alpha_ {f}}{R _ {M}} + \frac {\cos^ {2} \alpha_ {f}}{R _ {N}}\tag{8.1.29a}
$$

$$
\frac {1}{R _ {y T}} = \frac {\cos^ {2} \alpha_ {f}}{R _ {M}} + \frac {\sin^ {2} \alpha_ {f}}{R _ {N}}\tag{8.1.29b}
$$

$$
\frac {1}{\tau_ {f}} = \left(\frac {1}{R _ {M}} - \frac {1}{R _ {N}}\right) \sin \alpha_ {f} \cos \alpha_ {f}\tag{8.1.29c}
$$

$R _ { x T }$ 和 $R _ { \mathit { y T } }$ 称为自由方位等效曲率半径 $\cdot \frac { 1 } { \tau _ { f } }$ 称为扭曲率 $\frac { 1 } { R _ { x T } }$ 和 $\frac { 1 } { R _ { s T } }$ 即为地球沿平台轴 $x _ { T }$ 和 $y _ { T }$ 方向的曲率.

为了用 $C _ { e } ^ { T }$ 的元素表示式 (8. 1. 28). 需作如下恒等变换处理。由式 (8. 1. 11).

$$
\sin_ {\alpha_ {f}} \cos L = C _ {1 3}, \quad \cos_ {\alpha_ {f}} \cos L = C _ {2 3}, \quad \sin L = C _ {3 3}
$$

所以

$$
\sin^ {2} \alpha_ {f} = \frac {C _ {1 3} ^ {2}}{C _ {1 3} ^ {2} + C _ {2 3} ^ {2}}\tag{8.1.30a}
$$

$$
\cos^ {2} \alpha_ {f} = \frac {C _ {2 3} ^ {2}}{C _ {1 3} ^ {2} + C _ {2 3} ^ {2}}\tag{8.1.30b}
$$

$$
\sin_ {\alpha_ {f}} \cos_ {\alpha_ {f}} = \frac {C _ {1 3} C _ {2 3}}{C _ {1 3} ^ {2} + C _ {2 3} ^ {2}}\tag{8.1.30c}
$$

(7.2.9) 和式 (7.2.10)可写成

$$
\frac {1}{R _ {N}} = \frac {1}{R _ {e}} (1 - f C _ {3 3} ^ {2})\tag{8.1.30d}
$$

$$
\frac {1}{R _ {M}} = \frac {1}{R _ {e}} (1 + 2 f - 3 f C _ {3 3} ^ {2})\tag{8.1.30e}
$$

将式(8.1.30)代入式(8.1.29)，整理后得

$$
\frac {1}{R _ {x T}} = \frac {1}{R _ {e}} (1 - f C _ {3 3} ^ {2} + 2 f C _ {1 3} ^ {2})\tag{8.1.31a}
$$

$$
\frac {1}{R _ {y T}} = \frac {1}{R _ {e}} (1 - f C _ {3 3} ^ {2} + 2 f C _ {2 3} ^ {2})\tag{8.1.31b}
$$

$$
\frac {1}{\tau_ {f}} = \frac {2 f}{R _ {e}} C _ {1 3} C _ {2 3}\tag{8.1.31c}
$$

将式 (8. 1. 31) 代入式 (8. 1. 28) ，得

$$
\omega_ {e T x} ^ {T} = - \frac {2 f}{R _ {e}} C _ {1 3} C _ {2 3} V _ {x} ^ {T} - \frac {1}{R _ {e}} (1 - f C _ {3 3} ^ {2} + 2 f C _ {2 3} ^ {2}) V _ {y} ^ {T}\tag{8.1.32a}
$$

$$
\omega_ {e F y} ^ {T} = \frac {1}{R _ {f}} (1 - f C _ {3 3} ^ {2} + 2 f C _ {1 3} ^ {2}) V _ {x} ^ {T} + \frac {2 f}{R _ {e}} C _ {1 3} C _ {2 3} V _ {y} ^ {T}\tag{8.1.32b}
$$

## 3. 速度方程

根据式(7.4. 7) ，自由方位系统的比力方程为

$$
\dot {\boldsymbol {V}} _ {e T} ^ {T} = \boldsymbol {f} ^ {T} - \left(2 \boldsymbol {C} _ {e} ^ {T} \boldsymbol {\omega} _ {i e} ^ {e} + \boldsymbol {\omega} _ {e T} ^ {T}\right) \times \boldsymbol {V} _ {e T} ^ {T} + \boldsymbol {g} ^ {T}
$$

将上式写成分量形式，并考虑到式(8. 1. 24)

$$
\left[ \begin{array}{l} \dot {V} _ {x} ^ {T} \\ \dot {V} _ {y} ^ {T} \\ \dot {V} _ {z} ^ {T} \end{array} \right] = \left[ \begin{array}{l} f _ {x} ^ {T} \\ f _ {y} ^ {T} \\ f _ {z} ^ {T} \end{array} \right] - C _ {w} \left(2 \left[ \begin{array}{c c c} C _ {1 1} & C _ {1 2} & C _ {1 3} \\ C _ {2 1} & C _ {2 2} & C _ {2 3} \\ C _ {3 1} & C _ {3 2} & C _ {3 3} \end{array} \right] \left[ \begin{array}{l} 0 \\ 0 \\ \omega_ {i e} \end{array} \right] + \left[ \begin{array}{c} \boldsymbol {\omega} _ {e T x} ^ {T} \\ \boldsymbol {\omega} _ {e T y} ^ {T} \\ - C _ {3 3} \boldsymbol {\omega} _ {i e} \end{array} \right]\right) \cdot \left[ \begin{array}{l} V _ {x} ^ {T} \\ V _ {y} ^ {T} \\ V _ {z} ^ {T} \end{array} \right] + \left[ \begin{array}{l} 0 \\ 0 \\ - g \end{array} \right]
$$

$$
\begin{array}{l} = \left[ \begin{array}{c} f _ {x} ^ {T} \\ f _ {y} ^ {T} \\ f _ {z} ^ {T} \end{array} \right] - C _ {w} \left(\left[ \begin{array}{c} 2 \omega_ {i e} C _ {1 3} + \omega_ {e T x} ^ {T} \\ 2 \omega_ {i e} C _ {2 3} + \omega_ {e T y} ^ {T} \\ \omega_ {i e} C _ {3 3} \end{array} \right]\right) \cdot \left[ \begin{array}{c} V _ {x} ^ {T} \\ V _ {y} ^ {T} \\ V _ {z} ^ {T} \end{array} \right] + \left[ \begin{array}{c} 0 \\ 0 \\ - g \end{array} \right] \\ = \left[ \begin{array}{c} f _ {x} ^ {T} \\ f _ {y} ^ {T} \\ f _ {z} ^ {T} \end{array} \right] - \left[ \begin{array}{c c c} 0 & - \omega_ {i e} C _ {3 3} & 2 \omega_ {i e} C _ {2 3} + \omega_ {e T y} ^ {T} \\ \omega_ {i e} C _ {3 3} & 0 & - (2 \omega_ {i e} C _ {1 3} + \omega_ {e T x} ^ {T}) \\ - (2 \omega_ {i e} C _ {2 3} + \omega_ {e T y} ^ {T}) & 2 \omega_ {i e} C _ {1 3} + \omega_ {e T x} ^ {T} & 0 \end{array} \right] \left[ \begin{array}{c} V _ {x} ^ {T} \\ V _ {y} ^ {T} \\ V _ {z} ^ {T} \end{array} \right] + \left[ \begin{array}{c} 0 \\ 0 \\ - g \end{array} \right] \end{array}
$$

上式中，c" $C _ { w } ( \mathbf { \sigma } \cdot \mathbf { \sigma } )$ )表示用括号中的矢量构造叉乘斜对称矩阵。对于飞机和舰船，垂直速度远比水平速度小，当忽略其影响时，得水平速度方程

$$
\dot {V} _ {x} ^ {T} = f _ {x} ^ {T} + \omega_ {i e} C _ {3 3} V _ {y} ^ {T}\tag{8.1.33a}
$$

$$
\dot {V} _ {y} ^ {T} = f _ {y} ^ {T} - \omega_ {i e} C _ {3 3} V _ {x} ^ {T}\tag{8.1.33b}
$$

联立求解式 (8. 1. 21) 和式 (8. 1. 纱，并根据式 (8. 1. 纱，即可求解出 $C _ { e } ^ { T }$ 和 $V _ { x } ^ { T }$ 及 $V _ { y } ^ { T }$ ，完成导航计算。

## 4. 平台的指令角速度

根据 ${ \pmb { \omega } } _ { i T } ^ { T } { = } { \pmb { C } } _ { e } ^ { T } { \pmb { \omega } } _ { i e } ^ { e } + { \pmb { \omega } } _ { e T } ^ { T }$ 岳，得平台的指令角速度为

$$
\omega_ {\mathrm{cmd} x} ^ {T} = C _ {1 3} \omega_ {i e} + \omega_ {e T x} ^ {T}\tag{8.1.34a}
$$

$$
\omega_ {\mathrm{cmdy}} ^ {T} = C _ {2 3} \omega_ {i e} + \omega_ {e T y} ^ {T}\tag{8.1.34b}
$$

$$
\omega_ {\mathrm{cmd} z} ^ {T} = 0\tag{8.1.34c}
$$

其中， $, \omega _ { e T x } ^ { T }$ 和 $\omega _ { e T y } ^ { T }$ 由式 (8. 1. 32) 确定。

## 5. 自由方位系统的工作原理简圈

根据式 (8. 1. 33) 、式 (8. 1. 28) 、式 (8. 1. 1)、式 (8. 1. 22) 、式 (8. 1. 34) 、式 (8. 1. 12)8. 1. 、表 8. 1. ，可画出自由方位惯导系统的工作原理简图，如图 8. 1. 所示.

自由方位系统虽然避免了在高纬度地区对方位陀螺施矩的困难，但由式(8. 1. 12)知在极区仍存在导航计算溢出的问题，解决的办法是采用适用于极区导航的力学编排方案 极区平台导航、极区横向经纬度导航、极区格网导航。这些内容将在第 10 章中详细介绍。

![](images/b513acfdef63ffd95bf72792c5e42181629792645a9cb443db9b6e0fa5a35275.jpg)  
1. 自由方位惯导系统的工作原理简图

## 8. 1. 3 游移方位惯导系统的力学编排

## 1.游移方位角

游移方位惯导系统的导航坐标系仍然是地平坐标系，方位跟踪地球旋转，即方位陀螺的指令角速度为

$$
\omega_ {\mathrm{cmd} z} ^ {T} = \omega_ {i T z} ^ {T} = \omega_ {i e} \sin L\tag{8.1.35}
$$

![](images/c425d39ea48d1153bc5b26c05e57d7f56fca5fc87ce10d3c015ca3cd8b75bddd.jpg)  
8. 1. 航向角 ，平 航向  
角 $\psi _ { \textit { T b } }$ 和游移方位角 (1 (t)

与自 由方位系统类似，平台的水平轴 $x _ { T }$ 和 $y _ { T }$ 对东向轴和北向轴存在偏转角 ，此偏转角称为游移方位角，逆时针为正，如图 8. 1. 所示。

由图

$$
\Psi = \Psi_ {T b} - \alpha\tag{8.1.36}
$$

式中 平台航向角 $\Psi _ { \mathit { T b } }$ 平台环架轴上读取，}I顶时针为正。由于 ${ \pmb { \omega } } _ { i T } ^ { T } { = } { \pmb { C } } _ { g } ^ { T } { \pmb { \omega } } _ { i g } ^ { g } + { \pmb { \omega } } _ { g T } ^ { T }$ ，

$$
\left[ \begin{array}{c} \boldsymbol {\omega} _ {i T x} ^ {T} \\ \boldsymbol {\omega} _ {i T y} ^ {T} \\ \boldsymbol {\omega} _ {i T z} ^ {T} \end{array} \right] = \left[ \begin{array}{c c c} \cos \alpha & \sin \alpha & 0 \\ - \sin \alpha & \cos \alpha & 0 \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c} \boldsymbol {\omega} _ {i g, x} ^ {g} \\ \boldsymbol {\omega} _ {i g, y} ^ {g} \\ \boldsymbol {\omega} _ {i g, z} ^ {g} \end{array} \right] + \left[ \begin{array}{c} 0 \\ 0 \\ \dot {\alpha} (t) \end{array} \right]
$$

所以由上式

$$
\omega_ {i T z} ^ {T} = \omega_ {i g z} ^ {g} + \dot {\alpha} (t) = \omega_ {i e} \sin L + \frac {V _ {E}}{R _ {N}} \tan L + \dot {\alpha} (t)
$$

将式(8. 1. 35)代入上式，得游移方位角的变化规律

$$
\dot {\alpha} (t) = - \frac {V _ {E}}{R _ {N}} \tan L\tag{8.1.37}
$$

上式说明，当运载体向北运动或静止时，游移方位角保持不变，除在赤道上之外，只要有东向速度分量，游移方位角就是变化的。

## 2. 方向余弦矩阵和定位计算

## 1)方向余弦矩阵和定位计算间的关系

游移方位系统与自由方位系统的区别在于平台水平轴相对地理坐标系水平轴的偏转角，所以只需将式(8. 1.11)中的自由方位角 $\alpha _ { f }$ 换成游移方位角 ，就可得到游移方位系统的方向余弦矩阵

$$
\begin{array}{r l} \boldsymbol {C} _ {e} ^ {T} & = \left[ \begin{array}{c c c} C _ {1 1} & C _ {1 2} & C _ {1 3} \\ C _ {2 1} & C _ {2 2} & C _ {2 3} \\ C _ {3 1} & C _ {3 2} & C _ {3 3} \end{array} \right] \\ & = \left[ \begin{array}{c c c} - \cos \alpha \sin \lambda - \sin \alpha \sin L \cos \lambda & \cos \alpha \cos \lambda - \sin \alpha \sin L \sin \lambda & \sin \alpha \cos L \\ \sin \alpha \sin \lambda - \cos \alpha \sin L \cos \lambda & - \sin \alpha \cos \lambda - \cos \alpha \sin L \sin \lambda & \cos \alpha \cos L \\ \cos L \cos \lambda & \cos L \sin \lambda & \sin L \end{array} \right] \end{array}\tag{8.1.38}
$$

由上式得

$$
L = \arcsin C _ {3 3}\tag{8.1.39a}
$$

$$
\lambda_ {\text {主}} = \arctan \frac {C _ {3 2}}{C _ {3 1}}\tag{8.1.39b}
$$

$$
\alpha_ {\text {主}} = \arctan \frac {C _ {1 3}}{C _ {2 3}}\tag{8.1.39c}
$$

其中 的真值确定同表 8. 1. 的真值确定按表 8. 1. 3.

8.1.3 游移方位角的真值确定

## 2) 方向余弦阵 $\mathbf { C } _ { e } ^ { T }$ 的确定

由于 ${ \pmb { \omega } } _ { e T } ^ { T } { = } { \pmb { \omega } } _ { i T } ^ { T } { - } { \pmb { \omega } } _ { i e } ^ { T }$ ，所以根据式 (8. 1. 35) ,

$$
\omega_ {e T z} ^ {T} = \omega_ {i T z} ^ {T} - \omega_ {i e z} ^ {T} = \omega_ {i e} \sin L - \omega_ {i e} \sin L = 0
$$

记 $\pmb { \omega } _ { e T } ^ { T } = [ \pmb { \omega } _ { e T x } ^ { T } \quad \pmb { \omega } _ { e T y } ^ { T } \quad 0 ] ^ { \mathrm { T } }$ ，代入式 (8. 1. 20) ，得

<table><tr><td> $C_{23}$ \ $\alpha$ 主</td><td>+</td><td>-</td></tr><tr><td>+</td><td> $\alpha=\alpha$ 主</td><td> $\alpha=\alpha$ 主+ $360^{\circ}$ </td></tr><tr><td>-</td><td colspan="2"> $\alpha=\alpha$ 主+ $180^{\circ}$ </td></tr></table>

$$
\dot {C} _ {1 2} = - \omega_ {e T y} ^ {T} C _ {3 2} \tag {8.1.40a}
$$

$$
\dot {C} _ {1 3} = - \omega_ {e T y} ^ {T} C _ {3 3}\tag{8.1.40b}
$$

$$
\dot {C} _ {2 2} = \omega_ {e T x} ^ {T} C _ {3 2}\tag{8.1.40c}
$$

$$
\dot {C} _ {2 3} = \omega_ {e T x} ^ {T} C _ {3 3}\tag{8.1.40d}
$$

$$
\dot {C} _ {3 2} = \omega_ {e T y} ^ {T} C _ {1 2} - \omega_ {e T x} ^ {T} C _ {2 2}\tag{8.1.40e}
$$

$$
\dot {C} _ {3 3} = \omega_ {e T y} ^ {T} C _ {1 3} - \omega_ {e T x} ^ {T} C _ {2 3}\tag{8.1.40f}
$$

$$
C _ {1 1} = C _ {2 2} C _ {3 3} - C _ {2 3} C _ {3 2}\tag{8.1.41a}
$$

$$
C _ {2 1} = C _ {1 2} C _ {3 3} - C _ {1 3} C _ {3 2}\tag{8.1.41b}
$$

$$
C _ {3 1} = C _ {1 2} C _ {2 3} - C _ {2 2} C _ {1 3}\tag{8.1.41c}
$$

## 3) 位置速率 ${ \pmb { \omega } } _ { e T } ^ { T }$ 的确定

仿照自由方位系统位置速率的推导，用游移方位角 $\alpha$ 代替自由方位角 $\alpha _ { f }$ ，得游移方位系统的位置速率

$$
\omega_ {e T x} ^ {T} = - \frac {2 f}{R _ {e}} C _ {1 3} C _ {2 3} V _ {x} ^ {T} - \frac {1}{R _ {e}} (1 - f C _ {3 3} ^ {2} + 2 f C _ {2 3} ^ {2}) V _ {y} ^ {T}\tag{8.1.42a}
$$

$$
\omega_ {e T y} ^ {T} = \frac {1}{R _ {e}} (1 - f C _ {3 3} ^ {2} + 2 f C _ {1 3} ^ {2}) V _ {x} ^ {T} + \frac {2 f}{R _ {e}} C _ {1 3} C _ {2 3} V _ {y} ^ {T}\tag{8.1.42b}
$$

同样可得地球沿 台水平轴方向的曲率及扭曲率:

$$
\frac {1}{R _ {x T}} = \frac {1}{R _ {e}} (1 - f C _ {3 3} ^ {2} + 2 f C _ {1 3} ^ {2})\tag{8.1.43a}
$$

$$
\frac {1}{R _ {y T}} = \frac {1}{R _ {e}} (1 - f C _ {3 3} ^ {2} + 2 f C _ {2 3} ^ {2})\tag{8.1.43b}
$$

$$
\frac {1}{\tau} = \frac {2 f}{R _ {e}} C _ {1 3} C _ {2 3}\tag{8.1.43c}
$$

## 3. 速度方程

仿照式(8. 1. 33)的推导，并注意到 $\omega _ { e T z } ^ { T } = 0$ ，可得游移方位系统的水平速度方程:

$$
\dot {V} _ {x} ^ {T} = f _ {x} ^ {T} + 2 \omega_ {i e} C _ {3 3} V _ {y} ^ {T}\tag{8.1.44a}
$$

$$
\dot {V} _ {y} ^ {T} = f _ {y} ^ {T} - 2 \omega_ {i e} C _ {3 3} V _ {x} ^ {T}\tag{8.1.44b}
$$

## 4. 平台的指令角速度

由于 ${ \pmb { \omega } } _ { i T } ^ { T } { = } { \pmb { C } } _ { e } ^ { T } { \pmb { \omega } } _ { i e } ^ { e } + { \pmb { \omega } } _ { e T } ^ { T }$ ，所以平台的指令角速度

$$
\omega_ {\mathrm{cmd} x} ^ {T} = \omega_ {i T x} ^ {T} = C _ {1 3} \omega_ {i e} + \omega_ {e T x} ^ {T}\tag{8.1.45a}
$$

$$
\omega_ {\mathrm{cmdy}} ^ {T} = \omega_ {i T y} ^ {T} = C _ {2 3} \omega_ {i e} + \omega_ {e T y} ^ {T}\tag{8.1.45b}
$$

$$
\omega_ {\mathrm{cmd} z} ^ {T} = \omega_ {i T z} ^ {T} = C _ {3 3} \omega_ {i e}\tag{8.1.45c}
$$

其中， $, \omega _ { e T , \chi } ^ { T }$ 和 $\omega _ { e T _ { 3 } } ^ { T }$ 由式 (8. 1. 42) 确定。

## 5. 游移方位系统简图

在图 8. 1. 中用游移方位角 $\alpha$ 代替自由方位角 $\alpha _ { f }$ ，并注意到 $\omega _ { e T z } ^ { T } = 0$ 即得游移方位系统的工作原理简图，如图 8. 1. 所示

![](images/d9df7387c6eda97472485fb5000e06106951231490e044375729675af69d427b.jpg)  
1. 游移方位系统的 原理简图

与自由方位系统 游移方位系统也避开了在高纬度地区对方位陀螺的施矩困难。比较(8 1. 1)和式(8 1. 40)还可看出 :在计算方 余弦阵中，游移方位系统的计算量比自由方位系统的计算量小，所以是水平式平台惯导设计中的首选方案，LTN-72 系列惯导和国产平台式航空惯导都属于游移方位系统。

## 8.2 平台式惯导系统的误差分析

8. 节分析平台式惯导系统工作原理时，并没有考虑任何误差，将各系统都看作理想系统。但实际情况并非如此，惯性仪器和系统在制造、装调中总存在误差，所有这些误差因素称为误差源。误差游、大致可分为:

(1)元件误差。主要指陀螺漂移、指令角速度刻度系数误差、加速度计零偏和刻度系数误差、计算机舍人误差、电流变换装置误差等。

(2) 安装误差。主要是指陀螺和加速度计在平台上的安装误差。

(3) 初始条件误差。包括平台的初始对准误差、计算机在解算力学编排方程时引人的初始速度及位置误差。

(4) 干扰误差。主要包括冲击与振动运动干扰。

(5) 其他误差。如地球的模型描述误差、有害加速度补偿忽略二阶小量引起的误差等。

上述误差都将引起系统误差，本节对陀螺漂移和加速度计零偏等主要误差源引起的系统误差作全面分析。

## 8.2.1 误差分析中常用的基本关系

1.计算机坐标系(计算系)c

该坐标系是指由计算机输出结果确定的平台坐标系。例如在指北系统中，理想平台坐标系(即地理坐标系 $g )$ 是由所在位置的实际经、纬度 确定的，由计算机解算得的惯导输出经、纬度 $\lambda _ { c }$ 和 $L$ c 确定的地理坐标系即平台坐标系称为计算机坐标系，简称计算系，常用符号 来表示。由于 $\lambda _ { c }$ 和 $L _ { c }$ 相对 总存在误差，所以计算系 相对理想平台系总存在偏差角。

## 2. 位直误差与理想平台系 和计算系 之间的关系

设运载体所在点 的经、纬度分别为 ，平台的方位角为 K(平台水平轴相对东向和北向的偏转角，对指北系统 $K = 0$ 对自由方位系统 $K = \alpha _ { f }$ ，对游移方位系统$K { = } _ { \alpha } )$ ，，1. 对应的计算值为 $\lambda _ { c } \mathrm { ~ } , L _ { c }$ c'相应的误差为 \~À、悦和 \~K 则计算机坐标系 可由理想、平台系经三次旋转确定:第一次旋转绕点 的西向轴旋转乱，即绕 $x _ { g }$ 轴旋转 鼠，第二次旋转绕地球自转轴 $\mathfrak { Z } _ { e }$ 轴旋转$\delta \lambda$ 第三次旋转绕 $S ^ { \prime }$ 点的 $\mathcal { Z } _ { c }$ 轴旋转 \~K 如图 8.2.1 所示。由于 $\delta \lambda$ 、乱和 \~K 都是小量，所以这些旋转角都可看作角矢量，设兰次旋转引起的旋转角矢量为捕，则

$$
\delta \boldsymbol {\theta} = - \delta L \boldsymbol {i} _ {x _ {g}} + \delta \lambda \boldsymbol {k} _ {z _ {e}} + \delta K \boldsymbol {k} _ {z _ {c}}
$$

式中， $, i _ { x _ { g } } \ : , k _ { z _ { e } }$ 和 $k _ { z _ { c } }$ 分别为在地理坐标系 $g$ 、地球坐标系和计算系 $c$ 中相应坐标轴的单位矢量。将上式投影到地

![](images/4810befce688532182fa04cd42b9ff61934f49590d39cfae330afc9f785601b3.jpg)  
8.2.1 计算机坐标系与理想、平台系

理坐标系中，得

$$
\delta \boldsymbol {\theta} ^ {g} = \left[ \begin{array}{c} - \delta L \\ 0 \\ 0 \end{array} \right] + \left[ \begin{array}{c c c} \times & \times & 0 \\ \times & \times & \cos L \\ \times & \times & \sin L \end{array} \right] \left[ \begin{array}{l} 0 \\ 0 \\ \delta \lambda \end{array} \right] + \left[ \begin{array}{c c c} 1 & \delta \theta_ {z} & - \delta \theta_ {y} \\ - \delta \theta_ {z} & 1 & \delta \theta_ {x} \\ \delta \theta_ {y} & - \delta \theta_ {x} & 1 \end{array} \right] \left[ \begin{array}{l} 0 \\ 0 \\ \delta K \end{array} \right] = \left[ \begin{array}{c} - \delta L \\ \delta \lambda \cos L \\ \delta \lambda \sin L + \delta K \end{array} \right]
$$

上式中略去了关于 Ci $\delta \theta _ { x } \hphantom { 0 0 0 0 } . 0 \theta _ { y }$ i(}间的二阶小量，并应用了式(8. 1. 10)

将 $\delta \pmb { \theta }$ 写成在 系内的数学向量形式(在 系内和 系内数学向量仅差高阶无穷小) :

$$
\delta \boldsymbol {\theta} ^ {T} = \boldsymbol {C} _ {g} ^ {T} \delta \boldsymbol {\theta} ^ {g} = \left[ \begin{array}{c c c} \cos K & \sin K & 0 \\ - \sin K & \cos K & 0 \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c} - \delta L \\ \delta \lambda \cos L \\ \delta \lambda \sin L + \delta K \end{array} \right]
$$

即

$$
\delta \theta_ {x} = - \delta L \cos K + \delta \lambda \cos L \sin K\tag{8.2.1a}
$$

$$
\delta \theta_ {y} = \delta L \sin K + \delta \lambda \cos L \cos K\tag{8.2.1b}
$$

$$
\delta \theta_ {z} = \delta \lambda \sin L + \delta K\tag{8.2.1c}
$$

由上式得

$$
\delta L = - \delta \theta_ {x} \cos K + \delta \theta_ {y} \sin K\tag{8.2.2a}
$$

$$
\delta \lambda = \frac {1}{\cos L} (\delta \theta_ {x} \sin K + \delta \theta_ {y} \cos K)\tag{8.2.2b}
$$

$$
\delta K = - \tan L \left(\delta \theta_ {x} \sin K + \delta \theta_ {y} \cos K\right) + \delta \theta_ {z}\tag{8.2.2c}
$$

上式说明，根据 $\delta \theta _ { x } \mathcal { . } \delta \theta _ { y }$ 和 $\delta \theta _ { z }$ . 可确定出CiÀ、ClL和 $\delta K$ 因此称 $\delta \theta _ { x } \hphantom { 0 0 0 0 } . 0 \theta _ { y }$ 和 $\delta \theta _ { z }$ 为位置误差。

## 3. 理想平台革 $T$ 、实际平台革 和计算革 三者关革

根据计算机计算出的位置和速度计算出平台指令角速度和平台的修正回路执行指令后，平台实际位置即平台实际平台坐标系 才被确定，所以 系对 $P$ 系对 系对 系都存在误差。

设 $c$ 系相对 $T$ 系的误差角矢量为 $\begin{array} { r l r l } { \big [ \delta \theta _ { x } } & { { } } & { \delta \theta _ { y } } & { { } } & { \delta \theta _ { z } } \end{array}$ JT , 系相对 $c$ 系的误差角矢量为$\begin{array} { r l r } { \psi _ { x } } & { { } \psi _ { y } } & { \psi _ { z } } \end{array}$ JT $P$ 系相对 $T$ 系的误差角矢量为 $\begin{array} { r l r l } { \mathbf { \dot { \phi } } _ { x } } & { { } \phi _ { y } } & { } & { { } \phi _ { z } \mathbf { ] } ^ { \mathrm { T } } } \end{array}$ ，由于 $P$ 系、 $\setminus ^ { c }$ 系、 系仅差微小偏差，所以上述数学向量可看做 系内的，也可看作 系内的或 $P$ 系内的，引起的误差是关于误差角的高阶小量。根据式(1. 2. 3)

$$
\boldsymbol {C} _ {T} ^ {c} = \left[ \begin{array}{c c c} 1 & \delta \theta_ {z} & - \delta \theta_ {y} \\ - \delta \theta_ {z} & 1 & \delta \theta_ {x} \\ \delta \theta_ {y} & - \delta \theta_ {x} & 1 \end{array} \right]\tag{8.2.3a}
$$

$$
\boldsymbol {C} _ {c} ^ {P} = \left[ \begin{array}{c c c} 1 & \psi_ {z} & - \psi_ {y} \\ - \psi_ {z} & 1 & \psi_ {x} \\ \psi_ {y} & - \psi_ {x} & 1 \end{array} \right]\tag{8.2.3b}
$$

$$
\boldsymbol {C} _ {T} ^ {P} = \left[ \begin{array}{c c c} 1 & \phi_ {z} & - \phi_ {y} \\ - \phi_ {z} & 1 & \phi_ {x} \\ \phi_ {y} & - \phi_ {x} & 1 \end{array} \right]\tag{8.2.3c}
$$

由于 $\pmb { C } _ { T } ^ { P } { = } \pmb { C } _ { c } ^ { P } \cdot \pmb { C } _ { T } ^ { c }$ ，将式(8. 2. 3)代入该式，并略去二阶小量，得

$$
\left[ \begin{array}{c c c} 1 & \phi_ {z} & - \phi_ {y} \\ - \phi_ {z} & 1 & \phi_ {x} \\ \phi_ {y} & - \phi_ {x} & 1 \end{array} \right] = \left[ \begin{array}{c c c} 1 & \psi_ {z} + \delta \theta_ {z} & - (\psi_ {y} + \delta \theta_ {y}) \\ - (\psi_ {z} + \delta \theta_ {z}) & 1 & \psi_ {x} + \delta \theta_ {x} \\ \psi_ {y} + \delta \theta_ {y} & - (\psi_ {x} + \delta \theta_ {x}) & 1 \end{array} \right]
$$

从上式得

$$
\boldsymbol {\phi} = \boldsymbol {\psi} + \delta \boldsymbol {\theta}\tag{8.2.4}
$$

显然 $\phi$ 是平台的姿态误差角。

4. $\psi$ 方程

假设由计算机计算得的平台指令角速度为 ${ \pmb \omega } _ { c }$ ，加到陀螺力矩器上的指令角速度为 ${ \pmb { \omega } } _ { c } ^ { * }$ ，其中

$$
\boldsymbol {\omega} _ {c}
$$

$$
\pmb {\omega} _ {c} ^ {*}
$$

$$
\omega_ {c}
$$

$$
\boldsymbol {\omega} _ {c} ^ {*}
$$

大小相等，但方向不一致，两者相差角矢量 $\psi .$ '8.2.2 所示。由图可得，

$$
\boldsymbol {\omega} _ {c} ^ {*} = \boldsymbol {\omega} _ {c} + \boldsymbol {r}\tag{8.2.5}
$$

由于 $\psi { = } | \psi |$ 很小，所以可近似认为 $r$ 与 ${ \pmb { \omega } } _ { c }$ 正交，$| r | = | \pmb { \omega } _ { c } | \mathrm { t a n } \psi { = } | \pmb { \omega } _ { c } | \psi _ { \circ }$ 又由于 $\psi$ 与 ${ \pmb { \omega } } _ { c }$ 正交，所以位于l ${ \pmb { \psi } } \times \pmb { \omega } _ { c }$ 的方向上，该方向上的单位矢量为

![](images/21cc967f497911505b08294062c729dec5a55df3e75209e981f11df23a0152fc.jpg)

$$
\boldsymbol {r} _ {0} = \frac {\boldsymbol {\psi} \times \boldsymbol {\omega} _ {c}}{\psi | \boldsymbol {\omega} _ {c} |} \quad \boldsymbol {r} = | \boldsymbol {r} | r _ {0} = \boldsymbol {\psi} \times \boldsymbol {\omega} _ {c}
$$

8.2.2 ωc ${ \pmb { \omega } } _ { c } ^ { * }$ 间的关系

上式代入式 (8.2. 5) ，得

$$
\boldsymbol {\omega} _ {c} ^ {*} = \boldsymbol {\omega} _ {c} + \boldsymbol {\psi} \times \boldsymbol {\omega} _ {c}\tag{8.2.6}
$$

这是加到陀螺力矩器的指令角速度，考虑到陀螺的刻度系数误差 $\Delta K _ { G }$ 及陀螺漂移 的影响，平台的实际进动角速度为

$$
\pmb {\omega} _ {i P} = (1 + \Delta K _ {G}) \pmb {\omega} _ {c} ^ {*} + \pmb {\varepsilon}
$$

将式(8. 2. 6)代入上式，并略去三阶小量，得

$$
\boldsymbol {\omega} _ {i P} = \boldsymbol {\omega} _ {c} + \boldsymbol {\psi} \times \boldsymbol {\omega} _ {c} + \Delta K _ {G} \boldsymbol {\omega} _ {c} + \boldsymbol {\varepsilon}\tag{8.2.7}
$$

又由于

$$
\pmb {\omega} _ {i P} = \pmb {\omega} _ {c} + \pmb {\omega} _ {c P}\tag{8.2.8}
$$

式中， ${ \pmb \omega } _ { c P }$ 为平台系 相对计算系 的角速度，是在 系内观察到的 系角位移的变化率，而 $P$ 系相对 系的角位移为 $\psi$ '所以

$$
\pmb {\omega} _ {c P} = \frac {\mathrm{d} \pmb {\psi}}{\mathrm{d} t} \bigg | _ {c}
$$

该式代入式 (8.2. 的，得

$$
\boldsymbol {\omega} _ {i P} = \boldsymbol {\omega} _ {c} + \frac {\mathrm{d} \boldsymbol {\psi}}{\mathrm{d} t} \Big | _ {c}\tag{8.2.9}
$$

比较式 (8. 2. 7) 和式 (8.2. 的，得

$$
\left. \frac {\mathrm{d} \boldsymbol {\psi}}{\mathrm{d} t} \right| _ {c} = \boldsymbol {\psi} \times \boldsymbol {\omega} _ {c} + \Delta K _ {G} \boldsymbol {\omega} _ {c} + \boldsymbol {\varepsilon}\tag{8.2.10}
$$

(8. 2. 10)描述了 的变化规律，称 $\psi$ ￠方程。 $\psi$ ￠方程使误差分析中的高阶联立微分方程组分离成较低阶的微分方程组，从而简化解析分析，这一优点在自由方位系统分析中能够体现出来。

## 8.2.2 指北方位系统的误差分析

## 1.姿态误差方程

将式(8.2.4)写成计算系 内的数学向量形式

$$
\boldsymbol {\phi} ^ {c} = \boldsymbol {\psi} ^ {c} + \delta \boldsymbol {\theta} ^ {c}\tag{8.2.11}
$$

记 $\phi ^ { c } = [ \phi _ { x } \quad \phi _ { y } \quad \phi _ { z } ] ^ { \mathrm { T } } , \delta \theta ^ { c } = [ { \eth } \theta _ { x } \quad { \textnormal { \textsf { O } } } \theta _ { y } \quad { \textnormal { \textsf { O } } } \theta _ { z } ] ^ { \mathrm { T } } , \psi ^ { c } = [ \psi _ { x } \quad \psi _ { y } \quad \psi _ { z } ] ^ { \mathrm { T } }$ ，对式(8.2.11)两边求导，注意到

$$
\dot {\boldsymbol {\psi}} ^ {c} = \left. \frac {\mathrm{d} \boldsymbol {\psi}}{\mathrm{d} t} \right| _ {c} ^ {c}
$$

得

$$
\left[ \begin{array}{l} \dot {\phi} _ {x} \\ \dot {\phi} _ {y} \\ \dot {\phi} _ {z} \end{array} \right] = \left[ \begin{array}{l} \dot {\delta \theta} _ {x} \\ \dot {\delta \theta} _ {y} \\ \dot {\delta \theta} _ {z} \end{array} \right] + \frac {\mathrm{d} \boldsymbol {\psi}}{\mathrm{d} t} \Big | _ {c} ^ {c}
$$

将式(8.2.10)代入上式，为简化解析分析，暂略去 $\Delta K _ { G }$ 的影响，并用圆球近似描述地球，则指北方位系统的姿态误差方程为

$$
\left[ \begin{array}{l} \dot {\phi} _ {x} \\ \dot {\phi} _ {y} \\ \dot {\phi} _ {z} \end{array} \right] = \left[ \begin{array}{l} \delta \dot {\theta} _ {x} \\ \delta \dot {\theta} _ {y} \\ \delta \dot {\theta} _ {z} \end{array} \right] + \left[ \begin{array}{c c c} 0 & - \psi_ {z} & \psi_ {y} \\ \psi_ {z} & 0 & - \psi_ {x} \\ - \psi_ {y} & \psi_ {x} & 0 \end{array} \right] \left[ \begin{array}{l} \omega_ {i g x} ^ {g} \\ \omega_ {i g y} ^ {g} \\ \omega_ {i g z} ^ {g} \end{array} \right] + \left[ \begin{array}{l} \varepsilon_ {x} \\ \varepsilon_ {y} \\ \varepsilon_ {z} \end{array} \right]\tag{8.2.12}
$$

上式中用 ${ \pmb \omega } _ { i g }$ 代替 ${ \pmb { \omega } } _ { c }$ ，的原因是 ${ \pmb { \omega } } _ { c }$ 是为使平台跟踪地理坐标系而加的指令， ${ \bf \nabla } _ { \bf \Omega } \langle \pmb { \omega } _ { c }$ ，与 ${ \pmb { \omega } } _ { i g }$ 间的误差量是小量，所以lf ${ \pmb { \psi } } \times { \pmb { \omega } } _ { c } $ ，与 ${ \pmb { \psi } } \times { \pmb { \omega } } _ { i g }$ 差高阶小量。

对指北方位系统 $K = 0 , \delta K = 0$ 所以式(8.2. 1)可写成

$$
\delta \theta_ {x} = - \delta L\tag{8.2.13a}
$$

$$
\delta \theta_ {y} = \delta \lambda \cos L\tag{8.2.13b}
$$

$$
\delta \theta_ {z} = \delta \lambda \sin L\tag{8.2.13c}
$$

对上述诸式两边分别求导

$$
\dot {\delta \theta_ {x}} = - \delta \dot {L}\tag{8.2.14a}
$$

$$
\dot {\delta \theta_ {y}} = \dot {\delta \lambda} \cos L - \dot {L} \delta \lambda \sin L\tag{8.2.14b}
$$

$$
\dot {\delta \theta_ {z}} = \dot {\delta \lambda} \sin L + \dot {L} \delta \lambda \cos L\tag{8.2.14c}
$$

根据式 (8. 1. 6)

$$
\delta \dot {L} = \frac {\delta V _ {N}}{R}\tag{8.2.15}
$$

$$
\dot {\delta \lambda} = \frac {1}{R} (V _ {E} ^ {c} \sec L _ {c} - V _ {E} \sec L)
$$

按泰勒级数展开下式并取一阶近似以

所以

$$
\sec L _ {c} = \sec (L + \delta L) = \sec L + \delta L \sec L \tan L
$$

$$
\delta \dot {\lambda} = \frac {1}{R} \left(\delta V _ {E} \sec L + \delta L V _ {E} \sec L \tan L\right)\tag{8.2.16}
$$

将式(ω8.2.15) 、式 (8. 2. 16)及式 (8. 1. 6a)代入式 (8. 2. 14) ，得

$$
\dot {\delta \theta_ {x}} = - \frac {\delta V _ {N}}{R}\tag{8.2.17a}
$$

$$
\dot {\delta \theta_ {y}} = \frac {\delta V _ {E}}{R} + \frac {V _ {E}}{R} \delta L \tan L - \frac {V _ {N}}{R} \delta \lambda \sin L\tag{8.2.17b}
$$

$$
\dot {\delta \theta_ {z}} = \frac {\delta V _ {E}}{R} \tan L + \frac {V _ {E}}{R} \delta L \tan^ {2} L + \frac {V _ {N}}{R} \delta \lambda \cos L\tag{8.2.17c}
$$

又由式 (8.2.4) 和式 (8. 2. 13) ，得

$$
\phi_ {x} = \phi_ {x} + \delta L\tag{8.2.18a}
$$

$$
\phi_ {y} = \phi_ {y} - \delta \lambda \cos L\tag{8.2.18b}
$$

$$
\psi_ {z} = \phi_ {z} - \delta \lambda \sin L\tag{8.2.18c}
$$

将式 (8.2.17) 、式 (8. 2. 18) 和式 (8. 1. 2)代入式 (8.2.12) ，得指北方位系统的平台姿态误差方程

$$
\dot {\phi} _ {x} = - \frac {\delta V _ {N}}{R} + \phi_ {y} \left(\omega_ {i e} \sin L + \frac {V _ {E}}{R} \tan L\right) - \phi_ {z} \left(\omega_ {i e} \cos L + \frac {V _ {E}}{R}\right) + \varepsilon_ {E}\tag{8.2.19a}
$$

$$
\dot {\phi} _ {y} = \frac {\delta V _ {E}}{R} - \phi_ {z} \frac {V _ {N}}{R} - \phi_ {x} \left(\omega_ {i e} \sin L + \frac {V _ {E}}{R} \tan L\right) - \delta L \omega_ {i e} \sin L + \varepsilon_ {N}\tag{8.2.19b}
$$

$$
\dot {\phi} _ {z} = \frac {\delta V _ {E}}{R} \tan L + \delta L \left(\omega_ {i e} \cos L + \frac {V _ {E}}{R} \sec^ {2} L\right) + \phi_ {x} \left(\omega_ {i e} \cos L + \frac {V _ {E}}{R}\right) + \phi_ {y} \frac {V _ {N}}{R} + \varepsilon_ {U}\tag{8.2.19c}
$$

## 2. 速度误差方程

根据式(8. 1. 5) ，当不考虑任何误差时，无误差速度由下式确定

$$
\dot {V} _ {E} = f _ {E} + \left(2 \omega_ {i e} \sin L + \frac {V _ {E}}{R} \tan L\right) V _ {N}\tag{8.2.20a}
$$

$$
\dot {V} _ {N} = f _ {N} - \left(2 \omega_ {i e} \sin L + \frac {V _ {E}}{R} \tan L\right) V _ {E}\tag{8.2.20b}
$$

由于加速度计的输出误差、平台的姿态误差及计算误差等影响，系统的速度输出由下式确定

$$
\dot {V} _ {E c} = f _ {E c} + \left(2 \omega_ {i e} \sin L _ {c} + \frac {V _ {E c}}{R} \tan L _ {c}\right) V _ {N c}\tag{8.2.21a}
$$

$$
\dot {V} _ {N c} = f _ {N c} - \left(2 \omega_ {i e} \sin L _ {c} + \frac {V _ {E _ {c}}}{R} \tan L _ {c}\right) V _ {E c}\tag{8.2.21b}
$$

显然

$$
\delta V _ {E} = V _ {E c} - V _ {E}, \delta V _ {N} = V _ {N c} - V _ {N}, \delta L = L _ {c} - L
$$

是系统的误差。假设平台的姿态误差角为弘 $\phi _ { x } \setminus \phi _ { y } \setminus \phi _ { z }$ ，则加速度计输出的比力为

$$
\boldsymbol {f} _ {c} = \boldsymbol {C} _ {g} ^ {P} \boldsymbol {f} ^ {g} + \nabla^ {P} = \left[ \begin{array}{c c c} 1 & \phi_ {z} & - \phi_ {y} \\ - \phi_ {z} & 1 & \phi_ {x} \\ \phi_ {y} & - \phi_ {x} & 1 \end{array} \right] \left[ \begin{array}{l} f _ {E} \\ f _ {N} \\ f _ {U} \end{array} \right] + \left[ \begin{array}{l} \nabla_ {E} \\ \nabla_ {N} \\ \nabla_ {U} \end{array} \right]
$$

所以

$$
\begin{array}{l} f _ {E c} = f _ {E} + \phi_ {z} f _ {N} - \phi_ {y} f _ {U} + \nabla_ {E} \\ f _ {N c} = f _ {N} - \phi_ {z} f _ {E} + \phi_ {x} f _ {U} + \nabla_ {N} \end{array}\tag{8.2.22a}
$$

(8. 2. 22b)

按泰勒级数展开下式

$$
\begin{array}{c} \tan L _ {c} = \tan (L + \delta L) = \tan L + \delta L \sec^ {2} L \\ \sin L _ {c} = \sin (L + \delta L) = \sin L + \delta L \cos L \end{array}
$$

上述诸关系式代入式(8.2.21) ，整理并略去关于误差的二阶小量，得

$$
\begin{array}{r l} \dot {V} _ {E} + \delta \dot {V} _ {E} & = f _ {E} + \left(2 \omega_ {i e} \sin L + \frac {V _ {E}}{R} \tan L\right) (V _ {N} + \delta V _ {N}) + \delta L \left(2 \omega_ {i e} \cos L V _ {N} + \frac {V _ {E} V _ {N}}{R} \sec^ {2} L\right) \\ & + \delta V _ {E} \frac {V _ {N}}{R} \tan L + \phi_ {z} f _ {N} - \phi_ {y} f _ {U} + \nabla_ {E} \\ \dot {V} _ {N} + \delta \dot {V} _ {N} & = f _ {N} - \left(2 \omega_ {i e} \sin L + \frac {V _ {E}}{R} \tan L\right) (V _ {E} + \delta V _ {E}) - \delta L \left(2 \omega_ {i e} \cos L V _ {E} + \frac {V _ {E} ^ {2}}{R} \sec^ {2} L\right) \\ & - \delta V _ {E} \frac {V _ {E}}{R} \tan L - \phi_ {z} f _ {E} + \phi_ {x} f _ {U} + \nabla_ {N} \end{array} \tag {8.2}\tag{8.2.23a}
$$

(8. 2. 23b)

将式(8.2.23)与式(8.2.20)相应式左右两边相减，得速度误差方程:

$$
\begin{array}{r l} \delta \dot {V} _ {E} & = \left(2 \omega_ {i e} \sin L + \frac {V _ {E}}{R} \tan L\right) \delta V _ {N} + \delta L \left(2 V _ {N} \omega_ {i e} \cos L + \frac {V _ {E} V _ {N}}{R} \sec^ {2} L\right) \\ & + \delta V _ {E} \frac {V _ {N}}{R} \tan L + \phi_ {z} f _ {N} - \phi_ {y} f _ {U} + \nabla_ {E} \\ \delta \dot {V} _ {N} & = - \left(2 \omega_ {i e} \sin L + \frac {V _ {E}}{R} \tan L\right) \delta V _ {E} - \delta L \left(2 V _ {E} \omega_ {i e} \cos L + \frac {V _ {E} ^ {2}}{R} \sec^ {2} L\right) \\ & - \delta V _ {E} \frac {V _ {E}}{R} \tan L - \phi_ {z} f _ {E} + \phi_ {x} f _ {U} + \nabla_ {N} \end{array}\tag{8.2.24a}
$$

(8. 2. 24b)

根据式(8. 1. 4) ，当略去垂直速度 $V _ { U }$ 的影响时

$$
f _ {E} = \dot {V} _ {E} - \left(2 \omega_ {i e} \sin L + \frac {V _ {E}}{R} \tan L\right) V _ {N}\tag{8.2.25a}
$$

$$
f _ {N} = \dot {V} _ {N} + \left(2 \omega_ {i e} \sin L + \frac {V _ {E}}{R} \tan L\right) V _ {E}
$$

$$
f _ {U} = \dot {V} _ {U} - \left(2 \omega_ {i e} \cos L + \frac {V _ {E}}{R}\right) V _ {E} - \frac {V _ {N} ^ {2}}{R} + g\tag{8.2.25b}
$$

(8.2. 25c)

将式(8.2.25)代入式(8.2.24) ，得便于解析分析的速度误差方程

$$
\begin{array}{r l} \delta \dot {V} _ {E} & = \left(2 \omega_ {i e} \sin L + \frac {V _ {E}}{R} \tan L\right) \delta V _ {N} + \delta L \left(2 V _ {N} \omega_ {i e} \cos L + \frac {V _ {E} V _ {N}}{R} \sec^ {2} L\right) \\ & + \delta V _ {E} \frac {V _ {N}}{R} \tan L + \phi_ {z} \left[ \dot {V} _ {N} + \left(2 \omega_ {i e} \sin L + \frac {V _ {E}}{R} \tan L\right) V _ {E} \right] \\ & - \phi_ {y} \left[ \dot {V} _ {U} - \left(2 \omega_ {i e} \cos L + \frac {V _ {E}}{R}\right) V _ {E} - \frac {V _ {N} ^ {2}}{R} + g \right] + \nabla_ {E} \\ \delta \dot {V} _ {N} & = - \left(2 \omega_ {i e} \sin L + \frac {V _ {E}}{R} \tan L\right) \delta V _ {E} - \delta L \left(2 V _ {E} \omega_ {i e} \cos L + \frac {V _ {E} ^ {2}}{R} \sec^ {2} L\right) \\ & - \phi_ {z} \left[ \dot {V} _ {E} - \left(2 \omega_ {i e} \sin L + \frac {V _ {E}}{R} \tan L\right) V _ {N} \right] \end{array}\tag{8.2.26a}
$$

$$
+ \phi_ {x} \left[ \dot {V} _ {U} - \left(2 \omega_ {i e} \cos L + \frac {V _ {E}}{R}\right) V _ {E} - \frac {V _ {N} ^ {2}}{R} + g \right] + \nabla_ {N}\tag{8.2.26b}
$$

## 3. 定位误差方程

(8.2.1日和式(8.2. 16) 已给出了定位误差方程:

$$
\delta \dot {L} = \frac {\delta V _ {N}}{R}, \quad \delta \dot {\lambda} = \frac {\delta V _ {E}}{R} \sec L + \delta L \frac {V _ {E}}{R} \sec L \tan L
$$

## 4. 静基座条件下的系统误差方程

在静基座条件下，即 $V _ { E } = V _ { N } = V _ { U } = 0 , \dot { V } _ { E } = \dot { V } _ { N } = \dot { V } _ { U } = 0$ 根据式 (8.2.19) 、式 (8.2.26)(8.2. 15)及式 (8. 2. 16) ，系统误差方程为

$$
\dot {\delta V _ {E}} = 2 \omega_ {i e} \sin L \delta V _ {N} - \phi_ {y} g + \nabla_ {E}\tag{8.2.27a}
$$

$$
\delta \dot {V} _ {N} = - 2 \omega_ {i e} \sin L \delta V _ {E} + \phi_ {x} g + \nabla_ {N}\tag{8.2.27b}
$$

$$
\dot {\phi} _ {x} = - \frac {\delta V _ {N}}{R} + \phi_ {y} \omega_ {i e} \sin L - \phi_ {z} \omega_ {i e} \cos L + \varepsilon_ {E}\tag{8.2.27c}
$$

$$
\dot {\phi} _ {y} = \frac {\delta V _ {E}}{R} - \phi_ {x} \omega_ {i e} \sin L - \delta L \omega_ {i e} \sin L + \varepsilon_ {N}\tag{8.2.27d}
$$

$$
\dot {\phi} _ {z} = \frac {\delta V _ {E}}{R} \tan L + \delta L \omega_ {i e} \cos L + \phi_ {x} \omega_ {i e} \cos L + \varepsilon_ {U}\tag{8.2.27e}
$$

$$
\delta \dot {L} = \frac {\delta V _ {N}}{R}\tag{8.2.27f}
$$

$$
\delta \dot {\lambda} = \frac {\delta V _ {E}}{R} \sec L\tag{8.2.28}
$$

## 5. 静基座条件下系统误差传播特性分析

将式(8.2.27)写成矩阵形式

$$
\left[ \begin{array}{c} \delta \dot {V} _ {E} \\ \delta \dot {V} _ {N} \\ \delta \dot {L} \\ \dot {\phi} _ {x} \\ \dot {\phi} _ {y} \\ \dot {\phi} _ {z} \end{array} \right] = \left[ \begin{array}{c c c c c c} 0 & 2 \omega_ {i e} \sin L & 0 & 0 & - g & 0 \\ - 2 \omega_ {i e} \sin L & 0 & 0 & g & 0 & 0 \\ 0 & \frac {1}{R} & 0 & 0 & 0 & 0 \\ 0 & - \frac {1}{R} & 0 & 0 & \omega_ {i e} \sin L & - \omega_ {i e} \cos L \\ \frac {1}{R} & 0 & - \omega_ {i e} \sin L & - \omega_ {i e} \sin L & 0 & 0 \\ \frac {\tan L}{R} & 0 & \omega_ {i e} \cos L & \omega_ {i e} \cos L & 0 & 0 \end{array} \right]
$$

$$
\times \left[ \begin{array}{c} \delta V _ {E} \\ \delta V _ {N} \\ \delta L \\ \phi_ {x} \\ \phi_ {y} \\ \phi_ {z} \end{array} \right] + \left[ \begin{array}{c} \nabla_ {E} \\ \nabla_ {N} \\ 0 \\ \varepsilon_ {E} \\ \varepsilon_ {N} \\ \varepsilon_ {U} \end{array} \right]\tag{8.2.29}
$$

\~p

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {A} \boldsymbol {X} (t) + \boldsymbol {W}
$$

系统误差 X(t) 由两类误差源引起 元器件误差' $\nabla _ { E } \setminus \nabla _ { N } \setminus \mathfrak { E } _ { E } \setminus \mathfrak { E } _ { N } \setminus \mathfrak { E } _ { U }$ ; 系统误差初值 $\delta V _ { E } \left( 0 \right)$ $\begin{array} { r } { \hat { \partial } V _ { N } ( 0 ) \hat { \partial } \hat { L } ( 0 ) \hat { \partial } \hat { \partial } x \hat { ( 0 ) } \hat { \partial } \hat { \partial } y \hat { ( 0 ) } \hat { \partial } \hat { \partial } z \hat { ( 0 ) } } \end{array}$ 。对上式作拉氏变换

$$
\begin{array}{c} s \boldsymbol {X} (s) = \boldsymbol {A} \boldsymbol {X} (s) + \boldsymbol {X} (0) + \boldsymbol {W} (s) \\ \boldsymbol {X} (s) = (s \boldsymbol {I} - \boldsymbol {A}) ^ {- 1} [ \boldsymbol {X} (0) + \boldsymbol {W} (s) ] \end{array}\tag{8.2.30}
$$

系统的特征多项式为

$$
\Delta (s) = \left| s \boldsymbol {I} - \boldsymbol {A} \right| = \left(s ^ {2} + \omega_ {i e} ^ {2}\right) \left[ s ^ {4} + 2 s ^ {2} \left(\omega_ {s} ^ {2} + 2 \omega_ {i e} ^ {2} \sin^ {2} L\right) + \omega_ {s} ^ {4} \right]
$$

式中， $, \omega _ { s }$ 盹为休拉频率。所以系统的特征根由下述两个方程确定

$$
s ^ {2} + \omega_ {i e} ^ {2} = 0\tag{8.2.31}
$$

$$
s ^ {4} + 2 s ^ {2} \left(\omega_ {s} ^ {2} + 2 \omega_ {i e} ^ {2} \sin^ {2} L\right) + \omega_ {s} ^ {4} = 0\tag{8.2.32}
$$

由式 (8. 2. 31) ，得特征根

$$
s _ {1, 2} = \pm \mathrm{j} \omega_ {i e}\tag{8.2.33}
$$

记式 (8.2.32) 左侧为 $L \left( s \right)$ ，则

$$
\begin{array}{r l} L (s) & = s ^ {4} + 2 s ^ {2} \left(\omega_ {s} ^ {2} + 2 \omega_ {i e} ^ {2} \sin^ {2} L\right) + \omega_ {s} ^ {4} \\ & = \left(s ^ {2} + \omega_ {s} ^ {2}\right) ^ {2} - \left(j 2 s \omega_ {i e} \sin L\right) ^ {2} \\ & = \left(s ^ {2} + j 2 s \omega_ {i e} \sin L + \omega_ {s} ^ {2}\right) \left(s ^ {2} - j 2 s \omega_ {i e} \sin L + \omega_ {s} ^ {2}\right) \end{array}
$$

所以式 (8.2.32) 可写成

$$
\begin{array}{l} s ^ {2} + \mathrm{j} 2 s \omega_ {i e} \sin L + \omega_ {s} ^ {2} = 0 \\ s ^ {2} - \mathrm{j} 2 s \omega_ {i e} \sin L + \omega_ {s} ^ {2} = 0 \end{array}
$$

解该两方程，得

$$
\begin{array}{l} s _ {3, 4} = \frac {- \mathrm{j} 2 \omega_ {i e} \sin L \pm \sqrt {- 4 \omega_ {i e} ^ {2} \sin^ {2} L - 4 \omega_ {s} ^ {2}}}{2} \\ s _ {5, 6} = \frac {\mathrm{j} 2 \omega_ {i e} \sin L \pm \sqrt {- 4 \omega_ {i e} ^ {2} \sin^ {2} L - 4 \omega_ {s} ^ {2}}}{2} \end{array}
$$

由于 $\omega _ { s }$ >> $\omega _ { i e } \sin L$ ，所以

$$
\begin{array}{l l} s _ {3} = - \mathrm{j} \omega_ {i e} \sin L + \mathrm{j} \omega_ {s}, & s _ {4} = - \mathrm{j} \omega_ {i e} \sin L - \mathrm{j} \omega_ {s} \\ s _ {5} = \mathrm{j} \omega_ {i e} \sin L + \mathrm{j} \omega_ {s}, & s _ {6} = \mathrm{j} \omega_ {i e} \sin L - \mathrm{j} \omega_ {s} \end{array}
$$

\~p

$$
\begin{array}{l} s _ {5, 4} = \pm \mathrm{j} (\omega_ {s} + \omega_ {i e} \sin L) \\ s _ {3, 6} = \pm \mathrm{j} (\omega_ {s} - \omega_ {i e} \sin L) \end{array}\tag{8.2.34}
$$

(8. 2. 35)

由式 (8.2.33) 、式 (8.2.34)及式 (8.2.35) ，特征根 $s _ { 1 , 2 }$ 对应的振荡基础项为

$$
\sin \omega_ {i e} t, \cos \omega_ {i e} t
$$

特征根 $s _ { 5 , 4 }$ 对应的振荡基础项为

$$
\sin \left[ \left(\omega_ {s} + \omega_ {i e} \sin L\right) t \right], \quad \cos \left[ \left(\omega_ {s} + \omega_ {i e} \sin L\right) t \right]
$$

特征根 $s _ { 3 \cdot 6 }$ 对应的振荡基础项为

$$
\sin \left[ \left(\omega_ {s} - \omega_ {i e} \sin L\right) t \right], \quad \cos \left[ \left(\omega_ {s} - \omega_ {i e} \sin L\right) t \right]
$$

现以 $\sin [ ( \omega _ { s } + \omega _ { i e } \sin L ) t ]$ 为例分析振荡特点:

$$
\begin{array}{r l} \sin [ (\omega_ {s} + \omega_ {i e} \sin L) t ] & = \sin (\omega_ {s} t + \sin L \omega_ {i e} t) \\ & = \cos (\sin L \omega_ {i e} t) \sin (\omega_ {s} t) + \sin (\sin L \omega_ {i e} t) \cos (\omega_ {s} t) \\ & = a _ {c} (t) \sin (\omega_ {s} t) + a _ {s} (t) \cos (\omega_ {s} t) \end{array}
$$

若记 $s _ { 1 } \left( t \right) = a _ { c } \left( t \right) \sin ( \omega _ { s } t )$ ，则 $s _ { 1 } \left( t \right)$ 是 $a _ { c } \left( t \right) = \cos ( \sin L \omega _ { i e } t )$ 对 $\sin ( \omega _ { s } t )$ 的调制，如图 8.2.3 所示。其中， $\bullet \bullet _ { f } = \sin L \omega _ { i e }$ 称为傅科频率。

![](images/7e8541e55abb272f8accc31ed9bf5767efe5e298fa9a4669a4e6417884886ba3.jpg)  
8.2.3 傅科振荡对休拉振荡的调制

因此惯导系统误差中存在三种频率的振荡:

(1)地球振荡，频率为 $\omega _ { i e }$ ，周期为 24 小时。

(2) 傅科振荡，频率为 $\omega _ { i e } \sin { L }$ L，周期随纬度而变，纬度越低，周期越长，在赤道上，傅科频率为零，傅科振荡消失，在两极，傅科振荡蜕化为地球振荡。

(3) 休拉振荡，振荡周期为 84. 分钟。

由于傅科周期 $T _ { f } = { \frac { 2 \pi } { \omega _ { i e } \sin L } } { = } { \frac { 2 4 } { \sin L } }$ 小时， 中低纬度 域，如 $L = 3 0 ^ { \circ }$ 处， $T _ { f } { = } 4 8$ 小时，对于工作时间数小时的惯导， $\mathrm { s i n } L \omega _ { i e } t { \approx } 0$ 傅科振荡在系统误差中体现不明显，为简化分析，略去傅科振荡的影响，对式(8. 2. 30)作拉氏反变换，可得系统误差的时间解:

$$
\begin{array}{l} \delta V _ {E} (t) = \frac {g \sin L}{\omega_ {s} ^ {2} - \omega_ {i e} ^ {2}} \left(\sin \omega_ {i e} t - \frac {\omega_ {i e}}{\omega_ {s}} \sin \omega_ {s} t\right) \varepsilon_ {E} \\ \quad + \left(\frac {\omega_ {s} ^ {2} - \omega_ {i e} ^ {2} \cos^ {2} L}{\omega_ {s} ^ {2} - \omega_ {i e} ^ {2}} \cos \omega_ {s} t - \frac {\omega_ {s} ^ {2} \sin^ {2} L}{\omega_ {s} ^ {2} - \omega_ {i e} ^ {2}} \cos \omega_ {i e} t - \cos^ {2} L\right) \varepsilon_ {N} \\ \quad + \frac {R}{2} \sin 2 L \left(\frac {\omega_ {s} ^ {2}}{\omega_ {s} ^ {2} - \omega_ {i e} ^ {2}} \cos \omega_ {i e} t - \frac {\omega_ {i e} ^ {2}}{\omega_ {s} ^ {2} - \omega_ {i e} ^ {2}} \cos \omega_ {s} t - 1\right) \varepsilon_ {U} + \frac {\nabla_ {E}}{\omega_ {s}} \sin \omega_ {s} t \\ \delta V _ {N} (t) = \frac {g}{\omega_ {s} ^ {2} - \omega_ {i e} ^ {2}} (\cos \omega_ {i e} t - \cos \omega_ {s} t) \varepsilon_ {E} + \frac {g \sin L}{\omega_ {s} ^ {2} - \omega_ {i e} ^ {2}} \left(\sin \omega_ {i e} t - \frac {\omega_ {i e}}{\omega_ {s}} \sin \omega_ {s} t\right) \varepsilon_ {N} \\ \quad + \frac {\omega_ {s} \cos L}{\omega_ {s} ^ {2} - \omega_ {i e} ^ {2}} (\omega_ {i e} \sin \omega_ {s} t - \omega_ {s}) \varepsilon_ {U} + \frac {\nabla_ {N}}{\omega_ {s}} \sin \omega_ {s} t \end{array}\tag{8.2.36a}
$$

(8.2. 36b)

$$
\begin{array}{l} \delta L (t) = \frac {\omega_ {s} ^ {2}}{\omega_ {s} ^ {2} - \omega_ {i e} ^ {2}} \left(\frac {1}{\omega_ {i e}} \sin \omega_ {i e} t - \frac {1}{\omega_ {s}} \sin \omega_ {s} t\right) \varepsilon_ {E} \\ \quad + \left[ \frac {\omega_ {i e} \sin L}{\omega_ {s} ^ {2} - \omega_ {i e} ^ {2}} \left(\cos \omega_ {s} t - \frac {\omega_ {s} ^ {2}}{\omega_ {i e} ^ {2}} \cos \omega_ {i e} t\right) + \frac {\sin L}{\omega_ {i e}} \right] \varepsilon_ {N} \\ \quad + \left[ \frac {\omega_ {s} ^ {2} \cos L}{\omega_ {i e} (\omega_ {s} ^ {2} - \omega_ {i e} ^ {2})} \cos \omega_ {i e} t - \frac {\omega_ {i e} \cos L}{\omega_ {s} ^ {2} - \omega_ {i e} ^ {2}} \cos \omega_ {s} t - \frac {\cos L}{\omega_ {i e}} \right] \varepsilon_ {U} + \frac {\nabla_ {N}}{g} (1 - \cos \omega_ {s} t) \end{array}\tag{8.2.36c}
$$

$$
\begin{array}{l} \delta \lambda (t) = \left[ \frac {\tan L}{\omega_ {i e}} (1 - \cos \omega_ {i e} t) - \frac {\omega_ {i e} \tan L}{\omega_ {s} ^ {2} - \omega_ {i e} ^ {2}} (\cos \omega_ {i e} t - \cos \omega_ {s} t) \right] \varepsilon_ {E} \\ + \left[ \frac {\omega_ {s} ^ {2} - \omega_ {i e} ^ {2} \cos^ {2} L}{\omega_ {s} (\omega_ {s} ^ {2} - \omega_ {i e} ^ {2}) \cos L} \sin \omega_ {s} t - \frac {\omega_ {s} ^ {2} \tan L \sin L}{\omega_ {i e} (\omega_ {s} ^ {2} - \omega_ {i e} ^ {2})} \sin \omega_ {i e} t - t \cos L \right] \varepsilon_ {N} \\ + \left[ \frac {\omega_ {s} ^ {2} \sin L}{\omega_ {i e} (\omega_ {s} ^ {2} + \omega_ {i e} ^ {2})} \sin \omega_ {i e} t - \frac {\omega_ {i e} ^ {2} \sin L}{\omega_ {s} (\omega_ {s} ^ {2} + \omega_ {i e} ^ {2})} \sin \omega_ {s} t - t \sin L \right] \varepsilon_ {U} + \frac {\nabla_ {E}}{g \cos L} (1 - \cos \omega_ {s} t) \end{array}\tag{8.2.36d}
$$

$$
\begin{array}{l} \phi_ {x} (t) = \frac {1}{\omega_ {s} ^ {2} - \omega_ {i e} ^ {2}} (\omega_ {s} \cos \omega_ {s} t - \omega_ {i e} \sin \omega_ {i e} t) \varepsilon_ {E} + \frac {\omega_ {i e} \sin L}{\omega_ {s} ^ {2} - \omega_ {i e} ^ {2}} (\cos \omega_ {i e} t - \cos \omega_ {s} t) \varepsilon_ {N} \\ + \frac {\omega_ {i e} \cos L}{\omega_ {s} ^ {2} - \omega_ {i e} ^ {2}} (\cos \omega_ {s} t - \cos \omega_ {i e} t) \varepsilon_ {U} - \frac {\nabla_ {N}}{g} (1 - \cos \omega_ {s} t) \end{array}\tag{8.2.36e}
$$

$$
\begin{array}{l} \phi_ {y} (t) = \frac {\omega_ {i e} \sin L}{\omega_ {s} ^ {2} - \omega_ {i e} ^ {2}} (\cos \omega_ {s} t - \cos \omega_ {i e} t) \varepsilon_ {E} \\ \quad + \left[ \frac {\omega_ {s} ^ {2} - \omega_ {i e} ^ {2} \cos^ {2} L}{\omega_ {s} (\omega_ {s} ^ {2} - \omega_ {i e} ^ {2})} \sin \omega_ {s} t - \frac {\omega_ {i e} \sin^ {2} L}{\omega_ {s} ^ {2} - \omega_ {i e} ^ {2}} \sin \omega_ {i e} t \right] \varepsilon_ {N} \\ \quad + \frac {\omega_ {i e} \sin 2 L}{2 (\omega_ {s} ^ {2} - \omega_ {i e} ^ {2})} \Big (\sin \omega_ {i e} t - \frac {\omega_ {i e}}{\omega_ {s}} \sin \omega_ {s} t \Big) \varepsilon_ {U} + \frac {\nabla_ {E}}{g} (1 - \cos \omega_ {s} t) \end{array}\tag{8.2.36f}
$$

$$
\begin{array}{l} \phi_ {z} (t) = \left[ \frac {\omega_ {i e} \sin L \tan L}{\omega_ {s} ^ {2} - \omega_ {i e} ^ {2}} (\cos \omega_ {s} t - \cos \omega_ {i e} t) + \frac {1}{\omega_ {i e} \cos L} (1 - \cos \omega_ {i e} t) \right] \varepsilon_ {E} \\ \quad + \frac {\omega_ {i e} ^ {2} \sin 2 L - 2 \omega_ {s} ^ {2} \tan L}{2 (\omega_ {s} ^ {2} - \omega_ {i e} ^ {2})} \left(\frac {1}{\omega_ {i e}} \sin \omega_ {i e} t - \frac {1}{\omega_ {s}} \sin \omega_ {s} t\right) \varepsilon_ {N} \\ \quad + \left[ \frac {\omega_ {s} ^ {2} - \omega_ {i e} ^ {2} \cos^ {2} L}{\omega_ {i e} (\omega_ {s} ^ {2} - \omega_ {i e} ^ {2})} \sin \omega_ {i e} t - \frac {\omega_ {i e} ^ {2} \sin^ {2} L}{\omega_ {s} (\omega_ {s} ^ {2} - \omega_ {i e} ^ {2})} \sin \omega_ {s} t \right] \varepsilon_ {U} + \frac {\nabla_ {E} \tan L}{g} (1 - \cos \omega_ {s} t) \end{array}\tag{8.2.36g}
$$

从式(8.2.36)看出，陀螺漂移引起的系统误差分三类 ①振荡型，振荡周期有两种:84.4钟和 24 小时 ②常值型 ③积累型，随时间 线性增加。对系统精度影响大的是后两类误差，特别是积累型，引起导航精度的发散。

由式(8.2.36ω知，北向陀螺漂移和方位陀螺漂移引起经度的积累误差

$$
\delta \lambda_ {\text {积}} (t) = - (\varepsilon_ {N} \mathrm{cos} L + \varepsilon_ {U} \mathrm{sin} L) t
$$

设 $L { = } 4 5 ^ { o } , \varepsilon _ { N } { = } { \varepsilon } _ { U } { = } 0 . \ 0 1 ^ { o } / \mathrm { h } , t { = } 1 \mathrm { h }$ ,则

$$
\delta \lambda_ {\text {积}} = - \left[ 0. 0 1 \times \frac {\sqrt {2}}{2} + 0. 0 1 \times \frac {\sqrt {2}}{2} \right] \times 6 0 = - 0. 8 5 (^ {\prime})
$$

对应的大圆定位误差为 0.85n mile，接近 1 n mile 所以根据陀螺精度可对惯导系统的定位精度作粗略估计 若陀螺漂移为0.01°/h，则惯导的定位误差大约为 $\mathrm { 1 n ~ m i l e / h }$ o

从式(8.2.36)可归纳出表 8.2.1 所列的定性结论。

8.2.1 器件误差引起的系统误差

<table><tr><td>误差源\系统误差</td><td>速度</td><td>纬度</td><td>经度</td><td> $\phi_x$ </td><td> $\phi_y$ </td><td> $\phi_z$ </td></tr><tr><td> $\epsilon_E$ </td><td>振荡</td><td>振荡</td><td>常值</td><td>振荡</td><td>振荡</td><td>常值</td></tr><tr><td> $\epsilon_N$ </td><td>常值</td><td>常值</td><td>积累</td><td>振荡</td><td>振荡</td><td>振荡</td></tr><tr><td> $\epsilon_U$ </td><td>常值</td><td>常值</td><td>积累</td><td>振荡</td><td>振荡</td><td>振荡</td></tr><tr><td> $\nabla_E$ </td><td>振荡</td><td>0</td><td>常值</td><td>0</td><td>常值</td><td>常值</td></tr><tr><td> $\nabla_N$ </td><td>0</td><td>常值</td><td>0</td><td>常值</td><td>0</td><td>0</td></tr></table>

用类似方法可分析由误差初值引起的系统误差，此处不再作详细分析，而只给出结论:由误差初值引起的系统误差几乎都是振荡型的，振荡周期也有三种 休拉周期、地球周期和傅科周期，仅 $\phi _ { y } \left( 0 \right)$ 和 $\phi _ { z } \mathopen { } \mathclose \bgroup \left( 0 \aftergroup \egroup \right)$ 引起经度和方位的常值误差。

上述结论虽然是分析指北方位系统后得出的，但实际上也适用于其余系统。这是由于所有惯导系统都源于基本方程，仅导航坐标系选取不一样，使系统的力学编排存在差异，本质上各系统是等效的。

惯导系统的误差分析也可通过计算机仿真来完成，此时获得的是系统误差的数值解。具体做法大致为 根据假设的飞行轨迹解算出飞机的真实导航参数，如速度、纬度、经度等，采用四阶龙格-库塔算法联立求解式 (8.2. 19) 、式 (8.2.24) 、式 (8. 2. 15) 和式 (8.2.16)微分方程组.

## 8.2.3 游移方位惯导系统误差方程

游移方位系统克服了指北方位系统在高纬度地区不能正常工作的问题，与自由方位系统相比导航计算量相对较低，所以是平台式惯导系统设计中的主选系统，一度作为航空标准愤导的LTN-72 系列平台式惯导就属于此类系统，国产平台式航空惯导也都选用此设计方案。为便于在系统误差的计算机仿真分析和组合导航系统卡尔曼滤波器设计中的应用，此处详细推导游移方位系统的误差方程。

## 1.速度误差方程

记游移方位系统的理想平台坐标系为 $W$ 实际平台坐标系为 $P _ { c }$ 则由比力方程(7.4.7) ，无误差速度由下式确定

$$
\dot {\boldsymbol {V}} ^ {W} = \boldsymbol {f} ^ {W} - (2 \omega_ {i e} ^ {W} + \omega_ {e W} ^ {W}) \times \boldsymbol {V} ^ {W} + \boldsymbol {g} ^ {W}\tag{8.2.37}
$$

而实际系统总存在误差，系统输出的速度按下式解算

$$
\dot {\mathbf {V}} ^ {c} = \boldsymbol {f} ^ {c} - \left(2 \boldsymbol {\omega} _ {i e} ^ {c} + \boldsymbol {\omega} _ {e W} ^ {c}\right) \times \mathbf {V} ^ {c} + \mathbf {g} ^ {c}\tag{8.2.38}
$$

式中 v'=vW+aVW ωJ=ωI!+ðlωI! =ωI\~+ðco\~ gc = gW ag 0

设平台的姿态误差角为 ${ \boldsymbol { \phi } } ^ { W } = [ \phi _ { x } \quad \phi _ { y } \quad \phi _ { z } ] ^ { \mathrm { T } }$ ，加速度计的偏置量为

$$
\boldsymbol {\nabla} ^ {W} = \left[ \begin{array}{c c c} \nabla_ {x} & \nabla_ {y} & \nabla_ {z} \end{array} \right] ^ {\mathrm{T}}
$$

则由式 (8.2.38) ，得

$$
\begin{array}{r l} \dot {\boldsymbol {V}} ^ {W} + \delta \dot {\boldsymbol {V}} ^ {W} & = (\boldsymbol {I} + [ \delta \boldsymbol {K} _ {A} ]) (\boldsymbol {I} + [ \delta \boldsymbol {A} ]) \boldsymbol {C} _ {W} ^ {P} \boldsymbol {f} ^ {W} - [ 2 (\boldsymbol {\omega} _ {i e} ^ {W} + \delta \boldsymbol {\omega} _ {i e} ^ {W}) \\ & \quad + (\boldsymbol {\omega} _ {e W} ^ {W} + \delta \boldsymbol {\omega} _ {e W} ^ {W}) ] \times (\boldsymbol {V} ^ {W} + \delta \boldsymbol {V} ^ {W}) + \boldsymbol {g} ^ {W} + \delta \boldsymbol {g} ^ {W} + \nabla^ {W} \end{array}\tag{8.2.39}
$$

式中， $3 \times 3$ 的单位阵， $[ \delta A ]$ 为加速度计的安装误差阵， $\left[ \mathfrak { d } \mathbf { K } _ { \mathrm { A } } \right]$ 为刻度系数误差阵

式中

$$
\boldsymbol {C} _ {W} ^ {P} = \boldsymbol {I} + [ \boldsymbol {E} ]
$$

$$
[ \boldsymbol {E} ] = \left[ \begin{array}{c c c} 0 & \phi_ {z} & - \phi_ {y} \\ - \phi_ {z} & 0 & \phi_ {x} \\ \phi_ {y} & - \phi_ {x} & 0 \end{array} \right]\tag{8.2.40}
$$

为简化分析，假设

$$
\begin{array}{l} \left[ \delta \boldsymbol {A} \right] = \operatorname{diag} \left[ \begin{array}{c c c} \delta A _ {x} & \delta A _ {y} & \delta A _ {z} \end{array} \right] \\ \left[ \delta \boldsymbol {K} _ {A} \right] = \operatorname{diag} \left[ \begin{array}{c c c} \delta K _ {A x} & \delta K _ {A y} & \delta K _ {A z} \end{array} \right] \end{array}
$$

将式(8.2.39)与式(8.2.37)左右两边相减，略去关于误差的二阶小量，得

$$
\begin{array}{r l} \delta \dot {\boldsymbol {V}} ^ {W} = & ([ \delta \boldsymbol {A} ] + [ \delta \boldsymbol {K} _ {A} ] + [ \boldsymbol {E} ]) \boldsymbol {f} ^ {W} - (2 \boldsymbol {\omega} _ {i e} ^ {W} + \boldsymbol {\omega} _ {e W} ^ {\omega W}) \times \Delta \boldsymbol {V} ^ {W} \\ & - (2 \delta \boldsymbol {\omega} _ {i e} ^ {W} + \delta \boldsymbol {\omega} _ {e W} ^ {W}) \times \boldsymbol {V} ^ {W} + \delta \boldsymbol {g} ^ {W} + \nabla^ {W} \end{array}\tag{8.2.41}
$$

式中

$$
\boldsymbol {\omega} _ {i e} ^ {W} = \left[ \begin{array}{c} \omega_ {i e} \cos L \sin \alpha \\ \omega_ {i e} \cos L \cos \alpha \\ \omega_ {i e} \sin L \end{array} \right]\tag{8.2.42a}
$$

$$
\boldsymbol {\omega} _ {e W} ^ {W} = \left[ \begin{array}{c} - \frac {V _ {y} ^ {W}}{R} \\ \frac {V _ {x} ^ {W}}{R} \\ 0 \end{array} \right]\tag{8.2.42b}
$$

$$
\delta \boldsymbol {\omega} _ {i e} ^ {W} = \boldsymbol {\omega} _ {i e} ^ {c} - \boldsymbol {\omega} _ {i e} ^ {W} = \left[ \begin{array}{c} \omega_ {i e} \cos L ^ {c} \sin \alpha^ {c} \\ \omega_ {i e} \cos L ^ {c} \cos \alpha^ {c} \\ \omega_ {i e} \sin L ^ {c} \end{array} \right] - \left[ \begin{array}{c} \omega_ {i e} \cos L \sin \alpha \\ \omega_ {i e} \cos L \cos \alpha \\ \omega_ {i e} \sin L \end{array} \right] = \omega_ {i e} \left[ \begin{array}{c} \delta \alpha \cos L \cos \alpha - \delta L \sin L \sin \alpha \\ - \delta \alpha \cos L \sin \alpha - \delta L \sin L \cos \alpha \\ \delta L \cos L \end{array} \right]
$$

$$
\delta \boldsymbol {\omega} _ {e W} ^ {\boldsymbol {\omega} W} = \boldsymbol {\omega} _ {e W} ^ {c} - \boldsymbol {\omega} _ {e W} ^ {W} = \left[ \begin{array}{l} - \frac {V _ {y} ^ {c}}{R ^ {c}} \\ \frac {V _ {x} ^ {c}}{R} \\ 0 \end{array} \right] - \left[ \begin{array}{l} - \frac {V _ {y} ^ {W}}{R} \\ \frac {V _ {x} ^ {W}}{R} \\ 0 \end{array} \right] = \left[ \begin{array}{l} - \frac {V _ {y} ^ {W} + \Delta V _ {y}}{R + h + \Delta h} \\ \frac {V _ {x} ^ {W} + \Delta V _ {x}}{R + h + \Delta h} \\ 0 \end{array} \right] - \left[ \begin{array}{l} - \frac {V _ {y} ^ {W}}{R} \\ \frac {V _ {x} ^ {W}}{R} \\ 0 \end{array} \right]\tag{8.2.42c}
$$

按泰勒级数展开 $( R + h + \Delta h ) ^ { - 1 }$ 并取一次项，则上式为

$$
\delta \boldsymbol {\omega} _ {e W} ^ {W} = \left[ \begin{array}{c} - \frac {\delta V _ {y}}{R} + \frac {V _ {y} ^ {W}}{R ^ {2}} \delta h \\ \frac {\delta V _ {x}}{R} - \frac {V _ {x} ^ {W}}{R ^ {2}} \delta h \\ 0 \end{array} \right]\tag{8.2.42d}
$$

将式 (8.2.42)代入式 (8.2.41)

$$
\begin{array}{l} \left[ \begin{array}{c} \delta \dot {V} _ {x} \\ \delta \dot {V} _ {y} \\ \delta \dot {V} _ {z} \end{array} \right] = \left[ \begin{array}{c c c} \delta A _ {x} + \delta K _ {A x} & \phi_ {z} & - \phi_ {y} \\ - \phi_ {z} & \delta A _ {y} + \delta K _ {A y} & \phi_ {x} \\ \phi_ {y} & - \phi_ {x} & \delta A _ {z} + \delta K _ {A z} \end{array} \right] \left[ \begin{array}{c} f _ {x} ^ {W} \\ f _ {y} ^ {W} \\ f _ {z} ^ {W} \end{array} \right] \\ + \left[ \begin{array}{c c c} 0 & - \delta V _ {z} & \delta V _ {y} \\ \delta V _ {z} & 0 & - \delta V _ {x} \\ - \delta V _ {y} & \delta V _ {x} & 0 \end{array} \right] \left[ \begin{array}{c} 2 \omega_ {i e} \cos L \sin \alpha - \frac {V _ {y} ^ {W}}{R} \\ 2 \omega_ {i e} \cos L \cos \alpha + \frac {V _ {x} ^ {W}}{R} \\ 2 \omega_ {i e} \sin L \end{array} \right] + \left[ \begin{array}{c c c} 0 & - V _ {z} ^ {W} & V _ {y} ^ {W} \\ V _ {z} ^ {W} & 0 & - V _ {x} ^ {W} \\ - V _ {y} ^ {W} & V _ {x} ^ {W} & 0 \end{array} \right] \\ \times \left[ \begin{array}{c} 2 \delta \alpha \omega_ {i e} \cos L \cos \alpha - 2 \delta L \omega_ {i e} \sin L \sin \alpha - \frac {\delta V _ {y}}{R} + \frac {V _ {y} ^ {W}}{R ^ {2}} \delta h \\ - 2 \delta \alpha \omega_ {i e} \cos L \sin \alpha - 2 \delta L \omega_ {i e} \sin L \cos \alpha + \frac {\delta V _ {x}}{R} - \frac {V _ {x} ^ {W}}{R ^ {2}} \delta h \\ 2 \delta L \omega_ {i e} \cos L \end{array} \right] + \left[ \begin{array}{c} \delta g _ {x} \\ \delta g _ {y} \\ \delta g _ {z} \end{array} \right] + \left[ \begin{array}{c} \nabla_ {x} \\ \nabla_ {y} \\ \nabla_ {z} \end{array} \right] \end{array}
$$

展开上式，则得速度误差的分量方程

(8. 2. 43a)

$$
\begin{array}{l} \delta \dot {V} _ {x} = (\delta A _ {x} + \delta K _ {A x}) f _ {x} ^ {W} + \phi_ {z} f _ {y} ^ {W} - \phi_ {y} f _ {z} ^ {W} - \delta V _ {x} \frac {V _ {z} ^ {W}}{R} + \delta V _ {y} \cdot 2 \omega_ {i e} \sin L \\ \qquad - \delta V _ {z} \left(2 \omega_ {i e} \cos L \cos \alpha + \frac {V _ {x} ^ {W}}{R}\right) + \delta \alpha \cdot 2 \omega_ {i e} \cos L \sin \alpha V _ {z} ^ {W} \\ \qquad + \delta L \cdot 2 \omega_ {i e} (V _ {z} ^ {W} \sin L \cos \alpha + V _ {y} ^ {W} \cos L) + \delta h \frac {V _ {x} ^ {W} V _ {z} ^ {W}}{R ^ {2}} + \delta g _ {x} + \nabla_ {x} \\ \delta \dot {V} _ {y} = (\delta A _ {y} + \delta K _ {A y}) f _ {y} ^ {W} - \phi_ {z} f _ {x} ^ {W} + \phi_ {x} f _ {z} ^ {W} - \delta V _ {y} \frac {V _ {z} ^ {W}}{R} - \delta V _ {x} \cdot 2 \omega_ {i e} \sin L \\ \qquad + \delta V _ {z} \left(2 \omega_ {i e} \cos L \sin \alpha - \frac {V _ {y} ^ {W}}{R}\right) + \delta \alpha \cdot 2 \omega_ {i e} \cos L \cos \alpha V _ {z} ^ {W} \\ \qquad - \delta L \cdot 2 \omega_ {i e} (V _ {z} ^ {W} \sin L \sin \alpha + V _ {x} ^ {W} \cos L) + \delta h \frac {V _ {y} ^ {W} V _ {z} ^ {W}}{R ^ {2}} + \delta g _ {y} + \nabla_ {y} \\ \delta \dot {V} _ {z} = (\delta A _ {z} + \delta K _ {A z}) f _ {z} ^ {W} + \phi_ {y} f _ {x} ^ {W} - \phi_ {x} f _ {y} ^ {W} + \delta V _ {x} \cdot 2 \left(\omega_ {i e} \cos L \cos \alpha + \frac {V _ {x} ^ {W}}{R}\right) \\ \qquad - \delta V _ {y} \cdot 2 \left(\omega_ {i e} \cos L \sin \alpha - \frac {V _ {y} ^ {W}}{R}\right) - \delta \alpha \cdot 2 \omega_ {i e} \cos L (V _ {x} ^ {W} \sin \alpha + V _ {y} ^ {W} \cos \alpha) \\ \qquad + \delta L \cdot 2 \omega_ {i e} \sin L (V _ {y} ^ {W} \sin \alpha - V _ {x} ^ {W} \cos \alpha) - \delta h \frac {V _ {x} ^ {W ^ {2}} + V _ {y} ^ {W ^ {2}}}{R ^ {2}} + \delta g _ {z} + \nabla_ {z} \end{array}\tag{8.2.43b}
$$

(8.2. 43c)

## 2. 姿态误差方程

由于 $\phi$ 是实际平台系 相对理想平台系 的旋转角矢量，所以

$$
\dot {\boldsymbol {\phi}} = \boldsymbol {\omega} _ {T P}
$$

而 ${ \pmb { \omega } } _ { T P } = { \pmb { \omega } } _ { i P } - { \pmb { \omega } } _ { i T }$ • llP

$$
\dot {\boldsymbol {\phi}} ^ {P} = \boldsymbol {\omega} _ {i P} ^ {P} - \boldsymbol {C} _ {T} ^ {P} \boldsymbol {\omega} _ {i T} ^ {T}\tag{8.2.44}
$$

式中

$$
\pmb {\omega} _ {i P} ^ {P} = (\pmb {I} + \left[ \delta \pmb {K} _ {G} \right]) \pmb {\omega} _ {\mathrm{cmd}} ^ {P} + \pmb {\varepsilon} ^ {P}\tag{8.2.45}
$$

式中. $\cdot \pmb { \varepsilon } ^ { P }$ "为陀螺漂移 . $\pmb { \omega } _ { \mathrm { c m d } } ^ { P }$ 是指令角速度，如果导航坐标系 $n \left( \begin{array} { l l } \end{array} \right.$ 即为 系〕的实际角速度为 ${ \pmb { \omega } } _ { i T } ^ { T }$ ，而系统计算得的角速度为 ${ \pmb { \omega } } _ { i T } ^ { c }$ 则该计算值中既包含有 ${ \pmb { \omega } } _ { i T } ^ { T }$ ，又包含有误差 $\delta \pmb { \omega } _ { i T }$ ，即加入系统的指令角速度为

$$
\pmb {\omega} _ {\mathrm{cmd}} ^ {P} = \pmb {\omega} _ {i T} ^ {c} = \pmb {\omega} _ {i T} ^ {T} + \delta \pmb {\omega} _ {i T}\tag{8.2.46}
$$

将式 (8.2.46)代入式 (8.2.45)再代入式 (8.2.4纱，得

$$
\begin{array}{l} \dot {\boldsymbol {\phi}} ^ {P} = (\boldsymbol {I} + [ \delta \boldsymbol {K} _ {G} ]) (\boldsymbol {\omega} _ {i T} ^ {T} + \delta \boldsymbol {\omega} _ {i T}) + \varepsilon^ {P} - \boldsymbol {C} _ {T} ^ {P} \boldsymbol {\omega} _ {i T} ^ {T} \\ = \boldsymbol {\omega} _ {i T} ^ {T} + [ \delta \boldsymbol {K} _ {G} ] \boldsymbol {\omega} _ {i T} ^ {T} + \delta \boldsymbol {\omega} _ {i T} + \boldsymbol {\varepsilon} ^ {P} - (\boldsymbol {I} + [ \boldsymbol {E} ]) \boldsymbol {\omega} _ {i T} ^ {T} \\ = [ \delta \boldsymbol {K} _ {G} ] \boldsymbol {\omega} _ {i T} ^ {T} + \delta \boldsymbol {\omega} _ {i T} - [ \boldsymbol {E} ] \boldsymbol {\omega} _ {i T} ^ {T} + \boldsymbol {\varepsilon} ^ {P} \end{array}
$$

将式 (8.2.40) 和式 (8.2.42) 代入上式，得

$$
\begin{array}{r l} \left[ \begin{array}{c} \dot {\phi} _ {x} \\ \dot {\phi} _ {y} \\ \dot {\phi} _ {z} \end{array} \right] & = \left[ \begin{array}{c c c} \delta K _ {G x} & 0 & 0 \\ 0 & \delta K _ {G y} & 0 \\ 0 & 0 & \delta K _ {G z} \end{array} \right] \left[ \begin{array}{c} \omega_ {i e} \cos L \sin \alpha - \frac {V _ {y} ^ {W}}{R} \\ \omega_ {i e} \cos L \cos \alpha + \frac {V _ {x} ^ {W}}{R} \\ \omega_ {i e} \sin L \end{array} \right] \\ & + \left[ \begin{array}{c} \delta \alpha \omega_ {i e} \cos L \cos \alpha - \delta L \omega_ {i e} \sin L \sin \alpha - \frac {\delta V _ {y}}{R} + \frac {V _ {y} ^ {W}}{R ^ {2}} \delta h \\ - \delta \alpha \omega_ {i e} \cos L \sin \alpha - \delta L \omega_ {i e} \sin L \cos \alpha + \frac {\delta V _ {x}}{R} - \frac {V _ {x} ^ {W}}{R ^ {2}} \delta h \\ \delta h \omega_ {i e} \cos L \end{array} \right] \end{array}
$$

$$
- \left[ \begin{array}{c c c} 0 & \phi_ {z} & - \phi_ {y} \\ - \phi_ {z} & 0 & \phi_ {x} \\ \phi_ {y} & - \phi_ {x} & 0 \end{array} \right] \left[ \begin{array}{l} \omega_ {i e} \cos L \sin \alpha - \frac {V _ {y} ^ {W}}{R} \\ \omega_ {i e} \cos L \cos \alpha + \frac {V _ {x} ^ {W}}{R} \\ \omega_ {i e} \sin L \end{array} \right] + \left[ \begin{array}{l} \varepsilon_ {x} \\ \varepsilon_ {y} \\ \varepsilon_ {z} \end{array} \right]
$$

展开上式，得姿态误差方程=

$$
\begin{array}{r l} \dot {\phi} _ {x} = & \delta K _ {G x} \left(\omega_ {i e} \cos L \sin \alpha - \frac {V _ {y} ^ {W}}{R}\right) + \delta \alpha \omega_ {i e} \cos L \cos \alpha - \delta L \omega_ {i e} \sin L \sin \alpha \\ & - \frac {\delta V _ {y}}{R} + \frac {V _ {y} ^ {W}}{R ^ {2}} \delta h - \phi_ {z} \left(\omega_ {i e} \cos L \cos \alpha + \frac {V _ {x} ^ {W}}{R}\right) + \phi_ {y} \omega_ {i e} \sin L + \varepsilon_ {x} \end{array}\tag{8.2.47a}
$$

$$
\begin{array}{l} \dot {\phi} _ {y} = \delta K _ {G y} \left(\omega_ {i e} \cos L \cos \alpha + \frac {V _ {x} ^ {W}}{R}\right) - \delta \alpha \omega_ {i e} \cos L \sin \alpha - \delta L \omega_ {i e} \sin L \cos \alpha \\ + \frac {\delta V _ {x}}{R} - \frac {V _ {x} ^ {W}}{R ^ {2}} \delta h + \phi_ {z} \left(\omega_ {i e} \cos L \sin \alpha - \frac {V _ {y} ^ {W}}{R}\right) - \phi_ {x} \omega_ {i e} \sin L + \varepsilon_ {y} \end{array}\tag{8.2.47b}
$$

$$
\begin{array}{r l} \dot {\phi} _ {z} = & \delta K _ {G z} \omega_ {i e} \sin L + \delta L \omega_ {i e} \cos L - \phi_ {y} \left(\omega_ {i e} \cos L \sin \alpha - \frac {V _ {y} ^ {W}}{R}\right) \\ & + \phi_ {x} \left(\omega_ {i e} \cos L \cos \alpha + \frac {V _ {x} ^ {W}}{R}\right) + \varepsilon_ {z} \end{array}\tag{8.2.47c}
$$

## 3. 定位误差方程

由于 $\pmb { V } ^ { g } = \pmb { C } _ { W } ^ { g } \pmb { V } ^ { W }$ ，即

$$
\left[ \begin{array}{c} V _ {E} \\ V _ {N} \\ V _ {U} \end{array} \right] = \left[ \begin{array}{c c c} \cos \alpha & - \sin \alpha & 0 \\ \sin \alpha & \cos \alpha & 0 \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c} V _ {x} ^ {W} \\ V _ {y} ^ {W} \\ V _ {z} ^ {W} \end{array} \right] = \left[ \begin{array}{c} V _ {x} ^ {W} \cos \alpha - V _ {y} ^ {W} \sin \alpha \\ V _ {x} ^ {W} \sin \alpha + V _ {y} ^ {W} \cos \alpha \\ V _ {z} ^ {W} \end{array} \right]
$$

所以

$$
\dot {L} = \frac {V _ {N}}{R} = \frac {1}{R} \left(V _ {x} ^ {W} \sin \alpha + V _ {y} ^ {W} \cos \alpha\right)
$$

$$
\dot {\lambda} = \frac {V _ {E}}{R \cos L} = \frac {1}{R \cos L} (V _ {x} ^ {w} \cos \alpha - V _ {y} ^ {w} \sin \alpha)
$$

$$
\dot {h} = V _ {z}
$$

$$
\dot {\alpha} = - \frac {V _ {E}}{R} \tan L = \frac {1}{R} (V _ {y} ^ {W} \sin \alpha - V _ {x} ^ {W} \cos \alpha) \tan L
$$

上述诸式中，诸量用理论值与误差量之和代替之，取一阶全增量，得

$$
\delta \dot {L} = \frac {1}{R} \left(\delta V _ {x} \sin \alpha + \delta V _ {y} \cos \alpha + \delta \alpha V _ {E} - \delta h \frac {V _ {N}}{R}\right)\tag{8.2.48a}
$$

$$
\dot {\delta \lambda} = \frac {1}{R \cos L} \left(\delta V _ {x} \cos \alpha - \delta V _ {y} \sin \alpha - \delta \alpha V _ {N} + \delta L V _ {E} \tan L - \delta h \frac {V _ {E}}{R}\right)\tag{8.2.48b}
$$

$$
\dot {\delta h} = \delta V _ {z}\tag{8.2.48c}
$$

$$
\dot {\delta \alpha} = \frac {\tan L}{R} (\delta V _ {y} \sin \alpha - \delta V _ {x} \cos \alpha + \delta \alpha V _ {N}) + \delta L \frac {V _ {E}}{R \cos^ {2} L} + \delta h \frac {V _ {E} \tan L}{R ^ {2}}\tag{8.2.48d}
$$

式中

$$
V _ {E} = V _ {x} ^ {W} \cos \alpha - V _ {y} ^ {W} \sin \alpha , V _ {N} = V _ {x} ^ {W} \sin \alpha + V _ {y} ^ {W} \cos \alpha
$$

## 4. 航向误差

由式 (8. 1. 36) .航向角

$$
\Psi = \Psi_ {T b} - \alpha
$$

在实际系统中，航向由实测平台航向 $\Psi _ { P b }$ 和系统解算得的游移方位角 $\alpha _ { \iota }$ 确定:

![](images/fb407367ffaeecea67d807fc131bb2b97d05aaf61c4bf2ca4b391faacedc0e4f.jpg)  
8. 2. 4 $\psi _ { P b }$ 与 $\psi _ { T b }$ 间的关系

$$
\Psi^ {c} = \Psi_ {P b} - \alpha_ {c}
$$

式中， $, \alpha _ { c } = \alpha + \delta \alpha , \varPsi _ { P b }$ 与 $\Psi _ { T b }$ 间的关系如图 8.2.4 所示。由图 8.2.4 千导

$$
\Psi_ {P b} = \Psi_ {T b} + \phi_ {z}
$$

所以

$$
\begin{array}{r l} \delta \Psi & = \Psi^ {c} - \Psi \\ & = \Psi_ {T b} + \phi_ {z} - (\alpha + \delta \alpha) - (\Psi_ {T b} - \alpha) \\ & = \phi_ {z} - \delta \alpha \end{array} \tag {1}\tag{8.2.49}
$$

假设 $\phi _ { z }$ 和 $\delta \alpha$ 都是零均值的随机过程，且两者不相关，则航向误差的方差为

$$
\operatorname{var} [ \delta \Psi ] = E [ (\phi_ {z} - \delta \alpha) ^ {2} ] = \operatorname{var} [ \phi_ {z} ] + \operatorname{var} [ \delta \alpha ]\tag{8.2.50}
$$

## 8.3 平台式惯导系统的初始对准

在平台式惯导系统进人导航工作状态之前需要建立初始基准，如控制平台旋转使平台坐标系与导航坐标系 重合，计算出自由方位角或游移方位角等。这种为惯导系统建立初始基准的过程称为初始对准。初始对准使平台各轴的实际指向与要求指向间的偏差角不超过允许的误差范围，如水平误差不超过 $1 0 ^ { \prime \prime }$ 方位误差在 $2 ^ { \prime } { \sim } 5 ^ { \prime }$ 以内。实现初始对准的方法有:静基座上(运载体静止不动)的自主式对准，动基座上的传递对准，组合导航空中对准。此处只介绍静基座上的自主式对准，即利用自然参考量 $\pmb { \omega } _ { i e }$ 和 $\pmb { g }$ 由系统自行完成的对准。由于 ${ \pmb { \omega } } _ { i e } ^ { n }$ 在地球不同点是不相同的，所以自对准过程中，必须精确知道对准地点的纬度。

水平式平台惯导在初始对准之前先作环架锁定，即利用环架同步器输出直接驱动同轴上的力矩马达，使各轴接近互相正交，处于倾倒状态的平台被快速扶正。由于诸如飞机和舰船之类的运载体在停放时基本处于水平状态，所以扶正后的平台水平误差角在一定数值范围内，可视为小角，系统误差方程可视为线性的，这对简化对准过程是有利的。

## 8.3.1 指北方位系统的初始对准

指北方位系统的导航坐标系是地理坐标系，所以初始对准的目的是要控制平台旋转，使平台轴(由台体上的陀螺和加速度计的敏感轴确定)与地理坐标系的东、北、天指向重合。

指北方位系统的初始对准包括水平对准和方位对准两个过程。系统首先完成水平对准，此过程中仅系统的水平通道参与工作。水平对准结束后方位通道也参与工作，进行方位对准。

设对准地的纬度为 L(准确已知) ，忽略速度误差间的交叉耦合，则由式(8.2.27) ，得静基座条件下的系统误差方程为

$$
\delta \dot {V} _ {E} = - \phi_ {y} g + \nabla_ {E}\tag{8.3.1a}
$$

$$
\dot {\delta V} _ {N} = \phi_ {x} g + \nabla_ {N}\tag{8.3.1b}
$$

$$
\dot {\phi} _ {x} = - \frac {\delta V _ {N}}{R} + \phi_ {y} \omega_ {i e} \sin L - \phi_ {z} \omega_ {i e} \cos L + \varepsilon_ {E}\tag{8.3.1c}
$$

$$
\dot {\phi} _ {y} = \frac {\delta V _ {E}}{R} - \phi_ {x} \omega_ {i e} \sin L + \varepsilon_ {N}\tag{8.3.1d}
$$

$$
\dot {\phi} _ {z} = \frac {\delta V _ {E}}{R} \tan L + \phi_ {x} \omega_ {i e} \cos L + \varepsilon_ {U}\tag{8.3.1e}
$$

## 1.水平对准

快速扶正后，用水平加速度计输出控制横滚环电机和俯仰环电机，驱动平台使水平加速度计

的输出减小，这一过程即为水平粗对准，此时平台已接近水平。设平台的水平轴与东向和北向间的夹角为 如图 8. 3. 所示。此时$x$ 陀螺、和 陀螺都能感测到地球自转角速度的北向分量。设工陀螺和 陀螺的输出分别为 $\omega _ { x 0 }$ 和 $\omega _ { y 0 }$ 则

$$
\begin{array}{l} \omega_ {x 0} \approx \omega_ {i e} \cos L \sin K \\ \omega_ {y 0} \approx \omega_ {i e} \cos L \cos K \end{array}
$$

![](images/244a977e5489bea645847e9aae7d2bcab19896e596745f013edf1a822a926e3e.jpg)

所以 $K _ { \pm } \approx \arctan \frac { \omega _ { x 0 } } { \omega _ { y 0 } }$ ，按表 8. 3. 判定真值 后，对方位陀螺施图 8.3.1 平台的方位粗对准矩，驱动平台绕方位轴旋转，当旋转角位移达到 时，平台的方位失准角 $\phi _ { z }$ 是小角，这一过程即为方位粗对准。

8.3.1 方位失准角 的真值确定

<table><tr><td> $\omega_{\mathrm {r}0}$  $\omega_{\mathrm {y}0}$ </td><td>+</td><td>-</td></tr><tr><td>+</td><td> $K=K_{\text {主}}$ </td><td> $180^{\circ}+K_{\text {主}}$ </td></tr><tr><td>-</td><td> $360^{\circ}+K_{\text {主}}$ </td><td> $180^{\circ}+K_{\text {主}}$ </td></tr></table>

平台经水平粗对准和方位粗对准后，水平失准角和方位失准角都可视为小角， $\phi _ { x }$ 弘和 $\phi _ { y }$ 间的交叉耦合可忽略，此时水平通道的误差方程可简化为

$$
\delta \dot {V} _ {E} = - \phi_ {y} g + \nabla_ {E}\tag{8.3.2a}
$$

$$
\dot {\phi} _ {y} = \frac {\delta V _ {E}}{R} + \varepsilon_ {N}\tag{8.3.2b}
$$

$$
\delta \dot {V} _ {N} = \phi_ {x} g + \nabla_ {N}\tag{8.3.3a}
$$

$$
\dot {\phi} _ {x} = - \frac {\delta V _ {N}}{R} - \phi_ {z} \omega_ {i e} \cos L + \varepsilon_ {E}\tag{8.3.3b}
$$

按式 (8.3.2) 和式 (8.3.3) 可画出东向通道和北向通道误差方块图，如图 8.3.2和图 8.3.3 所示。

![](images/1ed876a9e0f349807d02f9f42179e7377f3b798229b24ff7680719d6e7d92c6a.jpg)  
8.3.2 指北系统东向通道误差方块图

![](images/ed69a5781496daea9df8f0024e2cd739b9abca0a173d38af59bfaa130c76d6ff.jpg)  
8.3.3 指北系统北向通道误差方块图

由图 8.3.2 和图 8. 3. 可看出，东向通道和北向通道实质上是体拉回路， $\cdot \phi _ { x }$ 和 $\phi _ { \cdot }$ y 做无阻尼振荡，且振荡周期为 84.4 分钟。为了引进阻尼，提高快速性和精度，分别引人虚线、点圃线和双点画线内反馈，如图 8.3.4 和图 8.3.5 所示。

$$
\frac {\phi_ {x} (s)}{\nabla_ {N} (s)} = \frac {\frac {1}{s + K _ {1}} \cdot \frac {1}{R} \cdot (- 1) \cdot \frac {1}{s}}{1 - \frac {1}{s + K _ {1}} \cdot \frac {1}{R} \cdot (- 1) \cdot \frac {1}{s} g} = - \frac {\frac {1}{R}}{s ^ {2} + K _ {1} s + \omega_ {s} ^ {2}}
$$

![](images/58ac08e5a063a3f55569325d28b9fbc678867ee96198d30147cc8a6d364b05a5.jpg)  
8.3.4 指北系统水平对准回路的东向通道

现以北向通道为例，分析水平对准回路的特性。

(1)根据图 8.3.5 ，当仅引人虚线所示 $K _ { 1 }$ 内反馈环节时，北向通道的特征多项式为

![](images/7f31f722b3e7621c6bd238c77d92f7a0294a99d3ff1348146eabe509bd72b4ac.jpg)  
8.3.5 指北系统水平对准回路的北向通道

$$
\Delta_ {1} (s) = s ^ {2} + K _ {1} s + \omega_ {s} ^ {2}
$$

阻尼比为 $\varepsilon { = } \frac { K _ { 1 } } { 2 \omega _ { s } }$ ，自振频率为 $\omega _ { n } { = } \omega _ { s }$ 0

可见系统虽然引人了阻尼比 $\xi , \phi _ { x }$ 能收敛，但收敛速度非常慢，84.4 分钟才完成一个周期的衰减，该回路因此称为二阶慢型水平对准回路。

(2) 引人 $K _ { 1 }$ 后，再接人 $\frac { K _ { 2 } } { R }$ 向画线顺馈回路，此时

$$
\frac {\phi_ {x} (s)}{\nabla_ {N} (s)} = \frac {\frac {1}{s + K _ {1}} \cdot \frac {1 + K _ {2}}{R} \cdot (- 1) \cdot \frac {1}{s}}{1 - \frac {1}{s + K _ {1}} \cdot \frac {1 + K _ {2}}{R} \cdot (- 1) \cdot \frac {1}{s} g} = - \frac {\frac {1 + K _ {2}}{R}}{s ^ {2} + K _ {1} s + (1 + K _ {2}) \omega_ {s} ^ {2}}
$$

系统的特征多项式为

$$
\Delta_ {2} (s) = s ^ {2} + K _ {1} s + \left(1 + K _ {2}\right) \omega_ {s} ^ {2}
$$

阻尼比为 $\xi = \frac { K _ { 1 } } { 2 \sqrt { 1 + K _ { 2 } } \omega _ { s } }$ ，自振频率 $\omega _ { n } = \sqrt { 1 + K _ { 2 } } \omega _ { s }$ ，适当选择 $K _ { 2 }$ 可加快收敛，该回路称为二阶快型水平对准回路。

现再分析对准精度。假设 $\nabla _ { N } \bullet _ { E }$ 和 $\phi _ { z }$ 均看作常值，则根据图 8. 3.

$$
\begin{array}{r l} \phi_ {x} (s) & = \frac {\left(\frac {\varepsilon_ {E}}{s} - \frac {\phi_ {z} \omega_ {i e} \cos L}{s}\right) \frac {1}{s} + \frac {\nabla_ {N}}{s} \cdot \frac {1}{s + K _ {1}} \cdot \frac {1 + K _ {2}}{R} \cdot (- 1) \cdot \frac {1}{s}}{1 - \frac {1}{s + K _ {1}} \cdot \frac {1 + K _ {2}}{R} \cdot (- 1) \cdot \frac {1}{s} g} \\ & = \frac {(s + K _ {1}) \left(\frac {\varepsilon_ {E}}{s} - \frac {\phi_ {z} \omega_ {i e} \cos L}{s}\right) - \frac {\nabla_ {N}}{s} \cdot (1 + K _ {2}) \omega_ {s} ^ {2} \cdot \frac {1}{g}}{s ^ {2} + K _ {1} s + (1 + K _ {2}) \omega_ {s} ^ {2}} \end{array}
$$

对准的稳态误差为

$$
\phi_ {x s s} = \lim _ {s \rightarrow 0} \phi_ {x} (s) = \frac {K _ {1}}{(1 + K _ {2}) \omega_ {s} ^ {2}} \left(\varepsilon_ {E} - \phi_ {z} \omega_ {i e} \cos L\right) - \frac {\nabla_ {N}}{g}
$$

(3) 为了消除 $\varepsilon _ { E }$ 和 $\phi _ { ; }$ 对 $\phi _ { x }$ 的影响，在引人 $K _ { 1 }$ 和 $\frac { K _ { 2 } } { R }$ 后再引人 $\cdot \frac { K _ { 3 } } { s }$ 顺馈回路，如图 8.3.5双点画线所示，此时

$$
\begin{array}{r l} \phi_ {x} (s) & = \frac {\left(\frac {\varepsilon_ {E}}{s} - \frac {\phi_ {z} \omega_ {i e} \cos L}{s}\right) \frac {1}{s} + \frac {\nabla_ {N}}{s} \cdot \frac {1}{s + K _ {1}} \cdot \left(\frac {1 + K _ {2}}{R} + \frac {K _ {3}}{s}\right) \cdot (- 1) \cdot \frac {1}{s}}{1 - \frac {1}{s + K _ {1}} \cdot \left(\frac {1 + K _ {2}}{R} + \frac {K _ {3}}{s}\right) \cdot (- 1) \cdot \frac {1}{s} g} \\ & = \frac {(s + K _ {1}) (\varepsilon_ {E} - \phi_ {z} \omega_ {i e} \cos L) - \nabla_ {N} \cdot \left(\frac {1 + K _ {2}}{R} + \frac {K _ {3}}{s}\right)}{s ^ {3} + K _ {1} s ^ {2} + (1 + K _ {2}) \omega_ {s} ^ {2} s + g K _ {3}} \end{array}
$$

系统的特征多项式为

$$
\Delta_ {3} (s) = s ^ {3} + K _ {1} s ^ {2} + \left(1 + K _ {2}\right) \omega_ {s} ^ {2} s + g K _ {3}\tag{8.3.4}
$$

所以该对准回路称为三阶水平对准回路， $\phi _ { x }$ 达到的稳态值，即对准精度为

$$
\phi_ {x s s} = \lim _ {s \rightarrow 0} s \phi_ {x} (s) = - \frac {\nabla_ {N}}{g}\tag{8.3.5}
$$

从式(8. 3. 5)知，三阶水平对准回路的对准精度不受 $K _ { 1 } \ 、 K _ { 2 } 、 K _ { 3 }$ 的影响，所以根据对准的收敛速度来选这些参数。

设根据快速性要求，要求对准回路的衰减系数为 $\sigma$ 阻尼自振频率为 $\omega _ { d }$ ，则三阶系统的特征根应为

$$
s _ {1} = - \sigma , \quad s _ {2} = - \sigma + \mathrm{j} \omega_ {d}, \quad s _ {3} = - \sigma - \mathrm{j} \omega_ {d}
$$

所以特征多项式为

$$
\begin{array}{r l} \Delta_ {3} (s) & = (s + \sigma) (s + \sigma - \mathrm{j} \omega_ {d}) (s + \sigma + \mathrm{j} \omega_ {d}) \\ & = s ^ {3} + 3 \sigma s ^ {2} + (3 \sigma^ {2} + \omega_ {d} ^ {2}) s + \sigma^ {3} + \sigma \omega_ {d} ^ {2} \end{array}\tag{8.3.6}
$$

比较式 (8. 3. 4) 和式 (8. 3. 6) 的系数，得

$$
K _ {1} = 3 \sigma , \quad (K _ {2} + 1) \omega_ {s} ^ {2} = 3 \sigma^ {2} + \omega_ {d} ^ {2}, \quad g K _ {3} = \sigma^ {3} + \sigma \omega_ {d} ^ {2}
$$

即

$$
K _ {1} = 3 \sigma , \quad K _ {2} = \frac {3 \sigma^ {2} + \omega_ {d} ^ {2}}{\omega_ {s} ^ {2}} - 1, \quad K _ {3} = \frac {\sigma^ {3} + \sigma \omega_ {d} ^ {2}}{g}
$$

若已知系统要求的阻尼比为 $\xi ,$ 衰减系数为 $\sigma \cdot$ 则

$$
K _ {1} = 3 \sigma , \quad K _ {2} = \frac {\sigma^ {2}}{\omega_ {s} ^ {2}} \left(2 + \frac {1}{\xi^ {2}}\right) - 1, \quad K _ {3} = \frac {\sigma^ {3}}{g \xi^ {2}}
$$

对东向通道的分析与上述分析方法是类似的，读者可自行进行。采用三阶水平对准回路时，$\phi _ { y }$ 达到的稳态值为

$$
\phi_ {y s s} = \frac {\nabla_ {E}}{g}\tag{8.3.7}
$$

(8. 3. 5)和式(8. 3. 7)给出了惯导系统的水平对准极限精度，可见水平对准精度取决于水平加速度计的精度。

## 2. 方位对准

1)罗经法对准

指北方位系统的方位对准是指控制平台绕方位轴旋转，使平台的 $y _ { P }$ 轴指向北.经过方位粗对准和水平精对准，水平失准角已达角秒级，方位失准角达到 左右。由于对准地的纬度准确已知，所以指令可精确计算得

$$
\boldsymbol {\omega} _ {\mathrm{emd}} = \boldsymbol {\omega} _ {i g} ^ {g} = \left[ \begin{array}{c} 0 \\ \omega_ {i e} \cos L \\ \omega_ {i e} \sin L \end{array} \right]
$$

忽略陀螺力矩器等存在的误差，假设平台完全跟随指令，则

$$
\boldsymbol {\omega} _ {i P} ^ {P} = \left[ \begin{array}{c} 0 \\ \omega_ {i e} \cos L \\ \omega_ {i e} \sin L \end{array} \right]
$$

不计 $\phi _ { x }$ 和 $\phi _ { : }$ 的影响

$$
\begin{array}{r l} \boldsymbol {\omega} _ {i P} ^ {g} = \boldsymbol {C} _ {P} ^ {g} \boldsymbol {\omega} _ {i P} ^ {P} & = \left[ \begin{array}{c c c} 1 & - \phi_ {z} & 0 \\ \phi_ {z} & 1 & 0 \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c} 0 \\ \omega_ {i e} \cos L \\ \omega_ {i e} \sin L \end{array} \right] = \left[ \begin{array}{c} - \phi_ {z} \omega_ {i e} \cos L \\ \omega_ {i e} \cos L \\ \omega_ {i e} \sin L \end{array} \right] \\ \boldsymbol {\omega} _ {g P} ^ {g} & = \boldsymbol {\omega} _ {i P} ^ {g} - \boldsymbol {\omega} _ {i g} ^ {g} = \left[ \begin{array}{c} - \phi_ {z} \omega_ {i e} \cos L \\ 0 \\ 0 \end{array} \right] \end{array}
$$

即

$$
\omega_ {g P x} ^ {g} = - \phi_ {z} \omega_ {i e} \cos L\tag{8.3.8}
$$

上式说明，由于存在方位失准角，平台相对当地水平面绕西向轴做向北倾斜转动，旋转角速度为$\phi _ { z } \omega _ { i e } \cos L$ ，此角速度称为罗经项。平台的向北倾斜转动使水平精对准确定的 $\phi _ { \mathscr { x } }$ 发生改变，北向加速度计感测这种倾斜变化，输出 $\phi _ { x } g$ ，经积分所得的 $\delta V _ { N }$ 中含有 $\phi$ 的信息。

所谓罗经法对准就是利用罗经项 $\phi _ { z } \omega _ { i e } \cos L$ 引起的 $\delta V _ { N }$ 用回路反馈的方法控制平台绕方位轴旋转，使 $\phi _ { z }$ 逐渐减小至极限值。图 8.3.6 为罗经法对准的原理方块图，其中 $K ( s )$ 是低通滤波器，由于1> $\phi _ { z } \omega _ { i e } \cos L$ 引起 $\phi _ { \cdot }$ 的变化十分缓慢 $\delta V _ { N }$ 的变化也很缓慢，有用信号的主要成分是低频信息，所以从 $\delta V _ { N }$ 中提取 $\varphi _ { z }$ 必须采用低通滤波器。将图 8.3.6 与图 8.3.5 作比较，可看出罗经对准回路实质上是北向通道二阶快型水平对准回路的改进。

由式(8. 3. 5)知，若采用三阶水平对准回路，则 $\delta V _ { N }$ 中不包含 $\phi$ 信息，所以罗经对准不能在三阶水平对准回路上进行。

由图 8.3. 6 \~嘻

$$
\begin{array}{r l} s \delta V _ {N} (s) & = g \phi_ {x} (s) + \nabla_ {N} (s) - K _ {1} \delta V _ {N} (s) + \delta V _ {N} (0) \\ s \phi_ {x} (s) & = - \frac {K _ {2} + 1}{R} \delta V _ {N} (s) - \phi_ {z} (s) \omega_ {i e} \cos L + \varepsilon_ {E} (s) + \phi_ {x} (0) \\ s \phi_ {z} (s) & = K (s) \delta V _ {N} (s) + \varepsilon_ {U} (s) + \phi_ {z} (0) \end{array}
$$

即

$$
\begin{array}{c} (s + K _ {1}) \delta V _ {N} (s) - g \phi_ {x} (s) = \nabla_ {N} (s) + \delta V _ {N} (0) \\ \frac {K _ {2} + 1}{R} \delta V _ {N} (s) + s \phi_ {x} (s) + \phi_ {z} (s) \omega_ {i e} \cos L = \varepsilon_ {E} (s) + \phi_ {x} (0) \\ - K (s) \delta V _ {N} (s) + s \phi_ {z} (s) = \varepsilon_ {U} (s) + \phi_ {z} (0) \end{array}
$$

写成矩阵形式

![](images/ffc4d87410e6256e46d1808f9a361a2448408366704463cdd16242aa604c7124.jpg)  
8.3.6 罗经法方位对准

$$
\left[ \begin{array}{c c c} s + K _ {1} & - g & 0 \\ \frac {K _ {2} + 1}{R} & s & \omega_ {i e} \cos L \\ - K (s) & 0 & s \end{array} \right] \left[ \begin{array}{c} \delta V _ {N} (s) \\ \phi_ {x} (s) \\ \phi_ {z} (s) \end{array} \right] = \left[ \begin{array}{c} \nabla_ {N} (s) + \delta V _ {N} (0) \\ \varepsilon_ {E} (s) + \phi_ {x} (0) \\ \varepsilon_ {U} (s) + \phi_ {z} (0) \end{array} \right]
$$

利用求逆公式，得

$$
\begin{array}{r l} \left[ \begin{array}{c} \delta V _ {N} (s) \\ \phi_ {x} (s) \\ \phi_ {z} (s) \end{array} \right] & = \frac {1}{\Delta^ {*} (s)} \left[ \begin{array}{c c c} s ^ {2} & g s & - g \omega_ {i e} \cos L \\ - \frac {K _ {2} + 1}{R} s - \omega_ {i e} \cos L K (s) & (s + K _ {1}) s & - (s + K _ {1}) \omega_ {i e} \cos L \\ s K (s) & g K (s) & s (s + K _ {1}) + \frac {K _ {2} + 1}{R} g \end{array} \right] \\ & \times \left[ \begin{array}{c} \nabla_ {N} (s) + \delta V _ {N} (0) \\ \varepsilon_ {E} (s) + \phi_ {x} (0) \\ \varepsilon_ {U} (s) + \phi_ {z} (0) \end{array} \right] \end{array}
$$

式中

$$
\Delta^ {*} (s) = \left| \begin{array}{c c c} s + K _ {1} & - g & 0 \\ \frac {K _ {2} + 1}{R} & s & \omega_ {i e} \cos L \\ - K (s) & 0 & s \end{array} \right| = s ^ {3} + K _ {1} s ^ {2} + \omega_ {s} ^ {2} \left(K _ {2} + 1\right) s + g \omega_ {i e} \cos L K (s)
$$

取

$$
K (s) = \frac {K _ {3}}{\omega_ {i e} \cos L (s + K _ {4})}\tag{8.3.9}
$$

则

$$
\Delta^ {*} (s) = s ^ {3} + K _ {1} s ^ {2} + \omega_ {s} ^ {2} (K _ {2} + 1) s + \frac {K _ {3}}{s + K _ {4}} g\tag{8.3.10}
$$

设' $\nabla _ { N } , \varepsilon _ { E } , \varepsilon _ { U }$ 均为常值量，则

$$
\begin{array}{r l} \phi_ {z} (s) & = \frac {s K (s)}{\Delta^ {*} (s)} \left[ \frac {\nabla_ {N} (s)}{s} + \delta V _ {N} (0) \right] + \frac {g K (s)}{\Delta^ {*} (s)} \left[ \frac {\varepsilon_ {E}}{s} + \phi_ {x} (0) \right] \\ & + \frac {s (s + K _ {1}) + \frac {K _ {2} + 1}{R} g}{\Delta^ {*} (s)} \left[ \frac {\varepsilon_ {U}}{s} + \phi_ {z} (0) \right] \end{array}
$$

将式(8.3.9)和式(8. 3. 10)代入上式，并求稳态值，得

$$
\phi_ {z s s} = \lim _ {s \rightarrow 0} \phi_ {z} (s) = \frac {\varepsilon_ {E}}{\omega_ {i e} \cos L} + \frac {(K _ {2} + 1) K _ {4}}{R K _ {3}} \varepsilon_ {U}\tag{8.3.11}
$$

由于方位陀螺漂移的影响可通过选择参数 $K _ { 2 } \ 、 K _ { 3 } \ 、 K _ { 4 }$ 来减小，所以罗经对准中，方位对准的极限精度为

$$
\phi_ {z \min} = \frac {\varepsilon_ {E}}{\omega_ {i e} \cos L}\tag{8.3.12}
$$

上式说明，要提高方位对准精度，就必须减小东向陀螺的漂移.

事实上，由图 8.3.6 知， $, \varepsilon E$ 和罗经项 $\phi _ { z } \omega _ { i e } \cos L$ L对北向通道的作用是等效的，当 $\varepsilon _ { E }$ 和世 $\phi _ { z } \omega _ { i \epsilon }$ cosL在数值上相等时

$$
\varepsilon_ {E} - \phi_ {z} \omega_ {i e} \cos L = 0\tag{8.3.13}
$$

即罗经项被 $\varepsilon _ { E }$ 抵消. $\delta V _ { N }$ 中反映不出 $\phi _ { z }$ 的信息，对准也就达到了稳态，由式(8.3.13)确定的方位失准角稳态值与式(8.3. 12)是相同的。

在罗经法方位对准中，北向通道是二阶快型水平对准回路，但回路中 $K _ { 1 }$ 和 $K _ { 2 }$ ，与水平精对准中的参数并不相同，必须另行确定。

设根据动态响应要求，要求阻尼比为\~ $\xi = \frac { \sqrt { 2 } } { 2 }$ .并设衰减系数为 $\sigma ,$ ，则阻尼振荡频率为 $\omega _ { d } = \sigma _ { \it : \mathrm { ~  ~ } }$ ，所以特征根为

$$
s _ {1, 2} = - \sigma - \mathrm{j} \sigma , \quad s _ {3, 4} = - \sigma + \mathrm{j} \sigma
$$

特征多项式为

$$
\Delta (s) = \left[ (s + \sigma + j \sigma) (s + \sigma - j \sigma) \right] ^ {2} = s ^ {4} + 4 \sigma s ^ {3} + 8 \sigma^ {2} s ^ {2} + 8 \sigma^ {3} s + 4 \sigma^ {4}\tag{8.3.14}
$$

又根据式(8.3.10) .方位对准回路的特征多项式为

$$
\begin{array}{l} \Delta (s) = s ^ {3} (s + K _ {4}) + K _ {1} s ^ {2} (s + K _ {4}) + \omega_ {s} ^ {2} (K _ {2} + 1) s (s + K _ {4}) + g K _ {3} \\ = s ^ {4} + (K _ {1} + K _ {4}) s ^ {3} + \left[ \omega_ {s} ^ {2} (K _ {2} + 1) + K _ {1} K _ {4} \right] s ^ {2} + \omega_ {s} ^ {2} (K _ {2} + 1) K _ {4} s + g K _ {3} \end{array}\tag{8.3.15}
$$

比较式 (8.3.14) 和式 (8.3.1日，得

$$
\left\{ \begin{array}{l} K _ {1} + K _ {4} = 4 \sigma \\ \omega_ {s} ^ {2} (K _ {2} + 1) + K _ {1} K _ {4} = 8 \sigma^ {2} \\ \omega_ {s} ^ {2} (K _ {2} + 1) K _ {4} = 8 \sigma^ {3} \\ g K _ {3} = 4 \sigma^ {4} \end{array} \right.
$$

这是关于未知参数的非线性代数方程组，且有多解性，选 $K _ { 1 } = K _ { 4 }$ ，则得唯一解

$$
K _ {1} = K _ {4} = 2 \sigma , \quad K _ {2} = \frac {4 \sigma^ {2}}{\omega_ {s} ^ {2}} - 1, \quad K _ {3} = \frac {4 \sigma^ {4}}{g}
$$

2) 陀螺测漂和计算法方位对准

经水平精对准后，将东向和北向通道都接成二阶'快型水平对准回路，如图 8. 3. 所示。

![](images/9ffa38d0b99e4b44c8ab05e4e2dac1fb060eab7bb66346dfac226ed6322494af.jpg)  
8.3.7 计算法方位对准及测漂

由图得

$$
\dot {\phi} _ {x} = - \frac {K _ {2} + 1}{R} \delta V _ {N} - \phi_ {z} \omega_ {i e} \cos L + \varepsilon_ {E}, \quad \dot {\phi} _ {y} = \frac {K _ {2} + 1}{R} \delta V _ {E} + \varepsilon_ {N}
$$

当回路达到稳、态后，即 $\dot { \phi } _ { x } = 0 , \dot { \phi } _ { y } = 0$

$$
\frac {K _ {2} + 1}{R} \delta V _ {N} = - \phi_ {z} \omega_ {i e} \cos L + \varepsilon_ {E}\tag{8.3.16a}
$$

$$
\frac {K _ {2} + 1}{R} \delta V _ {E} = - \varepsilon_ {N}\tag{8.3.16b}
$$

记

$$
\delta \omega_ {x} = \frac {K _ {2} + 1}{R} \delta V _ {N}, \quad \delta \omega_ {y} = \frac {K _ {2} + 1}{R} \delta V _ {E}
$$

显然 $\delta \omega _ { x }$ 是对 $_ { \mathcal { X } }$ 陀螺的错误控制信号， $\delta \omega _ { y }$ 是对 $y$ 陀螺的错误控制信号。 $\delta \omega _ { \boldsymbol { x } }$ 通过测量 陀螺力矩器中的施矩电流获得; $\delta \omega _ { y }$ 的获取方法为:测量 $y$ 陀螺力矩器中的施矩电流，用力矩器转换系数折算成角速度，再用该角速度减去 $\omega _ { i e } \cos L$ 。所以式 (8. 3. 16) 可写成:

$$
\begin{array}{l} \delta \omega_ {x} = - \phi_ {z} \omega_ {i e} \cos L + \varepsilon_ {E} \\ \delta \omega_ {y} = - \varepsilon_ {N} \end{array}\tag{8.3.17}
$$

即

$$
\varepsilon_ {N} = - \delta \omega_ {y}\tag{8.3.18}
$$

(8. 3. 18)即为北向陀螺的测漂公式。

对方位陀螺施矩，使方位环反时针旋转 $9 0 ^ { \circ }$ ，并将转动之前的位置称为第 位置，转动之后的位置称为第 II 位置，如图 8.3.8 所示。注意到在平台旋转过程中陀螺漂移不会改变，则在第置上，有:

$$
\delta \omega_ {x} ^ {\mathrm{I}} = - \phi_ {z} \omega_ {i e} \cos L + \varepsilon_ {E} ^ {\mathrm{I}}\tag{8.3.19a}
$$

$$
\varepsilon_ {y} = \varepsilon_ {N} ^ {\mathrm{I}} = - \delta \omega_ {y} ^ {\mathrm{I}}\tag{8.3.19b}
$$

在第II位置上，有:

$$
\delta \omega_ {y} ^ {\mathrm{II}} = - \phi_ {z} \omega_ {i e} \cos L + \varepsilon_ {E} ^ {\mathrm{II}}
$$

$$
\varepsilon_ {x} = \varepsilon_ {N} ^ {\mathrm{II}} = - \delta \omega_ {\mathrm{x}} ^ {\mathrm{II}}\tag{8.3.20a}
$$

(8.3.20b)

按式 (8. 3. 19b) 和式 (8. 3. 20b) 测出漂移 $\varepsilon _ { x }$ 和 $\varepsilon _ { y }$ 后，则由8.3.8 知，第 位置上的东向漂移即 陀螺的漂移，第置上的西向漂移即为 陀螺的漂移，即

![](images/6bd82f00526b143f075d6658b923e99a4bec7cfd7a444ad48d2801a67a7acd2d.jpg)

$$
\varepsilon_ {E} ^ {\mathrm{I}} = \varepsilon_ {x} = - \delta \omega_ {x} ^ {\mathrm{II}}\tag{8.3.21}
$$

$$
\varepsilon_ {E} ^ {\mathrm{II}} = - \varepsilon_ {y} = \delta \omega_ {y} ^ {\mathrm{I}}\tag{8.3.22}
$$

8. 3. 平台的

位置和第 位置

将式 (8. 3. 1)代入式 (8.3. 19a) ，得

$$
\phi_ {z} = \frac {\varepsilon_ {E} ^ {\mathrm{I}} - \delta \omega_ {x} ^ {\mathrm{I}}}{\omega_ {i e} \cos L} = - \frac {\delta \omega_ {x} ^ {\mathrm{I}} + \delta \omega_ {x} ^ {\mathrm{II}}}{\omega_ {i e} \cos L}\tag{8.3.23}
$$

若将式 (8.3. 22) 代入式 (8. 3. 20a) ，则可得 $\phi _ { z }$ 的另一个计算公式:

$$
\phi_ {z} = \frac {\varepsilon_ {E} ^ {\mathrm{II}} - \delta \omega_ {y} ^ {\mathrm{II}}}{\omega_ {i e} \cos L} = \frac {\delta \omega_ {y} ^ {\mathrm{I}} - \delta \omega_ {y} ^ {\mathrm{II}}}{\omega_ {i e} \cos L}\tag{8.3.24}
$$

按式 (8.3.23 )或式 (8.3.24 )计算出后 $\phi _ { z }$ 后，对方位陀螺一次施矩，施矩量对应$( 9 0 ^ { \circ } + \phi _ { z } )$ 使方位环顺时针旋转 $9 0 ^ { \circ } + \phi _ { z }$ ，既消除了 $\phi _ { z }$ ，又使 $\mathcal { X } P$ 轴指向东 $y _ { P }$ 轴指向北。

## 8.3.2 游移方位系统的初始对准

游移方位系统的水平对准与指北方位系统相似，也是靠物理方法将平台控制在当地水平面内。但方位对准与指北系统截然不同，指北系统方位对准是靠物理方法控制平台的 $y _ { P }$ 轴精确指北，而游移方位系统是靠精确确定出游移方位角 $\alpha$ 实现方位对准的。在对准过程中，指北系统先完成水平精对准，再作方位对准，而游移方位系统的水平对准和方位对准是交替进行的。

## 1.对准基本方程

## 1)姿态误差方程

与指北方位系统类似，游移方位系统在对准前先作环架锁定，再利用水平加速度计输出控制平台的水平轴伺服电机，实现快速模拟调平，使平台处于近似水平状态。设在 $t _ { 0 }$ 时刻开始系统作水平精对准，记与实际平台坐标系 对应的水平坐标系为理想、平台坐标系 $x _ { W }$ 和 $y _ { W }$ 分别是 $\boldsymbol { \mathcal { X } } \boldsymbol { P }$ 和 $y _ { P }$ 在水平面内的投影 ${ \mathcal { Z } } _ { W }$ 位于当地垂线，向上为正，并设系统的游移方位角为 ，即$x _ { W }$ 与 $x _ { g }$ 间的夹角为 $\alpha$ 。由于经环架锁定和水平粗对准后平台的水平失准角已经十分微小(角分，所以在 $t _ { 0 }$ 时刻以后实际平台坐标系 相对理想、平台坐标系 绕方位轴的偏差角可视为零，即

由此得 $\dot { \phi } _ { z }$ =0 ，即

$$
\phi_ {z} = 0\tag{8.3.25}
$$

$$
\omega_ {W P _ {z}} ^ {W} = 0
$$

上式也可写成

即

$$
\omega_ {i P z} ^ {W} - \omega_ {i W z} ^ {W} = 0
$$

$$
\omega_ {i W z} ^ {W} = \omega_ {i P z} ^ {W}\tag{8.3.26}
$$

上式说明，理想平台坐标系 在方位上跟随实际平台坐标系 $P$ o

此外，由于 $W$ 坐标系是水平坐标系，所以 坐标系的水平轴跟踪地球旋转角速度的水平分量，即

$$
\omega_ {i W x} ^ {W} = \omega_ {i e} \cos L \sin \alpha\tag{8.3.27a}
$$

$$
\omega_ {i W y} ^ {W} = \omega_ {i e} \cos L \cos \alpha\tag{8.3.27b}
$$

不计陀螺力矩器和伺服回路的误差，则实际平台系 $P$ 的角速度为

$$
\pmb {\omega} _ {i P} = \pmb {\omega} _ {\mathrm{cmd}} + \pmb {\varepsilon}\tag{8.3.28}
$$

式中， $, \pmb { \varepsilon }$ 为陀螺漂移。根据式(8. 1. 45)关于游移方位系统指令角速度计算公式，注意到在静基座条件下 $\mathcal { X }$ 通道和 $y$ 通道的速度输出为速度误差 $\delta V _ { x }$ 和 $\delta V _ { y }$ ，略去关于速度误差与扁率 $f$ 形成的工阶小量，得

$$
\left[ \begin{array}{l} \omega_ {\mathrm{cmd} x} ^ {P} \\ \omega_ {\mathrm{cmd} y} ^ {P} \\ \omega_ {\mathrm{cmd} z} ^ {P} \end{array} \right] = \left[ \begin{array}{c} \omega_ {i e} \cos L \sin \alpha_ {c} - \frac {\delta V _ {y}}{R} \\ \omega_ {i e} \cos L \cos \alpha_ {c} + \frac {\delta V _ {x}}{R} \\ \omega_ {i e} \sin L \end{array} \right]\tag{8.3.29}
$$

式中， $, \alpha _ { c }$ 为游移方位角 $\alpha$ 的估算值。上式代入式(8. 3. 28) ，得

$$
\left[ \begin{array}{c} \omega_ {i P x} ^ {P} \\ \omega_ {i P y} ^ {P} \\ \omega_ {i P z} ^ {P} \end{array} \right] = \left[ \begin{array}{c} \omega_ {i e} \cos L \sin \alpha_ {c} - \frac {\delta V _ {y}}{R} + \varepsilon_ {x} \\ \omega_ {i e} \cos L \cos \alpha_ {c} + \frac {\delta V _ {x}}{R} + \varepsilon_ {y} \\ \omega_ {i e} \sin L + \varepsilon_ {z} \end{array} \right]\tag{8.3.30}
$$

根据式 (8.3.25)

$$
\begin{array}{c} \boldsymbol {C} _ {W} ^ {P} = \left[ \begin{array}{c c c} 1 & 0 & - \phi_ {y} \\ 0 & 1 & \phi_ {x} \\ \phi_ {y} & - \phi_ {x} & 1 \end{array} \right] \\ \boldsymbol {\omega} _ {i P} ^ {W} = \boldsymbol {C} _ {P} ^ {W} \boldsymbol {\omega} _ {i P} ^ {P} \end{array}\tag{8.3.31}
$$

将式 (8.3.30) 和式 (8. 3. 31)代入上式，得

$$
\left[ \begin{array}{c} \omega_ {i P x} ^ {W} \\ \omega_ {i P y} ^ {W} \\ \omega_ {i P z} ^ {W} \end{array} \right] = \left[ \begin{array}{c c c} 1 & 0 & \phi_ {y} \\ 0 & 1 & - \phi_ {x} \\ - \phi_ {y} & \phi_ {x} & 1 \end{array} \right] \left[ \begin{array}{c} \omega_ {i e} \cos L \sin \alpha_ {c} - \frac {\delta V _ {y}}{R} + \varepsilon_ {x} \\ \omega_ {i e} \cos L \cos \alpha_ {c} + \frac {\delta V _ {x}}{R} + \varepsilon_ {y} \\ \omega_ {i e} \sin L + \varepsilon_ {z} \end{array} \right]
$$

略去关于误差的二阶小量，根据式(8. 3. 26) ，得

$$
\omega_ {i W z} ^ {W} = \omega_ {i P z} ^ {W} = \omega_ {i e} \sin L + \varepsilon_ {z} - \varphi_ {y} \omega_ {i e} \cos L \sin \alpha + \varphi_ {x} \omega_ {i e} \cos L \cos \alpha\tag{8.3.32}
$$

由于

$$
\dot {\boldsymbol {\phi}} ^ {P} = \boldsymbol {\omega} _ {i P} ^ {P} - \boldsymbol {C} _ {W} ^ {P} \boldsymbol {\omega} _ {i W} ^ {W}\tag{8.3.33}
$$

将式 (8.3.30) 、式 (8.3.27) 及式 (8.3.32) 代入上式，得

$$
\left[ \begin{array}{c} \dot {\phi} _ {x} \\ \dot {\phi} _ {y} \\ \dot {\phi} _ {z} \end{array} \right] = \left[ \begin{array}{c} \omega_ {i e} \cos L \sin \alpha_ {c} - \frac {\delta V _ {y}}{R} + \varepsilon_ {x} \\ \omega_ {i e} \cos L \cos \alpha_ {c} + \frac {\delta V _ {x}}{R} + \varepsilon_ {y} \\ \omega_ {i e} \sin L + \varepsilon_ {z} \end{array} \right] - \left[ \begin{array}{c c c} 1 & 0 & - \phi_ {y} \\ 0 & 1 & \phi_ {x} \\ \phi_ {y} & - \phi_ {x} & 1 \end{array} \right] \left[ \begin{array}{c} \omega_ {i W x} ^ {W} \\ \omega_ {i W y} ^ {W} \\ \omega_ {i W z} ^ {W} \end{array} \right]
$$

展开上式，略去关于误差的二阶小量，得用于对准的姿态误差方程:

$$
\dot {\phi} _ {x} = \omega_ {i e} \cos L \sin \alpha_ {c} - \frac {\delta V _ {y}}{R} + \varepsilon_ {x} - \omega_ {i e} \cos L \sin \alpha + \phi_ {y} \omega_ {i e} \sin L\tag{8.3.34a}
$$

$$
\dot {\phi} _ {y} = \omega_ {i e} \cos L \cos \alpha_ {c} + \frac {\delta V _ {x}}{R} + \varepsilon_ {y} - \omega_ {i e} \cos L \cos \alpha - \phi_ {x} \omega_ {i e} \sin L\tag{8.3.34b}
$$

$$
\begin{array}{r l} \dot {\phi} _ {z} & = \omega_ {i e} \sin L + \varepsilon_ {z} - (\omega_ {i e} \sin L + \varepsilon_ {z} - \phi_ {y} \omega_ {i e} \cos L \sin \alpha + \phi_ {x} \omega_ {i e} \cos L \cos \alpha) \\ & - \phi_ {y} \omega_ {i e} \cos L \sin \alpha + \phi_ {x} \omega_ {i e} \cos L \cos \alpha = 0 \end{array}\tag{8.3.34c}
$$

## 2) 速度误差方程

在静基座条件下， $V _ { x } ^ { W } = V _ { y } ^ { W } = V _ { z } ^ { W } = 0 , f _ { x } ^ { W } = f _ { y } ^ { W } = 0$ ，对准点纬度精确已知 $\delta L = 0 , f = - \thinspace g$ ，即$f _ { z } ^ { W } = g$ 略去通道间的交叉耦合，由式(8.2.43)可得用于对准的速度误差方程:

$$
\dot {\delta V} _ {x} = - \phi_ {y} g + \nabla_ {x}\tag{8.3.35a}
$$

$$
\dot {\delta V _ {y}} = \phi_ {x} g + \nabla_ {y}\tag{8.3.35b}
$$

## 2. 对准过程分析

根据式(8. 3. 34)和式(8. 3. 日，略去通道间的相合，可画出水平通道，如图 8.3.9 所示。图中， $, \theta _ { i e } \cos L \mathrm { s i n } \alpha$ 和 $\omega _ { i e } \cos L \mathrm { c o s } \alpha$ 是基座具有的角速度，是未知的， $\omega _ { i e } \cos L \sin \alpha _ { c }$ 和 $\omega _ { i e } \cos L \mathrm { c o s } \alpha _ { c }$ 是对陀螺的施矩量，如果两者相等，则平台跟踪地球旋转且保持水平， $\alpha { = } \alpha _ { c }$ ，即游移方位角可以从陀螺的施矩量中求取。但 $\alpha _ { c }$ 目前也是未知的，暂且假设 $\alpha _ { c } = 0 ($ 也可取其余任何值) ，此时采用工阶水平对准回路。如图 8. 3. 10 所示。

![](images/3d7fad79d075a2ae6b9c7975e6b19e5aad806cec52dae819fc1744c965543266.jpg)  
8.3.9 游移方位系统对准的水平通道

![](images/903ae0f5106b15b76a1609d9203aa99b8263363fe9342e631991b89cfc1c4c99.jpg)

![](images/1b8ef89684e1811575be379cde9846112b2e1dbe662c5403f0d5c7a8038d67a1.jpg)  
8. 3. 10 游移方位系统的二阶水平对准回路

水平对准回路达到稳态后， $\dot { \phi } _ { x } = 0 , \dot { \phi } _ { y } = 0$ ，在取 ${ \alpha _ { c } } = 0$ 的条件 $\sf { F }$ ，由图 8.3.10

$$
\frac {K _ {2} + 1}{R} \delta V _ {y} - \omega_ {i e} \cos L \sin \alpha + \varepsilon_ {x} = 0\tag{8.3.36a}
$$

$$
\frac {K _ {2} + 1}{R} \delta V _ {x} + \omega_ {i e} \cos L - \omega_ {i e} \cos L \cos \alpha + \varepsilon_ {y} = 0\tag{8.3.36b}
$$

式中

$$
\omega_ {c x} ^ {P} = - \frac {K _ {2} + 1}{R} \delta V _ {y}\tag{8.3.37a}
$$

$$
\omega_ {c y} ^ {P} = \frac {K _ {2} + 1}{R} \delta V _ {x} + \omega_ {i e} \cos L\tag{8.3.37b}
$$

是对水平陀螺的施矩量，可以通过测量工通道和 $y$ 通道速度输出 $\delta V _ { \mathcal { I } }$ 和 $\delta V _ { y }$ 以及计算值 $\omega _ { i e }$ cosL确定出。

由于式 (8.3. 36) 是在 $\alpha _ { c } = 0$ 条件下确定出的，施矩误差远大于陀螺漂移，所以略去陀螺漂移的影响，式(8. 3. 36)可近似为

$$
\omega_ {c x} ^ {P} \approx \omega_ {i e} \cos L \sin \alpha\tag{8.3.38a}
$$

$$
\omega_ {c y} ^ {P} \approx \omega_ {i e} \cos L \cos \alpha\tag{8.3.38b}
$$

从上式得

$$
\alpha_ {c 1} = \arctan \frac {\omega_ {c x} ^ {P}}{\omega_ {c y} ^ {P}}\tag{8.3.39}
$$

为了提高估算值 $\alpha _ { c 1 }$ 的可靠程度，减少测量误差的影响，可在一段时间内取平均值，根据(8. 3. 37) ，得

$$
\overline {{{{\omega}}}} _ {c x} ^ {P} = \frac {1}{n} \sum_ {i = 1} ^ {n} \omega_ {c x i} ^ {P} = - \frac {K _ {2} + 1}{R} \delta \overline {{{{V}}}} _ {y}
$$

$$
\bar {\omega} _ {c y} ^ {P} = \frac {1}{n} \sum_ {i = 1} ^ {n} \omega_ {c y i} ^ {P} = \frac {K _ {2} + 1}{R} \delta \overline {{V}} _ {x} + \omega_ {i e} \cos L
$$

式中

$$
\delta \overline {{{V}}} _ {x} = \frac {1}{n} \sum_ {i = 1} ^ {n} \delta V _ {x i}, \quad \delta \overline {{{V}}} _ {y} = \frac {1}{n} \sum_ {i = 1} ^ {n} \delta V _ {y i}
$$

由式 (8.3.38)

$$
\frac {(\overline {{\omega}} _ {c x} ^ {P}) ^ {2} + (\overline {{\omega}} _ {c y} ^ {P}) ^ {2}}{\omega_ {i e} ^ {2} \cos^ {2} L} \approx 1
$$

所以 $\alpha _ { c 1 }$ 的可信程度可通过下式检验

$$
0. 8 7 5 \leqslant \frac {\left(\frac {K _ {2} + 1}{R} \cdot \delta \bar {V} _ {y}\right) ^ {2} + \left(\omega_ {i e} \cos L + \frac {K _ {2} + 1}{R} \delta \bar {V} _ {x}\right) ^ {2}}{\omega_ {i e} ^ {2} \cos^ {2} L} \leqslant 1. 2 5\tag{8.3.40}
$$

若式 (8.3.40)成立，则按式 (8.3.39)计算出的 $\alpha _ { c 1 }$ 是可信的 若不成立，则 $\bar { \omega } _ { c x } ^ { P }$ 和 $\bar { \omega } _ { c y } ^ { P }$ 必须重新测量和计算，再按式(8. 3. 39)重新计算 $\alpha _ { c 1 }$ ，并按式 (8.3.40)重新检验，直至式 (8.3.40)成立。

若 $\alpha _ { c 1 }$ 经检验后可信，则 $\delta \alpha ^ { = } \alpha _ { c 1 } ^ { \phantom { } } - \alpha$ 已经很小，由图 8.3.10 ，得

$$
\begin{array}{l} \dot {\phi} _ {x} = - \frac {K _ {2} + 1}{R} \delta V _ {y} + \omega_ {i e} \cos L \sin \alpha_ {c} - \omega_ {i e} \cos L \sin \alpha + \varepsilon_ {x} \\ \dot {\phi} _ {y} = \frac {K _ {2} + 1}{R} \delta V _ {x} + \omega_ {i e} \cos L \cos \alpha_ {c} - \omega_ {i e} \cos L \cos \alpha + \varepsilon_ {y} \end{array}
$$

将 $\alpha { = } \alpha _ { c 1 } { - } \alpha$ 代入上式，其中 $\delta \alpha$ z是对 $\alpha _ { c 1 }$ 的修正量，并注意到 8α 是小角，则得

$$
\begin{array}{l} \dot {\phi} _ {x} = - \frac {K _ {2} + 1}{R} \delta V _ {y} + \delta \alpha \omega_ {i e} \cos L \cos \alpha_ {c 1} + \varepsilon_ {x} \\ \dot {\phi} _ {y} = \frac {K _ {2} + 1}{R} \delta V _ {x} - \delta \alpha \omega_ {i e} \cos L \sin \alpha_ {c 1} + \varepsilon_ {y} \end{array}
$$

水平回路达到稳态后，即 $\dot { \phi } _ { x } = 0 , \dot { \phi } _ { y } = 0$ ，上两式为

$$
\delta \alpha \omega_ {i e} \cos L \cos \alpha_ {c 1} = \frac {K _ {2} + 1}{R} \delta V _ {y} - \varepsilon_ {x}\tag{8.3.41a}
$$

$$
\delta \alpha \omega_ {i e} \cos L \sin \alpha_ {c 1} = \frac {K _ {2} + 1}{R} \delta V _ {x} + \varepsilon_ {y}\tag{8.3.41b}
$$

将式 (8. 3. 41a) $\mathrm { c o s } \alpha _ { c 1 }$ 、式 (8. 3. 41b) $\sin \alpha _ { c 1 }$ 后两式相加，得

$$
\delta \alpha = \frac {\left(\frac {K _ {2} + 1}{R} \delta V _ {y} - \varepsilon_ {x}\right) \cos \alpha_ {c 1} + \left(\frac {K _ {2} + 1}{R} \delta V _ {x} + \varepsilon_ {y}\right) \sin \alpha_ {c 1}}{\omega_ {i e} \cos L}\tag{8.3.42}
$$

忽略 $\varepsilon _ { x }$ 和 $\varepsilon _ { y }$ 的影响，得修正量计算值

$$
\delta \alpha_ {c} = \frac {K _ {2} + 1}{R} \cdot \frac {\delta V _ {y} \cos \alpha_ {c 1} + \delta V _ {x} \sin \alpha_ {c 1}}{\omega_ {i e} \cos L}\tag{8.3.43}
$$

为了增加计算得的 $\delta \alpha$ 的可靠性，对 $\delta V _ { \mathcal { x } }$ 和 $\delta V _ { y }$ 作 $n$ 次测量，按式(8.3.43)计算出 $\delta \alpha _ { i } , i = 1 , 2$ $3 , \cdots , n$ 再作平均

$$
\delta \bar {\alpha} = \frac {1}{n} \sum_ {i = 1} ^ {n} \delta \alpha_ {i}\tag{8.3.44}
$$

求得 $\delta \stackrel { - } { \alpha }$ 后，对 $\alpha _ { c 1 }$ 作修正:

$$
\hat {\alpha} _ {c} = \alpha_ {c 1} - \delta \bar {\alpha}\tag{8.3.45}
$$

按式 (8.3.43)确定的修正量 $\delta \alpha _ { c }$ 忽略了陀螺漂移的影响，由式(8.3.42)和式(8.3.43)知，由此引起的修正量误差为

$$
\begin{array}{r l} \delta (\delta \alpha) & = \delta \alpha_ {c} - \delta \alpha \\ & = \frac {K _ {2} + 1}{R} \cdot \frac {\delta V _ {y} \cos \alpha_ {c 1} + \delta V _ {x} \sin \alpha_ {c 1}}{\omega_ {i e} \cos L} \\ & - \frac {\left(\frac {K _ {2} + 1}{R} \delta V _ {y} - \varepsilon_ {x}\right) \cos \alpha_ {c 1} + \left(\frac {K _ {2} + 1}{R} \delta V _ {x} + \varepsilon_ {y}\right) \sin \alpha_ {c 1}}{\omega_ {i e} \cos L} \\ & = \frac {\varepsilon_ {x} \cos \alpha_ {c 1} - \varepsilon_ {y} \sin \alpha_ {c 1}}{\omega_ {i e} \cos L} \approx \frac {\varepsilon_ {x} \cos \alpha - \varepsilon_ {y} \sin \alpha}{\omega_ {i e} \cos L} = \frac {\varepsilon_ {E}}{\omega_ {i e} \cos L} \end{array}\tag{8.3.46}
$$

式中 $, \varepsilon _ { E } = \varepsilon _ { x } \cos \alpha - \varepsilon _ { y } \sin \alpha$ 陀螺、和 $y$ 陀螺、引起的等效东向漂移。

由于游移方位系统的方位对准是要确定出游移方位角 $\alpha \bullet$ ，而 $\alpha$ 是平台的 $x _ { P }$ 轴与东向轴间的夹角(亦 $y _ { P }$ 轴与北向轴间的夹角) ，理想、平台坐标系 $\mathcal { X } \boldsymbol { W }$ 轴和 $y _ { W }$ 轴是 $\mathcal { X } P$ 轴和 $y _ { P }$ 轴在当地水平面内的投影，所以 $\phi _ { z } = 0$ ，这说明，由式(8.3.46)确定的游移方位角修正量误差即为方位对准误差。比较式(8. 3. 46)和式(8. 3. 12) ，可看出游移方位系统和指北方位系统对准方法虽然不一样，但本质上是相同的，都应用了罗经效应。

## 3. 水平陀螺测漂

按式(8. 3. 45)对游移方位角估算值作修正后， $\hat { \mathbf { \Omega } } , \hat { \alpha } _ { c }$ 已达到一定的精度，此时式(8. 3. 1)可写成

$$
- \frac {K _ {2} + 1}{R} \delta V _ {y} + \varepsilon_ {x} = - \delta \alpha \omega_ {i e} \cos L \cos \hat {\alpha} _ {c}\tag{8.3.47a}
$$

$$
\frac {K _ {2} + 1}{R} \delta V _ {x} + \varepsilon_ {y} = \delta \alpha \omega_ {i e} \cos L \sin \hat {\alpha} _ {c}\tag{8.3.47b}
$$

将式 (8. 3. 47a) 两边同乘 $\dot { \mathrm { s i n } \alpha _ { c } }$ (8. 3. 47b) 两边同乘 $\mathrm { c o s } \hat { \alpha } _ { c }$ 之后两式相加，得

$$
\begin{array}{l} \varepsilon_ {x} \sin \hat {\alpha} _ {c} + \varepsilon_ {y} \cos \hat {\alpha} _ {c} \\ = \frac {K _ {2} + 1}{R} (\delta V _ {y} \sin \hat {\alpha} _ {c} - \delta V _ {x} \cos \hat {\alpha} _ {c}) \end{array}
$$

![](images/7d5e6432360edebecd46d8aa8b729edbb2d34091f78ae9085619a8e7505f40c8.jpg)  
8.3.11 陀螺的北向等效漂移

即

$$
\hat {\varepsilon} _ {N} = \frac {K _ {2} + 1}{R} (\delta V _ {y} \sin \hat {\alpha} _ {c} - \delta V _ {x} \cos \hat {\alpha} _ {c})
$$

式中， $, \hat { \epsilon } _ { N } = \epsilon _ { x } \sin \hat { \alpha _ { c } } + \varepsilon _ { y } \cos \hat { \alpha _ { \mathrm { c } } }$ 是由工陀螺和 $y$ 陀螺形成的等效北向漂移估算值，如图 8.3.11 所示。为提高测漂精度，可取平均值:

$$
\bar {\varepsilon} _ {N} = \frac {K _ {2} + 1}{R} \left(\sin \hat {\alpha} _ {c} \cdot \frac {1}{n} \sum_ {i = 1} ^ {n} \delta V _ {y i} - \cos \hat {\alpha} _ {c} \cdot \frac {1}{n} \sum_ {i = 1} ^ {n} \delta V _ {x i}\right)\tag{8.3.48}
$$

由图 8.3.11 ，若已求得 $\overline { { \varepsilon } } _ { N }$ '

$$
\hat {\varepsilon} _ {x} = \bar {\varepsilon} _ {N} \sin \hat {\alpha} _ {c}\tag{8.3.49a}
$$

$$
\hat {\varepsilon} _ {y} = \bar {\varepsilon} _ {N} \cos \hat {\alpha} _ {c}\tag{8.3.49b}
$$

(8.3.48)和式 (8.3.49)即为水平陀螺的测漂计算公式。求得 $\hat { \varepsilon } _ { x }$ 和 $\hat { \varepsilon } _ { y }$ 后，代入(8. 3. 42) ，可获得更精确的游移方位角修正量 $\mathfrak { F } \alpha$ ，从而根据式(8. 3. 45)可更精确地确定出游移方位角 $\hat { \alpha } _ { c }$ 0

为了较形象地说明游移方位系统初始对准的具体过程，图 8. 3. 12 示出了 LTN-7Z初始对准顺序示意图。

![](images/8069a9f3631798e175197e8b2dcc0e0dd25cf1aa50a4ba0e562af95c97c69983.jpg)  
8. 3. 12 LTN-72 初始对准顺序示意图

## 扩展导航参数解算和飞机转弯方式8.4

惯导系统输出的速度、位置、航向、姿态信息反映了飞机的瞬时飞行状态，通常将这些量称为导航参数。为了使飞机从起飞机场准确地飞向目的地，起飞前必须做好飞行任务计划的编制，即确定出中途飞越的航路点，由相邻航路点确定出地球大圆弧，即飞机的期望航迹。导航计算机根据惯导系统输出的位置信息实时解算出飞机偏离期望航迹的偏航距;同时，为了确保飞机沿期望航迹稳定飞行，还需要实时计算飞机的航迹角误差。经自动驾驶仪(或飞行员于动驾驶)操纵飞机使偏航距和航迹角误差保持为零，这样，飞机就能沿着期望的航线飞行。通常将偏航距、航迹角误差、偏流角、至下一个航路点的待飞距离和待飞时间等称为扩展导航参数或制导参数，下面简要介绍这些参数的解算方法，以及飞机从前一航线切换到下一航线的转弯方式。

## 8.4.1 扩展导航参数解算

1.用地理坐标表示的地垂线方向

![](images/da9c970814c39cc87d9780bea0d30ac444c533145f02870232929be7465fd140.jpg)  
8.4.1 点的地垂线

为地球上的某一点，该点的地理坐标为(L h) 。设 点的地垂线单位向量为 规定

地垂线向量向上为正(图 8.4. 1)。记 为地球坐标系， $g$ 为P点处的地理坐标系 $, x _ { g } , y _ { g } , z _ { g }$ 分别指向 点处的东、北、天。

$$
\begin{array}{c} \boldsymbol {C} _ {e} ^ {g} = \left[ \begin{array}{c c c} - \sin \lambda & \cos \lambda & 0 \\ - \sin L \cos \lambda & - \sin L \sin \lambda & \cos L \\ \cos L \cos \lambda & \cos L \sin \lambda & \sin L \end{array} \right] \\ \boldsymbol {C} _ {g} ^ {e} \boldsymbol {u} ^ {g} = \left[ \begin{array}{c c c} - \sin \lambda & - \sin L \cos \lambda & \cos L \cos \lambda \\ \cos \lambda & - \sin L \sin \lambda & \cos L \sin \lambda \\ 0 & \cos L & \sin L \end{array} \right] \left[ \begin{array}{l} 0 \\ 0 \\ 1 \end{array} \right] = \left[ \begin{array}{c} \cos L \cos \lambda \\ \cos L \sin \lambda \\ \sin L \end{array} \right] \end{array}\tag{8.4.1}
$$

## 2. 待飞距离和待飞时间解算

$$
(L, \lambda , h)
$$

一个航路点为 $P _ { i } , P _ { i }$ 点的地理坐标为 $( L _ { i } , \lambda _ { i } , h _ { i } )$ ，其中 $P ( L , \lambda , h )$ 由惯导系统提供 $P _ { i } ( L _ { i } , \lambda _ { i } , h _ { i } )$ 存储于计算机内。设 点和 $P _ { i }$ 点处地垂线方向的单位向量分别为 和 $\pmb { u } _ { i }$ ，则根据式 (8.4. 1)

$$
\boldsymbol {u} ^ {e} = \left[ \begin{array}{c} \cos L \cos \lambda \\ \cos L \sin \lambda \\ \sin L \end{array} \right]\tag{8.4.2}
$$

$$
\boldsymbol {u} _ {i} ^ {e} = \left[ \begin{array}{c} \cos L _ {i} \cos \lambda_ {i} \\ \cos L _ {i} \sin \lambda_ {i} \\ \sin L _ {i} \end{array} \right]\tag{8.4.3}
$$

由于

$$
\boldsymbol {u} \cdot \boldsymbol {u} _ {i} = \cos (\widehat {\boldsymbol {u} , \boldsymbol {u} _ {i}})
$$

式中， $( \pmb { u } , \pmb { u } _ { i } )$ 表示 $\pmb { u } _ { i }$ 间的夹角，所以，

$$
(\widehat {\boldsymbol {u} , \boldsymbol {u} _ {i}}) = \arccos (\boldsymbol {u} \cdot \boldsymbol {u} _ {i}) = \arccos (\boldsymbol {u} ^ {e} \cdot \boldsymbol {u} _ {i} ^ {e})
$$

$P$ 点至 $P _ { i }$ 点的待飞距离为

$$
\begin{array}{r l} D & = (R + h) \cdot (\widehat {\boldsymbol {u} ^ {e} \cdot \boldsymbol {u} _ {i} ^ {e}}) \\ & = (R + h) \arccos (\cos L \cos \lambda \cos L _ {i} \cos \lambda_ {i} \\ & \quad + \cos L \sin \lambda \cos L _ {i} \sin \lambda_ {i} + \sin L \sin L _ {i}) \end{array}\tag{8.4.4}
$$

点至 $P _ { i }$ 点的待飞时间为

$$
T _ {P P _ {i}} = \frac {D}{V}\tag{8.4.5}
$$

式中 为飞机的地速，由惯导系统提供。

## 3. 偏航距解算

设飞机自航路点 $P _ { i - 1 }$ 飞往下一个航路点 $P _ { i }$ 飞机的瞬时位置为 如图 8.4.2 所示，则大圆

$$
\widehat {P M}
$$

$\widehat { P _ { i - 1 } P _ { i } }$ 最短距离所确定的点。显然大圆弧 $\widehat { P M }$ 在M点的切线垂直于 $\pmb { u } _ { i - 1 }$ 和 $\pmb { u } _ { i }$ 所确定的平面。设沿该切线的单位向量为 ${ \pmb u } _ { ( i - 1 ) \times i } , P _ { i - 1 }$ 点和 $P _ { i }$ 点的地垂线单位向量分别为$\pmb { u } _ { i - 1 }$ 和 $\pmb { u } _ { i }$ '

$$
\boldsymbol {u} _ {(i - 1) \times i} = \frac {\boldsymbol {u} _ {i - 1} \times \boldsymbol {u} _ {i}}{| \boldsymbol {u} _ {i - 1} \times \boldsymbol {u} _ {i} |}
$$

![](images/2612a6c8054bf339c8ce7e0fd989bedf1a238148e982304627382604b1b1b20a.jpg)

由于 $\vec { \bf \Phi } ( { \pmb u } , { \pmb u } _ { M } ) = 9 0 ^ { \circ } - ( \pmb u , \pmb u _ { ( i - 1 ) \times i } )$ ，所以

$$
\begin{array}{r l} \sin (\boldsymbol {u}, \boldsymbol {u} _ {M}) & = \cos (\boldsymbol {u}, \boldsymbol {u} _ {(i - 1) \times i}) = \boldsymbol {u} ^ {e} \cdot \boldsymbol {u} _ {(i - 1) \times i} ^ {e} \\ (\widehat {\boldsymbol {u} , \boldsymbol {u} _ {M}}) & = \arcsin (\boldsymbol {u} ^ {e} \cdot \boldsymbol {u} _ {(i - 1) \times i} ^ {e}) \end{array}
$$

地心

偏航距为

$$
E = (R + h) \left(\widehat {\boldsymbol {u} , \boldsymbol {u} _ {M}}\right) = (R + h) \arcsin \left(\boldsymbol {u} ^ {e} \cdot \boldsymbol {u} _ {(i - 1) \times i} ^ {e}\right)\tag{8.4.6}
$$

式中 $\pmb { u } ^ { e }$ 根据式(8.4. 1)确定，

$$
\boldsymbol {u} _ {(i - 1) \times i} ^ {e} = \frac {\boldsymbol {u} _ {(i - 1)} ^ {e} \times \boldsymbol {u} _ {i} ^ {e}}{| \boldsymbol {u} _ {(i - 1)} ^ {e} \times \boldsymbol {u} _ {i} ^ {e} |}\tag{8.4.7}
$$

而

![](images/21c8d31538d845c1b5379ee4b99e36356d9a9ad5cea3baaf8fc5c83a2f64ed79.jpg)  
8.4.2 偏航距 $\widehat { P M }$

$$
\begin{array}{r l} \boldsymbol {u} _ {(i - 1)} ^ {e} \times \boldsymbol {u} _ {i} ^ {e} & = \left[ \begin{array}{c c c} 0 & - \sin L _ {i - 1} & \cos L _ {i - 1} \sin \lambda_ {i - 1} \\ \sin L _ {i - 1} & 0 & - \cos L _ {i - 1} \cos \lambda_ {i - 1} \\ - \cos L _ {i - 1} \sin \lambda_ {i - 1} & \cos L _ {i - 1} \cos \lambda_ {i - 1} & 0 \end{array} \right] \left[ \begin{array}{c} \cos L _ {i} \cos \lambda_ {i} \\ \cos L _ {i} \sin \lambda_ {i} \\ \sin L _ {i} \end{array} \right] \\ & = \left[ \begin{array}{c} - \sin L _ {i - 1} \cos L _ {i} \sin \lambda_ {i} + \cos L _ {i - 1} \sin \lambda_ {i - 1} \sin L _ {i} \\ \sin L _ {i - 1} \cos L _ {i} \cos \lambda_ {i} - \cos L _ {i - 1} \cos \lambda_ {i - 1} \sin L _ {i} \\ - \cos L _ {i - 1} \sin \lambda_ {i - 1} \cos L _ {i} \cos \lambda_ {i} + \cos L _ {i - 1} \cos \lambda_ {i - 1} \cos L _ {i} \sin \lambda_ {i} \end{array} \right] \end{array}\tag{8.4.8}
$$

记

$$
\begin{array}{l} U _ {(i - 1) \times i} ^ {x} = - \sin L _ {i - 1} \cos L _ {i} \sin \lambda_ {i} + \cos L _ {i - 1} \sin \lambda_ {i - 1} \sin L _ {i} \\ U _ {(i - 1) \times i} ^ {y} = \sin L _ {i - 1} \cos L _ {i} \cos \lambda_ {i} - \cos L _ {i - 1} \cos \lambda_ {i - 1} \sin L _ {i} \\ U _ {(i - 1) \times i} ^ {z} = - \cos L _ {i - 1} \sin \lambda_ {i - 1} \cos L _ {i} \cos \lambda_ {i} + \cos L _ {i - 1} \cos \lambda_ {i - 1} \cos L _ {i} \sin \lambda_ {i} \end{array}
$$

则

(8.4.9)

式中

$$
\boldsymbol {u} _ {(i - 1) \times i} ^ {e} = \left[ \begin{array}{l} \frac {U _ {(i - 1) \times i} ^ {x}}{U _ {(i - 1) \times i}} \\ \frac {U _ {(i - 1) \times i} ^ {y}}{U _ {(i - 1) \times i}} \\ \frac {U _ {(i - 1) \times i} ^ {z}}{U _ {(i - 1) \times i}} \end{array} \right]
$$

$$
U _ {(i - 1) \times i} = \left\{\left[ U _ {(i - 1) \times i} ^ {x} \right] ^ {2} + \left[ U _ {(i - 1) \times i} ^ {y} \right] ^ {2} + \left[ U _ {(i - 1) \times i} ^ {z} \right] ^ {2} \right\} ^ {\frac {1}{2}}\tag{8.4.10}
$$

4. 航迹角和航迹角误差解算

![](images/37c712ec4c20a7cc7658f651e225e7a273b38b62f0cb289ca94409c623ff045d.jpg)  
8.4.3 航迹角1J $\psi _ { s }$

点为飞机的瞬时位置 $P _ { i }$ 为下一个航路点 为地球北极 $\widehat { N P }$ 为过 点的子午圈 $\widehat { P P } _ { i }$

为过 点的大圆圈，即飞机所要执行的飞行航线 'P 点处飞机的期望航迹角，则 $\psi _ { s }$ 为子午圈 $\widehat { N P }$ 平面与大圆圆 $\overset { \widehat { P P } } { }$ 平面所夹的角，规定北偏东为正。两平面间的夹角也可表示为两平面法线间的夹角。

由图 8.4.3 ，子午面 NCP 的法线单位向量为

$$
\boldsymbol {u} _ {P \times e} ^ {e} = \frac {\boldsymbol {u} ^ {e} \times \boldsymbol {u} _ {e} ^ {e}}{| \boldsymbol {u} ^ {e} \times \boldsymbol {u} _ {e} ^ {e} |}
$$

大圆圈平面 $P C P _ { i }$ 的法线单位向量为

$$
\begin{array}{l} \boldsymbol {u} _ {P \times i} ^ {e} = \frac {\boldsymbol {u} ^ {e} \times \boldsymbol {u} _ {i} ^ {e}}{| \boldsymbol {u} ^ {e} \times \boldsymbol {u} _ {i} ^ {e} |} \\ \Psi_ {s} = (\widehat {\boldsymbol {u} _ {P \times i} ^ {e}}, \widehat {\boldsymbol {u} _ {P \times e} ^ {e}}) \end{array}
$$

为负。

规定当 ${ \pmb u } _ { P \times \epsilon }$ e至 ${ \pmb u } _ { P \times \mathrm { ~ ; ~ } }$ 的旋转方向指向地心时 $\psi$ 为正，背向地心时 $\Psi$

由于

$$
\boldsymbol {u} _ {P \times i} ^ {e} \cdot \boldsymbol {u} _ {P \times e} ^ {e} = \cos (\widehat {\boldsymbol {u} _ {P \times i} ^ {e} , \boldsymbol {u} _ {P \times e} ^ {e}}) = \cos \Psi_ {s}
$$

所以期望航迹角为

$$
\Psi_ {s} = \arccos (\boldsymbol {u} _ {P \times i} ^ {e} \cdot \boldsymbol {u} _ {P \times e} ^ {e})\tag{8.4.11}
$$

式中

$$
\boldsymbol {u} _ {P \times i} ^ {e} = \frac {\boldsymbol {U} _ {P \times i} ^ {e}}{| \boldsymbol {U} _ {P \times i} ^ {e} |}\tag{8.4.12}
$$

$$
\boldsymbol {u} _ {P \times e} ^ {e} = \frac {\boldsymbol {U} _ {P \times e} ^ {e}}{| \boldsymbol {U} _ {P \times e} ^ {e} |}\tag{8.4.13}
$$

$$
\begin{array}{r l} \boldsymbol {U} _ {P \times i} ^ {e} = \boldsymbol {u} ^ {e} \times \boldsymbol {u} _ {i} ^ {e} & = \left[ \begin{array}{c c c} 0 & - \sin L & \cos L \sin \lambda \\ \sin L & 0 & - \cos L \cos \lambda \\ - \cos L \sin \lambda & \cos L \cos \lambda & 0 \end{array} \right] \left[ \begin{array}{c} \cos L _ {i} \cos \lambda_ {i} \\ \cos L _ {i} \sin \lambda_ {i} \\ \sin L _ {i} \end{array} \right] \\ & = \left[ \begin{array}{c} - \sin L \cos L _ {i} \sin \lambda_ {i} + \cos L \sin \lambda \sin L _ {i} \\ \sin L \cos L _ {i} \cos \lambda_ {i} - \cos L \cos \lambda \sin L _ {i} \\ - \cos L \sin \lambda \cos L _ {i} \cos \lambda_ {i} + \cos L \cos \lambda \cos L _ {i} \sin \lambda_ {i} \end{array} \right] \end{array}\tag{8.4.14}
$$

$$
\boldsymbol {U} _ {P \times e} ^ {e} = \boldsymbol {u} ^ {e} \times \boldsymbol {u} _ {e} ^ {e} = - \boldsymbol {u} _ {e} ^ {e} \times \boldsymbol {u} ^ {e} = - \left[ \begin{array}{c c c} 0 & - 1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 0 \end{array} \right] \left[ \begin{array}{c} \cos L \cos \lambda \\ \cos L \sin \lambda \\ \sin L \end{array} \right] = \left[ \begin{array}{c} \cos L \sin \lambda \\ - \cos L \cos \lambda \\ 0 \end{array} \right]\tag{8.4.15}
$$

设飞机在 点处的瞬时东向速度和北向速度分别为 $V _ { E }$ 和 $V _ { N }$ 风速为 $\mathbf { V } _ { W }$ 则地速为

$$
\mathbf {V} = \mathbf {V} _ {T} + \mathbf {V} _ {W}
$$

式中 ${ \bf \nabla } , { \bf V } _ { T }$ 为真空速，方向沿飞机纵轴，由大气数据系统提供。 $V _ { T } , V _ { W } , V$ 间的关系如图 8.4.4示，图中?为飞机航向角， $\psi _ { s }$ 为飞机的期望航迹角 $\varPsi _ { s r }$ 为飞机的实际航迹角， 为偏流角，规定飞机右偏流时 为正，左偏流时 为负。由图得

$$
\boldsymbol {\Psi} _ {s r} = \arctan \frac {V _ {E}}{V _ {N}}\tag{8.4.16}
$$

由式 (8.4. 11) (8.4. 16) ，航迹角误差为

$$
\begin{array}{l} \Delta \boldsymbol {\Psi} _ {s} = \boldsymbol {\Psi} _ {s r} - \boldsymbol {\Psi} _ {s} \\ = \arctan \frac {V _ {E}}{V _ {N}} - \arccos (\boldsymbol {u} _ {P \times i} ^ {e} \cdot \boldsymbol {u} _ {P \times e} ^ {e}) \end{array}\tag{8.4.17}
$$

显然，当 $\Delta \pmb { \Psi } _ { s } > 0$ 时，飞机右偏航 $\Delta \pmb { \Psi } _ { s } < 0$ 时，飞机左偏航。

又由图得风速的东向和北向分量为

$$
V _ {W E} = V _ {E} - V _ {T} \sin \boldsymbol {\Psi}
$$

![](images/fa4c2f4d76cb7259934c0f37ff3cb7bd37cd0dc88397d459c97fd6f3c504f629.jpg)  
8.4.4 航迹角误差6.1J'，和偏流角

$$
V _ {W N} = V _ {N} - V _ {T} \cos \boldsymbol {\Psi}
$$

所以风向(风速与北向的夹角)的主值为

$$
\theta_ {W \text {主}} = \arctan \frac {V _ {W E}}{V _ {W N}} = \arctan \frac {V _ {E} - V _ {T} \sin \Psi}{V _ {N} - V _ {T} \cos \Psi}\tag{8.4.18}
$$

风向真值由表 8.4.1 确定。

8.4.1 凤向真值 ${ \pmb \theta } _ { W }$ 的确定

<table><tr><td> ${V}_{\mathrm{N}}$  ${V}_{\mathrm{E}}$ </td><td>&lt;0</td><td>=0</td><td>&gt;0</td></tr><tr><td>&lt;0</td><td> ${270}^{ \circ } - {\theta }_{W\text{主 }}$ </td><td> ${180}^{ \circ }$ </td><td> ${180}^{ \circ } + {\theta }_{W\text{主 }}$ </td></tr><tr><td>=0</td><td> ${270}^{ \circ }$ </td><td>无风</td><td> ${90}^{ \circ }$ </td></tr><tr><td>&gt;0</td><td> ${360}^{ \circ } + {\theta }_{W\text{主 }}$ </td><td> ${0}^{ \circ }$ </td><td> ${\theta }_{W\text{主 }}$ </td></tr></table>

## 8.4.2 飞机转弯方式

设 $P _ { i - 1 } \lor P _ { i } \land P _ { i + 1 }$ 为主个顺序航路点，当飞机飞至 $P _ { i }$ 点时，应由沿 $\widehat { P _ { i - 1 } P _ { i } }$ 大圆圈航线飞行改向沿 $\cdot P _ { i } P _ { i + 1 }$ 大圆圆航线飞行，此时飞机必须转弯，在飞行任务计划制订中，有多种转弯方式供选择以完成航线切换。

## 1.压点转弯

压点转弯是指飞机飞至航路点 $P _ { i }$ 上空才开始转弯，转弯时飞机按照预定的转弯坡度(倾斜角)和地速做圆弧飞行。当转弯圆弧与飞机至下一个目标航路点 $P _ { i + 1 }$ 的连线相切时，转弯结束。航段长为转弯圆弧长与转弯结束点到下一个航路点 $P _ { i + }$ 距离之和，即航线长度= $S + L _ { c }$ 由于飞机到达航路点 $P _ { i }$ 后才开始转弯，所以压点转弯也称到点转弯，如图 8.4.5 所示。轰炸机、强击机、歼击机和运输机通常采用压点转弯航线。

## 2. 向点车专弯

向点转弯是指飞机临近目标航路点 $P _ { i }$ 之前就按预定的转弯坡度和地速做转弯，到达 $P _ { i }$ 点上空时转弯就结束。航段长为转弯圆弧长与转弯起始点到上一个目标航路点 $P _ { i ^ { - } }$ 距离之和，即航线长度二 $S { + } L _ { \circ }$ 如图 8.4.6 所示。轰炸机、强击机、歼击机和运输机有时采用向点转弯航线。

![](images/263829f832b4a550d5d13e60c8bcfcbdb384c4137083e6969387aeda543c0cbd.jpg)  
8.4.5 压点转弯航线

![](images/72b049bd9aaf84af13681402c27a5ddb8600fb9ac31e0d5a9cfb57954a052e2f.jpg)  
8.4.6 向点转弯航线

## 3. 切点转弯

切点转弯是指飞机临近目标航路点 $P _ { i }$ 之前就按预定的转弯坡度和地速做转弯，当转弯圆弧与航路点 $P _ { i } \setminus P _ { i + }$ 的连线相切时转弯结束。航段长为转弯圆弧长与转弯结束点到下一个目标航路点 $P _ { i + }$ 距离之和，即:航线长度=5 。如图 8.4.7 所示。轰炸机、强击机、歼击机和运输机有时采用切点转弯航线。

## 4. 绕点转弯

绕点转弯以目标航路点 $P _ { i }$ 为圆心，当飞行航段与由预定的转弯坡度和地速确定的转弯半径垂直时，飞机做圆弧转弯，当转弯圆弧与飞机至下一个目标航路点 $P _ { i + 1 }$ 的连线相切时转弯结束。航段长为转弯圆弧长与转弯结束点至下一个目标航路点 $P _ { i + }$ 距离之和，即:航线长度=5+L8.4.8 所示。做绕点转弯时始终能看到转弯点(航路点 $P _ { i } )$ 所以适合于前向下视视界较差的飞机。

![](images/d47fc143da5a74422a2ac9ca4357267e995c0fb5b9bd8f83da77ee8928db2a26.jpg)  
8.4.7 切点转弯航线

![](images/73144763601e174ba6ae193eec751b9da1c95b22e5f0a27ef567c0c516fb0bb8.jpg)  
8.4.8 绕点转弯航线

## 5. 混合转弯

混合转弯是指由两种以上转弯方式组合而成的转弯。轰炸机、强击机、歼击机和运输机有时采用混合转弯航线。

## 8. 5 惯导定位等概率误差椭圆与散布圆

RNP Required Navigation Performance 的缩写，意即导航性能要求。飞机在飞行过程中，真实位置是无法确切知道的，惯导系统给出的指示位置或多或少包含误差。飞机配备的惯导系统精度越高，飞机的真实位置相对指示位置的偏离值就越小，否则偏离值就越大，飞机真实位置的不确定性就越大。有些场合对飞机真实飞行状态有非常严格的要求，如允许的空中走廊很窄，进场着陆时，飞机的离地高度、距着陆门界的距离、下滑角、速度等的允许偏差必须小于某一指标，这就必须对惯导系统提出可容忍的最低精度下限。此处研究:若要求飞机的真实位置以某指定概率落在惯导指示位置附近的某 范围内，惯导系统定位误差的最大允许值究竟是多少。

## 8.5.1 飞机真实位置相对指示位置的偏离值与惯导系统定位误差的关系

设飞机在飞行过程中的真实经、纬度分别为 Lr 惯导系统给出的指示经、纬度为 λc$L ^ { c }$ 则飞机真实位置相对指示位置的偏离值为

$$
\begin{array}{l} \mathrm{d} L = L ^ {r} - L ^ {c} \\ \mathrm{d} \lambda = \lambda^ {r} - \lambda^ {c} \end{array}
$$

而惯导系统的定位误差为

$$
\begin{array}{l} \delta L = L ^ {c} - L ^ {r} \\ \delta \lambda = \lambda^ {c} - \lambda^ {r} \end{array}
$$

所以

$$
\mathrm{d} L = - \delta L\tag{8.5.1a}
$$

$$
\mathrm{d} \lambda = - \delta \lambda\tag{8.5.1b}
$$

假设'惯导系统的定位误差悦和 òÀ 服从零均值正态分布，方差分别为 $\sigma _ { L } ^ { 2 }$ 和 $\sigma _ { \lambda } ^ { 2 }$ ，则根据式(8. 5. 1)知，也和d).也服从零均值正态分布，方差分别为

$$
\begin{array}{r l} & D (\mathrm{d} L) = E (\mathrm{d} L ^ {2}) = E (\delta L ^ {2}) = \sigma_ {L} ^ {2} \\ & D (\mathrm{d} \lambda) = E (\mathrm{d} \lambda^ {2}) = E (\delta \lambda^ {2}) = \sigma_ {\lambda} ^ {2} \end{array}
$$

所以dL和出的概率密度函数为

$$
p (\mathrm{d} L) = \frac {1}{\sqrt {2 \pi} \sigma_ {L}} \exp \left(- \frac {\mathrm{d} L ^ {2}}{2 \sigma_ {L} ^ {2}}\right) = \frac {1}{\sqrt {2 \pi} \sigma_ {L}} \exp \left(- \frac {\delta L ^ {2}}{2 \sigma_ {L} ^ {2}}\right) = p (\delta L)\tag{8.5.2a}
$$

$$
p (\mathrm{d} \lambda) = \frac {1}{\sqrt {2 \pi} \sigma_ {\lambda}} \exp \left(- \frac {\mathrm{d} \lambda^ {2}}{2 \sigma_ {\lambda} ^ {2}}\right) = \frac {1}{\sqrt {2 \pi} \sigma_ {\lambda}} \exp \left(- \frac {\delta \lambda^ {2}}{2 \sigma_ {\lambda} ^ {2}}\right) = p (\delta \lambda)\tag{8.5.2b}
$$

(8. 5. 2)表明，研究dL、心的分布情况与研究乱、拟的分布情况是等价的。在下述叙述中，均以惯导系统的导航误差乱和讪为研究对象

此处假设惯导系统的定位误差均 的原因 :定位误差的非零均值实 是惯导系统的系统误差，即系统存在的偏置误差，一般情况下，这种误差是可以补偿的。

## 8.5.2 等概率定位误差方程

飞机在飞行过程中是用地理坐标，即经度、纬度和高度定位的。对于 维定位，飞机的位置实际上是子午线与纬度线的交点，子午线和纬度线可看作夹角为$9 0 ^ { \circ }$ 的位置线，如图 8. .1 所示。

设飞机的真实位置 点，而惯导系统给出的指示位置为M $M ^ { \prime }$ ，见图 8.5.1 ，则悦和 òÀ 即为定位误差。

假设定位误差服从零均值的正态分布，则悦和 òÀ的概 密度函数可

![](images/33d8db5abc3ab6607b001d96dc889d3468c825c8658ca0421866772ac620b206.jpg)  
8.5 经度和纬度确定的位置线和位置

$$
p (\delta L) = \frac {1}{\sqrt {2 \pi} \sigma_ {L}} \exp \left(- \frac {\delta L ^ {2}}{2 \sigma_ {L} ^ {2}}\right)\tag{8.5.3a}
$$

$$
p (\delta \lambda) = \frac {1}{\sqrt {2 \pi} \sigma_ {\lambda}} \exp \left(- \frac {\delta \lambda^ {2}}{2 \sigma_ {\lambda} ^ {2}}\right)\tag{8.5.3b}
$$

'6 和'6，1.的联合分布概率密度函数为

$$
p (\delta L, \delta \lambda) = \frac {1}{2 \pi \sigma_ {L} \sigma_ {\lambda} \sqrt {1 - \rho_ {r} ^ {2} \sigma_ {L} \sigma_ {\lambda}}} \exp \left[ - \frac {1}{2 (1 - \rho_ {r} ^ {2})} \left(\frac {\delta L ^ {2}}{\sigma_ {L} ^ {2}} + \frac {\delta \lambda^ {2}}{\sigma_ {\lambda} ^ {2}} - \frac {2 \rho_ {r} \delta L \delta \lambda}{\sigma_ {L} \sigma_ {\lambda}}\right) \right]\tag{8.5.4}
$$

![](images/dc060706337d72137bb49b90a2d2c739935421c29d4adc533352708aab872564.jpg)  
8.5.2 aL 和似的联合分布概 密度函数

式中， ${ \bf , } \rho _ { r }$ 为'6 和拟 的相关系数。对于常用导航系统，如惯导系统、卫星导航系统、罗兰无线电导航系统，经、纬度误差是不相关的，即有 ${ \rho _ { r } } = 0$ ，所以对常用惯导系统，式 (8.5.4)写成

$$
p (\delta L, \delta \lambda) = \frac {1}{2 \pi \sigma_ {L} \sigma_ {\lambda}} \exp \left[ - \frac {1}{2} \left(\frac {\delta L ^ {2}}{\sigma_ {L} ^ {2}} + \frac {\delta \lambda^ {2}}{\sigma_ {\lambda} ^ {2}}\right) \right]\tag{8.5.5}
$$

8.5.2 是式 (8.5.5) 的立体示意图。由式(8. 5. 5) 知，当

$$
\frac {1}{2} \left(\frac {\delta L ^ {2}}{\sigma_ {L} ^ {2}} + \frac {\delta \lambda^ {2}}{\sigma_ {\lambda} ^ {2}}\right) = K ^ {2}\tag{8.5.6}
$$

其中 为某一常值，则此时的概率密度为常值， 此称满足式(8 .6)方程的曲 为等概率误差曲线，如图 8. (L，，1.)平面内的虚线曲线所示。式(8. 5. 称为 误差椭圆方程。

## 8.5.3 等概率误差椭圆分析

根据式 (8.5 肘，等概率误差椭圆方程写成

$$
\frac {x ^ {2}}{2 K ^ {2} \sigma_ {x} ^ {2}} + \frac {y ^ {2}}{2 K ^ {2} \sigma_ {y} ^ {2}} = 1\tag{8.5.7}
$$

式中 $x = R \mathrm { c o s } L \mathbb { \partial } \lambda , y = R \mathbb { \partial } L , \sigma _ { x } = R \mathrm { c o s } L \sigma _ { \lambda } , \sigma _ { y } = R \sigma _ { L }$ 为飞机所在点的纬度 为地球半径。

由于地球上任一点处的 午线与纬度线互相垂 ，所 所在坐标系为直角坐标系(8. 5. 7)为标准型椭圆方程，该方程所在坐标系原点为 坐标轴沿 处的子午线标轴沿 点处的纬度线， 长轴 半短 确定如下

若 $R \sigma _ { L } { > } R \mathrm { c o s } L \sigma _ { \lambda }$ ，则

$$
\left\{ \begin{array}{l} a = \sqrt {2} K R \sigma_ {L} \\ b = \sqrt {2} K R \cos L \sigma_ {\lambda} \end{array} \right.\tag{8.5.8a}
$$

长轴位 子午线， 轴位 纬度线

若 $R \sigma _ { L } { < } R \mathrm { c o s } L \sigma _ { \lambda }$ ，则

$$
\left\{ \begin{array}{l} a = \sqrt {2} K R \cos L \sigma_ {\lambda} \\ b = \sqrt {2} K R \sigma_ {L} \end{array} \right.\tag{8.5.8b}
$$

长轴位于纬度线，短轴位于子午线。

若 $R \sigma _ { L } = R \mathrm { c o s } L \sigma _ { \lambda } = \sigma$ ，则椭圆退化成圆，圆半径为 $r { = } \sqrt { 2 } K \sigma$ 0

## 8.5.4 等概率误差椭圆的确定

(8. 5. 8)说明等概率误差椭圆的大小(半长轴和半短轴)与 有关，而 是概率的函数，下面分析这一关系。

记等概率误差椭圆为 ，如图 8.5.2 $( L , \lambda )$ 平面中的虚线所示， 围成的区域为 $D _ { \circ }$ 则 x和 $y$ 位于 域内的概率为

$$
P = \frac {1}{2 \pi \sigma_ {x} \sigma_ {y}} \iint_ {D} \exp \left[ - \frac {1}{2} \left(\frac {x ^ {2}}{\sigma_ {x} ^ {2}} + \frac {y ^ {2}}{\sigma_ {y} ^ {2}}\right) \right] d x d y\tag{8.5.9}
$$

为便于计算式(8.5.9)中的积分，引人极坐标

$$
x = \rho \cos \theta , \quad y = \rho \sin \theta
$$

则

$$
\frac {x ^ {2}}{2 \sigma_ {x} ^ {2}} + \frac {y ^ {2}}{2 \sigma_ {y} ^ {2}} = \rho^ {2} \left(\frac {\cos^ {2} \theta}{2 \sigma_ {x} ^ {2}} + \frac {\sin^ {2} \theta}{2 \sigma_ {y} ^ {2}}\right)
$$

记

$$
c (\theta) = \sqrt {\frac {\cos^ {2} \theta}{2 \sigma_ {x} ^ {2}} + \frac {\sin^ {2} \theta}{2 \sigma_ {y} ^ {2}}}\tag{8.5.10}
$$

则

$$
\exp \left[ - \frac {1}{2} \left(\frac {x ^ {2}}{\sigma_ {x} ^ {2}} + \frac {y ^ {2}}{\sigma_ {y} ^ {2}}\right) \right] = e ^ {- c ^ {2} (\theta) \rho^ {2}}\tag{8.5.11}
$$

将式 (8. 5. 7)改写成

$$
\frac {x ^ {2}}{2 \sigma_ {x} ^ {2}} + \frac {y ^ {2}}{2 \sigma_ {y} ^ {2}} = K ^ {2}\tag{8.5.12}
$$

该式即为式(8. 5. 9)中积分域 的边界 的方程。将式(8.5.12)写成极坐标形式

$$
\rho_ {E} ^ {2} \left(\frac {\cos^ {2} \theta}{2 \sigma_ {x} ^ {2}} + \frac {\sin^ {2} \theta}{2 \sigma_ {y} ^ {2}}\right) = K ^ {2}
$$

根据式(8.5.10) ，上式可写成

$$
\rho_ {E} ^ {2} c ^ {2} (\theta) = K ^ {2}
$$

即

$$
\rho_ {E} = \frac {K}{c (\theta)}
$$

该式即为围成 域的椭圆 的极坐标方程 域的向径变化范围为 $0 { \sim } _ { \rho _ { E } }$ ，极角变化范围为 $0 \sim$ $2 \pi$ ，极坐标下积分微面积元为 $\rho \mathrm { d } \theta \mathrm { d } \rho$ o 所以式 (8. 5. 9)可写成

$$
\begin{array}{l} P = \frac {1}{2 \pi \sigma_ {x} \sigma_ {y}} \int_ {0} ^ {2 \pi} \int_ {0} ^ {\rho_ {E}} \mathrm{e} ^ {- c ^ {2} (\theta) \rho^ {2}} \rho \mathrm{d} \theta \mathrm{d} \rho \\ = \frac {1}{2 \pi \sigma_ {x} \sigma_ {y}} \int_ {0} ^ {2 \pi} \int_ {0} ^ {\frac {K}{c (\theta)}} \mathrm{e} ^ {- c ^ {2} (\theta) \rho^ {2}} \rho \mathrm{d} \theta \mathrm{d} \rho \end{array}
$$

$$
\begin{array}{l} = \frac {1}{2 \pi \sigma_ {x} \sigma_ {y}} \int_ {0} ^ {2 \pi} \frac {1}{2 c ^ {2} (\theta)} \left[ - \mathrm{e} ^ {- c ^ {2} (\theta) \rho^ {2}} \Big | _ {\rho = 0} ^ {\rho = \frac {K}{c (\theta)}} \right] \mathrm{d} \theta \\ = \frac {1}{2 \pi \sigma_ {x} \sigma_ {y}} \frac {1}{2} (1 - e ^ {- K ^ {2}}) \int_ {0} ^ {2 \pi} \frac {1}{c ^ {2} (\theta)} \mathrm{d} \theta \end{array}\tag{8.5.13}
$$

由式 (8. 5. 8) 知，当 $K = \infty$ 时，等概率误差椭圆的半长轴和半短轴都趋 $\mp \infty$ ，积分域 包含整个误差平面，此时误差落在整个误差平面内的概率为 ，即

$$
1 = \lim _ {K \rightarrow \infty} P = \frac {1}{4 \pi \sigma_ {x} \sigma_ {y}} \int_ {0} ^ {2 \pi} \frac {1}{c ^ {2} (\theta)} \mathrm{d} \theta
$$

所以

$$
\int_ {0} ^ {2 \pi} \frac {1}{c ^ {2} (\theta)} \mathrm{d} \theta = 4 \pi \sigma_ {x} \sigma_ {y}\tag{8.5.14}
$$

将式 (8.5.14)代入式 (8.5.13) 时，得

$$
P = \frac {1}{4 \pi \sigma_ {x} \sigma_ {y}} \left(1 - \mathrm{e} ^ {- K ^ {2}}\right) \cdot 4 \pi \sigma_ {x} \sigma_ {y} = 1 - \mathrm{e} ^ {- K ^ {2}}\tag{8.5.15}
$$

(8. 5. 15)给出了惯导系统误差为 $\sigma _ { x }$ 和 $\sigma _ { y }$ 时，不同 值(亦即半长轴和半短轴取不同值〉条件下，飞机指示位置落在以真实位置为中心的椭圆内的概率。式(8. 5. 15)也可写成

$$
K = \sqrt {- \ln (1 - P)}\tag{8.5.16}
$$

(8. 5. 16)给出了已知概率为 时计算等概率误差椭圆参数 的方法。 一旦确定出，则根据式(8. 5. 8)可确定出等概率误差椭圆的大小，即椭圆的半长轴和半短轴.表 8.5.1 列出了的部分对应值.

8.5.1 对应于不同概率 的E

<table><tr><td>P</td><td>0</td><td>0.05</td><td>0.2</td><td>0.5</td><td>0.633</td><td>0.8</td><td>0.95</td><td>0.99</td><td>1</td></tr><tr><td>K</td><td>0</td><td>0.227</td><td>0.472</td><td>0.832</td><td>1</td><td>1.268</td><td>1.730</td><td>2.147</td><td>∞</td></tr></table>

## 8.5.5 根据 RNP指标确定惯导系统的精度要求

## 1. RNP指标为标准椭圆时惯导系统精度要求的确定

由式(8. 5. 2)知，飞机真实位置相对指示位置的偏离值(dL，但)与惯导系统的导航误差〈悦，以)的概率分布密度是相同的，唯一不同的是两者所取原点不同 对于(Il ，以) ，取飞机的真实位置点作为其坐标系的原点，而对于(dL，也) ，取指示位置点作为其坐标系的原点。

根据上述分析，可归纳出根据 RNP指标确定惯导系统应满足精度要求的方法。

设时\~P指标为 飞机真实位置位于以指示位置为中心的椭圆内的概率为 该椭圆的半长轴为 半短轴为 则惯导系统应满足的精度要求按如下步骤确定。

(1)根据 RNP 要求的概率 查表 8.5.1 或按式(8. 5. 16)确定出相应的参数 K.

(2) 根据求得的参数 RNP要求的椭圆半长轴 和半短轴 确定出惯导系统应满足的经、纬度误差均方根 $\sigma _ { \lambda }$ 和 $\sigma _ { L }$ 具体方法如下。

①若椭圆半长轴位于子午线方向，半短轴位于纬度线方向，则由式(8.5. 8a) ，可得

$$
\sigma_ {L} = \frac {a}{\sqrt {2} K R}\tag{8.5.17a}
$$

$$
\sigma_ {\lambda} = \frac {b}{\sqrt {2} K R \cos L}\tag{8.5.17b}
$$

②若椭圆半长轴位 纬度线方向， 短轴位 子午线方向，则由式(8.5. 8b) ，可得

$$
\sigma_ {L} = \frac {b}{\sqrt {2} K R}\tag{8.5.18a}
$$

$$
\sigma_ {\lambda} = \frac {a}{\sqrt {2} K R \cos L}\tag{8.5.18b}
$$

## 2. RNP 指标为非标准椭圆时惯导系统精度要求的确定

RNP 指标为非标准椭圆，椭圆长轴沿 $X _ { \mathfrak { s } }$ 轴，短轴沿 $Y _ { \varepsilon }$ 轴，Xε 轴与 $x _ { g }$ 轴(地理坐标系 $g$ 的东向轴)夹角为 $\beta , \beta$ 卢以逆时针方向为正，如图 8. .3 所示。 g(N)

记沿 $X _ { \varepsilon }$ 轴的误差量为 $\boldsymbol { \mathcal { X } } _ { \varepsilon }$ ,沿 $Y _ { \varepsilon }$ 轴的误差量为 $y _ { \varepsilon }$ ，对应于 非标准椭圆的误差向 $\pmb { \mu }$ ，则坐标系 $O \mathrm { - } x _ { g } y _ { g } z _ { g }$ 简记为 $g )$ 坐标系 $O { - } X _ { \varepsilon } Y _ { \varepsilon } Z _ { \varepsilon }$ 简记为 $\varepsilon )$ 的变换矩阵为

$$
\boldsymbol {C} _ {g} ^ {\xi} = \left[ \begin{array}{c c c} \cos \beta & \sin \beta & 0 \\ - \sin \beta & \cos \beta & 0 \\ 0 & 0 & 1 \end{array} \right]
$$

![](images/4afa6747ebb4eacdad85b3fa21352f26c6f005ada70a275f38412130645ae8a4.jpg)

$$
\boldsymbol {\mu} ^ {g} = \left[ \boldsymbol {C} _ {g} ^ {\epsilon} \right] ^ {T} \boldsymbol {\mu} ^ {\varepsilon} = \left[ \begin{array}{c c c} \cos \beta & - \sin \beta & 0 \\ \sin \beta & \cos \beta & 0 \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c} x _ {\varepsilon} \\ y _ {\varepsilon} \\ z _ {\varepsilon} \end{array} \right]
$$

5. 3 非标准椭圆 RNP

维定位，

$$
\mu_ {x} ^ {\mathrm{g}} = x _ {\varepsilon} \cos \beta - y _ {\varepsilon} \sin \beta\tag{8.5.19a}
$$

$$
\mu_ {y} ^ {\mathrm{g}} = x _ {\varepsilon} \sin \beta + y _ {\varepsilon} \cos \beta\tag{8.5.19b}
$$

假设 $\mathcal { X } _ { \varepsilon }$ 和 $y _ { \varepsilon }$ 不相 ，均值均为 ，方 分别为 $\sigma _ { x _ { \varepsilon } } ^ { 2 }$ 和 $\sigma _ { y _ { \xi } } ^ { 2 }$ ，非标准椭圆的 长轴和 短轴分别为 $a$ 和 $b$ 则根据式 (8. 5. 7)

$$
\sigma_ {x \varepsilon} = \frac {a}{\sqrt {2} K}\tag{8.5.20a}
$$

$$
\sigma_ {y \varepsilon} = \frac {b}{\sqrt {2} K}\tag{8.5.20b}
$$

式中 根据式 8. 5. 16)确定

由式 (8.5. 19) $\mathbf { \nabla } \cdot \mu _ { x } ^ { g }$ 和 $\mu _ { y } ^ { g }$ 的均值也为 ，方差分别为

$$
\begin{array}{l} \sigma_ {\mu_ {x}} ^ {2} = \sigma_ {x \varepsilon} ^ {2} \cos^ {2} \beta + \sigma_ {y \varepsilon} ^ {2} \sin^ {2} \beta \\ \sigma_ {\mu_ {y}} ^ {2} = \sigma_ {x \varepsilon} ^ {2} \sin^ {2} \beta + \sigma_ {y \varepsilon} ^ {2} \cos^ {2} \beta \end{array}
$$

即

$$
\begin{array}{l} \sigma_ {\lambda} ^ {2} R ^ {2} \cos^ {2} L = \sigma_ {x \varepsilon} ^ {2} \cos^ {2} \beta + \sigma_ {y \varepsilon} ^ {2} \sin^ {2} \beta \\ \sigma_ {L} ^ {2} R ^ {2} = \sigma_ {x \varepsilon} ^ {2} \sin^ {2} \beta + \sigma_ {y \varepsilon} ^ {2} \cos^ {2} \beta \end{array}
$$

所以对惯导系统的精度 求为

$$
\sigma_ {\lambda} = \frac {1}{R \cos L} \sqrt {\sigma_ {x \varepsilon} ^ {2} \cos^ {2} \beta + \sigma_ {y \varepsilon} ^ {2} \sin^ {2} \beta}\tag{8.5.21a}
$$

$$
\sigma_ {L} = \frac {1}{R} \sqrt {\sigma_ {x \varepsilon} ^ {2} \sin^ {2} \beta + \sigma_ {y \varepsilon} ^ {2} \cos^ {2} \beta}\tag{8.5.21b}
$$

式中， $, \sigma _ { x }$ 和 $\sigma _ { \mathcal { \bar { E } } }$ 按式 (8.5.20)确定-

## 8.5.6 误差困半径与散布圆

等概率误差椭圆完整描述了定位误差按一定概率的分布情况，表达式中使用经度误差和纬度误差两个自变量，自变量对应的坐标轴分别沿纬度线和子午线。有时为简化问题，只关心计算得的飞机位置相对飞机的真实位置偏离了多少距离，这就使两个变量的定位问题简化成单个变量的定位问题。为叙述方便，将飞机的计算位置偏离飞机的真实位置的距离称作误差距离。

设Il 和以为惯导系统的定位误差，地球近似视为圆球，地球半径为 $R _ { \odot }$ 。取飞机真实位置所在点为坐标原点，该点的东向为 轴，北向为 轴，飞机所在点的纬度为 ，则导航误差沿东向和北向的直角坐标为

$$
\left\{ \begin{array}{l} x = R \cos L \delta \lambda \\ y = R \delta L \end{array} \right.\tag{8.5.22}
$$

惯导系统的指示位置偏离飞机真实位置的误差距离为

$$
s = \sqrt {x ^ {2} + y ^ {2}} = R \sqrt {\cos^ {2} L \delta \lambda^ {2} + \delta L ^ {2}}
$$

假设llL和似的均值为 ，方差分别为 $\sigma _ { L } ^ { 2 }$ 和 $\sigma _ { \lambda } ^ { 2 }$ ，即

$$
\begin{array}{c} \sigma_ {x} ^ {2} = R ^ {2} \cos^ {2} L \sigma_ {\lambda} ^ {2} \\ \sigma_ {y} ^ {2} = R ^ {2} \sigma_ {L} ^ {2} \end{array}
$$

的均值也为 ，方差为

$$
\sigma_ {s} ^ {2} = R ^ {2} \cos^ {2} L \sigma_ {\lambda} ^ {2} + R ^ {2} \sigma_ {L} ^ {2}\tag{8.5.23}
$$

根据式 (8.5.8) ，若取 $K { = } \frac { 1 } { \sqrt { 2 } }$ (对应于 二川剧) ，则

$$
\sigma_ {s} ^ {2} = a ^ {2} + b ^ {2}
$$

式中 分别为P\~39. 34%条件下等概率误差椭圆的半长轴和半短轴。

若取 $K { = } \sqrt { 3 } ($ (对应于 $P { = } 0$ 9502) ，根据式 (8. 5. 8)

$$
a ^ {2} + b ^ {2} = 6 (R ^ {2} \sigma_ {L} + R ^ {2} \cos^ {2} L \sigma_ {\lambda} ^ {2})
$$

所以

$$
\sigma_ {s} ^ {2} = \frac {1}{6} (a ^ {2} + b ^ {2})
$$

式中 分别为P\~95. 02%条件下等概率误差椭圆的半长轴和半短轴。

由式(8.5.5) ，惯导系统的经纬度误差服从联合正态分布，概率密度为

$$
p (\delta L, \delta \lambda) = \frac {1}{2 \pi \sigma_ {L} \sigma_ {\lambda}} \exp \left[ - \frac {1}{2} \left(\frac {\delta L ^ {2}}{\sigma_ {L} ^ {2}} + \frac {\delta \lambda^ {2}}{\sigma_ {\lambda} ^ {2}}\right) \right]
$$

根据式(8. 5. 22) ，相应的直角坐标值 也服从联合正态分布，概率密度为

$$
p (x, y) = \frac {1}{2 \pi \sigma_ {x} \sigma_ {y}} \exp \left[ - \frac {1}{2} \left(\frac {x ^ {2}}{\sigma_ {x} ^ {2}} + \frac {y ^ {2}}{\sigma_ {y} ^ {2}}\right) \right]\tag{8.5.24}
$$

引人极坐标: $x { = } \rho \mathrm { c o s } \theta , y { = } \rho \mathrm { s i n } \theta$ ， 如图 8.5.4 所示。这样式 (8. 5. 24) 可写成:

$$
\begin{array}{r l} p (\rho , \theta) & = \frac {1}{2 \pi \sigma_ {x} \sigma_ {y}} \exp \left[ - \frac {\rho^ {2}}{2} \left(\frac {\cos^ {2} \theta}{\sigma_ {x} ^ {2}} + \frac {\sin^ {2} \theta}{\sigma_ {y} ^ {2}}\right) \right] \\ & = \frac {1}{2 \pi R ^ {2} \cos L \sigma_ {L} \sigma_ {\lambda}} \exp \left[ - \frac {\rho^ {2}}{2} \left(\frac {\cos^ {2} \theta}{R ^ {2} \cos^ {2} L \sigma_ {\lambda} ^ {2}} + \frac {\sin^ {2} \theta}{R ^ {2} \sigma_ {L} ^ {2}}\right) \right] \end{array}\tag{8.5.25}
$$

导航误差落在以 为半径、飞机真实位置点为圆心的圆内的概率为

$$
P (\rho \leqslant r, 0 \leqslant \theta \leqslant 2 \pi) = \int_ {0} ^ {r} \int_ {0} ^ {2 \pi} p (\rho , \theta) \rho \mathrm{d} \theta \mathrm{d} \rho
$$

![](images/d624fa99725bc5a781cd5e5f4fc5d474b157a7f28e92ade708503ee8b9d4f237.jpg)  
8\_ 5. 4 极坐标及误差圆

(8. 5. 26)

式中， $\phi ( \rho , \theta )$ 由式(8. 5. 25)确定。对于某一概率 ，可根据式(8. 5. 26)求出相应的圆半径 $r _ { \mathrm { ~ c ~ } }$

国际民航组织(ICAO) 建议:用概率 $P { = } 9 5$ %所对应的圆半径 $R _ { 0 }$ 作为衡量导航系统精度的标准，并称该圆半径 $R _ { 0 }$ 。为误差圆半径。

根据式 (8.5.26) $R _ { 0 }$ 。满足的方程为

$$
95\% = \int_{0}^{R_{0}}\int_{0}^{2\pi}p(\rho ,\theta)\rho \mathrm{d}\theta \mathrm{d}\rho\tag{8.5.27}
$$

式和式 (8.5.23) 、式 (8.5.25) 知， $R _ { 0 }$ 凡是 $\sigma _ { x }$ 和 $\sigma _ { y }$ 亦即 $\sigma _ { s }$ 的函数，图 8. 5. 是根据式(8. 5. 27) 计算得的 $R _ { 0 }$ 与 $\sigma _ { x } \ l _ { \cdot } \sigma _ { y }$ 及 $\sigma _ { s }$ 间的关系曲线，图中纵坐标为 $\cdot \frac { R _ { \circ } } { 2 \sigma _ { s } }$ 横坐标为 $Q _ { e } = \frac { \sigma _ { x } } { \sigma _ { y } } + \frac { \sigma _ { y } } { \sigma _ { x } }$ 。如此取纵坐标和横坐标的目的是使曲线归一化，这样可根据该曲线求解出任意 $\sigma _ { x }$ 和 $\sigma _ { y }$ 情况下对应的误差圆半径。

![](images/19bbce00588db4eb213003a0efbeb57e7adc6394c14e57e9d57e73c9ffcbd8bc.jpg)  
95%概率误差圆半径与惯导系统定位误差间的关系

由于

$$
Q _ {e} = \frac {\sigma_ {x}}{\sigma_ {y}} + \frac {\sigma_ {y}}{\sigma_ {x}} = \frac {\left(\sigma_ {x} - \sigma_ {y}\right) ^ {2} + 2 \sigma_ {x} \sigma_ {y}}{\sigma_ {x} \sigma_ {y}} = 2 + \frac {\left(\sigma_ {x} - \sigma_ {y}\right) ^ {2}}{\sigma_ {x} \sigma_ {y}}
$$

且 $\sigma _ { x } { > } 0 , \sigma _ { y } { > } 0$ 所以 $Q _ { e } { \geqslant } 2$ ，这就是图 8.5.5 曲线的横坐标从 起始的原因

从图 8. 5. 曲线可 出:在 95% 率条件下， $\frac { R _ { \circ } } { 2 \sigma _ { s } }$ 的取值范围为 O. 86\~O. 98。因为根据式

(8.5.27) 计算 $R _ { 0 }$ 比较麻烦，而 $R _ { 0 }$ 与 $2 \sigma _ { s }$ 相差不大，所以为避免计算麻烦，工程上常近似取

$$
R _ {0} = (1. 9 6 \sim 2) \sigma_ {s}\tag{8.5.28}
$$

8.5.1 节分析知，飞机真实位置偏离指示位置(由惯导系统计算得)的偏离值与惯导系统的导航误差是等价的，所以误差圆半径 $R _ { 0 }$ 。实际上也反映了飞机真实位置相对指示位置的散布情况，即

飞机的真实位置以 95%的概率位于散布圆内，该散布圆的圆心位于指示位置，半径为

$$
R _ {0} = (1. 9 6 \sim 2) \sigma_ {s}
$$

式中

$$
\sigma_ {s} = \sqrt {\sigma_ {x} ^ {2} + \sigma_ {y} ^ {2}} = R \sqrt {\sigma_ {\lambda} ^ {2} \cos^ {2} L + \sigma_ {L} ^ {2}}
$$

$\sigma _ { \lambda }$ 和 $\sigma _ { L }$ 为惯导系统的经度误差和纬度误差。

