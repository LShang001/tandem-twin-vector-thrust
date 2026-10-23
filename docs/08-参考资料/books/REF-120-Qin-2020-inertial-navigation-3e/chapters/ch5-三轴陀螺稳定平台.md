# 第5章 三轴陀螺稳定平台

## 第5章 三轴陀螺稳定平台

飞机、导弹、舰艇、坦克等运载体在运动过程中的姿态、航向是任意变化的，要对其进行导航或制导控制，需要在运载体内部建立起导航坐标系，导航坐标系可通过两种途径建立起来:一种途径是将陀螺和加速度计直接固联在运载体上，通过对陀螺和加速度计的输出作解算获得姿态矩阵，并作导航解算，这类系统就是捷联式惯导系统，导航坐标系以数学平台形式体现。另一种途径是将陀螺和加速度计安装在用三个环架支承起来的平台台体上，通过控制台体的旋转使陀螺和加速度计的敏感轴始终与要求的导航坐标系重合，这类系统就是平台式惯导系统，导航坐标系以物理平台形式体现。

三轴平台是由单轴平台复合而成的，单轴平台的一些设计理论也适用三轴平台，但在实现几何稳定和空间积分两种工作状态的过程中出现了许多特殊问题，如平台驱动信号的合理分配，基座角运动的耦合和隔离，陀螺输出轴的交叉耦合，三轴平台系统的交叉耦合等，这些问题将在后续内容中逐一介绍。

## 5. 1 三轴平台简介

5. 1. 为三轴平台结构模型，图中 为横滚环 为俯仰环， 为方位环 $M _ { r } \mathbf { \nabla } . M _ { f } \mathbf { \nabla } . M _ { a }$ 二为安装在相应环架轴上的力矩马达 $G _ { x } , G _ { y } , G _ { z }$ 为单自由度陀螺仪，敏感轴相互正交，它们的指向构成了平台坐标系 $x _ { a } y _ { a } z _ { a } , A C R$ 为方位坐标分解器 SR 为俯仰正割分解器 $G _ { x } , G _ { y } , G _ { z }$ 陀螺的输出经

![](images/d6ec31cd71fe44f0c6edb712afa8fb97b51acc8f31818fed1cc46d164207f566.jpg)  
5. 1. 三轴平台结构模型

ACR 作信息分配和放大后分别馈人 $M _ { r } , M _ { f }$ 和 $M _ { a }$ .控制相应环架的角运动，其中控制横滚通道的信号还需经 SR 作俯仰角补偿处理。在各环架轴安装有角度传感器(图中未画出) .如自整角机发送器或线性旋转变压器，从方位环、俯仰环和横滚环上拾取的环架角分别为飞机的航向角、俯仰角和横滚角。

为了减小飞机机体振动对平台伺服系统和惯性仪表的影响，平台环架系统和基座之间装有减震器。用于飞机的兰轴平台系统方位环轴和横滚环轴的转动角度范围通常是无限制的，绕俯仰环轴的转动角度范围根据需要而定。凡是要求转动角度超过 $3 6 0 ^ { \circ }$ 。的环架轴，输电装置都要采用电刷滑环组件，对转动角小于 $1 8 0 ^ { \circ }$ 。的环架轴，通常采用软导线或其他形式的输电装置。

为使叙述和分析问题方便，此处对一些常用坐标系作如下规定:

(1)基座坐标系 $b { \mathrm { : } }$ 与机体固联. $x _ { b } \ldots y _ { b } \ldots z _ { b }$ 分别指向飞机的右、前、上。

(2) 横滚环坐标系 $r :$ 与横滚环回联 $y _ { r }$ 轴沿横滚环轴，与 $y _ { b }$ 轴指向相同 .r 系相对 系只能绕 $y _ { b }$ 轴旋转，产生横滚环角 $\theta _ { r }$ 0

(3) 俯仰环坐标系 $f { \mathrm { : } }$ 与俯仰环固联 $\boldsymbol { \mathcal { X } } _ { f }$ 轴沿俯仰环轴，与 $\boldsymbol { \mathcal { X } } _ { r }$ 轴指向相同 . $f$ 系相对 系只能绕 $\mathcal { X } _ { r }$ 轴旋转，产生俯仰环角。 $\theta _ { f }$

(4) 方位环坐标系 $a :$ 与方位环固联. $\mathcal { Z } _ { a }$ 轴沿方位环轴，与 $z _ { f }$ 轴指向相同， 系相对 系只能绕 $\boldsymbol { z } _ { f }$ 轴旋转，产生方位环角 $\theta _ { a }$

(5) 环架坐标系 $A : x _ { A } \mathrm { , } y _ { A } \mathrm { , } z _ { A }$ 轴分别为 $\mathcal { X } _ { f } \setminus \mathcal { Y } _ { r } \setminus \mathcal { Z } _ { a }$ 轴，即为相应轴上的力矩马达产生力矩的方向，所以也称为力矩马达坐标系。一般情况下该坐标系不是正交坐标系，只有当 $\theta _ { r } { = } 0 . \theta _ { f } { = } 0$ $\theta _ { a } = 0 ($ (即环架处于中立位置)时，才为正交坐标系。

## 5. 环架驱动信号的合理分配 方位坐标

## 分解器和俯仰正割分解器

## 5.2.1 角速度耦合关系分析

假设基座角速度为 $\begin{array} { r } { \pmb { \omega } _ { i b } ^ { b } = \mp \omega _ { i b x } ^ { b } \quad \omega _ { i b y } ^ { b } \quad \omega _ { i b z } ^ { b } } \end{array}$ ]T，各环架受力矩马达驱动后所具有的驱动角速度为 $\dot { \theta } _ { r } \lrcorner \dot { \theta } _ { f } \lrcorner \dot { \theta } _ { a }$ ，瞬时环架角为 $\theta _ { r } \ldots \theta _ { f } \ldots \theta _ { a }$ 此，则根据坐标系 $\boldsymbol { b } , \boldsymbol { r } , f , \boldsymbol { a }$ 的定义，可画出各坐标系间的角位置关系，如图 5. 2. 所示。下面分析各环架具有的角速度。

![](images/283660bcf3704f47813c47b22b9b1d934933bf1404c15c78e5f004c929838da1.jpg)

![](images/98b2e514ed234263eef3fa93408c8c9453cfc68d13601a2f8275bffa448329c3.jpg)

![](images/db049ddef3e76125de1cc3c5c160dcfabc5a0acef8154d2e24918f073d67c5b7.jpg)  
图 5.2.1 坐标系 $b , r , f , a$ 间的角位置关系

## 1. 横滚环 r 的角速度

由图5.2.1(a)，得

$$
\mathbf {C} _ {b} ^ {r} = \left[ \begin{array}{c c c} \cos \theta_ {r} & 0 & - \sin \theta_ {r} \\ 0 & 1 & 0 \\ \sin \theta_ {r} & 0 & \cos \theta_ {r} \end{array} \right]
$$

$$
\pmb {\omega} _ {i r} ^ {r} = \pmb {C} _ {b} ^ {r} \pmb {\omega} _ {i b} ^ {b} + \pmb {\omega} _ {b r} ^ {r}
$$

$$
\left[ \begin{array}{c} \boldsymbol {\omega} _ {i r x} ^ {r} \\ \boldsymbol {\omega} _ {i r y} ^ {r} \\ \boldsymbol {\omega} _ {i r z} ^ {r} \end{array} \right] = \left[ \begin{array}{c c c} \cos \theta_ {r} & 0 & - \sin \theta_ {r} \\ 0 & 1 & 0 \\ \sin \theta_ {r} & 0 & \cos \theta_ {r} \end{array} \right] \left[ \begin{array}{c} \boldsymbol {\omega} _ {i b x} ^ {b} \\ \boldsymbol {\omega} _ {i b y} ^ {b} \\ \boldsymbol {\omega} _ {i b z} ^ {b} \end{array} \right] + \left[ \begin{array}{c} 0 \\ \dot {\theta} _ {r} \\ 0 \end{array} \right] = \left[ \begin{array}{c} \boldsymbol {\omega} _ {i b x} ^ {b} \cos \theta_ {r} - \boldsymbol {\omega} _ {i b z} ^ {b} \sin \theta_ {r} \\ \boldsymbol {\omega} _ {i b y} ^ {b} + \dot {\theta} _ {r} \\ \boldsymbol {\omega} _ {i b x} ^ {b} \sin \theta_ {r} + \boldsymbol {\omega} _ {i b z} ^ {b} \cos \theta_ {r} \end{array} \right]\tag{5.2.1}
$$

## 2. 俯仰环 的角速度

由图 5.2. l(b) ，得

$$
\mathbf {C} _ {r} ^ {f} = \left[ \begin{array}{c c c} 1 & 0 & 0 \\ 0 & \cos \theta_ {f} & \sin \theta_ {f} \\ 0 & - \sin \theta_ {f} & \cos \theta_ {f} \end{array} \right]
$$

$$
\boldsymbol {\omega} _ {i f} ^ {f} = \mathbf {C} _ {r} ^ {f} \boldsymbol {\omega} _ {i r} ^ {r} + \boldsymbol {\omega} _ {r f} ^ {f}
$$

$$
\left[ \begin{array}{c} \boldsymbol {\omega} _ {i f x} ^ {f} \\ \boldsymbol {\omega} _ {i f y} ^ {f} \\ \boldsymbol {\omega} _ {i f z} ^ {f} \end{array} \right] = \left[ \begin{array}{c c c} 1 & 0 & 0 \\ 0 & \cos \theta_ {f} & \sin \theta_ {f} \\ 0 & - \sin \theta_ {f} & \cos \theta_ {f} \end{array} \right] \left[ \begin{array}{c} \boldsymbol {\omega} _ {i r x} ^ {r} \\ \boldsymbol {\omega} _ {i r y} ^ {r} \\ \boldsymbol {\omega} _ {i r z} ^ {r} \end{array} \right] + \left[ \begin{array}{c} \dot {\theta} _ {f} \\ 0 \\ 0 \end{array} \right] = \left[ \begin{array}{c} \boldsymbol {\omega} _ {i r x} ^ {r} + \dot {\theta} _ {f} \\ \boldsymbol {\omega} _ {i r y} ^ {r} \cos \theta_ {f} + \boldsymbol {\omega} _ {i r z} ^ {r} \sin \theta_ {f} \\ - \boldsymbol {\omega} _ {i r y} ^ {r} \sin \theta_ {f} + \boldsymbol {\omega} _ {i r z} ^ {r} \cos \theta_ {f} \end{array} \right]\tag{5.2.2}
$$

## 3. 方位环 的角速度

由图 5.2. l(c) ，得

$$
\boldsymbol {C} _ {f} ^ {a} = \left[ \begin{array}{c c c} \cos \theta_ {a} & \sin \theta_ {a} & 0 \\ - \sin \theta_ {a} & \cos \theta_ {a} & 0 \\ 0 & 0 & 1 \end{array} \right]
$$

$$
\boldsymbol {\omega} _ {i a} ^ {a} = \mathbf {C} _ {f} ^ {a} \boldsymbol {\omega} _ {i f} ^ {f} + \boldsymbol {\omega} _ {f a} ^ {a}
$$

$$
\left[ \begin{array}{c} \omega_ {i a x} ^ {a} \\ \omega_ {i a y} ^ {a} \\ \omega_ {i a z} ^ {a} \end{array} \right] = \left[ \begin{array}{c c c} \cos \theta_ {a} & \sin \theta_ {a} & 0 \\ - \sin \theta_ {a} & \cos \theta_ {a} & 0 \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c} \omega_ {i f x} ^ {f} \\ \omega_ {i f y} ^ {f} \\ \omega_ {i f z} ^ {f} \end{array} \right] + \left[ \begin{array}{c} 0 \\ 0 \\ \dot {\theta} _ {a} \end{array} \right] = \left[ \begin{array}{c} \omega_ {i f x} ^ {f} \cos \theta_ {a} + \omega_ {i f y} ^ {f} \sin \theta_ {a} \\ - \omega_ {i f x} ^ {f} \sin \theta_ {a} + \omega_ {i f y} ^ {f} \cos \theta_ {a} \\ \omega_ {i f z} ^ {f} + \dot {\theta} _ {a} \end{array} \right]\tag{5.2.3}
$$

上式改写成用沿三个环架轴 $x _ { f } \ 、 y _ { r } \ 、 z _ { a }$ 的角速度来表示的形式，即用 $\omega _ { i r y } ^ { r } \mathbf { \nabla } _ { \mathbf { \nu } } \omega _ { i f x } ^ { f } \mathbf { \nabla } _ { \mathbf { \nu } } \omega _ { i a z } ^ { a }$ 来表示之。由式 (5. 2. 2)

$$
\omega_ {i f y} ^ {f} = \omega_ {i r y} ^ {r} \cos \theta_ {f} + \omega_ {i r z} ^ {r} \sin \theta_ {f}
$$

又由式 (5. 2. 1)

$$
\omega_ {i r z} ^ {r} = \omega_ {i b x} ^ {b} \sin \theta_ {r} + \omega_ {i b z} ^ {b} \cos \theta_ {r}\tag{5.2.4}
$$

所以

$$
\omega_ {i f y} ^ {f} = \omega_ {i r y} ^ {r} \cos \theta_ {f} + \omega_ {i b x} ^ {b} \sin \theta_ {r} \sin \theta_ {f} + \omega_ {i b z} ^ {b} \cos \theta_ {r} \sin \theta_ {f}\tag{5.2.5}
$$

(5.2. 5) 代入式 (5.2.3) ，得

$$
\left[ \begin{array}{c} \boldsymbol {\omega} _ {i a x} ^ {a} \\ \boldsymbol {\omega} _ {i a y} ^ {a} \\ \boldsymbol {\omega} _ {i a z} ^ {a} \end{array} \right] = \left[ \begin{array}{c} \boldsymbol {\omega} _ {i f x} ^ {f} \cos \theta_ {a} + \boldsymbol {\omega} _ {i r y} ^ {r} \cos \theta_ {f} \sin \theta_ {a} \\ - \boldsymbol {\omega} _ {i f x} ^ {f} \sin \theta_ {a} + \boldsymbol {\omega} _ {i r y} ^ {r} \cos \theta_ {f} \cos \theta_ {a} \\ \boldsymbol {\omega} _ {i a z} ^ {a} \end{array} \right] + \left[ \begin{array}{c} \boldsymbol {\omega} _ {i b x} ^ {b} \sin \theta_ {r} \sin \theta_ {f} \sin \theta_ {a} + \boldsymbol {\omega} _ {i b z} ^ {b} \cos \theta_ {r} \sin \theta_ {f} \sin \theta_ {a} \\ \boldsymbol {\omega} _ {i b x} ^ {b} \sin \theta_ {r} \sin \theta_ {f} \cos \theta_ {a} + \boldsymbol {\omega} _ {i b z} ^ {b} \cos \theta_ {r} \sin \theta_ {f} \cos \theta_ {a} \\ 0 \end{array} \right]
$$

$$
\left[ \begin{array}{c c c} \cos \theta_ {a} & \cos \theta_ {f} \sin \theta_ {a} & 0 \\ - \sin \theta_ {a} & \cos \theta_ {f} \cos \theta_ {a} & 0 \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c} \omega_ {i f x} ^ {f} \\ \omega_ {i r y} ^ {r} \\ \omega_ {i a z} ^ {a} \end{array} \right] + \left[ \begin{array}{c c c} \sin \theta_ {r} \sin \theta_ {f} \sin \theta_ {a} & 0 & \cos \theta_ {r} \sin \theta_ {f} \sin \theta_ {a} \\ \sin \theta_ {r} \sin \theta_ {f} \cos \theta_ {a} & 0 & \cos \theta_ {r} \sin \theta_ {f} \cos \theta_ {a} \\ 0 & 0 & 0 \end{array} \right] \left[ \begin{array}{c} \omega_ {i b x} ^ {b} \\ \omega_ {i b y} ^ {b} \\ \omega_ {i b z} ^ {b} \end{array} \right]\tag{5.2.6}
$$

记

$$
\boldsymbol {T} _ {1} = \left[ \begin{array}{c c c} \cos \theta_ {a} & \cos \theta_ {f} \sin \theta_ {a} & 0 \\ - \sin \theta_ {a} & \cos \theta_ {f} \cos \theta_ {a} & 0 \\ 0 & 0 & 1 \end{array} \right]\tag{5.2.7}
$$

$$
\boldsymbol {T} _ {3} = \left[ \begin{array}{c c c} \sin \theta_ {r} \sin \theta_ {f} \sin \theta_ {a} & 0 & \cos \theta_ {r} \sin \theta_ {f} \sin \theta_ {a} \\ \sin \theta_ {r} \sin \theta_ {f} \cos \theta_ {a} & 0 & \cos \theta_ {r} \sin \theta_ {f} \cos \theta_ {a} \\ 0 & 0 & 0 \end{array} \right]\tag{5.2.8}
$$

$$
\boldsymbol {\omega} _ {i A} ^ {A} = \left[ \begin{array}{c} \boldsymbol {\omega} _ {i f x} ^ {f} \\ \boldsymbol {\omega} _ {i r y} ^ {r} \\ \boldsymbol {\omega} _ {i a z} ^ {a} \end{array} \right]\tag{5.2.9}
$$

则式 (5.2. 6) 可写成

$$
\pmb {\omega} _ {i a} ^ {a} = \pmb {T} _ {1} \pmb {\omega} _ {i A} ^ {A} + \pmb {T} _ {3} \pmb {\omega} _ {i b} ^ {b}\tag{5.2.10}
$$

式中 ${ \pmb T } _ { 1 }$ 称为环架系统几何关系阵，平台系统通过该矩阵将环架角速度传递给台体， ${ \bf { { T } } } _ { 3 }$ 称为基座角运动的几何约束耦合阵，平台系统通过该阵将基座角速度传递给台体.

(5. 2. 10)还可进一步写成用环架角变化率表示的形式。

由式 (5.2.2) 和式 (5.2. 1)，

$$
\omega_ {i f x} ^ {f} = \dot {\theta} _ {f} + \omega_ {i r x} ^ {r} = \dot {\theta} _ {f} + \omega_ {i b x} ^ {b} \cos \theta_ {r} - \omega_ {i b z} ^ {b} \sin \theta_ {r}
$$

由式 (5.2. 1)

$$
\omega_ {i r y} ^ {r} = \dot {\theta} _ {r} + \omega_ {i b y} ^ {b}
$$

由式 (5.2. 3) (5. 2. 2) 及式 (5.2. 1)

$$
\begin{array}{r l} \omega_ {i k z} ^ {a} & = \dot {\theta} _ {a} + \omega_ {i f z} ^ {f} = \dot {\theta} _ {a} - \omega_ {i r y} ^ {r} \sin \theta_ {f} + \omega_ {i r z} ^ {r} \cos \theta_ {f} \\ & = \dot {\theta} _ {a} - \omega_ {i b y} ^ {b} \sin \theta_ {f} - \dot {\theta} _ {r} \sin \theta_ {f} + \omega_ {i b x} ^ {b} \sin \theta_ {r} \cos \theta_ {f} + \omega_ {i b z} ^ {b} \cos \theta_ {r} \cos \theta_ {f} \end{array}
$$

所以式 (5.2.9) 可写成

$$
\begin{array}{r l} \left[ \begin{array}{c} \omega_ {i A x} ^ {A} \\ \omega_ {i A y} ^ {A} \\ \omega_ {i A z} ^ {A} \end{array} \right] & = \left[ \begin{array}{c} \omega_ {i f x} ^ {f} \\ \omega_ {i r y} ^ {r} \\ \omega_ {i a z} ^ {a} \end{array} \right] = \left[ \begin{array}{c} \dot {\theta} _ {f} + \omega_ {i b x} ^ {b} \cos \theta_ {r} - \omega_ {i b z} ^ {b} \sin \theta_ {r} \\ \dot {\theta} _ {r} + \omega_ {i b y} ^ {b} \\ \dot {\theta} _ {a} - \omega_ {i b y} ^ {b} \sin \theta_ {f} - \dot {\theta} _ {r} \sin \theta_ {f} + \omega_ {i b x} ^ {b} \sin \theta_ {r} \cos \theta_ {f} + \omega_ {i b z} ^ {b} \cos \theta_ {r} \cos \theta_ {f} \end{array} \right] \\ & = \left[ \begin{array}{c} \dot {\theta} _ {f} \\ \dot {\theta} _ {r} \\ \dot {\theta} _ {a} \end{array} \right] + \left[ \begin{array}{c c c} 0 & 0 & 0 \\ 0 & 0 & 0 \\ 0 & - \sin \theta_ {f} & 0 \end{array} \right] \left[ \begin{array}{c} \dot {\theta} _ {f} \\ \dot {\theta} _ {r} \\ \dot {\theta} _ {a} \end{array} \right] + \left[ \begin{array}{c c c} \cos \theta_ {r} & 0 & - \sin \theta_ {r} \\ 0 & 1 & 0 \\ \sin \theta_ {r} \cos \theta_ {f} & - \sin \theta_ {f} & \cos \theta_ {r} \cos \theta_ {f} \end{array} \right] \left[ \begin{array}{c} \omega_ {i b x} ^ {b} \\ \omega_ {i b y} ^ {b} \\ \omega_ {i b z} ^ {b} \end{array} \right] \end{array}\tag{5.2.11}
$$

(b)

$$
\dot {\boldsymbol {\theta}} _ {A} = \left[ \begin{array}{c} \dot {\theta} _ {f} \\ \dot {\theta} _ {r} \\ \dot {\theta} _ {a} \end{array} \right], \quad \boldsymbol {T} _ {4} = \left[ \begin{array}{c c c} 0 & 0 & 0 \\ 0 & 0 & 0 \\ 0 & - \sin \theta_ {f} & 0 \end{array} \right], \quad \boldsymbol {T} _ {5} = \left[ \begin{array}{c c c} \cos \theta_ {r} & 0 & - \sin \theta_ {r} \\ 0 & 1 & 0 \\ \sin \theta_ {r} \cos \theta_ {f} & - \sin \theta_ {f} & \cos \theta_ {r} \cos \theta_ {f} \end{array} \right]
$$

则式 (5.2. 11) 可写成

$$
\pmb {\omega} _ {i A} ^ {A} = \dot {\pmb {\theta}} _ {A} + \pmb {T} _ {4} \dot {\pmb {\theta}} _ {A} + \pmb {T} _ {5} \pmb {\omega} _ {i b} ^ {b}\tag{5.2.12}
$$

所以式 (5.2. 10) 可写成

$$
\boldsymbol {\omega} _ {i a} ^ {a} = \left(\boldsymbol {T} _ {1} + \boldsymbol {T} _ {1} \boldsymbol {T} _ {4}\right) \dot {\boldsymbol {\theta}} _ {A} + \left(\boldsymbol {T} _ {1} \boldsymbol {T} _ {5} + \boldsymbol {T} _ {3}\right) \boldsymbol {\omega} _ {i b} ^ {b}\tag{5.2.13}
$$

(5. 2. 13)说明:平台台体的角速度由电机的驱动角速度和基座角速度组成，如果适当控制电机的驱动，就能对消基座的角运动。其中 $\pmb { T } _ { 4 }$ 是 $\mathbf { \dot { \theta } } _ { A }$ 的摩擦约束(或直接带动)耦合阵， ${ \bf { T } } _ { 5 }$ 为 $\pmb { \omega } _ { i b } ^ { b }$ 的摩擦约束(或直接带动)相合阵。

## 5.2.2 环架驱动信号的合理分配一一方位坐标分解器和俯仰正割分解器

1.环架驱动信号不作分配存在的问题

5. 2. 是三轴平台的水平伺服回路示意图，图中陀螺 $G _ { y }$ 的输出信号用来控制横滚环，陀螺 $G _ { x }$ 的输出信号用来控制俯仰环，沿横滚环轴的负向作用有干扰力矩 $M _ { \mathrm { d } y }$ 0

![](images/8f1cacd573b0fb9f5a42ad828d23bfadee9205da88e7e5bdaf9985d3b2b8fd74.jpg)  
5.2.2 环架驱动信号未作分配时的平台伺服回路

假设飞机朝北飞行，如图 5. 2. 2(a)所示，环架处于中立位置。由于干扰力矩 $M _ { \mathrm { d y } }$ 的作用，横滚环绕 $y _ { r }$ 轴的负方向旋转，由于几何约束关系，俯仰环和方位环被带着一起旋转，陀螺 $G _ { y }$ 感测此转动，信号器输出与旋转角速度成正比的电压信号，经放大后加至横滚力矩电机，电机产生与 $M _ { \mathrm { d y } }$ 大小相等，方向相反的卸荷力矩，抵消干扰力矩。而此时，陀螺 $G _ { x }$ 的输入轴 $I _ { x }$ 与横滚环的旋转方向正交，所以俯仰伺服回路并不工作。综上所述，在图 5. 2. 2(a)所示位置，横滚和俯仰伺服回路能正常工作。

假设飞机由朝北飞行经右盘旋后改向朝东飞行，如图 5.2.2(b)所示，环架仍然处于中立位置。由于几何约束关系，横滚环和俯仰环均被顺时针带动转过 $9 0 ^ { \circ }$ ，而方位环沿 $\mathcal { Z } _ { a }$ 轴有伺服回路，方位伺服回路隔离了这一转动而保持原来方位，陀螺 $G _ { y }$ 和 $G _ { \mathcal { s } }$ 的输入轴方向仍保持原来方向。横滚环在 $M _ { \mathrm { d y } }$ 的作用下绕 $y _ { r }$ 的负方向旋转，旋转角速度被陀螺 $G _ { x }$ 感测，输出信号经放大后馈人俯仰力矩电机，在 $\boldsymbol { \mathcal { X } } _ { f }$ 轴上产生相应力矩 $M _ { \mathrm { d } x }$ 。而此时陀螺 $G _ { y }$ 的输入轴 $I _ { y }$ 与 $y _ { r }$ 轴正交，感测不到横滚环的旋转，输出为零，横滚伺服回路不工作，干扰力矩 $M _ { \mathrm { d } y }$ 仍然存在。此时 $M _ { \mathrm { d y } }$ 非但没被抵消，还产生了另外一个干扰力矩 $M _ { \mathrm { d } x }$ 。所以图 5.2.2(b)位置伺服回路不能正常工作。事实上，除图 5.2.2(a)位置外，所有位置都不能正常工作。

## 2. 环架驱动信号的合理分配分析

假设平台的指令角速度为 $\omega _ { \mathrm { c m d } x \setminus } \omega _ { \mathrm { c m d } y \setminus } \omega _ { \mathrm { c m d } z }$ ，将它们折算成指令电流加至陀螺 $G _ { x } , G _ { y } , G _ { z }$ 的力矩器中，忽略陀螺的瞬态响应，并记

$$
K _ {G i} = k _ {\omega i} k _ {g} k _ {u}, \quad i = x, y, z
$$

式中， $k _ { \omega i }$ 为指令角速度至指令电流的转换系数<sub>,</sub> $k _ { g } = \frac { H } { D }$ 为陀螺的传递系数， $k _ { u }$ 为信号器转角至电压的转换系数。

设平台的输出角速度为 $\omega _ { i a x } ^ { a } \ldots \omega _ { i a y } ^ { a } \ldots \omega _ { i a z } ^ { a }$ ，则从指令角速度到平台输出角速度的信息处理流程如5.2.3 所示。要求获得驱动环架的合适驱动信号，实际上就是要确定出 $T _ { 2 }$ ，该矩阵实现对陀螺输出信号作合理分配。

![](images/f45b03d0354e765398df3e921f3dedc2368370e223951f6fad7b1386cdd574ac.jpg)  
5.2.3 从指令角速度到平台输出角速度的信息处理流程图

由图 5.2.3

$$
\left[ \begin{array}{c} u _ {g x} ^ {a} \\ u _ {g y} ^ {a} \\ u _ {g z} ^ {a} \end{array} \right] = \left[ \begin{array}{c c c} K _ {G x} & 0 & 0 \\ 0 & K _ {G y} & 0 \\ 0 & 0 & K _ {G z} \end{array} \right] \left[ \begin{array}{c} \omega_ {\mathrm{cmdx}} \\ \omega_ {\mathrm{cmdy}} \\ \omega_ {\mathrm{cmdz}} \end{array} \right], \quad \left[ \begin{array}{c} u _ {g x} ^ {A} \\ u _ {g y} ^ {A} \\ u _ {g z} ^ {A} \end{array} \right] = \boldsymbol {T} _ {2} \left[ \begin{array}{c} u _ {g x} ^ {a} \\ u _ {g y} ^ {a} \\ u _ {g z} ^ {a} \end{array} \right]
$$

$$
\left[ \begin{array}{c} \boldsymbol {\omega} _ {i f x} ^ {f} \\ \boldsymbol {\omega} _ {i r y} ^ {r} \\ \boldsymbol {\omega} _ {i a z} ^ {a} \end{array} \right] = \left[ \begin{array}{c c c} W _ {f} & 0 & 0 \\ 0 & W _ {r} & 0 \\ 0 & 0 & W _ {a} \end{array} \right] \left[ \begin{array}{c} u _ {g x} ^ {A} \\ u _ {g y} ^ {A} \\ u _ {g z} ^ {A} \end{array} \right], \quad \left[ \begin{array}{c} \boldsymbol {\omega} _ {i a x} ^ {a} \\ \boldsymbol {\omega} _ {i a y} ^ {a} \\ \boldsymbol {\omega} _ {i a z} ^ {a} \end{array} \right] = \boldsymbol {T} _ {1} \left[ \begin{array}{c} \boldsymbol {\omega} _ {i f x} ^ {f} \\ \boldsymbol {\omega} _ {i r y} ^ {r} \\ \boldsymbol {\omega} _ {i a z} ^ {a} \end{array} \right]
$$

若在平台设计中取

$$
K _ {G x} = K _ {G y} = K _ {G z} = K _ {G}, \quad W _ {r} = W _ {f} = W _ {a} = W, W \cdot K _ {G} = 1
$$

则

$$
\left[ \begin{array}{l} \omega_ {i a x} ^ {a} \\ \omega_ {i a y} ^ {a} \\ \omega_ {i a z} ^ {a} \end{array} \right] = \boldsymbol {T} _ {1} W \boldsymbol {I} \cdot \boldsymbol {T} _ {2} K _ {G} \boldsymbol {I} \left[ \begin{array}{l} \omega_ {\mathrm{cmd} x} \\ \omega_ {\mathrm{cmd} y} \\ \omega_ {\mathrm{cmd} z} \end{array} \right] = \boldsymbol {T} _ {1} \cdot \boldsymbol {T} _ {2} \left[ \begin{array}{l} \omega_ {\mathrm{cmd} x} \\ \omega_ {\mathrm{cmd} y} \\ \omega_ {\mathrm{cmd} z} \end{array} \right]
$$

要使平台准确跟踪指令角速度，需有

$$
\boldsymbol {T} _ {1} \cdot \boldsymbol {T} _ {2} = \boldsymbol {I}
$$

即

$$
\boldsymbol {T} _ {2} = \boldsymbol {T} _ {1} ^ {- 1}\tag{5.2.14}
$$

将式 (5.2. 7) 代入上式，得

$$
\boldsymbol {T} _ {2} = \left[ \begin{array}{c c c} \cos \theta_ {a} & \cos \theta_ {f} \sin \theta_ {a} & 0 \\ - \sin \theta_ {a} & \cos \theta_ {f} \cos \theta_ {a} & 0 \\ 0 & 0 & 1 \end{array} \right] ^ {- 1} = \left[ \begin{array}{c c c} \cos \theta_ {a} & - \sin \theta_ {a} & 0 \\ \sin \theta_ {a} \sec \theta_ {f} & \cos \theta_ {a} \sec \theta_ {f} & 0 \\ 0 & 0 & 1 \end{array} \right]\tag{5.2.15}
$$

所以由陀螺输出至伺服回路输入间的信号分配关系为

$$
\left[ \begin{array}{l} u _ {g x} ^ {A} \\ u _ {g y} ^ {A} \\ u _ {g z} ^ {A} \end{array} \right] = \boldsymbol {T} _ {2} \left[ \begin{array}{l} u _ {g x} ^ {a} \\ u _ {g y} ^ {a} \\ u _ {g z} ^ {a} \end{array} \right] = \left[ \begin{array}{c} u _ {g x} ^ {a} \cos \theta_ {a} - u _ {g y} ^ {a} \sin \theta_ {a} \\ (u _ {g x} ^ {a} \sin \theta_ {a} + u _ {g y} ^ {a} \cos \theta_ {a}) \sec \theta_ {f} \\ u _ {g z} ^ {a} \end{array} \right]\tag{5.2.16}
$$

(5. 2. 16)可用图解形式来表示，如图 5.2.4 所示。

在平台式惯导系统的惯性平台中，采用旋转变压器实现方位坐标分解，如图 5. 2. 所示，旋转变压器的旋转轴与方位环轴同轴，定子固联在俯仰环上，转子则固联在方位环上，转子和定子上都绕有两组相互垂直的绕组，转子上的两组绕组作为原边，分别通人 $G _ { x }$ 和 $G _ { y }$ 陀螺的输出(400 周的调制信号) ，定子上的绕组作为副边，输出即为经正、余弦变换后的 400 周调制信号。

![](images/17a25b8e89899db09e0b1b546a352dcf2b8e388b5bada28f6c27d347061a52ef.jpg)  
5.2.4 环架驱动信号的分配

![](images/69a59d490ae7d6389b941d13a40bd3ecdb13388dd46edc7dd0eb03b640c02186.jpg)  
5.2.5 旋转变压器

设由 $u _ { g x } ^ { a }$ 和 $u _ { g y } ^ { a }$ 在 $x _ { a }$ 轴和 $y _ { a }$ 轴上产生的磁场分别为 ${ \dot { \phi } } _ { x } ^ { a }$ 和 $\dot { \phi } _ { y } ^ { a }$ ，方位环存在方位环角 $\theta _ { a }$ ，则磁场在 $\boldsymbol { \mathcal { X } } _ { f }$ 和 $y _ { f }$ 方向上的分量为

$$
\dot {\phi} _ {x} ^ {f} = \dot {\phi} _ {x} ^ {a} \cos \theta_ {a} - \dot {\phi} _ {y} ^ {a} \sin \theta_ {a}, \quad \dot {\phi} _ {y} ^ {f} = \dot {\phi} _ {x} ^ {a} \sin \theta_ {a} + \dot {\phi} _ {y} ^ {a} \cos \theta_ {a}
$$

式中， $\dot { \phi } _ { x } ^ { a } = k _ { 1 } u _ { g x } ^ { a } , \dot { \phi } _ { y } ^ { a } = k _ { 1 } u _ { g y } ^ { a }$ C

旋转变压器的副边输出为

$$
\begin{array}{l} U _ {g x} ^ {A} = k _ {2} \dot {\phi} _ {x} ^ {f} = k _ {\mathrm{ACR}} (u _ {g x} ^ {a} \cos \theta_ {a} - u _ {g y} ^ {a} \sin \theta_ {a}) \\ U _ {g y} ^ {A} = k _ {2} \dot {\phi} _ {y} ^ {f} = k _ {\mathrm{ACR}} (u _ {g x} ^ {a} \sin \theta_ {a} + u _ {g y} ^ {a} \cos \theta_ {a}) \end{array}
$$

式中， $, k _ { \mathrm { A C R } } = k _ { 1 } k _ { 2 }$ 为方位坐标分解器的变换系数。

在实际平台系统中，俯仰正割分解器采用计算电路实现，设电路的传递系数为 $k _ { \mathrm { S R } }$ ，则陀螺输出信号经分配后馈人伺服网络的信号为

$$
u _ {g x} ^ {A} = k _ {\mathrm{ACR}} \left(u _ {g x} ^ {a} \cos \theta_ {a} - u _ {g y} ^ {a} \sin \theta_ {a}\right)\tag{5.2.17a}
$$

$$
u _ {\mathrm{gy}} ^ {A} = k _ {\mathrm{ACR}} k _ {\mathrm{SR}} \left(u _ {\mathrm{gx}} ^ {a} \sin \theta_ {a} + u _ {\mathrm{gy}} ^ {a} \cos \theta_ {a}\right) \sec \theta_ {f}\tag{5.2.17b}
$$

$$
u _ {g z} ^ {A} = u _ {g z} ^ {a}\tag{5.2.17c}
$$

## 5. 3 三轴平台工作原理分析

## 5.3.1 三轴平台的稳定回路工作原理{几何稳定状态下的工作原理)

1.对干扰力矩的卸荷作用

如图 5.3. 1，假设三轴稳定平台在横滚轴上作用有干扰力矩 $M _ { \mathrm { d y } }$ 此时，横滚环绕 $y _ { r }$ 轴旋转，产生角速度 $\omega _ { \varsigma }$ 。由于几何约束关系，方位环也具有此角速度，陀螺 $G _ { x }$ 和 $G _ { y }$ 感测到的角速度分别为(图 5.3. 1)

![](images/571f7e4b607ac34700da45b8f89edfcb4a8757d1024c73f19c0bb9cd08bc887c.jpg)  
5.3.1 三轴稳定平台对干扰力矩的卸荷作用

陀螺 $G _ { x }$ 和 $G _ { y }$ 的输出分别为

$$
u _ {g x} ^ {a} = \frac {k _ {g} k _ {u} \omega (s) \sin \theta_ {a}}{s (\tau_ {g} s + 1)}, \quad u _ {g y} ^ {a} = \frac {k _ {g} k _ {u} \omega (s) \cos \theta_ {a}}{s (\tau_ {g} s + 1)}
$$

经方位坐标分解器和俯仰正割分解器后馈人俯仰伺服放大器的信号为

$$
u _ {g x} ^ {A} (s) = k _ {\mathrm{ACR}} \left(u _ {g x} ^ {a} \cos \theta_ {a} - u _ {g y} ^ {a} \sin \theta_ {a}\right) = \frac {k _ {g} k _ {u} k _ {\mathrm{ACR}} \omega (s)}{s \left(\tau_ {g} s + 1\right)} \left(\sin \theta_ {a} \cos \theta_ {a} - \cos \theta_ {a} \sin \theta_ {a}\right) = 0
$$

馈人横滚伺服放大器的信号为

$$
\begin{array}{r l} u _ {g y} ^ {A} (s) & = k _ {\mathrm{ACR}} k _ {\mathrm{SR}} \left(u _ {g x} ^ {a} \sin \theta_ {a} + u _ {g y} ^ {a} \cos \theta_ {a}\right) \sec \theta_ {f} \\ & = \frac {k _ {g} k _ {u} k _ {\mathrm{ACR}} k _ {\mathrm{SR}} \omega (s)}{s (\tau_ {g} s + 1)} \sec \theta_ {f} \left(\sin^ {2} \theta_ {a} + \cos^ {2} \theta_ {a}\right) = \frac {k _ {g} k _ {u} k _ {\mathrm{ACR}} k _ {\mathrm{SR}} \omega (s)}{s (\tau_ {g} s + 1)} \sec \theta_ {f} \end{array}
$$

$u _ { \mathrm { g y } } ^ { A }$ 经横滚伺服困路放大和校正后馈入横滚力矩电机，产生与干扰力矩 $M _ { \mathrm { d } y }$ 大小相等，方向相反的卸荷力矩，抵消干扰力矩。

在俯仰回路和方位回路干扰力矩的卸荷原理与此相仿，读者可自行分析。

## 2. 对基座角运动的隔离原理

设基座的角速度为曲 $\pmb { \omega } _ { i b }$ ，由式 (5.2.13) 和式 (5.2.10) 知 $\mathbf { | } \pmb { \omega } _ { i b }$ 通过摩擦耦合和几何约束耦合影响平台的角速度。摩擦耦合通过摩擦力矩影响平台，平台对此干扰的隔离本质上是对摩擦力矩作卸荷，分析方法与上一节介绍的方法相同。对几何约束耦合引起的干扰角速度的隔离本质上是角速度对消，下面对此作详细分析。

由式 (5.2. 1)，

$$
\omega_ {i r z} ^ {r} = \omega_ {i b x} ^ {b} \sin \theta_ {r} + \omega_ {i b z} ^ {b} \cos \theta_ {r}
$$

该角速度在俯仰环的 $y _ { f }$ 轴上的分量为

$$
\omega_ {y _ {f}} ^ {f} \left(\omega_ {i b}\right) = \omega_ {i r z} ^ {r} \sin \theta_ {f}\tag{5.3.1}
$$

由于几何约束关系，该角速度分量直接传递到平台台体上(图 5.3.2)

设平台的方位环角为 $\theta _ { a }$ ，陀螺 $G _ { x }$ 的敏感轴沿 $x _ { a }$ 轴 $G _ { y }$ 的敏感轴沿 $y _ { a }$ 轴，由横滚环轴力矩电机产生的横滚环轴伺服角速度为 $\omega _ { r } ^ { \mathrm { S F } }$ ，则 $G _ { x }$ 的输出为

$$
u _ {g x} ^ {a} = \frac {k _ {g} k _ {u} \left(\omega_ {i r z} ^ {r} \sin \theta_ {f} + \omega_ {r} ^ {\mathrm{SF}} \cos \theta_ {f}\right) \sin \theta_ {a}}{s \left(\tau_ {g} s + 1\right)}
$$

$G _ { y }$ 的输出为

$$
u _ {\mathrm{gy}} ^ {a} = \frac {k _ {g} k _ {u} \left(\omega_ {i r z} ^ {r} \sin \theta_ {f} + \omega_ {r} ^ {\mathrm{SF}} \cos \theta_ {f}\right) \cos \theta_ {a}}{s \left(\tau_ {g} s + 1\right)}
$$

经方位坐标分解器和俯仰正割分解器处理后馈人俯仰伺服回路的信号为

$$
u _ {g x} ^ {A} = k _ {\mathrm{ACR}} \left(u _ {g x} ^ {a} \cos \theta_ {a} - u _ {g y} ^ {a} \sin \theta_ {a}\right) = \frac {k _ {\mathrm{ACR}} k _ {g} k _ {u} \left(\omega_ {i r z} ^ {r} \sin \theta_ {f} + \omega_ {r} ^ {\mathrm{SF}} \cos \theta_ {f}\right)}{s \left(\tau_ {g} s + 1\right)} \left(\sin \theta_ {a} \cos \theta_ {a} - \cos \theta_ {a} \sin \theta_ {a}\right) = 0
$$

馈人横滚伺服回路的信号为

$$
\begin{array}{l} u _ {\mathrm{gy}} ^ {A} = k _ {\mathrm{ACR}} k _ {\mathrm{SR}} \left(u _ {\mathrm{gx}} ^ {a} \sin \theta_ {a} + u _ {\mathrm{gy}} ^ {a} \cos \theta_ {a}\right) \sec \theta_ {f} = \frac {k _ {\mathrm{ACR}} k _ {\mathrm{SR}} k _ {g} k _ {u} \left(\omega_ {i r z} ^ {r} \sin \theta_ {f} + \omega_ {r} ^ {\mathrm{SF}} \cos \theta_ {f}\right)}{s \left(\tau_ {g} s + 1\right)} \left(\sin^ {2} \theta_ {a} + \cos^ {2} \theta_ {a}\right) \sec \theta_ {f} \\ = \frac {k _ {\mathrm{ACR}} k _ {\mathrm{SR}} k _ {g} k _ {u} \left(\omega_ {i r z} ^ {r} \tan \theta_ {f} + \omega_ {r} ^ {\mathrm{SF}}\right)}{s \left(\tau_ {g} s + 1\right)} \end{array}
$$

所以横滚环力矩电机产生的伺服力矩为

$$
M _ {m r} (s) = - k _ {a r} F _ {r} (s) \frac {k _ {m r}}{R _ {r}} u _ {\mathrm{gy}} ^ {A} (s) = - k _ {\mathrm{ACR}} k _ {\mathrm{SR}} k _ {g} k _ {u} k _ {a r} \frac {k _ {m r}}{R _ {r}} \frac {F _ {r} (s)}{s \left(\tau_ {g} s + 1\right)} \left[ \omega_ {i r z} ^ {r} (s) \tan \theta_ {f} + \omega_ {r} ^ {\mathrm{SF}} (s) \right]
$$

式中， $\boldsymbol { \mathbf { \ell } } , k _ { a r }$ 和 $\boldsymbol { F } _ { r } \left( s \right)$ 分别为横滚伺服回路放大倍数和校正网络传递函数 $k _ { m r }$ 和 $R _ { r }$ 分别为横滚力矩电机的力矩系数和电枢回路电阻。在该力矩作用下横滚环的伺服角速度为

$$
\omega_ {r} ^ {\mathrm{SF}} (s) = \frac {M _ {m r} (s)}{s J _ {r}} = - \frac {k _ {\mathrm{ACR}} k _ {\mathrm{SR}} k _ {g} k _ {u} k _ {a r} k _ {m r}}{R _ {r} J _ {r}} \frac {F _ {r} (s)}{s ^ {2} (\tau_ {g} s + 1)} \left[ \omega_ {i r z} ^ {r} (s) \tan \theta_ {f} + \omega_ {r} ^ {\mathrm{SF}} (s) \right]
$$

记

$$
Y (s) = \frac {k _ {\mathrm{ACR}} k _ {\mathrm{SR}} k _ {g} k _ {u} k _ {a r} k _ {m r}}{R _ {r} J _ {r}} \frac {F _ {r} (s)}{s ^ {2} (\tau_ {g} s + 1)}
$$

则

$$
\omega_ {r} ^ {\mathrm{SF}} (s) = - \frac {Y (s) \omega_ {i r z} ^ {r} (s) \tan \theta_ {f}}{1 + Y (s)}
$$

假设 $\omega _ { i r s } ^ { r }$ 为常值，则平台系统达到稳态时

$$
\omega_ {r s s} ^ {\mathrm{SF}} = \lim _ {s \rightarrow 0} \cdot \omega_ {r} ^ {\mathrm{SF}} (s) = - \omega_ {i r z} ^ {r} \tan \theta_ {f}\tag{5.3.2}
$$

由图 5.3.2 知，该伺服角速度在俯仰环的 $y _ { f }$ 轴上的分量为

$$
\left(\omega_ {r s s} ^ {\mathrm{SF}}\right) _ {y _ {f}} = \omega_ {r s s} ^ {\mathrm{SF}} \cos \theta_ {f} = - \omega_ {i r z} ^ {r} \sin \theta_ {f}\tag{5.3.3}
$$

![](images/4ccba8d082cc542bfca8bc2cbd7c1607a7fb4eeaa120e8e6b3d666a22ad3ce88.jpg)  
5.3.2 兰轴平台对几何相合基座角运动的隔离作用

所以，俯仰环绕 $y _ { f }$ 轴的旋转角速度为

$$
\omega_ {f y} ^ {a} = \omega_ {y _ {f}} ^ {f} (\omega_ {i b}) + (\omega_ {r s s} ^ {\mathrm{SF}}) _ {y _ {f}}
$$

将式 (5.3. 1)和式 (5. 3. 3) 代入上式，得

$$
\omega_ {f y} ^ {f} = \omega_ {i r z} ^ {r} \sin \theta_ {f} - \omega_ {i r z} ^ {r} \sin \theta_ {f} = 0
$$

由几何约束关系，平台台体(方位环)绕俯仰环 $y _ { f }$ 轴的旋转角速度应与 $\omega _ { f y } ^ { f }$ 相等，也为零。

从上述分析可看出，由几何约束引起的基座角速度耦合是依靠伺服回路产生的伺服角速度对泊的。

由图 5.3.2 知，由基座角速度和伺服角速度引起俯仰环沿 $\mathcal { Z } _ { f }$ 轴方向的角速度为

$$
\omega_ {\Sigma z} ^ {f} = \omega_ {i r z} ^ {r} \cos \theta_ {f} - \omega_ {r} ^ {\mathrm{SF}} \sin \theta_ {f}
$$

由式 (5. 2. 1)和式 (5.3.2)

$$
\omega_ {\Sigma z} ^ {f} = \omega_ {i r z} ^ {r} (\cos \theta_ {f} + \tan \theta_ {f} \sin \theta_ {f}) = \frac {\omega_ {i r z} ^ {r}}{\cos \theta_ {f}} = \left(\omega_ {i b x} ^ {b} \sin \theta_ {r} + \omega_ {i b z} ^ {b} \cos \theta_ {r}\right) \sec \theta_ {f}
$$

由于平台台体仅跟踪指令角速度旋转，所以 $\omega _ { \Sigma z } ^ { f }$ 引起平台台体方位轴与轴承(轴承固联在俯仰环上)之间的相对转动而产生摩擦力矩，此摩擦力矩将由方位环稳定回路产生的卸荷力矩对消掉。

## 5.3.2 三轴平台的修正回路工作原理{空间积分状态下的工作原理)

设平台的指令角速度为

$$
\boldsymbol {\omega} _ {\mathrm{cmd}} ^ {a} = \left[ \begin{array}{c} 0 \\ \omega_ {\mathrm{cmdy}} \\ 0 \end{array} \right]
$$

在伺服回路作用下平台的进动角速度为 $\omega _ { y }$ o 为使分析过程简化，此处只讨论。 $\theta _ { f } = 0$ 和方位环架角为 $\theta _ { a }$ 的情况，平台采用积分陀螺作为敏感器件，如图 5.3.3 所示。

![](images/9a7c10d5762bb53db837281a8354bce084e9f29129bcde34cb0fc65c624eb2b7.jpg)  
5.3.3 三轴平台的修正回路工作原理

系统工作时，将指令角速度换算成施矩电流，馈人 $G _ { y }$ 陀螺的力矩器，电流方向的确定原则是:当 $\omega _ { \mathrm { c m d y } }$ 为正时，馈人力矩电流后，应使 $G _ { y }$ 陀螺产生负向输出角，即陀螺输出角 $\sigma _ { c }$ 与指令角速度反号:

$$
\frac {\sigma_ {c} (s)}{\omega_ {\mathrm{cmdy}} (s)} = - \frac {k _ {g}}{s (\tau_ {g} s + 1)}
$$

由于伺服回路作用，平台产生进动角速度 $\omega _ { y }$ ，而该角速度 $G _ { y }$ 陀螺能感测到。设对应于 $\omega _ { y }$ 的 $G _ { y }$ 陀螺的输出角为 $\boldsymbol { \sigma } _ { i }$ 则根据式 (2. 3. 6)

$$
\frac {\sigma_ {i} (s)}{\omega_ {y} (s)} = \frac {k _ {g}}{s (\tau_ {g} s + 1)}
$$

所以 $G _ { y }$ 陀螺的输出角为

$$
\sigma (s) = \sigma_ {c} (s) + \sigma_ {i} (s) = - \frac {k _ {g}}{s (\tau_ {g} s + 1)} \left[ \omega_ {\mathrm{cmdy}} (s) - \omega_ {y} (s) \right]
$$

$G _ { y }$ 陀螺的信号器输出为

$$
u _ {\mathrm{gy}} ^ {a} (s) = - \frac {k _ {\mathrm{g}} k _ {u}}{s (\tau_ {\mathrm{g}} s + 1)} \left[ \omega_ {\mathrm{cmdy}} (s) - \omega_ {\mathrm{y}} (s) \right]
$$

由于 $G _ { x }$ 陀螺指令角速度为零，所以

$$
u _ {g x} ^ {a} (s) = 0
$$

经方位坐标分解和俯仰正割分解后，输入俯仰伺服放大器的电压信号为

$$
\begin{array}{r l} u _ {g x} ^ {A} (s) & = k _ {\mathrm{ACR}} \left[ u _ {g x} ^ {a} (s) \cos \theta_ {a} - u _ {g y} ^ {a} (s) \sin \theta_ {a} \right] \\ & = \frac {k _ {g} k _ {u} k _ {\mathrm{ACR}}}{s (\tau_ {g} s + 1)} \left[ \omega_ {\mathrm{cmdy}} (s) - \omega_ {y} (s) \right] \sin \theta_ {a} \end{array}
$$

输入横滚伺服放大器的电压信号为

$$
\begin{array}{r l} u _ {g y} ^ {A} (s) & = k _ {\mathrm{ACR}} k _ {\mathrm{SR}} \left[ u _ {g x} ^ {a} (s) \sin \theta_ {a} + u _ {g y} ^ {a} (s) \cos \theta_ {a} \right] \sec \theta_ {f} \\ & = - \frac {k _ {g} k _ {u} k _ {\mathrm{ACR}} k _ {\mathrm{SR}}}{s (\tau_ {g} s + 1)} \left[ \omega_ {\mathrm{cmdy}} (s) - \omega_ {y} (s) \right] \cos \theta_ {a} \end{array}
$$

上述信号经放大校正后，分别馈人俯仰力矩电机和横滚力矩电机，俯仰力矩电机产生的伺服力矩为

$$
M _ {m x} ^ {f} (s) = - \frac {k _ {a f} k _ {m f}}{R _ {f}} F _ {f} (s) u _ {g x} ^ {A} (s) = - \frac {k _ {g} k _ {u} k _ {\mathrm{ACR}} k _ {a f} k _ {m f} F _ {f} (s)}{s (\tau_ {g} s + 1) R _ {f}} [ \omega_ {\mathrm{cmdy}} (s) - \omega_ {y} (s) ] \sin \theta_ {a}
$$

横滚力矩电机产生的伺服力矩为

$$
M _ {m y} ^ {r} (s) = - \frac {k _ {a r} k _ {m r}}{R _ {r}} F _ {r} (s) u _ {g y} ^ {A} (s) = \frac {k _ {g} k _ {u} k _ {\mathrm{ACR}} k _ {\mathrm{SR}} k _ {a r} k _ {m r} F _ {r} (s)}{s \left(\tau_ {g} s + 1\right) R _ {r}} \left[ \omega_ {\mathrm{cmdy}} (s) - \omega_ {y} (s) \right] \cos \theta_ {a}
$$

假设基座无角运动，则在上述力矩作用下，平台绕俯仰环轴 $x _ { f }$ 和横滚环轴 $y _ { i }$ 的角速度为

$$
\omega_ {i f x} ^ {f} (s) = \frac {M _ {m x} ^ {f} (s)}{s J _ {f}} = - \frac {k _ {g} k _ {u} k _ {\mathrm{ACR}} k _ {a f} k _ {m f} F _ {f} (s)}{s ^ {2} (\tau_ {g} s + 1) R _ {f} J _ {f}} \left[ \omega_ {\mathrm{cmdy}} (s) - \omega_ {\mathrm{y}} (s) \right] \sin \theta_ {a}
$$

$$
\omega_ {i r y} ^ {r} (s) = \frac {M _ {m y} ^ {r} (s)}{s J _ {r}} = \frac {k _ {g} k _ {u} k _ {\mathrm{ACR}} k _ {\mathrm{SR}} k _ {a r} k _ {m r} F _ {r} (s)}{s ^ {2} \left(\tau_ {g} s + 1\right) R _ {r} J _ {r}} \left[ \omega_ {\mathrm{cmdy}} (s) - \omega_ {y} (s) \right] \cos \theta_ {a}
$$

而 $\omega _ { y }$ 是 $\omega _ { i f x } ^ { f }$ 和 $\omega _ { i r y } ^ { r }$ 合成的结果，由图 5.3.3

$$
\begin{array}{l} \omega_ {y} (s) = \sqrt {\left[ \omega_ {i f x} ^ {f} (s) \right] ^ {2} + \left[ \omega_ {i r y} ^ {r} (s) \right] ^ {2}} \\ = \frac {k _ {g} k _ {u} k _ {\mathrm{ACR}}}{s ^ {2} (\tau_ {g} s + 1)} \left[ \omega_ {\mathrm{cmdy}} (s) - \omega_ {y} (s) \right] \left\{\left[ \frac {k _ {a f} k _ {m f} F _ {f} (s) \sin \theta_ {a}}{R _ {f} J _ {f}} \right] ^ {2} + \left[ \frac {k _ {\mathrm{SR}} k _ {a r} k _ {m r} F _ {r} (s) \cos \theta_ {a}}{R _ {r} J _ {r}} \right] ^ {2} \right\} ^ {\frac {1}{2}} \end{array}
$$

设 $\omega _ { y }$ 与横滚轴间的夹角为 $\theta _ { a } ^ { \prime }$ ,则

$$
\tan \theta_ {a} ^ {\prime} = \frac {\frac {k _ {g} k _ {u} k _ {\mathrm{ACR}} k _ {a f} k _ {m f} F _ {f} (s)}{s ^ {2} (\tau_ {g} s + 1) R _ {f} J _ {f}} \left[ \omega_ {\mathrm{cmdy}} (s) - \omega_ {y} (s) \right] \sin \theta_ {a}}{\frac {k _ {g} k _ {u} k _ {\mathrm{ACR}} k _ {\mathrm{SR}} k _ {a r} k _ {m r} F _ {r} (s)}{s ^ {2} (\tau_ {g} s + 1) R _ {r} J _ {r}} \left[ \omega_ {\mathrm{cmdy}} (s) - \omega_ {y} (s) \right] \cos \theta_ {a}} = \frac {\frac {k _ {a f} k _ {m f} F _ {f} (s)}{R _ {f} J _ {f}}}{\frac {k _ {\mathrm{SR}} k _ {a r} k _ {m r} F _ {r} (s)}{R _ {r} J _ {r}}} \tan \theta_ {a}
$$

如果在设计中取

$$
\frac {k _ {a f} k _ {m f}}{R _ {f} J _ {f}} = \frac {k _ {\mathrm{SR}} k _ {a r} k _ {m r}}{R _ {r} J _ {r}} = K\tag{5.3.4}
$$

$$
F _ {f} (s) = F _ {r} (s) = F (s)\tag{5.3.5}
$$

!iltl $\tan \theta _ { a } ^ { \prime } = \tan \theta _ { a }$ ，即

$$
\theta_ {a} ^ {\prime} = \theta_ {a}\tag{5.3.6}
$$

$$
\omega_ {y} (s) = \frac {k _ {g} k _ {u} k _ {\mathrm{ACR}}}{s ^ {2} (\tau_ {g} s + 1)} K F (s) \left(\sin^ {2} \theta_ {a} + \cos^ {2} \theta_ {a}\right) ^ {\frac {1}{2}} \left[ \omega_ {\mathrm{cmdy}} (s) - \omega_ {y} (s) \right]
$$

今 $K _ { \Sigma } { = } k _ { \mathrm { g } } k _ { u } k _ { \mathrm { A C R } } K$ ，则

$$
\begin{array}{l} \omega_ {y} (s) = \frac {K _ {\Sigma} F (s)}{s ^ {2} (\tau_ {g} s + 1)} [ \omega_ {\mathrm{cmdy}} (s) - \omega_ {y} (s) ] \\ \omega_ {y} (s) = \frac {\frac {K _ {\Sigma} F (s)}{s ^ {2} (\tau_ {g} s + 1)}}{1 + \frac {K _ {\Sigma} F (s)}{s ^ {2} (\tau_ {g} s + 1)}} \omega_ {\mathrm{cmdy}} (s) = \frac {K _ {\Sigma} F (s)}{s ^ {2} (\tau_ {g} s + 1) + K _ {\Sigma} F (s)} \omega_ {\mathrm{cmdy}} (s) \end{array}
$$

对于常值指令角速度， ${ \omega } _ { \mathrm { c m d } y } \left( s \right) = \frac { { \omega } _ { \mathrm { c m d y } } } { s }$ ，平台进动角速度的稳态值为

$$
\omega_ {y s s} = \lim _ {s \rightarrow 0} \cdot \omega_ {y} (s) = \omega_ {\mathrm{cmdy}}
$$

上述分析说明，只要平台设计中满足式(5.3.4) 、式(5.3.5) ，平台就能准确跟踪指令角速度。

## 5.4 三轴平台的方位锁定原理

当主轴平台使用双自由度陀螺仪作为敏感器件时，需要两个陀螺仪。而双自由度陀螺仪有两个敏感轴，控制主轴平台时有一个敏感轴冗余。这个冗余轴必须控制住使之随平台一起旋转，即陀螺在该轴上的信号器输出经放大后馈人控制该轴的力矩器中，使陀螺在该轴上形成闭环，工作在力反馈状态。如果该轴与平台的伺服回路未形成闭环而处于自由状态，由于陀螺的定轴性，台体旋转但转子轴未转动，则这种相对转动有可能使陀螺和止档装置发生碰撞影响正常工作，甚至会损坏陀螺。

5.4.1 为双自由度陀螺仪在三轴平台上的哑铃式配置示意图，其中上陀螺 $G _ { T }$ 的输出角$\beta _ { y }$ 用来控制平台的 轴， $\alpha _ { z T }$ 用来控制平台的 $Z$ 下陀螺 $G _ { B }$ 的输出角 $\beta _ { x }$ 用来控制平台的轴，输出角 $\alpha _ { z B }$ 冗余。

设平台在指令控制下方位改变 $\Delta \Psi .$ ，若下陀螺 $G _ { B }$ 的外环轴不作控制，则由于陀螺的定轴性，$H _ { B }$ 仍然指向 $Y ( 0 )$ 而平台的方位环已转至 $Y ( t )$ ，下陀螺的外环有可能与国联在方位环上的限位装置相碰而损坏陀螺。如果 $H _ { B }$ 随 $Y$ 轴一起进动，则陀螺外环相对平台方位环的位置保持不变，就不会发生碰撞。因此必须使下陀螺的外环轴处于锁定状态，具体做法是 下陀螺 $G _ { B }$ 外环轴上的信号器输出经放大后馈人 $G _ { B }$ 内环轴上的力矩器中，在该锁定力矩作用 $G _ { B }$ 的转子绕外环轴进动，进动角速度与航向角变化率 ，p相同，这样就实现了下陀螺在方位上的锁定。从上述分析可看出，锁定实质是使陀螺处于力反馈工作状态，跟随的角速度越大，加入力矩器中的锁定电流就越大，但锁定电流受力矩器的承载能力限制，锁定电流是不能太大的。诸如飞机、火箭、舰艇等运载体的俯仰角速度和横滚角速度都比航向变化率大，所以三轴平台中，被锁定的轴一般应选择方位轴，即采用方位锁定来解决一个敏感轴的冗余问题。

![](images/e9893aa945ddcff87c78b679cb6dbd7d5d248dbf0f05f17f6c6fa90e28f21be0.jpg)  
5.4.1 双自由度陀螺仪在兰轴平台上的哑铃式配置

## 5. 5 兰轴平台的动力学方程和系统的传递函数阵

从单轴平台伺服回路设计介绍中知，当平台的机械参数(如环架的转动惯量)和元件(如陀螺、和力矩电机等)的传递函数和机电参数给定后，要使平台系统的动静态性能达到设计要求，就必须合理地设计电子线路和选择校正网络的形式和参数，这一工作可在频域内进行，即以频率特性作为设计对象，使平台系统具有满足要求的频率特性，也可在复频域内进行，即采用零、极点配置法使平台系统的传递函数具有合适的零、极点。兰轴平台与单轴平台的设计是类似的，不同点在于三轴平台的动力方程是三维向量方程，诸如陀螺仪控制量(指令角速度)、干扰输入量(陀螺漂移、干扰力矩及基座角速度)、输出量(台体的角速度)、误差输出量(平台漂移角速度和平台误差角)等都是三维向量，所以三轴平台实际上是一个多输入多输出系统，输入输出之间存在交叉相合影响，设计兰轴平台要比设计单轴平台的难度高。工程上常采用简化设计方法，即先按单通道设计伺服回路，然后校验三轴间的交叉相合影响及误差大小，看其是否满足系统的设计要求，若不满足，应重新修改。

## 5.5.1 三轴平台的原理方块图

根据图 5. 2. 所示三轴平台的信息处理流程图，可画出三轴平台的原理方块图，如图 5.5.1所示，图中双线箭头表示兰维向量间的信息传递。下面对图中各环节的传递函数进行分析。

![](images/e7a1c0f4371be29930723526e70b60b5cdb26150c895b1844ba6c8b50cedb5f9.jpg)  
5.5.1 兰轴平台原理方块图

1.陀螺组件的传递函数阵和输出轴耦合阵

1)采用单自由度积分陀螺时的分析

5.5.2 为采用单自由度积分陀螺时，在三轴平台上的陀螺安装布置方案。图中取 XYZ平台坐标系 $x _ { a } y _ { a } z _ { a }$ 为方位环坐标系，由陀螺的敏感轴 $I _ { x } , I _ { y }$ Iz 确定。

![](images/a5ee1f99f665d87e7d927a77d790db09b57c5ff256c4c149072c15a0fbe8cc00.jpg)  
5.5.2 单自由度陀螺仪在平台上的配置方案之一

设各陀螺的角动量均为 H.绕各自输出轴的转动惯量均为 $I _ { g }$ .阻尼系数均为 ，对平台的指令角速度为 $\begin{array} { r } { \pmb { \omega } _ { \mathrm { c m d } } = \left[ \omega _ { \mathrm { c m d } x } \mathrm { ~ \omega ~ } \omega _ { \mathrm { c m d y } } \mathrm { ~ \omega ~ } \omega _ { \mathrm { c m d z } } \right] ^ { \mathrm { T } } } \end{array}$ .平台的输出角速度为 $\mathbf { \omega } \mathbf { \omega } \mathbf { \omega } \mathbf { \omega } \omega = [ \omega _ { x } \quad \omega _ { y } \quad \omega _ { z } ] ^ { \mathrm { T } }$ ，陀螺 $G _ { x } \ : , G _ { y }$ $G _ { z }$ 的输出角分别为 $\alpha _ { g x } \backprime \alpha _ { g y } \backprime \alpha _ { g z }$ 。则各陀螺的输出方程为

$$
I _ {g} \left(\dot {\omega} _ {y} + \ddot {\alpha} _ {g x}\right) = H \omega_ {x} - D \dot {\alpha} _ {g x} + T _ {\mathrm{cmd} x} + T _ {D x}\tag{5.5.1a}
$$

$$
I _ {g} \left(\dot {\omega} _ {z} + \ddot {\alpha} _ {g y}\right) = H \omega_ {y} - D \dot {\alpha} _ {g y} + T _ {\mathrm{cmdy}} + T _ {D y}\tag{5.5.1b}
$$

$$
I _ {g} \left(\dot {\omega} _ {x} + \ddot {\alpha} _ {g z}\right) = H \omega_ {z} - D \dot {\alpha} _ {g z} + T _ {\mathrm{cmd} z} + T _ {D z}\tag{5.5.1c}
$$

根据式 (2. 3. 7)

$$
\begin{array}{r l r} T _ {\mathrm{cmd} x} = - H \omega_ {\mathrm{cmd} x}, & T _ {\mathrm{cmd} y} = - H \omega_ {\mathrm{cmd} y}, & T _ {\mathrm{cmd} z} = - H \omega_ {\mathrm{cmd} z} \\ T _ {D x} = - H \varepsilon_ {g x}, & T _ {D y} = - H \varepsilon_ {g y}, & T _ {D z} = - H \varepsilon_ {g z} \end{array}
$$

式中 . $, \varepsilon _ { g x } \setminus \varepsilon _ { g y } \setminus \varepsilon _ { g z }$ 』分别为陀螺 $G _ { x } , G _ { y } , G _ { z }$ ，的漂移。

(5.5. 1)可写成

$$
\begin{array}{r l} (I _ {g} s ^ {2} + D s) \alpha_ {g x} (s) & = H \omega_ {x} (s) - I _ {g} s \omega_ {y} (s) - H [ \omega_ {\mathrm{cmd} x} (s) + \varepsilon_ {g x} (s) ] \\ \alpha_ {g x} (s) & = \frac {1}{s (\tau_ {g} s + 1)} \left\{k _ {g} \omega_ {x} (s) - \tau_ {g} s \omega_ {y} (s) - k _ {g} [ \omega_ {\mathrm{cmd} x} (s) + \varepsilon_ {g x} (s) ] \right\} \\ & = - \frac {k _ {g}}{s (\tau_ {g} s + 1)} [ \omega_ {\mathrm{cmd} x} (s) + \varepsilon_ {g x} (s) ] + \frac {k _ {g}}{s (\tau_ {g} s + 1)} [ \omega_ {x} (s) - \frac {\tau_ {g}}{k _ {g}} s \omega_ {y} (s) ] \end{array}
$$

式中， ${ , } \tau _ { g } = \frac { I _ { g } } { D } , k _ { g } = \frac { H } { D } ,$ 。同理可得

$$
\begin{array}{l} \alpha_ {g y} (s) = - \frac {k _ {g}}{s (\tau_ {g} s + 1)} [ \omega_ {\mathrm{cmdy}} (s) + \varepsilon_ {g y} (s) ] + \frac {k _ {g}}{s (\tau_ {g} s + 1)} \left[ \omega_ {y} (s) - \frac {\tau_ {g}}{k _ {g}} s \omega_ {z} (s) \right] \\ \alpha_ {g z} (s) = - \frac {k _ {g}}{s (\tau_ {g} s + 1)} [ \omega_ {\mathrm{cmdz}} (s) + \varepsilon_ {g z} (s) ] + \frac {k _ {g}}{s (\tau_ {g} s + 1)} \left[ \omega_ {z} (s) - \frac {\tau_ {g}}{k _ {g}} s \omega_ {x} (s) \right] \end{array}
$$

即

$$
\begin{array}{l} \left[ \begin{array}{l} \alpha_ {g x} (s) \\ \alpha_ {g y} (s) \\ \alpha_ {g z} (s) \end{array} \right] = - \frac {k _ {g}}{s (\tau_ {g} s + 1)} I \left[ \begin{array}{l} \omega_ {\mathrm{cmd} x} (s) + \varepsilon_ {g x} (s) \\ \omega_ {\mathrm{cmd} y} (s) + \varepsilon_ {g y} (s) \\ \omega_ {\mathrm{cmd} z} (s) + \varepsilon_ {g z} (s) \end{array} \right] + \frac {k _ {g}}{s (\tau_ {g} s + 1)} \left[ \begin{array}{c c c} 1 & - \frac {s \tau_ {g}}{k _ {g}} & 0 \\ 0 & 1 & - \frac {s \tau_ {g}}{k _ {g}} \\ - \frac {s \tau_ {g}}{k _ {g}} & 0 & 1 \end{array} \right] \left[ \begin{array}{l} \omega_ {x} (s) \\ \omega_ {y} (s) \\ \omega_ {z} (s) \end{array} \right] \\ = - \frac {k _ {g}}{s (\tau_ {g} s + 1)} I \left\{\left[ \begin{array}{l} \omega_ {\mathrm{cmd} x} (s) \\ \omega_ {\mathrm{cmd} y} (s) \\ \omega_ {\mathrm{cmd} z} (s) \end{array} \right] + \left[ \begin{array}{l} \varepsilon_ {g x} (s) \\ \varepsilon_ {g y} (s) \\ \varepsilon_ {g z} (s) \end{array} \right] - \left[ \begin{array}{l} \omega_ {x} (s) \\ \omega_ {y} (s) \\ \omega_ {z} (s) \end{array} \right] \right\} \\ - \frac {k _ {g}}{s (\tau_ {g} s + 1)} I \left[ \begin{array}{c c c} 0 & \frac {s \tau_ {g}}{k _ {g}} & 0 \\ 0 & 0 & \frac {s \tau_ {g}}{k _ {g}} \\ \frac {s \tau_ {g}}{k _ {g}} & 0 & 0 \end{array} \right] \left[ \begin{array}{l} \omega_ {x} (s) \\ \omega_ {y} (s) \\ \omega_ {z} (s) \end{array} \right] \end{array} \tag {5.5}\tag{5.5.2}
$$

令

$$
\mathbf {Y} _ {1} (s) = \frac {k _ {g}}{s (\tau_ {g} s + 1)} \mathbf {I}\tag{5.5.3}
$$

$$
\mathbf {Y} _ {2} (s) = \frac {s \tau_ {g}}{k _ {g}} \left[ \begin{array}{c c c} 0 & 1 & 0 \\ 0 & 0 & 1 \\ 1 & 0 & 0 \end{array} \right] = \frac {s I _ {g}}{H} \left[ \begin{array}{c c c} 0 & 1 & 0 \\ 0 & 0 & 1 \\ 1 & 0 & 0 \end{array} \right]\tag{5.5.4}
$$

$\mathbf { \cal { Y } } _ { 2 } \left( s \right)$ 反映了陀螺输出之间的耦合关系，所以称之为耦合阵。则式(5. 5. 2)可写成

$$
\begin{array}{r l} \alpha_ {g} ^ {a} (s) = & - \mathbf {Y} _ {1} (s) \left[ \omega_ {\mathrm{cmd}} ^ {a} (s) + \varepsilon_ {g} ^ {a} (s) - \omega_ {k i} ^ {a} (s) \right] - \mathbf {Y} _ {1} (s) \mathbf {Y} _ {2} (s) \omega_ {k i} ^ {a} (s) \\ = & - \mathbf {Y} _ {1} (s) \left\{\omega_ {\mathrm{cmd}} ^ {a} (s) + \varepsilon_ {g} ^ {a} (s) - [ \mathbf {I} - \mathbf {Y} _ {2} (s) ] \omega_ {k i} ^ {a} (s) \right\} \end{array}\tag{5.5.5}
$$

若陀螺配置采用图 5.5.3 所示方案，则 $\mathbf { { \cal Y } } _ { 1 } \left( s \right)$ 与式(5. 5. 3)所列相同，耦合阵则成

$$
\mathbf {Y} _ {2} (s) = \frac {s \tau_ {g}}{k _ {g}} \left[ \begin{array}{c c c} 0 & 1 & 0 \\ 1 & 0 & 0 \\ 1 & 0 & 0 \end{array} \right]\tag{5.5.6}
$$

陀螺信号器输出为

$$
\boldsymbol {u} _ {g} ^ {a} (s) = - k _ {u} \boldsymbol {I} \boldsymbol {\alpha} _ {g} ^ {a} (s)\tag{5.5.7}
$$

![](images/a49f4e1bd263f69338118068f99b641fb6c1589c4425cc6ba5dafadfab0d46ba.jpg)  
5.5.3 单自由度陀螺仪在平台上的配置方案之二

2) 采用双自由度角位置陀螺时的分析

根据式 (2.2. 10) 和式 (2.2. 11). 可得输出角的拉氏表达式:

$$
\left[ \begin{array}{l} \alpha_ {g x} (s) \\ \alpha_ {g y} (s) \\ \alpha_ {g z} (s) \end{array} \right] = \frac {1}{s} \left[ \begin{array}{l} \omega_ {\mathrm{cmd} r} (s) + \varepsilon_ {g x} (s) - \omega_ {x} (s) \\ \omega_ {\mathrm{cmd} y} (s) + \varepsilon_ {g y} (s) - \omega_ {y} (s) \\ \omega_ {\mathrm{cmd} z} (s) + \varepsilon_ {g z} (s) - \omega_ {z} (s) \end{array} \right]
$$

即

$$
\alpha_ {g} ^ {a} (s) = \frac {1}{s} I \left[ \omega_ {\mathrm{cmd}} ^ {a} (s) + \varepsilon_ {g} ^ {a} (s) - \omega_ {i a} ^ {a} (s) \right]
$$

所以

$$
\mathbf {Y} _ {1} (s) = \frac {1}{s} \mathbf {I}\tag{5.5.8}
$$

耦合阵

$$
\mathbf {Y} _ {2} (s) = 0\tag{5.5.9}
$$

陀螺信号器输出为

$$
\boldsymbol {u} _ {g} ^ {a} (s) = k _ {u} \boldsymbol {I} \boldsymbol {\alpha} _ {g} ^ {a} (s)\tag{5.5.10}
$$

3) 采用挠性陀螺时的分析

根据图 2.4.6 和式 (2.4.29) .得

$$
\left[ \begin{array}{l} \alpha_ {g x} (s) \\ \alpha_ {g y} (s) \\ \alpha_ {g z} (s) \end{array} \right] = \frac {\tau_ {g}}{\tau_ {g} s + 1} \left[ \begin{array}{l} \omega_ {\mathrm{cmd} x} (s) + \varepsilon_ {g x} (s) - \omega_ {x} (s) \\ \omega_ {\mathrm{cmd} y} (s) + \varepsilon_ {g y} (s) - \omega_ {y} (s) \\ \omega_ {\mathrm{cmd} z} (s) + \varepsilon_ {g z} (s) - \omega_ {z} (s) \end{array} \right]
$$

挠性陀螺的时间常数一般都很大 $\frac { \tau _ { g } } { \tau _ { g } s + 1 } { \approx } \frac { 1 } { s }$ 所以上式可改写成

$$
\alpha_ {g} ^ {a} (s) = \frac {1}{s} I \left[ \omega_ {\mathrm{cmd}} ^ {a} (s) + \varepsilon_ {g} ^ {a} (s) - \omega_ {i a} ^ {a} (s) \right]
$$

可见与双自由度角位置陀螺稳定平台一样，

$$
\mathbf {Y} _ {1} (s) = \frac {1}{s} \mathbf {I}\tag{5.5.11}
$$

$$
\mathbf {Y} _ {2} (s) = 0\tag{5.5.12}
$$

陀螺信号器输出为

$$
\pmb {u} _ {g} ^ {a} (s) = k _ {u} \pmb {I} \alpha_ {g} ^ {a} (s)\tag{5.5.13}
$$

## 2. 伺服放大器(平台电子网络)的传递函数阵

设经方位坐标分解器和俯仰正割分解器处理后的输出信号为 $U _ { g } ^ { A } \left( s \right)$ 则控制俯仰环、横滚环和方位环力矩电机的电压为

$$
\boldsymbol {u} _ {g} ^ {A} (s) = \boldsymbol {Y} _ {3} \boldsymbol {U} _ {g} ^ {A} (s)\tag{5.5.14}
$$

式中

$$
\boldsymbol {U} _ {g} ^ {A} (s) = \boldsymbol {T} _ {2} \boldsymbol {u} _ {g} ^ {a} (s) \cdot \operatorname{diag} \left[ k _ {\mathrm{ACR}} \quad k _ {\mathrm{ACR}} k _ {\mathrm{SR}} \quad 1 \right]\tag{5.5.15}
$$

$$
\mathbf {Y} _ {3} = \operatorname{diag} \left[ \begin{array}{l l l} Y _ {f _ {1}} (s) & Y _ {r _ {1}} (s) & Y _ {a _ {1}} (s) \end{array} \right]\tag{5.5.16}
$$

式中 $, Y _ { f _ { 1 } } \left( s \right) = k _ { a f } F _ { f } \left( s \right) , Y _ { r _ { 1 } } \left( s \right) = k _ { a r } F _ { r } \left( s \right) , Y _ { a _ { 1 } } \left( s \right) = k _ { a a } F _ { a } \left( s \right) ; k _ { a f } \mathrm { ~ } , k _ { a r } \mathrm { ~ } , k _ { a a }$ 分别为俯仰环、横滚环、方位环电子网络的总放大倍数. $F _ { f } ( s ) \lrcorner F _ { r } ( s ) \lrcorner F _ { a } ( s )$ 分别为俯仰环、横滚环、方位环校正网络的传递函数。

## 3. 力矩电机的传递 $z ^ { \vec { \cdot } }$ 数矩阵

忽略电机的时间常数

$$
M _ {m f} (s) = \frac {k _ {m f}}{R _ {f}} u _ {g x} ^ {A} (s), \quad M _ {m r} (s) = \frac {k _ {m r}}{R _ {r}} u _ {g y} ^ {A} (s), \quad M _ {m a} (s) = \frac {k _ {m a}}{R _ {a}} u _ {g z} ^ {A} (s)
$$

即

$$
\left[ \begin{array}{l} M _ {m f} (s) \\ M _ {m r} (s) \\ M _ {m a} (s) \end{array} \right] = \left[ \begin{array}{c c c} \frac {k _ {m f}}{R _ {f}} & 0 & 0 \\ 0 & \frac {k _ {m r}}{R _ {r}} & 0 \\ 0 & 0 & \frac {k _ {m a}}{R _ {a}} \end{array} \right] \left[ \begin{array}{l} u _ {g x} ^ {A} (s) \\ u _ {g y} ^ {A} (s) \\ u _ {g z} ^ {A} (s) \end{array} \right]
$$

或

$$
\boldsymbol {M} _ {m} ^ {A} = \boldsymbol {Y} _ {4} (s) \boldsymbol {u} _ {g} ^ {A} (s)\tag{5.5.17}
$$

式中

$$
\mathbf {Y} _ {4} (s) = \operatorname{diag} \left[ \begin{array}{c c c} k _ {m f} & k _ {m r} & k _ {m a} \\ \hline R _ {f} & R _ {r} & R _ {a} \end{array} \right]\tag{5.5.18}
$$

## 4. 环架系统的传递函数矩阵

设作用在俯仰轴、横滚轴、方位轴上的综合力矩分别为 $M _ { \Sigma x } ^ { f } \ , M _ { \Sigma y } ^ { r } \ , M _ { \Sigma z } ^ { a }$ 且也，则根据参考文(Broxmayer C, 1972)

$$
\left[ \begin{array}{c c c} J _ {f x} + \left(J _ {a x} \cos^ {2} \theta_ {a} + J _ {a y} \sin^ {2} \theta_ {a}\right) & \left(J _ {a x} - J _ {a y}\right) \sin \theta_ {a} \cos \theta_ {a} \cos \theta_ {f} & 0 \\ \left(J _ {a x} - J _ {a y}\right) \sin \theta_ {a} \cos \theta_ {a} \cos \theta_ {f} & J _ {r y} + \left(J _ {f y} \cos^ {2} \theta_ {f} + J _ {f z} \sin^ {2} \theta_ {f}\right) & J _ {a z} \sin \theta_ {f} \\ & + \left(J _ {a x} \cos^ {2} \theta_ {a} + J _ {a y} \sin^ {2} \theta_ {a}\right) \cos^ {2} \theta_ {f} & \\ 0 & 0 & J _ {a z} \end{array} \right] \left[ \begin{array}{l} \ddot {\theta} _ {f} \\ \ddot {\theta} _ {r} \\ \ddot {\theta} _ {a} \end{array} \right] = \left[ \begin{array}{l} M _ {\Sigma x} ^ {f} \\ M _ {\Sigma y} ^ {r} \\ M _ {\Sigma z} ^ {a} \end{array} \right]
$$

实际系统设计中常取 $J _ { a x } { \approx } J _ { a y }$ '所以上式可近似为

$$
\left[ \begin{array}{c c c} J _ {f x} + J _ {a x} & 0 & 0 \\ 0 & J _ {r y} + \left(J _ {f y} \cos^ {2} \theta_ {f} + J _ {f z} \sin^ {2} \theta_ {f}\right) + J _ {a x} \cos^ {2} \theta_ {f} & J _ {a z} \sin \theta_ {f} \\ 0 & 0 & J _ {a z} \end{array} \right] \left[ \begin{array}{c} \ddot {\theta} _ {f} \\ \ddot {\theta} _ {r} \\ \ddot {\theta} _ {o} \end{array} \right] = \left[ \begin{array}{c} M _ {\Sigma x} ^ {f} \\ M _ {\Sigma y} ^ {r} \\ M _ {\Sigma z} ^ {a} \end{array} \right]
$$

记

$$
\begin{array}{l} I _ {f x} = J _ {f x} + J _ {a x} \\ I _ {r y} = J _ {r y} + (J _ {f y} \cos^ {2} \theta_ {f} + J _ {f z} \sin^ {2} \theta_ {f}) + J _ {a x} \cos^ {2} \theta_ {f} \\ I _ {a z} = J _ {a z} \end{array}
$$

则上式写成

$$
\left[ \begin{array}{c c c} s I _ {f x} & 0 & 0 \\ 0 & s I _ {r y} & s I _ {a z} \sin \theta_ {f} \\ 0 & 0 & s I _ {a z} \end{array} \right] \left[ \begin{array}{c} \dot {\theta} _ {f} (s) \\ \dot {\theta} _ {r} (s) \\ \dot {\theta} _ {a} (s) \end{array} \right] = \left[ \begin{array}{c} M _ {\Sigma x} ^ {f} (s) \\ M _ {\Sigma y} ^ {r} (s) \\ M _ {\Sigma z} ^ {a} (s) \end{array} \right]
$$

$$
\left[ \begin{array}{c} \dot {\theta} _ {f} (s) \\ \dot {\theta} _ {r} (s) \\ \dot {\theta} _ {a} (s) \end{array} \right] = \left[ \begin{array}{c c c} s I _ {f x} & 0 & 0 \\ 0 & s I _ {r y} & s I _ {a z} \sin \theta_ {f} \\ 0 & 0 & s I _ {a z} \end{array} \right] ^ {- 1} \left[ \begin{array}{c} M _ {\Sigma x} ^ {f} (s) \\ M _ {\Sigma y} ^ {r} (s) \\ M _ {\Sigma z} ^ {a} (s) \end{array} \right] = \left[ \begin{array}{c c c} \frac {1}{s I _ {f x}} & 0 & 0 \\ 0 & \frac {I}{s I _ {r y}} & - \frac {\sin \theta_ {f}}{s I _ {r y}} \\ 0 & 0 & \frac {1}{s I _ {a z}} \end{array} \right] \left[ \begin{array}{c} M _ {\Sigma x} ^ {f} (s) \\ M _ {\Sigma y} ^ {r} (s) \\ M _ {\Sigma z} ^ {a} (s) \end{array} \right]
$$

即

$$
\boldsymbol {\omega} _ {i A} ^ {A} = \mathbf {Y} _ {5} (s) \mathbf {M} _ {\Sigma} ^ {A} (s)\tag{5.5.19}
$$

式中

$$
\mathbf {Y} _ {5} (s) = \left[ \begin{array}{c c c} \frac {1}{s I _ {f x}} & 0 & 0 \\ 0 & \frac {1}{s I _ {r y}} & - \frac {\sin \theta_ {f}}{s I _ {r y}} \\ 0 & 0 & \frac {1}{s I _ {a z}} \end{array} \right]\tag{5.5.20}
$$

## 5.5.2 三轴平台前向通道的传递函数矩阵

根据图 5.5.1 得三轴平台前向通道的传递函数阵为

$$
\mathbf {Y} (s) = \mathbf {T} _ {1} \mathbf {Y} _ {5} (s) \mathbf {Y} _ {4} (s) \mathbf {Y} _ {3} (s) \mathbf {T} _ {2} (s) k _ {u} \mathbf {I Y} _ {1} (s)\tag{5.5.21}
$$

分别将式 (5. 2. 7) 、式 (5. 5. 20) 、式 (5. 5. 18) 、式 (5. 5. 16) 、式 (5.2.15)代入上式，得

$$
\begin{array} { r l } \mathbf { Y } ( s ) & { } = \left[ \begin{array} { c c c } \cos \theta _ { a } & \cos \theta _ { f } \sin \theta _ { a } & 0 \\ - \sin \theta _ { a } & \cos \theta _ { f } \cos \theta _ { a } & 0 \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array} { c c c } \frac { 1 } { s I _ { f x } } & 0 & 0 \\ 0 & \frac { 1 } { s I _ { r y } } & - \frac { \sin \theta _ { f } } { s I _ { r y } } \\ 0 & 0 & \frac { 1 } { s I _ { a z } } \end{array} \right] \left[ \begin{array} { c c c } \frac { k _ { m f } } { R _ { f } } & 0 & 0 \\ 0 & \frac { k _ { m r } } { R _ { r } } & 0 \\ 0 & 0 & \frac { k _ { m a } } { R _ { a } } \end{array} \right] \\ & { } \times \left[ \begin{array} { c c c } k _ { a f } F _ { f } ( s ) & 0 & 0 \\ 0 & k _ { a r } F _ { r } ( s ) & 0 \\ 0 & 0 & k _ { a a } F _ { a } ( s ) \end{array} \right] \left[ \begin{array} { c c c } k _ { A C R } & 0 & 0 \\ 0 & k _ { A C R } k _ { S R } & 0 \\ 0 & 0 & 1 \end{array} \right] \\ & { } \times \left[ \begin{array} { c c c } \cos \theta _ { a } & 0 & 0 \\ \sin \theta _ { a } \sec \theta _ { f } & \cos \theta _ { a } \sec \theta _ { f } & 0 \\ 0 & 0 & 1 \end{array} \right] k _ { u } \mathbf { Y } _ { 1 } ( s ) \\ & { } = \frac { 1 } { s } \left[ \begin{array} { c c c } \frac { \cos \theta _ { a } } { I _ { f x } } & \frac { \cos \theta _ { f } \sin \theta _ { a } } { I _ { r y } } & - \frac { \sin 2 \theta _ { f } \sin \theta _ { a } } { 2 I _ { r y } } \\ - \frac { \sin \theta _ { a } } { I _ { f x } } & \frac { \cos \theta _ { f } \cos \theta _ { a } } { I _ { r y } } & - \frac { \sin 2 \theta _ { f } \cos \theta _ { a } } { 2 I _ { r y } } \\ 0 & 0 & \frac { 1 } { I _ { a z } } \end{array} \right] \\ & { } \times \left[ \begin{array} { c c c } \frac { k _ { m f } k _ { a f } k _ { A C R } F _ { f } ( s ) } { R _ { f } } & 0 & 0 \\ 0 & \frac { k _ { m r } k _ { a r } k _ { A C R } k _ { S R } F _ { r } ( s ) } { R _ { r } } & 0 \\ 0 & 0 & \frac { k _ { m a } k _ { a a } F _ { a } ( s ) } { R _ { a } } \end{array} \right] \\ & { } \times \left[ \begin{array} { c c c } \cos \theta _ { a } & - \sin \theta _ { a } & 0 \\ \sin \theta _ { a } \sec \theta _ { f } & \cos \theta _ { a } \sec \theta _ { f } & 0 \\ 0 & 0 & 1 \end{array} \right] k _ { u } \mathbf { Y } _ { 1 } ( s ) \\ & { } = \frac { 1 } { s } \left[ \begin{array}  l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l   =   [   ]   ]   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   .    ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [   ]   [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]     [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]    [    ]        ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       ,       |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |         |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |        |         |         |
|
$$

$$
\begin{array}{r l} & = \left[ \begin{array}{c} \frac {k _ {m f} k _ {a f} k _ {\mathrm{ACR}} F _ {f} (s) \cos^ {2} \theta_ {a}}{I _ {f x} R _ {f}} + \frac {k _ {m r} k _ {a r} k _ {\mathrm{ACR}} k _ {\mathrm{SR}} F _ {r} (s) \sin^ {2} \theta_ {a}}{I _ {r y} R _ {r}} \\ - \frac {k _ {m f} k _ {a f} k _ {\mathrm{ACR}} F _ {f} (s) \sin 2 \theta_ {a}}{2 I _ {f x} R _ {f}} + \frac {k _ {m r} k _ {a r} k _ {\mathrm{ACR}} k _ {\mathrm{SR}} F _ {r} (s) \sin 2 \theta_ {a}}{2 I _ {r y} R _ {r}} \\ 0 \end{array} \right. \\ & - \frac {k _ {m f} k _ {a f} k _ {\mathrm{ACR}} F _ {f} (s) \sin 2 \theta_ {a}}{2 I _ {f x} R _ {f}} + \frac {k _ {m r} k _ {a r} k _ {\mathrm{ACR}} k _ {\mathrm{SR}} F _ {r} (s) \sin 2 \Theta_ {a}}{2 I _ {r y} R _ {r}} - \frac {k _ {m a} k _ {a a} F _ {a} (s) \sin 2 \theta_ {f} \sin \theta_ {a}}{2 I _ {r y} R _ {a}} \\ & \frac {k _ {m f} k _ {a f} k _ {\mathrm{ACR}} F _ {f} (s) \sin^ {2} \theta_ {a}}{I _ {f x} R _ {f}} + \frac {k _ {m r} k _ {a r} k _ {\mathrm{ACR}} k _ {\mathrm{SR}} F _ {r} (s) \cos^ {2} \theta_ {a}}{I _ {r y} R _ {r}} - \frac {k _ {m a} k _ {a a} F _ {a} (s) \sin 2 \theta_ {f} \cos \theta_ {a}}{2 I _ {r y} R _ {a}} \\ 0 & \frac {k _ {m a} k _ {a a} F _ {a} (s)}{I _ {a z} R _ {a}} \end{array} ] k _ {u} Y _ {1} (s)
$$

根据式 (5. 3. 4) 和式 (5. 3. 5) 的设计要求，并记

$$
\frac {k _ {m f} k _ {a f} F _ {f} (s)}{I _ {f x} R _ {f}} = \frac {k _ {m r} k _ {a r} k _ {\mathrm{SR}} F _ {r} (s)}{I _ {r y} R _ {r}} = K F (s)
$$

当采用单自由度积分陀螺作为敏感器件时，即

$$
\mathbf {Y} _ {1} (s) = \frac {k _ {\mathrm{g}}}{s (\tau_ {\mathrm{g}} s + 1)} \mathbf {I}
$$

则

$$
\mathbf {Y} (s) = \left[ \begin{array}{c c c} \frac {k _ {g} k _ {u} k _ {\mathrm{ACR}} K F (S)}{s ^ {2} \left(\tau_ {g} s + 1\right)} & 0 & - \frac {k _ {m a} k _ {a a} F _ {a} (s) \sin 2 \theta_ {f} \sin \theta_ {a}}{2 I _ {r y} R _ {a} s ^ {2} \left(\tau_ {g} s + 1\right)} \\ 0 & \frac {k _ {g} k _ {u} k _ {\mathrm{ACR}} K F (s)}{s ^ {2} \left(\tau_ {g} s + 1\right)} & - \frac {k _ {m a} k _ {a a} F _ {a} (s) \sin 2 \theta_ {f} \cos \theta_ {a}}{2 I _ {r y} R _ {a} s ^ {2} \left(\tau_ {g} s + 1\right)} \\ 0 & 0 & \frac {k _ {m a} k _ {a a} F _ {a} (s)}{I _ {a z} R _ {a} s ^ {2} \left(\tau_ {g} s + 1\right)} \end{array} \right]
$$

记

$$
Y _ {h} (s) = \frac {k _ {g} k _ {u} k _ {\mathrm{ACR}} K F (s)}{s ^ {2} (\tau_ {g} s + 1)}\tag{5.5.22}
$$

$$
Y _ {z} (s) = \frac {k _ {m a} k _ {a a} F _ {a} (s)}{I _ {a z} R _ {a} s ^ {2} (\tau_ {g} s + 1)}\tag{5.5.23}
$$

$$
C _ {1} = - \frac {I _ {a z} \sin 2 \theta_ {f} \sin \theta_ {a}}{2 I _ {r y}}\tag{5.5.24a}
$$

$$
C _ {2} = - \frac {I _ {a z} \sin 2 \theta_ {f} \cos \theta_ {a}}{2 I _ {r y}}\tag{5.5.24b}
$$

则三轴平台的前向通道传递函数阵为

$$
\mathbf {Y} (s) = \left[ \begin{array}{c c c} Y _ {h} (s) & 0 & C _ {1} Y _ {z} (s) \\ 0 & Y _ {h} (s) & C _ {2} Y _ {z} (s) \\ 0 & 0 & Y _ {z} (s) \end{array} \right]\tag{5.5.25}
$$

## 5.5.3 从台体 到环架轴坐标系 的力矩传递矩阵

以俯仰环为研究对象，即取俯仰环为分离体。记 $M _ { a f x } ^ { f } \ . M _ { a f y } ^ { f } \ . M _ { a f z } ^ { f }$ 为方位环 对俯仰环作用力矩。由于几何约束关系，作用在横滚环上的力矩中只有沿 $\mathcal { Z } _ { r }$ 轴和 $y _ { r }$ 轴的力矩可传递给俯仰环，而沿 $\mathcal { X } _ { r }$ 轴(即 $\boldsymbol { \mathcal { X } } _ { f }$ 轴)的力矩由于平台的卸荷作用传递不到俯仰环上。记横滚环对俯仰环的作用力矩为 $M _ { r f z } ^ { r }$ 和 $M _ { \Sigma y } ^ { r }$ 如图 5.5.4Cb)所示。

![](images/49d84ebaa68b998e8b0a68184937bea07f3478164f7dc8d5c1b953cf0eb4cb0e.jpg)

![](images/ce78d08f52cf8d9e936824222d33fc08436e5df5e82b863422d071931a6f65d1.jpg)  
5.5.4 各环架上受到的力矩

当俯仰环 $f$ 处于平衡状态时，沿 $\boldsymbol { \mathcal { X } } _ { f }$ 轴的合力矩为

$$
M _ {\Sigma x} ^ {f} + M _ {a f x} ^ {f} = 0\tag{a}
$$

沿 $y _ { f }$ 轴的合力矩为

$$
M _ {\Sigma y} ^ {r} \cos \theta_ {f} + M _ {r f z} ^ {r} \sin \theta_ {f} + M _ {a f y} ^ {f} = 0\tag{b}
$$

沿 $\boldsymbol { z } _ { f }$ 轴的合力矩为

$$
M _ {\Sigma y} ^ {r} \sin \theta_ {f} + M _ {r f z} ^ {r} \cos \theta_ {f} + M _ {a f z} ^ {f} = 0\tag{c}
$$

由于方位环 $a$ 支承在俯仰环 $f$ 内， $M _ { a f z } ^ { f }$ 是 $a$ 环对 $f$ 环的反作用力矩，而沿方位环轴对 $a$ 环的外作用力矩是由 环土的力矩电机提供的，所以 $M _ { f a z } ^ { f } = M _ { \Sigma } ^ { a }$

$$
M _ {a f z} ^ {f} = - M _ {f a z} ^ {f} = - M _ {\Sigma z} ^ {a}\tag{5.5.26a}
$$

又由式(a)

$$
M _ {a f x} ^ {f} = - M _ {\Sigma x} ^ {f}\tag{5.5.26b}
$$

由式 Cb) 乘以 $\mathrm { c o s } \theta _ { f }$ 减去式(c)乘以 $\mathrm { s i n } \theta _ { f }$ ，得

$$
M _ {\Sigma y} ^ {r} + M _ {a f y} ^ {f} \cos \theta_ {f} + M _ {\Sigma z} ^ {a} \sin \theta_ {f} = 0
$$

ep

$$
M _ {a f y} ^ {f} = - M _ {\Sigma y} ^ {r} \sec \theta_ {f} - M _ {\Sigma z} ^ {a} \tan \theta_ {f}\tag{5.5.26c}
$$

将式(5.5.26)写成矩阵形式:

$$
\left[ \begin{array}{c} M _ {a f x} ^ {f} \\ M _ {a f y} ^ {f} \\ M _ {a f z} ^ {f} \end{array} \right] = \left[ \begin{array}{c c c} - 1 & 0 & 0 \\ 0 & - \sec \theta_ {f} & - \tan \theta_ {f} \\ 0 & 0 & - 1 \end{array} \right] \left[ \begin{array}{c} M _ {\Sigma x} ^ {f} \\ M _ {\Sigma y} ^ {r} \\ M _ {\Sigma z} ^ {a} \end{array} \right]
$$

上式是方位环作用在俯仰环上的力矩，根据作用和反作用原理，俯仰环对方位环的作用力矩为

$$
\left[ \begin{array}{c} M _ {f a x} ^ {f} \\ M _ {f a y} ^ {f} \\ M _ {f a z} ^ {f} \end{array} \right] = \left[ \begin{array}{c} - M _ {a f x} ^ {f} \\ - M _ {a f y} ^ {f} \\ - M _ {a f z} ^ {f} \end{array} \right] = \left[ \begin{array}{c c c} 1 & 0 & 0 \\ 0 & \sec \theta_ {f} & \tan \theta_ {f} \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c} M _ {\Sigma x} ^ {f} \\ M _ {\Sigma y} ^ {r} \\ M _ {\Sigma z} ^ {a} \end{array} \right]
$$

将上述数学向量变换到方位环a坐标系中，得

$$
\begin{array}{r l} \left[ \begin{array}{c} M _ {f a x} ^ {a} \\ M _ {f a y} ^ {a} \\ M _ {f a z} ^ {a} \end{array} \right] & = \left[ \begin{array}{c c c} \cos \theta_ {a} & \sin \theta_ {a} & 0 \\ - \sin \theta_ {a} & \cos \theta_ {a} & 0 \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c c c} 1 & 0 & 0 \\ 0 & \sec \theta_ {f} & \tan \theta_ {f} \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c} M _ {\Sigma x} ^ {f} \\ M _ {\Sigma y} ^ {r} \\ M _ {\Sigma z} ^ {a} \end{array} \right] \\ & = \left[ \begin{array}{c c c} \cos \theta_ {a} & \sin \theta_ {a} \sec \theta_ {f} & \sin \theta_ {a} \tan \theta_ {f} \\ - \sin \theta_ {a} & \cos \theta_ {a} \sec \theta_ {f} & \cos \theta_ {a} \tan \theta_ {f} \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c} M _ {\Sigma x} ^ {f} \\ M _ {\Sigma y} ^ {r} \\ M _ {\Sigma z} ^ {a} \end{array} \right] \end{array}\tag{5.5.27}
$$

上式是沿环架轴作用的力矩传递到台体上的传递关系式

$$
\boldsymbol {T} _ {\Lambda} ^ {a} = \left[ \begin{array}{c c c} \cos \theta_ {a} & \sin \theta_ {a} \sec \theta_ {f} & \sin \theta_ {a} \tan \theta_ {f} \\ - \sin \theta_ {a} & \cos \theta_ {a} \sec \theta_ {f} & \cos \theta_ {a} \tan \theta_ {f} \\ 0 & 0 & 1 \end{array} \right]
$$

是环架坐标系 至方位环坐标系 的力矩传递矩阵。由于坐标系 在 $\theta _ { f } \neq 0$ 时不是正交坐标系，所以 $\pmb { T } _ { A } ^ { a }$ 不是单位正交矩阵。

显然， $\pmb { T } _ { a } ^ { 4 } = [ \pmb { T } _ { A } ^ { u } ] ^ { - 1 }$ 系到 系的力矩传递矩阵

$$
\begin{array}{l} \boldsymbol {T} _ {a} ^ {\mathrm{A}} = \left(\boldsymbol {C} _ {f} ^ {\alpha} \left[ \begin{array}{c c c} 1 & 0 & 0 \\ 0 & \sec \theta_ {f} & \tan \theta_ {f} \\ 0 & 0 & 1 \end{array} \right]\right) ^ {- 1} = \left[ \begin{array}{c c c} 1 & 0 & 0 \\ 0 & \sec \theta_ {f} & \tan \theta_ {f} \\ 0 & 0 & 1 \end{array} \right] ^ {- 1} [ \boldsymbol {C} _ {f} ^ {\alpha} ] ^ {\mathrm{T}} \\ = \frac {1}{\sec \theta_ {f}} \left[ \begin{array}{c c c} \sec \theta_ {f} & 0 & 0 \\ 0 & 1 & - \tan \theta_ {f} \\ 0 & 0 & \sec \theta_ {f} \end{array} \right] \left[ \begin{array}{c c c} \cos \theta_ {a} & - \sin \theta_ {a} & 0 \\ \sin \theta_ {a} & \cos \theta_ {a} & 0 \\ 0 & 0 & 1 \end{array} \right] = \left[ \begin{array}{c c c} \cos \theta_ {a} & - \sin \theta_ {a} & 0 \\ \sin \theta_ {a} \cos \theta_ {f} & \cos \theta_ {a} \cos \theta_ {f} & - \sin \theta_ {f} \\ 0 & 0 & 1 \end{array} \right] = \boldsymbol {T} _ {6} \end{array}
$$

式中

$$
\boldsymbol {T} _ {6} = \left[ \begin{array}{c c c} \cos \theta_ {a} & - \sin \theta_ {a} & 0 \\ \sin \theta_ {a} \cos \theta_ {f} & \cos \theta_ {a} \cos \theta_ {f} & - \sin \theta_ {f} \\ 0 & 0 & 1 \end{array} \right]\tag{5.5.28}
$$

## 5.5.4 三轴平台的动力学方程及其解

由式 (5. 5. 5) 、式 (5. 5. 7) 、式 (5.5.14) 、式 (5.5.15) 、式 (5.5.17) 、式 (5. 5. 19) 、式 (5. 2. 13)5.5.1 ，可画出三轴平台的闭环方块图，如图 5.5.5 所示。图中 $H ^ { k }$ 阶方阵，是由陀螺配置确定的角动量陀螺力矩形成矩阵。

假设基座角速度 $\pmb { \omega } _ { i b }$ 和环架角速度。 $\mathbf { \dot { \theta } } _ { A }$ 的摩擦约束耦合已由平台稳定回路的卸荷作用消除掉，则上图可简化成图 5.5.6 由图得

$$
\mathbf {Y} _ {5} ^ {- 1} (s) \boldsymbol {\omega} _ {i A} ^ {A} = \mathbf {M} _ {m} ^ {A} + \mathbf {M} _ {\mathrm{d}} ^ {A} + \mathbf {T} _ {6} \mathbf {H} ^ {k} \boldsymbol {\omega} _ {i a} ^ {a}\tag{5.5.29}
$$

$$
\boldsymbol {M} _ {m} ^ {A} (s) = k _ {u} \boldsymbol {Y} _ {4} (s) \boldsymbol {Y} _ {3} (s) \boldsymbol {T} _ {2} \boldsymbol {Y} _ {1} (s) \left\{\boldsymbol {\omega} _ {\mathrm{cmd}} + \boldsymbol {\varepsilon} _ {g} - \left[ \boldsymbol {I} - \boldsymbol {Y} _ {2} (s) \right] \boldsymbol {\omega} _ {i a} ^ {a} \right\}\tag{5.5.30}
$$

$$
\pmb {\omega} _ {i a} ^ {a} = \pmb {T} _ {1} \pmb {\omega} _ {i A} ^ {A} + \pmb {T} _ {3} \pmb {\omega} _ {i b} ^ {b}
$$

根据上式和式 (5 2. 14)

$$
\boldsymbol {\omega} _ {i A} ^ {A} = \boldsymbol {T} _ {1} ^ {- 1} \boldsymbol {\omega} _ {i a} ^ {a} - \boldsymbol {T} _ {1} ^ {- 1} \boldsymbol {T} _ {3} \boldsymbol {\omega} _ {i b} ^ {b} = \boldsymbol {T} _ {2} \boldsymbol {\omega} _ {i a} ^ {a} - \boldsymbol {T} _ {2} \boldsymbol {T} _ {3} \boldsymbol {\omega} _ {i b} ^ {b}\tag{5.5.31}
$$

将式 (5. 5. 30) 和式 (5. 5. 1)代入式 (5.5.29) ，得

$$
\begin{array}{l} \mathbf {Y} _ {5} ^ {- 1} (s) (\mathbf {T} _ {2} \boldsymbol {\omega} _ {i a} ^ {a} - \mathbf {T} _ {2} \mathbf {T} _ {3} \boldsymbol {\omega} _ {i b} ^ {b}) \\ = k _ {u} \mathbf {Y} _ {4} (s) \mathbf {Y} _ {3} (s) \mathbf {T} _ {2} \mathbf {Y} _ {1} (s) \{\boldsymbol {\omega} _ {\mathrm{cmd}} + \boldsymbol {\varepsilon} _ {g} - [ \mathbf {I} - \mathbf {Y} _ {2} (s) ] \boldsymbol {\omega} _ {i a} ^ {a} \} + \mathbf {M} _ {\mathrm{d}} ^ {A} + \mathbf {T} _ {6} \mathbf {H} ^ {k} \boldsymbol {\omega} _ {k i} ^ {a} \\ \{\mathbf {Y} _ {5} ^ {- 1} (s) \mathbf {T} _ {2} - \mathbf {T} _ {6} \mathbf {H} ^ {k} + k _ {u} \mathbf {Y} _ {4} (s) \mathbf {Y} _ {3} (s) \mathbf {T} _ {2} \mathbf {Y} _ {1} (s) [ \mathbf {I} - \mathbf {Y} _ {2} (s) ] \} \boldsymbol {\omega} _ {i a} ^ {a} \\ = k _ {u} \mathbf {Y} _ {4} (s) \mathbf {Y} _ {3} (s) \mathbf {T} _ {2} \mathbf {Y} _ {1} (s) (\boldsymbol {\omega} _ {\mathrm{cmd}} + \boldsymbol {\varepsilon} _ {g}) + \mathbf {M} _ {\mathrm{d}} ^ {A} + \mathbf {Y} _ {5} ^ {- 1} (s) \mathbf {T} _ {2} \mathbf {T} _ {3} \boldsymbol {\omega} _ {i b} ^ {b} \end{array}
$$

上式两边左乘 $\pmb { T } _ { 1 } \pmb { Y } _ { 5 } \left( s \right)$ ,得

$$
\begin{array}{c} \left\{\boldsymbol {I} - \boldsymbol {T} _ {1} \boldsymbol {Y} _ {5} (s) \boldsymbol {T} _ {6} \boldsymbol {H} ^ {k} + k _ {u} \boldsymbol {T} _ {1} \boldsymbol {Y} _ {5} (s) \boldsymbol {Y} _ {4} (s) \boldsymbol {Y} _ {3} (s) \boldsymbol {T} _ {2} \boldsymbol {Y} _ {1} (s) [ \boldsymbol {I} - \boldsymbol {Y} _ {2} (s) ] \right\} \boldsymbol {\omega} _ {i a} ^ {a} \\ = k _ {u} \boldsymbol {T} _ {1} \boldsymbol {Y} _ {5} (s) \boldsymbol {Y} _ {4} (s) \boldsymbol {Y} _ {3} (s) \boldsymbol {T} _ {2} \boldsymbol {Y} _ {1} (s) (\boldsymbol {\omega} _ {\mathrm{cmd}} + \boldsymbol {\varepsilon} _ {g}) + \boldsymbol {T} _ {1} \boldsymbol {Y} _ {5} (s) \boldsymbol {M} _ {\mathrm{d}} ^ {A} + \boldsymbol {T} _ {3} \boldsymbol {\omega} _ {i b} ^ {b} \end{array}
$$

![](images/8246236a0cc76291f159ac0e88cd619119a895824f76aa6a780fc880406fe6ab.jpg)

5.5.5 三轴平台闭环方块图  
![](images/084ec093b507d36b8b1b1c272e84ba0bb1eb06745553394407fcbc19299e1a21.jpg)  
5.5.6 简化后的 轴平台闭环方块图

根据式 (5. 5. 21) ，上式可写成

$$
\begin{array}{l} \left\{\boldsymbol {I} + \boldsymbol {Y} (s) \left[ \boldsymbol {I} - \boldsymbol {Y} _ {2} (s) \right] - \boldsymbol {T} _ {1} \boldsymbol {Y} _ {5} (s) \boldsymbol {T} _ {6} \boldsymbol {H} ^ {k} \right\} \boldsymbol {\omega} _ {i a} ^ {a} \\ = \boldsymbol {Y} (s) (\boldsymbol {\omega} _ {\mathrm{cmd}} + \boldsymbol {\varepsilon} _ {g}) + \boldsymbol {T} _ {1} \boldsymbol {Y} _ {5} (s) \boldsymbol {M} _ {\mathrm{d}} ^ {A} + \boldsymbol {T} _ {3} \boldsymbol {\omega} _ {i b} ^ {b} \end{array}\tag{5.5.32}
$$

工程上常采用小角动量陀螺，上式中由陀螺角动 引起的陀螺力矩 项可忽略，记

$$
\boldsymbol {X} (s) = \boldsymbol {Y} (s) \left[ \boldsymbol {I} - \boldsymbol {Y} _ {2} (s) \right] - \boldsymbol {T} _ {1} \boldsymbol {Y} _ {5} (s) \boldsymbol {T} _ {6} \boldsymbol {H} ^ {k} \approx \boldsymbol {Y} (s) \left[ \boldsymbol {I} - \boldsymbol {Y} _ {2} (s) \right]\tag{5.5.33}
$$

则式 (5. 5. 32)

$$
\begin{array}{c} \left[ \boldsymbol {I} + \boldsymbol {X} (s) \right] \boldsymbol {\omega} _ {i a} ^ {a} = \boldsymbol {Y} (s) (\boldsymbol {\omega} _ {\mathrm{cmd}} + \boldsymbol {\varepsilon} _ {g}) + \boldsymbol {T} _ {1} \boldsymbol {Y} _ {5} (s) \boldsymbol {M} _ {\mathrm{d}} ^ {A} + \boldsymbol {T} _ {3} \boldsymbol {\omega} _ {i b} ^ {b} \\ \boldsymbol {\omega} _ {i a} ^ {a} = [ \boldsymbol {I} + \boldsymbol {X} (s) ] ^ {- 1} [ \boldsymbol {Y} (s) (\boldsymbol {\omega} _ {\mathrm{cmd}} + \boldsymbol {\varepsilon} _ {g}) + \boldsymbol {T} _ {1} \boldsymbol {Y} _ {5} (s) \boldsymbol {M} _ {\mathrm{d}} ^ {A} + \boldsymbol {T} _ {3} \boldsymbol {\omega} _ {i b} ^ {b c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c c} \end{array}\tag{5.5.34}
$$

上式 轴平台动力学方程在复频域内的解。显然平台应该复 ${ \pmb \omega } _ { \mathrm { c m d } }$ 而对干扰 $\pmb { \varepsilon } _ { \mathrm { g } } \mathrm { \nabla } \cdot \pmb { M } _ { \mathrm { d } } ^ { \mathrm { A } }$ 和 $| \pmb { \omega } _ { i b } ^ { b }$

能尽量抑制。

## 5. 6 采用积分陀螺时三轴平台系统的输出轴耦合分析

## 5.6.1 输出轴耦合函数分析

由式(5. 5. 34) ，平台的输出角速度对指令的响应为

$$
\boldsymbol {\omega} _ {i a} ^ {a} = [ \boldsymbol {I} + \boldsymbol {X} (s) ] ^ {- 1} \boldsymbol {Y} (s) \boldsymbol {\omega} _ {\mathrm{cmd}} (s)\tag{5.6.1}
$$

当环架处于中立位置时，即 $\theta _ { f } = 0$ 根据式 (5. 5. 33) 、式 (5.5.25) 和式 (5. 5. 6)

$$
\begin{array}{l} = \boldsymbol {I} + \boldsymbol {Y} (s) [ \boldsymbol {I} - \boldsymbol {Y} _ {2} (s) ] = \boldsymbol {I} + \left[ \begin{array}{c c c} Y _ {h} (s) & 0 & 0 \\ 0 & Y _ {h} (s) & 0 \\ 0 & 0 & Y _ {z} (s) \end{array} \right] \left[ \begin{array}{c c c} 1 & - \frac {\tau_ {g}}{k _ {g}} s & 0 \\ - \frac {\tau_ {g}}{k _ {g}} s & 1 & 0 \\ - \frac {\tau_ {g}}{k _ {g}} s & 0 & 1 \end{array} \right] \\ = \left[ \begin{array}{c c c} 1 + Y _ {h} (s) & - \frac {\tau_ {g}}{k _ {g}} s & 0 \\ - \frac {\tau_ {g}}{k _ {g}} s Y _ {h} (s) & 1 + Y _ {h} (s) & 0 \\ - \frac {\tau_ {g}}{k _ {g}} s Y _ {z} (s) & 0 & 1 + Y _ {z} (s) \end{array} \right] \\ [ \boldsymbol {I} + \boldsymbol {X} (s) ] ^ {- 1} \\ = \frac {1}{\Delta (s)} \left[ \begin{array}{c c c} [ 1 + Y _ {h} (s) ] [ 1 + Y _ {z} (s) ] & \frac {\tau_ {g}}{k _ {g}} s Y _ {h} (s) [ 1 + Y _ {z} (s) ] & 0 \\ \frac {\tau_ {g}}{k _ {g}} s Y _ {h} (s) [ 1 + Y _ {z} (s) ] & [ 1 + Y _ {h} (s) ] [ 1 + Y _ {z} (s) ] & 0 \\ \frac {\tau_ {g}}{k _ {g}} s Y _ {z} (s) [ 1 + Y _ {h} (s) ] & \left(\frac {\tau_ {g}}{k _ {g}}\right) ^ {2} s ^ {2} Y _ {h} (s) Y _ {z} (s) & \left(\frac {\tau_ {g}}{k _ {g}}\right) ^ {2} s ^ {2} Y _ {h} ^ {2} (s) \end{array} \right] \end{array}\tag{5.6.2}
$$

式中

$$
\Delta (s) = \left[ 1 + Y _ {h} (s) \right] ^ {2} \left[ 1 + Y _ {z} (s) \right] - \left(\frac {\tau_ {g}}{k _ {g}}\right) ^ {2} s ^ {2} Y _ {h} ^ {2} (s) \left[ 1 + Y _ {z} (s) \right]\tag{5.6.3}
$$

现以平台的 通道为例说明指令交叉耦合的影响@由式(5. 6. 1)、式(5.6.2)和式(5.6.3)平台沿 $_ { \mathcal { X } }$ 轴的输出为

$$
\begin{array}{l} \omega_ {i a x} ^ {a} (s) = \frac {[ 1 + Y _ {h} (s) ] [ 1 + Y _ {z} (s) ]}{\Delta (s)} Y _ {h} (s) \omega_ {\mathrm{cmd} x} (s) + \frac {\frac {\tau_ {g}}{k _ {g}} s Y _ {h} (s) [ 1 + Y _ {z} (s) ]}{\Delta (s)} Y _ {h} (s) \omega_ {\mathrm{cmd} y} (s) \\ = \frac {[ 1 + Y _ {h} (s) ] Y _ {h} (s)}{\left[ 1 + Y _ {h} (s) \right] ^ {2} - \left(\frac {\tau_ {g}}{k _ {g}}\right) ^ {2} s ^ {2} Y _ {h} ^ {2} (s)} \omega_ {\mathrm{cmd} x} (s) + \frac {\frac {\tau_ {g}}{k _ {g}} s Y _ {h} ^ {2} (s)}{\left[ 1 + Y _ {h} (s) \right] ^ {2} - \left(\frac {\tau_ {g}}{k _ {g}}\right) ^ {2} s ^ {2} Y _ {h} ^ {2} (s)} \omega_ {\mathrm{cmd} y} (s) \end{array}
$$

$$
= F _ {1} (s) \left\{\frac {Y _ {h} (s)}{1 + Y _ {h} (s)} \omega_ {\mathrm{cmd} x} (s) + \frac {\frac {\tau_ {g}}{k _ {g}} s Y _ {h} ^ {2} (s)}{\left[ 1 + Y _ {h} (s) \right] ^ {2}} \omega_ {\mathrm{cmd} y} (s) \right\}\tag{5.6.4}
$$

式中

$$
F _ {1} (s) = \frac {1}{1 - \left\{\frac {\tau_ {g} s Y _ {h} (s)}{k _ {g} [ 1 + Y _ {h} (s) ]} \right\} ^ {2}}\tag{5.6.5}
$$

## 5.6.2 耦合影晌分析

根据式(5.6.4)可画出平台工通道的方块图，如图 5.6.1 所示。

![](images/4fe9ed8355643c4af93d2fb03fa1c1a1a1cb5f16171bad302350ce4be7f367f7.jpg)  
5.6.1 通道方块图

从图 .6.1 看出，要减小指令的交叉耦合影响，应使 $\displaystyle { \dot { \frac { \tau _ { g } } { k _ { g } } } }$ 尺量小，应采用时间常数 $\tau _ { g }$ 很小而陀螺、传递系数 $k _ { g }$ 很大的单自由度陀螺仪，而 $\tau _ { g } = \frac { I _ { 0 } } { D } , k _ { g } = \frac { H } { D }$ ，所以所选陀螺应该是角动量大、浮液阻尼系数大、陀螺组件绕输出轴的转动惯量小

从图 .6.1 还可看出，要使平台正确跟踪指令 $\omega _ { \mathrm { c m d } x }$ '还要求在全频范围内 $\mid F _ { 1 } ( \mathrm { j } \omega ) \mid = 1$ ，下面分析 $F _ { 1 } ( \mathrm { j } \omega )$ 及其影响。将式(5. 6. 5)写成

$$
\begin{array}{r l} F _ {1} (s) & = \frac {1}{1 - P (s)} \\ F _ {1} (\mathrm{j} \omega) & = \frac {1}{1 - P (\mathrm{j} \omega)} \end{array}\tag{5.6.6}
$$

式中

$$
P (s) = \left\{\frac {\tau_ {g} s Y _ {h} (s)}{k _ {g} [ 1 + Y _ {h} (s) ]} \right\} ^ {2}
$$

将式 (5. 5. 22)代入上式，得

$$
\begin{array}{c} P (s) = \frac {\tau_ {g} ^ {2}}{k _ {g} ^ {2}} s ^ {2} \frac {k _ {g} ^ {2} k _ {u} ^ {2} k _ {\mathrm{ACR}} ^ {2} K ^ {2} F ^ {2} (s)}{\left[ s ^ {2} (\tau_ {g} s + 1) + k _ {g} k _ {u} k _ {\mathrm{ACR}} K F (s) \right] ^ {2}} \\ P (\mathrm{j} \omega) = - (\tau_ {g} k _ {u} k _ {\mathrm{ACR}} K) ^ {2} \omega^ {2} \frac {F ^ {2} (\mathrm{j} \omega)}{\left[ - \omega^ {2} (\mathrm{j} \tau_ {g} \omega + 1) + k _ {g} k _ {u} k _ {\mathrm{ACR}} K F (\mathrm{j} \omega) \right] ^ {2}} \end{array}\tag{5.6.7}
$$

(1) $\omega = 0$ 时，由于 $F ( \mathfrak { j } 0 ) = L ($ (校正网络增益)为有限值，所以

$$
P (\mathrm{j} 0) = 0
$$

即在低频段 $P ( \mathrm { j } \omega ) { \approx } 0$ ，根据式(5. 6. 6)知此时有

$$
F _ {1} (\mathrm{j} \omega) \approx 1
$$

(2) $\omega$ 很大时，即在高频段，根据图 4.3.6，此时 $F ( \mathrm { j } \omega ) = L$ 为有限值，由式 (5.6.7)$P ( \mathrm { j } \omega ) { = } 0$ ，所以由式(5. 6. 6)知，此时有

$$
F _ {1} (\mathrm{j} \omega) = 1
$$

(3) 对中频段的分析。

i C;

$$
\frac {1}{V _ {1} (s)} = - \left(\frac {\tau_ {g}}{k _ {g}}\right) ^ {2} \frac {s ^ {2} Y _ {h} ^ {2} (s)}{\left[ 1 + Y _ {h} (s) \right] ^ {2}}\tag{5.6.8}
$$

则式 (5. 6. 5) 可写成

$$
F _ {1} (s) = \frac {V _ {1} (s)}{1 + V _ {1} (s)}, \quad F _ {1} (\mathrm{j} \omega) = \frac {V _ {1} (\mathrm{j} \omega)}{1 + V _ {1} (\mathrm{j} \omega)}
$$

从上式可看出 : $\mathrm { : } V _ { 1 } ( \mathrm { j } \omega )$ 越接近一 ,F, (jω)偏离 就越远 $V _ { 1 } ( \mathrm { j } \omega )$ 偏离-1 越远 $F _ { 1 } ( \mathrm { j } \omega )$ 〉就越接近10 由式 (5.6.8)

$$
V _ {1} (\mathrm{j} \omega) = \left(\frac {k _ {\mathrm{g}}}{\tau_ {\mathrm{g}}}\right) ^ {2} \frac {\left[ 1 + Y _ {h} (\mathrm{j} \omega) \right] ^ {2}}{\omega^ {2} Y _ {h} ^ {2} (\mathrm{j} \omega)}
$$

做出中频段内 $V _ { 1 } ( \mathrm { j } \omega )$ 的幅频特性及相频特性，找出最接近 dB和 $- 1 8 0 ^ { \circ }$ 相位的频率，则在该频率上引起的交叉耦合误差最大，计算出在该频率上 $F _ { 1 } ( \mathrm { j } \omega )$ 1偏离 的最大误差，若不满足要求，应重新设计系统。

三轴平台各通道间的交叉耦合是比较弱的。但为了尽量减小其影响，选用单自由度陀螺作为敏感器件时，应当尽可能选择时间常数小而角动量大的陀螺，同时应适当选择三个陀螺在台体上的方位配置方案。此外，由式(5.5. 的知，当采用双自由度角位置陀螺作为敏感器件时，陀螺仪输出不存在交叉耦合误差，三轴平台各通道间不存在交叉耦合影响，所以从消除交叉糯比去影响的角度来看，兰轴平台采用双自由度角位置陀螺比采用单自由度陀螺有利.

## 5. 7 三轴平台的误差分析

## 5.7.1 三轴平台误差模型

引起三轴平台误差的因素有以下三种

(1)采用单自由度陀螺作为敏感器件时，陀螺力矩引起陀螺输出轴之间的耦合，使平台各通道之间产生耦合误差，引起平台的耦合误差。

(2) 当俯仰环角。 $\theta _ { f } \neq 0$ ，即平台并不处于中立位置时，平台各通道之间产生耦合误差，引起平台的耦合误差。

(3) 陀螺漂移 ${ \pmb { \varepsilon } } _ { \mathrm { g } }$ 、干扰力矩 $M _ { \mathrm { d } }$ 和基座角速度 $\pmb { \omega } _ { i b }$ 引起平台误差。

对于因素(1) 5.6.2 节中已做过分析，所以此处只讨论后两种因素的影响。

为简化分析，此处假设三轴平台采用双自由度角位置陀螺(或时间常数 $\tau _ { g }$ 很大的挠性陀螺〉作为敏感器件。根据式 (5. 5. 8) 和式 (5. 5. 9)

$$
\mathbf {Y} _ {1} (s) = \frac {1}{s} \mathbf {I}\tag{5.5.8}
$$

$$
\mathbf {Y} _ {2} (s) = \mathbf {0}\tag{5.5.9}
$$

将式(5.5.8)代入式(5. 5. 1)，仿照推导式(5.5.2日的分析方法，三轴平台的前向通道传递函数阵为

$$
\mathbf {Y} (s) = \left[ \begin{array}{c c c} Y _ {h} (s) & 0 & C _ {1} Y _ {z} (s) \\ 0 & Y _ {h} (s) & C _ {2} Y _ {z} (s) \\ 0 & 0 & Y _ {z} (s) \end{array} \right]\tag{5.7.1}
$$

式中 $\mathbf { \nabla } _ { C _ { 1 } }$ 和 $C _ { 2 }$ 同式(5.5.24)

$$
Y _ {h} (s) = k _ {h} \frac {F (s)}{s ^ {2}}\tag{5.7.2}
$$

$$
Y _ {z} (s) = k _ {a} \frac {F _ {a} (s)}{s ^ {2}}\tag{5.7.3}
$$

式中 $, k _ { h } = k _ { u } k _ { \mathrm { A C R } } K , k _ { a } = \frac { k _ { m a } k _ { a a } } { I _ { a z } R _ { a } }$ O

将式 (5.5.9) 代入式 (5. 5. 33) ，得

$$
\boldsymbol {X} (s) = \boldsymbol {Y} (s)
$$

根据式(5. 5. 34) ，此时平台的角速度输出为

$$
\boldsymbol {\omega} _ {i a} ^ {a} = [ \boldsymbol {I} + \boldsymbol {Y} (s) ] ^ {- 1} [ \boldsymbol {Y} (s) (\boldsymbol {\omega} _ {\mathrm{cmd}} + \boldsymbol {\varepsilon} _ {g}) + \boldsymbol {T} _ {1} \boldsymbol {Y} _ {5} (s) \boldsymbol {M} _ {\mathrm{d}} + \boldsymbol {T} _ {3} \boldsymbol {\omega} _ {i b} ^ {b} ]\tag{5.7.4}
$$

考虑平台各通道间无耦合的情况，即

$$
\mathbf {Y} ^ {n} (s) = \operatorname{diag} \left[ \begin{array}{c c c} Y _ {h} (s) & Y _ {h} (s) & Y _ {z} (s) \end{array} \right]\tag{5.7.5}
$$

$$
\mathbf {Y} _ {5} ^ {n} (s) = \operatorname{diag} \left[ \frac {1}{I _ {f x} s} \quad \frac {1}{I _ {r y} s} \quad \frac {1}{I _ {a z} s} \right]\tag{5.7.6}
$$

则平台对指令的响应为

$$
\boldsymbol {\omega} _ {i a} ^ {a n} (s) = \left[ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) \right] ^ {- 1} \boldsymbol {Y} ^ {n} (s) \boldsymbol {\omega} _ {\mathrm{cmd}}\tag{5.7.7}
$$

${ \pmb \omega } _ { i a } ^ { a n } \left( s \right)$ 为平台的理想输出。所以平台的输出误差为

$$
\delta \pmb {\omega} (s) = \pmb {\omega} _ {i a} ^ {a} (s) - \pmb {\omega} _ {i a} ^ {a n} (s)
$$

将式 (5. 7. 4) 和式 (5.7.7)代入上式，并记

$$
\Delta \mathbf {Y} (s) = \left[ \begin{array}{c c c} 0 & 0 & C _ {1} Y _ {z} (s) \\ 0 & 0 & C _ {2} Y _ {z} (s) \\ 0 & 0 & 0 \end{array} \right]\tag{5.7.8}
$$

$$
\Delta \mathbf {Y} _ {5} (s) = \left[ \begin{array}{c c c} 0 & 0 & 0 \\ 0 & 0 & - \frac {\sin \theta_ {f}}{I _ {r y} s} \\ 0 & 0 & 0 \end{array} \right]\tag{5.7.9}
$$

$$
\begin{array}{l} \delta \boldsymbol {\omega} (s) = [ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) + \Delta \boldsymbol {Y} (s) ] ^ {- 1} \left\{\left[ \boldsymbol {Y} ^ {n} (s) + \Delta \boldsymbol {Y} (s) \right] (\boldsymbol {\omega} _ {\mathrm{cmd}} + \boldsymbol {\varepsilon} _ {g}) \right. \\ \quad \left. + \boldsymbol {T} _ {1} \left[ \boldsymbol {Y} _ {5} ^ {n} (s) + \Delta \boldsymbol {Y} _ {5} (s) \right] \boldsymbol {M} _ {\mathrm{d}} ^ {A} + \boldsymbol {T} _ {3} \boldsymbol {\omega} _ {i b} ^ {b} \right\} - [ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) ] ^ {- 1} \boldsymbol {Y} ^ {n} (s) \boldsymbol {\omega} _ {\mathrm{cmd}} \end{array}\tag{5.7.10}
$$

对上式简化要用到如下关系式

$$
\left[ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) + \Delta \boldsymbol {Y} (s) \right] ^ {- 1} = \left[ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) \right] ^ {- 1} - \left[ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) \right] ^ {- 1} \Delta \boldsymbol {Y} (s) \left[ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) \right] ^ {- 1}\tag{5.7.11}
$$

现证明式(5.7.11)的正确性。证明思路是 如果式(5.7.11)成立，则对该式作恒等变形一定能得到显式恒等式。

对式 (5.7.11) 的两边左乘 $I + Y ^ { n } \left( s \right) + \Delta { Y } \left( s \right)$ 右乘 $\pmb { I } \mathrm { + } \pmb { Y } ^ { n } \left( s \right)$ ，得

$$
\boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) = \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) + \Delta \boldsymbol {Y} (s) - [ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) + \Delta \boldsymbol {Y} (s) ] \cdot [ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) ] ^ {- 1} \Delta \boldsymbol {Y} (s)\tag{a}
$$

对式 (5.7.11) 两边右乘 $I \mathrm { + } Y ^ { n } \left( s \right) + \Delta { Y } \left( s \right)$ 左乘 $\pmb { I } \mathrm { + } \pmb { Y } ^ { n } \left( s \right)$ ,得

$$
\boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) = \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) + \Delta \boldsymbol {Y} (s) - \Delta \boldsymbol {Y} (s) [ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) ] ^ {- 1} \cdot [ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) + \Delta \boldsymbol {Y} (s) ]\tag{b}
$$

将式 (a) 减去式 (b) ，得

$$
\Delta \mathbf {Y} (s) [ \mathbf {I} + \mathbf {Y} ^ {n} (s) ] ^ {- 1} \cdot [ \mathbf {I} + \mathbf {Y} ^ {n} (s) + \Delta \mathbf {Y} (s) ] = [ \mathbf {I} + \mathbf {Y} ^ {n} (s) + \Delta \mathbf {Y} (s) ] [ \mathbf {I} + \mathbf {Y} ^ {n} (s) ] ^ {- 1} \Delta \mathbf {Y} (s)
$$

记上式左侧项为 $\mathbf { { { T } } } _ { L }$ ，右侧项为 $\pmb { T } _ { R }$ ,则

$$
\begin{array}{r l} \boldsymbol {T} _ {L} & = \Delta \boldsymbol {Y} (s) \left\{\left[ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) \right] ^ {- 1} \left[ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) \right] + \left[ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) \right] ^ {- 1} \Delta \boldsymbol {Y} (s) \right\} \\ & = \Delta \boldsymbol {Y} (s) + \Delta \boldsymbol {Y} (s) \left[ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) \right] ^ {- 1} \Delta \boldsymbol {Y} (s) \end{array}
$$

同理

$$
\boldsymbol {T} _ {R} = \Delta \boldsymbol {Y} (s) + \Delta \boldsymbol {Y} (s) [ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) ] ^ {- 1} \Delta \boldsymbol {Y} (s)
$$

${ \bf { T } } _ { L } = { \bf { T } } _ { R }$ 可见式 (5.7.11)是成立的。

将式 (5.7. 11)代入式 (5.7.10) ，略去关 $\Delta { \bf Y } ( s )$ 及 $\pmb { \varepsilon } _ { \textit { g } }$ 的二阶小量，得三轴平台的输出角速度误差.

$$
\begin{array}{l} \delta \boldsymbol {\omega} (s) = \left\{\left[ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) \right] ^ {- 1} - \left[ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) \right] ^ {- 1} \Delta \boldsymbol {Y} (s) \left[ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) \right] ^ {- 1} \right\} \cdot \left\{\left[ \boldsymbol {Y} ^ {n} (s) + \Delta \boldsymbol {Y} (s) \right] (\boldsymbol {\omega} _ {\mathrm{cmd}} + \boldsymbol {\varepsilon} _ {\mathrm{g}}) \right. \\ \quad + \boldsymbol {T} _ {1} \left[ \boldsymbol {Y} _ {5} ^ {n} (s) + \Delta \boldsymbol {Y} _ {5} (s) \right] \boldsymbol {M} _ {\mathrm{d}} ^ {\mathrm{A}} + \boldsymbol {T} _ {3} \left. \boldsymbol {\omega} _ {i b} ^ {b} \right\} - \left[ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) \right] ^ {- 1} \boldsymbol {Y} ^ {n} (s) \boldsymbol {\omega} _ {\mathrm{cmd}} \\ = [ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) ] ^ {- 1} \Delta \boldsymbol {Y} (s) \left\{\boldsymbol {I} - [ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) ] ^ {- 1} \boldsymbol {Y} ^ {n} (s) \right\} \boldsymbol {\omega} _ {\mathrm{cmd}} + [ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) ] ^ {- 1} \boldsymbol {Y} ^ {n} (s) \boldsymbol {\varepsilon} _ {\mathrm{g}} \\ + [ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) ] ^ {- 1} \left\{\boldsymbol {I} - \Delta \boldsymbol {Y} (s) [ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) ] ^ {- 1} \right\} \boldsymbol {T} _ {1} [ \boldsymbol {Y} _ {5} ^ {n} (s) + \Delta \boldsymbol {Y} _ {5} (s) ] \boldsymbol {M} _ {\mathrm{d}} ^ {\mathrm{A}} \\ + [ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) ] ^ {- 1} \left\{\boldsymbol {I} - \Delta \boldsymbol {Y} (s) [ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) ] ^ {- 1} \boldsymbol {T} _ {3} \boldsymbol {\omega} _ {i b} ^ {b} \right\} \end{array} \tag {5.7.12}
$$

平台的输出误差角为

$$
\delta \boldsymbol {\phi} (s) = \frac {\delta \boldsymbol {\omega} (s)}{s}\tag{5.7.13}
$$

## 5.7.2 各因素引起的平台误差分析

1.交叉揭合 $( \theta _ { f } \ne 0 )$ 引起的对指令角速度的跟踪误差

由式 (5.7.12) ，有

$$
\delta \boldsymbol {\omega} _ {\theta f} (s) = [ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) ] ^ {- 1} \Delta \boldsymbol {Y} (s) \left\{\boldsymbol {I} - [ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) ] ^ {- 1} \boldsymbol {Y} ^ {n} (s) \right\} \boldsymbol {\omega} _ {\text {cmd}}
$$

将式 (5. 7. 5) 、式 (5.7.8) 代入上式，得

$$
\begin{array}{r l} \left[ \begin{array}{c} \delta \omega_ {\theta_ {f} x} (s) \\ \delta \omega_ {\theta_ {f} y} (s) \\ \delta \omega_ {\theta_ {f} z} (s) \end{array} \right] & = \left[ \begin{array}{c c c} \frac {1}{1 + Y _ {h} (s)} & 0 & 0 \\ 0 & \frac {1}{1 + Y _ {h} (s)} & 0 \\ 0 & 0 & \frac {1}{1 + Y _ {z} (s)} \end{array} \right] \left[ \begin{array}{c c c} 0 & 0 & C _ {1} Y _ {z} (s) \\ 0 & 0 & C _ {2} Y _ {z} (s) \\ 0 & 0 & 0 \end{array} \right] \left\{ \begin{array}{c c c} 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{array} \right. \\ & - \left[ \begin{array}{c c c} \frac {Y _ {h} (s)}{1 + Y _ {h} (s)} & 0 & 0 \\ 0 & \frac {Y _ {h} (s)}{1 + Y _ {h} (s)} & 0 \\ 0 & 0 & \frac {Y _ {z} (s)}{1 + Y _ {z} (s)} \end{array} \right] \Bigg \} \boldsymbol {\omega} _ {\text {cmd}} \end{array}
$$

$$
\begin{array}{l} = \left[ \begin{array}{c c c} 0 & 0 & \frac {C _ {1} Y _ {z} (s)}{1 + Y _ {h} (s)} \\ 0 & 0 & \frac {C _ {2} Y _ {z} (s)}{I + Y _ {h} (s)} \\ 0 & 0 & 0 \end{array} \right] \left[ \begin{array}{c c c} \frac {1}{1 + Y _ {h} (s)} & 0 & 0 \\ 0 & \frac {1}{1 + Y _ {h} (s)} & 0 \\ 0 & 0 & \frac {1}{1 + Y _ {z} (s)} \end{array} \right] \left[ \begin{array}{c} \omega_ {\mathrm{cmdx}} (s) \\ \omega_ {\mathrm{cmdy}} (s) \\ \omega_ {\mathrm{cmdz}} (s) \end{array} \right] \\ = \left[ \begin{array}{c} \frac {C _ {1}}{1 + Y _ {h} (s)} \cdot \frac {Y _ {z} (s)}{1 + Y _ {z} (s)} \omega_ {\mathrm{cmdz}} (s) \\ \frac {C _ {2}}{1 + Y _ {h} (s)} \cdot \frac {Y _ {z} (s)}{1 + Y _ {z} (s)} \omega_ {\mathrm{cmdz}} (s) \\ 0 \end{array} \right] \end{array}\tag{5.7.14}
$$

根据式 (5.7.5) 和式 (5.7.7)

$$
\left[ \begin{array}{l} \omega_ {i a x} ^ {a n} (s) \\ \omega_ {i a y} ^ {a n} (s) \\ \omega_ {i a z} ^ {a n} (s) \end{array} \right] = \left[ \begin{array}{l} \frac {Y _ {h} (s)}{1 + Y _ {h} (s)} \omega_ {\mathrm{cmd} x} (s) \\ \frac {Y _ {h} (s)}{1 + Y _ {h} (s)} \omega_ {\mathrm{cmd} y} (s) \\ \frac {Y _ {z} (s)}{1 + Y _ {z} (s)} \omega_ {\mathrm{cmd} z} (s) \end{array} \right]
$$

即有

$$
\frac {Y _ {z} (s)}{1 + Y _ {z} (s)} \omega_ {\mathrm{cmd} z} (s) = \omega_ {i a z} ^ {a n} (s)\tag{5.7.15}
$$

将式 (5.7.15) 代入式 (5.7.14) ，得

$$
\begin{array}{l} \delta \omega_ {\theta_ {f} x} (\mathrm{j} \omega) = \frac {C _ {1}}{Y _ {h} (\mathrm{j} \omega)} \cdot \frac {Y _ {h} (\mathrm{j} \omega)}{1 + Y _ {h} (\mathrm{j} \omega)} \omega_ {i a z} ^ {a n} (\mathrm{j} \omega) \\ \delta \omega_ {\theta_ {f} y} (\mathrm{j} \omega) = \frac {C _ {2}}{Y _ {h} (\mathrm{j} \omega)} \cdot \frac {Y _ {h} (\mathrm{j} \omega)}{1 + Y _ {h} (\mathrm{j} \omega)} \omega_ {i a z} ^ {a n} (\mathrm{j} \omega), \quad \delta \omega_ {\theta_ {f} z} (\mathrm{j} \omega) = 0 \end{array}
$$

下面分析 $\delta \omega _ { \theta _ { f } x } ( \mathrm { j } \omega )$

(1)在低频区，即 $\omega$ 接近 时，根据式(5.7.2)

$$
\frac {Y _ {h} (\mathrm{j} \omega)}{1 + Y _ {h} (\mathrm{j} \omega)} = \frac {k _ {h} F (\mathrm{j} \omega)}{- \omega^ {2} + k _ {h} F (\mathrm{j} \omega)} \approx \frac {k _ {h} F (0)}{k _ {h} F (0)} = 1, \quad \frac {C _ {1}}{Y _ {h} (\mathrm{j} \omega)} = - \frac {C _ {1} \omega^ {2}}{k _ {h} F (\mathrm{j} \omega)} \approx 0
$$

所以，在低频区

$$
\delta \omega_ {\theta_ {f} x} (\mathrm{j} \omega) \approx 0
$$

(2) 在高频区，即 $\omega$ 很大时，由式 (5.7.2)

$$
\mid Y _ {h} (\mathrm{j} \omega) \mid = \frac {k _ {h} F (\mathrm{j} \omega)}{\omega^ {2}} = \frac {k _ {h} L}{\omega^ {2}}
$$

式中， 为校正网络增益，当 $\omega$ 很大时， $| Y _ { h } ( \mathbf { j } \omega )$ 1很小，所以

$$
\frac {Y _ {h} (\mathrm{j} \omega)}{1 + Y _ {h} (\mathrm{j} \omega)} \approx Y _ {h} (\mathrm{j} \omega), \quad \delta \omega_ {\theta_ {f} x} (\mathrm{j} \omega) = C _ {1} \omega_ {i a z} ^ {a n} (\mathrm{j} \omega)
$$

(3) 在中频区，由于在谐振峰值频率 $\omega _ { M }$ 处

$$
\left| \frac {Y _ {h} (\mathrm{j} \omega)}{1 + Y _ {h} (\mathrm{j} \omega)} \right| = M _ {r}
$$

达到最大值，所以平台在 $\omega { = } \omega _ { M }$ 处交叉耦合误差达到最大。又由于谐振峰值频率 $\omega _ { M }$ 与系统的开环交截频率 $\omega _ { C }$ 接近相等，而 $\omega _ { C }$ 处开环增益为 $\mathrm { O d B }$ ，即

所以

$$
\begin{array}{c} \mid Y _ {h} (\mathrm{j} \omega_ {M}) \mid \approx \mid Y _ {h} (\mathrm{j} \omega_ {C}) \mid = 1 \\ \max \left[ \delta \omega_ {\theta_ {f} x} (\mathrm{j} \omega) \right] \approx C _ {1} M _ {r} \omega_ {i a z} ^ {a n} (\mathrm{j} \omega) \end{array}
$$

对 $\delta \omega _ { \theta _ { f } y } ( \mathrm { j } \omega )$ 的分析与上述分析相同，此处不再重复。

## 2. 陀螺漂移引起的平台误差

由式(5. 7. 12) ，陀螺漂移引起的平台漂移为

$$
\delta \boldsymbol {\omega} _ {\varepsilon} (s) = [ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) ] ^ {- 1} \boldsymbol {Y} ^ {n} (s) \boldsymbol {\varepsilon} _ {g} (s)
$$

即

$$
\left[ \begin{array}{l} \delta \omega_ {\varepsilon x} (s) \\ \delta \omega_ {\varepsilon y} (s) \\ \delta \omega_ {\varepsilon z} (s) \end{array} \right] = \left[ \begin{array}{l} \frac {Y _ {h} (s)}{1 + Y _ {h} (s)} \varepsilon_ {g x} (s) \\ \frac {Y _ {h} (s)}{1 + Y _ {h} (s)} \varepsilon_ {g y} (s) \\ \frac {Y _ {z} (s)}{1 + Y _ {z} (s)} \varepsilon_ {g z} (s) \end{array} \right]
$$

将式(5. 7. 2) 、式 (5.7.3) 代入上式，得常值漂移引起的平台漂移

$$
\delta \omega_ {\varepsilon x} (s) = \frac {k _ {h} F (s)}{s ^ {2} + k _ {h} F (s)} \frac {\varepsilon_ {g x}}{s}, \quad \delta \omega_ {\varepsilon y} (s) = \frac {k _ {h} F (s)}{s ^ {2} + k _ {h} F (s)} \frac {\varepsilon_ {g y}}{s}, \quad \delta \omega_ {\varepsilon z} (s) = \frac {k _ {a} F _ {a} (s)}{s ^ {2} + k _ {a} F _ {a} (s)} \frac {\varepsilon_ {g z}}{s}
$$

引起的平台稳态漂移为

$$
\left(\delta \omega_ {\varepsilon x}\right) _ {s s} = \lim _ {s \rightarrow 0} \cdot \frac {k _ {h} F (s)}{s ^ {2} + k _ {h} F (s)} \cdot \frac {\varepsilon_ {g x}}{s} = \varepsilon_ {g x}\tag{5.7.16a}
$$

$$
\left(\delta \omega_ {\varepsilon y}\right) _ {s} = \lim _ {s \rightarrow 0} \cdot \frac {k _ {h} F (s)}{s ^ {2} + k _ {h} F (s)} \cdot \frac {\varepsilon_ {g y}}{s} = \varepsilon_ {g y}\tag{5.7.16b}
$$

$$
\left(\delta \omega_ {\varepsilon z}\right) _ {s s} = \lim _ {s \rightarrow 0} \cdot \frac {k _ {a} F _ {a} (s)}{s ^ {2} + k _ {a} F _ {a} (s)} \cdot \frac {\varepsilon_ {g z}}{s} = \varepsilon_ {g z}\tag{5.7.16c}
$$

## 3. 干扰力矩引起的平台误差

由式(5.7.12) ，干扰力矩引起的平台漂移为

$$
\delta \boldsymbol {\omega} _ {M} (s) = [ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) ] ^ {- 1} \left\{\boldsymbol {I} - \Delta \boldsymbol {Y} (s) [ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) ] ^ {- 1} \right\} \boldsymbol {T} _ {1} \left[ \boldsymbol {Y} _ {5} ^ {n} (s) + \Delta \boldsymbol {Y} _ {5} (s) \right] \boldsymbol {M} _ {\mathrm{d}} ^ {A} (s)
$$

将式 (5.7.5) 、式 (5.7.6) 、式 (5.7.8) 和式 (5.7.9)代入上式，得

$$
\begin{array}{r l} \left[ \begin{array}{c} \delta \omega_ {M x} (s) \\ \delta \omega_ {M y} (s) \\ \delta \omega_ {M z} (s) \end{array} \right] & = \left[ \begin{array}{c c c} \frac {1}{1 + Y _ {h} (s)} & 0 & 0 \\ 0 & \frac {1}{1 + Y _ {h} (s)} & 0 \\ 0 & 0 & \frac {1}{1 + Y _ {z} (s)} \end{array} \right] \left\{\boldsymbol {I} - \left[ \begin{array}{c c c} 0 & 0 & C _ {1} Y _ {z} (s) \\ 0 & 0 & C _ {2} Y _ {z} (s) \\ 0 & 0 & 0 \end{array} \right] \right. \\ & \times \left[ \begin{array}{c c c} \frac {1}{1 + Y _ {h} (s)} & 0 & 0 \\ 0 & \frac {1}{1 + Y _ {h} (s)} & 0 \\ 0 & 0 & \frac {1}{1 + Y _ {z} (s)} \end{array} \right] \left\{\left[ \begin{array}{c c c} \cos \theta_ {a} & \cos \theta_ {f} \sin \theta_ {a} & 0 \\ - \sin \theta_ {a} & \cos \theta_ {f} \cos \theta_ {a} & 0 \\ 0 & 0 & 1 \end{array} \right] \right. \end{array}
$$

$$
\begin{array} { r l } & { } \left[ \begin{array} { c c c } \frac { 1 } { I _ { f x } s } & 0 & 0 \\ 0 & \frac { 1 } { I _ { r y } s } & - \frac { \sin \theta _ { f } } { I _ { r y } s } \\ 0 & 0 & \frac { 1 } { I _ { a z } s } \end{array} \right] \left[ \begin{array} { c } M _ { \mathrm{d} x } ^ { A } ( s ) \\ M _ { \mathrm{d} y } ^ { A } ( s ) \\ M _ { \mathrm{d} z } ^ { A } ( s ) \end{array} \right] \\ & { } = \left[ \begin{array} { c c c } \frac { 1 } { 1 + Y _ { h } ( s ) } & 0 & 0 \\ 0 & \frac { 1 } { 1 + Y _ { h } ( s ) } & 0 \\ 0 & 0 & \frac { 1 } { 1 + Y _ { z } ( s ) } \end{array} \right] \left[ \begin{array} { c c c } 1 & 0 & - \frac { C _ { 1 } Y _ { z } ( s ) } { 1 + Y _ { z } ( s ) } \\ 0 & 1 & - \frac { C _ { 2 } Y _ { z } ( s ) } { 1 + Y _ { z } ( s ) } \\ 0 & 0 & 1 \end{array} \right] \\ & { } \times \left[ \begin{array} { c c c } \frac { \cos \theta _ { a } } { I _ { f x } s } & \frac { \cos \theta _ { f } \sin \theta _ { a } } { I _ { r y } s } & - \frac { \sin 2 \theta _ { f } \sin \theta _ { a } } { 2 I _ { r y } s } \\ - \frac { \sin \theta _ { a } } { I _ { f x } s } & \frac { \cos \theta _ { f } \cos \theta _ { a } } { I _ { r y } s } & - \frac { \sin 2 \theta _ { f } \cos \theta _ { a } } { 2 I _ { r y } s } \\ 0 & 0 & \frac { 1 } { I _ { a z } s } \end{array} \right] \left[ \begin{array} { c } M _ { \mathrm{d} x } ^ { A } ( s ) \\ M _ { \mathrm{d} y } ^ { A } ( s ) \\ M _ { \mathrm{d} z } ^ { A }( s ) \end{array} \right] \\ & { } = \left[ \begin{array} { c c c } \frac { 1 } { 1 + Y _ { h } ( s ) } & 0 & - \frac { C _ { 1 } Y _ { z } ( s ) } { [ 1 + Y _ { h } ( s ) ] [ 1 + Y _ { z } ( s ) ] } \\ 0 & \frac { 1 } { 1 + Y _ { h } ( s ) } & - \frac { C _ { 2 } Y _ { z } ( s ) } { [ 1 + Y _ { h } ( s ) ] [ 1 + Y _ { z } ( s ) ] } \\ 0 & 0 & \frac { 1 } { 1 + Y _ { z } ( s ) } \end{array} \right] \\ & { } \times \left[ \begin{array} { c c c } \frac { \cos \theta _ { a } } { I _ { f x } s } & \frac { \cos \theta _ { f } \sin \theta _ { a } } { I _ { r y } s } & - \frac { \sin 2 \theta _ { f } \sin \theta _ { d a }} { 2 I _ { r y } s } \\ - \frac { \sin \theta _ { a } } { I _ { f x } s } & \frac { \cos \theta _ { f } \cos \theta _ { a } } { I _ { r y } s } & - \frac { \sin 2 \theta _ { f } \cos \theta _ { d a }} { 2 I _ { r y } s } \\ 0 & 0 & \frac { 1 } { I _ { a z } s } \end{array} \right] \left[ \begin{array} { c } M _ { \mathrm{d} x } ^ { A } ( s ) \\ M _ { \mathrm{d} y } ^ { A } ( s ) \\ M _ { \mathrm{d} z } ^ A ( s ) \end{array} \right] \\ &  = [ [ I _  f x s t a n t i o n t a n t i o n t a n t i o n t a n t i o n t a n t i o n t a n t i o n t a n t i o n t a n t i o n t a n t i o n t a n t i o n t a n t i o n t a n t i o n t a n t i o n t a n t i o n t a n t i o n t ] ] \\ & - [ I _  f x s t a n t i o n t a n t i o n t a n t i o n t a n t i o n t a n t i o n t a n t i o n t a n t i o n t a n t i o n t ] ] \\ & - [ I _  f x s t a n t i o n t a n t i o n t a n t i o n t a n t i o n t ] ] \\ & - [ I _  f x s t a n t i o n t a n t i o n t a n t i o n t ] ] \\ & - [ I _  f x s t a n t i o n t a n t i o n t ] ] \\ & - [ I _  f x s t a n t i o n t a n t i o n t ] ] \\ & - [ I _  f x s t a n t i o n t a n t i o n t ] ] \\ & - [ I _  f x s t a n t i o n t a n t i o n t ] ] \\ - [ I _  f x s t a n t i o n t a n t i o n t ] ] \\ & - [ I _  f x s t a n t i o n t a n t i o n t ] ] \\ & - [ I _  f x s t a n t i o n t a n t i o n t ] ] \\ & - [ I _  f x s t a n t i o n t a n t t e r m , j e l l e , k e l l e , l e , m , j e l l e , l e , m , j e l l e , l e , m , j e l l e , l e , m , j e l l e , l e , m , j e l l e , l e , m , j e l l e , l e , m , j e l l e , l e , m , j e l l e , l e , m , j e l l e , l e , m , j . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
$$

\~p

$$
\begin{array}{l} \delta \omega_ {M x} (s) = - \frac {\cos \theta_ {a}}{I _ {f x} s [ 1 + Y _ {h} (s) ]} M _ {\mathrm{d} x} ^ {A} (s) + \frac {\cos \theta_ {f} \sin \theta_ {a}}{I _ {r y} s [ 1 + Y _ {h} (s) ]} M _ {\mathrm{d} y} ^ {A} (s) \\ \quad - \left\{\frac {\sin 2 \theta_ {f} \sin \theta_ {a}}{2 I _ {r y} s [ 1 + Y _ {h} (s) ]} + \frac {C _ {1} Y _ {z} (s)}{I _ {a z} s [ 1 + Y _ {h} (s) ] [ 1 + Y _ {z} (s) ]} \right\} M _ {\mathrm{d} z} ^ {A} (s) \\ \delta \omega_ {M y} (s) = - \frac {\sin \theta_ {a}}{I _ {f x} s [ 1 + Y _ {h} (s) ]} M _ {\mathrm{d} x} ^ {A} (s) + \frac {\cos \theta_ {f} \cos \theta_ {a}}{I _ {r y} s [ 1 + Y _ {h} (s) ]} M _ {\mathrm{d} y} ^ {A} (s) \\ \quad - \left\{\frac {\sin 2 \theta_ {f} \cos \theta_ {a}}{2 I _ {r y} s [ 1 + Y _ {h} (s) ]} + \frac {C _ {2} Y _ {z} (s)}{I _ {a z} s [ 1 + Y _ {h} (s) ] [ 1 + Y _ {z} (s) ]} \right\} M _ {\mathrm{d} z} ^ {A} (s) \\ \delta \omega_ {M z} (s) = \frac {1}{I _ {a z} s [ 1 + Y _ {z} (s) ]} M _ {\mathrm{d} z} ^ {A} (s) \end{array}
$$

将式(5.7.2) 和式(5. 7. 3) 代入上式，并假设干扰力矩为常值，则

$$
\begin{array}{l} \delta \omega_ {M x} (s) = \frac {s \cos \theta_ {a}}{I _ {f x} [ s ^ {2} + k _ {h} F (s) ]} \frac {M _ {\mathrm{d} x} ^ {\mathrm{A}}}{s} + \frac {s \cos \theta_ {f} \sin \theta_ {a}}{I _ {r y} [ s ^ {2} + k _ {h} F (s) ]} \frac {M _ {\mathrm{d} y} ^ {\mathrm{A}}}{s} \\ - \left\{\frac {\sin 2 \theta_ {f} \sin \theta_ {a}}{2 I _ {r y} [ s ^ {2} + k _ {h} F (s) ]} + \frac {s C _ {1} k _ {a} F _ {a} (s)}{I _ {a z} [ s ^ {2} + k _ {h} F (s) ] [ s ^ {2} + k _ {a} F _ {a} (s) ]} \right\} \frac {M _ {\mathrm{d} z} ^ {\mathrm{A}}}{s} \\ \delta \omega_ {M y} (s) = - \frac {s \sin \theta_ {a}}{I _ {f x} [ s ^ {2} + k _ {h} F (s) ]} \cdot \frac {M _ {\mathrm{d} x} ^ {\mathrm{A}}}{s} + \frac {s \cos \theta_ {f} \cos \theta_ {a}}{I _ {r y} [ s ^ {2} + k _ {h} F (s) ]} \cdot \frac {M _ {\mathrm{d} y} ^ {\mathrm{A}}}{s} \\ - \left\{\frac {s \sin 2 \theta_ {f} \cos \theta_ {a}}{2 I _ {r y} [ s ^ {2} + k _ {h} F (s) ]} + \frac {s C _ {2} k _ {a} F _ {a} (s)}{I _ {a z} [ s ^ {2} + k _ {h} F (s) ] [ s ^ {2} + k _ {a} F _ {a} (s) ]} \right\} \frac {M _ {\mathrm{d} z} ^ {\mathrm{A}}}{s} \end{array}
$$

$$
\delta \omega_ {M z} (s) = \frac {s}{I _ {a z} [ s ^ {2} + k _ {a} F _ {a} (s) ]} \cdot \frac {M _ {\mathrm{d} z} ^ {A}}{s}
$$

平台偏差角为

$$
\begin{array}{r l} \delta \phi_ {M x} (s) & = \frac {1}{s} \delta \omega_ {M x} (s) \\ & = \frac {\cos \theta_ {a}}{I _ {f x} [ s ^ {2} + k _ {h} F (s) ]} \cdot \frac {M _ {\mathrm{d} x} ^ {A}}{s} + \frac {\cos \theta_ {f} \sin \theta_ {a}}{I _ {r y} [ s ^ {2} + k _ {h} F (s) ]} \cdot \frac {M _ {\mathrm{d} y} ^ {A}}{s} \\ & - \left\{\frac {\sin 2 \theta_ {f} \sin \theta_ {a}}{2 I _ {r y} [ s ^ {2} + k _ {h} F (s) ]} + \frac {C _ {1} k _ {a} F _ {a} (s)}{I _ {a z} [ s ^ {2} + k _ {h} F (s) ] [ s ^ {2} + k _ {a} F _ {a} (s) ]} \right\} \cdot \frac {M _ {\mathrm{d} z} ^ {A}}{s} \end{array}\tag{5.7.17a}
$$

$$
\begin{array}{r l} \delta \phi_ {M _ {\mathrm{y}}} (s) & = \frac {1}{s} \delta \omega_ {M _ {\mathrm{y}}} (s) \\ & = - \frac {\sin \theta_ {a}}{I _ {f x} [ s ^ {2} + k _ {h} F (s) ]} \cdot \frac {M _ {\mathrm{dx}} ^ {A}}{s} + \frac {\cos \theta_ {f} \cos \theta_ {a}}{I _ {r y} [ s ^ {2} + k _ {h} F (s) ]} \cdot \frac {M _ {\mathrm{dy}} ^ {A}}{s} \\ & - \left\{\frac {\sin 2 \theta_ {f} \cos \theta_ {a}}{2 I _ {r y} [ s ^ {2} + k _ {h} F (s) ]} + \frac {C _ {2} k _ {a} F _ {a} (s)}{I _ {a z} [ s ^ {2} + k _ {h} F (s) ] [ s ^ {2} + k _ {a} F _ {a} (s) ]} \right\} \cdot \frac {M _ {\mathrm{dz}} ^ {A}}{s} \end{array}\tag{5.7.17b}
$$

$$
\delta \phi_ {M z} (s) = \frac {1}{s} \delta \omega_ {M z} (s) = \frac {1}{I _ {a z} [ s ^ {2} + k _ {a} F _ {a} (s) ]} \cdot \frac {M _ {\mathrm{d} z} ^ {A}}{s}\tag{5.7.17c}
$$

应用终值定理，得稳态值

$$
\begin{array}{l} \left(\delta \omega_ {M x}\right) _ {s s} = 0, \quad \left(\delta \omega_ {M y}\right) _ {s s} = 0, \quad \left(\delta \omega_ {M z}\right) _ {s s} = 0 \\ \left(\delta \phi_ {M x}\right) _ {s s} = \frac {\cos \theta_ {a}}{I _ {f x} k _ {h} F (0)} M _ {\mathrm{d} x} ^ {A} + \frac {\cos \theta_ {f} \sin \theta_ {a}}{I _ {r y} k _ {h} F (0)} M _ {\mathrm{d} y} ^ {A} - \left[ \frac {\sin 2 \theta_ {f} \sin \theta_ {a}}{2 I _ {r y} k _ {h} F (0)} + \frac {C _ {1} k _ {a} F _ {a} (0)}{I _ {a z} k _ {h} k _ {a} F (0) F _ {a} (0)} \right] M _ {\mathrm{d} z} ^ {A} \\ = \frac {\cos \theta_ {a}}{I _ {f x} S _ {\Phi_ {h}} (0)} M _ {\mathrm{d} x} ^ {A} + \frac {\cos \theta_ {f} \sin \theta_ {a}}{I _ {r y} S _ {\Phi_ {h}} (0)} M _ {\mathrm{d} y} ^ {A} - \left[ \frac {\sin 2 \theta_ {f} \sin \theta_ {a}}{2 I _ {r y} S _ {\Phi_ {h}} (0)} + \frac {C _ {1}}{I _ {a z} S _ {\Phi_ {h}} (0)} \right] M _ {\mathrm{d} z} ^ {A} \\ \left(\delta \phi_ {M y}\right) _ {s s} = - \frac {\sin \theta_ {a}}{I _ {f x} S _ {\Phi_ {h}} (0)} M _ {\mathrm{d} x} ^ {A} + \frac {\cos \theta_ {f} \cos \theta_ {a}}{I _ {r y} S _ {\Phi_ {h}} (0)} M _ {\mathrm{d} y} ^ {A} - \left[ \frac {\sin 2 \theta_ {f} \cos \theta_ {a}}{2 I _ {r y} S _ {\Phi_ {h}} (0)} + \frac {C _ {2}}{I _ {a z} S _ {\Phi_ {h}} (0)} \right] M _ {\mathrm{d} z} ^ {A} \\ (\delta \phi_ {M z}) _ {s s} = \frac {M _ {\mathrm{d} z} ^ {A}}{I _ {a z} S _ {\Phi_ {a}} (0)} \end{array}
$$

4. 由基座角运动耦合引起的平台误差

由式(5.7.12) ，由基座角运动引起的平台漂移为

$$
\begin{array}{l} \delta \boldsymbol {\omega} _ {B} (s) = [ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) ] ^ {- 1} \{\boldsymbol {I} - \Delta \boldsymbol {Y} (s) [ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) ] ^ {- 1} \} \boldsymbol {T} _ {3} \boldsymbol {\omega} _ {i b} ^ {b} \\ = \left[ \begin{array}{c c c} \frac {1}{1 + Y _ {h} (s)} & 0 & - \frac {C _ {1} Y _ {z} (s)}{\left[ 1 + Y _ {h} (s) \right] \left[ 1 + Y _ {z} (s) \right]} \\ 0 & \frac {1}{1 + Y _ {h} (s)} & - \frac {C _ {2} Y _ {z} (s)}{\left[ 1 + Y _ {h} (s) \right] \left[ 1 + Y _ {z} (s) \right]} \\ 0 & 0 & \frac {1}{1 + Y _ {z} (s)} \end{array} \right] \\ \times \left[ \begin{array}{c c c} \sin \theta_ {r} \sin \theta_ {f} \sin \theta_ {a} & 0 & \cos \theta_ {r} \sin \theta_ {f} \sin \theta_ {a} \\ \sin \theta_ {r} \sin \theta_ {f} \cos \theta_ {a} & 0 & \cos \theta_ {r} \sin \theta_ {f} \cos \theta_ {a} \\ 0 & 0 & 0 \end{array} \right] \left[ \begin{array}{c} \boldsymbol {\omega} _ {i b x} ^ {b} (s) \\ \boldsymbol {\omega} _ {i b y} ^ {b} (s) \\ \boldsymbol {\omega} _ {i b z} ^ {b} (s) \end{array} \right] \\ = \left[ \begin{array}{c c c} \frac {\sin \theta_ {r} \sin \theta_ {f} \sin \theta_ {a}}{1 + Y _ {h} (s)} & 0 & \frac {\cos \theta_ {r} \sin \theta_ {f} \sin \theta_ {a}}{1 + Y _ {h} (s)} \\ \frac {\sin \theta_ {r} \sin \theta_ {f} \cos \theta_ {a}}{1 + Y _ {h} (s)} & 0 & \frac {\cos \theta_ {r} \sin \theta_ {f} \cos \theta_ {a}}{1 + Y _ {h} (s)} \\ 0 & 0 & 0 \end{array} \right] \left[ \begin{array}{c} \boldsymbol {\omega} _ {i b x} ^ {b} (s) \\ \boldsymbol {\omega} _ {i b y} ^ {b} (s) \\ \boldsymbol {\omega} _ {i b z} ^ {b} (s) \end{array} \right] \end{array}
$$

\~p

$$
\begin{array}{l} \delta \omega_ {B x} (s) = \frac {1}{1 + Y _ {h} (s)} \left[ \sin \theta_ {r} \sin \theta_ {f} \sin \theta_ {a} \omega_ {i b x} ^ {b} (s) + \cos \theta_ {r} \sin \theta_ {f} \sin \theta_ {a} \omega_ {i b z} ^ {b} (s) \right] \\ = \frac {s ^ {2} \sin \theta_ {f} \sin \theta_ {a}}{s ^ {2} + k _ {h} F (s)} \left[ \sin \theta_ {r} \omega_ {i b x} ^ {b} (s) + \cos \theta_ {r} \omega_ {i b z} ^ {b} (s) \right] \\ \delta \omega_ {B y} (s) = \frac {1}{1 + Y _ {h} (s)} \left[ \sin \theta_ {r} \sin \theta_ {f} \cos \theta_ {a} \omega_ {i b x} ^ {b} (s) + \cos \theta_ {r} \sin \theta_ {f} \cos \theta_ {a} \omega_ {i b z} ^ {b} (s) \right] \\ = \frac {s ^ {2} \sin \theta_ {f} \cos \theta_ {a}}{s ^ {2} + k _ {h} F (s)} \left[ \sin \theta_ {r} \omega_ {i b x} ^ {b} (s) + \cos \theta_ {r} \omega_ {i b z} ^ {b} (s) \right] \\ \delta \omega_ {B z} (s) = 0 \end{array}
$$

$$
\delta \phi_ {B x} (s) = \frac {\delta \omega_ {B x} (s)}{s} = \frac {s \sin \theta_ {f} \sin \theta_ {a}}{s ^ {2} + k _ {h} F (s)} \left[ \sin \theta_ {r} \omega_ {i b x} ^ {b} (s) + \cos \theta_ {r} \omega_ {i b z} ^ {b} (s) \right]
$$

$$
\delta \phi_ {B y} (s) = \frac {\delta \omega_ {B y} (s)}{s} = \frac {s \sin \theta_ {f} \cos \theta_ {a}}{s ^ {2} + k _ {h} F (s)} \left[ \sin \theta_ {r} \omega_ {i b x} ^ {b} (s) + \cos \theta_ {r} \omega_ {i b z} ^ {b} (s) \right]
$$

$$
\delta \phi_ {B z} (s) = \frac {\delta \omega_ {B z} (s)}{s} = 0
$$

(1) 基座角速度为常值时，即

$$
\omega_ {i b x} ^ {b} (s) = \frac {\omega_ {i b x} ^ {b}}{s}, \quad \omega_ {i b z} ^ {b} (s) = \frac {\omega_ {i b z} ^ {b}}{s}
$$

则

$$
\left(\delta \omega_ {B x}\right) _ {s s} = 0, \quad \left(\delta \omega_ {B y}\right) _ {s s} = 0, \quad \left(\delta \omega_ {B z}\right) _ {s s} = 0, \left(\delta \phi_ {B x}\right) _ {s s} = 0, \quad \left(\delta \phi_ {B y}\right) _ {s s} = 0, \quad \left(\delta \phi_ {B z}\right) _ {s s} = 0
$$

(2) 基座角加速度为常值时，即

$$
\dot {\omega} _ {i b x} ^ {b} (s) = \frac {\dot {\omega} _ {i b x} ^ {b}}{s}, \quad \dot {\omega} _ {i b z} ^ {b} (s) = \frac {\dot {\omega} _ {i b z} ^ {b}}{s}
$$

RP

$$
\omega_ {i b x} ^ {b} (s) = \frac {\dot {\omega} _ {i b x} ^ {b}}{s ^ {2}}, \quad \omega_ {i b z} ^ {b} (s) = \frac {\dot {\omega} _ {i b z} ^ {b}}{s ^ {2}}
$$

则

$$
\begin{array}{l} \left(\delta \omega_ {B x}\right) _ {s s} = 0, \quad \left(\delta \omega_ {B y}\right) _ {s s} = 0, \quad \left(\delta \omega_ {B z}\right) _ {s s} = 0 \\ \left(\delta \phi_ {B x}\right) _ {s s} = \frac {\sin \theta_ {f} \sin \theta_ {a}}{k _ {h} F (0)} \left(\dot {\omega} _ {i b x} ^ {b} \sin \theta_ {r} + \dot {\omega} _ {i b z} ^ {b} \cos \theta_ {r}\right) = \frac {\sin \theta_ {f} \sin \theta_ {a}}{S _ {\Phi_ {h}} (0)} \left(\dot {\omega} _ {i b x} ^ {b} \sin \theta_ {r} + \dot {\omega} _ {i b z} ^ {b} \cos \theta_ {r}\right) \\ \left(\delta \phi_ {B y}\right) _ {s s} = \frac {\sin \theta_ {f} \cos \theta_ {a}}{S _ {\Phi_ {h}} (0)} \left(\dot {\omega} _ {i b x} ^ {b} \sin \theta_ {r} + \dot {\omega} _ {i b z} ^ {b} \cos \theta_ {r}\right) \\ \left(\delta \phi_ {B z}\right) _ {s s} = 0 \end{array}
$$

## 5. 振动诱导漂移

在工程实际中，陀螺单独测试时可达到某一精度，但用在平台系统中后，平台却远远达不到这一精度。造成这一现象的原因之一是陀螺的偏心振动，即陀螺自转引起偏心振动，产生对平台的干扰力矩，此干扰力矩引起平台绕稳定轴的角振动，陀螺偏心振动与平台角振动间的耦合引起陀螺的附加常值漂移，从而引起平台漂移。下面详细分析这一问题的定量关系。

## 1) 平台的偏心振动

XYZ 为平台坐标系，记作 $a _ { \circ }$ 平台由两个双自由度陀螺控制，上陀螺记为 ，下陀螺记为$B _ { \circ }$ 上、下陀螺的自转角速度均为 $\omega _ { s }$ 。 $C _ { T }$ 和 $C _ { B }$ 分别为上、下陀螺的质心， $\mathbf { \Omega } , O _ { T }$ .和 $O _ { B }$ 分别为上、下陀螺的支点。记上陀螺的偏心向量为 $\pmb { \mu }$ ，质量为 $m _ { T }$ '下陀螺的偏心向量为1. $\mathbf { \mu } _ { \mathbf { \nu } } \mathbf { \pmb { \nu } } _ { \mathbf { \nu } } \mathbf { \pmb { \nu } } _ { \mathbf { \nu } } \mathbf { \pmb { \nu } } _ { \mathbf { \nu } } \mathbf { \pmb { \nu } } _ { \mathbf { \nu } } \mathbf { \pmb { \nu } } _ { \mathbf { \nu } } \mathbf { \pmb { \nu } } _ { \mathbf { \nu } } \mathbf { \pmb { \nu } } _ { \mathbf { \nu } } \mathbf { \pmb { \nu } } _ { \mathbf { \nu } } \mathbf { \pmb { \nu } } _ { \mathbf { \nu } } \mathbf { \pmb { \nu } } _ { \mathbf { \nu } } \mathbf { \pmb { \nu } } _ { \mathbf { \nu } } \mathbf { \pmb { \nu } } _ { \mathbf { \nu } } \mathbf { \pmb { \nu } } _ { \mathbf { \nu } } \mathbf { \pmb { \nu } } _ { \mathbf { \nu } } \mathbf { \pmb { \nu } } _ { \mathbf { \nu } } \mathbf { \pmb { \nu } } _ { \mathbf { \nu } } \mathbf { \pmb { \nu } } _ { \mathbf { \nu } } \mathbf { \pmb { \nu } } _ \mathbf { \nu } \mathbf { \pmb { \nu } } \mathbf { \pmb \nu } _ { \mathbf { \nu } } \mathbf _ { \mathbf { \nu } } \mathbf { \pmb \nu } _ { \mathbf \nu } \mathbf { \pmb { \nu } } \mathbf _ { \nu \pmb } \mathbf { \nu } _ \mathbf { \nu \pmb } \mathbf { \nu } _ \mathbf { \nu \pmb } \mathbf { \nu } _ \mathbf { \nu \nu } \mathbf _ { \nu \nu } \mathbf _ { \nu \nu \pmb } \mathbf _ { \nu \nu \nu } \mathbf _ { \nu \nu \nu } \mathbf _ \mathbf { \nu \nu \nu } \mathbf _ { \nu \nu \nu } \mathbf _ \mathbf { \nu \nu \nu } \mathbf _ \nu \mathbf { \nu \nu \nu } \mathbf _ \nu \mathbf { \nu \nu \nu } \mathbf _ \nu \mathbf \nu \nu \mathbf { \nu \nu } \mathbf \mathbf \pmb _ \nu \nu \nu \mathbf \nu \nu \mathbf \nu \nu \mathbf \nu \nu _ \nu \nu \nu \mathbf \nu \nu \nu \mathbf \nu \mathbf \nu \nu \nu \mathbf \nu \ m \ m \ m $ ，质量为 $m _ { B \circ L _ { 1 } }$ 和$L _ { \mathrm { 2 } }$ 为陀螺支点距平台支点。的距离。如图 5.7.1 所示。图中， $, \mu _ { 0 }$ 。为上陀螺偏心向量 $\mathbf { \nabla } _ { \mathbf { \mu } } \mathbf { \mu } _ { \mathbf { \mu } } \mathbf { \mu } _ { \mathbf { \mu } } \mathbf { \mu } _ { \mathbf { \mu } } \mathbf { \mu } _ { \mathbf { \mu } } \mathbf { \mu } _ { \mathbf { \mu } } \mathbf { \mu } _ { \mathbf { \mu } } \mathbf { \mu } _ { \mu } \mathbf { \mu } _ { \mathbf { \nu } } \mathbf { \mu } _ { \mathbf { \mu } }$ 在陀螺赤道平面内的投影，片为 $, \mu _ { a }$ $J \pmb { \mu }$ 在上陀螺极轴上的投影， $\upsilon _ { 0 }$ 为 $\mathbf { \sigma } _ { \mathbf { \nu } } \mathbf { \cdot } \mathbf { \sigma } _ { \mathbf { \nu } } \mathrm { ~ }$ 在下陀螺赤道平面内的投影， $\upsilon _ { a }$ 为 ${ \bf \omega } _ { \bf \upsilon }$ 在下陀螺极轴上的投影。

设转子在自转的同时还引起转子沿转子轴的轴向窜动，上陀螺的窜动量为 $\mu _ { 1 }$ ，下陀螺的窜动量为 $\upsilon _ { 1 }$ ，则上、下陀螺质心在平台坐标系 内的数学向量表达式分别为

$$
\boldsymbol {\mu} ^ {a} = \left[ \begin{array}{c} \mu_ {0} \cos \omega_ {s} t \\ \mu_ {0} \sin \omega_ {s} t \\ \mu_ {a} + \mu_ {1} \cos (\omega_ {s} t + \varphi_ {1}) \end{array} \right], \quad \boldsymbol {v} ^ {a} = \left[ \begin{array}{c} v _ {0} \sin (\omega_ {s} t + \varphi_ {0}) \\ v _ {0} \cos (\omega_ {s} t + \varphi_ {0}) \\ v _ {a} + v _ {1} \cos (\omega_ {s} t + \varphi_ {0} + \varphi_ {2}) \end{array} \right]
$$

![](images/8cd267c72a637dd1bf66a7d93630195182e557c633002009a46f21e1738c0e65.jpg)  
5. 7. 1 上下陀螺振动对 轴平台的影响

略去平台台体相对惯性空间的角速度影响，则有

$$
\left. \frac {\mathrm{d} ^ {2} \boldsymbol {\mu}}{\mathrm{d} t ^ {2}} \right| _ {i} ^ {a} \approx \ddot {\boldsymbol {\mu}} ^ {a} = \left[ \begin{array}{c} - \mu_ {0} \omega_ {s} ^ {2} \cos \omega_ {s} t \\ - \mu_ {0} \omega_ {s} ^ {2} \sin \omega_ {s} t \\ - \mu_ {1} \omega_ {s} ^ {2} \cos (\omega_ {s} t + \varphi_ {1}) \end{array} \right], \quad \left. \frac {\mathrm{d} ^ {2} \boldsymbol {v}}{\mathrm{d} t ^ {2}} \right| _ {i} ^ {a} \approx \ddot {\boldsymbol {v}} ^ {a} = \left[ \begin{array}{c} - v _ {0} \omega_ {s} ^ {2} \sin (\omega_ {s} t + \varphi_ {0}) \\ - v _ {0} \omega_ {s} ^ {2} \cos (\omega_ {s} t + \varphi_ {0}) \\ - v _ {1} \omega_ {s} ^ {2} \cos (\omega_ {s} t + \varphi_ {0} + \varphi_ {2}) \end{array} \right]
$$

由上述加速度引起的上下陀螺转子惯性力分别为

$$
\begin{array}{r l} \boldsymbol {P} ^ {a} = & - m _ {T} \ddot {\boldsymbol {\mu}} ^ {a} = \left[ \begin{array}{c} m _ {T} \mu_ {0} \omega_ {s} ^ {2} \cos \omega_ {s} t \\ m _ {T} \mu_ {0} \omega_ {s} ^ {2} \sin \omega_ {s} t \\ m _ {T} \mu_ {1} \omega_ {s} ^ {2} \cos (\omega_ {s} t + \varphi_ {1}) \end{array} \right] \\ \boldsymbol {Q} ^ {a} = & - m _ {B} \ddot {\boldsymbol {v}} ^ {a} = \left[ \begin{array}{c} m _ {B} \nu_ {0} \omega_ {s} ^ {2} \sin (\omega_ {s} t + \varphi_ {0}) \\ m _ {B} \nu_ {0} \omega_ {s} ^ {2} \cos (\omega_ {s} t + \varphi_ {0}) \\ m _ {B} \nu_ {1} \omega_ {s} ^ {2} \cos (\omega_ {s} t + \varphi_ {0} + \varphi_ {2}) \end{array} \right] \end{array}\tag{5.7.18}
$$

(5. 7. 19)

上述惯性力由转子产生，通过框架作用在平台台体 由于偏心 十分微小，所以 $O _ { T }$ 和 $O _ { B }$ 可看做力的作用点。由图 5.7.1 可求得 对平台 生的力矩

$$
\boldsymbol {M} _ {\mathrm{d}} ^ {a} = \boldsymbol {L} _ {1} ^ {a} \times \boldsymbol {P} ^ {a} + \boldsymbol {L} _ {2} ^ {a} \times \boldsymbol {Q} ^ {a}\tag{5.7.20}
$$

式中， $\mathbf { { \cal { L } } } _ { 1 } ^ { a } = \binom { 0 } { 0 } , \mathbf { { \cal { L } } } _ { 2 } ^ { a } = \boxed { - \cal { L } _ { 2 } }$ 。将上述两式和式 (5. 7. 18) 、式 (5. 7. 19) 代入式 (5.7.20) ，得

$$
\begin{array}{r l} \left[ \begin{array}{l} M _ {\mathrm{d} x} ^ {a} \\ M _ {\mathrm{d} y} ^ {a} \\ M _ {\mathrm{d} z} ^ {a} \end{array} \right] & = \left[ \begin{array}{c c c} 0 & - L _ {1} & 0 \\ L _ {1} & 0 & 0 \\ 0 & 0 & 0 \end{array} \right] \left[ \begin{array}{c} m _ {T} \mu_ {0} \omega_ {s} ^ {2} \cos \omega_ {s} t \\ m _ {T} \mu_ {0} \omega_ {s} ^ {2} \sin \omega_ {s} t \\ m _ {T} \mu_ {1} \omega_ {s} ^ {2} \cos (\omega_ {s} t + \varphi_ {1}) \end{array} \right] + \left[ \begin{array}{c c c} 0 & 0 & - L _ {2} \\ 0 & 0 & 0 \\ L _ {2} & 0 & 0 \end{array} \right] \left[ \begin{array}{c} m _ {B} \nu_ {0} \omega_ {s} ^ {2} \sin (\omega_ {s} t + \varphi_ {0}) \\ m _ {B} \nu_ {0} \omega_ {s} ^ {2} \cos (\omega_ {s} t + \varphi_ {0}) \\ m _ {B} \nu_ {1} \omega_ {s} ^ {2} \cos (\omega_ {s} t + \varphi_ {0} + \varphi_ {2}) \end{array} \right] \\ & = \left[ \begin{array}{c} - m _ {T} \mu_ {0} L _ {1} \omega_ {s} ^ {2} \sin \omega_ {s} t - m _ {B} \nu_ {1} L _ {2} \omega_ {s} ^ {2} \cos (\omega_ {s} t + \varphi_ {0} + \varphi_ {2}) \\ m _ {T} \mu_ {0} L _ {1} \omega_ {s} ^ {2} \cos \omega_ {s} t \\ m _ {B} \nu_ {0} L _ {2} \omega_ {s} ^ {2} \sin (\omega_ {s} t + \varphi_ {0}) \end{array} \right] \end{array} \tag {5.7.21}
$$

由于此干扰力矩是由陀螺转子振动引起的，所以称之为振动诱导干扰力矩。

2) 平台系统对振动诱导干扰力矩的响应

由式(5.7.17)及式(5.5.24) ，当平台处于中立位置，即 $\theta _ { f } = 0$ 时，平台偏差角为

$$
\begin{array}{l} \delta \phi_ {x} (s) = \frac {\cos \theta_ {a}}{I _ {f x} [ s ^ {2} + k _ {h} F (s) ]} M _ {\mathrm{d} x} ^ {A} (s) + \frac {\sin \theta_ {a}}{I _ {r y} [ s ^ {2} + k _ {h} F (s) ]} M _ {\mathrm{d} y} ^ {A} (s) \\ \delta \phi_ {y} (s) = - \frac {\sin \theta_ {a}}{I _ {f x} [ s ^ {2} + k _ {h} F (s) ]} M _ {\mathrm{d} x} ^ {A} (s) + \frac {\cos \theta_ {a}}{I _ {r y} [ s ^ {2} + k _ {h} F (s) ]} M _ {\mathrm{d} y} ^ {A} (s) \\ \delta \phi_ {z} (s) = \frac {1}{I _ {a z} [ s ^ {2} + k _ {a} F _ {a} (s) ]} M _ {\mathrm{d} z} ^ {A} (s) \end{array}
$$

设'P为运载体的平台航向角，规定顺时针为正， $\theta _ { a }$ 此为平台的方位环角，规定逆时针为正，如5.7.2 所示。由图知当 $\theta _ { a }$ 为正， $\Psi$ P也为正，即有 χ.(ν4 yb)

$$
\begin{array}{r l} & {\Psi = \theta_ {a}} \\ & {\mathbf {C} _ {A} ^ {a} = \left[ \begin{array}{c c c} {\cos \theta_ {a}} & {\sin \theta_ {a}} & {0} \\ {- \sin \theta_ {a}} & {\cos \theta_ {a}} & {0} \\ {0} & {0} & {1} \end{array} \right] = \left[ \begin{array}{c c c} {\cos \Psi} & {\sin \Psi} & {0} \\ {- \sin \Psi} & {\cos \Psi} & {0} \\ {0} & {0} & {1} \end{array} \right]} \\ & {\left[ M _ {\mathrm{d} x} ^ {A} (s) \right] = \left[ \begin{array}{c c c} {\cos \Psi} & {- \sin \Psi} & {0} \\ {\sin \Psi} & {\cos \Psi} & {0} \\ {0} & {0} & {1} \end{array} \right] \left[ \begin{array}{c} {M _ {\mathrm{d} x} ^ {a} (s)} \\ {M _ {\mathrm{d} y} ^ {a} (s)} \\ {M _ {\mathrm{d} z} ^ {a} (s)} \end{array} \right]} \\ & {\quad = \left[ \begin{array}{c} {M _ {\mathrm{d} x} ^ {a} (s) \cos \Psi - M _ {\mathrm{d} y} ^ {a} (s) \sin \Psi} \\ {M _ {\mathrm{d} x} ^ {a} (s) \sin \Psi + M _ {\mathrm{d} y} ^ {a} (s) \cos \Psi} \\ {M _ {\mathrm{d} z} ^ {a} (s)} \end{array} \right]} \end{array}
$$

![](images/32f9c2d4d80f2ba80ee91881115d67dc197a7daec87419a467cde39b3136fc44.jpg)

角íJl'与方位环角

所以

$$
\begin{array}{r l} \delta \phi_ {x} (s) & = \frac {M _ {\mathrm{d} x} ^ {a} (s) \cos^ {2} \Psi - \frac {1}{2} M _ {\mathrm{d} y} ^ {a} (s) \sin 2 \Psi}{I _ {f x} [ s ^ {2} + k _ {h} F (s) ]} + \frac {M _ {\mathrm{d} x} ^ {a} (s) \sin^ {2} \Psi + \frac {1}{2} M _ {\mathrm{d} y} ^ {a} (s) \sin 2 \Psi}{I _ {r y} [ s ^ {2} + k _ {h} F (s) ]} \\ \delta \phi_ {y} (s) & = - \frac {\frac {1}{2} M _ {\mathrm{d} x} ^ {a} (s) \sin 2 \Psi - M _ {\mathrm{d} y} ^ {a} (s) \sin^ {2} \Psi}{I _ {f x} [ s ^ {2} + k _ {h} F (s) ]} + \frac {\frac {1}{2} M _ {\mathrm{d} x} ^ {a} (s) \sin 2 \Psi + M _ {\mathrm{d} y} ^ {a} (s) \cos^ {2} \Psi}{I _ {r y} [ s ^ {2} + k _ {h} F (s) ]} \\ \delta \phi_ {z} (s) & = \frac {1}{I _ {a z} [ s ^ {2} + k _ {a} F _ {a} (s) ]} M _ {\mathrm{d} z} ^ {a} (s) \end{array}\tag{5.7.22a}
$$

(5. 7. 22b)

2c)

由式(5.7.21)知，振动诱导干扰力矩的频率为 $\omega _ { s }$ 即转子的自转角速度，远远超出平台电子

网络的频带，即有

$$
F _ {h} (\mathrm{j} \omega_ {s}) = 0, \quad F _ {a} (\mathrm{j} \omega_ {s}) = 0
$$

所以在 $\omega { = } \omega _ { s }$ 附近，式 (5.7.22) 可写成

$$
\delta \phi_ {x} (s) = \left(\frac {\cos^ {2} \Psi}{I _ {f x} s ^ {2}} + \frac {\sin^ {2} \Psi}{I _ {r y} s ^ {2}}\right) M _ {d x} ^ {a} (s) + \left(\frac {\sin 2 \Psi}{2 I _ {r y} s ^ {2}} - \frac {\sin 2 \Psi}{2 I _ {f x} s ^ {2}}\right) M _ {d y} ^ {a} (s)\tag{5.7.23a}
$$

$$
\delta \phi_ {y} (s) = \left(\frac {\sin 2 \Psi}{2 I _ {r y} s ^ {2}} - \frac {\sin 2 \Psi}{2 I _ {f x} s ^ {2}}\right) M _ {d x} ^ {a} (s) + \left(\frac {\sin^ {2} \Psi}{I _ {f x} s ^ {2}} + \frac {\cos^ {2} \Psi}{I _ {r y} s ^ {2}}\right) M _ {d y} ^ {a} (s)\tag{5.7.23b}
$$

$$
\delta \phi_ {z} (s) = \frac {1}{I _ {a z} s ^ {2}} M _ {\mathrm{d} z} ^ {a} (s)\tag{5.7.23c}
$$

记

$$
\begin{array}{c} \Phi_ {x x} (s) = \frac {1}{s ^ {2}} \left(\frac {\cos^ {2} \Psi}{I _ {f x}} + \frac {\sin^ {2} \Psi}{I _ {r y}}\right), \quad \Phi_ {x y} (s) = \frac {\sin 2 \Psi}{2 s ^ {2}} \left(\frac {1}{I _ {r y}} - \frac {1}{I _ {f x}}\right) \\ \Phi_ {y x} (s) = \frac {\sin 2 \Psi}{2 s ^ {2}} \left(\frac {1}{I _ {r y}} - \frac {1}{I _ {f x}}\right), \quad \Phi_ {y y} (s) = \frac {1}{s ^ {2}} \left(\frac {\sin^ {2} \Psi}{I _ {f x}} + \frac {\cos^ {2} \Psi}{I _ {r y}}\right), \Phi_ {z z} (s) = \frac {1}{s ^ {2} I _ {a z}} \end{array}
$$

则式(5.7.23)可画成方块图形式，如图 5.7.3 所示。图中

$$
\begin{array}{l l} \mid \Phi_ {x x} (\mathrm{j} \omega_ {s}) \mid = \frac {1}{\omega_ {s} ^ {2}} \left(\frac {\cos^ {2} \Psi}{I _ {f x}} + \frac {\sin^ {2} \Psi}{I _ {r y}}\right), & \arg \Phi_ {x x} (\mathrm{j} \omega_ {s}) = - \pi \\ \mid \Phi_ {x y} (\mathrm{j} \omega_ {s}) \mid = \frac {\sin 2 \Psi}{2 \omega_ {s} ^ {2}} \left(\frac {1}{I _ {r y}} - \frac {1}{I _ {f x}}\right), & \arg \Phi_ {x y} (\mathrm{j} \omega_ {s}) = - \pi \\ \mid \Phi_ {y x} (\mathrm{j} \omega_ {s}) \mid = \frac {\sin 2 \Psi}{2 \omega_ {s} ^ {2}} \left(\frac {1}{I _ {r y}} - \frac {1}{I _ {f x}}\right), & \arg \Phi_ {y x} (\mathrm{j} \omega_ {s}) = - \pi \\ \mid \Phi_ {y y} (\mathrm{j} \omega_ {s}) \mid = \frac {1}{\omega_ {s} ^ {2}} \left(\frac {\sin^ {2} \Psi}{I _ {f x}} + \frac {\cos^ {2} \Psi}{I _ {r y}}\right), & \arg \Phi_ {y y} (\mathrm{j} \omega_ {s}) = - \pi \\ \mid \Phi_ {z z} (\mathrm{j} \omega_ {s}) \mid = \frac {1}{\omega_ {s} ^ {2} I _ {a z}}, & \arg \Phi_ {z z} (\mathrm{j} \omega_ {s}) = - \pi \end{array}
$$

由线性系统理论知，若线性系统的频率响应为 $\boldsymbol { \varPhi } ( \mathrm { j } \omega )$ ，则系统对简谐输入 $\scriptstyle { x ( t ) = A \sin ( \omega t + }$ $\varphi _ { 0 } )$ 的响应为

$$
y (t) = \left| \Phi (\mathrm{j} \omega) \right| A \sin \left[ \omega t + \varphi_ {0} + \arg \Phi (\mathrm{j} \omega) \right]
$$

所以由图 5.7.3 和式 (5.7.2 1) ，得

$$
\begin{array}{l} \delta \phi_ {x} (t) = \left(\frac {\cos^ {2} \Psi}{I _ {f x}} + \frac {\sin^ {2} \Psi}{I _ {r y}}\right) \times \left[ - m _ {T} \mu_ {0} L _ {1} \sin (\omega_ {s} t - \pi) \right. \\ \left. - m _ {B} \nu_ {1} L _ {2} \cos (\omega_ {s} t + \varphi_ {0} + \varphi_ {2} - \pi) \right] + \frac {\sin 2 \Psi}{2} \left(\frac {1}{I _ {r y}} - \frac {1}{I _ {f x}}\right) m _ {T} \mu_ {0} L _ {1} \cos (\omega_ {s} t - \pi) \\ = \left(\frac {\cos^ {2} \Psi}{I _ {f x}} + \frac {\sin^ {2} \Psi}{I _ {r y}}\right) \left[ m _ {T} \mu_ {0} L _ {1} \sin \omega_ {s} t + m _ {B} \nu_ {1} L _ {2} \cos (\omega_ {s} t + \varphi_ {0} + \varphi_ {2}) \right] \\ - \frac {\sin 2 \Psi}{2} \left(\frac {1}{I _ {r y}} - \frac {1}{I _ {f x}}\right) m _ {T} \mu_ {0} L _ {1} \cos \omega_ {s} t \\ \delta \phi_ {y} (t) = \frac {\sin 2 \Psi}{2} \left(\frac {1}{I _ {r y}} - \frac {1}{I _ {f x}}\right) \left[ - m _ {T} \mu_ {0} L _ {1} \sin (\omega_ {s} t - \pi) - m _ {B} \nu_ {1} L _ {2} \cos (\omega_ {s} t + \varphi_ {0} + \varphi_ {2} - \pi) \right] \\ + \left(\frac {\sin^ {2} \Psi}{I _ {f x}} + \frac {\cos^ {2} \Psi}{I _ {r y}}\right) m _ {T} \mu_ {0} L _ {1} \cos (\omega_ {s} t - \pi) \end{array} \tag {5.7}\tag{5.7.24a}
$$

$$
= \frac {\sin 2 \Psi}{2} \left(\frac {1}{I _ {r y}} - \frac {1}{I _ {f x}}\right) \left[ m _ {T} \mu_ {0} L _ {1} \sin \omega_ {s} t + m _ {B} v _ {1} L _ {2} \cos \left(\omega_ {s} t + \varphi_ {0} + \varphi_ {2}\right) \right]
$$

$$
\left(\frac {\sin^ {2} \Psi}{I _ {f x}} + \frac {\cos^ {2} \Psi}{I _ {r y}}\right) m _ {T} \mu_ {0} L _ {1} \cos \omega_ {s} t\tag{5.7.24b}
$$

$$
\delta \phi_ {z} (t) = \frac {1}{I _ {a z}} m _ {B} v _ {0} L _ {2} \sin (\omega_ {s} t + \varphi_ {0} - \pi) = - \frac {1}{I _ {a z}} m _ {B} v _ {0} L _ {2} \sin (\omega_ {s} t + \varphi_ {0})\tag{5.7.24c}
$$

从式(5. 7. 24)可看出:①陀螺转子的偏心振动诱发的干扰力矩使平台产生同频变化的交变偏差角，即平台出现角抖动;②角抖动的幅值与航向?有关，在某些航向上抖动幅值最大

![](images/d8d9aa7f8eafc9d043033a056496e631c69d98ae6168f0ec248df4f2ad1a16f7.jpg)  
5.7. 振动诱导干扰力 引起的平台偏差角

3) 平台角振动与转子偏心振动耦合诱发的平台漂移

(1)平台角振动引起的转子质心线加速度。

设平台抖动角速度和抖动角加速度分别为 $\delta \dot { \varphi _ { \textit { x } } } \mathrm { , \nabla \delta \dot { \phi _ { \textit { y } } } } \mathrm { , \nabla \delta \dot { \phi } }$ 和( $\delta \ddot { \phi } _ { \textit { x } , \delta } \ddot { \phi } _ { \textit { y } , \delta } \ddot { \phi }$ z 由于抖动角 $\delta \phi _ { x } \backsim \delta \phi _ { y } \backsim \delta \phi _ { z }$ 都很小，所以这些角速度和角加速度可近似看做沿平台台体轴，如图 5. .1 所示。

由平台角振动引起 陀螺质心的切向加速度为

$$
\boldsymbol {t} ^ {a} = \left[ \begin{array}{c} L _ {1} \delta \ddot {\phi} _ {y} \\ - L _ {1} \delta \ddot {\phi} _ {x} \\ 0 \end{array} \right]
$$

引起下陀螺质心的切向加速度为

$$
\boldsymbol {\tau} ^ {a} = \left[ \begin{array}{c} L _ {2} \delta \ddot {\phi} _ {z} \\ 0 \\ - L _ {2} \delta \ddot {\phi} _ {x} \end{array} \right]
$$

将式 (5. 7. 24) 代入上述两式，得

$$
\begin{array}{r l} t _ {x} ^ {a} = & - L _ {1} \omega_ {s} ^ {2} \left\{\frac {\sin 2 \Psi}{2} \left(\frac {1}{I _ {r y}} - \frac {1}{I _ {f x}}\right) \left[ m _ {T} \mu_ {0} L _ {1} \sin \omega_ {s} t + m _ {B} v _ {1} L _ {2} \cos (\omega_ {s} t + \varphi_ {0} + \varphi_ {2}) \right] \right. \\ & \left. - \left(\frac {\sin^ {2} \Psi}{I _ {f x}} + \frac {\cos^ {2} \Psi}{I _ {r y}}\right) m _ {T} \mu_ {0} L _ {1} \cos \omega_ {s} t \right\} \end{array}\tag{5.7.25a}
$$

$$
\begin{array}{l} t _ {y} ^ {a} = L _ {1} \omega_ {s} ^ {2} \left\{\left(\frac {\sin^ {2} \Psi}{I _ {r y}} + \frac {\cos^ {2} \Psi}{I _ {f x}}\right) \left[ m _ {T} \mu_ {0} L _ {1} \sin \omega_ {s} t + m _ {B} \nu_ {1} L _ {2} \cos (\omega_ {s} t + \varphi_ {0} + \varphi_ {2}) \right] \right. \\ \left. - \frac {\sin 2 \Psi}{2} \left(\frac {1}{I _ {r y}} - \frac {1}{I _ {f x}}\right) m _ {T} \mu_ {0} L _ {1} \cos \omega_ {s} t \right\} \end{array}\tag{5.7.25b}
$$

(5.7. 25c)

$$
\tau_ {x} ^ {a} = L _ {2} \omega_ {s} ^ {2} \frac {1}{I _ {a z}} m _ {B} v _ {0} L _ {2} \sin (\omega_ {s} t + \varphi_ {0})\tag{5.7.26a}
$$

(5.7. 26b)

$$
\begin{array}{l} \tau_ {z} ^ {a} = L _ {2} \omega_ {s} ^ {2} \left\{\left(\frac {\sin^ {2} \Psi}{I _ {r y}} + \frac {\cos^ {2} \Psi}{I _ {f x}}\right) \left[ m _ {T} \mu_ {0} L _ {1} \sin \omega_ {s} t + m _ {B} \nu_ {1} L _ {2} \cos (\omega_ {s} t + \varphi_ {0} + \varphi_ {2}) \right] \right. \\ \left. - \frac {\sin 2 \Psi}{2} \left(\frac {1}{I _ {r y}} - \frac {1}{I _ {f x}}\right) m _ {T} \mu_ {0} L _ {1} \cos \omega_ {s} t \right\} \end{array}\tag{5.7.26c}
$$

由平台振动引起上陀螺质心的法向加速度为

$$
\boldsymbol {n} ^ {a} = \left[ \begin{array}{c} 0 \\ 0 \\ L _ {1} (\delta \dot {\phi} _ {y} ^ {2} + \delta \dot {\phi} _ {x} ^ {2}) \end{array} \right]
$$

引起下陀螺质心的法向加速度为

$$
\boldsymbol {u} ^ {a} = \left[ \begin{array}{c} 0 \\ L _ {2} (\delta \dot {\phi} _ {x} ^ {2} + \delta \dot {\phi} _ {z} ^ {2}) \\ 0 \end{array} \right]
$$

由于法向加速度 $\pmb { n } ^ { a }$ 和 $\pmb { u } ^ { a }$ 中每项都含有关于 $\mu _ { 0 } \setminus \upsilon _ { 0 }$ 胁 $\upsilon _ { 1 }$ 阳的三次项，而切向加速度 $t ^ { a }$ 和 $\boldsymbol { \mathsf { I } } \tau ^ { a }$ 只含有上述量的一次项，所以法向加速度 $\pmb { n } ^ { a }$ 和 $\pmb { u } ^ { a }$ 可看作二阶小量忽略不计.

(2) 平台角振动诱发的陀螺干扰力矩。

由上陀螺转子质心加速度引起的惯性力为

$$
\boldsymbol {p} ^ {a} = - m _ {T} \boldsymbol {t} ^ {a} = - m _ {T} \left[ \begin{array}{c} t _ {x} ^ {a} \\ t _ {y} ^ {a} \\ 0 \end{array} \right]
$$

由下陀螺转子质心加速度引起的惯性力为

$$
\boldsymbol {q} ^ {a} = - m _ {B} \boldsymbol {\tau} ^ {a} = - m _ {B} \left[ \begin{array}{c} \tau_ {x} ^ {a} \\ 0 \\ \tau_ {z} ^ {a} \end{array} \right]
$$

$p$ 和 $\pmb q$ 由转子产生，作用在陀螺框架上，对上陀螺支点产生的力矩为

$$
\begin{array}{r l} \boldsymbol {U} ^ {a} = \boldsymbol {\mu} ^ {a} \times \boldsymbol {p} ^ {a} = & - \boldsymbol {p} ^ {a} \times \boldsymbol {\mu} ^ {a} = m _ {T} \left[ \begin{array}{c c c} 0 & 0 & t _ {y} ^ {a} \\ 0 & 0 & - t _ {x} ^ {a} \\ - t _ {y} ^ {a} & t _ {x} ^ {a} & 0 \end{array} \right] \left[ \begin{array}{c} \mu_ {0} \cos \omega_ {s} t \\ \mu_ {0} \sin \omega_ {s} t \\ \mu_ {a} + \mu_ {1} \cos (\omega_ {s} t + \varphi_ {1}) \end{array} \right] \\ = & \left[ \begin{array}{l} m _ {T} t _ {y} ^ {a} [ \mu_ {a} + \mu_ {1} \cos (\omega_ {s} t + \varphi_ {1}) ] \\ - m _ {T} t _ {x} ^ {a} [ \mu_ {a} + \mu_ {1} \cos (\omega_ {s} t + \varphi_ {1}) ] \\ - m _ {T} t _ {y} ^ {a} \mu_ {0} \cos \omega_ {s} t + m _ {T} t _ {x} ^ {a} \mu_ {0} \sin \omega_ {s} t \end{array} \right] \end{array}
$$

对下陀螺支点产生的力矩为

$$
\begin{array}{l} \mathbf {V} ^ {a} = \mathbf {v} ^ {a} \times \mathbf {q} ^ {a} = - \mathbf {q} ^ {a} \times \mathbf {v} ^ {a} = m _ {B} \left[ \begin{array}{c c c} 0 & - \tau_ {z} ^ {a} & 0 \\ \tau_ {z} ^ {a} & 0 & - \tau_ {x} ^ {a} \\ 0 & \tau_ {x} ^ {a} & 0 \end{array} \right] \left[ \begin{array}{c} v _ {0} \sin (\omega_ {s} t + \varphi_ {0}) \\ v _ {0} \cos (\omega_ {s} t + \varphi_ {0}) \\ v _ {a} + v _ {1} \cos (\omega_ {s} t + \varphi_ {0} + \varphi_ {2}) \end{array} \right] \\ = \left[ \begin{array}{c} - m _ {B} \tau_ {z} ^ {a} v _ {0} \cos (\omega_ {s} t + \varphi_ {0}) \\ m _ {B} \tau_ {z} ^ {a} v _ {0} \sin (\omega_ {s} t + \varphi_ {0}) - m _ {B} \tau_ {x} ^ {a} [ v _ {a} + v _ {1} \cos (\omega_ {s} t + \varphi_ {0} + \varphi_ {2}) ] \\ m _ {B} \tau_ {x} ^ {a} v _ {0} \cos (\omega_ {s} t + \varphi_ {0}) \end{array} \right] \end{array}
$$

作用在上陀螺 轴上的干扰力矩为

$$
\begin{array}{r l} M _ {A} ^ {T} = U _ {x} ^ {a} & = m _ {T} L _ {1} \omega_ {s} ^ {2} \left\{\left(\frac {\cos^ {2} \Psi}{I _ {f x}} + \frac {\sin^ {2} \Psi}{I _ {r y}}\right) \times \left[ m _ {T} \mu_ {0} L _ {1} \sin \omega_ {s} t + m _ {B} \nu_ {1} L _ {2} \cos (\omega_ {s} t + \varphi_ {0} + \varphi_ {2}) \right] \right. \\ & \quad \left. - \frac {\sin 2 \Psi}{2} \left(\frac {1}{I _ {r y}} - \frac {1}{I _ {f x}}\right) m _ {T} \mu_ {0} L _ {1} \cos \omega_ {s} t \right\} \cdot \left[ \mu_ {a} + \mu_ {1} \cos (\omega_ {s} t + \varphi_ {1}) \right] \\ M _ {B} ^ {T} = U _ {y} ^ {a} & = m _ {T} L _ {1} \omega_ {s} ^ {2} \left\{\frac {\sin 2 \Psi}{2} \left(\frac {1}{I _ {r y}} - \frac {1}{I _ {f x}}\right) \cdot \left[ m _ {T} \mu_ {0} L _ {1} \sin \omega_ {s} t + m _ {B} \nu_ {1} L _ {2} \cos (\omega_ {s} t + \varphi_ {0} + \varphi_ {2}) \right] \right. \\ & \quad \left. - \left(\frac {\sin^ {2} \Psi}{I _ {f x}} + \frac {\cos^ {2} \Psi}{I _ {r y}}\right) m _ {T} \mu_ {0} L _ {1} \cos \omega_ {s} t \right\} \cdot \left[ \mu_ {a} + \mu_ {1} \cos (\omega_ {s} t + \varphi_ {1}) \right] \end{array}
$$

作用在下陀螺 轴上的干扰力矩为

$$
\begin{array}{l} M _ {A} ^ {B} = V _ {z} ^ {a} = \frac {m _ {B} ^ {2} L _ {2} \omega_ {s} ^ {2}}{I _ {a z}} v _ {0} L _ {2} \sin (\omega_ {s} t + \varphi_ {0}) \cdot v _ {0} \cos (\omega_ {s} t + \varphi_ {0}) \\ M _ {B} ^ {B} = V _ {x} ^ {a} = - m _ {B} L _ {2} \omega_ {s} ^ {2} \left\{\left(\frac {\cos^ {2} \Psi}{I _ {f x}} + \frac {\sin^ {2} \Psi}{I _ {r y}}\right) \cdot \left[ m _ {T} \mu_ {0} L _ {1} \sin \omega_ {s} t + m _ {B} v _ {1} L _ {2} \cos (\omega_ {s} t + \varphi_ {0} + \varphi_ {2}) \right] \right. \\ \left. - \frac {\sin 2 \Psi}{2} \left(\frac {1}{I _ {r y}} - \frac {1}{I _ {f x}}\right) m _ {T} \mu_ {0} L _ {1} \cos \omega_ {s} t \right\} \cdot v _ {0} \cos (\omega_ {s} t + \varphi_ {0}) \end{array}
$$

上述诱导干扰力矩中的直流分量为

$$
\begin{array}{l} \overline {{{M}}} _ {A} ^ {T} = \frac {1}{2} m _ {T} L _ {1} \omega_ {s} ^ {2} \left\{\left(\frac {\cos^ {2} \Psi}{I _ {f x}} + \frac {\sin^ {2} \Psi}{I _ {r y}}\right) [ m _ {T} \mu_ {0} L _ {1} \mu_ {1} \sin (- \varphi_ {1}) \right. \\ \left. + m _ {B} v _ {1} L _ {2} \mu_ {1} \cos (\varphi_ {0} + \varphi_ {2} - \varphi_ {1}) ] - \frac {\sin 2 \Psi}{2} \left(\frac {1}{I _ {r y}} - \frac {1}{I _ {f x}}\right) m _ {T} \mu_ {0} L _ {1} \mu_ {1} \cos \varphi_ {1} \right\} \\ = \frac {1}{2} m _ {T} \mu_ {1} L _ {1} \omega_ {s} ^ {2} \left\{\left(\frac {\cos^ {2} \Psi}{I _ {f x}} + \frac {\sin^ {2} \Psi}{I _ {r y}}\right) [ - m _ {T} \mu_ {0} L _ {1} \sin \varphi_ {1} + m _ {B} v _ {1} L _ {2} \cos (\varphi_ {0} + \varphi_ {2} - \varphi_ {1}) ] \right. \\ \left. - \frac {\sin 2 \Psi}{2} \left(\frac {1}{I _ {r y}} - \frac {1}{I _ {f x}}\right) m _ {T} \mu_ {0} L _ {1} \cos \varphi_ {1} \right\} \\ \overline {{{M}}} _ {B} ^ {T} = \frac {1}{2} m _ {T} L _ {1} \omega_ {s} ^ {2} \left\{\frac {\sin 2 \Psi}{2} \left(\frac {1}{I _ {r y}} - \frac {1}{I _ {f x}}\right) [ m _ {T} \mu_ {0} L _ {1} \mu_ {1} \sin (- \varphi_ {1}) \right. \\ \left. + m _ {B} v _ {1} L _ {2} \mu_ {1} \cos (\varphi_ {0} + \varphi_ {2} - \varphi_ {1}) ] - \left(\frac {\sin^ {2} \Psi}{I _ {f x}} + \frac {\cos^ {2} \Psi}{I _ {r y}}\right) m _ {T} \mu_ {0} L _ {1} \mu_ {1} \cos \varphi_ {1} \right\} \\ = \frac {1}{2} m _ {T} \mu_ {1} L _ {1} \omega_ {s} ^ {2} \left\{\frac {\sin 2 \Psi}{2} \left(\frac {1}{I _ {r y}} - \frac {1}{I _ {f x}}\right) [ - m _ {T} \mu_ {0} L _ {1} \sin \varphi_ {1} + m _ {B} v _ {1} L _ {2} \cos (\varphi_ {0} + \varphi_ {2} - \varphi_ {1}) ] - \left(\frac {\sin^ {2} \Psi}{I _ {f x}} + \frac {\cos^ {2} \Psi}{I _ {r y}}\right) m _ {T} \mu_ {0} L _ {1} \cos \varphi_ {1},\right) \\ - \left(\frac {\sin^ {2} \Psi}{I _ {f x}} + \frac {\cos^ {2} \Psi}{I _ {r y}}\right) m _ {T} \mu_ {0} L _ {1} \cos \varphi_ {1} \\ = 0 \\ m _ {\bar {\Lambda}} ^ {- B} = 0 \\ (5. 7) & (5. 7) \\ (5. 7) & (5. 7) \\ (5. 7) & (5. 7) \\ (5. 7) & (5. 7) \\ (5. 7) & (5. 7) \\ (5. 7) & (5. 7) \\ (5. 7) & (5. 7) \\ (5. 7) & (5. 7) \\ (\textbf {- p r a c t i o n s t a r e f o r e}) & (5. 7) \\ (\textbf {- p r a c t i o n s t a r e}) & (5. 7) \\ (\textbf {- p r a c t i o n s t a r e}) & (5. 7) \\ (\textbf {- p r a c t i o n s t a r e}) & (5. 7) \\ (\textbf {- p r a c t i o n s t a r e}) & (5. 7) \\ (\textbf {- p r a c t i r e s t a r e}) & (5. 7) \\ (\textbf {- p r a c t i r e s t a r e}) & (5. 7) \\ (\textbf {- p r a c t i r e s t a r e}) & (5. 7) \\ (\textbf {- p r a c t i r e s t a r e}) & (5. 7) \\ (\textbf {- p r a t i o n s t a r e}) & (5. 7) \\ (\textbf {- p r a t i r e s t a r e}) & (5. 7) \\ (\textbf {- p r a t i r e s t a r e}) & (5. 7) \\ (\textbf {- p r a t i r e s t a r e}) & (5. 7) \\ (\textbf {- p r a t i r e s t a r e}) & (5. 7) \\ (\textbf {- p r a t i r i o n s t a r e}) & (5. 7) \\ (\textbf {- p r a t i r i o n s t a r e}) & (5. 7) \\ (\textbf {- p r a t i r i o n s t a r e}) & (5. 7) \\ (\textbf {- p r a t i r i o n s t a r e}) & (5. 7) \\ (\textbf {- q u a l l o w h e d}) & (5. 7) \\ (\textbf {- q u a l l o w h e d}) & (5. 7) \\ (\textbf {- q u a l l o w h e d}) & (5. 7) \\ (\textbf {- q u a l l o w h e d}) & (5. 7) \\ (\textbf {- q u a l l o w h e d}) & (5. 7). \\ (\textbf {- q u a l l o w h e d}) & (5. 7). \\ (\textbf {- q u a l l o w h e d}) & (5. 7). \\ (\textbf {- q u a l l o w h e d}) & (5. 7). \\ (\textbf {- q u a l l o w h e d}) & (5. 7). \\ (\textbf {- q u u a l l o w h e d}) & (5. 7). \\ (\textbf {- q u u a l l o w h e d}) & (5. 7). \\ (\textbf {- q u u a l l o w h e d}) & (5. 7). \\ (\textbf {- q u u a l l o w h e d}) & (5. 7). \\ (\textbf {- q u u a l | b u l d}) & (5. 7). \\ (\textbf {- q u u a l | b u l d}) & (5. 7). \\ (\textbf {- q u u a l | b u l d}) & (5. 7). \\ (\textbf {- q u u a l | b u l d}) & (5. 7). \\ (\textbf {- q u u a l | b u l d}) & (5. 6). \\ (\textbf {- q u u a l | b u l d}) & (5. 6). \\ (\textbf {- q u u a l | b u l d}) & (5. 6). \\ (\textbf {- q u u a l | b u l d}) & (5. 6). \\ (\textbf {- q u u a l | b u l d}) & (5. 6). \\ (\textbf - q U A B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I NO B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N O B I N< nl>\tag{5.7.27a}
$$

(5. 7. 27b)

(5. 7. 28a)

$$
\begin{array}{l} \overline {{{M}}} _ {B} ^ {B} = - \frac {1}{2} m _ {B} L _ {2} \omega_ {s} ^ {2} \left\{\left(\frac {\cos^ {2} \Psi}{I _ {f x}} + \frac {\sin^ {2} \Psi}{I _ {r y}}\right) \left[ m _ {T} \mu_ {0} L _ {1} v _ {0} \sin (- \varphi_ {0}) + m _ {B} v _ {1} L _ {2} v _ {0} \cos \varphi_ {2} \right] \right. \\ \left. - \frac {\sin 2 \Psi}{2} \left(\frac {1}{I _ {r y}} - \frac {1}{I _ {f x}}\right) m _ {T} \mu_ {0} L _ {1} v _ {0} \cos \varphi_ {0} \right\} \\ = \frac {1}{2} m _ {B} v _ {0} L _ {2} \omega_ {s} ^ {2} \left[ \left(\frac {\cos^ {2} \Psi}{I _ {f x}} + \frac {\sin^ {2} \Psi}{I _ {r y}}\right) (m _ {T} \mu_ {0} L _ {1} \sin \varphi_ {0} - m _ {B} v _ {1} L _ {2} \cos \varphi_ {2}) \right. \\ \left. - \frac {\sin 2 \Psi}{2} \left(\frac {1}{I _ {r y}} - \frac {1}{I _ {f x}}\right) m _ {T} \mu_ {0} L _ {1} \cos \varphi_ {0} \right] \end{array}\tag{5.7.28b}
$$

## (3) 振动耦合诱发的平台漂移

作用在陀螺 轴上的常值干扰力矩引起陀螺的常值漂移，由式(5.7.16)知，陀螺漂移直接引起平台漂移。所以

$$
\begin{array}{l} \delta \omega_ {x} ^ {a} = \varepsilon_ {A} ^ {T} = - \frac {\overline {{{M}}} _ {B} ^ {T}}{H _ {T}} \\ = - \frac {1}{2 H _ {T}} m _ {T} \mu_ {1} L _ {1} \omega_ {s} ^ {2} \left\{\frac {\sin 2 \Psi}{2} \left(\frac {1}{I _ {r y}} - \frac {1}{I _ {f x}}\right) \times \left[ - m _ {T} \mu_ {0} L _ {1} \sin \varphi_ {1} \right. \right. \\ \left. + m _ {B} v _ {1} L _ {2} \cos (\varphi_ {0} + \varphi_ {2} - \varphi_ {1}) \right] - \left(\frac {\sin^ {2} \Psi}{I _ {f x}} + \frac {\cos^ {2} \Psi}{I _ {r y}}\right) m _ {T} \mu_ {0} L _ {1} \cos \varphi_ {1} \Bigg \} \end{array}\tag{5.7.29a}
$$

$$
\begin{array}{l} \delta \omega_ {y} ^ {a} = \varepsilon_ {B} ^ {T} = \frac {\overline {{{M}}} _ {A} ^ {T}}{H _ {T}} \\ = \frac {1}{2 H _ {T}} m _ {T} \mu_ {1} L _ {1} \omega_ {s} ^ {2} \left\{\left(\frac {\cos^ {2} \Psi}{I _ {f x}} + \frac {\sin^ {2} \Psi}{I _ {r y}}\right) \times \left[ - m _ {T} \mu_ {0} L _ {1} \sin \varphi_ {1} \right. \right. \\ \left. + m _ {B} v _ {1} L _ {2} \cos (\varphi_ {0} + \varphi_ {2} - \varphi_ {1}) \right] - \frac {\sin 2 \Psi}{2} \left(\frac {1}{I _ {r y}} - \frac {1}{I _ {f x}}\right) m _ {T} \mu_ {0} L _ {1} \cos \varphi_ {1} \Bigg \} \end{array}\tag{5.7.29b}
$$

$$
\begin{array}{r l} \delta \omega_ {z} ^ {a} & = \varepsilon_ {A} ^ {B} = - \frac {\overline {{{M}}} _ {B} ^ {B}}{H _ {B}} \\ & = - \frac {1}{2 H _ {B}} m _ {B} \upsilon_ {0} L _ {2} \omega_ {s} ^ {2} \left\{\left(\frac {\cos^ {2} \Psi}{I _ {f x}} + \frac {\sin^ {2} \Psi}{I _ {r y}}\right) \times (m _ {T} \mu_ {0} L _ {1} \sin \varphi_ {0} - m _ {B} \upsilon_ {1} L _ {2} \cos \varphi_ {2}) \right. \\ & \left. - \frac {\sin 2 \Psi}{2} \left(\frac {1}{I _ {r y}} - \frac {1}{I _ {f x}}\right) m _ {T} \mu_ {0} L _ {1} \cos \varphi_ {0} \right\} \end{array}\tag{5.7.29c}
$$

从式 (5.7.29) 可看出

(1)由陀螺偏心振动和平台角振动耦合诱发的平台漂移与航向?有关，在某一航向上漂移会达到最大。当 $I _ { f x } = I _ { r y }$ 时，漂移与航向无关。

(2) 减小诱导漂移的途径有 ①对陀螺作严格的动平衡和静平衡处理 ②采用转子质量小的陀螺 ③上、下陀螺采用不同的驱动转速。

## 6. 圆锥效应漂移

当平台绕某两轴做同频不同相的角振动时，第三轴在空间扫出一个圆锥，即该轴绕圆锥中心线做旋转运动。如果平台采用单自由度陀螺作为敏感器件，则位于该第三轴上的陀螺能感测到这种旋转。这种平台的某两轴做同频不同相的角振动而在第三轴上整流出常值漂移角速度的现象称为平台的圆锥效应。

下面分析圆锥效应产生的机理。

设三轴平台处于中立位置，对平台的指令为零，平台绕各轴做微幅角振动

$$
\theta_ {r} = \theta_ {m} \sin \Omega t, \quad \theta_ {f} = \theta_ {f m} \sin (\Omega t + \alpha_ {1}), \quad \theta_ {a} = \theta_ {a m} \sin (\Omega t + \alpha_ {2})
$$

则绕各轴的振动角速度为

$$
\dot {\theta} _ {r} = \theta_ {m n} \Omega \cos \Omega t, \quad \dot {\theta} _ {f} = \theta_ {f m} \Omega \cos (\Omega t + \alpha_ {1}), \quad \dot {\theta} _ {a} = \theta_ {a m} \Omega \cos (\Omega t + \alpha_ {2})
$$

根据式(5.2. 11)，当不考 $\pmb { \omega } _ { i b }$ 品的影响时

$$
\omega_ {i f x} ^ {f} = \dot {\theta} _ {f} = \theta_ {f m} \Omega \cos (\Omega t + \alpha_ {1}), \quad \omega_ {i r y} ^ {r} = \dot {\theta} _ {r} = \theta_ {m n} \Omega \cos \Omega t
$$

$$
\omega_ {i a z} ^ {a} = \dot {\theta} _ {a} - \dot {\theta} _ {r} \sin \theta_ {f} = \theta_ {a m} \Omega \cos (\Omega t + \alpha_ {2}) - \theta_ {f m} \sin (\Omega t + \alpha_ {1}) \cdot \theta_ {m n} \Omega \cos \Omega t
$$

上述诸式代入式(5.2.6) ，不计 ${ \bf \hat { \omega } } _ { \pmb { \omega } }$ 影响时

$$
\left[ \begin{array}{c} \omega_ {i a x} ^ {a} \\ \omega_ {i a y} ^ {a} \\ \omega_ {i a z} ^ {a} \end{array} \right] = \left[ \begin{array}{c} \omega_ {i f x} ^ {f} \cos \theta_ {a} + \omega_ {i r y} ^ {r} \cos \theta_ {f} \sin \theta_ {a} \\ - \omega_ {i f x} ^ {f} \sin \theta_ {a} + \omega_ {i r y} ^ {r} \cos \theta_ {f} \cos \theta_ {a} \\ \omega_ {i a z} ^ {a} \end{array} \right] = \left[ \begin{array}{c} \theta_ {f m} \Omega \cos (\Omega t + \alpha_ {1}) + \theta_ {m n} \theta_ {a m} \Omega \cos \Omega t \sin (\Omega t + \alpha_ {2}) \\ - \theta_ {f m} \theta_ {a m} \Omega \cos (\Omega t + \alpha_ {1}) \sin (\Omega t + \alpha_ {2}) + \theta_ {m n} \Omega \cos \Omega t \\ \theta_ {a m} \Omega \cos (\Omega t + \alpha_ {2}) - \theta_ {f m} \theta_ {m n} \Omega \sin (\Omega t + \alpha_ {1}) \cos \Omega t \end{array} \right]
$$

上式中的直流分量为

$$
\left[ \begin{array}{c} \overline {{\omega}} _ {k i x} ^ {a} \\ \overline {{\omega}} _ {k i y} ^ {a} \\ \overline {{\omega}} _ {k i z} ^ {a} \end{array} \right] = \left[ \begin{array}{c} \frac {1}{2} \Omega \theta_ {r m} \theta_ {a n} \sin \alpha_ {2} \\ - \frac {1}{2} \Omega \theta_ {f m} \theta_ {a m} \sin (\alpha_ {2} - \alpha_ {1}) \\ - \frac {1}{2} \Omega \theta_ {r m} \theta_ {f m} \sin \alpha_ {1} \end{array} \right]
$$

上述平台角速度是由圆锥效应整流出来的直流分量，是沿两正交轴角振动复合的结果，安装在平台上的陀螺能感测到这些角速度，根据式 $\left( 2 , 3 , 6 \right) , G _ { x } , G _ { y } , G _ { z }$ 陀螺的输出角依次为

$$
\alpha_ {G x} = \frac {k _ {g}}{s (\tau_ {g} s + 1)} \bar {\omega} _ {i a x} ^ {a}, \quad \alpha_ {G y} = \frac {k _ {g}}{s (\tau_ {g} s + 1)} \bar {\omega} _ {i a y} ^ {a}, \quad \alpha_ {G z} = \frac {k _ {g}}{s (\tau_ {g} s + 1)} \bar {\omega} _ {i a z} ^ {a}
$$

根据式(2.3.8) ，陀螺对指令的传递函数为

$$
\frac {\alpha (s)}{\omega_ {\mathrm{cmd}} (s)} = - \frac {k _ {g}}{s (\tau_ {g} s + 1)}
$$

即

$$
\alpha (s) = - \frac {k _ {\mathrm{g}}}{s (\tau_ {\mathrm{g}} s + 1)} \omega_ {\mathrm{cmd}} (s)
$$

而伺服系统只根据陀螺的输出控制平台旋转，不能区别陀螺输出角是由指令引起的还是圆锥效应引起的，只能按照指令来处理，即

$$
\alpha (s) = \frac {k _ {g}}{s (\tau_ {g} s + 1)} \omega_ {I} (s) = - \frac {k _ {g}}{s (\tau_ {g} s + 1)} [ - \omega_ {I} (s) ]
$$

伺服系统将 $\omega _ { I } \left( s \right)$ 误处理为指令

$$
\omega_ {\mathrm{cmd} x} ^ {(e f)} = - \bar {\omega} _ {i a x} ^ {a}, \quad \omega_ {\mathrm{cmd} y} ^ {(e f)} = - \bar {\omega} _ {i a y} ^ {a}, \quad \omega_ {\mathrm{cmd} z} ^ {(e f)} = - \bar {\omega} _ {i a z} ^ {a}
$$

由于伺服回路的作用，平台跟踪这些伪指令，所以平台的实际输出角速度为

$$
\omega_ {i a x} ^ {a (r)} = - \bar {\omega} _ {i a x} ^ {a} = - \frac {1}{2} \Omega \theta_ {m} \theta_ {a m} \sin \alpha_ {2}\tag{5.7.30a}
$$

$$
\omega_ {i a y} ^ {a (r)} = - \bar {\omega} _ {i a y} ^ {a} = \frac {1}{2} \Omega \theta_ {f m} \theta_ {a m} \sin (\alpha_ {2} - \alpha_ {1})\tag{5.7.30b}
$$

$$
\omega_ {i a z} ^ {a (r)} = - \bar {\omega} _ {i a z} ^ {a} = \frac {1}{2} \Omega \theta_ {m} \theta_ {f m} \sin \alpha_ {1}\tag{5.7.30c}
$$

由于对平台的实际指令为零，所以上述输出角速度实质上为平台漂移角速度，是由平台的圆锥运动引起的，由式(5. 7. 30)可看出，当 $\alpha _ { 1 } = 0 , \alpha _ { 2 } = 0$ ，即各轴角振动为同相时，不会引起平台漂移。

