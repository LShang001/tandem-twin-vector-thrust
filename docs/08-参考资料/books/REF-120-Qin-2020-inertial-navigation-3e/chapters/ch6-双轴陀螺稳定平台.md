# 第6章 双轴陀螺稳定平台

## 第6章 双轴陀螺稳定平台

有些使用场合只需双轴稳定就能满足要求，如对射击瞄准线、光轴及地垂线的稳定等。由于减少了一个稳定轴，所以元论从体积、重量及成本都要比三轴稳定平台小。双轴稳定平台的设计基础仍然是单轴稳定平台，本章以稳定瞄准线的双轴平台和模拟水平面(地垂线)的双平台为对象，分析平台的工作原理和存在的问题。

## 6. 1 稳定瞄准线的双轴稳定平台

## 6. 1. 1 原理结构及伺服回路方块图

6. 1. 为稳定瞄准线的双轴稳定平台。图中，方位环与基座用轴承相连，俯仰环安装在方位环内;俯仰环上安装有单自由度陀螺 $G _ { x }$ 和 $G _ { z } , G _ { x }$ 陀螺控制俯仰环轴 $G _ { z }$ 陀螺控制方位环轴;$M _ { a }$ 和 $M _ { f }$ 分别为方位轴力矩马达和俯仰轴力矩马达; $K _ { a }$ 和 $K _ { f }$ 分别为方位回路和俯仰回路伺服放大器;SR 为俯仰正割分解器; $S _ { a }$ 和 $S _ { f }$ 为角度传感器，分别拾取方位环角和俯仰环角;瞄准线(光轴)与 $y _ { f }$ 平行。此处未考虑方位环与俯仰环间的相合。

![](images/05c7dd8ca7e99f2e0a312011828d3aa825d18d9cc676478a3da78918dd4294e5.jpg)  
6. 1. 稳定瞄准线的双轴稳定平台

为分析方便，定义:

(1)基座坐标系 $b { \mathrm { : } }$ 与基座固联;

(2) 方位环坐标系 $a :$ 与方位环固联，当方位环角 $\theta _ { a } = 0$ 时， 系与 系重合;

(3) 俯仰环坐标系 $f { \mathrm { : } }$ 与俯仰环固联，当俯仰角。 $\theta _ { f } = 0$ 时， $f$ 系与 系重合。由图 6. 1. 得变换矩阵:

$$
\boldsymbol {C} _ {b} ^ {a} = \left[ \begin{array}{c c c} \cos \theta_ {a} & \sin \theta_ {a} & 0 \\ - \sin \theta_ {a} & \cos \theta_ {a} & 0 \\ 0 & 0 & 1 \end{array} \right], \quad \boldsymbol {C} _ {a} ^ {f} = \left[ \begin{array}{c c c} 1 & 0 & 0 \\ 0 & \cos \theta_ {f} & \sin \theta_ {f} \\ 0 & - \sin \theta_ {f} & \cos \theta_ {f} \end{array} \right]
$$

假设所采用的单自由度陀螺时间常数很小，陀螺的传递函数为

$$
\frac {k _ {\mathrm{g}}}{s (\tau_ {\mathrm{g}} s + 1)} \approx \frac {k _ {\mathrm{g}}}{s}
$$

且忽略方位环与俯仰环间的耦合，则平台的方块图如图 6. 1. 所示

![](images/7185f86e453b67119593cad6d3739a53209e70ceca58cfab09c573031e462460.jpg)

![](images/e6a4e2871ebe541e55ddbfd74ebb28b0c3969682c809f5a2b5a4d8806923718d.jpg)  
6. 1. 稳定瞄准线的双轴稳定平台方块图

## 6. 1. 2 基座角运动对平台的影晌

设基座角速度为

$$
\boldsymbol {\omega} _ {i b} ^ {b} = \left[ \begin{array}{c} \boldsymbol {\omega} _ {i b x} ^ {b} \\ \boldsymbol {\omega} _ {i b y} ^ {b} \\ \boldsymbol {\omega} _ {i b z} ^ {b} \end{array} \right]
$$

则

$$
\boldsymbol {\omega} _ {i a} ^ {a} = \boldsymbol {C} _ {b} ^ {a} \boldsymbol {\omega} _ {i b} ^ {b} + \boldsymbol {\omega} _ {b a} ^ {a} = \left[ \begin{array}{c c c} \cos \theta_ {a} & \sin \theta_ {a} & 0 \\ - \sin \theta_ {a} & \cos \theta_ {a} & 0 \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c} \boldsymbol {\omega} _ {i b x} ^ {b} \\ \boldsymbol {\omega} _ {i b y} ^ {b} \\ \boldsymbol {\omega} _ {i b z} ^ {b} \end{array} \right] + \left[ \begin{array}{c} 0 \\ 0 \\ \dot {\theta} _ {a} \end{array} \right] = \left[ \begin{array}{c} \boldsymbol {\omega} _ {i b x} ^ {b} \cos \theta_ {a} + \boldsymbol {\omega} _ {i b y} ^ {b} \sin \theta_ {a} \\ - \boldsymbol {\omega} _ {i b x} ^ {b} \sin \theta_ {a} + \boldsymbol {\omega} _ {i b y} ^ {b} \cos \theta_ {a} \\ \boldsymbol {\omega} _ {i b z} ^ {b} + \dot {\theta} _ {a} \end{array} \right]\tag{6.1.1}
$$

式中 $\dot { \theta } _ { a }$ 是 $G _ { z }$ 陀螺感测到 $\omega _ { i b z } ^ { b }$ 擦力矩)后方位环伺服回路控制方位环的旋转角速度 $\dot { \theta } _ { a }$ 的作用是对消 $\omega _ { i b z } ^ { b }$ ，即

$$
\omega_ {i a z} ^ {a} = \omega_ {i b z} ^ {b} + \dot {\theta} _ {a} = 0
$$

此时，俯仰环的角速度为

$$
\boldsymbol {\omega} _ {i f} ^ {f} = \mathbf {C} _ {a} ^ {f} \boldsymbol {\omega} _ {i a} ^ {a} + \boldsymbol {\omega} _ {a f} ^ {f} = \left[ \begin{array}{c c c} 1 & 0 & 0 \\ 0 & \cos \theta_ {f} & \sin \theta_ {f} \\ 0 & - \sin \theta_ {f} & \cos \theta_ {f} \end{array} \right] \left[ \begin{array}{c} \boldsymbol {\omega} _ {i a x} ^ {a} \\ \boldsymbol {\omega} _ {i a y} ^ {a} \\ 0 \end{array} \right] + \left[ \begin{array}{c} \dot {\theta} _ {f} \\ 0 \\ 0 \end{array} \right] = \left[ \begin{array}{c} \boldsymbol {\omega} _ {i a x} ^ {a} + \dot {\theta} _ {f} \\ \boldsymbol {\omega} _ {i a y} ^ {a} \cos \theta_ {f} \\ - \boldsymbol {\omega} _ {i a y} ^ {a} \sin \theta_ {f} \end{array} \right]
$$

式中 $\dot { \theta } _ { f }$ : $G _ { x }$ 陀螺感测 $\omega _ { i a x } ^ { a }$ 后俯仰环伺服回路控制俯仰环的伺服角速度， $\dot { \theta } _ { f }$ 的作用是对消 $\omega _ { i a x } ^ { a }$ 即有

$$
\omega_ {i f x} ^ {f} = \omega_ {i a x} ^ {a} + \dot {\theta} _ {f} = 0\tag{6.1.2a}
$$

$$
\omega_ {i f y} ^ {f} = \omega_ {i a y} ^ {a} \cos \theta_ {f}\tag{6.1.2b}
$$

$$
\omega_ {i f z} ^ {f} = - \omega_ {i a y} ^ {a} \sin \theta_ {f}\tag{6.1.2c}
$$

若 $\theta _ { f } \neq 0$ ，则 $\omega _ { i f z } ^ { f } \neq 0 , G _ { z }$ 陀螺感测此角速度，通过方位环伺服回路控制方位环旋转，考虑正割分解器的作用，伺服回路产生的伺服角速度为

$$
\dot {\theta} _ {z 1} ^ {a} = - \omega_ {i f z} ^ {f} \sec \theta_ {f}
$$

所以俯仰环所具有的角速度为

$$
\begin{array}{r l} \boldsymbol {\omega} _ {i f \Sigma} ^ {f} & = \left[ \begin{array}{c} 0 \\ \omega_ {i f y} ^ {f} \\ \omega_ {i f z} ^ {f} \end{array} \right] + \mathbf {C} _ {a} ^ {f} \left[ \begin{array}{c} 0 \\ 0 \\ \dot {\theta} _ {z 1} ^ {a} \end{array} \right] = \left[ \begin{array}{c} 0 \\ \omega_ {i f y} ^ {f} \\ \omega_ {i f z} ^ {f} \end{array} \right] + \left[ \begin{array}{c c c} 1 & 0 & 0 \\ 0 & \cos \theta_ {f} & \sin \theta_ {f} \\ 0 & - \sin \theta_ {f} & \cos \theta_ {f} \end{array} \right] \left[ \begin{array}{c} 0 \\ 0 \\ - \omega_ {i f z} ^ {f} \sec \theta_ {f} \end{array} \right] \\ & = \left[ \begin{array}{c} 0 \\ \omega_ {i f y} ^ {f} - \omega_ {i f z} ^ {f} \sec \theta_ {f} \sin \theta_ {f} \\ \omega_ {i f z} ^ {f} - \omega_ {i f z} ^ {f} \sec \theta_ {f} \cos \theta_ {f} \end{array} \right] = \left[ \begin{array}{c} 0 \\ \omega_ {i f y} ^ {f} - \omega_ {i f z} ^ {f} \tan \theta_ {f} \\ 0 \end{array} \right] \end{array}
$$

考虑到式(6. 1. 1)和式(6. 1. 2)所列关系，上式可写成

$$
\omega_ {i f \Sigma x} ^ {f} = 0\tag{6.1.3a}
$$

$$
\omega_ {i f \Sigma z} ^ {f} = 0\tag{6.1.3b}
$$

$$
\begin{array}{r l} \omega_ {i f \Sigma y} ^ {f} & = \omega_ {i f y} ^ {f} - \omega_ {i f z} ^ {f} \tan \theta_ {f} = \omega_ {i a y} ^ {a} (\cos \theta_ {f} + \tan \theta_ {f} \sin \theta_ {f}) \\ & = \omega_ {i a y} ^ {a} \frac {1}{\cos \theta_ {f}} (\cos^ {2} \theta_ {f} + \sin^ {2} \theta_ {f}) = (- \omega_ {i b x} ^ {b} \sin \theta_ {a} + \omega_ {i b y} ^ {b} \cos \theta_ {a}) \sec \theta_ {f} \end{array}\tag{6.1.3c}
$$

(6. 1. 3)说明双轴稳定平台不能完全隔离基座的角运动。如果被稳定对象是成像系统的镜头，则镜头绕光轴以 $\omega _ { i f \Sigma y } ^ { f }$ 角速度做旋转运动，这等同于景象绕光轴以 $\cdot \omega _ { i f \Sigma y } ^ { f }$ 在旋转，而且俯仰角。 $\theta _ { f }$ 越大，旋转越快。所以若用双轴稳定平台稳定成像系统镜头，则运载体应该避免做俯仰和横滚运动。

## 6.2 水平式双轴稳定平台

## 6.2.1 系统结构和工作原理

双轴稳定平台可以工作在几何稳定状态，也可以工作在指令跟踪状态。若双轴平台指令跟踪状态的控制信号来自敏感水平面的传感器，且跟踪的对象是当地水平面，则此种平台称为水平式双轴稳定平台。

6.2.1 为水平式双轴稳定平台结构示意图，该平台用来模拟当地水平面。图中，在台体上安装有被稳定对象，为使示意图简洁，被稳定对象在图中未示出。台体 上还安装有两个单自由度陀螺 $G _ { 1 }$ 和 $G _ { 2 }$ ，以及敏感水平面的液体开关 3. $G _ { 1 }$ ，陀螺的信号器输出经 $K _ { x }$ 放大后馈入俯仰力矩电机 ，构成平台的内环轴稳定回路，对绕内环轴的干扰力矩起卸荷作用。 $G _ { 2 }$ ，陀螺的信号器输出经 $K _ { y }$ 放大后馈入横滚力矩电机 ，构成平台的外环轴稳定回路，对绕外环轴的干扰力矩起卸荷作用。液体开关的纵向通道(由触点 1-1 连线确定的方向)输出作为 方向的指令馈人$G _ { 1 }$ 陀螺的力矩器，横向通道(由触点 2-2 连线确定的方向)输出作为 $y$ 方向的指令馈人 $G _ { 2 }$ ，陀螺的力矩器，通过稳定回路的作用分别驱动平台绕俯仰轴和横滚轴旋转，达到跟踪水平面的目的。下面简述系统的工作原理。

![](images/2053b9067a7591187b6fb39d84ccac198a3f2c3a8a39e6df89a4355a115e5348.jpg)  
6. 2. 1 水平式双轴稳定平台结构示意图  
1.俯仰力矩电机; 2. 台体; 3. 液体开关(水平仪); 4. 平台外环;  
5. 基座; 6. 横滚力矩电机; 7. 俯仰角传感器; 8. 横滚角传感器

## 1.稳定回路工作原理

设俯仰轴存在干扰力矩 $M _ { \mathrm { d } f x } ^ { f }$ ' $G _ { 1 }$ 陀螺进动产生输出角 $\alpha _ { 1 }$ .经 $G _ { 1 }$ 陀螺、信号器转换并经 $K _ { x }$ 放大后，馈人俯仰力矩电机产生力矩 $M _ { m f } \circ M _ { m f }$ 的符号与 $\alpha _ { 1 }$ 的符号相反，所以 $M _ { m f }$ 的方向与$M _ { \mathrm { d } f x } ^ { f }$ 的方向相反，当 $M _ { m f } + M _ { \mathrm { d } f x } ^ { f } = 0$ . $M _ { \mathrm { d } f x } ^ { f }$ 被对消，达到了卸荷目的。同理，若横滚轴存在干扰力矩 $M _ { \mathrm { d } r y } ^ { r }$ d巾，横滚力矩电机产生的伺服力矩 $M _ { m r }$ 对 $M _ { \mathrm { d } r y } ^ { r }$ 起卸荷作用。

## 2. 修正回路工作原理

若平台偏离水平面，设绕俯仰轴的偏差角为 $\varphi _ { x }$ ，则水平仪纵向通道有输出，输出电压加至 $G _ { 1 }$ 陀螺力矩器产生输出角，由 $G _ { 1 }$ 陀螺的信号器转换成电压，经 $K _ { x }$ 放大后馈人俯仰力矩电机，电机拖动平台向轩的反方向旋转，直到水平仪的纵向通道输出为霉，从而消除水平偏差角 $\varphi _ { x }$ $\varphi _ { x }$

同理，若平台绕横滚轴的偏差角为 $\varphi _ { y }$ ，则水平仪的横向通道有输出，输出电压加到 $G _ { 2 }$ 陀螺的力矩器，使陀螺产生输出角，经 $G _ { 2 }$ 信号器转换成电压后，由 $K _ { y }$ 放大并馈人横滚力矩电机，拖动平台向冉的反方向旋转，直到水平仪的横向通道输出为零，从而消除 $\varphi _ { y }$ $\varphi _ { y }$

采用单自由度积分陀螺作为敏感器件时，陀螺配置有兰种方案，如图 6.2.2 所示，图中仅示

出了平台台体和陀螺的角动量指向。

![](images/4a14192e4df8980aad60839698dbd1d756296e598de27ade8eff87c60fb11d17.jpg)  
6.2.2 单自由度陀螺的三种配置方案

## 6.2.2 水平式双轴稳定平台的动力学性能分析

1.坐标系及角速度分析

为分析方便，引人如下坐标系:

(1)飞机机体坐标系 $b : x _ { b } \ldots y _ { b } \ldots z _ { b }$ 分别指向飞机的右、前、上。

(2) 当地地平坐标系 $h : x _ { h } \left( X \right) , y _ { h } \left( Y \right)$ 是 $x _ { b } 、 y _ { b }$ 在当地水平面内的技影 $z _ { h } \left( Z \right)$ 沿当地地垂线，向上为正。

(3) 平台坐标系 $p : \boldsymbol { \mathscr { x } } _ { p } \boldsymbol { \mathscr { y } } _ { p } \boldsymbol { \mathscr { z } } _ { p }$ 相对 $x _ { h } y _ { h } z _ { h }$ 存在水平偏差角 $\varphi _ { x }$ 和 $\varphi _ { y } , \varphi _ { z }$ 和 $\varphi _ { y }$ 均为微小角。

(4) 平台外环坐标系 $r :$ 与外环固联，平台安装时外环轴与飞机纵轴平行，所以 $y _ { r }$ 与 $y _ { b }$ 平行。

设飞机的东向速度为吨 $\mathcal { V } _ { E }$ 北向速度为时 $\gamma _ { N }$ 航向角为'P(北偏东为正) ，飞机所在点的纬度为，则

$$
\boldsymbol {\omega} _ {i h} ^ {h} = \mathbf {C} _ {g} ^ {h} \boldsymbol {\omega} _ {i e} ^ {g} + \mathbf {C} _ {g} ^ {h} \boldsymbol {\omega} _ {e g} ^ {g} + \boldsymbol {\omega} _ {g h} ^ {h}\tag{6.2.1}
$$

式中， $g$ 为地理坐标系 $, x _ { g } \setminus y _ { g } \setminus z _ { g }$ 分别指向飞机所在地的东、北、天方向

$$
\boldsymbol {C} _ {g} ^ {h} = \left[ \begin{array}{c c c} \cos \Psi & - \sin \Psi & 0 \\ \sin \Psi & \cos \Psi & 0 \\ 0 & 0 & 1 \end{array} \right]
$$

将式(6.2. 1)写成数学向量形式

$$
\begin{array}{r l} \left[ \begin{array}{c} \omega_ {i h x} ^ {h} \\ \omega_ {i h y} ^ {h} \\ \omega_ {i h z} ^ {h} \end{array} \right] & = \left[ \begin{array}{c c c} \cos \Psi & - \sin \Psi & 0 \\ \sin \Psi & \cos \Psi & 0 \\ 0 & 0 & 1 \end{array} \right] \left\{\left[ \begin{array}{c} 0 \\ \omega_ {i e} \cos L \\ \omega_ {i e} \sin L \end{array} \right] + \left[ \begin{array}{c} - \frac {v _ {N}}{R} \\ \frac {v _ {E}}{R} \\ \frac {v _ {E}}{R} \tan L \end{array} \right] \right\} + \left[ \begin{array}{c} 0 \\ 0 \\ - \dot {\Psi} \end{array} \right] \\ & = \left[ \begin{array}{c} - \frac {v _ {N}}{R} \cos \Psi - \frac {v _ {E}}{R} \sin \Psi - \omega_ {i e} \cos L \sin \Psi \\ - \frac {v _ {N}}{R} \sin \Psi + \frac {v _ {E}}{R} \cos \Psi + \omega_ {i e} \cos L \cos \Psi \\ \frac {v _ {E}}{R} \tan L + \omega_ {i e} \sin L - \dot {\Psi} \end{array} \right] \end{array}
$$

平台的绝对角速度为

$$
\boldsymbol {\omega} _ {i p} ^ {p} = \boldsymbol {C} _ {h} ^ {p} \boldsymbol {\omega} _ {i h} ^ {h} + \boldsymbol {\omega} _ {h p} ^ {p}\tag{6.2.2}
$$

假设平台相对水平面存在偏差角 $\varphi _ { x }$ 和 $\varphi _ { y }$ ，则

$$
\boldsymbol {C} _ {h} ^ {p} = \left[ \begin{array}{c c c} 1 & 0 & - \varphi_ {y} \\ 0 & 1 & \varphi_ {x} \\ \varphi_ {y} & - \varphi_ {x} & 1 \end{array} \right], \quad \boldsymbol {\omega} _ {h p} ^ {p} = \left[ \begin{array}{c} \dot {\varphi} _ {x} \\ \dot {\varphi} _ {y} \\ 0 \end{array} \right]
$$

所以式 (6.2.2) 可写成

$$
\omega_ {X} = \dot {\varphi} _ {x} + \omega_ {i h x} ^ {h} - \varphi_ {y} \omega_ {i h z} ^ {h} \approx \dot {\varphi} _ {x} + \omega_ {i h x} ^ {h}\tag{6.2.3a}
$$

$$
\omega_ {Y} = \dot {\varphi} _ {y} + \omega_ {i h y} ^ {h} + \varphi_ {x} \omega_ {i h z} \approx \dot {\varphi} _ {y} + \omega_ {i h y} ^ {h}\tag{6.2.3b}
$$

$$
\omega_ {Z} = \omega_ {i h z} ^ {h} + \varphi_ {y} \omega_ {i h x} ^ {h} - \varphi_ {x} \omega_ {i h y} ^ {h} \approx \omega_ {i h z} ^ {h}\tag{6.2.3c}
$$

上式中， $\pmb { \omega } _ { i p } ^ { p } { = } [ \omega _ { X } \quad \omega _ { Y } \quad \omega _ { Z } ] ^ { \mathrm { T } }$ ，并略去了由水平偏差角引起的角速度耦合影响。

## 2. 各环节的传递函数

## 1)水平仪修正回路的传递函数

设 $k _ { u _ { 1 } }$ 和 $k _ { u _ { \gamma } }$ 分别为液体开关纵向通道(由 1-1 触点确定)和横向通道(由 2-2 触点确定)的传递系数， $\mathbf { \nabla } _ { \mathbf { \Omega } } \mathcal { k } _ { T _ { 1 } }$ 和 $k _ { T _ { 2 } }$ 分别为 $G _ { 1 }$ 陀螺和 $G _ { 2 }$ 陀螺力矩器的力矩系数，则加在 $G _ { 1 }$ 陀螺和 $G _ { 2 }$ 陀螺输出轴上的控制力矩分别为

$$
M _ {C 1} = k _ {T _ {1}} \cdot k _ {u _ {1}} \cdot \varphi_ {x}, \quad M _ {C 2} = k _ {T _ {2}} \cdot k _ {u _ {2}} \cdot \varphi_ {y}
$$

写成复频域形式

$$
M _ {C 1} (s) = k _ {\varphi_ {x}} \varphi_ {x} (s)\tag{6.2.4a}
$$

$$
M _ {C 2} (s) = k _ {\varphi_ {y}} \varphi_ {y} (s)\tag{6.2.4b}
$$

式中，是 $k _ { \varphi _ { x } } = k _ { T _ { 1 } } k _ { u _ { 1 } } , k _ { \varphi _ { v } } = k _ { T _ { 2 } } k _ { u _ { 2 } }$

2) 陀螺的输出角

设由 $M _ { C 1 }$ 和 $M _ { C 2 }$ 引起陀螺 $G _ { 1 }$ $G _ { 2 }$ 的输出角分别为 $\alpha _ { 1 }$ 和 $\alpha _ { 2 }$

6. 2. 示出了框架坐标系与测量坐标系间的关系，图 6.2.3 (a) 为陀螺 $G _ { 1 }$ 的关系，6. 2. 3(b) 为陀螺 $G _ { 2 }$ 的关系。由图得

![](images/ddcf81ca1cd41d0ae76a6001036cec3286874aec4779ca58921c9557fcbd38e3.jpg)  
(a)

![](images/3587728d4187b7c92126df8390059925edc8d92e661018168e19d25c87eeab1b.jpg)  
(b)  
6.2.3 陀螺 $G _ { 1 }$ 和 $G _ { 2 }$ 的框架坐标系与测量坐标系的关系

$$
\begin{array}{l} I _ {O _ {1}} \left(\ddot {\alpha} _ {1} + \dot {\omega} _ {Y}\right) = - D _ {1} \dot {\alpha} _ {1} + H _ {1} \omega_ {X} \cos \alpha_ {1} - H _ {1} \omega_ {Z} \sin \alpha_ {1} + M _ {C 1} + M _ {d 1} \\ I _ {O _ {2}} \left(\ddot {\alpha} _ {2} + \dot {\omega} _ {X}\right) = - D _ {2} \dot {\alpha} _ {2} + H _ {2} \omega_ {Y} \cos \alpha_ {2} + H _ {2} \omega_ {Z} \sin \alpha_ {2} + M _ {C 2} + M _ {d 2} \end{array}
$$

式中， $M _ { \mathrm { d l } }$ 和 $M _ { \mathrm { d 2 } }$ 为干扰力矩。由于输出角 $\alpha _ { 1 }$ 和 $\alpha _ { 2 }$ 都很小，且变化范围也很小，在计算陀螺力矩中可近似视为常值 $\alpha _ { 1 o }$ 和 $\alpha _ { 2 o }$ 所以

$$
\begin{array}{c} \cos \alpha_ {1} \approx 1, \quad \cos \alpha_ {2} \approx 1, \quad \sin \alpha_ {1} \approx \alpha_ {1 o}, \quad \sin \alpha_ {2} \approx \alpha_ {2 o} \\ I _ {O _ {1}} s [ s \alpha_ {1} (s) + \omega_ {Y} (s) ] + D _ {1} s \alpha_ {1} (s) = H _ {1} \omega_ {X} (s) - H _ {1} \omega_ {Z} (s) \alpha_ {1 o} + M _ {\mathrm{C} 1} (s) + M _ {\mathrm{d} 1} (s) \\ I _ {O _ {2}} s [ s \alpha_ {2} (s) + \omega_ {X} (s) ] + D _ {2} s \alpha_ {2} (s) = H _ {2} \omega_ {Y} (s) + H _ {2} \omega_ {Z} (s) \alpha_ {2 o} + M _ {\mathrm{C} 2} (s) + M _ {\mathrm{d} 2} (s) \end{array}
$$

将式 (6.2.3) 和式 (6.2.4) 代入上式，得

$$
\begin{array}{r l} & I _ {O _ {1}} s \big [ s \alpha_ {1} (s) + s \varphi_ {y} (s) + \omega_ {i h y} ^ {h} (s) \big ] + D _ {1} s \alpha_ {1} (s) \\ & = H _ {1} \big [ s \varphi_ {x} (s) + \omega_ {i h x} ^ {h} (s) \big ] - H _ {1} \alpha_ {1 o} \omega_ {i h z} ^ {h} (s) + k _ {\varphi_ {x}} \varphi_ {x} (s) + M _ {\mathrm{d} 1} (s) \\ & I _ {O _ {2}} s \big [ s \alpha_ {2} (s) + s \varphi_ {x} (s) + \omega_ {i h x} ^ {h} (s) \big ] + D _ {2} s \alpha_ {2} (s) \\ & = H _ {2} \big [ s \varphi_ {y} (s) + \omega_ {i h y} ^ {h} (s) \big ] + H _ {2} \alpha_ {2 o} \omega_ {i h z} ^ {h} (s) + k _ {\varphi_ {y}} \varphi_ {y} (s) + M _ {\mathrm{d} 2} (s) \end{array}
$$

所以

$$
\begin{array}{l} \alpha_ {1} (s) = \frac {\left(k _ {\varphi_ {x}} + H _ {1} s\right) \varphi_ {x} (s) - I _ {O _ {1}} s ^ {2} \varphi_ {y} (s) + H _ {1} \left[ \omega_ {t h x} ^ {h} (s) - \alpha_ {1 o} \omega_ {t h z} ^ {h} (s) \right] + M _ {\mathrm{d} 1} (s) - I _ {O _ {1}} s \omega_ {t h y} ^ {h} (s)}{s \left(I _ {O _ {1}} s + D _ {1}\right)} \\ \alpha_ {2} (s) = \frac {\left(k _ {\varphi_ {y}} + H _ {2} s\right) \varphi_ {y} (s) - I _ {O _ {2}} s ^ {2} \varphi_ {x} (s) + H _ {2} \left[ \omega_ {t h y} ^ {h} (s) + \alpha_ {2 o} \omega_ {t h z} ^ {h} (s) \right] + M _ {\mathrm{d} 2} (s) - I _ {O _ {2}} s \omega_ {t h x} ^ {h} (s)}{s \left(I _ {O _ {2}} s + D _ {2}\right)} \end{array}\tag{6.2.5a}
$$

(6. 2. 5b)

## 3) 稳定回路的传递函数

由图 6.2.1 ， $G _ { 1 }$ (;，陀螺的输出角 $\alpha _ { 1 }$ 经信号器转换成电压信号 $k _ { \alpha _ { 1 } } \cdot \alpha _ { 1 }$ ，经放大校正后成为$\boldsymbol { k } _ { a f } \bullet \boldsymbol { F } _ { f } ( s ) \bullet \boldsymbol { k } _ { \alpha _ { 1 } } \bullet \boldsymbol { \alpha } _ { 1 }$ 反相后馈入俯仰力矩电机，产生的俯仰驱动力矩为

$$
M _ {m f} = - k _ {m f} k _ {a f} k _ {\alpha_ {1}} F _ {f} (s) \alpha_ {1}
$$

所以俯仰稳定回路的传递函数为

$$
\frac {M _ {m f} (s)}{\alpha_ {1} (s)} = - k _ {m f} k _ {a f} k _ {\alpha_ {1}} F _ {f} (s) = - k _ {f} F _ {f} (s)\tag{6.2.6}
$$

同理得横滚稳定回路的传递函数为

$$
\frac {M _ {n r} (s)}{\alpha_ {2} (s)} = - k _ {r} F _ {r} (s)\tag{6.2.7}
$$

式中

$$
k _ {f} = k _ {m f} k _ {a f} k _ {\alpha_ {1}}, \quad k _ {r} = k _ {m r} k _ {a r} k _ {\alpha_ {2}}
$$

## 。支撑环架的传递函数

## (1)沿环架轴的外作用力矩。

设平台的俯仰环架角为 $\theta _ { f }$ (平台台体相对横滚环的旋转角) ，如图 6.2.4

由于陀螺壳体固联在台体(俯仰环)上，且 $G _ { 1 }$ ，的输出轴 $O _ { 1 }$ 平行于 Yf $y _ { f }$ $( Y$ 轴)， $G _ { 2 }$ ，的输出轴$O _ { 2 }$ 平行于 $\boldsymbol { \mathcal { X } } _ { f }$ 轴(见图 6. 2. 1)，所以陀螺的指令力矩 $M _ { C 1 }$ 和 $M _ { C 2 }$ 会对台体产生反作用力矩，转子产生的陀螺力矩通过框架也作用在台体上，根据图 6.2.1，可确定出沿平台轴陀螺对平台的反作用力矩为

$$
M _ {G P X} = - M _ {C 2} - H _ {2} \left(\omega_ {Y} + \dot {\alpha} _ {1}\right), \quad M _ {G P Y} = - M _ {C 1} - H _ {1} \left(\omega_ {X} + \dot {\alpha} _ {2}\right), \quad M _ {G P Z} = 0
$$

![](images/21cf0517e5f47d42a98791f961e1714d81c84d343de364c8b3ca5c0c4490adc5.jpg)  
6.2.4 $\theta _ { f } \ne 0$ 时的俯仰环和横滚环

将上述投影变换到横滚环坐标系 $r$ 中

$$
\boldsymbol {M} _ {G P} ^ {r} = \boldsymbol {C} _ {P} ^ {r} \boldsymbol {M} _ {G P} ^ {P} = \left[ \begin{array}{c c c} 1 & 0 & 0 \\ 0 & \cos \theta_ {f} & \sin \theta_ {f} \\ 0 & - \sin \theta_ {f} & \cos \theta_ {f} \end{array} \right] \left[ \begin{array}{c} M _ {G P X} \\ M _ {G P Y} \\ 0 \end{array} \right]
$$

即有

$$
M _ {G P y} ^ {r} = M _ {G P Y} \cos \theta_ {f} = \left[ - M _ {C 1} - H _ {1} \left(\omega_ {X} + \dot {\alpha} _ {2}\right) \right] \cos \theta_ {f}
$$

这一力矩通过俯仰环的几何约束传递至横滚环轴上，所以作用在横滚轴上的总外力矩为

$$
M _ {r _ {\mathrm{y}}} = M _ {\mathrm{GPy}} ^ {r} + M _ {m r} + M _ {\mathrm{dr}} = \left[ - M _ {\mathrm{Cl}} - H _ {1} \left(\omega_ {\mathrm{X}} + \dot {\alpha_ {2}}\right) \right] \cos \theta_ {f} + M _ {m r} + M _ {\mathrm{dr}}
$$

$M _ { G P X }$ 为作用在俯仰轴上的反作用力矩，即

$$
M _ {G P x} ^ {f} = M _ {G P X}
$$

所以作用在俯仰轴上的总外力矩为

$$
\begin{array}{r l} M _ {f x} & = M _ {\mathrm{GP} x} ^ {f} + M _ {m f} + M _ {\mathrm{df}} \\ & = - M _ {\mathrm{C2}} - H _ {2} \left(\omega_ {\mathrm{Y}} + \dot {\alpha_ {1}}\right) + M _ {m f} + M _ {\mathrm{df}} \end{array}
$$

将式 (6.2.3) 、式 (6.2.4) 、式 (6.2.6) 和式 (6.2.7) 代入上述两式:

$$
\begin{array}{r l} M _ {f x} (s) & = - k _ {\varphi_ {y}} \varphi_ {y} (s) - H _ {2} [ \omega_ {Y} (s) + s \alpha_ {1} (s) ] - k _ {f} F _ {f} (s) \alpha_ {1} (s) + M _ {\mathrm{d} f} (s) \\ & = - k _ {\varphi_ {y}} \varphi_ {y} (s) - H _ {2} [ \omega_ {t h y} ^ {h} (s) + s \varphi_ {y} (s) + s \alpha_ {1} (s) ] - k _ {f} F _ {f} (s) \alpha_ {1} (s) + M _ {\mathrm{d} f} (s) \\ & = - (H _ {2} s + k _ {\varphi_ {y}}) \varphi_ {y} (s) - [ H _ {2} s + k _ {f} F _ {f} (s) ] \alpha_ {1} (s) - H _ {2} \omega_ {t h y} ^ {h} (s) + M _ {\mathrm{d} f} (s) \end{array}\tag{6.2.8}
$$

$$
\begin{array}{r l} M _ {r y} (s) & = \{- k _ {\varphi_ {x}} \varphi_ {x} (s) - H _ {1} [ \omega_ {X} (s) + s \alpha_ {2} (s) ] \} \cos \theta_ {f} - k _ {r} F _ {r} (s) \alpha_ {2} (s) + M _ {\mathrm{dr}} (s) \\ & = \{- k _ {\varphi_ {x}} \varphi_ {x} (s) - H _ {1} [ \omega_ {i h x} ^ {h} (s) + s \varphi_ {x} (s) + s \alpha_ {2} (s) ] \} \cos \theta_ {f} - k _ {r} F _ {r} (s) \alpha_ {2} (s) + M _ {\mathrm{dr}} (s) \\ & = - (H _ {1} s + k _ {\varphi_ {x}}) \cos \theta_ {f} \varphi_ {x} (s) - [ H _ {1} s \cos \theta_ {f} + k _ {r} F _ {r} (s) ] \alpha_ {2} (s) \\ & - H _ {1} \cos \theta_ {f} \omega_ {i h x} ^ {h} (s) + M _ {\mathrm{dr}} (s) \end{array} \tag {6.2}\tag{6.2.9}
$$

(2) 各环架的传递函数。

设平台台体绕 轴的转动惯量分别为 $J _ { X \setminus J _ { Y } \setminus J _ { Z } }$ .外环(不包含台体)绕 $y _ { r }$ 轴的转动惯量为 $J _ { r y }$ ，则外环组件(包含外环和台体)绕 $y _ { r }$ 轴的转动惯量为

$$
J _ {\Sigma y} = J _ {r y} + J _ {Y} \cos^ {2} \theta_ {f} + J _ {Z} \sin^ {2} \theta_ {f}
$$
