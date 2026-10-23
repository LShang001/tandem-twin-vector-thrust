根据动量矩定理

$$
J _ {\Sigma y} \dot {\omega} _ {i r y} ^ {r} = M _ {r y}\tag{6.2.10a}
$$

$$
J _ {X} \dot {\omega} _ {i f x} ^ {f} = M _ {f x}\tag{6.2.10b}
$$

式中， $\bullet \omega _ { i r y } ^ { r }$ 和 $\omega _ { i f x } ^ { f }$ 是电机驱动产生的平台运动角速度.

由于俯仰轴 $\boldsymbol { \mathcal { X } } _ { f }$ 就是平台的 轴，所以电机的驱动角速度就是平台的俯仰角速度

$$
\omega_ {i f x} ^ {f} = \omega_ {X}
$$

但当存在俯仰环架角。 $\theta _ { f }$ 时(图 6.2.4) ，横滚轴驱动电机产生的驱动角速度 $\omega _ { i r y } ^ { r }$ 仅 $\omega _ { i r y } ^ { r } \cos \theta _ { f }$ 可产生平台绕 轴的驱动角速度，即有

$$
\omega_ {i r y} ^ {r} \cos \theta_ {f} = \omega_ {Y}\tag{6.2.11a}
$$

所以

$$
\omega_ {i r y} ^ {r} = \omega_ {\mathrm{Y}} \sec \theta_ {f}\tag{6.2.11b}
$$

根据式 (6.2. 11)，式 (6.2.10) 可写成

$$
\begin{array}{c} J _ {\Sigma y} s \omega_ {Y} (s) \sec \theta_ {f} = M _ {r y} (s), \quad J _ {X} s \omega_ {X} (s) = M _ {f x} (s), \\ \omega_ {Y} (s) = \frac {M _ {r y} (s) \cos \theta_ {f}}{s J _ {\Sigma y}}, \quad \omega_ {X} (s) = \frac {M _ {f x} (s)}{s J _ {X}} \end{array}
$$

将式 (6.2.3) 、式 (6.2.8) 和式 (6.2.9)代人上述两式，得

$$
\begin{array}{r l} s \varphi_ {x} (s) + \omega_ {i h x} ^ {h} (s) & = \{- (H _ {2} s + k _ {\varphi_ {y}}) \varphi_ {y} (s) - [ H _ {2} s + k _ {f} F _ {f} (s) ] \alpha_ {1} (s) \\ & \quad - H _ {2} \omega_ {i h y} ^ {h} (s) + M _ {\mathrm{d} f} (s) \} / s J _ {X} \\ s \varphi_ {y} (s) + \omega_ {i h y} ^ {h} (s) & = \{- (H _ {1} s + k _ {\varphi_ {x}}) \cos \theta_ {f} \varphi_ {x} (s) - [ H _ {1} s \cos \theta_ {f} + k _ {r} F _ {r} (s) ] \alpha_ {2} (s) \\ & \quad - H _ {1} \cos \theta_ {f} \omega_ {i h x} ^ {h} (s) + M _ {\mathrm{d} r} (s) \} \cos \theta_ {f} / s J _ {\Sigma y} \end{array}
$$

所以

$$
\begin{array}{r l} \varphi_ {x} (s) = & - \frac {1}{s ^ {2} J _ {X}} \left\{\left[ H _ {2} s + k _ {f} F _ {f} (s) \right] \alpha_ {1} (s) + \left(H _ {2} s + k _ {\varphi_ {y}}\right) \varphi_ {y} (s) \right. \\ & \left. + H _ {2} \omega_ {i h y} ^ {h} (s) - M _ {\mathrm{df}} (s) \right\} - \frac {\omega_ {i h x} ^ {h} (s)}{s} \end{array}\tag{6.2.12a}
$$

$$
\begin{array}{l} \varphi_ {y} (s) = - \frac {\cos \theta_ {f}}{s ^ {2} J _ {\Sigma y}} \left\{\left[ H _ {1} s \cos \theta_ {f} + k _ {r} F _ {r} (s) \right] \alpha_ {2} (s) + \left(H _ {1} s + k _ {\varphi_ {x}}\right) \cos \theta_ {f} \varphi_ {x} (s) \right. \\ \left. + H _ {1} \cos \theta_ {f} \omega_ {i h x} ^ {h} (s) - M _ {\mathrm{d} r} (s) \right\} - \frac {\omega_ {i h y} ^ {h} (s)}{s} \end{array}\tag{6.2.12b}
$$

## 3. 系统方块囹

根据式(6.2.5)和式(6.2.12) ，可画出水平双轴平台的方块图，如图 6.2.5 所示。

## 4. 革统稳定性分析

为简化分析，忽略通道间的交叉搞合，并假设 $F _ { r } ( s ) = F _ { f } ( s ) = 1$ 则由图 6.2.5，平台偏差角$\varphi _ { x } \ldots \varphi _ { y }$ 与陀螺干扰力矩 $M _ { \mathrm { d 1 } }$ 1和 $M _ { \mathrm { d 2 } }$ ，间的传递函数为

![](images/77ec04b13fe1332fc5b18f966e2b3ef7e7e76926d8078aa08f240a082be5cf8c.jpg)  
6.2.5 水平式双轴平台方块图

$$
\begin{array}{r l} H _ {x} (s) & = \frac {\varphi_ {x} (s)}{M _ {\mathrm{d1}} (s)} = \frac {\frac {1}{s \left(I _ {O 1} s + D _ {1}\right)} \left(H _ {2} s + k _ {f}\right) \left(- \frac {1}{s ^ {2} J _ {X}}\right)}{1 + \frac {1}{s \left(I _ {O 1} s + D _ {1}\right)} \left(H _ {2} s + k _ {f}\right) \frac {1}{s ^ {2} J _ {X}} \left(H _ {1} s + k _ {\varphi_ {x}}\right)} \\ & = - \frac {H _ {2} s + k _ {f}}{s ^ {3} J _ {X} \left(I _ {O 1} s + D _ {1}\right) + \left(H _ {2} s + k _ {f}\right) \left(H _ {1} s + k _ {\varphi_ {x}}\right)} \\ & = - \frac {H _ {2} s + k _ {f}}{J _ {X} I _ {O 1} s ^ {4} + J _ {X} D _ {1} s ^ {3} + H _ {1} H _ {2} s ^ {2} + \left(H _ {1} k _ {f} + H _ {2} k _ {\varphi_ {x}}\right) s + k _ {f} k _ {\varphi_ {x}}} \\ H _ {y} (s) & = \frac {\varphi_ {y} (s)}{M _ {\mathrm{d2}} (s)} = \frac {\frac {1}{s \left(I _ {O 2} s + D _ {2}\right)} \left(H _ {1} s \cos \theta_ {f} + k _ {r}\right) \left(- \frac {\cos \theta_ {f}}{s ^ {2} J _ {\Sigma y}}\right)}{1 + \frac {1}{s \left(I _ {O 2} s + D _ {2}\right)} \left(H _ {1} s \cos \theta_ {f} + k _ {r}\right) \frac {\cos \theta_ {f}}{s ^ {2} J _ {\Sigma y}} \left(H _ {2} s + k _ {\varphi_ {y}}\right)} \\ & = - \frac {(H _ {1} s \cos \theta_ {f} + k _ {r}) \cos \theta_ {f}}{s ^ {3} J _ {\Sigma y} \left(I _ {O 2} s + D _ {2}\right) + (H _ {1} s \cos \theta_ {f} + k _ {r}) \cos \theta_ {f} \left(H _ {2} s + k _ {\varphi_ {y}}\right)} \\ & = - \frac {(H _ {1} s \cos \theta_ {f} + k _ {r}) \cos \theta_ {f}}{J _ {\Sigma y} I _ {O 2} s ^ {4} + J _ {\Sigma y} D _ {2} s ^ {3} + H _ {1} H _ {2} \cos^ {2} \theta_ {f} s ^ {2} + (H _ {1} k _ {\varphi_ {y}} \cos^ {2} \theta_ {f} + H _ {2} k _ {r} \cos \theta_ {f}) s + k _ {r} k _ {\varphi_ {y}} \cos \theta_ {f}}. \end{array} \tag {6.}\tag{6.2.13a}
$$

(6.2. 13b)

通道和 通道的特征方程为

$$
\begin{array}{l} \Delta_ {x} = J _ {X} I _ {O 1} s ^ {4} + J _ {X} D _ {1} s ^ {3} + H _ {1} H _ {2} s ^ {2} + (H _ {1} k _ {f} + H _ {2} k _ {\varphi_ {x}}) s + k _ {f} k _ {\varphi_ {x}} = 0 \\ \Delta_ {y} = J _ {\Sigma y} I _ {O 2} s ^ {4} + J _ {\Sigma y} D _ {2} s ^ {3} + H _ {1} H _ {2} \cos^ {2} \theta_ {f} s ^ {2} + (H _ {1} k _ {\varphi_ {y}} \cos^ {2} \theta_ {f} + H _ {2} k _ {r} \cos \theta_ {f}) s + k _ {r} k _ {\varphi_ {y}} \cos \theta_ {f} = 0 \end{array}
$$

现以 $y$ 通道为对象，分析系统的稳定性。记

$$
\Delta_ {y} = a _ {0} s ^ {4} + a _ {1} s ^ {3} + a _ {2} s ^ {2} + a _ {3} s + a _ {4}
$$

则

$$
\begin{array}{l} a _ {0} = J _ {\Sigma y} I _ {O 2}, \quad a _ {1} = J _ {\Sigma y} D _ {2}, \quad a _ {2} = H _ {1} H _ {2} \cos^ {2} \theta_ {f}, \\ a _ {3} = H _ {1} k _ {\varphi_ {y}} \cos^ {2} \theta_ {f} + H _ {2} k _ {r} \cos \theta_ {f}, \quad a _ {4} = k _ {r} k _ {\varphi_ {y}} \cos \theta_ {f} \end{array}
$$

根据霍尔维茨判据，要使 通道稳定，需满足=

$$
a _ {i} > 0 \quad i = 0, 1, 2, 3, 4
$$

$$
D _ {2} = \left| \begin{array}{l l} a _ {1} & a _ {3} \\ a _ {0} & a _ {2} \end{array} \right| = a _ {1} a _ {2} - a _ {0} a _ {3} > 0, \quad D _ {3} = \left| \begin{array}{c c c} a _ {1} & a _ {3} & 0 \\ a _ {0} & a _ {2} & a _ {4} \\ 0 & a _ {1} & a _ {3} \end{array} \right| = a _ {3} D _ {2} - a _ {4} a _ {1} ^ {2} > 0
$$

$$
D _ {4} = \left| \begin{array}{c c c c} a _ {1} & a _ {3} & 0 & 0 \\ a _ {0} & a _ {2} & a _ {4} & 0 \\ 0 & a _ {1} & a _ {3} & 0 \\ 0 & a _ {0} & a _ {2} & a _ {4} \end{array} \right| = a _ {4} D _ {3} > 0
$$

下面分析俯仰角。 $\theta _ { f }$ 对 $y$ 通道稳定性的影响.由

$$
D _ {2} = a _ {1} a _ {2} - a _ {0} a _ {3} = J _ {\Sigma y} D _ {2} H _ {1} H _ {2} \cos^ {2} \theta_ {f} - J _ {\Sigma y} I _ {O 2} \left(H _ {1} k _ {\varphi_ {y}} \cos^ {2} \theta_ {f} + H _ {2} k _ {r} \cos \theta_ {f}\right) > 0
$$

得

$$
\left(D _ {2} H _ {1} H _ {2} - I _ {O 2} H _ {1} k _ {\varphi_ {y}}\right) \cos \theta_ {f} > I _ {O 2} H _ {2} k _ {r}
$$

即

$$
\begin{array}{l} \frac {I _ {\mathrm{O2}} H _ {2} k _ {r}}{D _ {2} H _ {1} H _ {2} - I _ {\mathrm{O2}} H _ {1} k _ {\varphi_ {y}}} <   \cos \theta_ {f} \\ k _ {r} <   \frac {(D _ {2} H _ {1} H _ {2} - I _ {\mathrm{O2}} H _ {1} k _ {\varphi_ {y}}) \cos \theta_ {f}}{I _ {\mathrm{O2}} H _ {2}} \end{array}\tag{6.2.14}
$$

由式(6.2.14)可看出，横滚通道的稳定性要求随俯仰角。 $\theta _ { f }$ 而变，这使水平式双轴平台的横滚通道稳定性设计具有较大的难度。设计中可根据临界角 $\theta _ { f c }$ 所确定的稳定条件设计系统，在实际工作过程中，运载体的俯仰角应远离此临界角 $\theta _ { f c }$ 。也可加入俯仰正割补偿器 ${ \sec \theta } _ { f }$ 来提高系统的稳定性。

## 6.2.3 水平式双轴稳定平台误差分析

## 1.沿陀螺输出轴的干扰力矩引起的平台误差

为简化分析，忽略图 6.2.5 中①和 $\textcircled{1}$ 处的交叉搞合，由图得仅由 $M _ { \mathrm { d 1 } }$ $M _ { \mathrm { d 2 } }$ 2引起的平台偏差角为

$$
\begin{array}{l} \varphi_ {x} ^ {1} (s) = M _ {\mathrm{d} 1} (s) \frac {H _ {2} s + k _ {f} F _ {f} (s)}{s (I _ {O 1} s + D _ {1})} \left(- \frac {1}{s ^ {2} J _ {X}}\right) \\ \quad + \varphi_ {x} ^ {1} (s) \frac {(H _ {1} s + k _ {\varphi_ {x}}) [ H _ {2} s + k _ {f} F _ {f} (s) ]}{s (I _ {O 1} s + D _ {1})} \left(- \frac {1}{s ^ {2} J _ {X}}\right) + \varphi_ {y} ^ {1} (s) \left[ - \frac {H _ {2} s + k _ {\varphi_ {y}}}{s ^ {2} J _ {X}} \right] \\ \varphi_ {y} ^ {1} (s) = M _ {\mathrm{d} 2} (s) \frac {H _ {1} s \cos \theta_ {f} + k _ {r} F _ {r} (s)}{s (I _ {O 2} s + D _ {2})} \left(- \frac {\cos \theta_ {f}}{s ^ {2} J _ {\Sigma y}}\right) + \varphi_ {x} ^ {1} (s) (H _ {1} s + k _ {\varphi_ {x}}) \cos \theta_ {f} \left(- \frac {\cos \theta_ {f}}{s ^ {2} J _ {\Sigma y}}\right) \\ \quad + \varphi_ {y} ^ {1} (s) \frac {(H _ {2} s + k _ {\varphi_ {y}}) [ H _ {1} s \cos \theta_ {f} + k _ {r} F _ {r} (s) ]}{s (I _ {O 2} s + D _ {2})} \left(- \frac {\cos \theta_ {f}}{s ^ {2} J _ {\Sigma y}}\right) \\ \varphi_ {x} ^ {1} (s) \left\{s ^ {3} J _ {X} (I _ {O 1} s + D _ {1}) + (H _ {1} s + k _ {\varphi_ {x}}) [ H _ {2} s + k _ {f} F _ {f} (s) ] \right\} + \varphi_ {y} ^ {1} (s) s (I _ {O 1} s + D _ {1}) (H _ {2} s + k _ {\varphi_ {y}}) \\ = - M _ {\mathrm{d} 1} (s) [ H _ {2} s + k _ {f} F _ {f} (s) ] \\ \varphi_ {x} ^ {1} (s) s (I _ {O 2} s + D _ {2}) (H _ {1} s + k _ {\varphi_ {x}}) \cos^ {2} \theta_ {f} + \varphi_ {y} ^ {1} (s) \left\{s ^ {3} J _ {\Sigma y} (I _ {O 2} s + D _ {2}) \right. \end{array}
$$

即

$$
\begin{array}{r l} & + (H _ {2} s + k _ {\varphi_ {y}}) \big [ H _ {1} s \mathrm{cos} \theta_ {f} + k _ {r} F _ {r} (s) \big ] \} \mathrm{cos} \theta_ {f} \\ = & - M _ {\mathrm{d} 2} (s) \big [ H _ {1} s \mathrm{cos} \theta_ {f} + k _ {r} F _ {r} (s) \big ] \mathrm{cos} \theta_ {f} \end{array}
$$

将上述两式写成矩阵形式:

$$
\left[ \begin{array}{l l} A _ {1 1} (s) & A _ {1 2} (s) \\ A _ {2 1} (s) & A _ {2 2} (s) \end{array} \right] \left[ \begin{array}{l} \varphi_ {x} ^ {1} (s) \\ \varphi_ {y} ^ {1} (s) \end{array} \right] = \left[ \begin{array}{l} - \left[ H _ {2} s + k _ {f} F _ {f} (s) \right] M _ {\mathrm{d} 1} (s) \\ - \left[ H _ {1} s \cos \theta_ {f} + k _ {r} F _ {r} (s) \right] M _ {\mathrm{d} 2} (s) \end{array} \right]
$$

式中

$$
A _ {1 1} (s) = s ^ {3} J _ {X} \left(I _ {0 1} s + D _ {1}\right) + \left(H _ {1} s + k _ {\varphi_ {x}}\right) \left[ H _ {2} s + k _ {f} F _ {f} (s) \right]\tag{6.2.15a}
$$

$$
A _ {1 2} (s) = s \left(I _ {O 1} s + D _ {1}\right) \left(H _ {2} s + k _ {\varphi_ {y}}\right)\tag{6.2.15b}
$$

$$
A _ {2 1} (s) = s \left(I _ {O 2} s + D _ {2}\right) \left(H _ {1} s + k _ {\varphi_ {x}}\right) \cos \theta_ {f}\tag{6.2.15c}
$$

$$
A _ {2 2} (s) = s ^ {3} J _ {\Sigma y} \left(I _ {O 2} s + D _ {2}\right) + \left(H _ {2} s + k _ {\varphi y}\right) \left[ H _ {1} s \cos \theta_ {f} + k _ {r} F _ {r} (s) \right]\tag{6.2.15d}
$$

系统的特征多项式为

$$
\Delta (s) = \left| \begin{array}{l l} A _ {1 1} (s) & A _ {1 2} (s) \\ A _ {2 1} (s) & A _ {2 2} (s) \end{array} \right| = A _ {1 1} (s) A _ {2 2} (s) - A _ {1 2} (s) A _ {2 1} (s)\tag{6.2.16}
$$

$$
\left[ \begin{array}{c} \varphi_ {x} ^ {1} (s) \\ \varphi_ {y} ^ {1} (s) \end{array} \right] = \frac {1}{\Delta (s)} \left[ \begin{array}{c c} A _ {2 2} (s) & - A _ {1 2} (s) \\ - A _ {2 1} (s) & A _ {1 1} (s) \end{array} \right] \left[ \begin{array}{c} - [ H _ {2} s + k _ {f} F _ {f} (s) ] M _ {\mathrm{d} 1} (s) \\ - [ H _ {1} s \cos \theta_ {f} + k _ {r} F _ {r} (s) ] M _ {\mathrm{d} 2} (s) \end{array} \right]
$$

$$
\begin{array}{r l} \varphi_ {x} ^ {1} (s) & = \frac {1}{\Delta (s)} \{- A _ {2 2} (s) [ H _ {2} s + k _ {f} F _ {f} (s) ] M _ {\mathrm{d} 1} (s) \\ & + A _ {1 2} (s) [ H _ {1} s \cos \theta_ {f} + k _ {r} F _ {r} (s) ] M _ {\mathrm{d} 2} (s) \} \end{array}\tag{6.2.17a}
$$

$$
\begin{array}{r l} \varphi_ {y} ^ {1} (s) & = \frac {1}{\Delta (s)} \left\{A _ {2 1} (s) \left[ H _ {2} s + k _ {f} F _ {f} (s) \right] M _ {\mathrm{d} 1} (s) \right. \\ & \left. - A _ {1 1} (s) \left[ H _ {1} s \cos \theta_ {f} + k _ {r} F _ {r} (s) \right] M _ {\mathrm{d} 2} (s) \right\} \end{array}\tag{6.2.17b}
$$

由式 (6.2.15) 和式 (6.2.16)

$$
\begin{array}{c}\lim _ {s \rightarrow 0} A _ {1 1} = k _ {\varphi_ {x}} k _ {f} F _ {f} (0), \lim _ {s \rightarrow 0} A _ {2 2} = k _ {\varphi_ {y}} k _ {r} F _ {r} (0), \lim _ {s \rightarrow 0} A _ {1 2} = 0, \quad \lim _ {s \rightarrow 0} A _ {2 1} = 0,\\\lim _ {s \rightarrow 0} \Delta (s) = k _ {\varphi_ {x}} k _ {\varphi_ {y}} k _ {f} k _ {r} F _ {f} (0) F _ {r} (0)\end{array}
$$

所以当陀螺绕输出轴的干扰力矩均为常值时，即

$$
M _ {\mathrm{d} 1} (s) = \frac {M _ {\mathrm{d} 1}}{s}, \quad M _ {\mathrm{d} 2} (s) = \frac {M _ {\mathrm{d} 2}}{s}
$$

由式(6.2.17) ，平台水平偏差角的稳态值为

$$
\begin{array}{l}\varphi_ {x s s} ^ {1} = \lim _ {s \rightarrow 0} \varphi_ {x} ^ {1} (s) = \frac {- k _ {\varphi_ {y}} k _ {f} k _ {r} F _ {f} (0) F _ {r} (0) M _ {\mathrm{d} 1}}{k _ {\varphi_ {x}} k _ {\varphi_ {y}} k _ {f} k _ {r} F _ {f} (0) F _ {r} (0)} = - \frac {M _ {\mathrm{d} 1}}{k _ {\varphi_ {x}}}\\\varphi_ {y s s} ^ {1} = \lim _ {s \rightarrow 0} \varphi_ {y} ^ {1} (s) = \frac {- k _ {\varphi_ {x}} k _ {f} k _ {r} F _ {f} (0) F _ {r} (0) M _ {\mathrm{d} 2}}{k _ {\varphi_ {x}} k _ {\varphi_ {y}} k _ {f} k _ {r} F _ {f} (0) F _ {r} (0)} = - \frac {M _ {\mathrm{d} 2}}{k _ {\varphi_ {y}}}\end{array}\tag{6.2.18a}
$$

(6.2.18b>

根据 2.3.3 节中关于单自由度陀螺对陀螺漂移传递函数的介绍 $M _ { \mathrm { d l } }$ 和 $M _ { \mathrm { d 2 } }$ 实际对应着陀螺漂移，所以式(6.2.18)实际上反映了陀螺漂移对平台偏差角的影响。

2. 沿平台稳定轴的干扰力矩引起的平台误差

忽略图 6.2.5 中①和 $1 \textcircled{1}$ 处的交叉藕合，由图得仅由 $M _ { \mathrm { d } f }$ 和 $M _ { \mathrm { d } r }$ 引起的平台偏差角为

$$
\varphi_ {x} ^ {2} (s) = - M _ {\mathrm{d} f} (s) \left(- \frac {1}{s ^ {2} J _ {X}}\right) + \varphi_ {x} ^ {2} (s) \frac {(H _ {1} s + k _ {\varphi_ {x}}) [ H _ {2} s + k _ {f} F _ {f} (s) ]}{s (I _ {O 1} s + D _ {1})}
$$

$$
\begin{array}{l} \times \left(- \frac {1}{s ^ {2} J _ {X}}\right) + \varphi_ {y} ^ {2} (s) \left[ - \frac {H _ {2} s + k _ {\varphi_ {y}}}{s ^ {2} J _ {X}} \right] \\ \varphi_ {y} ^ {2} (s) = M _ {\mathrm{dr}} (s) \left(- \frac {\cos \theta_ {f}}{s ^ {2} J _ {\Sigma y}}\right) + \varphi_ {y} ^ {2} (s) \frac {(H _ {2} s + k _ {\varphi_ {y}}) [ H _ {1} s \cos \theta_ {f} + k _ {r} F _ {r} (s) ]}{s (I _ {O 2} s + D _ {2})} \\ \times \left(- \frac {\cos \theta_ {f}}{s ^ {2} J _ {\Sigma y}}\right) + \varphi_ {x} ^ {2} (s) (H _ {1} s + k _ {\varphi_ {x}}) \cos \theta_ {f} \left(- \frac {\cos \theta_ {f}}{s ^ {2} J _ {\Sigma y}}\right) \\ \varphi_ {x} ^ {2} (s) \left\{s ^ {3} J _ {X} (I _ {O 1} s + D _ {1}) + (H _ {1} s + k _ {\varphi_ {x}}) [ H _ {2} s + k _ {f} F _ {f} (s) ] \right\} + \varphi_ {y} ^ {2} (s) s (I _ {O 1} s + D _ {1}) (H _ {2} s + k _ {\varphi_ {y}}) \\ = M _ {\mathrm{df}} (s) s (I _ {O 1} s + D _ {1}) \\ \varphi_ {x} ^ {2} (s) s (I _ {O 2} s + D _ {2}) (H _ {1} s + k _ {\varphi_ {x}}) \cos^ {2} \theta_ {f} + \varphi_ {y} ^ {2} (s) \left\{s ^ {3} J _ {\Sigma y} (I _ {O 2} s + D _ {2}) \right. \\ + (H _ {2} s + k _ {\varphi_ {y}}) [ H _ {1} s \cos \theta_ {f} + k _ {r} F _ {r} (s) ] \} \cos \theta_ {f} \\ = - M _ {\mathrm{dr}} (s) s (I _ {O 2} s + D _ {2}) \cos \theta_ {f} \end{array}
$$

将上述两式写成矩阵形式

$$
\left[ \begin{array}{l l} A _ {1 1} (s) & A _ {1 2} (s) \\ A _ {2 1} (s) & A _ {2 2} (s) \end{array} \right] \left[ \begin{array}{l} \varphi_ {x} ^ {2} (s) \\ \varphi_ {y} ^ {2} (s) \end{array} \right] = \left[ \begin{array}{l} s \left(I _ {O 1} s + D _ {1}\right) M _ {\mathrm{d} f} (s) \\ - s \left(I _ {O 2} s + D _ {2}\right) M _ {\mathrm{d} r} (s) \end{array} \right]
$$

式中 $A _ { 1 1 } \left( s \right) \mathrm { , } A _ { 1 2 } \left( s \right) \mathrm { , } A _ { 2 1 } \left( s \right) \mathrm { , } A _ { 2 2 } \left( s \right)$ 与式 (6.2.15)相同。

$$
\begin{array}{l} \left[ \begin{array}{c} \varphi_ {x} ^ {2} (s) \\ \varphi_ {y} ^ {2} (s) \end{array} \right] = \frac {1}{\Delta (s)} \left[ \begin{array}{c c} A _ {2 2} (s) & - A _ {1 2} (s) \\ - A _ {2 1} (s) & A _ {1 1} (s) \end{array} \right] \left[ \begin{array}{c} s (I _ {O 1} s + D _ {1}) M _ {\mathrm{d} f} (s) \\ - s (I _ {O 2} s + D _ {2}) M _ {\mathrm{d} r} (s) \end{array} \right] \\ \varphi_ {x} ^ {2} (s) = \frac {1}{\Delta (s)} \left\{A _ {2 2} (s) M _ {\mathrm{d} f} (s) s (I _ {O 1} s + D _ {1}) + A _ {1 2} (s) M _ {\mathrm{d} r} (s) s (I _ {O 2} s + D _ {2}) \right\} \\ \varphi_ {y} ^ {2} (s) = \frac {1}{\Delta (s)} \left\{- A _ {2 1} (s) M _ {\mathrm{d} f} (s) s (I _ {O 1} s + D _ {1}) - A _ {2 2} (s) M _ {\mathrm{d} r} (s) s (I _ {O 2} s + D _ {2}) \right\} \end{array}\tag{6.2.19a}
$$

(6.2. 19b)

式中 ，.ð(s) 与式 (6.2.16)相同。

当绕平台稳定轴的干扰力矩均为常值时，即

$$
M _ {\mathrm{df}} (s) = \frac {M _ {\mathrm{df}}}{s}, \quad M _ {\mathrm{dr}} (s) = \frac {M _ {\mathrm{dr}}}{s}
$$

由式(6.2.19) ，平台水平偏差角的稳态值为

$$
\varphi_ {x s s} ^ {2} = \lim _ {s \rightarrow 0} \varphi_ {x} ^ {2} (s) = 0, \quad \varphi_ {y s s} ^ {2} = \lim _ {s \rightarrow 0} \varphi_ {y} ^ {2} (s) = 0
$$

上两式说明沿平台稳定轴的常值干扰力矩不会引起平台偏差角，这实质上是平台的稳定回路起卸荷作用的结果。

## 3. 线加速度误差和盘旋误差

加减速和转弯是飞机、舰船等运载体的基本运动状态，与之相关联的水平过载使液体开关的液面倾斜，即液面跟踪视在垂线，通过修正回路对平台做错误修正，产生水平偏差角，水平过载越大，错误修正越严重，偏差角也越大。为了减小这种误差，当运载体做纵向加速或减速时，可断开平台的纵向修正回路;当运载体转弯(飞机做盘旋〉时，可断开平台的横向修正回路，使平台暂时工作在几何稳定状态，一旦机动运动结束，就恢复修正回路工作。可见水平式双轴稳定平台与直读式陀螺地平仪的工作原理是相仿的，不同之处在于此类陀螺地平仪不具备对于扰力矩的卸荷能力.

## 习题一

2-1 简述如下内容:

(1)陀螺稳定装置的分类及各自的特点;

(2) 陀螺稳定装置的两个基本功能及各自的作用;

(3) 制导系统的组成和作用;

(4) 舰船减摇装置的工作原理

2-2 稳定平台使用时间常数很大的挠性陀螺作为敏感元件，忽略水平通道间的搞合，则沿 个轴的控制回路如图所示，其中 $\omega$ 为平台沿稳定铀的输出角速率 $\boldsymbol { \mathbf { \ell } } , k _ { p }$ 陀螺信号器的传递系数， $, k _ { a }$ 为平台电子的综合放大系数， $, k _ { m }$ 分别为力矩 达的转矩系数和总阻抗 $F ( s )$ 为校正网络的传递函数

(1)常值陀螺漂移引起的平台漂移;

(2)在常值于扰力矩 $M _ { \mathrm { d } }$ 作用下的平台漂移、偏差角;

(3)平台的力矩刚度。

![](images/5f6e4508768d69e98369dad6c23a55bfff96ea9c61b5c7471bbe9d4e06bd8b4f.jpg)  
稳定平台沿一个轴的控制回路

2-3 用积分陀螺单轴稳定平台稳定船载天线 稳定轴沿船舶纵轴设置，并由力矩马达直接驱动。由于风影响，船舶绕纵轴 横摇 $\smash { : \theta _ { b } \left( t \right) = \theta _ { b m } \sin \Omega t }$ 。横摇引起轴承的干 力矩为

$$
M _ {i} (t) = \left\{ \begin{array}{c c} - M _ {i m} & - \pi \leqslant \Omega t \leqslant 0 \\ M _ {i p} & 0 \leqslant \Omega t \leqslant \pi \end{array} \right. \quad (M _ {i m} > M _ {i p})
$$

忽略平台偏心。

(1)求摩擦力矩的直流分量和由该直流分量引起的平台稳态漂移和稳态偏差角;

(2) 求平台对横摇的隔离度。

2-4 积分陀螺单轴稳定平台用减速比为 的一级齿轮传动拖动台体，电机轴 轮和电机电枢的转动惯量为$J _ { o }$ ，平台轴齿轮和台体的转动转动惯量为 $J _ { \rho }$ 电机时间常数 力矩系数 电枢电阻分别为 $\tau _ { e } \ : , k _ { m }$ 陀螺时间数为 $\tau _ { g }$ ，传递系数为 $k _ { g }$ 陀螺信号器传递系数为 $k _ { u }$ ，平台电子放大系数为 $k _ { a }$ 正网络传递函数为 $F ( s )$ 基座角速度为 $\omega _ { i b }$

(1)写出平台的动力学方程式;

(2) 画出平台系统方块图;

(3) 求陀螺常值漂移引起的平台漂移;

(4) 台的力矩刚度;

(5) 求在常值干扰力矩 $M _ { \mathrm { d } }$ 作用下的平台漂移、平台偏差角。

2-5 下因为使用角速 陀螺作为敏感元件的单轴稳定平台，其中 $\beta$ 为陀螺输出角 $\ l , \phi$ 为平台输出角 设平台绕输出轴的转动惯量为 $\boldsymbol { J } _ { \boldsymbol { \phi } }$ 陀螺组件绕输出轴的转动惯量为]，陀螺信号器的变换系数为 $k _ { \beta }$ ，放大器放大系数为 $k _ { a }$ 力矩马达力矩系数为 $k _ { m }$ ，弹簧的弹性系数为 阻尼器阻尼系数为 ，假设 $J _ { \phi } \gg J$ 陀螺频带远比稳定回路的频带宽。

(1)列写 该稳定平台的动力学方程;

(2) 稳定轴上有常值干扰力矩 $M _ { \mathrm { d } x }$ 时的 台漂移方程;

(3) 如果 $M _ { \mathrm { d y } } = 0$ ，计算由常值 扰力矩 $M _ { \mathrm { d } x }$ 引起的平台稳态漂移。 要消除此漂移，应 取何措施?分

此时的平台稳定误差。

![](images/df23dd77a8f7173d9f54c9f784d0d814d43538b96c771f9cb37710f4182d4cc2.jpg)  
2-5 角速率陀螺作为敏 器件的 轴稳定平台

2-6 下图为 基座条件下的陀螺罗经。设罗经所在地纬度为 罗经 动量为 ，不平衡质量块的质量为$m ,$ 量块质心距陀螺支承中心的距离为 加在外框架轴上的控制力矩即阻尼力矩为 $f \beta _ { \mathfrak { c } }$ 。图中，下标 $g$ 表示地理坐标系，下标 陀螺壳体坐标系， 和 $\beta$ 均为小角

(1 证明:当 $f > 0$ 时，即有阻尼时，罗经为 系统;求出罗经收敛所达到的方位精度和水平精度。

(2) 证明: $f { = } 0$ ，即无阻尼时，罗经作无阻尼振 ;求出元阻尼自振频率。

![](images/2521f939b0cfff1db21730448c0a5e90b86b1ea1f83cf6ca9e36f9922915d11c.jpg)  
静基座条件下的陀螺罗经

## 第三篇惯性导航系统原理

本篇详细介绍惯性导航系统基本方程 平台式惯导系统的力学编排、误差分析及初始对准捷联式惯导系统的姿态更新、速度更新及位置更新算法，这些算法以陀螺和加速度计输出角增量和速度增量为前提，考虑了对有限转动不可交换误差的补偿，即对圆锥误差、划桨误差及涡卷误差的精确补偿，适用于高动态环境条件下的系统设计;捷联惯导的自主式初始对准及动基座传递对准 极区惯性导航。

## 惯导系统中的基本关系

## 7. 1 惯导系统的分类

惯性导航系统是以陀螺和加速度计为敏感器件的导航参数解算系统，该系统根据陀螺的输出建立起导航坐标系，根据加速度计输出解算出运载体的速度和位置。图 7. 1. 表示加速度计安装在运载体上的示意图，图中质量 受弹簧约束，悬挂弹簧的加速度计壳体固联在运载体上。当运载体以加速度 做水平运动时，弹簧质量系统达到平衡后，质量 受到的水平约束力加速度 的关系满足牛顿第二定律:

$$
\boldsymbol {a} = \frac {\boldsymbol {F}}{m}
$$

![](images/d56febca704326af47ff95b04c4bc939bf66dbf5f00b84c57678e0e82f7fab76.jpg)  
7. 1. 运载体上的加速度计示意图

只要测量得水平弹性约束力 ，就可求得水平加速度 ，对 积分一次即可得水平速度，再积分一次即可得水平位移。但这是简单化了的理想情况，由于运载体不可能只做水平运动，且还有姿态变化，所以用于导航解算的加速度信息必须是导航坐标系内的数学向量。可通过两种途径实现这一要求。一种途径是将加速度计安装在稳定平台上，稳、定平台由陀螺控制，使平台始终跟踪要求的导航坐标系;另一种途径是将加速度计和陀螺、都直接安装在运载体上，陀螺输出用来解算运载体相对导航坐标系的姿态变换矩阵，加速度计输出经姿态阵变换至导航坐标系内，这相当于建立起了数学平台。

根据构建导航坐标系方法的不同可将惯导系统分为两大类型:采用物理平台模拟导航坐标系的系统称为平台式惯导系统;采用数学算法确定出导航坐标系的系统称为捷联式惯导系统。根据物理平台模拟的坐标系类型不同，平台式惯导系统又可分为两类:若物理平台模拟惯性坐标系，则系统称为解析式惯导系统;若物理平台模拟当地水平坐标系，则系统称为当地水平式惯导系统。根据平台跟踪地球自转角速度和跟踪水平坐标系类型的不同，当地水平式惯导系统又可分为三种:若物理平台跟踪地理坐标系(必然要跟踪地球自转角速度) ，则系统称为指北方位惯导系统;若平台跟踪地球自转角速度并跟踪当地水平面，则系统称为游移方位惯导系统;若平台只跟踪地球自转角速度的水平分量，并跟踪当地水平面，则系统称为自由方位惯导系统。

## 7.2 地球的描述

导航系统的任务是要确定运载体的位置、航向及姿态，而要确定这些量必须要有参照体。星际航行以宇宙空间的天体作为参照体 在地球上作导航则以地球作为参照体。本书所讨论的导航都以地球作为参照体，所以有必要介绍地球的几何形状和力学特性，

## 7.2.1 地球的几何形状

地球是一个不规则球体，内部是熔岩，由于自转的影响，地球呈扁圆状，沿赤道方向鼓出，南极稍微凹人，形状似梨 $\mathrm { ( O ^ { \prime } K e e f e \ J \ e t \ a l . \ , 1 9 5 9 ) }$ 。常采用三种几何模型对地球作近似描述

(1)大地水准体=通过全球海平面的地球重力场等势面围成的空间体。

(2) 圆球 球心位于地心，半径 $R { = } 6 3 7 1 \mathrm { k m }$ o

(3) 参考旋转椭球体 中心位于地心，分别以 $R _ { e }$ ，和 $R _ { \mathit { \phi } }$ 为半 主轴和半短轴的椭圆绕地球自转轴旋转 $1 8 0 ^ { o }$ 所形成的椭球体，其中 $R _ { e }$ 和 $R _ { \rho }$ 通过大地测量确定，如表 7.2.1 所示。

7.2.1 常用参考旋转椭球体参数

<table><tr><td>名称</td><td>半长轴  $R_e$ /m</td><td>半短轴  $R_p$ /m</td><td>扁率  $f=\frac{R_e-R_p}{R_e}$ </td><td>适用地区</td></tr><tr><td>克拉索夫斯基</td><td>6378254</td><td>6356803</td><td> $\frac{1}{298.3}$ </td><td>俄、中</td></tr><tr><td>海福德</td><td>6378389</td><td>6356912</td><td> $\frac{1}{297.0}$ </td><td>西欧</td></tr><tr><td>1975年国际推荐</td><td>6378140</td><td>6356755</td><td> $\frac{1}{298.257}$ </td><td></td></tr><tr><td>克拉克</td><td>6378206</td><td>6356584</td><td> $\frac{1}{295.0}$ </td><td>美</td></tr><tr><td>WGS-84</td><td>6378137</td><td>6356752</td><td> $\frac{1}{298.257}$ </td><td>GPS</td></tr></table>

由于地球表面大部分是海洋，所以三种模型中，大地水准体对地球的近似度最好，而旋转椭球体与大地水准体非常接近，在垂直方向的最大误差约为 150m，垂线偏离真垂线(大地水准面的法线)最大误差为 $3 ^ { \prime \prime }$ ，因此惯导系统中常用旋转椭球作为地球的模型描述。圆球偏离大地水准体的误差最大，一般在近似分析中采用。

## 7.2.2 地垂线和纬度

如图 7.2.1，设 为地球表面某一点，过 点作大地水准面、旋转椭球面及圆球面的法线，依次得到的交点为 $P _ { 1 } , P _ { 0 }$ 。和 $P _ { 2 }$ ，则定义

大地水准面的法线 $P P _ { 1 }$ 为 $P$ 点的天文垂线，也称真垂线;

旋转椭球面的法线 $P P _ { 0 }$ 为 点的地理垂线

圆球的法线 $P P _ { 2 }$ 为 $P$ 点的地心垂线

天文垂线与赤道平面的夹角 $L _ { g }$ 为 $P$ 点的天文纬度

地理垂线与赤道平面的夹角 点的地理纬度，常简称纬度

地心垂线与赤道平面的夹角 $L _ { c }$ ，为 $P$ 点的地心纬度。

$P P _ { 0 }$ 为飞机在 点处的飞行高度

$P P _ { 1 }$ 为飞机在 点处的海拔高度;

$P P _ { 3 }$ 为飞机在 点处的相对高度;

其中 $P _ { 3 }$ 点处的地理垂线与真实地形的交点。

![](images/53f5a3809e4886ca5dcc85a12b9d39b38d73c6294c930f9db8459a8b17a8ec49.jpg)  
7.2.1 地垂线和纬度

地理纬度和地心纬度对应着不同的垂线定义，两者的差异实质上反映了地理垂线和地心垂线间的偏差。

为旋转椭球面上的一点，记该点的地理纬度和地心纬度分别为 $L _ { c }$ 。图 7.2.2 为过点和对称轴的旋转椭球的剖面图。

点的坐标为( $( x , z )$ ，则坐标满足椭圆方程:

![](images/c01e3351b75588e2b859f1ced5ae49eb83f253af05c0e9d6063ef9b9780be20e.jpg)  
7.2.2 点在椭圆上的坐标

$$
\frac {x ^ {2}}{R _ {e} ^ {2}} + \frac {z ^ {2}}{R _ {p} ^ {2}} = 1
$$

上式两边对 求导，得

即

$$
\frac {2 x}{R _ {e} ^ {2}} \frac {\mathrm{d} x}{\mathrm{d} z} + \frac {2 z}{R _ {p} ^ {2}} = 0
$$

$$
\frac {\mathrm{d} x}{\mathrm{d} z} = - \frac {R _ {e} ^ {2}}{R _ {p} ^ {2}} \frac {z}{x}
$$

因为 点切线的斜率为 $\frac { \mathrm { d } z } { \mathrm { d } x }$ 点法线的斜率为$\tan L$ L，两者互为负倒数，所以

$$
\tan L = - \frac {1}{\frac {\mathrm{d} z}{\mathrm{d} x}} = - \frac {\mathrm{d} x}{\mathrm{d} z} = \frac {R _ {e} ^ {2}}{R _ {p} ^ {2}} \frac {z}{x}
$$

而 $\mathrm { t a n } L _ { c } = \frac { z } { x }$ ，令 $\Delta L { = } L { - } L _ { c }$ 所以

$$
\begin{array}{r l} \tan \Delta L & = \tan (L - L _ {c}) = \frac {\tan L - \tan L _ {c}}{1 + \tan L \tan L _ {c}} \\ & = \frac {\frac {R _ {e} ^ {2}}{R _ {p} ^ {2}} \frac {z}{x} - \frac {z}{x}}{1 + \frac {R _ {e} ^ {2}}{R _ {p} ^ {2}} \frac {z}{x} \cdot \frac {z}{x}} = \frac {(R _ {e} ^ {2} - R _ {p} ^ {2}) x z}{R _ {e} ^ {2} R _ {p} ^ {2} \left(\frac {x ^ {2}}{R _ {e} ^ {2}} + \frac {z ^ {2}}{R _ {p} ^ {2}}\right)} = \frac {(R _ {e} ^ {2} - R _ {p} ^ {2}) x z}{R _ {e} ^ {2} R _ {p} ^ {2}} \end{array}
$$

而 $x { = } R \mathrm { c o s } L _ { c } , z { = } R \mathrm { s i n } L _ { c }$ 所以

$$
\tan \Delta L = \frac {R _ {e} ^ {2} - R _ {p} ^ {2}}{2 R _ {e} ^ {2} R _ {p} ^ {2}} R ^ {2} \sin 2 L _ {c} = \frac {R _ {e} - R _ {p}}{R _ {e}} \cdot \frac {R _ {e} + R _ {p}}{2 R _ {p}} \cdot \frac {R ^ {2}}{R _ {e} R _ {p}} \sin 2 L _ {c}
$$

由于 $\therefore \frac { R _ { e } - R _ { \phi } } { R _ { e } } = f , \frac { R _ { e } + R _ { \phi } } { 2 R _ { \phi } } { \approx } 1 , \frac { R ^ { 2 } } { R _ { e } R _ { \phi } } { \approx } 1 , L _ { c } { \approx } L , { \Delta } L$ 是小量，所以

$$
\tan \Delta L \approx \Delta L \approx f \sin 2 L\tag{7.2.1}
$$

可见地理垂线和地心垂线的最大偏差发生在纬度为 $4 5 ^ { \circ }$ 的地方，约为 $1 1 ^ { \prime }$ ，而地理垂线与真垂线的最大偏差角约为 $3 ^ { \prime \prime }$ ，所以用地理垂线近似真垂线有足够高的精度。

按照航海界的规定，若同一子午圈上两点的纬度差 $1 ^ { \prime }$ ，则两点间的距离为 海里(nauticalmile，单位符号口 mile) ，将地球近似视为圆球，则

$$
\mathrm{1nmile} = \frac {1}{6 0} \times \frac {\pi}{1 8 0} \times 6 3 7 1 0 0 0 = 1 8 5 3. 2 (\mathrm{m}) \approx 1. 8 5 (\mathrm{km})
$$

因此由式(7.2. 1)知，同一地点若分别用地理纬度和地心纬度来表示其位置，则引起南北方向的最大偏差为l1 mile，约为 20km

## 7.2.3 参考旋转椭球体的曲率半径

## 1.主曲率半径

圆球面上任意点沿任何方向的曲率半径都是相等的，但旋转椭球面上同一点沿不同方向的曲率半径是不相等的。

7.2.3 为地球的椭球模型。设 点为旋转椭球面上的某一点 点处的法线 NS椭球面的对称轴。过 NS 的垂直平面，截椭球面所得的平面曲线 lPl 称为 点处的纬圈;

![](images/65fc718a57b755193e59bcd74d053562b076f15bcf113188cc332413d6dd370e.jpg)  
7.2.3 旋转椭球面上的主曲率半径

点和直线 NS 作平面截椭球面所得的平面曲线 $m P m$ 称为 $P$ 点处的经圈(子午圈) ;过 $P$ 点作纬圈 IPI 的切线 $t P t$ ,用 $t P t$ 和法线 形成的平面截椭球面所得的平面曲线 $r P r$ 称为 点处的卵西圈。 点处沿子午圈 $m P m$ 的曲率半径 $R _ { M }$ 和沿卵西圈 $r P r$ 的曲率半径 $R _ { N }$ 称为旋转椭球面在 点处的主曲率半径。

## 2. 主曲率半径的求取

为地球上一点 点的地心纬度和地理纬度分别为 $L _ { c }$ 。图 7.2.4 为过 点和对称轴的旋转椭球的剖面。椭圆参数方程为

![](images/2d0f1dc8a2010ee120d7589d26267d8272182d56260d342444d107eb85f4db34.jpg)  
7.2.4 点和对称轴的旋转椭球剖面

$$
\begin{array}{l} x = R _ {e} \cos t \\ z = R _ {p} \sin t \end{array}
$$

式中 为椭圆的离心角。由于离心角物理意义不明显，地球扁率约为千分之三，接近圆球，离心角与地心纬度 $L _ { c }$ 的差异在 $\mid L _ { c } \mid = 4 4 . 9 5 2 ^ { \circ }$ 处达到最大值 $5 . 7 6 ^ { \prime }$ ，在赤道和南、北极为 ，越接近赤道和两极，差异越小。所以为简化分析，此处离心角近似用地心纬度替代，椭圆参数方程取

$$
\begin{array}{l} x = R _ {e} \cos L _ {c} \\ z = R _ {p} \sin L _ {c} = R _ {e} (1 - f) \sin L _ {c} \end{array}
$$

上两式对 $L _ { c }$ 求导，得

$$
\begin{array}{r l} & \frac {\mathrm{d} x}{\mathrm{d} L _ {c}} = - R _ {e} \sin L _ {c} \\ & \frac {\mathrm{d} z}{\mathrm{d} L _ {c}} = R _ {e} (1 - f) \cos L _ {c} \end{array}
$$

由于法线 的斜率与 $P$ 点切线的斜率互为负倒数，所以有

$$
\tan L = - \frac {1}{\frac {\mathrm{d} z}{\mathrm{d} x}} = - \frac {\mathrm{d} x}{\mathrm{d} z} = - \frac {\frac {\mathrm{d} x}{\mathrm{d} L _ {c}}}{\frac {\mathrm{d} z}{\mathrm{d} L _ {c}}} = \frac {1}{1 - f} \tan L _ {c}\tag{7.2.2}
$$

弧微分 ds dx dz 的关系为

$$
\begin{array}{r l} \mathrm{d} s ^ {2} & = \mathrm{d} x ^ {2} + \mathrm{d} z ^ {2} = (- R _ {e} \sin L _ {c} \mathrm{d} L _ {c}) ^ {2} + [ R _ {e} (1 - f) \cos L _ {c} \mathrm{d} L _ {c} ] ^ {2} \\ & = R _ {e} ^ {2} [ \sin^ {2} L _ {c} + (1 - f) ^ {2} \cos^ {2} L _ {c} ] \mathrm{d} L _ {c} ^ {2} \end{array}\tag{7.2.3}
$$

根据兰角关系式和式(7. 2. 2)

$$
\sin^ {2} L _ {c} = \frac {\tan^ {2} L _ {c}}{1 + \tan^ {2} L _ {c}} = \frac {(1 - f) ^ {2} \tan^ {2} L}{1 + (1 - f) ^ {2} \tan^ {2} L}\tag{7.2.4a}
$$

$$
\cos^ {2} L _ {c} = \frac {1}{1 + \tan^ {2} L _ {c}} = \frac {1}{1 + (1 - f) ^ {2} \tan^ {2} L}\tag{7.2.4b}
$$

对式 (7.2.2) 求微分:

$$
\begin{array}{r l} & \frac {1}{\cos^ {2} L} \mathrm{d} L = \frac {1}{1 - f} \cdot \frac {1}{\cos^ {2} L _ {c}} \mathrm{d} L _ {c} \\ & \mathrm{d} L _ {c} = \frac {(1 - f) \cos^ {2} L _ {c}}{\cos^ {2} L} \mathrm{d} L = \frac {1 - f}{1 + (1 - f) ^ {2} \tan^ {2} L} \cdot \frac {1}{\cos^ {2} L} \mathrm{d} L \end{array}\tag{7.2.4c}
$$

将式 (7.2.4) 代人式 (7.2.3) ，得

$$
\begin{array}{r l} \mathrm{d} s & = R _ {e} \left[ \frac {(1 - f) ^ {2} \tan^ {2} L}{1 + (1 - f) ^ {2} \tan^ {2} L} + (1 - f) ^ {2} \frac {1}{1 + (1 - f) ^ {2} \tan^ {2} L} \right] ^ {\frac {1}{2}} \\ & \times \frac {1 - f}{1 + (1 - f) ^ {2} \tan^ {2} L} \cdot \frac {1}{\cos^ {2} L} \mathrm{d} L \\ & = R _ {e} \sqrt {\frac {(1 - f) ^ {2} (1 + \tan^ {2} L) \cos^ {2} L}{[ 1 + (1 - f) ^ {2} \tan^ {2} L ] \cos^ {2} L}} \cdot \frac {1 - f}{1 + (1 - f) ^ {2} \tan^ {2} L} \cdot \frac {1}{\cos^ {2} L} \mathrm{d} L \\ & = \frac {R _ {e} (1 - f) ^ {2}}{\left[ \cos^ {2} L + (1 - f) ^ {2} \sin^ {2} L \right] ^ {\frac {3}{2}}} \mathrm{d} L \end{array}
$$

所以沿子午圈的曲率半径为

$$
R _ {M} = \frac {\mathrm{d} s}{\mathrm{d} L} = \frac {R _ {e} (1 - f) ^ {2}}{\left[ \cos^ {2} L + (1 - f) ^ {2} \sin^ {2} L \right] ^ {\frac {3}{2}}} = R _ {e} (1 - f) ^ {2} [ 1 - (2 - f) f \sin^ {2} L ] ^ {- \frac {3}{2}}\tag{7.2.5}
$$

参阅图 7.2.3 ，沿卵西圈的曲率半径为 $R _ { N } = P Q$ 。由图 7.2.4

$$
x = P Q \cos L = R _ {N} \cos L
$$

又由椭圆的参数方程之\~: $x { = } R _ { e } \mathrm { c o s } L _ { c }$ '

$$
R _ {N} = R _ {e} \frac {\cos L _ {c}}{\cos L}
$$

将式 (7.2.4b) 代人，得

$$
\begin{array}{r l} R _ {N} & = \frac {R _ {e}}{\sqrt {1 + (1 - f) ^ {2} \tan^ {2} L} \cdot \cos L} = \frac {R _ {e}}{\sqrt {\cos^ {2} L + (1 - f) ^ {2} \sin^ {2} L}} \\ & = R _ {e} [ 1 - (2 - f) f \sin^ {2} L ] ^ {- \frac {1}{2}} \end{array}\tag{7.2.6}
$$

采用麦克劳林级数展开上式，令 $u = ( 2 - f ) \ f \mathrm { s i n } ^ { 2 } L$ ，并记 $f _ { 1 } \left( u \right) = \left( 1 - u \right) ^ { - \frac { 3 } { 2 } }$ $f _ { 2 } \left( u \right) = \left( 1 - u \right) ^ { - \frac { 1 } { 2 } }$ ,则

$$
\begin{array}{l} f _ {1} ^ {\prime} (u) = \left(- \frac {3}{2}\right) (- 1) (1 - u) ^ {- \frac {5}{2}} = \frac {3}{2} (1 - u) ^ {- \frac {5}{2}} \\ f _ {1} ^ {\prime \prime} (u) = \frac {3}{2} \left(- \frac {5}{2}\right) (- 1) (1 - u) ^ {- \frac {7}{2}} = \frac {1 5}{4} (1 - u) ^ {- \frac {7}{2}} \\ \dots \\ f _ {2} ^ {\prime} (u) = \left(- \frac {1}{2}\right) (- 1) (1 - u) ^ {- \frac {3}{2}} = \frac {1}{2} (1 - u) ^ {- \frac {3}{2}} \\ f _ {2} ^ {\prime \prime} (u) = \frac {1}{2} \left(- \frac {3}{2}\right) (- 1) (1 - u) ^ {- \frac {5}{2}} = \frac {3}{4} (1 - u) ^ {- \frac {5}{2}} \\ \dots \\ f _ {1} (u) = f _ {1} (0) + \frac {1}{1 !} f _ {1} ^ {\prime} (0) u + \frac {1}{2 !} f _ {1} ^ {\prime \prime} (0) u ^ {2} + \dots = 1 + \frac {3}{2} u + \frac {1 5}{8} u ^ {2} + \dots \\ f _ {2} (u) = f _ {2} (0) + \frac {1}{1 !} f _ {2} ^ {\prime} (0) u + \frac {1}{2 !} f _ {2} ^ {\prime \prime} (0) u ^ {2} + \dots = 1 + \frac {1}{2} u + \frac {3}{8} u ^ {2} + \dots \end{array}
$$

所以

$$
\begin{array}{r l} R _ {M} & = R _ {e} (1 - f) ^ {2} \left[ 1 + \frac {3}{2} (2 - f) f \sin^ {2} L + \dots \right] \\ & = R _ {e} (1 - 2 f + f ^ {2}) \left(1 + 3 f \sin^ {2} L - \frac {3}{2} f ^ {2} \sin^ {2} L + \dots\right) \\ & = R _ {e} \left[ 1 + 3 f \sin^ {2} L - 2 f + f ^ {2} \left(1 - \frac {1 5}{2} \sin^ {2} L\right) + \dots \right] \\ R _ {N} & = R _ {e} \left[ 1 + \frac {1}{2} (2 - f) f \sin^ {2} L + \dots \right] = R _ {e} (1 + f \sin^ {2} L - \frac {1}{2} f ^ {2} \sin^ {2} L + \dots) \end{array}
$$

略去关于 的二阶以上微量，则

$$
R _ {M} = R _ {e} (1 - 2 f + 3 f \sin^ {2} L)\tag{7.2.7}
$$

$$
R _ {N} = R _ {e} (1 + f \sin^ {2} L)\tag{7.2.8}
$$

对应的曲率为

$$
\frac {1}{R _ {M}} = \left[ R _ {e} (1 - 2 f + 3 f \sin^ {2} L) \right] ^ {- 1} \approx \frac {1}{R _ {e}} (1 + 2 f - 3 f \sin^ {2} L)\tag{7.2.9}
$$

$$
\frac {1}{R _ {N}} = \left[ R _ {e} (1 + f \sin^ {2} L) \right] ^ {- 1} \approx \frac {1}{R _ {e}} (1 - f \sin^ {2} L)\tag{7.2.10}
$$

式中 $f$ 为旋转椭球扁率(或称椭圆度)

$$
f = \frac {R _ {e} - R _ {p}}{R _ {e}}
$$

## 3. 沿任意方向的曲率半径

设飞机上装备有当地水平式平台惯导系统，工和 为平台的水平轴 轴位于垂线方向。 $_ { \mathcal { X } }$ 轴和 $y$ 轴分别偏离东向和北向的夹角为 ，如图 7.2.5 所示。飞机的飞行速度沿 方向和 $y$ 方向的分量分别为 $V _ { x }$ 和 $V _ { y }$ o 要保持 $x P y$ 平面始终跟踪当地水平面，必须使该平面绕水平轴旋转，旋转角速度为

$$
\left[ \begin{array}{c} \omega_ {x} \\ \omega_ {y} \end{array} \right] = \left[ \begin{array}{c c} \cos \alpha & \sin \alpha \\ - \sin \alpha & \cos \alpha \end{array} \right] \left(\left[ \begin{array}{c} \omega_ {E} \\ \omega_ {N} \end{array} \right] + \left[ \begin{array}{c} 0 \\ \Omega_ {N} \end{array} \right]\right)
$$

式中， $\varOmega _ { N }$ 是地球自转角速度的北向分量(东向分量为零)。

![](images/b0d8ef33e400819947954829a52714761c455e860c587aad7dd826fc58884f32.jpg)  
(a)

![](images/9a06f768b627b48c0e300cf6dd9c7b2ebc1dd8a5a645a61f650293cfc1ae6408.jpg)  
(b)  
7.2.5 平台的 $x , y$ 轴与东、北向间的关系

$$
\left[ \begin{array}{c} \omega_ {E} \\ \omega_ {N} \end{array} \right] = \left[ \begin{array}{c} - \frac {V _ {N}}{R _ {M}} \\ \frac {V _ {E}}{R _ {N}} \end{array} \right] = \left[ \begin{array}{c c} 0 & - \frac {1}{R _ {M}} \\ \frac {1}{R _ {N}} & 0 \end{array} \right] \left[ \begin{array}{c} V _ {E} \\ V _ {N} \end{array} \right]
$$

而

所以

$$
\left[ \begin{array}{c} V _ {E} \\ V _ {N} \end{array} \right] = \left[ \begin{array}{c c} \cos \alpha & - \sin \alpha \\ \sin \alpha & \cos \alpha \end{array} \right] \left[ \begin{array}{c} V _ {x} \\ V _ {y} \end{array} \right]
$$

$$
\begin{array}{r l} \left[ \begin{array}{l} \omega_ {x} \\ \omega_ {y} \end{array} \right] & = \left[ \begin{array}{l l} \cos \alpha & \sin \alpha \\ - \sin \alpha & \cos \alpha \end{array} \right] \left[ \begin{array}{c c} 0 & - \frac {1}{R _ {M}} \\ \frac {1}{R _ {N}} & 0 \end{array} \right] \left[ \begin{array}{l l} \cos \alpha & - \sin \alpha \\ \sin \alpha & \cos \alpha \end{array} \right] \left[ \begin{array}{l} V _ {x} \\ V _ {y} \end{array} \right] + \left[ \begin{array}{l} \Omega_ {N} \sin \alpha \\ \Omega_ {N} \cos \alpha \end{array} \right] \\ & = \left[ \begin{array}{c c} \left(\frac {1}{R _ {N}} - \frac {1}{R _ {M}}\right) \sin \alpha \cos \alpha & - \left(\frac {\sin^ {2} \alpha}{R _ {N}} + \frac {\cos^ {2} \alpha}{R _ {M}}\right) \\ \frac {\cos^ {2} \alpha}{R _ {N}} + \frac {\sin^ {2} \alpha}{R _ {M}} & \left(- \frac {1}{R _ {N}} + \frac {1}{R _ {M}}\right) \sin \alpha \cos \alpha \end{array} \right] \left[ \begin{array}{l} V _ {x} \\ V _ {y} \end{array} \right] + \left[ \begin{array}{l} \Omega_ {N} \sin \alpha \\ \Omega_ {N} \cos \alpha \end{array} \right] \end{array}
$$

记

$$
\frac {1}{R _ {x}} = \frac {\cos^ {2} \alpha}{R _ {N}} + \frac {\sin^ {2} \alpha}{R _ {M}}\tag{7.2.11a}
$$

$$
\frac {1}{R _ {y}} = \frac {\sin^ {2} \alpha}{R _ {N}} + \frac {\cos^ {2} \alpha}{R _ {M}}\tag{7.2.11b}
$$

$$
\frac {1}{\tau_ {a}} = \left(\frac {1}{R _ {M}} - \frac {1}{R _ {N}}\right) \sin \alpha \cos \alpha\tag{7.2.11c}
$$

则

$$
\left[ \begin{array}{l} \omega_ {x} \\ \omega_ {y} \end{array} \right] = \left[ \begin{array}{c c} - \frac {1}{\tau_ {a}} & - \frac {1}{R _ {y}} \\ \frac {1}{R _ {x}} & \frac {1}{\tau_ {a}} \end{array} \right] \left[ \begin{array}{l} V _ {x} \\ V _ {y} \end{array} \right] + \left[ \begin{array}{l} \Omega_ {N} \sin \alpha \\ \Omega_ {N} \cos \alpha \end{array} \right]
$$

\~p

$$
\omega_ {x} = - \frac {V _ {y}}{R _ {y}} - \frac {V _ {x}}{\tau_ {a}} + \Omega_ {N} \sin \alpha\tag{7.2.12a}
$$

$$
\omega_ {y} = \frac {V _ {x}}{R _ {x}} + \frac {V _ {y}}{\tau_ {a}} + \Omega_ {N} \cos \alpha\tag{7.2.12b}
$$

(7.2.12)说明，当平台的 轴、 $\cdot y$ 轴分别与当地的东向、北向存在夹角 $\alpha$ 时 $, x$ 方向的速度不但引起水平面绕 轴的旋转，而且还引起绕 轴的旋转 $y$ 方向的速度也具有相同的影响。沿一个轴的飞行速度引起水平面绕同一轴的旋转反映了水平面的扭曲旋转，所以将式(7. 2. 11c)定的 $\frac { 1 } { \tau _ { a } }$ 为椭球面在 点处的扭曲率，而将式 <sup>(7.</sup> 2. <sup>11a)</sup> 和式 (7.2.11b) 确定的'....."..."..--...-..............- ...,,,\_..............,\_.\_.\_,, $\frac { 1 } { R _ { x } }$ 和 $\frac { 1 } { R _ { y } }$ 分别称为椭球面在 点处沿 $\mathcal { X }$ 方向的曲率和沿 $y$ 方向的曲率 $R _ { x }$ 和 $R _ { y }$ 为沿相应方向的曲率半径。从(7.2.11) 可看出，椭球面上同一点沿不同方向的曲率半径和扭曲率是不相同的。当 $\alpha { = } 0$ \~Px轴、 $\cdot y$ 轴分别指向东和北时

$$
R _ {x} = R _ {N}, \quad R _ {y} = R _ {M}, \quad \frac {1}{\tau_ {a}} = 0
$$

此时 $, x$ 方向的速度(即东向速度〉只引起水平面绕 $y$ 轴(北向轴〉的旋转 $y$ 方向的速度(即北向速度〉只引起水平面绕 $_ { \mathcal { X } }$ 轴(东向轴)的旋转，水平面不存在扭曲旋转，两个方向的运动是解祸的，所以将 $R _ { M }$ 和 $R _ { N }$ 称为主曲率半径。

## 7.2.4 重力加速度

为地球上某一点，该点纬度为 点处自由放置着质量 如图 7.2.6 所示。则质量

![](images/55f7aeda055300d68ad1222647564940ffa831704dbf29c97210dfdcbbc20250.jpg)  
7.2.6 重力加速度、引力加速度、向心加速度兰者间的关系

$m$ 受到地球的万有引力 mG 的作用，该力指向地心，同时维持质跟随地球旋转需要有外力提供向心力 $\pmb { F } _ { c }$ ，所以向心力实质上是万有引力的一个分量，用于维持质量 跟随地球旋转，重力$m { \bf g }$ 是万有引力的另一个分量，因此有

即

$$
m \mathbf {G} = m \mathbf {g} + \mathbf {F} _ {c}
$$

$$
\mathbf {G} = \mathbf {g} + \mathbf {a} _ {c}\tag{7.2.13}
$$

式中， ${ \mathbf { } } \cdot { \mathbf { } } \mathbf { = } \frac { \mathbf { } \mathbf { } F _ { c } } { m }$ 为 $P$ 点处地球旋转引起的向心加速度。由于向心加速度随纬度而变，所以重力加速度 与引力加速度 间的夹角(ì8也随纬度而变，在 $L = 4 5 ^ { \circ }$ \~f) 达到最大值约为 ${ 1 0 } ^ { \prime }$ 。重力

加速度的大小随纬度的变化规律可近似为

$$
g (L) = g _ {0} \left(1 + 0. 0 0 5 2 8 8 4 \sin^ {2} L - 0. 0 0 0 0 0 5 9 \sin^ {2} 2 L\right)\tag{7.2.14}
$$

式中， $g _ { 0 } { = } 9 7 8 . 0 4 9 \mathrm { c m / s } ^ { 2 }$ ;或

$$
g (L) = g _ {0} \left(1 - 0. 0 0 2 6 3 7 3 \cos 2 L + 0. 0 0 0 0 0 5 9 \cos^ {2} 2 L\right)\tag{7.2.15}
$$

式中， $g _ { 0 } = 9 8 0 .$ . 61 $\mathrm { 6 c m / s ^ { 2 } }$ ，式 (7.2.15) 称为达朗贝尔方程。

WGS-84 全球大地坐标系体系中选用的重力加速度模型为

$$
g (L) = \frac {g _ {e} (1 + k \sin^ {2} L)}{\sqrt {1 - e _ {1} ^ {2} \sin^ {2} L}}\tag{7.2.16}
$$

式中

$$
k = \frac {R _ {p} g _ {p}}{R _ {e} g _ {e}} - 1, \quad e _ {1} = \frac {\sqrt {R _ {e} ^ {2} - R _ {p} ^ {2}}}{R _ {e}}
$$

$g _ { e }$ 为赤道上的理论重力加速度， $g _ { \boldsymbol { \rho } }$ 为两极处的理论重力加速度 $e _ { 1 }$ 为第一偏心率。将 WGS-84的具体数据代人，得

$$
g (L) = 9 7 8. 0 3 2 6 7 7 1 4 \times \frac {1 + 0 . 0 0 1 9 3 1 8 5 1 3 8 6 3 9 \sin^ {2} L}{\sqrt {1 - 0 . 0 0 6 6 9 4 3 7 9 9 9 0 1 3 \sin^ {2} L}}\tag{7.2.17}
$$

重力加速度随高度的变化规律为

$$
g (h) = g _ {0} \frac {R _ {e} ^ {2}}{\left(R _ {e} + h\right) ^ {2}} \approx g _ {0} \left(1 - \frac {2 h}{R _ {e}}\right)\tag{7.2.18}
$$

## 7.2.5 地球上的定位参数

1.地球定位中两类坐标体系

## 。地球直角坐标系

地球直角坐标系的原点位于旋转椭球体中心 轴与地球自转轴重合，正向沿地球自转方向，工和 轴位于赤道平面内 轴穿过本初子午线 轴穿过东经 $9 0 ^ { \circ }$ 子午线。 点的位置用

点在该坐标系内的坐标( $( x , y , z )$ 来表示。

## 2) 地球球面坐标系

地球球面坐标系的原点位于旋转椭球中心， 点的位置用 点的经度、纬度和高度来表示。

2. 两类坐标参数的转换

点在地球直角坐标系内的坐标为( $( x , y , z )$ ，在地球球面坐标系内的坐标为 $( \lambda , L , h )$ ,如7.2.7 所示。

![](images/361c1738bb3bfa4f25582a8ed99de0932b60aa8c62192f8d67adc1322c2834fa.jpg)  
7.2.7 地球直角坐标系和地球球面坐标系

1)球面坐标至直角坐标的转换

由图 7.2.7

$$
x = \left(R _ {N} + h\right) \cos L \cos \lambda\tag{7.2.19a}
$$

$$
y = (R _ {N} + h) \cos L \sin \lambda\tag{7.2.19b}
$$

$$
z = R _ {N} (1 - f) ^ {2} \sin L \quad (\text {不考虑} h)\tag{7.2.19c}
$$

或

$$
z = \left[ R _ {N} (1 - f) ^ {2} + h \right] \sin L \quad (\text {考虑} h)\tag{7.2.19d}
$$

2) 直角坐标至球面坐标的转换

①当 $h = 0$ 时，由式 (7.2.19) ，得

$$
\lambda_ {\text {主}} = \arctan {\frac {y}{x}}\tag{7.2.20a}
$$

$$
L = \arctan \left[ \frac {1}{(1 - f) ^ {2}} \cdot \frac {z}{\sqrt {x ^ {2} + y ^ {2}}} \right] = \arctan \left[ \frac {R _ {e} ^ {2}}{R _ {p} ^ {2}} \cdot \frac {z}{\sqrt {x ^ {2} + y ^ {2}}} \right]\tag{7.2.20b}
$$

已知，且 不大时

$$
L \approx \arctan \left[ \left(\frac {R _ {e} + h}{R _ {p} + h}\right) ^ {2} \cdot \frac {z}{\sqrt {x ^ {2} + y ^ {2}}} \right]\tag{7.2.20c}
$$

②当 $h \neq 0$ 未知时

$$
\lambda_ {\text {主}} = \arctan {\frac {y}{x}}
$$

由迭代公式求出

$$
(R _ {N} + h) _ {i + 1} = \frac {x}{\cos L _ {i} \cos \lambda}\tag{7.2.21a}
$$

$$
(R _ {N}) _ {i + 1} = \frac {R _ {e}}{\sqrt {\cos^ {2} L _ {i} + (1 - e _ {1} ^ {2}) \sin^ {2} L _ {i}}}\tag{7.2.21b}
$$

$$
L _ {i + 1} = \arctan \left[ \frac {\left(R _ {N} + h\right) _ {i + 1}}{\left(R _ {N} + h\right) _ {i + 1} - \left(R _ {N}\right) _ {i + 1} e _ {1} ^ {2}} \cdot \frac {z}{\sqrt {x ^ {2} + y ^ {2}}} \right]\tag{7.2.21c}
$$

$$
L _ {0} = \arctan \left[ \frac {1}{(1 - f) ^ {2}} \cdot \frac {z}{\sqrt {x ^ {2} + y ^ {2}}} \right]\tag{7.2.21d}
$$

式中 $\cdot e _ { 1 } = \frac { \sqrt { R _ { e } ^ { 2 } - R _ { \phi } ^ { 2 } } } { R _ { e } }$ 设经 次迭代达到精度要求，则

$$
h = (R _ {N} + h) _ {k} - (R _ {N}) _ {k}\tag{7.2.21e}
$$

真值按表 7.2.2 确定，其中正、负号表示东经和西经。

7.2.2λ 真值确定

<table><tr><td>x</td><td>+</td><td>+</td><td>-</td><td>-</td></tr><tr><td>y</td><td>+</td><td>-</td><td>+</td><td>-</td></tr><tr><td>λ</td><td colspan="2">λ主</td><td>180°+λ主</td><td>-180°+λ主</td></tr></table>

## 3. 在不同大地坐标革内定位参数的转换

旋转椭球体是对地球的近似描述，椭球体的参数是通过大地测量获得的，即根据对地球的某一局部地区的测量值求解出椭球参数，所以确定出的椭球对该地区的拟合精度最好，但对其他地区精度就差些。 WGS-84 坐标系是根据外层空间对地球的观测值确定出的，该坐标系全面反映了地球的几何特征，所以此处以 WGS-84 作为基准，其他坐标系与之比较来讨论各坐标之间存在的差异，表 7.2.3 列出了比较结果。从表中可看出，各坐标之间的差异是很小的。

7.2.3 大地坐标系间的比较

<table><tr><td>大地坐标系名称</td><td>测量原点</td><td>参考椭球名称</td><td>适用地区</td><td>原点差异 Δx,Δy,Δz/m</td></tr><tr><td>1980 北京</td><td></td><td>1975 国际</td><td>中国</td><td></td></tr><tr><td>1942 普尔柯夫</td><td>59°46′18.55′′N30°19′42.09′′E</td><td>克拉索夫斯基</td><td>俄国</td><td></td></tr><tr><td>1927 北美</td><td>39°13′26.666′′N98°32′30.506′′W</td><td>克拉克</td><td>北美</td><td>-22,+157,+180.5</td></tr><tr><td>1918 东京</td><td>35°39′57.51′′N139°44′40.50′′E</td><td>贝塞尔</td><td>日本及中国台湾省</td><td>-140,+576,+677.5</td></tr><tr><td>1952 欧洲</td><td>52°22′51.445′′N13°03′58.928′′E</td><td>海福德</td><td>欧洲、北美及中近东</td><td>-84,-103,-122.5</td></tr><tr><td>WGS-84</td><td>地心</td><td>WGS-84</td><td>全球</td><td>0,0,0</td></tr></table>

假设大地坐标系 的相应轴互相平行，大地坐标系 的原点偏离大地坐标系 的原点的偏离值为 $\Delta x , . \Delta y , \Delta z , P$ 点在大地坐标系 内的坐标为() $( \lambda _ { 1 } , L _ { 1 } , h _ { 1 } )$ 点在大地坐标系

的坐标为

$$
\lambda_ {2} = \lambda_ {1} + \Delta \lambda , L _ {2} = L _ {1} + \Delta L, h _ {2} = h _ {1} + \Delta h
$$

式中

$$
\begin{array}{l} \Delta \lambda = \frac {\sec \lambda_ {1}}{R _ {N}} (- \sin \lambda_ {1} \Delta x + \cos \lambda_ {1} \Delta y) \\ \Delta L = \frac {1}{R _ {M}} \left\{- \sin L _ {1} \cos \lambda_ {1} \Delta x - \sin L _ {1} \sin \lambda_ {1} \Delta y + \cos L _ {1} \Delta z + \frac {R _ {N}}{R _ {e}} f (2 - f) \sin L _ {1} \cos L _ {1} \Delta R _ {e} \right. \\ \left. + \left[ \frac {R _ {M}}{1 - f} + R _ {N} (1 - f) \right] \sin L _ {1} \cos L _ {1} \Delta f \right\} \end{array} \tag {7.2}\tag{7.2.22a}
$$

(7 μ2b)

$$
\Delta h = \cos L _ {1} \cos \lambda_ {1} \Delta x + \cos L _ {1} \sin \lambda_ {1} \Delta y + \sin L \Delta z - \frac {R _ {e}}{R _ {N}} \Delta R _ {e} + (1 - f) R _ {N} \sin^ {2} L _ {1} \Delta f\tag{7.2.22c}
$$

式中 . $\Delta f = f _ { 2 } - f _ { 1 } , \Delta R _ { e } = R _ { e _ { 2 } } - R _ { e _ { 1 } } , R _ { e }$ 可任意选用两坐标系中的椭球参数。

## 7.3 休拉调谐

## 7.3.1 休拉调谐原理

在运载体上确定出地垂线后即可确定出运载体的姿态。在静止或匀速直线运动条件下，地垂线可用单摆等简单方法确定出来，但当运载体具有加速度时，单摆跟踪视在垂线，加速度越大，单摆偏离地垂线越严重，所以采用简单办法是无法正确指示地垂线的。

1910 年德国科学家休拉发现当陀螺罗经的无阻尼振荡周期为 84.4 分钟时，罗经的指北精度将不受外界干扰加速度的影响，随后发现对垂线指示系统同样具有这一结论.1923 年休拉发表论文详细阐述了惯性系统的无干扰条件(Schuler M. 1923).即休拉谐调原理。

假设垂线指示系统用物理摆来实现，物理摆悬挂在飞机上，物理摆绕悬挂中心的转动惯量为].飞机在 时刻以前处于水平匀速直线运动状态，摆处于 OA位置(垂线位置) .0 时刻以后以加速度 做水平直线加速运动.t 时刻到达 点，由于加速度 引起的愤性力的作用，摆偏开垂线Iß.产生相对垂线的偏差角，如图 7.3.1 所示。

点(对应于 时刻tl) 点(对应于 时刻〉地垂线的角位移为 $\alpha _ { b }$ ，物理摆的角位移为$\alpha _ { a }$ .由图 7.3.1

$$
\alpha_ {a} = \alpha + \alpha_ {b}\tag{7.3.1}
$$

根据动量矩定理，物理摆的运动方程为

$$
J _ {\alpha_ {a}} ^ {\prime \prime} = - m g l \sin \alpha + m a l \cos \alpha\tag{7.3.2}
$$

由式 (7.3.1)

$$
\ddot {\alpha} _ {a} = \ddot {\alpha} + \ddot {\alpha} _ {b}
$$

式中， $\ddot { \alpha } _ { b }$ 是由飞机运动引起的地垂线的角加速度

$$
\ddot {\alpha} _ {b} = \frac {a}{R}
$$

所以式 (7.3.2) 可写成

$$
J _ {\alpha} ^ {\cdot \cdot} + \frac {J a}{R} = m a l \cos \alpha - m g l \sin \alpha
$$

假设垂线偏差角很小，则上式可写成

$$
\ddot {\alpha} + \frac {m g l}{J} \alpha = \left(\frac {m l}{J} - \frac {1}{R}\right) a\tag{7.3.3}
$$

如果物理摆满足

$$
\frac {m l}{J} - \frac {1}{R} = 0
$$

\~p

$$
\frac {m l}{J} = \frac {1}{R}\tag{7.3.4}
$$

则式 (7.3.3)

$$
\ddot {\alpha} + \frac {g}{R} \alpha = 0\tag{7.3.5}
$$

![](images/e7d857db18edaed39dae598b327181c3003cfe02c7da17d5ef9a25161c579fd8.jpg)  
物理摆受运载体运动的影响7.3.1

(7.3.5)说明:当物理摆满足式(7.3.4)时，摆偏离垂线的偏差角 $\alpha$ 与飞机的运动状态无 关， $, \alpha$ 的变化规律满足式(7.3.5)所示齐次方程，解该方程得

$$
\alpha (t) = \alpha (0) \cos \omega_ {s} t + \frac {\dot {\alpha} (0)}{\omega_ {s}} \sin \omega_ {s} t\tag{7.3.6}
$$

式中

$$
\omega_ {s} = \sqrt {\frac {g}{R}}\tag{7.3.7}
$$

称为休拉频率 ${ \mathfrak { s o } } ( 0 )$ 和 $\overset { \cdot } { \alpha } \left( 0 \right)$ 为摆的初始偏差角和偏差角变化率初值。

根据式(7.3.7) ，可计算出对应于 $\omega _ { s }$ 的振荡周期:

$$
T _ {s} = \frac {2 \pi}{\omega_ {s}} = 2 \pi \sqrt {\frac {R}{g}}\tag{7.3.8}
$$

取 $g { = } 9 . 8 1 \mathrm { { m } / \mathrm { { s } ^ { 2 } , } } R { = } 6 ~ 3 7 1 ~ 0 0 0 \mathrm { { m } }$ ，则

$$
T _ {s} = 8 4. 4 \mathrm{分钟}\tag{7.3.9}
$$

称为地球上的休拉周期。

从式(7.3.6)可看出，如果 $\alpha ( 0 ) = 0 , \dot { \alpha } ( 0 ) = 0$ ，则飞机不管飞到何处，不管飞机的运动状态如何，摆都能正确指示地垂线，这种摆称为休拉摆。式(7.3. 的是实现休拉摆的条件，称之为休拉调谐条件。此条件是地球环境下推出的，但实际也适用于其他星球，唯一不同的是 $_ { g }$ 应更换成该星球的星球半径和重力加速度。

对照单摆的振荡周期 $T _ { \mathit { p } } { = } 2 \pi \sqrt { \frac { \mathit { l } } { \mathit { g } } }$ ，其中 为摆线长度，可看出:如果要使单摆成为休拉摆，则 $l { = } R$ 即摆线长度应等于地球半径，所以单摆是不可能正确跟踪地垂线的。事实上，物理摆也不能成为休拉摆。原因分析如下。

根据式(7.3.4) ，物理摆实现休拉调谐的条件是:

$$
l = \frac {J}{m R}
$$

由于 是地球半径，要使 尽量大，须在摆的质量一定的条件下转动惯量尽量大，所以将物理摆设计成环状是最佳方案。假设环半径 $r { = } 0 . 5 \mathrm { m }$ 环的质量全部集中在圆周上，可计算得物理摆的悬挂点距环质，心的距离 $l { = } 0 . 0 4 \mu \mathrm { m }$ ，大概是头发粗细的万分之一，这在工程上是元法做到的。

## 7.3.2 单轴惯导系统和休挂调谐的实现

虽然用单摆和物理摆等简单方法是无法实现休拉调谐的，但惯导系统却很容易实现休拉调谐。

7.3.2 为单通道惯性导航系统，该系统只能对沿子午线运动的运载体作导航解算。单轴陀螺稳定平台是该系统的核心，平台稳定轴沿东西方向，由一个单自由度陀螺 $G _ { E }$ 控制，平台上安装加速度计 $A _ { N }$ '敏感轴位于北向。

![](images/ba3119bd173a2be06b7e6d7dc9b1aa4ced763cadcc360ad67884f2a8abbaac6d.jpg)  
7.3.2 单轴惯导系统

系统工作原理简述如下:北向加速度计 $A _ { N }$ 感测飞机的北向加速度 $\boldsymbol { a } _ { N }$ ，经一次积分后得北向速度 $V _ { N }$ 求出地垂线的旋转角速度 $V _ { N } / R$ 并作为指令角速度馈入东向陀螺 $G _ { E }$ 力矩器，产生指令力矩 $M _ { K }$ '驱动陀螺组件旋转产生输出角 $\sigma ,$ ，经放大校正后馈人力矩电机，电机驱动平台以角速度 $\dot { \phi } _ { c }$ 旋转，而 $\dot { \phi } _ { c }$ 使陀螺产生陀螺、力矩 $\dot { \mathcal { I } } \dot { \phi }$ ，方向沿陀螺输出轴反方向，当陀螺力矩 $\dot { H \phi }$ 与指令力矩大小相等时，陀螺输出轴上的力矩达到平衡， $\dot { \phi } _ { \mathfrak { c } }$ 件达到稳态，平台跟踪地垂线的转动。这就是陀螺稳定平台处于空间积分状态的工作原理。当平台稳定轴上作用有干扰力矩 $M _ { \mathrm { d } }$ 时，陀螺发生进动， $, \vec { p }$ 生输出角 $\sigma ,$ 经放大校正后馈人力矩电机，力矩电机产生卸荷力矩 $M _ { m }$ ' $M _ { m }$ 与 $M _ { d }$ 达到平衡时，输出角 $\sigma$ 达到稳态。这就是平台处于几何稳定状态的工作原理。

系统中根据陀螺输出角控制力矩电机产生卸荷力矩或平台驱动力矩的回路称为单轴平台的稳定回路;解算指令角速度，控制陀螺产生相应的输出角并通过稳定回路驱动平台按指令角速度旋转的回路称为单轴平台的修正回路。

下面分析单轴惯导系统惯性平台的方块图。图 7.3.3 为加速度计的工作情况。

![](images/62aca8770053dacc3b8919a327773816e44849da4e6aaf6742f19b9381e849b3.jpg)  
7.3.3 平台上加速度计的输出

## 1)加速度计的输入及输出

设 $N _ { 0 } N _ { 0 }$ 。为 $t _ { 0 }$ 时刻地垂线位置，假设 $t _ { 0 }$ 时刻平台建立的指示垂线与地垂线重合。由于飞机的运动 $, t _ { 1 }$ 时刻地垂线处于 $N _ { 1 } N _ { 1 }$ 位置，从 $t _ { 0 }$ 至 $t _ { 1 }$ 时刻，地垂线的角位移为 $\phi , t _ { 1 }$ 时刻建立的指示垂线为 $N _ { 1 } ^ { \prime } N _ { 1 } ^ { \prime }$ 从 $t _ { 0 }$ 至 $t _ { 1 }$ 时刻平台的旋转角为 $\phi _ { c }$ 。在 $t _ { 1 }$ 时刻平台的水平误差角为

$$
\alpha = \phi_ {c} - \phi\tag{7.3.10}
$$

式中， $\phi$ 由 $a _ { N }$ 引起， $\ddot { \phi } = \frac { a _ { N } } { R }$ ，即

$$
s ^ {2} \phi (s) = \frac {a _ {N} (s)}{R}\tag{7.3.11}
$$

此时，加速度计敏感重力加速度分量 $\alpha g$ ，在该分量作用下，弹簧被拉长，这相当于在加速度计敏感轴的反方向作用有加速度

$$
a _ {i} = \alpha g
$$

所以加速度计的输入为

$$
a _ {I} = a _ {N} \cos \alpha - \alpha g
$$

由于平台的水平偏差角为小角， $\mathrm { c o s } \alpha { \approx } 1$ ，所以加速度计的输入为

$$
a _ {I} = a _ {N} - (\phi_ {c} - \phi) g
$$

RP

$$
a _ {I} (s) = a _ {N} (s) - \left[ \phi_ {c} (s) - \phi (s) \right] g\tag{7.3.12}
$$

设加速度计的刻度系数为 $K _ { a }$ ，则输出为

$$
u _ {A} (s) = K _ {a} a _ {I} (s)
$$

2) 陀螺的动力学方程

由图 7.3.2

$$
\ddot {I \sigma} = M _ {k} - D \dot {\sigma} - H \dot {\phi} _ {c}\tag{7.3.13}
$$

即

$$
\sigma (s) = \frac {1}{I s ^ {2} + D s} \left[ M _ {k} (s) - H s \phi_ {c} (s) \right]\tag{7.3.14}
$$

式中， 为陀螺组件绕输出轴的转动惯量 $M _ { k } \left( s \right)$ 是指令力矩，与指令电流的关系为

$$
M _ {k} (s) = K _ {m} i _ {\mathrm{cmd}} (s)\tag{7.3.15}
$$

$$
i _ {\mathrm{cmd}} (s) = u _ {A} (s) \cdot \frac {1}{s} \cdot \frac {1}{R} \cdot K _ {u}\tag{7.3.16}
$$

式中 $K _ { u }$ 为积分器的传递系数 $K _ { m }$ 为陀螺力矩器的力矩系数。

3) 平台台体的动力学方程

设平台绕稳定轴的转动惯量为]，则由图 7.3.2

$$
J \ddot {\phi} _ {c} = M _ {m} + H \dot {\sigma} - M _ {\mathrm{d}}
$$

即

$$
\phi_ {c} (s) = \frac {1}{J s ^ {2}} \left[ M _ {m} (s) + H s \sigma (s) - M _ {\mathrm{d}} (s) \right]\tag{7.3.17}
$$

式中， $M _ { m }$ .为电机的伺服力矩

$$
M _ {m} (s) = K _ {M} K (s) K _ {V \sigma} (s)\tag{7.3.18}
$$

式中 $K _ { V }$ 为陀螺信号器的传递系数 $K ( s )$ 为校正放大网络的传递函数 $K _ { M }$ 为力矩电机的力矩系数，

由式(7. 3. 10)\~ (7.3.18) ，可画出单轴惯性平台的方块图，如图 7.3.4 所示。

由于在平台设计中，稳定回路带宽设计得足够宽，平台具有很好的快速性，所以可以把平台看成一个双自由度等效陀螺仪，单自由度陀螺的输出轴可看做内环轴，平台稳定轴可看做外环轴，等效陀螺角动量为 $H , \dot { \phi } _ { \mathfrak { c } }$ 户可看作在 $M _ { k }$ ，作用下等效陀螺的进动角速度

$$
\dot {\phi} _ {c} = \frac {M _ {k}}{H}
$$

RP

$$
\phi_ {c} (s) = \frac {M _ {k} (s)}{H s}\tag{7.3.19}
$$

所以，图 7.3.4 可简化成图 7.3.5

![](images/44c65cf2f1b9c48a502589d65dcf1225485d3f7d3000c20bd32a0f0c50561aa8.jpg)

7.3.4 单轴惯性平台方块图  
![](images/385ded5dce89fad39f9449eeb7a51231bb88e232fa861e5622c23d12590ec546.jpg)  
7.3.5 单轴惯性平台经动力学简化后的方块图

将图 7.3.5 中的积分环节移至比较点①的外面，并令

$$
K _ {\Sigma} (s) = K _ {a} \cdot \frac {K _ {u}}{s} \cdot \frac {1}{R} \cdot K _ {m} \cdot \frac {1}{H} = \frac {K _ {a} K _ {u} K _ {m}}{R H s}\tag{7.3.20}
$$

则图 7.3.5 还可简化成图 7.3.6

![](images/9632d0b03eecd801772c5baf786c49cbcb5788891796c3d899fe66e2b4829108.jpg)  
7.3.6 单轴惯性平台经等效简化后的方块图

由图得

$$
X (s) = \left[ a _ {N} (s) - \alpha (s) g \right] K _ {\Sigma} (s) = K _ {\Sigma} (s) a _ {N} (s) - K _ {\Sigma} (s) g \alpha (s)
$$

$$
Y (s) = - \frac {1}{R s} a _ {N} (s) + X (s) = \left[ K _ {\Sigma} (s) - \frac {1}{R s} \right] a _ {N} (s) - K _ {\Sigma} (s) g \alpha (s)\tag{7.3.21a}
$$

$$
\alpha (s) = \frac {1}{s} Y (s)\tag{7.3.21b}
$$

由式(7.3.21)可画出单轴惯性平台的最简方块图，如图 7.3.7(a)所示。

![](images/5bf21a154227c2f426917de3a00564a6be54585f72570e19af650ae6aff3c0e0.jpg)  
(a)

![](images/1f6d22f44cb8913b9675075a82ca34306a25736fb915dd0b2bf24898fca304ee.jpg)  
(b)  
7.3.7 单轴惯性平台的最简方块图

从图 7. 3. 7(a) 可看出，当 $K _ { \Sigma } ( s ) - \frac { 1 } { R s } \mathrm { = } 0$ ep

$$
K _ {\Sigma} (s) = \frac {1}{R s}\tag{7.3.22}
$$

时，平台的水平偏差角与飞机的加速度 $\boldsymbol { a } _ { N }$ 元关。根据式 (7.3.22) ，图 7.3.7(a) 可进一步简化成7. 3. 7(b) 。由图 7. 3. 7(b)

$$
\alpha (s) = 0 \cdot \frac {\frac {1}{s}}{1 + \frac {g}{R s} \cdot \frac {1}{s}} = \frac {0}{s ^ {2} + \frac {g}{R}}
$$

即

$$
\left(s ^ {2} + \frac {g}{R}\right) \alpha (s) = 0
$$

对应的时域方程为

$$
\ddot {\alpha} (t) + \frac {g}{R} \alpha (t) = 0, \alpha (t) = \alpha (0) \cos \omega_ {s} t + \frac {\dot {\alpha} (0)}{\omega_ {s}} \sin \omega_ {s} t
$$

式中， $, \alpha ( 0 )$ 和 $\dot { \alpha } ( 0 )$ 为水平误差角及其变化率的初始值

$$
\omega_ {s} = \sqrt {\frac {g}{R}}
$$

对应的周期为

$$
T _ {s} = \frac {2 \pi}{\omega_ {s}} = 2 \pi \sqrt {\frac {R}{g}} = 8 4. 4 \text {分钟}
$$

将式 (7.3.20) 代人式 (7.3.22) ，得

$$
\frac {K _ {a} K _ {u} K _ {m}}{H} = 1\tag{7.3.23}
$$

上述分析说明 设计单轴惯性平台时，只要满足式(7. 3. 23) ，就可使平台具有 84.4 分钟的振荡周期，实现休拉调谐。

三轴平台是单轴平台的复合，各通道间藕合是很弱的，俯仰环和横滚环的休拉调谐都可按单轴平台分析和设计。

## 7.4 惯导系统的基本方程 比力方程

为分析和讨论方便，首先定义几个常用坐标系。

(1)地心惯性坐标系，:见 2. 1. 节中的定义。

(2) 地球坐标系 $e :$ 原点位于地心 $\mathcal { X } _ { e }$ 轴穿越本初子午线与赤道的交点 $\mathfrak { Z } _ { e }$ 轴穿越地球北极点 $\cdot \ y _ { e }$ 轴穿越东经 $9 0 ^ { \circ }$ 子午线与赤道的交点。该坐标系与地球固联。

(3) 地理坐标系 $g :$ 原点位于运载体质心 $, x _ { g } , y _ { g } , z _ { g }$ 分别指向所在地的东、北、夭。

(4) 导航坐标系 $n \colon$ 用于导航解算的参考坐标。

(5) 理想平台坐标系 T:导航坐标系的无误差复现。

(6) 实际平台坐标系 $P :$ 由平台台体上的愤性仪器敏感轴确定，是导航坐标系的具体复现。相对 系存在平台失准角。

(7) 机体坐标系 $b { \mathrm { : } }$ 原点位于飞机质 $\therefore \dot { \Sigma } \cdot \mathcal { X } _ { b } \setminus \mathcal { Y } _ { b } \setminus \mathcal { Z } _ { b }$ 分别指向右、前、上。

自地心至理想平台坐标系 的支点引位置矢量 则根据哥氏定理

$$
\left. \frac {\mathrm{d} \boldsymbol {R}}{\mathrm{d} t} \right| _ {i} = \left. \frac {\mathrm{d} \boldsymbol {R}}{\mathrm{d} t} \right| _ {e} + \boldsymbol {\omega} _ {i e} \times \boldsymbol {R}\tag{7.4.1}
$$

时， $\frac { \mathrm { d } R } { \mathrm { d } t } \bigg \vert$ 地球上观察到的位置矢量的变化率，所以是运载体相对地球的运动速度，简称为地速，记作 $\mathbf { V } _ { e T }$

对式(7.4. 1)两边求绝对变化率，并再次使用哥氏定理，其中相对变化率对 系求取

$$
\left. \frac {\mathrm{d} ^ {2} \boldsymbol {R}}{\mathrm{d} t ^ {2}} \right| _ {i} = \left. \frac {\mathrm{d} \boldsymbol {V} _ {e T}}{\mathrm{d} t} \right| _ {T} + \boldsymbol {\omega} _ {i T} \times \boldsymbol {V} _ {e T} + \boldsymbol {\omega} _ {i e} \times (\boldsymbol {V} _ {e T} + \boldsymbol {\omega} _ {i e} \times \boldsymbol {R}) + \frac {\mathrm{d} \boldsymbol {\omega} _ {i e}}{\mathrm{d} t} \Bigg | _ {i} \times \boldsymbol {R}
$$

由于 $\pmb { \omega } _ { i T } = \pmb { \omega } _ { i e } + \pmb { \omega } _ { e T } , \frac { \mathrm { d } \pmb { \omega } _ { i e } } { \mathrm { d } t } \bigg | _ { i } = \pmb { 0 }$ 以上式可写成

$$
\left. \frac {\mathrm{d} ^ {2} \boldsymbol {R}}{\mathrm{d} t ^ {2}} \right| _ {i} = \left. \frac {\mathrm{d} \boldsymbol {V} _ {e T}}{\mathrm{d} t} \right| _ {T} + (2 \boldsymbol {\omega} _ {i e} + \boldsymbol {\omega} _ {e T}) \times \boldsymbol {V} _ {e T} + \boldsymbol {\omega} _ {i e} \times (\boldsymbol {\omega} _ {i e} \times \boldsymbol {R})\tag{7.4.2}
$$

设平台上加速度计质量块的质量为 $m$ 质量 受到的力为非引力外力 和地球引力mG，为引力加速度。根据牛顿第二定律

$$
\boldsymbol {F} + m \boldsymbol {G} = m \frac {\mathrm{d} ^ {2} \boldsymbol {R}}{\mathrm{d} t ^ {2}} \Bigg | _ {i}
$$

即

式中

$$
\left. \frac {\mathrm{d} ^ {2} \boldsymbol {R}}{\mathrm{d} t ^ {2}} \right| _ {i} = \boldsymbol {f} + \boldsymbol {G}\tag{7.4.3}
$$

$$
f = \frac {\boldsymbol {F}}{m}\tag{7.4.4}
$$

是单位质量上作用的非引力外力，称之为比力 (specific force)

将式 (7.4.3) 代人式 (7.4.2) ，得

$$
\left. \frac {\mathrm{d} \boldsymbol {V} _ {e T}}{\mathrm{d} t} \right| _ {T} = \boldsymbol {f} - (2 \boldsymbol {\omega} _ {i e} + \boldsymbol {\omega} _ {e T}) \times \boldsymbol {V} _ {e T} + \boldsymbol {G} - \boldsymbol {\omega} _ {i e} \times (\boldsymbol {\omega} _ {i e} \times \boldsymbol {R})\tag{7.4.5}
$$

7.4.1 示出了式(7.4.5) 中最后两项与重力加速度间的关系。由图7.4. l(a) ，得

$$
\left| \omega_ {i e} \times R \right| = \omega_ {i e} R \sin (9 0 ^ {\circ} - L) = \omega_ {i e} R \cos L
$$

$$
\left| \boldsymbol {\omega} _ {i e} \times \left(\boldsymbol {\omega} _ {i e} \times \boldsymbol {R}\right) \right| = \omega_ {i e} \cdot \left(\omega_ {i e} R \cos L\right) \sin 9 0 ^ {\circ} = R \cos L \omega_ {i e} ^ {2}
$$

$\pmb { \omega } _ { i e } \times ( \pmb { \omega } _ { i e } \times \pmb { R } )$ 的方向指向地轴。

![](images/604feee78d07ef674d1cb27a1b351ae2cde82103a50e69fa8fc65fde8376ff1e.jpg)

![](images/b8af7855fa786df90ff123b03f08a4fff7aa2a9c7561e45f238b66c020285213.jpg)  
7.4.1 地球旋转引起的向心加速度

考察 点跟随地球旋转引起的向心加速度。由图 7.4. l(b) ，向心加速度 $\pmb { a } _ { c }$ 的大小为 $\boldsymbol { a } _ { c } =$ $R { \mathrm { c o s } } L \omega _ { i e } ^ { 2 }$ ，方向指向地轴。

由上述分析知

$$
\boldsymbol {\omega} _ {i e} \times (\boldsymbol {\omega} _ {i e} \times \boldsymbol {R}) = \boldsymbol {a} _ {c}
$$

又根据式 (7. 2. 13)

$$
\mathbf {g} = \mathbf {G} - \mathbf {a} _ {c}
$$

因此式 (7.4.5) 可写成

$$
\left. \frac {\mathrm{d} \mathbf {V} _ {e T}}{\mathrm{d} t} \right| _ {T} = \boldsymbol {f} - (2 \boldsymbol {\omega} _ {i e} + \boldsymbol {\omega} _ {e T}) \times \mathbf {V} _ {e T} + \mathbf {g}\tag{7.4.6}
$$

该式即为比力方程，是惯导系统的基本方程。

现对比力方程作如下说明:

(1) $\left. \frac { \mathrm { d } \mathbf { V } _ { e T } } { \mathrm { d } t } \right| _ { T }$ 是在平台坐标系内观察到的地速矢量 $\mathbf { V } _ { e T }$ 的变化率，如果将式(7.4. 6) 系内技影，则比力方程可写成分量形式:

$$
\dot {\mathbf {V}} _ {e T} ^ {T} = \boldsymbol {f} ^ {T} - \left(2 \boldsymbol {\omega} _ {i e} ^ {T} + \boldsymbol {\omega} _ {e T} ^ {T}\right) \times \mathbf {V} _ {e T} ^ {T} + \boldsymbol {g} ^ {T}\tag{7.4.7}
$$

(2) 是加速度计的测量值，比力方程说明只有当 清除掉了有害加速度之后，才能积分获得地速。其中有害加速度包括三部分:有害加速度 $2 \pmb { \omega } _ { i e } \times \pmb { V } _ { e T }$ 为哥氏加速度，由运载体相对地球运动(相对运动)和地球旋转(牵连运动)引起;有害加速度 ${ \pmb { \omega } } _ { e T } \times \pmb { V } _ { e T }$ 是运载体保持在地球表面运动(圆周运动)引起的对地向心加速度;重力加速度 是有害加速度的第兰部分。

(3) 在静基座条件下，即 ${ \pmb V } _ { e T } = { \bf 0 } , f = - { \pmb g } ,$ 。由 $\scriptstyle \mp { F = m } f$ 是作用在质量块上的非引力外力，亦即质量块受到的约束力。根据作用与反作用原理，质量块对约束体的反作用力为 $A = - F =$ ${ \bf \cdot } { \bf \cdot } { \bf \cdot } f = m { \bf g }$ 其中 即为质量块产生的惯性力，所以静基座条件下质量块的单位质量惯性力为 $F _ { i }$ $= \frac { \mathbf { A } } { m } { = } \pmb { g } _ { \circ }$

## 7.5 惯性高度通道的稳定性分析

<sup>TT</sup> 1 VeTx T <sup>T</sup>  11ω1口| [ωeTx V\~ = lV\~y I ,F = I 川 ，ω = wiey I 'ωL ωeTy ，则式(7.4.7)可写成： ITTT <sub>eTz</sub> .rT 11ω四 T ωeTz

$$
\left[ \begin{array}{c} \dot {V} _ {e T x} ^ {T} \\ \dot {V} _ {e T y} ^ {T} \\ \dot {V} _ {e T z} ^ {T} \end{array} \right] = \left[ \begin{array}{c} f _ {x} ^ {T} \\ f _ {y} ^ {T} \\ f _ {z} ^ {T} \end{array} \right] - \left[ \begin{array}{c c c} 0 & - (2 \omega_ {i e z} ^ {T} + \omega_ {e T z} ^ {T}) & 2 \omega_ {i e y} ^ {T} + \omega_ {e T y} ^ {T} \\ 2 \omega_ {i e z} ^ {T} + \omega_ {e T z} ^ {T} & 0 & - (2 \omega_ {i e x} ^ {T} + \omega_ {e T x} ^ {T}) \\ - (2 \omega_ {i e y} ^ {T} + \omega_ {e T y} ^ {T}) & 2 \omega_ {i e x} ^ {T} + \omega_ {e T x} ^ {T} & 0 \end{array} \right] \times \left[ \begin{array}{c} V _ {e T x} ^ {T} \\ V _ {e T y} ^ {T} \\ V _ {e T z} ^ {T} \end{array} \right] + \left[ \begin{array}{c} 0 \\ 0 \\ - g \end{array} \right]
$$

从上式可得

$$
\dot {V} _ {e T z} ^ {T} = f _ {z} ^ {T} + \left(2 \omega_ {i e y} ^ {T} + \omega_ {e T y} ^ {T}\right) \cdot V _ {e T x} ^ {T} - \left(2 \omega_ {i e x} ^ {T} + \omega_ {e T x} ^ {T}\right) \cdot V _ {e T y} ^ {T} - g
$$

记

$$
a _ {z} ^ {T} = - \left(2 \omega_ {i e y} ^ {T} + \omega_ {e T y} ^ {T}\right) \cdot V _ {e T x} ^ {T} + \left(2 \omega_ {i e x} ^ {T} + \omega_ {e T x} ^ {T}\right) \cdot V _ {e T y} ^ {T}
$$

则

$$
\dot {V} _ {e T z} ^ {T} = f _ {z} ^ {T} - a _ {z} ^ {T} - g\tag{7.5.1}
$$

根据式(7.5. 1)和式(7. 2. 肘，可画出纯惯性高度通道的方块图，如图 7.5.1 所示。

![](images/751ebc6fceb906a4f0d6e6f1df66f1003fea6d42d2a4961461e0c9f0edfc1da5.jpg)  
7.5.1 纯惯性高度通道方块图

由图得

$$
\frac {h (s)}{f _ {z} (s)} = \frac {\frac {1}{s ^ {2}}}{1 + \frac {1}{s ^ {2}} \cdot \frac {2 g _ {0}}{R} (- 1)} = \frac {1}{s ^ {2} - \frac {2 g _ {0}}{R}}\tag{7.5.2}
$$

特征方程为 $s ^ { 2 } - \frac { 2 g _ { 0 } } { R } = 0$ ，特征根 $s _ { 1 } = \sqrt { \frac { 2 g _ { 0 } } { R } } , s _ { 2 } = - \sqrt { \frac { 2 g _ { 0 } } { R } }$ 可见高度通道是不稳定的。

假设加速叫量误差如偏酌， $\nabla _ { z }$ 独考虑由 $\nabla _ { z }$ 引起的高度误差记 $s _ { 0 } = \sqrt { \frac { 2 g _ { 0 } } { R } }$ ，由式 (7.5.2) ，得

$$
\delta h (s) = \frac {\nabla_ {z}}{s} \cdot \frac {1}{(s - s _ {0}) (s + s _ {0})} = \frac {\nabla_ {z}}{2 s _ {0} ^ {2}} \left(\frac {1}{s - s _ {0}} + \frac {1}{s + s _ {0}} - \frac {2}{s}\right)
$$

$$
\delta h (t) = \frac {\nabla_ {z}}{2 s _ {0} ^ {2}} \left(\mathrm{e} ^ {s _ {0} t} + \mathrm{e} ^ {- s _ {0} t} - 2\right) = \frac {R \nabla_ {z}}{4 g _ {0}} \left(\mathrm{e} ^ {\sqrt {\frac {2 g _ {0}}{R} t}} + \mathrm{e} ^ {- \sqrt {\frac {2 g _ {0}}{R} t}} - 2\right)
$$

设 $\nabla _ { z } = 1 \times 1 0 ^ { - 4 } g$ ，取 $g _ { 0 } { = } 9 7 8 . \ 0 4 9 \mathrm { c m / s ^ { 2 } } , R { = } 6 3 7 1 \ 0 0 0 \mathrm { m }$ ，则

$$
\begin{array}{r l} t _ {1} = 1 0 0 \mathrm{s}, & \delta h (t _ {1}) = 5 \mathrm{m}; \quad t _ {2} = 1 0 0 0 \mathrm{s}, \quad \delta h (t _ {2}) = 6 3 0 \mathrm{m}; \\ & t _ {3} = 3 6 0 0 \mathrm{s}, \quad \delta h (t _ {3}) = 8 7 3 8 1 \mathrm{m} \end{array}
$$

引起纯惯性高度通道发散的根本原因是系统无阻尼，使系统出现正特征根。为此，常引人其他系统提供的高度信息(常采用气压高度或无线电高度信息)使惯性高度通道具有阻尼，这可通过两种途径实现:①采用回路反馈法，如图 7.5.2 所示，此时系统阻尼系数是固定不变的;②采用卡尔曼滤波法，此时系统阻尼系数是时变的，根据外来高度参考信息误差特征实时确定，读者可参阅关于卡尔曼滤披及组合导航的诸多参考文献(秦永元等，1998) ，此处由于篇幅所限不作介绍。

![](images/d186de14b8dadc541787764541744821f30a8e881f6cdbbb6abcdaae1cd1b8e8.jpg)  
7.5.2 惯性高度通道的二阶阻尼回路

7.5.2 为惯性高度通道的二阶阻尼回路，由图得

$$
\begin{array}{c} s V _ {e T z} ^ {T} = f _ {z} ^ {T} - K _ {2} (h - h _ {r}) - \left(g _ {0} - \frac {2 g _ {0}}{R} h\right) - a _ {z} ^ {T} \\ s h = V _ {e T z} ^ {T} - K _ {1} (h - h _ {r}) \end{array}
$$

即

$$
s V _ {e T z} ^ {T} + \left(K _ {2} - \frac {2 g _ {0}}{R}\right) h = f _ {z} ^ {T} + K _ {2} h _ {r} - g _ {0} - a _ {z} ^ {T}
$$

$$
- V _ {e T z} ^ {T} + (s + K _ {1}) h = K _ {1} h _ {r}
$$

将上述两式写成矩阵形式：

$$
\left[ \begin{array}{c c} s & K _ {2} - \frac {2 g _ {0}}{R} \\ - 1 & s + K _ {1} \end{array} \right] \left[ \begin{array}{c} V _ {e T z} ^ {T} \\ h \end{array} \right] = \left[ \begin{array}{c} f _ {z} ^ {T} + K _ {2} h _ {r} - g _ {0} - a _ {z} ^ {T} \\ K _ {1} h _ {r} \end{array} \right]
$$

系统的特征多项式为

$$
\Delta (s) = \left| \begin{array}{c c} s & K _ {2} - \frac {2 g _ {0}}{R} \\ - 1 & s + K _ {1} \end{array} \right| = s ^ {2} + K _ {1} s + K _ {2} - \frac {2 g _ {0}}{R}
$$

对照二阶系统特征多项式的标准形式 $s ^ { 2 } + 2 \xi \omega _ { n } s + \omega _ { n } ^ { 2 }$ ，得

$$
\omega_ {n} = \sqrt {K _ {2} - \frac {2 g _ {0}}{R}}\tag{7.5.3a}
$$

$$
\xi = \frac {K _ {1}}{2 \sqrt {K _ {2} - \frac {2 g _ {0}}{R}}}\tag{7.5.3b}
$$

若要获得更高的阻尼特性，可采用三阶阻尼回路，如图 7.5.3 所示。

![](images/7d19c86bb1c364d1a61b1840bb79141a8b5dfdc4eb759ab54e4d85425986de74.jpg)  
7. 5. 3 惯性高度通道的三阶阻尼回路

由图得

$$
\begin{array}{c} s h = V _ {e T z} ^ {T} - K _ {1} (h + K _ {4} V _ {e T z} ^ {T} - h _ {r}) \\ s V _ {e T z} ^ {T} = f _ {z} ^ {T} - \left(g _ {0} - \frac {2 g _ {0}}{R} h\right) - \left[ X + K _ {2} (h - h _ {r} + K _ {4} V _ {e T z} ^ {T}) \right] - a _ {z} ^ {T} \\ s X = K _ {3} (h + K _ {4} V _ {e T z} ^ {T} - h _ {r}) \end{array}
$$

即

$$
\begin{array}{r l} & (s + K _ {1}) h + (K _ {1} K _ {4} - 1) V _ {e T z} ^ {T} = K _ {1} h _ {r} \\ & \left(K _ {2} - \frac {2 g _ {0}}{R}\right) h + (s + K _ {2} K _ {4}) V _ {e T z} ^ {T} + X = f _ {z} ^ {T} - g _ {0} + K _ {2} h _ {r} - a _ {z} ^ {T} \\ & - K _ {3} h - K _ {3} K _ {4} V _ {e T z} ^ {T} + s X = - K _ {3} h _ {r} \end{array}
$$

写成矩阵形式

$$
\left[ \begin{array}{c c c} s + K _ {1} & K _ {1} K _ {4} - 1 & 0 \\ K _ {2} - \frac {2 g _ {0}}{R} & s + K _ {2} K _ {4} & 1 \\ - K _ {3} & - K _ {3} K _ {4} & s \end{array} \right] \left[ \begin{array}{c} h \\ V _ {e T z} ^ {T} \\ X \end{array} \right] = \left[ \begin{array}{c} K _ {1} h _ {r} \\ f _ {z} ^ {T} - g _ {0} + K _ {2} h _ {r} - a _ {z} ^ {T} \\ - K _ {3} h _ {r} \end{array} \right]
$$

系统的特征多项式为

$$
\begin{array}{r l} \Delta (s) & = \left| \begin{array}{c c c} s + K _ {1} & K _ {1} K _ {4} - 1 & 0 \\ K _ {2} - \frac {2 g _ {0}}{R} & s + K _ {2} K _ {4} & 1 \\ - K _ {3} & - K _ {3} K _ {4} & s \end{array} \right| \\ & = s (s + K _ {1}) (s + K _ {2} K _ {4}) - K _ {3} (K _ {1} K _ {4} - 1) \\ & \quad - (K _ {1} K _ {4} - 1) \left(K _ {2} - \frac {2 g _ {0}}{R}\right) s + K _ {3} K _ {4} (s + K _ {1}) \\ & = s ^ {3} + (K _ {1} + K _ {2} K _ {4}) s ^ {2} + \left[ K _ {3} K _ {4} + K _ {2} + \frac {2 g _ {0}}{R} (K _ {1} K _ {4} - 1) \right] s + K _ {3} \end{array}
$$

参数 $K _ { 1 } \ 、 K _ { 2 } \ 、 K _ { 3 } \ 、 K _ { 4 }$ 按性能要求选取，一 $K _ { 4 }$ 取 O.5\~O. 缸，并按等根要求设计，即系统多项式为

$$
\Delta (s) = \left(s + \frac {1}{\tau}\right) ^ {3}
$$

根据比较系数法可确定出 $K _ { 1 } \ldots K _ { 2 } \ldots K _ { 3 }$ 为

$$
\begin{array}{l} K _ {1} = \frac {\frac {2 g _ {0}}{R} K _ {4} \tau^ {3} - 3 \tau^ {2} + 3 K _ {4} \tau - K _ {4} ^ {2}}{\left(\frac {2 g _ {0}}{R} K _ {4} ^ {2} - 1\right) \tau^ {3}} \\ K _ {2} = \frac {K _ {4} - 3 \tau + \frac {2 g _ {0}}{R} (3 \tau^ {2} K _ {4} - \tau^ {3})}{\left(\frac {2 g _ {0}}{R} K _ {4} ^ {2} - 1\right) \tau^ {3}} \\ K _ {3} = \frac {1}{\tau^ {3}} \end{array}\tag{7.5.4a}
$$

(7. 5. 4b)

(7. 5. 4c)

## 平台式惯导系统

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

将式 (8. 1. 1)代人式 (7.4.7) ，得

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

将式 (8. 1. 16) 代人上式，得

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

上式代人式 (8. 1. 19) ，得

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

将式 (8. 1. 27) 代人式 (8. 1. 26)后再代人式 (8. 1. 25).

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

将式 (8. 1. 31) 代人式 (8. 1. 28) ，得

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

将式(8. 1. 35)代人上式，得游移方位角的变化规律

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

记 $\pmb { \omega } _ { e T } ^ { T } = [ \pmb { \omega } _ { e T x } ^ { T } \quad \pmb { \omega } _ { e T y } ^ { T } \quad 0 ] ^ { \mathrm { T } }$ ，代人式 (8. 1. 20) ，得

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

上式代人式 (8.2. 5) ，得

$$
\boldsymbol {\omega} _ {c} ^ {*} = \boldsymbol {\omega} _ {c} + \boldsymbol {\psi} \times \boldsymbol {\omega} _ {c}\tag{8.2.6}
$$

这是加到陀螺力矩器的指令角速度，考虑到陀螺的刻度系数误差 $\Delta K _ { G }$ 及陀螺漂移 的影响，平台的实际进动角速度为

$$
\pmb {\omega} _ {i P} = (1 + \Delta K _ {G}) \pmb {\omega} _ {c} ^ {*} + \pmb {\varepsilon}
$$

将式(8. 2. 6)代人上式，并略去三阶小量，得

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

该式代人式 (8.2. 的，得

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

将式(8.2.10)代人上式，为简化解析分析，暂略去 $\Delta K _ { G }$ 的影响，并用圆球近似描述地球，则指北方位系统的姿态误差方程为

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

将式 (8.2.17) 、式 (8. 2. 18) 和式 (8. 1. 2)代人式 (8.2.12) ，得指北方位系统的平台姿态误差方程

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

上述诸关系式代人式(8.2.21) ，整理并略去关于误差的二阶小量，得

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

将式(8.2.25)代人式(8.2.24) ，得便于解析分析的速度误差方程

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

## 4. 静基座条件下的革统误差方程

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

## 5. 静基座条件下革统误差传播特性分析

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

对应的大圆定位误差为 O. 85n mile，接近 1n mile 所以根据陀螺精度可对惯导系统的定位精度作粗略估计 若陀螺漂移为队 01 /h，则惯导的定位误差大约为 $\mathrm { 1 n ~ m i l e / h }$ o

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

将式 (8.2.42)代人式 (8.2.41)

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

将式 (8.2.46)代人式 (8.2.45)再代人式 (8.2.4纱，得

$$
\begin{array}{l} \dot {\boldsymbol {\phi}} ^ {P} = (\boldsymbol {I} + [ \delta \boldsymbol {K} _ {G} ]) (\boldsymbol {\omega} _ {i T} ^ {T} + \delta \boldsymbol {\omega} _ {i T}) + \varepsilon^ {P} - \boldsymbol {C} _ {T} ^ {P} \boldsymbol {\omega} _ {i T} ^ {T} \\ = \boldsymbol {\omega} _ {i T} ^ {T} + [ \delta \boldsymbol {K} _ {G} ] \boldsymbol {\omega} _ {i T} ^ {T} + \delta \boldsymbol {\omega} _ {i T} + \boldsymbol {\varepsilon} ^ {P} - (\boldsymbol {I} + [ \boldsymbol {E} ]) \boldsymbol {\omega} _ {i T} ^ {T} \\ = [ \delta \boldsymbol {K} _ {G} ] \boldsymbol {\omega} _ {i T} ^ {T} + \delta \boldsymbol {\omega} _ {i T} - [ \boldsymbol {E} ] \boldsymbol {\omega} _ {i T} ^ {T} + \boldsymbol {\varepsilon} ^ {P} \end{array}
$$

将式 (8.2.40) 和式 (8.2.42) 代人上式，得

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

设对准地的纬度为 L(准确已知) ，忽略速度误差间的交叉搞合，则由式(8.2.27) ，得静基座条件下的系统误差方程为

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

平台经水平粗对准和方位粗对准后，水平失准角和方位失准角都可视为小角， $\phi _ { x }$ 弘和 $\phi _ { y }$ 间的交叉搞合可忽略，此时水平通道的误差方程可简化为

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

将式(8.3.9)和式(8. 3. 10)代人上式，并求稳态值，得

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

将式 (8. 3. 1)代人式 (8.3. 19a) ，得

$$
\phi_ {z} = \frac {\varepsilon_ {E} ^ {\mathrm{I}} - \delta \omega_ {x} ^ {\mathrm{I}}}{\omega_ {i e} \cos L} = - \frac {\delta \omega_ {x} ^ {\mathrm{I}} + \delta \omega_ {x} ^ {\mathrm{II}}}{\omega_ {i e} \cos L}\tag{8.3.23}
$$

若将式 (8.3. 22) 代人式 (8. 3. 20a) ，则可得 $\phi _ { z }$ 的另一个计算公式:

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

式中， $, \alpha _ { c }$ 为游移方位角 $\alpha$ 的估算值。上式代人式(8. 3. 28) ，得

$$
\left[ \begin{array}{c} \omega_ {i P x} ^ {P} \\ \omega_ {i P y} ^ {P} \\ \omega_ {i P z} ^ {P} \end{array} \right] = \left[ \begin{array}{c} \omega_ {i e} \cos L \sin \alpha_ {c} - \frac {\delta V _ {y}}{R} + \varepsilon_ {x} \\ \omega_ {i e} \cos L \cos \alpha_ {c} + \frac {\delta V _ {x}}{R} + \varepsilon_ {y} \\ \omega_ {i e} \sin L + \varepsilon_ {z} \end{array} \right]\tag{8.3.30}
$$

根据式 (8.3.25)

$$
\begin{array}{c} \boldsymbol {C} _ {W} ^ {P} = \left[ \begin{array}{c c c} 1 & 0 & - \phi_ {y} \\ 0 & 1 & \phi_ {x} \\ \phi_ {y} & - \phi_ {x} & 1 \end{array} \right] \\ \boldsymbol {\omega} _ {i P} ^ {W} = \boldsymbol {C} _ {P} ^ {W} \boldsymbol {\omega} _ {i P} ^ {P} \end{array}\tag{8.3.31}
$$

将式 (8.3.30) 和式 (8. 3. 31)代人上式，得

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

将式 (8.3.30) 、式 (8.3.27) 及式 (8.3.32) 代人上式，得

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

在静基座条件下， $V _ { x } ^ { W } = V _ { y } ^ { W } = V _ { z } ^ { W } = 0 , f _ { x } ^ { W } = f _ { y } ^ { W } = 0$ ，对准点纬度精确已知 $\delta L = 0 , f = - \thinspace g$ ，即$f _ { z } ^ { W } = g$ 略去通道间的交叉搞合，由式(8.2.43)可得用于对准的速度误差方程:

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

将 $\alpha { = } \alpha _ { c 1 } { - } \alpha$ 代人上式，其中 $\delta \alpha$ z是对 $\alpha _ { c 1 }$ 的修正量，并注意到 8α 是小角，则得

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

(8.3.48)和式 (8.3.49)即为水平陀螺的测漂计算公式。求得 $\hat { \varepsilon } _ { x }$ 和 $\hat { \varepsilon } _ { y }$ 后，代人(8. 3. 42) ，可获得更精确的游移方位角修正量 $\mathfrak { F } \alpha$ ，从而根据式(8. 3. 45)可更精确地确定出游移方位角 $\hat { \alpha } _ { c }$ 0

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

将式 (8.5.14)代人式 (8.5.13) 时，得

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

## 1. RNP指标为标准椭圆时惯导革统精度要求的确定

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

## 章捷联式惯导系统

## 9. 概述

9. 1. 为捷联惯导系统的原理简图，图中 为运载体的机体坐标系 为导航坐标系。陀螺和加速度计直接回联在运载体上，所以此类惯导称为捷联式惯导系统，简称捷联惯导。陀螺和加速度计分别用来测量运载体的角运动信息和线运动信息，机载计算机根据这些测量信息解算出运载体的航向、姿态、速度及位置。姿态阵解算相当于建立起数学平台 $\hat { \pmb { \omega } } _ { i n } ^ { b }$ 此相当于对数学平台作施矩的指令量，该指令量根据所选定的导航坐标系和解算得的速度和位置计算出。因此捷联式惯导系统与平台式惯导系统在本质上是相同的，但在系统的具体实现上却存在着明显的不同。

![](images/1ce402da8500b98f1d8a3fdb92a7539e0464bbc01487d7a84cb476673e86a076.jpg)  
9. 1. 捷联式惯导系统原理简图

## 1)陀螺动态范围要求不同

平台式惯导系统陀螺安装在平台台体上，陀螺感测台体偏离导航坐标系的偏差，平台通过稳定回路消除这种偏差，其作用是隔离掉运载体的角运动，使陀螺的工作环境不受运载体角运动的影响。同时，平台通过修正回路使陀螺按一定要求进动，控制平台跟踪导航坐标系的旋转运动。而导航坐标系的旋转仅由运载体相对地球的线运动及地球的自转引起，这些旋转角速度都十分微小，所以对陀螺的指令施矩电流是很小的。这就是说平台式惯导陀螺的动态范围可设计得较小。但捷联式惯导陀螺直接安装在运载体上，陀螺必须跟随运载体的角运动，施矩电流远比仅跟踪导航坐标系的施矩电流大，即捷联惯导所用陀螺的动态范围远比平台式惯导所采用的大。

## 2) 惯性器件的工作环境不同，惯性器件动态误差和静态误差的补偿要求也不同

在平台式惯导系统中，平台对运载体的角运动起隔离作用，安装在平台上的惯性器件只需对线加速度引起的静态误差作补偿。而捷联式惯导系统中的惯性器件除补偿静态误差外，还需要对角速度和角加速度引起的动态误差作补偿。因此必须在实验室条件下对捷联陀螺和加速度计

的动、静态误差系数作严格的测试和标定。

## 3) 捷联式惯导必须对三种算法误差作补偿

在实际系统中，为了降低捷联陀螺和加速度计的输出噪声对系统解算精度的影响，并且能够完全利用输出信息，陀螺和加速度计的输出全部采用增量形式，即加速度计输出为速度增量，陀螺输出为角增量(液浮陀螺或挠性陀螺及加速度计输出采用 I-F V-F转换成脉冲输出，激光陀螺本身就是脉冲输出〕。在此情况下，姿态解算和导航解算只能通过求解差分方程来完成，而当运载体存在线振动和角振动，或运载体做机动运动时，在姿态解算中会引起圆锥误差，在速度解算中会引起划桨误差，在位置解算中会引起涡卷误差。这些误差中，圆锥误差对捷联惯导精度的影响最严重，划桨误差次之，涡卷误差最轻，在相应算法中视需要作严格补偿。

## 4) 计算量不同

平台式惯导中平台以物理实体形式存在，平台模拟了导航坐标系，运载体的姿态角及航向角可直接从平台框架上拾取或仅通过少量计算获得。但在捷联惯导中，平台并不实体存在，而以数学平台形式存在，姿态角和航向角都必须通过计算获得，计算量庞大。

尽管在惯性器件、计算量等方面捷联惯导远比平台惯导要求苛刻，但由于省去了复杂的机电平台，结构简单、体积小、重量轻、成本低、维护简单、可靠性高、还可通过余度技术提高其容错能力。并且，由于诸如激光陀螺、光纤陀螺等固态惯性器件的出现，计算机技术的快速发展和计算理论的日益完善，捷联式惯导的优越性日趋显露。 20 世纪 90 年代，美国利登公司停止了平台式惯导 LTN-72 的批量生产，而转向激光捷联惯导 LTN-92 LTN-101 等的生产。在研发激光捷联惯导方面，霍尼韦尔公司和利登公司走在世界前列，产品广泛应用于波音 757 767 A320A330 等最新商用飞机上。

## 9.2 姿态更新计算的四元数算法

设由运载体的机体轴确定的坐标系为 惯导系统所采用的导航坐标系为 则由 系到系的坐标变换矩阵 称为运载体的姿态矩阵。姿态更新是指根据惯性器件的输出实时计算出矩阵.由于 系和 系均为直角坐标系，各轴之间始终保持直角，所以可将坐标系理解成刚体，当只研究两个坐标系间的角位置关系时，可对一个坐标系做平移，使其原点与另一个坐标系的原点重舍。因此，两坐标系间的空间角位置关系可理解成刚体的定点转动。从这一基本思想出发，可获得姿态更新的四元数算法及旋转矢量算法。本节详细介绍四元数更新算法。

## 9.2.1 四元敛

## 1.四元数定义

顾名思义，四元数是由四个元构成的数

$$
Q \left(q _ {0}, q _ {1}, q _ {2}, q _ {3}\right) = q _ {0} + q _ {1} \boldsymbol {i} + q _ {2} \boldsymbol {j} + q _ {3} \boldsymbol {k}\tag{9.2.1}
$$

式中 $, q _ { 0 } \ldots q _ { 1 } \ldots q _ { 2 } \ldots q _ { 3 }$ 是实数 i、 既是互相正交的单位向量，又是虚单位ν1 ，具体规定体现在如下四元数乘法关系中

$$
\left. \begin{array}{l l l} i \otimes i = - 1, & j \otimes j = - 1, & k \otimes k = - 1 \\ i \otimes j = k, & j \otimes k = i, & k \otimes i = j \\ j \otimes i = - k, & k \otimes j = - i, & i \otimes k = - j \end{array} \right\}\tag{9.2.2}
$$

式中，⑧表示四元数乘法。

上述关系可叙述为 相同单位向量做四元数乘时呈虚单位特性;相异单位向量作四元数乘时呈单位向量叉乘特性。所以四元数既可看做四维空间中的一个向量，又可看作一个超复数。

2. 四元数的表达方式

(1)矢量式

$$
\boldsymbol {Q} = q _ {0} + \boldsymbol {q}\tag{9.2.3}
$$

式中 $, q _ { 0 }$ 称四元数 的标量部分 $, \pmb q$ 称四元数 $\varrho$ 的矢量部分。对照式(9. 2. 1)，可看出 $\pmb q$ 是三维空间中的一个向量。

(2) 复数式

$$
\boldsymbol {Q} = q _ {0} + q _ {1} \boldsymbol {i} + q _ {2} \boldsymbol {j} + q _ {3} \boldsymbol {k}\tag{9.2.4}
$$

可视为一个超复数 的共辄复数记为

$$
\boldsymbol {Q} ^ {*} = q _ {0} - q _ {1} \boldsymbol {i} - q _ {2} \boldsymbol {j} - q _ {3} \boldsymbol {k}\tag{9.2.5}
$$

$\boldsymbol { Q } ^ { * }$ 称为 的共辄四元数。

。〉三角式

$$
\boldsymbol {Q} = \cos \frac {\theta}{2} + \boldsymbol {u} \sin \frac {\theta}{2}\tag{9.2.6}
$$

式中，()为实数 为单位向量。

(4) 指数式

$$
Q = \mathrm{e} ^ {u \frac {\theta}{2}}\tag{9.2.7}
$$

。和 同上。

(5) 矩阵式

$$
\boldsymbol {Q} = \left[ \begin{array}{c} q _ {0} \\ q _ {1} \\ q _ {2} \\ q _ {3} \end{array} \right]\tag{9.2.8}
$$

3. 四元数的大小 范数

四元数的大小用四元数的范数来表示

$$
\| Q \| = q _ {0} ^ {2} + q _ {1} ^ {2} + q _ {2} ^ {2} + q _ {3} ^ {2}\tag{9.2.9}
$$

I $Q \parallel = 1$ ，则 称为规范化四元数。

4. 四元数的运算 加减乘除

1)加法和减法

设

$$
\begin{array}{l} \boldsymbol {Q} = q _ {0} + q _ {1} \boldsymbol {i} + q _ {2} \boldsymbol {j} + q _ {3} \boldsymbol {k} \\ \boldsymbol {P} = p _ {0} + p _ {1} \boldsymbol {i} + p _ {2} \boldsymbol {j} + p _ {3} \boldsymbol {k} \end{array}
$$

则

$$
\boldsymbol {Q} \pm \boldsymbol {P} = \left(q _ {0} \pm p _ {0}\right) + \left(q _ {1} \pm p _ {1}\right) \boldsymbol {i} + \left(q _ {2} \pm p _ {2}\right) \boldsymbol {j} + \left(q _ {3} \pm p _ {3}\right) \boldsymbol {k}\tag{9.2.10}
$$

2) 乘法

$$
a Q = a q _ {0} + a q _ {1} \boldsymbol {i} + a q _ {2} \boldsymbol {j} + a q _ {3} \boldsymbol {k}\tag{9.2.11}
$$

式中 为标量。

$$
\begin{array}{r l} \boldsymbol {P} \otimes \boldsymbol {Q} & = (p _ {0} + p _ {1} \boldsymbol {i} + p _ {2} \boldsymbol {j} + p _ {3} \boldsymbol {k}) \otimes (q _ {0} + q _ {1} \boldsymbol {i} + q _ {2} \boldsymbol {j} + q _ {3} \boldsymbol {k}) \\ & = (p _ {0} q _ {0} - p _ {1} q _ {1} - p _ {2} q _ {2} - p _ {3} q _ {3}) + (p _ {0} q _ {1} + p _ {1} q _ {0} + p _ {2} q _ {3} - p _ {3} q _ {2}) \boldsymbol {i} \\ & \quad + (p _ {0} q _ {2} + p _ {2} q _ {0} + p _ {3} q _ {1} - p _ {1} q _ {3}) \boldsymbol {j} + (p _ {0} q _ {3} + p _ {3} q _ {0} + p _ {1} q _ {2} - p _ {2} q _ {1}) \boldsymbol {k} \\ & = r _ {0} + r _ {1} \boldsymbol {i} + r _ {2} \boldsymbol {j} + r _ {3} \boldsymbol {k} \end{array}\tag{9.2.12}
$$

上式写成矩阵形式

$$
\left[ \begin{array}{l} r _ {0} \\ r _ {1} \\ r _ {2} \\ r _ {3} \end{array} \right] = \left[ \begin{array}{c c c c} p _ {0} & - p _ {1} & - p _ {2} & - p _ {3} \\ p _ {1} & p _ {0} & - p _ {3} & p _ {2} \\ p _ {2} & p _ {3} & p _ {0} & - p _ {1} \\ p _ {3} & - p _ {2} & p _ {1} & p _ {0} \end{array} \right] \left[ \begin{array}{l} q _ {0} \\ q _ {1} \\ q _ {2} \\ q _ {3} \end{array} \right] = \boldsymbol {M} (\boldsymbol {P}) \boldsymbol {Q}\tag{9.2.13}
$$

或

$$
\left[ \begin{array}{l} r _ {0} \\ r _ {1} \\ r _ {2} \\ r _ {3} \end{array} \right] = \left[ \begin{array}{c c c c} q _ {0} & - q _ {1} & - q _ {2} & - q _ {3} \\ q _ {1} & q _ {0} & q _ {3} & - q _ {2} \\ q _ {2} & - q _ {3} & q _ {0} & q _ {1} \\ q _ {3} & q _ {2} & - q _ {1} & q _ {0} \end{array} \right] \left[ \begin{array}{l} p _ {0} \\ p _ {1} \\ p _ {2} \\ p _ {3} \end{array} \right] = \boldsymbol {M} ^ {\prime} (\boldsymbol {Q}) \boldsymbol {P}\tag{9.2.14}
$$

其中 $M ( P )$ 的构成形式为 第一列是四元数 本身，第一行是 的共辄四元数 $\pmb { P } ^ { * }$ 的转置，划去第一行和第一列余下的部分。

$$
\mathbf {V} _ {P} = \left[ \begin{array}{c c c} p _ {0} & - p _ {3} & p _ {2} \\ p _ {3} & p _ {0} & - p _ {1} \\ - p _ {2} & p _ {1} & p _ {0} \end{array} \right]\tag{9.2.15}
$$

称为 $M ( P )$ 的核，是由四元数 的元构成的反对称矩阵。同理 $M ^ { \prime } ( Q )$ 的核为

$$
\boldsymbol {V} _ {Q} ^ {\prime} = \left[ \begin{array}{c c c} q _ {0} & q _ {3} & - q _ {2} \\ - q _ {3} & q _ {0} & q _ {1} \\ q _ {2} & - q _ {1} & q _ {0} \end{array} \right]\tag{9.2.16}
$$

可见 $M ^ { \prime } ( Q )$ 与 $M ( \ Q )$ 构成相似，但核不同。

由式(9. 2. 13)和式(9.2. 14) ，得四元数乘法的矩阵表示形式

$$
\begin{array}{l} \boldsymbol {P} \otimes \boldsymbol {Q} = \boldsymbol {M} (\boldsymbol {P}) \boldsymbol {Q} \\ \boldsymbol {P} \otimes \boldsymbol {Q} = \boldsymbol {M} ^ {\prime} (\boldsymbol {Q}) \boldsymbol {P} \end{array}\tag{9.2.17a}
$$

(9. 2. 17b)

由于 $M ( P )$ 和 $M ^ { \prime } ( P )$ 的核不同，所以

$$
\boldsymbol {P} \otimes \boldsymbol {Q} = \boldsymbol {M} (\boldsymbol {P}) \boldsymbol {Q} \neq \boldsymbol {M} ^ {\prime} (\boldsymbol {P}) \boldsymbol {Q} = \boldsymbol {Q} \otimes \boldsymbol {P}
$$

上式说明四元数乘法不满足交换律。

四元数乘法满足分配律和结合律

$$
\boldsymbol {P} \otimes (\boldsymbol {Q} + \boldsymbol {R}) = \boldsymbol {P} \otimes \boldsymbol {Q} + \boldsymbol {P} \otimes \boldsymbol {R}\tag{9.2.18}
$$

$$
\boldsymbol {P} \otimes \boldsymbol {Q} \otimes \boldsymbol {R} = (\boldsymbol {P} \otimes \boldsymbol {Q}) \otimes \boldsymbol {R} = \boldsymbol {P} \otimes (\boldsymbol {Q} \otimes \boldsymbol {R})\tag{9.2.19}
$$

关于结合律的证明如下。

证明

$$
\boldsymbol {P} = \left[ \begin{array}{l} p _ {0} \\ p _ {1} \\ p _ {2} \\ p _ {3} \end{array} \right], \quad \boldsymbol {R} = \left[ \begin{array}{l} r _ {0} \\ r _ {1} \\ r _ {2} \\ r _ {3} \end{array} \right]
$$

则

$$
\begin{array}{r l} \boldsymbol {M} (\boldsymbol {P}) \boldsymbol {M} ^ {\prime} (\boldsymbol {R}) & = \left[ \begin{array}{c c c c} p _ {0} & - p _ {1} & - p _ {2} & - p _ {3} \\ p _ {1} & p _ {0} & - p _ {3} & p _ {2} \\ p _ {2} & p _ {3} & p _ {0} & - p _ {1} \\ p _ {3} & - p _ {2} & p _ {1} & p _ {0} \end{array} \right] \left[ \begin{array}{c c c c} r _ {0} & - r _ {1} & - r _ {2} & - r _ {3} \\ r _ {1} & r _ {0} & r _ {3} & - r _ {2} \\ r _ {2} & - r _ {3} & r _ {0} & r _ {1} \\ r _ {3} & r _ {2} & - r _ {1} & r _ {0} \end{array} \right] \\ & = \left[ \begin{array}{l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l} p _ {0} r _ {0} - p _ {1} r _ {1} - p _ {2} r _ {2} - p _ {3} r _ {3} & - p _ {0} r _ {1} - p _ {1} r _ {0} + p _ {2} r _ {3} - p _ {3} r _ {2} \\ p _ {0} r _ {1} + p _ {1} r _ {0} + p _ {2} r _ {3} - p _ {3} r _ {2} & p _ {0} r _ {0} - p _ {1} r _ {1} + p _ {2} r _ {2} + p _ {3} r _ {3} \\ p _ {0} r _ {2} - p _ {1} r _ {3} + p _ {2} r _ {0} + p _ {3} r _ {1} & - p _ {0} r _ {3} - p _ {1} r _ {2} - p _ {2} r _ {1} + p _ {3} r _ {0} \\ p _ {0} r _ {3} + p _ {1} r _ {2} - p _ {2} r _ {1} + p _ {3} r _ {0} & p _ {0} r _ {2} - p _ {1} r _ {3} - p _ {2} r _ {0} - p _ {3} r _ {1} \end{array} \right. \\ & \quad \left. \begin{array}{l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l l} - p _ {0} r _ {2} - p _ {1} r _ {3} - p _ {2} r _ {0} + p _ {3} r _ {1} & - p _ {0} r _ {3} + p _ {1} r _ {2} - p _ {2} r _ {1} - p _ {3} r _ {0} \\ p _ {0} r _ {3} - p _ {1} r _ {2} - p _ {2} r _ {1} - p _ {3} r _ {0} & - p _ {0} r _ {2} - p _ {1} r _ {3} + p _ {2} r _ {0} - p _ {3} r _ {1} \\ p _ {0} r _ {0} + p _ {1} r _ {1} - p _ {2} r _ {2} + p _ {3} r _ {3} & p _ {0} r _ {1} - p _ {1} r _ {0} - p _ {2} r _ {3} - p _ {3} r _ {2} \\ - p _ {0} r _ {1} + p _ {1} r _ {0} - p _ {2} r _ {3} - p _ {3} r _ {2} & p _ {0} r _ {0} + p _ {1} r _ {1} + p _ {2} r _ {2} - p _ {3} r _ {3}. \end{array} \right] = \boldsymbol {\mathbf M}. \end{array}
$$

$$
\boldsymbol {M} ^ {\prime} (\boldsymbol {R}) \boldsymbol {M} (\boldsymbol {P}) = \left[ \begin{array}{c c c c} r _ {0} & - r _ {1} & - r _ {2} & - r _ {3} \\ r _ {1} & r _ {0} & r _ {3} & - r _ {2} \\ r _ {2} & - r _ {3} & r _ {0} & r _ {1} \\ r _ {3} & r _ {2} & - r _ {1} & r _ {0} \end{array} \right] \left[ \begin{array}{c c c c} p _ {0} & - p _ {1} & - p _ {2} & - p _ {3} \\ p _ {1} & p _ {0} & - p _ {3} & p _ {2} \\ p _ {2} & p _ {3} & p _ {0} & - p _ {1} \\ p _ {3} & - p _ {2} & p _ {1} & p _ {0} \end{array} \right] = \boldsymbol {M}
$$

所以

$$
\boldsymbol {M} (\boldsymbol {P}) \boldsymbol {M} ^ {\prime} (\boldsymbol {R}) = \boldsymbol {M} ^ {\prime} (\boldsymbol {R}) \boldsymbol {M} (\boldsymbol {P})\tag{9.2.20}
$$

由于

$$
\begin{array}{l} \boldsymbol {P} \otimes (\boldsymbol {Q} \otimes \boldsymbol {R}) = \boldsymbol {P} \otimes [ \boldsymbol {M} ^ {\prime} (\boldsymbol {R}) \boldsymbol {Q} ] = \boldsymbol {M} (\boldsymbol {P}) \boldsymbol {M} ^ {\prime} (\boldsymbol {R}) \boldsymbol {Q} \\ (\boldsymbol {P} \otimes \boldsymbol {Q}) \otimes \boldsymbol {R} = [ \boldsymbol {M} (\boldsymbol {P}) \boldsymbol {Q} ] \otimes \boldsymbol {R} = \boldsymbol {M} ^ {\prime} (\boldsymbol {R}) \boldsymbol {M} (\boldsymbol {P}) \boldsymbol {Q} \end{array}
$$

根据式 (9.2.20) ，得

$$
P \otimes Q \otimes R = (P \otimes Q) \otimes R = P \otimes (Q \otimes R)
$$

此外还有

$$
(\boldsymbol {P} \otimes \boldsymbol {Q}) ^ {*} = \boldsymbol {Q} ^ {*} \otimes \boldsymbol {P} ^ {*}\tag{9.2.21}
$$

读者可自行证明。

3) 除法一一求逆

如果 $\begin{array} { r } { { P } \bigotimes \pmb { R } = 1 } \end{array}$ 则称 的逆，记为 ${ \pmb R } = { \pmb P } ^ { - 1 }$ 或称 的逆，记为 ${ \bf P } \mathrm { = } { \bf R } ^ { - 1 }$ C

根据范数定义和式(9.2. 12)

$$
\begin{array}{r l} \boldsymbol {P} \otimes \boldsymbol {P} ^ {*} & = (p _ {0} + p _ {1} \boldsymbol {i} + p _ {2} \boldsymbol {j} + p _ {3} \boldsymbol {k}) \otimes (p _ {0} - p _ {1} \boldsymbol {i} - p _ {2} \boldsymbol {j} - p _ {3} \boldsymbol {k}) \\ & = p _ {0} ^ {2} + p _ {1} ^ {2} + p _ {2} ^ {2} + p _ {3} ^ {2} = \| \boldsymbol {P} \| \end{array}
$$

所以， $. P \otimes { \frac { \pmb { P } ^ { * } } { \parallel \pmb { P } \parallel } } = 1$ ，根据上述关于逆的定义 $\frac { \pmb { P } ^ { * } } { \Vert \pmb { P } \Vert }$ 即为 的逆，即

$$
\boldsymbol {P} ^ {- 1} = \frac {\boldsymbol {P} ^ {*}}{\| \boldsymbol {P} \|}\tag{9.2.22}
$$

## 9.2.2 四元数与姿态阵间的关系

![](images/a37e50aeb6c2169ffbba38b98150fed89db8e0645a3de5481c29010155fca39d.jpg)

![](images/987865a84b0633227d31d05d2069a0ec245162a768e4c1999743b9a088d69072.jpg)  
9.2.1 刚体的等效旋转

设有参考坐标系 坐标轴为 $x _ { 0 } \ldots y _ { 0 } \ldots z _ { 0 }$ 口，坐标轴方向的单位向量为 ，坐标轴方向的单位向量为 $\pmb { i } _ { 0 } \ r , j _ { 0 } \ r , \pmb { k } _ { 0 }$ 。刚体相对 。刚体相对R系

由三重矢积计算公式:

做定点转动，定点为 $O _ { \circ }$ 。取坐标系 与刚体国联 系的坐标轴为 $x \cdot y \cdot z \cdot$ 坐标轴方向的单位向量为 。假设初始时刻$b$ 系与 $R$ 系重合。为了便于分析刚体的空间角位置，在刚体上取一点 转动点。至该点引位置向量OA 如图 9. 2.示。则该位置向量的空间位置实际上描述了刚体的空间角位置。

设刚体以 $\pmb { \omega } = _ { \omega _ { x } } \pmb { i } + \omega _ { y } \pmb { j } + \omega _ { z } \pmb { k }$ 相对 系旋转，初始时刻位置向量处于 $o \mathbf { A } = r .$ 经过时间 后位置向量处于 ${ \cal O } { \bf A } ^ { \prime } = r ^ { \prime }$ 。根据欧拉定理，仅考虑刚体在 时刻和 时刻的角位置时，刚体位置转到 $A ^ { \prime }$ 位置的转动可等效成绕瞬轴 u(单位向量)转过。角一次完成。这样，位置向量做圆锥运动 $A ^ { \prime }$ 位于同一圆上 $\boldsymbol { r } ^ { \prime }$ 位于同一圆锥面上。

下面分析 $\boldsymbol { r } ^ { \prime }$ 与 $r$ 的关系。在圆上取一点 $\angle A O ^ { \prime } B =$ $9 0 ^ { \circ }$ ，由图得

$$
\begin{array}{r l} \mathbf {O O} ^ {\prime} & = (r \cdot u) u \\ \mathbf {O} ^ {\prime} A & = r - \mathbf {O O} ^ {\prime} = r - (r \cdot u) u \\ \mathbf {O} ^ {\prime} B & = u \times \mathbf {O} ^ {\prime} A \\ & = u \times r - (r \cdot u) u \times u = u \times r \\ \mathbf {O} ^ {\prime} A ^ {\prime} & = \mathbf {O} ^ {\prime} A \cos \theta + \mathbf {O} ^ {\prime} B \sin \theta \\ & = r \cos \theta - (r \cdot u) u \cos \theta + u \times r \sin \theta \end{array}
$$

所以

$$
\begin{array}{r l} \boldsymbol {r} ^ {\prime} & = \boldsymbol {0 0} ^ {\prime} + \boldsymbol {0} ^ {\prime} \boldsymbol {A} ^ {\prime} \\ & = \boldsymbol {r} \cos \theta + (1 - \cos \theta) (\boldsymbol {r} \cdot \boldsymbol {u}) \boldsymbol {u} + \boldsymbol {u} \times \boldsymbol {r} \sin \theta \end{array}
$$

$$
\begin{array}{r l} \boldsymbol {u} \times (\boldsymbol {u} \times \boldsymbol {r}) & = \boldsymbol {u} (\boldsymbol {u} \cdot \boldsymbol {r}) - (\boldsymbol {u} \cdot \boldsymbol {u}) \boldsymbol {r} \\ & = (\boldsymbol {r} \cdot \boldsymbol {u}) \boldsymbol {u} - \boldsymbol {r} \end{array}
$$

即

$$
(\boldsymbol {r} \cdot \boldsymbol {u}) \boldsymbol {u} = \boldsymbol {r} + \boldsymbol {u} \times (\boldsymbol {u} \times \boldsymbol {r})
$$

所以

$$
\begin{array}{r l} \boldsymbol {r} ^ {\prime} & = \boldsymbol {r} \cos \theta + (1 - \cos \theta) [ \boldsymbol {r} + \boldsymbol {u} \times (\boldsymbol {u} \times \boldsymbol {r}) ] + \boldsymbol {u} \times \boldsymbol {r} \sin \theta \\ & = \boldsymbol {r} + \boldsymbol {u} \times \boldsymbol {r} \sin \theta + (1 - \cos \theta) \boldsymbol {u} \times (\boldsymbol {u} \times \boldsymbol {r}) \end{array}
$$

将上式向 系内投影:

$$
\boldsymbol {r} ^ {\prime R} = \boldsymbol {r} ^ {R} + (\boldsymbol {u} \times \boldsymbol {r}) ^ {R} \sin \theta + (1 - \cos \theta) [ \boldsymbol {u} \times (\boldsymbol {u} \times \boldsymbol {r}) ] ^ {R}
$$

记

$$
\boldsymbol {r} ^ {\prime R} = \left[ \begin{array}{l} r _ {x} ^ {\prime} \\ r _ {y} ^ {\prime} \\ r _ {z} ^ {\prime} \end{array} \right], \quad \boldsymbol {r} ^ {R} = \left[ \begin{array}{l} r _ {x} \\ r _ {y} \\ r _ {z} \end{array} \right], \quad \boldsymbol {u} ^ {R} = \left[ \begin{array}{l} l \\ m \\ n \end{array} \right]
$$

又根据叉乘关系表达式

$$
(\boldsymbol {u} \times \boldsymbol {r}) ^ {R} = \left[ \begin{array}{c c c} 0 & - n & m \\ n & 0 & - l \\ - m & l & 0 \end{array} \right] \left[ \begin{array}{c} r _ {x} \\ r _ {y} \\ r _ {z} \end{array} \right]
$$

记

$$
\boldsymbol {U} = \left[ \begin{array}{c c c} 0 & - n & m \\ n & 0 & - l \\ - m & l & 0 \end{array} \right]\tag{9.2.23}
$$

则

$$
(\boldsymbol {u} \times \boldsymbol {r}) ^ {R} = \boldsymbol {U r} ^ {R}, [ \boldsymbol {u} \times (\boldsymbol {u} \times \boldsymbol {r}) ] ^ {R} = \boldsymbol {U} \cdot \boldsymbol {U r} ^ {R}
$$

所以

$$
\begin{array}{r l} \boldsymbol {r} ^ {\prime R} & = \boldsymbol {r} ^ {R} + \boldsymbol {U r} ^ {R} \sin \theta + (1 - \cos \theta) \boldsymbol {U} \cdot \boldsymbol {U r} ^ {R} \\ & = (\boldsymbol {I} + 2 \boldsymbol {U} \sin \frac {\theta}{2} \cos \frac {\theta}{2} + 2 \sin^ {2} \frac {\theta}{2} \boldsymbol {U} \cdot \boldsymbol {U}) \boldsymbol {r} ^ {R} \end{array}\tag{9.2.24}
$$

<

$$
\boldsymbol {D} = \boldsymbol {I} + 2 \boldsymbol {U} \sin \frac {\theta}{2} \cos \frac {\theta}{2} + 2 \sin^ {2} \frac {\theta}{2} \boldsymbol {U} \cdot \boldsymbol {U}\tag{9.2.25}
$$

则式 (9.2. 24) 可写成

$$
\boldsymbol {r} ^ {\prime R} = \boldsymbol {D r} ^ {R}\tag{9.2.26}
$$

记初始时刻的刚体固联坐标系为 $b _ { 0 }$ ，由于初始时刻刚体固联坐标系与参考坐标系重合， 所以

$$
\boldsymbol {r} ^ {R} = \boldsymbol {r} ^ {b _ {0}}\tag{9.2.27}
$$

而在转动过程中，位置向量和 系都同刚体固联，所以位置向量和 系的相对角位置始终不变，即有

$$
\pmb {r} ^ {b _ {0}} = \pmb {r} ^ {\prime b}\tag{9.2.28}
$$

将式 (9.2.28) 代人式 (9. 2. 27).

$$
\boldsymbol {r} ^ {R} = \boldsymbol {r} ^ {\prime b}\tag{9.2.29}
$$

将式 (9. 2. 29)代人式 (9. 2. 26).

$$
\boldsymbol {r} ^ {\prime R} = \boldsymbol {D r} ^ {\prime b}
$$

该式说明 即为 系至 系的坐标变换矩阵，根据式(9.2.2日和式(9. 2. 23)

$$
\boldsymbol {C} _ {b} ^ {R} = \boldsymbol {D} = \boldsymbol {I} + 2 \boldsymbol {U} \sin \frac {\theta}{2} \cos \frac {\theta}{2} + 2 \sin^ {2} \frac {\theta}{2} \boldsymbol {U} \cdot \boldsymbol {U}\tag{9.2.30a}
$$

即

$$
\begin{array}{l} \mathbf {C} _ {b} ^ {R} = \left[ \begin{array}{l l l} 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{array} \right] + 2 \cos \frac {\theta}{2} \left[ \begin{array}{c c c} 0 & - n \sin \frac {\theta}{2} & m \sin \frac {\theta}{2} \\ n \sin \frac {\theta}{2} & 0 & - l \sin \frac {\theta}{2} \\ - m \sin \frac {\theta}{2} & l \sin \frac {\theta}{2} & 0 \end{array} \right] \\ + 2 \left[ \begin{array}{c c c} - (m ^ {2} + n ^ {2}) \sin^ {2} \frac {\theta}{2} & l m \sin^ {2} \frac {\theta}{2} & l n \sin^ {2} \frac {\theta}{2} \\ l m \sin^ {2} \frac {\theta}{2} & - (l ^ {2} + n ^ {2}) \sin^ {2} \frac {\theta}{2} & m n \sin^ {2} \frac {\theta}{2} \\ l n \sin^ {2} \frac {\theta}{2} & m n \sin^ {2} \frac {\theta}{2} & - (m ^ {2} + l ^ {2}) \sin^ {2} \frac {\theta}{2} \end{array} \right] \end{array}\tag{9.2.30b}
$$

$$
\left\{ \begin{array}{l} q _ {0} = \cos \frac {\theta}{2} \\ q _ {1} = l \sin \frac {\theta}{2} \\ q _ {2} = m \sin \frac {\theta}{2} \\ q _ {3} = n \sin \frac {\theta}{2} \end{array} \right.\tag{9.2.31}
$$

并以 $q _ { 0 } \ldots q _ { 1 } \ldots q _ { 2 } \ldots q _ { 3 }$ 构造四元数

$$
\begin{array}{r l} \boldsymbol {Q} & = q _ {0} + q _ {1} \boldsymbol {i} _ {0} + q _ {2} \boldsymbol {j} _ {0} + q _ {3} \boldsymbol {k} _ {0} = \cos \frac {\theta}{2} + (l \boldsymbol {i} _ {0} + m \boldsymbol {j} _ {0} + n \boldsymbol {k} _ {0}) \sin \frac {\theta}{2} \\ & = \cos \frac {\theta}{2} + \boldsymbol {u} ^ {R} \sin \frac {\theta}{2} \end{array}\tag{9.2.32}
$$

则可得如下结论

(1)四元数 $Q { = } \cos \frac { \theta } { 2 } { + } { \pm } u ^ { R } \sin \frac { \theta } { 2 }$ 描述了刚体的定点转动，即当只关心 系相对 系的角位置时，可认为 系是由 系经过无中间过程的一次性等效旋转形成的 包含了这种等效旋转的全部信息: $\mathbf { \Omega } _ { : \pmb { u } } { } ^ { R }$ "为旋转瞬轴和旋转方向，()为转过的角度。

(2) 四元数可确定出 系至 系的坐标变换矩阵。将式 (9. 2. 31) 代人式 (9.2. 30b) ，得

$$
\boldsymbol {C} _ {b} ^ {R} = \left[ \begin{array}{l l l} 1 - 2 \left(q _ {2} ^ {2} + q _ {3} ^ {2}\right) & 2 \left(q _ {1} q _ {2} - q _ {0} q _ {3}\right) & 2 \left(q _ {1} q _ {3} + q _ {0} q _ {2}\right) \\ 2 \left(q _ {1} q _ {2} + q _ {0} q _ {3}\right) & 1 - 2 \left(q _ {1} ^ {2} + q _ {3} ^ {2}\right) & 2 \left(q _ {2} q _ {3} - q _ {0} q _ {1}\right) \\ 2 \left(q _ {1} q _ {3} - q _ {0} q _ {2}\right) & 2 \left(q _ {2} q _ {3} + q _ {0} q _ {1}\right) & 1 - 2 \left(q _ {1} ^ {2} + q _ {2} ^ {2}\right) \end{array} \right]\tag{9.2.33}
$$

由于 $\mid \mid Q \mid \mid = q _ { 0 } ^ { 2 } + q _ { 1 } ^ { 2 } + q _ { 2 } ^ { 2 } + q _ { 3 } ^ { 2 } = \cos ^ { 2 } { \frac { \theta } { 2 } } + ( l ^ { 2 } + m ^ { 2 } + n ^ { 2 } ) \sin ^ { 2 } { \frac { \theta } { 2 } } = 1$ ，所以可进一步推得如下结论

(1)描述刚体旋转的四元数是规范化四元数。

(2) (9.2.33) 可写成

$$
\boldsymbol {C} _ {b} ^ {R} = \left[ \begin{array}{c c c} q _ {0} ^ {2} + q _ {1} ^ {2} - q _ {2} ^ {2} - q _ {3} ^ {2} & 2 (q _ {1} q _ {2} - q _ {0} q _ {3}) & 2 (q _ {1} q _ {3} + q _ {0} q _ {2}) \\ 2 (q _ {1} q _ {2} + q _ {0} q _ {3}) & q _ {0} ^ {2} - q _ {1} ^ {2} + q _ {2} ^ {2} - q _ {3} ^ {2} & 2 (q _ {2} q _ {3} - q _ {0} q _ {1}) \\ 2 (q _ {1} q _ {3} - q _ {0} q _ {2}) & 2 (q _ {2} q _ {3} + q _ {0} q _ {1}) & q _ {0} ^ {2} - q _ {1} ^ {2} - q _ {2} ^ {2} + q _ {3} ^ {2} \end{array} \right]\tag{9.2.34}
$$

(3) 如果将向量r $r ^ { R }$ <和 $r ^ { b }$ 看作零标量的四元数，则r $r ^ { R }$ <和 $r ^ { b }$ 间的变换关系可采用四元数乘法表示

$$
\boldsymbol {r} ^ {R} = \boldsymbol {Q} \otimes \boldsymbol {r} ^ {b} \otimes \boldsymbol {Q} ^ {*}\tag{9.2.35}
$$

该式称为坐标变换的四元数乘表示法，其中 系至 系的旋转四元数。证明如下

$$
\boldsymbol {Q} \otimes \boldsymbol {r} ^ {b} \otimes \boldsymbol {Q} ^ {*} = \boldsymbol {M} (\boldsymbol {Q}) \boldsymbol {M} ^ {\prime} (\boldsymbol {Q} ^ {*}) \left[ \begin{array}{c} 0 \\ \boldsymbol {r} ^ {b} \end{array} \right]
$$

$$
\begin{array}{r l} & = \left[ \begin{array}{c c c c} q _ {0} & - q _ {1} & - q _ {2} & - q _ {3} \\ q _ {1} & q _ {0} & - q _ {3} & q _ {2} \\ q _ {2} & q _ {3} & q _ {0} & - q _ {1} \\ q _ {3} & - q _ {2} & q _ {1} & q _ {0} \end{array} \right] \left[ \begin{array}{c c c c} q _ {0} & q _ {1} & q _ {2} & q _ {3} \\ - q _ {1} & q _ {0} & - q _ {3} & q _ {2} \\ - q _ {2} & q _ {3} & q _ {0} & - q _ {1} \\ - q _ {3} & - q _ {2} & q _ {1} & q _ {0} \end{array} \right] \left[ \begin{array}{l} 0 \\ r _ {x} ^ {b} \\ r _ {y} ^ {b} \\ r _ {z} ^ {b} \end{array} \right] \\ & = \left[ \begin{array}{c c c c} \times & 0 & 0 & 0 \\ \times & q _ {1} ^ {2} + q _ {0} ^ {2} - q _ {3} ^ {2} - q _ {2} ^ {2} & 2 (q _ {1} q _ {2} - q _ {0} q _ {3}) & 2 (q _ {1} q _ {3} + q _ {0} q _ {2}) \\ \times & 2 (q _ {1} q _ {2} + q _ {0} q _ {3}) & q _ {2} ^ {2} - q _ {3} ^ {2} + q _ {0} ^ {2} - q _ {1} ^ {2} & 2 (q _ {2} q _ {3} - q _ {0} q _ {1}) \\ \times & 2 (q _ {1} q _ {3} - q _ {0} q _ {2}) & 2 (q _ {2} q _ {3} + q _ {0} q _ {1}) & q _ {3} ^ {2} - q _ {2} ^ {2} - q _ {1} ^ {2} + q _ {0} ^ {2} \end{array} \right] \left[ \begin{array}{l} 0 \\ r _ {x} ^ {b} \\ r _ {y} ^ {b} \\ r _ {z} ^ {b} \end{array} \right] \end{array}
$$

对比式(9. 2. 34) ，知上式矩阵中右下角的 $3 \times 3$ 方块即为 $C _ { b } ^ { R }$ ，所以式 (9.2.35) 可写成

$$
\left[ \begin{array}{l} 0 \\ \boldsymbol {r} ^ {R} \end{array} \right] = \left[ \begin{array}{c c c c} \times & 0 & 0 & 0 \\ \times & & & \\ \times & & \boldsymbol {C} _ {b} ^ {R} & \\ \times & & & \end{array} \right] \left[ \begin{array}{l} 0 \\ \boldsymbol {r} ^ {b} \end{array} \right]
$$

即

$$
\boldsymbol {r} ^ {R} = \boldsymbol {C} _ {b} ^ {R} \boldsymbol {r} ^ {b}\tag{9.2.36}
$$

该式称为坐标变换的矩阵表示法。所以式(9.2. 35)与式(9.2.36)所示坐标变换的四元数乘表示法和矩阵表示法是等价的.

如果参考坐标系 是导航坐标系 tl体圃联坐标系 为机体坐标系，则坐标变换阵。就是姿态矩阵 $\mathbf { C } _ { b } ^ { n }$ ，而由姿态矩阵可计算出航向角和姿态角.

设运载体的航向角为'P(习惯上以北偏东为正) .俯仰角为()，横滚角为 $\gamma ,$ 取地理坐标系 $_ g$ 为导航坐标系，并规定 $\mathcal { X } _ { g } \ : , \ : \mathcal { Y } _ { g } \ : , \ : \mathcal { Z } _ { g }$ 的指向依次为东、北、天，则机体坐标系 $b$ 与导航坐标系 n(即地理坐标系 $g )$ 的关系如图1. 2. 所示。

由该图可得三次基本旋转对应的坐标变换阵为

$$
\boldsymbol {C} _ {n} ^ {\mathrm{l}} = \boldsymbol {C} _ {g} ^ {\mathrm{l}} = \left[ \begin{array}{c c c} \cos \Psi & - \sin \Psi & 0 \\ \sin \Psi & \cos \Psi & 0 \\ 0 & 0 & 1 \end{array} \right]\tag{9.2.37}
$$

$$
\boldsymbol {C} _ {1} ^ {b} = \boldsymbol {C} _ {2} ^ {b} \boldsymbol {C} _ {1} ^ {c} = \left[ \begin{array}{c c c} \cos \gamma & 0 & - \sin \gamma \\ 0 & 1 & 0 \\ \sin \gamma & 0 & \cos \gamma \end{array} \right] \left[ \begin{array}{c c c} 1 & 0 & 0 \\ 0 & \cos \theta & \sin \theta \\ 0 & - \sin \theta & \cos \theta \end{array} \right]
$$

$$
\begin{array}{r l} & = \left[ \begin{array}{c c c} \cos \gamma & \sin \theta \sin \gamma & - \cos \theta \sin \gamma \\ 0 & \cos \theta & \sin \theta \\ \sin \gamma & - \sin \theta \cos \gamma & \cos \theta \cos \gamma \end{array} \right] \\ & = \left[ \begin{array}{c c c} \cos \gamma \cos \Psi + \sin \gamma \sin \Psi \sin \theta & - \cos \gamma \sin \Psi + \sin \gamma \cos \Psi \sin \theta & - \sin \gamma \cos \theta \\ \sin \Psi \cos \theta & \cos \Psi \cos \theta & \sin \theta \\ \sin \gamma \cos \Psi - \cos \gamma \sin \Psi \sin \theta & - \sin \gamma \sin \Psi - \cos \gamma \cos \Psi \sin \theta & \cos \gamma \cos \theta \end{array} \right] \end{array} \tag {9.2}\tag{9.2.38}
$$

(9. 2. 39)

记 ${ \bf C } _ { b } ^ { n } = \left[ \begin{array} { l l l } { T _ { 1 1 } } & { T _ { 1 2 } } & { T _ { 1 3 } } \\ { T _ { 2 1 } } & { T _ { 2 2 } } & { T _ { 2 3 } } \\ { T _ { 3 1 } } & { T _ { 3 2 } } & { T _ { 3 3 } } \end{array} \right]$ ，由于 系至 系的旋转过程中坐标系始终保持直角坐标系，所以 CC

正交矩阵

$$
\boldsymbol {C} _ {n} ^ {b} = \left(\boldsymbol {C} _ {b} ^ {n}\right) ^ {\mathrm{T}} = \left[ \begin{array}{l l l} T _ {1 1} & T _ {2 1} & T _ {3 1} \\ T _ {1 2} & T _ {2 2} & T _ {3 2} \\ T _ {1 3} & T _ {2 3} & T _ {3 3} \end{array} \right]\tag{9.2.40}
$$

比较式 (9. 2. 39) 和式 (9.2.40) ，得

$$
\left\{ \begin{array}{l} \theta = \arcsin \left(T _ {3 2}\right) \\ \gamma_ {\text {主}} = \arctan \left(- \frac {T _ {3 1}}{T _ {3 3}}\right) \\ \Psi_ {\text {主}} = \arctan \left(\frac {T _ {1 2}}{T _ {2 2}}\right) \end{array} \right.\tag{9.2.41}
$$

航向角和横滚角的真值按表 9.2.1 和表 9.2.2 确定。

9.2.1 航向角.y真值表

<table><tr><td> $T_{22}$ </td><td> $T_{12}$ </td><td> $\Psi$ </td></tr><tr><td>→0</td><td>+</td><td>90°</td></tr><tr><td>→0</td><td>-</td><td>-90°</td></tr><tr><td>+</td><td>+</td><td> $\Psi_{\text{主}}$ </td></tr><tr><td>+</td><td>-</td><td> $\Psi_{\text{主}}$ </td></tr><tr><td>-</td><td>+</td><td> $\Psi_{\text{主}} + 180^{\circ}$ </td></tr><tr><td>-</td><td>-</td><td> $\Psi_{\text{主}} - 180^{\circ}$ </td></tr></table>

9.2.2 横滚角 真值表

<table><tr><td> $\gamma_{\text{主}}$ </td><td> $T_{33}$ </td><td> $\gamma$ </td></tr><tr><td>+</td><td rowspan="2">+</td><td rowspan="2"> $\gamma_{\text{主}}$ </td></tr><tr><td>-</td></tr><tr><td>+</td><td>-</td><td> $\gamma_{\text{主}} - 180^{\circ}$ </td></tr><tr><td>-</td><td>-</td><td> $\gamma_{\text{主}} + 180^{\circ}$ </td></tr></table>

上述分析说明 如果表征 系至 系的旋转四元数 已确定，则按式(9.2.33)或式(9.2.34)可计算出姿态阵 $C _ { b } ^ { n }$ ，再按式(9.2.41)和表 9.2.1 及表 9.2.2 可确定出运载体的航向角、俯仰角和横滚角，因此，四元数 包含了所有的姿态信息，捷联惯导中的姿态更新实质上是如何计算四元数 $Q .$

为使式(9. 2. 30a) 9.6 节的速度算法推导中便于应用，此处给出该式用等效旋转矢量叉乘的斜对称矩阵表示形式.

将 $t _ { k }$ 和 $t _ { k + 1 }$ 时刻的机体坐标系记为 b(川和 $b ( k + 1 )$ ， 当取 $b ( k )$ 为参考坐标系时，根据式 (9.2.30a)

式中

$$
\boldsymbol {C} _ {b (k + 1)} ^ {b (k)} = \boldsymbol {I} + \sin \Phi \boldsymbol {U} + (1 - \cos \Phi) \boldsymbol {U} \cdot \boldsymbol {U}
$$

$$
\boldsymbol {U} = \left[ \begin{array}{c c c} 0 & - n & m \\ n & 0 & - l \\ - m & l & 0 \end{array} \right]
$$

即为由等效旋转单位矢量

$$
\frac {\boldsymbol {\Phi}}{\Phi} = \left[ \begin{array}{c} l \\ m \\ n \end{array} \right]
$$

构造的叉乘反对称矩阵，式中， $\Phi { = } | \Phi |$ 。记

$$
\boldsymbol {U} = \left(\frac {\boldsymbol {\Phi}}{\Phi} \times\right)
$$

则

$$
\boldsymbol {C} _ {b (k + 1)} ^ {b (k)} = \boldsymbol {I} + \frac {\sin \Phi}{\Phi} (\boldsymbol {\Phi} \times) + \frac {1 - \cos \Phi}{\Phi^ {2}} (\boldsymbol {\Phi} \times) (\boldsymbol {\Phi} \times)\tag{9.2.42}
$$

## 9.2.3 四元数微分方程

记t和 $t + \Delta t$ 时刻的机体坐标系分别为 $b \left( t \right)$ 和 $b ( t + \Delta t )$ 参考坐标系取导航坐标系，由于导航坐标系旋转十分缓慢，在姿态更新周期(毫秒级〕内可视为无旋转，与时间无关，记导航坐标系为 $n _ { \mathrm { ~ c ~ } }$ 。则t时刻和 $t + \Delta t$ 时刻的姿态四元数可写成

$$
Q _ {n} ^ {b} (t) = Q _ {n} ^ {b (t)}\tag{9.2.43}
$$

$$
Q _ {n} ^ {b} (t + \Delta t) = Q _ {n} ^ {b (t + \Delta t)}\tag{9.2.44}
$$

此处. $Q _ { n } ^ { b }$ 是描述 系至 $b$ 系等效旋转的四元数。

向量 在机体坐标系 $b \left( t + \Delta t \right)$ 和导航坐标系 内的变换关系为

$$
\boldsymbol {r} ^ {n} = \boldsymbol {C} _ {b (t)} ^ {n} \boldsymbol {C} _ {b (t + \Delta t)} ^ {b (t)} \boldsymbol {r} ^ {b (t + \Delta t)}
$$

根据式 (9. 2. 35). 上式可写成

$$
\boldsymbol {r} ^ {n} = \boldsymbol {Q} _ {n} ^ {b (t)} \otimes \left\{\boldsymbol {Q} _ {b (t)} ^ {b (t + \Delta t)} \otimes \boldsymbol {r} ^ {b (t + \Delta t)} \otimes \boldsymbol {Q} _ {b (t)} ^ {b (t + \Delta t) *} \right\} \boldsymbol {Q} _ {n} ^ {b (t) ^ {*}}
$$

根据四元数乘法结合律和式(9. 2. 21).上式可写成

$$
\boldsymbol {r} ^ {n} = \left[ \boldsymbol {Q} _ {n} ^ {b (t)} \otimes \boldsymbol {Q} _ {b (t)} ^ {b (t + \Delta t)} \right] \otimes \boldsymbol {r} ^ {b (t + \Delta t)} \otimes \left[ \boldsymbol {Q} _ {n} ^ {b (t)} \otimes \boldsymbol {Q} _ {b (t)} ^ {b (t + \Delta t)} \right] *
$$

从该式得

$$
\boldsymbol {Q} _ {n} ^ {b} (t + \Delta t) = \boldsymbol {Q} _ {n} ^ {b (t)} \otimes \boldsymbol {Q} _ {b (t)} ^ {b (t + \Delta t)} = \boldsymbol {Q} _ {n} ^ {b} (t) \otimes \boldsymbol {Q} _ {b (t)} ^ {b (t + \Delta t)}\tag{9.2.45}
$$

设机体坐标系 $[ t , t + \Delta t ]$ 内相对导航坐标系 的角速度为 $\omega _ { n b } ^ { b } = \omega \pmb { u } ^ { b }$ 系在 $[ t , t + \Delta t ]$ 内产生的角增量为 $\phi { = } \omega \Delta t$ .假设 $\Delta t$ 为小量，则角增量 $\phi$ 也为小量，相应四元数为

$$
\boldsymbol {Q} _ {b (t)} ^ {b (t + \Delta t)} = \cos \frac {\phi}{2} + \boldsymbol {u} ^ {b} \sin \frac {\phi}{2} \doteq 1 + \boldsymbol {u} ^ {b} \frac {\omega \Delta t}{2} = 1 + \frac {1}{2} \boldsymbol {\omega} _ {n b} ^ {b} \Delta t\tag{9.2.46}
$$

式中，土表示近似等于。上式代人式(9.2.45) .得

$$
\begin{array}{r l} \boldsymbol {Q} _ {n} ^ {b} (t + \Delta t) & = \boldsymbol {Q} _ {n} ^ {b} (t) \otimes \left(1 + \frac {1}{2} \boldsymbol {\omega} _ {n b} ^ {b} \Delta t\right) = \boldsymbol {Q} _ {n} ^ {b} (t) \otimes \left[ (1 + \boldsymbol {\theta}) + \left(0 + \frac {1}{2} \boldsymbol {\omega} _ {n b} ^ {b} \Delta t\right) \right] \\ & = \boldsymbol {Q} _ {n} ^ {b} (t) + \boldsymbol {Q} _ {n} ^ {b} (t) \otimes \left(0 + \frac {1}{2} \boldsymbol {\omega} _ {n b} ^ {b} \Delta t\right) \end{array}
$$

所以

$$
\dot {Q} _ {n} ^ {b} (t) = \lim _ {\Delta t \rightarrow 0} \frac {Q _ {n} ^ {b} (t + \Delta t) - Q _ {n} ^ {b} (t)}{\Delta t} = Q _ {n} ^ {b} (t) \otimes \left(0 + \frac {1}{2} \omega_ {n b} ^ {b}\right) = \frac {1}{2} Q _ {n} ^ {b} (t) \otimes \omega_ {n b} ^ {b}\tag{9.2.47}
$$

记

$$
\boldsymbol {\omega} _ {n b} ^ {b} = \left[ \begin{array}{c} \omega_ {x} \\ \omega_ {y} \\ \omega_ {z} \end{array} \right]
$$

则式(9.2.47)可以写成矩阵形式

$$
\frac {\mathrm{d} \boldsymbol {Q}}{\mathrm{d} t} = \frac {1}{2} \boldsymbol {M} ^ {\prime} \left(\boldsymbol {\omega} _ {n b} ^ {b}\right) \boldsymbol {Q}\tag{9.2.48}
$$

RP

$$
\left[ \begin{array}{l} \dot {q} _ {0} \\ \dot {q} _ {1} \\ \dot {q} _ {2} \\ \dot {q} _ {3} \end{array} \right] = \frac {1}{2} \left[ \begin{array}{c c c c} 0 & - \omega_ {x} & - \omega_ {y} & - \omega_ {z} \\ \omega_ {x} & 0 & \omega_ {z} & - \omega_ {y} \\ \omega_ {y} & - \omega_ {z} & 0 & \omega_ {x} \\ \omega_ {z} & \omega_ {y} & - \omega_ {x} & 0 \end{array} \right] \left[ \begin{array}{l} q _ {0} \\ q _ {1} \\ q _ {2} \\ q _ {3} \end{array} \right]\tag{9.2.49}
$$

式中， $, \pmb { \omega } _ { n b } ^ { b }$ 的获取按下式进行

$$
\pmb {\omega} _ {n b} ^ {b} = \pmb {\omega} _ {i b} ^ {b} - \pmb {C} _ {n} ^ {b} (\pmb {\omega} _ {i e} ^ {n} + \pmb {\omega} _ {e n} ^ {n})\tag{9.2.50}
$$

式中， $\mathbf { \omega } _ { i b } ^ { b }$ 且是捷联陀螺的输出(对机械转子陀螺必须经过动、静态误差的补偿) ; $: C _ { n } ^ { b }$ :由姿态更新的最新值确定 $\mathbf { ; } \pmb { \omega } _ { e n } ^ { n }$ 和1m!分别是位置速率和地球自转速率，对于导航坐标系取地理坐标系的情况

$$
\boldsymbol {\omega} _ {i e} ^ {n} + \boldsymbol {\omega} _ {e n} ^ {n} = \left[ \begin{array}{c} - \frac {V _ {N}}{R _ {M}} \\ \omega_ {i e} \cos L + \frac {V _ {E}}{R _ {N}} \\ \omega_ {i e} \sin L + \frac {V _ {E}}{R _ {N}} \tan L \end{array} \right]
$$

式中 . $V _ { N } , V _ { E } , L$ 为导航计算所得的最新值。

## 9.2.4 四元数微分方程的毕卡求解法

捷联陀螺的输出一般情况下是采样时间间隔内的角增量，为了避免噪声的微分放大，应直接用角增量来确定四元数，而不应该将角增量换算成角速度。毕卡算法就是由角增量计算四元数的常用算法。

## 1.定时采样增量法

此算法中，角增量对应的采样时间间隔是相同的。式(9.2.48)是关于 的齐次线性方程，解为

$$
\boldsymbol {Q} \left(t _ {k + 1}\right) = \mathrm{e} ^ {\frac {1}{2} \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {M} ^ {\prime} \left(\boldsymbol {\omega} _ {n b} ^ {b}\right) \mathrm{d} t} \cdot \boldsymbol {Q} \left(t _ {k}\right)\tag{9.2.51}
$$

.&.

$$
\Delta \boldsymbol {\Theta} = \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {M} ^ {\prime} \left(\boldsymbol {\omega} _ {n b} ^ {b}\right) \mathrm{d} t = \int_ {t _ {k}} ^ {t _ {k + 1}} \left[ \begin{array}{c c c c} 0 & - \omega_ {x} & - \omega_ {y} & - \omega_ {z} \\ \omega_ {x} & 0 & \omega_ {z} & - \omega_ {y} \\ \omega_ {y} & - \omega_ {z} & 0 & \omega_ {x} \\ \omega_ {z} & \omega_ {y} & - \omega_ {x} & 0 \end{array} \right] \mathrm{d} t \approx \left[ \begin{array}{c c c c} 0 & - \Delta \theta_ {x} & - \Delta \theta_ {y} & - \Delta \theta_ {z} \\ \Delta \theta_ {x} & 0 & \Delta \theta_ {z} & - \Delta \theta_ {y} \\ \Delta \theta_ {y} & - \Delta \theta_ {z} & 0 & \Delta \theta_ {x} \\ \Delta \theta_ {z} & \Delta \theta_ {y} & - \Delta \theta_ {x} & 0 \end{array} \right]
$$

式中 ， $\Delta \theta _ { x } \ 、 \Delta \theta _ { y } \ 、 \Delta \theta _ { z }$ 为 $x \cdot y \cdot z$ 陀螺在队 $[ \mathbf { \sigma } _ { t _ { k } } , \mathbf { \sigma } _ { t _ { k + 1 } }$ J采样时间间隔内的角增量(已经过位置速率及地球

自转速率的补偿)。

对式(9. 2. 51)作泰勒级数展开

$$
\boldsymbol {Q} \left(t _ {k + 1}\right) = \mathrm{e} ^ {\frac {1}{2} \Delta \boldsymbol {\Theta}} \cdot \boldsymbol {Q} \left(t _ {k}\right) = \left[ \boldsymbol {I} + \frac {\frac {1}{2} \Delta \boldsymbol {\Theta}}{1 !} + \frac {\left(\frac {1}{2} \Delta \boldsymbol {\Theta}\right) ^ {2}}{2 !} + \dots \right] \boldsymbol {Q} \left(t _ {k}\right)
$$

由于

$$
\begin{array}{r l} \Delta \boldsymbol {\Theta} ^ {2} & = \left[ \begin{array}{c c c c} 0 & - \Delta \theta_ {x} & - \Delta \theta_ {y} & - \Delta \theta_ {z} \\ \Delta \theta_ {x} & 0 & \Delta \theta_ {z} & - \Delta \theta_ {y} \\ \Delta \theta_ {y} & - \Delta \theta_ {z} & 0 & \Delta \theta_ {x} \\ \Delta \theta_ {z} & \Delta \theta_ {y} & - \Delta \theta_ {x} & 0 \end{array} \right] \left[ \begin{array}{c c c c} 0 & - \Delta \theta_ {x} & - \Delta \theta_ {y} & - \Delta \theta_ {z} \\ \Delta \theta_ {x} & 0 & \Delta \theta_ {z} & - \Delta \theta_ {y} \\ \Delta \theta_ {y} & - \Delta \theta_ {z} & 0 & {\Delta \theta_ {x}} \\ \Delta \theta_ {z} & \Delta \theta_ {y} & - \Delta \theta_ {x} & 0 \end{array} \right] \\ & = \left[ \begin{array}{c c c c} - \Delta \theta^ {2} & 0 & 0 & 0 \\ 0 & - \Delta \theta^ {2} & 0 & 0 \\ 0 & 0 & - \Delta \theta^ {2} & 0 \\ 0 & 0 & 0 & - \Delta \theta^ {2} \end{array} \right] = -   \Delta \theta^ {2}   I \end{array}
$$

式中 $\Delta \theta ^ { 2 } = \Delta \theta _ { x } ^ { 2 } + \Delta \theta _ { y } ^ { 2 } + \Delta \theta _ { z } ^ { 2 }$ ，所以

$$
\begin{array}{r l} & {\triangle \pmb {\Theta} ^ {3} = \triangle \pmb {\Theta} ^ {2} \cdot \triangle \pmb {\Theta} = - \triangle \theta^ {2} \triangle \pmb {\Theta}} \\ & {\triangle \pmb {\Theta} ^ {4} = \triangle \pmb {\Theta} ^ {2} \cdot \triangle \pmb {\Theta} ^ {2} = \triangle \theta^ {4} \pmb {I}} \\ & {\triangle \pmb {\Theta} ^ {5} = \triangle \pmb {\Theta} ^ {4} \cdot \triangle \pmb {\Theta} = \triangle \theta^ {4} \triangle \pmb {\Theta}} \\ & {\triangle \pmb {\Theta} ^ {6} = \triangle \pmb {\Theta} ^ {4} \cdot \triangle \pmb {\Theta} ^ {2} = - \triangle \theta^ {6} \pmb {I}} \end{array}
$$

$$
\begin{array}{r l} \boldsymbol {Q} (t _ {k + 1}) = & \left\{\boldsymbol {I} + \boldsymbol {I} \left[ \frac {\frac {\Delta \boldsymbol {\Theta}}{2}}{1 !} + \frac {- \left(\frac {\Delta \theta}{2}\right) ^ {2}}{2 !} + \frac {- \left(\frac {\Delta \theta}{2}\right) ^ {2} \frac {\Delta \boldsymbol {\Theta}}{2}}{3 !} + \frac {\left(\frac {\Delta \theta}{2}\right) ^ {4}}{4 !} \right. \right. \\ & \left. \left. + \frac {\left(\frac {\Delta \theta}{2}\right) ^ {4} \frac {\Delta \boldsymbol {\Theta}}{2}}{5 !} + \frac {- \left(\frac {\Delta \theta}{2}\right) ^ {6}}{6 !} + \dots \right] \right\} \boldsymbol {Q} (t _ {k}) \end{array}
$$

RP

$$
\begin{array}{r l} \boldsymbol {Q} (t _ {k + 1}) & = \left\{\boldsymbol {I} \left[ 1 - \frac {\left(\frac {\Delta \theta}{2}\right) ^ {2}}{2 !} + \frac {\left(\frac {\Delta \theta}{2}\right) ^ {4}}{4 !} - \frac {\left(\frac {\Delta \theta}{2}\right) ^ {6}}{6 !} + \dots \right] \right. \\ & \quad \left. + \frac {\Delta \boldsymbol {\Theta}}{2} \left[ \frac {\frac {\Delta \theta}{2}}{1 !} - \frac {\left(\frac {\Delta \theta}{2}\right) ^ {3}}{3 !} + \frac {\left(\frac {\Delta \theta}{2}\right) ^ {5}}{5 !} - \dots \right] \frac {1}{\frac {\Delta \theta}{2}} \right\} \boldsymbol {Q} (t _ {k}) \\ & = \left\{\boldsymbol {I} \cos \frac {\Delta \theta}{2} + \Delta \boldsymbol {\Theta} \frac {\sin \frac {\Delta \theta}{2}}{\Delta \theta} \right\} \boldsymbol {Q} (t _ {k}) \end{array}\tag{9.2.52}
$$

在实际解算过程中， $\cos { \frac { \Delta \theta } { 2 } }$ ".- $\sin { \frac { \Delta \theta } { 2 } }$ 必须按级数展开的有限项计算。对式(9.2.52)取有限项，得四元数的各阶近似算法。

一阶近似算法

$$
\boldsymbol {Q} (t _ {k + 1}) = \left(\boldsymbol {I} + \frac {\Delta \boldsymbol {\Theta}}{2}\right) \boldsymbol {Q} (t _ {k})\tag{9.2.53}
$$

二阶近似算法

$$
\boldsymbol {Q} \left(t _ {k + 1}\right) = \left[ \boldsymbol {I} \left(1 - \frac {\Delta \theta^ {2}}{8}\right) + \frac {\Delta \boldsymbol {\Theta}}{2} \right] \boldsymbol {Q} \left(t _ {k}\right)\tag{9.2.54}
$$

三阶近似算法

$$
\boldsymbol {Q} \left(t _ {k + 1}\right) = \left[ \boldsymbol {I} \left(1 - \frac {\Delta \theta^ {2}}{8}\right) + \left(\frac {1}{2} - \frac {\Delta \theta^ {2}}{4 8}\right) \Delta \boldsymbol {\Theta} \right] \boldsymbol {Q} \left(t _ {k}\right)\tag{9.2.55}
$$

四阶近似算法

$$
\boldsymbol {Q} \left(t _ {k + 1}\right) = \left[ \boldsymbol {I} \left(1 - \frac {\Delta \theta^ {2}}{8} + \frac {\Delta \theta^ {4}}{3 8 4}\right) + \left(\frac {1}{2} - \frac {\Delta \theta^ {2}}{4 8}\right) \Delta \boldsymbol {\Theta} \right] \boldsymbol {Q} \left(t _ {k}\right)\tag{9.2.56}
$$

## 2. 定量采样增量法

在定量采样增量法姿态更新计算中，只有当陀螺输出的角增量达到某一固定的增量值 $\Delta \theta _ { 0 }$ 时才作四元数更新。设 $x , y , z$ 陀螺的输出角增量达到 $\Delta \theta _ { 0 }$ 的时间分别为 $t _ { k + 1 } \ldots t _ { k + 2 } \ldots t _ { k + 3 }$ 且假设$t _ { k + 1 } { < } t _ { k + 2 } { < } t _ { k + 3 }$ 则根据式 (9.2.52)

$$
\boldsymbol {Q} \left(t _ {k + 1}\right) = \left(\boldsymbol {I} \cos \frac {\Delta \theta_ {0}}{2} + \Delta \boldsymbol {\Theta} _ {x} \frac {\sin \frac {\Delta \theta_ {0}}{2}}{\Delta \theta_ {0}}\right) \boldsymbol {Q} \left(t _ {k}\right)
$$

$$
\boldsymbol {Q} \left(t _ {k + 2}\right) = \left(\boldsymbol {I} \cos \frac {\Delta \theta_ {0}}{2} + \Delta \boldsymbol {\Theta} _ {y} \frac {\sin \frac {\Delta \theta_ {0}}{2}}{\Delta \theta_ {0}}\right) \boldsymbol {Q} \left(t _ {k + 1}\right)
$$

$$
\boldsymbol {Q} \left(t _ {k + 3}\right) = \left(\boldsymbol {I} \cos \frac {\Delta \theta_ {0}}{2} + \Delta \boldsymbol {\Theta} _ {z} \frac {\sin \frac {\Delta \theta_ {0}}{2}}{\Delta \theta_ {0}}\right) \boldsymbol {Q} \left(t _ {k + 2}\right)
$$

式中

$$
\Delta \boldsymbol {\Theta} _ {x} = \left[ \begin{array}{c c c c} 0 & - \Delta \theta_ {0} & 0 & 0 \\ \Delta \theta_ {0} & 0 & 0 & 0 \\ 0 & 0 & 0 & \Delta \theta_ {0} \\ 0 & 0 & - \Delta \theta_ {0} & 0 \end{array} \right]\tag{9.2.57}
$$

$$
\Delta \boldsymbol {\Theta} _ {y} = \left[ \begin{array}{c c c c} 0 & 0 & - \Delta \theta_ {0} & 0 \\ 0 & 0 & 0 & - \Delta \theta_ {0} \\ \Delta \theta_ {0} & 0 & 0 & 0 \\ 0 & \Delta \theta_ {0} & 0 & 0 \end{array} \right]\tag{9.2.58}
$$

$$
\Delta \boldsymbol {\Theta} _ {z} = \left[ \begin{array}{c c c c} 0 & 0 & 0 & - \Delta \theta_ {0} \\ 0 & 0 & \Delta \theta_ {0} & 0 \\ 0 & - \Delta \theta_ {0} & 0 & 0 \\ \Delta \theta_ {0} & 0 & 0 & 0 \end{array} \right]\tag{9.2.59}
$$

记

$$
\boldsymbol {U} _ {x} = \boldsymbol {I} \cos \frac {\Delta \theta_ {0}}{2} + \Delta \boldsymbol {\Theta} _ {x} \frac {\sin \frac {\Delta \theta_ {0}}{2}}{\Delta \theta_ {0}}\tag{9.2.60}
$$

$$
\boldsymbol {U} _ {y} = \boldsymbol {I} \cos \frac {\Delta \theta_ {0}}{2} + \Delta \boldsymbol {\Theta} _ {y} \frac {\sin \frac {\Delta \theta_ {0}}{2}}{\Delta \theta_ {0}}\tag{9.2.61}
$$

$$
\boldsymbol {U} _ {z} = \boldsymbol {I} \cos \frac {\Delta \theta_ {0}}{2} + \Delta \boldsymbol {\Theta} _ {z} \frac {\sin \frac {\Delta \theta_ {0}}{2}}{\Delta \theta_ {0}}\tag{9.2.62}
$$

由于 $\Delta \theta _ { 0 }$ 是固定值，所以Ll. $\Delta \Theta _ { x } , \Delta \Theta _ { y } , \Delta \Theta _ { z }$ .是固定阵 $U _ { x } , U _ { y } , U _ { z }$ 也是固定阵。因此定量采样增量法可归纳如下

(1)按式 (9.2.60) 、式 (9.2.6 1)、式 (9.2. 62) 预先计算 $U _ { x } , U _ { y } , U _ { z }$ ，并存储。

(2) 记下角增量达到 $\Delta \theta _ { 0 }$ 的陀螺 及时间 $t _ { k + 1 }$ '则更新四元数为

$$
\boldsymbol {Q} \left(t _ {k + 1}\right) = \boldsymbol {U} _ {j} \boldsymbol {Q} \left(t _ {k}\right) \quad (j = x, y, z; k = 0, 1, 2, \dots)
$$

显然在定量采样增量法四元数更新中，时间更新的间隔是不相等的，且是随机的。该法适用于角运动比较缓慢的运载体，如运输机、轰炸机、舰船等。

## 9.2.5 四元数的初值确定和规范化处理

四元数的初值 $Q ( 0 )$ 由捷联惯导的初始对准确定。设初始对准确定的姿态阵为c， $C _ { b } ^ { n } { = } [ T _ { i j } ]$ 根据式(9. 2. 34)及描述刚体旋转的四元数为规范化四元数的结论，有如下方程式成立

$$
\left\{ \begin{array}{l} q _ {0} ^ {2} + q _ {1} ^ {2} - q _ {2} ^ {2} - q _ {3} ^ {2} = T _ {1 1} \\ q _ {0} ^ {2} - q _ {1} ^ {2} + q _ {2} ^ {2} - q _ {3} ^ {2} = T _ {2 2} \\ q _ {0} ^ {2} - q _ {1} ^ {2} - q _ {2} ^ {2} + q _ {3} ^ {2} = T _ {3 3} \\ q _ {0} ^ {2} + q _ {1} ^ {2} + q _ {2} ^ {2} + q _ {3} ^ {2} = 1 \\ 2 \left(q _ {1} q _ {2} - q _ {0} q _ {3}\right) = T _ {1 2} \\ 2 \left(q _ {1} q _ {3} + q _ {0} q _ {2}\right) = T _ {1 3} \\ 2 \left(q _ {1} q _ {2} + q _ {0} q _ {3}\right) = T _ {2 1} \\ 2 \left(q _ {2} q _ {3} - q _ {0} q _ {1}\right) = T _ {2 3} \\ 2 \left(q _ {1} q _ {3} - q _ {0} q _ {2}\right) = T _ {3 1} \\ 2 \left(q _ {2} q _ {3} + q _ {1} q _ {0}\right) = T _ {3 2} \end{array} \right.
$$

从上述方程可解得

$$
\left\{ \begin{array}{l} \mid q _ {1} \mid = \frac {1}{2} \sqrt {1 + T _ {1 1} - T _ {2 2} - T _ {3 3}} \\ \mid q _ {2} \mid = \frac {1}{2} \sqrt {1 - T _ {1 1} + T _ {2 2} - T _ {3 3}} \\ \mid q _ {3} \mid = \frac {1}{2} \sqrt {1 - T _ {1 1} - T _ {2 2} + T _ {3 3}} \\ \mid q _ {0} \mid = \frac {1}{2} \sqrt {1 + T _ {1 1} + T _ {2 2} + T _ {3 3}} \\ \left\{ \begin{array}{l} 4 q _ {1} q _ {0} = T _ {3 2} - T _ {2 3} \\ 4 q _ {2} q _ {0} = T _ {1 3} - T _ {3 1} \\ 4 q _ {3} q _ {0} = T _ {2 1} - T _ {1 2} \end{array} \right. \end{array} \right.\tag{9.2.63}
$$

所以 $q _ { 0 } \ldots q _ { 1 } \ldots q _ { 2 } \ldots q _ { 3 }$ 的符号可按下式确定:

$$
\left\{ \begin{array}{l} \operatorname{sign} \left(q _ {1}\right) = \operatorname{sign} \left(q _ {0}\right) \left[ \operatorname{sign} \left(T _ {3 2} - T _ {2 3}\right) \right] \\ \operatorname{sign} \left(q _ {2}\right) = \operatorname{sign} \left(q _ {0}\right) \left[ \operatorname{sign} \left(T _ {1 3} - T _ {3 1}\right) \right] \\ \operatorname{sign} \left(q _ {3}\right) = \operatorname{sign} \left(q _ {0}\right) \left[ \operatorname{sign} \left(T _ {2 1} - T _ {1 2}\right) \right] \end{array} \right.\tag{9.2.64}
$$

上式中 $\mathrm { s i g n } ( q _ { 0 } )$ 可任选，原因分析如下。

设表征从角位置 $A _ { 0 }$ 。至角位置 $A _ { 1 }$ 的旋转四元数为

$$
\boldsymbol {Q} = \cos \frac {\theta}{2} + \boldsymbol {u} \sin \frac {\theta}{2}
$$

设

$$
Q _ {1} = - Q
$$

则 $Q _ { 1 }$ 四个元的符号与 $Q$ 四个元的符号相反，

$$
\boldsymbol {Q} _ {1} = - \cos \frac {\theta}{2} - \boldsymbol {u} \sin \frac {\theta}{2} = \cos \left(\pi - \frac {\theta}{2}\right) - \boldsymbol {u} \sin \left(\pi - \frac {\theta}{2}\right) = \cos \frac {2 \pi - \theta}{2} - \boldsymbol {u} \sin \frac {2 \pi - \theta}{2}
$$

![](images/429a250d211d67cdc4f18c0ff7eb362a7e9a54aca3dfaaeaf8de4e38337ba8bb.jpg)

上式说明 : $\varrho _ { \mathrm { l } }$ 表征的旋转是刚体绕 旋转 $2 \pi - \theta$ 角度，如图 9.2.2所示，从图中可看出，刚体从角位置 $A _ { 0 }$ 旋转至角位置 $A _ { 1 }$ 。由于四元数只表征从初始位置至终了位置的一次性等效旋转，而并不反映从初始位置至终了位置的中间过程，所以由 $Q _ { 1 }$ 和 $\varrho$ 表征的旋转关系是相同的。由式 (9.2.64)知，当 $q _ { 0 }$ 取反号时 $q _ { 1 } \ldots q _ { 2 } \ldots q _ { 3 }$ 也跟着取反号，相当于四元数取反号，而 $Q$ 和 $- \varrho$ 表征了相同的等效旋转，所以 $q _ { 0 }$ 的符号可任取。

9.2.2 仅考虑初始和终了角位置时的等效旋转

表征旋转的四元数应该是规范化四元数，即 $Q \parallel = 1$ ，但是由于计算误差等因素，计算过程中四元数会逐渐失去规范化特性，因此若

干次更新后， $\dot { \mathcal { L } }$ 须对四元数做规范化处理:

$$
q _ {i} = \frac {\hat {q} _ {i}}{\sqrt {\hat {q} _ {0} ^ {2} + \hat {q} _ {1} ^ {2} + \hat {q} _ {2} ^ {2} + \hat {q} _ {3} ^ {2}}}, \quad i = 0, 1, 2, 3\tag{9.2.65}
$$

式中 $, \hat { q } _ { 0 } , \hat { q } _ { 1 } , \hat { q } _ { 2 } , \hat { q } _ { 3 }$ 是四元数更新所得的值。

## 9. 3 姿态更新计算的等效旋转矢量算法

采用毕卡法求解四元数微分方程时使用了陀螺的角增量输出，角增量虽然微小，但不能视为无穷小，而刚体做有限转动时，刚体的空间角位置与旋转次序有关，即存在旋转的不可交换性误差。等效旋转矢量法在利用角增量计算等效旋转矢量时，对这种不可交换误差作了适当补偿，在姿态更新周期内包含的角增量子样数越多，补偿就越精确。就算法本质上来看，姿态更新计算的四元数毕卡算法实质上就是单子样旋转矢量算法。因此多子祥旋转矢量算法比四元数毕卡算法的精度高.并且旋转矢量算法可获得锥运动环境下的优化算法，因此特别适用于在高动态环境下工作.

## 9.3.1 旋转矢量与姿态四元数阔的关系

设 $t _ { k }$ 时刻的机体坐标系为b(剖，导航坐标系为 $n ( k ) , t _ { k + 1 }$ 时刻的机体坐标系为 b(k+l)航坐标系为 $n ( k { + } 1 )$ . b(k) $b ( k + 1 )$ 的旋转四元数为 q(h) k) $b ( k )$ 的旋转四元数为$Q ( t _ { k } ) , n ( k { + } 1 )$ 至 $b ( k { + } 1 )$ 的旋转四元数为 $\mathcal { Q } ( t _ { k + 1 } ) , n ( k )$ 至 $n ( k + 1 )$ 的旋转四元数为 $\pmb { p } ( h )$ 其中$h { = } t _ { k + 1 } { - } t _ { k }$ . Jj!tl

$$
\boldsymbol {r} ^ {n (k + 1)} = \boldsymbol {C} _ {b (k + 1)} ^ {n (k + 1)} \boldsymbol {r} ^ {b (k + 1)}\tag{9.3.1a}
$$

亦即

$$
\boldsymbol {r} ^ {n (k + 1)} = \boldsymbol {C} _ {n (k)} ^ {n (k + 1)} \boldsymbol {C} _ {b (k)} ^ {n (k)} \boldsymbol {C} _ {b (k + 1)} ^ {b (k)} \boldsymbol {r} ^ {b (k + 1)}\tag{9.3.1b}
$$

根据式(9. 2. 35)和式(9.2. 36)所示向量坐标变换的四元数乘表示法和变换矩阵表示法间的等价关系，式(9.3.1a)等价于如下四元数乘表达式

$$
\boldsymbol {r} ^ {n (k + 1)} = \boldsymbol {Q} \left(t _ {k + 1}\right) \otimes \boldsymbol {r} ^ {b (k + 1)} \otimes \boldsymbol {Q} ^ {*} \left(t _ {k + 1}\right)\tag{9.3.1c}
$$

(9.3.1b)等价于如下四元数乘表达式

$$
\boldsymbol {r} ^ {n (k + 1)} = \boldsymbol {p} ^ {*} (h) \otimes \left\{\boldsymbol {Q} \left(t _ {k}\right) \otimes \left[ \boldsymbol {q} (h) \otimes \boldsymbol {r} ^ {b (k + 1)} \otimes \boldsymbol {q} ^ {*} (h) \right] \otimes \boldsymbol {Q} ^ {*} \left(t _ {k}\right) \right\} \otimes \boldsymbol {p} (h)
$$

根据四元数乘法结合律和式(9. 2. 1)，上式可写成

$$
\boldsymbol {r} ^ {n (k + 1)} = \left[ \boldsymbol {p} ^ {*} (h) \otimes \boldsymbol {Q} \left(t _ {k}\right) \otimes \boldsymbol {q} (h) \right] \otimes \boldsymbol {r} ^ {b (k + 1)} \otimes \left[ \boldsymbol {p} ^ {*} (h) \otimes \boldsymbol {Q} \left(t _ {k}\right) \otimes \boldsymbol {q} (h) \right] ^ {*}\tag{9.3.1d}
$$

比较式 (9. 3. 1c) 和式 (9. 3. 1d) ，得

$$
\boldsymbol {Q} \left(t _ {k + 1}\right) = \boldsymbol {p} ^ {*} (h) \otimes \boldsymbol {Q} \left(t _ {k}\right) \otimes \boldsymbol {q} (h)\tag{9.3.2}
$$

在姿态更新周期 $h = t _ { k + 1 } - t _ { k }$ 内，导航坐标系的变化十分缓慢 $\pmb { p } ( h ) \approx 1 + \pmb { 0 }$ ，所以式 (9.3.2)可写成

$$
\boldsymbol {Q} (t _ {k + 1}) = \boldsymbol {Q} (t _ {k}) \otimes \boldsymbol {q} (h)\tag{9.3.3}
$$

式中

$$
\boldsymbol {q} (h) = \cos \frac {\Phi}{2} + \frac {\boldsymbol {\Phi}}{\Phi} \sin \frac {\Phi}{2}\tag{9.3.4}
$$

$\pmb { \phi }$ b(晶)至 $b ( k { + } 1 )$ 的等效旋转矢量， $\scriptstyle \Phi = \vert \Phi \vert$ 。为便于叙述，将 $Q ( t _ { k + 1 } )$ 和 $Q ( t _ { k } )$ 称为 $t _ { k + 1 }$ 时刻和$t _ { k }$ 时刻的姿态四元数 $\pmb q ( h )$ 称为[ $[ t _ { k } , t _ { k + 1 }$ J时间段内的姿态变化四元数。

姿态四元数理应按式(9. 3. 2)进行更新，但由于导航坐标系的旋转十分缓慢，并且姿态更新周期又十分短，所以实际解算过程中可按式(9.3.3)作姿态更新，但若干步后需要作适当的修正，具体方法分析如下。

设对姿态计算作修正计算的周期为

$$
t _ {j + 1} - t _ {j} = N h
$$

式中 为姿态更新周期， 为某一正整数。则根据式(9. 3. 3)

$$
\boldsymbol {Q} (t _ {j + 1}) = \boldsymbol {Q} [ t _ {j} + (N - 1) h ] \otimes \boldsymbol {q} (h)
$$

此姿态四元数所对应的姿态阵为 $C _ { b ( j + 1 ) } ^ { n ( j ) }$ '因此 $t _ { j + 1 }$ 时刻正确的姿态阵应为

$$
\boldsymbol {C} _ {b (j + 1)} ^ {n (j + 1)} = \boldsymbol {C} _ {n (j)} ^ {n (j + 1)} \boldsymbol {C} _ {b (j + 1)} ^ {n (j)}\tag{9.3.5}
$$

式中 $C _ { b ( j + 1 ) } ^ { n ( j ) }$ 根据 $Q ( t _ { j + 1 } )$ 确定，C: $C _ { n ( j ) } ^ { n ( j + 1 ) }$ )可按下法近似确定。

设 $t _ { j }$ 至 $t _ { j + 1 }$ 时刻运载体的经纬度增量分别为 $\Delta \lambda$ 和 $\Delta L$ L，并假设选取 Nh 时确保，; $\Delta \lambda$ \和 $\Delta L$ L均为微小角，则 $n ( j )$ 至 $n ( j + 1 )$ 的旋转矢量为

$$
\boldsymbol {\eta} ^ {n} = \left[ \begin{array}{c} - \Delta L \\ 0 \\ 0 \end{array} \right] + \boldsymbol {C} _ {e} ^ {n} \left[ \begin{array}{l} 0 \\ 0 \\ \Delta \lambda \end{array} \right] = \left[ \begin{array}{c} - \Delta L \\ \Delta \lambda \cos L \\ \Delta \lambda \sin L \end{array} \right]
$$

所以

$$
\boldsymbol {C} _ {n (j)} ^ {n (j + 1)} = \left[ \begin{array}{c c c} 1 & \Delta \lambda \sin L & - \Delta \lambda \cos L \\ - \Delta \lambda \sin L & 1 & - \Delta L \\ \Delta \lambda \cos L & \Delta L & 1 \end{array} \right]
$$

## 9.3.2 旋转矢量微分方程

对于 $t _ { k } { \leqslant } t { \leqslant } t _ { k + 1 }$ 根据式 (9.3.3) ，有

$$
\boldsymbol {Q} (t) = \boldsymbol {Q} (t _ {k}) \otimes \boldsymbol {q} (t - t _ {k})
$$

上式两边对时间求导

$$
\frac {\mathrm{d} \boldsymbol {Q} (t)}{\mathrm{d} t} = \boldsymbol {Q} \left(t _ {k}\right) \otimes \frac {\mathrm{d} \boldsymbol {q} \left(t - t _ {k}\right)}{\mathrm{d} t}
$$

根据式 (9.2.47)

$$
\frac {\mathrm{d} \boldsymbol {Q} (t)}{\mathrm{d} t} = \frac {1}{2} \boldsymbol {Q} (t) \otimes \boldsymbol {\omega} _ {n b} ^ {b} = \frac {1}{2} \boldsymbol {Q} (t _ {k}) \otimes \boldsymbol {q} (t - t _ {k}) \otimes \boldsymbol {\omega} _ {n b} ^ {b}
$$

比较上述两式，得

$$
\boldsymbol {Q} \left(t _ {k}\right) \otimes \frac {\mathrm{d} \boldsymbol {q}}{\mathrm{d} t} = \frac {1}{2} \boldsymbol {Q} \left(t _ {k}\right) \otimes \boldsymbol {q} \otimes \boldsymbol {\omega} _ {n b} ^ {b}
$$

两边左乘 $Q ^ { * } \left( t _ { k } \right)$ 注意到姿态四元数为规范化四元数，Il $Q ^ { * } \left( t _ { k } \right) \bigotimes Q ( t _ { k } ) = 1$ 所以

$$
\frac {\mathrm{d} \boldsymbol {q}}{\mathrm{d} t} = \frac {1}{2} \boldsymbol {q} \otimes \boldsymbol {\omega} _ {n b} ^ {b}\tag{9.3.6}
$$

此即姿态变化四元数满足的微分方程。

对应的旋转矢量为 $\pmb { \phi }$ ，即

$$
\boldsymbol {\Phi} = \boldsymbol {\Phi} \boldsymbol {u} ^ {b}\tag{9.3.7}
$$

则 $\pmb q$ 可写成

$$
\boldsymbol {q} = \cos \frac {\Phi}{2} + \boldsymbol {u} ^ {b} \sin \frac {\Phi}{2} = q _ {o} + \boldsymbol {q} _ {1 - 3}\tag{9.3.8}
$$

式中 $\mathrm { , } q _ { o } = \cos \frac { \varPhi } { 2 } , q _ { \mathrm { 1 - 3 } } = \mathbf { u } ^ { b } \sin \frac { \varPhi } { 2 }$ 这样，式(9.3 问写成

$$
\frac {\mathrm{d}}{\mathrm{d} t} \left[ \begin{array}{l} q _ {0} \\ \boldsymbol {q} _ {1 - 3} \end{array} \right] = \frac {1}{2} \boldsymbol {M} ^ {\prime} \left(\boldsymbol {\omega} _ {n b} ^ {b}\right) \boldsymbol {q} = \frac {1}{2} \left[ \begin{array}{c c} 0 & - \boldsymbol {\omega} _ {n b} ^ {b T} \\ \boldsymbol {\omega} _ {n b} ^ {b} & - \left(\boldsymbol {\omega} _ {n b} ^ {b} \times\right) \end{array} \right] \left[ \begin{array}{l} \cos \frac {\Phi}{2} \\ \boldsymbol {u} ^ {b} \sin \frac {\Phi}{2} \end{array} \right]
$$

即

$$
\dot {q} _ {0} = - \frac {1}{2} \sin \frac {\Phi}{2} \boldsymbol {\omega} _ {n b} ^ {b T} \boldsymbol {u} ^ {b}\tag{9.3.9a}
$$

$$
\dot {\boldsymbol {q}} _ {1 - 3} = \frac {1}{2} \cos \frac {\Phi}{2} \boldsymbol {\omega} _ {n b} ^ {b} - \frac {1}{2} \sin \frac {\Phi}{2} \left(\boldsymbol {\omega} _ {n b} ^ {b} \times\right) \boldsymbol {u} ^ {b}\tag{9.3.9b}
$$

(9. 3. 8)对时间求导，得

$$
\dot {q} _ {0} + \dot {\boldsymbol {q}} _ {1 - 3} = - \frac {\dot {\Phi}}{2} \sin \frac {\Phi}{2} + \boldsymbol {u} ^ {b} \sin \frac {\Phi}{2} + \boldsymbol {u} ^ {b} \frac {\dot {\Phi}}{2} \cos \frac {\Phi}{2}
$$

RP

$$
q _ {0} = - \frac {\dot {\varPhi}}{2} \sin \frac {\varPhi}{2}\tag{9.3.10a}
$$

$$
\dot {\boldsymbol {q}} _ {1 - 3} = \dot {\boldsymbol {u}} ^ {b} \sin \frac {\Phi}{2} + \boldsymbol {u} ^ {b} \frac {\dot {\Phi}}{2} \cos \frac {\Phi}{2}\tag{9.3.10b}
$$

比较式 (9. 3. 9) 和式 (9.3.10) ，得

$$
- \frac {\dot {\Phi}}{2} \sin \frac {\Phi}{2} = - \frac {1}{2} \sin \frac {\Phi}{2} \boldsymbol {\omega} _ {n b} ^ {b T} \boldsymbol {u} ^ {b}\tag{9.3.11a}
$$

$$
\dot {\boldsymbol {u}} ^ {b} \sin \frac {\Phi}{2} + \boldsymbol {u} ^ {b} \frac {\dot {\Phi}}{2} \cos \frac {\Phi}{2} = \frac {1}{2} \cos \frac {\Phi}{2} \boldsymbol {\omega} _ {n b} ^ {b} - \frac {1}{2} \sin \frac {\Phi}{2} (\boldsymbol {\omega} _ {n b} ^ {b} \times) \boldsymbol {u} ^ {b}\tag{9.3.11b}
$$

由式 (9. 3. 11)

$$
\dot {\boldsymbol {\Phi}} = \pmb {\omega} _ {n b} ^ {b T} \pmb {u} ^ {b}\tag{9.3.12}
$$

$$
\dot {\boldsymbol {u}} ^ {b} \sin \frac {\Phi}{2} = - \frac {1}{2} \sin \frac {\Phi}{2} \left(\boldsymbol {\omega} _ {n b} ^ {b} \times\right) \boldsymbol {u} ^ {b} + \frac {1}{2} \cos \frac {\Phi}{2} \left(\boldsymbol {\omega} _ {n b} ^ {b} - \dot {\Phi} \boldsymbol {u} ^ {b}\right)\tag{9.3.13}
$$

利用式 (9. 3. 12)关系，式 (9. 3. 13)可写成

$$
\dot {\boldsymbol {u}} ^ {b} = - \frac {1}{2} \left(\boldsymbol {\omega} _ {n b} ^ {b} \times\right) \boldsymbol {u} ^ {b} + \frac {1}{2} \cot \frac {\Phi}{2} \left[ \boldsymbol {\omega} _ {n b} ^ {b} - \left(\boldsymbol {\omega} _ {n b} ^ {b T} \boldsymbol {u} ^ {b}\right) \boldsymbol {u} ^ {b} \right]\tag{9.3.14}
$$

根据三重矢积公式

$$
\begin{array}{c} \boldsymbol {a} \times (\boldsymbol {b} \times \boldsymbol {c}) = (\boldsymbol {a} \cdot \boldsymbol {c}) \boldsymbol {b} - (\boldsymbol {a} \cdot \boldsymbol {b}) \boldsymbol {c} \\ \boldsymbol {u} ^ {b} \times (\boldsymbol {u} ^ {b} \times \boldsymbol {\omega} _ {n b} ^ {b}) = (\boldsymbol {u} ^ {b} \cdot \boldsymbol {\omega} _ {n b} ^ {b}) \boldsymbol {u} ^ {b} - (\boldsymbol {u} ^ {b} \cdot \boldsymbol {u} ^ {b}) \boldsymbol {\omega} _ {n b} ^ {b} = (\boldsymbol {\omega} _ {n b} ^ {b T} \boldsymbol {u} ^ {b}) \boldsymbol {u} ^ {b} - \boldsymbol {\omega} _ {n b} ^ {b} \end{array}
$$

所以

$$
\left(\boldsymbol {\omega} _ {n b} ^ {b T} \boldsymbol {u} ^ {b}\right) \boldsymbol {u} ^ {b} = \boldsymbol {\omega} _ {n b} ^ {b} + \boldsymbol {u} ^ {b} \times \left(\boldsymbol {u} ^ {b} \times \boldsymbol {\omega} _ {n b} ^ {b}\right)\tag{9.3.15}
$$

将式 (9.3.1日代人式 (9.3. 14) ，得

$$
\dot {\boldsymbol {u}} ^ {b} = \frac {1}{2} \boldsymbol {u} ^ {b} \times \boldsymbol {\omega} _ {n b} ^ {b} - \frac {1}{2} \cot \frac {\Phi}{2} \boldsymbol {u} ^ {b} \times (\boldsymbol {u} ^ {b} \times \boldsymbol {\omega} _ {n b} ^ {b})\tag{9.3.16}
$$

(9.3. 7)两边对时间求导，得

$$
\dot {\boldsymbol {\Phi}} = \dot {\boldsymbol {\Phi}} \boldsymbol {u} ^ {b} + \boldsymbol {\Phi} \dot {\boldsymbol {u}} ^ {b}\tag{9.3.17}
$$

将式 (9. 3. 12) 和式 (9.3.16) 代人式 (9.3.17) ，得

$$
\dot {\boldsymbol {\Phi}} = \left(\boldsymbol {\omega} _ {n b} ^ {b T} \boldsymbol {u} ^ {b}\right) \boldsymbol {u} ^ {b} + \frac {\Phi}{2} \boldsymbol {u} ^ {b} \times \boldsymbol {\omega} _ {n b} ^ {b} - \frac {\Phi}{2} \cot \frac {\Phi}{2} \boldsymbol {u} ^ {b} \times \left(\boldsymbol {u} ^ {b} \times \boldsymbol {\omega} _ {n b} ^ {b}\right)\tag{9.3.18}
$$

将式 (9. 3. 7) 和式 (9. 3. 15) 代人式 (9. 3. 18) ，得

$$
\begin{array}{r l} \dot {\boldsymbol {\Phi}} & = \boldsymbol {\omega} _ {n b} ^ {b} + \boldsymbol {u} ^ {b} \times (\boldsymbol {u} ^ {b} \times \boldsymbol {\omega} _ {n b} ^ {b}) + \frac {\Phi}{2} \boldsymbol {u} ^ {b} \times \boldsymbol {\omega} _ {n b} ^ {b} - \frac {\Phi}{2} \cot \frac {\Phi}{2} \boldsymbol {u} ^ {b} \times (\boldsymbol {u} ^ {b} \times \boldsymbol {\omega} _ {n b} ^ {b}) \\ & = \boldsymbol {\omega} _ {n b} ^ {b} + \frac {1}{2} \boldsymbol {\Phi} \times \boldsymbol {\omega} _ {n b} ^ {b} + \frac {1}{\Phi^ {2}} \left(1 - \frac {\Phi}{2} \cot \frac {\Phi}{2}\right) \times \boldsymbol {\Phi} \times (\boldsymbol {\Phi} \times \boldsymbol {\omega} _ {n b} ^ {b}) \end{array}\tag{9.3.19}
$$

(9.3. 19) 即为旋转矢量微分方程，即 Bortz 方程 (Bortz J E , 1971; Miller R B, 1983; SavageP G,1998;Esmat Bekir 2007)。对该式中的三角函数作级数展开，还可得到更为简洁的近似方程。

由于

$$
\begin{array}{c} \frac {\Phi}{2} \cot \frac {\Phi}{2} = \frac {\Phi}{2} \left[ \frac {2}{\Phi} - \frac {1}{3} \frac {\Phi}{2} - \frac {1}{4 5} \left(\frac {\Phi}{2}\right) ^ {3} - \frac {2}{9 4 5} \left(\frac {\Phi}{2}\right) ^ {5} - \frac {1}{4 7 2 5} \left(\frac {\Phi}{2}\right) ^ {7} - \dots \right] \\ = 1 - \frac {\Phi^ {2}}{1 2} - \frac {\Phi^ {4}}{7 2 0} - \frac {\Phi^ {6}}{3 0 2 4 0} - \frac {\Phi^ {8}}{1 2 0 9 6 0 0} - \dots \end{array}
$$

上式代人式(9. 3. 19) ，由于姿态更新周期一般都很短， $\varPhi$ 很小， $\varPhi$ 的高次项可略去不计，得工程上常用的近似方程

$$
\dot {\boldsymbol {\Phi}} = \boldsymbol {\omega} _ {n b} ^ {b} + \frac {1}{2} \boldsymbol {\Phi} \times \boldsymbol {\omega} _ {n b} ^ {b} + \frac {1}{1 2} \boldsymbol {\Phi} \times (\boldsymbol {\Phi} \times \boldsymbol {\omega} _ {n b} ^ {b})\tag{9.3.20}
$$

式中， $\pmb { \phi } .$ 是机体坐标系从 $t _ { k }$ 时刻 $t _ { k + 1 }$ 时刻角位置变化所对应的等效旋转矢量， $\pmb { \omega } _ { n b } ^ { b }$ 是该时间段内的机体角速率。

## 9.3.3 旋转矢量的求解

直接按式(9. 3. 19)和式(9.3.20)求解旋转矢量有诸多不便，主要是:①激光陀螺等一般输出为角增量，如果将角增量折算成角速率，则微商运算将引起严重的噪声放大效应 ②即使可获得陀螺的角速率输出，由于对上述两方程只能采用数值求解，所以对角速率必须采样，采样意味着仅采样点上的角速率得到了利用，而采样点之间的角速率信息并未利用，在姿态更新中实际上丢失了很多信息;③式(9. 3. 3)说明，姿态更新中只需求解从 $t _ { k }$ 时刻至 $t _ { k + 1 }$ 时刻机体坐标系旋转所对应的等效旋转矢量，而不必知道 $t _ { k }$ 至 $t _ { k + 1 }$ 时间段内旋转矢量的演变过程，因此可采用泰勒级数展开法求解旋转矢量。

设 $\pmb { \phi } ( h )$ 为[ $[ t _ { k } , t _ { k + 1 }$ J时间段内的等效旋转矢量，其中 $h = t _ { k + 1 } - t _ { k }$ ，运载体的角速度用抛物线拟合

$$
\boldsymbol {\omega} _ {n b} ^ {b} \left(t _ {k} + \tau\right) = \boldsymbol {a} + 2 \boldsymbol {b} \tau + 3 \boldsymbol {c} \tau^ {2}, \quad 0 \leqslant \tau \leqslant h\tag{9.3.21}
$$

对 $\pmb { \phi } ( h )$ 作泰勒级数展开

$$
\boldsymbol {\Phi} (h) = \boldsymbol {\Phi} (0) + h \dot {\boldsymbol {\Phi}} (0) + \frac {h ^ {2}}{2 !} \ddot {\boldsymbol {\Phi}} (0) + \dots\tag{9.3.22}
$$

式中，! $\pmb { \Phi } ( 0 )$ 是 $\cdot \left[ t _ { k } , t _ { k _ { - } } \right]$ 时间段内的旋转矢量，由于时间间隔为 ，所以

$$
\boldsymbol {\Phi} (0) = \mathbf {0}\tag{9.3.23}
$$

记角增量为

$$
\Delta \boldsymbol {\theta} (\tau) = \int_ {0} ^ {\tau} \boldsymbol {\omega} _ {n b} ^ {b} (t _ {k} + \tau) \mathrm{d} \tau\tag{9.3.24}
$$

则由式 (9. 3. 21) 和式 (9.3.24)

$$
\boldsymbol {\omega} _ {n b} ^ {b} \left(t _ {k}\right) = \boldsymbol {\omega} _ {n b} ^ {b} \left(t _ {k} + \tau\right) | _ {\tau = 0} = \boldsymbol {a}\tag{9.3.25a}
$$

$$
\dot {\boldsymbol {\omega}} _ {n b} ^ {b} \left(t _ {k}\right) = \dot {\boldsymbol {\omega}} _ {n b} ^ {b} \left(t _ {k} + \tau\right) | _ {\tau = 0} = 2 \boldsymbol {b}\tag{9.3.25b}
$$

$$
\ddot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k}) = \ddot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k} + \tau) | _ {\tau = 0} = 6 \boldsymbol {c}\tag{9.3.25c}
$$

$$
\boldsymbol {\omega} _ {n b} ^ {b (i)} \left(t _ {k}\right) = \boldsymbol {\omega} _ {n b} ^ {b (i)} \left(t _ {k} + \tau\right) = \mathbf {0}, \quad i = 3, 4, 5, \dots\tag{9.3.25d}
$$

$$
\Delta \boldsymbol {\theta} (0) = \Delta \boldsymbol {\theta} (\tau) \mid_ {\tau = 0} = \mathbf {0}\tag{9.3.26a}
$$

$$
\Delta \dot {\boldsymbol {\theta}} (0) = \Delta \dot {\boldsymbol {\theta}} (\tau) | _ {\tau = 0} = \boldsymbol {\omega} _ {n b} ^ {b} \left(t _ {k} + \tau\right) | _ {\tau = 0} = \boldsymbol {a}\tag{9.3.26b}
$$

$$
\Delta \ddot {\boldsymbol {\theta}} (0) = \Delta \ddot {\boldsymbol {\theta}} (\tau) | _ {\tau = 0} = \dot {\omega} _ {n b} ^ {b} \left(t _ {k} + \tau\right) | _ {\tau = 0} = 2 \boldsymbol {b}\tag{9.3.26c}
$$

$$
\Delta \ddot {\boldsymbol {\theta}} (0) = \Delta \ddot {\boldsymbol {\theta}} (\tau) | _ {\tau = 0} = \ddot {\omega} _ {n b} ^ {b} \left(t _ {k} + \tau\right) | _ {\tau = 0} = 6 c\tag{9.3.26d}
$$

$$
\Delta \boldsymbol {\theta} ^ {(i)} (0) = \Delta \boldsymbol {\theta} ^ {(i)} (\tau) | _ {\tau = 0} = \omega_ {n b} ^ {b (i - 1)} \left(t _ {k} + \tau\right) = \mathbf {0}, \quad i = 4, 5, 6, \dots\tag{9.3.26e}
$$

又由于姿态更新周期 一般为毫秒级的量， $\varPhi$ 也可视为小量，因此根据式 (9.3.20) 计算 $\pmb { \phi } ( \tau )$ 在$\tau { = } 0$ 时的各阶导数时，可将该式中的第三项视为 $\varPhi$ 的二阶小量而略去不计，并将第二项中的$\pmb { \phi } ( \tau )$ 〉用角增量代替，即

$$
\pmb {\Phi} (\tau) \approx \Delta \pmb {\theta} (\tau)
$$

这样式 (9. 3. 20)可写成

$$
\dot {\boldsymbol {\Phi}} (\tau) = \boldsymbol {\omega} _ {n b} ^ {b} (t _ {k} + \tau) + \frac {1}{2} \Delta \boldsymbol {\theta} (\tau) \times \boldsymbol {\omega} _ {n b} ^ {b} (t _ {k} + \tau), \quad 0 \leqslant \tau \leqslant h
$$

对上式求各阶导数，并考虑到式(ω9.3.25d)和式 (9. 3. 26e)

$$
\begin{array}{l} \ddot {\boldsymbol {\Phi}} (\tau) = \dot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k} + \tau) + \frac {1}{2} \Delta \dot {\boldsymbol {\theta}} (\tau) \times \boldsymbol {\omega} _ {n b} ^ {b} (t _ {k} + \tau) + \frac {1}{2} \Delta \boldsymbol {\theta} (\tau) \times \dot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k} + \tau) \\ \dddot {\boldsymbol {\Phi}} (\tau) = \dddot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k} + \tau) + \frac {1}{2} \Delta \ddot {\boldsymbol {\theta}} (\tau) \times \boldsymbol {\omega} _ {n b} ^ {b} (t _ {k} + \tau) + \frac {1}{2} \Delta \dot {\boldsymbol {\theta}} (\tau) \times \dot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k} + \tau) \\ \quad + \frac {1}{2} \Delta \dot {\boldsymbol {\theta}} (\tau) \times \dot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k} + \tau) + \frac {1}{2} \Delta \boldsymbol {\theta} (\tau) \times \dddot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k} + \tau) \\ = \ddot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k} + \tau) + \frac {1}{2} \Delta \ddot {\boldsymbol {\theta}} (\tau) \times \boldsymbol {\omega} _ {n b} ^ {b} (t _ {k} + \tau) + \Delta \dot {\boldsymbol {\theta}} (\tau) \\ \quad \times \dot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k} + \tau) + \frac {1}{2} \Delta \boldsymbol {\theta} (\tau) \times \ddot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k} + \tau) \\ \boldsymbol {\Phi} ^ {(4)} (\tau) = \frac {1}{2} \Delta \dddot {\boldsymbol {\theta}} (\tau) \times \boldsymbol {\omega} _ {n b} ^ {b} (t _ {k} + \tau) + \frac {1}{2} \Delta \ddot {\boldsymbol {\theta}} (\tau) \times \dot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k} + \tau) + \Delta \ddot {\boldsymbol {\theta}} (\tau) \times \dot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k} + \tau) \\ \quad + \Delta \dot {\boldsymbol {\theta}} (\tau) \times \dddot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k} + \tau) + \frac {1}{2} \Delta \dot {\boldsymbol {\theta}} (\tau) \times \dddot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k} + \tau) \\ = \frac {1}{2} \Delta \dddot {\boldsymbol {\theta}} (\tau) \times \boldsymbol {\omega} _ {n b} ^ {b} (t _ {k} + \tau) + \frac {3}{2} \Delta \ddot {\boldsymbol {\theta}} (\tau) \times \dot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k} + \tau) + \frac {3}{2} \Delta \dot {\boldsymbol {\theta}} (\tau) \times \dddot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k} + \tau). \end{array}
$$

$$
\begin{array}{r l} \boldsymbol {\Phi} ^ {(5)} (\tau) & = \frac {1}{2} \Delta \ddot {\boldsymbol {\theta}} (\tau) \times \dot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k} + \tau) + \frac {3}{2} \Delta \ddot {\boldsymbol {\theta}} (\tau) \times \dot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k} + \tau) \\ & + \frac {3}{2} \Delta \ddot {\boldsymbol {\theta}} (\tau) \times \ddot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k} + \tau) + \frac {3}{2} \Delta \ddot {\boldsymbol {\theta}} (\tau) \times \ddot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k} + \tau) \\ & = 2 \Delta \ddot {\boldsymbol {\theta}} (\tau) \times \dot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k} + \tau) + 3 \Delta \ddot {\boldsymbol {\theta}} (\tau) \times \ddot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k} + \tau) \\ \boldsymbol {\Phi} ^ {(6)} (\tau) & = 2 \Delta \ddot {\boldsymbol {\theta}} (\tau) \times \ddot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k} + \tau) + 3 \Delta \ddot {\boldsymbol {\theta}} (\tau) \times \ddot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k} + \tau) = 5 \Delta \ddot {\boldsymbol {\theta}} (\tau) \times \ddot {\boldsymbol {\omega}} _ {n b} ^ {b} (t _ {k} + \tau) \\ & \quad \boldsymbol {\Phi} ^ {(i)} (\tau) = 0, i = 7, 8, 9, \dots \end{array}
$$

用 $\tau { = } 0$ 代人上述诸式，并应用式 (9. 3. 25) 和式 (9. 3. 26) ，得

$$
\dot {\pmb {\Phi}} (0) = \pmb {a}
$$

$$
\ddot {\boldsymbol {\Phi}} (0) = 2 \boldsymbol {b} + \frac {1}{2} \boldsymbol {a} \times \boldsymbol {a} = 2 \boldsymbol {b}
$$

$$
\ddot {\boldsymbol {\Phi}} (0) = 6 \boldsymbol {c} + \frac {1}{2} \cdot 2 \boldsymbol {b} \times \boldsymbol {a} + \boldsymbol {a} \times 2 \boldsymbol {b} = 6 \boldsymbol {c} + \boldsymbol {a} \times \boldsymbol {b}
$$

$$
\boldsymbol {\Phi} ^ {(4)} (0) = \frac {1}{2} \cdot 6 \boldsymbol {c} \times \boldsymbol {a} + \frac {3}{2} \cdot 2 \boldsymbol {b} \times 2 \boldsymbol {b} + \frac {3}{2} \cdot \boldsymbol {a} \times 6 \boldsymbol {c} = 6 \boldsymbol {a} \times \boldsymbol {c}
$$

$$
\boldsymbol {\Phi} ^ {(5)} (0) = 2 \cdot 6 \boldsymbol {c} \times 2 \boldsymbol {b} + 3 \cdot 2 \boldsymbol {b} \times 6 \boldsymbol {c} = 1 2 \boldsymbol {b} \times \boldsymbol {c}
$$

$$
\begin{array}{l} \boldsymbol {\Phi} ^ {(6)} (0) = 5 \cdot 6 \boldsymbol {c} \times 6 \boldsymbol {c} = \mathbf {0} \\ \boldsymbol {\Phi} ^ {(i)} (0) = \mathbf {0}, \quad i = 7, 8, 9, \dots \end{array}
$$

上式诸式和式 (9.3.23)代人式 (9. 3. 22) ，得

$$
\begin{array}{r l} \boldsymbol {\Phi} (h) & = \boldsymbol {\Phi} (0) + \dot {\boldsymbol {\Phi}} (0) h + \frac {1}{2} \ddot {\boldsymbol {\Phi}} (0) h ^ {2} + \frac {1}{6} \dddot {\boldsymbol {\Phi}} (0) h ^ {3} + \frac {1}{2 4} \boldsymbol {\Phi} ^ {(4)} (0) h ^ {4} + \frac {1}{1 2 0} \boldsymbol {\Phi} ^ {(5)} (0) h ^ {5} \\ & = a h + b h ^ {2} + c h ^ {3} + \frac {1}{6} a \times b h ^ {3} + \frac {1}{4} a \times c h ^ {4} + \frac {1}{1 0} b \times c h ^ {5} \end{array} \tag {9.}\tag{9.3.27}
$$

为了将上式中的参数 用角增量来表示，记

$$
\Delta \boldsymbol {\theta} _ {i} = \int_ {\frac {i - 1}{3} h} ^ {\frac {i}{3} h} \boldsymbol {\omega} _ {n b} ^ {b} (t _ {k} + \tau) \mathrm{d} \tau , \quad i = 1, 2, 3
$$

\~p

$$
\Delta \boldsymbol {\theta} _ {1} = \frac {1}{3} \boldsymbol {a h} + \frac {1}{9} \boldsymbol {b h} ^ {2} + \frac {1}{2 7} \boldsymbol {c h} ^ {3}\tag{1}
$$

$$
\Delta \boldsymbol {\theta} _ {2} = \frac {1}{3} \boldsymbol {a h} + \frac {3}{9} \boldsymbol {b h} ^ {2} + \frac {7}{2 7} \boldsymbol {c h} ^ {3}\tag{2}
$$

$$
\Delta \boldsymbol {\theta} _ {3} = \frac {1}{3} \boldsymbol {a h} + \frac {5}{9} \boldsymbol {b h} ^ {2} + \frac {1 9}{2 7} \boldsymbol {c h} ^ {3}\tag{3}
$$

由式(1)+式(2)+ (3) ，得

$$
\boldsymbol {a h} + \boldsymbol {b h} ^ {2} + \boldsymbol {c h} ^ {3} = \Delta \boldsymbol {\theta} _ {1} + \Delta \boldsymbol {\theta} _ {2} + \Delta \boldsymbol {\theta} _ {3}\tag{4}
$$

(3) (2) X2+式(1) ，得

$$
\boldsymbol {c h} ^ {3} = \frac {9}{2} (\Delta \boldsymbol {\theta} _ {3} - 2 \Delta \boldsymbol {\theta} _ {2} + \Delta \boldsymbol {\theta} _ {1})\tag{5}
$$

(2) 一式(1) ，并以式 (5) 代人之，得