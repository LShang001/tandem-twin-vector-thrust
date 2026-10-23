# 第3章 摆式加速度计

## 第3章 摆式加速度计

陀螺用来感测运载体的角运动信息，而加速度计用来感测运载体的线运动信息，两者都是构造惯导系统的核心器件，其精度高低和性能优劣基本上决定了惯导系统的精度和性能。本章重点介绍挠性加速度计的工作原理及动、静态误差模型。

## 3. 1 加速度计简介

## 3. 1. 1 加速度计的基本测量原理

3. 1. 是一种加速度计的简化模型，由质量块 弹簧 和阻尼器 组成，敏感轴 沿铅

![](images/aabdea0f71d8a3dad3368f96e0aa8fa1a132c28072a95a97c1b2e9630833579c.jpg)  
图3.1.1 加速度计简化模型

垂向上，虚线表示基座无加速度时质量块处于自由状态时的位置，此时质量块的质心定义为位移 的原点。设基座具有沿 方向的加速，则在惯性力作用下，弹簧发生变形，产生的弹性力与变形方向相反，阻尼力与变形速率方向相反，所以根据牛顿第二定律

$$
m (\ddot {x} + \boldsymbol {a}) = - D \dot {x} - c x + m \mathbf {g}
$$

该式在 $x$ 轴上的投影形式为

$$
m \ddot {x} + D \dot {x} + c x = - m (a + g)\tag{3.1.1}
$$

对常值加速度 $\mathbf { \pmb { a } }$ ，质量块 运动达到稳态时， $\mathbf { \ddot { \boldsymbol { x } } } = 0 , \mathbf { \dot { \boldsymbol { x } } } = 0$ ，所以有

$$
a + g = - \frac {c x}{m} = \frac {F _ {\text {弹}}}{m}
$$

由于比力 j(specific force)定义为作用在单位质量上的非引力外力，所以 ${ \frac { F _ { \vec { \mathfrak { g } } \vec { \underline { { \mathfrak { h } } } } } } { m } } = f$ 即为比力。在该简单特例中， $F _ { \vec { y } \vec { \sharp } }$ 是可通过弹簧的变形量来测量的，即加速度计可测量得比力 $f .$ ，而 $f { = } a + g$ ，所以要

对加速度计的输出作适当处理后才能获得基座的运动加速度，在此例中应从 $f$ 中扣除 $g$ 才能获得 $a$ 。因此严格地说，加速度计应该称为比力计。

如果图 3. 1. 所示简化模型的敏感轴 处于水平位置，并假设沿 轴存在阻力 $F _ { \sharp \sharp }$ ，则式(3. 1. 1)应改写成

$$
m \ddot {x} + D \dot {x} + c x = - m a + F _ {\text {阻}}\tag{3.1.2}
$$

小到使- $- m a + F _ { \parallel } = 0$ 成立，则 $x { = } 0$ 此时加速度 $a \neq 0$ ，但加速度计元输出，定义此加速度为加速度计的灵敏度，即

$$
a _ {\text {灵敏阈}} = \frac {F _ {\text {阻}}}{m}\tag{3.1.3}
$$

该指标反映了加速度计能测出的最小加速度。此外位移信号检测器的灵敏度也影响灵敏度。惯性级加速度计的灵敏度应优于 $1 0 ^ { - 4 } g$

反映加速度计性能的另一个指标是加速度计的灵敏度，可用刻度系数描述之。前者通常指它的稳、态放大系数:

$$
K _ {\mathrm{ss}} = \frac {m}{c}\tag{3.1.4}
$$

后者以其输出信号来表示，其中计人了信号器的灵敏度 $K _ { \rho }$

$$
K _ {S F} = \frac {m}{c} K _ {p}\tag{3.1.5}
$$

反映加速度计性能的第三个指标是自然频率，它表征了加速度计的动态特性:

$$
\omega_ {n} = \sqrt {\frac {c}{m}}\tag{3.1.6}
$$

该指标是根据使用要求来确定的，一般为数十赫兹至数百赫兹。自然频率一旦确定，仪表的测量范围，即其上、下限也就确定 $\vec { \mathrm { ~ \ j ~ } }$

## 3. 1. 2 摆式加速度计简介

按图 3. 1. 所示简化模型设计加速度计存在诸多问题，如质量块与基座支承面间存在摩擦力，由式。.1. 3)知该摩擦力将严重影响加速度计的灵敏度;对质量块的运动难以实现精确约束;输出信号检测和对质量块的力反馈控制困难。因此，工程上都采用具有偏心质量的摆式结构作为加速度计的设计方案，常见的摆式加速度计简介如下。

## 1.液浮摆式加速度计

为了提高加速度计的灵敏度，必须尽量减小质量块支承处的摩擦影响。与液浮陀螺相仿，解决摩擦影响的有效途径是采用液浮支承，图 3. 1. 是按此思路设计的液浮摆式加速度计。摆组件浸泡在浮液中，产生的浮力使轴承处的正压力减少至接近为 $0 _ { c }$ 。设 $c _ { m }$ 为摆组件的质量中 $\therefore \dot { \mathsf { L } } \mathsf { \Omega } ^ { } \bullet \mathsf { c } _ { _ F }$ 为摆组件的浮心，距支承轴的距离分别为 $L _ { m }$ 和 $L _ { F }$ '摆组件的质量为 $m .$ 浮力为 $F _ { \mathrm { ~ c ~ } }$ 。记 $I _ { A } , O _ { A } , P _ { A }$ 分别为加速度的输入轴、输出轴和摆轴，则在重力和浮力的共同作用下，摆组件绕输出轴的摆性力矩为

$$
M _ {P} = m g L _ {m} + F L _ {F}
$$

该力矩等效为摆组件的重力偏心力矩 $m g L$ 其中 为摆组件的等效重力力臂，即

$$
m g L = m g L _ {m} + F L _ {F}
$$

![](images/2d827f960b30aaeaa65619e1cc4b5902fce948beee62ab4628d5afde76e86c0a.jpg)  
3. 1. 液浮摆式加速度计

![](images/3ac45420f6f5c0a4b3b767176bebbb6b81b23025e455fcf956dd8343bf094ff8.jpg)  
3. 1. 陀螺积分加速度计

将

$$
P = m L = m L _ {m} + \frac {F}{g} L _ {F}\tag{3.1.7}
$$

称为加速度计的摆性。

设沿 $I _ { A }$ 轴作用有加速度 $\textit { a } _ { I }$ 所产生的摆性力矩使摆组件绕 $O _ { A }$ 轴旋转产生输出角。 $\theta$ ，信号器输出与 $\theta$ 相应的电信号，经前置放大、解调和直流放大后加至力矩器，产生绕 $O _ { A }$ 轴的恢复力矩，平衡掉摆性力矩，从力矩器电流采样值可提取出输入加速度 $\boldsymbol { \mathcal { a } } _ { I }$

## 2. 陀螺积分加速度计

在远程制导武器中常用基于陀螺原理的加速度计，即陀螺积分加速度计，图3. 1. 为工作原理图，该型加速度计在德国V-2 火箭中最早使用。陀螺积分加速度

计实质上是一个双自由度陀螺仪，内环轴处于水平位置，外环轴处于铅垂位置，在内环上距支承中心 $L$ 处安装有一个偏心质量块 $m$ 。设陀螺的角动量为 H. $y$ 轴作用有加速度 $a _ { I }$ ，则沿 $_ { \mathcal { X } }$ 轴作用有摆'性力矩 $M _ { x } { = } m L a _ { I }$ 从而使陀螺绕 轴进动，进动角速度为

$$
\dot {\beta} = \frac {m L a _ {I}}{H}
$$

在外环轴上的信号器可拾取卢 $\beta$ ，而 $\beta$ 是 $\dot { \boldsymbol { \beta } }$ 的积分:

$$
\beta = \frac {m L}{H} \int_ {0} ^ {t} a _ {I} \mathrm{d} \tau = \frac {m L}{H} (V - V _ {0})\tag{3.1.8}
$$

根据输出的 $\beta$ 角可计算出沿 $y$ 轴的速度:

$$
V = V _ {0} + \frac {H \beta}{m L}\tag{3.1.9}
$$

在陀螺、绕外环轴做进动运动时，外环轴轴承的摩擦力矩会引起陀螺绕内环轴进动，这种进动会使陀螺感受加速度时绕外环轴进动的角动量发生变化，甚至可使陀螺的角动量趋向与外环轴重合，此时陀螺退化为定轴转动刚体，丧失测量功能。为了避免上述问题的出现，确保陀螺能准确感测 $y$ 轴方向的加速度，由内环轴信号器检测出的内环偏角信号经放大后送至外环力矩器，组成力矩再平衡系统，按负反馈状态工作，就可以使陀螺绕内环轴的转动受到控制。

## 3. 2 挠性摆式加速度计

采用挠性支承是摆式加速度的另一种典型结构，由于挠性支承不存在摩擦，弹性变形产生的恢复力符合胡克定律，变化规律是确定的，特别是采用石英材料制作挠性杆，能有效降低温度影响，且材料性能稳定，有利于精度的提高和实现批量生产，石英挠性摆式加速度计在导航系统集成中占有主要的市场地位，为此这里对该型加速度计作详细介绍。

3. 2. 为挠性摆式加速度计的结构示意图。加速度计由摆组件、信号传感器、推挽式永磁力矩器和挠性杆组成，表内充有硅油作为阻尼液体，并设有精密温控装置，以保证仪表在恒定的工作温度下工作。挠性杆一端固定在表壳上，另一端粘贴有信号器动圈，形成悬臂梁，在摆片上固定有力矩器动圈，永久磁铁一端固定在表壳上，另一端空套在力矩器动圈内。当表壳(基座)具有沿 $I _ { A }$ 轴方向的加速度 时，挠性杆在惯性力矩作用下绕挠性接头中心位置旋转，产生输出角$\theta _ { 0 }$ .信号器检测出 $\theta _ { 0 }$ 。并转换成电压信号，经放大后加人力矩器动圈，产生的推挽力矩抵消惯性力矩，使 $\theta _ { 0 }$ 。回零，再平衡回路中的电流经采样电阻变换成输出电压，从此电压可提取出加速度 $a$ 为分析方便，常将加速度计的输入轴(敏感轴)记为 $I _ { A }$ 输出轴记为 $O _ { A }$ .摆轴(挠性杆处于中立位置时的对称轴)记为 $P _ { A }$

![](images/70f260f2069e1f4e69bd0c79c12796a2cc52af3088a2c569e86e046dfdb0e1ed.jpg)  
3.2.1 挠性摆式加速度计结构示意图  
1.挠性支承; 2. 壳体; 3. 辄铁; 4. 力矩器动圈; 5. 永久磁铁;  
6. 摆件; 7. 信号器激磁线圈; 8. 信号器动圈; 9. 放大器

## 3.2.1 挠性摆式加速度计的动力学方程

为分析问题方便起见，假设加速度计的敏感轴 $I _ { A }$ 处于水平位置，重力加速度沿加速度计的输出轴 $O _ { A }$ 方向，如图 3.2.2 所示。图中，输出轴 $O _ { A }$ 指向纸面里面，所以用括号括起来。

![](images/0b8ff40bfbcb112b1108c970799baaaf22515196f39f90cf4b395770187af624.jpg)  
3.2.2 摆组件的受力情况

设表壳(基座)沿 $I _ { A }$ 轴和沿 $P _ { A }$ 轴存在加速度 $\boldsymbol { a } _ { \scriptscriptstyle I }$ 和 $\boldsymbol { a } _ { \scriptscriptstyle P }$ ，摆组件绕 $O _ { A }$ 轴的转动惯量为 $I _ { O }$ ，挠性杆的弹性系数为 ，浮液的阻尼系数为 ，信号器传递系数为 $K _ { s }$ ，伺服放大器增益为 $K _ { a }$ 负载系数为 $K _ { R } \left( K _ { R } = \frac { 1 } { R _ { \mathrm { 0 } } + R _ { T } } , R _ { \mathrm { 0 } } \right)$ 为采样电阻 $R _ { T }$ 为力矩器内阻)<sub>I</sub> ，力矩器标度因数为 $K _ { T }$ 则根据动量矩定理，

$$
I _ {O} \ddot {\theta} _ {O} = m L a _ {I} - m L \sin \theta_ {O} a _ {P} - D \dot {\theta} _ {O} - C \theta_ {O} - K _ {T} i + M _ {\mathrm{d}}
$$

式中 $i { = } K _ { R } K _ { a } K _ { s } \theta _ { O }$ 是加人力矩器的力反馈电流 $M _ { \mathrm { d } }$ 是沿 $O _ { A }$ 轴的干扰力矩。由于加速度计工作时必须处于闭环状态，即再平衡回路工作，控制。 $\theta _ { O }$ 角接近等于零，所以上式可写成:

$$
\begin{array}{c} (I _ {O} s ^ {2} + D s + C) \theta_ {O} (s) = m L \big [ a _ {I} (s) - \theta_ {O} (s) a _ {P} (s) \big ] - K _ {T} i (s) + M _ {\mathrm{d}} (s) \\ i (s) = K _ {R} K _ {a} K _ {s} \theta_ {O} (s) \end{array}
$$

由上述两式可画出挠性摆式加速度计的方块图，如图 3.2.3 所示。

![](images/dadc40e875564b5f2f296e68014dc7e061c5b63712509e483bd05592d80fe954.jpg)  
3.2.3 挠性摆式加速度计方块图

由图可看出，挠性摆式加速度计是二阶线性系统。略去 $a _ { _ P } ( s ) \theta _ { 0 } ( s )$ 的影响，由图得

$$
i (s) = \frac {m L K _ {s} K _ {a} K _ {R}}{I _ {O} s ^ {2} + D s + C + K _ {s} K _ {a} K _ {R} K _ {T}} a _ {I} (s) + \frac {K _ {s} K _ {a} K _ {R}}{I _ {O} s ^ {2} + D s + C + K _ {s} K _ {a} K _ {R} K _ {T}} M _ {\mathrm{d}} (s)
$$

对常值加速度输入和常值干扰力矩， ep $a _ { I } ( s ) = \frac { a _ { I } } { s } , M _ { \mathrm { d } } ( s ) = \frac { M _ { \mathrm { d } } } { s }$ ，再平衡回路的电流输出稳态值为

$$
i _ {S S} = \frac {m L K _ {s} K _ {a} K _ {R}}{C + K _ {s} K _ {a} K _ {R} K _ {T}} a _ {I} + \frac {K _ {s} K _ {a} K _ {R}}{C + K _ {s} K _ {a} K _ {R} K _ {T}} M _ {\mathrm{d}}\tag{3.2.1}
$$

式中， $K _ { 1 } { = } \frac { m L K _ { s } K _ { a } K _ { R } } { C { + } K _ { s } K _ { a } K _ { R } K _ { T } }$ 称为加速度计的标度因数。通常有

$$
C \ll K _ {s} K _ {a} K _ {R} K _ {T}
$$

所以

$$
K _ {1} \approx \frac {m L}{K _ {T}}\tag{3.2.2}
$$

(3. 2. 1)两边同乘采样电 $R _ { 0 }$ ，即得采样电压的稳态值 $u _ { S S }$ ，考虑到上述两式所列关系，得

$$
u _ {\mathrm{SS}} = K _ {1} R _ {0} a _ {I} + \frac {R _ {0}}{K _ {T}} M _ {\mathrm{d}}\tag{3.2.3}
$$

由上式得 $a _ { I }$ 的准确值应为

$$
a _ {I} = \frac {1}{K _ {1} R _ {0}} \left(u _ {\mathrm{SS}} - \frac {R _ {0}}{K _ {T}} M _ {\mathrm{d}}\right) = \frac {u _ {\mathrm{SS}}}{K _ {1} R _ {0}} - \frac {M _ {\mathrm{d}}}{m L}
$$

而 $M _ { \mathrm { d } }$ 是干扰力矩，是无法准确测量的，所以 $a _ { I }$ 只能通过采样电压的稳态值来确定:

$$
\hat {a} _ {I} = \frac {u _ {\mathrm{SS}}}{K _ {1} R _ {0}}
$$

引起对 $\boldsymbol { \mathcal { a } } _ { I }$ 的测量误差为

$$
\delta a _ {I} = \hat {a} _ {I} - a _ {I} = \frac {M _ {\mathrm{d}}}{m L}\tag{3.2.4}
$$

从上式可看出，增大加速度计的摆性 $P { = } m L$ 对降低干扰力矩的影响是有好处的。

## 3.2.2 挠性摆式加速度计的静态数学模型

记 $I O P$ 为加速度计的测量坐标系，简记为 ，由摆组件确定的坐标系为 $I _ { A } O _ { A } P _ { A }$ 简记为A.由于存在输出角。和工艺制造误差 坐标系相对 坐标系存在偏差角。 $\theta _ { I } \ 、 \theta _ { O } \ 、 \theta _ { P }$ 品，它们都是小角，所以

$$
C _ {M} ^ {A} = \left[ \begin{array}{c c c} 1 & \theta_ {P} & - \theta_ {O} \\ - \theta_ {P} & 1 & \theta_ {I} \\ \theta_ {O} & - \theta_ {I} & I \end{array} \right]\tag{3.2.5}
$$

式中， $\theta _ { \mathrm { { O } } }$ )，。为加速度计的输出角， $\mathbf { \nabla } \cdot \theta _ { I }$ 和 $\theta _ { P }$ 由工艺制造误差引起。

设表壳(基座)做加速运动，加速度计质量块产生的单位质量惯性力在测量坐标系 内的投影为

$$
\boldsymbol {F} ^ {M} = \left[ \begin{array}{c c c} F _ {I} & F _ {O} & F _ {P} \end{array} \right] ^ {\mathrm{T}}
$$

质量块质量为 质心沿 $I _ { A } , O _ { A } , P _ { A }$ 轴的固定偏心距为 $l _ { \mathrm { \Omega } _ { L A } } \setminus l _ { \mathrm { \Omega } _ { \mathrm { O A } } } \setminus L$ ，摆组件的弹性变形张量为$\left[ \begin{array} { l l l } { C _ { I I } } & { C _ { I O } } & { C _ { I P } } \\ { C _ { O I } } & { C _ { O O } } & { C _ { O P } } \\ { C _ { P I } } & { C _ { P O } } & { C _ { P P } } \end{array} \right]$ '其中 $l _ { { \scriptscriptstyle L A } } \setminus l _ { \mathrm { { o A } } }$ 为制造偏差， 为等效摆臂， 决定加速度计的摆性 $( P =$

$m L )$ ， $C _ { i j }$ 启示 方向的单位载荷引起 方向的变形量，当 $i \neq j$ 时， $C _ { i j }$ 可视为小量。则单位质量惯性力在摆组件坐标系 的投影为

$$
\boldsymbol {F} ^ {A} = \boldsymbol {C} _ {M} ^ {A} \boldsymbol {F} ^ {M} = \left[ \begin{array}{c c c} 1 & \theta_ {P} & - \theta_ {O} \\ - \theta_ {P} & 1 & \theta_ {I} \\ \theta_ {O} & - \theta_ {I} & 1 \end{array} \right] \left[ \begin{array}{c} F _ {I} \\ F _ {O} \\ F _ {P} \end{array} \right]
$$

\~p

$$
F _ {I A} = F _ {I} + \theta_ {P} F _ {O} - \theta_ {O} F _ {P}\tag{3.2.6a}
$$

$$
F _ {O A} = F _ {O} - \theta_ {P} F _ {I} + \theta_ {I} F _ {P}\tag{3.2.6b}
$$

$$
F _ {P A} = F _ {P} + \theta_ {O} F _ {I} - \theta_ {I} F _ {O}\tag{3.2.6c}
$$

引起的惯性力如图 3.2.4所示。引起质量块质心偏离摆组件支承中心的总偏心量为

$$
\left[ \begin{array}{l} L _ {I A} \\ L _ {O A} \\ L _ {P A} \end{array} \right] = \left[ \begin{array}{l} l _ {I A} \\ l _ {O A} \\ L \end{array} \right] + m \left[ \begin{array}{c c c} C _ {I I} & C _ {I O} & C _ {I P} \\ C _ {O I} & C _ {O O} & C _ {O P} \\ C _ {P I} & C _ {P O} & C _ {P P} \end{array} \right] \left[ \begin{array}{l} F _ {I} + \theta_ {P} F _ {O} - \theta_ {O} F _ {P} \\ F _ {O} - \theta_ {P} F _ {I} + \theta_ {I} F _ {P} \\ F _ {P} + \theta_ {O} F _ {I} - \theta_ {I} F _ {O} \end{array} \right]
$$

展开上式，略去三阶小量得

$$
L _ {I A} = l _ {I A} + m C _ {I I} F _ {I} + m \left(C _ {I O} + C _ {I I} \theta_ {P}\right) F _ {O} + m \left(C _ {I P} - C _ {I I} \theta_ {O}\right) F _ {P}\tag{3.2.7a}
$$

$$
L _ {O A} = l _ {O A} + m \left(C _ {O I} - C _ {O O} \theta_ {P}\right) F _ {I} + m C _ {O O} F _ {O} + m \left(C _ {O P} + C _ {O O} \theta_ {I}\right) F _ {P}\tag{3.2.7b}
$$

$$
L _ {P A} = L + m \left(C _ {P I} + C _ {P P} \theta_ {O}\right) F _ {I} + m \left(C _ {P O} - C _ {P P} \theta_ {I}\right) F _ {O} + m C _ {P P} F _ {P}\tag{3.2.7c}
$$

由图 3.2.4，可计算得由惯性力引起的绕 $O _ { A }$ 轴的惯性力矩为

$$
M _ {1} = m F _ {I A} \cdot L _ {P A} - m F _ {P A} \cdot L _ {I A}
$$

![](images/e68e6434c8846b4211b6a345d02e55a6e13d7f36824b12f61e741951d8463bc9.jpg)  
3.2.4 作用在质量块质心土的惯性力

将式(3.2. 6) 和式 (3.2. 7) 代入上式，略去二阶小量，得

$$
\begin{array}{r l} M _ {1} = & m L \left(F _ {I} + \theta_ {P} F _ {O} - \theta_ {O} F _ {P}\right) - m l _ {I A} \left(F _ {P} + \theta_ {O} F _ {I} \right. \\ & \left. - \theta_ {I} F _ {O}\right) + m ^ {2} \left[ C _ {P I} F _ {I} ^ {2} + C _ {P O} F _ {I} F _ {O} \right. \\ & \left. - C _ {I O} F _ {O} F _ {P} + \left(C _ {P P} - C _ {I I}\right) F _ {P} F _ {I} - C _ {I P} F _ {P} ^ {2} \right] \end{array}\tag{3.2.8}
$$

由图 3. 2.

$$
\frac {\theta_ {O} (s)}{a _ {I} (s)} = \frac {m L}{I _ {O} s ^ {2} + D s + C + K _ {s} K _ {a} K _ {R} K _ {T}}
$$

对于常值输入加速度 $a _ { I } \left( s \right) = \frac { a _ { I } } { s }$ ，输出角的稳态值为

$$
\theta_ {\mathrm{Oss}} = \frac {m L}{C + K _ {s} K _ {a} K _ {R} K _ {T}} a _ {I} = \frac {m L}{K} a _ {I}\tag{3.2.9}
$$

式中

$$
K = C + K _ {s} K _ {a} K _ {R} K _ {T}\tag{3.2.10}
$$

将式 (3.2.9) 代入式 (3.2. 剖，并考虑到 $a _ { I } = - F _ { I }$ ，则得

$$
\begin{array}{l} M _ {1} = m L F _ {I} + (m L \theta_ {P} + m l _ {I A} \theta_ {I}) F _ {O} - m l _ {I A} F _ {P} + m ^ {2} C _ {P O} F _ {I} F _ {O} - m ^ {2} C _ {I O} F _ {O} F _ {P} \\ \quad + \left[ m ^ {2} \left(C _ {P P} - C _ {I I}\right) + \frac {m ^ {2} L ^ {2}}{K} \right] F _ {I} F _ {P} + \left[ m ^ {2} C _ {P I} + \frac {m ^ {2} l _ {I A} L}{K} \right] F _ {I} ^ {2} - m ^ {2} C _ {I P} F _ {P} ^ {2} \end{array}
$$

由于加速度计处在力反馈工作状态，力矩器产生的反馈力矩对消输出轴上的外作用力矩，即

$$
K _ {T} i _ {\mathrm{SS}} = - M _ {1}
$$

所以再平衡回路内的采样电压为

$$
\begin{array}{l} u _ {\mathrm{SS}} = - \frac {R _ {0}}{K _ {T}} M _ {1} = - \frac {R _ {0} m L}{K _ {T}} F _ {I} - \frac {R _ {0}}{K _ {T}} (m L \theta_ {P} + m l _ {I A} \theta_ {I}) F _ {O} + \frac {R _ {0} m l _ {I A}}{K _ {T}} F _ {P} - \frac {R _ {0} m ^ {2} C _ {P O}}{K _ {T}} F _ {I} F _ {O} \\ \quad + \frac {R _ {0} m ^ {2} C _ {I O}}{K _ {T}} F _ {O} F _ {P} - \frac {R _ {0}}{K _ {T}} \left[ m ^ {2} \left(C _ {P P} - C _ {I I}\right) + \frac {m L}{K} \right] F _ {I} F _ {P} \\ \quad - \frac {R _ {0}}{K _ {T}} \left[ m ^ {2} C _ {P I} + \frac {m ^ {2} l _ {I A} L}{K} \right] F _ {I} ^ {2} - \frac {R _ {0} m ^ {2} C _ {I P}}{K _ {T}} F _ {P} ^ {2} \end{array} \tag {3.}\tag{3.2.11}
$$

记

$$
K _ {I} = - \frac {R _ {0} m L}{K _ {T}}\tag{3.2.12a}
$$

$$
K _ {O} = - \frac {R _ {0}}{K _ {T}} (m L \theta_ {P} + m l _ {_ {I A}} \theta_ {I})\tag{3.2.12b}
$$

$$
K _ {P} = \frac {R _ {0} m l _ {I A}}{K _ {T}}\tag{3.2.12c}
$$

$$
K _ {I O} = - \frac {R _ {0} m ^ {2} C _ {P O}}{K _ {T}}\tag{3.2.12d}
$$

$$
K _ {O P} = \frac {R _ {0} m ^ {2} C _ {I O}}{K _ {T}}\tag{3.2.12e}
$$

$$
K _ {I P} = - \frac {R _ {0}}{K _ {T}} \left[ m ^ {2} \left(C _ {P P} - C _ {I I}\right) + \frac {m L}{K} \right]\tag{3.2.12f}
$$

$$
K _ {I I} = - \frac {R _ {0}}{K _ {T}} \left(m ^ {2} C _ {P I} + \frac {m ^ {2} l _ {I A} L}{K}\right)\tag{3.2.12g}
$$

$$
K _ {P P} = - \frac {R _ {0} m ^ {2} C _ {I P}}{K _ {T}}\tag{3.2.12h}
$$

并引人与运动无关的加速度计零位误差 $K _ { F }$ 则式 (3.2.11) 可写成

$$
\begin{array}{r l} u _ {S S} = & K _ {F} + K _ {I} F _ {I} + K _ {O} F _ {O} + K _ {P} F _ {P} + K _ {I O} F _ {I} F _ {O} + K _ {O P} F _ {O} F _ {P} \\ & + K _ {P I} F _ {P} F _ {I} + K _ {I I} F _ {I} ^ {2} + K _ {P P} F _ {P} ^ {2} \end{array}\tag{3.2.13}
$$

(3. 2. 13)即为加速度计的静态数学模型，之所以这样称谓，是由于模型中除包含误差项外，还包含了有用信息项，即右侧第二项正是需要获取的。式中的每项系数是与运动元关的固定值，如式(3.2.12)所示，与物理参数有着确定的函数关系。 $K _ { F }$ 称零次项误差，反映了加速度计的偏置误差 $K _ { O } F _ { O }$ 和 $K _ { P } F _ { P }$ 称一次项误差，后四项称二次项误差。对于工作在大加速度环境条件下的加速度计，还应补充三次项误差，以提高模型的描述精度，此时静态数学模型为

$$
\begin{array}{r l} u _ {S S} & = K _ {F} + K _ {I} F _ {I} + K _ {O} F _ {O} + K _ {P} F _ {P} + K _ {I O} F _ {I} F _ {O} + K _ {O P} F _ {O} F _ {P} \\ & \quad + K _ {P I} F _ {P} F _ {I} + K _ {I I} F _ {I} ^ {2} + K _ {P P} F _ {P} ^ {2} + K _ {I I I} F _ {I} ^ {3} \end{array}\tag{3.2.14}
$$

虽然各误差系数与诸物理参数有确定的函数关系，但误差系数并不是通过这些函数关系计算出来的，而是通过实验室条件下的测试确定出来的。对于式(3. 2. 13)所示模型，利用重力加速度 $\pmb { g }$ 即可确定全部误差系数，此时

$$
F _ {I} = g _ {I}, \quad F _ {O} = g _ {O}, \quad F _ {P} = g _ {P}
$$

其中 gI..gO gp $\pmb { g }$ 在加速度计的输入轴、输出轴和摆轴上的投影值.用于测试的模型为

$$
\begin{array}{r l} u _ {S S} & = K _ {F} + K _ {I} g _ {I} + K _ {O} g _ {O} + K _ {P} g _ {P} + K _ {I O} g _ {I} g _ {O} + K _ {O P} g _ {O} g _ {P} \\ & \quad + K _ {P I} g _ {P} g _ {I} + K _ {I I} g _ {I} ^ {2} + K _ {P P} g _ {P} ^ {2} \end{array}\tag{3.2.15}
$$

若已测得各项误差系数，则将他们存入计算机，根据式 (3.2.13) 可精确求得比力值${ f _ { I } } \mathrm { { = } } \mathrm { { - } } { F _ { I } } \mathrm { { , } } { F _ { O } }$ 和 $\boldsymbol { F } _ { F }$ 由另外两只加速度计提供。

## 3.2.3 挠性摆式加速度计的动态误差数学模型

与机械式转子陀螺的动态误差相仿，加速度计的动态误差是指表壳(基座)角运动引起加速度计对加速度的测量误差。

设表壳 沿测量坐标系 诸轴具有角速度 $\omega _ { I } \circ \omega _ { \mathsf { O } } \mathsf { \Omega } , \omega _ { P }$ ，沿输入轴 方向有加速度 $a _ { I }$ . 为使分析工作得以简化，此处不考虑误差角 $\theta _ { I }$ 和 $\theta _ { P }$ 的影响，而只考虑输出角。 $\theta _ { \mathrm { { O } } }$ 的影响，则从测量坐标系 至摆组件坐标系的变换阵为

$$
\boldsymbol {C} _ {M} ^ {A} = \left[ \begin{array}{c c c} 1 & 0 & - \theta_ {O} \\ 0 & 1 & 0 \\ \theta_ {O} & 0 & 1 \end{array} \right]
$$

$$
\boldsymbol {\omega} _ {i A} ^ {A} = \boldsymbol {C} _ {M} ^ {A} \boldsymbol {\omega} _ {i S} ^ {M} + \boldsymbol {\omega} _ {S A} ^ {A} = \left[ \begin{array}{c c c} 1 & 0 & - \theta_ {O} \\ 0 & 1 & 0 \\ \theta_ {O} & 0 & 1 \end{array} \right] \left[ \begin{array}{c} \boldsymbol {\omega} _ {I} \\ \boldsymbol {\omega} _ {O} \\ \boldsymbol {\omega} _ {P} \end{array} \right] + \left[ \begin{array}{c} 0 \\ \dot {\theta} _ {O} \\ 0 \end{array} \right] = \left[ \begin{array}{c} \boldsymbol {\omega} _ {I} - \theta_ {O}   \boldsymbol {\omega} _ {P} \\ \boldsymbol {\omega} _ {O} + \dot {\theta} _ {O} \\ \boldsymbol {\omega} _ {P} + \theta_ {O}   \boldsymbol {\omega} _ {I} \end{array} \right]\tag{3.2.16}
$$

设摆组件的惯性张量为

$$
\left[ I _ {A} \right] = \left[ \begin{array}{c c c} I _ {I} & - I _ {I O} & - I _ {P I} \\ - I _ {I O} & I _ {O} & - I _ {O P} \\ - I _ {P I} & - I _ {O P} & I _ {P} \end{array} \right]
$$

则摆组件所具有的动量矩为

$$
\boldsymbol {H} _ {A} ^ {A} = \left[ I _ {A} \right] \boldsymbol {\omega} _ {i A} ^ {A} = \left[ \begin{array}{c c c} I _ {I} & - I _ {I O} & - I _ {P I} \\ - I _ {I O} & I _ {O} & - I _ {O P} \\ - I _ {P I} & - I _ {O P} & I _ {P} \end{array} \right] \left[ \begin{array}{c} \omega_ {I} - \theta_ {O}   \omega_ {P} \\ \omega_ {O} + \dot {\theta} _ {O} \\ \omega_ {P} + \theta_ {O}   \omega_ {I} \end{array} \right]
$$

展开上式，略去关于 $I _ { I O } \ 、 I _ { P I } \ 、 I _ { O P } \ 、 \theta _ { O }$ 和 $\dot { \theta } _ { O }$ 形成的二阶小量，得

$$
H _ {A (I A)} = I _ {I} \omega_ {I} - I _ {I} \theta_ {O} \omega_ {P} - I _ {I O} \omega_ {O} - I _ {P I} \omega_ {P}\tag{3.2.17a}
$$

$$
H _ {A (O A)} = I _ {O} \omega_ {O} + I _ {O} \dot {\theta} _ {O} - I _ {I O} \omega_ {I} - I _ {O P} \omega_ {P}\tag{3.2.17b}
$$

$$
H _ {A (P A)} = I _ {P} \omega_ {P} + I _ {P} \theta_ {O} \omega_ {I} - I _ {P I} \omega_ {I} - I _ {O P} \omega_ {O}\tag{3.2.17c}
$$

根据动量矩定理

$$
\left. \frac {\mathrm{d} \boldsymbol {H} _ {A}}{\mathrm{d} t} \right| _ {i} = \boldsymbol {M}
$$

式中.M为作用在摆组件输出轴上的外力矩。应用哥氏定理，并将各矢量向摆组件坐标系投影

$$
\left. \frac {\mathrm{d} \boldsymbol {H} _ {A}}{\mathrm{d} t} \right| _ {A} ^ {A} + \boldsymbol {\omega} _ {i A} ^ {A} \times \boldsymbol {H} _ {A} ^ {A} = \boldsymbol {M} ^ {A}
$$

将式 (3.2. 16) 代入上式，得

$$
\left[ \begin{array}{c} \dot {H} _ {A (I A)} \\ \dot {H} _ {A (O A)} \\ \dot {H} _ {A (P A)} \end{array} \right] + \left[ \begin{array}{c c c} 0 & - (\omega_ {P} + \theta_ {O} \omega_ {I}) & \omega_ {O} + \dot {\theta} _ {O} \\ \omega_ {P} + \theta_ {O} \omega_ {I} & 0 & - (\omega_ {I} - \theta_ {O} \omega_ {P}) \\ - (\omega_ {O} + \dot {\theta} _ {O}) & \omega_ {I} - \theta_ {O} \omega_ {P} & 0 \end{array} \right] \left[ \begin{array}{c} H _ {A (I A)} \\ H _ {A (O A)} \\ H _ {A (P A)} \end{array} \right] = \left[ \begin{array}{c} 0 \\ M _ {O} \\ 0 \end{array} \right]
$$

只讨论沿 $O _ { A }$ 轴的方程，即得加速度计的动力学方程:

$$
\dot {H} _ {A (O A)} + \left(\omega_ {P} + \theta_ {O} \omega_ {I}\right) H _ {A (I A)} - \left(\omega_ {I} - \theta_ {O} \omega_ {P}\right) H _ {A (P A)} = M _ {O}
$$

将式(3.2.17)代入上式，经整理并略去二阶小量，得

$$
\begin{array}{l} I _ {O} \left(\ddot {\theta} _ {O} + \dot {\omega} _ {O}\right) - \left(I _ {P} - I _ {I}\right) \omega_ {P} \omega_ {I} - I _ {I O} \left(\dot {\omega} _ {I} + \omega_ {O} \omega_ {P}\right) - I _ {O P} \left(\dot {\omega} _ {P} - \omega_ {I} \omega_ {O}\right) \\ - I _ {P I} \left(\omega_ {P} ^ {2} - \omega_ {I} ^ {2}\right) - \left(I _ {P} - I _ {I}\right) \left(\omega_ {I} ^ {2} - \omega_ {P} ^ {2}\right) \theta_ {O} = M _ {O} \end{array}\tag{3.2.18}
$$

式中. $M _ { O }$ o是沿输出轴作用在摆组件上的外力矩

$$
M _ {O} = M _ {i} + M _ {k} + M _ {\mathrm{d}} - D \dot {\theta} _ {O} - C \theta_ {O}
$$

式中. $C$ 为挠性接头的弹性恢复系数.D为浮液的阻尼系数. $M _ { \mathrm { d } }$ 为干扰力矩. $M _ { k }$ 为再平衡回路产生的反馈力矩， $M _ { i }$ 为加速度 $\textit { a } _ { I }$ 引起的惯性力矩。设加速度计摆性 $P { = } m L$ .

$$
M _ {i} = m L a _ {I}
$$

所以式 (3. 2. 18) 可写成

$$
I _ {O} \ddot {\theta} _ {O} + D \dot {\theta} _ {O} + C \theta_ {O} = m L a _ {I} + M _ {k} + M _ {\mathrm{d}} + M _ {C}\tag{3.2.19}
$$

式中

$$
\begin{array}{r l} M _ {C} = & - I _ {O} \dot {\omega} _ {O} + (I _ {P} - I _ {I}) \omega_ {P} \omega_ {I} + I _ {I O} (\dot {\omega} _ {I} + \omega_ {O} \omega_ {P}) + I _ {O P} (\dot {\omega} _ {P} - \omega_ {I} \omega_ {O}) \\ & + I _ {P I} (\omega_ {P} ^ {2} - \omega_ {I} ^ {2}) + (I _ {P} - I _ {I}) (\omega_ {I} ^ {2} - \omega_ {P} ^ {2}) \theta_ {O} \end{array}\tag{3.2.20}
$$

不考虑干扰力矩 $M _ { \mathrm { d } }$ 的影响，则由式(3. 2. 19)可求得加速度 $a _ { _ { I } }$ 的准确值

$$
a _ {I} = \frac {1}{m L} (I _ {O} \ddot {\theta} _ {O} + D \dot {\theta} _ {O} + C \theta_ {O} - M _ {k} - M _ {C})
$$

但在实际系统中 $a _ { I }$ 只能根据再平衡回赂的反馈力矩 $M _ { k }$ ，确定出来

$$
\hat {a} _ {I} = - \frac {M _ {k}}{m L}
$$

所以引起的测量误差为

$$
\Delta a _ {I} = \dot {a} _ {I} - a _ {I} = - \frac {I _ {O} \ddot {\theta} _ {O}}{m L} - \frac {D \dot {\theta} _ {O}}{m L} - \frac {C \theta_ {O}}{m L} + \frac {M _ {C}}{m L}
$$

上式右侧最后一项是由基座角运动引起的测量误差，定义此测量误差为加速度计的动态误差，根据式 (3.2.20)

$$
\begin{array}{r l} \delta a _ {I} & = \frac {M _ {C}}{m L} = - \frac {I _ {O}}{m L} \dot {\omega} _ {O} + \frac {I _ {P} - I _ {I}}{m L} \omega_ {P} \omega_ {I} + \frac {I _ {I O}}{m L} (\dot {\omega} _ {I} + \omega_ {O} \omega_ {P}) \\ & + \frac {I _ {O P}}{m L} (\dot {\omega} _ {P} - \omega_ {I} \omega_ {O}) + \frac {I _ {P I}}{m L} (\omega_ {P} ^ {2} - \omega_ {I} ^ {2}) + \frac {I _ {P} - I _ {I}}{m L} (\omega_ {I} ^ {2} - \omega_ {P} ^ {2}) \theta_ {O} \end{array}\tag{3.2.21}
$$

上式右侧第一项称为角加速度误差，第二项称为不等惯性误差，第三、四、五项称为惯性积误差，最后一项称为不等惯性耦合误差。各误差项都有明确的含意，下面对它们作简要说明。

## 1.角加速度误差

角加速度误差的表达式为

$$
\delta a _ {_ {I 1}} = - \frac {I _ {O}}{m L} \dot {\omega} _ {O}
$$

该项误差由表壳绕输出轴的角加速度 $\dot { \omega } _ { O }$ 引起，也可以看成是由表壳相对惯性空间旋转的角加速度 $\dot { \omega } _ { \mathrm { { O } } }$ 所形成的牵连惯性力矩作用在摆组件上所引起的，其中 $\natural { \frac { I _ { O } } { m L } }$ 为角加速度误差系数。

假设加速度计的摆性 $\displaystyle m L = 1 { \mathrm { g } } \cdot { \mathrm { c m } }$ ，摆组件绕输出轴的转动惯量 $I _ { O } { = } 2 4 \mathrm { g } \cdot \mathrm { c m } ^ { 2 }$ ，则角加速度误差系数达到 0. $0 2 4 \mathrm { g / ( r a d / s ^ { 2 } ) }$ o 若表壳绕输出轴的角加速度为 $\mathrm { 1 r a d / s ^ { 2 } }$ ，则所造成的测量误差为2. $4 \times 1 0 ^ { - 2 } \mathrm { g }$ 。可见角加速度误差是一项相当大的误差，在捷联惯导系统中应对此项误差作补偿。

## 2. 不等惯性误差

不等惯性误差的表达式为

$$
\delta a _ {I 2} = \frac {I _ {P} - I _ {I}}{m L} \omega_ {P} \omega_ {I}
$$

该项误差是由摆组件绕摆轴的转动惯量 $I _ { P }$ 与绕输入轴的转动惯量 $I _ { I }$ ，不相等，在角运动情况下形成的陀螺力矩不可对消而引起的，其中 $\frac { I _ { P } - I _ { I } } { m L }$ 称为不等惯性误差系数。

为了减少不等惯性误差，在结构设计中应尽量使摆组件对摆轴和对输入轴的转动惯量相等或接近，由于结构设计上易于实现这一要求，所以不等惯性误差系数一般较小，典型值约为$1 . 5 \mu \mathrm { g } / ( \mathrm { r a d } / \mathrm { s } ) ^ { 2 }$ 的量级。

## 3. 摆组件惯性积误差

摆组件惯性积误差的表达式为

$$
\delta a _ {I 3} = \frac {I _ {I O}}{m L} \left(\dot {\omega} _ {I} + \omega_ {O} \omega_ {P}\right) + \frac {I _ {O P}}{m L} \left(\dot {\omega} _ {P} - \omega_ {I} \omega_ {O}\right) + \frac {I _ {P I}}{m L} \left(\omega_ {P} ^ {2} - \omega_ {I} ^ {2}\right)
$$

该顶误差是由摆组件惯性积 $I _ { I O \setminus I _ { O P } }$ 和 $I _ { P I }$ 在角运动条件下形成离心惯性力矩引起的。其中，$\frac { I _ { I O } } { m L } . \frac { I _ { O P } } { m L } . \frac { I _ { P I } } { m L }$ 称为摆组件惯性积误差系数。

为了减少摆组件惯性积误差，在结构设计中应尽量使摆组件对输入轴、输出轴和摆轴的质量分布对称，以使摆组件对这些轴的惯性积减少到最低限度，摆组件结构经精心设计后，惯性积误差一般是很小的。

## 4. 不等惯性耦合误差

不等惯性耦合误差的表达式为

$$
\delta a _ {I 4} = \frac {I _ {P} - I _ {I}}{m L} (\omega_ {I} ^ {2} - \omega_ {P} ^ {2}) \theta_ {O}
$$

该项误差是由摆组件绕摆轴的转动惯量 $I _ { P }$ 与绕输入轴的转动惯量 $I _ { I }$ ，不相等，并通过加速度计输出角。 $\theta _ { \mathrm { { O } } }$ 。的耦合引起的。由于加速度计工作在力反馈状态，根据式(3.2.9) ，加速度计的稳态输出角。 $\theta _ { O }$ 。与输入加速度的关系为

$$
\theta_ {O} = \frac {m L}{K} a _ {I}
$$

式中 $K { = } C { + } K _ { s } K _ { a } K _ { R } K _ { T }$ '所以

$$
\delta a _ {I 4} = \frac {I _ {P} - I _ {I}}{K} a _ {I} (\omega_ {I} ^ {2} - \omega_ {P} ^ {2})\tag{3.2.22}
$$

式中， $\frac { I _ { P } - I _ { I } } { K }$ 称为不等惯性耦合误差系数。

为了减少不等'惯性耦合误差，一方面应使摆组件转动惯量 $I _ { P }$ 与 $I _ { I }$ ，相等或尽量接近相等，另一方面应尽量提高再平衡回路的增益。由于这两方面的工作都较容易实现，所以不等惯性耦合误差系数一般是很小的.

用式(3.2.22)代替式(3. 2. 21)的右侧最后一项，经整理后得

$$
\begin{array}{r l} \delta a _ {I} & = \frac {I _ {I O}}{m L} \dot {\omega} _ {I} - \frac {I _ {O}}{m L} \dot {\omega} _ {O} + \frac {I _ {O P}}{m L} \dot {\omega} _ {P} + \frac {I _ {P} - I _ {I}}{m L} \omega_ {P} \omega_ {I} + \frac {I _ {I O}}{m L} \omega_ {O} \omega_ {P} - \frac {I _ {O P}}{m L} \omega_ {I} \omega_ {O} \\ & + \frac {I _ {P I}}{m L} \omega_ {P} ^ {2} - \frac {I _ {P I}}{m L} \omega_ {I} ^ {2} + \frac {I _ {P} - I _ {I}}{K} a _ {I} \omega_ {I} ^ {2} - \frac {I _ {P} - I _ {I}}{K} a _ {I} \omega_ {P} ^ {2} \end{array}\tag{3.2.23}
$$

记

$$
C _ {I} = \frac {I _ {I O}}{m L}\tag{3.2.24a}
$$

$$
C _ {O} = - \frac {I _ {O}}{m L}\tag{3.2.24b}
$$

$$
C _ {P} = \frac {I _ {O P}}{m L}\tag{3.2.24c}
$$

$$
C _ {P I} = \frac {I _ {P} - I _ {I}}{m L}\tag{3.2.24d}
$$

$$
C _ {O P} = \frac {I _ {I O}}{m L}\tag{3.2.24e}
$$

$$
C _ {I O} = - \frac {I _ {O P}}{m L}\tag{3.2.24f}
$$

$$
C _ {P P} = \frac {I _ {P I}}{m L}\tag{3.2.24g}
$$

$$
C _ {I I} = - \frac {I _ {P I}}{m L}\tag{3.2.24h}
$$

$$
C _ {I I I} = \frac {I _ {P} - I _ {I}}{K}\tag{3.2.24i}
$$

$$
C _ {I P P} = - \frac {I _ {P} - I _ {I}}{K}\tag{3.2.24j}
$$

则式(3.2. 23)可写成用综合系数表示的形式

$$
\begin{array}{r l} \delta a _ {I} & = C _ {I} \dot {\omega} _ {I} + C _ {O} \dot {\omega} _ {O} + C _ {P} \dot {\omega} _ {P} + C _ {P I} \omega_ {P} \omega_ {I} + C _ {O P} \omega_ {O} \omega_ {P} + C _ {I O} \omega_ {I} \omega_ {O} \\ & + C _ {P P} \omega_ {P} ^ {2} + C _ {I I} \omega_ {I} ^ {2} + C _ {I I I} a _ {I} \omega_ {I} ^ {2} + C _ {I P P} a _ {I} \omega_ {P} ^ {2} \end{array}\tag{3.2.25}
$$

(3.2.24)虽然给出了综合系数与物理参数之间的函数关系，但这些系数仍然在实验室条件下通过测试来确定，测试必须在摇摆台上进行，以产生角速度和角加速度激励。测得这些系数后存人导航计算机，捷联惯导根据陀螺提供的角速度测量值和加速度计提供的加速度测量值，代入式(3.2. 25)所列模型，即可计算出加速度计的动态误差补偿量，用于对加速度计的输出作校正，从而消除由角运动引起的加速度计动态误差。

## 习题一

1-1 惯性仪器是指哪两类仪器，分别用来测 运载体的什么运动?

1-2 机械式惯性仪器具有哪两类确定性误差，分别由什么引起?

1-3 设自由陀螺位于赤道，初始时刻转子自转轴处于铅垂位置，地球上的观察者在 24 小时内看到自转轴做什么运动?这种现象称为陀螺的什么运动?

1-4 挠性陀螺达到动力调谐时，哪两种力矩互相平衡?

1-5 单自由度积分陀螺的动态误差是由哪些误差项组成的?分别简要说明引起各项误差的原因。

1-6 性陀螺的动态误差是由哪些误差项组成的?分别简要说明引起各项误差的原因。当转子的转速偏离调谐转速时会引起哪些误差?

1-7 当单自由度陀螺仪沿输入轴方向旋转时会产生什么力矩?说明该力矩的施力者和受力者

1-8 挠性陀螺的挠性杆在抗扭刚度和抗弯刚度上有什么不同要求?

1-9 若以挠性陀螺为惯性器件，平台式惯导和捷联式惯导对陀螺的动态误差和静态误差补偿有何要求?简要说明该两系统中 所起的作用，陀螺工作在开环还是闭环状态。

1-10 单轴速率转台的旋转轴调整在铅垂方向，光纤陀螺安装在转台台面 ，陀螺敏感轴沿铅垂方向，向上为正。设转台所在地纬度为北纬 $3 0 ^ { \circ }$ ，不计陀螺漂移。分别计算下述条件下的陀螺输出: (1)转台静止;(2)台以 $1 ^ { \circ } / \mathrm { s }$ 的角速率顺时针旋转;(3)转台以 /s 的角速率逆时针旋转。

1-11 单自由度陀螺的动力学方程为

$$
J _ {0} \ddot {\alpha} _ {0} - \omega_ {I} H = M _ {0}
$$

式中， $J _ { 0 }$ 为陀螺组件绕输出轴的转动惯量， $, \alpha _ { 0 }$ 为输出角， $, \omega I$ 为输入角速度， 为转子角动量 为输出轴外作用力矩。若陀螺为积分陀螺，设陀螺的力矩阻尼系数为 $D _ { \circ }$

(1)求陀螺的传递函数及时间常数

(2) 若陀螺工作在力反馈状态，陀螺的力反馈回路放大倍数为 力矩器的力矩系数为 $k _ { m }$ ，求陀螺的传递函数。

1-12 设双自由度陀螺仪(参阅图 .2. 4)绕外环轴和内环轴 转动惯量均为 1=4 ·厘米· 在内环作用有常值干扰力矩 $M _ { y } = 1$ 克·厘米，转子角动 $H { = } 1 0 0 0 0$ 克·厘米·秒。计算：

(1)陀螺绕外环轴的漂移

(2) 陀螺绕内环轴的偏置角及振动频率和振幅。

1-13 使用双轴位置转台标定激光陀螺的脉冲当量，台台面调整在水平位置，陀螺安装时敏感轴垂直转台台面，测试点纬度为北纬 $L _ { ☉ }$ 0

(1)若在转台台面反时针旋转 周的时间 TC分钟内，对陀螺输出的脉冲计数为 ，求脉冲 量;

(2) 若台面反时针旋转一周后再顺时针旋转一周，相应的脉冲计数分别为 $N _ { \sqrt { \gtrsim } }$ 和 $N _ { \parallel \parallel }$ ，旋转 周所用时间均为T(分钟) ，求脉冲当量;

上述两种方法中，哪种方法测试精度高?为什么?

14 左因为采用单轴伺服转台对单自由度陀螺作伺服转台测试的原理图，用于标定单自由度陀螺的静态漂移误差系数 自由度陀螺固定安 在转台面上，陀螺敏感轴沿转台轴方向，陀螺输 放大并反相后控制平 旋转

(1)求系统闭合 陀螺的输出角速度;

(2) 求转台旋转角速度自与陀螺漂移间的 系。

![](images/f390b47e3f6c29713faf3e6a95f6f836bad6bb182869a58041d040e976dfdac3.jpg)  
14 图单轴伺服转台示意图

