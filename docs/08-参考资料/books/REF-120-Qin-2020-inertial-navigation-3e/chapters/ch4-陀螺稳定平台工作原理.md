# 第4章 陀螺稳定平台工作原理

## 第二篇陀螺稳定平台

陀螺具有定轴性和进动性，即具有相对惯性空间指向保持不变的能力和按照要求的规律相对惯性空间旋转的能力，因此可利用陀螺来控制被控对象的角运动。控制角运动包含两个含义一是隔离运载体的角运动对被控对象的角运动的影响，例如船舶上的同步卫星接收天线，当船舶受风浪作用而摇晃时，天线能始终指向同步卫星;二是能使被控对象按指令即要求的规律旋转，指向始终跟踪变动着的方向，例如监测火箭发射的地面光学跟踪系统，光轴始终跟踪观察点与火箭的连线，而该连线的指向在不断地变化着。

以陀螺为敏感元件，能隔离基座的角运动并能使被控对象按指令旋转的机电控制系统称为陀螺稳定平台。此处称谓的平台具有泛指含意，是一种统称。对于不同的使用场合，陀螺稳定平台的精度和被控对象的大小是不一样的，习惯称谓也有所不同。如平台式惯导系统中使用的稳定平台要求精确模拟惯导系统所采用的导航坐标系，精度要求达到角分级甚至角秒级，此类平台习惯上称之为惯性平台。又如夭线、炮塔等稳定系统，被控对象体积和重量都十分庞大，精度要求不必像惯性平台那样高，控制的旋转轴也可能少于三轴，此类稳定平台习惯上称之为陀螺稳定装置。

从上述简单说明中可看出陀螺稳定平台有两个基本功能=一是稳定功能，即对外界干扰起对消作用，如基座角运动是一种外界干扰，在稳定平台与基座的轴承连接处将引起摩擦力矩作用在平台上，或由于几何约束关系台体被强制带动跟随基座一起旋转，稳定平台能自动产生卸荷力矩对消摩擦力矩，同时能产生适当的反旋转对消被基座带动的强制旋转;二是跟踪功能，即能跟踪指令，按要求的角速度旋转，确保平台的坐标轴指向要求的方位。

## 第4章 陀螺稳定平台工作原理

## 4. 1 各类稳定平台简介

陀螺稳定平台的稳定作用之一是平台能自动产生卸荷力矩对消干扰力矩。卸荷力矩由两类不同的力矩组成:一是由固联在台体上的陀螺产生的陀螺力矩;二是由平台的伺服回路产生的伺服力矩。按照这两类力矩在卸荷力矩中所起作用的不同，可对陀螺稳定平台作如下分类:

(1)干扰力矩全部由陀螺力矩对消，则此类稳定平台称为直接式陀螺稳定平台。

(2) 陀螺安装在基座上，陀螺感测的基座角运动信息经处理后用来控制平台，干扰力矩全部由伺服力短对消，则此类稳定平台称为间接式陀螺稳定平台。

安装在台体上的陀螺属 角位置陀螺，如双 由度液浮角位置陀螺、静电陀螺、挠性陀螺等，此类陀螺不产生陀螺力矩，干扰力矩全部由伺服力矩对消，则此类稳定平台称为指示式陀螺稳定平台。

(4) 陀螺力矩只是在平台系统的过渡过程中参与对消干扰力矩，当平台系统达到稳态后，干扰力矩全部由伺服力矩对消，则此类稳定平台称为动力式陀螺稳定平台

(5) 陀螺力矩虽然在 台系统的过渡过程中参与了对消 扰力矩，但所起作用很小， 平台系统达到稳态后，干扰力矩全部由伺服力矩对消，则此类稳定平台称为指示-动力式陀螺稳定平台。采用单自由度液浮积分陀螺和速率陀螺为敏感元件的稳定平台就属于此类系统，其中陀螺的主要作用是感测角速度。

下面通过实例分析，进 步对各类陀螺稳定平台作说明

## 4. 1. 1 直接式陀螺稳定平台

4. 1. 是单轨列车设计方案，该型列车本质上就是一种直接式陀螺稳定平台，工作原理分析如下。

![](images/e4e3e462cee2fbe2b843aa4d31c257a4909ef8ed62cd9db05252845f557f0f69.jpg)  
4. 1. 单轨列车设计方

iD $\mathcal { X } _ { b } \mathcal { Y } _ { b } \mathcal { Z } _ { b }$ 为车体坐标系，简记 该坐标系与车厢固联 $x _ { b } \backprime \texty _ { b } \backprime z _ { b }$ 取向分别为右、前、上。记$x y z$ 为陀螺框架坐标系，简记 $G$ ，相对车体坐标系绕 $x _ { b }$ 有偏角卢 $\beta$ C

设车厢的旋转角速度沿 $x _ { b } \ldots y _ { b } \ldots z _ { b }$ 的分量分别为 $\omega _ { x } \mathbf { \nabla } _ { \mathbf { \nu } } \cdot \boldsymbol { \omega } _ { y } \mathbf { \nabla } _ { \mathbf { \nu } } \omega _ { z }$ ，转子自转产生的角动量为 ，框架组件(包括转子和框架)的惯性张量为 $[ I ] = \mathrm { d i a g } [ I _ { x } ~ I _ { y } ~ I _ { z } ]$ 车厢作用有倾倒力矩 $M _ { \mathrm { d y } }$ ，由此引起的框架进动角速度为 $\dot { \boldsymbol { \beta } }$ 。该陀螺稳定系统实质上是单自由度陀螺，倾倒力矩的对消过程大致在倾倒力矩 $M _ { \mathrm { d } y }$ 此处假设沿列车前向， GP图中穿入纸面方向)作用下，陀螺发生进动，引起的进动角速度 $\dot { \boldsymbol { \beta } }$ 向左，即 $x _ { b }$ 负方向，此进动角速度引起陀螺力矩 ${ \pmb { M } } _ { \mathrm { g } } = { \pmb { H } } \times { \dot { \pmb { \beta } } }$ 方向沿列车后向，即图中穿出纸面方向，因此陀螺力矩对倾倒力矩直接起到了对消作用。

以框架组件为分析对象，框架组件的总动量矩为

$$
\boldsymbol {H} _ {G} ^ {b} = \boldsymbol {H} ^ {b} + [ \boldsymbol {I} ] (\boldsymbol {\omega} _ {i b} ^ {b} + \boldsymbol {\omega} _ {l G} ^ {b}) = \left[ \begin{array}{c} 0 \\ H \sin \beta \\ H \cos \beta \end{array} \right] + \left[ \begin{array}{c c c} I _ {x} & 0 & 0 \\ 0 & I _ {y} & 0 \\ 0 & 0 & I _ {z} \end{array} \right] \left[ \begin{array}{c} \omega_ {x} - \dot {\beta} \\ \omega_ {y} \\ \omega_ {z} \end{array} \right] = \left[ \begin{array}{c} I _ {x} (\omega_ {x} - \dot {\beta}) \\ H \sin \beta + I _ {y} \omega_ {y} \\ H \cos \beta + I _ {z} \omega_ {z} \end{array} \right]
$$

根据动量矩定理，并应用哥氏定理，得

$$
\left. \frac {\mathrm{d} \boldsymbol {H} _ {G}}{\mathrm{d} t} \right| _ {b} ^ {b} + \boldsymbol {\omega} _ {i b} ^ {b} \times \boldsymbol {H} _ {G} ^ {b} = \boldsymbol {M} ^ {b}
$$

GP

$$
\left[ \begin{array}{c} I _ {x} (\dot {\omega} _ {x} - \ddot {\beta}) \\ H \dot {\beta} \cos \beta + I _ {y} \dot {\omega} _ {y} \\ - H \dot {\beta} \sin \beta + I _ {z} \dot {\omega} _ {z} \end{array} \right] + \left[ \begin{array}{c c c} 0 & - \omega_ {z} & \omega_ {y} \\ \omega_ {z} & 0 & - \omega_ {x} \\ - \omega_ {y} & \omega_ {x} & 0 \end{array} \right] \left[ \begin{array}{c} I _ {x} (\omega_ {x} - \dot {\beta}) \\ H \sin \beta + I _ {y} \omega_ {y} \\ H \cos \beta + I _ {z} \omega_ {z} \end{array} \right] = \left[ \begin{array}{c} M _ {x} ^ {b} \\ M _ {\mathrm{dy}} \\ M _ {z} ^ {b} \end{array} \right]
$$

展开上式得

$$
\begin{array}{l} I _ {x} (\dot {\omega} _ {x} - \ddot {\beta}) - \omega_ {z} (H \sin \beta + I _ {y} \omega_ {y}) + \omega_ {y} (H \cos \beta + I _ {z} \omega_ {z}) = M _ {x} ^ {b} \\ H \dot {\beta} \cos \beta + I _ {y} \dot {\omega} _ {y} + \omega_ {z} I _ {x} (\omega_ {x} - \dot {\beta}) - \omega_ {x} (H \cos \beta + I _ {z} \omega_ {z}) = M _ {\mathrm{dy}} \\ - H \dot {\beta} \sin \beta + I _ {z} \dot {\omega} _ {z} - \omega_ {y} I _ {x} (\omega_ {x} - \dot {\beta}) + \omega_ {x} (H \sin \beta + I _ {y} \omega_ {y}) = M _ {z} ^ {b} \end{array}
$$

为便于说明问题，假设 $\omega _ { x } = \dot { \omega } _ { x } = \omega _ { z } = \dot { \omega } _ { z } = 0$ ，则上述三式成

$$
- I _ {x} \ddot {\beta} + \omega_ {y} H \cos \beta = M _ {x} ^ {b}\tag{4.1.1a}
$$

$$
H \dot {\beta} \cos \beta + I _ {y} \dot {\omega} _ {y} = M _ {\mathrm{dy}}\tag{4.1.1b}
$$

$$
H \dot {\beta} \sin \beta = - M _ {z} ^ {b} + \omega_ {y} I _ {x} \dot {\beta}\tag{4.1.1c}
$$

由式 (4. 1. 1b)

$$
H \dot {\beta} \cos \beta = M _ {\mathrm{dy}} - I _ {y} \dot {\omega} _ {y}
$$

上式中 $H \dot { \beta } \cos \beta$ 是由陀螺角动量 和进动角速度 $\mathbf { \dot { \boldsymbol { { \beta } } } }$ 形成的陀螺力矩在 $y _ { b }$ 轴上的分量，由转子产生，经框架轴传递，作用在车厢上，对倾倒力矩起对消作用。又由式(4. 1. 1c)知，陀螺力矩沿 ${ \mathcal { Z } } _ { b }$ 轴的分量 $H \dot { \beta } \mathrm { s i n } \beta$ 由车轮与导轨间的抗扭转约束力矩平衡。

直接式陀螺稳定平台存在的问题是

(1)只有 $\dot { \boldsymbol { \beta } } \ne 0$ 。时陀螺力矩才存在，如果倾倒力矩长时间存在，则 $\beta$ 将越来越大，当 $\beta { = } 9 0 ^ { \circ }$ 时， $\dot { H _ { \beta \mathrm { c o s } \beta } } { = } 0$ ，即陀螺力矩失去卸荷作用，所以图 4. 1. 所示单轨列车实质上是一个正反馈不稳定系统，要成为稳定系统，必须采取措施使 $\beta = 0$ ，即在对消倾倒力矩的同时应使倾倒角回零。

(2) 只有当角动量 非常巨大时才能产生足够大的陀螺力矩，所以设施非常笨重。

单轨列车曾经一度引起过关注，1912 年曾研制出可乘坐 40 人的实验样车，但由于安全因素等原因，单轨列车并没有得到实际应用。直接式陀螺稳定平台的另一项应用实例是舰船的减摇装置，如图 4. 1. 所示，该装置的作用是能减轻舰船的横摇，其工作原理分析与单轨列车分析类似，读者可自行完成。

![](images/603f5411ca53bfefbba355bd23faa57af830588162b3d963c2539394b710cbcc.jpg)  
4. 1. 舰船减摇装置

## 4. 1. 2 间接式陀螺稳定平台

4. 1. 是间接式陀螺稳定平台的典型结构。该系统中，垂直陀螺作为角位置敏感元件安装在基座上，被稳定对象安装在平台台体上，台体由两个环架支承，当稳定平台和陀螺都处于中立位置时，环架轴与陀螺的框架轴对应平行。

![](images/e8071840f50190b67b4a3980f9ac5a6cc0a3ab25ea932933df6fb8b8c85d5e28.jpg)  
4. 1. 双轴间接式陀螺稳定平台

陀螺的内、外框架轴上的电位计式角度传感 $P _ { 4 }$ 和 $P _ { 2 }$ 拾取基座角位移信息，以此信息为基准信息与稳定平台相应轴上的电位计输出作比较，所得比较量经放大后馈人相应轴上的力矩马达，马达拖动稳定平台旋转，确保稳定平台始终跟随垂直陀螺，由于陀螺的定轴性，陀螺的角动量始终指向铅垂位置，所以稳定平台的台面始终处于水平位置，隔离了基座的水平角运动。现以平台外环轴为对象分析稳定平台的工作原理.

(1)设基座无角运动，当沿稳定平台外环轴作用有干扰力矩 $M _ { \mathrm { d } }$ 时，平台绕 $y _ { b }$ 轴产生旋转角速度 $\dot { \alpha }$ ，从而带动电位计 $P _ { 1 }$ 的电刷沿绕组转动 $\alpha$ $P _ { 1 }$ 输出与 成比例的电信号。由于陀螺的 $\alpha$ 定轴性，陀螺外环轴上的电位计 $P _ { 2 }$ 输出为零，所以进入 $A _ { 1 }$ 的信号仅为 $P _ { 1 }$ 的输出信号，该信号经放大后馈入力矩马达 $\mathrm { S M } _ { 1 } , \mathrm { S M } _ { 1 }$ ，产生抑制 $\alpha$ 增大的稳定力矩 $M _ { m 1 }$ .当 $M _ { m 1 }$ 与 $M _ { \mathrm { d } }$ 相等时， $, \alpha$ 角不再增大，从而保持了平台绕外环轴的稳定。

(2) 若基座绕横滚轴 $y _ { b }$ 滚转，产生横滚角 $\gamma ,$ 由于陀螺的定轴性，电位计 $P _ { 2 }$ 输出与 $\gamma$ 成正比的电信号 $E _ { 2 }$ .由于存在摩擦等原因，稳定平台被基座带动也随之绕 $y _ { b }$ 轴旋转产生平台横滚角$\gamma ^ { \prime }$ ，显然r $\gamma ^ { \prime } \ne \gamma$ 电位计 $P _ { 1 }$ 输出与 $\gamma ^ { \prime }$ '成正比的电信号 $E _ { 1 }$ " 比较点 $C _ { 1 }$ 输出信号 $E _ { 1 } - E _ { 2 } \neq 0$ ，经 $A _ { 1 }$ 放大后馈人力矩马达，力矩马达拖动平台绕 $y _ { b }$ 旋转，当 $E _ { 1 } - E _ { 2 } = 0$ 时，即 $\gamma ^ { \prime } = \gamma$ 时，力矩马达停止旋转，从而达到了隔离基座角运动的目的。

稳定平台绕俯仰轴的稳定过程的分析与上述分析相仿，读者可自行分析。

间接式陀螺稳定平台在工程中的应用是很广泛的，其最大优点是不同的被稳定对象可共用一个信息参考基准，这对降低成本，减小重量和体积是十分有利的。但这类稳定平台达到的精度一般都不高，如图 4. 1. 所示双轴水平稳定平台，其精度很难达到 0.2

## 4.1.3 动力式陀螺稳定平台

动力式陀螺稳定平台是工程上最常用的一类系统，这类平台可以是单轴的、双轴的和三轴的。苏联的 -15 型自动驾驶仪的航向稳定器可将重量和转动惯量很大的轰炸瞄准具稳定在一定航向上; $\mathrm { { \ ; { \it ~ I I P } } }$ rB型双轴动力式陀螺平台可以给飞机上众多设备提供飞机的姿态角信息;剑1-15 型自动驾驶仪中使用了一个三轴动力式陀螺稳定平台，它可以作为飞机偏航角、姿态角的角度传感器。此外，动力式陀螺稳定平台也可用于坦克火炮的稳定，航空摄像机和雷达天线稳定等。

无论是双轴的还是三轴的动力式陀螺稳定平台，它们的工作原理、系统设计都以单轴稳定平台为基础，所以此处对单轴动力式陀螺稳定平台做较详细的介绍。

## 1.稳定回路工作原理(几何稳定状态下的工作原理)

4. 1. 为动力式单轴陀螺稳定平台工作原理图。台体上安装了一个单自由度陀螺仪，陀螺仪信号器输出经放大后馈人力矩马达，马达带动平台旋转。该稳定平台可理解成带有稳定回路的双自由度陀螺仪，陀螺的外环轴为稳定轴，稳定回路由内环轴上的信号器、放大器、力矩马达和齿轮减速器组成。

设沿稳定轴负方向作用有干扰力矩 $M _ { \mathrm { d } x }$ 由于陀螺的进动作用，角动量 倒向 $M _ { \mathrm { d } x }$ .陀螺产生输出角 $\beta ,$ 旋转方向与陀螺输出轴 的指向相反，记此 $\beta$ 角为负值，信号器将 $\beta$ 变换成电压信号，经放大处理后馈入力矩马达，信号接人力矩马达时正负方向确定的原则是 输出角 $\beta$ 为负时，力矩马达产生正向力矩(与 $x _ { O }$ 轴指向相同) ;输出角 $\beta$ 卢为正时，力矩马达产生负向力矩。即

$$
M _ {m} = - K \beta\tag{4.1.2}
$$

式中 $K = k _ { m } k _ { a } k _ { u } , k _ { u }$ 为信号器的传递系数. $k _ { a }$ 为放大器的放大倍数， $k _ { m }$ 为力矩电机的力矩系数。所以对于 $M _ { \mathrm { d } x }$ 为负向干扰力矩时 $, \beta$ 角为负值，力矩马达产生的卸荷力矩 $M _ { m }$ 现为正向力矩，对

$M _ { \mathrm { d } x }$ 起对 肖作用，当 $M _ { m } < | M _ { \mathrm { d } x } |$ 时， $| \beta |$ |继续增大 $M _ { m }$ 也随之增大，当 $M _ { m } = | M _ { \mathrm { d } x } |$ 时， $| \beta |$ |停止增大，干扰力矩被完全对消，实现了平台台体的角位置稳定。读者可自行分析当干扰力矩沿稳定轴正向时平台的稳定过程。

![](images/0c52e70f6b642f70902225ff17522e548632222c0d81f8beda668fe38136d591.jpg)  
4. 1. 动力式单轴陀螺稳定平台

2. 修正回路工作原理(空间积分状态下的工作原理)

修正回路的作用是?使平台跟踪给定的指令，相对惯性空间以该指令角速度旋转。

设指令角速度为 $\omega _ { \mathrm { c m d } }$ ，将该指令角速度折算成指令电流

$$
i _ {\mathrm{cmd}} = k _ {i \omega} \omega_ {\mathrm{cmd}}
$$

式中， $, k _ { i \omega }$ 根据力矩器额定电流和指令角速度最大值确定，该指令电流馈人陀螺力矩器，产生的指令力矩为

$$
M _ {\mathrm{cmd}} = - k _ {T} k _ {i \omega} \omega_ {\mathrm{cmd}}
$$

式中 $\boldsymbol { k } _ { T }$ 为力矩器力矩系数。指令电流接人力矩器时正负方向确定的原则是:若 cmd沿稳定轴 $\omega _ { \mathrm { c m d } }$ 正向，则产生的指令力矩应沿陀螺输出轴的负向;若 $\omega _ { \mathrm { c m d } }$ 沿稳定轴负向，则产生的指令力矩应沿陀螺输出轴的正向。两者符号正好相反。

由于在稳定轴上存在很大的摩擦力矩，且被稳定对象(台体)具有很大的转动惯量，所以在$M _ { \mathrm { c m d } }$ 作用下，陀螺不会绕稳定轴进动，而只引起陀螺框架组件绕 $y _ { O }$ 轴做一般刚体转动:

$$
I _ {G} \ddot {\beta} = M _ {\mathrm{cmd}} - D _ {\beta} \dot {\beta}\tag{4.1.3}
$$

对应于 $\omega _ { \mathrm { c m d } }$ 为正，则输出角 $\beta$ 为负，按稳定回路中力矩马达馈人信号的正负向接法，此时力矩马达产生正向力矩 $M _ { m }$ 驱动平台旋转，平台转角 $\phi$ 满足如下方程:

$$
J _ {P} \ddot {\phi} = M _ {m} - H \dot {\beta}\tag{4.1.4}
$$

式中， $H \dot { \boldsymbol { \beta } }$ 为 $\dot { \boldsymbol { \beta } }$ 引起的陀螺力矩。由于产生了< $\dot { \phi }$ ，应将陀螺的框架组件的动力学方程式(4. 1. 3)改写成

$$
I _ {G} \ddot {\beta} = M _ {\mathrm{cmd}} + H \cos \beta \dot {\phi} - D _ {\beta} \dot {\beta}\tag{4.1.5}
$$

式中 $H \mathrm { c o s } \beta \dot { \phi }$ 是由 $\dot { \phi }$ 引起的陀螺力矩。可以证明，当

$$
M _ {\mathrm{cmd}} + H \cos \beta \dot {\phi} = 0
$$

时 $ \stackrel { \cdot } { \beta } = 0 , \stackrel { \cdot } { \beta } = 0$ ， $\beta$ 卢达到稳态值。此时

$$
H \cos \beta \dot {\phi} = - M _ {\mathrm{cmd}} = k _ {T} k _ {i \omega} \omega_ {\mathrm{cmd}}\tag{4.1.6}
$$

由于输出角 $\beta$ 是小角， $\cos \beta { \approx } 1$ ，所以由式 (4. 1. 6)

$$
\dot {\phi} = \frac {k _ {T} k _ {i \omega}}{H} \omega_ {\mathrm{cmd}}
$$

设计中取

$$
\frac {k _ {T} k _ {i \omega}}{H} = 1\tag{4.1.7}
$$

则1 $\dot { \phi } { = } \omega _ { \mathrm { c m d } }$ ，即平台能跟随指令角速度旋转，其中 $\cdot \frac { k _ { T } k _ { i \omega } } { H }$ 称为陀螺的刻度因数。从上述分析可以看出，陀螺输出角 $\beta$ 越小，式(4. 1. 7)近似度就越好，平台跟踪精度就越高。

从式(4. 1. 4)可看出，当陀螺还未达到稳定时，即 $\dot { \boldsymbol { \beta } } \ne 0$ 时，陀螺力矩 $\dot { H \beta }$ 也参与了对平台的驱动，而一旦陀螺达到稳态，即 ${ \dot { \boldsymbol { \beta } } } { = } 0$ 陀螺力矩消失，平台完全由伺服力矩驱动，所以此类平台定义为动力式陀螺稳定平台.

## 3. 稳定性分析

若考虑作用在稳定轴上的阻尼力矩和干扰力矩，并考虑式(4. 1. 2) ，则式(4. 1. 4)应改写成

$$
J _ {P} \ddot {\phi} = - K \beta + M _ {d x} - H \dot {\beta} - D _ {\phi} \dot {\phi}\tag{4.1.8}
$$

根据式(4. 1. 5)和式(4. 1. 8) ，得平台和陀螺动力学方程的拉氏变换

$$
\begin{array}{c} (J _ {P} s ^ {2} + D _ {\phi} s) \phi (s) + (H s + K) \beta (s) = M _ {\mathrm{d} x} (s) \\ - H s \phi (s) + (I _ {G} s ^ {2} + D _ {\beta} s) \beta (s) = M _ {\mathrm{cmd}} (s) \end{array}
$$

写成矩阵形式

$$
\left[ \begin{array}{c c} J _ {P} s ^ {2} + D _ {\phi} s & H s + K \\ - H s & I _ {G} s ^ {2} + D _ {\beta} s \end{array} \right] \left[ \begin{array}{c} \phi (s) \\ \beta (s) \end{array} \right] = \left[ \begin{array}{c} M _ {\mathrm{dx}} (s) \\ M _ {\mathrm{cmd}} (s) \end{array} \right]
$$

系统的特征方程为

$$
\left| \begin{array}{c c} J _ {P} s ^ {2} + D _ {\phi} s & H s + K \\ - H s & I _ {G} s ^ {2} + D _ {\beta} s \end{array} \right| = 0
$$

展开行列式

$$
\begin{array}{c} (J _ {P} s ^ {2} + D _ {\phi} s) (I _ {G} s ^ {2} + D _ {\beta} s) + (H s + K) H s = 0 \\ J _ {P} I _ {G} s ^ {3} + (J _ {P} D _ {\beta} + I _ {G} D _ {\phi}) s ^ {2} + (D _ {\phi} D _ {\beta} + H ^ {2}) s + K H = 0 \end{array}
$$

即

$$
a _ {0} s ^ {3} + a _ {1} s ^ {2} + a _ {2} s + a _ {3} = 0
$$

式中 $, a _ { 0 } = J _ { P } I _ { G } , a _ { 1 } = J _ { P } D _ { \beta } + I _ { G } D _ { \phi } , a _ { 2 } = D _ { \phi } D _ { \beta } + H ^ { 2 } , a _ { 3 } = K H$ 根据劳斯霍尔维茨判据，系统稳定的条件为

$$
a _ {1} > 0, \quad \left| \begin{array}{c c} a _ {1} & a _ {3} \\ a _ {0} & a _ {2} \end{array} \right| > 0
$$

$a _ { 1 } = J _ { P } D _ { \beta } + I _ { G } D _ { \phi } > 0$ 自然满足:

$$
\left| \begin{array}{c c} a _ {1} & a _ {3} \\ a _ {0} & a _ {2} \end{array} \right| = \left| \begin{array}{c c} J _ {P} D _ {\beta} + I _ {G} D _ {\phi} & K H \\ J _ {P} I _ {G} & D _ {\phi} D _ {\beta} + H ^ {2} \end{array} \right| = (J _ {P} D _ {\beta} + I _ {G} D _ {\phi}) (D _ {\phi} D _ {\beta} + H ^ {2}) - K H J _ {P} I _ {G} > 0
$$

由于 $D _ { \phi } D _ { \beta } { \ll } H ^ { 2 }$ 所以要确保陀螺稳定平台稳定，应满足

$$
K <   \frac {H \left(J _ {P} D _ {\beta} + I _ {G} D _ {\phi}\right)}{J _ {P} I _ {G}} = H \left(\frac {D _ {\beta}}{I _ {G}} + \frac {D _ {\phi}}{J _ {P}}\right)\tag{4.1.9}
$$

## 4. 关于伺服回路增益 的解释

由式(4. 1.的，当稳定平台达到稳定时， $\ddot { \phi } = 0 , \dot { \phi } = 0 , \dot { \beta } = 0$ ，此时

$$
K = \frac {M _ {\mathrm{d} x}}{\beta} \quad \text {或} \quad \beta = \frac {M _ {\mathrm{d} x}}{K}
$$

式中 $K { = } k _ { u } k _ { a } k _ { m }$ 为平台伺服回路的增益。从上两式可看出 具有角刚度的概念，对于一定的平台干扰力矩 $M _ { \mathrm { d } x }$ 越大，则陀螺输出角就越小，所以从减小陀螺输出角 $\beta ,$ ，提高平台的控制精度来看， 应该尽量大;但是由式 (4. 1. 9) 知，从平台控制系统的稳定要求来看， 不能超过$H \left( \frac { D _ { \beta } } { I _ { G } } + \frac { D _ { \phi } } { J _ { P } } \right)$ ，而阻尼系数 $D _ { \beta }$ 和 $D _ { \phi }$ 都很小，要提高 的允许值，唯一的途径就是增大陀螺的角动量 ，所以动力式陀螺稳定平台设计中必须选用大角动量陀螺。

## 4. 1. 4 指示式陀螺稳定平台

如果在系统设计中选用双自由度陀螺仪作为角位置敏感器件，则该类稳定平台称为指示式陀螺稳定平台，如图 4. 1. 所示。

![](images/9fe85dfdb650ef154b959177d133420c766d5d40f70e6f94b9b41df3e38b126d.jpg)  
4. 1. 指示式陀螺稳定平台结构示意图

在此类陀螺稳定平台中，力矩马达馈人信号接人时正负方向的确定原则是:当陀螺输出角为正，即沿稳定轴正向时，产生的伺服力矩也为正，即也沿稳定轴正向 ;α 为负时，产生的伺服力矩也为负。

## 1.稳定回路工作原理(几何稳定状态下的工作原理)

设沿稳定轴 $\mathcal { X } _ { r }$ 的负方向作用有干扰力矩 $M _ { \mathrm { d } }$ '则平台做一般刚体旋转，产生偏角< $\phi ^ { ( }$ (引起偏角的旋转方向与 $\mathcal { X } _ { r }$ 同向时 $\phi$ 为正，否则为负) ，由于采用了双自由度陀螺仪，角动量 相对惯性空间稳定，所以陀螺绕外环轴具有输出 $\alpha { = } - \phi$ ，信号器将 $\alpha$ 转换成电压信号，经放大处理后馈人力矩马达，产生与 $\alpha$ 同号即沿 $x _ { i }$ 轴正向的伺服力矩，对消干扰力矩 $M _ { \mathrm { d } }$ '此过程中不产生任何陀螺力矩，卸荷完全依靠伺服力矩实现.

## 2. 修正回路工作原理(空间积分状态下的工作原理〕

若要求稳定平台沿 $\mathcal { X } _ { r }$ 轴正向以角速度 $\omega _ { \mathrm { c m d } }$ 旋转，则将 $\omega _ { \mathrm { c m d } }$ 折算成电流

$$
i _ {\mathrm{cmd}} = k _ {i \omega} \omega_ {\mathrm{cmd}}
$$

将 $i _ { \mathrm { c m d } }$ '''''馈人陀螺力矩器，电流接入时正负方向确定的原则是 $\omega _ { \mathrm { c m d } }$ 为正，即沿稳定轴正向，则产生的力矩为负，即沿陀螺内环轴的负向;若ωd为负，则产生的力矩为正，即沿陀螺内环轴的正 $\omega _ { \mathrm { c m d } }$ 向。所以在上述 $\omega _ { \mathrm { c m d } }$ 为正的'情况下，力矩 $\vec { \bf \cal J } ^ { \pm }$ 产生的指令力矩沿 $y$ 轴负向

$$
M _ {\mathrm{cmd}} = - k _ {T} k _ {i \omega} \omega_ {\mathrm{cmd}}
$$

式中 $, k _ { T }$ 为力矩器力矩系数， $k _ { i \omega }$ 为力矩器指令电流折算系数。在 $M _ { \mathrm { c m d } }$ 作用下，陀螺角动量向 $M _ { \mathrm { c m d } }$ ，产生沿外环轴正向的输出角 $\alpha \cdot$ ，外环上的信号器将 $\alpha$ 转换成电压信号，经放大处理后馈人力矩马达，力矩马达产生与 $\alpha$ 同向的伺服力矩，驱动平台绕 $x _ { r }$ 轴旋转，旋转方向同 $\mathcal { X } _ { r }$ 轴的正向。此时陀螺外框架组件绕外环轴相对惯性空间的角速度为

$$
\omega_ {i G} = \dot {\alpha} + \dot {\phi}
$$

引起的陀螺力矩为

$$
M _ {g} = H (\dot {\alpha} + \dot {\phi})
$$

方向沿内环轴正向。当作用在内环轴上的陀螺力矩与指令力矩平衡时，即

$$
M _ {g} + M _ {\mathrm{cmd}} = 0, \quad H (\dot {\alpha} + \dot {\phi}) = - M _ {\mathrm{cmd}} = k _ {T} k _ {i \omega} \omega_ {\mathrm{cmd}}
$$

时，陀螺停止进动，即 $\dot { \alpha } = 0$ ，系统达到稳态，此时

$$
H \dot {\phi} = k _ {T} k _ {i \omega} \omega_ {\mathrm{cmd}}, \quad \dot {\phi} = \frac {k _ {T} k _ {i \omega}}{H} \omega_ {\mathrm{cmd}}
$$

系统设计中，取 $\frac { k _ { T } k _ { i \omega } } { H } = 1$ ，所以

$$
\dot {\phi} = \omega_ {\mathrm{cmd}}
$$

从上述分析中可看出，在对消干扰力矩的工作过程中并不产生陀螺力矩，在跟踪指令角速度的工作过程中，陀螺力矩作用在稳定轴的正交方向，并未对稳定轴直接起作用，所以将此类陀螺稳定平台称为指示式陀螺稳定平台。

当设计陀螺稳定平台中选用液浮积分陀螺时，由于液浮的蒙古稠系数较大，阻尼系数 $D _ { \beta }$ 相对较大，所以允许刚度 可以适当大些(见式(4. 1. 9))。若要达到与动力式陀螺稳定平台相同的刚度，则陀螺角动量可以大幅度降低，所以陀螺力矩也相对较小，过渡过程中对消干扰力矩的作用并不明显，干扰力矩几乎完全由伺服力矩对消，为了便于与动力式陀螺稳定平台作区别，将此类稳定平台定义为指示动力式陀螺稳定平台。

## 4.1.5 用速率陀螺仪构成的陀螺稳定平台

4. 1. 是用速率陀螺构造的单轴陀螺稳定平台，为了便于说明工作原理，图中稳定平台仅画出了陀螺壳体部分，因陀螺壳体与平台台体固联，所以陀螺壳体的角运动实质上就是平台台体的角运动。设平台的指令角速度为零，平台(包括被稳定对象和陀螺)绕稳定轴 $x _ { O }$ 轴的转动惯量为 $\textit { J } _ { P }$ .陀螺组件绕输出轴 的转动惯量为 $I _ { G }$ 作用在稳定轴上的干扰力矩为 $M _ { \mathrm { d } x }$ 作用在陀螺输出轴上的干扰力矩为 $M _ { \mathrm { d y } }$ 陀螺角动量为 H.陀螺绕输出轴的阻尼系数和弹性系数分别为和走，陀螺信号器的传递系数为 $k _ { u }$ ，放大器的放大系数为 $k _ { a }$ ，力矩马达力矩系数为 $k _ { m }$ ，则平台旋转角 $\phi$ 满足如下方程:

$$
J _ {P} \ddot {\phi} = M _ {\mathrm{d} x} - H \dot {\beta} \cos \beta - k _ {m} k _ {a} k _ {u} \beta
$$

式中 . $H \dot { \boldsymbol { \beta } } \cos \beta$ 是由 $\boldsymbol { \beta }$ 引起的陀螺、力矩，力矩马达的伺服力矩与陀螺输出角 $\beta$ 反号。陀螺输出角 $\beta$ 满足如下方程:

$$
I _ {G} \ddot {\beta} = M _ {\mathrm{dy}} - k \beta - D \dot {\beta} + H \dot {\phi} \cos \beta
$$

式中 . $H \dot { \phi } \mathrm { c o s } \beta$ 为 $\dot { \phi }$ 引起的陀螺力矩。

![](images/fdf8850739e428196a868aecfad61a278e5c07cfe9f4cc614cbd097b692e6548.jpg)  
4. 1. 速率陀螺单轴稳定平台工作原理图

由于陀螺输出角 $\beta$ 在系统设计中控制得很小.cos如1，所以上述两式可写成

$$
J _ {P} \ddot {\phi} = M _ {\mathrm{d} x} - H \dot {\beta} - K \beta\tag{4.1.10a}
$$

$$
I _ {G} \ddot {\beta} = M _ {\mathrm{dy}} - k \beta - D \dot {\beta} + H \dot {\phi}\tag{4.1.10b}
$$

式中 . $K { = } k _ { m } k _ { a } k _ { u }$ 0

由于陀螺的带宽远高于稳定平台的带宽，所以考虑平台的响应时可不考虑陀螺的过渡过程，

而只需要考虑陀螺的稳态输出。由式(4. 1. 10b) ， ${ \ddot { \beta } } = 0 , { \dot { \beta } } = 0$ 时

$$
\beta = \frac {M _ {\mathrm{dy}}}{k} + \frac {H}{k} \dot {\phi}\tag{4.1.11}
$$

上式代入式 (4. 1. lOa) ，得

$$
J _ {P} \ddot {\phi} + \frac {K H}{k} \dot {\phi} = M _ {\mathrm{d} x} - \frac {K}{k} M _ {\mathrm{d} y}
$$

由于指令角速度为零，所以 $\dot { \phi }$ 即为平台的漂移角速度，记

$$
\varepsilon = \dot {\phi}
$$

则

$$
J _ {P} \dot {\varepsilon} + \frac {K H}{k} \varepsilon = M _ {\mathrm{d} x} - \frac {K}{k} M _ {\mathrm{d} y}
$$

对上式作拉氏变换

$$
\varepsilon (s) = \frac {M _ {\mathrm{d} x} (s) - \frac {K}{k} M _ {\mathrm{d} y} (s)}{J _ {P} s + \frac {K H}{k}}
$$

对于常值干扰力矩，平台漂移的稳态值为

$$
\varepsilon_ {s s} = \lim _ {s \rightarrow 0} s \cdot \frac {\frac {M _ {\mathrm{d} x}}{s} - \frac {K}{k s} M _ {\mathrm{d} y}}{J _ {P S} + \frac {K H}{k}} = \frac {k}{K H} M _ {\mathrm{d} x} - \frac {M _ {\mathrm{d} y}}{H}
$$

从上式可看出=绕稳定轴和绕陀螺输出轴的干扰力矩都能引起平台漂移，增大陀螺角动量能有效降低它们的影响 弹性约束是一种干扰，增大稳定回赂的增益能降低其影响。

若对上述方案作如下修改 ①采用液浮陀螺，输出轴上的干扰力矩 $M _ { \mathrm { d } y }$ 可略去不计;②力矩马达除输出与陀螺输出角成正比的伺服力矩外，还输出与输出角积分成正比的伺服力矩。则(4. 1. 11)应改写为

$$
\beta = \frac {H}{k} \dot {\phi}\tag{4.1.12}
$$

力矩马达的输出力矩为

$$
M _ {m} = - K \beta - k ^ {\prime} \int \beta \mathrm{d} t
$$

此时，式 (4. 1. 10a)应改写为

$$
J _ {P} \ddot {\phi} = M _ {\mathrm{d} x} - k ^ {\prime} \int \beta \mathrm{d} t - K \beta
$$

(4. 1. 12) 代入上式，得

$$
J _ {P} \ddot {\phi} + \frac {K H}{k} \dot {\phi} + \frac {k ^ {\prime} H}{k} \phi = M _ {\mathrm{d} x}
$$

$$
\phi (s) = \frac {M _ {\mathrm{dx}} (s)}{J _ {P} s ^ {2} + \frac {K H}{k} s + \frac {k ^ {\prime} H}{k}}
$$

对稳定轴上的常值干扰力矩

$$
M _ {\mathrm{d} x} (s) = \frac {M _ {\mathrm{d} x}}{s}
$$

$$
\phi_ {\mathrm{ss}} = \lim _ {s \rightarrow 0} s \cdot \frac {\frac {M _ {\mathrm{dx}}}{s}}{J _ {P} s ^ {2} + \frac {K H}{k} s + \frac {k ^ {\prime} H}{k}} = \frac {k M _ {\mathrm{dx}}}{k ^ {\prime} H}
$$

此时，只存在稳定偏差角，而平台的漂移角速度为零。

## 4.2 力矩装置和平台台体的传递函数

## 4.2.1 力矩装置的传递函数

在稳定平台回路中，平衡干扰力矩和按要求驱动平台旋转是依靠力矩装置实现的，力矩装置应满足如下要求:

(1)对平台施加的干扰力矩要小;

(2) 传动应平稳可靠，定位分辨力要高，滞环要小;

(3) 功耗小;

(4) 对平台附加的转动惯量要小;

(5) 体积小，重量轻，结构紧凑。

工程上常采用直接驱动式和间接驱动式两类力矩驱动装置。直接驱动式力矩装置的力矩马达驱动轴直接与稳定轴相连，力矩马达可以是交流的，也可以是直流的，但目前广泛使用直流型。间接驱动式力矩装置的力矩电机驱动轴通过齿轮减速器与稳定轴相连，常采用交流伺服电机。

## 1.直流力矩电机的传递函数

4. 2. 是直流力矩电机的等效电路图。

![](images/647f889d4181f0822f793d00a02d52c072cd0c5fe46aac4fea560363f3f6d244.jpg)  
4.2.1 直流力矩电机等效电路图

$u _ { a } :$ 控制电压 $\mathbf { \nabla } _ { \mathbf { { i } } } \Phi ;$ 激磁磁通 $: L _ { a }$ : 电枢绕组电感; $R _ { a }$ 电枢绕组电阻;

$R _ { 1 }$ 串联电阻 $\mathfrak { ; \omega } _ { m } ^ { \prime }$ 电机角速度 $: e :$ 由 $\omega _ { m } ^ { ' }$ 引起的反电势

根据回路电压定理

$$
u _ {a} - e = i _ {a} \left(R _ {a} + R _ {1}\right) + L _ {a} \frac {\mathrm{d} i _ {a}}{\mathrm{d} t}
$$

记 $R { = } R _ { a } { + } R _ { 1 }$ ，则上式的拉氏变换形式为

$$
u _ {a} (s) = e (s) + R i _ {a} (s) + L _ {a} s i _ {a} (s)
$$

$$
i _ {a} (s) = \frac {u _ {a} (s) - e (s)}{s L _ {a} + R} = \frac {1}{R} \frac {u _ {a} (s) - e (s)}{\tau_ {e} s + 1}
$$

式中 $\ , \tau _ { e } = \frac { L _ { a } } { R }$ 为电机的时间常数。

由 $i _ { a }$ .产生的电机输出扭矩为

$$
M _ {m} ^ {\prime} (s) = k _ {m} i _ {a} (s) = \frac {k _ {m}}{R} \frac {u _ {a} (s) - e (s)}{\tau_ {e} s + 1}\tag{4.2.1}
$$

式中. $k _ { m }$ 为电机的扭矩系数，即电机的电流-力矩系数。电机的传递函数为

$$
\frac {M _ {m} ^ {\prime} (s)}{u _ {a} (s) - e (s)} = \frac {k _ {m}}{R} \frac {1}{\tau_ {e} s + 1}
$$

其中反电势由电机旋转引起。设电机旋转角速度为 $\omega _ { m } ^ { ' }$ 则

$$
e (s) = k _ {e} \omega_ {m} ^ {\prime} (s)
$$

式中.k 为电机的反电势系数。所以式(4.2. 1)可写成

$$
M _ {m} ^ {\prime} (s) = \frac {k _ {m}}{R} \frac {1}{\tau_ {e} s + 1} \left[ u _ {a} (s) - k _ {e} \omega_ {m} ^ {\prime} (s) \right]\tag{4.2.2}
$$

电机的稳态输出力矩为

$$
M _ {m s s} ^ {\prime} = \lim _ {s \rightarrow 0} s \cdot M _ {m} ^ {\prime} (s) = \frac {k _ {m}}{R} \left(u _ {a} - k _ {e} \omega_ {m} ^ {\prime}\right) = k _ {u} u _ {a} - k _ {\omega} \omega_ {m} ^ {\prime}
$$

式中 .k.二 $= \frac { k _ { m } } { R }$ 为电机的电岳力矩系数 $k _ { \omega } = k _ { u } k$ , j;J电机的反电势阻尼力矩系数·

## 2. 基座旋转引起的平台干扰力矩分析

忽略电机的时间常数，由式(4.2.2)

$$
M _ {m} ^ {\prime} (s) = k _ {u} u _ {a} (s) - k _ {\omega} \omega_ {m} ^ {\prime} (s)\tag{4.2.3}
$$

假设电机输出轴与稳定轴间用减速器相连，传动比〈减速比)为'.不考虑减速器的功率损 失，则

$$
M _ {m} ^ {\prime} \omega_ {m} ^ {\prime} = M _ {m} \omega_ {m}
$$

式中. $\boldsymbol { M } _ { m } ^ { \prime }$ 和 $\omega _ { m } ^ { ' }$ 分别为电机输出扭矩和角速度. $M _ { m }$ .和 $\omega _ { m }$ 分别为平台获得的扭矩和相对基座的角速度.所以

$$
M _ {m} ^ {\prime} = \frac {M _ {m}}{i}
$$

代入式 (4.2.3).

$$
\frac {1}{i} M _ {m} (s) = k _ {u} u _ {a} (s) - k _ {\omega} i \omega_ {m} = k _ {u} u _ {a} (s) - k _ {\omega} i (\omega_ {i p} - \omega_ {i b})
$$

GP

$$
M _ {m} (s) = i k _ {u} u _ {a} (s) - i ^ {2} k _ {\omega} \omega_ {i p} (s) + i ^ {2} k _ {\omega} \omega_ {i b} (s)
$$

或

$$
M _ {m} (s) = M _ {m 1} (s) + M _ {i} (s)\tag{4.2.4}
$$

式中

$$
M _ {i} (s) = i ^ {2} k _ {\omega} \omega_ {i b} (s)\tag{4.2.5}
$$

为基座角运动引起的平台干扰力矩

$$
M _ {m 1} (s) = i k _ {u} u _ {a} (s) - i ^ {2} k _ {\omega} \omega_ {i p} (s)\tag{4.2.6}
$$

为当输入为 $u _ { a }$ ，并考虑平台绝对角速度等效反电势后的平台驱动力矩。

## 4.2.2 平台台体的传递函数

1.直接由力矩马达驱动时的传递函数

设平台台体的转动惯量为 ，则由动量矩定理

$$
J \dot {\omega} _ {i p} = M _ {\Sigma}
$$

式中， $\textstyle M _ { \Sigma } = M _ { m } = M _ { m 1 } + M _ { i }$ 。由上式，得平台台体的传递函数:

$$
\frac {\omega_ {i p} (s)}{M _ {\Sigma} (s)} = \frac {1}{J s}\tag{4.2.7}
$$

## 2. 经减速器驱动时的传递函数

1)减速器的等效转动惯量、干扰转动惯量和平台的驱动力矩

4.2.2 级减速器驱动示意图，其中 号齿轮轴与电机轴连接 号齿轮轴与平台台体轴连接，每个齿轮由固联在一起的大小齿轮组成，传动过程中由 号小齿轮拨动 号大齿轮，构成第是十 级传动。

![](images/b8e2ee4f695dfc794cb6b1e63bcb9a2adf6414581a37b989774f9bc79a989514.jpg)

![](images/2012f621b023221d0af088df73ab51514b7875679fe1b18fb9fab1117caa2521.jpg)  
4.2.2 级减速驱动

![](images/14d059fc045973aad0f75eb136d6481685d890745d24645ea6a94c48dba0979e.jpg)

为分析方便，规定:

(1)是级传动比表示为 $i _ { k - 1 / k } = \frac { \omega _ { b k } } { \omega _ { b k } } ( > 1 )$ )。式中 为基座坐标系， $\bullet \omega _ { b ( k - 1 ) }$ 为走一 号齿轮相对 坐标系的角速度， $\omega _ { b e }$ 为 号齿轮相对 坐标系的角速度。

(2) $M _ { ( k - 1 ) k }$ 表示是号齿轮受走一 号齿轮拨动而获得的扭矩 $M _ { ( k + 1 ) k } ^ { r }$ 表示是号齿轮拨动k+1号齿轮时受到 号齿轮的反作用力矩。

考察是号齿轮的动力学方程(是 $( k = 1 , 2 , \cdots , n )$ 根据动量短定理

$$
J _ {k} \dot {\omega} _ {i k} = M _ {(k - 1) k} - M _ {(k + 1) k} ^ {r}\tag{4.2.8}
$$

式中 $\boldsymbol { J } _ { k }$ 为是号齿轮的转动惯量。对 $\omega _ { i k }$ 和 $M _ { ( k + 1 ) k } ^ { r }$ 的分析如下:

$$
\omega_ {i k} = \omega_ {i b} + \omega_ {b k} = \omega_ {i b} + i _ {k / n} \omega_ {b n} = \omega_ {i b} + i _ {k / n} \left(\omega_ {i n} - \omega_ {i b}\right) = \omega_ {i b} \left(1 - i _ {k / n}\right) + \omega_ {i n} i _ {k / n}\tag{4.2.9}
$$

式中

$$
i _ {k / n} = \frac {\omega_ {b k}}{\omega_ {b n}} = \frac {\omega_ {b k}}{\omega_ {b (k + 1)}} \cdot \frac {\omega_ {b (k + 1)}}{\omega_ {b (k + 2)}} \dots \frac {\omega_ {b (n - 1)}}{\omega_ {b n}} = i _ {k / (k + 1)} \cdot i _ {(k + 1) / (k + 2)} \dots i _ {(n - 1) / n}\tag{4.2.10}
$$

4. 2. 表示 级传动中走号齿轮和k+1 号齿轮间的作用力关系，其中 $P _ { k ( k + 1 ) } \ : , M _ { k ( k + 1 ) }$ 分别表示 号齿轮对是十 号齿轮的驱动力和驱动力矩 $P _ { ( k + 1 ) k } ^ { r }$ 和 $M _ { ( k + 1 ) k } ^ { r }$ 分别表示是 +1 号齿轮号齿轮的反作用力和反作用力矩，显然 $P _ { k ( k + 1 ) }$ )与 $P _ { ( k + 1 ) k } ^ { r }$ 大小相等，方向相反，且

$$
\begin{array}{r l} M _ {k (k + 1)} = - & \mid P _ {k (k + 1)} \mid R _ {k + 1} \\ M _ {(k + 1) k} ^ {r} = - & \mid P _ {(k + 1) k} ^ {r} \mid r _ {k} \end{array}
$$

式中，负号表示力短方向穿人纸面。所以有

$$
\frac {M _ {(k + 1) k} ^ {r}}{M _ {k (k + 1)}} = \frac {r _ {k}}{R _ {k + 1}}
$$

因两齿轮在点 的速度相同，即 $r _ { k } \omega _ { b k } = R _ { k + 1 } \omega _ { b ( k + 1 ) }$ 川所以

$$
\begin{array}{l} \frac {R _ {k + 1}}{r _ {k}} = \frac {\omega_ {b k}}{\omega_ {b (k + 1)}} = i _ {k / k + 1} \\ M _ {(k + 1) k} ^ {r} = \frac {1}{i _ {k / k + 1}} M _ {k (k + 1)} \end{array}\tag{4.2.11}
$$

将式 (4.2.9) 和式 (4.2. 11) 代入式 (4.2. 的，得

$$
\begin{array}{c} \frac {1}{i _ {k / k + 1}} M _ {k (k + 1)} = M _ {(k - 1) k} - J _ {k} \left[ \dot {\omega} _ {i b} \left(1 - i _ {k / n}\right) + \dot {\omega} _ {i n} i _ {k / n} \right] \\ M _ {k (k + 1)} = i _ {k / k + 1} M _ {(k - 1) k} + i _ {k / k + 1} \left(i _ {k / n} - 1\right) J _ {k} \dot {\omega} _ {i b} - i _ {k / k + 1} i _ {k / n} J _ {k} \dot {\omega} _ {i n} \\ k = 1, 2, \dots , n \end{array}\tag{4.2.12}
$$

上式即为齿轮逐级传递力矩的递推公式。下面确定递推初值 $M _ { 0 1 }$ 。考虑第一级传动，如图4.2.4 所示。

![](images/b9b1d32812479f4c005b110e04ef0cfd2167e53a0c01b8404a813507f12dca1b.jpg)  
是号齿轮和是 +1 号齿轮间的作用力4.2.3

设电机的驱动力矩(输出力矩)为 $M _ { m } ^ { \prime } , 0$ 号齿轮的转动惯量为 $J _ { \mathrm { 0 } }$ '

$$
J _ {0} \dot {\omega} _ {i o} = M _ {m} ^ {\prime} - M _ {1 0} ^ {r}\tag{4.2.13}
$$

由图 4.2.41

$$
\begin{array}{l} M _ {0 1} = - \mid P _ {0 1} \mid R _ {1}, M _ {1 0} = - \mid P _ {1 0} ^ {r} \mid r _ {0} \\ \frac {M _ {1 0} ^ {r}}{M _ {0 1}} = \frac {r _ {0}}{R _ {1}} = \frac {1}{i _ {0 / 1}} \\ M _ {1 0} ^ {r} = \frac {1}{i _ {0 / 1}} M _ {0 1} \end{array}
$$

上式代入式 (4.2.13) ，得

$$
\frac {1}{i _ {0 / 1}} M _ {0 1} = M _ {m} ^ {\prime} - J _ {0} \dot {\omega} _ {i 0}
$$

即

$$
\begin{array}{r l} M _ {0 1} & = i _ {0 / 1} M _ {m} ^ {\prime} - i _ {0 / 1} J _ {0} (\dot {\omega} _ {i b} + \dot {\omega} _ {b 0}) = i _ {0 / 1} M _ {m} ^ {\prime} - i _ {0 / 1} J _ {0} (\dot {\omega} _ {i b} + i _ {0 / n} \dot {\omega} _ {b n}) \\ & = i _ {0 / 1} / M _ {m} ^ {\prime} - i _ {0 / 1} J _ {0} [ \dot {\omega} _ {i b} + i _ {0 / n} (\dot {\omega} _ {i n} - \dot {\omega} _ {i b}) ] \\ & = i _ {0 / 1} M _ {m} ^ {\prime} + i _ {0 / 1} (i _ {0 / n} - 1) J _ {0} \dot {\omega} _ {i b} - i _ {0 / 1} i _ {0 / n} J _ {0} \dot {\omega} _ {i n} \end{array}\tag{4.2.14}
$$

![](images/8ef64bb0aed6f31c7254567ea31135aa40ccaf0785acbbd6d09f818cae9e5874.jpg)  
4.2.4 减速器第一级传动

根据式 (4.2.12) 和式 (4.2.14)

对于 $k = 1$

$$
\begin{array}{r l} M _ {1 2} & = i _ {1 / 2} M _ {0 1} + i _ {1 / 2} (i _ {1 / n} - 1) J _ {1} \dot {\omega} _ {i b} - i _ {1 / 2} i _ {1 / n} J _ {1} \dot {\omega} _ {i n} \\ & = i _ {1 / 2} i _ {0 / 1} M _ {m} ^ {\prime} + [ i _ {1 / 2} i _ {0 / 1} (i _ {0 / n} - 1) J _ {0} + i _ {1 / 2} (i _ {1 / n} - 1) J _ {1} ] \dot {\omega} _ {i b} \\ & \quad - (i _ {1 / 2} i _ {0 / 1} i _ {0 / n} J _ {0} + i _ {1 / 2} i _ {1 / n} J _ {1}) \dot {\omega} _ {i n} \end{array}
$$

对于走 =2

$$
\begin{array}{l} M _ {2 3} = i _ {2 / 3} M _ {1 2} + i _ {2 / 3} (i _ {2 / n} - 1) J _ {2} \dot {\omega} _ {i b} - i _ {2 / 3} i _ {2 / n} J _ {2} \dot {\omega} _ {i n} \\ = i _ {2 / 3} i _ {1 / 2} i _ {0 / 1} M _ {m} ^ {\prime} + [ i _ {2 / 3} i _ {1 / 2} i _ {0 / 1} (i _ {0 / n} - 1) J _ {0} + i _ {2 / 3} i _ {1 / 2} (i _ {1 / n} - 1) J _ {1} \\ \quad + i _ {2 / 3} (i _ {2 / n} - 1) J _ {2} ] \dot {\omega} _ {i b} - (i _ {2 / 3} i _ {1 / 2} i _ {0 / 1} i _ {0 / n} J _ {0} + i _ {2 / 3} i _ {1 / 2} i _ {1 / n} J _ {1} + i _ {2 / 3} i _ {2 / n} J _ {2}) \dot {\omega} _ {i n} \\ \dots \dots \end{array}
$$

根据以上推导，可归纳得，对于 $k { = } n { - } 1$

$$
\begin{array}{r l} M _ {(n - 1) n} & = i _ {(n - 1) / n} i _ {(n - 2) / (n - 1)} \dots i _ {0 / 1} M _ {m} ^ {\prime} + \{i _ {(n - 1) / n} i _ {(n - 2) / (n - 1)} \dots i _ {0 / 1} (i _ {0 / n} - 1) J _ {0} \\ & \quad + i _ {(n - 1) / n} i _ {(n - 2) / (n - 1)} \dots i _ {1 / 2} (i _ {1 / n} - 1) J _ {1} + \dots + i _ {(n - 1) / n} [ i _ {(n - 1) / n} - 1 ] J _ {n - 1} \} \dot {\omega} _ {i b} \\ & \quad - [ i _ {(n - 1) / n} i _ {(n - 2) / (n - 1)} \dots i _ {0 / 1} i _ {0 / n} J _ {0} + i _ {(n - 1) / n} i _ {(n - 2) / (n - 1)} \dots i _ {1 / 2} i _ {1 / n} J _ {1} + \dots + i _ {(n - 1) / n} i _ {(n - 1) / n} J _ {n - 1} ] \dot {\omega} _ {i m} \\ & \text {根据式(4.2.10):上式可写成,} \end{array}
$$

根据式(4.2.10) ，上式可写成:

$$
\begin{array}{r l} M _ {(n - 1) n} & = i _ {0 / n} M _ {m} ^ {\prime} + \left\{i _ {0 / n} (i _ {0 / n} - 1) J _ {0} + i _ {1 / n} (i _ {1 / n} - 1) J _ {1} + \dots \right. \\ & \quad \left. + i _ {(n - 1) / n} [ i _ {(n - 1) / n} - 1 ] J _ {n - 1} \right\} \dot {\omega} _ {i b} - \left[ i _ {0 / n} ^ {2} J _ {0} + i _ {1 / n} ^ {2} J _ {1} + \dots + i _ {(n - 1) / n} ^ {2} J _ {n - 1} \right] \dot {\omega} _ {i n} \\ & = i _ {0 / n} M _ {m} ^ {\prime} + J _ {i} \dot {\omega} _ {i b} - J _ {e q} \dot {\omega} _ {i n} \end{array}\tag{4.2.15}
$$

此即为平台获得的驱动力矩，其中

$$
J _ {i} = i _ {0 / n} \left(i _ {0 / n} - 1\right) J _ {0} + i _ {1 / n} \left(i _ {1 / n} - 1\right) J _ {1} + \dots + i _ {(n - 1) / n} \left[ i _ {(n - 1) / n} - 1 \right] J _ {n - 1} = \sum_ {k = 0} ^ {n - 1} i _ {k / n} \left(i _ {k / n} \right.\tag{1) \(J_{k}\}
$$

(4.2.16)

为减速器的干扰转动惯量。

$$
J _ {e q} = i _ {0 / n} ^ {2} J _ {0} + i _ {1 / n} ^ {2} J _ {1} + \dots + i _ {(n - 1) / n} ^ {2} J _ {n - 1} = \sum_ {k = 0} ^ {n - 1} i _ {k / n} ^ {2} J _ {k}\tag{4.2.17}
$$

为减速器的等效转动惯量。

## 2) 平台台体的传递函数

设平台台体的转动惯量为 ，作用在稳定轴上的干扰力矩为 $M _ { \mathrm { d } }$ '

$$
J \dot {\omega} _ {i p} = M _ {(n - 1) n} + M _ {\mathrm{d}} = i _ {0 / n} M _ {m} ^ {\prime} + J _ {i} \dot {\omega} _ {i b} - J _ {e q} \dot {\omega} _ {i n} + M _ {\mathrm{d}}
$$

由于 $\omega _ { i n } = \omega _ { i p }$ ，所以

$$
(J + J _ {e q}) \dot {\omega} _ {i p} = i _ {0 / n} M _ {m} ^ {\prime} + J _ {i} \dot {\omega} _ {i b} + M _ {\mathrm{d}}
$$

记 $J _ { \Sigma } { = } J { + } J _ { e q }$ ，则由上式得

$$
J _ {\Sigma} s \omega_ {i p} (s) = i _ {0 / n} M _ {m} ^ {\prime} (s) + J _ {i} s \omega_ {i b} (s) + M _ {\mathrm{d}} (s)\tag{4.2.18}
$$

根据式 (4.2.1 肘，可画出平台台体的方块图如图 4.2.5 所示，其中 $\phi ^ { ( \boldsymbol { s } ) }$ 为平台的旋转角。

![](images/0c22f52eb6f313d30451dd62d84876e8d80db8032c78bce9a4608ff7c7513748.jpg)  
4.2.5 平台台体方块图

## 4.3 单轴陀螺稳定平台

## 4.3.1 单轴稳定平台的方块图及传递函数

## 1. 积分陀螺单轴稳定平台的传递函数

此处为简单起见，以直接驱动式单轴平台为研究对象。根据图 2. 3. 所示积分陀螺的方块图、式(4.2.2)和式(4.2.7) ，可得积分陀螺单轴平台的闭环方块图如图 4.3.1(a)所示，图中考虑了陀螺输出角变化率 $\dot { \alpha }$ 引起的陀螺力矩对平台稳定轴的影响，环节- $\cdot k _ { u } k _ { a } F ( s )$ 中的负号是由于力矩马达产生的伺服力矩与陀螺输出反号， $k _ { u }$ 为陀螺信号器的传递系数， $k _ { a }$ 为电子网络放大系数， $F ( s )$ 为校正网络的传递函数。

在工程上为使问题得到简化，常突出一些主要影响因素，而忽略一些次要影响因素，此处对4. 3. l( a) 作如下简化:

(1)因 $\dot { \omega } _ { 0 }$ 中。反映了沿陀螺输出轴方向的角加速度对平台稳定轴的影响，而平台稳定轴与陀螺输入轴方向相同，与陀螺输出轴方向正交，所以 $\dot { \omega } _ { 0 }$ 。实质上反映了三轴平台正交轴之间的交叉耦合影响，而交叉耦合是较弱的，此处不予考虑。

![](images/e9a09a74862c1848927f27e8bfc45265dd48672633ed0383b0d37df7eaf7417b.jpg)

(a)  
![](images/e0c799d143d77efc55b7ace838f97bd7cdb635c2b9b19085f68555f06d53f635.jpg)  
(b)

![](images/d4364fa1c9714fc19f80796f973efa089b67616c6c40e28e04f8a722ee620678.jpg)  
4.3.1 积分陀螺单轴稳定平台方块图

(2) 所用陀螺的角动量 一般都较小(如液浮陀螺) ，所以作用在稳定轴上的陀螺力矩 $H { \dot { \alpha } }$ 可略去不计。

(3) 虽然 $e { = } k _ { e } \omega$ 在系统中具有负反馈校正作用，但比校正网络所起的校正作用弱得多，在系统分析中可略去不计。

(4) 电机的时间常数 $\tau _ { e }$ 是毫秒级的，电机的带宽远高于平台的带宽，所以电机的过渡过程可以不考虑。

基于上述简化，图 4.3.1(a) 可简化成图 4.3.1(b) 。记

$$
Y ^ {\prime} (s) = \frac {k _ {g} k _ {u} k _ {a} k _ {m} F (s)}{R (\tau_ {g} s + 1)}\tag{4.3.1}
$$

则图 4.3. l(b)又可画成图 4. 3. l(c)所示更简洁的形式。由图得稳定平台的开环传递函数为

$$
Y _ {0} (s) = \frac {Y ^ {\prime} (s)}{J _ {\Sigma} s ^ {2}}\tag{4.3.2}
$$

## 2. 双自由度角位置陀螺单轴稳定平台的传递函数

与前面讨论方法相仿，可得双自由度角位置陀螺单轴稳定平台方块图如图 4.3.2(a)所示。略去电机的反电势影响及过渡过程，可简化成图 4. 3. 2(b) 。记

$$
Y ^ {\prime} (s) = \frac {k _ {u} k _ {a} k _ {m} F (s)}{R}\tag{4.3.3}
$$

则可得图 4.3.2(c)所示更简洁的形式。由图得稳定平台的开环传递函数为

$$
Y _ {0} (s) = \frac {Y ^ {\prime} (s)}{J _ {\Sigma} s ^ {2}}\tag{4.3.4}
$$

(a)  
(b)  
![](images/20b39e74f5668c2b8414bd46191875641285a101378c36318f06a566bea9f3a8.jpg)

(a)  
![](images/32adc36a3fc53fbc52ba69fd1a64b3a8b8cf006eb3d7e06f5e585567bbe23b94.jpg)

![](images/e66886979b5d937d72d725536905080403deaff8a71a02c49b8486c70bf74da9.jpg)  
4.3.2 双自由度角位置陀螺单轴稳定平台方块图

## 3. 挠性陀螺单轴稳定平台传递函数

根据式(2.4. 30)所示挠性陀螺的传递函数，可画出挠性陀螺单轴稳定平台简化方块图如图4. 3. 所示，由图 4.3.3(b)可得稳定平台的开环传递函数

$$
Y _ {0} (s) = \frac {Y ^ {\prime} (s)}{J _ {\Sigma} s}\tag{4.3.5}
$$

式中

$$
Y ^ {\prime} (s) = \frac {\tau_ {g} k _ {u} k _ {a} k _ {m} F (s)}{R (\tau_ {g} s + 1)}\tag{4.3.6}
$$

![](images/7d8cd345e2737255f61d8ed141c9c229fb64a36191612a5f20adf34110eb1142.jpg)

![](images/e5fe2f98c4a3e884e788bea0c63ef256cd7ec7e5d64acf6bb795de7d4112df10.jpg)  
4.3.3 挠性陀螺单轴稳定平台方块图

从式(4.3.5)可以看出，挠性陀螺单轴稳定平台比前两种稳定平台的开环传递函数少一个积分环节，要达到与前两种稳定平台相同的无静差度，在校正环节中需增加一个积分环节，这将引起 $9 0 ^ { \circ } ;$ 相位滞后，系统的设计难度比前两种平台大。如果挠性陀螺的时间常数 $\tau _ { g }$ 很大，则传递函数近似为 $\frac { \beta ( s ) } { \varPhi ( s ) } { \approx - 1 }$ ，此时，挠性陀螺可近似看作一个角位置陀螺。

## 4.3.2 单轴稳定平台的设计指标

平台系统设计的目的是为了保证平台的动静态性能，稳定平台应按要求能相对惯性空间稳定或能跟踪预定方位。在设计过程中，通常对平台系统提出如下的性能指标。

## 1. 闭环力矩刚度

4. 1. 节中讨论了速率陀螺单轴稳定平台在干扰力矩作用下平台的偏差角，稳定回路虽然能对干扰力矩起卸载作用，但仍会引起偏差角，干扰力矩与偏差角的比值是力矩刚度的概念，平台设计中应尽量提高力矩刚度，而力矩刚度是由平台系统的回路参数确定的。

对于积分陀螺单轴稳定平台和双自由度角位置陀螺单轴稳定平台，由图 4.3.1(c)4. 3. 2(c)

$$
\frac {\omega (s)}{M _ {\mathrm{d}} (s)} = \frac {s}{J _ {\Sigma} s ^ {2} + Y ^ {\prime} (s)}
$$

而 $\mathbf { \chi } _ { \omega } ( \mathbf { \chi } _ { s } ) = s \Phi ( s )$ 其中 $\varPhi$ 为 $M _ { \mathrm { d } }$ 作用下的平台偏角。所以上式可写成

$$
\frac {\Phi (s)}{M _ {\mathrm{d}} (s)} = \frac {1}{J _ {\Sigma} s ^ {2} + Y ^ {\prime} (s)}
$$

写成力矩刚度的形式

$$
S _ {\Phi} (s) = \frac {M _ {\mathrm{d}} (s)}{\Phi (s)} = J _ {\Sigma} s ^ {2} + Y ^ {\prime} (s)
$$

力矩刚度的频率响应为

$$
S _ {\Phi} (\mathrm{j} \omega) = - J _ {\Sigma} \omega^ {2} + Y ^ {\prime} (\mathrm{j} \omega)\tag{4.3.7}
$$

对于积分陀螺单轴稳定平台 $. Y ^ { \prime } ( s )$ 按式 (4.3.1) 确定，此时

$$
S _ {\Phi} (\mathrm{j} \omega) = - J _ {\Sigma} \omega^ {2} + \frac {k _ {g} k _ {u} k _ {a} k _ {m} F (\mathrm{j} \omega)}{R (\mathrm{j} \omega \tau_ {g} + 1)}
$$

由于陀螺频带远比稳定平台的频带宽，所以可不考虑陀螺的过渡过程，上式可写成

$$
S _ {\Phi} (\mathrm{j} \omega) = - J _ {\Sigma} \omega^ {2} + \frac {k _ {g} k _ {u} k _ {a} k _ {m}}{R} F (\mathrm{j} \omega)\tag{4.3.8}
$$

$S _ { \phi } ( \mathrm { j } \omega )$ 表示在频率为 $\omega$ 的简谐交变干扰力矩作用下平台的力矩刚度，它反映了平台抵抗简谐交变干扰力矩的能力，所以称之为动态力矩刚度。当 $\omega { = } 0$ 时

$$
S _ {\Phi} (0) = \frac {k _ {g} k _ {u} k _ {a} k _ {m}}{R} F (0)\tag{4.3.9}
$$

$S _ { \phi } ( 0 )$ 表示在常值干扰力矩(不变力矩)作用下平台的力矩刚度，它反映了平台抵抗常值干扰力矩的能力，所以称之为静态力矩刚度.

根据式 (4.3.9) ，式 (4.3.8) 可写成

$$
S _ {\Phi} (\mathrm{j} \omega) = S _ {\Phi \mathrm{N}} (\mathrm{j} \omega) + S _ {\Phi \mathrm{S}} (\mathrm{j} \omega)\tag{4.3.10}
$$

式中

$$
S _ {\Phi \mathrm{N}} (\mathrm{j} \omega) = - \omega^ {2} J _ {\Sigma}\tag{4.3.11}
$$

$$
S _ {\Phi S} (\mathrm{j} \omega) = \frac {S _ {\Phi} (0)}{F (0)} F (\mathrm{j} \omega)\tag{4.3.12a}
$$

或

$$
S _ {\Phi S} (\mathrm{j} \omega) = \frac {k _ {g} k _ {u} k _ {a} k _ {m}}{R} F (\mathrm{j} \omega)\tag{4.3.12b}
$$

$S _ { \varPhi N } \left( \mathrm { j } \omega \right)$ 称为惯性力矩刚度，是负刚度，对于平台力矩刚度起削弱作用。 $S _ { \Phi S } \left( \mathrm { j } \omega \right)$ 〉称为伺服力矩刚度，伺服回路增益越大，力矩刚度就越大，但应注意增益的上限还受平台系统稳定性要求的限制，是不能随意增大的。

如果采用挠性陀螺作为惯性器件，且在校正环节中未加入积分环节，则由式 (4.3.5)(4.3.6) 和图 4.3.3(b) ，得

$$
\begin{array}{l} \frac {\omega (s)}{M _ {\mathrm{d}} (s)} = \frac {s \Phi (s)}{M _ {\mathrm{d}} (s)} = \frac {1}{J _ {\Sigma} s + Y ^ {\prime} (s)} \\ S _ {\Phi} (s) = \frac {M _ {\mathrm{d}} (s)}{\Phi (s)} = J _ {\Sigma} s ^ {2} + s Y ^ {\prime} (s) \end{array}
$$

平台力矩刚度为

$$
S _ {\Phi} (\mathrm{j} \omega) = - J _ {\Sigma} \omega^ {2} + \mathrm{j} \omega Y ^ {\prime} (\mathrm{j} \omega) = - J _ {\Sigma} \omega^ {2} + \mathrm{j} \omega \frac {\tau_ {g}}{\mathrm{j} \omega \tau_ {g} + 1} \cdot \frac {k _ {u} k _ {a} k _ {m}}{R} F (\mathrm{j} \omega)
$$

静态力矩刚度为

$$
S _ {\phi} (0) = 0
$$

即平台不具备抵抗常值干扰力矩的能力，所以校正网络中必须包含积分环节。

因为 $S _ { \phi } ( \mathrm { j } \omega )$ 直接影响平台在干扰力矩作用下的平台偏差角 $\Phi ,$ ，所以工程上希望通过测量 $\varPhi$ 角来判断稳定平台的力矩刚度是否足够。但 $\varPhi$ 角是不可观测的，而陀螺输出角 $\alpha$ 可通过信号器输出观测。所以工程上希望通过观测 $\alpha$ 角来判断力矩刚度是杏足够。这就提出了陀螺输出角力矩刚度的概念 $S _ { \alpha } ( \mathrm { j } \omega )$ ，其中

$$
S _ {\alpha} (s) = \frac {M _ {\mathrm{d}} (s)}{\alpha (s)}\tag{4.3.13}
$$

并用 $S _ { \alpha } ( \mathrm { j } \omega )$ 来间接说明 $S _ { \phi } ( \mathrm { j } \omega )$ 。在平台系统的分析设计中， $S _ { \alpha } \left( \mathrm { j } \omega \right)$ 和 $S _ { \phi } ( \mathrm { j } \omega )$ 具有同等的重要性。

## 2. 闭环振荡度

陀螺单轴稳定平台是单位反馈系统，如图 4.3.1- 4.3.3 所示，所以闭环频率特性为

$$
H (\mathrm{j} \omega) = \frac {Y _ {0} (\mathrm{j} \omega)}{1 + Y _ {0} (\mathrm{j} \omega)}
$$

式中 $Y _ { 0 } ( \mathrm { j } \omega )$ 为系统的开环频率特性。若 $\omega _ { M }$ 为谐振频率，则

$$
M = \frac {\mid H (\mathrm{j} \omega_ {M}) \mid}{\mid H (0) \mid}\tag{4.3.14}
$$

称为平台系统的振荡度。

振荡度 是平台系统频域内的设计指标，它与时域设计指标超调量相对应。如果系统在频域里的振荡度 大，则说明系统在单位阶跃输入下的超调量也大。振荡度 大和超调量大都意味着系统的阻尼系数小。 越大，力矩马达越有可能出现饱和，使电机功耗增大，甚至造成系统的非线性振荡，使系统不稳定而使平台翻倒。此外，力矩谐振还会使陀螺动态误差角增大，使输出角超出线性区域或出现输出饱和，这也会使系统产生非线性振荡，破坏系统的正常工作，或使信号器碰上止档而损坏陀螺仪。为了保证系统的工作品质并合理利用电机功率，通常选M= 1. 1\~1. 50

对闭环振荡度 的要求可转化为对开环相位裕量的要求。这是因为在复平面内.M 圆同$Y ( \mathrm { j } \omega )$ 的幅相特性曲线的切点与原点间的连线相对一 线间的夹角近似于系统开环相位裕量的缘故(郭富强等.1995;胡寿松.1984)0 间的对应关系如表 4.3.1 所示.平台系统与一般控制系统一样，为了系统工作的稳定性，除提出相位裕量的要求外，还必须同时提出增益裕量的要求。

4.3.1 相位裕度与振荡度的对应关系

<table><tr><td>M</td><td>1.1</td><td>1.2</td><td>1.3</td><td>1.4</td><td>1.5</td></tr><tr><td>λ</td><td>54°10&#x27;</td><td>49°15&#x27;</td><td>45°10&#x27;</td><td>41°50&#x27;</td><td>38°55&#x27;</td></tr></table>

## 3. 闭环通频带与开环交截频率

对于单位反馈系统，定义满足 $| H ( \mathrm { j } \omega ) | \geqslant 0 . 7 0 7 | H ( 0 )$ 的最高频率 $\omega _ { B }$ 为系统的闭环通频带，其中 $H ( \mathrm { j } \omega )$ 为系统的闭环频率响应 $\mathbf { ; } \omega _ { B }$ 的高低，则反映了系统对输入响应的快慢.频带宽虽然使系统快速性提高，但也会使系统易受干扰的影响.对平台系统，频带选择应注意以下几点

(1)作用在平台稳定轴上的干扰力矩一部分是由运载体姿态变化和弹性变形等因素引起的，干扰频率一般低于 5Hz.通频带应比 5Hz 大好几倍，才能抑制干扰。

(2) 电子系统的噪声也会形成干扰，频率约在 50\~5000Hz.平台系统应该滤除这些干扰，所以平台的通频带不宜太宽.

(3) 对工作在空间积分状态下的平台，指令信号由计算机计算后经由陀螺力矩器控制平台，所以平台系统的通频带应考虑计算机输出的最高频率。

综合以上各种因素及根据实践经验，平台系统的通频带一般在 50\~200rad/s 范围内选取，这样选择通频带后，平台系统的调节时间在 O.ls 以内。

通频带是系统在频域里的闭环指标，与开环交截频率有密切关系。系统的谐振频率 $\omega _ {  { \boldsymbol { M } } }$ 与开环交截频率 $\omega _ { C }$ 接近，ωB 越大，ωM 也越大，所以如果要求 ωB 大， $, \omega B$ $, \omega M$ $\omega _ { B }$ $\omega _ { C }$ 应大。因此在系统设计中，对闭环通频带的要求可以转换为对开环交截颁率 $\omega _ { C }$ 的要求。

## 4.3.3 平台系统的校正方法

## 1.未校正时平台系统存在的问题

由式(4.3.2)及式(4.3.4)知，对于采用积分陀螺及双自由度位置陀螺为惯性器件的单轴稳定平台，以及采用挠性陀螺的单轴稳定平台(在校正网络设计中应增加一个积分环节) .这些平台系统的开环传递函数中都包含 个积分环节，在任何频率上相位滞后至少为 $1 8 0 ^ { \circ }$ 系统本身是不稳定的，必须校正，使平台系统不但稳定，而且还具有要求的相位裕量，从而提高系统的动态品质。

校正是指在回路中串入一个线性环节，使系统的开环幅频特性在交截频率(穿越零分贝线的对应频率〉处的相位裕度量达到要求的值。

## 2. 系统的超前校正

## 1)超前校正网络的传递函数和频率特性

超前校正网络的传递函数为

$$
G _ {1} (s) = \frac {T _ {2} s + 1}{T _ {3} s + 1}\tag{4.3.15}
$$

式中 ${ \cal T } _ { 2 } = h { \cal T } _ { 3 } , h > 1$ 。幅频特性和相频特性如图 4. 3. 所示，图中 $\omega$ 为圆频率，而不是角频率。最大超前相位为

$$
\varphi_ {m} = \arcsin \frac {h - 1}{h + 1}\tag{4.3.16}
$$

最大超前相位的发生频率为

$$
\omega_ {m} = \frac {1}{\sqrt {T _ {2} T _ {3}}} = \frac {1}{\sqrt {h} T _ {3}}\tag{4.3.17}
$$

4. 3. 列出了最大超前相位 $\varphi _ { m }$ 与 的关系。

![](images/abd57d1cfc1c3cf3e5765df4c92bc8eee6f38a0aba60fca24b8eabdc25370ef6.jpg)

![](images/4024e7f9f19148b1f3332b7fa296f1a09e7f752581648039460b86a287593aed.jpg)  
4.3.4 超前校正网络的幅频特性和相频特性

4.3.2 最大超前相位 $\pmb { \varphi } _ { m }$ 与 的关系

<table><tr><td>h</td><td>1</td><td>2</td><td>3</td><td>4</td><td>5</td><td>6</td><td>7</td><td>8</td></tr><tr><td> $\varphi_m/(^\circ)$ </td><td>0</td><td>19.4</td><td>30</td><td>36.9</td><td>41.8</td><td>45.6</td><td>48.6</td><td>51</td></tr><tr><td>h</td><td>9</td><td>10</td><td>11</td><td>12</td><td>13</td><td>14</td><td>15</td><td></td></tr><tr><td> $\varphi_m/(^\circ)$ </td><td>53.1</td><td>54.9</td><td>56.4</td><td>57.8</td><td>59</td><td>60.1</td><td>61</td><td></td></tr></table>

## 2) 最大超前相位 $\varphi _ { m }$ 的选取

应该在开环幅频特性的交截频率 $\omega _ { C }$ 处获得最大的相位校正量，即应该使

$$
\omega_ {m} = \omega_ {C}\tag{4.3.18}
$$

设校正前开环相频特性在 $\omega _ { C }$ 处的相位为 $\varphi _ { \mathrm { 0 } } \left( \omega _ { C } \right)$ 要求系统达到的相位裕量为 ，则校正后的相位应为

$$
\varphi (\omega_ {C}) = - 1 8 0 ^ {\circ} + \lambda\tag{4.3.19}
$$

所以校正网络的最大相位超前量应为

$$
\varphi_ {m} \geqslant \varphi (\omega_ {C}) - \varphi_ {0} (\omega_ {C}) = \lambda - 1 8 0 ^ {\circ} - \varphi_ {0} (\omega_ {C})\tag{4.3.20}
$$

3) $T _ { 2 }$ ，和 $T _ { 3 }$ 的确定

根据按式 (4.3.20)选定的 $\varphi _ { m }$ ，查表 4.3.2 可得 $h$ . 由开环幅频特性确定出 $\omega _ { C }$ 后，取 $\omega _ { m } =$ $\omega _ { C } ,$ ，曲式 (4.3.17) 确定出

$$
T _ {3} = \frac {1}{\omega_ {m} \sqrt {h}}
$$

则

$$
T _ {2} = h T _ {3}
$$

$T _ { 2 }$ 和 $T _ { 3 }$ ，确定后，就可按式(4.3.15)确定出校正网络的传递函数。

## 。开环增益调整

串人超前校正网络后，平台系统的开环频率特性成

$$
Y _ {1} ^ {\prime} (\mathrm{j} \omega) = G _ {1} (\mathrm{j} \omega) Y _ {0} (\mathrm{j} \omega)
$$

由于 h>l 所以

$$
\mid G _ {1} (\mathrm{j} \omega) \mid = \sqrt {\frac {1 + (h T _ {3} \omega) ^ {2}}{1 + (T _ {3} \omega) ^ {2}}} > 1
$$

$Y _ { 1 } ^ { \prime } ( \mathrm { j } \omega )$ 幅频特性的交截频率移向高频处，为了使交截频率 $\omega _ { C }$ 仍为设计要求值，必须对Y $Y _ { 1 } ^ { \prime } ( \mathrm { j } \omega )$ 适当调整放大倍数。设经增益调整后的开环传递函数为

$$
Y _ {1} (s) = k _ {c} G _ {1} (s) Y _ {0} (s)
$$

若平台采用积分陀螺作为惯性器件，则

$$
Y _ {1} (s) = \frac {T _ {2} s + 1}{T _ {3} s + 1} \frac {k _ {g} k _ {u} k _ {a} ^ {\prime} k _ {m}}{R J _ {\Sigma} s ^ {2} (\tau_ {g} s + 1)}
$$

式中， $, k _ { a } ^ { \prime } = k _ { c } k _ { c }$ 是经调整以后电子线路的放大倍数。记

$$
K _ {0} = \frac {k _ {g} k _ {u} k _ {a} ^ {\prime} k _ {m}}{R J _ {\Sigma}}\tag{4.3.21}
$$

则

$$
Y _ {1} (s) = \frac {K _ {0} \left(T _ {2} s + 1\right)}{s ^ {2} \left(\tau_ {g} s + 1\right) \left(T _ {3} s + 1\right)}\tag{4.3.22}
$$

经调整后开环幅频特性的交截颜率仍然应该为 $\omega _ { C }$ ，即

$$
2 0 \lg | Y _ {1} (\mathrm{j} \omega) | _ {\omega = \omega_ {C}} = 0 \mathrm{dB}
$$

根据式(4.3.22) ，上式可写成

$$
\begin{array}{l} 2 0 \lg K _ {0} + 1 0 \lg [ (T _ {2} \omega_ {C}) ^ {2} + 1 ] - 4 0 \lg \omega_ {C} \\ - 1 0 \lg [ (\tau_ {g} \omega_ {C}) ^ {2} + 1 ] - 1 0 \lg [ (T _ {3} \omega_ {C}) ^ {2} + 1 ] = 0 \end{array}\tag{4.3.23}
$$

从上式可解出 $K _ { 0 }$ ，再根据式(4.3.21) ，可求出电子线路放大倍数的调整值

$$
k _ {a} ^ {\prime} = \frac {R J _ {\Sigma} K _ {0}}{k _ {g} k _ {u} k _ {m}}\tag{4.3.24}
$$

## 5) 静态力矩刚度校核

根据式(4.3.9) ，平台系统的静态力矩刚度为

$$
S _ {\Phi} (0) = \frac {k _ {g} k _ {u} k _ {a} ^ {\prime} k _ {m}}{R} F (0)
$$

对于超前校正网络， $F ( s ) = G _ { 1 } ( s ) = \frac { T _ { 2 } s + 1 } { T _ { 3 } s + 1 } , F ( 0 ) \colon$ =1 ，所以

$$
S _ {\Phi} (0) = \frac {k _ {g} k _ {u} k _ {a} ^ {\prime} k _ {m}}{R}
$$

根据式(4.3.2 1) ，上式又可写成

$$
S _ {\Phi} (0) = \frac {k _ {g} k _ {u} k _ {a} ^ {\prime} k _ {m}}{R J _ {\Sigma}} \cdot J _ {\Sigma} = K _ {0} J _ {\Sigma}\tag{4.3.25}
$$

式中 $K _ { 0 }$ 由式 (4.3.23) 确定。

## 3. 系统的超前-滞后校正

当按照上一节介绍的超前校正设计的平台系统达不到要求的静态力矩刚度时，可改用超前-滞后校正网络，即在串入超前校正网络的同时再串人滞后网络。校正网络的传递函数为

$$
G (s) = \frac {T _ {2} s + 1}{T _ {3} s + 1} \cdot \frac {T _ {2} ^ {\prime} s + 1}{T _ {3} ^ {\prime} s + 1}
$$

式中 $T _ { 2 } { = } h T _ { 3 } , T _ { 3 } ^ { \prime } { = } L T _ { 2 } ^ { \prime } , h { > } 1 , L { > } 1 , T _ { 2 } ^ { \prime } { > } T _ { 2 }$ C

## 1)滞后网络的传递函数

滞后网络的传递函数为

$$
G ^ {\prime} (s) = \frac {T _ {2} ^ {\prime} s + 1}{T _ {3} ^ {\prime} s + 1}\tag{4.3.26}
$$

式中， ${ \cal T } _ { 3 } ^ { \prime } { = } L { \cal T } _ { 2 } ^ { \prime } , L { > } 1$ 。幅频特性和相频特性如图 4.3.5 所示，图中 $\omega$ 为圆频率。

![](images/e7f8b2393b361f34e42c9cc8e251189d50e10cb96a21701166c0bcb7ac3e681b.jpg)

![](images/5e799ef28d018b8895172142c0dd1fc111b7332756bdba704834950733840b47.jpg)  
4.3.5 滞后网络的幅频特性和相频特性

![](images/9c65815d501d34aacd5b5b0ccb874c90d3492881c5e6189344cd613957ec03fb.jpg)  
4.3.6 超前-滞后校正网络的幅频特性和相频特性

由式 (4.3.26) 知，由于 $T _ { 3 } ^ { \prime } > T _ { 2 } ^ { \prime }$ ，所以

$$
\left| G _ {2} ^ {\prime} (\mathrm{j} \omega) \right| = \sqrt {\frac {\left(T _ {2} ^ {\prime} \omega\right) ^ {2} + 1}{\left(T _ {3} ^ {\prime} \omega\right) ^ {2} + 1}} \leqslant 1, \quad \left| G _ {2} ^ {\prime} (\mathrm{j} \infty) \right| = \frac {1}{L}
$$

为使高频增益达到 OdB，实际应用中，滞后网络应改造为

$$
G _ {2} (s) = L \cdot G _ {2} ^ {\prime} (s) = \frac {T _ {3} ^ {\prime}}{T _ {2} ^ {\prime}} \cdot \frac {T _ {2} ^ {\prime} s + 1}{T _ {3} ^ {\prime} s + 1} = \frac {s + \omega_ {2} ^ {\prime}}{s + \omega_ {3} ^ {\prime}}\tag{4.3.27}
$$

式中， ${ \bf \Omega } , \omega _ { 2 } ^ { \prime } = \frac { 1 } { T _ { 2 } ^ { \prime } } , \omega _ { 3 } ^ { \prime } = \frac { 1 } { T _ { 3 } ^ { \prime } }$ 。此时

$$
G _ {2} (\mathrm{j} \omega) = \frac {\mathrm{j} \omega + \omega_ {2} ^ {\prime}}{\mathrm{j} \omega + \omega_ {3} ^ {\prime}}, \quad G _ {2} (0) = \frac {\omega_ {2} ^ {\prime}}{\omega_ {3} ^ {\prime}} = \frac {T _ {3} ^ {\prime}}{T _ {2} ^ {\prime}} = L
$$

只需将 $2 0 \log | G _ { 2 } ^ { \prime } ( \mathrm { j } \omega )$ 1 的频率轴向下移 $2 0 \mathrm { l g } L$ 即为 $2 0 \log | G _ { 2 } ( \mathrm { j } \omega ) |$ 1 ，如图 4. 3. 中原点用 (0) 表 $\overline { { \overline { { \mathbf { \Omega } } } } } \overline { { \mathbf { \Omega } } }$ 频率轴用(ω)表示的图形。

## 2) 超前滞后校正网络

4. 3. 为超前滞后校正网络的幅频特性和相频特性示意图。

滞后校正网络必须与超前校正网络结合起来使用，构成超前-滞后校正网络:

$$
\begin{array}{l} F (s) = G _ {1} (s) \bullet G _ {2} (s) = G _ {1} (s) \frac {s + \omega_ {2} ^ {\prime}}{s + \omega_ {3} ^ {\prime}} \\ F (0) = G _ {1} (0) \bullet G _ {2} (0) = L \end{array}\tag{4.3.28}
$$

由式 (4.3.9) $G _ { 2 } \left( s \right)$ 滞后网络的串人提高了校正网络的低频增益，从而使平台的静态力矩刚度提高 倍。表 4.3.3 列出了对应于 的增益提高分贝值。

4.3.3 增益提高值 AH 的关系( $\left( \Delta H { = } 2 0 \lg L \right)$

<table><tr><td>L</td><td>1</td><td>2</td><td>3</td><td>4</td><td>5</td><td>6</td><td>7</td><td>8</td><td>9</td></tr><tr><td>ΔH/dB</td><td>0</td><td>6.02</td><td>9.54</td><td>12.0</td><td>14.0</td><td>15.6</td><td>16.9</td><td>18.1</td><td>19.1</td></tr><tr><td>L</td><td>10</td><td>11</td><td>12</td><td>13</td><td>14</td><td>15</td><td>16</td><td>17</td><td>18</td></tr><tr><td>ΔH/dB</td><td>20</td><td>20.9</td><td>21.6</td><td>22.3</td><td>22.9</td><td>23.5</td><td>24.1</td><td>24.6</td><td>25.1</td></tr></table>

## 3) 的确定

设串入超前校正网络后的静态力矩刚度为 $S _ { \varPhi } \left( 0 \right)$ ，而设计要求的静态力矩刚度为 $\left[ S _ { \Phi } ( 0 ) \right]$ 则应取 $L \geqslant \frac { \left[ S _ { \Phi } ( 0 ) \right] } { S _ { \Phi } ( 0 ) }$ C

4) $T _ { 2 } ^ { \prime }$ 和 $T _ { 3 } ^ { \prime }$ 的确定

$\omega _ { 2 } ^ { ' } { = } \frac { 1 } { T _ { 2 } ^ { ' } }$ 的选取原则为

从提高平台系统的频带来在 ${ \bf \nabla } , \omega _ { 2 } ^ { ' } { = } \frac { 1 } { T _ { 2 } ^ { ' } }$ 应尽量选大些;

(2) 从避免对中频区影响过大来看， ${ \bf \nabla } \cdot { \boldsymbol { \omega } } _ { 2 } ^ { \prime } = \frac { 1 } { T _ { 2 } ^ { \prime } }$ 应尽量远离 $\frac { 1 } { T _ { 2 } }$ ，一般选取 $\frac { 1 } { T _ { 2 } ^ { \prime } } = \Bigl ( \frac { 1 } { 5 } { \sim } \frac { 1 } { 1 0 } \Bigr ) \frac { 1 } { T _ { 2 } }$ 'T， 确定后 $\boldsymbol { T } _ { 3 } ^ { \prime } { = } \boldsymbol { L } \boldsymbol { T } _ { 2 } ^ { \prime }$

## 液浮陀螺单轴稳定平台设计举例

设计液浮陀螺单轴稳定平台。己选定的陀螺角动量 $H { = } 1 0 ^ { 5 } \mathrm { g \bullet \ c m ^ { 2 } / s }$ ，陀螺绕输出轴的转动惯量为 $I _ { 0 } = 2 1 5 \mathrm { g \cdot \ c m ^ { 2 } }$ ;浮油的阻尼系数为 $D { = } 1 0 ^ { 5 } \mathrm { g } \bullet \mathrm { c m } ^ { 2 } / \mathrm { s }$ 平台绕稳定轴的转动惯量为$J _ { \mathrm { { z } } } { = } 1 . 8 { \times } 1 0 ^ { 5 } { \mathrm { g } } \cdot { \mathrm { c m } } ^ { 2 }$ 0

要求平台系统经校正后满足的性能指标为: $M \leqslant 1 . 5 , \ \omega _ { C } \approx 1 1 5 , \ \left[ S _ { \phi } \left( \mathbf { \omega } \right) \right] \geqslant 3 . \ 0 4 \ \times$ $1 0 ^ { 9 } \mathrm { g } \cdot \mathrm { c m } ^ { 2 } / \mathrm { s } ^ { 2 }$ o

由给定的陀螺参数，可计算得 $k _ { g } = \frac { H } { D } { = } 1 , \tau _ { g } { = } \frac { I _ { 0 } } { D } { = } 2 . 1 5 { \times } 1 0 ^ { - 3 } \mathrm { s }$ ，所以

$$
Y _ {0} (s) = \frac {K _ {0} ^ {\prime}}{s ^ {2} (2 . 1 5 \times 1 0 ^ {- 3} s + 1)}, \quad Y _ {0} (\mathrm{j} \omega) = - \frac {K _ {0} ^ {\prime}}{\omega^ {2} (2 . 1 5 \times 1 0 ^ {- 3} \mathrm{j} \omega + 1)}
$$

查表 ι3.1，对应于 $M { \leqslant } 1$ . 开环相位裕度 $\lambda { \geqslant } 3 8 ^ { \circ } 5 5 ^ { \prime }$ ，任意选定 $K _ { \mathrm { 0 } } ^ { \prime }$ 画出 $Y _ { 0 } ( \mathrm { j } \omega )$ 的波特图，如4.3.7

从图可见，此时交截频 $\omega _ { C } ^ { \prime } \approx 3 5 0$ ，相应的相位裕量为 $- 4 5 ^ { \circ }$ ，而且在 $\left[ 0 , \infty \right)$ )全频程内，$Y _ { 0 } ( \mathrm { j } \omega )$ 〉的相位始终小于 $- 1 8 0 ^ { \circ }$ ，即相位裕量不可能出现正值，所以必须采用超前网络来提供正的相位裕度。

记系统未校正时在 $\omega _ { C }$ 处的相位为 $\varphi _ { \mathrm { 0 } } \mathrm { ~ } ( \omega _ { C } )$ 校正后在 $\omega _ { C }$ 处的相位为 $\varphi \left( \omega c \right)$ 则根据(4.3.20) ，有

$$
\varphi_ {m} \geqslant \varphi (\omega_ {C}) - \varphi_ {0} (\omega_ {C}) + \Delta \varphi
$$

式中， $\Delta \varphi$ 是考虑滞后校正(后面采用〉 $\omega _ { C }$ 处引起的附加相位滞后而附加的补偿量， $\varphi ( \omega _ { C } ) = \lambda$ $1 8 0 ^ { \circ }$ ，贝

$$
\varphi_ {m} \geqslant \lambda - 1 8 0 ^ {\circ} - \varphi_ {0} (\omega_ {C}) + \Delta \varphi
$$

由 $_ { G C } = 1 1 5$ ，从图 4.3.7 查得 $\varphi _ { \mathrm { { 0 } } } \left( \omega _ { C } \right) = - 2 0 0 ^ { \circ }$ ，取 $\lambda = 3 8 ^ { \circ } 5 5 ^ { \prime } , \Delta \varphi = 2 ^ { \circ }$ ，可计算得 $\varphi _ { m } = 6 1 ^ { \circ }$ ，查

![](images/f0223d24aebd0dff68af77ee753973c7b2fe5dd3b5723e34fabc48f0d6ff63f5.jpg)

![](images/2ca9f7c7fe3485a3eefa99b13ac5756ca4ea4570393b857ae8f2316787b421ca.jpg)  
.7 $Y _ { 0 } ( \mathrm { j } \omega )$ 的波特图

3.2 ，得 $h = 1 5$ 2

$$
\begin{array}{r l} T _ {3} & = \frac {1}{\omega_ {m} \sqrt {h}} = \frac {1}{1 1 5 \times \sqrt {1 5}} = 2. 2 \times 1 0 ^ {- 3} (\mathrm{s}), \quad T _ {2} = h T _ {3} = 0. 0 3 3 (\mathrm{s}) \\ G _ {1} (s) & = \frac {T _ {2} s + 1}{T _ {3} s + 1} = \frac {0 . 0 3 3 s + 1}{2 . 2 \times 1 0 ^ {- 3} s + 1} \\ Y _ {1} (\mathrm{j} \omega) & = \frac {- K _ {0} ^ {\prime} (0 . 0 3 3 \mathrm{j} \omega + 1)}{\omega^ {2} (2 . 1 5 \times 1 0 ^ {- 3} \mathrm{j} \omega + 1) (2 . 2 \times 1 0 ^ {- 3} \mathrm{j} \omega + 1)} \\ & = \frac {K _ {0} ^ {\prime} \sqrt {3 . 3 ^ {2} \times 1 0 ^ {- 4} \omega^ {2} + 1}}{\omega^ {2} \sqrt {(2 . 1 5 ^ {2} \times 1 0 ^ {- 6} \omega^ {2} + 1) (2 . 2 ^ {2} \times 1 0 ^ {- 6} \omega^ {2} + 1)}} \mathrm{e} ^ {\mathrm{j} \varphi_ {1} (\omega)} \end{array}
$$

在交截频率 $_ { G C } = 1 1 5$ 处，有

$$
2 0 \lg | Y _ {1} (\mathrm{j} \omega_ {\mathrm{C}}) | = 2 0 \lg \frac {K _ {0} ^ {\prime} \sqrt {3 . 3 ^ {2} \times 1 0 ^ {- 4} \omega_ {\mathrm{C}} ^ {2} + 1}}{\omega_ {\mathrm{C}} ^ {2} \sqrt {(2 . 1 5 ^ {2} \times 1 0 ^ {- 6} \omega_ {\mathrm{C}} ^ {2} + 1) (2 . 2 ^ {2} \times 1 0 ^ {- 6} \omega_ {\mathrm{C}} ^ {2} + 1)}} = 0
$$

即

$$
\begin{array}{l} \lg K _ {0} ^ {\prime} + \frac {1}{2} \lg (3. 3 ^ {2} \times 1 0 ^ {- 4} \omega_ {\mathrm{C}} ^ {2} + 1) - 2 \lg \omega_ {\mathrm{C}} - \frac {1}{2} \lg (2. 1 5 ^ {2} \times 1 0 ^ {- 6} \omega_ {\mathrm{C}} ^ {2} + 1) \\ - \frac {1}{2} \lg (2. 2 ^ {2} \times 1 0 ^ {- 6} \omega_ {\mathrm{C}} ^ {2} + 1) = 0 \end{array}
$$

从上式可解得1 $1 \mathrm { g } K _ { 0 } ^ { \prime } = 3 . 5 5 , K _ { 0 } ^ { \prime } = 3 5 4 8 , 2 0 \mathrm { l g } K _ { 0 } ^ { \prime } = 7 1 \mathrm { d B } ,$ 而由图 4.3.7 中 $G _ { 1 } ( \mathrm { j } \omega Y _ { 0 } ( \mathrm { j } \omega )$ 〉的对数幅频特性知，当 $\omega { = } 1$ . $2 0 \log | Y _ { 1 } ( \mathrm { j } \omega )$ 的值约为 105dB.所以必须将 dB线上拾到dB作为新的 dB线才能使 $2 0 \mathrm { l g } | Y _ { 1 } ( \mathrm { j } \omega _ { C } ) | = 0 \mathrm { d B }$ .所以 34dB即为增益调整量。

下面校核静态力矩刚度。由图 4.3.7.经增益调整后，原系统 $Y _ { 0 } ( \mathrm { j } \omega )$ 的交截频率为W $\omega _ { C 1 } = 5 8$ 在低频处 $| Y _ { 0 } ( \mathrm { j } \omega )$ 1 的渐近线为 40dB/十倍频，若设 $K _ { \mathrm { 0 } } = \vert \ : Y _ { \mathrm { 0 } } \ : ( \mathrm { 0 } )$ 1. 则由图得 $\frac { 2 0 1 \mathrm { g } K _ { \mathrm { 0 } } } { \mathrm { l g } \omega _ { \mathrm { C l } } } { = 4 0 }$ .$\lg K _ { 0 } = 2 \lg \omega _ { \mathrm { C 1 } } = \lg \omega _ { \mathrm { C 1 } } ^ { 2 }$ o

所以 $K _ { 0 } { = } _ { \omega _ { C 1 } ^ { 2 } }$ ，根据式(4.3.25) .静态力矩刚度为

$$
\begin{array}{r l} S _ {\Phi} (0) & = J _ {\Sigma} K _ {0} = J _ {\Sigma} \omega_ {C 1} ^ {2} = 1. 8 \times 1 0 ^ {5} \times 5 8 ^ {2} \\ & = 6. 0 6 \times 1 0 ^ {8} (\mathrm{g} \cdot \mathrm{cm} ^ {2} / \mathrm{s} ^ {2}) <   [ S _ {\Phi} (0) ] = 3. 0 4 \times 1 0 ^ {9} \mathrm{g} \cdot \mathrm{cm} ^ {2} / \mathrm{s} ^ {2} \end{array}
$$

因此，还需要涕后网络来提高系统的静态增益，以提高静态力矩刚度。

$$
L \geqslant \frac {3 . 0 4 \times 1 0 ^ {9}}{6 . 0 6 \times 1 0 ^ {8}} = 5
$$

由于前面已确定出 $T _ { \mathrm { 2 } } { = } 0 . 0 3 3 \mathrm { s }$ .所以选 ${ \frac { 1 } { T _ { 2 } ^ { \prime } } } { = } { \frac { 1 } { 6 } } { \frac { 1 } { T _ { 2 } } } , T _ { 2 } ^ { \prime } { = } 6 T _ { 2 } { = } 0 . 2 \mathrm { s } , T _ { 3 } ^ { \prime } { = } L T _ { 2 } ^ { \prime } { = } 1 \mathrm { s }$ 即 $\omega _ { 2 } ^ { \prime } = 5$ $\omega _ { 3 } ^ { \prime } = 1$ ，因此滞后网络为

$$
G _ {2} (s) = 5 \frac {s + 5}{s + 1}
$$

校正网络的传递函数为 $F ( s ) = 5 \frac { 0 . 0 3 3 s + 1 } { 2 . 2 \times 1 0 ^ { - 3 } s + 1 } \cdot \frac { s + 5 } { s + 1 } \circ$

由于串入了滞后网络，可能会对超前网络引人的相位裕量有影响，所以应该校核。串入超前滞后校正网络后，平台系统的开环频率特性为

$$
Y (\mathrm{j} \omega) = \frac {3 5 4 8 (0 . 0 3 3 \mathrm{j} \omega + 1) (\mathrm{j} \omega + 5)}{(\mathrm{j} \omega) ^ {2} (2 . 1 5 \times 1 0 ^ {- 3} \mathrm{j} \omega + 1) (2 . 2 \times 1 0 ^ {- 3} \mathrm{j} \omega + 1) (\mathrm{j} \omega + 1)}
$$

从上式得 $Y ( \mathrm { j } \omega )$ 的相位为

$$
\begin{array}{r l} \varphi (\omega_ {C}) & = - 1 8 0 ^ {\circ} + \arctan 0. 0 3 3 \omega_ {C} + \arctan \frac {\omega_ {C}}{5} - \arctan 2. 1 5 \times 1 0 ^ {- 3} \omega_ {C} \\ & - \arctan 2. 2 \times 1 0 ^ {- 3} \omega_ {C} - \arctan \omega_ {C} \\ & = - 1 8 0 ^ {\circ} + 7 5. 2 4 ^ {\circ} + 8 1. 5 1 ^ {\circ} - 1 3. 8 9 ^ {\circ} - 1 4. 2 ^ {\circ} - 8 9. 5 ^ {\circ} = - 1 4 0. 8 ^ {\circ} \end{array}
$$

所以相位裕量为

$$
\lambda = \varphi (\omega_ {C}) + 1 8 0 ^ {\circ} = 3 9. 6 ^ {\circ} > [ \lambda ] = 3 8. 5 5 ^ {\circ}
$$

所设计的平台系统满足静态力矩刚度要求和振荡度要求。

## 4.3.4 单轴平台系统对干扰和输入的响应

1.基座角运动引起的沿平台轴作用的干扰力矩

1)干摩擦力矩 $M _ { \mathrm { d 1 } } ( t )$

如图 4.3.8，设基座绕平台稳定轴做往复摇摆

$$
\theta_ {b} = \theta_ {b m} \sin \Omega t
$$

式中， $\theta _ { b m }$ 为摇摆幅度， $, \varOmega$ 为摇摆角频率。由于平台轴通过轴承支承在基座上，所以轴承对轴作用着交变的摩擦力矩，力矩方向与摇摆运动方向一致:

$$
M _ {\mathrm{dl}} (t) = \left\{ \begin{array}{l l} - M _ {\mathrm{dm}}, & - \pi <   \Omega t <   0 \\ M _ {\mathrm{dp}}, & 0 <   \Omega t <   \pi \end{array} \right.
$$

上式中，规定逆时针力矩为正，顺时针力矩为负 $M _ { \mathrm { d } \rho }$ 和 $M _ { \mathrm { d } m }$ 只表示数值大小。假设 $M _ { \mathrm { d } \boldsymbol { p } } { > } M _ { \mathrm { d } \boldsymbol { m } }$ 由于

$$
- M _ {\mathrm{d} m} = \frac {M _ {\mathrm{d} p} - M _ {\mathrm{d} m}}{2} - \frac {M _ {\mathrm{d} p} + M _ {\mathrm{d} m}}{2}, \quad M _ {\mathrm{d} p} = \frac {M _ {\mathrm{d} p} - M _ {\mathrm{d} m}}{2} + \frac {M _ {\mathrm{d} p} + M _ {\mathrm{d} m}}{2}
$$

所以

$$
M _ {\mathrm{dl}} (t) = \frac {M _ {\mathrm{d} p} - M _ {\mathrm{dm}}}{2} + \frac {M _ {\mathrm{d} p} + M _ {\mathrm{dm}}}{2} f (\Omega t)
$$

式中 $f ( \varOmega t )$ 的周期为 $2 \pi$ ，在主值周期内

$$
f (\Omega t) = \left\{ \begin{array}{l l} 1, & 0 <   \Omega t <   \pi \\ - 1, & - \pi <   \Omega t <   0 \end{array} \right.
$$

可见 $f ( \varOmega t )$ 是奇函数，傅里叶级数中 $a _ { 0 }$ 和 $a _ { k }$ 都为零，

$$
\begin{array}{r l} b _ {k} & = \frac {1}{2 \pi} \int_ {- \pi} ^ {\pi} f (\Omega t) \sin k \Omega t \mathrm{d} (\Omega t) \\ & = \frac {1}{2 \pi} \left[ \int_ {- \pi} ^ {0} - 1 \cdot \sin k \Omega t \mathrm{d} (\Omega t) + \int_ {0} ^ {\pi} 1 \cdot \sin k \Omega t \mathrm{d} (\Omega t) \right] \\ & = \frac {1}{2 \pi} \left[ \frac {1}{k} \cos k (\Omega t) \Big | _ {- \pi} ^ {0} - \frac {1}{k} \cos k (\Omega t) \Big | _ {0} ^ {\pi} \right] = \frac {1}{k \pi} (1 - \cos k \pi) \end{array}
$$

![](images/e37c1e0b64e25ee0988d373178dbc136a620f3a31c8b206a0fa65c1a41d38955.jpg)  
3. 8 平台轴受到的干摩擦力矩

所以

$$
b _ {1} = \frac {2}{\pi}, b _ {3} = \frac {2}{3 \pi}, b _ {5} = \frac {2}{5 \pi}, b _ {7} = \frac {2}{7 \pi}, \dots
$$

$$
b _ {2} = 0, b _ {4} = 0, b _ {6} = 0, \dots
$$

$$
f (\Omega t) = a _ {0} + 2 \sum_ {k = 1} ^ {\infty} \left(a _ {k} \cos k \Omega t + b _ {k} \sin k \Omega t\right) = \frac {4}{\pi} \left(\sin \Omega t + \frac {1}{3} \sin 3 \Omega t + \frac {1}{5} \sin 5 \Omega t + \dots\right)
$$

$$
M _ {\mathrm{dl}} (t) = \frac {M _ {\mathrm{d} p} - M _ {\mathrm{dm}}}{2} + \frac {2 \left(M _ {\mathrm{d} p} + M _ {\mathrm{dm}}\right)}{\pi} \left(\sin \Omega t + \frac {1}{3} \sin 3 \Omega t + \frac {1}{5} \sin 5 \Omega t + \dots\right)
$$

2) 矩电机反电势干扰力矩 $M _ { \mathrm { d 2 } } \left( t \right)$

略去地球自转的影响，基座角速度为

$$
\omega_ {i b} (t) \approx \dot {\theta} _ {b} = \theta_ {b m} \Omega \cos \Omega t
$$

根据式(4.2. ，由基座摇摆运动引起的平台干扰 矩为

$$
M _ {\mathrm{d} 2} (t) = \frac {k _ {e} k _ {m} \theta_ {b m} \Omega}{R} \cos \Omega t
$$

3) 质量不平衡干扰力矩

设平台质量为 平台重心 偏离稳定轴轴心线的偏心距为 平台稳定轴与运载体的旋转

中心( 心)间的距 运载 绕质心做往复摆。由于 稳定作用 偏心距 保持水平(此处考 力偏心力矩最大 如图 3. 9所示。由图得:重 引起的平台干扰力矩为 $m g r$ ，摇摆引起的平台质心离心力为 $m l { \dot { \theta } } _ { b } ^ { 2 }$ 此离心力引起的台干 姐为 $m l \dot { \theta } _ { b } ^ { 2 } \bullet r \mathrm { c o s } ( \theta _ { b } + \delta ) \approx - m r l \dot { \theta } _ { b } ^ { 2 }$ 所以由质 平衡引起的平台干 力矩为

$$
\begin{array}{r l} M _ {\mathrm{d3}} (t) & = m g r - m r l \left(\Omega \theta_ {b m}\right) ^ {2} \cos^ {2} \Omega t \\ & = m g r - \frac {1}{2} m r l \Omega^ {2} \theta_ {b m} ^ {2} (1 + \cos 2 \Omega t) \end{array}
$$

![](images/1bbda435cf3262fd1881ee5984800d64f79e562cda255a5906a9ccf2f2e137ac.jpg)

齿轮减速器 扰转动惯 $J _ { i }$ 引起的平台扰力

由图 引起 矩为

图4.3.9 质量不平衡干扰力矩

$$
M _ {\mathrm{d} 4} (t) = J _ {i} \dot {\omega} _ {i b} = J _ {i} \ddot {\theta} _ {b} = - J _ {i} \Omega^ {2} \theta_ {b m} \sin \Omega t
$$

式中 $, J _ { i } = \sum _ { k = 0 } ^ { n - 1 } i _ { k / n } ( i _ { k / n } - 1 ) J _ { k }$ 0

5) 扰力

$$
\begin{array}{r l} M _ {\mathrm{d}} (t) & = M _ {\mathrm{d1}} (t) + M _ {\mathrm{d2}} (t) + M _ {\mathrm{d3}} (t) + M _ {\mathrm{d4}} (t) \\ & = \frac {M _ {\mathrm{dp}} - M _ {\mathrm{dm}}}{2} + \frac {2 (M _ {\mathrm{dp}} + M _ {\mathrm{dm}})}{\pi} \left(\sin \Omega t + \frac {1}{3} \sin 3 \Omega t + \frac {1}{5} \sin 5 \Omega t + \dots\right) \\ & + \frac {k _ {e} k _ {m} \theta_ {b m} \Omega}{R} \cos \Omega t + m g r - \frac {1}{2} m r l \Omega^ {2} \theta_ {b m} ^ {2} (1 + \cos 2 \Omega t) - J _ {i} \Omega^ {2} \theta_ {b m} \sin \Omega t \\ & = M _ {\mathrm{dc}} + M _ {\mathrm{do}} \end{array}
$$

式中 常值 扰力矩

$$
M _ {\mathrm{dc}} = \frac {M _ {\mathrm{dp}} - M _ {\mathrm{dm}}}{2} + m g r - \frac {1}{2} m r l \Omega^ {2} \theta_ {b m} ^ {2}\tag{4.3.29}
$$

交变干扰力矩

$$
\begin{array}{l} M _ {\mathrm{do}} = b _ {\mathrm{do1}} \sin \Omega t + a _ {\mathrm{do1}} \cos \Omega t + \frac {2 (M _ {\mathrm{dp}} + M _ {\mathrm{dm}})}{\pi} \left(\frac {1}{3} \sin 3 \Omega t + \frac {1}{5} \sin 5 \Omega t + \dots\right) \\ - \frac {1}{2} m r l \Omega^ {2} \theta_ {b m} ^ {2} \cos 2 \Omega t \end{array}\tag{4.3.30}
$$

式中

$$
b _ {\mathrm{dol}} = \frac {2 (M _ {\mathrm{dp}} + M _ {\mathrm{dm}})}{\pi} - J _ {i} \Omega^ {2} \theta_ {b m}\tag{4.3.31a}
$$

$$
a _ {\mathrm{do1}} = \frac {k _ {e} k _ {m} \theta_ {b m} \Omega}{R}\tag{4.3.31b}
$$

2. 常值干扰力矩引起的平台误差

此处以双自由度陀螺单轴稳定平台为对象，如图 4.3.10 所示，分析常值干扰力矩引起的平台误差。

![](images/e3b278f48cd088f76c4fe37c135473217fa48dae9ccd392d2ec5f24cd0cd6619.jpg)  
4. 3. 10 双自由度陀螺单轴稳定平台方块图

由图 4.3.10

$$
G _ {\omega} (s) = \frac {\omega (s)}{M _ {\mathrm{dc}} (s)} = \frac {s}{J _ {\Sigma} s ^ {2} + Y ^ {\prime} (s)}
$$

式中 $, Y ^ { \prime } ( s ) = \frac { k _ { u } k _ { a } k _ { m } F ( s ) } { R }$ 。且

$$
G _ {\phi} (s) = \frac {\phi (s)}{M _ {\mathrm{dc}} (s)} = \frac {1}{J _ {\Sigma} s ^ {2} + Y ^ {\prime} (s)}
$$

所以

$$
\omega (s) = \frac {s M _ {\mathrm{dc}} (s)}{J _ {\Sigma} s ^ {2} + Y ^ {\prime} (s)}, \quad \phi (s) = \frac {M _ {\mathrm{dc}} (s)}{J _ {\Sigma} s ^ {2} + Y ^ {\prime} (s)}
$$

从上两式得摇摆干扰力矩中的常值分量 $M _ { \mathrm { d } c }$ 此引起的稳定平台稳态漂移角速率为

$$
\omega_ {s s} = \lim _ {s \rightarrow 0} \cdot \frac {s \cdot \frac {M _ {\mathrm{dc}}}{s}}{J _ {\Sigma} s ^ {2} + Y ^ {\prime} (s)} = 0
$$

引起的稳定平台稳态偏差角为

$$
\phi_ {s s} = \lim _ {s \rightarrow 0} s \cdot \frac {\frac {M _ {\mathrm{dc}}}{s}}{J _ {\Sigma} s ^ {2} + Y ^ {\prime} (s)} = \frac {M _ {\mathrm{dc}}}{Y ^ {\prime} (0)}
$$

根据式 (4.3.7) 和式 (4. 3. 3) $, Y ^ { \prime } ( 0 ) = \frac { k _ { u } k _ { a } k _ { m } F ( 0 ) } { R } = S _ { \phi } ( 0 )$ ，即静态力矩刚度，所以

$$
\phi_ {s s} = \frac {M _ {\mathrm{dc}}}{S _ {\phi} (0)}\tag{4.3.32}
$$

3. 周期干扰力矩引起的平台误差

当平台系统采用超前滞后校正网络时

$$
\begin{array}{r l} G _ {\phi} (s) & = \frac {1}{J _ {\Sigma} s ^ {2} + \frac {k _ {u} k _ {a} k _ {m} F (s)}{R}} \\ & = \frac {T _ {3} T _ {3} ^ {\prime} s ^ {2} + \left(T _ {3} + T _ {3} ^ {\prime}\right) s + 1}{J _ {\Sigma} s ^ {2} \left[ T _ {3} T _ {3} ^ {\prime} s ^ {2} + \left(T _ {3} + T _ {3} ^ {\prime}\right) s + 1 \right] + S _ {\phi} (0) \left[ T _ {2} T _ {2} ^ {\prime} s ^ {2} + \left(T _ {2} + T _ {2} ^ {\prime}\right) s + 1 \right]} \end{array}
$$

式中， $S _ { \phi } ( 0 ) { = } J _ { \Sigma } \omega _ { c 2 } ^ { 2 } , \omega _ { c 2 }$ 为串人校正网络后的交截频率。由于运载体的摇摆干扰低频段影响大于高频段，对 $G _ { \phi } ( \mathrm { j } \omega )$ 的低频段， $\omega { \ll } _ { < } \omega _ { c 2 }$ ，分母的前一项远小于后一项，分子的第一项远小于后两项，并且由于 $T _ { 3 } ^ { \prime } { \gg } T _ { 3 } , T _ { 2 } ^ { \prime } { = } ( 6 { \sim } 1 0 ) T _ { 2 }$ ，所以上式可近似为

$$
\begin{array}{c} G _ {\phi} (s) = \frac {T _ {3} ^ {\prime} s + 1}{S _ {\phi} (0) (T _ {2} T _ {2} ^ {\prime} s ^ {2} + T _ {2} ^ {\prime} s + 1)} \\ | G _ {\phi} (\mathrm{j} \omega) | = \frac {1}{S _ {\phi} (0)} \sqrt {\frac {1 + (T _ {3} ^ {\prime} \omega) ^ {2}}{(1 - T _ {2} T _ {2} ^ {\prime} \omega^ {2}) ^ {2} + (T _ {2} ^ {\prime} \omega) ^ {2}}} \\ \arg G _ {\phi} (\mathrm{j} \omega) = \theta_ {\phi} (\omega) = \arctan (T _ {3} ^ {\prime} \omega) - \arctan \frac {T _ {2} ^ {\prime} \omega}{1 - T _ {2} T _ {2} ^ {\prime} \omega^ {2}} \\ G _ {\omega} (s) = \frac {s}{J _ {\Sigma} s ^ {2} + \frac {k _ {u} k _ {a} k _ {m} F (s)}{R}} = s G _ {\phi} (s) \\ | G _ {\omega} (\mathrm{j} \omega) | = \omega | G _ {\phi} (\mathrm{j} \omega) | \\ \arg G _ {\omega} (\mathrm{j} \omega) = \theta_ {\omega} (\omega) = 9 0 ^ {\circ} + \theta_ {\phi} (\omega) \end{array}\tag{4.3.33}
$$

(4.3.34)

对于摇摆干扰中的一次谐波(基频)干扰

$$
M _ {\mathrm{dol}} = b _ {\mathrm{dol}} \sin \Omega t + a _ {\mathrm{dol}} \cos \Omega t = A _ {\mathrm{dl}} \sin (\Omega t + \theta_ {\mathrm{dl}})
$$

式中 $b _ { \mathrm { d o l } }$ 和 $a _ { \mathrm { d } o 1 }$ 由式 (4.3.3 1)确定

$$
A _ {\mathrm{dl}} = \sqrt {b _ {\mathrm{dol}} ^ {2} + a _ {\mathrm{dol}} ^ {2}}\tag{4.3.35}
$$

$$
\theta_ {\mathrm{dl}} = \arctan \frac {a _ {\mathrm{dol}}}{b _ {\mathrm{dol}}}\tag{4.3.36}
$$

平台对 $M _ { \mathrm { d o l } }$ 的响应为

$$
\begin{array}{r l} \phi_ {1} (t) & = A _ {\mathrm{dl}} | G _ {\phi} (\mathrm{j} \omega) | \sin [ \Omega t + \theta_ {\mathrm{dl}} + \arg G _ {\phi} (\mathrm{j} \omega) ] \\ & = \phi_ {1 m} \sin [ \Omega t + \theta_ {\mathrm{dl}} + \arg G _ {\phi} (\mathrm{j} \omega) ] \\ \omega_ {1} (t) & = A _ {\mathrm{dl}} \Omega | G _ {\phi} (\mathrm{j} \omega) | \sin [ \Omega t + \theta_ {\mathrm{dl}} + 9 0 ^ {\circ} + \arg G _ {\phi} (\mathrm{j} \omega) ] \\ & = \omega_ {1 m} \cos [ \Omega t + \theta_ {\mathrm{dl}} + \arg G _ {\phi} (\mathrm{j} \omega) ] \end{array}\tag{4.3.37}
$$

(4.3.38)

式中

$$
\phi_ {1 m} = A _ {\mathrm{dl}} \mid G _ {\phi} (\mathrm{j} \omega) \mid\tag{4.3.39}
$$

$$
\omega_ {1 m} = A _ {\mathrm{dl}} \Omega | G _ {\phi} (\mathrm{j} \omega) | = \Omega \phi_ {1 m}\tag{4.3.40}
$$

$| G _ { \phi } ( \mathrm { j } \omega )$ 和 $\arg G _ { \phi } ( \mathrm { j } \omega )$ 由式 (4.3.33) 和式 (4.3.34) 确定。

## 4. 平台对基座角振荡的隔离度

(4.3.37)和式(4.3.38)说明 当基座做角振荡时，所产生的干扰力矩会引起平台的同频率角振荡，定义平台角振荡的幅值与基座角振荡的幅值之比为稳定平台的隔离度

$$
I _ {\theta} = \frac {\phi_ {1 m}}{\theta_ {b m}}\tag{4.3.41}
$$

隔离度反映了平台对基座角振荡振幅的衰减程度，亦即对基座角运动的隔离能力， $I _ { \theta }$ 越小，表明平台的隔离效果越好，所以是平台伺服困路的重要设计指标之一，常用分贝值来表示

$$
2 0 \mathrm{lg} I _ {\theta} = 2 0 \mathrm{lg} \phi_ {1 m} - 2 0 \mathrm{lg} \theta_ {b m}\tag{4.3.42}
$$

由式 (4.3.39) 和式 (4.3.33)

$$
\phi_ {1 m} = \frac {A _ {\mathrm{dl}}}{S _ {\phi} (0)} \sqrt {\frac {1 + \left(T _ {3} ^ {\prime} \Omega\right) ^ {2}}{\left(1 - T _ {2} T _ {2} ^ {\prime} \Omega^ {2}\right) ^ {2} + \left(T _ {2} ^ {\prime} \Omega\right) ^ {2}}}
$$

可见提高平台的静态力矩刚度是减小 $I _ { \theta }$ 的有效途径。

上述分析说明稳定平台实际上不能完全隔离基座角振荡，性能卓越的平台系统的 $I _ { \theta }$ ，值很小，引起的残余振荡对系统的影响可忽略不计。但如果系统设计不合理，引起的残余振荡的幅值过大，超出了陀螺仪信号器的线性输出范围，就可能使系统出现非线性振荡而使平台不能正常工作，甚至可能使陀螺转子与止挡相撞，损坏陀螺.

## 5. 平台对指令角速度的响应

由图 4.3.10

$$
\frac {\Delta \omega (s)}{\omega_ {\mathrm{cmd}} (s)} = \frac {1}{1 + \frac {k _ {u} k _ {a} k _ {m} F (s)}{R J _ {\Sigma} s ^ {2}}} = \frac {s ^ {2}}{s ^ {2} + \frac {k _ {u} k _ {a} k _ {m} F (s)}{R J _ {\Sigma}}}
$$

对于指令角速度为常值和随时间线性变化的情况，即

$$
\omega_ {\mathrm{cmd}} (s) = \frac {\omega_ {\mathrm{cmd}}}{s} \quad \text {或} \quad \omega_ {\mathrm{cmd}} (s) = \frac {\dot {\omega} _ {\mathrm{cmd}}}{s ^ {2}}
$$

$\Delta \omega$ 的稳态值均为零，即平台系统为无静差系统，平台能正确跟踪常值和随时间线性变化的指令角速度。

