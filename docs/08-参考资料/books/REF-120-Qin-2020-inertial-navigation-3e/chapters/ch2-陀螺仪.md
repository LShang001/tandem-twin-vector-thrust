# 第2章 陀螺仪

## 第一篇惯性器件

惯性器件或称惯性仪表，即陀螺仪和加速度计。陀螺仪用来测量运载体的角运动，或在控制角运动的伺服回路中用作控制环节，加速度计用来测量运载体的加速度.其中，"惯性"具有两重含义 陀螺和加速度计服从牛顿力学，基本工作原理是动量矩定理和牛顿第二定理，即基本惯性原理 作为测量元件时输出量都是相对惯性空间的测量值，如角速度输出是相对惯性空间的角速度，加速度输出是绝对加速度，陀螺作为控制元件时，产生的角速度是相对惯性空间的角速度.

本篇主要对陀螺仪和加速度计作系统介绍。

## 第2章 陀螺仪

传统意义上的陀螺仪是指转子陀螺仪，转子陀螺仪的运动特性区别于一般刚体的根本原因在于转子旋转产生的角动量，这种陀螺仪服从牛顿力学。随着激光技术的发展，建立在全新测量原理上的另一类陀螺已经蓬勃发展起来，这就是光学陀螺，这类陀螺服从量子力学。所以日前所指的陀螺己突破了经典含义而具有广义含义。

## 2. 1 转子陀螺的力学原理

## 2. 1. 1 惯性坐标系

以地心为球，心，半径无穷大的球体称为天球。地球赤道平面无限延伸后截天球所得的圆称

为天球赤道，地球公转平面无限延伸后截天球所得的圆称天球黄道，天球赤道和天球黄道在天球上相交得春分点和秋分点，见图 2. 1. 。由于天球赤道平面垂直于地球自转轴，天球黄道平面垂直于地球绕太阳的公转轴，而地球的自转轴和公转轴在惯性空间内保持指向不变，所以春分点和秋分点在惯性空间内保持位置不变，且位于无穷远处。其中春分点的位置确定如下:若白地心向春分点引矢量 $\overrightarrow { O S }$ ，则凉的单位矢量为

$$
\pmb {u} = \frac {\pmb {\Omega} _ {\text {公}} \times \pmb {\Omega} _ {\text {自}}}{| \pmb {\Omega} _ {\text {公}} \times \pmb {\Omega} _ {\text {自}} |}\tag{2.1.1}
$$

![](images/b146b144ec816babdccd9fc4e1e3081a1a8b0322df46f37a2345b153f44e1eb0.jpg)  
2. 1. 春分点和秋分点

$$
O - x _ {i} y _ {i} z _ {i}
$$

位于地心。 $O , x _ { i }$ 轴指向春分点 ${ \mathcal { Z } } _ { i }$ 轴沿地球自转轴' $y _ { i }$ 轴与 $\mathcal { X } _ { i } \setminus \mathcal { Z } _ { i }$ 轴构成右手系。

![](images/b123cfdae5f21a2f6ccc352dfde6879bb2198de4db87d2fe3708e6bb746c049b.jpg)  
2. 1. 质点的动量矩

由于春分点在元穷远处，所以在地球的公转轨道上，无论地球运行到何处，地心心与春分点的连线始终保持平行，因此 $O { \ – } x _ { i } y _ { i } z _ { i }$ 的各轴相对惯性空间的指向保持不变，该坐标系为惯性坐标系。

## 2. 1. 2 角动量和动量矩定理

设质点的质量为 dm 质点的速度为质点的动量 Vdm 对某空间点的矩称为该质点对该点的动量矩，如图 2. 1. 所示。即

$$
\mathrm{d} \boldsymbol {H} = \boldsymbol {r} \times \mathbf {V} \mathrm{d} m
$$

式中 是空间点。至质点的距离矢量。

如果点。是曲线运动的瞬时中心，则

$$
\boldsymbol {V} = \boldsymbol {\omega} \times \boldsymbol {r}
$$

式中， $\pmb { \omega }$ 由是质点做曲线运动引起的距离矢量 的旋转角速度.

质点系的动量矩是所有质点动量矩的总和，对分布质量的质点系来说，动量矩为

$$
\boldsymbol {H} = \iiint_ {V} \boldsymbol {r} \times (\boldsymbol {\omega} \times \boldsymbol {r}) \mathrm{d} m
$$

对做定点转动的刚体来说，各质点的旋转角速度都相同，所以

$$
\boldsymbol {H} = \iiint_ {V} \boldsymbol {r} \times (- \boldsymbol {r} \times \boldsymbol {\omega}) \mathrm{d} m = - \iiint_ {V} \boldsymbol {r} \times (\boldsymbol {r} \times \boldsymbol {\omega}) \mathrm{d} m\tag{2.1.2}
$$

设刚体绕点O做定点转动，O-xyz为某一参考坐标系，记作 $g$ ，并设

$$
\boldsymbol {r} ^ {g} = \left[ \begin{array}{l} x \\ y \\ z \end{array} \right], \quad \boldsymbol {\omega} ^ {g} = \left[ \begin{array}{l} \omega_ {x} \\ \omega_ {y} \\ \omega_ {z} \end{array} \right], \quad \boldsymbol {H} ^ {g} = \left[ \begin{array}{l} H _ {x} \\ H _ {y} \\ H _ {z} \end{array} \right]
$$

则式 (2. 1. 2)

$$
\begin{array}{r l} \left[ \begin{array}{c} H _ {x} \\ H _ {y} \\ H _ {z} \end{array} \right] & = - \iiint_ {V} \left[ \begin{array}{c c c} 0 & - z & y \\ z & 0 & - x \\ - y & x & 0 \end{array} \right] \left(\left[ \begin{array}{c c c} 0 & - z & y \\ z & 0 & - x \\ - y & x & 0 \end{array} \right] \left[ \begin{array}{c} \omega_ {x} \\ \omega_ {y} \\ \omega_ {z} \end{array} \right]\right) \mathrm{d} m \\ & = - \iiint_ {V} \left[ \begin{array}{c c c} 0 & z & - y \\ - z & 0 & x \\ y & - x & 0 \end{array} \right] \left[ \begin{array}{c c c} 0 & z & - y \\ - z & 0 & x \\ y & - x & 0 \end{array} \right] \mathrm{d} m \left[ \begin{array}{c} \omega_ {x} \\ \omega_ {y} \\ \omega_ {z} \end{array} \right] \\ & = \iiint_ {V} \left[ \begin{array}{c c c} y ^ {2} + z ^ {2} & - x y & - x z \\ - x y & x ^ {2} + z ^ {2} & - y z \\ - x z & - y z & x ^ {2} + y ^ {2} \end{array} \right] \mathrm{d} m \left[ \begin{array}{c} \omega_ {x} \\ \omega_ {y} \\ \omega_ {z} \end{array} \right] = [ I ] \left[ \begin{array}{c} \omega_ {x} \\ \omega_ {y} \\ \omega_ {z} \end{array} \right] \end{array}\tag{2.1.3}
$$

式中， $[ I ] = \left[ { \begin{array} { c c c } { I _ { x } } & { - I _ { x y } } & { - I _ { x z } } \\ { - I _ { x y } } & { I _ { y } } & { - I _ { y z } } \\ { - I _ { x z } } & { - I _ { y z } } & { I _ { z } } \end{array} } \right]$ 称为刚制 $_ g$ 坐标系棚中元叫

$$
I _ {x} = \iiint_ {V} (y ^ {2} + z ^ {2}) \mathrm{d} m\tag{2.1.4a}
$$

$$
I _ {y} = \iiint_ {V} (x ^ {2} + z ^ {2}) \mathrm{d} m\tag{2.1.4b}
$$

$$
I _ {z} = \iiint_ {V} (x ^ {2} + y ^ {2}) \mathrm{d} m\tag{2.1.4c}
$$

$$
I _ {x y} = \iiint_ {V} x y \mathrm{d} m\tag{2.1.4d}
$$

$$
I _ {x z} = \iiint_ {V} x z \mathrm{d} m\tag{2.1.4e}
$$

$$
I _ {y z} = \iiint_ {V} y z \mathrm{d} m\tag{2.1.4f}
$$

式中 . $I _ { x } , I _ { y } , I _ { z }$ 分别称为同 体对 轴、 $. y$ 轴 ${ \mathcal { Z } }$ 轴的转动惯量. $I _ { x y } \setminus I _ { x z } \setminus I _ { y z }$ 称为刚体的惯性积。若刚体为匀质体，密度为 $\rho \mathrm { : }$ ，则

$$
I _ {x} = \rho \iiint_ {V} (y ^ {2} + z ^ {2}) \mathrm{d} x \mathrm{d} y \mathrm{d} z\tag{2.1.5a}
$$

$$
I _ {y} = \rho \iiint_ {V} (x ^ {2} + z ^ {2}) \mathrm{d} x \mathrm{d} y \mathrm{d} z\tag{2.1.5b}
$$

$$
I _ {z} = \rho \iiint_ {V} (x ^ {2} + y ^ {2}) \mathrm{d} x \mathrm{d} y \mathrm{d} z\tag{2.1.5c}
$$

$$
I _ {x y} = \rho \iiint_ {V} x y \mathrm{d} x \mathrm{d} y \mathrm{d} z\tag{2.1.5d}
$$

$$
I _ {x z} = \rho \iiint_ {V} x z \mathrm{d} x \mathrm{d} y \mathrm{d} z\tag{2.1.5e}
$$

$$
I _ {y z} = \rho \iiint_ {V} y z \mathrm{d} x \mathrm{d} y \mathrm{d} z\tag{2.1.5f}
$$

如果适当选择坐标系。-xyz 后能使惯性积为零，则该坐标系的各轴称为刚体的惯性主轴。对于圆柱体转子，转子的自转轴和赤道平面内与自转轴相交的所有轴都是惯性主轴。沿自转轴的转动惯量称为极转动惯量，沿赤道轴的转动惯量称为赤道转动惯量。

在转子陀螺的讨论中，常将转子具有的动量矩称为角动量。从式(2. 1. 2)可看出角动量的量纲为

$$
[ \text {角动量} ] = \frac {[ \text {质量} ] \cdot [ \text {长度} ] ^ {2}}{[ \text {时间} ]}\tag{2.1.6}
$$

对于小型陀螺而言，常用单位是克·厘米2/秒或达因①.厘米·秒和克力·厘米·秒。

$$
1 \text {克力} \cdot \text {厘米} \cdot \text {秒} = 9 8 0 \text {达因} \cdot \text {厘米} \cdot \text {秒} = 9 8 0 \text {克} \cdot \text {厘米} ^ {2} / \text {秒}
$$

2.1 设转子的极转动惯量 $I _ { z } = 3 9 8$ 克·厘米 ，转子的转速为 $n { = } 2 4 0 0 0$ 转/分钟，求转子的角动量。

解转子的自转角速度为 $\scriptstyle \omega = 2 \pi { \frac { n } { 6 0 } } = { \frac { 2 \pi \times 2 4 0 0 0 } { 6 0 } } = 2 5 1 3 .$ .27( 弧度/秒)角动量的大小为

$$
H = I _ {z} \omega = 3 9 8 \times 2 5 1 3. 2 7 = 1 0 ^ {6} (\text {克} \cdot \text {厘米} ^ {2} / \text {秒}) = 1 0 2 0 (\text {克力} \cdot \text {厘米} \cdot \text {秒})
$$

方向沿转子自转角速度方向。

2.2 如图 2. 1. ，求实心圆柱转子及空心圆柱转子的极转动惯量，其中实心转子的外径2R 空心转子的外径为 2R 内径为 2r 高度都为 密度都为 $\rho _ { \mathfrak { c } }$

![](images/f4056ff1621db41aed15e968d7be906bc1b023eea4d8a6907ac32522a4e29333.jpg)  
2. 1. 实心圆柱转子的积分微元

解在实心转子中取高度为 的微环为积分微元，该微环的内半径为).，环壁厚为d) $\mathrm { d } \lambda$ ，则由(2. 1. 4c) ,

① 达因 $\mathrm { ( d y n ) = } 1 0 ^ { - 5 } \mathrm { { N } }$ C

$$
\mathrm{d} m = 2 \pi \lambda \mathrm{d} \lambda h \rho = 2 \pi h \rho \lambda \mathrm{d} \lambda
$$

$$
I _ {z} = \int_ {0} ^ {R} \lambda^ {2} 2 \pi \lambda h \rho \mathrm{d} \lambda = \frac {\pi}{2} \rho h R ^ {4}
$$

对于空心转子

$$
I _ {z} = \int_ {r} ^ {R} \lambda^ {2} 2 \pi \lambda h \rho d \lambda = \frac {\pi}{2} \rho h (R ^ {4} - r ^ {4})
$$

定点转动质点系的动量矩定理描述为

$$
\left. \frac {\mathrm{d} \boldsymbol {H}}{\mathrm{d} t} \right| _ {i} = \boldsymbol {M}\tag{2.1.7}
$$

式中， 为定点转动质点系对该定点的角动量总和， 为作用在该质点系上对该定点的合外力矩， $\left. \frac { \mathrm { d } \pmb { H } } { \mathrm { d } t } \right|$ 表示在惯性坐标系内观察到的时间变化率此处应注意质点系可理解成多个刚体联结在一起的联结组合体。

## 2. 2 双自由度陀螺仪

## 2.2.1 定轴性、进动性和陀螺力矩

工程上，为了使陀螺转子获得三个所需要的角转动自由度，典型的办法是将陀螺转子支承在由内、外平衡环构成的卡登万向环架中，设计中确保转子质心与支承点重合，所以转子可看做定点转动刚体，如图 2.2.1 所示。图中， 为信号器，用于拾取陀螺输出角， 为力矩器，用于控制转子绕 轴和 轴的旋转。由于转子、内平衡环、外平衡环与安装座各轴之间都是互相垂直的，所以转子可以处于任意空间位置。工作过程中，转子高速自转产生必要的角动量，内、外平衡环轴是其测量轴。

![](images/e4e6df390a9f8a90dc88c19e86e63118a3a299bdbf6a2fc0e21a52e9848e474f.jpg)  
2.2.1 双自由度陀螺结构示意图

为便于分析陀螺运动的基本特性和影响陀螺特性的主要因素，假设陀螺仪制造得十分理想和完美，是一种理想化的陀螺模型，即陀螺符合下列条件:

(1)陀螺转子为一理想的轴对称旋转体，在自转过程中，转子的对称轴和自转轴完全重合。

(2) 转子的自转角速度很高，且保持常值，所形成的自转角动量远大于非自转角动量。

(3) 陀螺转子及万向支架结构刚度很大，变形微小得足以忽略不计，并且内、外平衡环的转动惯量与转子相比可忽略不计。

(4) 陀螺的自转轴，内、外平衡环轴依次严格保持相互垂直，三轴汇交于一点，陀螺内、外环组件的质，心与支点重合，所以不会产生重力力矩和惯性力矩。

(5) 万向支架系统的支撑为理想、支撑，不存在摩擦力矩和其他干扰力矩。

此时陀螺转子为绕支点做定点转动的刚体，根据动量矩定理

$$
\left. \frac {\mathrm{d} \boldsymbol {H}}{\mathrm{d} t} \right| _ {i} = \boldsymbol {M}
$$

## 1.定轴性和表观运动

M=O 时， 相对惯性空间保持恒定不变，即转子自转轴指向相对惯性空间恒定不变，这就是陀螺的定轴性。

根据哥氏定理

$$
\left. \frac {\mathrm{d} \boldsymbol {H}}{\mathrm{d} t} \right| _ {i} = \left. \frac {\mathrm{d} \boldsymbol {H}}{\mathrm{d} t} \right| _ {e} + \boldsymbol {\omega} _ {i e} \times \boldsymbol {H}
$$

式中 为与地球固连的地球坐标系。

当 ${ \bf { \cal M } } { = } { \bf { 0 } }$ 时，

$$
\left. \frac {\mathrm{d} \boldsymbol {H}}{\mathrm{d} t} \right| _ {e} = \boldsymbol {H} \times \boldsymbol {\omega} _ {i e}
$$

式中， $\cdot \frac { \mathrm { d } \pmb { H } } { \mathrm { d } t } \bigg |$ 是角动量 的矢端 在地球上观察到的速度 速度 的方向如图 2.2.2 所以小为e

$$
V = H \omega_ {i e} \sin \theta , \quad H = | \boldsymbol {H} |, \quad \omega_ {i e} = | \boldsymbol {\omega} _ {i e} |
$$

所以矢端 $O ^ { \prime } O$ 轴的旋转角速度大小为

$$
\omega_ {e 4 G} = \frac {V}{O ^ {\prime} E} = \frac {H \omega_ {i e} \sin \theta}{H \sin \theta} = \omega_ {i e}
$$

方向自 $O ^ { \prime }$ 指向 ，即有

$$
\pmb {\omega} _ {e G} = - \pmb {\omega} _ {i e}
$$

上述分析说明，当自由陀螺的角动量与地球自转角速度间的夹角。 $\theta \ne 0$ 时，地球上的观察者所看到的陀螺自转轴以 ${ \pmb { \omega } } _ { i e }$ 为角速度做旋转，旋转所形成的曲面为一圆锥面，对称轴平行于地轴，半锥角为，陀螺的这种运动称为表观运动。

![](images/ba2c569672641304b3025d165d9943b587c2a217350a01f5e25162a95e123c13.jpg)  
2.2.2 陀螺的表观运动

## 2. 进动性

当 ${ \bf \nabla } M { \ne } { \bf 0 }$ 时，根据动量矩定理

$$
\left. \frac {\mathrm{d} \boldsymbol {H}}{\mathrm{d} t} \right| _ {i} = \boldsymbol {M}
$$

式中， $\left. \frac { \mathrm { d } \pmb { H } } { \mathrm { d } t } \right|$ 是角动量 的矢端 的速度，即

$$
\boldsymbol {V} = \boldsymbol {M}
$$

上式说明角动量的矢端速度大小等于 $M { = } | M |$ ，方向平行于 $M _ { \circ }$ 由于有矢端速度存在，所以

![](images/271d914ea88ae5760b22308cabaa02486a76d8f396c09fb16b227f065d75f8ef.jpg)  
2.2.3 陀螺的进动和陀螺力矩

绕支点。旋转，转子绕 点做旋转运动，即陀螺、发生进动。

设陀螺的进动角速度为 ，由图 2. 2.

$$
| \boldsymbol {\omega} | = \frac {| \boldsymbol {V} |}{| \boldsymbol {H} |} = \frac {M}{H}
$$

由于 $\pmb { \omega }$ 是由矢端以速度 运动引起的， 垂直所在的平面，所以 位于 $H \times M$ 的方向上，该方向上的单位矢量为

因此

$$
\boldsymbol {u} = \frac {\boldsymbol {H} \times \boldsymbol {M}}{| \boldsymbol {H} \times \boldsymbol {M} |}
$$

$$
\boldsymbol {\omega} = | \boldsymbol {\omega} | \boldsymbol {u} = \frac {M}{H} \frac {\boldsymbol {H} \times \boldsymbol {M}}{H M \sin 9 0 ^ {\circ}} = \frac {\boldsymbol {H} \times \boldsymbol {M}}{H ^ {2}}
$$

上式两边叉乘 ，得

$$
\boldsymbol {\omega} \times \boldsymbol {H} = \frac {(\boldsymbol {H} \times \boldsymbol {M}) \times \boldsymbol {H}}{H ^ {2}}
$$

根据兰重矢积公式 $( { \pmb a } \times { \pmb b } ) \times { \pmb c } = ( { \pmb a } \cdot { \pmb c } ) { \pmb b } - ( { \pmb b } \cdot { \pmb c } ) { \pmb a }$ ,有

$$
\begin{array}{r l} \boldsymbol {\omega} \times \boldsymbol {H} & = \frac {1}{H ^ {2}} [ (\boldsymbol {H} \cdot \boldsymbol {H}) \boldsymbol {M} - (\boldsymbol {M} \cdot \boldsymbol {H}) \boldsymbol {H} ] \\ & = \frac {1}{H ^ {2}} (H ^ {2} \cos 0 ^ {\circ} \boldsymbol {M} - M H \cos 9 0 ^ {\circ} \boldsymbol {H}) = \boldsymbol {M} \end{array}\tag{2.2.1}
$$

(2. 2. 1)即为陀螺的进动方程。该式说明，当双自由度陀螺在某一环架轴上有作用力矩时，陀螺绕另一环架轴以 $\pmb { \omega }$ 做进动运动:角动量 以最短路径倒向外力矩 ，由此确定进动角速度的方向;进动角速度的大小由下式确定:

$$
| \omega | = \frac {M}{H}\tag{2.2.2}
$$

(2.2. 1)还说明，一旦存在外力矩，就马上出现进动角速度，所以陀螺进动是一种无惯性运动。

## 3. 陀螺力矩

在陀螺的进动方程式(2.2. 1)中， 是外部施力者(内环)加到陀螺转子上去的，根据牛顿第三定律描述的作用和反作用关系，转子一定会对该施力者作用有反作用力矩 $M _ { G }$ '该反作用力矩大小相等方向相反，即

$$
\boldsymbol {M} _ {G} = - \boldsymbol {M}
$$

$M _ { G }$ 即为陀螺力矩。由于转子支承在内环中，所以陀螺力矩 $M _ { G }$ 由转子作用在内环上， $M _ { G }$ 作用在两个不同的对象上，不能互相抵消。由式(2.2. 1)，陀螺力矩的计算公式为

$$
\boldsymbol {M} _ {G} = \boldsymbol {H} \times \boldsymbol {\omega}\tag{2.2.3}
$$

式中， 分别是转子的角动量和角速度。

## 2.2.2 双自由度陀螺仪的动力学分析

O-xyz 为测量坐标系，该坐标系与陀螺壳体固连，当陀螺无输出角时，陀螺的外环轴内环轴 及转子自转轴 分别与工轴、 轴及 轴重合。为便于讨论，定义转子坐标系(莱查坐标系 $) O { - } x _ { 1 } y _ { 1 } z _ { 1 }$ : $O z _ { 1 }$ 轴沿转子的自转轴 $O x _ { 1 }$ 和 $O y _ { 1 }$ 轴位于转子的赤道平面内，当输出角 $\alpha { = }$ $0 , \beta = 0$ 时 $O x _ { 1 }$ 轴与 $O x$ 轴重合 $O y _ { 1 }$ 轴与 $O y$ 轴重合 $O x _ { 1 }$ 轴和 $O y _ { 1 }$ 轴均不参与转子的自转。当陀螺输出角 $\alpha { \neq } 0 , \beta { \neq } 0$ 时， $O { - } x y z$ 经两次基本旋转楠 $- x _ { 1 } y _ { 1 } z _ { 1 }$ (参阅图 2.2.4) :

$$
O - x y z \xrightarrow [ \text {旋转} \alpha ]{\text {绕} O x \text {轴}} O - x y _ {1} z ^ {\prime} \xrightarrow [ \text {旋转} \beta ]{\text {绕} O y _ {1} \text {轴}} O - x _ {1} y _ {1} z _ {1}
$$

下面用动静法列写出陀螺的进动方程。

设转子的角动量为 ，陀螺、内环组件(包括转子和内环框架)绕内环轴的转动惯量为 $I _ { y }$ ，陀螺外环组件(包括内环组件和外环框架)绕外环轴的转动惯量为 $I _ { x }$ '在内外环轴上分别作用有外力矩 $M _ { x }$ 和 $M _ { y }$ 基座具有的角速度 O-XYZ 坐标系内的分量为 $\omega _ { x } \circ \omega _ { y } \circ \omega _ { z }$ ，如图 2.2.4 所示。根据动静法，各环架轴上的力矩相平衡，得方程:

$$
\begin{array}{l} M _ {x} - I _ {x} (\ddot {\alpha} + \dot {\omega} _ {x}) - H \dot {\beta} - H \omega_ {y} - H \cos \beta \omega_ {z} \sin \alpha = 0 \\ M _ {y} + H \dot {\alpha} + H \omega_ {x} - I _ {y} (\ddot {\beta} + \dot {\omega} _ {y}) - H \omega_ {z} \cos \alpha \sin \beta = 0 \end{array}
$$

![](images/43d709aac0e4240b1e59d1b344195ada01ef2437c709091e8ac7b5ba0ba35078.jpg)  
2.2.4 双自由度陀螺仪的技术方程模型

由于工程应用中陀螺正常工作时的输出角 $\alpha$ 和 $\beta$ 都很小，所以上述方程可近似为

$$
I _ {x} \ddot {\alpha} + H \dot {\beta} + H \omega_ {z} \alpha = M _ {x} - I _ {x} \dot {\omega} _ {x} - H \omega_ {y}\tag{2.2.4a}
$$

$$
I _ {y} \ddot {\beta} - H \dot {\alpha} + H \omega_ {z} \beta = M _ {y} - I _ {y} \dot {\omega} _ {y} + H \omega_ {x}\tag{2.2.4b}
$$

(2.2. 的是在近似条件下推得的，习惯上称之为陀螺的技术方程。该式说明，影响陀螺输出角 $\alpha$ 和 $\beta$ 的因素有外力矩和基座角运动。由于技术方程是一组线性方程，具有线性系统的可加性，所以可以分别单独考虑基座角速度和外力矩的影响，此处仅考虑外力矩的影响:

$$
I _ {x} \ddot {\alpha} + H \dot {\beta} = M _ {x}\tag{2.2.5a}
$$

$$
I _ {y} \ddot {\beta} - H \dot {\alpha} = M _ {y}\tag{2.2.5b}
$$

对式(2. 2. 5)作拉氏变换，并假设在外力矩作用之前，陀螺输出角的初始条件为 ，即 $\alpha ( 0 ) =$

$\beta ( 0 ) = \dot { \alpha } \left( 0 \right) = \dot { \beta } ( 0 ) = 0$ ，则

$$
I _ {x} s ^ {2} \alpha (s) + H s \beta (s) = M _ {x} (s)\tag{2.2.6a}
$$

$$
I _ {y} s ^ {2} \beta (s) - H s \alpha (s) = M _ {y} (s)\tag{2.2.6b}
$$

根据式(2.2.6)可得陀螺的方块图，如图 2.2.5 所示。

![](images/0f5a03f287652a8ff31e13937f1ad753277ab29f0a588e89ca446a5a25a7bc99.jpg)  
2.2.5 双自由度陀螺仪的等效方块图

根据图 2.2.5 ，可得出各变量间的传递函数:

$$
\frac {\alpha (s)}{M _ {x} (s)} = \frac {I _ {y}}{I _ {x} I _ {y} s ^ {2} + H ^ {2}}\tag{2.2.7a}
$$

$$
\frac {\alpha (s)}{M _ {y} (s)} = - \frac {H}{s \left(I _ {x} I _ {y} s ^ {2} + H ^ {2}\right)} = - \frac {1}{s H} + \frac {\frac {I _ {x} I _ {y}}{H} s}{I _ {x} I _ {y} s ^ {2} + H ^ {2}}\tag{2.2.7b}
$$

$$
\begin{array}{l} \frac {\beta (s)}{M _ {x} (s)} = \frac {H}{s \left(I _ {x} I _ {y} s ^ {2} + H ^ {2}\right)} = \frac {1}{s H} - \frac {\frac {I _ {x} I _ {y}}{H} s}{I _ {x} I _ {y} s ^ {2} + H ^ {2}} \\ \frac {\beta (s)}{M _ {y} (s)} = \frac {I _ {x}}{I _ {x} I _ {y} s ^ {2} + H ^ {2}} \end{array}\tag{2.2.7c}
$$

(2. 2. 7d)

系统的特征方程为

$$
I _ {x} I _ {y} s ^ {2} + H ^ {2} = 0
$$

可见双自由度陀螺仪是一个二阶系统，系统的阻尼系数和自振频率为

$$
\xi = 0, \quad \omega_ {n} = \frac {H}{\sqrt {I _ {x} I _ {y}}}
$$

在具体设计中，陀螺的内、外环框架都设计得比较轻巧，所以 $I _ { x }$ 和 $I _ { y }$ 近似等于转子的赤道转动惯量 e'且转子的极转动惯量 $I _ { \mathrm { p } }$ 设计得约为 $I _ { \mathrm { e } }$ 倍，所以陀螺自振频率的估算值为

$$
\omega_ {n} \approx \frac {2 I _ {\mathrm{e}} \Omega}{\sqrt {I _ {\mathrm{e}} I _ {\mathrm{e}}}} = 2 \Omega
$$

式中， $, \varOmega$ 为转子的自转角速度。

从式(2. 2. 7a)和式(2. 2. 7d)可看出，作用在某一轴上的外力矩只引起陀螺绕该轴的元阻尼振荡，振荡频率为自转角速度的 倍。从式(2. 2. 7b)和式(2.2.7c)可看出，作用在某一轴上的外力矩引起陀螺绕另一轴的运动有两种，具体如下所述。

第一种运动:元阻尼振荡，振荡角频率也是转子自转角速度的 倍。

第二种运动

$$
\frac {\alpha (s)}{M _ {y} (s)} = - \frac {1}{s H}, \quad \frac {\beta (s)}{M _ {x} (s)} = \frac {1}{s H}
$$

即

$$
s \alpha (s) = - \frac {M _ {y} (s)}{H}, \quad s \beta (s) = \frac {M _ {x} (s)}{H}
$$

若外力矩 $M _ { x }$ 和 $M _ { y }$ ，为常值，则输出角速度为

$$
\dot {\alpha} (t) = - \frac {M _ {y}}{H}\tag{2.2.8a}
$$

$$
\dot {\beta} (t) = \frac {M _ {x}}{H}\tag{2.2.8b}
$$

式中， $\dot { \alpha } ( t )$ 和 $\dot { \boldsymbol { \beta } } ( t )$ 是陀螺绕外环轴和内环轴的进动角速度，式(2.2.8)是式(2.2. 1)的分量表达式。

频率为 倍自转角速度的无阻尼振荡是双自由度陀螺仪的章动。章动对陀螺仪的测量精度是不利的，但它是固有的。由于章动是一种高频微幅振荡，所以只要陀螺的角动量足够大，其对测量精度的影响是可以忽略不计的。但在设计中应注意避免共振，环境振动角频率不能是转子自转角速度的偶数倍。

## 2.2.3 双自由度角位置陀螺的传递函数

当双自由度陀螺仪用于控制被稳定对象时，陀螺的信号器输出用来控制被稳定对象的相应轴，陀螺是伺服回路中的一个环节，而陀螺本身处在开环状态。若要使被控制对象以要求的角速度旋转，则指令量需加入陀螺相应轴的力矩器中。所以伺服回路中，陀螺被当作角位置陀螺使用。

## 1.对基座角位移的传递函数

当基座沿 $_ { \mathcal { X } }$ 轴和 $y$ 轴有角位移 $\varPhi _ { \mathscr { s } }$ 和矶 $\varPhi _ { y }$ ，由于陀螺的定轴性，陀螺沿外环轴和内环轴的输出角为

$$
\alpha = - \Phi_ {x}, \quad \beta = - \Phi_ {y}
$$

所以陀螺对基座角位移的传递函数为

$$
\frac {\alpha (s)}{\Phi_ {x} (s)} = - 1\tag{2.2.9a}
$$

$$
\frac {\beta (s)}{\Phi_ {y} (s)} = - 1\tag{2.2.9b}
$$

从式(2. 2. 9)可得陀螺对基座角速度的传递函数

$$
\frac {\alpha (s)}{\omega_ {x} (s)} = - \frac {1}{s}\tag{2.2.10a}
$$

$$
\frac {\beta (s)}{\omega_ {y} (s)} = - \frac {1}{s}\tag{2.2.10b}
$$

## 2. 对指令角速度的传递函数

若要求被控对象以角速度 $\omega _ { \mathrm { c m d } x }$ 和 $\omega _ { \mathrm { c m d y } }$ 分别绕 轴和 $y$ 轴旋转，则应根据下式计算出相应的指令力矩

$$
\omega_ {\mathrm{cmd} x} = - \frac {M _ {\mathrm{cmd} y}}{H}, \quad \omega_ {\mathrm{cmd} y} = \frac {M _ {\mathrm{cmd} x}}{H}
$$

并根据计算得的指令力矩和力姐器的电压/力矩系数确定出相应的控制电压加至力矩器电枢回路中，使陀螺进动，所产生的输出角的变化率与指令角速度相等，即

$$
\dot {\alpha} = \omega_ {\mathrm{cmd} x}, \quad \dot {\beta} = \omega_ {\mathrm{cmd} y}
$$

所以陀螺对指令角速度的传递函数为

$$
\frac {\alpha (s)}{\omega_ {\mathrm{cmd} x} (s)} = \frac {1}{s}\tag{2.2.11a}
$$

$$
\frac {\beta (s)}{\omega_ {\mathrm{cmdy}} (s)} = \frac {1}{s}\tag{2.2.11b}
$$

## 2. 3 单自由度陀螺仪

## 2.3.1 单自由度陀螺仪的技术方程

2.3.1 为单自由度陀螺仪的模型简图，单自由度陀螺仪比双自由度陀螺仪少一个旋转自由度，陀螺的力矩器和信号器都安装在输出轴上，所以动力学特性与双自由度陀螺相比有很大的差别。

![](images/927c72b1eb637b7f1a71dffd8dee23e05917cec7ca56894ab05e3d352803da96.jpg)  
2.3.1 单自由度陀螺仪的模型简图

取 $O { - } x y z$ 为测 坐标系，该坐标系与基座固连，记为 $B , O \mathrm { - } x _ { _ G } y _ { _ { G } } z _ { _ G }$ 为陀螺、组件坐标系，记为，其中 Ox 轴对应于陀螺的输入轴 $I , O y$ 轴对应于陀螺的输出轴 $O , O z _ { _ G }$ 轴对应于转子的自转轴S。

以环架和转子组成的陀螺组件为研究对象，陀螺组件总角动量为

$$
\boldsymbol {H} _ {\Sigma} = [ \boldsymbol {I} ] \boldsymbol {\omega} _ {i G} + \boldsymbol {H}\tag{2.3.1}
$$

式中， 为转子产生的角动量，[IJ为陀螺组件的惯性张量， ${ \pmb { \omega } } _ { i G } = { \pmb { \omega } } _ { i B } + { \pmb { \omega } } _ { B G }$

根据动量矩定理

$$
\left. \frac {\mathrm{d} \boldsymbol {H} _ {\Sigma}}{\mathrm{d} t} \right| _ {i} = \boldsymbol {M}
$$

GP

$$
\left. \frac {\mathrm{d} \boldsymbol {H}}{\mathrm{d} t} \right| _ {i} + [ \boldsymbol {I} ] \left(\frac {\mathrm{d} \boldsymbol {\omega} _ {i B}}{\mathrm{d} t} \right| _ {i} + \left. \frac {\mathrm{d} \boldsymbol {\omega} _ {B G}}{\mathrm{d} t} \right| _ {i}) = \boldsymbol {M}
$$

应用哥氏定理，上式可写成

$$
\left. \frac {\mathrm{d} \boldsymbol {H}}{\mathrm{d} t} \right| _ {G} + \boldsymbol {\omega} _ {i G} \times \boldsymbol {H} + [ \boldsymbol {I} ] \left(\frac {\mathrm{d} \boldsymbol {\omega} _ {i B}}{\mathrm{d} t} \right| _ {B} + \boldsymbol {\omega} _ {i B} \times \boldsymbol {\omega} _ {i B} + \frac {\mathrm{d} \boldsymbol {\omega} _ {B G}}{\mathrm{d} t} \Big | _ {B} + \boldsymbol {\omega} _ {i B} \times \boldsymbol {\omega} _ {B G}) = \boldsymbol {M}
$$

上式向 坐标系投影

$$
\left. \frac {\mathrm{d} \boldsymbol {H}}{\mathrm{d} t} \right| _ {G} ^ {G} + \boldsymbol {\omega} _ {i G} ^ {G} \times \boldsymbol {H} ^ {G} + [ \boldsymbol {I} ] \boldsymbol {C} _ {B} ^ {G} \left(\frac {\mathrm{d} \boldsymbol {\omega} _ {i B}}{\mathrm{d} t} \right| _ {B} ^ {B} + \frac {\mathrm{d} \boldsymbol {\omega} _ {B G}}{\mathrm{d} t} \left| _ {B} ^ {B} + \boldsymbol {\omega} _ {i B} ^ {B} \times \boldsymbol {\omega} _ {B G} ^ {B}\right) = \boldsymbol {M} ^ {G}
$$

GP

$$
\dot {\boldsymbol {H}} ^ {G} + \boldsymbol {\omega} _ {i G} ^ {G} \times \boldsymbol {H} ^ {G} + [ \boldsymbol {I} ] \boldsymbol {C} _ {B} ^ {G} (\dot {\boldsymbol {\omega}} _ {i B} ^ {B} + \dot {\boldsymbol {\omega}} _ {B G} ^ {B} + \boldsymbol {\omega} _ {i B} ^ {B} \times \boldsymbol {\omega} _ {B G} ^ {B}) = \boldsymbol {M} ^ {G}\tag{2.3.2}
$$

设基座角速度、陀螺组件的惯性张量及作用在陀螺组件上的外力矩分别为

$$
\boldsymbol {\omega} _ {i B} ^ {B} = \left[ \begin{array}{c c c} \omega_ {x} & \omega_ {y} & \omega_ {z} \end{array} \right] ^ {\mathrm{T}}, \quad [ \boldsymbol {I} ] = \operatorname{diag} \left[ \begin{array}{c c c} I _ {I} & I _ {O} & I _ {S} \end{array} \right], \quad \boldsymbol {M} ^ {G} = \left[ \begin{array}{c c c} M _ {I} ^ {G} & M _ {O} ^ {G} & M _ {S} ^ {G} \end{array} \right] ^ {\mathrm{T}}
$$

由图 2.3.1 ，有

$$
\boldsymbol {H} ^ {G} = \left[ \begin{array}{c c c} 0 & 0 & H \end{array} \right] ^ {\mathrm{T}}, \quad \boldsymbol {\omega} _ {B G} ^ {G} = \left[ \begin{array}{c c c} 0 & \dot {\alpha} & 0 \end{array} \right] ^ {\mathrm{T}}
$$

并且单自由度陀螺正常工作时，输出角 控制得很小，转子角动量应为常值，所以有

$$
\boldsymbol {C} _ {B} ^ {\mathrm{G}} = \left[ \begin{array}{c c c} 1 & 0 & - \alpha \\ 0 & 1 & 0 \\ \alpha & 0 & 1 \end{array} \right], \quad \dot {\boldsymbol {H}} ^ {\mathrm{G}} = \left[ \begin{array}{l l l} 0 & 0 & \dot {\boldsymbol {H}} \end{array} \right] ^ {\mathrm{T}} = \left[ \begin{array}{l l l} 0 & 0 & 0 \end{array} \right] ^ {\mathrm{T}}
$$

上述诸关系式代入式(2.3.2) ，得

$$
\left[ \begin{array}{c c c} 0 & - H & 0 \\ H & 0 & 0 \\ 0 & 0 & 0 \end{array} \right] \left[ \begin{array}{l} \omega_ {x} - \alpha \omega_ {z} \\ \omega_ {y} + \dot {\alpha} \\ \alpha \omega_ {x} + \omega_ {z} \end{array} \right] + \left[ \begin{array}{c c c} I _ {I} & 0 & 0 \\ 0 & I _ {O} & 0 \\ 0 & 0 & I _ {S} \end{array} \right] \left[ \begin{array}{c c c} 1 & 0 & - \alpha \\ 0 & 1 & 0 \\ \alpha & 0 & 1 \end{array} \right]
$$

$$
\times \left(\left[ \begin{array}{c} \dot {\omega} _ {x} \\ \dot {\omega} _ {y} \\ \dot {\omega} _ {z} \end{array} \right] + \left[ \begin{array}{c} 0 \\ \ddot {\alpha} \\ 0 \end{array} \right] - \left[ \begin{array}{c c c} 0 & 0 & \dot {\alpha} \\ 0 & 0 & 0 \\ - \dot {\alpha} & 0 & 0 \end{array} \right] \left[ \begin{array}{c} \omega_ {x} \\ \omega_ {y} \\ \omega_ {z} \end{array} \right]\right) = \left[ \begin{array}{c} M _ {I} ^ {G} \\ M _ {O} ^ {G} \\ M _ {S} ^ {G} \end{array} \right]
$$

由上式可得沿输出轴 的动力学方程

$$
H \left(\omega_ {x} - \alpha \omega_ {z}\right) + I _ {O} \left(\dot {\omega} _ {y} + \ddot {\alpha}\right) = M _ {O} ^ {G}\tag{2.3.3}
$$

式中， $M _ { O } ^ { G }$ 为沿陀螺输出轴的外作用力矩，在图 2.3.1 存在弹性约束和阻尼的情况下

$$
M _ {O} ^ {G} = - k \alpha - D \dot {\alpha}
$$

式中 为弹性力矩系数， 为阻尼力矩系数。

为简化分析，忽略非敏感轴方向上的角速度的影响，并记 $\omega _ { x } { = } _ { \omega _ { I } }$ ，则式(2.3.3)可以近似为

$$
I _ {O} \ddot {\alpha} + D \dot {\alpha} + k \alpha = H \omega_ {I}\tag{2.3.4}
$$

上式即为单自由度陀螺仪的技术方程。由该式可看出单自由度陀螺仪敏感沿 轴方向的角速度，所以 轴为其输入轴。

## 2.3.2 单自由度陀螺仪的分类

1)若 $k { = } 0 , D { \neq } 0$

(2. 3. 4)

$$
I _ {O} \ddot {\alpha} + D \dot {\alpha} = H \omega_ {I}
$$

陀螺达到稳态时 $\overset { \bullet } { \alpha } = 0$ ，所以

$$
\dot {\alpha} = \frac {H}{D} \omega_ {I}, \quad \alpha = \frac {H}{D} \int \omega_ {I} \mathrm{d} t
$$

即输出角与输入角速度的积分成正比，所以 $k = 0 , D \neq 0$ 的单自由度陀螺称之为积分陀螺。无弹性约束的液浮陀螺就属于此类陀螺。

2) $D { = } 0 , k { \neq } 0$

(2. 3. 4)

$$
I _ {O} \ddot {\alpha} + k \alpha = H \omega_ {I}
$$

陀螺达到稳态时

$$
\alpha = \frac {H}{k} \omega_ {I}
$$

即输出角与输入角速度成正比，所以 $D { = } 0 , k { \neq } 0$ 的单自由度陀螺称之为速率陀螺。

3) $D { = } 0 , k { = } 0$

(2. 3. 4)

$$
\begin{array}{l} I _ {O} \ddot {\alpha} = H \omega_ {I} \\ \alpha = \frac {H}{I _ {O}} \iint \omega_ {I} \mathrm{d} \tau \mathrm{d} t \end{array}
$$

即输出角与输入角速度的二次积分成正比，所以 $D = 0 , k = 0$ 的单自由度陀螺称之为双重积分陀螺。

## 2.3.3 单自由度陀螺的传递函数

液浮陀螺采用浮桶液浮支承，浮桶产生的浮力与陀螺组件的重力对消，使陀螺组件沿输出轴的摩擦力矩接近为零，所以精密制造的液浮陀螺的漂移十分微小，达到惯性级水平，即漂移优于0.01"/h，这种精度等级的液浮陀螺常用来构建高精度平台式惯导的惯性平台。因此积分陀螺常用于稳定平台和惯性平台的设计。

忽略交叉耦合 $\alpha H _ { G o _ { z } }$ ，并假设输出轴上作用有指令力矩 $M _ { \mathrm { c m d } }$ 和干扰力矩 $M _ { \mathrm { { i n t } } }$ ，则由式 (2. 3. 3)

$$
I _ {O} \ddot {\alpha} + D \dot {\alpha} = H \omega_ {I} + M _ {\mathrm{cmd}} + M _ {\mathrm{int}} - I _ {O} \dot {\omega} _ {O}
$$

式中， $, \dot { \omega } \mathrm { o }$ 。是基座沿陀螺输出轴的角加速度。上式两边作拉氏变换，得

$$
(I _ {O} s ^ {2} + D s) \alpha (s) = H \omega_ {I} (s) + M _ {\mathrm{cmd}} (s) + M _ {\mathrm{int}} (s) - I _ {O} \dot {\omega} _ {O} (s)\tag{2.3.5}
$$

1)陀螺对输入角速度的传递函数

由式 (2. 3. 5)

$$
\frac {\alpha (s)}{\omega_ {I} (s)} = \frac {H}{s \left(I _ {O} s + D\right)} = \frac {k _ {g}}{s \left(\tau_ {g} s + 1\right)}\tag{2.3.6}
$$

式中， $k _ { g } = \frac { H } { D }$ 为陀螺的传递系数 $\tau _ { g } = \frac { I _ { O } } { D }$ 为陀螺的时间常数。

## 2) 陀螺对指令角速度的传递函数

积分陀螺工作在伺服状态时，陀螺与稳定平台的连接方式如图 2.3.2 所示。

![](images/96cecf4ac34ca177bd7a0a72513073fef79a1e2c92ea622955228d752a611a98.jpg)  
2. 3. 2 陀螺与稳定平台的连接

陀螺控制平台的过程为:若要求平台以指令角速度 ${ \pmb \omega } _ { \mathrm { c m d } }$ 绕 $x _ { \phi }$ 的正方向旋转，则需向陀螺力矩器 加人施矩电流，产生沿陀螺输出轴 负方向的指令力矩 $M _ { \mathrm { c m d } }$ ，使陀螺产生负的输出角，信号器 拾取该输出角，经反相和放大后加至力矩马达 $\vec { \cal J } ^ { \pm }$ 产生 $x _ { p }$ 正向的拖动力矩拖动平台以$\omega _ { \mathrm { c m d } }$ 的角速度旋转。此时陀螺感测到该旋转角速度，转子产生陀螺力矩

$$
\pmb {M} _ {G} = \pmb {H} \times \pmb {\omega} _ {\mathrm{cmd}}
$$

$M _ { G }$ 由转子施加陀螺环架，当陀螺达到稳态时

$$
\boldsymbol {M} _ {\mathrm{cmd}} = - \boldsymbol {M} _ {G}
$$

上式在 轴的投影形式为

$$
M _ {\mathrm{cmd}} = - M _ {G} = - H \omega_ {\mathrm{cmd}}\tag{2.3.7}
$$

由式 (2. 3. 5)

$$
(I _ {O} s ^ {2} + D s) \alpha (s) = M _ {\mathrm{cmd}} (s)
$$

将式 (2.3. 7) 代入上式，得

$$
(I _ {O} s ^ {2} + D s) \alpha (s) = - H \omega_ {\mathrm{cmd}} (s)
$$

所以

$$
\frac {\alpha (s)}{\omega_ {\mathrm{cmd}} (s)} = - \frac {H}{I _ {O} s ^ {2} + D s} = - \frac {k _ {\mathrm{g}}}{s (\tau_ {\mathrm{g}} s + 1)}\tag{2.3.8}
$$

## 3) 对陀螺漂移的传递函数

陀螺漂移由沿输出轴 的干扰力矩 $M _ { \mathrm { i n t } }$ 引起。假设 ${ \pmb \omega } _ { \mathrm { c m d } } = 0 , { \pmb M } _ { \mathrm { i n t } } { \mpb \omega }$ 。由于陀螺、不能区分作用在输出轴上的力矩是指令力矩还是干扰力矩，此时伺服回路误将干扰力矩当作指令力矩而控制平台旋转，所产生的平台旋转角速度即为平台漂移，所以对漂移的传递函数与对指令角速度的传递函数是相同的，即

$$
\frac {\alpha (s)}{\varepsilon (s)} = - \frac {k _ {g}}{s (\tau_ {g} s + 1)}\tag{2.3.9}
$$

。对沿输出轴的基座角加速度 $\dot { \omega } _ { O }$ 的传递函数

由式 (2. 3. 5)

$$
\begin{array}{l} (I _ {O} s ^ {2} + D s) \alpha (s) = - I _ {O} \dot {\omega} _ {O} (s) \\ \frac {\alpha (s)}{\dot {\omega} _ {O} (s)} = - \frac {I _ {O}}{I _ {O} s ^ {2} + D s} = - \frac {I _ {O} / D}{s (\tau_ {g} s + 1)} \end{array}\tag{2.3.10}
$$

综合式 (2. 3. 6)\~ (2. 3. 10) ，可画出积分陀螺的方块图，如图 2.3.3 所示。

![](images/1e8a92996263aedacf8c83de0a112380da863a0e566e5e0f3864a1485117ecf0.jpg)  
2. 积分陀螺方块图

## 2.3.4 单自由度陀螺仪的静态漂移误差数学模型

如果将单自由度陀螺仪的信号器输出经过放大后加至力矩器，则陀螺工作在力反馈状态，即闭环状态，图 .3.4 为其工作原理图。加至力矩器的电信号正比于基座的旋转角速度，所以此时陀螺实际上是基座角运动的测量环节，在捷联惯导系统中，用此类陀螺测量运载体的角速度。作用在陀螺上的干扰力矩会引起陀螺对角速度的测量误差。

![](images/7977229740e3d0d6dc81a754adade686ac3eddb65c07b2fc1479f876e5223c33.jpg)  
2.3.4 具有力反馈再平衡回路的单自由度陀螺仪方块图

由图 2.3.4

$$
i (s) = \frac {H \omega_ {I} (s) + M _ {\mathrm{d}} (s)}{I _ {O} s ^ {2} + D s + k} \cdot k _ {u} k _ {i}
$$

式中 $\mathrm { , } k { = } k _ { u } k _ { i } k _ { m } , k _ { u }$ 为信号器的传递系数 $k _ { i }$ 为功率放大器的放大倍数 $k _ { m }$ 为力矩器的力矩流系数 $M _ { \mathrm { d } }$ 为沿输出轴的干扰力矩 上式，被测角速度的精确解应为

$$
\omega_ {I} (s) = \frac {I _ {O} s ^ {2} + D s + k}{H} \cdot \frac {i (s)}{k _ {u} k _ {i}} - \frac {M _ {\mathrm{d}} (s)}{H}
$$

但实际测量中，角速度只能根据反馈电流 i(s)确定:

$$
\hat {\omega} _ {I} (s) = \frac {I _ {O} s ^ {2} + D s + k}{H} \cdot \frac {i (s)}{k _ {u} k _ {i}}
$$

引起的测量误差即陀螺漂移:

$$
\omega_ {\mathrm{d}} (s) = \hat {\omega} _ {I} (s) - \omega_ {I} (s) = \frac {M _ {\mathrm{d}} (s)}{H}\tag{2.3.11}
$$

## 1.由固定偏心引起的干扰力矩

Or 为陀螺、组件的回转中心 为陀螺的输入轴、输出轴和自转轴，陀螺、组件质量为质心沿 方向的偏心量为 $l _ { I } , l _ { O } , l _ { S }$ 基座具有的加速度为 $a _ { I } , \mathbf { \nabla } , a _ { O } , a _ { S }$ ，由基座运动引起的单位质量'惯性力为 $F _ { I } , F _ { O } , F _ { S }$ ，如图 2.3.5 所示。由惯性力引起的绕轴 的干扰力矩为

$$
M _ {\mathrm{dl}} = m F _ {I} l _ {S} - m F _ {S} l _ {I}\tag{2.3.12}
$$

![](images/fe6eb0cb4da892175a82c369e56e834fd78a1b406a97138110e706c6712d1f17.jpg)  
2.3.5 静态和动态偏心引起的干扰力矩

## 2. 由动态偏心引起的干扰力矩

设基座静止时陀螺组件质心与 Or 重合，则在惯性力作用下弹性变形引起的质，心位移为

$$
\left[ \begin{array}{l} \delta_ {I} \\ \delta_ {O} \\ \delta_ {S} \end{array} \right] = m \left[ \begin{array}{c c c} C _ {I I} & C _ {I O} & C _ {I S} \\ C _ {O I} & C _ {O O} & C _ {O S} \\ C _ {S I} & C _ {S O} & C _ {S S} \end{array} \right] \left[ \begin{array}{l} F _ {I} \\ F _ {O} \\ F _ {S} \end{array} \right] = \left[ \begin{array}{l} m (C _ {I I} F _ {I} + C _ {I O} F _ {O} + C _ {I S} F _ {S}) \\ m (C _ {O I} F _ {I} + C _ {O O} F _ {O} + C _ {O S} F _ {S}) \\ m (C _ {S I} F _ {I} + C _ {S O} F _ {O} + C _ {S S} F _ {S}) \end{array} \right]
$$

式中 $C _ { i j }$ 方向的载荷引起 方向变形的柔度系数。

动态偏心引起的干扰力矩为

$$
\begin{array}{r l} M _ {\mathrm{d2}} & = m F _ {I} \delta_ {S} - m F _ {S} \delta_ {I} \\ & = m ^ {2} C _ {S O} F _ {O} F _ {I} - m ^ {2} C _ {I O} F _ {O} F _ {S} + m ^ {2} (C _ {S S} - C _ {I I}) F _ {S} F _ {I} + m ^ {2} C _ {S I} F _ {I} ^ {2} - m ^ {2} C _ {I S} F _ {S} ^ {2} \end{array}
$$

## 3. 静态漂移数学模型

除动、静态偏心引起的干扰力矩外，还有由导电游丝等引起的固定干扰力矩 $M _ { \mathrm { d 0 } }$ ，根据(2. 3. 11) ，由 $M _ { \mathrm { d 0 } } \Omega M _ { \mathrm { d l } }$ 和 $M _ { \mathrm { d 2 } }$ 引起的漂移为

$$
\begin{array}{r l} \omega_ {\mathrm{d}} & = \frac {1}{H} (M _ {\mathrm{d0}} + M _ {\mathrm{d1}} + M _ {\mathrm{d2}}) \\ & = [ M _ {\mathrm{d0}} + m F _ {I} l _ {S} - m F _ {S} l _ {I} + m ^ {2} C _ {S O} F _ {I} F _ {O} - m ^ {2} C _ {I O} F _ {O} F _ {S} \\ & \quad + m ^ {2} (C _ {S S} - C _ {I I}) F _ {S} F _ {I} + m ^ {2} C _ {S I} F _ {I} ^ {2} - m ^ {2} C _ {I S} F _ {S} ^ {2} ] \frac {1}{H} \\ & = \frac {M _ {\mathrm{d0}}}{H} + \frac {m l _ {S}}{H} F _ {I} - \frac {m l _ {I}}{H} F _ {S} + \frac {m ^ {2} C _ {S O}}{H} F _ {I} F _ {O} - \frac {m ^ {2} C _ {I O}}{H} F _ {O} F _ {S} \\ & \quad + \frac {m ^ {2} (C _ {S S} - C _ {I I})}{H} F _ {S} F _ {I} + \frac {m ^ {2} C _ {S I}}{H} F _ {I} ^ {2} - \frac {m ^ {2} C _ {I S}}{H} F _ {S} ^ {2} \end{array}\tag{2.3.13}
$$

记

$$
\begin{array}{l} D _ {F} = \frac {M _ {\mathrm{d0}}}{H}, \quad D _ {I} = \frac {m l _ {S}}{H}, \quad D _ {S} = - \frac {m l _ {I}}{H}, \quad D _ {I O} = \frac {m ^ {2} C _ {\mathrm{SO}}}{H}, \\ D _ {\mathrm{OS}} = - \frac {m ^ {2} C _ {I O}}{H}, \quad D _ {S I} = \frac {m ^ {2} (C _ {\mathrm{SS}} - C _ {I I})}{H}, \quad D _ {I I} = \frac {m ^ {2} C _ {S I}}{H}, \quad D _ {\mathrm{SS}} = - \frac {m ^ {2} C _ {I S}}{H} \end{array}
$$

上述 $D _ { F } , D _ { I }$ 等诸系数称为静态误差系数，具有明确的物理含义，如 $D _ { S I }$ 反映了陀螺组件不等弹性的影响，与物理参数有一一对应的关系。根据上述定义，式(2. 3. 13)可写成

$$
\omega_ {\mathrm{d}} = D _ {F} + D _ {I} F _ {I} + D _ {S} F _ {S} + D _ {I O} F _ {I} F _ {O} + D _ {O S} F _ {O} F _ {S} + D _ {S I} F _ {S} F _ {I} + D _ {I I} F _ {I} ^ {2} + D _ {S S} F _ {S} ^ {2}
$$

上式是根据理论分析获得的，实际应用中还需增补沿输出轴的一次和二次项

$$
\begin{array}{r l} \omega_ {\mathrm{d}} & = D _ {F} + D _ {I} F _ {I} + D _ {O} F _ {O} + D _ {S} F _ {S} + D _ {I O} F _ {I} F _ {O} + D _ {O S} F _ {O} F _ {S} \\ & + D _ {S I} F _ {S} F _ {I} + D _ {I I} F _ {I} ^ {2} + D _ {O O} F _ {O} ^ {2} + D _ {S S} F _ {S} ^ {2} \end{array}\tag{2.3.14}
$$

(2.3.14)即为单自由度陀螺的静态漂移数学模型。若巳测试出静态误差系数，则根据捷联加速度计的输出可得 $F _ { I } , F _ { O } , F _ { S }$ 按该式计算出 $\omega _ { \mathrm { d } }$ ，再从陀螺输出中扣除，即消除了陀螺的静态漂移.

诸静态误差系数在实验室条件下利用重力加速度基准信息即可确定出，此时 $\scriptstyle { F = g } $ 说明见7.4 节) ，所以测试中所用模型为

$$
\begin{array}{r l} \omega_ {\mathrm{d}} & = D _ {F} + D _ {I} g _ {I} + D _ {O} g _ {O} + D _ {S} g _ {S} + D _ {I O} g _ {I} g _ {O} + D _ {O S} g _ {O} g _ {S} \\ & + D _ {S I} g _ {S} g _ {I} + D _ {I I} g _ {I} ^ {2} + D _ {O O} g _ {O} ^ {2} + D _ {S S} g _ {S} ^ {2} \end{array}\tag{2.3.15}
$$

式中， $\cdot g _ { \scriptscriptstyle I } \cdot g _ { \scriptscriptstyle O }$ 和 $g _ { s }$ 分别为重力加速度沿陀螺的输入轴、输出轴及自转轴方向的分量。

## 2.3.5 单自由度陀螺仪的动态漂移误差数学模型

## 1.动态漂移误差形成机理分析

设单自由度陀螺工作在力反馈状态，基座角速度及陀螺组件在 坐标系内的惯'性张量分别为

$$
\boldsymbol {\omega} _ {i B} ^ {B} = \left[ \begin{array}{c} \omega_ {I} \\ \omega_ {O} \\ \omega_ {S} \end{array} \right], \quad [ I ] = \left[ \begin{array}{c c c} I _ {I} & - I _ {I O} & - I _ {I S} \\ - I _ {I O} & I _ {O} & - I _ {O S} \\ - I _ {I S} & - I _ {I O S} & I _ {S} \end{array} \right]
$$

为分析方便，将陀螺的测量坐标系(基座坐标系)记作 $I O S$ 根据式(2.3.1) ，陀螺组件的总角动量为

$$
\boldsymbol {H} _ {\Sigma} = [ \boldsymbol {I} ] \boldsymbol {\omega} _ {i G} + \boldsymbol {H}
$$

式中

$$
\boldsymbol {\omega} _ {i G} ^ {G} = \boldsymbol {C} _ {B} ^ {G} \boldsymbol {\omega} _ {i B} ^ {B} + \boldsymbol {\omega} _ {B G} ^ {G} = \left[ \begin{array}{c c c} 1 & 0 & - \alpha \\ 0 & 1 & 0 \\ \alpha & 0 & 1 \end{array} \right] \left[ \begin{array}{c} \omega_ {I} \\ \omega_ {O} \\ \omega_ {S} \end{array} \right] + \left[ \begin{array}{c} 0 \\ \dot {\alpha} \\ 0 \end{array} \right] = \left[ \begin{array}{c} \omega_ {I} - \alpha \omega_ {S} \\ \omega_ {O} + \dot {\alpha} \\ \omega_ {S} + \alpha \omega_ {I} \end{array} \right]
$$

$$
\boldsymbol {H} _ {\Sigma} ^ {G} = \left[ \begin{array}{c c c} I _ {I} & - I _ {I O} & - I _ {I S} \\ - I _ {I O} & I _ {O} & - I _ {O S} \\ - I _ {I S} & - I _ {O S} & I _ {S} \end{array} \right] \left[ \begin{array}{c} \omega_ {I} - \alpha \omega_ {S} \\ \omega_ {O} + \dot {\alpha} \\ \omega_ {S} + \alpha \omega_ {I} \end{array} \right] + \left[ \begin{array}{c} 0 \\ 0 \\ H \end{array} \right]
$$

$$
\left[ \begin{array}{c} H _ {\Sigma I} \\ H _ {\Sigma O} \\ H _ {\Sigma S} \end{array} \right] = \left[ \begin{array}{c} I _ {I} \omega_ {I} - I _ {I} \alpha \omega_ {S} - I _ {I O} \omega_ {O} - I _ {I O} \dot {\alpha} - I _ {I S} \omega_ {S} \\ I _ {O} \omega_ {O} + I _ {O} \dot {\alpha} - I _ {I O} \omega_ {I} - I _ {O S} \omega_ {S} \\ I _ {S} \omega_ {S} + I _ {S} \alpha \omega_ {I} - I _ {I S} \omega_ {I} - I _ {O S} \omega_ {O} - I _ {O S} \dot {\alpha} + H \end{array} \right]\tag{2.3.16}
$$

上式中，略去了关于惯性积与 的二阶小量.

根据动量矩定理

$$
\left. \frac {\mathrm{d} \boldsymbol {H} _ {\Sigma}}{\mathrm{d} t} \right| _ {i} = \boldsymbol {M}
$$

应用哥氏定理，选取 坐标系为参考坐标系，并向 坐标系内投影

$$
\left. \frac {\mathrm{d} \boldsymbol {H} _ {\Sigma}}{\mathrm{d} t} \right| _ {G} ^ {G} + \boldsymbol {\omega} _ {i G} ^ {G} \times \boldsymbol {H} _ {\Sigma} ^ {G} = \boldsymbol {M} ^ {G}
$$

RP

$$
\left[ \begin{array}{c} \dot {H} _ {\Sigma I} \\ \dot {H} _ {\Sigma O} \\ \dot {H} _ {\Sigma S} \end{array} \right] + \left[ \begin{array}{c c c} 0 & - (\omega_ {S} + \alpha \omega_ {I}) & \omega_ {O} + \dot {\alpha} \\ \omega_ {S} + \alpha \omega_ {I} & 0 & - (\omega_ {I} - \alpha \omega_ {S}) \\ - (\omega_ {O} + \dot {\alpha}) & \omega_ {I} - \alpha \omega_ {S} & 0 \end{array} \right] \left[ \begin{array}{c} H _ {\Sigma I} \\ H _ {\Sigma O} \\ H _ {\Sigma S} \end{array} \right] = \left[ \begin{array}{c} M _ {I} \\ M _ {O} \\ M _ {S} \end{array} \right]
$$

由上式和式(2. 3. 16).可得沿输出轴 的动力学方程

$$
\begin{array}{l} I _ {O} \ddot {\alpha} + I _ {O} \dot {\omega} _ {O} - I _ {I O} \dot {\omega} _ {I} - I _ {O S} \dot {\omega} _ {S} + (\omega_ {S} + \alpha \omega_ {I}) (I _ {I} \omega_ {I} - I _ {I} \alpha \omega_ {S} - I _ {I O} \omega_ {O} - I _ {I O} \dot {\alpha} - I _ {I S} \omega_ {S}) \\ - (\omega_ {I} - \alpha \omega_ {S}) (I _ {S} \omega_ {S} + I _ {S} \alpha \omega_ {I} - I _ {I S} \omega_ {I} - I _ {O S} \omega_ {O} - I _ {O S} \dot {\alpha} + H) = - D \dot {\alpha} - k \alpha + M _ {\mathrm{d}} \end{array}
$$

式中.D为液浮陀螺的阻尼系数.k 为力反馈系数. $M _ { \mathrm { d } }$ 为干扰力矩。略去关于惯性积与 $\alpha$ 的二阶小量，并忽略 $M _ { \mathrm { d } }$ 的影响，则上式可写成

$$
I _ {O} \ddot {\alpha} + D \dot {\alpha} + k \alpha = H \omega_ {I} + M _ {c}\tag{2.3.17}
$$

式中， $M _ { k } = - k \alpha$ 为陀螺的反馈力矩

$$
\begin{array}{r l} M _ {c} = & - I _ {O} \dot {\omega} _ {O} + (I _ {S} - I _ {I}) \omega_ {S} \omega_ {I} + I _ {I O} (\dot {\omega} _ {I} + \omega_ {O} \omega_ {S}) + I _ {O S} (\dot {\omega} _ {S} - \omega_ {I} \omega_ {O}) \\ & + I _ {I S} (\omega_ {S} ^ {2} - \omega_ {I} ^ {2}) + [ (I _ {S} - I _ {I}) (\omega_ {I} ^ {2} - \omega_ {S} ^ {2}) + H \omega_ {S} ] \alpha \end{array}\tag{2.3.18}
$$

当陀螺达到稳态时， $\dot { \alpha } = 0 , \stackrel { \cdot } { \alpha } = 0$ ，由式(2.3.17)可得输入角速度 $\omega _ { I }$ 的正确值

$$
\omega_ {I} = - \frac {M _ {k}}{H} - \frac {M _ {c}}{H}
$$

但实际系统中，仅 $M _ { k }$ 可通过测量再平衡回路中的电压或电流获得，而 $M _ { c }$ ，是未知的，所以实际获得的测量角速度为

$$
\dot {\omega} _ {I} = - \frac {M _ {k}}{H}
$$

引起的测量误差即动态漂移误差为

$$
\begin{array}{r l} \delta \omega_ {I} & = \dot {\omega} _ {I} - \omega_ {I} = \frac {M _ {c}}{H} \\ & = - \frac {I _ {O}}{H} \dot {\omega} _ {O} + \frac {I _ {S} - I _ {I}}{H} \omega_ {S} \omega_ {I} + \frac {I _ {I O}}{H} (\dot {\omega} _ {I} + \omega_ {O} \omega_ {S}) + \frac {I _ {O S}}{H} (\dot {\omega} _ {S} - \omega_ {I} \omega_ {O}) \\ & + \frac {I _ {I S}}{H} (\omega_ {S} ^ {2} - \omega_ {I} ^ {2}) + \frac {I _ {S} - I _ {I}}{H} (\omega_ {I} ^ {2} - \omega_ {S} ^ {2}) \alpha + \omega_ {S} \alpha \end{array}\tag{2.3.19}
$$

上式右侧第一项称角加速度误差，由沿输出轴的角加速度引起 第二项称不等惯性误差，由陀螺组件绕 轴和 轴的转动惯量不相等( $( I _ { S } { \neq } I _ { I } )$ 引起 第三至五项称惯性积误差，由陀螺框架的惯性积引起 第六项称不等惯性耦合误差，由 $I _ { S } \ne I _ { I }$ ，和 $\chi \neq 0$ 引起;第七项称交叉耦合误差，由 $\alpha \neq 0$ 使陀螺错误感测 $\omega _ { S }$ 引起。

## 2. 对动态漂移误差中各项误差的讨论

## 1)角加速度误差

角加速度误差表达式为

$$
\delta \omega_ {I 1} = - \frac {I _ {O}}{H} \dot {\omega} _ {O}
$$

式中 $\cdot \frac { I _ { o } } { H }$ 为角加速度误差辙。该项误差是由壳体相对惯性空间的旋转角加速度 $\dot { \omega } _ { \mathrm { O } }$ 所形成的牵连惯性力矩作用在陀螺上引起的。

对于单自由度液浮陀螺，角动量的典型数值范围为 $5 \times 1 0 ^ { 4 } { \sim } 3 { \times } 1 0 ^ { 6 } \mathrm { g } \cdot \mathrm { c m } ^ { 2 } / \mathrm { s } , I _ { O }$ 的典型数值范围为 $1 0 0 { \sim } 2 0 0 0 \mathrm { g } \cdot \mathrm { c m } ^ { 2 }$ ，因而角加速度误差系数的典型数值范围为 $1 4 0 { \sim } 4 1 0 ^ { \circ } / \mathrm { h } ( \mathrm { r a d } / \mathrm { s } ^ { 2 } )$ 这表明，当壳体 $\dot { \omega } _ { \mathrm { O } } = 1 \mathrm { r a d / s } ^ { 2 }$ 时，所造成的角速度测量误差超过 $\mathrm { 1 0 0 ^ { \circ } / h }$ ，可见角加速度误差是一项相当大的动态误差，在捷联式惯导系统中必须对此项误差作补偿。

## 2) 不等惯性误差

不等惯性误差的表达式为

$$
\delta \omega_ {I 2} = \frac {I _ {S} - I _ {I}}{H} \omega_ {S} \omega_ {I}
$$

式中， $\frac { I _ { S } - I _ { I } } { H }$ 二称为不等惯性误差系数.该项误差是由于陀螺组件绕输入轴的转动惯量与绕自转轴的转动惯量不相等，在角运动条件下，由 $\omega _ { S }$ 和 $\omega _ { I }$ 形成的陀螺力矩不能互相抵消引起的。

单自由度液浮陀螺的不等惯性误差系数的典型数值范围为 $4 { \sim } 3 0 ^ { \circ } / \mathrm { h } / ( \mathrm { r a d } / \mathrm { s } ) ^ { 2 }$ 这表明，当$\omega _ { S } \cdot _ { \omega _ { I } } { = } 1 ( \mathrm { r a d / s } ) ^ { 2 }$ 时，所造成的角速度测量误差将达到每小时几度以上。可见此项误差也是不可忽视的，在捷联惯导系统中也应对此项误差作补偿。

为了减小不等惯性误差，应尽量减小 $I _ { S }$ 与 $I _ { I }$ ，的差异。由于

$$
\begin{array}{r l} I _ {S} - I _ {I} & = (I _ {G S} + I _ {R S}) - (I _ {G I} + I _ {R I}) = (I _ {G S} - I _ {G I}) + (I _ {R S} - I _ {R I}) \\ & = (I _ {G S} - I _ {R I}) + (I _ {R S} - I _ {G I}) \end{array}
$$

式中， 分别表示转子和框架.由此可知有两种途径达到此目的:一是转子和框架对 轴和轴的转动惯量分别各自相等，如采用球形对称结构;二是使框架对 轴的转动惯量与转子对轴的转动惯量相等，框架对 轴的转动惯量与转子对 轴的转动惯量相等，有些陀螺仪的框架上还设置有不等惯性补偿机构，用来改变框架的转动惯量.

## 3) 框架惯性积误差

框架惯性积误差的表达式为

$$
\delta \omega_ {I 3} = \frac {I _ {I O}}{H} (\dot {\omega} _ {I} + \omega_ {O} \omega_ {S}) + \frac {I _ {O S}}{H} (\dot {\omega} _ {S} - \omega_ {I} \omega_ {O}) + \frac {I _ {I S}}{H} (\omega_ {S} ^ {2} - \omega_ {I} ^ {2})
$$

式中， $\frac { I _ { I O } } { H } , \frac { I _ { O S } } { H } , \frac { I _ { I S } } { H }$ 为框架惯性积误差系数。为了减少此项误差，在结构设计中应尽量使框架对这些轴的惯性积减小到最低限度。框架惯性积一般较小，如对于角动量为 $2 \times 1 0 ^ { 5 } \mathrm { g } \cdot \mathrm { c m } ^ { 2 } / \mathrm { s }$ 的液浮陀螺，转子的赤道转动惯量和极转动惯量分别为 $5 0 \mathrm { g } \cdot \mathrm { c m } ^ { 2 }$ 和 $8 0 \mathrm { g } \cdot \mathrm { c m } ^ { 2 }$ .陀螺组件对输出轴的转动惯量为 $2 5 0 \mathrm { g } \cdot \mathrm { c m } ^ { 2 }$ .而框架惯性积 $\begin{array} { r } { I _ { I O } = I _ { O S } = 1 . 5 \mathrm { g } \bullet \mathrm { c m } ^ { 2 } , I _ { I S } = 0 . 3 \mathrm { g } \bullet \mathrm { c m } ^ { 2 } } \end{array}$ .所以框架惯性积误差一般都较小。

## 4) 不等惯性耦合误差

不等惯性耦合误差的表达式为

$$
\delta \omega_ {I 4} = \frac {I _ {S} - I _ {I}}{H} (\omega_ {I} ^ {2} - \omega_ {S} ^ {2}) \alpha
$$

由式(2.3.17) ，当陀螺达到稳态时，略去 ，的影响

$$
\alpha \approx \frac {H}{k} \omega_ {I}\tag{2.3.20}
$$

式中.k 为工作在力反馈状态下陀螺再平衡回路的增益。所以

$$
\delta \omega_ {I 4} = \frac {I _ {S} - I _ {I}}{k} \omega_ {I} (\omega_ {I} ^ {2} - \omega_ {S} ^ {2})
$$

式中 $\frac { I _ { S } - I _ { I } } { k }$ 称为不等惯性耦合系数。为了如此项髓，一方面应尽量减小 $I _ { S }$ Ej $I _ { I }$ á9盟，另一方面应尽量提高再平衡回路的增益晶，在 足够高的情况下不等惯性耦合误差是比较小的。

## 5) 交叉耦合误差

交叉耦合误差表达式为

$$
\delta \omega_ {I 5} = \omega_ {S} \alpha
$$

该项误差产生的原因是:当输出角 $\alpha \neq 0$ 时，陀螺壳体绕自转轴的旋转角速度 $\omega _ { S }$ 的正弦分量$\omega _ { S } \sin \alpha { \approx } \omega _ { S } \alpha$ 被陀螺感测到，引起陀螺错误输出。

将式(2.3.20)代入上式，得

$$
\delta \omega_ {I 5} = \frac {H}{k} \omega_ {S} \omega_ {I}
$$

式中， $\frac { H } { k }$ ?称为交叉耦合础。该项误差与陀螺的角动量 成正比，与再平衡峭的增益成反比，所以提高品对减小此项误差显得特别重要。例如，若 $H { = } 2 . 5 \times 1 0 ^ { 5 } \mathrm { g \cdot { c m ^ { 2 } } / s }$ k=$2 . 5 { \times } 1 0 ^ { 2 } \mathrm { N } \cdot \mathrm { m } ^ { 2 } / \mathrm { r a d } \mathrm { H } \bar { \cdot }$ ，交叉耦合系数约为 $2 0 ^ { \circ } / \mathrm { h } / ( \mathrm { r a d } / \mathrm { s } ) ^ { 2 }$ ，若 增大 10 倍，则此项误差减小为$1 / 1 0$ 。因此应尽量提高 以减小交叉耦合误差。

## 3. 单自由度陀螺动态漂移误差的数学模型

将式 (2.3.20)代入式 (2. 3. 19) ，经整理后得

$$
\delta \omega_ {I} = \frac {I _ {I O}}{H} \dot {\omega} _ {I} - \frac {I _ {O}}{H} \dot {\omega} _ {O} + \frac {I _ {O S}}{H} \dot {\omega} _ {S} - \frac {I _ {O S}}{H} \omega_ {I} \omega_ {O} + \frac {I _ {I O}}{H} \omega_ {O} \omega_ {S} + \left(\frac {I _ {S} - I _ {I}}{H} + \frac {H}{k}\right) \omega_ {S} \omega_ {I}
$$

$$
- \frac {I _ {I S}}{H} \omega_ {I} ^ {2} + \frac {I _ {I S}}{H} \omega_ {S} ^ {2} + \frac {I _ {S} - I _ {I}}{k} \omega_ {I} ^ {3} - \frac {I _ {S} - I _ {I}}{k} \omega_ {I} \omega_ {S} ^ {2}
$$

记

$$
K _ {I} = \frac {I _ {I O}}{H}, \quad K _ {O} = - \frac {I _ {O}}{H}, \quad K _ {S} = \frac {I _ {O S}}{H}, \quad K _ {I O} = - \frac {I _ {O S}}{H}, \quad K _ {O S} = \frac {I _ {I O}}{H},
$$

$$
K _ {S I} = \frac {I _ {S} - I _ {I}}{H} + \frac {H}{k}, \quad K _ {I I} = - \frac {I _ {I S}}{H}, \quad K _ {S S} = \frac {I _ {I S}}{H}, \quad K _ {I I I} = \frac {I _ {S} - I _ {I}}{k}, \quad K _ {I S S} = - \frac {I _ {S} - I _ {I}}{k}
$$

则得单自由度陀螺的动态漂移数学模型

$$
\begin{array}{r l} \delta \omega_ {I} & = K _ {I} \dot {\omega} _ {I} + K _ {O} \dot {\omega} _ {O} + K _ {S} \dot {\omega} _ {S} + K _ {I O} \omega_ {I} \omega_ {O} + K _ {O S} \omega_ {O} \omega_ {S} + K _ {S I} \omega_ {S} \omega_ {I} \\ & + K _ {I I} \omega_ {I} ^ {2} + K _ {S S} \omega_ {S} ^ {2} + K _ {I I I} \omega_ {I} ^ {3} + K _ {I S S} \omega_ {I} \omega_ {S} ^ {2} \end{array}\tag{2.3.21}
$$

如果忽略误差模型中的次要误差项，仅考虑角加速度误差、不等惯性误差和交叉耦合误差，则动态漂移数学模型可简化为

$$
\delta \omega_ {I} = - \frac {I _ {O}}{H} \dot {\omega} _ {O} + \left(\frac {I _ {S} - I _ {I}}{H} + \frac {H}{k}\right) \omega_ {S} \omega_ {I}
$$

该模型仍具有很好的精度。

## 2.4 动力调谐陀螺仪

## 2.4.1 动力调谐陀螺仪的原理结构

2. 节介绍的双自由度陀螺仪采用了内装式结构，即支承转子的万向环架处于转子的外部，所以这种支承方式也称为外支承，陀螺常称为框架式陀螺。这种陀螺精度不高，影响其精度的主要因素有:①环架轴承存在干摩擦力矩，引起陀螺漂移;②自转轴做高速旋转引起轴承磨损，使转子的质量中心位置改变，产生不平衡力矩引起陀螺漂移。 1946 年英国皇家研究院提出挠性支承概念，用挠性支承的弹性变形为陀螺转子提供转动自由度，避免了没有规律的干摩擦，而弹性变形产生的弹性恢复力矩在弹性范围内服从胡克定律，是可以精确补偿的。最早挠性支承采用细颈式结构，但这种陀螺还未被广泛推广开，就被性能更优越的动力调谐陀螺取代了。

动力调谐陀螺的原理结构如图 2.4.1 所示。转子通过一对外扭杆与平衡环连接，平衡环通过一对内扭杆与电机轴连接。内外扭杆具有很大的抗弯刚度和很小的抗扭刚度，所以电机的驱

![](images/bf6966394f65d170a4dc3a1c5426b37aeb47d956946dadce5a14f6b99b8805f4.jpg)  
2.4.1 动力调谐陀螺仪的原理结构图

动力矩可通过内、外扭杆传给陀螺，而转子可绕内、外扭杆轴线转动，在赤道平面内具有两个旋转自由度，所以动力调谐陀螺是一种双自由度陀螺仪。

在陀螺壳体上安装了位置中心对称的两对信号器和两对力矩器，一轴上的电气对称轴与另一轴上的电气对称轴相互正交，是陀螺的测量基准轴，同轴上的力矩器和信号器的电气对称轴相重合。转子镶有径向充磁的磁钢，力矩器是绕制在陶瓷片骨架上圆柱面内的线圈，线圈有效工作段镶在转子的环形槽内，且与环形槽留有一定的活动间隙。当线圈通人施矩电流时，在径向磁场的作用下，转子受到沿转子自转轴方向的电磁作用力，同轴上的力矩器工作在推挽状态，产生的电磁作用力对另一轴形成力偶，从而实现对陀螺施矩。

信号器用来测量转子赤道平面相对壳体基准面(由电气对称轴确定的平面〉的偏转角，同轴上的信号器输出接成差动形式。

由于转子与电机驱动轴通过内、外扭杆连接，转子相对支承体的运动仅引起扭转角变形，而不存在干摩擦，电机高速旋转只影响电机轴承的磨损，而这种磨损不会影响转子质心的偏移，所以这种陀螺完全避免了外支承双自由度陀螺的固有缺陷，精度因此大幅提高。动力调谐陀螺的另一个优点是能实现动力调谐，在动力调谐工作状态下，平衡环扭摆产生的惯性力矩正好平衡掉扭杆的扭转弹性变形产生的弹性恢复力矩，使转子成为自由陀螺。

## 2.4.2 动力调谐原理

## 1.平衡环的扭摆运动分析

为分析方便，引人如下坐标系.

(1)基座坐标系 $O { - x _ { c } y _ { c } z _ { c } }$ 即陀螺壳体坐标系，简记 $C :$ 与陀螺壳体固联 (A 轴〉和 $x _ { c }$ $y _ { _ { C } }$ (B 轴)由信号器和力矩器的电气对称轴确定，是陀螺的测量轴，该两轴形成的平面是陀螺的测 量基准面.

(2) 电机轴坐标系 $O { - x } _ { s } y _ { s } z _ { s }$ '简记 5:与电机轴固联 轴与内扭杆轴重合，该坐标系相对 $x _ { s }$ 壳体坐标系绕 $z _ { c }$ 轴(电机驱动轴)旋转过。，其中 $\dot { \theta } { = } \varOmega , \varOmega$ 为电机的驱动角速度。

(3) 平衡环坐标系 $O \mathrm { - } x _ { r } y _ { r } z _ { r }$ '简记 $r :$ 与平衡环固联， $x _ { r }$ ，和 $y _ { i }$ 轴在平衡环赤道平面内， $\mathcal { X } _ { r }$ 轴沿内扭杆轴 $\boldsymbol { \cdot } \boldsymbol { y } _ { i }$ 轴沿外扭杆轴 $\bullet z _ { r }$ 轴沿平衡环中心线。

(4) 转子坐标系 $O \ – \ x _ { _ R } y _ { _ R } z _ { _ R }$ ，简记 R:与转子固联 $x _ { _ R }$ $y _ { _ R }$ 轴在转子赤道平面内 $z _ { \scriptscriptstyle R }$ 沿转子中心线，该坐标系参与转子自转。

(5) 转子参考坐标系 $O { - } x y z$ 简记 $G { : } z$ 轴始终与 ZR轴重合 轴位于赤道平面内，当 $z _ { _ R }$ $z _ { \scriptscriptstyle R }$ 轴与 $z _ { s }$ 轴重合时 轴与 $x _ { c }$ 轴重合 $y$ 轴与 $y _ { _ C }$ 轴重合。该坐标系不参与转子自转。 $G$ 系相对系的偏转角即为陀螺的输出角。

(6) 惯性坐标系 $O \mathrm { - } \boldsymbol { x } _ { i } \boldsymbol { y } _ { i } \boldsymbol { z } _ { i }$ '简记，:定义同第 2. 1. 节的规定。

平衡环和转子相对壳体的角位置关系如图 2.4.2 所示，各坐标系间的关系如下

$$
O - x _ {c} y _ {c} z _ {c} \xrightarrow [ \text {旋转} \theta ]{\text {绕} z _ {c} \text {轴}} O - x _ {s} y _ {s} z _ {s} \xrightarrow [ \text {旋转} \gamma ]{\text {绕} x _ {s} \text {轴}} O - x _ {r} y _ {r} z _ {r} \xrightarrow [ \text {旋转} \phi ]{\text {绕} y _ {r} \text {轴}} O - x _ {R} y _ {R} z _ {R}
$$

其中， $\theta = \Omega t \ : , \Omega$ 为电机的驱动角速度， $\gamma$ 为内扭杆的扭转角： $\phi$ 为外扭杆的扭转角，动力调谐陀螺正常工作时，γ和 $\phi$ 都控制在小角范围内，所以γ和 $\phi$ 都可看作角矢量

$$
\boldsymbol {\varphi} ^ {r} = \left[ \begin{array}{c} 0 \\ \varphi \\ 0 \end{array} \right], \quad \boldsymbol {\gamma} ^ {S} = \left[ \begin{array}{c} \gamma \\ 0 \\ 0 \end{array} \right], \quad \boldsymbol {C} _ {S} ^ {r} = \left[ \begin{array}{c c c} 1 & 0 & 0 \\ 0 & 1 & \gamma \\ 0 & - \gamma & 1 \end{array} \right]
$$

$$
\boldsymbol {\varphi} ^ {S} = \boldsymbol {C} _ {r} ^ {S} \boldsymbol {\varphi} ^ {r} = \left[ \begin{array}{c c c} 1 & 0 & 0 \\ 0 & 1 & - \gamma \\ 0 & \gamma & 1 \end{array} \right] \left[ \begin{array}{c} 0 \\ \varphi \\ 0 \end{array} \right] = \left[ \begin{array}{c} 0 \\ \varphi \\ \gamma \varphi \end{array} \right] \approx \left[ \begin{array}{c} 0 \\ \varphi \\ 0 \end{array} \right]
$$

$$
(\boldsymbol {\varphi} + \boldsymbol {\gamma}) ^ {S} = \left[ \begin{array}{c} \gamma \\ \phi \\ 0 \end{array} \right]\tag{2.4.1}
$$

![](images/2d2a4ec296602123f8ad4d9364d4b055e6924c6236a441056d47ac73524b4488.jpg)  
2.4.2 平衡环和转子相对壳体的角位置

由图 2. .2 可看出， 实质上是平衡环绕内扭杆相对电机铀坐标系的偏转角，该角变形使扭杆受弯，变形可忽略， 可视为转子绕内扭忏相对电机轴坐标系的偏转角。 $\phi$ 是转子绕外扭杆相对平衡环的偏转角，该角变形使内扭杆受弯，变形可忽略， $\phi$ <þ可视为转子绕外扭杆相对电机轴坐标系的偏转角。因此式(2.4. 1)中， $\gamma$ 和 $\phi$ 分别是转子赤道平面绕电机轴坐标系的 $x _ { s }$ 轴和 $y _ { s }$ 轴的偏转角

将该角矢量投影到陀螺壳体 标系 $O ^ { - } x _ { c } y _ { c } z _ { c } \not \equiv$

$$
(\boldsymbol {\varphi} + \boldsymbol {\gamma}) ^ {C} = \boldsymbol {C} _ {S} ^ {C} (\boldsymbol {\varphi} + \boldsymbol {\gamma}) ^ {S} = \left[ \begin{array}{c c c} \cos \theta & - \sin \theta & 0 \\ \sin \theta & \cos \theta & 0 \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c} \gamma \\ \phi \\ 0 \end{array} \right] = \left[ \begin{array}{c} \gamma \cos \theta - \phi \sin \theta \\ \gamma \sin \theta + \phi \cos \theta \\ 0 \end{array} \right]\tag{2.4.2}
$$

式中， $\gamma \mathrm { c o s } \theta - \phi \mathrm { s i n } \theta$ 和 $\gamma \mathrm { s i n } \theta + \phi \mathrm { c o s } \theta$ 分别是转子赤道平面绕壳体坐标系的 $x _ { c }$ 轴和 $y _ { c }$ 轴的偏转角，即为陀螺的输出角 和 $\beta$ o

## 2. 作用在转子上的弹性恢复力矩

由式 (2.4.2) $\scriptstyle \alpha = \gamma \cos \theta - \phi \sin \theta , \beta = \gamma \sin \theta + \phi \cos \theta$ ; 由此得扭杆变形角与输出角的关系:

$$
\gamma = \alpha \cos \theta + \beta \sin \theta\tag{2.4.3a}
$$

$$
\phi = - \alpha \sin \theta + \beta \cos \theta\tag{2.4.3b}
$$

为使分析简便，假设基座仅绕 $x _ { _ C }$ 轴相对惯性空间转过 个恒定角 $\Phi _ { x O } , \pmb { \omega } _ { i C } = \mathbf { 0 }$ ，并假设陀螺处于动力调谐状态，陀螺转子轴相对惯性空间稳定，所以 $\varPhi _ { x O }$ 实际上是基座相对转子轴转过的角度，而 $\alpha$ 是转子轴相对基座转过的角度，所以

$$
\alpha = - \Phi_ {x O}, \quad \beta = - \Phi_ {y O} = 0
$$

$$
\gamma = - \Phi_ {x O} \cos \theta\tag{2.4.4a}
$$

$$
\varphi = \Phi_ {x O} \sin \theta\tag{2.4.4b}
$$

式中， 是平衡环赤道平面相对测量基准面绕内扭杆轴的偏转角

$$
\boldsymbol {\gamma} ^ {s} = \left[ \begin{array}{c} - \Phi_ {x O} \cos \theta \\ 0 \\ 0 \end{array} \right]
$$

在基座坐标系 观察到的此偏转角为

$$
\boldsymbol {\gamma} ^ {C} = \boldsymbol {C} _ {S} ^ {C} \boldsymbol {\gamma} ^ {S} = \left[ \begin{array}{c c c} \cos \theta & - \sin \theta & 0 \\ \sin \theta & \cos \theta & 0 \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c} - \Phi_ {x O} \cos \theta \\ 0 \\ 0 \end{array} \right] = \left[ \begin{array}{c} - \frac {\Phi_ {x O}}{2} - \frac {\Phi_ {x O}}{2} \cos (2 \Omega t + 2 \varphi_ {0}) \\ - \frac {\Phi_ {x O}}{2} \sin (2 \Omega t + 2 \varphi_ {0}) \\ 0 \end{array} \right]\tag{2.4.5}
$$

式中， $\varphi _ { 0 }$ 。为内扭杆轴线与 $x _ { c }$ 在初始时刻的夹角，即 $\theta { = } \Omega t + \varphi _ { 0 }$ 。上式说明在基座上观察到平衡环绕测量轴做扭摆振动，扭摆的角频率是陀螺白转角速度的 倍。

如图 2.4.3 ，由 $\gamma ^ { \stackrel {  } { J ^ { \mu } } }$ 生的作用在平衡环上的弹性恢复力矩为

$$
\boldsymbol {M} _ {\gamma} ^ {S} = \left[ \begin{array}{c} - K \gamma \\ 0 \\ 0 \end{array} \right]
$$

该力矩通过外扭杆的抗弯功能传递作用在转子上。

由 $\phi$ 产生的作用在转子上的弹性恢复力矩为

$$
\boldsymbol {M} _ {\phi} ^ {r} = \left[ \begin{array}{c} 0 \\ - K \phi \\ 0 \end{array} \right]
$$

![](images/f1ba1bc7cc19107e9d06d0084dfcb33cd3b4c978c45ef0162c82cff9855ad013.jpg)

所以作用在转子上的弹性恢复力矩为

$$
\pmb {M} _ {\text {弹}} ^ {S} = \pmb {M} _ {\gamma} ^ {S} + \pmb {C} _ {r} ^ {S} \pmb {M} _ {\phi} ^ {r}
$$

略去 阶小量，并考虑式(2.4. 的，得

2. .3 内外扭杆的角变形和弹性恢 力矩

$$
\boldsymbol {M} _ {\text {弹}} ^ {S} = \left[ \begin{array}{c} - K \gamma \\ 0 \\ 0 \end{array} \right] + \left[ \begin{array}{c c c} 1 & 0 & 0 \\ 0 & 1 & - \gamma \\ 0 & \gamma & 1 \end{array} \right] \left[ \begin{array}{c} 0 \\ - K \phi \\ 0 \end{array} \right] = \left[ \begin{array}{c} K \Phi_ {x O} \cos \theta \\ - K \Phi_ {x O} \sin \theta \\ 0 \end{array} \right]
$$

$$
\boldsymbol {M} _ {\text {弹}} ^ {C} = \boldsymbol {C} _ {S} ^ {C} \boldsymbol {M} _ {\text {弹}} ^ {S} = \left[ \begin{array}{c c c} \cos \theta & - \sin \theta & 0 \\ \sin \theta & \cos \theta & 0 \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c} K \Phi_ {x O} \cos \theta \\ - K \Phi_ {x O} \sin \theta \\ 0 \end{array} \right] = \left[ \begin{array}{c} K \Phi_ {x O} \\ 0 \\ 0 \end{array} \right]\tag{2.4.6}
$$

3. 平衡坏的惯性力矩和陀螺力矩对转子的干扰

外扭杆抗弯刚度很大，而抗扭刚度接近 ，所以 衡环只有沿 $x _ { r }$ 轴的惯性力矩和陀螺力矩可通过外扭杆传递至转子，这些力矩对转子起干扰作用

$$
\boldsymbol {M} _ {\text {惯、陀}} ^ {r} = \left[ \begin{array}{c} - a \dot {\omega} _ {i r x} ^ {r} + (a - c) \omega_ {i r y} ^ {r} \omega_ {i r z} ^ {r} \\ 0 \\ 0 \end{array} \right]
$$

式中， $, a$ 和 $c$ 分别是平衡环的赤道转动惯 和极转动惯量。

$$
\begin{array}{r l} \boldsymbol {M} _ {\text {惯、陀}} ^ {\mathrm{C}} & = \boldsymbol {C} _ {\mathrm{S}} ^ {\mathrm{C}} \boldsymbol {C} _ {r} ^ {\mathrm{S}} \boldsymbol {M} _ {\text {惯、陀}} ^ {r} = \left[ \begin{array}{c c c} \cos \theta & - \sin \theta & 0 \\ \sin \theta & \cos \theta & 0 \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c c c} 1 & 0 & 0 \\ 0 & 1 & - \gamma \\ 0 & \gamma & 1 \end{array} \right] \left[ \begin{array}{c} - a \dot {\omega} _ {i r x} ^ {r} + (a - c) \omega_ {i r y} ^ {r} \omega_ {i r z} ^ {r} \\ 0 \\ 0 \end{array} \right] \\ & = \left[ \begin{array}{c} - a \dot {\omega} _ {i r x} ^ {r} \cos \theta + (a - c) \omega_ {i r y} ^ {r} \omega_ {i r z} ^ {r} \cos \theta \\ - a \dot {\omega} _ {i r x} ^ {r} \sin \theta + (a - c) \omega_ {i r y} ^ {r} \omega_ {i r z} ^ {r} \sin \theta \\ 0 \end{array} \right] \\ & \boldsymbol {\omega} _ {i r} = \boldsymbol {\omega} _ {i C} + \boldsymbol {\omega} _ {C S} + \boldsymbol {\omega} _ {S r} \end{array}\tag{2.4.7}
$$

忽略基座角运动的影响

$$
\begin{array}{c} \boldsymbol {\omega} _ {i r} ^ {r} = \boldsymbol {C} _ {S} ^ {r} \boldsymbol {\omega} _ {C S} ^ {S} + \boldsymbol {\omega} _ {S r} ^ {r} \\ \left[ \begin{array}{l} \boldsymbol {\omega} _ {i r x} ^ {r} \\ \boldsymbol {\omega} _ {i r y} ^ {r} \\ \boldsymbol {\omega} _ {i r z} ^ {r} \end{array} \right] = \left[ \begin{array}{c c c} 1 & 0 & 0 \\ 0 & 1 & \gamma \\ 0 & - \gamma & 1 \end{array} \right] \left[ \begin{array}{l} 0 \\ 0 \\ \Omega \end{array} \right] + \left[ \begin{array}{l} \dot {\gamma} \\ 0 \\ 0 \end{array} \right] = \left[ \begin{array}{l} \dot {\gamma} \\ \gamma \Omega \\ \Omega \end{array} \right] \end{array}
$$

式中 $\theta { = } \Omega t { + } \varphi _ { 0 }$ 0

根据式 (2.4.4a)

$$
\begin{array}{c} \dot {\gamma} = \Phi_ {x 0} \Omega \sin \theta \\ \omega_ {i r x} ^ {r} = \Phi_ {x 0} \Omega \sin \theta , \quad \omega_ {i r y} ^ {r} = - \Phi_ {x 0} \Omega \cos \theta , \quad \omega_ {i r z} ^ {r} = \Omega \\ \dot {\omega} _ {i r x} ^ {r} = \Phi_ {x 0} \Omega^ {2} \cos \theta \end{array}
$$

上述诸式代入式(2.4.7) ，得

$$
\begin{array}{r l} \boldsymbol {M} _ {\text {惯,陀}} ^ {C} & = \left[ \begin{array}{c} - a \Phi_ {x 0} \Omega^ {2} \cos^ {2} \theta - (a - c) \Phi_ {x 0} \Omega^ {2} \cos^ {2} \theta \\ - a \Phi_ {x 0} \Omega^ {2} \sin \theta \cos \theta - (a - c) \Phi_ {x 0} \Omega^ {2} \sin \theta \cos \theta \\ 0 \end{array} \right] \\ & = \left[ \begin{array}{c} - \left(a - \frac {c}{2}\right) \Phi_ {x 0} \Omega^ {2} - \left(a - \frac {c}{2}\right) \Phi_ {x 0} \Omega^ {2} \cos 2 \theta \\ - \left(a - \frac {c}{2}\right) \Phi_ {x 0} \Omega^ {2} \sin 2 \theta \\ 0 \end{array} \right] \end{array}\tag{2.4.8}
$$

根据式(2.4. 6)和式(2.4.8)得，作用在转子的干扰力矩为

$$
\boldsymbol {M} _ {\mathrm{d}} ^ {\mathrm{C}} = \left[ \begin{array}{c} K \Phi_ {x O} - \left(a - \frac {c}{2}\right) \Phi_ {x O} \Omega^ {2} - \left(a - \frac {c}{2}\right) \Phi_ {x O} \Omega^ {2} \cos 2 \theta \\ - \left(a - \frac {c}{2}\right) \Phi_ {x O} \Omega^ {2} \sin 2 \theta \\ 0 \end{array} \right]
$$

式中 $\theta { = } \Omega t { + } \varphi _ { 0 }$ 上式说明，作用在转子上的干扰力矩有两类=一类是交变干扰力矩，角频率是陀螺自转角速度的 倍，只引起转子的高频振荡;另一类是常值干扰力矩，作用在 $x _ { _ C }$ 轴上，具体为

$$
\overline {{{M}}} _ {\mathrm{d} x} ^ {C} = \left[ K - \left(a - \frac {c}{2}\right) \Omega^ {2} \right] \Phi_ {x O}
$$

常值干扰力矩将引起严重的陀螺漂移，影响远比交变干扰力矩严重。但从上式可看出，若满足

$$
K - \left(a - \frac {c}{2}\right) \Omega^ {2} = 0\tag{2.4.9}
$$

则常值干扰力矩为零，此时称陀螺工作在动力调谐状态。式(2.4.9)即为动力调谐条件， 为陀螺的调谐角速度。

从上述分析可看出，动力调谐实质上是平衡环的惯性力矩和陀螺力矩与弹性恢复力矩达到了 平衡。动力调谐的实现可通过调整平衡环的赤道转动惯量和极转动惯量来实现，常用办法是在平 衡环上安装一个质量杆，修磨质量杆达到精翩翩力调谐的剿，但在设计中必须保证 $a { > } \frac { c } { 2 }$

## 2.4.3 动力调谐陀螺的动力学方程

## 1.动力调谐陀螺的欧拉方程

设基座(陀螺壳体)相对惯性空间的角速度和转子的自转角速度分别为

$$
\begin{array}{l} \boldsymbol {\omega} _ {i C} ^ {C} = [ \omega_ {x} \quad \omega_ {y} \quad \omega_ {z} ] ^ {\mathrm{T}} \\ \boldsymbol {\omega} _ {G R} ^ {G} = [ 0 \quad 0 \quad \Omega ] ^ {\mathrm{T}} \end{array}
$$

转子的惯性张量为[ $[ J _ { R } ] = \operatorname { d i a g } [ A \quad A \quad C ]$ 其中 分别为转子的赤道转动惯量和极转动惯量。陀螺绕 $x _ { c }$ 轴的输出角为 $\alpha$ ，绕 $y _ { _ C }$ 轴的输出角为 $\beta$ $\alpha$ 和 $\beta$ 均为小量，则

$$
\boldsymbol {\omega} _ {i G} ^ {G} = \boldsymbol {C} _ {C} ^ {G} \boldsymbol {\omega} _ {i C} ^ {C} + \boldsymbol {\omega} _ {C G} ^ {G} = \left[ \begin{array}{c c c} 1 & 0 & - \beta \\ 0 & 1 & \alpha \\ \beta & - \alpha & 1 \end{array} \right] \left[ \begin{array}{l} \omega_ {x} \\ \omega_ {y} \\ \omega_ {z} \end{array} \right] + \left[ \begin{array}{l} \dot {\alpha} \\ \dot {\beta} \\ 0 \end{array} \right] = \left[ \begin{array}{l} \omega_ {x} + \dot {\alpha} - \beta \omega_ {z} \\ \omega_ {y} + \dot {\beta} + \alpha \omega_ {z} \\ \omega_ {z} + \beta \omega_ {x} - \alpha \omega_ {y} \end{array} \right]\tag{2.4.10}
$$

$$
\boldsymbol {\omega} _ {i R} ^ {G} = \boldsymbol {\omega} _ {i G} ^ {G} + \boldsymbol {\omega} _ {G R} ^ {G} = \left[ \begin{array}{c} \omega_ {x} + \dot {\alpha} - \beta \omega_ {z} \\ \omega_ {y} + \dot {\beta} + \alpha \omega_ {z} \\ \omega_ {z} + \beta \omega_ {x} - \alpha \omega_ {y} + \Omega \end{array} \right]
$$

转子的角动量为

$$
\begin{array}{r l} \boldsymbol {H} _ {R} ^ {G} & = [ J _ {R} ] \boldsymbol {\omega} _ {i R} ^ {G} = \left[ \begin{array}{c c c} A & 0 & 0 \\ 0 & A & 0 \\ 0 & 0 & C \end{array} \right] \left[ \begin{array}{c} \omega_ {x} + \dot {\alpha} - \beta \omega_ {z} \\ \omega_ {y} + \dot {\beta} + \alpha \omega_ {z} \\ \omega_ {z} + \beta \omega_ {x} - \alpha \omega_ {y} + \Omega \end{array} \right] \\ & = \left[ \begin{array}{c} A (\omega_ {x} + \dot {\alpha} - \beta \omega_ {z}) \\ A (\omega_ {y} + \dot {\beta} + \alpha \omega_ {z}) \\ C (\omega_ {z} + \beta \omega_ {x} - \alpha \omega_ {y} + \Omega) \end{array} \right] \approx \left[ \begin{array}{c} A (\omega_ {x} + \dot {\alpha} - \beta \omega_ {z}) \\ A (\omega_ {y} + \dot {\beta} + \alpha \omega_ {z}) \\ H \end{array} \right] \end{array}\tag{2.4.11}
$$

$$
\begin{array}{r l} \boldsymbol {H} _ {R} ^ {C} & = \boldsymbol {C} _ {G} ^ {C} \boldsymbol {H} _ {R} ^ {G} = \left[ \begin{array}{c c c} 1 & 0 & \beta \\ 0 & 1 & - \alpha \\ - \beta & \alpha & 1 \end{array} \right] \left[ \begin{array}{c} A (\omega_ {x} + \dot {\alpha} - \beta \omega_ {z}) \\ A (\omega_ {y} + \dot {\beta} + \alpha \omega_ {z}) \\ H \end{array} \right] \\ & = \left[ \begin{array}{c} A (\omega_ {x} + \dot {\alpha}) + \beta (H - A \omega_ {z}) \\ A (\omega_ {y} + \dot {\beta}) - \alpha (H - A \omega_ {z}) \\ - \beta A (\omega_ {x} + \dot {\alpha} - \beta \omega_ {z}) + \alpha A (\omega_ {y} + \dot {\beta} + \alpha \omega_ {z}) + H \end{array} \right] \end{array}
$$

式中 $H { = } C \Omega$ 是转子的角动量。

由于 $H \gg A \omega _ { z } , H \gg - \beta A \left( \omega _ { x } + \dot { \alpha } - \beta \omega _ { z } \right) + \alpha A \left( \omega _ { y } + \dot { \beta } + \alpha \omega _ { z } \right)$ ，所以

$$
\boldsymbol {H} _ {R} ^ {C} \approx \left[ \begin{array}{c} A (\omega_ {x} + \dot {\alpha}) + \beta H \\ A (\omega_ {y} + \dot {\beta}) - \alpha H \\ H \end{array} \right]
$$

根据动量矩定理和哥氏定理

$$
\left. \frac {\mathrm{d} \boldsymbol {H} _ {R}}{\mathrm{d} t} \right| _ {i} = \boldsymbol {M} + \boldsymbol {M} _ {\mathrm{d}}, \quad \left. \frac {\mathrm{d} \boldsymbol {H} _ {R}}{\mathrm{d} t} \right| _ {C} + \boldsymbol {\omega} _ {i C} \times \boldsymbol {H} _ {R} = \boldsymbol {M} + \boldsymbol {M} _ {d}
$$

式中， $M _ { \mathrm { d } }$ 分别为作用在转子上的控制力矩和干扰力矩。

上式向基座坐标系 投影，并写成数学矢量形式

$$
\frac {\mathrm{d}}{\mathrm{d} t} \left[ \begin{array}{c} A (\omega_ {x} + \dot {\alpha}) + \beta H \\ A (\omega_ {y} + \dot {\beta}) - \alpha H \\ H \end{array} \right] + \left[ \begin{array}{c c c} 0 & - \omega_ {z} & \omega_ {y} \\ \omega_ {z} & 0 & - \omega_ {x} \\ - \omega_ {y} & \omega_ {x} & 0 \end{array} \right] \times \left[ \begin{array}{c} A (\omega_ {x} + \dot {\alpha}) + \beta H \\ A (\omega_ {y} + \dot {\beta}) - \alpha H \\ H \end{array} \right] = \left[ \begin{array}{c} M _ {x} ^ {C} + M _ {\mathrm{d} x} ^ {C} \\ M _ {y} ^ {C} + M _ {\mathrm{d} y} ^ {C} \\ M _ {z} ^ {C} + M _ {\mathrm{d} z} ^ {C} \end{array} \right]
$$

由上式得

$$
\begin{array}{l} A (\dot {\omega} _ {x} + \ddot {\alpha}) + \dot {\beta} H - A \omega_ {z} (\omega_ {y} + \dot {\beta}) + \alpha \omega_ {z} H + \omega_ {y} H = M _ {x} ^ {C} + M _ {\mathrm{d} x} ^ {C} \\ A (\dot {\omega} _ {y} + \ddot {\beta}) - \dot {\alpha} H + A \omega_ {z} (\omega_ {x} + \dot {\alpha}) + \beta \omega_ {z} H - \omega_ {x} H = M _ {y} ^ {C} + M _ {\mathrm{d} y} ^ {C} \end{array}
$$

由于 $\alpha$ 和 $\beta$ 都是小量，所以有

$$
\begin{array}{l} - \alpha \omega_ {z} H + \omega_ {y} H = (\omega_ {y} - \alpha \omega_ {z}) H \approx \omega_ {y} H \\ \beta \omega_ {z} H - \omega_ {x} H = - (\omega_ {x} - \beta \omega_ {z}) H \approx - \omega_ {x} H \end{array}
$$

略去陀螺力矩 $A \omega _ { z } ( \omega _ { y } + \dot { \beta } )$ )及 $A \omega _ { z } ( \omega _ { x } + \dot { \alpha } )$ 丰)的影响，上述两式变成

$$
A (\dot {\omega} _ {x} + \ddot {\alpha}) + \dot {\beta} H + \omega_ {y} H = M _ {x} ^ {C} + M _ {\mathrm{d} x} ^ {C}\tag{2.4.12a}
$$

$$
A \left(\dot {\omega} _ {y} + \ddot {\beta}\right) - \dot {\alpha} H - \omega_ {x} H = M _ {y} ^ {C} + M _ {d y} ^ {C}\tag{2.4.12b}
$$

## 2. 转子上的外作用力矩分析

转子上的外作用力矩有两类 平衡环传递给转子的力矩，转子在气体介质中旋转引起的风阻力矩及扭杆弹性变形引起的内阻尼力矩。

1)平衡环传递给转子的惯性力矩和陀螺力矩

平衡环角速度由如下矢量和确定

$$
\begin{array}{l} \boldsymbol {\omega} _ {i r} ^ {r} = \boldsymbol {C} _ {S} ^ {r} \boldsymbol {C} _ {C} ^ {S} \boldsymbol {\omega} _ {i C} ^ {C} + \boldsymbol {C} _ {S} ^ {r} \boldsymbol {\omega} _ {C S} ^ {S} + \boldsymbol {\omega} _ {S r} ^ {r} \\ = \left[ \begin{array}{c c c} 1 & 0 & 0 \\ 0 & 1 & \gamma \\ 0 & - \gamma & 1 \end{array} \right] \left[ \begin{array}{c c c} \cos \theta & \sin \theta & 0 \\ - \sin \theta & \cos \theta & 0 \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{l} \omega_ {x} \\ \omega_ {y} \\ \omega_ {z} \end{array} \right] + \left[ \begin{array}{c c c} 1 & 0 & 0 \\ 0 & 1 & \gamma \\ 0 & - \gamma & 1 \end{array} \right] \left[ \begin{array}{l} 0 \\ 0 \\ \Omega \end{array} \right] + \left[ \begin{array}{l} \dot {\gamma} \\ 0 \\ 0 \end{array} \right] \\ = \left[ \begin{array}{c} \dot {\gamma} + \omega_ {x} \cos \theta + \omega_ {y} \sin \theta \\ \gamma \Omega - \omega_ {x} \sin \theta + \omega_ {y} \cos \theta + \gamma \omega_ {z} \\ \Omega + \gamma \omega_ {x} \sin \theta - \gamma \omega_ {y} \cos \theta + \omega_ {z} \end{array} \right] \end{array}
$$

(2.4.3) 代入上式得

$$
\begin{array}{l} \omega_ {i r x} ^ {r} = \dot {\alpha} \cos \theta - \alpha \Omega \sin \theta + \dot {\beta} \sin \theta + \beta \Omega \cos \theta + \omega_ {x} \cos \theta + \omega_ {y} \sin \theta \\ \omega_ {i r y} ^ {r} = (\alpha \cos \theta + \beta \sin \theta) (\Omega + \omega_ {z}) - \omega_ {x} \sin \theta + \omega_ {y} \cos \theta \\ \omega_ {i r z} ^ {r} = \Omega + (\alpha \cos \theta + \beta \sin \theta) (\omega_ {x} \sin \theta - \omega_ {y} \cos \theta) + \omega_ {z} \end{array}\tag{2.4.13a}
$$

由于陀螺的自转角速度 $\varOmega$ 远大于基座角速度 $\omega _ { x } \circ \omega _ { y }$ 和 $\omega _ { z }$ ，所以

$$
\omega_ {i r y} ^ {r} \approx \alpha \Omega \cos \theta + \beta \Omega \sin \theta\tag{2.4.13b}
$$

$$
\omega_ {i r z} ^ {r} \approx \Omega\tag{2.4.13c}
$$

又由式 (2.4. 13a) ，得

$$
\begin{array}{r l} \dot {\omega} _ {i r x} ^ {r} = & \ddot {\alpha} \cos \theta - 2 \dot {\alpha} \Omega \sin \theta - \alpha \Omega^ {2} \cos \theta + \ddot {\beta} \sin \theta + 2 \dot {\beta} \Omega \cos \theta - \beta \Omega^ {2} \sin \theta \\ & + \dot {\omega} _ {x} \cos \theta - \omega_ {x} \Omega \sin \theta + \dot {\omega} _ {y} \sin \theta + \omega_ {y} \Omega \cos \theta \end{array}\tag{2.4.14}
$$

根据图 2.4.4 ，只有沿 $\mathcal { X } _ { r }$ 轴的平衡环惯性力矩和陀螺力矩可通过外扭杆的抗弯功能传递给转子，所以

$$
\begin{array}{r l} M _ {I G x} ^ {r} & = - a \dot {\omega} _ {i r x} ^ {r} - c \omega_ {i r z} ^ {r} \omega_ {i r y} ^ {r} + a \omega_ {i r y} ^ {r} \omega_ {i r z} ^ {r} \\ & = - a [ (\dot {\alpha} - \alpha \Omega^ {2} + 2 \dot {\beta} \Omega + \dot {\omega} _ {x} + \omega_ {y} \Omega) \cos \theta + (\dot {\beta} - \beta \Omega^ {2} - 2 \dot {\alpha} \Omega \\ & + \dot {\omega} _ {y} - \omega_ {x} \Omega) \sin \theta ] + (a - c) \alpha \Omega^ {2} \cos \theta + (a - c) \beta \Omega^ {2} \sin \theta \end{array}
$$

$$
\begin{array}{c} M _ {I G y} ^ {r} = 0 \\ M _ {I G z} ^ {r} = 0 \end{array}
$$

$$
\boldsymbol {M} _ {i G} ^ {C} = \boldsymbol {C} _ {S} ^ {C} \boldsymbol {C} _ {r} ^ {S} \boldsymbol {M} _ {i G} ^ {r} = \left[ \begin{array}{c c c} \cos \theta & - \sin \theta & 0 \\ \sin \theta & \cos \theta & 0 \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c c c} 1 & 0 & 0 \\ 0 & 1 & - \gamma \\ 0 & \gamma & 1 \end{array} \right] \left[ \begin{array}{c} M _ {I G x} ^ {r} \\ 0 \\ 0 \end{array} \right] = \left[ \begin{array}{c} M _ {I G x} ^ {r} \cos \theta \\ M _ {I G x} ^ {r} \sin \theta \\ 0 \end{array} \right]
$$

![](images/d839862b63bda87efdb91018e2d97520af77d9d867daea7983e714babc5ae866.jpg)  
2.4.4 平衡环的惯性力矩和陀螺力矩

$$
\begin{array}{l} M _ {I G x} ^ {C} = M _ {I G x} ^ {r} \cos \theta \\ = - a \left[ \frac {1 + \cos 2 \theta}{2} \left(\dot {\alpha} - \alpha \Omega^ {2} + 2 \dot {\beta} \Omega + \dot {\omega} _ {x} + \omega_ {y} \Omega\right) + \frac {1}{2} \sin 2 \theta \left(\dot {\beta} - \beta \Omega^ {2} \right. \right. \\ \left. - 2 \dot {\alpha} \Omega + \dot {\omega} _ {y} - \omega_ {x} \Omega\right) \bigg ] + \frac {1 + \cos 2 \theta}{2} (a - c) \alpha \Omega^ {2} + \frac {1}{2} (a - c) \beta \Omega^ {2} \sin 2 \theta \\ = - \frac {a}{2} \left(\dot {\alpha} + 2 \dot {\beta} \Omega + \dot {\omega} _ {x} + \omega_ {y} \Omega\right) + \left(a - \frac {c}{2}\right) \alpha \Omega^ {2} \end{array}
$$

$$
\begin{array}{l} + \left[ - \frac {a}{2} (\dot {\alpha} + 2 \dot {\beta} \Omega + \dot {\omega} _ {x} + \omega_ {y} \Omega) + \left(a - \frac {c}{2}\right) \alpha \Omega^ {2} \right] \cos 2 \theta \\ + \left[ - \frac {a}{2} (\dot {\beta} - 2 \dot {\alpha} \Omega + \dot {\omega} _ {y} - \omega_ {x} \Omega) + \left(a - \frac {c}{2}\right) \beta \Omega^ {2} \right] \sin 2 \theta \end{array}
$$

$$
\begin{array}{r l} M _ {I G y} ^ {C} & = M _ {I G x} ^ {x} \sin \theta \\ & = - \frac {a}{2} (\dot {\alpha} - \alpha \Omega^ {2} + 2 \dot {\beta} \Omega + \dot {\omega} _ {x} + \omega_ {y} \Omega) \sin 2 \theta - \frac {a (1 - \cos 2 \theta)}{2} (\dot {\beta} - \beta \Omega^ {2} - 2 \dot {\alpha} \Omega + \dot {\omega} _ {y} - \omega_ {x} \Omega) \\ & + \frac {a - c}{2} \alpha \Omega^ {2} \sin 2 \theta + \frac {1 - \cos 2 \theta}{2} (a - c) \beta \Omega^ {2} \\ & = - \frac {a}{2} (\dot {\beta} - 2 \dot {\alpha} \Omega + \dot {\omega} _ {y} - \omega_ {x} \Omega) + (a - \frac {c}{2}) \beta \Omega^ {2} \\ & + [ - \frac {a}{2} (\dot {\alpha} + 2 \dot {\beta} \Omega + \dot {\omega} _ {x} + \omega_ {y} \Omega) + (a - \frac {c}{2}) \alpha \Omega^ {2} ] \sin 2 \theta \\ & + [ \frac {a}{2} (\dot {\beta} - 2 \dot {\alpha} \Omega + \dot {\omega} _ {y} - \omega_ {x} \Omega) - (a - \frac {c}{2}) \beta \Omega^ {2} ] \cos 2 \theta \end{array}
$$

M"""二。

$M _ { I G x } ^ { C } \mathcal { M } _ { I G y } ^ { C }$ 、且也的直流干扰力矩为

$$
\overline {{M}} _ {I G x} ^ {C} = - \frac {a}{2} (\ddot {\alpha} + 2 \dot {\beta} \Omega + \dot {\omega} _ {x} + \omega_ {y} \Omega) + \left(a - \frac {c}{2}\right) _ {\alpha} \Omega^ {2}\tag{2.4.15a}
$$

$$
\overline {{{M}}} _ {I G y} ^ {C} = - \frac {a}{2} (\ddot {\beta} - 2 \dot {\alpha} \Omega + \dot {\omega} _ {y} - \omega_ {x} \Omega) + \left(a - \frac {c}{2}\right) \beta \Omega^ {2}\tag{2.4.15b}
$$

$$
\overline {{M}} _ {I G z} ^ {C} = 0\tag{2.4.15c}
$$

除直流干扰力矩外，还有关于 $\varOmega$ 的倍频交变干扰力矩，这些交变干扰力矩使陀螺输出在零附近做微幅振荡，对陀螺输出影响并不严重，必要时可对陀螺输出作低通滤被以消除之。

2) 作用在转子上的弹性恢复力矩

设陀螺的输出角为 $\alpha$ 和 $\beta$ ，则根据式(2.4.3) ，内外扭杆的扭转变形角为

$$
\gamma = \alpha \cos \theta + \beta \sin \theta , \quad \phi = - \alpha \sin \theta + \beta \cos \theta
$$

所产生的弹性恢复力矩为(参阅图 2.4.3)

$$
\boldsymbol {M} _ {E} ^ {\mathrm{S}} = \left[ \begin{array}{c} - K \gamma \\ - K \phi \\ 0 \end{array} \right]
$$

式中 $- K \gamma$ 通过外扭杆的抗弯功能传递给转子 $- K \phi$ 直接作用在转子上。

$$
\begin{array}{r l} \boldsymbol {M} _ {E} ^ {C} & = \boldsymbol {C} _ {S} ^ {C} \boldsymbol {M} _ {E} ^ {S} = \left[ \begin{array}{c c c} \cos \theta & - \sin \theta & 0 \\ \sin \theta & \cos \theta & 0 \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c} - K \gamma \\ - K \phi \\ 0 \end{array} \right] = - K \left[ \begin{array}{c} \gamma \cos \theta - \phi \sin \theta \\ \gamma \sin \theta + \phi \cos \theta \\ 0 \end{array} \right] \\ & = - K \left[ \begin{array}{c} \alpha \cos^ {2} \theta + \beta \sin \theta \cos \theta + \alpha \sin^ {2} \theta - \beta \sin \theta \cos \theta \\ \beta \cos^ {2} \theta + \alpha \sin \theta \cos \theta + \beta \sin^ {2} \theta - \alpha \sin \theta \cos \theta \\ 0 \end{array} \right] = \left[ \begin{array}{c} - K \alpha \\ - K \beta \\ 0 \end{array} \right] \end{array}\tag{2.4.16}
$$

3) 作用在转子上的阻尼力矩

陀螺转子工作在气体介质中，旋转引起的风阻力矩沿转子自转轴的反向，在转子参考坐标系

中的数学矢量为

$$
\boldsymbol {M} _ {D W} ^ {G} = \left[ \begin{array}{c} 0 \\ 0 \\ - D \Omega \end{array} \right]
$$

式中，D为风阻系数。

由于转子赤道平面与测量基准面的夹角即为陀螺输出角 $\alpha$ 和 $\beta$ ，且 $\alpha$ 和 $\beta$ 都是小角，所以

$$
\begin{array}{c} \boldsymbol {C} _ {C} ^ {G} = \left[ \begin{array}{c c c} 1 & 0 & - \beta \\ 0 & 1 & \alpha \\ \beta & - \alpha & 1 \end{array} \right] \\ \boldsymbol {M} _ {D W} ^ {C} = \boldsymbol {C} _ {G} ^ {C} \boldsymbol {M} _ {D W} ^ {G} = \left[ \begin{array}{c c c} 1 & 0 & \beta \\ 0 & 1 & - \alpha \\ - \beta & \alpha & 1 \end{array} \right] \left[ \begin{array}{c} 0 \\ 0 \\ - D \Omega \end{array} \right] = \left[ \begin{array}{c} - \beta D \Omega \\ \alpha D \Omega \\ - D \Omega \end{array} \right] \end{array}\tag{2.4.17}
$$

内外扭杆的扭转角变形会引起弹性材料的内阻尼力矩 $M _ { D V }$ ，设材料的内阻尼系数为δ，则

$$
\boldsymbol {M} _ {D V} ^ {R} = \boldsymbol {C} _ {r} ^ {R} \left[ \begin{array}{c} - \dot {\gamma} \delta \\ 0 \\ 0 \end{array} \right] + \left[ \begin{array}{c} 0 \\ - \dot {\phi} \delta \\ 0 \end{array} \right] = \left[ \begin{array}{c c c} 1 & 0 & - \phi \\ 0 & 1 & 0 \\ \phi & 0 & 1 \end{array} \right] \left[ \begin{array}{c} - \dot {\gamma} \delta \\ 0 \\ 0 \end{array} \right] + \left[ \begin{array}{c} 0 \\ - \dot {\phi} \delta \\ 0 \end{array} \right] = \left[ \begin{array}{c} - \dot {\gamma} \delta \\ - \dot {\phi} \delta \\ 0 \end{array} \right]\tag{2.4.18}
$$

上式中略去了 $\phi$ 和 形成的工阶小量。

$$
\begin{array}{l} \boldsymbol {M} _ {D V} ^ {C} = \boldsymbol {C} _ {R} ^ {C} \boldsymbol {M} _ {D V} ^ {R} = \boldsymbol {C} _ {G} ^ {C} \boldsymbol {C} _ {R} ^ {G} \boldsymbol {M} _ {D V} ^ {R} \\ = - \delta \left[ \begin{array}{c c c} 1 & 0 & \beta \\ 0 & 1 & - \alpha \\ - \beta & \alpha & 1 \end{array} \right] \left[ \begin{array}{c c c} \cos \theta & - \sin \theta & 0 \\ \sin \theta & \cos \theta & 0 \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c} \dot {\gamma} \\ \dot {\phi} \\ 0 \end{array} \right] \\ = - \delta \left[ \begin{array}{c c c} \cos \theta & - \sin \theta & \beta \\ \sin \theta & \cos \theta & - \alpha \\ \alpha \sin \theta - \beta \cos \theta & \alpha \cos \theta + \beta \sin \theta & 1 \end{array} \right] \left[ \begin{array}{c} \dot {\gamma} \\ \dot {\phi} \\ 0 _ {-} ^ {-} \end{array} \right] \\ = - \delta \left[ \begin{array}{c} \dot {\gamma} \cos \theta - \dot {\phi} \sin \theta \\ \dot {\gamma} \sin \theta + \dot {\phi} \cos \theta \\ \dot {\gamma} (\alpha \sin \theta - \beta \cos \theta) + \dot {\phi} (\alpha \cos \theta + \beta \sin \theta) \end{array} \right] \end{array}
$$

由式 (2.4. 3)

$$
\begin{array}{r l} \dot {\gamma} & = \dot {\alpha} \cos \theta - \alpha \Omega \sin \theta + \dot {\beta} \sin \theta + \beta \Omega \cos \theta \\ \dot {\phi} & = - \dot {\alpha} \sin \theta - \alpha \Omega \cos \theta + \dot {\beta} \cos \theta - \beta \Omega \sin \theta \\ M _ {D V x} ^ {C} & = - \delta (\dot {\alpha} \cos^ {2} \theta - \alpha \Omega \sin \theta \cos \theta + \dot {\beta} \sin \theta \cos \theta + \beta \Omega \cos^ {2} \theta \\ & + \dot {\alpha} \sin^ {2} \theta + \alpha \Omega \sin \theta \cos \theta - \dot {\beta} \sin \theta \cos \theta + \beta \Omega \sin^ {2} \theta) \\ & = - \delta (\dot {\alpha} + \beta \Omega) \\ M _ {D V y} ^ {C} & = - \delta (\dot {\alpha} \cos \theta \sin \theta - \alpha \Omega \sin^ {2} \theta + \dot {\beta} \sin^ {2} \theta + \beta \Omega \cos \theta \sin \theta \\ & - \dot {\alpha} \sin \theta \cos \theta - \alpha \Omega \cos^ {2} \theta + \dot {\beta} \cos^ {2} \theta - \beta \Omega \sin \theta \cos \theta) \\ & = - \delta (\dot {\beta} - \alpha \Omega) \end{array}\tag{2.4.19a}
$$

(2. 4. 19b)

由于 $\alpha \cdot \beta$ 都是小量，略去二阶小量后可得

$$
M _ {D V z} ^ {C} = 0\tag{2.4.19c}
$$

## 3. 动力调谐陀螺的动力学方程

由式 (2.4. 15) 、式 (2. 4. 16) 、式 (2.4.17) 和式 (2.4.19) 得，作用在转子上沿 $x _ { _ C }$ 和 $y _ { _ { C } }$ 轴的非交变干扰力矩为

$$
M _ {\mathrm{dx}} ^ {\mathrm{C}} = - \frac {a}{2} (\ddot {\alpha} + 2 \dot {\beta} \Omega + \dot {\omega} _ {x} + \omega_ {y} \Omega) + (a - \frac {c}{2}) _ {\alpha} \Omega^ {2} - K _ {\alpha} - \delta (\dot {\alpha} + \beta \Omega) - \beta D \Omega\tag{2.4.20a}
$$

$$
M _ {\mathrm{dy}} ^ {C} = - \frac {a}{2} (\ddot {\beta} - 2 \dot {\alpha} \Omega + \dot {\omega} _ {y} - \omega_ {x} \Omega) + (a - \frac {c}{2}) \beta \Omega^ {2} - K \beta - \delta (\dot {\beta} - \alpha \Omega) + \alpha D \Omega\tag{2.4.20b}
$$

(2.4.20)代入式(2. 4. 12) ，经整理得动力调谐陀螺的动力学方程

$$
\begin{array}{l} \left(A + \frac {a}{2}\right) \ddot {\alpha} + \delta \dot {\alpha} + \left[ K - \left(a - \frac {c}{2}\right) \Omega^ {2} \right] _ {\alpha} + (H + a \Omega) \dot {\beta} + \Omega (\delta + D) \beta \\ = M _ {x} ^ {C} - \left(A + \frac {a}{2}\right) \dot {\omega} _ {x} - \left(H + \frac {a}{2} \Omega\right) \omega_ {y} \\ \left(A + \frac {a}{2}\right) \ddot {\beta} + \delta \dot {\beta} + \left[ K - \left(a - \frac {c}{2}\right) \Omega^ {2} \right] _ {\beta} - (H + a \Omega) \dot {\alpha} + \Omega (\delta + D) _ {\alpha} \\ = M _ {y} ^ {C} - \left(A + \frac {a}{2}\right) \dot {\omega} _ {y} + \left(H + \frac {a}{2} \Omega\right) \omega_ {x} \end{array}\tag{2.4.21a}
$$

(2. 4. 21b)

对式(2.4.2 1)作进一步简化=

$$
A + \frac {a}{2} \approx A, \qquad H + a \Omega \approx H, \qquad H + \frac {a}{2} \Omega \approx H
$$

假设陀螺并未完全达到动力调谐状态，存在残余刚度

$$
\Delta K = K - \left(a - \frac {c}{2}\right) \Omega^ {2}
$$

记

$$
\lambda = \Omega (\delta + D)\tag{2.4.22}
$$

则式(2.4.21)可写成

$$
A _ {\alpha} ^ {\ddot {\alpha}} + \delta \dot {\alpha} + \Delta K _ {\alpha} + H \dot {\beta} + \lambda \beta = M _ {x} ^ {C} - A \dot {\omega} _ {x} - H \omega_ {y}\tag{2.4.23a}
$$

$$
A \ddot {\beta} + \delta \dot {\beta} + \Delta K \beta - H \dot {\alpha} + \lambda \alpha = M _ {y} ^ {C} - A \dot {\omega} _ {y} + H \omega_ {x}\tag{2.4.23b}
$$

假设陀螺具有零初始条件

$$
\begin{array}{l} \alpha (0) = \beta (0) = \Phi_ {x} (0) = \Phi_ {y} (0) = 0 \\ \dot {\alpha} (0) = \dot {\beta} (0) = \dot {\Phi} _ {x} (0) = \dot {\Phi} _ {y} (0) = 0 \end{array}
$$

则对式(2.4.23)作拉氏变换，得

$$
\begin{array}{l} (A s ^ {2} + \delta s + \Delta K) \alpha (s) + (H s + \lambda) \beta (s) = M _ {x} ^ {C} (s) - A s ^ {2} \Phi_ {x} (s) - H s \Phi_ {y} (s) \\ (A s ^ {2} + \delta s + \Delta K) \beta (s) - (H s + \lambda) \alpha (s) = M _ {y} ^ {C} (s) - A s ^ {2} \Phi_ {y} (s) + H s \Phi_ {x} (s) \end{array}\tag{2.4.24a}
$$

(2. 4. 24b)

由此可得动力调谐陀螺的方块图，如图 2.4.5 所示。

![](images/3f370af72d8d9a2b5625e33e56ab788efca429f9225f19d8b01ab304553fc017.jpg)  
2.4.5 动力调谐陀螺方块图

联立求解式 (2.4.24) ，得

$$
\begin{array}{r l} \alpha (s) = & \frac {1}{\Delta (s)} \left\{\left(A s ^ {2} + \delta s + \Delta K\right) \left[ M _ {x} ^ {C} (s) - A s ^ {2} \Phi_ {x} (s) - H s \Phi_ {y} (s) \right] \right. \\ & \left. - (H s + \lambda) \left[ M _ {y} ^ {C} (s) - A s ^ {2} \Phi_ {y} (s) + H s \Phi_ {x} (s) \right] \right\} \end{array}\tag{2.4.25a}
$$

$$
\begin{array}{r l} \beta (s) = & \frac {1}{\Delta (s)} \{(A s ^ {2} + \delta s + \Delta K) [ M _ {y} ^ {C} (s) - A s ^ {2} \Phi_ {y} (s) + H s \Phi_ {x} (s) ] \\ & + (H s + \lambda) [ M _ {x} ^ {C} (s) - A s ^ {2} \Phi_ {x} (s) - H s \Phi_ {y} (s) ] \} \end{array}\tag{2.4.25b}
$$

式中，陀螺的特征多项式为

$$
\Delta (s) = (H s + \lambda) ^ {2} + (A s ^ {2} + \delta s + \Delta K) ^ {2}\tag{2.4.26}
$$

4. 动力调谐陀螺的运动特性分析

由式(2.4.26) ，陀螺的特征方程为

$$
(H s + \lambda) ^ {2} + (A s ^ {2} + \delta s + \Delta K) ^ {2} = 0
$$

即

$$
\begin{array}{r l} \left[ (A s ^ {2} + \delta s + \Delta K) + j (H s + \lambda) \right] \cdot \left[ (A s ^ {2} + \delta s + \Delta K) - j (H s + \lambda) \right] & = 0 \\ A s ^ {2} + \delta s + \Delta K + j (H s + \lambda) & = 0 \\ A s ^ {2} + \delta s + \Delta K - j (H s + \lambda) & = 0 \end{array}
$$

解上述二次方程，经简化处理后，得特征根

$$
s _ {1, 2} = - \frac {\lambda}{H} \pm \mathrm{j} \frac {\Delta K}{H}\tag{2.4.27}
$$

$$
s _ {3, 4} = - \left(\frac {\delta}{A} - \frac {\lambda}{H}\right) \pm \mathrm{j} \left(\frac {H}{A} + \frac {\Delta K}{H}\right)\tag{2.4.28}
$$

$s _ { 1 }$ 和 $s _ { 2 }$ 的实部始终为负， $s _ { 1 }$ 和 $s _ { 2 }$ 始终位于左半[5]平面内。 $s _ { 3 }$ 和 $s _ { 4 }$ 的实部分析如下:考虑到式(2.4.22) C=2A 的关系

$$
R _ {e} s _ {3, 4} = \frac {\lambda}{H} - \frac {\delta}{A} = \frac {\Omega (\delta + D)}{2 A \Omega} - \frac {\delta}{A} = \frac {D - \delta}{2 A}
$$

在设计中一般都能满足 $D { < } \delta$ ，即材料内阻尼大于外阻尼，所以 $s _ { 3 }$ 和 $s _ { 4 }$ 也位于左半[5]平面内。所以动力调谐陀螺是一种稳定系统。

上述分析说明动力调谐陀螺的输出是一种衰减振荡， $s _ { 1 }$ 和 $s _ { 2 }$ 应的振荡频率为 $\textstyle { \frac { \Delta K } { H } }$ ， $\Delta K$ 是调谐残余刚度，数值非常小，所以对应着低频振荡。这种低频振荡是衰减的，是进动过程中的伴随振荡，当陀螺完全达到动力调谐时，b $\Delta K { = } 0$ ，此时进动无振荡。 $s _ { 3 }$ 和 $s _ { 4 }$ 对应的振荡频率为

$$
\frac {H}{A} + \frac {\Delta K}{H} \approx 2 \Omega
$$

所以对应着高频振荡，这就是章动，章动角频率是自转角速度的 倍。

## 5. 动力调谐陀螺的传递函数

由图 2.4.5 可看出，动力调谐陀螺的两个通道存在交叉耦合。引起交叉耦合的因素有:调谐残余刚度，弹性扭杆的内阻尼系数及转子绕输出轴旋转所引起的惯性力矩。若忽略这些交叉耦合影响，则图 2. 4. 可简化成图 2.4.6

![](images/41d52cef3d68e1bcf157ad5e207cf5177fb5430e7191599ab89b8d5016ceacfc.jpg)  
2.4.6 动力调谐陀螺解调简化后的方块图

由图 2.4.6 可得陀螺的传递函数:

$$
\frac {\alpha (s)}{\Phi_ {x} (s)} = - \frac {H s}{H s + \lambda} = - \frac {\tau_ {g} s}{\tau_ {g} s + 1}\tag{2.4.29a}
$$

$$
\frac {\beta (s)}{\Phi_ {y} (s)} = - \frac {H s}{H s + \lambda} = - \frac {\tau_ {g} s}{\tau_ {g} s + 1}\tag{2.4.29b}
$$

$$
\frac {\alpha (s)}{M _ {y} ^ {C} (s)} = - \frac {1}{H s + \lambda} = - \frac {\frac {1}{\lambda}}{\tau_ {g} s + 1}\tag{2.4.30a}
$$

$$
\frac {\beta (s)}{M _ {x} ^ {C} (s)} = \frac {1}{H s + \lambda} = \frac {\frac {1}{\lambda}}{\tau_ {g} s + 1}\tag{2.4.30b}
$$

上述诸式中

$$
\tau_ {g} = \frac {H}{\lambda}
$$

为陀螺的时间常数。若，1=0，即陀螺元风阻阻尼(外阻尼)和材料的内阻尼，则 $\tau _ { g } \to \infty$ ，此时有

$$
\frac {\alpha (s)}{\Phi_ {x} (s)} = - 1, \quad \frac {\beta (s)}{\Phi_ {y} (s)} = - 1
$$

$$
\frac {s \alpha (s)}{M _ {y} ^ {C} (s)} = - \frac {1}{H}, \quad \frac {s \beta (s)}{M _ {x} ^ {C} (s)} = \frac {1}{H}
$$

即

$$
\frac {\dot {\alpha} (s)}{M _ {y} ^ {C} (s)} = - \frac {1}{H}\tag{2.4.31a}
$$

$$
\frac {\dot {\beta} (s)}{M _ {x} ^ {C} (s)} = \frac {1}{H}\tag{2.4.31b}
$$

上述诸式说明，只有当陀螺的时间常数为无穷大时，动力调谐陀螺才成为理想的角位置陀螺，陀螺才呈现理想的无章动进动特性。

## 2.4.4 动力调谐陀螺的静态漂移误差数学模型

为分析问题方便，将陀螺壳体坐标系的 $x _ { c }$ 轴记为A轴， $y _ { _ { C } }$ 轴记为B轴， $z _ { c }$ 轴记为 轴。假设陀螺工作在动力调谐状态，输出角 $\alpha { = } 0 , \beta { = } 0$ ，作用在 轴和 轴上的干扰力短分别为 $M _ { \mathrm { d } A }$ A和$M _ { \mathrm { d } B }$ ，则根据式(2.4.31) ，引起的陀螺漂移为

$$
\omega_ {\mathrm{dA}} = - \frac {M _ {\mathrm{dB}}}{H}\tag{2.4.32a}
$$

$$
\omega_ {\mathrm{dB}} = \frac {M _ {\mathrm{dA}}}{H}\tag{2.4.32b}
$$

由于陀螺处于动力调谐状态，转子成为自由陀螺，可视为不受外界机械约束。设转子质量为质心相对支撑中心(内外扭杆轴交点)沿 轴的偏差为 $l _ { _ A }$ 和 $l _ { _ B }$ 由运载体运动引起的单位质量'惯性力为:. $\begin{array} { r } { \pmb { F } ^ { C } = [ F _ { A } \quad F _ { B } \quad F _ { S } ] ^ { \mathrm { T } } } \end{array}$ ，如图 2.4.7 所示。则由静态偏心引起的沿 轴的干扰力矩为

$$
M _ {\mathrm{dA1}} = m F _ {S} l _ {B} - m F _ {B} l _ {S}
$$

$$
M _ {\mathrm{dB1}} = m F _ {A} l _ {S} - m F _ {S} l _ {A}\tag{2.4.33a}
$$

(2. 4. 33b)

![](images/4dde1f9cad94ac364ac169eb7370f1265c244e2b9ce8f754b35c986241574fc1.jpg)  
2.4.7 静态偏心和动态偏心引起的干扰力矩

设陀螺支撑系统(由内外扭杆及平衡环构成)的柔度张量为

$$
\left[ \begin{array}{c c c} C _ {A A} & C _ {A B} & C _ {A S} \\ C _ {B A} & C _ {B B} & C _ {B S} \\ C _ {S A} & C _ {S B} & C _ {S S} \end{array} \right]
$$

则转子的动态偏心为

$$
\left[ \begin{array}{l} \delta_ {A} \\ \delta_ {B} \\ \delta_ {S} \end{array} \right] = m \left[ \begin{array}{c c c} C _ {A A} & C _ {A B} & C _ {A S} \\ C _ {B A} & C _ {B B} & C _ {B S} \\ C _ {S A} & C _ {S B} & C _ {S S} \end{array} \right] \left[ \begin{array}{l} F _ {A} \\ F _ {B} \\ F _ {S} \end{array} \right] = \left[ \begin{array}{l} m (C _ {A A} F _ {A} + C _ {A B} F _ {B} + C _ {A S} F _ {S}) \\ m (C _ {B A} F _ {A} + C _ {B B} F _ {B} + C _ {B S} F _ {S}) \\ m (C _ {S A} F _ {A} + C _ {S B} F _ {B} + C _ {S S} F _ {S}) \end{array} \right]
$$

在惯性力作用下所产生的干扰力矩为

$$
\begin{array}{r l} M _ {\mathrm{dA2}} & = m F _ {S} \delta_ {B} - m F _ {B} \delta_ {S} \\ & = m ^ {2} (C _ {B A} F _ {A} F _ {S} + C _ {B B} F _ {B} F _ {S} + C _ {B S} F _ {S} ^ {2}) - m ^ {2} (C _ {S A} F _ {A} F _ {B} + C _ {S B} F _ {B} ^ {2} + C _ {S S} F _ {S} F _ {B}) \\ M _ {\mathrm{dB2}} & = m F _ {A} \delta_ {S} - m F _ {S} \delta_ {A} \\ & = m ^ {2} (C _ {S A} F _ {A} ^ {2} + C _ {S B} F _ {B} F _ {A} + C _ {S S} F _ {S} F _ {A}) - m ^ {2} (C _ {A A} F _ {A} F _ {S} + C _ {A B} F _ {B} F _ {S} + C _ {A S} F _ {S} ^ {2}) \end{array}\tag{2.4.34a}
$$

(2. 4. 34b)

根据式(2.4. 32). 干扰力矩引起的漂移为

$$
\omega_ {\mathrm{d} A} = - \frac {M _ {\mathrm{d} B 0}}{H} - \frac {M _ {\mathrm{d} B 1}}{H} - \frac {M _ {\mathrm{d} B 2}}{H}, \quad \omega_ {\mathrm{d} B} = \frac {M _ {\mathrm{d} A 0}}{H} + \frac {M _ {\mathrm{d} A 1}}{H} + \frac {M _ {\mathrm{d} A 2}}{H}
$$

式中. $M _ { \mathrm { d A 0 } }$ 。和 $M _ { \mathrm { d } B 0 }$ 是与运动元关的干扰力矩，如力矩器的零位误差等。式 (2.4.33)(2.4.34)代入上式，整理后得

$$
\begin{array}{r l} \omega_ {\mathrm{dA}} = & - \frac {M _ {\mathrm{dB0}}}{H} - \frac {m l _ {\mathrm{s}}}{H} F _ {A} + \frac {m l _ {A}}{H} F _ {S} - \frac {m ^ {2} C _ {S B}}{H} F _ {A} F _ {B} + \frac {m ^ {2} C _ {A B}}{H} F _ {S} F _ {B} \\ & - \frac {m ^ {2} (C _ {\mathrm{SS}} - C _ {A A})}{H} F _ {A} F _ {S} - \frac {m ^ {2} C _ {\mathrm{SA}}}{H} F _ {A} ^ {2} + \frac {m ^ {2} C _ {\mathrm{AS}}}{H} F _ {S} ^ {2} \\ \omega_ {\mathrm{dB}} = & \frac {M _ {\mathrm{dA0}}}{H} - \frac {m l _ {\mathrm{s}}}{H} F _ {B} + \frac {m l _ {B}}{H} F _ {S} - \frac {m ^ {2} C _ {\mathrm{SA}}}{H} F _ {A} F _ {B} + \frac {m ^ {2} C _ {\mathrm{BA}}}{H} F _ {A} F _ {S} \\ & + \frac {m ^ {2} (C _ {\mathrm{BB}} - C _ {\mathrm{SS}})}{H} F _ {B} F _ {S} - \frac {m ^ {2} C _ {\mathrm{SB}}}{H} F _ {B} ^ {2} + \frac {m ^ {2} C _ {\mathrm{BS}}}{H} F _ {S} ^ {2} \end{array}
$$

记

$$
\begin{array}{l} D _ {F} (A) = - \frac {M _ {\mathrm{dB0}}}{H}, \quad D _ {A} (A) = - \frac {m l _ {S}}{H}, \quad D _ {S} (A) = \frac {m l _ {A}}{H}, \\ D _ {A B} (A) = - \frac {m ^ {2} C _ {S B}}{H}, \quad D _ {B S} (A) = \frac {m ^ {2} C _ {A B}}{H}, \quad D _ {A S} (A) = - \frac {m ^ {2} (C _ {S S} - C _ {A A})}{H}, \\ D _ {A A} (A) = - \frac {m ^ {2} C _ {S A}}{H}, \quad D _ {S S} (A) = \frac {m ^ {2} C _ {A S}}{H}, \\ D _ {F} (B) = \frac {M _ {\mathrm{dA0}}}{H}, \quad D _ {B} (B) = - \frac {m l _ {S}}{H}, \quad D _ {S} (B) = \frac {m l _ {B}}{H}, \\ D _ {A B} (B) = - \frac {m ^ {2} C _ {S A}}{H}, \quad D _ {A S} (B) = \frac {m ^ {2} C _ {B A}}{H}, \quad D _ {B S} (B) = \frac {m ^ {2} (C _ {B B} - C _ {S S})}{H}, \\ D _ {B B} (B) = - \frac {m ^ {2} C _ {S B}}{H}, \quad D _ {S S} (B) = \frac {m ^ {2} C _ {B S}}{H} \end{array}
$$

则陀螺的静态漂移误差数学模型为

$$
\begin{array}{r l} \omega_ {\mathrm{dA}} & = D _ {F} (A) + D _ {A} (A) F _ {A} + D _ {B} (A) F _ {B} + D _ {S} (A) F _ {S} + D _ {A B} (A) F _ {A} F _ {B} + D _ {A S} (A) F _ {A} F _ {S} \\ & \quad + D _ {B S} (A) F _ {B} F _ {S} + D _ {A A} (A) F _ {A} ^ {2} + D _ {B B} (A) F _ {B} ^ {2} + D _ {S S} (A) F _ {S} ^ {2} \quad \text {(2.4)} \\ \omega_ {\mathrm{dB}} & = D _ {F} (B) + D _ {A} (B) F _ {A} + D _ {B} (B) F _ {B} + D _ {S} (B) F _ {S} + D _ {A B} (B) F _ {A} F _ {B} + D _ {A S} (B) F _ {A} F _ {S} \\ & \quad + D _ {B S} (B) F _ {B} F _ {S} + D _ {A A} (B) F _ {A} ^ {2} + D _ {B B} (B) F _ {B} ^ {2} + D _ {S S} (B) F _ {S} ^ {2} \quad \text {(2.4)} \end{array}\tag{2.4.35a}
$$

(2. 4. 35b)

式中 $D _ { B } ( A ) F _ { B } , D _ { B B } ( A ) F _ { B } ^ { 2 } , D _ { A } ( B ) F _ { A } , D _ { A A } ( B ) F _ { A } ^ { 2 }$ 根据对称关系增补的误差项，由于误差系数是通过测试确定的，增补项是否存在由确定出的误差系数是否为零来确定，所以增补后并不影响模型的真实性。

在实验室条件下 $\scriptstyle { F = g }$ 所以在实验室条件下用于测试的静态模型为

$$
\begin{array}{r l} \omega_ {\mathrm{dA}} & = D _ {F} (A) + D _ {A} (A) g _ {A} + D _ {B} (A) g _ {B} + D _ {S} (A) g _ {S} + D _ {A B} (A) g _ {A} g _ {B} + D _ {A S} (A) g _ {A} g _ {S} \\ & \quad + D _ {B S} (A) g _ {B} g _ {S} + D _ {A A} (A) g _ {A} ^ {2} + D _ {B B} (A) g _ {B} ^ {2} + D _ {S S} (A) g _ {S} ^ {2} \quad \text {(2.)} \\ \omega_ {\mathrm{dB}} & = D _ {F} (B) + D _ {A} (B) g _ {A} + D _ {B} (B) g _ {B} + D _ {S} (B) g _ {S} + D _ {A B} (B) g _ {A} g _ {B} + D _ {A S} (B) g _ {A} g _ {S} \\ & \quad + D _ {B S} (B) g _ {B} g _ {S} + D _ {A A} (B) g _ {A} ^ {2} + D _ {B B} (B) g _ {B} ^ {2} + D _ {S S} (B) g _ {S} ^ {2} \quad \text {(2.)} \end{array}\tag{2.4.36a}
$$

(2.4. 36b)

式中 $g _ { A } \cdot g _ { B }$ 和 $g _ { s }$ 为重力加速度沿 轴、 轴和 轴的分量。

## 2.4.5 动力调谐陀螺的动态漂移误差数学模型

1.动态漂移误差形成机理分析

以陀螺转子为研究对象，根据动量矩定理

$$
\left. \frac {\mathrm{d} \boldsymbol {H} _ {R}}{\mathrm{d} t} \right| _ {i} = \boldsymbol {M}
$$

应用哥氏定理，并向转子参考坐标系 投影

$$
\left. \frac {\mathrm{d} \boldsymbol {H} _ {R}}{\mathrm{d} t} \right| _ {G} ^ {G} + \boldsymbol {\omega} _ {i G} ^ {G} \times \boldsymbol {H} _ {R} ^ {G} = \boldsymbol {M} ^ {G}
$$

(2.4.10) 和式 (2.4. 11)代入上式

$$
\begin{array}{l} \frac {\mathrm{d}}{\mathrm{d} t} \left[ \begin{array}{c} A (\omega_ {A} + \dot {\alpha} - \beta \omega_ {S}) \\ A (\omega_ {B} + \dot {\beta} + \alpha \omega_ {S}) \\ C (\omega_ {S} + \beta \omega_ {A} - \alpha \omega_ {B} + \Omega) \end{array} \right] \\ + \left[ \begin{array}{c c c} 0 & - (\omega_ {S} + \beta \omega_ {A} - \alpha \omega_ {B}) & \omega_ {B} + \dot {\beta} + \alpha \omega_ {S} \\ \omega_ {S} + \beta \omega_ {A} - \alpha \omega_ {B} & 0 & - (\omega_ {A} + \dot {\alpha} - \beta \omega_ {S}) \\ - (\omega_ {B} + \dot {\beta} + \alpha \omega_ {S}) & \omega_ {A} + \dot {\alpha} - \beta \omega_ {S} & 0 \end{array} \right] \\ \times \left[ \begin{array}{c} A (\omega_ {A} + \dot {\alpha} - \beta \omega_ {S}) \\ A (\omega_ {B} + \dot {\beta} + \alpha \omega_ {S}) \\ C (\omega_ {S} + \beta \omega_ {A} - \alpha \omega_ {B} + \Omega) \end{array} \right] = \left[ \begin{array}{c} M _ {A G} \\ M _ {B G} \\ M _ {S G} \end{array} \right] \end{array}
$$

展开上式，略去关 $\digamma _ { \alpha , \beta , \dot { \alpha } , \dot { \beta } }$ 的二阶小量，并记 $H { = } C \Omega$ ，得

$$
\begin{array}{l} A (\dot {\omega} _ {A} + \ddot {\alpha}) + H (\dot {\beta} + \omega_ {B}) + (C - A) \omega_ {B} \omega_ {S} - [ (C - A) (\omega_ {B} ^ {2} - \omega_ {S} ^ {2}) - H \omega_ {S} ] \alpha \\ + [ (C - A) \omega_ {A} \omega_ {B} - A \dot {\omega} _ {S} ] \beta = M _ {A G} \\ A (\dot {\omega} _ {B} + \ddot {\beta}) - H (\dot {\alpha} + \omega_ {A}) - (C - A) \omega_ {A} \omega_ {S} - [ (C - A) (\omega_ {A} ^ {2} - \omega_ {S} ^ {2}) - H \omega_ {S} ] \beta \\ + [ (C - A) \omega_ {A} \omega_ {B} + A \dot {\omega} _ {S} ] \alpha = M _ {B G} \\ C (\dot {\omega} _ {S} + \beta \dot {\omega} _ {A} + \dot {\beta} \omega_ {A} - \alpha \dot {\omega} _ {B} - \dot {\alpha} \omega_ {B}) = M _ {S G} \end{array}\tag{2.4.37a}
$$

(2. 4. 37b)

设外作用力矩在基座坐标系内的分量为 $M _ { A } , M _ { B } , M _ { S }$ ,则

$$
\left[ \begin{array}{c} M _ {A G} \\ M _ {B G} \\ M _ {S G} \end{array} \right] = \boldsymbol {C} _ {C} ^ {G} \left[ \begin{array}{c} M _ {A} \\ M _ {B} \\ M _ {S} \end{array} \right] = \left[ \begin{array}{c c c} 1 & 0 & - \beta \\ 0 & 1 & \alpha \\ \beta & - \alpha & 1 \end{array} \right] \left[ \begin{array}{c} M _ {A} \\ M _ {B} \\ M _ {S} \end{array} \right] = \left[ \begin{array}{c} M _ {A} - \beta M _ {S} \\ M _ {B} + \alpha M _ {S} \\ M _ {S} + \beta M _ {A} - \alpha M _ {B} \end{array} \right]
$$

式中. $M _ { S }$ :二为电机的驱动力矩，对于恒速驱动

$$
M _ {S} = C (\dot {\omega} _ {S} + \dot {\Omega}) = C \dot {\omega} _ {S}
$$

所以

$$
\begin{array}{l} M _ {A G} = M _ {A} - \beta C \dot {\omega} _ {S} = M _ {k A} + M _ {\mathrm{d} A} - \beta C \dot {\omega} _ {S} \\ M _ {B G} = M _ {B} + \alpha C \dot {\omega} _ {S} = M _ {k B} + M _ {\mathrm{d} B} + \alpha C \dot {\omega} _ {S} \end{array}
$$

式中. $M _ { k 4 }$ 和 $M _ { k B }$ 是作用在 轴上的反馈力矩，由再平衡回路提供. $M _ { \mathrm { d A } }$ A和 $M _ { \mathrm { d } B }$ 是作用在轴上的干扰力矩。上述两式分别代入式(2.4.37) .得

$$
A \ddot {\alpha} + H \dot {\beta} = - H \omega_ {B} + M _ {k A} + M _ {C A} + M _ {\mathrm{dA}}\tag{2.4.38a}
$$

$$
A \ddot {\beta} - H \dot {\alpha} = H \omega_ {A} + M _ {k B} + M _ {C B} + M _ {\mathrm{dB}}\tag{2.4.38b}
$$

式中

$$
\begin{array}{r l} M _ {C A} = & - A \dot {\omega} _ {A} - (C - A) \omega_ {B} \omega_ {S} + [ (C - A) (\omega_ {B} ^ {2} - \omega_ {S} ^ {2}) - H \omega_ {S} ] \alpha \\ & - [ (C - A) \omega_ {A} \omega_ {B} + (C - A) \dot {\omega} _ {S} ] \beta \\ M _ {C B} = & - A \dot {\omega} _ {B} + (C - A) \omega_ {A} \omega_ {S} + [ (C - A) (\omega_ {A} ^ {2} - \omega_ {S} ^ {2}) - H \omega_ {S} ] \beta \\ & - [ (C - A) \omega_ {A} \omega_ {B} - (C - A) \dot {\omega} _ {S} ] \alpha \end{array}\tag{2.4.39a}
$$

4. 39b)

当动力调谐陀螺工作在力反馈状态时，陀螺的作用是要测量出沿 测量轴的运载体角速度，用于捷联惯导解算，由式(2.4. 38).角速度的精确值应为

$$
\begin{array}{l} \omega_ {A} = \frac {1}{H} (A \ddot {\beta} - H \dot {\alpha}) - \frac {M _ {k B}}{H} - \frac {M _ {C B}}{H} - \frac {M _ {\mathrm{dB}}}{H} \\ \omega_ {B} = - \frac {1}{H} (A \ddot {\alpha} + H \dot {\beta}) + \frac {M _ {k A}}{H} + \frac {M _ {C A}}{H} + \frac {M _ {\mathrm{dA}}}{H} \end{array}
$$

但在实际测量过程中，只能根据再平衡回路的电流(或电压)确定 $\omega _ { A }$ 和 $\omega _ { B }$ ，即反馈力矩 $M _ { k 4 }$ ，和$M _ { k B }$ 是可测量得的，所以

$$
\hat {\omega} _ {A} = - \frac {M _ {k B}}{H}, \quad \hat {\omega} _ {B} = \frac {M _ {k A}}{H}
$$

引起的测量误差为

$$
\Delta \omega_ {A} = \dot {\omega} _ {A} - \omega_ {A} = - \frac {1}{H} (A \ddot {\beta} - H \dot {\alpha}) + \frac {M _ {C B}}{H} + \frac {M _ {\mathrm{d} B}}{H}, \quad \Delta \omega_ {B} = \dot {\omega} _ {B} - \omega_ {B} = \frac {1}{H} (A \ddot {\alpha} + H \dot {\beta}) - \frac {M _ {C A}}{H} - \frac {M _ {\mathrm{d} A}}{H}
$$

式中

$$
\Delta \omega_ {A 1} = - \frac {1}{H} (A \ddot {\beta} - H \dot {\alpha}), \quad \Delta \omega_ {B 1} = \frac {1}{H} (A \ddot {\alpha} + H \dot {\beta})
$$

是由惯性力矩 $A { \ddot { \beta } }$ 和 $A { \ddot { \alpha } }$ 以及陀螺力矩 $H _ { \alpha } ^ { * }$ 和 $H \dot { \boldsymbol { \beta } }$ 引起的，而斗， $\dot { \alpha } , \dot { \beta } , \ddot { \alpha } , \ddot { \beta }$ ;;、卢是无法测量的，所以 $\Delta \omega _ { A 1 }$ 和$\Delta \omega _ { B 1 }$ 1无法补偿。

$\Delta \omega _ { A 2 } = \frac { M _ { \mathrm { d } B } } { H }$ 和 $\Delta \omega _ { B 2 } = - \frac { M _ { \mathrm { d A } } } { H }$ 扰力矩确定，对于随机干扰力矩，跚跚差也无法补偿。

$$
\delta \omega_ {A} = \frac {M _ {C B}}{H}, \quad \delta \omega_ {B} = - \frac {M _ {C A}}{H}
$$

是由基座角运动引起的，而基座角速度可由捷联陀螺测量，所以是可以补偿的。将式(2.4.39)人上述两式，得

$$
\begin{array}{r l} \delta \omega_ {A} & = - \frac {A}{H} \dot {\omega} _ {B} + \frac {C - A}{H} \omega_ {A} \omega_ {S} + \frac {1}{H} [ (C - A) (\omega_ {A} ^ {2} - \omega_ {S} ^ {2}) - H \omega_ {S} ] \beta \\ & - \frac {1}{H} [ (C - A) \omega_ {A} \omega_ {B} - (C - A) \dot {\omega} _ {S} ] \alpha \\ \delta \omega_ {B} & = \frac {A}{H} \dot {\omega} _ {A} + \frac {C - A}{H} \omega_ {B} \omega_ {S} - \frac {1}{H} [ (C - A) (\omega_ {B} ^ {2} - \omega_ {S} ^ {2}) - H \omega_ {S} ] \alpha \\ & + \frac {1}{H} [ (C - A) \omega_ {A} \omega_ {B} + (C - A) \dot {\omega} _ {S} ] \beta \end{array}\tag{2.4.40a}
$$

(2.4.40b)

2. 对动态漂移误差中各项误差的讨论

将式(2.4.40)写成矩阵形式

$$
\begin{array}{r l} \left[ \begin{array}{c} \delta \omega_ {A} \\ \delta \omega_ {B} \end{array} \right] & = \frac {A}{H} \left[ \begin{array}{c} - \dot {\omega} _ {B} \\ \dot {\omega} _ {A} \end{array} \right] + \frac {C - A}{H} \left[ \begin{array}{c} \omega_ {A} \omega_ {S} \\ \omega_ {B} \omega_ {S} \end{array} \right] + \frac {C - A}{H} \left[ \begin{array}{c c} \dot {\omega} _ {S} - \omega_ {A} \omega_ {B} & \omega_ {A} ^ {2} - \omega_ {S} ^ {2} \\ \omega_ {S} ^ {2} - \omega_ {B} ^ {2} & \dot {\omega} _ {S} + \omega_ {A} \omega_ {B} \end{array} \right] \left[ \begin{array}{c} \alpha \\ \beta \end{array} \right] \\ & + \left[ \begin{array}{c c} 0 - \omega_ {S} \\ \omega_ {S} & 0 \end{array} \right] \left[ \begin{array}{c} \alpha \\ \beta \end{array} \right] \end{array}\tag{2.4.41}
$$

对上式各项讨论如下

1)角加速度误差

$$
\left[ \begin{array}{c} \delta \omega_ {A 1} \\ \delta \omega_ {B 1} \end{array} \right] = \frac {A}{H} \left[ \begin{array}{c} - \dot {\omega} _ {B} \\ \dot {\omega} _ {A} \end{array} \right]
$$

由测量轴方向的基座角加速度引起。由于在设计中采取 $C { = } 2 A$ .所以 $\frac { A } { H } = \frac { 1 } { 2 \Omega }$

$$
\left[ \begin{array}{c} \delta \omega_ {A 1} \\ \delta \omega_ {B 1} \end{array} \right] = \frac {1}{2 \Omega} \left[ \begin{array}{c} - \dot {\omega} _ {B} \\ \dot {\omega} _ {A} \end{array} \right]
$$

式中， $\frac { A } { H } \Re \Re \frac { 1 } { 2 \Omega }$ 称为角加速度误差系数。

动力调谐陀螺转子转速的典型数值范围为 $6 0 0 0 { \sim } 2 0 0 0 0 \mathrm { r / m i n }$ . \~p $\Omega { = } 6 2 8 { \sim } 2 0 9 3 \mathrm { r a d / s }$ . 因而角加速度误差系数的典型数值范围为 $5 0 { \sim } 1 6 5 ^ { \circ } / \mathrm { h } / ( \mathrm { r a d } / \mathrm { s } ^ { 2 } )$ 这表明，当基座绕测量轴的角加速度为 $\mathrm { 1 r a d / s ^ { 2 } }$ 时，所引起的角速度测量误差达到每小时数十度，因此角加速度误差是一项相当大的动态误差，捷联惯导中应对此项误差作补偿。

2) 不等惯性误差

$$
\left[ \begin{array}{l} \delta \omega_ {A 2} \\ \delta \omega_ {B 2} \end{array} \right] = \frac {C - A}{H} \left[ \begin{array}{l} \omega_ {A} \omega_ {S} \\ \omega_ {B} \omega_ {S} \end{array} \right] = \frac {1}{2 \Omega} \left[ \begin{array}{l} \omega_ {A} \omega_ {S} \\ \omega_ {B} \omega_ {S} \end{array} \right]
$$

式中， $\frac { C - A } { H }$ 或 $\frac { 1 } { 2 \Omega }$ 称为不等惯性误差系数。

不等惯性误差系数与角加速度误差系数相同，其典型数值范围也是 $5 0 { \sim } 1 6 5 ^ { \circ } / \mathrm { h } / ( \mathrm { r a d / s } ) ^ { 2 }$ 这表明当 $\omega _ { A } \omega _ { S } { = } 1 ( \mathrm { r a d / s } ) ^ { 2 }$ 或 $\omega _ { B } \omega _ { S } { = } 1 ( \mathrm { r a d / s } ) ^ { 2 }$ 时，所引起的角速度测量误差也达到每小时数十度，因此也是一项不可忽视的动态误差，在捷联惯导中是应该补偿的。

3) 不等惯性耦合误差

$$
\left[ \begin{array}{l} \delta \omega_ {A 3} \\ \delta \omega_ {B 3} \end{array} \right] = \frac {C - A}{H} \left[ \begin{array}{c c} \dot {\omega} _ {S} - \omega_ {A} \omega_ {B} & \omega_ {A} ^ {2} - \omega_ {S} ^ {2} \\ \omega_ {S} ^ {2} - \omega_ {B} ^ {2} & \dot {\omega} _ {S} + \omega_ {A} \omega_ {B} \end{array} \right] \left[ \begin{array}{l} \alpha \\ \beta \end{array} \right]
$$

该项误差是由转子的极转动惯量与赤道转动惯量不相等，并通过陀螺、输出角 $\alpha$ 和 $\beta$ 的耦合作用引起的。由于测量输出角 $\alpha$ 和 $\beta$ 并用于动态误差补偿有诸多不便，所以应该用输入角速度来表达 $\alpha$ 和 $\beta$ o

假设陀螺的力矩再平衡回路具有解耦作用，即两测量通道是无耦合的，如图 2.4.6 所示。在力矩反馈状态下

$$
\begin{array}{c} M _ {y} ^ {C} (s) = k \alpha (s) \\ M _ {x} ^ {C} (s) = - k \beta (s) \end{array}\tag{2.4.42a}
$$

(2. 4. 42b)

则两测量通道的方块图如图 2. 4. 所示。

![](images/3756c19f2ec07e3e94c7ab62da0b61fcbf4d80b3d3e8c1f9c037c93759f79466.jpg)  
2.4.8 测量通道解榈条件下的方块图

考虑到是>>À 由图得

$$
\begin{array}{l} \frac {\alpha (s)}{\omega_ {A} (s)} = - \frac {H}{H s + \lambda + k} \approx - \frac {H}{H s + k} \\ \frac {\beta (s)}{\omega_ {B} (s)} = - \frac {H}{H s + \lambda + k} \approx - \frac {H}{H s + k} \end{array}
$$

对于常值输入角速度，即 $\omega _ { A } \left( s \right) = \frac { \omega _ { A } } { s } , \omega _ { B } \left( s \right) = \frac { \omega _ { B } } { s }$ 旦，输出角的稳态值为

$$
\alpha = \lim _ {s \rightarrow 0} s \cdot \frac {- H}{H s + k} \cdot \frac {\omega_ {A}}{s} = - \frac {H}{k} \omega_ {A}\tag{2.4.43a}
$$

$$
\beta = \lim _ {s \rightarrow 0} s \cdot \frac {- H}{H s + k} \cdot \frac {\omega_ {B}}{s} = - \frac {H}{k} \omega_ {B}\tag{2.4.43b}
$$

因此

$$
\left[ \begin{array}{c} \delta \omega_ {A 3} \\ \delta \omega_ {B 3} \end{array} \right] = \frac {C - A}{H} \left(- \frac {H}{k}\right) \left[ \begin{array}{c} \dot {\omega} _ {S} \omega_ {A} - \omega_ {A} ^ {2} \omega_ {B} + \omega_ {A} ^ {2} \omega_ {B} - \omega_ {B} \omega_ {S} ^ {2} \\ \omega_ {A} \omega_ {S} ^ {2} - \omega_ {A} \omega_ {B} ^ {2} + \dot {\omega} _ {S} \omega_ {B} + \omega_ {A} \omega_ {B} ^ {2} \end{array} \right]
$$

$$
= \frac {C - A}{k} \left[ \begin{array}{c} \omega_ {B} \omega_ {S} ^ {2} - \dot {\omega} _ {S} \omega_ {A} \\ - \omega_ {A} \omega_ {S} ^ {2} - \dot {\omega} _ {S} \omega_ {B} \end{array} \right]\tag{2.4.44}
$$

对于 $C { = } 2 A$ 的陀螺

$$
\left[ \begin{array}{c} \delta \omega_ {A 3} \\ \delta \omega_ {B 3} \end{array} \right] = \frac {C}{2 k} \left[ \begin{array}{c} \omega_ {B} \omega_ {S} ^ {2} - \dot {\omega} _ {S} \omega_ {A} \\ - \omega_ {A} \omega_ {S} ^ {2} - \dot {\omega} _ {S} \omega_ {B} \end{array} \right]\tag{2.4.45}
$$

$\frac { C - A } { k }$ 或 $\frac { C } { 2 k }$ 称为不等惯性耦合误差系数。

为了减小不等惯性耦合误差，一方面应尽量减小转子的极转动惯量与赤道转动惯量的差值，另一方面还应尽量提高再平衡回路的增益晶，在 足够高的情况下，不等惯性耦合误差是比较小的。

4) 交叉耦合误差

$$
\left[ \begin{array}{c} \delta \omega_ {A 4} \\ \delta \omega_ {B 4} \end{array} \right] = \left[ \begin{array}{c} - \beta \omega_ {S} \\ \alpha \omega_ {S} \end{array} \right]
$$

该项误差是由于输出角 $\alpha { \neq } 0 , \beta { \neq } 0$ 时 $\omega _ { S }$ 在陀螺敏感轴上出现分量引起的。式(2.4.43)人上式，得

$$
\left[ \begin{array}{c} \delta \omega_ {A 4} \\ \delta \omega_ {B 4} \end{array} \right] = \frac {H}{k} \left[ \begin{array}{c} \omega_ {B} \omega_ {S} \\ - \omega_ {A} \omega_ {S} \end{array} \right]
$$

时，? $\frac { H } { k }$ 称为交叉耦合误差系数。由于该系数的分子是数值很大的角动量，所以提高再平衡回路的增益对减小该误差系数显得特别重要。例如，设陀螺角动量 $H { = } 1 0 ^ { 6 } \mathrm { g } \bullet \mathrm { c m } ^ { 2 } / \mathrm { s }$ ，当再平衡回路增益 $k { = } 1 0 ^ { 3 } \mathrm { N } \cdot \mathrm { m } / \mathrm { r a d } \mathrm { H }$ 时，交叉耦合误差系数为 $2 0 ^ { \circ } / \mathrm { h } / ( \mathrm { r a d } / \mathrm { s } ) ^ { 2 }$ ，若增益增大十倍，则误差系数减小到1/10 因此，再平衡回路应具有足够高的增益，以减小交叉耦合误差。

以上四种动态误差是原理性误差，与陀螺的制造精度和供电电源无关。下面讨论陀螺未达到动力调谐工作状态所引起的两种误差 失调转速误差和失调转速耦合误差，

5) 失调转速误差

由式(2.4. 9) ，动力调谐陀螺的调谐角速度为

$$
\Omega_ {0} = \sqrt {\frac {K}{a - \frac {c}{2}}}\tag{2.4.46}
$$

设陀螺的实际自转角速度为

$$
\Omega^ {\prime} = \Omega_ {0} + \delta \Omega
$$

式中，&0为调谐转速误差.则陀螺角动量为

$$
H ^ {\prime} = C \left(\Omega_ {0} + \delta \Omega\right) = H \left(1 + \frac {\delta \Omega}{\Omega_ {0}}\right)
$$

式中 $H { = } C \Omega _ { 0 }$ 0

根据式(2.4. 43) ，陀螺输出角为

$$
\left[ \begin{array}{l} \alpha^ {\prime} \\ \beta^ {\prime} \end{array} \right] = - \frac {H ^ {\prime}}{k} \left[ \begin{array}{l} \omega_ {A} \\ \omega_ {B} \end{array} \right] = - \frac {H}{k} \left[ \begin{array}{l} \omega_ {A} + \frac {\delta \Omega}{\Omega_ {0}} \omega_ {A} \\ \omega_ {B} + \frac {\delta \Omega}{\Omega_ {0}} \omega_ {B} \end{array} \right]
$$

由上式并考虑式(2.4.42)所列关系，得

$$
\omega_ {A} = - \frac {k \alpha^ {\prime}}{H} - \frac {\delta \Omega}{\Omega_ {0}} \omega_ {A} = - \frac {M _ {k B} ^ {\prime}}{H} - \frac {\delta \Omega}{\Omega_ {0}} \omega_ {A} = \hat {\omega} _ {A} - \frac {\delta \Omega}{\Omega_ {0}} \omega_ {A}
$$

$$
\omega_ {B} = - \frac {k \beta^ {\prime}}{H} - \frac {\delta \Omega}{\Omega_ {0}} \omega_ {B} = \frac {M _ {k A} ^ {\prime}}{H} - \frac {\delta \Omega}{\Omega_ {0}} \omega_ {B} = \dot {\omega} _ {B} - \frac {\delta \Omega}{\Omega_ {0}} \omega_ {B}
$$

所以由 $\mathfrak { d } \Omega$ 引起的角速度测量误差为

$$
\delta \omega_ {A 5} = \hat {\omega} _ {A} - \omega_ {A} = \frac {\delta \Omega}{\Omega_ {0}} \omega_ {A}\tag{2.4.47a}
$$

$$
\delta \omega_ {B 5} = \dot {\omega} _ {B} - \omega_ {B} = \frac {\delta \Omega}{\Omega_ {0}} \omega_ {B}\tag{2.4.47b}
$$

式中， $\cdot \frac { \mathfrak { d } \varOmega } { \varOmega _ { 0 } }$ 称为失调转速误差系数。失调转速误差系数的大小取决于陀螺驱动电机供电频率的稳定度。例如，若供电频率稳定度为 $1 0 ^ { - 4 }$ .则失调转速误差系数为 $2 0 ^ { \circ } / \mathrm { h } / ( \mathrm { r a d } / \mathrm { s } )$ .当基座角速度$\omega _ { A } { = } 1 \mathrm { r a d / s }$ 或 $\omega _ { B } { = } 1 \mathrm { r a d / s }$ 时，引起的角速度测量误差达到 $2 0 ^ { \circ } / \mathrm { h }$ 。若稳定度提高一个数量级，则角速度测量误差降低一个数量级。

## 6) 失调转速耦合误差

若陀螺的自转角速度为调谐角速度，即 $\varOmega = \varOmega _ { \mathrm { 0 } }$ 则作用在陀螺上的等效弹性恢复系数为零，即

$$
K - \left(a - \frac {c}{2}\right) \Omega_ {0} ^ {2} = 0\tag{2.4.48}
$$

但当陀螺的实际自转角速度为 $\Omega ^ { \prime } { = } \Omega _ { 0 } { + } \mathfrak { d }$ 时，等效弹性恢复系数(残余弹性系数)为

$$
\Delta K = K - \left(a - \frac {c}{2}\right) \Omega^ {\prime 2} = K - \left(a - \frac {c}{2}\right) (\Omega_ {0} + \delta \Omega) ^ {2} \neq 0
$$

略去二阶小量并考虑到式(2.4.46)

$$
\Delta K = K - \left(a - \frac {c}{2}\right) \Omega_ {0} ^ {2} - 2 \Omega_ {0} \left(a - \frac {c}{2}\right) \delta \Omega = - 2 \Omega_ {0} \left(a - \frac {c}{2}\right) \delta \Omega
$$

由输出角 $\alpha$ 和 $\beta ^ { \stackrel { \triangledown } { \scriptscriptstyle \vec { J } ^ { E } } }$ 生的弹性恢复力矩为

$$
M _ {\Delta K A} = \Delta K (- \alpha) = 2 \Omega_ {0} \left(a - \frac {c}{2}\right) \delta \Omega \alpha
$$

$$
M _ {\Delta K B} = \Delta K (- \beta) = 2 \Omega_ {0} \left(a - \frac {c}{2}\right) \delta \Omega \beta
$$

(2.4.43) 代入上式，得

$$
M _ {\Delta K A} = - 2 \Omega_ {0} \left(a - \frac {c}{2}\right) \delta \Omega \frac {H}{k} \omega_ {A}, \quad M _ {\Delta K B} = - 2 \Omega_ {0} \left(a - \frac {c}{2}\right) \delta \Omega \frac {H}{k} \omega_ {B}
$$

根据式 (2.4.32) .由 $M _ { \Delta K A }$ 和 $M _ { \Delta K B }$ 引起的陀螺漂移为

$$
\delta \omega_ {A 6} = - \frac {M _ {\Delta K B}}{H} = \frac {2 \Omega_ {0} \left(a - \frac {c}{2}\right) \delta \Omega}{k} \omega_ {B}
$$

$$
\delta \omega_ {B 6} = \frac {M _ {\Delta K A}}{H} = - \frac {2 \Omega_ {0} \left(a - \frac {c}{2}\right) \delta \Omega}{k} \omega_ {A}
$$

若将式(2.4.46)代入上式，则上述漂移还可以写成

$$
\delta \omega_ {A 6} = \frac {2 \delta \Omega \sqrt {K (a - \frac {c}{2})}}{k} \omega_ {B}\tag{2.4.49a}
$$

$$
\delta \omega_ {B 6} = - \frac {2 \delta \Omega \sqrt {K (a - \frac {c}{2})}}{k} \omega_ {A}\tag{2.4.49b}
$$

式中， $\frac { 2 \mathfrak { d } \Omega \sqrt { K \left( a - \frac { c } { 2 } \right) } } { k }$ 称为失调转速耦合误差系数。

3. 动力调谐陀螺的动态漂移误差数学模型

计人式 (2.4.47) 和式 (2.4.49)所列误差后，式 (2.4.41)应改写为

$$
\begin{array}{r l} \left[ \begin{array}{c} \delta \omega_ {A} \\ \delta \omega_ {B} \end{array} \right] & = \frac {A}{H} \left[ \begin{array}{c} - \dot {\omega} _ {B} \\ \dot {\omega} _ {A} \end{array} \right] + \frac {C - A}{H} \left[ \begin{array}{c} \omega_ {A} \omega_ {S} \\ \omega_ {B} \omega_ {S} \end{array} \right] + \frac {C - A}{k} \left[ \begin{array}{c} \omega_ {B} \omega_ {S} ^ {2} - \dot {\omega} _ {S} \omega_ {A} \\ - \omega_ {A} \omega_ {S} ^ {2} - \dot {\omega} _ {S} \omega_ {B} \end{array} \right] \\ & + \frac {H}{k} \left[ \begin{array}{c} \omega_ {B} \omega_ {S} \\ - \omega_ {A} \omega_ {S} \end{array} \right] + \frac {\delta \Omega}{\Omega_ {0}} \left[ \begin{array}{c} \omega_ {A} \\ \omega_ {B} \end{array} \right] + \frac {2 \delta \Omega \sqrt {K (a - \frac {c}{2})}}{k} \left[ \begin{array}{c} \omega_ {B} \\ - \omega_ {A} \end{array} \right] \end{array}\tag{2.4.50}
$$

上式右侧第一项为角加速度误差，第二项为不等惯性误差，第三项为不等惯性耦合误差，第四项为交叉耦合误差，第五项为失调转速误差，第六项为失调转速耦合误差，每项都有明确的物理意义。从表面上看式(2.4.50)可直接应用于工程，其中角速度由捷联陀螺提供，各系数可根据物理参数确定之，但由于制造偏差和因无法准确确定出，所以直接应用于工程还存在诸多不便.工程上常在实验室条件下采用测试的方法确定出综合系数，而不必确定出每个物理参数，因此动态漂移误差模型采用如下形式

$$
\begin{array}{r l} \delta \omega_ {A} & = K _ {B d} (A) \dot {\omega} _ {B} + K _ {A} (A) \omega_ {A} + K _ {B} (A) \omega_ {B} + K _ {A S} (A) \omega_ {A} \omega_ {S} + K _ {B S} (A) \omega_ {B} \omega_ {S} \\ & + K _ {B S S} (A) \omega_ {B} \omega_ {S} ^ {2} + K _ {A S d} (A) \omega_ {A} \dot {\omega} _ {S} \end{array}\tag{2.4.51a}
$$

$$
\begin{array}{r l} \delta \omega_ {B} & = K _ {A d} (B) \dot {\omega} _ {A} + K _ {A} (B) \omega_ {A} + K _ {B} (B) \omega_ {B} + K _ {A S} (B) \omega_ {A} \omega_ {S} + K _ {B S} (B) \omega_ {B} \omega_ {S} \\ & + K _ {A S S} (B) \omega_ {A} \omega_ {S} ^ {2} + K _ {B S d} (B) \omega_ {B} \dot {\omega} _ {S} \end{array}\tag{2.4.51b}
$$

其中主要误差有三项 角加速度误差、不等惯性误差和交叉耦合误差，工程上常取这三项作为简化模型:

$$
\delta \omega_ {A} = K _ {B d} (A) \dot {\omega} _ {B} + K _ {A S} (A) \omega_ {A} \omega_ {S} + K _ {B S} (A) \omega_ {B} \omega_ {S}\tag{2.4.52a}
$$

$$
\delta \omega_ {B} = K _ {A d} (B) \dot {\omega} _ {A} + K _ {A S} (B) \omega_ {A} \omega_ {S} + K _ {B S} (B) \omega_ {B} \omega_ {S}\tag{2.4.52b}
$$

该模型虽然是简化模型，但因忽略的是相对微小的误差项，所以仍具有很好的精确性。

## 2.5 激光陀螺

机械式转子陀螺的工作原理是建立在牛顿力学基础上的，动量矩定理是分析陀螺动力学特性的基本方程，具有角动量是陀螺与一般刚体的根本区别。而角动量由机械旋转产生，机械旋转必须依靠支承，所以支承技术是机械式转子陀螺的关键技术，陀螺的性能指标越高，支承技术就越复杂，成本也就越高，这就是机械式转子陀螺的局限性。激光陀螺(RL<王〉的工作原理是建立在量子力学基础上的，与转子陀螺的工作原理有本质区别。原理上，这种陀螺是固体型的，不需要活动部件，不存在支承问题;用做捷联惯导的角速度测量时，不存在动态误差和静态误差;启动快，能瞬时启动，而转子陀螺达到稳态时需要数分钟;耐冲击高达 $5 0 0 g$ 甚至更高，而转子陀螺仅耐 $4 0 g$ 左右的冲击;动态测量范围高达 O. 0010/h\~1060/h;天然的数字输出，无须模数转换;工作可靠，寿命长，MTBF 最高达到 90000h;受温度影响小，一般情况下仅需用温度模型作温补而不必温控即可有效解决温度对漂移的影响。因此激光陀螺特别适用于构建捷联惯导系统，同时也可用于构建惯性稳定装置，用于隔离基座角运动对被稳定对象的影响并控制被稳定对象按要求的角速度旋转。

此处仅从系统设计的角度出发，对激光陀螺的工作原理和误差特性作简单介绍。

## 2.5.1 萨格奈克干涉仪

1913 年，法国科学家萨格奈克(G. Saganac)研制出了一种光学干涉仪，用于验证用无运动部件的光学系统同样可以检测出相对惯'性空间的旋转运动 (Herve C L ,2002;Sagnac G , 1913a;Sa-

![](images/d5c1aa1ae06abe01d7ed19b75b8edd1e3277222c447517d96128290c05d10716.jpg)  
2.5.1 迈克尔孙光学陀螺装置

gnac G , 1913b) 0 1925 年美国科学家迈克尔孙(A. H. Michelson) 和盖尔 (G. H. Gale) 根据干涉仪研制出了一个巨型光学陀螺装置(Michelson A H et al. ,1925) ，用于测量地球的自转角速度，该陀螺装置由300mX600m 的矩形光学回路构成，光源采用普通光，如2.5.1 所示。下面对该装置的工作原理做简要分析，并从直观的物理概念上对萨格奈克效应作说明。

从点。出发的光经过狭缝 后形成一束光，此光束到达半透半反分光镜 后产生两束光，一束为透射光 $a$ 光束，另一束为反射光 光束。透射光 $a$ 经反射镜 $M _ { 1 } \ldots M _ { 2 } \ldots M _ { 3 }$ 到达分光镜 后又形成反射光和透射光，其中透射光到达屏幕 $Q ;$ 反射光 经反射镜 $M _ { 3 }$ $M _ { 2 } , M _ { 1 }$ 到达分光镜 后，其中的反射光到达屏幕 $Q _ { c }$

$$
\begin{array}{r l} & {\text {设闭合光路的长度为}} \\ & {\qquad L (= S M _ {1} + M _ {1} M _ {2} + M _ {2} M _ {3} + M _ {3} S)} \end{array}
$$

当 $\omega = 0$ 时，透射光 $a$ 和反射光 所走的光路长度相

等，走完光路所用时间相等，即

$$
t _ {a} = t _ {b} = \frac {L}{c}
$$

式中 为光速。由于两路光同时到达 ，且光束来自同一光源，频率相同，所以相位差为零，在屏幕上产生的干涉条纹相对点 作对称分布。

当 $\omega \ne 0$ 时， $S \lrcorner M \sb { 1 } \lrcorner M \sb { 2 } \lrcorner M \sb { 3 }$ 具有切向速度

$$
v = \frac {L}{4} \cos 4 5 ^ {\circ} \cdot \omega = \frac {L}{4 \sqrt {2}} \omega
$$

沿光路的速度为

$$
v \cos 4 5 ^ {\circ} = \frac {L}{8} \omega
$$

在光束 离开 后的行进过程中，由于 $M _ { 1 }$ ，沿 $S M _ { 1 }$ 的方向有速度 ${ \mathrm { v } } _ { a } = { \frac { L } { 8 } } \omega \cdot$ ，所以光束$S M _ { 1 }$ 长度多走了一段距离 $\Delta L _ { a 1 }$ 光束 自离开 至到达 $M _ { 1 }$ ，所用的时间为

$$
t _ {a 1} = \frac {\frac {L}{4} + \Delta L _ {a 1}}{c}
$$

而 $\Delta L _ { a 1 }$ 也可以解释为 $M _ { 1 }$ ，在 $t _ { a 1 }$ 时间内沿 SM 移动的距离

$$
\Delta L _ {a 1} = t _ {a 1} \cdot v _ {a} = \frac {\frac {L}{4} + \Delta L _ {a 1}}{c} \cdot \frac {L}{8} \omega
$$

即

$$
8 c \Delta L _ {a 1} = \frac {L ^ {2}}{4} \omega + L \omega \Delta L _ {a 1}, \quad \Delta L _ {a 1} = \frac {\frac {L ^ {2}}{4} \omega}{8 c - L \omega}
$$

同理，光束 $M _ { 1 }$ ，至 $M _ { 2 }$ ，自 $M _ { 2 }$ ，至 $M _ { 3 }$ ，自 $M _ { 3 }$ ，至 都多走了 $\Delta { { L } _ { a 1 } }$ 的光程，所以光束 走完一个闭合光路总共多走的光程为

$$
\Delta L _ {a} = 4 \Delta L _ {a 1} = \frac {L ^ {2} \omega}{8 c - L \omega}
$$

光来 所走的总光程为

$$
L _ {a} = L + \Delta L _ {a} = L + \frac {L ^ {2} \omega}{8 c - L \omega} = \frac {8 c L - L ^ {2} \omega + L ^ {2} \omega}{8 c - L \omega} = \frac {L}{1 - \frac {L \omega}{8 c}}
$$

同理可以推得光束 所走的总光程为

$$
L _ {b} = \frac {L}{1 + \frac {L \omega}{8 c}}
$$

这样，光束 和光束 行进一周后到达 的光程差为

$$
L _ {a} - L _ {b} = \frac {L}{1 - \frac {L \omega}{8 c}} - \frac {L}{1 + \frac {L \omega}{8 c}} = \frac {\frac {L ^ {2} \omega}{4 c}}{1 - \left(\frac {L \omega}{8 c}\right) ^ {2}}
$$

由于 是册， $\left( \frac { L \omega } { 8 c } \right) ^ { 2 }$ 与 相比可以忽略不计，所以上式可以表示成

$$
L _ {a} - L _ {b} = \frac {L ^ {2} \omega}{4 c}
$$

当光路为正方形时，光路所包围的面积为 $A { = } \Big ( \frac { L } { 4 } \Big ) ^ { 2 } { = } \frac { L ^ { 2 } } { 1 6 }$ ，即 $L ^ { 2 } = 1 6 A$ 所以

$$
L _ {a} - L _ {b} = \frac {4 A}{c} \omega
$$

迈克尔列矩形光学陀螺所围的面积为 $3 0 0 \mathrm { m } \times 6 0 0 \mathrm { m }$ ，测量的地球旋转角速度为 $1 5 ^ { \circ } / 1$ h，若不计纬度的影响，可计算得光程差仅为 $0 . 1 7 4 \mu \mathrm { m }$ ，相当于光源波长 $\lambda { = } 0 . 7 \mu \mathrm { m }$ 1/4，即干涉条纹仅移动了 1/4 的条纹间距，所以测量灵敏度和精度都非常低。但是这种根据光学原理测量角运动信息无疑是一种新概念，它为惯性技术的发展开辟了一个全新的领域。

## 2.5.2 激光陀螺工作原理简介

1962 年氦-氖激光器问世，为光学陀螺这一新概念的实现奠定了技术基础。 1962 年起美国开始研制环形激光陀螺，其中霍尼韦尔(Honeywell)公司起步最早，成果最为显著。 1975 年该公司研制的激光陀螺惯导系统在 飞机上试飞成功，精度为 2.2 n mile/h, 1978 年在波音 727 机上试飞成功，精度为 1 n mile 1982 年，在波音 $7 4 7$ 飞机上试飞成功，精度高达 0.26n mile/h(许国祯， 1995)

1982 年起，霍尼韦尔公司的 GG1342 激光陀螺投人批量生产，目前月产量为 220 个，该陀螺的零偏稳定性达到 $0 . 0 1 ^ { \circ } / \mathrm { h } ( 1 \sigma )$ ，随机游走系数 O.005 !h，刻度系数稳定性和线性度分别达到 $5 \times 1 0 ^ { - 6 }$ 和 $6 \times 1 0 ^ { - 6 }$ ，成为研制标准航空惯导的典型陀螺之一(许国祯，1995)。相对萨格奈克干涉仪，激光陀螺做了如下两点关键改进:

(1)采用激光作为光源，激光优良的相干性，使正反方向运行的两束光在陀螺腔体内形成谐振，即光束沿腔体环路反复运行时一直能保持相干，而萨格奈克干涉仪只能走一圈

2) 改测 光程差(即相位差)为测 两束光的频率差，即拍频，这显著提高了陀螺的测量灵敏度。

2. 5. 为激光陀螺的工作原理简图。图中，激光陀螺采用三个反射镜组成环形谐振腔，即闭合光路。激光管沿光轴传播的光子向两侧经过透镜 $M _ { 4 }$ 斗和 $M _ { 5 }$ 射出，再分别由 $M _ { 1 } {  } M _ { 2 } {  } M _ { 3 }$

![](images/48a21256158567f9d2fff77c53b83aadf8e097e4047c99d8ab4cc52c7cbe7458.jpg)  
.2 激光陀螺工作原理简图

$\mathrm { \dot { \cdot } } \Pi M _ { 3 } {  } M _ { 2 } {  } M _ { 1 }$ 另一端反射回来，于是回路中有传播方向相反的两路光束。对每 光束来说，只有经过 圈返回原处时相位差为 $2 \pi$ 整数倍的光子才能诱发出与之相应的第二代光子，并以此规律逐渐增强，对于相位差不满足 $2 \pi$ 整数倍的光子，则逐渐衰减直至消失，若增强的光子多于衰减的光子，则闭合光路 作在谐振状态 谐振腔形成谐振的条件是

$$
\begin{array}{c} \frac {2 \pi L}{\lambda} = 2 \pi q \\ L = q \lambda \end{array}\tag{2.5.1}
$$

式中， 为谐振腔长度，即一圈光程 为波长 $q$ 为正整数。当满足以 条件时，若谐振腔角速度为零，则正、反方向运行的两束光在腔体 形成驻波，干涉

条纹静止不动。 $q$ 每取一个值，使得到可能在谐振腔内存在的频率或波长，即谐振腔的一个振模。同时，根据反射镜对不同偏振光的响应不同，采用奇数个反射镜的光学系统中，只出现单一的偏振光。

为分析方便 只画出激光陀螺的环形光路，如图 所示

设环形光路长度为 ，基座角速度为 $\omega$ 则点 的速度为

$$
v = \frac {\frac {L}{6}}{\cos 3 0 ^ {\circ}} \omega = \frac {L}{3 \sqrt {3}} \omega
$$

此速度沿光路方向的技

$$
v _ {a} = v _ {b} = v \cos 6 0 ^ {\circ} = \frac {L}{6 \sqrt {3}} \omega
$$

![](images/95bd3cf43941a83f0685fbd2ceb4bc5d4ccfd6daec0abc3fd4e602c9fc89dbf7.jpg)  
2.5.3 激光陀螺的环形光路

速度 $ { \mathcal { V } } _ { a }$ 使光束 的光程增加，速度 $\%$ 使光束 的光程减少。设光束 增加的光程为 $\Delta L _ { a }$ 光束 走完一个闭合光路所用时间为 $t _ { a }$ 则

$$
\Delta L _ {a} = v _ {a} \cdot t _ {a} = \frac {L}{6 \sqrt {3}} \omega \frac {L + \Delta L _ {a}}{c}
$$

从上式可解得

$$
\Delta L _ {a} = \frac {L ^ {2} \omega}{6 \sqrt {3} c - L \omega}
$$

所以，光束 的一周光程为

$$
L _ {a} = L + \Delta L _ {a} = L + \frac {L ^ {2} \omega}{6 \sqrt {3} c - L \omega} = \frac {L}{1 - \frac {L \omega}{6 \sqrt {3} c}}
$$

同理可得光束 的一周光程为

$$
L _ {b} = \frac {L}{1 + \frac {L \omega}{6 \sqrt {3} c}}
$$

根据光束 和光束 的谐振条件及波长、频率与光速间的关系:

$$
L _ {a} = q \lambda_ {a}, \quad L _ {b} = q \lambda_ {b}, \quad \lambda f = c
$$

可得两束光的频率为

$$
f _ {a} = \frac {c q}{L _ {a}}, \quad f _ {b} = \frac {c q}{L _ {b}}
$$

频率差(拍频)为

$$
\Delta f = f _ {b} - f _ {a} = \frac {L _ {a} - L _ {b}}{L _ {a} \cdot L _ {b}} c q
$$

式中

$$
L _ {a} \bullet L _ {b} = \frac {L ^ {2}}{1 - \left(\frac {L \omega}{6 \sqrt {3} c}\right) ^ {2}}
$$

$$
L _ {a} - L _ {b} = \frac {L}{1 - \frac {L \omega}{6 \sqrt {3} c}} - \frac {L}{1 + \frac {L \omega}{6 \sqrt {3} c}} = \frac {\frac {L ^ {2} \omega}{3 \sqrt {3} c}}{1 - \left(\frac {L \omega}{6 \sqrt {3} c}\right) ^ {2}}
$$

由于 $\left( \frac { L \omega } { 6 \sqrt { 3 } c } \right) ^ { 2 }$ 相比小到足以忽略不计，所以

$$
L _ {a} \cdot L _ {b} = L ^ {2}, \quad L _ {a} - L _ {b} = \frac {L ^ {2} \omega}{3 \sqrt {3} c}
$$

而周长为 的等边兰角形的面积

$$
A = \frac {1}{2} \cdot \frac {L}{3} \cdot \frac {L}{3} \sin 6 0 ^ {\circ} = \frac {\sqrt {3}}{3 6} L ^ {2}
$$

即

$$
L ^ {2} = \frac {3 6}{\sqrt {3}} A
$$

所以

$$
L _ {a} - L _ {b} = \frac {4 A}{c} \omega
$$

$$
\Delta f = \frac {\frac {4 A}{c} \omega}{L \cdot \frac {L}{q}} \cdot c = \frac {4 A}{L \lambda} \omega = K \omega\tag{2.5.2a}
$$

或

$$
\omega = \frac {L \lambda}{4 A} \Delta f = \frac {\Delta f}{K}\tag{2.5.2b}
$$

$$
K = \frac {4 A}{L \lambda}\tag{2.5.3}
$$

式中， 为谐振腔光程， 为激光源波长 为谐振腔光路所围的面积， $\Delta f$ 为正反方向行进的两束光的频率差，即拍频，单位为 Hz 为陀螺刻度系数，单位为 $1 / \mathrm { r a d }$ C

由物理学知，具有频率差的两束光的干涉条纹以一定的速度向某一个方向不断地移动，只要对单位时间内移过的条纹数作计数，就能求得拍频 $\Delta f$ ，从而按式(2. 5. 2b)计算出基座的角速度 $\omega _ { v }$

若用氮氛激光器作为光源，光波的波长 $\lambda { = } 0 . 6 3 2 8 \mu \mathrm { m }$ ，陀螺谐振腔光程 $L { = } 4 0 \mathrm { c m }$ ，用来测量地球的自转角速度 $\omega _ { \mathrm { i e } } { = } 1 5 ^ { \circ } / \mathrm { h }$ ，则根据式(2. 5. 2a)可计算得拍频为 8. 87Hz，而目前的光电读出电路可分辨出 0.005Hz甚至更低的拍频，所以与萨格奈克干涉仪相比，激光陀螺具有很高的灵敏度和精度。

## 2.5.3 激光陀螺的基本误差

## 1.闭锁效应

由式(2.5.2)知激光陀螺的拍频输出与输入角速度呈线性关系，但工程实际中并非如此，事实

上，当输入角速度小到一定程度时，陀螺的拍频输出为零。这种在小角速度输入下陀螺无输出的现象称为激光陀螺的闭锁效应，如图 2.5.4 所示，闭锁效应使陀螺在测量中出现死区。

产生闭锁效应的主要原因是:光路的各种非均匀因素，如反射镜及透镜各点的非均匀损耗与散射，谐振腔内的尘埃微粒，散射使正反方向传播的两束光互相耦合，发生频率牵引，使两束光趋向同步，频率差消失。解决闭锁效应的方法主要有兰种。

## 1)抖振偏频法

![](images/8ce52e4cb3ed479cd7d57a9b4740c83a0748bb6e966eed4cbc5b0c4b8a771896.jpg)

陀螺安装在抖动轮外般土，轮轴固定在基座上，抖动轮

2.5.4 激光陀螺的闭锁效应

辐沿径向贴有压电陶瓷，压电陶瓷的伸缩推挽作用使轮般相对轮轴做微辐往复角运动，带动陀螺绕输入轴做角抖动，抖动频率约 400Hz，幅度数角分。

设陀螺的闭锁阔值为 $\omega _ { L }$ L' 需要测量的角速度 $\omega _ { i } < \omega _ { L }$ '所加抖振角速度为 $\omega _ { \mathrm { A } } \sin { \frac { 2 \pi } { T _ { \mathrm { d } } } } t$ ，如2.5.5 所示。则沿陀螺敏感轴的角速度为

$$
\omega = \omega_ {i} + \omega_ {A} \sin \frac {2 \pi}{T _ {\mathrm{d}}} t
$$

![](images/3ea06583ebb83e77bdf8c62f373f355fe95d668ab199cad34e65ab8def45f3c0.jpg)  
2.5.5 抖动偏频角速度关系

若不存在闭锁区，即 $\omega _ { L } = 0$ ，则陀螺能感测 $\omega$ 的全部信息，在一个抖振周期内的角增量输出为

$$
\Delta \theta = \int_ {0} ^ {T _ {\mathrm{d}}} \left(\omega_ {i} + \omega_ {A} \sin \frac {2 \pi}{T _ {\mathrm{d}}} t\right) \mathrm{d} t = \int_ {0} ^ {T _ {\mathrm{d}}} \omega_ {i} \mathrm{d} t
$$

若存在闭锁区，即 $\omega _ { L } \ne 0$ ，则在一个抖振周期内的角增量输出为

$$
\begin{array}{r l} \Delta \theta^ {\prime} & = \int_ {0} ^ {T _ {\mathrm{d}}} \omega_ {i} \mathrm{d} t - \left[ \int_ {0} ^ {t _ {1}} \left(\omega_ {i} + \omega_ {A} \sin \frac {2 \pi}{T _ {\mathrm{d}}} t\right) \mathrm{d} t + \int_ {t _ {2}} ^ {t _ {3}} \left(\omega_ {i} + \omega_ {A} \sin \frac {2 \pi}{T _ {\mathrm{d}}} t\right) \mathrm{d} t \right. \\ & \quad \left. + \int_ {t _ {4}} ^ {T _ {\mathrm{d}}} \left(\omega_ {i} + \omega_ {A} \sin \frac {2 \pi}{T _ {\mathrm{d}}} t\right) \mathrm{d} t \right] = \Delta \theta - \delta \theta \end{array}
$$

由于 $T _ { \mathrm { d } } { = } \frac { 1 } { 4 0 0 } \mathrm { s } { = } 2 . 5 \mathrm { m s }$ 所以 穿越闭锁区的时间非常短，问小，用 $\Delta \theta ^ { ' }$ 近似 $\Delta \theta$ 是可以接受的。

抖振偏频法是工程上已成熟的方案，达到惯性级(漂移小于 $0 . 0 1 ^ { \circ } / \mathrm { h } )$ 的激光陀螺一般都采用此方案，GG1342 即属此类陀螺。

## 2) 恒速偏置法

RLG 安装在某一旋转装置上，该装置以恒定角速度绕陀螺敏感轴旋转，使陀螺的实际工作点移向线性区域。该方案看似直观简单，实际上存在两个工程实现难度十分大的问题:一是偏置角速度必须十分精确;二是旋转装置内部与外部间的信息传输及电源供电的复杂度与平台式惯导系统几乎相同，失去了捷联惯导应有的优点。在此基础上推出了往复式速率偏频方案，在此方案中，供电和信息传输可通过软导线来实现。

## 3) 磁镜偏频法

![](images/f3f998aae139b13d19d661416f236b26782934494a53b2f3718cac45dd2d58eb.jpg)  
2. 磁镜偏频法

磁镜偏频法利用了横向磁光克尔效应，如图 2.5.6

两束光同频率同相位，人射方向相反，如果所加磁场方向平行于反射镜，且垂直于人射面，则在该磁场作用下，光束在反射后会产生附加相移，即光程发生改变，光束 和光束 相移大小相等，但符号相反。这就是横向磁光克尔效应。

如果所加磁场是交变的，则 两束光之间的相移也是交变的，相位差(光程差)也是交变的。当输入角

速度 $\omega _ { i } = 0$ 时，光程差在一个周期内的积分为零;若 $\omega _ { i } \neq 0$ ，则光程差在一个周期内的积分即为由$\omega _ { i }$ 引起的角增量。所以此法实质上与抖振偏频法的思路是一样的，不同的是一个是磁场抖振，另一个是机械抖振。

## 2. 随机游走误差

设激光陀螺的输入角速度为 $\omega _ { i }$ ，闭锁阔值为 $\omega _ { L }$ L' 则根据文献 (Aronowitz F et al. 1970) ，正反方向传播的两束线偏振光的相位差?由下式确定:

$$
\dot {\Psi} = 2 \pi K (\omega_ {i} + \omega_ {L} \sin \Psi)\tag{2.5.4}
$$

式中 $\scriptstyle \cdot K = { \frac { 4 A } { L \lambda } } , { \dot { \Psi } } = 2 \pi \Delta f$ 。由上式可看出，当 $\omega _ { i } > _ { \omega _ { L } }$ ， $\dot { \Psi }$ 始终不为霉，即总会有拍频输出。但若 $\omega _ { i } \leqslant _ { \infty } \omega _ { L }$ L' 随着? $\boldsymbol { \Psi }$ 的变化，总会在某一时刻使 $\omega _ { L } \sin \Psi { = } - \omega _ { i }$ 成立，此时? $\dot { \Psi } { = } 0$ ，即?为恒定值，使$\omega _ { L } \sin \Psi { = } - \omega _ { i }$ 副恒成立，拍频输出恒为零，陀螺处于闭锁状态。

采用机械抖振法消除闭锁的做法是使陀螺沿输入轴方向做交变的简谐角抖动，根据文献(Hammons S W et al. , 1982; Macek W M et al. 1963) ，式 (2.5.4) 可以改写为

$$
\dot {\Psi} = 2 \pi K (\omega_ {i} + \omega_ {L} \sin \Psi + \omega_ {A} \cos \omega_ {\mathrm{d}} t)\tag{2.5.5}
$$

式中， ${ \bf \nabla } \Rightarrow \omega _ { \mathrm { d } }$ 为抖振角频率， $, \omega A$ 为角抖振速率的峰值， $\omega _ { \mathrm { d } }$ 和 $\omega _ { A }$ 的选取准则是

$$
\omega_ {A} \gg \omega_ {L}, \quad \omega_ {\mathrm{d}} \gg \omega_ {i}
$$

在此条件下，式(2. 5. 5)可略去 $\omega _ { L } \sin \Psi$ I'的影响，从而得

$$
\Psi \approx 2 \pi \left(K I _ {\omega} + K \frac {\omega_ {A}}{\omega_ {\mathrm{d}}} \sin \omega_ {\mathrm{d}} t\right)\tag{2.5.6}
$$

式中， $I _ { \omega } = \int _ { o } ^ { t } \omega _ { i } \mathrm { d } t$ 0

将式 (2. 5. 6) 代入式 (2.5.5) ，得

$$
\begin{array}{r l} \dot {\Psi} & = 2 \pi K \left[ \omega_ {i} + \omega_ {L} \sin 2 \pi \left(K I _ {\omega} + \frac {K \omega_ {A}}{\omega_ {\mathrm{d}}} \sin \omega_ {\mathrm{d}} t\right) + \omega_ {A} \cos \omega_ {\mathrm{d}} t \right] \\ & = 2 \pi K \omega_ {i} + 2 \pi K \omega_ {L} \cos (2 \pi K I _ {\omega}) \sin \left(2 \pi \frac {K \omega_ {A}}{\omega_ {\mathrm{d}}} \sin \omega_ {\mathrm{d}} t\right) \\ & + 2 \pi K \omega_ {L} \sin (2 \pi K I _ {\omega}) \cos \left(2 \pi \frac {K \omega_ {A}}{\omega_ {\mathrm{d}}} \sin \omega_ {\mathrm{d}} t\right) + 2 \pi K \omega_ {A} \cos \omega_ {\mathrm{d}} t \end{array}
$$

由于

$$
\begin{array}{l} \sin \left(2 \pi \frac {K \omega_ {A}}{\omega_ {\mathrm{d}}} \sin \omega_ {\mathrm{d}} t\right) = J _ {o} \left(2 \pi \frac {K \omega_ {A}}{\omega_ {\mathrm{d}}}\right) + 2 \sum_ {n = 1} ^ {\infty} J _ {2 n} \left(2 \pi \frac {K \omega_ {A}}{\omega_ {\mathrm{d}}}\right) \cos (2 n \omega_ {\mathrm{d}} t) \\ \cos \left(2 \pi \frac {K \omega_ {A}}{\omega_ {\mathrm{d}}} \sin \omega_ {\mathrm{d}} t\right) = 2 \sum_ {n = 1} ^ {\infty} J _ {2 n - 1} \left(2 \pi \frac {K \omega_ {A}}{\omega_ {\mathrm{d}}}\right) \sin [ (2 n - 1) \omega_ {\mathrm{d}} t ] \end{array}
$$

式中 $J _ { \mu } \Big ( 2 \pi \frac { K \omega _ { A } } { \omega _ { \mathrm { d } } } \Big )$ 为第一类 $\mu$ 阶贝塞尔函数，表达式为

$$
J _ {\mu} (z) = \sum_ {k = 0} ^ {\infty} \frac {(- 1) ^ {k}}{k ! (\mu + k + 1) !} \left(\frac {z}{2}\right) ^ {\mu + 2 k}
$$

$\mu { = } 1 , 2 , 3 , \cdots , z$ 为复变量， $| \arg z | < \pi$

所以

$$
\begin{array}{l} \dot {\Psi} = 2 \pi K \omega_ {i} + 2 \pi K \omega_ {L} \cos (2 \pi K I _ {\omega}) J _ {o} \left(2 \pi \frac {K \omega_ {A}}{\omega_ {\mathrm{d}}}\right) + 4 \pi K \omega_ {L} \cos (K I _ {\omega}) \\ \times \sum_ {n = 1} ^ {\infty} J _ {2 n} \left(2 \pi \frac {K \omega_ {A}}{\omega_ {\mathrm{d}}}\right) \cos (2 n \omega_ {\mathrm{d}} t) + 4 \pi K \omega_ {L} \sin (2 \pi K I _ {\omega}) \\ \times \sum_ {n = 1} ^ {\infty} J _ {2 n - 1} \left(2 \pi \frac {K \omega_ {A}}{\omega_ {\mathrm{d}}}\right) \sin [ (2 n - 1) \omega_ {\mathrm{d}} t ] + 2 \pi K \omega_ {A} \cos \omega_ {\mathrm{d}} t \end{array}
$$

上式后三项以抖振频率 $\omega _ { \mathrm { d } }$ 或以 $\omega _ { \mathrm { d } }$ 的倍频做简谐振荡，而激光陀螺的输出量是抖振周期整数倍时间内的角增量，积分后这些交变项在输出中会消失掉，所以仅讨论激光陀螺输出时上式可写成

$$
\dot {\Psi} = 2 \pi K \omega_ {i} + 2 \pi K \omega_ {L} \cos (2 \pi K I _ {\omega}) J _ {o} \left(2 \pi \frac {K \omega_ {A}}{\omega_ {\mathrm{d}}}\right)\tag{2.5.7}
$$

由此式可看出，当 $J _ { o } \left( \frac { 2 \pi K \omega _ { A } } { \omega _ { \mathrm { d } } } \right) = 0$ 时，即 $\frac { 2 \pi K \omega _ { A } } { \omega _ { \mathrm { d } } }$ 正好是第一类零阶贝塞尔函数的过零点时，

$$
\dot {\Psi} = 2 \pi K \omega_ {i}\tag{2.5.8}
$$

即

$$
\Delta f = K \omega_ {i}
$$

此时陀螺具有式(2.5.2)所示的理想输出特性，闭锁区完全消失。文献(Chow W W et al. ,1980)列出了第一类零阶贝塞尔函数的部分过零点。

$$
\frac {2 \pi K \omega_ {A}}{\omega_ {\mathrm{d}}} = 2. 4 0 5, 5. 5 2 0, \dots\tag{2.5.9}
$$

但实际的工程实现中，式(2. 5. 9)是很难满足的，原因是由抖动轮和陀螺组成的机械系统必须处于谐振状态。在设计抖振系统时，在首先满足谐振的条件下，应尽量减小 $J _ { o } \left( \frac { 2 \pi K \omega _ { A } } { \omega _ { \mathrm { d } } } \right)$ )，从而

降低闭锁效应影响。

由式(2. 5. 7)和式(2. 5. 8)可得抖振陀螺的拍频输出误差

$$
\delta \dot {\Psi} = 2 \pi K \omega_ {L} \cos (2 \pi K I _ {\omega}) J _ {o} \left(\frac {2 \pi K \omega_ {A}}{\omega_ {\mathrm{d}}}\right) \leqslant 2 \pi K \omega_ {L} J _ {o} \left(\frac {2 \pi K \omega_ {A}}{\omega_ {\mathrm{d}}}\right)
$$

为分析方便，对拍频误差的上界进行讨论，并仍用 'lÌ'表示如下

$$
\delta \dot {\Psi} = 2 \pi K \omega_ {L} J _ {o} \left(\frac {2 \pi K \omega_ {A}}{\omega_ {\mathrm{d}}}\right)
$$

在一个抖振周期 $T _ { \mathrm { d } }$ 内的积分值为

$$
\delta \Psi = 2 \pi T _ {\mathrm{d}} K \omega_ {L} J _ {o} \left(\frac {2 \pi K \omega_ {A}}{\omega_ {\mathrm{d}}}\right)
$$

式中， $T _ { \mathrm { d } } = \frac { 2 \pi } { \omega _ { \mathrm { d } } }$ .a'l'是相位角误差.

根据零阶贝塞尔函数的渐近关系

$$
J _ {o} (x) = \sqrt {\frac {2}{\pi x}} \cos \left(x - \frac {\pi}{4}\right), \quad x \rightarrow \infty
$$

由于 $\cdot \frac { 2 \pi K \omega _ { A } } { \omega _ { \mathrm { d } } }$ 是很大的数，所以

$$
J _ {o} \left(\frac {2 \pi K \omega_ {A}}{\omega_ {\mathrm{d}}}\right) = \sqrt {\frac {2}{\pi \frac {2 \pi K \omega_ {A}}{\omega_ {\mathrm{d}}}}} \cos \left(\frac {2 \pi K \omega_ {A}}{\omega_ {\mathrm{d}}} - \frac {\pi}{4}\right) = \frac {1}{\pi} \sqrt {\frac {\omega_ {\mathrm{d}}}{K \omega_ {A}}} \cos \left(\frac {2 \pi K \omega_ {A}}{\omega_ {\mathrm{d}}} - \frac {\pi}{4}\right)
$$

$$
\delta \Psi = 2 \pi \frac {2 \pi}{\omega_ {\mathrm{d}}} K \omega_ {L} \frac {1}{\pi} \sqrt {\frac {\omega_ {\mathrm{d}}}{K \omega_ {A}}} \cos \left(\frac {2 \pi K \omega_ {A}}{\omega_ {\mathrm{d}}} - \frac {\pi}{4}\right) = 4 \pi \omega_ {L} \sqrt {\frac {K}{\omega_ {A} \omega_ {\mathrm{d}}}} \cos \left(\frac {2 \pi K \omega_ {A}}{\omega_ {\mathrm{d}}} - \frac {\pi}{4}\right)
$$

假设抖振速率的幅值含有随机干扰 $W _ { A }$ 则上式改写成

$$
\delta \Psi = 4 \pi \omega_ {L} \sqrt {\frac {K}{\omega_ {A} \omega_ {\mathrm{d}}}} \cos \left(\frac {2 \pi K \omega_ {A}}{\omega_ {\mathrm{d}}} - \frac {\pi}{4} + \frac {2 \pi K W _ {A}}{\omega_ {\mathrm{d}}}\right)
$$

为简化分析，式中忽略了根式分母中 $W _ { A }$ 的影响。记。 $\theta { = } \frac { 2 \pi K W _ { A } } { \omega _ { \mathrm { d } } }$ .并假设随机相位。在 $[ - \pi , \pi ]$ 内服从均匀分布，则。的概率密度为

$$
f (\theta) = \left\{ \begin{array}{l l} \frac {1}{2 \pi}, & - \pi \leqslant \theta \leqslant \pi \\ 0, & \text {其余} \theta \text {值} \end{array} \right.
$$

所以 'l'的均值为

$$
E [ \delta \Psi ] = 4 \pi \omega_ {L} \sqrt {\frac {K}{\omega_ {A} \omega_ {\mathrm{d}}}} \cdot \frac {1}{2 \pi} \int_ {- \pi} ^ {\pi} \cos \left(\frac {2 \pi K \omega_ {A}}{\omega_ {\mathrm{d}}} - \frac {\pi}{4} + \theta\right) \mathrm{d} \theta = 0
$$

'l'的方差为

$$
E \left[ \delta \Psi^ {2} \right] = \frac {1 6 \pi^ {2} \omega_ {L} ^ {2} K}{\omega_ {A} \omega_ {\mathrm{d}}} \cdot \frac {1}{2 \pi} \int_ {\pi} ^ {- \pi} \left[ \frac {1}{2} + \frac {1}{2} \cos \left(\frac {4 \pi K \omega_ {A}}{\omega_ {\mathrm{d}}} - \frac {\pi}{2} + 2 \theta\right) \right] \mathrm{d} \theta = \frac {8 \pi^ {2} \omega_ {L} ^ {2} K}{\omega_ {A} \omega_ {\mathrm{d}}}
$$

所以在一个抖振周期 $T _ { \mathrm { d } }$ 内的均方根为

$$
\sigma_ {\delta \Psi} = 2 \sqrt {2} \pi \omega_ {L} \sqrt {\frac {K}{\omega_ {A} \omega_ {\mathrm{d}}}}
$$

在一个抖振周期内的正半周和负半周内的均方根和方差分别为

$$
\sigma_ {\delta \Psi} ^ {\prime} = \sqrt {2} \pi \omega_ {L} \sqrt {\frac {K}{\omega_ {A} \omega_ {\mathrm{d}}}}\tag{2.5.10}
$$

$$
\sigma_ {\delta \Psi} ^ {' 2} = \frac {2 \pi^ {2} \omega_ {L} ^ {2} K}{\omega_ {A} \omega_ {\mathrm{d}}}\tag{2.5.11}
$$

设对陀螺的测试时间为 $T$ 内包含的抖振半周期个数为 $\frac { 2 T } { T _ { \mathrm { d } } } { = } \frac { T \omega _ { \mathrm { d } } } { \pi }$ ，所以在 'P的方差为

$$
\sigma_ {\Delta \Psi} ^ {2} = \frac {2 \pi^ {2} \omega_ {L} ^ {2} K}{\omega_ {A} \omega_ {\mathrm{d}}} \cdot \frac {T \omega_ {\mathrm{d}}}{\pi} = \frac {2 \pi K \omega_ {L} ^ {2} T}{\omega_ {A}}\tag{2.5.12}
$$

均方根为

$$
\sigma_ {\Delta \Psi} = \omega_ {L} \sqrt {\frac {2 \pi K T}{\omega_ {A}}}\tag{2.5.13}
$$

由式(2. 5. 2b)知，当拍频输出有误差 $\delta f$ 时，角速度测量误差为

$$
\delta \omega = \frac {\delta f}{K} = \frac {2 \pi \delta f}{2 \pi K}
$$

对上式在[O T]内积分，得

$$
\int_ {0} ^ {T} \delta \omega \mathrm{d} t = \frac {1}{2 \pi K} \int_ {0} ^ {T} 2 \pi \delta f \mathrm{d} t
$$

即

$$
\Delta \varPhi = \frac {1}{2 \pi K} \Delta \varPsi
$$

式中，c $\Delta \varPhi$ 为陀螺的角增量测量误差，ð.'P为两束光的相位差误差，由此

$$
\sigma_ {\Delta \Phi} = \frac {1}{2 \pi K} \sigma_ {\Delta \Psi} = \omega_ {L} \sqrt {\frac {T}{2 \pi K \omega_ {A}}}\tag{2.5.14}
$$

上式说明在 $[ 0 , T _ { - } ^ { - }$ 内角增量测量误差的方差与 成正比，对照卡尔曼滤波理论中对基本有色噪声的介绍，此种误差为随机游走(秦永元，1998) ，常用游走系数来描述其统计特性

$$
\sqrt {Q _ {W}} = \frac {\sigma_ {\Delta \Phi}}{\sqrt {T}} = \frac {\omega_ {L}}{\sqrt {2 \pi K \omega_ {A}}}\tag{2.5.15}
$$

由于随机游走是自噪声的积分，而角增量是角速度的积分，所以反映在角增量输出中的随机游走误差与陀螺漂移中的自噪声(不相关误差)本质上是同一种误差，区别仅仅为前者是后者的积分，式(2.5.15)是自噪声的方差强度的平方根。

## 3. 激光陀螺的其余误差

## 1)陀螺漂移的零偏重复性误差

与机械式转子陀螺相同，激光陀螺漂移具有确定的偏置量，是可以补偿的误差.但该偏置量并非完全固定，陀螺每次启动测得的偏置量都不相同，补偿的只是平均值，所以重复性误差(有时也称逐次启动漂移)是零均值的随机常数

$$
\dot {\pmb {\varepsilon}} _ {b} = 0\tag{2.5.16}
$$

## 2) 陀螺漂移的零偏稳定性误差

陀螺漂移补偿固定偏置量后，剩下的漂移是随时间不断变化的。所以除了上述用随机常数描述的分量之外，还有随时间变化的分量，该分量反映了零偏的稳定性，常用相关模型如一阶马尔可夫过程描述

$$
\dot {\varepsilon} _ {r} = - \frac {1}{\tau} \varepsilon_ {r} + W\tag{2.5.17}
$$

影响零偏的因素有

(1)反射镜光轴的稳定性和反射镜表面的粗糙程度

(2) 谐振腔内杂质气体的含量

(3) 石英材料的膨胀，收缩和弯曲变形引起光路的变化，为补偿这种变化，需对光路作精密控制;

(4) 维持激光谐振的控制电流存在误差，控制电流误差一般应小于 50 毫微安。

3) 热漂移误差及补偿

## (1)热漂移补偿模型

由于激光陀螺石英块材料的导热率低，温度梯度大，加温后陀螺特性反而会下降，所以激光陀螺一般不加温控，而采用温度建模和软件补偿的办法.

设激光陀螺的初始温度为 $T _ { 0 }$ 陀螺启动时刻的温度) ，当前温度为 ，则陀螺漂移的当前零偏补偿值为

$$
L = a _ {0} + a _ {1} T + a _ {2} T ^ {2}\tag{2.5.18}
$$

式中，温补系数由下式确定

$$
a _ {0} = A _ {0 0} + A _ {0 1} T _ {0} + A _ {0 2} T _ {0} ^ {2}\tag{2.5.19a}
$$

$$
a _ {1} = A _ {1 0} + A _ {1 1} T _ {0} + A _ {1 2} T _ {0} ^ {2}\tag{2.5.19b}
$$

$$
a _ {2} = A _ {2 0} + A _ {2 1} T _ {0} + A _ {2 2} T _ {0} ^ {2}\tag{2.5.19c}
$$

系数 $A _ { 0 0 } , A _ { 0 1 } , \cdots , A _ { 2 2 }$ 是预先标定的常系数，下面介绍这些常系数的标定方法。

## (2) 温补系数的确定

标定必须在高低温箱内进行。在要求的工作温度范围内均匀选取 个初始温度点 $T _ { 0 i } ( i$ =$1 , 2 , \cdots , I )$ ，在 $T _ { 0 i }$ 至最高工作温度范围内均匀选取K; 个工作温度点 $T _ { i k } ( k { = } 1 , 2 , \cdots , K _ { i } )$ ，下标和品分别表示对应于初始温度 $T _ { 0 i }$ 的第 个工作温度点。

设在工作温度点 $T _ { i k }$ 上测得的陀螺漂移零偏为 $L _ { i k }$ ，则据式 (2. 5. 18)

$$
L _ {i k} = a _ {0} (i) + a _ {1} (i) T _ {i k} + a _ {2} (i) T _ {i k} ^ {2}
$$

对 $K _ { i }$ 个工作温度点，即 $k = 1 , 2 , \cdots , K _ { i }$ ，有

$$
\begin{array}{c} L _ {i 1} = a _ {0} (i) + a _ {1} (i) T _ {i 1} + a _ {2} (i) T _ {i 1} ^ {2} \\ L _ {i 2} = a _ {0} (i) + a _ {1} (i) T _ {i 2} + a _ {2} (i) T _ {i 2} ^ {2} \\ \vdots \\ L _ {i K _ {i}} = a _ {0} (i) + a _ {1} (i) T _ {i K _ {i}} + a _ {2} (i) T _ {i K _ {i}} ^ {2} \end{array}
$$

对于 个初始温度，即 $i = 1 , 2 , \cdots , I$ 组方程。

时，有

$$
\left[ \begin{array}{c} L _ {1 1} \\ L _ {1 2} \\ \vdots \\ L _ {1 K _ {1}} \end{array} \right] = \left[ \begin{array}{c c c} 1 & T _ {1 1} & T _ {1 1} ^ {2} \\ 1 & T _ {1 2} & T _ {1 2} ^ {2} \\ \vdots & \vdots & \vdots \\ 1 & T _ {1 K _ {1}} & T _ {1 K _ {1}} ^ {2} \end{array} \right] \left[ \begin{array}{c} a _ {0} (1) \\ a _ {1} (1) \\ a _ {2} (1) \end{array} \right]
$$

即 $\pmb { L } _ { 1 } = \pmb { h } _ { 1 } \pmb { a } _ { 1 }$ ，所以

$$
\left[ \begin{array}{c c c} \hat {a} _ {0} (1) & \hat {a} _ {1} (1) & \hat {a} _ {2} (1) \end{array} \right] ^ {\mathrm{T}} = \hat {\boldsymbol {a}} _ {1} = (\boldsymbol {h} _ {1} ^ {\mathrm{T}} \boldsymbol {h} _ {1}) ^ {- 1} \boldsymbol {h} _ {1} ^ {\mathrm{T}} \boldsymbol {L} _ {1}\tag{2.5.20}
$$

i=2 时，有

$$
\left[ \begin{array}{c} L _ {2 1} \\ L _ {2 2} \\ \vdots \\ L _ {2 K _ {2}} \end{array} \right] = \left[ \begin{array}{c c c} 1 & T _ {2 1} & T _ {2 1} ^ {2} \\ 1 & T _ {2 2} & T _ {2 2} ^ {2} \\ \vdots & \vdots & \vdots \\ 1 & T _ {2 K _ {2}} & T _ {2 K _ {2}} ^ {2} \end{array} \right] \left[ \begin{array}{c} a _ {0} (2) \\ a _ {1} (2) \\ a _ {2} (2) \end{array} \right]
$$

即 $\pmb { L } _ { 2 } = \pmb { h } _ { 2 } \pmb { a } _ { 2 }$ 所以

$$
\left[ \begin{array}{c c c} \hat {a} _ {0} (2) & \hat {a} _ {1} (2) & \hat {a} _ {2} (2) \end{array} \right] ^ {\mathrm{T}} = \hat {\boldsymbol {a}} _ {2} = (\boldsymbol {h} _ {2} ^ {\mathrm{T}} \boldsymbol {h} _ {2}) ^ {- 1} \boldsymbol {h} _ {2} ^ {\mathrm{T}} \boldsymbol {L} _ {2}\tag{2.5.21}
$$

i=I 时，有

$$
\left[ \begin{array}{c} L _ {I 1} \\ L _ {I 2} \\ \vdots \\ L _ {I K _ {I}} \end{array} \right] = \left[ \begin{array}{c c c} 1 & T _ {I 1} & T _ {I 1} ^ {2} \\ 1 & T _ {I 2} & T _ {I 2} ^ {2} \\ \vdots & \vdots & \vdots \\ 1 & T _ {I K _ {I}} & T _ {I K _ {I}} ^ {2} \end{array} \right] \left[ \begin{array}{c} a _ {0} (I) \\ a _ {1} (I) \\ a _ {2} (I) \end{array} \right]
$$

即 ${ \pmb { L } } _ { I } = { \pmb { h } } _ { I } { \pmb { a } } _ { I }$ "所以

$$
\left[ \begin{array}{c c c} \hat {a} _ {0} (I) & \hat {a} _ {1} (I) & \hat {a} _ {2} (I) \end{array} \right] ^ {\mathrm{T}} = \hat {\boldsymbol {a}} _ {I} = (\boldsymbol {h} _ {I} ^ {\mathrm{T}} \boldsymbol {h} _ {I}) ^ {- 1} \boldsymbol {h} _ {I} ^ {\mathrm{T}} \boldsymbol {L} _ {I}\tag{2.5.22}
$$

(3) 温补系数拟合常系数的确定

根据式(2. 5. 19) ，对应于初始温度 $T _ { 0 i }$

$$
\begin{array}{l} a _ {0} (i) = A _ {0 0} + A _ {0 1} T _ {0 i} + A _ {0 2} T _ {0 i} ^ {2} \\ a _ {1} (i) = A _ {1 0} + A _ {1 1} T _ {0 i} + A _ {1 2} T _ {0 i} ^ {2} \\ a _ {2} (i) = A _ {2 0} + A _ {2 1} T _ {0 i} + A _ {2 2} T _ {0 i} ^ {2} \end{array}
$$

对应于 $i = 1 , 2 , \cdots , I$ 将关于 $a _ { 0 } \left( 1 \right) , a _ { 0 } \left( 2 \right) , \cdots , a _ { 0 } \left( I \right)$ 个方程写在一起

$$
\begin{array}{c} a _ {0} (1) = A _ {0 0} + A _ {0 1} T _ {0 1} + A _ {0 2} T _ {0 1} ^ {2} \\ a _ {0} (2) = A _ {0 0} + A _ {0 1} T _ {0 2} + A _ {0 2} T _ {0 2} ^ {2} \\ \vdots \\ a _ {0} (I) = A _ {0 0} + A _ {0 1} T _ {0 I} + A _ {0 2} T _ {0 I} ^ {2} \end{array}
$$

用式 (2.5.20)- (2.5.22)求得的 ${ \hat { a } } _ { 0 } \left( 1 \right) , { \hat { a } } _ { 0 } \left( 2 \right) , \cdots , { \hat { a } } _ { 0 } \left( I \right)$ 代替 $a _ { 0 } \left( 1 \right) , a _ { 0 } \left( 2 \right) , \cdots , a _ { 0 } \left( I \right)$ 并将上式写成矩阵形式

$$
\left[ \begin{array}{c} \dot {a} _ {0} (1) \\ \dot {a} _ {0} (2) \\ \vdots \\ \dot {a} _ {0} (I) \end{array} \right] = \left[ \begin{array}{c c c} 1 & T _ {0 1} & T _ {0 1} ^ {2} \\ 1 & T _ {0 2} & T _ {0 2} ^ {2} \\ \vdots & \vdots & \vdots \\ 1 & T _ {0 I} & T _ {0 I} ^ {2} \end{array} \right] \left[ \begin{array}{c} A _ {0 0} \\ A _ {0 1} \\ A _ {0 2} \end{array} \right]
$$

即 $\hat { \pmb { a } } _ { 0 } { = } \pmb { H } \pmb { A } _ { 0 }$ 所以

$$
\left[ \begin{array}{c c c} \hat {A} _ {0 0} & \hat {A} _ {0 1} & \hat {A} _ {0 2} \end{array} \right] ^ {\mathrm{T}} = \hat {\boldsymbol {A}} _ {0} = (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {H}) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \hat {\boldsymbol {a}} _ {0}\tag{2.5.23}
$$

将关于向 $a _ { 1 } ( 1 ) , a _ { 1 } ( 2 ) , \cdots , a _ { 1 } ( I )$ 个方程写在一起，用 ${ \hat { a } } _ { 1 } ( 1 ) , { \hat { a } } _ { 1 } ( 2 ) , \cdots , { \hat { a } } _ { 1 } ( I )$ )代替 $a _ { 1 } \left( 1 \right)$ $a _ { 1 } ( 2 ) , \cdots , a _ { 1 } ( I )$ ，并写成矩阵形式

$$
\left[ \begin{array}{c} \hat {a} _ {1} (1) \\ \hat {a} _ {1} (2) \\ \vdots \\ \hat {a} _ {1} (I) \end{array} \right] = \left[ \begin{array}{c c c} 1 & T _ {0 1} & T _ {0 1} ^ {2} \\ 1 & T _ {0 2} & T _ {0 2} ^ {2} \\ \vdots & \vdots & \vdots \\ 1 & T _ {0 I} & T _ {0 I} ^ {2} \end{array} \right] \left[ \begin{array}{c} A _ {1 0} \\ A _ {1 1} \\ A _ {1 2} \end{array} \right]
$$

即 $\hat { \pmb { a } } _ { 1 } = \pmb { H } \pmb { A } _ { 1 }$ 所以

$$
\left[ \begin{array}{c c c} \hat {A} _ {1 0} & \hat {A} _ {1 1} & \hat {A} _ {1 2} \end{array} \right] ^ {\mathrm{T}} = \hat {\boldsymbol {A}} _ {1} = (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {H}) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \hat {\boldsymbol {a}} _ {1}\tag{2.5.24}
$$

将关于向 $a _ { 2 } ( 1 ) , a _ { 2 } ( 2 ) , \cdots , a _ { 2 } ( I )$ 个方程写在一起，用 ${ \hat { a } } _ { 2 } \left( 1 \right) , { \hat { a } } _ { 2 } \left( 2 \right) , \cdots , { \hat { a } } _ { 2 } \left( I \right)$ 代替 $a _ { 2 } \left( 1 \right)$ $a _ { 2 } \left( 2 \right) , \cdots , a _ { 2 } \left( I \right)$ ，并写成矩阵形式

$$
\left[ \begin{array}{c} \dot {a} _ {2} (1) \\ \dot {a} _ {2} (2) \\ \vdots \\ \dot {a} _ {2} (I) \end{array} \right] = \left[ \begin{array}{c c c} 1 & T _ {0 1} & T _ {0 1} ^ {2} \\ 1 & T _ {0 2} & T _ {0 2} ^ {2} \\ \vdots & \vdots & \vdots \\ 1 & T _ {0 I} & T _ {0 I} ^ {2} \end{array} \right] \left[ \begin{array}{c} A _ {2 0} \\ A _ {2 1} \\ A _ {2 2} \end{array} \right]
$$

即 $\hat { \pmb { a } } _ { 2 } = { \pmb { H } } \mathbf { A } _ { 2 }$ 所以

$$
\left[ \begin{array}{c c c} \hat {A} _ {2 0} & \hat {A} _ {2 1} & \hat {A} _ {2 2} \end{array} \right] ^ {\mathrm{T}} = \hat {\boldsymbol {A}} _ {2} = (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {H}) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \hat {\boldsymbol {a}} _ {2}\tag{2.5.25}
$$

## 4) 磁致漂移误差

磁场能引起线偏振光的畸变，而激光陀螺工作环境中的磁场有地磁场、仪器仪表及磁性物质产生的磁场，因此，激光陀螺必须采用磁屏蔽技术。无磁屏蔽陀螺的磁致漂移可达 0.04 /(h/Gs) ，适当屏蔽后可降到原来的 1/60

## 5) 温度系数误差

激光陀螺的刻度系数即为脉冲当量，即一个脉冲输出对应的角增量。刻度系数误差从数个 ppm①至数十个 ppm不等，视陀螺质量高低而定。

如果要去掉刻度系数中的温度影响，可按热补偿模型作软件补偿

$$
K = K _ {0} + K _ {1} (T _ {p} - T _ {r})\tag{2.5.26}
$$

式中， $T _ { r }$ ，和 $T _ { \mathbf { \nabla } \phi }$ 分别为基准参考温度和当前温度 $K _ { 0 }$ 为 $T _ { r }$ ，条件下的刻度系数 $K _ { 1 }$ 为补偿系数。

## 6) 温度系数的不对称误差

刻度系数不对称误差定义为

$$
\Delta K _ {a s y} = \frac {K _ {+} - K _ {-}}{2}
$$

式中 $K _ { + }$ 为正向脉冲当量， 为负向脉冲当量。

## 2.6 光纤陀螺简介

20世纪 70 年代，在电信应用的推动下，低损耗光纤、固态半导体光源和探测器的研发取得了巨大成就，用多匮光纤线圈代替环形激光器，通过多次循环来增强萨格奈克效应已有物质基础， 1967 Pircher Hepner 提出了光纤陀螺， 1976 Vali Shorthill 进行了实验演示 (ValiV et al. 1976) ，立即受到惯性技术界的高度重视。经过 20 多年的努力，光纤陀螺已形成了从$1 0 ^ { \circ } / \mathrm { h } { \sim } 0 . 0 1 ^ { \circ } .$ /h精度覆盖面较宽的系列产品，特别是中低精度 $1 0 ^ { \circ } / \mathrm { h } { \sim } 0 . 1 ^ { \circ } / \mathrm { h }$ 的产品，以其成本低，MTBF 高，体积重量小，功耗低等独特的优势，在战术武器制导和民航飞机导航系统备份应用中具有重要地位。

光纤陀螺就其工作原理实质上是单模光纤环构成的萨格奈克干涉仪，其光学原理如图 2.6.1 所示。激光器发出的光束经半透半反分光镜进入多臣光纤线圈的两端，两束光在光纤内的传播方向相反。若光纤环相对惯性空间静止，则两束反向传播的光束到达接收器时具有相同的相位。但若光纤环相对惯性空间有垂直于光纤环平面的角速度 时，两束光的传播光程将发生变化，根据萨格奈克干涉仪给出的关系

$$
\Delta L = \frac {4 N A}{c} \omega
$$

![](images/a18c92ed241b243538afd5a64757af42a0772527d0f379023c611951c32af13a.jpg)

6. 1)

式中， 为光纤环的绕制圈数 为一圈光纤所包围的酬，对圆柱形光纤环 $. A { = } \frac { \pi D ^ { 2 } } { 4 }$ 为光纤环直径。

2.6.1 光纤陀螺光学原理图

将式(2.6. 1)写成相位差形式

$$
\Delta \phi = \frac {2 \pi L D}{c \lambda} \omega\tag{2.6.2}
$$

式中， 为光纤长度， 为光源波长。由于光纤长度非常长，可达 100\~1000m (Herve C, 2002) ,所以用相位差测量角速度仍具有很高的灵敏度，与此不同的是激光陀螺根据拍频测量角速度。显然光纤陀螺是通过增加光纤臣数以增大光路所围的面积，提高陀螺的灵敏度，使萨格奈克干涉仪可用于工程实际。此外，与谐振腔激光陀螺相比，光纤陀螺不存在低角速度输入时的闭锁效应问题。

随着光学技术的发展，光纤陀螺也由分立光学元件逐渐向集成光学元件结构发展，国际上已出现了低漂移光纤陀螺、零相位光纤陀螺、、全集成光纤陀螺与数字式数据处理光纤陀螺等。

