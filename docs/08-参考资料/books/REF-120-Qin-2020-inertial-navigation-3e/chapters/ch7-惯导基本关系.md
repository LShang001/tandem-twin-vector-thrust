# 第7章 惯导系统中的基本关系

## 第7章 惯导系统中的基本关系

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

将式 (7.2.4) 代入式 (7.2.3) ，得

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

将式 (7.2.4b) 代入，得

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

$g _ { e }$ 为赤道上的理论重力加速度， $g _ { \boldsymbol { \rho } }$ 为两极处的理论重力加速度 $e _ { 1 }$ 为第一偏心率。将 WGS-84的具体数据代入，得

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

将式 (7.3.20) 代入式 (7.3.22) ，得

$$
\frac {K _ {a} K _ {u} K _ {m}}{H} = 1\tag{7.3.23}
$$

上述分析说明 设计单轴惯性平台时，只要满足式(7. 3. 23) ，就可使平台具有 84.4 分钟的振荡周期，实现休拉调谐。

三轴平台是单轴平台的复合，各通道间耦合是很弱的，俯仰环和横滚环的休拉调谐都可按单轴平台分析和设计。

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

将式 (7.4.3) 代入式 (7.4.2) ，得

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

