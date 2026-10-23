# 第8章 卡尔曼滤波理论在组合导航系统设计中的应用

# 第八章 卡尔曼滤波理论在组合导航系统设计中的应用

## 8.1 概述

将航行体从起始点导引到目的地的技术或方法称为导航。能够向航行体的操纵者或控制系统提供航行体的位置、速度、航向、姿态等即时运动状态的系统都可作为导航系统。飞机通常采用的导航系统有：惯性导航系统、GPS导航系统、多普勒导航系统、双曲线无线电导航系统、VOR-DME等。所有导航系统都具有各自独特的优点，但单独使用时都存在一定的缺陷。下面以惯性导航系统和GPS导航系统为例来说明之。

惯性导航系统根据惯性原理工作，而惯性是任何质量体的基本属性，所以惯导系统工作时不需要任何外来信息，也不向外辐射任何信息，仅靠系统本身就能在全天候条件下，在全球范围内和任何介质环境里自主地、隐蔽地进行连续的三维空间定位和三维空间定向，能够提供反映航行体完整运动状态的完整信息。惯导系统具有极宽的频带，它能够跟踪和反映航行体的任何机动运动，输出又非常平稳。正由于惯导系统的自主性、隐蔽性、信息的全面性和宽频带，所以是重要航行体（如潜艇、洲际导弹、宇宙飞船、远程飞机等）必不可少的导航设备。但是惯导系统的导航误差随时间而积累，这对于续航时间长的航行体来说无疑是致命的缺陷。

GPS 导航系统根据接收到的导航卫星信号解算出航行体的位置和速度, 其误差是有界的, 具有很好的长期稳定性, 但也存在着诸多缺陷:

(1) GPS 系统由美国国防部直接控制,使用权受制于人,是不能绝对依赖的系统。

（2）除美国国防部的特许用户外，所有用户都只能使用 C/A 码信号进行导航计算，自 1993 年 GPS 的 24 颗导航卫星全部部署完毕起，美国国防部为防止 C/A 码信号被用于战略武器系统，专门在 C/A 码信号内加入了 SA 误差（Selective Availability，意在警告人们 C/A 码信号的适用范围是有选择性的，而并非对任何应用场合都适用），定位误差由数十米增加到数百米。此外，由于 GPS C/A 码接收机的环路带宽不能同时满足抗干扰性能和动态跟踪性能之间的矛盾要求，所以接收机的动态跟踪能力一般较低，当航行体作大机动运动时，环路极易失锁，此时，GPS 信号会完全丢失。

从上述简单介绍可看出,惯性导航系统和 GPS 导航系统各有优缺点,但在误差传播性能上正好是互补的,前者长期稳定性差,但短期稳定性好,而后者正好相反。所以可采用组合导航技术将这些性能各异的不同导航系统有机地组合起来,以提高导航系统的整体性能。

组合导航技术是指使用两种或两种以上的不同导航系统对同一信息源作测量，从这些测量值的比较值中提取出各系统的误差并校正之。采用组合导航技术的系统称组合导航系统，参与组合的各导航系统称子系统。由于惯导系统具有自主性、隐蔽性、信息的全面性和宽频带等特有优点，所以一般都以惯导系统作为组合导航系统的关键子系统。又由于惯导系统和GPS导航系统性能互补，所以，以该两子系统构造出组合导航系统是航空导航的最佳方案。

实现组合导航有两种基本方法如下：

（1）回路反馈法，即采用经典的控制方法，抑制系统误差，并使子系统间性能互补；

（2）最优估计法，即采用现代控制理论中的最优估计法（常采用卡尔曼滤波或维纳滤波），从概率统计最优的角度估算出系统误差并消除之。

两种方法都使各子系统间的信息互相渗透,起到性能互补的功效。但由于各子系统的误差源和量测中引入的误差都是随机的,所以第二种方法远优于第一种方法。

组合导航系统一般具有如下功能：

（1）协合超越功能：利用各子系统的导航信息并作有机处理，形成单个子系统不具备的功能和精度。

（2）互补功能：组合导航系统综合利用了各子系统的信息，使各子系统取长补短，扩大使用范围。

（3）余度功能：各子系统感测同一信息源，测量值冗余，增加了导航系统的可靠性。

组合导航系统的发展方向是容错组合导航系统,这种系统具有故障检测、诊断、隔离和系统重构的功能。

## 8.2 组合导航系统的设计模式

## 8.2.1 状态和量测的选取——直接法和间接法

当设计组合导航系统的卡尔曼滤波器时,首先必须列写出描述系统动态特性的系统方程和反映量测与状态关系的量测方程。如果直接以各导航子系统的导航输出参数作为状态,即直接以导航参数作为估计对象,则称实现组合导航的滤波处理为直接法滤波。如果以各子系统的误差量作为状态,即以导航参数的误差量作为估计对象,则称实现组合导航的滤波处理为间接法滤波。

直接法滤波中,卡尔曼滤波器接收各导航子系统的导航参数,经过滤波计算,得到导航参数的最优估计,如图8.2.1所示。

![](images/95df1d5602ab72f91ff26e9a1064d4b8e73c0d1e7fdb11d631939e243aefa041.jpg)  
图 8.2.1 直接法滤波示意图

间接法滤波中，卡尔曼滤波器接收两个导航子系统对同一导航参数输出值的差值，经过滤波计算，估计出各误差量。用惯导系统误差的估计值去校正惯导系统输出的导航参数，以得到导航参数的最优估计；或者用惯导系统误差的估计值去校正惯导系统力学编排中的相应导航参数，即将误差估计值反馈到惯导系统的内部，如图8.2.2所示。前者称为输出校正，后者称为反馈校正。

![](images/229f94e859f10d410143dd1adfddd1bd9f1ade6df4c1c3d03e5e8ad5887b5e50.jpg)  
图8.2.2 间接法滤波示意图

直接法滤波和间接法滤波各有优缺点,综合起来主要体现在以下几方面:

（1）直接法的模型系统方程直接描述系统导航参数的动态过程，它能较准确地反映真实状态的演变情况；间接法的模型系统方程是误差方程，它是按一阶近似推导出来的，有一定的近似性。

（2）直接法的模型系统方程是惯导力学编排方程和某些误差变量方程(例如平台倾角)的综合。滤波器既能达到力学编排方程解算导航参数的目的,又能起到滤波估计的作用。滤波器输出的就是导航参数的估计以及某些误差量的估计。因此,采用直接法可使惯导系统避免力学编排方程的许多重复计算。但如果组合导航在转换到纯惯导工作方式时,惯导系统不用卡尔曼滤波。这时,还须要另外编排一套程序解算力学编排方程,这是不便之处。而间接法却相反,虽然系统须要分别解算力学编排方程和滤波计算方程,但在程序上也便于由组合导航方式向纯惯导方式转换。

（3）两种方法的系统方程有相同之处。状态中都包括速度(或速度误差)、位置(或位置误差)和平台误差角。但是，它们最大的区别在于直接法的速度状态方程中包括计算坐标系相应轴向的比力量测值以及由于平台有倾角而产生的其他轴向比力的分量，而间接法的速度误差方程中只包括其他轴向比力的分量。比力量测值主要是运载体运动的加速度，它主要受运载体推力的控制，也受运载体姿态和外界环境干扰的影响。因此，它的变化比速度要快得多。为了得到准确的估计，卡尔曼滤波的计算周期必须很短，这对计算机计算速度提出了较高的要求，而间接法却没有这种要求。根据有关文献介绍，间接法量测值的采样周期(一般也是滤波的计算周期)在几秒到一分钟的范围内，基本上不影响滤波器的有效性能。

（4）直接法的系统方程一般都是非线性方程，卡尔曼滤波必须采用广义滤波。而间接法的系统方程都是线性方程，可以采用基本滤波方程。

（5）间接法的各个状态都是误差量，相应的数量级是相近的。而直接法的状态，有的是导航参数本身，如速度和位置，有的却是数值很小的误差，如姿态误差角，数值相差很大，这给数值计算带来一定的困难，且影响这些误差估计的准确性。

综上所述，虽然直接法能直接反映出系统的动态过程，但在实际应用中却还存在着不少困难。只有在空间导航的惯性飞行阶段，或在加速度变化缓慢的舰船中，惯导系统的卡尔曼滤波才采用直接法。对没有惯导系统的组合导航系统，如果系统方程中不需要速度方程，也可以采用直接法，而在飞行器的惯导系统中，目前一般都采用间接法的卡尔曼滤波。

## 8.2.2 输出校正和反馈校正

从卡尔曼滤波器得到的估计有两种利用方法:一种是将估计作为组合导航系统的输出,或作为惯导系统输出的校正量,这种方法称为开环法;另一种是将估计反馈到惯导系统和其余子系统中,估计出的导航参数就作为惯导力学编排方程中的相应参数,估计出的误差作为校正量,将惯导系统或其他导航设备中的相应误差量校正掉,这种方法称为闭环法。从直接法和间接法得到的估计都可以采用开环法和闭环法进行校正。间接法估计的都是误差量,这些估计是作为校正量来利用的。因此,间接法中的开环法也称为输出校正,闭环法也称为反馈校正。这就是图8.2.2所示的内容。下面分析输出校正和反馈校正这两种方法的特点。

设将滤波估计作为输出校正的系统方程和量测方程分别为

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {F} (t) \boldsymbol {X} (t) + \boldsymbol {G} (t) \boldsymbol {w} (t)\tag{8.2.1}
$$

$$
\mathbf {Z} (t) = \mathbf {H} (t) \mathbf {X} (t) + \boldsymbol {\nu} (t)\tag{8.2.2}
$$

式中各符号的意义和噪声统计特性描述都与第二章所述相同， $X(t)$ 为导航系统的误差状态。
根据式(2.3.40)～式(2.3.42)，滤波方程为

$$
\dot {\hat {\mathbf {X}}} (t) = \mathbf {F} (t) \hat {\mathbf {X}} (t) + \overline {{\mathbf {K}}} (t) [ \mathbf {Z} (t) - \mathbf {H} (t) \hat {\mathbf {X}} (t) ]\tag{8.2.3}
$$

$$
\bar {\boldsymbol {K}} (t) = \boldsymbol {P} (t) \boldsymbol {H} ^ {\mathrm{T}} (t) \boldsymbol {r} ^ {- 1} (t)\tag{8.2.4}
$$

$$
\dot {\boldsymbol {P}} (t) = \boldsymbol {P} (t) \boldsymbol {F} ^ {\mathrm{T}} (t) + \boldsymbol {F} (t) \boldsymbol {P} (t) - \boldsymbol {P} (t) \boldsymbol {H} ^ {\mathrm{T}} (t) \boldsymbol {r} ^ {- 1} (t) \boldsymbol {H} (t) \boldsymbol {P} (t) + \boldsymbol {G} (t) \boldsymbol {q} (t) \boldsymbol {G} ^ {\mathrm{T}} (t)\tag{8.2.5}
$$

$X(t)$ 的估计误差定义为

$$
\widetilde {\boldsymbol {X}} (t) = \boldsymbol {X} (t) - \hat {\boldsymbol {X}} (t)
$$

$\widetilde{X}(t)$ 就是惯导系统经过输出校正以后的状态。系统和滤波的和方块图见图 8.2.3。根据式(8.2.1)～式(8.2.3)， $\widetilde{X}(t)$ 的动态特性由下式描述：

$$
\dot {\hat {\mathbf {X}}} (t) = [ \mathbf {F} (t) - \bar {\mathbf {K}} (t) \mathbf {H} (t) ] \tilde {\mathbf {X}} (t) + \mathbf {G} (t) \mathbf {w} (t) - \bar {\mathbf {K}} (t) \mathbf {v} (t)\tag{8.2.6}
$$

![](images/c8eaf2c78fea431c7bb27b763339f4a8322fff72dbf6ea33d3a9ca40a9622226.jpg)  
图 8.2.3 输出校正方块图

如果采用反馈校正,并认为各状态都能控制,则系统方程为

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {F} (t) \boldsymbol {X} (t) + \boldsymbol {G} (t) \boldsymbol {w} (t) + \boldsymbol {u} (t)\tag{8.2.7}
$$

按式 $(5.5.3)$ ，反馈控制为

$$
\boldsymbol {u} (t) = - \bar {\boldsymbol {K}} (t) \boldsymbol {Z} (t)\tag{8.2.8}
$$

所以式 $(8.2.7)$ 为

$$
\dot {\boldsymbol {X}} (t) = \left[ \boldsymbol {F} (t) - \bar {\boldsymbol {K}} (t) \boldsymbol {H} (t) \right] \boldsymbol {X} (t) + \boldsymbol {G} (t) \boldsymbol {w} (t) - \bar {\boldsymbol {K}} (t) \boldsymbol {v} (t)\tag{8.2.9}
$$

系统和滤波器的方块图见图 8.2.4。 $X(t)$ 本身就是惯导系统经过反馈校正后的状态。

![](images/25ea9629e2e62086284c854436332febfcaeed8a9be5a3ae833ec4dce5126e47.jpg)  
图8.2.4 反馈校正方块图

比较这两种方法,可以得出以下特点:

（1）式(8.2.6)和式(8.2.9)除了状态变量不同外,其他参数完全相同,即这两种系统校正后的状态动态特性是完全一样的。因此,如果模型系统方程和量测方程能正确反映系统本身,则输出校正和反馈校正在本质上是一样的,即估计和校正的效果是一样的。对于离散系统,情况也是如此。

（2）输出校正利用滤波器的估计去校正惯导系统输出的导航参数，其作用是改善输出的准确性，而反馈校正是利用滤波器输出的控制量去校正式(8.2.7)所示系统的状态。虽然这些状态都是误差量，但对这些误差量作校正，实际上就是校正惯导系统的状态。因此，控制量用来校正与误差状态相应的状态。例如，速度误差的控制量实际上就是输入到惯导力学编排方程中去校正速度量。即：不论是输出校正还是反馈校正，实质上都是对导航参数进行校正。

（3）输出校正增益陈的计算方程式(8.2.4)和式(8.2.5)在形式上与反馈校正的增益阵计算方程完全一样，但式(8.2.7)和式(8.2.8)比式(8.2.3)简单，这是反馈校正方便之处。

(4) 比较式(8.2.1)和式(8.2.7)，虽然在形式上后者仅多了一项控制项，但两式中 $X(t)$ 的意义是不同的。式(8.2.1)中的 $X(t)$ 是系统未经输出校正的误差状态，而式(8.2.7)中的 $X(t)$ 是系统经过反馈校正的误差状态。当然，经过校正的 $X(t)$ 要比未经校正的 $X(t)$ 小。对这两个都经过一阶近似的误差状态方程来讲， $X(t)$ 越小，则近似的准确性就越高。因此，利用反馈校正的系统方程式(8.2.7)更能接近反映系统误差状态的真实动态过程。在一般情况下，输出校正要得到与反馈校正相同的估计精度，应该采用较为复杂的模型系统方程。

由于以上原因,惯导系统通常采用反馈校正。在系统中,如果实际控制量能达到的最大值小于式(8.2.8)中的 $\boldsymbol{u}(t)$ 可能达到的最大值,则可采用输出校正。对于离散系统,情况也是如此。

## 8.2.3 阻尼问题

在间接法滤波中,状态变量是误差量,所以量测量应该是这些误差量的线性函数,这只有当来自不同系统的同一导航参数作比较后才能做到这一点。这说明,在间接法滤波中,要实现对某一导航系统的阻尼,必须利用来自其他导航系统的外来信息,而不能利用自身信息作为阻尼。

而在直接法滤波中以实际物理量作为状态变量,如例2-13所讨论的问题,在此情况下,通过卡尔曼滤波处理,系统自身能够提供阻尼,而并不要求必须由外来信息提供阻尼。

## 8.3 组合导航系统设计中一些常用导航子系统的误差模型

在 8.2 节中已经说明,设计组合导航系统时一般都采用间接法滤波,而在间接法滤波中都以误差量作为状态。所以,当设计组合导航系统的卡尔曼滤波器时,必须具备的已知条件之一是参与组合的导航子系统的误差模型。误差模型描述了噪声激励源驱动之下误差的动态传播规律。本节介绍组合导航系统设计中一些常用导航子系统的工作原理和误差模型。由于惯导系统的工作原理和误差传播特性远比其余导航子系统复杂,为便于叙述,关于惯导系统的内容在下一节中单独讨论。

## 8.3.1 GPS 导航仪

GPS的全称为全球定位系统(Global Positioning System)，是美国国防部从20世纪70年代开始发展的以卫星为基础的无线电通讯导航系统，它能在全球范围内全天候地提供高精度的三维位置、三维速度和时间基准信息，具有静态定位、动态导航及精密授时的功能。GPS系统由三部分组成：空间部分(24颗卫星)、地面监测控制部分和用户设备部分(GPS接收机)。GPS信号采用码分址结构。码分P码和C/A码两种，它们都是伪随机二进制码。C/A码的周期为1ms，在一个周期内共有1023个码片；P码的周期为一星期，在一个周期内共有 $6187104\times 10^{6}$ 个码片。由于接收GPS信号时必须知道被接收信号的伪码结构，而P码的结构是绝密的，且破译P码的概率为零，所以只有美国国防部的特许用户才能使用P码信号，其余用户只能使用C/A码信号。习惯上将C/A码信号称为民用码信号，实现标准定位服务(SPS)。实际上C/A码信号并非为民用目的而设置，而是为使用P码信号而设置的初捕码，即捕获P码信号之前，必须先捕获C/A码，再过渡到P码信号，这样做的目的是要缩短对P码的搜索时间。由于C/A码在一个周期内的码片不多，容易被破译，所以美国国防部公开C/A码结构，具体体现在公开销售GPS C/A码接收机。GPS C/A码信号的原设计定位误差不超过 $50\mathrm{m}$ ，且由于GPS定位误差不随时间发散，所以GPS系统开始建立后立即引起了世界导航界的注意。美国国防部为了防止他国将GPS C/A码信号用于战略武器系统，于1993年部署完毕起故意加入了SA误差。由于GPS完全由美国国防部控制，所以，美国交通部制订标准明确指出GPS只能用作机载辅助导航设备[77]。

SA 通过两个入口引入到 GPS 信号中: 时钟频率的随机抖动和叠加在星历数据上的随机误差。

有很多种物理因素影响 GPS 的定位精度, 主要因素有:

（1）距地面20 200 km的卫星发射的电磁波必须穿透电离层和对流层才能到达地面，而电磁波在真空中和在电离层及对流层中的折射系数是不相同的，这意味着电磁波到达地面时将产生附加延时；

（2）卫星运行的线速度为4 km/s，这将引起时钟的相对论效应，使卫星钟和地面站钟不同步，此外还有时钟本身的走时误差，而GPS是根据相关接收中测定出的传播延时作定位计算的，所以时钟误差和相对论效应都直接影响定位误差；

（3）GPS 卫星的运行轨道参数主要由地球重力场决定，地球表面的不规则性、地球重力分布的不均匀性及地球内部熔液的活动，太阳和月球引力场的作用，太阳光辐射压力的作用，都

会引起星历误差。

但是与 SA 相比,这些非人为因素引起的定位误差只是 GPS 定位误差中的次要成分,而 SA 是最主要成分。

SA 体现在定位上是一种变化缓慢、相关强烈的非平稳过程 $^{[78]}$ ，图 8.3.1 示出了西安地区采集到的一组样本，其中点密的虚线为高度误差，点稀的虚线为纬度误差，实线为经度误差，经度和纬度误差已折算成沿东向和沿北向的距离误差。

消除或减小 SA 误差的途径有三种：

(1) 采用差分技术；

（2）通过时序分析和辨识对 SA 建模，将 GPS 导航仪的位置输出作为量测量，根据 SA 模型，估计出 SA $^{[79]}$ ，但由于缺少外来信息，估计效果受到限制；

（3）通过对 SA 实测样本作功率谱分析，获得 SA 的平稳线性模型，再从 GPS 导航仪与其余非相似导航子系统构成的量测量中，用卡尔曼滤波技术实时估计出各导航子系统的误差及 SA 的实时数据，估计

![](images/95b7e5a5057754ae0581cd61536ecb05fd97badd9ce9eaa98c927ee55ad02a91.jpg)  
图 8.3.1 西安地区的实测 SA 位置误差样本

效果体现在整个导航系统的综合精度的提高上。从整个时间过程来看，SA为非平稳过程，但对时间有限的导航来说，SA误差可近似看做平稳过程，即用平稳线性模型描述之 $^{[80]}$ 。

获得完全正确描述 SA 动力学特性的模型是很困难的，并且在组合导航系统设计过程中，SA 模型被列入卡尔曼滤波器状态，模型越复杂，用于描述其动力学特性的状态变量就越多，卡尔曼滤波的计算量就越大，实时计算就越难实现。鉴于上述原因，所建立的 SA 模型既要反映出 SA 主要的动力学特性，又要简单实用，阶数应该适当低。而 SA 在时间域内的表现形式是变化缓慢相关强烈，所以采用一阶马尔柯夫过程描述 SA 的动力学特性是合适的。

记反映在经度、纬度及高度上的 SA 位置误差分别为: $\delta\lambda_{SA}$ , $\delta L_{SA}$ , $\delta h_{SA}$ ，则位置误差模型为

$$
\delta \dot {\lambda} _ {\mathrm{SA}} = - \frac {1}{\tau_ {\mathrm{SA} \lambda}} \delta \lambda_ {\mathrm{SA}} + w _ {\mathrm{SA} \lambda}\tag{8.3.1}
$$

$$
\dot {\delta} \dot {L} _ {\mathrm{SA}} = - \frac {1}{\tau_ {\mathrm{SAL}}} \delta L _ {\mathrm{SA}} + w _ {\mathrm{SAL}}\tag{8.3.2}
$$

$$
\delta \dot {h} _ {\mathrm{SA}} = - \frac {1}{\tau_ {\mathrm{SAh}}} \delta h _ {\mathrm{SA}} + w _ {\mathrm{SAh}}\tag{8.3.3}
$$

式中相关时间 $\tau_{SA\lambda}$ ， $\tau_{SAL}$ ， $\tau_{SAh}$ 分别为 $100 \sim 200 \, s$ ， $\delta\lambda_{SA}$ 和 $\delta L_{SA}$ 的均方根约为 $0.02'$ ， $\delta h_{SA}$ 的均方根约为 $50 \, m$ ，驱动白噪声 $w_{SA\lambda}$ ， $w_{SAL}$ 和 $w_{SAh}$ 的方差强度按式(3.1.32)确定。

记反映沿东向,北向和天向的 SA 速度误差分别为: $\delta v_{ESA}$ , $\delta v_{NSA}$ , $\delta v_{USA}$ , 则速度误差模型为:

$$
\delta \dot {v} _ {\mathrm{ESA}} = - \frac {1}{\tau_ {\mathrm{SAvE}}} \delta v _ {\mathrm{ESA}} + w _ {\mathrm{SAvE}}\tag{8.3.4}
$$

$$
\dot {\delta v} _ {\mathrm{NSA}} = - \frac {1}{\tau_ {\mathrm{SA} v \mathrm{N}}} \delta v _ {\mathrm{NSA}} + w _ {\mathrm{SA} v \mathrm{N}}\tag{8.3.5}
$$

$$
\dot {\delta v _ {\mathrm{USA}}} = - \frac {1}{\tau_ {\mathrm{SAvU}}} \delta v _ {\mathrm{USA}} + w _ {\mathrm{SAvU}}\tag{8.3.6}
$$

其中相关时间 $\tau_{SAvE}$ ， $\tau_{SAvN}$ ， $\tau_{SAvU}$ 分别为 100～200 s，均方根分别为 0.1～0.2 m/s，驱动白噪声 $w_{SAvE}$ ， $w_{SAvN}$ ， $w_{SAvU}$ 的方差强度按式(3.1.32)确定。

## 8.3.2 多普勒导航系统

多普勒导航系统使用机载多普勒雷达测量飞机相对地球的速度，通过对地速的积分获得导航解。多普勒雷达的测速原理是多普勒效应。多普勒效应是指：当机械波或电磁波的发射源与接收点间沿两者连线方向存在相对速度时，接收频率与发射频率并不相同，这一频率差称为多普勒频移。多普勒频移与这一相对速度成正比，因此根据发射频率和多普勒频移就能求出这一相对速度。多普勒效应是在1842年由奥地利物理学家多普勒(Christian Doppler)在研究声学问题时，即机械波时首次发现的。1938年又证实了在电磁波领域内同样存在多普勒效应。

设无线电发射机的发射频率为 $f_{0}$ ，发射机以速度 v 向接收机运动，接收机固定在 B 点，如图 8.3.2 所示。

发射机在 $t_{0}$ 时刻位于 A 点，在 $t_{1}$ 时刻位于 $A_{1}$ 点，A 和 B 两点间的距离为 d。由于传播延迟，在 A 点发射的信号到达 B 点的时间为

![](images/b02cf9b8bfb9ea23f89db0b98295b8cc1199eb5b859a697154ea0dee22fdcc4b.jpg)  
图 8.3.2 发射机运动, 接收机固定时的多普勒频移

$$
T _ {0} = t _ {0} + \frac {d}{c}
$$

在 $A_{1}$ 点发射的信号到达 $B$ 点的时间为

$$
T _ {1} = t _ {1} + \frac {d - v \Delta t}{c}
$$

式中 c 为光速, $\Delta t=t_{1}-t_{0}$ 。

在 $\left[t_{0},t_{1}\right]$ 时间段内,发射机发出的信号波动周数为

$$
n = \Delta t f _ {0}
$$

设接收到的信号频率为 $f_{r}$ ，则接收到的波数为

$$
\begin{array}{r l} N & = (T _ {1} - T _ {0}) f _ {r} = \\ & \left[ t _ {1} + \frac {d - v \Delta t}{c} - \left(t _ {0} + \frac {d}{c}\right) \right] f _ {r} = \left(1 - \frac {v}{c}\right) \Delta t f _ {r} \end{array}
$$

由于波动是连续的,所以在 $\left[T_{0},T_{1}\right]$ 内接收到的信号波数N应与 $\left[t_{0},t_{1}\right]$ 内发出的信号波数n相等,即

$$
\begin{array}{r l} \Delta t f _ {0} & = \left(1 - \frac {v}{c}\right) \Delta t f _ {r} \\ f _ {r} & = \frac {c}{c - v} f _ {0} \end{array}\tag{8.3.7}
$$

所以多普勒频移为

$$
f _ {d} = f _ {r} - f _ {0} = \frac {v}{c - v} f _ {0}\tag{8.3.8}
$$

如果发射机与接收机安装在同一运载体上，反射点 B 固定不动，运载体以速度 v 向 B 点运动，如图 8.3.3 所示。则根据式(8.3.7)，信号到达 B 点时的振荡频率为

$$
f _ {b} = \frac {c}{c - v} f _ {0}
$$

![](images/169df47a4c164751ea56647a9638d7afcc98878e0400959f48a573b3d2e75ec9.jpg)  
图 8.3.3 发射机和接收机安装在同一运载体上的多普勒频移

信号到达 B 点后将被反射回 A 点, 这相当于 B 点发射的信号, 发射频率为 $f_{b}$ , 由于存在相对速度 v, 所以 A 点接收到的信号频率为

$$
f _ {a} = \frac {c}{c - v} f _ {b} = \left(1 - \frac {v}{c}\right) ^ {- 2} f _ {0}
$$

又由于 $\frac{v}{c} \ll 1$ ，所以

$$
f _ {a} = \left(1 + \frac {2 v}{c}\right) f _ {0}\tag{8.3.9}
$$

多普勒频移为

$$
f _ {d} = f _ {a} - f _ {0} = \frac {2 v}{c} f _ {0}\tag{8.3.10}
$$

或

$$
f _ {d} = \frac {2 v}{\lambda_ {0}}\tag{8.3.11}
$$

式中 $\lambda_{0}=\frac{c}{f_{0}}$ 为发射信号的波长。

式 $(8.3.11)$ 即为机载多普勒雷达的测速原理。

早期的机载多普勒雷达系统是双波束系统,系统具有两副天线。天线安装在伺服平台上,两个波束产生的多普勒频移之差驱动伺服平台转动,当两个多普勒频移相等时,伺服平台停止转动,平台相对飞机纵向对称面转过的几何角即为偏流角。根据测得的多普勒频移和天线安装角解算出飞机的地速。这种双波束系统既笨重,结构又复杂,目前已很少使用,而广泛使用具有固定天线的四波束系统。

四波束系统的4个发射天线是固定在机身上的平板天线，天线发射波束的方向相对机体坐标系的角位置是固定不变的，如图8.3.4所示。其中， $x_{\mathrm{b}}y_{\mathrm{b}}z_{\mathrm{b}}$ 为飞机的机体坐标系，并规定右、前、上为正向，记单位向量分别为 $i,j,k$ 。 $\pmb{n}_1,\pmb{n}_2,\pmb{n}_3$ 和 $\pmb{n}_4$ 为4个波束的发射方向。

设波束相对飞机纵向对称面的水平偏角为 $\beta$ ，水平倾角为 $\alpha$ ，飞机的地速向量为

$$
\boldsymbol {v} = v _ {x} \boldsymbol {i} + v _ {y} \boldsymbol {j} + v _ {z} \boldsymbol {k}
$$

由图8.3.4得

$$
\boldsymbol {n} _ {2} = \cos \alpha \sin \beta \boldsymbol {i} + \cos \alpha \cos \beta \boldsymbol {j} - \sin \alpha \boldsymbol {k}\tag{8.3.12a}
$$

由于波束对称配置,所以

$$
\boldsymbol {n} _ {1} = - \cos \alpha \sin \beta i + \cos \alpha \cos \beta j - \sin \alpha k\tag{8.3.12b}
$$

$$
\boldsymbol {n} _ {3} = \cos \alpha \sin \beta \boldsymbol {i} - \cos \alpha \cos \beta \boldsymbol {j} - \sin \alpha \boldsymbol {k}\tag{8.3.12c}
$$

![](images/1640eec0eba2f1f58a7585b33a9af20e5bb2ac48d47025b8135733c6967bc77c.jpg)  
图 8.3.4 固定天线的发射方向

$$
\boldsymbol {n} _ {4} = - \cos \alpha \sin \beta \boldsymbol {i} - \cos \alpha \cos \beta \boldsymbol {j} - \sin \alpha \boldsymbol {k}\tag{8.3.12d}
$$

地速向量 v 在波束 1,2,3,4 方向上的投影为

$$
v _ {1} = \boldsymbol {v} \cdot \boldsymbol {n} _ {1} = - v _ {x} \cos \alpha \sin \beta + v _ {y} \cos \alpha \cos \beta - v _ {z} \sin \alpha\tag{8.3.13a}
$$

$$
v _ {2} = \boldsymbol {v} \cdot \boldsymbol {n} _ {2} = v _ {x} \cos \alpha \sin \beta + v _ {y} \cos \alpha \cos \beta - v _ {z} \sin \alpha\tag{8.3.13b}
$$

$$
v _ {3} = \boldsymbol {v} \cdot \boldsymbol {n} _ {3} = v _ {x} \cos \alpha \sin \beta - v _ {y} \cos \alpha \cos \beta - v _ {z} \sin \alpha\tag{8.3.13c}
$$

$$
v _ {4} = \boldsymbol {v} \cdot \boldsymbol {n} _ {4} = - v _ {x} \cos \alpha \sin \beta - v _ {y} \cos \alpha \cos \beta - v _ {z} \sin \alpha\tag{8.3.13d}
$$

根据式(8.3.11)，得4个波束的多普勒频移分别为

$$
f _ {\mathrm{dl}} = \frac {2}{\lambda_ {0}} (- v _ {x} \cos \alpha \sin \beta + v _ {y} \cos \alpha \cos \beta - v _ {z} \sin \alpha)\tag{8.3.14a}
$$

$$
f _ {\mathrm{d} 2} = \frac {2}{\lambda_ {0}} (v _ {x} \cos \alpha \sin \beta + v _ {y} \cos \alpha \cos \beta - v _ {z} \sin \alpha)\tag{8.3.14b}
$$

$$
f _ {\mathrm{d} 3} = \frac {2}{\lambda_ {0}} (v _ {x} \cos \alpha \sin \beta - v _ {y} \cos \alpha \cos \beta - v _ {z} \sin \alpha)\tag{8.3.14c}
$$

$$
f _ {\mathrm{d4}} = \frac {2}{\lambda_ {0}} (- v _ {x} \cos \alpha \sin \beta - v _ {y} \cos \alpha \cos \beta - v _ {z} \sin \alpha)\tag{8.3.14d}
$$

由式(8.3.14)得

$$
v _ {x} = \frac {\lambda_ {0}}{4 \cos \alpha \sin \beta} (f _ {\mathrm{d2}} - f _ {\mathrm{d1}})\tag{8.3.15a}
$$

$$
v _ {y} = \frac {\lambda_ {0}}{4 \cos \alpha \cos \beta} (f _ {\mathrm{d2}} - f _ {\mathrm{d3}})\tag{8.3.15b}
$$

$$
v _ {z} = - \frac {\lambda_ {0}}{4 \sin \alpha} (f _ {\mathrm{d1}} + f _ {\mathrm{d3}})\tag{8.3.15c}
$$

设飞机的航向角为 $\Psi$ ，俯仰角为 $\theta$ ，横滚角为 $\gamma$ ，则飞机的机体坐标系 $Ox_{h}y_{h}z_{h}$ （右、前、上）与水平坐标系 $Ox_{h}y_{h}x_{h}$ 间的角位置关系如图 8.3.5 所示。水平坐标系 $Ox_{h}y_{h}x_{h}$ 与地理坐标系

$Ox_{g}y_{g}z_{g}$ （东、北、天）间的角位置关系如图8.3.6所示。

![](images/f56b53b43acab42f5450315073d18539bf5c739601f17decd54bf4912255644e.jpg)  
图8.3.5 机体坐标系与水平坐标系间的角位置关系

![](images/e0792d95a5c53a2465b9b6a467303959e3fab6cf4e37212a637d4acfa2e3508a.jpg)  
图 8.3.6 水平坐标系与地理坐标系间的角位置关系

由图8.3.5和图8.3.6得变换矩阵

$$
\begin{array}{r l} & C _ {\mathrm{h}} ^ {\mathrm{b}} = \left[ \begin{array}{c c c} \cos \gamma & \sin \gamma \sin \theta & - \sin \gamma \cos \theta \\ 0 & \cos \theta & \sin \theta \\ \sin \gamma & - \cos \gamma \sin \theta & \cos \gamma \cos \theta \end{array} \right] \\ & C _ {\mathrm{g}} ^ {\mathrm{b}} = C _ {\mathrm{h}} ^ {\mathrm{b}} \cdot C _ {\mathrm{g}} ^ {\mathrm{h}} = \left[ \begin{array}{c c c} \cos \gamma \cos \Psi - \sin \gamma \sin \theta \sin \Psi & \cos \gamma \sin \Psi + \sin \gamma \sin \theta \cos \Psi & - \sin \gamma \cos \theta \\ - \cos \theta \sin \Psi & \cos \theta \cos \Psi & \sin \theta \\ \sin \gamma \cos \Psi + \cos \gamma \sin \theta \sin \Psi & \sin \gamma \sin \Psi - \cos \gamma \sin \theta \cos \Psi & \cos \gamma \cos \theta \end{array} \right] \end{array} \tag {8.3}\tag{8.3.16}
$$

所以

$$
\begin{array}{r l} & \boldsymbol {C} _ {\mathrm{b}} ^ {\mathrm{h}} = \left[ \begin{array}{c c c} \cos \gamma & 0 & \sin \gamma \\ \sin \gamma \sin \theta & \cos \theta & - \cos \gamma \sin \theta \\ - \sin \gamma \cos \theta & \sin \theta & \cos \gamma \cos \theta \end{array} \right] \\ & \boldsymbol {C} _ {\mathrm{b}} ^ {\mathrm{g}} = \left[ \begin{array}{c c c} \cos \gamma \cos \Psi - \sin \gamma \sin \theta \sin \Psi & - \cos \theta \sin \Psi & \sin \gamma \cos \Psi + \cos \gamma \sin \theta \sin \Psi \\ \cos \gamma \sin \Psi + \sin \gamma \sin \theta \cos \Psi & \cos \theta \cos \Psi & \sin \gamma \sin \Psi - \cos \gamma \sin \theta \cos \Psi \\ - \sin \gamma \cos \theta & \sin \theta & \cos \gamma \cos \theta \end{array} \right] \end{array}\tag{8.3.17}
$$

(8.3.18)

地速向量 v 在水平坐标系 $Ox_{h}y_{h}z_{h}$ 内的分量为

$$
\boldsymbol {v} ^ {\mathrm{h}} = \boldsymbol {C} _ {\mathrm{b}} ^ {\mathrm{h}} \boldsymbol {v} ^ {\mathrm{b}}
$$

即

$$
\begin{array}{l} v _ {x} ^ {\mathrm{h}} = v _ {x} \cos \gamma + v _ {z} \sin \gamma \\ v _ {y} ^ {\mathrm{h}} = v _ {x} \sin \gamma \sin \theta + v _ {y} \cos \theta - v _ {z} \cos \gamma \sin \theta \\ v _ {z} ^ {\mathrm{h}} = v _ {x} \sin \gamma \cos \theta + v _ {y} \sin \theta + v _ {z} \cos \gamma \cos \theta \end{array}
$$

所以偏流角为

$$
\delta = \arctan \frac {v _ {x} ^ {\mathrm{h}}}{v _ {y} ^ {\mathrm{h}}} = \arctan \frac {v _ {x} \cos \gamma + v _ {z} \sin \gamma}{v _ {x} \sin \gamma \sin \theta + v _ {y} \cos \theta - v _ {z} \cos \gamma \sin \theta}\tag{8.3.19}
$$

地速向量在地理坐标系 $Ox_{g}y_{g}z_{g}$ 内的分量为

$$
\boldsymbol {v} ^ {\mathrm{g}} = \boldsymbol {C} _ {\mathrm{b}} ^ {\mathrm{g}} \boldsymbol {v} ^ {\mathrm{b}}
$$

即

$$
\begin{array}{r l} & v _ {\mathrm{E}} = (\cos \gamma \cos \Psi - \sin \gamma \sin \theta \sin \Psi) v _ {x} - \cos \theta \sin \Psi v _ {y} + \\ & (\sin \gamma \cos \Psi + \cos \gamma \sin \theta \sin \Psi) v _ {z} \\ & v _ {\mathrm{N}} = (\cos \gamma \sin \Psi + \sin \gamma \sin \theta \cos \Psi) v _ {x} + \cos \theta \cos \Psi v _ {y} + \\ & (\sin \gamma \sin \Psi - \cos \gamma \sin \theta \cos \Psi) v _ {z} \\ & v _ {\mathrm{U}} = - \sin \gamma \cos \theta v _ {x} + \sin \theta v _ {y} + \cos \gamma \cos \theta v _ {z} \end{array}\tag{8.3.20a}
$$

(8.3.20b)

(8.3.20c)

定位计算方程为

$$
\dot {L} (t) = \frac {v _ {\mathrm{N}}}{R}\tag{8.3.21a}
$$

$$
\dot {\lambda} (t) = \frac {v _ {\mathrm{E}}}{R \cos L}\tag{8.3.21b}
$$

$$
\dot {h} (t) = v _ {\mathrm{U}}\tag{8.3.21c}
$$

由式(8.3.15)知,多普勒导航系统是通过测量多普勒频移获得地速向量的,从多普勒频移计算出地速须经过转换,比例系数分别为

$$
k _ {\mathrm{d} x} = \frac {\lambda_ {0}}{4 \cos \alpha \sin \beta}\tag{8.3.22a}
$$

$$
k _ {\mathrm{dy}} = \frac {\lambda_ {0}}{4 \cos \alpha \cos \beta}\tag{8.3.22b}
$$

$$
k _ {\mathrm{dz}} = - \frac {\lambda_ {0}}{4 \sin \alpha}\tag{8.3.22c}
$$

所以多普勒雷达的测速刻度系数误差应与发射信号波长 $\lambda_{0}$ 的稳定性、天线的水平偏角 $\beta$ 及水平倾角 $\alpha$ 的安装精度有关。

设发射信号波长的设计值为 $\lambda_{0}$ ，实际发射信号的波长具有偏差 $\delta\lambda_{0}$ ，发射波束的水平偏角及水平倾角的设计值分别为 $\beta$ 和 $\alpha$ ，由于安装误差，上述两个几何角分别具有偏差 $\delta\beta$ 和 $\delta\alpha$ ，则实际的比例系数为

$$
k _ {\mathrm{d} x} ^ {c} = \frac {\lambda_ {0} + \delta \lambda_ {0}}{4 \cos (\alpha + \delta \alpha) \sin (\beta + \delta \beta)}\tag{8.3.23a}
$$

$$
k _ {\mathrm{dy}} ^ {c} = \frac {\lambda_ {0} + \delta \lambda_ {0}}{4 \cos (\alpha + \delta \alpha) \cos (\beta + \delta \beta)}\tag{8.3.23b}
$$

$$
k _ {\mathrm{dz}} ^ {c} = - \frac {\lambda_ {0} + \delta \lambda_ {0}}{4 \sin (\alpha + \delta \alpha)}\tag{8.3.23c}
$$

将式(8.3.23a)、式(8.3.23b)、式(8.3.23c)分别在 $\lambda_{0},\alpha$ 和 $\beta$ 处展成泰勒级数，并只保留关于误差 $\delta\lambda_{0}$ ， $\delta\alpha$ 和 $\delta\beta$ 的一次项，得

$$
\begin{array}{r l} k _ {\mathrm{d} x} ^ {c} & = \frac {1}{4} \lambda_ {0} \sec \alpha \csc \beta + \frac {1}{4} \delta \lambda_ {0} \sec \alpha \csc \beta + \\ & \quad \frac {1}{4} \delta \alpha \lambda_ {0} \sec \alpha \tan \alpha \csc \beta - \frac {1}{4} \delta \beta \lambda_ {0} \sec \alpha \csc \beta \cot \beta \\ k _ {\mathrm{dy}} ^ {c} & = \frac {1}{4} \lambda_ {0} \sec \alpha \sec \beta + \frac {1}{4} \delta \lambda_ {0} \sec \alpha \sec \beta + \end{array}\tag{8.3.24a}
$$

$$
\frac {1}{4} \delta \alpha \lambda_ {0} \sec \alpha \tan \alpha \sec \beta + \frac {1}{4} \delta \beta \lambda_ {0} \sec \alpha \sec \beta \tan \beta\tag{8.3.24b}
$$

$$
k _ {\mathrm{dz}} ^ {c} = - \frac {1}{4} \lambda_ {0} \csc \alpha - \frac {1}{4} \delta \lambda_ {0} \csc \alpha + \frac {1}{4} \delta \alpha \lambda_ {0} \csc \alpha \cot \alpha\tag{8.3.24c}
$$

比较式 $(8.3.24)$ 和式 $(8.3.22)$ ，得比例系数误差为

$$
\begin{array}{r l} \delta k _ {\mathrm{dx}} & = k _ {\mathrm{dx}} ^ {c} - k _ {\mathrm{dx}} = \\ & \frac {1}{4} (\delta \lambda_ {0} \sec \alpha \csc \beta + \delta \alpha \lambda_ {0} \sec \alpha \tan \alpha \csc \beta - \delta \beta \lambda_ {0} \sec \alpha \csc \beta \cot \beta) \\ \delta k _ {\mathrm{dy}} & = k _ {\mathrm{dy}} ^ {c} - k _ {\mathrm{dy}} = \\ & \frac {1}{4} (\delta \lambda_ {0} \sec \alpha \sec \beta + \delta \alpha \lambda_ {0} \sec \alpha \tan \alpha \sec \beta + \delta \beta \lambda_ {0} \sec \alpha \sec \beta \tan \beta) \\ \delta k _ {\mathrm{dz}} & = k _ {\mathrm{dz}} ^ {c} - k _ {\mathrm{dz}} = \\ & - \frac {1}{4} (\delta \lambda_ {0} \csc \alpha - \delta \alpha \lambda_ {0} \csc \alpha \cot \alpha) \end{array}\tag{8.3.25a}
$$

(8.3.25b)

(8.3.25c)

所以多普勒雷达的测速刻度系数误差为

$$
\delta K _ {\mathrm{d} x} = \frac {\delta k _ {\mathrm{d} x}}{k _ {\mathrm{d} x}} = \frac {\delta \lambda_ {0}}{\lambda_ {0}} + \delta \alpha \tan \alpha - \delta \beta \cot \beta\tag{8.3.26a}
$$

$$
\delta K _ {\mathrm{dy}} = \frac {\delta k _ {\mathrm{dy}}}{k _ {\mathrm{dy}}} = \frac {\delta \lambda_ {0}}{\lambda_ {0}} + \delta \alpha \tan \alpha + \delta \beta \tan \beta\tag{8.3.26b}
$$

$$
\delta K _ {\mathrm{dz}} = \frac {\delta k _ {\mathrm{dz}}}{k _ {\mathrm{dz}}} = \frac {\delta \lambda_ {0}}{\lambda_ {0}} - \delta \alpha \cot \alpha\tag{8.3.26c}
$$

设波束的水平偏角和水平倾角设计值均为 $45^{\circ}$ ，天线安装不准引起的误差角 $\delta \alpha$ 和 $\delta \beta$ 均为随机量，显然 $\delta \alpha$ 和 $\delta \beta$ 互相独立，且均值都为零。若均方根都为 $6'$ ，则由安装误差引起的刻度系数误差的均方根为 $\sqrt{2} \left( \frac{6}{60} \times \frac{\pi}{180} \right) = 2.4 \times 10^{-3}$ 。而多普勒雷达发射的是厘米波，发射频率的稳定度不会低于 $10^{-6}$ ，所以多普勒雷达的测速刻度系数误差主要取决于固定天线的安装误差。由于天线的安装角相对飞机是固定不变的，所以由安装误差角引起的刻度系数误差也是固定不变的，但都是随机变量，因此用随机常数描述之

$$
\delta \dot {K} _ {\mathrm{d} x} = 0\tag{8.3.27a}
$$

$$
\delta \dot {K} _ {\mathrm{dy}} = 0\tag{8.3.27b}
$$

$$
\delta \dot {K} _ {\mathrm{dz}} = 0\tag{8.3.27c}
$$

这些随机常数的均值都为零，均方根约为 $10^{-3}\sim10^{-4}$ 。

设飞机上的角位置信息系统(如惯导、航姿系统或垂直陀螺和航向系统等)给出的航向角和姿态角分别为

$$
\begin{array}{r l} & \Psi_ {c} = \Psi + \delta \Psi \\ & \theta_ {c} = \theta + \delta \theta \\ & \gamma_ {c} = \gamma + \delta \gamma \end{array}
$$

式中， $\Psi, \theta$ 和 $\gamma$ 分别为飞机的航向角、俯仰角及横滚角的真实值； $\delta \Psi, \delta \theta$ 和 $\delta \gamma$ 分别为实际系统对航向角、俯仰角及横滚角的测量误差。并设飞机的地速向量在机体坐标系内各分量的真实值为 $v_x, v_y, v_z$ ，多普勒雷达的测速刻度系数误差为 $\delta K_{\mathrm{d}x}, \delta K_{\mathrm{d}y}, \delta K_{\mathrm{d}z}$ ，则根据式(8.3.20)，多普勒导航系统输出的地速向量在地球坐标系内的分量为

$$
\begin{array}{r l} v _ {\mathrm{E}} ^ {c} & = \left[ \cos (\gamma + \delta \gamma) \cos (\Psi + \delta \Psi) - \sin (\gamma + \delta \gamma) \sin (\theta + \delta \theta) \sin (\Psi + \delta \Psi) \right] \times \\ & (1 + \delta K _ {\mathrm{dx}}) v _ {x} - \cos (\theta + \delta \theta) \sin (\Psi + \delta \Psi) (1 + \delta K _ {\mathrm{dy}}) v _ {y} + \\ & [ \sin (\gamma + \delta \gamma) \cos (\Psi + \delta \Psi) + \cos (\gamma + \delta \gamma) \sin (\theta + \delta \theta) \sin (\Psi + \delta \Psi) ] \times \\ & (1 + \delta K _ {\mathrm{dz}}) v _ {z} \\ v _ {\mathrm{N}} ^ {c} & = [ \cos (\gamma + \delta \gamma) \sin (\Psi + \delta \Psi) + \sin (\gamma + \delta \gamma) \sin (\theta + \delta \theta) \cos (\Psi + \delta \Psi) ] \times \\ & (1 + \delta K _ {\mathrm{dx}}) v _ {x} + \cos (\theta + \delta \theta) \cos (\Psi + \delta \Psi) (1 + \delta K _ {\mathrm{dy}}) v _ {y} + \\ & [ \sin (\gamma + \delta \gamma) \sin (\Psi + \delta \Psi) - \cos (\gamma + \delta \gamma) \sin (\theta + \delta \theta) \cos (\Psi + \delta \Psi) ] \times \\ & (1 + \delta K _ {\mathrm{dz}}) v _ {z} \\ v _ {\mathrm{U}} ^ {c} & = - \sin (\gamma + \delta \gamma) \cos (\theta + \delta \theta) (1 + \delta K _ {\mathrm{dx}}) v _ {x} + \sin (\theta + \delta \theta) (1 + \delta K _ {\mathrm{dy}}) v _ {y} + \\ & \cos (\gamma + \delta \gamma) \cos (\theta + \delta \theta) (1 + \delta K _ {\mathrm{dz}}) v _ {z} \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (2 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 5 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 6 2 4 2 2 \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (1 + \delta K _ {\mathrm{dx}}) v _ {x} - \cos (\theta + \delta \theta) v _ {y} - \sin (\theta + \delta \theta) v _ {z} - v _ {x} - v _ {y} - v _ {z} - v _ {y} - v _ {z} - v _ {x} - v _ {y} - v _ {z} - v _ {y} - v _ {z} - v _ {x} - v _ {y} - v _ {z} - v _ {y} - v _ {z} - v _ {x} - v _ {y} - v _ {z} - v _ {y} - v _ {z} - v _ {y} - v _ {z} - v _ {x} - v _ {y} - v _ {z} - v _ {y} - v _ {z} - v _ {x} - v _ {y} - v _ {z} - v _ {y} - v _ {z} - v _ {x} - v _ {y} - v_ {x}, \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (8. 3) \\ & (7. 3) \\ & (7. 3) \\ & (7. 3) \\ & (7. 3) \\ & (7. 3) \\ & (7. 3) \\ & (7. 3) \\ & (7. 3) \\ & (7. 3) \\ & (7. 3) \\ & (7. 3) \\ & (7. 3) \\ & (7. 3). \\ & (7. 3). \\ & (7. 3). \\ & (7. 3). \\ & (7. 3). \\ & (7. 3). \\ & (7. 3). \\ & (7. 3). \\ & (7. 3). \\ & (7. 3). \\ & (7. 3). \\ & (7. 3). \\ & (7. 3). \\ & (8. 3). \\ & (8. 3). \\ & (8. 3). \\ & (8. 3). \\ & (8. 3). \\ & (8. 3). \\ & (8. 3). \\ & (8. 3). \\ & (8. 3). \\ & (8. 3). \\ & (8. 3). \\ & (8. 3). \\ & (8. 3), \\ & (- s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n(\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s i n (\gamma + s is a b d e r t a c t y o f a r e d e r t a c t y o f a r e d e r t a c t y o f a r e d e r t a c t y o f a r e d e r t a c t y o f a r e d e r t a c t y o f a r e d e r t a c t y o f a r e d e r t a c t y o f a r e d e r t a c t y, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, u, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, u, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, y, z, w, x, u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u u U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U U I N O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O O S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S A S B A B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B C D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D D E F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F F W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W W Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y Y T M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M M m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m m c o r p h e r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r e f o r   e f o r   e   f o r   e   f o r   e   f o r   e   f o r   e   f o r   e   f o r   e   f o r   e   f o r   e   f o r   e   f o r   e   f o r   e   f o r   e   f o r   e   f o r   e   f o r   e   f o r   e   f o r    | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | | |
| V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V= V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V = V / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R /R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / R / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L / L /L : * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *< lcel>< nl>\tag{8.3.28a}
$$

(8.3.28b)

(8.3.28c)

只保留关于误差的一次项，则由式(8.3.20)和式(8.3.28)，得多普勒导航系统输出的在地理坐标系内的地速分量误差为

$$
\begin{array}{r l} \delta v _ {\mathrm{dE}} & = v _ {\mathrm{E}} ^ {c} - v _ {\mathrm{E}} = \delta K _ {\mathrm{dx}} (\cos \gamma \cos \Psi - \sin \gamma \sin \theta \sin \Psi) v _ {x} - \\ & \delta K _ {\mathrm{dy}} \cos \theta \sin \Psi v _ {y} + \delta K _ {\mathrm{dz}} (\sin \gamma \cos \Psi + \cos \gamma \sin \theta \sin \Psi) v _ {z} + \\ & \delta \Psi [ - (\cos \gamma \sin \Psi + \cos \Psi \sin \gamma \sin \theta) v _ {x} - \cos \Psi \cos \theta v _ {y} + \\ & (\cos \Psi \cos \gamma \sin \theta - \sin \Psi \sin \gamma) v _ {z} ] + \\ & \delta \theta (- \cos \theta \sin \gamma \sin \Psi v _ {x} + \sin \theta \sin \Psi v _ {y} + \cos \theta \cos \gamma \sin \Psi v _ {z}) + \\ & \delta \gamma [ - (\cos \Psi \sin \gamma + \cos \gamma \sin \theta \sin \Psi) v _ {x} + \\ & (\cos \gamma \cos \Psi - \sin \gamma \sin \theta \sin \Psi) v _ {z} ] \end{array}\tag{8.3.29a}
$$

$$
\begin{array}{r l} \delta v _ {\mathrm{dN}} & = v _ {\mathrm{N}} ^ {c} - v _ {\mathrm{N}} = \delta K _ {\mathrm{dx}} (\cos \gamma \sin \Psi + \sin \gamma \sin \theta \cos \Psi) v _ {x} + \\ & \delta K _ {\mathrm{dy}} \cos \theta \cos \Psi v _ {y} + \delta K _ {\mathrm{dz}} (\sin \gamma \sin \Psi - \cos \gamma \sin \theta \cos \Psi) v _ {z} + \\ & \delta \Psi [ (\cos \Psi \cos \gamma - \sin \Psi \sin \gamma \sin \theta) v _ {x} - \sin \Psi v _ {y} + \\ & (\cos \Psi \sin \gamma + \sin \Psi \cos \gamma \sin \theta) v _ {z} ] + \\ & \delta \theta (\cos \theta \sin \gamma \cos \Psi v _ {x} - \sin \theta v _ {y} - \cos \theta \cos \gamma \cos \Psi v _ {z}) + \\ & \delta \gamma [ (\cos \gamma \sin \theta \cos \Psi - \sin \gamma \sin \Psi) v _ {x} + (\cos \gamma \sin \Psi + \sin \gamma \sin \theta \cos \Psi) v _ {z} ] \end{array}\tag{8.3.29b}
$$

$$
\begin{array}{r l} \delta v _ {\mathrm{dU}} & = v _ {\mathrm{U}} ^ {c} - v _ {\mathrm{U}} = - \delta K _ {\mathrm{dx}} \sin \gamma \cos \theta v _ {x} + \delta K _ {\mathrm{dy}} \sin \theta v _ {y} + \delta K _ {\mathrm{dz}} \cos \gamma \cos \theta v _ {z} + \\ & \delta \theta (- \sin \theta \sin \gamma v _ {x} + \cos \theta v _ {y} - \sin \theta \cos \gamma v _ {z}) + \\ & \delta \gamma (- \cos \gamma \cos \theta v _ {x} - \sin \gamma \cos \theta v _ {z}) \end{array}\tag{8.3.29c}
$$

由式 $(8.3.21)$ 得定位误差方程

$$
\delta \dot {L} _ {\mathrm{d}} = \frac {\delta v _ {\mathrm{dN}}}{R}\tag{8.3.30a}
$$

$$
\delta \dot {\lambda} _ {\mathrm{d}} = \frac {\delta v _ {\mathrm{dE}}}{R} \sec L + \delta L _ {\mathrm{d}} \frac {v _ {\mathrm{E}}}{R} \sec L \tan L\tag{8.3.30b}
$$

$$
\delta \dot {h} _ {\mathrm{d}} = \delta v _ {\mathrm{dU}}\tag{8.3.30c}
$$

确定飞机的三个地速分量仅需三个不共面波束,但工程上常采用四波束方案,原因是:(1)平面阵列天线能自然产生四波束;(2)利用四波束系统提供的冗余度可对多普勒雷达作故障检测,并可采用最小二乘算法提高地速分量的计算精度。

## 8.3.3 大气数据系统

大气数据系统通过安装在飞机机身外侧的全动压管路、静压管路、总温传感器和攻角传感器测量飞机周围流场内的静压、动压、总温和飞机的攻角，并将这些信息送到计算机中，解算出飞机的空气动力信息和导航信息，其中与导航有关的信息为真空速、升降速度和气压高度。随着计算机技术、集成电路和和新型压力传感器的发展，大气数据系统已由早先的机、电式模拟系统发展成数字式系统，并成为大多数飞机的标准机载设备。

影响大气数据系统精度的因素是很多的,有原理误差,也有方法误差,如静压与高度间的非线性关系,温度补偿不准确引起的误差等。但由于气压高度和空速是根据测量得的动压和静压经过解算获得的,诸多误差都可并入这种转换不准确而引起的转换误差中,即可以用刻度系数误差来描述之。做这样的简化处理对设计组合导航系统中的卡尔曼滤波器十分有利,因为在设计卡尔曼滤波器时,为避免阶数过高而使计算失去实时性,描述系统模型的状态应尽量少,只能选择反映系统主要特性的误差量作为状态。

此外,影响气压高度精度的主要误差源还有气压方法误差。从实际测量的气压值换算成的气压高度是相对标准海平面(气温为 $15^{\circ}C$ ,气压为101.325 kPa的海平面)的高度,而飞机所在处的实际基准海平面气压值与标准海平面的气压值存在差异,这种差异反映在气压高度上就是气压方法误差。不同地点的气压方法误差是不相同的,就是在同一地点,这一误差也会随时间改变,且这种误差是随机过程。

气压方法误差可用一阶马尔柯夫过程近似描述

$$
\delta \dot {h} _ {\mathrm{b}} = - \frac {1}{\tau_ {\mathrm{b}}} \delta h _ {\mathrm{b}} + w _ {\mathrm{b}}\tag{8.3.31}
$$

其中相关时间 $\tau_{b}$ 视飞机的飞行速度和活动范围而定，对于飞行速度较低、活动范围较小的飞机，如直升飞机，相关时间长些，而对于飞行速度高、飞行距离长的飞机，相关时间则短些。一般情况下，相关时间约为 1000 s，距离误差均方根为 50～100 m。

空速是飞机相对周围空气团的速度,如果将空速近似取作地速,则风速被当作误差引入到地速的测量值中,即

$$
\nu_ {A} = (1 + \delta K _ {a}) \nu_ {a} + \nu_ {\mathrm{w}}\tag{8.3.32}
$$

式中， $v_{a}$ 为实际的真空速； $\delta K_{a}$ 为空速测量中的刻度系统误差； $v_{w}$ 为风速向量。

假设空速向量沿飞机的纵轴方向，飞机上其他导航系统提供的航向和姿态信息分别为 $\Psi +\delta \Psi ,\theta +\delta \theta ,\gamma +\delta \gamma$ ，其中 $\delta \Psi ,\delta \theta ,\delta \gamma$ 分别为导航设备对真实值 $\Psi ,\theta ,\gamma$ 的测量误差，则大气数据系统输出的空速向量在地理坐标系（东、北、天）内的分量为

$$
\boldsymbol {v} _ {A} ^ {\mathrm{g}} = (1 + \delta K _ {a}) \boldsymbol {C} _ {\mathrm{b}} ^ {\mathrm{g}} \boldsymbol {v} _ {\mathrm{a}} ^ {\mathrm{b}} + \boldsymbol {C} _ {\mathrm{b}} ^ {\mathrm{g}} \boldsymbol {v} _ {\mathrm{w}} ^ {\mathrm{b}}\tag{8.3.33}
$$

式中，角标 b 表示在飞机的机体坐标系（右、前、上）的量， $v_{a}^{b}=\left[\begin{array}{ccc}0 & v_{a} & 0\end{array}\right]^{T}$ 。

式 $(8.3.18)$ 代入式 $(8.3.33)$ ，得

$$
\left[ \begin{array}{l} v _ {\mathrm{AE}} \\ v _ {\mathrm{AN}} \\ v _ {\mathrm{AU}} \end{array} \right] = \left[ \begin{array}{l} - (1 + \delta K _ {a}) \cos (\theta + \delta \theta) \sin (\Psi + \delta \Psi) v _ {a} \\ (1 + \delta K _ {a}) \cos (\theta + \delta \theta) \cos (\Psi + \delta \Psi) v _ {a} \\ (1 + \delta K _ {a}) \sin (\theta + \delta \theta) v _ {a} \end{array} \right] + \left[ \begin{array}{l} v _ {w \mathrm{E}} \\ v _ {w \mathrm{N}} \\ v _ {w \mathrm{U}} \end{array} \right]\tag{8.3.34}
$$

对上式取一阶近似,则可得将空速取作地速时引起的误差

$$
\delta v _ {A E} = - \delta K _ {a} v _ {a} \cos \theta \sin \Psi + \delta \theta v _ {a} \sin \theta \sin \Psi - \delta \Psi v _ {a} \cos \Psi \cos \theta + v _ {w E}\tag{8.3.35a}
$$

$$
\delta v _ {A N} = \delta K _ {a} v _ {a} \cos \theta \cos \Psi - \delta \theta v _ {a} \sin \theta \cos \Psi - \delta \Psi v _ {a} \sin \Psi \cos \theta + v _ {w N}\tag{8.3.35b}
$$

$$
\delta v _ {A U} = \delta K _ {a} v _ {a} \sin \theta + \delta \theta v _ {a} \cos \theta + v _ {w U}\tag{8.3.35c}
$$

式中 $v_{wE}$ ， $v_{wN}$ ， $v_{wU}$ 为风速沿东向、北向和天向的分量。根据气象资料，风可模型化成三种成分：随机常值风，随机低频风和随机高频风（突风）。数学描述分别近似为：随机常数，一阶马尔柯夫过程和白噪声，即

$$
v _ {w i} = v _ {w b i} + v _ {w r i} + v _ {w w i}\tag{8.3.36}
$$

式中

$$
\dot {v} _ {w b i} = 0\tag{8.3.37}
$$

$$
\dot {v} _ {u r i} = - \frac {1}{\tau_ {u i}} v _ {u r i} + W _ {w i}\tag{8.3.38}
$$

式中 i=E, N, U。由于地球的东西方向大气环流明显，特别是在高空处更是如此，所以东向分量的均方根比北向和天向分量的均方根大，一般可取：东向风速的均方根为 $5 \sim 10 \, m/s$ ，北向风速的均方根为 $3 \sim 5 \, m/s$ ，天向风速的均方根为 $0.5 \sim 2 \, m/s$ ；相关风的相关时间约为数 $10 \, s$ 。

## 8.3.4 VOR - DME

VOR 是甚高频全向信标(VHF Omni-Range)的缩写,是一种相位测角系统,由地面信标台和机载接收指示设备两部分组成。这种系统为飞机提供相对地面信标台的方位。工作频率为 108～118 MHz,作用距离数百公里,测角精度优于 1.4°。VOR 地面台的发射天线布置如图 8.3.7 所示。

VOR 地面台共使用三副天线, 天线 1 是全向天线, 天线 2-2 和 3-3 都是分集天线。天线 1 发射的信号为

$$
e _ {1} = E _ {m} \left[ 1 + m \cos \left(2 \pi F _ {n} t + \frac {\Delta F _ {n}}{F} \cos 2 \pi F t\right) \right] \cos 2 \pi f t
$$

天线2-2发射的信号为

$$
e _ {2} = E _ {m} A \cos 2 \pi F t \cos 2 \pi f t
$$

天线3-3发射的信号为

$$
e _ {3} = E _ {m} A \sin 2 \pi F t \cos 2 \pi f t
$$

上述诸式中，f 为信号载频，振幅调制频率 $F = 30 \, Hz$ ，分载频 $F_{n} = 9960 \, Hz$ ，调频最大频偏 $\Delta F_{n} = \pm 480 \, Hz$ ，m 和 A 均为确定性常值， $e_{2}$ 和 $e_{3}$ 分别垂直天线平面 2-2 和 3-3。

设接收方向与北向夹角为 $\theta$ ，则由于 $e_{1}$ 由全向天线

![](images/23b428fb1678d20a5b9520438659a685f1e0f8b9c40907e88ee698a1d95ecf9f.jpg)  
图 8.3.7 VOR 台的发射天线布置

发射，与接收方向无关，而 $e_2$ 和 $e_3$ 均由分集天线发射，具有方向性，所以，在接收方向上接收到的合成信号为

$$
\begin{array}{r l} e = & E _ {m} \left[ 1 + m \cos \left(2 \pi F _ {n} t + \frac {\Delta F _ {n}}{F} \cos 2 \pi F t\right) + A \cos \theta \cos 2 \pi F t + A \sin \theta \sin 2 \pi F t \right] \cos 2 \pi f t = \\ & E _ {m} \left[ 1 + A \cos (\theta - 2 \pi F t) + m \cos \left(2 \pi F _ {n} t + \frac {\Delta F _ {n}}{F} \cos 2 \pi F t\right) \right] \cos 2 \pi f t \end{array} \tag {8.3}\tag{8.3.39}
$$

记

$$
s _ {1} = E _ {m} \left[ 1 + A \cos (\theta - 2 \pi F t) \right] \cos 2 \pi f t\tag{8.3.40}
$$

$$
s _ {2} = E _ {m} m \cos \left(2 \pi F _ {n} t + \frac {\Delta F _ {n}}{F} \cos 2 \pi F t\right) \cos 2 \pi f t\tag{8.3.41}
$$

则

$$
e = s _ {1} + s _ {2}
$$

这说明在用户接收到的信号中，有一部分信号 $(s_{1})$ 与用户相对VOR台的方位角 $\theta$ 有关，而另一部分信号 $(s_{2})$ 与方位无关（相当于 $\theta=0$ ），因此如果将 $s_{1}$ 和 $s_{2}$ 分离开，便可确定出 $\theta$ 。

再观察 $s_1$ 中的调制信号 $E_{m}[1 + A\cos (\theta -2\pi Ft)]$ 。由于 $E_{m}(1 + A\cos \theta)$ 为心形线，而 $E_{m}[1 + A\cos (\theta -2\pi Ft)]$ 相对 $E_{m}(1 + A\cos \theta)$ 滞后相角

而

$$
\begin{array}{l} \varphi (t) = 2 \pi F t \\ \dot {\varphi} (t) = 2 \pi F \end{array}
$$

所以 $s_{1}$ 形成的场强是在水平面内以 $2\pi F$ 为角速度旋转的心形图，即每秒旋转 30 周。所以在不同方位上调幅信号的初相是不相同的。图 8.3.8 示出了旋转辐射场在 $\theta=0^{\circ},\theta=90^{\circ},\theta=180^{\circ},\theta=270^{\circ}$ 时用户所接收到的调幅信号。

![](images/2bf5f9b46acfc5614f43631b3118585a6ac3f9809cef65f519c48f690be5421c.jpg)  
图 8.3.8 旋转辐射场和不同方位上的调幅信号

由于 $s_{2}$ 中频率为 F 的分量与方位 $\theta$ 无关，这相当于 $\theta = 0^{\circ}$ ，所以可看做北向的基准信号。机载接收设备接收上述两种信号，测量出两者的相位差，即可获得方位角 $\theta$ 。

在伏尔系统中,方位角 $\theta$ 是通过比对 $s_{1}$ 和 $s_{2}$ 中频率为 F 的信号分量的相位获得的,当信号中混杂有干扰时,将引起方位误差角。

设 $s_{2}$ 中频率为 F 的信号分量为 $c_{2}(t)$ ，信号中的噪声干扰为 $n(t)$ ，其中 $n(t)$ 为随机过程，则

实际用于确定方位的信号为

$$
\boldsymbol {s} (t) = \boldsymbol {c} _ {2} (t) + \boldsymbol {n} (t)
$$

$n(t)$ 的相位是随机的，当 $n(t)$ 与 $c_{2}(t)$ 正交时引起的方位误差最严重，如图 8.3.9 所示。方位误差 $\delta\theta$ 是零均值的随机量。

设 q 为信噪比，则 $\delta\theta$ 的均方根根据信噪比的高低确定如下 $^{[84]}$ ：

$$
\sigma_ {\delta \theta} = \left\{ \begin{array}{l l} \frac {1}{q}, & q \gg 1 \\ \sqrt {\frac {\pi^ {3}}{3} - q \sqrt {2 \pi}}, & q \ll 1 \end{array} \right.\tag{8.3.42}
$$

![](images/bafba375458afb0fdd259106ccc52e1653187ae327759f0e5c1d8f0c11e51b89.jpg)  
图8.3.9 噪声干扰对方位测定的影响

式中 $q \gg 1$ 表示信号很强的情况, $q \ll 1$ 则表示信号一般的情况。

DME 是一种询问应答式脉冲测距系统, 系统由机载询问器和地面应答器构成, 如图 8.3.10 所示。

地面应答器接收机接收到机载询问器发射的脉冲信号后，经过一个固定的延时 $t_{0}$ （一般为 $50\mu s$ ），转发一个与询问脉冲相对应的应答脉冲，机载询问器接收机接收此信号后，测出从发出询问脉冲到接收到应答脉冲的时间差t，则飞机至应答器的斜距为

$$
D = \frac {c}{2} (t - t _ {0})\tag{8.3.43}
$$

式中 $c$ 为光速。

![](images/af575222ffa80294cceeca39ce31c505a5f98b3396e3a658b4dad979cbc1fba2.jpg)  
图8.3.10 DME系统框图

根据机载高度系统(如气压高度表、无线电高度表等)给出的高度信息 H, 可计算出飞机至 DME 转发台站(位置准确已知)的水平距离为

$$
d = \sqrt {D ^ {2} - H ^ {2}}
$$

为了使地面应答器能够同时应答多架飞机的询问,信号采用了编码识别,编码识别体现在以下两个方面:

## 1. 采用不同的载波频率

DEM 的工作波段为 962～1213 MHz，每隔 1 MHz 安排一个工作频率。机载询问器的载频安排在 $1\ 025 \sim 1\ 150\ MHz$ 范围内，共有 126 个询问频率。地面应答器的载波频率安排在 $962 \sim 1\ 213\ MHz$ 范围内，共有 252 个应答频率。

## 2. 采用不同的时间编码脉冲

DME 信号是钟形脉冲对对载波的调制信号。根据形成钟形脉冲对两脉冲间时间间隔的不同分为 x 波道时间编码和 y 波道时间编码。对 x 波道，询问脉冲对和应答脉冲对两脉冲间的时间间隔都是 $12 \mu s$ ，而对 y 波道，询问脉冲对两脉冲间的时间间隔则为 $36 \mu s$ ，应答脉冲对则为 $30 \mu s$ ，其中各脉冲宽度约为 $3.5 \mu s$ ，采用钟形脉冲的好处是使信号带宽受到压缩，以减少相邻波道间的干扰。

根据以上编码格式,可形成252种不同的信号编码识别。询问和应答间的编码识别具有固定的搭配关系,如图8.3.11所示。信号的编码脉冲识别共有252种,即有252个波道。无论是地面应答器还是机载询问器,接收和发射载波频率之差都是63 MHz。在252个波道中,有52个波道在一般情况下避免使用。这52个波道是: $1x\sim16x$ , $1y\sim16y$ , $60x\sim69x$ , $60y\sim69y$ 。这是由于机场空中交通管制应答器的编码格式虽然与DME的编码格式不同,但使用的工作频率也位于上述波道的工作频率范围内,为了避免可能引起的干扰,应尽量避免使用上述52个波道。

![](images/908b4b192f2102219f154bcad4db6f7e6bf824fc921a389174bc052e7a374434.jpg)  
图 8.3.11 DME 信号的编码识别搭配关系

机载询问器发射的钟形脉冲对重复频率设计得与工作状态有关。在信号的搜索期间，为了尽快捕获信号，询问频度适当高些，最大速率可达150对脉冲每秒。在信号的跟踪期间，为了使应答器的工作容量得到充分利用，为尽量多的询问者服务，应尽量降低询问频度，机载询问器发射钟形脉冲对的重复频率一般约为24对脉冲每秒。对这些要求，询问器是能够自动满足的。此外，当询问的飞机数目接近或超过应答器的工作容量时，由于距离越近的飞机得到服务的希望越迫切，比如处于着陆准备状态，所以应答器应该有区别地对待迫切性不同的飞机的询问。而距离越远，到达应答器的询问脉冲信号的信噪比就越低，应答器的响应灵敏度就越低；距离越近，信噪比则越高，应答器的响应灵敏度就越高。这就自动区分了处于不同距离上

的用户的使用优先权。此外，应答器的接收灵敏度设计成可变的，即询问的飞机数目越多，灵敏度就降得越低，相反，飞机数目越少，灵敏度就越高。这样，应答器在工作繁忙时只为处于着陆准备状态等急于得到服务的飞机服务，而在空闲时也能为有足够准备时间的飞机服务。DME 的作用距离约为 300～500 km。

由于受噪声干扰的影响,从接收机输出端得到的钟形脉冲上叠加了噪声干扰,使钟形脉冲的形状变得模糊不清,如图8.3.12所示。而电磁波的传播延时是靠比对钟形脉冲的前沿来确定的,因此这些噪声干扰直接引入到测距误差中。

![](images/0153e1ad6226a5f75cbb7a14397bd6628fbda6bdc0c06b2b8d2ada124dba3518.jpg)  
图 8.3.12 有噪声干扰时 DME 接收机输出端的钟形脉冲

由于询问器发射的询问脉冲对是间歇性的,所以由此确定的斜距在时间上是离散的,测距误差是时间序列。而且钟形脉冲上叠加的是快速跳变的噪声干扰,相关性很弱,经过时间离散化处理后相关性就更弱了,所以 DME 的测距误差可看做是白噪声序列,均值为零,均方根约为 0.7 km。

## 8.3.5 双曲线无线电导航系统

双曲线无线电导航系统是一类远程无线电定位系统，它利用无线电信息测量出距离差，并根据双曲线定位原理确定出用户位置。双曲线定位原理见图8.3.13所示。

![](images/1a7e71795d69903da5a2e671b3de01de62949d8f99e7ee47e340eb4c837a44e7.jpg)  
图 8.3.13 双曲线定位原理

设在地球上 A, B, C 三点设置有无线电发射台，用户 P 处于无线电台的作用范围之内。若能测出 P 分别到 A, B 的距离差 $\Delta R_{1}$ 和到 B, C 的距离差 $\Delta R_{2}$ ，即

$$
\Delta R _ {1} = P A - P B
$$

$$
\Delta R _ {2} = P B - P C
$$

则根据 $\Delta R_{1}$ 和 $\Delta R_{2}$ 可分别确定出双曲线 I 和 II，而双曲线的交点之一即为 P 点。由于双曲线

Ⅰ 和 Ⅱ 形成的两交点相距很远, 所以根据已作出的过去时刻的定位值是很容易区分这种定位模糊性的。

双曲线定位系统有奥米加系统和罗兰系统。双曲线定位系统的关键是测出不同发射台发射的信号到达用户的时间差。时间差信息可利用无线电信号的相位来测量，也可采用脉冲信号的传播延时来测量。奥米加系统利用信号的相位信息来测量距离差；罗兰A系统利用脉冲信号的传播延时来测量距离差；而罗兰C系统则同时使用脉冲信号的传播延时和载波的相位信息来测量距离差，其中脉冲的时间信息用于获得粗测读数，即获得读数的高位数部分，相位信息用于获得精测读数，即获得读数的低位部分。罗兰A已经淘汰。下面对奥米加系统和罗兰C系统作简要介绍。

## 1. 奥米加系统

奥米加系统的地面台全世界共有8个，它们分别用前8个英文字母命名，即A台、B台、…、H台。工作频率为10.2 kHz，此外还发射13.6 kHz和11.33 kHz等频率的信号。8个台的工作频率全都相同，区别在于各台发射某一频率的信号时所占用的时间段顺序不相同。各台发射的信号格式如图8.3.14所示。由图知，各台的发射周期为10 s，各时间段内当频率发生转换时有0.2 s的停顿间隙。信号的区分在于时间段的分割方式。诸频率中，10.2 kHz用于定位，13.6 kHz和11.33 kHz用于巷区识别，以解决定位模糊问题，这些频率称主频率，其余频率称副频率。

由于奥米加系统发射的信号属于长波,而长波的穿透力强,传播衰减小,传播时的相位稳定,所以奥米加系统的作用距离远,8个发射台的信号能覆盖全球范围,即在地球上的任何地方至少能收到3个奥米加台的信号。

由于奥米加系统采用时间分割方式工作，所以各发射台间时间必须严格同步，各台都配备有高稳定度原子种，可达300年内只差1 s的精度。用户接收奥米加信号时，必须实现精确的时间段同步，以便识别各台所发射的10.2 kHz主频信号。为此，接收机中采用了 $2\times10^{-9}/d$ 高稳定度的晶体振荡器产生本机基准信号。

若使用 A 台和 B 台确定位置线(双曲线)，则分别测出 A 台信号相对本机基准的相位差 $\varphi_{A}^{r}$ ，即 A 台的相对相位差，以及 B 台的相对相位差 $\varphi_{B}^{r}$ 。从 $\varphi_{A}^{r}$ 和 $\varphi_{B}^{r}$ 确定出 A，B 台信号的相位差，从而确定出位置线。

设本机参考信号的相位为

$$
\varphi_ {0} = \omega_ {0} t
$$

接收到的 A 台和 B 台的相位分别为

$$
\varphi_ {\mathrm{A}} = \omega_ {0} \left(t - \frac {d _ {\mathrm{A}}}{c}\right)
$$

$$
\varphi_ {\mathrm{B}} = \omega_ {0} \left(t - \frac {d _ {\mathrm{B}}}{c}\right)
$$

式中 $\omega_{0}$ 为奥米加系统的主频率， $d_{A}$ 和 $d_{B}$ 分别为用户到 A 台和 B 台的距离，c 为光速，本机参考信号与发射信号严格同步。则 A 台信号和 B 台信号的相对相位差分别为

$$
\varphi_ {\mathrm{A}} ^ {r} = \varphi_ {0} - \varphi_ {\mathrm{A}} = \omega_ {0} \frac {d _ {\mathrm{A}}}{c}
$$

$$
\varphi_ {\mathrm{B}} ^ {r} = \varphi_ {0} - \varphi_ {\mathrm{B}} = \omega_ {0} \frac {d _ {\mathrm{B}}}{c}
$$

所以用户至 A 台和 B 台的距离差为

$$
d _ {\mathrm{A}} - d _ {\mathrm{B}} = \frac {c}{\omega_ {0}} (\varphi_ {\mathrm{A}} ^ {r} - \varphi_ {\mathrm{B}} ^ {r})\tag{8.3.44}
$$

从而可得到由 A 台和 B 台确定的位置线。

![](images/150d792d42b5897ae0d39c018ddfbeb74ad650694dfced85739bc42051bea197.jpg)

![](images/1822a03d1b77aef1aecb659b3c591fd8724eabb8637649a5bd1be647fd8fa532.jpg)  
图 8.3.14 奥米加系统在 10 s 周期内各发射台的信号发射格式

## 2. 罗兰 C 导航系统

罗兰 C 是一种相位-时间测距离差双曲线定位系统，工作频率为 $90 \sim 110 \, kHz$ 。罗兰 C 的发射系统至少需由 3 个地面台站构成台链，一般由 4 个台站构成台链。在一个台链中，有一个台为主台，记为 M，其余台为副台，记为 X, Y, Z, W，台链的各台位置一般采用三角形、Y 形或星

形配置，如图8.3.15所示。图中M为主台，其余为副台。主台和副台的配置形式取决于工作区的覆盖和定位精度要求。主、副台之间的基线长度为 $500\sim 1000\mathrm{n}$ mile，系统的作用距离约为1000n mile。对不同的台链，各台链发射脉冲信号的重复频率是不一样的，即采用频率分割方式来区分不同台链的信号。而在同一个台链内，各台采用时间分割方式发射信号，即在一个发射周期内，主台先发射，然后副台依次发射。且所有的副台在发射时间上和发射载波相位上都与主台同步，以保证双曲线格网的稳定。目前，在罗兰C系统中，常采用高精度的原子钟作为时频标准（铷原子钟的频率稳定度为 $10^{-11}/d$ ，铯原子钟则为 $10^{-12}/d$ ），使各台链中各台既能独立地工作，又能确保发射信号在时间和相位上的同步。

![](images/82d279be978da3987368bcf623731cf1c0737e2eb9e55372b5f1f4bcc2d802bc.jpg)  
(a)

![](images/ecaeda0593a633cb19e8f792dd32acae234f9d87bd9c5802c723f04d46b4ad57.jpg)  
(b)

![](images/a9738742fcae72300f5b74d02904f98e096baa85a1976f2b951adb5f4397f30c.jpg)  
(c)  
图 8.3.15 罗兰 C 台链的几何配置  
(a) 三角形；(b) 星形；(c) Y 形

在同一台链中，主、副台的发射格式的顺序有严格的规定，如图8.3.16所示。主、副台间歇地发射具有确定格式的信号脉冲组。在同一个发射周期内，主台发射的脉冲组中共有9个脉冲，前8个脉冲间的间隔都是 $1000\ \mu s$ ，第8个和第9个脉冲间的间隔为 $2000\ \mu s$ 。各副台发射的脉冲组中都包含8个脉冲，各脉冲间隔为 $1000\ \mu s$ 。各台的工作顺序是：主台最先发射，经 $\tau_{X}$ 的时间休止后，第一副台发射，经 $\tau_{Y}$ 休止后第二副台发射，……。发射休止时间 $\tau_{X},\tau_{Y},\tau_{Z},\tau_{W}$ 取值的不同使发射周期不同，从而使发射重复频率不同，因此休止时间可用来区分不同的台链。主台的第9个脉冲用于构成识别台链中各台工作是否正常的编码。

![](images/889a40f4f520391764dfd09ec21756f1254dec80f8fba484ad62101d2ca6bdb6.jpg)  
图 8.3.16 罗兰 C 信号脉冲组格式和发射顺序

各脉冲信号是钟形脉冲对载波信号的调制,调制脉冲的前沿有严格的要求,发射天线电流脉冲波形所满足的数学表达式为

$$
i (t) = \left\{ \begin{array}{l l} 0, & t <   \tau_ {d} \\ A _ {m} (t - \tau_ {d}) ^ {2} \exp \left[ - \frac {2 (t - \tau_ {d})}{6 5} \right] \sin (0. 2 \pi + P _ {c}), \tau_ {d} <   t <   6 5 + \tau_ {d} \end{array} \right.\tag{8.3.45}
$$

式中： $A_{m}$ 为无线电流峰值，单位为 A；t 为时间，单位为 $\mu s; \tau_{d}$ 为包周差，单位为 $\mu s; P_{c}$ 为相位编码参数，单位为 rad，正相位编码对取值为零，负相位编码时取值为 $\pi$ 。在相邻的两个发射周期内，相位编码如表 8.3.1 所列。可见，相位周期是发射周期的两倍。

表 8.3.1 相位编码

<table><tr><td>周期名\台名</td><td colspan="9">主 台</td><td colspan="8">副 台</td></tr><tr><td>前一发射周期</td><td>+</td><td>+</td><td>-</td><td>-</td><td>+</td><td>-</td><td>+</td><td>-</td><td>+</td><td>+</td><td>+</td><td>+</td><td>+</td><td>+</td><td>-</td><td>-</td><td>+</td></tr><tr><td>后一发射周期</td><td>+</td><td>-</td><td>-</td><td>+</td><td>+</td><td>+</td><td>+</td><td>+</td><td>-</td><td>+</td><td>-</td><td>+</td><td>-</td><td>+</td><td>+</td><td>-</td><td>-</td></tr></table>

发射信号如图 8.3.17 所示。

![](images/5417b47444550e2c3cb378a3c00a3b6588154b87fd257c7562821e74a1f3d40d.jpg)  
图 8.3.17 罗兰 C 发射脉冲信号

罗兰 C 的用户设备是罗兰 C 接收机, 其任务是: 测量主、副台发射的同步脉冲信号到达的

时间差；测量主、副台脉冲信号内 $100\mathrm{kHz}$ 同步载波信号的相位差。图8.3.18示出了由脉冲确定时差的粗测原理，其中图（a）表示接收到的主、副台信号间的时间关系，这些信号经检波得到钟形脉冲调制信号，并作 $5\mu \mathrm{s}$ 滞后处理，所得信号如图（b）所示；将图（b）信号作反相处理，所得结果如图（c）所示；将图（a）的包络与图（c）信号相加，所得波形如图（d）所示。将合成包络振幅从正移向负的零交会点（位于脉冲前沿30 $\mu \mathrm{s}$ 处）作为基准，产生采样波门。设分别对应于主台零交会点和副台零交会点的波门间的时间长度为 $\tau_{\mathrm{MX}}$ ，由于副台X发射的脉冲信号相对主台M具有固定的延迟 $\tau_{\mathrm{X}}$ ，所以主、副台间的传播时间差为 $\tau_{\mathrm{MX}} - \tau_{\mathrm{X}}$ ，从而可确定出一条位置线。但主、副台脉冲形成器输出的脉冲序列各自对应于主、副台脉冲组信号包络内的哪一个载波

![](images/f2844469b61b188874063d3174c4cf3dcb35985dd1a354113f17b2b341d91f06.jpg)  
图 8.3.18 接收信号、取样点和包络的测定  
(a) 接收信号；(b) $5 \mu s$ 移相；(c) 图(b) 的倒相  
(d) 图(a) + 图(c); (e) 波门对准采样点

周期是不确定的,这样包络时差测定就不够准确,所以这仅仅是对传播时差的粗测。

为了更精确地测定传播时差,需进行主、副台载频的相位比较,即进行载波的周期匹配。所谓周期匹配,就是使主、副台门脉冲形成器输出的脉冲序列,分别锁住来自主、副台脉冲组中每个脉冲所合载波的第三周过零点,即图8.3.17中所标的过零点。选用第三周的原因是:第一、第二周的信号较弱,信噪比低,过零采样点不能有效地确定;第三周以后的信号虽然幅度较大,但有可能受到天波的干扰。

图 8.3.19 示出了实现周期匹配的原理方框图，图 8.3.20 为周期匹配环路中相应的波形图。为了进行周期匹配，对于相位解码电路输出的每个脉中信号在同门脉冲序列比相之前要进行如下处理：解调的射频脉冲信号分成两路，一路放大 1.35 倍再移相 $180^{\circ}$ 与另一路相加，使相加后的输出信号正好在载频第三周处出现包络零点，见图 8.3.20。

![](images/afdd6e3d73a31b1c75d049fffb39e69e9d1ec8602601b133a7f1cea760060282.jpg)  
图 8.3.19 周期匹配锁相环原理框图

![](images/d89c051fe9e875aceb81053b971765e42dbd0a51e142a32d4a4583679bb54106.jpg)

![](images/62ca5b20e73038fccb0c7a64567f6a7dcf9302d00c8d6584d2cb07b3a437c7da.jpg)  
图 8.3.20 周期匹配环路中相应的波形  
(a) 接收和放大倒相后的钟形脉冲；(b) 相加器输出；  
(c) 相位检波器输出；(d) 放大限幅器输出；(e) 采样脉冲

经过相位检波,取出具有正负极性的包络,在放大限幅电路中,变成双极性不对称方波后加到零点采样器,门脉冲形成电路输出的采样脉冲(宽度约为 $5\ \mu s$ )也加到零点采样器,对不对称方波进行过零采样。如果采样脉冲中线不对准采样点,则积累滤波电路就有正或负误差信号输出,正或负极性取决于采样脉冲相对采样点是滞后还是超前。振荡器在误差信号的控制下,相应于误差信号的极性,使其输出信号的相位超前(误差信号为正)或滞后(误差信号为负)。这样,采样脉冲也相应地向前或向后移动,直到采样脉冲对准采样点为止。这种周期匹配过程,在主、副锁相环路中以同样的方式进行。当主、副环周期匹配完成时,便能得到精测时差,其数值以 $\mu s$ 和 $0.1\ \mu s$ 显示在计数器上。

粗测与精测读数相结合,即为用 $\mu s$ 表示的时间差读数。同时可用相同的方法,自动测出另一对主台与副台之间的时间差。目前，以自动连续跟踪方式工作时，每秒钟至少可以测量5次，连续显示两条位置线，并可自动算出用户的地理位置。

影响罗兰 C 精度的因素主要有：

（1）双曲线位置线几何因子的发散特性，导致定位准确度随用户距发射台的距离增加而降低。

(2) 电波传播途中, 经过不规则、不均匀地带时, 由于传播条件的改变而产生测距误差。

（3）系统设备本身存在误差，特别是当信噪比降低到零分贝以下时，对测量误差的影响尤其不能忽视。

(4) 罗兰 C 台链发射机在稳定性方面存在 $\pm0.01\sim\pm0.05\mu s$ 的误差。

(5) 主、副台之间的同步误差, 一天之内约为 $0.03 \sim 0.06 \mu s$ 。

考虑到上述因素,可给出罗兰 C 地波定位准确度范围:

370 km 距离上, 定位准确度为 15 \~ 90 m;

925 km 距离上, 定位准确度为 60 \~ 210 m;

1 390 km 距离上，定位准确度为 90～340 m；

1 850 km 距离上，定位准确度为 150～520 m。

利用天波接收可以获得更大的作用距离,但由于电离层不稳定,而使定位准确度降低。例如在2780 km距离上,定位准确度降低到约18 km;在3700 km距离上,定位准确度降低到31 km。

## 8.3.6 磁航向仪

磁航向仪使用磁敏感器件感测地球磁场的方向,即磁子午线方向,并给出飞机纵轴相对磁子午线在水平面内的夹角,即磁航向角。

地球可视为一块大磁铁，地磁北极近期位于地理北纬 $70^{\circ}$ ，西经 $100^{\circ}$ ，地磁南极位于地理南纬 $68^{\circ}$ ，东经 $143^{\circ}$ 。磁子午线并非象地理子午线一样固定不变，而是随时间和在地球上的不同位置而变化。主要体现在以下几方面[85]：

（1）从宏观角度看，地磁场的极性在缓慢地改变，地磁场的南北极约经50万年便会翻转一次，完成一次翻转所需时间约为5千年，在过去的400万年内，地磁场已翻转过9次。据测定，在过去的300年内，伦敦磁偏角由西向 $4^{\circ}$ 变至西向 $23^{\circ}$ 后又变至西向 $10^{\circ}$ ，磁倾角从 $75^{\circ}$ 变至 $67^{\circ}$ ；巴黎的磁偏角由西向 $2^{\circ}$ 变至西向 $22^{\circ}$ 后又变至西向 $8^{\circ}$ ，磁倾角由 $75^{\circ}$ 变至 $64^{\circ}$ 。

（2）太阳和月亮的周期变化会影响地磁场，这是由于这些天体影响了电离层。地磁场的日变化量不超过零点几度，年变化量也是如此。由太阳耀斑引起的磁暴约为 $5^{\circ}$ ，并能持续数日。

（3）铁矿磁场有的比地磁场强得多，这就是磁场异常，在这些地方是无法使用磁航向仪的。

用于车辆导航的磁航向仪还受以下因素的影响：

（1）城市建筑、地下建筑、桥梁和其他钢铁结构会引起罗差，特别是在车辆使用直流电驱动的区域内，磁航向仪根本无法使用。

（2）当直流电驱动的机车驶过时，特别是正在加速行驶时，附近的车辆立即被磁化，所有原来作的罗差补偿全部失效，车体也不再会恢复到原来的磁场结构。

尽管磁航向仪受外界的影响大,精度也不易提高,但由于结构简单,仪表本身的可靠性高,成本又低,所以仍被很多使用场合采用,在飞机上常被用做应急航向设备。

飞机用磁航向仪必须经过严格的磁差补偿和罗差补偿。所谓罗差补偿，就是消除飞机电器和铁磁结构形成的环境磁场对磁航向仪的影响。磁差补偿是指消除飞机所在点磁偏角和磁倾角影响，根据磁航向获得真航向。经过磁差和罗差补偿后，磁航向仪的误差可近似用一阶马尔柯夫过程来描述

$$
\delta \dot {\Psi} _ {\mathrm{MG}} = - \frac {1}{\tau_ {\mathrm{MG}}} \delta \Psi_ {\mathrm{MG}} + w _ {\mathrm{MG}}\tag{8.3.46}
$$

其中均方根为 $30'$ ，相关时间 $\tau_{GM}$ 约为数十秒至数百秒，视飞机速度而定，速度越高，相关时间越短，反之则越长。

## 8.3.7 垂直陀螺仪

垂直陀螺仪是指示飞机姿态的应急备用设备。由一个框架式双自由度陀螺和液体摆构成闭环控制系统，使陀螺的自转轴稳定在当地地垂线方向上。飞机的俯仰角和横滚角通过测量陀螺的内、外环架角读出。

影响垂直陀螺仪姿态测量精度的主要因素为陀螺漂移和同步器精度。影响陀螺漂移的因素有:由于飞机的运动干扰产生的干扰加速度对陀螺的误修正,作用在框架轴承上的摩擦力矩,这些都将产生陀螺的随机漂移。这种随机漂移又可分为三种成分:随机常值漂移 $\varepsilon_{b}$ ,相关漂移 $\varepsilon_{r}$ 和不相关漂移 $\varepsilon_{w}$ 。所以垂直陀螺仪沿两个输出轴的随机漂移为

$$
\varepsilon_ {i} = \varepsilon_ {b i} + \varepsilon_ {r i} + \varepsilon_ {w i} \quad i = x, y\tag{8.3.47}
$$

式中

$$
\dot {\varepsilon} _ {b i} = 0 \quad i = x, y\tag{8.3.48}
$$

$$
\dot {\varepsilon} _ {n} = - \frac {1}{\tau_ {i}} \varepsilon_ {n} + w _ {i} \quad i = x, y\tag{8.3.49}
$$

式中随机常值漂移和相关漂移的均方根分别为 $10^{\circ}/h$ 和 $1^{\circ}/h$ 左右，相关时间约为1000 s，不相关漂移的方差强度约为 $(0.1\sim0.5)^{\circ}/\sqrt{h}$ 。

陀螺的输出姿态角误差为

$$
\delta \dot {\theta} = \varepsilon_ {x}\tag{8.3.50}
$$

$$
\delta \dot {\gamma} = \varepsilon_ {y}\tag{8.3.51}
$$

式中 $\delta\theta$ 和 $\delta\gamma$ 分别为俯仰角误差和横滚角误差，机体坐标系的 $x_{b}, y_{b}, z_{b}$ 轴分别沿飞机的右、前、上方向，同步器误差可归并入 $\delta\theta$ 和 $\delta\gamma$ 的初值中考虑。

## 8.3.8 天文导航系统

天文导航利用对星体的观测数据来确定航行体的位置,这门技术早在两千年前就应用于航海。随着科学技术的发展,20世纪60年代出现了高精度星体自动跟踪器,能在白天观测三等星，晚上观测七等星，并有自动寻星、搜索和跟踪等功能。

为便于说明天文导航的原理,下面介绍一些有关定义和关系。

设载体位于地球上的 P 点，该点的经、纬度分别为 $\lambda$ 和 L，如图 8.3.21 所示。

![](images/bd7c499d46711bd8582a671ac1f20abf97cacaceb9638e3f79f2c4f7c5b3e6bd.jpg)  
图8.3.21 $P$ 点在地球上的经、纬度

![](images/2836f8444bca2db13b29f6f1b4ca6cc081739b1b5547edd005ae50f2c65fd50b.jpg)  
图8.3.22 星体 $S$ 的方位角和高度角

设 S 为某星体, 已测得 S 的高度角 h 和方位角 $\Psi$ , 如图 8.3.22 所示, 其中高度角是星视线 PS 与 P 点处水平面的夹角, 方位角是星视线的水平投影与 P 点处南向间的夹角。

以 P 为球心,无穷大为半径作圆球,则该圆球称为天球,如图 8.2.23 所示。

![](images/39bb7f902dad0ddd99290ad491ef932300b592dca7e2bc7aa79885954040d71b.jpg)  
图8.3.23 星体 $S$ 在天球中的位置

过 P 点且平行于地球赤道面的平面截天球所得的圆称天球赤道。

地球的公转平面截天球所得的圆称天球黄道。

天球赤道平面与天球黄道平面的交线与天球相交得两个交点,其中一点称春分点,另一点称秋分点。由于地球的自转和公转相对惯性空间是恒定的,所以春分点和秋分点是惯性空间内的两个固定点。又由于春分点位于无穷远处，所以地球上的任何一点与春分点的连线可看做是重合的。

过 P 点平行于地球自转轴的直线称天轴。

P 点处的垂线称 P 点处的天顶线。

设春分点为 $\Gamma$ ，星视线 $PS$ 在天球赤道内的投影的延长线为 $PS''$ ，则 $\angle GPS'' = \alpha$ 称为星体 $S$ 的天球赤经， $\angle S''PS = \delta$ 称为星体 $S$ 的天球赤纬，星体的天球赤经和天球赤纬均可在天文年历内查得。

根据上述定义,可得如下关系:

(1) 星视线 PS 与天顶线 PZ 间的夹角为 $90^{\circ}-h$ ;

(2) 天轴北 $PP_{N}$ 与天顶线 PZ 间的夹角为 $90^{\circ}-L$ ;

(3) 天轴北 $PP_{N}$ 与星视线 PS 间的夹角为 $90^{\circ}-\delta$ 。

因此在球面三角形 ASC 中的边角关系如图 8.3.24 所示, 其中:

$$
A = \tau = \lambda + t _ {G} - \alpha\tag{8.3.52a}
$$

$$
\eta = 9 0 ^ {\circ} - h\tag{8.3.52b}
$$

$$
\beta = 9 0 ^ {\circ} - L\tag{8.3.52c}
$$

$$
\gamma = 9 0 ^ {\circ} - \delta\tag{8.3.52d}
$$

$$
C = 1 8 0 ^ {\circ} - \Psi\tag{8.3.52e}
$$

上述诸式中，h 和 $\Psi$ 分别为星体 S 的高度角和方位角； $t_{G} = \omega_{ie} t$ 为本初子午线的赤经； $\alpha$ 和 $\delta$ 分别为星体 S 的赤经和赤纬，由天文年历查得； $\lambda$ 和 L 分别为 P 点即用户所在地的经、纬度。

根据球面三角中的边余弦定理：

$$
\cos \eta = \cos \beta \cos \gamma + \sin \beta \sin \gamma \cos A
$$

将式 $(8.3.52)$ 代入得

$$
\cos (9 0 ^ {\circ} - h) = \cos (9 0 ^ {\circ} - L) \cos (9 0 ^ {\circ} - \delta) + \sin (9 0 ^ {\circ} - L) \sin (9 0 ^ {\circ} - \delta) \cos \tau
$$

即

$$
\sin h = \sin L \sin \delta + \cos L \cos \delta \cos (\lambda + t _ {G} - \alpha)
$$

该式说明,要求取 P 点的经、纬度,须要测量两颗星体的高度角 $h_{1}$ 和 $h_{2}$ 。

$$
\left. \begin{array}{l} \sin h _ {1} = \sin L \sin \delta_ {1} + \cos L \cos \delta_ {1} \cos (\lambda + t _ {G} - \alpha_ {1}) \\ \sin h _ {2} = \sin L \sin \delta_ {2} + \cos L \cos \delta_ {2} \cos (\lambda + t _ {G} - \alpha_ {2}) \end{array} \right\}\tag{8.3.53}
$$

![](images/71c1b0f70916e383d83e749dd1b0586df006268aa533f6432ff7ae049dda8170.jpg)  
图 8.3.24 球面三角形中的边角关系

(8.3.54)

式 $(8.3.54)$ 描述了双星定位原理。

也可以同时测量一颗星的方位角 $\Psi$ 和高度角 h 来确定 $\lambda$ 和 L。对如图 8.3.24 所示球面三角形应用角余切定理

$$
\cot C \sin A = \cot \gamma \sin \beta - \cos A \cos \beta
$$

将式 $(8.3.52)$ 诸关系代入上式,得

$$
- \cot \Psi \sin \tau = \tan \delta \cos L - \cos \tau \sin L
$$

即

$$
\cot \Psi = \frac {\cos (\lambda + t _ {G} - \alpha) \sin L - \cos L \tan \delta}{\sin (\lambda + t _ {G} - \alpha)}\tag{8.3.55}
$$

联立求解式(8.3.53)和式(8.3.55)，即可确定出 $\lambda$ 和L。

天文导航的误差主要来自高度角和方位角的测量误差，根据文献[86]，这种测量误差可视为白噪声，均方根约为 $10^{\prime \prime}$ 。但测量星体高度角和方位角需要有水平基准和方位基准，而这些基准常由惯导系统提供，惯导的水平精度远比方位精度高，所以工程上常测两颗恒星的高度角实现定位。

## 8.4 惯性导航系统的误差模型

## 8.4.1 惯性导航系统的基本工作原理

## 1. 概述

惯性导航系统是十分复杂的高精度机电综合系统，只有当科学技术发展到一定高度时工程上才能实现这种系统，但其基本工作原理却以经典的牛顿力学为基础。

![](images/914b2059c2bbbcc30bf2ce185e01f6e9f219ad252a29985ebce94d94056dadb1.jpg)  
图8.4.1 运载体上的加速度计

如图 8.4.1 所示, 表示加速度计安装在运载体上的情况。设质量 m 受弹簧约束, 悬挂弹簧的壳体固定在载体上, 载体以加速度 a 作水平运动, 则 m 处于平衡后, 所受到的水平约束力 F 与 a 的关系满足牛顿第二定律:

$$
a = \frac {F}{m}
$$

测量水平约束力 F，即可求得 a，对 a 积分一次，即得水平速度，再积分一次即得水平位移。

以上所述是简单化了的理想情况。由于运载体不可能只作水平运动，当有姿态变化时，必须测得沿固定坐标系的加速度，所以加速度计必须安装在惯性平台上，平台靠陀螺维持要求的空间角位置，导航计算和对平台的控制由计算机完成。

参与控制和测量的陀螺和加速度计称为惯性器件, 这是因为陀螺和加速度计都是相对惯性空间测量的, 也就是说加速度计输出的是运载体的绝对加速度, 陀螺输出的是运载体相对惯性空间的角速度或角增量。而加速度和角速度或角增量包含了运载体运动的全部信息, 所以惯导系统仅靠系统本身的惯性器件就能获得导航用的全部信息, 它既不向外辐射任何信息, 也不需要任何其他系统提供的外来信息,就能在全天候条件下,在全球范围内和所有介质环境里自主、隐蔽地进行三维导航,也可用于外层空间的三维导航。惯导系统的自主性和隐蔽性对军事应用特别重要,所以尽管无线电导航系统和卫星导航系统精度的长期稳定性远比惯导好,但惯导系统仍然是重要运载体不可缺少的重要导航设备。

惯导在重力场内能正常工作的关键是建立起不受运动干扰影响的人工地垂线。德国科学家休拉(M. Schuler)在1923年指出:当垂线指示系统的自振周期为84.4 min时,人工垂线能自动跟踪当地垂线,而与运载体的运动状态无关。这就是休拉摆原理。

重力摆可以用来指示垂线,但当摆的悬挂点作加速运动时,指示垂线就会偏离真实垂线。如果要使单摆成为休拉摆,则摆线长度必须等于地球半径;如果要使物理摆成为休拉摆,则摆的悬挂中心距摆的质心仅为数十纳米( $1\ nm=1/10^{9}\ m$ ,约为10个中等原子的直径和)。这些要求在物理上都是无法实现的,而惯导系统能够严格实现休拉调谐成为休拉摆。惯性平台是惯导系统实现休拉调谐的核心部件,惯性平台具有两个基本控制回路:稳定回路和修正回路。稳定回路的作用是隔离运载体的角运动,修正回路的作用是使平台始终跟踪当地水平面,这样,为安装在惯性平台上的陀螺和加速度计建立起一个不受运动影响的测量基准。

根据构建惯性平台的方法不同可将惯导系统分为两大类:采用物理方法构建平台的系统称为平台式惯导系统;采用数学算法构建平台的系统称为捷联式惯导系统。根据物理平台模拟的坐标系类型不同,平台式惯导系统又可分为两类:若平台模拟惯性坐标系,则系统称为解析式惯导系统;若平台模拟当地水平坐标系,则系统称为当地水平式惯导系统。根据平台跟踪地球自转角速度和跟踪水平坐标系类型的不同,当地水平式惯导系统又可分为三种:若平台跟踪地理坐标系(必然要跟踪地球自转角速度),则系统称指北方位系统;若平台跟踪地球自转角速度和当地水平面,则系统称游移方位系统;若平台跟踪地球自转角速度的水平分量和当地水平面,则系统称自由方位系统。

## 2. 比力方程

惯导系统根据与系统类型相应的数学方程(称之为力学编排)对惯性器件的输出作处理,从而获得导航数据。尽管各种类型的系统相应的力学编排各不相同,但它们都源自同一个方程——比力方程。比力方程描述了加速度计输出量与运载体速度之间的解析关系 $^{[87\sim90]}$ :

$$
\left. \frac {\mathrm{d} \boldsymbol {v} _ {e T}}{\mathrm{d} t} \right| _ {T} = \boldsymbol {f} - (2 \boldsymbol {\omega} _ {i e} + \boldsymbol {\omega} _ {e T}) \times \boldsymbol {v} _ {e T} + \boldsymbol {g}\tag{8.4.1}
$$

式中， $v_{eT}$ 为运载体的地速向量；f 为比力向量，是作用在加速度计质量块单位质量上的非引力外力，由加速度计测量；g 为重力加速度； $\omega_{ie}$ 为地球自转角速度； $\omega_{eT}$ 为惯性平台所模拟的平台坐标系 T 相对地球的旋转角速度； $\frac{dv_{eT}}{dt}\bigg|_{T}$ 表示在平台坐标系 T 内观察到的地速向量的时间变化率。

式(8.4.1)式说明,用加速度计的比力输出计算地速时,必须对比力输出中的三种有害加速度成分作补偿:

（1） $2\omega_{ie} \times v_{eT}$ ，即由地球自转（牵连运动）和运载体相对地球的运动（相对运动）引起的哥氏加速度；

(2) $\omega_{eT} \times v_{eT}$ ，即运载体保持在地球表面运动（绕地球作圆周运动）引起的相对地心的向心加速度；

(3) g, 即重力加速度。

## 3. 水平式平台惯导系统的指令角速度

地球是球体,当运载体在地球表面运动时,水平面将连续地转动,同时,地球的自转运动又带动当地水平面相对惯性空间旋转。所以,要使惯性平台模拟当地水平面,平台的修正回路必须控制平台跟踪两种角运动:地球的自转运动和运载体在地球表面运动时水平面相对地球的旋转运动,也就是说惯性平台的施矩角速度必须与这两种旋转运动的角速度相等。而要使平台旋转,只有通过对控制平台的相应陀螺施矩才能实现。在除陀螺力矩器之外的任何地方输入外作用,企图使平台旋转都是徒劳的,因为平台的稳定回路都将这些输入视为干扰而隔离掉。

为控制平台旋转而馈入陀螺力矩器的控制量(具体为电流)称为平台的指令角速度。指北方位系统、自由方位系统和游移方位系统的惯性平台模拟的当地水平坐标系是不一样的,所以各系统的指令角速度各不相同。

指北系统的惯性平台模拟当地的地理坐标系 $g$ ，其中 $x_{\mathrm{g}},y_{\mathrm{g}},z_{\mathrm{g}}$ 分别指向当地的东、北、天，所以指令角速度为

$$
\boldsymbol {\omega} _ {c m d} ^ {T} = \boldsymbol {\omega} _ {i g} ^ {g} = \boldsymbol {\omega} _ {i e} ^ {g} + \boldsymbol {\omega} _ {e g} ^ {g}\tag{8.4.2}
$$

式中，i 为惯性坐标系；T 为理想平台坐标系，即与 g 系完全重合的坐标系；上标表示向量在所标坐标系内的投影。

如图 8.4.2 所示给出了地球自转和运载体的水平速度引起的坐标系旋转角速度关系，其中 L 为运载体所在点 P 的纬度， $v_{E}$ 和 $v_{N}$ 分别为运载体的东向和北向水平速度分量。

![](images/e836324b1193179a2332e4c94556305920761a69551204a680b0c27bd07bb2bf.jpg)  
图 8.4.2 地理坐标系的角速度

由图得

$$
\pmb {\omega} _ {i e} ^ {T} = \left[ \begin{array}{c c} & 0 \\ \omega_ {i e} & \cos L \\ \omega_ {i e} & \sin L \end{array} \right]\tag{8.4.3}
$$

$$
\boldsymbol {\omega} _ {e T} ^ {T} = \left[ \begin{array}{l} - \frac {v _ {\mathrm{N}}}{R + h} \\ \frac {v _ {\mathrm{E}}}{R + h} \\ \frac {v _ {\mathrm{E}}}{R + h} \tan L \end{array} \right]\tag{8.4.4}
$$

这样，式(8.4.2)具体表达为

$$
\boldsymbol {\omega} _ {c m d} ^ {T} = \left[ \begin{array}{c} - \frac {v _ {\mathrm{N}}}{R + h} \\ \omega_ {\mathrm{ie}} \cos L + \frac {v _ {\mathrm{E}}}{R + h} \\ \omega_ {\mathrm{ie}} \sin L + \frac {v _ {\mathrm{E}}}{R + h} \tan L \end{array} \right]\tag{8.4.5}
$$

若考虑地球的椭球特性,上式应改写为

$$
\boldsymbol {\omega} _ {c m d} ^ {T} = \left[ \begin{array}{l} - \frac {v _ {\mathrm{N}}}{R _ {M} + h} \\ \omega_ {\mathrm{ie}} \cos L + \frac {v _ {\mathrm{E}}}{R _ {N} + h} \\ \omega_ {\mathrm{ie}} \sin L + \frac {v _ {\mathrm{E}}}{R _ {N} + h} \tan L \end{array} \right]\tag{8.4.6}
$$

式中， $R_{M}$ 为 P 点处子午圈（沿南北方向）的曲率半径， $R_{N}$ 为 P 点处卯酉圈（沿东西方向）的曲率半径，具体值为

$$
R _ {M} = R _ {e} (1 - 2 e + 3 e \sin^ {2} L)\tag{8.4.7a}
$$

$$
R _ {N} = R _ {e} (1 + e \sin^ {2} L)\tag{8.4.7b}
$$

$R_{e}$ 和 e 分别为椭球模型的半长轴和椭圆度，具体取值根据所选椭球模型确定，详见文献 $[87\sim89]$ 和 $[120]$ 。

从指令角速度表达式可看出，在高纬度地区，对方位陀螺的施矩量十分巨大，所以指北系统只能在中、低纬度地区使用。自由方位系统和游移方位系统就是为克服这一问题而提出的系统，这两种系统也跟踪当地水平面，但对方位陀螺的施矩与指北系统不同：自由方位系统对方位陀螺根本不施矩，让平台在方位上相对惯性空间处于自由状态；游移方位系统在方位上跟踪地球旋转引起的角速度分量，而并不跟踪东向速度引起的地理坐标系在方位上的旋转。

自由方位系统 F 和游移方位系统 $\omega$ 的指令角速度分别为

$$
\boldsymbol {\omega} _ {c m d} ^ {F} = \left[ \begin{array}{c} C _ {1 3} ^ {F} \omega_ {i e} + \omega_ {e F x} ^ {F} \\ C _ {2 3} ^ {F} \omega_ {i e} + \omega_ {e F y} ^ {F} \\ 0 \end{array} \right]\tag{8.4.8}
$$

$$
\boldsymbol {\omega} _ {c m d} ^ {w} = \left[ \begin{array}{c} C _ {1 3} ^ {w} \omega_ {i e} + \omega_ {e w x} ^ {w} \\ C _ {2 3} ^ {w} \omega_ {i e} + \omega_ {e w y} ^ {w} \\ C _ {3 3} ^ {w} \omega_ {i e} \end{array} \right]\tag{8.4.9}
$$

式中 $F$ 和 $\omega$ 分别为自由方位系统和游移方位系统的理想平台坐标系

$$
\omega_ {e F x} ^ {F} = - \frac {2 e}{R _ {e}} C _ {1 3} ^ {F} C _ {2 3} ^ {F} v _ {x} ^ {F} - \frac {1}{R _ {e}} \left[ 1 - e (C _ {3 3} ^ {F}) ^ {2} + 2 e (C _ {2 3} ^ {F}) ^ {2} \right] v _ {y} ^ {F}\tag{8.4.10a}
$$

$$
\omega_ {e F y} ^ {F} = \frac {1}{R _ {e}} [ 1 - e (C _ {3 3} ^ {F}) ^ {2} + 2 e (C _ {1 3} ^ {F}) ^ {2} ] v _ {x} ^ {F} + \frac {2 e}{R _ {e}} C _ {1 3} ^ {F} C _ {2 3} ^ {F} v _ {y} ^ {F}\tag{8.4.10b}
$$

$$
\omega_ {e w x} ^ {w} = - \frac {2 e}{R _ {e}} C _ {1 3} ^ {w} C _ {2 3} ^ {w} v _ {x} ^ {w} - \frac {1}{R _ {e}} \left[ 1 - e (C _ {3 3} ^ {w}) ^ {2} + 2 e (C _ {2 3} ^ {w}) ^ {2} \right] v _ {y} ^ {w}\tag{8.4.11a}
$$

$$
\omega_ {e w y} ^ {w} = \frac {1}{R _ {e}} \left[ 1 - e (C _ {3 3} ^ {w}) ^ {2} + 2 e (C _ {1 3} ^ {w}) ^ {2} \right] v _ {x} ^ {w} + \frac {2 e}{R _ {e}} C _ {1 3} ^ {w} C _ {2 3} ^ {w} v _ {y} ^ {w}\tag{8.4.11b}
$$

式中， $v_{x}^{F}, v_{x}^{w}$ 和 $v_{y}^{F}, v_{y}^{w}$ 分别为运载体沿相应坐标系 x 轴和 y 轴方向的速度分量，由求解比力方程确定； $R_{e}$ 和 e 同式（8.4.7）说明；C 为相应系统中方向余弦矩阵的元，由求解方向余弦矩阵微分方程确定。详细介绍请参阅文献 $[87 \sim 89]$ 和 $[120]$ 。

## 4. 捷联惯导系统的姿态更新和数学平台

捷联惯导系统并不存在物理平台，惯性器件直接固联在机体上，但就功能上来说，捷联惯导的平台是存在的，它以数学形式存在，所以称之为数学平台。它是由计算机作姿态更新求解后建立起来的。姿态更新的算法有很多种，如欧拉角法、方向余弦法、等效旋转矢量法、四元数法等，其中四元数法是最常用的一种。

四元数满足的方程为

$$
\left[ \begin{array}{c} \dot {q} _ {0} \\ \dot {q} _ {1} \\ \dot {q} _ {2} \\ \dot {q} _ {3} \end{array} \right] = \frac {1}{2} \left[ \begin{array}{c c c c} 0 & - \omega_ {x} & - \omega_ {y} & - \omega_ {z} \\ \omega_ {x} & 0 & \omega_ {z} & - \omega_ {y} \\ \omega_ {y} & - \omega_ {z} & 0 & \omega_ {x} \\ \omega_ {z} & \omega_ {y} & - \omega_ {x} & 0 \end{array} \right] \left[ \begin{array}{c} q _ {0} \\ q _ {1} \\ q _ {2} \\ q _ {3} \end{array} \right]\tag{8.4.12}
$$

式中 $\omega_{nb}^{b}=\left[\omega_{x}\quad\omega_{y}\quad\omega_{z}\right]^{T}$ 称为姿态速率，获取途径为

$$
\boldsymbol {\omega} _ {n b} ^ {b} = \boldsymbol {\omega} _ {i b} ^ {b} - \boldsymbol {C} _ {n} ^ {b} \boldsymbol {\omega} _ {c m d} ^ {n}\tag{8.4.13}
$$

式中，b 为机体坐标系，一般取右、前、上为正方向；n 为导航坐标系，一般取地理坐标系为导航坐标系，即东、北、天为正方向； $\omega_{ib}^{b}$ 为捷联陀螺的全姿态测量值（已经过动、静态误差补偿）； $C_{n}^{b}$ 为由上一次姿态更新确定的姿态阵； $\omega_{cmd}^{b}$ 为数学平台的指令角速度，若以地理坐标系作为导航坐标系，则按式(8.4.5)或式(8.4.6)确定。

求解式 $(8.4.12)$ 得四元数的更新值后,姿态阵按下式确定

$$
\mathbf {C} _ {b} ^ {n} = \left[ \begin{array}{l l l} T _ {1 1} & T _ {1 2} & T _ {1 3} \\ T _ {2 1} & T _ {2 2} & T _ {2 3} \\ T _ {3 1} & T _ {3 2} & T _ {3 3} \end{array} \right] = \left[ \begin{array}{l l l} q _ {0} ^ {2} + q _ {1} ^ {2} - q _ {2} ^ {2} - q _ {3} ^ {2} & 2 (q _ {1} q _ {2} - q _ {0} q _ {2}) & 2 (q _ {1} q _ {3} + q _ {0} q _ {2}) \\ 2 (q _ {1} q _ {2} + q _ {0} q _ {3}) & q _ {0} ^ {2} - q _ {1} ^ {2} + q _ {2} ^ {2} - q _ {3} ^ {2} & 2 (q _ {2} q _ {3} - q _ {0} q _ {1}) \\ 2 (q _ {1} q _ {3} - q _ {0} q _ {2}) & 2 (q _ {2} q _ {3} + q _ {1} q _ {0}) & q _ {0} ^ {2} - q _ {1} ^ {2} - q _ {2} ^ {2} + q _ {3} ^ {2} \end{array} \right]\tag{8.4.14}
$$

运载体的俯仰角 $\theta$ ，横滚角 $\gamma$ 和方位角 $\Psi$ 确定如下

$$
\theta = \arcsin T _ {3 2}\tag{8.4.15a}
$$

$$
\gamma_ {\mathrm{主}} = \arctan \left(- \frac {T _ {3 1}}{T _ {3 3}}\right)\tag{8.4.15b}
$$

$$
\Psi_ {\mathrm{主}} = \arctan \left(- \frac {T _ {1 2}}{T _ {2 2}}\right)\tag{8.4.15c}
$$

$\gamma$ 和 $\Psi$ 的真值按表 8.4.1 和表 8.4.2 确定。

表 8.4.1

<table><tr><td> $\gamma_{\text{主}}$ </td><td> $T_{33}$ </td><td> $\gamma$ </td></tr><tr><td>+</td><td rowspan="2">+</td><td rowspan="2"> $\gamma_{\text{主}}$ </td></tr><tr><td>-</td></tr><tr><td>+</td><td>-</td><td> $\gamma_{\text{主}} - 180^{\circ}$ </td></tr><tr><td>-</td><td>-</td><td> $\gamma_{\text{主}} + 180^{\circ}$ </td></tr></table>

表 8.4.2

<table><tr><td> $\Psi_{\text{主}}$ </td><td> $T_{22}$ </td><td> $\Psi$ </td></tr><tr><td>+</td><td>+</td><td> $\Psi_{\text{主}}$ </td></tr><tr><td>-</td><td>-</td><td> $180^{\circ} + \Psi_{\text{主}}$ </td></tr><tr><td>+</td><td>-</td><td> $180^{\circ} - \Psi_{\text{主}}$ </td></tr><tr><td>-</td><td>+</td><td> $360^{\circ} + \Psi_{\text{主}}$ </td></tr></table>

从上述对捷联惯导的简单介绍可看出： $C_{b}^{n}$ 的确定实际上建立起了数学平台；若取地理坐标系作为导航坐标系，则捷联惯导系统完全等效为指北方位系统。由于捷联惯导系统对数学平台的施矩只体现在算法上，所以不存在指北方位系统中施矩量受物理条件限制的问题，但仍受计算机字距的限制。

## 8.4.2 惯导系统的误差方程

平台式惯导系统一般都是游移方位系统，如 $\mathrm{LTN} - 72$ 等系统，这是由于游移方位系统既能避免指北方位系统中对方位陀螺的施矩受纬度的限制，又比自由方位系统计算量小，所以工程上一般都采用这种力学编排。

随着激光陀螺制造工艺的成熟和性能的提高，激光陀螺捷联惯导系统正逐渐普及并取代平台式惯导系统，美国利登公司自1995年起停止了LTN-72生产而转向LTN-90的批量生产。捷联惯导系统与平台式惯导系统的主要区别在于平台的构建方式上，前者采用数学方式，后者采用物理方式，但在本质上两种系统是一样的。唯一的差异在于陀螺漂移对姿态误差的影响。在平台式惯导中，陀螺的作用是控制惯性平台的角运动，陀螺漂移直接引起惯性平台漂移，两者符号相同，即正向陀螺漂移引起的平台漂移也为正向，反之亦然。而在捷联惯导中，陀螺仅用作测量元件，陀螺漂移与数学平台的漂移方向相反，即正向陀螺漂移引起数学平台的负向漂移，反之亦然。

此处以平台式惯导系统为例推导其误差方程，改变陀螺漂移的符号即可获得捷联惯导的误差方程。关于捷联惯导误差方程的详细推导可参阅文献[120]。

## 1. 姿态误差方程和速度误差方程的一般形式

设惯导系统的平台要求模拟的导航坐标系为 n，这就是理想平台坐标系 T。而实际建立的平台坐标系为 P。由于计算误差、误差源影响及施矩误差，P 坐标系相对要求的 T 坐标系有偏差角 $\varphi$ 。显然 $\varphi$ 是以 T 为基准观察到的，所以

$$
\dot {\boldsymbol {\varphi}} = \boldsymbol {\omega} _ {i P} - \boldsymbol {\omega} _ {i T}
$$

设陀螺的刻度系数误差为 $\delta K_{Gx}$ ， $\delta K_{Gy}$ ， $\delta K_{Gz}$ ，漂移为 $\varepsilon$ ，平台的实际指令为

$$
\boldsymbol {\omega} _ {c m d} = \boldsymbol {\omega} _ {i n} ^ {n} + \delta \boldsymbol {\omega} _ {i n}
$$

式中 $\delta \omega_{in}$ 为偏开理想值 $\omega_{in}^{n}$ 的偏差，它由导航误差引起。所以平台的实际角速度为

$$
\boldsymbol {\omega} _ {i P} ^ {P} = (\boldsymbol {I} + \delta \boldsymbol {K} _ {G}) (\boldsymbol {\omega} _ {i n} ^ {n} + \delta \boldsymbol {\omega} _ {i n} ^ {P}) + \boldsymbol {\varepsilon} ^ {P} =
$$

$$
\left(\boldsymbol {I} + \delta \boldsymbol {K} _ {G}\right) \left(\boldsymbol {\omega} _ {i n} ^ {n} + \delta \boldsymbol {\omega} _ {i e} ^ {P} + \delta \boldsymbol {\omega} _ {e n} ^ {P}\right) + \boldsymbol {\varepsilon} ^ {P}
$$

记

$$
\dot {\boldsymbol {\varphi}} ^ {T} = \boldsymbol {C} _ {P} ^ {T} \boldsymbol {\omega} _ {i P} ^ {P} - \boldsymbol {\omega} _ {i n} ^ {n}
$$

$$
\boldsymbol {\varphi} ^ {T} = \left[ \begin{array}{l} \varphi_ {x} \\ \varphi_ {y} \\ \varphi_ {z} \end{array} \right]
$$

则

$$
\mathbf {C} _ {T} ^ {P} = \left[ \begin{array}{c c c} 1 & \varphi_ {z} & - \varphi_ {y} \\ - \varphi_ {z} & 1 & \varphi_ {x} \\ \varphi_ {y} & - \varphi_ {x} & 1 \end{array} \right]
$$

所以

$$
\boldsymbol {C} _ {P} ^ {\mathrm{T}} = \left[ \begin{array}{c c c} 1 & - \varphi_ {z} & \varphi_ {y} \\ \varphi_ {z} & 1 & - \varphi_ {x} \\ - \varphi_ {y} & \varphi_ {z} & 1 \end{array} \right]
$$

记

$$
[ \pmb {\varphi} ] = \left[ \begin{array}{c c c} 0 & - \varphi_ {z} & \varphi_ {y} \\ \varphi_ {z} & 0 & - \varphi_ {x} \\ - \varphi_ {y} & \varphi_ {x} & 0 \end{array} \right]\tag{8.4.16}
$$

则

$$
\boldsymbol {C} _ {P} ^ {\mathrm{T}} = \boldsymbol {I} + [ \boldsymbol {\varphi} ]
$$

因此姿态误差角满足下述方程

式中

$$
\begin{array}{r l} \dot {\boldsymbol {\varphi}} ^ {\mathrm{T}} = & (\boldsymbol {I} + [ \boldsymbol {\varphi} ]) [ (\boldsymbol {I} + \delta \boldsymbol {K} _ {G}) (\boldsymbol {\omega} _ {i n} ^ {n} + \delta \boldsymbol {\omega} _ {i e} ^ {P} + \delta \boldsymbol {\omega} _ {e n} ^ {P}) + \boldsymbol {\varepsilon} ^ {P} ] - \boldsymbol {\omega} _ {i n} ^ {n} = \\ & ([ \boldsymbol {\varphi} ] + \delta \boldsymbol {K} _ {G}) \boldsymbol {\omega} _ {i n} ^ {n} + \delta \boldsymbol {\omega} _ {i e} ^ {n} + \delta \boldsymbol {\omega} _ {e n} ^ {n} + \boldsymbol {\varepsilon} ^ {P} \\ & \delta \boldsymbol {K} _ {G} = \text { diag } [ \delta K _ {G x} \quad \delta K _ {G y} \quad \delta K _ {G z} ] \end{array}\tag{8.4.17}
$$

推导中略去了关于误差的二阶及二阶以上的小量。

将比力方程式(8.4.1)向导航坐标系 n 投影得

$$
\dot {\boldsymbol {v}} ^ {n} = \boldsymbol {f} ^ {n} - (2 \boldsymbol {\omega} _ {i e} ^ {n} + \boldsymbol {\omega} _ {e n} ^ {n}) \times \boldsymbol {v} ^ {n} + \boldsymbol {g} ^ {n}\tag{8.4.18}
$$

设加速度计具有偏置误差 $\nabla^{p}$ 和刻度系数误差 $\delta K_{Ax},\delta K_{Ay},\delta K_{Az}$ ，实际平台坐标系P具有姿态误差角 $\varphi$ ，则加速度计的输出为

$$
\boldsymbol {f} _ {C} = \boldsymbol {C} _ {n} ^ {P} (\boldsymbol {I} + \delta \boldsymbol {K} _ {A}) \boldsymbol {f} ^ {n} + \boldsymbol {\nabla} ^ {P}
$$

用于计算有害加速度的实际角速度为

$$
\begin{array}{r l} \boldsymbol {\omega} _ {i e c} & = \boldsymbol {\omega} _ {i e} ^ {n} + \delta \boldsymbol {\omega} _ {i e} \\ \boldsymbol {\omega} _ {e n c} & = \boldsymbol {\omega} _ {e n} ^ {n} + \delta \boldsymbol {\omega} _ {e n} \end{array}
$$

由于比力输出和补偿有害加速度的计算都有误差,所以按比力方程确定的速度也有误差,设速度误差为 $\delta v$ , 则

$$
\begin{array}{r l} \dot {\boldsymbol {v}} _ {n} + \delta \dot {\boldsymbol {v}} & = (\boldsymbol {I} + [ \boldsymbol {\varphi} ]) (\boldsymbol {I} + \delta \boldsymbol {K} _ {A}) \boldsymbol {f} ^ {n} + \nabla^ {p} - \\ & (2 \boldsymbol {\omega} _ {i e} ^ {n} + \boldsymbol {\omega} _ {e n} ^ {n} + 2 \delta \boldsymbol {\omega} _ {i e} + \delta \boldsymbol {\omega} _ {e n}) \times (\boldsymbol {v} ^ {n} + \delta \boldsymbol {v}) + \boldsymbol {g} ^ {n} \end{array}\tag{8.4.18'}
$$

比较式(8.4.18)和式(8.4.18'),略去关于误差的二阶和二阶以上小量,则速度误差方程为

$$
\begin{array}{r l} \delta \dot {\boldsymbol {v}} & = (\delta \boldsymbol {K} _ {A} - [ \boldsymbol {\varphi} ]) \boldsymbol {f} ^ {n} - (2 \boldsymbol {\omega} _ {i e} ^ {n} + \boldsymbol {\omega} _ {e n} ^ {n}) \times \delta \boldsymbol {v} - (2 \delta \boldsymbol {\omega} _ {i e} + \delta \boldsymbol {\omega} _ {e n}) \times \boldsymbol {v} ^ {n} + \boldsymbol {\nabla} ^ {P} \\ & \delta \boldsymbol {K} _ {A} = \operatorname{diag} [ \delta K _ {A x} \quad \delta K _ {A y} \quad \delta K _ {A z} ] \end{array}\tag{8.4.19}
$$

式中

## 2. 指北方位系统的误差方程

取地理坐标系 g 为导航坐标系 n，根据式(8.4.3)和式(8.4.4)，当速度和纬度存在误差 $\delta v_{E}, \delta v_{N}, \delta L$ 时，则

$$
\begin{array}{r l} & \delta \boldsymbol {\omega} _ {i e} = \left[ \begin{array}{c} 0 \\ - \omega_ {i e} \sin L \delta L \\ \omega_ {i e} \cos L \delta L \end{array} \right] \\ & \delta \boldsymbol {\omega} _ {e n} = \left[ \begin{array}{c} - \frac {\delta v _ {\mathrm{N}}}{R + h} + \frac {v _ {\mathrm{N}}}{(R + h) ^ {2}} \delta h \\ \frac {\delta v _ {\mathrm{E}}}{R + h} - \frac {v _ {\mathrm{E}}}{(R + h) ^ {2}} \delta h \\ \frac {\delta v _ {\mathrm{E}}}{R + h} \tan L - \frac {v _ {\mathrm{E}} \tan L}{(R + h) ^ {2}} \delta h + \frac {v _ {\mathrm{E}}}{R + h} \sec^ {2} L \delta L \end{array} \right] \end{array}\tag{8.4.20}
$$

(8.4.21)

将式 $(8.4.20)$ 和式 $(8.4.21)$ 代入式 $(8.4.17)$ ，得姿态误差方程

$$
\begin{array}{r l} \dot {\varphi} _ {\mathrm{E}} & = - \frac {v _ {\mathrm{N}}}{R + h} \delta K _ {G \mathrm{E}} - \frac {\delta v _ {\mathrm{N}}}{R + h} + \frac {v _ {\mathrm{N}}}{(R + h) ^ {2}} \delta h + \\ & (\omega_ {i e} \sin L + \frac {v _ {\mathrm{E}}}{R + h} \tan L) \varphi_ {\mathrm{N}} - \left(\omega_ {i e} \cos L + \frac {v _ {\mathrm{E}}}{R + h}\right) \varphi_ {\mathrm{U}} + \varepsilon_ {\mathrm{E}} \\ \dot {\varphi} _ {\mathrm{N}} & = \left(\omega_ {i e} \cos L + \frac {v _ {\mathrm{E}}}{R + h}\right) \delta K _ {G \mathrm{N}} + \frac {\delta v _ {\mathrm{E}}}{R + h} - \frac {v _ {\mathrm{E}}}{(R + h) ^ {2}} \delta h - \omega_ {i e} \sin L \delta L - \\ & \left(\omega_ {i e} \sin L + \frac {v _ {\mathrm{E}}}{R + h} \tan L\right) \varphi_ {\mathrm{E}} - \frac {v _ {\mathrm{N}}}{R + h} \varphi_ {\mathrm{U}} + \varepsilon_ {\mathrm{N}} \\ \dot {\varphi} _ {\mathrm{U}} & = \left(\omega_ {i e} \sin L + \frac {v _ {\mathrm{E}}}{R + h} \tan L\right) \delta K _ {G U} + \frac {\delta v _ {\mathrm{E}}}{R + h} \tan L - \frac {v _ {\mathrm{E}} \tan L}{(R + h) ^ {2}} \delta h + \\ & \left(\omega_ {i e} \cos L + \frac {v _ {\mathrm{E}}}{R + h} \sec^ {2} L\right) \delta L + \left(\omega_ {i e} \cos L + \frac {v _ {\mathrm{E}}}{R + h}\right) \varphi_ {\mathrm{E}} + \frac {v _ {\mathrm{N}}}{R + h} \varphi_ {\mathrm{N}} + \varepsilon_ {\mathrm{U}}. \end{array} \tag {8.4}\tag{8.4.22a}
$$

(8.4.22b)

(8.4.22c)

将式 $(8.4.20)$ 和式 $(8.4.21)$ 代入式 $(8.4.19)$ ，得速度误差方程为

$$
\begin{array}{r l} \delta \dot {v} _ {\mathrm{E}} & = f _ {\mathrm{E}} \delta K _ {\mathrm{AE}} + f _ {\mathrm{N}} \varphi_ {\mathrm{U}} - f _ {\mathrm{U}} \varphi_ {\mathrm{N}} + \left(\frac {v _ {\mathrm{N}}}{R + h} \tan L - \frac {v _ {\mathrm{U}}}{R + h}\right) \delta v _ {\mathrm{E}} + \\ & \quad \left(2 \omega_ {i e} \sin L + \frac {v _ {\mathrm{E}}}{R + h} \tan L\right) \delta v _ {\mathrm{N}} - \left(2 \omega_ {i e} \cos L + \frac {v _ {\mathrm{E}}}{R + h}\right) \delta v _ {\mathrm{U}} + \\ & \quad \left(2 \omega_ {i e} \cos L v _ {\mathrm{N}} + \frac {v _ {\mathrm{E}} v _ {\mathrm{N}}}{R + h} \sec^ {2} L + 2 \omega_ {i e} \sin L v _ {\mathrm{U}}\right) \delta L + \\ & \quad \frac {v _ {\mathrm{E}} v _ {\mathrm{U}} - v _ {\mathrm{E}} v _ {\mathrm{N}} \tan L}{(R + h) ^ {2}} \delta h + \nabla_ {\mathrm{E}} \\ \delta \dot {v} _ {\mathrm{N}} & = f _ {\mathrm{N}} \delta K _ {\mathrm{AN}} + f _ {\mathrm{U}} \varphi_ {\mathrm{E}} - f _ {\mathrm{E}} \varphi_ {\mathrm{U}} - 2 \left(\omega_ {i e} \sin L + \frac {v _ {\mathrm{E}} \tan L}{R + h}\right) \delta v _ {\mathrm{E}} - \\ & \quad \frac {v _ {\mathrm{U}}}{R + h} \delta v _ {\mathrm{N}} - \frac {v _ {\mathrm{N}}}{R + h} \delta v _ {\mathrm{U}} - \left(2 \omega_ {i e} \cos L v _ {\mathrm{E}} + \frac {v _ {\mathrm{E}} ^ {2} \sec^ {2} L}{R + h}\right) \delta L + \\ & \quad \frac {v _ {\mathrm{E}} ^ {2} \tan L + v _ {\mathrm{N}} v _ {\mathrm{U}}}{(R + h) ^ {2}} \delta h + \nabla_ {\mathrm{N}} \\ \delta \dot {v} _ {\mathrm{U}} & = f _ {\mathrm{U}} \delta K _ {\mathrm{AU}} + f _ {\mathrm{E}} \varphi_ {\mathrm{N}} - f _ {\mathrm{N}} \varphi_ {\mathrm{E}} + 2 \left(\omega_ {i e} \cos L + \frac {v _ {\mathrm{E}}}{R + h}\right) \delta v _ {\mathrm{E}} + \end{array}\tag{8.4.23a}
$$

(8.4.23b)

$$
\frac {2 v _ {\mathrm{N}}}{R + h} \delta v _ {\mathrm{N}} - 2 \omega_ {i e} v _ {\mathrm{E}} \sin L \delta L - \frac {v _ {\mathrm{N}} ^ {2} + v _ {\mathrm{E}} ^ {2}}{(R + h) ^ {2}} \delta h + \nabla_ {\mathrm{U}}\tag{8.4.23c}
$$

在指北方位系统中,经度 $\lambda$ 、纬度 L 和高度 h 的求解方程为

$$
\dot {L} = \frac {v _ {\mathrm{N}}}{R + h}\tag{8.4.24a}
$$

$$
\dot {\lambda} = \frac {v _ {\mathrm{E}}}{(R + h) \cos L}\tag{8.4.24b}
$$

$$
\dot {h} = v _ {\mathrm{U}}\tag{8.4.24c}
$$

当东、北、天向速度以及纬度、高度分别存在误差 $\delta v_{E}, \delta v_{N}, \delta v_{U}, \delta L, \delta h$ 时，定位误差方程为

$$
\delta \dot {L} = \frac {\delta v _ {\mathrm{N}}}{R + h} - \frac {v _ {\mathrm{N}}}{(R + h) ^ {2}} \delta h\tag{8.4.25a}
$$

$$
\delta \dot {\lambda} = \frac {\delta v _ {\mathrm{E}}}{R + h} \sec L + \frac {v _ {\mathrm{E}}}{R + h} \sec L \tan L \delta L - \frac {v _ {\mathrm{E}}}{(R + h) ^ {2}} \sec L \delta h\tag{8.4.25b}
$$

$$
\delta \dot {h} = \delta v _ {\mathrm{U}}\tag{8.4.25c}
$$

此处说明一点,纯惯性高度通道是不稳定的,即诸如加速度计偏置等误差源引起的高度误差随时间的增加而加速增加,所以惯导系统单独工作时一般不作高度计算。但对组合导航系统来说,由于卡尔曼滤波器是一种变增益的反馈控制系统,它从量测量中分离出高度误差,并作校正,所以高度误差以估计和校正后的残留误差作为初值进行误差传播,滤波周期就是重新设置初值的时间间隔。这样高度误差始终被钳制在小数值范围内,这实质上给纯惯性高度通道引入了阻尼。

3. 游移方位系统在游移方位坐标系内列写的误差方程

记游移方位坐标系为 $\omega$ 。将式(8.4.17)写成 w 坐标系内的投影形式

$$
\dot {\boldsymbol {\varphi}} ^ {w} = \left(\left[ \boldsymbol {\varphi} \right] + \delta \mathbf {K} _ {G}\right) \left(\boldsymbol {\omega} _ {i e} ^ {w} + \boldsymbol {\omega} _ {e w} ^ {w}\right) + \delta \boldsymbol {\omega} _ {i e} ^ {w} + \delta \boldsymbol {\omega} _ {e w} ^ {w} + \boldsymbol {\varepsilon} ^ {P}\tag{8.4.26}
$$

由于 $w$ 坐标系相对地理坐标系 $g$ 转过了游移方位角 $\alpha$ ，所以

$$
\boldsymbol {\omega} _ {i e} ^ {w} = \left[ \begin{array}{c} \omega_ {i e} \cos L \sin \alpha \\ \omega_ {i e} \cos L \cos \alpha \\ \omega_ {i e} \sin L \end{array} \right]\tag{8.4.27}
$$

$$
\boldsymbol {\omega} _ {e v} ^ {w} = \left[ \begin{array}{c} - \frac {v _ {y}}{R + h} \\ \frac {v _ {x}}{R + h} \\ 0 \end{array} \right]\tag{8.4.28}
$$

当高度、纬度、游移方位角和速度分别存在误差 $\delta h, \delta L, \delta \alpha, \delta v_{x}, \delta v_{y}$ 时

$$
\delta \boldsymbol {\omega} _ {i e} ^ {w} = \left[ \begin{array}{c} - \delta \alpha \omega_ {i e} \cos L \cos \alpha - \delta L \omega_ {i e} \sin L \sin \alpha \\ - \delta \alpha \omega_ {i e} \cos L \sin \alpha - \delta L \omega_ {i e} \sin L \cos \alpha \\ \delta L \omega_ {i e} \cos L \end{array} \right]\tag{8.4.29}
$$

$$
\delta \boldsymbol {\omega} _ {e w} ^ {w} = \left[ \begin{array}{c} - \frac {\delta v _ {y}}{R + h} + \frac {v _ {y}}{(R + h) ^ {2}} \delta h \\ \frac {\delta v _ {x}}{R + h} - \frac {v _ {x}}{(R + h) ^ {2}} \delta h \\ 0 \end{array} \right]\tag{8.4.30}
$$

将式 $(8.4.27)\sim$ 式 $(8.4.30)$ 代入式 $(8.4.26)$ ，得姿态误差方程为

$$
\begin{array}{r l} \dot {\varphi} _ {x} & = \delta K _ {G x} \left(\omega_ {i e} \cos L \sin \alpha - \frac {v _ {y}}{R + h}\right) + \delta \alpha \omega_ {i e} \cos L \cos \alpha - \delta L \omega_ {i e} \sin L \sin \alpha - \\ & \frac {\delta v _ {y}}{R + h} + \frac {v _ {y}}{(R + h) ^ {2}} \delta h - \varphi_ {z} \left(\omega_ {i e} \cos L \cos \alpha + \frac {v _ {x}}{R + h}\right) + \varphi_ {y} \omega_ {i e} \sin L + \varepsilon_ {x} \\ & \dot {\varphi} _ {y} = \delta K _ {G y} \left(\omega_ {i e} \cos L \cos \alpha + \frac {v _ {x}}{R}\right) - \delta \alpha \omega_ {i e} \cos L \sin \alpha - \\ & \delta L \omega_ {i e} \sin L \cos \alpha + \frac {\delta v _ {x}}{R + h} - \frac {v _ {x}}{(R + h) ^ {2}} \delta h + \\ & \varphi_ {z} \left(\omega_ {i e} \cos L \sin \alpha - \frac {v _ {y}}{R + h}\right) - \varphi_ {x} \omega_ {i e} \sin L + \varepsilon_ {y} \\ & \dot {\varphi} _ {z} = \delta K _ {G z} \omega_ {i e} \sin L + \delta L \omega_ {i e} \cos L - \varphi_ {y} \left(\omega_ {i e} \cos L \sin \alpha - \frac {v _ {y}}{R + h}\right) + \\ & \varphi_ {x} \left(\omega_ {i e} \cos L \cos \alpha + \frac {v _ {x}}{R}\right) + \varepsilon_ {z} \end{array}\tag{8.4.31a}
$$

(8.4.31b)

(8.4.31c)

将式(8.4.27)～式(8.4.30)代入式(8.4.19)，经整理后得速度误差方程

$$
\begin{array}{r l} \delta \dot {v} _ {x} & = f _ {x} \delta K _ {A x} + f _ {y} \varphi_ {z} - f _ {z} \varphi_ {y} - \frac {v _ {z}}{R + h} \delta v _ {x} + 2 \omega_ {i e} \sin L \delta v _ {y} - \\ & \left(2 \omega_ {i e} \cos L \cos \alpha + \frac {v _ {x}}{R + h}\right) \delta v _ {z} + 2 \omega_ {i e} (v _ {z} \sin L \cos \alpha + v _ {y} \cos L) \delta L + \\ & 2 v _ {z} \omega_ {i e} \cos L \sin \alpha \delta \alpha + \frac {v _ {x} v _ {z}}{(R + h) ^ {2}} \delta h + \nabla_ {x} \\ \delta \dot {v} _ {y} & = f _ {y} \delta K _ {A y} - f _ {x} \varphi_ {z} + f _ {z} \varphi_ {x} - 2 \omega_ {i e} \sin L \delta v _ {x} - \frac {v _ {z}}{R + h} \delta v _ {y} + \\ & \left(2 \omega_ {i e} \cos L \sin \alpha - \frac {v _ {y}}{R + h}\right) \delta v _ {z} - 2 \omega_ {i e} (v _ {z} \sin L \sin \alpha + v _ {x} \cos L) \delta L \\ & + 2 v _ {z} \omega_ {i e} \cos L \cos \alpha \delta \alpha + \frac {v _ {y} v _ {z}}{(R + h) ^ {2}} \delta h + \nabla_ {y} \\ \delta \dot {v} _ {z} & = f _ {z} \delta K _ {A z} - f _ {y} \varphi_ {x} + f _ {x} \varphi_ {y} + 2 (\omega_ {i e} \cos L \cos \alpha + \frac {v _ {x}}{R + h}) \delta v _ {x} - \\ & 2 (\omega_ {i e} \cos L \sin \alpha - \frac {v _ {y}}{R + h}) \delta v _ {y} + 2 \omega_ {i e} \sin L (v _ {y} \sin \alpha - v _ {x} \cos \alpha) \delta L - \\ & 2 \omega_ {i e} \cos L (v _ {x} \sin \alpha + v _ {y} \cos \alpha) \delta \alpha - \frac {v _ {x} ^ {2} + v _ {y} ^ {2}}{(R + h) ^ {2}} \delta h + \nabla_ {z} \end{array}\tag{8.4.32a}
$$

(8.4.32b)

(8.4.32c)

在游移方位系统中,纬度 L, 经度 $\lambda$ , 高度 h 和游移方位角 $\alpha$ 满足

$$
\dot {L} = \frac {1}{R + h} (v _ {x} \sin \alpha + v _ {y} \cos \alpha)\tag{8.4.33a}
$$

$$
\dot {\lambda} = \frac {1}{(R + h) \cos L} (v _ {x} \cos \alpha - v _ {y} \sin \alpha)\tag{8.4.33b}
$$

$$
\dot {h} = v _ {z}\tag{8.4.33c}
$$

$$
\dot {\alpha} = \frac {1}{R + h} (v _ {y} \sin \alpha - v _ {x} \cos \alpha) \tan L\tag{8.4.33d}
$$

所以 $L, \lambda, h$ 和 $\alpha$ 的误差方程为

$$
\delta \dot {L} = \frac {1}{R + h} \left(\sin \alpha \delta v _ {x} + \cos \alpha \delta v _ {y} + v _ {\mathrm{E}} \delta \alpha - \frac {v _ {\mathrm{N}}}{R + h} \delta h\right)\tag{8.4.34a}
$$

$$
\delta \dot {\lambda} = \frac {1}{(R + h) \cos L} \left(\cos \alpha \delta v _ {x} - \sin \alpha \delta v _ {y} - v _ {\mathrm{N}} \delta \alpha - \frac {v _ {\mathrm{E}}}{R + h} \delta h\right) + \frac {v _ {\mathrm{E}}}{R + h} \sec L \tan L \delta L\tag{8.4.34b}
$$

$$
\delta \dot {h} = \delta v _ {z}\tag{8.4.34c}
$$

$$
\dot {\delta \alpha} = \frac {\tan L}{R + h} \left(\sin \alpha \delta v _ {y} - \cos \alpha \delta v _ {x} + v _ {\mathrm{N}} \delta \alpha\right) + \frac {v _ {\mathrm{E}}}{R + h} \sec^ {2} L \delta L + \frac {v _ {\mathrm{E}} \tan L}{(R + h) ^ {2}} \delta h\tag{8.4.34d}
$$

式中

$$
\begin{array}{l} v _ {\mathrm{E}} = v _ {x} \cos \alpha - v _ {y} \sin \alpha \\ v _ {\mathrm{N}} = v _ {x} \sin \alpha + v _ {y} \cos \alpha \end{array}\tag{8.4.35a}
$$

(8.4.35b)

无任何误差时,游移方位系统的航向角 $\Psi$ 为

$$
\Psi = \alpha + \Psi_ {T b}
$$

式中： $\alpha$ 为游移方位角； $\Psi_{Tb}$ 为平台航向角，它是运载体相对平台的航向角。

系统实际给出的航向为

而

$$
\begin{array}{r l} & \Psi^ {c} = \alpha + \delta \alpha + \Psi_ {P b} \\ & \Psi_ {P b} = \Psi_ {P T} + \Psi_ {T b} \end{array}
$$

其中 P 为实际平台坐标系, T 为理想平台坐标系, 显然

$$
\Psi_ {P T} = - \Psi_ {T P} = - \varphi_ {z}
$$

所以

$$
\delta \Psi = \Psi^ {C} - \Psi = \delta \alpha - \varphi_ {z}\tag{8.4.36}
$$

式(8.4.36)说明,游移方位系统中,航向误差由游移方位误差 $\delta\alpha$ 和平台方位误差 $\varphi_{z}$ 两部分构成。若在组合导航系统中获得了 $\delta\alpha$ 和 $\varphi_{z}$ 的估计值 $\delta\hat{\alpha}$ 和 $\hat{\varphi}_{z}$ ,则航向误差的估计值和估计误差分别为

$$
\begin{array}{l} \delta \hat {\Psi} = \delta \hat {\alpha} - \hat {\varphi} _ {z} \\ \delta \widetilde {\Psi} = \delta \Psi - \delta \hat {\Psi} = \delta \tilde {\alpha} - \tilde {\varphi} _ {z} \end{array}
$$

式中 $\delta\tilde{\alpha}=\delta\alpha-\delta\hat{\alpha},\quad\tilde{\varphi}_{z}=\varphi_{z}-\hat{\varphi}_{z}$ 分别为 $\delta\alpha$ 和 $\varphi_{z}$ 的估计误差。所以 $\delta\Psi$ 的估计均方误差为

$$
P (\delta \widetilde {\Psi}) = E [ (\delta \tilde {\alpha} - \tilde {\varphi} _ {z}) ^ {2} ] = P (\delta \tilde {\alpha}) + P (\tilde {\varphi} _ {z}) - 2 E [ \delta \tilde {\alpha} \cdot \tilde {\varphi} _ {z} ]\tag{8.4.37}
$$

4. 游移方位系统在地理坐标系 g 内列写的误差方程

（1）姿态误差方程。由于陀螺漂移 $\varepsilon$ 是小量，在实际平台坐标系 $P$ 内的投影与理想平台坐标系 $\omega$ 内的投影只差二阶小量，所以式(8.4.26)可写为

$$
\begin{array}{r l} \dot {\boldsymbol {\varphi}} ^ {w} = & ([ \boldsymbol {\varphi} ] + \delta \mathbf {K} _ {G}) (\boldsymbol {\omega} _ {i e} ^ {w} + \boldsymbol {\omega} _ {e w} ^ {w}) + \delta \boldsymbol {\omega} _ {i e} ^ {w} + \delta \boldsymbol {\omega} _ {e w} ^ {w} + \boldsymbol {\varepsilon} ^ {w} = \\ & \boldsymbol {\varphi} ^ {w} \times (\boldsymbol {\omega} _ {i e} ^ {w} + \boldsymbol {\omega} _ {e w} ^ {w}) + \delta \mathbf {K} _ {G} (\boldsymbol {\omega} _ {i e} ^ {w} + \boldsymbol {\omega} _ {e w} ^ {w}) + \delta \boldsymbol {\omega} _ {i e} ^ {w} + \delta \boldsymbol {\omega} _ {e w} ^ {w} + \boldsymbol {\varepsilon} ^ {w} \end{array}\tag{8.4.38}
$$

根据哥氏定理，有

$$
\dot {\boldsymbol {\varphi}} ^ {w} \stackrel {\text { def }} {=} \frac {\mathrm{d} \boldsymbol {\varphi} ^ {w}}{\mathrm{d} t} \Big | _ {w} = \frac {\mathrm{d} \boldsymbol {\varphi} ^ {w}}{\mathrm{d} t} \Big | _ {g} + \boldsymbol {\omega} _ {w g} ^ {w} \times \boldsymbol {\varphi} ^ {w}\tag{8.4.39}
$$

式中，竖线旁的下标表示求变化率的参照坐标系。

由于

$$
\begin{array}{r l} \delta \boldsymbol {\omega} _ {i e} ^ {w} + \delta \boldsymbol {\omega} _ {e w} ^ {w} & = (\boldsymbol {\omega} _ {i e} ^ {c} + \boldsymbol {\omega} _ {e c} ^ {c}) - (\boldsymbol {\omega} _ {i e} ^ {w} + \boldsymbol {\omega} _ {e w} ^ {w}) = \\ & \boldsymbol {C} _ {w} ^ {c} (\boldsymbol {\omega} _ {i e} ^ {w} + \boldsymbol {\omega} _ {e w} ^ {w} + \boldsymbol {\omega} _ {w c} ^ {w}) - (\boldsymbol {\omega} _ {i e} ^ {w} + \boldsymbol {\omega} _ {e w} ^ {w}) = \end{array}
$$

$$
\begin{array}{l} (I - [ \boldsymbol {\theta} ]) (\boldsymbol {\omega} _ {i e} ^ {w} + \boldsymbol {\omega} _ {e w} ^ {w} + \frac {\mathrm{d} \boldsymbol {\theta} ^ {w}}{\mathrm{d} t} \Big | _ {w}) - (\boldsymbol {\omega} _ {i e} ^ {w} + \boldsymbol {\omega} _ {e w} ^ {w}) \approx \\ - \boldsymbol {\theta} ^ {w} \times (\boldsymbol {\omega} _ {i e} ^ {w} + \boldsymbol {\omega} _ {e w} ^ {w}) + \frac {\mathrm{d} \boldsymbol {\theta} ^ {w}}{\mathrm{d} t} \Big | _ {g} + \boldsymbol {\omega} _ {w g} ^ {w} \times \boldsymbol {\theta} ^ {w} = \\ - \boldsymbol {\theta} ^ {w} \times (\boldsymbol {\omega} _ {i e} ^ {w} + \boldsymbol {\omega} _ {e g} ^ {w}) + \frac {\mathrm{d} \boldsymbol {\theta} ^ {w}}{\mathrm{d} t} \Big | _ {g} \end{array}\tag{8.4.40}
$$

式中 $\theta^w$ 为游移方位系统的位置误差向量

$$
\boldsymbol {\theta} ^ {w} = \left[ \begin{array}{c} \delta \lambda \cos L \sin \alpha - \delta L \cos \alpha \\ \delta \lambda \cos L \cos \alpha + \delta L \sin \alpha \\ \delta \lambda \sin L + \delta \alpha \end{array} \right]\tag{8.4.41}
$$

$[\theta]$ 为 $\theta^{w}$ 各分量构造的 $3 \times 3$ 反对称矩阵。

将式 $(8.4.39)$ 和式 $(8.4.40)$ 代入式 $(8.4.38)$ 得

$$
\begin{array}{r l} \frac {\mathrm{d} \boldsymbol {\varphi} ^ {w}}{\mathrm{d} t} \Bigg | _ {g} & = \boldsymbol {\varphi} ^ {w} \times (\boldsymbol {\omega} _ {i e} ^ {w} + \boldsymbol {\omega} _ {e w} ^ {w} + \boldsymbol {\omega} _ {w g} ^ {w}) - \boldsymbol {\theta} ^ {w} \times (\boldsymbol {\omega} _ {i e} ^ {w} + \boldsymbol {\omega} _ {e w} ^ {w} + \boldsymbol {\omega} _ {w g} ^ {w}) + \\ & \delta \mathbf {K} _ {G} (\boldsymbol {\omega} _ {i e} ^ {w} + \boldsymbol {\omega} _ {e w} ^ {w}) + \frac {\mathrm{d} \boldsymbol {\theta} ^ {w}}{\mathrm{d} t} \Bigg | _ {g} + \boldsymbol {\varepsilon} ^ {w} = \\ & \boldsymbol {\varphi} ^ {w} \times (\boldsymbol {\omega} _ {i e} ^ {w} + \boldsymbol {\omega} _ {e g} ^ {w}) - \boldsymbol {\theta} ^ {w} \times (\boldsymbol {\omega} _ {i e} ^ {w} + \boldsymbol {\omega} _ {e g} ^ {w}) + \frac {\mathrm{d} \boldsymbol {\theta} ^ {w}}{\mathrm{d} t} \Bigg | _ {g} + \delta \mathbf {K} _ {G} \boldsymbol {\omega} _ {i w} ^ {w} + \boldsymbol {\varepsilon} ^ {w} \end{array}\tag{8.4.42}
$$

显然 $\omega_{iw}^{w}$ 就是游移方位系统的指令角速度, 记该指令为

$$
\boldsymbol {\omega} _ {c m d} ^ {w} = \left[ \begin{array}{l} \boldsymbol {\omega} _ {c m d x} ^ {w} \\ \boldsymbol {\omega} _ {c m d y} ^ {w} \\ \boldsymbol {\omega} _ {c m d z} ^ {w} \end{array} \right]
$$

式(8.4.42)两边左乘变换矩阵 $C_{w}^{g}$ 得

式中

$$
\begin{array}{r l} \dot {\boldsymbol {\varphi}} ^ {g} = & \boldsymbol {\varphi} ^ {g} \times \boldsymbol {\omega} _ {i g} ^ {g} - \boldsymbol {\theta} ^ {g} \times (\boldsymbol {\omega} _ {i e} ^ {g} + \boldsymbol {\omega} _ {e g} ^ {g}) + \dot {\boldsymbol {\theta}} ^ {g} + C _ {w} ^ {g} \delta K _ {G} \boldsymbol {\omega} _ {c m d} ^ {w} + C _ {u} ^ {g} \boldsymbol {\varepsilon} ^ {w} \\ & \dot {\boldsymbol {\theta}} ^ {g} = \frac {\mathrm{d} \boldsymbol {\theta} ^ {g}}{\mathrm{d} t} \Big | _ {g} \end{array}\tag{8.4.43}
$$

由于 w 坐标系相对 g 坐标系仅绕铅垂轴 $z_{g}$ 转过了游移方位角 $\alpha$ ，所以

$$
\boldsymbol {C} _ {g} ^ {w} = \left[ \begin{array}{c c c} \cos \alpha & \sin \alpha & 0 \\ - \sin \alpha & \cos \alpha & 0 \\ 0 & 0 & 1 \end{array} \right]
$$

并且 g 坐标系相当于 w 坐标系当 $\alpha = 0$ 时的情况，因此由式(8.4.41)得指北方位系统的位置误差向量

$$
\boldsymbol {\theta} _ {g} ^ {g} = \left[ \begin{array}{l} - \delta L \\ \delta \lambda \cos L \\ \delta \lambda \sin L \end{array} \right]\tag{8.4.44}
$$

由式(8.4.41)

$$
\boldsymbol {\theta} ^ {g} = \boldsymbol {C} _ {w} ^ {g} \boldsymbol {\theta} ^ {w} = \left[ \begin{array}{c c c} \cos \alpha & - \sin \alpha & 0 \\ \sin \alpha & \cos \alpha & 0 \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c} \delta \lambda \cos L \sin \alpha - \delta L \cos \alpha \\ \delta \lambda \cos L \cos \alpha + \delta L \sin \alpha \\ \delta \lambda \sin L + \delta \alpha \end{array} \right] = \left[ \begin{array}{c} - \delta L \\ \delta \lambda \cos L \\ \delta \lambda \sin L + \delta \alpha \end{array} \right]\tag{8.4.45}
$$

比较式(8.4.44)和式(8.4.45)得

$$
\Delta \pmb {\theta} ^ {g} = \pmb {\theta} ^ {g} - \pmb {\theta} _ {g} ^ {g} = \left[ \begin{array}{l l l} 0 & 0 & \delta \alpha \end{array} \right] ^ {\mathrm{T}}\tag{8.4.46}
$$

$$
\boldsymbol {\theta} ^ {g} = \boldsymbol {\theta} _ {g} ^ {g} + \Delta \boldsymbol {\theta} ^ {g}\tag{8.4.47}
$$

$$
\dot {\boldsymbol {\theta}} ^ {g} = \dot {\boldsymbol {\theta}} _ {g} ^ {g} + \Delta \dot {\boldsymbol {\theta}} ^ {g}\tag{8.4.48}
$$

将式 $(8.4.47)$ 和式 $(8.4.48)$ 代入式 $(8.4.43)$ 得

$$
\begin{array}{r l} \dot {\boldsymbol {\varphi}} ^ {g} - \Delta \dot {\boldsymbol {\theta}} ^ {g} & = (\boldsymbol {\varphi} ^ {g} - \Delta \boldsymbol {\theta} ^ {g}) \times (\boldsymbol {\omega} _ {i e} ^ {g} + \boldsymbol {\omega} _ {e g} ^ {g}) - \boldsymbol {\theta} _ {g} ^ {g} \times (\boldsymbol {\omega} _ {i e} ^ {g} + \boldsymbol {\omega} _ {e g} ^ {g}) + \\ & \quad \dot {\boldsymbol {\theta}} _ {g} ^ {g} + \boldsymbol {C} _ {w} ^ {g} [ \delta \boldsymbol {K} _ {G} ] \boldsymbol {\omega} _ {c m d} ^ {w} + \boldsymbol {C} _ {w} ^ {g} \boldsymbol {\varepsilon} ^ {w} \end{array}\tag{8.4.49}
$$

令

$$
\boldsymbol {\Phi} ^ {g} = \boldsymbol {\varphi} ^ {g} - \Delta \boldsymbol {\theta} ^ {g}\tag{8.4.50}
$$

则式 $(8.4.49)$ 可写成

$$
\dot {\boldsymbol {\Phi}} ^ {g} = \left(\boldsymbol {\Phi} ^ {g} - \boldsymbol {\theta} _ {g} ^ {g}\right) \times \left(\boldsymbol {\omega} _ {i e} ^ {g} + \boldsymbol {\omega} _ {e g} ^ {g}\right) + \dot {\boldsymbol {\theta}} _ {g} ^ {g} + \boldsymbol {C} _ {w} ^ {g} \left[ \delta \boldsymbol {K} _ {G} \right] \boldsymbol {\omega} _ {c m d} ^ {w} + \boldsymbol {C} _ {w} ^ {g} \boldsymbol {\varepsilon} ^ {w}\tag{8.4.51}
$$

式(8.4.51)中的 $\dot{\theta}_{g}^{g}$ 确定如下。由式(8.4.44)、式(8.4.24)和式(8.4.25)得

$$
\dot {\boldsymbol {\theta}} _ {g} ^ {g} = \left[ \begin{array}{c} - \delta \dot {L} \\ \delta \dot {\lambda} \cos L - \dot {L} \sin L \delta \lambda \\ \delta \dot {\lambda} \sin L + \dot {L} \cos L \delta \lambda \end{array} \right] = \left[ \begin{array}{c} - \frac {\delta v _ {\mathrm{N}}}{R} \\ \frac {\delta v _ {\mathrm{E}}}{R} + \delta L \frac {v _ {\mathrm{E}} \tan L}{R} - \delta \lambda \frac {v _ {\mathrm{N}} \sin L}{R} \\ \frac {\delta v _ {\mathrm{E}}}{R} \tan L + \delta L \frac {v _ {\mathrm{E}}}{R} \tan^ {2} L + \delta \lambda \frac {v _ {\mathrm{N}} \cos L}{R} \end{array} \right]\tag{8.4.52}
$$

为简化推导,式(8.4.52)中忽略了高度的影响。将式(8.4.52)、式(8.4.3)和式(8.4.4)代入式(8.4.51)得

$$
\left[ \begin{array}{c} \dot {\Phi} _ {\mathrm{E}} \\ \dot {\Phi} _ {\mathrm{N}} \\ \dot {\Phi} _ {\mathrm{U}} \end{array} \right] = \left(\left[ \begin{array}{c c c} 0 & - \Phi_ {\mathrm{U}} & \Phi_ {\mathrm{N}} \\ \Phi_ {\mathrm{U}} & 0 & - \Phi_ {\mathrm{E}} \\ - \Phi_ {\mathrm{N}} & \Phi_ {\mathrm{E}} & 0 \end{array} \right] - \left[ \begin{array}{c c c} 0 & - \delta \lambda \sin L & \delta \lambda \cos L \\ \delta \lambda \sin L & 0 & \delta L \\ - \delta \lambda \cos L & - \delta L & 0 \end{array} \right]\right).
$$

$$
\left[ \begin{array}{c} - \frac {v _ {\mathrm{N}}}{R} \\ \frac {v _ {\mathrm{E}}}{R} + \omega_ {i e} \cos L \\ \frac {v _ {\mathrm{E}}}{R} \tan L + \omega_ {i e} \sin L \end{array} \right] + \left[ \begin{array}{c} - \frac {\delta v _ {\mathrm{N}}}{R} \\ \frac {\delta v _ {\mathrm{E}}}{R} + \delta L \frac {v _ {\mathrm{E}} \tan L}{R} - \delta \lambda \frac {v _ {\mathrm{N}} \sin L}{R} \\ \frac {\delta v _ {\mathrm{E}}}{R} \tan L + \delta L \frac {v _ {\mathrm{E}}}{R} \tan^ {2} L + \delta \lambda \frac {v _ {\mathrm{N}} \cos L}{R} \end{array} \right] +
$$

$$
\left[ \begin{array}{c c c} \cos \alpha & - \sin \alpha & 0 \\ \sin \alpha & \cos \alpha & 0 \\ 0 & 0 & 1 \end{array} \right] \quad \left[ \begin{array}{c c c} \delta K _ {G x} & 0 & 0 \\ 0 & \delta K _ {G y} & 0 \\ 0 & 0 & \delta K _ {G z} \end{array} \right] \quad \left[ \begin{array}{l} \omega_ {c m d x} ^ {w} \\ \omega_ {c m d y} ^ {w} \\ \omega_ {c m d z} ^ {w} \end{array} \right] +
$$

$$
\left[ \begin{array}{c c c} \cos \alpha & - \sin \alpha & 0 \\ \sin \alpha & \cos \alpha & 0 \\ 0 & 0 & 1 \end{array} \right] \quad \left[ \begin{array}{c} \varepsilon_ {x} ^ {w} \\ \varepsilon_ {y} ^ {w} \\ \varepsilon_ {z} ^ {w} \end{array} \right]
$$

展开上式，得

$$
\begin{array}{r l} \dot {\Phi} _ {\mathrm{E}} & = - \Phi_ {\mathrm{U}} \left(\frac {v _ {\mathrm{E}}}{R} + \omega_ {i e} \cos L\right) + \Phi_ {\mathrm{N}} \left(\frac {v _ {\mathrm{E}}}{R} \tan L + \omega_ {i e} \sin L\right) - \\ & \frac {\delta v _ {\mathrm{N}}}{R} + \delta K _ {G x} \omega_ {c m d x} ^ {w} \cos \alpha - \delta K _ {G y} \omega_ {c m d y} ^ {w} \sin \alpha + \varepsilon_ {x} ^ {w} \cos \alpha - \varepsilon_ {y} ^ {w} \sin \alpha \end{array}\tag{8.4.53a}
$$

$$
\begin{array}{r l} \dot {\Phi} _ {\mathrm{N}} & = - \Phi_ {\mathrm{E}} \left(\frac {v _ {\mathrm{E}}}{R} \tan L + \omega_ {i e} \sin L\right) - \Phi_ {\mathrm{U}} \frac {v _ {\mathrm{N}}}{R} + \frac {\delta v _ {\mathrm{E}}}{R} - \\ & \delta L \omega_ {i e} \sin L + \delta K _ {G x} \omega_ {c m d x} ^ {w} \sin \alpha + \delta K _ {G y} \omega_ {c m d y} ^ {w} \cos \alpha + \\ & \varepsilon_ {x} ^ {w} \sin \alpha + \varepsilon_ {y} ^ {w} \cos \alpha \\ \dot {\Phi} _ {\mathrm{U}} & = \Phi_ {\mathrm{E}} \left(\frac {v _ {\mathrm{E}}}{R} + \omega_ {i e} \cos L\right) + \Phi_ {\mathrm{N}} \frac {v _ {\mathrm{N}}}{R} + \frac {\delta v _ {\mathrm{E}}}{R} \tan L + \\ & \delta L \left(\frac {v _ {\mathrm{E}}}{R} \sec^ {2} L + \omega_ {i e} \cos L\right) + \delta K _ {G z} \omega_ {c m d z} ^ {w} + \varepsilon_ {z} ^ {w} \end{array}\tag{8.4.53b}
$$

(8.4.53c)

（2）速度误差方程。设采用 g 坐标系的游移方位系统的速度误差为 $\delta v^{g}$ ，它是三个计算的速度分量与三个真实的速度分量的差值，所以也称分量差速度误差，这是惯导系统中常用的速度误差，它并不是向量，所以不能通过坐标变换获得。下面从速度计算方程和速度方程的差来导出 $\delta v^{g}$ 的方程。

记速度计算值为 $v_{c}^{g_{c}}$ ，速度真实值为 $v^{g}$ ，则根据比力方程式(8.4.1)有

$$
\begin{array}{r l} \dot {\boldsymbol {v}} _ {c} ^ {g _ {c}} & = \boldsymbol {f} _ {c} ^ {g _ {c}} - (2 \boldsymbol {\omega} _ {i e} ^ {g _ {c}} + \boldsymbol {\omega} _ {e g _ {c}} ^ {g _ {c}}) \times \boldsymbol {v} _ {c} ^ {g _ {c}} + \boldsymbol {g} _ {c} ^ {g _ {c}} \\ \dot {\boldsymbol {v}} ^ {g} & = \boldsymbol {f} ^ {g} - (2 \boldsymbol {\omega} _ {i e} ^ {g} + \boldsymbol {\omega} _ {e g} ^ {g}) \times \boldsymbol {v} ^ {g} + \boldsymbol {g} ^ {g} \end{array}\tag{8.4.54}
$$

(8.4.55)

式 $(8.4.54)$ 减去式 $(8.4.55)$ 并注意到

$$
\begin{array}{r l} f _ {c c} ^ {g} & = C _ {c c} ^ {g} (f ^ {P} + \nabla^ {P}) \approx \\ & C _ {g c} ^ {g} C _ {w} ^ {g} C _ {c c} ^ {w} C _ {w} ^ {P} C _ {g g} ^ {w} f ^ {g} + C _ {w} ^ {g} \nabla^ {w} = \\ & (I - [ \theta_ {g} ^ {g} ]) C _ {w} ^ {g} (I + [ \theta^ {w} ]) (I - [ \varphi^ {w} ]) C _ {g} ^ {w} f ^ {g} + C _ {w} ^ {g} \nabla^ {w} \approx \\ & \{I - ([ \varphi^ {g} ] - [ \theta^ {g} ] + [ \theta_ {g} ^ {g} ]) \} f ^ {g} + C _ {w} ^ {g} \nabla^ {w} = \\ & \{I - ([ \varphi^ {g} ] - [ \Delta \theta^ {g} ]) \} f ^ {g} + C _ {w} ^ {g} \nabla^ {w} = \\ & (I - [ \Phi^ {g} ]) f ^ {g} + C _ {w} ^ {g} \nabla^ {g} \end{array}\tag{8.4.56}
$$

式中 P 为游移方位系统的实际平台坐标系，方括号表示由相应误差向量构造的 $3 \times 3$ 反对称矩阵。则采用 g 坐标系的游移方位系统的速度误差方程为

$$
\begin{array}{r l} \delta \dot {\boldsymbol {v}} ^ {g} & = - (2 \boldsymbol {\omega} _ {i e} ^ {g} + \boldsymbol {\omega} _ {e g} ^ {g}) \times \delta \boldsymbol {v} ^ {g} - (2 \delta \boldsymbol {\omega} _ {i e} ^ {g} + \delta \boldsymbol {\omega} _ {e g} ^ {g}) \times \boldsymbol {v} ^ {g} - \\ & [ \boldsymbol {\Phi} ^ {g} ] \boldsymbol {f} ^ {g} + \boldsymbol {C} _ {w} ^ {g} \nabla^ {w} + \delta \boldsymbol {g} ^ {g} \end{array}\tag{8.4.57}
$$

式中

$$
\delta \omega_ {i e} ^ {g} = \omega_ {i e} ^ {g _ {c}} - \omega_ {i e} ^ {g}\tag{8.4.58a}
$$

$$
\delta \boldsymbol {\omega} _ {e g} ^ {g} = \boldsymbol {\omega} _ {e g _ {c}} ^ {g _ {c}} - \boldsymbol {\omega} _ {e g} ^ {g}\tag{8.4.58b}
$$

$$
\delta \mathbf {g} ^ {\mathrm{g}} = \mathbf {g} _ {c} ^ {\mathrm{g} _ {c}} - \mathbf {g} ^ {\mathrm{g}}\tag{8.4.58c}
$$

式中 $\delta\omega_{ie}^{g}$ 和 $\delta\omega_{eg}^{g}$ 与指北方位系统速度误差方程中的相应项相同。展开式(8.4.57)，整理得

$$
\begin{array}{r l} \delta \dot {v} _ {\mathrm{E}} & = \left(2 \omega_ {i e} \sin L + \frac {v _ {\mathrm{E}}}{R} \tan L\right) \delta v _ {\mathrm{N}} + \frac {1}{R} (v _ {\mathrm{N}} \tan L - v _ {\mathrm{U}}) \delta v _ {\mathrm{E}} - \\ & \left(2 \omega_ {i e} \cos L + \frac {v _ {\mathrm{E}}}{R}\right) \delta v _ {\mathrm{U}} + \left[ 2 \omega_ {i e} (v _ {\mathrm{N}} \cos L + v _ {\mathrm{U}} \sin L) + \right. \\ & \left. \frac {1}{R} v _ {\mathrm{N}} v _ {\mathrm{E}} \sec^ {2} L \right] \delta L + \frac {1}{R ^ {2}} (v _ {\mathrm{E}} v _ {\mathrm{U}} - v _ {\mathrm{N}} v _ {\mathrm{E}} \tan L) \delta h + \\ & f _ {\mathrm{N}} \Phi_ {\mathrm{U}} - f _ {\mathrm{U}} \Phi_ {\mathrm{N}} + \nabla_ {x} \cos \alpha - \nabla_ {y} \sin \alpha \\ \delta \dot {v} _ {\mathrm{N}} & = - \frac {v _ {\mathrm{U}}}{R} \delta v _ {\mathrm{N}} - 2 (\omega_ {i e} \sin L + \frac {v _ {\mathrm{E}}}{R} \tan L) \delta v _ {\mathrm{E}} - \frac {v _ {\mathrm{N}}}{R} \delta v _ {\mathrm{U}} - \end{array}\tag{8.4.59a}
$$

$$
\begin{array}{r l} & {\left(2 \omega_ {i e} \cos L + \frac {v _ {\mathrm{E}}}{R} \sec^ {2} L\right) v _ {\mathrm{E}} \delta L + \frac {1}{R ^ {2}} (v _ {\mathrm{N}} v _ {\mathrm{U}} + v _ {\mathrm{E}} ^ {2} \tan L) \delta h +} \\ & {f _ {\mathrm{U}} \Phi_ {\mathrm{E}} - f _ {\mathrm{E}} \Phi_ {\mathrm{U}} + \nabla_ {x} \sin \alpha + \nabla_ {y} \cos \alpha} \\ & {\delta \dot {v} _ {\mathrm{U}} = \frac {2 v _ {\mathrm{N}}}{R} \delta v _ {\mathrm{N}} + 2 (\omega_ {i e} \cos L + \frac {v _ {\mathrm{E}}}{R}) \delta v _ {\mathrm{E}} - 2 v _ {\mathrm{E}} \omega_ {i e} \sin L \delta L -} \\ & {\frac {v _ {\mathrm{E}} ^ {2} + v _ {\mathrm{N}} ^ {2}}{R ^ {2}} \delta h + f _ {\mathrm{E}} \Phi_ {\mathrm{N}} - f _ {\mathrm{N}} \Phi_ {\mathrm{E}} + \nabla_ {z}} \end{array}\tag{8.4.59b}
$$

(8.4.59c)

根据式(8.4.46)和式(8.4.50)，式(8.4.59a)、式(8.4.59b)、式(8.4.59c)中

$$
\begin{array}{l} \Phi_ {\mathrm{E}} = \varphi_ {\mathrm{E}} \\ \Phi_ {\mathrm{N}} = \varphi_ {\mathrm{N}} \\ \Phi_ {\mathrm{U}} = \varphi_ {\mathrm{U}} - \delta \alpha \end{array}
$$

（3）位置误差方程和游移方位角误差方程。位置误差 $\delta \lambda, \delta L, \delta h$ 和游移方位角误差 $\delta \alpha$ 与误差方程坐标系无关，但误差方程式(8.4.34)与误差方程坐标系有关。只要找出式中速度误差 $\delta v_{x}, \delta v_{y}$ 与采用 $g$ 坐标系的速度误差 $\delta v_{\mathrm{E}}, \delta v_{\mathrm{N}}$ 的关系，就能将式(8.4.34)变换成与 $g$ 坐标系有关。由于

$$
\boldsymbol {C} _ {w} ^ {g} \delta \boldsymbol {v} ^ {w} = \boldsymbol {C} _ {w} ^ {g} (\boldsymbol {v} _ {c} ^ {c} - \boldsymbol {v} ^ {w})\tag{8.4.60}
$$

式中

$$
\begin{array}{r l} \boldsymbol {C} _ {w} ^ {g} (\boldsymbol {v} _ {c} ^ {c} - \boldsymbol {v} ^ {w}) & = \boldsymbol {C} _ {g _ {c}} ^ {g} \boldsymbol {C} _ {w} ^ {g _ {c}} \boldsymbol {C} _ {w} ^ {c} \boldsymbol {v} _ {c} ^ {w} - \boldsymbol {v} ^ {g} = \\ & (\boldsymbol {I} + [ \boldsymbol {\theta} _ {g} ^ {g} ]) \boldsymbol {C} _ {w} ^ {g _ {c}} (\boldsymbol {I} - [ \boldsymbol {\theta} ^ {w} ]) \boldsymbol {v} _ {c} ^ {w} - \boldsymbol {v} ^ {g} \approx \\ & (\boldsymbol {v} _ {c} ^ {g _ {c}} - \boldsymbol {v} ^ {g}) + ([ \boldsymbol {\theta} _ {g} ^ {g} ] - [ \boldsymbol {\theta} ^ {g} ]) \boldsymbol {v} ^ {g} = \\ & \delta \boldsymbol {v} ^ {g} - [ \delta \boldsymbol {\theta} ^ {g} ] \boldsymbol {v} ^ {g} \end{array}
$$

将上述关系代入式 $(8.4.60)$ ，并写成分量形式

$$
\left[ \begin{array}{c} \delta v _ {x} \cos \alpha - \delta v _ {y} \sin \alpha \\ \delta v _ {x} \sin \alpha + \delta v _ {y} \cos \alpha \\ \delta v _ {z} \end{array} \right] = \left[ \begin{array}{c} \delta v _ {\mathrm{E}} + v _ {\mathrm{N}} \delta \alpha \\ \delta v _ {\mathrm{N}} - v _ {\mathrm{E}} \delta \alpha \\ \delta v _ {\mathrm{U}} \end{array} \right]\tag{8.4.61}
$$

上式说明了 $\delta v_{x}, \delta v_{y}$ 与 $\delta v_{E}, \delta v_{N}$ 的关系，即 $\delta v_{x}, \delta v_{y}$ 变换到东向和北向，还要增加因 $\delta \alpha$ 而导致的误差。

将式(8.4.61)代入式(8.4.34)，可得采用 g 坐标系的游移方位惯导位置误差方程和游移方位角误差方程

$$
\delta \dot {L} = \frac {\delta v _ {\mathrm{N}}}{R} - \frac {v _ {\mathrm{N}}}{R ^ {2}} \delta h\tag{8.4.62a}
$$

$$
\delta \dot {\lambda} = \frac {1}{R \cos L} \left(\delta v _ {\mathrm{E}} + v _ {\mathrm{E}} \tan L \delta L - \frac {v _ {\mathrm{E}}}{R} \delta h\right)\tag{8.4.62b}
$$

$$
\delta \dot {h} = \delta v _ {\mathrm{U}}\tag{8.4.62c}
$$

$$
\dot {\delta \alpha} = \frac {1}{R} \tan L \left(- \delta v _ {\mathrm{E}} + \frac {v _ {\mathrm{E}}}{\cos L \sin L} \delta L + \frac {v _ {\mathrm{E}}}{R} \delta h\right)\tag{8.4.62d}
$$

（4）采用 g 坐标系的游移方位惯导基本误差方程的特点。式(8.4.53)、式(8.4.59)和式(8.4.62)构成了采用 g 坐标系的游移方位惯导系统的基本误差方程。从上述诸式可看出：

1）组合导航系统卡尔曼滤波器若以沿 g 坐标系的速度误差分量和平台误差角分量( $\delta v_{E}$ ， $\delta v_{N}$ ， $\delta v_{U}$ ， $\Phi_{E}$ ， $\Phi_{N}$ ， $\Phi_{U}$ )作为状态，则惯导基本误差状态方程即为上述三组方程。

2）虽然与 w 坐标系误差状态一样，采用 g 坐标系的惯导基本误差状态为 10 个，但游移方位角误差 $\delta\alpha$ 并不影响其他误差状态。所以，如果组合导航系统不必估计 $\delta\alpha$ ，则滤波器中惯导基本误差状态可减少为 9 个。

3）姿态误差方程组中， $\Phi_{E}$ 和 $\Phi_{N}$ 即为平台沿东向和北向的水平倾角 $\varphi_{E}$ 和 $\varphi_{N}$ ，但 $\Phi_{U}$ 并不是平台的方位误差角 $\varphi_{U}$ ，而是 $\varphi_{U}$ 与游移方位误差角 $\delta\alpha$ 的差值。 $\Phi_{U}$ 也就是惯导的航向误差角 $\delta\Psi$ ，从航向信号是惯导输出量这个意义上讲，对 $\Phi_{U}$ 的估计比对 $\varphi_{U}$ 的估计更为有意义。

4）采用 g 坐标系的游移方位惯导误差方程与指北方位系统误差方程极为相似，主要差别在于：

a. 方程中惯性器件误差的影响是以等效误差的形式出现的。例如平台误差角 $\Phi_{\mathrm{E}}$ 方程中，陀螺的影响为 $\varepsilon_{x}\cos \alpha - \varepsilon_{y}\sin \alpha$ ，这是等效东向陀螺漂移，而指北方位系统中， $\varepsilon_{\mathrm{E}}$ 是实际的东向陀螺漂移。游移方位角 $\alpha$ 在惯导工作过程中随着载体东向速度而变化，虽然变化缓慢，但却使得游移方位惯导在与其他导航系统组合时，增加了惯性器件的可观测性。

b. g 坐标系的游移方位惯导平台误差方程采用广义平台误差角作为误差状态，仅在方位上与平台误差角差 $\delta\alpha$ 。而指北方位系统中，方位误差角就是平台真实的方位误差角。

5）惯导系统基本误差方程除了用于设计组合导航滤波器外，还是设计和分析惯导系统和包含有惯导系统的组合导航系统性能的基本数学模型。对游移方位惯导系统，若选用上述 $g$ 坐标系误差方程组，则可直接和指北方位惯导系统相比较，便于性能分析。有的文献[91]将采用 $g$ 坐标的各类惯导的基本误差方程统称为惯导统一误差方程，原因就在于此。

以上内容以游移方位惯导系统为例,分别介绍了用 g 坐标系和 w 坐标系列写的基本误差方程的特点。在设计组合导航系统卡尔曼滤波器时,如果惯导是游移方位系统,则可根据其他导航系统的类型、组合导航系统的要求以及这两种基本误差方程的特点等因素,选用其中一种误差方程作为滤波器的基本状态方程。对于其他类型的惯导系统,例如导弹和空间飞行器上常用的解析式惯导系统,也可选用惯性坐标系或轨道水平坐标系甚至地理坐标系等不同坐标系作为其误差方程的坐标系。不同坐标系的误差方程都有其固有的特点,因篇幅有限,此处不再一一讨论了。

## 5. 捷联惯导系统的误差方程

前已指出,如果导航坐标系取地理坐标系,则捷联惯导系统与指北方位系统等效。但陀螺的刻度系数误差对系统的影响方式不同。在指北方位系统中,此误差通过对平台的指令引入系统,即为式(8.4.22)诸式中的第一项;而在捷联惯导系统中,此误差引起对载体角速度的测量误差,经姿态更新计算引入系统。

设陀螺的刻度系数误差为 $\delta K_{gi}(i = x,y,z)$ ，陀螺的随机常值漂移、相关漂移和不相关漂移分别为 $\varepsilon_{bi},\varepsilon_{ri},w_{gi}(i = x,y,z)$ ，载体的角速度为 $\omega^b = [\omega_x\omega_y\omega_z]^{\mathrm{T}}$ ，则陀螺在地理坐标系内的等效漂移为

$$
\left[ \begin{array}{l} \varepsilon_ {\mathrm{E}} \\ \varepsilon_ {\mathrm{N}} \\ \varepsilon_ {\mathrm{U}} \end{array} \right] = \boldsymbol {C} _ {b} ^ {n} \left[ \begin{array}{l} \varepsilon_ {b x} + \varepsilon_ {r x} + \delta K _ {g x} \omega_ {x} + w _ {g x} \\ \varepsilon_ {b y} + \varepsilon_ {r y} + \delta K _ {g y} \omega_ {y} + w _ {g y} \\ \varepsilon_ {b z} + \varepsilon_ {r z} + \delta K _ {g z} \omega_ {z} + w _ {g z} \end{array} \right]
$$

即

$$
\begin{array}{r l} \varepsilon_ {\mathrm{E}} = & C _ {b} ^ {n} (1, 1) (\varepsilon_ {b x} + \varepsilon_ {r x} + \delta K _ {g x} \omega_ {x} + w _ {g x}) + \\ & C _ {b} ^ {n} (1, 2) (\varepsilon_ {b y} + \varepsilon_ {r y} + \delta K _ {g y} \omega_ {y} + w _ {g y}) + \\ & C _ {b} ^ {n} (1, 3) (\varepsilon_ {b z} + \varepsilon_ {r z} + \delta K _ {g z} \omega_ {z} + w _ {g z}) \end{array}\tag{8.4.63a}
$$

$$
\begin{array}{r l} \varepsilon_ {\mathrm{N}} & = C _ {b} ^ {n} (2, 1) (\varepsilon_ {b x} + \varepsilon_ {r x} + \delta K _ {g x} \omega_ {x} + w _ {g x}) + \\ & \quad C _ {b} ^ {n} (2, 2) (\varepsilon_ {b y} + \varepsilon_ {r y} + \delta K _ {g y} \omega_ {y} + w _ {g y}) + \\ & \quad C _ {b} ^ {n} (2, 3) (\varepsilon_ {b z} + \varepsilon_ {r z} + \delta K _ {g z} \omega_ {z} + w _ {g z}) \\ \varepsilon_ {\mathrm{U}} & = C _ {b} ^ {n} (3, 1) (\varepsilon_ {b x} + \varepsilon_ {r x} + \delta K _ {g x} \omega_ {x} + w _ {g x}) + \\ & \quad C _ {b} ^ {n} (3, 2) (\varepsilon_ {b y} + \varepsilon_ {r y} + \delta K _ {g y} \omega_ {y} + w _ {g y}) + \\ & \quad C _ {b} ^ {n} (3, 3) (\varepsilon_ {b z} + \varepsilon_ {r z} + \delta K _ {g z} \omega_ {z} + w _ {g z}) \end{array}\tag{8.4.63b}
$$

(8.4.63c)

同理可得加速度计的等效偏置量为

$$
\begin{array}{r l} & \nabla_ {\mathrm{E}} = C _ {b} ^ {n} (1, 1) (\nabla_ {b x} + \delta K _ {a x} f _ {x} + w _ {a x}) + C _ {b} ^ {n} (1, 2) (\nabla_ {b y} + \delta K _ {a y} f _ {y} + \\ & \quad w _ {a y}) + C _ {b} ^ {n} (1, 3) (\nabla_ {b z} + \delta K _ {a z} f _ {z} + w _ {a z}) \\ & \nabla_ {\mathrm{N}} = C _ {b} ^ {n} (2, 1) (\nabla_ {b x} + \delta K _ {a x} f _ {x} + w _ {a x}) + C _ {b} ^ {n} (2, 2) (\nabla_ {b y} + \delta K _ {a y} f _ {y} + \\ & \quad w _ {a y}) + C _ {b} ^ {n} (2, 3) (\nabla_ {b z} + \delta K _ {a z} f _ {z} + w _ {a z}) \\ & \nabla_ {\mathrm{U}} = C _ {b} ^ {n} (3, 1) (\nabla_ {b x} + \delta K _ {a x} f _ {x} + w _ {a x}) + C _ {b} ^ {n} (3, 2) (\nabla_ {b y} + \delta K _ {a y} f _ {y} + \\ & \quad w _ {a y}) + C _ {b} ^ {n} (3, 3) (\nabla_ {b z} + \delta K _ {a z} f _ {z} + w _ {a z}) \end{array}\tag{8.4.64a}
$$

(8.4.64b)

(8.4.64c)

式中， $f_{i}(i=x,y,z)$ 为载体的理想比力在机体坐标系内的投影， $\nabla_{bi},\delta K_{ai},w_{ai}(i=x,y,z)$ 分别为加速度计的偏置量、刻度系数误差及白噪声误差。

只须去掉式(8.4.22)和式(8.4.23)中关于刻度系数误差的项,而陀螺漂移和加速度计的零偏用式(8.4.63)和式(8.4.64)所确定的等效量代替,并考虑到捷联惯导中,陀螺漂移引起的姿态误差变化方向与陀螺漂移方向相反这一特点,即可获得捷联惯导系统的姿态误差方程和速度误差方程。由于篇幅关系,此处不再一一列出。

## 8.4.3 惯导系统的误差源模型

## 1. 陀螺误差模型

陀螺是运载体角运动的测量器件,对惯导系统的姿态误差产生直接的影响。陀螺的误差主要体现为漂移和刻度系数误差,这两类误差都是随机误差。

刻度系数误差一般用随机常数来描述

$$
\delta \dot {K} _ {G i} = 0 \quad i = x, y, z\tag{8.4.65}
$$

陀螺的确定性漂移经标定后能够得到较好的补偿,但剩余的随机漂移是无法通过标定确定的。随机漂移是十分复杂的随机过程,大致可概括为三种分量:

（1）逐次启动漂移。它取决于启动时刻的环境条件和电气参数的随机性等因素，一旦启动完成，这种漂移便保持在某一固定值上，但这一固定值是一个随机变量，所以这种分量可用随机常数描述之

$$
\dot {\varepsilon} _ {b i} = 0 \quad i = x, y, z\tag{8.4.66}
$$

（2）慢变漂移。陀螺在工作过程中，环境条件、电气参数都在作随机改变，所以陀螺漂移在随机常数分量的基础上以较慢的速率变化。由于变化较缓慢，变化过程中前后时刻上的漂移值有一定的关联性，即后一时刻的漂移值程度不等地取决于前一时刻的漂移值，两者的时间点靠得越近，这种依赖关系就越明显。这种漂移分量可用一阶马尔柯夫过程描述之

$$
\dot {\varepsilon} _ {n} = - \frac {1}{\tau_ {G}} \varepsilon_ {n} + w _ {n} \quad i = x, y, z\tag{8.4.67}
$$

(3) 快变漂移。表现为在上述两种分量基础上的杂乱无章的高频跳变,不管两时间点靠得多近,该两时间点上的漂移值依赖关系十分微弱或几乎不存在。这种漂移分量可抽象化为白噪声过程 $w_{gi}$ ，即

$$
E \left[ w _ {g i} (t) w _ {g i} (\tau) \right] = q _ {g i} \delta (t - \tau) \quad i = x, y, z\tag{8.4.68}
$$

式中 $\delta(t-\tau)$ 为狄拉克 $\delta$ 函数。

综上所述，陀螺漂移可模型化为

$$
\varepsilon_ {i} (t) = \varepsilon_ {b i} (t) + \varepsilon_ {r i} (t) + w _ {g i} (t)\tag{8.4.69}
$$

陀螺漂移是反映在陀螺对运载体角速率测量上的误差。液浮单自由度陀螺直接感测角速率，而挠性陀螺的和激光陀螺测量的是角增量，即角速率在某段时间内的积分值。根据3.1节关于有色噪声的讨论知，随机游走是白噪声的积分，所以反映在角增量上的随机游走与反映在角速率上的白噪声是等效的。在激光陀螺的性能指标中所给出的游走系数（如 $0.01^{\circ}/\sqrt{h}$ ）实质上就是陀螺漂移白噪声分量 $w_{gi}(t)$ 的方差强度的平方根。

激光陀螺的随机游走系数远比挠性陀螺的游走系数大,相关时间也短得多。激光陀螺的游走系数与随机常值漂移的大小几乎是同一数量级的,相关时间约为300 s;而挠性陀螺的游走系数比随机常值漂移小1～2个数量级,相关时间约为1 h。

在卡尔曼滤波器的设计中,白噪声分量并不列为状态,而随机常值分量和相关分量都列入状态。有时为了降低滤波器的维数,常对相关分量作简化处理,简化处理可分别按三种不同情况考虑:

（1）若相关时间很短，则相关漂移可近似视为白噪声。一阶马尔柯夫过程的相关函数和功率谱分别为

$$
R (\tau) = R (0) \mathrm{e} ^ {- \alpha | \tau |}\tag{8.4.70}
$$

$$
S (\omega) = \frac {2 R (0) \alpha}{\omega^ {2} + \alpha^ {2}}\tag{8.4.71}
$$

式中， $\alpha=\frac{1}{T}$ （T为相关时间）， $R(0)$ 为相关漂移的均方值。

由式(8.4.71)可看出,当T很小,即 $\alpha$ 很大时,

$$
S (\omega) \approx 2 R (0) T
$$

即功率谱在很宽的频带内保持为常值。而惯导平台的修正回路自振频率为休拉频率 $\omega_{s}$ （休拉周期 $T_{s} = 84.4\mathrm{min}$ ），这实质上是一个低通滤波器，截止频率 $\omega_{c}$ 比 $\omega_{s}$ 稍大。如果 $2\pi \alpha \gg \omega_{s}$ ，即 $T \ll \frac{2\pi}{\omega_{s}}$ ，则 $S(\omega)$ 在 $[0, \omega_{s}]$ 内近似为常值，相关漂移在修正回路中的作用与白噪声的作用是等效的（白噪声的功率谱在整个频率轴上保持常值）。所以当相关时间很短时，例如低于数百秒时，一阶马尔柯夫过程可近似视为白噪声过程，白噪声过程的方差强度为

$$
q = S (\omega) = 2 R (0) T
$$

（2）当相关时间很长时，则相关漂移可近似看做随机常数或随机游走。由式(8.4.70)可看出，当 $T$ 特别大，即 $\alpha$ 特别小时

$$
R (\tau) \approx R (0)\tag{8.4.72}
$$

即当惯导的工作时间远小于相关时间时,相关漂移可近似看做随机常值而并入陀螺的逐次启动误差内

$$
\varepsilon_ {b} ^ {\prime} = \varepsilon_ {b} + \varepsilon_ {r}\tag{8.4.73}
$$

等效随机常值漂移 $\varepsilon_{b}^{\prime}$ 的方差为

$$
R _ {b} ^ {\prime} = R _ {b} + R _ {r} (0)\tag{8.4.74}
$$

若将一阶马尔柯夫过程近似为随机游走，即

$$
\dot {\varepsilon} _ {w} = \xi_ {w}\tag{8.4.75}
$$

则由于随机游走的均方值随时间而增长，所以，不可能根据近似前后两种模型的漂移均方值相等这个条件来求得 $\xi_w$ 的方差强度 $q_{w}$ ，而只能根据经验或通过误差分析来选取。文献[91]提出采用以下方程计算：

$$
q _ {w} = R _ {r} (0) \alpha
$$

显然，当 $t = \frac{1}{\alpha}$ 时

$$
E [ \varepsilon_ {w} ^ {2} (t) ] = q _ {w} t = R _ {r} (0)
$$

这意味着 $q_{w}$ 的选择要使 $\varepsilon_{w}$ 在 $t = \frac{1}{\alpha}$ 时的均方值与 $\varepsilon_{r}$ 的均方值相等。

将 $\varepsilon_{r}$ 简化为 $\varepsilon_{w}$ 的方法有两个优点: 一是将随机游走的估计用来反馈校正陀螺漂移的计算简单; 二是可以和偏置合并为一个模型方程, 即

$$
\varepsilon = \varepsilon_ {b} + \varepsilon_ {w}
$$

合并后的漂移模型为

因

$$
\begin{array}{r} \dot {\varepsilon} = \xi_ {w} \\ E [ \varepsilon_ {w} ^ {2} (0) ] = 0 \end{array}
$$

所以滤波器对漂移 $\varepsilon$ 的初始估计均方误差 $P_{\varepsilon}(0)$ 可选取

$$
P _ {\varepsilon} (0) = E [ \varepsilon^ {2} (0) ] = E [ \varepsilon_ {b} ^ {2} ] = R _ {b}
$$

（3）当相关时间很长，且比惯导工作时间长时，随机常值漂移和相关漂移可用等效相关漂移描述之

$$
\varepsilon = \varepsilon_ {b} + \varepsilon_ {r}
$$

式中 $\varepsilon$ 可近似用另一个一阶马尔柯夫过程 $\varepsilon_{e}$ 等效

$$
\dot {\varepsilon} _ {e} = - \alpha_ {e} \varepsilon_ {e} + w _ {e}\tag{8.4.76}
$$

等效的条件是： $\varepsilon_{e}$ 的均方值与 $\varepsilon_{b}$ 和 $\varepsilon_{r}$ 两个分量的均方值之和相等； $\varepsilon_{e}$ 和 $\varepsilon_{r}$ 的相关函数在 $\tau=0$ 处的斜率相同，如图 8.4.3 所示。即有

$$
R _ {e} (0) = R _ {b} + R _ {r} (0)\tag{8.4.77}
$$

$$
\alpha_ {e} = \frac {R _ {r} (0)}{R _ {r} (0) + R _ {b}} \alpha_ {r}\tag{8.4.78}
$$

![](images/428ccebfea396e0f1340249fd2978d207ecdb2f6ecc1b908bbf56656c0faf11b.jpg)  
图 8.4.3 相关漂移和随机常值漂移的等效漂移

## 2. 加速度计误差模型

与陀螺漂移误差模型的分析类似,加速度计误差模型可分为三种分量,但在组合导航设计中,一般只考虑随机常值误差,即偏置误差 $\nabla_{i}(i=x,y,z)$ ,而忽略相关误差。这是由于这种分量相对较小,同时也为了使滤波器的维数尽量低些。所以加速度计误差模型一般考虑为

$$
\nabla_ {i} = \nabla_ {b i} + w _ {a i} \quad i = x, y, z\tag{8.4.79}
$$

式中

$$
\dot {\nabla} _ {b i} = 0 \quad i = x, y, z\tag{8.4.80}
$$

$w_{ai}(i=x,y,z)$ 为白噪声过程。

## 8.5 卡尔曼滤波理论在惯导系统初始对准中的应用

## 8.5.1 概述

惯导系统初始对准的目的是在惯导系统进入导航工作状态之前建立起导航坐标系。对平台式惯导系统来说，就是控制平台旋转使之与要求的导航坐标系相重合；对捷联式惯导系统来说，就是计算出机体坐标系 $b$ 到导航坐标系 $n$ 的姿态矩阵 $\mathbf{C}_b^n$ 。初始对准过程中，一般还须要计算出陀螺漂移的偏置量，即测漂。

根据对准过程中使用参考基准信息的不同,初始对准可分为自对准、传递对准和空中对准方法。自对准使用天然基准信息重力加速度和地球自转角速度;传递对准以主惯导为基准,使子惯导建立的导航坐标系重合于主惯导已建立起来的导航坐标系;空中对准从惯导相对其他导航系统提供的导航参数(如速度等)的偏差中估计出惯导系统的失准角并校正之。

有两种完成初始对准的实施方案:采用回路反馈法实现的经典对准方案和采用卡尔曼滤波或递推最小二乘法实现的广义数字信号处理方案。由于后者考虑了惯导工作环境中的随机干扰因素,如阵风等引起的飞机的随机晃动,飞行过程中空气动力的随机改变引起的飞机结构的挠曲变形等,所以无论是改善对准精度还是缩短对准时间,后者都优于前者。关于对前者的讨论读者可参阅文献 $[87\sim90]$ ,本节将对后者作详细讨论。

## 8.5.2 平台式惯导系统自对准卡尔曼滤波器设计

## 1. 静基座条件下自对准卡尔曼滤波器设计

（1）系统模型和量测方程。静基座条件下(例如实验室条件)的初始对准是比较简单的情况,此处为便于说明问题,以指北方位系统为例来说明滤波器的设计。

由于在静基座上,速度恒为零,对准地点的经、纬度准确已知,所以根据式(8.4.22),姿态误差方程为

$$
\left. \begin{array}{l} \dot {\varphi} _ {\mathrm{E}} = \omega_ {i e} \sin L \varphi_ {\mathrm{N}} - \omega_ {i e} \cos L \varphi_ {\mathrm{U}} + \varepsilon_ {\mathrm{E}} + w _ {\mathrm{E}} \\ \dot {\varphi} _ {\mathrm{N}} = - \omega_ {i e} \sin L \varphi_ {\mathrm{E}} + \varepsilon_ {\mathrm{N}} + w _ {\mathrm{N}} \\ \dot {\varphi} _ {\mathrm{U}} = \omega_ {i e} \cos L \varphi_ {\mathrm{E}} + \varepsilon_ {\mathrm{U}} + w _ {\mathrm{U}} \end{array} \right\}\tag{8.5.1}
$$

从式(8.5.1)可看出,平台误差角的变化与陀螺漂移有关。上节曾指出,陀螺漂移包含三种分量:随机常值漂移;相关漂移(一阶马氏过程)和不相关漂移(白噪声)。对转子结构的陀螺来说,相关漂移的相关时间大于1 h,所以对于时间20 min左右的初始对准来讲,这种相关漂移可近似视为随机常数，且与随机常值漂移相比，这种漂移小1～2个数量级。所以初始对准中陀螺漂移模型可简化为偏置

$$
\left. \begin{array}{l} \dot {\varepsilon} _ {E} = 0 \\ \dot {\varepsilon} _ {N} = 0 \\ \dot {\varepsilon} _ {U} = 0 \end{array} \right\}\tag{8.5.2}
$$

和白噪声分量 $w_{E}$ ， $w_{N}$ ， $w_{U}$ 。在静基座上， $f_{E}=f_{N}=0$ ， $f_{U}=g$ ，略去速度误差间的耦合，则由式(8.4.23)得速度误差方程

$$
\left\{ \begin{array}{l} \delta \dot {v} _ {\mathrm{E}} = - g \varphi_ {\mathrm{N}} + \nabla_ {\mathrm{E}} + V _ {\mathrm{E}} \\ \delta \dot {v} _ {\mathrm{N}} = g \varphi_ {\mathrm{E}} + \nabla_ {\mathrm{N}} + V _ {\mathrm{N}} \end{array} \right.
$$

这也是加速度计的输出,并以此作为量测量

$$
\left. \begin{array}{l} Z _ {\mathrm{E}} = - g \varphi_ {\mathrm{N}} + \nabla_ {\mathrm{E}} + V _ {\mathrm{E}} \\ Z _ {\mathrm{N}} = g \varphi_ {\mathrm{E}} + \nabla_ {\mathrm{N}} + V _ {\mathrm{N}} \end{array} \right\}\tag{8.5.3}
$$

式中， $V_{E}$ 和 $V_{N}$ 为东向和北向加速度计误差输出中的白噪声分量； $\nabla_{E}$ 和 $\nabla_{N}$ 为偏置分量，满足

$$
\left. \begin{array}{l} \dot {\nabla} _ {E} = 0 \\ \dot {\nabla} _ {N} = 0 \end{array} \right\}\tag{8.5.4}
$$

式(8.5.1)，式(8.5.2)和式(8.5.4)构成了静基座条件下的系统模型，式(8.5.3)为量测方程。

（2）系统可检测性的判别和处理。由惯导系统原理知，惯导系统采用自对准作初始对准时，能达到的水平极限精度为

$$
\left. \begin{array}{l} \varphi_ {\mathrm{E} m} = - \frac {\nabla_ {\mathrm{N}}}{g} \\ \varphi_ {\mathrm{N} m} = \frac {\nabla_ {\mathrm{E}}}{g} \end{array} \right\}\tag{8.5.5}
$$

方位极限精度为

$$
\varphi_ {\mathrm{Um}} = \frac {\varepsilon_ {\mathrm{E}}}{\omega_ {i e} \cos L}\tag{8.5.6}
$$

上述误差是无法消除的,即使采用卡尔曼滤波方法也是如此。下面按滤波稳定性的观点分析之。先判别系统是否可检测。选取状态向量为

$$
\boldsymbol {X} = \left[ \begin{array}{l l l l l l l l} \varphi_ {\mathrm{N}} & \varphi_ {\mathrm{E}} & \varphi_ {\mathrm{U}} & \varepsilon_ {\mathrm{N}} & \varepsilon_ {\mathrm{U}} & \varepsilon_ {\mathrm{E}} & \nabla_ {\mathrm{E}} & \nabla_ {\mathrm{N}} \end{array} \right] ^ {\mathrm{T}}
$$

则状态方程和量测方程为

$$
\begin{array}{l} \dot {\boldsymbol {X}} (t) = \boldsymbol {F} \boldsymbol {X} (t) + \boldsymbol {w} (t) \\ \boldsymbol {Z} (t) = \boldsymbol {H} \boldsymbol {X} (t) + \boldsymbol {V} (t) \end{array}
$$

或

$$
\left[ \begin{array}{c} \dot {\varphi} _ {N} \\ \dot {\varphi} _ {E} \\ \dot {\varphi} _ {U} \\ \dot {\varepsilon} _ {N} \\ \dot {\varepsilon} _ {U} \\ \vdots \\ \dot {\varepsilon} _ {E} \\ \vdots \\ \nabla_ {E} \\ \vdots \\ \nabla_ {N} \end{array} \right] = \left[ \begin{array}{c c c c c c c c} 0 & - \Omega_ {U} & 0 & 1 & 0 & 0 & 0 & 0 \\ \Omega_ {U} & 0 & - \Omega_ {N} & 0 & 0 & 1 & 0 & 0 \\ 0 & \Omega_ {N} & 0 & 0 & 1 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\ \hline 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \end{array} \right] \left[ \begin{array}{c} \varphi_ {N} \\ \varphi_ {E} \\ \varphi_ {U} \\ \varepsilon_ {N} \\ \varepsilon_ {U} \\ \vdots \\ \varepsilon_ {E} \\ \nabla_ {E} \\ \nabla_ {N} \end{array} \right] +
$$

$$
\left[ \begin{array}{l} w _ {\mathrm{N}} \\ w _ {\mathrm{E}} \\ w _ {\mathrm{U}} \\ 0 \\ 0 \\ 0 \\ 0 \\ 0 \end{array} \right] \stackrel {{\text { def }}} {{=}} \left[ \begin{array}{c c c c} & 0 & 0 & 0 \\ & 1 & 0 & 0 \\ \mathbf {F} _ {1} & 0 & 0 & 0 \\ & 0 & 0 & 0 \\ & 0 & 0 & 0 \\ \hdashline \mathbf {0} & & \mathbf {0} \end{array} \right] \mathbf {X} (t) + \left[ \begin{array}{l} w _ {1} \\ \mathbf {w} _ {1} \\ \mathbf {0} \end{array} \right]\tag{8.5.7}
$$

式中

$$
\begin{array}{r l} & \Omega_ {\mathrm{N}} = \omega_ {i e} \cos L, \Omega_ {\mathrm{U}} = \omega_ {i e} \sin L \\ & \left[ \begin{array}{c} Z _ {\mathrm{E}} \\ Z _ {\mathrm{N}} \end{array} \right] = \left[ \begin{array}{c c c c c c c c} - g & 0 & 0 & 0 & 0 & 0 & 1 & 0 \\ 0 & g & 0 & 0 & 0 & 0 & 0 & 1 \end{array} \right] X (t) + \left[ \begin{array}{c} V _ {\mathrm{E}} \\ V _ {\mathrm{N}} \end{array} \right] \stackrel {\text {def}} {=} \\ & [ H _ {1} H _ {2} ] X (t) + V (t) \end{array}\tag{8.5.8}
$$

令

$$
\boldsymbol {X} (t) = \boldsymbol {T} _ {o} \boldsymbol {X} ^ {o} (t)\tag{8.5.9}
$$

取

$$
\boldsymbol {T} _ {o} = \left[ \begin{array}{l l} \boldsymbol {I} _ {5} & \boldsymbol {L} \\ \boldsymbol {0} & \boldsymbol {I} _ {3} \end{array} \right]\tag{8.5.10}
$$

式中 $I_{5}$ 和 $I_{3}$ 分别为 5 阶和 3 阶单位阵

$$
\boldsymbol {L} = \left[ \begin{array}{c c c} 0 & \frac {1}{g} & \mathbf {0} \\ 0 & 0 & - \frac {1}{g} \\ - \frac {1}{\Omega_ {\mathrm{N}}} & - \frac {\Omega_ {\mathrm{U}}}{\Omega_ {\mathrm{N}} g} & 0 \\ 0 & 0 & - \frac {\Omega_ {\mathrm{U}}}{g} \\ 0 & 0 & \frac {\Omega_ {\mathrm{N}}}{g} \end{array} \right]\tag{8.5.11}
$$

由于

$$
\left[ \begin{array}{c c} I _ {5} & L \\ 0 & I _ {3} \end{array} \right] \quad \left[ \begin{array}{c c} I _ {5} & - L \\ 0 & I _ {3} \end{array} \right] = \left[ \begin{array}{c c} I _ {5} & - I _ {5} L + L I _ {3} \\ 0 & I _ {3} \end{array} \right] = I _ {8}
$$

所以

$$
\boldsymbol {T} _ {o} ^ {- 1} = \left[ \begin{array}{c c} \boldsymbol {I} _ {5} & - \boldsymbol {L} \\ \boldsymbol {0} & \boldsymbol {I} _ {3} \end{array} \right]\tag{8.5.12}
$$

$$
\boldsymbol {T} _ {o} ^ {- 1} \boldsymbol {w} = \left[ \begin{array}{c c} \boldsymbol {I} _ {5} & - \boldsymbol {L} \\ \boldsymbol {0} & \boldsymbol {I} _ {3} \end{array} \right] \quad \left[ \begin{array}{c} w _ {\mathrm{N}} \\ w _ {\mathrm{E}} \\ w _ {\mathrm{U}} \\ 0 \\ \vdots \\ 0 \end{array} \right] = \left[ \begin{array}{c} w _ {\mathrm{N}} \\ w _ {\mathrm{E}} \\ w _ {\mathrm{U}} \\ 0 \\ \vdots \\ 0 \end{array} \right] = \left[ \begin{array}{c} w _ {1} \\ \boldsymbol {0} \end{array} \right]
$$

因此，原系统方程和量测方程经相似变换后成

$$
\begin{array}{r l} & \dot {\mathbf {X}} ^ {o} (t) = \mathbf {F} _ {o} \mathbf {X} ^ {o} (t) + \mathbf {w} (t) \\ & \mathbf {Z} (t) = \mathbf {H} _ {o} \mathbf {X} ^ {o} (t) + \mathbf {V} (t) \end{array}
$$

式中

$$
\begin{array}{r l} & {\pmb {F} _ {o} = \pmb {T} _ {o} ^ {- 1} \pmb {F} \pmb {T} _ {o} = \left[ \begin{array}{l l} {\pmb {F} _ {1}} & {\mathbf {0}} \\ {\mathbf {0}} & {\mathbf {0}} \end{array} \right]} \\ & {\pmb {H} _ {o} = \pmb {H} \pmb {T} _ {o} = \left[ \begin{array}{l l} {\pmb {H} _ {1}} & {\mathbf {0}} \end{array} \right]} \end{array}\tag{8.5.13}
$$

(8.5.14)

由式(8.5.9)、式(8.5.11)和式(8.5.12)得

$$
\boldsymbol {X} ^ {o} (t) = \left[ \begin{array}{c} \boldsymbol {X} _ {1} ^ {o} (t) \\ \boldsymbol {X} _ {2} ^ {o} (t) \end{array} \right] = \boldsymbol {T} _ {o} ^ {- 1} \boldsymbol {X} (t)
$$

式中

$$
\boldsymbol {X} _ {1} ^ {o} (t) = \left[ \begin{array}{l} \varphi_ {\mathrm{N}} ^ {o} \\ \varphi_ {\mathrm{E}} ^ {o} \\ \varphi_ {\mathrm{U}} ^ {o} \\ \varepsilon_ {\mathrm{N}} ^ {o} \\ \varepsilon_ {\mathrm{U}} ^ {o} \end{array} \right] = \left[ \begin{array}{c} \varphi_ {\mathrm{N}} - \frac {\nabla_ {\mathrm{E}}}{g} \\ \varphi_ {\mathrm{E}} + \frac {\nabla_ {\mathrm{N}}}{g} \\ \varphi_ {\mathrm{U}} + \frac {\varepsilon_ {\mathrm{E}}}{\Omega_ {\mathrm{N}}} + \frac {\Omega_ {\mathrm{U}}}{\Omega_ {\mathrm{N}} g} \nabla_ {\mathrm{E}} \\ \varepsilon_ {\mathrm{N}} + \frac {\Omega_ {\mathrm{U}}}{g} \nabla_ {\mathrm{N}} \\ \varepsilon_ {\mathrm{U}} - \frac {\Omega_ {\mathrm{N}}}{g} \nabla_ {\mathrm{N}} \end{array} \right]\tag{8.5.15}
$$

$$
\boldsymbol {X} _ {2} ^ {o} (t) = \left[ \begin{array}{c} \varepsilon_ {\mathrm{E}} ^ {o} \\ \nabla_ {\mathrm{E}} ^ {o} \\ \nabla_ {\mathrm{N}} ^ {o} \end{array} \right] = \left[ \begin{array}{c} \varepsilon_ {\mathrm{E}} \\ \nabla_ {\mathrm{E}} \\ \nabla_ {\mathrm{N}} \end{array} \right]\tag{8.5.16}
$$

由上述分析知,满秩变换将原系统变换成两个子系统。子系统1的状态为 $X_{1}^{o}(t)$ ,状态方程和量测方程为

$$
\left. \begin{array}{l} \boldsymbol {X} _ {1} ^ {o} (t) = \boldsymbol {F} _ {1} \boldsymbol {X} _ {1} ^ {o} (t) + \boldsymbol {w} _ {1} (t) \\ \boldsymbol {Z} (t) = \boldsymbol {H} _ {1} \boldsymbol {X} _ {1} ^ {o} (t) + \boldsymbol {V} (t) \end{array} \right\}\tag{8.5.17}
$$

子系统 2 的状态为 $X_{2}^{o}(t)$ ，状态方程为

$$
\dot {X} _ {2} ^ {o} (t) = 0\tag{8.5.18}
$$

子系统2由状态 $\varepsilon_{\mathrm{E}},\nabla_{\mathrm{E}}$ 和 $\nabla_{\mathrm{N}}$ 组成，它们都是偏置量，在整个时间过程中不变。因此子系统2本身是稳定的，但不是渐近稳定的。系统的量测 $Z$ 与 $X_2^0 (t)$ 无关，所以 $X_2^o (t)$ 是不可观测的。

根据式 $(4.2.17)$ ，子系统1可观测判别式为

$$
\left[ \begin{array}{l l l l} \boldsymbol {H} _ {1} ^ {\mathrm{T}} & \boldsymbol {F} _ {1} ^ {\mathrm{T}} \boldsymbol {H} _ {1} ^ {\mathrm{T}} & (\boldsymbol {F} _ {1} ^ {2}) ^ {\mathrm{T}} \boldsymbol {H} _ {1} ^ {T} \end{array} \right] = \left[ \begin{array}{c c c c c c} - 1 & 0 & 0 & \Omega_ {\mathrm{U}} & \Omega_ {\mathrm{U}} ^ {2} & 0 \\ 0 & 1 & \Omega_ {\mathrm{U}} & 0 & 0 & - \omega_ {i e} ^ {2} \\ 0 & 0 & 0 & - \Omega_ {\mathrm{N}} & - \Omega_ {\mathrm{U}} \Omega_ {\mathrm{N}} & 0 \\ 0 & 0 & - 1 & 0 & 0 & \Omega_ {\mathrm{U}} \\ 0 & 0 & 0 & 0 & 0 & - \Omega_ {\mathrm{N}} \end{array} \right]
$$

将其中第1，2，3，4，6列组成5阶行列式，并按拉普拉斯展开，得

$$
\left| \begin{array}{c c c c c} - 1 & 0 & 0 & \Omega_ {\mathrm{U}} & 0 \\ 0 & 1 & \Omega_ {\mathrm{U}} & 0 & - \omega_ {i e} ^ {2} \\ 0 & 0 & 0 & - \Omega_ {\mathrm{N}} & 0 \\ 0 & 0 & - 1 & 0 & \Omega_ {\mathrm{U}} \\ 0 & 0 & 0 & 0 & - \Omega_ {\mathrm{N}} \end{array} \right| = - \Omega_ {\mathrm{N}} ^ {2}
$$

这说明，只要 $\Omega_{N} \neq 0$ ，即 $L \neq \pm 90^{\circ}$ ，子系统1是完全可观测的。如果 $\Omega_{N} = 0$ ，则从式(8.5.7)可看出，方位回路与南北回路解耦，无法根据罗经方法作方位对准。这时，方位误差角和方位陀螺偏置是不可观测的，这与罗经法方位对准不能在高纬度地区进行的结论是一致的。

综上所述，变换后的系统可分为可观测和不可观测的两个子系统。不可观测的子系统本身不是渐近稳定的，因此，整个系统是不可检测的，满足不了滤波稳定的要求。但是从式(8.5.17)和式(8.5.18)可看出，两个子系统互相独立， $X_{2}^{o}(t)$ 不但本身稳定，而且也不受 $X_{1}^{o}(t)$ 的影响，始终保持常值，它也不影响 $X_{1}^{o}(t)$ 。因此，可以将 $X_{2}^{o}(t)$ 去掉，仅用子系统1和量测组成滤波器，估计出 $X_{1}^{o}(t)$ 。当然， $X_{1}^{o}(t)$ 是变换后的状态向量， $\varphi_{N}^{o},\varphi_{E}^{o},\varphi_{U}^{o},\varepsilon_{N}^{o}$ 和 $\varepsilon_{U}^{o}$ 都不是原来的状态，而需经过一定的变换关系求出原来的状态。以 $\varphi_{N}$ 为例，根据式(8.5.15)

$$
\varphi_ {\mathrm{N}} = \varphi_ {\mathrm{N}} ^ {o} - \frac {\nabla_ {\mathrm{E}}}{g}
$$

如果 $\varphi_{\mathrm{N}}^{o}$ 的估计完全正确，则平台按 $\hat{\varphi}_{\mathrm{N}}^{o}$ 校正后的残余失准角为

$$
\varphi_ {\mathrm{N}} = \frac {\nabla_ {\mathrm{E}}}{g}
$$

这与式(8.5.5)所示经典对准方案的结论是一样的。这说明初始对准中平台水平失准角受加速度计偏置影响，以及方位失准角受东向陀螺偏置影响的原因是由于这些影响因素是不可观测的。

因为变换后的子系统1的系统阵和量测阵是原系统的 $F_{1}$ 和 $H_{1}$ ，所以，针对滤波器列写系统状态方程和量测方程时，完全可以按原系统列写，状态就是 $X_{0}^{1}(t)$ ，而不必再作变换，即

$$
\left[ \begin{array}{c} \dot {\varphi} _ {N} ^ {o} \\ \dot {\varphi} _ {E} ^ {o} \\ \dot {\varphi} _ {U} ^ {o} \\ \dot {\varepsilon} _ {N} ^ {o} \\ \dot {\varepsilon} _ {U} ^ {o} \end{array} \right] = \left[ \begin{array}{c c c c c} 0 & - \Omega_ {U} & 0 & 1 & 0 \\ \Omega_ {U} & 0 & - \Omega_ {N} & 0 & 0 \\ 0 & \Omega_ {N} & 0 & 0 & 1 \\ 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 \end{array} \right] \left[ \begin{array}{c} \varphi_ {N} ^ {o} \\ \varphi_ {E} ^ {o} \\ \varphi_ {U} ^ {o} \\ \varepsilon_ {N} ^ {o} \\ \varepsilon_ {U} ^ {o} \end{array} \right] + \left[ \begin{array}{c} w _ {N} \\ w _ {E} \\ w _ {U} \\ 0 \\ 0 \end{array} \right]\tag{8.5.19}
$$

$$
\left[ \begin{array}{c} Z _ {\mathrm{E}} \\ Z _ {\mathrm{N}} \end{array} \right] = \left[ \begin{array}{c c c c c} - g & 0 & 0 & 0 & 0 \\ 0 & g & 0 & 0 & 0 \end{array} \right] X _ {1} ^ {o} (t) + \left[ \begin{array}{c} V _ {\mathrm{E}} \\ V _ {\mathrm{N}} \end{array} \right]\tag{8.5.20}
$$

有的文献称这种带圆圈符号的状态为伪状态,意思是说它与真正需要估计的状态尚有差别。也有文献先设 $\varepsilon_{E}=\nabla_{E}=\nabla_{N}=0$ , 用 $\varphi_{N}$ 等状态列写出系统模型方程和量测方程,作滤波估计,然后再考虑这些偏置的影响。

（2）量测值的选择。在实际惯导系统中，加速度计一般都是积分型的，即其输出是加速度的积分量。为了能用作计算机的输入量，它必须经过量化处理。即先将输出量转化为电流脉冲形式，每个脉冲代表一定数量的速度增量 $\Delta v$ ，然后用计数器累计，得到速度量。这样，加速度计输出的形式是整量化的速度增量或速度量。但是量化处理会引入量化误差，这是一种随机量，在量测方程中必须考虑到。下面讨论用速度作量测值的静基座初始对准状态方程和量测方程。

如果不考虑加速度计偏置，则由式(8.5.3)，加速度计的积分输出为

$$
\left. \begin{array}{l} Z _ {\mathrm{E}} = \int_ {0} ^ {t} (- g \varphi_ {\mathrm{N}} + V _ {\mathrm{E}}) \mathrm{d} t + V _ {q \mathrm{E}} \\ Z _ {\mathrm{N}} = \int_ {0} ^ {t} (g \varphi_ {\mathrm{E}} + V _ {\mathrm{N}}) \mathrm{d} t + V _ {q \mathrm{N}} \end{array} \right\}\tag{8.5.21}
$$

式中 $V_{qE}$ 和 $V_{qN}$ 为分别为东向和北向加速度计的量化误差。

令

$$
\left\{ \begin{array}{l} I _ {\mathrm{E}} = - \int_ {0} ^ {t} g \varphi_ {\mathrm{N}} \mathrm{d} t \\ I _ {\mathrm{N}} = \int_ {0} ^ {t} g \varphi_ {\mathrm{E}} \mathrm{d} t \end{array} \right.
$$

则

$$
\left. \begin{array}{l} \dot {I} _ {\mathrm{E}} = - g \varphi_ {\mathrm{N}} \\ \dot {I} _ {\mathrm{N}} = g \varphi_ {\mathrm{E}} \end{array} \right\}\tag{8.5.22}
$$

因为 $I_{E}$ 和 $I_{N}$ 是量测值中的一部分，式(8.5.22) 又说明了它与状态 $\varphi_{N}$ 和 $\varphi_{E}$ 的关系，所以尚须扩充 $I_{E}$ 和 $I_{N}$ 为状态。

令

$$
\begin{array}{r l} & M _ {\mathrm{E}} = \int_ {0} ^ {t} V _ {\mathrm{E}} \mathrm{d} t \\ & M _ {\mathrm{N}} = \int_ {0} ^ {t} V _ {\mathrm{N}} \mathrm{d} t \end{array}
$$

它们分别是东向和北向加速度计输出中的白噪声分量的积分,则

$$
\left. \begin{array}{l} \dot {M} _ {\mathrm{E}} = V _ {\mathrm{E}} \\ \dot {M} _ {\mathrm{N}} = V _ {\mathrm{N}} \end{array} \right\}\tag{8.5.23}
$$

显然 $M_{E}$ 和 $M_{N}$ 是随机游走，是有色噪声，应该扩充为状态，但考虑到 $V_{E}$ 和 $V_{N}$ 是由于加速度计摆锤质量受电子伺服回路干扰而产生的摆动噪声，它的积分量与量化误差相比一般较小，为使问题简化，此处不考虑这种误差。这样量测方程为

$$
\left. \begin{array}{l} Z _ {\mathrm{E}} = I _ {\mathrm{E}} + V _ {q \mathrm{E}} \\ Z _ {\mathrm{N}} = I _ {\mathrm{N}} + V _ {q \mathrm{N}} \end{array} \right\}\tag{8.5.24}
$$

综上所述，状态方程为

$$
\left[ \begin{array}{l} \dot {I} _ {\mathrm{E}} \\ \dot {I} _ {\mathrm{N}} \\ \dot {\varphi} _ {\mathrm{N}} \\ \dot {\varphi} _ {\mathrm{E}} \\ \dot {\varphi} _ {\mathrm{U}} \\ \dot {\varepsilon} _ {\mathrm{N}} \\ \dot {\varepsilon} _ {\mathrm{U}} \end{array} \right] = \left[ \begin{array}{c c c c c c c} 0 & 0 & - g & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & g & 0 & 0 & 0 \\ 0 & 0 & 0 & - \Omega_ {\mathrm{U}} & 0 & 1 & 0 \\ 0 & 0 & \Omega_ {\mathrm{U}} & 0 & - \Omega_ {\mathrm{N}} & 0 & 0 \\ 0 & 0 & 0 & \Omega_ {\mathrm{N}} & 0 & 0 & 1 \\ 0 & 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 & 0 \end{array} \right] \left[ \begin{array}{l} I _ {\mathrm{E}} \\ I _ {\mathrm{N}} \\ \varphi_ {\mathrm{N}} \\ \varphi_ {\mathrm{E}} \\ \varphi_ {\mathrm{U}} \\ \varepsilon_ {\mathrm{N}} \\ \varepsilon_ {\mathrm{U}} \end{array} \right] + \left[ \begin{array}{l} 0 \\ 0 \\ w _ {\mathrm{N}} \\ w _ {\mathrm{E}} \\ w _ {\mathrm{U}} \\ 0 \\ 0 \end{array} \right]\tag{8.5.25}
$$

式(8.5.25)和式(8.5.24)组成了静基座条件下,利用速度作为量测值的初始对准系统的模型方程和量测方程。同样可以判别出这些状态都是可观测的。

从上可看出,为了适应实际系统的加速度计类型,并考虑量化噪声这个误差因素,常用速度量作为量测值,使量测方程较真实。但是,这增加了状态,增加了计算量。

与采用速度量类似,还可采用速度增量作为量测值 $^{[92]}$ ,即量测的采样时间采用两个速度脉冲出现的间隔时间(不是定值),量测值就是一个速度脉冲。这种方法可以免除量化噪声,改善对准的准确度,但是计算比较复杂,此处不再讨论。

## 2. 晃动基座条件下系统方程和量测方程的列写

机载惯导作自对准时，飞机停留在停机坪上，发动机不准启动，并尽可能减少人为干扰。但由于阵风、加油等影响，飞机仍会产生晃动。而惯性平台不可能正好安装在飞机的质心上，飞机的晃动使平台受到线加速度的干扰，这种干扰会反映到初始对准的量测方程中。因此，必须在状态方程中增加描述这种干扰的微分方程。

阵风对飞机的干扰是随机性的力干扰，一般可用一阶马尔柯夫过程来描述。而飞机对力干扰的位移响应可用二阶振荡环节来描述。如图8.5.1所示。图中： $f(t)$ 为随机力干扰； $n_a(t), n_v(t), n_D(t)$ 分别为加速度、速度、位移响应； $m(t)$ 是零均值的单位强度白噪声； $R_f(0)$ 是力干扰的方差； $\alpha$ 是力干扰的反相关时间，相关时间一般为数秒到数十秒； $\zeta, \omega_r$ 和 $K$ 分别为平台安装部位线晃动的阻尼系数、固有频率和弹性系数。

![](images/3bec9f44b752c1b72280728beb4a952a44b65c03f27bdd7c08a14a2cb9d4bef5.jpg)  
图 8.5.1 飞机对随机力干扰的加速度、速度和位移响应

如果初始对准采用速度作量测，则量测值受摆动基座的影响而增加了速度噪声 $n_{v}(t)$ 。在式(8.5.24)和式(8.5.25)基础上考虑晃动的影响，则量测方程为

$$
\left. \begin{array}{l} Z _ {\mathrm{E}} = I _ {\mathrm{E}} + n _ {v \mathrm{E}} + V _ {q \mathrm{E}} \\ Z _ {\mathrm{N}} = I _ {\mathrm{N}} + n _ {v \mathrm{N}} + V _ {q \mathrm{N}} \end{array} \right\}\tag{8.5.26}
$$

为简化分析,设飞机沿两个方向的晃动是独立的,即 $n_{vE}$ 和 $n_{vN}$ 互不相关。在模型状态方程中,必须增加对 $n_{vE}$ 和 $n_{vN}$ 作描述的微分方程。对 $n_{vE}$ ,有

$$
\begin{array}{r l} \left[ \begin{array}{c} \dot {n} _ {D E} \\ \dot {n} _ {v E} \\ \dot {n} _ {a E} \end{array} \right] & = \left[ \begin{array}{c c c} 0 & 1 & 0 \\ 0 & 0 & 1 \\ - \alpha \omega_ {r E} ^ {2} & - (2 \zeta_ {E} \omega_ {r E} \alpha + \omega_ {r E} ^ {2}) & - (\alpha + 2 \zeta_ {E} \omega_ {r E}) \end{array} \right] \\ & n _ {f E} = \frac {\sqrt {2 R _ {f} (0) \alpha} \omega_ {r E} ^ {2} m (t)}{K _ {E}} \end{array} \quad \left[ \begin{array}{c} n _ {D E} \\ n _ {v E} \\ n _ {a E} \end{array} \right] + \left[ \begin{array}{c} 0 \\ 0 \\ n _ {f E} \end{array} \right]\tag{8.5.27}
$$

式中

类似地,可得 $n_{vN}$ 的描述方程。这样,将描述这种噪声的 6 个微分方程扩增到式(8.5.25)中,状态共有 13 个。

从上述分析可看出,在考虑了晃动影响后,系统模型状态的阶数增加较多。虽然能改善对准的准确度,但却大大增加了计算量。

## 3. 根据估计作校正的具体措施

晃动基座初始对准的滤波器可对13个状态作出估计，但是，需要利用的只有3个平台误差角和2个陀螺偏置。这些估计并不是用作输出，而是用于校正的。此处介绍反馈校正的具体实施。

在晃动基座条件下,状态向量为

$$
\boldsymbol {X} = \left[ n _ {D E} \quad n _ {v E} \quad n _ {a E} \quad n _ {D N} \quad n _ {v N} \quad n _ {a N} \quad I _ {E} \quad I _ {N} \quad \varphi_ {N} \quad \varphi_ {E} \quad \varphi_ {U} \quad \varepsilon_ {N} \quad \varepsilon_ {U} \right] ^ {\mathrm{T}}
$$

设

$$
\mathbf {K} _ {k} = \left[ \begin{array}{c c} K _ {k} ^ {1, 1} & K _ {k} ^ {1, 2} \\ \vdots & \vdots \\ K _ {k} ^ {1 3, 1} & K _ {k} ^ {1 3, 2} \end{array} \right]
$$

则按式 $(5.4.5)$ 和式 $(5.4.9)$ ，估计为

$$
\left[ \begin{array}{l} \hat {\varphi} _ {k} ^ {\mathrm{N}} \\ \hat {\varphi} _ {k} ^ {\mathrm{E}} \\ \hat {\varphi} _ {k} ^ {\mathrm{U}} \\ \hat {\varepsilon} _ {k} ^ {\mathrm{N}} \\ \hat {\varepsilon} _ {k} ^ {\mathrm{U}} \end{array} \right] = \left[ \begin{array}{l l} K _ {k} ^ {9, 1} & K _ {k} ^ {9, 2} \\ K _ {k} ^ {1 0, 1} & K _ {k} ^ {1 0, 2} \\ K _ {k} ^ {1 1, 1} & K _ {k} ^ {1 1, 2} \\ K _ {k} ^ {1 2, 1} & K _ {k} ^ {1 2, 2} \\ K _ {k} ^ {1 3, 1} & K _ {k} ^ {1 3, 2} \end{array} \right] \quad \left[ \begin{array}{l} Z _ {k} ^ {\mathrm{E}} \\ Z _ {k} ^ {\mathrm{N}} \end{array} \right]\tag{8.5.28}
$$

平台误差角的校正量是持续的施矩电流。虽然陀螺偏置的校正量也是持续的施矩电流，但它一经施加给陀螺，就能抵消偏置的作用，其性质与脉冲控制量相似。如果认为 $t_k$ 时刻就能计算得到估计 $\hat{X}_k$ ，则按式(5.4.3)，陀螺偏置的校正量（施矩电流）为

$$
\begin{array}{l} u _ {k} ^ {\varepsilon_ {N}} = - \hat {\varepsilon} _ {k} ^ {N} \\ u _ {k} ^ {\varepsilon_ {U}} = - \hat {\varepsilon} _ {k} ^ {U} \end{array}
$$

需要说明的是:陀螺偏置的估计 $\hat{\varepsilon}_{k}^{N}$ 和 $\hat{\varepsilon}_{k}^{U}$ 是在k-1时刻的估计 $\hat{\varepsilon}_{k-1}^{N}$ 和 $\hat{\varepsilon}_{k-1}^{U}$ 以及校正量 $u_{k-1}^{\varepsilon_{N}}$ 和 $u_{k-1}^{\varepsilon_{U}}$ 的基础上作出的。因此,前后时刻的校正量应该相加,上式应改写为

$$
\begin{array}{l} u _ {k} ^ {\varepsilon_ {\mathrm{N}}} = - \hat {\varepsilon} _ {k} ^ {\mathrm{N}} + u _ {k - 1} ^ {\varepsilon_ {\mathrm{N}}} \\ u _ {k} ^ {\varepsilon_ {\mathrm{U}}} = - \hat {\varepsilon} _ {k} ^ {\mathrm{U}} + u _ {k - 1} ^ {\varepsilon_ {\mathrm{U}}} \end{array}
$$

下面计算平台误差角校正量。根据式(5.4.16)，平台误差角持续校正量（施矩电流）为

$$
\boldsymbol {B} _ {k} \left[ \begin{array}{c} u _ {k} ^ {\varphi_ {\mathrm{N}}} \\ u _ {k} ^ {\varphi_ {\mathrm{E}}} \\ u _ {k} ^ {\varphi_ {\mathrm{U}}} \end{array} \right] = - \left[ \begin{array}{c c c} 1 & - \Omega_ {\mathrm{U}} T & 0 \\ \Omega_ {\mathrm{U}} T & 1 & - \Omega_ {\mathrm{N}} T \\ 0 & \Omega_ {\mathrm{N}} T & 1 \end{array} \right] \left[ \begin{array}{c} \hat {\varphi} _ {k} ^ {\mathrm{N}} \\ \hat {\varphi} _ {k} ^ {\mathrm{E}} \\ \hat {\varphi} _ {k} ^ {\mathrm{U}} \end{array} \right]\tag{8.5.29}
$$

式中，一步转移阵只计算到线性项。根据式(5.4.13)

$$
\boldsymbol {B} _ {x} = \int_ {t _ {k}} ^ {t _ {k + 1}} \left[ \begin{array}{c c c} 1 & - \Omega_ {\mathrm{U}} t & 0 \\ \Omega_ {\mathrm{U}} t & 1 & - \Omega_ {\mathrm{N}} t \\ 0 & \Omega_ {\mathrm{N}} t & 1 \end{array} \right] \mathrm{d} t = \left[ \begin{array}{c c c} T & - \frac {1}{2} \Omega_ {\mathrm{U}} T ^ {2} & 0 \\ \frac {1}{2} \Omega_ {\mathrm{U}} T ^ {2} & T & - \frac {1}{2} \Omega_ {\mathrm{N}} T ^ {2} \\ 0 & \frac {1}{2} \Omega_ {\mathrm{N}} T ^ {2} & T \end{array} \right]
$$

将上式代入式(8.5.29)，得

$$
\left. \begin{array}{l} u _ {k} ^ {\varphi_ {\mathrm{N}}} = - \left(\frac {\hat {\varphi} _ {k} ^ {\mathrm{N}}}{T} - \frac {\Omega_ {\mathrm{U}} \hat {\varphi} _ {k} ^ {\mathrm{E}}}{2}\right) \\ u _ {k} ^ {\varphi_ {\mathrm{E}}} = - \left(\frac {\hat {\varphi} _ {k} ^ {\mathrm{E}}}{T} + \frac {\Omega_ {\mathrm{U}} \hat {\varphi} _ {k} ^ {\mathrm{N}}}{2} - \frac {\Omega_ {\mathrm{N}} \hat {\varphi} _ {k} ^ {\mathrm{N}}}{2}\right) \\ u _ {k} ^ {\varphi_ {\mathrm{U}}} = \left(\frac {\hat {\varphi} _ {k} ^ {\mathrm{U}}}{T} + \frac {\Omega_ {\mathrm{N}} \hat {\varphi} _ {k} ^ {\mathrm{E}}}{2}\right) \end{array} \right\}\tag{8.5.30}
$$

式(8.5.29)和式(8.5.30)就是利用滤波估计对平台失准角和陀螺偏置作反馈校正的校正量(控制量)。一般情况下,初始对准的计算周期较短,3个误差角之间的交联影响比失准角本身要小得多。因此有时略去交联影响,各轴各自校正失准角本身,控制量的计算就简单得多。还须说明, $u_{k}^{\varepsilon_{N}}$ 和 $u_{k}^{\varphi_{N}}$ 都是对平台北向轴的施矩量,应该相加合并, $u_{k}^{\varepsilon_{U}}$ 和 $u_{k}^{\varphi_{U}}$ 的情况也一样。

如图 8.5.2 所示给出了东向回路用卡尔曼滤波反馈校正进行初始对准的过程。

![](images/66c1891673fbf378b131e13fc188caf2389f6e3efb8ab6ccd6122313d9823c59.jpg)  
图 8.5.2 用反馈校正进行初始对准的平台东向回路方块图

虽然利用卡尔曼滤波器的估计作初始对准是在粗对准完毕的基础上进行的,但是,平台倾角的初始值仍可达 $1^{\circ}$ 左右。而量测值直接与平台倾角有关,这使倾角估计在开始几步递推计算中就能很快接近真值。因此,在每步时间T内要完全校正掉估计量,施矩电流必然很大,这对没有大施矩电流的惯导系统就无法实现,但可由以下办法补救:

(1) 初始对准只作估计, 对准结束后再加施矩电流一次校正;

（2）估计开始几秒钟停止估计，先用施矩电流一次校正，然后重新启动滤波器再次估计。当再次估计时，因为平台倾角已经很小，状态间相互耦合量更小，所以更便于采用次优滤波方法。

例8-1 文献[93]介绍了地-地导弹惯导系统采用卡尔曼滤波进行初始对准的情况，地

理坐标系以天、南、东为正，系统模型状态方程为

$$
\dot {\boldsymbol {X}} = \boldsymbol {F} \boldsymbol {X} + \boldsymbol {w}
$$

式中

$$
\begin{array}{r l r l} \mathbf {X} & = [ I _ {\mathrm{S}} - I _ {\mathrm{E}} - n _ {v \mathrm{S}} - n _ {v \mathrm{E}} - n _ {a \mathrm{S}} - n _ {D \mathrm{S}} - n _ {a \mathrm{E}} - n _ {D \mathrm{E}} - \varphi_ {\mathrm{U}} - \varphi_ {\mathrm{S}} - \varphi_ {\mathrm{E}} - \varepsilon_ {\mathrm{U}} - \varepsilon_ {\mathrm{S}} ] ^ {\mathrm{T}} \\ & F _ {1, 1 1} = - g & & F _ {2, 1 0} = g \\ & F _ {3, 5} = 1 & & F _ {4, 7} = 1 \\ & F _ {5, 3} = - (2 \zeta \omega_ {r} \alpha + \omega_ {r} ^ {2}) & & F _ {5, 5} = - (\alpha + 2 \zeta \omega_ {r}) \\ & F _ {5, 6} = - \alpha \omega_ {r} ^ {2} & & F _ {6, 3} = 1 \\ & F _ {7, 4} = - (2 \zeta \omega_ {r} \alpha + \omega_ {r} ^ {2}) & & F _ {7, 7} = - (\alpha + 2 \zeta \omega_ {r}) \\ & F _ {7, 8} = - \alpha \omega_ {r} ^ {2} & & F _ {8, 4} = 1 \\ & F _ {9, 1 1} = \omega_ {i e} \cos L & & F _ {9, 1 2} = 1 \\ & F _ {1 0, 1 1} = \omega_ {i e} \sin L & & F _ {1 0, 1 3} = 1 \\ & F _ {1 1, 9} = - \omega_ {i e} \cos L & & F _ {1 1, 1 0} = - \omega_ {i e} \sin L \\ & w _ {5} = n _ {f S} & & w _ {7} = n _ {f E} \end{array}
$$

除上述非零元外， $F_{13\times13}$ 和 $w_{13\times1}$ 的其余元全部为零。此处， $\alpha=0.1/s,\zeta=0.1,\omega_{r}=2.09rad/s$ 。因为导弹在发射前垂直放置，所以两个水平方向晃动的有关参数是相同的，即

$$
E [ n _ {f \mathrm{S}} ^ {2} ] = E [ n _ {f \mathrm{E}} ^ {2} ] = q _ {f}
$$

量测方程为

$$
\mathbf {Z} _ {k} = \mathbf {H X} _ {k} + \mathbf {V}
$$

式中 $\pmb{H}$ 的非零元为

$$
\begin{array}{r l} & H _ {1, 1} = 1, \quad H _ {1, 3} = 1, \quad H _ {2, 2} = 1, \quad H _ {2, 4} = 1 \\ & E [ \mathbf {V} \mathbf {V} ^ {\mathrm{T}} ] = \mathrm{diag} [ r _ {a} \quad r _ {a} ] \end{array}
$$

滤波采用开环估计的方法,量测更新周期为 1 s。滤波效果采用计算机仿真的方法求得。仿真中采用的数据为

$$
\begin{array}{r l} & {\varphi_ {\mathrm{S}} (0) = \varphi_ {\mathrm{E}} (0) = \varphi_ {\mathrm{U}} (0) = 1 ^ {\circ}} \\ & {\varepsilon_ {\mathrm{S}} = \varepsilon_ {\mathrm{U}} = 0. 1 5 ^ {\circ} / \mathrm{h}, \quad \varepsilon_ {\mathrm{E}} = 0} \\ & {E [ \varphi_ {\mathrm{S}} ^ {2} (0) ] = E [ \varphi_ {\mathrm{E}} ^ {2} (0) ] = E [ \varphi_ {\mathrm{U}} ^ {2} (0) ] = (1 ^ {\circ}) ^ {2}} \\ & {E [ \varepsilon_ {\mathrm{S}} ^ {2} ] = E [ \varepsilon_ {\mathrm{U}} ^ {2} ] = (0. 1 5 ^ {\circ} / \mathrm{h}) ^ {2}} \\ & {E [ n _ {D} ^ {2} ] = 1 0 0 \: \mathrm{cm} ^ {2}, \quad E [ n _ {v} ^ {2} ] = 4 4 0 (\mathrm{cm/s}) ^ {2}} \\ & {E [ n _ {a} ^ {2} ] = 1 9 0 0 (\mathrm{cm/s} ^ {2}) ^ {2}} \end{array}
$$

设初始状态的均值都为零，则以上各均方值就是 $P(0)$ 阵中的对角线元素。除了以上5类初始估计均方误差值外， $P(0)$ 阵的其他项都为零，初始状态估计为零。

仿真结果表明:两个水平倾角的估计误差在滤波开始后数秒钟即可降得很低,方位估计误差和两个陀螺偏置的估计误差分别如图8.5.3～图8.5.5所示。这些诸图中给出了全阶滤波器和次优滤波器的仿真结果。从这些图中可看出,方位估计误差、方位陀螺偏置和南向陀螺偏置的估计误差达到最小的时间各约为15 min,40 min,10 min。这与用校正环节的经典初始对准方案的结果也是一致的,即水平对准时间最短,方位对准和南向陀螺测漂次之,方位陀螺测漂时间最长。如图8.5.6还示出了南向加速度计输出 $Z_{S}$ 对 $\varphi_{U}$ 和 $I_{S}$ 估计的增益变化曲线。模拟计算表明,交叉影响的增益始终是很小的(例如 $Z_{S}$ 对 $\varphi_{S}$ , $I_{E}$ 和 $\varepsilon_{S}$ 等状态估计的增益)。

须要指出的是:晃动干扰 $n_{a}$ 的估计不太准确。这是因为量测的采样周期为 1 s, 而导弹的晃动周期为 3 s, 虽然满足香农采样定理, 但余量过小, 采样仍有一定的失真。若缩短采样周期, 则能改善估计效果。

## 4. 次优滤波器设计

初始对准采用的滤波器与组合导航的滤波器有相似之处，两者都使用误差量作为状态。初始对准时飞机停放在停机坪上，速度为零，惯导输出就是误差量。组合导航系统中，两个非相似导航系统相应导航参数（如速度）之差仅包含误差量。因此，有可能将两种滤波器的计算程序统一，稍作修改就能使用。在这种情况下，初始对准滤波器常采用两个加速度计回路相互交联在一起的系统模型状态方程，阶数较高。一般大型飞机导航设备齐全，有组合导航功能，常采用这种方案。对小型飞机，如果仅在初始对准时使用滤波器，或即使在飞行过程中采用组合导航系统，也只是采用简化的方案，则初始对准的滤波器常采用次优方案。次优方案很多，下面介绍常见的几种。

![](images/9573707fbf2436d4214062c4c18feecbfc5b4df56a3b186f74a7760bc95b5fab.jpg)  
图 8.5.3 方位估计误差曲线

![](images/7ac25ee5d7e0fd45e833e90e4d06cca2960d617eb135469665d2f9938e916237.jpg)  
图 8.5.4 方位陀螺偏置估计误差曲线

![](images/d32cd8eabfacf196f30938a4a0cd97c9e51eae7ef0f1e488dd100d1a916c15de.jpg)  
图 8.5.5 南向陀螺偏置估计误差曲线

![](images/add7d1b2b7e13882a51bfa9f0833926f917c7f00e7cb8c86e83c2fbd308cad61.jpg)  
图 8.5.6 滤波增益曲线

（1）回路间解耦。上小节曾指出，两个加速度计回路交叉耦合对增益的影响较小。由式(8.5.25)可看出，这种耦合是由地球自转角速度的天向分量 $\Omega_{\mathrm{U}}$ 在平台的两个水平轴上的投影分量引起的。如果略去这两个投影分量，则两个回路就完全解耦，可设计成两个独立的滤

波器,各自状态向量为

$$
\begin{array}{r l} & {\left[ I _ {\mathrm{S}} n _ {v \mathrm{S}} n _ {a \mathrm{S}} n _ {D \mathrm{S}} \varphi_ {\mathrm{U}} \varphi_ {\mathrm{E}} \varepsilon_ {\mathrm{U}} \right] ^ {\mathrm{T}}} \\ & {\left[ I _ {\mathrm{E}} n _ {v \mathrm{E}} n _ {a \mathrm{E}} n _ {D \mathrm{E}} \varphi_ {\mathrm{S}} \varepsilon_ {\mathrm{S}} \right] ^ {\mathrm{T}}} \end{array}
$$

量测值各为 $Z_{S}$ 和 $Z_{E}$ 。这样，计算量可大为减少。实际上，利用校正环节的经典对准方案常将系统解耦成两个独立回路。

（2）增益分段典型化。卡尔曼滤波计算增益阵时须作矩阵求逆，计算量较大。因此，可事先将增益曲线分段与典型曲线拟合，并将拟合曲线参数存储在计算机中。这样，初始对准就可直接利用存储数据而不必对增益作实时计算。

对例8-1所述系统，如果采用解耦和增益分段典型化的方法组成次优滤波器，则具体的典型曲线如表8.5.1所列。这种次优滤波器的估计误差分别见图8.5.4～图8.5.6中的次优滤波曲线。可以看出，与全阶滤波器的结果是比较接近的。在一定条件和要求下，这种方法是可以采用的。

表 8.5.1 典型曲线

<table><tr><td>状态</td><td>近似增益的曲线形式</td></tr><tr><td> $I_{S}$ ,  $I_{E}$ </td><td>指数曲线</td></tr><tr><td> $n_{vS}$ ,  $n_{vE}$ </td><td>直线(增益为常值)</td></tr><tr><td> $n_{aS}$ ,  $n_{aE}$ </td><td>直线(增益为常值)</td></tr><tr><td> $n_{DS}$ ,  $n_{DE}$ </td><td>直线(增益为常值)</td></tr><tr><td> $φ_{U}$ </td><td>直线</td></tr><tr><td> $φ_{E}$ ,  $φ_{S}$ </td><td>指数曲线</td></tr><tr><td> $ε_{U}$ ,  $ε_{S}$ </td><td>直线</td></tr></table>

（3）方位陀螺测漂的取舍。从图8.5.4看出，方位陀螺测漂时间很长，这是由于该状态在量测值 $Z_{S}$ 中的信息比例很小。因此，不但估计时间长，而且估计精度受模型统计参数不准确的影响也较大。图中次优滤波器的估计误差就明显地增加。对飞机初始对准，一般都要求在10～15 min以内甚至更短时间内完成。而在短时间内要对 $\varepsilon_{U}$ 进行估计是很困难的。因此在对准时间受限的条件下，常略去状态 $\varepsilon_{U}$ 和对它的估计。只要 $\varepsilon_{U}$ 的数值在1～2 meru（地球自转角速度的千分之一）以下，忽略 $\varepsilon_{U}$ 引起的对其他状态估计的影响是较小的。

（4）基座运动干扰的简化。为了描述基座运动的规律，在模型中增加了6个状态，即2个线加速度干扰，2个速度干扰和2个位置干扰，这些干扰都是随机的。在次优条件下可以考虑简化这些随机干扰。最简单的方法就是将两个线加速度干扰简化为白噪声。根据量测是加速度还是速度，可以全部忽略这6个状态或其中的大部分。对初始对准精度要求不甚高的系统常采用这种方案。

## 8.5.3 捷联惯导系统的自对准滤波器设计

使用最优估计理论实现惯导系统的初始对准,都是估计出惯导系统的失准角。在平台式惯导系统中,估计出的失准角被用来控制平台旋转,使平台的各轴(惯性器件的敏感轴)与要求的导航坐标系重合;而在捷联惯导系统中,失准角的估计量用来修正姿态矩阵,事实上是一种信号处理方法。捷联惯导初始对准的方法有很多种,此处介绍在晃动基座上采用最优估计

算法实现的参数辨识法对准 $^{[104]}$ 。

1. 数学平台及其对基座晃动的动力学隔离作用

设 b 为机体坐标系(右、前、上)，导航坐标系 n 取地理坐标系(东、北、天)，则姿态阵满足：

$$
\dot {\boldsymbol {C}} _ {b} ^ {n} = \boldsymbol {C} _ {b} ^ {n} \boldsymbol {\Omega} _ {n b} ^ {b}\tag{8.5.31}
$$

式中 $\Omega_{nb}^{b}$ 是由 $\omega_{nb}^{b}$ 构造的反对称阵。显然，由于对准时飞机不作移动， $\omega_{en}=0,\quad\omega_{nb}$ 就是载体的晃动角速度 $\omega_{d}$ ，所以

$$
\omega_ {i b} = \omega_ {i e} + \omega_ {d}
$$

式中 $\omega_{ib}$ 可由捷联陀螺测量， $\omega_{ie}^{n}$ 准确已知，这样

$$
\boldsymbol {\omega} _ {d} ^ {b} = \boldsymbol {\omega} _ {i b} ^ {b} - C _ {n} ^ {b} \boldsymbol {\omega} _ {i e} ^ {n}\tag{8.5.32}
$$

则式 $(8.5.31)$ 成

$$
\dot {\boldsymbol {C}} _ {b} ^ {n} = \boldsymbol {C} _ {b} ^ {n} \boldsymbol {\Omega} _ {d} ^ {b}\tag{8.5.33}
$$

由于捷联陀螺具有测量误差, $C_{n}^{b}$ 由计算获得,带有一定的误差,记为 $C_{n'}^{b}$ ,其中 $n'$ 相对n具有姿态误差角,所以式(8.5.32)应为

$$
\hat {\boldsymbol {\omega}} _ {d} ^ {b} = \widetilde {\boldsymbol {\omega}} _ {i b} ^ {b} - \mathbf {C} _ {n ^ {\prime}} ^ {b} \boldsymbol {\omega} _ {i e} ^ {n}\tag{8.5.34}
$$

式中 $\tilde{\omega}_{ib}^{b}$ 为捷联陀螺的输出

$$
\boldsymbol {\omega} _ {i e} ^ {n} = \left[ \begin{array}{l l l} 0 & \omega_ {i e} \cos L & \omega_ {i e} \sin L \end{array} \right] ^ {\mathrm{T}}\tag{8.5.35}
$$

L 为对准地纬度, 是已知值。

这样式(8.5.31)成

$$
\dot {\mathbf {C}} _ {b} ^ {n ^ {\prime}} = \mathbf {C} _ {b} ^ {n ^ {\prime}} \hat {\boldsymbol {\Omega}} _ {d} ^ {b}\tag{8.5.36}
$$

式中 $C_{b}^{n'}$ 是比式(8.5.34) 中的 $C_{n'}^{b}$ 更新了一步的姿态阵， $\hat{\Omega}_{d}^{b}$ 是 $\hat{\omega}_{d}^{b}$ 构造的反对称阵。

设 $n'$ 坐标系相对 n 坐标系的失准角为 $\varphi_{E}, \varphi_{N}, \varphi_{U}$ ，经粗对准后它们都是小角，所以

$$
C _ {n} ^ {n ^ {\prime}} = I - E\tag{8.5.37}
$$

式中

$$
\boldsymbol {E} = \left[ \begin{array}{c c c} 0 & - \varphi_ {\mathrm{U}} & \varphi_ {\mathrm{N}} \\ \varphi_ {\mathrm{U}} & 0 & - \varphi_ {\mathrm{E}} \\ - \varphi_ {\mathrm{N}} & \varphi_ {\mathrm{E}} & 0 \end{array} \right]\tag{8.5.38}
$$

如果能求得 $\varphi_{E}, \varphi_{N}, \varphi_{U}$ ，则按下式修正获得

$$
\boldsymbol {C} _ {b} ^ {n} = \boldsymbol {C} _ {n ^ {\prime}} ^ {n} \boldsymbol {C} _ {b} ^ {n ^ {\prime}} = (\boldsymbol {I} + \boldsymbol {E}) \boldsymbol {C} _ {b} ^ {n ^ {\prime}}\tag{8.5.39}
$$

式中 $C_{b}^{n'}$ 按式(8.5.36)确定。因此对准问题成为如何确定出 $\varphi_{E}, \varphi_{N}$ 和 $\varphi_{U}$ 。

可以证明,姿态误差角满足如下方程 $^{[104]}$ :

$$
\left[ \begin{array}{c} \dot {\varphi} _ {\mathrm{E}} \\ \dot {\varphi} _ {\mathrm{N}} \\ \dot {\varphi} _ {\mathrm{U}} \end{array} \right] = \left[ \begin{array}{c c c} 0 & \omega_ {i e} \sin L & - \omega_ {i e} \cos L \\ - \omega_ {i e} \sin L & 0 & 0 \\ \omega_ {i e} \cos L & 0 & 0 \end{array} \right] \quad \left[ \begin{array}{c} \varphi_ {\mathrm{E}} \\ \varphi_ {\mathrm{N}} \\ \varphi_ {\mathrm{U}} \end{array} \right] - \left[ \begin{array}{c} \varepsilon_ {\mathrm{E}} \\ \varepsilon_ {\mathrm{N}} \\ \varepsilon_ {\mathrm{U}} \end{array} \right] - \left[ \begin{array}{c} W _ {\mathrm{E}} \\ W _ {\mathrm{N}} \\ W _ {\mathrm{U}} \end{array} \right]\tag{8.5.40}
$$

式中， $\left[\varepsilon_{E}\quad\varepsilon_{N}\quad\varepsilon_{U}\right]^{T}$ 为捷联陀螺的等效随机常值漂移， $\left[W_{E}\quad W_{N}\quad W_{U}\right]^{T}$ 为等效时变漂移，它是由陀螺的安装误差和刻度系数误差引起的对晃动角速度的测量误差。

文献[104]证明得如下结论：

当飞机接近水平，陀螺的安装误差为 $10^{\prime \prime}$ ，刻度系数误差为 $50 \times 10^{-6}$ 时，由陀螺的时变漂移引起姿态误差角不超过 $0.04''$ 的常值偏置量和 $5 \times 10^{-6} \,^{\circ}/h$ 的漂移量，这些影响是可以忽略的，所以式(8.5.40)的解为

$$
\left. \begin{array}{l} \varphi_ {\mathrm{E}} = \varphi_ {\mathrm{E0}} + u _ {\mathrm{E}} t + \frac {t ^ {2}}{2} \omega_ {i e} (u _ {\mathrm{N}} \sin L - u _ {\mathrm{U}} \cos L) \\ \varphi_ {\mathrm{N}} = \varphi_ {\mathrm{N0}} + u _ {\mathrm{N}} t - \frac {t ^ {2}}{2} u _ {\mathrm{E}} \omega_ {i e} \sin L \\ \varphi_ {\mathrm{U}} = \varphi_ {\mathrm{U0}} + u _ {\mathrm{U}} t + \frac {t ^ {2}}{2} u _ {\mathrm{E}} \omega_ {i e} \sin L \end{array} \right\}\tag{8.5.41}
$$

式中 $\varphi_{\mathrm{E0}}$ ， $\varphi_{\mathrm{N0}}$ ， $\varphi_{\mathrm{U0}}$ 分别为 $\varphi_{\mathrm{E}}$ ， $\varphi_{\mathrm{N}}$ ， $\varphi_{\mathrm{U}}$ 的初值

$$
\left. \begin{array}{l} u _ {\mathrm{E}} = \varphi_ {\mathrm{N0}} \omega_ {i e} \sin L - \varphi_ {\mathrm{U0}} \omega_ {i e} \cos L - \varepsilon_ {\mathrm{E}} \\ u _ {\mathrm{N}} = - \varphi_ {\mathrm{E0}} \omega_ {i e} \sin L - \varepsilon_ {\mathrm{N}} \\ u _ {\mathrm{U}} = \varphi_ {\mathrm{E0}} \omega_ {i e} \cos L - \varepsilon_ {\mathrm{U}} \end{array} \right\}\tag{8.5.42}
$$

式 $(8.5.41)$ 说明：

（1）姿态误差角与晃动干扰无关，这是由于式(8.5.36)对飞机晃动的跟踪计算有效地隔离了晃动干扰，建立起了数学平台。

（2）确定姿态误差问题转化为确定其初值和等效常值漂移，初始对准问题转化为参数辨识问题。

## 2. 由参数辨识法确定姿态误差角

在晃动基座上，加速度计感测两种成分：重力加速度和晃动干扰加速度。设加速度计具有安装误差 $\delta A_{x}$ ， $\delta A_{y}$ ， $\delta A_{z}$ ，刻度系数误差 $\delta K_{Ax}$ ， $\delta K_{Ay}$ ， $\delta K_{Az}$ 及偏置量 $\nabla_{x}$ ， $\nabla_{y}$ ， $\nabla_{z}$ ，则加速度计的输出为：

$$
\begin{array}{r l} \widetilde {f} ^ {b} & = (I + \delta K _ {A}) (I + \delta A) (- g ^ {b} + f _ {d} ^ {b}) + \nabla^ {b} = \\ & - g ^ {b} + f _ {d} ^ {b} + (\delta K _ {A} + \delta A) (- g ^ {b} + f _ {d} ^ {b}) + \nabla^ {b} \end{array}\tag{8.5.43}
$$

式中

$$
\begin{array}{r l} & \delta K _ {A} = \mathrm{diag} [ \delta K _ {A x} \quad \delta K _ {A y} \quad \delta K _ {A z} ] \\ & \delta A = \left[ \begin{array}{c c c} 0 & \delta A _ {y} & - \delta A _ {z} \\ - \delta A _ {y} & 0 & \delta A _ {x} \\ \delta A _ {z} & - \delta A _ {x} & 0 \end{array} \right] \\ & \nabla^ {b} = [ \nabla_ {x} \quad \nabla_ {y} \quad \nabla_ {z} ] ^ {\mathrm{T}} \end{array}
$$

$\tilde{f}^{b}$ 在由式(8.5.36)建立的导航坐标系 $n^{\prime}$ 内的分量为

$$
\begin{array}{r l} \widetilde {\boldsymbol {f}} ^ {n ^ {\prime}} & = \boldsymbol {C} _ {b} ^ {n ^ {\prime}} \widetilde {\boldsymbol {f}} ^ {b} = \boldsymbol {C} _ {n} ^ {n ^ {\prime}} \boldsymbol {C} _ {b} ^ {n} \widetilde {\boldsymbol {f}} ^ {b} = \\ & \quad \boldsymbol {C} _ {n} ^ {n ^ {\prime}} \left\{- \boldsymbol {g} ^ {n} + \boldsymbol {f} _ {d} ^ {n} + \nabla^ {n} + \boldsymbol {C} _ {b} ^ {n} (\delta \boldsymbol {K} _ {A} + \delta \boldsymbol {A}) (- \boldsymbol {g} ^ {b} + \boldsymbol {f} _ {d} ^ {b}) \right\} = \\ & \quad \left[ \begin{array}{c} - g \varphi_ {\mathrm{N}} + f _ {d \mathrm{E}} + \nabla_ {\mathrm{E}} + \delta f _ {\mathrm{E}} \\ g \varphi_ {\mathrm{E}} + f _ {d \mathrm{N}} + \nabla_ {\mathrm{N}} + \delta f _ {\mathrm{N}} \\ g + f _ {d \mathrm{U}} + \nabla_ {\mathrm{U}} + \delta f _ {\mathrm{U}} \end{array} \right] \end{array}\tag{8.5.44}
$$

式中

$$
\left[ \begin{array}{l l l} \delta f _ {\mathrm{E}} & \delta f _ {\mathrm{N}} & \delta f _ {\mathrm{U}} \end{array} \right] ^ {\mathrm{T}} = \boldsymbol {C} _ {b} ^ {n} (\delta \boldsymbol {K} _ {A} + \delta \boldsymbol {A}) \boldsymbol {f} _ {d} ^ {b}
$$

为干扰加速度测量误差,而

$$
\left[ \begin{array}{l l l} \nabla_ {E} & \nabla_ {N} & \nabla_ {U} \end{array} \right] ^ {T} = C _ {b} ^ {n} \left\{\boldsymbol {\nabla} ^ {b} + (\delta \boldsymbol {K} _ {A} + \delta \boldsymbol {A}) (- \boldsymbol {g} ^ {b}) \right\}
$$

为等效加速度计偏置。

为方便起见,定义

$$
\left[ \begin{array}{l l l} f _ {D E} & f _ {D N} & f _ {D U} \end{array} \right] ^ {\mathrm{T}} = \left[ \begin{array}{l l l} f _ {d E} & f _ {d N} & f _ {d U} \end{array} \right] ^ {\mathrm{T}} + \left[ \begin{array}{l l l} \delta f _ {E} & \delta f _ {N} & \delta f _ {U} \end{array} \right] ^ {\mathrm{T}}
$$

为加速度计输出中的干扰加速度部分,显然,它与 $f_{d}^{b}$ 同频率且作简谐波动。

由式(8.5.45)得 $\tilde{f}^{n'}$ 的水平分量为

$$
\left\{ \begin{array}{l} f _ {\mathrm{E} ^ {\prime}} = - g \varphi_ {\mathrm{N}} + f _ {D \mathrm{E}} + \nabla_ {\mathrm{E}} \\ f _ {\mathrm{N} ^ {\prime}} = g \varphi_ {\mathrm{E}} + f _ {D \mathrm{N}} + \nabla_ {\mathrm{N}} \end{array} \right.
$$

将式(8.5.41)中的 $\varphi_{E}$ 和 $\varphi_{N}$ 代入上式，并在 $[0, t]$ 内积分，得速度增量：

$$
v _ {\mathrm{E}} (t) = (\nabla_ {\mathrm{E}} - g \varphi_ {\mathrm{N0}}) t - \frac {t ^ {2}}{2} g u _ {\mathrm{N}} + \frac {t ^ {3}}{6} g \omega_ {i e} u _ {\mathrm{E}} \sin L + V _ {D E}
$$

(8.5.45a)

$$
v _ {\mathrm{N}} (t) = (\nabla_ {\mathrm{N}} + g \varphi_ {\mathrm{E0}}) t + \frac {t ^ {2}}{2} g u _ {\mathrm{E}} + \frac {t ^ {3}}{6} g \omega_ {i e} (u _ {\mathrm{N}} \sin L - u _ {\mathrm{U}} \cos L) + V _ {D N}\tag{8.5.45b}
$$

从式(8.5.45a)和式(8.5.45b)可看出：

（1）在 $[0, t]$ 内的速度误差增量中，水平姿态误差初值构成时间的一次方项，包含陀螺等效北向漂移信息的 $u_{\mathrm{N}}$ 及包含方位误差角初值信息的 $u_{\mathrm{E}}$ 构成时间的二次方项，而包含陀螺等效方位漂移信息的 $u_{\mathrm{U}}$ 构成时间的三次方项。由于三次曲线在时间较短时只呈现线性特性，因此，辨识等效北向漂移和方位误差初值所需时间远比辨识水平误差初值的时间长，辨识等效方位陀螺漂移的时间更长。这说明对准时间同测漂和对准精度不能同时兼顾，更不可能无限缩短对准时间。

(2) $V_{\mathrm{DE}}$ 和 $V_{\mathrm{DN}}$ 是由作简谐波动的等效干扰加速度积分获得的, 没有随时间增长的趋势, 而速度增量 $\upsilon_{\mathrm{E}}$ 和 $\upsilon_{\mathrm{N}}$ 是时间 $t$ 的三次函数, 所以信噪比随对准时间的增加而加速提高, 这对抑制晃动干扰提高辨识精度是有益的。

将式 $(8.5.45)$ 改写成

$$
v _ {\mathrm{E}} (k) = a _ {1 \mathrm{E}} k T _ {\mathrm{S}} + a _ {2 \mathrm{E}} (k T _ {\mathrm{S}}) ^ {2} + a _ {3 \mathrm{E}} (k T _ {\mathrm{S}}) ^ {3} + V _ {D \mathrm{E}}\tag{8.5.46a}
$$

$$
v _ {\mathrm{N}} (k) = a _ {1 \mathrm{N}} k T _ {\mathrm{S}} + a _ {2 \mathrm{N}} (k T _ {\mathrm{S}}) ^ {2} + a _ {3 \mathrm{N}} (k T _ {\mathrm{S}}) ^ {3} + V _ {D \mathrm{N}}\tag{8.5.46b}
$$

式中 $T_{\mathrm{s}}$ 为采样周期

$$
\left. \begin{array}{l} a _ {1 \mathrm{E}} = \nabla_ {\mathrm{E}} - g \varphi_ {\mathrm{N0}} \\ a _ {2 \mathrm{E}} = - \frac {1}{2} g u _ {\mathrm{N}} \\ a _ {3 \mathrm{E}} = \frac {1}{6} g \omega_ {i e} u _ {\mathrm{E}} \sin L \\ a _ {1 \mathrm{N}} = \nabla_ {\mathrm{N}} + g \varphi_ {\mathrm{E0}} \\ a _ {2 \mathrm{N}} = \frac {1}{2} g u _ {\mathrm{E}} \\ a _ {3 \mathrm{N}} = \frac {1}{6} g \omega_ {i e} (u _ {\mathrm{N}} \sin L - u _ {\mathrm{U}} \cos L) \end{array} \right\}\tag{8.5.47}
$$

(8.5.48)

它们都是常值参数。根据上述关系式，可采用辨识技术从量测值 $v_{\mathrm{E}}(k)$ 和 $v_{\mathrm{N}}(k)$ 中提取出这些参数。

定义

$$
\pmb {\Theta} _ {\mathrm{E}} = \left[ \begin{array}{l} a _ {1 \mathrm{E}} \\ a _ {2 \mathrm{E}} \\ a _ {3 \mathrm{E}} \end{array} \right], \qquad \pmb {\Theta} _ {\mathrm{N}} = \left[ \begin{array}{l} a _ {1 \mathrm{N}} \\ a _ {2 \mathrm{N}} \\ a _ {3 \mathrm{N}} \end{array} \right]
$$

则由 $\Theta_{\mathrm{E}}$ 和 $\Theta_{\mathrm{N}}$ 构成的系统方程和量测方程为

$$
\left. \begin{array}{l} \Theta_ {\mathrm{E}} (k + 1) = \Theta_ {\mathrm{E}} (k) \\ v _ {\mathrm{E}} (k) = H (k) \Theta_ {\mathrm{E}} (k) + V _ {D \mathrm{E}} (k) \\ \Theta_ {\mathrm{N}} (k + 1) = \Theta_ {\mathrm{N}} (k) \\ v _ {\mathrm{N}} (k) = H (k) \Theta_ {\mathrm{N}} (k) + V _ {D \mathrm{N}} (k) \end{array} \right\}\tag{8.5.49}
$$

(8.5.50)

式中

$$
\mathbf {H} (k) = \left[ k T _ {\mathrm{s}} \quad (k T _ {\mathrm{s}}) ^ {2} \quad (k T _ {\mathrm{s}}) ^ {3} \right]
$$

若 $k$ 时刻参数估计为 $\hat{\Theta}(k)$ ，估计误差的方差阵为 $P(k), k + 1$ 时刻的量测值为 $v(k + 1)$ ，则 $\hat{\Theta}$ 的递推最小二乘算法为

$$
\left. \begin{array}{l} \hat {\boldsymbol {\Theta}} (k + 1) = \hat {\boldsymbol {\Theta}} (k) + \boldsymbol {P} (k + 1) \boldsymbol {H} ^ {\mathrm{T}} (k + 1) [ v (k + 1) - \boldsymbol {H} (k + 1) \hat {\boldsymbol {\Theta}} (k) ] \\ \boldsymbol {P} (k + 1) = \boldsymbol {P} (k) - \Gamma (k + 1) \boldsymbol {P} (k) \boldsymbol {H} ^ {\mathrm{T}} (k + 1) \boldsymbol {H} (k + 1) \boldsymbol {P} (k) \\ \Gamma (k + 1) = [ 1 + \boldsymbol {H} (k + 1) \boldsymbol {P} (k) \boldsymbol {H} ^ {\mathrm{T}} (k + 1) ] ^ {- 1} \end{array} \right\}\tag{8.5.51}
$$

式中， $\hat{\Theta}(0)$ 可任选，一般可选零向量； $P(0)=I_{\alpha}$ ， $I$ 为单位阵， $\alpha$ 为任选的非常大的正实数。

由递推算法求得的参数估值是逐渐向真实值收敛的,所以在递推计算的开始阶段,参数估计值是不能用的,随着递推步数的增加,参数估值中包含的量测值信息越来越多,可用性逐渐提高。这一特点正好与量测值的三次曲线特性相一致,即在递推的开始阶段,量测值仅反映了三次曲线的线性部分,从量测值中不能提取出所有参数,而这段时间正好是递推算法的启动阶段,所以采用递推算法在时间配合上十分合适。

参数辨识的另一种实用算法是改良卡尔曼滤波法 $^{[95]}$ 。对应于系统方程和量测方程

$$
\left\{ \begin{array}{l} \boldsymbol {\Theta} (k + 1) = \boldsymbol {\Theta} (k) \\ \mathbf {Z} (k) = \mathbf {H} (k) \boldsymbol {\Theta} (k) + \mathbf {V} (k) \end{array} \right.
$$

滤波算法如下：

$$
\left. \begin{array}{l} \hat {\boldsymbol {\Theta}} (k + 1) = \hat {\boldsymbol {\Theta}} (k) + \boldsymbol {K} (k) [ \boldsymbol {Z} (k) - \boldsymbol {H} (k) \hat {\boldsymbol {\Theta}} (k) ] \\ \hat {\boldsymbol {\Theta}} (0) = \boldsymbol {\Theta} _ {0} \\ \boldsymbol {K} (k) = \boldsymbol {P} (k) \boldsymbol {H} ^ {\mathrm{T}} (k) [ \boldsymbol {H} (k) \boldsymbol {P} (k) \boldsymbol {H} ^ {\mathrm{T}} (k) + \hat {\boldsymbol {\Lambda}} (k + 1) ] ^ {- 1} \\ \boldsymbol {P} (k + 1) = \boldsymbol {P} (k) - \boldsymbol {K} (k) [ \boldsymbol {H} (k) \boldsymbol {P} (k) \boldsymbol {H} ^ {\mathrm{T}} (k) + \hat {\boldsymbol {\Lambda}} (k + 1) ] \boldsymbol {K} ^ {\mathrm{T}} (k) \\ \boldsymbol {P} (0) = \boldsymbol {P} _ {0} \\ \hat {\boldsymbol {\Lambda}} (k + 1) = \hat {\boldsymbol {\Lambda}} (k) + \frac {\boldsymbol {e} (k) \boldsymbol {e} ^ {\mathrm{T}} (k) - \hat {\boldsymbol {\Lambda}} (k)}{k + 1} \\ \hat {\boldsymbol {\Lambda}} (0) = \boldsymbol {\Lambda} _ {0} \\ \boldsymbol {e} (k) = \boldsymbol {Z} (k) - \boldsymbol {H} (k) \hat {\boldsymbol {\Theta}} (k) \end{array} \right\}\tag{8.5.52}
$$

式中 $\Theta_{o}, P_{0}, \Lambda_{0}$ 均可任选。

上述算法中,量测噪声并未直接使用,而将它合并到量测的估计误差 $e(k)$ 中。由于 $e(k)$ 每步递推计算中均可求得,所以该法无须知道量测噪声的统计特性,这对简化计算十分有利。另外,滤波器增益利用了状态的估计均方误差和量测值的估计均方误差作自适应加权处理,加速了算法的收敛。

由递推算法辨识得 $a_{ij}(i=1,2,3; j=\mathrm{E},\mathrm{N})$ 后，根据式(8.5.47)和式(8.5.48)可求得 $\varphi_{E0}, \varphi_{N0}, u_{E}, u_{N}, u_{U}$ ，并由式(8.5.42)，得

$$
\varphi_ {\mathrm{U0}} = \varphi_ {\mathrm{N0}} \tan L - \frac {u _ {\mathrm{E}} + \varepsilon_ {\mathrm{E}}}{\omega_ {i e} \cos L}\tag{8.5.53}
$$

将上述求得的参数代入式(8.5.41)，即可求得姿态误差角。在辨识稳定后，所求得的姿态误差角代入式(8.5.39)，即可完成对准。

## 3. 双位置对准及测漂计算

由式(8.5.53)知,方位误差与等效东向陀螺漂移有关,该漂移可由双位置对准测出。方法是:将惯性元件安装在可绕机体轴 $z_{b}$ 转动的圆盘上,相对惯性器件的正常工作位置圆盘转过 $90^{\circ}$ 作为第I位置,由测量轴确定的坐标系记作 $b'$ ,在该位置上完成精对准,以求出等效北向漂移;再将圆盘转回到正常工作位置,作为第II位置,由测量轴确定的坐标系即为机体坐标系b,在该位置上完成精对准和所有陀螺的测漂。显然第II位置是基准,第I位置相对第II位置的转角为 $90^{\circ}+\delta\gamma,\delta\gamma$ 为转角误差。且有

$$
\boldsymbol {C} _ {b} ^ {b ^ {\prime}} = \left[ \begin{array}{c c c} - \sin \delta \gamma & \cos \delta \gamma & 0 \\ - \cos \delta \gamma & - \sin \delta \gamma & 0 \\ 0 & 0 & 1 \end{array} \right]\tag{8.5.54}
$$

由于陀螺固定在圆盘上,所以位置Ⅰ和位置Ⅱ上的陀螺漂移在 $b'$ 坐标内和b坐标系内的分量相同,即

$$
\boldsymbol {\varepsilon} _ {I} ^ {b ^ {\prime}} = \boldsymbol {\varepsilon} _ {I I} ^ {b} = \left[ \begin{array}{l l l} \varepsilon_ {x} & \varepsilon_ {y} & \varepsilon_ {z} \end{array} \right] ^ {T}\tag{8.5.55}
$$

在第Ⅰ位置上，陀螺在导航坐标系n内的等效漂移为

$$
\boldsymbol {\varepsilon} _ {1} ^ {n} = \boldsymbol {C} _ {b} ^ {n} \boldsymbol {C} _ {b ^ {\prime}} ^ {b} \boldsymbol {\varepsilon} _ {1} ^ {b ^ {\prime}} = \boldsymbol {C} _ {b} ^ {n} \left[ \begin{array}{c c c} - \sin \delta \gamma & - \cos \delta \gamma & 0 \\ \cos \delta \gamma & - \sin \delta \gamma & 0 \\ 0 & 0 & 1 \end{array} \right] \quad \left[ \begin{array}{l} \varepsilon_ {x} \\ \varepsilon_ {y} \\ \varepsilon_ {z} \end{array} \right]
$$

考虑到 $\delta \gamma$ 在 $1^{\circ}$ 左右， $\delta \gamma$ 和 $\varepsilon_{x}, \varepsilon_{y}, \varepsilon_{z}$ 均可看做小量，所以

$$
\boldsymbol {\varepsilon} _ {1} ^ {n} = \boldsymbol {C} _ {b} ^ {n} \left[ \begin{array}{c} - \varepsilon_ {y} \\ \varepsilon_ {x} \\ \varepsilon_ {z} \end{array} \right]\tag{8.5.56}
$$

在第Ⅱ位置上，陀螺在导航坐标系n内的等效漂移为

$$
\boldsymbol {\varepsilon} _ {\text {II}} ^ {n} = \boldsymbol {C} _ {b} ^ {n} \boldsymbol {\varepsilon} _ {\text {II}} ^ {b} = \boldsymbol {C} _ {b} ^ {n} \left[ \begin{array}{l} \varepsilon_ {x} \\ \varepsilon_ {y} \\ \varepsilon_ {z} \end{array} \right]\tag{8.5.57}
$$

初始对准时飞机停在停机坪上接近水平,所以有

$$
\boldsymbol {C} _ {b} ^ {n} \approx \left[ \begin{array}{c c c} \cos \Psi & - \sin \Psi & 0 \\ \sin \Psi & \cos \Psi & 0 \\ 0 & 0 & 1 \end{array} \right]\tag{8.5.58}
$$

式中 $\Psi$ 为飞机的航向角。

将式 $(8.5.58)$ 代入式 $(8.5.56)$ 和式 $(8.5.57)$ 得

$$
\boldsymbol {\varepsilon} _ {1} ^ {n} = \left[ \begin{array}{c} - (\varepsilon_ {x} \sin \Psi + \varepsilon_ {y} \cos \Psi) \\ \varepsilon_ {x} \cos \Psi - \varepsilon_ {y} \sin \Psi \\ \varepsilon_ {z} \end{array} \right]\tag{8.5.59}
$$

$$
\boldsymbol {\varepsilon} _ {\parallel} ^ {n} = \left[ \begin{array}{c} \varepsilon_ {x} \cos \Psi - \varepsilon_ {y} \sin \Psi \\ \varepsilon_ {x} \sin \Psi + \varepsilon_ {y} \cos \Psi \\ \varepsilon_ {z} \end{array} \right]\tag{8.5.60}
$$

比较上式(8.5.59)和式(8.5.60)，得

$$
\varepsilon_ {\mathrm{E}} ^ {\parallel} = \varepsilon_ {\mathrm{N}} ^ {1}\tag{8.5.61}
$$

即，第Ⅱ位置的等效东向漂移可由第Ⅰ位置对准中测得的等效北向漂移求得。

由式(8.5.57)

$$
\left[ \begin{array}{l} \varepsilon_ {x} \\ \varepsilon_ {y} \\ \varepsilon_ {z} \end{array} \right] = C _ {n} ^ {b} \left[ \begin{array}{l} \varepsilon_ {\mathrm{E}} ^ {\text {II}} \\ \varepsilon_ {\mathrm{N}} ^ {\text {II}} \\ \varepsilon_ {\mathrm{U}} ^ {\text {II}} \end{array} \right] = C _ {n} ^ {b} \left[ \begin{array}{l} \varepsilon_ {\mathrm{N}} ^ {\text {I}} \\ \varepsilon_ {\mathrm{N}} ^ {\text {II}} \\ \varepsilon_ {\mathrm{U}} ^ {\text {II}} \end{array} \right]\tag{8.5.62}
$$

由于测漂在对准结束时进行,此时 $C_{n}^{b}$ 已达到对准精度,所以测漂计算可按下式进行:

$$
\left[ \begin{array}{l} \hat {\varepsilon} _ {x} \\ \hat {\varepsilon} _ {y} \\ \hat {\varepsilon} _ {z} \end{array} \right] = \boldsymbol {C} _ {n ^ {\prime}} ^ {b} \left[ \begin{array}{l} \varepsilon_ {N} ^ {I} \\ \varepsilon_ {N} ^ {I I} \\ \varepsilon_ {U} ^ {I I} \end{array} \right]\tag{8.5.63}
$$

使用上式进行测漂的前提是第Ⅱ位置对准时间足够长， $\varepsilon_{U}^{II}$ 被较精确地测出。如果对准的允许时间较短， $\varepsilon_{U}^{II}$ 还来不及测出，则只要飞机接近水平， $\varepsilon_{x}$ 和 $\varepsilon_{y}$ 可按下述方法近似计算出。

当飞机接近水平时

$$
\boldsymbol {C} _ {n} ^ {b} \approx \left[ \begin{array}{l l l} c _ {1 1} & c _ {1 2} & 0 \\ c _ {2 1} & c _ {2 2} & 0 \\ c _ {3 1} & c _ {3 2} & c _ {3 3} \end{array} \right]
$$

所以由式(8.5.63)

$$
\left. \begin{array}{l} \hat {\varepsilon} _ {x} \approx C _ {n ^ {\prime}} ^ {b} (1, 1) \varepsilon_ {\mathrm{N}} ^ {\mathrm{I}} + C _ {n ^ {\prime}} ^ {b} (1, 2) \varepsilon_ {\mathrm{N}} ^ {\mathrm{II}} \\ \hat {\varepsilon} _ {y} \approx C _ {n ^ {\prime}} ^ {b} (2, 1) \varepsilon_ {\mathrm{N}} ^ {\mathrm{I}} + C _ {n ^ {\prime}} ^ {b} (2, 2) \varepsilon_ {\mathrm{N}} ^ {\mathrm{II}} \end{array} \right\}\tag{8.5.64}
$$

对辨识对准的有效性作了仿真分析,仿真中环境条件和元器件精度假设如下:

(1) 飞机绕机体轴作简谐晃动, 晃动角为

$$
\alpha_ {j} = A _ {j} \sin (\omega_ {d j} + p _ {j}) \quad j = x, y, z
$$

式中， $A_{x} = A_{y} = 6^{\prime}$ ， $A_{z} = 3^{\prime}$ ， $p_x = p_y = 0^\circ$ ， $p_z = 60^\circ$ ， $\omega_{dj} = 2\pi$ rad/s

杆臂长 $R_{x}=R_{y}=R_{z}=1.5\ m$ 。

(2) 飞机俯仰角和倾斜角均为 $5^{\circ}$ ，方位角为 $15^{\circ}$ ，纬度为 $33^{\circ}$ 。

（3）陀螺随机常值漂移 $\varepsilon_{x} = \varepsilon_{y} = 0.1^{\circ} / \mathrm{h},\varepsilon_{z} = 0.5^{\circ} / \mathrm{h}$ ，刻度系数误差为 $10^{-4}$ ，加速度计偏置为 $5\times 10^{-4}\mathrm{g}$ ，刻度系数误差为 $2.5\times 10^{-4}$ ，惯性器件安装误差均为 $10^{\prime \prime}$ 。

模拟中比较计算了6种预滤波处理后的对准结果：

(1) 末作预滤波处理；

(2) 阻带为 $0.9 \sim 1.1 \, Hz$ 的数字带阻；

(3) 阻带为 $0.5 \sim 1.5 \, Hz$ 的数字带阻；

(4) 阻带为 $0.5 \sim 2.5 \, Hz$ 的数字带阻；

(5) 截止频率为 0.5 Hz 的数字低通；

(6) 截止频率为 0.25 Hz 的数字低通。

表 8.5.2 列出了各预滤波情况对准和测漂达到稳定所需的时间(在该时刻以后被检查量的相对变化量小于1%)。表 8.5.3 列出了对准结束时的对准误差和测漂结果。表中，Ⅰ和Ⅱ分别表示第Ⅰ位置和第Ⅱ位置对准。

表 8.5.2 采用不同数字预滤波时对准误差量和测漂量达到 99% 相对稳定度所需时间

<table><tr><td rowspan="2" colspan="2"> $\begin{array}{c} B \\ A\end{array}$ </td><td rowspan="2">无</td><td colspan="3">数字带阻 /Hz</td><td colspan="2">数字低通 /Hz</td></tr><tr><td>0.9~1.1</td><td>0.5~1.5</td><td>0.5~2.5</td><td>0.5</td><td>0.25</td></tr><tr><td rowspan="2">方位</td><td>I</td><td>600</td><td>480</td><td>340</td><td>420</td><td>560</td><td>820</td></tr><tr><td>II</td><td>1 120</td><td>700</td><td>580</td><td>600</td><td>760</td><td>840</td></tr><tr><td rowspan="2">俯仰</td><td>I</td><td>200</td><td>180</td><td>140</td><td>140</td><td>200</td><td>360</td></tr><tr><td>II</td><td>200</td><td>180</td><td>200</td><td>220</td><td>240</td><td>280</td></tr><tr><td rowspan="2">倾斜</td><td>I</td><td>260</td><td>220</td><td>180</td><td>200</td><td>280</td><td>1 000</td></tr><tr><td>II</td><td>300</td><td>240</td><td>360</td><td>360</td><td>440</td><td>560</td></tr><tr><td> $\hat{\varepsilon}_{N}$ </td><td>I</td><td>700</td><td>560</td><td>460</td><td>540</td><td>740</td><td>&gt;1 200</td></tr><tr><td> $\hat{\varepsilon}_{U}$ </td><td>II</td><td>&gt;1 200</td><td>1 140</td><td>800</td><td>1 040</td><td>1 200</td><td>&gt;1 200</td></tr><tr><td> $\hat{\varepsilon}_{x}$ </td><td>II</td><td>1 040</td><td>760</td><td>720</td><td>780</td><td>1 000</td><td>&gt;1 200</td></tr><tr><td> $\hat{\varepsilon}_{y}$ </td><td>II</td><td>1 060</td><td>740</td><td>760</td><td>820</td><td>1 020</td><td>&gt;1 200</td></tr><tr><td> $\hat{\varepsilon}_{z}$ </td><td>II</td><td>&gt;1 200</td><td>840</td><td>880</td><td>940</td><td>1 140</td><td>&gt;1 200</td></tr></table>

注：A——对准误差量及测漂值；  
B——各量分别达到99%相对稳定度所需时间(s);  
C——数字预滤波器采用情况。

表 8.5.3 对准完成时的对准误差和测漂结果

<table><tr><td rowspan="2"> $B$  $A$ </td><td rowspan="2">C</td><td rowspan="2">无</td><td colspan="3">数字带阻 /Hz</td><td colspan="2">数字低通 /Hz</td></tr><tr><td>0.9~1.1</td><td>0.5~1.5</td><td>0.5~2.5</td><td>0.5</td><td>0.25</td></tr><tr><td rowspan="2">方位/(&#x27;)</td><td>I</td><td>49.875</td><td>50.817</td><td>51.238</td><td>51.414</td><td>51.914</td><td>56.863</td></tr><tr><td>II</td><td>-4.630</td><td>-6.089</td><td>-6.782</td><td>-6.943</td><td>-7.790</td><td>-20.741</td></tr><tr><td rowspan="2">俯仰/(&#x27;)</td><td>I</td><td>-6.433</td><td>-6.418</td><td>-6.412</td><td>-6.410</td><td>-6.404</td><td>-6.151</td></tr><tr><td>II</td><td>2.064</td><td>2.040</td><td>2.041</td><td>2.042</td><td>2.046</td><td>2.064</td></tr><tr><td rowspan="2">倾斜/(&#x27;)</td><td>I</td><td>3.878</td><td>3.847</td><td>3.831</td><td>3.826</td><td>3.806</td><td>3.198</td></tr><tr><td>II</td><td>-1.428</td><td>-1.411</td><td>-1.432</td><td>-1.442</td><td>-1.473</td><td>-1.656</td></tr><tr><td> $\hat{\varepsilon}_{\mathrm{N}}/(^{\circ})\cdot h^{-1}$ </td><td>I</td><td>-0.075</td><td>-0.072</td><td>-0.071</td><td>-0.070</td><td>-0.068</td><td>-0.028</td></tr><tr><td> $\hat{\varepsilon}_{\mathrm{U}}/(^{\circ})\cdot h^{-1}$ </td><td>II</td><td>0.661</td><td>0.558</td><td>0.496</td><td>0.475</td><td>0.400</td><td>-0.102</td></tr><tr><td> $\hat{\varepsilon}_{x}/(^{\circ})\cdot h^{-1}$ </td><td>II</td><td>0.096</td><td>0.086</td><td>0.084</td><td>0.084</td><td>0.081</td><td>0.064</td></tr><tr><td> $\hat{\varepsilon}_{y}/(^{\circ})\cdot h^{-1}$ </td><td>II</td><td>0.074</td><td>0.079</td><td>0.079</td><td>0.079</td><td>0.080</td><td>0.052</td></tr><tr><td> $\hat{\varepsilon}_{z}/(^{\circ})\cdot h^{-1}$ </td><td>II</td><td>0.448</td><td>0.530</td><td>0.566</td><td>0.569</td><td>0.616</td><td>0.798</td></tr></table>

注：A，C说明同表8.5.2；  
B——位置Ⅰ和位置Ⅱ对准结束时的对准误差及测漂结果。  
从表 8.5.2 和表 8.5.3 可看出: 参数辨识法对准算法本身具有对晃动干扰的滤波作用。

预滤波处理能加速辨识算法的收敛,缩短对准时间,但会损害对准精度,预滤波器的阻带越宽,低端截止频率越低,损害就越严重。由于机体晃动是干扰力短暂作用后机体产生的自由振荡,振荡频率是机体的固有频率,是具体的确定值,所以宜采用带阻滤波器,阻带应包含机体的固有频率,且阻带应窄些。关于预滤波器设计等问题,读者可参阅文献[105]。

## 8.5.4 捷联惯导系统在运动基座上的传递对准

机载导弹发射前，弹载惯导系统的初始对准一般由传递对准来完成。根据所用量测信息的不同，可得传递对准不同的匹配方案，文献[96]对此作了综述。由于空战形势瞬息万变，稍纵即逝，适用于机载战术导弹的传递对准必须满足两个基本要求：快，准。目前的机载导航系统是由惯导系统和GPS为主要子系统构成的组合导航系统，机载主惯导已得到较好校正，如何合理利用主惯导的高精度导航信息是实现既快又准传递对准的关键。

传递对准分两步完成:第一步是装订,也就是将机载主惯导输出的导航信息(姿态阵或四元数,速度及位置)通过火控系统传送给弹载子惯导,使子惯导获得初始值并开始作导航解算;第二步是匹配,即以主惯导的导航输出作为参考信息,构造出量测量,由卡尔曼滤波器估计出子惯导的姿态误差等导航误差,待滤波达到稳态后,用这些误差估计值对子惯导作修正。由于惯导精度的短期稳定性好,GPS精度的短期稳定性差,而传递对准在短时间内完成,所以在匹配过程中,应该断开GPS对主惯导的修正,使主惯导处于纯惯性工作状态,以避免GPS噪声对传递对准精度的影响。

传递对准中可资利用的主惯导信息有:位置、速度、姿态、角速度及加速度。由于平台误差须经一定时间后才能反映到位置误差上,所以位置匹配时间很长,不适于快速传递对准;在加速度匹配方案中,杆臂效应难以彻底补偿,残余误差被直接引入量测量中,直接影响对准精度。因此,此处仅对速度、角速度和姿态信息匹配方案作探讨。具体介绍的内容为:主、子惯导误差模型的建立,机翼挠曲角变形模型的建立,各种匹配方案中量测量的获取,各匹配方案的定性分析。

## 1. 主惯导误差模型

设主惯导为激光陀螺捷联惯导系统，与 GPS 构成组合导航系统。取地理坐标系为导航坐标系。由于 GPS 的长时间稳定性和高精度定位能力，惯导误差受到强烈抑制，但滤波并不彻底，所以导航误差在时间和空间上相关。为了减少传递对准卡尔曼滤波器的维数，忽略空间上的相关，速度误差和姿态误差简化成相互独立的一阶马尔柯夫过程

$$
\delta \dot {v} _ {m i} = - \frac {1}{\tau_ {m v i}} \delta v _ {m i} + w _ {m v i} \qquad i = \mathrm{E}, \mathrm{N}, \mathrm{U}\tag{8.5.65}
$$

$$
\dot {\varphi} _ {m i} = - \frac {1}{\tau_ {m \varphi i}} \varphi_ {m i} + w _ {m \varphi i} \qquad i = \mathrm{E}, \mathrm{N}, \mathrm{U}\tag{8.5.66}
$$

## 2. 子惯导误差模型

设飞机的航向角、俯仰角及横滚角分别为 $\Psi$ 、 $\theta$ 和 $\gamma$ ，则姿态阵为

$$
\pmb {C} _ {b} ^ {n} = \left[ \begin{array}{l l l} T _ {1 1} & T _ {1 2} & T _ {1 3} \\ T _ {2 1} & T _ {2 2} & T _ {2 3} \\ T _ {3 1} & T _ {3 2} & T _ {3 3} \end{array} \right] =
$$

$$
\left[ \begin{array}{c c c} \cos \gamma \cos \Psi - \sin \gamma \sin \theta \sin \Psi & - \cos \theta \sin \Psi & \sin \gamma \cos \Psi + \cos \gamma \sin \theta \sin \Psi \\ \cos \gamma \sin \Psi + \sin \gamma \sin \theta \cos \Psi & \cos \theta \cos \Psi & \sin \gamma \sin \Psi - \cos \gamma \sin \theta \cos \Psi \\ - \sin \gamma \cos \theta & \sin \theta & \cos \gamma \cos \theta \end{array} \right]\tag{8.5.67}
$$

式中 n 为导航坐标系(东、北、天)，b 为机体坐标系(右、前、上)。

由式 $(8.5.67)$ 式得

1. 2. 3.

$$
\left. \begin{array}{l} \tan \Psi = - \frac {T _ {1 2}}{T _ {2 2}} \\ \tan \gamma = - \frac {T _ {3 1}}{T _ {3 3}} \\ \sin \theta = T _ {3 2} \end{array} \right\}\tag{8.5.68}
$$

设弹体坐标系 $b'$ 沿弹体的右、前、上，子惯导经四元数更新建立的姿态阵为 $C_{b'}^{i}$ ，其中 $n'$ 为子惯导数学平台确定的导航坐标系，它相对导航坐标系 n（地理坐标系）存在误差角向量

$$
\boldsymbol {\varphi} = \left[ \begin{array}{l l l} \varphi_ {\mathrm{E}} & \varphi_ {\mathrm{N}} & \varphi_ {\mathrm{U}} \end{array} \right] ^ {\mathrm{T}}
$$

所以

$$
\boldsymbol {C} _ {n} ^ {n ^ {\prime}} = \boldsymbol {I} - \boldsymbol {E}\tag{8.5.69}
$$

式中， $I$ 为单位阵

$$
\boldsymbol {E} = \left[ \begin{array}{c c c} 0 & - \varphi_ {\mathrm{U}} & \varphi_ {\mathrm{N}} \\ \varphi_ {\mathrm{U}} & 0 & - \varphi_ {\mathrm{E}} \\ - \varphi_ {\mathrm{N}} & \varphi_ {\mathrm{E}} & 0 \end{array} \right]\tag{8.5.70}
$$

如果能求得 $\varphi$ ，则数学平台确定如下

$$
\boldsymbol {C} _ {b ^ {\prime}} ^ {n} = (\boldsymbol {I} + \boldsymbol {E}) \boldsymbol {C} _ {b ^ {\prime}} ^ {n ^ {\prime}}\tag{8.5.71}
$$

由式(8.4.22)， $\varphi$ 的变化规律满足

$$
\left. \begin{array}{l} \dot {\varphi} _ {\mathrm{E}} = \left(\omega_ {i e} \sin L + \frac {v _ {\mathrm{E}}}{R + h} \tan L\right) \varphi_ {\mathrm{N}} - \left(\omega_ {i e} \cos L + \frac {v _ {\mathrm{E}}}{R + h}\right) \varphi_ {\mathrm{U}} - \frac {\delta v _ {\mathrm{N}}}{R + h} + \varepsilon_ {\mathrm{E}} + w _ {\varepsilon \mathrm{E}} \\ \dot {\varphi} _ {\mathrm{N}} = - \left(\omega_ {i e} \sin L + \frac {v _ {\mathrm{E}}}{R + h} \tan L\right) \varphi_ {\mathrm{E}} - \frac {v _ {\mathrm{N}}}{R + h} \varphi_ {\mathrm{U}} + \frac {\delta v _ {\mathrm{E}}}{R + h} + \varepsilon_ {\mathrm{N}} + w _ {\varepsilon \mathrm{N}} \\ \dot {\varphi} _ {\mathrm{U}} = \left(\omega_ {i e} \cos L + \frac {v _ {\mathrm{E}}}{R + h}\right) \varphi_ {\mathrm{E}} + \frac {v _ {\mathrm{N}}}{R + h} \varphi_ {\mathrm{N}} + \frac {\delta v _ {\mathrm{E}}}{R + h} \tan L + \varepsilon_ {\mathrm{U}} + w _ {\varepsilon \mathrm{U}} \end{array} \right\}\tag{8.5.72}
$$

式中，h 和 L 为飞机的飞行高度和纬度， $v_{E}$ 和 $v_{N}$ 为飞机的东向和北向速度，这些量均由主惯导提供； $\varepsilon_{i}$ 和 $w_{\varepsilon i}(i=\mathrm{E},\mathrm{N},\mathrm{U})$ 分别为子惯导陀螺的等效漂移的相关量和不相关量。由于对准时间很短，所以相关量可近似视为随机常数

$$
\dot {\varepsilon} _ {i} = 0 \qquad i = \mathrm{E}, \mathrm{N}, \mathrm{U}\tag{8.5.73}
$$

由于对准中子惯导所用的导航参数 $(L,h,v_{\mathrm{E}},v_{\mathrm{N}}$ 等）来自与GPS组合后的主惯导，误差均可忽略，所以子惯导的速度误差方程为

$$
\left. \begin{array}{l} \delta \dot {v} _ {\mathrm{E}} = - f _ {\mathrm{U}} \varphi_ {\mathrm{N}} + f _ {\mathrm{N}} \varphi_ {\mathrm{U}} + \nabla_ {\mathrm{E}} + w _ {\nabla \mathrm{E}} \\ \delta \dot {v} _ {\mathrm{N}} = f _ {\mathrm{U}} \varphi_ {\mathrm{E}} - f _ {\mathrm{E}} \varphi_ {\mathrm{U}} + \nabla_ {\mathrm{N}} + w _ {\nabla \mathrm{N}} \\ \delta \dot {v} _ {\mathrm{U}} = - f _ {\mathrm{N}} \varphi_ {\mathrm{E}} + f _ {\mathrm{E}} \varphi_ {\mathrm{N}} + \nabla_ {\mathrm{U}} + w _ {\nabla \mathrm{u}} \end{array} \right\}\tag{8.5.74}
$$

式中 $w_{\nabla i}$ 为加速度计的白噪声误差， $\nabla_{i}$ 为等效偏置

$$
\dot {\nabla} _ {i} = 0 \qquad i = \mathrm{E}, \mathrm{N}, \mathrm{U}\tag{8.5.75}
$$

$f_{E}$ ， $f_{N}$ ， $f_{U}$ 为比力，由主惯导经运算后提供。

## 3. 机翼挠曲变形引起的角运动模型

机载导弹常吊挂在机翼下，由于空气动力及振动的作用，机翼相对机体产生相对角运动，而描述这种角运动的模型至少为二阶 $^{[96]}$ 。为了尽量减少传递对准滤波器的阶数，此处取二阶模型。设挠曲变形引起的弹体坐标系相对机体坐标系沿机体轴方向的角变形为 $\lambda_{i}$ ，相应角速度为 $\omega_{\lambda_{i}}$ ，则二阶模型为

$$
\left. \begin{array}{l} \dot {\lambda} _ {i} = \omega_ {\lambda i} \\ \dot {\omega} _ {\lambda i} = - \beta^ {2} \lambda_ {i} - 2 \beta \omega_ {\lambda i} + w _ {\lambda i} \quad i = x, y, z \end{array} \right\}\tag{8.5.76}
$$

## 4. 传递对准中的匹配量

传递对准中的匹配量即为卡尔曼滤波器中的量测量。

（1）速度匹配量。设主惯导的速度输出为 $v_{mE}^{c}$ ， $v_{mN}^{c}$ ， $v_{mU}^{c}$ ，子惯导的速度输出为 $v_{E}^{c}$ ， $v_{N}^{c}$ ， $v_{U}^{c}$ ，则速度匹配量为

$$
\left. \begin{array}{l} Z _ {1} = v _ {\mathrm{E}} ^ {c} - v _ {m \mathrm{E}} ^ {c} = \delta v _ {\mathrm{E}} - \delta v _ {m \mathrm{E}} + V _ {1} \\ Z _ {2} = v _ {\mathrm{N}} ^ {c} - v _ {m \mathrm{N}} ^ {c} = \delta v _ {\mathrm{N}} - \delta v _ {m \mathrm{N}} + V _ {2} \\ Z _ {3} = v _ {\mathrm{U}} ^ {c} - v _ {m \mathrm{U}} ^ {c} = \delta v _ {\mathrm{U}} - \delta v _ {m \mathrm{U}} + V _ {3} \end{array} \right\}\tag{8.5.77}
$$

（2）角速度匹配量。设飞机的角速度为 $\omega^{b}=\left[\omega_{x}\quad\omega_{y}\quad\omega_{z}\right]^{T}$ 。假设飞机设计中确保 $b'$ 与 b 相一致，由机翼的挠曲变形引起的 $b'$ 相对 b 的角运动向量为

$$
\boldsymbol {\lambda} ^ {b ^ {\prime}} = \boldsymbol {\lambda} ^ {b} = \left[ \begin{array}{l l l} \lambda_ {x} & \lambda_ {y} & \lambda_ {z} \end{array} \right] ^ {\mathrm{T}}
$$

设主惯导陀螺组的角速度测量值为 $\omega_{m}^{b}$ ，假设主惯导陀螺的安装误差、刻度系数误差等都已得到很好校正，并且角速度匹配中飞机须作姿态机动，陀螺漂移在角速度输出中所占的比例完全可以忽略，所以

$$
\boldsymbol {\omega} _ {m} ^ {b} = \boldsymbol {\omega} ^ {b}
$$

设子惯导陀螺组的角速度测量值为 $\omega_{s}^{b'}$ ，显然

$$
\boldsymbol {\omega} _ {s} ^ {b ^ {\prime}} = \boldsymbol {\omega} ^ {b ^ {\prime}} + \boldsymbol {\omega} _ {\lambda} ^ {b ^ {\prime}} + \boldsymbol {\varepsilon} ^ {b ^ {\prime}}
$$

量测量是主、子惯导陀螺组的角速度输出量之间的差值，即

$$
\mathbf {Z} = \boldsymbol {\omega} _ {s} ^ {b ^ {\prime}} - \boldsymbol {\omega} _ {m} ^ {b} = C _ {b} ^ {b ^ {\prime}} \boldsymbol {\omega} ^ {b} + \boldsymbol {\omega} _ {\lambda} ^ {b ^ {\prime}} + \varepsilon^ {b ^ {\prime}} - \boldsymbol {\omega} ^ {b} = - \boldsymbol {\Lambda} \boldsymbol {\omega} ^ {b} + \boldsymbol {\omega} _ {\lambda} ^ {b ^ {\prime}} + \varepsilon^ {b ^ {\prime}}
$$

式中

$$
\boldsymbol {\Lambda} = \left[ \begin{array}{c c c} 0 & - \lambda_ {z} & \lambda_ {y} \\ \lambda_ {z} & 0 & - \lambda_ {x} \\ - \lambda_ {y} & \lambda_ {x} & 0 \end{array} \right]\tag{8.5.78}
$$

展开式(8.5.78)，得角速度匹配量为

$$
\left. \begin{array}{l} Z _ {1} = \omega_ {y} \lambda_ {z} - \omega_ {z} \lambda_ {y} + \omega_ {\lambda x} + \varepsilon_ {x} + V _ {1} \\ Z _ {2} = - \omega_ {x} \lambda_ {z} + \omega_ {z} \lambda_ {x} + \omega_ {\lambda y} + \varepsilon_ {y} + V _ {2} \\ Z _ {3} = \omega_ {x} \lambda_ {y} - \omega_ {y} \lambda_ {x} + \omega_ {\lambda z} + \varepsilon_ {z} + V _ {3} \end{array} \right\}\tag{8.5.79}
$$

获得 $\hat{\lambda}_{x}, \hat{\lambda}_{y}, \hat{\lambda}_{z}$ 后，子惯导的姿态阵按下式确定：

$$
\boldsymbol {C} _ {b ^ {\prime}} ^ {n} = \boldsymbol {C} _ {b} ^ {n} \boldsymbol {C} _ {b ^ {\prime}} ^ {b} = \boldsymbol {C} _ {b} ^ {n} (\boldsymbol {I} + \boldsymbol {\Lambda})\tag{8.5.80}
$$

(3) 姿态角匹配量。设子惯导和主惯导确定的姿态阵分别为 $C_{b}^{n'}$ 和 $C_{b}^{n}$ ，记

$$
\boldsymbol {C} _ {b ^ {\prime}} ^ {n ^ {\prime}} = \left[ \begin{array}{l l l} T _ {1 1} ^ {\prime} & T _ {1 2} ^ {\prime} & T _ {1 3} ^ {\prime} \\ T _ {2 1} ^ {\prime} & T _ {2 2} ^ {\prime} & T _ {2 3} ^ {\prime} \\ T _ {3 1} ^ {\prime} & T _ {3 2} ^ {\prime} & T _ {3 3} ^ {\prime} \end{array} \right]
$$

由子惯导确定的航向、俯仰及横滚角分别为

$$
\Psi_ {s} = \Psi + \delta \Psi
$$

$$
\theta_ {s} = \theta + \delta \theta
$$

$$
\gamma_ {s} = \gamma + \delta \gamma
$$

其中 $\delta\Psi,\delta\theta,\delta\gamma$ 为相应误差。

而

$$
\boldsymbol {C} _ {b ^ {\prime}} ^ {n ^ {\prime}} = \boldsymbol {C} _ {n} ^ {n ^ {\prime}} \boldsymbol {C} _ {b} ^ {n} \boldsymbol {C} _ {b ^ {\prime}} ^ {b} = (\boldsymbol {I} - \boldsymbol {E}) \boldsymbol {C} _ {b} ^ {n} (\boldsymbol {I} + \boldsymbol {\Lambda}) = \boldsymbol {C} _ {b} ^ {n} - \boldsymbol {E} \boldsymbol {C} _ {b} ^ {n} + \boldsymbol {C} _ {b} ^ {n} \boldsymbol {\Lambda}
$$

将式 $(8.5.38)$ 、式 $(8.5.78)$ 代入上式，得

$$
\begin{array}{l} T _ {1 2} ^ {\prime} = T _ {1 2} + T _ {2 2} \varphi_ {\mathrm{U}} - T _ {3 2} \varphi_ {\mathrm{N}} - T _ {1 1} \lambda_ {z} + T _ {1 3} \lambda_ {x} \\ T _ {2 2} ^ {\prime} = T _ {2 2} - T _ {1 2} \varphi_ {\mathrm{U}} + T _ {3 2} \varphi_ {\mathrm{E}} - T _ {2 1} \lambda_ {z} + T _ {2 3} \lambda_ {x} \\ T _ {3 1} ^ {\prime} = T _ {3 1} + T _ {1 1} \varphi_ {\mathrm{N}} - T _ {2 1} \varphi_ {\mathrm{E}} + T _ {3 2} \lambda_ {z} - T _ {3 3} \lambda_ {y} \\ T _ {3 3} ^ {\prime} = T _ {3 3} + T _ {1 3} \varphi_ {\mathrm{N}} - T _ {2 3} \varphi_ {\mathrm{E}} + T _ {3 1} \lambda_ {y} - T _ {3 2} \lambda_ {x} \\ T _ {3 2} ^ {\prime} = T _ {3 2} + T _ {1 2} \varphi_ {\mathrm{N}} - T _ {2 2} \varphi_ {\mathrm{E}} - T _ {3 1} \lambda_ {z} + T _ {3 3} \lambda_ {x} \end{array}
$$

$$
\tan (\Psi + \delta \Psi) = - \frac {T _ {1 2} ^ {\prime}}{T _ {2 2} ^ {\prime}} = - \frac {T _ {1 2} + T _ {2 2} \varphi_ {\mathrm{U}} - T _ {3 2} \varphi_ {\mathrm{N}} - T _ {1 1} \lambda_ {z} + T _ {1 3} \lambda_ {x}}{T _ {2 2} - T _ {1 2} \varphi_ {\mathrm{U}} + T _ {3 2} \varphi_ {\mathrm{E}} - T _ {2 1} \lambda_ {Z} + T _ {2 3} \lambda_ {x}}\tag{8.5.81}
$$

$$
\tan (\gamma + \delta \gamma) = - \frac {T _ {3 1} ^ {\prime}}{T _ {3 3} ^ {\prime}} = - \frac {T _ {3 1} + T _ {1 1} \varphi_ {\mathrm{N}} - T _ {2 1} \varphi_ {\mathrm{E}} + T _ {3 2} \lambda_ {z} - T _ {3 3} \lambda_ {y}}{T _ {3 3} + T _ {1 3} \varphi_ {\mathrm{N}} - T _ {2 3} \varphi_ {\mathrm{E}} + T _ {3 1} \lambda_ {y} - T _ {3 2} \lambda_ {x}}\tag{8.5.82}
$$

$$
\sin (\theta + \delta \theta) = T _ {3 2} ^ {\prime} = T _ {3 2} + T _ {1 2} \varphi_ {\mathrm{N}} - T _ {2 2} \varphi_ {\mathrm{E}} - T _ {3 1} \lambda_ {z} + T _ {3 3} \lambda_ {x}\tag{8.5.83}
$$

先讨论式 $(8.5.81)$ 右边项。按泰勒级数展开下式：

$$
\begin{array}{r l} & (T _ {2 2} - T _ {1 2} \varphi_ {\mathrm{U}} + T _ {3 2} \varphi_ {\mathrm{E}} - T _ {2 1} \lambda_ {z} + T _ {2 3} \lambda_ {x}) ^ {- 1} = \\ & \frac {1}{T _ {2 2}} - \frac {1}{T _ {2 2} ^ {2}} (T _ {3 2} \varphi_ {\mathrm{E}} - T _ {1 2} \varphi_ {\mathrm{U}} - T _ {2 1} \lambda_ {z} + T _ {2 3} \lambda_ {x}) + \dots \end{array}
$$

将上式代入式(8.5.81)，并取关于姿态误差角和挠曲变形角的一次项，得

$$
\begin{array}{r l} \tan \Psi + (1 + \tan^ {2} \Psi) \delta \Psi & = - \frac {T _ {1 2}}{T _ {2 2}} - \left(1 + \frac {T _ {1 2} ^ {2}}{T _ {2 2} ^ {2}}\right) \varphi_ {\mathrm{U}} + \frac {T _ {3 2}}{T _ {2 2}} \varphi_ {\mathrm{N}} + \frac {T _ {1 2} T _ {3 2}}{T _ {2 2} ^ {2}} \varphi_ {\mathrm{E}} - \\ & \left(\frac {T _ {1 3}}{T _ {2 2}} + \frac {T _ {2 3} T _ {1 2}}{T _ {2 2} ^ {2}}\right) \lambda_ {x} + \left(\frac {T _ {1 1}}{T _ {2 2}} + \frac {T _ {1 2} ^ {2}}{T _ {2 2} ^ {2}}\right) \lambda_ {z} \\ \tan \Psi & = - \frac {T _ {1 2}}{T _ {2 2}}, \quad 1 + \tan^ {2} \Psi = \frac {T _ {1 2} ^ {2} + T _ {2 2} ^ {2}}{T _ {2 2} ^ {2}} \end{array}
$$

由于

所以

$$
\begin{array}{r l} \delta \Psi = & - \varphi_ {\mathrm{U}} + \frac {T _ {3 2} T _ {2 2}}{T _ {1 2} ^ {2} + T _ {2 2} ^ {2}} \varphi_ {\mathrm{N}} + \frac {T _ {1 2} T _ {3 2}}{T _ {1 2} ^ {2} + T _ {2 2} ^ {2}} \varphi_ {\mathrm{E}} - \\ & \frac {T _ {1 3} T _ {2 2} + T _ {2 3} T _ {1 2}}{T _ {1 2} ^ {2} + T _ {2 2} ^ {2}} \lambda_ {x} + \frac {T _ {1 1} T _ {2 2} + T _ {1 2} ^ {2}}{T _ {1 2} ^ {2} + T _ {2 2} ^ {2}} \lambda_ {z} \end{array}\tag{8.5.84}
$$

同理可得

$$
\delta \gamma = \frac {T _ {2 1} T _ {3 3} - T _ {2 3} T _ {3 1}}{T _ {3 1} ^ {2} + T _ {3 3} ^ {2}} \varphi_ {\mathrm{E}} + \frac {T _ {1 3} T _ {3 1} - T _ {1 1} T _ {3 3}}{T _ {3 1} ^ {2} + T _ {3 3} ^ {2}} \varphi_ {\mathrm{N}} +
$$

$$
\frac {T _ {3 2} T _ {3 1}}{T _ {3 1} ^ {2} + T _ {3 3} ^ {2}} \lambda_ {x} + \lambda_ {y} - \frac {T _ {3 2} T _ {3 3}}{T _ {3 1} ^ {2} + T _ {3 3} ^ {2}} \lambda_ {z}\tag{8.5.85}
$$

$$
\delta \theta = \frac {T _ {1 2}}{\sqrt {1 - T _ {3 2} ^ {2}}} \varphi_ {\mathrm{N}} - \frac {T _ {2 2}}{\sqrt {1 - T _ {3 2} ^ {2}}} \varphi_ {\mathrm{E}} - \frac {T _ {3 1}}{\sqrt {1 - T _ {3 2} ^ {2}}} \lambda_ {z} + \frac {T _ {3 3}}{\sqrt {1 - T _ {3 2} ^ {2}}} \lambda_ {x}\tag{8.5.86}
$$

主惯导给出的航向、横滚和俯仰角误差分别为

$$
\delta \Psi_ {m} = - \varphi_ {m \mathrm{U}} + \frac {T _ {3 2} T _ {2 2}}{T _ {2 2} ^ {2} + T _ {1 2} ^ {2}} \varphi_ {m \mathrm{N}} + \frac {T _ {1 2} T _ {3 2}}{T _ {2 2} ^ {2} + T _ {1 2} ^ {2}} \varphi_ {m \mathrm{E}}\tag{8.5.87}
$$

$$
\delta \gamma_ {m} = \frac {T _ {2 1} T _ {3 3} - T _ {2 3} T _ {3 1}}{T _ {3 3} ^ {2} + T _ {3 1} ^ {2}} \varphi_ {m \mathrm{E}} + \frac {T _ {1 3} T _ {3 1} - T _ {1 1} T _ {3 3}}{T _ {3 3} ^ {2} + T _ {3 1} ^ {2}} \varphi_ {m \mathrm{N}}\tag{8.5.88}
$$

$$
\delta \theta_ {m} = - \frac {T _ {2 2}}{\sqrt {1 - T _ {3 2} ^ {2}}} \varphi_ {m E} + \frac {T _ {1 2}}{\sqrt {1 - T _ {3 2} ^ {2}}} \varphi_ {m N}\tag{8.5.89}
$$

所以量测量为

$$
\begin{array}{r l} & Z _ {1} = \Psi_ {s} - \Psi_ {m} = \delta \Psi - \delta \Psi_ {m} = \\ & \quad - \varphi_ {\mathrm{U}} + \frac {T _ {3 2} T _ {2 2}}{T _ {1 2} ^ {2} + T _ {2 2} ^ {2}} \varphi_ {\mathrm{N}} + \frac {T _ {1 2} T _ {3 2}}{T _ {1 2} ^ {2} + T _ {2 2} ^ {2}} \varphi_ {\mathrm{E}} - \frac {T _ {1 3} T _ {2 2} + T _ {2 3} T _ {1 2}}{T _ {1 2} ^ {2} + T _ {2 2} ^ {2}} \lambda_ {x} + \\ & \quad \frac {T _ {1 1} T _ {2 2} + T _ {1 2} ^ {2}}{T _ {1 2} ^ {2} + T _ {2 2} ^ {2}} \lambda_ {z} + \varphi_ {m \mathrm{U}} - \frac {T _ {3 2} T _ {2 2}}{T _ {1 2} ^ {2} + T _ {2 2} ^ {2}} \varphi_ {m \mathrm{N}} - \frac {T _ {1 2} T _ {3 2}}{T _ {1 2} ^ {2} + T _ {2 2} ^ {2}} \varphi_ {m \mathrm{E}} + V _ {1} \\ & Z _ {2} = \gamma_ {s} - \gamma_ {m} = \delta \gamma - \delta \gamma_ {m} = \\ & \quad \frac {T _ {2 1} T _ {3 3} - T _ {2 3} T _ {3 1}}{T _ {3 1} ^ {2} + T _ {3 3} ^ {2}} \varphi_ {\mathrm{E}} + \frac {T _ {1 3} T _ {3 1} - T _ {1 1} T _ {3 3}}{T _ {3 1} ^ {2} + T _ {3 3} ^ {2}} \varphi_ {\mathrm{N}} + \frac {T _ {3 2} T _ {3 1}}{T _ {3 1} ^ {2} + T _ {3 3} ^ {2}} \lambda_ {x} + \\ & \quad \lambda_ {\mathrm{y}} - \frac {T _ {3 2} T _ {3 3}}{T _ {3 1} ^ {2} + T _ {3 3} ^ {2}} \lambda_ {\mathrm{z}} - \frac {T _ {2 1} T _ {3 3} - T _ {2 3} T _ {3 1}}{T _ {3 1} ^ {2} + T _ {3 3} ^ {2}} \varphi_ {\mathrm{mE}} - \frac {T _ {1 3} T _ {3 1} - T _ {1 1} T _ {3 3}}{T _ {3 1} ^ {2} + T _ {3 3} ^ {3}} \varphi_ {\mathrm{mN}} + V _ {2}. \end{array}\tag{8.5.90}
$$

(8.5.91)

$$
\begin{array}{r l} Z _ {3} & = \theta_ {s} - \theta_ {m} = \delta \theta - \delta \theta_ {m} = \\ & \frac {T _ {1 2}}{\sqrt {1 - T _ {3 2} ^ {2}}} \varphi_ {\mathrm{N}} - \frac {T _ {2 2}}{\sqrt {1 - T _ {3 2} ^ {2}}} \varphi_ {\mathrm{E}} - \frac {T _ {3 1}}{\sqrt {1 - T _ {3 2} ^ {2}}} \lambda_ {z} + \\ & \frac {T _ {3 3}}{\sqrt {1 - T _ {3 2} ^ {2}}} \lambda_ {x} - \frac {T _ {1 2}}{\sqrt {1 - T _ {3 2} ^ {2}}} \varphi_ {m \mathrm{N}} + \frac {T _ {2 2}}{\sqrt {1 - T _ {3 2} ^ {2}}} \varphi_ {m \mathrm{E}} + V _ {3} \end{array}\tag{8.5.92}
$$

（4）姿态阵匹配量。设主、子惯导选用相同的导航坐标系 n，输出的姿态阵分别为 $\hat{C}_{b_{m}}^{n}$ 和 $\hat{C}_{b_{s}}^{n}$ ，其中 $b_{m}$ 为机载主惯导的测量坐标系， $b_{s}$ 为弹载子惯导的测量坐标系，主、子惯导的设计安装方式为平行安装， $b_{s}$ 相对 $b_{m}$ 存在偏差角 $\mu$ 和 $\lambda, \mu$ 由导弹相对机体的安装误差和子惯导 IMU 相对弹体的安装误差引起， $\lambda$ 由挠曲变形和高频颤振引起。

构造矩阵

$$
\boldsymbol {M} = \hat {\boldsymbol {C}} _ {b _ {m}} ^ {n} (\hat {\boldsymbol {C}} _ {b _ {s}} ^ {n}) ^ {\mathrm{T}}\tag{8.5.93}
$$

记主、子惯导建立的数学平台分别为 $n'$ 和 $n''$ ，姿态误差角分别为 $\phi_{m}$ 和 $\phi$ ，则

$$
\begin{array}{r l} & \boldsymbol {M} = \boldsymbol {C} _ {n} ^ {n ^ {\prime}} \boldsymbol {C} _ {b _ {m}} ^ {n} (\boldsymbol {C} _ {n} ^ {n ^ {\prime \prime}} \boldsymbol {C} _ {b _ {s}} ^ {n}) \mathrm{T} = \boldsymbol {C} _ {n} ^ {n ^ {\prime}} \boldsymbol {C} _ {b _ {m}} ^ {n} \boldsymbol {C} _ {n ^ {\prime}} ^ {b _ {s}} \boldsymbol {C} _ {n ^ {\prime \prime}} ^ {n} = (\boldsymbol {I} - \boldsymbol {\phi} _ {m} \times) \boldsymbol {C} _ {b _ {m}} ^ {n} \boldsymbol {C} _ {b _ {m} ^ {\prime}} ^ {b _ {s}} \boldsymbol {C} _ {n ^ {\prime}} ^ {b _ {m}} (\boldsymbol {I} + \boldsymbol {\phi} \times) = \\ & [ \boldsymbol {I} - (\boldsymbol {\phi} _ {m}) \times ] \boldsymbol {C} _ {b _ {m}} ^ {n} [ \boldsymbol {I} - (\boldsymbol {\mu} + \boldsymbol {\lambda}) \times ] \boldsymbol {C} _ {n ^ {\prime}} ^ {b _ {m}} (\boldsymbol {I} + \boldsymbol {\varphi} \times) \end{array}\tag{8.5.94}
$$

式(8.5.94)中， $\mu$ 为子惯导相对主惯导的静态安装误差， $\lambda$ 为机翼的挠曲变形角，与 $\phi_{m},\phi$ 类似，均为小角，矩阵乘展开过程中，略去了关于这些小角的二阶小量。

对式 $(8.5.94)$ 右侧第二项做如下化简处理。记

$$
\boldsymbol {C} _ {b _ {m}} ^ {n} = \left[ \begin{array}{l l l} C _ {1 1} & C _ {1 2} & C _ {1 3} \\ C _ {2 1} & C _ {2 2} & C _ {2 3} \\ C _ {3 1} & C _ {3 2} & C _ {3 3} \end{array} \right], \quad \boldsymbol {\mu} + \boldsymbol {\lambda} = \boldsymbol {r} ^ {b} = \left[ \begin{array}{l} r _ {x} ^ {b} \\ r _ {y} ^ {b} \\ r _ {z} ^ {b} \end{array} \right]
$$

则有

即

$$
\boldsymbol {r} ^ {n} = \boldsymbol {C} _ {b _ {m}} ^ {n} \boldsymbol {r} ^ {b}
$$

$$
\begin{array}{r l} \left[ \begin{array}{l} r _ {x} ^ {n} \\ r _ {y} ^ {n} \\ r _ {z} ^ {n} \end{array} \right] & = \left[ \begin{array}{l l l} C _ {1 1} & C _ {1 2} & C _ {1 3} \\ C _ {2 1} & C _ {2 2} & C _ {2 3} \\ C _ {3 1} & C _ {3 2} & C _ {3 3} \end{array} \right] \left[ \begin{array}{l} r _ {x} ^ {b} \\ r _ {y} ^ {b} \\ r _ {z} ^ {b} \end{array} \right] = \left[ \begin{array}{l} C _ {1 1} r _ {x} ^ {b} + C _ {1 2} r _ {y} ^ {b} + C _ {1 3} r _ {z} ^ {b} \\ C _ {2 1} r _ {x} ^ {b} + C _ {2 2} r _ {y} ^ {b} + C _ {2 3} r _ {z} ^ {b} \\ C _ {3 1} r _ {x} ^ {b} + C _ {3 2} r _ {y} ^ {b} + C _ {3 3} r _ {z} ^ {b} \end{array} \right] \\ & (\boldsymbol {r} ^ {b} \times) = \boldsymbol {R - R ^ {\mathrm{T}}} \end{array}\tag{8.5.95}
$$

其中

$$
\boldsymbol {R} = \left[ \begin{array}{c c c} 0 & 0 & 0 \\ r _ {z} ^ {b} & 0 & 0 \\ - r _ {y} ^ {b} & r _ {x} ^ {b} & 0 \end{array} \right] \boldsymbol {C} _ {b _ {m}} ^ {n} (\boldsymbol {r} ^ {b} \times) \boldsymbol {C} _ {n} ^ {b _ {m}} = \boldsymbol {C} _ {b _ {m}} ^ {n} \boldsymbol {R} \boldsymbol {C} _ {n} ^ {b _ {m}} - \boldsymbol {C} _ {b _ {m}} ^ {n} \boldsymbol {R} ^ {\mathrm{T}} \boldsymbol {C} _ {n} ^ {b _ {m}} = \boldsymbol {C} _ {b _ {m}} ^ {n} \boldsymbol {R} \boldsymbol {C} _ {n} ^ {b _ {m}} - (\boldsymbol {C} _ {b _ {m}} ^ {n} \boldsymbol {R} \boldsymbol {C} _ {n} ^ {b _ {m}}) ^ {\mathrm{T}}
$$

记

$$
\begin{array}{r} N = C _ {b _ {m}} ^ {n} R C _ {n ^ {m}} ^ {b} \\ L = C _ {b _ {m}} ^ {n} (r ^ {b} \times) C _ {n} ^ {b _ {m}} \end{array}
$$

则有

$$
\boldsymbol {L} = \boldsymbol {N} - \boldsymbol {N} ^ {\mathrm{T}}\tag{8.5.96}
$$

其中

$$
\begin{array}{l} \boldsymbol {N} = \left[ \begin{array}{c c c} C _ {1 1} & C _ {1 2} & C _ {1 3} \\ C _ {2 1} & C _ {2 2} & C _ {2 3} \\ C _ {3 1} & C _ {3 2} & C _ {3 3} \end{array} \right] \left[ \begin{array}{c c c} 0 & 0 & 0 \\ r _ {z} ^ {b} & 0 & 0 \\ - r _ {y} ^ {b} & r _ {x} ^ {b} & 0 \end{array} \right] \left[ \begin{array}{c c c} C _ {1 1} & C _ {2 1} & C _ {3 1} \\ C _ {1 2} & C _ {2 2} & C _ {3 2} \\ C _ {1 3} & C _ {2 3} & C _ {3 3} \end{array} \right] = \\ \left[ \begin{array}{c c c} C _ {1 2} r _ {z} ^ {b} - C _ {1 3} r _ {y} ^ {b} & C _ {1 3} r _ {x} ^ {b} & 0 \\ C _ {2 2} r _ {z} ^ {b} - C _ {2 3} r _ {y} ^ {b} & C _ {2 3} r _ {x} ^ {b} & 0 \\ C _ {3 2} r _ {z} ^ {b} - C _ {3 3} r _ {y} ^ {b} & C _ {3 3} r _ {x} ^ {b} & 0 \end{array} \right] \left[ \begin{array}{c c c} C _ {1 1} & C _ {2 1} & C _ {3 1} \\ C _ {1 2} & C _ {2 2} & C _ {3 2} \\ C _ {1 3} & C _ {2 3} & C _ {2 3} \end{array} \right] = \\ \left[ \begin{array}{c c c} C _ {1 1} (C _ {1 2} r _ {z} ^ {b} - C _ {1 3} r _ {y} ^ {b}) + C _ {1 2} C _ {1 3} r _ {x} ^ {b} & C _ {2 1} (C _ {1 2} r _ {z} ^ {b} - C _ {1 3} r _ {y} ^ {b}) + C _ {1 3} C _ {2 2} r _ {x} ^ {b} \\ C _ {1 1} (C _ {2 2} r _ {z} ^ {b} - C _ {2 3} r _ {y} ^ {b}) + C _ {1 2} C _ {2 3} r _ {x} ^ {b} & C _ {2 1} (C _ {2 2} r _ {z} ^ {b} - C _ {2 3} r _ {y} ^ {b}) + C _ {2 3} C _ {2 2} r _ {x} ^ {b} \\ C _ {1 1} (C _ {3 2} r _ {z} ^ {b} - C _ {3 3} r _ {\mathrm{y}} ^ {b}) + C _ {1 2} C _ {3 3} r _ {\mathrm{x}} ^ {b} & C _ {2 1} (C _ {3 2} r _ {\mathrm{z}} ^ {b} - C _ {3 3} r _ {\mathrm{y}} ^ {\mathrm{b}}) + C _ {3 3} C _ {2 2} r _ {\mathrm{x}} ^ {\mathrm{b}} \\ C _ {3 1} (C _ {1 2} r _ {\mathrm{z}} ^ {\mathrm{b}} - C _ {1 3} r _ {\mathrm{y}} ^ {\mathrm{b}}) + C _ {1 3} C _ {\mathrm{32}} r _ {\mathrm{x}} ^ {\mathrm{b}} \\ C _ {\mathrm{31}} (C _ {\mathrm{22}} r _ {\mathrm{z}} ^ {\mathrm{b}} - C _ {\mathrm{23}} r _ {\mathrm{y}} ^ {\mathrm{b}}) + C _ {\mathrm{23}} C _ {\mathrm{32}} r _ {\mathrm{x}} ^ {\mathrm{b}} \\ C _ {\mathrm{31}} (C _ {\mathrm{32}} r _ {\mathrm{z}} ^ {\mathrm{b}} - C _ {\mathrm{33}} r _ {\mathrm{y}} ^ {\mathrm{b}}) + C _ {\mathrm{33}} C _ {\mathrm{32}} r _ {\mathrm{x}} ^ {\mathrm{b}} \end{array}\tag{8.5.97}
$$

式 $(8.5.97)$ 代入式 $(8.5.96)$ ，得

$$
\begin{array}{r l} & L (1, 1) = 0 \quad L (2, 2) = 0 \quad L (3, 3) = 0 \\ & L (2, 1) = N (2, 1) - N (1, 2) = (C _ {1 2} C _ {2 3} - C _ {1 3} C _ {2 2}) r _ {x} ^ {b} + (C _ {2 1} C _ {1 3} - C _ {1 1} C _ {2 3}) r _ {y} ^ {b} + \\ & (C _ {1 1} C _ {2 2} - C _ {2 1} C _ {1 2}) r _ {z} ^ {b} \\ & L (3, 1) = N (3, 1) - N (1, 3) = (C _ {1 2} C _ {3 3} - C _ {1 3} C _ {3 2}) r _ {x} ^ {b} + (C _ {3 1} C _ {1 3} - C _ {1 1} C _ {3 3}) r _ {y} ^ {b} + \end{array}\tag{8.5.98a}
$$

$$
\begin{array}{r l} & (C _ {1 1} C _ {3 2} - C _ {3 1} C _ {1 2}) r _ {z} ^ {b} \\ L (3, 2) = N (3, 2) - N (2, 3) = & (C _ {2 2} C _ {3 3} - C _ {2 3} C _ {3 2}) r _ {x} ^ {b} + (C _ {3 1} C _ {2 3} - C _ {2 1} C _ {3 3}) r _ {y} ^ {b} + \\ & (C _ {2 1} C _ {3 2} - C _ {3 1} C _ {2 2}) r _ {z} ^ {b} \\ & L (1, 2) = N (1, 2) - N (2, 1) = - L (2, 1) \\ & L (1, 3) = N (1, 3) - N (3, 1) = - L (3, 1) \\ & L (2, 3) = N (2, 3) - N (3, 2) = - L (3, 2) \end{array}\tag{8.5.98b}
$$

(8.5.98c)

由于姿态阵 $C_{b_m}^m$ 为单位正交阵，有

$$
(\boldsymbol {C} _ {b _ {m}} ^ {n}) ^ {- 1} = (\boldsymbol {C} _ {b _ {m}} ^ {n}) ^ {\mathrm{T}}
$$

即

$$
\left(\left[ \begin{array}{l l l} C _ {1 1} & C _ {1 2} & C _ {1 3} \\ C _ {2 1} & C _ {2 2} & C _ {2 3} \\ C _ {3 1} & C _ {3 2} & C _ {3 3} \end{array} \right]\right) ^ {- 1} = \left[ \begin{array}{l l l} C _ {1 1} & C _ {2 1} & C _ {3 1} \\ C _ {1 2} & C _ {2 2} & C _ {3 2} \\ C _ {1 3} & C _ {2 3} & C _ {3 3} \end{array} \right]
$$

所以

$$
\begin{array}{l l l} C _ {1 1} = C _ {2 2} C _ {3 3} - C _ {2 3} C _ {3 2} & C _ {1 2} = C _ {2 3} C _ {3 1} - C _ {2 1} C _ {3 3} & C _ {1 3} = C _ {2 1} C _ {3 2} - C _ {2 2} C _ {3 1} \\ C _ {2 1} = C _ {1 3} C _ {3 2} - C _ {1 2} C _ {3 3} & C _ {2 2} = C _ {1 1} C _ {3 3} - C _ {1 3} C _ {3 1} & C _ {2 3} = C _ {1 2} C _ {3 1} - C _ {1 1} C _ {3 2} \\ C _ {3 1} = C _ {1 2} C _ {2 3} - C _ {1 3} C _ {2 2} & C _ {3 2} = C _ {1 3} C _ {2 1} - C _ {1 1} C _ {2 3} & C _ {3 3} = C _ {1 1} C _ {2 2} - C _ {1 2} C _ {2 1} \end{array}
$$

上述关系式代入式(8.5.98)，并根据式(8.5.95)，得

$$
\begin{array}{r l} & L (2, 1) = C _ {3 1} r _ {x} ^ {b} + C _ {3 2} r _ {y} ^ {b} + C _ {3 3} r _ {z} ^ {b} = r _ {z} ^ {n} \\ & L (3, 1) = - (C _ {2 1} r _ {x} ^ {b} + C _ {2 2} r _ {y} ^ {b} + C _ {2 3} r _ {z} ^ {b}) = - r _ {y} ^ {n} \\ & L (3, 2) = C _ {1 1} r _ {x} ^ {b} + C _ {1 2} r _ {y} ^ {b} + C _ {1 3} r _ {z} ^ {b} = r _ {x} ^ {n} \end{array}
$$

所以

$$
\boldsymbol {L} = \left[ \begin{array}{c c c} 0 & - r _ {z} ^ {n} & r _ {y} ^ {n} \\ r _ {z} ^ {n} & 0 & - r _ {x} ^ {n} \\ - r _ {y} ^ {n} & r _ {x} ^ {n} & 0 \end{array} \right] = (\boldsymbol {r} ^ {n} \times) = [ (\boldsymbol {C} _ {b _ {m}} ^ {n} \boldsymbol {r} ^ {b}) \times ]
$$

即

$$
\boldsymbol {C} _ {b _ {m}} ^ {n} [ (\boldsymbol {\mu} + \boldsymbol {\lambda}) \times ] \boldsymbol {C} _ {n ^ {m}} ^ {b} = \left\{\left[ \boldsymbol {C} _ {b _ {m}} ^ {n} (\boldsymbol {\mu} + \boldsymbol {\lambda}) \right] \times \right\}\tag{8.5.99}
$$

式 $(8.5.99)$ 代入式 $(8.5.94)$ ，得

$$
\boldsymbol {M} = \boldsymbol {I} + \left\{\left[ \boldsymbol {\phi} - \boldsymbol {C} _ {b _ {m}} ^ {n} (\boldsymbol {\mu} + \lambda) - \boldsymbol {\phi} _ {m} \right] \times \right\}\tag{8.5.100}
$$

可见，M 是反对称矩阵，可写成如下形式

$$
\boldsymbol {M} = \left[ \begin{array}{c c c} 1 & - Z _ {z} & Z _ {y} \\ Z _ {z} & 1 & - Z _ {x} \\ - Z _ {y} & Z _ {x} & 1 \end{array} \right]\tag{8.5.101}
$$

取 $Z_{DCM} = [Z_{x} \quad Z_{y} \quad Z_{z}]^{T}$ 作为量测量，则有

$$
\begin{array}{r l} & Z _ {z} = M (2, 1), \quad Z _ {y} = M (1, 3), \quad Z _ {x} = M (3, 2) \\ & - Z _ {z} = M (1, 2), \quad - Z _ {y} = M (3, 1), \quad - Z _ {x} = M (2, 3) \end{array}
$$

由此得

$$
Z _ {x} = \frac {1}{2} [ M (3, 2) - M (2, 3) ]\tag{8.5.102a}
$$

$$
Z _ {y} = \frac {1}{2} [ M (1, 3) - M (3, 1) ]\tag{8.5.102b}
$$

$$
Z _ {z} = \frac {1}{2} \left[ M (2, 1) - M (1, 2) \right]\tag{8.5.102c}
$$

比较式(8.5.101)和式(8.5.100)，可得量测量 $Z_{DCM}$ 与误差量之间的函数关系式

$$
\mathbf {Z} _ {D C M} = \boldsymbol {\phi} - \mathbf {C} _ {b _ {m}} ^ {m} (\boldsymbol {\mu} + \boldsymbol {\lambda}) - \boldsymbol {\phi} _ {m}\tag{8.5.103}
$$

式(8.5.103)即为姿态阵匹配量测方程,其中量测量 $Z_{DCM}$ 按式(8.5.102)确定,矩阵M按式(8.5.93)确定。

（5）姿态四元数匹配量。设主、子惯导选用相同的导航坐标系 $n$ ，输出的姿态四元数分别为 $\hat{Q}_m$ 和 $\hat{Q}_s$ 。 $b_{m}$ 和 $b_{s}$ 坐标系的定义和 $\pmb{\mu},\lambda$ 等物理量的定义同前所述。根据姿态阵与姿态四元数间的等价关系，与 $\hat{C}_{b_m}^n = C_{b_m}^{n'} = C_n^{n'}C_{b_m}^n$ 对应的四元数为

$$
\hat {Q} _ {m} = \hat {Q} _ {n ^ {m}} ^ {b} = Q _ {n ^ {\prime}} ^ {n} \otimes Q _ {n ^ {m}} ^ {b}
$$

与 $\hat{C}_{b_s}^n = C_{b_s}^{n''} = C_n^n C_{b_s}^n$ 对应的四元数为

$$
\hat {Q} _ {m} = \hat {Q} _ {n ^ {s}} ^ {b} = Q _ {n ^ {\prime \prime}} ^ {n} \otimes Q _ {n ^ {s}} ^ {b}
$$

其中 $n^{\prime}$ 和 $n''$ 分别为主、子惯导建立的数学平台。四元数 $Q_{n}^{b_m}$ 描述了坐标系 $n$ 至坐标系 $b_{m}$ 的等效旋转，其余用上下标表示的四元数说明与之类似。

用 $\hat{Q}_m$ 和 $\hat{Q}_s$ 构建四元数

$$
\boldsymbol {Q} = \hat {\boldsymbol {Q}} _ {s} ^ {*} \otimes \hat {\boldsymbol {Q}} _ {m}\tag{8.5.104}
$$

则

$$
\boldsymbol {Q} = \left[ \left(\boldsymbol {Q} _ {n ^ {s}} ^ {b}\right) ^ {*} \otimes \left(\boldsymbol {Q} _ {n ^ {\prime \prime}} ^ {n}\right) ^ {*} \right] \otimes \left(\boldsymbol {Q} _ {n ^ {\prime}} ^ {n} \otimes \boldsymbol {Q} _ {n ^ {m}} ^ {b}\right) = \boldsymbol {Q} _ {b _ {s}} ^ {n} \otimes \left[ \begin{array}{l} 1 \\ \frac {\boldsymbol {\phi}}{2} \end{array} \right] \otimes \left[ \begin{array}{l} 1 \\ - \frac {\boldsymbol {\phi} _ {m}}{2} \end{array} \right] \otimes \boldsymbol {Q} _ {n ^ {m}} ^ {b}
$$

式中， $\phi$ 和 $\phi_{m}$ 分别为子惯导和主惯导的姿态误差角，是在导航坐标系内的数学向量。并且

$$
\left[ \begin{array}{l} 1 \\ \frac {\boldsymbol {\phi}}{2} \end{array} \right] \otimes \left[ \begin{array}{l} 1 \\ - \frac {\boldsymbol {\phi} _ {m}}{2} \end{array} \right] = \left(1 + \frac {\boldsymbol {\phi}}{2}\right) \otimes \left(1 - \frac {\boldsymbol {\phi} _ {m}}{2}\right) = 1 + \frac {\boldsymbol {\phi}}{2} - \frac {\boldsymbol {\phi} _ {m}}{2} = \left[ \begin{array}{l} 1 \\ \frac {\boldsymbol {\phi} - \boldsymbol {\phi} _ {m}}{2} \end{array} \right]
$$

上式中，略去了向量叉乘引起的二阶小量。所以

$$
\begin{array}{r l} Q & = Q _ {b _ {s} ^ {m}} ^ {b} \otimes Q _ {b _ {m}} ^ {n} \otimes \left[ \frac {1}{\frac {\phi - \phi_ {m}}{2}} \right] \otimes Q _ {n} ^ {b _ {m}} = \left[ \frac {1}{- \frac {\mu + \lambda}{2}} \right] \otimes \left[ Q _ {b _ {m}} ^ {n} \otimes \frac {\phi - \phi_ {m}}{2} \otimes Q _ {n} ^ {b _ {m}} \right] = \\ & \left[ - \frac {\mu + \lambda}{2} \right] \otimes \left[ C _ {n} ^ {b _ {m}} \frac {\phi - \phi_ {m}}{2} \right] \end{array}
$$

略去向量叉乘引起的二阶小量，

$$
Q = \left[ \begin{array}{c} 1 \\ - \frac {\boldsymbol {\mu} + \lambda}{2} + \mathbf {C} _ {n ^ {m}} ^ {\phi} \frac {\boldsymbol {\phi} - \boldsymbol {\phi} _ {m}}{2} \end{array} \right]
$$

记 $Q=\begin{bmatrix}1\\ Q_{1}\\ Q_{2}\\ Q_{3}\end{bmatrix}$ ，并取量测量

$$
\mathbf {Z} _ {Q} = \left[ \begin{array}{l} 2 Q _ {1} \\ 2 Q _ {2} \\ 2 Q _ {3} \end{array} \right]\tag{8.5.105}
$$

则量测量与误差量间的函数关系为

$$
\mathbf {Z} _ {Q} = \mathbf {C} _ {n ^ {m}} ^ {b} \boldsymbol {\phi} - \mathbf {C} _ {n ^ {m}} ^ {b} \boldsymbol {\phi} _ {m} - \boldsymbol {\mu} - \lambda_ {f} - \lambda_ {\nu}\tag{8.5.106}
$$

由于主惯导的姿态误差远小于子惯导的姿态误差,为简化传递对准算法,将主惯导的姿态误差近似视为白噪声,与高频颤振归并入量测噪声。

$$
\boldsymbol {V} _ {Q} = - \boldsymbol {C} _ {n ^ {m}} ^ {b} \boldsymbol {\phi} _ {m} - \lambda_ {\nu}\tag{8.5.107}
$$

所以由四元数构造的匹配量可写成

$$
\mathbf {Z} _ {Q} = \mathbf {C} _ {n ^ {m}} ^ {b} \boldsymbol {\phi} - \boldsymbol {\mu} - \boldsymbol {\lambda} _ {f} + \mathbf {V} _ {Q}\tag{8.5.108}
$$

其中

$$
\boldsymbol {R} _ {Q} = E \left[ \boldsymbol {V} _ {Q} \boldsymbol {V} _ {Q} ^ {\mathrm{T}} \right] = \boldsymbol {C} _ {n ^ {m}} ^ {b} \boldsymbol {R} _ {\phi_ {m}} \boldsymbol {C} _ {b _ {m}} ^ {n} + \boldsymbol {R} _ {\lambda \nu}\tag{8.5.109}
$$

$R_{\phi_{m}}$ 为主惯导姿态误差的方差阵， $R_{\lambda\nu}$ 为高频颤振的方差阵。

匹配量的选取与主、子惯导的类型有关。由于成本和重量、体积的关系，子惯导一般都采用捷联惯导，而主惯导有可能是平台式惯导，也可能是捷联式惯导。如果主惯导是平台式的，由于平台式惯导不能给出姿态角速度信息，且测量平台环架角的同步器误差过大使姿态角精度很低，所以当主惯导为平台式惯导时，一般采用速度匹配，也可采用速度和方位同时匹配 $^{[99]}$ 。如果主惯导也是捷联式惯导系统，则以上所介绍的匹配方案都可采用。

表 8.5.4 列出各种匹配方案的特点。对于空-空弹，载机捕获目标后必须紧紧跟踪并锁定目标，与此同时必须完成对弹载子惯导的传递对准，所以载机应尽量避免机动，特别是盘旋和俯仰运动，以防止雷达丢失目标。因此同时采用姿态角和速度作为匹配量的方案是较合适的传递对准方案。

表 8.5.4 各种匹配方案的特点

<table><tr><td>匹配方案</td><td>辅助机动</td><td>滤波计算量</td><td>速度估计效果</td><td>方位、姿态估计收敛速度</td></tr><tr><td>速度</td><td>盘旋或直线加速</td><td>小</td><td>好</td><td>慢</td></tr><tr><td>角速度</td><td>摇摆</td><td>小</td><td>差</td><td>快</td></tr><tr><td>姿态角</td><td>不需要</td><td>小</td><td>差</td><td>快</td></tr><tr><td>姿态阵</td><td>不需要</td><td>大</td><td>差</td><td>快</td></tr><tr><td>姿态角和速度</td><td>不需要</td><td>较大</td><td>好</td><td>快</td></tr></table>

## 8.6 应用基本滤波理论设计组合导航系统

## 8.6.1 列写系统方程和量测方程的原则

导航系统一般都是非线性系统,描述系统的方程都很复杂。导航系统的误差方程本身也都是非线性方程,但由于具有一定精度的导航系统的误差量都可看做小量,非线性方程中关于误差量的高阶项都可看做高阶小量而略去不计,所以误差方程可描述为线性方程。为了使量测方程也具有线性形式,即量测量也是误差量的线性函数,量测量必须是两种非相似导航系统对同一导航参数输出量的差值。在这种差值中,真实导航参数已经互相对消掉,剩下的只是这两种导航系统的相应误差量。因此,用误差量作为状态的系统方程和量测方程都是线性方程,这就可以采用基本方程来设计卡尔曼滤波器,避免了采用广义卡尔曼滤波所造成的不必要的麻烦。滤波器的输出是对误差量的估计值,误差被估计出来后,可用来校正相应的导航系统,以提高导航系统的精度。

以上是选择状态变量和量测量,列写状态方程和量测方程的一般原则。除此之外,还应注意以下几点:

（1）在满足估计精度的前提下，尽可能地减少状态，以减少滤波器的复杂性。但是一般总要包括速度误差、位置误差和平台误差角等状态。在速度组合或位置组合中，这些状态的估计效果一般都是比较明显的。

（2）除了速度误差、位置误差和平台误差角之外，尚需要考虑的状态还有惯性器件的误差量。在这些误差量中，加速度计偏置和平台倾角之间的不可分辨性仍然存在。因此，一般在状态方程中不考虑加速度计偏置。其次，从速度误差和位置误差的近似解中看出，方位陀螺漂移在速度误差量测值或位置误差量测值中的信息非常小，所以，它的估计效果一般不会很好（除非它的值很大，或两次量测之间的时间间隔很长，或量测噪声小）。因此，在连续工作时间不太长的组合导航系统中，常略去对它的估计，也就是仅仅将方位陀螺的漂移考虑为白噪声，在平台方位误差方程中成为模型噪声。至于两个水平陀螺漂移，它们在量测值中的信息也是比较小的，因此，也存在与方位陀螺漂移类似的情况，即要求两次量测量之间的时间间隔长些。如果漂移值不大或量测噪声大，以致使估计效果不好，则同样可以考虑不作估计。

（3）由惯性导航理论知，惯导系统的纯惯性高度通道是不稳定的，要克服这一问题，必须向惯导高度通道引入其余导航系统提供的高度信息，从外部引入阻尼。

## 8.6.2 由集中滤波器实现的组合导航系统设计例举

假设构成组合导航系统的子系统有:激光陀螺捷联惯导系统(INS)、C/A码GPS接收机(GPS)、多普勒雷达(DVS)、大气数据计算机(ADS)、垂直陀螺(VG)和磁航向仪(MG)。采用一个卡尔曼滤波器,即集中滤波器实现系统间的组合。激光捷联惯导与其余子系统形成的量测量全部作为该集中滤波器的量测量。下面介绍该滤波器的设计,并给出在设定飞行轨迹条件下,滤波效果的部分仿真计算结果。

## 1. 状态变量的选取和系统方程的建立

(1) 激光陀螺捷联惯导系统。选择惯导系统的状态向量为

$$
\begin{array}{r l} \mathbf {X} _ {\mathrm{INS}} = & \left[ \delta L \quad \delta \lambda \quad \delta h \quad \delta v _ {\mathrm{E}} \quad \delta v _ {\mathrm{N}} \quad \delta v _ {\mathrm{U}} \quad \varphi_ {\mathrm{E}} \quad \varphi_ {\mathrm{N}} \quad \varphi_ {\mathrm{U}} \right. \\ & \left. \varepsilon_ {b x} \quad \varepsilon_ {b y} \quad \varepsilon_ {b z} \quad \varepsilon_ {r x} \quad \varepsilon_ {r y} \quad \varepsilon_ {r z} \quad \bigtriangledown_ {x} \quad \bigtriangledown_ {y} \quad \bigtriangledown_ {z} \right. \\ & \left. \delta K _ {g x} \quad \delta K _ {g y} \quad \delta K _ {g z} \quad \delta K _ {a x} \quad \delta K _ {a y} \quad \delta K _ {a z} \right] ^ {\mathrm{T}} \end{array}\tag{8.6.1}
$$

状态向量共 24 维。根据 8.4 节中关于捷联惯导误差方程的介绍，状态方程为

$$
\dot {\boldsymbol {X}} _ {\mathrm{INS}} = \boldsymbol {F} _ {\mathrm{INS}} \boldsymbol {X} _ {\mathrm{INS}} + \boldsymbol {G} _ {\mathrm{INS}} \boldsymbol {w} _ {\mathrm{INS}}\tag{8.6.2}
$$

式中，系统噪声为

$$
\boldsymbol {w} _ {\mathrm{INS}} = \left[ \begin{array}{l l l l l l l l l} w _ {g x} & w _ {g y} & w _ {g z} & w _ {r x} & w _ {r y} & w _ {r z} & w _ {a x} & w _ {a y} & w _ {a z} \end{array} \right] ^ {\mathrm{T}}\tag{8.6.3}
$$

系统噪声分配阵为

$$
\mathbf {G} _ {\mathrm{INS}} = \left[ \begin{array}{l l l} \mathbf {0} _ {6 \times 3} & \mathbf {0} _ {6 \times 3} & \mathbf {0} _ {6 \times 3} \\ \mathbf {C} _ {b} ^ {n} & \mathbf {0} _ {3 \times 3} & \mathbf {0} _ {3 \times 3} \\ \mathbf {0} _ {3 \times 3} & \mathbf {0} _ {3 \times 3} & \mathbf {0} _ {3 \times 3} \\ \mathbf {0} _ {3 \times 3} & I _ {3 \times 3} & \mathbf {0} _ {3 \times 3} \\ \mathbf {0} _ {3 \times 3} & \mathbf {0} _ {3 \times 3} & I _ {3 \times 3} \\ \mathbf {0} _ {6 \times 3} & \mathbf {0} _ {6 \times 3} & \mathbf {0} _ {6 \times 3} \end{array} \right]\tag{8.6.4}
$$

$F_{INS}$ 的非零元为

$$
F _ {1, 3} = - \frac {v _ {\mathrm{N}}}{(R + h) ^ {2}}
$$

$$
F _ {1, 5} = \frac {1}{R + h}
$$

$$
F _ {2, 1} = \frac {v _ {\mathrm{E}}}{R + h} \sec L \tan L
$$

$$
F _ {2, 3} = - \frac {v _ {\mathrm{E}}}{(R + h) ^ {2}} \sec L
$$

$$
F _ {2, 4} = \frac {\sec L}{R + h}
$$

$$
F _ {3, 6} = 1
$$

$$
F _ {4, 1} = 2 \omega_ {i e} \cos L v _ {\mathrm{N}} + \frac {v _ {\mathrm{E}} v _ {\mathrm{N}}}{R + h} \sec^ {2} L + 2 \omega_ {i e} \sin L v _ {\mathrm{U}}
$$

$$
F _ {4, 3} = \frac {v _ {\mathrm{E}} v _ {\mathrm{U}} - v _ {\mathrm{E}} v _ {\mathrm{N}} \tan L}{(R + h) ^ {2}}
$$

$$
F _ {4, 4} = \frac {v _ {\mathrm{N}}}{R + h} \tan L - \frac {v _ {\mathrm{U}}}{R + h}
$$

$$
F _ {4, 5} = 2 \omega_ {i e} \sin L + \frac {v _ {\mathrm{E}}}{R + h} \tan L
$$

$$
F _ {4, 6} = - \left(2 \omega_ {i e} \cos L + \frac {v _ {\mathrm{E}}}{R + h}\right)
$$

$$
F _ {4, 8} = - f _ {\mathrm{U}}
$$

$$
F _ {4, 9} = f _ {\mathrm{N}}
$$

$$
F _ {4, 1 6} = C _ {b} ^ {n} (1, 1)
$$

$$
F _ {4, 1 7} = C _ {b} ^ {n} (1, 2)
$$

$$
F _ {4, 1 8} = C _ {b} ^ {n} (1, 3)
$$

$$
F _ {4, 2 2} = C _ {b} ^ {n} (1, 1) f _ {x}
$$

$$
F _ {4, 2 3} = C _ {b} ^ {n} (1, 2) f _ {y}
$$

$$
F _ {4, 2 4} = C _ {b} ^ {n} (1, 3) f _ {z}
$$

$$
F _ {5, 1} = - \left(2 \omega_ {i e} \cos L v _ {\mathrm{E}} + \frac {v _ {\mathrm{E}} ^ {2} \sec^ {2} L}{R + h}\right)
$$

$$
F _ {5, 3} = \frac {v _ {\mathrm{E}} ^ {2} \tan L + v _ {\mathrm{N}} v _ {\mathrm{U}}}{(R + h) ^ {2}}
$$

$$
F _ {5, 4} = - 2 \left(\omega_ {i e} \sin L + \frac {v _ {\mathrm{E}}}{R + h} \tan L\right)
$$

$$
F _ {5, 5} = - \frac {v _ {\mathrm{U}}}{R + h}
$$

$$
F _ {5, 6} = - \frac {v _ {\mathrm{N}}}{R + h}
$$

$$
F _ {5. 7} = f _ {\mathrm{U}}
$$

$$
F _ {5, 9} = - f _ {\mathrm{E}}
$$

$$
F _ {5, 1 6} = C _ {b} ^ {n} (2, 1)
$$

$$
F _ {5, 1 7} = C _ {b} ^ {n} (2, 2)
$$

$$
F _ {5, 1 8} = C _ {b} ^ {n} (2, 3)
$$

$$
F _ {5, 2 2} = C _ {b} ^ {n} (2, 1) f _ {x}
$$

$$
F _ {5. 2 3} = C _ {b} ^ {n} (2, 2) f _ {y}
$$

$$
F _ {5, 2 4} = C _ {b} ^ {n} (2, 3) f _ {z}
$$

$$
F _ {6, 1} = - 2 \omega_ {i e} v _ {\mathrm{E}} \sin L
$$

$$
F _ {6, 3} = - \frac {v _ {\mathrm{E}} ^ {2} + v _ {\mathrm{N}} ^ {2}}{(R + h) ^ {2}}
$$

$$
F _ {6, 4} = 2 \left(\omega_ {i e} \cos L + \frac {v _ {\mathrm{E}}}{R + h}\right)
$$

$$
F _ {6. 5} = \frac {2 v _ {\mathrm{N}}}{R + h}
$$

$$
F _ {6, 7} = - f _ {\mathrm{N}}
$$

$$
F _ {6, 8} = f _ {\mathrm{E}}
$$

$$
F _ {6, 1 6} = C _ {b} ^ {n} (3, 1)
$$

$$
F _ {6, 1 7} = C _ {b} ^ {n} (3, 2)
$$

$$
F _ {6, 1 8} = C _ {b} ^ {n} (3, 3)
$$

$$
F _ {6, 2 2} = C _ {b} ^ {n} (3, 1) f _ {x}
$$

$$
F _ {6, 2 3} = C _ {b} ^ {n} (3, 2) f _ {y}
$$

$$
F _ {6, 2 4} = C _ {b} ^ {n} (3, 3) f _ {z}
$$

$$
F _ {7, 3} = \frac {v _ {\mathrm{N}}}{(R + h) ^ {2}}
$$

$$
F _ {7, 5} = - \frac {1}{R + h}
$$

$$
F _ {7, 8} = \omega_ {i e} \sin L + \frac {v _ {\mathrm{E}}}{R + h} \tan L
$$

$$
F _ {7, 9} = - \left(\omega_ {i e} \cos L + \frac {v _ {\mathrm{E}}}{R + h}\right)
$$

$$
F _ {7, 1 0} = C _ {b} ^ {n} (1, 1)
$$

$$
F _ {7, 1 1} = C _ {b} ^ {n} (1, 2)
$$

$$
F _ {7, 1 2} = C _ {b} ^ {n} (1, 3)
$$

$$
F _ {7, 1 4} = C _ {b} ^ {n} (1, 2)
$$

$$
F _ {7, 1 3} = C _ {b} ^ {n} (1, 1)
$$

$$
F _ {7, 1 5} = C _ {b} ^ {n} (1, 3)
$$

$$
F _ {7, 1 9} = C _ {b} ^ {n} (1, 1) \omega_ {x}
$$

$$
F _ {7, 2 0} = C _ {b} ^ {n} (1, 2) \omega_ {y}
$$

$$
F _ {7, 2 1} = C _ {b} ^ {n} (1, 3) \omega_ {z}
$$

$$
F _ {8, 1} = - \omega_ {i e} \sin L
$$

$$
F _ {8, 3} = - \frac {v _ {\mathrm{E}}}{(R + h) ^ {2}}
$$

$$
F _ {8, 4} = \frac {1}{R + h}
$$

$$
F _ {8, 7} = - \left(\omega_ {i e} \sin L + \frac {v _ {\mathrm{E}}}{R + h} \tan L\right)
$$

$$
F _ {8. 9} = - \frac {v _ {\mathrm{N}}}{R + h}
$$

$$
F _ {8, 1 0} = C _ {b} ^ {n} (2, 1)
$$

$$
F _ {8, 1 1} = C _ {b} ^ {n} (2, 2)
$$

$$
F _ {8, 1 2} = C _ {b} ^ {n} (2, 3)
$$

$$
F _ {8, 1 3} = C _ {b} ^ {n} (2, 1)
$$

$$
F _ {8, 1 4} = C _ {b} ^ {n} (2, 2)
$$

$$
F _ {8, 1 5} = C _ {b} ^ {n} (2, 3)
$$

$$
F _ {8, 1 9} = C _ {b} ^ {n} (2, 1) \omega_ {x}
$$

$$
F _ {8, 2 0} = C _ {b} ^ {n} (2, 2) \omega_ {y}
$$

$$
F _ {8, 2 1} = C _ {b} ^ {n} (2, 3) \omega_ {z}
$$

$$
F _ {9, 1} = \omega_ {i e} \cos L + \frac {v _ {\mathrm{E}}}{R + h} \sec^ {2} L
$$

$$
F _ {9, 3} = - \frac {v _ {\mathrm{E}} \tan L}{(R + h) ^ {2}}
$$

$$
F _ {9, 4} = \frac {\tan L}{R + h}
$$

$$
F _ {9, 7} = \omega_ {i e} \cos L + \frac {v _ {\mathrm{E}}}{R + h}
$$

$$
F _ {9. 8} = \frac {v _ {\mathrm{N}}}{R + h}
$$

$$
F _ {9, 1 0} = C _ {b} ^ {n} (3, 1)
$$

$$
F _ {9, 1 1} = C _ {b} ^ {n} (3, 2)
$$

$$
F _ {9, 1 2} = C _ {b} ^ {n} (3, 3)
$$

$$
F _ {9, 1 3} = C _ {b} ^ {n} (3, 1)
$$

$$
F _ {9, 1 4} = C _ {b} ^ {n} (3, 2)
$$

$$
F _ {9, 1 5} = C _ {b} ^ {n} (3, 3)
$$

$$
F _ {9, 1 9} = C _ {b} ^ {n} (3, 1) \omega_ {x}
$$

$$
F _ {9, 2 0} = C _ {b} ^ {n} (3, 2) \omega_ {y}
$$

$$
F _ {9, 2 1} = C _ {b} ^ {n} (3, 3) \omega_ {z}
$$

$$
F _ {1 3, 1 3} = - \frac {1}{\tau_ {G}}
$$

$$
F _ {1 4, 1 4} = - \frac {1}{\tau_ {G}}
$$

$$
F _ {1 5, 1 5} = - \frac {1}{\tau_ {G}}
$$

(2) C/A 码 GPS 接收机。选择 C/A 码 GPS 接收机的状态为

$$
\boldsymbol {X} _ {\mathrm{GPS}} = \left[ \begin{array}{l l l l l l} \delta L _ {\mathrm{SA}} & \delta \lambda_ {\mathrm{SA}} & \delta h _ {\mathrm{SA}} & \delta v _ {\mathrm{ESA}} & \delta v _ {\mathrm{NSA}} & \delta v _ {\mathrm{USA}} \end{array} \right] ^ {\mathrm{T}}\tag{8.6.5}
$$

根据 8.3.1 小节中关于使用 C/A 码的 GPS 导航仪的介绍, 得 GPS 子系统的状态方程为

$$
\dot {\boldsymbol {X}} _ {\mathrm{GPS}} = \boldsymbol {F} _ {\mathrm{GPS}} \boldsymbol {X} _ {\mathrm{GPS}} + \boldsymbol {w} _ {\mathrm{GPS}}\tag{8.6.6}
$$

式中

$$
\boldsymbol {F} _ {\mathrm{GPS}} = \operatorname{diag} \left[ - \frac {1}{\tau_ {\mathrm{SAL}}} - \frac {1}{\tau_ {\mathrm{SA} \lambda}} - \frac {1}{\tau_ {\mathrm{SA} h}} - \frac {1}{\tau_ {\mathrm{SA} v E}} - \frac {1}{\tau_ {\mathrm{SA} v N}} - \frac {1}{\tau_ {\mathrm{SA} v U}} \right]\tag{8.6.7}
$$

$$
\boldsymbol {w} _ {\mathrm{GPS}} = \left[ \begin{array}{l l l l l l} w _ {\mathrm{SAL}} & w _ {\mathrm{SA} \lambda} & w _ {\mathrm{SA} h} & w _ {\mathrm{SA} v \mathrm{E}} & w _ {\mathrm{SA} v \mathrm{N}} & w _ {\mathrm{SA} v \mathrm{U}} \end{array} \right] ^ {\mathrm{T}}\tag{8.6.8}
$$

(3) 多普勒雷达测速仪。选择多普勒雷达的的测速误差为状态

$$
\boldsymbol {X} _ {\mathrm{DVS}} = \left[ \begin{array}{l l l l l l} \delta v _ {d x} & \delta v _ {d y} & \delta v _ {d z} & \delta K _ {d x} & \delta K _ {d y} & \delta K _ {d z} \end{array} \right] ^ {\mathrm{T}}\tag{8.6.9}
$$

式中， $\delta K_{di}(i = x, y, z)$ 为多普勒雷达测速的刻度系数误差，根据8.3.2节中分析，它们为随机常数，如式(8.3.27)所示。 $\delta v_{di}(i = x, y, z)$ 为由地形变化引起的测速误差。由于地形变化很少有突变，所以可用一阶马尔柯夫过程近似描述之。因此，状态方程为

$$
\dot {\boldsymbol {X}} _ {\mathrm{DVS}} = \boldsymbol {F} _ {\mathrm{DVS}} \boldsymbol {X} _ {\mathrm{DVS}} + \boldsymbol {G} _ {\mathrm{DVS}} \boldsymbol {w} _ {\mathrm{DVS}}\tag{8.6.10}
$$

式中

$$
\boldsymbol {F} _ {\mathrm{DVS}} = \operatorname{diag} \left[ - \frac {1}{\tau_ {d x}} - \frac {1}{\tau_ {d y}} - \frac {1}{\tau_ {d z}} 0 0 0 \right]\tag{8.6.11}
$$

$$
\boldsymbol {w} _ {\mathrm{DVS}} = \left[ \begin{array}{c c c} w _ {d x} & w _ {d y} & w _ {d z} \end{array} \right] ^ {\mathrm{T}}\tag{8.6.12}
$$

$$
\mathbf {G} _ {\mathrm{DVS}} = \left[ \begin{array}{c} \mathbf {I} _ {3 \times 3} \\ \mathbf {0} _ {3 \times 3} \end{array} \right]\tag{8.6.13}
$$

(4) 大气数据计算机。取大气数据计算机的状态为

$$
\boldsymbol {X} _ {\mathrm{ADS}} = \left[ v _ {u b \mathrm{E}} \quad v _ {u b \mathrm{N}} \quad v _ {u b \mathrm{U}} \quad v _ {u r \mathrm{E}} \quad v _ {w r \mathrm{N}} \quad v _ {w r \mathrm{U}} \quad \delta K _ {\mathrm{ADS}} \quad \delta h _ {b} \right] ^ {\mathrm{T}}\tag{8.6.14}
$$

式中， $v_{wbi}(i=\mathrm{E},\mathrm{N},\mathrm{U})$ 为随机常值风，它表示大气中的平稳气流，主要由地球的东西向环流引起； $v_{wri}(i=\mathrm{E},\mathrm{N},\mathrm{U})$ 为相关风，它表示变化较为缓慢的低频风； $\delta K_{ADS}$ 为空速测量中的刻度系数误差； $\delta h_{b}$ 为气压高度的测量误差，由于飞行过程中飞经各地的气压是不同的，所以假设它在时间上相关，相关时间为 $\tau_{b}$ 。这样，状态方程为

$$
\dot {\boldsymbol {X}} _ {\mathrm{ADS}} = \boldsymbol {F} _ {\mathrm{ADS}} \boldsymbol {X} _ {\mathrm{ADS}} + \boldsymbol {G} _ {\mathrm{ADS}} \boldsymbol {w} _ {\mathrm{ADS}}\tag{8.6.15}
$$

式中

$$
\boldsymbol {F} _ {\mathrm{ADS}} = \operatorname{diag} \left[ 0 \quad 0 \quad 0 \quad - \frac {1}{\tau_ {w E}} \quad - \frac {1}{\tau_ {w N}} \quad - \frac {1}{\tau_ {w U}} \quad 0 \quad - \frac {1}{\tau_ {b}} \right]
$$

$$
\boldsymbol {w} _ {\mathrm{ADS}} = \left[ \begin{array}{l l l l} w _ {w \mathrm{E}} & w _ {w \mathrm{N}} & w _ {w \mathrm{U}} & w _ {h b} \end{array} \right] ^ {\mathrm{T}}\tag{8.6.16}
$$

(8.6.17)

$$
\mathbf {G} _ {\mathrm{ADS}} = \left[ \begin{array}{c c c} \mathbf {0} _ {3 \times 4} & \\ \mathbf {I} _ {3 \times 3} & \mathbf {0} _ {3 \times 1} \\ \mathbf {0} _ {1 \times 3} & 1 \end{array} \right]\tag{8.6.18}
$$

(5) 垂直陀螺。取垂直陀螺的状态为

$$
\boldsymbol {X} _ {\mathrm{VG}} = \left[ \begin{array}{l l l l} \varphi_ {\mathrm{VG} x} & \varphi_ {\mathrm{VG} y} & \varepsilon_ {\mathrm{VG} x} & \varepsilon_ {\mathrm{VG} y} \end{array} \right] ^ {\mathrm{T}}\tag{8.6.19}
$$

式中， $\varphi_{VGx}$ 和 $\varphi_{VGy}$ 是沿俯仰轴和横滚轴的水平姿态测量误差； $\varepsilon_{VGx}$ 和 $\varepsilon_{VGy}$ 为垂直陀螺的漂移，用一阶马尔柯夫过程描述之。则垂直陀螺的状态方程为

$$
\dot {\boldsymbol {X}} _ {\mathrm{VG}} = \boldsymbol {F} _ {\mathrm{VG}} \boldsymbol {X} _ {\mathrm{VG}} + \boldsymbol {G} _ {\mathrm{VG}} \boldsymbol {w} _ {\mathrm{VG}}\tag{8.6.20}
$$

式中

$$
\boldsymbol {F} _ {\mathrm{VG}} = \left[ \begin{array}{c c c} & 1 & 0 \\ & 0 & 1 \\ \mathbf {0} _ {4 \times 2} & - \frac {1}{\tau_ {\mathrm{VG} x}} & 0 \\ & 0 & - \frac {1}{\tau_ {\mathrm{VG} y}} \end{array} \right]\tag{8.6.21}
$$

$$
\mathbf {G} _ {\mathrm{VG}} = \left[ \begin{array}{c} \mathbf {0} _ {2 \times 2} \\ \mathbf {I} _ {2 \times 2} \end{array} \right]\tag{8.6.22}
$$

$$
\boldsymbol {w} _ {\mathrm{VG}} = \left[ \begin{array}{l l} w _ {\mathrm{VG} x} & w _ {\mathrm{VG} y} \end{array} \right] ^ {\mathrm{T}}\tag{8.6.23}
$$

(6) 磁航向仪。磁航向仪只取一个状态

$$
X _ {\mathrm{MG}} = \delta \Psi_ {\mathrm{MG}}\tag{8.6.24}
$$

由飞行引起的沿途各地的磁航向误差用一阶马尔柯夫过程描述之

$$
\dot {X} _ {\mathrm{MG}} = - \frac {1}{\tau_ {\mathrm{MG}}} X _ {\mathrm{MG}} + w _ {\mathrm{MG}}\tag{8.6.25}
$$

## 2. 量测方程的建立

将捷联惯导分别与其余子系统形成的所有量测量作为集中滤波器的量测量。这样做的目的是:充分利用高精度导航子系统的信息来校正惯导,同时又使惯导对精度较低的子系统作标定和校正,为一旦惯导失效而只能使用其余导航系统时的应急导航方案作准备。

（1）惯导与 GPS 形成的量测量。将 GPS 导航仪输出的位置和速度信息与惯导的相应输出信息相减得量测方程为

$$
\mathbf {Z} _ {\mathrm{GPS}} = \left[ \begin{array}{l} L _ {\mathrm{INS}} - L _ {\mathrm{GPS}} \\ \lambda_ {\mathrm{INS}} - \lambda_ {\mathrm{GPS}} \\ h _ {\mathrm{INS}} - h _ {\mathrm{GPS}} \\ v _ {\mathrm{EINS}} - v _ {\mathrm{EGPS}} \\ v _ {\mathrm{NINS}} - v _ {\mathrm{NGPS}} \\ v _ {\mathrm{UINS}} - v _ {\mathrm{UGPS}} \end{array} \right] = \left[ \begin{array}{l} \delta L - \delta L _ {\mathrm{SA}} + V _ {1} \\ \delta \lambda - \delta \lambda_ {\mathrm{SA}} + V _ {2} \\ \delta h - \delta h _ {\mathrm{SA}} + V _ {3} \\ \delta v _ {\mathrm{E}} - \delta v _ {\mathrm{ESA}} + V _ {4} \\ \delta v _ {\mathrm{N}} - \delta v _ {\mathrm{NSA}} + V _ {5} \\ \delta v _ {\mathrm{U}} - \delta v _ {\mathrm{USA}} + V _ {6} \end{array} \right]\tag{8.6.26}
$$

即

$$
\mathbf {Z} _ {\mathrm{GPS}} = \left[ \begin{array}{l l} \mathbf {H} _ {1} & \mathbf {H} _ {\mathrm{GPS}} \end{array} \right] \left[ \begin{array}{l} \mathbf {X} _ {\mathrm{INS}} \\ \mathbf {X} _ {\mathrm{GPS}} \end{array} \right] + \mathbf {V} _ {\mathrm{GPS}}
$$

(8.6.27)

式中

$$
\pmb {H} _ {1} = \left[ \begin{array}{l l} \pmb {I} _ {6 \times 6} & \pmb {0} _ {6 \times 1 8} \end{array} \right]\tag{8.6.28}
$$

$$
\boldsymbol {H} _ {\mathrm{GPS}} = \left[ - \boldsymbol {I} _ {6 \times 6} \right]\tag{8.6.29}
$$

（2）惯导与多普勒雷达测速仪(DVS)形成的量测量。四波束多普勒雷达可测得地速在机体坐标系内的分量，要使其输出的速度与惯导的输出速度形成量测量，必须将DVS的输出速度变换到导航坐标系中(此处导航坐标系取地理坐标系)。假设变换中所用的姿态矩阵 $C_{b}^{n'}$ 由惯导提供，则

$$
\begin{array}{r l} \boldsymbol {v} _ {\mathrm{DVS}} ^ {n ^ {\prime}} = & \boldsymbol {C} _ {n} ^ {n ^ {\prime}} \boldsymbol {C} _ {b} ^ {n} (\boldsymbol {v} ^ {b} + \delta \boldsymbol {v} _ {\mathrm{DVS}} ^ {b}) = \\ & \boldsymbol {C} _ {n} ^ {n ^ {\prime}} \boldsymbol {v} ^ {n} + \boldsymbol {C} _ {n} ^ {n ^ {\prime}} \boldsymbol {C} _ {b} ^ {n} \left[ \begin{array}{l} \delta v _ {d x} + \delta K _ {d x} v _ {x} \\ \delta v _ {d y} + \delta K _ {d y} v _ {y} \\ \delta v _ {d z} + \delta K _ {d z} v _ {z} \end{array} \right] \end{array}
$$

式中

$$
\boldsymbol {C} _ {n} ^ {n ^ {\prime}} = \left[ \begin{array}{c c c} 1 & \varphi_ {\mathrm{U}} & - \varphi_ {\mathrm{N}} \\ - \varphi_ {\mathrm{U}} & 1 & \varphi_ {\mathrm{E}} \\ \varphi_ {\mathrm{N}} & - \varphi_ {\mathrm{E}} & 1 \end{array} \right], \quad \boldsymbol {v} ^ {b} = \left[ \begin{array}{l} v _ {x} \\ v _ {y} \\ v _ {z} \end{array} \right]
$$

略去由误差量构成的二阶小量,得多普勒测速仪输出的地速在地理坐标系内的分量

$$
\begin{array}{r l} v _ {\mathrm{EDVS}} & \approx v _ {\mathrm{E}} + \varphi_ {\mathrm{U}} v _ {\mathrm{N}} - \varphi_ {\mathrm{N}} v _ {\mathrm{U}} + C _ {b} ^ {n} (1, 1) \delta v _ {d x} + c _ {b} ^ {n} (1, 2) \delta v _ {d y} + \\ & \quad C _ {b} ^ {n} (1, 3) \delta v _ {d z} + C _ {b} ^ {n} (1, 1) \delta K _ {d x} v _ {x} + C _ {b} ^ {n} (1, 2) \delta K _ {d y} v _ {y} + C _ {b} ^ {n} (1, 3) \delta K _ {d z} v _ {z} \end{array}\tag{8.6.30a}
$$

$$
v _ {\mathrm{NDVS}} \approx v _ {\mathrm{N}} - \varphi_ {\mathrm{U}} v _ {\mathrm{E}} + \varphi_ {\mathrm{E}} v _ {\mathrm{U}} + C _ {b} ^ {n} (2, 1) \delta v _ {d x} + C _ {b} ^ {n} (2, 2) \delta v _ {d y} +
$$

$$
\begin{array}{r l} & C _ {b} ^ {n} (2, 3) \delta v _ {d z} + C _ {b} ^ {n} (2, 1) \delta K _ {d x} v _ {x} + C _ {b} ^ {n} (2, 2) \delta K _ {d y} v _ {y} + C _ {b} ^ {n} (2, 3) \delta K _ {d z} v _ {z} \\ & v _ {\mathrm{UDVS}} \approx v _ {\mathrm{U}} + \varphi_ {\mathrm{N}} v _ {\mathrm{E}} - \varphi_ {\mathrm{E}} v _ {\mathrm{N}} + C _ {b} ^ {n} (3, 1) \delta v _ {d x} + C _ {b} ^ {n} (3, 2) \delta v _ {d y} + \\ & C _ {b} ^ {n} (3, 3) \delta v _ {d z} + C _ {b} ^ {n} (3, 1) \delta K _ {d x} v _ {x} + C _ {b} ^ {n} (3, 2) \delta K _ {d y} v _ {y} + C _ {b} ^ {n} (3, 3) \delta K _ {d z} v _ {z} \end{array} \tag {8.6.30b}\tag{8.6.30c}
$$

由多普勒雷达和惯导形成的量测量为

$$
\mathbf {Z} _ {\mathrm{DVS}} = \left[ \begin{array}{l} Z _ {\mathrm{DVS1}} \\ Z _ {\mathrm{DVS2}} \\ Z _ {\mathrm{DVS3}} \end{array} \right] = \left[ \begin{array}{l} v _ {\mathrm{EINS}} - v _ {\mathrm{EDVS}} \\ v _ {\mathrm{NINS}} - v _ {\mathrm{NDVS}} \\ v _ {\mathrm{UINS}} - v _ {\mathrm{UDVS}} \end{array} \right]
$$

将式(8.6.30)代入得

$$
\begin{array}{r l} & Z _ {\mathrm{DVS1}} = \delta v _ {\mathrm{E}} - \varphi_ {\mathrm{U}} v _ {\mathrm{N}} + \varphi_ {\mathrm{N}} v _ {\mathrm{U}} - C _ {b} ^ {n} (1, 1) \delta v _ {d x} - C _ {b} ^ {n} (1, 2) \delta v _ {d y} - \\ & \qquad C _ {b} ^ {n} (1, 3) \delta v _ {d z} - C _ {b} ^ {n} (1, 1) \delta K _ {d x} v _ {x} - C _ {b} ^ {n} (1, 2) \delta K _ {d y} v _ {y} - C _ {b} ^ {n} (1, 3) \delta K _ {d z} v _ {z} + V _ {7} \\ & Z _ {\mathrm{DVS2}} = \delta v _ {\mathrm{N}} + \varphi_ {\mathrm{U}} v _ {\mathrm{E}} - \varphi_ {\mathrm{E}} v _ {\mathrm{U}} - C _ {b} ^ {n} (2, 1) \delta v _ {d x} - C _ {b} ^ {n} (2, 2) \delta v _ {d y} - \\ & \qquad C _ {b} ^ {n} (2, 3) \delta v _ {d z} - C _ {b} ^ {n} (2, 1) \delta K _ {d x} v _ {x} - C _ {b} ^ {n} (2, 2) \delta K _ {d y} v _ {y} - C _ {b} ^ {n} (2, 3) \delta K _ {d z} v _ {z} + V _ {8} \\ & Z _ {\mathrm{DVS3}} = \delta v _ {\mathrm{U}} - \varphi_ {\mathrm{N}} v _ {\mathrm{E}} + \varphi_ {\mathrm{E}} v _ {\mathrm{N}} - C _ {b} ^ {n} (3, 1) \delta v _ {d x} - C _ {b} ^ {n} (3, 2) \delta v _ {d y} - \\ & \qquad C _ {b} ^ {n} (3, 3) \delta v _ {d z} - C _ {b} ^ {n} (3, 1) \delta K _ {d x} v _ {x} - C _ {b} ^ {n} (3, 2) \delta K _ {d y} v _ {y} - C _ {b} ^ {n} (3, 3) \delta K _ {d z} v _ {z} + V _ {9} \end{array}
$$

所以量测方程为

$$
\mathbf {Z} _ {\mathrm{DVS}} = \left[ \begin{array}{l l} \mathbf {H} _ {2} & \mathbf {H} _ {\mathrm{DVS}} \end{array} \right] \left[ \begin{array}{l} \mathbf {X} _ {\mathrm{INS}} \\ \mathbf {X} _ {\mathrm{DVS}} \end{array} \right] + \mathbf {V} _ {\mathrm{DVS}}\tag{8.6.31}
$$

式中

$$
\begin{array}{r l} \boldsymbol {H} _ {2} & = \left[ \begin{array}{c c c c c} & 0 & v _ {\mathrm{U}} & - v _ {\mathrm{N}} \\ \mathbf {0} _ {3 \times 3} & I _ {3 \times 3} & - v _ {\mathrm{U}} & 0 & v _ {\mathrm{E}} & \mathbf {0} _ {3 \times 1 5} \\ & & v _ {\mathrm{N}} & - v _ {\mathrm{E}} & 0 \end{array} \right] \\ \boldsymbol {H} _ {\mathrm{DVS}} & = \left[ \begin{array}{c c c c c} & - C _ {b} ^ {n} (1, 1) v _ {x} & - C _ {b} ^ {n} (1, 2) v _ {y} & - C _ {b} ^ {n} (1, 3) v _ {z} \\ - C _ {b} ^ {n} & - C _ {b} ^ {n} (2, 1) v _ {x} & - C _ {b} ^ {n} (2, 2) v _ {y} & - C _ {b} ^ {n} (2, 3) v _ {z} \\ & - C _ {b} ^ {n} (3, 1) v _ {x} & - C _ {b} ^ {n} (3, 2) v _ {y} & - C _ {b} ^ {n} (3, 3) v _ {z} \end{array} \right] \end{array}\tag{8.6.32}
$$

(8.6.33)

（3）惯导与大气数据系统(ADS)形成的量测量。大气数据系统输出飞机沿机体纵轴的空速，空速与地速相差风速 $v_{w}$ 。

设测量空速时的刻度系数误差为 $\delta K_{ADS}$ ，由机体坐标系到地理坐标系的姿态矩阵由惯导提供，则大气数据系统的输出在地理坐标系内的分量为

$$
\begin{array}{r l} \boldsymbol {v} _ {\mathrm{ADS}} ^ {n ^ {\prime}} = & \boldsymbol {C} _ {b} ^ {n ^ {\prime}} (1 + \delta K _ {\mathrm{ADS}}) (\boldsymbol {v} ^ {b} + \boldsymbol {v} _ {w} ^ {b}) = \boldsymbol {C} _ {n} ^ {n ^ {\prime}} (1 + \delta K _ {\mathrm{ADS}}) (\boldsymbol {v} ^ {n} + \boldsymbol {v} _ {w} ^ {n}) = \\ & \left[ \begin{array}{c c c} 1 & \varphi_ {\mathrm{U}} & - \varphi_ {\mathrm{N}} \\ - \varphi_ {\mathrm{U}} & 1 & \varphi_ {\mathrm{E}} \\ \varphi_ {\mathrm{N}} & - \varphi_ {\mathrm{E}} & 1 \end{array} \right] (1 + \delta K _ {\mathrm{ADS}}) \left[ \begin{array}{c} v _ {\mathrm{E}} + v _ {w b \mathrm{E}} + v _ {w r \mathrm{E}} \\ v _ {\mathrm{N}} + v _ {w b \mathrm{N}} + v _ {w r \mathrm{N}} \\ v _ {\mathrm{U}} + v _ {w b \mathrm{U}} + v _ {w r \mathrm{U}} \end{array} \right] \end{array}
$$

略去姿态误差与刻度系数误差间形成的二阶小量，并且，为避免非线性滤波，略去风速与姿态误差角及刻度系数误差间的乘积项得

$$
\begin{array}{r l} & v _ {\mathrm{EADS}} \approx v _ {\mathrm{E}} + \delta K _ {\mathrm{ADS}} v _ {\mathrm{E}} + v _ {u b \mathrm{E}} + v _ {u r \mathrm{E}} + \varphi_ {\mathrm{U}} v _ {\mathrm{N}} - \varphi_ {\mathrm{N}} v _ {\mathrm{U}} \\ & v _ {\mathrm{NADS}} \approx v _ {\mathrm{N}} + \delta K _ {\mathrm{ADS}} v _ {\mathrm{N}} + v _ {u b \mathrm{N}} + v _ {w r \mathrm{N}} - \varphi_ {\mathrm{U}} v _ {\mathrm{E}} + \varphi_ {\mathrm{E}} v _ {\mathrm{U}} \end{array}
$$

$$
v _ {\mathrm{UADS}} \approx v _ {\mathrm{U}} + \delta K _ {\mathrm{ADS}} v _ {\mathrm{U}} + v _ {w b \mathrm{U}} + v _ {w r \mathrm{U}} + \varphi_ {\mathrm{N}} v _ {\mathrm{E}} - \varphi_ {\mathrm{E}} v _ {\mathrm{N}}
$$

由惯导和大气数据系统形成的量测量为

$$
\begin{array}{r l} \mathbf {Z} _ {\mathrm{ADS}} & = \left[ \begin{array}{l} v _ {\mathrm{EINS}} - v _ {\mathrm{EADS}} \\ v _ {\mathrm{NINS}} - v _ {\mathrm{NADS}} \\ v _ {\mathrm{UINS}} - v _ {\mathrm{UADS}} \\ h _ {\mathrm{INS}} - h _ {\mathrm{ADS}} \end{array} \right] = \\ & \left[ \begin{array}{l} \delta v _ {\mathrm{E}} - \delta K _ {\mathrm{ADS}} v _ {\mathrm{E}} - v _ {w b \mathrm{E}} - v _ {w r \mathrm{E}} - \varphi_ {\mathrm{U}} v _ {\mathrm{N}} + \varphi_ {\mathrm{N}} v _ {\mathrm{U}} + V _ {1 0} \\ \delta v _ {\mathrm{N}} - \delta K _ {\mathrm{ADS}} v _ {\mathrm{N}} - v _ {w b \mathrm{N}} - v _ {w r \mathrm{N}} + \varphi_ {\mathrm{U}} v _ {\mathrm{E}} - \varphi_ {\mathrm{E}} v _ {\mathrm{U}} + V _ {1 1} \\ \delta v _ {\mathrm{U}} - \delta K _ {\mathrm{ADS}} v _ {\mathrm{U}} - v _ {w b \mathrm{U}} - v _ {w r \mathrm{U}} - \varphi_ {\mathrm{N}} v _ {\mathrm{E}} + \varphi_ {\mathrm{E}} v _ {\mathrm{N}} + V _ {1 2} \\ \delta h - \delta h _ {b} + V _ {1 3} \end{array} \right] \end{array}\tag{8.6.34}
$$

所以量测方程为

$$
\mathbf {Z} _ {\mathrm{ADS}} = \left[ \begin{array}{l l} \mathbf {H} _ {3} & \mathbf {H} _ {\mathrm{ADS}} \end{array} \right] \left[ \begin{array}{l} \mathbf {X} _ {\mathrm{INS}} \\ \mathbf {X} _ {\mathrm{ADS}} \end{array} \right] + \mathbf {V} _ {\mathrm{ADS}}\tag{8.6.35}
$$

式中

$$
\boldsymbol {H} _ {3} = \left[ \begin{array}{c c c c c c c c} & & & 0 & v _ {\mathrm{U}} & - v _ {\mathrm{N}} \\ \mathbf {0} _ {3 \times 3} & I _ {3 \times 3} & - v _ {\mathrm{U}} & 0 & v _ {\mathrm{E}} & \mathbf {0} _ {4 \times 5} \\ \hline 0 & 0 & 1 & 0 & 0 & 0 & v _ {\mathrm{N}} & - v _ {\mathrm{E}} & 0 \end{array} \right]\tag{8.6.36}
$$

$$
\boldsymbol {H} _ {\mathrm{ADS}} = \left[ \begin{array}{c c c c} & & - v _ {\mathrm{E}} & 0 \\ \boldsymbol {I} _ {3 \times 3} & \boldsymbol {I} _ {3 \times 3} & - v _ {\mathrm{N}} & 0 \\ \hdashline & & - v _ {\mathrm{U}} & 0 \\ & \mathbf {0} _ {1 \times 6} & 0 & - 1 \end{array} \right]\tag{8.6.37}
$$

（4）惯导与垂直陀螺形成的量测量。设惯导给出的载体俯仰角和横滚角分别为 $\theta_{INS}$ 和 $\gamma_{INS}$ ，垂直陀螺给出的载体俯仰角和横滚角分别为 $\theta_{VG}$ 和 $\gamma_{VG}$ ，其中

$$
\begin{array}{r l} & {\left[ \begin{array}{l} \theta_ {\mathrm{VG}} \\ \gamma_ {\mathrm{VG}} \end{array} \right] = \left[ \begin{array}{l} \theta \\ \gamma \end{array} \right] + \left[ \begin{array}{l} \varphi_ {\mathrm{VG} _ {x}} \\ \varphi_ {\mathrm{VG} _ {y}} \end{array} \right]} \\ & {\left[ \begin{array}{l} \theta_ {\mathrm{INS}} \\ \gamma_ {\mathrm{INS}} \end{array} \right] = \left[ \begin{array}{l} \theta \\ \gamma \end{array} \right] + \left[ \begin{array}{c c} \cos \Psi & - \sin \Psi \\ \sin \Psi & \cos \Psi \end{array} \right] \left[ \begin{array}{l} \varphi_ {\mathrm{E}} \\ \varphi_ {\mathrm{N}} \end{array} \right]} \end{array}
$$

诸式中， $\theta$ 和 $\gamma$ 为载体俯仰角和横滚的真实值； $\varphi_{\mathrm{VG}_x}$ 和 $\varphi_{\mathrm{VG}_y}$ 为垂直陀螺的测量误差； $\Psi$ 为载体的航向角； $\varphi_{\mathrm{E}}$ 和 $\varphi_{\mathrm{N}}$ 为捷联惯导数学平台沿东向和北向的水平倾角。则量测量为

$$
\mathbf {Z} _ {\mathrm{VG}} = \left[ \begin{array}{l} \theta_ {\mathrm{INS}} - \theta_ {\mathrm{VG}} \\ \gamma_ {\mathrm{INS}} - \gamma_ {\mathrm{VG}} \end{array} \right] = \left[ \begin{array}{l} \cos \Psi \varphi_ {\mathrm{E}} - \sin \Psi \varphi_ {\mathrm{N}} - \varphi_ {\mathrm{VG} x} + V _ {1 4} \\ \sin \Psi \varphi_ {\mathrm{E}} + \cos \Psi \varphi_ {\mathrm{N}} - \varphi_ {\mathrm{VG} y} + V _ {1 5} \end{array} \right]\tag{8.6.38}
$$

所以量测方程为

$$
\mathbf {Z} _ {\mathrm{VG}} = \left[ \begin{array}{l l} \mathbf {H} _ {4} & \mathbf {H} _ {\mathrm{VG}} \end{array} \right] \left[ \begin{array}{l} \mathbf {X} _ {\mathrm{INS}} \\ \mathbf {X} _ {\mathrm{VG}} \end{array} \right] + \mathbf {V} _ {\mathrm{VG}}\tag{8.6.39}
$$

式中

$$
\boldsymbol {H} _ {4} = \left[ \begin{array}{c c c c} & \cos \Psi & - \sin \Psi & \\ \mathbf {0} _ {2 \times 6} & & & \mathbf {0} _ {2 \times 1 6} \\ & \sin \Psi & \cos \Psi & \end{array} \right]\tag{8.6.40}
$$

$$
\mathbf {H} _ {\mathrm{VG}} = [ - \mathbf {I} _ {2 \times 2} \quad \mathbf {0} _ {2 \times 2} ]\tag{8.6.41}
$$

（5）惯导与磁航向仪形成的量测量。由惯导输出的航向信息 $\Psi_{INS}$ 与磁航向仪输出的航向信息 $\Psi_{MG}$ 形成的量测量为

$$
Z _ {\mathrm{MG}} = \Psi_ {\mathrm{INS}} - \Psi_ {\mathrm{MG}} = \varphi_ {\mathrm{U}} - \delta \Psi_ {\mathrm{MG}} + V _ {1 6}\tag{8.6.42}
$$

所以量测方程为

$$
Z _ {\mathrm{MG}} = \left[ \begin{array}{l l} \boldsymbol {H} _ {5} & H _ {\mathrm{MG}} \end{array} \right] \left[ \begin{array}{l} \boldsymbol {X} _ {\mathrm{INS}} \\ X _ {\mathrm{MG}} \end{array} \right] + V _ {\mathrm{MG}}\tag{8.6.43}
$$

式中

$$
\begin{array}{r l} \mathbf {H} _ {5} & = [ \mathbf {0} _ {1 \times 8} \quad 1 \quad \mathbf {0} _ {1 \times 1 5} ] \\ & H _ {\mathrm{MG}} = - 1 \end{array}\tag{8.6.44}
$$

(8.6.45)

3. 集中滤波器的设计

取状态向量为

$$
\boldsymbol {X} = \left[ \begin{array}{l l l l l l} \boldsymbol {X} _ {\mathrm{INS}} ^ {\mathrm{T}} & \boldsymbol {X} _ {\mathrm{GPS}} ^ {\mathrm{T}} & \boldsymbol {X} _ {\mathrm{DVS}} ^ {\mathrm{T}} & \boldsymbol {X} _ {\mathrm{ADS}} ^ {\mathrm{T}} & \boldsymbol {X} _ {\mathrm{VG}} ^ {\mathrm{T}} & \boldsymbol {X} _ {\mathrm{MG}} \end{array} \right] ^ {\mathrm{T}}\tag{8.6.46}
$$

共49维。则状态方程为

式中

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {F} (t) \boldsymbol {X} (t) + \boldsymbol {G} (t) \boldsymbol {w} (t)\tag{8.6.47}
$$

(8.6.48)

$$
\begin{array}{r l} \boldsymbol {F} (t) & = \left[ \begin{array}{c c c c c c} \boldsymbol {F} _ {\mathrm{INS}} & & & & & \boldsymbol {0} \\ & \boldsymbol {F} _ {\mathrm{GPS}} & & & & \\ & & \boldsymbol {F} _ {\mathrm{DVS}} & & & \\ & & & \boldsymbol {F} _ {\mathrm{ADS}} & & \\ & & & & \boldsymbol {F} _ {\mathrm{VG}} & \\ \boldsymbol {0} & & & & & F _ {\mathrm{MG}} \end{array} \right] \\ \boldsymbol {G} (t) & = \left[ \begin{array}{c c c c c c} \boldsymbol {G} _ {\mathrm{INS}} & & & & & \boldsymbol {0} \\ & \boldsymbol {I} _ {6 \times 6} & & & & \\ & & \boldsymbol {G} _ {\mathrm{DVS}} & & & \\ & & & \boldsymbol {G} _ {\mathrm{ADS}} & & \\ & & & & \boldsymbol {G} _ {\mathrm{VG}} \\ \boldsymbol {0} & & & & & 1 \end{array} \right] \\ \boldsymbol {w} (t) & = [ \boldsymbol {w} _ {\mathrm{INS}} ^ {\mathrm{T}} \quad \boldsymbol {w} _ {\mathrm{GPS}} ^ {\mathrm{T}} \quad \boldsymbol {w} _ {\mathrm{DVS}} ^ {\mathrm{T}} \quad \boldsymbol {w} _ {\mathrm{ADS}} ^ {\mathrm{T}} \quad \boldsymbol {w} _ {\mathrm{VG}} ^ {\mathrm{T}} \quad \boldsymbol {w} _ {\mathrm{MG}} ] ^ {\mathrm{T}} \end{array}\tag{8.6.49}
$$

(8.6.50)

量测方程为

$$
\mathbf {Z} = \mathbf {H X} + \mathbf {V}\tag{8.6.51}
$$

式中

$$
\mathbf {Z} = \left[ \begin{array}{l} \mathbf {Z} _ {\mathrm{GPS}} \\ \mathbf {Z} _ {\mathrm{DVS}} \\ \mathbf {Z} _ {\mathrm{ADS}} \\ \mathbf {Z} _ {\mathrm{VG}} \\ \mathbf {Z} _ {\mathrm{MG}} \end{array} \right]\tag{8.6.52}
$$

$$
\boldsymbol {V} = \left[ \begin{array}{l} \boldsymbol {V} _ {\mathrm{GPS}} \\ \boldsymbol {V} _ {\mathrm{DVS}} \\ \boldsymbol {V} _ {\mathrm{ADS}} \\ \boldsymbol {V} _ {\mathrm{VG}} \\ \boldsymbol {V} _ {\mathrm{MG}} \end{array} \right]\tag{8.6.53}
$$

$$
\boldsymbol {H} = \left[ \begin{array}{c c c c c c} \boldsymbol {H} _ {1} & \boldsymbol {H} _ {\mathrm{GPS}} & & & & \\ \boldsymbol {H} _ {2} & & \boldsymbol {H} _ {\mathrm{DVS}} & & \boldsymbol {0} & \\ \boldsymbol {H} _ {3} & & & \boldsymbol {H} _ {\mathrm{ADS}} & & \\ \boldsymbol {H} _ {4} & & \boldsymbol {0} & & \boldsymbol {H} _ {\mathrm{VG}} & \\ \boldsymbol {H} _ {5} & & & & & H _ {\mathrm{MG}} \end{array} \right]\tag{8.6.54}
$$

$$
\mathbf {R} _ {\mathrm{GPS}} = \operatorname{diag} \left[ (0. 0 0 1 ^ {\prime}) ^ {2} \quad (0. 0 0 1 ^ {\prime}) ^ {2} \quad (1 0 \mathrm{m}) ^ {2} \right.
$$

$$
(0. 0 4 \mathrm{m} / \mathrm{s}) ^ {2} (0. 0 4 \mathrm{m} / \mathrm{s}) ^ {2} (0. 0 4 \mathrm{m} / \mathrm{s}) ^ {2} ]\tag{8.6.55}
$$

$$
\boldsymbol {R} _ {\mathrm{DVS}} = \operatorname{diag} \left[ (0. 0 5 \mathrm{m/s}) ^ {2} \quad (0. 0 5 \mathrm{m/s}) ^ {2} \quad (0. 0 5 \mathrm{m/s}) ^ {2} \right]\tag{8.6.56}
$$

$$
\mathbf {R} _ {\mathrm{ADS}} = \operatorname{diag} \left[ (0. 1 \mathrm{m} / \mathrm{s}) ^ {2} \quad (0. 1 \mathrm{m} / \mathrm{s}) ^ {2} (0. 1 \mathrm{m} / \mathrm{s}) ^ {2} \quad (1 0 \mathrm{m}) ^ {2} \right]\tag{8.6.57}
$$

$$
\begin{array}{r l} & {\pmb {R} _ {\mathrm{VG}} = \mathrm{diag} [ (2 ^ {\prime}) ^ {2} \quad (2 ^ {\prime}) ^ {2} ]} \\ & {\pmb {R} _ {\mathrm{MG}} = (1 ^ {\prime}) ^ {2}} \end{array}\tag{8.6.58}
$$

(8.6.59)

各子系统驱动白噪声的方差强度阵取值如下。

(1) 激光陀螺捷联惯导系统：

$$
\boldsymbol {q} _ {\text {INS}} = \operatorname{diag} \left[ q _ {g x} \quad q _ {g y} \quad q _ {g z} \quad q _ {r x} \quad q _ {r y} \quad q _ {r z} \quad q _ {a x} \quad q _ {a y} \quad q _ {a z} \right]\tag{8.6.60}
$$

式中

$$
q _ {n} = \frac {2 R _ {n}}{\tau_ {G i}}\tag{8.6.61}
$$

对于激光陀螺， $\tau_{Gi}=300\ s,\ R_{ni}=(0.1^{\circ}/h)^{2}$ ；随机游走系数 $q_{gi}=(0.1^{\circ}/\sqrt{h})^{2}$ 。

$$
q _ {a i} = \frac {2 R _ {a i}}{\tau_ {A i}}\tag{8.6.62}
$$

对于摆式加速度计 $\tau_{Ai}=2\sim3\ h,\ R_{ai}=(10^{-4}\ g)^{2}$ 。上述诸式中，i=x，y，z。

(2) C/A 码 GPS 接收机：

$$
\boldsymbol {q} _ {\mathrm{GPS}} = \operatorname{diag} \left[ q _ {\mathrm{SAL}} \quad q _ {\mathrm{SA} \lambda} \quad q _ {\mathrm{SAh}} \quad q _ {\mathrm{SAvE}} \quad q _ {\mathrm{SAvN}} \quad q _ {\mathrm{SAvU}} \right]\tag{8.6.63}
$$

式中

$$
\begin{array}{r l} & q _ {i} = \frac {2 R _ {i}}{\tau_ {i}} \\ & \tau_ {i} = 1 0 0 \sim 2 0 0 \mathrm{s}, \quad R _ {\mathrm{SAh}} = (5 0 \mathrm{m}) ^ {2} \\ & \quad R _ {\mathrm{SAL}} = R _ {\mathrm{SA} \lambda} = (0. 0 2 ^ {\prime}) ^ {2} \\ & R _ {\mathrm{SAvE}} = R _ {\mathrm{SAvN}} = R _ {\mathrm{SAvU}} = (0. 2 5 \mathrm{m/s}) ^ {2} \end{array}\tag{8.6.64}
$$

上述诸式中， $i = \text{SAL}$ ， $\text{SA}\lambda$ ， $\text{SAh}$ ， $\text{SAvE}$ ， $\text{SAvN}$ ， $\text{SAvU}$ 。

(3) 多普勒雷达：

$$
\boldsymbol {q} _ {\mathrm{DVS}} = \operatorname{diag} \left[ q _ {d x} \quad q _ {d y} \quad q _ {d z} \right]\tag{8.6.65}
$$

式中

$$
q _ {d i} = \frac {2 R _ {d i}}{\tau_ {d i}}\tag{8.6.66}
$$

取 $\tau_{di}=300\ s,\ R_{di}=(0.1\ m/s)^{2}$ 。上述诸式中，i=x，y，z。

(4) 大气数据系统：

$$
\boldsymbol {q} _ {\mathrm{DVS}} = \operatorname{diag} \left[ q _ {w x} \quad q _ {w y} \quad q _ {w z} \quad q _ {h b} \right]\tag{8.6.67}
$$

式中

$$
q _ {i} = \frac {2 R _ {i}}{\tau_ {i}} \quad (i = w E, w N, w U, h b)\tag{8.6.68}
$$

取

$$
\begin{array}{r l} & {\tau_ {w \mathrm{E}} = \tau_ {w \mathrm{N}} = \tau_ {w \mathrm{U}} = 3 0 \mathrm{s}} \\ & {R _ {w \mathrm{E}} = (5 \mathrm{m/s}) ^ {2}, R _ {w \mathrm{N}} = (3 \mathrm{m/s}) ^ {2}, R _ {w \mathrm{U}} = (1 \mathrm{m/s}) ^ {2}} \\ & {\tau_ {h b} = 1 0 0 0 \mathrm{s}, R _ {h b} = (5 0 \mathrm{m}) ^ {2}.} \end{array}
$$

(5) 垂直陀螺仪：

$$
\boldsymbol {q} _ {\mathrm{VG}} = \operatorname{diag} \left[ q _ {\mathrm{VGx}} \quad q _ {\mathrm{VGy}} \right]\tag{8.6.69}
$$

式中

$$
q _ {\mathrm{VGi}} = \frac {2 R _ {\mathrm{VGi}}}{\tau_ {\mathrm{VGi}}}\tag{8.6.70}
$$

取

$$
\tau_ {\mathrm{VG} i} = 1 0 0 0 \mathrm{s}, R _ {\mathrm{VG} i} = (1 0 ^ {\circ} / \mathrm{h}) ^ {2}
$$

上述诸式中， $i = x$ ， $y_{\circ}$

(6) 磁航向仪：

$$
q _ {\mathrm{MG}} = \frac {2 R _ {\mathrm{MG}}}{\tau_ {\mathrm{MG}}}\tag{8.6.71}
$$

取 $\tau_{MG}=30\ s,\ R_{MG}=(30')^{2}$

卡尔曼滤波器按照基本方程式(2.2.4)设计。49个状态的初始估值都取零。估计的均方误差阵初值 $P_{0}$ 取值如下。

假设捷联惯导系统采用自对准，并且对准时间充分，对准能达到极限精度

$$
\left\{ \begin{array}{l} \varphi_ {\mathrm{Emin}} = - \frac {\nabla_ {\mathrm{N}}}{g} \\ \varphi_ {\mathrm{Nmin}} = \frac {\nabla_ {\mathrm{E}}}{g} \\ \varphi_ {\mathrm{Umin}} = \frac {\varepsilon_ {\mathrm{E}}}{\omega_ {i c} \cos L} \end{array} \right.
$$

式中 $\nabla_{E}$ 和 $\nabla_{N}$ 分别为等效东向和北向加速度计误差， $\varepsilon_{E}$ 为等效东向陀螺漂移。

根据上述三式及其余各子系统的特性,可得 $P_{0}$ 取值

$$
\begin{array}{r l} & P _ {0} (1, 1) = E [ \delta L ^ {2} ] = (1 ^ {\prime}) ^ {2} \\ & P _ {0} (2, 2) = E [ \delta \lambda^ {2} ] = (1 ^ {\prime}) ^ {2} \\ & P _ {0} (3, 3) = E [ \delta h ^ {2} ] = (1 0 0 \mathrm{m}) ^ {2} \\ & P _ {0} (4, 4) = E [ \delta v _ {\mathrm{E}} ^ {2} ] = 0 \\ & P _ {0} (5, 5) = E [ \delta v _ {\mathrm{N}} ^ {2} ] = 0 \\ & P _ {0} (6, 6) = E [ \delta v _ {\mathrm{U}} ^ {2} ] = 0 \\ & P _ {0} (7, 7) = E [ \varphi_ {\mathrm{E}} ^ {2} ] = \frac {1}{g ^ {2}} \{[ C _ {b} ^ {n} (2, 1) ] ^ {2} E [ \nabla_ {x} ^ {2} ] + \end{array}
$$

<div class="mineru-algorithm" style="white-space: pre-wrap; font-family:monospace;">
$\left[C_{b}^{n}(2,2)\right]^{2}E\left[\nabla_{y}^{2}\right]+\left[C_{b}^{n}(2,3)\right]^{2}E\left[\nabla_{z}^{2}\right]$ $P_{0}(8,8)=E\left[\varphi_{N}^{2}\right]=\frac{1}{g^{2}}\left\{\left[C_{b}^{n}(1,1)\right]^{2}E\left[\nabla_{x}^{2}\right]+\left[C_{b}^{n}(1,2)\right]^{2}E\left[\nabla_{y}^{2}\right]+\left[C_{b}^{n}(1,3)\right]^{2}E\left[\nabla_{z}^{2}\right]\right\}$ $P_{0}(9,9)=E\left[\varphi_{U}^{2}\right]=\frac{1}{(\omega_{ie}\cos L)^{2}}\left\{\left[C_{b}^{n}(1,1)\right]^{2}(E\left[\varepsilon_{bx}^{2}\right]+E\left[\varepsilon_{rx}^{2}\right)+\left[C_{b}^{n}(1,2)\right]^{2}(E\left[\varepsilon_{by}^{2}\right]+E\left[\varepsilon_{ry}^{2}\right)+\left[C_{b}^{n}(1,3)\right]^{2}(E\left[\varepsilon_{bz}^{2}\right]+E\left[\varepsilon_{rz}^{2}\right])\right\}$ $P_{0}(10,10)=E\left[\varepsilon_{bx}^{2}\right]=(0.1^{\circ}/h)^{2}$ $P_{0}(11,11)=E\left[\varepsilon_{by}^{2}\right]=(0.1^{\circ}/h)^{2}$ $P_{0}(12,12)=E\left[\varepsilon_{bz}^{2}\right]=(0.1^{\circ}/h)^{2}$ $P_{0}(13,13)=E\left[\varepsilon_{rx}^{2}\right]=(0.1^{\circ}/h)^{2}$ $P_{0}(14,14)=E\left[\varepsilon_{ry}^{2}\right]=(0.1^{\circ}/h)^{2}$ $P_{0}(15,15)=E\left[\varepsilon_{rz}^{2}\right]=(0.1^{\circ}/h)^{2}$ $P_{0}(16,16)=E\left[\nabla_{rx}^{2}\right]=(0.0001 g)^{2}$ $P_{0}(17,17)=E\left[\nabla_{ry}^{2}\right]=(0.0001 g)^{2}$ $P_{0}(18,18)=E\left[\nabla_{rz}^{2}\right]=(0.0001 g)^{2}$ $P_{0}(19,19)=E\left[\delta K_{gx}^{2}\right]=(0.001)^{2}$ $P_{0}(20,20)=E\left[\delta K_{gy}^{2}\right]=(0.001)^{2}$ $P_{0}(21,21)=E\left[\delta K_{gz}^{2}\right]=(0.001)^{2}$ $P_{0}(22,22)=E\left[\delta K_{ax}^{2}\right]=(0.001)^{2}$ $P_{0}(23,23)=E\left[\delta K_{ay}^{2}\right]=(0.001)^{2}$ $P_{0}(24,24)=E\left[\delta K_{az}^{2}\right]=(0.001)^{2}$ $P_{0}(7,16)=P_{0}(16,7)=E\left[\varphi_{E}\cdot\nabla_{rx}\right]=-C_{b}^{n}(2,1)E\left[\nabla_{rx}^{2}\right]/g$ $P_{0}(7,17)=P_{0}(17,7)=E\left[\varphi_{E}\cdot\nabla_{ry}\right]=-C_{b}^{n}(2,2)E\left[\nabla_{ry}^{2}\right]/g$ $P_{0}(7,18)=P_{0}(18,7)=E\left[\varphi_{E}\cdot\nabla_{rz}\right]=-C_{b}^{n}(2,3)E\left[\nabla_{rz}^{2}\right]/g$ $P_{0}(8,16)=P_{0}(16,8)=E\left[\varphi_{N}\cdot\nabla_{rx}\right]=C_{b}^{n}(1,1)E\left[\nabla_{rx}^{2}\right]/g$ $P_{0}(8,17)=P_{0}(17,8)=E\left[\varphi_{N}\cdot\nabla_{ry}\right]=C_{b}^{n}(1,2)E\left[\nabla_{ry}^{2}\right]/g$ $P_{0}(8,18)=P_{0}(18,8)=E\left[\varphi_{N}\cdot\nabla_{rz}\right]=C_{b}^{n}(1,3)E\left[\nabla_{rz}^{2}\right]/g$ $P_{0}(9,10)=P_{0}(10,9)=E\left[\varphi_{U}\cdot\varepsilon_{bx}\right]=C_{b}^{n}(1,1)E\left[\varepsilon_{bx}^{2}\right]/(\omega_{ie}\cos L)$ $P_{0}(9,11)=P_{0}(11,9)=E\left[\varphi_{U}\cdot\varepsilon_{by}\right]=C_{b}^{n}(1,2)E\left[\varepsilon_{by}^{2}\right]/(\omega_{ie}\cos L)$ $P_{0}(9,12)=P_{0}(12,9)=E\left[\varphi_{U}\cdot\varepsilon_{bz}\right]=C_{b}^{n}(1,3)E\left[\varepsilon_{bz}^{2}\right]/(\omega_{ie}\cos L)$ $P_{0}(9,13)=P_{0}(13,9)=E\left[\varphi_{U}\cdot\varepsilon_{rx}\right]=C_{b}^{n}(1,1)E\left[\varepsilon_{rx}^{2}\right]/(\omega_{ie}\cos L)$ $P_{0}(9,14)=P_{0}(14,9)=E\left[\varphi_{U}\cdot\varepsilon_{ry}\right]=C_{b}^{n}(1,2)E\left[\varepsilon_{ry}^{2}\right]/(\omega_{ie}\cos L)$ $P_{0}(9,15)=P_{0}(15,9)=E\left[\varphi_{U}\cdot\varepsilon_{rz}\right]=C_{b}^{n}(1,3)E\left[\varepsilon_{rz}^{2}\right]/(\omega_{ie}\cos L)$ $P_{0}(25,25)=E\left[\delta L _{SA } ^ {  2 }\right]=(0. 0  2 ^ { \prime }) ^ {  2 }$ $P _{o}(  26 ,  26 ) = E [ \delta \lambda _{ SA } ^ {  2 } ] = (  0 .  0  2 ^ { \prime } ) ^ {  2 }$ $P _{o}(  27 ,  27 ) = E [ \delta h _{ SA } ^ {  2 } ] = (  5  0 m ) ^ {  2 }$ $P _{o}(  28 ,  28 ) = E [ \delta v _{ ESA } ^ {  2 } ] = (  0 .  25 m / s ) ^ {  2 }$ $P _{o}(  29 ,  29 ) = E [ \delta v _{ NSA } ^ {  2 } ] = (  0 .  25 m / s ) ^ {  2 }$
</div>

$$
\begin{array}{r l} & P _ {0} (3 0, 3 0) = E [ \delta v _ {\mathrm{USA}} ^ {2} ] = (0. 2 5 \mathrm{m/s}) ^ {2} \\ & P _ {0} (3 1, 3 1) = E [ \delta v _ {d x} ^ {2} ] = (0. 1 \mathrm{m/s}) ^ {2} \\ & P _ {0} (3 2, 3 2) = E [ \delta v _ {d y} ^ {2} ] = (0. 1 \mathrm{m/s}) ^ {2} \\ & P _ {0} (3 3, 3 3) = E [ \delta v _ {d z} ^ {2} ] = (0. 1 \mathrm{m/s}) ^ {2} \\ & P _ {0} (3 4, 3 4) = E [ \delta K _ {d x} ^ {2} ] = (0. 0 1) ^ {2} \\ & P _ {0} (3 5, 3 5) = E [ \delta K _ {d y} ^ {2} ] = (0. 0 0 2) ^ {2} \\ & P _ {0} (3 6, 3 6) = E [ \delta K _ {d z} ^ {2} ] = (0. 0 0 5) ^ {2} \\ & P _ {0} (3 7, 3 7) = E [ v _ {u b \mathrm{E}} ^ {2} ] = (5 \mathrm{m/s}) ^ {2} \\ & P _ {0} (3 8, 3 8) = E [ v _ {u b \mathrm{N}} ^ {2} ] = (3 \mathrm{m/s}) ^ {2} \\ & P _ {0} (3 9, 3 9) = E [ v _ {u b \mathrm{U}} ^ {2} ] = (1 \mathrm{m/s}) ^ {2} \\ & P _ {0} (4 0, 4 0) = E [ v _ {u r \mathrm{E}} ^ {2} ] = (5 \mathrm{m/s}) ^ {2} \\ & P _ {0} (4 1, 4 1) = E [ v _ {u r \mathrm{N}} ^ {2} ] = (3 \mathrm{m/s}) ^ {2} \\ & P _ {0} (4 2, 4 2) = E [ v _ {u r \mathrm{U}} ^ {2} ] = (1 \mathrm{m/s}) ^ {2} \\ & P _ {0} (4 3, 4 3) = E [ \delta K _ {\mathrm{ADS}} ^ {2} ] = (0. 0 0 3) ^ {2} \\ & P _ {0} (4 4, 4 4) = E [ \delta h _ {b} ^ {2} ] = (5 0 \mathrm{m}) ^ {2} \\ & P _ {0} (4 5, 4 5) = E [ \varphi_ {\mathrm{VG}, x} ^ {2} ] = (1 ^ {\circ}) ^ {2} \\ & P _ {0} (4 6, 4 6) = E [ \varphi_ {\mathrm{VG}, y} ^ {2} ] = (1 ^ {\circ}) ^ {2} \\ & P _ {0} (4 7, 4 7) = E [ \varepsilon_ {\mathrm{VG}, x} ^ {2} ] = (1 0 ^ {\circ} / \mathrm{h}) ^ {2} \\ & P _ {0} (4 8, 4 8) = E [ \varepsilon_ {\mathrm{VG}, y} ^ {2} ] = (1 0 ^ {\circ} / \mathrm{h}) ^ {2} \\ & P _ {0} (4 9, 4 9) = E [ \delta \Psi_ {\mathrm{MG}} ^ {2} ] = (3 0 ^ {\prime}) ^ {2} \end{array}
$$

仿真计算中所采用的飞行轨迹如表8.6.1所列。该表描述了对地攻击机从起飞、巡航、低空突防接近战区、对地攻击、返航的全过程。如图 $8.6.1\sim$ 图8.6.3所示，其实线分别为集中滤波器对惯导的经度、纬度及高度误差作估计时估计误差的均方根，这些均方根经蒙特-卡洛仿真获得，其中样本数为10，滤波周期为 $1\mathrm{s}$ 。图8.6.1和图8.6.2中套叠在上部的曲线是舍弃原曲线前 $10~s$ 数据后所绘出的，这样做的目的是能更清楚地描述滤波误差达到基本稳定后的变化细节。由诸曲线可看出，惯导的定位误差在滤波开始后的数秒内即可得到很好估计，并且随着滤波的推进，估计效果还会改善。这是由于滤波器对这些被估计量作直接测量，GPS的精度又很高的缘故。惯导的其余误差和参与组合的其余子系统也都得到较好估计，由于篇幅有限，这些曲线不再一一列出。

表 8.6.1 飞行轨迹参数表

<table><tr><td rowspan="2">飞行阶段</td><td rowspan="2">飞行动作</td><td rowspan="2">起始时刻s</td><td rowspan="2">持续时间s</td><td>加速度</td><td colspan="2">航向Ψ</td><td colspan="2">倾斜角</td><td colspan="2">俯仰角</td><td rowspan="2">速度m/s</td><td rowspan="2">高度m</td></tr><tr><td> $\frac{a_y^t}{m/s^2}$ </td><td> $\dot{\Psi}/(^{\circ})/s$ </td><td> $\frac{\Psi}{(^{\circ})}$ </td><td> $\dot{\gamma}/(^{\circ})/s$ </td><td> $\frac{\gamma}{(^{\circ})}$ </td><td> $\dot{\theta}/(^{\circ})/s$ </td><td> $\frac{\theta}{(^{\circ})}$ </td></tr><tr><td>1</td><td>滑跑起飞</td><td>0</td><td>20</td><td>2.5</td><td>0</td><td>135</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td></tr><tr><td>2</td><td>进入爬升</td><td>20</td><td>2</td><td>0</td><td>0</td><td>135</td><td>0</td><td>0</td><td>15</td><td>0</td><td>50</td><td>0</td></tr><tr><td>3</td><td>爬升</td><td>22</td><td>114</td><td>0</td><td>0</td><td>135</td><td>0</td><td>0</td><td>0</td><td>30</td><td>50</td><td>25.6</td></tr><tr><td>4</td><td>改平</td><td>136</td><td>2</td><td>0</td><td>0</td><td>135</td><td>0</td><td>0</td><td>-15</td><td>30</td><td>50</td><td>2 875.6</td></tr><tr><td rowspan="2">飞行阶段</td><td rowspan="2">飞行动作</td><td rowspan="2">起始时刻/s</td><td rowspan="2">持续时间/s</td><td>加速度</td><td colspan="2">航向 Ψ</td><td colspan="2">倾斜角</td><td colspan="2">俯仰角</td><td rowspan="2">速度/m/s</td><td rowspan="2">高度/m</td></tr><tr><td> $\frac{a_y'}{m/s^2}$ </td><td> $\dot{\Psi}/(^{\circ})/s$ </td><td> $\frac{\Psi}{(^{\circ})}$ </td><td> $\dot{\gamma}/(^{\circ})/s$ </td><td> $\frac{\gamma}{(^{\circ})}$ </td><td> $\dot{\theta}/(^{\circ})/s$ </td><td> $\frac{\theta}{(^{\circ})}$ </td></tr><tr><td>5</td><td>匀速平飞</td><td>138</td><td>100</td><td>0</td><td>0</td><td>135</td><td>0</td><td>0</td><td>0</td><td>0</td><td>50</td><td>2 901.2</td></tr><tr><td>6</td><td>加速平飞</td><td>238</td><td>25</td><td>1</td><td>0</td><td>135</td><td>0</td><td>0</td><td>0</td><td>0</td><td>50</td><td>2 901.2</td></tr><tr><td>7</td><td>匀速平飞</td><td>263</td><td>300</td><td>0</td><td>0</td><td>135</td><td>0</td><td>0</td><td>0</td><td>0</td><td>75</td><td>2 901.2</td></tr><tr><td>8</td><td>向右倾斜</td><td>563</td><td>1</td><td>0</td><td>0</td><td>135</td><td>15</td><td>0</td><td>0</td><td>0</td><td>75</td><td>2 901.2</td></tr><tr><td>9</td><td>右盘旋</td><td>564</td><td>23</td><td>0</td><td>2</td><td>136</td><td>0</td><td>15</td><td>0</td><td>0</td><td>75</td><td>2 901.2</td></tr><tr><td>10</td><td>退出盘旋</td><td>587</td><td>1</td><td>0</td><td>2</td><td>182.2</td><td>-15</td><td>15</td><td>0</td><td>0</td><td>75</td><td>2 901.2</td></tr><tr><td>11</td><td>匀速平飞</td><td>588</td><td>200</td><td>0</td><td>0</td><td>183.2</td><td>0</td><td>0</td><td>0</td><td>0</td><td>75</td><td>2 901.2</td></tr><tr><td>12</td><td>向左倾斜</td><td>788</td><td>1</td><td>0</td><td>0</td><td>183.2</td><td>-15</td><td>0</td><td>0</td><td>0</td><td>75</td><td>2 901.2</td></tr><tr><td>13</td><td>左盘旋</td><td>789</td><td>23</td><td>0</td><td>-2</td><td>182.2</td><td>0</td><td>-15</td><td>0</td><td>0</td><td>75</td><td>2 901.2</td></tr><tr><td>14</td><td>退出盘肇</td><td>812</td><td>1</td><td>0</td><td>-2</td><td>136</td><td>15</td><td>-15</td><td>0</td><td>0</td><td>75</td><td>2 901.2</td></tr><tr><td>15</td><td>匀速平飞</td><td>813</td><td>320</td><td>0</td><td>0</td><td>135</td><td>0</td><td>0</td><td>0</td><td>0</td><td>75</td><td>2 901.2</td></tr><tr><td>16</td><td>向右倾斜</td><td>1 133</td><td>1</td><td>0</td><td>0</td><td>135</td><td>15</td><td>0</td><td>0</td><td>0</td><td>75</td><td>2 901.2</td></tr><tr><td>17</td><td>右盘旋</td><td>1 134</td><td>180</td><td>0</td><td>2</td><td>136</td><td>0</td><td>15</td><td>0</td><td>0</td><td>75</td><td>2 901.2</td></tr><tr><td>18</td><td>退出盘旋</td><td>1 314</td><td>1</td><td>0</td><td>2</td><td>137.5</td><td>-15</td><td>15</td><td>0</td><td>0</td><td>75</td><td>2 901.2</td></tr><tr><td>19</td><td>匀速平飞</td><td>1 315</td><td>150</td><td>0</td><td>0</td><td>138.5</td><td>0</td><td>0</td><td>0</td><td>0</td><td>75</td><td>2 901.2</td></tr><tr><td>20</td><td>进入俯冲</td><td>1 465</td><td>2</td><td>0</td><td>0</td><td>138.5</td><td>0</td><td>0</td><td>-15</td><td>0</td><td>75</td><td>2 901.2</td></tr><tr><td>21</td><td>俯冲</td><td>1 467</td><td>40</td><td>0</td><td>0</td><td>138.5</td><td>0</td><td>0</td><td>0</td><td>-30</td><td>75</td><td>2 862.8</td></tr><tr><td>22</td><td>改平</td><td>1 507</td><td>2</td><td>0</td><td>0</td><td>138.5</td><td>0</td><td>0</td><td>15</td><td>-30</td><td>75</td><td>1 362.8</td></tr><tr><td>23</td><td>匀速平飞</td><td>1 509</td><td>400</td><td>0</td><td>0</td><td>138.5</td><td>0</td><td>0</td><td>0</td><td>0</td><td>75</td><td>1 324.4</td></tr><tr><td>24</td><td>向左倾斜</td><td>1 909</td><td>1</td><td>0</td><td>0</td><td>138.5</td><td>-15</td><td>0</td><td>0</td><td>0</td><td>75</td><td>1 324.4</td></tr><tr><td>25</td><td>左盘旋</td><td>1 910</td><td>202</td><td>0</td><td>-2</td><td>137.5</td><td>0</td><td>-15</td><td>0</td><td>0</td><td>75</td><td>1 324.4</td></tr><tr><td>26</td><td>退出盘旋</td><td>2 112</td><td>1</td><td>0</td><td>-2</td><td>91.8</td><td>15</td><td>-15</td><td>0</td><td>0</td><td>75</td><td>1 324.4</td></tr><tr><td>27</td><td>匀速平飞</td><td>2 113</td><td>960</td><td>0</td><td>0</td><td>90.8</td><td>0</td><td>0</td><td>0</td><td>0</td><td>75</td><td>1 324.4</td></tr><tr><td>28</td><td>向右倾斜</td><td>3 073</td><td>1</td><td>0</td><td>0</td><td>90.8</td><td>15</td><td>0</td><td>0</td><td>0</td><td>75</td><td>1 324.4</td></tr><tr><td>29</td><td>右盘旋</td><td>3 074</td><td>112</td><td>0</td><td>2</td><td>91.8</td><td>0</td><td>15</td><td>0</td><td>0</td><td>75</td><td>1 324.4</td></tr><tr><td>30</td><td>退出盘旋</td><td>3 186</td><td>1</td><td>0</td><td>2</td><td>316.7</td><td>-15</td><td>15</td><td>0</td><td>0</td><td>75</td><td>1 324.4</td></tr><tr><td>31</td><td>匀速平飞</td><td>3 187</td><td>400</td><td>0</td><td>0</td><td>317.7</td><td>0</td><td>0</td><td>0</td><td>0</td><td>75</td><td>1 324.4</td></tr><tr><td>32</td><td>加速平飞</td><td>3 587</td><td>15</td><td>1</td><td>0</td><td>317.7</td><td>0</td><td>0</td><td>0</td><td>0</td><td>75</td><td>1 324.4</td></tr><tr><td>33</td><td>匀速平飞</td><td>3 602</td><td>80</td><td>0</td><td>0</td><td>317.7</td><td>0</td><td>0</td><td>0</td><td>0</td><td>90</td><td>1 324.4</td></tr><tr><td rowspan="2">飞行阶段</td><td rowspan="2">飞行动作</td><td rowspan="2">起始时刻S</td><td rowspan="2">持续时间S</td><td>加速度</td><td colspan="2">航向 Ψ</td><td colspan="2">倾斜角</td><td colspan="2">俯仰角</td><td rowspan="2">速度m/s</td><td rowspan="2">高度m</td></tr><tr><td> $\frac{a'_v}{m/s^2}$ </td><td> $\dot{\Psi}/(^{\circ})/s$ </td><td> $\frac{\Psi}{(^{\circ})}$ </td><td> $\dot{\gamma}/(^{\circ})/s$ </td><td> $\frac{\gamma}{(^{\circ})}$ </td><td> $\dot{\theta}/(^{\circ})/s$ </td><td> $\frac{\theta}{(^{\circ})}$ </td></tr><tr><td>34</td><td>进入俯冲</td><td>3 682</td><td>2</td><td>0</td><td>0</td><td>317.7</td><td>0</td><td>0</td><td>-15</td><td>0</td><td>90</td><td>1 324.4</td></tr><tr><td>35</td><td>俯冲</td><td>3 684</td><td>25</td><td>0</td><td>0</td><td>317.7</td><td>0</td><td>0</td><td>0</td><td>-30</td><td>90</td><td>1 278.4</td></tr><tr><td>36</td><td>改平</td><td>3 709</td><td>2</td><td>0</td><td>0</td><td>317.7</td><td>0</td><td>0</td><td>15</td><td>-30</td><td>90</td><td>153.4</td></tr><tr><td>37</td><td>匀速平飞</td><td>3 711</td><td>120</td><td>0</td><td>0</td><td>317.7</td><td>0</td><td>0</td><td>0</td><td>0</td><td>90</td><td>107.3</td></tr><tr><td>38</td><td>向右倾斜</td><td>3 831</td><td>2</td><td>0</td><td>0</td><td>317.7</td><td>15</td><td>0</td><td>0</td><td>0</td><td>90</td><td>107.3</td></tr><tr><td>39</td><td>右盘旋</td><td>3 833</td><td>100</td><td>0</td><td>3.6</td><td>321.1</td><td>0</td><td>30</td><td>0</td><td>0</td><td>90</td><td>107.3</td></tr><tr><td>40</td><td>退出盘旋</td><td>3 933</td><td>2</td><td>0</td><td>3.6</td><td>321.7</td><td>-15</td><td>30</td><td>0</td><td>0</td><td>90</td><td>107.3</td></tr><tr><td>41</td><td>匀速平飞</td><td>3 935</td><td>150</td><td>0</td><td>0</td><td>325.15</td><td>0</td><td>0</td><td>0</td><td>0</td><td>90</td><td>107.3</td></tr><tr><td>42</td><td>进入爬升</td><td>4 085</td><td>2</td><td>0</td><td>0</td><td>325.15</td><td>0</td><td>0</td><td>15</td><td>0</td><td>90</td><td>107.3</td></tr><tr><td>43</td><td>爬升</td><td>4 087</td><td>60</td><td>0</td><td>0</td><td>325.15</td><td>0</td><td>0</td><td>0</td><td>30</td><td>90</td><td>153.4</td></tr><tr><td>44</td><td>退出爬升</td><td>4 147</td><td>2</td><td>0</td><td>0</td><td>325.15</td><td>0</td><td>0</td><td>-15</td><td>30</td><td>90</td><td>2 853.4</td></tr><tr><td>45</td><td>匀速平飞</td><td>4 149</td><td>300</td><td>0</td><td>0</td><td>325.15</td><td>0</td><td>0</td><td>0</td><td>0</td><td>90</td><td>2 899.4</td></tr><tr><td>46</td><td>减速</td><td>4 449</td><td>15</td><td>-1</td><td>0</td><td>325.15</td><td>0</td><td>0</td><td>0</td><td>0</td><td>90</td><td>2 899.4</td></tr><tr><td>47</td><td>匀速平飞</td><td>4 464</td><td>450</td><td>0</td><td>0</td><td>325.15</td><td>0</td><td>0</td><td>0</td><td>0</td><td>75</td><td>2 899.4</td></tr><tr><td>48</td><td>向左倾斜</td><td>4 914</td><td>2</td><td>0</td><td>0</td><td>325.15</td><td>-15</td><td>0</td><td>0</td><td>0</td><td>75</td><td>2 899.4</td></tr><tr><td>49</td><td>左盘旋</td><td>4 916</td><td>10</td><td>0</td><td>-4.33</td><td>321</td><td>0</td><td>-30</td><td>0</td><td>0</td><td>75</td><td>2 899.4</td></tr><tr><td>50</td><td>退出盘旋</td><td>4 926</td><td>2</td><td>0</td><td>-4.33</td><td>277.8</td><td>15</td><td>-30</td><td>0</td><td>0</td><td>75</td><td>2 899.4</td></tr><tr><td>51</td><td>返航</td><td>4 928</td><td>200</td><td>0</td><td>0</td><td>273.6</td><td>0</td><td>0</td><td>0</td><td>0</td><td>75</td><td>2 899.4</td></tr></table>

![](images/a4cb1b71b82000253a376e8c22a26668651e6ed895a46c7786e7fbbbf256aed8.jpg)  
图 8.6.1 集中滤波器和联邦滤波器对惯导经度的估计误差

![](images/c17129e8fcb11b8fa2ec91851fe279c54706097544d1469d910dd2745a3ea03d.jpg)  
图 8.6.2 集中滤波器和联邦滤波器对惯导纬度的估计误差

![](images/2f4ba5b06b0af2998a96142d9500551c86a9d9ccbbcc38d388071d128abfcbe6.jpg)  
图 8.6.3 集中滤波器和联邦滤波器对惯导高度的估计误差

## 8.7 应用联邦滤波理论设计容错组合导航系统

## 8.7.1 联邦滤波器设计及性能仿真

假设机载导航子系统配备情况与8.6节介绍集中滤波器设计时假设的配备情况相同，即非相似导航子系统有：激光陀螺捷联惯导系统(RLG SINS)，C/A码GPS接收机(GPS)，多普勒雷达(DVS)，大气数据系统(ADS)，垂直陀螺(VG)，磁航向仪(MG)。选择惯导为公共参考系统，与其余子系统分别组合，构成5个卡尔曼子滤波器，则联邦滤波器结构如图8.7.1所示。其中子滤波器 $i(i=1,2,\cdots,5)$ 给出惯导及相应子系统状态的最优估计。由于子滤波器i作量测更新时只利用了与之相应的量测 $Z_{i}(i=1,2,\cdots,5)$ ，所以获得的关于惯导状态的估计只是局部最优的。主滤波器的作用是将各子滤波器给出的关于惯导状态的局部最优估计按融合算法合成得关于惯导状态的全局最优估计，即根据所有量测 $Z_{1},Z_{2},\cdots,Z_{5}$ 确定的关于惯导状态的最优估计。为了确保组合导航系统具有高容错性能，联邦滤波器采用无复位结构。

各子滤波器的状态及量测维数如表 8.7.1 所列。

![](images/b8a0f91c27ab2c7853ec526fc6d07cca6286bd5260866afc5c4b0f07f77803a5.jpg)  
图 8.7.1 由 RLG SINS, GPS, DVS, ADS, VG 及 MG 构成的联邦滤波器

各子滤波器的状态方程和量测方程具体列写为

子滤波器1：

$$
\left[ \begin{array}{l} \dot {\boldsymbol {X}} _ {\mathrm{INS}} \\ \dot {\boldsymbol {X}} _ {\mathrm{GPS}} \end{array} \right] = \left[ \begin{array}{c c} \boldsymbol {F} _ {\mathrm{INS}} & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {F} _ {\mathrm{GPS}} \end{array} \right] \quad \left[ \begin{array}{l} \boldsymbol {X} _ {\mathrm{INS}} \\ \boldsymbol {X} _ {\mathrm{GPS}} \end{array} \right] + \left[ \begin{array}{c c} \boldsymbol {G} _ {\mathrm{INS}} & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {I} \end{array} \right] \quad \left[ \begin{array}{l} \boldsymbol {w} _ {\mathrm{INS}} \\ \boldsymbol {w} _ {\mathrm{GPS}} \end{array} \right]\tag{8.7.1}
$$

$$
\mathbf {Z} _ {\mathrm{GPS}} = \left[ \begin{array}{l l} \mathbf {H} _ {1} & \mathbf {H} _ {\mathrm{GPS}} \end{array} \right] \left[ \begin{array}{l} \mathbf {X} _ {\mathrm{INS}} \\ \mathbf {X} _ {\mathrm{GPS}} \end{array} \right] + \mathbf {V} _ {\mathrm{GPS}}
$$

式中 $X_{GPS}$ ， $F_{GPS}$ ， $w_{GPS}$ 分别见式(8.6.5)、式(8.6.7)和式(8.6.8)。

表 8.7.1 各子滤波器的状态和量测维数

<table><tr><td rowspan="2">子滤波器</td><td rowspan="2">量测维数</td><td rowspan="2">状态维数</td><td colspan="2">子系统</td><td colspan="2">公共参考系统</td></tr><tr><td>名称</td><td>维数</td><td>名称</td><td>维数</td></tr><tr><td>1</td><td>6</td><td>30</td><td>GPS</td><td>6</td><td rowspan="5">RLG SINS</td><td rowspan="5">24</td></tr><tr><td>2</td><td>3</td><td>30</td><td>DVS</td><td>6</td></tr><tr><td>3</td><td>4</td><td>32</td><td>ADS</td><td>8</td></tr><tr><td>4</td><td>2</td><td>28</td><td>VG</td><td>4</td></tr><tr><td>5</td><td>1</td><td>25</td><td>MG</td><td>1</td></tr></table>

子滤波器2：

$$
\left[ \begin{array}{l} \dot {\boldsymbol {X}} _ {\mathrm{INS}} \\ \dot {\boldsymbol {X}} _ {\mathrm{DVS}} \end{array} \right] = \left[ \begin{array}{c c} \boldsymbol {F} _ {\mathrm{INS}} & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {F} _ {\mathrm{DVS}} \end{array} \right] \quad \left[ \begin{array}{l} \boldsymbol {X} _ {\mathrm{INS}} \\ \boldsymbol {X} _ {\mathrm{DVS}} \end{array} \right] + \left[ \begin{array}{c c} \boldsymbol {G} _ {\mathrm{INS}} & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {G} _ {\mathrm{DVS}} \end{array} \right] \quad \left[ \begin{array}{l} \boldsymbol {w} _ {\mathrm{INS}} \\ \boldsymbol {w} _ {\mathrm{DVS}} \end{array} \right]\tag{8.7.2}
$$

$$
\mathbf {Z} _ {\mathrm{DVS}} = \left[ \begin{array}{l l} \mathbf {H} _ {2} & \mathbf {H} _ {\mathrm{DVS}} \end{array} \right] \left[ \begin{array}{l} \mathbf {X} _ {\mathrm{INS}} \\ \mathbf {X} _ {\mathrm{DVS}} \end{array} \right] + \mathbf {V} _ {\mathrm{DVS}}
$$

式中 $X_{DVS}$ ， $F_{DVS}$ ， $w_{DVS}$ 和 $G_{DVS}$ 分别见式(8.6.9)，式(8.6.11)～式(8.6.13)。

子滤波器 3:

$$
\left[ \begin{array}{l} \dot {\mathbf {X}} _ {\mathrm{INS}} \\ \dot {\mathbf {X}} _ {\mathrm{ADS}} \end{array} \right] = \left[ \begin{array}{c c} \mathbf {F} _ {\mathrm{INS}} & \mathbf {0} \\ \mathbf {0} & \mathbf {F} _ {\mathrm{ADS}} \end{array} \right] \quad \left[ \begin{array}{l} \mathbf {X} _ {\mathrm{INS}} \\ \mathbf {X} _ {\mathrm{ADS}} \end{array} \right] + \left[ \begin{array}{c c} \mathbf {G} _ {\mathrm{INS}} & \mathbf {0} \\ \mathbf {0} & \mathbf {G} _ {\mathrm{ADS}} \end{array} \right] \quad \left[ \begin{array}{l} w _ {\mathrm{INS}} \\ w _ {\mathrm{ADS}} \end{array} \right]\tag{8.7.3}
$$

$$
\mathbf {Z} _ {\mathrm{ADS}} = \left[ \begin{array}{l l} \mathbf {H} _ {3} & \mathbf {H} _ {\mathrm{ADS}} \end{array} \right] \left[ \begin{array}{l} \mathbf {X} _ {\mathrm{INS}} \\ \mathbf {X} _ {\mathrm{ADS}} \end{array} \right] + \mathbf {V} _ {\mathrm{ADS}}
$$

式中 $X_{ADS}$ ， $F_{ADS}$ ， $w_{ADS}$ 及 $G_{ADS}$ 分别见式(8.6.14)，式(8.6.16)～式(8.6.18)。

子滤波器 4:

$$
\left[ \begin{array}{l} \dot {\boldsymbol {X}} _ {\mathrm{INS}} \\ \dot {\boldsymbol {X}} _ {\mathrm{VG}} \end{array} \right] = \left[ \begin{array}{c c} \boldsymbol {F} _ {\mathrm{INS}} & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {F} _ {\mathrm{VG}} \end{array} \right] \quad \left[ \begin{array}{l} \boldsymbol {X} _ {\mathrm{INS}} \\ \boldsymbol {X} _ {\mathrm{VG}} \end{array} \right] + \left[ \begin{array}{c c} \boldsymbol {G} _ {\mathrm{INS}} & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {G} _ {\mathrm{VG}} \end{array} \right] \quad \left[ \begin{array}{l} \boldsymbol {w} _ {\mathrm{INS}} \\ \boldsymbol {w} _ {\mathrm{VG}} \end{array} \right]\tag{8.7.4}
$$

$$
\mathbf {Z} _ {\mathrm{VG}} = \left[ \begin{array}{l l} \mathbf {H} _ {4} & \mathbf {H} _ {\mathrm{VG}} \end{array} \right] \left[ \begin{array}{l} \mathbf {X} _ {\mathrm{INS}} \\ \mathbf {X} _ {\mathrm{VG}} \end{array} \right] + \mathbf {V} _ {\mathrm{VG}}
$$

式中 $X_{VG}$ ， $F_{VG}$ ， $w_{VG}$ 及 $G_{VG}$ 分别见式(8.6.19)、式(8.6.21)～式(8.6.23)。

子滤波器 5:

$$
\left[ \begin{array}{l} \dot {\boldsymbol {X}} _ {\mathrm{INS}} \\ \dot {\boldsymbol {X}} _ {\mathrm{MG}} \end{array} \right] = \left[ \begin{array}{c c} \boldsymbol {F} _ {\mathrm{INS}} & \boldsymbol {0} \\ \boldsymbol {0} & - \frac {1}{\tau_ {\mathrm{MG}}} \end{array} \right] \quad \left[ \begin{array}{l} \boldsymbol {X} _ {\mathrm{INS}} \\ \boldsymbol {X} _ {\mathrm{MG}} \end{array} \right] + \left[ \begin{array}{c c} \boldsymbol {G} _ {\mathrm{INS}} & \boldsymbol {0} \\ \boldsymbol {0} & 1 \end{array} \right] \quad \left[ \begin{array}{l} w _ {\mathrm{INS}} \\ w _ {\mathrm{MG}} \end{array} \right]\tag{8.7.5}
$$

$$
Z _ {\mathrm{MG}} = \left[ \begin{array}{l l} \boldsymbol {H} _ {5} & H _ {\mathrm{MG}} \end{array} \right] \left[ \begin{array}{l} \boldsymbol {X} _ {\mathrm{INS}} \\ X _ {\mathrm{MG}} \end{array} \right] + V _ {\mathrm{MG}}
$$

式中

$$
X _ {\mathrm{MG}} = \delta \Psi_ {\mathrm{MG}}
$$

以上诸式中， $X_{INS}$ ， $w_{INS}$ 及 $G_{INS}$ 分别见式(8.6.1)，式(8.6.3)及式(8.6.4)， $F_{INS}$ 见 8.6.2 小节中关于 $F_{INS}$ 非零元的定义。

设计各子滤波器时，所选参数与8.6.2小节中所选参数相同。仿真计算时，飞行轨迹的假定与8.6.2小节中所假设的也一样。各子滤波器的滤波周期为1s，且假定各子滤波器保持同步滤波，主滤波器的融合周期为10s。惯导作为公共参考系统，假设在滤波过程中无任何故障发生，其信息只在滤波器启动时进行平均分配。显然联邦滤波结果是全局次优的。如图8.6.1～图8.6.3所示其虚线分别为联邦滤波器对惯导的经度、纬度及高度误差作估计时，估计误差的均方根。与集中滤波器情况一样，均方根也经蒙特-卡洛仿真获得，其中样本数也为10。

从图 8.6.1～图 8.6.3 可看出，虽然在所假设条件下获得的估计结果是全局次优的，但其估计精度与由集中滤波器获得的全局最优估计精度十分接近。由于联邦滤波采用了各子滤波器的并行分散滤波和主滤波器的合成融合算法，且融合周期远比滤波周期长，所以计算量远比集中滤波器小。因此，联邦滤波器特别适用于由众多非相似导航子系统构成的组合导航系统的设计。

## 8.7.2 联邦滤波器设计中的信息分配和信息同步处理

在实际系统的设计中,参与滤波的各子系统的导航精度高低不一,惯导作为公共参考系统,其信息由各子滤波器共同分享,如何合理分配惯导信息,使惯导的融合精度和其余各子系统的估计精度都达到满意的程度,这是必须解决的实际问题之一。另外,参与组合的各子系统的输出率千差万别,一般是不同步的。在各子滤波器以各自的滤波周期进行滤波的条件下,如何既不影响各子滤波器的正常工作,又能在融合时间点上获得各子滤波器的同步输出,这是必须解决的第二个实际问题。

## 1. 公共参考信息的分配原则

设参与组合的导航子系统除激光陀螺捷联惯导外,还有 N 个非相似导航子系统。由于惯导是公共参考系统,它参与了由该 N 个子系统和惯导构成的 N 个子滤波器的滤波,所以惯导的信息应在这 N 个子滤波器之间进行分配,根据信息守恒原理,分配系数应满足

$$
\sum_ {i = 1} ^ {N} \beta_ {i} = 1
$$

式中 $\beta_{i}$ 为第 i 个子滤波器获得信息的分配系数。

设 X 的估计误差的均方误差阵为 P，则 P 描述了对 X 的估计质量，由 2.2.4 小节和 3.4 节知， $P^{-1}$ 为信息矩阵。所以，P 越大，X 的估计质量就越差，此时，信息矩阵 $P^{-1}$ 就越小；反之，P 越小，X 的估计质量就越好，信息矩阵就越大。因此，对惯导系统的信息作分配，实质上就是将参与第 i 个子滤波器滤波的惯导的估计均方误差阵 $P_{C}$ 扩大 $\frac{1}{\beta_{i}}$ 倍。从中可看出， $\beta_{i}$ 越小，则 $P_{C}$ 扩大的倍数就越大。由于卡尔曼滤波器能自动根据信息质量的优劣作权重不同的利用，所以 $\beta_{i}$ 越小，对惯导信息的利用权重就越低，该子滤波器的滤波精度主要取决于子系统 i 的信息质量，而惯导系统的输出信息所起的作用相对降低；反之亦然。

从上述分析可得出惯导信息分配的一般原则:在子滤波器i中,子系统i的精度越差,则惯导信息的分配系数 $\beta_{i}$ 就应该越大;子系统i的精度越高,子滤波器i的滤波精度受 $\beta_{i}$ 的影响就越小,在这种情况下, $\beta_{i}$ 应适当取得小些,以便使总量有限的惯导信息在较低精度子系统所在的子滤波器中能充分发挥作用。

仿真结果证实了上述结论，如图8.7.2和图8.7.3所示。其中图8.7.2为惯导的纬度估计误差，GPS所在子滤波器的惯导信息分配系数为 $\beta_{1} = 0.1$ ，其余子滤波器的分配系数约为0.225，虚线曲线为从联邦滤波器获得的结果，实线曲线为从集中滤波器获得的结果， $\beta_{1} = 0.9$ ， $\beta_{i} = 0.025(i = 2,3,4,5)$ 的结果与此结果几乎重合，所以末画出。为了表达清当估计误差已降得很低时的细微变化，专门画出了从 $10\mathrm{s}$ 开始绘出的曲线，即图中套叠在上部的曲线。图8.7.3为垂直陀螺所在子滤波器的分配系数取不同值时，垂直陀螺沿飞机纵轴的水平姿态估计误差，其中实线和虚线对应的分配系数分别为0.9和0.025。

## 2. 各子滤波器输出信息的同步处理

设惯导的输出周期为 $T_{\mathrm{INS}}$ ，联邦滤波器的融合周期为 $T$ ，子滤波器 $i$ 的滤波周期为 $T_{i} = N_{i}T_{\mathrm{INS}}$ ，在第 $j$ 个融合时间点上，子滤波器 $i$ 的时标差为 $\Delta \tau_{i}(j)$ ，相对第 $j + 1$ 个融合时间点，子滤波器 $i$ 的融合同步时间差为 $\Delta t_i(j)$ ，在时间段 $[jT,(j + 1)T)$ 内子滤波器 $i$ 共输出 $K_{i}(j)$ 次，

如图8.7.4所示。

![](images/483bd5504f886e11209b17740187faee16c0ea08f62ded155837c7afdcff1552.jpg)  
图8.7.2 $\beta_{1} = 0.1$ 时主滤波器对惯导纬度的估计误差

![](images/0008579a50b0890a4f5a03f1509cb9d030d115fe0cbe38c84305c68fa73bff1a.jpg)  
图8.7.3 $\beta_{4} = 0.9$ 和 $\beta_{4} = 0.025$ 时垂直陀螺沿飞机纵轴的水平姿态估计误差

![](images/ddf169a7fffc20a41bbb3487928d10528c319114f61a513ca292b2ac4ffa56e0.jpg)  
图 8.7.4 子滤波器 i 的输出时间与融合时间的关系

由图8.7.4得

$$
\Delta \tau_ {i} (j) + [ K _ {i} (j) - 1 ] T _ {i} + \Delta t _ {i} (j) = T\tag{8.7.6}
$$

即

$$
\Delta t _ {i} (j) = T - \left[ K _ {i} (j) - 1 \right] T _ {i} - \Delta \tau_ {i} (j)\tag{8.7.7}
$$

$$
\Delta \tau_ {i} (j + 1) = K _ {i} (j) T _ {i} + \Delta \tau_ {i} (j) - T\tag{8.7.8}
$$

式中 $K_{i}(j)$ 为正整数。

由式(8.7.6)得

$$
K _ {i} (j) = \frac {T - \Delta \tau_ {i} (j)}{T _ {i}} - \frac {\Delta t _ {i} (j)}{T _ {i}} + 1
$$

由于 $0 \leqslant \Delta t_{i}(j) < T_{i}$ ，即 $0 \leqslant \frac{\Delta t_{i}(j)}{T_{i}} < 1$ ，所以

$$
K _ {i} (j) = \left[ \left[ \frac {T - \Delta \tau_ {i} (j)}{T _ {i}} \right] \right] + 1\tag{8.7.9}
$$

上述诸式中， $i=1,2,\cdots,N,j=0,1,2,\cdots,\Delta\tau_{i}(0)$ 为开始滤波时子滤波器 i 的输出滞后，双写方括号表示对括号内的数取整。

由于子滤波器 i 在 $(j+1)T$ 时刻无量测值，所以在该融合时间点上子滤波器 i 参与融合的滤波值只能由时间更新确定：

$$
\hat {\boldsymbol {X}} _ {C i} [ (j + 1) T ] = \boldsymbol {\Phi} [ (j + 1) T, t _ {i} (j) ] \hat {\boldsymbol {X}} _ {C i} [ t _ {i} (j) ]\tag{8.7.10}
$$

$$
\boldsymbol {P} _ {i} [ (j + 1) T ] = \boldsymbol {\Phi} [ (j + 1) T, t _ {i} (j) ] \boldsymbol {P} _ {i} [ t _ {i} (j) ] \boldsymbol {\Phi} ^ {\mathrm{T}} [ (j + 1) T, t _ {i} (j) ] + \bar {\boldsymbol {Q}} _ {i} (j)\tag{8.7.11}
$$

式中

$$
t _ {i} (j) = j T + \Delta \tau_ {i} (j) + [ K _ {i} (j) - 1 ] T _ {i}
$$

$$
\Phi [ (j + 1) T, t _ {i} (j) ] = \sum_ {n = 0} ^ {\infty} \frac {[ F _ {C j} \Delta t _ {i} (j) ] ^ {n}}{n !}\tag{8.7.12}
$$

$$
\begin{array}{r l} \overline {{{Q}}} _ {i} (j) = & q \Delta t _ {i} (j) + [ F _ {C j} q + (F _ {C j} q) ^ {\mathrm{T}} ] \frac {\Delta t _ {i} ^ {2} (j)}{2 !} + \\ & \{F _ {C j} [ F _ {C j} q + (F _ {C j} q) ^ {\mathrm{T}} ] + [ F _ {C j} (F _ {C j} q + (F _ {C j} q) ^ {\mathrm{T}}) ] ^ {\mathrm{T}} \} \frac {\Delta t _ {i} ^ {3} (j)}{3 !} + \dots \end{array}\tag{8.7.13}
$$

式中， $\Delta t_{i}(j)$ 为同步差； $F_{Cj}$ 为 $t=t_{i}(j)$ 时刻惯导的系统阵；q 为惯导的激励白噪声的噪声方差强度阵。

综上所述,可得对子滤波器 i 作同步处理的一般步骤:

(1) 按式(8.7.9) 计算 $[jT, (j+1)T]$ 时间段内子滤波器 i 的输出次数 $K_{i}(j)$ ;

(2) 按式(8.7.7) 计算在 $(j+1)T$ 融合时间点上的同步时间差 $\Delta t_{i}(j)$ ;

(3) 按式(8.7.8)计算下一个融合周期内的时标差 $\Delta\tau_{i}(j+1)$ ;

（4）对子滤波器 i 的输出从 t=jT 起进行计数，当计数值达到 $K_{i}(j)$ 时，按式(8.7.10) 计算子滤波器 i 在融合时间点 $t=(j+1)T$ 上的输出。

顺序执行上述过程,即可计算出各融合时间点上各子滤波器的同步输出。

在对信息的同步处理中，各子滤波器在融合时间点上只作时间更新而无量测更新，这必然会影响融合精度。图8.7.5为惯导的纬度经融合后的估计误差曲线，其中实线曲线为各子滤波器保持同步时的结果，虚线曲线为滤波不同步而经信息同步处理后的结果。融合周期为10 s，各子系统输出周期如表8.7.2所列。

表 8.7.2 各子系统的输出周期  
单位：s

<table><tr><td>子系统名</td><td>同步</td><td>不同步</td></tr><tr><td>激光陀螺捷联惯导</td><td>0.02</td><td>0.02</td></tr><tr><td>C/A码GPS接收机</td><td>1.0</td><td>1.0</td></tr><tr><td>多普勒雷达</td><td>1.0</td><td>0.3</td></tr><tr><td>大气数据计算机</td><td>1.0</td><td>0.6</td></tr><tr><td>垂直陀螺</td><td>1.0</td><td>1.0</td></tr><tr><td>磁航向仪</td><td>1.0</td><td>0.12</td></tr></table>

从图 8.7.5 可看出,与各子滤波器完全保持同步时的融合精度相比较,经同步处理后,融合精度的下降是很微小的。

![](images/d465f9f032e6e37a28b0c3b73773b3936c588394207a74e4f4277631681a8831.jpg)  
图 8.7.5 惯导纬度融合估计误差

## 8.7.3 容错组合导航系统设计和容错性能分析

## 1. 捷联惯导余度斜置陀螺的容错全姿态测量

组合导航系统要维持联邦滤波结构，以便使导航系统具有高容错性，作为公共参考系统的惯导必须绝对可靠地工作。但是，任何惯性器件都无法确保绝对不出故障，所以，采用余度斜置技术和容错测量技术是获取运载体全姿态信息以确保惯导可靠工作的关键。对于由惯性器件失效而引起的硬故障，采用奇偶方程校验就可将失效器件隔离出来。对于由惯性器件性能发生变化而引起的软故障，极大似然法是进行故障隔离的有效办法。显然软故障的检测和隔离远比硬故障困难。此处以陀螺为例说明软故障的检测和隔离。

设运载体的角速度为

$$
\boldsymbol {\omega} = \left[ \begin{array}{l l l} \omega_ {x} & \omega_ {y} & \omega_ {z} \end{array} \right] ^ {\mathrm{T}}
$$

捷联陀螺测量组采用图 7.4.5 所介绍的 6 陀螺斜置配置，并假设各陀螺的随机常值漂移已得到补偿。则当所有陀螺都正常工作时的测量方程为

$$
\mathbf {Z} = \mathbf {H} \boldsymbol {\omega} + \varepsilon\tag{8.7.14}
$$

式中

$$
\begin{array}{r l} & {\mathbf {Z} = [ Z _ {1} \quad Z _ {2} \quad \dots \quad Z _ {6} ] ^ {\mathrm{T}}} \\ & {\pmb {\varepsilon} = [ \pmb {\varepsilon} _ {1} \quad \pmb {\varepsilon} _ {2} \quad \dots \quad \pmb {\varepsilon} _ {6} ] ^ {\mathrm{T}}} \\ & {\pmb {\varepsilon} _ {i} \sim N (0, \sigma^ {2}) \qquad i = 1, 2, \dots , 6} \\ & {\pmb {H} = \left[ \begin{array}{c c c c c c} {\sin \alpha} & {- \sin \alpha} & {\cos \alpha} & {\cos \alpha} & {0} & {0} \\ {0} & {0} & {\sin \alpha} & {- \sin \alpha} & {\cos \alpha} & {\cos \alpha} \\ {\cos \alpha} & {\cos \alpha} & {0} & {0} & {\sin \alpha} & {- \sin \alpha} \end{array} \right] ^ {\mathrm{T}}} \\ & {2 \alpha = 6 3 ^ {\circ} 2 6 ^ {\prime} 5. 8 ^ {\prime \prime}} \end{array}\tag{7.4.5}
$$

构造奇偶向量：

$$
\mathbf {P} = \mathbf {V Z}\tag{8.7.15}
$$

式中 V 为 $3 \times 6$ 矩阵, 且满足

$$
\left\{ \begin{array}{l} \mathbf {V H} = \mathbf {0} \\ \mathbf {V V ^ {\mathrm{T}}} = \mathbf {I} \end{array} \right.\tag{8.7.16}
$$

根据 7.4.3 节中所介绍的方法求解上述方程得

$$
\boldsymbol {V} = \left[ \begin{array}{c c c c c c} 0. 7 0 7 1 1 & - 0. 3 1 6 2 3 & - 0. 3 1 6 2 3 & - 0. 3 1 6 2 3 & - 0. 3 1 6 2 3 & 0. 3 1 6 2 3 \\ 0 & 0. 6 3 2 4 6 & 0. 1 9 5 4 4 & 0. 1 9 5 4 4 & - 0. 5 1 1 6 7 & 0. 5 1 1 6 7 \\ 0 & 0 & 0. 6 0 1 5 0 & - 0. 6 0 1 5 0 & - 0. 3 7 1 7 5 & - 0. 3 7 1 7 5 \end{array} \right]
$$

显然

$$
\boldsymbol {P} = V \varepsilon\tag{8.7.17}
$$

这说明,当陀螺测量组都工作正常时,P也服从高斯分布,且

$$
\begin{array}{l} {E [ P _ {i} ] = 0} \\ {E [ P _ {i} P _ {j} ] = \sigma^ {2} \delta_ {i j} \qquad i, j = 1, 2, 3} \end{array}
$$

即 P 的各分量 $P_{i}(i=1,2,3)$ 相互独立，且满足：

$$
P _ {i} \sim N (0, \sigma^ {2}) \qquad (i = 1, 2, 3)
$$

所以用 P 构造的统计量

$$
D = \left(\frac {P _ {1}}{\sigma}\right) ^ {2} + \left(\frac {P _ {2}}{\sigma}\right) ^ {2} + \left(\frac {P _ {3}}{\sigma}\right) ^ {2}\tag{8.7.18}
$$

服从自由度为3的 $\chi^2$ 分布。

设 D 的密度函数为 $K(D)$ ， $T_{D}$ 为某一阈值，在无故障条件下（用 $H_{0}$ 表示）D 超越 $T_{D}$ 的概率为 $\alpha$ ，即

$$
P _ {r} [ (D > T _ {D}) / H _ {0} ] = \alpha
$$

如果所取 $\alpha$ 很小，则在无故障条件下 D 超越 $T_{D}$ 是小概率事件。而小概率事件在一次实现中几乎是不可能出现的，所以 D 一旦超越了 $T_{D}$ ，就可以有把握地断定这决不可能是在无故障条件下发生的，而只可能是在有故障条件下发生的。因此故障判决准则为：

若 $D > T_{D}$ ，则判定测量组中至少有一个陀螺出故障；

若 $D \leqslant T_{D}$ , 则判定测量组中所有陀螺工作都正常。

若已检测出陀螺测量组有故障，并设故障向量为 $b_{f}$ ，则式(8.7.14)成

$$
\mathbf {Z} = \mathbf {H} \boldsymbol {\omega} + \boldsymbol {\varepsilon} + \mathbf {b} _ {f}
$$

若只有第 j 个陀螺出故障, 并记该事件为 $H_{j}$ , 则

$$
\boldsymbol {b} _ {f} = \boldsymbol {e} _ {j} f\tag{8.7.19}
$$

式中 $e_{j}$ 表示与Z同维的向量，它的第j个分量为1，而其余分量全为零，f表示故障的大小，是确定性标量。

根据式(7.4.32)，在 $H_{j}$ 情况下的故障隔离判决函数为

$$
D _ {j} = \frac {\left[ \boldsymbol {P} ^ {\mathrm{T}} \boldsymbol {V} _ {j} \right] ^ {2}}{\| \boldsymbol {V} _ {j} \|}\tag{8.7.20}
$$

式中， $V_{j}$ 为矩阵 V 的第 j 列， $\|V_{j}\|=V_{j}^{T}V_{j}$ 为 $V_{j}$ 的范数。

按式(8.7.20)分别计算 $D_{1}, D_{2}, \cdots, D_{6}$ ，若 $D_{k}$ （k 为 1, 2, …, 6 中的某一确定值）为最大，则判定第 k 个陀螺有故障。

根据式(7.4.29)，故障量 $f$ 的极大似然估计为

$$
\hat {f} = \frac {\boldsymbol {P} ^ {\mathrm{T}} \boldsymbol {V} _ {j}}{\| \boldsymbol {V} _ {j} \|}\tag{8.7.21}
$$

在计算机仿直分析中，环境条件假设如下：捷联惯导包含6个单轴激光陀螺，各陀螺敏感轴分别沿正12面体的法线，如图7.4.5所示；各陀螺的随机常值漂移已得到补偿，无故障时的量测误差为零均值白噪声，均方根为 $\sigma_{m}$ ；在各时间段内，仅有一个陀螺发生软故障，故障情况如表8.7.3所列。故障量分别为 $0.05^{\circ} / \mathrm{h}, 0.06^{\circ} / \mathrm{h}, 0.07^{\circ} / \mathrm{h}, 0.08^{\circ} / \mathrm{h}, 0.09^{\circ} / \mathrm{h}$ 和 $0.1^{\circ} / \mathrm{h}$ 。表8.7.4和表8.7.5列出了量测噪声分别为 $0.01^{\circ} / \mathrm{h}$ 和 $0.001^{\circ} / \mathrm{h}$ 时的仿真计算结果，其中漏检率为有故障时间段内未报警时间所占的比例，误警率为无故障时间段内错误报警时间所占的比例，故障估值 $\hat{f}$ 为按式(8.7.21)计算得的估计值在故障时间段内的算术平均值。

表 8.7.3 捷联陀螺发生软故障的时间段

<table><tr><td>陀螺号</td><td>1</td><td>2</td><td>3</td><td>4</td><td>5</td><td>6</td></tr><tr><td>故障时间段 /s</td><td>20~50</td><td>100~150</td><td>200~225</td><td>300~330</td><td>400~460</td><td>550~630</td></tr></table>

表 8.7.4 量测噪声均方根 $\sigma_{m}=0.01^{\circ}/h$ 时故障诊断结果

<table><tr><td>故障量 $f/(^{\circ})\cdot h^{-1}$ </td><td>0.05</td><td>0.06</td><td>0.07</td><td>0.08</td><td>0.09</td><td>0.10</td></tr><tr><td>漏检率/(%)</td><td>4.37</td><td>1.13</td><td>0.14</td><td>0</td><td>0</td><td>0</td></tr><tr><td>误警率/(%)</td><td>8.73</td><td>8.73</td><td>8.73</td><td>8.73</td><td>8.73</td><td>8.73</td></tr><tr><td>故障定位准确率/(%)</td><td>96.90</td><td>97.89</td><td>99.44</td><td>99.86</td><td>100</td><td>100</td></tr><tr><td>故障量估计值 $\hat{f}/(^{\circ})\cdot h^{-1}$ </td><td>0.0515</td><td>0.0565</td><td>0.0698</td><td>0.0966</td><td>0.0922</td><td>0.105</td></tr><tr><td> $\hat{f}$ 的相对误差/(%)</td><td>2.97</td><td>5.83</td><td>0.33</td><td>15.23</td><td>2.43</td><td>5.00</td></tr></table>

表 8.7.5 量测噪声均方根 $\sigma_{m}=0.001^{\circ}/h$ 时故障诊断结果

<table><tr><td>故障量 $f/( (^{\circ}) \cdot h^{-1})$ </td><td>0.05</td><td>0.06</td><td>0.07</td><td>0.08</td><td>0.09</td><td>0.10</td></tr><tr><td>漏检率/(%)</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td><td>0</td></tr><tr><td>误警率/(%)</td><td>4.08</td><td>4.08</td><td>4.08</td><td>4.08</td><td>4.08</td><td>4.08</td></tr><tr><td>故障定位准确率/(%)</td><td>100</td><td>100</td><td>100</td><td>100</td><td>100</td><td>100</td></tr><tr><td>故障量估计值 $\hat{f}/( (^{\circ}) \cdot h^{-1})$ </td><td>0.0512</td><td>0.0595</td><td>0.0695</td><td>0.0820</td><td>0.0890</td><td>0.1002</td></tr><tr><td> $\hat{f}$ 的相对误差/(%)</td><td>2.33</td><td>0.76</td><td>0.66</td><td>2.45</td><td>1.18</td><td>0.20</td></tr></table>

## 附录

附录 A 最小二乘估计和加权最小二乘估计的推导

A1. 矩阵与矩阵间的微分关系

设有矩阵 $C_{s\times r},B_{p\times q}$ ，则

$$
\frac {\mathrm{d} \boldsymbol {C}}{\mathrm{d} \boldsymbol {B}} = \left[ \begin{array}{c c c c} \frac {\partial \boldsymbol {C}}{\partial b _ {1 1}} & \frac {\partial \boldsymbol {C}}{\partial b _ {1 2}} & \dots & \frac {\partial \boldsymbol {C}}{\partial b _ {1 q}} \\ \frac {\partial \boldsymbol {C}}{\partial b _ {2 1}} & \frac {\partial \boldsymbol {C}}{\partial b _ {2 2}} & \dots & \frac {\partial \boldsymbol {C}}{\partial b _ {2 q}} \\ \vdots & \vdots & & \vdots \\ \frac {\partial \boldsymbol {C}}{\partial b _ {p 1}} & \frac {\partial \boldsymbol {C}}{\partial b _ {p 2}} & \dots & \frac {\partial \boldsymbol {C}}{\partial b _ {p q}} \end{array} \right]\tag{A - 1}
$$

式中 $b_{ij}(i=1,2,\cdots,p;\quad j=1,2,\cdots,q)$ 为 B 的元。

设 $X=\left[x_{1}\quad x_{2}\quad\cdots\quad x_{n}\right]^{T}$

(1) 若 C=X, $B=X^{T}$ , 则

$$
\frac {\mathrm{d} \boldsymbol {X}}{\mathrm{d} \boldsymbol {X} ^ {\mathrm{T}}} = \left[ \begin{array}{l l l l} \frac {\partial \boldsymbol {X}}{\partial x _ {1}} & \frac {\partial \boldsymbol {X}}{\partial x _ {2}} & \dots & \frac {\partial \boldsymbol {X}}{\partial x _ {n}} \end{array} \right] = \left[ \begin{array}{l l l l} \boldsymbol {e} _ {1} & \boldsymbol {e} _ {2} & \dots & \boldsymbol {e} _ {n} \end{array} \right] = \boldsymbol {I} _ {n}\tag{A-2}
$$

式中 $e_{i}(i=1,2,\cdots,n)$ 是仅第 i 个元为 1 而其余元全为零的列向量。

(2) 若 $C = X^{T}$ , B = X, 则

$$
\frac {\mathrm{d} \boldsymbol {X} ^ {\mathrm{T}}}{\mathrm{d} \boldsymbol {X}} = \left[ \begin{array}{c} \frac {\partial \boldsymbol {X} ^ {\mathrm{T}}}{\partial x _ {1}} \\ \frac {\partial \boldsymbol {X} ^ {\mathrm{T}}}{\partial x _ {2}} \\ \vdots \\ \frac {\partial \boldsymbol {X} ^ {\mathrm{T}}}{\partial x _ {n}} \end{array} \right] = \left[ \begin{array}{c} \boldsymbol {e} _ {1} ^ {\mathrm{T}} \\ \boldsymbol {e} _ {2} ^ {\mathrm{T}} \\ \vdots \\ \boldsymbol {e} _ {n} ^ {\mathrm{T}} \end{array} \right] = \boldsymbol {I} _ {n}\tag{A-3}
$$

(3) 若 C=X, B=X，则

表 8.7.6 各子系统的故障设置和两种检测法的检测结果

<table><tr><td>子系统</td><td>故障变量</td><td> $\frac{\text{故障时间段}}{\text{s}}$ </td><td>故障类型及大小</td><td>状态检测告警时间 /s</td><td>残差检测告警时间 /s</td></tr><tr><td rowspan="5">GPS</td><td> $\lambda/(')$ </td><td>20~50</td><td>突变 0.3</td><td>20~50</td><td>20~52*</td></tr><tr><td> $L/(')$ </td><td>100~140</td><td>慢变  $0.0008(t-100)^{2}$ </td><td>115~171</td><td>109~142</td></tr><tr><td> $v_{E}/(m·s^{-1})$ </td><td>300~330</td><td>突变 3.0</td><td>300~335</td><td>300~333*</td></tr><tr><td>h/m</td><td>600~650</td><td>突变 640</td><td>601~650</td><td>600~653*</td></tr><tr><td> $v_{N}/(m·s^{-1})$ </td><td>1000~1060</td><td>慢变 0.15(t-1000)</td><td>1021~1036</td><td>1001~1064</td></tr><tr><td>DVS</td><td> $v_{d}/(m·s^{-1})$ </td><td>500~540</td><td>突变 2.0</td><td>505~540</td><td>500~550*</td></tr><tr><td rowspan="3">ADS</td><td> $v_{wbE}/(m·s^{-1})$ </td><td>900~930</td><td>突变 2.8</td><td>919~982</td><td>900~930*</td></tr><tr><td> $v_{wbN}/(m·s^{-1})$ </td><td>360~385</td><td>慢变  $0.0018(t-360)^{2}$ </td><td>374~402</td><td>360~388*</td></tr><tr><td> $h_{b}/m$ </td><td>1200~1240</td><td>慢变  $1.2(t-1200)^{2}$ </td><td>1200~1301</td><td>1200~1254</td></tr><tr><td>MG</td><td> $\Psi/(')$ </td><td>70~80</td><td>突变 0.3</td><td>70~96</td><td>70~81</td></tr><tr><td rowspan="2">VG</td><td> $\varphi_{VGx}/(^{\circ})$ </td><td>200~215</td><td>慢变  $0.0001(t-200)^{2}$ </td><td>204~227</td><td>201~237</td></tr><tr><td> $\varphi_{VGy}/(^{\circ})$ </td><td>800~825</td><td>突变 0.005</td><td>800~877</td><td>800~835</td></tr></table>

![](images/9b4674ef04a763bded3a31a4d1e98dafa074575add948cc4c1b7c21670eb5a04.jpg)  
图 8.7.6 故障隔离后经融合获得的纬度估计误差

![](images/1ce500222779d4dded1fec5eb8ccf69315416d6985f17f807e1682b84eb03fd6.jpg)  
图 8.7.7 故障隔离后经融合获得的高度估计误差

## 2. 具有联邦滤波结构的组合导航系统的容错性能分析

采用联邦滤波结构设计组合导航系统的目的有两个:一是分散和减少卡尔曼滤波计算量,以便在有很多非相似导航子系统参与组合的情况下,仍能保证组合导航系统实时工作;二是能在滤波过程中,很方便地对各导航子系统作故障检测、诊断和隔离,并能方便地对组合导航系统进行重构,以确保组合导航系统可靠地工作。现根据7.3节中所介绍的系统级故障检测与隔离理论,对8.7.1节所讨论的组合导航系统作容错性能分析。

假设捷联惯导采用惯性器件的余度配置技术后能可靠地获得载体的全姿态信息；在计算机设计等方面也采用容错设计技术。并假设采用这些容错设计后，惯导系统能可靠地工作。

组合导航系统按照8.7.1节中所介绍的方法设计，即：联邦滤波器采用无复位结构，公共参考系统为激光陀螺捷联惯导系统，其余非相似导航子系统分别为GPS C/A码接收机、多普勒雷达、大气数据计算机、垂直陀螺、磁航向仪，惯导信息只在初始时刻作分配，分配系数均等，即 $\beta_{i}=\frac{1}{5}(i=1,2,\cdots,5)$ 。所设置的各子系统的故障分两类：一类是突变形，即系统的误差在原来基础上以突跳形式增加到某一值；另一类是慢变形，即系统误差自某一时刻起在原来基础上以直线或抛物线增长关系逐渐增长。

故障检测算法采用 7.3 节中所介绍的状态 $\chi^{2}$ 检验法和残差 $\chi^{2}$ 检验法。这些算法可确定出各子滤波器工作是否正常。由于联邦滤波结构中，子滤波器 i 仅由惯导和导航子系统 $i(i=1,2,\cdots,5)$ 构成，而经过惯导本身的容错设计，其可靠性已得到保证，所以一旦子滤波器 i 出故障，即可断定导航子系统 i 出故障。

如果检测出某子滤波器有故障,则按 7.2 节中所介绍的融合算法中,主滤波器拒绝接纳该子滤波器的输出,而仅对无故障子滤波器的输出作融合计算,即可实现对故障子系统的有效隔离,这实际上也就是对组合导航系统作了重构。

表 8.7.6 列出了各导航子系统出现故障的时间段和故障的具体形式,以及采用状态 $\chi^{2}$ 检验法和残差 $\chi^{2}$ 检验法时的告警情况。其中告警时间一栏中,带 \* 号者表示告警时间段内存在漏检,即告警是间歇性的。

从表 8.7.6 可看出，残差 $\chi^{2}$ 检验法与状态 $\chi^{2}$ 检验法相比较，前者对故障的响应比后者灵敏，即告警延迟量小，而且后者算法中，除用到卡尔曼滤波器的自然输出量外，还须计算仅由时间更新确定的卡尔曼滤波值（见式（7.3.2）和式（7.3.3）），这无疑增加了额外的计算量；而前者算法中，所用到的残差是卡尔曼滤波过程中自然存在的量，不必另行计算。但前者存在漏检的缺陷。为了克服这一问题，可对残差 $\chi^{2}$ 检验法作安全观察处理，即在停止告警后的某一段时间内仍拒绝接纳该子滤波器的输出，直到这一安全观察期满而未出现过告警才开始接收。图 8.7.6 和图 8.7.7 为表 8.7.6 所示故障环境下，故障隔离后融合获得的纬度估计误差和高度估计误差。图 8.7.6 中，虚线为同时采用残差 $\chi^{2}$ 检验法和状态 $\chi^{2}$ 检验法隔离故障后的结果，实线为采用残差 $\chi^{2}$ 检验法并经 90 s 安全观察隔离故障后的结果；图 8.7.7 中，虚线为仅采用残差 $\chi^{2}$ 检验法隔离故障后的结果，实线为除采用残差 $\chi^{2}$ 检验法还经 90 s 安全观察隔离故障后的结果。

由图 8.7.6 和图 8.7.7 诸曲线可看出，采用残差 $\chi^{2}$ 检验法并经一定时间的安全观察能有效克服残差 $\chi^{2}$ 检验法漏检引起的故障渗透，融合精度接近彻底清除故障影响时的精度。并且，由于残差 $\chi^{2}$ 检验法所用数据来自滤波计算，不必另行计算，所以此法是有效隔离故障子系统较为合适的方法。

$$
\frac {\mathrm{d} \boldsymbol {X}}{\mathrm{d} \boldsymbol {X}} = \left[ \begin{array}{l} \frac {\partial \boldsymbol {X}}{\partial x _ {1}} \\ \frac {\partial \boldsymbol {X}}{\partial x _ {2}} \\ \vdots \\ \frac {\partial \boldsymbol {X}}{\partial x _ {n}} \end{array} \right] = \left[ \begin{array}{l} \boldsymbol {e} _ {1} \\ \boldsymbol {e} _ {2} \\ \vdots \\ \boldsymbol {e} _ {n} \end{array} \right] = \mathrm{cs} (\boldsymbol {I} _ {n})\tag{A - 4}
$$

式中 $\mathrm{cs}(I_{n})$ 表示由 $I_{n}$ 的各列顺序串接成的列向量。

(4) 若 $C = X^{T}$ , $B = X^{T}$ , 则

$$
\frac {\mathrm{d} \boldsymbol {X} ^ {\mathrm{T}}}{\mathrm{d} \boldsymbol {X} ^ {\mathrm{T}}} = \left[ \begin{array}{l l l l} \frac {\partial \boldsymbol {X} ^ {\mathrm{T}}}{\partial x _ {1}} & \frac {\partial \boldsymbol {X} ^ {\mathrm{T}}}{\partial x _ {2}} & \dots & \frac {\partial \boldsymbol {X} ^ {\mathrm{T}}}{\partial x _ {n}} \end{array} \right] = \left[ \begin{array}{l l l l} \boldsymbol {e} _ {1} ^ {\mathrm{T}} & \boldsymbol {e} _ {2} ^ {\mathrm{T}} & \dots & \boldsymbol {e} _ {n} ^ {\mathrm{T}} \end{array} \right] = \operatorname{rs} (\boldsymbol {I} _ {n})\tag{A - 5}
$$

其中 $\mathrm{rs}(I_{n})$ 表示由 $I_{n}$ 的各行顺序串接成的行向量。

A2. 矩阵积与矩阵间的微分关系

(1) 克朗尼克积。设有矩阵 $C_{m \times n}$ ， $D_{p \times q}$ ，则 C 对 D 的克朗尼克积为

$$
\boldsymbol {C} \otimes \boldsymbol {D} = \left[ \begin{array}{c c c c} C _ {1 1} \boldsymbol {D} & C _ {1 2} \boldsymbol {D} & \dots & C _ {1 n} \boldsymbol {D} \\ C _ {2 1} \boldsymbol {D} & C _ {2 2} \boldsymbol {D} & \dots & C _ {2 n} \boldsymbol {D} \\ \vdots & \vdots & & \vdots \\ C _ {m 1} \boldsymbol {D} & C _ {m 2} \boldsymbol {D} & \dots & C _ {m n} \boldsymbol {D} \end{array} \right]\tag{A - 6}
$$

其中 $C_{ij}(i=1,2,\cdots,m,\quad j=1,2,\cdots,n)$ 为 C 的元。

(2) 积的微分。设有矩阵 $C_{n \times r}$ ， $D_{r \times m}$ ， $B_{p \times q}$ ，A = CD，则

$$
\begin{array}{r l} \frac {\mathrm{d} \boldsymbol {A}}{\mathrm{d} \boldsymbol {B}} = & \frac {\mathrm{d} (\boldsymbol {C D})}{\mathrm{d} \boldsymbol {B}} = \left[ \begin{array}{c c c} \frac {\partial \boldsymbol {C}}{\partial b _ {1 1}} & \dots & \frac {\partial \boldsymbol {C}}{\partial b _ {1 q}} \\ \vdots & & \vdots \\ \frac {\partial \boldsymbol {C}}{\partial b _ {p 1}} & \dots & \frac {\partial \boldsymbol {C}}{\partial b _ {p q}} \end{array} \right] \left[ \begin{array}{c c c} \boldsymbol {D} & & \boldsymbol {0} \\ & \boldsymbol {D} & \\ & & \ddots \\ \boldsymbol {0} & & \boldsymbol {D} \end{array} \right] + \\ & \left[ \begin{array}{c c c} \boldsymbol {C} & & \boldsymbol {0} \\ & \boldsymbol {C} & \\ & & \ddots \\ \boldsymbol {0} & & \boldsymbol {C} \end{array} \right] \left[ \begin{array}{c c c} \frac {\partial \boldsymbol {D}}{\partial b _ {1 1}} & \dots & \frac {\partial \boldsymbol {D}}{\partial b _ {1 q}} \\ \vdots & & \vdots \\ \frac {\partial \boldsymbol {D}}{\partial b _ {p 1}} & \dots & \frac {\partial \boldsymbol {D}}{\partial b _ {p q}} \end{array} \right] = \\ & \frac {\mathrm{d} \boldsymbol {C}}{\mathrm{d} \boldsymbol {B}} (\boldsymbol {I} _ {q} \otimes \boldsymbol {D}) + (\boldsymbol {I} _ {p} \otimes \boldsymbol {C}) \frac {\mathrm{d} \boldsymbol {D}}{\mathrm{d} \boldsymbol {B}} \end{array}\tag{A-7}
$$

式(A-7)中 $b_{ij}(i=1,2,\cdots,p;j=1,2,\cdots,q)$ 为B的元。

A3. 最小二乘估计和加权最小二乘估计的推导

对于加权最小二乘估计,指标函数最优的条件为

$$
J (\hat {\mathbf {X}}) = (\mathbf {Z} - \mathbf {H} \hat {\mathbf {X}}) ^ {\mathrm{T}} \mathbf {W} (\mathbf {Z} - \mathbf {H} \hat {\mathbf {X}}) = \min
$$

要使上式成立, $\hat{X}$ 应满足:

$$
\left. \frac {\partial J (\boldsymbol {X})}{\partial \boldsymbol {X}} \right| _ {\boldsymbol {X} = \dot {\boldsymbol {X}}} = \boldsymbol {0}
$$

取

$$
\begin{array}{l} \mathbf {C} = (\mathbf {Z} - \mathbf {H X}) ^ {\mathrm{T}} \mathbf {W} \\ \mathbf {D} = \mathbf {Z} - \mathbf {H X} \end{array}
$$

则 $J(X)=CD$ ，根据式(A-7)有

$$
\begin{array}{r l} & {\frac {\mathrm{d} J (\mathbf {X})}{\mathrm{d} \mathbf {X}} = \frac {\mathrm{d} [ (\mathbf {Z} - \mathbf {H X}) ^ {\mathrm{T}} \mathbf {W} ]}{\mathrm{d} \mathbf {X}} [ \mathbf {I} _ {1} \otimes (\mathbf {Z} - \mathbf {H X}) ] +} \\ & {[ \mathbf {I} _ {n} \otimes (\mathbf {Z} - \mathbf {H X}) ^ {\mathrm{T}} \mathbf {W} ] \frac {\mathrm{d} (\mathbf {Z} - \mathbf {H X})}{\mathrm{d} \mathbf {X}} = - \frac {\mathrm{d} \mathbf {X} ^ {\mathrm{T}}}{\mathrm{d} \mathbf {X}} \mathbf {H} ^ {\mathrm{T}} \mathbf {W} (\mathbf {Z} - \mathbf {H X}) +} \\ & {\left[ (Z - H X) ^ {\mathrm{T}} W \quad 0 \quad \dots \quad 0 \right.} \\ & {\left. 0 \quad (Z - H X) ^ {\mathrm{T}} W \quad \dots \quad 0 \right.} \\ & {\left. \vdots \quad \vdots \quad \dots \quad \vdots \right.} \\ & {\left. 0 \quad 0 \quad \dots \quad (Z - H X) ^ {\mathrm{T}} W \right]} \end{array}
$$

根据式(A-7)

$$
\frac {\mathrm{d} (\boldsymbol {H X})}{\mathrm{d} \boldsymbol {X}} = \mathbf {0} + (\boldsymbol {I} _ {n} \otimes \boldsymbol {H}) \mathrm{cs} (\boldsymbol {I} _ {n}) = \left[ \begin{array}{c c c c} \boldsymbol {H} & & & \mathbf {0} \\ & \boldsymbol {H} & & \\ & & \ddots & \\ \mathbf {0} & & & \boldsymbol {H} \end{array} \right] \left[ \begin{array}{c} \boldsymbol {e} _ {1} \\ \boldsymbol {e} _ {2} \\ \vdots \\ \boldsymbol {e} _ {n} \end{array} \right]
$$

所以

$$
\frac {\mathrm{d} J (\boldsymbol {X})}{\mathrm{d} \boldsymbol {X}} = - \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} (\boldsymbol {Z} - \boldsymbol {H X}) - \left[ \begin{array}{c c c c} (\boldsymbol {Z} - \boldsymbol {H X}) ^ {\mathrm{T}} \boldsymbol {W} & \boldsymbol {0} & \dots & \boldsymbol {0} \\ \boldsymbol {0} & ((\boldsymbol {Z} - \boldsymbol {H X}) ^ {\mathrm{T}} \boldsymbol {W} & \dots & \boldsymbol {0} \\ \vdots & \vdots & & \vdots \\ \boldsymbol {0} & \boldsymbol {0} & \dots & (\boldsymbol {Z} - \boldsymbol {H X}) ^ {\mathrm{T}} \boldsymbol {W} \end{array} \right]
$$

$$
\left[ \begin{array}{c c c c} \boldsymbol {H} & & & \boldsymbol {0} \\ & \boldsymbol {H} & & \\ & & \ddots & \\ \boldsymbol {0} & & & \boldsymbol {H} \end{array} \right] \quad \left[ \begin{array}{c} \boldsymbol {e} _ {1} \\ \boldsymbol {e} _ {2} \\ \vdots \\ \boldsymbol {e} _ {n} \end{array} \right] =
$$

$$
- \mathbf {H} ^ {\mathrm{T}} \mathbf {W} (\mathbf {Z} - \mathbf {H X}) - \left[ \begin{array}{c} (\mathbf {Z} - \mathbf {H X}) ^ {\mathrm{T}} \mathbf {W H e} _ {1} \\ (\mathbf {Z} - \mathbf {H X}) ^ {\mathrm{T}} \mathbf {W H e} _ {2} \\ \vdots \\ (\mathbf {Z} - \mathbf {H X}) ^ {\mathrm{T}} \mathbf {W H e} _ {n} \end{array} \right]
$$

由于 $Z \in R^{m}$ , $W_{m \times m}$ , $H_{m \times n}$ , $e_{i} \in R^{n} (i=1,2,\cdots,n)$ ，所以 $(\mathbf{Z}-\mathbf{H}\mathbf{X})^{\mathrm{T}}\mathbf{W}\mathbf{H}\mathbf{e}_{i} (i=1,2,\cdots,n)$ 为标量，转置后值不变，即有

$$
(\mathbf {Z} - \mathbf {H X}) ^ {\mathrm{T}} \mathbf {W H e} _ {i} = \mathbf {e} _ {i} ^ {\mathrm{T}} \mathbf {H} ^ {\mathrm{T}} \mathbf {W} ^ {\mathrm{T}} (\mathbf {Z} - \mathbf {H X})
$$

因此

$$
\frac {\mathrm{d} J (\boldsymbol {X})}{\mathrm{d} \boldsymbol {X}} = - \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} (\boldsymbol {Z} - \boldsymbol {H} \boldsymbol {X}) - \left[ \begin{array}{c} \boldsymbol {e} _ {1} ^ {\mathrm{T}} \\ \boldsymbol {e} _ {2} ^ {\mathrm{T}} \\ \vdots \\ \boldsymbol {e} _ {n} ^ {\mathrm{T}} \end{array} \right] \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} ^ {\mathrm{T}} (\boldsymbol {Z} - \boldsymbol {H} \boldsymbol {X}) = - \boldsymbol {H} ^ {\mathrm{T}} (\boldsymbol {W} + \boldsymbol {W} ^ {\mathrm{T}}) (\boldsymbol {Z} - \boldsymbol {H} \boldsymbol {X}) = \mathbf {0}
$$

从中解得

$$
\hat {\mathbf {X}} = \left[ \mathbf {H} ^ {\mathrm{T}} (\mathbf {W} + \mathbf {W} ^ {\mathrm{T}}) \mathbf {H} \right] ^ {- 1} \mathbf {H} ^ {\mathrm{T}} (\mathbf {W} + \mathbf {W} ^ {\mathrm{T}}) \mathbf {Z}\tag{A - 8}
$$

若加权阵取成对称阵，即 $W^{\mathrm{T}} = W$ ，则

$$
\hat {\boldsymbol {X}} = \left(\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {H}\right) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {Z}\tag{A - 9}
$$

若加权阵为单位阵，即 W = I，则

$$
\hat {\mathbf {X}} = (\mathbf {H} ^ {\mathrm{T}} \mathbf {H}) ^ {- 1} \mathbf {H} ^ {\mathrm{T}} \mathbf {Z}\tag{A - 10}
$$

这就是一般最小二乘估计。

## 附录 B 矩阵反演公式的推导

B1. 分块三角矩阵的求逆

设有分块三角矩阵 $A=\begin{bmatrix}A_{11}^{m\times m}&A_{12}^{m\times n}\\0^{n\times m}&A_{22}^{n\times n}\end{bmatrix}$

$$
\mathbf {A} \mathbf {A} ^ {- 1} = \left[ \begin{array}{c c} \mathbf {A} _ {1 1} & \mathbf {A} _ {1 2} \\ \mathbf {0} & \mathbf {A} _ {2 2} \end{array} \right] \left[ \begin{array}{c c} \mathbf {A} _ {1 1} ^ {- 1} & - \mathbf {A} _ {1 1} ^ {- 1} \mathbf {A} _ {1 2} \mathbf {A} _ {2 2} ^ {- 1} \\ \mathbf {0} & \mathbf {A} _ {2 2} ^ {- 1} \end{array} \right] = \left[ \begin{array}{c c} \mathbf {I} _ {m \times m} & \mathbf {0} _ {m \times n} \\ \mathbf {0} _ {n \times m} & \mathbf {I} _ {n \times n} \end{array} \right] = \mathbf {I} _ {(m + n) \times (m + n)}
$$

所以

$$
\left[ \begin{array}{c c} \mathbf {A} _ {1 1} & \mathbf {A} _ {1 2} \\ \mathbf {0} & \mathbf {A} _ {2 2} \end{array} \right] ^ {- 1} = \left[ \begin{array}{c c} \mathbf {A} _ {1 1} ^ {- 1} & - \mathbf {A} _ {1 1} ^ {- 1} \mathbf {A} _ {1 2} \mathbf {A} _ {2 2} ^ {- 1} \\ \mathbf {0} & \mathbf {A} _ {2 2} ^ {- 1} \end{array} \right]\tag{B-1}
$$

若分块三角矩阵为 $A=\begin{bmatrix}A_{11}^{m\times m}&0^{m\times n}\\A_{21}^{n\times m}&A_{22}^{n\times n}\end{bmatrix}$

$$
\mathbf {A} \mathbf {A} ^ {- 1} = \left[ \begin{array}{c c} \mathbf {A} _ {1 1} & \mathbf {0} \\ \mathbf {A} _ {2 1} & \mathbf {A} _ {2 2} \end{array} \right] \left[ \begin{array}{c c} \mathbf {A} _ {1 1} ^ {- 1} & \mathbf {0} \\ - \mathbf {A} _ {2 2} ^ {- 1} \mathbf {A} _ {2 1} \mathbf {A} _ {1 1} ^ {- 1} & \mathbf {A} _ {2 2} ^ {- 1} \end{array} \right] = \left[ \begin{array}{c c} \mathbf {I} _ {m \times m} & \mathbf {0} _ {m \times n} \\ \mathbf {0} _ {n \times m} & \mathbf {I} _ {n \times n} \end{array} \right] = \mathbf {I} _ {(m + n) \times (m + n)}
$$

所以

$$
\left[ \begin{array}{c c} \mathbf {A} _ {1 1} & \mathbf {0} \\ \mathbf {A} _ {2 1} & \mathbf {A} _ {2 2} \end{array} \right] ^ {- 1} = \left[ \begin{array}{c c} \mathbf {A} _ {1 1} ^ {- 1} & \mathbf {0} \\ - \mathbf {A} _ {2 2} ^ {- 1} \mathbf {A} _ {2 1} \mathbf {A} _ {1 1} ^ {- 1} & \mathbf {A} _ {2 2} ^ {- 1} \end{array} \right]\tag{B-2}
$$

B2. 一般矩阵的分块三角阵分解

设 $A=\begin{bmatrix}A_{11}^{m\times m}&A_{12}^{m\times n}\\A_{21}^{n\times m}&A_{22}^{n\times n}\end{bmatrix}_{(m+n)\times(m+n)}$

其中 $A_{11}^{-1}$ 和 $A_{22}^{-1}$ 存在，则 A 可分解成：

$$
\mathbf {A} = \left[ \begin{array}{c c} \mathbf {I} _ {m \times m} & \mathbf {0} \\ \mathbf {A} _ {2 1} \mathbf {A} _ {1 1} ^ {- 1} & \mathbf {I} _ {n \times n} \end{array} \right] \left[ \begin{array}{c c} \mathbf {A} _ {1 1} & \mathbf {A} _ {1 2} \\ \mathbf {0} & \mathbf {A} _ {2 2} - \mathbf {A} _ {2 1} \mathbf {A} _ {1 1} ^ {- 1} \mathbf {A} _ {1 2} \end{array} \right]\tag{B-3}
$$

或

$$
\mathbf {A} = \left[ \begin{array}{c c} \mathbf {I} _ {m \times m} & \mathbf {A} _ {1 2} \mathbf {A} _ {2 2} ^ {- 1} \\ \mathbf {0} & \mathbf {I} _ {n \times n} \end{array} \right] \left[ \begin{array}{c c} \mathbf {A} _ {1 1} - \mathbf {A} _ {1 2} \mathbf {A} _ {2 2} ^ {- 1} \mathbf {A} _ {2 1} & \mathbf {0} \\ \mathbf {A} _ {2 1} & \mathbf {A} _ {2 2} \end{array} \right]\tag{B-4}
$$

B3. 矩阵反演公式

根据式(B-3)

$$
\mathbf {A} ^ {- 1} = \left[ \begin{array}{c c} \mathbf {A} _ {1 1} & \mathbf {A} _ {1 2} \\ \mathbf {0} & \mathbf {A} _ {2 2} - \mathbf {A} _ {2 1} \mathbf {A} _ {1 1} ^ {- 1} \mathbf {A} _ {1 2} \end{array} \right] ^ {- 1} \left[ \begin{array}{c c} \mathbf {I} _ {m \times m} & \mathbf {0} \\ \mathbf {A} _ {2 1} \mathbf {A} _ {1 1} ^ {- 1} & \mathbf {I} _ {n \times n} \end{array} \right] ^ {- 1}
$$

上式右侧求逆分别应用式(B-1)和式(B-2)，

$$
\mathbf {A} ^ {- 1} = \left[ \begin{array}{c c} \mathbf {A} _ {1 1} ^ {- 1} & - \mathbf {A} _ {1 1} ^ {- 1} \mathbf {A} _ {1 2} (\mathbf {A} _ {2 2} - \mathbf {A} _ {2 1} \mathbf {A} _ {1 1} ^ {- 1} \mathbf {A} _ {1 2}) ^ {- 1} \\ \mathbf {0} & (\mathbf {A} _ {2 2} - \mathbf {A} _ {2 1} \mathbf {A} _ {1 1} ^ {- 1} \mathbf {A} _ {1 2}) ^ {- 1} \end{array} \right] \left[ \begin{array}{c c} \mathbf {I} _ {m \times m} & \mathbf {0} \\ - \mathbf {A} _ {2 1} \mathbf {A} _ {1 1} ^ {- 1} & \mathbf {I} _ {n \times n} \end{array} \right] =
$$

$$
\left[ \begin{array}{c c} A _ {1 1} ^ {- 1} + A _ {1 1} ^ {- 1} A _ {1 2} (A _ {2 2} - A _ {2 1} A _ {1 1} ^ {- 1} A _ {1 2}) ^ {- 1} A _ {2 1} A _ {1 1} ^ {- 1} & - A _ {1 1} ^ {- 1} A _ {1 2} (A _ {2 2} - A _ {2 1} A _ {1 1} ^ {- 1} A _ {1 2}) ^ {- 1} \\ - (A _ {2 2} - A _ {2 1} A _ {1 1} ^ {- 1} A _ {1 2}) ^ {- 1} A _ {2 1} A _ {1 1} ^ {- 1} & (A _ {2 2} - A _ {2 1} A _ {1 1} ^ {- 1} A _ {1 2}) ^ {- 1} \end{array} \right]\tag{B-5}
$$

根据式(B-4)

$$
\boldsymbol {A} ^ {- 1} = \left[ \begin{array}{c c} \boldsymbol {A} _ {1 1} - \boldsymbol {A} _ {1 2} \boldsymbol {A} _ {2 2} ^ {- 1} \boldsymbol {A} _ {2 1} & \boldsymbol {0} \\ \boldsymbol {A} _ {2 1} & \boldsymbol {A} _ {2 2} \end{array} \right] ^ {- 1} \left[ \begin{array}{c c} \boldsymbol {I} _ {m \times m} & \boldsymbol {A} _ {1 2} \boldsymbol {A} _ {2 2} ^ {- 1} \\ \boldsymbol {0} & \boldsymbol {I} _ {n \times n} \end{array} \right] ^ {- 1}
$$

上式右侧求逆分别应用式(B-2)和式(B-1)，

$$
\begin{array}{r l} \boldsymbol {A} ^ {- 1} = & \left[ \begin{array}{c c} (\boldsymbol {A} _ {1 1} - \boldsymbol {A} _ {1 2} \boldsymbol {A} _ {2 2} ^ {- 1} \boldsymbol {A} _ {2 1}) ^ {- 1} & \boldsymbol {0} \\ - \boldsymbol {A} _ {2 2} ^ {- 1} \boldsymbol {A} _ {2 1} (\boldsymbol {A} _ {1 1} - \boldsymbol {A} _ {1 2} \boldsymbol {A} _ {2 2} ^ {- 1} \boldsymbol {A} _ {2 1}) ^ {- 1} & \boldsymbol {A} _ {2 2} ^ {- 1} \end{array} \right] \left[ \begin{array}{c c} \boldsymbol {I} _ {m \times m} & - \boldsymbol {A} _ {1 2} \boldsymbol {A} _ {2 2} ^ {- 1} \\ \boldsymbol {0} & \boldsymbol {I} _ {n \times n} \end{array} \right] = \\ & \left[ \begin{array}{c c} (\boldsymbol {A} _ {1 1} - \boldsymbol {A} _ {1 2} \boldsymbol {A} _ {2 2} ^ {- 1} \boldsymbol {A} _ {2 1}) ^ {- 1} & - (\boldsymbol {A} _ {1 1} - \boldsymbol {A} _ {1 2} \boldsymbol {A} _ {2 2} ^ {- 1} \boldsymbol {A} _ {2 1}) ^ {- 1} \boldsymbol {A} _ {1 2} \boldsymbol {A} _ {2 2} ^ {- 1} \\ - \boldsymbol {A} _ {2 2} ^ {- 1} \boldsymbol {A} _ {2 1} (\boldsymbol {A} _ {1 1} - \boldsymbol {A} _ {1 2} \boldsymbol {A} _ {2 2} ^ {- 1} \boldsymbol {A} _ {2 1}) ^ {- 1} & \boldsymbol {A} _ {\text {22}} ^ {- 1} \boldsymbol {A} _ {\text {21}} (\boldsymbol {A} _ {\text {11}} - \boldsymbol {A} _ {\text {12}} \boldsymbol {A} _ {\text {22}} ^ {- 1} \boldsymbol {A} _ {\text {21}}) ^ {- 1} \boldsymbol {A} _ {\text {12}} \boldsymbol {A} _ {\text {22}} ^ {- 1} + \boldsymbol {A} _ {\text {22}} ^ {- 1} \end{array} \right] \end{array}\tag{B-6}
$$

比较式(B-5)和式(B-6)左上角分块,得

$$
\left(\boldsymbol {A} _ {1 1} - \boldsymbol {A} _ {1 2} \boldsymbol {A} _ {2 2} ^ {- 1} \boldsymbol {A} _ {2 1}\right) ^ {- 1} = \boldsymbol {A} _ {1 1} ^ {- 1} + \boldsymbol {A} _ {1 1} ^ {- 1} \boldsymbol {A} _ {1 2} \left(\boldsymbol {A} _ {2 2} - \boldsymbol {A} _ {2 1} \boldsymbol {A} _ {1 1} ^ {- 1} \boldsymbol {A} _ {1 2}\right) ^ {- 1} \boldsymbol {A} _ {2 1} \boldsymbol {A} _ {1 1} ^ {- 1}
$$

此即矩阵求逆反演公式(2.1.23)式。

## 参考文献

1 俞济祥. 卡尔曼滤波及其在惯性导航中的应用. 北京: 航空专业教材编审组, 1984.

2 Wiener N. The extrapolation, interpolation and smoothing of stationary time series. OSRD370, Report to the Services 19, research project DIC-6037, MIT, 1942.

3 Kalmogorov A N. Interpolation and extrapolation von stationaren zufalligen folgen. Bull Acad. Sci. USSR, Ser. Math. 1941, 5: 3-14.

4 Bode H W, Shannon C E. A simplified derivation of linear least square smoothing and predication theory. Proc. IRE, 1950, 38:417-425.

5 Zadeh L A, Ragazzini J R. An extension of Wiener's theory of prediction. Journal of Appli. Phys, 1950, 21: 645-655.

6 Shinbrot M. Optimization of time-varying linear systems with nonstationary inputs. Trans. ASME, 1958, 80:457-462.

7 Swerling P. First order error propagation in a stagewise smoothing procedure for satellite observations. Journal of Astronautical Science, 1959, 6:46-52.

8 Kalman R E. A new approach to linear filtering and prediction problems. Journal of Basic Eng(ASME), 1960, 82D: 35-46.

9 Kalman R E, Bucy R S. New results in linear filtering and prediction theory. Journal of Basic Eng (ASME), 1961, 83D: 95-108.

10 Blum M. A stagewise parameter estimation procedure for correlated data. Numer. Math, 1961, 3: 202-208.

11 Battin R H. A statistical optimizing navigation procedure for space flight. ARS Journal, 1962, 32: 1681-1696.

12 Kalman R E. New methods in Wiener filtering theory. New York: John Wiley & Sons Inc, 1963.

13 Schmidt S F. The Kalman filter: Its recognition and development for aerospace application. Journal of Guidance and Control, 1981, 4 (1).

14 Schmidt S F, Lukesh J S. Application of Kalman filtering for the C-5 guidance and control system. AD-704306, 1970.

15 Bierman G J. Sequential square root filtering and smoothing of discrete linear systems. Automation, 1974, 10: 147-158.

16 Carlson N A. Fast triangular factorization of the square root filter. AIAA Journal, 1973, 11(9): 1259-1265.

17 Kaminski P G, Bryson A E, Schmidt S F. Discrete square root filtering: a survey of current techniques. IEEE Trans. on Automatic Control, 1971, AC-16 (6): 727-735.

18 Andrews A. A square root formulation of the Kalman covariance equations. AIAA Journal, 1968, 6: 1165-1166.

19 Schmidt S F. Computational techniques in Kalman filtering, in theory and applications of Kalman filtering. NATO Advisory Group for Aerospace Research and Development, AGARDOGRAPH 139, 1970.

20 Dyer P, McReynolds S. Extension of square root filtering to include process noise. Journal of Optimize Theory Appli, 1969, 3 (6): 444-459.

21 Bierman G J. Measurement updating using U-D factorization. Proc. IEEE Conf. on Decision and Control, Houston, TX, 1975: 337-346.

22 Thornton C L, Bierman G J, Schmidt G. Algorithms for covariance propagation. Proc. IEEE Conf. on Decision and Control, Houston, TX, 1975, 489–498.

23 Leonds C T. Control and dynamic systems. New York: Academic press Inc, 1980, 16: 177-247.

24 Sorenson H W, Stubberud A R. Nonlinear filtering by approximation of the aposteriori density. Int. Journal of Control, 1968, 18:33–51.

25 Phaneuf R J. Approximate nonlinear estimation. [Ph. D. Thesis]. MIT: 1968.

26 Sunahara Y. An approximate method of state estimation for nonlinear dynamical systems. Joint Automatic Control Conf, Univ. of Colorado, 1969.

27 Wishner R P, Tabaezynski J A, Athans M. A comparison of three nonlinear filters. Automatica, 1969, 5: 457-496.

28 Leondes C T, Peller J B, Stear E B. Nonlinear smoothing theory. IEEE Trans. Systems Science and Cybernetics, 1970, SSC6(1).

29 Bucy R S, Renne K D. Digital synthesis of nonlinear filters. Automatica, 1971, 7(3): 287-289.

30 Figucircdo R J D, Jan Y G. Spline filters. Proc. 2nd Symp. on Nonlinear Estimation Theory and Its Applications, San Diego, 1971, 127–141.

31 Gelb A, Warren R S. Direct statistical analysis of nonlinear systems. Proc. AIAA Guidance And Control Conf, Palo Alto, 1972.

32 Bizwas K K, Mahatanabis A K. Suboptimal algorithms for nonlinear smoothing. IEEE Trans. on Aerospace and Electronic Systems, 1973, AES-9(4): 529-534.

33 Mehra R K. On-line identification of linear dynamic systems with applications to Kalman filtering. IEEE Trans. on Automatic Control, 1971, AC-16(1): 12-21.

34 Speyer J L. Computation and transmission requirements for a decentralized linear - quadratic - Gaussian control problem. IEEE Trans. on Automatic Control, 1979, AC-24 (2): 266-269.

35 Chang T S. Comments on computation and transmission requirements for a decentralized linear-quadratic-Gaussian control. IEEE Trans. on Automatic Control, 1980, AC-25(3).

36 Willsky A S, Bello M G, Castanon D A, Levy B C, Verghese G C. Combining and updating of local estimates and regional maps along sets of one-dimensional tracks. IEEE Trans. on Automatic Control, 1982, AC-27(4): 799-813.

37 Levy B C, Castanon D A, Verghese G C, Willsky A S. A scattering framework for decentralized estimation problems. Automatica, 1983, 49(4).

38 Castanon D A, Teneketzis D. Distributed estimation algorithms for nonlinear systems. IEEE Trans. on Automatic Control, 1985, AC-30.

39 Kerr T H. Decentralized filtering and redundancy management for multisensor navigation. IEEE Trans. on Aerospace and Electronic Systems, 1987, AES-23(1): 83-119.

40 Bierman G J, Belzer M R. A decentralized square root information filter/smoother. Proc. of 24th IEEE Conf. on Decision and Control, Ft. Lauderdale, FL, 1985.

41 Carlson N A. Federated square root filter for decentralized parallel processes. Proc. NAECON, Dayton, OH, 1987. 1448–1456.

42 Carlson N A. Federated filter for fault - tolerant integrated navigation systems. Proc. of IEEE PLANS' 88, Orlando, FL 1988, 110-119.

43 Loomis P V W, Carlson N A, Berarducci M P. Common Kalman filter: fault - tolerant navigation for

next generation aircraft. Proc. of the Inst. of Navigation Conf, Santa Barbara, CA, 1988. 38–45.

44 Covino J M, Griffiths B E. A new estimation architecture for multisensor data fusion. Proc. of the International Symposium and Exhibition on Optical Engineering and Photonics, Orlando, FL, 1991.

45 Carlson N A. Federated filter for fault - tolerant integrated navigation. AGARDOGRAPH, Aerospace Navigation Systems, 1993.

46 Carlson N A, Berarducci M P. Federated Kalman filter simulation results. Navigation Journal of ION, 1994, 41(3): 297-321.

47 Chin P P. Real time Kalman filtering of Apollo LM/AGS rendezvous radar data. AIAA Guidance and Control and Flight Mechanics Conference, Santa Barbara, CA, 1970.

48 Schmidt S F, Weinberg J D, Lukesh J S. Case study of Kalman filtering in the C-5 aircraft navigation system. Case Studies in System Control, Univ. of Michigan, 1968, 57-109.

49 Schmidt S F, Lukesh J S. Application of Kalman filtering of the C-5 guidance and control system. AD -704306, 1970, 289-334.

50 Schmidt G T, Brock L D. General questions on Kalman filtering in navigation systems. AD-704306, 1970, 205-230.

51 Huddle J R. Application of Kalman filtering theory to augmented inertial navigation systems. AD-704306, 1970, 231-268.

52 Barham P M, Manville P. Application of Kalman filtering to baro/inerital height systems. AD-704306, 1970, 269-287.

53 Stein K J. Self-contained avionics broaden scope of C-5 missions. Proc. Aviation Week and Space Technology, 1967, 87(21):192-202.

54 Smith G L. Application of statistical filter theory to the optimal estimation of position and velocity on board a circumlunar vehicle. NASA Technical Report R-135, Washington DC, 1962.

55 Lukesh J S. Simulation of the nortronics Kalman filter - augmented C-5 inertial Doppler navigator. Proc. NAECON, Dayton, OH, 1968, 171-180.

56 Brown R G, Friest D T. Optimization of a hybrid inertial solar - tracker navigation system. IEEE International Convention Record, 1964, 12(7): 121-135.

57 Bona B E, Smay Q J. Optimum reset of ship's inertial navigation system. IEEE Trans. on Aerospace and Electronic Systems, 1966, AES-2(4): 409-414.

58 Dworetzky L H, Edwards A. Principles of Doppler-inertial guidance. American Rocket Society Journal, 1959, 967–972.

59 Wilford J N. We reach the moon. New York: W W Norton & Co. Inc, 1972.

60 郑政谋. 最佳线性滤波. 北京：航空专业教材编审组，1983.

61 秦永元，俞济祥. 直升机海面悬停时精确高度的维纳滤波提取法. 航空学报, 1994, 15(11): 1341-1347.

62 Bendat J S. Principles and applications of random noise theory. New York: John Wiley and Sons Inc, 1968.

63 Sweeting D. Some design aspects of the WG13 rigid rotor helicopter. AGARD-CP-86-71.

64 Bierman G J. Factorization methods for discrete sequential estimation. New York: Academic Press, 1977.

65 Grewal M S, Miyasako R S, Smith J M. Application of fixed point smoothing to the calibration, alignment and navigation data of inertial navigation systems. Proc. of IEEE PLANS'88, Orlando, FL, 1988, 476–479.

66 Watanabe K. New computationally efficient formula for backward-pass fixed-interval smoother and its UD

factorisation algorithm. IEE Proc, 1989, 136Pt. D(2):73-78.

67 Deyst J J, Price C F. Conditions for asymptotic stability of the discrete minimum variance linear estimations. IEEE Trans. on Automatic Control, 1968, AC-13: 702-705.

68 Jazwinski A H. Stochastic processes and filtering theory. New York, Academic Press, 1970.

69 中科院数学所概率组. 离散时间系统滤波的数学方法. 北京: 国防工业出版社, 1975.

70 Anderson B D O. Stability properties of Kalman-Buccy filters. Journal of Franklin Inst, 1971, 291(2).

71 张仲俊, 杨翠莲. 陀螺角速度漂移数学模型识别. 上海: 上海交通大学科技资料, 1977.

72 Sinha N K, Kuszta B. Modeling and identification of dynamic systems. New York: Van Nostrand, 1985.

73 Reddy P B. On stationary and nonstationary models of long term random errors of gyroscopes and accelerometers in an inertial navigation system. Proc. of Conf. on Decision and Control, and 16th Symposium on Adaptive Processes, and Special Symposium on Fuzzy Set Theory and Applications, 1977, 1.

74 Reddy P B. On identification of nonstationary random noise models. Proc. of 8th Modeling and Simulation Conf., Univ. of Pittsburgh, 1977.

75 Mehra R K. Approaches to adaptive filtering. IEEE Trans. on Automatic Control, 1972, AC-17(5): 693-698.

76 Ashjaee J M, Bourn D R, Helkey R J etc. Ashtech XII GPS receiver, the ALL-IN-ONE ALL-IN-VIEW. Proc. of IEEE PLANS'88, Orlando, FL, 1988, 426-433.

77 TSO-C129, Airborne supplemental navigation equipment using the global positioning system (GPS). Washington DC: Department of Transportation, Federal Aviation Administration, Aircraft Certification Service, 1992.

78 Conley R. GPS Performance: What is normal? Navigation, 1993, 30(3):261-281.

79 郑建平, 宋光普, 等. GPS 导航仪的反 SA 方法研究. 导航, 1994, 30(1):23-32.

80 秦永元. SA 建模的功率谱分析法. 西北工业大学学报, 1996, 14(3): 454-457.

81 Schneider A M, Maida J L. A Kalman filter for an integrated Doppler/GPS navigation system. Proc. of IEEE PLANS88, Orlando, FL, 1988, 408-415.

82 West—Vukovich G, Zywiel J, Scherzinger B. The Honeywell/DND helicopter integrated navigation system (HINS). Proc. of IEEE PLANS88, Orlando, FL, 1988. 416–425.

83 Huddle J R. Application of Kalman filtering theory to augmented inertial navigation systems. AD-704306, 1970, 231-268.

84 魏光顺，郑玉簋，张欲敏. 无线电导航原理. 南京：东南大学出版社，1987.

85 Whitcomb L A. Using low cost magnetic sensors on magnetically hostile land vehicles. Proc. of IEEE PLANS88, Orlando, FL, 1988, 34–38.

86 Roberts B A, Vallot L C. Vision aided inertial navigation. Proc. of IEEE PLANS'90, Las Vegas, NV, 1990, 347–352.

87 Britting K R. Inertial navigation systems analysis. New York: Wiley-Interscience, 1971.

88 以光衢，等．惯性导航原理．北京：航空工业出版社，1987.

89 崔中兴. 惯性导航系统. 北京: 国防工业出版社, 1982.

90 郭富强,于波,汪叔华.陀螺稳定装置及其应用.西安:西北工业大学出版社,1995.

91 Waner J C. Practical considerations in implementing Kalman filters. AD-A024377, 1976.

92 Vanallen R L, Brown R G. An incremental velocity measurement algorithm for use in inertial navigation alignment. AD-738025, 1972.

93 Brock L D, Schmidt G T. General questions on Kalman filtering in navigation systems. AD - 704306, 1970.

94 夏天长. 系统辨识(最小二乘法). 北京: 清华大学出版社, 1983.

95 Panuska V. A new from of the extended Kalman filter for parameter estimation in linear systems with correlated noise. IEEE Trans. on Automatic Control, 1980, AC-25(2): 229-235.

96 俞济祥. 惯性导航系统各种传递对准方法讨论. 航空学报, 1988, 9(5): A211-A217.

97 Ross C C, Elbert T F. A Transfer alignment algorithm study based on actual flight test data from a tactical air-to-ground weapon launch. Proc. of IEEE PLANS94, Las Vegas, NV, 1994, 431-438.

98 Rogers R M. Weapon IMU transfer alignment using aircraft position from actual tests. Proc. of IEEE PLANS96, Atlanta, GA, 1996, 328-335.

99 Hepburn J S A, Deyle C P. Motion compensation for ASTOR long range SAR. Proc. of IEEE PLANS' 90, Las Vegas, NV, 1990, 205–211.

100 Spalding K. An efficient rapid transfer alignment filter. Proc. of AIAA Guidance, Navigation and Control Conf, 1992, 1276–1286.

101 Reiner J. In-flight-transfer-alignment using aircraft-to-wing stiff-angle estimation, Proc. of 36th Israel Annual Conf. on Aerospace Sciences, 1996, A96-22520: 01-05.

102 Graham W, Shortelle K. Advanced transfer alignment for inertial navigation (A - TRAIN). Proc. of ION National Technical Meeting. Anaheim, CA, 1995, 113-124.

103 Kain J E, Cloutier J R. Rapid transfer alignment for tactical weapon applications. AIAA Guidance, Navigation and Control Conf. 1989, 290–300.

104 秦永元. 捷联惯导系统初始对准的参数辨识法. 中国惯性技术学报, 1990(2): 1-16.

105 秦永元. 捷联惯导系统参数辨识法对准中的预滤波处理. 西北工业大学学报, 1991, 9(4): 501-508.

106 秦永元, 牛惠芳. 余度斜置捷联惯导单个陀螺软故障的诊断和修复. 导航, 1997, 33(1): 78-82.

107 Pearson J D. Dynamic decomposition techniques in optimization methods for large-scale system. McGraw-Hill, 1971.

108 Wei M, Schwarz K P. Testing a decentralized filter for GPS/INS integration. Proc of IEEE PLANS'90, Las Vegas, NV, 1990, 429–439.

109 Gao Y etc. Comparison of centralized and federated filters. Navigation, 1992, 40(1): 69-86.

110 袁信,俞济祥,陈哲.导航系统.北京:航空工业出版社,1992.

111 Ren Da. Failure detection of dynamic systems with the state chi-square test. Journal of Guidance, Control and Dynamics, 1994, 17(2): 271-277.

112 Zhang H G, Zhang H Y. Fault tolerant scheme for multi-sensor navigation systems. Proc. of 18th Congress of the International Council of the Aeronautical Sciences, Beijing, 1992, 669–678.

113 袁信，郑鄂. 捷联式惯性导航系统. 北京：航空专业教材编审组，1985.

114 Hamrrison J V, Gai E G. Evaluation sensor orientations for navigation performance and failure detection. IEEE Trans. on Aerospace and Electronic Systems, 1977, AES-13(6):631-643.

115 Sweetman B. Military navigation: the forth generation. Inetravia Aerospace Review, 1990, 767–775.

116 Potter J E, Suman M C. Thresholdless redundancy management with arrays of skewed instrument. AGARDOGR APH224, 1977, 15-1-15-25.

117 Hall S R etc. In-flight parity vector compensation for FDI. IEEE Trans. on Aerospace and Electronic Systems, 1983, AES-19(5): 668-675.

118 Rauch H E, Tung T & Striebel C T. Maximum Likelihood Estimates of Linear Dynamic Systems, AIAA' Journal, 1965, 3(8): 1445-1450.

119 Dan Simon. Optimal State Estimation—Kalman, H∞ and Nonlinear Approaches. John Wiley & Sons, 2006.

120 秦永元. 惯性导航. 科学出版社, 2014.

121 Simon Haykim. Kalman Filtering and Neural Networks. John Wiley & Sons, 2001.

122 Doucet A, de Freitas N, Gordon N. Sequential Monte Carlo Methods in practice. Springer - Verlay, New York, 2001.

123 Gray J, Murray W. A derivation of an analytical expression for the tracking index for the alpha-beta-gamma filtering. IEEE Trans. on Aerospace and Electronic Systems, 1993, 29(3): 1064-1065.

124 Isaard M, Blake A. Contour tracking by stochastic propagation of conditional density. European Conference on Computer Vision, 1996, 343–356.

125 MacCormick J, Blake A. A probabilistic exclusion principle for tracking multiple objects. International Conference on Computer Vision, 1999, 572–578.

126 Moral P. Measure valued processes and interacting particle systems: Application to non-linear filtering problems. Annals of Applied Probability, 1998, 8(2):438-495.

127 Kitagawa G. Monte Carlo filter and smoother for non-Gaussian nonlinear state space models. Journal of Computational and Graphical Statistics, 1996,5(1): 1-25.

128 Andrieu C, Doucet A, Singh S, Tadic V. Particle methods for change detection, system identification and control. Proceedings of the IEEE, 2004, 92(3): 428-438.

129 Crisan D, Doucet A. A survey of convergence results on particle filtering methods for practitioners. IEEE Trans. on Signal processing, 2002, 50(3):736-746.

130 Wiener N. I Am a Mathematician. MIT Press, Cambridge, Massachusetts, 1956.

131 Poor V, Looze D. Minimax state estimation for linear stochastic systems with noise uncertainty. IEEE Trans. on Automatic Control, 1981, AC-26(4): 902-906.

132 Darragh J, Looze D. Noncausal minimax linear state estimation for Systems with uncertain second-order statistics. IEEE Trans. on Automatic Control, 1984, AC-29(6):555-557.

133 Verdu S, Poor H. Minimax liner observers and regulators for Stochastic Systems with uncertain Second-order Statistics. IEEE Trans. on Automatic Control, 1984, AC-29(6):499-511.

134 Grimble M. H $_{\infty}$ design of optimal linear filters, in Linear Circuits, Systems and Signal Processing: Theory and Applications, C. Byrnes, C. Matin and R. Sacks (Eds.) North-Holland, Amsterdam, the Netherlands, 1998:533-540.

135 Grimble M., El Sayed A. Solution of the $H_{\infty}$ optimal linear filtering problem for discrete-time systems. IEEE Trans. on Acoustics, Speech and Signal Processing, 1990, 38(7):1092-1104.

136 Hassibi B, Kailath T. H∞ adaptive filtering. IEEE International Conference on Acoustics, Speech and Signal Processing, 1995:949–952.

137 Simon D, El-Sherief H. Hybrid Kalman/minimax filtering in phase-locked loops. Control Engineering Practice, 1996,4:615-623.

138 Kailath T, Sayed A, Hassibi B. Linear Estimation. Prentice - Hall, Upper Saddle River, New Jersey, 2000.

139 Athans M, Wisher R, Bertolini A. Suboptimal state estimation algorithm for continuous-time nonlinear systems from discrete measurements. IEEE Transactions on Automatic Control, 1968, AC-13(5): 504-515.

140 Efron B. The Bootstrap Jacknife and other Resampling Plans. Philadelphia: SIAM, 1982.

141 Rubin D B. Using the SIR algorithm to simulate posterior distributions. Bayesian Statistics 3, Oxford University Press, 1998, 395-402.

142 Smith A F M, Gelfand A E. Bayesian Statics without tears: a sampling - resampling perspective. American Statistician, 1992, 46:84-88.

143 MacKay D. A practical Bayesian framework for back-propagation networks. Neural Computation, 1992, 4: 448-472.

144 Higuchi T. Monte Carlo filter using the genetic algorithm operators. Journal of Statistical Computation and Simulation, 1997, 59: 1-23.
