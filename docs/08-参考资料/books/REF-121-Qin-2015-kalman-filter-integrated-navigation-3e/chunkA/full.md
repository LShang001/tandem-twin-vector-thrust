# 卡尔曼滤波与组合导航原理

# (第3版)

秦永元 张洪钺 汪叔华◎编著

![](images/5e3b8f0736f4c16b78339fb8ccadd1fd10abd91f00d6f66746bbf1906d8914a5.jpg)  
西北工業大學出版社

# 卡尔曼滤波与组合导航原理（第3版）

策划编辑/杨军

责任编辑/杨军

封面设计/盛熙

# 卡尔曼滤波与组合导航原理

(第3版)

秦永元 张洪钺 汪叔华 编著

【内容简介】本书是《卡尔曼滤波与组合导航原理》的第3版。书中着重阐述了卡尔曼滤波基本理论，以及近10年发展起来的有关卡尔曼滤波的新理论和新方法，容错组合导航设计理论和方法，另外还有作者的部分科研成果。内容安排上力求循序渐进，由浅入深，确保知识连贯。为便于读者理解概念内涵，公式和定理一般都附有详细推导和证明。

本书是高等学校控制、导航专业研究生通用教科书，也可作为相关专业高年级学生及研究工作者的参考书。

## 图书在版编目(CIP)数据

卡尔曼滤波与组合导航原理/秦永元, 张洪钺, 汪叔华编著. —3 版. —西安: 西北工业大学出版社, 2015.2

ISBN 978-7-5612-4350-3

I. ①卡…
II. ①秦…
②张…
③汪…
III. ①卡尔曼滤波
②组合导航
IV. ①O211.64
②TN967.2

中国版本图书馆 CIP 数据核字(2015)第 047467 号

出版发行:西北工业大学出版社

通信地址:西安市友谊西路127号 邮编:710072

电话：(029)88493844 88491757

网址: www.nwpup.com

印刷者:兴平市博闻印务有限公司

开 本:787 mm×1092 mm 1/16

印 张:25.5

字 数:622 千字

版次:2015年6月第3版

定 价:58.00元

2015年6月第1次印刷

## 前言

卡尔曼滤波是对随机信号作估计的算法之一。与最小二乘、维纳滤波等诸多估计算法相比，卡尔曼滤波具有显著的优点：采用状态空间法在时域内设计滤波器，用状态方程描述任何复杂多维信号的动力学特性，避开了在频域内对信号功率谱作分解带来的麻烦，滤波器设计简单易行；采用递推算法，实时量测信息经提炼被浓缩在估计值中，而不必存储时间过程中的量测量。所以，卡尔曼滤波能适用于白噪声激励的任何平稳或非平稳随机向量过程的估计，所得估计在线性估计中精度最佳。正由于其独特的优点，卡尔曼滤波在20世纪60年代初一经提出，立即受到工程界，特别是空间技术和航空界的高度重视。阿波罗登月计划中的导航系统设计和C-5A飞机的多模式导航系统的设计是卡尔曼滤波早期应用中最为成功的实例。随着计算机技术的发展，目前卡尔曼滤波的应用几乎涉及通讯、导航、遥感、地震测量、石油勘探、经济和社会学研究等所有领域。

本书是航空工业总公司组织编写的研究生用统编教材。全书着重阐述卡尔曼滤波基本理论，近十年发展起来的有关卡尔曼滤波的新理论和新方法。此外，还专门介绍了与组合导航系统设计理论和方法有关的内容以及作者的部分研究成果。本书的内容既使读者能全面、系统地了解卡尔曼滤波理论，又为组合导航系统的设计提供了完整的设计理论和方法，也为卡尔曼滤波理论在其他领域中的应用提供了研究方法上的参考和借鉴。

本书共分八章。第一至第七章系统介绍卡尔曼滤波的基本理论，包括滤波基本方程，滤波中的技术处理，滤波稳定性分析，广义滤波理论，滤波器的容错设计理论等；第八章介绍滤波理论在组合导航系统设计中的应用，包括各种常用导航子系统的介绍，卡尔曼滤波在惯性导航系统对准和容错组合导航系统设计中的应用。在内容阐述上注意到知识的连贯性，力求做到循序渐进，由浅入深。为了使读者能从本质上理解概念的内涵，公式和定理的提出一般都经过详细的推导和证明。

本书是在西北工业大学俞济祥教授主编的《卡尔曼滤波及其在惯性导航中的应用》[1]一书的基础上撰写的，由秦永元主编。各章编写分工：第一至第五章、第八章由西北工业大学秦永元编写，第六章由南京航空航天大学汪叔华编写，第七章由北京航空航天大学张洪钺编写。本书承蒙空军工程学院张宗麟教授详细审阅并提出了许多宝贵意见。在此谨致深切谢意。

编著者

1998年2月

## 第3版说明

本书利用再次列入出版计划的机会,在第2版基础上作了如下修订:

(1)3.11节中，增写了 $H_{\circ}$ 滤波驻点方程(3.11.10a)式的详细推导；6.2节中增加了二阶广义卡尔曼滤波(二阶EKF)的介绍和推导，并以雷达测距系统确定空气中自由落体的运动参数为例说明该型滤波器的设计方法；6.4节中细化了粒子滤波二次采样的介绍和分析；增写了附录B，详细推导了矩阵反演公式。

(2) 更正了第 2 版中的印刷错误。

本书第 3 版工作由秦永元完成。

希望经过第 3 版淘洗提高本书的含金量,获得更多读者的接受,更好地为教学和科研服务。

编著者

2014年12月

## 再版说明

自 1998 年本书第 1 版出版以来的 13 年间, 大量的最优估计新理论和新方法涌现了出来。为适应工程应用需要, 更好地为国民经济和国防建设服务, 笔者在第 1 版基础上作了修改和增添, 完成了本书的编写。本次修订的主要内容如下:

(1)校正了第1版中的印刷错误。对固定滞后平滑、固定区间平滑、信息滤波、 $\alpha-\beta-\gamma$ 滤波和传递对准中姿态信息匹配量构造方法等作了改写。

(2) 增加了 $H_{\infty}$ 滤波、UKF 非线性滤波、粒子滤波、估计均方误差阵及最佳增益阵同解形式的介绍和推导。

本书再版工作由秦永元完成。

由于水平和时间有限,再版后还会存在不尽如人意之处,敬请广大读者批评指正。

2011年12月

编著者

## 目录

第一章 绪论 …… 1
1.1 卡尔曼滤波所要解决的问题 …… 1
1.2 卡尔曼滤波理论的发展和工程应用 …… 2
1.3 组合导航简介 …… 3

第二章 几种最优估计和卡尔曼滤波基本方程 …… 5
2.1 几种最优估计 …… 5
2.2 离散型卡尔曼滤波 …… 33
2.3 连续型卡尔曼滤波 …… 57
习题 …… 68

第三章 卡尔曼滤波中的技术处理 …… 71
3.1 有色噪声的白化 …… 71
3.2 有色噪声条件下的卡尔曼滤波 …… 79
3.3 序贯处理 …… 84
3.4 信息滤波 …… 86
3.5 卡尔曼滤波发散的抑制 …… 87
3.6 平方根滤波 …… 93
3.7 UDU $^{T}$ 分解滤波 …… 103
3.8 自适应滤波 …… 118
3.9 次优滤波 …… 121
3.10 卡尔曼滤波误差分析 …… 129
3.11 H $_{\infty}$ 滤波 …… 139
习题 …… 153

第四章 卡尔曼滤波稳定性介绍 …… 156
4.1 稳定性定义 …… 156
4.2 判别卡尔曼滤波稳定的充分条件 …… 157
4.3 适用于惯导系统的滤波稳定判别条件 …… 167
习题 …… 169

第五章 滤波系统的校正 …… 170
5.1 概述 …… 170

5.2 离散系统的分离定理 …… 171
5.3 连续系统的分离定理 …… 175
5.4 离散系统的估计直接反馈控制 …… 176
5.5 连续系统的估计直接反馈控制 …… 180
习题…… 180

第六章 卡尔曼滤波的推广…… 182
6.1 最优线性平滑 …… 182
6.2 非线性系统滤波之一: EKF …… 199
6.3 非线性系统滤波之二: UKF …… 221
6.4 非线性系统滤波之三: 粒子滤波…… 231
习题…… 238

第七章 容错组合导航的设计理论…… 239
7.1 概述 …… 239
7.2 联邦滤波器算法原理 …… 240
7.3 系统级故障检测与隔离的原理与方法 …… 254
7.4 惯性器件的故障检测与隔离原理 …… 262
7.5 故障的统计检测原理与风险分析 …… 277

第八章 卡尔曼滤波理论在组合导航系统设计中的应用…… 287
8.1 概述 …… 287
8.2 组合导航系统的设计模式 …… 288
8.3 组合导航系统设计中一些常用导航子系统的误差模型 …… 292
8.4 惯性导航系统的误差模型 …… 316
8.5 卡尔曼滤波理论在惯导系统初始对准中的应用 …… 335
8.6 应用基本滤波理论设计组合导航系统 …… 362
8.7 应用联邦滤波理论设计容错组合导航系统 …… 377

附录…… 388
附录 A 最小二乘估计和加权最小二乘估计的推导 …… 388
附录 B 矩阵反演公式的推导 …… 391

参考文献…… 393

## 第一章 绪论

## 1.1 卡尔曼滤波所要解决的问题

所谓滤波就是从混合在一起的诸多信号中提取出所需要的信号。

信号是传递和运载信息的时间或空间函数。有一类信号的变化规律是既定的，如调幅广播中的载波信号、阶跃信号、脉宽固定的矩形脉冲信号等，它们都具有确定的频谱，这类信号称为确定性信号。另一类信号没有既定的变化规律，在相同的初始条件和环境条件下，信号的每次实现都不一样，如陀螺漂移、海浪、作水平飞行的飞机飞越山区时无线电高度表的输出信号、惯导系统的导航输出误差、GPS的SA误差等，它们没有确定的频谱，这类信号称为随机信号。

由于确定性信号具有确定的频谱,所以可根据各信号频带的不同,设置具有相应频率特性的滤波器,如低通、高通、带通、带阻滤波器,使有用信号无衰减地通过,使干扰信号受到抑制。这类滤波器可用物理方法实现,此即模拟滤波器,也可用计算机通过算法实现,此即数字滤波器。对确定性信号的滤波处理也称常规滤波。

随机信号没有确定的频谱，无法用常规滤波提取或抑制信号，但随机信号具有确定的功率谱，所以可根据有用信号和干扰信号的功率谱设计滤波器。维纳滤波是解决此类问题的方法之一。但设计维纳滤波器须作功率谱分解，只有当被处理信号为平稳的，干扰信号和有用信号均为一维，且功率谱为有理分式时，维纳滤波器的传递函数才可用伯特-香农设计法较容易地求解出，否则设计维纳滤波器存在着诸多困难。维纳滤波除设计思想与常规滤波不同外，对信号作抑制和选通这一点是相似的。

卡尔曼滤波从与被提取信号有关的量测量中通过算法估计出所需信号。其中被估计信号是由白噪声激励引起的随机响应，激励源与响应之间的传递结构（系统方程）已知，量测量与被估计量之间的函数关系（量测方程）也已知。估计过程中利用了如下信息：系统方程、量测方程、白噪声激励的统计特性、量测误差的统计特性。由于所用信息都是时域内的量，所以卡尔曼滤波器是在时域内设计的，且适用于多维情况。这就完全避免了维纳滤波器在频域内设计遇到的限制和障碍，适用范围远比维纳滤波器广。

从以上简述中可看出卡尔曼滤波有如下特点：

(1) 卡尔曼滤波处理的对象是随机信号；

(2) 被处理信号无有用和干扰之分, 滤波的目的是要估计出所有被处理信号;

（3）系统的白噪声激励和量测噪声并不是需要滤除的对象，它们的统计特性正是估计过程中需要利用的信息。

所以确切地说,卡尔曼滤波应称作最优估计理论,此处称谓的滤波与常规滤波具有完全不同的概念和含意。

就实现形式而言，卡尔曼滤波器实质上是一套由数字计算机实现的递推算法，每个递推周期中包含对被估计量的时间更新和量测更新两个过程。时间更新由上一步的量测更新结果和设计卡尔曼滤波器时的先验信息确定，量测更新则在时间更新的基础上根据实时获得的量测值确定。因此，量测量可看做卡尔曼滤波器的输入，估计值可看做输出。输入与输出之间由时间更新和量测更新算法联系，这与数字信号处理概念是类似的，所以有些书上称卡尔曼滤波为广义数字信号处理似有一定道理。

## 1.2 卡尔曼滤波理论的发展和工程应用

随机信号没有既定的变化规律，对它们的估计也不可能完全准确，所谓最优估计也仅仅是指在某一准则下的最优。根据不同的最优准则，可获得随机信号的不同最优估计。使贝叶斯风险达到最小的估计为贝叶斯估计；使关于条件概率密度的似然函数达到极大的估计为极大似然估计；使验后概率密度达到极大的估计为极大验后估计；使估计误差的均方误差达到最小的估计为最小方差估计，若估计具有线性形式，则估计为线性最小方差估计，卡尔曼滤波即属此类估计。

卡尔曼滤波理论的创立是科学技术和社会需要发展到一定程度的必然结果。早在1795年，高斯(Karl Gauss)为测定行星运动轨道而提出了最小二乘估计法。20世纪40年代，为了解决火力控制系统精确跟踪问题，维纳(N. Weaner)于1942年提出了维纳滤波理论[2]。维纳根据有用信号和干扰信号的功率谱确定出线性滤波器的频率特性，首次将数理统计理论与线性系统理论有机地联系在一起，形成了对随机信号作平滑、估计或预测的最优估计新理论。比维纳稍早，前苏联科学家戈尔莫克洛夫(A.N.Kolmogorov)于1941年也曾提出过类似理论[3]。维纳给出了由功率谱求解维纳滤波器频率特性闭合解的一般方法，包括对功率谱的上、下平面分解及傅里叶变换和反变换，运算繁杂，解析求解十分困难。1950年，伯特和香农给出了在功率谱为有理谱这一特殊条件下，由功率谱直接求取维纳滤波器传递函数的设计方法[4]，这一方法简单易行，具有一定的工程实用价值。维纳滤波的最大缺点是适用范围极其有限，它要求被处理信号必须是平稳的，且是一维的。人们试图将维纳滤波推广到非平稳和多维的情况[5-6]，都因无法突破计算上的困难而难以推广和应用。

采用频域设计法是造成维纳滤波器设计困难的根本原因。因此人们逐渐转向寻求在时域内直接设计最优滤波器的新方法 $^{[7-12]}$ 。其中卡尔曼的研究最具有代表性，他提出的递推最优估计理论也因此而被称为卡尔曼滤波。由于采用了状态空间法描述系统，算法采用递推形式，所以卡尔曼滤波能处理多维和非平稳的随机过程。

卡尔曼滤波理论一经提出，立即受到了工程界的重视，而工程应用中遇到的实际问题又使卡尔曼滤波的研究更深入更完善。1959年起美国太空署即NASA(National Aeronautics and Space Administration)开始研究载人太空船登月方案，当时提出了两个主要问题：①中途导航和制导；②液体燃料助推器大挠度条件下的自动驾驶问题。因这两项研究的工作量都很庞大，无力同时进行，所以选择前者作为重点，即宇宙飞船的测轨问题。导航问题中主要解决对太空船运动状态的估计。量测信息来自三个子系统：飞船装备的惯性测量装置和天文观测仪，地面测轨系统，测轨数据经数据链传送至太空船。估计方法曾试图采用递推加权最小二乘和维纳滤波，均因精度满足不了要求和计算过于繁杂而不得不放弃。1960年秋，卡尔曼访问了

NASA, 提出了卡尔曼滤波算法, 立即引起重视并投入研究。由于最初提出的卡尔曼滤波仅适用于线性系统, 而实际系统是非线性系统, 滤波初值应如何取才合理, 这些都迫使卡尔曼作进一步的思考, 广义卡尔曼滤波就是在此情况下提出来的。阿波罗计划中的导航系统后由麻省理工学院研制完成[13]。卡尔曼滤波早期应用中的另一成功实例为 C-5A 飞机的多模式导航系统[14]。

卡尔曼滤波比维纳滤波的应用范围广，设计方法也简单易行得多，但它必须在计算机上执行，而20世纪60年代初，无论是速度、字长、容量等方面，计算机还处于低水平阶段。为了适应当时的技术水平，避免由于字长不够产生的舍入误差引起卡尔曼滤波的计算发散，Bierman, Carlson和Schmidt等人提出了平方根滤波算法和 $UDU^{T}$ 分解滤波算法，以确保卡尔曼滤波增益回路中的滤波方差阵始终正定 $^{[15-23]}$ 。

卡尔曼最初提出的滤波基本理论只适用于线性系统，并且要求量测也必须是线性的。在之后的十多年时间内，Bucy, Sunahara 等人致力于研究卡尔曼滤波理论在非线性系统和非线性量测情况下的推广 $^{[24-33]}$ ，拓宽了卡尔曼滤波理论的适用范围。

卡尔曼滤波最成功的工程应用是设计运载体的高精度组合导航系统。20世纪80年代起，可供运载体装备的导航系统越来越多，非相似导航子系统的增加使量测信息增多，这对提高组合导航系统的精度十分有利。但是，如果采用集中式卡尔曼滤波器实现组合，则存在两个致命问题：① 滤波器计算量以状态维数的三次方剧增，无法满足导航的实时性要求；② 导航子系统的增加使故障率也随之增加，只要有一个子系统发生故障又没有及时检测出并隔离掉，则整个导航系统都会被污染。为了解决这一矛盾，1979年至1985年间，Speyer, Bierman和Kerr等人先后提出了分散滤波思想 $^{[34-40]}$ 。并行计算技术的成熟为分散滤波的发展创造了有利条件。1988年起，Carlson提出了联邦滤波理论(Federated Filtering) $^{[41-46]}$ ，旨在为容错组合导航系统提供设计理论。Carlson在装备运载体的诸多非相似导航子系统中选择导航信息全面、输出速率高、可靠性绝对保证的子系统作为公共参考系统，与其余子系统两两组合，形成若干个子滤波器。各子滤波器并行运行，获得建立在子滤波器局部量测基础上的局部最优估计，这些局部最优估计在第二级滤波器即主滤波器内按融合算法合成，获得建立在所有量测基础上的全局估计。全局估计再按信息守恒原则反馈给各子滤波器。实际设计的联邦滤波器是全局次优的，但是对于自主性要求特别高的重要运载体来说，导航系统的可靠性比精度更为重要。采用联邦滤波结构设计组合导航系统，虽然相对最优损失了少许精度，但换来的却是组合导航系统的高容错能力。目前美国空军已将联邦滤波器确定为新一代导航系统的通用滤波器 $^{[42,46]}$ 。

## 1.3 组合导航简介

将运载体从起始点引导到目的地的技术或方法称为导航。导航系统测量并解算出运载体的瞬时运动状态和位置，提供给驾驶员或自动驾驶仪实现对运载体的正确操纵或控制。随着科学技术的发展，可资利用的导航信息源越来越多，导航系统的种类也越来越多。以航空导航为例，目前可供装备的机载导航系统有惯性导航系统、GPS导航系统、多普勒导航系统、罗兰C导航系统等，这些导航系统各有特色，优缺点并存。比如，惯性导航（以下简称惯导）系统的优点是：不需要任何外来信息也不向外辐射任何信息，可在任何介质和任何环境条件下实现导航，且能输出飞机的位置、速度、方位和姿态等多种导航参数，系统的频带宽，能跟踪运载体的任何机动运动，导航输出数据平稳，短期稳定性好。但惯导系统具有固有的缺点：导航精度随时间而发散，即长期稳定性差。GPS导航系统导航精度高，在美国国防部加入SA(Selective Availability)误差后，使用C/A码信号的水平和垂直定位精度仍分别可达 $100\mathrm{m}$ 和 $157\mathrm{m}(2\sigma)$ ，且不随时间发散，这种高精度和长期稳定性是惯导系统望尘莫及的。但GPS导航系统也有其致命弱点：频带窄，当运载体作较高机动运动时，接收机的码环和载波环极易失锁而丢失信号，从而完全丧失导航能力；完全依赖于GPS卫星发射的导航信息，受制于他人，且易受人为干扰和电子欺骗。其余导航系统也有各自的优缺点。

各种导航系统单独使用时是很难满足导航性能要求的，提高导航系统整体性能的有效途径是采用组合导航技术，即用两种或两种以上的非相似导航系统对同一导航信息作测量并解算以形成量测量，从这些量测量中计算出各导航系统的误差并校正之。采用组合导航技术的系统称组合导航系统，参与组合的各导航系统称子系统。

实现组合导航有两种基本方法：

（1）回路反馈法，即采用经典的回路控制方法，抑制系统误差，并使各子系统间实现性能互补。第二次世界大战期间，英国为解决海上巡逻机长时间连续飞行的导航精度问题，率先研究了该型系统。

（2）最优估计法，即采用卡尔曼滤波或维纳滤波，从概率统计最优的角度估计出系统误差并消除之。两种方法都使各子系统内的信息互相渗透，有机结合，起到性能互补的功效。但由于各子系统的误差源和量测误差都是随机的，因此第二种方法远优于第一种方法。设计组合导航系统时一般都采用卡尔曼滤波。由于惯导和GPS在性能上正好形成互补，因此采用该两种系统作为组合导航设计中的子系统是世界公认的最佳方案，如图1.3.1所示为该导航系统原理图。

组合导航系统一般具有以下 3 种功能：

（1）协合超越功能。组合导航系统能充分利用各子系统的导航信息，形成单个子系统不具备的功能和精度。

（2）互补功能。由于组合导航系统综合利用了各子系统的信息，所以各子系统能取长补短，扩大使用范围。

![](images/81529acb419d766a6f731f8046e4c4bfde80dc761b6895633701c82ffef0bfc7.jpg)  
图 1.3.1 惯导 /GPS 组合导航系统原理图

（3）余度功能。各子系统感测同一信息源，使测量值冗余，提高整个系统的可靠性。

组合导航系统的发展方向是容错组合导航系统和导航专家系统，这些系统具有故障检测、诊断、隔离和系统重构的功能。目前，很多国家都在发展以激光陀螺捷联惯导系统和GPS导航系统为主要子系统并辅以其余子系统的组合导航系统，如Litton公司的FLAGSHIP系统和Honeywell公司的GAINS系统，平均故障时间可达12000小时，可根据不同的使用要求和系统故障情况选择不同的工作模式，都已成功地应用于诸多先进飞机上。

# 第二章 几种最优估计和卡尔曼滤波基本方程

## 2.1 几种最优估计

所谓估计就是根据测量得出的与状态 $X(t)$ 有关的数据 $Z(t)=h[X(t)]+V(t)$ 解算出 $X(t)$ 的计算值 $\hat{X}(t)$ ，其中随机向量 $V(t)$ 为量测误差， $\hat{X}$ 称为 X 的估计，Z 称为 X 的量测。因为 $\hat{X}(t)$ 是根据 $Z(t)$ 确定的，所以 $\hat{X}(t)$ 是 $Z(t)$ 的函数。若 $\hat{X}$ 是 Z 的线性函数，则 $\hat{X}$ 称作 X 的线性估计。

设在 $\left[t_{0},t_{1}\right]$ 时间段内的量测为Z，相应的估计为 $\hat{X}(t)$ ，则

当 $t = t_{1}$ 时， $\hat{\pmb{X}} (t)$ 称为 $\pmb {X}(t)$ 的估计；

当 $t > t_1$ 时， $\hat{\pmb{X}}(t)$ 称为 $\pmb {X}(t)$ 的预测；

当 $t < t_1$ 时， $\hat{\pmb{X}}(t)$ 称为 $\pmb {X}(t)$ 的平滑。

最优估计是指某一指标函数达到最值时的估计。

若以量测估计 $\hat{Z}$ 的偏差的平方和达到最小为指标，即

$$
(\mathbf {Z} - \hat {\mathbf {Z}}) ^ {\mathrm{T}} (\mathbf {Z} - \hat {\mathbf {Z}}) = \min
$$

则所得估计 $\hat{X}$ 为 X 的最小二乘估计。

若以状态估计 $\hat{X}$ 的均方误差集平均达到最小为指标, 即

$$
E \left[ (\boldsymbol {X} - \hat {\boldsymbol {X}}) ^ {\mathrm{T}} (\boldsymbol {X} - \hat {\boldsymbol {X}}) \right] = \min
$$

则所得估计 $\hat{X}$ 为X的最小方差估计；若 $\hat{X}$ 又是X的线性估计，则 $\hat{X}$ 为X的线性最小方差估计。

也可用估计值出现的概率作为估计指标,这样的估计有极大验后估计、贝叶斯估计和极大似然估计。本节将对上述各种估计作详细介绍。

## 2.1.1 最小二乘估计

1. 一般最小二乘估计

最小二乘估计是高斯(Karl Gauss)在1795年为测定行星轨道而提出的参数估计算法。这种估计的特点是算法简单，不必知道与被估计量及量测量有关的任何统计信息。

设 X 为某一确定性常值向量, 维数为 n 。一般情况下对 X 不能直接测量, 而只能测量到 X 各分量的线性组合。记第 i 次量测 $Z_{i}$ 为

$$
\mathbf {Z} _ {i} = \mathbf {H} _ {i} \mathbf {X} + \mathbf {V} _ {i}\tag{2.1.1}
$$

式中 $Z_{i}$ 为 $m_{i}$ 维向量， $H_{i}$ 和 $V_{i}$ 为第 i 次测量的量测矩阵和随机量测噪声。

若共测量 r 次, 即

$$
\left. \begin{array}{c} \mathbf {Z} _ {1} = \mathbf {H} _ {1} \mathbf {X} + \mathbf {V} _ {1} \\ \mathbf {Z} _ {2} = \mathbf {H} _ {2} \mathbf {X} + \mathbf {V} _ {2} \\ \dots \dots \\ \mathbf {Z} _ {r} = \mathbf {H} _ {r} \mathbf {X} + \mathbf {V} _ {r} \end{array} \right\}\tag{2.1.2}
$$

则由上述诸式可得描述 $r$ 次量测的量测方程

$$
Z = H X + V\tag{2.1.3}
$$

式中， $Z,V$ 为 $\sum_{i = 1}^{r}m_{i} = m$ 维向量， $\pmb{H}$ 为 $m\times n$ 矩阵。

最小二乘估计的指标是:使各次量测 $Z_{i}$ 与由估计 $\hat{X}$ 确定的量测的估计 $\hat{Z}_{i}=H_{i}\hat{X}$ 之差的平方和最小,即

$$
J (\hat {\mathbf {X}}) = (\mathbf {Z} - \mathbf {H} \hat {\mathbf {X}}) ^ {\mathrm{T}} (\mathbf {Z} - \mathbf {H} \hat {\mathbf {X}}) = \min\tag{2.1.4}
$$

而要使上式达到最小,需满足(详细推导见附录 A)

$$
\left. \frac {\partial J}{\partial \boldsymbol {X}} \right| _ {\boldsymbol {X} = \hat {\boldsymbol {X}}} = - 2 \boldsymbol {H} ^ {\mathrm{T}} (\boldsymbol {Z} - \boldsymbol {H} \hat {\boldsymbol {X}}) = \boldsymbol {0}
$$

若 $\pmb{H}$ 具有最大秩 $n$ ，即 $\pmb{H}^{\mathrm{T}}\pmb{H}$ 正定，且 $m = \sum_{i=1}^{r} m_{i} > n$ ，则 $\pmb{X}$ 的最小二乘估计为

$$
\hat {\mathbf {X}} = (\mathbf {H} ^ {\mathrm{T}} \mathbf {H}) ^ {- 1} \mathbf {H} ^ {\mathrm{T}} \mathbf {Z}\tag{2.1.5}
$$

从上式可看出最小二乘估计是一种线性估计。

为了说明最小二乘估计最优的含义,将式(2.1.4)改写成

$$
\begin{array}{r l} J (\hat {\mathbf {X}}) & = \left[ (\mathbf {Z} _ {1} - \mathbf {H} _ {1} \hat {\mathbf {X}}) ^ {\mathrm{T}} (\mathbf {Z} _ {2} - \mathbf {H} _ {2} \hat {\mathbf {X}}) ^ {\mathrm{T}} \dots (\mathbf {Z} _ {r} - \mathbf {H} _ {r} \hat {\mathbf {X}}) ^ {\mathrm{T}} \right] \left[ \begin{array}{c} \mathbf {Z} _ {1} - \mathbf {H} _ {1} \hat {\mathbf {X}} \\ \mathbf {Z} _ {2} - \mathbf {H} _ {2} \hat {\mathbf {X}} \\ \vdots \\ \mathbf {Z} _ {r} - \mathbf {H} _ {r} \hat {\mathbf {X}} \end{array} \right] = \\ & \sum_ {i = 1} ^ {r} (\mathbf {Z} _ {i} - \mathbf {H} _ {i} \hat {\mathbf {X}}) ^ {\mathrm{T}} (\mathbf {Z} _ {i} - \mathbf {H} _ {i} \hat {\mathbf {X}}) = \min \end{array}
$$

这说明,最小二乘估计虽然不能满足式(2.1.2)中的每一个方程,既使每个方程都有偏差,但它使所有方程偏差的平方和达到最小。这实际上兼顾了所有方程的近似程度,使整体误差达到最小,这对抑制测量误差 $V_{1},V_{2},\cdots,V_{r}$ 的影响是有益的。

最小二乘估计具有如下性质：

若量测噪声 V 是均值为零、方差为 R 的随机向量，则

(1) 最小二乘估计是无偏估计, 即

$$
E [ \hat {X} ] = X\tag{2.1.6}
$$

或

$$
E [ \widetilde {X} ] = 0\tag{2.1.7}
$$

式中 $\hat{X} = X - \hat{X}$ 为 $\hat{X}$ 的估计误差。

(2) 最小二乘估计的均方误差阵为

$$
E [ \widetilde {\boldsymbol {X}} \widetilde {\boldsymbol {X}} ^ {\mathrm{T}} ] = (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {H}) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {R} \boldsymbol {H} (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {H}) ^ {- 1}\tag{2.1.8}
$$

证明

① 由式(2.1.5)， $\hat{X}$ 的估计误差为

$$
\tilde {\boldsymbol {X}} = \boldsymbol {X} - \hat {\boldsymbol {X}} = \boldsymbol {X} - (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {H}) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {Z}
$$

由于 $E[\mathbf{V}] = \mathbf{0}$ ，所以

$$
\begin{array}{r l} E [ \widetilde {\mathbf {X}} ] & = E [ (\mathbf {H} ^ {\mathrm{T}} \mathbf {H}) ^ {- 1} (\mathbf {H} ^ {\mathrm{T}} \mathbf {H}) \mathbf {X} - (\mathbf {H} ^ {\mathrm{T}} \mathbf {H}) ^ {- 1} \mathbf {H} ^ {\mathrm{T}} \mathbf {Z} ] = (\mathbf {H} ^ {\mathrm{T}} \mathbf {H}) ^ {- 1} \mathbf {H} ^ {\mathrm{T}} E [ \mathbf {H} \mathbf {X} - \mathbf {Z}) ] = \\ & - (\mathbf {H} ^ {\mathrm{T}} \mathbf {H}) ^ {- 1} \mathbf {H} ^ {\mathrm{T}} E [ \mathbf {V} ] = \mathbf {0} \end{array}
$$

$$
② E [ \widetilde {\mathbf {X}} \widetilde {\mathbf {X}} ^ {\mathrm{T}} ] = (\mathbf {H} ^ {\mathrm{T}} \mathbf {H}) ^ {- 1} \mathbf {H} ^ {\mathrm{T}} E [ \mathbf {V V} ^ {\mathrm{T}} ] \mathbf {H} (\mathbf {H} ^ {\mathrm{T}} \mathbf {H}) ^ {- 1} = (\mathbf {H} ^ {\mathrm{T}} \mathbf {H}) ^ {- 1} \mathbf {H} ^ {\mathrm{T}} \mathbf {R H} (\mathbf {H} ^ {\mathrm{T}} \mathbf {H}) ^ {- 1}
$$

例 2-1 用一台仪器对未知确定性标量 X 作 r 次直接测量，量测值分别为 $Z_{1}, Z_{2}, \cdots, Z_{r}$ ，测量误差的均值为零，方差为 R，求 X 的最小二乘估计 $\hat{X}$ ，并计算估计的均方误差。

解 由题意, r 次直接测量所得的量测方程为

$$
\mathbf {Z} = \mathbf {H} X + \mathbf {V}
$$

式中

$$
\mathbf {Z} = \left[ \begin{array}{l l l l} Z _ {1} & Z _ {2} & \dots & Z _ {r} \end{array} \right] ^ {\mathrm{T}}, \mathbf {H} = \left[ \begin{array}{l l l l} 1 & 1 & \dots & 1 \end{array} \right] ^ {\mathrm{T}}, E [ \mathbf {V V} ^ {\mathrm{T}} ] = R \mathbf {I}
$$

根据式 $(2.1.5)$ ，得

$$
\hat {X} = \frac {1}{r} (Z _ {1} + Z _ {2} + \dots + Z _ {r})
$$

估计的均方误差为

$$
E [ \widetilde {X} ^ {2} ] = \frac {R}{r}
$$

例2-2 设有线性系统满足

$$
\boldsymbol {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {X} _ {k - 1}
$$

对 $X_{k}$ 的量测为标量, 满足

$$
Z _ {k} = \boldsymbol {h} _ {k} \boldsymbol {X} _ {k} + V _ {k}
$$

式中， $X_{k}$ 为 n 维向量； $V_{k}$ 为随机误差；k 为时刻 $t_{k}$ 。求 $X_{0}$ 基于 $Z_{1}, Z_{2}, \cdots, Z_{m} (m \geqslant n)$ 的最小二乘估计 $\hat{X}_{0/m}$ 。

解 $X_{0}$ 的第 i 次量测为

$$
Z _ {i} = \boldsymbol {h} _ {i} \boldsymbol {X} _ {i} + V _ {i} = \boldsymbol {h} _ {i} \boldsymbol {\Phi} _ {i, 0} \boldsymbol {X} _ {0} + V _ {i} \quad i = 1, 2, \dots , m
$$

$X_0$ 的前 $m$ 次量测为

$$
\mathbf {Z} _ {m} = \mathbf {H} _ {m, 0} \mathbf {X} _ {0} + \mathbf {V} _ {m}
$$

式中

$$
\mathbf {Z} _ {m} = \left[ \begin{array}{c} Z _ {1} \\ Z _ {2} \\ \vdots \\ Z _ {m} \end{array} \right], \mathbf {H} _ {m, 0} = \left[ \begin{array}{c} h _ {1} \boldsymbol {\Phi} _ {1, 0} \\ h _ {2} \boldsymbol {\Phi} _ {2, 0} \\ \vdots \\ h _ {m} \boldsymbol {\Phi} _ {m, 0} \end{array} \right], \mathbf {V} _ {m} = \left[ \begin{array}{c} V _ {1} \\ V _ {2} \\ \vdots \\ V _ {m} \end{array} \right]
$$

根据式(2.1.5)，得 $X_{0}$ 基于 $Z_{1}, Z_{2}, \cdots, Z_{m}$ 的最小二乘估计为

$$
\hat {\mathbf {X}} _ {0 / m} = (\mathbf {H} _ {m, 0} ^ {\mathrm{T}} \mathbf {H} _ {m, 0}) ^ {- 1} \mathbf {H} _ {m, 0} ^ {\mathrm{T}} \mathbf {Z} _ {m}
$$

例2-3 用两台仪器对未知标量 $X$ 各直接测量一次，量测量分别为 $Z_{1}$ 和 $Z_{2}$ ，仪器的测量误差是均值为零，方差分别为 $r$ 和 $4r$ 的随机量，求 $X$ 的最小二乘估计，并计算估计的均方误差。

解 由题意得量测方程

$$
Z = H X + V
$$

式中

$$
\mathbf {Z} = \left[ \begin{array}{l} Z _ {1} \\ Z _ {2} \end{array} \right],   \mathbf {H} = \left[ \begin{array}{l} 1 \\ 1 \end{array} \right],   \mathbf {R} = \left[ \begin{array}{l l} r & 0 \\ 0 & 4 r \end{array} \right]
$$

根据式 $(2.1.5)$ 和式 $(2.1.8)$ ，得

$$
\hat {X} = \frac {1}{2} (Z _ {1} + Z _ {2})
$$

$$
E [ \hat {X} ^ {2} ] = \frac {1}{2} [ 1 1 ] \left[ \begin{array}{l l} r & 0 \\ 0 & 4 r \end{array} \right] \left[ \begin{array}{l} 1 \\ 1 \end{array} \right] \frac {1}{2} = \frac {5}{4} r > r
$$

上式说明,使用精度差一倍的两台仪器同时进行测量,最小二乘估计效果还不如只使用一台精度高的仪器时好。

## 2. 加权最小二乘估计

从例2-3看出，一般最小二乘估计精度不高的原因之一是不分优劣地使用了量测值。如果对不同量测值的质量有所了解，则可用加权的办法分别对待各量测量，精度质量高的权重取得大些，精度质量差的权重取得小些。这就是加权最小二乘估计的思路。加权最小二乘估计 $\hat{X}$ 的求取准则是

$$
J (\hat {\boldsymbol {X}}) = (\boldsymbol {Z} - \boldsymbol {H} \hat {\boldsymbol {X}}) ^ {\mathrm{T}} \boldsymbol {W} (\boldsymbol {Z} - \boldsymbol {H} \hat {\boldsymbol {X}}) = \min\tag{2.1.9}
$$

式中 W 是适当取值的正定加权矩阵。当 W=I 时，式(2.1.9) 就是一般最小二乘准则。

要使式(2.1.9)成立, $\hat{X}$ 应满足

$$
\left. \frac {\partial J (\boldsymbol {X})}{\partial \boldsymbol {X}} \right| _ {\boldsymbol {X} = \hat {\boldsymbol {X}}} = - \boldsymbol {H} ^ {\mathrm{T}} (\boldsymbol {W} + \boldsymbol {W} ^ {\mathrm{T}}) (\boldsymbol {Z} - \boldsymbol {H} \hat {\boldsymbol {X}}) = \mathbf {0}
$$

从中解得

$$
\hat {\mathbf {X}} = \left[ \mathbf {H} ^ {\mathrm{T}} (\mathbf {W} + \mathbf {W} ^ {\mathrm{T}}) \mathbf {H} \right] ^ {- 1} \mathbf {H} ^ {\mathrm{T}} (\mathbf {W} + \mathbf {W} ^ {\mathrm{T}}) \mathbf {Z}
$$

一般情况下,加权阵取成对称阵,即 $W^{T}=W$ ,所以加权最小二乘估计为

$$
\hat {\boldsymbol {X}} = (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {H}) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {Z}\tag{2.1.10}
$$

估计误差为

$$
\begin{array}{r l} \tilde {\boldsymbol {X}} = \boldsymbol {X} - \hat {\boldsymbol {X}} & = (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {H}) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {H} \boldsymbol {X} - (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {H}) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {Z} = \\ & (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {H}) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} (\boldsymbol {H} \boldsymbol {X} - \boldsymbol {Z}) = \\ & - (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {H}) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {V} \end{array}
$$

如果量测误差 V 的均值为零, 方差阵为 R, 则加权最小二乘估计也是无偏估计, 估计的均方误差为

$$
E \left[ \tilde {\boldsymbol {X}} \tilde {\boldsymbol {X}} ^ {\mathrm{T}} \right] = \left(\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {H}\right) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {R} \boldsymbol {W} \boldsymbol {H} \left(\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {H}\right) ^ {- 1}\tag{2.1.11}
$$

如果 $W = R^{-1}$ ，则加权最小二乘估计

$$
\hat {\mathbf {X}} = (\mathbf {H} ^ {\mathrm{T}} \mathbf {R} ^ {- 1} \mathbf {H}) ^ {- 1} \mathbf {H} ^ {\mathrm{T}} \mathbf {R} ^ {- 1} \mathbf {Z}\tag{2.1.12}
$$

又称为马尔柯夫估计。

马尔柯夫估计的均方误差为

$$
E [ \widetilde {\boldsymbol {X}} \widetilde {\boldsymbol {X}} ^ {\mathrm{T}} ] = (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {R} ^ {- 1} \boldsymbol {H}) ^ {- 1}\tag{2.1.13}
$$

马尔柯夫估计的均方误差比任何其他加权最小二乘估计的均方误差都要小,所以是加权最小二乘估计中的最优者。下面证明这一结论。

设 A 和 B 分别为 $n \times m$ 和 $m \times l$ 的矩阵，且 $AA^{T}$ 满秩，则因

$$
\left[ \boldsymbol {B} - \boldsymbol {A} ^ {\mathrm{T}} (\boldsymbol {A A} ^ {\mathrm{T}}) ^ {- 1} \boldsymbol {A B} \right] ^ {\mathrm{T}} \left[ \boldsymbol {B} - \boldsymbol {A} ^ {\mathrm{T}} (\boldsymbol {A A} ^ {\mathrm{T}}) ^ {- 1} \boldsymbol {A B} \right] =
$$

$$
\boldsymbol {B} ^ {\mathrm{T}} \boldsymbol {B} - 2 \boldsymbol {B} ^ {\mathrm{T}} \boldsymbol {A} ^ {\mathrm{T}} (\boldsymbol {A A} ^ {\mathrm{T}}) ^ {- 1} \boldsymbol {A B} + \boldsymbol {B} ^ {\mathrm{T}} \boldsymbol {A} ^ {\mathrm{T}} (\boldsymbol {A A} ^ {\mathrm{T}}) ^ {- 1} \boldsymbol {A A} ^ {\mathrm{T}} (\boldsymbol {A A} ^ {\mathrm{T}}) ^ {- 1} \boldsymbol {A B} =
$$

$$
\boldsymbol {B} ^ {\mathrm{T}} \boldsymbol {B} - (\boldsymbol {A B}) ^ {\mathrm{T}} (\boldsymbol {A A} ^ {\mathrm{T}}) ^ {- 1} (\boldsymbol {A B}) \geqslant \boldsymbol {0}
$$

所以有

$$
\boldsymbol {B} ^ {\mathrm{T}} \boldsymbol {B} \geqslant (\boldsymbol {A B}) ^ {\mathrm{T}} (\boldsymbol {A A} ^ {\mathrm{T}}) ^ {- 1} (\boldsymbol {A B})\tag{2.1.14}
$$

这就是矩阵型的许瓦茨不等式。

由矩阵理论知,正定阵 R 可表示成 $R = S^{T}S$ , 其中 S 为满秩矩阵。令

$$
\mathbf {A} = \mathbf {H} ^ {\mathrm{T}} \mathbf {S} ^ {- 1}\tag{2.1.15a}
$$

$$
\boldsymbol {B} = \boldsymbol {S W H} (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W H}) ^ {- 1}\tag{2.1.15b}
$$

则

$$
\boldsymbol {A} \boldsymbol {B} = \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {S} ^ {- 1} \boldsymbol {S} \boldsymbol {W} \boldsymbol {H} (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {H}) ^ {- 1} = \boldsymbol {I}\tag{2.1.16}
$$

$$
\boldsymbol {B} ^ {\mathrm{T}} \boldsymbol {B} = (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {H}) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {R} \boldsymbol {W} \boldsymbol {H} (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {W} \boldsymbol {H}) ^ {- 1}\tag{2.1.17}
$$

根据许瓦茨不等式和式(2.1.16)及式(2.1.15a)，有

$$
\boldsymbol {B} ^ {\mathrm{T}} \boldsymbol {B} \geqslant (\boldsymbol {A B}) ^ {\mathrm{T}} (\boldsymbol {A A} ^ {\mathrm{T}}) ^ {- 1} (\boldsymbol {A B}) = (\boldsymbol {A A} ^ {\mathrm{T}}) ^ {- 1} = (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {R} ^ {- 1} \boldsymbol {H}) ^ {- 1}
$$

式 $(2.1.17)$ 代入上式,得

$$
(\mathbf {H} ^ {\mathrm{T}} \mathbf {W} \mathbf {H}) ^ {- 1} \mathbf {H} ^ {\mathrm{T}} \mathbf {W} \mathbf {R} \mathbf {W} \mathbf {H} (\mathbf {H} ^ {\mathrm{T}} \mathbf {W} \mathbf {H}) ^ {- 1} \geqslant (\mathbf {H} ^ {\mathrm{T}} \mathbf {R} ^ {- 1} \mathbf {H}) ^ {- 1}\tag{2.1.18}
$$

式(2.1.18)说明，只有当 $W = R^{-1}$ 时，估计的均方误差才达到最小，最小值为 $(H^{T}R^{-1}H)^{-1}$ 。

例2-4 对例2-3采用马尔柯夫估计，并求估计的均方误差。

解 取

$$
\boldsymbol {W} = \boldsymbol {R} ^ {- 1} = \left[ \begin{array}{l l} \frac {1}{r} & 0 \\ 0 & \frac {1}{4 r} \end{array} \right]
$$

$$
\hat {X} = ([ 1 1 ] \left[ \begin{array}{l l} \frac {1}{r} & 0 \\ 0 & \frac {1}{4 r} \end{array} \right] [ \begin{array}{l} 1 \\ 1 \end{array} ]) ^ {- 1} [ 1 1 ] \left[ \begin{array}{l l} \frac {1}{r} & 0 \\ 0 & \frac {1}{4 r} \end{array} \right] [ \begin{array}{l} Z _ {1} \\ Z _ {2} \end{array} ] = \frac {4}{5} Z _ {1} + \frac {1}{5} Z _ {2}
$$

$$
E [ \widetilde {X} \widetilde {X} ] = ([ 1 1 ] \left[ \begin{array}{l l} \frac {1}{r} & 0 \\ 0 & \frac {1}{4 r} \end{array} \right] \left[ \begin{array}{l} 1 \\ 1 \end{array} \right]) ^ {- 1} = \frac {4}{5} r <   r
$$

这说明,对精度高的量测值取权重系数 $\frac{4}{5}$ ,对精度低的量测值取权重系数 $\frac{1}{5}$ ,估计精度高于仅用精度高的量测值所得估计的精度。所以增加不同的量测值,并根据其精度质量区别对待利用之,能有效提高估计精度。

## 3. 递推最小二乘估计

从上述诸例可看出,量测值越多,只要处理得合适,最小二乘估计的均方误差就越小。采用批处理实现的最小二乘算法,须存储所有的量测值。若量测值数量十分庞大,则计算机必须具备巨大的存储容量,这显然是不经济的。递推最小二乘估计从每次获得的量测值中提取出被估计量信息,用于修正上一步所得的估计。获得量测的次数越多,修正的次数也越多,估计的精度也越高。下面详细介绍该算法。

设 X 为确定性常值向量, 前 k 次观测积累的量测为 $\bar{Z}_{k}$ , 量测方程为

$$
\overline {{{{\bf Z}}}} _ {k} = \overline {{{{\bf H}}}} _ {k} {\bf X} + \overline {{{{\bf V}}}} _ {k}
$$

式中

$$
\overline {{{\mathbf {Z}}}} _ {k} = \left[ \begin{array}{c} \mathbf {Z} _ {1} \\ \mathbf {Z} _ {2} \\ \vdots \\ \mathbf {Z} _ {k} \end{array} \right], \overline {{{\mathbf {H}}}} _ {k} = \left[ \begin{array}{c} \mathbf {H} _ {1} \\ \mathbf {H} _ {2} \\ \vdots \\ \mathbf {H} _ {k} \end{array} \right], \overline {{{\mathbf {V}}}} _ {k} = \left[ \begin{array}{c} \mathbf {V} _ {1} \\ \mathbf {V} _ {2} \\ \vdots \\ \mathbf {V} _ {k} \end{array} \right]
$$

$Z_{i}$ 为第 i 次量测,量测方程为

$$
\mathbf {Z} _ {i} = \mathbf {H} _ {i} \mathbf {X} + \mathbf {V} _ {i} \quad i = 1, 2, \dots , k
$$

则前 $k + 1$ 次量测为

$$
\overline {{{{\mathbf {Z}}}}} _ {k + 1} = \overline {{{{\mathbf {H}}}}} _ {k + 1} \mathbf {X} + \overline {{{{\mathbf {V}}}}} _ {k + 1}
$$

式中

$$
\overline {{{\mathbf {Z}}}} _ {k + 1} = \left[ \begin{array}{l} \overline {{{\mathbf {Z}}}} _ {k} \\ \mathbf {Z} _ {k + 1} \end{array} \right], \overline {{{\mathbf {H}}}} _ {k + 1} = \left[ \begin{array}{l} \overline {{{\mathbf {H}}}} _ {k} \\ \mathbf {H} _ {k + 1} \end{array} \right], \overline {{{\mathbf {V}}}} _ {k + 1} = \left[ \begin{array}{l} \overline {{{\mathbf {V}}}} _ {k} \\ \mathbf {V} _ {k + 1} \end{array} \right]
$$

$Z_{k+1}$ 为第 $k+1$ 次量测，量测方程为

$$
\mathbf {Z} _ {k + 1} = \mathbf {H} _ {k + 1} \mathbf {X} + \mathbf {V} _ {k + 1}
$$

根据式(2.1.10)，由前 k 次量测确定的加权最小二乘估计为

$$
\hat {\boldsymbol {X}} _ {k} = \left(\overline {{\boldsymbol {H}}} _ {k} ^ {\mathrm{T}} \overline {{\boldsymbol {W}}} _ {k} \overline {{\boldsymbol {H}}} _ {k}\right) ^ {- 1} \overline {{\boldsymbol {H}}} _ {k} ^ {\mathrm{T}} \overline {{\boldsymbol {W}}} _ {k} \overline {{\boldsymbol {Z}}} _ {k}
$$

式中

$$
\overline {{{W}}} _ {k} = \left[ \begin{array}{c c c c} W _ {1} & & & \\ & W _ {2} & & 0 \\ 0 & & \ddots & \\ & & & W _ {k} \end{array} \right]
$$

令

$$
\boldsymbol {P} _ {k} = (\overline {{\boldsymbol {H}}} _ {k} ^ {\mathrm{T}} \overline {{\boldsymbol {W}}} _ {k} \overline {{\boldsymbol {H}}} _ {k}) ^ {- 1}\tag{2.1.19}
$$

则

$$
\hat {\boldsymbol {X}} _ {k} = \boldsymbol {P} _ {k} \overline {{\boldsymbol {H}}} _ {k} ^ {\mathrm{T}} \overline {{\boldsymbol {W}}} _ {k} \overline {{\boldsymbol {Z}}} _ {k}\tag{2.1.20}
$$

由前 $k + 1$ 次量测确定的加权最小二乘估计为

$$
\hat {\boldsymbol {X}} _ {k + 1} = (\overline {{\boldsymbol {H}}} _ {k + 1} ^ {\mathrm{T}} \overline {{\boldsymbol {W}}} _ {k + 1} \overline {{\boldsymbol {H}}} _ {k + 1}) ^ {- 1} \overline {{\boldsymbol {H}}} _ {k + 1} ^ {\mathrm{T}} \overline {{\boldsymbol {W}}} _ {k + 1} \overline {{\boldsymbol {Z}}} _ {k + 1} = \boldsymbol {P} _ {k + 1} [ \overline {{\boldsymbol {H}}} _ {k} ^ {\mathrm{T}} \boldsymbol {H} _ {k + 1} ^ {\mathrm{T}} ] \left[ \begin{array}{c c} \overline {{\boldsymbol {W}}} _ {k} & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {W} _ {k + 1} \end{array} \right] \left[ \begin{array}{c} \overline {{\boldsymbol {Z}}} _ {k} \\ \boldsymbol {Z} _ {k + 1} \end{array} \right] =
$$

$$
\boldsymbol {P} _ {k + 1} \overline {{\boldsymbol {H}}} _ {k} ^ {\mathrm{T}} \overline {{\boldsymbol {W}}} _ {k} \overline {{\boldsymbol {Z}}} _ {k} + \boldsymbol {P} _ {k + 1} \boldsymbol {H} _ {k + 1} ^ {\mathrm{T}} \boldsymbol {W} _ {k + 1} \boldsymbol {Z} _ {k + 1}\tag{2.1.21}
$$

式中

$$
\boldsymbol {P} _ {k + 1} = (\overline {{\boldsymbol {H}}} _ {k + 1} ^ {\mathrm{T}} \overline {{\boldsymbol {W}}} _ {k + 1} \overline {{\boldsymbol {H}}} _ {k + 1}) ^ {- 1} = ([ \overline {{\boldsymbol {H}}} _ {k} ^ {\mathrm{T}} \boldsymbol {H} _ {k + 1} ^ {\mathrm{T}} ] \left[ \begin{array}{c c} \overline {{\boldsymbol {W}}} _ {k} & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {W} _ {k + 1} \end{array} \right] \left[ \begin{array}{c} \overline {{\boldsymbol {H}}} _ {k} \\ \boldsymbol {H} _ {k + 1} \end{array} \right]) ^ {- 1} =
$$

$$
(\overline {{\boldsymbol {H}}} _ {k} ^ {\mathrm{T}} \overline {{\boldsymbol {W}}} _ {k} \overline {{\boldsymbol {H}}} _ {k} + \boldsymbol {H} _ {k + 1} ^ {\mathrm{T}} \boldsymbol {W} _ {k + 1} \boldsymbol {H} _ {k + 1}) ^ {- 1} = (\boldsymbol {P} _ {k} ^ {- 1} + \boldsymbol {H} _ {k + 1} ^ {\mathrm{T}} \boldsymbol {W} _ {k + 1} \boldsymbol {H} _ {k + 1}) ^ {- 1}\tag{2.1.22}
$$

由矩阵反演公式(详细推导见附录 B)

$$
\left(\boldsymbol {A} _ {1 1} - \boldsymbol {A} _ {1 2} \boldsymbol {A} _ {2 2} ^ {- 1} \boldsymbol {A} _ {2 1}\right) ^ {- 1} = \boldsymbol {A} _ {1 1} ^ {- 1} + \boldsymbol {A} _ {1 1} ^ {- 1} \boldsymbol {A} _ {1 2} \left(\boldsymbol {A} _ {2 2} - \boldsymbol {A} _ {2 1} \boldsymbol {A} _ {1 1} ^ {- 1} \boldsymbol {A} _ {1 2}\right) ^ {- 1} \boldsymbol {A} _ {2 1} \boldsymbol {A} _ {1 1} ^ {- 1}\tag{2.1.23}
$$

令 $A_{11}=P_{k}^{-1},A_{12}=-H_{k+1}^{T},A_{22}^{-1}=W_{k+1},A_{21}=H_{k+1}$ ，得

$$
\boldsymbol {P} _ {k + 1} = \boldsymbol {P} _ {k} - \boldsymbol {P} _ {k} \boldsymbol {H} _ {k + 1} ^ {\mathrm{T}} \left(\boldsymbol {W} _ {k + 1} ^ {- 1} + \boldsymbol {H} _ {k + 1} \boldsymbol {P} _ {k} \boldsymbol {H} _ {k + 1} ^ {\mathrm{T}}\right) ^ {- 1} \boldsymbol {H} _ {k + 1} \boldsymbol {P} _ {k}\tag{2.1.24}
$$

再考察式(2.1.21)中的第一项,由式(2.1.20)和式(2.1.22),得

$$
\overline {{{{\boldsymbol {H}}}}} _ {k} ^ {\mathrm{T}} \overline {{{{\boldsymbol {W}}}}} _ {k} \overline {{{{\boldsymbol {Z}}}}} _ {k} = \boldsymbol {P} _ {k} ^ {- 1} \hat {\boldsymbol {X}} _ {k}
$$

$$
\boldsymbol {P} _ {k} ^ {- 1} = \boldsymbol {P} _ {k + 1} ^ {- 1} - \boldsymbol {H} _ {k + 1} ^ {\mathrm{T}} \boldsymbol {W} _ {k + 1} \boldsymbol {H} _ {k + 1}
$$

所以该项为

$$
\begin{array}{r l} & {\pmb {P} _ {k + 1} \overline {{\pmb {H}}} _ {k} ^ {\mathrm{T}} \overline {{\pmb {W}}} _ {k} \overline {{\pmb {Z}}} _ {k} = \pmb {P} _ {k + 1} \pmb {P} _ {k} ^ {- 1} \hat {\pmb {X}} _ {k} = \pmb {P} _ {k + 1} (\pmb {P} _ {k + 1} ^ {- 1} - \pmb {H} _ {k + 1} ^ {\mathrm{T}} \pmb {W} _ {k + 1} \pmb {H} _ {k + 1}) \hat {\pmb {X}} _ {k} =} \\ & {\qquad \hat {\pmb {X}} _ {k} - \pmb {P} _ {k + 1} \pmb {H} _ {k + 1} ^ {\mathrm{T}} \pmb {W} _ {k + 1} \pmb {H} _ {k + 1} \hat {\pmb {X}} _ {k}} \end{array}
$$

因此式 $(2.1.21)$ 成

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {k + 1} & = \hat {\mathbf {X}} _ {k} - \mathbf {P} _ {k + 1} \mathbf {H} _ {k + 1} ^ {\mathrm{T}} \mathbf {W} _ {k + 1} \mathbf {H} _ {k + 1} \hat {\mathbf {X}} _ {k} + \mathbf {P} _ {k + 1} \mathbf {H} _ {k + 1} ^ {\mathrm{T}} \mathbf {W} _ {k + 1} \mathbf {Z} _ {k + 1} = \\ & \quad \hat {\mathbf {X}} _ {k} + \mathbf {P} _ {k + 1} \mathbf {H} _ {k + 1} ^ {\mathrm{T}} \mathbf {W} _ {k + 1} (\mathbf {Z} _ {k + 1} - \mathbf {H} _ {k + 1} \hat {\mathbf {X}} _ {k}) \end{array}\tag{2.1.25}
$$

式(2.1.24)和式(2.1.25)即为递推最小二乘估计的全套算法。

式(2.1.25)说明, $k+1$ 时刻的估计由对k时刻的估计作修正而获得,修正量由对 $k+1$ 时刻的量测的估计误差 $\tilde{Z}_{k+1}=Z_{k+1}-H_{k+1}\hat{X}_{k}$ 经增益阵 $K_{k+1}=P_{k+1}H_{k+1}^{T}W_{k+1}$ 加权后确定,其中 $P_{k+1}$ 由式(2.1.24)确定。

式(2.1.24)和式(2.1.25)确定的算法是递推的,只要给定初始值 $\hat{X}_{0}$ 和 $P_{0}$ ,即可获得X在任意时刻的最小二乘估计。 $\hat{X}_{0}$ 和 $P_{0}$ 的选取可以是任意的,一般可取 $\hat{X}_{0}=0,P_{0}=pI$ ,其中p为很大的正数。由于初值选取盲目,所以递推过程中,刚开始计算时,估计误差跳跃剧烈,随着量测次数的增加,初值影响逐渐消失,估计值逐渐趋于稳定而逼近被估计量。

例2-5 在例2-1中求出 $\hat{X}_k$ 后，试求获得新量测值 $Z_{k + 1}$ 后的递推最小二乘估计。

解 例2-1采用了一般最小二乘算法，这实际上加权阵取作单位阵。根据式(2.1.19)、式(2.1.24)及式(2.1.25)，得

$$
\begin{array}{l} P _ {k} = ([ 1 1 \dots 1 ] \left[ \begin{array}{c} 1 \\ 1 \\ \vdots \\ 1 \end{array} \right]) ^ {- 1} = \frac {1}{k} \\ P _ {k + 1} = \frac {1}{k} - \frac {1}{k} \left(1 + \frac {1}{k}\right) ^ {- 1} \frac {1}{k} = \frac {1}{k + 1} \\ \hat {X} _ {k + 1} = \hat {X} _ {k} + \frac {1}{k + 1} (Z _ {k + 1} - \hat {X} _ {k}) \end{array}
$$

最小二乘估计的最大优点是算法简单,特别是一般最小二乘估计,根本不必知道量测误差的统计信息。但正是这种优点又引起了使用上的局限性,主要体现在如下两点上:

(1) 最小二乘算法只能估计确定性的常值向量,而无法估计随机向量的时间过程。

（2）最小二乘的最优指标只保证了量测的估计均方误差之和最小，而并未确保被估计量的估计误差达到最佳，所以估计精度不高。

## 2.1.2 最小方差估计

1. 最小方差估计与条件均值

设 X 为随机向量, Z 为 X 的量测向量, 即 $\mathbf{Z} = \mathbf{Z}(\mathbf{X}) + \mathbf{V}$ , 求 X 的估计 $\hat{X}$ 就是根据 Z 解算出 X, 显然 $\hat{X}$ 是 Z 的函数, 由于 V 是随机误差, 所以 X 无法从 Z 的函数关系式中直接求取, 而必须按统计意义的最优标准求取。

最小方差估计是使下述指标达到最小的估计 $\hat{X}_{\mathrm{MV}}(Z)$

$$
J = E _ {X, Z} \left\{\left[ X - \hat {X} (Z) \right] ^ {\mathrm{T}} \left[ X - \hat {X} (Z) \right] \right\} | _ {\hat {X} (Z) = \hat {X} _ {\mathrm{MV}} (Z)} = \min\tag{2.1.26}
$$

式中 $X - \hat{X}$ 为估计误差。对于 Z 的某一个实现， $\hat{X}(Z)$ 是与之对应的某一具体样本，所以上式中求均值是对 X 和 Z 同时进行的。

定理2.1

最小方差估计等于量测为某一具体实现条件下的条件均值

$$
\hat {\mathbf {X}} _ {\mathrm{MV}} (\mathbf {Z}) = E [ \mathbf {X} / \mathbf {Z} ]\tag{2.1.27}
$$

下面详细证明这一定理。

证明

$$
\begin{array}{r l} & J = E _ {X, Z} \{\left[ X - \hat {X} _ {\mathrm{MV}} (Z) \right] ^ {\mathrm{T}} [ X - \hat {X} _ {\mathrm{MV}} (Z) ] \} = \\ & \int_ {- \infty} ^ {\infty} \int_ {- \infty} ^ {\infty} [ x - \hat {X} _ {\mathrm{MV}} (z) ] ^ {\mathrm{T}} [ x - \hat {X} _ {\mathrm{MV}} (z) ] p (x, z) \mathrm{d} x \mathrm{d} z \end{array}
$$

式中 $p(x,z)$ 为 X 和 Z 的联合分布密度, 是关于 x 和 z 的标量函数。根据贝叶斯定理:

$$
p (x, z) = p _ {z} (z) \cdot p (x / z)
$$

式中 $p_z(z)$ 为 $\mathbf{Z}$ 的边缘分布， $p(x / z)$ 为 $\mathbf{X}$ 关于 $\mathbf{Z}$ 的条件分布。所以

$$
J = \int_ {- \infty} ^ {\infty} p _ {z} (z) \left\{\int_ {- \infty} ^ {\infty} [ x - \hat {X} _ {\mathrm{MV}} (z) ] ^ {\mathrm{T}} [ x - \hat {X} _ {\mathrm{MV}} (z) ] p (x / z) \mathrm{d} x \right\} \mathrm{d} z\tag{2.1.28}
$$

由于 $\hat{X}_{MV}$ 的选择对 J 的影响只体现在上式的内层积分中，又 $p_{z}(z)$ 恒大于零，所以要使 J 达到最小，只须使上式中的内层积分达到最小即可，即

$$
g = \int_ {- \infty} ^ {\infty} [ \boldsymbol {x} - \hat {\boldsymbol {X}} _ {\mathrm{MV}} (\boldsymbol {z}) ] ^ {\mathrm{T}} [ \boldsymbol {x} - \hat {\boldsymbol {X}} _ {\mathrm{MV}} (\boldsymbol {z}) ] p (\boldsymbol {x} / \boldsymbol {z}) \mathrm{d} \boldsymbol {x} = \min
$$

对 g 作恒等变形, 并注意到: $E[X/z]$ 和 $\hat{X}(z)$ 都是确定量, x 是 X 样本空间内的所有样本, 而 z 是 Z 样本空间内的某一样本, 所以对 x 的积分中, $E[X/z], \hat{X}(z)$ 与 x 无关, 因此

$$
\begin{array}{r l} g & = \int_ {- \infty} ^ {\infty} [ x - E (X / z) + E (X / z) - \hat {X} _ {\mathrm{MV}} (z) ] ^ {\mathrm{T}} \cdot \\ & [ x - E (X / z) + E (X / z) - \hat {X} _ {\mathrm{MV}} (z) ] p (x / z) \mathrm{d} x = \\ & \int_ {- \infty} ^ {\infty} [ x - E (X / z) ] ^ {\mathrm{T}} [ x - E (X / z) ] p (x / z) \mathrm{d} x + \\ & [ E (X / z) - \hat {X} _ {\mathrm{MV}} (z) ] ^ {\mathrm{T}} [ E (X / z) - \hat {X} _ {\mathrm{MV}} (z) ] \int_ {- \infty} ^ {\infty} p (x / z) \mathrm{d} x + \\ & \int_ {- \infty} ^ {\infty} [ x - E (X / z) ] ^ {\mathrm{T}} p (x / z) \mathrm{d} x [ E (X / z) - \hat {X} _ {\mathrm{MV}} (z) ] + \\ & [ E (X / z) - \hat {X} _ {\mathrm{MV}} (z) ] ^ {\mathrm{T}} \int_ {- \infty} ^ {\infty} [ x - E (X / z) ] p (x / z) \mathrm{d} x \end{array}
$$

由于

$$
\begin{array}{r l} & \int_ {- \infty} ^ {\infty} p (x / z) \mathrm{d} x = 1 \\ & \int_ {- \infty} ^ {\infty} [ x - E (X / z) ] p (x / z) \mathrm{d} x = 0 \end{array}
$$

所以

$$
\begin{array}{r l} g = & \int_ {- \infty} ^ {\infty} [ \boldsymbol {x} - E (\boldsymbol {X} / \boldsymbol {z}) ] ^ {\mathrm{T}} [ \boldsymbol {x} - E (\boldsymbol {X} / \boldsymbol {z}) ] p (\boldsymbol {x} / \boldsymbol {z}) \mathrm{d} \boldsymbol {x} + \\ & [ E (\boldsymbol {X} / \boldsymbol {z}) - \hat {\boldsymbol {X}} _ {\mathrm{MV}} (\boldsymbol {z}) ] ^ {\mathrm{T}} [ E (\boldsymbol {X} / \boldsymbol {z}) - \hat {\boldsymbol {X}} _ {\mathrm{MV}} (\boldsymbol {z}) ] \end{array}
$$

上式中，第一项与 $\hat{X}_{\mathrm{MV}}(z)$ 的选取无关，第二项为向量的内积，总是大于或等于零，所以要使 $g$

达到最小,必须使该项内积为零,而要使内积为零,必须使该向量为零,即

$$
\hat {\boldsymbol {X}} _ {\mathrm{MV}} (\boldsymbol {z}) = E [ \boldsymbol {X} / \boldsymbol {z} ]
$$

注意到 z 是式(2.1.28)中的积分变量,积分区间为 Z 的整个样本空间,所以在 Z 的整个样本空间内上式均成立,即

$$
\hat {\mathbf {X}} _ {\mathrm{MV}} (\mathbf {Z}) = E [ \mathbf {X} / \mathbf {Z} ]
$$

2. 最小方差估计的无偏性

定理2.2

最小方差估计是 X 的无偏估计, 即

$$
E [ \mathbf {X} - \hat {\mathbf {X}} _ {\mathrm{MV}} (\mathbf {Z}) ] = \mathbf {0}\tag{2.1.29}
$$

或

$$
E [ \hat {\mathbf {X}} _ {\mathrm{MV}} (\mathbf {Z}) ] = E [ \mathbf {X} ]\tag{2.1.30}
$$

证明

由式(2.1.27)，最小方差估计为 $\hat{\mathbf{X}}_{\mathrm{MV}}(\mathbf{Z}) = E[\mathbf{X} / \mathbf{Z}]$ ，其中求均值是对 $\mathbf{X}$ 求均值，所以 $\hat{\mathbf{X}}_{\mathrm{MV}}(\mathbf{Z})$ 是关于 $\mathbf{Z}$ 的函数，对 $\hat{\mathbf{X}}_{\mathrm{MV}}(\mathbf{Z})$ 求均值实质上是对 $\mathbf{Z}$ 求均值，即

$$
\begin{array}{r l} E [ \hat {\mathbf {X}} _ {\mathrm{MV}} (\mathbf {Z}) ] & = E _ {\mathbf {Z}} [ \hat {\mathbf {X}} _ {\mathrm{MV}} (\mathbf {Z}) ] = E _ {\mathbf {Z}} [ E _ {\mathbf {X}} (\mathbf {X} / \mathbf {Z}) ] = \\ & \int_ {- \infty} ^ {\infty} \left[ \int_ {- \infty} ^ {\infty} x p (\mathbf {x} / \mathbf {z}) \mathrm{d} \mathbf {x} \right] p _ {z} (\mathbf {z}) \mathrm{d} \mathbf {z} \end{array}
$$

应用贝叶斯定理, 得

$$
\begin{array}{r l} E [ \hat {\mathbf {X}} _ {\mathrm{MV}} (\mathbf {Z}) ] & = \int_ {- \infty} ^ {\infty} \int_ {- \infty} ^ {\infty} x p (\mathbf {x}, z) \mathrm{d} z \mathrm{d} \mathbf {x} = \int_ {- \infty} ^ {\infty} x \left[ \int_ {- \infty} ^ {\infty} p (\mathbf {x}, z) \mathrm{d} z \right] \mathrm{d} \mathbf {x} = \\ & \quad \int_ {- \infty} ^ {\infty} x p _ {x} (\mathbf {x}) \mathrm{d} x = E [ \mathbf {X} ] \end{array}
$$

3. 正态随机向量的最小方差估计

定理2.3

若被估计向量 $X_{n\times1}$ 和量测向量 $Z_{m\times1}$ 都服从正态分布，且

$$
\begin{array}{r l} & E [ \mathbf {X} ] = m _ {x}, E [ \mathbf {Z} ] = m _ {z} \\ & \mathrm{Cov} [ \mathbf {X}, \mathbf {Z} ] = E [ (\mathbf {X} - m _ {x}) (\mathbf {Z} - m _ {z}) ^ {\mathrm{T}} ] = C _ {x z} \\ & \mathrm{Var} [ \mathbf {Z} ] = E [ (\mathbf {Z} - m _ {z}) (\mathbf {Z} - m _ {z}) ^ {\mathrm{T}} ] = C _ {z} \end{array}
$$

则 $X$ 的最小方差估计为

$$
\hat {\mathbf {X}} _ {\mathrm{MV}} (\mathbf {Z}) = \mathbf {m} _ {X} + \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} (\mathbf {Z} - \mathbf {m} _ {Z})\tag{2.1.31}
$$

该估计的均方误差为

$$
\operatorname{Var} \left[ \boldsymbol {X} - \hat {\boldsymbol {X}} _ {\mathrm{MV}} (\boldsymbol {Z}) \right] = \boldsymbol {P} = \boldsymbol {C} _ {X} - \boldsymbol {C} _ {X Z} \boldsymbol {C} _ {Z} ^ {- 1} \boldsymbol {C} _ {Z X}\tag{2.1.32}
$$

证明

构造随机向量

$$
\mathbf {Y} = \left[ \begin{array}{c} \mathbf {X} \\ \mathbf {Z} \end{array} \right]
$$

则

$$
E [ \mathbf {Y} ] = \boldsymbol {m} _ {Y} = \left[ \begin{array}{l} \boldsymbol {m} _ {X} \\ \boldsymbol {m} _ {Z} \end{array} \right], \operatorname{Var} [ \mathbf {Y} ] = \boldsymbol {C} _ {Y} = \left[ \begin{array}{l l} \boldsymbol {C} _ {X} & \boldsymbol {C} _ {X Z} \\ \boldsymbol {C} _ {Z X} & \boldsymbol {C} _ {Z} \end{array} \right]
$$

X 和 Z 的联合分布密度为

$$
\begin{array}{r l} p (\mathbf {y}) = & p (\mathbf {x}, \mathbf {z}) = \frac {1}{(2 \pi) ^ {\frac {m + n}{2}} | \mathbf {C} _ {Y} | ^ {\frac {1}{2}}} \exp \left\{- \frac {1}{2} [ \mathbf {x} ^ {\mathrm{T}} - \mathbf {m} _ {X} ^ {\mathrm{T}} \quad \mathbf {z} ^ {\mathrm{T}} - \mathbf {m} _ {Z} ^ {\mathrm{T}} ] \mathbf {C} _ {Y} ^ {- 1} \cdot \right. \\ & \left. [ \mathbf {x} ^ {\mathrm{T}} - \mathbf {m} _ {X} ^ {\mathrm{T}} \quad \mathbf {z} ^ {\mathrm{T}} - \mathbf {m} _ {Z} ^ {\mathrm{T}} ] ^ {\mathrm{T}} \right\} \end{array}\tag{2.1.33}
$$

Z 的边缘分布密度为

$$
p (z) = \frac {1}{(2 \pi) ^ {\frac {m}{2}} | C _ {Z} | ^ {\frac {1}{2}}} \exp \left\{- \frac {1}{2} (z - m _ {Z}) ^ {\mathrm{T}} C _ {Z} ^ {- 1} (z - m _ {Z}) \right\}\tag{2.1.34}
$$

根据贝叶斯定理，X 对 Z 的条件分布密度为

$$
p (x / z) = \frac {p (x , z)}{p (z)}\tag{2.1.35}
$$

由于 X 满足正态分布, 所以从 $p(x/z)$ 中可获得 X 对于 Z 的条件均值 E[X/Z], 也就求得 X 的最小方差估计。下面根据 $p(y)$ , $p(z)$ 和贝叶斯公式求出 $p(x/z)$ , 其中 $C_{Y}^{-1}$ 和 $|C_{Y}|$ 的求解如下:

(1) $C_{Y}^{-1}$ 的求解。由于

$$
\begin{array}{r l} \left[ \begin{array}{c c} I & - C _ {X Z} C _ {Z} ^ {- 1} \\ 0 & I \end{array} \right] & \left[ \begin{array}{c c} C _ {X} & C _ {X Z} \\ C _ {Z X} & C _ {Z} \end{array} \right] \left[ \begin{array}{c c} I & 0 \\ - C _ {Z} ^ {- 1} C _ {X Z} ^ {\mathrm{T}} & I \end{array} \right] = \left[ \begin{array}{c c} C _ {X} - C _ {X Z} C _ {Z} ^ {- 1} C _ {Z X} & 0 \\ 0 & C _ {Z} \end{array} \right] \\ & \left[ \begin{array}{c c} C _ {X} & C _ {X Z} \\ C _ {Z X} & C _ {Z} \end{array} \right] = C _ {Y} \end{array}\tag{2.1.36}
$$

式中

所以对上式两边分别求逆，再左乘 $\begin{bmatrix} I & 0 \\ -C_Z^{-1}C_{XZ}^{\mathrm{T}} & I \end{bmatrix}$ ，右乘 $\begin{bmatrix} I & -C_{XZ}C_Z^{-1} \\ 0 & I \end{bmatrix}$ ，得

$$
\boldsymbol {C} _ {Y} ^ {- 1} = \left[ \begin{array}{c c} \boldsymbol {I} & \boldsymbol {0} \\ - \boldsymbol {C} _ {Z} ^ {- 1} \boldsymbol {C} _ {X Z} ^ {\mathrm{T}} & \boldsymbol {I} \end{array} \right] \left[ \begin{array}{c c} (\boldsymbol {C} _ {X} - \boldsymbol {C} _ {X Z} \boldsymbol {C} _ {Z} ^ {- 1} \boldsymbol {C} _ {Z X}) ^ {- 1} & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {C} _ {Z} ^ {- 1} \end{array} \right] \left[ \begin{array}{c c} \boldsymbol {I} & - \boldsymbol {C} _ {X Z} \boldsymbol {C} _ {Z} ^ {- 1} \\ \boldsymbol {0} & \boldsymbol {I} \end{array} \right]\tag{2.1.37}
$$

(2) $|C_{Y}|$ 的求取。对式(2.1.36)两边分别求取行列式,得

$$
\left| \begin{array}{c c} I & - C _ {X Z} C _ {Z} ^ {- 1} \\ 0 & I \end{array} \right| \cdot | C _ {Y} | \cdot \left| \begin{array}{c c} I & 0 \\ - C _ {Z} ^ {- 1} C _ {X Z} ^ {\mathrm{T}} & I \end{array} \right| = \left| \begin{array}{c c} C _ {X} - C _ {X Z} C _ {Z} ^ {- 1} C _ {Z X} & 0 \\ 0 & C _ {Z} \end{array} \right|
$$

按拉普拉斯法对各行列式作展开,得

$$
\mid C _ {Y} \mid = \mid C _ {X} - C _ {X Z} C _ {Z} ^ {- 1} C _ {Z X} \mid \cdot \mid C _ {Z} \mid\tag{2.1.38}
$$

将式 $(2.1.37)$ 和式 $(2.1.38)$ 代入式 $(2.1.33)$ ，其中指数部分为

$$
\begin{array}{l} E = \left[ \begin{array}{c c} x ^ {\mathrm{T}} - m _ {X} ^ {\mathrm{T}} & z ^ {\mathrm{T}} - m _ {Z} ^ {\mathrm{T}} \end{array} \right] C _ {Y} ^ {- 1} \left[ \begin{array}{c c} x ^ {\mathrm{T}} - m _ {X} ^ {\mathrm{T}} & z ^ {\mathrm{T}} - m _ {Z} ^ {\mathrm{T}} \end{array} \right] ^ {\mathrm{T}} = \\ \left\{\left[ (x - m _ {X}) ^ {\mathrm{T}} (z - m _ {Z}) ^ {\mathrm{T}} \right] \left[ \begin{array}{c c} I & 0 \\ - C _ {Z} ^ {- 1} C _ {X Z} ^ {\mathrm{T}} & I \end{array} \right] \left[ \begin{array}{c c} (C _ {X} - C _ {X Z} C _ {Z} ^ {- 1} C _ {Z X}) ^ {- 1} & 0 \\ 0 & C _ {Z} ^ {- 1} \end{array} \right] \right\}. \\ \left\{\left[ \begin{array}{c c} I & - C _ {X Z} C _ {Z} ^ {- 1} \\ 0 & I \end{array} \right] \left[ \begin{array}{c} x - m _ {X} \\ z - m _ {Z} \end{array} \right] \right\} = \left[ \left\{(x - m _ {X}) ^ {\mathrm{T}} - (z - m _ {Z}) ^ {\mathrm{T}} C _ {Z} ^ {- 1} C _ {X Z} ^ {\mathrm{T}} \right\} \cdot \right. \\ (C _ {X} - C _ {X Z} C _ {Z} ^ {- 1} C _ {Z X}) ^ {- 1} (z - m _ {Z}) ^ {\mathrm{T}} C _ {Z} ^ {- 1} \left[ \begin{array}{c} x - m _ {X} - C _ {X Z} C _ {Z} ^ {- 1} (z - m _ {Z}) \\ z - m _ {Z} \end{array} \right] \end{array}
$$

令 $\overline{m}_X = m_X + C_{XZ}C_Z^{-1}(z - m_Z)$ ，得

$$
\begin{array}{r l} E = & \left[ (x ^ {\mathrm{T}} - \overline {{m}} _ {X} ^ {\mathrm{T}}) (C _ {X} - C _ {X Z} C _ {Z} ^ {- 1} C _ {Z X}) ^ {- 1} \quad (z - m _ {Z}) ^ {\mathrm{T}} C _ {Z} ^ {- 1} \right] \left[ \begin{array}{l} x - \overline {{m}} _ {X} \\ z - m _ {Z} \end{array} \right] = \\ & (x ^ {\mathrm{T}} - \overline {{m}} _ {X} ^ {\mathrm{T}}) (C _ {X} - C _ {X Z} C _ {Z} ^ {- 1} C _ {Z X}) ^ {- 1} (x - \overline {{m}} _ {x}) + (z - m _ {Z}) ^ {\mathrm{T}} C _ {Z} ^ {- 1} (z - m _ {Z}) \end{array}
$$

所以

$$
\begin{array}{r l} p (x, z) = & \frac {1}{(2 \pi) ^ {\frac {m + n}{2}} \left| C _ {X} - C _ {X Z} C _ {Z} ^ {- 1} C _ {Z X} \right| ^ {\frac {1}{2}} \left| C _ {Z} \right| ^ {\frac {1}{2}}} \cdot \\ & \exp \left\{- \frac {1}{2} (x - \overline {{m}} _ {X}) ^ {\mathrm{T}} (C _ {X} - C _ {X Z} C _ {Z} ^ {- 1} C _ {Z X}) ^ {- 1} (x - \overline {{m}} _ {X}) \right\} \cdot \\ & \exp \left\{- \frac {1}{2} (z - m _ {Z}) ^ {\mathrm{T}} C _ {Z} ^ {- 1} \cdot (z - m _ {Z}) \right\} \end{array}\tag{2.1.39}
$$

将式(2.1.34)和式(2.1.39)代入式(2.1.35)，得

$$
\begin{array}{r l} p (\boldsymbol {x} / \boldsymbol {z}) & = \frac {1}{(2 \pi) ^ {\frac {n}{2}} \left| \boldsymbol {C} _ {X} - \boldsymbol {C} _ {X Z} \boldsymbol {C} _ {Z} ^ {- 1} \boldsymbol {C} _ {Z X} \right| ^ {\frac {1}{2}}} \cdot \\ & \exp \left\{- \frac {1}{2} (\boldsymbol {x} - \overline {{\boldsymbol {m}}} _ {X}) ^ {\mathrm{T}} (\boldsymbol {C} _ {X} - \boldsymbol {C} _ {X Z} \boldsymbol {C} _ {Z} ^ {- 1} \boldsymbol {C} _ {Z X}) ^ {- 1} (\boldsymbol {x} - \overline {{\boldsymbol {m}}} _ {X}) \right\} \end{array}\tag{2.1.40}
$$

从上式得 $X$ 关于 $\mathbf{Z}$ 的条件均值为

$$
E [ \mathbf {X} / \mathbf {Z} ] = \overline {{\boldsymbol {m}}} _ {X} = \boldsymbol {m} _ {X} + \boldsymbol {C} _ {X Z} \boldsymbol {C} _ {Z} ^ {- 1} (\mathbf {Z} - \boldsymbol {m} _ {Z})\tag{2.1.41}
$$

X 偏离 $\overline{m}_{x}$ 的方差阵为

$$
\boldsymbol {P} = \boldsymbol {C} _ {X} - \boldsymbol {C} _ {X Z} \boldsymbol {C} _ {Z} ^ {- 1} \boldsymbol {C} _ {Z X}\tag{2.1.42}
$$

根据定理 2.1, 上述两式分别为 X 的最小方差估计和估计的均方误差阵的计算公式。

定理 2.3 说明, 当被估计量 X 和量测量 Z 都服从正态分布时, X 的最小方差估计不必通过对条件概率密度的积分求取, 而只须知道 X 及 Z 的一阶和二阶矩。又从式(2.1.31) 知, $\hat{X}_{\mathrm{MV}}(\mathbf{Z})$ 是关于量测量 Z 的线性函数, 所以 $\hat{X}_{\mathrm{MV}}(\mathbf{Z})$ 是一种线性估计。

若量测量 Z 与被估计量 X 间具有线性关系，则式(2.1.31)和式(2.1.32)中的互协方差阵 $C_{XZ}$ 和 $C_{ZX}$ 可简化成 X 和 Z 各自的一、二阶矩，这给计算带来更大的方便。

$$
\mathbf {Z} = \mathbf {H X} + \mathbf {V}\tag{2.1.43}
$$

式中 V 为量测噪声， $E[V]=0$ ， $\operatorname{Var}[V]=C_{V}$ ， $E[X]=m_{X}$ ， $\operatorname{Var}[X]=C_{X}$ ，X 和 V 互不相关，H 为量测阵，是确定性矩阵。

由式(2.1.43)得

$$
\begin{array}{r l} & {\pmb {m} _ {Z} = \pmb {H m} _ {X}} \\ & {\pmb {C} _ {X Z} = E [ (\pmb {X} - \pmb {m} _ {X}) (\pmb {Z} - \pmb {m} _ {Z}) ^ {\mathrm{T}} ] = \pmb {C} _ {X} \pmb {H} ^ {\mathrm{T}}} \\ & {\pmb {C} _ {Z X} = \pmb {C} _ {X Z} ^ {\mathrm{T}} = \pmb {H C} _ {X}} \\ & {\pmb {C} _ {Z} = E [ (\pmb {Z} - \pmb {m} _ {Z}) (\pmb {Z} - \pmb {m} _ {Z}) ^ {\mathrm{T}} ] = \pmb {H C} _ {X} \pmb {H} ^ {\mathrm{T}} + \pmb {C} _ {V}} \end{array}
$$

将上述诸式代入式(2.1.31)和式(2.1.32)，得

$$
\hat {\mathbf {X}} _ {M V} (\mathbf {Z}) = \mathbf {m} _ {X} + \mathbf {C} _ {X} \mathbf {H} ^ {\mathrm{T}} (\mathbf {H C} _ {X} \mathbf {H} ^ {\mathrm{T}} + \mathbf {C} _ {V}) ^ {- 1} (\mathbf {Z} - \mathbf {H m} _ {X})\tag{2.1.44}
$$

$$
\boldsymbol {P} = \boldsymbol {C} _ {X} - \boldsymbol {C} _ {X} \boldsymbol {H} ^ {\mathrm{T}} (\boldsymbol {H C} _ {X} \boldsymbol {H} ^ {\mathrm{T}} + \boldsymbol {C} _ {V}) ^ {- 1} \boldsymbol {H C} _ {X}\tag{2.1.45}
$$

上述两式的另一种表达式为

$$
\hat {\boldsymbol {X}} _ {\mathrm{MV}} (\boldsymbol {Z}) = \left(\boldsymbol {C} _ {X} ^ {- 1} + \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {C} _ {V} ^ {- 1} \boldsymbol {H}\right) ^ {- 1} \left(\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {C} _ {V} ^ {- 1} \boldsymbol {Z} + \boldsymbol {C} _ {X} ^ {- 1} \boldsymbol {m} _ {X}\right)\tag{2.1.46}
$$

$$
\boldsymbol {P} = (\boldsymbol {C} _ {X} ^ {- 1} + \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {C} _ {V} ^ {- 1} \boldsymbol {H}) ^ {- 1}\tag{2.1.47}
$$

例2-6 设 $X$ 为服从正态分布的随机变量，均值为 $m_X$ ，方差为 $C_X$ ，对 $X$ 用 $m$ 台仪器同时直接测量，测量误差都是服从正态分布的随机变量，均值为零，方差为 $C_V$ ，求 $X$ 的最小方差估计和估计的均方误差。

解 根据题意,量测方程为 $Z=HX+V$

式中 $Z=\begin{bmatrix}Z_{1}\\ Z_{2}\\ \vdots\\ Z_{m}\end{bmatrix},H=\begin{bmatrix}1\\ 1\\ \vdots\\ 1\end{bmatrix},V=\begin{bmatrix}V_{1}\\ V_{2}\\ \vdots\\ V_{m}\end{bmatrix},VarV=C_{V}I$ ，由式(2.1.46)和式(2.1.47)，得

$$
\begin{array}{r l} \hat {X} _ {\mathrm{MV}} & = \left(\frac {1}{C _ {X}} + \frac {m}{C _ {V}}\right) ^ {- 1} \left(\frac {1}{C _ {V}} \sum_ {i = 1} ^ {m} Z _ {i} + \frac {m _ {X}}{C _ {X}}\right) = \\ & m _ {X} + \frac {m C _ {X}}{m C _ {X} + C _ {V}} \left(\frac {1}{m} \sum_ {i = 1} ^ {m} Z _ {i} - m _ {X}\right) \\ P & = \left(\frac {1}{C _ {x}} + \frac {m}{C _ {V}}\right) ^ {- 1} = \frac {C _ {X} C _ {V}}{m C _ {X} + C _ {V}} \end{array}
$$

由于 $P=\mathrm{Var}(X-\hat{X})$ ，它反映了估计误差的分散程度，所以 P 描述了估计的精度，P 越小，估计精度就越高。从 P 的表达式可看出，测量仪器越多，估计精度就越高。

## 2.1.3 极大验后估计

设 X 为随机向量, Z 为 X 的量测, $p(x/z)^{\circ}$ 为 Z=z 条件下 X 的条件概率密度(亦称 X 的验后概率密度)。如果估计值 $\hat{X}_{\mathrm{MA}}(z)$ 使下述指标满足

$$
p (x / z) \mid_ {x = \hat {X} _ {\mathrm{MA}} (z)} = \max\tag{2.1.48}
$$

则 $\hat{X}_{\mathrm{MA}}(z)$ 称为 $X$ 的极大验后估计，其中 $z$ 为 $Z$ 的某一实现。

由于 $\ln p(x/z)$ 与 $p(x/z)$ 在相同的 x 处取得极大值，所以式(2.1.48) 又可改写为

$$
\ln p (x / z) \mid_ {x = \hat {X} _ {\mathrm{MA}} (z)} = \max\tag{2.1.49}
$$

定理2.4

如果 X 和 Z 都服从正态分布, 则 X 的极大验后估计与 X 的最小方差估计相等, 即

$$
\hat {\mathbf {X}} _ {\mathrm{MA}} (\mathbf {Z}) = \hat {\mathbf {X}} _ {\mathrm{MV}} (\mathbf {Z})\tag{2.1.50}
$$

证明 由于 X 和 Z 都服从正态分布, 根据式(2.1.40)～式(2.1.42)得 X 的条件概率密度为

$$
p (\boldsymbol {x} / \boldsymbol {z}) = \frac {1}{(2 \pi) ^ {\frac {n}{2}} | \boldsymbol {P} | ^ {\frac {1}{2}}} \exp \left\{- \frac {1}{2} [ \boldsymbol {x} - \hat {\boldsymbol {X}} _ {\mathrm{MV}} (\boldsymbol {z}) ] ^ {\mathrm{T}} \boldsymbol {P} ^ {- 1} [ \boldsymbol {x} - \hat {\boldsymbol {X}} _ {\mathrm{MV}} (\boldsymbol {z}) ] \right\}
$$

对上式两边求对数, 得

$$
\ln p (\boldsymbol {x} / \boldsymbol {z}) = - \frac {n}{2} \ln (2 \pi) - \frac {1}{2} \ln | \boldsymbol {P} | - \frac {1}{2} [ \boldsymbol {x} - \hat {\boldsymbol {X}} _ {\mathrm{MV}} (\boldsymbol {z}) ] ^ {\mathrm{T}} \cdot \boldsymbol {P} ^ {- 1} [ \boldsymbol {x} - \boldsymbol {X} _ {\mathrm{MV}} (\boldsymbol {z}) ]
$$

要使上式达到最大值,必须有

$$
\left. \frac {\partial \ln p (x / z)}{\partial x} \right| _ {x = \hat {X} _ {\mathrm{MA}} (z)} = - P ^ {- 1} \left[ \hat {X} _ {\mathrm{MA}} (z) - \hat {X} _ {\mathrm{MV}} (z) \right] = 0
$$

所以

$$
\hat {\mathbf {X}} _ {\mathrm{MA}} (\mathbf {Z}) = \hat {\mathbf {X}} _ {\mathrm{MV}} (\mathbf {Z})
$$

必须注意, 只有当 X 和 Z 都服从正态分布时, 式(2.1.50)才成立。如果 X 和 Z 并不都满足正态分布, 则极大验后估计必须根据式(2.1.49)求取。

## 2.1.4 贝叶斯估计

前面介绍的最小方差估计和极大验后估计,实质上都是贝叶斯估计的特殊形式,因此有必

要对贝叶斯估计作介绍。

设 X 为被估计量, Z 是 X 的量测量, $\hat{X}(Z)$ 是根据 Z 给出的对 X 的估计, $\tilde{X} = X - \hat{X}(Z)$ 为估计误差, 如果标量函数

$$
L (\tilde {\boldsymbol {X}}) = L [ \boldsymbol {X} - \hat {\boldsymbol {X}} (\boldsymbol {Z}) ]\tag{2.1.51}
$$

具有如下性质：

(1) 当 $\|\tilde{X}_{2}\|\geqslant\|\tilde{X}_{1}\|$ 时， $L(\tilde{X}_{2})\geqslant L(\tilde{X}_{1})\geqslant0;$

(2) 当 $\|\tilde{X}\|=0$ 时, $L(\tilde{X})=0$ ;

(3) $L(\widetilde{\mathbf{X}}) = L(-\widetilde{\mathbf{X}})$

则称 $L(\tilde{X})$ 为 $\hat{X} (Z)$ 对被估计量 $X$ 的损失函数，也称代价函数，并称其期望值

$$
B (\hat {\mathbf {X}}) = E [ L (\tilde {\mathbf {X}}) ]\tag{2.1.52}
$$

为 $\hat{X}(Z)$ 的贝叶斯风险。其中 $\|\tilde{X}\|$ 为 $\tilde{X}$ 的范数。

将上式中的数学期望写成积分形式

$$
\begin{array}{r l} B (\hat {\boldsymbol {X}}) & = \int_ {- \infty} ^ {\infty} \int_ {- \infty} ^ {\infty} L [ \boldsymbol {x} - \hat {\boldsymbol {X}} (\boldsymbol {z}) ] p (\boldsymbol {x}, \boldsymbol {z}) \mathrm{d} \boldsymbol {x} \mathrm{d} \boldsymbol {z} = \\ & \quad \int_ {- \infty} ^ {\infty} \left\{\int_ {- \infty} ^ {\infty} L [ \boldsymbol {x} - \hat {\boldsymbol {X}} (\boldsymbol {z}) ] p (\boldsymbol {x} / \boldsymbol {z}) \mathrm{d} \boldsymbol {x} \right\} p _ {Z} (\boldsymbol {z}) \mathrm{d} \boldsymbol {z} \end{array}
$$

如果估计量 $\hat{X}_{B}(Z)$ 使贝叶斯风险

$$
B (\hat {\mathbf {X}} _ {B}) = E \left\{L (\mathbf {X} - \hat {\mathbf {X}} (\mathbf {Z}) \right] \} _ {\hat {\mathbf {X}} = \hat {\mathbf {X}} _ {B} (\mathbf {Z})} = \min
$$

则称 $\hat{X}_{B}(Z)$ 为 X 的贝叶斯估计。

显然，当 $L(\tilde{\mathbf{X}}) = \tilde{\mathbf{X}}^{\mathrm{T}}\tilde{\mathbf{X}}$ 时， $\hat{\mathbf{X}}_B(\mathbf{Z})$ 就是 $\mathbf{X}$ 的最小方差估计 $\hat{\mathbf{X}}_{\mathrm{MV}}(\mathbf{Z})$ 。

下面再分析贝叶斯估计与极大验后估计的关系。取估计量 $\hat{X}$ 的损失函数为

$$
L (\boldsymbol {X} - \hat {\boldsymbol {X}}) = \left\{ \begin{array}{l l} 0, & \| \boldsymbol {X} - \hat {\boldsymbol {X}} \| <   \frac {\varepsilon}{2} \\ \frac {1}{\varepsilon}, & \| \boldsymbol {X} - \hat {\boldsymbol {X}} \| \geqslant \frac {\varepsilon}{2} \end{array} \right.\tag{2.1.53}
$$

$\hat{\pmb{X}}$ 的贝叶斯风险为

$$
\begin{array}{l} B (\hat {\mathbf {X}}) = E [ L (\mathbf {X} - \hat {\mathbf {X}}) ] = \int_ {- \infty} ^ {\infty} \left[ \int_ {\| \mathbf {X} - \hat {\mathbf {X}} \| \geqslant \frac {\varepsilon}{2}} \frac {1}{\varepsilon} p (\mathbf {x} / \mathbf {z}) \mathrm{d} \mathbf {x} \right] p _ {Z} (\mathbf {z}) \mathrm{d} \mathbf {z} = \\ \int_ {- \infty} ^ {\infty} \frac {1}{\varepsilon} \left[ 1 - \int_ {\| \mathbf {X} - \hat {\mathbf {X}} \| <   \frac {\varepsilon}{2}} p (\mathbf {x} / \mathbf {z}) \mathrm{d} \mathbf {x} \right] p _ {Z} (\mathbf {z}) \mathrm{d} \mathbf {z} \end{array}
$$

设 $\hat{\boldsymbol{X}}_{B}(\boldsymbol{Z})$ 为 X 的贝叶斯估计,由上式知

$$
B (\hat {\mathbf {X}}) \mid_ {\hat {\mathbf {x}} = \hat {\mathbf {x}} _ {B}} = \min
$$

等价于

$$
\int_ {\| \boldsymbol {x} - \hat {\boldsymbol {X}} \| <   \frac {\varepsilon}{2}} p (\boldsymbol {x} / \boldsymbol {z}) \mathrm{d} \boldsymbol {x} \Big | _ {\hat {\boldsymbol {x}} = \hat {\boldsymbol {x}} _ {B}} = \max
$$

当 $\varepsilon$ 足够小 $(\varepsilon > 0)$ 时，这又相当于要求

$$
p (x / z) \mid_ {x = \hat {X} _ {B}} = \max
$$

此时 $\hat{X}_{B}$ 又是 X 的极大验后估计 $\hat{X}_{MA}$ 。因此，当损失函数为式(2.1.53)，且 $\varepsilon$ 足够小时，X 的贝叶斯估计就是 X 的极大验后估计。

## 2.1.5 极大似然估计

极大验后估计 $\hat{X}_{\mathrm{MA}}$ 是在 $Z = z$ 条件下，使被估计量 $X$ 的条件概率密度 $p(x / z)$ 达到最大的 $x$ 值，而极大似然估计则是使 $Z$ 的条件概率密度 $p(z / x)$ 在量测值 $Z = z$ 处达到最大的 $x$ 值。

设 X 是被估计量, Z 是 X 的量测值, $p(z/x)$ 是 X=x 的条件下, 量测量 Z 的条件概率密度, $p(z/x)$ 称为 X 的似然函数。如果由获得的量测量 Z=z 解算得的估计值 $\hat{X}_{\mathrm{ML}}(Z)$ 使

$$
p (z / x) \mid_ {x = \hat {X} _ {\mathrm{ML}} (Z)} = \max\tag{2.1.54}
$$

则 $\hat{X}_{\mathrm{ML}}(Z)$ 称为 $X$ 的极大似然估计。

与极大验后估计的定义式一样,式(2.1.54)可改写为

$$
\ln p (z / x) \mid_ {x = \hat {X} _ {\mathrm{ML}}} = \max\tag{2.1.55}
$$

式中 $\hat{X}_{\mathrm{ML}}$ 应满足方程

$$
\left. \frac {\partial \ln p (z / x)}{\partial x} \right| _ {x = \hat {X} _ {\mathrm{ML}}} = 0\tag{2.1.56}
$$

式 $(2.1.56)$ 称为似然方程。

与极大验后估计相比，极大似然估计有两个优点：① 确定似然函数 $p(z / x)$ 比确定验后概率密度 $p(x / z)$ 容易些；② 在极大似然估计中，被估计量 $X$ 可以是随机的，也可以是非随机的，适用的范围比极大验后估计广。但是，如果对 $X$ 已具有验前知识，则极大似然估计 $\hat{X}_{\mathrm{ML}}$ 的精度不如极大验后估计 $\hat{X}_{\mathrm{MA}}$ 的精度高，这是因为求取 $\hat{X}_{\mathrm{MA}}$ 时利用了 $X$ 的验前统计信息 $p_X(x)$ ，而 $\hat{X}_{\mathrm{ML}}$ 中并未利用这一信息。只有在不掌握 $X$ 的任何验前知识时，两者的估计精度才相同。下面说明这一点。

定理2.5

如果对 X 没有任何验前知识, 则 X 的极大似然估计与 X 的极大验后估计相同, 即

$$
\hat {\mathbf {X}} _ {\mathrm{ML}} (\mathbf {Z}) = \hat {\mathbf {X}} _ {\mathrm{MA}} (\mathbf {Z})\tag{2.1.57}
$$

证明 根据联合分布与条件分布及边缘分布之间的关系,有

$$
p (\boldsymbol {x} / \boldsymbol {z}) p _ {Z} (\boldsymbol {z}) = p (\boldsymbol {z} / \boldsymbol {x}) p _ {X} (\boldsymbol {x})
$$

即

$$
p (\boldsymbol {x} / \boldsymbol {z}) = \frac {p (\boldsymbol {z} / \boldsymbol {x}) p _ {X} (\boldsymbol {x})}{p _ {Z} (\boldsymbol {z})}
$$

两边取对数,得

$$
\ln p (x / z) = \ln p (z / x) + \ln p _ {x} (x) - \ln p _ {z} (z)\tag{2.1.58}
$$

设 $\hat{X}_{\mathrm{MA}}$ 为 $X$ 的极大验后估计，即

$$
p (x / z) \mid_ {x = \hat {X} _ {\mathrm{MA}}} = \max
$$

则

$$
\left. \frac {\partial \ln p (x / z)}{\partial x} \right| _ {x = \hat {X} _ {\mathrm{MA}}} = 0
$$

将式(2.1.58)代入上式，并注意到 $\ln p_{z}(z)$ 对 x 的偏导数为零，得

$$
\left. \frac {\partial \ln p (z / x)}{\partial x} \right| _ {x = \dot {X} _ {\mathrm{MA}}} + \left. \frac {\mathrm{d} \ln p _ {X} (x)}{\mathrm{d} x} \right| _ {x = \dot {X} _ {\mathrm{MA}}} = 0\tag{2.1.59}
$$

由于假设对 X 没有任何验前知识, 只能认为 X 取得任何值的概率都相等, 于是可将 X 的验前概率密度近似看做方差为无穷大的高斯概率密度

$$
p _ {X} (\boldsymbol {x}) = \frac {1}{\sqrt {(2 \pi) ^ {n} | \boldsymbol {V} _ {X} |}} \exp \left[ - \frac {1}{2} (\boldsymbol {x} - \boldsymbol {\mu} _ {X}) ^ {\mathrm{T}} \boldsymbol {V} _ {X} ^ {- 1} (\boldsymbol {x} - \boldsymbol {\mu} _ {X}) \right]
$$

即

$$
\ln p _ {X} (\boldsymbol {x}) = - \frac {1}{2} \ln [ (2 \pi) ^ {n} | \boldsymbol {V} _ {X} | ] - \frac {1}{2} (\boldsymbol {x} - \boldsymbol {\mu} _ {X}) ^ {\mathrm{T}} \boldsymbol {V} _ {X} ^ {- 1} (\boldsymbol {x} - \boldsymbol {\mu} _ {X})
$$

式中

$$
V _ {X} = \sigma^ {2} I \qquad \sigma \rightarrow \infty
$$

所以

$$
\frac {\mathrm{d} \ln p _ {X} (\boldsymbol {x})}{\mathrm{d} \boldsymbol {x}} = - \boldsymbol {V} _ {X} ^ {- 1} (\boldsymbol {x} - \boldsymbol {\mu} _ {X}) = - \lim _ {\sigma \rightarrow \infty} \frac {1}{\sigma^ {2}} \boldsymbol {I} (\boldsymbol {x} - \boldsymbol {\mu} _ {X}) = \mathbf {0}
$$

因此式(2.1.59)为

$$
\left. \frac {\partial \ln p (z / x)}{\partial x} \right| _ {x = \dot {X} _ {\mathrm{MA}}} = 0\tag{2.1.60}
$$

比较式(2.1.60)和式(2.1.56)知,在缺乏X的任何验前知识时

$$
\hat {\boldsymbol {X}} _ {\mathrm{MA}} = \hat {\boldsymbol {X}} _ {\mathrm{ML}}
$$

例2-7 设 $X$ 为 $n$ 维随机向量，服从 $N(\mu, P)$ 分布， $Z = HX + V$ ，其中 $V$ 为 $m$ 维随机向量，服从 $N(0, R)$ 分布， $X$ 与 $V$ 相互独立，求 $\hat{X}_{\mathrm{ML}}$ 。

解 由于 X 和 V 都服从正态分布, 所以 $Z = HX + V$ 也服从正态分布, X 和 Z 的联合分布 $p(x, z)$ 也是正态的, 根据贝叶斯定理

$$
p (z / x) = \frac {p (x , z)}{p _ {x} (x)}
$$

也是正态的。要获得 $p(z / x)$ ，只须求取 $\mathbf{Z}$ 的条件均值和条件方差。由定理2.1知， $\mathbf{Z}$ 在 $X = x$ 条件下的条件均值即为 $\mathbf{Z}$ 的最小方差估计，根据式(2.1.31)和式(2.1.32)，得 $\mathbf{Z}$ 的条件均值和条件方差为

$$
\begin{array}{r l} & E [ \mathbf {Z} / \mathbf {x} ] = E [ \mathbf {Z} ] + \operatorname{Cov} (\mathbf {Z}, \mathbf {X}) (\operatorname{Var} [ \mathbf {X} ]) ^ {- 1} (\mathbf {x} - E [ \mathbf {X} ]) \\ & \operatorname{Var} [ \mathbf {Z} / \mathbf {x} ] = \operatorname{Var} [ \mathbf {Z} ] - \operatorname{Cov} (\mathbf {Z}, \mathbf {X}) (\operatorname{Var} [ \mathbf {X} ]) ^ {- 1} \operatorname{Cov} (\mathbf {X}, \mathbf {Z}) \end{array}
$$

而

$$
\begin{array}{r l} & E [ X ] = \mu , \quad \mathrm{Var} [ X ] = P \\ & E [ Z ] = H \mu , \quad \mathrm{Var} [ Z ] = H P H ^ {\mathrm{T}} + R \\ & \mathrm{Cov} (X, Z) = P H ^ {\mathrm{T}} \\ & \mathrm{Cov} (Z, X) = [ \mathrm{Cov} (X, Z) ] ^ {\mathrm{T}} = H P \end{array}
$$

所以

$$
\begin{array}{r l} & E [ \mathbf {Z} / x ] = H \boldsymbol {\mu} + H P P ^ {- 1} (x - \boldsymbol {\mu}) = H x \\ & \operatorname{Var} [ \mathbf {Z} / x ] = H P H ^ {\mathrm{T}} + R - H P P ^ {- 1} P H ^ {\mathrm{T}} = R \end{array}
$$

因此似然函数为

$$
p (\mathbf {z} / \mathbf {x}) = \frac {1}{\sqrt {(2 \pi) ^ {m} | \mathbf {R} |}} \exp \left[ - \frac {1}{2} (\mathbf {z} - \mathbf {H x}) ^ {\mathrm{T}} \mathbf {R} ^ {- 1} (\mathbf {z} - \mathbf {H x}) \right]
$$

对数形式为

$$
\ln p (\mathbf {z} / \mathbf {x}) = - \frac {m}{2} \ln (2 \pi) - \frac {1}{2} \ln | \mathbf {R} | - \frac {1}{2} (\mathbf {z} - \mathbf {H x}) ^ {\mathrm{T}} \mathbf {R} ^ {- 1} (\mathbf {z} - \mathbf {H x})
$$

极大似然估计应满足

$$
\left. \frac {\partial \ln p (z / x)}{\partial x} \right| _ {x = \dot {X} _ {\mathrm{ML}}} = 0
$$

即

$$
\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {R} ^ {- 1} (\boldsymbol {z} - \boldsymbol {H} \hat {\boldsymbol {X}} _ {\mathrm{ML}}) = \boldsymbol {0}
$$

所以

$$
\hat {\boldsymbol {X}} _ {\mathrm{ML}} = (\boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {R} ^ {- 1} \boldsymbol {H}) ^ {- 1} \boldsymbol {H} ^ {\mathrm{T}} \boldsymbol {R} ^ {- 1} \boldsymbol {z}
$$

式中 z 是某一量测值。

## 2.1.6 线性最小方差估计

## 1. 线性最小方差估计的求取

由式(2.1.31)知,当被估计量 X 和量测量 Z 都服从正态分布时,X 的最小方差估计 $\hat{X}_{MV}$ 是 Z 的线性函数,求解 $\hat{X}_{MV}$ 时只须知道 X 和 Z 的一、二阶矩。现在反过来问,如果 X 和 Z 并不满足正态分布,而 X 的估计也要表达成 Z 的线性函数,满足的最优指标也是使均方误差最小,X 的估计应怎样求,这一估计与 X 的最小方差估计有何不同。

设 X 为被估计量, Z 是 X 的量测量, 如果 $\hat{X}_{L}(Z)=AZ+b$ 使下述指标满足

$$
E \left[ (\boldsymbol {X} - \hat {\boldsymbol {X}}) ^ {\mathrm{T}} (\boldsymbol {X} - \hat {\boldsymbol {X}}) \right] | _ {\hat {\boldsymbol {X}} = \hat {\boldsymbol {X}} _ {L} (\mathbf {Z})} = \min\tag{2.1.61}
$$

则称 $\hat{X}_L(Z)$ 为 $X$ 在 $Z$ 上的线性最小方差估计，有时用符号 $E^{*}[X / Z]$ 表示之， $*$ 表示与条件均值的区别。

有关量测量 Z 的线性函数有无穷多个,但能使 $\hat{X}$ 具有最小均方误差的线性函数却只有一个,记为

$$
\hat {\mathbf {X}} _ {L} (\mathbf {Z}) = \mathbf {A} ^ {0} \mathbf {Z} + \mathbf {b} ^ {0}
$$

$$
\boldsymbol {A} ^ {0}
$$

$$
\boldsymbol {b} ^ {0}
$$

关系式1

设随机向量 $W = [W_{1} W_{2} \cdots W_{n}]^{T}$ ，则

$$
E [ \boldsymbol {W} ^ {\mathrm{T}} \boldsymbol {W} ] = \operatorname{tr} (\boldsymbol {C} _ {w} + E \boldsymbol {W} \boldsymbol {E} \boldsymbol {W} ^ {\mathrm{T}})
$$

式中 $C_{w}=E\left[(W-EW)(W-EW)^{\mathrm{T}}\right]$ ，tr 表示对矩阵求迹。

证明

$$
\begin{array}{r l} E [ \mathbf {W} ^ {\mathrm{T}} \mathbf {W} ] & = E [ W _ {1} ^ {2} + W _ {2} ^ {2} + \dots + W _ {n} ^ {2} ] = E [ \operatorname{tr} (\mathbf {W W} ^ {\mathrm{T}}) ] = \\ & \quad \operatorname{tr} \{E [ (\mathbf {W} - E \mathbf {W} + E \mathbf {W}) (\mathbf {W} - E \mathbf {W} + E \mathbf {W}) ^ {\mathrm{T}} ] \} = \\ & \quad \operatorname{tr} \{\mathbf {C} _ {w} + E \mathbf {W} E \mathbf {W} ^ {\mathrm{T}} + E [ (\mathbf {W} - E \mathbf {W}) E \mathbf {W} ^ {\mathrm{T}} ] + E [ E \mathbf {W} (\mathbf {W} - E \mathbf {W}) ^ {\mathrm{T}} ] \} = \\ & \quad \operatorname{tr} (\mathbf {C} _ {w} + E \mathbf {W} E \mathbf {W} ^ {\mathrm{T}}) \end{array}
$$

关系式2

设 $Y = X - A^0 Z - b^0$ ， $EX = m_X$ ， $EZ = m_Z$ ，则

$$
\begin{array}{r l} & E \mathbf {Y} = \mathbf {m} _ {X} - \mathbf {A} ^ {0} \mathbf {m} _ {Z} - \mathbf {b} ^ {0} \\ & \mathbf {C} _ {Y} = E [ (\mathbf {Y} - E \mathbf {Y}) (\mathbf {Y} - E \mathbf {Y}) ^ {\mathrm{T}} ] = E \{\left[ (\mathbf {X} - \mathbf {m} _ {X}) - \mathbf {A} ^ {0} (\mathbf {Z} - \mathbf {m} _ {Z}) \right] [ (\mathbf {X} - \mathbf {m} _ {X}) ^ {\mathrm{T}} - \\ & (\mathbf {Z} - \mathbf {m} _ {Z}) ^ {\mathrm{T}} \mathbf {A} ^ {0 \mathrm{T}} ] \} = \mathbf {C} _ {X} + \mathbf {A} ^ {0} \mathbf {C} _ {Z} \mathbf {A} ^ {0 \mathrm{T}} - \mathbf {C} _ {X Z} \mathbf {A} ^ {0 \mathrm{T}} - \mathbf {A} ^ {0} \mathbf {C} _ {Z X} \end{array}
$$

关系式3

设 $L_{(n\times m)}$ , $C_{Z(m\times m)}$ ，其中 $C_{Z}$ 为随机向量 Z 的协方差阵，则 $\mathrm{tr}[LC_{Z}L^{T}]\geqslant0$ ; 若 $\mathrm{tr}[LC_{Z}L^{T}]=0$ ，则 L=0。

证明

$$
\operatorname{tr} \left[ L C _ {Z} L ^ {\mathrm{T}} \right] = \operatorname{tr} \left\{L E \left[ (Z - m _ {Z}) (Z - m _ {Z}) ^ {\mathrm{T}} \right] L ^ {\mathrm{T}} \right\} = \operatorname{tr} \left\{E \left[ [ L (Z - m _ {Z}) ] [ L (Z - m _ {Z}) ] ^ {\mathrm{T}} \right] \right\} =
$$

$$
E \{\operatorname{tr} [ [ L (Z - m _ {z}) ] [ L (Z - m _ {z}) ] ^ {\mathrm{T}} ] \}
$$

令 $L(Z - m_{Z}) = V = [V_{1} V_{2} \cdots V_{n}]^{T}$ ，则

$$
\operatorname{tr} \left[ L C _ {Z} L ^ {\mathrm{T}} \right] = E \left[ \operatorname{tr} \left(V V ^ {\mathrm{T}}\right) \right] = E \left[ V _ {1} ^ {2} + V _ {2} ^ {2} + \dots + V _ {n} ^ {2} \right] \geqslant 0
$$

若 $\operatorname{tr}[LC_Z L^{\mathrm{T}}] = 0$ ，则 $V_{1}^{2} + V_{2}^{2} + \dots + V_{n}^{2} = 0$ ，即

$$
L (Z - m _ {z}) = 0
$$

由于 Z 为随机向量,要使上式恒成立,必须有

$$
L = 0
$$

关系式4

$$
\begin{array}{r l} & (A ^ {0} - C _ {X Z} C _ {Z} ^ {- 1}) C _ {Z} (A ^ {0 \mathrm{T}} - C _ {Z} ^ {- 1} C _ {Z X}) + C _ {X} - C _ {X Z} C _ {Z} ^ {- 1} C _ {Z X} = \\ & A ^ {0} C _ {Z} A ^ {0 \mathrm{T}} - A ^ {0} C _ {Z} C _ {Z} ^ {- 1} C _ {Z X} - C _ {X Z} C _ {Z} ^ {- 1} C _ {Z} A ^ {0 \mathrm{T}} + C _ {X Z} C _ {Z} ^ {- 1} C _ {Z} C _ {Z} ^ {- 1} C _ {Z X} + C _ {X} - C _ {X Z} C _ {Z} ^ {- 1} C _ {Z X} = \\ & C _ {X} + A ^ {0} C _ {Z} A ^ {0 \mathrm{T}} - A ^ {0} C _ {Z X} - C _ {X Z} A ^ {0 \mathrm{T}} \end{array}
$$

利用上述 4 个辅助关系式, 可方便地确定出满足式(2.1.61) 的 $A^{0}$ 和 $b^{0}$ 。

令

则

$$
\begin{array}{r l} & {\mathbf {Y} = \mathbf {X} - \mathbf {A} ^ {0} \mathbf {Z} - \mathbf {b}} \\ & {J = E [ (\mathbf {X} - \mathbf {A} ^ {0} \mathbf {Z} - \mathbf {b} ^ {0}) ^ {\mathrm{T}} (\mathbf {X} - \mathbf {A} ^ {0} \mathbf {Z} - \mathbf {b} ^ {0}) ] = E [ \mathbf {Y} ^ {\mathrm{T}} \mathbf {Y} ]} \end{array}
$$

将关系式 1 代入上式, 得

$$
J = E [ \mathbf {Y} ^ {\mathrm{T}} \mathbf {Y} ] = \operatorname{tr} (\mathbf {C} _ {Y} + E \mathbf {Y} E \mathbf {Y} ^ {\mathrm{T}}) = \operatorname{tr} \mathbf {C} _ {Y} + E \mathbf {Y} ^ {\mathrm{T}} E \mathbf {Y}
$$

将关系式 2 代入上式, 得

$$
J = \operatorname{tr} \left(\boldsymbol {C} _ {X} + \boldsymbol {A} ^ {0} \boldsymbol {C} _ {Z} \boldsymbol {A} ^ {0 \mathrm{T}} - \boldsymbol {C} _ {X Z} \boldsymbol {A} ^ {0 \mathrm{T}} - \boldsymbol {A} ^ {0} \boldsymbol {C} _ {Z X}\right) + \left(\boldsymbol {m} _ {X} - \boldsymbol {A} ^ {0} \boldsymbol {m} _ {Z} - \boldsymbol {b} ^ {0}\right) ^ {\mathrm{T}} \left(\boldsymbol {m} _ {X} - \boldsymbol {A} ^ {0} \boldsymbol {m} _ {Z} - \boldsymbol {b} ^ {0}\right)
$$

利用关系式 4, 上式成

$$
\begin{array}{r l} J & = \mathrm{tr} \big [ (\mathbf {A} ^ {0} - \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1}) \mathbf {C} _ {Z} (\mathbf {A} ^ {0 \mathrm{T}} - \mathbf {C} _ {Z} ^ {- 1} \mathbf {C} _ {Z X}) \big ] + \mathrm{tr} (\mathbf {C} _ {X} - \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} \mathbf {C} _ {Z X}) + \\ & \quad (\mathbf {m} _ {X} - \mathbf {A} ^ {0} \mathbf {m} _ {Z} - \mathbf {b} ^ {0}) ^ {\mathrm{T}} (\mathbf {m} _ {X} - \mathbf {A} ^ {0} \mathbf {m} _ {Z} - \mathbf {b} ^ {0}) \end{array}
$$

上式三项中,第二项与 $A^{0}$ 和 $b^{0}$ 的选择无关。第三项为向量的内积,恒大于等于零,要使J达到最小,必须有

$$
\boldsymbol {m} _ {X} - \boldsymbol {A} ^ {0} \boldsymbol {m} _ {Z} - \boldsymbol {b} ^ {0} = \boldsymbol {0}\tag{2.1.62}
$$

又由关系式 3 知, 第一项也大于等于零, 当该项取最小值零时, 须有

$$
\mathbf {A} ^ {0} - \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} = \mathbf {0}\tag{2.1.63}
$$

由式 $(2.1.62)$ 和式 $(2.1.63)$ ，得

$$
\begin{array}{l} \mathbf {A} ^ {0} = \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} \\ \mathbf {b} ^ {0} = \mathbf {m} _ {X} - \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} \mathbf {m} _ {Z} \end{array}
$$

因此 X 在 Z 上的线性最小方差估计为

$$
\hat {\mathbf {X}} _ {L} (\mathbf {Z}) = \mathbf {m} _ {X} + \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} (\mathbf {Z} - \mathbf {m} _ {Z})\tag{2.1.64}
$$

下面分析线性最小方差估计的均方误差

$$
\begin{array}{r l} \mathbf {P} & = E \left\{\left[ \mathbf {X} - \hat {\mathbf {X}} _ {L} (\mathbf {Z}) \right] \left[ \mathbf {X} - \hat {\mathbf {X}} _ {L} (\mathbf {Z}) \right] ^ {\mathrm{T}} \right\} = \\ & E \left\{\left[ (\mathbf {X} - \mathbf {m} _ {X}) - \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} (\mathbf {Z} - \mathbf {m} _ {Z}) \right] \left[ (\mathbf {X} - \mathbf {m} _ {X}) - \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} (\mathbf {Z} - \mathbf {m} _ {Z}) \right] ^ {\mathrm{T}} \right\} = \\ & \mathbf {C} _ {X} + \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} \mathbf {C} _ {Z} \mathbf {C} _ {Z} ^ {- 1} \mathbf {C} _ {Z X} - \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} \mathbf {C} _ {Z X} - \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} \mathbf {C} _ {Z X} = \\ & \mathbf {C} _ {X} - \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} \mathbf {C} _ {Z X} \end{array}
$$

(2.1.65)

比较式(2.1.31)、式(2.1.32)、式(2.1.64)和式(2.1.65)，知式(2.1.31)与式(2.1.64)具有完全相同的形式，式(2.1.32)与式(2.1.65)也是这样。这说明：如果被估计量X和量测量Z都服从正态分布，则线性最小方差估计与最小方差估计是相同的，即按式(2.1.64)计算得的估计不但在所有的线性估计中精度最优，而且在所有的估计中精度也是最优的。

## 2. 线性最小方差估计的性质

性质1 无偏性

线性最小方差估计是 X 在 Z 上的无偏估计, 即

$$
E [ \mathbf {X} - \hat {\mathbf {X}} _ {l} (\mathbf {Z}) ] = \mathbf {0}
$$

或

$$
E _ {Z} [ \hat {\mathbf {X}} _ {L} (\mathbf {Z}) ] = E \mathbf {X}
$$

根据式 $(2.1.64)$ 对Z求均值即可证明此性质。

性质2 线性1

线性最小方差估计具有线性性质,即若 X 的线性最小方差估计为 $E^{*}[X/Z]$ , 则 $FX + e$ 的线性最小方差估计为

$$
E ^ {*} \left[ (F X + e) / Z \right] = F E ^ {*} \left[ X / Z \right] + e\tag{2.1.66}
$$

式中，F 为确定性矩阵，e 为确定性向量。

证明 令 $FX + e = K$ ，则

$$
\boldsymbol {m} _ {K} = \boldsymbol {F m} _ {X} + \boldsymbol {e}
$$

$$
\mathbf {C} _ {\mathrm{KZ}} = E \left[ (\mathbf {K} - \mathbf {m} _ {\mathrm{K}}) (\mathbf {Z} - \mathbf {m} _ {\mathrm{Z}}) ^ {\mathrm{T}} \right] = E \left[ (\mathbf {F X} + e - \mathbf {F m} _ {\mathrm{X}} - e) (\mathbf {Z} - \mathbf {m} _ {\mathrm{Z}}) ^ {\mathrm{T}} \right] = \mathbf {F C} _ {\mathrm{XZ}}
$$

根据式 $(2.1.64)$ ，得

$$
\begin{array}{r l} & E ^ {*} [ \mathbf {K} / \mathbf {Z} ] = m _ {K} + \mathbf {C} _ {K Z} \mathbf {C} _ {Z} ^ {- 1} (\mathbf {Z} - m _ {Z}) = \mathbf {F} m _ {X} + e + \mathbf {F C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} (\mathbf {Z} - m _ {Z}) = \\ & \qquad \qquad \qquad \qquad \qquad \mathbf {F} [ m _ {X} + \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} (\mathbf {Z} - m _ {Z}) ] + e = \mathbf {F E} ^ {*} [ \mathbf {X} / \mathbf {Z} ] + e \end{array}
$$

性质3 线性2

若 $Y$ 与 $Z$ 互不相关，则

$$
E ^ {*} [ \mathbf {X} / \mathbf {Y}, \mathbf {Z} ] = E ^ {*} [ \mathbf {X} / \mathbf {Y} ] + E ^ {*} [ \mathbf {X} / \mathbf {Z} ] - E \mathbf {X}\tag{2.1.67}
$$

证明 令 $\pmb{T} = \begin{bmatrix} \mathbf{Y}\\ \mathbf{Z} \end{bmatrix}$ ，则

$$
E ^ {*} [ \mathbf {X} / \mathbf {Y}, \mathbf {Z} ] = E ^ {*} [ \mathbf {X} / \mathbf {T} ] = E \mathbf {X} + \mathbf {C} _ {\mathbf {X T}} \mathbf {C} _ {T} ^ {- 1} (\mathbf {T} - E \mathbf {T})
$$

式中

$$
\boldsymbol {C} _ {X T} = E \left\{\left[ \boldsymbol {X} - \boldsymbol {m} _ {X} \right] \left[ \begin{array}{l} \boldsymbol {Y} - \boldsymbol {m} _ {Y} \\ \boldsymbol {Z} - \boldsymbol {m} _ {Z} \end{array} \right] ^ {\mathrm{T}} \right\} = \left[ \begin{array}{l l} \boldsymbol {C} _ {X Y} & \boldsymbol {C} _ {X Z} \end{array} \right]
$$

$$
\boldsymbol {C} _ {T} = E \left\{\left[ \begin{array}{l} \boldsymbol {Y} - \boldsymbol {m} _ {Y} \\ \boldsymbol {Z} - \boldsymbol {m} _ {Z} \end{array} \right] \left[ \begin{array}{l l} (\boldsymbol {Y} - \boldsymbol {m} _ {Y}) ^ {\mathrm{T}} & (\boldsymbol {Z} - \boldsymbol {m} _ {Z}) ^ {\mathrm{T}} \end{array} \right] \right\}
$$

由于 $Y$ 和 $Z$ 互不相关，所以

$$
\begin{array}{r l} & \mathbf {C} _ {T} = \left[ \begin{array}{l l} \mathbf {C} _ {Y} & \mathbf {0} \\ \mathbf {0} & \mathbf {C} _ {Z} \end{array} \right] \\ & E ^ {*} [ \mathbf {X} / \mathbf {Y}, \mathbf {Z} ] = E \mathbf {X} + [ \mathbf {C} _ {X Y} - \mathbf {C} _ {X Z} ] \left[ \begin{array}{l l} \mathbf {C} _ {Y} & \mathbf {0} \\ \mathbf {0} & \mathbf {C} _ {Z} \end{array} \right] ^ {- 1} \left[ \begin{array}{l} \mathbf {Y} - m _ {Y} \\ \mathbf {Z} - m _ {Z} \end{array} \right] = \\ & E \mathbf {X} + C _ {X Y} C _ {Y} ^ {- 1} (\mathbf {Y} - m _ {Y}) + E \mathbf {X} + C _ {X Z} C _ {Z} ^ {- 1} (\mathbf {Z} - m _ {Z}) - E \mathbf {X} = \\ & E ^ {*} [ \mathbf {X} / \mathbf {Y} ] + E ^ {*} [ \mathbf {X} / \mathbf {Z} ] - E \mathbf {X} \end{array}
$$

例2-8 设 $S(t)$ 为零均值标量平稳随机过程，相关函数 $R_{s}(\tau) = E[S(t)S(t + \tau)] =$

$\frac{1}{2}\mathrm{e}^{-|\tau|}$ ，试根据 $S(t)$ 用线性最小方差估计预测 $S(t + T)$ 。

解 根据题意, $S(t)$ 为量测,记为 $Z,S(t+T)$ 为被估计量,记为X。由于 $S(t)$ 为零均值的标量平稳随机过程,所以

$$
C _ {Z} = E [ S (t) S (t) ] = R _ {S} (\tau) \mid_ {\tau = 0} = \frac {1}{2}
$$

$$
C _ {X Z} = E [ S (t + T) S (t) ] = R _ {S} (\tau) \mid_ {\tau = T} = \frac {1}{2} \mathrm{e} ^ {- T}
$$

$$
m _ {X} = 0, m _ {Z} = 0
$$

根据式 $(2.1.64)$ ，得

$$
\hat {S} (t + T) = \frac {1}{2} \mathrm{e} ^ {- T} \times 2 \times S (t) = \mathrm{e} ^ {- T} S (t)
$$

例 2-9 设 $X(t)$ 为标量平稳随机过程, 均值为零, 相关函数为 $R(\tau)$ , 量测量 $Z(t_k) = X(t_k) + V(t_k), k = 1, 2, \cdots, V(t_k)$ 为零均值的白噪声序列, 方差为 $C_V, V(t_k)$ 与 $X(t_i) (i = 1, 2, \cdots, k)$ 不相关, 试用 $Z(t_{k-1})$ 和 $Z(t_k)$ 求线性最小方差估计 $\hat{X}(t_k)$ , 并与单独使用 $Z(t_k)$ 求解 $\hat{X}(t_k)$ 的精度作比较。

$$
\begin{array}{r l} & {\text {解} \quad \mathbf {Z} = \left[ \begin{array}{l} {Z (t _ {k - 1})} \\ {Z (t _ {k})} \end{array} \right] = \left[ \begin{array}{l} {X (t _ {k - 1}) + V (t _ {k - 1})} \\ {X (t _ {k}) + V (t _ {k})} \end{array} \right]} \\ & {\mathbf {C} _ {X Z} = E \{X (t _ {k}) [ X (t _ {k - 1}) + V (t _ {k - 1}) \quad X (t _ {k}) + V (t _ {k}) ] \} = [ R (T) \quad R (0) ]} \\ & {\qquad \mathbf {C} _ {Z} = E \Bigg \{\left[ \begin{array}{l l} {X (t _ {k - 1}) + V (t _ {k - 1})} \\ {X (t _ {k}) + V (t _ {k})} \end{array} \right] [ X (t _ {k - 1}) + V (t _ {k - 1}) \quad X (t _ {k}) + V (t _ {k}) ] \Bigg \} =} \\ & {\qquad \left[ \begin{array}{l l} {R (0) + C _ {V}} & {R (T)} \\ {R (T)} & {R (0) + C _ {V}} \end{array} \right]} \\ & {\qquad T = t _ {k} - t _ {k - 1}, \quad C _ {X} = E [ X ^ {2} (t _ {k}) ] = R (0)} \end{array}
$$

式中

根据式(2.1.64)， $X(t_{k})$ 的估计为

$$
\hat {X} (t _ {k}) = \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} \mathbf {Z} = \left[ \begin{array}{l l} R (T) & R (0) \end{array} \right] \left[ \begin{array}{c c} R (0) + C _ {V} & R (T) \\ R (T) & R (0) + C _ {V} \end{array} \right] ^ {- 1} \mathbf {Z}
$$

根据式 $(2.1.65)$ ，估计的均方误差为

$$
P = C _ {X} - \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} \mathbf {C} _ {Z X} = R (0) - [ R (T) \quad R (0) ] \left[ \begin{array}{c c} R (0) + C _ {V} & R (T) \\ R (T) & R (0) + C _ {V} \end{array} \right] ^ {- 1} \left[ \begin{array}{c} R (T) \\ R (0) \end{array} \right] =
$$

$$
R (0) - \frac {R ^ {2} (T) C _ {V} - R ^ {2} (T) R (0) + R ^ {2} (0) C _ {V} + R ^ {3} (0)}{\left[ R (0) + C _ {V} \right] ^ {2} - R ^ {2} (T)}
$$

若单独使用 $Z(t_{k})$ 来估计 $X(t_{k})$ ，根据式(2.1.65)得估计的均方误差为

$$
P ^ {\prime} = R (0) - \frac {R ^ {2} (0)}{R (0) + C _ {V}}
$$

所以

$$
P ^ {\prime} - P = \frac {R ^ {2} (T) C _ {V} ^ {2}}{[ R (0) + C _ {V} ] \{[ R (0) + C _ {V} ] ^ {2} - R ^ {2} (T) \}}
$$

因为 $R(0) > 0, C_V > 0, R(0) \geqslant R(T)$ ，所以只要 $R(T) \neq 0$ ，即 $X(t_k)$ 与 $X(t_{k-1})$ 之间有依从关系，则上式值总为正值，这说明同时使用 $Z(t_k)$ 和 $Z(t_{k-1})$ 比单独使用 $Z(t_k)$ 估计 $X(t_k)$ 的精度要高。因此尽量多地合理利用量测量能有效提高估计精度。

## 2.1.7 维纳滤波

## 1. 维纳滤波所要解决的问题

设有用信号 $S(t)$ 和干扰信号 $N(t)$ 都是随机信号，现要求从混有干扰的量测量 $Z(t)=S(t)+N(t)$ 中提取出有用信号 $S(t)$ 。由于 $S(t)$ 和 $N(t)$ 都是随机信号，没有确定的频谱，所以不能采用常规滤波的方法来滤除干扰。

设 $S(t)$ 和 $N(t)$ 都是平稳过程，且互相独立，自相关函数及功率谱密度 $R_{S}(\tau), R_{N}(\tau)$ ， $G_{S}(\omega), G_{N}(\omega)$ 都已知。

若无干扰,则滤波系统的输出即为理想输出:

$$
I (t) = \int_ {- \infty} ^ {\infty} h _ {l} (\tau) S (t - \tau) \mathrm{d} \tau\tag{2.1.68}
$$

式中 $h_I(\tau)$ 为理想滤波器的单位脉冲响应，其傅里叶变换 $H(\mathrm{j}\omega) = \mathcal{F}\{h_I(t)\}$ 就是理想滤波器的频率响应。若要求 $I(t) = S(t)$ ，即滤波的情况，则 $h_I(t) = \delta (t), H(\mathrm{j}\omega) = 1$ ；若要求 $I(t) = S(t + T)$ ，即预报的情况，则 $h_I(t) = \delta (t + T), H(\mathrm{j}\omega) = \mathrm{e}^{\mathrm{j}\omega T}$ 。

维纳滤波所要解决的问题是:求出一个物理可实现的线性定常系统的频率特性 $H(\mathrm{j}\omega)$ 或单位脉冲响应 $h(t)$ ，使滤波器的实际输出

$$
y (t) = \int_ {0} ^ {\infty} h (\tau) Z (t - \tau) \mathrm{d} \tau\tag{2.1.69}
$$

![](images/e9b110a10194e5b5d47075cea80eb0ff11d4b2723c83be9aea2a03479dff9fb8.jpg)

与理想输出 $I(t)$ 的误差 $\varepsilon(t)=I(t)-y(t)$ 的均方值达到最小，即

图 2.1.1 维纳滤波示意图

$$
E [ \varepsilon^ {2} (t) ] = E \{[ I (t) - y (t) ] ^ {2} \} = \min\tag{2.1.70}
$$

它可用图 2.1.1 来表示。

## 2. 维纳 - 霍甫方程

由于滤波器的实际输出为 $y(t)$ ，如式(2.1.69)所示，所以相对理想输出 $I(t)$ 的误差为

$$
\varepsilon (t) = I (t) - y (t) = I (t) - \int_ {0} ^ {\infty} h (\tau) Z (t - \tau) \mathrm{d} \tau
$$

平方误差为

$$
\varepsilon^ {2} (t) = I ^ {2} (t) - 2 \int_ {0} ^ {\infty} h (\tau) I (t) Z (t - \tau) \mathrm{d} \tau + \left[ \int_ {0} ^ {\infty} h (\tau) Z (t - \tau) \mathrm{d} \tau \right] \left[ \int_ {0} ^ {\infty} h (\tau_ {1}) Z (t - \tau_ {1}) \mathrm{d} \tau_ {1} \right]
$$

均方误差为

$$
\begin{array}{r l} E [ \varepsilon^ {2} (t) ] & = E [ I ^ {2} (t) ] - 2 \int_ {0} ^ {\infty} h (\tau) E [ I (t) Z (t - \tau) ] \mathrm{d} \tau + \\ & \quad \int_ {0} ^ {\infty} \int_ {0} ^ {\infty} h (\tau) h (\tau_ {1}) E [ Z (t - \tau) Z (t - \tau_ {1}) ] \mathrm{d} \tau \mathrm{d} \tau_ {1} \end{array}
$$

即

$$
\begin{array}{r l} E [ \varepsilon^ {2} (t) ] & = R _ {I} (0) - 2 \int_ {0} ^ {\infty} h (\tau) R _ {I Z} (\tau) \mathrm{d} \tau + \\ & \quad \int_ {0} ^ {\infty} \int_ {0} ^ {\infty} h (\tau) h (\tau_ {1}) R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau \mathrm{d} \tau_ {1} \end{array}\tag{2.1.71}
$$

显然 $E[\varepsilon^2 (t)]$ 取决于 $h(t)$ ，为便于说明问题，记 $\overline{\varepsilon^2(h)} = E[\varepsilon^2 (t)]$ 。下面分析要使

$E[\varepsilon^{2}(t)]=\min,h(t)$ 应满足什么条件。

设 $g(t)$ 是任意的一个可微函数，它和 $h(t)$ 一样也满足条件 $g(t)=0(t<0)$ ，这样 $h(t)+\alpha g(t)$ 也是物理上可实现的滤波系统的脉冲响应函数（其中 $\alpha$ 为实变数）。如果 $h(t)$ 使 $\overline{\varepsilon^{2}(h)}=\min$ ，则对任一实数 $\alpha$ 及任一函数 $g(t)$ ，总有 $\overline{\varepsilon^{2}(h+\alpha g)}\geqslant\overline{\varepsilon^{2}(h)}$ ，即函数 $J(\alpha)=\overline{\varepsilon^{2}(h+\alpha g)}$ 在 $\alpha=0$ 处取得极小值。

以 $h(t) + \alpha g(t)$ 代替式(2.1.71)中的 $h(t)$ ，得

$$
\begin{array}{r l} J (\alpha) & = R _ {I} (0) - 2 \int_ {0} ^ {\infty} [ h (\tau) + \alpha g (\tau) ] R _ {I Z} (\tau) \mathrm{d} \tau + \\ & \quad \int_ {0} ^ {\infty} \int_ {0} ^ {\infty} [ h (\tau) + \alpha g (\tau) ] [ h (\tau_ {1}) + \alpha g (\tau_ {1}) ] R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau \mathrm{d} \tau_ {1} \end{array}
$$

即

$$
\begin{array}{r l} J (\alpha) & = R _ {I} (0) - 2 \int_ {0} ^ {\infty} h (\tau) R _ {I Z} (\tau) \mathrm{d} \tau + \int_ {0} ^ {\infty} \int_ {0} ^ {\infty} h (\tau) h (\tau_ {1}) R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau \mathrm{d} \tau_ {1} - \\ & 2 \alpha \int_ {0} ^ {\infty} g (\tau) R _ {I Z} (\tau) \mathrm{d} \tau + 2 \alpha \int_ {0} ^ {\infty} \int_ {0} ^ {\infty} h (\tau_ {1}) g (\tau) R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau \mathrm{d} \tau_ {1} + \\ & \alpha^ {2} \int_ {0} ^ {\infty} \int_ {0} ^ {\infty} g (\tau) g (\tau_ {1}) R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau \mathrm{d} \tau_ {1} \end{array}
$$

记上式右侧前三项为 $J(0)$ ，则上式可写为

$$
\begin{array}{r l} J (\alpha) & = J (0) - 2 \alpha \int_ {0} ^ {\infty} g (\tau) [ R _ {I Z} (\tau) - \int_ {0} ^ {\infty} h (\tau_ {1}) R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau_ {1} ] \mathrm{d} \tau + \\ & \quad \alpha^ {2} \int_ {0} ^ {\infty} \int_ {0} ^ {\infty} g (\tau) g (\tau_ {1}) R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau \mathrm{d} \tau_ {1} \end{array}\tag{2.1.72}
$$

由于 $J(\alpha)$ 在 $\alpha = 0$ 处取得极小值的必要条件是

$$
\frac {\mathrm{d}}{\mathrm{d} \alpha} J (\alpha) \Bigg | _ {\alpha = 0} = 0
$$

所以有

$$
\frac {\mathrm{d}}{\mathrm{d} \alpha} J (\alpha) \Bigg | _ {\alpha = 0} = - 2 \int_ {0} ^ {\infty} g (\tau) \left[ R _ {I Z} (\tau) - \int_ {0} ^ {\infty} h (\tau_ {1}) R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau_ {1} \right] \mathrm{d} \tau = 0
$$

又由于 $g(t) = 0 (t < 0)$ ，且 $g(t)$ 为任意函数，因此要使上式成立，相当于对于一切 $\tau \geqslant 0$ ，方括号中的值恒为零。于是得 $\overline{\varepsilon^2(h)} = \min$ 的必要条件为

$$
R _ {I Z} (\tau) - \int_ {0} ^ {\infty} h (\tau_ {1}) R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau_ {1} = 0 \qquad \tau \geqslant 0\tag{2.1.73}
$$

事实上，式(2.1.73)也是 $\varepsilon^{2}(h)=\min$ 的充分条件，下面说明之。

设式 $(2.1.73)$ 成立，则式 $(2.1.72)$ 可写成

$$
\begin{array}{r l} J (\alpha) & = J (0) + \alpha^ {2} \int_ {0} ^ {\infty} \int_ {0} ^ {\infty} g (\tau) g (\tau_ {1}) R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau \mathrm{d} \tau_ {1} = \\ & J (0) + \alpha^ {2} E \left\{\int_ {0} ^ {\infty} g (\tau) Z (t - \tau) \mathrm{d} \tau \int_ {0} ^ {\infty} g (\tau_ {1}) Z (t - \tau_ {1}) \mathrm{d} \tau_ {1} \right\} = \\ & J (0) + \alpha^ {2} E \left\{\left[ \int_ {0} ^ {\infty} g (\tau) Z (t - \tau) \mathrm{d} \tau \right] ^ {2} \right\} \end{array}
$$

注意到上式的最后一项为非负项,所以有

$$
J (\alpha) \geqslant J (0)
$$

即

$$
\overline {{\varepsilon^ {2} (h + \alpha g)}} \geqslant \overline {{\varepsilon^ {2} (h)}}
$$

这说明,当滤波系统的单位脉冲响应 $h(t)$ 满足式(2.1.73)时,滤波系统的实际输出

$$
y (t) = \int_ {0} ^ {\infty} h (\tau) Z (t - \tau) \mathrm{d} \tau
$$

相对理想输出 $I(t)$ 的均方误差达到最小，因此式(2.1.73) 又是 $\varepsilon^{2}(h)=\min$ 的充分条件。

综上所述， $\varepsilon^2 (h) = \min$ 的充要条件为

$$
R _ {I Z} (\tau) - \int_ {0} ^ {\infty} h (\tau_ {1}) R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau_ {1} = 0 \qquad \tau \geqslant 0
$$

该方程称为维纳-霍甫方程, 是关于最佳单位脉冲响应 $h(t)$ 的积分方程, 其中 $R_{IZ}(\tau)$ 和 $R_{Z}(\tau)$ 按下述两式确定:

$$
\begin{array}{r l} & {R _ {I Z} (\tau) = E [ I (t) Z (t - \tau) ] =} \\ & {\qquad E \Big \{\left[ \int_ {- \infty} ^ {\infty} h _ {I} (\lambda) S (t - \lambda) \mathrm{d} \lambda \right] Z (t - \tau) \Big \} =} \\ & {\qquad \int_ {- \infty} ^ {\infty} h _ {I} (\lambda) E \{S (t - \lambda) [ S (t - \tau) + N (t - \tau) ] \} \mathrm{d} \lambda =} \\ & {\qquad \int_ {- \infty} ^ {\infty} h _ {I} (\lambda) [ R _ {S} (\tau - \lambda) + R _ {S N} (\tau - \lambda) ] \mathrm{d} \lambda =} \\ & {\qquad h _ {I} (\tau) * [ R _ {S} (\tau) + R _ {S N} (\tau) ]} \\ & {R _ {Z} (\tau) = E [ Z (t) Z (t - \tau) ] = E \{[ S (t) + N (t) ] [ S (t - \tau) + N (t - \tau) ] \} =} \\ & {\qquad R _ {S} (\tau) + R _ {N} (\tau) + R _ {S N} (\tau) + R _ {N S} (\tau)} \end{array}
$$

若对 $R_{IZ}(\tau)$ 和 $R_Z(\tau)$ 作傅里叶变换，则可得 $I(t)$ 与 $Z(t)$ 的互功率谱 $G_{IZ}(\omega)$ 和 $Z(t)$ 的自功率谱 $G_Z(\omega)$ ，即

$$
G _ {I Z} (\omega) = \mathcal {F} \{R _ {I Z} (\tau) \} = H _ {I} (\mathrm{j} \omega) [ G _ {S} (\omega) + G _ {S N} (\omega) ]\tag{2.1.74}
$$

$$
G _ {Z} (\omega) = \mathcal {F} \{R _ {Z} (\tau) \} = G _ {S} (\omega) + G _ {N} (\omega) + G _ {S N} (\omega) + G _ {N S} (\omega)\tag{2.1.75}
$$

式中， $H_{1}(j\omega)$ 为滤波器的理想频率响应，由滤波器的设计要求确定。

3. 维纳－霍甫方程的求解(设计维纳滤波器的一般方法)

式(2.1.73)中, $\tau\geqslant0$ 是滤波系统的物理可实现条件,正是这一限制给求解式(2.1.73)带来了一定的困难,下面说明这一点。

暂且假设没有 $\tau \geqslant 0$ 的限制，即当 $\tau < 0$ 时式(2.1.73)也成立，即

$$
R _ {I Z} (\tau) = \int_ {- \infty} ^ {\infty} h (\tau_ {1}) R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau_ {1} = h (\tau) * R _ {Z} (\tau) \quad (- \infty <   \tau <   \infty)\tag{2.1.76}
$$

对上式两边作傅里叶变换,得

$$
G _ {I Z} (\omega) = H (\mathrm{j} \omega) G _ {Z} (\omega)
$$

所以

$$
\begin{array}{r} H (\mathrm{j} \omega) = \frac {G _ {\mathrm{IZ}} (\omega)}{G _ {\mathrm{Z}} (\omega)} \\ G _ {\mathrm{IZ}} (\omega) = \mathcal {F} \{R _ {\mathrm{IZ}} (\tau) \}, G _ {\mathrm{Z}} (\omega) = \mathcal {F} \{R _ {\mathrm{Z}} (\tau) \} \end{array}\tag{2.1.77}
$$

式中

滤波系统的单位脉冲响应为

$$
h (t) = \mathcal {F} ^ {- 1} \left\{H (\mathrm{j} \omega) \right\} = \frac {1}{2 \pi} \int_ {- \infty} ^ {\infty} \frac {G _ {I Z} (\omega)}{G _ {Z} (\omega)} \mathrm{e} ^ {\mathrm{j} \omega t} \mathrm{d} \omega\tag{2.1.78}
$$

可是，从式(2.1.73)的推导过程知，只有当 $\tau \geqslant 0$ 时式(2.1.73)才成立，而当 $\tau < 0$ 时式(2.1.73)并不成立，所以对式(2.1.76)作傅里叶变换时，积分限只能取 $[0, \infty)$ ，而不能取 $(- \infty, \infty)$ ，即

$$
\begin{array}{r l} & {\int_ {0} ^ {\infty} R _ {I Z} (\tau) \mathrm{e} ^ {- \mathrm{j} \omega \tau} \mathrm{d} \tau = \int_ {0} ^ {\infty} \left[ \int_ {0} ^ {\infty} h (\tau_ {1}) R _ {Z} (\tau - \tau_ {1}) \mathrm{d} \tau_ {1} \right] \mathrm{e} ^ {- \mathrm{j} \omega \tau} \mathrm{d} \tau =} \\ & {\qquad \int_ {0} ^ {\infty} h (\tau_ {1}) \left[ \int_ {0} ^ {\infty} R _ {Z} (\tau - \tau_ {1}) \mathrm{e} ^ {- \mathrm{j} \omega \tau} \mathrm{d} \tau \right] \mathrm{d} \tau_ {1}} \end{array}
$$

上式方括号中是对 $\tau$ 积分， $\tau_{1}$ 可看做常量，令 $\tau - \tau_{1} = \lambda$ ，则 $d\tau = d\lambda, \lambda$ 的变化区间为 $[- \tau_{1}, \infty)$ ，因此有

$$
\int_ {0} ^ {\infty} R _ {I Z} (\tau) \mathrm{e} ^ {- \mathrm{j} \omega \tau} \mathrm{d} \tau = \int_ {0} ^ {\infty} h (\tau_ {1}) \left[ \int_ {- \tau_ {1}} ^ {\infty} R _ {Z} (\lambda) \mathrm{e} ^ {- \mathrm{j} \omega \lambda} \mathrm{d} \lambda \right] \mathrm{e} ^ {- \mathrm{j} \omega \tau_ {1}} \mathrm{d} \tau_ {1}
$$

由于上式方括号中的积分是 $\tau_{1}$ 的函数，上式无法写成谱密度的形式，因此不具备式(2.1.77)和式(2.1.78)确定的解。

式(2.1.73)可采用谱分解的方法求解。在讨论该法之前,先介绍谱分解中要使用的定理,即傅里叶变换的解析性定理。

定理 2.6 （解析性定理）

如果

$$
f (t) \left\{ \begin{array}{l l} = 0, t \geqslant 0 \\ \neq 0, t <   0 \end{array} \right.
$$

且其傅里叶变换 $F(\omega) = \mathcal{F}\{f(t)\}$ 存在，则 $F(\omega)$ 在 $[\omega]$ 的上半平面内解析有界。若 $F(\omega)$ 是 $\omega$ 的有理分式，则 $F(\omega)$ 仅在 $[\omega]$ 的下半平面内有极点，即极点 $\omega_{i} = u_{i} + \mathrm{j}v_{i}$ 中， $v_{i} < 0$ ；反之，若 $F(\omega)$ 在 $[\omega]$ 的上半平面内解析有界，则

$$
f (t) = \mathcal {F} ^ {- 1} \{F (\omega) \} \left\{ \begin{array}{l l} = 0, t \geqslant 0 \\ \neq 0, t <   0 \end{array} \right.
$$

类似地，如果 $f(t)\left\{ \begin{array}{l} \neq 0, (t \geqslant 0) \\ = 0, (t < 0) \end{array} \right.$ ，且 $F(\omega) = \mathcal{F}\{f(t)\}$ 存在，则 $F(\omega)$ 在 $[\omega]$ 的下半平面内解析有界。若 $F(\omega)$ 是 $\omega$ 的有理分式，则 $F(\omega)$ 仅在上半平面内有极点，即极点 $\omega_i = u_i + \mathrm{j}v_i$ 中， $v_i \geqslant 0$ 。反之，若 $F(\omega)$ 在 $[\omega]$ 的下半平面内解析有界，则

$$
f (t) = \mathcal {F} ^ {- 1} \{F (\omega) \} \left\{ \begin{array}{l l} \neq 0, t \geqslant 0 \\ = 0, t <   0 \end{array} \right.
$$

由于篇幅有限,定理2.6将不加证明地应用,详细证明请读者参阅文献[60]。

将式(2.1.73)写成

$$
f (t) = \int_ {0} ^ {\infty} h (\tau) R _ {Z} (t - \tau) \mathrm{d} \tau - R _ {I Z} (t)
$$

显然

$$
f (t) \left\{ \begin{array}{l l} = 0, t \geqslant 0 \\ \neq 0, t <   0 \end{array} \right.
$$

根据定理 2.6, $F(\omega) = \mathcal{F}\{f(t)\}$ 在 $[\omega]$ 的上半平面内解析有界。而

$$
\begin{array}{r l} F (\omega) = \mathcal {F} \{f (t) \} & = \int_ {- \infty} ^ {\infty} \left[ \int_ {0} ^ {\infty} h (\tau) R _ {Z} (t - \tau) \mathrm{d} \tau - R _ {I Z} (t) \right] \mathrm{e} ^ {- \mathrm{j} \omega t} \mathrm{d} t = \\ & \quad \int_ {0} ^ {\infty} h (\tau) \left[ \int_ {- \infty} ^ {\infty} R _ {Z} (t - \tau) \mathrm{e} ^ {- \mathrm{j} \omega t} \mathrm{d} t \right] \mathrm{d} \tau - \int_ {- \infty} ^ {\infty} R _ {I Z} (t) \mathrm{e} ^ {- \mathrm{j} \omega t} \mathrm{d} t \end{array}
$$

在上式的第一项积分中，令 $t - \tau = \lambda$ 。由于 $t: -\infty \to \infty, \tau: 0 \to \infty$ ，所以 $\lambda: -\infty \to \infty$ 。并且由于方括号中对 $t$ 积分时， $\tau$ 为常量，所以 $\mathrm{d}t = \mathrm{d}\lambda$ 。上式又可写成

$$
\begin{array}{r l} F (\omega) & = \int_ {0} ^ {\infty} h (\tau) \left[ \int_ {- \infty} ^ {\infty} R _ {Z} (\lambda) \mathrm{e} ^ {- \mathrm{j} \omega (\lambda + \tau)} \mathrm{d} \lambda \right] \mathrm{d} \tau - G _ {I Z} (\omega) = \\ & \quad \int_ {0} ^ {\infty} h (\tau) \mathrm{e} ^ {- \mathrm{j} \omega \tau} \mathrm{d} \tau \int_ {- \infty} ^ {\infty} R _ {Z} (\lambda) \mathrm{e} ^ {- \mathrm{j} \omega \lambda} \mathrm{d} \lambda - G _ {I Z} (\omega) = \\ & H (\mathrm{j} \omega) G _ {Z} (\omega) - G _ {I Z} (\omega) \end{array}\tag{2.1.79}
$$

为了解出 $H(j\omega)$ ，考虑 $G_{Z}(\omega)$ 可分解成下述形式的特殊情况：

$$
G _ {Z} (\omega) = \Psi_ {\mathrm{下解}} (\omega) \Psi_ {\mathrm{上解}} (\omega)\tag{2.1.80}
$$

式中， $\Psi_{\text{下解}}(\omega)$ 在 $[\omega]$ 的下半平面内解析有界，且没有零极点； $\Psi_{\text{上解}}(\omega)$ 在 $[\omega]$ 的上半平面内解析有界，且没有零极点。

式(2.1.80)在实际系统中一般都能满足,特别是当 $Z(t)$ 为零均值且方差有限的平稳随机过程时, $G_{Z}(\omega)$ 为 $\omega$ 的有理分式, $\Psi_{\text{下解}}(\omega)$ 和 $\Psi_{\text{上解}}(\omega)$ 互为共轭。

式(2.1.79)两边除以 $\Psi_{\text{上解}}(\omega)$ ，得

$$
\frac {F (\omega)}{\Psi_ {\mathrm{上解}} (\omega)} = H (\mathrm{j} \omega) \Psi_ {\mathrm{下解}} (\omega) - \frac {G _ {I Z} (\omega)}{\Psi_ {\mathrm{上解}} (\omega)}\tag{2.1.81}
$$

显然 $\frac{F(\omega)}{\Psi_{\text{上解}}(\omega)}$ 在 $[\omega]$ 的上半平面内解析有界，根据定理2.6，有

$$
\mathcal {F} ^ {- 1} \left\{\frac {F (\omega)}{\Psi_ {\mathrm{上解}} (\omega)} \right\} = \alpha (t) \left\{ \begin{array}{l l} = 0, t \geqslant 0 \\ \neq 0, t <   0 \end{array} \right.
$$

所以当 $t \geqslant 0$ 时

(2.1.82)

因为

$$
\begin{array}{r l} & {\mathcal {F} ^ {- 1} \left\{H (\mathrm{j} \omega) \Psi_ {\text {下解}} (\omega) \right\} = \mathcal {F} ^ {- 1} \left\{\frac {G _ {\mathrm{IZ}} (\omega)}{\Psi_ {\text {上解}} (\omega)} \right\}} \\ & {\qquad h (t) \left\{ \begin{array}{l l} {\neq 0, t \geqslant 0} \\ {= 0, t <   0} \end{array} \right.} \\ & {\qquad H (\mathrm{j} \omega) = \mathcal {F} \{h (t) \}} \end{array}
$$

所以

在 $[\omega]$ 的下半平面内解析有界。由此， $H(\mathrm{j}\omega)\Psi_{\text{下解}}(\omega)$ 在下半平面内解析有界，根据定理2.6

$$
\mathcal {F} ^ {- 1} \left\{H (\mathrm{j} \omega) \Psi_ {\text {下解}} (\omega) \right\} = \beta (t) \left\{ \begin{array}{l l} \neq 0, t \geqslant 0 \\ = 0, t <   0 \end{array} \right.
$$

将式 $(2.1.82)$ 代入上式,得

$$
\beta (t) = \left\{ \begin{array}{l l} \mathcal {F} ^ {- 1} \left\{\frac {G _ {I Z} (\omega)}{\Psi_ {\mathrm{上解}} (\omega)} \right\} = \frac {1}{2 \pi} \int_ {- \infty} ^ {\infty} \frac {G _ {I Z} (\omega)}{\Psi_ {\mathrm{上解}} (\omega)} \mathrm{e} ^ {\mathrm{j} \omega t} \mathrm{d} \omega , t \geqslant 0 \\ 0, t <   0 \end{array} \right.\tag{2.1.83}
$$

所以 $\beta (t)$ 的傅里叶变换为

$$
\begin{array}{r l} & {H (\mathrm{j} \omega) \Psi_ {\mathrm{下解}} (\omega) = \int_ {- \infty} ^ {\infty} \beta (t) \mathrm{e} ^ {- \mathrm{j} \omega t} \mathrm{d} t = \int_ {0} ^ {\infty} \beta (t) \mathrm{e} ^ {- \mathrm{j} \omega t} \mathrm{d} t =} \\ & {\qquad \int_ {0} ^ {\infty} \left[ \frac {1}{2 \pi} \int_ {- \infty} ^ {\infty} \frac {G _ {I Z} (\omega)}{\Psi_ {\mathrm{上解}} (\omega)} \mathrm{e} ^ {\mathrm{j} \omega t} \mathrm{d} \omega \right] \mathrm{e} ^ {- \mathrm{j} \omega t} \mathrm{d} t} \\ & {\qquad H (\mathrm{j} \omega) = \frac {1}{2 \pi \Psi_ {\mathrm{下解}} (\omega)} \int_ {0} ^ {\infty} \left[ \int_ {- \infty} ^ {\infty} \frac {G _ {I Z} (\omega)}{\Psi_ {\mathrm{上解}} (\omega)} \mathrm{e} ^ {\mathrm{j} \omega t} \mathrm{d} \omega \right] \mathrm{e} ^ {- \mathrm{j} \omega t} \mathrm{d} t} \end{array}\tag{2.1.84}
$$

式(2.1.84)就是式(2.1.73)的解,即维纳滤波器频率特性的求解公式,从表达式可看出

运算是十分繁杂的,这也是维纳滤波器设计过程中的困难所在。

综上所述,可归纳出维纳滤波器的一般设计步骤:

(1) 按式(2.1.74) 和式(2.1.75) 计算 $G_{IZ}(\omega)$ 和 $G_{Z}(\omega)$ ;

(2) 按式(2.1.80) 分解 $G_{Z}(\omega)$ ;

(3) 按式(2.1.83)求解 $\beta(t)$ ;

(4) 求出 $\beta(t)$ 的傅里叶变换

$$
B (\mathrm{j} \omega) = \int_ {0} ^ {\infty} \beta (t) \mathrm{e} ^ {- \mathrm{j} \omega t} \mathrm{d} t
$$

(5) 维纳滤波器的频率特性为

$$
H (\mathrm{j} \omega) = \frac {B (\mathrm{j} \omega)}{\Psi_ {\mathrm{下解}} (\omega)}
$$

例2-10 设有用信号 $S(t)$ 的功率谱为 $G_{\mathrm{S}}(\omega) = \frac{1}{1 + \omega^2}$ ，干扰 $N(t)$ 为白噪声过程，功率谱为 $G_{\mathrm{N}}(\omega) = \sigma^2$ ，有用信号和干扰互不相关。要求设计维纳滤波器，该滤波器能抑制干扰 $N(t)$ ，并能对信号 $S$ 作超前 $T$ 的预测，即 $t$ 时刻能预测出 $t + T$ 时刻的值。

解

$$
\begin{array}{r l} & {(1) G _ {Z} (\omega) = G _ {S} (\omega) + G _ {N} (\omega) = \frac {1 + \sigma^ {2} + \sigma^ {2} \omega^ {2}}{1 + \omega^ {2}}} \\ & {\quad H _ {I} (\mathrm{j} \omega) = \mathrm{e} ^ {\mathrm{j} \omega T}} \\ & {\quad G _ {I Z} (\omega) = \frac {\mathrm{e} ^ {\mathrm{j} \omega T}}{1 + \omega^ {2}}} \\ & {(2) G _ {Z} (\omega) = \frac {1 + \sigma^ {2} + \sigma^ {2} \omega^ {2}}{1 + \omega^ {2}} = \frac {\sqrt {1 + \sigma^ {2}} + \mathrm{j} \sigma \omega}{1 + \mathrm{j} \omega} \frac {\sqrt {1 + \sigma^ {2}} - \mathrm{j} \sigma \omega}{1 - \mathrm{j} \omega}} \\ & {\quad \Psi_ {\text {下解}} (\omega) = \frac {\sqrt {1 + \sigma^ {2}} + \mathrm{j} \sigma \omega}{1 + \mathrm{j} \omega}, \quad \Psi_ {\text {上解}} (\omega) = \frac {\sqrt {1 + \sigma^ {2}} - \mathrm{j} \sigma \omega}{1 - \mathrm{j} \omega}} \end{array}
$$

(3) $t \geqslant 0$ 时，

$$
\begin{array}{r l} \beta (t) & = \frac {1}{2 \pi} \int_ {- \infty} ^ {\infty} \frac {\mathrm{e} ^ {\mathrm{j} \omega T}}{(1 + \mathrm{j} \omega) (1 - \mathrm{j} \omega)} \frac {1 - \mathrm{j} \omega}{\sqrt {1 + \sigma^ {2}} - \mathrm{j} \sigma \omega} \mathrm{e} ^ {\mathrm{j} \omega t} \mathrm{d} \omega = \\ & \frac {1}{2 \pi} \int_ {- \infty} ^ {\infty} \frac {1}{(1 + \mathrm{j} \omega) (\sqrt {1 + \sigma^ {2}} - \mathrm{j} \sigma \omega)} \mathrm{e} ^ {\mathrm{j} \omega (t + T)} \mathrm{d} \omega = \\ & \frac {1}{\sigma + \sqrt {1 + \sigma^ {2}}} \frac {1}{2 \pi} \int_ {- \infty} ^ {\infty} \left(\frac {1}{1 + \mathrm{j} \omega} + \frac {\sigma}{\sqrt {1 + \sigma^ {2}} - \mathrm{j} \sigma \omega}\right) \mathrm{e} ^ {\mathrm{j} \omega (t + T)} \mathrm{d} \omega = \\ & \frac {\mathrm{e} ^ {- (t + T)}}{\sigma + \sqrt {1 + \sigma^ {2}}} \end{array}
$$

$t < 0$ 时， $\beta(t) = 0$ 。

$$
B (\mathrm{j} \omega) = \int_ {0} ^ {\infty} \beta (t) \mathrm{e} ^ {- \mathrm{j} \omega t} \mathrm{d} t = \frac {\mathrm{e} ^ {- T}}{\sigma + \sqrt {1 + \sigma^ {2}}} \int_ {0} ^ {\infty} \mathrm{e} ^ {- (1 + \mathrm{j} \omega) t} \mathrm{d} t = \frac {\mathrm{e} ^ {- T}}{\sigma + \sqrt {1 + \sigma^ {2}}} \frac {1}{1 + \mathrm{j} \omega}
$$

(5) 维纳滤波器的频率特性为

$$
H (\mathrm{j} \omega) = \frac {B (\mathrm{j} \omega)}{\Psi_ {\mathrm{下解}} (\omega)} = \frac {\mathrm{e} ^ {- \mathrm{T}}}{\sigma + \sqrt {1 + \sigma^ {2}}} \frac {1}{1 + \mathrm{j} \omega} \frac {1 + \mathrm{j} \omega}{\sqrt {1 + \sigma^ {2}} + \mathrm{j} \sigma \omega} =
$$

$$
\frac {\mathrm{e} ^ {- \mathrm{T}}}{\sigma + \sqrt {1 + \sigma^ {2}}} \frac {1}{\sqrt {1 + \sigma^ {2}} + \mathrm{j} \sigma \omega}
$$

4. 维纳滤波器的伯特－香农(Bode-Shannon)设计法

在式(2.1.84)中,如果 $\frac{G_{IZ}(\omega)}{\Psi_{\text{上解}}(\omega)}$ 可分解为

$$
\frac {G _ {I Z} (\omega)}{\Psi_ {\mathrm{上解}} (\omega)} = \left[ \frac {G _ {I Z} (\omega)}{\Psi_ {\mathrm{上解}} (\omega)} \right] _ {\mathrm{上极}} + \left[ \frac {G _ {I Z} (\omega)}{\Psi_ {\mathrm{上解}} (\omega)} \right] _ {\mathrm{下极}}\tag{2.1.85}
$$

式中， $[\cdot]_{上极}$ 表示只在 $[\omega]$ 的上半平面内有极点； $[\cdot]_{下极}$ 表示只在 $[\omega]$ 的下半平面内有极点。则， $[\cdot]_{上极}$ 在 $[\omega]$ 的下半平面内解析有界； $[\cdot]_{下极}$ 在 $[\omega]$ 的上半平面内解析有界。

$$
\frac {1}{2 \pi} \int_ {- \infty} ^ {\infty} \left[ \frac {G _ {I Z} (\omega)}{\varPsi_ {\mathrm{上解}} (\omega)} \right] _ {\mathrm{上极}} \mathrm{e} ^ {\mathrm{j} \omega t} \mathrm{d} \omega = \xi (t) \left\{ \begin{array}{l l} {{\neq 0, t \geqslant 0}} \\ {{= 0, t <   0}} \end{array} \right.\tag{2.1.86}
$$

$$
\frac {1}{2 \pi} \int_ {- \infty} ^ {\infty} \left[ \frac {G _ {I Z} (\omega)}{\Psi_ {\mathrm{上解}} (\omega)} \right] _ {\mathrm{下极}} \mathrm{e} ^ {\mathrm{j} \omega t} \mathrm{d} \omega = \zeta (t) \left\{ \begin{array}{l l} = 0, t \geqslant 0 \\ \neq 0, t <   0 \end{array} \right.\tag{2.1.87}
$$

因此式(2.1.84)为

$$
\begin{array}{r l} H (\mathrm{j} \omega) & = \frac {1}{\Psi_ {\text {下解}} (\omega)} \int_ {0} ^ {\infty} [ \xi (t) + \zeta (t) ] \mathrm{e} ^ {- \mathrm{j} \omega t} \mathrm{d} t = \frac {1}{\Psi_ {\text {下解}} (\omega)} \int_ {- \infty} ^ {\infty} \xi (t) \mathrm{e} ^ {- \mathrm{j} \omega t} \mathrm{d} t = \\ & \quad \frac {1}{\Psi_ {\text {下解}} (\omega)} \bigg [ \frac {G _ {I Z} (\omega)}{\Psi_ {\text {上解}} (\omega)} \bigg ] _ {\text {上极}} \end{array}\tag{2.1.88}
$$

式(2.1.88)说明,如果 $\frac{G_{IZ}(\omega)}{\Psi_{\text{上解}}(\omega)}$ 是有理分式,即它可分解成仅在上半平面内有极点和仅在下半平面有极点的两部分,则维纳滤波器的频率特性求取方法可以简化。下面具体说明之。

设有用信号 $S(t)$ 和干扰 $N(t)$ 均为平稳随机过程，且互相独立，则

$$
\begin{array}{r l} R _ {Z} (\tau) & = E [ Z (t + \tau) Z (t) ] = E \{[ S (t + \tau) + N (t + \tau) ] [ S (t) + N (t) ] \} = \\ & R _ {S} (\tau) + R _ {N} (\tau) \end{array}\tag{2.1.89}
$$

所以

$$
\begin{array}{r l} & G _ {Z} (\omega) = G _ {S} (\omega) + G _ {N} (\omega) \\ & R _ {I Z} (\tau) = E [ I (t) Z (t - \tau) ] = E \left[ \int_ {- \infty} ^ {\infty} h _ {I} (\lambda) S (t - \lambda) \mathrm{d} \lambda Z (t - \tau) \right] = \\ & \int_ {- \infty} ^ {\infty} h _ {I} (\lambda) \{E [ S (t - \lambda) S (t - \tau) + S (t - \lambda) N (t - \lambda) ] \} \mathrm{d} \lambda = \\ & \int_ {- \infty} ^ {\infty} h _ {I} (\lambda) R _ {S} (\tau - \lambda) \mathrm{d} \lambda = h _ {I} (\tau) * R _ {S} (\tau) \end{array}\tag{2.1.90}
$$

所以

$$
G _ {I Z} (\omega) = H _ {l} (\mathrm{j} \omega) G _ {S} (\omega)
$$

对于滤波问题, $H_{I}(j\omega)=1$

$$
\begin{array}{r} {G _ {I Z} (\omega) = G _ {S} (\omega)} \\ {\Psi_ {\mathrm{上解}} (\omega) \Psi_ {\mathrm{下解}} (\omega) = G _ {Z} (\omega)} \end{array}\tag{2.1.91}
$$

又由于

所以

$$
\Psi_ {\mathrm{上解}} (\omega) = \frac {G _ {Z} (\omega)}{\Psi_ {\mathrm{下解}} (\omega)} = \frac {G _ {S} (\omega) + G _ {N} (\omega)}{\Psi_ {\mathrm{下解}} (\omega)}\tag{2.1.92}
$$

将式(2.1.91)、式(2.1.92)代入式(2.1.88)，得

$$
\begin{array}{r l} H (\mathrm{j} \omega) = & \frac {1}{\Psi_ {\mathrm{下解}} (\omega)} \left[ \frac {G _ {S} (\omega)}{\frac {G _ {S} (\omega) + G _ {N} (\omega)}{\Psi_ {\mathrm{下解}} (\omega)}} \right] _ {\mathrm{上极}} = \\ & \frac {1}{\Psi_ {\mathrm{下解}} (\omega)} \left[ \frac {G _ {S} (\omega)}{G _ {S} (\omega) + G _ {N} (\omega)} \cdot \Psi_ {\mathrm{下解}} (\omega) \right] _ {\mathrm{上极}} \end{array}\tag{2.1.93}
$$

如果能将式(2.1.93)的右边写成关于 $j\omega$ 的表达式，则仅需将 $j\omega$ 换成拉普拉斯算子p即可获得滤波器的传递函数。但 $[\cdot]_{上极}$ 中仅在 $[\omega]$ 的上半平面内有极点的条件又转化成怎样的条件呢？

设

$$
\begin{array}{r l} & {\Psi_ {\mathrm{下解}} (\omega) = H _ {1} (\mathrm{j} \omega)} \\ & {\frac {G _ {S} (\omega)}{G _ {S} (\omega) + G _ {N} (\omega)} = H _ {0} (\mathrm{j} \omega)} \\ & {H _ {1} (\mathrm{j} \omega) H _ {0} (\mathrm{j} \omega) = H _ {2} (\mathrm{j} \omega)} \end{array}
$$

$\left[\frac{G_S(\omega)}{G_S(\omega) + G_N(\omega)} \cdot \Psi_{\text{下解}}(\omega)\right]_{\text{上极}}$ 的任一极点为 $\omega_i = u_i + \mathrm{j}v_i$ 。

由于 $\omega_{i}$ 仅位于 $[\omega]$ 的上半平面内，即 $v_{i} > 0$ ，当以 $p$ 代替 $\mathrm{j}\omega$ 时，[ω]平面内的极点 $\omega_{i} = u_{i} + \mathrm{j}v_{i}$ 就变成[p]平面内的极点 $p_i = \mathrm{j}\omega_i = \mathrm{j}(u_i + \mathrm{j}v_i) = -v_i + \mathrm{j}u_i$ ，而 $-v_{i} < 0$ ，所以位于[ω]上半平面内的极点被变换成位于[p]左半平面内的极点。这样式(2.1.93)改写为

$$
H (p) = \frac {1}{H _ {1} (p)} [ H _ {0} (p) \bullet H _ {1} (p) ] _ {\mathrm{左极}}\tag{2.1.94}
$$

式中 $[\cdot]_{\text{左极}}$ 表示在 $H_0(p)H_1(p)$ 的部分分式展开式中只取极点位于 $[p]$ 平面左侧的部分。

式(2.1.94)确定的 $H(p)$ 就是维纳滤波器的传递函数, 此设计方法称为伯特-香农设计法。

根据上述分析,可归纳出采用伯特-香农法设计维纳滤波器的步骤如下:

(1) 由 $G_{S}(\omega) + G_{N}(\omega) = |H_{1}(j\omega)|^{2}$ 确定出 $H_{1}(j\omega)$ 和 $H_{1}(p)$ ;

(2) 计算 $H_{2}(j\omega)=H_{0}(j\omega)H_{1}(j\omega)$ ，其中

$$
H _ {0} (\mathrm{j} \omega) = \frac {G _ {S} (\omega)}{G _ {S} (\omega) + G _ {N} (\omega)}
$$

根据 $H_{2}(j\omega)$ 确定出 $H_{2}(p)$ ;

(3) 将 $H_{2}(p)$ 分解成部分分式和；

（4）取 $H_{2}(p)$ 部分分式和式中极点仅位于左半平面内（包括虚轴）的所有项的和并记为 $H_{3}(p)$ ;

(5) 维纳滤波器的传递函数为

$$
H (p) = H _ {3} (p) H _ {1} ^ {- 1} (p)
$$

请读者注意,伯特-香农设计法的适用范围仅限于:

（1）有用信号 $S(t)$ 和干扰信号 $N(t)$ 均为零均值的平稳随机过程， $S(t)$ 和 $N(t)$ 互相独立，它们的功率谱均为有理分式；

(2) 适用于干扰的滤除,而不能解决预报和平滑问题。

例2-11 设计具有滤波功能的维纳滤波器，所处理的信号和干扰同例2-10。

解 由于 $G_{S} = \frac{1}{1 + \omega^{2}}, G_{N}(\omega) = \sigma^{2}$ , 有用信号和干扰的功率谱都是有理谱, 且要求滤波器

具有滤波功能,所以可采用伯特-香农法设计维纳滤波器。

$$
(1) G _ {S} (\omega) + G _ {N} (\omega) = \frac {1}{1 + \omega^ {2}} + \sigma^ {2} =
$$

$$
\frac {\sqrt {1 + \sigma^ {2}} + \mathrm{j} \sigma \omega}{1 + \mathrm{j} \omega} \frac {\sqrt {1 + \sigma^ {2}} - \mathrm{j} \sigma \omega}{1 - \mathrm{j} \omega} = H _ {1} (\mathrm{j} \omega) H _ {1} ^ {*} (\mathrm{j} \omega)
$$

$$
H _ {1} (\mathrm{j} \omega) = \frac {\sqrt {1 + \sigma^ {2}} + \mathrm{j} \sigma \omega}{1 + \mathrm{j} \omega}
$$

$$
H _ {1} (p) = \frac {\sqrt {1 + \sigma^ {2}} + \sigma p}{1 + p}
$$

$$
(2) H _ {0} (\mathrm{j} \omega) = \frac {\frac {1}{1 + \omega^ {2}}}{\frac {1}{1 + \omega^ {2}} + \sigma^ {2}} = \frac {1}{1 + \sigma^ {2} + \sigma^ {2} \omega^ {2}}
$$

$$
H _ {2} (\mathrm{j} \omega) = \frac {1}{1 + \sigma^ {2} + \sigma^ {2} \omega^ {2}} \frac {\sqrt {1 + \sigma^ {2}} + \mathrm{j} \sigma \omega}{1 + \mathrm{j} \omega} = \frac {1}{\sqrt {1 + \sigma^ {2}} - \mathrm{j} \sigma \omega} \frac {1}{1 + \mathrm{j} \omega}
$$

$$
H _ {2} (p) = \frac {1}{\sqrt {1 + \sigma^ {2}} - \sigma p} \frac {1}{1 + p}
$$

$$
H _ {2} (p) = \frac {1}{\sqrt {1 + \sigma^ {2}} + \sigma} \left(\frac {\sigma}{\sqrt {1 + \sigma^ {2}} - \sigma p} + \frac {1}{1 + p}\right) \tag {3}
$$

$$
H _ {3} (p) = \frac {1}{\sqrt {1 + \sigma^ {2}} + \sigma} \frac {1}{1 + p}
$$

(5) 维纳滤波器的传递函数为

$$
H (p) = \frac {1}{\sqrt {1 + \sigma^ {2}} + \sigma} \frac {1}{1 + p} \frac {1 + p}{\sqrt {1 + \sigma^ {2}} + \sigma p} = \frac {1}{\sqrt {1 + \sigma^ {2}} + \sigma} \frac {1}{\sqrt {1 + \sigma^ {2}} + \sigma p}
$$

## 2.1.8 几种最优估计的优缺点比较

由于各种估计满足的最优指标不一样,利用的信息不一样,所以适用的对象、达到的精度和计算的复杂性各不一样。

最小二乘估计法适用于对常值向量或随机向量的估计。由于使用的最优指标是使量测估计的精度达到最佳，估计中不必使用与被估计量有关的动态信息与统计信息，甚至连量测误差的统计信息也可不必使用，所以估计精度不高。这种方法的最大优点是算法简单，在对被估计量和量测误差缺乏了解的情况下仍能适用，所以至今仍被大量采用。

最小方差估计是所有估计中估计的均方误差为最小的估计，是所有估计中的最佳者。但这种最优估计只确定出了估计值是被估计量在量测空间上的条件均值这一抽象关系。一般情况下条件均值须通过条件概率密度求取，而条件概率密度的获取本身就非易事，所以按条件均值的一般求法求取最小方差估计是很困难的。

线性最小方差估计是所有线性估计中的最优者，只有当被估计量和量测量都服从正态分布时，线性最小方差估计才与最小方差估计等同，即在所有估计中也是最优的。线性最小方差估计可适用于随机过程的估计，估计过程中只须知道被估计量和量测量的一阶和二阶矩。对于平稳过程，这些一阶和二阶矩都为常值，但对非平稳过程，一阶和二阶矩随时间而变，必须确切知道每一估计时刻的一、二阶矩才能求出估计值，这种要求是十分苛刻的。所以线性最小方差估计适用于平稳过程而难以适用非平稳过程。由例2-9知，估计过程中不同时刻的量测量使用得越多，估计精度就越高，但矩阵求逆的阶数也越高，计算量也越大。

极大验后估计、贝叶斯估计、极大似然估计都与条件概率密度有关，除一些特殊的分布外，如正态分布情况，计算都十分困难。这些估计常用于故障检测和识别的算法中。

维纳滤波是线性最小方差估计的一种。维纳滤波器是一种线性定常系统，适用于对有用信号和干扰信号都是零均值的平稳随机过程的处理。设计维纳滤波器时必须知道有用信号和干扰信号的自功率谱和互功率谱。当功率谱都是有理分式时，可采用伯特-香农设计法求取具有滤波功能的维纳滤波器的传递函数。对于复杂的有用信号和干扰信号，功率谱并非有理谱，此时可将功率谱拟合成有理谱后按伯特-香农法进行设计。与卡尔曼滤波相比，维纳滤波在适用范围、设计方法等方面存在着诸多不足，但对于被估计参量较少的情况，如直升飞机悬停时仅须对高度作估计，结合使用数字滤波技术，维纳滤波仍不失为一种简单而有效的方法[61]。

## 2.2 离散型卡尔曼滤波

2.1节介绍的线性最小方差估计具有一定的实用性，不同时刻的量测信息利用得越多，估计的精度就越高。但这种算法使用了被估计量与量测量的一、二阶矩，对于非平稳过程，必须确切知道这种一、二阶矩的变化规律，这种要求是十分苛刻的，一般是无法办到的。此外，算法中采用对不同时刻的量测值作集中处理的办法，这使计算随着估计过程的推移而逐渐加重。所以线性最小方差估计并不是一种实用的估计算法，特别是对非平稳过程的处理更是困难重重。那么，能否找到一种实用的线性最小方差估计算法，适用于非平稳过程，并与递推最小二乘估计一样，算法采用递推，从量测信息中实时提取出被估计量信息并积存在估计值中呢？答案是肯定的，这种新型算法就是卡尔曼滤波。

1960年由卡尔曼(R.E.Kalman)首次提出的卡尔曼滤波[8]是一种线性最小方差估计。相对2.1节介绍的几种最优估计，卡尔曼滤波具有如下特点：

（1）算法是递推的，且使用状态空间法在时域内设计滤波器，所以卡尔曼滤波适用于对多维随机过程的估计。

（2）采用动力学方程即状态方程描述被估计量的动态变化规律，被估计量的动态统计信息由激励白噪声的统计信息和动力学方程确定。由于激励白噪声是平稳过程，动力学方程已知，所以被估计量既可以是平稳的，也可以是非平稳的，即卡尔曼滤波也适用于非平稳过程。

（3）卡尔曼滤波具有连续型和离散型两类算法，离散型算法可直接在数字计算机上实现。

正由于上述特点，卡尔曼滤波理论一经提出立即受到了工程应用的重视，阿波罗登月飞船和C-5A飞机导航系统的设计是早期应用中的最成功者[13-14]。目前，卡尔曼滤波理论作为一种最重要的最优估计理论被广泛应用于各种领域，组合导航系统的设计是其成功应用中的一个最主要方面。

## 2.2.1 离散型卡尔曼滤波基本方程

设 $t_k$ 时刻的被估计状态 $\mathbf{X}_k$ 受系统噪声序列 $\pmb{W}_{k - 1}$ 驱动，驱动机理由下述状态方程描述

$$
\boldsymbol {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {X} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1}\tag{2.2.1}
$$

对 $X_{k}$ 的量测满足线性关系,量测方程为

$$
\mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k}\tag{2.2.2}
$$

式中， $\Phi_{k,k-1}$ 为 $t_{k-1}$ 时刻至 $t_{k}$ 时刻的一步转移阵； $\Gamma_{k-1}$ 为系统噪声驱动阵； $H_{k}$ 为量测阵； $V_{k}$ 为量测噪声序列； $W_{k}$ 为系统激励噪声序列。

同时， $W_{k}$ 和 $V_{k}$ 满足

$$
\left. \begin{array}{r l} E [ \boldsymbol {W} _ {k} ] = \mathbf {0}, & \operatorname{Cov} [ \boldsymbol {W} _ {k}, \boldsymbol {W} _ {j} ] = E [ \boldsymbol {W} _ {k} \boldsymbol {W} _ {j} ^ {\mathrm{T}} ] = Q _ {k} \delta_ {k j} \\ E [ \boldsymbol {V} _ {k} ] = \mathbf {0}, & \operatorname{Cov} [ \boldsymbol {V} _ {k}, \boldsymbol {V} _ {j} ] = E [ \boldsymbol {V} _ {k} \boldsymbol {V} _ {j} ^ {\mathrm{T}} ] = R _ {k} \delta_ {k j} \\ & \operatorname{Cov} [ \boldsymbol {W} _ {k}, \boldsymbol {V} _ {j} ] = E [ \boldsymbol {W} _ {k} \boldsymbol {V} _ {j} ^ {\mathrm{T}} ] = \mathbf {0} \end{array} \right\}\tag{2.2.3}
$$

式中， $Q_{k}$ 为系统噪声序列的方差阵，假设为非负定阵； $R_{k}$ 为量测噪声序列的方差阵，假设为正定阵。

定理2.7

如果被估计状态 $X_{k}$ 满足式(2.2.1)，对 $X_{k}$ 的量测量 $Z_{k}$ 满足式(2.2.2)，系统噪声 $W_{k}$ 和量测噪声 $V_{k}$ 满足式(2.2.3)，系统噪声方差阵 $Q_{k}$ 非负定，量测噪声方差阵 $R_{k}$ 正定， $k$ 时刻的量测为 $Z_{k}$ ，则 $X_{k}$ 的估计 $\hat{X}_{k}$ 按下述方程求解：

状态一步预测

$$
\hat {\boldsymbol {X}} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1}\tag{2.2.4a}
$$

状态估计

$$
\hat {\mathbf {X}} _ {k} = \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1})\tag{2.2.4b}
$$

滤波增益

$$
\boldsymbol {K} _ {k} = \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1}\tag{2.2.4c}
$$

或

$$
\boldsymbol {K} _ {k} = \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1}\tag{2.2.4c'}
$$

一步预测均方误差

$$
\boldsymbol {P} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {Q} _ {k - 1} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}}\tag{2.2.4d}
$$

估计均方误差

$$
\boldsymbol {P} _ {k} = (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1} (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) ^ {\mathrm{T}} + \boldsymbol {K} _ {k} \boldsymbol {R} _ {k} \boldsymbol {K} _ {k} ^ {\mathrm{T}}\tag{2.2.4e}
$$

或

$$
\boldsymbol {P} _ {k} = (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1}\tag{2.2.4e'}
$$

或

$$
\boldsymbol {P} _ {k} ^ {- 1} = \boldsymbol {P} _ {k / k - 1} ^ {- 1} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}\tag{2.2.4e \( ^{''} \)}
$$

式(2.2.4)即为离散型卡尔曼滤波基本方程。只要给定初值 $\hat{X}_{0}$ 和 $P_{0}$ ，根据k时刻的量测 $Z_{k}$ ，就可递推计算得k时刻的状态估计 $\hat{X}_{k}(k=1,2,\cdots)$ 。

式(2.2.4)所示算法可用图2.2.1来表示。从图中可明显看出卡尔曼滤波具有两个计算回路：增益计算回路和滤波计算回路。其中增益计算回路是独立计算回路，而滤波计算回路依赖于增益计算回路。

在一个滤波周期内，从卡尔曼滤波在使用系统信息和量测信息的先后次序来看，卡尔曼滤波具有两个明显的信息更新过程：时间更新过程和量测更新过程。式(2.2.4a)说明了根据 $k - 1$ 时刻的状态估计预测 $k$ 时刻状态估计的方法，式(2.2.4d)对这种预测的质量优劣作了定量描述。该两式的计算中仅使用了与系统动态特性有关的信息，如一步转移阵、噪声驱动阵、驱动噪声的方差阵。从时间的推移过程来看，该两式将时间从 $k - 1$ 时刻推进到 $k$ 时刻。所以该两式描述了卡尔曼滤波的时间更新过程。式(2.2.4)的其余诸式用来计算对时间更新值的修正量，该修正量由时间更新的质量优劣 $(P_{k / k - 1})$ 、量测信息的质量优劣 $(R_{k})$ 、量测与状态的关系 $(H_{k})$ 以及具体的量测值 $Z_{k}$ 所确定。所有这些方程围绕一个目的，即正确合理地利用量测 $Z_{k}$ 所以这一过程描述了卡尔曼滤波的量测更新过程。该两过程如图2.2.1所示。

![](images/392e773b04005fe70eb35bdc5f0826c96ef4472dd347ba2bdbacf9bca2e8561e.jpg)  
图2.2.1 卡尔曼滤波的两个计算回路和两个更新过程

## 2.2.2 离散型卡尔曼滤波基本方程的直观推导

为了加深对离散型卡尔曼滤波基本方程的理解, 此处根据直观理解和物理概念推导出式(2.2.4)。

## 1. 一步预测方程推导

一步预测是根据 $k - 1$ 时刻的状态估计预测 $k$ 时刻的状态，即根据 $k - 1$ 个量测 $\mathbf{Z}_1, \mathbf{Z}_2, \mathbf{Z}_{k-1}$ 对 $\mathbf{X}_k$ 作线性最小方差估计

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {k / k - 1} & = E ^ {*} \left[ \mathbf {X} _ {k} / \mathbf {Z} _ {1} \mathbf {Z} _ {2} \dots \mathbf {Z} _ {k - 1} \right] = \\ & E ^ {*} \left[ (\boldsymbol {\Phi} _ {k, k - 1} \mathbf {X} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \mathbf {W} _ {k - 1}) / \mathbf {Z} _ {1} \mathbf {Z} _ {2} \dots \mathbf {Z} _ {k - 1} \right] \end{array}
$$

根据线性最小方差估计的性质(线性1)，有

$$
\hat {\mathbf {X}} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} E ^ {*} \left[ \mathbf {X} _ {k - 1} / \mathbf {Z} _ {1} \mathbf {Z} _ {2} \dots \mathbf {Z} _ {k - 1} \right] + \boldsymbol {\Gamma} _ {k - 1} E ^ {*} \left[ \mathbf {W} _ {k - 1} / \mathbf {Z} _ {1} \mathbf {Z} _ {2} \dots \mathbf {Z} _ {k - 1} \right]
$$

由式(2.2.1)知, $W_{k-1}$ 只影响 $X_{k}$ , 所以 $W_{k-1}$ 与 $Z_{1}Z_{2}\cdots Z_{k-1}$ 不相关, 且 $E[W_{k-1}] = 0$ , 根据式(2.1.64), 有

$$
E ^ {*} \left[ W _ {k - 1} / Z _ {1} Z _ {2} \dots Z _ {k - 1} \right] = 0
$$

而

$$
E ^ {*} \left[ \mathbf {X} _ {k - 1} / \mathbf {Z} _ {1} \mathbf {Z} _ {2} \dots \mathbf {Z} _ {k - 1} \right] = \hat {\mathbf {X}} _ {k - 1}
$$

因此

$$
\hat {\boldsymbol {X}} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1}
$$

## 2. 状态估计方程推导

用一步预测代替真实状态 $X_{k}$ 引起的误差为

$$
\hat {\boldsymbol {X}} _ {k / k - 1} = \boldsymbol {X} _ {k} - \hat {\boldsymbol {X}} _ {k / k - 1}
$$

引起对量测的估计误差为

$$
\widetilde {\mathbf {Z}} _ {k / k - 1} = \mathbf {Z} _ {k} - \hat {\mathbf {Z}} _ {k / k - 1} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1} = \mathbf {H} _ {k} \widetilde {\mathbf {X}} _ {k / k - 1} + \mathbf {V} _ {k}
$$

滤波理论中称 $\tilde{Z}_{k/k-1}$ 为残差(也称新息)。

从上式可看出，残差包含有一步预测误差信息，对 $\widetilde{\mathbf{Z}}_{k / k - 1}$ 作适当的加权处理就能将 $\widetilde{\mathbf{X}}_{k / k - 1}$ 分离出来，用来修正 $\hat{\mathbf{X}}_{k / k - 1}$ 即可得到状态的估计

$$
\hat {\mathbf {X}} _ {k} = \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} \hat {\mathbf {Z}} _ {k / k - 1} = \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1})
$$

式中， $K_{k}$ 为对残差的加权阵，称为滤波增益阵。

## 3. 滤波增益阵和估计均方误差阵的推导

增益阵的选取准则是使估计的均方误差阵 $P_{k}=E[\widetilde{X}_{k}\widetilde{X}_{k}^{T}]$ 达到最小，其中 $\widetilde{X}_{k}=X_{k}-\widehat{X}_{k}$ 为估计误差。而

$$
\begin{array}{r l} \widetilde {\mathbf {X}} _ {k} & = \mathbf {X} _ {k} - \hat {\mathbf {X}} _ {k} = \mathbf {X} _ {k} - [ \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1}) ] = \\ & \quad \widetilde {\mathbf {X}} _ {k / k - 1} - \mathbf {K} _ {k} (\mathbf {H} _ {k} \widetilde {\mathbf {X}} _ {k / k - 1} + \mathbf {V} _ {k}) = \\ & \quad (\mathbf {I} - \mathbf {K} _ {k} \mathbf {H} _ {k}) \widetilde {\mathbf {X}} _ {k / k - 1} - \mathbf {K} _ {k} \mathbf {V} _ {k} \end{array}
$$

所以

$$
\begin{array}{r l} \mathbf {P} _ {k} & = E [ \widetilde {\mathbf {X}} _ {k} \widetilde {\mathbf {X}} _ {k} ^ {\mathrm{T}} ] = E \{[ (\mathbf {I} - \mathbf {K} _ {k} \mathbf {H} _ {k}) \widetilde {\mathbf {X}} _ {k / k - 1} - \mathbf {K} _ {k} \mathbf {V} _ {k} ] [ (\mathbf {I} - \mathbf {K} _ {k} \mathbf {H} _ {k}) \widetilde {\mathbf {X}} _ {k / k - 1} - \mathbf {K} _ {k} \mathbf {V} _ {k} ] ^ {\mathrm{T}} \} = \\ & (\mathbf {I} - \mathbf {K} _ {k} \mathbf {H} _ {k}) E [ \widetilde {\mathbf {X}} _ {k / k - 1} \widetilde {\mathbf {X}} _ {k / k - 1} ^ {\mathrm{T}} ] (\mathbf {I} - \mathbf {K} _ {k} \mathbf {H} _ {k}) ^ {\mathrm{T}} + \mathbf {K} _ {k} E [ \mathbf {V} _ {k} \mathbf {V} _ {k} ^ {\mathrm{T}} ] \mathbf {K} _ {k} ^ {\mathrm{T}} - \\ & (\mathbf {I} - \mathbf {K} _ {k} \mathbf {H} _ {k}) E [ \widetilde {\mathbf {X}} _ {k / k - 1} \mathbf {V} _ {k} ^ {\mathrm{T}} ] \mathbf {K} _ {k} ^ {\mathrm{T}} - \mathbf {K} _ {k} E [ \mathbf {V} _ {k} \widetilde {\mathbf {X}} _ {k / k - 1} ^ {\mathrm{T}} ] (\mathbf {I} - \mathbf {K} _ {k} \mathbf {H} _ {k}) ^ {\mathrm{T}} \end{array}
$$

由于 $\hat{\mathbf{X}}_{k / k - 1}$ 是根据 $k - 1$ 时刻前的量测对 $k$ 时刻的状态所作的估计，而 $\mathbf{V}_k$ 是 $k$ 时刻的量测噪声，所以 $\mathbf{V}_k$ 与 $\tilde{\mathbf{X}}_{k / k - 1} = \mathbf{X}_k - \hat{\mathbf{X}}_{k / k - 1}$ 不相关，并注意到 $E[\mathbf{V}_k] = \mathbf{0}$ ，因此有

$$
E \left[ \widetilde {\boldsymbol {X}} _ {k / k - 1} \boldsymbol {V} _ {k} ^ {\mathrm{T}} \right] = E \left[ \boldsymbol {V} _ {k} \widetilde {\boldsymbol {X}} _ {k / k - 1} ^ {\mathrm{T}} \right] = \mathbf {0}
$$

代入 $P_{k}$ 的表达式，得

$$
\boldsymbol {P} _ {k} = \left(\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}\right) \boldsymbol {P} _ {k / k - 1} \left(\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}\right) ^ {\mathrm{T}} + \boldsymbol {K} _ {k} \boldsymbol {R} _ {k} \boldsymbol {K} _ {k} ^ {\mathrm{T}}
$$

式中

$$
\begin{array}{r l} & {\pmb {P} _ {k / k - 1} = E [ \widetilde {\pmb {X}} _ {k / k - 1} \widetilde {\pmb {X}} _ {k / k - 1} ^ {\mathrm{T}} ]} \\ & {\pmb {R} _ {k} = E [ \pmb {V} _ {k} \pmb {V} _ {k} ^ {\mathrm{T}} ]} \end{array}
$$

下面根据极值原理从式(2.2.4e)推导出滤波增益阵 $K_{k}$ 。

设 $K_{k}$ 是使估计的均方误差阵达到最小的最佳增益阵，并设该最小均方误差阵为 $P_{k}$ 。显然，若滤波增益阵偏离最佳增益阵的偏离量为 $\delta K_{k}$ ，则由式(2.2.4e)确定的估计的均方误差将偏离最小值 $P_{k}$ 而达到 $P_{k} + \delta P_{k}$ ，且 $\delta P_{k}$ 为非负定阵，即 $\delta P_{k} \geqslant 0$ 。 $K_{k} + \delta K_{k}$ 和 $P_{k} + \delta P_{k}$ 满足的方程为

$$
\begin{array}{r l} \boldsymbol {P} _ {k} + \delta \boldsymbol {P} _ {k} & = [ \boldsymbol {I} - (\boldsymbol {K} _ {k} + \delta \boldsymbol {K} _ {k}) \boldsymbol {H} _ {k} ] \boldsymbol {P} _ {k / k - 1} [ \boldsymbol {I} - (\boldsymbol {K} _ {k} + \delta \boldsymbol {K} _ {k}) \boldsymbol {H} _ {k} ] ^ {\mathrm{T}} + \\ & (\boldsymbol {K} _ {k} + \delta \boldsymbol {K} _ {k}) \boldsymbol {R} _ {k} (\boldsymbol {K} _ {k} + \delta \boldsymbol {K} _ {k}) ^ {\mathrm{T}} \end{array}
$$

其中 $K_{k}$ 和 $P_{k}$ 满足式(2.2.4e)。将式(2.2.4e)代入上式，得

$$
\delta \boldsymbol {P} _ {k} = \boldsymbol {W} + \boldsymbol {W} ^ {\mathrm{T}} + \delta \boldsymbol {K} _ {k} \left(\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}\right) \delta \boldsymbol {K} _ {k} ^ {\mathrm{T}}
$$

式中

$$
\boldsymbol {W} = - \delta \boldsymbol {K} _ {k} \left[ \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} (\boldsymbol {I} - \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {K} _ {k} ^ {\mathrm{T}}) - \boldsymbol {R} _ {k} \boldsymbol {K} _ {k} ^ {\mathrm{T}} \right] =
$$

若取

$$
\begin{array}{r l} & - \delta \mathbf {K} _ {k} \left[ \mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} - (\mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} + \mathbf {R} _ {k}) \mathbf {K} _ {k} ^ {\mathrm{T}} \right] \\ & \quad \mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} - (\mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} + \mathbf {R} _ {k}) \mathbf {K} _ {k} ^ {\mathrm{T}} = \mathbf {0} \end{array}
$$

即

$$
\boldsymbol {K} _ {k} = \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1}
$$

则

$$
\begin{array}{r l} & W = 0 \\ \delta P _ {k} = \delta K _ {k} (H _ {k} P _ {k / k - 1} H _ {k} ^ {\mathrm{T}} + R _ {k}) \delta K _ {k} ^ {\mathrm{T}} \end{array}
$$

由于 $R_{k}$ 为正定阵， $P_{k/k-1}$ 至少为非负定阵，所以 $H_{k}P_{k/k-1}H_{k}^{\mathrm{T}}+R_{k}$ 至少为非负定阵。若 $\delta K_{k}$ 为非零阵，则 $\delta P_{k}$ 至少为非负定阵，即 $\delta P_{k}\geqslant0$ 。这说明，若 $K_{k}$ 按式(2.2.4c)确定，则对于相对增益阵 $K_{k}$ 的任何偏离 $\delta K_{k}\neq0$ ，估计的均方误差将产生非负的偏差 $\delta P_{k}$ ，因此 $K_{k}$ 是使估计的均方误差达到最小的最佳增益阵。

## 4. 一步预测均方误差阵的推导

一步预测产生的误差为

$$
\begin{array}{r l} \widetilde {\boldsymbol {X}} _ {k / k - 1} & = \boldsymbol {X} _ {k} - \hat {\boldsymbol {X}} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {X} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1} - \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1} = \\ & \quad \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1} \end{array}
$$

所以一步预测均方误差阵为

$$
\begin{array}{r l} \boldsymbol {P} _ {k / k - 1} & = E [ \widetilde {\boldsymbol {X}} _ {k / k - 1} \widetilde {\boldsymbol {X}} _ {k / k - 1} ^ {\mathrm{T}} ] = \\ & E \{[ \boldsymbol {\Phi} _ {k, k - 1} \widetilde {\boldsymbol {X}} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1} ] [ \boldsymbol {\Phi} _ {k, k - 1} \widetilde {\boldsymbol {X}} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1} ] ^ {\mathrm{T}} \} \end{array}
$$

由于 $\widetilde{\pmb{X}}_{k - 1} = \pmb{X}_{k - 1} - \hat{\pmb{X}}_{k - 1},\pmb{W}_{k - 1}$ 只影响 $X_{k}$ ，不影响 $X_{k - 1}$ ，所以 $W_{k - 1}$ 与 $\widetilde{\pmb{X}}_{k - 1}$ 不相关，又因 $E[\pmb {W}_{k - 1}] = \mathbf{0}$ ，因此

$$
\boldsymbol {P} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {Q} _ {k - 1} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}}
$$

## 2.2.3 离散型卡尔曼滤波基本方程的正交投影推导

基本方程除利用物理概念经直观推导获得外,还可用严密的数学概念推导获得。介绍这一内容的目的是使读者了解卡尔曼滤波的数学内涵。

1. 正交投影定理

设 X 和 Z 都是随机向量, 如果

$$
E [ \mathbf {X Z} ^ {\mathrm{T}} ] = \mathbf {0}\tag{2.2.5}
$$

则称 X 与 Z 正交。

应注意随机向量间正交、不相关和独立三者的区别。

如果随机向量 X 和 Z 的协方差阵为零, 即

$$
\operatorname{Cov} (\mathbf {X}, \mathbf {Z}) = E \left[ \left(\mathbf {X} - \boldsymbol {m} _ {\mathrm{X}}\right) \left(\mathbf {Z} - \boldsymbol {m} _ {\mathrm{Z}}\right) ^ {\mathrm{T}} \right] = E \left[ \mathbf {X Z} ^ {\mathrm{T}} \right] - \boldsymbol {m} _ {\mathrm{X}} \boldsymbol {m} _ {\mathrm{Z}} ^ {\mathrm{T}} = \mathbf {0}
$$

即

$$
E [ \mathbf {X Z} ^ {\mathrm{T}} ] = \boldsymbol {m} _ {X} \boldsymbol {m} _ {Z} ^ {\mathrm{T}}
$$

(2.2.6)

式中 $m_{X}$ 和 $m_{Z}$ 分别为 X 和 Z 的均值，则称 X 和 Z 不相关。

如果 $X$ 和 $Z$ 的联合分布密度

$$
p (x, z) = p _ {1} (x) \cdot p _ {2} (z)\tag{2.2.7}
$$

式中 $p_{1}(x)$ 和 $p_{2}(z)$ 分别为 X 和 Z 的分布密度，则称 X 和 Z 互相独立。

从上述定义可得出如下结论：

（1）若 X 与 Z 独立，则 X 与 Z 一定不相关。但逆命题一般并不成立，只有当 X 和 Z 都服从正态分布时才成立。

(2) 如果 X 和 Z 的数学期望至少有一个为零, 则不相关与正交等价。

（3）如果 X 和 Z 都服从正态分布，且至少有一个数学期望为零，则不相关、正交和独立三者等价。

如果存在某矩阵 $A^{1}$ 和某向量 $b^{1}$ ，对任意矩阵 A 和任意向量 b 都能使下式成立

$$
E \left\{\left[ \boldsymbol {X} - (\boldsymbol {A} ^ {1} \boldsymbol {Z} + \boldsymbol {b} ^ {1}) \right] (\boldsymbol {A Z} + \boldsymbol {b}) ^ {\mathrm{T}} \right\} = \mathbf {0}\tag{2.2.8}
$$

则称 $A^{1}Z + b^{1}$ 为 X 在 Z 上的正交投影。

式 $(2.2.8)$ 也可改写成如下形式：

$$
\begin{array}{r l} & E \{\left[ \mathbf {X} - (\mathbf {A} ^ {1} \mathbf {Z} + \mathbf {b} ^ {1}) \right] (\mathbf {A Z} + \mathbf {b}) ^ {\mathrm{T}} \} = \\ & E \{\left[ \mathbf {X} - (\mathbf {A} ^ {1} \mathbf {Z} + \mathbf {b} ^ {1}) \right] \mathbf {Z} ^ {\mathrm{T}} \} \mathbf {A} ^ {\mathrm{T}} + E \{\mathbf {X} - (\mathbf {A} ^ {1} \mathbf {Z} + \mathbf {b} ^ {1}) \} \mathbf {b} ^ {\mathrm{T}} = \mathbf {0} \end{array}
$$

由于 A 为任意矩阵, b 为任意向量, 要使上式恒能成立, 须有

$$
\left. \begin{array}{l} E \{\left[ \mathbf {X} - (\mathbf {A} ^ {1} \mathbf {Z} + \mathbf {b} ^ {1}) \right] \mathbf {Z} ^ {\mathrm{T}} \} = \mathbf {0} \\ E [ \mathbf {X} - (\mathbf {A} ^ {1} \mathbf {Z} + \mathbf {b} ^ {1}) ] = \mathbf {0} \end{array} \right\}\tag{2.2.9}
$$

式 $(2.2.9)$ 是正交投影定义的另一种形式。从该式可看出：

（1）正交投影是量测量Z和常值向量 $b^{1}$ 的线性组合，位于Z张成的空间内，该空间也称量测空间。

（2）若用正交投影作为 X 的估计，则估计误差与量测空间正交。

(3) 正交投影是 X 的无偏估计。

根据上述三点,可画出正交投影的几何示意图,如图2.2.2所示。

定理2.8

![](images/2cb20a1c8616edc7b6a00bb5fba81481306503fac3844f6c6e8b8ae5b321782e.jpg)

X 在 Z 上的正交投影即为 X 在 Z 上的线性最小方差估计, 反之亦然, 即

$$
\mathbf {A} ^ {1} \mathbf {Z} + \mathbf {b} ^ {1} = E ^ {*} [ \mathbf {X} / \mathbf {Z} ]
$$

图2.2.2 正交投影的几何示意图

(2.2.10)

证明

(1) 设 $E^{*}[X/Z]$ 是 X 在 Z 上的线性最小方差估计, 根据式(2.1.64), 有

$$
\begin{array}{r l} E \{(\mathbf {X} - E ^ {*} [ \mathbf {X} / \mathbf {Z} ]) \mathbf {Z} ^ {\mathrm{T}} \} & = E \{[ \mathbf {X} - m _ {X} - C _ {X Z} C _ {Z} ^ {- 1} (\mathbf {Z} - m _ {Z}) ] \mathbf {Z} ^ {\mathrm{T}} \} = \\ & E [ \mathbf {X Z} ^ {\mathrm{T}} ] - m _ {X} m _ {Z} ^ {\mathrm{T}} - C _ {X Z} C _ {Z} ^ {- 1} E [ \mathbf {Z Z} ^ {\mathrm{T}} ] + C _ {X Z} C _ {Z} ^ {- 1} m _ {Z} m _ {Z} ^ {\mathrm{T}} \end{array}
$$

由于

$$
E [ \mathbf {X Z} ^ {\mathrm{T}} ] = E \left\{\left[ (\mathbf {X} - \boldsymbol {m} _ {X}) + \boldsymbol {m} _ {X} \right] \left[ (\mathbf {Z} - \boldsymbol {m} _ {Z}) + \boldsymbol {m} _ {Z}\right) ^ {\mathrm{T}} \right\} = \boldsymbol {C} _ {X Z} + \boldsymbol {m} _ {X} \boldsymbol {m} _ {Z} ^ {\mathrm{T}}
$$

同理

$$
E [ \mathbf {Z} \mathbf {Z} ^ {\mathrm{T}} ] = C _ {Z} + m _ {Z} m _ {Z} ^ {\mathrm{T}}
$$

所以

$$
\begin{array}{r l} E \{(\mathbf {X} - E ^ {*} [ \mathbf {X} / \mathbf {Z} ]) \mathbf {Z} ^ {\mathrm{T}} \} & = \mathbf {C} _ {X Z} + m _ {X} \mathbf {m} _ {Z} ^ {\mathrm{T}} - m _ {X} \mathbf {m} _ {Z} ^ {\mathrm{T}} - \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} (\mathbf {C} _ {Z} + m _ {Z} \mathbf {m} _ {Z} ^ {\mathrm{T}}) + \\ & \quad \mathbf {C} _ {X Z} \mathbf {C} _ {Z} ^ {- 1} \mathbf {m} _ {Z} \mathbf {m} _ {Z} ^ {\mathrm{T}} = \mathbf {0} \end{array}
$$

此外，由于线性最小方差估计是无偏估计，即

$$
E \{\mathbf {X} - E ^ {*} [ \mathbf {X} / \mathbf {Z} ] \} = \mathbf {0}
$$

根据式(2.2.9)所给定义知, $E^{*}[X/Z]$ 是X在Z上的正交投影。

(2) 设 $A^{1}Z + b^{1}$ 为 X 在 Z 上的正交投影，则根据式(2.2.9)，有

$$
E \{\left[ \mathbf {X} - (\mathbf {A} ^ {\mathrm{I}} \mathbf {Z} + \mathbf {b} ^ {\mathrm{I}}) \right] \mathbf {Z} ^ {\mathrm{T}} \} = \mathbf {0}\tag{1}
$$

$$
E [ \mathbf {X} - (\mathbf {A} ^ {1} \mathbf {Z} + \mathbf {b} ^ {1}) ] = \mathbf {0}\tag{2}
$$

由(1)证明了线性最小方差估计 $E^{*}[X/Z]=A^{0}Z+b^{0}$ 是 X 在 Z 上的正交投影, 即有

$$
E \left\{\left[ \boldsymbol {X} - \left(\boldsymbol {A} ^ {0} \boldsymbol {Z} + \boldsymbol {b} ^ {0}\right) \right] \boldsymbol {Z} ^ {\mathrm{T}} \right\} = \mathbf {0}\tag{3}
$$

$$
E \{\mathbf {X} - (\mathbf {A} ^ {0} \mathbf {Z} + \mathbf {b} ^ {0}) \} = \mathbf {0}\tag{4}
$$

式(1) 减去式(3)，得

$$
\left(\boldsymbol {A} ^ {1} - \boldsymbol {A} ^ {0}\right) E \left[ \boldsymbol {Z} \boldsymbol {Z} ^ {\mathrm{T}} \right] + \left(\boldsymbol {b} ^ {1} - \boldsymbol {b} ^ {0}\right) E \left[ \boldsymbol {Z} ^ {\mathrm{T}} \right] = \mathbf {0}
$$

由于 $E[\mathbf{ZZ}^{\mathrm{T}}] = \mathbf{C}_{\mathrm{Z}} + \mathbf{m}_{\mathrm{Z}}\mathbf{m}_{\mathrm{Z}}^{\mathrm{T}}$ ，因此

$$
\left(\boldsymbol {A} ^ {1} - \boldsymbol {A} ^ {0}\right) \boldsymbol {C} _ {Z} + \left[ \left(\boldsymbol {A} ^ {1} - \boldsymbol {A} ^ {0}\right) m _ {Z} + \boldsymbol {b} ^ {1} - \boldsymbol {b} ^ {0} \right] m _ {Z} ^ {\mathrm{T}} = \mathbf {0}\tag{5}
$$

式(2) 减去式(4)，得

$$
(\mathbf {A} ^ {1} - \mathbf {A} ^ {0}) \mathbf {m} _ {Z} + \mathbf {b} ^ {1} - \mathbf {b} _ {0} = \mathbf {0}\tag{6}
$$

将式(6)代入式(5)，得

$$
(A ^ {1} - A ^ {0}) C _ {Z} = 0
$$

在估计过程中,为了充分利用量测信息,Z 的各分量必须是独立的,即有

$$
\begin{array}{r l} & E \big [ (Z _ {i} - m _ {Z _ {i}}) (Z _ {j} - m _ {Z _ {j}}) \big ] = \sigma_ {i} ^ {2} \delta_ {i j} \quad i = 1, 2, \dots , m, j = 1, 2, \dots , m \\ & \qquad \mathbf {C} _ {Z} = \mathrm{diag} \big [ \sigma_ {1} ^ {2} \sigma_ {2} ^ {2} \dots \sigma_ {m} ^ {2} \big ] \end{array}
$$

即 $C_z$ 满秩，因此有

$$
\mathbf {A} ^ {1} = \mathbf {A} ^ {0}
$$

将此关系式代入式(6)，得

$$
\boldsymbol {b} ^ {1} = \boldsymbol {b} ^ {0}
$$

从而有

$$
\mathbf {A} ^ {1} \mathbf {Z} + \mathbf {b} ^ {1} = \mathbf {A} ^ {0} \mathbf {Z} + \mathbf {b} ^ {0} = E ^ {*} [ \mathbf {X} / \mathbf {Z} ]
$$

这就证明了定理2.8。

2. 更新信息定理

定理 2.9(更新信息定理)

设 $X_{k}$ 为随机向量， $Z_{k}$ 为第 $k$ 次量测， $\overline{Z}_{k-1}$ 和 $\overline{Z}_{k}$ 分别为前 $k-1$ 次量测和前 $k$ 次量测，则

$$
E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{\mathbf {Z}}} _ {k} \right] = E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{\mathbf {Z}}} _ {k - 1} \right] + E \left[ \widetilde {\mathbf {X}} _ {k / k - 1} \widetilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} \right] \cdot \left\{E \left[ \widetilde {\mathbf {Z}} _ {k / k - 1} \widetilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} \right] \right\} ^ {- 1} \widetilde {\mathbf {Z}} _ {k / k - 1}\tag{2.2.11}
$$

即

$$
\hat {\mathbf {X}} _ {k} = \hat {\mathbf {X}} _ {k / k - 1} + E [ \widetilde {\mathbf {X}} _ {k / k - 1} \widetilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] \left\{E [ \widetilde {\mathbf {Z}} _ {k / k - 1} \widetilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] \right\} ^ {- 1} \widetilde {\mathbf {Z}} _ {k / k - 1}\tag{2.2.12}
$$

式中

$$
\begin{array}{r l} & \widetilde {\mathbf {X}} _ {k / k - 1} = \mathbf {X} _ {k} - E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{\mathbf {Z}}} _ {k - 1} \right] \\ & \widetilde {\mathbf {Z}} _ {k / k - 1} = \mathbf {Z} _ {k} - E ^ {*} \left[ \mathbf {Z} _ {k} / \overline {{\mathbf {Z}}} _ {k - 1} \right] \end{array}
$$

更新信息定理说明只需用 $\widetilde{\mathbf{Z}}_{k / k - 1}$ 对 $\hat{\mathbf{X}}_{k / k - 1}$ 作修正即可获得 $\hat{\mathbf{X}}_k$ ，所以 $\widetilde{\mathbf{Z}}_{k / k - 1}$ 是卡尔曼滤波中的一个重要信息源。由于 $\hat{\mathbf{Z}}_{k / k - 1} = E^{*}\left[\mathbf{Z}_{k} / \overline{\mathbf{Z}}_{k - 1}\right]$ 是前 $k - 1$ 次量测已提供的关于 $\mathbf{Z}_k$ 的信息，而 $\widetilde{\mathbf{Z}}_{k / k - 1}$ 含有第 $k$ 次量测 $\mathbf{Z}_k$ 新提供的信息，所以常将 $\widetilde{\mathbf{Z}}_{k / k - 1}$ 称为新息，这一定理也由此而得名。

证明

$$
\hat {\mathbf {X}} _ {k} = E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{{\mathbf {Z}}}} _ {k} \right] = E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{{\mathbf {Z}}}} _ {k - 1}, \mathbf {Z} _ {k} \right]
$$

而

$$
\mathbf {Z} _ {k} = \mathbf {Z} _ {k} - \hat {\mathbf {Z}} _ {k / k - 1} + \hat {\mathbf {Z}} _ {k / k - 1} = \hat {\mathbf {Z}} _ {k / k - 1} + \hat {\mathbf {Z}} _ {k / k - 1}\tag{1}
$$

式中

即 $\hat{\mathbf{Z}}_{k / k - 1}$ 是关于 $\overline{\mathbf{Z}}_{k - 1}$ 的线性函数，由式(1)可推知， $\mathbf{Z}_k$ 可由 $\overline{\mathbf{Z}}_{k - 1}$ 和 $\widetilde{\mathbf{Z}}_{k / k - 1}$ 线性表示，所以 $\hat{\mathbf{X}}_k$ 可由 $\overline{\mathbf{Z}}_{k - 1}$ 和 $\widetilde{\mathbf{Z}}_{k / k - 1}$ 线性表示

$$
\hat {\mathbf {X}} _ {k} = E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{\mathbf {Z}}} _ {k - 1}, \widetilde {\mathbf {Z}} _ {k / k - 1} \right]\tag{2}
$$

由于 $\widetilde{\mathbf{Z}}_{k / k - 1} = \mathbf{Z}_k - \hat{\mathbf{Z}}_{k / k - 1} = \mathbf{Z}_k - E^*\left[\mathbf{Z}_k / \overline{\mathbf{Z}}_{k - 1}\right]$ ，由定理2.8知 $\widetilde{\mathbf{Z}}_{k / k - 1}$ 与 $\overline{\mathbf{Z}}_{k - 1}$ 正交，即

$$
E [ \widetilde {\mathbf {Z}} _ {k / k - 1} \overline {{\mathbf {Z}}} _ {k - 1} ^ {\mathrm{T}} ] = \mathbf {0}\tag{3}
$$

且

$$
E [ \widetilde {\mathbf {Z}} _ {k / k - 1} ] = \mathbf {0}\tag{4}
$$

根据线性最小方差估计性质3(线性2)，式(2.1.64)及式(3)、式(4)，有

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {k} & = E ^ {*} [ \mathbf {X} _ {k} / \bar {\mathbf {Z}} _ {k - 1}, \tilde {\mathbf {Z}} _ {k / k - 1} ] = \\ & E ^ {*} [ \mathbf {X} _ {k} / \bar {\mathbf {Z}} _ {k - 1} ] + E ^ {*} [ \mathbf {X} _ {k} / \tilde {\mathbf {Z}} _ {k / k - 1} ] - E [ \mathbf {X} _ {k} ] = \\ & \hat {\mathbf {X}} _ {k / k - 1} + E [ \mathbf {X} _ {k} ] + E \{(\mathbf {X} _ {k} - E [ \mathbf {X} _ {k} ]) \tilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} \} \cdot \\ & \{E [ \tilde {\mathbf {Z}} _ {k / k - 1} \tilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] \} ^ {- 1} \tilde {\mathbf {Z}} _ {k / k - 1} - E [ \mathbf {X} _ {k} ] = \\ & \hat {\mathbf {X}} _ {k / k - 1} + \{E [ \mathbf {X} _ {k} \tilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] - E [ \mathbf {X} _ {k} ] E [ \tilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] \} \cdot \\ & \{E [ \tilde {\mathbf {Z}} _ {k / k - 1} \tilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] \} ^ {- 1} \tilde {\mathbf {Z}} _ {k / k - 1} = \\ & \hat {\mathbf {X}} _ {k / k - 1} + E [ (\hat {\mathbf {X}} _ {k / k - 1} + \tilde {\mathbf {X}} _ {k / k - 1}) \tilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] \cdot \\ & \{E [ \tilde {\mathbf {Z}} _ {k / k - 1} \tilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] \} ^ {- 1} \tilde {\mathbf {Z}} _ {k / k - 1} \end{array}
$$

注意到 $\hat{X}_{k/k-1}$ 是确定性向量, 所以

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {k} & = \hat {\mathbf {X}} _ {k / k - 1} + \{\hat {\mathbf {X}} _ {k / k - 1} E [ \widetilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] + E [ \widetilde {\mathbf {X}} _ {k / k - 1} \widetilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] \} \cdot \{E [ \widetilde {\mathbf {Z}} _ {k / k - 1} \widetilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] \} ^ {- 1} \widetilde {\mathbf {Z}} _ {k / k - 1} = \\ & \hat {\mathbf {X}} _ {k / k - 1} + E [ \widetilde {\mathbf {X}} _ {k / k - 1} \widetilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] \{E [ \widetilde {\mathbf {Z}} _ {k / k - 1} \widetilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] \} ^ {- 1} \widetilde {\mathbf {Z}} _ {k / k - 1} \end{array}
$$

## 3. 基本方程的正交投影推导

下面根据更新信息定理推导卡尔曼滤波的状态估计方程和最佳增益阵。

由于

$$
\begin{array}{r l} \widetilde {\mathbf {X}} _ {k / k - 1} & = \mathbf {X} _ {k} - \hat {\mathbf {X}} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \widetilde {\mathbf {X}} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \mathbf {W} _ {k - 1} \\ \widetilde {\mathbf {Z}} _ {k / k - 1} & = \mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1} = \mathbf {H} _ {k} \widetilde {\mathbf {X}} _ {k / k - 1} + \mathbf {V} _ {k} \end{array}
$$

并注意到 k 时刻的量测噪声不会影响 k-1 时刻前的估计, 所以

$$
\begin{array}{r l} & E [ \widetilde {\mathbf {X}} _ {k / k - 1} \widetilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] = E [ \widetilde {\mathbf {X}} _ {k / k - 1} (\mathbf {H} _ {k} \widetilde {\mathbf {X}} _ {k / k - 1} + \mathbf {V} _ {k}) ^ {\mathrm{T}} ] = \\ & \qquad E [ \widetilde {\mathbf {X}} _ {k / k - 1} \widetilde {\mathbf {X}} _ {k / k - 1} ^ {\mathrm{T}} ] \mathbf {H} _ {k} ^ {\mathrm{T}} + E [ \widetilde {\mathbf {X}} _ {k / k - 1} ] E [ \mathbf {V} _ {k} ^ {\mathrm{T}} ] = \\ & \qquad \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \\ & E [ \widetilde {\mathbf {Z}} _ {k / k - 1} \widetilde {\mathbf {Z}} _ {k / k - 1} ^ {\mathrm{T}} ] = E [ (\mathbf {H} _ {k} \widetilde {\mathbf {X}} _ {k / k - 1} + \mathbf {V} _ {k}) (\mathbf {H} _ {k} \widetilde {\mathbf {X}} _ {k / k - 1} + \mathbf {V} _ {k}) ^ {\mathrm{T}} ] = \\ & \qquad \mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} + \mathbf {R} _ {k} \end{array}
$$

将上述关系式代入式 $(2.2.12)$ ，得

$$
\hat {\boldsymbol {X}} _ {k} = \hat {\boldsymbol {X}} _ {k / k - 1} + \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \left(\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}\right) ^ {- 1} \left(\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \hat {\boldsymbol {X}} _ {k / k - 1}\right)
$$

令

$$
\boldsymbol {K} _ {k} = \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1}
$$

则

$$
\hat {\mathbf {X}} _ {k} = \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1})
$$

一步预测方程、一步预测的均方误差阵方程和估计的均方误差阵方程的推导与直观推导方法中所述的一样，此处不再重复。

从上述推导过程可看出， $\hat{\mathbf{X}}_k$ 是量测空间 $\overline{\mathbf{Z}}_k$ 上的正交投影，根据定理2.8，正交投影与线性最小方差估计两者等价，所以卡尔曼滤波是 $X_k$ 在 $\overline{\mathbf{Z}}_k$ 上的线性最小方差估计， $\hat{\mathbf{X}}_k$ 利用了 $k$ 时刻及 $k$ 时刻以前的所有量测信息，即按式(2.2.4)确定的估计与按式(2.1.64)确定的估计是一样的，只是算法不同而已。与式(2.1.64)相比，离散型卡尔曼滤波基本方程有如下明显优点：

（1）由于采用了递推算法，不同时刻的量测值不必储存起来，而是经实时处理提炼成被估计状态的信息，随着滤波步数的增加，提取出的信息浓度逐渐增加。

（2）不必了解被估计量和量测量在不同时刻的一、二阶矩，而只须知道驱动被估计量的驱动噪声的统计特性、描述这种驱动作用的系统状态方程及量测噪声的统计特性。驱动噪声和量测噪声都是白噪声，是平稳过程，统计特性不随时间而变，系统的状态方程又是准确已知的，所以卡尔曼滤波能对非平稳的被估计量作估计。

(3) 算法可直接在计算机上实现。

正由于卡尔曼滤波具有其他估计不具备的优点,所以受到工程界的高度重视。

## 2.2.4 离散型卡尔曼滤波基本方程使用要点

1. 滤波初值的选取

卡尔曼滤波是一种递推算法，启动时必须先给定初值 $\hat{X}_{0}$ 和 $P_{0}$ 。

定理2.10

如果选取 $\hat{X}_{0}=m_{X_{0}}$ ，则滤波过程中估计始终无偏，即 $E[\hat{X}_{k}]=E[X_{k}]$ 。

证明

由式(2.2.4b)、式(2.2.1)及式(2.2.2)，得

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {1} & = \boldsymbol {\Phi} _ {1, 0} \hat {\mathbf {X}} _ {0} + \mathbf {K} _ {1} (\mathbf {Z} _ {1} - \mathbf {H} _ {1} \boldsymbol {\Phi} _ {1, 0} \hat {\mathbf {X}} _ {0}) = \\ & \quad \boldsymbol {\Phi} _ {1, 0} \hat {\mathbf {X}} _ {0} + \mathbf {K} _ {1} [ \mathbf {H} _ {1} (\boldsymbol {\Phi} _ {1, 0} \mathbf {X} _ {0} + \boldsymbol {\Gamma} _ {0} \mathbf {W} _ {0}) + \mathbf {V} _ {1} - \mathbf {H} _ {1} \boldsymbol {\Phi} _ {1, 0} \hat {\mathbf {X}} _ {0} ] \end{array}
$$

对上式取均值，并设 $\hat{\pmb{X}}_0 = \pmb{m}_{X_0}$ ，则有

$$
E [ \hat {X} _ {1} ] = \Phi_ {1, 0} m _ {X _ {0}}
$$

而

$$
E [ \boldsymbol {X} _ {1} ] = E [ \boldsymbol {\Phi} _ {1, 0} \boldsymbol {X} _ {0} + \boldsymbol {\Gamma} _ {0} \boldsymbol {W} _ {0} ] = \boldsymbol {\Phi} _ {1, 0} \boldsymbol {m} _ {X _ {0}} = E [ \hat {\boldsymbol {X}} _ {1} ]
$$

同理有

$$
\begin{array}{r l} E [ \hat {X} _ {2} ] & = E [ \Phi_ {2, 1} \hat {X} _ {1} + K _ {2} (Z _ {2} - H _ {2} \Phi_ {2, 1} \hat {X} _ {1}) ] = \\ & E [ \Phi_ {2, 1} \Phi_ {1, 0} m _ {X _ {0}} + K _ {2} H _ {2} \Phi_ {2, 1} (X _ {1} - \hat {X} _ {1}) + K _ {2} V _ {2} + K _ {2} H _ {1} \Gamma_ {1} W _ {1} ] = \\ & \Phi_ {2, 0} m _ {X _ {0}} = E [ X _ {2} ] \end{array}
$$

根据数学归纳法可以证明

$$
E [ \hat {X} _ {k} ] = \Phi_ {k, 0} m _ {X _ {0}} = E [ X _ {k} ]
$$

从证明过程可看出，一步预测 $\hat{\pmb{X}}_{k / k - 1}$ 和 $\hat{\pmb{Z}}_{k / k - 1}$ 也都是无偏的，即

$$
\begin{array}{l} E [ \hat {\mathbf {X}} _ {k / k - 1} ] = E [ \mathbf {X} _ {k} ] \\ E [ \hat {\mathbf {Z}} _ {k / k - 1} ] = E [ \mathbf {Z} _ {k} ] \end{array}
$$

需要指出,如果滤波的起始时刻有量测量 $Z_{0}$ , 则可令

$$
\begin{array}{r l} & \hat {\boldsymbol {X}} _ {0 / - 1} = \boldsymbol {m} _ {X _ {0}} \\ & \boldsymbol {P} _ {0 / - 1} = \boldsymbol {C} _ {X _ {0}} \end{array}
$$

此时 $\hat{X}_{0}$ 就不再是初始值了，而是经过 $Z_{0}$ 修正后的估计。根据式(2.2.4b) 和式(2.2.4c')， $\hat{X}_{0}$ 为

$$
\hat {\boldsymbol {X}} _ {0} = \boldsymbol {m} _ {X _ {0}} + \boldsymbol {C} _ {X _ {0}} \boldsymbol {H} _ {0} ^ {\mathrm{T}} (\boldsymbol {H} _ {0} \boldsymbol {C} _ {X _ {0}} \boldsymbol {H} _ {0} ^ {\mathrm{T}} + \boldsymbol {R} _ {0}) ^ {- 1} (\boldsymbol {Z} _ {0} - \boldsymbol {H} _ {0} \boldsymbol {m} _ {X _ {0}})\tag{2.2.13}
$$

如果并不了解初始状态的统计特性,常令

$$
\hat {X} _ {0} = 0, P _ {0} = \alpha I
$$

或

其中 $\alpha$ 为很大的正数，在此情况下，滤波器不能保证是无偏的。由于 $P_0 \neq C_{X_0}$ ，所以实际的估计均方误差也不一定是最小的， $P_0$ 对滤波的影响将在第四章中讨论。事实上，如果系统是一致完全随机可控和一致完全随机可观测的，则卡尔曼滤波器一定是一致渐近稳定的。随着滤波步数的增加，盲目选取的滤波初值 $\hat{X}_0$ 和 $P_0$ 对滤波值 $\hat{X}_k$ 和 $P_k$ 的影响将逐渐减弱直至消失，估计逐渐趋向无偏，估计的均方误差也逐渐和 $P_0 = C_{X_0}$ 时的结果相一致。因此，在不了解初始状态的统计特性的情况下，一般应将滤波器设计成一致渐近稳定的。第四章将对这一问题作详细探讨。

## 2. 估计均方误差阵的等价形式及选用

式 $(2.2.4)$ 给出了计算估计的均方误差阵的三种等价形式

$$
\boldsymbol {P} _ {k} = (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1} (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) ^ {\mathrm{T}} + \boldsymbol {K} _ {k} \boldsymbol {P} _ {k} \boldsymbol {K} _ {k} ^ {\mathrm{T}}\tag{2.2.4e}
$$

$$
\boldsymbol {P} _ {k} = (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1}\tag{2.2.4e'}
$$

$$
\boldsymbol {P} _ {k} ^ {- 1} = \boldsymbol {P} _ {k / k - 1} ^ {- 1} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}\tag{2.2.4e \( ^{''} \)}
$$

它们形式上迥然不同,但本质上具有同解关系,通过恒等变形可逐一推导出这种同解关系。展开式(2.2.4e),得

$$
\begin{array}{r l} & {\pmb {P} _ {k} = (\pmb {I} - \pmb {K} _ {k} \pmb {H} _ {k}) \pmb {P} _ {k / k - 1} (\pmb {I} - \pmb {K} _ {k} \pmb {H} _ {k}) ^ {\mathrm{T}} + \pmb {K} _ {k} \pmb {R} _ {k} \pmb {K} _ {k} ^ {\mathrm{T}} =} \\ & {\qquad \pmb {P} _ {k / k - 1} - \pmb {P} _ {k / k - 1} \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {K} _ {k} ^ {\mathrm{T}} - \pmb {K} _ {k} \pmb {H} _ {k} \pmb {P} _ {k / k - 1} + \pmb {K} _ {k} \pmb {H} _ {k} \pmb {P} _ {k / k - 1} \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {K} _ {k} ^ {\mathrm{T}} + \pmb {K} _ {k} \pmb {R} _ {k} \pmb {K} _ {k} ^ {\mathrm{T}} =} \\ & {\qquad \pmb {P} _ {k / k - 1} - \pmb {P} _ {k / k - 1} \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {K _ {k}} ^ {\mathrm{T}} - \pmb {K} _ {k} \pmb {H} _ {k} \pmb {P} _ {k / k - 1} + \pmb {K} _ {k} (\pmb {H} _ {k} \pmb {P} _ {k / k - 1} \pmb {H} _ {k} ^ {\mathrm{T}} + \pmb {R} _ {k}) \pmb {K _ {k}} ^ {\mathrm{T}}} \end{array}
$$

上式右侧第4项中的 $K_{k}$ 用式(2.2.4c)代入，得

$$
\begin{array}{r l} & {\pmb {P} _ {k} = \pmb {P} _ {k / k - 1} - \pmb {P} _ {k / k - 1} \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {K} _ {k} ^ {\mathrm{T}} - \pmb {K} _ {k} \pmb {H} _ {k} \pmb {P} _ {k / k - 1} + \pmb {P} _ {k / k - 1} \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {K} _ {k} ^ {\mathrm{T}} =} \\ & {\qquad (\pmb {I} - \pmb {K} _ {k} \pmb {H} _ {k}) \pmb {P} _ {k / k - 1}} \end{array}\tag{2.2.4e'}
$$

根据式 $(2.2.4c)$ ，式 $(2.2.4e')$ 又可写成

$$
\boldsymbol {P} _ {k} = \boldsymbol {P} _ {k / k - 1} - \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \left(\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}\right) ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1}
$$

根据矩阵反演公式(2.1.23):

令

$$
\begin{array}{r l} & (A _ {1 1} - A _ {1 2} A _ {2 2} ^ {- 1} A _ {2 1}) ^ {- 1} = A _ {1 1} ^ {- 1} + A _ {1 1} ^ {- 1} A _ {1 2} (A _ {2 2} - A _ {2 1} A _ {1 1} ^ {- 1} A _ {1 2}) ^ {- 1} A _ {2 1} A _ {1 1} ^ {- 1} \\ & \quad P _ {k / k - 1} = A _ {1 1} ^ {- 1}, \quad - H _ {k} ^ {\mathrm{T}} = A _ {1 2}, \quad H _ {k} = A _ {2 1}, \quad R _ {k} = A _ {2 2} \end{array}
$$

则

$$
\boldsymbol {P} _ {k} = (\boldsymbol {P} _ {k / k - 1} ^ {- 1} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}) ^ {- 1}
$$

即

$$
\boldsymbol {P} _ {k} ^ {- 1} = \boldsymbol {P} _ {k / k - 1} ^ {- 1} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}\tag{2.2.4e \( ^{''} \)}
$$

又由式 $(2.2.4e^{\prime\prime})$ 可推导出式 $(2.2.4c)$ 的等价式 $(2.2.4c^{\prime})$ 。

先求解(2.2.4c)式中的 $\left(\boldsymbol{H}_{k}\boldsymbol{P}_{k/k-1}\boldsymbol{H}_{k}^{\mathrm{T}}+\boldsymbol{R}_{k}\right)^{-1}$ ，根据矩阵反演公式(2.1.23)，令 $R_{k}=A_{11}$ ， $H_{k}=-A_{12},P_{k/k-1}=A_{22}^{-1},H_{k}^{\mathrm{T}}=A_{21}$ ，则

$$
\begin{array}{r l} & (\mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} + \mathbf {R} _ {k}) ^ {- 1} = \mathbf {R} _ {k} ^ {- 1} + \mathbf {R} _ {k} ^ {- 1} (- \mathbf {H} _ {k}) [ \mathbf {P} _ {k / k - 1} ^ {- 1} - \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} (- \mathbf {H} _ {k}) ] ^ {- 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} = \\ & \quad \mathbf {R} _ {k} ^ {- 1} - \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k} (\mathbf {P} _ {k / k - 1} ^ {- 1} + \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k}) ^ {- 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \end{array}
$$

式 $(2.2.4e^{\prime\prime})$ 代入上式，得

$$
\begin{array}{r l} & (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1} = [ \boldsymbol {I} - \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} (\boldsymbol {P} _ {k} ^ {- 1}) ^ {- 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} ] \boldsymbol {R} _ {k} ^ {- 1} = \\ & (\boldsymbol {I} - \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}}) \boldsymbol {R} _ {k} ^ {- 1} \end{array}
$$

上式代入式(2.2.4c)，并根据式(2.2.4e"），得

$$
\begin{array}{r l} \boldsymbol {K} _ {k} & = \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {I} - \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}}) \boldsymbol {R} _ {k} ^ {- 1} = \boldsymbol {P} _ {k / k - 1} (\boldsymbol {P} _ {k} ^ {- 1} - \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} = \\ & \boldsymbol {P} _ {k / k - 1} \boldsymbol {P} _ {k / k - 1} ^ {- 1} \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} = \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \end{array}\tag{2.2.4c'}
$$

式(2.2.4e)、式(2.2.4e')及式(2.2.4e'')是等价的，滤波过程中应根据实际情况选择使用。式(2.2.4e')形式简单，计算量较小，但计算中的积累误差容易使 $P_{k}$ 失去非负定性甚至对称性。所以实际使用中常用式(2.2.4e)。如果在滤波初始时刻对被估计量的统计特性缺乏了解，选取滤波初值盲目，相应地 $P_{0}$ 选得十分巨大，计算 $P_{1/0}$ 和 $K_{1}$ 就会有溢出的可能，在此情况下，宜采用式(2.2.4e'')，一步预测的均方误差阵也用逆阵来表示。这种逆阵称为信息矩阵，采用信息矩阵表示的滤波方程称为信息滤波方程。关于信息滤波将在第三章中介绍。

## 3. 一步转移阵和等效离散系统噪声方差阵的计算

由卡尔曼滤波的推导知，式(2.2.4)所示基本方程只适用于系统方程和量测方程都是离散型的情况，如式(2.2.1)和式(2.2.2)所示。但实际的物理系统一般都是连续的，动力学特性用连续微分方程描述。所以使用基本方程之前，必须对系统方程和量测方程作离散化处理。

设描述物理系统动力学特性的系统方程为

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {F} (t) \boldsymbol {X} (t) + \boldsymbol {G} (t) \boldsymbol {w} (t)\tag{2.2.14}
$$

其中系统的驱动源 $w(t)$ 为白噪声过程, 即

$$
\begin{array}{l} E [ \boldsymbol {w} (t) ] = \mathbf {0} \\ E [ \boldsymbol {w} (t) \boldsymbol {w} ^ {\mathrm{T}} (\tau) ] = \boldsymbol {q} \delta (t - \tau) \end{array}
$$

式中 q 为 $w(t)$ 的方差强度阵。

根据线性系统理论,系统方程的离散化形式为

$$
\boldsymbol {X} (t _ {k + 1}) = \boldsymbol {\Phi} (t _ {k + 1}, t _ {k}) \boldsymbol {X} (t _ {k}) + \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {\Phi} (t _ {k + 1}, \tau) \boldsymbol {G} (\tau) w (\tau) \mathrm{d} \tau\tag{2.2.15}
$$

其中，一步转移阵 $\Phi (t_{k + 1},t_k)$ 满足方程

$$
\left. \begin{array}{l} \dot {\boldsymbol {\Phi}} (t, t _ {k}) = \boldsymbol {F} (t) \boldsymbol {\Phi} (t, t _ {k}) \\ \boldsymbol {\Phi} (t _ {k}, t _ {k}) = \boldsymbol {I} \end{array} \right\}\tag{2.2.16}
$$

求解该方程, 得

$$
\boldsymbol {\Phi} (t _ {k + 1}, t _ {k}) = \exp \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {F} (t) \mathrm{d} t
$$

当滤波周期 $T(T=t_{k+1}-t_{k})$ 较短时， $F(t)$ 可近似看做常阵，即

$$
\mathbf {F} (t) \approx \mathbf {F} (t _ {k}) \qquad t _ {k} \leqslant t <   t _ {k + 1}
$$

此时

$$
\Phi (t _ {k + 1}, t _ {k}) = \mathrm{e} ^ {T F (t _ {k})}
$$

即

$$
\boldsymbol {\Phi} _ {k + 1, k} = \boldsymbol {I} + T \boldsymbol {F} _ {k} + \frac {T ^ {2}}{2 !} \boldsymbol {F} _ {k} ^ {2} + \frac {T ^ {3}}{3 !} \boldsymbol {F} _ {k} ^ {3} + \dots\tag{2.2.17}
$$

其中 $F_{k}=F(t_{k})$ 。

式 $(2.2.17)$ 就是一步转移阵的实时计算公式。

连续系统的离散化处理还包括对激励白噪声过程 $w(t)$ 的等效离散化处理。

令

$$
\boldsymbol {W} _ {k} = \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {\Phi} (t _ {k + 1}, \tau) \boldsymbol {G} (\tau) w (\tau) \mathrm{d} \tau\tag{2.2.18}
$$

则式 $(2.2.15)$ 可写成

$$
\boldsymbol {X} (t _ {k + 1}) = \boldsymbol {\Phi} (t _ {k + 1}, t _ {k}) \boldsymbol {X} (t _ {k}) + \boldsymbol {W} _ {k}
$$

简写成

$$
\boldsymbol {X} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {X} _ {k} + \boldsymbol {W} _ {k}\tag{2.2.19}
$$

下面考察由式(2.2.18)定义的 $W_{k}$ 。

$$
E [ \pmb {W} _ {k} ] = \int_ {t _ {k}} ^ {t _ {k + 1}} \pmb {\Phi} (t _ {k + 1}, \tau) \pmb {G} (\tau) E [ \pmb {w} (\tau) ] \mathrm{d} \tau = \mathbf {0}\tag{2.2.20}
$$

$$
\begin{array}{r l} E [ \boldsymbol {W} _ {k} \boldsymbol {W} _ {j} ^ {\mathrm{T}} ] & = E \left[ \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {\Phi} (t _ {k + 1}, t) \boldsymbol {G} (t) w (t) \mathrm{d} t \cdot \int_ {t _ {j}} ^ {t _ {j + 1}} w ^ {\mathrm{T}} (\tau) \boldsymbol {G} ^ {\mathrm{T}} (\tau) \boldsymbol {\Phi} ^ {\mathrm{T}} (t _ {k + 1}, \tau) \mathrm{d} \tau \right] = \\ & \quad \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {\Phi} (t _ {k + 1}, t) \boldsymbol {G} (t) \left[ \int_ {t _ {j}} ^ {t _ {j + 1}} E [ w (t) w ^ {\mathrm{T}} (\tau) ] \cdot \boldsymbol {G} ^ {\mathrm{T}} (\tau) \boldsymbol {\Phi} ^ {\mathrm{T}} (t _ {k + 1}, \tau) \mathrm{d} \tau \right] \mathrm{d} t = \\ & \quad \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {\Phi} (t _ {k + 1}, t) \boldsymbol {G} (t) \left[ \int_ {t _ {j}} ^ {t _ {j + 1}} q \delta (t - \tau) \boldsymbol {G} ^ {\mathrm{T}} (\tau) \boldsymbol {\Phi} (t _ {k + 1}, \tau) \mathrm{d} \tau \right] \mathrm{d} t \end{array}
$$

式中， $\delta(t - \tau)$ 为狄拉克 $\delta$ 函数， $t \in [t_k, t_{k+1}), \tau \in [t_j, t_{j+1})$ 。很明显，如果该两区间不重合，即 $j \neq k$ ，则 $t$ 和 $\tau$ 就不可能有相等的机会，此时积分值恒为零。而当两区间重合时，即 $j = k$ 时

$$
E \left[ \boldsymbol {W} _ {k} \boldsymbol {W} _ {j} ^ {\mathrm{T}} \right] | _ {j = k} = \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {\Phi} (t _ {k + 1}, t) \boldsymbol {G} (t) \boldsymbol {q} \boldsymbol {G} ^ {\mathrm{T}} (t) \boldsymbol {\Phi} ^ {\mathrm{T}} (t _ {k + 1}, t) \mathrm{d} t
$$

所以

$$
E [ \boldsymbol {W} _ {k} \boldsymbol {W} _ {j} ^ {\mathrm{T}} ] = \boldsymbol {Q} _ {k} \delta_ {k j}\tag{2.2.21}
$$

其中

$$
\delta_ {k j} = \left\{ \begin{array}{l l} 1, & j = k \\ 0, & j \neq k \end{array} \right.
$$

$$
\boldsymbol {Q} _ {k} = \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {\Phi} (t _ {k + 1}, t) \boldsymbol {G} (t) \boldsymbol {q} \boldsymbol {G} ^ {\mathrm{T}} (t) \boldsymbol {\Phi} ^ {\mathrm{T}} (t _ {k + 1}, t) \mathrm{d} t\tag{2.2.22}
$$

由式(2.2.20)和式(2.2.21)知， $W_{k}$ 为白噪声序列，所以式(2.2.19)描述的等效离散系统满足式(2.2.3)所示离散型卡尔曼滤波基本方程的要求。 $W_{k}$ 的方差阵 $Q_{k}$ 按式(2.2.22)求取。但若直接按式(2.2.22)计算 $Q_{k}$ ，在计算机上执行还存在诸多不便。下面通过对式(2.2.22)作恒等变形，推导出便于在计算机上执行的递推算法。

假设滤波周期 $T(T = t_{k + 1} - t_k)$ 较短， $\mathbf{G}(t)$ 在该周期内变化不大，即有

$$
\mathbf {G} (t) \approx \mathbf {G} _ {k} \qquad t _ {k} \leqslant t <   t _ {k + 1}
$$

式中 $\pmb{G}_k$ 是 $\pmb{G}(t_k)$ 的简写。记

$$
\bar {Q} = G _ {k} q G _ {k} ^ {\mathrm{T}}\tag{2.2.23}
$$

则由式(2.2.17)和式(2.2.22)有

$$
\begin{array} { r l }Q _ { k }&{ } = \int _ { t _ { k } } ^ { t _ { k + 1 } } \left[ I + F _ { k } ( t _ { k + 1 } - t ) + \frac { F _ { k } ^ { 2 } } { 2 ! } ( t _ { k + 1 } - t ) ^ { 2 } + \frac { F _ { k } ^ { 3 } } { 3 ! } ( t _ { k + 1 } - t ) ^ { 3 } + \cdots \right] \cdot\\&{ } \bar { Q } \left[ I + F _ { k } ( t _ { k + 1 } - t ) + \frac { F _ { k } ^ { 2 } } { 2 ! } ( t _ { k + 1 } - t ) ^ { 2 } + \frac { F _ { k } ^ { 3 } } { 3 ! } ( t _ { k + 1 } - t ) ^ { 3 } + \dots \right] ^ { T } d t =\\&{ } \int _ { t _ { k } } ^ { t _ { k + 1 } } \left[ \bar { Q } + \bar { Q } F _ { k } ^ { T } ( t _ { k + 1 } - t ) + \frac { 1 } { 2 ! } \bar { Q } F _ { k } ^ { 2 T } ( t _ { k + 1 } - t ) ^ { 2 } + \right.\\&{ } \frac { 1 } { 3 ! } \bar { Q } F _ { k } ^ { 3 T } ( t _ { k + 1 } - t ) ^ { 3 } + \cdots + F _ { k } \bar { Q } ( t _ { k + 1 } - t ) +\\&{ } F _ { k } \bar { Q } F _ { k } ^ { T } ( t _ { k + 1 } - t ) ^ { 2 } + \frac { 1 } { 2 ! } F _ { k } \bar { Q } F _ { k } ^ { 2 T } ( t _ { k + 1 } - t ) ^ { 3 } + \cdots +\\&{ } \frac { 1 } { 2 ! } F _ { k } ^ { 2 } \bar { Q } ( t _ { k + 1 } - t ) ^ { 2 } + \frac { 1 } { 2 ! } F _ { k } ^ { 2 } \bar { Q } F _ { k } ^ { T } ( t _ { k + 1 } - t ) ^ { 3 } + \cdots +\\&{ } \frac { 1 } { 3 ! } F _ { k } ^ { 3 } \bar { Q } ( t _ { k + 1 } - t ) ^ { 3 } + \frac { 1 } { 3 ! } F _ { k } ^ { 3 } \bar { Q } F _ { k } ^ { T } ( t _ { k + 1 } - t ) ^ { 4 } + \cdots ] d t =\\&{ } \int _ { t _ { k } } ^ { t _ { k + 1 } } \left\{ \bar { Q } + ( \bar { Q } F _ { k } ^ { T } + F _ { k } \bar { Q} ) ( t _ { k + 1 } - t ) + [ \frac { 1 } { 2 ! } ( \bar { Q } F _ { k } ^ { 2 T } + F _ { k } ^ { 2 } \bar { Q } + F _ { k } \bar { Q} F _ { k } ^ { T} ] ( t _ { k + 1 } - t ) ^ { 2 } + [ \frac { 1 } { 3 !} ( \bar { Q } F _ { k } ^ { 3 T } + F _ { k } ^ { 3 } \bar { Q} ) + \right.\\&{ } \left. \frac { 1 } { 2 ! } ( F _ { k } \bar { Q } F _ { k } ^ { 2 T } + F _ { k } ^ { 2 } \bar { Q } F _ { k } ^ { T} ) ] ( t _ { k + 1 } - t ) ^ { 3 } + \cdots \right\} d t =\\&{ } \bar { Q T} + \frac { T ^ { 2 }}{ 2 !} \left[ \frac { 1}{ 1 !} ( \bar { Q F _ { k }} ^ { T } + F _ { k} \bar { Q} ) \right] + \frac { T ^ { 3 }}{ 3 !} \left[ \frac { 1}{ 2 !} ( \bar { Q F _ { k }} ^ { 2 T } + F _ { k} ^ { 2} \bar { Q} ) + \right.\\&{ } \left. \frac { 1}{ 1 !   1 !} F _ { k} \bar { Q F _ { k }} ^ { T} \right] + \frac { T ^ { 4 }}{ 4 !} \left[ \frac { 1}{ 3 !} ( \bar { Q F _ { k }} ^ { 3 T } + F _ { k} ^ { 3} \bar { Q} ) + \right.\\&{ } \left. \right. \frac { 1}{ 2 !   1 !} ( F _ { k} \bar { Q F _ { k }} ^ { 2 T } + F _ { k} ^ { 2} \bar { Q F _ { k }} ^ { T} ] + \frac { T ^ { 5 }}{ 5 !} \left[ \frac { 1}{ 4 !} ( \bar { Q F _ { k }} ^ { 4 T } + F _ { k} ^ { 4} \bar { Q} ) + \right.\\&{ } \left. \right. \frac { 1}{ 3 !   1 !} ( F _ { k} \bar { Q F _ { k }} ^ { 3 T } + F _ { k} ^ { 3} \bar { Q F _ { k }} ^ { T} ) + \frac { 1}{ 2 !   2 !} F _ { k} ^ { 2} \bar { Q F _ { k }} ^ { 2 T} ] + \cdots =\\&{\cal T Q} + \frac {\cal T ^ { 2 }}{2 !} ( \bar {\cal Q} F _ {\mathrm{k}} ^ {\mathrm{T}} + F _ {\mathrm{k}} \bar {\cal Q}) + \frac {\cal T ^ {\mathrm{3}}}{3 !} ( \bar {\cal Q} F _ {\mathrm{k}} ^ {\mathrm{2T}} + F _ {\mathrm{k}} ^ {\mathrm{2}} \bar {\cal Q} +\\&\cal 2 F _ {\mathrm{k}} \bar {\cal Q} F _ {\mathrm{k}} ^ {\mathrm{T}}) + \frac {\cal T ^ {\mathrm{4}}}{4 !} ( \bar {\cal Q} F _ {\mathrm{k}} ^ {\mathrm{3T}} + F _ {\mathrm{k}} ^ {\mathrm{3}} \bar {\cal Q} + {\cal 3 F _ {\mathrm{k}} \bar {\cal Q} F _ {\mathrm{k}} ^ {\mathrm{2T}}} +\\&\cal 3 F _ {\mathrm{k}} ^ {\mathrm{2}} \bar {\cal Q} F _ {\mathrm{k}} ^ {\mathrm{T}}) + \frac {\cal T ^ {\mathrm{5}}}{5 !} ( \bar {\cal Q} F _ {\mathrm{k}} ^ {\mathrm{4T}} + F _ {\mathrm{k}} ^ {\mathrm{4}} \bar {\cal Q} + {\cal 4 F _ {\mathrm{k}} \bar {\cal Q} F _ {\mathrm{k}} ^ {\mathrm{3T}}} +\\&\cal 4 F _ {\mathrm{k}} ^ {\mathrm{3}} \bar {\cal Q} F _ {\mathrm{k}} ^ {\mathrm{T}} + {\cal 6 F _ {\mathrm{k}} ^ {\mathrm{2}} \bar {\cal Q} F _ {\mathrm{k}} ^ {\mathrm{2T}}} ) + \cdots .\end{array}
$$

记

$$
\boldsymbol {M} _ {1} = \bar {\boldsymbol {Q}}\tag{2.2.24}
$$

$$
\boldsymbol {M} _ {2} = \overline {{\boldsymbol {Q}}} \boldsymbol {F} _ {k} ^ {\mathrm{T}} + \boldsymbol {F} _ {k} \overline {{\boldsymbol {Q}}}\tag{2.2.25}
$$

$$
\boldsymbol {M} _ {3} = \overline {{\boldsymbol {Q}}} \boldsymbol {F} _ {k} ^ {2 \mathrm{T}} + \boldsymbol {F} _ {k} ^ {2} \overline {{\boldsymbol {Q}}} + 2 \boldsymbol {F} _ {k} \overline {{\boldsymbol {Q}}} \boldsymbol {F} _ {k} ^ {\mathrm{T}}\tag{2.2.26}
$$

$$
\boldsymbol {M} _ {4} = \overline {{\boldsymbol {Q}}} \boldsymbol {F} _ {k} ^ {3 \mathrm{T}} + \boldsymbol {F} _ {k} ^ {3} \overline {{\boldsymbol {Q}}} + 3 \boldsymbol {F} _ {k} \overline {{\boldsymbol {Q}}} \boldsymbol {F} _ {k} ^ {2 \mathrm{T}} + 3 \boldsymbol {F} _ {k} ^ {2} \overline {{\boldsymbol {Q}}} \boldsymbol {F} _ {k} ^ {\mathrm{T}}\tag{2.2.27}
$$

$$
\boldsymbol {M} _ {5} = \overline {{\boldsymbol {Q}}} \boldsymbol {F} _ {k} ^ {4 \mathrm{T}} + \boldsymbol {F} _ {k} ^ {4} \overline {{\boldsymbol {Q}}} + 4 \boldsymbol {F} _ {k} \overline {{\boldsymbol {Q}}} \boldsymbol {F} _ {k} ^ {3 \mathrm{T}} + 4 \boldsymbol {F} _ {k} ^ {3} \overline {{\boldsymbol {Q}}} \boldsymbol {F} _ {k} ^ {\mathrm{T}} + 6 \boldsymbol {F} _ {k} ^ {2} \overline {{\boldsymbol {Q}}} \boldsymbol {F} _ {k} ^ {2 \mathrm{T}}\tag{2.2.28}
$$

则

$$
Q _ {k} = \frac {T}{1 !} M _ {1} + \frac {T ^ {2}}{2 !} M _ {2} + \frac {T ^ {3}}{3 !} M _ {3} + \frac {T ^ {4}}{4 !} M _ {4} + \frac {T ^ {5}}{5 !} M _ {5} + \dots\tag{2.2.29}
$$

而

$$
\begin{array}{r l} & {\mathbf {F} _ {k} \mathbf {M} _ {1} + (\mathbf {F} _ {k} \mathbf {M} _ {1}) ^ {\mathrm{T}} = \mathbf {F} _ {k} \overline {{\mathbf {Q}}} + \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {\mathrm{T}}} \\ & {\mathbf {F} _ {k} \mathbf {M} _ {2} + (\mathbf {F} _ {k} \mathbf {M} _ {2}) ^ {\mathrm{T}} = \mathbf {F} _ {k} \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {\mathrm{T}} + \mathbf {F} _ {k} ^ {2} \overline {{\mathbf {Q}}} + (\mathbf {F} _ {k} \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {\mathrm{T}} + \mathbf {F} _ {k} ^ {2} \overline {{\mathbf {Q}}}) ^ {\mathrm{T}} =} \\ & {\quad \mathbf {F} _ {k} ^ {2} \overline {{\mathbf {Q}}} + \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {2 \mathrm{T}} + 2 \mathbf {F} _ {k} \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {\mathrm{T}}} \\ & {\mathbf {F} _ {k} \mathbf {M} _ {3} + (\mathbf {F} _ {k} \mathbf {M} _ {3}) ^ {\mathrm{T}} = \mathbf {F} _ {k} \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {2 \mathrm{T}} + \mathbf {F} _ {k} ^ {3} \overline {{\mathbf {Q}}} + 2 \mathbf {F} _ {k} ^ {2} \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {\mathrm{T}} +} \\ & {\quad (\mathbf {F} _ {k} \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {2 \mathrm{T}} + \mathbf {F} _ {k} ^ {3} \overline {{\mathbf {Q}}} + 2 \mathbf {F} _ {k} ^ {2} \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {- 1}) ^ {\mathrm{T}} =} \\ & {\quad \mathbf {F} _ {k} ^ {3} \overline {{\mathbf {Q}}} + \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {3 \mathrm{T}} + 3 \mathbf {F} _ {k} ^ {2} \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {- 1} + 3 \mathbf {F} _ {k} \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {- 2 \mathrm{T}}} \\ & {\mathbf {F} _ {k} \mathbf {M} _ {4} + (\mathbf {F} _ {k} \mathbf {M} _ {4}) ^ {\mathrm{T}} = \mathbf {F} _ {k} \overline {{\mathbf {Q}}} \mathbf {F} _ {k} ^ {- 3 \mathrm{T}} + \mathbf {F} _ {k} ^ {- 4} \overline {{\mathbf {Q}}} + 3 \mathbf {F} _ {k} ^ {- 2} \overline {{\mathbf {Q}}} \mathbf {F} _ {- k} ^ {- 2 \mathrm{T}} + 3 \mathbf {F} _ {- k} ^ {- 3} \overline {{\mathbf {Q}}} \mathbf {F} _ {- k} ^ {- 1}} \\ & {\quad (\mathbf {F} _ {- k} \overline {{\mathbf {Q}}} \mathbf {F} _ {- k} ^ {- 3 \mathrm{T}} + \mathbf {F} _ {- k} ^ {- 4} \overline {{\mathbf {Q}}} + 3 \mathbf {F} _ {- k} ^ {- 2} \overline {{\mathbf {Q}}} \mathbf {F} _ {- k} ^ {- 2 \mathrm{T}} + 3 \mathbf {F} _ {- k} ^ {- 3} \overline {{\mathbf {Q}}} \mathbf {F} _ {- k}) ^ {\mathrm{T}} =} \\ & {\quad \mathbf {F} _ {- k} ^ {- 4} \overline {{\mathbf {Q}}} + \overline {{\mathbf {Q}}} \mathbf {F} _ {- k} ^ {- 4 \mathrm{T}} + 4 \mathbf {F} _ {- k} \overline {{\mathbf {Q}}} \mathbf {F} _ {- k - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1 - 1}} \\ & {\dots ..}. \end{array}
$$

上面诸式分别与式(2.2.24)～式(2.2.28)相比较，得

$$
\begin{array}{r l} & {\pmb {M} _ {2} = \pmb {F} _ {k} \pmb {M} _ {1} + (\pmb {F} _ {k} \pmb {M} _ {1}) ^ {\mathrm{T}}} \\ & {\pmb {M} _ {3} = \pmb {F} _ {k} \pmb {M} _ {2} + (\pmb {F} _ {k} \pmb {M} _ {2}) ^ {\mathrm{T}}} \\ & {\pmb {M} _ {4} = \pmb {F} _ {k} \pmb {M} _ {3} + (\pmb {F} _ {k} \pmb {M} _ {3}) ^ {\mathrm{T}}} \\ & {\pmb {M} _ {5} = \pmb {F} _ {k} \pmb {M} _ {4} + (\pmb {F} _ {k} \pmb {M} _ {4}) ^ {\mathrm{T}}} \\ & {\dots \dots} \end{array}
$$

即

$$
\boldsymbol {M} _ {i + 1} = \boldsymbol {F} _ {k} \boldsymbol {M} _ {i} + (\boldsymbol {F} _ {k} \boldsymbol {M} _ {i}) ^ {\mathrm{T}} \quad i = 1, 2, 3, \dots\tag{2.2.30}
$$

式中 $M_{1}$ 按式(2.2.24) 确定。

由式 $(2.2.29)$ 可得

$$
Q _ {k} = \sum_ {i = 1} ^ {\infty} M _ {i} \frac {T ^ {i}}{i !}\tag{2.2.31}
$$

式中 $M_{i}$ 按式(2.2.30)递推计算。

按式(2.2.17)计算一步转移阵和按式(2.2.31)计算等效离散系统噪声方差阵可用图2.2.3所示计算框图表示。

如果滤波周期较长,且系统矩阵 $F(t)$ 随时间变化较剧烈,而 $F(t)$ 在一个滤波周期内能得到 N 个采样值,则计算一步转移阵和等效离散系统噪声方差阵时可利用这些采样值,以简化算法。

设滤波周期为 $T(T = t_{k + 1} - t_k)$ ，在 $[t_k,t_{k + 1})$ 内每隔 $\Delta T = \frac{T}{N}$ 就能得到系统阵的采样值 $\pmb {F}(t_k + i\Delta T)$ ，并记

$$
\boldsymbol {F} _ {k} (i) = \boldsymbol {F} (t _ {k} + i \Delta T) \quad i = 0, 1, 2, \dots , N - 1
$$

则 $t_{k+1}=t_{k}+N\Delta T$ ，并且一步转移阵 $\Phi_{k+1,k}$ 和等效离散系统噪声方程阵 $Q_{k}$ 可按如下方法作简化计算。

![](images/5f0ba5ef7c81422f4f1cdb87598aa59ac80e89fdc076174982e3f0cf0fd01c3b.jpg)  
图 2.2.3 一步转移阵和等效离散系统噪声方差阵计算框图

(1) 一步转移阵的简化算法。由于

$$
\begin{array}{r l} \boldsymbol {\Phi} _ {k + 1, k} & = \boldsymbol {\Phi} [ t _ {k} + N \Delta T, t _ {k} + (N - 1) \Delta T ] \cdot \boldsymbol {\Phi} [ t _ {k} + (N - 1) \Delta T, t _ {k} + (N - 2) \Delta T ] \dots \\ & \boldsymbol {\Phi} [ t _ {k} + \Delta T, t _ {k} ] \end{array}
$$

$\Phi[t_{k}+(i+1)\Delta T,t_{k}+i\Delta T]$ ( $i=0,1,2,\cdots,N-1$ ) 的转移时间间隔为 $\Delta T=\frac{T}{N}$ ，若 N 很大，则 $\Delta T$ 很小，该转移阵的收敛速度远比 $\Phi_{k+1,k}$ 快，级数中关于 $\Delta T$ 的高次项可略去不计，所以

$$
\begin{array}{r l} \boldsymbol {\Phi} _ {k + 1, k} & \approx [ \boldsymbol {I} + \Delta T \boldsymbol {F} _ {k} (N - 1) ] [ \boldsymbol {I} + \Delta T \boldsymbol {F} _ {k} (N - 2) ] \dots \\ & [ \boldsymbol {I} + \Delta T \boldsymbol {F} _ {k} (0) ] = \boldsymbol {I} + \Delta T \sum_ {i = 0} ^ {N - 1} \boldsymbol {F} _ {k} (i) \end{array}\tag{2.2.32}
$$

式中略去了乘积中关于 $\Delta T$ 的高次项。

(2) 等效离散系统噪声方差阵的简化算法。

由式 $(2.2.22)$

$$
Q _ {k} = \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {\Phi} (t _ {k + 1}, t) \boldsymbol {G} (t) \boldsymbol {q} \boldsymbol {G} ^ {\mathrm{T}} (t) \boldsymbol {\Phi} ^ {\mathrm{T}} (t _ {k + 1}, t) \mathrm{d} t
$$

在时间段 $[t_k, t_{k+1})$ 内，取 $G(t) \approx G(t_k)$ ，并记

$$
\bar {Q} = G (t _ {k}) q G ^ {\mathrm{T}} (t _ {k})
$$

则

$$
Q _ {k} = \int_ {t _ {k}} ^ {t _ {k + 1}} \boldsymbol {\Phi} (t _ {k + 1}, t) \overline {{\boldsymbol {Q}}} \boldsymbol {\Phi} ^ {\mathrm{T}} (t _ {k + 1}, t) \mathrm{d} t = \sum_ {i = 0} ^ {N - 1} \int_ {t _ {k} + i \Delta T} ^ {t _ {k} + (i + 1) \Delta T} \boldsymbol {\Phi} (t _ {k + 1}, t) \overline {{\boldsymbol {Q}}} \boldsymbol {\Phi} ^ {\mathrm{T}} (t _ {k + 1, t}) \mathrm{d} t\tag{2.2.33}
$$

记

$$
\boldsymbol {I} (i) = \int_ {t _ {k} + i \Delta T} ^ {t _ {k} + (i + 1) \Delta T} \boldsymbol {\Phi} (t _ {k + 1}, t) \overline {{\boldsymbol {Q}}} \boldsymbol {\Phi} ^ {\mathrm{T}} (t _ {k + 1}, t) \mathrm{d} t
$$

由于式中

$$
\begin{array}{r l} \boldsymbol {\Phi} (t _ {k + 1}, t) & = \boldsymbol {\Phi} [ t _ {k} + N \Delta T, t _ {k} + (N - 1) \Delta T ] \cdot \boldsymbol {\Phi} [ t _ {k} + (N - 1) \Delta T, t _ {k} + (N - 2) \Delta T ] \dots \\ & \quad \boldsymbol {\Phi} [ t _ {k} + (i + 2) \Delta T, t _ {k} + (i + 1) \Delta T ] \cdot \boldsymbol {\Phi} [ t _ {k} + (i + 1) \Delta T, t ] \approx \\ & [ I + \Delta T F _ {k} (N - 1) ] [ I + \Delta T F _ {k} (N - 2) ] \dots [ I + \Delta T F _ {k} (i + 1) ] \cdot \\ & \{I + [ t _ {k} + (i + 1) \Delta T - t ] F _ {k} (i) \} \end{array}
$$

仅保留式 $(2.2.33)$ 被积函数中关于时间间隔的线性项，则

$$
\begin{array}{r l} \boldsymbol {I} (i) & \approx \int_ {t _ {k + i \Delta T}} ^ {t _ {k + (i + 1) \Delta T}} \{\overline {{Q}} + \Delta T [ F _ {k} (N - 1) + F _ {k} (N - 2) + \dots + \\ & \quad F _ {k} (i + 1) ] \overline {{Q}} + [ t _ {k} + (i + 1) \Delta T - t ] F _ {k} (i) \overline {{Q}} + \\ & \quad \Delta T \overline {{Q}} [ F _ {k} ^ {\mathrm{T}} (N - 1) + F _ {k} ^ {\mathrm{T}} (N - 2) + \dots + F _ {k} ^ {\mathrm{T}} (i + 1) ] + \\ & \quad \overline {{Q}} [ t _ {k} + (i + 1) \Delta T - t ] F _ {k} ^ {\mathrm{T}} (i) \} \mathrm{d} t = \\ & \quad \overline {{Q}} \Delta T + \Delta T ^ {2} \{[ F _ {k} (N - 1) + F _ {k} (N - 2) + \dots + F _ {k} (i + 1) ] \overline {{Q}} + \\ & \quad \overline {{Q}} [ F _ {k} ^ {\mathrm{T}} (N - 1) + F _ {k} ^ {\mathrm{T}} (N - 2) + \dots + F _ {k} ^ {\mathrm{T}} (i + 1) ] \} + \\ & \quad \frac {\Delta T ^ {2}}{2} F _ {k} (i) \overline {{Q}} + \frac {\Delta T ^ {2}}{2} \overline {{Q}} F _ {k} ^ {\mathrm{T}} (i) \qquad i = 0, 1, 2, \dots , N - 1 \end{array}
$$

由上式得

$$
\begin{array}{r l} & {\pmb {I} (0) = \overline {{{\pmb {Q}}}} \Delta T + \Delta T ^ {2} \left\{\left[ \pmb {F} _ {k} (N - 1) + \pmb {F} _ {k} (N - 2) + \dots + \pmb {F} _ {k} (1) + \frac {1}{2} \pmb {F} _ {k} (0) \right] \overline {{{\pmb {Q}}}} + \right.} \\ & {\qquad \left. \overline {{{\pmb {Q}}}} \left[ \pmb {F} _ {k} ^ {\mathrm{T}} (N - 1) + \pmb {F} _ {k} ^ {\mathrm{T}} (N - 2) + \dots + \pmb {F} _ {k} ^ {\mathrm{T}} (1) + \frac {1}{2} \pmb {F} _ {k} ^ {\mathrm{T}} (0) \right] \right\}} \\ & {\pmb {I} (1) = \overline {{{\pmb {Q}}}} \Delta T + \Delta T ^ {2} \left\{\left[ \pmb {F} _ {k} (N - 1) + \pmb {F} _ {k} (N - 2) + \dots + \pmb {F} _ {k} (2) + \frac {1}{2} \pmb {F} _ {k} (1) \right] \overline {{{\pmb {Q}}}} + \right.} \\ & {\qquad \left. \overline {{{\pmb {Q}}}} \left[ \pmb {F} _ {k} ^ {\mathrm{T}} (N - 1) + \pmb {F} _ {k} ^ {\mathrm{T}} (N - 2) + \dots + \pmb {F} _ {k} (\mathrm{2}) + \frac {1}{2} \pmb {F} _ {k} ^ {\mathrm{T}} (1) \right] \right\}} \end{array}
$$

$$
\begin{array}{r l} & {\pmb {I} (N - 2) = \overline {{{\pmb {Q}}}} \Delta T + \Delta \pmb {T} ^ {2} \left\{\left[ \pmb {F} _ {k} (N - 1) + \frac {1}{2} \pmb {F} _ {k} (N - 2) \right] \overline {{{\pmb {Q}}}} + \right.} \\ & {\qquad \left. \overline {{{\pmb {Q}}}} \left[ \pmb {F} _ {k} ^ {\mathrm{T}} (N - 1) + \frac {1}{2} \pmb {F} _ {k} ^ {\mathrm{T}} (N - 2) \right] \right\}} \\ & {\pmb {I} (N - 1) = \overline {{{\pmb {Q}}}} \Delta T + \Delta T ^ {2} \left[ \frac {1}{2} \pmb {F} _ {k} (N - 1) \overline {{{\pmb {Q}}}} + \frac {1}{2} \overline {{{\pmb {Q}}}} \pmb {F} _ {k} ^ {\mathrm{T}} (n - 1) \right]} \end{array}
$$

所以

$$
\begin{array}{r l} & {\pmb {Q} _ {k} = \pmb {I} (0) + \pmb {I} (1) + \dots + \pmb {I} (N - 2) + \pmb {I} (N - 1) = N \Delta T \overline {{{\pmb {Q}}}} +} \\ & {\qquad \Delta T ^ {2} \left\{\left[ \left(N - 1 + \frac {1}{2}\right) \pmb {F} _ {k} (N - 1) + \left(N - 2 + \frac {1}{2}\right) \pmb {F} _ {k} (N - 2) + \dots + \right. \right.} \end{array}
$$

$$
\begin{array}{l} \left(1 + \frac {1}{2}\right) \boldsymbol {F} _ {k} (1) + \frac {1}{2} \boldsymbol {F} _ {k} (0) \Big ] \overline {{\boldsymbol {Q}}} + \overline {{\boldsymbol {Q}}} \Big [ \left(N - 1 + \frac {1}{2}\right) \boldsymbol {F} _ {k} ^ {\mathrm{T}} (N - 1) + \\ \left(N - 1 + \frac {1}{2}\right) \boldsymbol {F} _ {k} ^ {\mathrm{T}} (N - 2) + \dots + \left(1 + \frac {1}{2}\right) \boldsymbol {F} _ {k} ^ {\mathrm{T}} (1) + \frac {1}{2} \boldsymbol {F} _ {k} ^ {\mathrm{T}} (0) \Big ] \Big \} = \\ T \overline {{\boldsymbol {Q}}} + \Delta T ^ {2} \left\{\left[ \sum_ {i = 0} ^ {N - 1} \left(i + \frac {1}{2}\right) \boldsymbol {F} _ {k} (i) \right] \overline {{\boldsymbol {Q}}} + \overline {{\boldsymbol {Q}}} \left[ \sum_ {i = 0} ^ {N - 1} \left(i + \frac {1}{2}\right) \boldsymbol {F} _ {k} ^ {\mathrm{T}} (i) \right] \right\} \end{array}\tag{2.2.34}
$$

考察式(2.2.32)和式(2.2.34)可看出,计算一步转移阵 $\Phi_{k+1,k}$ 所需的矩阵运算仅需作N步矩阵求和,计算等效离散系统噪声方差阵所需的矩阵运算为作N步矩阵求和后作一步矩阵相乘,与式(2.2.31)相比计算量大为减少,这对工程实现十分有利。此外,与式(2.2.17)和式(2.2.31)仅用到 $t_{k}$ 时刻系统的动态信息相比,简化算法中利用了 $[t_{k},t_{k+1})$ 内N次采样获得的系统动态信息,这在一定程度上弥补了级数过早截断引起的精度损失。

## 4. 系统有确定性控制时的滤波基本方程

设系统的状态方程除有白噪声驱动项外,还有确定性驱动项

$$
\boldsymbol {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {X} _ {k - 1} + \boldsymbol {B} _ {k - 1} \boldsymbol {u} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1}
$$

根据线性最小方差估计的性质3，得

$$
\hat {\boldsymbol {X}} _ {k, k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1} + \boldsymbol {B} _ {k - 1} \boldsymbol {u} _ {k - 1}\tag{2.2.35}
$$

所以状态估计方程为

$$
\hat {\boldsymbol {X}} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1} + \boldsymbol {B} _ {k - 1} \boldsymbol {u} _ {k - 1} + \boldsymbol {K} _ {k} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1} - \boldsymbol {H} _ {k} \boldsymbol {B} _ {k - 1} \boldsymbol {u} _ {k - 1})\tag{2.2.36}
$$

式中 $K_{k}, P_{k/k-1}, P_{k}$ 的计算与式(2.2.4)相同。

## 5. 一步预测基本方程

一步预测基本方程是指利用 $Z_{k}$ 和 $\hat{X}_{k/k-1}$ 递推计算 $\hat{X}_{k+1/k}$ 的全套方程。由式(2.2.4a)和式(2.2.4b)得

$$
\hat {\mathbf {X}} _ {k + 1 / k} = \boldsymbol {\Phi} _ {k + 1, k} \hat {\mathbf {X}} _ {k} = \boldsymbol {\Phi} _ {k + 1, k} \left[ \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} \left(\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1}\right) \right]
$$

令

$$
\boldsymbol {K} _ {k} ^ {*} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {K} _ {k}\tag{2.2.37}
$$

则

$$
\hat {\mathbf {X}} _ {k + 1 / k} = \boldsymbol {\Phi} _ {k + 1, k} \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} ^ {*} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1})\tag{2.2.38}
$$

式中

$$
\boldsymbol {K} _ {k} ^ {*} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1}\tag{2.2.39}
$$

根据式(2.2.4d)、式(2.2.4e')和式(2.2.37)，得

$$
\boldsymbol {P} _ {k + 1, k} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k} \boldsymbol {Q} _ {k} \boldsymbol {\Gamma} _ {k} ^ {\mathrm{T}} = \boldsymbol {\Phi} _ {k + 1, k} (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k} \boldsymbol {Q} _ {k} \boldsymbol {\Gamma} _ {k} ^ {\mathrm{T}} =
$$

$$
\left(\boldsymbol {\Phi} _ {k + 1, k} - \boldsymbol {K} _ {k} ^ {*} \boldsymbol {H} _ {k}\right) \boldsymbol {P} _ {k / k - 1} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k} \boldsymbol {Q} _ {k} \boldsymbol {\Gamma} _ {k} ^ {\mathrm{T}}\tag{2.2.40}
$$

式 $(2.2.38)$ 、式 $(2.2.39)$ 和式 $(2.2.40)$ 即为一步预测基本方程。

例2-12 设有线性定常系统

$$
\begin{array}{r} X _ {k} = \Phi X _ {k - 1} + W _ {k - 1} \\ Z _ {k} = X _ {k} + V _ {k} \end{array}
$$

式中状态变量 $X_{k}$ 与量测 $Z_{k}$ 均为标量， $\Phi$ 为常数。 $\{W_{k}\}$ 和 $\{V_{k}\}$ 为零均值的白噪声序列，分别具有协方差

$$
E [ W _ {k} W _ {j} ] = Q \delta_ {k j}, E [ V _ {k} V _ {j} ] = R \delta_ {k j}
$$

并且 $\{W_k\}, \{V_k\}, X_0$ 三者互不相关。求 $\hat{X}_k$ 的递推方程。

解 根据式 $(2.2.4)$ ，得如下关系式

$$
\hat {X} _ {k / k - 1} = \Phi \hat {X} _ {k - 1}\tag{1}
$$

$$
\hat {X} _ {k} = \hat {X} _ {k / k - 1} + K _ {k} (Z _ {k} - \hat {X} _ {k / k - 1}) = (1 - K _ {k}) \hat {X} _ {k / k - 1} + K _ {k} Z _ {k}\tag{2}
$$

$$
K _ {k} = P _ {k / k - 1} (P _ {k / k - 1} + R) ^ {- 1} = \frac {P _ {k / k - 1}}{P _ {k / k - 1} + R}\tag{3}
$$

$$
P _ {k / k - 1} = \Phi^ {2} P _ {k - 1} + Q\tag{4}
$$

$$
P _ {k} = (1 - K _ {k}) P _ {k / k - 1} = \left(1 - \frac {P _ {k / k - 1}}{P _ {k / k - 1} + R}\right) P _ {k / k - 1} = \frac {R P _ {k / k - 1}}{P _ {k / k - 1} + R} = R K _ {k}\tag{5}
$$

由式(2)知， $K_{k}$ 实际上决定了对量测值 $Z_{k}$ 和上一步估计值 $\hat{X}_{k-1}$ 利用的比例程度。若 $K_{k}$ 增大，则 $Z_{k}$ 的利用权重增大，而 $\hat{X}_{k-1}$ 的利用权重相对降低。又由式(3)和式(4)知， $K_{k}$ 由量测噪声方差 R 和上一步估计的均方误差 $P_{k-1}$ 决定。假设 Q 一定，k 时刻的估计精度较高，由(4)式确定的 $P_{k/k-1}$ 较小，若量测精度很差，即 R 很大，则 $K_{k}$ 很小，结果是对 $Z_{k}$ 的利用权重减小，而对 $\hat{X}_{k-1}$ 的利用权重增大。若 $\hat{X}_{k-1}$ 精度很差，即 $P_{k-1}$ 很大，而量测精度很高，即 R 很小，则 $K_{k}$ 变大，确定 $\hat{X}_{k}$ 时对 $Z_{k}$ 的利用权重增大，而对 $\hat{X}_{k-1}$ 的利用权重相对减小。因此卡尔曼滤波能定量识别各种信息的质量，自动确定对这些信息的利用程度，是一种具有一定智能功能的算法。

例2-13 $\alpha -\beta -\gamma$ 滤波。

设运动体沿某一直线运动，t 时刻的位移、速度、加速度和加加速度分别为 $s(t)$ ， $v(t)$ ， $a(t)$ ， $j(t)$ ， $t_{k}$ 时刻的量测为 $Z_{k}=s_{k}+V_{k}$ ，其中 $s_{k}=s(t_{k})$ ， $k=1,2,3,\cdots$ 。运动体机动运动引起的运动不定性用 $j(t)$ 描述，其中

$$
E [ j (t) ] = 0, \quad E [ j (t) j (\tau) ] = q \delta (t - \tau)
$$

$$
E [ V _ {k} ] = 0, \quad E [ V _ {k} V _ {l} ] = r \delta_ {k l}
$$

量测量的采样周期为 T。求对 $s_{k}, v_{k}, a_{k}$ 的估计。

解 根据 $\dot{s}(t)=v(t)$ , $\dot{v}(t)=a(t)$ , $\dot{a}(t)=j(t)$ , 可列写出状态方程:

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {F} \boldsymbol {X} (t) + \boldsymbol {G} j (t)
$$

其中

$$
\mathbf {X} (t) = \left[ \begin{array}{l} s (t) \\ v (t) \\ a (t) \end{array} \right], \quad \mathbf {F} = \left[ \begin{array}{l l l} 0 & 1 & 0 \\ 0 & 0 & 1 \\ 0 & 0 & 0 \end{array} \right], \quad \mathbf {G} = \left[ \begin{array}{l} 0 \\ 0 \\ 1 \end{array} \right]
$$

对上式作离散化处理,得

$$
\mathbf {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \mathbf {X} _ {k - 1} + \int_ {t _ {k - 1}} ^ {t _ {k}} \boldsymbol {\Phi} (t _ {k}, \tau) \mathbf {G} j (\tau) \mathrm{d} \tau\tag{1}
$$

其中

$$
\Phi_ {k, k - 1} = I + F T + \frac {T ^ {2}}{2 !} F ^ {2} + \frac {T ^ {3}}{3 !} F ^ {3} + \dots =
$$

$$
\left[ \begin{array}{l l l} 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{array} \right] + \left[ \begin{array}{l l l} 0 & T & 0 \\ 0 & 0 & T \\ 0 & 0 & 0 \end{array} \right] + \left[ \begin{array}{l l l} 0 & 0 & \frac {T ^ {2}}{2 !} \\ 0 & 0 & 0 \\ 0 & 0 & 0 \end{array} \right] = \left[ \begin{array}{l l l} 1 & T & \frac {T ^ {2}}{2} \\ 0 & 1 & T \\ 0 & 0 & 1 \end{array} \right]\tag{2}
$$

$$
\int_ {t _ {k - 1}} ^ {t _ {k}} \pmb {\Phi} (t _ {k}, \tau) \pmb {G} j (\tau) \mathrm{d} \tau \approx \int_ {t _ {k - 1}} ^ {t _ {k}} \pmb {\Phi} (t _ {k}, \tau) \pmb {G} \mathrm{d} \tau \cdot j _ {k - 1} =
$$

$$
\int_ {t _ {k - 1}} ^ {t _ {k}} \left[ \begin{array}{c c c} 1 & t _ {k} - \tau & \frac {(t _ {k} - \tau) ^ {2}}{2} \\ 0 & 1 & t _ {k} - \tau \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{l} 0 \\ 0 \\ 1 \end{array} \right] \mathrm{d} \tau \cdot j _ {k - 1} = \left[ \begin{array}{l} \frac {T ^ {3}}{6} \\ \frac {T ^ {2}}{2} \\ T \end{array} \right] j _ {k - 1}
$$

其中

$$
j _ {k - 1} = \frac {1}{T} \int_ {t _ {k - 1}} ^ {t _ {k}} j (\tau) \mathrm{d} \tau\tag{3}
$$

即 $j(t)$ 在 $[t_{k - 1}, t_k]$ 内的平均值。由式(3)，

$$
E [ j _ {k} ] = \frac {1}{T} \int_ {t _ {k}} ^ {t _ {k + 1}} E [ j (\tau) ] \mathrm{d} \tau = 0
$$

$$
E \left[ j _ {k} j _ {l} \right] = \frac {1}{T ^ {2}} \int_ {t _ {k}} ^ {t _ {k + 1}} \int_ {t _ {l}} ^ {t _ {l + 1}} E \left[ j (t) \cdot j (\tau) \right] \mathrm{d} t \mathrm{d} \tau = \frac {1}{T ^ {2}} \int_ {t _ {k}} ^ {t _ {k + 1}} \int_ {t _ {l}} ^ {t _ {l + 1}} q \delta (t - \tau) \mathrm{d} t \mathrm{d} \tau
$$

显然，只有当 $[t_l, t_{l+1}]$ 和 $[t_k, t_{k+1}]$ 重合时才有可能 $t = \tau$ ，所以当 $l = k$ 时，

$$
E [ j _ {k} ^ {2} ] = \frac {1}{T ^ {2}} \int_ {t _ {k}} ^ {t _ {k + 1}} q \mathrm{d} \tau = \frac {q}{T}
$$

当 $l \neq k$ 时， $E[j_{k}j_{l}] = 0$ 。可见 $E[j_{k}j_{l}] = \frac{q}{T}\delta_{kl}$ ，由式(3)确定的序列为白噪声，其方差为 $Q = \frac{q}{T}$ 。

所以式(1)可写成

$$
\boldsymbol {X} _ {k} = \boldsymbol {\Phi} \boldsymbol {X} _ {k - 1} + \boldsymbol {\Gamma} j _ {k - 1}
$$

此即系统方程,其中

$$
\mathbf {X} _ {k} = \left[ \begin{array}{l} s _ {k} \\ v _ {k} \\ a _ {k} \end{array} \right], \quad \boldsymbol {\Phi} = \left[ \begin{array}{l l l} 1 & T & \frac {T ^ {2}}{2} \\ 0 & 1 & T \\ 0 & 0 & 1 \end{array} \right], \quad \boldsymbol {\Gamma} = \left[ \begin{array}{l} \frac {T ^ {3}}{6} \\ \frac {T ^ {2}}{2} \\ 1 \end{array} \right], \quad Q = \frac {q}{T}
$$

量测方程为

$$
Z _ {k} = s _ {k} + V _ {k} = \mathbf {H X} _ {k} + V _ {k}
$$

其中 $H = \begin{bmatrix} 1 & 0 & 0 \end{bmatrix}$ 。

可见这是定常系统的滤波问题。应用卡尔曼滤波基本方程

$$
\hat {\mathbf {X}} _ {k} = \boldsymbol {\Phi} \hat {\mathbf {X}} _ {k - 1} + \mathbf {K} _ {k} (Z _ {k} - H \boldsymbol {\Phi} \hat {\mathbf {X}} _ {k - 1})\tag{4}
$$

$$
\boldsymbol {P} _ {k / k - 1} = \boldsymbol {\Phi} \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} ^ {\mathrm{T}} + \boldsymbol {\Gamma} \boldsymbol {Q} \boldsymbol {\Gamma} ^ {\mathrm{T}}\tag{5}
$$

$$
\boldsymbol {P} _ {k} = (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H}) \boldsymbol {P} _ {k / k - 1}\tag{6}
$$

$$
\mathbf {K} _ {k} = \mathbf {P} _ {k} \mathbf {H} ^ {\mathrm{T}} \mathbf {R} ^ {- 1}\tag{7}
$$

式中

$$
Q = \frac {q}{T}, \quad R = r
$$

当滤波达到稳态时， $P_{k}=P_{k-1}=P$ 为定值，此时，由式(6)、(7)和式(5)，得

$$
\boldsymbol {P} = (\boldsymbol {I} - \boldsymbol {P} \boldsymbol {H} ^ {\mathrm{T}} r ^ {- 1} \boldsymbol {H}) (\boldsymbol {\Phi} \boldsymbol {P} \boldsymbol {\Phi} ^ {\mathrm{T}} + \boldsymbol {\Gamma} \frac {q}{T} \boldsymbol {\Gamma} ^ {\mathrm{T}})
$$

这是关于 P 的矩阵代数方程,从中可解出 P,假设解得

$$
\mathbf {P} = \left[ \begin{array}{l l l} P _ {1 1} & P _ {1 2} & P _ {1 3} \\ P _ {2 1} & P _ {2 2} & P _ {2 3} \\ P _ {3 1} & P _ {3 2} & P _ {3 3} \end{array} \right]
$$

代入式(7)，得

$$
\mathbf {K} = \mathbf {P H} ^ {\mathrm{T}} r ^ {- 1} = \left[ \begin{array}{l l l} P _ {1 1} & P _ {1 2} & P _ {1 3} \\ P _ {2 1} & P _ {2 2} & P _ {2 3} \\ P _ {3 1} & P _ {3 2} & P _ {3 3} \end{array} \right] \left[ \begin{array}{l} 1 \\ 0 \\ 0 \end{array} \right] \frac {1}{r} = \left[ \begin{array}{l} \frac {P _ {1 1}}{r} \\ \frac {P _ {2 1}}{r} \\ \frac {P _ {3 1}}{r} \end{array} \right] \stackrel {\text {def}} {=} \left[ \begin{array}{l} \alpha \\ \beta \\ \gamma \end{array} \right]
$$

$$
\hat {\mathbf {X}} _ {k} = \boldsymbol {\Phi} \hat {\mathbf {X}} _ {k - 1} + \left[ \begin{array}{l} \alpha \\ \beta \\ \gamma \end{array} \right] (Z _ {k} - H \boldsymbol {\Phi} \hat {\mathbf {X}} _ {k - 1})
$$

此即为 $\hat{s}_k, \hat{v}_k$ 和 $\hat{a}_k$ 。

在该题中，被估计信息为 $s_{k}, v_{k}, a_{k}$ ，相应的稳态增益为 $\alpha, \beta, \gamma$ ，习惯上称此种滤波为 $\alpha - \beta - \gamma$ 滤波。当被估计量为 $s_{k}$ 和 $v_{k}$ 时，相应的稳态滤波称为 $\alpha - \beta$ 滤波。

例2-14 在例2-12中， $\Phi = Q = R = P_0 = 1$ ，求 $K_{k}$ 和 $P_{k}$ 。

解 根据卡尔曼滤波基本方程, 得

$$
\begin{array}{r l} & P _ {1 / 0} = P _ {0} + Q = 2 \\ & K _ {1} = P _ {1 / 0} (P _ {1 / 0} + R) ^ {- 1} = \frac {2}{3} = 0. 6 7 \\ & P _ {1} = (1 - K _ {1}) P _ {1 / 0} = P _ {1 / 0} (P _ {1 / 0} + R) ^ {- 1} = K _ {1} \end{array}
$$

并计算得

$$
P _ {2} = K _ {2} = \frac {P _ {1} + 1}{P _ {1} + 2} = \frac {5}{8} = 0. 6 2 5
$$

$$
P _ {3} = K _ {3} = \frac {P _ {2} + 1}{P _ {2} + 2} = \frac {1 3}{2 1} = 0. 6 2
$$

$$
P _ {k} = K _ {k} = \frac {P _ {k - 1} + 1}{P _ {k - 1} + 2}
$$

根据上述计算结果绘出的 $P_{k / k - 1}$ 和 $P_{k}(K_{k})$ 在滤波过程中的变化图形，如图2.2.4所示。从图中

![](images/633df0c106fd3cd44256383ec4cd9b8a317c9fb71c4fc4fb03ee1163327493d5.jpg)  
图2.2.4 不同 $k$ 时刻的 $P_{k}(K_{k})$ 和 $P_{k / k - 1}$

可看出，随着滤波步数 k 的增大， $K_{k}$ 和 $P_{k}$ 逐渐减小，并逐渐趋于稳态值，这意味着滤波刚开始时，对状态的估计主要依赖于量测，量测的修正作用不断地改善状态的估计精度，使 $P_{k}$ 逐渐降低。由于估计精度的不断提高，估计值本身所具有信息的可利用程度也在逐渐提高，滤波过程中的时间更新作用逐渐加强，而量测更新作用则逐渐相对减弱。当两者的作用趋于平衡时，滤波达到稳态，此时 $K_{k}$ 和 $P_{k}$ 达到稳态值， $P_{k / k - 1}$ 和 $P_{k}$ 的差别也逐渐减小并趋于一致。

例2-15 设电离层探测器上装有惯性导航系统，在飞行初始阶段用无线电定位测量的方法来实现飞行中导航参数的校正。为分析方便，仅考虑单轴情况，且认为惯导系统的主要误差源是初始条件（位置、速度和加速度）的误差。略去高阶项，则适用于短时间（几分钟）的惯导系统位置误差方程的解为

$$
\delta P (t) = \delta P (0) + \delta v (0) t + \delta a (0) \frac {t ^ {2}}{2}
$$

式中 $\delta P(0)$ , $\delta v(0)$ 和 $\delta a(0)$ 各表示惯导系统在无线电定位信息引入前的位置误差、速度误差和加速度误差。因此系统状态方程为

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {F} \boldsymbol {X} (t)
$$

式中

$$
\mathbf {X} (t) = \left[ \begin{array}{l} \delta P (t) \\ \delta v (t) \\ \delta a (t) \end{array} \right], \quad \mathbf {F} = \left[ \begin{array}{l l l} 0 & 1 & 0 \\ 0 & 0 & 1 \\ 0 & 0 & 0 \end{array} \right]
$$

量测量 Z 是惯导系统解算得的位置量 $P_{i}$ 与无线电定位系统指示的位置量 $P_{r}$ 之差，即

$$
Z = P _ {i} - P _ {r} = (P + \delta P) - (P + \delta P _ {r}) = \delta P - \delta P _ {r}
$$

式中 $\delta P_{r}$ 为无线电定位误差。

空中校正分别在 t=0,0.5 和 1 分钟时间点上用量测量 Z 对惯导系统进行三次估计，估计出 $\delta\hat{P},\delta\hat{v}$ 和 $\delta\hat{a}$ ，设计该系统的卡尔曼滤波器。

解 根据题意,可画出系统和量测的方块图,如图 2.2.5 所示。

无线电定位系统的定位噪声是跳变剧烈的无线电噪声,可用白噪声来描述之,所以量测方程为

![](images/3e8ae9b649bed751a160729435e0b45c466b3e0c20659eff927315ecbbeb2532.jpg)  
图2.2.5 系统和量测的方块图

$$
Z _ {k} = [ 1 0 0 ] \mathbf {X} _ {k} + V _ {k} = \mathbf {H X} _ {k} + V _ {k}
$$

式中

$$
E [ V _ {k} V _ {j} ] = R \delta_ {k j}, E [ V _ {k} ] = 0
$$

将系统状态方程离散化,系统转移矩阵为

$$
\boldsymbol {\Phi} (T) = \boldsymbol {I} + \boldsymbol {F} T + \frac {T ^ {2}}{2 !} \boldsymbol {F} ^ {2} + \frac {T ^ {3}}{3 !} \boldsymbol {F} ^ {3} + \dots =
$$

$$
\left[ \begin{array}{l l l} 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{array} \right] + \left[ \begin{array}{l l l} 0 & 1 & 0 \\ 0 & 0 & 1 \\ 0 & 0 & 0 \end{array} \right] T + \left[ \begin{array}{l l l} 0 & 0 & 1 \\ 0 & 0 & 0 \\ 0 & 0 & 0 \end{array} \right] \frac {T ^ {2}}{2} = \left[ \begin{array}{l l l} 1 & T & \frac {T ^ {2}}{2} \\ 0 & 1 & T \\ 0 & 0 & 1 \end{array} \right]
$$

设初始时刻状态的均值和方差阵为

$$
\boldsymbol {m} _ {X _ {0}} = \left[ \begin{array}{l} m _ {P _ {0}} \\ m _ {v _ {0}} \\ m _ {a _ {0}} \end{array} \right], \quad \boldsymbol {C} _ {X _ {0}} = \left[ \begin{array}{c c c} C _ {P _ {0}} & 0 & 0 \\ 0 & C _ {v _ {0}} & 0 \\ 0 & 0 & C _ {a _ {0}} \end{array} \right]
$$

在 $k = 0$ 时刻作第一次定位测量和估计，即利用 $Z_{0}$ 去估计 $\pmb{X}_0$ ，并求出 $\pmb{P}_0$ 。令

$$
\hat {\boldsymbol {X}} _ {0 / - 1} = \boldsymbol {m} _ {X _ {0}}, \quad \boldsymbol {P} _ {0 / - 1} = \boldsymbol {C} _ {X _ {0}}
$$

则

$$
\begin{array}{r l} & {\mathbf {K} _ {0} = \mathbf {P} _ {0 / - 1} \mathbf {H} ^ {\mathrm{T}} (\mathbf {H P} _ {0 / - 1} \mathbf {H} ^ {\mathrm{T}} + R) ^ {- 1} = \left[ \begin{array}{c} C _ {P _ {0}} \\ \hline C _ {P _ {0}} + R \\ 0 \\ 0 \end{array} \right]} \\ & {\hat {\mathbf {X}} _ {0} = \hat {\mathbf {X}} _ {0 / - 1} + \mathbf {K} _ {0} (Z _ {0} - \mathbf {H} \hat {\mathbf {X}} _ {0 / - 1})} \end{array}
$$

下面仅从估计的均方误差来看无线电定位量测对估计的作用。

$$
\mathbf {P} _ {0} = \mathbf {P} _ {0 / - 1} - \mathbf {K} _ {0} \mathbf {H P} _ {0 / - 1} = \left[ \begin{array}{c c c} C _ {P _ {0}} - \frac {C _ {P _ {0}} ^ {2}}{C _ {P _ {0}} + R} & 0 & 0 \\ 0 & C _ {v _ {0}} & 0 \\ 0 & 0 & C _ {a _ {0}} \end{array} \right]
$$

因为惯导系统初始位置误差的方差 $C_{P_0}$ 远比无线电定位误差的方差大，即 $C_{P_0} \gg R$ ，所以

$$
P _ {0} ^ {1 1} = \frac {C _ {P _ {0}} R}{C _ {P _ {0}} + R} \approx R
$$

即第一次定位组合后的 $\delta P$ 估计均方误差接近无线电定位误差 R，相当于惯导的定位精度被提高到接近无线电定位的精度。而 $\delta v$ 和 $\delta a$ 未能估计，因估计的均方误差仍为 $C_{v_{0}}$ 和 $C_{u_{0}}$ 。

在第二次定位组合中

$$
\begin{array}{r l} & {\mathbf {P} _ {1 / 0} = \boldsymbol {\Phi} \mathbf {P} _ {0} \boldsymbol {\Phi} ^ {\mathrm{T}} = \left[ \begin{array}{c c c} R + C _ {v _ {0}} T + C _ {a _ {0}} \frac {T ^ {2}}{2} & C _ {v _ {0}} T + C _ {a _ {0}} \frac {T ^ {3}}{2} & C _ {a _ {0}} \frac {T ^ {2}}{2} \\ C _ {v _ {0}} T + C _ {a _ {0}} \frac {T ^ {3}}{2} & C _ {v _ {0}} + C _ {a _ {0}} T ^ {2} & C _ {a _ {0}} T \\ C _ {a _ {0}} \frac {T ^ {2}}{2} & C _ {a _ {0}} T & C _ {a _ {0}} \end{array} \right] \stackrel {\text {def}} {=}} \\ & {\left[ \begin{array}{c c c} P _ {1 / 0} ^ {1 1} & P _ {1 / 0} ^ {1 2} & P _ {1 / 0} ^ {1 3} \\ P _ {1 / 0} ^ {2 1} & P _ {1 / 0} ^ {2 2} & P _ {1 / 0} ^ {2 3} \\ P _ {1 / 0} ^ {3 1} & P _ {1 / 0} ^ {3 2} & P _ {1 / 0} ^ {3 3} \end{array} \right]} \\ & {\mathbf {P} _ {1} = \mathbf {P} _ {1 / 0} - \mathbf {P} _ {1 / 0} \mathbf {H} ^ {\mathrm{T}} (\mathbf {H P} _ {1 / 0} \mathbf {H} ^ {\mathrm{T}} + R) ^ {- 1} \mathbf {H P} _ {1 / 0} =} \\ & {\mathbf {P} _ {1 / 0} - \frac {1}{P _ {1 / 0} ^ {1 1} + R} \left[ \begin{array}{c c c} (P _ {1 / 0} ^ {1 1}) ^ {2} & P _ {1 / 0} ^ {1 1} P _ {1 / 0} ^ {1 2} & P _ {1 / 0} ^ {1 1} P _ {1 / 0} ^ {1 3} \\ P _ {1 / 0} ^ {1 1} P _ {1 / 0} ^ {1 2} & (P _ {1 / 0} ^ {1 2}) ^ {2} & P _ {1 / 0} ^ {1 2} P _ {1 / 0} ^ {1 3} \\ P _ {1 / 0} ^ {1 1} P _ {1 / 0} ^ {1 3} & P _ {1 / 0} ^ {1 2} P _ {1 / 0} ^ {1 3} & (P _ {1 / 0} ^ {1 3}) ^ {2} \end{array} \right]} \end{array}
$$

即

$$
P _ {1} ^ {1 1} = P _ {1 / 0} ^ {1 1} \left(1 - \frac {P _ {1 / 0} ^ {1 1}}{P _ {1 / 0} ^ {1 1} + R}\right) = P _ {1 / 0} ^ {1 1} - \frac {\left(R + C _ {v _ {0}} T + C _ {a _ {0}} \frac {T ^ {2}}{2}\right) ^ {2}}{2 R + C _ {v _ {0}} T + C _ {a _ {0}} \frac {T ^ {2}}{2}}
$$

$$
P _ {1} ^ {2 2} = P _ {1 / 0} ^ {2 2} - \frac {(P _ {1 / 0} ^ {1 2}) ^ {2}}{P _ {1 / 0} ^ {1 1} + R} = P _ {1 / 0} ^ {2 2} - \frac {\left(C _ {v _ {0}} T + C _ {a _ {0}} \frac {T ^ {3}}{2}\right) ^ {2}}{2 R + C _ {v _ {0}} T + C _ {a _ {0}} \frac {T ^ {2}}{2}}
$$

$$
P _ {1} ^ {3 3} = P _ {1 / 0} ^ {3 3} - \frac {(P _ {1 / 0} ^ {1 3}) ^ {2}}{P _ {1 / 0} ^ {1 1} + R} = P _ {1 / 0} ^ {3 3} \left(1 - \frac {C _ {a _ {0}} \frac {T ^ {4}}{4}}{2 R + C _ {v _ {0}} T + C _ {a _ {0}} \frac {T ^ {2}}{2}}\right)
$$

可以看出,在定位时间间隔 T 较短的情况下,第二次定位组合后的估计,仍是 $\delta P$ 下降得较多, $\delta v$ 下降次之, $\delta a$ 下降最少。如果

$$
\begin{array}{l} C _ {P _ {0}} = (1 \mathrm{nmile}) ^ {2}, C _ {v _ {0}} = (7. 3 \mathrm{nmile/h}) ^ {2} \\ C _ {a _ {0}} = 0, \sqrt {R} = 9. 1 \mathrm{m} \end{array}
$$

则包括高阶项的完整方程计算的结果 $\left(\sqrt{P_{k}^{11}}\right)$ 如图2.2.6所示。可以看出，分析结果与图是一致的，其中 $\sqrt{P_{3/2}^{11}}$ 是第三次定位组合后 $\sqrt{P^{11}}$ 的演变情况。

![](images/8761bf8cb91c8ecd3f40a0d1b9da230f25de8fbafa03e96d96c5fa1dc6313878.jpg)  
图 2.2.6 用无线电定位对惯导系统作导航参数校正

## 2.2.5 白噪声条件下离散型卡尔曼滤波方程的一般形式

设系统方程和量测方程的一般形式为

$$
\boldsymbol {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {X} _ {k - 1} + \boldsymbol {B} _ {k - 1} \boldsymbol {u} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1}\tag{2.2.41}
$$

$$
\mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {Y} _ {k} + \mathbf {V} _ {k}\tag{2.2.42}
$$

式中， $\{\pmb{u}_k\}$ 和 $\{\mathbf{Y}_k\}$ 都是已知的确定性输入序列， $u_{k}$ 可理解为控制项， $Y_{k}$ 可理解为量测的系统性

误差。其余项的规定同前，但认为同时刻的 $W_{k}$ 与 $\mathbf{V}_k$ 是相关的，即

$$
E \left[ \boldsymbol {W} _ {k} \boldsymbol {V} _ {j} ^ {\mathrm{T}} \right] = \boldsymbol {S} _ {k} \delta_ {k j}
$$

## 1. 滤波一般方程

将式 $(2.2.41)$ 改写为

$$
\boldsymbol {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {X} _ {k - 1} + \boldsymbol {B} _ {k - 1} \boldsymbol {u} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1} + \boldsymbol {J} _ {k - 1} (\boldsymbol {Z} _ {k - 1} - \boldsymbol {H} _ {k - 1} \boldsymbol {X} _ {k - 1} - \boldsymbol {Y} _ {k - 1} - \boldsymbol {V} _ {k - 1})
$$

式中 $J_{k-1}$ 为待定的 $n \times m$ 矩阵。令

$$
\boldsymbol {\Phi} _ {k, k - 1} ^ {*} = \boldsymbol {\Phi} _ {k, k - 1} - \boldsymbol {J} _ {k - 1} \boldsymbol {H} _ {k - 1}\tag{2.2.43}
$$

$$
\boldsymbol {W} _ {k - 1} ^ {*} = \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1} - \boldsymbol {J} _ {k - 1} \boldsymbol {V} _ {k - 1}\tag{2.2.44}
$$

则

$$
\boldsymbol {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} ^ {*} \boldsymbol {X} _ {k - 1} + \boldsymbol {B} _ {k - 1} \boldsymbol {u} _ {k - 1} + \boldsymbol {J} _ {k - 1} (\boldsymbol {Z} _ {k - 1} - \boldsymbol {Y} _ {k - 1}) + \boldsymbol {W} _ {k - 1} ^ {*}\tag{2.2.45}
$$

显然式(2.2.45)是式(2.2.41)的恒等变形，其中 $B_{k-1}u_{k-1}+J_{k-1}(Z_{k-1}-Y_{k-1})$ 可看做式(2.2.45)所示系统的控制项。 $\{W_{k}^{*}\}$ 和 $\{V_{k}\}$ 之间的协方差函数阵为

$$
E \left[ \boldsymbol {W} _ {k} ^ {*} \boldsymbol {V} _ {j} ^ {\mathrm{T}} \right] = E \left[ \left(\boldsymbol {\Gamma} _ {k} \boldsymbol {W} _ {k} - \boldsymbol {J} _ {k} \boldsymbol {V} _ {k}\right) \boldsymbol {V} _ {j} ^ {\mathrm{T}} \right] = \left(\boldsymbol {\Gamma} _ {k} \boldsymbol {S} _ {k} - \boldsymbol {J} _ {k} \boldsymbol {R} _ {k}\right) \delta_ {k j}
$$

选择

$$
\boldsymbol {J} _ {k} = \boldsymbol {\Gamma} _ {k} \boldsymbol {S} _ {k} \boldsymbol {R} _ {k} ^ {- 1}\tag{2.2.46}
$$

则

$$
E [ \boldsymbol {W} _ {k} ^ {*} \boldsymbol {V} _ {j} ^ {\mathrm{T}} ] = \mathbf {0}
$$

这说明若按式(2.2.46)选择 $J_{k}$ ，则式(2.2.45)所示系统的系统噪声 $W_{k}^{*}$ 与量测噪声 $V_{k}$ 不相关，这就和2.2.4节中4.的情况完全一样，所以一步预测方程为

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {k / k - 1} & = \boldsymbol {\Phi} _ {k, k - 1} ^ {*} \hat {\mathbf {X}} _ {k - 1} + \mathbf {B} _ {k - 1} \mathbf {u} _ {k - 1} + \mathbf {J} _ {k - 1} (\mathbf {Z} _ {k - 1} - \mathbf {Y} _ {k - 1}) = \\ & \quad \boldsymbol {\Phi} _ {k, k - 1} \hat {\mathbf {X}} _ {k - 1} + \mathbf {B} _ {k - 1} \mathbf {u} _ {k - 1} + \mathbf {J} _ {k - 1} (\mathbf {Z} _ {k - 1} - \mathbf {Y} _ {k - 1} - \mathbf {H} _ {k - 1} \hat {\mathbf {X}} _ {k - 1}) \end{array}\tag{2.2.47}
$$

从上式可看出， $J_{k-1}$ 相当于状态一步预测的增益阵。

$$
\hat {\mathbf {X}} _ {k} = \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} (\mathbf {Z} _ {k} - \mathbf {Y} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1})\tag{2.2.48}
$$

$$
\boldsymbol {K} _ {k} = \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1}\tag{2.2.49}
$$

下面推导 $P_{k/k-1}$ 和 $P_{k}$ 。

式 $(2.2.45)$ 减去式 $(2.2.47)$ ，得

$$
\widetilde {X} _ {k / k - 1} = \Phi_ {k, k - 1} ^ {*} \widetilde {X} _ {k - 1} + W _ {k - 1} ^ {*}
$$

所以

$$
\begin{array}{r l} \boldsymbol {P} _ {k / k - 1} & = E [ \widetilde {\boldsymbol {X}} _ {k / k - 1} \widetilde {\boldsymbol {X}} _ {k / k - 1} ^ {\mathrm{T}} ] = \boldsymbol {\Phi} _ {k, k - 1} ^ {*} \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {* \mathrm{T}} + \\ & \quad E [ (\boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1} - \boldsymbol {J} _ {k - 1} \boldsymbol {V} _ {k - 1}) (\boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1} - \boldsymbol {J} _ {k - 1} \boldsymbol {V} _ {k - 1}) ^ {\mathrm{T}} ] = \\ & \quad \boldsymbol {\Phi} _ {k, k - 1} ^ {*} \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {* \mathrm{T}} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {Q} _ {k - 1} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}} + \boldsymbol {J} _ {k - 1} \boldsymbol {R} _ {k - 1} \boldsymbol {J} _ {k - 1} ^ {\mathrm{T}} - \\ & \quad \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {S} _ {k - 1} \boldsymbol {J} _ {k - 1} ^ {\mathrm{T}} - \boldsymbol {J} _ {k - 1} \boldsymbol {S} _ {k - 1} ^ {\mathrm{T}} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}} = \\ & \quad (\boldsymbol {\Phi} _ {k, k - 1} - \boldsymbol {J} _ {k - 1} \boldsymbol {H} _ {k - 1}) \boldsymbol {P} _ {k - 1} (\boldsymbol {\Phi} _ {k, k - 1} - \boldsymbol {J} _ {k - 1} \boldsymbol {H} _ {k - 1}) ^ {\mathrm{T}} + \\ & \quad \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {Q} _ {k - 1} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}} - \boldsymbol {J} _ {k - 1} \boldsymbol {S} _ {k - 1} ^ {\mathrm{T}} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}} \\ & \quad \boldsymbol {P} _ {k} = (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1} \end{array}\tag{2.2.50}
$$

(2.2.51)

式(2.2.46)～式(2.2.51)就是系统噪声与量测噪声相关条件下离散卡尔曼滤波的一般方程。同样，当初始条件 $\hat{X}_{0}=m_{X0}(P_{0}=C_{X0})$ 时，估计也是无偏的。

2. 一步预测一般方程

将式 $(2.2.47)$ 改写成

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {k + 1 / k} & = \boldsymbol {\Phi} _ {k + 1, k} \hat {\mathbf {X}} _ {k} + \boldsymbol {B} _ {k} \boldsymbol {u} _ {k} + \boldsymbol {J} _ {k} (\mathbf {Z} _ {k} - \mathbf {Y} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k}) = \\ & (\boldsymbol {\Phi} _ {k + 1, k} - \boldsymbol {J} _ {k} \mathbf {H} _ {k}) \hat {\mathbf {X}} _ {k} + \boldsymbol {B} _ {k} \boldsymbol {u} _ {k} + \boldsymbol {J} _ {k} (\mathbf {Z} _ {k} - \mathbf {Y} _ {k}) \end{array}
$$

式 $(2.2.48)$ 代入上式，得

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {k + 1 / k} & = (\boldsymbol {\Phi} _ {k + 1, k} - \boldsymbol {J} _ {k} \boldsymbol {H} _ {k}) [ \hat {\mathbf {X}} _ {k / k - 1} + \boldsymbol {K} _ {k} (\mathbf {Z} _ {k} - \mathbf {Y} _ {k} - \boldsymbol {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1}) ] + \boldsymbol {B} _ {k} \boldsymbol {u} _ {k} + \boldsymbol {J} _ {k} (\mathbf {Z} _ {k} - \mathbf {Y} _ {k}) = \\ & \quad \boldsymbol {\Phi} _ {k + 1, k} \hat {\mathbf {X}} _ {k / k - 1} + \boldsymbol {B} _ {k} \boldsymbol {u} _ {k} + (\boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {K} _ {k} + \boldsymbol {J} _ {k} - \boldsymbol {J} _ {k} \boldsymbol {H} _ {k} \boldsymbol {K} _ {k}) (\mathbf {Z} _ {k} - \mathbf {Y} _ {k} - \boldsymbol {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1}) \end{array}
$$

令

$$
\Phi_ {k + 1, k} \mathbf {K} _ {k} + \mathbf {J} _ {k} - \mathbf {J} _ {k} \mathbf {H} _ {k} \mathbf {K} _ {k} \stackrel {\mathrm{det}} {=} \overline {{\mathbf {K}}} _ {k}\tag{2.2.52}
$$

则

$$
\hat {\mathbf {X}} _ {k + 1 / k} = \boldsymbol {\Phi} _ {k + 1, k} \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {B} _ {k} \mathbf {u} _ {k} + \bar {\mathbf {K}} _ {k} (\mathbf {Z} _ {k} - \mathbf {Y} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1})\tag{2.2.53}
$$

根据式(2.2.46)， $\bar{K}_{k}$ 可写成

$$
\overline {{{\boldsymbol {K}}}} _ {k} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {K} _ {k} + \boldsymbol {\Gamma} _ {k} \boldsymbol {S} _ {k} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {I} - \boldsymbol {H} _ {k} \boldsymbol {K} _ {k})
$$

又因

$$
\begin{array}{r l} \mathbf {I} - \mathbf {H} _ {k} \mathbf {K} _ {k} & = \mathbf {I} - \mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} (\mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} + \mathbf {R} _ {k}) ^ {- 1} = \\ & (\mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} + \mathbf {R} _ {k}) (\mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} + \mathbf {R} _ {k}) ^ {- 1} - \\ & \mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} (\mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} + \mathbf {R} _ {k}) ^ {- 1} = \\ & \mathbf {R} _ {k} (\mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} + \mathbf {R} _ {k}) ^ {- 1} \end{array}
$$

所以

$$
\bar {\boldsymbol {K}} = \left(\boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k} \boldsymbol {S} _ {k}\right) \left(\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}\right) ^ {- 1}\tag{2.2.54}
$$

下面求预测均方误差阵。将式(2.2.42)代入式(2.2.53)，得

$$
\hat {\mathbf {X}} _ {k + 1 / k} = \boldsymbol {\Phi} _ {k + 1, k} \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {B} _ {k} \mathbf {u} _ {k} + \overline {{\mathbf {K}}} _ {k} \left[ \mathbf {H} _ {k} \left(\mathbf {X} _ {k} - \hat {\mathbf {X}} _ {k / k - 1}\right) + \mathbf {V} _ {k} \right]
$$

所以

$$
\begin{array}{r l} & {\widetilde {X} _ {k + 1 / k} = (\Phi_ {k + 1, k} - \overline {{{K}}} _ {k} H _ {k}) \widetilde {X} _ {k / k - 1} + \Gamma_ {k} W _ {k} - \overline {{{K}}} _ {k} V _ {k}} \\ & {P _ {k + 1 / k} = E [ \widetilde {X} _ {k + 1 / k} \widetilde {X} _ {k + 1 / k} ^ {\mathrm{T}} ] = (\Phi_ {k + 1, k} - \overline {{{K}}} _ {k} H _ {k}) P _ {k / k - 1} (\Phi_ {k + 1, k} - \overline {{{K}}} _ {k} H _ {k}) ^ {\mathrm{T}} +} \\ & {\quad \Gamma_ {k} Q _ {k} \Gamma_ {k} ^ {\mathrm{T}} + \overline {{{K}}} _ {k} R _ {k} \overline {{{K}}} _ {k} ^ {\mathrm{T}} - \Gamma_ {k} S _ {k} \overline {{{K}}} _ {k} ^ {\mathrm{T}} - \overline {{{K}}} _ {k} S _ {k} ^ {\mathrm{T}} \Gamma_ {k} ^ {\mathrm{T}} =} \\ & {\quad \Phi_ {k + 1, k} P _ {k / k - 1} \Phi_ {k + 1, k} ^ {\mathrm{T}} + \Gamma_ {k} Q _ {k} \Gamma_ {k} ^ {\mathrm{T}} - \overline {{{K}}} _ {k} (H _ {k} P _ {k / k - 1} \Phi_ {k + 1, k} ^ {\mathrm{T}} + S _ {k} ^ {\mathrm{T}} \Gamma_ {k} ^ {\mathrm{T}}) -} \\ & {\quad (\Phi_ {k + 1, k} P _ {k / k - 1} H _ {k} ^ {\mathrm{T}} + \Gamma_ {k} S _ {k}) \overline {{{K}}} _ {k} ^ {\mathrm{T}} + \overline {{{K}}} _ {k} (H _ {k} P _ {k / k - 1} H _ {k} ^ {\mathrm{T}} + R _ {k}) \overline {{{K}}} _ {k} ^ {\mathrm{T}}} \end{array}
$$

将式 $(2.2.54)$ 代入上式,经整理,得

$$
\boldsymbol {P} _ {k + 1 / k} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k} \boldsymbol {Q} _ {k} \boldsymbol {\Gamma} _ {k} ^ {\mathrm{T}} - \bar {\boldsymbol {K}} _ {k} (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \boldsymbol {S} _ {k} ^ {\mathrm{T}} \boldsymbol {\Gamma} _ {k} ^ {\mathrm{T}})\tag{2.2.55}
$$

式(2.2.53)、式(2.2.54)和式(2.2.55)即为一步预测一般方程。如果 $S_{k}=0$ ，则式(2.2.55)和式(2.2.40)完全一样。

一般方程除了适用于系统和量测有确定性的输入项以及噪声之间相关等情况外，在处理量测中的有色噪声时也要用到，这将在第三章中讨论。

## 2.3 连续型卡尔曼滤波

采用递推算法是离散型卡尔曼滤波的最大优点,算法可由计算机执行,不必存储时间过程中的大量量测数据,因此离散型卡尔曼滤波在工程上得到了广泛的应用。虽然很多物理系统是连续系统,但只要离散化,就能使用离散型卡尔曼滤波方程。

连续型卡尔曼滤波则根据连续时间过程中的量测值,采用求解矩阵微分方程的方法估计系统状态变量的时间连续值,因此算法失去了递推性。连续型卡尔曼滤波是最优估计理论中的一部分,讨论该算法是为了使读者加深对卡尔曼滤波理论的理解。

## 2.3.1 连续型卡尔曼滤波方程的推导

连续型卡尔曼滤波方程可在离散型卡尔曼滤波基本方程的基础上推导出来。基本思路是:将连续系统离散化,应用离散型卡尔曼滤波基本方程和导数概念推导出连续型滤波方程。为叙述方便,先介绍一些基本关系。

1. 连续型白噪声过程和等效白噪声序列之间的关系

设连续系统的系统方程和量测方程分别为

$$
\left. \begin{array}{l} \dot {\boldsymbol {X}} (t) = \boldsymbol {F} (t) \boldsymbol {X} (t) + \boldsymbol {G} (t) \boldsymbol {w} (t) \\ \boldsymbol {Z} (t) = \boldsymbol {H} (t) \boldsymbol {X} (t) + \boldsymbol {v} (t) \end{array} \right\}\tag{2.3.1}
$$

式中

$$
\left. \begin{array}{l l} E [ w (t) ] = 0, & E [ w (t) w ^ {\mathrm{T}} (\tau) ] = q (t) \delta (t - \tau) \\ E [ v (t) ] = 0, & E [ v (t) v ^ {\mathrm{T}} (\tau) ] = r (t) \delta (t - \tau) \end{array} \right\}\tag{2.3.2}
$$

$w(t)$ 和 $v(t)$ 不相关, $q(t)$ 为非负定阵, $r(t)$ 为正定阵。

对式 $(2.3.1)$ 和式 $(2.3.2)$ 作离散化处理,得

$$
\left. \begin{array}{l} \mathbf {X} (t _ {k} + \Delta t) = \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) \mathbf {X} (t _ {k}) + \int_ {t _ {k}} ^ {t _ {k} + \Delta t} \boldsymbol {\Phi} (t _ {k} + \Delta t, \tau) \mathbf {G} (\tau) w (\tau) d \tau \\ \mathbf {Z} (t _ {k} + \Delta t) = \mathbf {H} (t _ {k} + \Delta t) \mathbf {X} (t _ {k} + \Delta t) + \mathbf {V} _ {k} \end{array} \right\}\tag{2.3.3}
$$

式中 $\Phi(t_k + \Delta t, t_k)$ 满足方程

$$
\left. \begin{array}{l} \dot {\boldsymbol {\Phi}} (t, t _ {k}) = \boldsymbol {F} (t) \boldsymbol {\Phi} (t, t _ {k}) \\ \boldsymbol {\Phi} (t _ {k}, t _ {k}) = \boldsymbol {I} \end{array} \right\}\tag{2.3.4}
$$

作如下等效处理

$$
\boldsymbol {W} _ {k} = \frac {1}{\Delta t} \int_ {t _ {k}} ^ {t _ {k} + \Delta t} \boldsymbol {w} (t) \mathrm{d} t\tag{2.3.5}
$$

$$
\mathbf {V} _ {k} = \frac {1}{\Delta t} \int_ {t _ {k}} ^ {t _ {k} + \Delta t} \mathbf {v} (t) \mathrm{d} t\tag{2.3.6}
$$

根据式(2.3.5)对 $W_{k}$ 的定义,有

$$
\begin{array}{r l} E [ \pmb {W} _ {k} \pmb {W} _ {j} ^ {\mathrm{T}} ] = & E \left[ \frac {1}{\Delta t} \int_ {t _ {k}} ^ {t _ {k} + \Delta t} \pmb {w} (t) \mathrm{d} t \cdot \frac {1}{\Delta t} \int_ {t _ {j}} ^ {t _ {j} + \Delta t} \pmb {w} ^ {\mathrm{T}} (\tau) \mathrm{d} \tau \right] = \\ & \frac {1}{\Delta t ^ {2}} \int_ {t _ {k}} ^ {t _ {k} + \Delta t} \int_ {t _ {j}} ^ {t _ {j} + \Delta t} E [ \pmb {w} (t) \pmb {w} ^ {\mathrm{T}} (\tau) ] \mathrm{d} \tau \mathrm{d} t = \\ & \frac {1}{\Delta t ^ {2}} \int_ {t _ {k}} ^ {t _ {k} + \Delta t} \pmb {q} (t) \int_ {t _ {j}} ^ {t _ {j} + \Delta t} \delta (t - \tau) \mathrm{d} \tau \mathrm{d} t \end{array}
$$

由于积分变量 $t \in [t_k, t_k + \Delta t), \tau \in [t_j, t_j + \Delta t)$ ，而当 $t \neq \tau$ 时， $\delta(t - \tau) = 0$ ，所以只有当上述两积分区间重合时，即 $j = k$ 时，上述积分才不为零，所以

$$
E \left[ \boldsymbol {W} _ {k} \boldsymbol {W} _ {j} ^ {\mathrm{T}} \right] = \frac {1}{\Delta t ^ {2}} \int_ {t _ {k}} ^ {t _ {k} + \Delta t} \boldsymbol {q} (t) \int_ {t _ {j}} ^ {t _ {j} + \Delta t} \delta (t - \tau) \mathrm{d} \tau \mathrm{d} t \delta_ {k j} = \frac {1}{\Delta t ^ {2}} \int_ {t _ {k}} ^ {t _ {k} + \Delta t} \boldsymbol {q} (t) \mathrm{d} t \delta_ {k j}
$$

$q(t)$ 是白噪声过程 $w(t)$ 的协方差强度阵, 虽然 $w(t)$ 变化剧烈, 但 $q(t)$ 变化比较缓慢, 对平稳过

程， $q(t)$ 是常阵，根本不随时间而变，所以当 $\Delta t$ 不大时， $q(t)$ 在该区间内可近似看做常阵

$$
E [ \pmb {W} _ {k} \pmb {W} _ {j} ^ {\mathrm{T}} ] \approx \frac {1}{\Delta t ^ {2}} \pmb {q} (t _ {k}) \int_ {t _ {k}} ^ {t _ {k} + \Delta t} 1 \mathrm{d} t \delta_ {k j} = \frac {\pmb {q} (t _ {k})}{\Delta t} \delta_ {k j}\tag{2.3.7}
$$

从式(2.3.7)可看出,白噪声序列 $W_{k}$ 的协方差阵为

$$
Q _ {k} = \frac {\pmb {q} (t _ {k})}{\Delta t}\tag{2.3.8}
$$

同理可得

$$
E [ \mathbf {V} _ {k} \mathbf {V} _ {j} ^ {\mathrm{T}} ] = \mathbf {R} _ {k} \delta_ {k j}
$$

式中

$$
\mathbf {R} _ {k} = \frac {\mathbf {r} (t _ {k})}{\Delta t}\tag{2.3.9}
$$

根据式 $(2.3.5)$ ，式 $(2.3.3)$ 可近似为

$$
\boldsymbol {X} (t _ {k} + \Delta t) \approx \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) \boldsymbol {X} (t _ {k}) + \boldsymbol {\Gamma} (t _ {k} + \Delta t, t _ {k}) \boldsymbol {W} _ {k}\tag{2.3.10}
$$

式中

$$
\Gamma (t _ {k} + \Delta t, t _ {k}) = \int_ {t _ {k}} ^ {t _ {k} + \Delta t} \Phi (t _ {k} + \Delta t, \tau) G (\tau) \mathrm{d} \tau\tag{2.3.11}
$$

$\Delta t$ 越小, 式(2.3.10)的近似度就越好。

2. 连续型卡尔曼滤波基本方程的推导

根据滤波基本方程式(2.2.4)，并考虑式(2.3.8)和式(2.3.9)，得等效离散系统的卡尔曼滤波基本方程

$$
\hat {\mathbf {X}} (t _ {k} + \Delta t / t _ {k}) = \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) \hat {\mathbf {X}} (t _ {k} / t _ {k})\tag{2.3.12}
$$

$$
\hat {\mathbf {X}} (t _ {k} + \Delta t / t _ {k} + \Delta t) = \hat {\mathbf {X}} (t _ {k} + \Delta t / t _ {k}) + \mathbf {K} (t _ {k} + \Delta t) [ \mathbf {Z} (t _ {k} + \Delta t) -
$$

$$
\mathbf {H} (t _ {k} + \Delta t) \hat {\mathbf {X}} (t _ {k} + \Delta t / t _ {k}) ]\tag{2.3.13}
$$

$$
\mathbf {K} (t _ {k} + \Delta t) = \mathbf {P} (t _ {k} + \Delta t / t _ {k} + \Delta t) \mathbf {H} ^ {\mathrm{T}} (t _ {k} + \Delta t) \left[ \frac {\mathbf {r} (t _ {k} + \Delta t)}{\Delta t} \right] ^ {- 1}\tag{2.3.14}
$$

$$
\mathbf {P} (t _ {k} + \Delta t / t _ {k} + \Delta t) = [ \mathbf {I} - \mathbf {K} (t _ {k} + \Delta t) \mathbf {H} (t _ {k} + \Delta t) ] \mathbf {P} (t _ {k} + \Delta t / t _ {k})\tag{2.3.15}
$$

$$
\boldsymbol {P} (t _ {k} + \Delta t / t _ {k}) = \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) \boldsymbol {P} (t _ {k} / t _ {k}) \boldsymbol {\Phi} ^ {\mathrm{T}} (t _ {k} + \Delta t, t _ {k}) +
$$

$$
\boldsymbol {\Gamma} (t _ {k} + \Delta t, t _ {k}) \frac {\boldsymbol {q} (t _ {k})}{\Delta t} \boldsymbol {\Gamma} ^ {\mathrm{T}} (t _ {k} + \Delta t, t _ {k})\tag{2.3.16}
$$

将 $\boldsymbol{\Gamma}(t_{k}+\Delta t,t_{k})$ 和 $\boldsymbol{\Phi}(t_{k}+\Delta t,t_{k})$ 在 $t_{k}$ 处作泰勒级数展开。应用式(2.3.4)所列关系，得

$$
\begin{array}{r l} \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) & = \dot {\boldsymbol {\Phi}} (t _ {k}, t _ {k}) + \dot {\boldsymbol {\Phi}} (t _ {k} + \Delta t, t _ {k}) | _ {\Delta t = 0} \cdot \Delta t + \varepsilon_ {1} (\Delta t) = \\ & \quad I + F (t _ {k}) \Delta t + \varepsilon_ {1} (\Delta t) \end{array}\tag{2.3.17}
$$

式中 $\varepsilon_{1}(\Delta t)$ 为 $\Delta t$ 的高阶无穷小量。

$$
\Gamma (t _ {k} + \Delta t, t _ {k}) = \Gamma (t _ {k}, t _ {k}) + \dot {\Gamma} (t _ {k} + \Delta t, t _ {k}) | _ {\Delta t = 0} \cdot \Delta t + \varepsilon_ {2} (\Delta t)
$$

由式(2.3.11)知

$$
\Gamma (t _ {k}, t _ {k}) = 0
$$

$$
\dot {\Gamma} (t _ {k} + \Delta t, t _ {k}) \mid_ {\Delta t = 0} = \Phi (t _ {k} + \Delta t, t _ {k} + \Delta t) G (t _ {k} + \Delta t) \mid_ {\Delta t = 0} = G (t _ {k})
$$

所以

$$
\boldsymbol {\Gamma} (t _ {k} + \Delta t, t _ {k}) = \mathbf {G} (t _ {k}) \Delta t + \varepsilon_ {2} (\Delta t)\tag{2.3.18}
$$

式中 $\varepsilon_{2}(\Delta t)$ 为 $\Delta t$ 的高阶无穷小。

根据导数定义，并注意到 $\Delta t \rightarrow 0$ 时， $t_{k} = k\Delta t$ 成为连续时间变量，且记为 t，有

$$
\begin{array}{r l} \dot {\hat {X}} (t / t) & = \lim _ {\Delta t \to 0} \frac {\hat {X} (t _ {k} + \Delta t / t _ {k} + \Delta t) - \hat {X} (t _ {k} / t _ {k})}{\Delta t} = \\ & \quad \lim _ {\Delta t \to 0} \frac {1}{\Delta t} [ \Phi (t _ {k} + \Delta t, t _ {k}) - I ] \hat {X} (t _ {k} / t _ {k}) + \\ & \quad \lim _ {\Delta t \to 0} \frac {1}{\Delta t} K (t _ {k} + \Delta t) [ Z (t _ {k} + \Delta t) - H (t _ {k} + \Delta t) \hat {X} (t _ {k} + \Delta t / t _ {k}) ] = \\ & \quad \lim _ {\Delta t \to 0} \frac {1}{\Delta t} [ F (t _ {k}) \Delta t + \varepsilon_ {1} (\Delta t) ] \hat {X} (t _ {k} / t _ {k}) + \\ & \quad \lim _ {\Delta t \to 0} \frac {1}{\Delta t} P (t _ {k} + \Delta t / t _ {k} + \Delta t) H ^ {\mathrm{T}} (t _ {k} + \Delta t) r ^ {- 1} (t _ {k} + \Delta t) \Delta t \cdot \\ & \quad [ Z (t _ {k} + \Delta t) - H (t _ {k} + \Delta t) \hat {X} (t _ {k} + \Delta t / t _ {k}) ] = \\ & \quad F (t) \hat {X} (t / t) + P (t / t) H ^ {\mathrm{T}} (t) r ^ {- 1} (t) [ Z (t) - H (t) \hat {X} (t / t) ] \end{array}
$$

记

$$
\boldsymbol {K} (t) = \boldsymbol {P} (t / t) \boldsymbol {H} ^ {\mathrm{T}} (t) \boldsymbol {r} ^ {- 1} (t)
$$

则

$$
\dot {\hat {\mathbf {X}}} (t / t) = \mathbf {F} (t) \hat {\mathbf {X}} (t / t) + \mathbf {K} (t) [ \mathbf {Z} (t) - \mathbf {H} (t) \hat {\mathbf {X}} (t / t) ]
$$

将式 $(2.3.16)$ 代入式 $(2.3.15)$ ，得

$$
\begin{array}{r l} & {\pmb {P} (t _ {k} + \Delta t / t _ {k} + \Delta t) = [ \pmb {I} - \pmb {K} (t _ {k} + \Delta t) \pmb {H} (t _ {k} + \Delta t) ] [ \pmb {\Phi} (t _ {k} + \Delta t, t _ {k}) \bullet} \\ & {\qquad \pmb {P} (t _ {k} / t _ {k}) \pmb {\Phi} ^ {\mathrm{T}} (t _ {k} + \Delta t, t _ {k}) + \pmb {\Gamma} (t _ {k} + \Delta t, t _ {k}) \frac {\pmb {q} (t _ {k})}{\Delta t} \bullet \pmb {\Gamma} ^ {\mathrm{T}} (t _ {k} + \Delta t, t _ {k}) ] =} \\ & {\qquad [ \pmb {I} - \pmb {P} (t _ {k} + \Delta t / t _ {k} + \Delta t) \pmb {H} ^ {\mathrm{T}} (t _ {k} + \Delta t) \pmb {r} ^ {- 1} (t _ {k} + \Delta t) \bullet} \\ & {\qquad \Delta t \pmb {H} (t _ {k} + \Delta t) ] \{\left[ \pmb {I} + \pmb {F} (t _ {k}) \Delta t + \pmb {\varepsilon} _ {1} (\Delta t) \right] \bullet} \\ & {\qquad \pmb {P} (t _ {k} / t _ {k}) [ \pmb {I} + \pmb {F} (t _ {k}) \Delta t + \pmb {\varepsilon} _ {1} (\Delta t) ] ^ {\mathrm{T}} +} \\ & {\qquad [ \pmb {G} (t _ {k}) \Delta t + \pmb {\varepsilon} _ {2} (\Delta t) ] \frac {\pmb {q} (t _ {k})}{\Delta t} [ \pmb {G} (t _ {k}) \Delta t + \pmb {\varepsilon} _ {2} (\Delta t) ] ^ {\mathrm{T}} \} =} \\ & {\qquad \pmb {P} (t _ {k} / t _ {k}) + \pmb {P} (t _ {k} / t _ {k}) \pmb {F} ^ {\mathrm{T}} (t _ {k}) \Delta t + \pmb {F} (t _ {k}) \pmb {P} (t _ {k} / t _ {k}) \Delta t -} \\ & {\qquad \pmb {P} (t _ {k} + \Delta t / t _ {k} + \Delta t) \pmb {H} ^ {\mathrm{T}} (t _ {k} + \Delta t) \pmb {r} ^ {- 1} (t _ {k} + \Delta t) \Delta t \bullet} \\ & {\qquad \pmb {H} (t _ {k} + \Delta t) \pmb {P} (t _ {k} / t _ {k}) + \pmb {G} (t _ {k}) \pmb {q} (t _ {k}) \pmb {G} ^ {\mathrm{T}} (t _ {k}) \Delta t + \pmb {E} (\Delta t)} \end{array}
$$

式中 $E(\Delta t)$ 是 $\Delta t$ 的高阶小量。所以

$$
\begin{array}{r l} \dot {\mathbf {P}} (t / t) & = \lim _ {\Delta t \to 0} \frac {\mathbf {P} (t _ {k} + \Delta t / t _ {k} + \Delta t) - \mathbf {P} (t _ {k} / t _ {k})}{\Delta t} = \\ & \quad \lim _ {\Delta t \to 0} [ \mathbf {P} (t _ {k} / t _ {k}) \mathbf {F} ^ {\mathrm{T}} (t _ {k}) + \mathbf {F} (t _ {k}) \mathbf {P} (t _ {k} / t _ {k}) - \mathbf {P} (t _ {k} + \Delta t / t _ {k} + \Delta t) \cdot \\ & \quad \mathbf {H} ^ {\mathrm{T}} (t _ {k} + \Delta t) \mathbf {r} ^ {- 1} (t _ {k} + \Delta t) \mathbf {H} (t _ {k} + \Delta t) \mathbf {P} (t _ {k} / t _ {k}) + \\ & \quad \mathbf {G} (t _ {k}) \mathbf {q} (t _ {k}) \mathbf {G} ^ {\mathrm{T}} (t _ {k}) + \frac {\mathbf {E} (\Delta t)}{\Delta t} ] \end{array}
$$

注意到 $\frac{E(\Delta t)}{\Delta t}$ 至少是关于 $\Delta t$ 的一阶多项式，当 $\Delta t \to 0$ 时 $t_k$ 成为连续变量，所以

$$
\begin{array}{r l} \dot {\mathbf {P}} (t / t) & = \mathbf {P} (t / t) \mathbf {F} ^ {\mathrm{T}} (t) + \mathbf {F} (t) \mathbf {P} (t / t) - \mathbf {P} (t / t) \mathbf {H} ^ {\mathrm{T}} (t) \mathbf {r} ^ {- 1} (t) \cdot \\ & \quad \mathbf {H} (t) \mathbf {P} (t / t) + \mathbf {G} (t) \mathbf {q} (t) \mathbf {G} ^ {\mathrm{T}} (t) \end{array}
$$

记 $\hat{\boldsymbol{X}}(t)=\hat{\boldsymbol{X}}(t/t),\boldsymbol{P}(t)=\boldsymbol{P}(t/t)$ ，则上述所推方程可写成

1. 2. 3. 4. 5. 6. 7. 8. 9. 10. 11. 12. 13. 14. 15. 16. 17. 18. 19. 20. 21. 22. 23. 24. 25. 26. 27. 28. 29. 30. 31. 32. 33. 34. 35. 36. 37. 38. 39. 40. 41. 42. 43. 44. 45. 46. 47. 48. 49. 50. 51. 52. 53. 54. 55. 56. 57. 58. 59. 60. 61. 62. 63. 64. 65. 66. 67. 68. 69. 70. 71. 72. 73. 74. 75. 76. 77. 78. 79. 80. 81. 82. 83. 84. 85. 86. 87. 88. 89. 90. 91. 92. 93. 94. 95. 96. 97. 98. 99.

$$
\dot {\hat {\mathbf {X}}} (t) = \mathbf {F} (t) \hat {\mathbf {X}} (t) + \mathbf {K} (t) [ \mathbf {Z} (t) - \mathbf {H} (t) \hat {\mathbf {X}} (t) ]\tag{2.3.19}
$$

$$
\boldsymbol {K} (t) = \boldsymbol {P} (t) \boldsymbol {H} ^ {\mathrm{T}} (t) \boldsymbol {r} ^ {- 1} (t)\tag{2.3.20}
$$

$$
\dot {\boldsymbol {P}} (t) = \boldsymbol {P} (t) \boldsymbol {F} ^ {\mathrm{T}} (t) + \boldsymbol {F} (t) \boldsymbol {P} (t) - \boldsymbol {P} (t) \boldsymbol {H} ^ {\mathrm{T}} (t) \boldsymbol {r} ^ {- 1} (t) \boldsymbol {H} (t) \boldsymbol {P} (t) + \boldsymbol {G} (t) \boldsymbol {q} (t) \boldsymbol {G} ^ {\mathrm{T}} (t)\tag{2.3.21}
$$

式(2.3.19)、式(2.3.20)、式(2.3.21)即为连续型卡尔曼滤波基本方程,这是一组矩阵微分方程组,其中式(2.3.21)是关于 $P(t)$ 的非线性矩阵微分方程,称之为黎卡蒂(Riccati)方程。解这组矩阵微分方程的初值选取如下

$$
\begin{array}{r l} & \hat {\mathbf {X}} (t _ {0}) = E [ \mathbf {X} (t _ {0}) ] \\ & \mathbf {P} (t _ {0}) = \mathrm{Var} [ \mathbf {X} (t _ {0}) ] \end{array}
$$

例2-16 $\alpha -\beta$ 滤波。

设被估计信息 $S(t)$ 是时间的近似线性函数, 即可近似为

$$
S (t) = a + b t
$$

但具有模型误差 $\ddot{S}(t)=w(t)$ , $w(t)$ 为零均值的白噪声过程, 协方差为 $E[w(t)w(\tau)]=q\delta(t-\tau)$ 。 $S(t)$ 的量测值为

$$
Z (t) = S (t) + V (t)
$$

量测噪声 $V(t)$ 也是零均值的白噪声过程，协方差为 $E[V(t)V(\tau)] = r\delta(t - \tau)$ 。求卡尔曼滤波器达到稳态时的增益阵 K。

解 选取状态变量

$$
X _ {1} = S (t), X _ {2} = \dot {S} (t)
$$

则状态方程为

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {F} \boldsymbol {X} (t) + \boldsymbol {w} (t)
$$

式中

$$
\boldsymbol {F} = \left[ \begin{array}{l l} 0 & 1 \\ 0 & 0 \end{array} \right], \boldsymbol {w} (t) = \left[ \begin{array}{l} 0 \\ \boldsymbol {w} (t) \end{array} \right]
$$

$$
E [ w (t) ] = 0, E [ w (t) w ^ {\mathrm{T}} (\tau) ] = \left[ \begin{array}{l l} 0 & 0 \\ 0 & q \end{array} \right] \delta (t - \tau)
$$

即

$$
\boldsymbol {q} (t) = \left[ \begin{array}{l l} 0 & 0 \\ 0 & q \end{array} \right]
$$

量测方程为

$$
Z (t) = \mathbf {H} \mathbf {X} + V (t)
$$

式中

$$
\boldsymbol {H} = \left[ \begin{array}{l l} 1 & 0 \end{array} \right], \quad E [ V (t) V (\tau) ] = r \delta (t - \tau)
$$

可以证明系统完全可控和完全可观测,所以卡尔曼滤波器是渐近稳定的。当滤波达到稳定时, $\dot{\boldsymbol{P}}(t)=\boldsymbol{0}$ ,所以黎卡蒂方程为

$$
\boldsymbol {F} \boldsymbol {P} + \boldsymbol {P} \boldsymbol {F} ^ {\mathrm{T}} - \boldsymbol {P} \boldsymbol {H} ^ {\mathrm{T}} r ^ {- 1} \boldsymbol {H} \boldsymbol {P} + q = 0
$$

由此可解得滤波达到稳态时的协方差阵为

$$
\boldsymbol {P} = \left[ \begin{array}{l l} P _ {1 1} & P _ {1 2} \\ P _ {2 1} & P _ {2 2} \end{array} \right] = \left[ \begin{array}{c c} \sqrt {2 r \sqrt {q r}} & \sqrt {q r} \\ \sqrt {q r} & \sqrt {2 q \sqrt {q r}} \end{array} \right]
$$

所以稳态增益阵为

$$
\mathbf {K} = \mathbf {P H} ^ {\mathrm{T}} \mathbf {r} ^ {- 1} = \left[ \begin{array}{c} \sqrt {2 \sqrt {q / r}} \\ \sqrt {q / r} \end{array} \right] = \left[ \begin{array}{c} \alpha \\ \beta \end{array} \right]
$$

例2-17 设线性系统的状态方程为

$$
\dot {X} (t) = F X (t) + w (t)
$$

量测方程为

$$
\mathbf {Z} (t) = \mathbf {H} X (t) + \mathbf {V} (t)
$$

式中， $F$ 为常值， $\mathbf{Z} = \begin{bmatrix} Z_1\\ Z_2 \end{bmatrix},\mathbf{H} = \begin{bmatrix} 1\\ 1 \end{bmatrix},\mathbf{V}(t) = \begin{bmatrix} V_1(t)\\ V_2(t) \end{bmatrix},w(t),V_1(t),V_2(t)$ 是零均值互不相关的白噪声过程，且

$$
E [ w (t) w (\tau) ] = q \delta (t - \tau)
$$

$$
E [ \mathbf {V} (t) \mathbf {V} ^ {\mathrm{T}} (\tau) ] = \left[ \begin{array}{l l} r _ {1} & 0 \\ 0 & r _ {2} \end{array} \right] \delta (t - \tau)
$$

求滤波达到稳态时的状态估计方程。

解 滤波达到稳态时

$$
\dot {P} = F P + P F - P \mathbf {H} ^ {\mathrm{T}} \mathbf {r} ^ {- 1} \mathbf {H} P + q = 0
$$

即

$$
F P + F P - P [ 1 1 ] \left[ \begin{array}{l l} r _ {1} & 0 \\ 0 & r _ {2} \end{array} \right] ^ {- 1} \left[ \begin{array}{l} 1 \\ 1 \end{array} \right] P + q = 0
$$

$$
\left(\frac {1}{r _ {1}} + \frac {1}{r _ {2}}\right) P ^ {2} - 2 F P - q = 0
$$

解得

$$
P = \frac {F + \sqrt {F ^ {2} + q \left(\frac {1}{r _ {1}} + \frac {1}{r _ {2}}\right)}}{\frac {1}{r _ {1}} + \frac {1}{r _ {2}}}
$$

$$
\mathbf {K} = P \mathbf {H} ^ {\mathrm{T}} \mathbf {r} ^ {- 1} = P [ 1 1 ] \left[ \begin{array}{l l} \frac {1}{r _ {1}} & 0 \\ 0 & \frac {1}{r _ {2}} \end{array} \right] = \left[ \begin{array}{l l} \frac {P}{r _ {1}} & \frac {P}{r _ {2}} \end{array} \right]
$$

$$
\dot {\hat {X}} (t) = F \hat {X} (t) + \mathbf {K} [ \mathbf {Z} (t) - \mathbf {H} \hat {X} (t) ]
$$

$$
\dot {\hat {X}} (t) = F \hat {X} (t) + \left[ \frac {P}{r _ {1}} \quad \frac {P}{r _ {2}} \right] \left\{\left[ \begin{array}{l} Z _ {1} (t) \\ Z _ {2} (t) \end{array} \right] - \left[ \begin{array}{l} 1 \\ 1 \end{array} \right] \hat {X} (t) \right\} =
$$

$$
\left(F - \frac {P}{r _ {1}} - \frac {P}{r _ {2}}\right) \hat {X} (t) + \frac {P}{r _ {1}} Z _ {1} (t) + \frac {P}{r _ {2}} Z _ {2} (t)
$$

## 2.3.2 黎卡蒂方程的求解

定理2.11

黎卡蒂方程(见式(2.3.21))

$$
\dot {\boldsymbol {P}} (t) = \boldsymbol {P} (t) \boldsymbol {F} ^ {\mathrm{T}} (t) + \boldsymbol {F} () (t) \boldsymbol {P} (t) - \boldsymbol {P} (t) \boldsymbol {H} ^ {\mathrm{T}} (t) \boldsymbol {r} ^ {- 1} (t) \boldsymbol {H} (t) \boldsymbol {P} (t) + \boldsymbol {G} (t) \boldsymbol {q} (t) \boldsymbol {G} ^ {\mathrm{T}} (t)
$$

在 $\boldsymbol{P}(t_{0})=\operatorname{Var}\boldsymbol{X}(t_{0})=\boldsymbol{P}_{0}$ 初始条件下的解为

$$
\boldsymbol {P} (t) = \boldsymbol {Y} (t) \boldsymbol {\Lambda} ^ {- 1} (t)\tag{2.3.22}
$$

式中 $Y(t)$ 和 $\Lambda(t)$ 为线性矩阵微分方程组

$$
\left. \begin{array}{l} \dot {\mathbf {Y}} (t) = \mathbf {F} (t) \mathbf {Y} (t) + \mathbf {q} (t) \boldsymbol {\Lambda} (t) \\ \dot {\boldsymbol {\Lambda}} (t) = \mathbf {H} ^ {\mathrm{T}} (t) \mathbf {r} ^ {- 1} (t) \mathbf {H} (t) \mathbf {Y} (t) - \mathbf {F} ^ {\mathrm{T}} (t) \boldsymbol {\Lambda} (t) \end{array} \right\}\tag{2.3.23}
$$

满足 $Y(t_{0})=P_{0},\Lambda(t_{0})=I$ 初始条件的解。

证明 设

$$
\boldsymbol {Y} (t) = \boldsymbol {P} (t) \boldsymbol {\Lambda} (t)\tag{2.3.24}
$$

式中 $Y(t)$ 和 $\pmb{\Lambda}(t)$ 满足矩阵微分方程组(2.3.23)。下面证明式(2.3.24)中的 $P(t)$ 为黎卡蒂方程式(2.3.21)的解。

对式 $(2.3.24)$ 两边求导数,得

$$
\dot {\mathbf {Y}} (t) = \dot {\mathbf {P}} (t) \boldsymbol {\Lambda} (t) + \mathbf {P} (t) \dot {\boldsymbol {\Lambda}} (t)
$$

将式 $(2.3.23)$ 关系代入上式,得

$$
\boldsymbol {F} (t) \boldsymbol {Y} (t) + \boldsymbol {q} (t) \boldsymbol {\Lambda} (t) = \dot {\boldsymbol {P}} (t) \boldsymbol {\Lambda} (t) + \boldsymbol {P} (t) [ \boldsymbol {H} (t) \boldsymbol {r} ^ {- 1} (t) \boldsymbol {H} (t) \boldsymbol {Y} (t) - \boldsymbol {F} ^ {\mathrm{T}} (t) \boldsymbol {\Lambda} (t) ]
$$

将式 $(2.3.24)$ 代入上式,得

$$
[ \dot {\boldsymbol {P}} (t) - \boldsymbol {F} (t) \boldsymbol {P} (t) - \boldsymbol {P} (t) \boldsymbol {F} ^ {\mathrm{T}} (t) + \boldsymbol {P} (t) \boldsymbol {H} ^ {\mathrm{T}} (t) \boldsymbol {r} ^ {- 1} (t) \boldsymbol {H} (t) \boldsymbol {P} (t) - \boldsymbol {q} (t) ] \boldsymbol {\Lambda} (t) = \boldsymbol {0}\tag{2.3.25}
$$

又由式(2.3.23)和式(2.3.24)，得

又因为

$$
\begin{array}{r l} \dot {\boldsymbol {\Lambda}} (t) & = [ \boldsymbol {H} ^ {\mathrm{T}} (t) \boldsymbol {r} ^ {- 1} (t) \boldsymbol {H} (t) \boldsymbol {P} (t) - \boldsymbol {F} ^ {\mathrm{T}} (t) ] \boldsymbol {\Lambda} (t) \\ & \quad \boldsymbol {\Lambda} (t _ {0}) = \boldsymbol {I} \end{array}
$$

所以

$$
\pmb {\Lambda} (t) = \exp \int_ {t _ {0}} ^ {t} \pmb {A} (\tau) \mathrm{d} \tau
$$

式中

$$
\mathbf {A} (t) = \mathbf {H} ^ {\mathrm{T}} (t) \mathbf {r} ^ {- 1} (t) \mathbf {H} (t) \mathbf {P} (t) - \mathbf {F} ^ {\mathrm{T}} (t)
$$

可见 $\Lambda(t)$ 总是满秩, 因此由式(2.3.25) 得

$$
\dot {\boldsymbol {P}} (t) - \boldsymbol {F} (t) \boldsymbol {P} (t) - \boldsymbol {P} (t) \boldsymbol {F} ^ {\mathrm{T}} (t) + \boldsymbol {P} (t) \boldsymbol {H} ^ {\mathrm{T}} (t) \boldsymbol {r} ^ {- 1} (t) \boldsymbol {H} (t) \boldsymbol {P} (t) - \boldsymbol {q} (t) = \boldsymbol {0}
$$

这是黎卡蒂方程。可见，由式(2.3.22)确定的 $P(t)$ 是黎卡蒂方程的解。

根据上述分析,可归纳出黎卡蒂方程求解的一般步骤。

(1) 将式(2.3.23)写成矩阵形式如下:

$$
\left[ \begin{array}{c} \dot {\boldsymbol {Y}} (t) \\ \dot {\boldsymbol {\Lambda}} (t) \end{array} \right] = \left[ \begin{array}{c c} \boldsymbol {F} (t) & \boldsymbol {q} (t) \\ \boldsymbol {H} ^ {\mathrm{T}} (t) \boldsymbol {r} ^ {- 1} (t) \boldsymbol {H} (t) & - \boldsymbol {F} ^ {\mathrm{T}} (t) \end{array} \right] \left[ \begin{array}{c} \boldsymbol {Y} (t) \\ \boldsymbol {\Lambda} (t) \end{array} \right]\tag{2.3.26}
$$

其中初始条件为

$$
\left[ \begin{array}{c} \mathbf {Y} (t _ {0}) \\ \boldsymbol {\Lambda} (t _ {0}) \end{array} \right] = \left[ \begin{array}{c} \mathbf {P} _ {0} \\ \mathbf {I} \end{array} \right]
$$

(2) 计算式(2.3.26) 所示系统的一步转移阵得

$$
\Theta (t, t _ {0}) = \left[ \begin{array}{l l} \Theta_ {1 1} (t, t _ {0}) & \Theta_ {1 2} (t, t _ {0}) \\ \Theta_ {2 1} (t, t _ {0}) & \Theta_ {2 2} (t, t _ {0}) \end{array} \right]\tag{2.3.27}
$$

(3) 计算式(2.3.26)的解

$$
\left[ \begin{array}{l} \mathbf {Y} (t) \\ \boldsymbol {\Lambda} (t) \end{array} \right] = \left[ \begin{array}{l l} \boldsymbol {\Theta} _ {1 1} & \boldsymbol {\Theta} _ {1 2} \\ \boldsymbol {\Theta} _ {2 1} & \boldsymbol {\Theta} _ {2 2} \end{array} \right] \left[ \begin{array}{l} \boldsymbol {P} _ {0} \\ \boldsymbol {I} \end{array} \right]
$$

即

$$
\left. \begin{array}{l} \mathbf {Y} (t) = \boldsymbol {\Theta} _ {1 1} \mathbf {P} _ {0} + \boldsymbol {\Theta} _ {1 2} \\ \boldsymbol {\Lambda} (t) = \boldsymbol {\Theta} _ {2 1} \mathbf {P} _ {0} + \boldsymbol {\Theta} _ {2 2} \end{array} \right\}\tag{2.3.28}
$$

(4) 计算黎卡蒂方程的解

$$
\boldsymbol {P} (t) = \boldsymbol {Y} (t) \boldsymbol {\Lambda} ^ {- 1} (t)
$$

例2-18 设系统方程和量测方程分别为

$$
\begin{array}{l} {\dot {\mathbf {X}} (t) = \mathbf {F} (t) \mathbf {X} (t)} \\ {\mathbf {Z} (t) = \mathbf {H} (t) \mathbf {X} (t) + \mathbf {v} (t)} \end{array}
$$

式中

$$
E [ \boldsymbol {v} (t) \boldsymbol {v} ^ {\mathrm{T}} (\tau) ] = \boldsymbol {r} (t) \delta (t - \tau)
$$

求连续型卡尔曼滤波中估计的均方误差阵 $P(t)$ 。

解 由于 $q(t) = 0$ ，所以

$$
\begin{array}{r l} & {\left[ \begin{array}{l} \dot {\mathbf {Y}} (t) \\ \dot {\boldsymbol {\Lambda}} (t) \end{array} \right] = \left[ \begin{array}{c c} \mathbf {F} (t) & \mathbf {0} \\ \mathbf {H} ^ {\mathrm{T}} (t) \mathbf {r} ^ {- 1} (t) \mathbf {H} (t) & - \mathbf {F} ^ {\mathrm{T}} (t) \end{array} \right] \left[ \begin{array}{l} \mathbf {Y} (t) \\ \boldsymbol {\Lambda} (t) \end{array} \right]} \\ & {\quad \dot {\mathbf {Y}} (t) = \mathbf {F} (t) \mathbf {Y} (t) \qquad (\mathbf {Y} (t _ {0}) = \mathbf {P} _ {0})} \end{array}\tag{1}
$$

设方程(1)的转移矩阵为 $\Phi(t,t_{0})$ ，则

$$
\mathbf {Y} (t) = \boldsymbol {\Phi} (t, t _ {0}) \mathbf {P} _ {0}\tag{2}
$$

式(2)与式(2.3.28)比较,得

$$
\begin{array}{r l} \Theta_ {1 1} & = \Phi (t, t _ {0}) \\ \Theta_ {1 2} & = 0 \end{array}
$$

又由于

$$
\begin{array}{r l} \dot {\pmb {\Lambda}} (t) & = \pmb {H} ^ {\mathrm{T}} (t) \pmb {r} ^ {- 1} (t) \pmb {H} (t) \pmb {Y} (t) - \pmb {F} ^ {\mathrm{T}} (t) \pmb {\Lambda} (t) = \\ & - \pmb {F} ^ {\mathrm{T}} (t) \pmb {\Lambda} (t) + \pmb {H} ^ {\mathrm{T}} (t) \pmb {r} ^ {- 1} (t) \pmb {H} (t) \pmb {\Phi} (t, t _ {0}) \pmb {P} _ {0} \\ & \pmb {\Lambda} (t _ {0}) = \pmb {I} \end{array}\tag{3}
$$

可见式(3)相应的齐次方程为式(1)所描述系统的伴随系,式(3)的转移矩阵为 $\boldsymbol{\Phi}^{\mathrm{T}}(t_{0},t)$ 。所以

$$
\pmb {\Lambda} (t) = \pmb {\Phi} ^ {\mathrm{T}} (t _ {0}, t) + \int_ {t _ {0}} ^ {t} \pmb {\Phi} ^ {\mathrm{T}} (\tau , t) \pmb {H} ^ {\mathrm{T}} (\tau) \pmb {r} ^ {- 1} (\tau) \pmb {H} (\tau) \pmb {\Phi} (\tau , t _ {0}) \mathrm{d} \tau \pmb {P} _ {0}
$$

又由于

$$
\begin{array}{r l} & \int_ {t _ {0}} ^ {t} \boldsymbol {\Phi} ^ {\mathrm{T}} (\tau , t) \boldsymbol {H} ^ {\mathrm{T}} (\tau) \boldsymbol {r} ^ {- 1} (\tau) \boldsymbol {H} (\tau) \boldsymbol {\Phi} (\tau , t _ {0}) \mathrm{d} \tau = \\ & \int_ {t _ {0}} ^ {t} [ \boldsymbol {\Phi} (\tau , t _ {0}) \boldsymbol {\Phi} (t _ {0}, t) ] ^ {\mathrm{T}} \boldsymbol {H} ^ {\mathrm{T}} (\tau) \boldsymbol {r} ^ {- 1} (\tau) \boldsymbol {H} (\tau) \boldsymbol {\Phi} (\tau , t _ {0}) \mathrm{d} \tau = \end{array}
$$

$$
\boldsymbol {\Phi} ^ {\mathrm{T}} \left(t _ {0}, t\right) \int_ {t _ {0}} ^ {t} \boldsymbol {\Phi} ^ {\mathrm{T}} \left(\tau , t _ {0}\right) \boldsymbol {H} ^ {\mathrm{T}} (\tau) \boldsymbol {r} ^ {- 1} (\tau) \boldsymbol {H} (\tau) \boldsymbol {\Phi} (\tau , t _ {0}) \mathrm{d} \tau
$$

所以

$$
\boldsymbol {\Lambda} (t) = \boldsymbol {\Phi} ^ {\mathrm{T}} (t _ {0}, t) + \boldsymbol {\Phi} ^ {\mathrm{T}} (t _ {0}, t) \int_ {t _ {0}} ^ {t} \boldsymbol {\Phi} ^ {\mathrm{T}} (\tau , t _ {0}) \boldsymbol {H} ^ {\mathrm{T}} (\tau) \boldsymbol {r} ^ {- 1} (\tau) \boldsymbol {H} (\tau) \boldsymbol {\Phi} (\tau , t _ {0}) \mathrm{d} \tau \boldsymbol {P} _ {0}\tag{4}
$$

式(4)与式(2.3.28)相比较,得

$$
\begin{array}{l} \pmb {\Theta} _ {2 1} = \pmb {\Phi} ^ {\mathrm{T}} (t _ {0}, t) \int_ {t _ {0}} ^ {t} \pmb {\Phi} ^ {\mathrm{T}} (\tau , t _ {0}) \pmb {H} ^ {\mathrm{T}} (\tau) \pmb {r} ^ {- 1} (\tau) \pmb {H} (\tau) \pmb {\Phi} (\tau , t _ {0}) \mathrm{d} \tau \\ \pmb {\Theta} _ {2 2} = \pmb {\Phi} ^ {\mathrm{T}} (t _ {0}, t) \end{array}
$$

根据式 $(2.3.22)$ ，估计的均方误差阵为

$$
\boldsymbol {P} (t) = \boldsymbol {Y} (t) \boldsymbol {\Lambda} ^ {- 1} (t) = \boldsymbol {\Theta} _ {1 1} \boldsymbol {P} _ {0} (\boldsymbol {\Theta} _ {2 1} \boldsymbol {P} _ {0} + \boldsymbol {\Theta} _ {2 2}) ^ {- 1}
$$

## 2.3.3 连续型卡尔曼滤波方程的一般形式

设连续系统的系统方程和量测方程的一般形式为

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {F} (t) \boldsymbol {X} (t) + \boldsymbol {G} (t) \boldsymbol {w} (t) + \boldsymbol {E} (t) \boldsymbol {U} (t)\tag{2.3.29}
$$

$$
\mathbf {Z} (t) = \mathbf {H} (t) \mathbf {X} (t) + \mathbf {Y} (t) + \mathbf {v} (t)\tag{2.3.30}
$$

式中， $Y(t)$ 和 $U(t)$ 为已知的时间函数， $w(t)$ 和 $v(t)$ 的假设除与式(2.3.2)相同外，还假设两者相关，即

$$
E [ \boldsymbol {w} (t) \boldsymbol {v} ^ {\mathrm{T}} (\tau) ] = \boldsymbol {s} (t) \delta (t - \tau)\tag{2.3.31}
$$

对式 $(2.3.29)$ 和式 $(2.3.30)$ 作离散化处理,得

$$
\left. \begin{array}{l} \mathbf {X} (t _ {k} + \Delta t) = \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) \mathbf {X} (t _ {k}) + \boldsymbol {\Gamma} (t _ {k} + \Delta t, t) \mathbf {W} _ {k} + \mathbf {B} (t _ {k} + \Delta t, t _ {k}) \mathbf {U} (t _ {k}) \\ \mathbf {Z} (t _ {k} + \Delta t) = \mathbf {H} (t _ {k} + \Delta t) \mathbf {X} (t _ {k} + \Delta t) + \mathbf {Y} (t _ {k} + \Delta t) + \mathbf {V} _ {k} \end{array} \right\}\tag{2.3.32}
$$

式中 $W_{k}$ 和 $V_{k}$ 是按式(2.3.5)和式(2.3.6)确定的等效白噪声序列，

$$
\begin{array}{r l} & W _ {k} = \frac {1}{\Delta t} \int_ {t _ {k}} ^ {t _ {k} + \Delta t} w (t) \mathrm{d} t \\ & V _ {k} = \frac {1}{\Delta t} \int_ {t _ {k}} ^ {t _ {k} + \Delta t} v (t) \mathrm{d} t \end{array}
$$

根据式 $(2.3.8)$ 和式 $(2.3.9)$

$$
Q _ {k} = \frac {\pmb {q} (t _ {k})}{\Delta t}\tag{2.3.33}
$$

$$
\boldsymbol {R} _ {k} = \frac {\boldsymbol {r} (t _ {k})}{\Delta t}\tag{2.3.34}
$$

仿照式 $(2.3.7)$ 的推导,得

$$
E [ \boldsymbol {W} _ {k} \boldsymbol {V} _ {j} ^ {\mathrm{T}} ] = \frac {\boldsymbol {s} (t _ {k})}{\Delta t} \delta_ {k j}
$$

即

$$
\mathbf {S} _ {k} = \frac {\mathbf {s} (t _ {k})}{\Delta t}\tag{2.3.35}
$$

根据式 $(2.3.17)$ 和式 $(2.3.18)$

$$
\boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) = \boldsymbol {I} + \boldsymbol {F} (t _ {k}) \Delta t + \varepsilon_ {1} (\Delta t)\tag{2.3.36}
$$

$$
\boldsymbol {\Gamma} (t _ {k} + \Delta t, t _ {k}) = \boldsymbol {G} (t _ {k}) \Delta t + \boldsymbol {\varepsilon} _ {2} (\Delta t)\tag{2.3.37}
$$

同理可推得

$$
\boldsymbol {B} (t _ {k} + \Delta t, t _ {k}) = \boldsymbol {E} (t _ {k}) \Delta t + \boldsymbol {\varepsilon} _ {3} (\Delta t)\tag{2.3.38}
$$

又根据式 $(2.2.46)$ ，有

$$
\boldsymbol {J} \left(t _ {k}\right) = \boldsymbol {\Gamma} \left(t _ {k} + \Delta t, t _ {k}\right) \boldsymbol {S} _ {k} \boldsymbol {R} _ {k} ^ {- 1} = \boldsymbol {\Gamma} \left(t _ {k} + \Delta t, t _ {k}\right) \boldsymbol {s} \left(t _ {k}\right) \boldsymbol {r} ^ {- 1} \left(t _ {k}\right)\tag{2.3.39}
$$

因此，按照离散卡尔曼滤波一般方程式(2.2.46)～式(2.2.51)，式(2.3.32)对应的滤波方程为

$$
\begin{array}{r l} & \hat {\mathbf {X}} (t _ {k} + \Delta t / t _ {k}) = \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) \hat {\mathbf {X}} (t _ {k}) + \boldsymbol {B} (t _ {k} + \Delta t, t _ {k}) \boldsymbol {U} (t _ {k}) + \\ & \qquad \boldsymbol {\Gamma} (t _ {k} + \Delta t, t _ {k}) s (t _ {k}) r ^ {- 1} (t _ {k}) [ \mathbf {Z} (t _ {k}) - \mathbf {Y} (t _ {k}) - \mathbf {H} (t _ {k}) \hat {\mathbf {X}} (t _ {k}) ] \\ & \hat {\mathbf {X}} (t _ {k} + \Delta t) = \hat {\mathbf {X}} (t _ {k} + \Delta t / t _ {k}) + \boldsymbol {K} (t _ {k} + \Delta t) [ \mathbf {Z} (t _ {k} + \Delta t) - \mathbf {Y} (t _ {k} + \Delta t) - \\ & \qquad \boldsymbol {H} (t _ {k} + \Delta t) \boldsymbol {X} (t _ {k} + \Delta t / t _ {k}) ] \\ & \boldsymbol {K} (t _ {k} + \Delta t) = \boldsymbol {P} (t _ {k} + \Delta t) \boldsymbol {H} ^ {\mathrm{T}} (t _ {k} + \Delta t) \left[ \frac {\boldsymbol {r} (t _ {k} + \Delta t)}{\Delta t} \right] ^ {- 1} \\ & \boldsymbol {P} (t _ {k} + \Delta t) = [ \boldsymbol {I} - \boldsymbol {K} (t _ {k} + \Delta t) \boldsymbol {H} (t _ {k} + \Delta t) ] \boldsymbol {P} (t _ {k} + \Delta t / t _ {k}) \\ & \boldsymbol {P} (t _ {k} + \Delta t / t _ {k}) = [ \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) - \boldsymbol {J} (t _ {k}) \boldsymbol {H} (t _ {k}) ] \boldsymbol {P} (t _ {k}) [ \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) - \boldsymbol {J} (t _ {k}) \boldsymbol {H} (t _ {k}) ] ^ {\mathrm{T}} + \\ & \qquad \boldsymbol {\Gamma} (t _ {k} + \Delta t, t _ {k})   \frac {\boldsymbol {q} (t _ {k})}{\Delta t} \boldsymbol {\Gamma} ^ {\mathrm{T}} (t _ {k} + \Delta t, t _ {k}) - \boldsymbol {J} (t _ {k} + \Delta t, t _ {k})   \frac {\boldsymbol {s} ^ {\mathrm{T}} (t _ {k})}{\Delta t} \boldsymbol {\Gamma} ^ {\mathrm{T}} (t _ {k} + \Delta t, t _ {k}) \end{array}
$$

所以根据导数定义

$$
\begin{array}{r l} & {\dot {\hat {X}} (t) = \lim _ {\Delta t \to 0} \frac {\hat {X} (t _ {k} + \Delta t) - \hat {X} (t _ {k})}{\Delta t} =} \\ & {\quad \lim _ {\Delta t \to 0} \frac {1}{\Delta t} [ \Phi (t _ {k} + \Delta t, t _ {k}) - I ] \hat {X} (t _ {k}) + \lim _ {\Delta t \to 0} \frac {1}{\Delta t} P (t _ {k} + \Delta t) H ^ {\mathrm{T}} (t _ {k} + \Delta t) \cdot} \\ & {\quad \left[ \frac {r (t _ {k} + \Delta t)}{\Delta t} \right] ^ {- 1} [ Z (t _ {k} + \Delta t) - Y (t _ {k} + \Delta t) - H (t _ {k} + \Delta t) \hat {X} (t _ {k} + \Delta t / t _ {k}) ] +} \\ & {\quad B (t _ {k} + \Delta t, t _ {k}) U (t _ {k}) + I (t _ {k} + \Delta t, t _ {k}) s (t _ {k}) r ^ {- 1} (t _ {k}) \cdot} \\ & {\quad [ Z (t _ {k}) - Y (t _ {k}) - H (t _ {k}) \hat {X} (t _ {k}) ]} \end{array}
$$

将式(2.3.36)、式(2.3.37)、式(2.3.38)代入上式，并注意到 $\pmb{\varepsilon}_1(\Delta t),\pmb{\varepsilon}_2(\Delta t),\pmb{\varepsilon}_3(\Delta t)$ 都是关于 $\Delta t$ 的高阶无穷小，当 $\Delta t\to 0$ 时， $t_k = k\Delta t$ 成为连续变量 $t$ ，所以

$$
\begin{array}{r l} \dot {\hat {X}} (t) & = F (t) \hat {X} (t) + P (t) H ^ {\mathrm{T}} (t) r ^ {- 1} (t) [ Z (t) - Y (t) - H (t) \hat {X} (t) ] + \\ & E (t) U (t) + G (t) s (t) r ^ {- 1} (t) [ Z (t) - Y (t) - H (t) \hat {X} (t) ] = \\ & F (t) \hat {X} (t) + E (t) U (t) + [ P (t) H ^ {\mathrm{T}} (t) + G (t) s (t) ] r ^ {- 1} (t) \cdot \\ & [ Z (t) - Y (t) - H (t) \hat {X} (t) ] \end{array}
$$

令

$$
\bar {\boldsymbol {K}} (t) = \left[ \boldsymbol {P} (t) \boldsymbol {H} ^ {\mathrm{T}} (t) + \boldsymbol {G} (t) \boldsymbol {s} (t) \right] \boldsymbol {r} ^ {- 1} (t)\tag{2.3.40}
$$

则

$$
\begin{array}{r l} & {\dot {\hat {\mathbf {X}}} (t) = \mathbf {F} (t) \hat {\mathbf {X}} (t) + \mathbf {E} (t) \mathbf {U} (t) + \overline {{{\mathbf {K}}}} (t) [ \mathbf {Z} (t) - \mathbf {Y} (t) - \mathbf {H} (t) \hat {\mathbf {X}} (t) ]} \\ & {\mathbf {P} (t _ {k} + \Delta t) = \left\{\mathbf {I} - \mathbf {P} (t _ {k} + \Delta t) \mathbf {H} ^ {\mathrm{T}} (t _ {k} + \Delta t) \left[ \frac {\mathbf {r} (t _ {k} + \Delta t)}{\Delta t} \right] ^ {- 1} \mathbf {H} (t _ {k} + \Delta t) \right\}.} \end{array}\tag{2.3.41}
$$

$$
\begin{array}{l} \{\left[ \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) - \boldsymbol {J} (t _ {k}) \boldsymbol {H} (t _ {k}) \right] \boldsymbol {P} (t _ {k}) \left[ \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) - \boldsymbol {J} (t _ {k}) \boldsymbol {H} (t _ {k}) \right] ^ {\mathrm{T}} + \\ \boldsymbol {\Gamma} (t _ {k} + \Delta t, t _ {k}) \frac {\boldsymbol {q} (t _ {k})}{\Delta t} \boldsymbol {\Gamma} ^ {\mathrm{T}} (t _ {k} + \Delta t, t _ {k}) - \boldsymbol {J} (t _ {k}) \frac {\boldsymbol {s} ^ {\mathrm{T}} (t _ {k})}{\Delta t} \boldsymbol {\Gamma} ^ {\mathrm{T}} (t _ {k} + \Delta t, t _ {k}) \} = \\ \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) \boldsymbol {P} (t _ {k}) \boldsymbol {\Phi} ^ {\mathrm{T}} (t _ {k} + \Delta t, t _ {k}) - \boldsymbol {J} (t _ {k}) \boldsymbol {H} (t _ {k}) \boldsymbol {P} (t _ {k}) [ \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) - \\ \boldsymbol {J} (t _ {k}) \boldsymbol {H} (t _ {k}) ] ^ {\mathrm{T}} - \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) \boldsymbol {P} (t _ {k}) [ \boldsymbol {J} (t _ {k}) \boldsymbol {H} (t _ {k}) ] ^ {\mathrm{T}} - \\ \boldsymbol {P} (t _ {k} + \Delta t) \boldsymbol {H} ^ {\mathrm{T}} (t _ {k} + \Delta t) \left[ \frac {\boldsymbol {r} (t _ {k} + \Delta t)}{\Delta t} \right] ^ {- 1} \boldsymbol {H} (t _ {k} + \Delta t) [ \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) - \\ \boldsymbol {J} (t _ {k}) \boldsymbol {H} (t _ {k}) ] \boldsymbol {P} (t _ {k}) [ \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) - \boldsymbol {J} (t _ {k}) \boldsymbol {H} (t _ {k}) ] ^ {\mathrm{T}} + \\ \left\{\boldsymbol {I} - \boldsymbol {P} (t _ {k} + \Delta t) \boldsymbol {H} ^ {\mathrm{T}} (t _ {k} + \Delta t) \left[ \frac {\boldsymbol {r} (t _ {k} + \Delta t)}{\Delta t} \right] ^ {- 1} \boldsymbol {H} (t _ {k} + \Delta t) \right\}. \\ \{\boldsymbol {\Gamma} (t _ {k} + \Delta t, t _ {k})   \frac {\boldsymbol {q} (t _ {k})}{\Delta t}   \boldsymbol {\Gamma} ^ {\mathrm{T}} (t _ {k} + \Delta t, t _ {k}) - \boldsymbol {J} (t _ {k})   \frac {\boldsymbol {s} ^ {\mathrm{T}} (t _ {k})}{\Delta t}   \boldsymbol {\Gamma} ^ {\mathrm{T}} (t _ {k} + \Delta t, t _ {k}) \} \end{array}
$$

将式 $(2.3.36)$ 、式 $(2.3.37)$ 代入上式，并注意到

$$
\begin{array}{r l} & {\underset {\Delta t \to 0} {\lim} \Phi (t _ {k} + \Delta t, t _ {k}) = I} \\ & {\underset {\Delta t \to 0} {\lim} J (t _ {k}) = \underset {\Delta t \to 0} {\lim} \Gamma (t _ {k} + \Delta t, t _ {k}) s (t _ {k}) r ^ {- 1} (t _ {k}) =} \\ & {\underset {\Delta t \to 0} {\lim} [ \Delta t G (t _ {k}) + \varepsilon_ {2} (\Delta t) ] s (t _ {k}) r ^ {- 1} (t _ {k}) = 0} \end{array}
$$

所以

$$
\begin{array}{r l}\dot {\boldsymbol {P}} (t)&= \lim _ {\Delta t \rightarrow 0} \frac {\boldsymbol {P} (t _ {k} + \Delta t) - \boldsymbol {P} (t _ {k})}{\Delta t} =\\&\lim _ {\Delta t \rightarrow 0} \frac {1}{\Delta t} \left\{\Delta t \boldsymbol {F} (t _ {k}) \boldsymbol {P} (t _ {k}) + \boldsymbol {P} (t _ {k}) \Delta t \boldsymbol {F} ^ {\mathrm{T}} (t _ {k}) - \Delta t \boldsymbol {G} (t _ {k}) \boldsymbol {s} (t _ {k}) \boldsymbol {r} ^ {- 1} (t _ {k}) \cdot \right.\\&\left. \boldsymbol {H} (t _ {k}) \boldsymbol {P} (t _ {k}) [ \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) - \Delta t \boldsymbol {G} (t _ {k}) \boldsymbol {s} (t _ {k}) \boldsymbol {r} ^ {- 1} (t _ {k}) \boldsymbol {H} (t _ {k}) ] ^ {\mathrm{T}} - \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) \cdot \right.\\&\left. \boldsymbol {P} (t _ {k}) [ \Delta t \boldsymbol {G} (t _ {k}) \boldsymbol {s} (t _ {k}) \boldsymbol {r} ^ {- 1} (t _ {k}) \boldsymbol {H} (t _ {k}) ] ^ {\mathrm{T}} - \boldsymbol {P} (t _ {k} + \Delta t) \boldsymbol {H} ^ {\mathrm{T}} (t _ {k} + \Delta t) \Delta t \cdot \right.\\&\left. \boldsymbol {r} ^ {- 1} (t _ {k} + \Delta t) \boldsymbol {H} (t _ {k} + \Delta t) [ \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) - \boldsymbol {J} (t _ {k}) \boldsymbol {H} (t _ {k}) ] \boldsymbol {P} (t _ {k}) \cdot \right.\\&\left[ \right. \boldsymbol {\Phi} (t _ {k} + \Delta t, t _ {k}) - \boldsymbol {J} (t _ {k}) \boldsymbol {H} (t _ {k}) ] ^ {\mathrm{T}} \left. \right\} + \lim _ {\Delta t \rightarrow 0} \frac {1}{\Delta t} \left\{\boldsymbol {I} - \boldsymbol {P} (t _ {k} + \Delta t) \boldsymbol {H} ^ {\mathrm{T}} (t _ {k} + \Delta t) \cdot \right.\\&\left. \Delta t r ^ {- 1} (t _ {k} + \Delta t) \boldsymbol {H} (t _ {k} + \Delta t) \right\} \cdot \left\{\Delta t \boldsymbol {G} (t _ {k})   \frac {\boldsymbol {q} (t _ {k})}{\Delta t}   \Delta t \boldsymbol {G} ^ {\mathrm{T}} (t _ {k}) - \Delta t \boldsymbol {G} (t _ {k})   \cdot \right.\\&s (t _ {k}) r ^ {- 1} (t _ {k})   \frac {\boldsymbol {s} ^ {\mathrm{T}} (t _ {k})}{\Delta t}   \Delta t G ^ {\mathrm{T}} (t _ {k})   +   \lim _ {\Delta t \rightarrow 0} (\Delta t)\\&\end{array}
$$

式中 $\varepsilon(\Delta t)$ 是关于 $\Delta t$ 的高阶小量, 所以

$$
\begin{array}{r l} \dot {\mathbf {P}} (t) & = \mathbf {F} (t) \mathbf {P} (t) + \mathbf {P} (t) \mathbf {F} ^ {\mathrm{T}} (t) - \mathbf {G} (t) \mathbf {s} (t) \mathbf {r} ^ {- 1} (t) \mathbf {H} (t) \mathbf {P} (t) - \\ & \quad \mathbf {P} (t) [ \mathbf {G} (t) \mathbf {s} (t) \mathbf {r} ^ {- 1} (t) \mathbf {H} (t) ] ^ {\mathrm{T}} - \mathbf {P} (t) \mathbf {H} ^ {\mathrm{T}} (t) \mathbf {r} ^ {- 1} (t) \mathbf {H} (t) \mathbf {P} (t) + \\ & \quad \mathbf {G} (t) \mathbf {q} (t) \mathbf {G} ^ {\mathrm{T}} (t) - \mathbf {G} (t) \mathbf {s} (t) \mathbf {r} ^ {- 1} (t) \mathbf {s} ^ {\mathrm{T}} (t) \mathbf {G} ^ {\mathrm{T}} (t) \end{array}
$$

由于

$$
\begin{array}{r l} \overline {{{\boldsymbol {K}}}} (t) \boldsymbol {r} (t) \overline {{{\boldsymbol {K}}}} ^ {\mathrm{T}} (t) & = [ \boldsymbol {P} (t) \boldsymbol {H} ^ {\mathrm{T}} (t) + \boldsymbol {G} (t) \boldsymbol {s} (t) ] \boldsymbol {r} ^ {- 1} (t) \boldsymbol {r} (t) \boldsymbol {r} ^ {- 1} (t) \cdot \\ & [ \boldsymbol {H} (t) \boldsymbol {P} (t) + \boldsymbol {s} ^ {\mathrm{T}} (t) \boldsymbol {G} ^ {\mathrm{T}} (t) ] = \\ & \boldsymbol {P} (t) \boldsymbol {H} ^ {\mathrm{T}} (t) \boldsymbol {r} ^ {- 1} (t) \boldsymbol {H} (t) \boldsymbol {P} (t) + \boldsymbol {G} (t) \boldsymbol {s} (t) \boldsymbol {r} ^ {- 1} (t) \boldsymbol {H} (t) \boldsymbol {P} (t) + \\ & \boldsymbol {P} (t) \boldsymbol {H} ^ {\mathrm{T}} (t) \boldsymbol {r} ^ {- 1} (t) \boldsymbol {s} ^ {\mathrm{T}} (t) \boldsymbol {G} ^ {\mathrm{T}} (t) + \boldsymbol {G} (t) \boldsymbol {s} (t) \boldsymbol {r} ^ {- 1} (t) \boldsymbol {s} ^ {\mathrm{T}} (t) \boldsymbol {G} ^ {\mathrm{T}} (t) \end{array}
$$

所以

$$
\dot {\boldsymbol {P}} (t) = \boldsymbol {F} (t) \boldsymbol {P} (t) + \boldsymbol {P} (t) \boldsymbol {F} ^ {\mathrm{T}} (t) + \boldsymbol {G} (t) \boldsymbol {q} (t) \boldsymbol {G} ^ {\mathrm{T}} (t) - \overline {{\boldsymbol {K}}} (t) \boldsymbol {r} (t) \overline {{\boldsymbol {K}}} ^ {\mathrm{T}} (t)\tag{2.3.42}
$$

式(2.3.40)、式(2.3.41)和式(2.3.42)即为连续型卡尔曼滤波的一般方程。

## 习题

2-1 对确定性状态 $X$ 进行了3次量测，量测方程为

$$
Z _ {1} = 3 = [ 1 1 ] X + V _ {1}
$$

$$
Z _ {2} = 1 = [ 0 1 ] \pmb {X} + V _ {2}
$$

$$
Z _ {3} = 2 = [ 1 0 ] X + V _ {3}
$$

已知量测误差为零均值、方差为 r 的白噪声序列，试分别用一般最小二乘和递推最小二乘估计出 X。

2-2 设 $X$ 为标量随机变量，均值为 $m_X$ ，方差为 $C_X$ ，如果用同一台仪器对 $X$ 直接测量 $k$ 次，得量测值 $Z_1, Z_2, \dots, Z_k$ ，用一般最小二乘估计作为 $X$ 的均值的估计，仪器量测误差的均值为零，方差为 $C_V$ ，试求均值的估计及估计的均方误差。

2-3 设某一电压是正态分布的随机变量，均值为 $m_{e}$ ，方差为 $C_{e}$ 。

（1）用两块较好的电压表同时测量得电压 $Z_{1}$ 和 $Z_{2}$ ，量测误差是零均值的正态分布随机变量，方差为 $C_{V}$ ，求最小方差估计 $\hat{E}$ 和均方误差 $P_{e}$ 。

（2）如果用四块较差的电压表同时测量得的电压为 $Z_{1}, Z_{2}, Z_{3}$ 和 $Z_{4}$ ，量测误差也都是零均值的正态分布随机变量，方差都是 $2C_{V}$ ，求最小方差估计 $\hat{E}$ 和均方误差 $P_{e}$ 。

(3) 比较两种方法的精度。

（4）如果用同一块表顺次测量4次，得到的量测值为 $Z_{1}, Z_{2}, Z_{3}$ 和 $Z_{4}$ ，表的测量误差为零均值正态分布的随机变量，方差为 $C_V$ ，试根据4个量测值确定出相应的4个最小方差估计和均方误差，说明所得结果与(1)、(2)情况所得结果的不同意义。

2-4 设有正态分布的标量平稳随机过程 $X(t)$ ，其相关函数为 $R(\tau)$ ，均值为零， $X(t)$ 能够正确测量到，试根据 $X(t)$ 用最小方差估计的方法来预测 $X(t + T)$ 。

2-5 设 $X$ 和 $N$ 为相互独立的正态随机变量，均值分别为 $m_X$ 和 $m_N$ ，方差分别为 $V_X$ 和 $V_N$ ，量测为 $Z = X + N$ ，测量得 $Z = z$ ，求 $X$ 的最小方差估计 $\hat{X}_{\mathrm{MV}}(z)$ 。

2-6 试根据式(2.1.41)证明式(2.1.42)。

2-7 设 $X$ 是服从正态分布的随机变量，均值为 $m$ ，方差为 $\sigma^2$ ，其中 $\sigma^2$ 已知， $m$ 未知，对 $X$ 作 $n$ 次独立量测，得量测值 $X = [X_1 X_2 \cdots X_n]^T$ ，求 $m$ 的极大似然估计 $m_{\mathrm{ML}}(X)$ 。

2-8 证明下述结论：

(1) 若 X 和 Y 为两个相互独立的随机向量，则 X 和 Y 一定不相关。

(2) 若 X 和 Y 是两个服从正态分布且不相关的随机向量，则 X 和 Y 一定相互独立。

2-9 设 $S(t)$ 和 $N(t)$ 分别为有用信号和干扰信号，两者都是平稳过程，且互不相关，功率谱分别为

$$
S _ {S} (\omega) = \frac {1}{\omega^ {2} + a ^ {2}}, \quad S _ {N} (\omega) = \frac {1}{\omega^ {2} + b ^ {2}}
$$

$a > 0, b > 0$ ，量测为 $Z(t) = S(t) + N(t)$ ，求实现滤波的维纳滤波器的传递函数。

2-10 设 $S(t)$ 的功率谱为

$$
S _ {s} (\omega) = \frac {1}{1 + \omega^ {4}}
$$

求实现最佳预测的维纳滤波器的频率特性。

2-11 试比较线性最小方差估计方程与卡尔曼滤波基本方程的异同。

2-12 设 $X$ 和 $Z$ 为随机向量。如果存在一种估计 $E^{**}[X / Z] = A^1 Z$ ，使

$$
E \left[ (\boldsymbol {X} - \boldsymbol {A} ^ {1} \boldsymbol {Z}) ^ {\mathrm{T}} (\boldsymbol {X} - \boldsymbol {A} ^ {1} \boldsymbol {Z}) \right] \leqslant E \left[ (\boldsymbol {X} - \boldsymbol {A} \boldsymbol {Z}) ^ {\mathrm{T}} (\boldsymbol {X} - \boldsymbol {A} \boldsymbol {Z}) \right]
$$

其中 $\mathbf{A}$ 为任一矩阵，证明：

(1) $A^{1}=R_{XZ}R_{Z}^{-1}$ ，其中 $R_{XZ}=E[XZ^{T}]$ ， $R_{Z}=E[ZZ^{T}]$ ;

(2) 估计有偏；

(3) $E\{(\mathbf{X}-E^{**}[\mathbf{X}/\mathbf{Z}])\mathbf{Z}^{\mathrm{T}}\}=0$ 且只要 $E[(\mathbf{X}-\mathbf{A}\mathbf{Z})\mathbf{Z}^{\mathrm{T}}]=0$ ，则 $A=A^{1}$ 。

2-13 设 $X$ 和 $Z$ 为随机向量，证明：

(1) 若 X 与 AZ + b ( $\forall A$ 和 b) 正交，则 $E^{*}[X/Z] = 0$ ;

(2) 若 X 与 Z 正交, 且 $E[X]=0$ , 则 $E^{*}[X/Z]=0$ 。

2-14 试证明，若 $R_{k}, Q_{k}$ 和 $P_{0}$ 都扩大 $\alpha$ 倍，则卡尔曼滤波基本方程中的增益阵 $K_{k}$ 不变。

2-15 设系统和量测方程分别为

$$
\begin{array}{r} X _ {k + 1} = X _ {k} + W _ {k} \\ Z _ {k} = X _ {k} + V _ {k} \end{array}
$$

$X_{k}$ 和 $Z_{k}$ 都是标量， $\{W_{k}\}$ 和 $\{V_{k}\}$ 都是零均值的白噪声序列，且有

$$
\mathrm{Cov} [ W _ {k}, W _ {j} ] = \delta_ {k j}
$$

$$
\mathrm{Cov} [ V _ {k}, V _ {j} ] = \delta_ {k j}
$$

$W_{k}, V_{k}$ 和 $X_{0}$ 三者互不相关， $m_{X0}=0$ ，量测序列为

$$
\{Z _ {i} \} = \{1, - 2, 4, 3, - 1, 1, 1 \}
$$

试按下述3种情况计算 $\hat{X}_{k + 1 / k}$ 和 $P_{k + 1 / k}$

(1) $P_{0} = \infty$ ; (2) $P_{0} = 1$ ; (3) $P_{0} = 0$ 。

2-16 卫星在空间以恒定的角速度旋转,其角位置每隔 T 秒测量一次,即

$$
Z _ {k} = \theta_ {k} + V _ {k}
$$

式中 $\theta_{k}$ 为卫星在 t=kT 时刻的真实角位置， $V_{k}$ 为量测白噪声，并有

$$
E [ V _ {k} ^ {2} ] = (5 ^ {\circ}) ^ {2} \qquad E [ \theta_ {0} ^ {2} ] = (2 0 ^ {\circ}) ^ {2}
$$

$$
E [ \dot {\theta} _ {0} ^ {2} ] = (2 0 ^ {\circ} / \mathrm{s}) ^ {2} \quad E [ \theta_ {0} ] = E [ \dot {\theta} _ {0} ] = E [ \theta_ {0} \dot {\theta} _ {0} ] = 0
$$

为了估计角位置和角速率，试写出系统的状态方程，并计算 $P_{1/0}, K_1, P_1$ 和 $\hat{X}_1$ 。

2-17 计算机仿真练习题:巡航导弹由舰艇发射,发射点距离目标的距离为 540 km,导弹发射后以 300 m/s 的速度直线等速飞向目标。为了保护目标,目标处装备有监视雷达,并用卡尔曼滤波器估计来袭导弹的距离和速度。雷达以 0.1s 的间隔测量导弹的距离,测量误差为零均值的白噪声,均方根为 10 m。设计卡尔曼滤波器时,距离和速度估计初值取为零,相应的误差均方根分别取 500 km 和 350 m/s,来袭导弹飞行状态的随机性用加速度为白噪声来描述,取方差强度 $q=0.01\ m^{2}/s^{3}$ 。

(1) 设计卡尔曼滤波器；

(2) 作滤波效果的仿真计算(仿真时间:0\~1000s,绘出估计误差曲线)。

2-18 设系统方程和量测方程分别为

$$
\begin{array}{r l} \dot {X} (t) & = - X (t) + w (t) \\ Z (t) & = X (t) + v (t) \end{array}
$$

$X(t)$ 和 $Z(t)$ 均为标量，列写出连续型卡尔曼滤波方程，并求出滤波达到稳态时的估计均方误差和最佳增益。其中 $w(t)$ 和 $v(t)$ 是零均值且互不相关的白噪声过程， $q=2\alpha, r=\alpha$ 。

2-19 若定常连续系统噪声方差强度阵 $q$ 和 $r$ 都增加 $\alpha$ 倍，试证明稳态滤波器的增益 $\overline{K}$ 不变。

2-20 设系统方程和量测方程分别为

$$
\begin{array}{r l} \dot {\mathbf {X}} (t) & = \left[ \begin{array}{l l} 0 & 1 \\ - 1 & 0 \end{array} \right] \mathbf {X} (t) + \left[ \begin{array}{l} 0 \\ 1 \end{array} \right] w (t) \\ Z (t) & = [ 1 0 ] \mathbf {X} (t) + v (t) \end{array}
$$

其中 $w(t)$ 和 $v(t)$ 是零均值且互不相关的白噪声过程, q=2, r=1 。列写出连续型卡尔曼滤波方程, 并求出稳态解。

## 2-21 设连续系统状态方程为

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {F} (t) \boldsymbol {X} (t) + \boldsymbol {E} (t) \boldsymbol {U} (t) + \boldsymbol {G} (t) \boldsymbol {w} (t)
$$

其中 $w(t)$ 是均值为零方差强度为 $q(t)$ 的白噪声过程向量。系统经离散化处理后得

$$
\boldsymbol {X} \left(t _ {k + 1}\right) = \boldsymbol {\Phi} \left(t _ {k + 1}, t _ {k}\right) \boldsymbol {X} \left(t _ {k}\right) + \boldsymbol {B} \left(t _ {k}\right) \boldsymbol {U} \left(t _ {k}\right) + \boldsymbol {\Gamma} \left(t _ {k}\right) \boldsymbol {W} _ {k}
$$

其中

$$
\pmb {\Gamma} (t _ {k}) = \int_ {t _ {k}} ^ {t _ {k + 1}} \pmb {\Phi} (t _ {k + 1}, \tau) \pmb {G} (\tau) \mathrm{d} \tau
$$

$$
\pmb {B} (t _ {k}) = \int_ {t _ {k}} ^ {t _ {k + 1}} \pmb {\Phi} (t _ {k + 1}, \tau) \pmb {E} (\tau) \mathrm{d} \tau
$$

$\Phi(t_{k+1},t_{k})$ 为一步转移阵。证明

$$
\begin{array}{l} \lim _ {T \to 0} \frac {\boldsymbol {\Gamma} (t _ {k})}{T} = \boldsymbol {G} (t) \\ \lim _ {T \to 0} \frac {\boldsymbol {B} (t _ {k})}{T} = \boldsymbol {E} (t) \\ \lim _ {T \to 0} \boldsymbol {\Phi} (t _ {k + 1}, t _ {k}) = \boldsymbol {I} \\ T = t _ {k + 1} - t _ {k}. \end{array}
$$

其中

2-22 设运动体沿某直线运动，t 时刻的位移，速度和加速度分别为 $s(t)$ ， $v(t)$ 和 $a(t)$ 。 $t_{k}$ 时刻的量测为 $Z_{k}=S_{k}+V_{k}$ ，其中 $s_{k}=s(t_{k})$ ， $k=1,2,3,\cdots$ 。运动体机动运动引起的运动不定性用 $a(t)$ 描述，其中

$$
\begin{array}{r l} & E [ a (t) ] = 0, \quad E [ a (t) a (\tau) ] = q \delta (t - \tau) \\ & E [ V _ {k} ] = 0, \quad E [ V _ {k} \quad V _ {l} ] = R \delta_ {k l} \end{array}
$$

量测量的采样周期为 T。列写出 $s_{k}$ 和 $v_{k}$ 的稳态滤波方程。

# 第三章 卡尔曼滤波中的技术处理

在第二章关于卡尔曼滤波方程的推导中，曾明确指出系统驱动噪声和量测噪声都必须是白噪声，但实际上有些系统的驱动噪声和量测噪声是有色的。在此情况下，卡尔曼滤波方程应作如何修改才能适用。

离散型卡尔曼滤波采用了递推算法,每一步计算误差都被保留在递推结果中。如果计算机字长较短,舍入误差将会逐渐积累,引起滤波的计算发散。解决这一问题的对策是什么?

当量测维数 m 和系统维数 n 很大时，卡尔曼滤波计算的求逆阶数很高，与 $n^{3} + n^{2}m$ 成正比的卡尔曼滤波计算量十分巨大，这将使实时计算无法实现。如何寻找同解的滤波算法或精度下降不大的次优滤波算法。

本章将就上述问题和其他一些实际问题进行讨论。

## 3.1 有色噪声的白化

## 3.1.1 白色噪声和有色噪声

若随机过程 $w(t)$ 满足

$$
\left. \begin{array}{l} E [ w (t) ] = 0 \\ E [ w (t) w ^ {\top} (\tau) ] = q \delta (t - \tau) \end{array} \right\}\tag{3.1.1}
$$

则 $w(t)$ 称为白色噪声过程。

式(3.1.1)的第二式即为 $w(t)$ 的自相关函数,即

$$
\boldsymbol {R} _ {w} (t - \tau) = \boldsymbol {q} \delta (t - \tau)\tag{3.1.2}
$$

从式(3.1.1)可看出， $w(t)$ 的均值和自相关函数与时间间隔 $\mu=t-\tau$ 有关，而与时间点 t 无关，所以 $w(t)$ 是平稳过程，式(3.1.2)可写为

$$
\boldsymbol {R} _ {w} (\mu) = \boldsymbol {q} \delta (\mu)
$$

q 称为 $w(t)$ 的方差强度。

因此 $w(t)$ 的功率谱为

$$
\mathbf {S} _ {w} (\omega) = \mathcal {F} \{\mathbf {R} _ {w} (\mu) \} = \int_ {- \infty} ^ {\infty} q \delta (\mu) \mathrm{e} ^ {- \mathrm{j} \omega \mu} \mathrm{d} \mu = \mathbf {q}\tag{3.1.3}
$$

式(3.1.3)说明,白噪声 $w(t)$ 的功率谱在整个频率区间内都为常值q,这与白色光的光谱分布在整个可见光频率区间内的现象是类似的,所以 $w(t)$ 被称作白色噪声过程,且功率谱与方差强度相等。

由式(3.1.1)还可看出,无论时间 $\tau$ 和t靠得多么近,只要 $\tau\neq t$ , $w(\tau)$ 与 $w(t)$ 总不相关,两者没有任何依赖关系,这一特性在时间过程中的体现是信号作直上直下的跳变。

若随机序列 $\{\pmb{W}_k\}$ 满足

$$
\left. \begin{array}{l} E [ \boldsymbol {W} _ {k} ] = \mathbf {0} \\ E [ \boldsymbol {W} _ {k} \boldsymbol {W} _ {j} ^ {\mathrm{T}} ] = Q \delta_ {k j} \end{array} \right\}\tag{3.1.4}
$$

则 $W_{k}$ 称为白色噪声序列。 $W_{k}$ 在时间过程中是出现在离散时间点上的杂乱无章的上下跳动。

白色噪声过程和白色噪声序列有时都简称为白噪声，在具体问题中根据具体含义很容易区分出是过程还是序列。白噪声实际上是一种数学抽象，实际系统中变化跳变剧烈的干扰都可近似看做白噪声。

凡是不满足式(3.1.1)的噪声都称为有色噪声过程。有色噪声的功率谱随频率而变，这与有色光的光谱分布在某一频率段内的现象是类似的，有色一词也由此而得名。

## 3.1.2 有色噪声的建模和白化

有色噪声可看做某一动力学系统在白色噪声驱动下的响应。所谓对有色噪声建模就是确定出这一动力学系统。建模的方法一般有两种：相关函数法和时间序列分析法。

## 1. 相关函数法——成形滤波器法

设有一单位白噪声过程(功率谱为1) $w(t)$ ，输入到传递函数为 $\Phi(p)$ 的线性系统中。根据线性系统理论，对应的输出信号 $N(t)$ 的功率谱密度为

$$
S _ {N} (\omega) = \mid \Phi (\mathrm{j} \omega) \mid^ {2} \times 1 = \Phi (\mathrm{j} \omega) \Phi (- \mathrm{j} \omega)\tag{3.1.5}
$$

因此，如果有色噪声 $N(t)$ 的功率谱可写成 $\Phi (\mathrm{j}\omega)\Phi (-\mathrm{j}\omega)$ 的形式，则 $N(t)$ 可看做传递函数为 $\Phi (p)$ 的线性系统对单位强度白噪声 $\varpi (t)$ 的响应，即 $N(t)$ 可用 $\varpi (t)$ 来表示，这就实现了对有色噪声 $N(t)$ 的白化。 $\Phi (p)$ 是实现白化的关键，被称为成形滤波器。

## 定理3.1

若 $N(t)$ 是各态历经的随机过程，则 $N(t)$ 的功率谱 $S_N(\omega)$ 为偶函数；若 $S_N(\omega)$ 又是有理谱，则 $S_N(\omega)$ 一定可分解成 $\Phi (\mathrm{j}\omega)\Phi (-\mathrm{j}\omega)$ 的形式， $N(t)$ 的成形滤波器为

$$
\Phi (p) = \Phi (\mathrm{j} \omega) \mid_ {\mathrm{j} \omega = p}\tag{3.1.6}
$$

证明

由于 $N(t)$ 是各态历经的随机过程, 所以自相关函数为

$$
R _ {N} (\tau) = E [ N (t) N (t + \tau) ] = \lim _ {T \to \infty} \frac {1}{T} \int_ {0} ^ {T} N (t) N (t + \tau) \mathrm{d} t
$$

令 $t + \tau = \mu$ ，当 $t$ 从 $0\to T$ 时， $\mu$ 即从 $\tau \rightarrow T + \tau$ ，所以

$$
\begin{array}{r l} R _ {N} (\tau) = & \lim _ {T \to \infty} \frac {1}{T} \int_ {\tau} ^ {T + \tau} N (\mu - \tau) N (\mu) \mathrm{d} \mu = \\ & \lim _ {T \to \infty} \frac {1}{T} \int_ {0} ^ {T} N (\mu) N (\mu - \tau) \mathrm{d} \mu + \\ & \lim _ {T \to \infty} \frac {1}{T} \left[ \int_ {T} ^ {T + \tau} N (\mu) N (\mu - \tau) \mathrm{d} \mu - \int_ {0} ^ {\tau} N (\mu) N (\mu - \tau) \mathrm{d} \mu \right] \end{array}
$$

上式方括号内的积分区间长度为有限值 $\tau$ ，积分结果也为有限值，所以

$$
R _ {N} (\tau) = \lim _ {T \to \infty} \frac {1}{T} \int_ {0} ^ {T} N (\mu) N (\mu - \tau) \mathrm{d} \mu = R _ {N} (- \tau)
$$

$N(t)$ 的功率谱为

$$
S _ {N} (\omega) = \mathcal {F} \{R _ {N} (\tau) \} = \int_ {- \infty} ^ {\infty} R _ {N} (\tau) \mathrm{e} ^ {- \mathrm{j} \omega \tau} \mathrm{d} \tau =
$$

$$
\int_ {- \infty} ^ {\infty} R _ {N} (\tau) \cos \omega \tau \mathrm{d} \tau - \mathrm{j} \int_ {- \infty} ^ {\infty} R _ {N} (\tau) \sin \omega \tau \mathrm{d} \tau
$$

式中第二项的被积函数是关于 $\tau$ 的奇函数, 第一项的被积函数是关于 $\tau$ 的偶函数, 积分都在对称区间上进行, 所以

$$
S _ {N} (\omega) = 2 \int_ {0} ^ {\infty} R _ {N} (\tau) \cos \omega \tau \mathrm{d} \tau
$$

可见 $S_{N}(\omega)$ 是 $\omega$ 的偶函数。

若 $S_{N}(\omega)$ 又是有理谱，则 $S_{N}(\omega)$ 一定具有如下形式

$$
\begin{array}{r l} S _ {N} (\omega) & = \frac {b _ {0} \omega^ {2 m} + b _ {1} \omega^ {2 m - 2} + \cdots + b _ {m}}{a _ {0} \omega^ {2 n} + a _ {1} \omega^ {2 n - 2} + \cdots + a _ {n}} = \\ & \frac {b _ {0} [ (\omega^ {2}) ^ {m} + b _ {1} ^ {\prime} (\omega^ {2}) ^ {m - 1} + \cdots + b _ {m} ^ {\prime} ]}{a _ {0} [ (\omega^ {2}) ^ {n} + a _ {1} ^ {\prime} (\omega^ {2}) ^ {n - 1} + \cdots + a _ {n} ^ {\prime} ]} = \\ & \frac {b _ {0} (\omega^ {2} - z _ {1}) (\omega^ {2} - z _ {2}) \cdots (\omega^ {2} - z _ {m})}{a _ {0} (\omega^ {2} - p _ {1}) (\omega^ {2} - p _ {2}) \cdots (\omega^ {2} - p _ {n})} \end{array}
$$

而

$$
\begin{array}{l l} \omega^ {2} - z _ {i} = (\sqrt {- z _ {i}} + \mathrm{j} \omega) (\sqrt {- z _ {i}} - \mathrm{j} \omega) & i = 1, 2, \dots , m \\ \omega^ {2} - p _ {i} = (\sqrt {- p _ {i}} + \mathrm{j} \omega) (\sqrt {- p _ {i}} - \mathrm{j} \omega) & i = 1, 2, \dots , n \end{array}
$$

所以

$$
\begin{array}{r l} S _ {N} (\omega) & = \sqrt {\frac {b _ {0}}{a _ {0}}} \frac {(\sqrt {- z _ {1}} + \mathrm{j} \omega) (\sqrt {- z _ {2}} + \mathrm{j} \omega) \cdots (\sqrt {- z _ {m}} + \mathrm{j} \omega)}{(\sqrt {- p _ {1}} + \mathrm{j} \omega) (\sqrt {- p _ {2}} + \mathrm{j} \omega) \cdots (\sqrt {- p _ {n}} + \mathrm{j} \omega)} \\ & \quad \sqrt {\frac {b _ {0}}{a _ {0}}} \frac {(\sqrt {- z _ {1}} - \mathrm{j} \omega) (\sqrt {- z _ {2}} - \mathrm{j} \omega) \cdots (\sqrt {- z _ {m}} - \mathrm{j} \omega)}{(\sqrt {- p _ {1}} - \mathrm{j} \omega) (\sqrt {- p _ {2}} - \mathrm{j} \omega) \cdots (\sqrt {- p _ {n}} - \mathrm{j} \omega)} \end{array}
$$

令

$$
\Phi (\mathrm{j} \omega) = \sqrt {\frac {b _ {0}}{a _ {0}}} \frac {(\sqrt {- z _ {1}} + \mathrm{j} \omega) (\sqrt {- z _ {2}} + \mathrm{j} \omega) \cdots (\sqrt {- z _ {m}} + \mathrm{j} \omega)}{(\sqrt {- p _ {1}} + \mathrm{j} \omega) (\sqrt {- p _ {2}} + \mathrm{j} \omega) \cdots (\sqrt {- p _ {n}} + \mathrm{j} \omega)}
$$

则

$$
S _ {N} (\omega) = \Phi (\mathrm{j} \omega) \Phi (- \mathrm{j} \omega)
$$

所以成形滤波器为

$$
\Phi (p) = \Phi (\mathrm{j} \omega) \mid_ {\mathrm{j} \omega = p}
$$

对随机过程作建模处理时,一般都假设满足各态历经,即由在一个样本时间过程中采集到的数据计算出相关函数,再由相关函数求出功率谱,然后由功率谱求出成形滤波器,所以这种方法称相关函数法。

## 2. 时间序列分析法

时间序列分析法把平稳的有色噪声序列看做是各时刻相关的序列和各时刻出现的白噪声所组成, 即 k 时刻的有色噪声 $N_{k}$ 为

$$
\begin{array}{r l} N _ {k} & = \varphi_ {1} N _ {k - 1} + \varphi_ {2} N _ {k - 2} + \dots + \varphi_ {p} N _ {k - p} + \\ & W _ {k} - \theta_ {1} W _ {k - 1} - \theta_ {2} W _ {k - 2} - \dots - \theta_ {q} W _ {k - q} \end{array}\tag{3.1.7}
$$

式中， $\varphi_{i}<1(i=1,2,\cdots,p)$ 为自回归参数； $\theta_{i}<1(i=1,2,\cdots,q)$ 为滑动平均参数； $\{W_{k}\}$ 为白噪声序列。

上述表示有色噪声的递推方程称为 $(p,q)$ 阶的自回归滑动平均模型 ARMA $(p,q)$ 。

如果模型中 $\theta_{i}=0\quad(i=1,2,\cdots,q)$ ，则模型简化为

$$
N _ {k} = \varphi_ {1} N _ {k - 1} + \varphi_ {2} N _ {k - 2} + \dots + \varphi_ {p} N _ {k - p} + W _ {k}\tag{3.1.8}
$$

这种模型称为 p 阶自回归模型 AR(p)。

如果模型中 $\varphi_{i}=0\quad(i=1,2,\cdots,p)$ ，则模型简化为

$$
N _ {k} = W _ {k} - \theta_ {1} W _ {k - 1} - \theta_ {2} W _ {k - 2} - \dots - \theta_ {q} W _ {k - q}\tag{3.1.9}
$$

这种模型称为 q 阶滑动平均模型 MA(q)。

时间序列分析法还可推广应用于非平稳的有色噪声。如果非平稳有色噪声序列经过 m 次差分后变成平稳序列，利用时间序列分析法得到这种平稳序列的模型为 ARMA $(p,q)$ ，则非平稳序列的模型称为 $(p,m,q)$ 阶的自回归积分滑动平均模型 ARIMA $(p,m,q)$ ，即

$$
\begin{array}{r l} M _ {k} & = N _ {k} - N _ {k - m} = \varphi_ {1} M _ {k - 1} + \dots + \varphi_ {p} M _ {k - p} + \\ & W _ {k} - \theta_ {1} W _ {k - 1} - \dots - \theta_ {q} W _ {k - q} \end{array}\tag{3.1.10}
$$

不论是平稳的还是非平稳的有色噪声，建模的任务都是确定模型方程中的各项参数值 $\left(\varphi_{i},\theta_{i}\right)$ 和白噪声序列 $\{W_{k}\}$ 的方差值。建模过程一般分两步，首先利用噪声的相关函数和功率谱密度的特性确定出模型的形式（AR，MA或ARMA）；然后用参数估计的方法估计出模型中的各参数值。由于实际的有色噪声模型中p和q的阶数一般都不大于2，因此也可以直接从简单的模型开始拟合，然后根据拟合后的残差大小最后确定模型。具体的建模方法可参阅文献 $[71\sim72]$ 。

模型确定后,还须根据滤波的要求,将模型方程改写成一阶差分方程组或一阶微分方程组。有些简单的模型本身就是一阶差分方程,在离散系统的滤波器设计中可直接使用这种模型方程。如果要应用于连续系统,可以直接将一阶差分方程写成一阶微分方程形式 $^{[73-74]}$ 。

## 3.1.3 几种常见的有色噪声

## 1. 随机常数

连续型随机常数满足方程

$$
\dot {N} (t) = 0\tag{3.1.11}
$$

其中 $N(0)$ 为随机变量。

由式(3.1.11)可直接得出随机常数的成形滤波器,如图3.1.1(a)所示。

离散型随机常数 $N_{k}$ 满足的差分方程为

$$
N _ {k + 1} = N _ {k}\tag{3.1.12}
$$

式中 $N_{0}$ 为随机变量。

## 2. 随机游走

连续型随机游走满足的方程为

$$
\dot {N} (t) = w (t)
$$

(3.1.13)

式中 $w(t)$ 为白噪声过程。随机游走的成形滤波器如图 3.1.1(b) 所示。

下面分析随机游走的均值、相关函数及方差，以便了解其特性。

设

$$
N (0) = 0
$$

$$
E [ w (t) w (\tau) ] = q _ {w} \delta (t - \tau)
$$

则

$$
N (t) = \int_ {0} ^ {t} w (\tau) \mathrm{d} \tau
$$

$N(t)$ 的均值为

$$
E [ N (t) ] = E \left[ \int_ {0} ^ {t} w (\tau) \mathrm{d} \tau \right] = \int_ {0} ^ {t} E [ w (\tau) ] \mathrm{d} \tau = 0
$$

$N(t)$ 的相关函数为

$$
\begin{array}{r l} E [ N (\tau) N (t) ] & = E \left[ \int_ {0} ^ {\tau} w (\mu) \mathrm{d} \mu \int_ {0} ^ {t} w (\lambda) \mathrm{d} \lambda \right] = \\ & \quad \int_ {0} ^ {t} \int_ {0} ^ {\tau} E [ w (\mu) w (\lambda) ] \mathrm{d} \mu \mathrm{d} \lambda = \\ & \quad \int_ {0} ^ {t} \int_ {0} ^ {\tau} q _ {w} \delta (\mu - \lambda) \mathrm{d} \mu \mathrm{d} \lambda \end{array}
$$

![](images/35f56e8a8bff6dc3f7887b2a05b5bf6f734bb1d9ac4fadd299a4cb370a2aa903.jpg)

(c)  
(a)  
![](images/3150f0944e81fa48969eeedc9c89cd26fa73dccf6547800c75081049447a86b8.jpg)

若 $t > \tau$ ，则

(b)

$$
\begin{array}{r l} E [ N (\tau) N (t) ] & = \int_ {0} ^ {\tau} \left[ \int_ {0} ^ {\tau} q _ {w} \delta (\mu - \lambda) \mathrm{d} \mu \right] \mathrm{d} \lambda + \\ & \quad \int_ {\tau^ {+}} ^ {t} \left[ \int_ {0} ^ {\tau} q _ {w} \delta (\mu - \lambda) \mathrm{d} \mu \right] \mathrm{d} \lambda \end{array}
$$

![](images/6cd0831d7ba9fda57b8af521b02a18f734d114bf1ecede22ec843ec23891f758.jpg)

上式第二项中， $\mu$ 在 $[0,\tau]$ 内变化， $\lambda$ 在 $(\tau,t]$ 内变化，即总有 $\mu\neq\lambda,\delta(\mu-\lambda)=0$ ，所以此项积分值为零。

$$
E [ N (\tau) N (t) ] = \int_ {0} ^ {\tau} q _ {w} \mathrm{d} \lambda = q _ {w} \tau
$$

图 3.1.1 几种成形滤波器  
(a) 随机常数；

(3.1.14)

(b) 随机游走；(c) 随机斜坡

若 $\tau > t$ ，则

$$
\begin{array}{r l} E [ N (\tau) N (t) ] & = \int_ {0} ^ {\tau} \int_ {0} ^ {t} q _ {w} \delta (\mu - \lambda) \mathrm{d} \lambda \mathrm{d} \mu = \\ & \quad \int_ {0} ^ {t} \left[ \int_ {0} ^ {t} q _ {w} \delta (\mu - \lambda) \mathrm{d} \lambda \right] \mathrm{d} \mu + \\ & \quad \int_ {t +} ^ {\tau} \left[ \int_ {0} ^ {t} q _ {w} \delta (\mu - \lambda) \mathrm{d} \lambda \right] \mathrm{d} \mu = q _ {w} t \end{array}\tag{3.1.15}
$$

综合式(3.1.14)和式(3.1.15)，得随机游走的相关函数为

$$
E [ N (\tau) N (t) ] = q _ {w} [ t, \tau ] _ {\min}\tag{3.1.16}
$$

上式说明: 时间过程越长, 随机游走的相关就越强烈。

由于随机游走的均值为零,所以在式(3.1.16)中,当 $\tau=t$ 时即可得随机游走的方差

$$
E [ N ^ {2} (t) ] = q _ {w} t\tag{3.1.17}
$$

上式说明,方差随时间而变,所以随机游走是非平稳过程。

由于

$$
N (t) = \int_ {0} ^ {t} w (\tau) \mathrm{d} \tau
$$

所以随机游走序列满足下述方程

$$
\begin{array}{r l} N (t _ {k + 1}) & = \int_ {0} ^ {t _ {k + 1}} w (\tau) \mathrm{d} \tau = \int_ {0} ^ {t _ {k}} w (\tau) \mathrm{d} \tau + \int_ {t _ {k}} ^ {t _ {k + 1}} w (\tau) \mathrm{d} \tau = \\ & N (t _ {k}) + W _ {k} \end{array}\tag{3.1.18}
$$

式中

$$
W _ {k} = \int_ {t _ {k}} ^ {t _ {k + 1}} w (\tau) \mathrm{d} \tau\tag{3.1.19}
$$

仿照式(2.3.5)所确定的序列为白噪声的证明,可以证明由式(3.1.19)确定的 $W_{k}$ 是白噪声序列, $W_{k}$ 的方差为

$$
E [ W _ {k} W _ {j} ] = Q _ {k} \delta_ {k j}\tag{3.1.20}
$$

式中

$$
\begin{array}{r} {Q _ {k} = T q _ {w}} \\ {T = t _ {k + 1} - t _ {k}} \end{array}\tag{3.1.21}
$$

下面再分析随机游走序列的均值和方差。

为书写简便,将式(3.1.18)改写为

$$
N _ {k + 1} = N _ {k} + W _ {k}
$$

由递推关系可得

$$
\begin{array}{r l} N _ {k} & = N _ {k - 1} + W _ {k - 1} = N _ {k - 2} + W _ {k - 2} + W _ {k - 1} = \dots \dots = \\ & N _ {0} + W _ {0} + W _ {1} + \dots + W _ {k - 1} \end{array}
$$

对于 $N_{0} = 0$

$$
N _ {k} = \sum_ {i = 0} ^ {k - 1} W _ {i}
$$

所以均值为

$$
E [ N _ {k} ] = \sum_ {i = 0} ^ {k - 1} E [ W _ {i} ] = 0\tag{3.1.22}
$$

方差为

$$
E \left[ N _ {k} ^ {2} \right] = \left[ \left(W _ {0} + W _ {1} + \dots + W _ {k - 1}\right) \left(W _ {0} + W _ {1} + \dots + W _ {k - 1}\right) \right]
$$

由于 $\{W_{k}\}$ 为白噪声序列，即有式(3.1.20)，所以

$$
E \left[ N _ {k} ^ {2} \right] = E \left[ W _ {0} ^ {2} \right] + E \left[ W _ {1} ^ {2} \right] + \dots + E \left[ W _ {k - 1} ^ {2} \right] = k T q _ {w} = t _ {k} q _ {w}\tag{3.1.23}
$$

上式说明随机游走序列也是非平稳的。

3. 随机斜坡

连续型随机斜坡 $N_{1}(t)$ 满足方程

$$
\left. \begin{array}{l} \dot {N} _ {1} (t) = N _ {2} (t) \\ \dot {N} _ {2} (t) = 0 \end{array} \right\}\tag{3.1.24}
$$

式中 $N_{2}(t)$ 为随机常数。随机斜坡的成形滤波器如图 3.1.1(c) 所示。

设 $N_{2}(t)$ 的初值为随机变量 $N_{2}(0)$ , $N_{1}(t)$ 的初值为零, 则

$$
\begin{array}{l} N _ {1} (t) = t N _ {2} (0) \\ N _ {2} (t) = N _ {2} (0) \end{array}
$$

所以随机斜坡的均值和均方值分别为

$$
\begin{array}{r l} & E [ N _ {1} (t) ] = t E [ N _ {2} (0) ] \\ & E [ N _ {1} ^ {2} (t) ] = t ^ {2} E [ N _ {2} ^ {2} (0) ] \end{array}\tag{3.1.25}
$$

(3.1.26)

上式说明随机斜坡是非平稳过程。

对 $N_{1}(t)$ 和 $N_{2}(t)$ 以 T 为周期进行采样, 得

$$
\begin{array}{l} N _ {1} (k) = k T N _ {2} (0) \\ N _ {2} (k) = N _ {2} (0) \end{array}
$$

而

$$
\begin{array}{l} N _ {1} (k + 1) = (k + 1) T N _ {2} (0) = N _ {1} (k) + T N _ {2} (k) \\ N _ {2} (k + 1) = N _ {2} (k) \end{array}
$$

所以随机斜坡的离散化表达式为

$$
\left. \begin{array}{l} N _ {1} (k + 1) = N _ {1} (k) + T N _ {2} (k) \\ N _ {2} (k + 1) = N _ {2} (k) \end{array} \right\}\tag{3.1.27}
$$

相应的均值和均方值分别为

$$
\begin{array}{r l} & E [ N _ {1} (k) ] = k T E [ N _ {2} (0) ] \\ & E [ N _ {1} ^ {2} (k) ] = k ^ {2} T ^ {2} E [ N _ {2} ^ {2} (0) ] \end{array}
$$

可见离散型随机斜坡也是非平稳的。

4. 一阶马尔柯夫过程

相关函数为

$$
R _ {N} (\tau) = R _ {N} (0) \mathrm{e} ^ {- \alpha | \tau |}\tag{3.1.28}
$$

的有色噪声称为一阶马尔柯夫过程,有时简称为一阶马氏过程。式中: $\tau$ 为两时间点之间的时间间隔; $R_{N}(0)$ 为均方值; $\alpha$ 为反相关时间常数,其物理意义解释如下。

式(3.1.28) 可用图 3.1.2 示意图表示。右半部分曲线在 A 点处的切线斜率和切线方程为

$$
\begin{array}{r l} & R _ {N} ^ {\prime} (\tau) \mid_ {\tau = 0} = - \alpha R _ {N} (0) \\ & y = R _ {N} (0) - \alpha R _ {N} (0) \tau \end{array}
$$

该切线与 $\tau$ 轴的交点为

$$
\tau_ {0} = \frac {1}{\alpha}
$$

![](images/8cf7bf9a8044109417e1c36be32061360bab073b7190a6d260f58b302f2a8991.jpg)

所以 $\frac{1}{\alpha}$ 是相关函数 $R_{N}(\tau)$ 在 $\tau = 0$ 处的切线与 $\tau$ 轴的交点，具有时间的量纲，称之为相关时间。由于 $\alpha$ 是 $\frac{1}{\alpha}$ 的倒数，所以称 $\alpha$ 为图3.1.2 相关函数示意图反相关时间。反相关时间描述了不同时间点上随机信号的关联性， $\alpha$ 越大，则 $\frac{1}{\alpha}$ 越小，相关函数越陡峭，不同时间上随机信号的关联性下降越迅速，信号越接近白噪声。反之则相反。

下面以成形滤波器概念讨论一阶马氏过程的白化。

由式 $(3.1.28)$ 得一阶马氏过程的功率谱为

$$
\begin{array}{r l} S _ {N} (\omega) & = \mathcal {F} \{R _ {N} (\tau) \} = \int_ {- \infty} ^ {\infty} R _ {N} (\tau) \mathrm{e} ^ {- \mathrm{j} \omega \tau} \mathrm{d} \tau = 2 \int_ {0} ^ {\infty} R _ {N} (0) \mathrm{e} ^ {- \alpha \tau} \cos \omega \tau \mathrm{d} \tau = \frac {2 R _ {N} (0) \alpha}{\omega^ {2} + \alpha^ {2}} = \\ & \frac {\sqrt {2 R _ {N} (0) \alpha}}{\alpha + \mathrm{j} \omega} \frac {\sqrt {2 R _ {N} (0) \alpha}}{\alpha - \mathrm{j} \omega} = \Phi (\mathrm{j} \omega) \Phi (- \mathrm{j} \omega) \end{array}
$$

即

$$
\Phi (\mathrm{j} \omega) = \frac {\sqrt {2 R _ {N} (0) \alpha}}{\alpha + \mathrm{j} \omega}
$$

因此一阶马氏过程的成形滤波器的传递函数为

$$
\Phi (p) = \frac {\sqrt {2 R _ {N} (0) \alpha}}{p + \alpha}\tag{3.1.29}
$$

图 3.1.3 是根据式(3.1.29)画出的方块图。其中 $m(t)$ 是单位强度的白噪声过程。

由图 3.1.3 可得一阶马氏过程的时域动力学方程

$$
\dot {N} (t) = - \alpha N (t) + \sqrt {2 R _ {N} (0) \alpha} m (t)
$$

![](images/a74a89ba76609481b436f8bd524f0016e77c196deacd7f67d741bfe3410790b6.jpg)

或

$$
\dot {N} (t) = - \alpha N (t) + w (t)\tag{3.1.30}
$$

图 3.1.3 一阶马氏过程的成形滤波器

式中

$$
\begin{array}{r l} & w (t) = \sqrt {2 R _ {N} (0) \alpha} m (t) \\ & E [ w (t) ] = 0 \\ & E [ w (t) w (\tau) ] = 2 R _ {N} (0) \alpha \delta (t - \tau) \end{array}\tag{3.1.31}
$$

所以 $w(t)$ 的方差强度为

$$
q _ {w} = 2 R _ {N} (0) \alpha\tag{3.1.32}
$$

对式 $(3.1.30)$ 所示连续系统作离散化处理

$$
N _ {k + 1} = \Phi (T) N _ {k} + \int_ {t _ {k}} ^ {t _ {k + 1}} \Phi (t _ {k + 1} - \tau) w (\tau) \mathrm{d} \tau
$$

式中

$$
\left. \begin{array}{l} \Phi (T) = \mathrm{e} ^ {- a T} \\ T = t _ {k + 1} - t _ {k} \end{array} \right\}\tag{3.1.33}
$$

令

$$
W _ {k} = \int_ {t _ {k}} ^ {t _ {k + 1}} \Phi (t _ {k + 1} - \tau) w (\tau) \mathrm{d} \tau\tag{3.1.34}
$$

则得离散型一阶马氏过程的表达式为

$$
N _ {k + 1} = \mathrm{e} ^ {- \alpha T} N _ {k} + W _ {k}\tag{3.1.35}
$$

其中， $W_{k}$ 由式(3.1.34)定义。

下面分析 $W_{k}$ 。

$$
\begin{array}{r l} & E [ W _ {k} ] = \int_ {t _ {k}} ^ {t _ {k + 1}} \Phi (t _ {k + 1} - \tau) E [ w (\tau) ] \mathrm{d} \tau = \\ & \qquad \sqrt {2 R _ {N} (0) \alpha} \int_ {t _ {k}} ^ {t _ {k + 1}} \mathrm{e} ^ {- \alpha (t _ {k + 1} - \tau)} E [ m (\tau) ] \mathrm{d} \tau = 0 \\ & E [ W _ {k} W _ {j} ] = 2 R _ {N} (0) \alpha E \left[ \int_ {t _ {k}} ^ {t _ {k + 1}} \mathrm{e} ^ {- \alpha (t _ {k + 1} - \tau)} m (\tau) \mathrm{d} \tau \int_ {t _ {j}} ^ {t _ {j + 1}} \mathrm{e} ^ {- \alpha (t _ {j + 1} - \sigma)} m (\sigma) \mathrm{d} \sigma \right] = \\ & \qquad 2 R _ {N} (0) \alpha \int_ {t _ {k}} ^ {t _ {k + 1}} \int_ {t _ {j}} ^ {t _ {j + 1}} \mathrm{e} ^ {- \alpha (t _ {k + 1} - \tau)} \delta (\tau - \alpha) \mathrm{e} ^ {- \alpha (t _ {j + 1} - \sigma)} \mathrm{d} \sigma \mathrm{d} \tau \end{array}\tag{3.1.36}
$$

上式中，仅当 $\sigma=\tau$ 时积分才不为零，而 $\sigma\neq\tau$ 时积分恒为零，只有当两个积分区间重合时，即 j=k 时才有可能使 $\sigma=\tau$ 。

对 $j = k$ ，有

$$
\begin{array}{r l} & {E [ W _ {k} W _ {j} ] = 2 R _ {N} (0) \alpha \int_ {t _ {k}} ^ {t _ {k + 1}} \mathrm{e} ^ {- \alpha (t _ {k + 1} - \tau)} \mathrm{e} ^ {- \alpha (t _ {k + 1} - \tau)} \mathrm{d} \tau = \frac {2 R _ {N} (0) \alpha}{2 \alpha} \mathrm{e} ^ {- 2 \alpha t _ {k + 1}} \Big [ \mathrm{e} ^ {2 \alpha t _ {k + 1}} - \mathrm{e} ^ {2 \alpha t _ {k}} \Big ] =} \\ & {\qquad R _ {N} (0) (1 - \mathrm{e} ^ {- 2 \alpha T})} \end{array}
$$

即

$$
E [ W _ {k} W _ {j} ] = R _ {N} (0) (1 - \mathrm{e} ^ {- 2 \alpha T}) \delta_ {k j}\tag{3.1.37}
$$

由式(3.1.36)和式(3.1.37)知 $W_{k}$ 为白噪声序列,方差为

$$
Q _ {k} = R _ {N} (0) (1 - \mathrm{e} ^ {- 2 \alpha T})\tag{3.1.38}
$$

若将式 $(3.1.32)$ 代入式 $(3.1.38)$ ，得

$$
Q _ {k} = \frac {1}{2 \alpha} (1 - \mathrm{e} ^ {- 2 \alpha T}) q _ {w}\tag{3.1.39}
$$

式中 $q_{w}$ 为式(3.1.30)所示连续型一阶马氏过程激励噪声 $w(t)$ 的方差强度。

## 3.2 有色噪声条件下的卡尔曼滤波

## 3.2.1 系统噪声为有色而量测噪声为白色时的卡尔曼滤波

设系统方程和量测方程为

$$
\boldsymbol {X} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {X} _ {k} + \boldsymbol {\Gamma} _ {k} \boldsymbol {W} _ {k}\tag{3.2.1}
$$

$$
\mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k}\tag{3.2.2}
$$

其中量测噪声 $V_{k}$ 为零均值白噪声，系统噪声 $W_{k}$ 为有色噪声，满足方程

$$
\boldsymbol {W} _ {k + 1} = \boldsymbol {\Pi} _ {k + 1, k} \boldsymbol {W} _ {k} + \boldsymbol {\zeta} _ {k}\tag{3.2.3}
$$

$\zeta_{k}$ 为零均值白噪声。

将 $W_{k}$ 也列为状态, 则扩增后的状态为

$$
\boldsymbol {X} _ {k} ^ {a} = \left[ \begin{array}{c} \boldsymbol {X} _ {k} \\ \boldsymbol {W} _ {k} \end{array} \right]
$$

扩增状态后的系统方程和量测方程为

$$
\left[ \begin{array}{c} \boldsymbol {X} _ {k + 1} \\ \boldsymbol {W} _ {k + 1} \end{array} \right] = \left[ \begin{array}{c c} \boldsymbol {\Phi} _ {k + 1, k} & \boldsymbol {\Gamma} _ {k} \\ \boldsymbol {0} & \boldsymbol {\Pi} _ {k + 1, k} \end{array} \right] \left[ \begin{array}{c} \boldsymbol {X} _ {k} \\ \boldsymbol {W} _ {k} \end{array} \right] + \left[ \begin{array}{c} \boldsymbol {0} \\ \boldsymbol {I} \end{array} \right] \boldsymbol {\zeta} _ {k}
$$

$$
\mathbf {Z} _ {k} = \left[ \begin{array}{l l} \mathbf {H} _ {k} & \mathbf {0} \end{array} \right] \left[ \begin{array}{l} \mathbf {X} _ {k} \\ \mathbf {W} _ {k} \end{array} \right] + \mathbf {V} _ {k}
$$

即

$$
\begin{array}{r} \mathbf {X} _ {k + 1} ^ {a} = \boldsymbol {\Phi} _ {k + 1, k} ^ {a} \mathbf {X} _ {k} ^ {a} + \boldsymbol {\Gamma} _ {k} ^ {a} \mathbf {W} _ {k} ^ {a} \\ \mathbf {Z} _ {k} = \mathbf {H} _ {k} ^ {a} \mathbf {X} _ {k} ^ {a} + \mathbf {V} _ {k} \end{array}
$$

式中， $W_{k}^{a}(W_{k}^{a} = \zeta_{k}),\mathbf{V}_{k}$ 都是零均值白噪声，符合卡尔曼滤波基本方程的要求。

由于扩增状态后状态维数增加,所以滤波器阶数增高,计算量增加。

例3-1 指北惯导系统在静基座作初始对准，取北-西-天地理座系为导航坐标系。经过粗对准，北向通道可简化为

$$
\dot {\varphi} _ {\mathrm{w}} = \varepsilon_ {\mathrm{w}}
$$

式中， $\varepsilon_{w}$ 为西向陀螺漂移， $\varphi_{w}$ 为平台沿西向轴的水平姿态误差。 $\varphi_{w}$ 由北向加速度计测量。

$$
Z _ {N} = - g \varphi_ {w} + \nabla_ {N}
$$

假设 $\varepsilon_{w}$ 包含高频、低频和随机常值三种分量， $\nabla_{N}$ 为零均值的白噪声，试列写出适用于卡尔曼滤波的状态方程和量测方程。

解 西向陀螺漂移中的高频、低频和随机常值三种分量可分别用白噪声 $w_{\varepsilon}$ 、一阶马氏过程 $\varepsilon_{m}$ 和随机常数 $\varepsilon_{b}$ 近拟描述，即

$$
\left\{ \begin{array}{l} \dot {\varphi} _ {W} = \varepsilon_ {b} + \varepsilon_ {m} + w _ {\varepsilon} \\ \dot {\varepsilon} _ {b} = 0 \\ \dot {\varepsilon} _ {m} = - \alpha \varepsilon_ {m} + \xi_ {\varepsilon} \end{array} \right.
$$

将 $\varepsilon_{b}$ 和 $\varepsilon_{m}$ 也扩增为状态，则状态方程为

$$
\left[ \begin{array}{l} \dot {\varphi_ {w}} \\ \dot {\varepsilon_ {b}} \\ \dot {\varepsilon_ {m}} \end{array} \right] = \left[ \begin{array}{c c c} 0 & 1 & 1 \\ 0 & 0 & 0 \\ 0 & 0 & - \alpha \end{array} \right] \left[ \begin{array}{l} \varphi_ {w} \\ \varepsilon_ {b} \\ \varepsilon_ {m} \end{array} \right] + \left[ \begin{array}{l} w _ {\varepsilon} \\ 0 \\ \xi_ {\varepsilon} \end{array} \right]
$$

量测方程为

$$
Z _ {N} = \left[ - g 0 0 \right] \left[ \begin{array}{l} \varphi_ {\mathrm{w}} \\ \varepsilon_ {b} \\ \varepsilon_ {m} \end{array} \right] + \nabla_ {N}
$$

经过状态扩增后,系统噪声和量测噪声都成白噪声,符合使用卡尔曼滤波方程的要求。

## 3.2.2 系统噪声为白色而量测噪声为有色时的卡尔曼滤波

在 3.2.1 小节中介绍了采用状态扩增的办法解决系统噪声的白化，对于系统噪声有色而量测噪声白色的情况这种方法是有效的，惟一的代价是滤波器阶数升高，计算量增大。现在要问，对于系统噪声为白色而量测噪声为有色的情况，这种方法是否仍然有效呢？回答是否定的。下面通过分析说明这一点。

设系统方程和量测方程为

$$
\boldsymbol {X} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {X} _ {k} + \boldsymbol {\Gamma} _ {k} \boldsymbol {W} _ {k}\tag{3.2.4}
$$

$$
\mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k}\tag{3.2.5}
$$

式中， $W_{k}$ 为零均值白噪声，方差为 $Q_{k}, V_{k}$ 为有色噪声，满足方程

$$
\boldsymbol {V} _ {k + 1} = \boldsymbol {\Psi} _ {k + 1, k} \boldsymbol {V} _ {k} + \boldsymbol {\xi} _ {k}\tag{3.2.6}
$$

式中， $\xi_{k}$ 为零均值白噪声，方差为 $R_{k}, \xi_{k}$ 与 $W_{k}$ 不相关。

将 $V_{k}$ 也列为状态,则增广后的系统方程为

$$
\left[ \begin{array}{c} \boldsymbol {X} _ {k + 1} \\ \boldsymbol {V} _ {k + 1} \end{array} \right] = \left[ \begin{array}{c c} \boldsymbol {\Phi} _ {k + 1, k} & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {\Psi} _ {k + 1, k} \end{array} \right] \left[ \begin{array}{c} \boldsymbol {X} _ {k} \\ \boldsymbol {V} _ {k} \end{array} \right] + \left[ \begin{array}{c c c} \boldsymbol {\Gamma} _ {k} & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {I} \end{array} \right] \left[ \begin{array}{c} \boldsymbol {W} _ {k} \\ \boldsymbol {\xi} _ {k} \end{array} \right]
$$

量测方程为

$$
\mathbf {Z} _ {k} = \left[ \begin{array}{l l} \mathbf {H} _ {k} & \mathbf {I} \end{array} \right] \left[ \begin{array}{l} \mathbf {X} _ {k} \\ \mathbf {V} _ {k} \end{array} \right]
$$

经状态扩增后，量测方程中无量测噪声，这意味着量测噪声的方差为零。而在卡尔曼滤波方程中，为了保证增益阵中求逆的存在，要求量测噪声的方差阵必须正定，所以经状态扩增后的量测方程是不满足卡尔曼滤波要求的。

事实上,采用量测扩增的方法是解决量测噪声白化的有效途径。下面分别针对离散系统和连续系统讨论这种方法。

1. 离散系统

由量测方程得

$$
\mathbf {V} _ {k} = \mathbf {Z} _ {k} - \mathbf {H} _ {k} \mathbf {X} _ {k}
$$

所以

$$
\begin{array}{r l} & {\mathbf {Z} _ {k + 1} = \mathbf {H} _ {k + 1} \mathbf {X} _ {k + 1} + \mathbf {V} _ {k + 1} =} \\ & {\qquad \mathbf {H} _ {k + 1} [ \pmb {\Phi} _ {k + 1, k} \mathbf {X} _ {k} + \pmb {\Gamma} _ {k} \mathbf {W} _ {k} ] + \pmb {\Psi} _ {k + 1, k} \mathbf {V} _ {k} + \pmb {\xi} _ {k} =} \\ & {\qquad (\mathbf {H} _ {k + 1} \pmb {\Phi} _ {k + 1, k} - \pmb {\Psi} _ {k + 1, k} \mathbf {H} _ {k}) \mathbf {X} _ {k} + \pmb {\Psi} _ {k + 1, k} \mathbf {Z} _ {k} + \mathbf {H} _ {k + 1} \pmb {\Gamma} _ {k} \mathbf {W} _ {k} + \pmb {\xi} _ {k}} \end{array}
$$

即

$$
\mathbf {Z} _ {k + 1} - \boldsymbol {\Psi} _ {k + 1, k} \mathbf {Z} _ {k} = (\mathbf {H} _ {k + 1} \boldsymbol {\Phi} _ {k + 1, k} - \boldsymbol {\Psi} _ {k + 1, k} \mathbf {H} _ {k}) \mathbf {X} _ {k} + \mathbf {H} _ {k + 1} \boldsymbol {\Gamma} _ {k} \mathbf {W} _ {k} + \boldsymbol {\xi} _ {k}
$$

令

$$
\mathbf {Z} _ {k} ^ {*} = \mathbf {Z} _ {k + 1} - \boldsymbol {\Psi} _ {k + 1, k} \mathbf {Z} _ {k}\tag{3.2.7}
$$

$$
\boldsymbol {H} _ {k} ^ {*} = \boldsymbol {H} _ {k + 1} \boldsymbol {\Phi} _ {k + 1, k} - \boldsymbol {\Psi} _ {k + 1, k} \boldsymbol {H} _ {k}\tag{3.2.8}
$$

$$
\boldsymbol {V} _ {k} ^ {*} = \boldsymbol {H} _ {k + 1} \boldsymbol {\Gamma} _ {k} \boldsymbol {W} _ {k} + \boldsymbol {\xi} _ {k}\tag{3.2.9}
$$

则量测方程可写为

$$
\mathbf {Z} _ {k} ^ {*} = \mathbf {H} _ {k} ^ {*} \mathbf {X} _ {k} + \mathbf {V} _ {k} ^ {*}
$$

式中 $\mathbf{V}_k^*$ 的特性分析如下

$$
E [ \mathbf {V} _ {k} ^ {*} ] = \mathbf {H} _ {k + 1} \boldsymbol {\Gamma} _ {k} E [ \mathbf {W} _ {k} ] + E [ \boldsymbol {\xi} _ {k} ] = \mathbf {0}
$$

$$
E \left[ \mathbf {V} _ {k} ^ {*} \mathbf {V} _ {j} ^ {* T} \right] = E \left[ \left(\mathbf {H} _ {k + 1} \boldsymbol {\Gamma} _ {k} \mathbf {W} _ {k} + \boldsymbol {\xi} _ {k}\right) \left(\mathbf {H} _ {j + 1} \boldsymbol {\Gamma} _ {j} \mathbf {W} _ {j} + \boldsymbol {\xi} _ {j}\right) ^ {T} \right] =
$$

$$
\left(\boldsymbol {H} _ {k + 1} \boldsymbol {\Gamma} _ {k} \boldsymbol {Q} _ {k} \boldsymbol {\Gamma} _ {k} ^ {\mathrm{T}} \boldsymbol {H} _ {k + 1} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}\right) \delta_ {k j}
$$

所以 $V_{k}^{*}$ 是零均值的白噪声, 方差为

$$
\boldsymbol {R} _ {k} ^ {*} = \boldsymbol {H} _ {k + 1} \boldsymbol {\Gamma} _ {k} \boldsymbol {Q} _ {k} \boldsymbol {\Gamma} _ {k} ^ {\mathrm{T}} \boldsymbol {H} _ {k + 1} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}\tag{3.2.10}
$$

$$
E \left[ \boldsymbol {W} _ {k} \boldsymbol {V} _ {j} ^ {* T} \right] = E \left[ \boldsymbol {W} _ {k} \left(\boldsymbol {H} _ {j + 1} \boldsymbol {\Gamma} _ {j} \boldsymbol {W} _ {j} + \boldsymbol {\xi} _ {j}\right) ^ {T} \right] = Q _ {k} \boldsymbol {\Gamma} _ {k} ^ {T} \boldsymbol {H} _ {k + 1} ^ {T} \delta_ {k j}\tag{3.2.11}
$$

所以 $V_{k}^{*}$ 与系统噪声 $W_{k}$ 相关，且

$$
\boldsymbol {S} _ {k} = \boldsymbol {Q} _ {k} \boldsymbol {\Gamma} _ {k} ^ {\mathrm{T}} \boldsymbol {H} _ {k + 1} ^ {\mathrm{T}}\tag{3.2.12}
$$

根据 2.2.5 小节介绍的白噪声条件下一步预测一般方程式(2.2.53)～式(2.2.55)，得

$$
\hat {\boldsymbol {X}} _ {k + 1 / k} ^ {*} = \boldsymbol {\Phi} _ {k + 1, k} \hat {\boldsymbol {X}} _ {k / k - 1} ^ {*} + \bar {\boldsymbol {K}} _ {k} ^ {*} \left(\boldsymbol {Z} _ {k} ^ {*} - \boldsymbol {H} _ {k} ^ {*} \hat {\boldsymbol {X}} _ {k / k - 1} ^ {*}\right)\tag{3.2.13}
$$

$$
\boldsymbol {P} _ {k + 1 / k} ^ {*} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k / k - 1} ^ {*} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k} \boldsymbol {Q} _ {k} \boldsymbol {\Gamma} _ {k} ^ {\mathrm{T}} -
$$

$$
\bar {\boldsymbol {K}} _ {k} ^ {*} \left(\boldsymbol {H} _ {k} ^ {*} \boldsymbol {P} _ {k / k - 1} ^ {*} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \boldsymbol {S} _ {k} ^ {\mathrm{T}} \boldsymbol {\Gamma} _ {k} ^ {\mathrm{T}}\right)\tag{3.2.14}
$$

$$
\bar {\boldsymbol {K}} _ {k} ^ {*} = \left(\boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k / k - 1} ^ {*} \boldsymbol {H} _ {k} ^ {* T} + \boldsymbol {\Gamma} _ {k} \boldsymbol {S} _ {k}\right) \left(\boldsymbol {H} _ {k} ^ {*} \boldsymbol {P} _ {k / k - 1} ^ {*} \boldsymbol {H} _ {k} ^ {* T} + \boldsymbol {R} _ {k} ^ {*}\right) ^ {- 1}\tag{3.2.15}
$$

而

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {k + 1 / k} ^ {*} = & E ^ {*} \left[ \mathbf {X} _ {k + 1} / \mathbf {Z} _ {1} ^ {*} \mathbf {Z} _ {2} ^ {*} \dots \mathbf {Z} _ {k} ^ {*} \right] = \\ & E ^ {*} \left[ \mathbf {X} _ {k + 1} / \mathbf {Z} _ {1} \mathbf {Z} _ {2} \dots \mathbf {Z} _ {k} \mathbf {Z} _ {k + 1} \right] = \hat {\mathbf {X}} _ {k + 1} \end{array}
$$

所以

$$
\begin{array}{l} \boldsymbol {P} _ {k + 1 / k} ^ {*} = \boldsymbol {P} _ {k + 1} \\ \overline {{\boldsymbol {K}}} _ {k} ^ {*} = \overline {{\boldsymbol {K}}} _ {k + 1} \end{array}
$$

因此上述一步预测方程实际上就是滤波方程。将式(3.2.7)代入式(3.2.13)，得

$$
\hat {\boldsymbol {X}} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \hat {\boldsymbol {X}} _ {k} + \bar {\boldsymbol {K}} _ {k + 1} \left(\mathbf {Z} _ {k + 1} - \boldsymbol {\Psi} _ {k + 1, k} \mathbf {Z} _ {k} - \boldsymbol {H} _ {k} ^ {*} \hat {\boldsymbol {X}} _ {k}\right)\tag{3.2.16}
$$

$$
\bar {\boldsymbol {K}} _ {k + 1} = \left(\boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {* T} + \boldsymbol {\Gamma} _ {k} \boldsymbol {S} _ {k}\right) \left(\boldsymbol {H} _ {k} ^ {*} \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {* T} + \boldsymbol {R} _ {k} ^ {*}\right) ^ {- 1}\tag{3.2.17}
$$

$$
\boldsymbol {P} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k} \boldsymbol {Q} _ {k} \boldsymbol {\Gamma} _ {k} ^ {\mathrm{T}} - \overline {{\boldsymbol {K}}} _ {k + 1} (\boldsymbol {H} _ {k} ^ {*} \boldsymbol {P} _ {k} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \boldsymbol {S} _ {k} ^ {\mathrm{T}} \boldsymbol {\Gamma} _ {k} ^ {\mathrm{T}})\tag{3.2.18}
$$

式中 $H_{k}^{*}$ 和 $R_{k}^{*}$ 分别按式(3.2.8)和式(3.2.10)确定。

滤波初值 $\hat{X}_0$ 和 $P_0$ 的确定如下：

由式(3.2.16)知，估计 $\hat{\pmb{X}}_1$ 需要有 $Z_{0}$ ，即量测必须从 $k = 0$ 时刻开始，因此可利用 $Z_{0}$ 来估计 $\hat{\pmb{X}}_0$ ，即 $X_{0}$ 的线性最小方差估计

$$
\hat {\mathbf {X}} _ {0} = E ^ {*} \left[ \mathbf {X} _ {0} / \mathbf {Z} _ {0} \right] = m _ {X _ {0}} + C _ {X _ {0} Z _ {0}} C _ {Z _ {0}} ^ {- 1} (\mathbf {Z} _ {0} - m _ {Z _ {0}})
$$

由量测方程

$$
\begin{array}{r l} & {\mathbf {Z} _ {0} = \mathbf {H} _ {0} \mathbf {X} _ {0} + \mathbf {V} _ {0}} \\ & {\mathbf {C} _ {X _ {0} Z _ {0}} = E [ (\mathbf {X} _ {0} - \mathbf {m} _ {X _ {0}}) (\mathbf {H} _ {0} \mathbf {X} _ {0} + \mathbf {V} _ {0} - \mathbf {H} _ {0} \mathbf {m} _ {X _ {0}}) ^ {\mathrm{T}} ] =} \\ & {\qquad E [ (\mathbf {X} _ {0} - \mathbf {m} _ {X _ {0}}) (\mathbf {X} _ {0} - \mathbf {m} _ {X _ {0}}) ^ {\mathrm{T}} \mathbf {H} _ {0} ^ {\mathrm{T}} ] + E [ (\mathbf {X} _ {0} - \mathbf {m} _ {X _ {0}}) \mathbf {V} _ {0} ^ {\mathrm{T}} ]} \end{array}
$$

由于 $V_{0}$ 与 $X_{0}$ 不相关,所以

$$
\begin{array}{r} {\pmb {C} _ {X _ {0} Z _ {0}} = \pmb {C} _ {X _ {0}} \pmb {H} _ {0} ^ {\mathrm{T}}} \\ {\pmb {C} _ {Z _ {0}} = E [ (\pmb {H} _ {0} \pmb {X} _ {0} + \pmb {V} _ {0} - \pmb {H} _ {0} \pmb {m} _ {X _ {0}}) (\pmb {H} _ {0} \pmb {X} _ {0} + \pmb {V} _ {0} - \pmb {H} _ {0} \pmb {m} _ {X _ {0}}) ^ {\mathrm{T}} ] =} \\ {\pmb {H} _ {0} \pmb {C} _ {X _ {0}} \pmb {H} _ {0} ^ {\mathrm{T}} + \pmb {R} _ {0}} \end{array}
$$

所以

$$
\begin{array}{r l} & {\hat {X} _ {0} = m _ {X _ {0}} + C _ {X _ {0}} H _ {0} ^ {\mathrm{T}} (H _ {0} C _ {X _ {0}} H _ {0} ^ {\mathrm{T}} + R _ {0}) ^ {- 1} (Z _ {0} - H _ {0} m _ {X _ {0}})} \\ & {\cdot \tilde {X} _ {0} = X _ {0} - \hat {X} _ {0} =} \\ & {\quad X _ {0} - m _ {X _ {0}} - C _ {X _ {0}} H _ {0} ^ {\mathrm{T}} (H _ {0} C _ {X _ {0}} H _ {0} ^ {\mathrm{T}} + R _ {0}) ^ {- 1} [ H _ {0} (X _ {0} - m _ {X _ {0}}) + V _ {0} ]} \end{array}\tag{3.2.19}
$$

所以

$$
\boldsymbol {P} _ {0} = E \left[ \widetilde {\boldsymbol {X}} _ {0} \widetilde {\boldsymbol {X}} _ {0} ^ {\mathrm{T}} \right] = \boldsymbol {C} _ {X _ {0}} - \boldsymbol {C} _ {X _ {0}} \boldsymbol {H} _ {0} ^ {\mathrm{T}} \left(\boldsymbol {H} _ {0} \boldsymbol {C} _ {X _ {0}} \boldsymbol {H} _ {0} ^ {\mathrm{T}} + \boldsymbol {R} _ {0}\right) ^ {- 1} \boldsymbol {H} _ {0} \boldsymbol {C} _ {X 0}\tag{3.2.20}
$$

使用矩阵反演公式(2.1.23)，上式又可写成

$$
\boldsymbol {P} _ {0} = (\boldsymbol {C} _ {X _ {0}} ^ {- 1} + \boldsymbol {H} _ {0} ^ {\mathrm{T}} \boldsymbol {R} _ {0} ^ {- 1} \boldsymbol {H} _ {0}) ^ {- 1}\tag{3.2.21}
$$

式(3.2.8)、式(3.2.10)、式(3.2.12)、式(3.2.16)～式(3.2.21)即为量测噪声为有色时的白化处理全套算法。算法中，滤波器维数并未增加，但每步滤波必须计算 $H_{k}^{*}$ ， $R_{k}^{*}$ 和 $S_{k}$ ，滤波初值也必须通过计算获得。

2. 连续系统

设系统方程和量测方程为

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {F} (t) \boldsymbol {X} (t) + \boldsymbol {G} (t) \boldsymbol {w} (t)\tag{3.2.22}
$$

$$
\mathbf {Z} (t) = \mathbf {H} (t) \mathbf {X} (t) + \nu (t)\tag{3.2.23}
$$

式中 $v(t)$ 是零均值的有色噪声, 满足下述方程

$$
\dot {\boldsymbol {v}} (t) = \boldsymbol {D} (t) \boldsymbol {v} (t) + \boldsymbol {\xi} (t)\tag{3.2.24}
$$

上述诸式中， $w(t)$ 和 $\xi(t)$ 都是零均值白噪声； $w(t)$ 的方差强度阵为 $q(t)$ ; $\xi(t)$ 的方差强度阵为 $r(t)$ ; $X(0)$ 的均值为 $m_{X}(0)$ 、方差阵为 $C_{X}(0)$ ; $w(t)$ , $\xi(t)$ , $v(0)$ , $X(0)$ 之间互不相关。

与离散系统类似,设置一个包括量测微分的重构量测量 $Z^{*}(t)$

$$
\begin{array}{r l} \mathbf {Z} ^ {*} (t) & = \dot {\mathbf {Z}} (t) - \mathbf {D} (t) \mathbf {Z} (t) = \\ & \quad \dot {\mathbf {H}} (t) \mathbf {X} (t) + \mathbf {H} (t) \dot {\mathbf {X}} (t) + \dot {\mathbf {v}} (t) - \mathbf {D} (t) [ \mathbf {H} (t) \mathbf {X} (t) + \mathbf {v} (t) ] = \\ & \quad [ \dot {\mathbf {H}} (t) + \mathbf {H} (t) \mathbf {F} (t) - \mathbf {D} (t) \mathbf {H} (t) ] \mathbf {X} (t) + \mathbf {H} (t) \mathbf {G} (t) \mathbf {w} (t) + \boldsymbol {\xi} (t) \end{array}
$$

令

$$
\boldsymbol {H} ^ {*} (t) = \dot {\boldsymbol {H}} (t) + \boldsymbol {H} (t) \boldsymbol {F} (t) - \boldsymbol {D} (t) \boldsymbol {H} (t)\tag{3.2.25}
$$

$$
\boldsymbol {v} ^ {*} (t) = \boldsymbol {H} (t) \boldsymbol {G} (t) \boldsymbol {w} (t) + \xi (t)\tag{3.2.26}
$$

则

$$
\mathbf {Z} ^ {*} (t) = \mathbf {H} ^ {*} (t) \mathbf {X} (t) + \mathbf {v} ^ {*} (t)\tag{3.2.27}
$$

$v^{*}(t)$ 的分析如下：

$$
\begin{array}{r l} & E [ \boldsymbol {v} ^ {*} (t) ] = \boldsymbol {H} (t) \boldsymbol {G} (t) E [ \boldsymbol {w} (t) ] + E [ \boldsymbol {\xi} (t) ] = \boldsymbol {0} \\ & E [ \boldsymbol {v} ^ {*} (t) \boldsymbol {v} ^ {* T} (\tau) ] = E \{[ \boldsymbol {H} (t) \boldsymbol {G} (t) \boldsymbol {w} (t) + \boldsymbol {\xi} (t) ] [ \boldsymbol {H} (\tau) \boldsymbol {G} (\tau) \boldsymbol {w} (\tau) + \boldsymbol {\xi} (\tau) ] ^ {T} \} = \\ & [ \boldsymbol {H} (t) \boldsymbol {G} (t) \boldsymbol {q} (t) \boldsymbol {G} ^ {T} (t) \boldsymbol {H} ^ {T} (t) + \boldsymbol {r} (t) ] \delta (t - \tau) \end{array}
$$

所以 $v^{*}(t)$ 是白噪声, 均值为零, 方差强度阵为

$$
\boldsymbol {r} ^ {*} (t) = \boldsymbol {H} (t) \boldsymbol {G} (t) \boldsymbol {q} (t) \boldsymbol {G} ^ {\mathrm{T}} (t) \boldsymbol {H} ^ {\mathrm{T}} (t) + \boldsymbol {r} (t)\tag{3.2.28}
$$

$$
E [ \boldsymbol {w} (t) \boldsymbol {v} ^ {* T} (\tau) ] = E \left\{\boldsymbol {w} (t) \left[ \boldsymbol {H} (t) \boldsymbol {G} (t) \boldsymbol {w} (t) + \boldsymbol {\xi} (t) \right] ^ {T} \right\} = \boldsymbol {q} (t) \boldsymbol {G} ^ {T} (t) \boldsymbol {H} ^ {T} (t) \delta (t - \tau)
$$

所以 $w(t)$ 与 $v^{*}(t)$ 相关，且

$$
\boldsymbol {S} (t) = \boldsymbol {q} (t) \boldsymbol {G} ^ {\mathrm{T}} (t) \boldsymbol {H} ^ {\mathrm{T}} (t)\tag{3.2.29}
$$

根据 2.3.3 小节介绍的连续型卡尔曼滤波的一般形式, 即式(2.3.40)～(2.3.42), 对应于重构量测 $Z^{*}(t)$ , 得滤波方程

$$
\dot {\hat {\mathbf {X}}} (t) = \mathbf {F} (t) \hat {\mathbf {X}} (t) + \bar {\mathbf {K}} (t) [ \dot {\mathbf {Z}} (t) - \mathbf {D} (t) \mathbf {Z} (t) - \mathbf {H} ^ {*} (t) \hat {\mathbf {X}} (t) ]\tag{3.2.30}
$$

$$
\bar {\boldsymbol {K}} (t) = \left[ \boldsymbol {P} (t) \boldsymbol {H} ^ {* T} (t) + \boldsymbol {G} (t) \boldsymbol {S} (t) \right] \boldsymbol {r} ^ {* - 1} (t)\tag{3.2.31}
$$

$$
\dot {\boldsymbol {P}} (t) = \boldsymbol {F} (t) \boldsymbol {P} (t) + \boldsymbol {P} (t) \boldsymbol {F} ^ {\mathrm{T}} (t) - \bar {\boldsymbol {K}} (t) \boldsymbol {r} ^ {*} (t) \bar {\boldsymbol {K}} ^ {\mathrm{T}} (t) + \boldsymbol {G} (t) \boldsymbol {q} (t) \boldsymbol {G} ^ {\mathrm{T}} (t)\tag{3.2.32}
$$

式中 $H^{*}(t)$ , $S(t)$ 和 $r^{*}(t)$ 分别由式(3.2.25)、式(3.2.29)和式(3.2.28)确定。

式(3.2.25)、式(3.2.28)～式(3.2.32)即为连续系统在有色量测噪声条件下的卡尔曼滤波全套方程。从式(3.2.30)看出，为了求得估计，需要对量测作微分处理，由于噪声的微分放大作用，这将带来很大的方法误差。这可通过对式(3.2.30)作适当的变换来避免这种情况。变换是在 $\bar{K}(k)$ 分段连续的条件下进行的。将式(3.2.30)中的 $\bar{K}(t)\dot{Z}(t)$ 用下式表示

$$
\bar {\mathbf {K}} (t) \dot {\mathbf {Z}} (t) = \frac {\mathrm{d}}{\mathrm{d} t} [ \bar {\mathbf {K}} (t) \mathbf {Z} (t) ] - \dot {\bar {\mathbf {K}}} (t) \mathbf {Z} (t)
$$

将上式代入式 $(3.2.30)$ ，得

$$
\frac {\mathrm{d}}{\mathrm{d} t} [ \hat {\mathbf {X}} (t) - \bar {\mathbf {K}} (t) \mathbf {Z} (t) ] = \mathbf {F} (t) \hat {\mathbf {X}} (t) - \bar {\mathbf {K}} (t) [ \mathbf {D} (t) \mathbf {Z} (t) +
$$

$$
\left. \boldsymbol {H} ^ {*} (t) \hat {\boldsymbol {X}} (t) \right] - \dot {\bar {\boldsymbol {K}}} (t) \boldsymbol {Z} (t)\tag{3.2.33}
$$

用式(3.2.31)～式(3.2.33)构成滤波器,避免了对量测的微分处理,图3.2.1即为该滤波器的方块图。

滤波初值 $\hat{X}(0)$ 和 $P(0)$ 确定如下：

$$
\begin{array}{r l} \hat {\mathbf {X}} (0) & = \mathbf {m} _ {X} (0) + \mathbf {C} _ {X} (0) \mathbf {H} ^ {\mathrm{T}} (0) [ \mathbf {H} (0) \mathbf {C} _ {X} (0) \mathbf {H} ^ {\mathrm{T}} (0) + \boldsymbol {\Gamma} (0) ] ^ {- 1} \cdot \\ & [ \mathbf {Z} (0) - \mathbf {H} (0) \mathbf {m} _ {X} (0) ] \end{array}\tag{3.2.34}
$$

$$
\boldsymbol {P} (0) = \boldsymbol {C} _ {X} (0) - \boldsymbol {C} _ {X} (0) \boldsymbol {H} ^ {\mathrm{T}} (0) [ \boldsymbol {H} (0) \boldsymbol {C} _ {X} (0) \boldsymbol {H} ^ {\mathrm{T}} (0) + \boldsymbol {r} (0) ] ^ {- 1} \boldsymbol {H} (0) \boldsymbol {C} _ {X} (0)\tag{3.2.35}
$$

或

$$
\boldsymbol {P} (0) = \left[ \boldsymbol {C} _ {X} ^ {- 1} (0) + \boldsymbol {H} (0) \boldsymbol {r} ^ {- 1} (0) \boldsymbol {H} (0) \right] ^ {- 1}\tag{3.2.36}
$$

对于系统噪声和量测噪声都是有色噪声的情况,可同时采用状态扩增法和量测扩增法处理之,因为经状态扩增后,系统噪声被白化,此时就成为3.2.2小节所讨论的问题。详细的处理步骤此处不再赘述。

![](images/363428addb8c761c91439667e03ae4ef5bce4a7e85d4520c1c0ef4018cf4787f.jpg)  
图 3.2.1 有色量测噪声条件下的连续型卡尔曼滤波器

## 3.3 序贯处理

由卡尔曼滤波基本方程式(2.2.4)知,如果量测向量 $Z_{k}$ 的维数很大,即量测噪声方差阵 $R_{k}$ 的阶数很高,则求取最佳增益阵 $K_{k}$ 时矩阵求逆的阶数就很高。而求逆计算量与矩阵阶数的三次方近似成正比。序贯处理将量测更新中对 $Z_{k}$ 的集中处理分散为对 $Z_{k}$ 各分量组的顺序处理,使对高阶矩阵的求逆转变为对低阶矩阵的求逆,以便有效降低计算量。特别是当量测噪声方差阵为对角阵时,这种分散后的求逆转化为单纯的除法,计算量的降低就更明显了。

下面按量测噪声方差阵为分块对角阵和非分块对角阵两种情况来讨论序贯处理的具体方法。

## 3.3.1 量测噪声方差阵为分块对角阵时的序贯处理

设系统方程和量测方程分别为

$$
\begin{array}{l} \mathbf {X} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \mathbf {X} _ {k} + \mathbf {W} _ {k} \\ \mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k} \end{array}
$$

式中， $W_{k}$ 和 $V_{k}$ 都是零均值白噪声， $V_{k}$ 的方差阵为分块对角阵。

$$
\boldsymbol {R} _ {k} = \left[ \begin{array}{c c c c} \boldsymbol {R} _ {k} ^ {1} & & & \boldsymbol {0} \\ & \boldsymbol {R} _ {k} ^ {2} & & \\ & & \ddots & \\ \boldsymbol {0} & & & \boldsymbol {R} _ {k} ^ {r} \end{array} \right]
$$

式中 $R_{k}^{1}, R_{k}^{2}, \cdots, R_{k}^{r}$ 的阶数分别为 $m_{1}, m_{2}, \cdots, m_{r}; m_{1} + m_{2} + \cdots + m_{r} = m$ 为量测的维数。

将量测改写成

$$
\left[ \begin{array}{c} \mathbf {Z} _ {k} ^ {1} \\ \mathbf {Z} _ {k} ^ {2} \\ \vdots \\ \mathbf {Z} _ {k} ^ {r} \end{array} \right] = \left[ \begin{array}{c} \mathbf {H} _ {k} ^ {1} \\ \mathbf {H} _ {k} ^ {2} \\ \vdots \\ \mathbf {H} _ {k} ^ {r} \end{array} \right] \mathbf {X} _ {k} + \left[ \begin{array}{c} \mathbf {V} _ {k} ^ {1} \\ \mathbf {V} _ {k} ^ {2} \\ \vdots \\ \mathbf {V} _ {k} ^ {r} \end{array} \right]
$$

则

$$
\hat {\mathbf {X}} _ {k} = E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{\mathbf {Z}}} _ {k - 1}, \overline {{\mathbf {Z}}} _ {k} \right] = E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{\mathbf {Z}}} _ {k - 1}, \mathbf {Z} _ {k} ^ {1}, \mathbf {Z} _ {k} ^ {2}, \dots , \mathbf {Z} _ {k} ^ {r} \right]\tag{3.3.1}
$$

式中， $\bar{Z}_{k-1}$ 为前 k-1 个量测， $Z_{k}$ 为第 k 个量测。

式(3.3.1)中, $Z_{k}^{1},Z_{k}^{2},\cdots,Z_{k}^{r}$ 可视为在 k 时刻顺序得到的量测量,但各量测量间的时间间隔为 0,所以不必作时间更新,而仅需作量测更新。记

$$
\hat {\mathbf {X}} _ {k} ^ {i} = E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{\mathbf {Z}}} _ {k - 1}, \mathbf {Z} _ {k} ^ {1}, \dots , \mathbf {Z} _ {k} ^ {i} \right] \quad i = 1, 2, \dots , r
$$

则根据式 $(2.2.4b)$ 的含义,有如下关系式成立:

$$
\hat {\mathbf {X}} _ {k} ^ {r} = E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{\mathbf {Z}}} _ {k - 1}, \mathbf {Z} _ {k} ^ {1}, \mathbf {Z} _ {k} ^ {2}, \dots , \mathbf {Z} _ {k} ^ {r - 1}, \mathbf {Z} _ {k} ^ {r} \right] =
$$

$$
E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{\mathbf {Z}}} _ {k - 1}, \mathbf {Z} _ {k} ^ {1}, \mathbf {Z} _ {k} ^ {2}, \dots , \mathbf {Z} _ {k} ^ {r - 1} \right] + E ^ {* *} \left[ \widetilde {\mathbf {X}} _ {k} ^ {r} / \mathbf {Z} _ {k} ^ {r} \right] =
$$

$$
\hat {\mathbf {X}} _ {k} ^ {r - 1} + E ^ {* *} \left[ \tilde {\mathbf {X}} _ {k} ^ {r} / \mathbf {Z} _ {k} ^ {r} \right]\tag{1}
$$

$$
\hat {\mathbf {X}} _ {k} ^ {r - 1} = E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{\mathbf {Z}}} _ {k - 1}, \mathbf {Z} _ {k} ^ {1}, \mathbf {Z} _ {k} ^ {2}, \dots , \mathbf {Z} _ {k} ^ {r - 2} \right] + E ^ {* *} \left[ \widetilde {\mathbf {X}} _ {k} ^ {r - 1} / \mathbf {Z} _ {k} ^ {r - 1} \right] =
$$

$$
\hat {\mathbf {X}} _ {k} ^ {r - 2} + E ^ {* *} \left[ \tilde {\mathbf {X}} _ {k} ^ {r - 1} / \mathbf {Z} _ {k} ^ {r - 1} \right]\tag{2}
$$

......

$$
\hat {\mathbf {X}} _ {k} ^ {2} = E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{\mathbf {Z}}} _ {k - 1} \mathbf {Z} _ {k} ^ {1} \right] + E ^ {* *} \left[ \widetilde {\mathbf {X}} _ {k} ^ {2} / \overline {{\mathbf {Z}}} _ {k} ^ {2} \right]\tag{3}
$$

$$
\hat {\mathbf {X}} _ {k} ^ {1} = E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{\mathbf {Z}}} _ {k - 1} \right] + E ^ {* *} \left[ \widetilde {\mathbf {X}} _ {k} ^ {1} / \mathbf {Z} _ {k} ^ {1} \right] = \hat {\mathbf {X}} _ {k} ^ {0} + E ^ {* *} \left[ \widetilde {\mathbf {X}} _ {k} ^ {1} / \mathbf {Z} _ {k} ^ {1} \right]\tag{4}
$$

式中 $E^{**}[\widetilde{\mathbf{X}}_k^i /\mathbf{Z}_k^i ]$ 表示根据 $\mathbf{Z}_k^i$ 对 $\widetilde{\mathbf{X}}_k^i$ 所作的修正。显然

$$
\hat {\mathbf {X}} _ {k} ^ {0} = E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{\mathbf {Z}}} _ {k - 1} \right] = \hat {\mathbf {X}} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \hat {\mathbf {X}} _ {k - 1}
$$

$$
\hat {\boldsymbol {X}} _ {k} ^ {r} = \hat {\boldsymbol {X}} _ {k}
$$

因此 $\hat{X}_k$ 的量测更新计算可按上述诸式的(4)、(3)、(2)、(1)的顺序进行，具体算法为

$$
\boldsymbol {K} _ {k} ^ {i} = \boldsymbol {P} _ {k} ^ {i - 1} \boldsymbol {H} _ {k} ^ {i \mathrm{T}} (\boldsymbol {H} _ {k} ^ {i} \boldsymbol {P} _ {k} ^ {i - 1} \boldsymbol {H} _ {k} ^ {i \mathrm{T}} + \boldsymbol {R} _ {k} ^ {i}) ^ {- 1}\tag{3.3.2a}
$$

$$
\hat {\mathbf {X}} _ {k} ^ {i} = \hat {\mathbf {X}} _ {k} ^ {i - 1} + \mathbf {K} _ {k} ^ {i} (\mathbf {Z} _ {k} ^ {i} - \mathbf {H} _ {k} ^ {i} \hat {\mathbf {X}} _ {k} ^ {i - 1})\tag{3.3.2b}
$$

$$
\boldsymbol {P} _ {k} ^ {i} = (\boldsymbol {I} - \boldsymbol {K} _ {k} ^ {i} \boldsymbol {H} _ {k} ^ {i}) \boldsymbol {P} _ {k} ^ {i - 1} \quad i = 1, 2, \dots , r\tag{3.3.2c}
$$

式中

$$
\hat {\boldsymbol {X}} _ {k} ^ {0} = \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1}\tag{3.3.3a}
$$

$$
\boldsymbol {P} _ {k} ^ {0} = \boldsymbol {P} _ {k / k - 1}\tag{3.3.3b}
$$

由于 $R_{k}^{i}$ 为 $m_{i}\times m_{i}$ 方阵，而 $\sum_{i=1}^{n}m_{i}=m$ ，所以逆阵的阶数相对 $R_{k}(m\times m)$ 降低了很多，计算量也随之下降。

当量测向量的维数 m 接近状态变量的维数 n 时，序贯处理对降低计算量是十分明显的，但当 $m \leqslant \frac{n}{2}$ 时，效果就不显著了。

## 3.3.2 量测噪声方差阵为非分块对角阵时的序贯处理

设系统方程和量测方程与 3.3.1 小节所列相同, 对系统噪声和量测噪声的假设也相同, 惟一不同的是量测噪声方差阵为非分块对角阵。

设量测噪声方差阵为 $R_{k}$ 。由于 $R_{k}$ 为正定阵，所以总可分解为下三角平方根的形式

$$
\boldsymbol {R} _ {k} = \boldsymbol {N} _ {k} \boldsymbol {N} _ {k} ^ {\mathrm{T}}\tag{3.3.4}
$$

式中 $N_{k}$ 为下三角阵，且总为非奇异阵。

以 $N_{k}^{-1}$ 左乘量测方程

$$
\boldsymbol {N} _ {k} ^ {- 1} \boldsymbol {Z} _ {k} = \boldsymbol {N} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {X} _ {k} + \boldsymbol {N} _ {k} ^ {- 1} \boldsymbol {V} _ {k}
$$

令

$$
\overline {{{{\mathbf {Z}}}}} _ {k} = \mathbf {N} _ {k} ^ {- 1} \mathbf {Z} _ {k}\tag{3.3.5}
$$

$$
\overline {{{{\boldsymbol {H}}}}} _ {k} = \boldsymbol {N} _ {k} ^ {- 1} \boldsymbol {H} _ {k}\tag{3.3.6}
$$

$$
\overline {{{V}}} _ {k} = N _ {k} ^ {- 1} V _ {k}\tag{3.3.7}
$$

则重构后的量测方程为

$$
\overline {{{{\bf Z}}}} _ {k} = \overline {{{{\bf H}}}} _ {k} {\bf X} _ {k} + \overline {{{{\bf V}}}} _ {k}\tag{3.3.8}
$$

显然 $\bar{V}_{k}$ 是零均值白噪声, $\bar{V}_{k}$ 的方差阵为

$$
\overline {{{\boldsymbol {R}}}} _ {k} = E \left[ \overline {{{\boldsymbol {V}}}} _ {k} \overline {{{\boldsymbol {V}}}} _ {k} ^ {\mathrm{T}} \right] = \boldsymbol {N} _ {k} ^ {- 1} E \left[ \boldsymbol {V} _ {k} \boldsymbol {V} _ {k} ^ {\mathrm{T}} \right] \boldsymbol {N} _ {k} ^ {- \mathrm{T}} = \boldsymbol {N} _ {k} ^ {- 1} \left(\boldsymbol {N} _ {k} \boldsymbol {N} _ {k} ^ {\mathrm{T}}\right) \boldsymbol {N} _ {k} ^ {- \mathrm{T}} = \boldsymbol {I}\tag{3.3.9}
$$

$\overline{Z}_{k}$ 的量测噪声方差阵为单位阵, 因此可采用 3.3.1 节所介绍的序贯处理方法进行滤波计算。将式(3.3.8)写成

$$
\left[ \begin{array}{c} \overline {{Z}} _ {k} ^ {1} \\ \overline {{Z}} _ {k} ^ {2} \\ \vdots \\ \overline {{Z}} _ {k} ^ {m} \end{array} \right] = \left[ \begin{array}{c} \overline {{H}} _ {k} ^ {1} \\ \overline {{H}} _ {k} ^ {2} \\ \vdots \\ \overline {{H}} _ {k} ^ {m} \end{array} \right] X _ {k} + \left[ \begin{array}{c} \overline {{V}} _ {k} ^ {1} \\ \overline {{V}} _ {k} ^ {2} \\ \vdots \\ \overline {{V}} _ {k} ^ {m} \end{array} \right]
$$

式中 $\bar{Z}_{k}^{i}$ 和 $\bar{V}_{k}^{i}(i=1,2,\cdots,m)$ 都是标量， $\bar{H}_{k}^{i}(i=1,2,\cdots,m)$ 为 $1\times m$ 的行向量。

应用式(3.3.2)，得对应于量测为 $\bar{\mathbf{Z}}_k$ 的序贯处理算法为

$$
\boldsymbol {K} _ {k} ^ {i} = \boldsymbol {P} _ {k} ^ {i - 1} \overline {{\boldsymbol {H}}} _ {k} ^ {i \mathrm{T}} (\overline {{\boldsymbol {H}}} _ {k} ^ {i} \boldsymbol {P} _ {k} ^ {i - 1} \overline {{\boldsymbol {H}}} _ {k} ^ {i \mathrm{T}} + 1) ^ {- 1}\tag{3.3.10a}
$$

$$
\hat {\mathbf {X}} _ {k} ^ {i} = \hat {\mathbf {X}} _ {k} ^ {i - 1} + \mathbf {K} _ {k} ^ {i} (\overline {{Z}} _ {k} ^ {i} - \overline {{\mathbf {H}}} _ {k} ^ {i} \hat {\mathbf {X}} _ {k} ^ {i - 1})\tag{3.3.10b}
$$

$$
\boldsymbol {P} _ {k} ^ {i} = (1 - \boldsymbol {K} _ {k} ^ {i} \overline {{\boldsymbol {H}}} _ {k} ^ {i}) \boldsymbol {P} _ {k} ^ {i - 1}
$$

$$
i = 1, 2, \dots , m\tag{3.3.10c}
$$

式中

$$
\left. \begin{array}{l} \hat {\boldsymbol {X}} _ {k} ^ {0} = \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1} \\ \boldsymbol {P} _ {k} ^ {0} = \boldsymbol {P} _ {k / k - 1} \end{array} \right\}\tag{3.3.11a}
$$

(3.3.11b)

式(3.3.10a)中的求逆转化为除法，计算量显然小多了，但必须付出的代价是按式(3.3.4)对 $R_{k}$ 作下三角平方根分解。

## 3.4 信息滤波[119]

设 $\hat{X}$ 为 X 的估计, 估计的均方误差阵为

$$
\boldsymbol {P} = E \left[ (\boldsymbol {X} - \hat {\boldsymbol {X}}) (\boldsymbol {X} - \hat {\boldsymbol {X}}) ^ {\mathrm{T}} \right]
$$

如果估计完全正确, 即 $X = \hat{X}$ , 则 P = 0, $P^{-1} = \infty$ , 这说明 $\hat{X}$ 包含有 X 的全部信息。反之, 若估计误差很大, 则 P 的诸元都会很大, $P^{-1}$ 的诸元就会很小, $\hat{X}$ 包含 X 的信息很少。所以, $P^{-1}$ 可看作衡量 $\hat{X}$ 含有 X 信息的定量衡量值。习惯上将 $I = P^{-1}$ 称作信息矩阵。

设有系统方程和量测方程

$$
\begin{array}{r} \mathbf {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \mathbf {X} _ {k - 1} + \mathbf {W} _ {k - 1} \\ \mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k} \end{array}
$$

其中， $W_{k}$ 和 $V_{k}$ 为不相关零均值白噪声，方差阵分别为 $Q_{k}$ 和 $R_{k}$ 。

由式 $(2.2.4c')$ 、 $(2.2.4e'')$ 和 $(2.2.4d)$ ，得

$$
\mathbf {K} _ {k} = \mathbf {I} _ {k} ^ {- 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1}\tag{3.4.1}
$$

$$
\boldsymbol {I} _ {k} = \boldsymbol {I} _ {k / k - 1} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}\tag{3.4.2}
$$

$$
\boldsymbol {I} _ {k / k - 1} = \left[ \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {I} _ {k - 1} ^ {- 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \boldsymbol {Q} _ {k - 1} \right] ^ {- 1}
$$

根据矩阵求逆反演公式

$$
\left(\boldsymbol {A} _ {1 1} - \boldsymbol {A} _ {1 2} \boldsymbol {A} _ {2 2} ^ {- 1} \boldsymbol {A} _ {2 1}\right) ^ {- 1} = \boldsymbol {A} _ {1 1} ^ {- 1} + \boldsymbol {A} _ {1 1} ^ {- 1} \boldsymbol {A} _ {1 2} \left(\boldsymbol {A} _ {2 2} - \boldsymbol {A} _ {2 1} \boldsymbol {A} _ {1 1} ^ {- 1} \boldsymbol {A} _ {1 2}\right) ^ {- 1} \boldsymbol {A} _ {2 1} \boldsymbol {A} _ {1 1} ^ {- 1}
$$

取则

$$
\boldsymbol {A} _ {1 1} = \boldsymbol {Q} _ {k - 1}, \quad \boldsymbol {A} _ {1 2} = - \boldsymbol {\Phi} _ {k, k - 1}, \quad \boldsymbol {A} _ {2 2} = \boldsymbol {I} _ {k - 1}, \quad \boldsymbol {A} _ {2 1} = \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}}
$$

$$
\boldsymbol {I} _ {k / k - 1} = \boldsymbol {Q} _ {k - 1} ^ {- 1} - \boldsymbol {Q} _ {k - 1} ^ {- 1} \boldsymbol {\Phi} _ {k, k - 1} (\boldsymbol {I} _ {k - 1} + \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \boldsymbol {Q} _ {k - 1} ^ {- 1} \boldsymbol {\Phi} _ {k, k - 1}) ^ {- 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \boldsymbol {Q} _ {k - 1} ^ {- 1}\tag{3.4.3}
$$

综合上述分析,将信息滤波算法整理如下:

$$
\boldsymbol {I} _ {k / k - 1} = \boldsymbol {Q} _ {k - 1} ^ {- 1} - \boldsymbol {Q} _ {k - 1} ^ {- 1} \boldsymbol {\Phi} _ {k, k - 1} (\boldsymbol {I} _ {k - 1} + \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \boldsymbol {Q} _ {k - 1} ^ {- 1} \boldsymbol {\Phi} _ {k, k - 1}) ^ {- 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \boldsymbol {Q} _ {k - 1} ^ {- 1}\tag{3.4.3}
$$

$$
\boldsymbol {I} _ {k} = \boldsymbol {I} _ {k / k - 1} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}\tag{3.4.2}
$$

$$
\boldsymbol {K} _ {k} = \boldsymbol {I} _ {k} ^ {- 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1}\tag{3.4.1}
$$

$$
\hat {\boldsymbol {X}} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1}\tag{2.2.4a}
$$

$$
\hat {\mathbf {X}} _ {k} = \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1}) \quad k = 1, 2 \dots\tag{2.2.4b}
$$

信息滤波使用信息矩阵计算最佳增益。如果对 $X_{0}$ 的统计信息一无所知，就得盲目选取 $\hat{X}_{0}$ ，相应的 $P_{0}$ 就应选得很大，按式(2.2.4)递推计算均方误差阵时可能会产生溢出。而采用信息矩阵后就可避免出现此现象。信息滤波还被用在固定区间平滑算法的推导中，这将在6.1.4节中介绍。

## 3.5 卡尔曼滤波发散的抑制

## 3.5.1 卡尔曼滤波中的发散现象

在卡尔曼滤波计算中,常会出现这样一种现象:当量测值数目 k 不断增大时,按滤波方程计算的估计均方误差阵趋于零或趋于某一稳态值,但估计值相对实际的被估计值的偏差却越来越大,使滤波器逐渐失去估计作用,这种现象称为滤波器的发散。

引起滤波器发散的主要原因有两点：

（1）描述系统动力学特性的数学模型和噪声的统计模型不准确，不能真实地反映物理过程，使模型与获得的量测值不匹配，导致滤波器发散。这种由模型过于粗糙或失真引起的发散称为滤波发散。

（2）卡尔曼滤波是递推过程，随着滤波步数的增加，舍入误差逐渐积累。如果计算机字长不够长，这种积累误差有可能使估计的均方误差阵失去非负定性甚至失去对称性，使增益阵的计算值逐渐失去合适的加权作用而导致发散。这种由计算的舍入误差积累引起的滤波器发散称为计算发散。

为便于读者对发散的直观理解,此处举一简单例子说明之。

例3-2 飞机从高度 $X(0)$ 开始作等速爬高，垂直速度为恒定值 $u$ ，机载高度表每隔1s对高度作一次测量作为卡尔曼滤波器的量测量，测量误差为零均值的白噪声，方差为1。设计卡尔曼滤波器时误以为飞机高度不变，即系统方程错误地取作 $X_{k} = X_{k - 1}$ 。试分析对高度的估计效果。

解 根据题意, $\Phi_{k,k-1}=1,Q_{k}=0,H_{k}=1,R_{k}=1$ 。取滤波初值为 $\hat{X}_{0}=0,P_{0}\rightarrow\infty$ 。则关于 $X_{k}$ 的滤波方程为

$$
\hat {X} _ {k / k - 1} = \Phi_ {k, k - 1} \hat {X} _ {k - 1} = \hat {X} _ {k - 1}\tag{1}
$$

$$
P _ {k / k - 1} = \Phi_ {k, k - 1} P _ {k - 1} \Phi_ {k, k - 1} + Q _ {k - 1} = P _ {k - 1}\tag{2}
$$

由于

$$
P _ {k} ^ {- 1} = P _ {k / k - 1} ^ {- 1} + H _ {k} R _ {k} ^ {- 1} H _ {k} = P _ {k - 1} ^ {- 1} + 1
$$

$$
P _ {k} ^ {- 1} = P _ {0} ^ {- 1} + k = k\tag{3}
$$

所以

$$
P _ {k} = \frac {1}{k}
$$

$$
K _ {k} = P _ {k} H _ {k} R _ {k} ^ {- 1} = \frac {1}{k}\tag{4}
$$

$$
\hat {X} _ {k} = \hat {X} _ {k / k - 1} + K _ {k} (Z _ {k} - H _ {k} \hat {X} _ {k / k - 1}) = \hat {X} _ {k - 1} + \frac {1}{k} (Z _ {k} - \hat {X} _ {k - 1}) =
$$

$$
\frac {k - 1}{k} \hat {X} _ {k - 1} + \frac {1}{k} Z _ {k}\tag{5}
$$

由此得

$$
\hat {X} _ {1} = Z _ {1}
$$

$$
\hat {X} _ {2} = \frac {1}{2} Z _ {1} + \frac {1}{2} Z _ {2} = \frac {1}{2} (Z _ {1} + Z _ {2})
$$

$$
\hat {X} _ {3} = \frac {2}{3} \cdot \frac {1}{2} (Z _ {1} + Z _ {2}) + \frac {1}{3} Z _ {3} = \frac {1}{3} (Z _ {1} + Z _ {2} + Z _ {3})
$$

$$
\hat {X} _ {k} = \frac {1}{k} (Z _ {1} + Z _ {2} + Z _ {3} + \dots + Z _ {k})
$$

由上述结果得

$$
\lim _ {k \to \infty} P _ {k} = \lim _ {k \to \infty} \frac {1}{k} = 0
$$

$$
\lim _ {k \to \infty} K _ {k} = \lim _ {k \to \infty} \frac {1}{k} = 0
$$

下面分析实际的滤波误差。

由于实际高度为

$$
X _ {k} ^ {r} = X _ {k - 1} ^ {r} + u = X _ {0} ^ {r} + k u
$$

式中上标“r”表示真实状态值。所以高度表给出的量测值为

$$
Z _ {k} = X _ {k} + V _ {k} = X _ {0} + k u + V _ {k}
$$

$X_{k}$ 的滤波值为

$$
\hat {X} _ {k} = \frac {1}{k} \sum_ {i = 1} ^ {k} Z _ {i} = \frac {1}{k} \sum_ {i = 1} ^ {k} (X _ {0} + i u + V _ {i}) =
$$

$$
X _ {0} + \frac {k + 1}{2} u + \frac {1}{k} \sum_ {i = 1} ^ {k} V _ {i}
$$

实际的滤波误差为

$$
\widetilde {X} _ {k} = X _ {k} ^ {r} - \hat {X} _ {k} = (X _ {0} + k u) - \left(X _ {0} + \frac {k + 1}{2} u + \frac {1}{k} \sum_ {i = 1} ^ {k} V _ {i}\right) = \frac {k - 1}{2} u - \frac {1}{k} \sum_ {i = 1} ^ {k} V _ {i}
$$

实际的估计均方误差为

$$
\begin{array}{l} P _ {k} ^ {r} = E [ \widetilde {X} _ {k} ^ {2} ] = \frac {(k - 1) ^ {2}}{4} u ^ {2} + \frac {1}{k} \\ \lim _ {k \to \infty} P _ {k} ^ {r} = \lim _ {k \to \infty} \left[ \frac {(k - 1) ^ {2}}{4} u ^ {2} + \frac {1}{k} \right] = \infty \end{array}
$$

该例中发生滤波发散的根本原因是系统模型严重失真，使量测值与模型出现严重的不匹配现象。从计算过程来看，增益 $K_{k}$ 的迅速趋于零也是引起发散的一个重要原因，因为 $K_{k}$ 的迅速下降使当前的量测值 $Z_{k}$ 对滤波值 $\hat{X}_{k}$ 的修正作用迅速丧失，当 $K_{k}=0$ 时，滤波值完全依赖于错误的系统模型，导致滤波发散，直至滤波误差趋于无穷大。事实上，如果上述滤波问题中，当 $K_{k}$ 下降到一定程度时，限制它的下降，即将增益改由下式确定

$$
K _ {k} = \left\{ \begin{array}{l l} \frac {1}{k}, & k \leqslant M \\ \frac {1}{M}, & k > M \end{array} \right.
$$

则可计算得

$$
\lim _ {k \rightarrow \infty} P _ {k} = (M - 1) ^ {2} u ^ {2} + \frac {1}{2 M - 1}
$$

克服了上述滤波发散问题。这就启发我们，当滤波模型不准确时，可通过加大当前量测值的加权系数，相应地降低早期量测值的加权系数来抑制滤波发散现象。以下给出的衰减记忆法和限定记忆法就是在此思路基础上提出来的有效方法。

## 3.5.2 抑制滤波发散的方法

## 1. 衰减记忆法滤波

从 3.5.1 节的讨论中已看到, 当系统模型不准确时, 新鲜量测值对估计值的修正作用下降, 陈旧量测值的修正作用相对上升是引发滤波发散的一个重要因素。因此逐渐减小陈旧量测值的权重, 相应地增大新鲜量测值的权重, 这是扼制滤波发散的一个可行途径, 衰减记忆法滤波是通过这种途径扼制滤波发散的一种次优滤波方法。

设系统的状态模型与量测模型方程分别为

$$
\boldsymbol {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {X} _ {k - 1} + \boldsymbol {W} _ {k - 1}\tag{3.5.1}
$$

$$
\mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k}\tag{3.5.2}
$$

式中， $W_{k}$ 和 $V_{k}$ 都是零均值白噪声，方差阵分别为 $Q_{k}$ 和 $R_{k}$ ，初始状态的统计特性为 $E[X_{0}] = m_{X_{0}}$ ， $\operatorname{Var}[X_{0}] = P_{0}$ ，其中 $X_{0}, W_{k}, V_{k}$ 三者互不相关。

下面建立 $X_{k}$ 的衰减记忆滤波方程组。

$R_{k}^{-1}$ 和 $P_{0}^{-1}$ 是定量描述 $Z_{k}$ 和 $\hat{X}_{0}$ 中有用信息含量的信息质量标志，而卡尔曼滤波在计算 $\hat{X}_{N}$ 时能根据这些信息质量标志自动地确定出对 $\{Z_{k}\}(k\leqslant N)$ 和 $\hat{X}_{0}$ 的利用程度。这就给人以启示，要减小 $Z_{i}(i<N)$ 及 $\hat{X}_{0}$ 对 $\hat{X}_{N}$ 的影响，可通过增大 $R_{i}$ 及 $P_{0}$ 的值来实现。因此，为要建立系统（3.5.1）、（3.5.2）的衰减记忆滤波方程，可先修改噪声的统计模型。

$$
\boldsymbol {X} _ {k} ^ {N} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {X} _ {k - 1} ^ {N} + \boldsymbol {W} _ {k - 1} ^ {N}\tag{3.5.3}
$$

$$
\mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} ^ {N} + \mathbf {V} _ {k} ^ {N} (k \leqslant N)\tag{3.5.4}
$$

式中，量测噪声 $V_{k}^{N}$ 的统计特性取为

$$
E [ \mathbf {V} _ {k} ^ {N} ] = \mathbf {0}
$$

$$
E \left[ \boldsymbol {V} _ {k} ^ {N} \left(\boldsymbol {V} _ {k} ^ {N}\right) ^ {\mathrm{T}} \right] = \boldsymbol {R} _ {k} ^ {N} = \boldsymbol {R} _ {k} s ^ {N - k} (s > 1)\tag{3.5.5}
$$

初始状态 $X_0^N$ 的统计特性取为

$$
\left. \begin{array}{l} E [ \boldsymbol {X} _ {0} ^ {N} ] = \boldsymbol {m} _ {X _ {0}} \\ \operatorname{Var} [ \boldsymbol {X} _ {0} ^ {N} ] = \boldsymbol {P} _ {0} ^ {N} = \boldsymbol {P} _ {0} s ^ {N} \end{array} \right\}\tag{3.5.6}
$$

为使滤波方程简单起见,参考文献 $[74]$ 建议将 $W_{k-1}^{N}$ 的统计特性取为

$$
\left. \begin{array}{l} E [ \boldsymbol {W} _ {k - 1} ^ {N} ] = \mathbf {0} \\ E [ \boldsymbol {W} _ {k - 1} ^ {N} (\boldsymbol {W} _ {k - 1} ^ {N}) ^ {\mathrm{T}} ] = \boldsymbol {Q} _ {k - 1} ^ {N} = \boldsymbol {Q} _ {k - 1} s ^ {N - k} \end{array} \right\}\tag{3.5.7}
$$

并且 $X_0^N, V_k^N, W_k^N$ 互不相关。于是 $X_k^N (k \leqslant N)$ 的滤波方程为

$$
\hat {\boldsymbol {X}} _ {k} ^ {N} = \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1} ^ {N} + \boldsymbol {K} _ {k} ^ {N} \left[ \boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1} ^ {N} \right]\tag{3.5.8}
$$

$$
\boldsymbol {K} _ {k} ^ {N} = \boldsymbol {P} _ {k / k - 1} ^ {N} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \left[ \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} ^ {N} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k} ^ {N} \right] ^ {- 1}\tag{3.5.9}
$$

$$
\boldsymbol {P} _ {k / k - 1} ^ {N} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {P} _ {k - 1} ^ {N} \boldsymbol {\Phi} _ {k, k - 1} ^ {T} + \boldsymbol {Q} _ {k - 1} ^ {N}\tag{3.5.10}
$$

$$
\boldsymbol {P} _ {k} ^ {N} = (\boldsymbol {I} - \boldsymbol {K} _ {k} ^ {N} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1} ^ {N}\tag{3.5.11}
$$

滤波初值为

$$
\begin{array}{l} \hat {\mathbf {X}} _ {0} ^ {N} = E [ \mathbf {X} _ {0} ^ {N} ] = \mathbf {m} _ {X _ {0}} \\ \mathbf {P} _ {0} ^ {N} = \mathbf {P} _ {0} s ^ {N} \end{array}
$$

式(3.5.10)两边同乘 $s^{-(N-k)}$ ,

记

则上式成

$$
\begin{array}{r l} s ^ {- (N - k)} \boldsymbol {P} _ {k / k - 1} ^ {N} & = \boldsymbol {\Phi} _ {k, k - 1} s ^ {- [ N - (k - 1) ] + 1} \boldsymbol {P} _ {k - 1} ^ {N} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + s ^ {- (N - k)} \boldsymbol {Q} _ {k - 1} ^ {N} \\ & \quad \boldsymbol {P} _ {k / k - 1} ^ {*} = s ^ {- (N - k)} \boldsymbol {P} _ {k / k - 1} ^ {N} \\ & \quad \boldsymbol {P} _ {k} ^ {*} = \boldsymbol {P} _ {k} ^ {N} s ^ {- (N - k)} \\ & \quad \hat {\boldsymbol {X}} _ {k - 1} ^ {*} = \hat {\boldsymbol {X}} _ {k - 1} ^ {N} \\ & \quad \hat {\boldsymbol {X}} _ {k} ^ {*} = \hat {\boldsymbol {X}} _ {k} ^ {N} \\ & \boldsymbol {P} _ {k / k - 1} ^ {*} = \boldsymbol {\Phi} _ {k, k - 1} (\boldsymbol {P} _ {k - 1} ^ {*} \cdot s) \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \boldsymbol {Q} _ {k - 1} \end{array}\tag{3.5.12}
$$

式 $(3.5.9)$ 式可写成

$$
\begin{array}{r l} \boldsymbol {K} _ {k} ^ {N} & = s ^ {- (N - k)} \boldsymbol {P} _ {k / k - 1} ^ {N} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \left[ \boldsymbol {H} _ {k} s ^ {- (N - k)} \boldsymbol {P} _ {k / k - 1} ^ {N} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k} ^ {N} s ^ {- (N - k)} \right] ^ {- 1} = \\ & \boldsymbol {P} _ {k / k - 1} ^ {*} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \left[ \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} ^ {*} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k} \right] ^ {- 1} \triangleq \boldsymbol {K} _ {k} ^ {*} \end{array}
$$

式(3.5.11)两边同乘 $s^{-(N-k)}$ ，

$$
s ^ {- (N - k)} \boldsymbol {P} _ {k} ^ {N} = (\boldsymbol {I} - \boldsymbol {K} _ {k} ^ {*} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1} ^ {N} s ^ {- (N - k)}\tag{3.5.13}
$$

即

$$
\boldsymbol {P} _ {k} ^ {*} = (\boldsymbol {I} - \boldsymbol {K} _ {k} ^ {*} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1} ^ {*}\tag{3.5.14}
$$

所以式 $(3.5.8)$ 可写成

$$
\hat {\boldsymbol {X}} _ {k} ^ {*} = \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1} ^ {*} + \boldsymbol {K} _ {k} ^ {*} \left[ \boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1} ^ {*} \right]\tag{3.5.15}
$$

滤波初值为

$$
\hat {\boldsymbol {X}} _ {0} ^ {*} = \boldsymbol {m} _ {X _ {0}}, \boldsymbol {P} _ {0} ^ {*} = \boldsymbol {P} _ {0} ^ {N} s ^ {- N} = \boldsymbol {P} _ {0}
$$

式(3.5.12)～(3.5.15)所列滤波方程与卡尔曼滤波基本方程相比，不同的地方仅在式(3.5.12)多了一个标量因子s。由于 $s>1, P_{k/k-1}^{*}$ 总比 $P_{k/k-1}$ 大，所以总有 $K_{k}^{*}>K_{k}$ ，这意味着采用这套滤波算法时，对新量测值的利用权重比采用基本方程时的权重大。又由于

$$
\hat {\mathbf {X}} _ {k} ^ {*} = \left(\mathbf {I} - \mathbf {K} _ {k} ^ {*} \mathbf {H} _ {k}\right) \boldsymbol {\Phi} _ {k, k - 1} \hat {\mathbf {X}} _ {k - 1} ^ {*} + \mathbf {K} _ {k} ^ {*} \mathbf {Z} _ {k} = \left(\mathbf {I} - \mathbf {K} _ {k} ^ {*} \mathbf {H} _ {k}\right) \hat {\mathbf {X}} _ {k / k - 1} ^ {*} + \mathbf {K} _ {k} ^ {*} \mathbf {Z} _ {k}
$$

$K_{k}^{*}>K_{k}$ 意味着对 $\hat{X}_{k/k-1}^{*}$ 的利用权重相对地降低，即降低了陈旧量测值对估计值的影响。

在例3-2中，若采用衰减记忆滤波方程式(3.5.12)～(3.5.15)，取 $s = \frac{1}{c} (c < 1)$ ，可计算

得[69]

$$
\lim _ {k \to \infty} K _ {k} ^ {*} = \lim _ {k \to \infty} \frac {1 - c}{1 - c ^ {k}} = 1 - c
$$

$$
\lim _ {k \rightarrow \infty} P _ {k} ^ {*} = \left(\frac {c u}{1 - c}\right) ^ {2} + \frac {1 - c}{1 + c}
$$

增益下降受到抑制，滤波发散得到抑制。

## 2. 限定记忆法滤波

抑制滤波发散的另一种途径是限定记忆法滤波。由于 $\hat{X}_{k}=E^{*}\left[X_{k}/Z_{1}Z_{2}\cdots Z_{k}\right]$ ，所以卡尔曼滤波对量测数据的记忆是无限保存的。采用限定记忆法滤波计算 $X_{k}$ 的估计时，只使用离 k 时刻最近的 N 个量测值 $Z_{k-N+1}, Z_{k-N+2}, \cdots, Z_{k}$ ，而完全截断 $k-N+1$ 时刻以前的陈旧量测对滤波值的影响。

在具体讨论之前,先简要说明这种滤波方法的构成思路。

使用卡尔曼滤波基本方程求取 $\hat{X}_{k}$ 时，需使用前 k 个量测量 $Z_{1}, Z_{2}, \cdots, Z_{k}$ ，先利用前 k-1 个量测求取 $\hat{X}_{k/k-1}$ ，再在 $\hat{X}_{k/k-1}$ 的基础上使用 $Z_{k}$ 获得 $\hat{X}_{k}$ ，从而建立起 $\hat{X}_{k}$ 与 $\hat{X}_{k/k-1}$ 间的线性关系。

类似地,可以给出限定记忆滤波方程的推导思路如下:为要求得 $X_{k}$ 的限定记忆滤波值,对 $X_{k}$ 及其过去值进行了k次量测,量测值为 $Z_{1},Z_{2},\cdots,Z_{d},\cdots,Z_{k-1},Z_{k}$ 。设 $Z_{d}$ 至 $Z_{k}$ 共有 $N+1$ 个量测值,记

$$
\begin{array}{r l} \overline {{{\mathbf {Z}}}} _ {d, k} ^ {N + 1} = & \left[ \begin{array}{c} \mathbf {Z} _ {d} \\ \mathbf {Z} _ {d + 1} \\ \vdots \\ \mathbf {Z} _ {k - 1} \\ \mathbf {Z} _ {k} \end{array} \right], \quad \overline {{{\mathbf {Z}}}} _ {d, k - 1} ^ {N} = \left[ \begin{array}{c} \mathbf {Z} _ {d} \\ \mathbf {Z} _ {d + 1} \\ \vdots \\ \mathbf {Z} _ {k - 1} \end{array} \right], \quad \overline {{{\mathbf {Z}}}} _ {d + 1, k} ^ {N} = \left[ \begin{array}{c} \mathbf {Z} _ {d + 1} \\ \vdots \\ \mathbf {Z} _ {k - 1} \\ \mathbf {Z} _ {k} \end{array} \right] \\ & \hat {\mathbf {X}} _ {k} ^ {N + 1} = E ^ {*} [ \mathbf {X} _ {k} / \overline {{{\mathbf {Z}}}} _ {d, k} ^ {N + 1} ] \\ & \hat {\mathbf {X}} _ {k / k - 1} ^ {N} = E ^ {*} [ \mathbf {X} _ {k} / \overline {{{\mathbf {Z}}}} _ {d, k - 1} ^ {N} ] \\ & \hat {\mathbf {X}} _ {k} ^ {N} = E ^ {*} [ \mathbf {X} _ {k} / \overline {{{\mathbf {Z}}}} _ {d + 1, k} ^ {N} ] \end{array}
$$

建立起 $\hat{X}_{k}^{N+1}$ 与 $\hat{X}_{k/k-1}^{N}$ 间的线性关系式和 $\hat{X}_{k}^{N+1}$ 与 $\hat{X}_{k}^{N}$ 间的线性关系式，再根据这两式确定出 $\hat{X}_{k}^{N}$ 与 $\hat{X}_{k/k-1}^{N}$ 间的线性关系式，从而获得 $X_{k}$ 的限定记忆滤波方程。

下面根据上述思路导出限定记忆滤波方程。为使推导简洁，此处假设系统噪声为零，对于非零情况，推导过程类似，只是冗长繁杂些罢了。

设系统方程和量测方程为

$$
\boldsymbol {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {X} _ {k - 1}\tag{3.5.16}
$$

$$
\mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k}\tag{3.5.17}
$$

式中， $V_{k}$ 为零均值的白噪声，方差为 $R_{k}$ 。

(1) $\hat{X}_{k}^{N+1}$ 与 $\hat{X}_{k/k-1}^{N}$ 间的线性关系。根据线性最小方差估计的线性性质，由式(3.5.16)得

$$
E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{\mathbf {Z}}} _ {d, k - 1} ^ {N} \right] = \boldsymbol {\Phi} _ {k, k - 1} E ^ {*} \left[ \mathbf {X} _ {k - 1} / \overline {{\mathbf {Z}}} _ {d, k - 1} ^ {N} \right]
$$

即

$$
\hat {\boldsymbol {X}} _ {k / k - 1} ^ {N} = \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1} ^ {N}\tag{3.5.18}
$$

由于 $\overline{\mathbf{Z}}_{d,k}^{N + 1}$ 比 $\overline{\mathbf{Z}}_{d,k - 1}^{N}$ 多了在 $k$ 时刻的量测值 $\mathbf{Z}_k$ ，所以根据卡尔曼滤波基本方程

$$
\begin{array}{r l} E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{\mathbf {Z}}} _ {d, k} ^ {N + 1} \right] & = E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{\mathbf {Z}}} _ {d, k - 1} ^ {N}, \mathbf {Z} _ {k} \right] = \\ & E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{\mathbf {Z}}} _ {d, k - 1} ^ {N} \right] + E ^ {* *} \left[ \widetilde {\mathbf {X}} _ {k} / \mathbf {Z} _ {k} \right] \end{array}
$$

式中， $E^{**}[\widetilde{\mathbf{X}}_k / \mathbf{Z}_k]$ 表示根据 $\mathbf{Z}_k$ 对 $\widetilde{\mathbf{X}}_k$ 所作的修正。即

$$
\hat {\mathbf {X}} _ {k} ^ {N + 1} = \hat {\mathbf {X}} _ {k / k - 1} ^ {N} + \mathbf {J} _ {k} \left[ \mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1} ^ {N} \right]\tag{3.5.19}
$$

式中

$$
\boldsymbol {J} _ {k} = \boldsymbol {P} _ {k / k - 1} ^ {N} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \left[ \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} ^ {N} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k} \right] ^ {- 1}\tag{3.5.20}
$$

$$
\boldsymbol {P} _ {k / k - 1} ^ {N} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {P} _ {k - 1} ^ {N} \boldsymbol {\Phi} _ {k, k - 1} ^ {T}\tag{3.5.21}
$$

$$
\boldsymbol {P} _ {k} ^ {N + 1} = \left(\boldsymbol {I} - \boldsymbol {J} _ {k} \boldsymbol {H} _ {k}\right) \boldsymbol {P} _ {k / k - 1} ^ {N} = \boldsymbol {P} _ {k / k - 1} ^ {N} - \boldsymbol {P} _ {k / k - 1} ^ {N} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \left[ \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} ^ {N} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k} \right] ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} ^ {N}
$$

根据矩阵反演公式(2.1.23)

$$
\left(\boldsymbol {A} _ {1 1} - \boldsymbol {A} _ {1 2} \boldsymbol {A} _ {2 2} ^ {- 1} \boldsymbol {A} _ {2 1}\right) ^ {- 1} = \boldsymbol {A} _ {1 1} ^ {- 1} + \boldsymbol {A} _ {1 1} ^ {- 1} \boldsymbol {A} _ {1 2} \left(\boldsymbol {A} _ {2 2} - \boldsymbol {A} _ {2 1} \boldsymbol {A} _ {1 1} ^ {- 1} \boldsymbol {A} _ {1 2}\right) ^ {- 1} \boldsymbol {A} _ {2 1} \boldsymbol {A} _ {1 1} ^ {- 1}
$$

令 $A_{11}^{-1}=P_{k/k-1}^{N},A_{12}=-H_{k}^{T},A_{22}=R_{k},A_{21}=H_{k}$ ，则有

$$
\boldsymbol {P} _ {k} ^ {N + 1} = \left[ \left(\boldsymbol {P} _ {k / k - 1} ^ {N}\right) ^ {- 1} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \right] ^ {- 1}\tag{3.5.22}
$$

（2） $\hat{\mathbf{X}}_k^N$ 与 $\hat{\mathbf{X}}_k^{N + 1}$ 间的线性关系。由于 $\overline{\mathbf{Z}}_{d,k}^{N + 1}$ 比 $\overline{\mathbf{Z}}_{d + 1,k}^N$ 多了一个在 $d$ 时刻的量测值 $\mathbf{Z}_d$ ，所以根据卡尔曼滤波基本方程

$$
E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{\mathbf {Z}}} _ {d, k} ^ {N + 1} \right] = E ^ {*} \left[ \mathbf {X} _ {k} / \overline {{\mathbf {Z}}} _ {d + 1, k} ^ {N} \right] + E ^ {* *} \left[ \widetilde {\mathbf {X}} _ {k} / \mathbf {Z} _ {d} \right]\tag{3.5.23}
$$

式中， $E^{**}[\tilde{\mathbf{X}}_k / \mathbf{Z}_d]$ 表示根据 $\mathbf{Z}_d$ 对 $\tilde{\mathbf{X}}_k$ 所作的修正。而

$$
\mathbf {Z} _ {d} = \mathbf {H} _ {d} \mathbf {X} _ {d} + \mathbf {V} _ {d} = \mathbf {H} _ {d} \boldsymbol {\Phi} _ {d, k} \mathbf {X} _ {k} + \mathbf {V} _ {d}\tag{3.5.24}
$$

只要把 $H_{d}\Phi_{d,k}$ 看做量测阵，则 $\mathbf{Z}_d$ 就可看做是 $\mathbf{X}_k$ 的量测值，所以式(3.5.23）可写成

$$
\hat {\boldsymbol {X}} _ {k} ^ {N + 1} = \hat {\boldsymbol {X}} _ {k} ^ {N} + \bar {\boldsymbol {J}} _ {k} \left[ \boldsymbol {Z} _ {d} - \boldsymbol {H} _ {d} \boldsymbol {\Phi} _ {d, k} \hat {\boldsymbol {X}} _ {k} ^ {N} \right]\tag{3.5.25}
$$

式中

$$
\begin{array}{r l} \bar {\boldsymbol {J}} _ {k} & = \boldsymbol {P} _ {k} ^ {N} \boldsymbol {\Phi} _ {d, k} ^ {T} \boldsymbol {H} _ {d} ^ {T} [ \boldsymbol {H} _ {d} \boldsymbol {\Phi} _ {d, k} \boldsymbol {P} _ {k} ^ {N} \boldsymbol {\Phi} _ {d, k} ^ {T} \boldsymbol {H} _ {d} ^ {T} + \boldsymbol {R} _ {d} ] ^ {- 1} \\ & \quad \boldsymbol {P} _ {k} ^ {N} = E [ \widehat {\boldsymbol {X}} _ {k} ^ {N} (\widehat {\boldsymbol {X}} _ {k} ^ {N}) ^ {T} ] \\ \boldsymbol {P} _ {k} ^ {N + 1} & = (\boldsymbol {I} - \bar {\boldsymbol {J}} _ {k} \boldsymbol {H} _ {d} \boldsymbol {\Phi} _ {d, k}) \boldsymbol {P} _ {k} ^ {N} = \\ & [ (\boldsymbol {P} _ {k} ^ {N}) ^ {- 1} + \boldsymbol {\Phi} _ {d, k} ^ {T} \boldsymbol {H} _ {d} ^ {T} \boldsymbol {R} _ {d} ^ {- 1} \boldsymbol {H} _ {d} \boldsymbol {\Phi} _ {d, k} ] ^ {- 1} \end{array}\tag{3.5.26}
$$

(3.5.27)

式(3.5.25)～式(3.5.27)推导中使用了矩阵反演公式(2.1.23)。

(3) $X_{k}$ 的限定记忆滤波方程。式(3.5.25)与式(3.5.19)等式两边对应相减得

$$
\hat {\boldsymbol {X}} _ {k} ^ {N} - \hat {\boldsymbol {X}} _ {k / k - 1} ^ {N} = \boldsymbol {J} _ {k} \left[ \boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \hat {\boldsymbol {X}} _ {k / k - 1} ^ {N} \right] - \bar {\boldsymbol {J}} _ {k} \left[ \boldsymbol {Z} _ {d} - \boldsymbol {H} _ {d} \boldsymbol {\Phi} _ {d, k} \hat {\boldsymbol {X}} _ {k} ^ {N} \right]\tag{3.5.28}
$$

注意到

$$
\begin{array}{r l} \bar {\boldsymbol {J}} _ {k} [ \boldsymbol {Z} _ {d} - \boldsymbol {H} _ {d} \boldsymbol {\Phi} _ {d, k} \hat {\boldsymbol {X}} _ {k} ^ {N} ] = & \bar {\boldsymbol {J}} _ {k} [ \boldsymbol {Z} _ {d} - \boldsymbol {H} _ {d} \boldsymbol {\Phi} _ {d, k} \hat {\boldsymbol {X}} _ {k / k - 1} ^ {N} ] - \\ & \bar {\boldsymbol {J}} _ {k} \boldsymbol {H} _ {d} \boldsymbol {\Phi} _ {d, k} [ \hat {\boldsymbol {X}} _ {k} ^ {N} - \hat {\boldsymbol {X}} _ {k / k - 1} ^ {N} ] \end{array}\tag{3.5.29}
$$

式 $(3.5.28)$ 可写成

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {k} ^ {N} - \hat {\mathbf {X}} _ {k / k - 1} ^ {N} & = \mathbf {J} _ {k} [ \mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1} ^ {N} ] - \bar {\mathbf {J}} _ {k} [ \mathbf {Z} _ {d} - \mathbf {H} _ {d} \boldsymbol {\Phi} _ {d, k} \hat {\mathbf {X}} _ {k / k - 1} ^ {N} ] + \\ & \quad \bar {\mathbf {J}} _ {k} \mathbf {H} _ {d} \boldsymbol {\Phi} _ {d, k} [ \hat {\mathbf {X}} _ {k} ^ {N} - \hat {\mathbf {X}} _ {k / k - 1} ^ {N} ] \end{array}\tag{3.5.30}
$$

将式(3.5.30)右边最后一项移至左边，并以 $(I-\bar{J}_{k}H_{d}\Phi_{d,k})^{-1}$ 左乘两边，得

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {k} ^ {N} - \hat {\mathbf {X}} _ {k / k - 1} ^ {N} & = (\mathbf {I} - \bar {\mathbf {J}} _ {k} \mathbf {H} _ {d} \boldsymbol {\Phi} _ {d, k}) ^ {- 1} \mathbf {J} _ {k} [ \mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1} ^ {N} ] - \\ & (\mathbf {I} - \bar {\mathbf {J}} _ {k} \mathbf {H} _ {d} \boldsymbol {\Phi} _ {d, k}) ^ {- 1} \bar {\mathbf {J}} _ {k} [ \mathbf {Z} _ {d} - \mathbf {H} _ {d} \boldsymbol {\Phi} _ {d, k} \hat {\mathbf {X}} _ {k / k - 1} ^ {N} ] \end{array}\tag{3.5.31}
$$

记

$$
\boldsymbol {K} _ {k} = (\boldsymbol {I} - \bar {\boldsymbol {J}} _ {k} \boldsymbol {H} _ {d} \boldsymbol {\Phi} _ {d, k}) ^ {- 1} \boldsymbol {J} _ {k}\tag{3.5.32}
$$

$$
\bar {\boldsymbol {K}} _ {k} = (\boldsymbol {I} - \bar {\boldsymbol {J}} _ {k} \boldsymbol {H} _ {d} \boldsymbol {\Phi} _ {d, k}) ^ {- 1} \bar {\boldsymbol {J}} _ {k}\tag{3.5.33}
$$

并将式(3.5.18)代入式(3.5.31)得 $X_{k}$ 的限定记忆滤波方程

$$
\hat {\boldsymbol {X}} _ {k} ^ {N} = \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1} ^ {N} + \boldsymbol {K} _ {k} \left[ \boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1} ^ {N} \right] -
$$

$$
\bar {\mathbf {K}} _ {k} \left[ \mathbf {Z} _ {d} - \mathbf {H} _ {d} \boldsymbol {\Phi} _ {d, k} \boldsymbol {\Phi} _ {k, k - 1} \hat {\mathbf {X}} _ {k - 1} ^ {N} \right] \quad k > N\tag{3.5.34}
$$

再将式 $(3.5.20)$ 代入式 $(3.5.32)$ ，式 $(3.5.26)$ 代入式 $(3.5.33)$ ，经整理后得

$$
\boldsymbol {K} _ {k} = \boldsymbol {P} _ {k} ^ {\mathrm{N}} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1}\tag{3.5.35}
$$

$$
\bar {\boldsymbol {K}} _ {k} = \boldsymbol {P} _ {k} ^ {N} \boldsymbol {\Phi} _ {d, k} ^ {T} \boldsymbol {H} _ {d} ^ {T} \boldsymbol {R} _ {d} ^ {- 1}\tag{3.5.36}
$$

比较式 $(3.5.22)$ 和式 $(3.5.27)$ ，得

$$
\left(\boldsymbol {P} _ {k} ^ {\mathrm{N}}\right) ^ {- 1} = \boldsymbol {\Phi} _ {k - 1, k} ^ {\mathrm{T}} \left(\boldsymbol {P} _ {k - 1} ^ {\mathrm{N}}\right) ^ {- 1} \boldsymbol {\Phi} _ {k - 1, k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} - \boldsymbol {\Phi} _ {d, k} ^ {\mathrm{T}} \boldsymbol {H} _ {d} ^ {\mathrm{T}} \boldsymbol {R} _ {d} ^ {- 1} \boldsymbol {H} _ {d} \boldsymbol {\Phi} _ {d, k}\tag{3.5.37}
$$

式 $(3.5.34)$ ～式 $(3.5.37)$ 即为限定记忆滤波的递推方程。

（4）滤波初值的选取。当 $k \leqslant N$ 时，量测次数小于记忆长度 $N$ ，此时不能进行限定记忆滤波，而只能采用卡尔曼滤波基本方程，由初值 $\hat{\pmb{X}}_0 = m_{X_0}$ 和 $P_0 = \mathrm{Var}[X_0]$ 计算到 $\hat{\pmb{X}}_N$ 和 $P_N$ 。但由 $k = N + 1$ 起，如果直接取 $\hat{\pmb{X}}_N^N = \hat{\pmb{X}}_N, P_N^N = P_N$ 作为初值进行限定记忆滤波，则随后的滤波值 $\hat{\pmb{X}}_k^N$ 一直受 $\hat{\pmb{X}}_0$ 和 $P_0$ 的影响，这是不符合限定记忆要求的。为了避免原始初值 $\hat{\pmb{X}}_0$ 和 $P_0$ 对 $\hat{\pmb{X}}_k^N (k > N)$ 的影响，可按下式选取初值

$$
\boldsymbol {P} _ {N} ^ {N} = \left[ \boldsymbol {P} _ {N} ^ {- 1} - \boldsymbol {\Phi} _ {0, N} ^ {\mathrm{T}} \boldsymbol {P} _ {0} ^ {- 1} \boldsymbol {\Phi} _ {0, N} \right] ^ {- 1}
$$

$$
\hat {\boldsymbol {X}} _ {N} ^ {N} = \boldsymbol {P} _ {N} ^ {N} \left[ \boldsymbol {P} _ {N} ^ {- 1} \hat {\boldsymbol {X}} _ {N} - \boldsymbol {\Phi} _ {0, N} ^ {T} \boldsymbol {P} _ {0} ^ {- 1} \hat {\boldsymbol {X}} _ {0} \right]\tag{3.5.38}
$$

(3.5.39)

## 3.6 平方根滤波

## 3.6.1 平方根滤波所要解决的问题

在3.5.1小节中已指出，产生计算发散的主要原因是滤波计算中的舍入误差积累使 $P_{k}$ 和 $\pmb{P}_{k / k - 1}$ 逐渐失去非负定性。尤其当量测向量中某一个或某几个分量很准确，而数值计算的有效位数却相对少时这种现象较易发生。 $\pmb{P}_k$ 和 $P_{k / k - 1}$ 非负定性的逐渐丧失将使 $\pmb{K}_{k}$ 计算失真，从残差中提取对一步预测值的补偿信息越来越错误，最后造成发散。平方根滤波就是在滤波过程中不是计算 $\pmb{P}_k$ 和 $P_{k / k - 1}$ ，而是计算 $\pmb{P}_k$ 和 $P_{k / k - 1}$ 的平方根。

由矩阵理论知道,任意非零矩阵 $L(n \times m)$ 与其转置矩阵 $L^{T}$ 的乘积 $LL^{T} = A(n \times n)$ 一定是非负定的。L 称为 A 的平方根。如果在滤波计算中只对 $P_{k}$ 和 $P_{k/k-1}$ 的平方根 $\Delta_{k}$ 和 $\Delta_{k/k-1}$ 作计算,而 $\Delta_{k}$ 和 $\Delta_{k/k-1}$ 又都是非零矩阵,则 $P_{k} = \Delta_{k}\Delta_{k}^{T}$ 和 $P_{k/k-1} = \Delta_{k/k-1}\Delta_{k/k-1}^{T}$ 一定是非负定的。

为了方便，在对非负定阵 A 作平方根分解时，一般都使平方根矩阵的阶数与 A 的阶数相同，这对于阶数较高的矩阵，计算量是很大的。而从矩阵理论知，任何正定矩阵 B 都可作三角形平方根分解，即 $B = DD^{T}$ ，其中 D 为三角形矩阵，由 B 惟一确定，且三角形分解也较容易实现。所以平方根滤波中，平方根阵 $\Delta_{k}$ 和 $\Delta_{k/k-1}$ 一般常取三角形阵。

平方根滤波不但能保证 $P_{k}$ 和 $P_{k / k - 1}$ 的非负定性，而且在数值计算中，计算 $\Delta$ 的字长只须计算 $P$ 的字长的一半，就能达到相同的精度，这也是平方根滤波的另一个优点。平方根滤波的缺点是计算量比标准的滤波计算量大出 $0.5\sim 1.5$ 倍。

## 3.6.2 非负定阵的三角形分解

设 n 阶方阵 P 为任一非负定的对称阵, 根据矩阵理论, P 可作下三角分解

$$
\boldsymbol {P} = \boldsymbol {\Delta} \boldsymbol {\Delta} ^ {\mathrm{T}}\tag{3.6.1}
$$

也可作上三角分解

$$
\boldsymbol {P} = \boldsymbol {U} \boldsymbol {U} ^ {\mathrm{T}}\tag{3.6.2}
$$

式中 $\Delta$ 为下三角矩阵，即 $\Delta$ 的非零元仅位于主对角线及主对角线的左下方，主对角线右上方的元全为零；U 为上三角矩阵，即 U 的非零元仅位于主对角线及主对角线的右上方，主对角线左下方的元全为零。

下面分别介绍这两种分解方法。

1. 非负定阵的下三角分解——乔莱斯基分解法一

设非负定阵 $\pmb{P}$ 为

$$
\boldsymbol {P} = \left[ \begin{array}{c c c c} P _ {1 1} & P _ {1 2} & \dots & P _ {1 n} \\ P _ {2 1} & P _ {2 2} & \dots & P _ {2 n} \\ \vdots & \vdots & & \vdots \\ P _ {n 1} & P _ {n 2} & \dots & P _ {n n} \end{array} \right],
$$

且 $\pmb{P}$ 为对称阵。

P 的下三角分解平方根为

$$
\boldsymbol {\Delta} = \left[ \begin{array}{c c c c} \delta_ {1 1} & 0 & \dots & 0 \\ \delta_ {2 1} & \delta_ {2 2} & \dots & 0 \\ \vdots & \vdots & \ddots & \vdots \\ \delta_ {n 1} & \delta_ {n 2} & \dots & \delta_ {n n} \end{array} \right]
$$

则根据式(3.6.1)定义

$$
\left[ \begin{array}{c c c c} P _ {1 1} & P _ {1 2} & \dots & P _ {1 n} \\ P _ {2 1} & P _ {2 2} & \dots & P _ {2 n} \\ \vdots & \vdots & & \vdots \\ P _ {n 1} & P _ {n 2} & \dots & P _ {n n} \end{array} \right] = \left[ \begin{array}{c c c c} \delta_ {1 1} & 0 & \dots & 0 \\ \delta_ {2 1} & \delta_ {2 2} & \dots & 0 \\ \vdots & \vdots & & \vdots \\ \delta_ {n 1} & \delta_ {n 2} & \dots & \delta_ {n n} \end{array} \right] \left[ \begin{array}{c c c c} \delta_ {1 1} & \delta_ {2 1} & \dots & \delta_ {n 1} \\ 0 & \delta_ {2 2} & \dots & \delta_ {n 2} \\ \vdots & \vdots & & \vdots \\ 0 & 0 & \dots & \delta_ {n n} \end{array} \right]
$$

由于 P 是对称阵, 所以只须考虑下三角位置上的元 $P_{ij} (j \leqslant i)$

$$
\begin{array}{r l} & P _ {1 1} = \delta_ {1 1} ^ {2} \\ & P _ {2 1} = \delta_ {2 1} \delta_ {1 1} \\ & P _ {2 2} = \delta_ {2 1} ^ {2} + \delta_ {2 2} ^ {2} \\ & P _ {3 1} = \delta_ {3 1} \delta_ {1 1} \\ & P _ {3 2} = \delta_ {3 1} \delta_ {2 1} + \delta_ {3 2} \delta_ {2 2} \\ & P _ {3 3} = \delta_ {3 1} ^ {2} + \delta_ {3 2} ^ {2} + \delta_ {3 3} ^ {2} \\ & \quad \vdots \\ & P _ {n 1} = \delta_ {n 1} \delta_ {1 1} \\ & P _ {n 2} = \delta_ {n 1} \delta_ {2 1} + \delta_ {n 2} \delta_ {2 2} \\ & P _ {n 3} = \delta_ {n 1} \delta_ {3 1} + \delta_ {n 2} \delta_ {3 2} + \delta_ {n 3} \delta_ {3 3} \\ & \quad \vdots \end{array}
$$

$$
P _ {n n} = \delta_ {n 1} ^ {2} + \delta_ {n 2} ^ {2} + \dots + \delta_ {n n} ^ {2}
$$

从上述诸式可归纳出如下通项公式

$$
\left\{ \begin{array}{l} P _ {i j} = \sum_ {k = 1} ^ {j} \delta_ {i k} \delta_ {j k}, \quad j <   i \\ P _ {i i} = \sum_ {k = 1} ^ {i} \delta_ {i k} ^ {2}, \quad i = 1, 2, \dots , n \end{array} \right.
$$

即

$$
\left\{ \begin{array}{l} P _ {i j} = \sum_ {k = 1} ^ {j - 1} \delta_ {i k} \delta_ {j k} + \delta_ {i j} \delta_ {j j}, \quad j <   i \\ P _ {i i} = \sum_ {k = 1} ^ {i - 1} \delta_ {i k} ^ {2} + \delta_ {i i} ^ {2}, \quad i = 1, 2, \dots , n \end{array} \right.
$$

所以

$$
\begin{array}{l} \delta_ {i i} = \left(P _ {i i} - \sum_ {k = 1} ^ {i - 1} \delta_ {i k} ^ {2}\right) ^ {\frac {1}{2}}, \quad i = 1, 2, \dots , n \\ \delta_ {i j} = \left\{ \begin{array}{l l} P _ {i j} - \sum_ {k = 1} ^ {j - 1} \delta_ {i k} \delta_ {j k}, & j = 1, 2, \dots , i - 1 \\ 0 & , \quad j > i \end{array} \right. \end{array}\tag{3.6.3}
$$

例3-3 求 $\pmb{P} = \begin{bmatrix} 1 & 2 & 3\\ 2 & 8 & 2\\ 3 & 2 & 14 \end{bmatrix}$ 的下三角分解平方根矩阵。

解

$$
\begin{array}{r l} & \delta_ {1 1} = \sqrt {P _ {1 1}} = 1 \\ & \delta_ {2 1} = \frac {P _ {2 1} - 0}{\delta_ {1 1}} = \frac {2}{1} = 2 \\ & \delta_ {2 2} = \sqrt {P _ {2 2} - \delta_ {2 1} ^ {2}} = \sqrt {8 - 4} = 2 \\ & \delta_ {3 1} = \frac {P _ {3 1} - 0}{\delta_ {1 1}} = \frac {3}{1} = 3 \\ & \delta_ {3 2} = \frac {P _ {3 2} - \delta_ {3 1} \delta_ {2 1}}{\delta_ {2 2}} = \frac {2 - 3 \times 2}{2} = - 2 \\ & \delta_ {3 3} = \sqrt {P _ {3 3} - (\delta_ {3 1} ^ {2} + \delta_ {3 2} ^ {2})} = \sqrt {1 4 - (9 + 4)} = 1 \end{array}
$$

所以

$$
\boldsymbol {\Delta} = \left[ \begin{array}{c c c} 1 & 0 & 0 \\ 2 & 2 & 0 \\ 3 & - 2 & 1 \end{array} \right]
$$

2. 非负定阵的上三角分解——乔莱斯基分解法二设非负定阵 P 为对称阵

$$
\boldsymbol {P} = \left[ \begin{array}{c c c c} P _ {1 1} & P _ {1 2} & \dots & P _ {1 n} \\ P _ {2 1} & P _ {2 2} & \dots & P _ {2 n} \\ \vdots & \vdots & & \vdots \\ P _ {n 1} & P _ {n 2} & \dots & P _ {n n} \end{array} \right]
$$

$\pmb{P}$ 的上三角分解平方根阵为

$$
\boldsymbol {U} = \left[ \begin{array}{c c c c} u _ {1 1} & u _ {1 2} & \dots & u _ {1 n} \\ 0 & u _ {2 2} & \dots & u _ {2 n} \\ \vdots & \vdots & \ddots & \vdots \\ 0 & 0 & \dots & u _ {n n} \end{array} \right]
$$

根据式 $(3.6.2)$ 定义,仿照下三角分解的推导,得

$$
\begin{array}{r l} & u _ {i i} = \left(P _ {i i} - \sum_ {k = i + 1} ^ {n} u _ {i k} ^ {2}\right) ^ {\frac {1}{2}}, \quad i = 1, 2, \dots , n \\ & u _ {i j} = \left\{ \begin{array}{l l} P _ {i j} - \sum_ {k = j + 1} ^ {n} u _ {i k} u _ {j k}, & i = j - 1, j - 2, \dots , 2, 1 \\ \hline u _ {j j}, & i > j \end{array} \right. \end{array}\tag{3.6.4}
$$

## 3.6.3 平方根滤波的 Potter 算法

设系统方程和量测方程为

$$
\begin{array}{r} \mathbf {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \mathbf {X} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \mathbf {W} _ {k - 1} \\ \mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k} \end{array}\tag{3.6.5}
$$

(3.6.6)

式中， $W_{k}$ 和 $V_{k}$ 都是零均值白噪声，方差阵分别为 $Q_{k}$ 和 $R_{k}, W_{k}$ 和 $V_{k}$ 互相独立。记

$$
\begin{array}{l} \boldsymbol {P} _ {k} = \boldsymbol {\Delta} _ {k} \boldsymbol {\Delta} _ {k} ^ {\mathrm{T}} \\ \boldsymbol {P} _ {k / k - 1} = \boldsymbol {\Delta} _ {k / k - 1} \boldsymbol {\Delta} _ {k / k - 1} ^ {\mathrm{T}} \end{array}
$$

式中， $\Delta_{k}$ 和 $\Delta_{k/k-1}$ 都是下三角矩阵。

## 1. 平方根滤波的量测更新

(1) 量测为标量。量测为标量时,量测方程为

式中

$$
\begin{array}{r} { Z _ {k} = H _ {k} X _ {k} + V _ {k}} \\ { E [ V _ {k} ] = 0, \quad E [ V _ {k} ^ {2} ] = R _ {k}} \end{array}
$$

根据式 $(2.2.4)$ 有

$$
\boldsymbol {K} _ {k} = \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + R _ {k}) ^ {- 1}\tag{3.6.7}
$$

$$
\hat {\mathbf {X}} _ {k} = \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} (Z _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1})\tag{3.6.8}
$$

$$
\mathbf {P} _ {k} = (\mathbf {I} - \mathbf {K} _ {k} \mathbf {H} _ {k}) \mathbf {P} _ {k / k - 1}\tag{3.6.9}
$$

将式 $(3.6.7)$ 代入式 $(3.6.9)$ 得

$$
\boldsymbol {P} _ {k} = \boldsymbol {P} _ {k / k - 1} - \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \left(\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + R _ {k}\right) ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1}
$$

$P_{k}$ 和 $P_{k/k-1}$ 分别用平方根形式表示之，则上式成

$$
\boldsymbol {\Delta} _ {k} \boldsymbol {\Delta} _ {k} ^ {\mathrm{T}} = \boldsymbol {\Delta} _ {k / k - 1} \boldsymbol {\Delta} _ {k / k - 1} ^ {\mathrm{T}} - \boldsymbol {\Delta} _ {k / k - 1} \boldsymbol {\Delta} _ {k / k - 1} ^ {\mathrm{T}} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \boldsymbol {\Delta} _ {k / k - 1} \boldsymbol {\Delta} _ {k / k - 1} ^ {\mathrm{T}} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + R _ {k}) ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {\Delta} _ {k / k - 1} \boldsymbol {\Delta} _ {k / k - 1} ^ {\mathrm{T}} =
$$

$$
\boldsymbol {\Delta} _ {k / k - 1} \left[ \boldsymbol {I} - \boldsymbol {\Delta} _ {k / k - 1} ^ {\mathrm{T}} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \left(\boldsymbol {H} _ {k} \boldsymbol {\Delta} _ {k / k - 1} \boldsymbol {\Delta} _ {k / k - 1} ^ {\mathrm{T}} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + R _ {k}\right) ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {\Delta} _ {k / k - 1} \right] \boldsymbol {\Delta} _ {k / k - 1} ^ {\mathrm{T}}
$$

令

$$
\boldsymbol {a} _ {k} = \boldsymbol {\Delta} _ {k / k - 1} ^ {\mathrm{T}} \boldsymbol {H} _ {k} ^ {\mathrm{T}}\tag{3.6.10}
$$

$$
b _ {k} = (H _ {k} \Delta_ {k / k - 1} \Delta_ {k / k - 1} ^ {\mathrm{T}} H _ {k} ^ {\mathrm{T}} + R _ {k}) ^ {- 1}\tag{3.6.11}
$$

则

$$
\boldsymbol {\Delta} _ {k} \boldsymbol {\Delta} _ {k} ^ {\mathrm{T}} = \boldsymbol {\Delta} _ {k / k - 1} \left[ \boldsymbol {I} - b _ {k} \boldsymbol {a} _ {k} \boldsymbol {a} _ {k} ^ {\mathrm{T}} \right] \boldsymbol {\Delta} _ {k / k - 1} ^ {\mathrm{T}}\tag{3.6.12}
$$

令

$$
\boldsymbol {I} - b _ {k} \boldsymbol {a} _ {k} \boldsymbol {a} _ {k} ^ {\mathrm{T}} = [ \boldsymbol {I} - b _ {k} \gamma_ {k} \boldsymbol {a} _ {k} \boldsymbol {a} _ {k} ^ {\mathrm{T}} ] [ \boldsymbol {I} - b _ {k} \gamma_ {k} \boldsymbol {a} _ {k} \boldsymbol {a} _ {k} ^ {\mathrm{T}} ] ^ {\mathrm{T}}
$$

式中 $\gamma$ 为待定的标量。展开上式的右侧，得

$$
\boldsymbol {I} - b _ {k} \boldsymbol {a} _ {k} \boldsymbol {a} _ {k} ^ {\mathrm{T}} = \boldsymbol {I} - 2 b _ {k} \gamma_ {k} \boldsymbol {a} _ {k} \boldsymbol {a} _ {k} ^ {\mathrm{T}} + b _ {k} ^ {2} \gamma_ {k} ^ {2} \boldsymbol {a} _ {k} \boldsymbol {a} _ {k} ^ {\mathrm{T}} \boldsymbol {a} _ {k} \boldsymbol {a} _ {k} ^ {\mathrm{T}}
$$

即

$$
\boldsymbol {I} - b _ {k} \boldsymbol {a} _ {k} \boldsymbol {a} _ {k} ^ {\mathrm{T}} = \boldsymbol {I} - b _ {k} (2 \gamma_ {k} - b _ {k} \gamma_ {k} ^ {2} \boldsymbol {a} _ {k} ^ {\mathrm{T}} \boldsymbol {a} _ {k}) \boldsymbol {a} _ {k} \boldsymbol {a} _ {k} ^ {\mathrm{T}}
$$

比较上述等式各项,得

$$
2 \gamma_ {k} - b _ {k} \gamma_ {k} ^ {2} \boldsymbol {a} _ {k} ^ {\mathrm{T}} \boldsymbol {a} _ {k} = 1\tag{3.6.13}
$$

又由式 $(3.6.10)$ 和式 $(3.6.11)$

$$
\boldsymbol {a} _ {k} ^ {\mathrm{T}} \boldsymbol {a} _ {k} = \boldsymbol {H} _ {k} \boldsymbol {\Delta} _ {k / k - 1} \boldsymbol {\Delta} _ {k / k - 1} ^ {\mathrm{T}} \boldsymbol {H} _ {k} ^ {\mathrm{T}} = \frac {1}{b _ {k}} - R _ {k}
$$

所以式 $(3.6.13)$ 可写成

$$
2 \gamma_ {k} - b _ {k} \gamma_ {k} ^ {2} \left(\frac {1}{b _ {k}} - R _ {k}\right) = 1
$$

即

$$
(1 - b _ {k} R _ {k}) \gamma_ {k} ^ {2} - 2 \gamma_ {k} + 1 = 0
$$

解得

$$
\gamma_ {k} = \frac {1}{1 \pm \sqrt {b _ {k} R _ {k}}}\tag{3.6.14}
$$

这样式 $(3.6.12)$ 可写成

$$
\boldsymbol {\Delta} _ {k} \boldsymbol {\Delta} _ {k} ^ {\mathrm{T}} = \boldsymbol {\Delta} _ {k / k - 1} \left[ \boldsymbol {I} - b _ {k} \gamma_ {k} \boldsymbol {a} _ {k} \boldsymbol {a} _ {k} ^ {\mathrm{T}} \right] \left[ \boldsymbol {I} - b _ {k} \gamma_ {k} \boldsymbol {a} _ {k} \boldsymbol {a} _ {k} ^ {\mathrm{T}} \right] ^ {\mathrm{T}} \boldsymbol {\Delta} _ {k / k - 1} ^ {\mathrm{T}}
$$

因此有

$$
\boldsymbol {\Delta} _ {k} = \boldsymbol {\Delta} _ {k / k - 1} \left[ \boldsymbol {I} - b _ {k} \gamma_ {k} \boldsymbol {a} _ {k} \boldsymbol {a} _ {k} ^ {\mathrm{T}} \right]\tag{3.6.15}
$$

将式 $(3.6.10)$ 、式 $(3.6.11)$ 代入式 $(3.6.7)$ 得

$$
\boldsymbol {K} _ {k} = \boldsymbol {\Delta} _ {k / k - 1} \boldsymbol {\Delta} _ {k / k - 1} ^ {\mathrm{T}} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \boldsymbol {\Delta} _ {k / k - 1} \boldsymbol {\Delta} _ {k / k - 1} ^ {\mathrm{T}} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + R _ {k}) ^ {- 1} = b _ {k} \boldsymbol {\Delta} _ {k / k - 1} \boldsymbol {a} _ {k}\tag{3.6.16}
$$

所以式 $(3.6.15)$ 又可写成

$$
\boldsymbol {\Delta} _ {k} = \boldsymbol {\Delta} _ {k / k - 1} - \gamma_ {k} \boldsymbol {K} _ {k} \boldsymbol {a} _ {k} ^ {\mathrm{T}}\tag{3.6.17}
$$

综上所述,平方根滤波的量测更新方程为

$$
\begin{array}{r l} & {\pmb {a} _ {k} = \pmb {\Delta} _ {k / k - 1} ^ {\mathrm{T}} \pmb {H} _ {k} ^ {\mathrm{T}}} \\ & {b _ {k} = (\pmb {H} _ {k} \pmb {\Delta} _ {k / k - 1} \pmb {\Delta} _ {k / k - 1} ^ {\mathrm{T}} \pmb {H} _ {k} ^ {\mathrm{T}} + R _ {k}) ^ {- 1}} \end{array}
$$

或

$$
\begin{array}{r l} & b _ {k} = (\boldsymbol {a} _ {k} ^ {\top} \boldsymbol {a} _ {k} + R _ {k}) ^ {- 1} \\ & \gamma_ {k} = (1 + \sqrt {b _ {k} R _ {k}}) ^ {- 1} \\ & \boldsymbol {K} _ {k} = b _ {k} \boldsymbol {\Delta} _ {k / k - 1} \boldsymbol {a} _ {k} \end{array}\tag{3.6.18}
$$

$$
\begin{array}{r} \hat {\mathbf {X}} _ {k} = \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} (Z _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1}) \\ \boldsymbol {\Delta} _ {k} = \boldsymbol {\Delta} _ {k / k - 1} - \gamma_ {k} \mathbf {K} _ {k} \boldsymbol {a} _ {k} ^ {\mathrm{T}} \end{array}
$$

(2) 量测为 m 维向量。对于 m 维独立量测的情况, 量测噪声方差阵为对角阵

$$
\boldsymbol {R} _ {k} = \operatorname{diag} \left[ R _ {k} ^ {1} \quad R _ {k} ^ {2} \quad \dots \quad R _ {k} ^ {m} \right]
$$

则平方根滤波的量测更新可采用序贯处理来实现。

设根据 k-1 时刻的序贯处理结果已获得 $\hat{X}_{k/k-1}$ 和 $\Delta_{k/k-1}$ ，则 k 时刻的量测更新序贯处理可按下述步骤执行。

取

$$
\left. \begin{array}{l} \hat {\boldsymbol {X}} _ {k} ^ {0} = \hat {\boldsymbol {X}} _ {k / k - 1} \\ \boldsymbol {\Delta} _ {k} ^ {0} = \boldsymbol {\Delta} _ {k / k - 1} \end{array} \right\}\tag{3.6.19}
$$

对于 $j=1,2,\cdots,m$ ，递推计算下述方程

$$
\left. \begin{array}{l} \boldsymbol {a} _ {k} ^ {j} = (\boldsymbol {H} _ {k} ^ {j} \boldsymbol {\Delta} _ {k} ^ {j - 1}) ^ {\mathrm{T}} \\ b _ {k} ^ {j} = (\boldsymbol {a} _ {k} ^ {j \mathrm{T}} \boldsymbol {a} _ {k} ^ {j} + R _ {k} ^ {j}) ^ {- 1} \\ \gamma_ {k} ^ {j} = (1 + \sqrt {b _ {k} ^ {j} R _ {k} ^ {j}}) ^ {- 1} \\ \boldsymbol {K} _ {k} ^ {j} = b _ {k} ^ {j} \boldsymbol {\Delta} _ {k} ^ {j - 1} \boldsymbol {a} _ {k} ^ {j} \\ \hat {\boldsymbol {X}} _ {k} ^ {j} = \hat {\boldsymbol {X}} _ {k} ^ {j - 1} + \boldsymbol {K} _ {k} ^ {j} (Z _ {k} ^ {j} - \boldsymbol {H} _ {k} ^ {j} \hat {\boldsymbol {X}} _ {k} ^ {j - 1}) \\ \boldsymbol {\Delta} _ {k} ^ {j} = \boldsymbol {\Delta} _ {k} ^ {j - 1} - \gamma_ {k} ^ {j} \boldsymbol {K} _ {k} ^ {j} \boldsymbol {a} _ {k} ^ {j \mathrm{T}} \end{array} \right\}\tag{3.6.20}
$$

当 j=m 时, 即获得 k 时刻的量测更新结果

$$
\left. \begin{array}{l} \hat {\boldsymbol {X}} _ {k} = \hat {\boldsymbol {X}} _ {k} ^ {m} \\ \boldsymbol {\Delta} _ {k} = \boldsymbol {\Delta} _ {k} ^ {m} \end{array} \right\}\tag{3.6.21}
$$

2. 平方根滤波的时间更新

卡尔曼滤波的时间更新方程为

$$
\begin{array}{r} \hat {X} _ {k / k - 1} = \Phi_ {k, k - 1} \hat {X} _ {k - 1} \\ P _ {k / k - 1} = \Phi_ {k, k - 1} P _ {k - 1} \Phi_ {k, k - 1} ^ {\mathrm{T}} + \Gamma_ {k - 1} Q _ {k - 1} \Gamma_ {k - 1} ^ {\mathrm{T}} \end{array}
$$

求取 $P_{k/k-1}$ 的平方根 $\Delta_{k/k-1}$ 最直观的方法是直接按乔莱斯基法对 $P_{k/k-1}$ 作平方根分解，这种方法称为 RSS（矩阵根和方）法。RSS 法计算速度较快，但数值精度与标准卡尔曼滤波的时间更新算法的精度相同。由于滤波计算中对精度影响最大的是量测更新，特别是求逆，而时间更新的影响相对来说较小，所以 RSS 法的使用仍较普遍。

时间更新的另一种算法是三角化算法，在这种算法中，平方根分解是通过矩阵变换实现的。

设 $P_{k / k - 1}$ 和 $P_{k - 1}$ 的下三角分解平方根分别为 $\Delta_{k / k - 1}$ 和 $\Delta_{k - 1}$ ，则式(2.2.4b)可写为

$$
\boldsymbol {\Delta} _ {k / k - 1} \boldsymbol {\Delta} _ {k / k - 1} ^ {\mathrm{T}} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {\Delta} _ {k - 1} \boldsymbol {\Delta} _ {k - 1} ^ {\mathrm{T}} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {Q} _ {k - 1} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}}\tag{3.6.22}
$$

如果存在单位正交变换矩阵 T，使下式成立

$$
\left[ \begin{array}{l l} \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {\Delta} _ {k - 1} & \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {Q} _ {k - 1} ^ {\frac {1}{2}} \end{array} \right] \boldsymbol {T} = \left[ \begin{array}{l l} \boldsymbol {D} _ {k} & \boldsymbol {0} \end{array} \right]\tag{3.6.23}
$$

式中 $Q_{k-1}^{\frac{1}{2}}$ 为 $Q_{k-1}$ 的下三角分解平方根， $D_{k}$ 为某一下三角阵，则

$$
\pmb {D} _ {k} \pmb {D} _ {k} ^ {\mathrm{T}} = [ \pmb {D} _ {k} \quad \mathbf {0} ] \left[ \begin{array}{l} \pmb {D} _ {k} ^ {\mathrm{T}} \\ \mathbf {0} \end{array} \right] =
$$

$$
\begin{array}{r l} & {\left[ \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {\Delta} _ {k - 1} \quad \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {Q} _ {k - 1} ^ {\frac {1}{2}} \right] \boldsymbol {T} \boldsymbol {T} ^ {\mathrm{T}} \left[ \begin{array}{l} \boldsymbol {\Delta} _ {k - 1} ^ {\mathrm{T}} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \\ (\boldsymbol {Q} _ {k - 1} ^ {\frac {1}{2}}) ^ {\mathrm{T}} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}} \end{array} \right] =} \\ & {\boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {\Delta} _ {k - 1} \boldsymbol {\Delta} _ {k - 1} ^ {\mathrm{T}} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {Q} _ {k - 1} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}}} \end{array}
$$

上式与式 $(3.6.22)$ 比较,知

$$
\pmb {D} _ {k} = \pmb {\Delta} _ {k / k - 1}\tag{3.6.24}
$$

因此要求取 $\Delta_{k/k-1}$ ，只要对 $\left[\Phi_{k,k-1}\Delta_{k-1}\quad\Gamma_{k-1}Q_{k-1}^{\frac{1}{2}}\right]$ 作适当的变换。下面不加证明地给出实现 T 正交变换的两种有效方法。

将 $\left[\Phi_{k,k-1}\Delta_{k-1} \quad \Gamma_{k-1}Q_{k-1}^{\frac{1}{2}}\right]$ 变换为式（3.6.22）所示形式的下三角阵，实际上就是将 $\left[\begin{array}{l}\boldsymbol{\Delta}_{k-1}^{\mathrm{T}}\boldsymbol{\Phi}_{k,k-1}^{\mathrm{T}}\\ (\boldsymbol{\Gamma}_{k-1}\boldsymbol{Q}_{k-1}^{\frac{1}{2}})^{\mathrm{T}}\end{array}\right]$ 变换为上三角阵，对于这一高矩阵的变换可通过Householder变换或改进的Gram-Schmidt(MGS)法完成。

设系统驱动噪声为 l 维向量, 记

$$
\boldsymbol {A} _ {1} = \left[ \begin{array}{l} \boldsymbol {\Delta} _ {k - 1} ^ {\mathrm{T}} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \\ (\boldsymbol {\Gamma} _ {k - 1} \boldsymbol {Q} _ {k - 1} ^ {\frac {1}{2}}) ^ {\mathrm{T}} \end{array} \right] _ {(n + l) \times n}\tag{3.6.25}
$$

并记 $A_{i}$ 为迭代过程中的第 i 次迭代结果矩阵。

(1) Householder 变换法。Householder 变换法执行如下 n 次迭代计算。

对于 $i=1,2,\cdots,n$

$$
\begin{array}{r l} a _ {i} & = \sum_ {j = i} ^ {n + l} \sqrt {A _ {i} ^ {2} (j , i)} \cdot \mathrm{sign} [ A _ {i} (i, i) ] \\ d _ {i} & = \frac {1}{a _ {i} [ a _ {i} + A _ {i} (i , i) ]} \end{array}\tag{3.6.26a}
$$

(3.6.26b)

$$
\begin{array}{r l} & u _ {i} (j) = \left\{ \begin{array}{l l} 0, & j <   i \\ a _ {i} + A _ {i} (i, i), & j = i \\ A _ {i} (j, i), & j = i + 1, i + 2, \dots , n + l \end{array} \right. \\ & y _ {i} (j) = \left\{ \begin{array}{l l} 0, & j <   i \\ 1, & j = i \\ d _ {i} \mathbf {u} _ {i} ^ {\mathrm{T}} \mathbf {A} _ {i} ^ {(j)}, & j = i + 1, i + 2, \dots , n \end{array} \right. \\ & \quad \mathbf {A} _ {i + 1} = \mathbf {A} _ {i} - \mathbf {u} _ {i} \mathbf {y} _ {i} ^ {\mathrm{T}} \end{array}\tag{3.6.26c}
$$

(3.6.26d)

(3.6.26e)

式中 $\mathbf{A}_i^{(j)}$ 为 $\mathbf{A}_i$ 的第 $j$ 列

$$
\boldsymbol {u} _ {i} = \left[ \begin{array}{c} u _ {i} (1) \\ u _ {i} (2) \\ \vdots \\ u _ {i} (n + l) \end{array} \right], \boldsymbol {y} _ {i} = \left[ \begin{array}{c} y _ {i} (1) \\ y _ {i} (2) \\ \vdots \\ y _ {i} (n) \end{array} \right]
$$

经过 n 次迭代后, 得

$$
\mathbf {A} _ {n + 1} = \left[ \begin{array}{c} \boldsymbol {\Delta} _ {k / k - 1} ^ {\mathrm{T}} \\ \mathbf {0} \end{array} \right] \begin{array}{l} n \times n \\ l \times n \end{array}\tag{3.6.27}
$$

（2）改进的 Gram-Schmidt(MGS) 法。改进的 Gram-Schmidt 法执行如下 n 步递推计算。

对于 $i=1,2,\cdots,n$

$$
a _ {i} = \sqrt {\mathbf {A} _ {i} ^ {(i) \mathrm{T}} \mathbf {A} _ {i} ^ {(i)}}\tag{3.6.28a}
$$

$$
c (i, j) = \left\{ \begin{array}{l l} 0, & j = 1, 2, \dots , i - 1 \\ a _ {i}, & j = i \\ \frac {1}{a _ {i}} \mathbf {A} _ {i} ^ {(j) \mathrm{T}} \mathbf {A} _ {i} ^ {(j)}, & j = i + 1, i + 2, \dots , n \end{array} \right.\tag{3.6.28b}
$$

$$
\mathbf {A} _ {i + 1} ^ {(j)} = \mathbf {A} _ {i} ^ {(j)} - \frac {c (i , j)}{a _ {i}} \mathbf {A} _ {i} ^ {(i)}, \quad j = i + 1, i + 2, \dots , n\tag{3.6.28c}
$$

完成上述 n 步递推后, 得 n 阶方阵 C, 则

$$
\boldsymbol {\Delta} _ {k / k - 1} = \boldsymbol {C} ^ {\mathrm{T}}\tag{3.6.29}
$$

例3-4 设定常系统的各参数阵为

$$
\pmb {\Phi} = \left[ \begin{array}{l l} 1 & 0 \\ 0 & 1 \end{array} \right], \quad \pmb {H} = \left[ \begin{array}{l l} 1 & 0 \end{array} \right], \quad Q = \mathbf {0}
$$

量测为标量, $R=\varepsilon^{2}(\varepsilon\ll1)$ 。

计算机在计算过程中遵循如下原则：

$$
1 + \varepsilon^ {2} = 1, 1 + \varepsilon \neq 1
$$

式中 r 表示计算机执行过程中的具体处理。

$$
\pmb {P} _ {k / k - 1} = \left[ \begin{array}{l l} 1 & 0 \\ 0 & 1 \end{array} \right]
$$

试分别按标准滤波方程和平方根滤波方程求出两步滤波结果，并予以比较。

解

按标准滤波方程

$$
\mathbf {K} _ {k} = \left[ \begin{array}{l l} 1 & 0 \\ 0 & 1 \end{array} \right] \left[ \begin{array}{l} 1 \\ 0 \end{array} \right] ([ 1 \quad 0 ] \left[ \begin{array}{l l} 1 & 0 \\ 0 & 1 \end{array} \right] [ 1 ] + \varepsilon^ {2}) ^ {- 1} = \left[ \begin{array}{l} 1 \\ 0 \end{array} \right] (1 + \varepsilon^ {2}) ^ {- 1} \stackrel {r} {=} \left[ \begin{array}{l} 1 \\ 0 \end{array} \right]
$$

$$
\boldsymbol {P} _ {k} = (\boldsymbol {I} - \left[ \begin{array}{l} 1 \\ 0 \end{array} \right] \left[ \begin{array}{l l} 1 & 0 \end{array} \right]) \left[ \begin{array}{l l} 1 & 0 \\ 0 & 1 \end{array} \right] = \left[ \begin{array}{l l} 0 & 0 \\ 0 & 1 \end{array} \right]
$$

$$
\boldsymbol {P} _ {k + 1 / k} = \left[ \begin{array}{l l} 0 & 0 \\ 0 & 1 \end{array} \right]
$$

$$
\mathbf {K} _ {k + 1} = \left[ \begin{array}{l l} 0 & 0 \\ 0 & 1 \end{array} \right] \left[ \begin{array}{l} 1 \\ 0 \end{array} \right] ([ 1 \quad 0 ] \left[ \begin{array}{l l} 0 & 0 \\ 0 & 1 \end{array} \right] [ \begin{array}{l} 1 \\ 0 \end{array} ] + \varepsilon^ {2}) ^ {- 1} = \left[ \begin{array}{l} 0 \\ 0 \end{array} \right]
$$

$$
\boldsymbol {P} _ {k + 1} = \left[ \begin{array}{l l} 0 & 0 \\ 0 & 1 \end{array} \right]
$$

按平方根滤波。

根据 $P_{k/k-1}$ 分解得

$$
\Delta_ {k / k - 1} = \left[ \begin{array}{l l} 1 & 0 \\ 0 & 1 \end{array} \right]
$$

$$
\pmb {a} _ {k} = \pmb {\Delta} _ {k / k - 1} ^ {\mathrm{T}} \pmb {H} _ {k} ^ {\mathrm{T}} = \left[ \begin{array}{l l} 1 & 0 \\ 0 & 1 \end{array} \right] \left[ \begin{array}{l} 1 \\ 0 \end{array} \right] = \left[ \begin{array}{l} 1 \\ 0 \end{array} \right]
$$

$$
b _ {k} = (\boldsymbol {a} _ {k} ^ {\mathrm{T}} \boldsymbol {a} _ {k} + R) ^ {- 1} = ([ 1 \quad 0 ] \left[ \begin{array}{l} 1 \\ 0 \end{array} \right] + \varepsilon^ {2}) ^ {- 1} = (1 + \varepsilon^ {2}) ^ {- 1} \stackrel {r} {=} 1
$$

$$
\gamma_ {k} = \frac {1}{1 + \sqrt {b _ {k} R}} = \frac {1}{1 + \varepsilon} \stackrel {r} {=} 1 - \varepsilon
$$

$$
\mathbf {K} _ {k} = b _ {k} \pmb {\Delta} _ {k / k - 1} \mathbf {a} _ {k} = 1 \cdot \left[ \begin{array}{l l} 1 & 0 \\ 0 & 1 \end{array} \right] \left[ \begin{array}{l} 1 \\ 0 \end{array} \right] = \left[ \begin{array}{l} 1 \\ 0 \end{array} \right]
$$

$$
\pmb {\Delta} _ {k} = \pmb {\Delta} _ {k / k - 1} - \gamma_ {k} \pmb {K} _ {k} \pmb {a} _ {k} ^ {\mathrm{T}} =
$$

$$
\left[ \begin{array}{l l} 1 & 0 \\ 0 & 1 \end{array} \right] - (1 - \varepsilon) \left[ \begin{array}{l} 1 \\ 0 \end{array} \right] [ 1 \quad 0 ] = \left[ \begin{array}{l l} \varepsilon & 0 \\ 0 & 1 \end{array} \right]
$$

$$
\boldsymbol {P} _ {k} = \boldsymbol {\Delta} _ {k} \boldsymbol {\Delta} _ {k} ^ {\mathrm{T}} = \left[ \begin{array}{l l} \varepsilon & 0 \\ 0 & 1 \end{array} \right] \left[ \begin{array}{l l} \varepsilon & 0 \\ 0 & 1 \end{array} \right] = \left[ \begin{array}{l l} \varepsilon^ {2} & 0 \\ 0 & 1 \end{array} \right]
$$

$$
\boldsymbol {\Delta} _ {k + 1 / k} = \boldsymbol {\Phi} \boldsymbol {\Delta} _ {k} = \left[ \begin{array}{l l} 1 & 0 \\ 0 & 1 \end{array} \right] \left[ \begin{array}{l l} \varepsilon & 0 \\ 0 & 1 \end{array} \right] = \left[ \begin{array}{l l} \varepsilon & 0 \\ 0 & 1 \end{array} \right]
$$

$$
\boldsymbol {a} _ {k + 1} = \boldsymbol {\Delta} _ {k + 1 / k} ^ {\mathrm{T}} \boldsymbol {H} _ {k + 1} ^ {\mathrm{T}} = \left[ \begin{array}{l l} \varepsilon & 0 \\ 0 & 1 \end{array} \right] \left[ \begin{array}{l} 1 \\ 0 \end{array} \right] = \left[ \begin{array}{l} \varepsilon \\ 0 \end{array} \right]
$$

$$
b _ {k + 1} = \left(\boldsymbol {a} _ {k + 1} ^ {\mathrm{T}} \boldsymbol {a} _ {k + 1} + R\right) ^ {- 1} = \left(\left[ \begin{array}{l l} \varepsilon & 0 \end{array} \right] \left[ \begin{array}{l} \varepsilon \\ 0 \end{array} \right] + \varepsilon^ {2}\right) ^ {- 1} = \frac {1}{2 \varepsilon^ {2}}
$$

$$
\mathbf {K} _ {k + 1} = b _ {k + 1} \boldsymbol {\Delta} _ {k + 1 / k} \mathbf {a} _ {k + 1} = \frac {1}{2 \varepsilon^ {2}} \left[ \begin{array}{l l} \varepsilon & 0 \\ 0 & 1 \end{array} \right] \left[ \begin{array}{l} \varepsilon \\ 0 \end{array} \right] = \left[ \begin{array}{l} \frac {1}{2} \\ 0 \end{array} \right]
$$

$$
\gamma_ {k + 1} = \frac {1}{1 + \sqrt {b _ {k + 1} R}} = \frac {1}{1 + \sqrt {\frac {\varepsilon^ {2}}{2 \varepsilon^ {2}}}} = 2 - \sqrt {2}
$$

$$
\boldsymbol {\Delta} _ {k + 1} = \boldsymbol {\Delta} _ {k + 1 / k} - \gamma_ {k + 1} \boldsymbol {K} _ {k + 1} \boldsymbol {a} _ {k + 1} ^ {\mathrm{T}} = \left[ \begin{array}{l l} \varepsilon & 0 \\ 0 & 1 \end{array} \right] - (2 - \sqrt {2}) \left[ \begin{array}{l} \frac {1}{2} \\ 0 \end{array} \right] [ \varepsilon 0 ] =
$$

$$
\left[ \begin{array}{c c} \varepsilon - (1 - \frac {\sqrt {2}}{2}) \varepsilon & 0 \\ 0 & 1 \end{array} \right] = \left[ \begin{array}{c c} \frac {\sqrt {2}}{2} \varepsilon & 0 \\ 0 & 1 \end{array} \right]
$$

$$
\boldsymbol {P} _ {k + 1} = \boldsymbol {\Delta} _ {k + 1} \boldsymbol {\Delta} _ {k + 1} ^ {\mathrm{T}} = \left[ \begin{array}{l l} \frac {\varepsilon^ {2}}{2} & 0 \\ 0 & 1 \end{array} \right]
$$

可以看出，平方根滤波中 $K_{k + 1} = \left[ \begin{array}{c}\frac{1}{2}\\ 0 \end{array} \right]$ ，而标准卡尔曼滤波中 $K_{k + 1} = \left[ \begin{array}{c}0\\ 0 \end{array} \right]$ ，即量测值已失去修正作用。在平方根滤波中 $P_{k + 1}$ 仍保持为正定阵，而在标准卡尔曼滤波中， $P_{k + 1}$ 已退化为不再是正定阵。

## 3.6.4 平方根滤波的 Carlson 算法

与 Potter 算法一样, Carlson 算法在处理量测为向量时的平方根滤波时, 也采用序贯处理, 即转化成量测为标量的情况。不同的是 Potter 采用下三角分解, 而 Carlson 采用上三角分解。此处以标量测量为例介绍 Carlson 平方根滤波算法。由于篇幅有限, 只给出结果而不作推导。

1. 平方根滤波的量测更新

设 $U_{k / k - 1}$ 和 $U_{k}$ 分别为 $\pmb{P}_{k / k - 1}$ 和 $\pmb{P}_k$ 的上三角分解平方根，即

$$
\boldsymbol {P} _ {k / k - 1} = \boldsymbol {U} _ {k / k - 1} \boldsymbol {U} _ {k / k - 1} ^ {\mathrm{T}}\tag{3.6.30}
$$

$$
\boldsymbol {P} _ {k} = \boldsymbol {U} _ {k} \boldsymbol {U} _ {k} ^ {\mathrm{T}}\tag{3.6.31}
$$

取 $e_{i}(i=1,2,\cdots,n)$ 为 n 维向量

$$
d _ {0} = R _ {k}\tag{3.6.32a}
$$

$$
e _ {0} = 0\tag{3.6.32b}
$$

$$
\boldsymbol {a} = \boldsymbol {U} _ {k / k - 1} ^ {\mathrm{T}} \boldsymbol {H} _ {k} ^ {\mathrm{T}} = [ a (1) \quad a (2) \quad \dots \quad a (n) ] ^ {\mathrm{T}}\tag{3.6.32c}
$$

对于 $i=1,2,\cdots,n$ ，作 n 步递推

$$
d _ {i} = d _ {i - 1} + a (i)\tag{3.6.33a}
$$

$$
b _ {i} = \sqrt {\frac {d _ {i - 1}}{d _ {i}}}\tag{3.6.33b}
$$

$$
c _ {i} = \frac {a (i)}{\sqrt {d _ {i - 1} d _ {i}}}\tag{3.6.33c}
$$

$$
\boldsymbol {e} _ {i} = \boldsymbol {e} _ {i - 1} + \boldsymbol {U} _ {k / k - 1} ^ {(i)} a (i)\tag{3.6.33d}
$$

$$
\boldsymbol {U} _ {k} ^ {(i)} = b _ {i} \boldsymbol {U} _ {k / k - 1} ^ {(i)} - c _ {i} \boldsymbol {e} _ {i - 1}\tag{3.6.33e}
$$

式中 $U_{k}^{(i)}$ 和 $U_{k / k - 1}^{(i)}$ 分别表示 $U_{k}$ 和 $U_{k / k - 1}$ 的第 $i$ 列。

经过 n 步递推后, 得上三角阵

$$
\boldsymbol {U} _ {k} = \left[ \begin{array}{l l l l} \boldsymbol {U} _ {k} ^ {(1)} & \boldsymbol {U} _ {k} ^ {(2)} & \dots & \boldsymbol {U} _ {k} ^ {(n)} \end{array} \right]\tag{3.6.34}
$$

状态估计为

$$
\hat {\mathbf {X}} _ {k} = \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {e} _ {n} \left[ (Z _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1}) / d _ {n} \right]\tag{3.6.35}
$$

2. 平方根滤波的时间更新

Carlson 算法的时间更新与 Potter 算法的类似, 不同的是 Carlson 算法获得的时间更新平方根为上三角阵。具体求取方法如下:

记

$$
\mathbf {A} = \left[ \begin{array}{l} \mathbf {U} _ {k - 1} ^ {\mathrm{T}} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \\ (\boldsymbol {\Gamma} _ {k - 1} \boldsymbol {Q} _ {k - 1} ^ {\frac {1}{2}}) ^ {\mathrm{T}} \end{array} \right] \begin{array}{l} n \times n \\ l \times n \end{array}\tag{3.6.36}
$$

式中 l 为系统驱动噪声的维数。并记 $A^{(i)}$ 为 A 的第 i 列。

对于 $i=n, n-1, n-2, \cdots, 2, 1$ ，作 n 步递推计算

$$
U _ {k / k - 1} (i, i) = \sqrt {\mathbf {A} ^ {(i) \mathrm{T}} \mathbf {A} ^ {(i)}}\tag{3.6.37a}
$$

$$
\mathbf {V} _ {i} = \frac {1}{U _ {k / k - 1} (i , i)} \mathbf {A} ^ {(i)}\tag{3.6.37b}
$$

$$
U _ {k / k - 1} (j; i) = \mathbf {A} ^ {(j) \mathrm{T}} \mathbf {V} _ {i} \quad j = 1, 2, \dots , i - 1\tag{3.6.37c}
$$

$$
\boldsymbol {A} ^ {(j)} \leftarrow \boldsymbol {A} ^ {(j)} - U _ {k / k - 1} (j, i) \boldsymbol {V} _ {i} \quad j = 1, 2, \dots , i - 1\tag{3.6.37d}
$$

式中，符号“←”表示 $A^{(j)}$ 原有的数值与 $U_{k/k-1}(j,i)V_{i}$ 相减后所得的结果对 $A^{(j)}$ 重新赋值。

经过 n 次递推后， $U_{k/k-1}$ 即为上三角阵。

## 3.7 $UDU^{\mathrm{T}}$ 分解滤波

## 3.7.1 $UDU^{T}$ 分解滤波所要解决的问题

由卡尔曼滤波基本方程的增益回路式(2.2.4d)、式(2.2.4c)、式(2.2.4e)，有

$$
\begin{array}{r l} & {\pmb {P} _ {k / k - 1} = \pmb {\Phi} _ {k, k - 1} \pmb {P} _ {k - 1} \pmb {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \pmb {\Gamma} _ {k - 1} \pmb {Q} _ {k - 1} \pmb {\Gamma} _ {k - 1} ^ {\mathrm{T}}} \\ & {\qquad \pmb {K} _ {k} = \pmb {P} _ {k / k - 1} \pmb {H} _ {k} ^ {\mathrm{T}} (\pmb {H} _ {k} \pmb {P} _ {k / k - 1} \pmb {H} _ {k} ^ {\mathrm{T}} + \pmb {R} _ {k}) ^ {- 1}} \\ & {\pmb {P} _ {k} = (\pmb {I} - \pmb {K} _ {k} \pmb {H} _ {k}) \pmb {P} _ {k / k - 1} (\pmb {I} - \pmb {K} _ {k} \pmb {H} _ {k}) ^ {\mathrm{T}} + \pmb {K} _ {k} \pmb {R} _ {k} \pmb {K} _ {k} ^ {\mathrm{T}}} \end{array}
$$

可以看出，如果在计算过程中使 $P_{k / k - 1}$ 失去了非负定性，则 $\pmb{K}_k$ 计算中的求逆将会产生很大的误差；如果由于 $P_{k / k - 1}$ 的负定性使 $H_{k}P_{k / k - 1}H_{k}^{\mathrm{T}} + R_{k}$ 变成奇异阵或接近奇异阵，则逆不存在或计算出的逆会有巨大误差，这将导致 $\hat{\mathbf{X}}_k$ 的巨大估计误差。

由于计算误差而使 $P_{k/k-1}$ 失去非负定性在一般情况下是不容易发生的，但是在下列情况下 $P_{k/k-1}$ 就很容易失去非负定性：

(1) $\Phi_{k,k-1}$ 的元非常大，并且（或者） $P_{k-1}$ 为病态阵；

(2) $P_{k-1}$ 轻度负定, 即 $P_{k-1}$ 具有接近零的负特征值。

下面举例加以说明。

设

$$
\boldsymbol {P} _ {k - 1} = \left[ \begin{array}{c c c} 4 9 8 3 9. 9 6 4 & 3 3 4 0 0. 0 0 0 & - 5 5 1 1 9. 9 5 2 \\ 0. 9 4 4 & 2 5 1 0 0. 0 0 0 & - 3 6 2 0 0. 0 0 0 \\ - 0. 9 8 8 & - 0. 9 2 4 & 6 1 1 5 9. 9 3 6 \end{array} \right]
$$

$$
\boldsymbol {\Phi} _ {k, k - 1} = \left[ \begin{array}{c c c} 4 7 4 0. 0 & - 1 0 0 0. 0 & 3 6 8 0. 0 \\ - 4. 0 & 1. 0 & - 3. 0 \\ 0. 8 & 0. 0 & 0. 6 \end{array} \right]
$$

计算中不计舍入误差，并设系统噪声为零，则

$$
\boldsymbol {P} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} = \operatorname{diag} [ - 1 0 0 0. 0 \quad 1 0 0. 0 \quad 1 0 0 0. 0 ]
$$

一步计算就使 $P_{k/k-1}$ 的特征值达到 -1000.0，这是由于 $P_{k-1}$ 的特征值为 $1.3 \times 10^{5}, 2.8 \times 10^{3}$ ， $-2.7 \times 10^{-5}, P_{k-1}$ 具有接近零的负特征值，并且 $\Phi_{k,k-1}$ 的元很大引起的。

## 定理3.2

如果滤波过程中 $P_{k}$ 和 $P_{k/k-1}$ 为非负定阵，则 $P_{k}$ 和 $P_{k/k-1}$ 可分解成 $UDU^{T}$ 的形式，即

$$
\begin{array}{r} {\pmb {P} _ {k} = \pmb {U} _ {k} \pmb {D} _ {k} \pmb {U} _ {k} ^ {\mathrm{T}}} \\ {\pmb {P} _ {k / k - 1} = \pmb {U} _ {k / k - 1} \pmb {D} _ {k / k - 1} \pmb {U} _ {k / k - 1} ^ {\mathrm{T}}} \end{array}
$$

式中， $D_{k}$ 和 $D_{k/k-1}$ 为 $n \times n$ 的对角阵， $U_{k}$ 和 $U_{k/k-1}$ 为 $n \times n$ 的上三角阵，主对角元全为 1。

$UDU^{T}$ 分解滤波过程中并不直接求解 $P_{k}$ 和 $P_{k/k-1}$ ，而是求解 $U_{k}, U_{k/k-1}, D_{k}$ 和 $D_{k/k-1}$ 。由于 U 阵和 D 阵的特殊结构，确保了滤波过程中 $P_{k}$ 和 $P_{k/k-1}$ 的非负定性。关于定理 3.2 的证明，在对定理 3.3 和定理 3.4 的证明过程中自然会得到说明。

为便于叙述,各量表示时间的右下标 k 和 k/k-1 分别用上标“^”和“\~”来表示,即

$$
\begin{array}{l l} \hat {\boldsymbol {P}} = \boldsymbol {P} _ {k} & \quad \tilde {\boldsymbol {P}} = \boldsymbol {P} _ {k / k - 1} \\ \hat {\boldsymbol {U}} = \boldsymbol {U} _ {k} & \quad \tilde {\boldsymbol {U}} = \boldsymbol {U} _ {k / k - 1} \\ \hat {\boldsymbol {D}} = \boldsymbol {D} _ {k} & \quad \tilde {\boldsymbol {D}} = \boldsymbol {D} _ {k / k - 1} \end{array}
$$

$P_{k}$ 和 $P_{k/k-1}$ 的 $UDU^{T}$ 分解为

$$
\begin{array}{l} \hat {\boldsymbol {P}} = \hat {\boldsymbol {U}} \hat {\boldsymbol {D}} \hat {\boldsymbol {U}} ^ {\mathrm{T}} \\ \tilde {\boldsymbol {P}} = \tilde {\boldsymbol {U}} \tilde {\boldsymbol {D}} \tilde {\boldsymbol {U}} ^ {\mathrm{T}} \end{array}\tag{3.7.1}
$$

(3.7.2)

假设 k 时刻的量测为 m 维, 即

$$
\mathbf {Z} _ {k} = \left[ \begin{array}{c c c c} Z _ {k} ^ {1} & Z _ {k} ^ {2} & \dots & Z _ {k} ^ {m} \end{array} \right] ^ {\mathrm{T}}
$$

滤波过程中采用序贯处理,第 $\mu$ 次处理中量测为标量

$$
Z _ {k} ^ {\mu} = \boldsymbol {H} _ {k} ^ {\mu} \boldsymbol {X} _ {k} + V _ {k} ^ {\mu} \quad \mu = 1, 2, \dots , m
$$

$V_{k}^{\mu}$ 为零均值白噪声，方差为 $R_{k}^{\mu}$ 。为叙述中书写简洁，在以下介绍中省略上下标 $\mu$ 和 k。

## 3.7.2 $U$ 和 $D$ 的量测更新算法

定理3.3

设 $\tilde{U}$ 和 $\tilde{D}$ 已知， $\tilde{U}$ 为主对角元全为 1 的上三角阵， $\tilde{D}$ 为对角阵，标量量测的量测噪声为零均值白噪声，方差为 R，则 $\hat{U}$ 和 $\hat{D}$ 按如下诸式确定：

$$
f = \widetilde {U} ^ {\mathrm{T}} H ^ {\mathrm{T}}
$$

$$
g _ {i} = \widetilde {D} _ {i} f _ {i} \quad i = 1, 2, \dots , n\tag{3.7.3}
$$

(3.7.4)

$$
\alpha_ {j} = R + \sum_ {i = 1} ^ {j} f _ {i} g _ {i}\tag{3.7.5}
$$

或

$$
\begin{array}{r l} \alpha_ {j} = \alpha_ {j - 1} + f _ {j} g _ {j} & j = 1, 2, \dots , n \\ & \alpha_ {0} = R \end{array}\tag{3.7.6}
$$

$$
\lambda_ {j} = - \frac {f _ {j}}{\alpha_ {j - 1}} \qquad j = 1, 2, \dots , n\tag{3.7.7}
$$

$$
\hat {D} _ {j} = \widetilde {D} _ {j} \frac {\alpha_ {j - 1}}{\alpha_ {j}} \quad j = 1, 2, \dots , n\tag{3.7.8}
$$

$$
\begin{array}{r l} \hat {U} _ {i j} = & \widetilde {U} _ {i j} + \lambda_ {j} (g _ {i} + \sum_ {k = i + 1} ^ {j - 1} \widetilde {U} _ {i k} g _ {k}) \quad j = 2, 3, \dots , n \\ & \hat {U} _ {i i} = 1 \quad i = 1, 2, \dots , n \end{array}\tag{3.7.9}
$$

证明

(3.7.10)

由基本方程式(2.2.4e')和式(2.2.4c)得

$$
\hat {\boldsymbol {P}} = \tilde {\boldsymbol {P}} - \tilde {\boldsymbol {P}} \boldsymbol {H} ^ {\mathrm{T}} (\boldsymbol {R} + \boldsymbol {H} \tilde {\boldsymbol {P}} \boldsymbol {H} ^ {\mathrm{T}}) ^ {- 1} \boldsymbol {H} \tilde {\boldsymbol {P}}
$$

即

$$
\hat {\boldsymbol {U}} \hat {\boldsymbol {D}} \hat {\boldsymbol {U}} ^ {\mathrm{T}} = \tilde {\boldsymbol {U}} \tilde {\boldsymbol {D}} \tilde {\boldsymbol {U}} ^ {\mathrm{T}} - \tilde {\boldsymbol {U}} \tilde {\boldsymbol {D}} \tilde {\boldsymbol {U}} ^ {\mathrm{T}} \boldsymbol {H} ^ {\mathrm{T}} (R + \boldsymbol {H} \tilde {\boldsymbol {U}} \tilde {\boldsymbol {D}} \tilde {\boldsymbol {U}} ^ {\mathrm{T}} \boldsymbol {H} ^ {\mathrm{T}}) ^ {- 1} \boldsymbol {H} \tilde {\boldsymbol {U}} \tilde {\boldsymbol {D}} \tilde {\boldsymbol {U}} ^ {\mathrm{T}}
$$

令

$$
\alpha = R + \mathbf {H} \tilde {\mathbf {U}} \tilde {\mathbf {D}} \tilde {\mathbf {U}} ^ {\mathrm{T}} \mathbf {H} ^ {\mathrm{T}}\tag{3.7.11}
$$

$$
\begin{array}{r} \boldsymbol {f} = \widetilde {\boldsymbol {U}} ^ {\mathrm{T}} \boldsymbol {H} ^ {\mathrm{T}} \\ \boldsymbol {g} = \widetilde {\boldsymbol {D}} \widetilde {\boldsymbol {U}} ^ {\mathrm{T}} \boldsymbol {H} ^ {\mathrm{T}} = \widetilde {\boldsymbol {D}} \boldsymbol {f} \end{array}\tag{3.7.12}
$$

则

$$
\hat {\boldsymbol {U}} \hat {\boldsymbol {D}} \hat {\boldsymbol {U}} ^ {\mathrm{T}} = \tilde {\boldsymbol {U}} (\tilde {\boldsymbol {D}} - \frac {1}{\alpha} \boldsymbol {g} \boldsymbol {g} ^ {\mathrm{T}}) \tilde {\boldsymbol {U}} ^ {\mathrm{T}}\tag{3.7.13}
$$

式中

$$
\mathbf {g} = \left[ \begin{array}{c} \widetilde {D} _ {1} f _ {1} \\ \widetilde {D} _ {2} f _ {2} \\ \vdots \\ \widetilde {D} _ {n} f _ {n} \end{array} \right]
$$

即

$$
g _ {i} = \widetilde {D} _ {i} f _ {i} \quad i = 1, 2, \dots , n
$$

$$
\alpha = R + (\widetilde {U} ^ {\mathrm{T}} H ^ {\mathrm{T}}) ^ {\mathrm{T}} D \widetilde {U} ^ {\mathrm{T}} H ^ {\mathrm{T}} = R + \sum_ {i = 1} ^ {n} f _ {i} g _ {i}
$$

为推导方便，记

$$
\alpha_ {j} = R + \sum_ {i = 1} ^ {j} f _ {i} g _ {i}
$$

则 $\alpha_{j}$ 具有下述递推关系

$$
\alpha_ {j} = R + \sum_ {i = 1} ^ {j - 1} f _ {i} g _ {i} + f _ {j} g _ {j} = \alpha_ {j - 1} + f _ {j} g _ {j}
$$

式中

$$
\alpha_ {0} = R\tag{3.7.14}
$$

考察式(3.7.13)，由于 $\tilde{D} -\frac{1}{\alpha}\mathbf{gg}^{\mathrm{T}}$ 也是半正定阵，所以也可作 $UDU^{\mathrm{T}}$ 分解

$$
\bar {U} \bar {D} \bar {U} ^ {\mathrm{T}} = \tilde {D} - \frac {1}{\alpha} g g ^ {\mathrm{T}}\tag{3.7.15}
$$

式中， $\bar{D}$ 为对角阵， $\bar{U}$ 为 $n \times n$ 的单位上三角阵，即主对角元全为1的上三角阵。这样式(3.7.13)可写成

$$
\hat {\boldsymbol {U}} \hat {\boldsymbol {D}} \hat {\boldsymbol {U}} ^ {\mathrm{T}} = \tilde {\boldsymbol {U}} \bar {\boldsymbol {U}} \bar {\boldsymbol {D}} \bar {\boldsymbol {U}} ^ {\mathrm{T}} \tilde {\boldsymbol {U}} ^ {\mathrm{T}}
$$

取

$$
\hat {\boldsymbol {D}} = \bar {\boldsymbol {D}}\tag{3.7.16}
$$

则

$$
\hat {\boldsymbol {U}} = \tilde {\boldsymbol {U}} \bar {\boldsymbol {U}}\tag{3.7.17}
$$

记

$$
\overline {{{\boldsymbol {U}}}} = \left[ \begin{array}{l l l l} \overline {{{\boldsymbol {U}}}} ^ {(1)} & \overline {{{\boldsymbol {U}}}} ^ {(2)} & \dots & \overline {{{\boldsymbol {U}}}} ^ {(n)} \end{array} \right]
$$

式中 $\bar{U}^{(i)}$ 为 $\bar{U}$ 的第 $i$ 个列，即

$$
\overline {{{U}}} ^ {(i)} = \left[ \begin{array}{c} \overline {{{U}}} _ {1} ^ {(i)} \\ \overline {{{U}}} _ {2} ^ {(i)} \\ \vdots \\ \overline {{{U}}} _ {i} ^ {(i)} \\ 0 \\ \vdots \\ 0 \end{array} \right]
$$

则

$$
\overline {{{\boldsymbol {U}}}} \overline {{{\boldsymbol {D}}}} \overline {{{\boldsymbol {U}}}} ^ {\mathrm{T}} = \left[ \begin{array}{l l l l} \overline {{{\boldsymbol {U}}}} ^ {(1)} & \overline {{{\boldsymbol {U}}}} ^ {(2)} & \dots & \overline {{{\boldsymbol {U}}}} ^ {(n)} \end{array} \right] \left[ \begin{array}{c c c c} \overline {{{D}}} _ {1} & & & \mathbf {0} \\ & \overline {{{D}}} _ {2} & & \\ & & \ddots & \\ \mathbf {0} & & & \overline {{{D}}} _ {n} \end{array} \right] \left[ \begin{array}{c} \overline {{{\boldsymbol {U}}}} ^ {(1) \mathrm{T}} \\ \overline {{{\boldsymbol {U}}}} ^ {(2) \mathrm{T}} \\ \vdots \\ \overline {{{\boldsymbol {U}}}} ^ {(n) \mathrm{T}} \end{array} \right] = \sum_ {i = 1} ^ {n} \overline {{{D}}} _ {i} \overline {{{\boldsymbol {U}}}} ^ {(i)} \overline {{{\boldsymbol {U}}}} ^ {(i) \mathrm{T}}\tag{3.7.18}
$$

又因为

$$
\widetilde {\boldsymbol {D}} - \frac {1}{\alpha} \boldsymbol {g} \boldsymbol {g} ^ {\mathrm{T}} = \sum_ {i = 1} ^ {n} \widetilde {D} _ {i} \boldsymbol {e} _ {i} \boldsymbol {e} _ {i} ^ {\mathrm{T}} - \frac {1}{\alpha} \boldsymbol {g} \boldsymbol {g} ^ {\mathrm{T}}\tag{3.7.19}
$$

式中 $e_{i}$ 为 n 维空间的第 i 个基向量，即

$$
\pmb {e} _ {i} = \left[ \begin{array}{c} {{0}} \\ {{\vdots}} \\ {{0}} \\ {{1}} \\ {{0}} \\ {{\vdots}} \\ {{0}} \end{array} \right] \text {   ——   第   } i \text {   行   }
$$

将式 $(3.7.18)$ 、式 $(3.7.19)$ 代入式 $(3.7.15)$ ，得

$$
\sum_ {i = 1} ^ {n} \overline {{{D}}} _ {i} \overline {{{\boldsymbol {U}}}} ^ {(i)} \overline {{{\boldsymbol {U}}}} ^ {(i) \mathrm{T}} = \sum_ {i = 1} ^ {n} \widetilde {D} _ {i} \boldsymbol {e} _ {i} \boldsymbol {e} _ {i} ^ {\mathrm{T}} - \frac {1}{\alpha} \boldsymbol {g} \boldsymbol {g} ^ {\mathrm{T}}
$$

根据式 $(3.7.6)$ ，并记

$$
\boldsymbol {V} ^ {(n)} = \boldsymbol {g}\tag{3.7.20}
$$

$$
C _ {n} = \frac {1}{\alpha} = \frac {1}{\alpha_ {n}}\tag{3.7.21}
$$

则

$$
\begin{array}{r l} & {\sum_ {i = 1} ^ {n - 1} \overline {{{D}}} _ {i} \overline {{{U}}} ^ {(i)} \overline {{{U}}} ^ {(i) \mathrm{T}} + \overline {{{D}}} _ {n} \overline {{{U}}} ^ {(n)} \overline {{{U}}} ^ {(n) \mathrm{T}} =} \\ & {\qquad \sum_ {i = 1} ^ {n - 1} \widetilde {D} _ {i} \pmb {e} _ {i} \pmb {e} _ {i} ^ {\mathrm{T}} + [ \widetilde {D} _ {n} \pmb {e} _ {n} \pmb {e} _ {n} ^ {\mathrm{T}} - C _ {n} \pmb {V} ^ {(n)} \pmb {V} ^ {(n) \mathrm{T}} ]} \end{array}\tag{3.7.22}
$$

记式(3.7.22)左边的 $n \times n$ 矩阵为 $M_1$ , 右边的 $n \times n$ 矩阵为 $M_2$ , 则由 $\overline{U}^{(i)}$ 及 $e_i$ 的定义知, 等式左边的第一项只影响 $M_1$ 的前 $n - 1$ 列, 并不影响 $M_1$ 的第 $n$ 列, 而第二项影响所有列; 等式右边的第一项只影响 $M_2$ 的前 $n - 1$ 列, 并不影响 $M_2$ 的第 $n$ 列, 而方括号内的项影响到所有列。这说明 $M_1$ 和 $M_2$ 的最后一列分别仅由 $\overline{D}_n\overline{U}^{(n)}\overline{U}^{(n)\mathrm{T}}$ 和 $\widetilde{D}_n e_n e_n^{\mathrm{T}} - C_n V^{(n)} V^{(n)\mathrm{T}}$ 确定。要使

$M_{1}=M_{2}$ ，即式(3.7.22)成立，必须有

$$
\left[ \overline {{{D}}} _ {n} \overline {{{U}}} ^ {(n)} \overline {{{U}}} ^ {(n) \mathrm{T}} \right] | _ {\text {第} n \text {列}} = \left[ \widetilde {D} _ {n} e _ {n} e _ {n} ^ {\mathrm{T}} - C _ {n} V ^ {(n)} V ^ {(n) \mathrm{T}} \right] | _ {\text {第} n \text {列}}\tag{3.7.23}
$$

考察式(3.7.23)中的 $1\sim n-1$ 个元，设所讨论的元为第i个 $(i=1,2,\cdots,n-1)$ ，式(3.7.23)左边为

$$
\begin{array}{r l} {[ \overline {{{D}}} _ {n} \overline {{{U}}} ^ {(n)} \overline {{{U}}} ^ {(n) \mathrm{T}} | _ {\text {第} n \text {列}} = \{\overline {{{D}}} _ {n} \left[ \begin{array}{c} {\overline {{{U}}} _ {1} ^ {(n)}} \\ {\overline {{{U}}} _ {2} ^ {(n)}} \\ {\vdots} \\ {\overline {{{U}}} _ {n - 1} ^ {(n)}} \\ {1} \end{array} \right] [ \overline {{{U}}} _ {1} ^ {(n)} \quad \overline {{{U}}} _ {2} ^ {(n)} \quad \dots \quad \overline {{{U}}} _ {n - 1} ^ {(n)} \quad 1 ] \} | _ {\text {第} n \text {列}} =} \\ {\overline {{{D}}} _ {n} \left[ \begin{array}{c} {\overline {{{U}}} _ {1} ^ {(n)}} \\ {\overline {{{U}}} _ {2} ^ {(n)}} \\ {\vdots} \\ {\overline {{{U}}} _ {n - 1} ^ {(n)}} \\ {1} \end{array} \right] = \overline {{{D}}} _ {n} \overline {{{U}}} ^ {(n)}} \end{array}\tag{3.7.24}
$$

式 $(3.7.23)$ 的右边为

$$
\begin{array}{r l} {[ \widetilde {D} _ {n} \pmb {e} _ {n} \pmb {e} _ {n} ^ {\mathrm{T}} - C _ {n} \pmb {V} ^ {(n)} \pmb {V} ^ {(n) \mathrm{T}} ]   | _ {\text {第} n \text {列}} =} & {\left[ \begin{array}{c} {0} \\ {\vdots} \\ {0} \\ {\widetilde {D} _ {n}} \end{array} \right] - \{C _ {n} \left[ \begin{array}{c} {V _ {1} ^ {(n)}} \\ {V _ {2} ^ {(n)}} \\ {\vdots} \\ {V _ {n} ^ {(n)}} \end{array} \right] [ V _ {1} ^ {(n)} \quad V _ {2} ^ {(n)} \quad \dots \quad V _ {n} ^ {(n)} ] \}   | _ {\text {第} n \text {列}} =} \\ & {\left[ \begin{array}{c} {- C _ {n} V _ {1} ^ {(n)} V _ {n} ^ {(n)}} \\ {- C _ {n} V _ {2} ^ {(n)} V _ {n} ^ {(n)}} \\ {\vdots} \\ {- C _ {n} V _ {n - 1} ^ {(n)} V _ {n} ^ {(n)}} \\ {\widetilde {D} _ {n} - C _ {n} [ V _ {n} ^ {(n)} ] ^ {2}} \end{array} \right]} \end{array}
$$

比较式 $(3.7.24)$ 和式 $(3.7.25)$ 式的对应元,得

$$
\begin{array}{r l} \overline {{D}} _ {n} & = \widetilde {D} _ {n} - C _ {n} [ V _ {n} ^ {(n)} ] ^ {2} \\ \overline {{D}} _ {n} \overline {{U}} _ {i} ^ {(n)} & = - C _ {n} V _ {i} ^ {(n)} V _ {n} ^ {(n)} \qquad i = 1, 2, \dots , n - 1 \end{array}\tag{3.7.26}
$$

(3.7.27)

式 $(3.7.26)$ 也可写成

$$
\overline {{{D}}} _ {n} = \widetilde {D} _ {n} - \frac {1}{\alpha_ {n}} (g _ {n}) ^ {2} = \widetilde {D} _ {n} - \frac {1}{\alpha_ {n}} (\widetilde {D} _ {n} f _ {n}) ^ {2} = \widetilde {D} _ {n} \frac {\alpha_ {n} - g _ {n} f _ {n}}{\alpha_ {n}}
$$

又由式(3.7.6)得

$$
\alpha_ {n} - f _ {n} g _ {n} = \alpha_ {n - 1}
$$

所以前式又可写成

$$
\overline {{{D}}} _ {n} = \widetilde {D} _ {n} \frac {\alpha_ {n - 1}}{\alpha_ {n}}\tag{3.7.28}
$$

将式 $(3.7.28)$ 代入式 $(3.7.27)$

$$
\overline {{{U}}} _ {i} ^ {(n)} = - C _ {n} V _ {i} ^ {(n)} V _ {n} ^ {(n)} / \overline {{{D}}} _ {n} = - \frac {1}{\alpha_ {n}} g _ {i} g _ {n} \Bigg / \left(\widetilde {D} _ {n} \frac {\alpha_ {n - 1}}{\alpha_ {n}}\right) =
$$

$$
- \frac {1}{\alpha_ {n - 1}} \frac {g _ {n}}{\widetilde {D} _ {n}} g _ {i} = - \frac {f _ {n}}{\alpha_ {n - 1}} g _ {i} \quad i = 1, 2, \dots , n - 1\tag{3.7.29a}
$$

$$
\overline {{{U}}} _ {n} ^ {(n)} = 1\tag{3.7.29b}
$$

为了求解 $\bar{D}$ 及 $\bar{U}$ 的其余元，将式(3.7.22)改写成

$$
\begin{array}{r l} & \sum_ {i = 1} ^ {n - 2} \overline {{D}} _ {i} \overline {{U}} ^ {(i)} \overline {{U}} ^ {(i) \mathrm{T}} + \overline {{D}} _ {n - 1} \overline {{U}} ^ {(n - 1)} \overline {{U}} ^ {(n - 1) \mathrm{T}} + \overline {{D}} _ {n} \overline {{U}} ^ {(n)} \overline {{U}} ^ {(n) \mathrm{T}} = \\ & \sum_ {i = 1} ^ {n - 2} \widetilde {D} _ {i} e _ {i} e _ {i} ^ {\mathrm{T}} + \widetilde {D} _ {n - 1} e _ {n - 1} e _ {n - 1} ^ {\mathrm{T}} + \widetilde {D} _ {n} e _ {n} e _ {n} ^ {\mathrm{T}} - C _ {n} V ^ {(n - 1)} V ^ {(n - 1) \mathrm{T}} - \\ & C _ {n} \left[ \begin{array}{c} 0 \\ \vdots \\ 0 \\ g _ {n} \end{array} \right] [ 0 \dots 0 g _ {n} ] \end{array}\tag{3.7.30}
$$

显然影响式(3.7.30)第n-1列的项为

左边：

右边：

$$
\begin{array}{r l} & \overline {{D}} _ {n - 1} \overline {{U}} ^ {(n - 1)} \overline {{U}} ^ {(n - 1) \mathrm{T}} + \overline {{D}} _ {n} \overline {{U}} ^ {(n)} \overline {{U}} ^ {(n) \mathrm{T}} \\ & \widetilde {D} _ {n - 1} e _ {n - 1} e _ {n - 1} ^ {\mathrm{T}} - C _ {n} V ^ {(n - 1)} V ^ {(n - 1) \mathrm{T}} \end{array}
$$

要使式 $(3.7.30)$ 成立，该式左、右边的第n-1列必须相等。而

$$
[ \overline {{{D}}} _ {n - 1} \overline {{{U}}} ^ {(n - 1)} \overline {{{U}}} ^ {(n - 1) \mathrm{T}} + \overline {{{D}}} _ {n} \overline {{{U}}} ^ {(n)} \overline {{{U}}} ^ {(n) \mathrm{T}} | _ {\mathrm{第} n - 1 \mathrm{列}} =
$$

$$
\{\overline {{{D}}} _ {n - 1} \left[ \begin{array}{c} {{\overline {{{U}}} _ {1} ^ {(n - 1)}}} \\ {{\overline {{{U}}} _ {2} ^ {(n - 1)}}} \\ {{\vdots}} \\ {{\overline {{{U}}} _ {n - 2} ^ {(n - 1)}}} \\ {{1}} \\ {{0}} \end{array} \right] [ \overline {{{U}}} _ {1} ^ {(n - 1)} \quad \overline {{{U}}} _ {2} ^ {(n - 1)} \quad \dots \quad \overline {{{U}}} _ {n - 2} ^ {(n - 1)} \quad 1 \quad 0 ] +
$$

$$
\overline {{{D}}} _ {n} \left[ \begin{array}{c c c c c} {\overline {{{U}}} _ {1} ^ {(n)}} & & & & \\ {\overline {{{U}}} _ {2} ^ {(n)}} & & & & \\ {\vdots} & & & & \\ {\overline {{{U}}} _ {n - 1} ^ {(n)}} & & & & \\ {1} & & & & \end{array} \right] [ \overline {{{U}}} _ {1} ^ {(n)} \quad \overline {{{U}}} _ {2} ^ {(n)} \quad \dots \quad \overline {{{U}}} _ {n - 1} ^ {(n)} \quad 1 ] \} | _ {\text {第} n - 1 \text {列}} =
$$

$$
\begin{array}{r l} & {\overline {{{D}}} _ {n - 1} \left[ \begin{array}{c} {\overline {{{U}}} _ {1} ^ {(n - 1)}} \\ {\overline {{{U}}} _ {2} ^ {(n - 1)}} \\ {\vdots} \\ {\overline {{{U}}} _ {n - 2} ^ {(n - 1)}} \\ {1} \\ {0} \end{array} \right] + \overline {{{D}}} _ {n} \overline {{{U}}} _ {n - 1} ^ {(n)} \left[ \begin{array}{c} {\overline {{{U}}} _ {1} ^ {(n)}} \\ {\overline {{{U}}} _ {2} ^ {(n)}} \\ {\vdots} \\ {\overline {{{U}}} _ {n - 1} ^ {(n)}} \\ {1} \end{array} \right]} \\ & {\{\widetilde {D} _ {n - 1}   \pmb {e} _ {n - 1}   \pmb {e} _ {n - 1} ^ {\mathrm{T}} - C _ {n} \pmb {V} ^ {(n - 1)} \pmb {V} ^ {(n - 1) \mathrm{T}} \}   | _ {\text {第} n - 1 \text {列}} =} \end{array}\tag{3.7.31}
$$

$$
\{\left[ \begin{array}{c c c c} {0} & {\dots} & {0} & {0} \\ {\vdots} & & {\vdots} & {\vdots} \\ {0} & {\dots} & {\widetilde {D} _ {n - 1}} & {0} \\ {0} & {\dots} & {0} & {0} \end{array} \right] - C _ {n} \left[ \begin{array}{c} {g _ {1}} \\ {g _ {2}} \\ {\vdots} \\ {g _ {n - 1}} \\ {0} \end{array} \right] \left[ \begin{array}{c c c c c} {g _ {1}} & {g _ {2}} & {\dots} & {g _ {n - 1}} & {0} \end{array} \right] \} | _ {\text {第} n - 1 \text {列}} =
$$

$$
\left[ \begin{array}{c} 0 \\ \vdots \\ \widetilde {D} _ {n - 1} \\ 0 \end{array} \right] = C _ {n} g _ {n - 1} \left[ \begin{array}{c} g _ {1} \\ g _ {2} \\ \vdots \\ g _ {n - 1} \\ 0 \end{array} \right]\tag{3.7.32}
$$

所以式(3.7.30)式的第 $n - 1$ 列相等即为

$$
\left[ \begin{array}{c} 0 \\ \vdots \\ 0 \\ \widetilde {D} _ {n - 1} \\ 0 \end{array} \right] - C _ {n} g _ {n - 1} \left[ \begin{array}{c} g _ {1} \\ g _ {2} \\ \vdots \\ g _ {n - 1} \\ 0 \end{array} \right] = \overline {{D}} _ {n - 1} \left[ \begin{array}{c} \overline {{U}} _ {1} ^ {(n - 1)} \\ \overline {{U}} _ {2} ^ {(n - 1)} \\ \vdots \\ \overline {{U}} _ {n - 2} ^ {(n - 1)} \\ 1 \\ 0 \end{array} \right] + \overline {{D}} _ {n} \overline {{U}} _ {n - 1} ^ {(n)} \left[ \begin{array}{c} \overline {{U}} _ {1} ^ {(n)} \\ \overline {{U}} _ {2} ^ {(n)} \\ \vdots \\ \overline {{U}} _ {n - 1} ^ {(n)} \\ 1 \end{array} \right]\tag{3.7.33}
$$

考察式(3.7.33)中的第 $n - 1$ 个分量

$$
\widetilde {D} _ {n - 1} - C _ {n} g _ {n - 1} ^ {2} = \overline {{D}} _ {n - 1} + \overline {{D}} _ {n} \overline {{U}} _ {n - 1} ^ {(n)} \overline {{U}} _ {n - 1} ^ {(n)}
$$

由式 $(3.7.28)$ 、式 $(3.7.29a)$ 和式 $(3.7.21)$ ，则可写成

$$
\widetilde {D} _ {n - 1} - \frac {1}{\alpha_ {n}} g _ {n - 1} ^ {2} = \overline {{{{D}}}} _ {n - 1} + \widetilde {D} _ {n} \frac {\alpha_ {n - 1}}{\alpha_ {n}} \frac {f _ {n} ^ {2}}{\alpha_ {n - 1} ^ {2}} g _ {n - 1} ^ {2}\tag{3.7.34}
$$

又由式 $(3.7.4)$ 和式 $(3.7.6)$ ，

$$
\begin{array}{c} {{\widetilde {D} _ {n} f _ {n} = g _ {n}}} \\ {{g _ {n} f _ {n} = \alpha_ {n} - \alpha_ {n - 1}}} \end{array}
$$

所以式 $(3.7.34)$ 成

$$
\widetilde {D} _ {n - 1} - \frac {1}{\alpha_ {n}} g _ {n - 1} ^ {2} = \overline {{{{D}}}} _ {n - 1} + \frac {\alpha_ {n} - \alpha_ {n - 1}}{\alpha_ {n} \alpha_ {n - 1}} g _ {n - 1} ^ {2}
$$

即

$$
\begin{array}{r l} \overline {{{D}}} _ {n - 1} & = \widetilde {D} _ {n - 1} - \frac {1}{\alpha_ {n - 1}} g _ {n - 1} ^ {2} = \widetilde {D} _ {n - 1} - \frac {1}{\alpha_ {n - 1}} (D _ {n - 1} f _ {n - 1}) ^ {2} = \\ & \widetilde {D} _ {n - 1} \frac {\alpha_ {n - 1} - g _ {n - 1} f _ {n - 1}}{\alpha_ {n - 1}} = \widetilde {D} _ {n - 1} \frac {\alpha_ {n - 1} - (\alpha_ {n - 1} - \alpha_ {n - 2})}{\alpha_ {n - 1}} = \\ & \widetilde {D} _ {n - 1} \frac {\alpha_ {n - 2}}{\alpha_ {n - 1}} \end{array}\tag{3.7.35}
$$

考察式(3.7.33)中的第1至第n-2个分量,考虑到式(3.7.28)、式(3.7.29a)和式(3.7.35),对于 $i=1,2,\cdots,n-2$ ,有

$$
- C _ {n} g _ {n - 1} g _ {i} = \overline {{D}} _ {n - 1} \overline {{U}} _ {i} ^ {(n - 1)} + \overline {{D}} _ {n} \overline {{U}} _ {n - 1} ^ {(n)} \overline {{U}} _ {i} ^ {(n)}
$$

所以

$$
\begin{array}{r l} \overline {{{U}}} _ {i} ^ {(n - 1)} & = - \frac {1}{\overline {{{D}}} _ {n - 1}} [ C _ {n} g _ {n - 1} g _ {i} + \overline {{{D}}} _ {n} \overline {{{U}}} _ {n - 1} ^ {(n)} \overline {{{U}}} _ {i} ^ {(n)} ] = \\ & - \frac {\alpha_ {n - 1}}{\widetilde {D} _ {n - 1} \alpha_ {n - 2}} \left[ \frac {1}{\alpha_ {n}} g _ {n - 1} g _ {i} + \widetilde {D} _ {n} \frac {\alpha_ {n - 1}}{\alpha_ {n}} \frac {f _ {n} ^ {2}}{\alpha_ {n - 1} ^ {2}} g _ {n - 1} g _ {i} \right] = \\ & - \frac {g _ {n - 1}}{\widetilde {D} _ {n - 1}} \frac {\alpha_ {n - 1}}{\alpha_ {n - 2} \alpha_ {n}} g _ {i} \left(1 + \frac {g _ {n} f _ {n}}{\alpha_ {n - 1}}\right) = \\ & - f _ {n - 1} \frac {\alpha_ {n - 1}}{\alpha_ {n - 2} \alpha_ {n}} g _ {i} \left(1 + \frac {\alpha_ {n} - \alpha_ {n - 1}}{\alpha_ {n - 1}}\right) = \\ & - \frac {f _ {n - 1}}{\alpha_ {n - 2}} g _ {i} \end{array}\tag{3.7.36}
$$

仿照上述分析方法,分别讨论式(3.7.22)的第n-2列、第n-3列、…、第2列,可得 $\bar{D}$ 和 $\bar{U}$ 各元的通项

(3.7.8)

$$
\begin{array}{r l} \overline {{D}} _ {j} = \widetilde {D} _ {j} \frac {\alpha_ {j - 1}}{\alpha_ {j}} & j = 1, 2, \dots , n \\ \overline {{U}} _ {i} ^ {(j)} = \left\{ \begin{array}{l l} - \frac {f _ {j}}{\alpha_ {j - 1}} g _ {i}, & j = 2, 3, \dots , n; \\ 1, & i = 1, 2, \dots , j - 1 \\ & j = 1, 2, \dots , n; \\ & i = j \end{array} \right. \end{array}\tag{3.7.37}
$$

将式(3.7.28)、式(3.7.29)、式(3.7.35)及式(3.7.36)与式(3.7.8)及式(3.7.37)相比较,可看出:式(3.7.28)和式(3.7.29)为j=n的情况;式(3.7.35)和式(3.7.36)为j=n-1的情况。

获得 $\bar{U}$ 后, 即可根据式(3.7.13) 求取 $\hat{U}$ 。

由式(3.7.37)

$$
\overline {{{{\boldsymbol {U}}}}} ^ {(j)} = \left[ \begin{array}{c} {{\lambda_ {j} g _ {1}}} \\ {{\lambda_ {j} g _ {2}}} \\ {{\vdots}} \\ {{\lambda_ {j} g _ {j - 1}}} \\ {{1}} \\ {{0}} \\ {{\vdots}} \\ {{0}} \end{array} \right] = \lambda_ {j} {\boldsymbol {V}} ^ {(j - 1)} + {\boldsymbol {e}} _ {j}\tag{3.7.38}
$$

式中

$$
\begin{array}{r l} \lambda_ {j} & = - \frac {f _ {j}}{\alpha_ {j - 1}} \quad j = 2, 3, \dots , n \\ \mathbf {V} ^ {(j - 1)} & = [ g _ {1} \quad g _ {2} \quad \dots \quad g _ {j - 1} \quad 0 \quad \dots \quad 0 ] ^ {\mathrm{T}} \end{array}\tag{3.7.39}
$$

所以

$$
\begin{array}{r l} \bar {U} & = [ \bar {U} ^ {(1)} \quad \bar {U} ^ {(2)} \quad \dots \quad \bar {U} ^ {(n)} ] = \\ & [ e _ {1} \quad e _ {2} \quad \dots \quad e _ {n} ] + [ 0 \quad \lambda_ {2} V ^ {(1)} \quad \lambda_ {3} V ^ {(2)} \quad \dots \quad \lambda_ {n} V ^ {(n - 1)} ] = \\ & I + [ 0 \quad \lambda_ {2} V ^ {(1)} \quad \lambda_ {3} V ^ {(2)} \quad \dots \quad \lambda_ {n} V ^ {(n - 1)} ] \end{array}
$$

$$
\hat {\boldsymbol {U}} = \widetilde {\boldsymbol {U}} \overline {{\boldsymbol {U}}} = \widetilde {\boldsymbol {U}} + \left[ \begin{array}{l l l l l} \mathbf {0} & \lambda_ {2} \widetilde {\boldsymbol {U}} \boldsymbol {V} ^ {(1)} & \lambda_ {3} \widetilde {\boldsymbol {U}} \boldsymbol {V} ^ {(2)} & \dots & \lambda_ {n} \widetilde {\boldsymbol {U}} \boldsymbol {V} ^ {(n - 1)} \end{array} \right]\tag{3.7.40}
$$

$$
\hat {\boldsymbol {U}} ^ {(j)} = \tilde {\boldsymbol {U}} ^ {(j)} + \lambda_ {j} \tilde {\boldsymbol {U}} \boldsymbol {V} ^ {(j - 1)}\tag{3.7.41}
$$

式中 $\tilde{U} V^{(j)}$ 可写成递推的形式

$$
\widetilde {\boldsymbol {U}} \boldsymbol {V} ^ {(j)} = \widetilde {\boldsymbol {U}} [ \boldsymbol {V} ^ {(j - 1)} + g _ {j} \boldsymbol {e} _ {j} ] = \widetilde {\boldsymbol {U}} \boldsymbol {V} ^ {(j - 1)} + \widetilde {\boldsymbol {U}} ^ {(j)} g _ {j} \quad j = 2, 3, \dots , n\tag{3.7.42}
$$

$\hat{U}$ 也可按元逐个求取。考察式(3.7.40)中的方括号所示矩阵的第 $j$ 列向量 $\hat{U}^{*(j)}$ ， $\tilde{\boldsymbol{U}}^{*(j)} = \lambda_j\tilde{\boldsymbol{U}}\boldsymbol{V}^{(j - 1)} =$

$$
\begin{array}{r l} & {\lambda_ {j} \cdot \left[ \begin{array}{c c c c c c c c} {1} & {\widetilde {U} _ {1 2}} & {\widetilde {U} _ {1 3}} & {\dots} & {\widetilde {U} _ {1 i}} & {\widetilde {U} _ {1 (i + 1)}} & {\dots} & {\widetilde {U} _ {1 n}} \\ {0} & {1} & {\widetilde {U} _ {2 3}} & {\dots} & {\widetilde {U} _ {2 i}} & {\widetilde {U} _ {2 (i + 1)}} & {\dots} & {\widetilde {U} _ {2 n}} \\ {\vdots} & {\vdots} & {\vdots} & & {\vdots} & {\vdots} & & {\vdots} \\ {0} & {0} & {0} & {\dots} & {\widetilde {U} _ {(i - 1) i}} & {\widetilde {U} _ {(i - 1) (i + 1)}} & {\dots} & {\widetilde {U} _ {(i - 1) n}} \\ {0} & {0} & {0} & {\dots} & {1} & {\widetilde {U} _ {i (i + 1)}} & {\dots} & {\widetilde {U} _ {i n}} \\ {0} & {0} & {0} & {\dots} & {0} & {1} & {\dots} & {\widetilde {U} _ {(i + 1) n}} \\ {\vdots} & {\vdots} & {\vdots} & & {\vdots} & {\vdots} & & {\vdots} \\ {0} & {0} & {0} & {\dots} & {0} & {0} & {\dots} & {1} \end{array} \right] \left[ \begin{array}{l} V _ {1} ^ {(j - 1)} \\ V _ {2} ^ {(j - 1)} \\ \vdots \\ V _ {i} ^ {(j - 1)} \\ V _ {i + 1} ^ {(j - 1)} \\ \vdots \\ V _ {\{j - 1 \}} ^ {(j - 1)} \\ \vdots \\ 0 \\ \vdots \\ 0 \end{array} \right] \text {- - - 第 } i \text {- - - 第 } i + 1 \text {- - - 行}} \end{array}
$$

$\widetilde{U}^{*(j)}$ 的第 $i$ 个分量为

$$
\widetilde {U} _ {i} ^ {* (j)} = \lambda_ {j} [ V _ {i} ^ {(j - 1)} + \sum_ {k = i + 1} ^ {j - 1} \widetilde {U} _ {i k} V _ {k} ^ {(j - 1)} ]\tag{3.7.43}
$$

$\widetilde{U}_i^{*(j)}$ 就是式(3.7.40)方括号所示矩阵的第 $i$ 行第 $j$ 列元，因此由式(3.7.40)得

$$
\hat {U} _ {i j} = \widetilde {U} _ {i j} + \lambda_ {j} [ V _ {i} ^ {(j - 1)} + \sum_ {k = i + 1} ^ {j - 1} \widetilde {U} _ {i k} V _ {k} ^ {(j - 1)} ]\tag{3.7.44}
$$

由式 $(3.7.39)$ 又可写成

$$
\hat {U} _ {i j} = \widetilde {U} _ {i j} + \lambda_ {j} [ g _ {j} + \sum_ {k = i + 1} ^ {j - 1} \widetilde {U} _ {i k} g _ {k} ]
$$

式中 $g_{j}, \lambda_{j}, f_{j}, \alpha_{j}$ 分别由式(3.7.4)、式(3.7.7)、式(3.7.3)、式(3.7.6)确定。

$$
\hat {U} _ {i i} = 1
$$

根据上述推导,可将 U 和 D 的量测更新算法整理如下:

已知 $\tilde{U}$ 和 $\tilde{D}$ ，求 $\hat{U}$ 和 $\hat{D}$ 的方法

$$
\begin{array}{r l} & f = \widetilde {U} ^ {\mathrm{T}} H ^ {\mathrm{T}} \\ & g _ {i} = \widetilde {D} _ {i} f _ {i} \qquad i = 1, 2, \dots , n \\ & \alpha_ {j} = R + \sum_ {i = 1} ^ {j} f _ {i} g _ {i} \qquad j = 1, 2, \dots , n; \quad \alpha_ {0} = R \end{array}
$$

或

$$
\alpha_ {j} = \alpha_ {j - 1} + f _ {j} g _ {j} \quad j = 1, 2, \dots , n
$$

$$
\lambda_ {j} = - \frac {f _ {j}}{\alpha_ {j - 1}} \quad j = 2, 3, \dots , n
$$

$$
\hat {D} _ {j} = \widetilde {D} _ {j} \frac {\alpha_ {j - 1}}{\alpha_ {j}} \qquad j = 1, 2, \dots , n
$$

$$
\begin{array}{r l} \hat {U} _ {i j} = \widetilde {U} _ {i j} + \lambda_ {j} (g _ {i} + \sum_ {k = i + 1} ^ {j - 1} \widetilde {U} _ {i k} g _ {k}) & j = 2, 3, \dots , n; \quad i = 1, 2, \dots , j - 1 \\ \hat {U} _ {i i} = 1 & i = 1, 2, \dots , n \end{array}
$$

此即定理 3.3 所述内容。

## 3.7.3 $U$ 和 $D$ 的时间更新算法

定理3.4

设 $\hat{U}$ 和 $\hat{D}$ 已知， $\hat{U}$ 为主对角元全为 1 的上三角阵， $\hat{D}$ 为对角阵，则 $\tilde{U}$ 和 $\tilde{D}$ 按如下诸式确定：

$$
\widetilde {D} _ {j} = \boldsymbol {W} _ {j} ^ {(n - j)} \boldsymbol {D} \left[ \boldsymbol {W} _ {j} ^ {(n - j)} \right] ^ {\mathrm{T}} \quad j = n, n - 1, \dots 2\tag{3.7.45a}
$$

$$
\widetilde {U} _ {i, j} = \frac {\boldsymbol {W} _ {i} ^ {(n - j)} \boldsymbol {D} [ \boldsymbol {W} _ {j} ^ {(n - j)} ] ^ {\mathrm{T}}}{\widetilde {D} _ {i}} \quad j = n, n - 1, \dots , 2; \quad i = 1, 2, \dots , j - 1\tag{3.7.46}
$$

$$
\boldsymbol {W} _ {i} ^ {(n - j + 1)} = \boldsymbol {W} _ {i} ^ {(n - j)} - \widetilde {U} _ {i, j} \boldsymbol {W} _ {j} ^ {(n - j)} \quad j = n, n - 1, \dots , 2; \quad i = 1, 2, \dots , j - 1\tag{3.7.47}
$$

$$
\widetilde {D} _ {1} = \boldsymbol {W} _ {1} ^ {(n - 1)} \boldsymbol {D} \left[ \boldsymbol {W} _ {1} ^ {(n - 1)} \right] ^ {\mathrm{T}}\tag{3.7.45b}
$$

式中

$$
\boldsymbol {D} = \operatorname{diag} [ \hat {\boldsymbol {D}} \quad \boldsymbol {Q} ]\tag{3.7.48}
$$

$$
\boldsymbol {W} ^ {(0)} = \left[ \begin{array}{c c} \boldsymbol {\Phi} \hat {\boldsymbol {U}} & \boldsymbol {\Gamma} \end{array} \right] = \left[ \begin{array}{c} \boldsymbol {W} _ {1} ^ {(0)} \\ \boldsymbol {W} _ {2} ^ {(0)} \\ \vdots \\ \boldsymbol {W} _ {n} ^ {(0)} \end{array} \right]\tag{3.7.49}
$$

$W_{i}^{(n-j)}$ 为 $1 \times (n + n_{p})$ 的行向量，n 为滤波器维数， $n_{p}$ 为驱动噪声的维数。

注意:本分节所述的 $\hat{U},\hat{D}$ 和 $\tilde{U},\tilde{D}$ 应理解为

$$
\begin{array}{l l} \hat {\boldsymbol {U}} = \boldsymbol {U} _ {k - 1}, & \hat {\boldsymbol {D}} = \boldsymbol {D} _ {k - 1} \\ \tilde {\boldsymbol {U}} = \boldsymbol {U} _ {k / k - 1}, & \tilde {\boldsymbol {D}} = \boldsymbol {D} _ {k / k - 1} \end{array}
$$

证明

取

$$
\boldsymbol {W} = \left[ \begin{array}{l l} \boldsymbol {\Phi} \hat {\boldsymbol {U}} & \boldsymbol {\Gamma} \end{array} \right] = \left[ \begin{array}{c} \boldsymbol {W} _ {1} \\ \boldsymbol {W} _ {2} \\ \vdots \\ \boldsymbol {W} _ {n} \end{array} \right]\tag{3.7.50}
$$

显然 W 为 $n \times (n + n_{p})$ 的扁矩阵，其中 n 为滤波器维数， $n_{p}$ 为驱动噪声的维数。记

$$
N = n + n _ {p}
$$

取

$$
\boldsymbol {D} = \operatorname{diag} [ \hat {\boldsymbol {D}} \quad \boldsymbol {Q} ] = \operatorname{diag} [ D _ {1} \quad D _ {2} \quad \dots \quad D _ {N} ]
$$

考察 $WDW^{T}$ ，由式(3.7.50) 和上式

$$
\boldsymbol {W} \boldsymbol {D} \boldsymbol {W} ^ {\mathrm{T}} = \left[ \begin{array}{l l} \boldsymbol {\Phi} \hat {\boldsymbol {U}} & \boldsymbol {\Gamma} \end{array} \right] \left[ \begin{array}{l l} \hat {\boldsymbol {D}} & \mathbf {0} \\ \mathbf {0} & Q \end{array} \right] \left[ \begin{array}{l} \hat {\boldsymbol {U}} ^ {\mathrm{T}} \boldsymbol {\Phi} ^ {\mathrm{T}} \\ \boldsymbol {\Gamma} ^ {\mathrm{T}} \end{array} \right] = \boldsymbol {\Phi} \hat {\boldsymbol {U}} \hat {\boldsymbol {D}} \hat {\boldsymbol {U}} ^ {\mathrm{T}} \boldsymbol {\Phi} ^ {\mathrm{T}} + \boldsymbol {\Gamma} Q \boldsymbol {\Gamma} ^ {\mathrm{T}}
$$

根据式 $(3.7.1)$ 和式 $(2.2.4d)$

$$
\boldsymbol {\Phi} \hat {\boldsymbol {U}} \hat {\boldsymbol {D}} \hat {\boldsymbol {U}} ^ {\mathrm{T}} \boldsymbol {\Phi} ^ {\mathrm{T}} + \boldsymbol {\Gamma} \boldsymbol {Q} \boldsymbol {\Gamma} ^ {\mathrm{T}} = \boldsymbol {P} _ {k / k - 1}
$$

记

$$
\begin{array}{r l} & \hat {\boldsymbol {P}} = \boldsymbol {P} _ {k - 1} \\ & \tilde {\boldsymbol {P}} = \boldsymbol {P} _ {k / k - 1} = \tilde {\boldsymbol {U}} \tilde {\boldsymbol {D}} \tilde {\boldsymbol {U}} ^ {\mathrm{T}} \end{array}\tag{3.7.51}
$$

则

$$
\widetilde {\boldsymbol {P}} = \boldsymbol {W} \boldsymbol {D} \boldsymbol {W} ^ {\mathrm{T}}\tag{3.7.52}
$$

令

$$
\boldsymbol {W} = \bar {\boldsymbol {U}} \tilde {\boldsymbol {W}}\tag{3.7.53}
$$

式中， $\bar{U}$ 为 $n \times n$ 的某一变换矩阵， $\widetilde{W}$ 为 $n \times N$ 的某一扁矩阵。若能从式(3.7.53)中确定出 $\bar{U}$ 和 $\widetilde{W}$ ，其中 $\widetilde{W}$ 的各行向量 $\widetilde{W}_{i}(i=1,2,\cdots,n)$ 关于 D 正交，即 D 加权内积为

$$
<   \widetilde {W} _ {i}, \widetilde {W} _ {j} > _ {D} = \widetilde {W} _ {i} D \widetilde {W} _ {j} ^ {\mathrm{T}} = \widetilde {D} _ {j} \delta_ {i j}\tag{3.7.54}
$$

式中， $\widetilde{D}_j$ 为 $\tilde{\pmb{D}}$ 的主对角元。则式(3.7.52)成

$$
\tilde {\boldsymbol {P}} = \bar {\bar {\boldsymbol {U}}} \tilde {\boldsymbol {W}} \boldsymbol {D} \tilde {\boldsymbol {W}} ^ {\mathrm{T}} \bar {\bar {\boldsymbol {U}}} ^ {\mathrm{T}} = \bar {\bar {\boldsymbol {U}}} \tilde {\boldsymbol {D}} \bar {\bar {\boldsymbol {U}}} ^ {\mathrm{T}}
$$

与式(3.7.51)进行比较,可看出 $\bar{U}$ 就是 $\tilde{P}$ 的 $UDU^{T}$ 分解中的 $\tilde{U}$ , 即

$$
\tilde {\boldsymbol {U}} = \bar {\boldsymbol {U}}\tag{3.7.55}
$$

$$
\widetilde {\boldsymbol {D}} = \widetilde {\boldsymbol {W}} \boldsymbol {D} \widetilde {\boldsymbol {W}} ^ {\mathrm{T}}\tag{3.7.56}
$$

下面根据这一思路,确定出 $\tilde{U}$ 和 $\tilde{D}$ 。

为推导方便起见，记

$$
\boldsymbol {W} _ {i} ^ {(0)} = \boldsymbol {W} _ {i} \quad i = 1, 2, \dots , n\tag{3.7.57}
$$

式中， $W_{i}^{(0)}(k)=W_{i}(k)(k=1,2,\cdots,N,N=n+n_{p})$ 是 $W_{i}$ 的第k个元。
取

$$
\widetilde {W} _ {n} = W _ {n}\tag{3.7.58}
$$

并记

$$
\boldsymbol {W} _ {n} = \boldsymbol {W} ^ {0}\tag{3.7.59}
$$

将 $\boldsymbol{W}_{i}^{(0)}(i=1,2,\cdots,n)$ 沿 $\widetilde{W}_{n}$ 方向和与 $\widetilde{W}_{n}D$ 加权正交（沿该方向的向量与 $\widetilde{W}_{n}$ 的 D 加权内积为零）的方向分解。记 $W_{i}^{*}$ 为 $\boldsymbol{W}_{i}^{(0)}$ 沿 $\widetilde{W}_{n}$ 方向的分量， $\boldsymbol{W}_{i}^{(1)}$ 为沿正交方向的分量，如图 3.7.1 所示。

由图3.7.1得

$$
\boldsymbol {W} _ {i} ^ {(0)} = \boldsymbol {W} _ {i} ^ {(1)} + \boldsymbol {W} _ {i} ^ {*}
$$

因为 $W_{i}^{*}$ 是沿 $\widetilde{W}_{n}$ 的方向，即

$$
\boldsymbol {W} _ {i} ^ {*} = <   \boldsymbol {W} _ {i} ^ {(0)}, \widetilde {\boldsymbol {W}} _ {n} > _ {D} \cdot \frac {\widetilde {\boldsymbol {W}} _ {n}}{\| \widetilde {\boldsymbol {W}} _ {n} \| _ {D}} = \frac {<   \boldsymbol {W} _ {i} ^ {(0)} , \widetilde {\boldsymbol {W}} _ {n} > _ {D}}{<   \widetilde {\boldsymbol {W}} _ {n} , \widetilde {\boldsymbol {W}} _ {n} > _ {D}} \widetilde {\boldsymbol {W}} _ {n}
$$

所以

![](images/da6513092fb8f1b5ff6ac6a1f4232221fe9a72f904dff8d453eb4045e28e6ade.jpg)  
图3.7.1 $W_{i}^{(0)}$ 沿 $\widetilde{W}_n$ 方向和沿 $\widetilde{W}_n$ 的 $D$ 加权正交方向的分量

$$
\boldsymbol {W} _ {i} ^ {(0)} = \boldsymbol {W} _ {i} ^ {(1)} + \frac {<   \boldsymbol {W} _ {i} ^ {(0)} , \widetilde {\boldsymbol {W}} _ {n} > _ {D}}{<   \widetilde {\boldsymbol {W}} _ {n} , \widetilde {\boldsymbol {W}} _ {n} > _ {D}} \widetilde {\boldsymbol {W}} _ {n} \quad i = 1, 2, \dots , n\tag{3.7.60}
$$

其中加权内积的定义为

$$
<   W _ {i}, W _ {j} > _ {D} = W _ {i} D W _ {j} ^ {\mathrm{T}}
$$

式中， $W_{i}$ 为行向量，D 为矩阵。

当 i=n 时, 考虑到式(3.7.58)所列关系, 由式(3.7.60)得

$$
\boldsymbol {W} _ {n} ^ {(0)} = \boldsymbol {W} _ {n} = \boldsymbol {W} _ {n} ^ {(1)} + \frac {<   \boldsymbol {W} _ {n} ^ {(0)} , \widetilde {\boldsymbol {W}} _ {n} > _ {D}}{<   \widetilde {\boldsymbol {W}} _ {n} , \widetilde {\boldsymbol {W}} _ {n} > _ {D}} \widetilde {\boldsymbol {W}} _ {n} = \boldsymbol {W} _ {(n)} ^ {(1)} + \boldsymbol {W} _ {n}
$$

即

$$
\boldsymbol {W} _ {n} = \boldsymbol {W} _ {n} ^ {(1)} + \boldsymbol {W} _ {n}
$$

所以

$$
\boldsymbol {W} _ {n} ^ {(1)} = \boldsymbol {0}\tag{3.7.61}
$$

将式(3.7.60)写成矩阵形式,并考虑到式(3.7.57)及式(3.7.61)得

$$
\left[ \begin{array}{c} \boldsymbol {W} _ {1} \\ \boldsymbol {W} _ {2} \\ \vdots \\ \boldsymbol {W} _ {n} \end{array} \right] = \left[ \begin{array}{c} \boldsymbol {W} _ {1} ^ {(1)} \\ \boldsymbol {W} _ {2} ^ {(1)} \\ \vdots \\ \boldsymbol {W} _ {n - 1} ^ {(1)} \\ \boldsymbol {0} \end{array} \right] + \left[ \begin{array}{c} \bar {\bar {U}} _ {1, n} \\ \bar {\bar {U}} _ {2, n} \\ \vdots \\ \bar {\bar {U}} _ {n - 1, n} \\ 1 \end{array} \right] \widetilde {\boldsymbol {W}} _ {n}\tag{3.7.62}
$$

式中

$$
\bar {U} _ {i, n} = \frac {<   W _ {i} ^ {(0)} , \widetilde {W} _ {n} > _ {D}}{<   \widetilde {W} _ {n} , \widetilde {W} _ {n} > _ {D}} = \frac {<   W _ {i} ^ {(0)} , W _ {n} ^ {(0)} > _ {D}}{<   W _ {n} ^ {(0)} , W _ {n} ^ {(0)} > _ {D}} \quad i = 1, 2, \dots , n - 1\tag{3.7.63}
$$

这样式(3.7.60)也可改写成

$$
\boldsymbol {W} _ {i} ^ {(1)} = \left\{ \begin{array}{l l} \boldsymbol {W} _ {i} ^ {(0)} - \bar {\boldsymbol {U}} _ {i, n} \boldsymbol {W} _ {n} ^ {(0)} & i = 1, 2, \dots , n - 1 \\ \boldsymbol {0} & i = n \end{array} \right.\tag{3.7.64}
$$

取

$$
\widetilde {W} _ {n - 1} = W _ {n - 1} ^ {(1)}\tag{3.7.65}
$$

并将 $W_{i}^{(1)}(i=1,2,\cdots,n-1)$ 沿 $\widetilde{W}_{n-1}$ 方向和与 $\widetilde{W}_{n-1}D$ 加权正交的方向分解。仿照以上分析方法得

$$
\begin{array}{r l} & W _ {i} ^ {(2)} = W _ {i} ^ {(1)} - \frac {<   W _ {i} ^ {(1)} , \widetilde {W} _ {n - 1} > _ {D}}{<   \widetilde {W} _ {n - 1} , \widetilde {W} _ {n - 1} > _ {D}} \widetilde {W} _ {n - 1} = W _ {i} ^ {(1)} - \bar {U} _ {i, n - 1} \widetilde {W} _ {n - 1} = \\ & W _ {i} ^ {(1)} - \bar {\bar {U}} _ {i, n - 1} W _ {n - 1} ^ {(1)} \quad i = 1, 2, \dots , n - 2 \end{array}
$$

可得

$$
\boldsymbol {W} _ {i} ^ {(2)} = \left\{ \begin{array}{l l} \mathbf {0} & i = n - 1, n \\ \boldsymbol {W} _ {i} ^ {(1)} - \bar {\boldsymbol {U}} _ {i, n - 1} \boldsymbol {W} _ {n - 1} ^ {(1)} & i = 1, 2, \dots , n - 2 \end{array} \right.\tag{3.7.66}
$$

所以

$$
\boldsymbol {W} _ {i} ^ {(1)} = \boldsymbol {W} _ {i} ^ {(2)} + \bar {\boldsymbol {U}} _ {i, n - 1} \tilde {\boldsymbol {W}} _ {n - 1} \quad i = 1, 2, \dots , n - 2\tag{3.7.67}
$$

再由式 $(3.7.65)$ 可得

$$
\left[ \begin{array}{c} \boldsymbol {W} _ {1} ^ {(1)} \\ \boldsymbol {W} _ {2} ^ {(1)} \\ \vdots \\ \boldsymbol {W} _ {n - 2} ^ {(1)} \\ \boldsymbol {W} _ {n - 1} ^ {(1)} \end{array} \right] = \left[ \begin{array}{c} \boldsymbol {W} _ {1} ^ {(2)} \\ \boldsymbol {W} _ {2} ^ {(2)} \\ \vdots \\ \boldsymbol {W} _ {n - 2} ^ {(2)} \\ \boldsymbol {0} \end{array} \right] + \left[ \begin{array}{c} \bar {\bar {U}} _ {1, n - 1} \\ \bar {\bar {U}} _ {2, n - 1} \\ \vdots \\ \bar {\bar {U}} _ {n - 2, n - 1} \\ 1 \end{array} \right] \widetilde {\boldsymbol {W}} _ {n - 1}\tag{3.7.68}
$$

式中

$$
\bar {U} _ {i, n - 1} = \frac {<   W _ {i} ^ {(1)} , \tilde {W} _ {n - 1} > _ {D}}{<   \tilde {W} _ {n - 1} , \tilde {W} _ {n - 1} > _ {D}} = \frac {<   W _ {i} ^ {(1)} , W _ {n - 1} ^ {(1)} > _ {D}}{<   W _ {n - 1} ^ {(1)} , W _ {n - 1} ^ {(1)} > _ {D}} \quad i = 1, 2, \dots , n - 2\tag{3.7.69}
$$

将式 $(3.7.68)$ 代入式 $(3.7.62)$ 得

$$
\left[ \begin{array}{c} \boldsymbol {W} _ {1} \\ \boldsymbol {W} _ {2} \\ \vdots \\ \boldsymbol {W} _ {n - 2} \\ \boldsymbol {W} _ {n - 1} \\ \boldsymbol {W} _ {n} \end{array} \right] = \left[ \begin{array}{c} \boldsymbol {W} _ {1} ^ {(2)} \\ \boldsymbol {W} _ {2} ^ {(2)} \\ \vdots \\ \boldsymbol {W} _ {n - 2} ^ {(2)} \\ \boldsymbol {0} \\ \boldsymbol {0} \end{array} \right] + \left[ \begin{array}{c c} \bar {\bar {U}} _ {1, n - 1} & \bar {\bar {U}} _ {1, n} \\ \bar {\bar {U}} _ {2, n - 1} & \bar {\bar {U}} _ {2, n} \\ \vdots & \vdots \\ \bar {\bar {U}} _ {n - 2, n - 1} & \bar {\bar {U}} _ {n - 2, n} \\ 1 & \bar {\bar {U}} _ {n - 1, n} \\ 0 & 1 \end{array} \right] \left[ \begin{array}{c c} \widetilde {\boldsymbol {W}} _ {n - 1} & \\ \widetilde {\boldsymbol {W}} _ {n} & \end{array} \right]\tag{3.7.70}
$$

仿照上述方法推演,可得 W 与 $\tilde{W}$ 间的线性关系式

$$
\left[ \begin{array}{c} \widetilde {W} _ {1} \\ \widetilde {W} _ {2} \\ \vdots \\ \widetilde {W} _ {n} \end{array} \right] = \left[ \begin{array}{c c c c c} 1 & \bar {\bar {U}} _ {1, 2} & \dots & \bar {\bar {U}} _ {1, n - 1} & \bar {\bar {U}} _ {1, n} \\ 0 & 1 & \dots & \bar {\bar {U}} _ {2, n - 1} & \bar {\bar {U}} _ {2, n} \\ 0 & 0 & \dots & \bar {\bar {U}} _ {3, n - 1} & \bar {\bar {U}} _ {3, n} \\ \vdots & \vdots & & \vdots & \vdots \\ 0 & 0 & \dots & 1 & \bar {\bar {U}} _ {n - 1, n} \\ 0 & 0 & \dots & 0 & 1 \end{array} \right] \left[ \begin{array}{c} \widetilde {W} _ {1} \\ \widetilde {W} _ {2} \\ \widetilde {W} _ {3} \\ \vdots \\ \widetilde {W} _ {n - 1} \\ \widetilde {W} _ {n} \end{array} \right]\tag{3.7.71}
$$

现在要问,上式中的 $\widetilde{W}_{1},\widetilde{W}_{2},\cdots,\widetilde{W}_{n}$ 是否关于 D 互相正交呢?

在第一次分解中，由式(3.7.62)知， $\widetilde{W}_{n}$ 与 $W_{i}^{(1)}$ $(i=1,2,\cdots,n-1)$ 关于 D 正交。在第二次分解中，由式(3.7.65)知 $\widetilde{W}_{n-1}$ 就是 $W_{n-1}^{(1)}$ ，所以 $\widetilde{W}_{n}$ 与 $\widetilde{W}_{n-1}$ 关于 D 正交。又由式(3.7.66)知

$$
\begin{array}{r l} <   \widetilde {W} _ {n}, W _ {i} ^ {(2)} > _ {D} = & <   \widetilde {W} _ {n}, W _ {i} ^ {(1)} > _ {D} - \bar {U} _ {i, n - 1} <   \widetilde {W} _ {n}, \widetilde {W} _ {n - 1} > _ {D} = 0 \\ & i = 1, 2, \dots , n - 2 \end{array}
$$

所以 $\widetilde{W}_n$ 与 $W_{n - 2}^{(2)}$ 关于 $\pmb{D}$ 正交。

在第三次分解中，取 $\widetilde{W}_{n-2} = W_{n-2}^{(2)}$ ，所以 $\widetilde{W}_{n}$ 与 $\widetilde{W}_{n-2}$ 关于 D 正交。很明显， $\widetilde{W}_{n-1}$ 与 $\widetilde{W}_{n-2}$ 关于 D 正交。如此推断下去，可推证得 $\widetilde{W}_{n}, \widetilde{W}_{n-1}, \cdots, \widetilde{W}_{2}, \widetilde{W}_{1}$ 关于 D 互相正交。

记

$$
\bar {\bar {U}} = \left[ \begin{array}{c c c c c} 1 & \bar {\bar {U}} _ {1, 2} & \dots & \bar {\bar {U}} _ {1, n - 1} & \bar {\bar {U}} _ {1, n} \\ 0 & 1 & \dots & \bar {\bar {U}} _ {2, n - 1} & \bar {\bar {U}} _ {2, n} \\ 0 & 0 & \dots & \bar {\bar {U}} _ {3, n - 1} & \bar {\bar {U}} _ {3, n} \\ \vdots & \vdots & & \vdots & \vdots \\ 0 & 0 & \dots & 1 & \bar {\bar {U}} _ {n - 1, n} \\ 0 & 0 & \dots & 0 & 1 \end{array} \right]\tag{3.7.72}
$$

则

$$
\boldsymbol {W} = \bar {\boldsymbol {U}} \tilde {\boldsymbol {W}}
$$

所以

$$
\widetilde {\boldsymbol {P}} = \boldsymbol {W} \boldsymbol {D} \boldsymbol {W} ^ {\mathrm{T}} = \overline {{\boldsymbol {U}}} \widetilde {\boldsymbol {W}} \boldsymbol {D} \widetilde {\boldsymbol {W}} ^ {\mathrm{T}} \overline {{\boldsymbol {U}}} ^ {\mathrm{T}}
$$

记

$$
\bar {\boldsymbol {D}} = \widetilde {\boldsymbol {W}} \boldsymbol {D} \widetilde {\boldsymbol {W}} ^ {\mathrm{T}}
$$

由于 $\widetilde{W}_{n}, \widetilde{W}_{n-1}, \cdots, \widetilde{W}_{1}$ 关于 D 互相正交，所以 $\bar{D}$ 为对角阵。

$$
\tilde {\boldsymbol {P}} = \overline {{\boldsymbol {U}}} \overline {{\boldsymbol {D}}} \overline {{\boldsymbol {U}}} ^ {\mathrm{T}}\tag{3.7.73}
$$

由于非负定阵 $UDU^{T}$ 分解的唯一性, 比较式(3.7.73) 和式(3.7.51), 得

$$
\tilde {\boldsymbol {U}} = \bar {\boldsymbol {U}}\tag{3.7.74}
$$

$$
\tilde {\boldsymbol {D}} = \bar {\boldsymbol {D}} = \tilde {\boldsymbol {W}} \boldsymbol {D} \tilde {\boldsymbol {W}} ^ {\mathrm{T}}\tag{3.7.75}
$$

$\tilde{U}$ 的非主对角非零元可由式(3.7.63)和式(3.7.69)确定出通项公式

$$
\widetilde {U} _ {i, n - \mu} = \frac {<   W _ {i} ^ {(\mu)} , W _ {n - \mu} ^ {(\mu)} > _ {D}}{<   W _ {n - \mu} ^ {(\mu)} , W _ {n - \mu} ^ {(\mu)} > _ {D}}\tag{3.7.76}
$$

式中

$$
\mu = 0, 1, 2, \dots , n - 2; \quad i = 1, 2, \dots , n - (\mu + 1)
$$

在式(3.7.76)中， $\mu_{max}=n-2$ 的原因是：由式(3.7.72)知， $\widetilde{U}$ 的最左上角的非主对角元为 $\widetilde{U}_{1,2}$ ，即

$$
(n - \mu) _ {\min} = 2
$$

所以

$$
\mu_ {\max} = n - 2
$$

可将式 $(3.7.76)$ 改写成另一种便于使用的形式：

令 $n-\mu=j$ ，由于 $\mu=0,1,2,\cdots,n-2$ ，所以 $j=n,n-1,\cdots,2$ ，得

式中

$$
\begin{array}{r l} \widetilde {U} _ {i, j} & = \frac {<   W _ {i} ^ {(n - j)} , W _ {j} ^ {(n - j)} > _ {D}}{<   W _ {j} ^ {(n - j)} , W _ {j} ^ {(n - j)} > _ {D}} \\ j & = n, n - 1, \dots , 2; \quad i = 1, 2, \dots , j - 1 \end{array}
$$

$W_{i}$ 的各次关于 D 的分解由递推计算获得。由式(3.7.64)和式(3.7.66)，可得各次分解的通项公式为

$$
\boldsymbol {W} _ {i} ^ {(\gamma)} = \boldsymbol {W} _ {i} ^ {(\gamma - 1)} - \widetilde {U} _ {i, n - \gamma + 1} \boldsymbol {W} _ {n - \gamma + 1} ^ {(\gamma - 1)}
$$

式中

$$
\gamma = 1, 2, \dots , n - 1; \quad i = 1, 2, \dots , n - \gamma
$$

为使该关系式与式(3.7.47)的下标一致,取 $n-\gamma+1=j$ ,则 $j=n,n-1,\cdots,2$ 。这样各次分解的通项公式为

$$
\boldsymbol {W} _ {i} ^ {(n - j + 1)} = \boldsymbol {W} _ {i} ^ {(n - j)} - \widetilde {U} _ {i, j} \boldsymbol {W} _ {j} ^ {(n - j)}
$$

式中

$$
j = n, n - 1, \dots , 2; \quad i = 1, 2, \dots , j - 1
$$

式(3.7.46)和式(3.7.47)就是求解 $\tilde{U}$ 的递推公式。

下面确定 $\tilde{D}$ 。

由于 $\widetilde{W}_{1},\widetilde{W}_{2},\cdots,\widetilde{W}_{n}$ 关于 D 互相正交，即有

$$
<   \widetilde {W} _ {i}, \widetilde {W} _ {j} > _ {D} = \widetilde {W} _ {i} D \widetilde {W} _ {j} ^ {\mathrm{T}} = \widetilde {D} _ {j} \delta_ {i j}
$$

而在各次关于 D 的分解中, 取

$$
\begin{array}{r l} \widetilde {W} _ {n} & = W _ {n} ^ {(0)} \\ \widetilde {W} _ {n - 1} & = W _ {n - 1} ^ {(1)} \\ & \dots \dots \\ \widetilde {W} _ {1} & = W _ {1} ^ {(n - 1)} \end{array}
$$

所以

$$
\begin{array}{r l} \widetilde {D} _ {n} & = <   \widetilde {W} _ {n}, \widetilde {W} _ {n} > _ {D} = <   W _ {n} ^ {(0)}, W _ {n} ^ {(0)} > _ {D} \\ \widetilde {D} _ {n - 1} & = <   \widetilde {W} _ {n - 1}, \widetilde {W} _ {n - 1} > _ {D} = <   W _ {n - 1} ^ {(1)}, W _ {n - 1} ^ {(1)} > _ {D} \end{array}
$$

$$
\widetilde {D} _ {n - \mu} = <   \widetilde {W} _ {n - \mu}, \widetilde {W} _ {n - \mu} > _ {D} = <   W _ {n - \mu} ^ {(\mu)}, W _ {n - \mu} ^ {(\mu)} > _ {D} \quad \mu = 0, 1, 2, \dots , n - 1
$$

令 $n-\mu=j$ ，则 $j=n,n-1,\cdots,1$ ，上述诸式可写成

$$
\widetilde {D} _ {j} = <   \boldsymbol {W} _ {j} ^ {(n - j)}, \boldsymbol {W} _ {j} ^ {(n - j)} > _ {D} \quad j = n, n - 1, \dots , 1
$$

将 $\tilde{U}$ 和 $\tilde{D}$ 的求取同时进行，则 U 和 D 的时间更新算法如下：

$$
\begin{array}{r l} & \widetilde {D} _ {j} = <   W _ {j} ^ {(n - j)}, W _ {j} ^ {(n - j)} > _ {D} \\ & \widetilde {U} _ {i, j} = \frac {<   W _ {i} ^ {(n - j)} , W _ {j} ^ {(n - j)} > _ {D}}{\widetilde {D} _ {j}} \\ & W _ {i} ^ {(n - j + 1)} = W _ {i} ^ {(n - j)} - \widetilde {U} _ {i, j} W _ {j} ^ {(n - j)} \quad j = n, n - 1, \dots , 2; \quad i = 1, 2, \dots , j - 1 \\ & \widetilde {D} _ {1} = <   W _ {1} ^ {(n - 1)}, W _ {1} ^ {(n - 1)} > _ {D} \\ & W _ {i} ^ {(0)} = W _ {i} \quad i = 1, 2, \dots , n \end{array}
$$

式中，D 和 W 分别按式(3.7.48)和式(3.7.49)确定。

获得 $\tilde{U}$ 和 $\tilde{D}$ 后，可进行状态估计和最佳增益阵的计算。根据式(2.2.4a)、式(2.2.4b) 和式(2.2.4c)，并应用式(3.7.11)、式(3.7.3) 得

$$
\begin{array}{r l} \hat {\mathbf {X}} _ {k / k - 1} & = \boldsymbol {\Phi} _ {k, k - 1} \hat {\mathbf {X}} _ {k - 1} \\ \mathbf {K} _ {k} & = \frac {1}{\alpha_ {n}} \tilde {\mathbf {U}} \tilde {\mathbf {D}} f \\ \hat {\mathbf {X}} _ {k} & = \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} (Z _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1}) \end{array}
$$

上述诸式及定理 3.3 和定理 3.4 即为 $UDU^{T}$ 分解滤波的全套算法。

需要说明一点: 如果 $\widetilde{D}_{i}=0$ , 则 $\widetilde{\boldsymbol{U}}^{(i)}$ 可任选, 一般可选

$$
\widetilde {\pmb {U}} ^ {(i)} = \pmb {e} _ {i} = \left[ \begin{array}{c} {{0}} \\ {{\vdots}} \\ {{0}} \\ {{1}} \\ {{0}} \\ {{\vdots}} \\ {{0}} \end{array} \right] — — \text {第} i \text {个分量}
$$

下面说明其原因：

记

$$
\widetilde {U} = \left[ \begin{array}{c c c c c c} \widetilde {U} ^ {(1)} & \widetilde {U} ^ {(2)} & \dots & \widetilde {U} ^ {(i)} & \dots & \widetilde {U} ^ {(n)} \end{array} \right]
$$

式中

$$
\widetilde {U} ^ {(i)} = \left[ \begin{array}{l l l l l l l} \widetilde {U} _ {1} ^ {(i)} & \widetilde {U} _ {2} ^ {(i)} & \dots & \widetilde {U} _ {i - 1} ^ {(i)} & 1 & 0 & \dots & 0 \end{array} \right] ^ {\mathrm{T}} \quad i = 2, 3, \dots , n
$$

则

$$
\begin{array}{r l} \widetilde {U} \widetilde {D} & = \left[ \begin{array}{c c c c c c} 1 & \widetilde {U} _ {1} ^ {(2)} & \dots & \widetilde {U} _ {1} ^ {(i)} & \dots & \widetilde {U} _ {1} ^ {(n)} \\ 0 & 1 & \dots & \widetilde {U} _ {2} ^ {(i)} & \dots & \widetilde {U} _ {2} ^ {(n)} \\ \vdots & \vdots & & \vdots & & \vdots \\ 0 & 0 & \dots & 1 & \dots & \widetilde {U} _ {i} ^ {(n)} \\ \vdots & \vdots & & \vdots & & \vdots \\ 0 & 0 & \dots & 0 & \dots & 1 \end{array} \right] \left[ \begin{array}{c c c c c c} \widetilde {D} _ {1} & 0 & \dots & 0 & \dots & 0 \\ 0 & \widetilde {D} _ {2} & \dots & 0 & \dots & 0 \\ \vdots & \vdots & & \vdots & & \vdots \\ 0 & 0 & \dots & \widetilde {D} _ {i} & \dots & 0 \\ \vdots & \vdots & & \vdots & & \vdots \\ 0 & 0 & \dots & 0 & \dots & \widetilde {D} _ {n} \end{array} \right] = \\ & [ \begin{array}{c c c c c c} \widetilde {D} _ {1} & \widetilde {D} _ {2} \widetilde {U} _ {1} ^ {(2)} & \dots & \widetilde {D} _ {i} \widetilde {U} _ {1} ^ {(i)} & \dots & \widetilde {D} _ {n} \widetilde {U} _ {1} ^ {(n)} \\ 0 & \widetilde {D} _ {2} & \dots & \widetilde {D} _ {i} \widetilde {U} _ {2} ^ {(i)} & \dots & \widetilde {D} _ {n} \widetilde {U} _ {2} ^ {(n)} \\ \vdots & \vdots & & \vdots & & \vdots \\ 0 & 0 & \dots & \widetilde {D} _ {i} & \dots & \widetilde {D} _ {n} \widetilde {U} _ {i} ^ {(n)} \\ \vdots & \vdots & & \vdots & & \vdots \\ 0 & 0 & \dots & 0 & \dots & \widetilde {D} _ {n} \end{array} ] = \\ [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [   [    ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ] ],)
$$

$$
\begin{array}{r l} \widetilde {\boldsymbol {P}} = & \widetilde {\boldsymbol {U}} \widetilde {\boldsymbol {D}} \widetilde {\boldsymbol {U}} ^ {\mathrm{T}} = [ \widetilde {D} _ {1} \widetilde {\boldsymbol {U}} ^ {(1)} \quad \widetilde {D} _ {2} \widetilde {\boldsymbol {U}} ^ {(2)} \quad \dots \quad D _ {i} \widetilde {\boldsymbol {U}} ^ {(i)} \quad \dots \quad \widetilde {D} _ {n} \widetilde {\boldsymbol {U}} ^ {(n)} ] \\ & \left[ \begin{array}{c} \widetilde {\boldsymbol {U}} ^ {(1) \mathrm{T}} \\ \widetilde {\boldsymbol {U}} ^ {(2) \mathrm{T}} \\ \vdots \\ \widetilde {\boldsymbol {U}} ^ {(i) \mathrm{T}} \\ \vdots \\ \widetilde {\boldsymbol {U}} ^ {(n) \mathrm{T}} \end{array} \right] = \\ & \widetilde {D} _ {1} \widetilde {\boldsymbol {U}} ^ {(1)} \widetilde {\boldsymbol {U}} ^ {(1) \mathrm{T}} + \widetilde {D} _ {2} \widetilde {\boldsymbol {U}} ^ {(2)} \widetilde {\boldsymbol {U}} ^ {(2) \mathrm{T}} + \dots + \widetilde {D} _ {i} \widetilde {\boldsymbol {U}} ^ {(i)} \widetilde {\boldsymbol {U}} ^ {(i) \mathrm{T}} + \dots + \widetilde {D} _ {n} \widetilde {\boldsymbol {U}} ^ {(n)} \widetilde {\boldsymbol {U}} ^ {(n) \mathrm{T}} \end{array}
$$

若 $\widetilde{D}_i = 0$ ，则 $\widetilde{D}_i\widetilde{U}^{(i)}\widetilde{U}^{(i)\mathrm{T}} = 0,\widetilde{U}^{(i)}$ 不管取什么值均对 $\widetilde{P}$ 无影响，所以当 $\widetilde{D}_i = 0$ 时，可取 $\widetilde{U}^{(i)} = e_i$ 。

## 3.8 自适应滤波

自适应滤波也是一种具有抑制滤波器发散作用的滤波方法,它在滤波计算中,一方面利用量测不断地修正预测值,同时也对未知的或不确切知道的系统模型参数和噪声统计参数进行估计或修正。本节只讨论系统模型参数已知,而噪声统计参数 Q 和 R 未知或不确切知道时的自适应滤波。由于 Q 和 R 等参数最终是通过增益阵 $K_{k}$ 影响滤波值的,因此进行自适应滤波时,也可以不去估计 Q 和 R 等参数而直接根据量测数据估计 $K_{k}$ 。

自适应滤波的类型很多,大体上可分为贝叶斯法、极大似然法、相关法与协方差匹配法四类[75]。其中最基本的是相关法,而相关法中又以输出相关法最简单。因此本节以输出相关法

为例来说明相关法自适应滤波的基本思想。

设线性定常系统完全可控和完全可观测，系统方程和量测方程分别为

$$
\boldsymbol {X} _ {k} = \boldsymbol {\Phi} \boldsymbol {X} _ {k - 1} + \boldsymbol {W} _ {k - 1}\tag{3.8.1}
$$

$$
\mathbf {Z} _ {k} = \mathbf {H X} _ {k} + \mathbf {V} _ {k}\tag{3.8.2}
$$

式中， $W_{k}$ 和 $V_{k}$ 都是零均值的白噪声序列，对应的方差阵 Q 和 R 都是未知阵， $\Phi$ 和 H 是已知阵。假设滤波器已达到稳态，增益阵 $K_{k}$ 已趋近于稳态值 K。

输出相关法自适应滤波的基本途径是根据观测数据 $\{Z_{i}\}$ 估计出输出相关函数序列 $\{C_{i}\}$ ，再由 $\{C_{i}\}$ 推算出最佳稳态增益阵K，使得增益K不断地与实际量测数据 $\{Z_{i}\}$ 相适应。

1. 量测数据的相关函数 $C_{k}(k=1,2,\cdots,n)$ 与 $TH^{T}$ 阵

由状态方程和量测方程得

$$
\begin{array}{r l} & {\mathbf {X} _ {i} = \pmb {\Phi} \mathbf {X} _ {i - 1} + \mathbf {W} _ {i - 1} = \pmb {\Phi} ^ {k} \mathbf {X} _ {i - k} + \sum_ {l = 1} ^ {k} \pmb {\Phi} ^ {l - 1} \mathbf {W} _ {i - l}} \\ & {\mathbf {Z} _ {i} = \mathbf {H} \mathbf {X} _ {i} + \mathbf {V} _ {i} = \mathbf {H} \pmb {\Phi} ^ {k} \mathbf {X} _ {i - k} + \mathbf {H} \sum_ {l = 1} ^ {k} \pmb {\Phi} ^ {l - 1} \mathbf {W} _ {i - l} + \mathbf {V} _ {i}} \end{array}
$$

显然 $\{X_{i}\}$ 为平稳序列，记

$$
\boldsymbol {\Gamma} = E \left[ \boldsymbol {X} _ {i} \boldsymbol {X} _ {i} ^ {\mathrm{T}} \right]
$$

由于 $\pmb{\Gamma}$ 与 $i$ 无关，再考虑到 $\{\mathbf{V}_i\} ,\{\mathbf{W}_i\} ,\mathbf{X}_0$ 之间的不相关性，得 $\{\mathbf{Z}_i\}$ 的相关函数

$$
\boldsymbol {C} _ {0} = E \left[ \boldsymbol {Z} _ {i} \boldsymbol {Z} _ {i} ^ {\mathrm{T}} \right] = \boldsymbol {H} \boldsymbol {\Gamma} \boldsymbol {H} ^ {\mathrm{T}} + \boldsymbol {R}\tag{3.8.3}
$$

$$
\boldsymbol {C} _ {k} = E \left[ \boldsymbol {Z} _ {i} \boldsymbol {Z} _ {i - k} ^ {\mathrm{T}} \right] = \boldsymbol {H} \boldsymbol {\Phi} ^ {k} \boldsymbol {\Gamma} \boldsymbol {H} ^ {\mathrm{T}} \quad k > 0\tag{3.8.4}
$$

以上两式都含有 $TH^{T}$ ，该阵又与增益有关，所以 $TH^{T}$ 是沟通待求增益阵 K 与 $\{C_{k}\}$ 间关系的桥梁，是输出相关自适应滤波中的一个重要矩阵。根据式(3.8.4)可得

$$
\left[ \begin{array}{c} \mathbf {C} _ {1} \\ \mathbf {C} _ {2} \\ \vdots \\ \mathbf {C} _ {n} \end{array} \right] = \left[ \begin{array}{c} \mathbf {H} \boldsymbol {\Phi} \boldsymbol {\Gamma} \mathbf {H} ^ {\mathrm{T}} \\ \mathbf {H} \boldsymbol {\Phi} ^ {2} \boldsymbol {\Gamma} \mathbf {H} ^ {\mathrm{T}} \\ \vdots \\ \mathbf {H} \boldsymbol {\Phi} ^ {n} \boldsymbol {\Gamma} \mathbf {H} ^ {\mathrm{T}} \end{array} \right] = \left[ \begin{array}{c} \mathbf {H} \boldsymbol {\Phi} \\ \mathbf {H} \boldsymbol {\Phi} ^ {2} \\ \vdots \\ \mathbf {H} \boldsymbol {\Phi} ^ {n} \end{array} \right] \boldsymbol {\Gamma} \mathbf {H} ^ {\mathrm{T}} = \mathbf {A} \boldsymbol {\Gamma} \mathbf {H} ^ {\mathrm{T}}\tag{3.8.5}
$$

式中，n 是状态的维数，A 为系统的可观测阵

$$
\boldsymbol {A} = \left[ \begin{array}{c} \boldsymbol {H} \boldsymbol {\Phi} \\ \boldsymbol {H} \boldsymbol {\Phi} ^ {2} \\ \vdots \\ \boldsymbol {H} \boldsymbol {\Phi} ^ {n} \end{array} \right]\tag{3.8.6}
$$

根据系统完全可观测的假设, $\operatorname{rank}A=n,A^{T}A$ 为非奇异阵,于是由式(3.8.5)可解得

$$
\boldsymbol {\Gamma} \boldsymbol {H} ^ {\mathrm{T}} = (\boldsymbol {A} ^ {\mathrm{T}} \boldsymbol {A}) ^ {- 1} \boldsymbol {A} ^ {\mathrm{T}} \left[ \begin{array}{c} \boldsymbol {C} _ {1} \\ \boldsymbol {C} _ {2} \\ \vdots \\ \boldsymbol {C} _ {n} \end{array} \right]\tag{3.8.7}
$$

2. 由 $\Gamma H^{T}$ 求取最佳稳态增益阵 K

由式 $(2.2.4c)$ ，最佳稳态增阵为

$$
\boldsymbol {K} = \boldsymbol {P H} ^ {\mathrm{T}} \left[ \boldsymbol {H P H} ^ {\mathrm{T}} + \boldsymbol {R} \right] ^ {- 1}\tag{3.8.8}
$$

为了将上式转化为 $\Gamma H^{T}$ 的表达式, 将 $X_{k}$ 写成

$$
\boldsymbol {X} _ {k} = \hat {\boldsymbol {X}} _ {k / k - 1} + \tilde {\boldsymbol {X}} _ {k / k - 1}
$$

注意到 $\hat{\pmb{X}}_{k / k - 1}$ 与 $\hat{\pmb{X}}_{k / k - 1}$ 正交，于是有

$$
\boldsymbol {\Gamma} = E \left[ \boldsymbol {X} _ {k} \boldsymbol {X} _ {k} ^ {\mathrm{T}} \right] = E \left[ \left(\hat {\boldsymbol {X}} _ {k / k - 1} + \tilde {\boldsymbol {X}} _ {k / k - 1}\right) \left(\hat {\boldsymbol {X}} _ {k / k - 1} + \tilde {\boldsymbol {X}} _ {k / k - 1}\right) ^ {\mathrm{T}} \right] = \boldsymbol {F} + \boldsymbol {P}\tag{3.8.9}
$$

式中

$$
\boldsymbol {F} = E \left[ \hat {\boldsymbol {X}} _ {k / k - 1} \hat {\boldsymbol {X}} _ {k / k - 1} ^ {\mathrm{T}} \right]
$$

因为 $\{\pmb{X}_k\}$ 为平稳序列，所以 $\pmb{F}$ 与 $k$ 无关。将 $P = \Gamma - F$ 代入式(3.8.8)，得

$$
\begin{array}{r l} \mathbf {K} & = (\mathbf {\Gamma} - \mathbf {F}) \mathbf {H} ^ {\mathrm{T}} [ \mathbf {H} (\mathbf {\Gamma} - \mathbf {F}) \mathbf {H} ^ {\mathrm{T}} + \mathbf {R} ] ^ {- 1} = \\ & (\mathbf {\Gamma} \mathbf {H} ^ {\mathrm{T}} - \mathbf {F} \mathbf {H} ^ {\mathrm{T}}) [ \mathbf {H} \mathbf {\Gamma} \mathbf {H} ^ {\mathrm{T}} + \mathbf {R} - \mathbf {H} \mathbf {F} \mathbf {H} ^ {\mathrm{T}} ] ^ {- 1} \end{array}
$$

将式 $(3.8.3)$ 代入上式,得

$$
\boldsymbol {K} = (\boldsymbol {\Gamma} \boldsymbol {H} ^ {\mathrm{T}} - \boldsymbol {F} \boldsymbol {H} ^ {\mathrm{T}}) (\boldsymbol {C} _ {0} - \boldsymbol {H} \boldsymbol {F} \boldsymbol {H} ^ {\mathrm{T}}) ^ {- 1}\tag{3.8.10}
$$

式中 $C_{0}$ 与 $TH^{T}$ 在根据 $\{Z_{i}\}$ 估计出 $\{C_{i}\}$ 之后都是已知阵，剩下的未知阵只有 F。如果注意到 F 是 $X_{k}$ 的预测值的均方阵，则可根据 $X_{k}$ 的预测递推方程

$$
\hat {\boldsymbol {X}} _ {k + 1 / k} = \boldsymbol {\Phi} \hat {\boldsymbol {X}} _ {k} = \boldsymbol {\Phi} \left[ \hat {\boldsymbol {X}} _ {k / k - 1} + \boldsymbol {K} \left(\boldsymbol {H} \tilde {\boldsymbol {X}} _ {k / k - 1} + \boldsymbol {V} _ {k}\right) \right]
$$

确定出 F。

$$
\begin{array}{r l} & {\pmb {F} = E [ \hat {\pmb {X}} _ {k + 1 / k} \hat {\pmb {X}} _ {k + 1 / k} ^ {\mathrm{T}} ] =} \\ & {\qquad \pmb {\Phi} E [ \hat {\pmb {X}} _ {k / k - 1} + \pmb {K} (\pmb {H} \tilde {\pmb {X}} _ {k / k - 1} + \pmb {V} _ {k}) ] [ \hat {\pmb {X}} _ {k / k - 1} + \pmb {K} (\pmb {H} \tilde {\pmb {X}} _ {k / k - 1} + \pmb {V} _ {k}) ] ^ {\mathrm{T}} \pmb {\Phi} ^ {\mathrm{T}} =} \\ & {\qquad \pmb {\Phi} [ \pmb {F} + \pmb {K} (\pmb {H P H} ^ {\mathrm{T}} + \pmb {R}) \pmb {K} ^ {\mathrm{T}} ] \pmb {\Phi} ^ {\mathrm{T}}} \end{array}
$$

将式(3.8.10)代入上式,又因 $\Gamma,F,C_{0}$ 都是对称阵,则

$$
\boldsymbol {F} = \boldsymbol {\Phi} \left[ \boldsymbol {F} + \left(\boldsymbol {\Gamma} \boldsymbol {H} ^ {\mathrm{T}} - \boldsymbol {F} \boldsymbol {H} ^ {\mathrm{T}}\right) \left(\boldsymbol {C} _ {0} - \boldsymbol {H} \boldsymbol {F} \boldsymbol {H} ^ {\mathrm{T}}\right) ^ {- 1} \left(\boldsymbol {\Gamma} \boldsymbol {H} ^ {\mathrm{T}} - \boldsymbol {F} \boldsymbol {H} ^ {\mathrm{T}}\right) ^ {\mathrm{T}} \right] \boldsymbol {\Phi} ^ {\mathrm{T}}\tag{3.8.11}
$$

这是一个关于 F 的非线性矩阵方程，当 $\{C_{i}\}$ 已知时， $TH^{T}$ 是已知阵，采用近似解法可得到 F 的近似值。求得 F 后，根据式（3.8.10）即可确定出 K。最后剩下的问题是如何根据 $\{Z_{i}\}$ 估计出 $\{C_{i}\}$ 。

3. 由 $\{\mathbf{Z}_i\}$ 估计 $\{\mathbf{C}_i\}$

设已获得量测值 $Z_{1}, Z_{2}, \cdots, Z_{k}$ ，假设平稳序列 $\{Z_{i}\}$ 具有各态历经性，则 $\{Z_{i}\}$ 的自相关函数 $C_{i}$ 的估计 $\hat{C}_{i}^{k}$ （下标 i 表示时间间隔，上标 k 表示估计所依据的量测数据的个数）是

$$
\begin{array}{r l} \hat {\mathbf {C}} _ {i} ^ {k} & = \frac {1}{k} \sum_ {l = i + 1} ^ {k} \mathbf {Z} _ {l} \mathbf {Z} _ {l - i} ^ {\mathrm{T}} = \frac {1}{k} \mathbf {Z} _ {k} \mathbf {Z} _ {k - i} ^ {\mathrm{T}} + \frac {1}{k} \sum_ {l = i + 1} ^ {k - 1} \mathbf {Z} _ {l} \mathbf {Z} _ {l - i} ^ {\mathrm{T}} = \\ & \frac {1}{k} \mathbf {Z} _ {k} \mathbf {Z} _ {k - i} ^ {\mathrm{T}} + \left[ \frac {1}{k - 1} - \frac {1}{k (k - 1)} \right] \sum_ {l = i + 1} ^ {k - 1} \mathbf {Z} _ {l} \mathbf {Z} _ {l - i} ^ {\mathrm{T}} = \\ & \frac {1}{k - 1} \sum_ {l = i + 1} ^ {k - 1} \mathbf {Z} _ {l} \mathbf {Z} _ {l - i} ^ {\mathrm{T}} + \frac {1}{k} \left(\mathbf {Z} _ {k} \mathbf {Z} _ {k - i} ^ {\mathrm{T}} - \frac {1}{k - 1} \sum_ {l = i + 1} ^ {k - 1} \mathbf {Z} _ {l} \mathbf {Z} _ {l - i} ^ {\mathrm{T}}\right) = \\ & \hat {\mathbf {C}} _ {i} ^ {k - 1} + \frac {1}{k} (\mathbf {Z} _ {k} \mathbf {Z} _ {k - i} ^ {\mathrm{T}} - \hat {\mathbf {C}} _ {i} ^ {k - 1}) \quad i = 0, 1, 2, \dots , n \end{array}\tag{3.8.12}
$$

式(3.8.12)是 $\hat{C}_{i}^{k}$ 的递推公式。若已给定 $\hat{C}_{i}^{2i}$ 的值，则由 $\hat{C}_{i}^{2i}$ 及 $Z_{2i+1}Z_{i+1}^{T}$ 可得到 $\hat{C}_{i}^{2i+1}$ ，再由 $\hat{C}_{i}^{2i+1}$ 及 $Z_{2i+2}Z_{i+2}^{T}$ 可得 $\hat{C}_{i}^{2i+2}$ ，最后由 $\hat{C}_{i}^{k-1}$ 及 $Z_{k}Z_{k-i}^{T}$ 可得 $\hat{C}_{i}^{k}(i=0,1,2,\cdots,n)$ ，这样就解决了 $\{C_{i}\}$ 的估计问题。

4. 输出相关法自适应滤波方程

综合式(3.8.10)、式(3.8.7)、式(3.8.11)及式(3.8.12)得完全可控和完全可观测定常系统式(3.8.1)和式(3.8.2)的稳态输出相关自适应滤波方程

$$
\begin{array}{r l} & {\hat {X} _ {k} = \Phi \hat {X} _ {k / k - 1} + \hat {K} ^ {k} [ Z _ {k} - H \Phi \hat {X} _ {k - 1} ]} \\ & {\hat {K} ^ {k} = [ \hat {\Gamma} ^ {k} H ^ {\mathrm{T}} - \hat {F} ^ {k} H ^ {\mathrm{T}} ] [ \hat {C} _ {0} ^ {k} - H \hat {F} ^ {k} H ^ {\mathrm{T}} ] ^ {- 1}} \end{array}
$$

$$
A = \left[ \begin{array}{c} H \Phi \\ H \Phi^ {2} \\ \vdots \\ H \Phi^ {n} \end{array} \right]
$$

$$
\hat {\boldsymbol {\Gamma}} ^ {k} \boldsymbol {H} ^ {\mathrm{T}} = (\boldsymbol {A} ^ {\mathrm{T}} \boldsymbol {A}) ^ {- 1} \boldsymbol {A} ^ {\mathrm{T}} \left[ \begin{array}{c} \hat {\boldsymbol {C}} _ {1} ^ {k} \\ \hat {\boldsymbol {C}} _ {2} ^ {k} \\ \vdots \\ \hat {\boldsymbol {C}} _ {n} ^ {k} \end{array} \right]
$$

$$
\hat {\boldsymbol {F}} ^ {k} = \boldsymbol {\Phi} \left[ \hat {\boldsymbol {F}} ^ {k} + \left(\hat {\boldsymbol {\Gamma}} ^ {k} - \hat {\boldsymbol {F}} ^ {k}\right) \boldsymbol {H} ^ {\mathrm{T}} \left(\hat {\boldsymbol {C}} _ {0} ^ {k} - \boldsymbol {H} \hat {\boldsymbol {F}} ^ {k} \boldsymbol {H} ^ {\mathrm{T}}\right) ^ {- 1} \boldsymbol {H} \left(\hat {\boldsymbol {\Gamma}} ^ {k} - \hat {\boldsymbol {F}} ^ {k}\right) ^ {\mathrm{T}} \right] \boldsymbol {\Phi} ^ {\mathrm{T}}
$$

$$
\hat {\mathbf {C}} _ {i} ^ {k} = \hat {\mathbf {C}} _ {i} ^ {k - 1} + \frac {1}{k} (\mathbf {Z} _ {k} \mathbf {Z} _ {k - i} ^ {\mathrm{T}} - \hat {\mathbf {C}} _ {i} ^ {k - 1})
$$

式中

$$
\hat {\mathbf {X}} _ {0} = E [ \mathbf {X} _ {0} ] \quad i = 0, 1, 2, \dots , n
$$

$\hat{K}^{k},\hat{\Gamma}^{k},\hat{F}^{k}$ 和 $\hat{C}_{i}^{k}$ 的上标 k 表示估计所依据的量测数据的个数。

## 3.9 次优滤波

在实际应用中,系统状态向量的维数可能很大,噪声也很多。这不但使测定各噪声统计特性的工作加重,而且更使滤波计算量增大。这样,机上设备采用卡尔曼滤波时,由于机载计算机计算容量有限,就不得不考虑如何在滤波误差增加不多的情况下,适当简化滤波方法,以减少计算量。这种简化的滤波方法称为次优滤波方法,相应的滤波器称为次优滤波器。滤波简化的方法很多,但却没有一个统一的简化途径可遵循,而都是根据具体情况寻求各种简化方法。通常还要借助误差分析这个工具作各种数值模拟来确定方案,最后再通过实物试验检验其效果。

次优滤波的方法很多。常见的一种是用减少状态变量数目的方法来构造次优滤波器，这种滤波器常称为降阶滤波器。降阶的方法有很多。最直观的方法是把在系统中起不了多大作用的状态去掉后，再设计滤波器。当然滤波实际结果因为没有考虑这类状态的影响而不再是最优的。但因为这类状态对其他状态的影响很小，所以即使未考虑，滤波性能也下降不多。这种方法首先要求设计者对系统实际的物理过程了解得很彻底，并且还须要采用误差分析方法作数值计算，这样才能正确地确定出哪些状态可以忽略，并对可能达到的估计均方误差有所估计。这种方法在惯导系统中是经常使用的。例如，惯导系统初始对准中方位陀螺漂移对系统对准的姿态角和方位角都有影响。但是初始对准的时间短，方位陀螺的漂移在短时间内引起的影响并不明显。如果初始对准采用卡尔曼滤波方法进行，即对系统的有关状态（包括姿态和方位误差角）进行滤波估计，则在系统模型中常不考虑方位陀螺漂移这个状态变量，以便降低滤波器的阶数。

实际上,需要估计的状态常常是系统模型中所有状态的一部分,能否把系统模型的阶数降低到需要估计的状态数,而把删去的那些状态的影响在滤波方程中给以适当考虑,使估计误差尽可能少增大一些呢？答案是，对某些系统可行。下面介绍按照这种思路构成的降阶滤波器。

## 3.9.1 考虑了删去状态影响的降阶滤波方法

1. 降阶滤波方程的推导

以离散系统为例,设系统和量测方程为

$$
\left[ \begin{array}{c} \boldsymbol {X} _ {k + 1} ^ {1} \\ \boldsymbol {X} _ {k + 1} ^ {2} \end{array} \right] = \left[ \begin{array}{c c} \boldsymbol {\Phi} _ {k + 1, k} ^ {1 1} & \boldsymbol {\Phi} _ {k + 1, k} ^ {1 2} \\ \boldsymbol {0} & \boldsymbol {\Phi} _ {k + 1, k} ^ {2 2} \end{array} \right] \left[ \begin{array}{c} \boldsymbol {X} _ {k} ^ {1} \\ \boldsymbol {X} _ {k} ^ {2} \end{array} \right] + \left[ \begin{array}{c c} \boldsymbol {0} & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {\Gamma} _ {k} ^ {2} \end{array} \right] \left[ \begin{array}{c} \boldsymbol {0} \\ \boldsymbol {W} _ {k} ^ {2} \end{array} \right]\tag{3.9.1}
$$

$$
\mathbf {Z} _ {k} = \left[ \begin{array}{l l} \mathbf {H} _ {k} ^ {1} & \mathbf {0} \end{array} \right] \left[ \begin{array}{l} \mathbf {X} _ {k} ^ {1} \\ \mathbf {X} _ {k} ^ {2} \end{array} \right] + \mathbf {V} _ {k}\tag{3.9.2}
$$

其中 $\{\pmb{W}_k^2\}$ 和 $\{\pmb{V}_k\}$ 为互不相关的零均值白噪声序列，它们都与 $X_0$ 不相关，方差阵分别为 $Q_k^2$ 和 $R_{k}$ 。这种系统的 $X_{k}^{2}$ 相当于 $X_{k}^{1}$ 子系统的有色噪声。现在要求用相当于 $X_{k}^{1}$ 维数 $(n_1)$ 的降阶滤波器来估计 $X_{k}^{1}$ ，在滤波方程中考虑 $X_{k}^{2}$ 的影响，并要求估计的均方误差尽量小。

先不考虑 $X_{k}^{2}$ 对 $X_{k+1}^{1}$ 的影响, 根据式(2.2.4b) 得

$$
\hat {\boldsymbol {X}} _ {k + 1} ^ {1} = \boldsymbol {\Phi} _ {k + 1, k} ^ {1 1} \hat {\boldsymbol {X}} _ {k} ^ {1} + \boldsymbol {K} _ {k + 1} \left(\boldsymbol {Z} _ {k + 1} - \boldsymbol {H} _ {k + 1} ^ {1} \boldsymbol {\Phi} _ {k + 1, k} ^ {1 1} \hat {\boldsymbol {X}} _ {k} ^ {1}\right)\tag{3.9.3}
$$

式中的增益阵尚未确定。在确定 $K_{k+1}$ 时考虑 $X_{k}^{2}$ 的影响，则令

$$
\widetilde {\boldsymbol {X}} _ {k + 1} ^ {1} = \boldsymbol {X} _ {k + 1} ^ {1} - \hat {\boldsymbol {X}} _ {k + 1} ^ {1}
$$

式中 $\widetilde{X}_{k + 1}^{1}$ 为 $X_{k + 1}^{1}$ 的估计误差。根据式(3.9.1）～式(3.9.3）可得

$$
\widetilde {\boldsymbol {X}} _ {k + 1} ^ {1} = \left(\boldsymbol {I} - \boldsymbol {K} _ {k + 1} \boldsymbol {H} _ {k + 1} ^ {1}\right) \left(\boldsymbol {\Phi} _ {k + 1, k} ^ {1 1} \widetilde {\boldsymbol {X}} _ {k} ^ {1} + \boldsymbol {\Phi} _ {k + 1, k} ^ {1 2} \boldsymbol {X} ^ {2}\right) - \boldsymbol {K} _ {k + 1} \boldsymbol {V} _ {k + 1}\tag{3.9.4}
$$

上式说明估计误差与 $X_{k}^{2}$ 有关。为了求出 $\widetilde{X}_{k+1}^{1}$ 的均方阵，先写出扩增 $X_{k+1}^{2}$ 后的 $\widetilde{X}_{k+1}^{1}$ 方程

$$
\left[ \begin{array}{c} \widetilde {\mathbf {X}} _ {k + 1} ^ {1} \\ \mathbf {X} _ {k + 1} ^ {2} \end{array} \right] = \left[ \begin{array}{c c} \mathbf {I} _ {f} \boldsymbol {\Phi} _ {k + 1, k} ^ {1 1} & \mathbf {I} _ {f} \boldsymbol {\Phi} _ {k + 1, k} ^ {1 2} \\ \mathbf {0} & \boldsymbol {\Phi} _ {k + 1, k} ^ {2 2} \end{array} \right] \left[ \begin{array}{c} \widetilde {\mathbf {X}} _ {k} ^ {1} \\ \mathbf {X} _ {k} ^ {2} \end{array} \right] + \left[ \begin{array}{c} \mathbf {K} _ {k + 1} \mathbf {V} _ {k + 1} \\ \boldsymbol {\Gamma} _ {k} ^ {2} \mathbf {W} _ {k} ^ {2} \end{array} \right]\tag{3.9.5}
$$

式中

$$
\boldsymbol {I} _ {f} = \boldsymbol {I} - \boldsymbol {K} _ {k + 1} \boldsymbol {H} _ {k + 1} ^ {1}
$$

令式 $(3.9.5)$ 的均方阵为

$$
E \left[ \begin{array}{l l} \widetilde {\boldsymbol {X}} _ {k + 1} ^ {1} \widetilde {\boldsymbol {X}} _ {k + 1} ^ {1 \mathrm{T}} & \widetilde {\boldsymbol {X}} _ {k + 1} ^ {1} \boldsymbol {X} _ {k + 1} ^ {2 \mathrm{T}} \\ \boldsymbol {X} _ {k + 1} ^ {2} \widetilde {\boldsymbol {X}} _ {k + 1} ^ {1 \mathrm{T}} & \boldsymbol {X} _ {k + 1} ^ {2} \boldsymbol {X} _ {k + 1} ^ {2 \mathrm{T}} \end{array} \right] = \left[ \begin{array}{l l} \boldsymbol {P} _ {k + 1} ^ {1} & \boldsymbol {C} _ {k + 1} ^ {\mathrm{T}} \\ \boldsymbol {C} _ {k + 1} & \boldsymbol {A} _ {k + 1} \end{array} \right]
$$

则有

$$
\begin{array}{r l} \left[ \begin{array}{c c} \boldsymbol {P} _ {k + 1} ^ {1} & \boldsymbol {C} _ {k + 1} ^ {\mathrm{T}} \\ \boldsymbol {C} _ {k + 1} & \boldsymbol {A} _ {k + 1} \end{array} \right] = & \left[ \begin{array}{c c} \boldsymbol {I} _ {f} \boldsymbol {\Phi} _ {k + 1, k} ^ {1 1} & \boldsymbol {I} _ {f} \boldsymbol {\Phi} _ {k + 1, k} ^ {1 2} \\ \boldsymbol {0} & \boldsymbol {\Phi} _ {k + 1, k} ^ {2 2} \end{array} \right] \left[ \begin{array}{c c} \boldsymbol {P} _ {k} ^ {1} & \boldsymbol {C} _ {k} ^ {\mathrm{T}} \\ \boldsymbol {C} _ {k} & \boldsymbol {A} _ {k} \end{array} \right] \left[ \begin{array}{c c} \boldsymbol {\Phi} _ {k + 1, k} ^ {1 1 \mathrm{T}}, \boldsymbol {I} _ {f} ^ {\mathrm{T}} & \boldsymbol {0} \\ \boldsymbol {\Phi} _ {k + 1, k} ^ {1 2 \mathrm{T}}, \boldsymbol {I} _ {f} ^ {\mathrm{T}} & \boldsymbol {\Phi} _ {k + 1, k} ^ {2 2 \mathrm{T}} \end{array} \right] + \\ & \left[ \begin{array}{c c} \boldsymbol {K} _ {k + 1} \boldsymbol {R} _ {k + 1} \boldsymbol {K} _ {k + 1} ^ {\mathrm{T}} & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {\Gamma} _ {k} ^ {2} Q _ {k} ^ {2} \boldsymbol {\Gamma} _ {k} ^ {2 \mathrm{T}} \end{array} \right] \end{array}\tag{3.9.6}
$$

从上式得

$$
\begin{array}{r l} \boldsymbol {P} _ {k + 1} ^ {1} = & \boldsymbol {I} _ {f} \boldsymbol {\Phi} _ {k + 1, k} ^ {1 1} \boldsymbol {P} _ {k} ^ {1} \boldsymbol {\Phi} _ {k + 1, k} ^ {1 1 \mathrm{T}} \boldsymbol {I} _ {f} ^ {\mathrm{T}} + \boldsymbol {I} _ {f} \boldsymbol {\Phi} _ {k + 1, k} ^ {1 2} \boldsymbol {C} _ {k} \boldsymbol {\Phi} _ {k + 1, k} ^ {1 1 \mathrm{T}} \boldsymbol {I} _ {f} ^ {\mathrm{T}} + \\ & \boldsymbol {I} _ {f} \boldsymbol {\Phi} _ {k + 1, k} ^ {1 1} \boldsymbol {C} _ {k} ^ {\mathrm{T}} \boldsymbol {\Phi} _ {k + 1, k} ^ {1 2 \mathrm{T}} \boldsymbol {I} _ {f} ^ {\mathrm{T}} + \boldsymbol {I} _ {f} \boldsymbol {\Phi} _ {k + 1, k} ^ {1 2} \boldsymbol {A} _ {k} \boldsymbol {\Phi} _ {k + 1, k} ^ {1 2 \mathrm{T}} \boldsymbol {I} _ {f} ^ {\mathrm{T}} + \end{array}
$$

$$
\boldsymbol {K} _ {k + 1} \boldsymbol {R} _ {k + 1} \boldsymbol {K} _ {k + 1} ^ {\mathrm{T}}\tag{3.9.7}
$$

$$
\boldsymbol {C} _ {k + 1} ^ {\mathrm{T}} = \boldsymbol {I} _ {f} \boldsymbol {\Phi} _ {k + 1, k} ^ {1 1} \boldsymbol {C} _ {k} ^ {\mathrm{T}} \boldsymbol {\Phi} _ {k + 1, k} ^ {2 2 \mathrm{T}} + \boldsymbol {I} _ {f} \boldsymbol {\Phi} _ {k + 1, k} ^ {1 2} \boldsymbol {A} _ {k} \boldsymbol {\Phi} _ {k + 1, k} ^ {2 2 \mathrm{T}}\tag{3.9.8}
$$

$$
\boldsymbol {A} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} ^ {2 2} \boldsymbol {A} _ {k} \boldsymbol {\Phi} _ {k + 1, k} ^ {2 2 \mathrm{T}} + \boldsymbol {\Gamma} _ {k} ^ {2} \boldsymbol {Q} _ {k} ^ {2} \boldsymbol {\Gamma} _ {k} ^ {2 \mathrm{T}}\tag{3.9.9}
$$

$P_{k}^{1}$ 就是 $X_{k}^{1}$ 的实际估计均方误差阵，式(3.9.7)就是 $P_{k}^{1}$ 的递推方程。因为 $X_{k}^{2}$ 通过 $\Phi_{k+1,k}^{12}$ 与

$X_{k+1}^{1}$ 有关，所以 $P_{k+1}^{1}$ 除与 $P_{k}^{1}$ 有关外，还与均方阵 $C_{k}$ 及 $A_{k}$ 有关。为了将式(3.9.7)写成与一般的估计均方误差方程式(2.2.4e)有类似的形式，令

$$
\begin{array}{r l} \boldsymbol {\Sigma} _ {k + 1 / k} & = \boldsymbol {\Phi} _ {k + 1, k} ^ {1 1} \boldsymbol {P} _ {k} ^ {1} \boldsymbol {\Phi} _ {k + 1, k} ^ {1 1 \mathrm{T}} + \boldsymbol {\Phi} _ {k + 1, k} ^ {1 2} \boldsymbol {C} _ {k} \boldsymbol {\Phi} _ {k + 1, k} ^ {1 1 \mathrm{T}} + \\ & \quad \boldsymbol {\Phi} _ {k + 1, k} ^ {1 1 \mathrm{T}} \boldsymbol {C} _ {k} ^ {\mathrm{T}} \boldsymbol {\Phi} _ {k + 1, k} ^ {1 2 \mathrm{T}} + \boldsymbol {\Phi} _ {k + 1, k} ^ {1 2} \boldsymbol {A} _ {k} \boldsymbol {\Phi} _ {k + 1, k} ^ {1 2 \mathrm{T}} \end{array}\tag{3.9.10}
$$

则式 $(3.9.7)$ 可写成

$$
\boldsymbol {P} _ {k + 1} ^ {1} = \left(\boldsymbol {I} - \boldsymbol {K} _ {k + 1} \boldsymbol {H} _ {k + 1} ^ {\mathrm{T}}\right) \boldsymbol {\Sigma} _ {k + 1 / k} \left(\boldsymbol {I} - \boldsymbol {K} _ {k + 1} \boldsymbol {H} _ {k + 1} ^ {1}\right) ^ {\mathrm{T}} + \boldsymbol {K} _ {k + 1} \boldsymbol {R} _ {k + 1} \boldsymbol {K} _ {k + 1} ^ {\mathrm{T}}\tag{3.9.11}
$$

与2.2.2小节中对式(2.2.4b)推导中使 $P_{k}$ 达到最小而确定出 $\mathbf{K}_k$ 的思路一样，可从式(3.9.11）中求出使 $P_{k + 1}^{1}$ 达到最小的 $K_{k + 1}^{0}$

$$
\boldsymbol {K} _ {k + 1} ^ {0} = \boldsymbol {\Sigma} _ {k + 1 / k} \boldsymbol {H} _ {k + 1} ^ {\mathrm{T}} (\boldsymbol {H} _ {k + 1} \boldsymbol {\Sigma} _ {k + 1 / k} \boldsymbol {H} _ {k + 1} ^ {\mathrm{T}} + \boldsymbol {R} _ {k + 1}) ^ {- 1}\tag{3.9.12}
$$

将 $K_{k + 1}^{0}$ 代入式(3.9.11)得

$$
\begin{array}{r l} & {\pmb {P} _ {k + 1} ^ {1} = [ \pmb {I} - \pmb {\Sigma} _ {k + 1 / k} \pmb {H} _ {k + 1} ^ {\mathrm{T}} (\pmb {H} _ {k + 1} \pmb {\Sigma} _ {k + 1 / k} \pmb {H} _ {k + 1} ^ {\mathrm{T}} + \pmb {R} _ {k + 1}) ^ {- 1} \pmb {H} _ {k + 1} ] \pmb {\Sigma} _ {k + 1 / k} =} \\ & {\qquad (\pmb {I} - \pmb {K} _ {k + 1} ^ {0} \pmb {H} _ {k + 1}) \pmb {\Sigma} _ {k + 1 / k}} \end{array}\tag{3.9.13}
$$

式(3.9.8)～式(3.9.10)、式(3.9.12)、式(3.9.13)及式(3.9.3)构成了次优滤波递推计算公式，但应注意，式(3.9.8)中的 $I_{f}$ 应改写为

$$
\boldsymbol {I} _ {f} = \boldsymbol {I} - \boldsymbol {K} _ {k + 1} ^ {0} \boldsymbol {H} _ {k + 1} ^ {1}\tag{3.9.14}
$$

次优滤波递推计算框图如图 3.9.1 所示。

![](images/1926f09da7428311b95892b152c349339a749c349d57eb1f36d799c64f4145d7.jpg)  
图 3.9.1 次优滤波递推计算框图

由于 $K_{k+1}^{0}$ 考虑了 $X_{k}^{2}$ 的影响,所以按式(3.9.3)以 $K_{k+1}^{0}$ 为最佳增益确定出的 $\hat{X}_{k+1}^{1}$ 已考虑了 $X_{k}^{2}$ 的影响。

2. $\hat{X}_{k + 1}^{1}$ 是否使估计的均方误差最小

从上面推导 $K_{k}^{0}$ 的过程可知, 根据 $K_{1}^{0}$ 确定的 $P_{1}^{1}$ 比任何其他 $K_{1}$ 值确定的 $P_{1}^{1}$ 为小, 即对任何 $K_{1}$ , 有

$$
\boldsymbol {P} _ {1} ^ {1} (\boldsymbol {K} _ {1} ^ {0}) \leqslant \boldsymbol {P} _ {1} ^ {1} (\boldsymbol {K} _ {1})
$$

在 $P_{1}^{1}(K_{1}^{0})$ 基础上计算得到的 $K_{2}^{0}$ 比其他任何 $K_{2}$ 值为优，即对任何 $K_{2}$ ，有

$$
\boldsymbol {P} _ {2} ^ {1} (\boldsymbol {K} _ {1} ^ {0}, \boldsymbol {K} _ {2} ^ {0}) \leqslant \boldsymbol {P} _ {2} ^ {1} (\boldsymbol {K} _ {0} ^ {1}, \boldsymbol {K} _ {2})
$$

从而对任何 $\pmb{K}_k$ ，有

$$
\boldsymbol {P} _ {k} ^ {1} \left(\boldsymbol {K} _ {1} ^ {0}, \boldsymbol {K} _ {2} ^ {0}, \dots , \boldsymbol {K} _ {k - 1} ^ {0}, \boldsymbol {K} _ {k} ^ {0}\right) \leqslant \boldsymbol {P} _ {k} ^ {1} \left(\boldsymbol {K} _ {1} ^ {0}, \boldsymbol {K} _ {2} ^ {0}, \dots , \boldsymbol {K} _ {k - 1} ^ {0}, \boldsymbol {K} _ {k}\right)
$$

但根据上式不能得到如下关系：

$$
\boldsymbol {P} _ {k} ^ {1} \left(\boldsymbol {K} _ {1} ^ {0}, \boldsymbol {K} _ {2} ^ {0}, \dots , \boldsymbol {K} _ {k} ^ {0}\right) \leqslant \boldsymbol {P} _ {k} ^ {1} \left(\boldsymbol {K} _ {1}, \boldsymbol {K} _ {2}, \dots , \boldsymbol {K} _ {k}\right) \quad \forall \boldsymbol {K} _ {1}, \boldsymbol {K} _ {2}, \dots , \boldsymbol {K} _ {k}
$$

也就是说 $K_{k}^{0}$ 是根据一步最优的原则选择的。但是整个 $\{K_{1}^{0}, K_{2}^{0}, \cdots, K_{k}^{0}\}$ 的选取并不能保证整个估计过程是最优的，因此 $P_{k}^{1}$ 也不是最小的。这一点可以从下面的推导中得到概略的说明。

先从一般的估计均方误差来看，即相当于系统式(3.9.1)中没有 $X_{k}^{2}$ 。这时，根据式(2.2.4c)

$$
\boldsymbol {K} _ {1} ^ {0} = \boldsymbol {P} _ {1 / 0} \boldsymbol {H} _ {1} ^ {\mathrm{T}} (\boldsymbol {H} _ {1} \boldsymbol {P} _ {1 / 0} \boldsymbol {H} _ {1} ^ {\mathrm{T}} + \boldsymbol {R} _ {1}) ^ {- 1}
$$

此处，对 $K_{1}$ 也标以上标0，表示它使 $P_{1}$ 最小，即

$$
\boldsymbol {P} _ {1} (\boldsymbol {K} _ {1} ^ {0}) \leqslant \boldsymbol {P} _ {1} (\boldsymbol {K} _ {1})
$$

式中 $K_{1}$ 为任何值。下述分析中凡不标上标 0 的 K 值表示任何值。

根据式(2.2.4d)

$$
\boldsymbol {P} _ {2 / 1} = \boldsymbol {\Phi} _ {2, 1} \boldsymbol {P} _ {1} \boldsymbol {\Phi} _ {2, 1} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {1} \boldsymbol {Q} _ {1} \boldsymbol {\Gamma} _ {1} ^ {\mathrm{T}}
$$

可以看出

$$
\pmb {P} _ {2 / 1} (\pmb {K} _ {1} ^ {0}) \leqslant \pmb {P} _ {2 / 1} (\pmb {K} _ {1})
$$

如果选择 $K_{2}=K_{2}^{0}$ ，则根据式(2.2.4e $^{\prime\prime}$ )

$$
\boldsymbol {P} _ {2} ^ {- 1} = \boldsymbol {P} _ {2 / 1} ^ {- 1} + \boldsymbol {H} _ {2} ^ {\mathrm{T}} \boldsymbol {R} _ {2} ^ {- 1} \boldsymbol {H} _ {2}
$$

从线性代数的矩阵知识知道,如果 A 和 B 都是 $n \times n$ 正定阵,并有 $A \geqslant B$ , 则 $0 < A^{-1} \leqslant B^{-1}$ 。因此, 上式可得

$$
\mathbf {P} _ {2} (\mathbf {K} _ {1} ^ {0}, \mathbf {K} _ {2} ^ {0}) \leqslant \mathbf {P} _ {2} (\mathbf {K} _ {1}, \mathbf {K} _ {2})
$$

从而可推知

$$
\boldsymbol {P} _ {k} (\boldsymbol {K} _ {1} ^ {0}, \boldsymbol {K} _ {2} ^ {0}, \dots , \boldsymbol {K} _ {k} ^ {0}) \leqslant \boldsymbol {P} _ {k} (\boldsymbol {K} _ {1}, \boldsymbol {K} _ {2}, \dots , \boldsymbol {K} _ {k})
$$

但如果系统为式 $(3.9.1)$ ，则虽然有

$$
\boldsymbol {P} _ {1} ^ {1} (\boldsymbol {K} _ {1} ^ {0}) \leqslant \boldsymbol {P} _ {1} ^ {1} (\boldsymbol {K})
$$

但不能保证

$$
\mathbf {C} _ {1} (\mathbf {K} _ {1} ^ {0}) \leqslant \mathbf {C} _ {1} (\mathbf {K} _ {1})
$$

从而也就不能保证

$$
\pmb {\Sigma} _ {2 / 1} (\pmb {K} _ {1} ^ {0}) \leqslant \pmb {\Sigma} _ {2 / 1} (\pmb {K} _ {1})
$$

虽然

$$
\boldsymbol {P} _ {2} ^ {1} \left(\boldsymbol {K} _ {1} ^ {0}, \boldsymbol {K} _ {2} ^ {0}\right) \leqslant \boldsymbol {P} _ {2} ^ {1} \left(\boldsymbol {K} _ {1} ^ {0}, \boldsymbol {K} _ {2}\right)
$$

但不能保证

$$
\boldsymbol {P} _ {2} ^ {1} (\boldsymbol {K} _ {1} ^ {0}, \boldsymbol {K} _ {2} ^ {0}) \leqslant \boldsymbol {P} _ {2} ^ {1} (\boldsymbol {K} _ {1}, \boldsymbol {K} _ {2})
$$

因此也不能保证

$$
\boldsymbol {P} _ {k} ^ {1} \left(\boldsymbol {K} _ {1} ^ {0}, \boldsymbol {K} _ {2} ^ {0}, \dots , \boldsymbol {K} _ {k} ^ {0}\right) \leqslant \boldsymbol {P} _ {k} ^ {1} \left(\boldsymbol {K} _ {1}, \boldsymbol {K} _ {2}, \dots , \boldsymbol {K} _ {k}\right)
$$

究其原因， $K_{k}^{0}$ 是根据 $P_{k}^{1}$ 一步最优选择的，而均方阵 $C_{k}$ 却不是最优的，所以不能保证 $P_{k+1}^{1}$ 一定是最优的，换句话说，在选择 $K_{k+1}^{0}$ 时，虽然考虑了 $X_{k}^{2}$ 的影响，但 $X_{k}^{2}$ 的影响毕竟没有完全消除，不能保证 $P_{k+1}^{1}$ 为最小。因此，这种滤波方法仍然只是一种次优滤波方法。但由于它在一定程度上考虑了 $X_{k}^{2}$ 的影响，所以它是次优滤波方法中估计的均方误差较小的一种。

至于连续系统，则有同样的结果，这里就不再讨论了。

例3-5 设系统和量测为

$$
\left[ \begin{array}{l} X _ {k + 1} ^ {1} \\ X _ {k + 1} ^ {2} \end{array} \right] = \left[ \begin{array}{c c} 1 & 1 \\ 0 & 0. 5 \end{array} \right] \left[ \begin{array}{l} X _ {k} ^ {1} \\ X _ {k} ^ {2} \end{array} \right] + \left[ \begin{array}{l} 0 \\ W _ {k} ^ {2} \end{array} \right]
$$

$$
Z _ {k} = \left[ \begin{array}{l l} 1 & 0 \end{array} \right] \left[ \begin{array}{l} X _ {k} ^ {1} \\ X _ {k} ^ {2} \end{array} \right] + V _ {k}
$$

$\{W_k^2\}$ 和 $\{V_{k}\}$ 为零均值、互不相关的白噪声序列，并与 $X_0^1, X_0^2$ 也互不相关， $W_{k}^{2}$ 和 $V_{k}$ 的方差分别为

$$
Q _ {k} ^ {2} = 1, R _ {k} = 1
$$

并有

$$
E [ X _ {0} ^ {1} ] = E [ X _ {0} ^ {2} ] = 0
$$

$$
E \{\left[ \begin{array}{l} X _ {0} ^ {1} \\ X _ {0} ^ {2} \end{array} \right] \left[ \begin{array}{l l} X _ {0} ^ {1} & X _ {0} ^ {2} \end{array} \right] \} = \left[ \begin{array}{l l} 1 0 & 0 \\ 0 & 1 0 \end{array} \right]
$$

$X_{k}^{1}$ 是需要估计的状态。试分别用全阶卡尔曼滤波（即最优滤波）、考虑了删去状态影响的降阶（一阶）滤波和简单降阶（一阶）滤波来估计 $X_{k}^{1}$ ，并比较估计的均方误差。

## 解

(1) 全阶卡尔曼滤波。

根据式(2.2.4)

$$
\begin{array}{l} \hat {X} _ {k + 1} ^ {1} = \hat {X} _ {k} ^ {1} + \hat {X} _ {k} ^ {2} + K _ {k + 1} ^ {1} (Z _ {k + 1} - \hat {X} _ {k} ^ {1} - \hat {X} _ {k} ^ {2}) \\ \hat {X} _ {k + 1} ^ {2} = \hat {X} _ {k} ^ {2} + K _ {k + 1} ^ {2} (Z _ {k + 1} - \hat {X} _ {k} ^ {1} - \hat {X} _ {k} ^ {2}) \\ P _ {k / k - 1} ^ {1 1} = P _ {k - 1} ^ {1 1} + 2 P _ {k - 1} ^ {1 2} + P _ {k - 1} ^ {2 2} \\ P _ {k / k - 1} ^ {1 2} = 0. 5 (P _ {k - 1} ^ {1 2} + P _ {k - 1} ^ {2 2}) \\ P _ {k / k - 1} ^ {2 2} = 0. 2 5 P _ {k - 1} ^ {2 2} + 1 \\ K _ {k} ^ {1} = P _ {k} ^ {1 1} = \frac {P _ {k / k - 1} ^ {1 1}}{P _ {k / k - 1} ^ {1 1} + 1} \\ K _ {k} ^ {2} = P _ {k} ^ {1 2} = \frac {P _ {k / k - 1} ^ {1 2}}{P _ {k / k - 1} ^ {1 1} + 1} \\ P _ {k} ^ {2 2} = P _ {k / k - 1} ^ {2 2} - P _ {k} ^ {1 2} P _ {k / k - 1} ^ {1 2} \end{array}
$$

则从 $P_{0}^{11} = 10, P_{0}^{12} = 0, P_{0}^{22} = 10$ 可以算得

<table><tr><td>k</td><td>1</td><td>2</td><td>3</td><td>4</td><td>5</td><td>6</td><td>7</td></tr><tr><td> $P_{k}^{11}$ </td><td>0.952</td><td>0.815</td><td>0.736</td><td>0.700</td><td>0.693</td><td>0.693</td><td>0.693</td></tr></table>

(2) 考虑了删去状态 $X_{k}^{2}$ 影响的降阶滤波。

根据式 $(3.9.3)$ 、式 $(3.9.8)$ ～式 $(3.9.10)$ 、式 $(3.9.12)$ 和式 $(3.9.13)$ ，结合给出的参数，有

$$
\begin{array}{r l} & \Sigma_ {k + 1 / k} = P _ {k} + 2 C _ {k} + A _ {k} \\ & K _ {k + 1} ^ {0} = \Sigma_ {k + 1 / k} (\Sigma_ {k + 1 / k} + 1) ^ {- 1} \\ & P _ {k + 1} = (1 - K _ {k + 1} ^ {0}) \Sigma_ {k + 1 / k} \\ & A _ {k + 1} = 0. 2 5 A _ {k} + 1 \\ & C _ {k + 1} = 0. 5 (1 - K _ {k + 1} ^ {0}) (C _ {k} + A _ {k}) \\ & \hat {X} _ {k + 1} ^ {1} = \hat {X} _ {k} ^ {1} + K _ {k + 1} ^ {0} (Z _ {k + 1} - \hat {X} _ {k} ^ {1}) \end{array}
$$

可以看出，计算的方程比全阶滤波方程简单。从 $P_0 = 10, C_0 = 0, A_0 = 10$ ，可计算得

<table><tr><td>k</td><td>1</td><td>2</td><td>3</td><td>4</td><td>5</td><td>6</td><td>7</td></tr><tr><td> $P_{k}$ </td><td>0.952</td><td>0.831</td><td>0.799</td><td>0.735</td><td>0.719</td><td>0.715</td><td>0.714</td></tr></table>

(3) 简单降阶滤波。

系统模型仅考虑 $X_{k}^{1}$ ，且不考虑 $X_{k}^{2}$ 的影响，则根据式(2.2.4)，有

$$
\begin{array}{l} \hat {X} _ {k} ^ {1} = \hat {X} _ {k - 1} ^ {1} + K _ {k} ^ {1} (Z _ {k} - \hat {X} _ {k - 1} ^ {1}) \\ K _ {k} ^ {1} = P _ {k / k - 1} ^ {1} (P _ {k / k - 1} ^ {1} + 1) ^ {- 1} \\ P _ {k / k - 1} ^ {1} = P _ {k - 1} ^ {1} \\ P _ {k} ^ {1} = (1 - K _ {k} ^ {1}) P _ {k / k - 1} ^ {1} = \left(1 - \frac {P _ {k / k - 1} ^ {1}}{P _ {k / k - 1} ^ {1} + 1}\right) P _ {k / k - 1} ^ {1} = K _ {k} ^ {1} \end{array}
$$

式中上标 1 表示用 $X_{k}^{1}$ 作模型状态所得到的各种值。

从 $P_{0}^{1}=10$ 可算得 $P_{k}(K_{k})$ 的值，如下表第二行所列。根据卡尔曼滤波的误差分析（将在3.10节中介绍）可解得实际的均方误差（估计值相对真实状态的实际偏差） $P_{k}^{r}$ ，如下表第三行所列。

<table><tr><td>k</td><td>1</td><td>2</td><td>3</td><td>4</td><td>5</td><td>6</td><td>7</td></tr><tr><td> $P_{k}(K_{k})$ </td><td>0.909</td><td>0.476</td><td>0.323</td><td>0.244</td><td>0.196</td><td>0.164</td><td>0.141</td></tr><tr><td> $P_{k}^{r}$ </td><td>0.938</td><td>1.248</td><td>2.215</td><td>3.200</td><td>4.157</td><td>5.150</td><td>7.810</td></tr></table>

从上述三个表所列数据可看出，滤波(2)的估计均方误差与滤波(1)相近，而滤波(3)忽略了 $X_{k}^{2}$ ，相当于系统忽略了噪声，使得计算的均方误差虽然很快减小，但实际的均方误差却越来越大，即滤波器产生发散现象。

考虑了删去状态影响的降阶滤波方法比一般的次伏滤波方法好。但是，这种方法并不能保证在任意删去多少状态的条件下都能使估计均方误差接近最优。它的估计均方误差与系统结构、删去状态的多少及其在系统中的作用等因素有关。这种方法用作降阶滤波的不足之处是计算量比卡尔曼滤波基本方程的计算量减少不多。但这可采取与其他次优方法（例如下面将介绍的计算参数分段常值等）相结合的办法来解决。在采用其他降阶滤波方法时，也可以将这种方法作为可能降阶的程度以及明确哪些状态影响较小的参考。

## 3.9.2 常增益次优滤波

滤波计算中有关估计均方误差的计算都是为计算增益阵作准备的,因此可以说,滤波的主要计算量在于计算增益阵。如果增益阵取为常值阵,则这些计算都可以免去,计算量就会大大减少。如果对定常系统采用常增益阵,则常取稳态增益作为常值增益。对离散系统,常值增益为

$$
\boldsymbol {K} = \overline {{\boldsymbol {P}}} \boldsymbol {H} ^ {\mathrm{T}} (\boldsymbol {H} \overline {{\boldsymbol {P}}} \boldsymbol {H} ^ {\mathrm{T}} + \boldsymbol {R}) ^ {- 1}\tag{3.9.15}
$$

式中 $\bar{P}$ 为稳态一步预测均方误差阵, 按下式计算:

$$
\bar {P} = \Phi \bar {P} \Phi^ {\mathrm{T}} - \Phi \bar {P} H ^ {\mathrm{T}} [ H \bar {P} H ^ {\mathrm{T}} + R ] ^ {- 1} H \bar {P} \Phi^ {\mathrm{T}} + I Q \Gamma^ {\mathrm{T}}\tag{3.9.16}
$$

在达到稳态以前,这种滤波方法是次优的。当然,它达到稳态的时间比最优滤波达到稳态的时间长。这种滤波方法的估计均方误差(在滤波器中不需要计算)为

$$
\boldsymbol {P} _ {k / k - 1} = \boldsymbol {\Phi} \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} ^ {\mathrm{T}} + \boldsymbol {\Gamma} \boldsymbol {Q} \boldsymbol {\Gamma} ^ {\mathrm{T}}\tag{3.9.17}
$$

$$
\boldsymbol {P} _ {k} = (\boldsymbol {I} - \boldsymbol {K H}) \boldsymbol {P} _ {k / k - 1} (\boldsymbol {I} - \boldsymbol {K H}) ^ {\mathrm{T}} + \boldsymbol {K R K} ^ {\mathrm{T}}\tag{3.9.18}
$$

下面举一个简单的例子来说明常增益滤波方法。

例3-6 设系统和量测为

$$
\begin{array}{l} X _ {k + 1} = X _ {k} + W _ {k} \\ Z _ {k} = X _ {k} + V _ {k} \end{array}
$$

状态和量测都是标量。 $\{W_{k}\}$ 和 $\{V_{k}\}$ 是互不相关的零均值白噪声序列，且与 $X_{0}$ 不相关，方差都为 1，即

$$
Q _ {k} = 1, R _ {k} = 1
$$

且

$$
E [ X _ {0} ] = 0, \mathrm{Var} [ X _ {0} ] = 1 0
$$

按变增益设计最优滤波器和常增益设计次优滤波器，并比较两者的滤波效果。

解

按最优滤波有

$$
\begin{array}{l} {K _ {k} = P _ {k / k - 1} (P _ {k / k - 1} + 1) ^ {- 1}} \\ {P _ {k} = (1 - K _ {k}) P _ {k / k - 1} = K _ {k}} \\ {P _ {k + 1 / k} = (1 - K _ {k}) P _ {k / k - 1} + 1 = K _ {k} + 1} \end{array}
$$

取 $\hat{X}_0 = 0, P_0 = 10$ ，得

<table><tr><td>k</td><td>1</td><td>2</td><td>3</td><td>4</td><td>5</td><td>6</td><td>7</td></tr><tr><td> $P_{k/k-1}$ </td><td>11</td><td>1.917</td><td>1.657</td><td>1.624</td><td>1.619</td><td>1.618</td><td>1.618</td></tr><tr><td> $P_k = K_k$ </td><td>0.917</td><td>0.657</td><td>0.624</td><td>0.619</td><td>0.618</td><td>0.618</td><td>0.618</td></tr></table>

稳态一步预测均方误差阵按式 $(3.9.16)$ 确定

$$
\bar {P} ^ {2} - \bar {P} - 1 = 0
$$

$$
\bar {P} = \frac {1}{2} (1 \pm \sqrt {5})
$$

取正值得

$$
\bar {P} = 1. 6 1 8
$$

根据式 $(3.9.15)$ ，得

$$
K = \bar {P} (\bar {P} + 1) ^ {- 1} = 0. 6 1 8
$$

由式 $(2.2.4e^{\prime})$ 得稳态估计均方误差为

$$
P = (1 - K) \bar {P} = 0. 6 1 8
$$

如果按常增益滤波方法,则根据式(3.9.17)和式(3.9.18)有

$$
\begin{array}{l} P _ {k / k - 1} = P _ {k - 1} + 1 \\ P _ {k} = (1 - K) ^ {2} P _ {k / k - 1} + K ^ {2} \end{array}
$$

取 $K = 0.618$ ，则可算得

<table><tr><td>k</td><td>1</td><td>2</td><td>3</td><td>4</td><td>5</td><td>6</td><td>7</td><td>8</td></tr><tr><td> $P_{k/k-1}$ </td><td>11</td><td>2.987</td><td>1.818</td><td>1.647</td><td>1.622</td><td>1.619</td><td>1.618</td><td>1.618</td></tr><tr><td> $P_k$ </td><td>1.987</td><td>0.818</td><td>0.647</td><td>0.622</td><td>0.619</td><td>0.618</td><td>0.618</td><td>0.618</td></tr></table>

图3.9.2示出了两种滤波方法中 $P_{k}$ 的变化情况。从图中可看出，在常增益滤波中， $P_{k}$ 达

到稳定的时间比最佳增益滤波所需时间长，而最终两种滤波的 $P_{k}$ 会重合。这说明常增益滤波将逐渐趋于最优滤波，但在达到稳定之前，常增益滤波是次优的。

常增益滤波是一种最简单的次优滤波方法。如果它的估计均方误差不能满足要求，则还可采用分段常增益的方法，以改善滤波效果。例如上例中，可取 $k = 1$ 时刻的最优增益值0.917作为第一个常增益值 $K_{1}$ ，而其余时刻的常增益值 $K_{k}(k = 2,3,\dots)$ 均取

![](images/a031fb28e8e956baf650d533ced24e42ae73b2e799bf8f891661511d9432d274.jpg)  
图3.9.2 常增益滤波和最优增益滤波中 $P_{k}$ 的变化

为 0.618, 这样的常增益滤波的效果与最优滤波的效果几乎是相同的。

分段常值增益的滤波效果必须通过计算估计均方误差考核。

## 3.9.3 系统解耦次优滤波

系统中各个状态之间的联系有的紧密,有的疏松。如果能把状态向量分成几组状态向量,

每组状态向量由彼此关联紧密的状态所组成，而且，各组状态向量都有与之有关的量测值，则系统和量测可以分成阶数较低的几组，并以此来设计相应的滤波器，那么滤波计算量必然会有效的降低。当然，各组滤波器因没有考虑其他子系统的影响，所以是一种次优滤波方法。例如，系统和量测为

$$
\begin{array}{r l} \left[ \begin{array}{c} \boldsymbol {X} _ {k + 1} ^ {1} \\ \boldsymbol {X} _ {k + 1} ^ {2} \end{array} \right] & = \left[ \begin{array}{c c} \boldsymbol {\Phi} _ {k + 1, k} ^ {1 1} & \boldsymbol {\Phi} _ {k + 1, k} ^ {1 2} \\ \boldsymbol {\Phi} _ {k + 1, k} ^ {2 1} & \boldsymbol {\Phi} _ {k + 1, k} ^ {2 2} \end{array} \right] \left[ \begin{array}{c} \boldsymbol {X} _ {k} ^ {1} \\ \boldsymbol {X} _ {k} ^ {2} \end{array} \right] + \\ & \left[ \begin{array}{c c} \boldsymbol {\Gamma} _ {k} ^ {1} & \mathbf {0} \\ \mathbf {0} & \boldsymbol {\Gamma} _ {k} ^ {2} \end{array} \right] \left[ \begin{array}{c} \boldsymbol {W} _ {k} ^ {1} \\ \boldsymbol {W} _ {k} ^ {2} \end{array} \right] \\ & \left[ \begin{array}{c} \boldsymbol {Z} _ {k} ^ {1} \\ \boldsymbol {Z} _ {k} ^ {2} \end{array} \right] = \left[ \begin{array}{c c} \boldsymbol {H} _ {k} ^ {1} & \mathbf {0} \\ \mathbf {0} & \boldsymbol {H} _ {k} ^ {2} \end{array} \right] \left[ \begin{array}{c} \boldsymbol {X} _ {k} ^ {1} \\ \boldsymbol {X} _ {k} ^ {2} \end{array} \right] + \left[ \begin{array}{c} \boldsymbol {V} _ {k} ^ {1} \\ \boldsymbol {V} _ {k} ^ {2} \end{array} \right] \end{array}
$$

![](images/c0068ec109ee618612c2dfa808e5a93066608d39eaf4af182ad08c3ffb3502bf.jpg)

其中 $\{W_{k}^{1}\},\{W_{k}^{2}\},\{V_{k}^{1}\},\{V_{k}^{2}\}$ 为互不相关（或相关性很小）的白噪声序列。 $X_{k}^{1}$ 和 $X_{k}^{2}$ 各为 $n_{1}$ 图3.9.3 可以解耦的系统和量测结构图

和 $n_{2}$ 维状态向量，转移矩阵 $\Phi_{k+1,k}^{12}$ 和 $\Phi_{k+1,k}^{21}$ 是 $X_{k+1}^{1}$ 和 $X_{k+1}^{2}$ 的耦合矩阵。图 3.9.3 为系统和量测的结构图。

如果 $\Phi_{k + 1,k}^{12}$ 和 $\Phi_{k + 1,k}^{21}$ 中的每个元都非常小，即 $X_{k + 1}^{1}$ 和 $X_{k + 1}^{2}$ 之间的耦合非常弱，则可以忽略这种耦合，将系统解耦成两个阶数分别为 $n_1$ 和 $n_2$ 的子系统。 $X_{k}^{1}$ 和 $X_{k}^{2}$ 的估计分别由两个子系统和量测组成的独立滤波器中求取。这种处理方法在惯导系统中也是经常使用的。例如惯导系统的东西向回路（指东西向加速度计所在的回路）、南北向回路和方位回路，这三个回路是互相耦合的，但它们之间的耦合松紧程度不一样，南北向回路和方位回路耦合紧密，东西向回路与另两个回路耦合较松，尤其在某些工作情况下，例如运载体的速度不大或在静基座工作条件下(如地面初始对准或定期标定等),这种耦合关系就更松。根据对系统工作的要求程度,有时就把整个系统解耦成东西回路子系统、南北回路子系统及方位回路子系统来考虑。

系统能否解耦成若干个子系统，以及如何解耦，这不但与系统的结构以及对估计的性能要求有关，而且还要求对系统物理过程有充分的了解，并通过各种方案的误差数值计算（主要是估计均方误差的计算），才能初步确定下来。这种方案的估计效果最终还得通过实践才能得到证实。

## 3.10 卡尔曼滤波误差分析

设计卡尔曼滤波器时,首先必须已知系统和量测的模型以及系统噪声和量测噪声的统计特性。但由于对系统认识不全面,或者为了简化计算而将模型简化,往往使确定的滤波模型与实际系统不符,加之精确的噪声先验统计量很难获得。所以,滤波除了可能会产生发散现象外,必然会产生滤波误差。因此,在确定系统和量测模型时,一般都要根据滤波要求进行误差分析,主要是对估计的均方误差阵作误差分析。对次优卡尔曼滤波,误差分析更是常用的分析手段。

## 3.10.1 离散系统的卡尔曼滤波误差分析一般方法

## 1. 方差误差分析

设真实系统和量测为

$$
\boldsymbol {X} _ {k} ^ {r} = \boldsymbol {\Phi} _ {k, k - 1} ^ {r} \boldsymbol {X} _ {k - 1} ^ {r} + \boldsymbol {\Gamma} _ {k - 1} ^ {r} \boldsymbol {W} _ {k - 1} ^ {r}\tag{3.10.1}
$$

$$
\mathbf {Z} _ {k} = \mathbf {H} _ {k} ^ {r} \mathbf {X} _ {k} ^ {r} + \mathbf {V} _ {k} ^ {r}\tag{3.10.2}
$$

式中，上标 r 表示真实量； $\{W_{k}^{r}\}$ 和 $\{V_{k}^{r}\}$ 都是零均值白噪声序列；方差分别为 $Q_{k}^{r}$ 和 $R_{k}^{r}; W_{k}^{r}, V_{k}^{r}$ ， $X_{0}^{r}$ 三者互不相关。在设计卡尔曼滤波器时，系统模型和量测模型取作

$$
\boldsymbol {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {X} _ {k - 1} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {W} _ {k - 1}\tag{3.10.3}
$$

$$
\mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k}\tag{3.10.4}
$$

式中， $\{W_{k}\}$ 和 $\{V_{k}\}$ 都是零均值的白噪声序列；方差分别为 $Q_{k}$ 和 $R_{k}; W_{k}, V_{k}, X_{0}$ 三者互不相关。
根据式(2.2.4) 滤波方程为

$$
\hat {\mathbf {X}} _ {k} = \hat {\mathbf {X}} _ {k / k - 1} + \mathbf {K} _ {k} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k / k - 1})\tag{3.10.5}
$$

$$
\hat {\boldsymbol {X}} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \hat {\boldsymbol {X}} _ {k - 1}\tag{3.10.6}
$$

$$
\boldsymbol {K} _ {k} = \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1}\tag{3.10.7}
$$

$$
\boldsymbol {P} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {Q} _ {k - 1} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}}\tag{3.10.8}
$$

$$
\boldsymbol {P} _ {k} = (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1} (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) ^ {\mathrm{T}} + \boldsymbol {K} _ {k} \boldsymbol {R} _ {k} \boldsymbol {K} _ {k} ^ {\mathrm{T}} = (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1}\tag{3.10.9}
$$

应注意，滤波计算得的 $\hat{X}_{k}$ 是对模型状态 $X_{k}$ 的最优估计，而不是对真实状态 $X_{k}^{r}$ 的最优估计； $P_{k}$ 也仅是针对 $X_{k}$ 的估计均方误差阵，而不是实际的估计均方误差。误差分析就是要找出实际的估计均方误差，以了解实际的滤波效果。为此，令

$$
\begin{array}{r l} & {\widetilde {X} _ {k} ^ {P} \stackrel {\mathrm{def}} {=} X _ {k} ^ {r} - \hat {X} _ {k}} \\ & {\widetilde {X} _ {k / k - 1} ^ {P} \stackrel {\mathrm{def}} {=} X _ {k} ^ {r} - \hat {X} _ {k / k - 1}} \\ & {P _ {k} ^ {P} \stackrel {\mathrm{def}} {=} E [ \widetilde {X} _ {k} ^ {P} \widetilde {X} _ {k} ^ {P T} ]} \end{array}
$$

$$
\begin{array}{r l} & {\pmb {P} _ {k / k - 1} ^ {P} \stackrel {\mathrm{def}} {=} E [ \widetilde {\pmb {X}} _ {k / k - 1} ^ {P} \widetilde {\pmb {X}} _ {k / k - 1} ^ {P \mathrm{T}} ]} \\ & {\Delta \pmb {\Phi} _ {k, k - 1} \stackrel {\mathrm{def}} {=} \pmb {\Phi} _ {k, k - 1} ^ {r} - \pmb {\Phi} _ {k, k - 1}} \\ & {\qquad \Delta \pmb {H} _ {k} \stackrel {\mathrm{def}} {=} \pmb {H} _ {k} ^ {r} - \pmb {H} _ {k}} \end{array}
$$

根据式 $(3.10.2)$ 、式 $(3.10.5)$ 、式 $(3.10.6)$ 有

$$
\begin{array}{r l} & {\widetilde {X} _ {k} ^ {P} = X _ {k} ^ {r} - \hat {X} _ {k / k - 1} - K _ {k} (H _ {k} ^ {r} X _ {k} ^ {r} + V _ {k} ^ {r} - H _ {k} \hat {X} _ {k / k - 1}) =} \\ & {\quad (I - K _ {k} H _ {k}) \widetilde {X} _ {k / k - 1} ^ {P} - K _ {k} \Delta H _ {k} X _ {k} ^ {r} - K _ {k} V _ {k} ^ {r}} \\ & {\widetilde {X} _ {k / k - 1} ^ {P} = \Phi_ {k, k - 1} ^ {r} X _ {k - 1} ^ {r} + \Gamma_ {k - 1} ^ {r} W _ {k - 1} ^ {r} - \Phi_ {k, k - 1} \hat {X} _ {k - 1} =} \\ & {\quad \Phi_ {k, k - 1} \widetilde {X} _ {k - 1} ^ {P} + \Delta \Phi_ {k, k - 1} X _ {k - 1} ^ {r} + \Gamma_ {k - 1} ^ {r} W _ {k - 1} ^ {r}} \end{array}\tag{3.10.10}
$$

(3.10.11)

将式 $(3.10.10)$ 、式 $(3.10.11)$ 两边取均方，得

$$
\begin{array}{r l} & {\pmb {P} _ {k} ^ {P} = (\pmb {I} - \pmb {K} _ {k} \pmb {H} _ {k}) \pmb {P} _ {k / k - 1} ^ {P} (\pmb {I} - \pmb {K} _ {k} \pmb {H} _ {k}) ^ {\mathrm{T}} + \pmb {K} _ {k} \Delta \pmb {H} _ {k} \pmb {A} _ {k} \Delta \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {K} _ {k} ^ {\mathrm{T}} +} \\ & {\qquad \pmb {K} _ {k} \pmb {R} _ {k} ^ {r} \pmb {K} _ {k} ^ {\mathrm{T}} - (\pmb {I} - \pmb {K} _ {k} \pmb {H} _ {k}) \pmb {C} _ {k / k - 1} ^ {\mathrm{T}} \Delta \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {K} _ {k} ^ {\mathrm{T}} -} \\ & {\qquad \pmb {K} _ {k} \Delta \pmb {H} _ {k} \pmb {C} _ {k / k - 1} (\pmb {I} - \pmb {K} _ {k} \pmb {H} _ {k}) ^ {\mathrm{T}}} \\ & {\pmb {P} _ {k / k - 1} ^ {P} = \pmb {\Phi} _ {k, k - 1} \pmb {P} _ {k - 1} ^ {P} \pmb {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \Delta \pmb {\Phi} _ {k, k - 1} \pmb {A} _ {k - 1} \Delta \pmb {\Phi} _ {k, k - 1} ^ {\mathrm{T}} +} \\ & {\qquad \Gamma_ {k - 1} ^ {r} Q _ {k - 1} ^ {r} \Gamma_ {k - 1} ^ {r \mathrm{T}} + \pmb {\Phi} _ {k, k - 1} \pmb {C} _ {k - 1} ^ {\mathrm{T}} \Delta \pmb {\Phi} _ {k, k - 1} ^ {\mathrm{T}} +} \\ & {\qquad \Delta \pmb {\Phi} _ {k, k - 1} \pmb {C} _ {k - 1} \pmb {\Phi} _ {k, k - 1} ^ {\mathrm{T}}} \end{array}\tag{3.10.12}
$$

(3.10.13)

式中

$$
\begin{array}{r l} & {\mathbf {A} _ {k} \stackrel {\mathrm{def}} {=} E [ \mathbf {X} _ {k} ^ {r} \mathbf {X} _ {k} ^ {r \mathrm{T}} ]} \\ & {\mathbf {C} _ {k} \stackrel {\mathrm{def}} {=} E [ \mathbf {X} _ {k} ^ {r} \widetilde {\mathbf {X}} _ {k} ^ {P \mathrm{T}} ]} \\ & {\mathbf {C} _ {k / k - 1} \stackrel {\mathrm{def}} {=} E [ \mathbf {X} _ {k} ^ {r} \widetilde {\mathbf {X}} _ {k / k - 1} ^ {P \mathrm{T}} ]} \end{array}
$$

根据式(3.10.1)、式(3.10.10)、式(3.10.11)可推得以上各阵的递推公式

$$
\boldsymbol {A} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} ^ {r} \boldsymbol {A} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {r T} + \boldsymbol {\Gamma} _ {k - 1} ^ {r} \boldsymbol {Q} _ {k - 1} ^ {r} \boldsymbol {\Gamma} _ {k - 1} ^ {r T}\tag{3.10.14}
$$

$$
\boldsymbol {C} _ {k} = \boldsymbol {C} _ {k / k - 1} (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) ^ {\mathrm{T}} - \boldsymbol {A} _ {k} \Delta \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {K} _ {k} ^ {\mathrm{T}}\tag{3.10.15}
$$

$$
\boldsymbol {C} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} ^ {r} \boldsymbol {C} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \boldsymbol {\Phi} _ {k, k - 1} ^ {r} \boldsymbol {A} _ {k - 1} \Delta \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k - 1} ^ {r} \boldsymbol {Q} _ {k - 1} ^ {r} \boldsymbol {\Gamma} _ {k - 1} ^ {r \mathrm{T}}\tag{3.10.16}
$$

以上各种均方阵的初始值分别为

$$
\begin{array}{r l} \boldsymbol {P} _ {0} ^ {P} & = E \big [ (\boldsymbol {X} _ {0} ^ {r} - \hat {\boldsymbol {X}} _ {0}) (\boldsymbol {X} _ {0} ^ {r} - \hat {\boldsymbol {X}} _ {0}) ^ {\mathrm{T}} \big ] \\ & \quad \boldsymbol {A} _ {0} = E \big [ \boldsymbol {X} _ {0} ^ {r} \boldsymbol {X} _ {0} ^ {r ^ {\mathrm{T}}} \big ] \\ & \quad \boldsymbol {C} _ {0} = E \big [ \boldsymbol {X} _ {0} ^ {r} (\boldsymbol {X} _ {0} ^ {r} - \hat {\boldsymbol {X}} _ {0}) ^ {\mathrm{T}} \big ] \end{array}
$$

如果取 $\hat{X}_0 = 0$ ，则 $P_0^P = A_0 = C_0$ 。

在确知真实系统的情况下，不论是模型的参数不准（即存在 $\Delta \Phi_{k,k - 1}$ 或 $\Delta H_{k}$ ），还是初始估计的均方误差不准（即 $P_0\neq P_0^P$ ），都可以根据以上方程先计算 $A_{k}$ ，再计算 $C_k$ 和 $C_{k + 1 / k}$ ，进而算出 $P_{k + 1 / k}^{P}$ ，最后算出 $P_{k + 1}^{P}$ ，从而得到在简化的系统模型条件下所作的滤波估计的实际均方误差阵，以便从均方误差的角度来检验滤波器的估计精度是否满足要求。这种利用 $P_{k}^{P}$ 来进行误差分析的方法常称为方差误差分析。

## 2. 灵敏度分析

除了直接检验 $P_{k}^{P}$ 阵中对角线元素值是否满足滤波估计的要求外, 还可以采用灵敏度这个量表征系统模型参数的变化对估计的均方误差的影响, 以衡量系统模型各参数值的合理性。即令

$$
\boldsymbol {S} _ {i, k} = \frac {\boldsymbol {P} _ {k} ^ {P} - \boldsymbol {P} _ {k}}{\Delta b _ {i}}\tag{3.10.17}
$$

式中 $\Delta b_{i}$ 是系统模型中某参数值与真实值的差异。例如 $\Delta \Phi_{k,k - 1},\Delta H_k$ 或 $\Delta Q_{k}(\Delta Q_{k} = Q_{k}^{r} - Q_{k})$ 、 $\Delta R_{k}(\Delta R_{k} = R_{k}^{r} - R_{k})$ 中的某个元素值； $S_{i,k}$ 称为大尺度灵敏度； $P_{k}^{P}$ 和 $P_{k}$ 是系统模型中某参数值与真实值 $b_{i}$ 有差异时的实际估计均方误差阵和计算的估计均方误差阵。

在不少情况下,对真实系统的了解并不十分透彻,或缺乏精确的噪声方差值,因而不可能求出 $P_{k}^{P}$ 或 $S_{i,k}$ 。在此情况下,可采用“小尺度灵敏度”( $r_{i,k}$ )这个量作分析,即令

$$
\boldsymbol {r} _ {i, k} = \frac {\partial \boldsymbol {P} _ {k} ^ {P}}{\partial b _ {i} ^ {r}} \Bigg | _ {\boldsymbol {b} ^ {r} = \boldsymbol {b}}\tag{3.10.18}
$$

为了得到求解 $r_{i,k}$ 的方程，将式(3.10.12）两边对 $b_i^r$ 取偏导数，其中右边第二项取偏导数后为

$$
\left. \frac {\partial}{\partial b _ {i} ^ {r}} \left[ \mathbf {K} _ {k} \Delta \mathbf {H} _ {k} \mathbf {A} _ {k} \Delta \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {K} _ {k} ^ {\mathrm{T}} \right] \right| _ {\mathbf {b} ^ {r} = \mathbf {b}} = \frac {\partial}{\partial b _ {i} ^ {r}} \left[ \mathbf {K} _ {k} \left(\mathbf {H} _ {k} ^ {r} - \mathbf {H} _ {k}\right) \mathbf {A} _ {k} \left(\mathbf {H} _ {k} ^ {r} - \mathbf {H} _ {k}\right) ^ {\mathrm{T}} \mathbf {K} _ {k} ^ {\mathrm{T}} \right] \Bigg | _ {\mathbf {b} ^ {r} = \mathbf {b}}
$$

将其展开为四项，每项中的 $H_{k}^{r}$ 和 $\mathbf{A}_k$ 都是 $b_{i}^{r}$ 的函数，对 $b_{i}^{r}$ 求偏导数，并在 $\pmb{b}^r = \pmb{b}$ 的条件下，最后可得该项的偏导数为零。同样，对式(3.10.12)右边第四、五项中 $\Delta H_{k}$ 用 $(H_{k}^{r} - H_{k})$ 代替，对 $b_{i}^{r}$ 取偏导数，并考虑到

$$
\begin{array}{r l} \boldsymbol {C} _ {k / k - 1} \mid_ {\boldsymbol {b} ^ {r} = \boldsymbol {b}} & = E [ \boldsymbol {X} _ {k} \widetilde {\boldsymbol {X}} _ {k / k - 1} ^ {\mathrm{PT}} ] \mid_ {\boldsymbol {b} ^ {r} = \boldsymbol {b}} = \\ & E [ \boldsymbol {X} _ {k} ^ {r} \widetilde {\boldsymbol {X}} _ {k / k - 1} ^ {\mathrm{PT}} - \hat {\boldsymbol {X}} _ {k / k - 1} \widetilde {\boldsymbol {X}} _ {k / k - 1} ^ {\mathrm{T}} ] \mid_ {\boldsymbol {b} ^ {r} = \boldsymbol {b}} = \\ & E [ \boldsymbol {X} _ {k} \widetilde {\boldsymbol {X}} _ {k / k - 1} ^ {\mathrm{T}} - \hat {\boldsymbol {X}} _ {k / k - 1} \widetilde {\boldsymbol {X}} _ {k / k - 1} ^ {\mathrm{T}} ] = \\ & E [ \widetilde {\boldsymbol {X}} _ {k / k - 1} \widetilde {\boldsymbol {X}} _ {k / k - 1} ^ {\mathrm{T}} ] = \boldsymbol {P} _ {k / k - 1} \end{array}
$$

式中 $\tilde{X}_{k/k-1}$ 是一步预测 $\hat{X}_{k/k-1}$ 的预测误差，因为两者正交，所以 $E[\hat{X}_{k/k-1}\tilde{X}_{k/k-1}^{\mathrm{T}}]=0$ 。

同样可推得

$$
\boldsymbol {C} _ {k - 1} \mid_ {\boldsymbol {b} ^ {r} = \boldsymbol {b}} = \boldsymbol {P} _ {k - 1}
$$

因此，第四和第五项的偏导数分别为

$$
\begin{array}{l} \frac {\partial}{\partial b _ {i} ^ {r}} \left[ (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \boldsymbol {C} _ {k / k - 1} ^ {\mathrm{T}} \Delta \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {K} _ {k} ^ {\mathrm{T}} \right] | _ {\boldsymbol {b} ^ {r} = \boldsymbol {b}} = \\ (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1} ^ {\mathrm{T}} \frac {\partial \boldsymbol {H} _ {k} ^ {r}}{\partial b _ {i} ^ {r}} \Bigg | _ {\boldsymbol {b} ^ {r} = \boldsymbol {b}} \boldsymbol {K} _ {k} ^ {\mathrm{T}} \\ \frac {\partial}{\partial b _ {i} ^ {r}} \left[ \boldsymbol {K} _ {k} \Delta \boldsymbol {H} _ {k} \boldsymbol {C} _ {k / k - 1} (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) ^ {\mathrm{T}} \right] \Bigg | _ {\boldsymbol {b} ^ {r} = \boldsymbol {b}} = \\ \boldsymbol {K} _ {k} \frac {\partial \boldsymbol {H} _ {k} ^ {r}}{\partial b _ {i} ^ {r}} \Bigg | _ {\boldsymbol {b} ^ {r} = \boldsymbol {b}} \boldsymbol {P} _ {k / k - 1} (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) ^ {\mathrm{T}} \end{array}
$$

这样就得到求解小尺度灵敏度的方程

$$
\begin{array}{r l} \boldsymbol {r} _ {i, k} & = (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \boldsymbol {r} _ {i, k / k - 1} (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) ^ {\mathrm{T}} + \boldsymbol {K} _ {k} \frac {\partial \boldsymbol {R} _ {k} ^ {r}}{\partial b _ {i} ^ {r}} \Bigg | _ {\boldsymbol {b} ^ {r} = \boldsymbol {b}} \boldsymbol {K} _ {k} ^ {\mathrm{T}} - \\ & (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1} ^ {\mathrm{T}} \frac {\partial \boldsymbol {H} _ {k} ^ {r}}{\partial b _ {i} ^ {r}} \Bigg | _ {\boldsymbol {b} ^ {r} = \boldsymbol {b}} \boldsymbol {K} _ {k} ^ {\mathrm{T}} - \boldsymbol {K} _ {k} \frac {\partial \boldsymbol {H} _ {k} ^ {r}}{\partial b _ {i} ^ {r}} \Bigg | _ {\boldsymbol {b} ^ {r} = \boldsymbol {b}}. \\ & \boldsymbol {P} _ {k / k - 1} (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) ^ {\mathrm{T}} \end{array}
$$

式中

(3.10.19)

$$
\boldsymbol {r} _ {i, k / k - 1} = \frac {\partial \boldsymbol {P} _ {k / k - 1} ^ {P}}{\partial b _ {i} ^ {r}} \Bigg | _ {b ^ {r} = b}\tag{3.10.20}
$$