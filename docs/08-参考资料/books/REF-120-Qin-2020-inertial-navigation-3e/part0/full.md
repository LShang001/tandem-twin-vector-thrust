## 惯性导航 (第三版)

秦永元 编著

![](images/f29db6a60f3dba1fee7528fb175f38b8771e6ed4231f75f62233ae6764c58473.jpg)

普通高等教育“十五”国家级规划教材

# 惯性导航

(第三版)

秦永元 编著

科学出版社

北 京

## 内容简介

本书分三篇，全面介绍惯性导航基本理论. (1) 惯性仪表 包括转于陀螺力学基础 液浮积分陀螺、动力调谐陀螺、激光陀螺、摆式加速度计动力学分析及其误差补偿原理. (2) 陀螺稳定平台 包括单轴、三轴、双轴陀螺稳定平台动力学分析、回路设计及误差分析。 (3) 惯性导航系统 包括休拉词谱原理 平台式惯导系统的力学编排、误差分析及自主式初始对准 在惯性仪表增量输出条件下，捷联式惯导系统姿态更新四元数算法，多子样旋转矢量算法及补偿圆锥效应的优化算法，速度解算中补偿划桨效应及位置解算中补偿涡卷效应的优化算法 捷联惯导的自主式韧始对准及动基座传递对捷联惯性器件的余度技术 极区惯性导航，

本书为普通高等教育"十五"国家级规划教材，可作为惯性导航相关学科的本科生、研究生及专业厂、所、部队工程技术人员的教科书和参考书

圈书在版编目 ICIP)敛据

惯性导航/秦永元编著 .-3 版.一北京 科学出版社，2020.11

普通高等教育"十五"国家级规划教材

ISBN 978-7-03-<l66408-2

1. 惯… II. 秦… III. 惯性导航-高等学校-教材 N. TN96

中国版本图书馆 CIP 数据核字(2020) 198998

责任编辑 朱晓颖/责任校对 王瑞

责任印制 张伟/封面设计:迷底书裴

4t等钩，准熬出版

北京东黄城根北街16

邮政编码 100717

即，j/www.ocien呻幅

R"\~!t印刷哺依贵俗咱可耐印刷

科学出版社发行各地新华书店经销

2006 月第一版开本， 787XI092 1/16

2020 11 月第三版印张 .24 1/4

2020 11 月第十六次印刷字数 .602000

定价 :98.00

〈如有印装质量问题，我社负责调换〉

## 第三版前言

本书自 2014年出版第二版以来已使用七年，本次修订再版是在吸取了广大读者对本书的改进意见和建议、适应教学和工程应用需要的基础上进行的，具体内容如下。

(1)改写了 9.2.3 节四元数微分方程和 9.3.2 节旋转矢量微分方程.采用极限概念推导微分方程，使推演过程更加严密和自然，便于读者对抽象概念的理解和掌握。

(2) 增加了 8.5 节惯导定位等概率误差椭圆与散布圆。分析了飞机要求的定位精度与惯导精度下限间的定量关系，阐明了飞机相对惯导定位值的散布圆与惯导定位误差间的定量关系，为飞机空域管理提供理论依据和具体算法。

(3) 增加了 9.11. 节余度配置陀螺软故障检测。详细介绍了用奇偶向量构造 $\chi ^ { 2 }$ 分布统计量和定位软故障陀螺的方法，分析了软故障漏检和虚警间的关系。

(4) 增加了式(10.3.19)的详细推导过程。

(5) 增加了习题 3-33。旨在使读者深刻理解比力方程和加速度计输出。

(6) 7.2.3 节中的椭圆参数方程近似形式作了详细说明。

出版第三版旨在使本书更加严密和完整，为读者提供阅读和使用上的便利，使本书在教学和科研中发挥些许作用。

作 者

2020

## 第二版前言

本书自 2006 年出版以来已使用七年。在经过了几年的教学实践，昕取了教师和读者的意见之后，为适应工程应用需要，更好地为国民经济和国防建设服务，作者对本书进行修订，使之进一步完善.在保持教材主要内容、体系和叙述风格的基础上，在以下几方面进行了修订.

(1) 对传递对准中的姿态阵匹配作了改写。

（2）增写了下述内容：与惯性器件、陀螺稳定平台、惯性导航原理相对应的习题；罗德利格参数;抗摇摆粗对准 传递对准中的姿态四元数匹配;极区惯性导航。

(3) 对书中措辞不够严谨或表述不恰当的地方进行了修改。

(4) 更正了原书文字、插图及计算中的疏漏。

由于作者水平和时间有限，再版后还会存在不尽如人意之处，敬请读者批评指正。

作 者

2013

## 第一版前言

惯性导航是综合了机电、光学、数学、力学、控制 及计算机等学科的尖端技术，广泛应用于航空、航天、航海及重要车辆陆地导航中。由于惯性是所有质量体的基本属性，所以建立在惯性原理基础上的惯性导航系统无须任何外来信息，也不向外辐射任何信息，仅靠系统本身就能在全天候条件下、全球范围内和所有介质环境里自主地、隐蔽地进行三维定位和三维定向。这种同时具备自主性、隐蔽性和能获取运载体完备运动信息的独特优点是诸如卫星导航、无线电导航和天文导航等其他导航系统无法比拟的。尽管这些导航系统的某些导航性能远优于惯导系统，但惯导系统仍然是重要运载体不可缺少的核心导航设备。

本书为国家级"十五"重点立项教材，面向的读者是与惯性导航有关的导航、制导与控制，精密仪器及机械，检测技术与自动化装置等学科的本科生、研究生及相关专业厂、所、部队的工程技术人员。本书全面介绍了惯性仪表、惯性平台及惯性导航系统的设计理论，内容丰富，涉及面宽。全书注意了内容安排上的铺垫和连贯，阐述中采用物理概念直观解释和数学力学严格推导相结合的方法，引导读者顺利理解概念，掌握内容实质，力求提高可读性。同时，本书融入了作者的部分科研成果，对工程应用具有一定的参考价值。

全书共分三篇。第一篇(第 章)阐述了惯性仪表基本理论，包括陀螺力学基础、液浮陀螺、动力调谐陀螺、激光陀螺及摆式加速度计工作原理等。第二篇(第 4-6 章)系统介绍了陀螺稳定平台设计和分析理论，包括单轴稳定平台工作原理分析及系统回路设计理论、三轴陀螺稳定平台隔离干扰及跟踪指令工作原理分析、传递函数矩阵及动力学分析、平台误差分析、双轴平台设计分析。第三篇(第 7-9 章)介绍惯性导航系统，包括:地球上的定位，休拉调谐原理，平台式惯导的力学编排、误差分析及自主式初始对准，捷联式惯导姿态更新四元数算法和多子样旋转矢量算法及考虑圆锥效应补偿的优化算法，捷联惯导速度解算中考虑划桨效应补偿的优化算法及位置解算中考虑涡卷效应补偿的优化算法，捷联惯导的自主式初始对准和动基座传递对准，捷联惯性器件的余度技术。

本书各篇间既有一定的连贯性，又相对独立，读者可根据需要自行选用所需内容。

本书承蒙空军工程大学张宗麟教授审阅。张教授提出了许多宝贵意见，在此谨致深切谢意。

书中存在的缺点和错误，敬请读者批评指正。

作 者

2005年8月

## 目 录

第1章 绪论…… 1  
1.1 惯性技术的发展及现状 …… 1  
1.2 常用关系式 …… 4  

第一篇 惯性器件  

第2章 陀螺仪…… 9  
2.1 转子陀螺的力学原理 …… 9  
2.2 双自由度陀螺仪…… 12  
2.3 单自由度陀螺仪…… 18  
2.4 动力调谐陀螺仪…… 28  
2.5 激光陀螺…… 49  
2.6 光纤陀螺简介…… 62  

第3章 摆式加速度计 …… 64  
3.1 加速度计简介…… 64  
3.2 挠性摆式加速度计…… 66  

习题一 …… 76  

第二篇 陀螺稳定平台  

第4章 陀螺稳定平台工作原理 …… 78  
4.1 各类稳定平台简介…… 78  
4.2 力矩装置和平台台体的传递函数…… 88  
4.3 单轴陀螺稳定平台…… 93  

第5章 三轴陀螺稳定平台…… 111  
5.1 三轴平台简介 …… 111  
5.2 环架驱动信号的合理分配——方位坐标分解器和俯仰正割分解器 …… 112  
5.3 三轴平台工作原理分析 …… 118  
5.4 三轴平台的方位锁定原理 …… 123  
5.5 三轴平台的动力学方程和系统的传递函数阵 …… 124  
5.6 采用积分陀螺时三轴平台系统的输出轴耦合分析 …… 135  
5.7 三轴平台的误差分析 …… 137  

第6章 双轴陀螺稳定平台…… 154  
6.1 稳定瞄准线的双轴稳定平台 …… 154  
6.2 水平式双轴稳定平台 …… 156  

习题二 …… 167

## 第三篇惯性导航系统原理

第7章 惯导系统中的基本关系 …… 170
7.1 惯导系统的分类 …… 170
7.2 地球的描述 …… 171
7.3 休拉调谐 …… 181
7.4 惯导系统的基本方程——比力方程 …… 188
7.5 惯性高度通道的稳定性分析 …… 190
第8章 平台式惯导系统 …… 194
8.1 平台式惯导系统的力学编排 …… 194
8.2 平台式惯导系统的误差分析 …… 207
8.3 平台式惯导系统的初始对准 …… 222
8.4 扩展导航参数解算和飞机转弯方式 …… 237
8.5 惯导定位等概率误差椭圆与散布圆 …… 242
第9章 捷联式惯导系统 …… 251
9.1 概述 …… 251
9.2 姿态更新计算的四元数算法 …… 252
9.3 姿态更新计算的等效旋转矢量算法 …… 267
9.4 锥运动环境下旋转矢量算法的优化 …… 276
9.5 姿态更新的其他算法及各种算法的比较 …… 284
9.6 捷联惯导系统的速度算法 …… 296
9.7 捷联惯导系统的位置算法 …… 304
9.8 捷联惯导系统的误差模型 …… 317
9.9 捷联惯导系统抗干扰自对准 …… 322
9.10 捷联惯导系统的动基座传递对准 …… 338
9.11 捷联惯性器件的余度技术 …… 346
第10章 极区惯性导航 …… 358
10.1 极区平面惯性导航 …… 358
10.2 极区横向经纬度惯性导航 …… 362
10.3 极区格网惯性导航 …… 366
习题三 …… 372
参考文献 …… 376

## 第1章 绪 论

## 1. 1 惯性技术的发展及现状

导航是指将运动体从甲地导引到目的地乙地的技术，这门技术既古老又年轻 吉罗马人利用北极星和太阳作为方位基准，横渡地中海，来往于南欧和北非之间。郑和利用指南针率领庞大的船队七下西祥，开创了茫茫大海上的远航。在古代先民的导航过程中，利用的信息资源非常直观，采用的方法和原理十分简单，所以导航精度非常低。随着人类对自然现象本质的深入认识和科学技术的发展，导航信息新资源和新理论不断被发现。主哲伦使用六分仪的天文导航完成了环球航行。牛顿力学的创建使人们能正确认识惯性，根据惯性原理制造出在运动体内建立人工参考基准的惯性仪器，并逐步完善和发展成惯性导航系统，使人们能探索更深更远的壁间。

惯性导航是一门综合了机电、光学、数学、力学、控制及计算机等学科的尖端技术，是现代科学技术发展到一定阶段的产物。由于惯性是所有质量体的基本属性，所以建立在惯性原理基础上的惯性导航系统不需要任何外来信息，也不会向外辐射任何信息，仅靠惯性导航系统本身就能在全天候条件下，在全球范围内和任何介质环境里自主地、隐蔽地进行连续的三维定位和三维定向，这种同时具备自主性、隐蔽性和能获取运载体完备运动信息的独特优点是诸如无线电导航、卫星导航和天文导航等其他导航系统无法比拟的，尽管这些导航系统的某些导航性能可能远远优于惯性导航系统，但惯'性导航仍然是重要运载体不可缺少的核心导航设备。

作为一门高科技尖端技术，惯性技术是在先进科学理论和制造工艺支持条件下发展起来的。早在 1687 年，牛顿提出了力学三大定律，从本质上揭示了宏观力学的定量关系，为惯性导航奠定了理论基础。 1765 年俄国科学院院士欧拉出版了著作《刚体绕定点运动的理论))，首次利用解析分析的方法对定点转动刚体作了本质解释，创立了转子陀螺仪的力学基本理论。 1778 年法国科学家拉格朗日在《分析力学》一书中建立了在重力力矩作用 定点转动刚体的运动微分方程组1852 年法国科学家傅科根据欧拉和拉格朗日的刚体定点转动理论制造出了用于验证地球自转运动的测量装置，并在巳黎科学院做了实验演示，傅科把这一测量装置命名为 Gyros ope，在希腊文中为转动和观察的意思。虽然傅科只是定性地观察到了地球的自转而未能精确 出地球的自转角速度，但傅科的研究开创了人们对工程实用陀螺仪的研究和开发 转子陀螺仪的工程应用主要利用了陀螺的定轴性和进动性，而耍陀螺仪达到一定的精度，必须确保转子以恒定的角速度旋转，傅科采用人工拉动绳索驱动转子旋转是无法达到这一要求的。 19 世纪 80 年代陀螺技术有了重大突破，发明家特鲁和霍普金斯用电动机驱动陀螺转子，大大提高了陀螺转子的旋转速度和转速的稳定性，并制造出了陀螺罗经试验装置。 1908 年德国科学家安修茨设计了一种单转子摆式陀螺罗经，在该系统中，陀螺转子采用了三相交流电机驱动，陀螺组件用水银悬浮，支承中心偏离陀螺组件质心，使陀螺形成一个重力摆，从而制造出了世界上第一台摆式陀螺罗经，该系统依靠重力力矩自动找北。后人称颂这是陀螺技术中最巧妙的设计。陀螺罗经解决了当时舰船的远航和潜艇较长时间的潜航问题。 1910 年德国科学家休拉发现了陀螺罗经的元干扰条件，即当陀螺罗经的无阻尼振荡周期为 84.4 分钟时，陀螺罗经的指北精度不受外界加速度冲击的影响，这就是著名的体拉调谐原理. 1912 年安修茨对陀螺罗经原设计作了重大改进，用三个陀螺代替原设计中单个陀螺的结构，解决了原设计中舰船摇摆产生的摇摆误差。 1927 年安修茨将三陀螺罗经改进设计成双陀螺罗经，此即为安修茨 MK-1 型罗经，该罗经采用双转子液浮摆式结构，设计非常巧妙，成为两种典型的航海罗经之一，流传使用了半个多世纪，直至 20 世纪 50 年代出现了更先进的电磁控制罗经，才逐步被取代。另一类航海罗经是美国的斯派利罗经，该型罗经采用单转子钢丝悬挂式结构，其性能与安修茨罗经并驾齐驱，并列为舰船航海仪器的两大支柱，为航海事业做出了重大贡献。

休拉对惯性技术的发展起到了关键的理论指导作用。休拉在发现陀螺罗经无干扰条件的研究基础上，进一步发现元干扰条件具有普遍性，即休拉调谐原理不光适用于陀螺罗经，也同样适用于地垂线指示系统。任何陀螺装置、任何摆和机械仪器，只要系统具有 84.4 分钟的无阻尼振荡周期，运载体的加速度就不会影响系统的指示精度。 1923 年休拉发表论文《运载工具的加速度对于摆和陀螺仪的干扰 >>(Pitman G R, 1962;Schuler M, 1923) ，以垂线指示系统为例系统阐明了休拉摆原理，为惯性导航系统的设计奠定了理论基础。

第二次世界大战期间，德国的 V-2 火箭采用两个双自由度陀螺和一个陀螺积分加速度计构成惯性制导系统，这是惯性技术在导弹制导上的首次应用，但由于陀螺和加速度计精度很低，惯性系统设计又十分粗糙，根本实现不了休拉调谐要求，加上控制系统十分原始，制导精度极低，在轰炸伦敦的过程中，有 1/4 V-2 火箭提前掉人大海。

在确保陀螺仪转子自转角速度恒定的技术问题解决后，提高陀螺仪精度的主要途径是减小沿陀螺仪支承轴的干扰力矩，特别是摩擦力矩。 20 世纪 50 年代美国麻省理工学院德雷帕实验室采用液浮支承，成功研制了单自由度液浮陀螺，有效降低了支承引起的摩擦力矩，使陀螺漂移达到了惯性级要求(Draper C S,1960;Draper C S et al. 1956) ，并在 1949 年成功研制出了命名为"菲信"的惯性装置，这套装置安装在 B29 飞机上完成了从马萨诸塞州至新墨西哥州的 10小时长途全自动飞行。该系统虽然还不具备惯导系统的功能，但为惯导系统的研制打下了基础。1950 月美国北美航空公司奥托奈蒂克斯分公司为美国空军研制成功了第一套纯惯性导航系统 XN-l，并安装在C4 飞机上成功进行了试飞，为了适应航海应用，XN-1 经过适当改型，成N6 惯性导航系统。 1958 月美国海军鹦鹉螺号核潜艇进行了一次穿越北极冰盖的探险航行。鹦鹉螺号装备有一套 N6-A 惯性导航系统和一套 MK-19 平台罗经，从珍珠港出发，穿越北极冰盖，最终到达美国波特兰港，历时 21 夭。其中穿越北极时以 20 海里①的时速在水下潜航8146 海里，在即将到达目的地时潜艇浮出水面，经过测量，定位误差仅为 20 海里。这一震惊世界的成功，充分显示了惯性导航系统有别于其他导航系统的独特优点 自主性、隐蔽性、信息的完备性。这些特点在军事应用中尤为重要。

惯性导航系统发展依靠三方面科学技术发展的支撑 新概念测量原理和新型惯性器件、先进制造工艺、计算机技术。

20世纪 60 年代，在液浮陀螺臻至完善的同时，挠性陀螺的研究逐渐起步。关于挠性支承的概念首先是由英国皇家航空研究院(RAE) 1946 年提出的，1965 RAE研制了兰套挠性陀螺原理样机，证明了原理的可行性(许国祯，1995;Craig R J G, 1972) 1969 年英国费伦蒂公司根据国防合同研制了 个陀螺，进行了长达十年的长时间测试。挠性陀螺有两种支承形式 细颈支承和平衡环挠性接头支承.采用后一种支承的陀螺称为动力调谐陀螺，原因是陀螺达到设计转速时，平衡环扭摆产生的惯性力矩正好抵消挠性接头产生的弹性力矩，即陀螺正常工作时处于动力调谐状态。相对液浮陀螺，动力调谐陀螺结构简单，易于制造，成本低廉，因而得到了广泛应用。经过不断改进和完善，用于平台式惯导的动力调谐陀螺的漂移已达到了。.01 /h，精度最高的可达到 O.OOl /h。在 20 世纪的最后 20 多年时间内，用动力调谐陀螺构造的中等精度(航空标准惯导的定位精度为 1n mile/h)平台式惯性导航系统大量出现，最具代表性的是美国利登公司LTN-72 系列惯导系统，在当时几乎占据了世界民航飞机标准惯导的全部订单。

另一类新概念支承陀螺是静电陀螺，静电支承原理是由美国伊利诺伊大学诺特西克教授在1952 年率先提出的(Nordsieck A, 1962) 20 世纪 70 年代，美国霍尼韦尔公司和罗克韦尔公司研制出了静电陀螺监控器和导航仪。该型陀螺经过不断的改进和完善，漂移误差一般都小于$1 0 ^ { - 4 }$ o/h，更高精度可达到 $1 0 ^ { - 6 } \sim 1 0 ^ { - 7 }$ o/h，在人造卫星的失重和真空环境条件下，最高精度甚至达到了 $1 0 ^ { - 9 } \sim 1 0 ^ { - 1 1 }$ o/h，该类陀螺据称曾在人造卫星上做过对爱因斯坦相对论的验证试验。正由于静电陀螺具有如此高的精度，所以被重点用作战略武器机动技掷平台的高精度基准，诸如战略核潜艇的静电陀螺监控器和静电陀螺惯性导航系统，远程战略轰炸机的静电陀螺惯性导航系统等。

与静电支承相似的支承还有磁悬浮支承和气浮支承。由于这些支承存在易受环境条件干扰、制造工艺复杂等缺陷，精度远不如静电陀螺，所以应用范围都很有限，并未形成主流。激光陀螺和光纤陀螺的出现是惯性技术的一场大革命，这类光学陀螺与传统的机械转子陀螺的工作原理有本质的区别，后者服从解释宏观世界的牛顿力学，而前者服从解释微观世界的量子力学，因此是一种全新概念的惯性器件(Chow W W et al. 1980)。利用光的干涉原理测量旋转运动早20 世纪初就有人提出过。 1913 年法国物理学家萨格奈克研制了一种光学干涉仪(Sagnac G,1913a;Sagnac G, 1913b) , 1925 年迈克尔孙根据干涉仪研制出了一种光学陀螺测量装置，用于测量地球的自转角速度(Michelson A H et al. 1925) 。该装置由 300mX600m 的矩形光学回路构成，但所采用的光源是普通光，相干'性极差，干涉条纹的移动量仅为干涉条纹间距的 1/4，测量精度极低。 1960 年物理学家发明了激光，1962 年世界上第一台氮氛激光器问世，以美国为首的技术先进国家开始研制激光陀螺， 1963 年，斯派利陀螺公司首次成功演示了环形激光陀螺(Macek W M et al. 1963) 。经过不懈努力，美国霍尼韦尔公司于 1982 年批量生产 GG1342光陀螺，该型陀螺 MTBF高达 90000h，是机械转子陀螺无法比拟的(许国祯， 1995) 。抖动型激光陀螺的精度等级在 $1 0 ^ { - 2 } { \sim } 1 0 ^ { - 3 } ~ ^ { \circ } / \mathrm { h }$ 量级范围内，是构造航空标准惯导的理想惯性器件，美国利登公司生产的 LTN-92 系列激光捷联惯导被认定为美国民航飞机、政府运输和行政飞机的必备导航系统，取代了 LTN-72 而成为标准机载惯导系统。

光纤陀螺是比激光陀螺稍晚出现的另一类光学陀螺。与激光陀螺相比，光纤陀螺的体积更小，功耗更低，并且价格低廉，更便于批量生产(许国祯， 1995)0 尽管光纤陀螺的精度还赶不上激光陀螺，目前还只能满足战术武器的精度要求，但随着光纤制造技术和集成光学器件性能的不断完善，其潜在的优势将逐渐显露出来。

随着制作集成电路的硅半导体工艺的成熟和完善，20 世纪 80 年代开始出现了微型机械、微型传感器和微型执行器的微机械制造技术，这种采用微型机械机构和控制电路工艺制造微机电系统的技术常称为 MEMS技术。 MEMS技术在惯性技术领域中的成果体现是硅微陀螺及硅微加速度计。 1991 年麻省理工学院德雷帕实验室成功研制出微型惯性测量组合，包括三个陀螺仪、三个加速度计以及相应的控制电路，陀螺精度已达到漂移小于 $\mathrm { 1 0 ^ { \circ } / h }$ ，而整个惯性测量组合的体积仅为 $\mathrm { 2 c m } { \times } 2 \mathrm { c m } { \times } 0 . 5 \mathrm { c m }$ ，质量仅为 $5 \mathrm { g }$ ，并已在增程制导炮弹上作了试验(Ohlmeyer E Jet al. ,1998) 0 MEMS惯性器件不仅具有因为体积小、重量轻、易于安装、高可靠、耐冲击而应用广泛的特点，而且可以实现大批量的生产，在成本上具有优势，因此距离在民用领域及战术武器领域内普及应用的时间不会太久。

惯性技术水平的标志一方面反映在惯性器件的性能及制造工艺水平上，另一方面反映在系统设计理论及工程实现水平上。平台式惯性导航系统中，用机电控制方法建立起物理实体平台，用于模拟所要求的导航坐标系。由于有惯性平台隔离了运载体的角运动，导航坐标系的旋转又十分缓慢，所以平台式惯导系统中陀螺的动态范围可以很小，导航计算机的解算负担也比较轻，针对 20世纪 60\~70 年代计算机水平还不高，陀螺的施矩电流还不能太大的实际情况，采用物理平台构造惯性导航系统是十分合适的。平台式惯导的最大缺点是结构复杂、体积大、重量重、可靠性差，所以随着激光陀螺批量制造技术的成熟，捷联式惯性导航系统正在各个领域逐步取代平台式惯性导航系统，特别是诸如飞机、导弹等中低精度应用领域几乎都采用捷联式愤导系统。利登公司在十年前就推出 LTN-92 激光捷联式惯导系统，作为替换挠性陀螺平台式惯导 LTN-72的换代产品，波音和空中客车民航机几乎都装备 LTN-92 激光捷联式惯导系统。

捷联式惯导系统的最大特点是依靠算法建立起导航坐标系，即平台坐标系以数学平台形式存在，这样省略了复杂的物理实体平台，结构简单、体积小、重量轻、成本低、维护简便、可靠性高，还可通过余度技术提高系统的容错能力。但这些好处是用复杂的算法设计和繁重的计算负荷换取的。姿态更新解算是捷联式惯导的关键算法。传统的姿态更新算法有欧拉角法、方向余弦法和四元数法，其中四元数毕卡算法简单、计算量小，因而在工程实际中常采用。但四元数毕卡算法仅为单子样算法，不可交换误差补偿不彻底，特别是运载体姿态变化剧烈时，这种误差更加严重。 1971 Bortz Jordan 提出了等效旋转矢量概念 Bortz J E.1971).将运载体的姿态四元数更新转化为姿态变化四元数的更新，为姿态更新的多子样算法提供了理论依据。 1980 Gil-more 提出了在快速计算回路内迭代解算旋转矢量，在慢速计算回路内解算姿态四元数CGilmore J P.1980). 1983 il1er 探讨了锥运动条件下等效旋转矢量的三子样优化算法，优化指标是圆锥误差影响达到最小CMil1er R B.1983)。在此基础上， Lee Yoon 研究了四子样算法 Le J G et a1. .1990) .Jiang研究了利用本更新周期内的三子样及前更新周期内的角增量计算旋转矢量的优化算法(Jian Y E et al. .1992). 1995 Musoff 提出了圆锥补偿算法的优化指标，分析了算法误差与补偿周期的关系CMusoff H et a1. .1995)。对于运动状态变化剧烈和导航定位精度要求特别高的应用场合，除对圆锥运动效应作补偿讨算外，还要对划桨运动效应和涡卷运动效应作补偿计算，对此. Savage 作了系统研究 Savage P G.1998a.1998b)

## 1. 2 常用关系式

在本书的分析和推导中经常用到一些运算关系式，此处列出一部分.

## 1. 2.1 坐标变换

设坐标系 $O { - } X _ { 1 } Y _ { 1 } Z _ { 1 }$ 绕 $O Z _ { 1 }$ 轴旋转 角后得到坐标系 $O - X _ { 2 } Y _ { 2 } Z _ { 2 }$ ，空间矢量r在$O { - } X _ { 1 } Y _ { 1 } Z _ { 1 }$ 简称坐标系 1) 内的投影为 $\begin{array}{c} \begin{array} { r l r l } { \big [ r _ { _ { X _ { 1 } } } } & { { } } & { r _ { _ { Y _ { 1 } } } } & { { } \ r _ { _ { Z _ { 1 } } } } \end{array} \big ] ^ { \mathrm { T } }  \end{array}$ ，在 $O { - } X _ { 2 } Y _ { 2 } Z _ { 2 }$ 简称坐标系 2) 内的投影为 $\begin{array}{c} \begin{array} { r l r l } { \big [ r _ { _ { X _ { 2 } } } } & { { } } & { r _ { _ { Y _ { 2 } } } } & { { } \ r _ { _ { Z _ { 2 } } } } \end{array} \big ] ^ { \mathrm { T } }  \end{array}$ ，要求推导出两组坐标值间的关系.由于旋转轴绕 $O Z _ { 1 }$ 轴进行，所以 坐标未变，即有 $r _ { z _ { 1 } } = r _ { z _ { 2 } }$ 由图1. 2.

$$
r _ {X _ {2}} = O A + A B + B C = O D \cos \alpha + B D \sin \alpha + B F \sin \alpha = r _ {X _ {1}} \cos \alpha + r _ {Y _ {1}} \sin \alpha
$$

![](images/c4eb6782e893d05acd65b8c823126919fe1fe644c462a2c4d8d59bf44cb1781e.jpg)  
1. 2. 1 坐标系间的变换关系

将上述=式写成矩阵形式:

$$
\left[ \begin{array}{c} r _ {X _ {2}} \\ r _ {Y _ {2}} \\ r _ {Z _ {2}} \end{array} \right] = \left[ \begin{array}{c c c} \cos \alpha & \sin \alpha & 0 \\ - \sin \alpha & \cos \alpha & 0 \\ 0 & 0 & 1 \end{array} \right] \left[ \begin{array}{c} r _ {X _ {1}} \\ r _ {Y _ {1}} \\ r _ {Z _ {1}} \end{array} \right]\tag{1.2.1}
$$

记

$$
\boldsymbol {r} ^ {1} = \left[ \begin{array}{l} r _ {X _ {1}} \\ r _ {Y _ {1}} \\ r _ {Z _ {1}} \end{array} \right], \quad \boldsymbol {r} ^ {2} = \left[ \begin{array}{l} r _ {X _ {2}} \\ r _ {Y _ {2}} \\ r _ {Z _ {2}} \end{array} \right], \quad \boldsymbol {C} _ {1} ^ {2} = \left[ \begin{array}{c c c} \cos \alpha & \sin \alpha & 0 \\ - \sin \alpha & \cos \alpha & 0 \\ 0 & 0 & 1 \end{array} \right]
$$

则式(1. 2. 1)可写成

$$
\boldsymbol {r} ^ {2} = \boldsymbol {C} _ {1} ^ {2} \boldsymbol {r} ^ {1}
$$

该式描述了同一矢量在不同坐标系内投影间的变换关系， $\mathbf { \nabla } _ { \mathbf { C } _ { 1 } ^ { 2 } }$ 称为从坐标系 至坐标系 的变换矩阵。经观察后可发现 $C _ { 1 } ^ { 2 }$ 的诸元是坐标系 各轴上的单位 在坐标系 各轴上的投影，其中技影关系可用图1. 2. 来表示。

![](images/6737d322ee3e811c0290c3195f641efabcdb25fe7493bee558880105b104af03.jpg)  
1. 2. 2 坐标系 和坐标系 之间的投影关系

上述变换关系分析中，坐标系 是经坐标系 仅绕 $Z _ { 1 }$ 轴旋转 $\alpha$ 角后获得的，为便于叙述，称仅绕一根轴的旋转为基本旋转。两坐标系问任何复杂的角位置关系都可以看作有限次基本旋转的复合，变换矩阵等于基本旋转确定的变换矩阵的连乘，连乘顺序依基本旋转的先后次序由右向左排列。例如运载体的空间姿态可看做依次绕航向轴、俯仰轴、横滚轴做基本旋转后的复合结果，如图1. 2. 所示。图中 坐标系为地理坐标系 $X _ { n }$ 指东、 $. Y _ { n }$ 指北、 $. Z _ { n }$ 指天 $, b$ 坐标系为机体坐标系 $X _ { b }$ 指右、 $\cdot Y _ { b }$ 指前、 $\mathcal { L } _ { b }$ 指上。飞机的空间角位置由下述依次基本旋转确定:

$$
O - X _ {n} Y _ {n} Z _ {n} \xrightarrow [ \text {旋转} \Psi ]{\text {绕} - Z _ {n} \text {轴}} O - X _ {1} Y _ {1} Z _ {1} \xrightarrow [ \text {旋转} \theta ]{\text {绕} X _ {1} \text {轴}} O - X _ {2} Y _ {2} Z _ {2} \xrightarrow [ \text {旋转} \gamma ]{\text {绕} Y _ {2} \text {轴}} O - X _ {b} Y _ {b} Z _ {b}
$$

![](images/1e6755856beccf2e8acb2f7cfeb29f8238118808b29e79bcedc33a8e8ef4df06.jpg)  
图1. 2. 3 飞机空间角位置的确定

各次基本旋转对应的变换矩阵为

$$
\boldsymbol {C} _ {n} ^ {1} = \left[ \begin{array}{c c c} \cos \Psi & - \sin \Psi & 0 \\ \sin \Psi & \cos \Psi & 0 \\ 0 & 0 & 1 \end{array} \right], \quad \boldsymbol {C} _ {1} ^ {2} = \left[ \begin{array}{c c c} 1 & 0 & 0 \\ 0 & \cos \theta & \sin \theta \\ 0 & - \sin \theta & \cos \theta \end{array} \right], \quad \boldsymbol {C} _ {2} ^ {b} = \left[ \begin{array}{c c c} \cos \gamma & 0 & - \sin \gamma \\ 0 & 1 & 0 \\ \sin \gamma & 0 & \cos \gamma \end{array} \right]
$$

所以姿态矩阵为

$$
\begin{array}{r l} \boldsymbol {C} _ {n} ^ {b} & = \boldsymbol {C} _ {2} ^ {b} \boldsymbol {C} _ {1} ^ {2} \boldsymbol {C} _ {n} ^ {1} = \left[ \begin{array}{c c c} \cos \gamma & 0 & - \sin \gamma \\ 0 & 1 & 0 \\ \sin \gamma & 0 & \cos \gamma \end{array} \right] \left[ \begin{array}{c c c} 1 & 0 & 0 \\ 0 & \cos \theta & \sin \theta \\ 0 & - \sin \theta & \cos \theta \end{array} \right] \left[ \begin{array}{c c c} \cos \Psi & - \sin \Psi & 0 \\ \sin \Psi & \cos \Psi & 0 \\ 0 & 0 & 1 \end{array} \right] \\ & = \left[ \begin{array}{c c c} \cos \gamma \cos \Psi + \sin \gamma \sin \Psi \sin \theta & - \cos \gamma \sin \Psi + \sin \gamma \cos \Psi \sin \theta & - \sin \gamma \cos \theta \\ \sin \Psi \cos \theta & \cos \Psi \cos \theta & \sin \theta \\ \sin \gamma \cos \Psi - \cos \gamma \sin \Psi \sin \theta & - \sin \gamma \sin \Psi - \cos \gamma \cos \Psi \sin \theta & \cos \gamma \cos \theta \end{array} \right] \end{array}\tag{1.2.2}
$$

式中 $\mathbf { \nabla } , \mathbf { C } _ { n } ^ { b }$ 与旋转次序有关，即当旋转角1J $\psi , \theta , \gamma$ 不都为小角时，对应于不同的旋转次序，坐标系 $b$ 的最终空间位置是不同的，这就是常说的有限转动的不可交换性。但当? $\Psi , \theta , \gamma$ 都为小角时，忽略小角间的高阶小量

$$
\boldsymbol {C} _ {n} ^ {b} = \left[ \begin{array}{c c c} 1 & - \boldsymbol {\Psi} & - \boldsymbol {\gamma} \\ \boldsymbol {\Psi} & 1 & \theta \\ \boldsymbol {\gamma} & - \boldsymbol {\theta} & 1 \end{array} \right]
$$

式中，1 $\Psi , \theta , \gamma$ 的单位为弧度。此时由? $\Psi , \theta , \gamma$ 构成的列向量[1 $\begin{array} { r l } { \mathbf { \bar { \Psi } } _ { } \Psi } & { { } \theta _ { } } \end{array} \begin{array} { l } { \gamma _ { \pm } \mathbf { \bar { \mathbf { \Gamma } } } } \end{array}$ 可视为三维空间矢量，各分量正负号的规定为 当产生小角的旋转方向与坐标轴指向相同时该小角取正，否则取负。此时旋转后坐标系的最终角位置与旋转次序无关，这就是常说的无限转动与旋转次序无关。

根据上述分析，可得出如下一般关系 设坐标系 偏离坐标系 的偏离角 $\phi _ { x } \setminus \phi _ { y } \setminus \phi _ { z }$ 弘均为小角，则

$$
\boldsymbol {C} _ {T} ^ {P} = \left[ \begin{array}{c c c} 1 & \phi_ {z} & - \phi_ {y} \\ - \phi_ {z} & 1 & \phi_ {x} \\ \phi_ {y} & - \phi_ {x} & 1 \end{array} \right]\tag{1.2.3}
$$

由于直角坐标系间的变换矩阵为单位正交矩阵，所以如果在坐标系 至坐标系 的等效旋转中各坐标系都保持为直角坐标系，则根据单位正交矩阵的性质有

$$
\mathbf {C} _ {b} ^ {n} = (\mathbf {C} _ {n} ^ {b}) ^ {- 1} = (\mathbf {C} _ {n} ^ {b}) ^ {\mathrm{T}}
$$

## 1. 2. 2 矢量的叉乘

矢量是既有大小又有方向的量，常用两种方式来描述矢量。设有矢量 可用 的模 $r = \left| \boldsymbol { r } \right|$ 的单位矢量 $\pmb { u } = \frac { \pmb { r } } { r }$ 来描述: $\mathbf { \nabla } _ { : } r = r \pmb { u }$ ，如此描述的矢量常称为物理矢量。也可以用 在坐标系的各轴上的投影来描述: $\begin{array} { r l } { r ^ { n } = [ r _ { x } ^ { n } } & { { } r _ { y } ^ { n } \quad r _ { z } ^ { n } ] ^ { \mathrm { T } } } \end{array}$ 如此描述的矢量称为数学矢量。矢量的叉乘可用物理矢量和数学矢量来表示.

设有物理矢量 $\pmb { t = r } \times \pmb { s }$ 的方向由右手定则确定，大小为 $t { = } | { \pmb t } | = | { \pmb r } | \cdot | { \pmb s } | \sin ( r , s )$

设有数学矢量r" $\begin{array} { r } { { \boldsymbol { r } } ^ { n } = [ r _ { x } ^ { n } \quad r _ { y } ^ { n } \quad r _ { z } ^ { n } ] ^ { \mathrm { T } } , { \boldsymbol { s } } ^ { n } = [ s _ { x } ^ { n } \quad s _ { y } ^ { n } \quad s _ { z } ^ { n } ] ^ { \mathrm { T } } , t ^ { n } = { \boldsymbol { r } } ^ { n } \times { \bf s } ^ { n } } \end{array}$ ，则

$$
\left[ \begin{array}{c} t _ {x} ^ {n} \\ t _ {y} ^ {n} \\ t _ {z} ^ {n} \end{array} \right] = \left[ \begin{array}{c c c} 0 & - r _ {z} ^ {n} & r _ {y} ^ {n} \\ r _ {z} ^ {n} & 0 & - r _ {x} ^ {n} \\ - r _ {y} ^ {n} & r _ {x} ^ {n} & 0 \end{array} \right] \left[ \begin{array}{c} s _ {x} ^ {n} \\ s _ {y} ^ {n} \\ s _ {z} ^ {n} \end{array} \right]\tag{1.2.4}
$$

式中 ， $\pmb { t } ^ { n } = [ t _ { x } ^ { n } t _ { y } ^ { n } t _ { z } ^ { n } ] ^ { \mathrm { T } }$

## 1. 2.3 哥氏定理

哥民定理用于描述绝对变化率与相对变化率间的关系，设有矢量 是两个做相对旋转的坐标系，则哥氏定理可描述为

$$
\left. \frac {\mathrm{d} \boldsymbol {r}}{\mathrm{d} t} \right| _ {m} = \left. \frac {\mathrm{d} \boldsymbol {r}}{\mathrm{d} t} \right| _ {n} + \boldsymbol {\omega} _ {m n} \times \boldsymbol {r}\tag{1.2.5}
$$

式中， $\left. { \frac { \mathrm { d } \pmb { r } } { \mathrm { d } t } } \right| _ { m }$ 和 $\left. \frac { \mathrm { d } \pmb { r } } { \mathrm { d } t } \right| _ { 0 }$ |是分别在 坐标系和 坐标系内观且要到的 的时间变化率， ${ \pmb { \omega } } _ { m m }$ 是坐标系n对坐标系拙的旋转角速度。如果将式(1. 2. 5) 两边的矢量都向 坐标系投影，则有

$$
\boldsymbol {r} ^ {m} = \boldsymbol {C} _ {n} ^ {m} \boldsymbol {r} ^ {n} + \boldsymbol {\omega} _ {m n} ^ {m} \times \boldsymbol {r} ^ {m}\tag{1.2.6}
$$

式中

$$
\boldsymbol {r} ^ {m} = \left[ \begin{array}{c c c} \dot {r} _ {x} ^ {m} & \dot {r} _ {y} ^ {m} & \dot {r} _ {z} ^ {m} \end{array} \right] ^ {\mathrm{T}}, \quad \boldsymbol {r} ^ {n} = \left[ \begin{array}{c c c} \dot {r} _ {x} ^ {n} & \dot {r} _ {y} ^ {n} & \dot {r} _ {z} ^ {n} \end{array} \right] ^ {\mathrm{T}}
$$

## 第一篇惯性器件

惯性器件或称惯性仪表，即陀螺仪和加速度计。陀螺仪用来测量运载体的角运动，或在控制角运动的伺服回路中用作控制环节，加速度计用来测量运载体的加速度.其中，"惯性"具有两重含义 陀螺和加速度计服从牛顿力学，基本工作原理是动量矩定理和牛顿第二定理，即基本惯性原理 作为测量元件时输出量都是相对惯性空间的测量值，如角速度输出是相对惯性空间的角速度，加速度输出是绝对加速度，陀螺作为控制元件时，产生的角速度是相对惯性空间的角速度.

本篇主要对陀螺仪和加速度计作系统介绍。

## 章陀螺仪

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

上述诸关系式代人式(2.3.2) ，得

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

液浮陀螺采用浮桶液浮支承，浮桶产生的浮力与陀螺组件的重力对消，使陀螺组件沿输出轴的摩擦力矩接近为零，所以精密制造的液浮陀螺的漂移十分微小，达到 性级水平，即漂移优于O.Ol"/h，这种精度等级的液浮陀螺常用来构建高精度平台式惯导的惯性平台。因此积分陀螺常用于稳定平台和惯性平台的设计。

忽略交叉糯合 $\alpha H _ { G o _ { z } }$ ，并假设输出轴上作用有指令力矩 $M _ { \mathrm { c m d } }$ 和于扰力矩 $M _ { \mathrm { { i n t } } }$ ，则由式 (2. 3. 3)

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

将式 (2.3. 7) 代人上式，得

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

上式右侧第一项称角加速度误差，由沿输出轴的角加速度引起 第二项称不等惯性误差，由陀螺组件绕 轴和 轴的转动惯量不相等( $( I _ { S } { \neq } I _ { I } )$ 引起 第三至五项称惯性积误差，由陀螺框架的惯性积引起 第六项称不等惯性藕合误差，由 $I _ { S } \ne I _ { I }$ ，和 $\chi \neq 0$ 引起;第七项称交叉搞合误差，由 $\alpha \neq 0$ 使陀螺错误感测 $\omega _ { S }$ 引起。

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

## 4) 不等惯性藕合误差

不等惯性搞合误差的表达式为

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

式中 $\frac { I _ { S } - I _ { I } } { k }$ 称为不等惯性酣睡系数。为了如此项髓，一方面应尽量减小 $I _ { S }$ Ej $I _ { I }$ á9盟，另一方面应尽量提高再平衡回路的增益晶，在 足够高的情况下不等惯性搞合误差是比较小的。

## 5) 交叉糯合误差

交叉搞合误差表达式为

$$
\delta \omega_ {I 5} = \omega_ {S} \alpha
$$

该项误差产生的原因是:当输出角 $\alpha \neq 0$ 时，陀螺壳体绕自转轴的旋转角速度 $\omega _ { S }$ 的正弦分量$\omega _ { S } \sin \alpha { \approx } \omega _ { S } \alpha$ 被陀螺感测到，引起陀螺错误输出。

将式(2.3.20)代人上式，得

$$
\delta \omega_ {I 5} = \frac {H}{k} \omega_ {S} \omega_ {I}
$$

式中， $\frac { H } { k }$ ?称为交叉酣睡础。该项误差与陀螺的角动量 成正比，与再平衡峭的增益成反比，所以提高品对减小此项误差显得特别重要。例如，若 $H { = } 2 . 5 \times 1 0 ^ { 5 } \mathrm { g \cdot { c m ^ { 2 } } / s }$ k=$2 . 5 { \times } 1 0 ^ { 2 } \mathrm { N } \cdot \mathrm { m } ^ { 2 } / \mathrm { r a d } \mathrm { H } \bar { \cdot }$ ，交叉搞合系数约为 $2 0 ^ { \circ } / \mathrm { h } / ( \mathrm { r a d } / \mathrm { s } ) ^ { 2 }$ ，若 增大 10 倍，则此项误差减小为$1 / 1 0$ 。因此应尽量提高 以减小交叉榈合误差。

## 3. 羊自由度陀螺动态漂移误差的数学模型

将式 (2.3.20)代人式 (2. 3. 19) ，经整理后得

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

如果忽略误差模型中的次要误差项，仅考虑角加速度误差、不等惯性误差和交叉搞合误差，则动态漂移数学模型可简化为

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

上述诸式代人式(2.4.7) ，得

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

(2.4.3) 代人上式得

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

由图 2.4.5 可看出，动力调谐陀螺的两个通道存在交叉捐合。引起交叉搞合的因素有:调谐残余刚度，弹性扭杆的内阻尼系数及转子绕输出轴旋转所引起的惯性力矩。若忽略这些交叉榈合影响，则图 2. 4. 可简化成图 2.4.6

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

式中. $M _ { k 4 }$ 和 $M _ { k B }$ 是作用在 轴上的反馈力矩，由再平衡回路提供. $M _ { \mathrm { d A } }$ A和 $M _ { \mathrm { d } B }$ 是作用在轴上的干扰力矩。上述两式分别代人式(2.4.37) .得

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

3) 不等惯性精合误差

$$
\left[ \begin{array}{l} \delta \omega_ {A 3} \\ \delta \omega_ {B 3} \end{array} \right] = \frac {C - A}{H} \left[ \begin{array}{c c} \dot {\omega} _ {S} - \omega_ {A} \omega_ {B} & \omega_ {A} ^ {2} - \omega_ {S} ^ {2} \\ \omega_ {S} ^ {2} - \omega_ {B} ^ {2} & \dot {\omega} _ {S} + \omega_ {A} \omega_ {B} \end{array} \right] \left[ \begin{array}{l} \alpha \\ \beta \end{array} \right]
$$

该项误差是由转子的极转动惯量与赤道转动惯量不相等，并通过陀螺、输出角 $\alpha$ 和 $\beta$ 的榈合作用引起的。由于测量输出角 $\alpha$ 和 $\beta$ 并用于动态误差补偿有诸多不便，所以应该用输入角速度来表达 $\alpha$ 和 $\beta$ o

假设陀螺的力矩再平衡回路具有解藕作用，即两测量通道是元捐合的，如图 2.4.6 所示。在力矩反馈状态下

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

对于常值输人角速度，即 $\omega _ { A } \left( s \right) = \frac { \omega _ { A } } { s } , \omega _ { B } \left( s \right) = \frac { \omega _ { B } } { s }$ 旦，输出角的稳态值为

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

$\frac { C - A } { k }$ 或 $\frac { C } { 2 k }$ 称为不等惯性糯合误差系数。

为了减小不等惯性精合误差，一方面应尽量减小转子的极转动惯量与赤道转动惯量的差值，另一方面还应尽量提高再平衡回路的增益晶，在 足够高的情况下，不等惯性搞合误差是比较小的。

4) 交叉藕合误差

$$
\left[ \begin{array}{c} \delta \omega_ {A 4} \\ \delta \omega_ {B 4} \end{array} \right] = \left[ \begin{array}{c} - \beta \omega_ {S} \\ \alpha \omega_ {S} \end{array} \right]
$$

该项误差是由于输出角 $\alpha { \neq } 0 , \beta { \neq } 0$ 时 $\omega _ { S }$ 在陀螺敏感轴上出现分量引起的。式(2.4.43)人上式，得

$$
\left[ \begin{array}{c} \delta \omega_ {A 4} \\ \delta \omega_ {B 4} \end{array} \right] = \frac {H}{k} \left[ \begin{array}{c} \omega_ {B} \omega_ {S} \\ - \omega_ {A} \omega_ {S} \end{array} \right]
$$

时，? $\frac { H } { k }$ 称为交叉藕合误差系数。由于该系数的分子是数值很大的角动量，所以提高再平衡回路的增益对减小该误差系数显得特别重要。例如，设陀螺角动量 $H { = } 1 0 ^ { 6 } \mathrm { g } \bullet \mathrm { c m } ^ { 2 } / \mathrm { s }$ ，当再平衡回路增益 $k { = } 1 0 ^ { 3 } \mathrm { N } \cdot \mathrm { m } / \mathrm { r a d } \mathrm { H }$ 时，交叉藕合误差系数为 $2 0 ^ { \circ } / \mathrm { h } / ( \mathrm { r a d } / \mathrm { s } ) ^ { 2 }$ ，若增益增大十倍，则误差系数减小到1/10 因此，再平衡回路应具有足够高的增益，以减小交叉搞合误差。

以上四种动态误差是原理性误差，与陀螺的制造精度和供电电源无关。下面讨论陀螺未达到动力调谐工作状态所引起的两种误差 失调转速误差和失调转速藕合误差，

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

## 6) 失调转速藕合误差

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

若将式(2.4.46)代人上式，则上述漂移还可以写成

$$
\delta \omega_ {A 6} = \frac {2 \delta \Omega \sqrt {K (a - \frac {c}{2})}}{k} \omega_ {B}\tag{2.4.49a}
$$

$$
\delta \omega_ {B 6} = - \frac {2 \delta \Omega \sqrt {K (a - \frac {c}{2})}}{k} \omega_ {A}\tag{2.4.49b}
$$

式中， $\frac { 2 \mathfrak { d } \Omega \sqrt { K \left( a - \frac { c } { 2 } \right) } } { k }$ 称为失调转速藕合误差系数。

3. 动力调谐陀螺的动态漂移误差数学模型

计人式 (2.4.47) 和式 (2.4.49)所列误差后，式 (2.4.41)应改写为

$$
\begin{array}{r l} \left[ \begin{array}{c} \delta \omega_ {A} \\ \delta \omega_ {B} \end{array} \right] & = \frac {A}{H} \left[ \begin{array}{c} - \dot {\omega} _ {B} \\ \dot {\omega} _ {A} \end{array} \right] + \frac {C - A}{H} \left[ \begin{array}{c} \omega_ {A} \omega_ {S} \\ \omega_ {B} \omega_ {S} \end{array} \right] + \frac {C - A}{k} \left[ \begin{array}{c} \omega_ {B} \omega_ {S} ^ {2} - \dot {\omega} _ {S} \omega_ {A} \\ - \omega_ {A} \omega_ {S} ^ {2} - \dot {\omega} _ {S} \omega_ {B} \end{array} \right] \\ & + \frac {H}{k} \left[ \begin{array}{c} \omega_ {B} \omega_ {S} \\ - \omega_ {A} \omega_ {S} \end{array} \right] + \frac {\delta \Omega}{\Omega_ {0}} \left[ \begin{array}{c} \omega_ {A} \\ \omega_ {B} \end{array} \right] + \frac {2 \delta \Omega \sqrt {K (a - \frac {c}{2})}}{k} \left[ \begin{array}{c} \omega_ {B} \\ - \omega_ {A} \end{array} \right] \end{array}\tag{2.4.50}
$$

上式右侧第一项为角加速度误差，第二项为不等惯性误差，第三项为不等惯性搞合误差，第四项为交叉糯合误差，第五项为失调转速误差，第六项为失调转速糯合误差，每项都有明确的物理意义。从表面上看式(2.4.50)可直接应用于工程，其中角速度由捷联陀螺提供，各系数可根据物理参数确定之，但由于制造偏差和回无法准确确定出，所以直接应用于工程还存在诸多不便.工程上常在实验室条件下采用测试的方法确定出综合系数，而不必确定出每个物理参数，因此动态漂移误差模型采用如下形式

$$
\begin{array}{r l} \delta \omega_ {A} & = K _ {B d} (A) \dot {\omega} _ {B} + K _ {A} (A) \omega_ {A} + K _ {B} (A) \omega_ {B} + K _ {A S} (A) \omega_ {A} \omega_ {S} + K _ {B S} (A) \omega_ {B} \omega_ {S} \\ & + K _ {B S S} (A) \omega_ {B} \omega_ {S} ^ {2} + K _ {A S d} (A) \omega_ {A} \dot {\omega} _ {S} \end{array}\tag{2.4.51a}
$$

$$
\begin{array}{r l} \delta \omega_ {B} & = K _ {A d} (B) \dot {\omega} _ {A} + K _ {A} (B) \omega_ {A} + K _ {B} (B) \omega_ {B} + K _ {A S} (B) \omega_ {A} \omega_ {S} + K _ {B S} (B) \omega_ {B} \omega_ {S} \\ & + K _ {A S S} (B) \omega_ {A} \omega_ {S} ^ {2} + K _ {B S d} (B) \omega_ {B} \dot {\omega} _ {S} \end{array}\tag{2.4.51b}
$$

其中主要误差有三项 角加速度误差、不等惯性误差和交叉糯合误差，工程上常取这三项作为简化模型:

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

1962 年氮-氛激光器问世，为光学陀螺这一新概念的实现奠定了技术基础。 1962 年起美国开始研制环形激光陀螺，其中霍尼韦尔(Honeywell)公司起步最早，成果最为显著。 1975 年该公司研制的激光陀螺惯导系统在 飞机上试飞成功，精度为 2. 2n mile/h, 1978 年在波音 72机上试飞成功，精度为 1n mile 1982 年，在波音 $7 4 7$ 飞机上试飞成功，精度高达 O. 26n mile/h(许国祯， 1995)

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

由式(2.5.2)知激光陀螺的拍频输出与输人角速度呈线性关系，但工程实际中并非如此，事实

上，当输入角速度小到一定程度时，陀螺的拍频输出为零。这种在小角速度输入下陀螺无输出的现象称为激光陀螺的闭锁效应，如图 2.5.4 所示，闭锁效应使陀螺在测量中出现死区。

产生闭锁效应的主要原因是:光路的各种非均匀因素，如反射镜及透镜各点的非均匀损耗与散射，谐振腔内的尘埃微粒，散射使正反方向传播的两束光互相搞合，发生频率牵引，使两束光趋向同步，频率差消失。解决闭锁效应的方法主要有兰种。

## 1)抖振偏频法

![](images/8ce52e4cb3ed479cd7d57a9b4740c83a0748bb6e966eed4cbc5b0c4b8a771896.jpg)

陀螺安装在抖动轮外般土，轮轴固定在基座上，抖动轮

2.5.4 激光陀螺的闭锁效应

辐沿径向贴有压电陶瓷，压电陶瓷的伸缩推挽作用使轮般相对轮轴做微辐往复角运动，带动陀螺绕输人轴做角抖动，抖动频率约 400Hz，幅度数角分。

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

## 3) 磁镜偏颇法

![](images/f3f998aae139b13d19d661416f236b26782934494a53b2f3718cac45dd2d58eb.jpg)  
2. 磁镜偏频法

磁镜偏频法利用了横向磁光克尔效应，如图 2.5.6

两束光同频率同相位，人射方向相反，如果所加磁场方向平行于反射镜，且垂直于人射面，则在该磁场作用下，光束在反射后会产生附加相移，即光程发生改变，光束 和光束 相移大小相等，但符号相反。这就是横向磁光克尔效应。

如果所加磁场是交变的，则 两束光之间的相移也是交变的，相位差(光程差)也是交变的。当输入角

速度 $\omega _ { i } = 0$ 时，光程差在一个周期内的积分为零;若 $\omega _ { i } \neq 0$ ，则光程差在一个周期内的积分即为由$\omega _ { i }$ 引起的角增量。所以此法实质上与抖振偏颇法的思路是一样的，不同的是一个是磁场抖振，另一个是机械抖振。

## 2. 随机将走误差

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

将式 (2. 5. 6) 代人式 (2.5.5) ，得

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

## 5) \~J度系数误差

激光陀螺的刻度系数即为脉冲当量，即一个脉冲输出对应的角增量。刻度系数误差从数个 ppm①至数十个 ppm不等，视陀螺质量高低而定。

如果要去掉刻度系数中的温度影响，可按热补偿模型作软件补偿

$$
K = K _ {0} + K _ {1} (T _ {p} - T _ {r})\tag{2.5.26}
$$

式中， $T _ { r }$ ，和 $T _ { \mathbf { \nabla } \phi }$ 分别为基准参考温度和当前温度 $K _ { 0 }$ 为 $T _ { r }$ ，条件下的刻度系数 $K _ { 1 }$ 为补偿系数。

## 6) \~J度系数的不对称误差

刻度系数不对称误差定义为

$$
\Delta K _ {a s y} = \frac {K _ {+} - K _ {-}}{2}
$$

式中 $K _ { + }$ 为正向脉冲当量， 为负向脉冲当量。

## 光纤陀螺简介2. 6

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

## 章摆式加速度计

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

小到使- $- m a + F _ { \parallel } = 0$ 成立，则 $x { = } 0$ 此时加速度 $a \neq 0$ ，但加速度计元输出，定义此加速度为加速度计的灵敏阔，即

$$
a _ {\text {灵敏阈}} = \frac {F _ {\text {阻}}}{m}\tag{3.1.3}
$$

该指标反映了加速度计能测出的最小加速度。此外位移信号检测器的灵敏度也影响灵敏阔。惯性级加速度计的灵敏阔应优于 $1 0 ^ { - 4 } g$

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

按图 3. 1. 所示简化模型设计加速度计存在诸多问题，如质量块与基座支承面间存在摩擦力，由式。.1. 3)知该摩擦力将严重影响加速度计的灵敏阔;对质量块的运动难以实现精确约束;输出信号检测和对质量块的力反馈控制困难。因此，工程上都采用具有偏心质量的摆式结构作为加速度计的设计方案，常见的摆式加速度计简介如下。

## 1.液浮摆式加速度计

为了提高加速度计的灵敏阔，必须尽量减小质量块支承处的摩擦影响。与液浮陀螺相仿，解决摩擦影响的有效途径是采用液浮支承，图 3. 1. 是按此思路设计的液浮摆式加速度计。摆组件浸泡在浮液中，产生的浮力使轴承处的正压力减少至接近为 $0 _ { c }$ 。设 $c _ { m }$ 为摆组件的质量中 $\therefore \dot { \mathsf { L } } \mathsf { \Omega } ^ { } \bullet \mathsf { c } _ { _ F }$ 为摆组件的浮心，距支承轴的距离分别为 $L _ { m }$ 和 $L _ { F }$ '摆组件的质量为 $m .$ 浮力为 $F _ { \mathrm { ~ c ~ } }$ 。记 $I _ { A } , O _ { A } , P _ { A }$ 分别为加速度的输人轴、输出轴和摆轴，则在重力和浮力的共同作用下，摆组件绕输出轴的摆性力矩为

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

设沿 $I _ { A }$ 轴作用有加速度 $\textit { a } _ { I }$ 所产生的摆性力矩使摆组件绕 $O _ { A }$ 轴旋转产生输出角。 $\theta$ ，信号器输出与 $\theta$ 相应的电信号，经前置放大、解调和直流放大后加至力矩器，产生绕 $O _ { A }$ 轴的恢复力矩，平衡掉摆性力矩，从力矩器电流采样值可提取出输人加速度 $\boldsymbol { \mathcal { a } } _ { I }$

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

3. 2. 为挠性摆式加速度计的结构示意图。加速度计由摆组件、信号传感器、推挽式永磁力矩器和挠性杆组成，表内充有硅油作为阻尼液体，并设有精密温控装置，以保证仪表在恒定的工作温度下工作。挠性杆一端固定在表壳上，另一端粘贴有信号器动圃，形成悬臂梁，在摆片上固定有力矩器动圈，永久磁铁一端固定在表壳上，另一端空套在力矩器动圈内。当表壳(基座)具有沿 $I _ { A }$ 轴方向的加速度 时，挠性杆在惯性力矩作用下绕挠性接头中心位置旋转，产生输出角$\theta _ { 0 }$ .信号器检测出 $\theta _ { 0 }$ 。并转换成电压信号，经放大后加人力矩器动圈，产生的推挽力矩抵消惯性力矩，使 $\theta _ { 0 }$ 。回零，再平衡回路中的电流经采样电阻变换成输出电压，从此电压可提取出加速度 $a$ 为分析方便，常将加速度计的输入轴(敏感轴)记为 $I _ { A }$ 输出轴记为 $O _ { A }$ .摆轴(挠性杆处于中立位置时的对称轴)记为 $P _ { A }$

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

将式(3.2. 6) 和式 (3.2. 7) 代人上式，略去二阶小量，得

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

将式 (3.2.9) 代人式 (3.2. 剖，并考虑到 $a _ { I } = - F _ { I }$ ，则得

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

其中 gI..gO gp $\pmb { g }$ 在加速度计的输人轴、输出轴和摆轴上的投影值.用于测试的模型为

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

式中.M为作用在摆组件输出轴上的外力矩。应用哥民定理，并将各矢量向摆组件坐标系投影

$$
\left. \frac {\mathrm{d} \boldsymbol {H} _ {A}}{\mathrm{d} t} \right| _ {A} ^ {A} + \boldsymbol {\omega} _ {i A} ^ {A} \times \boldsymbol {H} _ {A} ^ {A} = \boldsymbol {M} ^ {A}
$$

将式 (3.2. 16) 代人上式，得

$$
\left[ \begin{array}{c} \dot {H} _ {A (I A)} \\ \dot {H} _ {A (O A)} \\ \dot {H} _ {A (P A)} \end{array} \right] + \left[ \begin{array}{c c c} 0 & - (\omega_ {P} + \theta_ {O} \omega_ {I}) & \omega_ {O} + \dot {\theta} _ {O} \\ \omega_ {P} + \theta_ {O} \omega_ {I} & 0 & - (\omega_ {I} - \theta_ {O} \omega_ {P}) \\ - (\omega_ {O} + \dot {\theta} _ {O}) & \omega_ {I} - \theta_ {O} \omega_ {P} & 0 \end{array} \right] \left[ \begin{array}{c} H _ {A (I A)} \\ H _ {A (O A)} \\ H _ {A (P A)} \end{array} \right] = \left[ \begin{array}{c} 0 \\ M _ {O} \\ 0 \end{array} \right]
$$

只讨论沿 $O _ { A }$ 轴的方程，即得加速度计的动力学方程:

$$
\dot {H} _ {A (O A)} + \left(\omega_ {P} + \theta_ {O} \omega_ {I}\right) H _ {A (I A)} - \left(\omega_ {I} - \theta_ {O} \omega_ {P}\right) H _ {A (P A)} = M _ {O}
$$

将式(3.2.17)代人上式，经整理并略去二阶小量，得

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

不考虑于扰力矩 $M _ { \mathrm { d } }$ 的影响，则由式(3. 2. 19)可求得加速度 $a _ { _ { I } }$ 的准确值

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

上式右侧第一项称为角加速度误差，第二项称为不等惯性误差，第三、四、五项称为惯性积误差，最后一项称为不等惯性搞合误差。各误差项都有明确的含意，下面对它们作简要说明。

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

## 3. 摆纽件惯性和、误差

摆组件惯性积误差的表达式为

$$
\delta a _ {I 3} = \frac {I _ {I O}}{m L} \left(\dot {\omega} _ {I} + \omega_ {O} \omega_ {P}\right) + \frac {I _ {O P}}{m L} \left(\dot {\omega} _ {P} - \omega_ {I} \omega_ {O}\right) + \frac {I _ {P I}}{m L} \left(\omega_ {P} ^ {2} - \omega_ {I} ^ {2}\right)
$$

该顶误差是由摆组件惯性积 $I _ { I O \setminus I _ { O P } }$ 和 $I _ { P I }$ 在角运动条件下形成离心惯性力矩引起的。其中，$\frac { I _ { I O } } { m L } . \frac { I _ { O P } } { m L } . \frac { I _ { P I } } { m L }$ 称为摆组件惯性积误差系数。

为了减少摆组件惯性积误差，在结构设计中应尽量使摆组件对输入轴、输出轴和摆轴的质量分布对称，以使摆组件对这些轴的惯性积减少到最低限度，摆组件结构经精心设计后，惯性积误差一般是很小的。

## 4. 不等惯性藕合误差

不等惯性藕合误差的表达式为

$$
\delta a _ {I 4} = \frac {I _ {P} - I _ {I}}{m L} (\omega_ {I} ^ {2} - \omega_ {P} ^ {2}) \theta_ {O}
$$

该项误差是由摆组件绕摆轴的转动惯量 $I _ { P }$ 与绕输入轴的转动惯量 $I _ { I }$ ，不相等，并通过加速度计输出角。 $\theta _ { \mathrm { { O } } }$ 。的搞合引起的。由于加速度计工作在力反馈状态，根据式(3.2.9) ，加速度计的稳态输出角。 $\theta _ { O }$ 。与输人加速度的关系为

$$
\theta_ {O} = \frac {m L}{K} a _ {I}
$$

式中 $K { = } C { + } K _ { s } K _ { a } K _ { R } K _ { T }$ '所以

$$
\delta a _ {I 4} = \frac {I _ {P} - I _ {I}}{K} a _ {I} (\omega_ {I} ^ {2} - \omega_ {P} ^ {2})\tag{3.2.22}
$$

式中， $\frac { I _ { P } - I _ { I } } { K }$ 称为不等惯性搞合误差系数。

为了减少不等'惯性搞合误差，一方面应使摆组件转动惯量 $I _ { P }$ 与 $I _ { I }$ ，相等或尽量接近相等，另一方面应尽量提高再平衡回路的增益。由于这两方面的工作都较容易实现，所以不等惯性搞合误差系数一般是很小的.

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

(3.2.24)虽然给出了综合系数与物理参数之间的函数关系，但这些系数仍然在实验室条件下通过测试来确定，测试必须在摇摆台上进行，以产生角速度和角加速度激励。测得这些系数后存人导航计算机，捷联惯导根据陀螺提供的角速度测量值和加速度计提供的加速度测量值，代人式(3.2. 25)所列模型，即可计算出加速度计的动态误差补偿量，用于对加速度计的输出作校正，从而消除由角运动引起的加速度计动态误差。

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

## 第二篇陀螺稳定平台

陀螺具有定轴性和进动性，即具有相对惯性空间指向保持不变的能力和按照要求的规律相对惯性空间旋转的能力，因此可利用陀螺来控制被控对象的角运动。控制角运动包含两个含义一是隔离运载体的角运动对被控对象的角运动的影响，例如船舶上的同步卫星接收天线，当船舶受风浪作用而摇晃时，天线能始终指向同步卫星;二是能使被控对象按指令即要求的规律旋转，指向始终跟踪变动着的方向，例如监测火箭发射的地面光学跟踪系统，光轴始终跟踪观察点与火箭的连线，而该连线的指向在不断地变化着。

以陀螺为敏感元件，能隔离基座的角运动并能使被控对象按指令旋转的机电控制系统称为陀螺稳定平台。此处称谓的平台具有泛指含意，是一种统称。对于不同的使用场合，陀螺稳定平台的精度和被控对象的大小是不一样的，习惯称谓也有所不同。如平台式惯导系统中使用的稳定平台要求精确模拟惯导系统所采用的导航坐标系，精度要求达到角分级甚至角秒级，此类平台习惯上称之为惯性平台。又如夭线、炮塔等稳定系统，被控对象体积和重量都十分庞大，精度要求不必像惯性平台那样高，控制的旋转轴也可能少于三轴，此类稳定平台习惯上称之为陀螺稳定装置。

从上述简单说明中可看出陀螺稳定平台有两个基本功能=一是稳定功能，即对外界干扰起对消作用，如基座角运动是一种外界干扰，在稳定平台与基座的轴承连接处将引起摩擦力矩作用在平台上，或由于几何约束关系台体被强制带动跟随基座一起旋转，稳定平台能自动产生卸荷力矩对消摩擦力矩，同时能产生适当的反旋转对消被基座带动的强制旋转;二是跟踪功能，即能跟踪指令，按要求的角速度旋转，确保平台的坐标轴指向要求的方位。

## 陀螺稳定平台工作原理

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

上式代人式 (4. 1. lOa) ，得

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

代人式 (4.2.3).

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

将式 (4.2.9) 和式 (4.2. 11) 代人式 (4.2. 的，得

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

上式代人式 (4.2.13) ，得

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

## 4. 3 单轴陀螺稳定平台

## 4.3.1 单轴稳定平台的方块图及传递函数

## 1.积分陀螺羊轴稳定平台的传递函数

此处为简单起见，以直接驱动式单轴平台为研究对象。根据图 2. 3. 所示积分陀螺的方块图、式(4.2.2)和式(4.2.7) ，可得积分陀螺单轴平台的闭环方块图如图 4.3.1(a)所示，图中考虑了陀螺输出角变化率 $\dot { \alpha }$ 引起的陀螺力矩对平台稳定轴的影响，环节- $\cdot k _ { u } k _ { a } F ( s )$ 中的负号是由于力矩马达产生的伺服力矩与陀螺输出反号， $k _ { u }$ 为陀螺信号器的传递系数， $k _ { a }$ 为电子网络放大系数， $F ( s )$ 为校正网络的传递函数。

在工程上为使问题得到简化，常突出一些主要影响因素，而忽略一些次要影响因素，此处对4. 3. l( a) 作如下简化:

(1)因 $\dot { \omega } _ { 0 }$ 中。反映了沿陀螺输出轴方向的角加速度对平台稳定轴的影响，而平台稳定轴与陀螺输人轴方向相同，与陀螺输出轴方向正交，所以 $\dot { \omega } _ { 0 }$ 。实质上反映了三轴平台正交轴之间的交叉搞合影响，而交叉搞合是较弱的，此处不予考虑。

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

## 2. 双自由度角位直陀螺羊轴稳定平台的传递函数

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

## 3. 挠性陀螺羊轴稳定平台传递函数

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

## 1.闭坏力矩刚皮

4. 1. 节中讨论了速率陀螺单轴稳定平台在于扰力矩作用下平台的偏差角，稳定回路虽然能对干扰力矩起卸载作用，但仍会引起偏差角，干扰力矩与偏差角的比值是力矩刚度的概念，平台设计中应尽量提高力矩刚度，而力矩刚度是由平台系统的回路参数确定的。

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

## 1.未校正时平台革统存在的问题

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

## 4.3.4 单轴平台系统对干扰和输入的晌应

1.基庄角运动引起的沿平台轴作用的干扰力矩

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

## 三轴陀螺稳定平台

飞机、导弹、舰艇、坦克等运载体在运动过程中的姿态、航向是任意变化的，要对其进行导航或制导控制，需要在运载体内部建立起导航坐标系，导航坐标系可通过两种途径建立起来:一种途径是将陀螺和加速度计直接固联在运载体上，通过对陀螺和加速度计的输出作解算获得姿态矩阵，并作导航解算，这类系统就是捷联式惯导系统，导航坐标系以数学平台形式体现。另一种途径是将陀螺和加速度计安装在用三个环架支承起来的平台台体上，通过控制台体的旋转使陀螺和加速度计的敏感轴始终与要求的导航坐标系重合，这类系统就是平台式惯导系统，导航坐标系以物理平台形式体现。

三轴平台是由单轴平台复合而成的，单轴平台的一些设计理论也适用三轴平台，但在实现几何稳定和空间积分两种工作状态的过程中出现了许多特殊问题，如平台驱动信号的合理分配，基座角运动的搞合和隔离，陀螺输出轴的交叉搞合，三轴平台系统的交叉榈合等，这些问题将在后续内容中逐一介绍。

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

## 5.2.1 角速度辑合关系分析

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

(5.2. 5) 代人式 (5.2.3) ，得

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

式中 ${ \pmb T } _ { 1 }$ 称为环架系统几何关系阵，平台系统通过该矩阵将环架角速度传递给台体， ${ \bf { { T } } } _ { 3 }$ 称为基座角运动的几何约束搞合阵，平台系统通过该阵将基座角速度传递给台体.

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

(5. 2. 13)说明:平台台体的角速度由电机的驱动角速度和基座角速度组成，如果适当控制电机的驱动，就能对消基座的角运动。其中 $\pmb { T } _ { 4 }$ 是 $\mathbf { \dot { \theta } } _ { A }$ 的摩擦约束(或直接带动)榈合阵， ${ \bf { T } } _ { 5 }$ 为 $\pmb { \omega } _ { i b } ^ { b }$ 的摩擦约束(或直接带动)相合阵。

## 5.2.2 环架驱动信号的合理分配一一方位坐标分解器和俯仰正割分解器

1.环架驱动信号不作分配存在的问题

5. 2. 是三轴平台的水平伺服回路示意图，图中陀螺 $G _ { y }$ 的输出信号用来控制横滚环，陀螺 $G _ { x }$ 的输出信号用来控制俯仰环，沿横滚环轴的负向作用有干扰力矩 $M _ { \mathrm { d } y }$ 0

![](images/8f1cacd573b0fb9f5a42ad828d23bfadee9205da88e7e5bdaf9985d3b2b8fd74.jpg)  
5.2.2 环架驱动信号未作分配时的平台伺服回路

假设飞机朝北飞行，如图 5. 2. 2(a)所示，环架处于中立位置。由于干扰力矩 $M _ { \mathrm { d y } }$ 的作用，横滚环绕 $y _ { r }$ 轴的负方向旋转，由于几何约束关系，俯仰环和方位环被带着一起旋转，陀螺 $G _ { y }$ 感测此转动，信号器输出与旋转角速度成正比的电压信号，经放大后加至横滚力矩电机，电机产生与 $M _ { \mathrm { d y } }$ 大小相等，方向相反的卸荷力矩，抵消干扰力矩。而此时，陀螺 $G _ { x }$ 的输入轴 $I _ { x }$ 与横滚环的旋转方向正交，所以俯仰伺服回路并不工作。综上所述，在图 5. 2. 2(a)所示位置，横滚和俯仰伺服回路能正常工作。

假设飞机由朝北飞行经右盘旋后改向朝东飞行，如图 5.2.2(b)所示，环架仍然处于中立位置。由于几何约束关系，横滚环和俯仰环均被顺时针带动转过 $9 0 ^ { \circ }$ ，而方位环沿 $\mathcal { Z } _ { a }$ 轴有伺服回路，方位伺服回路隔离了这一转动而保持原来方位，陀螺 $G _ { y }$ 和 $G _ { \mathcal { s } }$ 的输入轴方向仍保持原来方向。横滚环在 $M _ { \mathrm { d y } }$ 的作用下绕 $y _ { r }$ 的负方向旋转，旋转角速度被陀螺 $G _ { x }$ 感测，输出信号经放大后馈人俯仰力矩电机，在 $\boldsymbol { \mathcal { X } } _ { f }$ 轴上产生相应力矩 $M _ { \mathrm { d } x }$ 。而此时陀螺 $G _ { y }$ 的输人轴 $I _ { y }$ 与 $y _ { r }$ 轴正交，感测不到横滚环的旋转，输出为零，横滚伺服回路不工作，干扰力矩 $M _ { \mathrm { d } y }$ 仍然存在。此时 $M _ { \mathrm { d y } }$ 非但没被抵消，还产生了另外一个干扰力矩 $M _ { \mathrm { d } x }$ 。所以图 5.2.2(b)位置伺服回路不能正常工作。事实上，除图 5.2.2(a)位置外，所有位置都不能正常工作。

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

将式 (5.2. 7) 代人上式，得

$$
\boldsymbol {T} _ {2} = \left[ \begin{array}{c c c} \cos \theta_ {a} & \cos \theta_ {f} \sin \theta_ {a} & 0 \\ - \sin \theta_ {a} & \cos \theta_ {f} \cos \theta_ {a} & 0 \\ 0 & 0 & 1 \end{array} \right] ^ {- 1} = \left[ \begin{array}{c c c} \cos \theta_ {a} & - \sin \theta_ {a} & 0 \\ \sin \theta_ {a} \sec \theta_ {f} & \cos \theta_ {a} \sec \theta_ {f} & 0 \\ 0 & 0 & 1 \end{array} \right]\tag{5.2.15}
$$

所以由陀螺输出至伺服回路输人间的信号分配关系为

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

设基座的角速度为曲 $\pmb { \omega } _ { i b }$ ，由式 (5.2.13) 和式 (5.2.10) 知 $\mathbf { | } \pmb { \omega } _ { i b }$ 通过摩擦祸合和几何约束藕合影响平台的角速度。摩擦搞合通过摩擦力矩影响平台，平台对此干扰的隔离本质上是对摩擦力矩作卸荷，分析方法与上一节介绍的方法相同。对几何约束搞合引起的干扰角速度的隔离本质上是角速度对消，下面对此作详细分析。

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

将式 (5.3. 1)和式 (5. 3. 3) 代人上式，得

$$
\omega_ {f y} ^ {f} = \omega_ {i r z} ^ {r} \sin \theta_ {f} - \omega_ {i r z} ^ {r} \sin \theta_ {f} = 0
$$

由几何约束关系，平台台体(方位环)绕俯仰环 $y _ { f }$ 轴的旋转角速度应与 $\omega _ { f y } ^ { f }$ 相等，也为零。

从上述分析可看出，由几何约束引起的基座角速度搞合是依靠伺服回路产生的伺服角速度对泊的。

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

从单轴平台伺服回路设计介绍中知，当平台的机械参数(如环架的转动惯量)和元件(如陀螺、和力矩电机等)的传递函数和机电参数给定后，要使平台系统的动静态性能达到设计要求，就必须合理地设计电子线路和选择校正网络的形式和参数，这一工作可在频域内进行，即以频率特性作为设计对象，使平台系统具有满足要求的频率特性，也可在复频域内进行，即采用零、极点配置法使平台系统的传递函数具有合适的零、极点。兰轴平台与单轴平台的设计是类似的，不同点在于三轴平台的动力方程是三维向量方程，诸如陀螺仪控制量(指令角速度)、干扰输人量(陀螺漂移、干扰力矩及基座角速度)、输出量(台体的角速度)、误差输出量(平台漂移角速度和平台误差角)等都是三维向量，所以三轴平台实际上是一个多输入多输出系统，输入输出之间存在交叉相合影响，设计兰轴平台要比设计单轴平台的难度高。工程上常采用简化设计方法，即先按单通道设计伺服回路，然后校验三轴间的交叉相合影响及误差大小，看其是否满足系统的设计要求，若不满足，应重新修改。

## 5.5.1 三轴平台的原理方块图

根据图 5. 2. 所示三轴平台的信息处理流程图，可画出三轴平台的原理方块图，如图 5.5.1所示，图中双线箭头表示兰维向量间的信息传递。下面对图中各环节的传递函数进行分析。

![](images/e7a1c0f4371be29930723526e70b60b5cdb26150c895b1844ba6c8b50cedb5f9.jpg)  
5.5.1 兰轴平台原理方块图

1.陀螺组件的传递函数阵和输出轴搞合阵

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

$\mathbf { \cal { Y } } _ { 2 } \left( s \right)$ 反映了陀螺输出之间的稿合关系，所以称之为精合阵。则式(5. 5. 2)可写成

$$
\begin{array}{r l} \alpha_ {g} ^ {a} (s) = & - \mathbf {Y} _ {1} (s) \left[ \omega_ {\mathrm{cmd}} ^ {a} (s) + \varepsilon_ {g} ^ {a} (s) - \omega_ {k i} ^ {a} (s) \right] - \mathbf {Y} _ {1} (s) \mathbf {Y} _ {2} (s) \omega_ {k i} ^ {a} (s) \\ = & - \mathbf {Y} _ {1} (s) \left\{\omega_ {\mathrm{cmd}} ^ {a} (s) + \varepsilon_ {g} ^ {a} (s) - [ \mathbf {I} - \mathbf {Y} _ {2} (s) ] \omega_ {k i} ^ {a} (s) \right\} \end{array}\tag{5.5.5}
$$

若陀螺配置采用图 5.5.3 所示方案，则 $\mathbf { { \cal Y } } _ { 1 } \left( s \right)$ 与式(5. 5. 3)所列相同，捐合阵则成

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

榈合阵

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

## 4. 环架革统的传递函数矩阵

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

分别将式 (5. 2. 7) 、式 (5. 5. 20) 、式 (5. 5. 18) 、式 (5. 5. 16) 、式 (5.2.15)代人上式，得

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

假设基座角速度 $\pmb { \omega } _ { i b }$ 和环架角速度。 $\mathbf { \dot { \theta } } _ { A }$ 的摩擦约束搞合已由平台稳定回路的卸荷作用消除掉，则上图可简化成图 5.5.6 由图得

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

将式 (5. 5. 30) 和式 (5. 5. 1)代人式 (5.5.29) ，得

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

## 5. 6 采用积分陀螺时三轴平台系统的输出轴搞合分析

## 5.6.1 输出轴藕合函数分析

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

现以平台的 通道为例说明指令交叉藕合的影响@由式(5. 6. 1)、式(5.6.2)和式(5.6.3)平台沿 $_ { \mathcal { X } }$ 轴的输出为

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

## 5.6.2 辑合影晌分析

根据式(5.6.4)可画出平台工通道的方块图，如图 5.6.1 所示。

![](images/4fe9ed8355643c4af93d2fb03fa1c1a1a1cb5f16171bad302350ce4be7f367f7.jpg)  
5.6.1 通道方块图

从图 .6.1 看出，要减小指令的交叉藕合影响，应使 $\displaystyle { \dot { \frac { \tau _ { g } } { k _ { g } } } }$ 尺量小，应采用时间常数 $\tau _ { g }$ 很小而陀螺、传递系数 $k _ { g }$ 很大的单自由度陀螺仪，而 $\tau _ { g } = \frac { I _ { 0 } } { D } , k _ { g } = \frac { H } { D }$ ，所以所选陀螺应该是角动量大、浮液阻尼系数大、陀螺组件绕输出轴的转动惯量小

从图 .6.1 还可看出，要使平台正确跟踪指令 $\omega _ { \mathrm { c m d } x }$ '还要求在全频范围内 $\mid F _ { 1 } ( \mathrm { j } \omega ) \mid = 1$ ，下面分析 $F _ { 1 } ( \mathrm { j } \omega )$ 及其影响。将式(5. 6. 5)写成

$$
\begin{array}{r l} F _ {1} (s) & = \frac {1}{1 - P (s)} \\ F _ {1} (\mathrm{j} \omega) & = \frac {1}{1 - P (\mathrm{j} \omega)} \end{array}\tag{5.6.6}
$$

式中

$$
P (s) = \left\{\frac {\tau_ {g} s Y _ {h} (s)}{k _ {g} [ 1 + Y _ {h} (s) ]} \right\} ^ {2}
$$

将式 (5. 5. 22)代人上式，得

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

做出中频段内 $V _ { 1 } ( \mathrm { j } \omega )$ 的幅频特性及相频特性，找出最接近 dB和 $- 1 8 0 ^ { \circ }$ 相位的频率，则在该频率上引起的交叉搞合误差最大，计算出在该频率上 $F _ { 1 } ( \mathrm { j } \omega )$ 1偏离 的最大误差，若不满足要求，应重新设计系统。

三轴平台各通道间的交叉藕合是比较弱的。但为了尽量减小其影响，选用单自由度陀螺作为敏感器件时，应当尽可能选择时间常数小而角动量大的陀螺，同时应适当选择三个陀螺在台体上的方位配置方案。此外，由式(5.5. 的知，当采用双自由度角位置陀螺作为敏感器件时，陀螺仪输出不存在交叉糯合误差，三轴平台各通道间不存在交叉藕合影响，所以从消除交叉糯比去影响的角度来看，兰轴平台采用双自由度角位置陀螺比采用单自由度陀螺有利.

## 5. 7 三轴平台的误差分析

## 5.7.1 三轴平台误差模型

引起三轴平台误差的因素有以下三种

(1)采用单自由度陀螺作为敏感器件时，陀螺力矩引起陀螺输出轴之间的藕合，使平台各通道之间产生藕合误差，引起平台的精合误差。

(2) 当俯仰环角。 $\theta _ { f } \neq 0$ ，即平台并不处于中立位置时，平台各通道之间产生精合误差，引起平台的搞合误差。

(3) 陀螺漂移 ${ \pmb { \varepsilon } } _ { \mathrm { g } }$ 、干扰力矩 $M _ { \mathrm { d } }$ 和基座角速度 $\pmb { \omega } _ { i b }$ 引起平台误差。

对于因素(1) 5.6.2 节中已做过分析，所以此处只讨论后两种因素的影响。

为简化分析，此处假设三轴平台采用双自由度角位置陀螺(或时间常数 $\tau _ { g }$ 很大的挠性陀螺〉作为敏感器件。根据式 (5. 5. 8) 和式 (5. 5. 9)

$$
\mathbf {Y} _ {1} (s) = \frac {1}{s} \mathbf {I}\tag{5.5.8}
$$

$$
\mathbf {Y} _ {2} (s) = \mathbf {0}\tag{5.5.9}
$$

将式(5.5.8)代人式(5. 5. 1)，仿照推导式(5.5.2日的分析方法，三轴平台的前向通道传递函数阵为

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

将式 (5.5.9) 代人式 (5. 5. 33) ，得

$$
\boldsymbol {X} (s) = \boldsymbol {Y} (s)
$$

根据式(5. 5. 34) ，此时平台的角速度输出为

$$
\boldsymbol {\omega} _ {i a} ^ {a} = [ \boldsymbol {I} + \boldsymbol {Y} (s) ] ^ {- 1} [ \boldsymbol {Y} (s) (\boldsymbol {\omega} _ {\mathrm{cmd}} + \boldsymbol {\varepsilon} _ {g}) + \boldsymbol {T} _ {1} \boldsymbol {Y} _ {5} (s) \boldsymbol {M} _ {\mathrm{d}} + \boldsymbol {T} _ {3} \boldsymbol {\omega} _ {i b} ^ {b} ]\tag{5.7.4}
$$

考虑平台各通道间无藕合的情况，即

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

将式 (5.7. 11)代人式 (5.7.10) ，略去关 $\Delta { \bf Y } ( s )$ 及 $\pmb { \varepsilon } _ { \textit { g } }$ 的二阶小量，得三轴平台的输出角速度误差.

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

将式 (5. 7. 5) 、式 (5.7.8) 代人上式，得

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

将式 (5.7.15) 代人式 (5.7.14) ，得

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

达到最大值，所以平台在 $\omega { = } \omega _ { M }$ 处交叉祸合误差达到最大。又由于谐振峰值频率 $\omega _ { M }$ 与系统的开环交截频率 $\omega _ { C }$ 接近相等，而 $\omega _ { C }$ 处开环增益为 $\mathrm { O d B }$ ，即

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

将式(5. 7. 2) 、式 (5.7.3) 代人上式，得常值漂移引起的平台漂移

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

由式(5.7.12) ，于扰力矩引起的平台漂移为

$$
\delta \boldsymbol {\omega} _ {M} (s) = [ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) ] ^ {- 1} \left\{\boldsymbol {I} - \Delta \boldsymbol {Y} (s) [ \boldsymbol {I} + \boldsymbol {Y} ^ {n} (s) ] ^ {- 1} \right\} \boldsymbol {T} _ {1} \left[ \boldsymbol {Y} _ {5} ^ {n} (s) + \Delta \boldsymbol {Y} _ {5} (s) \right] \boldsymbol {M} _ {\mathrm{d}} ^ {A} (s)
$$

将式 (5.7.5) 、式 (5.7.6) 、式 (5.7.8) 和式 (5.7.9)代人上式，得

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

将式(5.7.2) 和式(5. 7. 3) 代人上式，并假设干扰力矩为常值，则

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

4. 由基座角运动稿合引起的平台误差

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

在工程实际中，陀螺单独测试时可达到某一精度，但用在平台系统中后，平台却远远达不到这一精度。造成这一现象的原因之一是陀螺的偏心振动，即陀螺自转引起偏心振动，产生对平台的干扰力矩，此干扰力矩引起平台绕稳定轴的角振动，陀螺偏心振动与平台角振动间的藕合引起陀螺的附加常值漂移，从而引起平台漂移。下面详细分析这一问题的定量关系。

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

式中， $\mathbf { { \cal { L } } } _ { 1 } ^ { a } = \binom { 0 } { 0 } , \mathbf { { \cal { L } } } _ { 2 } ^ { a } = \boxed { - \cal { L } _ { 2 } }$ 。将上述两式和式 (5. 7. 18) 、式 (5. 7. 19) 代人式 (5.7.20) ，得

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

3) 平台角振动与转子偏心振动搞合诱发的平台漂移

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

将式 (5. 7. 24) 代人上述两式，得

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

## (3) 振动藕合诱发的平台漂移

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

(1)由陀螺偏心振动和平台角振动藕合诱发的平台漂移与航向?有关，在某一航向上漂移会达到最大。当 $I _ { f x } = I _ { r y }$ 时，漂移与航向无关。

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

上述诸式代人式(5.2.6) ，不计 ${ \bf \hat { \omega } } _ { \pmb { \omega } }$ 影响时

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

## 章双轴陀螺稳定平台

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

且忽略方位环与俯仰环间的搞合，则平台的方块图如图 6. 1. 所示

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

上式中， $\pmb { \omega } _ { i p } ^ { p } { = } [ \omega _ { X } \quad \omega _ { Y } \quad \omega _ { Z } ] ^ { \mathrm { T } }$ ，并略去了由水平偏差角引起的角速度捐合影响。

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

将式 (6.2.3) 、式 (6.2.4) 、式 (6.2.6) 和式 (6.2.7) 代人上述两式:

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