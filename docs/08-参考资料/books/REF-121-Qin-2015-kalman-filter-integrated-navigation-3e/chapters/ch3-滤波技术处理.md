# 第3章 卡尔曼滤波中的技术处理

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

式(3.10.13)两边对 $b_{i}^{r}$ 取偏导数,得

$$
\begin{array}{r l} \boldsymbol {r} _ {i, k / k - 1} & = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {r} _ {i, k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \frac {\partial}{\partial b _ {i} ^ {r}} (\boldsymbol {\Gamma} _ {k - 1} ^ {r} \boldsymbol {Q} _ {k - 1} ^ {r} \boldsymbol {\Gamma} _ {k - 1} ^ {r \mathrm{T}}) \Bigg | _ {\boldsymbol {b} ^ {r} = \boldsymbol {b}} + \\ & \quad \boldsymbol {\Phi} _ {k / k - 1} \boldsymbol {P} _ {k - 1} ^ {\mathrm{T}} \frac {\partial \boldsymbol {\Phi} _ {k , k - 1} ^ {\mathrm{T}}}{\partial b _ {i} ^ {r}} \Bigg | _ {\boldsymbol {b} ^ {r} = \boldsymbol {b}} + \frac {\partial \boldsymbol {\Phi} _ {k , k - 1} ^ {r}}{\partial b _ {k} ^ {i}} \Bigg | _ {\boldsymbol {b} ^ {r} = \boldsymbol {b}} \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} \end{array}\tag{3.10.21}
$$

式中

$$
\boldsymbol {r} _ {i, 0} = \frac {\partial \boldsymbol {P} _ {0} ^ {P}}{\partial b _ {i} ^ {r}} \Bigg | _ {\boldsymbol {b} ^ {r} = \boldsymbol {b}}\tag{3.10.22}
$$

可以看出,小尺度灵敏度的求解不需要真实系统和量测的具体参数值,只需要模型的参数值。但如果真实的参数值与模型的参数值相差很大,而这又是不了解真实系统的经常有的情况,则小尺度灵敏度就不足以表征实际的灵敏度。因此,小尺度灵敏度的应用有一定的局限性。

## 3.10.2 连续系统的卡尔曼滤波误差分析一般方法

## 1. 方差误差分析

设真实系统和量测为

$$
\dot {\boldsymbol {X}} ^ {r} (t) = \boldsymbol {F} ^ {r} (t) \boldsymbol {X} ^ {r} (t) + \boldsymbol {G} ^ {r} (t) \boldsymbol {w} ^ {r} (t)\tag{3.10.23}
$$

$$
\mathbf {Z} (t) = \mathbf {H} ^ {r} (t) \mathbf {X} ^ {r} (t) + \mathbf {v} ^ {r} (t)\tag{3.10.24}
$$

而滤波器设计中采用的系统模型和量测模型为

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {F} (t) \boldsymbol {X} (t) + \boldsymbol {G} (t) \boldsymbol {w} (t)\tag{3.10.25}
$$

$$
\mathbf {Z} (t) = \mathbf {H} (t) \mathbf {X} (t) + \mathbf {v} (t)\tag{3.10.26}
$$

上述诸方程中的噪声统计特性与离散系统中的说明类似,这里不再详述。根据模型,滤波计算方程为

令

$$
\begin{array}{r l} \dot {\hat {X}} (t) & = F (t) \hat {X} (t) + \bar {K} (t) [ Z (t) - H (t) \hat {X} (t) ] \\ & \quad \tilde {X} ^ {P} (t) \stackrel {\text { def }} {=} X ^ {r} (t) - \hat {X} (t) \\ & \quad \Delta F (t) \stackrel {\text { def }} {=} F ^ {r} (t) - F (t) \\ & \quad \Delta H (t) \stackrel {\text { def }} {=} H ^ {r} (t) - H (t) \end{array}\tag{3.10.27}
$$

则

$$
\begin{array}{r l} \widetilde {\boldsymbol {X}} ^ {p} (t) & = [ \boldsymbol {F} (t) - \overline {{\boldsymbol {K}}} (t) \boldsymbol {H} (t) ] \widetilde {\boldsymbol {X}} ^ {p} (t) + [ \Delta \boldsymbol {F} (t) - \overline {{\boldsymbol {K}}} (t) \Delta \boldsymbol {H} (t) ] \boldsymbol {X} ^ {r} (t) + \\ & \quad \boldsymbol {G} ^ {r} (t) \boldsymbol {w} ^ {r} (t) - \overline {{\boldsymbol {K}}} (t) \boldsymbol {v} ^ {r} (t) \end{array}\tag{3.10.28}
$$

实际估计误差 $\widetilde{\mathbf{X}}^P (t)$ 的微分方程中还包含状态 $X^{r}(t)$ 。因此，为了求 $\widetilde{\mathbf{X}}^P (t)$ 的均方阵 $P^P (t)$ ，定义一个新的状态向量

$$
\boldsymbol {X} ^ {a} (t) \stackrel {\text { def }} {=} \left[ \begin{array}{c} \widetilde {\boldsymbol {X}} ^ {P} (t) \\ \boldsymbol {X} ^ {r} (t) \end{array} \right]
$$

从求 $X^{a}(t)$ 的均方阵求出 $\widetilde{X}^{P}(t)$ 的均方阵。 $X^{a}(t)$ 的状态方程为

$$
\dot {\boldsymbol {X}} ^ {a} = \boldsymbol {F} ^ {a} (t) \boldsymbol {X} ^ {a} (t) + \boldsymbol {w} ^ {a} (t)\tag{3.10.29}
$$

式中

$$
\boldsymbol {F} ^ {a} (t) = \left[ \begin{array}{c c} \boldsymbol {F} (t) - \bar {\boldsymbol {K}} (t) \boldsymbol {H} (t) & \Delta \boldsymbol {F} (t) - \bar {\boldsymbol {K}} (t) \Delta \boldsymbol {H} (t) \\ \boldsymbol {0} & \boldsymbol {F} ^ {r} (t) \end{array} \right]
$$

$$
\boldsymbol {w} ^ {a} (t) = \left[ \begin{array}{c} \boldsymbol {G} ^ {r} (t) \boldsymbol {w} ^ {r} (t) - \bar {\boldsymbol {K}} (t) \boldsymbol {v} ^ {r} (t) \\ \boldsymbol {G} ^ {r} (t) \boldsymbol {w} ^ {r} (t) \end{array} \right]
$$

从上式知 $w^{a}(t)$ 是白噪声向量, 其协方差阵为

$$
\operatorname{Cov} \left[ w ^ {a} (t), w ^ {a} (\tau) \right] = E \left[ w ^ {a} (t) w ^ {a T} (\tau) \right] = T (t) \delta (t - \tau)\tag{3.10.30}
$$

式中

$$
\boldsymbol {T} = \left[ \begin{array}{c c} \boldsymbol {G} ^ {r} (t) \boldsymbol {Q} ^ {r} (t) \boldsymbol {G} ^ {r \mathrm{T}} (t) + \bar {\boldsymbol {K}} (t) \boldsymbol {R} ^ {r} (t) \bar {\boldsymbol {K}} ^ {\mathrm{T}} (t) & \boldsymbol {G} ^ {r} (t) \boldsymbol {Q} ^ {r} (t) \boldsymbol {G} ^ {r \mathrm{T}} (t) \\ \boldsymbol {G} ^ {r} (t) \boldsymbol {Q} ^ {r} (t) \boldsymbol {G} ^ {r \mathrm{T}} (t) & \boldsymbol {G} ^ {r} (t) \boldsymbol {Q} ^ {r} (t) \boldsymbol {G} ^ {r \mathrm{T}} (t) \end{array} \right]\tag{3.10.31}
$$

令

$$
\boldsymbol {P} ^ {a} (t) = E \left[ \boldsymbol {X} ^ {a} (t) \boldsymbol {X} ^ {a T} (t) \right]
$$

则

$$
\begin{array}{r l} \dot {\mathbf {P}} ^ {a} (t) & = \frac {\mathrm{d}}{\mathrm{d} t} E [ \mathbf {X} ^ {a} (t) \mathbf {X} ^ {a \mathrm{T}} (t) ] = \\ & E [ \dot {\mathbf {X}} ^ {a} (t) \mathbf {X} ^ {a \mathrm{T}} (t) + \mathbf {X} ^ {a} (t) \dot {\mathbf {X}} ^ {a \mathrm{T}} (t) ] = \\ & E [ \mathbf {F} ^ {a} (t) \mathbf {X} ^ {a} (t) \mathbf {X} ^ {a \mathrm{T}} (t) + \mathbf {w} ^ {a} (t) \mathbf {X} ^ {a \mathrm{T}} (t) + \mathbf {X} ^ {a} (t) \mathbf {X} ^ {a \mathrm{T}} (t) \mathbf {F} ^ {a \mathrm{T}} (t) + \\ & \mathbf {X} ^ {a} (t) \mathbf {w} ^ {a \mathrm{T}} (t) ] = \mathbf {F} ^ {a} (t) \mathbf {P} ^ {a} (t) + \mathbf {P} ^ {a} (t) \mathbf {F} ^ {a \mathrm{T}} (t) + \\ & E [ \mathbf {w} ^ {a} (t) \mathbf {X} ^ {a \mathrm{T}} (t) + \mathbf {X} ^ {a} (t) \mathbf {w} ^ {a \mathrm{T}} (t) ] \end{array} \tag {3}\tag{3.10.32}
$$

为了求出上式中的均值项,需从式(3.10.29)中解出 $X^{a}(t)$ 。即

$$
\boldsymbol {X} ^ {a} (t) = \boldsymbol {\Phi} ^ {a} (t, t _ {0}) \boldsymbol {X} ^ {a} (t _ {0}) + \int_ {t _ {0}} ^ {t} \boldsymbol {\Phi} ^ {a} (t, \tau) \boldsymbol {w} ^ {a} (\tau) \mathrm{d} \tau\tag{3.10.33}
$$

其中一步转移阵 $\Phi^{a}(t,\tau)$ 由下述微分方程确定

$$
\left\{ \begin{array}{l} \frac {\mathrm{d}}{\mathrm{d} t} [ \boldsymbol {\Phi} ^ {a} (t, \tau) ] = \mathbf {F} ^ {a} (t) \boldsymbol {\Phi} ^ {a} (t, \tau) \\ \boldsymbol {\Phi} ^ {a} (t, t) = \mathbf {I} \end{array} \right.
$$

当 $t \geqslant t_0$ 时， $w^a(t)$ 与 $X^a(t_0)$ 不相关。所以

$$
\begin{array}{r l} & {E [ w ^ {a} (t) X ^ {a \mathrm{T}} (t) + X ^ {a} (t) w ^ {a \mathrm{T}} (t) ] =} \\ & {\int_ {t _ {0}} ^ {t} E [ w ^ {a} (t) w ^ {a \mathrm{T}} (\tau) ] \Phi^ {a \mathrm{T}} (t, \tau) \mathrm{d} \tau + \int_ {t _ {0}} ^ {t} \Phi^ {a} (t, \tau) E [ w ^ {a} (\tau) W ^ {a \mathrm{T}} (t) ] \mathrm{d} \tau =} \\ & {\int_ {t _ {0}} ^ {t} T (t) \Phi^ {a \mathrm{T}} (t, \tau) \delta (t - \tau) \mathrm{d} \tau + \int_ {t _ {0}} ^ {t} \Phi^ {a} (t, \tau) T (\tau) \delta (t - \tau) \mathrm{d} \tau} \end{array}
$$

为求上式中的积分,取

$$
R _ {c} (t - \tau) = \left\{ \begin{array}{l l} \frac {1}{\Delta t}, & t - \frac {\Delta t}{2} \leqslant \tau \leqslant t + \frac {\Delta t}{2} \\ 0, & \text {其余} t \end{array} \right.
$$

则

$$
\begin{array}{r l} \delta (t - \tau) & = \lim _ {\Delta t \to 0} R _ {c} (t - \tau) \\ \int_ {t _ {0}} ^ {t} T (t) \Phi^ {a \mathrm{T}} (t, \tau) \delta (t - \tau) \mathrm{d} \tau & = T (t) \int_ {t _ {0}} ^ {t} \Phi^ {a \mathrm{T}} (t, \tau) \lim _ {\Delta t \to 0} R _ {c} (t - \tau) \mathrm{d} \tau = \\ & T (t) \lim _ {\Delta t \to 0} \int_ {t - \frac {\Delta t}{2}} ^ {t} \Phi^ {a \mathrm{T}} (t, \tau) \frac {1}{\Delta t} \mathrm{d} \tau = \\ & T (t) I \cdot \frac {1}{\Delta (t)} (t - t + \frac {\Delta t}{2}) = \frac {1}{2} T (t) \end{array}
$$

同理可得

$$
\int_ {t _ {0}} ^ {t} \pmb {\Phi} ^ {a} (t, \tau) \pmb {T} (\tau) \delta (t - \tau) \mathrm{d} \tau = \frac {1}{2} \pmb {T} (t)
$$

所以式 $(3.10.32)$ 成

$$
\dot {\boldsymbol {P}} ^ {a} (t) = \boldsymbol {F} ^ {a} (t) \boldsymbol {P} ^ {a} (t) + \boldsymbol {P} ^ {a} (t) \boldsymbol {F} ^ {a T} (t) + \boldsymbol {T} (t)\tag{3.10.34}
$$

令

$$
\boldsymbol {P} ^ {a} (t) = \left[ \begin{array}{c c} \boldsymbol {P} ^ {P} (t) & \boldsymbol {C} ^ {\mathrm{T}} (t) \\ \boldsymbol {C} (t) & \boldsymbol {A} (t) \end{array} \right]
$$

式中

$$
\left. \begin{array}{l} \boldsymbol {P} ^ {P} (t) \stackrel {\text { def }} {=} E [ \widetilde {\boldsymbol {X}} ^ {P} (t) \widetilde {\boldsymbol {X}} ^ {P T} ] \\ \boldsymbol {C} (t) \stackrel {\text { def }} {=} E [ \boldsymbol {X} ^ {r} (t) \widetilde {\boldsymbol {X}} ^ {P T} (t) \\ \boldsymbol {A} (t) \stackrel {\text { def }} {=} E [ \boldsymbol {X} ^ {r} (t) \boldsymbol {X} ^ {r T} (t) ] \end{array} \right\}\tag{3.10.35}
$$

则式 $(3.10.34)$ 可写成

$$
\begin{array}{r l} \left[ \begin{array}{c c} \dot {\boldsymbol {P}} ^ {P} (t) & \dot {\boldsymbol {C}} ^ {\mathrm{T}} (t) \\ \dot {\boldsymbol {C}} (t) & \dot {\boldsymbol {A}} (t) \end{array} \right] = & \\ & \left[ \begin{array}{c c} \boldsymbol {F} (t) - \overline {{\boldsymbol {K}}} (t) \boldsymbol {H} (t) & \Delta \boldsymbol {F} (t) - \overline {{\boldsymbol {K}}} (t) \Delta \boldsymbol {H} (t) \\ \boldsymbol {0} & \boldsymbol {F} ^ {r} (t) \end{array} \right] \left[ \begin{array}{c c} \boldsymbol {P} ^ {P} (t) & \boldsymbol {C} ^ {\mathrm{T}} (t) \\ \boldsymbol {C} (t) & \boldsymbol {A} (t) \end{array} \right] + \\ & \left[ \begin{array}{c c} \boldsymbol {P} ^ {P} (t) & \boldsymbol {C} ^ {\mathrm{T}} (t) \\ \boldsymbol {C} (t) & \boldsymbol {A} (t) \end{array} \right] \left[ \begin{array}{c c} [ \boldsymbol {F} (t) - \overline {{\boldsymbol {K}}} (t) \boldsymbol {H} (t) ] ^ {\mathrm{T}} & \boldsymbol {0} \\ [ \Delta \boldsymbol {F} (t) - \overline {{\boldsymbol {K}}} (t) \Delta \boldsymbol {H} (t) ] ^ {\mathrm{T}} & \boldsymbol {F} ^ {r \mathrm{T}} (t) \end{array} \right] + \\ & \left[ \begin{array}{c c} \boldsymbol {G} ^ {r} (t) \boldsymbol {Q} ^ {r} (t) \boldsymbol {G} ^ {r \mathrm{T}} (t) + \overline {{\boldsymbol {K}}} (t) \boldsymbol {R} ^ {r} (t) \overline {{\boldsymbol {K}}} ^ {\mathrm{T}} (t) & \boldsymbol {G} ^ {r} (t) \boldsymbol {Q} ^ {r} (t) \boldsymbol {G} ^ {r \mathrm{T}} (t) \\ \boldsymbol {G} ^ {r} (t) \boldsymbol {Q} ^ {r} (t) \boldsymbol {G} ^ {r \mathrm{T}} (t) & \boldsymbol {G} ^ {r} (t) \boldsymbol {Q} ^ {r} (t) \boldsymbol {G} ^ {r \mathrm{T}} (t) \end{array} \right] \end{array}
$$

从上式得

$$
\begin{array}{r l} & {\dot {\mathbf {P}} ^ {P} (t) = [ \mathbf {F} (t) - \bar {\mathbf {K}} (t) \mathbf {H} (t) ] \mathbf {P} ^ {P} (t) + \mathbf {P} ^ {P} (t) [ \mathbf {F} (t) - \bar {\mathbf {K}} (t) \mathbf {H} (t) ] ^ {\mathrm{T}} +} \\ & {\qquad [ \Delta \mathbf {F} (t) - \bar {\mathbf {K}} (t) \Delta \mathbf {H} (t) ] \mathbf {C} (t) + \mathbf {C} ^ {\mathrm{T}} (t) [ \Delta \mathbf {F} (t) - \bar {\mathbf {K}} (t) \Delta \mathbf {H} (t) ] ^ {\mathrm{T}} +} \\ & {\qquad \mathbf {G} ^ {r} (t) \mathbf {Q} ^ {r} (t) \mathbf {G} ^ {r \mathrm{T}} (t) + \bar {\mathbf {K}} (t) \mathbf {R} ^ {r} (t) \bar {\mathbf {K}} ^ {\mathrm{T}} (t)} \\ & {\dot {\mathbf {C}} (t) = \mathbf {F} ^ {r} (t) \mathbf {C} (t) + \mathbf {C} (t) [ \mathbf {F} (t) - \bar {\mathbf {K}} (t) \mathbf {H} (t) ] ^ {\mathrm{T}} +} \\ & {\qquad \mathbf {A} (t) [ \Delta \mathbf {F} (t) - \bar {\mathbf {K}} (t) \Delta \mathbf {H} (t) ] ^ {\mathrm{T}} + \mathbf {G} ^ {r} (t) \mathbf {Q} ^ {r} (t) \mathbf {G} ^ {r \mathrm{T}} (t)} \\ & {\dot {\mathbf {A}} (T) = \mathbf {F} ^ {r} (t) \mathbf {A} (t) + \mathbf {A} (t) \mathbf {F} ^ {r \mathrm{T}} (t) + \mathbf {G} ^ {r} (t) \mathbf {Q} ^ {r} (t) \mathbf {G} ^ {r \mathrm{T}} (t)} \end{array}\tag{3.10.36}
$$

(3.10.37)

(3.10.38)

各均方阵的初始值为

$$
\begin{array}{r l} & \boldsymbol {P} ^ {P} (0) = E \{\left[ \boldsymbol {X} ^ {r} (0) - \hat {\boldsymbol {X}} (0) \right] \left[ \boldsymbol {X} ^ {r} (0) - \hat {\boldsymbol {X}} (0) \right] ^ {\mathrm{T}} \} \\ & \boldsymbol {A} (0) = E \left[ \boldsymbol {X} ^ {r} (0) \boldsymbol {X} ^ {r \mathrm{T}} (0) \right] \\ & \boldsymbol {C} (0) = E \{\boldsymbol {X} ^ {r} (0) \left[ \boldsymbol {X} ^ {r} (0) - \hat {\boldsymbol {X}} (0) \right] ^ {\mathrm{T}} \} \end{array}
$$

从式(3.10.38)、式(3.10.37)和式(3.10.36)可以依次解出 $A(t)$ , $C(t)$ , 最后解出 $P^{P}(t)$ 。

## 2. 灵敏度分析

与离散系统相同,大尺度灵敏度为

$$
\mathbf {S} _ {i} (t) \stackrel {\mathrm{def}} {=} \frac {\mathbf {P} ^ {P} (t) - \mathbf {P} (t)}{\Delta b _ {i}}\tag{3.10.39}
$$

小尺度灵敏度为

$$
\boldsymbol {r} _ {i} (t) \stackrel {\text { def }} {=} \frac {\partial \boldsymbol {P} ^ {p} (t)}{\partial b _ {i} ^ {r}} \Bigg | _ {\boldsymbol {b} ^ {r} = \boldsymbol {b}}\tag{3.10.40}
$$

对式(3.10.36)两边求偏导数,并交换微分次序,再令 $b^{r}=b$ ,则可得小尺度灵敏度的微分方程

$$
\begin{array}{r l} \dot {\boldsymbol {r}} _ {i} (t) & = [ \boldsymbol {F} (t) - \overline {{\boldsymbol {K}}} (t) \boldsymbol {H} (t) ] \boldsymbol {r} _ {i} (t) + \boldsymbol {r} _ {i} (t) [ \boldsymbol {F} (t) - \overline {{\boldsymbol {K}}} (t) \boldsymbol {H} (t) ] ^ {\mathrm{T}} + \\ & \frac {\partial [ \boldsymbol {F} ^ {r} (t) - \overline {{\boldsymbol {K}}} (t) \boldsymbol {H} ^ {r} (t) ]}{\partial b _ {i} ^ {r}} \Bigg | _ {b ^ {r} = b} \boldsymbol {P} (t) + \boldsymbol {P} (t) \frac {\partial [ \boldsymbol {F} ^ {r} (t) - \overline {{\boldsymbol {K}}} (t) \boldsymbol {H} ^ {r} (t) ] ^ {\mathrm{T}}}{\partial b _ {i} ^ {r}} \Bigg | _ {b ^ {r} = b} + \\ & \frac {\partial [ \boldsymbol {G} ^ {r} (t) \boldsymbol {Q} ^ {r} (t) \boldsymbol {G} ^ {\mathrm{T}} (t) ]}{\partial b _ {i} ^ {r}} \Bigg | _ {b ^ {r} = b} + \overline {{\boldsymbol {K}}} (t) \frac {\partial \boldsymbol {R} ^ {r} (t)}{\partial b _ {i} ^ {r}} \Bigg | _ {b ^ {r} = b} \overline {{\boldsymbol {K}}} (t) \end{array}\tag{3.10.41}
$$

下面举一简单例子说明求解灵敏度的过程。

例3-7 设飞行器作等高飞行，希望的飞行高度为 $H_0$ ，但实际的飞行高度为 $H^r$ 。用高度表监测飞行高度，高度测量值为 $M^r = h^r H^r + v^r$ ，其中 $E[v^r(t)] = 0, E[v^r(t)v^r(\tau)] = r^r\delta (t - \tau)$ 。取高度偏离值为状态，即 $X^r = H^r - H_0$ 。假定 $E[X^r(0)] = 0, E\{[X^r(0)]^2\} = C_{x_0}$ ，则真实的系统方程和量测方程为

$$
\begin{array}{l} \dot {X} ^ {r} (t) = 0 \\ Z (t) = h ^ {r} X ^ {r} (t) + v ^ {r} (t) \end{array}
$$

而在设计卡尔曼滤器时,选取的系统模型和量测模型为

$$
\begin{array}{l} \dot {X} (t) = 0 \\ Z (t) = h X (t) + v (t) \end{array}
$$

其中 $E[v(t)]=0, E[v(t)v(\tau)]=r\delta(t-\tau)$ ，且 $r \neq r^{r}, h \neq h^{r}$ 。

试对卡尔曼滤波器作关于 r、h 和 $C_{X_{0}}$ 的灵敏度分析。

解 按连续型卡尔曼滤波方程设计滤波器。根据式(2.3.21)，滤波的估计均方误差方程为

即

$$
\frac {\mathrm{d} P (t)}{\mathrm{d} t} = \frac {- h ^ {2} P ^ {2} (t)}{r}   \frac {1}{P ^ {2} (t)} \mathrm{d} P (t) = - \frac {h ^ {2}}{r} \mathrm{d} t
$$

对该式从0到 $t$ 积分，得

$$
P (t) = \frac {r P (0)}{h ^ {2} P (0) t + r}\tag{1}
$$

根据式 $(2.3.20)$ 得滤波增益为

$$
\overline {{{K}}} (t) = \frac {h P (0)}{h ^ {2} P (0) t + r}\tag{2}
$$

又由式(3.10.36)，实际的估计均方误差 $P^{P}(t)$ 方程为

$$
\dot {P} ^ {P} (t) = - \frac {2 h ^ {2} P (0)}{h ^ {2} P (0) t + r} P ^ {P} (t) - \frac {2 \Delta h h P (0)}{h ^ {2} P (0) t + r} C (t) + \left[ \frac {h P (0)}{h ^ {2} P (0) t + r} \right] ^ {2} r ^ {r}\tag{3}
$$

式中

$$
\begin{array}{r l} & \Delta h = h ^ {r} - h \\ & C (t) = E [ X ^ {r} (t) \widetilde {X} ^ {p} (t) ] \end{array}
$$

根据式(3.10.37)， $C(t)$ 满足方程

$$
\dot {C} (t) = - \frac {h ^ {2} P (0)}{h ^ {2} P (0) t + r} C (t) - \frac {h P (0) \Delta h}{h ^ {2} P (0) t + r} A (t)\tag{4}
$$

式中

$$
A (t) = E \{\left[ X ^ {r} (t) \right] ^ {2} \}
$$

又根据式(3.10.38)，A 满足方程

$$
\dot {A} (t) = 0\tag{5}
$$

因为 $E[X^{r}(0)]=0$ ，在滤波器设计中取 $E[X(0)]=0$ ，所以有

$$
\begin{array}{r l} C (0) & = E [ X ^ {r} (0) \widetilde {X} ^ {P} (0) ] = E \{X ^ {r} (0) [ X ^ {r} (0) - \hat {X} (0) ] \} = \\ & \quad E \{[ X ^ {r} (0) ] ^ {2} \} - \hat {X} (0) E [ X ^ {r} (0) ] = C _ {X} (0) \end{array}
$$

$$
A (0) = E \{\left[ X ^ {r} (0) \right] ^ {2} \} = C _ {X} (0)
$$

$$
\begin{array}{r l} P ^ {P} (0) & = E \{[ X ^ {r} (0) - \hat {X} (0) ] ^ {2} \} = E \{(X ^ {r} (0) - E [ X (0) ]) ^ {2} \} = \\ & E \{[ X ^ {r} (0) ] ^ {2} \} = C _ {X} (0) \end{array}
$$

所以由式(5)得

$$
A (t) = C _ {X} (0)
$$

将 $A(t)$ 代入式(4)，则式(4)成为一阶非齐次线性微分方程

$$
\dot {y} (t) + p (t) y (t) = q (t)
$$

其通解为

$$
y (t) = \mathrm{e} ^ {- \int p (t) \mathrm{d} t} [ \int q (t) \mathrm{e} ^ {\int p (t) \mathrm{d} t} \mathrm{d} t + a ]
$$

其中 a 为待定常数。由式(4)的具体参数, 可得

$$
C (t) = \frac {1}{h ^ {2} P (0) t + r} [ - h P (0) \Delta h C _ {X} (0) t + a ]\tag{6}
$$

当 $t = 0$ 时

$$
C (0) = C _ {X} (0) = \frac {a}{r}
$$

所以

$$
a = r C _ {X} (0)
$$

$$
C (t) = \frac {1}{h ^ {2} P (0) t + r} [ - h P (0) \Delta h C _ {X} (0) t + r C _ {X} (0) ]\tag{7}
$$

将 $C(t)$ 代入式(3)，同样求解一阶非齐次线性微分方程，得

$$
\begin{array}{r l} P ^ {P} (t) = & \frac {1}{[ h ^ {2} P (0) t + r ] ^ {2}} \{- \Delta h ^ {2} h ^ {2} P ^ {2} (0) C _ {X} (0) t ^ {2} + [ h ^ {2} P ^ {2} (0) r ^ {r} - \\ & 2 \Delta h h P (0) C _ {X} (0) r ] t + r ^ {2} C _ {X} (0) \} \end{array}\tag{8}
$$

应用式 $(3.10.39)$ 得大尺度灵敏度

$$
S _ {C _ {X} (0)} (t) = \left\{\frac {h ^ {2} P ^ {2} (0) r t + r ^ {2} C _ {X} (0)}{\left[ h ^ {2} P (0) t + r \right] ^ {2}} - \frac {r P (0)}{h ^ {2} P (0) t + r} \right\} \left[ C _ {X} (0) - P (0) \right] ^ {- 1} =
$$

$$
\frac {r ^ {2}}{\left[ h ^ {2} P (0) t + r \right] ^ {2}}\tag{9}
$$

$$
S _ {r} (t) = \left\{\frac {h ^ {2} P ^ {2} (0) r ^ {r} t + r ^ {2} P (0)}{[ h ^ {2} P (0) t + r ] ^ {2}} - \frac {r P (0)}{h ^ {2} P (0) t + r} \right\} (r ^ {r} - r) ^ {- 1} =\tag{10}
$$

$$
S _ {h} (t) = \frac {h ^ {2} \Delta h P ^ {3} (0) t ^ {2} - 2 P ^ {2} (0) r h t}{[ h ^ {2} P (0) t + r ] ^ {2}}\tag{11}
$$

应用式(3.10.41)可求得小尺度灵敏度。也可以根据定义直接从式(8)求取

$$
r _ {C _ {X} (0)} (t) = \frac {\partial P ^ {P} (t)}{\partial C _ {X} (0)} \Bigg | _ {C _ {X} (0) = P (0)} = \frac {r ^ {2}}{[ h ^ {2} P (0) t + r ] ^ {2}}\tag{12}
$$

$$
r _ {r} (t) = \frac {\partial P ^ {P} (t)}{\partial r} \Bigg | _ {r ^ {r} = r} = \frac {h ^ {2} P ^ {2} (0) t}{[ h ^ {2} P (0) t + r ] ^ {2}}\tag{13}
$$

$$
r _ {h} (t) = \frac {\partial P ^ {P} (t)}{\partial h} \Bigg | _ {h ^ {r} = h} = - \frac {2 h P ^ {2} (0) r t}{[ h ^ {2} P (0) t + r ] ^ {2}}\tag{14}
$$

式(9)～式(14)都是当 $P(0)\neq P^{P}(0),r\neq r^{r},h\neq h^{r}$ 时的灵敏度，它们反映了当 $P(0)$ ，r和h取得不准确时，实际滤波均方误差受到影响的程度。从诸表达式可看出， $S_{C_{X}(0)}(t)$ 与 $C_{X}(0)$ 无关，所以 $S_{C_{X}(0)}(t)=r_{C_{X}(0)}(t)$ 。同样 $S_{r}(t)$ 与 $r_{r}(t)$ 也相同，仅 $S_{h}(t)$ 与 $r_{h}(t)$ 不同。

在 $P(0)$ 与 $P^{P}(0)$ （即 $C_{X}(0)$ ）不同，r 与 $r^{r}$ 不同或 h 与 $h^{r}$ 不同的情况下，从两种灵敏度表达式可以看出对实际的估计均方误差的影响，也可看出模型的这些参数 $(P(0), r \text{ 与 } h)$ 与这种影响的关系。从灵敏度表达式还可看出，这种影响随时间而变化。当 t 相当大时， $P(0)$ 不同于 $C_{X}(0)$ 的影响以及 r 不同于 $r^{r}$ 的影响将逐渐消失，而 h 不同于 $h^{r}$ 的影响却始终存在。根据式（1）和式（8）（或式（11）），有

$$
\begin{array}{r l} & \lim _ {t \to \infty} P (t) = 0 \\ & \lim _ {t \to \infty} P ^ {P} (t) = \frac {(\Delta h) ^ {2} C _ {X} (0)}{h ^ {2}} \\ & \lim _ {t \to \infty} S _ {h} (t) = \frac {\Delta h P (0)}{h ^ {2}} \end{array}
$$

或

上述诸式说明:随着滤波时间的增大,计算的估计均方误差趋于零,而实际的估计均方误差却因模型的量测系数 h 有误差而并不趋于零。

方差误差分析和灵敏度分析在设计次优滤波器时，经常用来计算实际的估计均方误差或某些参数的灵敏度，以判别次优滤波器的滤波性能，了解系统参数变化对滤波性能的影响程度，从而更好地选择次优滤波器的结构和参数。必须指出，不论是实际估计均方误差的计算方程，还是灵敏度的计算方程，都是在真实系统为线性系统的前提下获得的。对非线性的真实系统，这些计算方程就不适用了。对这类系统，可以采用计算真实状态估计的方法进行误差分析，即根据真实的系统模型、一组初始状态的随机样本数据，计算出一组状态和量测的动态过程值，并根据量测值和采用的滤波器计算出估计的动态过程值并与真实状态比较，得出一组估计误差的动态过程值。按照上述方法，用不同的随机样本值，可以得出若干组估计误差的动态过程值。取其均方值，得到实际的估计均方误差。这种方法通常称作蒙特-卡洛仿真法。采用的随机样本数越多，所得到的实际的估计均方误差就越准确，但计算量也就越大。

## 3.10.3 模型正确时滤波误差上界分析

设系统和量测模型正确,即对离散系统,有

$$
\boldsymbol {\Phi} _ {k, k - 1} = \boldsymbol {\Phi} _ {k, k - 1} ^ {r}, \quad \boldsymbol {\Gamma} _ {k} = \boldsymbol {\Gamma} _ {l} ^ {r}, \quad \boldsymbol {H} _ {k} = \boldsymbol {H} _ {k} ^ {r}
$$

对连续系统，有

$$
\boldsymbol {F} (t) = \boldsymbol {F} ^ {r} (t), \quad \boldsymbol {G} (t) = \boldsymbol {G} ^ {r} (t), \quad \boldsymbol {H} (t) = \boldsymbol {H} ^ {r} (t)
$$

并设系统噪声方差阵为 $Q_{k}$ ，量测噪声方差阵为 $R_{k}, P_{0}^{P} = Var[X_{0}^{r}]$ ，则根据式（3.10.12）和式（3.10.13），反映真实估计误差和一步预测误差的均方阵为

$$
\boldsymbol {P} _ {k} ^ {P} = (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1} ^ {P} (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) ^ {\mathrm{T}} + \boldsymbol {K} _ {k} \boldsymbol {R} _ {k} ^ {r} \boldsymbol {K} _ {k} ^ {\mathrm{T}}\tag{3.10.42}
$$

$$
\boldsymbol {P} _ {k / k - 1} ^ {p} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {P} _ {k - 1} ^ {P} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {Q} _ {k - 1} ^ {r} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}}\tag{3.10.43}
$$

设计卡尔曼滤波器时,系统模型和量测模型虽然选取正确,但系统噪声方差阵,量测噪声方差阵及初始均方误差阵选取有误,即

$$
\begin{array}{l} {Q _ {k} \neq Q _ {k} ^ {r}} \\ {R _ {k} \neq R _ {k} ^ {r}} \\ {P _ {0} \neq P _ {0} ^ {P} = \mathrm{Var} [ X _ {0} ^ {r} ]} \end{array}
$$

则根据式 $(2.2.4e)$ 和式 $(2.2.4c)$ ，得滤波计算中获得的均方误差阵为

$$
\boldsymbol {P} _ {k} = \left(\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}\right) \boldsymbol {P} _ {k / k - 1} \left(\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}\right) ^ {\mathrm{T}} + \boldsymbol {K} _ {k} \boldsymbol {R} _ {k} \boldsymbol {K} _ {k} ^ {\mathrm{T}}\tag{3.10.44}
$$

$$
\boldsymbol {P} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \boldsymbol {\Gamma} _ {k - 1} \boldsymbol {Q} _ {k - 1} \boldsymbol {\Gamma} _ {k - 1} ^ {\mathrm{T}}\tag{3.10.45}
$$

式(3.10.44)与式(3.10.42)等式两边相减,式(3.10.45)与式(3.10.43)等式两边也相减,并令

$$
\begin{array}{r} \Delta P _ {k} = P _ {k} - P _ {k} ^ {P} \\ \Delta P _ {k / k - 1} = P _ {k / k - 1} - P _ {k / k - 1} ^ {P} \end{array}
$$

得

$$
\begin{array}{r l} \Delta P _ {k} & = (I - K _ {k} H _ {k}) \Delta P _ {k / k - 1} (I - K _ {k} H _ {k}) ^ {\mathrm{T}} + K _ {k} (R _ {k} - R _ {k} ^ {r}) K _ {k} ^ {\mathrm{T}} \\ \Delta P _ {k / k - 1} & = \Phi_ {k, k - 1} \Delta P _ {k - 1} \Phi_ {k, k - 1} ^ {\mathrm{T}} + \Gamma_ {k - 1} (Q _ {k - 1} - Q _ {k - 1} ^ {r}) \Gamma_ {k - 1} ^ {\mathrm{T}} \end{array}\tag{3.10.46}
$$

(3.10.47)

从上述两式可看出，如果滤波器设计中，选择参数时使 $Q_{k} \geqslant Q_{k}^{r}$ 和 $R_{k} \geqslant R_{k}^{r}$ ，则只要 $\Delta P_{k-1} \geqslant 0$ ，必有 $\Delta P_{k/k-1} \geqslant 0$ ，因而也有 $\Delta P_{k} \geqslant 0$ 。根据数学归纳法，可得到如下结论：

在只有 $P_0, Q_k$ 和 $R_k$ 有误差的情况下，如果选取

$$
\left. \begin{array}{l} \boldsymbol {P} _ {0} \geqslant \boldsymbol {P} _ {0} ^ {P} \\ \boldsymbol {Q} _ {k} \geqslant \boldsymbol {Q} _ {k} ^ {r} \\ \boldsymbol {R} _ {k} \geqslant \boldsymbol {R} _ {k} ^ {r} \end{array} \right\}\tag{3.10.48}
$$

则必有

$$
\boldsymbol {P} _ {k} \geqslant \boldsymbol {P} _ {k} ^ {P}
$$

反之，如果选取

$$
\begin{array}{l} \boldsymbol {P} _ {0} \leqslant \boldsymbol {P} _ {0} ^ {P} \\ \boldsymbol {Q} _ {k} \leqslant \boldsymbol {Q} _ {k} ^ {r} \\ \boldsymbol {R} _ {k} \leqslant \boldsymbol {R} _ {k} ^ {r} \end{array}
$$

则必有

$$
\boldsymbol {P} _ {k} \leqslant \boldsymbol {P} _ {k} ^ {P}
$$

因此可进一步得出如下结论：

在只有 $P_0, Q_k$ 和 $R_k$ 有误差的情况下，如果按式(3.10.48)选取滤波参数，并且系统是一致完全可控和一致完全可观测的，则 $P_k$ 有一致的有限上界。因为 $P_k^P \leqslant P_k$ ，所以 $P_k^P$ 一定有一致的有限上界。

以上结论有助于选择模型的噪声方程阵。例如， $P_{0}, Q_{k}$ 和 $R_{k}$ 若无法精确获得，而只知道它们可能的取值范围，则可以采用它们可能的较大值，也就是保守值。这样，只要滤波计算中获得的估计均方误差能满足设计要求，那么实际滤波误差的均方误差必定能满足要求。另外也可以适当选取 $Q_{k}$ 和 $R_{k}$ ，如果系统模型和量测模型满足一致完全可控和一致完全可观测，则 $P_{k}$ 必然有界，而 $P_{k}^{P} \leqslant P_{k}$ 。所以，这种保守设计在一定意义上可以防止实际的估计均方误差阵的

发散。

连续系统也有相同的结论,这里就不再介绍了。

## 3.11 $H_{\infty}$ 滤波 $^{[119]}$

设 $X_{k}$ 为被估计状态,相应的系统方程和量测方程为

$$
\boldsymbol {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {X} _ {k - 1} + \boldsymbol {W} _ {k - 1}\tag{3.11.1a}
$$

$$
\mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k}\tag{3.11.1b}
$$

式中， $W_{k}$ 和 $V_{k}$ 为独立的零均值高斯白噪声，方差阵分别为 $Q_{k}$ 和 $R_{k}$ 。

若 $\Phi_{k,k - 1},H_k,Q_k,R_k$ 和 $X_0$ 的先验信息准确已知，则采用标准卡尔曼算法可以获得 $X_{k}$ 的最小方差估计，估计精度优于任何其它估计。如果并不确切知道 $Q_{k},R_{k}$ 和 $X_0$ 的准确先验信息，例如对系统的不确定性了解不清，则在设计卡尔曼滤波器时应适当增大 $Q_{k}$ 的取值，以增大对实时量测量的利用权重，同时降低对一步预测的利用权重，此法俗称为调谐。但是调谐存在诸多盲目性，无法确定 $Q_{k}$ 究竟增大到多少才能使估计精度达到最佳。并且如果 $\mathbf{W}_k$ 和 $\mathbf{V}_k$ 不是白噪声，或者存在偏置量，则卡尔曼滤波效果会严重恶化，甚至发散。 $H_{\infty}$ 滤波是解决上述问题的有效算法。对 $H_{\infty}$ 滤波的研究始于20世纪80年代[131-133]，MikeGrimble在1988年提出了根据噪声与估计误差之比的上界来设计状态估计器频率响应的方法，最先提出了 $H_{\infty}$ 滤波[134]，但其研究是在频域内进行的，存在诸多设计上的不便。之后的研究都是在时域内进行的，简化了研究工作，并与卡尔曼滤波建立起了内在联系[135-138]。

$H_{\infty}$ 滤波的设计思路是: 在 $Q_{k}, R_{k}$ 和 $P_{0}$ 未知的情况下, 将 $W_{k}, V_{k}, X_{0}$ 的不确定性对估计精度的影响降低到最低程度, 使滤波器在最恶劣的条件下估计误差达到最小。所以 $H_{\infty}$ 滤波可理解成系统存在严重干扰条件下的最优滤波, 滤波鲁棒性是其最显著特点。下面, 根据这一思路详细介绍 $H_{\infty}$ 滤波算法。

## 3.11.1 约束条件下的极值求解

## 1. 静态约束条件下的极值求解

设有某标量函数 $J(X,W)$ ，其中 X 为 n 维向量，W 为 m 维向量， $f(X,W)=0$ 为 n 维向量方程，要求确定出在 $f(X,W)=0$ 约束条件下， $J(X,W)$ 的极值以及对应于该极值的 $X=X^{*}$ 和 $W=W^{*}$ ，习惯上将 $X^{*}$ 和 $W^{*}$ 称为 $J(X,W)$ 的驻点。解决约束条件下极值问题的有效方法是拉格朗日乘数法。具体方法简述如下。

构造增广代价函数

$$
J _ {a} = J + \lambda^ {T} f\tag{3.11.2}
$$

式中， $\lambda$ 为拉格朗日乘数，为 n 维待定向量。

由(3.11.2)式得

$$
\frac {\partial J _ {a}}{\partial \boldsymbol {X} ^ {\mathrm{T}}} = \frac {\partial J}{\partial \boldsymbol {X} ^ {\mathrm{T}}} + \boldsymbol {\lambda} ^ {\mathrm{T}} \frac {\partial f}{\partial \boldsymbol {X} ^ {\mathrm{T}}}\tag{3.11.3a}
$$

$$
\frac {\partial J _ {a}}{\partial \boldsymbol {W} ^ {\mathrm{T}}} = \frac {\partial J}{\partial \boldsymbol {W} ^ {\mathrm{T}}} + \boldsymbol {\lambda} ^ {\mathrm{T}} \frac {\partial \boldsymbol {f}}{\partial \boldsymbol {W} ^ {\mathrm{T}}}\tag{3.11.3b}
$$

$$
\frac {\partial J _ {a}}{\partial \lambda} = f\tag{3.11.3c}
$$

为求解 $J_{a}$ 的驻点，令上述诸式等于零，

$$
\boldsymbol {\lambda} ^ {\mathrm{T}} = - \frac {\partial J}{\partial \boldsymbol {X} ^ {\mathrm{T}}} \left(\frac {\partial \boldsymbol {f}}{\partial \boldsymbol {X} ^ {\mathrm{T}}}\right) ^ {- 1}\tag{3.11.4a}
$$

$$
\frac {\partial J}{\partial \boldsymbol {W} ^ {\mathrm{T}}} - \frac {\partial J}{\partial \boldsymbol {X} ^ {\mathrm{T}}} \left(\frac {\partial \boldsymbol {f}}{\partial \boldsymbol {X} ^ {\mathrm{T}}}\right) ^ {- 1} \frac {\partial \boldsymbol {f}}{\partial \boldsymbol {W} ^ {\mathrm{T}}} = \mathbf {0}\tag{3.11.4b}
$$

$$
f = 0\tag{3.11.4c}
$$

式(3.11.4)即为驻点应满足的必要条件,该式共有 $2n+m$ 个分量方程,联立求解这些方程,可以确定出驻点 $X^{*},W^{*}$ 和拉格朗日乘数 $\lambda$ ,代入(3.11.2)式可确定出 $J_{a}$ 的极值。

式(3.11.4)说明,拉格朗日乘数法将约束条件下的极值求解转化为无约束极值求解,约束方程在式(3.11.3)中被用作求取驻点的已知条件。

例3-8 确定出在 $x - 3 = 0$ 约束条件下 $J(x,u) = \frac{x^2}{2} + xu + u^2 + u$ 的极小值。

解 可采用直观分析法确定该条件极值,也可采用拉格朗日乘数法。

直观分析法

由约束方程得 $x = 3$ ，则 $J(x,u) = \frac{9}{2} +4u + u^2$ ，当 $u = -2$ 时， $J(x,u)$ 达到最小值 $\frac{1}{2}$ ，即 $J_{\mathrm{min}} = \frac{1}{2}$ 。

拉格朗日乘数法

$$
J _ {a} = J + \lambda f = \frac {x ^ {2}}{2} + x u + u ^ {2} + u + \lambda (x - 3)
$$

$$
\frac {\partial J _ {a}}{\partial x} = x + u + \lambda = 0
$$

$$
\frac {\partial J _ {a}}{\partial u} = x + 2 u + 1 = 0
$$

$$
\frac {\partial J _ {a}}{\partial \lambda} = x - 3 = 0
$$

联立求解上述三式，得 $x = 3, u = -2, \lambda = -1, J_{\min} = \frac{1}{2}$ 。

两种方法结果相同。

2. 动态约束条件下的极值求解

设有时变系统

$$
\boldsymbol {X} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {X} _ {k} + \boldsymbol {W} _ {k} \quad k = 0, 1, \dots , N - 1
$$

要求在该 N 个约束方程条件下, 确定下述标量代价函数的极值

$$
J = \psi (\mathbf {X} _ {0}) + \sum_ {k = 0} ^ {N - 1} L _ {k}\tag{3.11.5}
$$

其中 $\psi$ 为 $X_{0}$ 的已知函数， $L_{k}$ 为 $X_{k}$ 和 $W_{k}$ 的已知函数。

构造增广代价函数

$$
J _ {a} = J + \sum_ {k = 0} ^ {N - 1} \boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} (\boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {X} _ {k} + \boldsymbol {W} _ {k} - \boldsymbol {X} _ {k + 1})\tag{3.11.6}
$$

式 $(3.11.5)$ 代入上式,得:

$$
\begin{array}{r l} J _ {a} & = \psi (\mathbf {X} _ {0}) + \sum_ {k = 0} ^ {N - 1} [ L _ {k} + \boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} (\boldsymbol {\Phi} _ {k + 1, k} \mathbf {X} _ {k} + \mathbf {W} _ {k} - \mathbf {X} _ {k + 1}) ] = \\ & \psi (\mathbf {X} _ {0}) + \sum_ {k = 0} ^ {N - 1} [ L _ {k} + \boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} (\boldsymbol {\Phi} _ {k + 1, k} \mathbf {X} _ {k} + \mathbf {W} _ {k}) ] - \sum_ {k = 0} ^ {N - 1} \boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} \mathbf {X} _ {k + 1} = \\ & \psi (\mathbf {X} _ {0}) + \sum_ {k = 0} ^ {N - 1} \mathcal {H} _ {k} - \sum_ {k = 0} ^ {N} \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \mathbf {X} _ {k} + \boldsymbol {\lambda} _ {0} ^ {\mathrm{T}} \mathbf {X} _ {0} = \\ & \psi (\mathbf {X} _ {0}) + \sum_ {k = 0} ^ {N - 1} (\mathcal {H} _ {k} - \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \mathbf {X} _ {k}) - \boldsymbol {\lambda} _ {N} ^ {\mathrm{T}} \mathbf {X} _ {N} + \boldsymbol {\lambda} _ {0} ^ {\mathrm{T}} \mathbf {X} _ {0} \end{array}\tag{3.11.7}
$$

其中

$$
\mathcal {H} _ {k} = L _ {k} + \lambda_ {k + 1} ^ {\mathrm{T}} (\boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {X} _ {k} + \boldsymbol {W} _ {k})\tag{3.11.8}
$$

驻点必须满足的方程为

$$
\frac {\partial J _ {u}}{\partial \mathbf {X} _ {k} ^ {\mathrm{T}}} = \mathbf {0} \quad k = 0, 1, \dots , N\tag{3.11.9a}
$$

$$
\frac {\partial J _ {a}}{\partial \boldsymbol {W} _ {k} ^ {\mathrm{T}}} = \mathbf {0} \quad k = 0, 1, \dots , N - 1\tag{3.11.9b}
$$

$$
\frac {\partial J _ {a}}{\partial \lambda_ {k}} = 0 \quad k = 0, 1, \dots , N\tag{3.11.9c}
$$

由式(3.11.7)和式(3.11.8)，式(3.11.9a)和式(3.11.9b)可写成

$$
\lambda_ {0} ^ {\mathrm{T}} + \frac {\partial \psi (\mathbf {X} _ {0})}{\partial \mathbf {X} _ {0} ^ {\mathrm{T}}} = \mathbf {0}\tag{3.11.10a}
$$

$$
- \lambda_ {N} ^ {\mathrm{T}} = 0\tag{3.11.10b}
$$

$$
\frac {\partial \mathcal {H} _ {k}}{\partial \mathbf {X} _ {k} ^ {\mathrm{T}}} = \pmb {\lambda} _ {k} ^ {\mathrm{T}} \quad k = 1, 2, \dots , N - 1\tag{3.11.10c}
$$

$$
\frac {\partial \mathcal {H} _ {k}}{\partial \pmb {W} _ {k} ^ {\mathrm{T}}} = \pmb {0} \quad k = 0, 1, 2, \dots , N - 1\tag{3.11.10d}
$$

式 $(3.11.9c)$ 即为由不同时刻的系统方程确定的约束方程,分析如下:

由式 $(3.11.7)$ ，

$$
\frac {\partial J _ {a}}{\partial \lambda_ {N}} = - X _ {N} + \frac {\partial \mathcal {H} _ {N - 1}}{\partial \lambda_ {N}} = - X _ {N} + \Phi_ {N, N - 1} X _ {N - 1} + W _ {N - 1} = 0
$$

此即 $t_{N}$ 时刻的系统方程所确定的约束方程。

当 $k=1,2,\cdots,N-1$ 时，

$$
\frac {\partial J _ {a}}{\partial \lambda_ {k}} = - X _ {k} + \frac {\partial \mathcal {H} _ {k - 1}}{\partial \lambda_ {k}} = - X _ {k} + \Phi_ {k, k - 1} X _ {k - 1} + W _ {k - 1} = 0
$$

此即 $k=1,2,\cdots,N-1$ 时刻的系统方程确定的约束方程。

说明：由式(3.11.7)知， $\lambda_0^{\mathrm{T}}X_0$ 实质上并不存在，无论 $\lambda_0$ 任意取值，在该式中， $-\lambda_0^{\mathrm{T}}X_0$ 和 $+\lambda_0^{\mathrm{T}}X_0$ 总互相抵消。所以，只要 $\lambda_0$ 的取值能使式(3.11.7)右侧第二项对 $X_0$ 的偏导数为零，即可确保式(3.11.10a)成立。3.11.2节对此作了验证。

## 3.11.2 $H_{\infty}$ 滤波中的极小值

1. $H_{\infty}$ 滤波的代价函数

设系统方程和量测方程为

$$
\begin{array}{r} \mathbf {X} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \mathbf {X} _ {k} + \mathbf {W} _ {k} \\ \mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k} \end{array}
$$

式中， $W_{k}$ 和 $V_{k}$ 的特性未知，它们有可能是随机噪声，但均值不为零，也有可能是确定性干扰。在这种情况下，卡尔曼滤波是无法解决估计问题的。

假设需要估计的量是状态变量的某种线性组合：

$$
\mathbf {y} _ {k} = \mathbf {T} _ {k} \mathbf {X} _ {k}
$$

其中 $T_{k}$ 满秩。

实现 $H_{\infty}$ 滤波的代价函数为

$$
J _ {1} = \frac {\sum_ {k = 0} ^ {N - 1} \| \mathbf {y} _ {k} - \hat {\mathbf {y}} _ {k} \| _ {\mathbf {s} _ {k}} ^ {2}}{\| \mathbf {X} _ {0} - \hat {\mathbf {X}} _ {0} \| _ {\mathbf {P} _ {0} ^ {- 1}} ^ {2} + \sum_ {k = 0} ^ {N - 1} (\| \mathbf {W} _ {k} \| _ {\mathbf {Q} _ {k} ^ {- 1}} ^ {2} + \| \mathbf {V} _ {k} \| _ {\mathbf {R} _ {k} ^ {- 1}} ^ {2})}\tag{3.11.11}
$$

式中， $\| \mathbf{y}_k - \hat{\mathbf{y}}_k \|_{S_k}^2 = (\mathbf{y}_k - \hat{\mathbf{y}}_k)^{\mathrm{T}} S_k (\mathbf{y}_k - \hat{\mathbf{y}}_k)$ ，即向量 $\mathbf{y}_k - \hat{\mathbf{y}}_k$ 的加权内积， $S_k$ 为该向量的协方差阵，其余向量的加权内积说明同上。

假设代价函数的上界为 $\frac{1}{\theta}$ , 即

$$
J _ {1} <   \frac {1}{\theta}\tag{3.11.12}
$$

式中 $\theta$ 为 $H_{\infty}$ 滤波器的性能上界参数，由设计者选定。

由式(3.11.12)可重新定义代价函数

$$
J = - \frac {1}{\theta} \| \boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0} \| _ {\boldsymbol {P} _ {0} ^ {- 1}} ^ {2} + \sum_ {k = 0} ^ {N - 1} \left[ \| \boldsymbol {y} _ {k} - \hat {\boldsymbol {y}} _ {k} \| _ {\boldsymbol {s} _ {k}} ^ {2} - \frac {1}{\theta} \left(\| \boldsymbol {W} _ {k} \| _ {\boldsymbol {Q} _ {k} ^ {- 1}} ^ {2} + \| \boldsymbol {V} _ {k} \| _ {\boldsymbol {R} _ {k} ^ {- 1}} ^ {2}\right) \right] <   1\tag{3.11.13}
$$

$H_{\infty}$ 滤波所要满足的指标是: 在系统方程 $X_{k+1} = \Phi_{k+1,k} X_k + W_k (k = 0, 1, 2 \cdots, N-1)$ 所确定的约束条件下, 所确定的估计值 $\hat{y}_k$ 使 J 达到最小。(3.11.13) 式中, 分母上各向量的加权内积所用的加权阵分别为 $P_0, Q_k, R_k$ 的逆, 如果选择滤波初值时的误差很大, 系统的不确定性非常严重, 量测误差很大, 则这些阵的逆会很小, 相应的向量加权内积也会很小, 结果是使 J 增大。 $H_{\infty}$ 滤波的设计思路是在 $P_0, Q_k, R_k$ 达到上界时, 寻求相应对策, 使 J 达到最小, 即在干扰极大化背景下的估计误差最小化

$$
J ^ {*} = \min _ {\hat {y} _ {k}} \max _ {(W _ {k}, V _ {k}, X _ {0})} J
$$

由于 $\hat{\mathbf{y}}_k = \mathbf{T}_k\hat{\mathbf{X}}_k,\mathbf{V}_k = \mathbf{Z}_k - \mathbf{H}_k\mathbf{X}_k$ ，所以

$$
\left\| \mathbf {V} _ {k} \right\| _ {\mathbf {R} _ {k} ^ {- 1}} ^ {2} = \left\| \mathbf {Z} _ {k} - \mathbf {H} _ {k} \mathbf {X} _ {k} \right\| _ {\mathbf {R} _ {k} ^ {- 1}} ^ {2}\tag{3.11.14}
$$

$$
\left\| \mathbf {y} _ {k} - \hat {\mathbf {y}} _ {k} \right\| _ {S _ {k}} ^ {2} = \left(\mathbf {X} _ {k} - \hat {\mathbf {X}} _ {k}\right) ^ {\mathrm{T}} \mathbf {T} _ {k} ^ {\mathrm{T}} \mathbf {S} _ {k} \mathbf {T} _ {k} \left(\mathbf {X} _ {k} - \hat {\mathbf {X}} _ {k}\right) = \left\| \mathbf {X} _ {k} - \hat {\mathbf {X}} _ {k} \right\| _ {S _ {k}} ^ {2}\tag{3.11.15}
$$

其中

$$
\widetilde {\boldsymbol {S}} _ {k} = \boldsymbol {T} _ {k} ^ {\mathrm{T}} \boldsymbol {S} _ {k} \boldsymbol {T} _ {k}\tag{3.11.16}
$$

式(3.11.14)、式(3.11.15)代入式(3.11.13)，得

$$
J = - \frac {1}{\theta} \| \boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0} \| _ {\boldsymbol {P} _ {0} ^ {- 1}} ^ {2} + \sum_ {k = 0} ^ {N - 1} \left[ \| \boldsymbol {X} _ {k} - \hat {\boldsymbol {X}} _ {k} \| _ {\dot {\boldsymbol {S}} _ {k}} ^ {2} - \frac {1}{\theta} \left(\| \boldsymbol {W} _ {k} \| _ {\boldsymbol {Q} _ {k} ^ {- 1}} ^ {2} + \| \boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {X} _ {k} \| _ {\boldsymbol {R} _ {k} ^ {- 1}} ^ {2}\right) \right] =
$$

$$
\psi (\pmb {X} _ {0}) + \sum_ {k = 0} ^ {N - 1} L _ {k}\tag{3.11.17}
$$

式中

$$
\psi (\pmb {X} _ {0}) = - \frac {1}{\theta} \| \pmb {X} _ {0} - \hat {\pmb {X}} _ {0} \| _ {\pmb {P} _ {0} ^ {- 1}} ^ {2}\tag{3.11.18}
$$

$$
L _ {k} = \left\| \mathbf {X} _ {k} - \hat {\mathbf {X}} _ {k} \right\| _ {S _ {k}} ^ {2} - \frac {1}{\theta} (\left\| \mathbf {W} _ {k} \right\| _ {Q _ {k} ^ {- 1}} ^ {2} + \left\| \mathbf {Z} _ {k} - \mathbf {H} _ {k} \mathbf {X} _ {k} \right\| _ {\mathbf {R} _ {k} ^ {- 1}} ^ {2})\tag{3.11.19}
$$

构造增广代价函数

$$
J _ {a} = J + \sum_ {k = 0} ^ {N - 1} \frac {2 \lambda_ {k + 1} ^ {\mathrm{T}}}{\theta} (\boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {X} _ {k} + \boldsymbol {W} _ {k} - \boldsymbol {X} _ {k + 1})\tag{3.11.20}
$$

式中， $\frac{2\lambda_{k+1}^{T}}{\theta}$ 为拉格朗日乘数，与(3.11.6)式相比，仅差比例系数 $\frac{2}{\theta}$ ，并不影响极值分析。

式 $(3.11.17)$ 代入式 $(3.11.20)$

$$
J _ {a} = \psi (\mathbf {X} _ {0}) + \sum_ {k = 0} ^ {N - 1} L _ {k} + \sum_ {k = 0} ^ {N - 1} \frac {2 \pmb {\lambda} _ {k + 1} ^ {\mathrm{T}}}{\theta} (\pmb {\Phi} _ {k + 1, k} \mathbf {X} _ {k} + \mathbf {W} _ {k} - \mathbf {X} _ {k + 1}) =
$$

$$
\psi \left(\boldsymbol {X} _ {0}\right) + \sum_ {k = 0} ^ {N - 1} \left[ L _ {k} + \frac {2 \boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}}}{\theta} \left(\boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {X} _ {k} + \boldsymbol {W} _ {k}\right) \right] - \sum_ {k = 0} ^ {N} \frac {2 \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}}}{\theta} \boldsymbol {X} _ {k} + \frac {2 \boldsymbol {\lambda} _ {0} ^ {\mathrm{T}}}{\theta} \boldsymbol {X} _ {0} =
$$

$$
\psi (\mathbf {X} _ {0}) + \sum_ {k = 0} ^ {N - 1} \left(\mathcal {H} _ {k} - \frac {2 \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}}}{\theta} \mathbf {X} _ {k}\right) - \frac {2 \boldsymbol {\lambda} _ {N} ^ {\mathrm{T}}}{\theta} \mathbf {X} _ {N} + \frac {2 \boldsymbol {\lambda} _ {0} ^ {\mathrm{T}}}{\theta} \mathbf {X} _ {0}\tag{3.11.21}
$$

式中

$$
\mathcal {H} _ {k} = L _ {k} + \frac {2 \pmb {\lambda} _ {k + 1} ^ {\mathrm{T}}}{\theta} (\pmb {\Phi} _ {k + 1, k} \pmb {X} _ {k} + \pmb {W} _ {k})\tag{3.11.22}
$$

2. 求解代价函数由 $X_0$ 和 $W_k$ 确定的驻点

根据式 $(3.11.21)$ ，驻点必须满足的方程为

$$
\frac {2 \pmb {\lambda} _ {0} ^ {\mathrm{T}}}{\theta} + \frac {\partial \psi (\pmb {X} _ {0})}{\partial \pmb {X} _ {0} ^ {\mathrm{T}}} = \pmb {0}\tag{3.11.23a}
$$

$$
\frac {\boldsymbol {\lambda} _ {N} ^ {\mathrm{T}}}{\theta} = \mathbf {0}\tag{3.11.23b}
$$

$$
\frac {\partial \mathcal {H} _ {k}}{\partial W _ {k} ^ {\mathrm{T}}} = 0 \quad k = 0, 1, 2, \dots , N - 1\tag{3.11.23c}
$$

$$
\frac {\partial \mathcal {H} _ {k}}{\partial \mathbf {X} _ {k} ^ {\mathrm{T}}} = \frac {2 \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}}}{\theta} k = 0, 1, 2, \dots , N - 1\tag{3.11.23d}
$$

为分析方便，先计算 $\frac{\partial\parallel\mathbf{X}_0 - \hat{\mathbf{X}}_0\parallel^2\mathbf{P}_0^{-1}}{\partial\mathbf{X}_0^{\mathrm{T}}}$ 。记

$$
\boldsymbol {C} = (\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}) ^ {\mathrm{T}}, \quad \boldsymbol {D} = \boldsymbol {P} _ {0} ^ {- 1} (\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0})
$$

则

$$
\frac {\partial \parallel \boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0} \parallel_ {\boldsymbol {P} _ {0} ^ {- 1}} ^ {2}}{\partial \boldsymbol {X} _ {0} ^ {\mathrm{T}}} = \frac {\partial (\boldsymbol {C D})}{\partial \boldsymbol {X} _ {0} ^ {\mathrm{T}}}
$$

根据附录 A 中式(A-7)

$$
\frac {\partial (\boldsymbol {C D})}{\partial \boldsymbol {X} _ {0} ^ {\mathrm{T}}} = \frac {\partial \boldsymbol {C}}{\partial \boldsymbol {X} _ {0} ^ {\mathrm{T}}} (\boldsymbol {I} _ {q \times q} \otimes \boldsymbol {D}) + (\boldsymbol {I} _ {p \times p} \otimes \boldsymbol {C}) \frac {\partial \boldsymbol {D}}{\partial \boldsymbol {X} ^ {\mathrm{T}}}
$$

式中， $\otimes$ 为克朗尼克积，p,q分别为 $X_{0}^{T}$ 的行数和列数，即p=1,q=n。

$$
\frac {\partial \mathbf {C}}{\partial \mathbf {X} _ {0} ^ {\mathrm{T}}} (\mathbf {I} _ {n \times n} \otimes \mathbf {D}) = \frac {\partial \mathbf {X} _ {0} ^ {\mathrm{T}}}{\partial \mathbf {X} _ {0} ^ {\mathrm{T}}} (\mathbf {I} _ {n \times n} \otimes \mathbf {D}) =
$$

$$
\left[ \begin{array}{l l l l} \boldsymbol {e} _ {1} ^ {\mathrm{T}} & \boldsymbol {e} _ {2} ^ {\mathrm{T}} & \dots & \boldsymbol {e} _ {n} ^ {\mathrm{T}} \end{array} \right] \left[ \begin{array}{c c c} \boldsymbol {D} & & \boldsymbol {0} \\ & \ddots & \\ \boldsymbol {0} & & \boldsymbol {D} \end{array} \right] = \left[ \begin{array}{l l l l} \boldsymbol {e} _ {1} ^ {\mathrm{T}} \boldsymbol {D} & \boldsymbol {e} _ {2} ^ {\mathrm{T}} \boldsymbol {D} & \dots & \boldsymbol {e} _ {n} ^ {\mathrm{T}} \boldsymbol {D} \end{array} \right] =
$$

$$
\left[ \boldsymbol {e} _ {1} ^ {\mathrm{T}} \boldsymbol {P} _ {0} ^ {- 1} \left(\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}\right) \quad \boldsymbol {e} _ {2} ^ {\mathrm{T}} \boldsymbol {P} _ {0} ^ {- 1} \left(\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}\right) \quad \dots \quad \boldsymbol {e} _ {n} ^ {\mathrm{T}} \boldsymbol {P} _ {0} ^ {- 1} \left(\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}\right) \right]
$$

式中， $e_{i}$ 为第 i 行元为 1 的单位向量。

记 $\boldsymbol{P}_{0}^{-1}(\boldsymbol{X}_{0}-\hat{\boldsymbol{X}}_{0})=\left[p_{1}\quad p_{2}\quad\cdots\quad p_{n}\right]^{\mathrm{T}}$ ，并注意到 $P_{0}^{-1}$ 为对称阵，则

$$
\boldsymbol {e} _ {i} ^ {\mathrm{T}} \boldsymbol {P} _ {0} ^ {- 1} (\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}) = p _ {i}
$$

$$
\frac {\partial \boldsymbol {C}}{\partial \boldsymbol {X} _ {0} ^ {\mathrm{T}}} (\boldsymbol {I} _ {n \times n} \otimes \boldsymbol {D}) = [ p _ {1} \quad p _ {2} \quad \dots \quad p _ {n} ] = [ \boldsymbol {P} _ {0} ^ {- 1} (\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}) ] ^ {\mathrm{T}} = (\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}) ^ {\mathrm{T}} \boldsymbol {P} _ {0} ^ {- 1}
$$

$$
\left(\boldsymbol {I} _ {p \times p} \otimes \boldsymbol {C}\right) \frac {\partial \boldsymbol {D}}{\partial \boldsymbol {X} _ {0} ^ {\mathrm{T}}} = \left(\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}\right) ^ {\mathrm{T}} \frac {\partial \left[ \boldsymbol {P} _ {0} ^ {- 1} \left(\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}\right) \right]}{\partial \boldsymbol {X} _ {0} ^ {\mathrm{T}}} = \left(\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}\right) ^ {\mathrm{T}} \boldsymbol {P} _ {0} ^ {- 1}
$$

所以

$$
\frac {\partial \parallel \boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0} \parallel_ {\boldsymbol {P} _ {0} ^ {- 1}} ^ {2}}{\partial \boldsymbol {X} _ {0} ^ {\mathrm{T}}} = 2 (\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}) ^ {\mathrm{T}} \boldsymbol {P} _ {0} ^ {- 1}
$$

按同样的分析方法可得其余加权内积的偏微分。

式(3.11.18)代入式(3.11.23a)，并用上述所推关系代入，得

$$
\frac {2 \pmb {\lambda} _ {0} ^ {\mathrm{T}}}{\theta} - \frac {2}{\theta} (\pmb {X} _ {0} - \hat {\pmb {X}} _ {0}) ^ {\mathrm{T}} \pmb {P} _ {0} ^ {- 1} = \pmb {0}
$$

即

$$
\frac {2 \pmb {\lambda} _ {0}}{\theta} - \frac {2}{\theta} \pmb {P} _ {0} ^ {- 1} (\pmb {X} _ {0} - \hat {\pmb {X}} _ {0}) = \pmb {0}
$$

所以

$$
\boldsymbol {X} _ {0} = \hat {\boldsymbol {X}} _ {0} + \boldsymbol {P} _ {0} \lambda_ {0}\tag{3.11.24}
$$

由式(3.11.23b)，得

$$
\lambda_ {N} = 0\tag{3.11.25}
$$

由式(3.11.19)、式(3.11.22)和式(3.11.23c)，得

$$
\frac {2}{\theta} \boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} - \frac {2}{\theta} \boldsymbol {W} _ {k} ^ {\mathrm{T}} \boldsymbol {Q} _ {k} ^ {- 1} = \mathbf {0}
$$

即

$$
\frac {2}{\theta} \lambda_ {k + 1} - \frac {2}{\theta} Q _ {k} ^ {- 1} W _ {k} = 0
$$

所以

$$
W _ {k} = Q _ {k} \lambda_ {k + 1}\tag{3.11.26}
$$

系统方程可写成

$$
\boldsymbol {X} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {X} _ {k} + Q _ {k} \lambda_ {k + 1}\tag{3.11.27}
$$

由式 $(3.11.23d)$ 得

$$
\frac {2}{\theta} \boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} \boldsymbol {\Phi} _ {k + 1, k} + 2 (\boldsymbol {X} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \bar {\boldsymbol {S}} _ {k} - \frac {2}{\theta} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {X} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (- \boldsymbol {H} _ {k}) = \frac {2 \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}}}{\theta}
$$

即

$$
\begin{array}{r l} \frac {\boldsymbol {\lambda} _ {k}}{\theta} & = \frac {1}{\theta} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} \boldsymbol {\lambda} _ {k + 1} + \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {X} _ {k} - \hat {\boldsymbol {X}} _ {k}) + \frac {1}{\theta} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {X} _ {k}) \\ \boldsymbol {\lambda} _ {k} & = \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} \boldsymbol {\lambda} _ {k + 1} + \theta \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {X} _ {k} - \hat {\boldsymbol {X}} _ {k}) + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {X} _ {k}) \end{array}\tag{3.11.28}
$$

为了验证式(3.11.10a)和式(3.11.23a)的正确性,考察式(3.11.21)第二项对 $X_{0}$ 的偏导数:

$$
\frac {\partial}{\partial \mathbf {X} _ {0} ^ {\mathrm{T}}} (\mathcal {H} _ {0} - 2 \frac {\boldsymbol {\lambda} _ {0} ^ {\mathrm{T}}}{\theta} \mathbf {X} _ {0}) = \frac {\partial L _ {0}}{\partial \mathbf {X} _ {0} ^ {\mathrm{T}}} + \frac {2 \boldsymbol {\lambda} _ {1} ^ {\mathrm{T}}}{\theta} \boldsymbol {\Phi} _ {1, 0} - \frac {2 \boldsymbol {\lambda} _ {0} ^ {\mathrm{T}}}{\theta} =
$$

$$
2 \left(\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}\right) ^ {\mathrm{T}} \tilde {\boldsymbol {S}} _ {0} - \frac {1}{\theta} \cdot 2 \left(\boldsymbol {Z} _ {0} - \boldsymbol {H} _ {0} \boldsymbol {X} _ {0}\right) ^ {\mathrm{T}} \boldsymbol {R} _ {0} ^ {- 1} (- \boldsymbol {H} _ {0}) + \frac {2 \boldsymbol {\lambda} _ {1} ^ {\mathrm{T}}}{\theta} \boldsymbol {\Phi} _ {1, 0} - \frac {2 \boldsymbol {\lambda} _ {0} ^ {\mathrm{T}}}{\theta} =
$$

$$
\frac {2}{\theta} \left\{\left[ \boldsymbol {\Phi} _ {1, 0} ^ {\mathrm{T}} \boldsymbol {\lambda} _ {1} + \theta \tilde {\boldsymbol {S}} _ {0} (\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}) + \boldsymbol {H} _ {0} ^ {\mathrm{T}} \boldsymbol {R} _ {0} ^ {- 1} (\boldsymbol {Z} _ {0} - \boldsymbol {H} _ {0} \boldsymbol {X} _ {0}) \right] - \boldsymbol {\lambda} _ {0} \right\} ^ {\mathrm{T}}
$$

而由式(3.11.28)，当 $k = 0$ 时，有：

$$
\boldsymbol {\lambda} _ {0} = \boldsymbol {\Phi} _ {1, 0} ^ {\mathrm{T}} \boldsymbol {\lambda} _ {1} + \theta \tilde {\boldsymbol {S}} _ {0} (\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}) + \boldsymbol {H} _ {0} ^ {\mathrm{T}} \boldsymbol {R} _ {0} ^ {- 1} (\boldsymbol {Z} _ {0} - \boldsymbol {H} _ {0} \boldsymbol {X} _ {0})
$$

该式代入上式，得：

$$
\frac {\partial}{\partial \mathbf {X} _ {0} ^ {\mathrm{T}}} (\mathcal {H} _ {0} - \frac {2 \boldsymbol {\lambda} _ {0} ^ {\mathrm{T}}}{\theta} \mathbf {X} _ {0}) = \mathbf {0}
$$

所以式 $(3.11.23a)$ ，即式 $(3.11.10a)$ 成立。

式(3.11.24)为 $t_{0}$ 时刻真实状态与 $\mu_{0}=\hat{X}_{0}$ 间的关系,为便于推导,假设 $t_{k}$ 时刻也存在此关系

$$
\boldsymbol {X} _ {k} = \boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \lambda_ {k}\tag{3.11.29}
$$

式中 $\mu_{k}$ 和 $P_{k}$ 待定。

式 $(3.11.29)$ 分别代入式 $(3.11.27)$ 和式 $(3.11.28)$ ，得

$$
\begin{array}{r l} \boldsymbol {\mu} _ {k + 1} + \boldsymbol {P} _ {k + 1} \lambda_ {k + 1} & = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {\mu} _ {k} + \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k} \lambda_ {k} + Q _ {k} \lambda_ {k + 1} \\ \boldsymbol {\lambda} _ {k} & = \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} \boldsymbol {\lambda} _ {k + 1} + \theta \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \hat {\boldsymbol {X}} _ {k}) + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} [ \boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} (\boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k}) ] \end{array}\tag{3.11.30}
$$

(3.11.31)

从式(3.11.31)可解得

$$
\boldsymbol {\lambda} _ {k} = \left(\boldsymbol {I} - \theta \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k}\right) ^ {- 1} \left[ \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} \boldsymbol {\lambda} _ {k + 1} + \theta \tilde {\boldsymbol {S}} _ {k} \left(\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}\right) + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \left(\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}\right) \right]\tag{3.11.32}
$$

式 $(3.11.32)$ 代入式 $(3.11.30)$ ，得

$$
\begin{array}{r l} \boldsymbol {\mu} _ {k + 1} + \boldsymbol {P} _ {k + 1} \boldsymbol {\lambda} _ {k + 1} & = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {\mu} _ {k} + \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k} (\boldsymbol {I} - \theta \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k}) ^ {- 1} [ \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} \boldsymbol {\lambda} _ {k + 1} + \\ & \quad \theta \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ] + \boldsymbol {Q} _ {k} \boldsymbol {\lambda} _ {k + 1} \end{array}
$$

即

$$
\begin{array}{r l} \boldsymbol {\mu} _ {k + 1} - \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {\mu} _ {k} - \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k} (\boldsymbol {I} - \theta \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k}) ^ {- 1} [ \theta \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) + \\ & \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ] = \\ & [ - \boldsymbol {P} _ {k + 1} + \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k} (\boldsymbol {I} - \theta \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \left. \boldsymbol {P} _ {k}\right) ^ {- 1} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \boldsymbol {Q} _ {k}) ] \lambda_ {k + 1} \end{array}\tag{3.11.33}
$$

式 $(3.11.33)$ 左右两边均为零时等式仍然成立，左边为零时，有

$$
\boldsymbol {\mu} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {\mu} _ {k} + \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k} (\boldsymbol {I} - \theta \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k}) ^ {- 1} [ \theta \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ]\tag{3.11.34}
$$

其中

$$
\boldsymbol {\mu} _ {0} = \hat {\boldsymbol {X}} _ {0}\tag{3.11.35}
$$

右边为零时，有

$$
\boldsymbol {P} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k} (\boldsymbol {I} - \theta \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k}) ^ {- 1} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \boldsymbol {Q} _ {k}\tag{3.11.36}
$$

记

$$
\widetilde {\boldsymbol {P}} _ {k} = \boldsymbol {P} _ {k} (\boldsymbol {I} - \theta \widetilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k}) ^ {- 1} = (\boldsymbol {P} _ {k} ^ {- 1} - \theta \widetilde {\boldsymbol {S}} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}) ^ {- 1}\tag{3.11.37}
$$

则

$$
\boldsymbol {P} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \widetilde {\boldsymbol {P}} _ {k} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \boldsymbol {Q} _ {k}
$$

(3.11.38)

根据上述分析, 代价函数 J 由 $X_{0}$ 和 $W_{k}$ 确定的驻点按式(3.11.24), 式(3.11.26), 式(3.11.25) 计算

$$
\begin{array}{r l} & X _ {0} = \hat {X} _ {0} + P _ {0} \lambda_ {0} \\ & W _ {k} = Q _ {k} \lambda_ {k + 1} \\ & \lambda_ {N} = 0 \end{array}
$$

$\lambda_{k}(k=N-1,N-2,\cdots,1,0)$ 按式(3.11.32)、式(3.11.36)和式(3.11.34)确定：

$$
\boldsymbol {\lambda} _ {k} = \left(\boldsymbol {I} - \theta \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k}\right) ^ {- 1} \left[ \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} \boldsymbol {\lambda} _ {k + 1} + \theta \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) \right]
$$

$$
\begin{array}{r l} & {\pmb {P} _ {k + 1} = \pmb {\Phi} _ {k + 1, k} \pmb {P} _ {k} (\pmb {I} - \theta \tilde {\pmb {S}} _ {k} \pmb {P} _ {k} + \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H} _ {k} \pmb {P} _ {k}) ^ {- 1} \pmb {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \pmb {Q} _ {k}} \\ & {\qquad \pmb {\mu} _ {0} = \hat {\pmb {X}} _ {0}} \\ & {\pmb {\mu} _ {k + 1} = \pmb {\Phi} _ {k + 1, k} \pmb {\mu} _ {k} + \pmb {\Phi} _ {k + 1, k} \pmb {P} _ {k} (\pmb {I} - \theta \tilde {\pmb {S}} _ {k} \pmb {P} _ {k} + \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H} _ {k} \P_ {k}) ^ {- 1} [ \theta \tilde {\pmb {S}} _ {k} (\pmb {\mu} _ {k} - \hat {\pmb {X}} _ {k}) +} \\ & {\qquad \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} (\pmb {Z} _ {k} - \pmb {H} _ {k} \pmb {\mu} _ {k}) ]} \end{array}
$$

其中求解 $\lambda_{k}$ 为逆序递推，求解 $\pmb{\mu}_{k}$ 为顺序递推，初值分别按式(3.11.25)和式(3.11.35)确定。确定出 $\lambda_0$ 和 $\lambda_{k + 1}$ 后，即可按式(3.11.24）和式(3.11.26）确定驻点 $X_0^*$ 和 $W_{k}^{*}$ ，这说明(3.11.29）式所作的假设是成立的。

3. 求解代价函数由 $\hat{X}_{k}$ 和 $Z_{k}$ 确定的驻点

由式 $(3.11.24)$ 和式 $(3.11.29)$ ，得

$$
\lambda_ {0} = P _ {0} ^ {- 1} (X _ {0} - \hat {X} _ {0})\tag{3.11.39}
$$

$$
\boldsymbol {\lambda} _ {k} = \boldsymbol {P} _ {k} ^ {- 1} (\boldsymbol {X} _ {k} - \boldsymbol {\mu} _ {k})\tag{3.11.40}
$$

所以

$$
\| \boldsymbol {\lambda} _ {0} \| _ {\boldsymbol {P} _ {0}} ^ {2} = \boldsymbol {\lambda} _ {0} ^ {\mathrm{T}} \boldsymbol {P} _ {0} \boldsymbol {\lambda} _ {0} = (\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}) ^ {\mathrm{T}} \boldsymbol {P} _ {0} ^ {- 1} \boldsymbol {P} _ {0} \boldsymbol {P} _ {0} ^ {- 1} (\boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0}) = \| \boldsymbol {X} _ {0} - \hat {\boldsymbol {X}} _ {0} \| _ {\boldsymbol {P} _ {0} ^ {- 1}} ^ {2}\tag{3.11.41}
$$

式 $(3.11.41)$ 和式 $(3.11.29)$ 代入式 $(3.11.17)$ ，得

$$
\begin{array}{r l} & {J = - \frac {1}{\theta} \| \pmb {\lambda} _ {0} \| _ {\pmb {P} _ {0}} ^ {2} +  \sum_ {k = 0} ^ {N - 1} \Big [ \| \pmb {\mu} _ {k} + \pmb {P} _ {k} \pmb {\lambda} _ {k} - \hat {\pmb {X}} _ {k} \| _ {\pmb {S} _ {k}} ^ {2} - \frac {1}{\theta} (\| \pmb {W} _ {k} \| _ {\pmb {Q} _ {k} ^ {- 1}} ^ {2} + \| \pmb {Z} _ {k} - \pmb {H} _ {k} (\pmb {\mu} _ {k} + \pmb {P} _ {k} \pmb {\lambda} _ {k}) \| _ {\pmb {R} _ {k} ^ {- 1}} ^ {2}) \Big ]} \\ & {\text {式中} \| \pmb {W} _ {k} \| _ {\pmb {Q} _ {k} ^ {- 1}} ^ {2} = \pmb {W} _ {k} ^ {\mathrm{T}} \pmb {Q} _ {k} ^ {- 1} \pmb {W} _ {k}.} \end{array}
$$

应用式 $(3.11.26)$ 可计算得

$$
\| \boldsymbol {W} _ {k} \| _ {Q _ {k} ^ {- 1}} ^ {2} = \boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} Q _ {k} Q _ {k} ^ {- 1} Q _ {k} \boldsymbol {\lambda} _ {k + 1} = \boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} Q _ {k} \boldsymbol {\lambda} _ {k + 1} = \| \boldsymbol {\lambda} _ {k + 1} \| _ {Q _ {k}} ^ {2}\tag{3.11.42}
$$

所以

$$
\begin{array}{l} J = - \frac {1}{\theta} \| \boldsymbol {\lambda} _ {0} \| _ {\boldsymbol {P} _ {0}} ^ {2} + \sum_ {k = 0} ^ {N - 1} [ \| \boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \hat {\boldsymbol {X}} _ {k} \| _ {\hat {\boldsymbol {S}} _ {k}} ^ {2} - \\ \frac {1}{\theta} \| \boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} (\boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k}) \| _ {\boldsymbol {R} _ {k} ^ {- 1}} ^ {2} ] - \frac {1}{\theta} \sum_ {k = 0} ^ {N - 1} \| \boldsymbol {\lambda} _ {k + 1} \| _ {\mathcal {Q} _ {k}} ^ {2} \end{array}
$$

由于

$$
\begin{array}{r l} \sum_ {k = 0} ^ {N} \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \sum_ {k = 0} ^ {N - 1} \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} & = \boldsymbol {\lambda} _ {0} ^ {\mathrm{T}} \boldsymbol {P} _ {0} \boldsymbol {\lambda} _ {0} + \sum_ {k = 1} ^ {N} \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \sum_ {k = 0} ^ {N - 1} \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} = \\ & \| \boldsymbol {\lambda} _ {0} \| _ {\boldsymbol {P} _ {0}} ^ {2} + \sum_ {k = 0} ^ {N - 1} (\boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} \boldsymbol {P} _ {k + 1} \boldsymbol {\lambda} _ {k + 1} - \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k}) \end{array}\tag{3.11.43}
$$

而根据式 $(3.11.25)$

$$
\sum_ {k = 0} ^ {N} \lambda_ {k} ^ {\mathrm{T}} P _ {k} \lambda_ {k} - \sum_ {k = 0} ^ {N - 1} \lambda_ {k} ^ {\mathrm{T}} P _ {k} \lambda_ {k} = \lambda_ {N} ^ {\mathrm{T}} P _ {N} \lambda_ {N} + \sum_ {k = 0} ^ {N - 1} (\lambda_ {k} ^ {\mathrm{T}} P _ {k} \lambda_ {k} - \lambda_ {k} ^ {\mathrm{T}} P _ {k} \lambda_ {k}) = 0
$$

所以

$$
- \frac {1}{\theta} \| \boldsymbol {\lambda} _ {0} \| _ {\boldsymbol {P} _ {0}} ^ {2} = \frac {1}{\theta} \sum_ {k = 0} ^ {N - 1} (\boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} \boldsymbol {P} _ {k + 1} \boldsymbol {\lambda} _ {k + 1} - \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k})
$$

上式代入式(3.11.43)，由于 J 为标量，所以有

$$
\begin{array}{l} J = \sum_ {k = 0} ^ {N - 1} \left[ \| \boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \hat {\boldsymbol {X}} _ {k} \| _ {\tilde {\mathbf {s}} _ {k}} ^ {2} - \frac {1}{\theta} \| \boldsymbol {\lambda} _ {k + 1} \| _ {\tilde {\mathbf {Q}} _ {k}} ^ {2} + \frac {1}{\theta} (\boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} \boldsymbol {P} _ {k + 1} \boldsymbol {\lambda} _ {k + 1} - \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k}) - \right. \\ \left. \frac {1}{\theta} \| \boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} (\boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k}) \| _ {\tilde {\mathbf {R}} _ {k} ^ {- 1}} ^ {2} \right] = \sum_ {k = 0} ^ {N - 1} \left[ \| \boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k} \| _ {\tilde {\mathbf {s}} _ {k}} ^ {2} + 2 (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} + \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} + \right. \\ \left. \frac {1}{\theta} \| \boldsymbol {\lambda} _ {k + 1} \| _ {(P _ {k + 1} - Q _ {k})} ^ {2} - \frac {1}{\theta} \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \frac {1}{\theta} \| \boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k} \| _ {\tilde {\mathbf {R}} _ {k} ^ {- 1}} ^ {2} + \right. \\ \left. \frac {2}{\theta} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \frac {1}{\theta} \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} \right] \end{array}
$$

考察上述和式中的第四项,根据式(3.11.38),

$$
\begin{array}{r l} \left\| \boldsymbol {\lambda} _ {k + 1} \right\| _ {(P _ {k + 1} - Q _ {k})} ^ {2} & = \boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} (\boldsymbol {P} _ {k + 1} - Q _ {k}) \boldsymbol {\lambda} _ {k + 1} = \boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} \left(\boldsymbol {\Phi} _ {k + 1, k} \widetilde {\boldsymbol {P}} _ {k} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + Q _ {k} - Q _ {k}\right) \boldsymbol {\lambda} _ {k + 1} = \\ & \quad \boldsymbol {\lambda} _ {k + 1} ^ {\mathrm{T}} \boldsymbol {\Phi} _ {k + 1, k} \widetilde {\boldsymbol {P}} _ {k} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} \boldsymbol {\lambda} _ {k + 1} \end{array} \tag {3.}\tag{3.11.45}
$$

式中， $\tilde{P}_k$ 由式(3.11.37)确定。

又由式(3.11.31)，得

$$
\boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} \boldsymbol {\lambda} _ {k + 1} = \boldsymbol {\lambda} _ {k} - \theta \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \hat {\boldsymbol {X}} _ {k}) - \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} [ \boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} (\boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k}) ]
$$

将该式代入式(3.11.45)，所以

$$
\begin{array}{r l} \left\| \boldsymbol {\lambda} _ {k + 1} \right\| _ {(P _ {k + 1} - Q _ {k})} ^ {2} & = \{\boldsymbol {\lambda} _ {k} - \theta \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \hat {\boldsymbol {X}} _ {k}) - \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} [ \boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} (\boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k}) ] \} ^ {\mathrm{T}} \widetilde {\boldsymbol {P}} _ {k} \{\boldsymbol {\lambda} _ {k} - \\ & \quad \theta \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \hat {\boldsymbol {X}} _ {k}) - \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} [ \boldsymbol {Z} _ {h} - \boldsymbol {H} _ {k} (\boldsymbol {\mu} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k}) ] \} = \\ & \quad \{\boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} (\boldsymbol {I} - \theta \boldsymbol {P} _ {k} \tilde {\boldsymbol {S}} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}) - \theta (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \tilde {\boldsymbol {S}} _ {k} - \\ & \quad (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}) \widehat {\boldsymbol {P}} _ {k} \{\boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} (\boldsymbol {I} - \theta \boldsymbol {P} _ {k} \tilde {\boldsymbol {S}} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}) = \\ & \quad \theta (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \tilde {\boldsymbol {S}} _ {k} - (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}) ^ {\mathrm{T}} \end{array}
$$

由式 $(3.11.37)$

$$
\boldsymbol {I} - \theta \boldsymbol {P} _ {k} \tilde {\boldsymbol {S}} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} = \boldsymbol {P} _ {k} \tilde {\boldsymbol {P}} _ {k} ^ {- 1}
$$

所以

$$
\begin{array}{r l} \left\| \boldsymbol {\lambda} _ {k + 1} \right\| _ {(P _ {k + 1} - Q _ {k})} ^ {2} & = \left[ \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \widetilde {\boldsymbol {P}} _ {k} ^ {- 1} - \theta (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \widetilde {\boldsymbol {S}} _ {k} - (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \right] \widetilde {\boldsymbol {P}} _ {k} \\ & \left[ \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \widetilde {\boldsymbol {P}} _ {k} ^ {- 1} - \theta (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \widetilde {\boldsymbol {S}} _ {k} - (\boldsymbol {Z} k - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \right] ^ {\mathrm{T}} = \\ & \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \widetilde {\boldsymbol {P}} _ {k} ^ {- 1} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \theta (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \widetilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \\ & \theta \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \widetilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) + \theta^ {2} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \widetilde {\boldsymbol {S}} _ {k} \widetilde {\boldsymbol {P}} _ {k} \widetilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) + \\ & \theta (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k} \widetilde {\mathbf {P}} _ {k} \widetilde {\mathbf {S}} _ {k} (\boldsymbol {\mu} _ {k} - \hat {\mathbf {X}} _ {k}) - \boldsymbol {\lambda} ^ {\mathrm{T}} \mathbf {P} _ {k} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \boldsymbol {\mu} _ {k}) + \\ & \theta (\boldsymbol {\mu} _ {k} - \hat {\mathbf {X}} _ {k}) ^ {\mathrm{T}} \widetilde {\mathbf {S}} _ {k} \widetilde {\mathbf {P}} _ {k} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \boldsymbol {\mu} _ {k}) + \\ & (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k} \widetilde {\mathbf {P}} _ {k} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \boldsymbol {\mu} _ {k}) \end{array}
$$

该式中各项都是标量,转置后不变,所以

$$
\begin{array}{r l} \left\| \boldsymbol {\lambda} _ {k + 1} \right\| _ {(P _ {k + 1} - Q _ {k})} ^ {2} & = \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \widetilde {\boldsymbol {P}} _ {k} ^ {- 1} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - 2 \theta (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \widetilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - 2 (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} + \\ & \quad \theta^ {2} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \widetilde {\boldsymbol {S}} _ {k} \widetilde {\boldsymbol {P}} _ {k} \widetilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) + 2 \theta (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \widetilde {\boldsymbol {S}} _ {k} \widetilde {\boldsymbol {P}} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \\ & \quad \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) + (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \widetilde {\boldsymbol {P}} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) \end{array}
$$

由式 $(3.11.37)$ ，得

$$
\widetilde {\boldsymbol {P}} _ {k} ^ {- 1} = \boldsymbol {P} _ {k} ^ {- 1} (\boldsymbol {I} - \theta \boldsymbol {P} _ {k} \widetilde {\boldsymbol {S}} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k})\tag{3.11.47}
$$

所以

$$
\begin{array}{r l} \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \widetilde {\boldsymbol {P}} _ {k} ^ {- 1} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} & = \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} (\boldsymbol {I} - \theta \boldsymbol {P} _ {k} \widetilde {\boldsymbol {S}} _ {k} + \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} = \\ & \quad \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \theta \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \widetilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} + \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} \end{array}
$$

代入式 $(3.11.46)$ ，得

$$
\begin{array}{r l} \parallel \boldsymbol {\lambda} _ {k + 1} \parallel_ {(P _ {k + 1} - Q _ {k})} ^ {2} & = \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \theta \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \widetilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} + \boldsymbol {\lambda} _ {k} ^ {\mathrm{T}} \boldsymbol {P} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - \\ & 2 \theta (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} - 2 (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k} \boldsymbol {\lambda} _ {k} + \\ & \theta^ {2} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \tilde {\boldsymbol {S}} _ {k} \tilde {\boldsymbol {P}} _ {k} \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) + 2 \theta (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \tilde {\boldsymbol {S}} _ {k} \tilde {\boldsymbol {P}} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \\ & \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) + (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \tilde {\boldsymbol {P}} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) \end{array}\tag{3.11.48}
$$

式 $(3.11.48)$ 代入式 $(3.11.44)$ ，得

$$
\begin{array}{l} J = \sum_ {k = 0} ^ {N - 1} \left[ (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) - \frac {1}{\theta} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) + \right. \\ \left. \theta (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \tilde {\boldsymbol {S}} _ {k} \tilde {\boldsymbol {P}} _ {k} \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) + 2 (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \tilde {\boldsymbol {S}} _ {k} \tilde {\boldsymbol {P}} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) + \right. \\ \left. \frac {1}{\theta} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \tilde {\boldsymbol {P}} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) \right] = \\ \sum_ {k = 0} ^ {N - 1} \left[ (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} (\tilde {\boldsymbol {S}} _ {k} + \theta \tilde {\boldsymbol {S}} _ {k} \tilde {\boldsymbol {P}} _ {k} \tilde {\boldsymbol {S}} _ {k}) (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) + 2 (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol {X}} _ {k}) ^ {\mathrm{T}} \tilde {\boldsymbol {S}} _ {k} \tilde {\boldsymbol {P}} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {K} _ {k} ^ {- 1} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) + \right. \\ \left. \frac {1}{\theta} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) ^ {\mathrm{T}} (\boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \tilde {\boldsymbol {P}} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} - \boldsymbol {R} _ {k} ^ {- 1}) (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k}) \right] \\ \frac {\partial J}{\partial \hat {\boldsymbol {X}} _ {k}} = 2 (\tilde {\boldsymbol {S}} _ {k} + \theta \tilde {\boldsymbol {S}} _ {k} \tilde {\boldsymbol {P}} _ {k} \tilde {\boldsymbol {S}} _ {k}) (\hat {\boldsymbol {X}} _ {k} - \boldsymbol {\mu} _ {k}) + 2 \tilde {\boldsymbol {S}} _ {k} \tilde {\boldsymbol {P}} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} (\boldsymbol {H} _ {k} \boldsymbol {\mu} _ {k} - \boldsymbol Z _ {k}) = 0 \\ \frac {\partial J}{\partial Z _ {k}} = \frac 2{\theta} (\boldsymbol R _ {k} ^ {- 1} \boldsymbol H _ {k} \tilde {\boldsymbol P} _ {k} \boldsymbol H _ {k} ^ {\mathrm{T}} \boldsymbol R _ {k} ^ {- 1} - \boldsymbol R _ {k} ^ {- 1}) (\boldsymbol Z _ {k} - \boldsymbol H _ {k} \boldsymbol {\mu} _ {k}) + 2 \boldsymbol R _ {k} ^ {- 1} \boldsymbol H _ {k} \tilde {\boldsymbol P} _ {k} \tilde {\boldsymbol S} _ {k} (\boldsymbol {\mu} _ {k} - \hat {\boldsymbol X} _ {k}) = 0 \end{array}\tag{3.11.50}
$$

从式 $(3.11.49)$ 和式 $(3.11.50)$ 可得

$$
\hat {X} _ {k} = \mu_ {k}\tag{3.11.51a}
$$

$$
\mathbf {Z} _ {k} = \mathbf {H} _ {k} \boldsymbol {\mu} _ {k}\tag{3.11.51b}
$$

根据式 $(3.11.49)$ 和式 $(3.11.37)$

$$
\frac {\partial^ {2} J}{\partial \hat {\boldsymbol {X}} _ {k} \partial \hat {\boldsymbol {X}} _ {k} ^ {\mathrm{T}}} = 2 (\tilde {\boldsymbol {S}} _ {k} + \theta \tilde {\boldsymbol {S}} _ {k} \tilde {\boldsymbol {P}} _ {k} \tilde {\boldsymbol {S}} _ {k}) = 2 [ \tilde {\boldsymbol {S}} _ {k} + \theta \tilde {\boldsymbol {S}} _ {k} (\boldsymbol {P} _ {k} ^ {- 1} - \theta \tilde {\boldsymbol {S}} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}) ^ {- 1} \tilde {\boldsymbol {S}} _ {k} ]
$$

从上式可看出，只要 $\theta$ 取得足够小，则总能保证 $\frac{\partial^2J}{\partial\hat{X}_k\partial\hat{X}_k^{\mathrm{T}}}$ 正定，即

$$
\boldsymbol {P} _ {k} ^ {- 1} - \theta \tilde {\boldsymbol {S}} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} > 0\tag{3.11.52}
$$

从而使 $\hat{X}_{k}=\mu_{k}$ 时代价函数 J 达到最小。由(3.11.12)式知， $\theta$ 取得足够小意味着代价函数 $J_{1}$ 的上界定得较宽，因此参数 $\theta$ 与代价函数的上界有关，应该定得适当小，定得过大，则在 $\hat{X}_{k}=\mu_{k}$ 处代价函数 J 会出现最大值。

式 $(3.11.51)$ 代入式 $(3.11.34)$ ，得

其中

$$
\begin{array}{r l} & {\hat {\mathbf {X}} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \hat {\mathbf {X}} _ {k} + \boldsymbol {\Phi} _ {k + 1, k} \mathbf {K} _ {k} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k})} \\ & {\mathbf {K} _ {k} = \mathbf {P} _ {k} (\mathbf {I} - \theta \tilde {\mathbf {S}} _ {k} \mathbf {P} _ {k} + \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k} \mathbf {P} _ {k}) ^ {- 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1}} \end{array}\tag{3.11.53}
$$

(3.11.54)

根据式 $(3.11.50)$

$$
\frac {\partial^ {2} J}{\partial \mathbf {Z} _ {k} \partial \mathbf {Z} _ {k} ^ {\mathrm{T}}} = \frac {2}{\theta} (\mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k} \widetilde {\mathbf {P}} _ {k} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} - \mathbf {R} _ {k} ^ {- 1}) = \frac {2}{\theta} \mathbf {R} _ {k} ^ {- 1} (\mathbf {H} _ {k} \widetilde {\mathbf {P}} _ {k} \mathbf {H} _ {k} ^ {\mathrm{T}} - \mathbf {R} _ {k}) \mathbf {R} _ {k} ^ {- 1}
$$

从上式可看出, 如果 $R_{k}$ 取得过大, 使 $H_{k}\widetilde{P}_{k}H_{k}^{T}-R_{k}$ 为负定阵, 则在 $Z_{k}=H_{k}\mu_{k}$ 处代价函数 J 会出现极大值, 要使之成为极小值, 需有

$$
\boldsymbol {H} _ {k} \widetilde {\boldsymbol {P}} _ {k} \boldsymbol {H} _ {k} ^ {\mathrm{T}} - \boldsymbol {R} _ {k} > 0
$$

式 $(3.11.37)$ 代入上式，得

$$
\boldsymbol {H} _ {k} \boldsymbol {P} _ {k} (\boldsymbol {I} - \theta \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k}) ^ {- 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} - \boldsymbol {R} _ {k} > 0\tag{3.11.55}
$$

## 3.11.3 $H_{\infty}$ 滤波算法

1. 离散型 $H_{\infty}$ 滤波算法

综合以上讨论,整理出离散型 $H_{\infty}$ 滤波算法如下。

(1) 滤波处理对象

$$
\begin{array}{r l} \mathbf {X} _ {k + 1} & = \boldsymbol {\Phi} _ {k + 1, k} \mathbf {X} _ {k} + \mathbf {W} _ {k} \\ \mathbf {Z} _ {k} & = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k} \\ \mathbf {y} _ {k} & = \mathbf {T} _ {k} \mathbf {X} _ {k} \end{array}
$$

式中， $y_{k}$ 是被估计量， $W_{k}$ 和 $V_{k}$ 不一定是白噪声。

(2) 取代价函数

$$
J _ {1} = \frac {\sum_ {k = 0} ^ {N - 1} \| \mathbf {y} _ {k} - \hat {\mathbf {y}} _ {k} \| _ {s _ {k}} ^ {2}}{\| \mathbf {X} _ {0} - \hat {\mathbf {X}} _ {0} \| _ {P _ {0} ^ {- 1}} ^ {2} + \sum_ {k = 0} ^ {N - 1} (\| \mathbf {W} _ {k} \| _ {Q _ {k} ^ {- 1}} ^ {2} + \| \mathbf {V} _ {k} \| _ {R _ {k} ^ {- 1}} ^ {2})}\tag{3.11.11}
$$

选取 $P_{0}, Q_{k}, R_{k}$ 和 $S_{k}$ 时，必须保证这些矩阵对称和正定。

(3) 滤波算法

$$
\tilde {\boldsymbol {S}} _ {k} = \boldsymbol {T} _ {k} ^ {\mathrm{T}} \boldsymbol {S} _ {k} \boldsymbol {T} _ {k}\tag{3.11.16}
$$

$$
\hat {\mathbf {X}} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \hat {\mathbf {X}} _ {k} + \boldsymbol {\Phi} _ {k + 1, k} \mathbf {K} _ {k} (\mathbf {Z} _ {k} - \mathbf {H} _ {k} \hat {\mathbf {X}} _ {k})\tag{3.11.53}
$$

$$
\boldsymbol {K} _ {k} = \boldsymbol {P} _ {k} (\boldsymbol {I} - \theta \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k}) ^ {- 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1}\tag{3.11.54}
$$

$$
\boldsymbol {P} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k} (\boldsymbol {I} - \theta \tilde {\boldsymbol {S}} _ {k} \boldsymbol {P} _ {k} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k}) ^ {- 1} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \boldsymbol {Q} _ {k}\tag{3.11.36}
$$

式中 $k=0,1,2,\cdots$ 。

选取 $\theta$ 时需保证：

$$
\begin{array}{r} {\pmb {P} _ {k} ^ {- 1} - \theta \tilde {\pmb {S}} _ {k} + \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H} _ {k} > 0} \\ {\pmb {H} _ {k} \pmb {P} _ {k} (\pmb {I} - \theta \tilde {\pmb {S}} _ {k} \pmb {P} _ {k} + \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H} _ {k} \pmb {P} _ {k}) ^ {- 1} \pmb {H} _ {k} ^ {\mathrm{T}} - \pmb {R} _ {k} > 0} \end{array}\tag{3.11.52}
$$

(3.11.55)

2. 连续型 $H_{\infty}$ 滤波算法

设有连续型系统方程和量测方程

$$
\begin{array}{r l} \dot {\mathbf {X}} (t) & = \mathbf {F} (t) \mathbf {X} (t) + \mathbf {B} (t) \mathbf {u} (t) + \mathbf {W} (t) \\ \mathbf {Z} (t) & = \mathbf {H} (t) \mathbf {X} (t) + \mathbf {V} (t) \end{array}
$$

被估计量为

$$
\mathbf {y} (t) = \mathbf {T} (t) \mathbf {X} (t)
$$

代价函数选为

$$
J _ {1} = \frac {\int_ {0} ^ {T} \| \mathbf {y} (t) - \hat {\mathbf {y}} (t) \| _ {s} ^ {2} \mathrm{d} t}{\| \mathbf {X} (0) - \hat {\mathbf {X}} (0) \| _ {P _ {0} ^ {- 1}} ^ {2} + \int_ {0} ^ {T} (\| \mathbf {W} \| _ {q ^ {- 1}} ^ {2} + \| \mathbf {V} \| _ {r ^ {- 1}} ^ {2}) \mathrm{d} t}\tag{3.11.56}
$$

对应的 $H_{\infty}$ 滤波方程为

$$
\dot {\boldsymbol {P}} (t) = \boldsymbol {F} (t) \boldsymbol {P} (t) + \boldsymbol {P} (t) \boldsymbol {F} ^ {\mathrm{T}} (t) + \boldsymbol {q} - \boldsymbol {K} (t) \boldsymbol {H} (t) \boldsymbol {P} (t) + \theta \boldsymbol {P} (t) \boldsymbol {T} ^ {\mathrm{T}} (t) \boldsymbol {S T} (t) \boldsymbol {P} (t)\tag{3.11.57a}
$$

$$
\boldsymbol {K} (t) = \boldsymbol {P} (t) \boldsymbol {H} (t) \boldsymbol {r} ^ {- 1}\tag{3.11.57b}
$$

$$
\dot {\hat {\mathbf {X}}} (t) = \mathbf {F} (t) \hat {\mathbf {X}} (t) + \mathbf {B} (t) \mathbf {u} (t) + \mathbf {K} (t) [ \mathbf {Z} (t) - \mathbf {H} (t) \hat {\mathbf {X}} (t) ]\tag{3.11.57c}
$$

$$
\hat {\mathbf {y}} (t) = \mathbf {T} (t) \hat {\mathbf {X}} (t)\tag{3.11.57d}
$$

3. $H_{\infty}$ 滤波与卡尔曼滤波的比较

为了揭示卡尔曼滤波与 $H_{\infty}$ 滤波的内在联系，对卡尔曼滤波一步预测基本方程（见 2.2.4 节）作恒等变形。

$$
\hat {\boldsymbol {X}} _ {k + 1 / k} = \boldsymbol {\Phi} _ {k + 1, k} \hat {\boldsymbol {X}} _ {k / k - 1} + \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {K} _ {k} (\boldsymbol {Z} _ {k} - \boldsymbol {H} _ {k} \hat {\boldsymbol {X}} _ {k / k - 1})\tag{3.11.58a}
$$

$$
\boldsymbol {K} _ {k} = \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}) ^ {- 1}\tag{3.11.58b}
$$

$$
\boldsymbol {P} _ {k / k - 1} = \boldsymbol {\Phi} _ {k, k - 1} \boldsymbol {P} _ {k - 1} \boldsymbol {\Phi} _ {k, k - 1} ^ {\mathrm{T}} + \boldsymbol {Q} _ {k - 1}\tag{3.11.58c}
$$

$$
\boldsymbol {P} _ {k} = (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}) \boldsymbol {P} _ {k / k - 1}\tag{3.11.58d}
$$

上述方程对应的滤波对象如式 $(3.11.1)$ 所列。

根据矩阵反演公式，

$$
\left(\boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} + \boldsymbol {R} _ {k}\right) ^ {- 1} = \boldsymbol {R} _ {k} ^ {- 1} - \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \left(\boldsymbol {I} + \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}\right) ^ {- 1} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1}\tag{3.11.59}
$$

所以

$$
\begin{array}{r l} & {\mathbf {K} _ {k} = \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} (\mathbf {H} _ {k} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} + \mathbf {R} _ {k}) ^ {- 1} =} \\ & {\quad \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} - \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k} (\mathbf {I} + \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k}) ^ {- 1} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} =} \\ & {\quad [ \mathbf {I} - \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k} (\mathbf {I} + \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R } _ {k} ^ {- 1} \mathbf {H} _ {k}) ^ {- 1} ] \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} =} \\ & {\quad [ (\mathbf {I} + \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k}) - \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k}) ] (\mathbf {I} + \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k}) ^ {- 1} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathcal {R} _ {k} ^ {- 1}} \\ & {\quad (\mathbf {I} + \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k}) ^ {- 1} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbb {R} _ {k} ^ {- 1}} \\ & {\quad P _ {k + 1 / k} = \boldsymbol {\Phi_ {k + 1, k}} P _ {k} \boldsymbol {\Phi_ {k + 1, k}} + Q _ {k} =} \\ & {\quad \boldsymbol {\Phi_ {k + 1, k}} (\mathbf {I - K _ {k}} H _ {k}) P _ {k / k - 1} \boldsymbol {\Phi_ {k + 1, k}} + Q _ {k} =} \\ & {\quad \boldsymbol {\Phi_ {k + 1, k}} P _ {k / k - 1} \boldsymbol {\Phi_ {k + 1, k}} - \boldsymbol {\Phi_ {k + 1, k}} K _ {k} H _ {k} P _ {k / k - 1} \boldsymbol {\Phi_ {k + 1, k}} + Q _ {k} =} \\ & {\quad \boldsymbol {\Phi_ {k + 1, k}} P _ {k / k - 1} \boldsymbol {\Phi_ {k + 1, k}} - \boldsymbol {\Phi_ {k + 1, k}} (\mathbf {(I + P _ {_ k / k - 1}} H _ {_ k} ^ {\mathrm{T}} R _ {_ k} ^ {- 1} H _ {_ k}) ^ {- 1} P _ {_ k / k - 1} H _ {_ k} ^ {\mathrm{T}} R _ {_ k} ^ {- 1} H _ {_ k} P _ {_ k / k - 1} \boldsymbol {\Phi_ {k + 1, k}} + Q _ {_ k =}} \\ & {\quad \boldsymbol {\Phi_ {k + 1, k}} P _ {_ k / k - 1} \boldsymbol {\Phi_ {k + 1, k}} - \boldsymbol {\Phi_ {k + 1, k}} (P _ {_ k / k - 1} ^ {- 1} + H _ {_ k} ^ {\mathrm{T}} R _ {_ k} ^ {- 1} H _ {_ k}) ^ {- 1} H _ {_ k} ^ {\mathrm{T}} R _ {_ k} ^ {- 1} H _ {_ k} P _ {_ k / k - 1} \boldsymbol {\Phi_ {k + 1, k}} + Q _ {_ k}} \end{array}
$$

对上式第二项中的求逆应用矩阵反演公式,得

$$
\begin{array}{r l} \boldsymbol {P} _ {k + 1 / k} & = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} - \boldsymbol {\Phi} _ {k + 1, k} [ \boldsymbol {P} _ {k / k - 1} - \\ & \quad \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {R} _ {k} + \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}}) ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} ] \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + Q _ {k} = \\ & \quad \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k / k - 1} [ \boldsymbol {I} - \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} (\boldsymbol {R} _ {k} + \\ & \quad \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}}) ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} ] \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + Q _ {k} = \\ & \quad \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k / k - 1} \Pi_ {k} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + Q _ {k} \end{array}\tag{3.11.61}
$$

式中

$$
\boldsymbol {\Pi} _ {k} = \boldsymbol {I} - \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \left(\boldsymbol {R} _ {k} + \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}}\right) ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1}
$$

式中的 $(\pmb{R}_k + \pmb{H}_k\pmb{P}_{k / k - 1}\pmb{H}_k^{\mathrm{T}})^{-1}$ 采用矩阵求逆反演公式求解。

$$
\begin{array}{r l} \boldsymbol {\Pi} _ {k} & = \boldsymbol {I} - \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \left[ \boldsymbol {R} _ {k} ^ {- 1} - \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} (\boldsymbol {P} _ {k / k - 1} ^ {- 1} + \right. \\ & \left. \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k}) ^ {- 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \right] \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1} = \end{array}
$$

$$
\begin{array}{r l} & {\pmb {I} - \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H} _ {k} \pmb {P} _ {k / k - 1} + (\pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H} _ {k} \pmb {P} _ {k / k - 1}) ^ {2} =} \\ & {\pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H} _ {k} (\pmb {P} _ {k / k - 1} ^ {- 1} + \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H} _ {k}) ^ {- 1} (\pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H} _ {k} \pmb {P} _ {k / k - 1}) ^ {2} =} \\ & {\pmb {I} - \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H} _ {k} \pmb {P} _ {k / k - 1} + (\pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H _ {k}} \pmb {P} _ {k / k - 1}) ^ {2} =} \\ & {\pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H} _ {k} \pmb {P} _ {k / k - 1} (\pmb {I} + \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H _ {k}} \pmb {P} _ {k / k - 1}) ^ {- 1} (\pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H _ {k}} \pmb {P} _ {k / k - 1}) ^ {2} =} \\ & {\pmb {I} - \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H _ {k}} \pmb {P} _ {k / k - 1} + (\pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H _ {k}} \pmb {P} _ {k / k - 1}) ^ {2}} \\ & {\left(\pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R} _ {k} ^ {- 1} \pmb {H _ {k}} \pmb {P} _ {k / k - 1}\right) ^ {3} (\pmb {I} + \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R _ {k}} ^ {- 1} \pmb {H _ {k}} \pmb {P} _ {k / k - 1}) ^ {- 1}} \\ & {\left[ (\pmb {I} + \pmb {H} _ {k} ^ {\mathrm{T}} \pmb {R _ {k}} ^ {- 1} \pmb {H _ {k}} \pmb {P _ {k / k - 1}}) - \pmb {H _ {k}} ^ {\mathrm{T}} \pmb {R _ {k}} ^ {- 1} \pmb {H _ {k}} \pmb {P _ {k / k - 1}} (\pmb {I} + \pmb {H _ {k}} ^ {\mathrm{T}} \pmb {R _ {k}} ^ {- 1} \pmb {H _ {k}} \pmb {P _ {k / k - 1}}) + \right.} \\ & {\left. (\pmb {H _ {k}} ^ {\mathrm{T}} \pmb {R _ {k}} ^ {- 1} \pmb {H _ {k}} \pmb {P _ {k / k - 1}}) ^ {2} (\pmb {I} + \pmb {H _ {k}} ^ {\mathrm{T}} \pmb {R _ {k}} ^ {- 1} \pmb {H _ {k}} \pmb {P _ {k / k - 1}}) - \right.} \\ & \left. (\pmb {(H _ {{\bf k}}} ^ {\mathrm{T}} \pmb {{R_ {{\bf k}}} ^ {- 1}} \pmb {{H_ {{\bf k}}} P_ {{\bf k / k - 1}}}) ^ {- 3}\right) (\pmb {{I}} + \pmb {{H_ {{\bf k}}} ^ {\mathrm{T}} \pmb {{R_ {{\bf k}}} ^ {- 1}} \pmb {{H_ {{\bf k}}} P_ {{\bf k / k - 1}}}) ^ {- 1}} \\ & \left. (\pmb {{I}} + \pmb {{H_ {{\bf k}}} ^ {\mathrm{T}} \pmb {{R_ {{\bf k}}} ^ {- 1}} \pmb {{H_ {{\bf k}}} P_ {{\bf k / k - 1}}}) ^ {- 1}\right)} \end{array}\tag{3.11.62}
$$

式 $(3.11.62)$ 代入式 $(3.11.61)$ ，得

$$
\boldsymbol {P} _ {k + 1 / k} = \boldsymbol {\Phi} _ {k + 1, k} \boldsymbol {P} _ {k / k - 1} (\boldsymbol {I} + \boldsymbol {H} _ {k} ^ {\mathrm{T}} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \boldsymbol {P} _ {k / k - 1}) ^ {- 1} \boldsymbol {\Phi} _ {k + 1, k} ^ {\mathrm{T}} + \boldsymbol {Q} _ {k}\tag{3.11.63}
$$

由式 $(3.11.60)$ ，

$$
\begin{array}{r l} \mathbf {K} _ {k} & = (\mathbf {I} + \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k}) ^ {- 1} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} = \\ & \quad \mathbf {P} _ {k / k - 1} \mathbf {P} _ {k / k - 1} ^ {- 1} (\mathbf {I} + \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbf {R} _ {k} ^ {- 1} \mathbf {H} _ {k}) ^ {- 1} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbb {R} _ {k} ^ {- 1} = \\ & \quad \mathbf {P} _ {k / k - 1} (\mathbf {P} _ {k / k - 1} + \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbb {R} _ {k} ^ {- 1} \mathbf {H} _ {k} \mathbf {P} _ {k / k - 1}) ^ {- 1} \mathbf {P} _ {k / k - 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbb {R} _ {k} ^ {- 1} = \\ & \quad \mathbf {P} _ {k / k - 1} (\mathbf {I} + \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbb {R} _ {k} ^ {- 1} \mathbf {H} _ {k} \mathbf {P} _ {k / k - 1}) ^ {- 1} \mathbf {H} _ {k} ^ {\mathrm{T}} \mathbb {R} _ {k} ^ {- 1} \end{array}\tag{3.11.64}
$$

比较式(3.11.53)与式(3.11.58a)，式(3.11.54)与式(3.11.64)，和式(3.11.36)与式(3.11.63)，可看出当 $\theta=0$ 时， $H_{\infty}$ 滤波与卡尔曼滤波的一步预测计算完全相同。

综上所述可归纳出如下结论：

（1）设计卡尔曼滤波器时，所使用的系统噪声、量测噪声和初始估计误差的先验信息是这些误差的方差阵 $Q_{k}, R_{k}, P_{0}$ ，而设计 $H_{\infty}$ 滤波器时使用的是这些误差的最大幅值信息。

(2) 卡尔曼滤波是 $H_{\infty}$ 滤波 $\theta=0$ 时的特例，对应的代价函数是无界的，这意味卡尔曼滤波器不能保证系统在所有条件下的滤波效果，如果设计中使用的 $Q_{k}, R_{k}, P_{0}$ 与实际情况不符，滤波效果不会达到最优，差别越大，滤波效果就越差。调谐能一定程度改善滤波效果，但调谐具有盲目性。而 $H_{\infty}$ 滤波通过 $\theta\tilde{S}_{k}P_{k}$ 项实现增大 $K_{k}$ 和 $P_{k+1}$ （见式（3.11.54）和式（3.11.36）），从而增大对新鲜量测的利用权重。并且这一过程是通过定量计算实现的。所以 $H_{\infty}$ 滤波可看做卡尔曼滤波的鲁棒型。

(3) $H_{\infty}$ 滤波中， $\frac{1}{\theta}$ 是代价函数的界， $\theta$ 越小，式(3.11.52)和式(3.11.55)越容易满足，滤波器设计越容易，但代价函数的界就越高，滤波器的潜在风险就越大。这意味着一旦滤波器工作不正常，滤波精度恶化就越严重。

(4) 若只需估计状态变量的线性组合 $y_{k}=T_{k}X_{k}$ ，则卡尔曼滤波先计算 $\hat{X}_{k}$ ，再计算 $\hat{y}_{k}=T_{k}\hat{X}_{k}$ ，滤波过程与 $T_{k}$ 无关。而 $H_{\infty}$ 滤波中 $T_{k}$ 参与了滤波计算，与滤波效果密切相关。

例3-9 设标量系统方程和量测方程为

$$
\begin{array}{r l} & X _ {k + 1} = X _ {k} + W _ {k} \\ & Z _ {k} = X _ {k} + V _ {k} \\ & Y _ {k} = X _ {k} \end{array}
$$

其中 $W_{k}$ 和 $V_{k}$ 均为随机量，且统计特性未知。确定出稳态 $H_{\infty}$ 滤波方程，用于估计 $X_{k}$ 。

解 选择 $Q = R = S = 1$ ，根据题意， $\Phi_{k+1,k} = 1, H_k = 1, T_k = 1$ 。由式(3.11.36)和式(3.11.54)，

$$
\begin{array}{r l} P _ {k + 1} & = \frac {P _ {k}}{1 - \theta P _ {k} + P _ {k}} + 1 = \frac {1 - \theta P _ {k} + 2 P _ {k}}{1 - \theta P _ {k} + P _ {k}} \\ K _ {k} & = \frac {P _ {k}}{1 - \theta P _ {k} + P _ {k}} \end{array}
$$

滤波达到稳态时， $P_{k + 1} = P_k = P$ ，所以有

$$
\begin{array}{l} P (1 - \theta P + P) = 1 - \theta P + 2 P \\ (1 - \theta) P ^ {2} + (\theta - 1) P - 1 = 0 \end{array}
$$

$$
P = \frac {1 - \theta \pm \sqrt {(\theta - 1) ^ {2} + 4 (1 - \theta)}}{2 (1 - \theta)} = \frac {1 - \theta \pm \sqrt {(\theta - 1) (\theta - 5)}}{2 (1 - \theta)}
$$

1) 当 $1 - \theta > 0$ ，即取 $\theta < 1$ 时， $P = \frac{1}{2} \pm \frac{1}{2} \sqrt{\frac{5 - \theta}{1 - \theta}}$

$$
P _ {+} = \frac {1}{2} \left(1 + \sqrt {\frac {5 - \theta}{1 - \theta}}\right) > 0\tag{1}
$$

$P_{-} = \frac{1}{2}\left(1 - \sqrt{\frac{5 - \theta}{1 - \theta}}\right) < 0$ ，不合题意，应舍去。

2) 当取 $1 < \theta < 5$ 时， $(\theta - 1)(\theta - 5) < 0$ 。 $\sqrt{(\theta - 1)(\theta - 5)}$ 不存在，P 无解。所以取 $1 < \theta < 5$ 不能满足要求。

3）当取 $\theta > 5$ 时， $P = \frac{1}{2} \pm \frac{1}{2}\left[-\frac{\sqrt{(\theta - 1)(\theta - 5)}}{\theta - 1}\right] = \frac{1}{2}\left(1 \mp \sqrt{\frac{\theta - 5}{\theta - 1}}\right)$ 。因为在 $\theta > 5$ 条件下， $\frac{\theta - 5}{\theta - 1} = 1 - \frac{4}{\theta - 1} < 1$ ，即 $\sqrt{\frac{\theta - 5}{\theta - 1}} < 1$ 。所以

$$
P _ {+} = \frac {1}{2} \left(1 - \sqrt {\frac {\theta - 5}{\theta - 1}}\right) > 0\tag{2}
$$

$$
P _ {-} = \frac {1}{2} \left(1 + \sqrt {\frac {\theta - 5}{\theta - 1}}\right) > 0\tag{3}
$$

又根据式(3.11.52)， $\theta$ 取值应满足： $\frac{1}{P}-\theta+1>0$ ，即 $\theta$ 应满足： $\theta<1+\frac{1}{P}$ 。若能确定出 $1+\frac{1}{P}$ 的上限。即可确定出 $\theta$ 取值时的允许上限。

对于解(1)，由于 $\theta < 1, \frac{5 - \theta}{1 - \theta}$ 取得最小值时， $P$ 也取得最小值。而 $\theta = -\infty$ 时， $\frac{5 - \theta}{1 - \theta}$ 达到最小，即

$$
\lim _ {\theta \rightarrow - \infty} \frac {5 - \theta}{1 - \theta} = \lim _ {\theta \rightarrow - \infty} \frac {\frac {5}{\theta} - 1}{\frac {1}{\theta} - 1} = 1
$$

此时，P=1 为最小值， $\frac{1}{P}$ 达到最大值 1。所以 $\theta$ 应满足 $\theta<1+\frac{1}{P}=2$ ，即根据式(3.11.52)的要求， $\theta$ 取值不能超过 2。所以只有情况 1)，即 $\theta<1$ 才满足 $H_{\infty}$ 滤波要求，而不能取 $\theta>5$ 。稳态滤波方程为

$$
\hat {X} _ {k + 1} = \hat {X} _ {k} + \overline {{{K}}} (Z _ {k} - \hat {X} _ {k})
$$

式中

$$
\overline {{{K}}} = \frac {P}{1 - \theta P + P}
$$

$$
P = \frac {1}{2} \left(1 + \sqrt {\frac {5 - \theta}{1 - \theta}}\right)
$$

$$
\theta <   1
$$

$$
\text { 习   题 }
$$

3-1 在量测噪声为有色噪声的条件下,如果连续系统中有已知控制量,即

$$
\dot {\boldsymbol {X}} (t) = \boldsymbol {F} (t) \boldsymbol {X} (t) + \boldsymbol {u} (t) + \boldsymbol {w} (t)
$$

试导出滤波方程。

3-2 设系统和量测为

$$
\begin{array}{r l} & {\left[ \begin{array}{l} {\dot {X} _ {1} (t)} \\ {\dot {X} _ {2} (t)} \\ {\dot {X} _ {3} (t)} \end{array} \right] = \left[ \begin{array}{l l l} {0} & {1} & {0} \\ {0} & {0} & {1} \\ {- 1} & {0} & {- 2} \end{array} \right] \left[ \begin{array}{l} {X _ {1} (t)} \\ {X _ {2} (t)} \\ {X _ {3} (t)} \end{array} \right] + \left[ \begin{array}{l} {0} \\ {0} \\ {1} \end{array} \right] w (t)} \\ & {\qquad Z _ {1} (t) = X _ {1} (t) + v (t)} \\ & {\qquad Z _ {2} (t) = X _ {2} (t)} \end{array}
$$

按如下已知条件分别列写出滤波方程：

(1) $w(t)$ 和 $v(t)$ 为互不相关的零均值白噪声,方差强度均为1。

(2) $w(t)$ 和 $v(t)$ 为零均值白噪声,但两者相关,互协方差强度为 $S(t)$ 。

(3) $v(t)$ 为零均值的有色噪声

$$
\dot {v} (t) = - v (t) + \xi (t)
$$

$w(t)$ 和 $\xi(t)$ 都是互不相关的零均值白噪声，方差强度均为 1, $v(0)$ 的方差为零。

$$
v (t) = \zeta (t) + \xi (t) \tag {4}
$$

式中

$$
\dot {\zeta} (t) = - \zeta (t) + \xi (t)
$$

$w(t)$ 和 $\xi(t)$ 都是互不相关的零均值白噪声，方差强度都为 1，且 $\zeta(0)$ 的方差为零。

3-3 设系统方程和量测方程为

式中

$$
\begin{array}{r l} & {\mathbf {X} _ {k + 1} = \boldsymbol {\Phi} _ {k + 1, k} \mathbf {X} _ {k} + \boldsymbol {\Gamma} _ {k} \mathbf {W} _ {k}} \\ & {\mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k} + \boldsymbol {\eta} _ {k}} \\ & {\mathbf {V} _ {k + 1} = \boldsymbol {\Psi} _ {k + 1, k} \mathbf {V} _ {k} + \boldsymbol {\xi} _ {k}} \\ & {E [ \mathbf {W} _ {k} ] = \mathbf {0}, \quad E [ \mathbf {W} _ {k} \mathbf {W} _ {j} ^ {\mathrm{T}} ] = Q _ {k} \delta_ {k j}} \\ & {E [ \boldsymbol {\eta} _ {k} ] = \mathbf {0}, \quad E [ \boldsymbol {\eta} _ {k} \boldsymbol {\eta} _ {j} ^ {\mathrm{T}} ] = \boldsymbol {\rho} _ {k} \delta_ {k j}} \\ & {E [ \boldsymbol {\xi} _ {k} ] = \mathbf {0}, \quad E [ \boldsymbol {\xi} _ {k} \boldsymbol {\xi} _ {j} ^ {\mathrm{T}} ] = R _ {k} \delta_ {k j}} \end{array}
$$

$W_{k}, \eta_{k}$ 和 $\xi_{k}$ 互不相关。

试分别用状态扩增法和量测扩增法列写出递推滤波方程，并比较两种滤波方法的特点。

3-4 设系统方程和量测方程为

$$
\begin{array}{r l} & \mathbf {X} _ {k} = \boldsymbol {\Phi} _ {k, k - 1} \mathbf {X} _ {k - 1} \\ & \mathbf {Z} _ {k} = \mathbf {H} _ {k} \mathbf {X} _ {k} + \mathbf {V} _ {k} \end{array}
$$

其中 $Z_{k}$ 为 m 维向量， $V_{k}$ 为零均值的白噪声序列，方差阵 $R_{k}$ 为对角阵，试列写出 k-1 至 k 步的序贯处理平方根滤波方程。

3-5 如果例 3-3 中, $H = [1 \quad 1]$ , 按基本卡尔曼滤波方程和平方根滤波方程分别计算出 $K_{k+1}$ 和 $P_{k+1}$ , 并比较两者的结果。

3-6 如果量测不是标量而是向量，且不采用序贯处理，则在平方根滤波的 Potter 算法中，量测更新将会遇到什么障碍？

3-7 设计滤波器时选用的系统模型和量测模型为

$$
\begin{array}{r} {\pmb {X} _ {k} = \pmb {\Phi} _ {k, k - 1} \pmb {X} _ {k - 1} + \pmb {\Gamma} _ {k - 1} \pmb {W} _ {k - 1}} \\ {\pmb {Z} _ {k} = \pmb {H} _ {k} \pmb {X} _ {k} + \pmb {V} _ {k}} \end{array}
$$

证明如下结论：

(1) 若 $\Phi_{k,k-1}, H_{k}, \Gamma_{k}, Q_{k}$ 和 $R_{k}$ 有误差，则即使 $\hat{X}_{0} = E[X_{0}^{r}] = m_{X_{0}}^{r}$ ，估计仍然是有偏的；

(2) 在上述条件下, 如果 $m_{X_{0}}^{r}=0$ , 则估计是无偏的;

(3) 在(1)的条件下, 若 $\Delta\Phi_{k,k-1}=0,\Delta H_{k}=0$ , 则估计是无偏的。

3-8 在模型正确的条件下，令 $P_{k}^{r}$ 表示用正确参数得到的滤波估计均方误差，并令

$$
\begin{array}{l} \Delta P _ {k} ^ {p} = P _ {k} ^ {p} - P _ {k} ^ {r} \\ \Delta P _ {k} ^ {r} = P _ {k} - P _ {k} ^ {r} \end{array}
$$

如果选取：

$$
Q _ {k} \geqslant Q _ {k} ^ {r}, R _ {k} \geqslant R _ {k} ^ {r}, P _ {0} ^ {p} \geqslant P _ {0} ^ {r}, P _ {0} \geqslant P _ {0} ^ {r}
$$

证明：

$$
\Delta P _ {k} ^ {p} \geqslant 0, \Delta P _ {k} ^ {r} \geqslant 0
$$

（注：这说明 $P_{k}^{r}$ 是 $P_{k}^{p}$ 和 $P_{k}$ 可能达到的最小值）

3-9 对式(3.10.13)两边求偏导数,试推导出式(3.10.21)。

3-10 试用式(3.10.41)求解出例3-7的小尺度灵敏度方程(12)、(13)、(14)。

3-11 设系统和量测为

$$
\left[ \begin{array}{c} \boldsymbol {X} _ {k + 1} ^ {1} \\ \boldsymbol {X} _ {k + 1} ^ {2} \end{array} \right] = \left[ \begin{array}{c c} \boldsymbol {\Phi} _ {k + 1, k} ^ {1 1} & \boldsymbol {\Phi} _ {k + 1, k} ^ {1 2} \\ \boldsymbol {0} & \boldsymbol {\Phi} _ {k + 1, k} ^ {2 2} \end{array} \right] \left[ \begin{array}{c} \boldsymbol {X} _ {k} ^ {1} \\ \boldsymbol {X} _ {k} ^ {2} \end{array} \right] + \left[ \begin{array}{c c} \boldsymbol {\Gamma} _ {k} ^ {1} & \boldsymbol {0} \\ \boldsymbol {0} & \boldsymbol {\Gamma} _ {k} ^ {2} \end{array} \right] \left[ \begin{array}{c} \boldsymbol {W} _ {k} ^ {1} \\ \boldsymbol {W} _ {k} ^ {2} \end{array} \right]
$$

$$
\mathbf {Z} _ {k} = \left[ \begin{array}{l l} \mathbf {H} _ {k} ^ {1} & \mathbf {0} \end{array} \right] \left[ \begin{array}{l} \mathbf {X} _ {k} ^ {1} \\ \mathbf {X} _ {k} ^ {2} \end{array} \right] + \mathbf {V} _ {k}
$$

其中， $W_{k}^{1}$ 、 $W_{k}^{2}$ 和 $V_{k}$ 都是互不相关的零均值白噪声序列， $W_{k}^{1}$ 的方差阵为 $Q_{k}^{1}$ ， $W_{k}^{2}$ 的方差阵为 $Q_{k}^{2}$ ， $V_{k}$ 的方差阵为 $R_{k}$ 。

仿照式(3.9.3)～(3.9.14)的推导，推导出仅估计 $X_{k}^{1}$ 的降阶滤波方程。

3-12 设 $R_{c}(t - \tau) = \begin{cases} \frac{1}{\Delta t}, & t \leqslant \tau \leqslant t + \Delta t \\ 0, & \text{其余 } t \end{cases}$

则 $\delta (t - \tau) = \lim_{\Delta t\to 0}R_c(t - \tau)$

利用这一关系式,证明:

$$
\int_ {t _ {0}} ^ {\mathrm{T}} \boldsymbol {T} (t) \boldsymbol {\Phi} ^ {a \mathrm{T}} (t, \tau) \delta (t - \tau) \mathrm{d} \tau + \int_ {t _ {0}} ^ {\mathrm{T}} \boldsymbol {\Phi} ^ {a} (t - \tau) \boldsymbol {T} (t) \delta (t - \tau) \mathrm{d} \tau = \boldsymbol {T} (t)
$$

式中 $\Phi^{a}(t,\tau)$ 为某一线性系统的转移阵， $T(t)$ 为某一矩阵。

3-13 直升机作定高悬停，悬停高度用机载无线电高度表作监测。设无线电高度表的采样间隔为 $T$ ，高度测量误差为零均值的白噪声序列，方差为 $R$ 。由于气流影响，直升机在定高值附近作快速上下颠簸，高度变化近似描述为： $X_{k} - X_{k - 1} = W_{k - 1}$ ，其中 $W_{k - 1}$ 为零均值白噪声，方差为 $Q$ 。

(1) 设计提取精确高度的卡尔曼滤波器；

(2) 确定出当滤波达到稳态时的滤波方程。

3-14 设标量系统方程和量测方程为

$$
\begin{array}{r l} & X _ {k + 1} = X _ {k} + W _ {k} \\ & Z _ {k} = h X _ {k} + V _ {k} \\ & Y _ {k} = X _ {k} \end{array}
$$

式中 $W_{k}$ 和 $V_{k}$ 均为随机量，且统计特性未知。确定出稳态 $H_{\infty}$ 滤波方程，用于估计 $X_{k}$ 。

