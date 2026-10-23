Article

# Observability-Constrained Resampling-Free Cubature Kalman Filter for GNSS/INS with Measurement Outliers

Bingbo Cui <sup>1</sup> , Wu Chen <sup>2</sup> , Duojie Weng <sup>2,</sup>\*, Xinhua Wei <sup>1</sup>, Zeyu Sun <sup>1,3</sup>, Yan Zhao <sup>4</sup> and Yufei Liu <sup>3</sup>

School of Agricultural Engineering, Jiangsu University, Zhenjiang 212013, China; cuibingbo@ujs.edu.cn (B.C.); wxh@ujs.edu.cn (X.W.); 1000004203@ujs.edu.cn (Z.S.)

2 Department of Land Surveying and Geo-Informatics, The Hong Kong Polytechnic University, Hong Kong, China; wu.chen@polyu.edu.hk

3 College of Biosystems Engineering and Food Science, Zhejiang University, Hangzhou 310058, China; yufeiliu@zju.edu.cn

4 College of Mechanical and Electrical Engineering, Shihezi University, Shihezi 832003, China; zhaoyan@shzu.edu.cn

米 Correspondence: wengduojie.lsgi@polyu.edu.hk

![](images/19103cc94f9d55c289015e09102843a58150410a020f9138af5ae4d90ac3e748.jpg)

Citation: Cui, B.; Chen, W.; Weng, D.; Wei, X.; Sun, Z.; Zhao, Y.; Liu, Y. Observability-Constrained Resampling-Free Cubature Kalman Filter for GNSS/INS with Measurement Outliers. Remote Sens. 2023, 15, 4591. https://doi.org/ 10.3390/rs15184591

Academic Editors: Marcin Uradzinski, You Li and Hang Guo

Received: 18 August 2023 Revised: 14 September 2023 Accepted: 16 September 2023 Published: 18 September 2023

![](images/35e33b5358010cf9bf544375fc798a0a30270d9ba2e20db1a0b1b6cef9a3885b.jpg)

Copyright: © 2023 by the authors. Licensee MDPI, Basel, Switzerland. This article is an open access article distributed under the terms and conditions of the Creative Commons Attribution (CC BY) license (https:// creativecommons.org/licenses/by/ 4.0/).

Abstract: Integrating global navigation satellite systems (GNSSs) with inertial navigation systems (INSs) has been widely recognized as an ideal solution for autonomous vehicle navigation. How ever, GNSSs suffer from disturbances and signal blocking inevitably, making the performance of GNSS/INSs degraded in the occurrence of measurement outliers. It has been proven that the sigma points-based Kalman filter (KF) performs better than an extended KF in cases where large prior uncertainty is present in the state estimation of a GNSS/INS. By modifying the sigma points directly, the resampling-free sigma point update framework (SUF) propagates more information excepting Gaussian moments of prescribed accuracy, based on which the resampling-free cubature Kalman filter (RCKF) was developed in our previous work. In order to enhance the adaptivity and robustness of the RCKF, the resampling-free SUF depending on dynamic prediction residue should be improved by suppressing the time-varying measurement outlier. In this paper, a robust observability-constrained RCKF (ROCRCKF) is proposed based on adaptive measurement noise covariance estimation and outlier detection, where the occurrence of measurement outliers is modelled by the Bernoulli variable and estimated with the state simultaneously. Experiments based on car-mounted GNSS/INSs are per formed to verify the effectiveness of the ROCRCKF, and result indicates that the proposed algorithm outperforms the RCKF in the presence of measurement outliers, where the heading error and average root mean square error of the position are reduced from 1.96<sup>◦</sup> and 6.38 m to 0.27<sup>◦</sup> and 5.95 m, respec tively. The ROCRCKF is robust against the measurement outliers and time-varying model uncertainty, making it suitable for the real-time implementation of GNSS/INSs in GNSS-challenged environments.

Keywords: autonomous navigation; integrated navigation; cubature Kalman filter; measurement outlier detection; observability-constrained filter

## 1. Introduction

Reliable and accurate position and orientation information plays a vital role in the autonomous navigation of land vehicles, e.g., unmanned agricultural machinery and selfdriving cars [1,2]. Global navigation satellite systems (GNSSs) have become the default tool for land vehicle navigation, employing real-time kinematic technology to achieve centimeter-level positioning accuracy. Inertial navigation systems (INSs) represent a self contained navigation technology which updates the solution to a given problem by calculating the integrals of inertial sensor output given the initial state. Because the drift errors of INSs are accumulated with time, the integration of GNSSs with INSs has been widely studied and applied in autonomous navigation. The Kalman filter (KF) and its variants have dominated the information fusion of navigation and target tracking since the celebrated work of Kalman and Schmidt [3,4]. With precise prior knowledge on the underlying state and noise statistical properties, KF achieves optimal estimation under the linear minimum mean square error (LMMSE). However, GNSS suffers from multipath disturbances and signal blocking inevitably, making the hypotheses on the Gaussian distribution noise invalid and thus degrading the performance of the KF heavily.

Because there is always nonlinearity in practical state space models (SSMs), many sub-optimal filters have been developed to approximate the KF framework. The extended Kalman filter (EKF) is by far the most popular method for solving nonlinear filtering problems and linearizes nonlinear functions to the best available state by using first-order Taylor series expansion. In order to reduce the nonlinear approximation error, sigma-point KFs based on deterministic sampling and quadrature rules are proposed, e.g., unscented Kalman filters [5], central difference filters [6], and cubature Kalman filters (CKFs) [7], which not only avoid calculating the derivatives of functions but also showcase better convergence than EKFs in cases of large initial uncertainty [8]. By using the spherical-radial cubature rule, CKF approximates the Gaussian moments at third-order accuracy in terms of Taylor series and shows better numerical stability than UKFs in high-dimensional sys tems [9,10]. Particle filters (PFs) approximate the state posterior probability density function (PDF) by using a large number of weighted particles, and they make no assumptions concerning the state distribution and SSM, which is suitable for nonlinear and non-Gaussian filtering problems [11]. Although improved PFs have been proposed [12,13], their huge computational complexity still hinders their application in real-time GNSS/INSs. In the presence of time-varying noise statistics, model mismatches, and measurement outliers, the SSM of a GNSS/INS appears to have non-Gaussian and nonlinear properties [14–16]. Because there are no closed-form solutions for nonlinear and non-Gaussian filtering prob lems, the suboptimal filter design for navigation and target tracking is an active area of research [17,18].

The unknown process and measurement noise of KF-type filters can be estimated online by using maximum a posterior, or variational Bayesian (VB) technology [19–21]. However, the derivation of the KF is under the LMMSE, which makes it sensitive to measurement outliers and non-Gaussian noise. Other robust KFs include Huber-based KFs, maximum correntropy criterion (MCC)-based KFs [22–24], and finite impulse response KFs [25,26]. Although these filters showcase better performance than their LMMSE-based counterparts in the presences of measurement outliers, they don’t make full use of the feature of non-Gaussian distribution. Unlike [21], which estimates the measurement noise under a Gaussian SSM, the skew-t distribution is used to capture the skewness and heavy tailedness of asymmetric distributions [27]. However, the occurrences of state and mea surement outliers are time-varying, and there are non-stationary heavy-tailed processes and measurement noise. Recently, the mixture distribution has been employed to handle time-varying outliers, e.g., Gaussian mixtures [28] and Gaussian-Student’s mixtures [29], where both the prediction covariance and measurement noise covariance are estimated by VB after constructing the linear hierarchical Gaussian SSM. These robust KFs import extra parameters to formulate a robust model, which not only needs suitable hierarchical prior selection but also increases the computational complexity [30]. What is more, the above-mentioned robust KFs focus on constructing apposite models of non-Gaussian noise, whereas their uncertainty propagation is less effective when the prior uncertainty is high and the observation is accurate [31].

In the information fusion of a GNSS/INS, nonlinearity is negligible, provided that precision prior knowledge on noise and state statistics is provided; however, this randomly occurs in practical implementation. Furthermore, the under-estimation of VB-based parameters and inconsistent covariance in the KF framework degrade the state estimation of GNSS/INSs [27]. The main cause of inconsistent estimation is the linearization error of dynamic models, which produces spurious information and reduces the uncertainty incorrectly by coupling with the KF gain update [31–33]. Minimizing the linearization error of the dynamic model resulting from actual state approximation is an effective approach to relieving covariance inconsistency. A resampling-free SUF that modifies the sigma points directly based on model prediction residue was proposed in our previous work [31], where a resampling-free CKF (RCKF) was developed to handle the GNSS outage of GNSS/INSs. Based on the RCKF, the MCC-enhanced and VB-based resampling-free SUFs were then developed to handle the time-varying and non-Gaussian measurement noise [34,35]. However, the parameter selection of resampling-free SUFs is pre-determined by tuning attempts, and the effect of non-stationary heavy-tailed noise has not been fully taken into consideration.

Inspired by works on measurement outlier detection [36,37], an improved RCKF with enhanced robustness and adaptability is developed in this paper, where the time-varying measurement outlier is addressed by VB-based outlier detection and the sigma points are updated based on the prediction residue of the dynamic model. Consequently, a robust observability-constrained RCKF (ROCRCKF) is developed for land vehicle navigation based on outlier-robust measurement updates. The main contributions of this work are two-fold: (1) an improved RCKF is developed by employing a model indicator and a scale parameter to handle non-stationary measurement outliers, (2) the ROCRCKF is verified based on a car-mounted GNSS/INS with different maneuver modes, and the performance of a resampling-free SUF under different state observability is investigated.

The remainder of this paper is organized as follows. In Section 2, CKFs and resamplingfree CKFs are briefly reviewed. The ROCRCKF is then developed in Section 3 based on adaptive outlier detection. By employing the car-mount GNSS/INS field test, the proposed algorithm is verified in Section $^ { 4 , }$ and the result is discussed in Section 5. Section 6 concludes this paper.

## 2. Preliminaries

## 2.1. Cubature Kalman Filter

The discrete-time SSM of GNSS/INSs can be written as:

$$
\pmb {x} _ {k} = \mathsf {f} (\pmb {x} _ {k - 1}) + \pmb {w} _ {k - 1}\tag{1}
$$

$$
\boldsymbol {z} _ {k} = \mathbf {h} (\boldsymbol {x} _ {k}) + \boldsymbol {v} _ {k}\tag{2}
$$

where $\boldsymbol { x } _ { k } \in \mathbb { R } ^ { n \times 1 }$ and $z _ { k } \in \mathbb { R } ^ { p \times 1 }$ are the state and observations at time k; $w _ { k - 1 } \in \mathbb { R } ^ { n \times 1 }$ and ${ \boldsymbol { v } } _ { k } \in \mathbb { R } ^ { p \times 1 }$ are the process and measurement noise with the known covariance $Q _ { k - 1 } \in \mathbb { R } ^ { n \times n }$ and $R _ { k } \in \mathbb { R } ^ { p \times p } ;$ and $\mathrm { f } ( \cdot )$ and $\mathrm { h } ( \cdot )$ are the dynamic and measurement functions. The initial state $x _ { 0 } , w _ { k - 1 } ,$ , and $v _ { k }$ are mutually independent and subject to Gaussian distribution. By utilizing (1) and $( 2 ) .$ , the state prior PDF and likelihood function can be, respectively, written as $p ( \pmb { x } _ { k } ) = N \Big ( \pmb { x } _ { k } ; \hat { \pmb { x } } _ { k | k - 1 } , P _ { k | k - 1 } \Big )$ and $p ( z _ { k } ) = N \Big ( x _ { k } ; \hat { z } _ { k | k - 1 } , P _ { k | k - 1 } ^ { z z } \Big )$ , where $N ( x ; \hat { x } , P )$ denotes the Gaussian distribution with the mean, $\scriptstyle { \hat { x } } ,$ and covariance, P. The sigma points are defined as follows:

$$
\xi_ {i, k - 1} = \hat {\pmb {x}} _ {k - 1 | k - 1} + \sqrt {n P _ {k - 1 | k - 1}} [ \mathbf {1} ] _ {i}\tag{3}
$$

$$
\zeta_ {i, k} = \hat {\pmb {x}} _ {k | k - 1} + \sqrt {n P _ {k | k - 1}} [ \mathbf {1} ] _ {i}\tag{4}
$$

where $\xi _ { i , k - 1 }$ and $\zeta _ { i , k } .$ , respectively, correspond to the posterior PDF of time k−1 and the prior PDF at time k. Let $e _ { i }$ denote the i-th column of identity matrix $I _ { n \times n }$ and $[ \mathbf { 1 } ] _ { i }$ is defined as:

$$
[ \mathbf {1} ] _ {i} = \left\{ \begin{array}{c c} \boldsymbol {e} _ {i} & i = 1, \dots , n \\ - \boldsymbol {e} _ {i - n} & i = n + 1, \dots , 2 n \end{array} \right.
$$

Approximating the involved moments based on the third-degree spherical-radial cubature rule yields:

$$
\hat {x} _ {k | k - 1} = \sum_ {i = 1} ^ {N} \varpi_ {i} \mathbf {f} (\xi_ {i, k - 1})\tag{5}
$$

$$
P _ {k | k - 1} = \sum_ {i = 1} ^ {N} \varpi_ {i} \Big (\mathsf {f} (\xi_ {i, k - 1}) - \hat {\boldsymbol {x}} _ {k | k - 1} \Big) \Big (\mathsf {f} (\xi_ {i, k - 1}) - \hat {\boldsymbol {x}} _ {k | k - 1} \Big) ^ {T} + Q _ {k - 1}\tag{6}
$$

$$
\hat {z} _ {k | k - 1} = \sum_ {i = 1} ^ {N} \varpi_ {i} \mathbf {h} (\zeta_ {i, k})\tag{7}
$$

$$
P _ {k | k - 1} ^ {z z} = \sum_ {i = 1} ^ {N} \varpi_ {i} \Big (\mathsf {h} (\zeta_ {i, k}) - \hat {\mathbf {z}} _ {k | k - 1} \Big) \Big (\mathsf {h} (\zeta_ {i, k}) - \hat {\mathbf {z}} _ {k | k - 1} \Big) ^ {T} + R _ {k}\tag{8}
$$

$$
P _ {k | k - 1} ^ {\boldsymbol {x z}} = \sum_ {i = 1} ^ {N} \varpi_ {i} \left(\mathrm{f} (\xi_ {i, k - 1}) - \hat {\boldsymbol {x}} _ {k | k - 1}\right) \left(\mathrm{h} (\zeta_ {i, k}) - \hat {\boldsymbol {z}} _ {k | k - 1}\right) ^ {T}\tag{9}
$$

where $\varpi _ { i } = 1 / N$ is the weight of corresponding sigma points and $N = 2 n$ . Notably, $\xi _ { i , k - 1 }$ and $\zeta _ { i , k }$ are proportional to state dimension, n, leading to a non-local sampling problem in sigma points-based KFs. Once a new measurement, $z _ { k } ,$ , is received, the state posterior PDF can be written as $p ( \pmb { x } _ { k } ) = N \Big ( \pmb { x } _ { k } ; \hat { \pmb { x } } _ { k | k } , P _ { k | k } \Big )$ , which under Bayesian filter framework can be expressed as:

$$
\hat {\boldsymbol {x}} _ {k | k} = \hat {\boldsymbol {x}} _ {k | k - 1} + K _ {k} \left(\boldsymbol {z} _ {k} - \hat {\boldsymbol {z}} _ {k | k - 1}\right)\tag{10}
$$

$$
P _ {k | k} = P _ {k | k - 1} - K _ {k} P _ {k | k - 1} ^ {z z} K _ {k} ^ {T}\tag{11}
$$

$$
K _ {k} = P _ {k | k - 1} ^ {\boldsymbol {x z}} \left(P _ {k | k - 1} ^ {\boldsymbol {z z}}\right) ^ {- 1}\tag{12}
$$

The recursive sigma-point update framework of the CKF is based on approximated Gaussian moments, which are less efficient in cases where there is non-Gaussian noise or large uncertainty in the approximation of the state’s prior and posterior PDF. Moreover, reducing the covariance inconsistence by minimizing the linearization error of the dynamic function is suggested, especially when the nonlinearity of the dynamic function is strong [33]. Since the dynamics of function approximation errors are much slower than the instantiated function points, it is better to update the posterior points based on the model prediction residue.

## 2.2. Resampling-Free Sigma-Point Update Framework

Suppose that $\boldsymbol { x } \in \mathbb { R } ^ { n \times 1 }$ is a random variable subject to Gaussian distribution and its moments can be fully captured by $\{ \varsigma _ { i } , \varpi _ { i } \} , i = 1 , \cdot \cdot \cdot , N$ . The general transform of x can be formulated as:

$$
\boldsymbol {y} = g (\boldsymbol {x}) + \epsilon\tag{13}
$$

where $p ( \epsilon ) = N ( \epsilon ; 0 , P _ { \epsilon \epsilon } )$ represents the noise of direct point modification, and the point corresponding to random variable y is denoted as:

$$
\mathbf {Y} _ {i} = \overline {{g}} + T (g (\varsigma_ {i}) - \overline {{g}})\tag{14}
$$

$$
T = \sqrt {P _ {g g} + P _ {\epsilon \epsilon}} \left(\sqrt {P _ {g g}}\right) ^ {- 1}
$$

where $\begin{array} { r } { \overline { { g } } = \sum _ { i = 1 } ^ { N } \omega _ { i } g ( \varsigma _ { i } ) , P _ { g g } = \sum _ { i = 1 } ^ { N } \omega _ { i } \big ( g ( \varsigma _ { i } ) - \overline { { g } } \big ) \big ( g ( \varsigma _ { i } ) - \overline { { g } } \big ) ^ { T } } \end{array}$ , and the transform matrix are defined as:

(15)

Let $\left\{ X _ { i , k - 1 | k - 1 } ^ { + } , ~ \varpi _ { i } \right\}$ denote the posterior sigma points and weights of $x _ { k - 1 }$ , and define $\widetilde { X } _ { i , k | k - 1 } ^ { - } = f { \left( { X } _ { i , k - 1 | k - 1 } ^ { + } \right) } - \hat { x } _ { k | k - 1 }$ , where $\begin{array} { r } { \hat { \mathbf { x } } _ { k | k - 1 } = \sum _ { i = 1 } ^ { N } \varpi _ { i } f \Big ( X _ { i , k - 1 | k - 1 } ^ { + } \Big ) } \end{array}$ . By applying (14) to (1), we obtain:

$$
X _ {i, k | k - 1} = \hat {\boldsymbol {x}} _ {k | k - 1} + \sqrt {P _ {k | k - 1} + Q _ {k - 1}} \left(\sqrt {P _ {k | k - 1}}\right) ^ {- 1} \widetilde {X} _ {i, k | k - 1} ^ {-}\tag{16}
$$

Considering the linear measurement update step of CKF yields:

$$
X _ {i, k | k} ^ {-} = X _ {i, k | k - 1} + K _ {k} \Big (z _ {k} - Z _ {i, k | k} \Big)\tag{17}
$$

where $P _ { k | k - 1 } = \Sigma _ { i = 1 } ^ { N } \varpi _ { i } \widetilde X _ { i , k | k - 1 } ^ { - } \Big ( \widetilde X _ { i , k | k - 1 } ^ { - } \Big ) ^ { T } , \ Z _ { i , k | k } \ = \ h \Big ( X _ { i , k | k - 1 } \Big )$ , and $X _ { i , k | k } ^ { - }$ denote the sigma points after assimilating observation information. If we define $\begin{array} { r } { \overline { { \boldsymbol { x } } } _ { k | k } = \sum _ { i = 1 } ^ { N } \varpi _ { i } X _ { i , k | k } ^ { - } , } \end{array}$ $\begin{array} { r } { \pmb { \overline { { P } } } _ { k | k } = \sum _ { i = 1 } ^ { N } \pmb { \mathcal { O } } _ { i } \left( X _ { i , k | k } ^ { - } - \overline { { \pmb { x } } } _ { k | k } \right) \left( X _ { i , k | k } ^ { - } - \overline { { \pmb { x } } } _ { k | k } \right) ^ { T } } \end{array}$ and suppose that there is an approximation error in the linear Bayesian update (17), then by employing (14) in (17), we obtain [35]:

$$
X _ {i, k | k} ^ {+} = \overline {{\boldsymbol {x}}} _ {k | k} + \sqrt {\overline {{\boldsymbol {P}}} _ {k | k} + K _ {k} R _ {k} K _ {k} ^ {T}} \Big (\sqrt {\overline {{\boldsymbol {P}}} _ {k | k}} \Big) ^ {- 1} \Big (X _ {i, k | k} ^ {-} - \overline {{\boldsymbol {x}}} _ {k | k} \Big)\tag{18}
$$

where $K _ { k } R _ { k } K _ { k } ^ { T }$ accounts for the variance increase in direct point modification from $X _ { i , k | k } ^ { - }$ to $X _ { i , k | k } ^ { + }$ . Notably, importing the resampling-free SUF into CKF is straightforward, as it replaces (3) and (4) by setting $\xi _ { i , k } = X _ { i , k | k } ^ { + }$ and $\zeta _ { i , k + 1 } = f { \left( X _ { i , k | k } ^ { + } \right) }$

## 2.3. Observability-Constrained Resampling-Free CKF

Because of time-varying measurement noise and outliers, the direct point modification from $X _ { i , k | k } ^ { - }$ to $X _ { i , k | k } ^ { + }$ cannot be formulated as (18), i.e., the $K _ { k }$ approximates, 0 and $X _ { i , k \mid k ^ { \prime } } ^ { - }$ , should be replaced by $X _ { i , k \left| k - 1 \right. }$ in cases where there is a GNSS outage. Further, $\widetilde { X } _ { i , k | k } ^ { + } = X _ { i , k | k } ^ { + } - \hat { x } _ { k | k } , \varpi = \left[ \varpi _ { 1 } \quad \cdot \cdot \cdot \quad \varpi _ { N } \right] ^ { T }$ , and $W = \mathrm { d i a g } ( \varpi )$ , where diag( ) denotes an operation that builds a diagonal matrix with the element of vector $\pmb { \mathcal { O } }$ . Because the transformed points should at least match the first two moments of the prior PDF, we obtain [31]:

$$
\widetilde {X} _ {i, k | k - 1} ^ {-} \boldsymbol {\omega} = \mathbf {0}\tag{19a}
$$

$$
\widetilde {X} _ {i, k | k - 1} ^ {-} W \Big (\widetilde {X} _ {i, k | k - 1} ^ {-} \Big) ^ {\boldsymbol {T}} = P _ {k | k - 1} - Q _ {k - 1}\tag{19b}
$$

Analogously, employing the constraints for the moment approximation of posterior PDF yields:

$$
\widetilde {X} _ {i, k | k} ^ {+} \boldsymbol {\omega} = \mathbf {0}\tag{20a}
$$

$$
\widetilde {X} _ {i, k | k} ^ {+} W \Big (\widetilde {X} _ {i, k | k} ^ {+} \Big) ^ {\boldsymbol {T}} = P _ {k | k} + \Delta E\tag{20b}
$$

Take the posterior sigma points error $\widetilde { X } _ { i , k | k } ^ { + } = \Xi _ { k } \widetilde { X } _ { i , k | k - 1 } ^ { - } ,$ , where $\Xi _ { k }$ is the transform matrix accounting for the propagation of sigma point residue and ∆E denotes the uncertainty resulting from linearization errors and unmodeled dynamics. By setting $\Delta E = \Lambda _ { k } K _ { k } R _ { k } K _ { k } ^ { \check { T } }$ the spurious uncertainty reduction for unobservable states can be addressed, where $\Lambda _ { \mathbf { k } } = \operatorname { d i a g } \Bigl ( \left[ \eta _ { 1 } \quad \cdot \cdot \quad \eta _ { n } \right] ^ { \mathrm { T } } \Bigr )$ is the weight matrix accounting for different state observ abilities and $\ell \in [ 1 , n ]$ is the index of state elements. For the information fusion of the GNSS/INS, the corresponding $\eta _ { \ell }$ of sensor biases and attitude errors is set as 0 to prevent their posterior points from spurious uncertainty reduction. In order to ensure a variance increase in the calculation of the transform matrix while altering the points as little as possible [38], the $\eta _ { \ell }$ corresponding to the position and velocity error is set as slightly larger than 1. Notably, (20a) follows directly from (19a), and substituting (19b) into (20b), yields:

$$
\begin{array}{r l} \widetilde {X} _ {i, k | k} ^ {+} W \Big (\widetilde {X} _ {i, k | k} ^ {+} \Big) ^ {T} = \Xi_ {k} \widetilde {X} _ {i, k | k} ^ {-} W \Big (\widetilde {X} _ {i, k | k} ^ {-} \Big) ^ {T} \Xi_ {\mathbf {k}} ^ {\mathbf {T}} \\ = \Xi_ {k} \Big (P _ {k | k - 1} - Q _ {k - 1} \Big) \Xi_ {k} ^ {T} \end{array}\tag{21}
$$

The positive definiteness of $P _ { k | k - 1 } - Q _ { k - 1 }$ and $P _ { k | k } + \Delta E$ can be ensured by using thirddegree spherical-radial cubature. Then, based on singular value decomposition (SVD), we obtain $P _ { k | k - 1 } - Q _ { k - 1 } = L _ { k } ^ { - } S \left( L _ { k } ^ { - } \right) ^ { T }$ and $P _ { k | k } + \Delta E = \overline { { L _ { k } ^ { + } \overline { { S } } \big ( L _ { k } ^ { + } \big ) ^ { T } } }$ , where S and S are the diagonal matrices of singular values and $L _ { k } ^ { - }$ and $L _ { k } ^ { + }$ are orthonormal matrices. Based on (21), the transform matrix can be formulated as:

$$
\Xi_ {k} = L _ {k} ^ {+} \overline {{S}} ^ {\frac {1}{2}} S ^ {- \frac {1}{2}} \left(L _ {k} ^ {-}\right) ^ {- 1}\tag{22}
$$

where ${ \overline { { S } } } ^ { \frac { 1 } { 2 } } S ^ { - { \frac { 1 } { 2 } } }$ is element-wise multiplication, making the calculation of the transform matrix numerically stable. After that, we calculated $\Xi _ { k } ,$ the posterior sigma points at time $k ,$ denoted as:

$$
X _ {i, k | k} ^ {+} = \hat {\pmb {x}} _ {k | k} + \Xi_ {k} \widetilde {X} _ {i, k | k - 1} ^ {-}\tag{23}
$$

Based on the above derivations, the steps of the RCKF can be summarized as Algorithm 1. Notably, the RCKF generates new sigma points based on $\hat { \boldsymbol { x } } _ { k \mid k }$ and the transformed point residue $\widetilde { X } _ { i , k | k - 1 } ^ { - }$ from the dynamic function, which has no direct relation with state dimension ${ \boldsymbol { \mathrm { n } } } ,$ relieving the non-local sampling problem. Regarding specific nonlinear transforms, the third-degree cubature rule approximates the mean more accurately than covariance, and there is extra information in $X _ { i , k | k } ^ { + }$ except for Gaussian moments in cases where the dynamic function is of high unmodeled uncertainty.

<div class="mineru-algorithm" style="white-space: pre-wrap; font-family:monospace;">
Algorithm 1 The pseudocode of RCKF

Inputs: $\hat{x}_{k-1|k-1}$, $P_{k-1|k-1}$, $Q_{k-1}$, $R_k$

1. Initialize $\xi_{i,k-1}$, $\zeta_{i,k-1}$ by (3), (4) and update $X_{i,k|k}^+$ based on CKF and (18)

Time update:

2. Let $\xi_{i,k} = X_{i,k|k}^+$, and calculate $\hat{x}_{k+1|k}$, $P_{k+1|k}$ by (5) and (6)

3. Calculate $\widetilde{X}_{i,k+1|k}^- = f(X_{i,k|k}^+) - \hat{x}_{k+1|k}$, and update $X_{i,k+1|k}$ by (16)

Measurement update:

4. Calculate $\hat{z}_{k+1|k}$, $P_{k+1|k}^{zz}$ and $P_{k+1|k}^{xz}$ as

$\hat{z}_{k+1|k} = \sum_{i=1}^{N} \varpi_i h(X_{i,k+1|k})$ $P_{k+1|k}^{zz} = \sum_{i=1}^{N} \varpi_i (h(X_{i,k+1|k}) - \hat{z}_{k+1|k})(h(X_{i,k+1|k}) - \hat{z}_{k+1|k})^T)$ $P_{k+1|k}^{xz} = \sum_{i=1}^{N} \varpi_i (f(\xi_{i,k}) - \hat{x}_{k+1|k})(h(X_{i,k+1|k}) - \hat{z}_{k+1|k})^T)$

5. Update $\hat{x}_{k+1|k+1}$, $P_{k+1|k+1}$ by (10) and (11)

6. Calculate $X_{i,k+1|k+1}^+$ use (23) and return to Step 2 with $k = k + 1$

Outputs: $\hat{x}_{k+1|k+1}$, $P_{k+1|k+1}$, $X_{i,k+1|k+1}^+$
</div>

## 3. Methodology

By considering the unmodeled dynamic uncertainty and employing a dimensionindependent SUF, the RCKF outperforms CKFs for high-dimensional state estimation prob lems. However, the RCKF inherits the LMMSE-based observation assimilation framework of the ${ \mathrm { K F } } ,$ which shows obviously degraded performance in the occurrence of time-varying measurement noise and sporadic outliers. In this section, in order to address the non stationary heavy-tailed noise coming from time-varying GNSS measurement outliers, an improved RCKF is developed by employing the exponential multiplication form of two likelihood functions.

In the information fusion of the GNSS/INS, $v _ { k }$ suffers from time-varying non-Gaussian noise, which can be formulated as:

$$
p (\boldsymbol {z} _ {k} | \boldsymbol {x} _ {k}, y _ {k}) = \left(N \Big (\boldsymbol {z} _ {k}; \hat {\boldsymbol {z}} _ {k | k - 1}, R _ {k} \Big)\right) ^ {y _ {k}} \Big (N \Big (\boldsymbol {z} _ {k}; \hat {\boldsymbol {z}} _ {k | k - 1}, \widetilde {R} _ {k} \Big) \Big) ^ {1 - y _ {k}}\tag{24}
$$

where $y _ { k }$ is a binary indicator variable, $y _ { k } = 1$ denotes there is no measurement outlier, and the measurement covariance $R _ { k }$ satisfies $p ( R _ { k } | z _ { 1 : k } ) = I W \Big ( R _ { k } ; t _ { k | k } , T _ { k | k } \Big )$ ; otherwise, the measurement covariance is updated with a large-value $\widetilde { R } _ { k } = R _ { k } / \lambda _ { k } ,$ which indicates the occurrence of measurement outliers. $I W ( \cdot )$ denotes the inverse Wishart distribution and the prediction PDF of $R _ { k }$ satisfies $p ( R _ { k } | z _ { 1 : k - 1 } ) = I W \Big ( R _ { k } ; t _ { k | k - 1 } , T _ { k | k - 1 } \Big )$ , with $t _ { k | k - 1 } = \rho \Big ( t _ { k - 1 | k - 1 } - p - 1 \Big ) + p + 1 , T _ { k | k - 1 } = \rho T _ { k - 1 | k - 1 } ,$ , where $\rho ~ \in ~ ( 0 , 1 ]$ coincides with the assumption that the covariance changes slowly. In order to estimate the hyperparameters $y _ { k }$ and $\lambda _ { k } ,$ the following hierarchical priors are imported:

$$
p (y _ {k} | \pi_ {k}) = \pi_ {k} ^ {y _ {k}} (1 - \pi_ {k}) ^ {1 - y _ {k}}\tag{25}
$$

$$
p (\pi_ {k}) = B e (\pi_ {k}; e _ {0}, 1 - e _ {0})\tag{26}
$$

$$
p (\lambda_ {k}) = G \Bigl (\lambda_ {k}; \frac {\nu_ {k}}{2}, \frac {\nu_ {k}}{2} \Bigr)\tag{27}
$$

where $\pi _ { k }$ is the weight of different PDFs, $\lambda _ { k }$ is the scale coefficient of $R _ { k } ,$ , and $\lambda _ { k } \ll 1$ $\nu _ { k }$ is the degree of freedom (dof) parameter; $B e ( \cdot )$ and $G ( \cdot )$ denote the Beta and Gamma distribution, respectively.

In the framework of Bayesian filtering, the posterior PDF can be calculated by $p ( \phi _ { k } | z _ { 1 : k } , \Psi _ { k } ) = p ( z _ { 1 : k } | \phi _ { k } ) p \bar { ( \phi _ { k } | \Psi _ { k } ) } / p ( z _ { 1 : k } | \Psi _ { k } )$ , where $\begin{array} { l l l } { \phi _ { k } } & { = } & { \left\{ x _ { k } , y _ { k } , \pi _ { k } , \lambda _ { k } , R _ { k } \right\} } \end{array}$ and $\Psi _ { k } = \left\{ t _ { k | k - 1 } , T _ { k | k - 1 } , e _ { 0 } , \nu _ { k } \right\}$ . Suppose that $p \big ( \boldsymbol { \Phi } _ { k } \big | \boldsymbol { z } _ { 1 : k } , \boldsymbol { \Psi } _ { k } \big )$ can be approximated by a tractable distribution $q ( \Phi _ { k } ) ;$ by importing (25)–(27) we obtain:

$$
\begin{array}{c} q (\varPhi_ {k}) = q (x _ {k} | z _ {1: k - 1}) q (z _ {k} | x _ {k}, y _ {k}) q (R _ {k}) q (y _ {k} | \pi_ {k}) q (\pi_ {k}) q (\lambda_ {k}) \\ = N \Big (x _ {k}; \hat {x} _ {k | k - 1}, P _ {k | k - 1} \Big) \Big (N \Big (z _ {k}; \hat {z} _ {k | k - 1}, R _ {k} \Big) \Big) ^ {y _ {k}} \Bigg (N \Big (z _ {k}; \hat {z} _ {k | k - 1}, \frac {R _ {k}}{\lambda_ {k}} \Big) \Bigg) ^ {1 - y _ {k}} \\ \times \pi_ {k} ^ {y _ {k}} (1 - \pi_ {k}) ^ {1 - y _ {k}} B e (\pi_ {k}; e _ {0}, 1 - e _ {0}) G \Big (\lambda_ {k}; \frac {\nu_ {k}}{2}, \frac {\nu_ {k}}{2} \Big) I W \Big (R _ {k}; t _ {k | k - 1}, T _ {k | k - 1} \Big) \end{array}\tag{28}
$$

Suppose that ∆ is an arbitrary element of $\Phi _ { k }$ and $q ( \Delta )$ denotes the posterior PDF of $\Delta .$ The approximation solution for $p ( \Delta )$ is [28]:

$$
\log q (\Delta) = \mathrm{E} _ {\Phi_ {k} ^ {(- \Delta)}} [ \log q (\Phi_ {k}) ] + c _ {\Delta}\tag{29}
$$

where $\mathrm { E } [ \cdot ]$ is referred to the expectation operation, $\phi _ { k } ^ { ( - \Delta ) }$ denotes the set of elements of $\Phi _ { k }$ except for $\Delta ,$ and $c _ { \Delta }$ represents the constant terms that are independent of $\Delta$ .

Assume $\Delta = x _ { k }$ and substitute (28) into (29) to obtain:

$$
\begin{array}{r l} & {\log q ^ {(j + 1)} (\boldsymbol {x} _ {k}) = - 0. 5 \left(\boldsymbol {x} _ {k} - \hat {\boldsymbol {x}} _ {k | k - 1}\right) ^ {T} P _ {k | k - 1} ^ {- 1} \left(\boldsymbol {x} _ {k} - \hat {\boldsymbol {x}} _ {k | k - 1}\right) - 0. 5 \Bigl \{\mathrm{E} ^ {(j)} [ y _ {k} ]} \\ & {\quad + \left(1 - \mathrm{E} ^ {(j)} [ y _ {k} ]\right) \mathrm{E} ^ {(j)} [ \lambda_ {k} ] \Bigr \} \left(\boldsymbol {z} _ {k} - \hat {\boldsymbol {z}} _ {k | k - 1}\right) ^ {\boldsymbol {T}} \mathrm{E} ^ {(j)} \left[ R _ {k} ^ {- 1} \right] \left(\boldsymbol {z} _ {k} - \hat {\boldsymbol {z}} _ {k | k - 1}\right) + c _ {\boldsymbol {x} _ {k}}} \end{array}\tag{30}
$$

where j is the fixed-point iteration index in the variational parameters update. The posterior PDF of $q ^ { ( j + 1 ) } ( { \pmb x } _ { k } )$ is subject to Gaussian distribution and can be calculated based on $q ^ { ( j ) } \left( \Phi _ { k } ^ { ( - x _ { k } ) } \right)$ . If $\begin{array} { r } { q ^ { ( j + 1 ) } ( x _ { k } ) = \dot { N } \Big ( x _ { k } ; \hat { x } _ { k | k } ^ { ( j + 1 ) } , P _ { k | k } ^ { ( j + 1 ) } \Big ) } \end{array}$ , then $\hat { x } _ { k | k } ^ { ( j + 1 ) } , P _ { k | k } ^ { ( j + 1 ) }$ can be updated based on (14)-(16) with the moments calculated by (9)-(13), where $R _ { k }$ is replaced with:

$$
\overline {{R}} _ {k} ^ {(j + 1)} = \left(\mathrm{E} ^ {(j)} \left[ R _ {k} ^ {- 1} \right]\right) ^ {- 1} / \left\{\mathrm{E} ^ {(j)} [ y _ {k} ] + \left(1 - \mathrm{E} ^ {(j)} [ y _ {k} ]\right) \mathrm{E} ^ {(j)} [ \lambda_ {k} ] \right\}\tag{31}
$$

Assume $\Delta = y _ { k }$ and substitute (28) into (29) to yield:

$$
\begin{array}{c} \log q ^ {(j + 1)} (y _ {k}) = y _ {k} \Bigl \{\mathrm{E} ^ {(j)} [ \log \pi_ {k} ] - 0. 5 \mathrm{tr} \Bigl (B _ {k} ^ {(j + 1)} \mathrm{E} ^ {(j)} \left[ R _ {k} ^ {- 1} \right] \Bigr) \Bigr \} + (1 - y _ {k}) \\ \times \Bigl \{\mathrm{E} ^ {(j)} [ \log (1 - \pi_ {k}) ] + 0. 5 m \mathrm{E} ^ {(j)} [ \log \lambda_ {k} ] - 0. 5 \mathrm{E} ^ {(j)} [ \lambda_ {k} ] \mathrm{tr} \Bigl (B _ {k} ^ {(j + 1)} \mathrm{E} ^ {(j)} \left[ R _ {k} ^ {- 1} \right] \Bigr) \Bigr \} + c _ {y _ {k}} \end{array}\tag{32}
$$

where $\boldsymbol { B } _ { k } ^ { ( j + 1 ) } = \mathrm { E } ^ { ( j + 1 ) } \left[ \left( \boldsymbol { z } _ { k } - \hat { \boldsymbol { z } } _ { k | k - 1 } \right) \left( \boldsymbol { z } _ { k } - \hat { \boldsymbol { z } } _ { k | k - 1 } \right) ^ { T } \right]$ , with the result that:

$$
\begin{array}{r} P ^ {(j + 1)} (y _ {k} = 0) = \mathrm{A} \cdot \exp \Bigl \{\mathrm{E} ^ {(j)} [ \log (1 - \pi_ {k}) ] + 0. 5 m \mathrm{E} ^ {(j)} [ \log \lambda_ {k} ] \\ - 0. 5 \mathrm{E} ^ {(j)} [ \lambda_ {k} ] \mathrm{tr} \Bigl (B _ {k} ^ {(j + 1)} \mathrm{E} ^ {(j)} [ R _ {k} ^ {- 1} ] \Bigr) \Bigr \} \end{array}\tag{33}
$$

$$
P ^ {(j + 1)} \left(y _ {k} = 1\right) = \mathrm{A} \cdot \exp \left\{\mathrm{E} ^ {(j)} \left[ \log \pi_ {k} \right] - 0. 5 \operatorname{tr} \left(B _ {k} ^ {(j + 1)} \mathrm{E} ^ {(j)} \left[ R _ {k} ^ {- 1} \right]\right) \right\}\tag{34}
$$

where A is the normalizing constant and $\operatorname { t r } ( \cdot )$ is the trace operation of a matrix. Since $q ^ { ( j + 1 ) } ( y _ { k } )$ is subject to Bernoulli distribution, the expectation of $y _ { k }$ can be calculated as:

$$
\mathrm{E} ^ {(j + 1)} [ y _ {k} ] = \frac {P ^ {(j + 1)} (y _ {k} = 1)}{P ^ {(j + 1)} (y _ {k} = 0) + P ^ {(j + 1)} (y _ {k} = 1)}\tag{35}
$$

Assume $\Delta = \pi _ { k } ,$ and substitute (28) into (29) to obtain:

$$
\begin{array}{c} \operatorname{Log} q ^ {(j + 1)} (\pi_ {k}) = \left\{\operatorname{E} ^ {(j + 1)} [ y _ {k} ] + e _ {0} - 1 \right\} \log \pi_ {k} \\ + \left\{2 - e _ {0} - \operatorname{E} ^ {(j + 1)} [ y _ {k} ] - 1 \right\} \log (1 - \pi_ {k}) + c _ {\pi_ {k}} \end{array}\tag{36}
$$

where the posterior PDF $q ^ { ( j + 1 ) } ( \pi _ { k } )$ is defined as a Beta distribution, i.e., $q ^ { ( j + 1 ) } ( \pi _ { k } ) = \operatorname { \bar { \cal B } } e \left( \pi _ { k } ; e _ { k } ^ { ( j + 1 ) } , f _ { k } ^ { ( j + 1 ) } \right)$ , with the result that:

$$
e _ {k} ^ {(j + 1)} = e _ {0} + \mathrm{E} ^ {(j + 1)} [ y _ {k} ]\tag{37}
$$

$$
f _ {k} ^ {(j + 1)} = 2 - e _ {0} - \mathrm{E} ^ {(j + 1)} [ y _ {k} ]\tag{38}
$$

$$
\mathrm{E} ^ {(j + 1)} [ \log \pi_ {k} ] = \Psi \left(e _ {k} ^ {(j + 1)}\right) - \Psi \left(e _ {k} ^ {(j + 1)} + f _ {k} ^ {(j + 1)}\right)\tag{39}
$$

$$
\mathrm{E} ^ {(j + 1)} [ \log (1 - \pi_ {k}) ] = \Psi \left(f _ {k} ^ {(j + 1)}\right) - \Psi \left(e _ {k} ^ {(j + 1)} + f _ {k} ^ {(j + 1)}\right)\tag{40}
$$

where $\Psi ( \cdot )$ denotes the Digamma function.

Assume $\Delta = \lambda _ { k }$ and substitute (28) into (29) to yield:

$$
\begin{array}{c} \operatorname{Log} q ^ {(j + 1)} (\lambda_ {k}) = \left\{0. 5 p \Big (1 - \operatorname{E} ^ {(j + 1)} [ y _ {k} ] \Big) + 0. 5 \nu_ {k} - 1 \right\} \log \lambda_ {k} \\ - \left\{0. 5 \mathrm{tr} \Big (B _ {k} ^ {(j + 1)} \operatorname{E} ^ {(j)} \left[ R _ {k} ^ {- 1} \right] \Big) \Big (1 - \operatorname{E} ^ {(j + 1)} [ y _ {k} ] \Big) + 0. 5 \nu_ {k} \right\} \lambda_ {k} + c _ {\lambda_ {k}} \end{array}\tag{41}
$$

where the posterior PDF $\begin{array} { r } { q ^ { ( j + 1 ) } ( \lambda _ { k } ) = G \Big ( \lambda _ { k } ; \alpha _ { k } ^ { ( j + 1 ) } , \beta _ { k } ^ { ( j + 1 ) } \Big ) } \end{array}$ , with the result that:

$$
\alpha_ {k} ^ {(j + 1)} = 0. 5 p \left(1 - \mathrm{E} ^ {(j + 1)} [ y _ {k} ]\right) + 0. 5 \nu_ {k}\tag{42}
$$

$$
\beta_ {k} ^ {(j + 1)} = 0. 5 \mathrm{tr} \Big (B _ {k} ^ {(j + 1)} \mathrm{E} ^ {(j)} \left[ R _ {k} ^ {- 1} \right] \Big) \Big (1 - \mathrm{E} ^ {(j + 1)} [ y _ {k} ] \Big) + 0. 5 \nu_ {k}\tag{43}
$$

$$
\operatorname{E} ^ {(j + 1)} [ \lambda_ {k} ] = \frac {\alpha_ {k} ^ {(j + 1)}}{\beta_ {k} ^ {(j + 1)}}\tag{44}
$$

$$
\mathrm{E} ^ {(j + 1)} [ \log \lambda_ {k} ] = \Psi \left(\alpha_ {k} ^ {(j + 1)}\right) - \log \beta_ {k} ^ {(j + 1)}\tag{45}
$$

Assume $\Delta = R _ { k }$ and substitute (28) into (29) to obtain:

$$
\begin{array}{r l} & {\mathrm{Log} q ^ {(j + 1)} (R _ {k}) = - 0. 5 \Big (t _ {k | k - 1} + p + 2 \Big) \log | R _ {k} | - 0. 5 \mathrm{tr} \Big (\Big \{T _ {k | k - 1}} \\ & {+ \Big [ \mathrm{E} ^ {(j + 1)} [ \lambda_ {k} ] + \Big (1 - \mathrm{E} ^ {(j + 1)} [ \lambda_ {k} ] \Big) \mathrm{E} ^ {(j + 1)} [ y _ {k} ] \Big ] B _ {k} ^ {(j + 1)} \Big \} R _ {k} ^ {- 1} \Big) + c _ {R _ {k}}} \end{array}\tag{46}
$$

where the posterior PDF $q ^ { ( j + 1 ) } ( R _ { k } ) = I W \Bigl ( R _ { k } ; t _ { k | k } ^ { ( j + 1 ) } , T _ { k | k } ^ { ( j + 1 ) } \Bigr )$ , with the result that:

$$
t _ {k | k} ^ {(j + 1)} = t _ {k | k - 1} + 1\tag{47}
$$

$$
T _ {k | k} ^ {(j + 1)} = T _ {k | k - 1} + \left[ \mathrm{E} ^ {(j + 1)} [ \lambda_ {k} ] + (1 - \mathrm{E} ^ {(j + 1)} [ \lambda_ {k} ]) \mathrm{E} ^ {(j + 1)} [ y _ {k} ] \right] B _ {k} ^ {(j + 1)}\tag{48}
$$

$$
\mathrm{E} ^ {(j + 1)} \left[ R _ {k} ^ {- 1} \right] = t _ {k | k} ^ {(j + 1)} / T _ {k | k} ^ {(j + 1)}\tag{49}
$$

The steps for the ROCRCKF based on measurement outlier adaptive detection can be summarized as Algorithm 2. Notably, the final output $\mathrm { E } ^ { ( j + 1 ) } \left\lceil R _ { k } ^ { - 1 } \right\rceil$ and $q ^ { ( j + 1 ) } ( { \pmb x } _ { k } )$ are employed in the calculation of $\xi _ { i , k + 1 . }$ , making the resampling-free SUF insensitive to the time-varying noise and measurement outliers. Moreover, the observability-constrained resampling-free SUF combats the inconsistent estimation problem of outlier-robust filters by directly modifying the sigma points. The computational complexity of the RCKF has been discussed in our previous work [31], and is on the same order as the CKF in terms of floating-point operations. The difference between the ROCRCKF and the RCKF lies in the measurement update step only, and the increasing complexity of the ROCRCKF depends on the selection of $J _ { m a x } ,$ , which is a tradeoff between expected accuracy and efficiency.

<div class="mineru-algorithm" style="white-space: pre-wrap; font-family:monospace;">
Algorithm 2 The pseudocode of ROCRCKF

Inputs: $\hat{x}_{k-1|k-1}$, $P_{k-1|k-1}$, $Q_{k-1}$, $R_k$, $e_0$, $v_k$, $t_{k-1|k-1}$, $T_{k-1|k-1}$, $J_{max}$

1. Initialize $\widetilde{X}_{i,k-1|k-1}^+$ based on CKF and update $X_{i,k-1|k-1}^+$ by (18)

Time update:

2. Calculate $\hat{x}_{k|k-1}$, $P_{k|k-1}$, $X_{i,k+1|k}$ follow RCKF and propagate $t_{k|k-1}$, $T_{k|k-1}$

Measurement update

3. Initialization: $\hat{x}_{k|k}^{(1)} = \hat{x}_{k|k-1}$, E$^{(1)}[\lambda_k] = 1$, E$^{(1)}[\log \lambda_k] = 0$, E$^{(1)}[R_k^{-1}] = t_{k-1|k-1} T_{k-1|k-1}^{-1}$

E$^{(1)}[\log \pi_k] = \Psi(e_0) - \Psi(1)$, E$^{(1)}[\log(1 - \pi_k)] = \Psi(1 - e_0) - \Psi(1)$, E$^{(1)}[y_k] = 1$

For $j = 1: J_{max}$

4. Update $q^{(j+1)}[x_k]$ as Gaussian distribution based on (30)

Calculate $\overline{R}_k^{(j+1)}$ by (31), and update $\hat{x}_{k|k}^{(j+1)}$, $P_{k|k}^{(j+1)}$ by (10)-(12)

5. Update $q^{(j+1)}[y_k]$ as Bernoulli distribution based on (32)

Calculate $B_k^{(j+1)}$ and update E$^{(j+1)}[y_k]$ by (35)

6. Update $q^{(j+1)}[\pi_k]$ as Beta distribution based on (36)

Calculate $a_k^{(j+1)}$, $f_k^{(j+1)}$ by (37) and (38)

Calculate E$^{(j+1)}[\log \pi_k]$, E$^{(j+1)}[\log(1 - \pi_k)]$ by (39) and (40)

7. Update $q^{(j+1)}[\lambda_k]$ as Gamma distribution based on (41)

Calculate $\alpha_k^{(j+1)}$, $\beta_k^{(j+1)}$, E$^{(j+1)}[\lambda_k]$ and E$^{(j+1)}[\log \lambda_k]$ by (42)-(45)

8. Update $q^{(j+1)}[R_k]$ as inverse Wishart distribution based on (46)

Calculate $t_{k|k}^{(j+1)}$, $T_{k|k}^{(j+1)}$ by (47), (48) and update E$^{(j+1)}[R_k^{-1}]$ by (49)

End for

9. Update: $\hat{x}_{k|k} = \hat{x}_{k|k}^{(j+1)}$, $P_{k|k} = P_{k|k}^{(j+1)}$, $t_{k|k} = t_{k|k}^{(j+1)}$, $T_{k|k} = T_{k|k}^{(j+1)}$

10. Update $X_{i,k|k}^+$ by (23) and return to Step 2 with $k = k + 1$

Outputs: $\hat{x}_{k|k}$, $P_{k|k}$, $X_{i,k|k}^+$, $t_{k|k}$, $T_{k|k}$
</div>

## 4. Experiment Results and Analysis

Although the VB-based adaptive and robust filters for outlier detection and non-Gaussian noise have been widely tested by numerical simulation in existing works $[ 2 8 , 2 9 , 3 6 ] ,$ , they are seldom tested in practical application. We employ a car-mounted GNSS/INS to verify the effectiveness of the ROCRCKF in this section, where two trajectories with different maneuver modes and measurement outages are investigated.

## 4.1. Filter Model of the GNSS/INS

The dynamic model GNSS/INS is based on the error propagation model of the INS. In this paper, the navigation state is expressed in an Earth-centered, Earth-fixed frame (eframe), the body frame (b-frame) is defined as right-forward-up, and the IMU is fixed to the vehicle axis, which is aligned with the b-frame. The state at time index k can be written as $\begin{array} { r } { \boldsymbol { x } _ { k } = \left\lceil \delta \mathbf { { \varPsi } } ^ { T } \quad \delta \boldsymbol { v } ^ { T } \quad \delta \boldsymbol { p } ^ { T } \quad \boldsymbol { b } _ { a } ^ { T } \quad \boldsymbol { b } _ { g } ^ { T } \right\rceil ^ { T } } \end{array}$ , which denotes the error of attitude, velocity, position, and biases of the triaxial accelerometer and gyroscope, respectively. The corresponding error propagation model of the GNSS/INS is formulated as:

$$
\delta \dot {\pmb {\Psi}} = - \pmb {\omega} _ {i e} ^ {e} \times \delta \pmb {\Psi} + C _ {b} ^ {e} \pmb {b} _ {g}\tag{50}
$$

$$
\delta \dot {\pmb {v}} = - \left(C _ {b} ^ {e} \pmb {f} _ {i b} ^ {b}\right) \times \delta \pmb {\Psi} - 2 \pmb {\omega} _ {i e} ^ {e} \times \delta \pmb {v} + \frac {2 g _ {0}}{r _ {e S} ^ {e}} \frac {p}{| p | ^ {2}} \pmb {p} ^ {T} + C _ {b} ^ {e} \pmb {b} _ {a}\tag{51}
$$

$$
\delta \dot {\pmb {p}} = \delta \pmb {v}, \dot {\pmb {b}} _ {g} = 0, \dot {\pmb {b}} _ {a} = 0\tag{52}
$$

where the time is omitted for brevity; ${ \omega } _ { i e } ^ { e }$ is the angular rate of the Earth, which is expressed with respect to the geocentric inertial frame (i-frame) and resolved in the e-frame; $C _ { b } ^ { e }$ denotes the rotation transform from the b-frame to the e-frame; $f _ { i b } ^ { b }$ represents the specific force measured by accelerometers with respect to the i-frame resolved in the b-frame; g indicates the local gravity and $r _ { e S } ^ { e }$ is the geocentric radius at the Earth’s surface; a× denotes the skew symmetric matrix of vector ${ a } ;$ and $a ^ { T }$ and $| a |$ denote the transpose and magnitude value of a, respectively.

The measurement model of the GNSS/INS can be written as:

$$
\delta \mathbf {z} _ {k, G N S S / I N S} = \binom{\widetilde {\boldsymbol {p}} _ {G, k} - \widetilde {\boldsymbol {p}} _ {I, k} - \widetilde {C} _ {b} ^ {e} l _ {b a} ^ {b}}{\widetilde {\boldsymbol {v}} _ {G, k} - \widetilde {\boldsymbol {v}} _ {I, k} - \widetilde {C} _ {b} ^ {e} \Big (\widetilde {\boldsymbol {\omega}} _ {i b} ^ {b} \times l _ {b a} ^ {b} \Big) + \boldsymbol {\omega} _ {i e} ^ {e} \times \widetilde {C} _ {b} ^ {e} l _ {b a} ^ {b}}\tag{53}
$$

where a indicates a contains a measurement error; $\widetilde { p } _ { G , k }$ and $\widetilde { v } _ { G , k }$ are the position and velocity measurements from the GNSS receiver resolved in the e-frame; $\widetilde { \pmb { p } } _ { I , k } , \widetilde { \pmb { v } } _ { I , k } , \widetilde { C } _ { b } ^ { e } ,$ and $\widetilde { \omega } _ { i b } ^ { b }$ are the solutions of the INS after state error correction; $l _ { b a } ^ { b }$ is the lever arm of the antenna with respect to the IMU resolved in the b-frame. The dynamical model of the GNSS/INS can be formulated based on (50)–(52), while Equation (53) formulates the measurement model [39].

## 4.2. Experiment and Result Analysis

The experimental vehicle and setups for the field test are shown in Figure 1, where the raw sensor data of the NovAtel SPAN system were first recorded, then post-processed by the Inertial Explorer (IE) software of NovAtel and proposed as nonlinear filters, respectively. In cases where the SPAN system operates in the GNSS/INS tightly coupled mode, it outputs the position and heading continuously within 2 cm and 0.02 degrees, which are taken as the reference trajectories in this work. The IMU of the involved SPAN system includes a triaxial interferometric fiber optic gyroscope and a triaxial MEMS accelerometer; the specific parameters of the IMU are listed in Table 1. The frequencies of the GNSS, IMU, and<sup>EVIEW</sup> <sup>11</sup> <sup>of</sup> <sup>2</sup> reference trajectory are 5 Hz, 200 Hz, and 100 Hz.

![](images/a2c13d12f6b3f613d19578edef7cb6e9f233e29d96397416ea220af39265fb5f.jpg)  
<sup>Figure</sup> <sup>1.</sup> <sup>Experimental</sup> <sup>setup</sup> <sup>for</sup> <sup>field</sup> <sup>test</sup> <sup>data</sup> <sup>recording</sup>Figure 1. Experimental setup for field test data recording.

<sup>Table</sup> <sup>1.</sup> <sup>Parameter</sup> <sup>information</sup> <sup>of</sup> <sup>the</sup> <sup>IMU</sup>Table 1. Parameter information of the IMU.

<table><tr><td>Sensor</td><td>Characteristic</td><td>Value</td></tr><tr><td rowspan="3">Gyroscope</td><td>Rate bias</td><td>1 deg/h</td></tr><tr><td>Rate scale factor</td><td>100 ppm</td></tr><tr><td>Angular random walk</td><td>0.07 deg/ $\sqrt{h}$ </td></tr><tr><td rowspan="2">Accelerometer</td><td>Bias</td><td>0.3 mg</td></tr><tr><td>Scale factor</td><td>300 ppm</td></tr></table>

In this section, two trajectories are employed to verify the observability-constrainedIn this section, two trajectories are employed to verify the observability-constrained resampling-free SUF and outlier-robust measurement update of the ROCRCKF. Theresampling-free SUF and outlier-robust measurement update of the ROCRCKF. The VBCKF [19], MCCKF [23], and RCKF [31] are employed as comparisons to verify the suVBCKF [19], MCCKF [23], and RCKF [31] are employed as comparisons to verify the superiority of the ROCRCKF in terms of adaptivity and robustness. The forgetting factor is set as $\rho = 1 - e x p ( - 4 )$ and the iteration number is $J _ { m a x } = 2 0$ for the VBCKF and ROCRCKF. The other parameters of the ROCRCKF are $e _ { 0 } = 0 . 8 5$ , the dof $\begin{array} { r } { \boldsymbol { v } _ { k } = 5 , t _ { 0 | 0 } = p + 1 + \tau , } \end{array}$ $T _ { 0 | 0 } = \tau \cdot R _ { 0 } ,$ and $\tau = 3$ $R _ { 0 }$ denotes the initial measurement noise covariance for the VBCKF and ROCRCKF, and is also the constant noise covariance for the CKF, MCCKF, and RCKF. The kernel bandwidth and iteration number of the MCCKF are selected as 400 and 4. The same $\Lambda _ { k }$ is set for the RCKF and ROCRCKF, where for position and velocity, the corresponding $\eta _ { \ell }$ values are set as 1.01 and the other diagonal elements of $\Lambda _ { \mathrm { k } }$ are set as 0. The other parameters in the filter settings, except for the above-mentioned parameters, share the same values, and all of the parameters are pre-determined by a series of tuning attempts to make a fair comparison.

The first trajectory is shown in Figure 2, where there are frequent GNSS outages in the test, during which the filters work on prediction mode and thus the SUF based on the approximated moments of the posterior PDF is less efficient. The raw IMU output is shown on the right panel, indicating that the vehicle is still at the initial stage of the test, where ω and $f$ denote the angular rate and specific force. Notably, because of the turning and vibration of the land vehicle there are obvious outliers in the observations of the IMU. Figures $3 { - } 5$ compare the attitude and position error of different nonlinear filters. Notably, the ROCRCKF showcases better performance than the RCKF in terms of attitude, which not only converges faster at the initial stage but also achieves smaller stability errors. It is well recognized that the gyroscope bias in the direction of gravity is unobservable when the land vehicle runs along a straight horizontal line at constant velocity [40]. Consequently, the heading error accumulates with time in cases where the land vehicle runs along a straight line at a constant speed.

![](images/062f7430928b5af6f68ec1960415c3c3cedd2e0249a4d8912d3b9641646ce00a.jpg)  
(a)

![](images/5460c1774c30d4191fbcf7843bae1b32a5b644634387bb0e0003592d10f2f6b7.jpg)

![](images/5eb6388554126d8ec903f491144a0913e9bbc9397f53a3e25e2fc836434fb4b9.jpg)  
(b)  
Figure 2. Raw observations of the GNSS and IMU data of trajectory I. (a) Trajectory of the field testFigure 2. Raw observations of the GNSS and IMU data of trajectory I. (a) Trajectory of the field test with frequent GNSS outages, some of which are flagged in the rectangle and shown in an enlargedwith frequent GNSS outages, some of which are flagged in the rectangle and shown in an enlarged <sup>view;</sup> <sup>(b)</sup> <sup>Raw</sup> <sup>IMU</sup> <sup>output</sup> <sup>data</sup> <sup>indicating</sup> <sup>a</sup> <sup>stop</sup> <sup>at</sup> <sup>the</sup> <sup>initial</sup> <sup>stage</sup> <sup>of</sup> <sup>the</sup> <sup>trajectory</sup>view; (b) Raw IMU output data indicating a stop at the initial stage of the trajectory.

![](images/63774bcf1d9ce5404bfb0dbb954d37a144291ea3e254cd31c80f3d90f6389544.jpg)  
(a)

![](images/053039961a2fb5b25ae11ec6996d555b27fd690bb919acedb6ba14bc8ef5a9ab.jpg)  
(b)

Figure 3. Result of atitude estimation for trajectory I. (a) The roll error of diferent nonlinear filters;Figure 3. Result of attitude estimation for trajectory I. (a) The roll error of different nonlinear filters;<sup>EVIEW</sup> <sup>13</sup> <sup>of</sup> <sup>20</sup> (b) The pitch error of diferent nonlinear filters.(b) The pitch error of different nonlinear filters.  
![](images/b9fc891a422e5f3c6652a29ec624e4a38d4fe71c329b9a4c5f7793703192a600.jpg)  
Figure 4. Result of heading estimation for trajectory I.Figure 4. Result of heading estimation for trajectory I.

25The output trajectories of different nonlinear filters for trajectory I are shown in <sup>CKF</sup>Figure 6. As we can see from the regional enlarged views in Figure 6, both the RCKF and <sup>20</sup> <sub>MCCKF</sub>ROCRCKF showcase better position results than the VBCKF and CKF during the turn, <sup>VBCKF</sup>which is not only because the accelerometer bias is well compensated but also because <sup>ROCRCKF</sup>the velocity propagation error is reduced by the small heading error. The observability 10of the heading is low even during steady turns, whereas its accuracy can be improved <sup>8</sup> <sup>10</sup>due to the enhanced observability of tilt angles. Notably, the RCKF performs better than <sup>5</sup>the ROCRCKF in the occurrence of Outage #2, where the outage duration is short enough (less than 10 s) and there are sufficient observations for the RCKF converge to the steady state before another outage. However, in cases where the outage duration is increased, e.g., <sup>-5</sup>longer than 30 s, as shown in Outage #3, once the observations are occasionally provided, the RCKF cannot assimilate the innovation efficiently and shows degraded performance compared to the CKF and VBCKF. The sigma points of the CKF, VBCKF, and MCCKF -15depend on the time update stage of the KF framework, which leads to filtering divergence <sup>0</sup> <sup>300</sup> <sup>400</sup> <sup>500 0</sup> <sup>100</sup> <sup>200</sup> <sup>300</sup> <sup>400</sup> <sup>500</sup>gradually because of the uncompensated dynamic uncertainty. However, this is not the case for resampling-free SUFs, which modify the sigma points based on (23) by considering the state observability and propagating more information by using the prediction residue <sub>of the dynamic model.</sub>Figure 4. Result of headi

![](images/db2f2f4e73b69ea068e59a24f3759989a51326a642d3431b6154082d9d5228b2.jpg)  
(a)

![](images/f045c1e0e6881719d783b74a0bc728106f3399ff26ed56bd6b7e5f9f0674d055.jpg)  
(b)

Figure 5. Result of position estimation for trajectory I. (a) The east position error of diferent nonlin-Figure 5. Result of position estimation for trajectory I. (a) The east position error of different nonlinear <sub>ear</sub> <sub>filters;</sub> <sub>(b)</sub> <sub>The</sub> <sub>north</sub> <sub>position</sub> <sub>error</sub> <sub>of</sub> <sub>diferent</sub> <sub>nonlinear</sub> <sub>filfilters; (b) The north position error of different nonlinear filters.</sub>ing the state observability and propagating more informa  
![](images/9e959355cdc02e765a51ea60927540da3e90fcbac968b0f92bd34b4e66c60c64.jpg)  
<sup>Figure</sup> <sup>6.</sup> <sup>Output</sup> <sup>trajectories</sup> <sup>of</sup> <sup>diferent</sup> <sup>nonlinear</sup> <sup>filters</sup> <sup>for</sup> <sup>trajectory</sup> <sup>I.</sup>Figure 6. Output trajectories of different nonlinear filters for trajectory I.

More specific estimation results of different filters are listed in Table 2. The root mean square error (RMSE) is formulated as:square error (RMSE) is formulated as:

$$
\mathrm{RMSE} _ {\boldsymbol {x}} = \sqrt {\frac {1}{M} \sum_ {k = 1} ^ {M} \left(\hat {\boldsymbol {x}} _ {k} - \boldsymbol {x} _ {k}\right) ^ {2}}\tag{54}
$$

and �ො<sub>௞</sub> and �<sub>௞</sub> denote the estimated and<sub>where</sub> M <sub>is the total number of data sets,</sub> $x$ eference values at time k, respectively. The av-indicates the corresponding navigation state, erag<sub>and</sub> $\hat { \ v x } _ { k }$ <sup>MSE</sup><sub>and</sub> $x _ { k }$ <sup>RMSE)</sup> <sup>is</sup> <sup>taken</sup> <sup>as</sup> <sup>the</sup> <sup>performance</sup> <sup>metric</sup> <sup>for</sup> <sup>the</sup> <sup>po</sup><sub>denote the estimated and reference values at time</sub> $k ,$ <sup>on</sup> <sup>result,</sup> <sup>which</sup> <sup>is</sup><sub>respectively. The</sub> <sup>defined</sup> <sup>as:</sup> average RMSE(ARMSE) is taken as the performance metric for the position result, which is defined as:

$$
\mathrm{ARMSE} _ {p o s} = \sqrt {\frac {1}{2} \left(\mathrm{RMSE} _ {E P} ^ {2} + \mathrm{RMSE} _ {N P} ^ {2}\right)}\tag{55}
$$

where RMSE and RMSE denote the RMSE of positions in the east and north directions. Notably, in the enlarged image of the left panel of Figure 5, the ROCRCKF converges quickly at the initial stage of the trajectory in cases where the vehicle does not move. By efficiently assimilating the innovation and relieving the covariance inconsistence problem, the ROCRCKF shows better attitude and heading results than its resampling SUF-based counterparts. However, as there is no extra correction information for the states of strong observability during GNSS outages, the ROCRCKF does not improve the position and velocity results compared with the RCKF.

Table 2. The RMSEs of different nonlinear filters for trajectory I.

<table><tr><td>Method</td><td>Roll (°)</td><td>Pitch (°)</td><td>Heading (°)</td><td>ARMSEpos(m)</td></tr><tr><td>CKF</td><td>0.045</td><td>0.059</td><td>0.44</td><td>2.61</td></tr><tr><td>RCKF</td><td>0.045</td><td>0.065</td><td>0.17</td><td>2.50</td></tr><tr><td>MCCKF</td><td>0.059</td><td>0.080</td><td>0.38</td><td>5.48</td></tr><tr><td>VBCKF</td><td>0.046</td><td>0.059</td><td>0.48</td><td>2.53</td></tr><tr><td>ROCRCKF</td><td>0.030</td><td>0.041</td><td>0.04</td><td>2.55</td></tr></table>

The second trajectory is presented in Figure 7, where the navigation state observability is time-varying and the observability of the attitude is enhanced by frequent maneuver variation. The same parameters as that of trajectory I are employed in the filter configuration and only the observations are different for the comparative analysis of the observability constrained resampling-free SUF. The results in terms of attitude and position are shown in Figures 8–10. Notably, compared with the CKF, the VBCKF achieves slightly better position results but not better attitude results. Both the RCKF and ROCRCKF outperform the VBCKF in terms of attitude and position, and the MCCKF suffers from GNSS outages more obviously than the CKF and VBCKF, which coincides with the results shown in trajectory I. The MCCKF re-scales the uncertainty propagation of measurement updates based on the kernel function, whose performance heavily depends on the selection of kernel parameters, which is less effective in handling non-stationary non-Gaussian noise. Notably, in cases where the car is stopped, the observability constraint causes the direct observable state to approach the actual condition of the vehicle maneuver, which can be seen from the position result in Figure 10. However, the correction information of attitude error and sensor biases comes from prediction covariance, and both the RCKF and the ROCRCKF achieve better attitude results than their resampling-based SUF counterparts.

The output trajectories of different nonlinear filters for trajectory II are shown in Figure 11. As we can see from the regional enlarged views in Figure 11, the ROCRCKF showcases better position results than the RCKF and VBCKF during the turn (Outage #2), which coincides with the result presented in trajectory I. The MCCKF showcases better results than the RCKF, which may be because the MCCKF assimilates the occasionally provided observations efficiently, and the unmodeled dynamic model can be neglected when the vehicle runs in a straight line. The heading accuracy is improved due to the enhanced observability of tilt angles during the frequent turns shown in Figure 11, and thus during Outage #1 all the algorithms achieve acceptable position results. More specific results are listed in Table 3. Notably, the heading and position errors of the CKF are reduced from 2.6<sup>◦</sup> and 7.3 m to 1.96<sup>◦</sup> and 6.38 m by the RCKF, where the position improvement is more significant than that of the VBCKF. The RCKF reduces the $\mathbf { A R M S E } _ { p o s }$ of the VBCKF by 10.5%, where the corresponding value for the first trajectory is 1.2%, which confirms that the resampling-free SUF based on the dynamic prediction residue propagates more information than the SUF based on approximated Gaussian moments. Furthermore, the ROCRCKF outperforms the RCKF and VBCKF in both attitude and position results, where it reduces the position error by 6.7% and 16.5%, respectively. Compared with the results in Table 2, the attitude and heading results of the ROCRCKF shown in Table 3 do not degrade as heavily as those of the RCKF and VBCKF using the same filter configuration, which<sup>and</sup> <sup>the</sup> <sup>ROCRCKF</sup> <sup>achieve</sup> <sup>beter</sup> <sup>atitude</sup> <sup>results</sup> <sup>than</sup> <sup>their</sup> <sup>resampling-based</sup> <sup>SUF</sup> <sup>coun-</sup>and the ROCRCKF achieve beter atitude results than their resampling-based SUF coundemonstrates the adaptivity and robustness of the ROCRCKF.<sup>terparts.</sup> terparts.

![](images/ac338a187b98c360c4e94ce5d8b9f505933927600bee66bf5666ea99fc2fec58.jpg)  
(a)(a)

![](images/b81b41af2ae13ee2f0feee5e96203650cd60628cab19b9410fe94de488b5c44e.jpg)

![](images/c7e0b78736e0ad1981b4ad7313830d90fe0980e8835b260b9145bbb2af2a333d.jpg)  
(b)(b)

Figure 7. Raw observations of GNSS and IMU data of trajectory II. (a) Trajectory of field test withFigure 7. Raw observations of GNSS and IMU data of trajectory II. (a) Trajectory of field test with GNSS outages, some of which are flagged in the rectangle and shown in an enlarged view; (b) RawGNSS outages, some of which are flagged in the rectangle and shown in an enlarged view; (b) RawGNSS outages, some of which are flagged in the rectangle and shown in an enlarged view; (b) Raw <sup>IMU</sup> <sup>output</sup> <sup>data</sup> <sup>indicating</sup> <sup>there</sup> <sup>are</sup> <sup>two</sup> <sup>still</sup> <sup>periods</sup> <sup>in</sup> <sup>the</sup> <sup>trajectory</sup>IMU output data indicating there are two still periods in the trajectory.IMU output data indicating there are two still periods in the trajectory  
![](images/3d17e2a06ac6e2f53cdb6e9d6e893915467bf93efcbbc4595755da9884024f29.jpg)  
(a)(a)

![](images/bbaaaf41969710920a598bdff98a7cde32a02a462531723a26c5f7a195063622.jpg)  
(b)(b)  
Figure 8. Results of attitude estimation for trajectory II. (a) The roll error of different nonlinear filters; (b) The pitch error of different nonlinear filters.

![](images/b1456e9012a11caadfc5eba8837adabef94301f2fed86bf570c1a48347abd08f.jpg)  
Figure 9. Results of heading estimation for trajectory II.Figure 9. <sub>Resu</sub>l<sub>ts</sub> <sub>o</sub>f h<sub>ea</sub>d<sub>ing</sub> <sub>estimation</sub> f<sub>or</sub> <sub>trajectory</sub> <sub>II.</sub><sup>Figure</sup> <sup>9.</sup> <sup>Results</sup> <sup>of</sup> <sup>heading</sup> <sup>estimation</sup> <sup>for</sup> <sup>trajectory</sup> <sup>II.</sup>

![](images/a6c535332548a097faabc94445a6ef2687d713ae6bf94adf202e58d02dda5dd0.jpg)  
<sub>(a)</sub>(a)it r

![](images/143b4198e0cc15cb44526a005239ef2c2aed85df033cee011793304569178871.jpg)  
<sub>(b)</sub>(b)ive

Figure 10. Results of position estimation for trajectory II. (a) The east position error of diferent non-Figure 10. Results of position estimation for trajectory II. (a) The east position error of different <sub>linear</sub> <sub>filters;</sub> <sub>(b)</sub> <sub>The</sub> <sub>north</sub> <sub>position</sub> <sub>error</sub> <sub>of</sub> <sub>di</sub>f<sub>erent</sub> <sub>nonlinear</sub> <sub>filters.</sub> <sup>linear</sup> <sup>filters;</sup> <sup>(b)</sup> <sup>The</sup> <sup>north</sup> <sup>position</sup> <sup>error</sup> <sup>of</sup> <sup>diferent</sup> <sup>nonlinear</sup> <sup>filters.</sup> nonlinear filters; (b) The north position error of different nonlinear filters.  
![](images/06562e858e48817a8403541713dba7475342eed7480274a31d251d95acea8bab.jpg)  
Figure 11. Output trajectories of diferent nonlinear filters for trajectory II.Figure 11. Output trajectories of different nonlinear filters for trajectory II.

Table 3. The RMSEs of different nonlinear filters for trajectory II.

<table><tr><td>Method</td><td>Roll (°)</td><td>Pitch (°)</td><td>Heading (°)</td><td>ARMSEpos(m)</td></tr><tr><td>CKF</td><td>0.091</td><td>0.16</td><td>2.60</td><td>7.30</td></tr><tr><td>RCKF</td><td>0.057</td><td>0.10</td><td>1.96</td><td>6.38</td></tr><tr><td>MCCKF</td><td>0.105</td><td>0.15</td><td>2.00</td><td>6.18</td></tr><tr><td>VBCKF</td><td>0.097</td><td>0.16</td><td>2.94</td><td>7.13</td></tr><tr><td>ROCRCKF</td><td>0.058</td><td>0.08</td><td>0.27</td><td>5.95</td></tr></table>

## 5. Discussion

Notably, the RCKF does not outperform the CKF and VBCKF in terms of roll and pitch in case the maneuver of vehicle is not high, whereas it achieves better position and heading results, especially when there is a GNSS outage. On one hand, the RCKF relieves the covariance inconsistence problem of the CKF by setting the $\Lambda _ { k }$ when constructing the posterior sigma points, which makes the prediction covariance less sensitive to the change in innovation; on the other hand, it cannot assimilate the observation information efficiently once the measurement noise is small enough. The VBCKF improves the position result slightly compared with the CKF, which, however, does not outperform the CKF in terms of attitude and heading. This result coincides with the fact that the correction information of attitude comes from the off-diagonal element of prediction state covariance, which is updated at the prediction stage of nonlinear filters. The estimation results of the MCCKF have a direct relation to the duration of the outage, and in the field test based on trajectory I, it achieves similar heading results as the CKF but much worse position results, which indicates that the MCC-based measurement update cannot address the uncertainty propagation in cases of frequent GNSS outages. Generally, the measurement outliers are sparse in the GNSS/INS, and in cases where the noise is subject to Gaussian distribution, the MCC-based measurement update is sub-optimal and does not perform well. However, in Table 3, unlike the result listed in Table 2, the MCCKF achieves better heading and position results than the CKF and VBCKF, which indicates that the uncertainty or non-Gaussian process noise from the dynamic model has a significant effect on the state estimation of the GNSS/INS.

Notably, in Figures 5 and 10 the position error of the ROCRCKF in the north direction are noisy, and the position error may increase abruptly when the observations reappear after a GNSS outage. The reason behind these phenomena may be that the measurement outlier in different direction behaves differently, whereas the same scalar scale $\lambda _ { k }$ is applied for $\mathrm { E } ^ { ( j ) } [ R _ { k } ]$ . The estimation results of the binary indicator $\mathrm { E } ^ { ( j ) } [ y _ { k } ]$ and corresponding noise covariance of the position for trajectory II are shown in Figure 12. Notably, the GNSS outages are detected accurately as they coincide with the outages indicated in the left panel of Figure 7. Moreover, the noise covariance diagonal element is increased in the presence of missing observations. Although there is no state update during the GNSS outage, the parameters in $\Phi _ { k }$ are propagated in the prediction model during these periods, which is also helpful for the sigma point direct modification of the ROCRCKF. As we can see, based on the same filter model, filter configuration, and sensors, the state estimation result of trajectory II is worse than that of trajectory I, which indicates the unmodeled dynamic uncertainty has a significant effect on the state estimation of the GNSS/INS. The comparison analysis between trajectory I and II indicates that the $\Lambda _ { k }$ should be adjusted according to the instantaneous state observability, and thus the observability quantifying analysis based on the real-time degree of observability calculation is under consideration.

![](images/4b5cd30c188e16276fc7ee5a1a7b34419eda730d6761f4c48f4fdd956c3474c0.jpg)  
Figure 12. Result of binary indicator and measurement noise covariance estimation.Figure 12. Result of binary indicator and measurement noise covariance estimation.

## 6. Conclusions

This article presented an improved observability-constrained resampling-free CKF, the ROCRCKF, for the state estimation of a GNSS/INS with measurement outliers. The binary indicator of outliers, scale parameter of noise covariance, and state are estimated simultaneously based on variational Bayesian inferences. The proposed algorithm is verified by employing a car-mounted GNSS/INS, where two trajectories of different state observability are analyzed and compared. Experiment results demonstrate that, cases where there are frequent GNSS outages, the ROCRCKF outperforms the RCKF and VBCKF in terms of attitude without obvious position degradation. Moreover, when there is high uncertainty in the dynamic model prediction, the ROCRCKF achieves much better results than the RCKF and VBCKF in terms of heading and position results. The vehicle dynamic model-derived measurement should also be taken into consideration for the information fusion of land vehicle navigation, and the dynamic model prediction residue employed in the resampling-free SUF provided a new measure for the vehicle dynamic identification. In future works, multiple indicators of outliers should be employed, as the measurement outliers and noise characteristics may be independent in the observation domain.

Author Contributions: Conceptualization, B.C. and D.W.; methodology, B.C. and X.W.; software, B.C. and Z.S.; validation, B.C. and D.W.; formal analysis, X.W.; investigation, W.C.; resources, B.C. and Y.Z.; data curation, Y.L.; writing—original draft preparation, B.C.; writing—review and editing, W.C. and X.W.; supervision, W.C.; funding acquisition, B.C. and D.W. All authors have read and agreed to the published version of the manuscript.

Funding: This research was jointly funded by the Natural Science Foundation of China, grant number 32271999, the Jiangsu Province and Education Ministry Co-sponsored Synergistic Innovation Center of Modern Agricultural Equipment, grant number XTCX2009, the Primary Research & Development Plan of Jiangsu Province, grant number BE2021313, the Key Laboratory of Spectroscopy Sensing, Ministry of Agriculture and Rural Affairs, grant number 2022ZJUGP002, the University Grants Committee of Hong Kong under the scheme Research Impact Fund on the project R5009-21, and the Research Institute of Land and Space, Hong Kong Polytechnic University. The APC was funded by XTCX2009.

Data Availability Statement: The data presented in this study are available on request from the corresponding author. The data are not publicly available due to the privacy of the subjects involved in the study.

Acknowledgments: We thank the reviewers and editors for their insightful comments that helped to improve the quality of the paper a great deal

Conflicts of Interest: The authors declare no conflict of interest.

## References

1. Zhang, Q.; Chen, Q.J.; Xu, Z.P.; Zhang, T.S.; Niu, X.J. Evaluating the navigation performance of multi-information integration based on low-end inertial sensors for precision. Precis. Agric. 2021, 22, 627–646. [CrossRef]

2. Falco, G.; Nicola, M.; Pini, M. Positioning based on tightly coupled multiple sensors: A practical implementation and experimental assessment. IEEE Access 2018, 6, 13101–13116. [CrossRef]

3. Kalman, R.E. A new approach to linear filtering and prediction problems. Trans. ASME D J. Basic Eng. 1960, 82, 35–45. [CrossRef]

4. Schmidt, S.F. The Kalman filter: Its recognition and development for aerospace application. J. Guid. Control 1981, 4, 4–7. [CrossRef]

5. Julier, S.J.; Uhlman, J.K.; Durrant-Whyte, H.F. A new method for the nonlinear transformation of means and covariances in filters and estimation. IEEE Trans. Autom. Control 2000, 45, 477–482. [CrossRef]

6. Ito, K.; Xiong, K.Q. Gaussian filters for nonlinear filtering problems. IEEE Trans. Autom. Control 2000, 45, 910–927. [CrossRef]

7. Arasaratnam, I.; Haykin, S. Cubature Kalman filters. IEEE Trans. Autom. Control 2009, 54, 1254–1269. [CrossRef]

8. Crassidis, J.L. Sigma-point Kalman filtering for integrated GPS and inertial navigation. IEEE Trans. Aerosp. Electron. Syst. 2006, 42, 750–756. [CrossRef]

9. Zhao, Y.W. Performance evaluation of cubature Kalman filter in a GPS/IMU tightly-coupled navigation system. Signal Process. 2016, 119, 67–79. [CrossRef]

10. Li, Z.; Li, S.; Liu, B.; Yu, S.S.; Shi, P. A stochastic event-triggered robust cubature Kalman filter filtering approach to power system dynamic state estimation with non-Gaussian measurement noise. IEEE Trans. Control Syst. Technol. 2023, 31, 889–896. [CrossRef]

11. Arulampalam, M.S.; Maskell, S.; Gordon, N.; Clapp, T. A tutorial on particle filters for online nonlinear/non-Gaussian Bayesian tracking. IEEE Trans. Signal Process. 2002, 50, 174–188. [CrossRef]

12. Schön, T.; Gustafsson, F.; Nordlund, P.-J. Marginalized particle filters for mixed linear/nonlinear state-space models. IEEE Trans. Signal Process. 2002, 53, 2279–2289. [CrossRef]

13. Vouch, O.; Minetto, A.; Falco, G.; Dovis, F. On the adaptivity of unscented particle filter for GNSS/INS tightly-integrated navigation unit in urban environment. IEEE Access 2021, 9, 144157–144170. [CrossRef]

14. Gao, G.L.; Gao, B.B.; Gao, S.S.; Hu, G.G.; Zhong, Y.M. A hypothesis test-constrained robust Kalman filter for INS/GNSS integration with abnormal measurement. IEEE Trans. Veh. Technol. 2023, 72, 1662–1673. [CrossRef]

15. Hu, G.G.; Gao, B.B.; Zhong, Y.M.; Gu, C.F. Unscented Kalman filter with process noise covariance estimation for vehicular INS/GPS. Inf. Fusion 2020, 64, 194–204. [CrossRef]

16. Gao, X.L.; Luo, H.Y.; Ning, B.K.; Zhao, F.; Bao, L.F.; Gong, Y.L.; Xiao, Y.M.; Jiang, J.G. RL-AKF adaptive Kalman filter navigation algorithm based on reinforcement learning for ground vehicles. Remote Sens. 2020, 12, 1704. [CrossRef]

17. Xue, C.; Huang, Y.L.; Zhu, F.C.; Zhang, Y.G.; Chambers, J.A. An outlier-robust Kalman filter with adaptive selection of elliptically contoured distributions. IEEE Trans. Signal Process. 2022, 70, 994–1009. [CrossRef]

18. Sun, J.; Ye, Q.Q.; Lei, Y. In-motion alignment method of SINS based on improved Kalman filter under geographic latitude uncertainty. Remote Sens. 2022, 14, 2581. [CrossRef]

19. Särkkä, S.; Nummenmaa, A. Recursive adaptive Kalman filtering by variational Bayesian approximations. IEEE Trans. Autom. Control 2009, 54, 596–600. [CrossRef]

20. Cui, B.B.; Chen, X.Y.; Xu, Y.; Huang, H.Q.; Liu, X. Performance analysis of improved iterated cubature Kalman filter and its application to GNSS/INS. ISA Trans. 2017, 66, 460–468. [CrossRef]

21. Huang, Y.L.; Zhang, Y.G.; Wu, Z.M.; Li, N.; Chambers, J. A novel adaptive Kalman filter with inaccurate process and measurement noise covariance matrices. IEEE Trans. Autom. Control 2018, 63, 594–601. [CrossRef]

22. Chang, L.B.; Li, K.L.; Hu, B.Q. Huber’s M-estimation-based process uncertainty robust filter for integrated INS/GPS. IEEE Sens. J. 2015. 15, 3367–3374. [CrossRef]

23. Chen, B.D.; Liu, X.; Zhao, H.Q.; Principe, J.C. Maximum correntropy Kalman filter. Automatica 2017, 76, 70–77. [CrossRef]

24. Liu, D.; Chen, X.Y.; Xu, Y.; Liu, X.; Shi, C.F. Maximum correntropy generalized high-degree cubature Kalman filter with application to the attitude determination system of missile. Aerosp. Sci. Technol. 2019, 95, 105441. [CrossRef]

25. Shmaliy, Y.S. Linear optimal FIR estimation of discrete time-invariant state-space models. IEEE Trans. Signal Process. 2010, 58, 3086–3096. [CrossRef]

26. Shmaliy, Y.S. Predictive tracking under persistent disturbances and data error using H FIR approach. IEEE Trans. Ind. Electron. 2022, 69, 6121–6129. [CrossRef]

27. Nurminen, H.; Ardeshiri, T.; Piché, R.; Gustafsson, F. Skew-t filter and smoother with improved covariance matrix approximation. IEEE Trans. Signal Process. 2018, 66, 5618–5633. [CrossRef]

28. Zhu, H.; Zhang, G.R.; Li, Y.F.; Leung, H. A novel robust Kalman filter with unknown non-stationary heavy-tailed noise. Automatica 2021 127 [ f]

29. Huang, Y.L.; Zhang, Y.G.; Zhao, Y.; Chambers, J.A. A novel robust Gaussian-Student’s t mixture distribution based Kalman filter. IEEE Trans. Signal Process. 2019 67 [ f]

30. Fu, H.P.; Huang, W.; Li, Z.W.; Cheng, Y.M.; Zhang, T.Y. Robust cubature Kalman filter with Gaussian-multivariate Laplacian mixture distribution and partial variational Bayesian method. IEEE Trans. Signal Process. 2023, 71, 847–858. [CrossRef]

31. Cui, B.B.; Chen, X.Y.; Tang, X.H. Improved cubature Kalman filter for GNSS/INS based on transformation of posterior sigmapoints error. IEEE Trans. Signal Process. 2017, 65, 2975–2987. [CrossRef]

32. Wang, M.S.; Wu, W.Q.; Zhou, P.Y.; He, X.F. State transformation extended Kalman filter for GPS/SINS tightly coupled integration. GPS Solut. 2018, 22, 112. [CrossRef]

33. Yang, Y.L.; Huang, G.Q. Observalility analysis of aided INS with heterogeneous features of points, lines, and planes. IEEE Trans Robot. 2019, 35, 1399–1418. [CrossRef]

34. Cui, B.B.; Wei, X.H.; Chen, X.Y.; Li, J.Y.; Wang, A.C. Robust cubature Kalman filter based on variational Bayesian and transformed posterior sigma points error. ISA Trans. 2019, 86, 18–28. [CrossRef]

35. Wang, G.W.; Cui, B.B.; Tang, C.Y. Robust cubature Kalman filter based on maximum correntropy and resampling-free sigma-point update framework. Digital Signal Process. 2022, 126, 103495. [CrossRef]

36. Wang, H.W.; Li, H.B.; Fang, J.; Wang, H.P. Robust Gaussian Kalman filter with outlier detection. IEEE Signal Process. 2018, 25, 1236–1240. [CrossRef]

37. Li, H.Q.; Medina, D.; Vilà-Valls, J.; Closas, P. Robust variational-based Kalman filter for outlier rejection with correlated measurement. IEEE Trans. Signal Process. 2021, 69, 357–369. [CrossRef]

38. Straka, O.; Dunik, J. Resampling-free stochastic integration filter. In Proceedings of the IEEE 23rd International Conference on Information Fusion (FUSION), Sun City, South Africa, 6–9 July 2020.

39. Groves, P.D. Principles of GNSS, Intertial, and Multisensory Integrated Navigation Systems, 2nd ed.; Artech House: Boston, MA, USA, 2008; pp. 382–393.

40. Hong, S.; Lee, M.H.; Chun, H.H.; Kwon, S.H.; Speyer, J.L. Observability of error states in GPS/INS integration. IEEE Trans. Veh. Technol. 2005, 54, 731–743. [CrossRef]

Disclaimer/Publisher’s Note: The statements, opinions and data contained in all publications are solely those of the individual author(s) and contributor(s) and not of MDPI and/or the editor(s). MDPI and/or the editor(s) disclaim responsibility for any injury to people or property resulting from any ideas, methods, instructions or products referred to in the content.