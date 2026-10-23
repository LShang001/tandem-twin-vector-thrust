ORIGINAL ARTICLE

![](images/b105717821e2f2af614548bbb1e1a1e6133bcb19d1a95ce14a606a98039374c0.jpg)

# Robust state and protection‑level estimation within tightly coupled GNSS/INS navigation system

Shuchen Liu<sup>1</sup>  · Kaizheng Wang<sup>1</sup> · Dirk Abel<sup>1</sup>

Received: 9 March 2022 / Accepted: 24 March 2023 / Published online: 20 April 2023 © The Author(s) 2023

## Abstract

In autonomous applications for mobility and transport, a high-rate and highly accurate vehicle-state estimation is achieved by fusing measurements of global navigation satellite systems (GNSS) and inertial sensors. The state estimation and its protection-level generation often sufer from satellite-signal disturbances in urban environments and subsequent poor parametrization of the satellite observables. Thus, we propose an innovative scheme involving an extended $H _ { \infty }$ filter (EHF) for robust state estimation and zonotope for the protection-level generation. This scheme is shown as part of a tightly coupled navigation system based on an inertial navigation system and aided by the GPS/Galileo dual-constellation satel lite navigation system. Specifically, GNSS pseudorange and deltarange observables are utilized. The experimental result of post-processing a real-world dataset show significant advantages of EHF against a conventional extended Kalman filter regarding the navigation accuracy and robustness under various GNSS measurement parametrizations and environmental circumstances. The zonotope-based protection-level calculation is proven valid, computationally afordable, and feasible for real-time implementations.

Keywords Tightly coupled GNSS/INS · State estimation · Protection level · $H _ { \infty }$ filtering · Zonotope

## Introduction

Recently, considerable eforts have been made in autonomous driving, as it ofers great potential in terms of safety and eficiency. For autonomous applications, navigation systems are indispensable, which shall robustly and accurately estimate the vehicle states, namely position, velocity, and orientation. Further, integrity is also considered an essential requirement for navigation systems in safety–critical applications (Bhatti and Ochieng 2007). Often, a high-rate and highly accurate vehicle-state estimation is achieved by fusing measurements of global navigation satellite systems (GNSS) and inertial navigation systems (INS) using Kalman filtering. In tightly coupled navigation systems, Kalman filter solutions cannot guarantee a reliable vehicle-state estimation under inexact noise statistics. In this case, an extended $H _ { \infty }$ filter (EHF) can be used alternatively, outperforming the Kalman filtering regarding system uncertainties (Simon 2006).

Moreover, facing various software and hardware failures in real-world operations (Bhatti and Ochieng 2007), navigation filters need to be extended with an integrity monitoring system. Besides detecting and excluding faulty measurements, the integrity system shall generate a guaranteed protection level (PL), which bounds the estimation error (Groves 2013). Robust filtering of GNSS/INS measurements for accurate state estimation, together with a valid PL generation in the urban environment, remains under investigation and is targeted.

For the state estimation in changing environments, where the modeling of the satellite-measurement noise is often inadequate, a robust approach against environmental disturbances is crucial. Gehrt et al. (2020) have proposed a Bayesian-optimization-based adaptive parameterizing scheme within a Kalman filter, which chooses suitable parameters depending on the operating environment. The usage of this approach is limited to the designed environmental features. Instead of optimizing the parameter anytime and anywhere, robust estimation methods insensitive to environmental changes have been studied. As an alternative to Kalman filters, Simon (2006) has presented the $H _ { \infty }$ filtering in detail, which still utilizes the conventional filter structure but shows significantly higher performance under system and environment uncertainties. In the context of GNSS/INS, Osman et al. (2019) have introduced a multi-sensor fusion localization approach using the EHF, where the wheel encoders, GPS, and visual odometry are fused within a loosely coupled structure. For tightly coupled GNSS/INS integration, Jwo and Chen (2003) have demonstrated the robustness of a hybrid $H _ { 2 } / H _ { \infty }$ filter against noise variations, assuming that the state transition is of constant-velocity kinematics. More recently, Smolyakov and Langley (2020) have proposed a novel algorithm combining $H _ { \infty }$ filter and reinforcement learning for robust positioning solutions in urban environments. Our study focuses on the practical implementation of an EHF on a tightly coupled GNSS/INS system based on a modern strapdown architecture, emphasizing its robust ness against various parameterizations and environmental circumstances.

For the PL generation, statistic approaches (Lee et al. 2018; Zhu et  al. 2018) are widely applied and studied, assuming that the statistical distribution of the estimation error is known. Groves (2013) has indicated that such error distribution assumptions are dificult to validate. To release the PL generation from the statistical distribution conjecture, Combastel (2003) proposed set-membership approaches, which only require process and measurement errors to be bounded. In the context of GPS navigation, Dbouk and Schön (2018) compared various bounding methods, including zonotope, for GPS integrity and pointed out that the zonotope method describes the uncertainty propagation in a more meaningful manner. Dbouk and Schön (2020) further derived an outlier detection algorithm by applying zonotopes as uncertainty references for GPS integrity monitoring. In terms of zonotopes combined with an estimation observer, Zhang et al. (2020) have developed a robust and intervalbased error estimation approach for descriptor systems and validated it in a simulation environment. Shetty and Gao (2021) recently conducted a nonlinear stochastic reachability analysis to predict the state uncertainty bounds for GNSS based unmanned aerial vehicle (UAV)-navigation, where probabilistic zonotopes, instead of conventional zonotopes, are applied as the set representation. The experimental validation was conducted in a simulated urban environment. More recently, Bhamidipati et al. (2022) applied zonotopes to represent GNSS shadows for 3D-map-aided GNSS localization. The feasibility and performance of using zonotope on GNSS/INS navigation, particularly on tightly coupled systems, remains to be researched.

Therefore, we propose a novel scheme combining the EHF and zonotope within the navigation system, involving a strapdown-based INS tightly coupled with GNSS pseudorange and deltarange observables. First, we use an

EHF to estimate the states robustly against inappropriate parametrization of the GNSS measurement noise. Then, we apply zonotope to generate PL for the estimated vehicle states. The evaluation of the proposed methods uses a semi/sub-urban environment dataset from a commercial land vehicle. The EHF is compared with the extended Kalman filter (EKF) regarding the sensitivity of the state estimation accuracy by grid searching a defined parameter space of GNSS measurement noise. Further, the state estimation accuracy of the EHF and EKF is compared by applying the well-known sigma-ε model (Wieser et al. 2005; Breuer et al. 2016; Konrad et al. 2018) with the same GNSS parameters. Regarding the feasibility of applying zonotopes in real-time applications, we first evaluate the computational time of the 3D (for position PL) and 9D (for PL of the full vehicle states) zonotopes and their overestimation efects under various parameters. Finally, we apply the Stanford-ESA Integrity Diagram to validate the zonotope-based position PL generation, checking the consistency of the estimated error bound with the ground truth. The result of zonotopebased PL estimation is briefly compared with the results in (Zhu et al. 2018), where the PL is estimated based on EKF measurement innovation by applying the sigma-ε model.

## Methodology

In the methodology part, we present the navigation scheme involving the EHF for the state estimation and zonotope for the PL generation. As shown in Fig. 1, the EHF is embedded into an existing GNSS/INS frame, introduced in the previous fundamental work (Konrad et al. 2018) from the same research group, including:

• The strapdown-based state and covariance prediction by applying the 100 Hz inertial measurement unit (IMU) measurements;

• The GNSS pre-processing, including the correction of various error terms;

• The handling of measurements at various rates with delay.

Additionally, GNSS fault detection and exclusion (FDE), proposed in previous works (Liu et al. 2019, 2021), is performed after the correction of GNSS observables, such that a set of fault-free GNSS measurements is fed into the EHF.

The methodology of the EHF and zonotope-based PL generation is divided into two parts. The first part provides fundamental EHF and zonotope theories in the Appendix at the end of the manuscript. It is recommended to read the Appendix if the reader is unfamiliar with the corresponding theories. The second part explains their integration into the tightly coupled GNSS/INS in the rest of this section. A summary of the implementation in pseudocode form is given at the end of this section.

Fig. 1 Overview of the designed tightly coupled navigation system, including transitions of the states, their covariances, and error bounds. The blue shadow marks the zonotope-relevant components. k denotes the index of the time epoch  
![](images/8ff1bc2912b12c229e0f1e0a1f4749fc67007784e51db65b2731266b39d2f539.jpg)

## State estimation applying extended $H _ { \infty }$ filter

Applying the EHF to estimate the vehicle’s state, i.e., the three-dimensional position, velocity, and attitude, the following state vector x is designed:

$$
\boldsymbol {x} = \left(\boldsymbol {p} _ {e A} ^ {e, T} \boldsymbol {v} _ {e A} ^ {n, T} \boldsymbol {q} _ {b} ^ {n, T} \boldsymbol {b} _ {a} ^ {T} \boldsymbol {b} _ {g} ^ {T} c _ {b} c _ {d}\right) ^ {T}\tag{1}
$$

where the index $\cdot _ { A } \cdot$ stands for the antenna body frame, $\cdot _ { b } \cdot$ for the IMU body frame, $\cdot _ { e } ,$ for ECEF coordinates, and $\cdot _ { n } \cdot$ for the navigation frame. $\pmb { p } _ { e A } ^ { e } \in \mathbb { R } ^ { 3 \times 1 }$ represents the position of the antenna body frame origin in the earth-centeredearth-fixed (ECEF) frame. $\pmb { \nu } _ { e A } ^ { n } \in \mathbb { R } ^ { 3 \times 1 }$ is the velocity of the antenna body frame origin with respect to the ECEF frame, expressed in the navigation frame north-east-down (NED) coordinates. A quaternion $\pmb q _ { b } ^ { n } \in \mathbb { R } ^ { 4 \times 1 }$ for the alignment of the IMU body frame with the navigation frame is estimated as the expression of vehicle attitude. Additionally, the accelerometer bias ${ \pmb b } _ { a }$ and gyroscope bias ${ \pmb b } _ { g } ,$ each of dimension ℝ<sup>3×1</sup>, are part of the state vector, which is necessary for IMU integration. Further, the receiver clock bias $c _ { a }$ and drift $c _ { d }$ are estimated, which are essential for tightly coupled GNSS integration.

As shown in Fig. 1, the state vector x and its covariance P are propagated within a strapdown-based INS at 100 Hz. Konrad et al. (2018) have introduced the prediction of vehicle kinematics by applying the IMU accelerometer and gyroscope measurements and the prediction of IMU bias and receiver clock bias, summarized as (A4). Meanwhile, predicting the covariance by applying (A5) requires the system matrix $\boldsymbol { F } _ { k }$ and the noise shaping matrix $G _ { k } ,$ the derivation of which is given in (Wendel 2009) or (Groves 2013).

Further, the predicted state vector and covariance are corrected by applying (A6)–(A8) whenever a new set of GNSS measurements is available. The GNSS measurements, i.e., pseudoranges and deltaranges, are introduced in the form of a measurement equation as (A2). By applying such nonlinear measurement equations in EHF, a linearization is required to calculate the measurement Jacobian matrix $\pmb { H } _ { k }$ , which is suficiently described in (Wendel 2009) or (Groves 2013).

## Measurement model

The measurement equation of pseudorange is given as follows:

$$
\tilde {\rho} = \pmb {p} _ {e s} ^ {e} - \pmb {p} _ {e A} ^ {e} + c _ {b} + c _ {s} + I _ {r} + T _ {r} + M _ {\rho} + \nu_ {\rho}\tag{2}
$$

where $\widetilde { \rho }$ is the measured scalar pseudorange of one satellite, $\pmb { p } _ { e s } ^ { e }$ and $\pmb { p } _ { e A } ^ { e }$ represent the position of a satellite (denoted by $\mathbf { \boldsymbol { \cdot } } _ { S } \mathbf { \boldsymbol { \cdot } } _ { } )$ ) and the antenna in ECEF coordinates displayed in ECEF, respectively. The terms $c _ { b } , c _ { s } , I _ { r } , T _ { r } , M _ { \rho } , \nu _ { \rho }$ in (2) represent the receiver and satellite clock errors, the ionospheric error, the tropospheric error, the multi-path error, and the additive measurement noise, respectively. These error sources are corrected with diferential correction data from a base station introduced in “Experiment setup” section.

The deltarange observation is calculated as follows:

$$
\widetilde {d} = \left(\pmb {e} _ {A s} ^ {n}\right) ^ {T} \left(\pmb {\nu} _ {e s} ^ {n} - \pmb {\nu} _ {e A} ^ {n}\right) + c _ {d} + \nu_ {d}\tag{3}
$$

where $\widetilde { d }$ is the measured scalar relative velocity between the antenna and satellite, known as the deltarange. The vector $e _ { A s } ^ { n }$ is the direction from the antenna to a satellite. The terms $\pmb { \nu } _ { e s } ^ { n }$ and $ { \boldsymbol { \nu } } _ { e A } ^ { n }$ represent the velocity of the satellite and antenna with respect to the navigation frame, respectively. The terms $c _ { d }$ and $\nu _ { d }$ represent the clock drift and measurement noise, respectively.

In practice, the GNSS observables are measured in the antenna body frame, while the strapdown algorithm propagates the system state vector in the IMU body frame, shown in Fig. 1. In the navigation filter, this requires a transformation from the defined vehicle states $\pmb { p } _ { e A } ^ { e }$ and $\pmb { \nu } _ { e A } ^ { n }$ to the required states $\pmb { p } _ { e b } ^ { e }$ and $ { \boldsymbol { \nu } } _ { e b } ^ { n }$ in the IMU body frame, shown as follows:

$$
\pmb {p} _ {e b} ^ {e} = \pmb {p} _ {e A} ^ {e} - \pmb {C} _ {n} ^ {e} \pmb {C} _ {b} ^ {n} \pmb {l} _ {A} ^ {b}\tag{4}
$$

$$
\pmb {v} _ {e b} ^ {n} = \pmb {v} _ {e A} ^ {n} - \pmb {C} _ {b} ^ {n} \big (\pmb {\omega} _ {e b} ^ {b} \times \pmb {l} _ {A} ^ {b} \big)\tag{5}
$$

where $C _ { n } ^ { e }$ and ${ C } _ { b } ^ { n }$ denote the direction cosine matrix for the vector rotation among coordinate frames, derived from the estimated antenna position $\pmb { p } _ { e A } ^ { e }$ and the estimated attitude $\pmb q _ { b } ^ { n }$ (Groves 2013), respectively. The transformation uses the lever arm $l _ { A } ^ { b }$ of the antenna position in the IMU body frame, measured and given in Section "Experiment setup", and considers the translation velocity of the antenna caused by its rotation $\pmb { \omega } _ { e b } ^ { b } .$ , derived from the IMU measurement and estimated IMU bias ${ \pmb { b } } _ { g }$ (Wendel 2009).

Equations (2) and (3) give the measurement equations of a single satellite and are applied to all observed satellites within the EHF as a measurement update step (Wendel 2009; Groves 2013; Konrad et al. 2018).

## Error‑bound estimation using zonotope

The core issue of the PL calculation is to bound the state estimation error $\boldsymbol { e } _ { k } .$ Analog to the filter state transition, the estimation of the state error bound $[ e _ { k } ]$ also contains a-priori propagation and a-posterior correction. The error dynamic of the EHF is given as follows:

$$
\pmb {e} _ {k} ^ {-} = \pmb {x} _ {k} - \widehat {\pmb {x}} _ {k} ^ {-} = \pmb {F} _ {k - 1} \pmb {e} _ {k - 1} ^ {+} + \pmb {G} _ {k} \pmb {w} _ {k - 1}\tag{6}
$$

$$
\pmb {e} _ {k} ^ {+} = \pmb {x} _ {k} - \widehat {\pmb {x}} _ {k} ^ {+} = \left(\pmb {I} - \pmb {K} _ {k} \pmb {H} _ {k}\right) \pmb {e} _ {k} ^ {-} - \pmb {K} _ {k} \pmb {v} _ {k}\tag{7}
$$

where $e _ { k } ^ { - }$ and $\boldsymbol { e } _ { \iota _ { \nu } } ^ { + }$ represent the a-priori and a-posterior estimation error. $\widehat { \pmb { x } } _ { k } ^ { \tilde { - } } , \widehat { \pmb { x } } _ { k } ^ { \tilde { + } }$ , and $\pmb { K } _ { k }$ are filter-relevant variables and defined in (A4)–(A8).

Assume that the initial estimation error $\mathbf { \boldsymbol { e } } _ { 0 } .$ , the process noise ${ \pmb w } _ { k }$ and the measurement noise $\nu _ { k }$ are zero-mean and bounded by corresponding zonotopes:

$$
\boldsymbol {e} _ {0} \in <   \mathbf {0}, \mathcal {E} _ {\mathbf {0}} >, w _ {k} \in <   \mathbf {0}, \mathcal {W} _ {k} >, v _ {k} \in <   \mathbf {0}, \mathcal {V} _ {k} >\tag{8}
$$

where $\pmb { \varepsilon _ { 0 } } , \pmb { \ w { w } } _ { k }$ , and $\nu _ { k }$ represent the generation matrix of corresponding zonotopes, defined in (A12). $\varepsilon _ { \mathfrak { g } }$ shall be initialized large enough, considering the large system uncertainties in the filter initialization stage. $w _ { k }$ and $\nu _ { k }$ are calculated as diagonal matrices with their diagonal elements equal to $n _ { \sigma }$ times the standard deviation of process noise $\sigma _ { w _ { k } }$ and measurement noise $\sigma _ { \nu _ { k } } ,$ respectively. $n _ { \sigma }$ is a pre-defined parameter corresponding to the confidence level of the resulting zonotope. Noteworthy, for the calculation of $w _ { k }$ and $\nu _ { k }$ we still apply the statistical distribution of the process and measurement noise, which is a zero-mean normal distribution. However, such distribution is not required by the implementing zonotope. Instead, zonotope only requires the lower and upper bound, which can be obtained from the sensor characteristics of the manufacturer or vehicle dynamic limitations. In general, the measurement boundaries are more convenient to obtain and validate in practice, which is one main advantage of set-membership approaches over statistic approaches.

After the zonotope is initialized with $\varepsilon _ { \mathbf { 0 } } .$ , the a-priori and a-posteriori zonotope estimations are derived analogously to (6) and (7). The matrices multiplication and matrices addition in (6) and (7) correspond to linear image ⊙ and Minkowski sum ⊕ in the zonotope calculation, explained with (A13) and (A14), respectively:

$$
\begin{array}{l} <   \boldsymbol {c} _ {e _ {k}} ^ {-}, \boldsymbol {\mathcal {E}} _ {k} ^ {-} > = \left(\boldsymbol {F} _ {k - 1} \odot <   \boldsymbol {c} _ {e _ {k - 1}} ^ {+}, \boldsymbol {\mathcal {E}} _ {k - 1} ^ {+} >\right) \oplus \\ \left(\boldsymbol {G} _ {k - 1} \odot <   \boldsymbol {0}, \boldsymbol {\mathcal {W}} _ {k - 1} >\right) \end{array}\tag{9}
$$

$$
\begin{array}{l} <   \boldsymbol {c} _ {e _ {k}} ^ {+}, \boldsymbol {\mathcal {E}} _ {k} ^ {+} > = \Big ((I - K _ {k} H _ {k}) \odot <   \boldsymbol {c} _ {e _ {k}} ^ {-}, \boldsymbol {\mathcal {E}} _ {k} ^ {-} > \Big) \oplus \\ (\boldsymbol {K} _ {k} \odot <   \boldsymbol {0}, \boldsymbol {\mathcal {V}} _ {k} >) \end{array}\tag{10}
$$

where $\pmb { c } _ { e _ { k } } ^ { - } , \pmb { c } _ { e _ { k } } ^ { + } , \pmb { \mathcal { E } } _ { k } ^ { - } , \pmb { \mathcal { E } } _ { k } ^ { + }$ are the center and generation matrix of the a-priori and a-posteriori estimated bounding zonotope for the state estimation error $\boldsymbol { e } _ { k }$ . Since $< c _ { e _ { k } } ^ { - } , \pmb { \mathcal { E } } _ { k } ^ { - } >$ and $< c _ { e _ { k } } ^ { + } , \pmb { \mathcal { E } } _ { k } ^ { + } >$ are iteratively updated using the zero-mean zonotopes $< 0 , \mathcal { E } _ { 0 } > , < { \bf 0 } , \mathcal { W } _ { k } > , < { \bf 0 } , \mathcal { V } _ { k } > ,$ the center of th resulting zonotope $c _ { e _ { k } } ^ { - }$ and $c _ { e _ { k } } ^ { + }$ shall be zero, which means

$$
\boldsymbol {e} _ {k} \in <   \mathbf {0}, \boldsymbol {\mathcal {E}} _ {k} >, \forall k \in \{1, \dots , N \}\tag{11}
$$

Therefore, only the estimation of the generation matrix $\pmb { \mathcal { E } } _ { k } ^ { - }$ and $\mathcal { E } _ { k } ^ { + }$ is necessary. Equations (9) and (10) result in the following zonotope propagation and update steps:

$$
\boldsymbol {\mathcal {E}} _ {k} ^ {-} = \mathcal {R} \left(\left[ \begin{array}{c c} \boldsymbol {F} _ {k - 1} \boldsymbol {\mathcal {E}} _ {k - 1} ^ {+} & \boldsymbol {G} _ {k - 1} \boldsymbol {\mathcal {W}} _ {k} \end{array} \right]\right)\tag{12}
$$

$$
\boldsymbol {\mathcal {E}} _ {k} ^ {+} = \mathcal {R} \left(\left[ \left(\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k}\right) \boldsymbol {\mathcal {E}} _ {k} ^ {-} \boldsymbol {K} _ {k} \boldsymbol {\mathcal {V}} _ {k} \right]\right)\tag{13}
$$

Each zonotope estimation carries out a zonotope reduction, explained with (A16) and denoted as $\cdot _ { \mathcal { R } } ,$ . The dimension of the resulting zonotope is specified by a user-defined reduction order q $( n \leq q \leq m )$ , namely $\mathcal { R } ( \mathcal { H } ) \in \mathbb { R } ^ { n \times q }$ . Without applying the zonotope reduction, the dimension of the generation matrices $\pmb { \mathcal { E } } _ { k } ^ { - }$ and $\mathcal { E } _ { k } ^ { + }$ increases without limit along the operation time, which is not afordable for computation units.

In the EHF, the state vector x contains 18 states, including the quaternion $\pmb q _ { b } ^ { n } \in \mathbb { R } ^ { 4 \times 1 }$ . By the estimation of the state error covariance P, the error of the orientation vector $\mathbb { R } ^ { 3 \times 1 }$ is considered (Wendel 2009), which makes $P \in \mathbb { R } ^ { 1 7 \times 1 7 }$ . Therefore, the error space of the EHF is 17-dimensional. It results in the generation matrix for the EHF $\pmb { \mathcal { E } } \in \mathbb { R } ^ { 1 7 \times q }$ . However, in practice, the PLs of some states could be undesired, e.g., PLs of the IMU bias. Suppose that the sequence of the index of the desired PLs in the EHF’s error space is given as follows:

$$
\pmb {\tau} = \left\{\tau_ {1}, \tau_ {2}, \dots , \tau_ {n} \right\}\tag{14}
$$

where n is the dimension of the desired zonotope. A selection matrix $\mathbf { T } \in \mathbb { R } ^ { n \times 1 7 }$ can be applied to the generation matrix calculation to achieve the state selection. $\Gamma _ { i j }$ denotes the element of the i<sup>th</sup> column and $\mathrm { j } ^ { \mathrm { t h } }$ raw in 𝚪 and is given as follows:

$$
\Gamma_ {i j} = \left\{ \begin{array}{l} 1, \text {   for   } i = j \in \tau \\ 0, \text {   otherwise } \end{array} , \forall i \in \{1, 2, \dots , n \}, \forall j \in \{1, 2, \dots , 1 7 \} \right.\tag{15}
$$

The calculation of the zonotope $< \mathbf { 0 } , \pmb { \mathcal { E } } _ { r , k } >$ to bound the selected state errors is given as follows:

$$
\boldsymbol {\mathcal {E}} _ {r, k} ^ {-} = \mathcal {R} \big (\left[ \boldsymbol {\Gamma} \boldsymbol {F} _ {k - 1} \boldsymbol {\Gamma} ^ {T} \boldsymbol {\mathcal {E}} _ {r, k - 1} ^ {+} \right. \left. \boldsymbol {\Gamma} \big (\boldsymbol {G} _ {k - 1} \boldsymbol {\mathcal {W}} _ {k - 1} \big) \right] \big)\tag{16}
$$

$$
\boldsymbol {\mathcal {E}} _ {r, k} ^ {+} = \mathcal {R} \big (\left[ \boldsymbol {\Gamma} \big (\boldsymbol {I} - \boldsymbol {K} _ {k} \boldsymbol {H} _ {k} \big) \boldsymbol {\Gamma} ^ {T} \boldsymbol {\mathcal {E}} _ {r, k} ^ {-} \right. \left. \boldsymbol {\Gamma} \big (\boldsymbol {K} _ {k} \boldsymbol {\mathcal {V}} _ {k} \big) \right] \big)\tag{17}
$$

In practice, the error bound of the position, velocity, and orientation could be of interest, which can be estimated by the nine-dimensional zonotope given as $\mathcal { E } _ { r , \mathfrak { H } } \in \mathbb { R } ^ { 9 \times q }$ . The zonotope could be further reduced to three-dimensional $\mathcal { E } _ { r , 3 D } \in \mathbb { R } ^ { 3 \times q }$ if only the PL of the position is required. In the present work, the three- and nine-dimensional zonotopes are implemented and discussed, which are achieved by selection matrices $\Gamma _ { 3 D } \in \mathbb { R } ^ { 3 \times 1 7 }$ and $\Gamma _ { 9 D } \in \mathbb { R } ^ { 9 \times 1 7 }$

$$
\pmb {\tau} _ {3 D} = \{1, 2, 3 \}, \pmb {\Gamma} _ {3 D} = \left[ \begin{array}{l l} \pmb {I} _ {3 \times 3} & \mathbf {0} _ {3 \times 1 4} \end{array} \right]\tag{18}
$$

$$
\pmb {\tau} _ {9 D} = \{1, 2, \dots , 9 \}, \pmb {\Gamma} _ {9 D} = \left[ \begin{array}{l l} \pmb {I} _ {9 \times 9} & \mathbf {0} _ {9 \times 8} \end{array} \right]\tag{19}
$$

These two specific zonotopes will be evaluated in the experiment section.

## Protection‑level generation and integrity risk

In general, the objective of estimating the PL is to find the boundary of estimation error under an integrity risk:

$$
\boldsymbol {P} \boldsymbol {L} _ {k} = \sup \left\{\left[ \boldsymbol {e} _ {k} \right] | P \left(\hat {\boldsymbol {x}} _ {k} - \boldsymbol {x} _ {k} \in \left[ \boldsymbol {e} _ {k} \right]\right) \leq 1 - P _ {I R} \right\}\tag{20}
$$

where $\mathbf { \boldsymbol { x } } _ { k }$ is the unknown ground truth of the state estimate $\hat { x } _ { k } . P _ { I R }$ denotes the probability of integrity risk. $P L _ { k }$ is the vector calculated for all desired states and corresponds to the dimension of 𝛕.

Assuming that (8) is satisfied, (11) indicates that the estimation error must be within the zonotope given as $< \mathbf { 0 } , \pmb { \mathcal { E } } _ { k } >$ and, therefore, must be within its interval hull. Thus, we estimate the desired protection level as the interval hull box of the resulting reduced zonotope $< \mathbf { 0 } , \pmb { \mathcal { E } } _ { r , k } >$ from (16) and (17) by applying (A15). In practice, specific PLs could be desired, e.g., horizontal PL (HPL) and vertical PL (VPL) for positioning. The HPL is calculated as the diagonal distance of the interval hull box corresponding to positioning error in the north and east direction, while the VPL is the interval hull of positioning error in the down direction.

The risk of initializing the zonotopes $< { \bf 0 } , \mathcal { E } _ { 0 } > , < { \bf 0 } , \mathcal { W } _ { k } > ,$ and $< { \bf 0 } , { \nu _ { \scriptscriptstyle k } } > \mathrm { c a n }$ be quantified with the pre-defined parameter $n _ { \sigma } .$ . However, the zonotope operations sustainably change the integrity risk, such as Minkowski sum, or even deterministically reduce the integrity risk, such as zonotope reduction and interval hull generation. Since the zonotope is independent of the statistical distributions after being initialized with (8), the resulting integrity risk $P _ { I R }$ is hard to analyze in theory. We evaluate the integrity risk of zonotope-based protectionlevel generation by applying the Stanford-ESA diagram with recorded real-world data.

## Summary of the implementation scheme

The integration of the zonotope into the filter structure is summarized as Alg. 1. It should be noted that Alg. 1 is a general expression that does not require the applied filter to be the EHF. Any other form of filter, e.g., the EKF or the unscented Kalman filter (UKF), could also be applied as long as its error dynamic is known. Applying another filter requires adjusting (6), (7), (16), and (17) according to the filter dynamic.

![](images/660af2784c854fd3021d14c074b275015a17c6e3ec3fc6e390331f845b18daa9.jpg)  
Fig. 2 Measurement setup equipped on a car: the antenna and IMU were mounted on the roof (left); the rest of the hardware was placed in the trunk (right)

<table><tr><td colspan="2">Algorithm 1 EHF Extended with Zonotope</td></tr><tr><td colspan="2">Initialization:</td></tr><tr><td colspan="2">EHF:  $x_0, P_0$ ; Zonotope:  $\mathcal{E}_0$ </td></tr><tr><td colspan="2">Propagation:</td></tr><tr><td>EHF:  $\hat{x}_{k-1} \to \hat{x}_k^-$ ,  $\hat{P}_{k-1} \to \hat{P}_k$ </td><td>(A4)(A5)</td></tr><tr><td>Zonotope:  $\mathcal{E}_{r,k-1} \to \mathcal{E}_{r,k}^-$ </td><td>(16)</td></tr><tr><td colspan="2">Measurement Update:</td></tr><tr><td colspan="2">if new measurements are available, then</td></tr><tr><td>EHF:  $\hat{P}_k^- \to \hat{P}_k^+, \hat{x}_k^- \to \hat{x}_k^+$ </td><td>(A6)(A7)(A8)</td></tr><tr><td>Zonotope:  $\mathcal{E}_{r,k}^- \to \mathcal{E}_{r,k}^+$ </td><td>(17)</td></tr><tr><td colspan="2">end if</td></tr><tr><td colspan="2">Output Generation:</td></tr><tr><td colspan="2">if new measurements are available, then</td></tr><tr><td colspan="2">EHF:  $\hat{P}_k = \hat{P}_k^+, \hat{x}_k = \hat{x}_k^+$ ; Zonotope:  $\mathcal{E}_k = \mathcal{E}_k^+$ </td></tr><tr><td colspan="2">else</td></tr><tr><td colspan="2">EHF:  $\hat{P}_k = \hat{P}_k^-$ ,  $\hat{x}_k = \hat{x}_k^-$ ; Zonotope:  $\mathcal{E}_k = \mathcal{E}_k^-$ </td></tr><tr><td colspan="2">end if</td></tr><tr><td>Protection Level:  $\mathcal{E}_{r,k} \to [\boldsymbol{e}_{r,k}]$ </td><td>(A15)</td></tr></table>

## Experimental evaluation

In this section, we first introduce the hardware and software setup for data recording, followed by the parameterization of the variables mentioned in the methodology. We designed and summarized three experiments using the dataset, providing their results in a separate section.

## Experiment setup

The designed approach is validated in a post-processing environment using the data recorded on a commercial land vehicle in a real-world test campaign. Figure 2 shows the experimental vehicle with the measurement setups for realtime data recording. The lever arm of the antenna position in the IMU body frame $l _ { A } ^ { b }$ is measured as follows:

$$
\boldsymbol {l} _ {A} ^ {b} = \left( \begin{array}{c c c} 0 & 0 & - 0. 1 1 3 1 \mathrm{m} \end{array} \right) ^ {T}\tag{21}
$$

which is applied in (4) and (5) for the lever arm compensation.

Table 1 gives a list of the hardware and the corresponding parametrization. Specifically, the parameterization of the measurement noise $( \nu _ { \rho }$ and $\nu _ { d } )$ variance matrix R for all GNSS observables is crucial. Breuer et al. (2016) and Konrad et al. (2018) apply the sigma-ε model for this purpose, which calculates the standard deviation for each pseudorange and deltarange as:

$$
\sigma_ {\rho} ^ {2} = C _ {\rho} ^ {2} \cdot 1 0 ^ {- \frac {\mathrm{C/N} _ {0}}{1 0}}, \sigma_ {d} ^ {2} = C _ {d} ^ {2} \cdot 1 0 ^ {- \frac {\mathrm{C/N} _ {0}}{1 0}}\tag{22}
$$

where $\mathrm { C } / \mathrm { N } _ { 0 }$ is the carrier-to-noise ratio in dB-Hz and provided by the GNSS receiver. $\mathrm { C } / \mathrm { N } _ { 0 }$ varies among satellites and with time. $C _ { \rho }$ and $C _ { d }$ are tuning parameters defined for the pseudorange and deltarange measurements. The choice of $C _ { \rho }$ and $C _ { d }$ is evaluated and discussed in the next section.

The initial estimation error covariance matrix of the EHF $P _ { 0 }$ is calculated by the initial standard deviation $\pmb { \sigma } _ { 0 }$ given in Table 2.

The reduction order parameter q of zonotope is introduced in (A16) and is discussed with experimental results later in this section. Besides, $n _ { \sigma }$ is set to 3 for the zonotope initialization.

## Experiment overview

In this section, the test drive of the experimental evaluation is described, followed by an overview of the designed experiments. Figure 3 shows the bird-eye view of the RTK reference trajectory on the Google-Map-satellite-view layer. The testing area locates in Aachen, Germany, and is a mix of boulevard, semi-urban, sub-urban, and highway environments. The detailed testing area division refers to (Nitsch et al. 2021), where the test drive was carried out in the same area but applied to another measurement setup.

The entire test drive takes 1514 s and is divided into three rounds, the overview of which is given in Table 3.

Combining Figs. 3, 4, and Table 3, in Round I, the vehicle drives at a speed of up to 50 km/h in an interferencelack environment, where the number of available satellites is stable, and the $\mathrm { C } / \mathrm { N } _ { 0 }$ plot indicates that the signal quality is high and stable. Round II and III are much more challenging for satellite navigation, with high-speed tracks up to 70 km/h, and the vehicle drives under bridges, as marked in Fig. 3. The changing number of visible satellites and their parametrization (Round I) and a dataset for validation (Round II and III). And following experiments are designed:

• Section Sensitivity analysis of estimation error vs. $C _ { \rho }$ and $C _ { d } \mathrm { : }$ Here, we analyze the sensitivity of the state estimation accuracy from the EHF and EKF by applying a grid search upon the parameter set $C _ { \rho }$ and $C _ { d } .$ . This experiment uses the dataset Round I and determines the $C _ { \rho }$ and $C _ { d }$ that are applied in the following experiments.

Section "Comparison between EHF and EKF regarding state estimation accuracy": Here, we evaluate the state estimation accuracy of EHF and EKF using the datasets Round II and III, with the determined parameters from Round I. The idea is to evaluate the robustness against sub-optimal parameter settings since the applied parameter set is determined in a diferent environment, namely Round I, rather than its actual operation environment, namely Round II and III, which contains much more signal disturbances.

Section "Validation of PL generation using zonotope": Here, we discuss the choice of reduction order q by applying three and nine-dimensional zonotopes and vali-

Table 1   List of the applied hardware and the specifications. The PPS is used for measurement delay compensation (Konrad et al. 2018)

<table><tr><td>Hardware</td><td>Function/measurement (rate)</td><td>Parametrization</td></tr><tr><td>dSPACE SCALEXIO Autobox</td><td>Real-time data recording (1000 Hz)</td><td>N/A</td></tr><tr><td>LORD MicroStrain 3DM®-GX5-25 IMU</td><td>Acceleration (100 Hz)Angular rate (100 Hz)</td><td>Process noise variance Q: Allan variance analysis in (Breuer et al. 2016)</td></tr><tr><td rowspan="2">Septentrio AstRx3 HDC GNSS receiver</td><td>Pseudorange (10 Hz)Deltarange (10 Hz)</td><td>Measurement noise variance R: sigma-ε model in (22)</td></tr><tr><td>Pulse per second (PPS, 1 Hz)Real-time-kinematic ground-truth solution (RTK, 10 Hz)</td><td>N/A</td></tr><tr><td>Raspberry Pi 4</td><td>Differential correction data from SAPOS® NRW (1 Hz)</td><td>N/A</td></tr></table>

Table 2 Standard deviations of the initial states in the EHF

<table><tr><td>state</td><td>Pos. e/n</td><td>Pos. d</td><td>Velocity</td><td>Orientation</td></tr><tr><td> $\sigma_0$ </td><td>5 m</td><td>10 m</td><td>1 m/s</td><td>5°</td></tr><tr><td>State</td><td>Acc. bias</td><td>Gyro. bias</td><td>clock drift</td><td>Clock bias</td></tr><tr><td> $\sigma_0$ </td><td>0.1 m/s2</td><td>0.01 °/s</td><td>1000 m</td><td>10 m/s</td></tr></table>

unstable $\mathrm { C } / \mathrm { N } _ { 0 }$ even caused failed positioning from the reference receiver for a long period of around 500 s.

Considering the characteristics of the current dataset, we split the entire test drive into a dataset for date the PL estimation by applying the Stanford-ESA Integrity Diagram using the validation dataset Round II and III.

Fig. 3 Bird-eye view of the reference trajectory: valid RTK reference position in blue; invalid non-RTK reference position in yellow; unavailable reference position in red, where the positioning solution is estimated by the navigation filter. Map data ©2022 GeoBasis-DE/BKG (©2009), Google

![](images/6f696c17dc8b3958ada150d719fbab11df4ccee16c6f9f8341abdf37523c1483.jpg)

## Sensitivity analysis of estimation error vs. $c _ { \rho }$ and $c _ { d }$

This section analyzes and compares the sensitivity of the state estimation accuracy from EHF and EKF against the parameter set $C _ { \rho }$ and $C _ { d } .$ . A grid search is performed within the parameter range $C _ { \rho } \in [ 1 0 , 4 0 0 0 ] \mathrm { m } / \sqrt { \mathrm { H z } }$ and $C _ { \mathrm { d } } \in [ 0 . 2 5 , 1 9 2 ] \mathrm { m } / \left( \mathrm { s } \cdot \sqrt { \mathrm { H z } } \right)$ . Considering the $\mathrm { C } / \mathrm { N } _ { 0 }$ values plotted in Fig.  4, which are within the range of $\mathrm { C } / \mathrm { N } _ { 0 } \in [ 2 0 , 5 0 ] \mathrm { d B } { \cdot } \mathrm { H z }$ , the range of the standard deviation of pseudorange and deltarange observables can be estimated by applying the sigma-ε model given in (22) and summarized in Table 4.

The grid search results are shown in Fig. 5. Comparing the subfigure top left and right, the minimum average 3D positioning error of both filters is approximately 0.62 m, indicating that the EHF has no advantage over the EKF under optimal parameter settings. However, the positioning accuracy of the EHF is less sensitive to the parameters (varies between 0.62 m to 1 m) than the EKF (varies between 0.62 m to 2 m). Comparing the subfigure bottom left and right, the EHF is slightly less sensitive to the parameters regarding the velocity-estimation accuracy. It should be noted that these results are against the statement in (Simon 2006, pp. 368): ‘the filter (EHF) performance is more sensitive to the design parameters. The reason is that, instead of the numerical example in (Simon 2006), the navigation filter has much larger system uncertainties, such as GNSS signal disturbances. Under poor parametrizations, the unmodeled measurement error dominates the filter’s performance. In this case, the EHF outperforms the EKF due to its filter robustness against system disturbances.

As mentioned in the experiment overview, a parameter set of $C _ { \rho }$ and $C _ { d }$ shall be chosen and used for the following evaluations. It is noticeable that the minimum values are diferent in each subfigure in Fig. 5, meaning that there are no optimal parameter settings for both filters. Considering that the evaluation in the next section aims at proving the robustness of the EHF under non-optimal conditions, we chose the parameter set, therefore, for the best benefit of the EKF, applying the following cost function:

$$
\text { weighted   3D   error } = \frac {\text { mean   3D   position   error }}{\max (\text { mean   3D   position   error })}
$$

$$
+ \frac {\text { mean   3D   velocity   error }}{\max (\text { mean   3D   velocity   error })}\tag{23}
$$

With (23), the 3D position and velocity error under each parameter setting are normalized by the maximum error of all parameter combinations and equally weighted. Applying (23), the parameter set is chosen as $C _ { \rho } = 1 0 0 \mathrm { m } / \sqrt { \mathrm { H z } }$ and $C _ { d } = 1 ~ \mathrm { m } / \left( \mathrm { s } \cdot \sqrt { \mathrm { H z } } \right)$ . The corresponding standard-deviation ranges estimated by the sigma-ε model are given in Table 4.

Table 3   Description of the three test rounds

<table><tr><td></td><td>Round I</td><td>Round II</td><td>Round III</td></tr><tr><td>Time</td><td>0–360 s</td><td>360–900 s</td><td>900–1514 s</td></tr><tr><td>Location</td><td>Circle inner left</td><td>Circle outer left</td><td>Circle right</td></tr><tr><td>Direction</td><td>Clockwise</td><td>Clockwise</td><td>Clockwise</td></tr><tr><td>Environment</td><td>Boulevard, semi-urban</td><td>Boulevard, semi-urban, highway</td><td>Boulevard, sub-urban, highway</td></tr><tr><td>Signal Disturbance</td><td>Low</td><td>High</td><td>High</td></tr></table>

Fig. 4 Receiver data during the test drive: the absolute velocity estimated by the GNSS receiver, the PVT mode of the receiver where only RTK fix and float can be considered as valid reference, the number of available satellites during the drive, and the carrier-to-noise ratio of the tracked GPS and Galileo signals  
![](images/e72dc05fc48feb2ff7d06ed4d0a65ccb4d4efa6826fd9575df8f03c5bfbed206.jpg)

Comparison between EHF and EKF regarding state estimation accuracy  
This section evaluates the state estimation accuracy of the EHF and EKF using the validation data from Round II and  
Table 4 Standard deviation of the pseudorange and deltarange estimated by the sigma-ε model

<table><tr><td rowspan="2" colspan="2">Estimated sigma</td><td colspan="3">Pseudorange  $C_{\rho}$  m/ $\sqrt{Hz}$ </td><td colspan="3">Deltarange  $C_{d}$  m/ $(s \cdot \sqrt{Hz})$ </td></tr><tr><td>10</td><td>100*</td><td>4000</td><td>0.25</td><td>1*</td><td>192</td></tr><tr><td>C/N $_{0}$ </td><td>50</td><td> $3.2 \times 10^{-2}$ </td><td>0.32</td><td>12.7</td><td> $7.9 \times 10^{-4}$ </td><td> $3.2 \times 10^{-3}$ </td><td>0.61</td></tr><tr><td>dB-Hz</td><td>20</td><td>1</td><td>10</td><td>400</td><td> $2.5 \times 10^{-2}$ </td><td>0.1</td><td>19.2</td></tr></table>

\*Refers to the parameter choice for the following evaluations by applying (23) as a cost function

![](images/0eda9d0c79b24d47ff5cf99c5e250a5643fc87ce30d78be7e1610614b173c067.jpg)  
Fig. 5 Average position and velocity-estimation accuracy of the EHF and EKF under various $C _ { \rho }$ and $C _ { d }$ applying the data from Round I. Each sample data is an individual simulation, and the contour plots  
are generated with the MATLAB® contourf.m function with default data interpolation. Note that both the x- and y-axes are logarithmic representations

III, applying the parameter set $C _ { \rho } = 1 0 0 \mathrm { m } / \sqrt { \mathrm { H z } }$ and $C _ { d } = 1 ~ \mathrm { m } / \left( \mathrm { s } \cdot \sqrt { \mathrm { H z } } \right)$ , which is optimized using data from Round I for the EKF. Figure  6 shows the experimental results, and Table 5 shows various metrics of positioning accuracy.

The first subfigure shows that the EHF has a slight advantage over the EKF regarding horizontal positioning accuracy, the same as in Table 5. However, under extreme disturbances, e.g., at around 500 and 1350 s when the vehicle drives under a bridge and then, directly into environments that lack satellite signals, both the EHF and EKF experience horizontal error up to meters.

Meanwhile, the vertical error comparison in the second subfigure and Table 5 indicates a more accurate estimation from the EHF than from the EKF. Under extreme disturbances, the EHF maintains a vertical error of under 2 m, while the EKF has a vertical error of up to 4 m. The results of the horizontal and vertical position errors show that EHF is more robust against environmental changes and signal disturbances by applying the same parameters.

Noteworthy, applying the EKF, the horizontal estimation error is smaller than the vertical one, which is consistent with expectations: During the drive, the average horizontal and vertical dilution of precision (DOP) is 0.862 and 1.707, respectively. This results from the satellite-constellation limitation that the satellites distribute only above the horizon (Kaplan and Hegarty 2005, pp. 328–332). With the given DOPs, the vertical positioning uncertainty is expected to be twice as large as the one in the horizontal direction if a least square estimator was applied. However, applying the EHF, the horizontal estimation error is surprisingly larger than the vertical one, which further proves the robustness and advantage of EHF against non-optimal operation conditions, i.e., non-optimal satellite constellation.

Fig. 6 Comparison of the estimation accuracy of the EHF and EKF: horizontal and vertical position error in meter; horizontal and vertical velocity error in meter per second  
![](images/111e156ac1ac4dab230f50eec705cf2c4acfb8e34e715ba53f10da060d5b357e.jpg)

Table 5 Accuracy indicators of horizontal and vertical positioning error of EHF and EKF

<table><tr><td rowspan="2">Positioning error [m]</td><td colspan="3">Horizontal</td><td colspan="3">Vertical</td></tr><tr><td>Mean</td><td>Std</td><td>RMS</td><td>Mean</td><td>Std</td><td>RMS</td></tr><tr><td>EHF</td><td>0.730</td><td>0.668</td><td>0.989</td><td>0.582</td><td>0.505</td><td>0.771</td></tr><tr><td>EKF</td><td>0.779</td><td>0.707</td><td>1.052</td><td>0.878</td><td>0.897</td><td>1.255</td></tr></table>

“Std.” stands for standard deviation, and “RMS” for root mean square

Finally, the third and fourth subfigures show no significant diference between EHF and EKF regarding the accuracy of velocity estimation, which is similar to the results in Fig. 5. Due to the lack of accurate orientation reference, the accuracy of the orientation estimation is not included.

More experiment results can be found in the supplementary material, where the EHF outweighed the EKF under various parametrizations of GNSS measurement noise, not only in vertical but also in horizontal positioning. Under extreme system disturbances, the EKF estimate diverges from the ground truth, while the EHF estimation error maintains limited, proving the EHF’s theoretical strength.

## Validation of PL generation using zonotope

This section first discusses the choice of the reduction order q and its influence on the zonotope estimation. It then validates the correctness of the PL estimation by applying the Stanford-ESA Integrity Diagram. Noteworthy, all PLs generated by zonotopes in this section are 3 𝜎-boundaries, corresponding to an interval integrity risk of 0.27% by initialization. It should be emphasized that the overall integrity risk shall be much lower, as discussed with (20).

Figure 7 shows the average width of the estimated PL by applying three- and nine-dimensional zonotopes under various reduction order q within the EHF. We conducted the post-processing using MATLAB and Simulink platform on a PC equipped with an Intel® Core (TM) i7-11700 K @ 3.60 GHz CPU. The results in Fig. 7 include the run time of the EHF together with the zonotope computed on a single CPU core.

![](images/0500473b99a0e7adbf80eaa7d1743879155ed61bc25afe613cb8ceea1fce2fa2.jpg)  
Fig. 7 Computation time and mean value of the generated HPL and VPL for the position estimation using 3D and 9D zonotopes with various reduction order q

As discussed in Fig. 9, the smaller q indicates a larger overestimation of the zonotope and a smaller computational load. This is verified by the results in Fig. 7, where the following observations should be stressed:

• The computational time of the zonotopes increases quasiproportionally to the reduction order q.

• The computational time of the zonotopes is not proportional to the dimension of the zonotopes. With the same reduction order $q = 2 0 0 0 0$ , the run time of the 9D zonotope is only 1.8 times that of the 3D zonotope.

• Applying high-dimensional zonotopes makes the overestimation effect more severe, especially with small reduction orders. By reduction order $q = 1 0 0 , 3 0 0 .$ , and 500, the average HPL (9D) is approximately $8 . 7 \cdot 1 0 ^ { 2 7 }$ $1 . 7 \cdot 1 0 ^ { 4 }$ , and 137 m, respectively. For better visualization, they are not shown in Fig. 7.

• With small reduction orders, the estimated HPL is larger than the VPL because zonotope’s overestimation efect dominates the PL generation. The calculation of HPL regards two states (position in north and east direction), which is afected by the overestimation more severely than VPL, which only regards one state (position in down direction). The overestimation efect becomes less dominant with the increasing reduction order, and the measurement update (given as (17)) contributes more to the PL generation.

Fig. 8 Stanford-ESA Integrity Diagram with 105,448 epochs. NO=nominal operation; MI = misleading information; HO = hazardous operation; SU = system unavailable. This plot is generated with (Ogier 2022)  
![](images/1f55f10ec022c451d65e078a72d3db4eba3fed78ec52847e556b42269519162a.jpg)

![](images/bc3978925e672bc368da0c305b88dd59096a57da91130f71774433011a45bda2.jpg)

![](images/b84852a716aacc941249793beeffcb4acee9e409a78f5efb6f073297db3b8bf8.jpg)

![](images/19d6b924d9935a496e02be15df21ce8755220da7c5c387d437fad2dc9d011537.jpg)

Table 6 Comparison of the zonotope- and filter-innovation-based PL generation

<table><tr><td rowspan="2"></td><td colspan="3">HPL</td></tr><tr><td>Median [m]</td><td> $P_{mi}$  [%]</td><td>Availability [%]</td></tr><tr><td>Zonotope-based (q=20000)</td><td>10.71</td><td>0</td><td>100</td></tr><tr><td>Innovation-based (Zhu et al. 2018)</td><td>10.79</td><td>0.97</td><td>100</td></tr></table>

$P _ { m i }$ refers to the probability of misleading information

• Before $q = 2 0 0 0 0$ , increasing the reduction order reduces the overestimation efect of zonotopes significantly. After $q = 2 0 0 0 0$ , the benefit of further increasing the reduction order is not obvious. Moreover, it increases the risk of violating the 10 ms real-time threshold.

Summarizing the results, we can conclude that applying high-dimensional zonotopes requires larger reduction orders, which is more computationally consuming. Therefore, the choice of the zonotope dimension should be prudent and consider the application requirements to avoid wasting computing power.

The performance of the PL estimation is commonly evaluated by applying the Stanford-ESA Integrity Diagram, presented in Fig. 8. The core issue of the PL calculation is to bound the state estimation error, requiring the estimated PL of a state to be larger than the state error. Namely, the sample points shall locate in the top left area in Fig. 8. Further, the estimated PL shall be under a pre-defined limit, namely the alarm limit (AL). Otherwise, the integrity system shall be declared unavailable. Finally, the red area shall be avoided as the highest priority for integrity systems, namely the hazardous operation (also known as hazardously misleading information (HMI)), where the actual state estimation error is beyond the AL, and the estimated PL is under the AL. To conclude, the sample points shall locate in the nominal operation (NO) area, while the estimated PLs should be kept as small as possible.

The results in Fig. 8 are the position PLs generated by applying the three-dimensional zonotope. We cannot evaluate the PLs of velocity qualitatively due to insuficient accuracy and precision of the reference, which we demonstrate in the supplementary material. Further, we cannot evaluate the PLs of the orientation due to the missing reference data. Thus, we include the results of the velocity and orientation

PLs generated by the nine-dimensional zonotopes in the supplementary material.

Figure 8 shows that, regardless of the choice of the reduction order q, no misleading information is generated by the zonotope, verifying the correctness of the zonotope-based PL estimation. With this result, no integrity risk is observed with the proposed approach. Further, comparing the generated PL by applying $q = 1 0 0 0$ and $q = 2 0 0 0 0$ , the overestimation efect with reduction order $q = 1 0 0 0$ is notable: The sample points are distributed almost evenly along the y-axes, regardless of the positioning error. This also results in system unavailability of up to 39.084%. By applying q = 20000, all samples locate in the nominal operation area for HPL estimation.

Table 6 introduces the essential metrics to evaluate the zonotope-based HPL calculation and compares them with the results of the innovation-based method (Zhu et al. 2018), where an EKF applying the same sigma-ε model is developed. The results indicate that the zonotope-based approach outperforms the innovation-based approach in terms of misleading information without compromising on the size of the generated HPL and algorithm availability.

Still, the integrity requirement for safety–critical applications in (EUSPA 2021) is not met with the proposed system, namely HPL under 2 m. Such a requirement cannot be satisfied by simply increasing the reduction order. Instead, further reducing HPL relies on limiting the measurement uncertainty and, therefore, might be satisfied by integrating further GNSS observations, i.e., carrier phase, into the tightly coupled navigation filter and integrity system.

## Conclusion

This manuscript presented a novel robust state and protection-level estimation scheme within a tightly coupled navigation system involving the EHF and zonotope. The EHF guarantees a robust state estimation under poor parametrization or system disturbances, which was validated with real-world data from mixing environments (including semi/sub-urban environments) in post-processing. By grid searching the same parameter space of the GNSS measurement noise, the 3D positioning accuracy of the EHF varies between 0.6 m and 1 m, whereas the one from the EKF varies between 0.6 m and 2 m. This result proved that the estimation accuracy of the EHF is less sensitive to the GNSS measurement parametrization and, therefore, has less requirement on the measurement noise modeling. Applying the same GNSS measurement parametrization, the EHF shows higher robustness against real-world disturbances and outperforms the EKF regarding positioning accuracy, especially in the vertical direction. Considering the dificulty of the appropriate parametrization by applying the EKF within a GNSS/INS, the EHF is suggested to be an ideal alternative to the EKF in urban environments, where the quality of GNSS measurements varies continuously due to changing operating environments. The EHF could benefit UAV applications where vertical localization is essential.

Further, a feasibility study was carried out for the zonotopebased PL generation in a high-order nonlinear, tightly coupled GNSS/INS. In theory, applying zonotope releases the PL generation from the statistical assumptions. The experimental results prove that applying a zonotope for PL generation is valid when the reduction order is appropriately set. Compared to the conventional filter-innovation-based approach, applying zonotope decreases the probability of misleading information to zero and slightly reduces the size of generated PL. This approach is computationally intensive when a large reduction order is required. However, the results prove that a real-time implementation is promising. The size of estimated PLs should be further reduced by applying carrier phase measurements to apply zonotope for safety–critical autonomous applications.

## Appendix

## $H _ { \infty }$ filtering and zonotope theory

For the brevity of the main text, we present the basic theory of the EKF and zonotope in this appendix.

## Theory of extended $\pmb { H } _ { \infty }$ filter

Considering a nonlinear time-discrete system given as follows:

$$
\boldsymbol {x} _ {k} = \boldsymbol {f} \left(\boldsymbol {x} _ {k - 1}\right) + \boldsymbol {G} _ {k - 1} \boldsymbol {w} _ {k - 1}, \boldsymbol {w} _ {k} \sim \mathcal {N} (0, \boldsymbol {Q} _ {k})\tag{A1}
$$

$$
\boldsymbol {z} _ {k} = \boldsymbol {h} (\boldsymbol {x} _ {k}) + \nu_ {k}, \quad \nu_ {k} \sim \mathcal {N} (0, \boldsymbol {R} _ {k})\tag{A2}
$$

$$
\mathbf {y} _ {k} = \boldsymbol {L} _ {k} \mathbf {x} _ {k}\tag{A3}
$$

where $\pmb { x } _ { \mathrm { k } }$ and $x _ { \mathrm { k - 1 } }$ are the state vectors for time instance k and $k - 1$ , respectively. $z _ { \mathrm { k } }$ is the measurement vector. $f$ and h are the nonlinear time-invariant state transition and measurement functions, respectively. ${ \pmb w } _ { k }$ and $\nu _ { k }$ are the process and measurement noise vectors, respectively. $\mathbf { G } _ { k }$ denotes the shaping matrix, which maps the process noise ${ \pmb w } _ { k }$ into the state vector $\boldsymbol { x } _ { k } . \boldsymbol { w } _ { k }$ and $\nu _ { k }$ are assumed to satisfy a white Gaussian distribution, whose variance matrices are $Q _ { k }$ and $\pmb { R } _ { k } ,$ , respectively. $\mathbf { \vec { y } } _ { \mathrm { k } }$ denotes the desired estimation vector at the time instance k, which is defined by a user-designed matrix $\scriptstyle L _ { k }$ (full rank). Concerning the navigation solution, all states in $\mathbf { \boldsymbol { x } } _ { k }$ shall be estimated such that $\scriptstyle L _ { k }$ is defined as a unit matrix. The complete $H _ { \infty }$ filtering computation steps are rigorously derived in (Simon 2006) and summarized in (Loo et al. 2019) as follows:

$$
\hat {\pmb {x}} _ {k} ^ {-} = \pmb {f} \left(\hat {\pmb {x}} _ {k - 1} ^ {+}\right)\tag{A4}
$$

$$
\hat {\pmb {P}} _ {k} ^ {-} = \pmb {F} _ {k - 1} \hat {\pmb {P}} _ {k - 1} ^ {+} \pmb {F} _ {k - 1} ^ {T} + \pmb {G} _ {k - 1} \pmb {Q} _ {k - 1} \pmb {G} _ {k - 1} ^ {T}\tag{A5}
$$

$$
\hat {\boldsymbol {P}} _ {k} ^ {+} = \hat {\boldsymbol {P}} _ {k} ^ {-} \left[ \boldsymbol {I} - \gamma \boldsymbol {L} _ {k} ^ {T} \boldsymbol {S} _ {k} \boldsymbol {L} _ {k} \hat {\boldsymbol {P}} _ {k} ^ {-} + \boldsymbol {H} _ {k} ^ {T} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} \hat {\boldsymbol {P}} _ {k} ^ {-} \right]\tag{A6}
$$

$$
\boldsymbol {K} _ {k} = \hat {\boldsymbol {P}} _ {k} ^ {+} \boldsymbol {H} _ {k} ^ {T} \boldsymbol {R} _ {k} ^ {- 1}\tag{A7}
$$

$$
\hat {\boldsymbol {x}} _ {k} ^ {+} = \hat {\boldsymbol {x}} _ {k} ^ {-} + \boldsymbol {K} _ {k} \left(\boldsymbol {z} _ {k} - \boldsymbol {h} \left(\hat {\boldsymbol {x}} _ {k} ^ {-}\right)\right)\tag{A8}
$$

where $\hat { x } _ { k } ^ { - } , \hat { x } _ { k - 1 } ^ { + } , \hat { P } _ { k } ^ { - }$ , and $\hat { { P } } _ { k } ^ { + }$ denote the a-priori and a-posteriori state and covariance estimate, respectively. $\boldsymbol { F } _ { k - 1 }$ and $\pmb { H } _ { k }$ are the Jacobian matrix of $f$ and h, linearized at the operating points, respectively. $S _ { k }$ is a user-defined weighting matrix, specified as a unit matrix in current work, which weights the elements of ${ \bf y } _ { k }$ in the cost function given in (A9). $\pmb { K } _ { k }$ is the filter gain matrix. It should be noted that the calculation of $\pmb { K } _ { k }$ involves a parameter $\gamma$ , discussed later in this section.

The purpose of the optimal $H _ { \infty }$ filtering is to choose the suitable estimation strategy that minimizes the following cost function:

$$
J = \frac {\sum_ {k = 0} ^ {N - 1} \left\| \mathbf {y} _ {k} - \hat {\mathbf {y}} _ {k} \right\| _ {S _ {k}} ^ {2}}{\left\| \mathbf {x} _ {0} - \hat {\mathbf {x}} _ {0} \right\| _ {P _ {0} ^ {- 1}} ^ {2} + \sum_ {k = 0} ^ {N - 1} \left(\left\| \mathbf {w} _ {k} \right\| _ {Q _ {k} ^ {- 1}} ^ {2} + \left\| \mathbf {v} _ {k} \right\| _ {R _ {k} ^ {- 1}} ^ {2}\right)}\tag{A9}
$$

where ${ \boldsymbol { x } } _ { 0 } , { \boldsymbol { \hat { x } } } _ { 0 } ,$ and $P _ { 0 }$ represent the initial state vector, its estimation, and the covariance of the initialization error, respectively. $\hat { \bf y } _ { k }$ is the estimate of $\mathbf { \it y } _ { k } . {  { N } }$ denotes the current epoch. The cost function J can be understood as the normalization of all the weighted historical estimation errors of ${ \bf y } _ { k }$ from epoch 0 to the last epoch $N - 1$ , relative to the initialization error $\pmb { x } _ { 0 } - \hat { \pmb { x } } _ { 0 } .$ , the process noise ${ \pmb w } _ { k }$ and measurement noise $\nu _ { k }$ and considering their corresponding covariance $P _ { 0 } , Q _ { k }$ and $\pmb { R } _ { k } .$ . The choice of the cost function J is based on the game theory approach and is assumed to improve the filter robustness against worst-case situations (Simon 2006, p. 343).

However, the cost function cannot be directly minimized (Simon 2006, p. 344). Thus, a performance bound $\gamma ^ { - 1 }$ is introduced as a constraint for the optimization:

$$
J <   \gamma^ {- 1}\tag{A10}
$$

Solving the constrained optimization problem results in the estimation strategy (A6)–(A8). Meanwhile, the constrained optimization problem is only solvable when the following inequality condition is satisfied (Simon 2006, pp. 351–352):

![](images/801302d3c8370bf1e8ce57fd8800355c63f4bddc6537494dc192e6e40ddc8421.jpg)  
Fig. 9 An example of a two-dimensional six-order zonotope ℤ (yellow). $\mathbb { Z } _ { 1 }$ (blue) and $\mathbb { Z } _ { 2 }$ (grey) are reduced with $q _ { 1 } = 4$ and $q _ { 2 } = 3 ,$ respectively. The outer line is the boundary of the interval hull box [𝜶]

$$
\left(\hat {\boldsymbol {P}} _ {k} ^ {-}\right) ^ {- 1} + \boldsymbol {H} _ {k} ^ {T} \boldsymbol {R} _ {k} ^ {- 1} \boldsymbol {H} _ {k} - \gamma \boldsymbol {L} _ {k} ^ {T} \boldsymbol {S} _ {k} \boldsymbol {L} _ {k} > 0\tag{A11}
$$

where $\mathit { \Psi } \cdot \sum \mathit { \Psi } _ { \mathrm { = } } \cdot$ denotes that the resulting matrix on the lefthand side is positive-definite. It can be observed that the larger is $\gamma ,$ the more dificult that (A11) can be satisfied. Thus, $\gamma$ shall not be chosen too large. The choice 𝛾 in the vicinity of infinity might cause the divergence of the EHF (Hassibi et al. 1996). On the other hand, Simon (2006) has described $\gamma$ as a measure of filter robustness, i.e., the larger 𝛾, the smaller estimation error under worst-case situations, and therefore, the stronger robustness of the filter. Thus, the choice of $\gamma$ is crucial and decides the EHF performance. In practice, 𝛾 could be chosen conservatively by experience or estimated by solving the linear matrix inequality (LMI) problem described by (A11). We apply an LMI solver from MATLAB® to calculate 𝛾.

## Theory of zonotope

Zonotope is a type of centrally symmetric convex polytope. Zhang et al. (2020) have applied zonotope to present the bounding area of the estimated states within a filter. This section introduces the definition and necessary background knowledge of zonotope.

Definition 1 An n-dimensional zonotope with m-order $\mathbb { Z } \subset \mathbb { R } ^ { n } ( n \leq m )$ is an afine transformation of a hypercube $\mathbb { B } ^ { m } = [ - 1 , 1 ] ^ { m }$ (Zhang et al. 2020):

$$
\mathbb {Z} = \{\boldsymbol {\alpha} \in \mathbb {R} ^ {n} | \boldsymbol {\alpha} = \boldsymbol {c} + \mathcal {H} \boldsymbol {b}, \forall \boldsymbol {b} \in \mathbb {B} ^ {m} \}\tag{A12}
$$

where vector $c \in \mathbb { R } ^ { n }$ is the center of $\mathbb { Z } . \mathcal { H } \in \mathbb { R } ^ { n \times m }$ is called the generation matrix of zonotope $\mathbb { Z } ,$ which defines the shape and direction of ℤ. Each column of $\mathcal { H } = [ h _ { 1 } , . . . , h _ { m }$ ] denotes a translation direction and amplitude in Euclidean space. For a simple expression, we use $\mathbb { Z } = < \mathfrak { c } , \mathcal { H } >$ to denote a zonotope. Fig. 9 presents an example of a two-dimensional and six-order zonotope as the yellow area.

Property 1.1 The Minkowski sum of two zonotopes $\mathbb { Z } _ { 1 } = < \mathfrak { c } _ { 1 } , \mathcal { H } _ { 1 } > \subset \mathbb { R } ^ { n }$ and $\mathbb { Z } _ { 2 } = < \mathfrak { c } _ { 2 } , \mathcal { H } _ { 2 } > \subset \mathbb { R } ^ { n }$ is also a zonotope, which can be computed as follows:

$$
<   \boldsymbol {c} _ {1}, \mathcal {H} _ {1} > \oplus <   \boldsymbol {c} _ {2}, \mathcal {H} _ {2} > = <   \boldsymbol {c} _ {1} + \boldsymbol {c} _ {2}, [ \mathcal {H} _ {1} \mathcal {H} _ {2} ] >\tag{A13}
$$

Property 1.2 The image of a zonotope by a linear transformation L can be computed by:

$$
\mathcal {L} \odot <   c, \mathcal {H} > = <   \mathcal {L} c, \mathcal {L H} >\tag{A14}
$$

where ⊙ denotes the linear image operator.

Definition 2 The interval hull box [𝜶] of a zonotope ℤ is the smallest centered interval vector containing ℤ (Zhang et al. 2020). An example of a two-dimensional zonotope with six-order and its interval hull is depicted in Fig. A1. For an m-order zonotope $\mathbb { Z } = < \mathfrak { c } , \mathcal { H } > \subset \mathbb { R } ^ { n }$ , its interval hull box [𝜶] is computed as follows:

$$
\left[ \alpha_ {i} \right] = \left[ c _ {i} - \sum_ {j = 1} ^ {m} \left| \mathcal {H} _ {i j} \right|, c _ {i} + \sum_ {j = 1} ^ {m} \left| \mathcal {H} _ {i j} \right| \right], i \in \{1, \ldots , n \}\tag{A15}
$$

Remark 1 When the outer approximation of a zonotope $\mathbb { Z }$ by an aligned box is too conservative, a reduction operator $\mathcal { R } ( \mathcal { H } )$ can be used to reduce the complexity of $\mathbb { Z }$ (Combastel 2016). After the reduction, the original zonotope is a subset of the resulting zonotope:

$$
<   \boldsymbol {c}, \mathcal {H} > \subseteq <   \boldsymbol {c}, \mathcal {R} (\mathcal {H}) >\tag{A16}
$$

without changing the interval hull box. The reduction order $q \left( n \leq q \leq m \right)$ specifies the order of the resulting zonotope $< c , \mathcal { R } ( \mathcal { H } ) >$ , where $\mathcal { R } ( \mathcal { H } ) \in \mathbb { R } ^ { n \times q }$ . The realization of zonotope reduction is given in (Zhang et al. 2020).

In Fig. 9, an example of reducing the two-dimensional zonotope $( \mathbb { Z } = < c , \pmb { \mathcal { H } } > \subset \mathbb { R } ^ { 2 } )$ with six-order $( \mathcal { H } \in \mathbb { R } ^ { 2 \times 6 } )$ is illustrated. ℤ is reduced $\mathbf { t o } ~ \mathbb { Z } _ { 1 }$ and $\mathbb { Z } _ { 2 }$ with reduction order $q _ { 1 } = 4 \mathrm { a n d } q _ { 2 } = 3 ,$ respectively. A smaller reduction order q results in a larger overestimation but reduces the computational load. The reduction order $q$ is allowed to be freely chosen (Combastel 2003), compromising between the exactness (with larger q) and the domain complexity (with smaller q) of the resulting zonotope.

Supplementary Information The online version contains supplementary material available at https://doi.org/10.1007/s10291-023-01447-z.

Acknowledgements The authors gratefully acknowledge the contribution of the joint research project Firefly (Grant 50NA2103), which is supported by the German Federal Ministry for Economic Afairs and Climate Action (BMWK) in compliance with a resolution of the German Bundestag. Special thanks go to colleagues Jiaying Lin and David Benz for proofreading the article.

Funding Open Access funding enabled and organized by Projekt DEAL.

Data availability The datasets used and generated in this research are available from the corresponding author upon reasonable request.

Open Access This article is licensed under a Creative Commons Attri bution 4.0 International License, which permits use, sharing, adaptation, distribution and reproduction in any medium or format, as long as you give appropriate credit to the original author(s) and the source, provide a link to the Creative Commons licence, and indicate if changes were made. The images or other third party material in this article are included in the article's Creative Commons licence, unless indicated otherwise in a credit line to the material. If material is not included in the article's Creative Commons licence and your intended use is not permitted by statutory regulation or exceeds the permitted use, you wil need to obtain permission directly from the copyright holder. To view a copy of this licence, visit http://creativecommons.org/licenses/by/4.0/.

## References

Bhamidipati S, Kousik S, Gao G (2022) Set-valued shadow matching using zonotopes for 3-D map-aided GNSS localization. https:/ doi.org/10.48550/arXiv.2209.14238

Bhatti UI, Ochieng WY (2007) Failure modes and models for integrated GPS/INS systems. J Navig 60(2):327–348. https://doi.org 10.1017/S0373463307004237

Breuer M, Konrad T, Abel D (2016) High precision localisation in customised GNSS receiver for railway applications. In: Proceedings of the 29th international technical meeting of the satellite division of the institute of navigation (ION GNSS+ 2016). pp 779–787. https://doi.org/10.33012/2016.14696

Combastel C (2016) An extended zonotopic and Gaussian Kalman filter (EZGKF) merging set-membership and stochastic paradigms: toward non-linear filtering and Fault detection. Annu Rev Control 42:232–243. https://doi.org/10.1016/j.arcontrol.2016.07.002

Combastel C (2003) A state bounding observer based on zonotopes. In: 2003 European control conference (ECC). pp 2589–2594. https:/ doi.org/10.23919/ECC.2003.7085991

Dbouk H, Schön S (2018) Comparison of diferent bounding methods for providing GPS integrity information. In: 2018 IEEE/ION position, location and navigation symposium (PLANS). pp 355–366 https://doi.org/10.1109/PLANS.2018.8373401

Dbouk H, Schön S (2020) Reliable bounding zones and inconsistency measures for GPS positioning using geometrical constraints. Acta Cybern 24(3):573–591. https://doi.org/10.14232/actacyb.24.3. 2020.16

EUSPA (2021) Report on road user needs and requirements - outcome of the EUSPA user consultation platform. European Union Agency for the Space Program (EUSPA)

Gehrt J-J, Liu W, Stenger D, Liu S, Abel D (2020) Environmentall dependent adaptive parameterization of a GNSS-aided tightlycoupled navigation filter. In: 2020 European navigation conference (ENC). pp 1–10. https://doi.org/10.23919/ENC48637.2020.9317339

Groves PD (2013) Principles of GNSS, inertial, and multisensor integrated navigation systems, 2nd edn. Artech House, Boston

Hassibi B, Sayed AH, Kailath T (1996) Linear estimation in Krein spaces. II: applications. IEEE Trans Autom Control 41(1):34–49. https://doi. org/10.1109/9.481606

Jwo D-J, Chen R-J (2003) Mixed H2/H∞ filtering approach for GPS/ INS navigation systems. In: Proceedings of international conference on recent advances in space technologies, 2003. RAST ’03. pp 409–414. https://doi.org/10.1109/RAST.2003.1303951

Kaplan E, Hegarty C (2005) Understanding GPS principles and applications: principles and applications, 2nd edn. Artech house, Norwood

Konrad T, Gehrt J-J, Lin J, Zweigel R, Abel D (2018) Advanced state estimation for navigation of automated vehicles. Annu Rev Control 46:181–195. https://doi.org/10.1016/j.arcontrol.2018.09.002

Lee J, Kim M, Pullen S (2018) Integrity assurance of Kalman-filter based GNSS/IMU integrated systems against IMU faults for UAV applications. In: Proceedings of the 31st international technical meeting of the satellite division of the institute of navigation (ION GNSS+ 2018). pp 2484–2500. https://doi.org/10.33012/2018.15977

Liu S, Gehrt J-J, Abel D, Zweigel R (2019) Dual-constellation aided high integrity and high accuracy navigation filter for maritime applications. In: Proc. of ION ITM 2019. pp 762–774. https://doi.org/10. 33012/2019.16723

Liu S, Gehrt J-J, Abel D, Zweigel R (2021) Identification of multi-faults in GNSS signals using RSIVIA under dual constellation. Acta Cybern 25(1):69–84. https://doi.org/10.14232/actacyb.285315

Loo JY, Tan CP, Nurzaman SG (2019) H-Infinity based extended Kalman Filter for state estimation in highly non-linear soft robotic system. In: 2019 American control conference (ACC). pp 5154–5160. https:// doi.org/10.23919/ACC.2019.8814869

Nitsch M, Gehrt J-J, Heyn L, Zweigel R, Abel D (2021) Embedded tightly coupled INS/DGPS-DGAL navigation filter on a mass-market single-board computer. In: 2021 International conference on localization and GNSS (ICL-GNSS). IEEE, Tampere, Finland, pp 1–7. https://doi.org/10.1109/ICL-GNSS51451.2021.9452196

Ogier E (2022) Stanford diagram. MATLAB Central File Exchange. https://www.mathworks.com/matlabcentral/fileexchange/86383- stanford-diagram

Osman M, Alonso R, Hammam A, Moreno FM, Al-Kaf A, Hussein A (2019) Multisensor fusion localization using extended H∞ filter using Pre-filtered sensors measurements. In: 2019 IEEE intelligent vehicles symposium (IV). pp 1139–1144. https://doi.org/10.1109/ JVS.2019. 8814234

Shetty A, Gao GX (2021) Predicting state uncertainty bounds using nonlinear stochastic reachability analysis for urban GNSS-based UAS navigation. IEEE Trans Intell Transp Syst 22(9):5952–5961. https:// doi.org/10.1109/TITS.2020.3040517

Simon D (2006) Optimal state estimation: Kalman, H infinity, and nonlinear approaches. Wiley, New York

Smolyakov I, Langley RB (2020) Intelligent navigation in urban environments based on an H-infinity filter and reinforcement learning algorithms. In: 2020 IEEE/ION Position, Location and Navigation Symposium (PLANS). pp 328–333. https://doi.org/10.1109/PLANS 46316.2020.9109948

Wendel J (2009) Integrierte navigationssysteme: sensordatenfusion, GPS und inertiale navigation. Oldenbourg Wissenschaftsverlag, München

Wieser A, Gaggl M, Hartinger H (2005) Improved positioning accuracy with high-sensitivity GNSS receivers and SNR aided integrity monitoring of pseudo-range observations. In: Proceedings of the 18th international technical meeting of the satellite division of the institute of navigation (ION GNSS 2005). pp 1545–1554

Zhang W, Wang Z, Raïssi T, Wang Y, Shen Y (2020) A state augmentation approach to interval fault estimation for descriptor systems. Eur J Control 51:19–29. https://doi.org/10.1016/j.ejcon.2019.06.006

Zhu N, Betaille D, Marais J, Berbineau M (2018) Extended Kalman filter (EKF) innovation-based integrity monitoring scheme with C/N0 weighting. In: 2018 IEEE 4th international forum on research and

technology for society and industry (RTSI). IEEE, Palermo, pp 1–6. https://doi.org/10.1109/RTSI.2018.8548512

Publisher's Note Springer Nature remains neutral with regard to jurisdictional claims in published maps and institutional afiliations.

![](images/1081da4043ee28b95f79dc812b7f235ce98f7486223b6072b0dd0efc47a03993.jpg)

![](images/a2429a7257d0bddda3ef44ae130c7276bd7c5735778444bfe178ebf9d1c46a4a.jpg)

Shuchen Liu received his B. Sc. degree from Shanghai Jiao Tong University in 2014 and his M. Sc. degree from RWTH Aachen University in 2017, both in automation engineering. Since 2017, he has been a research associate and pursuing his Ph.D. degree in the Navigation research group at the Institute of Automatic Control. His research focuses on robust state estimation and integrity monitoring of GNSS-based navigation systems for diferent applications, such as automobiles and vessels.

Kaizheng Wang received his B.Eng. degree in mechatronics engineering from Zhejiang University in 2017 and M. Sc. degree in automation engineering from RWTH Aachen University in 2020. Currently, he is a Ph.D. candidate in the Department of Computer Science, M-Group, Campus Bruges, KU Leuven. His research activities focus on uncertainty modeling and optimization and epistemic artificial intelligence.

![](images/15eac8982fff50017d313f3c8c7a9adb1214f46be6321d33eb9675b507742956.jpg)  
Dirk Abel received his Dipl.-Ing degree in mechanical engineering in 1983, and the Dr.-Ing. Degree in 1987 from RWTH Aachen University. There, in 2001, he became a full professor and head of the Institute of Automatic Control. From 2009 to 2017, he was a member of the supervisory board of KUKA AG, Germany. Professor Abel’s main research area covers model predictive control, nonlinear and robust control, and rapid control prototyping in diferent application areas.