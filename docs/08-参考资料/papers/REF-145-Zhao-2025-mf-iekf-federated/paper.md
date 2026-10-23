Article

# MF-IEKF: A Multiplicative Federated Invariant Extended Kalman Filter for INS/GNSS

Lebin Zhao <sup>1</sup>, Tao Chen <sup>1,</sup>\*, Peipei Yuan <sup>1</sup>, Xiaoyang Li <sup>1</sup> and Yang Luo <sup>1,2</sup>

<sup>1</sup> School of Automobile, Chang’an University, Xi’an 710018, China

<sup>2</sup> Sichuan Chengneiyu Expressway Co., Ltd., Neijiang 641100, China

\* Correspondence: chentao@chd.edu.cn

## Highlights

What are the main findings?

Based on the two left-invariant errors on the SE2(3) Lie group, a corrected error on the Lie algebra is proposed for the multiplicative corrected left-invariant extended Kalman filter (MCL-IEKF).

Leveraging the decoupling of position and atitude in the right-invariant observation matrix of GNSS, a multiplicative federated IEKF (MF-IEKF) is developed by combining left- and right-invariant errors.

What are the implications of the main findings?

MCL-IEKF improves state estimation accuracy compared to classical multiplicative left- (right-)IEKF.

Under large misalignment angle conditions, MF-IEKF achieves faster convergence in position and velocity compared to the MCL-IEKF.

## Abstract

Academic Editor: Baocheng Zhang

Received: 24 November 2025 Revised: 21 December 2025 Accepted: 22 December 2025 Published: 24 December 2025

Copyright: © 2025 by the authors. Licensee MDPI, Basel, Switzerland. This article is an open access article distributed under the terms and conditions of the Creative Commons Atribution (CC BY) license.

The integration of an inertial navigation system (INS) with the Global Navigation Satellite System (GNSS) is crucial for suppressing the error drift of the INS. However, traditional fusion methods based on the extended Kalman filter (EKF) sufer from geometric inconsistency, leading to biased estimates—a problem markedly exacerbated under large initial misalignment angles. The invariant extended Kalman filter (IEKF) embeds the state in the Lie group SE2(3) to establish a more consistent framework, yet two limitations remain. First, its standard update fails to synergize complementary error information within the left-invariant formulation, capping estimation accuracy. Second, velocity and position states converge slowly under extreme misalignment. To address these issues, a multiplicative federated IEKF (MF-IEKF) was proposed. A geometrically consistent state propagation model on SE2(3) is derived from multiplicative IMU pre-integration. Two parallel, mutually inverse left-invariant error sub-filters (ML1-IEKF and ML2-IEKF) cooperate to improve overall accuracy. For large-misalignment scenarios, a short-term multiplicative right-invariant sub-filter is introduced to suppress initial position and velocity errors. Extensive Monte Carlo simulations and KITTI dataset experiments show that MF-IEKF achieves higher navigation accuracy and robustness than ML1-IEKF.

Keywords: multiplicative federated invariant EKF (MF-IEKF); inertial navigation system (INS); global navigation satellite system (GNSS); state estimation

## 1. Introduction

Inertial navigation systems (INSs) are indispensable in aerospace, autonomous driving, and robotics, owing to their full autonomy, high-rate output, and provision of complete motion information [1–3]. However, propagating atitude, velocity, and position through temporal integration of gyroscope and accelerometer data leads to the inherent accumulation and divergence of navigation errors, resulting in significant positional drift [4–6]. To suppress this drift and enhance accuracy, multi-sensor fusion, especially with the Global Navigation Satellite System (GNSS), has become indispensable.

Among existing fusion algorithms, frameworks based on the Extended Kalman Filter (EKF) [7,8] and its Error-State Kalman Filter (ESKF) variant [9,10] remain the most widely adopted. Nevertheless, these conventional methods sufer from fundamental limitations. Conventional methods model position and velocity in Euclidean space while representing atitude on the Lie group SO(3) or with quaternions. This mismatch in parameterization disrupts the intrinsic geometric structure. When state estimates deviate significantly, the local linearization employed by EKF/ESKF introduces substantial errors, jeopardizing filter convergence [11]. More critically, these methods exhibit geometric inconsistency [12]: the linearization process during measurement updates can induce spurious information gain in unobservable directions [13,14], thereby corrupting the state estimate. This phenomenon is well-established in Simultaneous Localization and Mapping (SLAM) [15–17] and visual-inertial odometry (VIO) systems [18,19].

To address this fundamental issue of geometric inconsistency, the invariant filtering framework, rooted in Riemannian manifold and Lie group theory, has emerged [20–24]. A key realization of this framework is the invariant extended Kalman filter (IEKF), which constructs a geometrically consistent filtering structure by embedding the entire system state into the matrix Lie group SE2(3). In this seting, the motion model satisfies the groupafine property [25], which results in error dynamics governed by a linear autonomous diferential equation, independent of the true state trajectory [22]. This structural decoupling enables the IEKF to theoretically overcome the convergence pitfalls of traditional EKF methods that stem from linearization-point dependence. Consequently, the IEKF maintains superior estimation accuracy and long-term stability, even under large initial misalignment [26,27].

The IEKF can be categorized into the left-IEKF (L-IEKF) and the right-IEKF (R-IEKF) according to the error definition. The system matrix of L-IEKF is completely independent of the state trajectory, often leading to higher estimation accuracy, whereas that of R-IEKF excludes rapidly varying states, yielding superior temporal stability [20]. Consequently, the observation model must be aligned with either the left- or right-invariant form depending on its physical nature (global or local) [28]. This alignment is crucial for maintaining filter consistency throughout the process and for achieving optimal estimation [28–31]. Guided by this principle, IEKF has been successfully applied in globally referenced inertial-integrated navigation [30,32] and locally referenced VIO systems [21,33–35].

Recognizing the complementary advantages of left- and right-invariant models, researchers have begun to explore their joint utilization. For instance, Ref. [31] proposed a federated IEKF that processes left- and right-invariant observation models in parallel and dynamically selects the output according to the scenario, representing a preliminary exploration of multi-model collaboration. However, the intrinsic synergistic mechanisms between these error models were not thoroughly analyzed. In contrast, Ref. [28] established a connection between left- and right-invariant errors for atitude estimation using the adjoint matrix of the Lie group, improving accuracy. Yet, this collaborative framework was not extended to full state estimation encompassing velocity and position. These works indicate that exploring a deep fusion of left- and right-invariant models is a clear and valuable direction.

Despite initial atempts at fusion architectures, existing Lie group-based navigation methods still face two fundamental challenges in delivering high-precision, high-robustness performance. The first is the challenge of model fidelity and generalizability. Many studies rely on simplified models, such as assuming a flat Earth and neglecting Coriolis efects [36], which deviate from real-world navigation conditions. To address this, Ref. [36] proposed an equivariant filtering framework that accounts for Coriolis forces and introduced two new left- and right-invariant error models. Recent research strives to enhance geometric consistency under more realistic models, e.g., constructing precise kinematics and error models on the SE(3) group in the Earth-centered inertial (ECI) or Earth-centered, Earth-fixed (ECEF) reference frames [37,38]. These works deepen the theoretical foundation and improve atitude—particularly heading—estimation under large misalignment angles. However, their focus remains on refining the model within a single reference frame and on theoretical analysis. The second challenge is computational eficiency and structural consistency. Although many methods define states on Lie groups, their kinematic equations often adopt decoupled computational propagation [21,28–35,39], ofering computational eficiency comparable to that of the traditional EKF without fully leveraging the structural advantages of Lie group methods. To address this, Ref. [40] introduced an auxiliary velocity and unified state propagation on the SE2(3) group, proposing an efficient multiplicative pre-integration theory that was later generalized into a unified mathematical framework by [41]. Ref. [42] applied this approach to INS/GNSS integration but did not efectively mitigate the dramatic growth of velocity and position errors caused by large misalignment angles during initial alignment. Recently, Ref. [43] proposed an Iterative Equivariant Filter (I-EqF) that enhances the robustness of tightly coupled SINS/GNSS navigation under nonlinearities and large misalignment through iterative updates and covariance reseting, representing progress in optimizing a single-filter algorithm.

Building on the aforementioned developments, current research has advanced invariant filtering technology along several fronts: model construction has become more rigorous [36–38]; numerical methods for single filters have been refined [43]; and valuable attempts have been made to fuse left- and right-invariant models [28,31]. Yet a unified framework that organically integrates two key considerations remains elusive: at the architectural level, how to transcend simple parallel or switching mechanisms to achieve deep synergy between left- and right-invariant error estimations;, in the presence of large initial misalignment, how to exploit the right-invariant model’s insensitivity to atitude error to accelerate the convergence of velocity and position states. Addressing these challenges motivates the present work.

To synergistically leverage the complementary strengths of diferent invariant error models and efectively handle large initial misalignment, we propose a novel multiplicative federated IEKF (MF-IEKF) framework as shown in Figure 1. The core contribution is an architectural innovation that fuses the advantages of multiple invariant models through a deep synergistic mechanism. Specifically:

1. Leveraging multiplicative pre-integration theory [40,41], we derive a state propagation equation that evolves on the SE2(3) Lie group, ensuring geometric consistency and computational eficiency.

2. Adopting the four invariant error models proposed in [36], we construct two parallel left-invariant error sub-filters using GNSS observations. By exploiting the inverse relationship between their error estimates, we perform collaborative correction that significantly improves steady-state accuracy.

3. During the initial phase, we introduce a multiplicative right-invariant sub-filter that exploits the attitude-error decoupling property of right-invariant observation models to accelerate velocity and position convergence, thereby effectively mitigating the slow initial convergence exhibited by left-invariant models under large misalignment.

4. Rigorous validation via extensive Monte Carlo simulations and experiments on public datasets demonstrates the efficacy of the proposed MF-IEKF. Compared with the multiplicative left-IEKF (ML1-IEKF) baseline, our method achieves consistent improvements of at least 5.26% in attitude, 8.72% in velocity, and 15.28% in position estimation.

![](images/71578fee81c92996fc42ead8e67555069459404b8c985399fafc8045d1eb7571.jpg)  
Figure 1. The proposed multiplicative federated invariant EKF framework.  represents the diferent error models.

## 2. IMU Model on Lie Group

This section develops a linearized inertial measurement unit (IMU) kinematic model within the Earth-Centered Earth-Fixed (ECEF) reference frame. The following coordinate systems are formally defined: e-frame (ECEF), b-frame (body-fixed coordinate system), iframe (inertial reference frame), and n-frame (local navigation frame).

## 2.1. IMU-Based Kinematic Equations

The kinematic diference equations for the atitude, velocity, and position in the eframe are given by

$$
\dot {\mathbf {R}} _ {b} ^ {e} = \mathbf {R} _ {b} ^ {e} \boldsymbol {\omega} _ {i b} ^ {b \wedge} - \boldsymbol {\omega} _ {i e} ^ {e \wedge} \mathbf {R} _ {b} ^ {e}\tag{1}
$$

$$
\dot {\pmb {v}} _ {e b} ^ {e} = \mathbf {R} _ {b} ^ {e} \pmb {a} _ {i b} ^ {b} - 2 \pmb {\omega} _ {i e} ^ {e \wedge} \pmb {v} _ {e b} ^ {e} + \pmb {g} _ {i b} ^ {e}\tag{2}
$$

$$
\dot {\pmb {p}} _ {e b} ^ {e} = \pmb {v} _ {e b} ^ {e}\tag{3}
$$

where $\mathbf { R } _ { b } ^ { e } \in \mathbb { R } ^ { 3 \times 3 }$ is the atitude matrix from b-frame to e-frame; the symbol $( \cdot ) _ { e b } ^ { e }$ in the velocity <sup>eb</sup>  and position <sup>eb</sup>  denotes the b-frame with respect to the e-frame in the e- $v _ { _ { e b } } ^ { e }$ $\pmb { p } _ { e b } ^ { e }$ frame; $\pmb { a } _ { i b } ^ { b } \mathrm { \bf \Pi } _ { \mathrm { a n d } } \ \pmb { \omega } _ { i b } ^ { b } \in \mathbb { R } ^ { 3 \times 1 }$ represent the acceleration and angular rate relative to the iframe expressed in the b-frame; $\boldsymbol { \omega } _ { i e } ^ { e } \in \mathbb { R } ^ { 3 \times 1 }$ is the angular rate of the Earth rotation in the e-frame; <sup>ib</sup>  is the gravity vector. The term $g _ { i b } ^ { e }$ $- 2 \omega _ { i e } ^ { e } \mathrm { \Delta } v _ { e b } ^ { e }$ are so called Coriolis acceleration, where the symbol $( \cdot ) ^ { \wedge } \in \mathbb { R } ^ { 3 \times 3 }$ is the skew-symmetric matrix operator. For a vector $\pmb { \omega } = \left[ \pmb { \omega } _ { x } , \pmb { \omega } _ { y } , \pmb { \omega } _ { z } \right] ^ { T }$ , its skew-symmetric matrix is defined as:

$$
\boldsymbol {\omega} ^ {\wedge} = \left[ \begin{array}{c c c} 0 & - \boldsymbol {\omega} _ {z} & \boldsymbol {\omega} _ {y} \\ \boldsymbol {\omega} _ {z} & 0 & - \boldsymbol {\omega} _ {x} \\ - \boldsymbol {\omega} _ {y} & \boldsymbol {\omega} _ {x} & 0 \end{array} \right].\tag{4}
$$

To establish a unified framework on the manifold, the paper employs an auxiliary velocity [40].

$$
\pmb {v} _ {i b} ^ {e} = \pmb {v} _ {e b} ^ {e} + \pmb {\omega} _ {i e} ^ {e \wedge} \pmb {p} _ {i b} ^ {e}\tag{5}
$$

where $p _ { i b } ^ { e }$ is the b-frame with respect to the i-frame in the e-frame. Given the colocation of origins between the e-frame and i-frame, there exists the equivalence relation ${ \pmb p } _ { i b } ^ { e } = { \pmb p } _ { e b } ^ { e }$ . Based on (3) and (5), the diferential equation of <sup>ib</sup>  can be writen as $p _ { i b } ^ { e }$

$$
\dot {\pmb {p}} _ {i b} ^ {e} = \dot {\pmb {p}} _ {e b} ^ {e} = \pmb {v} _ {e b} ^ {e} = \pmb {v} _ {i b} ^ {e} - \pmb {\omega} _ {i e} ^ {e \wedge} \pmb {p} _ {i b} ^ {e}\tag{6}
$$

${ \bf R } _ { b } ^ { e } , \ v _ { i b } ^ { e } \mathrm { a n d } \ p _ { i b } ^ { e }$ are embedded into the matrix Lie group $\mathrm { S E } _ { 2 } ( 3 )$ [22], the state X can be represented as

$$
\mathbf {X} = \left[ \begin{array}{c c c} \mathbf {R} _ {b} ^ {e} & \boldsymbol {v} _ {i b} ^ {e} & \boldsymbol {p} _ {i b} ^ {e} \\ \mathbf {0} _ {1 \times 3} & 1 & 0 \\ \mathbf {0} _ {1 \times 3} & 0 & 1 \end{array} \right] \in \mathbb {R} ^ {5 \times 5}\tag{7}
$$

Based on (1)–(6), the kinematic difference equation of state matrix X can be rewritten as

$$
\dot {\mathbf {X}} = g (\mathbf {X}) = \left[ \begin{array}{c c c} \dot {\mathbf {R}} _ {b} ^ {e} & \dot {\boldsymbol {v}} _ {i b} ^ {e} & \dot {\boldsymbol {p}} _ {i b} ^ {e} \\ \mathbf {0} _ {2 \times 3} & \mathbf {0} _ {2 \times 2} \end{array} \right] = \left[ \begin{array}{c c c} \mathbf {R} _ {b} ^ {e} \boldsymbol {\omega} _ {i b} ^ {b \wedge} - \boldsymbol {\omega} _ {i e} ^ {e \wedge} \mathbf {R} _ {b} ^ {e} & \mathbf {R} _ {b} ^ {e} \boldsymbol {a} _ {i b} ^ {b} - \boldsymbol {\omega} _ {i e} ^ {e \wedge} \boldsymbol {v} _ {i b} ^ {e} + \boldsymbol {G} _ {i b} ^ {e} & \boldsymbol {v} _ {i b} ^ {e} - \boldsymbol {\omega} _ {i e} ^ {e \wedge} \boldsymbol {p} _ {i b} ^ {e} \\ \mathbf {0} _ {2 \times 3} & \mathbf {0} _ {2 \times 2} \end{array} \right]\tag{8}
$$

where $G _ { i b } ^ { e } = ( \omega _ { i e } ^ { e } ) ^ { 2 } p _ { i b } ^ { e } + g _ { i b } ^ { e }$ is the gravitational vector, it can be derived through vector superposition of centrifugal acceleration with the gravity vector [36].

It is easily verified that (8) can be decomposed to [25]

$$
g (\mathbf {X}) = \mathbf {X U} + f (\mathbf {X}) + \mathbf {W X}\tag{9}
$$

where the matrix U is related to the input, and the form is

$$
\mathbf {U} = \left[ \begin{array}{c c c} \boldsymbol {\omega} _ {i b} ^ {b \wedge} & \boldsymbol {a} _ {i b} ^ {b} & \mathbf {0} _ {3 \times 1} \\ \mathbf {0} _ {2 \times 3} & & \mathbf {0} _ {2 \times 2} \end{array} \right]\tag{10}
$$

The matrix W is with respect to the Earth rotation and gravity vector in the e-frame, and it has

$$
\mathbf {W} = \left[ \begin{array}{c c} - \boldsymbol {\omega} _ {i e} ^ {e \wedge} & \mathbf {G} _ {i b} ^ {e} \quad \mathbf {0} _ {3 \times 1} \\ \mathbf {0} _ {2 \times 3} & \mathbf {0} _ {2 \times 2} \end{array} \right]\tag{11}
$$

$f ( \cdot )$ is a drift field related to velocity, as defined by

$$
f: \mathbf {X} = \left[ \begin{array}{c c c} \mathbf {R} _ {b} ^ {e} & \boldsymbol {v} _ {i b} ^ {e} & \boldsymbol {p} _ {i b} ^ {e} \\ \mathbf {0} _ {1 \times 3} & 1 & 0 \\ \mathbf {0} _ {1 \times 3} & 0 & 1 \end{array} \right] \mapsto f (\mathbf {X}) = \left[ \begin{array}{c c c} \mathbf {0} _ {3 \times 3} & \mathbf {0} _ {3 \times 1} & \boldsymbol {v} _ {i b} ^ {e} \\ \mathbf {0} _ {2 \times 3} & \mathbf {0} _ {2 \times 2} \end{array} \right]\tag{12}
$$

It can be easily checked that $g ( \pmb { X } )$ has group-afine property, and satisfies the following equation

$$
g (\mathbf {X} _ {1} \mathbf {X} _ {2}) = g (\mathbf {X} _ {1}) \mathbf {X} _ {2} + \mathbf {X} _ {1} g (\mathbf {X} _ {2}) - \mathbf {X} _ {1} g (\mathbf {I} _ {5 \times 5}) \mathbf {X} _ {2}\tag{13}
$$

where $\mathbf { X } _ { 1 } , \mathbf { X } _ { 2 } \in \mathrm { S E } _ { 2 } ( 3 )$ are the implementation of (6), and <sup>5 5×</sup>  is the group identity el- $\mathbf { I } _ { 5 \times 5 }$ ement in the $\mathrm { S E } _ { 2 } ( 3 )$ . Obviously, $f ( \cdot )$ also has the group-afine property

$$
f (\mathbf {X} _ {1} \mathbf {X} _ {2}) = f (\mathbf {X} _ {1}) \mathbf {X} _ {2} + \mathbf {X} _ {1} f (\mathbf {X} _ {2}) - \mathbf {X} _ {1} f (\mathbf {I} _ {5 \times 5}) \mathbf {X} _ {2}\tag{14}
$$

since $f \left( \mathbf { I } _ { 5 \times 5 } \right) = \mathbf { 0 } _ { 5 \times 5 }$ , (23) can be updated as

$$
f (\mathbf {X} _ {1} \mathbf {X} _ {2}) = f (\mathbf {X} _ {1}) \mathbf {X} _ {2} + \mathbf {X} _ {1} f (\mathbf {X} _ {2})\tag{15}
$$

## 2.2. Decomposition of IMU State Matrix on the Li Group in Continuous Time

As the group afine property of the $f ( \mathbf { X } )$ , the state matrix X at arbitrary time t in the e-frame can be decomposed into a product function of the initial state <sup>0</sup> as [40] $\mathbf { X } _ { 0 }$

$$
\mathbf {X} _ {t} = \mathbf {L} _ {t} \boldsymbol {\Phi} _ {t} (\mathbf {X} _ {0}) \boldsymbol {\Upsilon} _ {t}\tag{16}
$$

where the concrete definition of $\Phi _ { t } ( \cdot )$ can be given as [40]

$$
\boldsymbol {\Phi} _ {t}: \mathbf {X} = \left[ \begin{array}{c c c} \mathbf {R} _ {b} ^ {e} & \boldsymbol {v} _ {i b} ^ {e} & \boldsymbol {p} _ {i b} ^ {e} \\ \mathbf {0} _ {1 \times 3} & 1 & 0 \\ \mathbf {0} _ {1 \times 3} & 0 & 1 \end{array} \right] \mapsto \left[ \begin{array}{c c c} \mathbf {R} _ {b} ^ {e} & \boldsymbol {v} _ {i b} ^ {e} & \boldsymbol {p} _ {i b} ^ {e} + t \boldsymbol {v} _ {i b} ^ {e} \\ \mathbf {0} _ {1 \times 3} & 1 & 0 \\ \mathbf {0} _ {1 \times 3} & 0 & 1 \end{array} \right] = \mathbf {X} + t \cdot f (\mathbf {X})\tag{17}
$$

Obviously, $\Phi _ { t } ( \cdot )$ only depends on the time t. It is easy to verify the group automorphism property

$$
\boldsymbol {\Phi} _ {t} (\mathbf {X} _ {1} \mathbf {X} _ {2}) = \boldsymbol {\Phi} _ {t} (\mathbf {X} _ {1}) \boldsymbol {\Phi} _ {t} (\mathbf {I} _ {5 \times 5}) \boldsymbol {\Phi} _ {t} (\mathbf {X} _ {2}) = \boldsymbol {\Phi} _ {t} (\mathbf {X} _ {1}) \boldsymbol {\Phi} _ {t} (\mathbf {X} _ {2})\tag{18}
$$

According to the definition of $\vdots ,$ it can be concluded that $\Phi _ { t } ( \mathbf { I } _ { 5 \times 5 } ) = \mathbf { I } _ { 5 \times 5 }$

The concrete form of <sup>t</sup>  in (16) can be given as $\mathbf { L } _ { t }$

$$
\mathbf {L} _ {t} = \left[ \begin{array}{c c c} \mathbf {L} _ {t} ^ {R} & \mathbf {L} _ {t} ^ {v} & \mathbf {L} _ {t} ^ {p} \\ \mathbf {0} _ {1 \times 3} & 1 & 0 \\ \mathbf {0} _ {1 \times 3} & 0 & 1 \end{array} \right]\tag{19}
$$

Simultaneously, <sup>t</sup>  is the solution of diferential Equation (20) $\mathbf { L } _ { \phantom { } _ { t } }$

$$
\dot {\mathbf {L}} _ {t} = \left[ \begin{array}{c c c} - \boldsymbol {\omega} _ {i e} ^ {e} \wedge \mathbf {L} _ {t} ^ {R} & \boldsymbol {G} _ {i b} ^ {e} - \boldsymbol {\omega} _ {i e} ^ {e} \wedge \mathbf {L} _ {t} ^ {v} & \mathbf {L} _ {t} ^ {v} - \boldsymbol {\omega} _ {i e} ^ {e} \wedge \mathbf {L} _ {t} ^ {p} \\ \mathbf {0} _ {1 \times 3} & 1 & 0 \\ \mathbf {0} _ {1 \times 3} & 0 & 1 \end{array} \right] = \mathbf {W L} _ {t} + f (\mathbf {L} _ {t})\tag{20}
$$

where W is defined in (11). Based on the diferential equation in $( 2 0 ) , \mathbf { L } _ { t }$ represents the global increment in the e-frame that undergoes slow variation due to the action of the Earth rotation. In addition, the initial value is ${ \bf L } _ { 0 } = { \bf I } _ { 5 \times 5 }$

Meanwhile, the concrete form of $\mathbf { Y } _ { t }$ in (16) can be writen as

$$
\mathbf {\Upsilon} _ {t} = \left[ \begin{array}{c c c} \mathbf {\Upsilon} _ {t} ^ {R} & \mathbf {\Upsilon} _ {t} ^ {v} & \mathbf {\Upsilon} _ {t} ^ {p} \\ \mathbf {0} _ {1 \times 3} & 1 & 0 \\ \mathbf {0} _ {1 \times 3} & 0 & 1 \end{array} \right]\tag{21}
$$

and <sup>t</sup> is the solution of the diferential Equation (22) $\mathbf { Y } _ { t }$

$$
\dot {\mathbf {Y}} _ {t} = \left[ \begin{array}{c c c} \mathbf {Y} _ {t} ^ {R} \boldsymbol {\omega} _ {i b} ^ {b \wedge} & \mathbf {Y} _ {t} ^ {R} \boldsymbol {a} _ {i b} ^ {b} & \mathbf {Y} _ {t} ^ {v} \\ \mathbf {0} _ {1 \times 3} & 1 & 0 \\ \mathbf {0} _ {1 \times 3} & 0 & 1 \end{array} \right] = \mathbf {U Y} _ {t} + f (\mathbf {Y} _ {t})\tag{22}
$$

where U is defined in (10). Based on (22), <sup>t</sup> is associated with the high-frequency input $\mathbf { Y } _ { t }$ from the ${ \mathrm { I M U } } ,$ namely the acceleration <sup>ib</sup>  and angular velocity $\pmb { a } _ { i b } ^ { b }$ $\omega _ { i b } ^ { b }$ . It reflects the local increment during the motion process in the b-frame, and it can be used to pre-integrate. Similarly to $\mathbf { L } _ { 0 }$ , the initial value is also $\mathbf { Y } _ { 0 } = \mathbf { I } _ { 5 \times 5 }$ . It should be noted that both $\mathbf { L } _ { t }$ and $\mathbf { Y } _ { t }$ $\mathbf { X } _ { 0 }$ depend on the initial state

The state matrix <sup>t</sup> in (16) can be expanded as $\mathbf { X } _ { t }$

$$
\mathbf {X} _ {t} = \left[ \begin{array}{c c c} \mathbf {L} _ {t} ^ {R} \boldsymbol {R} _ {0} \boldsymbol {\Upsilon} _ {t} ^ {R} & \mathbf {L} _ {t} ^ {R} (\boldsymbol {R} _ {0} \boldsymbol {\Upsilon} _ {t} ^ {v} + \boldsymbol {v} _ {0}) + \mathbf {L} _ {t} ^ {v} & \mathbf {L} _ {t} ^ {R} (\boldsymbol {R} _ {0} \boldsymbol {\Upsilon} _ {t} ^ {p} + \boldsymbol {p} _ {0} + t \boldsymbol {v} _ {0}) + \mathbf {L} _ {t} ^ {p} \\ \mathbf {0} _ {1 \times 3} & 1 & 0 \\ \mathbf {0} _ {1 \times 3} & 0 & 1 \end{array} \right]\tag{23}
$$

where $\boldsymbol { \mathrm { R } } _ { \ u { 0 } , \ u } ^ { \mathrm { ~ } }$ and $p _ { 0 }$ with their frame notations omited here due to space constraints, correspond to $\mathbf { R } _ { b , 0 } ^ { e } , \ v _ { e b , 0 } ^ { e }$ and ${ p ^ { e } } _ { e b , 0 } ,$ separately.

$\mathbf { X } _ { t }$ Hereby, the paper establishes the equations for the continuous-time state matrix in the e-frame.

## 2.3. The State Matrix on Li Group in Discrete Time

In practical implementations, state estimation systems inherently process discretized sensor inputs, such as high-frequency acceleration and angular velocity measurements sampled at discrete intervals. This operational constraint necessitates the discretization of the continuous formulation presented in (16).

The state $\mathbf { X } _ { t }$ at the time k + 1 can be given as

$$
\mathbf {X} _ {k + 1} = \mathbf {L} _ {k + 1} \boldsymbol {\Phi} _ {k + 1} (\mathbf {X} _ {0}) \mathbf {\Upsilon} _ {k + 1}\tag{24}
$$

where $\mathbf { L } _ { \boldsymbol { k } + 1 } \mathbf { \Lambda } _ { \mathrm { a n d } } \ \mathbf { Y } _ { \boldsymbol { k } + 1 }$ <sup>1</sup> were defined in (19) and (21). According to (17), $\Phi _ { k + 1 } ( \mathbf { X } _ { 0 } )$ can be writen as

$$
\mathbf {\Phi} _ {k + 1} (\mathbf {X} _ {0}) = \left[ \begin{array}{c c c} \mathbf {R} _ {0} & v _ {0} & p _ {0} + t _ {k} v _ {i b} ^ {e} \\ \mathbf {0} _ {1 \times 3} & 1 & 0 \\ \mathbf {0} _ {1 \times 3} & 0 & 1 \end{array} \right]\tag{25}
$$

The initial objective focuses on deriving the temporal evolution of the global increment <sup>k</sup> across consecutive time steps. Through discretization of the continuous formu- $\mathbf { L } _ { \boldsymbol { k } }$ lation in (20), the discrete-time representations of global increments $\mathbf { L } _ { k + 1 } ^ { R } , \mathbf { L } _ { k + 1 } ^ { v }$ <sup>1</sup> , and $\mathbf { L } _ { k + 1 } ^ { p }$ at discrete time <sup>k</sup> <sup>1</sup>  <sup>+</sup> are expressed through the following relationships $t _ { k + 1 }$

$$
\mathbf {L} _ {k + 1} = \mathbf {L} _ {\Delta t} \boldsymbol {\Phi} _ {\Delta t} (\mathbf {L} _ {k})\tag{26}
$$

where

$$
\mathbf {L} _ {\Delta t} = \left[ \begin{array}{c c c} \mathbf {L} _ {\Delta t} ^ {R} & \mathbf {L} _ {\Delta t} ^ {v} & \mathbf {L} _ {\Delta t} ^ {p} \\ \mathbf {0} _ {1 \times 3} & 1 & 0 \\ \mathbf {0} _ {1 \times 3} & 0 & 1 \end{array} \right]\tag{27}
$$

$$
\boldsymbol {\Phi} _ {\Delta t} (\mathbf {L} _ {k}) = \left[ \begin{array}{c c c} \mathbf {L} _ {k} ^ {R} & \mathbf {L} _ {k} ^ {v} & \Delta t \mathbf {L} _ {k} ^ {v} + \mathbf {L} _ {k} ^ {p} \\ \mathbf {0} _ {1 \times 3} & 1 & 0 \\ \mathbf {0} _ {1 \times 3} & 0 & 1 \end{array} \right]\tag{28}
$$

$$
\mathbf {L} _ {\Delta t} ^ {R} = \Gamma_ {0} \left(- \pmb {\omega} _ {i e} ^ {e} \Delta t\right)
$$

$$
\mathbf {L} _ {\Delta t} ^ {v} = \Gamma_ {1} \left(- \pmb {\omega} _ {i e} ^ {e} \Delta t\right) \Delta t \pmb {G} _ {i b} ^ {e}\tag{29}
$$

$$
\mathbf {L} _ {\Delta t} ^ {p} = \Gamma_ {2} \left(- \boldsymbol {\omega} _ {i e} ^ {e} \Delta t\right) \Delta t ^ {2} \mathbf {G} _ {i b} ^ {e}
$$

The detailed derivation of $\mathbf { L } _ { \Delta t }$ can be found in Appendix A. The definition of function $\Gamma _ { { m } } \left( \varphi \right)$ is [29]

$$
\Gamma_ {m} (\varphi) \triangleq \sum_ {n = 0} ^ {\infty} \frac {1}{(n + m) !} \left(\varphi^ {\wedge}\right) ^ {n}, \quad \varphi \in \mathbb {R} ^ {3 \times 1}, \quad m = 0, 1, 2 \dots\tag{30}
$$

The above derivation quantifies the variation in the global increment across consecutive time intervals. Next, we now turn to the evolution of the local increment t over $\mathbf { Y } _ { t }$ consecutive time intervals. Based on (22), <sup>t</sup> at time <sup>k</sup> <sup>1</sup>  <sup>+</sup> can be expressed as $\mathbf { Y } _ { t }$ $t _ { k + 1 }$

$$
\boldsymbol {\Upsilon} _ {k + 1} = \boldsymbol {\Phi} _ {\Delta t} (\boldsymbol {\Upsilon} _ {k}) \boldsymbol {\Upsilon} _ {\Delta t}\tag{31}
$$

where

$$
\boldsymbol {\Phi} _ {\Delta t} (\boldsymbol {\Upsilon} _ {k}) = \left[ \begin{array}{c c c} \boldsymbol {\Upsilon} _ {k} ^ {R} & \boldsymbol {\Upsilon} _ {k} ^ {v} & \Delta t \boldsymbol {\Upsilon} _ {k} ^ {v} + \boldsymbol {\Upsilon} _ {k} ^ {p} \\ \mathbf {0} _ {1 \times 3} & 1 & 0 \\ \mathbf {0} _ {1 \times 3} & 0 & 1 \end{array} \right]\tag{32}
$$

$$
\boldsymbol {\Upsilon} _ {\Delta t} = \left[ \begin{array}{c c c} \boldsymbol {\Upsilon} _ {\Delta t} ^ {R} & \boldsymbol {\Upsilon} _ {\Delta t} ^ {v} & \boldsymbol {\Upsilon} _ {\Delta t} ^ {p} \\ \mathbf {0} _ {1 \times 3} & 1 & 0 \\ \mathbf {0} _ {1 \times 3} & 0 & 1 \end{array} \right]\tag{33}
$$

where $\mathrm { ~ \bf ~ r ~ } _ { \Delta t } ^ { R } , \mathrm { ~ \bf ~ r ~ } _ { \Delta t } ^ { v }$ and <sup>Δt</sup> are the local increments between two consecutive discrete- $\mathbf { Y } _ { \Delta t } ^ { p }$ time intervals $\Delta t$ , and based on $( 2 2 ) , \ \mathbf { Y } _ { \Delta t }$ can be obtained as

$$
\begin{array}{r} \mathbf {Y} _ {\Delta t} ^ {R} = \Gamma_ {0} (\pmb {\omega} _ {i b} ^ {b} \Delta t) \\ \mathbf {Y} _ {\Delta t} ^ {v} = \Gamma_ {1} (\pmb {\omega} _ {i b} ^ {b} \Delta t) \Delta t \pmb {a} _ {i b} ^ {b} \\ \mathbf {Y} _ {\Delta t} ^ {p} = \Gamma_ {2} (\pmb {\omega} _ {i b} ^ {b} \Delta t) \Delta t ^ {2} \pmb {a} _ {i b} ^ {b} \end{array}\tag{34}
$$

The detailed derivation of <sup>Δt</sup> can be found in Appendix A, where $\mathbf { Y } _ { \Delta t }$ $\Gamma _ { { m } } \left( \varphi \right)$ was defined in (30); the inputs ${ \pmb a } _ { i b \mathrm { \mathrm { ~ \tiny ~ a n d ~ } } } ^ { b } { \pmb \omega } _ { i b } ^ { b }$ are assumed to be constant during each discretetime interval <sup>t</sup> and take the initial values at $\Delta t$ $t _ { \iota _ { k } }$ . This assumption is reasonable and common when processing the IMU data.

By substituting (26) and (31) into (24), the discrete-time state equation can be derived as follows

$$
\mathbf {X} _ {k + 1} = \mathbf {L} _ {\Delta t} \boldsymbol {\Phi} _ {\Delta t} (\mathbf {X} _ {k}) \mathbf {\Upsilon} _ {\Delta t}\tag{35}
$$

Proof:

$$
\begin{array}{l} \mathbf {X} _ {k + 1} = \mathbf {L} _ {k + 1} \boldsymbol {\Phi} _ {k + 1} (\mathbf {X} _ {0}) \boldsymbol {\Upsilon} _ {k + 1} \\ \quad = \mathbf {L} _ {\Delta t} \boldsymbol {\Phi} _ {\Delta t} (\mathbf {L} _ {k}) \boldsymbol {\Phi} _ {k + \Delta t} (\mathbf {X} _ {0}) \boldsymbol {\Phi} _ {\Delta t} (\boldsymbol {\Upsilon} _ {k}) \boldsymbol {\Upsilon} _ {\Delta t} \\ \quad = \mathbf {L} _ {\Delta t} \boldsymbol {\Phi} _ {\Delta t} (\mathbf {L} _ {k}) \boldsymbol {\Phi} _ {\Delta t} (\boldsymbol {\Phi} _ {k} (\mathbf {X} _ {0})) \boldsymbol {\Phi} _ {\Delta t} (\boldsymbol {\Upsilon} _ {k}) \boldsymbol {\Upsilon} _ {\Delta t} \\ \quad = \mathbf {L} _ {\Delta t} \boldsymbol {\Phi} _ {\Delta t} (\mathbf {L} _ {k} \boldsymbol {\Phi} _ {k} (\mathbf {X} _ {0}) \boldsymbol {\Upsilon} _ {k}) \boldsymbol {\Upsilon} _ {\Delta t} \\ \quad = \mathbf {L} _ {\Delta t} \boldsymbol {\Phi} _ {\Delta t} (\mathbf {X} _ {k}) \boldsymbol {\Upsilon} _ {\Delta t} \end{array}\tag{36}
$$

where $\Phi _ { t } \left( \mathbf { X } \right)$ possesses another cumulative property with respect to time and the proof is presented as following

$$
\begin{array}{r l} \boldsymbol {\Phi} _ {t _ {i}} (\boldsymbol {\Phi} _ {t _ {j}} (\mathbf {X})) & = \boldsymbol {\Phi} _ {t _ {i}} (\mathbf {X} + t _ {j} f (\mathbf {X})) \\ & = \mathbf {X} + t _ {j} f (\mathbf {X}) + t _ {i} f (\mathbf {X} + f (\mathbf {X})) \\ & = \mathbf {X} + (t _ {i} + t _ {j}) f (\mathbf {X}) = \boldsymbol {\Phi} _ {t _ {i} + t _ {j}} (\mathbf {X}) \end{array}\tag{37}
$$

As demonstrated in (35), we have established the discrete time propagation equation for the state matrix X. This formulation exhibits intrinsic multiplicative properties that enable unified computation within the Lie group framework, in contrast to traditional

$$
\mathbf {X} _ {k + 1}
$$

EKFs that rely on additive error formulations. The updated state <sup>Xk+1</sup> comprises three geometrically interpretable components:

1. Global increment $\mathbf { L } _ { \Delta t }$ induced by Earth’s rotation.

2. Prior state $\Phi _ { \Delta t } ( \mathbf { X } _ { k } )$

3. Local increment $\mathrm { \Delta } \Upsilon _ { \Delta t }$ generated by IMU high-frequency inputs.

## 3. Invariant Error State in Earth Frame

3.1. Four Invariant Errors in E-Frame

$\mathrm { S E } _ { 2 } ( 3 )$ Similarly to the state matrix X, the error state is also defined on the Lie group

Definition 1 (invariant error [22]). The left- and right-invariant error matrices between the true state $\mathbf { X } \in \mathrm { S E } _ { 2 } ( 3 )$ and estimation $\hat { \mathbf { X } } \in \mathrm { S E } _ { 2 } ( 3 )$ are defined as

Right-invariant error matrix:

$$
\boldsymbol {\eta} ^ {r 1} = \hat {\mathbf {X}} \mathbf {X} ^ {- 1}\tag{38}
$$

Left-invariant error matrix:

$$
\pmb {\eta} ^ {l 1} = \pmb {X} ^ {- 1} \hat {\pmb {X}}\tag{39}
$$

Invariance Property: The term “invariant” signifies a key geometric property of these definitions. Consider an arbitrary fixed transformation $\mathbf { C } \in \mathrm { S E } _ { 2 } ( 3 )$ . If both states are transformed by the same left-multiplication $( \mathrm { i . e . }$ <sub>,</sub> X CX → <sub>and</sub> $\hat { \mathbf { X } } \to \mathbf { C } \hat { \mathbf { X } } _ { ) }$ , the computed error remains unchanged.

X and the estimate <sup>X</sup> by the same transformation C does not alter the error. Alter- $\hat { \mathbf { x } }$ native definitions:

Right-invariant error matrix:

$$
\boldsymbol {\eta} ^ {r 1} = \hat {\mathbf {X}} \mathbf {X} ^ {- 1} = (\mathbf {C} \hat {\mathbf {X}}) (\mathbf {C X}) ^ {- 1}\tag{40}
$$

Left-invariant error matrix:

$$
\boldsymbol {\eta} ^ {l 1} = \mathbf {X} ^ {- 1} \hat {\mathbf {X}} = (\mathbf {C X}) ^ {- 1} (\mathbf {C} \hat {\mathbf {X}})\tag{41}
$$

The introduction of the arbitrary matrix C in the above equalities is solely to demonstrate this invariance. It proves that the errors  and  depend only on the relative $\boldsymbol { \mathsf { \pmb { \eta } } } ^ { r 1 }$ $\boldsymbol { \mathsf { \Omega } } \boldsymbol { \mathsf { \Omega } } ^ { l 1 }$ diference between <sup>X</sup> and $\hat { \mathbf { x } }$ $\mathbf { \delta x , }$ and not on their absolute coordinates in the state space. This property is fundamental for the development of invariant filters, such as the MF-IEKF proposed in this work, which are known to exhibit improved convergence characteristics, especially under large misalignment conditions.

Exploiting this frame-invariance, one can equivalently define the errors inversely [30], leading to an alternative set of invariant errors:

Alternative right-invariant error matrix:

$$
\boldsymbol {\eta} ^ {r 2} = \mathbf {X} \hat {\mathbf {X}} ^ {- 1} = (\mathbf {C X}) (\mathbf {C} \hat {\mathbf {X}}) ^ {- 1}\tag{42}
$$

Alternative left-invariant error matrix:

$$
\boldsymbol {\eta} ^ {l 2} = \hat {\mathbf {X}} ^ {- 1} \mathbf {X} = (\mathbf {C} \hat {\mathbf {X}}) ^ {- 1} (\mathbf {C X}).\tag{43}
$$

It is evident that error states $\boldsymbol { \eta } ^ { r 1 } = ( \boldsymbol { \eta } ^ { r 2 } ) ^ { - 1 } \mathrm { a n d } \boldsymbol { \eta } ^ { l 1 } = ( \boldsymbol { \eta } ^ { l 2 } ) ^ { - 1 }$ represent two opposing characterizations on the manifold.

## 3.2. Noise Model and Approximation

IMU measurements comprise three principal components: true kinematic values, sensor biases, and stochastic noise. These components can be mathematically modeled through the following

$$
\tilde {\boldsymbol {a}} _ {i b} ^ {b} = \boldsymbol {a} _ {i b} ^ {b} + \boldsymbol {b} _ {a} + \boldsymbol {n} _ {a}\tag{44}
$$

$$
\tilde {\boldsymbol {\omega}} _ {i b} ^ {b} = \boldsymbol {\omega} _ {i b} ^ {b} + \boldsymbol {b} _ {\omega} + \boldsymbol {n} _ {\omega}\tag{45}
$$

where <sup>a</sup> and <sup>ω</sup> are the acceleration and gyroscope measurements expressed in the b- $\tilde { \mathbf { \ b { a } } }$ $\tilde { \omega }$ frame; <sup>a</sup>  and <sup>ω</sup>  modeled as white Gaussian noise processes, satisfy ${ \pmb n } _ { a }$ ${ \pmb n } _ { \omega }$ $\pmb { n } _ { a } \sim \mathcal { N } ( 0 , \pmb { \sigma } _ { a } ^ { 2 } )$ and $\pmb { n } _ { \omega } \sim \mathcal { N } ( 0 , \pmb { \sigma } _ { \omega } ^ { 2 } )$ , respectively; $b _ { a \mathrm { ~ \tiny ~ \ a n d ~ } } b _ { \omega }$ denote the acceleration and gyroscope biases, whose temporal evolution follows Gaussian Markov processes.

$$
\dot {\boldsymbol {b}} = \left[ \begin{array}{c} \dot {\boldsymbol {b}} _ {a} \\ \dot {\boldsymbol {b}} _ {\omega} \end{array} \right] = \left[ \begin{array}{c c} - \operatorname{diag} \left(\frac {1}{\tau_ {a}}\right) & \boldsymbol {0} _ {3 \times 3} \\ \boldsymbol {0} _ {3 \times 3} & - \operatorname{diag} \left(\frac {1}{\tau_ {\omega}}\right) \end{array} \right] \left[ \begin{array}{c} \boldsymbol {b} _ {a} \\ \boldsymbol {b} _ {\omega} \end{array} \right] + \left[ \begin{array}{c} \boldsymbol {n} _ {b _ {a}} \\ \boldsymbol {n} _ {b _ {\omega}} \end{array} \right] = \mathbf {F} _ {b} \boldsymbol {b} + \boldsymbol {n} _ {b}\tag{46}
$$

The noise in the IMU is not negligible, and the efect of such noise can be characterized as a right perturbation on the local increment

$$
\mathbf {\Upsilon} _ {\Delta t} = \hat {\mathbf {\Upsilon}} _ {\Delta t} \exp (- N _ {\Delta t}) = \hat {\mathbf {\Upsilon}} _ {\Delta t} \exp (- \mathbf {G} _ {k} \pmb {n} _ {k})\tag{47}
$$

where $N _ { \Delta t }$ is the perturbation and satisfies ${ \cal N } _ { \Delta t } \sim \mathcal { N } ( { \bf 0 } _ { \mathrm { g } _ { \times 1 } } , { \bf G } _ { k } \mathrm { c o v } ( n _ { k } ) { \bf G } _ { k } ^ { \mathrm { T } } )$ ; $\pmb { n } _ { k } = \left[ \pmb { n } _ { \omega } ^ { \mathrm { T } } \quad \pmb { n } _ { a } ^ { \mathrm { T } } \right] ^ { \mathrm { T } }$ defined in (44) and (45); and $\mathbf { G } _ { k } \in \mathbb { R } ^ { 9 \times 6 }$ is the Jacobi matrix with respect to the <sup>k</sup>  , and can be writen as ${ \pmb n } _ { k }$

$$
\mathbf {G} _ {k} = \left[ \begin{array}{c c} \Gamma_ {0} ^ {- 1} \Gamma_ {1} \Delta t & \mathbf {0} _ {3 \times 3} \\ - \Gamma_ {0} ^ {- 1} \Gamma_ {1} \hat {\boldsymbol {a}} _ {i b} ^ {b} \wedge \Gamma_ {2} ^ {\mathrm{T}} \Delta t ^ {2} & \Gamma_ {0} ^ {- 1} \Gamma_ {1} \Delta t \\ - \Gamma_ {0} ^ {- 1} \Gamma_ {2} \hat {\boldsymbol {a}} _ {i b} ^ {b} \wedge \Gamma_ {3} ^ {\mathrm{T}} \Delta t ^ {3} & \Gamma_ {0} ^ {- 1} \Gamma_ {2} \Delta t ^ {2} \end{array} \right]\tag{48}
$$

$\mathbf { G } _ { \boldsymbol { k } }$ where the detailed derivation of <sup>k</sup> can be found in Appendix B.

$\hat { \mathbf { Y } } _ { \Delta t }$ corresponds to the local increment that encompasses uncertainty. Based on (34), $\hat { \mathbf { Y } } _ { \Delta t }$ can be expressed as

$$
\hat {\mathbf {Y}} _ {\Delta t} = \left[ \begin{array}{c c c} \hat {\mathbf {Y}} _ {\Delta t} ^ {R} & \hat {\mathbf {Y}} _ {\Delta t} ^ {v} & \hat {\mathbf {Y}} _ {\Delta t} ^ {p} \\ 0 _ {1 \times 3} & 1 & 0 \\ 0 _ {1 \times 3} & 0 & 1 \end{array} \right]\tag{49}
$$

where

$$
\begin{array}{c} \hat {\mathbf {Y}} _ {\Delta t} ^ {R} = \Gamma_ {0} (\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t) \\ \hat {\mathbf {Y}} _ {\Delta t} ^ {v} = \Gamma_ {1} (\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t) \hat {\boldsymbol {a}} _ {i b} ^ {b} \Delta t \\ \hat {\mathbf {Y}} _ {\Delta t} ^ {p} = \Gamma_ {2} (\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t) \hat {\boldsymbol {a}} _ {i b} ^ {b} \Delta t ^ {2} \end{array}\tag{50}
$$

where $\begin{array} { r } { \hat { \pmb { \omega } } _ { i b } ^ { b } = \tilde { \pmb { \omega } } _ { i b } ^ { b } - \pmb { b } _ { \omega } \quad \hat { \pmb { a } } _ { i b } ^ { b } = \tilde { \pmb { a } } _ { i b } ^ { b } - \pmb { b } _ { a } } \end{array}$

3.3. Right- and Left-Invariant Error Kinematic Equations Based the Lie Algebra in Discrete Time

This section maps the error state matrices <sup>η</sup> in the Lie group ((40)–(43)) to Lie algebras $\pmb { x } = [ \pmb { x } _ { \varphi } ^ { \mathrm { T } } \quad \pmb { x } _ { v } ^ { \mathrm { T } } \quad \pmb { x } _ { p } ^ { \mathrm { T } } ] ^ { \mathrm { T } } \in \mathbb { R } ^ { 9 }$ , where $x _ { \varphi } \ : , \ : \ : x _ { v } \ : , \ : \ : x _ { p }$ correspond to the rotation, velocity, and position errors, respectively. These mappings exhibit the following relationship

$$
\boldsymbol {\eta} = \exp (\boldsymbol {x}) = \operatorname{Exp} \left(\boldsymbol {x} ^ {\wedge}\right)\tag{51}
$$

Moreover, based on the definition of $\Phi _ { t } ( \cdot )$ in (26), the log-linearity property can be straightforwardly verified as

$$
\left(\boldsymbol {\Phi} _ {t} (\boldsymbol {X})\right) ^ {- 1} = \boldsymbol {\Phi} _ {t} \left(\boldsymbol {X} ^ {- 1}\right)\tag{52}
$$

and

$$
\boldsymbol {\Phi} _ {t} (\exp (\boldsymbol {x})) = \boldsymbol {\Phi} _ {t} (\operatorname{Exp} (\boldsymbol {x} ^ {\wedge})) = \operatorname{Exp} (\mathbf {F} _ {t} \boldsymbol {x})\tag{53}
$$

where $\mathbf { F } _ { t }$ can be written as

$$
\mathbf {F} _ {t} = \left[ \begin{array}{l l l} \mathbf {I} _ {3 \times 3} & \mathbf {0} _ {3 \times 3} & \mathbf {0} _ {3 \times 3} \\ \mathbf {0} _ {3 \times 3} & \mathbf {I} _ {3 \times 3} & \mathbf {0} _ {3 \times 3} \\ \mathbf {0} _ {3 \times 3} & t \mathbf {I} _ {3 \times 3} & \mathbf {I} _ {3 \times 3} \end{array} \right]\tag{54}
$$

The adjoint operator of the state matrix X defined in (7) can be conveniently expressed as

$$
\mathbf {A d} _ {x} := \left[ \begin{array}{c c c} \mathbf {R} & \mathbf {0} _ {3 \times 3} & \mathbf {0} _ {3 \times 3} \\ v ^ {\wedge} \mathbf {R} & \mathbf {R} & \mathbf {0} _ {3 \times 3} \\ p ^ {\wedge} \mathbf {R} & \mathbf {0} _ {3 \times 3} & \mathbf {R} \end{array} \right] \in \mathbb {R} ^ {9 \times 9}\tag{55}
$$

Since (55) is a coordinate-agnostic mathematical definition, the subscripts and superscripts for $\mathbf { R } , v ,$ and p are omited.

And, it exhibits a useful property [40] as follows:

$$
\mathbf {X} \mathrm{exp} (\boldsymbol {x}) \mathbf {X} ^ {- 1} = \mathrm{exp} (\mathbf {A d} _ {\boldsymbol {x}} \boldsymbol {x})\tag{56}
$$

The right-invariant error state matrix $\boldsymbol { \mathsf { \pmb { \eta } } } _ { \boldsymbol { k } + 1 } ^ { r 1 }$ in (40) can be further deduced as

$$
\begin{array}{l} \boldsymbol {\eta} _ {k + 1} ^ {r 1} = \hat {\mathbf {X}} _ {k + 1} \mathbf {X} _ {k + 1} ^ {- 1} \\ \quad = \mathbf {L} _ {\Delta t} \boldsymbol {\Phi} _ {\Delta t} (\hat {\mathbf {X}} _ {k}) \hat {\boldsymbol {\Upsilon}} _ {\Delta t} \mathbf {X} _ {k + 1} ^ {- 1} \\ \quad = \mathbf {L} _ {\Delta t} \boldsymbol {\Phi} _ {\Delta t} (\boldsymbol {\eta} _ {k} ^ {r 1} \mathbf {X} _ {k}) \boldsymbol {\Upsilon} _ {\Delta t} \exp (- N _ {\Delta k}) ^ {- 1} \mathbf {X} _ {k + 1} ^ {- 1} \\ \quad = \mathbf {L} _ {\Delta t} \boldsymbol {\Phi} _ {\Delta t} (\boldsymbol {\eta} _ {k} ^ {r 1}) \mathbf {L} _ {\Delta t} ^ {- 1} \cdot \mathbf {L} _ {\Delta t} \boldsymbol {\Phi} _ {\Delta t} (\mathbf {X} _ {k}) \boldsymbol {\Upsilon} _ {\Delta t} \exp (- N _ {\Delta k}) ^ {- 1} \mathbf {X} _ {k + 1} ^ {- 1} \\ \quad = \exp (\mathbf {A d} _ {\mathrm{L} _ {\Delta t}} \mathbf {F} _ {\Delta t} \boldsymbol {x} _ {k} ^ {r 1}) \exp (\mathbf {A d} _ {\mathbf {x} _ {k + 1} ^ {r 1}} \mathbf {N} _ {\Delta k}) \end{array}\tag{57}
$$

Applying the BCH formula [44], the kinematic equation governing the prior rightinvariant error in its Lie algebra form is given by

$$
\boldsymbol {x} _ {k + 1} ^ {r 1 -} \approx \mathbf {A d} _ {\mathbf {L} _ {\Delta t}} \mathbf {F} _ {\Delta t} \boldsymbol {x} _ {k} ^ {r 1 +} + \mathbf {A d} _ {\mathbf {X} _ {k + 1} ^ {r 1}} \boldsymbol {N} _ {\Delta k}\tag{58}
$$

$$
\mathbf {A} \mathbf {d} _ {\mathbf {L} _ {\Delta t}} = \left[ \begin{array}{c c c} \mathbf {L} _ {\Delta t} ^ {R} & \mathbf {0} _ {3 \times 3} & \mathbf {0} _ {3 \times 3} \\ \mathbf {L} _ {\Delta t} ^ {v} \wedge \mathbf {L} _ {\Delta t} ^ {R} & \mathbf {L} _ {\Delta t} ^ {R} & \mathbf {0} _ {3 \times 3} \\ \mathbf {L} _ {\Delta t} ^ {p} \wedge \mathbf {L} _ {\Delta t} ^ {R} & \mathbf {0} _ {3 \times 3} & \mathbf {L} _ {\Delta t} ^ {R} \end{array} \right]
$$

where superscript $^ { \prime \prime } - ^ { \prime \prime }$ denotes the prior error state, and $\mathbf { L } _ { \Delta t } ^ { R } , \mathbf { L } _ { \Delta t \mathrm { a d } } ^ { v } \mathbf { L } _ { \Delta t } ^ { p }$ can be obtained from (29) and (40).

Obviously, another prior right-invariant error state <sup>1k+</sup> in (41) is the inverse of $\boldsymbol { \mathsf { \pmb { \eta } } } _ { \boldsymbol { k } + 1 } ^ { r 2 }$ $\boldsymbol { \mathsf { \pmb { \mathsf { \eta } } } } _ { \boldsymbol { \mathsf { k } } + 1 } ^ { r 1 }$ in (40). Therefore, the error equations on the Lie group and Lie algebra can be given as

$$
\boldsymbol {\eta} _ {k + 1} ^ {r 2} = \exp (- \mathbf {A d} _ {\mathbf {x} _ {k + 1} ^ {r 2}} N _ {\Delta k}) \exp (- \mathbf {A d} _ {\mathbf {L} _ {\Delta t}} \mathbf {F} _ {\Delta t} \boldsymbol {x} _ {k} ^ {r 2})\tag{59}
$$

$$
\boldsymbol {x} _ {k + 1} ^ {r 2 -} \approx - \mathbf {A d} _ {\mathbf {L} _ {\Delta t}} \mathbf {F} _ {\Delta t} \boldsymbol {x} _ {k} ^ {r 2} - \mathbf {A d} _ {\boldsymbol {X} _ {k + 1} ^ {r 2}} N _ {\Delta k}\tag{60}
$$

Similarly, for the two left-invariant errors in (42) and (43), the kinematic equations of the prior error states and vectors in discrete time can be formulated as

can be ob-

$$
\begin{array}{r l} \boldsymbol {\eta} _ {k + 1} ^ {l 1} & = \mathbf {X} _ {k + 1} ^ {- 1} \hat {\mathbf {X}} _ {k + 1} \\ & = (\mathbf {L} _ {\Delta t} \boldsymbol {\Phi} _ {\Delta t} (\mathbf {X} _ {k}) \boldsymbol {\Upsilon} _ {\Delta t}) ^ {- 1} \mathbf {L} _ {\Delta t} \boldsymbol {\Phi} _ {\Delta t} (\hat {\mathbf {X}} _ {k}) \hat {\boldsymbol {\Upsilon}} _ {\Delta t} \\ & = \boldsymbol {\Upsilon} _ {\Delta t} ^ {- 1} \boldsymbol {\Phi} _ {\Delta t} (\mathbf {X} _ {k}) ^ {- 1} \mathbf {L} _ {\Delta t} ^ {- 1} \mathbf {L} _ {\Delta t} \boldsymbol {\Phi} _ {\Delta t} (\hat {\mathbf {X}} _ {k}) \hat {\boldsymbol {\Upsilon}} _ {\Delta t} \\ & = \exp (- N _ {\Delta k}) ^ {- 1} (\hat {\boldsymbol {\Upsilon}} _ {\Delta t} ^ {- 1}) \boldsymbol {\Phi} _ {\Delta t} (\mathbf {X} _ {k} ^ {- 1} \hat {\mathbf {X}} _ {k}) \hat {\boldsymbol {\Upsilon}} _ {\Delta t} \\ & = \exp (N _ {\Delta k}) \exp (\mathbf {A d} _ {\hat {\mathbf {r}} _ {\Delta t} ^ {- 1}} \mathbf {F} _ {\Delta t} x _ {k} ^ {l 1}) \end{array}\tag{61}
$$

$$
\boldsymbol {x} _ {k + 1} ^ {l 1 -} \approx \mathbf {A d} _ {\hat {\mathbf {r}} _ {\Delta t} ^ {- 1}} \mathbf {F} _ {\Delta t} \boldsymbol {x} _ {k} ^ {l 1 +} + \boldsymbol {N} _ {\Delta k}\tag{62}
$$

$$
\boldsymbol {\eta} _ {k + 1} ^ {l 2 -} = \exp (- \mathbf {A d} _ {\hat {\mathbf {r}} _ {\Delta t} ^ {- 1}} \mathbf {F} _ {\Delta t} \boldsymbol {x} _ {k} ^ {l 2}) \exp (- N _ {\Delta k})\tag{63}
$$

$$
\boldsymbol {x} _ {k + 1} ^ {l 2 -} \approx - \mathbf {A d} _ {\hat {\mathbf {r}} _ {\Delta t} ^ {- 1}} \mathbf {F} _ {\Delta t} \boldsymbol {x} _ {k} ^ {l 2 +} - \boldsymbol {N} _ {\Delta k}\tag{64}
$$

$$
\mathbf {A d} _ {\hat {\mathbf {Y}} _ {\Delta t} ^ {- 1}} = \left[ \begin{array}{c c c} (\hat {\mathbf {Y}} _ {\Delta t} ^ {R}) ^ {- 1} & \mathbf {0} _ {3 \times 3} & \mathbf {0} _ {3 \times 3} \\ - ((\hat {\mathbf {Y}} _ {\Delta t} ^ {R}) ^ {- 1} \hat {\mathbf {Y}} _ {\Delta t} ^ {v}) ^ {\wedge} (\hat {\mathbf {Y}} _ {\Delta t} ^ {R}) ^ {- 1} & (\hat {\mathbf {Y}} _ {\Delta t} ^ {R}) ^ {- 1} & \mathbf {0} _ {3 \times 3} \\ - ((\hat {\mathbf {Y}} _ {\Delta t} ^ {R}) ^ {- 1} \hat {\mathbf {Y}} _ {\Delta t} ^ {p}) ^ {\wedge} (\hat {\mathbf {Y}} _ {\Delta t} ^ {R}) ^ {- 1} & \mathbf {0} _ {3 \times 3} & (\hat {\mathbf {Y}} _ {\Delta t} ^ {R}) ^ {- 1} \end{array} \right], \quad \hat {\mathbf {Y}} _ {\Delta t} ^ {R}, \quad \hat {\mathbf {Y}} _ {\Delta t} ^ {v}, \text {and} \quad \hat {\mathbf {Y}} _ {\Delta t} ^ {p}
$$

where

tained from (50).

Combined with (46), the discretization of the bias error state can be writen as

$$
\delta \boldsymbol {b} _ {k + 1} = \mathrm{Exp} (\mathbf {F} _ {b} \Delta t) \delta \boldsymbol {b} _ {k} + \boldsymbol {n} _ {b _ {k}}\tag{65}
$$

Let a new symbol  denote the error state containing x and $\xi$ $\delta \boldsymbol { b }$ $\pmb { \xi } = [ \pmb { x } ^ { \mathrm { T } } \quad \pmb { \delta b } ^ { \mathrm { T } } ] ^ { \mathrm { T } } \in \mathbb { R } ^ { 1 5 \times 1 }$ . By combining the error state equation and (65), the propagation equations for the two right-invariant errors $( \begin{array} { l l l } { \pmb { \xi } ^ { r 1 } } & { } & { } \\ { } & { \mathrm { a n d } } \end{array} \xi ^ { r 2 } ,$ ) in the e-frame, which consider accelerometer and gyroscope bias errors, are as follows

$$
\begin{array}{r} \pmb {\xi} _ {k + 1} ^ {r 1 -} \approx \mathbf {A} _ {k} ^ {r 1} \pmb {\xi} _ {k} ^ {r 1} + \mathbf {B} _ {k} ^ {r 1} \pmb {n} _ {k} ^ {\prime} \\ \pmb {\xi} _ {k + 1} ^ {r 2 -} \approx \mathbf {A} _ {k} ^ {r 2} \pmb {\xi} _ {k} ^ {r 2} + \mathbf {B} _ {k} ^ {r 2} \pmb {n} _ {k} ^ {\prime} \end{array}\tag{66}
$$

where $\mathbf { A } \in \mathbb { R } ^ { 1 5 \times 1 5 }$ is the transition matrix for the propagation of adjacent error states, and $\mathbf { B } \in \mathbb { R } ^ { 1 5 \times 1 2 }$ is the transition matrix for the noise components, These matrices can be writen as

$$
\mathbf {A} _ {k} ^ {r 1} = \left[ \begin{array}{c c} \mathbf {A d} _ {\mathbf {L} _ {\Delta t}} \mathbf {F} _ {\Delta t} & \mathbf {A d} _ {\mathbf {X} _ {k + 1} ^ {r 1}} \mathbf {G} _ {k} \\ \mathbf {0} _ {6 \times 9} & \operatorname{Exp} (\mathbf {F} _ {b} \Delta t) \end{array} \right], \quad \mathbf {B} _ {k} ^ {r 1} = \left[ \begin{array}{c c} \mathbf {A d} _ {\mathbf {X} _ {k + 1} ^ {r 1}} \mathbf {G} _ {k} & \mathbf {0} _ {9 \times 6} \\ \mathbf {0} _ {6 \times 6} & \mathbf {I} _ {6 \times 6} \end{array} \right],
$$

$$
\mathbf {A} _ {k} ^ {r 2} = \left[ \begin{array}{c c} - \mathbf {A d} _ {\mathbf {L} _ {\Delta t}} \mathbf {F} _ {\Delta t} & - \mathbf {A d} _ {\mathbf {X} _ {k + 1} ^ {r 2}} \mathbf {G} _ {k} \\ \mathbf {0} _ {6 \times 9} & \operatorname{Exp} (\mathbf {F} _ {b} \Delta t) \end{array} \right],   \mathbf {B} _ {k} ^ {r 2} = \left[ \begin{array}{c c} \mathbf {A d} _ {\mathbf {X} _ {k + 1} ^ {r 2}} \mathbf {G} _ {k} & \mathbf {0} _ {9 \times 6} \\ \mathbf {0} _ {6 \times 6} & \mathbf {I} _ {6 \times 6} \end{array} \right],
$$

$$
\boldsymbol {n} _ {k} ^ {\prime} = \left[ \begin{array}{c c c c} \boldsymbol {n} _ {\omega} ^ {\mathrm{T}} & \boldsymbol {n} _ {a} ^ {\mathrm{T}} & \boldsymbol {n} _ {b _ {\omega}} ^ {\mathrm{T}} & \boldsymbol {n} _ {b _ {a}} ^ {\mathrm{T}} \end{array} \right] ^ {\mathrm{T}}.
$$

It is clear that the error state transition matrix A in (66) is independent of the current trajectory, even when bias is taken into account. The covariances matrix of the propagation can be given as

$$
\mathbf {P} _ {k + 1} ^ {r 1 -} = \mathbf {A} _ {k} ^ {r 1} \mathbf {P} _ {k} ^ {r 1} \mathbf {A} _ {k} ^ {r 1 \mathrm{T}} + \mathbf {B} _ {k} ^ {r} \mathbf {Q} _ {k} ^ {r} \mathbf {B} _ {k} ^ {r \mathrm{T}}
$$

$$
\mathbf {P} _ {k + 1} ^ {r 2 -} = \mathbf {A} _ {k} ^ {r 2} \mathbf {P} _ {k} ^ {r 2} \mathbf {A} _ {k} ^ {r 2 \mathrm{T}} + \mathbf {B} _ {k} ^ {r} \mathbf {Q} _ {k} ^ {r} \mathbf {B} _ {k} ^ {r \mathrm{T}}\tag{67}
$$

where <sup>k</sup> represents the noise covariance matrix. $\mathbf { Q } _ { k }$

Similarly, the left-invariant errors  covariance models can be writen as $\xi ^ { l }$

$$
\pmb {\xi} _ {k + 1} ^ {l 1 -} \approx \mathbf {A} _ {k} ^ {l 1} \pmb {\xi} _ {k} ^ {l 1} + \mathbf {B} _ {k} ^ {l} n _ {k} ^ {\prime}
$$

$$
\pmb {\xi} _ {k + 1} ^ {l 2 -} \approx \mathbf {A} _ {k} ^ {l 2} \pmb {\xi} _ {k} ^ {l 2} + \mathbf {B} _ {k} ^ {l} n _ {k} ^ {\prime}\tag{68}
$$

$$
\mathbf {P} _ {k + 1} ^ {l 1} ^ {-} = \mathbf {A} _ {k} ^ {l 1} \mathbf {P} _ {k} ^ {l 1} \mathbf {A} _ {k} ^ {l 1 \mathrm{T}} + \mathbf {B} _ {k} ^ {l} \mathbf {Q} _ {k} ^ {l} \mathbf {B} _ {k} ^ {l \mathrm{T}}
$$

$$
\mathbf {P} _ {k + 1} ^ {l 2 -} = \mathbf {A} _ {k} ^ {l 2} \mathbf {P} _ {k} ^ {l 2} \mathbf {A} _ {k} ^ {l 2 \mathrm{T}} + \mathbf {B} _ {k} ^ {l} \mathbf {Q} _ {k} ^ {l} \mathbf {B} _ {k} ^ {l \mathrm{T}}\tag{69}
$$

where the state transition matrices and noise transition matrix be expressed as

$$
\mathbf {A} _ {k} ^ {l 1} = \left[ \begin{array}{c c} \mathbf {A d} _ {\hat {\mathbf {r}} _ {\Delta t} ^ {- 1}} \mathbf {F} _ {\Delta t} & \mathbf {G} _ {k} \\ \mathbf {0} _ {6 \times 9} & \mathrm{Exp} (\mathbf {F} _ {b} \Delta t) \end{array} \right], \quad \mathbf {B} _ {k} ^ {l} = \left[ \begin{array}{c c} \mathbf {G} _ {k} & \mathbf {0} _ {9 \times 6} \\ \mathbf {0} _ {6 \times 6} & \mathbf {I} _ {6 \times 6} \end{array} \right],
$$

$$
\mathbf {A} _ {k} ^ {l 2} = \left[ \begin{array}{c c} - \mathbf {A d} _ {\hat {\mathbf {r}} _ {\Delta t} ^ {- 1}} \mathbf {F} _ {\Delta t} & - \mathbf {G} _ {k} \\ \mathbf {0} _ {6 \times 9} & \mathrm{Exp} (\mathbf {F} _ {b} \Delta t) \end{array} \right]
$$

## 3.4. GNSS Observation Update

In the INS-based prediction process, we have defined four error states and their discrete-time error propagation equations. During the GNSS observation process, the rightand left-invariant positional error models can be writen as

$$
\begin{array}{r} \pmb {z} _ {k + 1} = \mathbf {H} _ {k + 1} ^ {r 1} \pmb {\xi} _ {k + 1} ^ {r 1 -} + \pmb {v} _ {o b s e r} ^ {e} \\ \pmb {z} _ {k + 1} = \mathbf {H} _ {k + 1} ^ {r 2} \pmb {\xi} _ {k + 1} ^ {r 2 -} + \pmb {v} _ {o b s e r} ^ {e} \end{array}\tag{70}
$$

$$
\pmb {z} _ {k + 1} = \mathbf {H} _ {k + 1} ^ {l 1} \pmb {\xi} _ {k + 1} ^ {-} + \pmb {v} _ {o b s e r} ^ {e}
$$

$$
\mathbf {z} _ {k + 1} = \mathbf {H} _ {k + 1} ^ {l 2} \boldsymbol {\xi} _ {k + 1} ^ {-} + \boldsymbol {v} _ {o b s e r} ^ {e}\tag{71}
$$

where <sup>H</sup> is the Jacobian matrix associated with the GNSS observation model. $\mathbf { H } ^ { ( \cdot ) }$

$$
\mathbf {H} ^ {l 1} = [ \mathbf {R} _ {b} ^ {e} l ^ {\wedge} \quad \mathbf {0} _ {3 \times 3} \quad - \mathbf {R} _ {b} ^ {e} \mathbf {J} ^ {l 1} \quad \mathbf {0} _ {3 \times 3} \quad \mathbf {0} _ {3 \times 3} ]
$$

$$
\mathbf {H} ^ {l 2} = [ - \mathbf {R} _ {b} ^ {e} \mathbf {l} ^ {\wedge} \quad \mathbf {0} _ {3 \times 3} \quad \mathbf {R} _ {b} ^ {e} \mathbf {J} ^ {l 2} \quad \mathbf {0} _ {3 \times 3} \quad \mathbf {0} _ {3 \times 3} ]\tag{72}
$$

$$
\mathbf {H} ^ {r 1} = [ - (\pmb {p} _ {i b} ^ {e} + \mathbf {R} _ {b} ^ {e} l) ^ {\wedge} \quad \mathbf {0} _ {3 \times 3} \quad \mathbf {J} ^ {r 1} \quad \mathbf {0} _ {3 \times 3} \quad \mathbf {0} _ {3 \times 3} ]
$$

$$
\mathbf {H} ^ {r 2} = [ (\pmb {p} _ {i b} ^ {e} + \mathbf {R} _ {b} ^ {e} \pmb {l}) ^ {\wedge} \quad \mathbf {0} _ {3 \times 3} \quad - \mathbf {J} ^ {r 2} \quad \mathbf {0} _ {3 \times 3} \quad \mathbf {0} _ {3 \times 3} ]\tag{73}
$$

The detailed derivations are presented in Appendix C. l is the lever arm vector. ${ v _ { o b s e r } ^ { e } }$ denotes the observation noise, which satisfies ${ \pmb v } _ { o b s e r } ^ { e } \sim \mathcal { N } ( 0 , { \bf V } )$ . The calculation of the Kalman gain <sup>k</sup> <sup>1+</sup> , error $\mathbf { K } _ { k + 1 } ^ { ( \cdot ) }$ $\xi _ { k + 1 } ^ { ( \cdot ) \ + }$ and covariance $\mathbf { P } _ { k + 1 } ^ { ( \cdot ) \ + }$ at time $t _ { k + 1 }$ can be expressed as

$$
\begin{array}{c} \mathbf {K} _ {k + 1} ^ {(\cdot)} = \mathbf {P} _ {k + 1} ^ {(\cdot) -} \mathbf {H} _ {k + 1} ^ {(\cdot) \text {T}} (\mathbf {H} _ {k + 1} ^ {(\cdot)} \mathbf {P} _ {k + 1} ^ {(\cdot) -} \mathbf {H} _ {k + 1} ^ {(\cdot) \text {T}} + \mathbf {V} _ {k + 1}) ^ {- 1} \\ \boldsymbol {\xi} _ {k + 1} ^ {(  \cdot)   +} = \mathbf {K} _ {k + 1} ^ {(\cdot)} \boldsymbol {\mathsf {z}} _ {k + 1} ^ {(\cdot)} \\ \mathbf {P} _ {k + 1} ^ {(  \cdot)   +} = (\mathbf {I} - \mathbf {K} _ {k + 1} ^ {(\cdot)} \mathbf {H} _ {k + 1} ^ {(\cdot)}) \mathbf {P} _ {k + 1} ^ {(  \cdot)   -} \end{array}\tag{74}
$$

where superscript $( \cdot )$ represents the diferent error models from (40)–(43) and $" + \prime \prime$ denotes the posterior state. It should be noted that the error vector $\xi _ { k + 1 } ^ { ( \cdot ) \ + }$ includes error state and biases $\pmb { \xi } = [ \pmb { x } ^ { \mathrm { T } } \quad \pmb { \delta b } ^ { \mathrm { T } } ] ^ { \mathrm { T } }$ . We can restore the true state in the e-frame, by the exponential mapping from the Lie group to Lie algebra, and the right-invariant errors are defined as

$$
\mathbf {X} _ {k + 1} ^ {r 1} = \boldsymbol {\eta} _ {k + 1} ^ {r 1} \hat {\mathbf {X}} _ {k + 1} ^ {r 1} = \exp (\boldsymbol {x} _ {k + 1} ^ {r 1 +}) ^ {- 1} \hat {\mathbf {X}} _ {k + 1} ^ {r 1}\tag{75}
$$

$$
\mathbf {X} _ {k + 1} ^ {r 2} = \hat {\mathbf {X}} _ {k + 1} ^ {r 2} \boldsymbol {\eta} _ {k + 1} ^ {r 2 - 1} = \exp (\boldsymbol {x} _ {k + 1} ^ {r 2 +}) \hat {\mathbf {X}} _ {k + 1} ^ {r 2}
$$

The definitions of two left-invariant errors are given by

$$
\begin{array}{l} \mathbf {X} _ {k + 1} ^ {l 1} = \hat {\mathbf {X}} _ {k + 1} ^ {l 1} \boldsymbol {\eta} _ {k + 1} ^ {l 1} = \hat {\mathbf {X}} _ {k + 1} ^ {l 1} \exp (x _ {k + 1} ^ {l 1 +}) ^ {- 1} \\ \mathbf {X} _ {k + 1} ^ {l 2} = \boldsymbol {\eta} _ {k + 1} ^ {l 2} \hat {\mathbf {X}} _ {k + 1} ^ {l 2 - 1} = \hat {\mathbf {X}} _ {k + 1} ^ {l 2} \exp (x _ {k + 1} ^ {l 2 +}) \end{array}\tag{76}
$$

where

$$
\exp (\boldsymbol {x} ^ {(\cdot)}) = \left[ \begin{array}{c c c} \exp (\boldsymbol {x} _ {\varphi}) & \mathbf {J} ^ {(\cdot)} \boldsymbol {x} _ {v} & \mathbf {J} ^ {(\cdot)} \boldsymbol {x} _ {p} \\ \mathbf {0} _ {1 \times 3} & 1 & 0 \\ \mathbf {0} _ {1 \times 3} & 0 & 1 \end{array} \right]\tag{77}
$$

where $\exp ( { \boldsymbol { x } } _ { \varphi } ) = \Gamma _ { 0 } ( { \boldsymbol { x } } _ { \varphi } )$ is the atitude error matrix in the Lie group; v <sup>Jx</sup> and p  rep- $\mathbf { J } \pmb { x } _ { v }$ $\mathbf { J } { \boldsymbol { x } } _ { p }$ resent the velocity error and position error, respectively; and $\mathbf { J } ^ { ( \cdot ) }$ is the left Jacobian matrix, and $\mathbf { J } ^ { ( \cdot ) } = \Gamma _ { 1 } ( \pmb { x } _ { \varphi } )$

## 3.5. The New Errors Based on Multiple Invariant Errors

Based on the definitions of the four invariant error matrices in (40)–(43), multiplying $\boldsymbol { \mathsf { \pmb { \eta } } } _ { t } ^ { r 1 } \mathbin { \mathrm {  ~ \ b y ~ } } \boldsymbol { \mathsf { \pmb { \eta } } } _ { t } ^ { r 2 } \mathrm {  ~ \ a n d ~ } \boldsymbol { \mathsf { \pmb { \eta } } } _ { t } ^ { l 1 } \mathbin { \mathrm {  ~ \ b y ~ } } \boldsymbol { \mathsf { \pmb { \eta } } } _ { t } ^ { l 2 }$ yields the following expression

$$
\begin{array}{r} \pmb {\eta} _ {k} ^ {l 1} \pmb {\eta} _ {k} ^ {l 2} \triangleq \mathbf {I} _ {5 \times 5} \\ \pmb {\eta} _ {k} ^ {r 1} \pmb {\eta} _ {k} ^ {r 2} \triangleq \mathbf {I} _ {5 \times 5} \end{array}\tag{78}
$$

According the mapping relationship between Lie groups and Lie algebras given in (51), (78) can be mapped to the Lie algebra, resulting in an equivalent error vector identity as follows

$$
\begin{array}{r} \pmb {x} _ {k + 1} ^ {l 1} \triangleq - \pmb {x} _ {k + 1} ^ {l 2} \\ \pmb {x} _ {k + 1} ^ {r 1} \triangleq - \pmb {x} _ {k + 1} ^ {r 2} \end{array}\tag{79}
$$

However, in practical operation, numerical errors, linearization and discretization approximations, and model mismatch prevent the theoretical symmetry (79) from holding exactly. To exploit this theoretical constraint and thus enhance estimation accuracy, we propose a lightweight post-processing correction strategy. Following each measurement update, we correct the error vector to mitigate random deviations induced by numerical errors while preserving the core filter structure. Since it is impractical to determine a priori which of $\pmb { x } _ { k + 1 } ^ { 1 }$ and $\pmb { x } _ { k + 1 } ^ { 2 }$ is superior, we adopt a symmetric fusion strategy: for the leftinvariant error, the corrected vector +1k <sup>x</sup> retains the sign of +1k <sup>x</sup> while its magnitude is $\pmb { x } _ { k + 1 } ^ { l 1 * }$ $\pmb { x } _ { k + 1 } ^ { l 1 }$ set to the weighted average of their absolute values,

$$
\boldsymbol {x} _ {k + 1} ^ {l 1} ^ {*} = \operatorname{sign} \left(\boldsymbol {x} _ {k + 1} ^ {l 1} ^ {+}\right) \odot \left(w \left| \boldsymbol {x} _ {k + 1} ^ {l 1} ^ {+} \right| + (1 - w) \left| \boldsymbol {x} _ {k + 1} ^ {l 2} ^ {+} \right|\right)\tag{80}
$$

where $s i g n ( \cdot )$ denotes the sign function, $\odot$ represents the element-wise multiplication operator, and <sup>w</sup> is a user-defined weight coeficient (here set to $w$ $w _ { \mathrm { \it ~  ~ } } = 0 . 5 )$ . As the correction magnitude is typically much smaller than the uncertainty range encoded in the covariance matrix P (experimentally, $\left\| \pmb { x } _ { k + 1 } ^ { l 1 } \right\| \middle / \mathbf { P } _ { k + 1 }$ averages about 0.08), its impact on covariance consistency is negligible.

The core significance of this method is that it converts the theoretical symmetry of left-invariant errors (79) into a lightweight, practical correction tool. By fusing the magnitude information of both errors, we efectively curb random deviations without modifying the core filter structure, leading to improved long-term stability and accuracy. Experiments show that this lightweight post-processing step markedly reduces the root-meansquare error of state estimation, underscoring the practical benefits of exploiting geometric constraints to improve filter robustness.

The proposed correction, as a lightweight post-processing step, enhances practicality while introducing inherent trade-ofs between optimality and computational eficiency. First, algorithmic simplicity and real-time constraints preclude a concurrent covariance update, causing a deliberate departure from the classical Kalman optimality framework and constituting a moderate compromise on theoretical rigor. Second, the method presupposes predominantly stochastic error deviations; performance degrades under pronounced systematic biases or severe dynamic mismatches. These limitations define the method’s applicability and motivate future work on approximate covariance updates or adaptive fusion weights.

Nevertheless, in the context of INS/GNSS integrated estimation under large misalignment angles, the corrected error vector remains subject to the inherent limitations of left-invariant errors. Specifically, the observation matrix H<sup>l</sup> defined in (72) contains the atitude matrix <sup>b</sup> . This introduces atitude-induced errors into the position and velocity $\mathbf { R } _ { b } ^ { e }$ estimates and consequently degrades the estimation performance under large misalignment conditions. Although the multiplicative right-IEKF generally exhibits lower overall accuracy and robustness compared to the multiplicative left-IEKF [40], under large misalignment angles, it demonstrates faster convergence speed and higher estimation accuracy for position and velocity during the initial stage. As the matrix H<sup>r</sup> (73) does not contain the atitude matrix, thereby mitigating the influence of atitude errors caused by large misalignments. Based on this analysis, this study further combines the advantages of the corrected left-invariant error $\pmb { x } _ { k + 1 } ^ { l 1 } \breve { \ast }$ and the right-invariant error +1k $\pmb { x } _ { k + 1 } ^ { r 1 }$ to propose a multiplicative federated IEKF (MF-IEKF). The new method first employs the multiplicative right1- IEKF in the initial phase and then switches to the multiplicative corrected left-IEKF (MCL-IEKF) to continue the task. The federated invariant error can be writen as

$$
\boldsymbol {x} _ {k + 1} ^ {F} (i) = \left\{ \begin{array}{l l} \boldsymbol {x} _ {k + 1} ^ {r 1} (i) & t _ {k + 1} <   t _ {s w i t c h} \\ \boldsymbol {x} _ {k + 1} ^ {l 1 *} (i) & t _ {k + 1} > t _ {s w i t c h} \end{array} \right.\tag{81}
$$

where $t _ { s w i t c h }$ denotes the switching time, also called the degradation time of MR1-IEKF, and is set to 10 s in this study. Although MR1-IEKF excels at the outset, ML-IEKF achieves superior performance over the entire duration. Consequently, the optimal switching instant coincides with the crossover point where both estimators exhibit equivalent accu-

racy. The choice of $t _ { _ { s w i t c h } }$ in our experiments resulted from a comprehensive trade-of and is not necessarily the global optimum; this choice does not undermine the demonstrated superiority of MF-IEKF.

Finally, the MF-IEKF for the INS/GNSS can be expressed as Algorithm 1.

<div class="mineru-algorithm" style="white-space: pre-wrap; font-family:monospace;">
Algorithm 1: MF-IEKF for the INS/GNSS

Input:  $\mathbf{X}_{0}\left(\mathbf{R}_{b,0}^{e},\mathbf{v}_{ib,0}^{e},\mathbf{p}_{ib,0}^{e}\right)$ ,  $(\mathbf{b}_{\omega,0},\mathbf{b}_{a,0})$ ,  $(\tilde{\omega}_{ib,k}^{b},\tilde{\mathbf{a}}_{ib,k}^{b})$ ,  $P_{0}$ 

Output:  $\mathbf{X}_{k}\left(\mathbf{R}_{b,k}^{e},\mathbf{v}_{ib,k}^{e},\mathbf{p}_{ib,k}^{e}\right)$ .

1: 1. Initialization:

2:  $\mathbf{X}_{0}=\mathrm{E}\left[\left(\mathbf{X}_{0}\right)\right]$ ,  $P_{0}=\mathrm{E}\left[\left(\mathbf{X}_{0}-\hat{\mathbf{X}}_{0}\right)\left(\mathbf{X}_{0}-\hat{\mathbf{X}}_{0}\right)^{\mathrm{T}}\right]$ 

3: 2. State prediction at  $t_{k+1}$ :

4: Priori invariant error and covariance propagation equation:

5:  $\xi_{k+1}^{(-)}\approx A_{k}^{(-)}\xi_{k}^{(-)}+B_{k}^{(-)}n_{k}^{\prime}$ 

6:  $P_{k+1}^{(-)}=A_{k}^{(-)}P_{k}^{(-)}A_{k}^{(-)T}+B_{k}^{(-)}Q_{k}^{(-)}B_{k}^{(-)T}$ 

7:  $A_{k+1}^{(-)}$ ,  $B_{k+1}^{(-)}$  matrices can be refer to Equations (66) and (68).

8: 3. State update at  $t_{k+1}$ :

9: While GNSS observation arrive:

10: Kalman gain, posteriori invariant error and covariance calculation:

11:  $K_{k+1}^{(-)}=P_{k+1}^{-}H_{k+1}^{(-)T}(H_{k+1}^{(-)}P_{k+1}^{(-)}H_{k+1}^{(-)T}+V_{k+1})^{-1}$ 

12:  $\xi_{k+1}^{(-)}=K_{k+1}^{(-)}z_{k+1}^{(-)}$ 

13:  $P_{k+1}^{(-)}=(I-K_{k+1}^{(-)}H_{k+1}^{(-)})P_{k+1}^{(-)}$ 

14: If  $t_{k+1}&lt;t_{switch'}$ , process sub-filter MR1-IEKF, the error calculation:

15:  $\xi_{k+1}^{+}=\xi_{k+1}^{r1+}$ 

16: else, process sub-filter MCL-IEKF, the error calculation:

17:  $\xi_{k+1}^{+}=\xi_{k+1}^{l1*+}=\left[x_{k+1}^{l1*}\delta b_{k+1}^{T}\right]$ 

18:  $x_{k}^{l1*}$  can be refer to Equation (80).

19: 4. Global state update:

20: If  $t_{k+1}&lt;t_{switch'}$ ,  $X_{k+1}=X_{k+1}^{r1}=\exp(x_{k+1}^{r1*})^{-1}\hat{X}_{k+1}^{r1}$ 

21: else  $X_{k+1}=X_{k+1}^{l1}=\hat{X}_{k+1}^{l1}\exp(x_{k+1}^{l1*})^{-1}$ 

22: return  $X_{k+1}, P_{k+1}^{(-)}$ 

23: End while
</div>

## 4. Experiments

This section evaluates the accuracy of the algorithms based on diferent error models through INS/GNSS simulation experiments and open source dataset experiments. A comparison was conducted among the following five algorithms:

The method based on the right- invariant error defined in (40), denoted as MR1-IEKF.

The method based on the left-invariant error defined in (58) is denoted as ML1-IEKF, which corresponds to the framework presented in [42].

The method based on the left-invariant error defined in (62), denoted as ML2-IEKF.

The method based on the corrected left-invariant error defined in (80), denoted as MCL-IEKF.

The method federated invariant error defined in (81), denoted as MF-IEKF. Note that the MCL-IEKF is a sub-filter of the MF-IEKF.

## 4.1. Numerical Simulations

IMU and GNSS data used in the simulations were generated by the open-source platform gnss-ins-sim [45]. The toolbox supports high-, medium-, and low-grade IMU configurations, together with a low-precision GNSS receiver model. To balance the validity of algorithm performance evaluation with the reasonableness of simulation conditions, a medium-grade IMU model was selected for this study. Its parameters are specified as follows: a gyroscope bias of $3 . 5 ^ { \circ } / \mathrm { h }$ and an angular random walk of $0 . 2 5 ^ { \circ / \sqrt { \mathrm { h } } }$ ; an accelerometer bias of 5 mg and a velocity random walk of $_ 3 \ \mathrm { m g / \sqrt { H z } _ { \cdot } A }$ low-precision GNSS receiver was simulated with positional noise standard deviations of $\sigma = ( 5 , 5 , 7 )$ m. The INS and GNSS output rates are 100 Hz and 10 Hz, respectively. The initial plat-form misalignment angles were set to [30°, 30°, 10°]. Figure 2 illustrates the reference trajectory, which includes acceleration, constant-speed cruising, and turning segments to emulate realistic vehicle dynamics.

To obtain statistically meaningful results, 500 independent Monte Carlo trials were conducted. Each trial lasted 105 s, suficient to capture the full transition from large initial misalignment to filter convergence. All estimators shared identical initial conditions: a zero error-state vector and an identity covariance matrix.

![](images/22848d2e6dd82e8eff991fb9a75c6e0cc92400c98b4a7f58c7433a7f4d4dddef.jpg)  
Figure 2. Reference trajectory for Monte Carlo experiments.

To quantify the estimation accuracy of the algorithms in position, velocity, and atitude, the root mean square error (RMSE) was calculated, respectively, using the following formula

$$
\operatorname{RMSE} (k) = \sqrt {\frac {1}{n} \sum_ {i = 1} ^ {n} \left(y _ {k , i} - \hat {y} _ {k , i}\right) ^ {2}}\tag{82}
$$

$y _ { k , i } \quad \hat { y } _ { k , i }$ represent the reference value and estimated value in the i-th simulation at k-th time step, respectively.

Figures 3–5 present the three-axis RMSE curves for atitude, velocity, and position, respectively, obtained from the five algorithms over 500 Monte Carlo runs. As shown in Figure 3, the atitude RMSE distributions exhibit minor variations across the algorithms, requiring further quantitative analysis. Figure 4 reveals that the velocity RMSE curves of the two ML1- and ML2-IEKF are significantly higher overall than that of MCL-IEKF and MF-IEKF. Magnified views of the y- and z-axes clearly indicate that the RMSE of MCL-IEKF, which combines two ML-IEKFs, is noticeably higher than that of MF-IEKF, which incorporates MR1-IEKF. The position RMSE distributions in Figure 5 further support these findings. In the enlarged views of the x- and y-axes, the RMSE of MCL-IEKF remains markedly higher than that of MF-IEKF. Moreover, the magnified sections in Figures 4 and

5 demonstrate that both MR1-IEKF and MF-IEKF achieve significantly faster convergence in velocity and position estimation compared to the ML-IEKF-based methods, which aligns with the theoretical analysis in Section 3 of the paper.

![](images/1d5928a782d8df8d0ed6c3207211c4dc849263ab7ac8a6cf8f5c1c671eeaea5d.jpg)

![](images/6230b37c9f9c3cbd4a684a02af3a4b27cb26a2b71addfcb68bb4897d312a7c23.jpg)

![](images/6d088f1ffe8daba935c4d25df5d5cd52b999b1a484abd21672dd4f0459bcd55d.jpg)  
Figure 3. The RMSE of atitude estimation.

![](images/c4564f6671296bdbd74c26cef86f14eee128b5f2b67c47c78b02209f814df99d.jpg)

![](images/408c16ce56eb6a3cf9492394b0bff93e7247a01eea48ea83aebca99b721b5278.jpg)

![](images/6bf3e8b66ce3dd2e2406f2a65d0a6d62da51e38295e67a86f3f2dabc6a70b545.jpg)  
Figure 4. The RMSE of velocity estimation.

![](images/3bc91ed31501cbda418492af4eb0600973ed97d7561f144ccb2b02872519c97b.jpg)

![](images/4ede5db10a08f6f24a198afd9278d35ea3655609265813f54973076de34c1192.jpg)

![](images/791a4f939bb5811c94b1f431347c1718018c0209958e8e9ae4f21ee2cc23102d.jpg)  
Figure 5. The RMSE of position estimation.

To facilitate a direct quantification and comparison of algorithm performance, the mean of RMSEs (MRMSE) is also computed as follows

$$
\mathrm{MRMSE} = \sqrt {\frac {1}{n} \sum_ {k = 1} ^ {n} \left(\frac {1}{3} \sum_ {i = 1} ^ {3} \left(\mathrm{RMSE} _ {i} (k)\right) ^ {2}\right)}\tag{83}
$$

where n denotes the total time step, and i = 1, 2, 3 corresponding to the X-Y-Z axes, respectively.

The MRMSE values in Table 1 quantitatively confirm this qualitative assessment. The proposed MF-IEKF achieves superior accuracy across all metrics. As a sub-filter, the MCL-IEKF itself demonstrates significant improvements over the baseline ML1-IEKF, with MRMSE reductions of 13.15% in atitude, 62.01% in velocity, and 13.28% in position. Furthermore, the complete MF-IEKF framework, which incorporates MR1-IEKF, achieves additional performance gains, ultimately reducing atitude, velocity, and position errors by 13.86%, 63.43%, and 15.28%, respectively, compared to the ML1-IEKF baseline.

Table 1. The MRMSEs of state in the Monte Carlo experiments.

<table><tr><td></td><td> $MR_{1}-IEKF$ </td><td> $ML_{1}-IEKF$ </td><td> $ML_{2}-IEKF$ </td><td>MCL-IEKF</td><td>MF-IEKF</td></tr><tr><td>Attitude (rad)</td><td>0.077</td><td>0.076</td><td>0.074</td><td>0.066</td><td>0.065</td></tr><tr><td>velocity(m/s)</td><td>2.012</td><td>4.186</td><td>4.660</td><td>1.590</td><td>1.530</td></tr><tr><td>position (m)</td><td>7.67</td><td>5.809</td><td>5.911</td><td>5.037</td><td>4.921</td></tr></table>

## 4.2. Open Source Dataset Experiments

To validate the proposed algorithm under real-world conditions, the widely used KITTI odometry dataset [46] was employed. The data were collected by a vehicle equipped with an OXTS RT3003 high-precision integrated navigation system, which provides time-synchronized IMU (100 Hz) and GNSS (10 Hz) measurements. The associated sensor noise parameters are as follows: gyroscope bias of $1 . 5 ^ { \circ / \mathrm { h } }$ and angular random walk of $0 . 1 5 ^ { \circ / \sqrt { \mathrm { h } } }$ ; accelerometer bias of 0.1 mg and velocity random walk of 0.05 $\mathrm { m g } / \sqrt \mathrm { H z }$ ; GNSS position measurement noise standard deviations of (0.02, 0.02, 0.05) m.

Sequence 2011\_10\_03\_drive\_0027 (duration ≈ 8 min) was selected for evaluation. This sequence covers typical urban, residential, and suburban driving environments, presenting rich motion dynamics and high representativeness; its trajectory is illustrated in Figure 6. To ensure a fair comparison, all filters were initialized under identical conditions: a zero error-state vector and an identity covariance matrix. The state update cycle was strictly synchronized with the timestamps of the dataset.

$$
\operatorname{AE} (k) = \left| y _ {k} - \hat {y} _ {k} \right|\tag{84}
$$

![](images/e3fe429f7530ed2129d02e3ad8c3a649f8c00cff688a9219d7b19743c46f8198.jpg)  
Figure 6. Ground truth trajectory for open source dataset experiments.

Figure 6 depicts the ground truth trajectory used in the dataset-based experiments. Figures $7 { - } 9$ present the absolute error (AE) distribution curves, as defined in (84), for atitude, velocity, and position estimates across the evaluated algorithms. As observed in Figure 7, the MF-IEKF achieves a notably lower AE in pitch angle compared to the other methods. The magnified view of velocity estimates in Figure 8 further demonstrates that the MF-IEKF maintains the lowest AE throughout the entire time period. Similarly, the enlarged section of position estimates in Figure 9 reveals a consistent trend, confirming that MF-IEKF has a faster convergence speed in the initial stage compared to ML-IEKF.

It should be noted that the sensors used in this dataset experiment exhibit high precision, which enables both $_ \mathrm { M L 1 - I E K F }$ and ${ \mathrm { M L } } _ { 2 } { \mathrm { - I E K F } }$ to also achieve reasonably accurate estimates. Under these conditions, the performance advantage of MCL-IEKF and MF-IEKF remains relatively limited outside the regions highlighted in the magnified views. Therefore, to enable an objective assessment of algorithm performance, a quantitative comparison based on the RMSE is necessary. The RMSE is calculated using the following formula

$$
\mathrm{RMSE} = \sqrt {\frac {1}{3} \sum_ {i = 1} ^ {3} \left(\frac {1}{n} \sum_ {k = 1} ^ {n} (y _ {i , k} - \hat {y} _ {i , k}) ^ {2}\right)}\tag{85}
$$

where n denotes the total time step, $y _ { k }$ and $\hat { y } _ { k }$ represent the reference value and estimated value at k-th time step, and i = 1, 2, 3 corresponding to the X-Y-Z axes, respectively.

![](images/e3bb6513dd376d8cf2e864a3df1a3a7429b07ed554bf6d3a37a486a3fddd66af.jpg)

![](images/5b8b833c4577de5abd1cc491db628e159a8e39369c863da2d800f812f4cce05e.jpg)

![](images/46967cf777f773c82591bed02c20ebc32568149a82b3d563ed6be77ecdd4a650.jpg)  
Figure 7. The absolute error of atitude estimation.

3D Velocity Errors  
![](images/80351337f375b2337fc38a6ae4c52a9148bb44a2e082a22728dcded7f64027f5.jpg)

![](images/2f548fe2422d29cd7a888c5d41f9e2993bd8dbde19e37e177cc3dfb6dd55c7d0.jpg)

![](images/b29d54f1bce5ad3d823dcb66b4397fbbd0c6204f3466e81cf8b36113c5f73b20.jpg)  
Figure 8. The absolute error of velocity estimation.

3D Position Errors  
![](images/1fdf3930580f2e0efb25335ecf462623e4883a8ca9cd1b8e549d6a73a8b5f0de.jpg)

![](images/64dec0f4ec31918cfaf7b974a5bdf3c1c432690da47fd92513de907fb268d878.jpg)

![](images/3bee1b9c1f7208af17037917ab8c1b44d4f14d32428f5bc5e3394cdeb8b3dd6e.jpg)  
Figure 9. The absolute error of position estimation.

The RMSE results are summarized in Table 2. Compared to the baseline ML1-IEKF, the MCL-IEKF sub-filter achieves reductions in RMSE of 5.26% in atitude, 5.81% in velocity, and 5.98% in position. Building upon MCL-IEKF, the MF-IEKF delivers further enhancements, reducing the RMSE by an additional 3% in velocity and 31.63% in position relative to MCL-IEKF. This improvement is atributed to the incorporation of MR1-IEKF results during the initial phase. Notably, the MR1-IEKF exhibits markedly greater errors in velocity and position estimation. This stems from its inherent alignment with a local coordinate frame, in contrast to the ML-IEKF, which is naturally suited to the global frame of GNSS observations [42]. This performance disparity is further amplified under highdynamic conditions. Overall, compared to ML1-IEKF, the MF-IEKF achieves total error reductions of 5.26% in atitude, 8.72% in velocity, and 35.69% in position. These results conclusively demonstrate the efectiveness of the proposed federated framework.

Table 2. The RMSEs of state in the open source dataset experiments.

<table><tr><td></td><td> $MR_{1}-IEKF$ </td><td> $ML_{1}-IEKF$ </td><td> $ML_{2}-IEKF$ </td><td>MCL-IEKF</td><td>MF-IEKF</td></tr><tr><td>Attitude (rad)</td><td>0.026</td><td>0.019</td><td>0.019</td><td>0.018</td><td>0.018</td></tr><tr><td>velocity(m/s)</td><td>9.380</td><td>0.344</td><td>0.342</td><td>0.324</td><td>0.314</td></tr><tr><td>position (m)</td><td>4.865</td><td>1.003</td><td>1.069</td><td>0.943</td><td>0.642</td></tr></table>

## 4.3. Summary and Analysis of Experiment Results

This section presents a comprehensive evaluation of five IEKF-based algorithms through rigorous numerical simulations and real-world dataset validation. The proposed MF-IEKF demonstrates superior performance in both controlled Monte Carlo simulations and KITTI dataset experiments. In simulated environments with large initial misalignments, the framework achieves statistically significant improvements in velocity and position estimation accuracy and convergence speed, atributable to its synergistic use of invariant filters. These quantitative advantages are further validated in real-world scenarios using the KITTI dataset, where MF-IEKF maintains excellent performance even under high-precision sensor conditions. This consistency verifies the algorithm’s efectiveness across diferent platforms and noise characteristics. The experimental results collectively confirm that the federated architecture, which strategically leverages both left- and rightinvariant error models, efectively enhances the precision and reliability of INS/GNSS integrated navigation in challenging scenarios.

## 5. Conclusions

An MF-IEKF framework is proposed for INS/GNSS integration. It enhances estimation accuracy compared to baseline ML1-IEKF methods, demonstrating faster convergence in velocity and position estimation, particularly under large misalignment angles. The framework is built upon multiplicative pre-integration theory and introduces two core contributions: First, it operates ML1-IEKF and ML2-IEKF in parallel, leveraging the inverse relationship between their left-invariant error vectors mapped onto the Lie algebra to perform collaborative error correction. This yields the MCF-IEKF sub-filter, which improves steady-state accuracy. Second, during the initial phase, it incorporates an MR1-IEKF subfilter to efectively suppress velocity and position errors induced by large misalignment angles, significantly enhancing both the convergence speed and accuracy of velocity and position estimates throughout the initial alignment process. Extensive Monte Carlo simulations and experiments on public datasets demonstrate that compared to the conventional ML1-IEKF, our proposed MF-IEKF framework achieves accuracy improvements of at least 5.26% in atitude, 8.72% in velocity, and 15.28% in position estimation, confirming its efectiveness and superiority in complex dynamic scenarios, particularly under large initial misalignment conditions. This research provides a viable solution for high-performance inertial-integrated navigation, and future work will focus on deeper integration of this framework with multi-source sensors (such as vision and LiDAR) and further theoretical analysis of the convergence and stability of the federated architecture.

Author Contributions: Conceptualization, L.Z.; methodology, L.Z.; software, L.Z. and P.Y.; validation, L.Z. and $\mathrm { T . C . } ;$ formal analysis, T.C.; investigation, X.L.; resources, X.L.; data curation, Y.L.; writing—original draft preparation, $\mathrm { L . } Z . ;$ writing—review and editing, $\mathrm { P . Y . } ;$ visualization, $\mathrm { Y . L . } ;$ supervision, $\scriptstyle \mathrm { { L . Z . } }$ and T.C.; project administration, T.C. All authors have read and agreed to the published version of the manuscript.

Funding: This work was supported by the National Natural Science Foundation of China (grant No. 51978075), General Project of Shaanxi Provincial Natural Science Basic Research Program (grant No. 2025JC-YBMS-458) and Research Project of Sichuan Expressway Construction and Development Group Co., Ltd. (grant No. 2024-cg-ky-22).

Institutional Review Board Statement: Not applicable.

Informed Consent Statement: Not applicable.

Data Availability Statement: The raw data supporting the conclusions of this article will be made available by the authors upon request.

Conflicts of Interest: Author Yang Luo was employed by the company Sichuan Chengneiyu Expressway $\mathrm { C o . , }$ Ltd. The remaining authors declare that the research was conducted in the absence of any commercial or financial relationships that could be construed as a potential conflict of interest.

## Appendix A

Between the discrete time instants k<sup>t</sup> and $t _ { k }$ $t _ { k + 1 }$ , both the Earth’s angular velocity expressed <sup>ie</sup> in the e-frame and the IMU measurements are modeled as constant. The so- $\omega _ { i e } ^ { e }$ lution of the diferential equations in (20) and (22) yields the discretized global and local increments. For the global increment, the recursive equations of global atitude, velocity and position can be obtained as follows

$$
\begin{array}{r l} & {\mathbf {L} _ {t _ {k + 1}} ^ {R} = \int_ {t _ {k}} ^ {t _ {k + 1}} \left(- \pmb {\omega} _ {i e} ^ {e}\right) ^ {\wedge} d \tau \mathbf {L} _ {t _ {k}} ^ {R} = \exp (- \pmb {\omega} _ {i e} ^ {e} \Delta t) \mathbf {L} _ {t _ {k}} ^ {R}} \\ & {\mathbf {L} _ {t _ {k + 1}} ^ {v} = \exp (- \pmb {\omega} _ {i e} ^ {e} \Delta t) \mathbf {L} _ {t _ {k}} ^ {v} + \int_ {0} ^ {\Delta t} \exp \left(- \pmb {\omega} _ {i e} ^ {e} (\tau)\right) G _ {i b} ^ {e} d \tau} \\ & {\mathbf {L} _ {t _ {k + 1}} ^ {p} = \exp (- \pmb {\omega} _ {i e} ^ {e} \Delta t) (\mathbf {L} _ {t _ {k}} ^ {p} + \Delta t \mathbf {L} _ {t _ {k}} ^ {v}) + \int_ {0} ^ {\Delta t} \tau \exp (- \pmb {\omega} _ {i e} ^ {e} \tau) G _ {i b} ^ {e} d \tau} \end{array}\tag{A1}
$$

where the time step is $\Delta t = t _ { { \scriptscriptstyle k } + 1 } - t _ { { \scriptscriptstyle k } }$

Over the time step $\Delta t ,$ the global increment can be writen as

$$
\begin{array}{l} \mathbf {L} _ {\Delta t} ^ {R} = \int_ {t _ {k}} ^ {t _ {k + 1}} \left(- \boldsymbol {\omega} _ {i e} ^ {e}\right) ^ {\wedge} d \tau = \exp (- \boldsymbol {\omega} _ {i e} ^ {e} \Delta t) = \boldsymbol {\Gamma} _ {0} \left(- \boldsymbol {\omega} _ {i e} ^ {e} \Delta t\right) \\ \mathbf {L} _ {\Delta t} ^ {v} = \int_ {0} ^ {\Delta t} \exp \left(- \boldsymbol {\omega} _ {i e} ^ {e} (\tau)\right) \boldsymbol {G} _ {i b} ^ {e} d \tau = \boldsymbol {\Gamma} _ {1} \left(- \boldsymbol {\omega} _ {i e} ^ {e} \Delta t\right) \Delta t \boldsymbol {G} _ {i b} ^ {e} \\ \mathbf {L} _ {\Delta t} ^ {p} = \int_ {t _ {k}} ^ {t _ {k + 1}} (t _ {k} - \tau) \exp (- \boldsymbol {\omega} _ {i e} ^ {e} (t _ {k} - \tau)) \boldsymbol {G} _ {i b} ^ {e} d \tau = \boldsymbol {\Gamma} _ {2} \left(- \boldsymbol {\omega} _ {i e} ^ {e} \Delta t\right) \Delta t ^ {2} \boldsymbol {G} _ {i b} ^ {e} \end{array}\tag{A2}
$$

where $\Gamma _ { { \scriptscriptstyle m } } ( \varphi )$ was defined in (30).

For the local increment, the recursive equations of local atitude, velocity and position can be obtained as follows

$$
\begin{array}{r l} \mathbf {Y} _ {t _ {k + 1}} ^ {R} & = \mathbf {Y} _ {t _ {k}} ^ {R} \exp (\boldsymbol {\omega} _ {i b} ^ {b} \Delta t) \\ \mathbf {Y} _ {t _ {k + 1}} ^ {v} & = \mathbf {Y} _ {t _ {k}} ^ {v} + \int_ {t _ {k}} ^ {t _ {k + 1}} \exp (\boldsymbol {\omega} _ {i b} ^ {b} \tau) \boldsymbol {a} _ {i b} ^ {b} d \tau \\ & = \mathbf {Y} _ {t _ {k}} ^ {v} + \mathbf {Y} _ {t _ {k}} ^ {R} \int_ {0} ^ {\Delta t} \exp (\boldsymbol {\omega} _ {i b} ^ {b} \tau) d \tau \boldsymbol {a} _ {i b} ^ {b} \\ \mathbf {Y} _ {k + 1} ^ {p} & = \mathbf {Y} _ {k} ^ {p} + \Delta t \mathbf {Y} _ {k} ^ {v} + \mathbf {Y} _ {t _ {k}} ^ {R} \int_ {0} ^ {\Delta t} \tau \exp (\boldsymbol {\omega} _ {i b} ^ {b} \tau) \boldsymbol {a} _ {i b} ^ {b} d \tau \end{array}\tag{A3}
$$

The local increment of discrete time can be writen as

$$
\begin{array}{l} \boldsymbol {\Upsilon} _ {\Delta t} ^ {R} = \exp (\boldsymbol {\omega} _ {i b} ^ {b} \Delta t) = \boldsymbol {\Gamma} _ {0} (\boldsymbol {\omega} _ {i b} ^ {b} \Delta t) \\ \boldsymbol {\Upsilon} _ {\Delta t} ^ {v} = \int_ {0} ^ {\Delta t} \exp (\boldsymbol {\omega} _ {i b} ^ {b} \tau) d \tau \boldsymbol {a} _ {i b} ^ {b} = \boldsymbol {\Gamma} _ {1} (\boldsymbol {\omega} _ {i b} ^ {b} \Delta t) \Delta t \boldsymbol {a} _ {i b} ^ {b} \\ \boldsymbol {\Upsilon} _ {\Delta t} ^ {p} = \int_ {0} ^ {\Delta t} \tau \exp (\boldsymbol {\omega} _ {i b} ^ {b} \tau) \boldsymbol {a} _ {i b} ^ {b} d \tau = \boldsymbol {\Gamma} _ {2} (\boldsymbol {\omega} _ {i b} ^ {b} \Delta t) \Delta t ^ {2} \boldsymbol {a} _ {i b} ^ {b} \end{array}\tag{A4}
$$

## Appendix B

Based on (47), $N _ { \Delta t }$ can be further writen as

$$
- \boldsymbol {N} _ {\Delta t} = - \boldsymbol {G} _ {k} \boldsymbol {n} _ {k} = \left[ \begin{array}{c c c} \hat {\boldsymbol {\varphi}} ^ {(n)} & \hat {\boldsymbol {v}} ^ {(n)} & \hat {\boldsymbol {p}} ^ {(n)} \end{array} \right] ^ {\mathrm{T}} \in \mathbb {R} ^ {9 \times 1}\tag{A5}
$$

where the Jacobian matrix $\mathbf { G } _ { k } \in \mathbb { R } ^ { 9 \times 6 }$ can be writen as

$$
\mathbf {G} _ {k} = \left[ \begin{array}{c c} \mathbf {G} _ {1 1} & \mathbf {G} _ {1 2} \\ \mathbf {G} _ {2 1} & \mathbf {G} _ {2 2} \\ \mathbf {G} _ {3 1} & \mathbf {G} _ {3 2} \end{array} \right]\tag{A6}
$$

The exponential form of $N _ { \Delta t }$ is

$$
\exp (- N _ {\Delta t}) = \left[ \begin{array}{c c c} \exp (\hat {\boldsymbol {\varphi}} ^ {(n)}) & \hat {\mathbf {J}} \hat {\boldsymbol {v}} ^ {(n)} & \hat {\mathbf {J}} \hat {\boldsymbol {p}} ^ {(n)} \\ 0 _ {1 \times 3} & 1 & 0 \\ 0 _ {1 \times 3} & 0 & 1 \end{array} \right]\tag{A7}
$$

where $\exp ( \hat { \varphi } ^ { ( n ) } ) = \Gamma _ { \scriptscriptstyle 0 } ( \hat { \varpi } _ { i b } ^ { b } \Delta t ) \quad \quad \Gamma _ { \scriptscriptstyle m } ( \varphi )$ was defined in (30), and $\hat { \bf J } = \Gamma _ { 1 } ( \hat { \omega } _ { i b } ^ { b } \Delta t )$ $\hat { \omega } _ { i b } ^ { b } = \tilde { \omega } _ { i b } ^ { b } - \hat { b } _ { \omega }$

Based on (44), the equation can be obtained as

$$
\hat {\mathbf {Y}} _ {\Delta t} ^ {- 1} \mathbf {Y} _ {\Delta t} = \left[ \begin{array}{c c c} \exp (\hat {\boldsymbol {\varphi}} ^ {(n)}) & \hat {\mathbf {J}} \hat {\boldsymbol {v}} ^ {(n)} & \hat {\mathbf {J}} \hat {\boldsymbol {p}} ^ {(n)} \\ 0 _ {1 \times 3} & 1 & 0 \\ 0 _ {1 \times 3} & 0 & 1 \end{array} \right]\tag{A8}
$$

where $\mathrm { \Delta } \Upsilon _ { \Delta t }$ and $\hat { \mathbf { Y } } _ { \Delta t } ^ { - 1 }$ can be found in (34) and (50). $- { N } _ { \Delta t }$ can be computed as follows

$$
\begin{array}{l} \hat {\varphi} ^ {(n)} = \log \left(\Gamma_ {0} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) ^ {- 1} \Gamma_ {0} \left(\boldsymbol {\omega} _ {i b} ^ {b} \Delta t\right)\right) \\ \approx \log \left(\Gamma_ {0} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) ^ {- 1} \Gamma_ {0} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) \Gamma_ {0} \left(\Gamma_ {1} ^ {\mathrm{T}} \left(\boldsymbol {\omega} _ {i b} ^ {b} \Delta t\right) (- \boldsymbol {n} _ {\omega} \Delta t)\right)\right) \\ = \log \left(\Gamma_ {0} \left(\Gamma_ {1} ^ {\mathrm{T}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) (- \boldsymbol {n} _ {\omega} \Delta t)\right)\right) \\ = - \Gamma_ {1} ^ {\mathrm{T}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) \Delta t \boldsymbol {n} _ {\omega} \\ = \Gamma_ {0} ^ {- 1} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) \Gamma_ {1} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) \Delta t \boldsymbol {n} _ {\omega} \end{array}\tag{A9}
$$

where $\log ( \cdot )$ is the inverse of the exponential operation; the real value of angular velocity $\mathrm { i s } \ : \ : \pmb { \omega } _ { i b } ^ { b } = \hat { \pmb { \omega } } _ { i b } ^ { b } - \pmb { n } _ { \omega }$ , and due to the noise ω is small, based on the property of ${ \pmb n } _ { \omega }$ $\Gamma _ { { m } } \left( \varphi \right)$

$$
\Gamma_ {m} (\boldsymbol {\varphi} + \boldsymbol {\varphi} ^ {\prime}) \approx \Gamma_ {m} (\boldsymbol {\varphi}) \Gamma_ {m} (\Gamma_ {m + 1} (- \boldsymbol {\varphi}) \boldsymbol {\varphi} ^ {\prime}) [ 2 9 ], \quad \boldsymbol {\Gamma} _ {0} (\boldsymbol {\omega} _ {i b} ^ {b} \Delta t)
$$

can be writen as

$$
\begin{array}{l} \hat {\boldsymbol {v}} ^ {(n)} = \Gamma_ {0} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) ^ {- 1} \left(\Gamma_ {1} \left(\boldsymbol {\omega} _ {i b} ^ {b} \Delta t\right) \boldsymbol {a} _ {i b} ^ {b} \Delta t - \Gamma_ {1} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) \hat {\boldsymbol {a}} _ {i b} ^ {b} \Delta t\right) \\ \approx \Gamma_ {0} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) ^ {- 1} \left(\Gamma_ {1} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) \Gamma_ {0} \left(\Gamma_ {2} ^ {\mathrm{T}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) (- \boldsymbol {n} _ {\omega} \Delta t)\right) \boldsymbol {a} _ {i b} ^ {b} \Delta t \right. \\ \left. - \Gamma_ {1} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) \hat {\boldsymbol {a}} _ {i b} ^ {b} \Delta t\right) \\ \approx \Gamma_ {0} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) ^ {- 1} \left(\Gamma_ {\mathrm{I}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) \boldsymbol {a} _ {i b} ^ {b} \Delta t - \Gamma_ {\mathrm{I}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) \hat {\boldsymbol {a}} _ {i b} ^ {b} \Delta t \right. \\ + \Gamma_ {\mathrm{I}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) \left(\Gamma_ {\mathrm{2}} ^ {\mathrm{T}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) (- \boldsymbol {n} _ {\omega} \Delta t)\right) ^ {\wedge} \boldsymbol {a} _ {i b} ^ {b} \Delta t   \\ = \Gamma_ {0} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) ^ {- 1} \left(\Gamma_ {\mathrm{I}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) (- \boldsymbol {n} _ {a} \Delta t) \right. \\ + \Gamma_ {\mathrm{I}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) (\boldsymbol {a} _ {i b} ^ {b}) ^ {\wedge}   \Gamma_ {\mathrm{2}} ^ {\mathrm{T}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) (\boldsymbol {n} _ {\omega} \Delta t)   \Delta t   \\ = - \Bigg (\Big (- \Gamma_ {\mathrm{0}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) ^ {- 1}   \Gamma_ {\mathrm{I}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) \cdot (\boldsymbol {a} _ {i b} ^ {b   \wedge})   \Gamma_ {\mathrm{2}} ^ {\mathrm{T}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) \Delta t ^ {2}   \Big)   n _ {\omega}, \\ + \Gamma_ {\mathrm{0}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) ^ {- 1}   \Gamma_ {\mathrm{I}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) \Delta t n _ {_ a}\Bigg)    . \end{array}\tag{A10}
$$

$$
\begin{array}{l} \hat {\boldsymbol {v}} ^ {(n)} = \Gamma_ {0} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) ^ {- 1} \left(\Gamma_ {1} \left(\boldsymbol {\omega} _ {i b} ^ {b} \Delta t\right) \boldsymbol {a} _ {i b} ^ {b} \Delta t - \Gamma_ {1} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) \hat {\boldsymbol {a}} _ {i b} ^ {b} \Delta t\right) \\ \approx \Gamma_ {0} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) ^ {- 1} \left(\Gamma_ {1} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) \Gamma_ {0} \left(\Gamma_ {2} ^ {\mathrm{T}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) (- \boldsymbol {n} _ {\omega} \Delta t)\right) \boldsymbol {a} _ {i b} ^ {b} \Delta t \right. \\ \left. - \Gamma_ {1} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) \hat {\boldsymbol {a}} _ {i b} ^ {b} \Delta t\right) \\ \approx \Gamma_ {0} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) ^ {- 1} \left(\Gamma_ {\mathrm{I}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) \boldsymbol {a} _ {i b} ^ {b} \Delta t - \Gamma_ {\mathrm{I}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) \hat {\boldsymbol {a}} _ {i b} ^ {b} \Delta t \right. \\ + \Gamma_ {\mathrm{I}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) \left(\Gamma_ {\mathrm{2}} ^ {\mathrm{T}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) (- \boldsymbol {n} _ {\omega} \Delta t)\right) ^ {\wedge} \boldsymbol {a} _ {i b} ^ {b} \Delta t   \\ = \Gamma_ {0} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) ^ {- 1} \left(\Gamma_ {\mathrm{I}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) (- \boldsymbol {n} _ {a} \Delta t) \right. \\ + \Gamma_ {\mathrm{I}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) (\boldsymbol {a} _ {i b} ^ {b}) ^ {\wedge}   \Gamma_ {\mathrm{2}} ^ {\mathrm{T}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) (\boldsymbol {n} _ {\omega} \Delta t)   \Delta t   \\ = - \Bigg (\Bigg (- \Gamma_ {\mathrm{0}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) ^ {- 1}   \Gamma_ {\mathrm{I}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) \cdot (\boldsymbol {a} _ {i b} ^ {b   \wedge})   \Gamma_ {\mathrm{2}} ^ {\mathrm{T}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right)   \Delta t ^ {2}   \Bigg)   n _ {\omega} \\ + \Gamma_ {\mathrm{0}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) ^ {- 1}   \Gamma_ {\mathrm{I}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right)   \Delta t n _ {a}   \Bigg)    . \end{array}\tag{A11}
$$

where $\pmb { a } _ { i b } ^ { b } = \hat { \pmb { a } } _ { i b } ^ { b } - \pmb { n } _ { a }$

$$
\begin{array}{l}\hat {\boldsymbol {p}} ^ {(n)} = \Gamma_ {0} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) ^ {- 1} \Gamma_ {2} \left(\boldsymbol {\omega} _ {i b} ^ {b} \Delta t\right) \boldsymbol {a} _ {i b} ^ {b} \Delta t ^ {2} - \Gamma_ {2} \left(\boldsymbol {\omega} _ {i b} ^ {b} \Delta t\right) \hat {\boldsymbol {a}} _ {i b} ^ {b} \Delta t ^ {2}\\= \Gamma_ {0} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) ^ {- 1} \left(\Gamma_ {2} \left(\boldsymbol {\omega} _ {i b} ^ {b} \Delta t\right) \Gamma_ {0} \left(\Gamma_ {3} ^ {\mathrm{T}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) (- \boldsymbol {n} _ {\omega} \Delta t)\right) \boldsymbol {a} _ {i b} ^ {b} \Delta t ^ {2} \right.\\- \Gamma_ {2} \left(- \hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) \hat {\boldsymbol {a}} _ {i b} ^ {b} \Delta t ^ {2})\\\approx \Gamma_ {0} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) ^ {- 1} \left(\Gamma_ {2} \left(\boldsymbol {\omega} _ {i b} ^ {b} \Delta t\right) \boldsymbol {a} _ {i b} ^ {b} \Delta t ^ {2} - \Gamma_ {2} \left(\boldsymbol {\omega} _ {i b} ^ {b} \Delta t\right) \hat {\boldsymbol {a}} _ {i b} ^ {b} \Delta   t ^ {2} \right.\\+ \Gamma_ {2} \left(- \hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right)\left(\Gamma_ {3} ^ {\mathrm{T}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) (- \boldsymbol {n} _ {\omega} \Delta t)\right) ^ {\wedge} \boldsymbol {a} _ {i b} ^ {b} \Delta t ^ {2}\left. \right)\\= - \Biggl (- \Gamma_ {0} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) ^ {- 1} \Gamma_ {2} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) (\boldsymbol {a} _ {i b} ^ {b}) ^ {\wedge} \Gamma_ {3} ^ {\mathrm{T}} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) \Delta t ^ {3} \boldsymbol {n} _ {\omega}\\+ \Gamma_ {0} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) ^ {- 1} \Gamma_ {2} \left(\hat {\boldsymbol {\omega}} _ {i b} ^ {b} \Delta t\right) \Delta t ^ {2} \boldsymbol {n} _ {a})\end{array}\tag{A12}
$$

In (A5), (A6) and (A8), since the dependent variable in function $\Gamma _ { _ m } ( \varphi ) _ { \mathrm { ~ \scriptsize ~ i s ~ } } \hat { \omega } _ { _ { i b } } ^ { b } \Delta t$ we omit it for brevity. The noise driven matrix k is shown below as $\mathbf { G } _ { \boldsymbol { k } }$

$$
\mathbf {G} _ {k} = \left[ \begin{array}{c c} \Gamma_ {0} ^ {- 1} \Gamma_ {1} \Delta t & \mathbf {0} _ {3 \times 3} \\ - \Gamma_ {0} ^ {- 1} \Gamma_ {1} \boldsymbol {a} _ {i b} ^ {b} \wedge \Gamma_ {2} ^ {\mathrm{T}} \Delta t ^ {2} & \Gamma_ {0} ^ {- 1} \Gamma_ {1} \Delta t \\ - \Gamma_ {0} ^ {- 1} \Gamma_ {2} \boldsymbol {a} _ {i b} ^ {b} \wedge \Gamma_ {3} ^ {\mathrm{T}} \Delta t ^ {3} & \Gamma_ {0} ^ {- 1} \Gamma_ {2} \Delta t ^ {2} \end{array} \right]\tag{A13}
$$

## Appendix C

Based on the definition of the left-invariant error matrix given in (42) and the mapping relationship between Lie groups and Lie algebras provided in (51), the left-invariant error matrix  can be further expressed as $\boldsymbol { \mathsf { \Omega } } \boldsymbol { \mathsf { \Omega } } ^ { l 1 }$

$$
\mathbf {X} ^ {- 1} \hat {\mathbf {X}} = \exp (\boldsymbol {x} ^ {l 1})\tag{A14}
$$

Expansion of both sides of (A14) gives

$$
\begin{array}{r l} & {\hat {\mathbf {R}} _ {b} ^ {e} = \mathbf {R} _ {b} ^ {e} \exp (\pmb {x} _ {\varphi} ^ {l 1 \wedge}) \approx \mathbf {R} _ {b} ^ {e} (\mathbf {I} _ {3 \times 3} + \pmb {x} _ {\varphi} ^ {l 1 \wedge})} \\ & {\delta \pmb {p} _ {i b} ^ {e} = \hat {\pmb {p}} _ {i b} ^ {e} - \pmb {p} _ {i b} ^ {e} = \hat {\mathbf {R}} _ {b} ^ {e} \mathbf {J} ^ {l 1} \pmb {x} _ {p} ^ {l 1}} \end{array}\tag{A15}
$$

The derivation of the observation error proceeds from (71) as follows

$$
\begin{array}{l} \boldsymbol {z} _ {k + 1} ^ {l 1} = \hat {\boldsymbol {p}} _ {o b s e r} ^ {e} - \hat {\boldsymbol {p}} _ {p r e d} ^ {e} \\ \qquad = \boldsymbol {p} _ {o b s e r} ^ {e} + \boldsymbol {v} _ {o b s e r} ^ {e} - (\boldsymbol {p} _ {p r e d} ^ {e} + \delta \boldsymbol {p} _ {p r e d} ^ {e} + \hat {\mathbf {R}} _ {b} ^ {e} \boldsymbol {l}) \\ \approx (\boldsymbol {p} _ {o b s e r} ^ {e} - \boldsymbol {p} _ {p r e d} ^ {e} - \mathbf {R} _ {b} ^ {e} \boldsymbol {l} - \delta \boldsymbol {p} _ {p r e d} ^ {e}) + \mathbf {R} _ {b} ^ {e} \boldsymbol {l} ^ {\wedge} \boldsymbol {x} _ {\varphi} ^ {l 1} + \boldsymbol {v} _ {o b s e r} ^ {e} \\ \qquad = - \delta \boldsymbol {p} _ {i b} ^ {e} + \mathbf {R} _ {b} ^ {e} \boldsymbol {l} ^ {\wedge} \boldsymbol {x} _ {\varphi} ^ {l 1} + \boldsymbol {v} _ {o b s e r} ^ {e} \\ \qquad = \mathbf {R} _ {b} ^ {e} \boldsymbol {l} ^ {\wedge} \boldsymbol {x} _ {\varphi} ^ {l 1} - \mathbf {R} _ {b} ^ {e} \mathbf {J} ^ {l 1} \boldsymbol {x} _ {p} ^ {l 1} + \boldsymbol {v} _ {o b s e r} ^ {e} \\ \qquad = \mathbf {H} ^ {l 1} \boldsymbol {\xi} ^ {l 1} + \boldsymbol {v} _ {o b s e r} ^ {e} \end{array}\tag{A16}
$$

where $\mathbf { H } ^ { l 1 } = \mathbf { R } _ { b } ^ { e } [ l ^ { \wedge } \quad \mathbf { 0 } _ { 3 \times 3 } \quad - \mathbf { J } ^ { l 1 } \quad \mathbf { 0 } _ { 3 \times 3 } \quad \mathbf { 0 } _ { 3 \times 3 } ]$ . According to (79), it can be readily verified that $\mathbf { H } ^ { l 2 } = - \mathbf { H } ^ { l 1 }$ . Similarly, the Jacobian matrices $\mathbf { H } ^ { r 1 }$ and $\mathbf { H } ^ { r 2 }$ can also be derived, and their derivations are omited here.

## References

1. Shi, P.; Zhu, Z.; Sun, S.; Zhao, X.; Tan, M. Invariant Extended Kalman Filtering for Tightly Coupled LiDAR-Inertial Odometry and Mapping. IEEE/ASME Trans. Mechatron. 2023, 28, 2213–2224. https://doi.org/10.1109/TMECH.2022.3233363.

2. Guo, F.; Yang, H.; Wu, X.; Dong, H.; Wu, Q.; Li, Z. Model-Based Deep Learning for Low-Cost IMU Dead Reckoning of Wheeled Mobile Robot. IEEE Trans. Ind. Electron. 2023, 71, 7531–7541. https://doi.org/10.1109/TIE.2023.3301531.

3. Sabatelli, S.; Galgani, M.; Fanucci, L.; Rocchi, A. A Double-Stage Kalman Filter for Orientation Tracking with an Integrated Processor in 9-D IMU. IEEE Trans. Instrum. Meas. 2012, 62, 590–598. https://doi.org/10.1109/TIM.2012.2218692.

4. Li, J.; Zhang, S.; Yang, H.; Jiang, Z.; Bai, X. A Fast Continuous Self-Calibration Method for Fog Rotational Inertial Navigation System Based on Invariant Extended Kalman Filter. IEEE Sens. J. 2022, 23, 2456–2469. https://doi.org/10.1109/JSEN.2022.3226327.

5. Wang, M.; Wu, W.; He, X.; Li, Y.; Pan, X. Consistent ST-EKF for Long Distance Land Vehicle Navigation Based on SINS/OD Integration. IEEE Trans. Veh. Technol. 2019, 68, 10525–10534. https://doi.org/10.1109/TVT.2019.2939679.

6. Scherzinger, B.M.; Reid, D.B. Modified Strapdown Inertial Navigator Error Models. In Proceedings of the 1994 IEEE Position, Location and Navigation Symposium (PLANS’94), Las Vegas, NV, USA, 11–15 April 1994; pp. 426–430. https://doi.org/10.1109/plans.1994.303345.

7. Heo, S.; Park, C.G. Consistent EKF-Based Visual-Inertial Odometry on Matrix Lie Group. IEEE Sens. J. 2018, 18, 3780–3788. https://doi.org/10.1109/JSEN.2018.2808330.

8. Julier, S.J.; Uhlmann, J.K. New Extension of the Kalman Filter to Nonlinear Systems. In Proceedings of the SPIE, Orlando, FL, USA, 21 April 1997; Volume 3068, pp. 182–193. https://doi.org/10.1117/12.280797.

9. Sola, J. Quaternion Kinematics for the Error-State Kalman Filter. arXiv 2017, arXiv:1711.02508. https://doi.org/10.48550/arXiv.1711.02508.

10. Zhao, L.; Chen, T.; Yuan, P.; Tang, Z.; Wang, J. A Novel Vehicle Localization Method Based on Adaptive Singular Spectrum Analysis Using Low-Cost INS/GNSS. IEEE Access 2023, 11, 88670–88685. https://doi.org/10.1109/ACCESS.2023.3305920.

11. Barrau, A. Non-Linear State Error Based Extended Kalman Filters with Applications to Navigation. Ph.D. Thesis, Mines Paris-Tech, Paris, France, 2015. Available online: https://pastel.hal.science/tel-01344622v1 (accessed on 19 May 2024).

12. Zhang, T.; Wu, K.; Song, J.; Huang, S.; Dissanayake, G. Convergence and Consistency Analysis for a 3-D Invariant-EKF SLAM. IEEE Robot. Autom. Lett. 2017, 2, 733–740. https://doi.org/10.1109/LRA.2017.2651376.

13. Li, F.; Chang, L. MEKF with Navigation Frame Attitude Error Parameterization for INS/GPS. IEEE Sens. J. 2019, 20, 1536–1549. https://doi.org/10.1109/JSEN.2019.2947456.

14. Wang, M.; Wu, W.; Zhou, P.; He, X. State Transformation Extended Kalman Filter for GPS/SINS Tightly Coupled Integration GPS Solut. 2018, 22, 112. https://doi.org/10.1007/s10291-018-0773-3.

15. Bailey, T.; Nieto, J.; Guivant, J.; Stevens, M.; Nebot, E. Consistency of the EKF-SLAM Algorithm. In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Beijing, China, 9–15 October 2006; pp. 3562–3568. https://doi.org/10.1109/IROS.2006.281644.

16. Huang, G.P.; Mourikis, A.I.; Roumeliotis, S.I. Observability-Based Rules for Designing Consistent EKF SLAM Estimators. Int. J Robot. Res. 2010, 29, 502–528. https://doi.org/10.1177/0278364909353640.

17. Brossard, M.; Barrau, A.; Bonnabel, S. Exploiting Symmetries to Design EKFs with Consistency Properties for Navigation and SLAM. IEEE Sens. J. 2018, 19, 1572–1579. https://doi.org/10.1109/JSEN.2018.2882714.

18. Hesch, J.A.; Kottas, D.G.; Bowman, S.L.; Roumeliotis, S.I. Camera-IMU-Based Localization: Observability Analysis and Consistency Improvement. Int. J. Robot. Res. 2014, 33, 182–201. https://doi.org/10.1177/0278364913509675.

19. Huang, G.; Kaess, M.; Leonard, J.J. Towards Consistent Visual-Inertial Navigation. In Proceedings of the 2014 IEEE International Conference on Robotics and Automation (ICRA), Hong Kong, China, 31 May–7 June 2014; pp. 4926–4933. https://doi.org/10.1109/icra.2014.6907581.

20. Barrau, A.; Bonnabel, S. Invariant Kalman Filtering. Annu. Rev. Control Robot. Auton. Syst. 2018, 1, 237–257. https://doi.org/10.1146/annurev-control-060117-105010.

21. Brossard, M.; Bonnabel, S.; Barrau, A. Invariant Kalman Filtering for Visual Inertial SLAM. In Proceedings of the 21st International Conference on Information Fusion, Cambridge, UK, 10–13 July 2018; pp. 2021–2028. https://doi.org/10.23919/ICIF.2018.8455807.

22. Barrau, A.; Bonnabel, S. The Invariant Extended Kalman Filter as a Stable Observer. IEEE Trans. Autom. Control 2016, 62, 1797– 1812. https://doi.org/10.1109/TAC.2016.2594085.

23. Lefferts, E.J.; Markley, F.L.; Shuster, M.D. Kalman Filtering for Spacecraft Attitude Estimation. J. Guid. Control Dyn. 1982, 5, 417– 429. https://doi.org/10.2514/6.1982-70

24. Chang, L.; Hu, B.; Li, K. Iterated Multiplicative Extended Kalman Filter for Attitude Estimation Using Vector Observations. IEEE Trans. Aerosp. Electron. Syst. 2016, 52, 2053–2060. https://doi.org/10.1109/TAES.2016.150237.

25. Barrau, A.; Bonnabel, S. Linear Observed Systems on Groups. Syst. Control Lett. 2019, 129, 36–42. https://doi.org/10.1016/j.sysconle.2019.05.005.

26. Chang, L.; Qin, F.; Xu, J. Strapdown Inertial Navigation System Initial Alignment Based on Group of Double Direct Spatial Isometries. IEEE Sens. J. 2021, 22, 803–818. https://doi.org/10.1109/JSEN.2021.3108497.

27. Cui, J.; Wang, M.; Wu, W.; Liu, R.; Yang, W. Enhanced LG-EKF Backtracking Framework for Body-Velocity-Aided Vehicular Integrated Navigation. IEEE Trans. Veh. Technol. 2024, 73, 14212–14223. https://doi.org/10.1109/TVT.2024.3405809.

28. Han, J.; Ouyang, W.; Zhu, M.; Wu, Y. On Covariance Switch-Based Invariant Extended Kalman Filter for Multi-source Fusion In Advances in Guidance, Navigation and Control, Proceedings of 2024 International Conference on Guidance, Navigation and Control, Changsha, China, 9–11 August 2024; Springer: Singapore, 2024; pp. 340–351. https://doi.org/10.1142/S2737480724500183.

29. Bloesch, M.; Hutter, M.; Hoepflinger, M.A.; Leutenegger, S.; Gehring, C.; Remy, C.D.; Siegwart, R. State Estimation for Legged Robots—Consistent Fusion of Leg Kinematics and IMU. In Proceedings of the Robotics: Science and System (RSS), Berlin, Germany, 24–28 June 2013. https://doi.org/10.15607/rss.2012.viii.003.

30. Cui, J.; Wang, M.; Wu, W.; He, X. Lie group based nonlinear state errors for MEMS-IMU/GNSS/magnetometer integrated navigation. J. Navig. 2021, 74, 887–900. https://doi.org/10.1017/s037346332100014x.

31. Hwang, J.H.; Cha, J.; Park, C.G. A Novel Federated Structure of Invariant EKF Using Left-/Right-Invariant Observations. IEEE Sens. J. 2022, 22, 20645–20654. https://doi.org/10.1109/JSEN.2022.3179833.

32. Du, S.; Huang, Y.; Lin, B.; Qian, J.; Zhang, Y. A Lie Group Manifold-Based Nonlinear Estimation Algorithm and Its Application to Low-Accuracy SINS/GNSS Integrated Navigation. IEEE Trans. Instrum. Meas. 2022, 71, 1002927. https://doi.org/10.1109/TIM.2022.3159950.

33. Brossard, M.; Barrau, A.; Bonnabel, S. RINS-W: Robust Inertial Navigation System on Wheels. In Proceedings of the 2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Macau, China, 3–8 November 2019; pp. 2068– 2075. https://doi.org/10.1109/IROS40897.2019.8968593.

34. Brossard, M.; Barrau, A.; Bonnabel, S. AI-IMU Dead-Reckoning. IEEE Trans. Intell. Veh. 2020, 5, 585–595. https://doi.org/10.1109/TIV.2020.2980758.

35. Wu, K.; Zhang, T.; Su, D.; Huang, S.; Dissanayake, G. An Invariant-EKF VINS Algorithm for Improving Consistency. In Proceedings of the 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Vancouver, BC, Canada, 24– 28 September 2017; pp. 1578–1585. https://doi.org/10.1109/IROS.2017.8205965.

36. Luo, Y.; Guo, C.; Liu, J. Equivariant Filtering Framework for Inertial-Integrated Navigation. Satell. Navig. 2021, 2, 30. https://doi.org/10.1186/s43020-021-00061-z.

37. Lin, H.; Bian, H.; Wang, R.; Tang, J. Research on Transfer Alignment Algorithms Based on SE(3) in ECEF Frame. Electronics 2025, 14, 453. https://doi.org/10.3390/electronics14030453.

38. Lu, W.; Luo, Y.; Guo, C.; Jiang, W. An Iterated Equivariant Filter and Its Application in Tightly Coupled SINS/GNSS Integrated Navigation. IEEE Trans. Instrum. Meas. 2025, 74, 9518314. https://doi.org/10.1109/tim.2025.3565028.

39. Forster, C.; Carlone, L.; Dellaert, F.; Scaramuzza, D. On-Manifold Preintegration for Real-Time Visual–Inertial Odometry. IEEE Trans. Robot. 2016, 33, 1–21. https://doi.org/10.1109/TRO.2016.2597321.

40. Brossard, M.; Barrau, A.; Chauchat, P.; Bonnabel, S. Associating Uncertainty to Extended Poses for on Lie Group IMU Preintegration with Rotating Earth. IEEE Trans. Robot. 2021, 38, 998–1015. https://doi.org/10.1109/TRO.2021.3100156.

41. Luo, Y.; Liu, Y.; Guo, C.; Liu, J. The Unified Mathematical Framework for IMU Preintegration in Inertial-Aided Navigation System. arXiv 2021, arXiv:2111.09100. https://doi.org/10.48550/arXiv.2111.09100.

42. Luo, Y.; Lu, F.; Guo, C.; Liu, J. Matrix Lie Group Based Extended Kalman Filtering for Inertial-Integrated Navigation in the Navigation Frame. IEEE Trans. Instrum. Meas. 2023, 73, 9503916. https://doi.org/10.1109/TIM.2023.3329103.

43. Fang, K.; Cai, T.; Wang, B. The Kinematic Models of the SINS and Its Errors on the SE(3) Group in the Earth-Centered Inertial Coordinate System. Sensors 2024, 24, 3864. https://doi.org/10.3390/s24123864.

44. Ding, C. Parameters of Several Classes of BCH Codes. IEEE Trans. Inf. Theory 2015, 61, 5322–5330. https://doi.org/10.1109/TIT.2015.2470251

45. Aceinna. gnss-ins-sim (Version 2.2.0) [Computer Software]. 2019. Available online: https://github.com/Aceinna/gnss-ins-sim (accessed on 4 June 2019).

46. Geiger, A.; Lenz, P.; Stiller, C.; Urtasun, R. Vision Meets Robotics: The KITTI Dataset. Int. J. Robot. Res. 2013, 32, 1231–1237. https://doi.org/10.1177/0278364913491297.

Disclaimer/Publisher’s Note: The statements, opinions and data contained in all publications are solely those of the individual author(s) and contributor(s) and not of MDPI and/or the editor(s). MDPI and/or the editor(s) disclaim responsibility for any injury to people or property resulting from any ideas, methods, instructions or products referred to in the content.