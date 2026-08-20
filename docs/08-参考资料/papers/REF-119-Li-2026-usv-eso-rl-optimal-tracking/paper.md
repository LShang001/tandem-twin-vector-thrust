Research Article

# Adaptive optimal tracking control for underactuated surface vessels using extended state observer and reinforcement learning

![](images/12df7cee29cd1265a3913324c4a5ace0f900cf6e35e1a459e5cf47d071b9da1a.jpg)

Yinkun Li <sup>a</sup>, Yawen Zhou <sup>a</sup>, Yufeng Zhou <sup>a,∗</sup>, Li Chai <sup>b,c,∗</sup>

<sup>a</sup> School of Artificial Intelligence and Automation, Wuhan University of Science and Technology, Wuhan, 430081, China

<sup>b</sup> State Key Laboratory of Industrial Control Technology, College of Control Science and Engineering, Zhejiang University, Hangzhou, 310027, China

<sup>c</sup> Huzhou Institute of Industrial Control Technology, Huzhou, 310027, China

## A R T I C L E I N F O

Keywords: Extended state observer Actor–critic networks Reinforcement learning Backstepping method Underactuated surface vessel

## A B S T R A C T

This paper investigates the adaptive optimal tracking control (AOTC) for underactuated surface vessels (USVs). Compared to the majority of existing studies, the control strategy in this paper innovatively combines an extended state observer (ESO) with reinforcement learning (RL). The designed ESO has high estimation accuracy and robust disturbance rejection capabilities for the unmeasurable information for USVs. To obtain the AOTC, the actor–critic (AC) networks based on RL are constructed to solve the Hamilton–Jacobi–Bellman (HJB) equations. Due to the uncertainties, it is challenging to obtain the optimal controller by directly solving the HJB equations. To address this issue, this paper employs neural networks (NNs) to approximate the uncertainties and solves the optimal controller via AC-RL and ESO. In addition, the adaptive parameters of the optimal controller is trained in parallel with AC networks, which can ensure that the trained networks can further improve tracking performance. The boundedness of AOTC for USVs is shown by Lyapunov stability theorem Finally, simulation results demonstrate the effectiveness of the proposed algorithm.

## 1. Introduction

Over the past few decades, the tracking control, formation control and path-following control of surface vessels have garnered increasing attention in the field of ocean engineering [1], driven by the expanding applications in oceanic transport, marine resource exploration, environmental monitoring, military operations, and maritime rescue missions [2]. In these practical scenarios, many surface vessel control systems face the challenge of being underactuated, meaning that they can only control the surge and yaw motions, while other degrees of freedom remain uncontrolled [3]. This underactuation, coupled with the difficulty of directly measuring certain system states, adds considerable complexity to the development of the effective and robust control strategies [4].

As control requirements become more sophisticated, the researches have yielded various theoretical approaches tailored to the unique needs of USVs [5], particularly in tasks such as formation control and tracking control [6–15]. For instance, Lyapunov method and backstepping method have been applied to achieve asymptotic tracking control [7,8]. Additionally, robust adaptive control methods have been designed to address system uncertainties, enabling point-to-point navigation under passive-boundedness conditions [9,10]. NNs has further enhanced the adaptability for uncertain vessels [12]. However, many existing methods fail to effectively address the challenge of unmeasurable system states [6–15], which is a key issue in achieving high-performance control in real-world marine environments.

State observation techniques, which enable the estimation of unmeasured system states, play an essential role in ensuring accurate control performance, particularly for systems with modeling uncertainties or incomplete measurements [16–24]. The integral observer combining the disturbance and state observers was designed to achieve the uniformly bounded state stability and consistent boundedness of nonlinear systems [17]. The robust adaptive control methods have been developed to handle the model uncertainties for path-following of USVs [20]. Recently, ESO utilizes a combination of state estimation and disturbance compensation to improve system performance without requiring precise dynamic models, making it particularly suitable for USVs [21,22]. These approaches have made significant progress, however, they often overlook optimization considerations, which leads to excessive energy consumption in the control efforts [17–24].

HJB equation provides a method for optimal control, outlining the best control actions under any given state of the system [25,26]. However, directly solving the HJB equation is computationally intractable due to its nonlinear and uncertainties, particularly for the complex USVs. To overcome this challenge, RL techniques have emerged as a powerful alternative for approximating the optimal solution [27–38]. For example, a fuzzy RL was developed to achieve optimal tracking control for USVs under model uncertainties [32]. In addition, NNs are highly effective in approximating complex functions, and the com bination of NNs and AC-RL brings great opportunities for complex USVs [34–36]. For example, an adaptive AC-RL combined with NNs was applied to optimal surrounding control of multiple $\mathrm { U S V } s ,$ effectively solving the HJB equation and improving trajectory efficiency and energy consumption [35]. AC-RL with NNs was applied to the discrete-time $\mathrm { U S V } \mathbf { s } ,$ enabling high-performance trajectory tracking with guaranteed stability and boundedness of all closed-loop signals [36]. However, the existing results required the measurable states [25–37], which is difficult to obtain accurate state information for USVs in complex environments.

Inspired of the aforementioned discussion, this paper investigates the AOTC for USVs via the designed ESO and RL control strategy, which has high estimation accuracy and robust disturbance rejection capabilities for the unmeasurable information for USVs. The main contributions are summarized as follows:

1. Compared with that only considers partial problems, an ESO combined AC-RL are designed for the composite disturbances and modeling uncertainties of USVs. The method in this paper weakens the constraints of the measurable system states [6–13], which can broaden the range of applications, especially for complex marine and military environments.

2. In contrast to the existing RL strategies [28–34], which rely on precisely known system dynamics, we propose an RL-based optimal control algorithm for an underactuated surface vessel, where internal uncertainties are estimated through an observer, eliminating the need for exact dynamic models.

3. Additionally, the dynamic surface control technique [39] is embedded into the AC-RL framework, the derivative of virtual controls in the backstepping process is avoided, simplifying the weight update law and reducing computational complexity. This integration enables the optimal tracking control problem to be solved in terms of optimal policy and cost functions.

Notations: � denotes a scalar variable. � denotes a column vector, and $\pmb { X } \in \mathbb { R } ^ { n \times n }$ is an �-dimensional real square matrix. ‖�‖ denotes the Euclidean norm of a vector �. ⊗ denotes the Kronecker product. $\lambda _ { \mathrm { m i n } } ( X )$ and $\lambda _ { \mathrm { m a x } } ( X )$ denote the minimum and maximum eigenvalues of matrix �.

## 2. Preliminaries

Consider the following three-dimensional USV [40]:

$$
\left\{ \begin{array}{l} \dot {\boldsymbol {a}} = \boldsymbol {J} (\psi) \boldsymbol {b}, \\ \boldsymbol {M} \dot {\boldsymbol {b}} = - \boldsymbol {C} (\boldsymbol {b}) \boldsymbol {b} - \boldsymbol {D} (\boldsymbol {b}) \boldsymbol {b} + \boldsymbol {g} (\boldsymbol {a}) + \tau , \end{array} \right.\tag{1}
$$

where Fig. 1 provides a conceptual overview. $\pmb { a } = [ x , y , \psi ] ^ { \mathrm { T } } \in \mathbb { R } ^ { 3 }$ is the surge displacement, sway displacement and heading angle in the inertial frame. $\boldsymbol { b } = [ u , v , r ] ^ { \mathrm { T } } \in \mathbb { R } ^ { 3 }$ is the surge, sway and yaw velocities in the body-fixed frame. $\boldsymbol { J } ( \boldsymbol { \psi } )$ is the transformation matrix between the body-fixed and inertial frames defined as:

$$
\boldsymbol {J} (\psi) = \left[ \begin{array}{c c c} \cos \psi & - \sin \psi & 0 \\ \sin \psi & \cos \psi & 0 \\ 0 & 0 & 1 \end{array} \right],\tag{2}
$$

which satisfies $\pmb { J } ^ { - 1 } ( \psi ) = \pmb { J } ^ { \mathrm { T } } ( \psi ) . \ \pmb { M } = \pmb { M } ^ { \mathrm { T } }$ is a positive definite inertia matrix. $C ( \boldsymbol { b } ) \ = \ - C ^ { \mathrm { T } } ( \boldsymbol { b } )$ is the Coriolis centripetal matrix and ${ \pmb D } ( { \pmb b } )$ is the damping matrix, which are generally unknown. The restoration force vector $\smash { g ( a ) \ \in \  { \mathbb { R } } ^ { 3 } }$ arises from the gravity and buoyancy. � = $[ \tau _ { u } , 0 , \tau _ { r } ] ^ { \mathrm { T } }$ represents the control forces, where the sway control force is unavailable in the sway motion and the yaw moment is obtained by adjusting the rudder angle or the speed of each propeller.

![](images/be61356cfc6853147dbb43312369e8bcae6411e627072260a9a619714fa6f18f.jpg)  
Fig. 1. The USV dynamic model.

Assumption 1. The reference trajectory $\mathbf { \boldsymbol { a } } _ { \mathrm { d } } = [ x _ { \mathrm { d } } , y _ { \mathrm { d } } , \psi _ { \mathrm { d } } ] ^ { \mathrm { T } }$ and $\dot { \pmb { a } } _ { \mathrm { d } }$ are both bounded. In addition, the velocity � satisfies $| v | \leq \bar { v } ,$ where ̄� is an unknown positive constant.

Assumption 2. The position $( x , y )$ and the heading angle � are observable, while the velocities �, � and � are not directly measurable.

## 3. Extended state observer design

## 3.1. Basic extended state observer design

The accurate model of USV is challenging in practical engineering and its velocity vector b is often not directly measurable. Due to the influence of external environmental disturbances, it is inaccurate for obtaining velocity information � by differentiating the directly measurable position (�, �) and heading angle �. To overcome this, an ESO with high estimation accuracy and robust disturbance rejection capabilities is designed to reconstruct the velocity vector � while simultaneousl estimating the uncertain parameters.

To obtain the dynamic model required by the observer, based on (1), there are

$$
\ddot {\boldsymbol {a}} = \dot {\boldsymbol {J}} (\psi) \boldsymbol {b} + \boldsymbol {J} (\psi) \dot {\boldsymbol {b}},\tag{3}
$$

$$
\dot {\boldsymbol {b}} = \boldsymbol {M} ^ {- 1} [ - \boldsymbol {C} (\boldsymbol {b}) \boldsymbol {b} - \boldsymbol {D} (\boldsymbol {b}) \boldsymbol {b} + \boldsymbol {g} (\boldsymbol {a}) + \tau ].\tag{4}
$$

Substituting (4) into (3), the following dynamic equation can be obtained:

$$
\left\{ \begin{array}{l} \ddot {\boldsymbol {a}} = \boldsymbol {\Theta} + \boldsymbol {B} \boldsymbol {\tau}, \\ \boldsymbol {\Theta} = \boldsymbol {B} \big [ - \boldsymbol {C} (\boldsymbol {b}) - \boldsymbol {D} (\boldsymbol {b}) \boldsymbol {b} + \boldsymbol {g} (\boldsymbol {a}) \big ] + \dot {\boldsymbol {J}} (\psi) \boldsymbol {b}, \\ \boldsymbol {B} = \boldsymbol {J} (\psi) \boldsymbol {M} ^ {- 1}. \end{array} \right.\tag{5}
$$

Define $\begin{array} { r } { { \pmb x } _ { 1 } = { \pmb a } , { \pmb x } _ { 2 } = \dot { { \pmb a } } , } \end{array}$ the ESO is designed:

$$
\left\{ \begin{array}{l} \dot {\hat {\boldsymbol {x}}} _ {1} = \hat {\boldsymbol {x}} _ {2} + l _ {1} (\boldsymbol {x} _ {1} - \hat {\boldsymbol {x}} _ {1}), \\ \dot {\hat {\boldsymbol {x}}} _ {2} = \hat {\boldsymbol {x}} _ {3} + \boldsymbol {B} \boldsymbol {\tau} + l _ {2} (\boldsymbol {x} _ {1} - \hat {\boldsymbol {x}} _ {1}), \\ \dot {\hat {\boldsymbol {x}}} _ {3} = l _ {3} (\boldsymbol {x} _ {1} - \hat {\boldsymbol {x}} _ {1}). \end{array} \right.\tag{6}
$$

where constant gains $l _ { 1 } > 0 , l _ { 2 } > 0 , l _ { 3 } > 0$ satisfy $l _ { 1 } l _ { 2 } > l _ { 3 }$ . The terms $\hat { \mathbf { x } } _ { 1 } , \hat { \mathbf { x } } _ { 2 } ,$ and $\hat { \pmb x } _ { 3 }$ estimate $a , { \dot { a } } ,$ and � respectively.

Define the estimation error vector $\boldsymbol { e } = [ e _ { 1 } ^ { \mathrm { T } } , e _ { 2 } ^ { \mathrm { T } } , e _ { 3 } ^ { \mathrm { T } } ] ^ { \mathrm { T } }$ and

$$
e _ {1} = x _ {1} - \hat {x} _ {1}, e _ {2} = x _ {2} - \hat {x} _ {2}, e _ {3} = \Theta - \hat {x} _ {3}.\tag{7}
$$

Based on (6), the observer error dynamics is governed by

$$
\left\{ \begin{array}{l} \dot {e} _ {1} = - l _ {1} e _ {1} + e _ {2}, \\ \dot {e} _ {2} = - l _ {2} e _ {1} + e _ {3}, \\ \dot {e} _ {3} = - l _ {3} e _ {1} + \dot {\Theta}, \end{array} \right.\tag{8}
$$

which admits the state-space representation:

$$
\dot {\boldsymbol {e}} = (\boldsymbol {A} \otimes \boldsymbol {I} _ {3}) \boldsymbol {e} + (\boldsymbol {K} \otimes \boldsymbol {I} _ {3}) \dot {\boldsymbol {\Theta}},\tag{9}
$$

where $\pmb { A } = [ - l _ { 1 } , 1 , 0 ; - l _ { 2 } , 0 , 1 ; - l _ { 3 } , 0 , 0 ] ; \pmb { K } = [ 0 ; 0 ; 1 ] .$

Theorem 1. Under bounded disturbance derivative $\| \dot { \pmb \theta } \| \le S , i f l _ { 1 } l _ { 2 } > l _ { 3 } ,$ the estimation error converges exponentially to the residual set:

$$
\Xi_ {1} = \left\{\boldsymbol {e} (t) \in \mathbb {R} ^ {9} \mid \| \boldsymbol {e} (t) \| \leq \sqrt {\frac {\| \boldsymbol {P} _ {\mathrm{o}} \boldsymbol {K} \| ^ {2} S ^ {2}}{2 \theta \ell_ {\mathrm{o}} \lambda_ {\min} (\boldsymbol {P} _ {1})}} \right\}.
$$

Proof. The characteristic equation det( $s I - A ) = s ^ { 3 } + l _ { 1 } s ^ { 2 } + l _ { 2 } s + l _ { 3 }$ satisfies Routh–Hurwitz stability criteria when $l _ { 1 } l _ { 2 } > l _ { 3 }$ . Therefore, there exists a positive define matrix $\boldsymbol { P } _ { \mathrm { o } } \in \mathbb { R } ^ { 3 \times 3 }$ satisfying $A ^ { \mathrm { T } } P _ { \mathrm { o } } + P _ { \mathrm { o } } A = - Q _ { \mathrm { o } }$ for $Q _ { \mathrm { o } } \in \mathbb { R } ^ { 3 \times 3 } \succ 0$

Define the Lyapunov function as $\begin{array} { r } { V _ { \mathrm { o } } = \frac { 1 } { 2 } e ^ { \mathrm { T } } ( P _ { \mathrm { o } } \otimes I _ { 3 } ) e _ { \mathrm { i } } } \end{array}$ , then,

$$
\begin{array}{l} \dot {V} _ {\mathrm{o}} = \frac {1}{2} e ^ {\mathrm{T}} \big ((A ^ {\mathrm{T}} P _ {\mathrm{o}} + P _ {\mathrm{o}} A) \otimes I _ {3} \big) e + e ^ {\mathrm{T}} \big ((P _ {\mathrm{o}} K) \otimes I _ {3} \big) \dot {\Theta} \\ \leq - \frac {1}{2} \lambda_ {\min} (Q _ {\mathrm{o}}) \| e \| ^ {2} + \| P _ {\mathrm{o}} K \| S \| e \| \\ \leq - \frac {1}{2} \lambda_ {\min} (Q _ {\mathrm{o}}) \| e \| ^ {2} + \frac {\theta}{2} \| e \| ^ {2} + \frac {1}{2 \theta} \| P _ {\mathrm{o}} K \| ^ {2} S ^ {2} \\ \leq - \ell_ {\mathrm{o}} V _ {\mathrm{o}} + \varphi_ {\mathrm{o}}, \end{array}\tag{10}
$$

where $\begin{array} { r } { S \ge 0 , \ell _ { \mathrm { o } } = \frac { \lambda _ { \operatorname* { m i n } } ( Q _ { \mathrm { o } } ) - \theta } { \lambda _ { \operatorname* { m a x } } ( P _ { \mathrm { o } } ) } , \varphi _ { \mathrm { o } } = \frac { \| P _ { \mathrm { o } } K \| ^ { 2 } S ^ { 2 } } { 2 \theta } , 0 < \theta < \lambda _ { \operatorname* { m i n } } ( Q _ { \mathrm { o } } ) . } \end{array}$ □

## 3.2. Advance extended state observer design

Based on (6), a novel adaptive nonlinear ESO is designed:

$$
\left\{ \begin{array}{l} \dot {\hat {x}} _ {1} = \hat {x} _ {2} + k _ {1} e _ {1}, \\ \dot {\hat {x}} _ {2} = \hat {x} _ {3} + k _ {2} \cdot \operatorname{sfal} (e _ {1}) + B \tau , \\ \dot {\hat {x}} _ {3} = k _ {3} \cdot \operatorname{sfal} (e _ {1}). \end{array} \right.\tag{11}
$$

The function sfal(⋅) = [sfal , sfal , sfal ]<sup>T</sup> ∈ R<sup>3</sup> is selected as:

$$
\operatorname{sfal} _ {i} (e _ {1 i}) = \left\{ \begin{array}{l l} e _ {1 i} / q ^ {1 - h}, & | e _ {1 i} | \leq q, \\ | e _ {1 i} | q ^ {h - 1 + \gamma (| e _ {1 i} | - q)} \operatorname{sgn} (e _ {1 i}), & | e _ {1 i} | > q, \end{array} \right.\tag{12}
$$

where $h \in [ 0 , 1 ]$ is a nonlinearity factor, $q > 0$ is a filtering factor, $\begin{array} { r } { 0 < \gamma < \frac { 1 - h } { \gamma } . } \end{array}$ � −� and $q _ { \mathrm { m a x } }$ is the system’s maximum allowable observation error, $e _ { 1 i } = x _ { 1 i } ^ { * } - \hat { x } _ { 1 i }$ and $x _ { 1 i }$ is the �th element of $\mathbf { \delta } x _ { 1 } , i = 1 , 2 , 3$

To satisfy the pole placement requirements, the ESO parameters satisfy:

$$
\left\{ \begin{array}{l} k _ {1} = \mathrm{diag} (3 \hbar , 3 \hbar , 3 \hbar), \\ k _ {2} = \mathrm{diag} (\frac {3 \hbar^ {2}}{F}, \frac {3 \hbar^ {2}}{F}, \frac {3 \hbar^ {2}}{F}), \\ k _ {3} = \mathrm{diag} (\frac {\hbar^ {3}}{F}, \frac {\hbar^ {3}}{F}, \frac {\hbar^ {3}}{F}), \end{array} \right.\tag{13}
$$

where ℏ is the bandwidth, $e _ { \operatorname* { m a x } } \leq F \leq q ^ { h - 1 } , e _ { \operatorname* { m a x } }$ is the threshold of the ESO’s observation error. Based on Theorem 1, the estimation error of nonlinear ESO (11) is convergent.

Remark 1. The function $s \mathrm { f a l } _ { i } ( \cdot ) ,$ as an adaptive nonlinear operator, enhances the extended observer’s capability to address composite dis turbances and modeling uncertainties through a dual-mode structure When the estimation error $| e _ { 1 i } |$ is small $\begin{array} { r } { \begin{array} { l c r } { ( | e _ { 1 i } | } & { \leq } & { q ) } \end{array} } \end{array}$ , the operator operates in a linear mode with normalized gain $1 / q ^ { 1 - h }$ . This configura tion preserves estimation accuracy and suppresses noise amplification, critical for maintaining precision in low-error regions. For larger errors $( | e _ { 1 i } | > q ) _ { : }$ , sfal (⋅) activates a nonlinear mode governed by an adaptive exponent $h + \gamma ( | e _ { 1 i } | - q ) ;$ , where � is a regulating factor that ensures the effectiveness of the observer. This mode dynamically scales the gain proportionally to the error magnitude, where can enable rapid attenuation of disturbances and accelerate convergence. Compared with Refs. [21,23,24], this adaptive nonlinear ESO ensures continuous function behavior at $| e _ { 1 i } | \ = \ q ,$ , which reduces chattering caused by abrupt transitions.

Remark 2. To balance the stability and smoothness, the normalization factor � is limited within a reasonable range. On the one hand, as the error approaches the maximum allowable threshold $e _ { \mathrm { m a x } } ,$ the lower limit $e _ { \mathrm { m a x } }$ prevents excessive of the gains $k _ { 2 }$ and $k _ { 3 } ,$ which prevents the observer output from saturation or divergence. On the other hand, the upper bound $q ^ { h - 1 }$ ensures that the linear segment gain $1 / q ^ { 1 - h }$ matches the derivative of the nonlinear segment $h q ^ { \bar { h } - 1 }$ at the switching point $| e _ { 1 i } | = q ,$ , which can eliminate chattering caused by discontinuity. This dual constraint ensures that the observer maintains stable performance over small and large error ranges, while preserving the continuit necessary for reliable operation under complex disturbances.

## 4. Optimized control design

## 4.1. Yaw motion control

In this section, an AC-RL strategy is employed to obtain the optimal control solution, where both the control policy and the value function are simultaneously and optimally updated.

Step 1: Define the orientation tracking error $z _ { 1 }$ as:

$$
z _ {1} = \psi - \psi_ {\mathrm{d}},\tag{14}
$$

where $\psi _ { \mathrm { d } } = $ atan $\left[ y _ { \mathrm { e } } , x _ { \mathrm { e } } \right) \in \left( - \pi , \pi \right]$ is the desired heading angle, $x _ { \mathrm { { e } } } =$ $x \mathrm { ~ - ~ } x _ { \mathrm { d } }$ and $y _ { \mathrm { e } } = y - y _ { \mathrm { d } }$ are the position errors, $( x _ { \mathrm { { d } } } , y _ { \mathrm { { d } } } )$ is the desired position. Based on (1) and (2),

$$
\dot {z} _ {\mathrm{l}} = \dot {\psi} - \dot {\psi} _ {\mathrm{d}} = r - \dot {\psi} _ {\mathrm{d}}.\tag{15}
$$

Remark 3. The orientation tracking error $z _ { 1 }$ in (14) measures the deviation between the actual yaw angle � and the desired yaw angle $\psi _ { \mathrm { d } } .$ This scalar error signal provides a compact way to analyze and control the yaw dynamics of the USV.

Furthermore, the designed ESO (11) can estimate the unmeasurable velocity � in (15), there is

$$
\dot {z} _ {1} = \hat {r} - \dot {\psi} _ {\mathrm{d}} + k _ {1 3} e _ {1 3},\tag{16}
$$

where ̂� is the estimation of � based on (11), $k _ { 1 i }$ denotes the �-th diagonal entry of the gain matrix $\boldsymbol { k } _ { 1 } , ~ \boldsymbol { e } _ { 1 i }$ is the �-th element of the estimation error vector $e _ { 1 } , i = 1 , 2 , 3 .$

Remark 4. Since the yaw rate � is not directly measurable in practice, the ESO introduced in (11) provides a real-time estimate ̂� to reconstruct the system dynamics in (15), enabling control design based on observable quantities

Let $\alpha _ { r } ( z _ { 1 } )$ be the virtual control input corresponding to the subsystem described in (16). The performance index function is formulated as:

$$
J _ {1} (z _ {1} (t)) = \int_ {t} ^ {\infty} h _ {1} (z _ {1} (s), \alpha_ {r} (z _ {1} (s))) \mathrm{d} s,\tag{17}
$$

where $h _ { 1 } ( z _ { 1 } ( s ) , \alpha _ { r } ( z _ { 1 } ( s ) ) ) = z _ { 1 } ^ { 2 } ( s ) + \alpha _ { r } ^ { 2 } ( z _ { 1 } ( s ) )$ . The optimal performance index function can be rewritten as

$$
J _ {1} ^ {*} (z _ {1} (t)) = \min _ {\alpha_ {r}} \int_ {t} ^ {\infty} h _ {1} (z _ {1} (s), \alpha_ {r} (z _ {1} (s))) \mathrm{d} s
$$

$$
= \int_ {t} ^ {\infty} z _ {1} ^ {2} (s) + \left(\alpha_ {r} ^ {*} (z _ {1} (s))\right) ^ {2} d s,\tag{18}
$$

where $\alpha _ { r } ^ { * } ( z _ { 1 } )$ is the optimal control of $\alpha _ { r } ( z _ { 1 } ) . { \mathrm { i } } . { \mathrm { e } } . , \alpha _ { r } ^ { * } \triangleq { \hat { r } } .$

Remark 5. The cost function in (17) incorporates a quadratic penalty on the virtual control $\alpha _ { r } ,$ which helps suppress excessive control effort. This design naturally promotes energy-efficient behavior while ensuring tracking performance—an essential feature for power-limited systems like USVs.

According to (16), (17) and (18), HJB function can be obtained as:

$$
\begin{array}{c} H _ {1} \bigg (z _ {1}, \alpha_ {r} ^ {*}, \frac {\partial J _ {1} ^ {*}}{\partial z _ {1}} \bigg) = h _ {1} \left(z _ {1}, \alpha_ {r} ^ {*} (z _ {1})\right) + \frac {\partial J _ {1} ^ {*}}{\partial z _ {1}} \dot {\hat {z}} _ {1} \\ = z _ {1} ^ {2} + \alpha_ {r} ^ {* 2} + \frac {\partial J _ {1} ^ {*}}{\partial z _ {1}} (\alpha_ {r} ^ {*} - \dot {\psi} _ {\mathrm{d}} + k _ {1 3} e _ {1 3}). \end{array}\tag{19}
$$

Since the optimal virtual control $\alpha _ { r } ^ { * }$ is unique satisfying (19), there is obtained by $\begin{array} { r } { \frac { \partial H _ { 1 } } { \partial \alpha _ { r } ^ { * } } = 0 } \end{array}$ as

$$
\alpha_ {r} ^ {*} = - \frac {1}{2} \frac {\partial J _ {1} ^ {*}}{\partial z _ {1}}.\tag{20}
$$

Remark 6. Substituting (20) into (19),

$$
\begin{array}{l} H _ {1} \left(z _ {1}, \alpha_ {r} ^ {*}, \frac {\partial J _ {1} ^ {*}}{\partial z _ {1}}\right) \\ = z _ {1} ^ {2} - \frac {1}{4} \left(\frac {\partial J _ {1} ^ {*}}{\partial z _ {1}}\right) ^ {2} - \frac {\partial J _ {1} ^ {*}}{\partial z _ {1}} \left(\dot {\psi} _ {\mathrm{d}} - k _ {1 3} e _ {1 3}\right) = 0. \end{array}\tag{21}
$$

The optimal virtual control $\alpha _ { r } ^ { * }$ can be generally obtained by solving Eq. (21). However, $\frac { \partial J _ { 1 } ^ { * } } { \partial z _ { 1 } }$ is nonlinear and unknowns, it is extremely <sup>1</sup>difficult to solve in practice. Therefore, AC-RL and NNs are employed to approximate the optimal virtual control $\alpha _ { r } ^ { * } .$

The unique solution $\frac { \partial J _ { 1 } ^ { * } } { \partial z _ { 1 } }$ can be decomposed as:

$$
\frac {\partial J _ {1} ^ {*}}{\partial z _ {1}} = 2 \beta_ {1} z _ {1} + J _ {1} ^ {0} (z _ {1}),\tag{22}
$$

where $\begin{array} { r } { \beta _ { 1 } > 0 , J _ { 1 } ^ { \mathrm { o } } ( z _ { 1 } ) = - 2 \beta _ { 1 } z _ { 1 } + \frac { \partial J _ { 1 } ^ { \ast } } { \partial z _ { 1 } } . } \end{array}$

Remark 7. The decomposition in (22) is introduced to facilitate the use of NNs for function approximation. The term $J _ { 1 } ^ { \mathrm { o } } ( z _ { 1 } )$ isolates the unknown nonlinear part of $\frac { \partial J _ { 1 } ^ { * } } { \partial z _ { 1 } }$ , making it suitable for learning via AC-RL.

Substituting (22) into (20), the optimal virtual control $\alpha _ { r } ^ { * }$ can be rewritten as:

$$
\alpha_ {r} ^ {*} = - \beta_ {1} z _ {1} - \frac {1}{2} J _ {1} ^ {\mathrm{o}} (z _ {1}),\tag{23}
$$

since $J _ { 1 } ^ { \mathrm { o } } ( z _ { 1 } )$ is unknown, it is approximated by NNs as:

$$
J _ {1} ^ {\mathrm{o}} (z _ {1}) = \omega_ {1} ^ {* \mathrm{T}} \vartheta_ {1} (z _ {1}) + \varepsilon_ {1} (z _ {1}),\tag{24}
$$

where $\pmb { \omega } _ { 1 } ^ { * } \in \mathbb { R } ^ { w }$ is the ideal weight vector, $\pmb { \vartheta } _ { 1 } ( z _ { 1 } ) \in \mathbb { R } ^ { w }$ is basis function vector, � is the number of neurons, $\varepsilon _ { 1 } ( z _ { 1 } )$ is the approximation error satisfying $\lVert \varepsilon _ { 1 } ( z _ { 1 } ) \rVert \leq \bar { \varepsilon } _ { 1 }$ . In addition, the AC-RL framework is employed to deal with the unknown ideal weight $\omega _ { 1 } ^ { * }$ in (24):

$$
\hat {J} _ {1} ^ {\mathrm{o}} (z _ {1}) = \hat {\omega} _ {\mathrm{c} 1} ^ {\mathrm{T}} \vartheta_ {1} (z _ {1}),\tag{25}
$$

where $\hat { J } _ { 1 } ^ { \mathrm { o } } ( z _ { 1 } )$ is the estimation of $J _ { 1 } ^ { \mathrm { o } } ( z _ { 1 } ) , \hat { \omega } _ { \mathrm { c 1 } } \in \mathbb { R } ^ { w }$ is the identifier weight vector of $\omega _ { 1 } ^ { * } .$ . The updating laws of $\hat { \omega } _ { \mathrm { c l } }$ is

$$
\dot {\hat {\boldsymbol {\omega}}} _ {\mathrm{cl}} = - \gamma_ {\mathrm{cl}} \boldsymbol {B} _ {1} \hat {\boldsymbol {\omega}} _ {\mathrm{cl}},\tag{26}
$$

where $\gamma _ { \mathrm { c l } } > 0 , d _ { 1 } > 0 , B _ { 1 } = \vartheta _ { 1 } ( z _ { 1 } ) \vartheta _ { 1 } ^ { \mathrm { T } } ( z _ { 1 } ) + d _ { 1 } I _ { w } , I _ { w } \in \mathbb { R } ^ { w \times w }$ is identity matrix. Therefore, the virtual control is defined as:

$$
\hat {\alpha} _ {r} = - \beta_ {1} z _ {1} - \frac {1}{2} \hat {\pmb {\omega}} _ {\mathrm{al}} ^ {\mathrm{T}} \pmb {\vartheta} _ {1} (z _ {1}).\tag{27}
$$

where $\hat { \alpha } _ { r }$ is the estimation of $\alpha _ { r } ^ { \ast } , \hat { \omega } _ { \mathrm { a l } } \in \mathbb { R } ^ { w }$ denotes the actor network weight. We propose the following updating law for the actor network weight:

$$
\dot {\hat {\boldsymbol {\omega}}} _ {\mathrm{al}} = - \boldsymbol {\mathcal {B}} _ {1} \left[ \gamma_ {\mathrm{al}} (\hat {\boldsymbol {\omega}} _ {\mathrm{al}} - \hat {\boldsymbol {\omega}} _ {\mathrm{cl}}) + \gamma_ {\mathrm{cl}} \hat {\boldsymbol {\omega}} _ {\mathrm{cl}} \right],\tag{28}
$$

where $\gamma _ { \mathrm { a l } } > 0$ is the actor learning rate.

Remark 8. The virtual controller $\hat { \alpha } _ { r }$ is designed to satisfied the HJB function $H _ { 1 } ( \cdot )  0$ . Specifically, setting (22), (24) and (27) into (19), there is

$$
\frac {\partial H _ {1} (\cdot)}{\partial \hat {\boldsymbol {\omega}} _ {\mathrm{al}}} = \frac {1}{2} \boldsymbol {\vartheta} _ {1} (z _ {1}) \boldsymbol {\vartheta} _ {1} ^ {\mathrm{T}} (z _ {1}) (\hat {\boldsymbol {\omega}} _ {\mathrm{al}} - \hat {\boldsymbol {\omega}} _ {\mathrm{cl}}).\tag{29}
$$

In order to verify $\partial H _ { 1 } ( \cdot ) / \partial \hat { \omega } _ { \mathrm { a l } } \to 0 , \mathrm { i . e . , } \hat { \omega } _ { \mathrm { a l } } \to \hat { \omega } _ { \mathrm { c l } }$ , define the following function:

$$
E _ {\mathrm{l}} = \frac {1}{2} (\hat {\pmb {\omega}} _ {\mathrm{al}} - \hat {\pmb {\omega}} _ {\mathrm{cl}}) ^ {\mathrm{T}} (\hat {\pmb {\omega}} _ {\mathrm{al}} - \hat {\pmb {\omega}} _ {\mathrm{cl}}).
$$

Substitute (26) and (28) into (30), we obtain

(30)

$$
\begin{array}{r l} \frac {\partial E _ {1}}{\partial t} & = (\hat {\boldsymbol {\omega}} _ {\mathrm{al}} - \hat {\boldsymbol {\omega}} _ {\mathrm{cl}}) ^ {\mathrm{T}} (\dot {\hat {\boldsymbol {\omega}}} _ {\mathrm{al}} - \dot {\hat {\boldsymbol {\omega}}} _ {\mathrm{cl}}) \\ & = (\hat {\boldsymbol {\omega}} _ {\mathrm{al}} - \hat {\boldsymbol {\omega}} _ {\mathrm{cl}}) ^ {\mathrm{T}} (- \gamma_ {\mathrm{al}} \pmb {B} _ {1} \hat {\boldsymbol {\omega}} _ {\mathrm{al}} + \gamma_ {\mathrm{al}} \pmb {B} _ {1} \hat {\boldsymbol {\omega}} _ {\mathrm{cl}}) \\ & \leq - \gamma_ {\mathrm{al}} \lambda_ {\min} (\pmb {B} _ {1}) (\hat {\boldsymbol {\omega}} _ {\mathrm{al}} - \hat {\boldsymbol {\omega}} _ {\mathrm{cl}}) ^ {\mathrm{T}} (\hat {\boldsymbol {\omega}} _ {\mathrm{al}} - \hat {\boldsymbol {\omega}} _ {\mathrm{cl}}) \\ & \leq 0, \end{array}\tag{31}
$$

which means $\hat { \omega } _ { \mathrm { a l } } \to \hat { \omega } _ { \mathrm { c l } }$ and $\partial H _ { 1 } ( \cdot ) / \partial \hat { \pmb { \omega } } _ { \mathrm { a l } }  0$ via the designed laws (26) and (28). Therefore, the proposed AC-RL algorithm is effective.

Remark 9. In practical applications, directly using the virtual control signal ̂�r often results in non-smooth control actions due to the presence of high-frequency components or abrupt variations, which can degrade the tracking performance of USVs. Thus, define the error variable $\hat { z } _ { 2 }$ and the boundary layer error � as:

$$
\hat {z} _ {2} = \hat {r} - \alpha_ {\mathrm{f} r}, y _ {r} = \alpha_ {\mathrm{f} r} - \hat {\alpha} _ {r},\tag{32}
$$

where the filtered signal $\alpha _ { \mathrm { f } r }$ serves as the actual input in the subsequent control law to improve smoothness and tracking quality. The first-order filter is designed as:

$$
c _ {1} \dot {\alpha} _ {\mathrm{f} r} + \alpha_ {\mathrm{f} r} = \hat {\alpha} _ {r}, \alpha_ {\mathrm{f} r} (0) = \hat {\alpha} _ {r} (0),\tag{33}
$$

where $c _ { 1 } > 0 .$ . Based on (14) and (16), there is

$$
\dot {z} _ {1} = \hat {z} _ {2} + \hat {\alpha} _ {r} + y _ {r} + k _ {1 3} e _ {1 3} - \dot {\psi} _ {\mathrm{d}}.\tag{34}
$$

where the boundary layer error $y _ { r }$ satisfies

$$
\dot {y} _ {r} = - \frac {y _ {r}}{c _ {1}} - D _ {1} (\cdot),\tag{35}
$$

where $D _ { 1 } ( \cdot ) \triangleq \dot { \hat { \alpha } } _ { r }$ is a nonlinear continuous function involving system signals. In summary, the first-order filter not only facilitates smooth trajectory tracking for marine vessels, but also enhances the robustness and implementability of the controller by avoiding repeated differenti ation of nonlinear signals

Consider the following Lyapunov function:

$$
V _ {\mathrm{l}} = \frac {1}{2} z _ {\mathrm{l}} ^ {2} + \frac {1}{2} \tilde {\omega} _ {\mathrm{cl}} ^ {\mathrm{T}} \tilde {\omega} _ {\mathrm{cl}} + \frac {1}{2} \tilde {\omega} _ {\mathrm{al}} ^ {\mathrm{T}} \tilde {\omega} _ {\mathrm{al}},\tag{36}
$$

where $\tilde { \omega } _ { \mathrm { c l } } = \hat { \omega } _ { \mathrm { c l } } - \omega _ { 1 } ^ { * } , \tilde { \omega } _ { \mathrm { a l } } = \hat { \omega } _ { \mathrm { a l } } - \omega _ { 1 } ^ { * }$ . Based on (26)–(28) and (34), there is

$$
\begin{array}{l} \dot {V} _ {1} = z _ {1} \big (\hat {z} _ {2} - \beta_ {1} z _ {1} - \frac {1}{2} \hat {\boldsymbol {\omega}} _ {\mathrm{al}} ^ {\mathrm{T}} \boldsymbol {\vartheta} _ {1} + y _ {r} + k _ {1 3} e _ {1 3} - \psi_ {\mathrm{d}} \big) \\ \qquad - \gamma_ {\mathrm{c} 1} \tilde {\boldsymbol {\omega}} _ {\mathrm{c} 1} ^ {\mathrm{T}} \boldsymbol {B} _ {1} \hat {\boldsymbol {\omega}} _ {\mathrm{c} 1} - \tilde {\boldsymbol {\omega}} _ {\mathrm{a} 1} ^ {\mathrm{T}} \boldsymbol {B} _ {1} [ \gamma_ {\mathrm{a} 1} (\hat {\boldsymbol {\omega}} _ {\mathrm{a} 1} - \hat {\boldsymbol {\omega}} _ {\mathrm{c} 1}) + \gamma_ {\mathrm{c} 1} \hat {\boldsymbol {\omega}} _ {\mathrm{c} 1} ] \\ = z _ {1} \hat {z} _ {2} - \beta_ {1} z _ {1} ^ {2} - \frac {1}{2} z _ {1} \hat {\boldsymbol {\omega}} _ {\mathrm{a} 1} ^ {\mathrm{T}} \boldsymbol {\vartheta} _ {1} + z _ {1} y _ {r} + k _ {1 3} z _ {1} e _ {1 3} \\ \qquad - z _ {1} \psi_ {\mathrm{d}} - \gamma_ {\mathrm{c} 1} \tilde {\boldsymbol {\omega}} _ {\mathrm{c} 1} ^ {\mathrm{T}} \boldsymbol {B} _ {1} \hat {\boldsymbol {\omega}} _ {\mathrm{c} 1} - \gamma_ {\mathrm{a} 1} \tilde {\boldsymbol {\omega}} _ {\mathrm{a} 1} ^ {\mathrm{T}} \boldsymbol {B} _ {1} \hat {\boldsymbol {\omega}} _ {\mathrm{a} 1} \\ \qquad + (\gamma_ {\mathrm{a} 1} - \gamma_ {\mathrm{c} 1}) \tilde {\boldsymbol {\omega}} _ {\mathrm{a} 1} ^ {\mathrm{T}} \boldsymbol {B} _ {1} \hat {\boldsymbol {\omega}} _ {\mathrm{c} 1}. \end{array}\tag{37}
$$

Based on (36), there is

$$
\begin{array}{r l} & {\gamma_ {\mathrm{c} 1} \tilde {\omega} _ {\mathrm{c} 1} ^ {\mathrm{T}} \boldsymbol {B} _ {1} \hat {\omega} _ {\mathrm{c} 1} = \frac {\gamma_ {\mathrm{c} 1}}{2} (\tilde {\omega} _ {\mathrm{c} 1} ^ {\mathrm{T}} \boldsymbol {B} _ {1} \tilde {\omega} _ {\mathrm{c} 1} + \hat {\omega} _ {\mathrm{c} 1} ^ {\mathrm{T}} \boldsymbol {B} _ {1} \hat {\omega} _ {\mathrm{c} 1} - \omega_ {1} ^ {* \mathrm{T}} \boldsymbol {B} _ {1} \omega_ {1} ^ {*}),} \\ & {\gamma_ {\mathrm{a} 1} \tilde {\omega} _ {\mathrm{a} 1} ^ {\mathrm{T}} \boldsymbol {B} _ {1} \hat {\omega} _ {\mathrm{a} 1} = \frac {\gamma_ {\mathrm{a} 1}}{2} (\tilde {\omega} _ {\mathrm{a} 1} ^ {\mathrm{T}} \boldsymbol {B} _ {1} \tilde {\omega} _ {\mathrm{a} 1} + \hat {\omega} _ {\mathrm{a} 1} ^ {\mathrm{T}} \boldsymbol {B} _ {1} \hat {\omega} _ {\mathrm{a} 1} - \omega_ {1} ^ {* \mathrm{T}} \boldsymbol {B} _ {1} \omega_ {1} ^ {*}),} \\ & {(\gamma_ {\mathrm{a} 1} - \gamma_ {\mathrm{c} 1}) \tilde {\omega} _ {\mathrm{a} 1} ^ {\mathrm{T}} \boldsymbol {B} _ {1} \hat {\omega} _ {\mathrm{c} 1} \leq \frac {\gamma_ {\mathrm{a} 1} - \gamma_ {\mathrm{c} 1}}{2} (\tilde {\omega} _ {\mathrm{a} 1} ^ {\mathrm{T}} \boldsymbol {B} _ {1} \tilde {\omega} _ {\mathrm{a} 1} + \hat {\omega} _ {\mathrm{c} 1} ^ {\mathrm{T}} \boldsymbol {B} _ {1} \hat {\omega} _ {\mathrm{c} 1}),} \end{array}
$$

where $\gamma _ { \mathrm { c l } } < \gamma _ { \mathrm { a l } } . \ \mathrm { B y }$ Young’s inequality,

$$
\begin{array}{l} \dot {V} _ {1} \leq - (\beta_ {1} - \frac {3}{2}) z _ {1} ^ {2} + \frac {1}{2} \hat {z} _ {2} ^ {2} + y _ {r} ^ {2} - \frac {\gamma_ {\mathrm{c} 1}}{2} \tilde {\boldsymbol {\omega}} _ {\mathrm{c} 1} ^ {\mathrm{T}} \boldsymbol {B} _ {1} \tilde {\boldsymbol {\omega}} _ {\mathrm{c} 1} \\ \quad - \frac {2 \gamma_ {\mathrm{c} 1} - \gamma_ {\mathrm{a} 1}}{2} \hat {\boldsymbol {\omega}} _ {\mathrm{c} 1} ^ {\mathrm{T}} \boldsymbol {B} _ {1} \hat {\boldsymbol {\omega}} _ {\mathrm{c} 1} - \frac {\gamma_ {\mathrm{c} 1}}{2} \tilde {\boldsymbol {\omega}} _ {\mathrm{a} 1} ^ {\mathrm{T}} \boldsymbol {B} _ {1} \tilde {\boldsymbol {\omega}} _ {\mathrm{a} 1} \\ \quad - \frac {1}{4} \hat {\boldsymbol {\omega}} _ {\mathrm{a} 1} ^ {\mathrm{T}} (2 \gamma_ {\mathrm{a} 1} - 1) \boldsymbol {B} _ {1} \hat {\boldsymbol {\omega}} _ {\mathrm{a} 1} + \varphi_ {1}, \end{array}\tag{38}
$$

where $\begin{array} { r } { \varphi _ { 1 } = k _ { 1 3 } ^ { 2 } e _ { 1 3 } ^ { 2 } + \dot { \psi } _ { \mathrm { d } } ^ { 2 } + \frac { \gamma _ { \mathrm { c l } } + \gamma _ { \mathrm { a l } } } { 2 } \omega _ { 1 } ^ { * \mathrm { T } } B _ { 1 } \omega _ { 1 } ^ { * } \mathrm { ~ . ~ I f ~ } 0 ~ < ~ \gamma _ { \mathrm { c l } } ~ < ~ \gamma _ { \mathrm { a l } } ~ < ~ 2 \gamma _ { \mathrm { c l } } , } \end{array}$ $\begin{array} { r } { \beta _ { 1 } - \frac { 3 } { 2 } > 0 , 2 \gamma _ { \mathrm { a l } } - 1 > 0 , } \end{array}$ there is

$$
\dot {V} _ {1} \leq - \ell_ {1} V _ {1} + \varphi_ {1} + \frac {1}{2} \hat {z} _ {2} ^ {2} + y _ {r} ^ {2},\tag{39}
$$

where $\begin{array} { r } { \ell _ { 1 } = \operatorname* { m i n } \{ \ell _ { 1 1 } , \ell _ { 1 2 } \} , \ell _ { 1 1 } = 2 ( \beta _ { 1 } - \frac 3 \gamma ) , \ell _ { 1 2 } = \gamma _ { \mathrm { c l } } \lambda _ { \mathrm { m i n } } ( B _ { 1 } ) . } \end{array}$

Step 2: Based on (11), (14) and (35),

$$
\dot {\hat {z}} _ {2} = \dot {\hat {r}} + \frac {y _ {r}}{c _ {1}} = \hat {x} _ {3 3} + \tau_ {r m} + k _ {2 3} f _ {1 3} + \frac {y _ {r}}{c _ {1}},\tag{40}
$$

where $f _ { 1 3 } = \mathrm { s f a l } ( e _ { 1 3 } , h , q , \gamma ) , \hat { x } _ { 3 i }$ is the �-th element of $\hat { x } _ { 3 }$ , and $\tau _ { r m } =$ $\tau _ { r } / m _ { r }$

Remark 10. The actual yaw moment $\tau _ { r }$ is calculated by multiplying $\tau _ { r m }$ with $m _ { r } ,$ , where $m _ { r }$ denotes the third diagonal element of the inertia matrix �.

Define the optimal value function as:

$$
\begin{array}{c} J _ {2} ^ {*} (\hat {z} _ {2}) = \min _ {\tau_ {r m}} \int_ {t} ^ {\infty} h _ {2} (\hat {z} _ {2} (s), \tau_ {r m} (\hat {z} _ {2}))   \mathrm{d} s \\ = \int_ {t} ^ {\infty} h _ {2} (\hat {z} _ {2} (s), \tau_ {r m} ^ {*} (\hat {z} _ {2}))   \mathrm{d} s, \end{array}\tag{41}
$$

where $h _ { 2 } ( \hat { z } _ { 2 } ( s ) , \tau _ { r m } ^ { * } ( \hat { z } _ { 2 } ) ) = \hat { z } _ { 2 } ^ { 2 } + \tau _ { r m } ^ { * 2 } , \tau _ { r m } ^ { * }$ is the optimal control of $\tau _ { r m ^ { * } }$ Based on (40) and (41), HJB equation is

$$
\begin{array}{l} H _ {2} (\hat {z} _ {2}, \tau_ {r m} ^ {*}, \frac {\partial J _ {2} ^ {*}}{\partial \hat {z} _ {2}}) \\ = \hat {z} _ {2} ^ {2} + \tau_ {r m} ^ {* 2} + \frac {\partial J _ {2} ^ {*}}{\partial \hat {z} _ {2}} \big (\hat {x} _ {3 3} + \tau_ {r m} ^ {*} + k _ {2 3} f _ {1 3} + \frac {y _ {r}}{c _ {1}} \big). \end{array}\tag{42}
$$

By solving $\partial H _ { 2 } / \partial \tau _ { r m } ^ { * } = 0 ;$ the optimal control is

$$
\tau_ {r m} ^ {*} = - \frac {1}{2} \frac {\partial J _ {2} ^ {*}}{\partial \hat {z} _ {2}}.\tag{43}
$$

Similar to $( 2 2 ) , \ \frac { \partial J _ { 2 } ^ { * } } { \partial \hat { z } _ { 2 } }$ can be decomposed as:

$$
\frac {\partial J _ {2} ^ {*}}{\partial \hat {z} _ {2}} = 2 \beta_ {2} \hat {z} _ {2} + 2 \hat {x} _ {3 3} + J _ {2} ^ {\mathrm{o}} (\hat {z} _ {2}).\tag{44}
$$

where $\beta _ { 2 } > 0 ;$ , the unknown function $J _ { 2 } ^ { \mathrm { o } }$ can be approximated as:

$$
J _ {2} ^ {\mathrm{o}} (\hat {z} _ {2}) = \omega_ {2} ^ {* \mathrm{T}} \vartheta_ {2} (\hat {z} _ {2}) + \varepsilon_ {2} (\hat {z} _ {2}),\tag{45}
$$

in which $\omega _ { \gamma } ^ { \ast } \in \mathbb { R } ^ { w }$ is the ideal weight, $\pmb { \vartheta } _ { 2 } ( \hat { z } _ { 2 } ) \in \mathbb { R } ^ { w }$ is the basis function 2 vector, $\| \varepsilon _ { 2 } \bar { ( } \hat { z } _ { 2 } ) \| \ \leq \ \bar { \varepsilon } _ { 2 } , \ \bar { \varepsilon } _ { 2 } \ > \ 0 ,$ . In addition, the AC-RL framework is employed to deal with the unknown ideal weight �<sup>∗</sup> in (45):

$$
\hat {\tau} _ {r m} = - \beta_ {2} \hat {z} _ {2} - \hat {x} _ {3 3} - \frac {1}{2} \hat {\pmb {\omega}} _ {\mathrm{a2}} ^ {\mathrm{T}} \pmb {\vartheta} _ {2} (\hat {z} _ {2}),\tag{46}
$$

$$
\frac {\partial \hat {J} _ {2} ^ {*}}{\partial \hat {z} _ {2}} = 2 \beta_ {2} \hat {z} _ {2} + 2 \hat {x} _ {3 3} + \hat {\pmb {\omega}} _ {\mathrm{c2}} ^ {\mathrm{T}} \pmb {\vartheta} _ {2} (\hat {z} _ {2}),\tag{47}
$$

where $\hat { \tau } _ { r m }$ is the estimation of $\begin{array} { r } { \tau _ { r m } ^ { * } , \ \frac { \partial \hat { J } _ { 2 } ^ { * } } { \partial \hat { z } _ { 2 } } } \end{array}$ is the estimation of $\frac { \partial J _ { 2 } ^ { * } } { \partial \hat { z } _ { \gamma } } , \hat { \omega } _ { \mathrm { c 2 } }$ and $\hat { \pmb { \omega } } _ { \mathrm { a } 2 }$ are the AC weights satisfying the following updating laws:

$$
\dot {\hat {\boldsymbol {\omega}}} _ {\mathrm{c2}} = - \gamma_ {\mathrm{c2}} \pmb {B} _ {2} \hat {\boldsymbol {\omega}} _ {\mathrm{c2}},\tag{48}
$$

$$
\dot {\hat {\boldsymbol {\omega}}} _ {\mathrm{a} 2} = - \boldsymbol {B} _ {2} \left[ \gamma_ {\mathrm{a} 2} (\hat {\boldsymbol {\omega}} _ {\mathrm{a} 2} - \hat {\boldsymbol {\omega}} _ {\mathrm{c} 2}) + \gamma_ {\mathrm{c} 2} \hat {\boldsymbol {\omega}} _ {\mathrm{c} 2} \right],\tag{49}
$$

in which $B _ { 2 } = \vartheta _ { 2 } ( \hat { z } _ { 2 } ) \vartheta _ { 2 } ^ { \mathrm { T } } ( \hat { z } _ { 2 } ) + d _ { 2 } I _ { w } , d _ { 2 } > 0 , \gamma _ { \mathrm { c } 2 } > 0 , \gamma _ { \mathrm { a } 2 } > 0 .$ . Similar to Remark 8, the proposed AC-RL algorithm $( 4 6 ) \substack { - ( 4 9 ) }$ is effective.

Consider the Lyapunov function:

$$
V _ {2} = V _ {1} + \frac {1}{2} \hat {z} _ {2} ^ {2} + \frac {1}{2} \tilde {\omega} _ {\mathrm{c2}} ^ {\mathrm{T}} \tilde {\omega} _ {\mathrm{c2}} + \frac {1}{2} \tilde {\omega} _ {\mathrm{a2}} ^ {\mathrm{T}} \tilde {\omega} _ {\mathrm{a2}} + \frac {1}{2} y _ {r} ^ {2},\tag{50}
$$

where $\tilde { \pmb { \omega } } _ { \mathrm { c } 2 } = \hat { \pmb { \omega } } _ { \mathrm { c } 2 } - \pmb { \omega } _ { \gamma } ^ { * } , \tilde { \pmb { \omega } } _ { \mathrm { a } 2 } = \hat { \pmb { \omega } } _ { \mathrm { a } 2 } - \pmb { \omega } _ { \gamma } ^ { * }$ . Based on (35), (39), (40), (46), (48), (49) and similar to Step 1,

$$
\begin{array}{l} \dot {V} _ {2} \leq - \ell_ {1} V _ {1} - (\beta_ {2} - \frac {5}{4} - \frac {1}{2 c _ {1}}) \hat {z} _ {2} ^ {2} - (\frac {1}{2 c _ {1}} - \frac {3}{2}) y _ {r} ^ {2} \\ \quad - \frac {\gamma_ {\mathrm{c} 2}}{2} \tilde {\boldsymbol {\omega}} _ {\mathrm{c} 2} ^ {\mathrm{T}} \boldsymbol {B} _ {2} \tilde {\boldsymbol {\omega}} _ {\mathrm{c} 2} - \frac {2 \gamma_ {\mathrm{c} 2} - \gamma_ {\mathrm{a} 2}}{2} \hat {\boldsymbol {\omega}} _ {\mathrm{c} 2} ^ {\mathrm{T}} \boldsymbol {B} _ {2} \hat {\boldsymbol {\omega}} _ {\mathrm{c} 2} \\ \quad - \frac {\gamma_ {\mathrm{c} 2}}{2} \tilde {\boldsymbol {\omega}} _ {\mathrm{a} 2} ^ {\mathrm{T}} \boldsymbol {B} _ {2} \tilde {\boldsymbol {\omega}} _ {\mathrm{a} 2} - \frac {1}{8} \hat {\boldsymbol {\omega}} _ {\mathrm{a} 2} ^ {\mathrm{T}} (4 \gamma_ {\mathrm{a} 2} - 1) \boldsymbol {B} _ {2} \hat {\boldsymbol {\omega}} _ {\mathrm{a} 2} \\ \quad + \varphi_ {1} + \varphi_ {2 1}, \end{array}\tag{51}
$$

where $\begin{array} { r } { \varphi _ { 2 } = \varphi _ { 1 } + \varphi _ { 2 1 } , \varphi _ { 2 1 } = k _ { 2 3 } ^ { 2 } f _ { 1 3 } ^ { 2 } + \frac { 1 } { 2 } D _ { 1 } ^ { 2 } + \frac { \gamma _ { \mathrm { c } 2 } + \gamma _ { \mathrm { a } 2 } } { 2 } \omega _ { 2 } ^ { * \mathrm { T } } B _ { 2 } \omega _ { 2 } ^ { * } \cdot \mathrm { I f ~ } 0 < \gamma _ { \mathrm { c } 2 } < } \end{array}$ $\gamma _ { \mathrm { a 2 } } < 2 \gamma _ { \mathrm { c 2 } } , \ell _ { 2 1 } > 0 , \ell _ { 2 2 } > 0 , 4 \bar { \gamma _ { \mathrm { a 2 } } } - 1 > 0 ,$ , there is

$$
\dot {V} _ {2} \leq - \ell_ {2} V _ {2} + \varphi_ {2},\tag{52}
$$

$$
\begin{array}{l} \text { where } \ell_ {2} = \min \{\ell_ {1}, \ell_ {2 1}, \ell_ {2 2}, \ell_ {2 3} \}, \ell_ {2 1} = 2 \beta_ {2} - \frac {5}{2} - \frac {1}{c _ {1}}, \ell_ {2 2} = \frac {1}{c _ {1}} - 3, \\ \ell_ {2 3} = \gamma_ {\mathrm{c} 2} \lambda_ {\min} (\boldsymbol {B} _ {2}). \end{array}
$$

## 4.2. Surge force control

Step 1: Define the position tracking error $z _ { 3 }$ as

$$
z _ {3} = \sqrt {x _ {\mathrm{e}} ^ {2} + y _ {\mathrm{e}} ^ {2}},\tag{53}
$$

where $x _ { \mathrm { e } }$ and $y _ { \mathrm { e } }$ are defined in (14). Let $\psi _ { \mathrm { e } } = z _ { \mathrm { l } } ,$ based on (1), (2) and (14), the time derivative of $z _ { 3 }$ is given by

$$
\begin{array}{l} \dot {z} _ {3} = \frac {x _ {\mathrm{e}} \dot {x} _ {\mathrm{e}} + y _ {\mathrm{e}} \dot {y} _ {\mathrm{e}}}{\sqrt {x _ {\mathrm{e}} ^ {2} + y _ {\mathrm{e}} ^ {2}}} \\ = \frac {x _ {\mathrm{e}} \dot {x} + y _ {\mathrm{e}} \dot {y}}{\sqrt {x _ {\mathrm{e}} ^ {2} + y _ {\mathrm{e}} ^ {2}}} - \dot {x} _ {\mathrm{d}} \cos \psi_ {\mathrm{d}} - \dot {y} _ {\mathrm{d}} \sin \psi_ {\mathrm{d}} \\ = u \cos \psi_ {\mathrm{e}} - v \sin \psi_ {\mathrm{e}} - \dot {x} _ {\mathrm{d}} \cos \psi_ {\mathrm{d}} - \dot {y} _ {\mathrm{d}} \sin \psi_ {\mathrm{d}} \\ = \hat {u} \cos \psi_ {\mathrm{e}} - \xi + \kappa_ {1}, \end{array}\tag{54}
$$

where $\xi = v \sin \psi _ { \mathrm { e } } + \dot { x } _ { \mathrm { d } } \cos \psi _ { \mathrm { d } } + \dot { y } _ { \mathrm { d } } \sin \psi _ { \mathrm { d } }$ is bounded, ̂� is the estimation of $u , \kappa _ { 1 } = ( u - \hat { u } ) \cos \psi _ { \mathrm { e } }$ is the estimation error caused by ̂�.

Let ̂� be the optimal virtual control, i.e., $\hat { u } \triangleq \alpha _ { u } ^ { * }$ , the optimal value function is given by

$$
\begin{array}{c} J _ {3} ^ {*} (z _ {3}) = \min _ {\alpha_ {u}} \int_ {t} ^ {\infty} h _ {3} (z _ {3} (s), \alpha_ {u} (z _ {3})) \mathrm{d} s \\ = \int_ {t} ^ {\infty} h _ {3} (z _ {3} (s), \alpha_ {u} * (z _ {3})) \mathrm{d} s, \end{array}\tag{55}
$$

where $h _ { 3 } ( z _ { 3 } ( s ) , \alpha _ { u } ( z _ { 3 } ) ) \ = \ z _ { 3 } ^ { 2 } ( s ) + \alpha _ { u } ^ { 2 } ( z _ { 3 } ) .$ . According to (54), the HJB optimal performance equation can be expressed as:

$$
H _ {3} \left(z _ {3}, \alpha_ {u} ^ {*}, \frac {\partial J _ {3} ^ {*}}{\partial z _ {3}}\right) = z _ {3} ^ {2} + \alpha_ {u} ^ {* 2} + \frac {\partial J _ {3} ^ {*}}{\partial z _ {3}} \left(\alpha_ {u} ^ {*} \cos \psi_ {\mathrm{e}} - \xi + \kappa_ {1}\right).\tag{56}
$$

Since the optimal virtual control $\alpha _ { u } ^ { * }$ is unique satisfying (56), there is obtained by $\begin{array} { r } { \frac { \partial H _ { 3 } } { \partial \alpha _ { u } ^ { * } } = 0 } \end{array}$ as

$$
\alpha_ {u} ^ {*} = - \frac {1}{2} \frac {\partial J _ {3} ^ {*}}{\partial z _ {3}} \cos \psi_ {\mathrm{e}}.\tag{57}
$$

3 $( 2 2 ) , \begin{array} { r } { \frac { \partial J _ { 3 } ^ { * } } { \partial z _ { 3 } } } \end{array}$ Similar to can be decomposed as:

$$
\frac {\partial J _ {3} ^ {*}}{\partial z _ {3}} = 2 \beta_ {3} z _ {3} + J _ {3} ^ {0} (z _ {3}),\tag{58}
$$

where $\beta _ { 3 } > 0 ,$ , the unknown function $J _ { 3 } ^ { \mathrm { o } } ( z _ { 3 } )$ can be approximated as:

$$
J _ {3} ^ {\mathrm{o}} (z _ {3}) = \omega_ {3} ^ {* \mathrm{T}} \vartheta_ {3} (z _ {3}) + \varepsilon_ {3} (z _ {3}),\tag{59}
$$

where $\omega _ { \ast } ^ { \ast } \in \mathbb { R } ^ { w }$ denotes the ideal weight vector, $\pmb { \vartheta } _ { 3 } ( z _ { 3 } ) ~ \in ~ \mathbb { R } ^ { w }$ is the basis function vector, $\varepsilon _ { 3 } ( z _ { 3 } )$ is the approximation error satisfying $\| \varepsilon _ { 3 } ( z _ { 3 } ) \| \ \leq \ \bar { \varepsilon } _ { 3 }$ . The AC-RL framework is employed to deal with the unknown ideal weight �<sup>∗</sup> in (59).

$$
\frac {\partial \hat {J} _ {3} ^ {*}}{\partial z _ {3}} = 2 \beta_ {3} z _ {3} + \hat {\pmb {\omega}} _ {\mathrm{c3}} ^ {\mathrm{T}} \pmb {\vartheta} _ {3} (z _ {3}),\tag{60}
$$

$$
\hat {\alpha} _ {u} = \left(- \beta_ {3} z _ {3} - \frac {1}{2} \hat {\pmb {\omega}} _ {\mathrm{a3}} ^ {\mathrm{T}} \pmb {\vartheta} _ {3} (z _ {3})\right) \cos \psi_ {\mathrm{e}},\tag{61}
$$

where $\hat { \alpha } _ { u }$ is the estimation of $\alpha _ { u } ^ { * } , \ \frac { \partial \hat { J } _ { 3 } ^ { * } } { \partial z _ { 3 } }$ is the estimation of $\underline { { \frac { \partial J _ { 3 } ^ { * } } { \partial z _ { 3 } } } } , \hat { \boldsymbol { \omega } } _ { \mathrm { c 3 } }$ and $\hat { \omega } _ { a 3 }$ are the AC weights satisfying the following updating laws:

$$
\dot {\hat {\omega}} _ {\mathrm{c3}} = - \gamma_ {\mathrm{c3}} B _ {3} \hat {\omega} _ {\mathrm{c3}},\tag{62}
$$

$$
\dot {\hat {\omega}} _ {\mathrm{a3}} = - \mathcal {B} _ {3} [ \gamma_ {\mathrm{a3}} (\hat {\omega} _ {\mathrm{a3}} - \hat {\omega} _ {\mathrm{c3}}) + \gamma_ {\mathrm{c3}} \hat {\omega} _ {\mathrm{c3}} ],\tag{63}
$$

where $B _ { 3 } = \vartheta _ { 3 } ( z _ { 3 } ) \vartheta _ { 3 } ^ { \mathrm { T } } ( z _ { 3 } ) + d _ { 3 } I _ { w } , d _ { 3 } > 0 , \gamma _ { \mathrm { c 3 } } > 0 , \gamma _ { \mathrm { a 3 } } > 0 .$ . Similar to Remark $^ { 8 , }$ the proposed AC-RL algorithm $( 6 0 ) \substack { - ( 6 3 ) }$ is effective.

Similar to Remark 9, define the error variable $\hat { z } _ { 4 }$ and the boundary layer error $y _ { u }$ as:

$$
\hat {z} _ {4} = \hat {u} - \alpha_ {\mathrm{f} u}, y _ {u} = \alpha_ {\mathrm{f} u} - \hat {\alpha} _ {u},\tag{64}
$$

where $\alpha _ { \mathrm { f } u }$ is the filtered signal. The first-order filter is

$$
c _ {2} \dot {\alpha} _ {\mathrm{f} u} + \alpha_ {\mathrm{f} u} = \hat {\alpha} _ {u}, \alpha_ {\mathrm{f} u} (0) = \hat {\alpha} _ {u} (0),\tag{65}
$$

where $c _ { 2 } > 0 .$ Based on (54) and (64), there is

$$
\dot {z} _ {3} = \left(\hat {z} _ {4} + y _ {u} + \hat {\alpha} _ {u}\right) \cos \psi_ {\mathrm{e}} - \xi + \kappa_ {1},\tag{66}
$$

where the boundary layer error $y _ { u }$ satisfies

$$
\dot {y} _ {u} = - \frac {y _ {u}}{c _ {2}} - D _ {2} (\cdot),\tag{67}
$$

where $D _ { 2 } ( \cdot ) \triangleq \dot { \hat { \alpha } } _ { u }$ is a nonlinear continuous function involving system signals.

Consider the following Lyapunov function:

$$
V _ {3} = \frac {1}{2} z _ {3} ^ {2} + \frac {1}{2} \tilde {\omega} _ {\mathrm{c3}} ^ {\mathrm{T}} \tilde {\omega} _ {\mathrm{c3}} + \frac {1}{2} \tilde {\omega} _ {\mathrm{a3}} ^ {\mathrm{T}} \tilde {\omega} _ {\mathrm{a3}},\tag{68}
$$

where $\tilde { \omega } _ { \mathrm { c 3 } } = \hat { \omega } _ { \mathrm { c 3 } } - \omega _ { \mathrm { 3 } } ^ { * } , \tilde { \omega } _ { \mathrm { a 3 } } = \hat { \omega } _ { \mathrm { a 3 } } - \omega _ { \mathrm { 3 } } ^ { * }$ . Based on (61)–(63), (66) and similar to (51), there is

$$
\begin{array}{r l} \dot {V} _ {3} \leq - (\beta_ {3} \cos^ {2} \psi_ {\mathrm{e}} - \frac {9}{4}) z _ {3} ^ {2} + \frac {1}{2} \gamma_ {u} ^ {2} + \frac {1}{2} \hat {z} _ {4} ^ {2} + \frac {1}{2} \xi^ {2} + \frac {1}{2} \kappa_ {1} ^ {2} \\ & - \frac {\gamma_ {\mathrm{c} 3}}{2} \tilde {\omega} _ {\mathrm{c} 3} ^ {\mathrm{T}} \boldsymbol {B} _ {3} \tilde {\omega} _ {\mathrm{c} 3} - \frac {\gamma_ {\mathrm{c} 3}}{2} \tilde {\omega} _ {\mathrm{a} 3} ^ {\mathrm{T}} \boldsymbol {B} _ {3} \tilde {\omega} _ {\mathrm{a} 3} \\ & - \frac {2 \gamma_ {\mathrm{c} 3} - \gamma_ {\mathrm{a} 3}}{2} \hat {\omega} _ {\mathrm{c} 3} ^ {\mathrm{T}} \boldsymbol {B} _ {3} \hat {\omega} _ {\mathrm{c} 3} + \frac {\gamma_ {\mathrm{c} 3} + \gamma_ {\mathrm{a} 3}}{2} \boldsymbol {\omega} _ {3} ^ {* \mathrm{T}} \boldsymbol {B} _ {3} \boldsymbol {\omega} _ {3} ^ {*} \\ & - \left(\frac {\gamma_ {\mathrm{a} 3}}{2} - \frac {1}{4} \cos^ {2} \psi_ {\mathrm{e}}\right) \hat {\omega} _ {\mathrm{a} 3} ^ {\mathrm{T}} \boldsymbol {B} _ {3} \hat {\omega} _ {\mathrm{a} 3}, \end{array}\tag{69}
$$

where $\begin{array} { r } { \varphi _ { 3 } = \frac { 1 } { 2 } \xi ^ { 2 } + \frac { 1 } { 2 } \kappa _ { 1 } ^ { 2 } + \frac { \gamma _ { \mathrm { c } 3 } + \gamma _ { \mathrm { a } 3 } } { 2 } \omega _ { 3 } ^ { * \mathrm { T } } B _ { 3 } \omega _ { 3 } ^ { * } . \mathrm { ~ I f ~ } 0 < \gamma _ { \mathrm { c } 3 } < \gamma _ { \mathrm { a } 3 } < 2 \gamma _ { \mathrm { c } 3 } , \gamma _ { \mathrm { a } 3 } > } \end{array}$ ${ \textstyle \frac { 1 } { 2 } } \cos ^ { 2 } \psi _ { \mathrm { e } } , \beta _ { 3 } > ^ { ^ { \angle } } { \frac { 9 } { 4 \cos ^ { 2 } \psi _ { \mathrm { e } } } } ,$ , there is

$$
\dot {V} _ {3} \leq - \ell_ {3} V _ {3} + \varphi_ {3} + \frac {1}{2} y _ {u} ^ {2} + \frac {1}{2} \hat {z} _ {4} ^ {2},\tag{70}
$$

$$
\text { where } \ell_ {3} = \min \{\ell_ {3 1}, \ell_ {3 2} \}, \ell_ {3 1} = 2 \left(\beta_ {3} \cos^ {2} \psi_ {\mathrm{e}} - \frac {9}{4}\right), \ell_ {3 2} = \gamma_ {\mathrm{c} 3} \lambda_ {\min} (\boldsymbol {B} _ {3}).
$$

Step 2: Based on (11) and (64),

$$
\dot {\hat {z}} _ {4} = \hat {x} _ {3 1} \cos \psi + \hat {x} _ {3 2} \sin \psi + \tau_ {u m} + \kappa_ {2} + \frac {y _ {u}}{c _ {2}},\tag{71}
$$

where $\kappa _ { 2 } = k _ { 2 }$ cos $\upsilon f _ { 1 1 } + k _ { 2 }$ sin $\boldsymbol { \prime } f _ { 1 2 } , f _ { 1 i } = \mathrm { s f a l } ( e _ { 1 i } , h , q , \gamma ) , \boldsymbol { \hat { x } _ { 3 i } }$ is the �-th element of ${ \hat { x } } _ { 3 } ,$ and $\tau _ { u m } = \tau _ { u } / m _ { u }$

Remark 11. The actual surge moment $\tau _ { u }$ is calculated by multiplying $\tau _ { u m }$ with $m _ { u } ,$ , where $m _ { u }$ denotes the first diagonal element of the inertia matrix �.

Define the optimal value function as

$$
J _ {4} ^ {*} (\hat {z} _ {4}) = \min _ {\tau_ {u m}} \int_ {t} ^ {\infty} h _ {4} (\hat {z} _ {4} (s), \tau_ {u m} (\hat {z} _ {4})) \mathrm{d} s
$$

$$
= \int_ {t} ^ {\infty} h _ {4} (\hat {z} _ {4} (s), \tau_ {u m} ^ {*} (\hat {z} _ {4})) \mathrm{d} s,\tag{72}
$$

with $h _ { 4 } ( \hat { z } _ { 4 } ( s ) , \tau _ { u m } ^ { * } ( \hat { z } _ { 4 } ) ) = \hat { z } _ { 4 } ^ { 2 } + \tau _ { u m } ^ { * 2 } ;$ , where $\tau _ { u m } ^ { * }$ is the optimal control. Based on (71) and (72), the HJB equation is

$$
H _ {4} (\hat {z} _ {4}, \tau_ {u m} ^ {*}, \frac {\partial J _ {4} ^ {*}}{\partial \hat {z} _ {4}}) = \hat {z} _ {4} ^ {2} + \tau_ {u m} ^ {* 2} + \frac {\partial J _ {4} ^ {*}}{\partial \hat {z} _ {4}}
$$

$$
\times \left(\hat {x} _ {3 1} \cos \psi + \hat {x} _ {3 2} \sin \psi + \tau_ {u m} ^ {*} + \kappa_ {2} + \frac {y _ {u}}{c _ {2}}\right).\tag{73}
$$

By $\partial H _ { 4 } / \partial \tau _ { u m } ^ { * } = 0 ;$ , the optimal control can be obtained as

$$
\tau_ {u m} ^ {*} = - \frac {1}{2} \frac {\partial J _ {4} ^ {*}}{\partial \hat {z} _ {4}}.\tag{74}
$$

Similar to (22), 2 � ̂� ��<sup>∗</sup><sub>4</sub> can be decomposed as: � ̂�

$$
\frac {\partial J _ {4} ^ {*}}{\partial \hat {z} _ {4}} = 2 \beta_ {4} \hat {z} _ {4} + 2 \hat {x} _ {3 1} \cos \psi + 2 \hat {x} _ {3 2} \sin \psi + J _ {4} ^ {0} (\hat {z} _ {4}),\tag{75}
$$

where $\beta _ { 4 } > 0 .$ . the unknown function $J _ { 4 } ^ { \mathrm { o } } ( \hat { z } _ { 4 } )$ can be approximated by NNs as

$$
J _ {4} ^ {\mathrm{o}} (\hat {z} _ {4}) = \omega_ {4} ^ {* \mathrm{T}} \vartheta_ {4} (\hat {z} _ {4}) + \varepsilon_ {4} (\hat {z} _ {4}),\tag{76}
$$

where $\omega _ { 4 } ^ { * }$ ∈ R<sup>�</sup> is the ideal weight, $\pmb { \vartheta } _ { 4 } ( \hat { z } _ { 4 } ) \in \mathbb { R } ^ { w }$ is the basis function vector, $\| \dot { \varepsilon } _ { 4 } ( \hat { z } _ { 4 } ) \| \leq \bar { \varepsilon } _ { 4 } , \bar { \varepsilon } _ { 4 } > 0 .$ The AC-RL framework is employed to deal with the unknown ideal weight �<sup>∗</sup> in (76).

$$
\frac {\partial \hat {J} _ {4} ^ {*}}{\partial \hat {z} _ {4}} = 2 \beta_ {4} \hat {z} _ {4} + 2 \hat {x} _ {3 1} \cos \psi + 2 \hat {x} _ {3 2} \sin \psi + \hat {\omega} _ {\mathrm{c4}} ^ {\mathrm{T}} \boldsymbol {\vartheta} _ {4} (\hat {z} _ {4}),\tag{77}
$$

$$
\hat {\tau} _ {u m} = - \beta_ {4} \hat {z} _ {4} - \hat {x} _ {3 1} \cos \psi - \hat {x} _ {3 2} \sin \psi - \frac {1}{2} \hat {\pmb {\omega}} _ {a 4} ^ {\mathrm{T}} \pmb {\vartheta} _ {4} (\hat {z} _ {4}),\tag{78}
$$

where $\hat { \tau } _ { u m }$ is the estimation of $\begin{array} { r } { \tau _ { u m } ^ { * } , ~ \frac { \partial \hat { J } _ { 4 } ^ { * } } { \partial \hat { z } _ { 4 } } } \end{array}$ is the estimation of $\frac { \partial J _ { 4 } ^ { * } } { \partial \hat { z } _ { 4 } } , \hat { \omega } _ { \mathrm { c 4 } }$ and $\hat { \pmb { \omega } } _ { \mathrm { a 4 } }$ are the AC weights satisfying the following updating laws:

$$
\dot {\hat {\omega}} _ {\mathrm{c4}} = - \gamma_ {\mathrm{c4}} \mathcal {B} _ {4} \hat {\omega} _ {\mathrm{c4}},\tag{79}
$$

$$
\dot {\hat {\boldsymbol {\omega}}} _ {\mathrm{a} 4} = - \mathcal {B} _ {4} \left[ \gamma_ {\mathrm{a} 4} (\hat {\boldsymbol {\omega}} _ {\mathrm{a} 4} - \hat {\boldsymbol {\omega}} _ {\mathrm{c} 4}) + \gamma_ {\mathrm{c} 4} \hat {\boldsymbol {\omega}} _ {\mathrm{c} 4} \right].\tag{80}
$$

where $B _ { 4 } = \vartheta _ { 4 } ( \hat { z } _ { 4 } ) \vartheta _ { 4 } ^ { \Gamma } ( \hat { z } _ { 4 } ) + d _ { 4 } I _ { w } , d _ { 4 } > 0 , \gamma _ { \mathrm { c } 4 } > 0 , \gamma _ { \mathrm { a } 4 } > 0 .$ . Similar to Remark 8, the proposed AC-RL algorithm $( 7 7 ) \textmd { - } ( 8 0 )$ is effective.

Consider the following Lyapunov function:

$$
V _ {4} = V _ {3} + \frac {1}{2} \hat {z} _ {4} ^ {\mathrm{T}} \hat {z} _ {4} + \frac {1}{2} \tilde {\omega} _ {\mathrm{c4}} ^ {\mathrm{T}} \tilde {\omega} _ {\mathrm{c4}} + \frac {1}{2} \tilde {\omega} _ {\mathrm{a4}} ^ {\mathrm{T}} \tilde {\omega} _ {\mathrm{a4}} + \frac {1}{2} y _ {u} ^ {2},\tag{81}
$$

where $\tilde { \omega } _ { \mathrm { c 4 } } = \hat { \omega } _ { \mathrm { c 4 } } - \omega _ { \mathrm { 4 } } ^ { * } , \tilde { \omega } _ { \mathrm { a 4 } } = \hat { \omega } _ { \mathrm { a 4 } } - \omega _ { \mathrm { 4 } } ^ { * }$ . Based on (78)–(80) and similar to (51),

$$
\begin{array}{r l} & {\dot {V} _ {4} \leq - \ell_ {3} V _ {3} + \varphi_ {3} - (\beta_ {4} - 1 - \frac {1}{2 c _ {2}}) \hat {z} _ {4} ^ {2} - (\frac {1}{2 c _ {2}} - \frac {3}{4}) y _ {u} ^ {2}} \\ & {- \frac {\gamma_ {\mathrm{c} 4}}{2} \tilde {\omega} _ {\mathrm{c} 4} ^ {\mathrm{T}} \pmb {B} _ {4} \tilde {\omega} _ {\mathrm{c} 4} - \frac {\gamma_ {\mathrm{c} 4}}{2} \tilde {\omega} _ {\mathrm{a} 4} ^ {\mathrm{T}} \pmb {B} _ {4} \tilde {\omega} _ {\mathrm{a} 4} + \varphi_ {4 1}} \\ & {- \frac {2 \gamma_ {\mathrm{c} 4} - \gamma_ {\mathrm{a} 4}}{2} \hat {\omega} _ {\mathrm{c} 4} ^ {\mathrm{T}} \pmb {B} _ {4} \hat {\omega} _ {\mathrm{c} 4} - (\frac {\gamma_ {\mathrm{a} 4}}{2} - \frac {1}{4}) \hat {\omega} _ {\mathrm{a} 4} ^ {\mathrm{T}} \pmb {B} _ {4} \hat {\omega} _ {\mathrm{a} 4},} \end{array}\tag{82}
$$

where $\begin{array} { r } { \varphi _ { 4 } = \varphi _ { 3 } + \varphi _ { 4 1 } , \varphi _ { 4 1 } = \kappa _ { 2 } ^ { 2 } + D _ { 2 } ^ { 2 } + \frac { \gamma _ { a 4 } + \gamma _ { c 4 } } { 2 } \omega _ { 4 } ^ { * \mathrm { T } } B _ { 4 } \omega _ { 4 } ^ { * } \cdot \mathrm { I f ~ } 0 < \gamma _ { c 4 } < \gamma _ { a 4 } < \gamma _ { c 4 } , } \end{array}$ $\begin{array} { r } { 2 \gamma _ { \mathrm { c } 4 } , \gamma _ { \mathrm { a } 4 } > \frac { 1 } { 2 } , \beta _ { 4 } > 1 + \frac { 1 } { 2 c _ { \mathscr { D } } } , 0 < c _ { 2 } < \frac { 2 } { 3 } , } \end{array}$ , there is

$$
\dot {V} _ {4} \leq - \ell_ {4} V _ {4} + \varphi_ {4},\tag{83}
$$

$$
\begin{array}{l} \text { where } \ell_ {4} = \min \{\ell_ {3}, \ell_ {4 1}, \ell_ {4 2}, \ell_ {4 3} \}, \ell_ {4 1} = 2 \left(\beta_ {4} - 1 - \frac {1}{2 c _ {2}}\right), \ell_ {4 2} = 2 \\ \left(\frac {1}{2 c _ {2}} - \frac {3}{4}\right), \ell_ {4 3} = \gamma_ {\mathrm{c} 4} \lambda_ {\min} (\boldsymbol {B} _ {4}). \end{array}
$$

Theorem 2. Under Assumptions 1–2 and the constructed ESO in (11), if the parameters satisfy:

$$
\beta_ {1} > \frac {3}{2}, 2 \gamma_ {\mathrm{cl}} > \gamma_ {\mathrm{al}} > \gamma_ {\mathrm{cl}} > 0, \gamma_ {\mathrm{al}} > \frac {1}{2},\tag{84}
$$

$$
\beta_ {2} > \frac {3}{4} + \frac {1}{2 c _ {1}}, c _ {1} <   \frac {1}{3}, 2 \gamma_ {\mathrm{c} 2} > \gamma_ {\mathrm{a} 2} > \gamma_ {\mathrm{c} 2} > 0, \gamma_ {\mathrm{a} 2} > \frac {1}{4},\tag{85}
$$

$$
\beta_ {3} > \frac {9}{4 \cos^ {2} \psi_ {\mathrm{e}}}, 2 \gamma_ {\mathrm{c} 3} > \gamma_ {\mathrm{a} 3} > \gamma_ {v} c 3 > 0, \gamma_ {\mathrm{a} 3} > \frac {1}{2} \cos^ {2} \psi_ {\mathrm{e}},\tag{86}
$$

$$
\beta_ {4} > 1 + \frac {1}{2 c _ {2}}, 0 <   c _ {2} <   \frac {2}{3}, 2 \gamma_ {\mathrm{c} 4} > \gamma_ {\mathrm{a} 4} > \gamma_ {\mathrm{c} 4} > 0, \gamma_ {\mathrm{a} 4} > \frac {1}{2},\tag{87}
$$

then, the designed AOTC in (23), (57), (46), (78) with the updates in (26), (48), (62), (79), (28), (49), (63), (80) can achieve that all error signals $\hat { z } _ { i } , ~ \tilde { \pmb { \omega } } _ { \mathrm { c } i } , ~ \tilde { \pmb { \omega } } _ { \mathrm { a } i } , ~ e _ { j }$ are semi-globally uniformly ultimately bounded, and the USV can track the predefined trajectory even in the presence of partially unmeasurable states.

Proof. The Lyapunov candidate is chosen as:

$$
V = V _ {2} + V _ {4}
$$

$$
= \frac {1}{2} \sum_ {i = 1} ^ {4} (\hat {z} _ {i} ^ {2} + \frac {1}{2} \tilde {\omega} _ {c i} ^ {T} \tilde {\omega} _ {c i} + \frac {1}{2} \tilde {\omega} _ {a i} ^ {T} \tilde {\omega} _ {a i}) + \frac {1}{2} y _ {r} ^ {2} + \frac {1}{2} y _ {u} ^ {2},\tag{88}
$$

substituting (39), (51), (69) and (82) into (88), there is

$$
\dot {V} \leq \ell V + \varphi ,\tag{89}
$$

where $\ell = \operatorname* { m i n } \{ \ell _ { 2 } , \ell _ { 4 } \} , \varphi = \varphi _ { 2 } + \varphi _ { 4 } , \ell _ { 2 }$ and $\varphi _ { 2 }$ are defined in (52), $\ell _ { 4 }$ and $\varphi _ { 4 }$ are defined in (83). Then,

$$
V (t) \leq e ^ {\ell} V (0) + \frac {\varphi}{\ell} (1 - e ^ {- \ell t}).\tag{90}
$$

The proof is completed. □

Remark 12. The condition $\beta _ { 3 } > \frac { 9 } { 4 \cos ^ { 2 } \psi _ { \mathrm { e } } }$ in (86) implies the practical limitations on heading deviation. When $\dot { | } \psi _ { \mathrm { e } } | = 9 0 ^ { \circ }$ , this is theoretically challenged. However, this critical condition rarely persists in physical systems due to the inherent inertia and continuous control adjustments of USVs. the momentum of USVs naturally prevents sustained orthog onal positioning relative to the desired trajectory, ensuring transient zero-crossings of cos � rather than persistent violations. Consequently, the Lyapunov function maintains the negative definiteness for dominant operational periods, guaranteeing asymptotic error convergence despite temporary singularity.

## 5. Simulation

To demonstrate the effectiveness of the proposed algorithm, a simulation is conducted by $\mathrm { ~ a ~ \ 1 ~ : ~ } \mathit { \Theta } 6 7$ scaled-down model of USV, h h h f $m \ = \ 2 7$ kg, a length of 1.2 m and a width of 0.3 m. The inertia, Coriolis centripetal and damping matrices are $M =$ $[ 2 5 , 0 , 0 ; 0 , 2 4 , 0 ; 0 , 0 , 3 . 7 ] , C = \left[ 0 , 0 , - 2 4 b _ { v } - 0 . 7 2 b _ { r } ; 0 , 0 , 2 5 b _ { u } ; 2 4 b _ { v } + 0 . 7 2 b _ { r } , 0 , 0 , 0 , 1 , 0 , 0 , 0 \right]$ $- 2 5 b _ { u } , 0 \Big | , D = [ 0 . 7 2 + 1 . 8 | b _ { u } | \overset { \cdot } { + } 6 . 3 b _ { u } ^ { 2 } , 0 , 0 ; 0 , 0 . 8 6 + 4 1 | b _ { v } | + 3 . 5 | b _ { r } | , - 0 . 1 - 0 . 3 | b _ { u } | ] ,$ $2 . 5 | b _ { v } | + 2 . 5 | b _ { r } | ; 0 , - 0 . 1 - 5 . 5 | b _ { v } | + 3 . 5 | b _ { r } | , 6 + 4 . 5 | b _ { v } | + 4 . 5 | b _ { r } | ]$ . The initial states of position and velocity are respectively given as $\pmb { a } ( 0 ) = [ 0 , 0 , 0 ] ^ { \mathrm { T } }$ and $\boldsymbol { b } ( 0 ) = [ 0 , 0 , 0 ] ^ { \mathrm { T } }$ . The parameters of the ESO are chosen as follows: $k _ { 1 } \ = \ \mathrm { d i a g } ( 5 , 5 , 5 ) , \ k _ { 2 } \ = \ \mathrm { d i a g } ( 8 , 8 , 8 ) , \ k _ { 3 } \ = \ \mathrm { d i a g } ( 1 0 , 1 0 , 1 0 ) , \ \gamma \ = \ 0 . 0 0 5 ,$ $h _ { 1 } = h _ { 2 } = 0 . 5 ,$ , and $q _ { 1 } = q _ { 2 } = 0 . 0 0 1$

Step 1 of 4.1: The number of neurons is 30 with centers $\mu _ { i }$ evenly spaced in the range $[ - 6 , 6 ]$ and widths $\phi _ { i } ~ = ~ 2 .$ . The parameters are designed as: � (0) = [0.2, 0.2 … 0.2]<sup>T</sup> ∈ R<sup>30×1</sup> $, \pmb { \omega } _ { \mathrm { a l } } ( 0 ) = [ 0 . 1 , 0 . 1 \ldots 0 . 1 ] ^ { \mathrm { T } }$ $\in \mathbb { R } ^ { 3 0 \times 1 } , \gamma _ { \mathrm { c l } } = 1 , \gamma _ { \mathrm { a l } } = 8 , c _ { 1 } = 0 . 1 , d _ { 1 } = 0 . 0 1$ and $\beta _ { 1 } = 6$

Step 2 of 4.1: Choose the same NNs as Step 1 of 4.1. The parameters are designed as: $\omega _ { \mathrm { c } 2 } ( 0 ) = [ 0 . 2 , 0 . 2 \dots 0 . 2 ] ^ { \mathrm { T } } \in \mathbb { R } ^ { 3 0 \times 1 } , \omega _ { \mathrm { a } 2 } ( 0 ) =$ $[ 0 . 1 , 0 . 1 \dots 0 . 1 ] ^ { \mathrm { T } } \in \mathbb { R } ^ { 3 0 \times 1 } , \gamma _ { \mathrm { c } 2 } = 2 . 7 , \gamma _ { \mathrm { a } 2 } = 1 0 , d _ { 2 } = 0 . 0 1 \mathrm { ~ a n d ~ } \beta _ { 2 } = 1 . 2$

Step 1 of $4 . 2 \colon$ Choose the same NNs as Step 1 of 4.1. The parameters are designed as: $\omega _ { \mathrm { c 3 } } ( 0 ) = [ 0 . 2 , 0 . 2 \dots 0 . 2 ] ^ { \mathrm { T } } \in \mathbb { R } ^ { 3 0 \times 1 } , \omega _ { \mathrm { a 3 } } ( 0 ) =$ $[ 0 . 1 , 0 . 1 \dots 0 . 1 ] ^ { \mathrm { T } } \in \mathbb { R } ^ { 3 0 \times 1 } , \gamma _ { \mathrm { c } 3 } = 3 , \gamma _ { \mathrm { a } 3 } = 1 2 , c _ { 2 } = 0 . 1 , d _ { 3 } = 0 . 0 1$ and $\beta _ { 3 } = 5 . 5$

Step 2 of 4.2: Choose the same NNs as Step 1 of 4.1. The parameters are designed as: $\omega _ { \mathrm { c } 4 } ( 0 ) = [ 0 . 2 , 0 . 2 \dots 0 . 2 ] ^ { \mathrm { T } } \in \mathbb { R } ^ { 3 0 \times 1 } , \omega _ { \mathrm { a } 4 } ( 0 ) =$ $[ 0 . 1 , 0 . 1 \dots 0 . 1 ] ^ { \mathrm { T } } \ \in \ \mathbb { R } ^ { 3 0 \times 1 } , \ \gamma _ { \mathrm { c } 4 } \ = \ 2 . 7 , \ \gamma _ { \mathrm { a } 4 } \ = \ 1 0 , \ d _ { 4 } \ = \ 0 . 0 1$ and $\beta _ { 4 } \ =$ 1.2. To further demonstrate the practical advantage of the proposed control strategy, we also conduct comparative simulations using a benchmark method—RLOTC [34]. The same model settings and refer ence trajectories are used to ensure fair comparison in both simulation environments.

Simulation Scenario 1: In the first scenario, the desired reference signal is ${ \pmb a } _ { \mathrm { d } } = [ 3 0 \sin ( 0 . 0 1 \pi t )$ 30 sin(0.01πt −

performance of the proposed ESO-based RL optimal tracking control approach under partially unobservable conditions, a comparative simulation experiment is conducted against the RLOTC method, which assumes full state availability. Figs. 2–3 present the state observation performance of the ESO, which is a unique component of our proposed approach. From the beginning of the operation, the ESO can closely track the actual states of the USV in real time, demonstrating strong robustness and estimation accuracy, even under initial uncertainty. To further assess tracking performance, Figs. 4–7 provide a comprehensive comparison between the proposed method and the RLOTC method. As shown in Fig. 4, both methods enable the USV to follow the desired trajectory, but the proposed method achieves slightly better trajectory tracking accuracy. This advantage is particularly evident in Fig. 5, where the distance tracking error $z _ { 3 }$ of our method remains consistently lower throughout the simulation. Fig. 6 illustrates the control input signals. It is noted that the proposed controller tends to apply a stronger thrust in the surge direction $\tau _ { u }$ at the beginning, which leads to a slightly higher cost during the early phase (as shown in Fig. 7). However, the cost gradually stabilizes and becomes comparable to that of the RLOTC method over time. Finally, Fig. 8 show the norms of the critic and actor NN weights under the proposed method, indicating smooth and bounded learning behavior through out the process. In summary, although both methods perform well the proposed approach demonstrates superior tracking accuracy while operating under more challenging conditions (i.e., partial observability). This highlights the practical applicability and robustness of the proposed control framework.

![](images/8948b19260b1aa2695ce29f87c3912e03bd43c1a83c24511a5b6aa5766db25b8.jpg)  
Fig. 2. The actual position state and its observed value in Scenario 1.

![](images/4a6c25219b984fcd6c824b75b06abfda6b553b519576827d54bfd13bfdbedbd0.jpg)  
Fig. 3. The actual velocity state and its observed value in Scenario 1.

![](images/4d1b6c96ac5a8cb86331f2bbd96625f24615b6be6be973ee3fb36f2288fa0f05.jpg)  
Fig. 4. Vessel position (�, �) tracks the reference trajectory $( x _ { \mathrm { d } } , y _ { \mathrm { d } } )$ in Scenario 1.

![](images/785b292f1ff4463d9c7ddc99e0e0f7ca1b9b036ef6e24cd5de8c2955a18359d5.jpg)

![](images/540e54481632b714e580b7170fa74352f1b66d2561013d82390fdb40d4e0beb3.jpg)  
Fig. 5. Tracking errors of $z _ { 1 }$ and $z _ { 3 }$ in Scenario 1.

![](images/90ee54969508aebfa78d6dd30a0e769717b1ded211ddb8fe443842e7a7e8cf74.jpg)

![](images/67a3bb64ad142676c7008bd61b445379f4b3a84b3e00b808aeaeebf4c3fcbf69.jpg)  
Fig. 6. Control input in Scenario 1.

Simulation Scenario 2: In the second scenario, the desired reference signal is $a _ { \mathrm { d } } = [ 3 0$ sin(0.01π�), 30 sin(0.01π�) ∗ cos(0.01π�), atan2(cos(��)(1 − $2 \sin ^ { 2 } ( \omega t ) ) , \cos ( 2 \omega t ) ) ] ^ { \mathrm { T } }$ . To rigorously evaluate our proposed method, we continue to conduct comparative experiments against the RLOTC approach, focusing on its ability to accurately and stably track this intricate trajectory. Figs. 9–10 illustrate the observation performance of the ESO under this new environment. Similar to Scenario 1, the ESO demonstrates rapid convergence and high estimation accuracy, with minimal steady-state error. These results once again confirm the reliability and robustness of the observer in handling partially unobservable system dynamics. Figs. 11–14 present the performance comparison between the proposed method and the RLOTC approach. As shown in Fig. 11, both methods achieve comparable path-following behavior. In terms of tracking errors (Fig. 12), the heading error $z _ { 1 }$ remains similar across both methods. Although the distance error $z _ { 3 }$ under the proposed method shows a slight increase in this more challenging scenario, such behavior is expected due to the partial unobservability of the system states. Nevertheless, the error remains within a controllable and acceptable range, demonstrating the robustness of the proposed approach. The control input comparison in Fig. 13 reveals that the proposed method applies less surge force $\tau _ { u }$ during the initial stage, leading to lower energy expenditure at startup. Over time, both meth ods converge to similar control behaviors. As a result, the proposed approach achieves a more favorable cost performance in the early phase, as evident in Fig. 14, where its cost is significantly lower than that of RLOTC at the beginning and remains competitive thereafter. Fi ll ll h b h f h d NN weights under the proposed framework. The stable and bounded evolution of the weights confirms the learning stability and efficiency of the adaptive control scheme. Overall, while both methods perform effectively, the proposed ESO-based RL controller continues to exhibit strong adaptability and control economy even under more complex path-tracking demands. Importantly, these results are achieved under partial state observability, further highlighting the practical value and robustness of the proposed approach.

![](images/8c8257107bb4a55f0b237cd1e9875cf3d4f819a9c2d12c5efb32ef2e58d03180.jpg)

![](images/a539cc9a080de18d3257eb9e3c56681d9e86a2431c60b13c247dacdb43efc297.jpg)

![](images/61902622ef86396a768dc8847b0836868edc886bff07d2554a1ee1c7e612d1d6.jpg)

![](images/9150785cd0e88dd64ffa81cb2d6080568308648ac44c320227b0a8bcc95400dc.jpg)  
Fig. 7. Cost functions in Scenario 1.

![](images/31ae80641b36f266abfdcd67edcc546bcd87550118eff29161edd210789801c5.jpg)

![](images/1df93c321fe2e3f810c63577a6402b9abb34be59dbbd02334c7af2197468c6dd.jpg)  
Fig. 8. Norms of critic and actor NN weights in Scenario 1.

![](images/e8cd0f037851fe888c2670e480e1afdc8f45994d5ff4ca35d707bcf1d306daed.jpg)

![](images/907ae06182b4429a5e938dbc330d78086a8fee88b7fe2111950c4612ae3863b4.jpg)

![](images/f70affb26241e838f390936fd9d2e5834a03b5ee0eae4e296fd7ca55d4c88434.jpg)  
Fig. 9. The actual position state and its observed value in Scenario 2.

![](images/bfbb656b86306755cc105a7528dba32dc617d0df06822d7a1bfba53674a9d62e.jpg)  
Fig. 10. The actual velocity state and its observed value in Scenario 2.

![](images/3222e31e5fa7bb477abe5ba20a2a3c81373690301276b7fc8e2b43421fb33e66.jpg)  
Fig. 11. Vessel position (�, �) tracks the reference trajectory $( x _ { \mathrm { { d } } } , y _ { \mathrm { { d } } } )$ in Scenario 2.

![](images/32e481677efa833e1a51dcdaf3b0e105affb328708c59374d94875509473309e.jpg)

![](images/d0e1f98a981fd4807e994da6c94705e72fb9e0ad3540434df322b23530a9800c.jpg)  
Fig. 12. Tracking errors of $z _ { 1 }$ and $z _ { 3 }$ in Scenario 2.

![](images/5f602b422ed047ac8a62e5fa361345872a3d6d09a5c6cf02199fea7d98cce704.jpg)

![](images/a7dd775c31d47272e7acd1de9504baf4ad033309a10d953a5737e083b621031e.jpg)  
Fig. 13. Control input in Scenario 2.

![](images/e1ee1ea664abd6e38825fbc8649df9335c4cadfaab35dd09519d211a43f31b84.jpg)  
Fig. 14. Cost function in Scenario 2.

## 6. Conclusion

By designing an ESO to estimate unmeasurable states and uncertainties, an AOTC for USVs is developed based on the backstepping technique and RL. In this scheme, a NN-based RL strategy with an AC architecture is adopted. where the critic-NN evaluates the control performance and the actor-NN generates control actions. The overall control of the vessel system is optimized by designing both virtual and actual control inputs as optimal solutions to their respective subsystems. Lyapunov-based analysis proves that the proposed control algorithm guarantees convergence to the desired trajectory. Future work will consider how to implement the collision-free formation control for USVs.

![](images/510f46b92df937963505c756cf505a59e084624ba34a61d9b703165dbda00725.jpg)

![](images/e81c49b148192b0776c50b57103c3433ad7cfcf00a503e7eb3838a129425425a.jpg)  
Fig. 15. Norms of critic and actor NN weights in Scenario 2.

## CRediT authorship contribution statement

Yinkun Li: Writing – original draft. Yawen Zhou: Writing – review & editing. Yufeng Zhou: Writing – review & editing, Writing – original draft. Li Chai: Supervision.

## Declaration of competing interest

The authors declare that they have no known competing financial interests or personal relationships that could have appeared to influence the work reported in this paper.

## Acknowledgments

The work was supported by the National Natural Science Foundation of China under Grants 62203338, 62173259 and U1913602, Zhejiang Provincial Natural Science Foundation of China under Grant LZ24F0390006, the Postdoctoral Science Foundation of China under Grant 2022M722485.

## Data availability

No data was used for the research described in the article.

## References

[1] C.A. Woolsey, Review of marine control systems: Guidance, navigation, and control of ships rigs and underwater vehicles J. Guid Control Dynam 28 (3) (2005) 574–575.

[2] Z. Peng, J. Wang, D. Wang, Q.-L. Han, An overview of recent advances in coordinated control of multiple autonomous surface vehicles, IEEE Trans. Ind. Inf, 17 (2) (2020) 732–745.

[3] C Pan Z Peng Y Li B Han D Wang Flocking of under-actuated unmanned surface vehicles via deep reinforcement learning and model predictive path integral control, JEEE Trans, Instrum, Meas, 73 (2024) 1–11.

[4] D. Xu, Z. Liu, X. Zhou, L. Yang, L. Huang, Trajectory tracking of underactuated unmanned surface vessels: Non-singular terminal sliding control with nonlinear disturbance observer, Appl, Sci, 12 (6) (2022) 3004.

[5] L. Hu, H. Hu, W. Naeem, Z. Wang, A review on COLREGs-compliant navigation of autonomous surface vehicles: From traditional to learning-based approaches, J. Autom, Intell, 1 (1) (2022) 100003.

[6] S. He, M. Wang, S.-L. Dai, F. Luo, Leader–follower formation control of USVs with prescribed performance and collision avoidance, IEEE Trans. Ind. Inf. 15 (1) (2019) 572–581

[7] Z.-P. Jiang, Global tracking control of underactuated ships by Lyapunov’s direct method, Automatica 38 (2) (2002) 301–309.

[8] Z. Liu, Practical backstepping control for underactuated ship path following associated with disturbances, IET Intell. Transp. Syst. 13 (5) (2019) 834–840.

[9] A. Gonzalez-Garcia, H. Castañeda, Guidance and control based on adaptive sliding mode strategy for a USV subject to uncertainties, IEEE J. Ocean. Eng. 46 (4) (2021) 1144–1154.

[10] J. Ghommam, S.E. Ferik, M. Saad, Robust adaptive path-following control of underactuated marine vessel with off-track error constraint, Int. J. Syst. Sci. 49 (7) (2018) 1540–1558.

[11] H. Zhang, X. Zhang, R. Bu, Radial basis function neural network sliding mode control for ship path following based on position prediction, J. Mar. Sci. Eng. 9 (10) (2021) 1055.

[12] S.-L. Dai, M. Wang, C. Wang, L. Li, Learning from adaptive neural network output feedback control of uncertain ocean surface ship dynamics, Int. J. Adapt. Control. 28 (3–5) (2014) 341–365.

[13] J. Wei, J. Zhang, H. Dong, Z. Liu, Prescribed-time trajectory tracking control for underactuated USV with input amplitude and rate constraints, Ocean Eng. 32 (2025) 120891–120891.

[14] J. Li, G. Zhang, W. Zhang, Q. Shan, W. Zhang, Cooperative path following control of USV-UAVs considerin low desi n complexit and command transmission requirements, IEEE Trans. Intell. Veh. 9 (1) (2024) 715–724.

[15] J. Li, G. Zhang, D. Cabecinhas, A.M. Pascoal, W. Zhang, Prescribed performance path following control of USVs via an output-based threshold rule, IEEE Trans Intell. Veh. 73 (5) (2024) 6171–6182.

[16] A. Merabet, S. Kanukollu, A. Al-Durra, E.F. El-Saadany, Adaptive recurrent neura network for uncertainties estimation in feedback control system, J. Autom. Intell 2 (3) (2023) 119–129

[17] M. Li, C. Guo, H. Yu, Extended state observer-based integral line-of-sight guidance law for path following of underactuated unmanned surface vehicles with uncertainties and ocean currents, Int. J. Adv. Robot. Syst. 18 (3) (2021) 17298814211011035.

[18] H. Jiang, G. Duan, M. Hou, State and disturbance observer-based controller design for fully actuated systems, IEEE Trans. Circuits Syst. I, Reg. Pap. 71 (11) (2024) 5261-5270

[19] J. Lan, Y.-J. Liu, D. Yu, G. Wen, S. Tong, L. Liu, Time-varving optimal formation control for second-order multiagent systems based on neural network observer and reinforcement learning JEEE Trans Neural Netw Learn Syst 35 (3) (2022) 3144–3155.

[20] Y. Li, Y. Liu, S. Tong, Observer-based neuro-adaptive optimized control of strict feedback nonlinear systems with state constraints, IEEE Trans. Neural Netw. Learn, Svst, 33 (7) (2021).3131–3145

[21] H. Wang, C. Liu, X. Huang, Y. Zhong, D. Qu, R.J. Patton, Extended state observerbased fault-tolerant control for an unmanned surface vehicle under as nchronou injection and deception attacks, Complex Eng. Syst. 4 (2024) 13.

[22] M.Y.-W. Zhang, L. Yang, Y. Zhu, Estimate error analysis of the nonlinear third order extended state observer. in: Proceedings of the 10th World Congress or Intelligent Control and Automation, IEEE, 2012, pp. 1621–1627.

[23] S. Song, Z. Liu, S. Yuan, Z. Wang, Cascaded extended state observers-based fixed-time line-of-sight path following control for unmanned surface vessels with disturbances and saturation, JEEE Trans Veh Technol 73 (6) (2024) 7733–7747

[24] Z. Peng, Y. Jiang, L. Liu, Y. Shi, Path-guided model-free flocking control of unmanned surface vehicles based on concurrent learning extended state observers, IEEE Trans. Syst. Man, Cybern.: Syst. 53 (8) (2023) 4729–4739.

[25] D. Liu, D. Wang, F.-Y. Wang, H. Li, X. Yang, Neural-network-based online HJB solution for optimal robust guaranteed cost control of continuous-time uncertain nonlinear systems JEEE Trans Cybern 44 (12) (2014) 2834–2847

[26] A. Al-Tamimi, E.L, Lewis, M. Abu-Khalaf, Discrete-time nonlinear HJB solutior using approximate dynamic programming: Convergence proof, IEEE Trans. Syst Man Cybern. B Cybern, 38 (4) (2008) 943–949.

[27] M. Ran, L. Xie, Adaptive observation-based efficient reinforcement learning for uncertain systems, IEEE Trans. Neural Netw. Learn. Syst. 33 (10) (2022) 5492–5503.

[28] W. Bai. T. Li. Y. Long, C.P. Chen. Event-triggered multigradient recursive reinforcement learning tracking control for multiagent systems, IEEE Trans Neural Netw, Learn. Syst, 34 (1) (2021) 366–379

[29] H. Li, Y. Wu, M. Chen, Adaptive fault-tolerant tracking control for discrete-time multiagent systems via reinforcement learning algorithm JEEE Trans Cybern 51 (3) (2020).1163-1174

[30] Y. Liu, Q. Zhu, G. Wen, Adaptive tracking control for perturbed strict-feedback nonlinear systems based on optimized backstepping technique, IEEE Trans. Neural Netw, Learn. Syst. 33 (2) (2020) 853–865

[31] Z. Yin, W. He, C. Yang, C. Sun, Control design of a marine vessel system using reinforcement learning, Neurocomputing 311 (2018) 353–362.

[32] Y. Su, F. Teng, T. Li, H. Liang, C.L.P. Chen, Fuzzy-based optimal control for an underactuated surface vessel with user-specified performance, IEEE Trans. Intell Transp. Syst. 26 (5) (2025) 7036–7050.

[33] Z. Ning, L. Xie, A survey on multi-agent reinforcement learning and its application, J. Autom. Intell. 3 (2) (2024) 73–91.

[34] L. Chen, S.-L. Dai, C. Dong, Adaptive optimal tracking control of an underactu ated surface vessel using actor–critic reinforcement learning, IEEE Trans. Neural Netw. Learn. Syst. 35 (6) (2024) 7520–7533.

[35] R. Lu, X. Wang, Y. Ding, H.-T. Zhang, F. Zhao, L. Zhu, Y. He, Adaptive optimal surrounding control of multiple unmanned surface vessels via actor-critic reinforcement learning, IEEE Trans. Neural Netw. Learn. Syst. 36 (7) (2025) 12173-12186

[36] Z. Li, Optimal control of unmanned surface vehicle trajectory tracking based on reinforcement learning, in: 2022 IEEE International Conference on Advances in Electrical Engineering and Computer Applications, AEECA, 2022, pp. 1315–1323.

[37] W. Chen, Q. Wei, A new optimal adaptive backstepping control approach fo nonlinear systems under deception attacks via reinforcement learning, J. Autom Intell. 3 (1) (2024) 34–39.

[38] G. Zhang, Z. Li, J. Li, W. Zhang, B. Qiu, Prescribed performance path-following control for rotor-assisted vehicles via an improved reinforcement learning mechanism, IEEE Trans. Neural Netw. Learn. Syst. 36 (9) (2025) 17395–17405.

[39] D. Swaroop, J.K. Hedrick, P.P. Yip, J.C. Gerdes, Dynamic surface control fo a class of nonlinear systems, IEEE Trans. Autom. Control 45 (10) (2000) 1893–1899.

[40] G. Wen, S.S. Ge, C.L.P. Chen, F. Tu, S. Wang, Adaptive tracking control of surface vessel using optimized backstepping technique, IEEE Trans. Cybern. 49 (9) (2019) 3420–3431.

![](images/56c78228950f30f0d75fb52c046c17942e83922473347ce76cd49e408b63ef99.jpg)  
Yinkun Li received the B.E. degree in Automation from the School of Electrical. Electronic and Control Engi neering, Qilu University of Technology, Jinan, China, in 2023. He is currently a master’s degree candidate in Electronic Information with the School of Artificial Intel. ligence and Automation, Wuhan University of Science and Technology Wuhan China His current research interests include unmanned surface vehicle control and reinforcement learning.

![](images/c3431dd108526355cfb205789ce121f74b2bfbf05c00c7cac9c10283ca6a96ec.jpg)

Yawen Zhou received the B.E. degree in Automation with the School of Information Science and Engineering from Wuhan University of Science and Technology, Wuhan, China in 2022. He is currently a master degree candidate in Electronic Information with the School of Artificial Intel ligence and Automation, Wuhan University of Science and Technology, Wuhan. His current research interests include multiagent system and event-triggered mechanism.

![](images/81acf37829e23ee9b6a5fc0db45374860ff251841a0d9a66c0b60cc3f0f699a5.jpg)

Yufeng Zhou received the Ph.D. degree from the School of Artificial Intelligence and Automation, Huazhong University of Science and Technology, Wuhan, China, in 2021. She was a Post-Doctoral Researcher in control science and engineer ing with the School of Information Science and Engineering, Wuhan University of Science and Technology, Wuhan, from 2021 to 2024. She is currently an Associate Professor in School of Artificial Intelligence and Automation, Wuhan University of Science and Technology, Wuhan. She current research interests include memristive neural networks, event-triggered mechanism, and reinforcement Learning.

![](images/0b95ace2e7897702d00418cb1960d2356c464dee6f265b6c354822338b849b5d.jpg)

Li Chai received the B.S. degree in applied mathematics and the M. S. degree in control science and engineering, both from Zhejiang University, China, in 1994 and 1997, respectively, and the Ph.D. degree in electrical engineering from Hong Kong University of Science and Technology ir 2002 From August 2002 to December 2007 he was witl Hangzhou Dianzi University, China. He was a Professor with the Wuhan University of Science and Technology, China, from 2008 to 2022. In August 2022, he joined Zhejiang University, where he is currently a Full Professor with the College of Control Science and Engineering. He has been

a Postdoctoral Researcher or a Visiting Scholar with Monash University, Newcastle University, Australia, and Harvard University, USA. He has published over 100 fully refereed papers in prestigious journals and leading conferences. His current research interests include distributed optimization. filter banks, graph signal processing, and networked control systems. He was a recipient of the Distinguished Young Scholar of the National Science Foundation of China He is the Expert of State Council Specia Allowance, China. He has been an Associate Editor of IEEE TRANSACTIONS ON CIRCUIT AND SYSTEMSII: EXPRESS BRIEFS and Journal of Control and Decision