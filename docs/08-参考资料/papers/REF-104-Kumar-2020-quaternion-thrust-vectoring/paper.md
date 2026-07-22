# Quaternion Feedback Based Autonomous Control of a Quadcopter UAV with Thrust Vectoring Rotors

Rumit Kumar1, Mahathi Bhargavapuri2, Aditya M. Deshpande1, Siddharth Sridhar1, Kelly Cohen1, and Manish Kumar1

Abstract—In this paper, we present an autonomous flight controller for a quadcopter with thrust vectoring capabilities. This UAV falls in the category of multirotors with tilt-motion enabled rotors. Since the vehicle considered is over-actuated in nature, the dynamics and control allocation have to be analysed carefully. Moreover, the possibility of hovering at large attitude maneuvers of this novel vehicle requires singularity-free attitude control. Hence, quaternion state feedback is utilized to compute the control commands for the UAV motors while avoiding the gimbal lock condition experienced by Euler angle based controllers. The quaternion implementation also reduces the overall complexity of state estimation due to absence of trigonometric parameters. The quadcopter dynamic model and state space is utilized to design the attitude controller and control allocation for the UAV. The control allocation, in particular, is derived by linearizing the system about hover condition. This mathematical method renders the control allocation more accurate than existing approaches. Lyapunov stability analysis of the attitude controller is shown to prove global stability. The quaternion feedback attitude controller is commanded by an outer position controller loop which generates rotortilt and desired quaternions commands for the system. The performance of the UAV is evaluated by numerical simulations for tracking attitude step commands and for following a waypoint navigation mission.

## I. InTRODuCTION

The design advances in vertical takeoff and landing (VTOL) UAV research have led to the development of various types of quadcopter platforms. These design variants are based on the simple four propeller model to produce thrust for VTOL but utilize different methodologies for attitude control and navigation of the quadcopter in threedimensional space. These methodologies can be implemented at flight software level as well as hardware level in the UAV. The hardware design advances include variable blade pitch quadcopters, tilt-rotor quadcopters, engine powered and re-configurable unmanned aerial systems. In a variable blade pitch quadcopter, motion and orientation control is achieved by the change in blade pitch angle of different rotors in various combinations. This UAV platform is capable of producing reverse thrust and inverted flights [1] [2]. In a tilt-rotor UAV, the propeller motors are actuated to tilt about the arm connecting to the main quadcopter body using servo motors [3]. This UAV can follow tight trajectories and provide better disturbance rejection towards uncertainties during flight [4] [5]. This paper focuses on position and attitude control design for the tilt-rotor UAV using quaternion state feedback and accurate control allocation for the over-actuated system. Dynamic modeling and control design methods for the various flight modes of tilt-rotor quadcopter are discussed in [4], [6], [7], [8] and [9]. The work in [10] has shown that the tilt-rotor quadcopter can achieve large attitude angles. In recent works, Franchi et al. in [11] have defined a general class of tilt-rotor UAV with laterally bounded input force for full-pose tracking. Invernizzi et al. in [12] have utilized geometric control theory to develop control laws for tilt-rotor UAV. Similarly, fault-tolerant control in case of a single propeller or motor failure using tilt-rotor UAV are discussed in [13][14] [15]. These previous works on tiltrotor quadcopter utilize Euler angles and direction cosine matrices formulation for developing flight controllers. From dynamics and control perspective, the attitude characteristics of any rigid body cannot be extracted by integrating the angular velocities in body frame because the Euler angles are defined in different frames and are only locally valid. So, the control engineers rely on integral solution of kinematic equations which are limited by inherent singularities and only a limiting solution for attitude can be obtained when two of the rotational axes coincide [16]. This phenomenon is termed as gimbal lock where, we can not distinguish between two degrees of freedom because those rotational axes coincide with each other [17]. Quaternion feedback based controller is a viable solution to overcome gimbal lock limitations as well as the complexity of estimating rotation matrices. The control applications of quaternion feedback based controller for multi-rotor UAV platforms are shown in [2], [18], [19], and [20]. Bhargavapuri et al. in [21] have used quaternion feedback based attitude controller for tilt-rotor quadcopter. The work by Fresk et al. in [17] and [20] for quaternion feedback based attitude controller for quadcopters is quite notable. Although, quaternion feedback controllers are developed in literature for traditional quadcopters, this work is one of the first attempts to develop quaternion feedback based attitude controller for an over-actuated tilt-rotor UAV. The detailed mathematical model for translational and rotational dynamics of the tilt-rotor UAV are presented. The moment equations are linearized using small perturbation theory or Taylor's series expansion to derive the necessary control allocation for the attitude control of the UAV. This is the first time such a mathematical approach is presented for derivation of control allocation for tilt-rotor UAVs. It has been shown that attitude control using quaternions is actually a regulation problem in terms of error quaternion. Further, Lyapunov stability analysis for quaternion control regulatory loop is shown to establish stability of the closedloop system. The attitude controller is validated by numerical simulations and the inner quaternion attitude feedback loop is commanded by integrating an external position controller loop for achieving autonomous way-point navigation.

<!-- image-->
Fig. 1: Tilt-rotor quadcopter free body diagram [4]

## II. Dynamic ModeL

This section provides a brief introduction to quaternions and the dynamic model including equations governing the translational and rotational motion of the UAV. A quaternion is a hypercomplex number in $\mathbb { R } ^ { 4 }$ , consisting of a scalar part and a vector part with three elements as shown in (1).

$$
\begin{array} { r l r } { \mathbb { A } } & { { } = } & { \mathbb { A } _ { \mathbb { 0 } } + \mathbb { A } _ { \mathbb { 1 } } i + \mathbb { A } _ { \mathbb { 2 } } j + \mathbb { A } _ { \mathbb { 3 } } k } \end{array}\tag{1}
$$

where, $\mathbb { A } _ { \mathbb { O } }$ is the scalar part and $\mathbb { A } _ { 1 } , \mathbb { A } _ { 2 } , \mathbb { A } _ { 3 }$ are the elements of the vector part $( \bar { \mathbb { A } } )$ of the quaternion. Quaternion multiplication $( \mathbb { A } \otimes \mathbb { B } )$ , quaternion conjugate (A\*), and quaternion normalize (Å) are the main operations used in this work. The detailed explanation on quaternion operations can be found in [22]. The expression of quaternion derivative for describing the vehicle attitude is shown in (2) and (3).

$$
\dot { \mathbb { G } } = \frac { 1 } { 2 } \mathbb { q } \bigotimes \frac { \ d \ b { 0 } } { \ d \Omega } \big [ \ b { 0 } \big ]\tag{2}
$$

$$
\begin{array} { r l r } { \left[ \dot { \mathbb { Q } _ { \mathrm { i } } } ^ { \mathrm { o } } \right] } & { { } = } & { \frac { 1 } { 2 } \left[ \begin{array} { c c c c } { 0 } & { - p } & { - q } & { - r } \\ { p } & { 0 } & { r } & { - q } \\ { q } & { - r } & { 0 } & { p } \\ { r } & { q } & { - p } & { 0 } \end{array} \right] \left[ \mathbb { Q } _ { \mathrm { i } } \right] } \end{array}\tag{3}
$$

In equation (2), $\Omega = \left[ p \quad q \quad r \right] ^ { T }$ represent the vehicle body rates and the quaternion can be estimated by integrating (2). The normalize operation is used to convert a quaternion into unit quaternion as shown in (4).

$$
\hat { \mathbb { q } } = \frac { \mathbb { q } _ { 0 } + \mathbb { q } _ { 1 1 } i + \mathbb { q } _ { 2 } j + \mathbb { q } _ { 3 } k } { \sqrt { { \mathbb q } _ { 0 } ^ { 2 } + { \mathbb q } _ { 1 1 } ^ { 2 } + { \mathbb q } _ { 2 } ^ { 2 } + { \mathbb q } _ { 3 } ^ { 2 } } }\tag{4}
$$

The conversion from quaternion to Euler angles can be achieved by using the operation shown in (5).

$$
\left[ \begin{array} { c } { \phi } \\ { \theta } \\ { \psi } \end{array} \right] = \left[ \begin{array} { c c c } { \mathrm { a t a n 2 } [ 2 ( \hat { \mathbb { q _ { 0 } } } \hat { \mathbb { q _ { 1 } } } + \hat { \mathbb { q _ { 2 } } } \hat { \mathbb { q _ { 3 } } } ) , 1 - 2 ( \hat { \mathbb { q _ { 1 } } } ^ { 2 } + \hat { \mathbb { q _ { 2 } } } ^ { 2 } ) ] } \\ { \mathrm { a s i n } [ 2 ( \hat { \mathbb { q _ { 0 } } } \hat { \mathbb { q _ { 2 } } } - \hat { \mathbb { q _ { 3 } } } \hat { \mathbb { q _ { 1 } } } ) ] } \\ { \mathrm { a t a n 2 } [ 2 ( \hat { \mathbb { q _ { 0 } } } \hat { \mathbb { q _ { 3 } } } + \hat { \mathbb { q _ { 1 } } } \hat { \mathbb { q _ { 2 } } } ) , 1 - 2 ( \hat { \mathbb { q _ { 2 } } } ^ { 2 } + \hat { \mathbb { q _ { 3 } } } ^ { 2 } ) ] } \end{array} \right]\tag{5}
$$

The rotating propellers in the quadcopter produce forces and moments and the tilting motion of rotating propellers helps in force and moment vectoring as shown in Fig. 1 [4]. The translational motion dynamics of the tilt-rotor quadcopter are described by equation (6)

$$
\left[ \begin{array} { c } { \ddot { x } } \\ { \vdots } \\ { \ddot { y } } \\ { \vdots } \\ { \ddot { z } } \end{array} \right] = \boldsymbol { \mathbb { Q } } \otimes \left[ \begin{array} { c } { \frac { F _ { 2 } s \theta _ { 2 } + F _ { 4 } s \theta _ { 4 } } { m } } \\ { \frac { - F _ { 1 } s \theta _ { 1 } - F _ { 3 } s \theta _ { 3 } } { m } } \\ { \frac { F _ { 1 } c \theta _ { 1 } + F _ { 2 } c \theta _ { 2 } + F _ { 3 } c \theta _ { 3 } + F _ { 4 } c \theta _ { 4 } } { m } } \end{array} \right] \otimes \boldsymbol { \mathbb { Q } } ^ { * } - \left[ \begin{array} { c } { 0 } \\ { 0 } \\ { 0 } \\ { \frac { - \left[ \begin{array} { c } { F _ { 2 } s \theta _ { 1 } + F _ { 4 } s \theta _ { 2 } + F _ { 4 } s \theta _ { 4 } } \\ { g } \end{array} \right] } { m } } \end{array} \right]\tag{6}
$$

where $\mathbb { Q }$ is the quaternion that transform the acceleration vector from body frame to the inertial frame of reference. The sine and cosine angle terms are shown as s∠ and $c /$ respectively. Further, $\theta _ { i } , \ \forall i \ \in \ \{ 1 , 2 , 3 , 4 \}$ are the rotor tilt angles, m is the total mass of quadcopter, g is the acceleration due to gravity, x, j and z are the linear accelerations in the world frame, $F _ { i } , \forall i \in \{ 1 , 2 , 3 , 4 \}$ represent the propeller thrust forces. The rotational dynamics of the UAV are represented as effective torque along body axes as shown in (7) and (8)

(7)

$$
\begin{array} { r c l } { { \dot { \Omega } } } & { { = } } & { { I ^ { - 1 } \ ( \tau - \Omega \times I \Omega ) } } \\ { { } } & { { } } & { { } } \\ { { \tau } } & { { = } } &  { { \left[ { l ( F _ { 2 } c \theta _ { 2 } - F _ { 4 } c \theta _ { 4 } ) + M _ { 2 } s \theta _ { 2 } + M _ { 4 } s \theta _ { 4 } } \right] } } \\ { { } } & { { } } & { { } } \\ { { \tau } } & { { = } } & { { { \left| l ( F _ { 3 } c \theta _ { 3 } - F _ { 1 } c \theta _ { 1 } ) + M _ { 3 } s \theta _ { 3 } + M _ { 1 } s \theta _ { 1 } \right| } } } \\ { { } } & { { } } & { { } } \\ { { } } & { { } } & { { { \left| l ( - F _ { 1 } s \theta _ { 1 } - F _ { 2 } s \theta _ { 2 } + F _ { 3 } s \theta _ { 3 } + F _ { 4 } s \theta _ { 4 } ) \right| } } } \\ { { } } & { { } } & { { { \left| - M _ { 1 } c \theta _ { 1 } + M _ { 2 } c \theta _ { 2 } - M _ { 3 } c \theta _ { 3 } + M _ { 4 } c \theta _ { 4 } \right| } } } \end{array}\tag{8}
$$

where I is the diagonal moment of inertia matrix with $[ I _ { x x } , I _ { y y } , I _ { z z } ]$ as the diagonal elements. M, $\forall i \in \{ 1 , 2 , 3 , 4 \}$ are the moments produced by the rotor drag. The propeller thrust force $F _ { i }$ and moment $M _ { i }$ are related to the rotational angular speed $\omega _ { i }$ of the ith rotor by (9)

$$
F _ { i } = k _ { f } \omega _ { i } ^ { 2 } , \qquad M _ { i } = k _ { m } \omega _ { i } ^ { 2 } , \quad \forall i \in \{ 1 , 2 , 3 , 4 \}\tag{9}
$$

where $k _ { f }$ and $k _ { m }$ are force and moment coefficients, respectively [23]. It should be noted that any variations in propeller speeds and rotor tilts lead to a resultant torque about body axes producing angular accelerations. The body rates can be obtained by integral solution of (7). These rates can be estimated in an actual system using an inertial measurement unit (IMU) sensor. Further, the body rates can be utilized to estimate quaternions using (2).

## III. ConTroLleR DeveLopment

## A. Attitude Control

In this section, the attitude controller of tilting rotor quadcopter is derived using quaternion feedback. The rotational dynamics of the system are linearized at a hovering state. At the hovering state, all propellers of the UAV are spinning at a nominal angular speed $( \omega _ { h } )$ and the rotors are tilted by $( \theta _ { h } )$ angle. The roll, pitch and yaw angles are all zero in the hovering state. This condition is related to a unit quaternion, thus the vehicle attitude can be represented by $\begin{array} { r l r l } { [ 1 } & { { } 0 } & { 0 } & { { } 0 ] ^ { T } } \end{array}$ quaternion. This assumption simplifies quaternion kinematics equation (3). The quaternion rate and acceleration directly relate to body rates and accelerations of the system respectively and the control input torques will affect the quaternion vector elements as shown in (10).

<!-- image-->
Fig. 2: Attitude control architecture

$$
\begin{array} { r l r } { \left[ \begin{array} { c } { \dot { \mathfrak { q } _ { 0 } } } \\ { \dot { \mathfrak { q } _ { 1 } } } \\ { \dot { \mathfrak { q } _ { 2 } } } \\ { \dot { \mathfrak { q } _ { 3 } } } \end{array} \right] } & { { } = } & { \frac { 1 } { 2 } \left[ \begin{array} { c } { 0 } \\ { p } \\ { q } \\ { r } \end{array} \right] ; \qquad \left[ \begin{array} { c } { \ddot { \mathfrak { q } _ { 0 } } } \\ { \ddot { \mathfrak { q } _ { 1 } } } \\ { \ddot { \mathfrak { q } _ { 2 } } } \\ { \ddot { \mathfrak { q } _ { 3 } } } \end{array} \right] = \frac { 1 } { 2 } \left[ \begin{array} { c } { 0 } \\ { \dot { p } } \\ { \dot { q } } \\ { \dot { r } } \end{array} \right] } \end{array}\tag{10}
$$

We can solve (7) analytically using Taylor's series expansion. The rate multiplication Coriolis terms are very small and we ignore them to simplify the rotational dynamics [16].

$$
\begin{array} { r l r } { I \left[ \begin{array} { c } { \dot { p } } \\ { \dot { q } } \\ { \dot { r } } \end{array} \right] } & { { } = } & { \left[ \begin{array} { c } { l ( F _ { 2 } c \theta _ { 2 } - F _ { 4 } c \theta _ { 4 } ) + M _ { 2 } s \theta _ { 2 } + M _ { 4 } s \theta _ { 4 } } \\ { \qquad } \\ { l ( F _ { 3 } c \theta _ { 3 } - F _ { 1 } c \theta _ { 1 } ) + M _ { 3 } s \theta _ { 3 } + M _ { 1 } s \theta _ { 1 } } \\ { \qquad } \\ { l ( - F _ { 1 } s \theta _ { 1 } - F _ { 2 } s \theta _ { 2 } + F _ { 3 } s \theta _ { 3 } + F _ { 4 } s \theta _ { 4 } ) } \\ { \qquad } \\ { - M _ { 1 } c \theta _ { 1 } + M _ { 2 } c \theta _ { 2 } - M _ { 3 } c \theta _ { 3 } + M _ { 4 } c \theta _ { 4 } } \end{array} \right] } \end{array}\tag{11}
$$

The solution for angular motion about $x _ { b }$ axis is presented here and it can be generalized about $y _ { b }$ and $z _ { b }$ axes. Equation for angular motion about $x _ { b }$ axis is shown in (12) using the force and moment values from equation (9).

$$
I _ { x x } \dot { p } = l k _ { f } \omega _ { 2 } ^ { 2 } c \theta _ { 2 } - l k _ { f } \omega _ { 4 } ^ { 2 } c \theta _ { 4 } + k _ { m } \omega _ { 2 } ^ { 2 } s \theta _ { 2 } + k _ { m } \omega _ { 4 } ^ { 2 } s \theta _ { 4 }\tag{12}
$$

Taylor's series expansion about the hover linearization point:

$$
\begin{array} { r } { I _ { x x } \delta \dot { p } = 2 l k _ { f } \omega _ { 2 } \delta \omega _ { 2 } c \theta _ { 2 } - l k _ { f } \omega _ { 2 } ^ { 2 } s \theta _ { 2 } \delta \theta _ { 2 } - 2 l k _ { f } \omega _ { 4 } \delta \omega _ { 4 } c \theta _ { 4 } } \\ { + l k _ { f } \omega _ { 4 } ^ { 2 } s \theta _ { 4 } \delta \theta _ { 4 } + 2 k _ { m } \omega _ { 2 } \delta \omega _ { 2 } s \theta _ { 2 } + k _ { m } \omega _ { 2 } ^ { 2 } c \theta _ { 2 } \delta \theta _ { 2 } } \\ { + 2 k _ { m } \omega _ { 4 } \delta \omega _ { 4 } s \theta _ { 4 } + k _ { m } \omega _ { 4 } ^ { 2 } c \theta _ { 4 } \delta \theta _ { 4 } } \end{array}
$$

Based on the assumption for linearization about hover condition $\omega _ { 2 } = \omega _ { 4 } = \omega _ { h }$ and $\theta _ { 2 } = \theta _ { 4 } = \theta _ { h }$

$$
\begin{array} { r } { I _ { x x } \delta \dot { p } = 2 l k _ { f } \omega _ { h } \delta \omega _ { 2 } c \theta _ { h } - l k _ { f } \omega _ { h } ^ { 2 } s \theta _ { h } \delta \theta _ { 2 } - 2 l k _ { f } \omega _ { h } \delta \omega _ { 4 } c \theta _ { h } } \\ { + l k _ { f } \omega _ { h } ^ { 2 } s \theta _ { h } \delta \theta _ { 4 } + 2 k _ { m } \omega _ { h } \delta \omega _ { 2 } s \theta _ { h } + k _ { m } \omega _ { h } ^ { 2 } c \theta _ { h } \delta \theta _ { 2 } } \\ { + 2 k _ { m } \omega _ { h } \delta \omega _ { 4 } s \theta _ { h } + k _ { m } \omega _ { h } ^ { 2 } c \theta _ { h } \delta \theta _ { 4 } } \end{array}
$$

The tilt-rotor quadcopter achieves hover condition at zero tilt angle and acts like a conventional quadcopter. Thus, $\theta _ { h } \to 0 .$

$$
\begin{array} { r l r } & { } & { I _ { x x } \delta \dot { p } = 2 l k _ { f } \omega _ { h } ( \delta \omega _ { 2 } - \delta \omega _ { 4 } ) + k _ { m } \omega _ { h } ^ { 2 } ( \delta \theta _ { 2 } + \delta \theta _ { 4 } ) } \\ & { } & { \delta \dot { p } = \frac { 2 l k _ { f } \omega _ { h } ( \delta \omega _ { 2 } - \delta \omega _ { 4 } ) } { I _ { x x } } + \frac { k _ { m } \omega _ { h } ^ { 2 } ( \delta \theta _ { 2 } + \delta \theta _ { 4 } ) } { I _ { x x } } } \end{array}\tag{13}
$$

This expression yields the change in roll rate when the rotor speeds and rotor tilts are exercised simultaneously. It should be noted that the expression in (13) relates to the control of first vector element of the quaternion attitude. The change in angular speed of propellers $( \delta \omega _ { 2 } - \delta \omega _ { 4 } = \Delta \omega _ { \phi } )$ and rotor tilts $( \delta \theta _ { 2 } + \delta \theta _ { 4 } = \Delta \theta _ { \phi } )$ provides the necessary control action for attitude change and it can be represented in state space form as shown in (14).

$$
\begin{array} { r l r } { \left[ \delta \dot { p } \right] } & { { } = } & { \left[ \begin{array} { c c } { 0 } & { 0 } \\ { 1 } & { 0 } \end{array} \right] \left[ \begin{array} { c } { \delta p } \\ { \delta \mathbb { Q } _ { 1 1 } } \end{array} \right] + \left[ \begin{array} { c c c } { \frac { l k _ { f } \omega _ { h } } { I _ { x x } } } & { } & { \frac { k _ { m } \omega _ { h } ^ { 2 } } { 2 I _ { x x } } } \\ { 0 } & { } & { 0 } \end{array} \right] \left[ \begin{array} { c } { \Delta \omega _ { \phi } } \\ { \Delta \theta _ { \phi } } \end{array} \right] \left( 1 4 \right) } \end{array}
$$

Similar expressions can be obtained about $y _ { b }$ and $z _ { b }$ axes. They can be easily accommodated in existing state space as shown from (15) to (18)

$$
\begin{array} { r c l } { \delta \dot { q } } & { = } & { \frac { l k _ { f } \omega _ { h } \left( \delta \omega _ { 3 } - \delta \omega _ { 1 } \right) } { I _ { y y } } + \frac { k _ { m } \omega _ { h } ^ { 2 } \left( \delta \theta _ { 3 } + \delta \theta _ { 1 } \right) } { 2 I _ { y y } } } \end{array}\tag{15}
$$

$$
\begin{array} { r l r } { [ \delta \dot { q } ] } & { = } & { [ 0  \qquad 0 ] [ \delta q ] + [ \frac { l k _ { f } \omega _ { h } } { I _ { y y } } \qquad \frac { k _ { m } \omega _ { h } ^ { 2 } } { 2 I _ { y y } } ] [ \Delta \omega _ { \theta } ] } \\ & { } & { [ 1 \qquad 0 ] [ \delta \mathbb { q } _ { \mathcal { Q } } ] + [ \begin{array} { c c c } { 0 } & { } & { 0 } \\ { 0 } & { } & { \phantom { } \frac { k _ { m } \omega _ { h } ^ { 2 } } { 2 I _ { y y } } } \end{array} ] [ \Delta \theta _ { \theta } ] } \end{array}\tag{16}
$$

$$
\delta \dot { r } = \frac { k _ { m } \omega _ { h } \Delta \omega _ { \psi } } { I _ { z z } } + \frac { l k _ { f } \omega _ { h } ^ { 2 } \Delta \theta _ { \psi } } { 2 I _ { z z } }\tag{17}
$$

$$
\begin{array} { r l r } { \left[ \begin{array} { c c } { \delta \dot { r } } \\ { \delta \dot { \mathbb { Q } } _ { 3 } } \end{array} \right] } & { = } & { \left[ \begin{array} { c c } { 0 } & { 0 } \\ { 1 } & { 0 } \end{array} \right] \left[ \begin{array} { c } { \delta r } \\ { \delta \mathbb { Q } _ { 3 } } \end{array} \right] + \left[ \begin{array} { c c c } { \frac { k _ { m } \omega _ { h } } { I _ { z z } } \quad } & { \frac { l k _ { f } \omega _ { h } ^ { 2 } } { 2 I _ { z z } } } \\ { 0 } & { 0 } \end{array} \right] \left[ \begin{array} { c } { \Delta \omega _ { \psi } } \\ { \Delta \theta _ { \psi } } \end{array} \right] \left( 1 8 \right) } \end{array}
$$

where change in rotors' angular speed for yaw control $\Delta \omega _ { \psi } \ = \ - \delta \omega _ { 1 } + \delta \omega _ { 2 } - \delta \omega _ { 3 } + \delta \omega _ { 4 }$ and change in rotors' tilt angle for yaw control $\Delta \theta _ { \psi } = - \delta \theta _ { 1 } - \delta \theta _ { 2 } + \delta \theta _ { 3 } + \delta \theta _ { 4 }$ The quaternion operations in the controller are performed in normalized form. The desired quaternion $\hat { \mathbb { Q } } ^ { d e s }$ and conjugate of the normalized quaternion  are used to compute the error quaternion $\hat { \mathbb { q } } ^ { e r r }$ as shown in (19).

$$
\hat { \mathbb { q } } ^ { e r r } = \hat { \mathbb { q } } ^ { d e s } \bigotimes \hat { \mathbb { q } } ^ { * }\tag{19}
$$

The error quaternion changes based on the required rotation for achieving the desired attitude. $\mathbf { A } \mathbf { s } ,$ the UAV attains the desired attitude, the error quaternion becomes unit quaternion. It should be noted that the desired error quaternion $\hat { \mathbb { G } } _ { e r r } ^ { d e s }$ is always a unit quaternion. Thus, the attitude control using quaternion feedback actually becomes a regulation problem in terms of quaternion error where the controller objective is to make the elements of the vector part of the error quaternion equal to zero. The change in propeller speeds and rotor tilt for achieving a desired orientation are shown in (20) and (21), which are similar to expressions presented in [17]. Fig. 2 shows the schematic of the attitude controller, here the outer loop generates commands for the inner loop.

$$
\begin{array} { r l r } { \left[ \Delta \omega _ { \phi } \right] } & { { } = } & { - k _ { \oplus } \left[ \hat { \mathbb { Q } _ { 1 2 } } ^ { e r r } \right] - k _ { \omega } \left[ \begin{array} { l } { p } \\ { q } \\ { \hat { \Delta \omega _ { \psi } } } \end{array} \right] } \end{array}\tag{20}
$$

$$
\begin{array} { r l r } { \left[ \Delta \theta _ { \phi } \right] } & { { } = } & { - k _ { \mathrm { q } } ^ { ' } \left[ \hat { \mathcal { 1 } } _ { \perp } ^ { ~ e r r } \right] - k _ { \omega } ^ { ' } \left[ \begin{array} { l } { p } \\ { q } \\ { \hat { \mathcal { 1 } } ^ { ~ e r r } } \end{array} \right] } \end{array}\tag{21}
$$

Here, $k _ { \mathfrak { a } } , k _ { \omega } , k _ { \mathfrak { a } } ^ { ' } , k _ { \omega } ^ { ' } ; \forall \in \mathrm { ~ R } ^ { 3 x 3 }$ are diagonal positive definite gain matrices.

## B. Lyapunov stability analysis

We use a similar Lyapunov function discussed in [24] to analyze the stability of the tilt-rotor UAV attitude controller. Lemma 1: For the error dynamics given by

$$
\begin{array} { r c l } { \dot { \hat { \mathbb { G } } } _ { e r r } } & { = } & { \displaystyle \frac { 1 } { 2 } \hat { \mathbb { G } } _ { e r r } \bigotimes \Big [ \Omega _ { e r r } ^ { \mathrm { ~ 0 ~ } } \Big ] } \end{array}\tag{22}
$$

$$
\begin{array} { r l r } { \dot { \Omega } _ { e r r } } & { { } = } & { \dot { \Omega } - \dot { \Omega } ^ { d e s } + \Omega \times \Omega ^ { d e s } } \end{array}\tag{23}
$$

where $\Omega _ { e r r } = \Omega - \Omega ^ { d e s } , \Omega ^ { d e s }$ is the desired angular velocity vector defined in the desired frame of reference and the vector elements for error quaternion $\hat { \mathbb { { q } } } _ { e r r }$ are described by $\epsilon = [ \hat { \mathbb { q } _ { 1 } } ^ { e r r } , \hat { \mathbb { q } _ { 2 } } ^ { e r r } , \hat { \mathbb { q } _ { 3 } } ^ { e r r } ] ^ { T }$ . The control law in (24) is globally stabilizing and reduces to expression similar to (20), (21) for stabilization to a desired attitude, i.e. $\Omega ^ { d e s } \to 0 \mathrm { ~ a s ~ } \epsilon \to 0 .$

$$
\begin{array} { r l r } { \tau \left( \Delta \omega _ { i } , \Delta \theta _ { i } \right) } & { = } & { - k _ { \mathbb { Q } } \epsilon - k _ { \Omega } \Omega _ { e r r } + I \dot { \Omega } ^ { d e s } \qquad ( 2 4 ) } \\ & { - } & { I \left( \Omega \times \Omega ^ { d e s } \right) + \Omega \times I \Omega \qquad \forall i \in \{ \phi , \theta , \psi \} } \end{array}
$$

Proof: Consider the Lyapunov' function candidate

$$
V = k _ { \mathbb { Q } } ( { \hat { \mathbb { q } } } ^ { e r r } - { \hat { \mathbb { G } } } _ { e r r } ^ { d e s } ) ^ { T } ( { \hat { \mathbb { q } } } ^ { e r r } - { \hat { \mathbb { q } } } _ { e r r } ^ { d e s } ) + \frac { 1 } { 2 } \Omega _ { e r r } ^ { T } I \Omega _ { e r r }
$$

The time derivative of the Lyapunov candidate is computed and substituting for $\dot { \hat { \mathbb { Q } } } _ { e r r } , \dot { \Omega } _ { e r r }$ and Ω from (22), (23) and (7).

$$
\begin{array} { r c l } { \dot { V } } & { = } & { 2 k _ { \mathbb { Q } } ( \hat { \mathbb { q } } ^ { e r r } - \hat { \mathbb { G } } _ { e r r } ^ { d e s } ) ^ { T } ( \dot { \hat { \mathbb { Q } } } ^ { e r r } - \dot { \hat { \mathbb { Q } } } _ { e r r } ^ { d e s } ) + \Omega _ { e r r } I \dot { \Omega } _ { e r r } } \\ { \dot { V } } & { = } & { k _ { \mathbb { Q } } \Omega _ { e r r } ^ { T } [ f ( \hat { \mathbb { Q } } ^ { e r r } ) ] ^ { T } ( \hat { \mathbb { Q } } ^ { e r r } - \hat { \mathbb { Q } } _ { e r r } ^ { d e s } ) } \\ & { + } & { \Omega _ { e r r } ^ { T } \Big ( I \dot { \Omega } - I \dot { \Omega } ^ { d e s } + I \Big ( \Omega \times \Omega ^ { d e s } \Big ) \Big ) } \end{array}
$$

where the definition of $[ f ( \hat { \mathbb { q } } ^ { e r r } ) ]$ is identical to that explained in [24], $[ f ( \hat { \mathbb { q } } ^ { e r r } ) ] ^ { T } \hat { \mathbb { q } } ^ { e r r } = 0$ and $[ f ( \hat { \mathbb { q } } ^ { e r r } ) ] ^ { T } \hat { \mathbb { q } } _ { e r r } ^ { d e s } = \epsilon .$ The vector part of error quaternion is given by $\epsilon = [ \hat { \mathbb { q } _ { \mathbb { 1 } } } ^ { e r r } , \hat { \mathbb { q } _ { \mathbb { 2 } } } ^ { e r r } , \hat { \mathbb { q } _ { \mathbb { 3 } } } ^ { e r r } ] ^ { T }$

$$
\begin{array} { r c l } { \dot { V } } & { = } & { k _ { \perp } \Omega _ { e r r } ^ { T } \epsilon + \Omega _ { e r r } ^ { T } \left( - \Omega \times I \Omega + \tau - I \dot { \Omega } ^ { d e s } + I ( \Omega \times \Omega ^ { d e s } ) \right) } \end{array}
$$

Substituting (24) in the Lyapunov time derivative equation, we obtain the following result after simplification

$$
\dot { V } = - k _ { \Omega } \Omega _ { e r r } ^ { T } \Omega _ { e r r } \leq 0
$$

where $k _ { \mathbb { Q } } , k _ { \Omega } ; \forall \in \mathrm { ~ \mathbb { R } ^ { 3 } } { } ^ { x 3 }$ are diagonal positive definite gain matrices. The control torque τ is a function of variation in propeller angular speeds and tilt angles as shown in (14), (16), (18). The Lyapunov function here considers $\Omega _ { e r r } ,$ but the final implementation reduces to the expressions shown in $( 2 0 ) , ~ ( 2 1 )$ upon simplification. Here, $\Omega ^ { d e s } ~ = ~ - k _ { \mathbb { Q } } \epsilon$ , and $\Omega ^ { d e s } \to 0 \mathrm { ~ a s ~ } \epsilon \to 0 .$ The Lyapunov candidate is unbounded, the stability properties hold globally. It is straightforward to show asymptotic stability using invariance principles.

## C. Position Control

Here, the position controller of the UAV is developed which generates quaternion commands for the inner attitude controller. The tilting motion of rotors causes acceleration along xy-axes. Hence the longitudinal and lateral motion of the system can also be controlled using rotor tilts. The position errors $( e _ { x } , e _ { y } , e _ { z } )$ ,velocity errors $( \dot { e } _ { x } , \dot { e } _ { y } , \dot { e } _ { z } )$ and error integrals are utilized by the outer PID controller loop as shown in (25) to compute rotor-tilt commands $\Delta \theta _ { i } ; \forall i \in \{ x , y \}$ and desired accelerations commands $\ddot { r } _ { i } ^ { d e s } ; \forall i \in \{ x , y , z \}$

$$
\begin{array} { r c l } { { \vec { r } _ { x } ^ { + d e s } } } & { { = } } & { { k _ { p _ { s } } e _ { x } + k _ { i _ { x } } \displaystyle \int e _ { x } d t + k _ { d _ { x } } \dot { e } _ { x } } } \\ { { } } & { { } } & { { } } \\ { { \vec { r } _ { y } ^ { + d e s } } } & { { = } } & { { k _ { p _ { s } } e _ { y } + k _ { i _ { y } } \displaystyle \int e _ { y } d t + k _ { d _ { y } } \dot { e } _ { y } } } \\ { { } } & { { } } & { { } } \\ { { \vec { r } _ { z } ^ { - d e s } } } & { { = } } & { { k _ { p _ { z } } e _ { z } + k _ { i _ { z } } \displaystyle \int e _ { z } d t + k _ { d _ { z } } \dot { e } _ { z } + \mathcal { g } } } \\ { { } } & { { } } & { { } } \\ { { \Delta \theta _ { x } } } & { { = } } & { { k _ { p _ { s } } e _ { x } + k _ { i _ { \theta _ { s } } } \displaystyle \int e _ { x } d t + k _ { d _ { e } } \dot { e } _ { x } } } \\ { { } } & { { } } & { { } } \\ { { \Delta \theta _ { y } } } & { { = } } & { { k _ { p _ { s } } e _ { y } + k _ { i _ { e } } \displaystyle \int e _ { y } d t + k _ { d _ { s } } \dot { e } _ { y } } } \end{array}\tag{25}
$$

Here, $k _ { p _ { i } } , k _ { i _ { i } }$ and $k _ { d _ { i } } \ \forall i \in \{ x , y , z \} , k _ { p _ { \theta _ { i } } } , k _ { i _ { \theta _ { i } } }$ , and $k _ { d _ { \theta _ { i } } } \ \forall i \in \{ x , y \}$ are the proportional, integral and derivative gains for the position controller. The angular speed required for individual propeller motors necessary for hovering and motion along the z-axis is given by (26).

$$
\omega _ { h } = \sqrt { \frac { m \ddot { r } _ { z } ^ { d e s } } { k _ { f } ( c \theta _ { 1 } + c \theta _ { 2 } + c \theta _ { 3 } + c \theta _ { 4 } ) } }\tag{26}
$$

The body accelerations are represented by $a _ { b _ { i } } ~ \forall i \in \{ x , y , z \}$ for the system. The objective is to determine the quaternion which will align the body acceleration vector $\begin{array} { r l } { a _ { b } } & { { } = } \end{array}$ $[ a _ { b _ { x } } \quad a _ { b _ { v } } \quad a _ { b _ { z } } ] ^ { T }$ along the desired inertial acceleration vector $\ddot { r } ^ { d e s } ~ = ~ [ \ddot { r _ { x } } ^ { d e s } ~ \ddot { r _ { y } } ^ { d e s } ~ \ddot { r _ { z } } ^ { d e s } ] ^ { T }$ from (25). This is achieved by normalizing the two vectors as $\hat { a _ { b } } = n o r m [ a _ { b } ] , \hat { a } _ { i } =$ $n o r m [ \ddot { r } ^ { d e s } ]$ and computing the required rotation Θ and axis of rotation ñ. The cosine, sine of the rotation angles and the axis of rotation can be calculated with vector multiplication operations [25]. The resulting quaternion is denoted by .

$$
\widetilde { \mathbb { q } } = \frac { 1 } { \sqrt { 2 ( 1 + \hat { a _ { b } } ^ { T } \hat { a _ { i } } ) } } \left[ \begin{array} { c } { 1 + \hat { a _ { b } } ^ { T } \hat { a } _ { i } } \\ { \hat { a _ { b } } \times \hat { a _ { i } } } \end{array} \right]\tag{27}
$$

The commanded orientation  can be corrected for desired yaw angle as discussed in [2] which yields $\mathbb { Q } ^ { d e s }$ . The desired quaternion is further normalized to $\hat { \mathbb { Q } } ^ { d e s }$ for commanding the attitude controller loop. The control allocation matrix for the entire system is shown in (28). The attitude control elements are derived from the state space formulation discussed earliar. The rotor speeds are represented by $\omega _ { i } = \omega _ { h } + \Delta \omega _ { j }$ , Vi $\{ 1 , 2 , 3 , 4 \} , j \in \{ \phi , \theta , \psi \}$ . Similarly, the rotor tilt angles are given as $\theta _ { i } = \theta _ { h } + \Delta \theta _ { j } , \forall i \in \{ 1 , 2 , 3 , 4 \} , j \in \{ \phi , \theta , \psi , x , y \}$

$$
\left[ \begin{array} { c } { \Delta \omega _ { 1 } } \\ { \Delta \omega _ { 2 } } \\ { \Delta \omega _ { 3 } } \\ { \Delta \omega _ { 4 } } \\ { \Delta \theta _ { 1 } } \\ { \Delta \theta _ { 2 } } \\ { \Delta \theta _ { 3 } } \\ { \Delta \theta _ { 4 } } \end{array} \right] = \left[ \begin{array} { c c c c c c c c c } { 0 } & { - 1 } & { - 1 } & { 0 } & { 0 } & { 0 } & { 0 } & { 0 } & { 0 } \\ { 1 } & { 0 } & { 1 } & { 0 } & { 0 } & { 0 } & { 0 } & { 0 } & { 0 } \\ { 0 } & { 1 } & { - 1 } & { 0 } & { 0 } & { 0 } & { 0 } & { 0 } & { 0 } \\ { - 1 } & { 0 } & { 1 } & { 0 } & { 0 } & { 0 } & { 0 } & { 0 } & { 0 } \\ { 0 } & { 0 } & { 0 } & { 0 } & { 1 } & { - 1 } & { 0 } & { - 1 } \\ { 0 } & { 0 } & { 0 } & { 1 } & { 0 } & { - 1 } & { 1 } & { 0 } \\ { 0 } & { 0 } & { 0 } & { 0 } & { 1 } & { 1 } & { 0 } & { - 1 } \\ { 0 } & { 0 } & { 0 } & { 1 } & { 0 } & { 1 } & { 1 } & { 0 } \end{array} \right] \left[ \begin{array} { c } { \Delta \omega _ { \phi } } \\ { \Delta \omega _ { \theta } } \\ { \Delta \omega _ { \psi } } \\ { \Delta \theta _ { \theta } } \\ { \Delta \theta _ { \theta } } \\ { \Delta \theta _ { \psi } } \\ { \Delta \theta _ { \psi } } \end{array} \right]\tag{28}
$$

IV. NUMerical SiMULationS And Results

Here, the proposed controller is validated by numerical simulations. The mathematical model of the UAV and controller are developed in MATLAB and Simulink R2017a.

<!-- image-->
Fig. 3: Variation in Euler angles

<!-- image-->
Fig. 4: Variation in quaternion elements

The parameters used in the simulations are $m = 1 . 5 6 k g$ , l = 0.12m, $k _ { f } = 2 . 2 e - 4 N s / r a d .$ $k _ { m } = 5 . 4 e \mathrm { ~ - ~ } 6 N s / r a d .$ $I _ { x x } =$ $I _ { y y } = 0 . 0 4 4 9 k g m ^ { 2 }$ $I _ { z z } = 0 . 0 8 9 9 k g m ^ { 2 } .$ The first simulation considers application of step input to the roll, pitch and yaw axes of the UAV. The roll command of 1rad is issued at $t = 5 s .$ Similarly, the pitch command is issued at $t = 1 5 s$ and yaw command is issued at $t = 2 5 s$ Figure 3 and 5 show the variation of Euler angles and body rates w. r. t. step inputs. It can be seen in Fig. 4 that the second element of quaternion changes when roll step input is commanded to the UAV. Similarly, third and fourth element of quaternion respond to pitch and yaw step inputs respectively. Further, we validate the flight controller by simulating a way point navigation mission. The UAV is initialized at the origin and commanded to visit a predefined set of way points at a height of 5m. The set of way points are [5, 5], [5, 10], [10, 10], [15, 20], [20, 20]. The position controller generates necessary rotor-tilt and desired quaternion commands to minimize the position error. The three dimensional trajectory followed by the UAV is shown in figure 7. The UAV visits all way-points by maintaining the desired height. The tilt-rotor UAV has redundancy in control of position and orientation because the position control is achieved by changing the UAV orientation as well as rotor-tilt angles. Figure 8 shows the variation of angular speeds of the UAV propellers. Similarly, figure 10 shows the variation of rotortilt angles. The error-quaternion elements are shown in figure 9. The error-quaternion elements change while the UAV is navigating between way-points and they converge to the unit quaternion as the UAV reaches goal position.

<!-- image-->
Fig. 5: Body rates about $x _ { b } y _ { b } z _ { b } – \mathrm { a x e s }$

<!-- image-->

<!-- image-->
Fig. 6: Variation in rotor tilt angles

<!-- image-->
Fig. 7: Trajectory for WPN

<!-- image-->
Fig. 8: Variation of rotor speeds during WPN

<!-- image-->

<!-- image-->

<!-- image-->

<!-- image-->

Fig. 9: Quaternion error elements during WPN
<!-- image-->

<!-- image-->
Fig. 10: Variation in rotor-tilt angles during WPN

## V. ConcLusIon

In this paper, position and attitude controller for the tiltrotor quadcopter with quaternion feedback was presented. The UAV dynamics for translational and rotational motion were shown. Taylor series expansion about the hover condition was used to derive the necessary control allocation for the system. Lyapunov stability analysis of the attitude controller was presented. The performance of the quaternion feedback attitude controller was shown for reference attitude tracking. The inner quaternion feedback loop was commanded using an external position controller for a way-point mission. The complete control allocation and simulations were presented for achieving way-point navigation. Future work will involve experimental validation of the proposed flight controller and more studies will be conducted to exploit the redundant control inputs for achieving faulttolerant control during flight.

## REfeRencES

[1] N. Gupta, M. Kothari et al., "Flight dynamics and nonlinear control design for variable-pitch quadrotors," in American Control Conference (ACC), 2016. IEEE, 2016, pp. 31503155.

[2] M. Cutler and J. P. How, "Analysis and control of a variable-pitch quadrotor for agile flight," Journal of Dynamic Systems, Measurement, and Control, vol. 137, no. 10, p. 101002, 2015.

[3] M. Ryll, H. H. Bülthoff, and P. R. Giordano, "Modeling and control of a quadrotor uav with tilting propellers," in Robotics and Automation (ICRA), 2012. IEEE, 2012, pp. 46064613.

[4] R. Kumar, A. Nemati, M. Kumar, R. Sharma, K. Cohen, and F. Cazaurang, "Tilting-rotor quadcopter for aggressive fight maneuvers using differential flatness based flight controller," in ASME 2017 Dynamic Systems and Control Conference. American Society of Mechanical Engineers, 2017, pp. V003T39A006V003T39A006.

[5] R. Kumar, A. Nemati, M. Kumar, K. Cohen, and F. Cazaurang, "Position and attitude control by rotor tilt and rotor speed synchronization for single axis tilting-rotor quadcopter," in ASME 2017 Dynamic Systems and Control Conference. American Society of Mechanical Engineers, 2017, pp. V003T39A005V003T39A005.

[6] A. Nemati and M. Kumar, "Modeling and control of a single axis tilting quadcopter," in American Control Conference (ACC), 2014. IEEE, 2014, pp. 30773082.

[7] M. Ryll, H. H. Bülthoff, and P. R. Giordano, "A novel overactuated quadrotor unmanned aerial vehicle: Modeling, control, and experimental validation," IEEE Transactions on Control Systems Technology, vol. 23, no. 2, pp. 540556, 2015.

[8] S. Sridhar, R. Kumar, M. Radmanesh, and M. Kumar, "Non-linear sliding mode control of a tilting-rotor quadcopter," in ASME 2017 Dynamic Systems and Control Conference. American Society of Mechanical Engineers Digital Collection.

[9] S. Sridhar, G. Gupta, R. Kumar, M. Kumar, and K. Cohen, "Tiltrotor quadcopter xplored: Hardware based dynamics, smart sliding mode controller, attitude hold & wind disturbance scenarios," in 2019 American Control Conference (ACC). IEEE, 2019, pp. 20052010.

[10] A. Oosedo, S. Abiko, S. Narasaki, A. Kuno, A. Konno, and M. Uchiyama, "Flight control systems of a quad tilt rotor unmanned aerial vehicle for a large attitude change," in 2015 IEEE International Conference on Robotics and Automation (ICRA). IEEE, 2015, pp. 23262331.

[11] A. Franchi, R. Carli, D. Bicego, and M. Ryll, "Full-pose tracking control for aerial robotic systems with laterally bounded input force," IEEE Transactions on Robotics, vol. 34, no. 2, pp. 534541, 2018.

[12] D. Invernizzi and M. Lovera, "Trajectory tracking control of thrustvectoring uavs," Automatica, vol. 95, pp. 180186, 2018.

[13] A. Nemati, R. Kumar, and M. Kumar, "Stabilizing and control of tilting-rotor quadcopter in case of a propeller failure," in ASME 2016 dynamic systems and control conference. American Society of Mechanical Engineers Digital Collection.

[14] R. Kumar, S. Sridhar, F. Cazaurang, K. Cohen, and M. Kumar, "Reconfigurable fault-tolerant tilt-rotor quadcopter system," in ASME 2018 Dynamic Systems and Control Conference. American Society of Mechanical Engineers Digital Collection, 2018.

[15] S. Sridhar, R. Kumar, K. Cohen, and M. Kumar, "Fault tolerance of a reconfigurable tilt-rotor quadcopter using sliding mode control," in ASME 2018 Dynamic Systems and Control Conference. American Society of Mechanical Engineers Digital Collection.

[16] B. N. Pamadi, Performance, stability, dynamics, and control of airplanes. American Institute of aeronautics and astronautics, Incorporated, 2015.

[17] E. Fresk and G. Nikolakopoulos, "Full quaternion based attitude control for a quadrotor," in Control Conference (ECC), 2013 European. IEEE, 2013, pp. 38643869.

[18] E. Stingu and F. Lewis, "Design and implementation of a structured fight controller for a 6dof quadrotor using quaternions," in Control and Automation, 2009. MED'09. 17th Mediterranean Conference on. IEEE, 2009, pp. 12331238.

[19] J. Carino, H. Abaunza, and P. Castillo, "Quadrotor quaternion control," in Unmanned Aircraft Systems (ICUAS), 2015 International Conference on. IEEE, 2015, pp. 825831.

[20] E. Fresk and G. Nikolakopoulos, "Experimental evaluation of a full quaternion based attitude quadrotor controller," in Emerging Technologies & Factory Automation (ETFA), 2015 IEEE 20th Conference on. IEEE, 2015, pp. 14.

[21] M. Bhargavapuri, A. K. Shastry, H. Sinha, S. R. Sahoo, and M. Kothari, "Vision-based autonomous tracking and landing of a fullyactuated rotorcraft," Control Engineering Practice, vol. 89, pp. 113 129, 2019.

[22] B. L. Stevens, F. L. Lewis, and E. N. Johnson, Aircraft control and simulation: dynamics, controls design, and autonomous systems. John Wiley & Sons, 2015.

[23] N. Michael, D. Mellinger, Q. Lindsey, and V. Kumar, "The grasp multiple micro-uav testbed," Robotics & Automation Magazine, IEEE, vol. 17, no. 3, pp. 5665, 2010.

[24] J. L. Junkins and H. Schaub, Analytical mechanics of space systems. American Institute of Aeronautics and Astronautics, 2009.

[25] A. G. Kehlenbeck, "Quaternion-based control for aggressive trajectory tracking with a micro-quadrotor uav," Ph.D. dissertation, 2014.