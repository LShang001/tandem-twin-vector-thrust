# Robust flight control using Hybrid Incremental Nonlinear Dynamic Inversion Control for Tiltrotor UAV

Hoijo Jeaong, Jinyoung Suk\*, and Seungkeun Kim

Chungnam National Univeristy, 99 Daehak-ro, Yuseong-gu, Daejeon 34134, Republic of Korea

## ABSTRACT

This paper uses a complementary filter to design a hybrid incremental nonlinear dynamic inversion controller as a model-based and sensorbased angular acceleration estimation method. The dynamic modeling is performed on the aerodynamic data of the tiltrotor UAV and the propulsion system. By acquiring the dynamic characteristic information of the system, the crossfrequency of the complementary filter is applied. This paper checks the control effectiveness of the hybrid incremental nonlinear dynamic inversion. The control performance against the doublet attitude command is compared for a proportional-integral-differential controller, nonlinear dynamic inversion, and incremental nonlinear dynamic inversion.

## 1 INTRODUCTION

In recent years, Unmanned Aerial Vehicles (UAVs) have been widely used for civilian and military purposes, such as traffic monitoring, delivery missions, and geographic surveying. They can reduce operating costs by replacing humancrewed aircraft exposed to repetitive missions or dangerous environments [1, 2]. UAVs may require robust control by disturbance depending on the mission environment. Moreover, it may be designed as a nonlinear, highly coupling, uncertain, time-varying system depending on the form of the UAV. Typical control methods have become challenging to meet the system’s good performance. Therefore, a control method used for flight control using angular acceleration by differentiating the angular velocity measured in the gyroscope has been proposed [3]. After the control concept using angular acceleration for fighter VAAC came out, it is possible to improve the system’s robustness by feedback angular acceleration, such as incremental nonlinear dynamic inversion (INDI) [4, 5], the filter of angular acceleration with noise [6].

There are three main advantages of applying the angular acceleration feedback to the control system.

1. Acceleration or angular acceleration has a close relationship with the force and moment applied to the system according to Newton’s law. The effect of vibration or disturbance applied to the system is related to the angular acceleration, which can be used to eliminate model uncertainty and disturbance of the system.

2. INDI is simplified for the information required by the controller in the affine equation of state. It reduces the system’s dependence and is robust against modeling errors and uncertainties.

3. Since there is no model dependency compared to nonlinear dynamic inversion (NDI), there is no need for complex gain scheduling. There is no heavy workload for designing flight control laws.

One of the challenges in INDI is how to obtain accurate angular acceleration signals [4]. Accurate angular acceleration is key to reinforcing the robustness of INDI. In general, angular acceleration is calculated numerically by differentiating the angular velocity measured by the gyroscope [7, 8]. At this time, noise increases, and a time delay occurs by the filter, reducing the stability of the flight control system (FCS). Many researchers have done much research on these issues. In Reference [9], the methods for measuring angular acceleration are divided into two methods: direct and indirect. The indirect method is to estimate the angular acceleration using the velocity, and the primary purpose is to attenuate the angular acceleration noise. In addition, studies are using predictive filters [10] and Kalman filters [11]. The above method has many parameters that need to be set, and there is a limit to reflect the system characteristics. In addition, a low-pass filter is usually designed to reduce the influence of noise, but it has a significant disadvantage in terms of time delay.

Motivated by the discussions mentioned above, in this paper, accurate angular acceleration is obtained in a relatively simple way using a complementary filter. In the following, we summarize the significant contributions of this paper:

• Numerical modeling and trim analysis for a tiltrotor UAV.

• Application of cross-frequency of complimentary filter based on dynamic characteristic information obtained in fixed-wing flight mode

• The observer is designed a hybrid incremental nonlinear dynamic inversion (HINDI) controller using modelbased and sensor-based angular acceleration signals.

• Comparison of attitude tracking performance of proportional-integral-differential controller (PID),

NDI, and INDI controllers for HINDI performance comparison.

This paper is structured as follows: In Section 2, numerical modeling of the tiltrotor UAV is performed through aerodynamic data and propulsion system modeling. In Section 3, the method of obtaining angular acceleration is divided into sensor-based INDI and model-based INDI. Moreover, an accurate angular acceleration observer is designed using the complementary filter and model-based and sensor-based angular acceleration information. Section 4 compares the performance of PID, NDI, INDI, and HINDI to compare attitude tracking performance. Finally, Section 5 concludes this paper.

## 2 NUMERICAL MODELING

## 2.1 Aerodynamic Modeling

<!-- image-->
Figure 1: DATCOM Longitudinal (Left), Lateral (Right)

External forces and moments are essential factors to ensure the modeling reliability of the platform to be modeled. The factors of external force and moment acting on each axis include aerodynamic effect, rotor thrust, gravity, and aerodynamic force. The aerodynamic coefficient is generally calculated theoretically or obtained by an experimental approach through a CFD technique or a wind tunnel test. In this study, static/dynamic non-dimensional aerodynamic coefficients are calculated using Digital DATCOM software [12]. The DAT-COM can calculate static stability and control surface deflection characteristics and includes an option to calculate the trim at subsonic Mach number.

Equations 1-7 show stability derivatives and force and moment coefficients of the tiltrotor UAV used in this study. Here, ${ \delta _ { e v } } ^ { R }$ and $\delta _ { e v } { } ^ { L }$ , mean the deflection angles of the left and right elevons and $x _ { C , G }$ and $x _ { A . C }$ mean the positions of the center of gravity and the center of aerodynamics based on the body coordinate system, respectively. If the left and right elevons are deflected in the same direction and magnitude, the sum of the rolling moments due to the $C _ { L _ { \delta _ { e \tau } } }$ becomes zero. Conversely, if the same magnitude deflects the left and right ailerons in opposite directions, the pitching moment due to

the $C _ { M _ { \delta _ { e v } } }$ becomes zero [13].

$$
\begin{array} { r l r } & { } & { C _ { L } = C _ { L _ { 0 } } + C _ { L _ { \alpha } } ( V _ { T } , \alpha ) \cdot \alpha + \cfrac { 1 } { 2 } C _ { L _ { \delta _ { e v } } R } ( V _ { T } , { \delta _ { e v } } ^ { R } ) \cdot \Delta { \delta _ { e v } } ^ { R } } \\ & { } & { \qquad + \cfrac { 1 } { 2 } C _ { L _ { \delta _ { e v } } L } \left( V _ { T } , { \delta _ { e v } } ^ { L } \right) \cdot \Delta { \delta _ { e v } } ^ { L } + \cfrac { \bar { c } } { 2 V _ { T } } C _ { L _ { q } } ( V _ { T } , \alpha ) \cdot q } \end{array}\tag{1}
$$

$$
\begin{array} { c } { { C _ { D } = C _ { D _ { 0 } } + \displaystyle \frac { 1 } { 2 } C _ { D _ { \delta _ { e v } } R } \left( V _ { T } , \delta _ { e v } { } ^ { R } \right) \cdot \Delta \delta _ { e v } { } ^ { R } \qquad \mathrm { ( } } } \\ { { \mathrm { } + \displaystyle \frac { 1 } { 2 } C _ { D _ { \delta _ { e v } } L } \left( V _ { T } , \delta _ { e v } { } ^ { L } \right) \cdot \Delta \delta _ { e v } { } ^ { L } \qquad \quad } } \\ { { C _ { Y } = C _ { Y _ { \beta } } \cdot \displaystyle \beta + \frac { b } { 2 V _ { T } } ( C _ { Y _ { p } } ( V _ { T } , \alpha ) \cdot p + C _ { Y _ { r } } ( V _ { T } , \alpha ) \cdot r ) \qquad } } \end{array}\tag{2}
$$

(3)

$$
\begin{array} { l } { \displaystyle C _ { l } = C _ { l _ { \beta } } ( V _ { T } , \alpha ) \cdot \beta + \frac { 1 } { 2 } C _ { l _ { \delta _ { e v } } R } ( V _ { T } , { \delta _ { e v } } ^ { R } ) \cdot \Delta { \delta _ { e v } } ^ { R } } \\ { \displaystyle ~ - \frac { 1 } { 2 } C _ { l _ { \delta _ { e v } } L } ( V _ { T } , { \delta _ { e v } } ^ { L } ) \cdot \Delta { \delta _ { e v } } ^ { L } } \\ { \displaystyle ~ + \frac { b } { 2 V _ { T } } ( C _ { l _ { p } } ( V _ { T } , \alpha ) \cdot p + C _ { l _ { r } } ( V _ { T } , \alpha ) \cdot r ) } \end{array}\tag{4}
$$

$$
\begin{array} { l } { { \displaystyle C _ { M } = C _ { M _ { 0 } } + C _ { M _ { \alpha } } ( V _ { T } , \alpha ) \cdot \alpha + \frac { 1 } { 2 } C _ { M _ { \delta _ { e v } } R } ( V _ { T } , { \delta _ { e v } } ^ { R } ) } } \\ { { \displaystyle ~ \cdot \Delta { \delta _ { e v } } ^ { R } + \frac { 1 } { 2 } C _ { M _ { \delta _ { e v } } L } \left( V _ { T } , { \delta _ { e v } } ^ { L } \right) \cdot \Delta { \delta _ { e v } } ^ { L } + } } \\ { { \displaystyle \frac { \bar { c } } { 2 V _ { T } } C _ { M _ { q } } ( V _ { T } , \alpha ) \cdot q + A } } \end{array}\tag{5}
$$

$$
C _ { N } = C _ { N _ { \beta } } \cdot \beta + \frac { b } { 2 V _ { T } } ( C _ { N _ { p } } ( V _ { T } , \alpha ) \cdot p + C _ { N _ { r } } ( V _ { T } , \alpha ) \cdot r ) - B\tag{6}
$$

where

$$
\begin{array} { l } { { A = ( x _ { C . G } - x _ { A . C } ) ( { C _ { L } } ^ { T o t a l } \cos \alpha + { C _ { D } } ^ { T o t a l } \sin \alpha ) } } \\ { { B = ( x _ { C . G } - x _ { A . C } ) \displaystyle \frac { \bar { c } } { b } { C _ { Y } } ^ { T o t a l } \cos \alpha . } } \end{array}\tag{7}
$$

## 2.2 Dynamic modeling

The dynamic modeling of the tiltrotor UAV is derived by forces and moments acting on the vehicle. Each force and moment consider the components of the rotor, the elevons, the blend-wing. This paper assumes constant mass and mass distribution during flight. The configuration of the tiltrotor UAV is shown in Figure 2. $X _ { E } , Y _ { E }$ and $Z _ { E }$ axes of the inertial coordinate system are defined as a right-handed Cartesian coordinate with North, East, and Downward directions, abbreviated as the NED coordinate. The dynamics can be represented as

$$
{ \dot { x } } ( t ) = f ( x ( t ) , u ( t ) )\tag{8}
$$

where

$$
\begin{array} { r }  \begin{array} { l } { x = [ \begin{array} { l l l l l l l l } { u } & { v } & { w } & { p } & { q } & { r } & { \phi } & { \theta } & { \psi } \end{array} ] ^ { T } , } \\ { u = [ \begin{array} { l l l l l l l } { R P M _ { 1 } } & { R P M _ { 2 } } & { R P M _ { 3 } } & { R P M _ { 4 } } & { \delta _ { e v } ^ { R } } & { \delta _ { e v } ^ { L } } \end{array} ] } \end{array} \end{array}\tag{ξ ],}
$$

(9)

<!-- image-->
Figure 2: Coordinate System of Tiltrotor UAV

, and $u , v ,$ &w and $p , q .$ &r are the projections of velocity and angular velocity onto the $X _ { B } , Y _ { B }$ , and $Z _ { B }$ axes of the bodyaxis system. Euler angles are presented by $\phi , \theta ,$ and $\psi .$ Six degree-of-freedom nonlinear equations of motion are derived by reflecting total force and moment as

$$
\begin{array} { r l } & { \dot { u } = r v - q w - g \sin \theta + F _ { x } / m } \\ & { \dot { v } = p w - r u + g \sin \phi \cos \theta + F _ { y } / m } \\ & { \dot { w } = q u - p v + g \cos \phi \cos \theta + F _ { z } / m } \\ & { \dot { p } = ( L + q r ( I _ { y } - I _ { z } ) ) / I _ { x } } \\ & { \dot { q } = ( M + r q ( I _ { z } - I _ { x } ) ) / I _ { y } } \\ & { \dot { r } = ( N + p q ( I _ { x } - I _ { y } ) ) / I _ { z } } \end{array}\tag{10}
$$

Here, the force and moment in each axis are represented as:

$$
\begin{array} { r l } & { \bar { F } _ { B } = \left[ \begin{array} { l } { F _ { x } } \\ { F _ { y } } \\ { F _ { z } } \end{array} \right] = \bar { F } _ { B _ { T } } + \bar { F } _ { B _ { A } } } \\ & { = \left[ \begin{array} { c } { T _ { j _ { 3 } } \cos \zeta + T _ { f _ { 4 } } \cos \zeta } \\ { 0 } \\ { T _ { f _ { 1 } } + T _ { f _ { 2 } } + T _ { f _ { 3 } } \sin \zeta + T _ { f _ { 4 } } \sin \zeta } \end{array} \right] + } \\ & { \quad \left[ \begin{array} { c } { Q S C _ { L } \sin \alpha - Q S C _ { D } \cos \alpha } \\ { Q S C _ { Y } } \\ { - Q S C _ { L } \cos \alpha - Q S C _ { D } \sin \alpha } \end{array} \right] } \end{array}\tag{11}
$$

$$
\bar { M } _ { B } = \left[ \begin{array} { c } { { L } } \\ { { M } } \\ { { N } } \end{array} \right] = \bar { M } _ { B _ { T } } + \bar { M } _ { B _ { g } } + \bar { M } _ { B _ { A } }\tag{12}
$$

where $Q = 0 . 5 \rho V ^ { 2 }$ , and

$$
\bar { M } _ { B _ { T } } = \left[ \begin{array} { c } { { T _ { f _ { 1 } } L _ { 2 } - T _ { f _ { 2 } } L _ { 2 } + T _ { f _ { 3 } } L _ { 4 } \sin \zeta } } \\ { { - T _ { f _ { 4 } } L _ { 4 } \sin \zeta } } \\ { { T _ { f _ { 1 } } L _ { 1 } + T _ { f _ { 2 } } L _ { 1 } - T _ { f _ { 3 } } L _ { 3 } \sin \zeta } } \\ { { - T _ { f _ { 4 } } L _ { 3 } \sin \zeta } } \\ { { - T _ { q _ { 1 } } L _ { 2 } + T _ { q _ { 2 } } L _ { 2 } + T _ { q _ { 3 } } L _ { 4 } - T _ { q _ { 4 } } L _ { 4 } } } \\ { { + T _ { f _ { 3 } } \cos \zeta \sin \delta _ { g } \sqrt { L _ { 3 } ^ { 2 } + L _ { 4 } ^ { 2 } } } } \\ { { - T _ { f _ { 4 } } \cos \zeta \sin \delta _ { g } \sqrt { L _ { 3 } ^ { 2 } + L _ { 4 } ^ { 2 } } } } \end{array} \right]\tag{13}
$$

$$
\bar { M } _ { B _ { g } } = \left[ \begin{array} { c } { { - J _ { r } q ( - \Omega _ { 1 } + \Omega _ { 2 } } } \\ { { + \Omega _ { 3 } \sin \zeta - \Omega _ { 4 } \sin \zeta ) } } \\ { { - J _ { r } r ( \Omega _ { 3 } - \Omega _ { 4 } ) \cos \zeta } } \\ { { + J _ { r } p ( - \Omega _ { 1 } + \Omega _ { 2 } + \Omega _ { 3 } \sin \zeta - \Omega _ { 4 } \sin \zeta ) } } \\ { { J _ { r } q ( \Omega _ { 3 } - \Omega _ { 4 } ) \cos \zeta } } \end{array} \right]\tag{14}
$$

$$
\begin{array} { l } { \displaystyle \bar { M } _ { B _ { A } } = \left[ \begin{array} { c } { Q S b C _ { l } } \\ { Q S c C _ { m } } \\ { Q S b C _ { n } } \end{array} \right] + } \\ { \displaystyle \left[ \begin{array} { c } { - ( x _ { C G _ { Z } } - x _ { A C _ { Z } } ) \bar { F } _ { B _ { A _ { Y } } } } \\ { ( x _ { C G _ { Z } } - x _ { A C _ { Z } } ) \bar { F } _ { B _ { A _ { X } } } - ( x _ { C G _ { Z } } - x _ { A C _ { Z } } ) \bar { F } _ { B _ { A _ { Z } } } } \\ { ( x _ { C G _ { Z } } - x _ { A C _ { Z } } ) \bar { F } _ { B _ { A _ { Y } } } } \end{array} \right] . } \end{array}\tag{15}
$$

Here, $\bar { F } _ { B }$ and $\bar { M } _ { B }$ are forces and moments based on the body coordinate system, $T _ { n _ { 1 } \sim 4 }$ means the rotor’s thrust, and $\zeta$ means the tilt angle of the rear rotor [14]. $L _ { 1 \sim 4 }$ are the horizontal and vertical distances from the rotor to the center of gravity. The torque component resulting from the rotor consists of each pair’s thrust difference and the gyroscopic effect [15].

## 3 CONTROL DESIGN

## 3.1 Sensor-Based INDI

The INDI is a concept expressed in the incremental form to improve the robustness of the closed-loop system and reduce the dependence on system information compared to NDI. Unlike NDI, which requires accurate model information in a closed-loop dynamic system, the INDI uses a state feedback structure. The general nonlinear control-affine system equation of an aircraft is as follows:

$$
\begin{array} { l } { \dot { x } = f ( x ) + g ( x ) u } \\ { \operatorname { y } = \operatorname { h } ( \operatorname { x } ) } \end{array}\tag{16}
$$

where $x \in \mathbb { R } ^ { n }$ represents the state vector, $u \in \mathbb { R } ^ { m }$ the input vector, $\boldsymbol { y } \in \mathbb { R } ^ { d }$ the output vector, f and h are smooth vector fields, and $g \in \mathbb { R } ^ { n \times m }$ is a control distribution function. Equation 16 is approximated using Taylor series expansion at the current state value $x _ { o }$ and the control input $u _ { o }$

$$
\begin{array} { l } { { \displaystyle \dot { x } \approx f ( x _ { 0 } ) + g ( x _ { 0 } ) u _ { 0 } + \frac { \partial } { \partial x } [ f ( x ) + g ( x ) u ] _ { u _ { 0 } , x _ { 0 } } ( x - x _ { 0 } ) } } \\ { { \displaystyle \qquad + \frac { \partial } { \partial u } [ f ( x ) + g ( x ) u ] _ { u _ { 0 } , x _ { 0 } } ( u - u _ { 0 } ) . } } \end{array}\tag{17}
$$

$x _ { o }$ and $u _ { o }$ are incremental instances of x and u of the previous time step, respectively. The higher-order term is assumed to be negligible, so it is not expressed in Equation 17. Moreover, in general, the change in the one-step delayed state in aircraft application control is tiny and therefore negligible. Furthermore, the control input u is faster than the state x, and according to the time scale separation principle, it can be simplified as:

$$
\dot { x } \approx \dot { x } _ { 0 } + g ( x _ { 0 } ) \Delta u\tag{18}
$$

where the increment in control input is defined as $\Delta u = u -$ $u _ { o }$ . The incremental control law is derived from Equation 18 and assumes that the state variable equals the output value. Here, ${ \dot { x } } _ { 0 }$ is the angular acceleration in the previous time step, and x˙ is expressed as $\nu$ in Equation 17. If the inverse of $g ( x )$ exists, the control increment is represented as:

$$
\Delta u = g ( x _ { 0 } ) ^ { - 1 } ( \nu - \dot { x } _ { 0 } )\tag{19}
$$

where $\nu$ is a dynamic characteristic that reflects the system design requirements and becomes the target dynamics. Based on the previous control input through Equation 19, the new control input is as follows:

$$
u = u _ { 0 } + g ( x _ { 0 } ) ^ { - 1 } ( \nu - \dot { x } _ { 0 } ) .\tag{20}
$$

Substituting Equation 20 into Equation 17 gives:

$$
{ \dot { x } } = \nu .\tag{21}
$$

The above equation shows that the existing system dynamic characteristics are removed, and characteristics according to the designer or system requirements are followed. In addition, the obtained control law does not depend on $f ( x )$ and requires more nominal information about the system model than NDI. The sensor-based INDI is a method of directly acquiring ${ \dot { x } } _ { 0 }$ based on sensor measurement. Since the information measured from the sensor contains uncertain information that cannot be accurately expressed mathematically, it is a very robust control concept for compensating for model uncertainty.

## 3.2 Model-Based INDI

The model-based INDI is formulated by the same derivation procedure as sensor-based INDI and is a method of calculating the angular acceleration or secondary time-varying of the current system by mathematical modeling. The modeled dynamic $f _ { m d i } ( x _ { 0 } ) + g _ { m d i } ( x _ { 0 } ) u _ { 0 }$ replaces $f ( x _ { 0 } ) + g ( x _ { 0 } ) u _ { 0 }$ in Equation 17, and the derived control law is as follows:

$$
\begin{array} { l } { { u = u _ { 0 } + g _ { m d i } ( x _ { 0 } ) ^ { - 1 } ( \nu - \dot { X } ) } } \\ { { \ = u _ { 0 } + g _ { m d i } ( x _ { 0 } ) ^ { - 1 } ( \nu - f _ { m d i } ( x _ { 0 } ) - g _ { m d i } ( x _ { 0 } ) u _ { 0 } ) } } \end{array}\tag{22}
$$

where $\dot { X } = f _ { m d i } ( x _ { 0 } ) + g _ { m d i } ( x _ { 0 } ) u _ { 0 }$ means the mathematical modeling equation of the system. In model-based INDI, the increment $u _ { 0 }$ is removed, so the expression is simplified as:

$$
u = g _ { m d i } ( x _ { 0 } ) ^ { - 1 } ( \nu - f _ { m d i } ( x _ { 0 } ) ) .\tag{23}
$$

The model-based INDI is called On-Board Model (OBM). The linear control law consists of linearized model information for each trim point in the system and a datasheet of trim state variables. A nonlinear system requires a relatively higher calculation than a linear system by mathematically expressing dynamic information.

## 3.3 Hybrid incremental nonlinear dynamic inversion

<!-- image-->
Figure 3: Control structure of the hybrid INDI control

A typical complementary filter reduces the effect of noise by combining multiple signals to use a wide frequency band of the signal. It is possible to generate a signal in which the noise characteristics of the low-frequency signal are subjected to high-pass filtered, and the high-frequency noise characteristics are subjected to low-pass filtered. In this paper, the sensor differentiation method and model prediction estimation method are considered to estimate acceleration or angular acceleration for dynamic system control. Both methods have their advantages and disadvantages. The model prediction estimation method is less noisy, has a little phase lag, and can compensate for the drawbacks of the sensor differentiation method. The sensor differentiation method can estimate the actual angular velocity or angular acceleration signal and does not depend on system model information. It is also possible to reflect disturbances and system uncertainties

$$
\begin{array} { c } { { \hat { \dot { x } } = [ \dot { x } _ { 2 } G ( s ) - \hat { \dot { x } } x G ( s ) ] [ H ( s ) ] + \dot { x } _ { 1 } } } \\ { { { } } } \\ { { = [ x _ { 2 } - \hat { \dot { x } } x \displaystyle \frac { 1 } { s } ] [ K _ { P } + \displaystyle \frac { K _ { I } } { S } ] + \dot { x } _ { 1 } } } \end{array}\tag{24}
$$

The Equation 24 can be rearranged as follows:

$$
\hat { \dot { x } } = L _ { f } ( s ) x _ { 2 } + H _ { f } ( s ) \dot { x } _ { 1 } .\tag{25}
$$

where $L _ { f } ( s ) { \boldsymbol { s } }$ and $H _ { f } ( s ) { \boldsymbol { \varepsilon } }$ s are

$$
L _ { f } ( s ) = \frac { K _ { P } s + K _ { I } } { s ^ { 2 } + K _ { P } s + K _ { I } } s , H _ { f } ( s ) = \frac { s ^ { 2 } } { s ^ { 2 } + K _ { P } s + K _ { I } } .\tag{26}
$$

## 4 SIMULATION AND RESULTS

## 4.1 Simulation conditions

As shown in Table 1, the simulation is performed using the trim conditions of the fixed-wing flight state. The attitude command is a doublet command, and the Butterworth filter method is applied to the command to prevent actuator saturation. To compare the performance of INDI proposed in this paper, the sensing noise of the gyroscope is added. Simula-

Table 1: Simulation conditions
<table><tr><td rowspan=1 colspan=1>Simulationtime(dt:0.01 s)</td><td rowspan=1 colspan=1> $0 \sim 6 0 s$ </td></tr><tr><td rowspan=1 colspan=1>Initialcondition</td><td rowspan=1 colspan=1> $\overline { { \phi = 0 \mathrm { d e g } } }$  $\theta = \alpha = 0 ~ \mathrm { d e g }$  $\psi = 0 ~ \mathrm { d e g }$  $\dot { p } = 0 ~ \deg / s$  ${ \dot { q } } = 0 ~ \deg / s$  $\dot { r } = 0 ~ \deg / s$  $V = 2 5 m / s$ </td></tr><tr><td rowspan=1 colspan=1>Controlsurfaces</td><td rowspan=1 colspan=1> $\overline { { \Omega _ { 1 } = 0 } }$  $\Omega _ { 2 } = 0$  $\Omega _ { 3 } = 2 2 9 2$  $\Omega _ { 4 } = 2 2 9 2$  $\delta _ { e v } ^ { R } = - 1 . 4 3 ~ \mathrm { d e g }$  $\delta _ { e v } ^ { L } = - 1 . 4 3 ~ \mathrm { d e g }$ </td></tr><tr><td rowspan=1 colspan=1>Attitude command(Butterworth filter)</td><td rowspan=1 colspan=1> $\theta _ { c m d } = 1 0 ^ { \circ } ~ ( 2 0 \sim 3 0 s )$  $\theta _ { c m d } = - 1 0 ^ { \circ } ~ ( 3 0 \sim 4 0 s )$ </td></tr><tr><td rowspan=1 colspan=1>Gyro sensornoise(Gaussian Distribution)</td><td rowspan=1 colspan=1> $\begin{array} { r } { \ddot { x } _ { n } ( x _ { n } ) = \frac { 1 } { \sigma \sqrt { 2 \pi } } e ^ { - \frac { 1 } { 2 } \left( \frac { x _ { n } - \mu } { \sigma } \right) ^ { 2 } } } \end{array}$  $w h e r e \sigma = 1 , \mu = 0$ </td></tr></table>

Table 2: Scenarios in Simulation
<table><tr><td>Scenarious</td><td></td><td>Actuator model Angular velocity noise</td></tr><tr><td>Case1</td><td>X</td><td>0</td></tr><tr><td>Case2</td><td>0</td><td>X</td></tr><tr><td>Case3</td><td>0</td><td>0</td></tr></table>

tions are performed in three scenarios. Depending on each scenario, actuator modeling is included, or noise is added to the angular velocity measurement. The PID, NDI, and INDI controllers are performed together on the same baseline control gain to compare the attitude tracking performance.

## 4.2 Case1. w/o Actuator model, w/o angluar velocity noise

Figure 4 shows the results of case 1 is a scenario without an actuator model and angular velocity noise. The PID controller shows a significant offset for the angle command. Conversely, NDI shows excellent tracking performance. However, the NDI controller tracks the attitude with a constant offset from the beginning of the simulation time. INDI and HINDI have attitude tracking performance similar to NDI. However, the HINDI has some oscillation in the beginning. This phenomenon is considered an error occurring in the initial simulation condition when the actuator value of the previous step is fed back.

Table 3 compares the root-mean-square error (RMSE). The angle error has the smallest in the NDI, but the angular velocity error has the smallest in the HINDI. In case 1, since there is no actuator model and angular velocity noise, the NDI tracking performance that actively uses model information is excellent.

## 4.3 Case2. w/ Actuator model, w/o angluar velocity noise

Figure 5 represents case 2 with the actuator model and no angular velocity noise. Having an actuator model means it takes time for the actuator to move to the target angle. Therefore, the model uncertainty problem appears among the disadvantages of the NDI. In the case of NDI, large oscillations appear in the actuator from the beginning of the simulation, and if it is an actual flying vehicle, the actuator would be damaged. The attitude and angular velocity also exhibited significant vibrations, resulting in a greater decrease in altitude. In the case of the INDI, an undershoot occurs due to attitude tracking performance. However, it can be seen that the HINDI properly follows the doublet command, and the altitude drop is negligible without actuator saturation.

In Table 4, the attitude error of NDI shows a significant vibration in attitude and angular velocity. However, the angular velocity error is the largest compared to other controllers. In the case of the NDI controller, since significant vibrations continuously occur throughout the simulation, it is not a typical simulation result, so the system performance cannot be compared only with the RMSE. In contrast, the HINDI has an angular velocity error of less than 1 deg /s and shows the highest performance.

## 4.4 Case3. w/ Actuator model, w/ angluar velocity noise

Figure 6 displays the simulation result considering the actuator model and angular velocity noise. The angular velocity noise causes the overall vibration. Since the NDI controller is greatly affected by model uncertainty and uses the model information, the angular velocity mixed with noise appears as a control output. In the case of the PID, the NDI, and the INDI, the attitude tracking performance is worse than that of case 2, and the actuator output also generates vibration. Conversely, the HINDI has improved attitude tracking performance, and the actuator output also has less vibration than other controllers. However, since the sensor-based angular velocity mixed with noise is used, the effect of vibration occurring throughout the simulation cannot be eliminated. Using complementary filters is that angular acceleration can be observed in both model-based and sensor-based methods. This structure showed a more robust performance against the uncertainty of the INDI model.

As shown in Table 5, HINDI shows improved attitude and angular velocity tracking compared to other controllers. The HINDI maintains robust performance while noise concerning angular velocity is appropriately canceled. It is considered that it is unreasonable to use only RMSE to analyze the con- and case 3. trol performance of a system with a high vibration like case 2

<!-- image-->

<!-- image-->

<!-- image-->

<!-- image-->

<!-- image-->

<!-- image-->

Figure 4: Simulation result-case1. w/o actuator model, w/o angular velocity noise
<!-- image-->

<!-- image-->

<!-- image-->

<!-- image-->

<!-- image-->

<!-- image-->
Figure 5: Simulation result-case2. w/ actuator model, w/o angular velocity noise

<!-- image-->

<!-- image-->

<!-- image-->

<!-- image-->

<!-- image-->

<!-- image-->
Figure 6: Simulation result-case3. w/ actuator model, w/ angular velocity noise

Table 3: RMSE result of Case1
<table><tr><td>RMSE</td><td>PID</td><td>NDI</td><td>INDI</td><td>HINDI</td><td>Unit</td></tr><tr><td>θ Tracking error</td><td>2.305</td><td>0.753</td><td>2.212</td><td>1.883</td><td>deg</td></tr><tr><td>q Tracking error</td><td>2.628</td><td>8.258</td><td>1.333</td><td>0.960</td><td>deg/s</td></tr></table>

Table 4: RMSE result of Case2
<table><tr><td>RMSE</td><td>PID</td><td>NDI</td><td>INDI</td><td>HINDI</td><td>Unit</td></tr><tr><td>θ Tracking error</td><td>2.312</td><td>1.015</td><td>2.243</td><td>1.884</td><td>deg</td></tr><tr><td>q Tracking error</td><td>2.616</td><td>16.532</td><td>1.387</td><td>0.963</td><td>deg/s</td></tr></table>

Table 5: RMSE result of Case3
<table><tr><td>RMSE</td><td>PID</td><td>NDI</td><td>INDI</td><td>HINDI</td><td>Unit</td></tr><tr><td>θ Tracking error</td><td>2.327</td><td>1.088</td><td>2.253</td><td>1.920</td><td>deg</td></tr><tr><td>q Tracking error</td><td>3.144</td><td>18.921</td><td>2.952</td><td>2.667</td><td>deg/s</td></tr></table>

## 5 CONCLUSIONS AND FUTURE WORK

To improve the noise or time-delayed angular acceleration of the existing INDI controller, in this paper, a HINDI controller was designed using a complementary filter. For numerical simulation, aerodynamic data and propulsion system modeling of a tiltrotor UAV were performed. Moreover, sensor-based and model-based INDI was discussed for acquiring angular acceleration signals of the existing INDI. Combining the advantages of the sensor-based and the modelbased INDI, a complementary filter was applied to design the HINDI controller structure. The attitude tracking simulations of PID, NDI, INDI, and HINDI controllers were performed, and it was confirmed that HINDI had the best attitude tracking performance in the three simulation cases.

The proposed algorithm will be applied to a tiltrotor UAV to verify the robust attitude control in transition and fixedwing flight modes in a future study.

## ACKNOWLEDGEMENTS

This work was supported by the Korea Evaluation Institute of Industrial Technology (KEIT) grant funded by the Korea government (MTIE) (No.20016463).

This work was supported by the National Research Foundation of Korea (NRF) grant funded by the Korea government(MSIT) (No. NRF-2021R1A2C2013363).

This work was supported by the National Research Foundation of Korea (NRF) grant funded by the Korea government(MSIT)(No. 2021R1A5A1031868).

## REFERENCES

[1] Z. Liu, Y. He, L. Yang, and J. Han. Control techniques of tilt rotor unmanned aerial vehicle systems: A review.

Chinese Journal of Aeronautics, 30(1):135–148, 2017.

[2] N. T. Hegde, V. I. George, C. G. Nayak, and K. Kumar. Design, dynamic modelling and control of tiltrotor uavs: a review. International Journal of Intelligent Unmanned Systems, 8(3), 2019.

[3] P Smith. A simplified approach to nonlinear dynamic inversion based flight control. In 23rd Atmospheric Flight Mechanics Conference, 1998.

[4] S. Sieberling, Q. P. Chu, and J. A. Mulder. Robust flight control using incremental nonlinear dynamic inversion and angular acceleration prediction. Journal of guidance, control, and dynamics, 33(6):1732–1742, 2010.

[5] P. Acquatella, W. Falkena, E. J. van Kampen, and Q. P. Chu. Robust nonlinear spacecraft attitude control using incremental nonlinear dynamic inversion. In AIAA Guidance, Navigation, and Control Conference, page 4623, 2012.

[6] Barton Bacon and Aaron Ostroff. Reconfigurable flight control using nonlinear dynamic inversion with a special accelerometer implementation. In AIAA Guidance, Navigation, and Control Conference and Exhibit, 2000.

[7] Hong-Xing Deng, Xian-Bin Wang, and Ka Liu. Design of an angular acceleration boundary observer. Journal of Harbin Institute of Technology, 42:1504–1508, 2010.

[8] Wang Wanjun Zhang Weigong Li Xu. Inertia electrical emulation and angular acceleration estimation for transmission test rig. Journal of Southeast University (Natural Science Edition), 42:62–66, 2012.

[9] Seppo J. Ovaska and Sami Valiviita. Angular acceleration measurement: A review. In IEEE Instrumentation and Measurement Technology Conference, 1998.

[10] Jari Pasanen, Olli Vainio, and Seppo J. Ovaska. Predictive synchronization and restoration of corrupted velocity samples. Measurement, 13(4):315–324, 1994.

[11] P. R. Belanger, P. Dobrovolny, A. Helmy, and X. Zhang. Estimation of angular velocity and acceleration from shaft-encoder measurements. The International Journal of Robotics Research, 17(11):1225–1233, 1998.

[12] W Blake. Prediction of fighter aircraft dynamic derivatives using digital datcom. In 3rd Applied Aerodynamics Conference, 1985.

[13] B. M. Min, S. S. Shin, H. C. Shim, and M. J. Tahk. Modeling and autopilot design of blended wing-body uav. International Journal of Aeronautical and Space Sciences, 9(1):121–128, 2008.

[14] R. Czyba, M. Lemanowicz, Z. Gorol, and T. Kudala. Construction prototyping, flight dynamics modeling, and aerodynamic analysis of hybrid vtol unmanned aircraft. Journal of Advanced Transportation, 2018.

[15] P. J. Shao, W. H. Dong, X. X. Sun, T. J. Ding, and Q. Zou. Dynamic surface control to correct for gyroscopic effect of propellers on quadrotor. In International Journal of Aeronautical and Space Sciences, 2015.