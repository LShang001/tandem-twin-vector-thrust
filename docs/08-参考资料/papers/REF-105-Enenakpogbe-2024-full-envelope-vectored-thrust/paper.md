Article

# Full Envelope Control of Over-Actuated Fixed-Wing Vectored Thrust eVTOL <sup>†</sup>

Emmanuel Enenakpogbe \* , James F. Whidborne \* and Linghai Lu

Centre for Aeronautics, Cranfield University, Cranfield MK43 0AL, UK; l.lu@cranfield.ac.uk

\* Correspondence: e.enenakpogbe@cranfield.ac.uk (E.E.); j.f.whidborne@cranfield.ac.uk (J.F.W.)

<sup>†</sup> This paper is an extension of work originally presented in 2024 UKACC 14th International Conference on Control (CONTROL) as “Control of an Over-actuated Fixed-wing Vectored Thrust eVTOL”. It focusses on the verification results of previously proposed techniques.

Abstract: A novel full-envelope controller for an over-actuated fixed-wing vectored thrust eVTOL aircraft is presented. It proposes a generic control architecture, which is applicable to piloted, semiautomatic, and fully automated flight, consisting of an aircraft-level controller (high-level controller) and a control allocation scheme. The aircraft-level controller consists of a main inner loop classical nonlinear dynamic inversion controller and an outer loop proportional–integral linear controller. The inner loop nonlinear dynamic inversion controller is a velocity controller that cancels the nonlinear bare airframe dynamics, while the outer loop proportional–integral linear controller is an attitude and navigation position controller. Together, they are used for hover/low-speed control and forward flight. The control allocation scheme uses a novel architecture, which transfers the nonlinearity in the vectored thrust effector model formulation to the computation of the actuator limits by converting the effector model from polar to rectangular form, thus allowing the use of classical control allocation linear optimisation technique. The linear optimisation technique is an active set linear quadratic programming constrained optimisation algorithm with a weighted least squares formulation. The control allocation allocates the overall control demand (virtual controls) to individual redundant effectors while performing control error minimisation, control channel prioritisation and control effort minimisation. Simulation results show the transition from hover to cruise, climb and descent, and coordinated turn clearly demonstrate that the controller can handle actuator saturation (position or rate).

![](images/ee09450b7192d3d8cdcbac463042b1c21700c7cea7cb0dad033fb3e0f15c709b.jpg)

Citation: Enenakpogbe, E.; Whidborne, J.F.; Lu, L. Full Envelope Control of Over-Actuated Fixed-Wing Vectored Thrust eVTOL. Aerospace 2024, 11, 979. https://doi.org/ 10.3390/aerospace11120979

Academic Editor: Shu-Guang Zhang

Received: 26 October 2024 Revised: 17 November 2024 Accepted: 21 November 2024 Published: 27 November 2024

![](images/0ee6ae0377e6c7a0fb192104293eacc2caea30f63ba90ec9bde33fc3562becc3.jpg)

Copyright: © 2024 by the authors. Licensee MDPI, Basel, Switzerland. This article is an open access article distributed under the terms and conditions of the Creative Commons Attribution (CC BY) license (https:// creativecommons.org/licenses/by/ 4.0/).

Keywords: NDI; transition; over-actuated; virtual controls; easy-to-fly; effector model; control allocation; active set; weighted least squares; optimal; high-level controller; unified control framework; real-time; online; constrained optimisation; linear quadratic programming

## 1. Introduction

This paper is the full paper version of an extended abstract conference paper [1] and includes more model details, details of the technique applied, more analysis and simulation results. The concept of Urban Air Mobility (UAM) has been proposed for addressing a need for new urban transport solutions to overcome the problems of congestion and pollution in the ever-growing major urban conurbations [2]. Fixed-wing aircraft technologies, despite being mature and reliable, have many constraints, both infrastructural and operational, which limit their applicability for UAM. Piloted all-electric vertical takeoff and landing (eVTOL) vehicles known as “air taxis” provide a promising option [3]. Furthermore, electric propulsion is lighter, quieter, and potentially has net zero emissions, with potentially lower operating costs, and they have higher bandwidth actuators than fuelled machines, thus allowing for novel configurations.

However, eVTOL aircraft operation requires nonlinear or scheduled control since they operate over a wide range of operating envelopes (hover, forward flight, forward transition from hover/low speed to cruise/forward flight, reverse transition from cruise to hover transition, post-stall operation, cruise, climb, descent, coordinated turn, etc.), requiring them to operate in several operating modes with a need to switch between flight controllers. Nonlinear control schemes, which can handle nonlinearities, offer potential solutions in providing a unified-control approach valid in all flight modes, without the need to perform predefined-gain scheduling [4,5].

A pre-requisite for the eVTOL air taxi business model is to have an aircraft that is relatively “easy-to-fly”, which requires minimal pilot training compared to commercial and military aircraft. This requires eVTOLs to provide a high degree of automation to reduce the pilot workload and cost of pilot training. A trend in eVTOL development is towards finding a unified-control approach valid for all flight modes without the need to switch between flight controllers or to perform predefined-gain scheduling in order to address the challenge of a need to mitigate control complexity and available computing resources [6,7]. This requires eVTOLs to have fly-by-wire (FBW) augmented flight control systems (also known as indirect flight control systems), which satisfies the primary flight objectives (stability, envelope protection, fault tolerance, robustness to both external and internal disturbances, etc.) and the secondary objectives (flight performance, efficiency, drag reduction, flying and handling qualities, etc.).

In order to ensure the required levels of reliability, safety, and actuator/effector fault tolerance, eVTOLs are usually overactuated, i.e. equipped with more effectors than the degrees of freedom they control, thus introducing redundancy in case of effector failure [8–11]. This over-actuation creates the challenge of allocating the overall control demand to individual redundant effectors known as Control Allocation (CA). CA provides an optimized allocation of control effectors for the various degrees of freedom under fault-free operation and optimisation of actuator utilisation (including secondary objectives like drag, wing load reduction, etc.) with respect to varying flight phases, and it provides scope for reconfiguration in the event of a fault in the control effector.

Vectored Thrust eVTOLs, one category of eVTOLs according to the Vertical Flight Society [12,13], are supported in vertical takeoffs and landing flights by the horizontally mounted propulsion systems, which direct thrust downwards to generate lift. Horizontal flight is supported primarily by the lift generated from the wings, with some or all of the propulsion systems rotated to the required position to provide horizontal thrust for forward speed. Vectored thrust eVTOLs can be further classified into two subcategories: tiltwing and tiltrotor.

An example of a tiltwing eVTOL is the Airbus A<sup>3</sup> Vahana [14] while that of a tiltrotor eVTOL is the Lilium Jet [15]. Tilt wings are a challenging configuration for control, particularly in the transition phase between hover and cruise, where the aerodynamics may be in a post-stall condition. One of the control challenges of vectored thrust tiltrotor eVTOLs is the interaction and coupling between the aeromechanical and dynamic forces and moments, which potentially deplete the natural aerodynamic damping or stiffness, tending to drive an aircraft unstable [16].

Classical nonlinear dynamic inversion (NDI) is a well-established nonlinear control approach, which has been widely applied in various forms [17–29]. Although NDI can deal with the nonlinearities in the motion of an aircraft, one drawback is its poor performance in the presence of external disturbances and model uncertainties because it relies on an accurate model of the controlled system. Hence, NDI has been combined with robust controllers to deal with these disturbances and uncertainties [30–33]. Another drawback of classical NDI is that it is applicable to systems with minimum phase characteristics because inversion of an unstable transition zeros (non-minimum phase NMP system) will result in instability [34].

There are several CA approaches and techniques used for the motion control of overactuated mechanical systems [35–38]. CA approaches for linear effector models include unconstrained linear CA approaches (e.g. generalised inverses or pseudoinverses, Moore– Penrose pseudo-inverse, and singular value decomposition-based generalised inverses)

and constrained linear CA approaches (redistributed pseudo-inverse, cascaded generalised inverse, daisy chaining, direct CA, simplex numerical linear programming, active set numerical quadratic programming (QP) method, interior point numerical quadratic programming method, iterative fixed-point numeric quadratic programming method, multi-parametric quadratic programming, binary search trees, lattice representations, decomposition approach for primary and secondary problems, an optimisation-based control allocation method integrated with a parameter estimation scheme, fault-tolerant optimisation-based control allocation, a dynamic control allocation approach, model pre dictive control optimisation-based control approach, etc.).

CA approaches for nonlinear effector models include nonlinear programming for control allocation, sequential QP, mixed-integer linear programming (LP), Lyapunov-design based nonlinear optimisation control allocation method, direct nonlinear allocation, etc. Optimal solutions of LP are found at vertices of the feasible set, thereby favouring the use of a smaller number of effectors, while methods based on a quadratic cost function and ∞-norm tend to use all effectors but to a smaller degree [39].

Active set methods [35,40–43] for QP are iterative methods whereby, at each iteration, they improve their guess of the optimal active set. The interior point method [40,42,43] for QP, on the other hand, replaces the inequality constraints with a barrier function that prevents the solution from going into the infeasible region.

One particularly challenging eVTOL configuration is the Lilium Jet [15], an early version of which is the 7-seater canard light sports aircraft with 36 tiltable all-electric engines (six on each of the two front canards and twelve on each rear main wing). The electric ducted fans (EDFs) are located on top of the control surfaces, which provides vectored thrust simultaneously with aerodynamic control. However, its canard configuration can destabilise the short-period mode [44,45], thus requiring FBW stability augmented flight control systems. The configuration is also challenging because the coupling between the aerodynamic surface control and the vectored thrust makes control allocation during transition very difficult. Furthermore, the electric engine arrangement of the Lilium-style multi-rotor vectored thrust tiltrotor aircraft results in eight control effectors to control six (roll, pitch, yaw, surge, sway, and heave) degrees of freedom, which results in a high degree of over-actuation consequently presenting a challenging CA.

The 7-seater Lilium Jet eVTOL configuration is also a challenging eVTOL because the vectored thrust results in nonlinear effector mapping (from the overall control demand to the individual effector demand), preventing the direct use of classical linear CA approaches.

This paper proposes a real-time linear analytical control allocation, which considers both actuator saturation and rate limits combined with classical NDI for full envelope control of a novel Lilium-style over-actuated vectored thrust eVTOL configuration. A QP error minimisation linear CA approach incorporating both actuator saturation and rate constraints has been carefully chosen.

In the research literature, a combination of an NDI-based high-level motion controller combined with a CA scheme under a unified framework and control architecture consisting of several flight control modes (requiring switching between them) has been applied for the full envelope control of several limited special class of eVTOL tiltrotor configurations [46–48].

The main contributions of this paper are summarized as follows:

It applies an easy-to-fly, simple and unified control framework with a functional architecture applicable for piloted, semi-automatic, and automated flight to a Liliumstyle multi-rotor vectored thrust tiltrotor eVTOL [15] configuration.

It proposes the application of NDI [49] as the high-level controller (aircraft-level controller) for the Lilium-style multi-rotor vectored thrust tiltrotor eVTOL [15] con figuration rather than the incremental NDI used in [48] because it is well established and has a proven stability analysis despite some drawbacks (poor external disturbances, rejection, and robustness to model uncertainties). The paper extends the results of [49] by including lateral-directional control channels and an effector model of an over-actuated vectored thrust Tilt rotor aircraft [15], as shown in Figure 1.

It extends the results of [50] from 3DoF to 6DoF and verifies a real-time analytical optimal CA approach, proposed as CA approach 1 (CA1) in [50], which considers both actuator saturation and rate limits, using an active set linear QP WLS formulation.

The rest of this paper is structured as follows: the full 6DoF equations of motion, vehicle dynamic model, aerodynamic and thrust models of a Lilium-style eVTOL canard planform shown in Figure 1 are outlined in Section 2. In Section 3, the control architecture and high-level controller are presented. CA is presented in Section 4. The nonlinear control simulation results of some selected manoeuvres are presented and analysed in Section 5. In the final section, Section 6, the proposed scheme and summary of major findings are discussed, as well as their limitations and future work.

## 2. Vehicle Dynamic Modelling

## 2.1. Overview of Vehicle

The vehicle model used for this work is based on the Lilium Jet [15], a 7-seater Lilium Jet, a canard light sports aircraft with 36 all-electric engines (six on each of the two front canards and twelve on each rear main wings). This results in eight control effectors to control six (roll, pitch, yaw, surge, sway, and heave) degrees of freedom, resulting in an over-actuated system and consequently results in a CA problem. The plan view of the vehicle is shown below in Figure 1.

![](images/3944db255c4be31cbd607c882e9a6345f18753728f2f18d36b0db80a4c8fdd3c.jpg)
Figure 1. Aircraft planform showing EDF set distribution.

The aircraft has a fixed-wing aerodynamic surface and twelve tilting EDFs distributed through the front canard and rear main wing sections of the aircraft (see Figure 1). The challenging part is estimating the aerodynamic and propulsion forces and moments acting on the aircraft, especially during the transition region. High-fidelity modelling of the transition dynamics is quite complex since tilting EDFs also generates aerodynamic forces/moments, and flow separation over the EDF surfaces might occur during the transition. Since the main focus and motivation of this work are the full envelope flight control of the unique eVTOL aircraft, the flight dynamics model is constructed considering the main effects at hover and high-speed forward flight. During the transition, the dynamics are obtained by blending the hover and forward flight aerodynamic models based on the airspeed, though in practice, this introduces significant nonlinearity in its aerodynamic forces and moments due to the complex interaction between the incoming freestream and the propeller slipstream. However, the complex interaction between the incoming freestream and the propeller slipstream is hard to accurately model, resulting in some unmodelled dynamics and a low-fidelity aerodynamic model. The eVTOL aircraft is a minimum phase system be cause of its canard configuration; its wings have no anhedral; it is an over-actuated system with vectored thrust and is a simple model with decoupled aerodynamic and propulsive forces; and moment has been assumed.

The aircraft parameters are as follows: the total aircraft mass, $m = 3 . 1 7 5 \times 1 0 ^ { 3 }$ kg, the wing surface area, $S = 8 . 4 6 4 \mathrm { m } ^ { 2 }$ , aerodynamic mean chord, $\bar { c } = 1 . 1$ m, span, $b = 1 3 . 9$ m, and moment of inertia

$$
\mathbf {I} = \left[ \begin{array}{c c c} I _ {x x} & - I _ {x y} & - I _ {x z} \\ - I _ {x y} & I _ {y y} & - I _ {y z} \\ - I _ {x z} & - I _ {y z} & I _ {z z} \end{array} \right] = \left[ \begin{array}{c c c} 7 4 8 9 & 0 & - 4 8 4 \\ 0 & 8 2 6 2 5 & 0 \\ - 4 8 4 & 0 & 8 8 8 4 3 \end{array} \right]\tag{1}
$$

## 2.2. Flight Mechanics and Equation of Motion

Consider an air vehicle with 6 degrees of freedom, pitch, roll, yaw, heave, surge and sway. Assuming a rigid body in still air, the dynamic equations for linear and angular accelerations (standard Newton-Euler equations) are as follows:

$$
\left[ \begin{array}{c} \dot {\mathbf {F}} \\ \dot {\mathbf {M}} \end{array} \right] = \left[ \begin{array}{c c} m \mathbf {I} _ {3} & 0 \\ 0 & \mathbf {I} _ {b} \end{array} \right] \left[ \begin{array}{c} \dot {\mathbf {V}} _ {b} \\ \dot {\boldsymbol {\omega}} \end{array} \right] + \left[ \begin{array}{c} m \boldsymbol {\omega} \times \mathbf {V} _ {b} \\ \boldsymbol {\omega} \times \mathbf {I} _ {b} \boldsymbol {\omega} \end{array} \right]\tag{2}
$$

where F is the total force vector acting on the centre of mass in the body axes and M is the total moment vector acting about the centre of mass given by the following:

$$
\mathbf {F} = \mathbf {F} _ {a e r o} + \mathbf {F} _ {p r o p} + \mathbf {F} _ {g r a v}\tag{3}
$$

$$
\mathbf {M} = \mathbf {M} _ {a e r o} + \mathbf {M} _ {p r o p}\tag{4}
$$

The aerodynamic contributions defined by means of their dimensionless coefficients are as follows:

$$
\mathbf {F} _ {a e r o} = \left[ \begin{array}{c} X \\ Y \\ Z \end{array} \right] = \left[ \begin{array}{c} C _ {X} \\ C _ {Y} \\ C _ {Z} \end{array} \right] \bar {q} S\tag{5}
$$

$$
\mathbf {M} _ {a e r o} = \left[ \begin{array}{c} L _ {a} \\ M _ {a} \\ N _ {a} \end{array} \right] = \left[ \begin{array}{c} b C _ {l} \\ \bar {c} C _ {m} \\ b C _ {n} \end{array} \right] \bar {q} S\tag{6}
$$

Details of $\mathbf { F } _ { p r o p } , \mathbf { F } _ { g r a v } , \mathbf { M } _ { p r o p }$ and other aerodynamic terms are contained in [34,51] and in the supplementary file.

## 2.3. Vehicle Dynamic Model

The cruise aerodynamic model is partially derived from [52] and based on [48,53], while the hover aerodynamic model is from [48,53]. The geometric information, moment of inertia, mass and effector distribution as indicated in Figure 1 are based on [52]. Full details of the dynamic model are in [51] and in the supplementary file.

## 2.4. Aerodynamic Model Blending

At hover $( u < 1 0 \mathrm { m } / \mathrm { s } ( u _ { h o v } ) )$ , the propulsive forces are dominant (aerodynamic control surfaces are ineffective with low control authority) while at high forward cruise speeds $( u > 2 0 \mathrm { m } / \mathrm { s } ( u _ { c r u } ) )$ ) the aerodynamic forces are dominant, separate aerodynamic models have been formulated for both flight regimes. In order to simulate the nonlinearities due to changes in aerodynamic forces and moment during transition between hover and cruise/forward flight a blending coefficient is used to blend the aerodynamics forces produced by both separate hover and cruise models using body x-axis velocity u as the scheduling parameter.

The blending coefficient for performing transition between hover and forward flight aerodynamic models is a continuous differentiable hyperbolic tangent function $K _ { b }$ of $u ,$ $u _ { h o v } ,$ , and $u _ { c r u }$ is given by:

$$
K _ {b} = \frac {1}{2} \left[ 1 + \tanh \left(\frac {4 u - 2 (u _ {c r u t h} + u _ {h o v t h})}{u _ {c r u t h} - u _ {h o v t h}}\right) \right]\tag{7}
$$

The blending coefficient $K _ { b }$ is set to 0 for values of $u \leq u _ { h o v } ,$ to 1 for $u \geq u _ { c r u } ,$ , to a value between 0 and 1 for $u _ { h o v } < u < u _ { c r u }$ is illustrated in Figure 2. $u _ { h o v t h } = 1 2 \mathrm { ( m / s ) }$ and $u _ { c r u t h } =$ 18 $\left( \mathbf { m } / \mathbf { s } \right)$

![](images/350f2dbaa6ae311009fa974632066c8730f983c06e494ca8af14a12193ed4838.jpg)
Figure 2. Aerodynamic blending coefficient.

The resultant aerodynamic forces and moments are given by the following:

$$
\mathbf {F} _ {a e r o} = K _ {b} \mathbf {F} _ {c r u} + (1 - K _ {b}) \mathbf {F} _ {h o v}\tag{8}
$$

$$
\mathbf {M} _ {a e r o} = K _ {b} \mathbf {M} _ {c r u} + (1 - K _ {b}) \mathbf {M} _ {h o v}\tag{9}
$$

where $\mathbf { F } _ { h o v }$ is the aerodynamic force vector $[ X _ { h o v } Y _ { h o v } Z _ { h o v } ] ^ { T }$ for the hover model, $\mathbf { F } _ { c r u }$ is the aerodynamic force vector $[ X _ { c r u } Y _ { c r u } Z _ { c r u } ] ^ { T }$ for the cruise model, ${ \bf M } _ { h o v }$ is the aerodynamic moment vector $[ L _ { A _ { h o v } } M _ { A _ { h o v } } N _ { A _ { h o v } } ] ^ { T }$ for the hover model, $\mathbf { M } _ { c r u }$ is the aerodynamic moment vector $[ L _ { A _ { c r u } } M _ { A _ { C R U } } N _ { A _ { c r u } } ] ^ { T }$ for the hover model, ${ \bf F } _ { a e r o }$ is the resultant blended aerodynamic force vector, ${ \bf M } _ { a e r o }$ is the resultant blended aerodynamic moment vector.

## 2.5. Hover Aerodynamic Model

The hover aerodynamic model is a resistance drag force model from [48,53] because, at hover and low-speed flight, it is reasonably assumed that the dominant aerodynamic effect is the resistance drag force in translational axes (horizontal, vertical, and sideward motion). The lift force and aerodynamic moments are considered negligible $( { \bf M } _ { h o v } = 0 )$ and not modelled at low-speed flight since the maximum speed is restricted to 10m/s for horizontal motion and $5 \mathrm { m } / \mathrm { s }$ for vertical motion. Although the ground effect is also relevant for hover flights, it is not modelled for the preliminary modelling since it requires more detailed and complex aerodynamic modelling. Full details of the hover dynamic model are in [51] and in the supplementary file.

## 2.6. Cruise Aerodynamic Model

The cruise model longitudinal static aerodynamic derivatives are partially obtained from [52,54] using an appropriate approach for this type of VTOL configuration, a computational fluid dynamics-based aerodynamic modelling method known as boundary layer ingestion. The cruise model longitudinal dynamic derivatives, lateral-directional static derivates and lateral-directional dynamic aerodynamic derivates are based on [48,53]. Refer to [51] for details of the Cruise Aerodynamic Model and in the supplementary file.

## 3. Control Design

This section presents the control architecture which describes the control framework, the command generator, and the main overall aircraft-level controller consisting of an outer loop PID-based linear controller and an inner loop NDI-based controller.

## 3.1. Control Architecture

As mentioned in Section 1, one of the main contributions of this work is to apply the easy-to-fly, simple and unified control Framework proposed in [51] (which is 6DoF version of that proposed in [49,50]) and shown in Figure 3 to the over-actuated Fixed Wing Vectored thrust eVTOL vehicle.

![](images/b1eab4747756c5aad6eff7723cb359f2f87448fbfdb4ab31761c97db2e1123e0.jpg)
Figure 3. Generic flight controller architecture for piloted, semi-automatic, and automated flight [49].

The proposed control architecture can be applied to piloted, semi-automatic and automated flight, and consist of the Command Generator, the Controller and the Physical Vehicle (represented by a simulation vehicle model) it controls. The control allocation scheme $\mathbf { K } _ { \mathrm { C A } }$ ensures that the forces and moments vector, τ, that characterizes the physical model given by Equation (17) is equal to the virtual control demands output, $\boldsymbol { \tau } _ { c } ,$ produced by the high-level control law (outer linear controller and inner loop LDI controller) ${ \bf K } _ { \mathrm { C T L } }$ The physical vehicle model consists of an effector/actuator model $\mathbf { B } _ { \mathrm { e } } ;$ thus, it would normally include a low-level actuator controller (e.g. servo controllers) and the virtual controls (generalised forces and moments) G. The inverse effector model $\mathbf { K } _ { \mathrm { C A } }$ (i.e. control allocation) using the effector/actuator model $\mathbf { B } _ { \mathrm { e } }$ will be covered in Section 4.

The physical vehicle model consists of an effector/actuator model, thus normally would include low-level actuator controller (e.g. servo controllers), and the physical model with virtual controls. For the remainder of the work, it is assumed that a perfect control allocation scheme is in operation so that the virtual control demands output, $\tau _ { c } ,$ , which is produced by the inner loop controller, $\mathbf { K } _ { \mathrm { i n n } } ,$ , is equal to the forces and moments vector, τ, that characterizes the physical model given by Equation (17).

Based on NASA Pilot-Automation-Interaction (PAI) framework [55,56] which characterises an aircraft real-time functional state automation dimensions, the control architecture in this work is PAI-3 (Maneouvre Command)—Autopilot with specified linkable maneouvres.

## Command Generator

The Command Generator and outer loop controller are shown in Figure 4. The Command Generator consists of the following:

• Manual and auto-pilot commands;

• Command limits and filtering;

• Command Computation;

<sub>•</sub> <sub>Manual–auto</sub> <sub>mode</sub> <sub>switching;</sub>ith virtual controls. For the remain

• Position control switching;<sup>ocation</sup> <sup>scheme</sup> <sup>is</sup> <sup>in</sup> <sup>operatio</sup>

• Outer loop linear controller;<sup>oduced</sup> <sup>by</sup> <sup>the</sup> <sup>inner-loop</sup> <sup>contr</sup>

<sub>•</sub> <sub>Control</sub> <sub>mode</sub> <sub>switching;</sub>at characterizes the physical

Command blending between hover and forward flight using Equation (7) in Section 2.4<sup>ased</sup> <sup>on</sup> <sup>NASA</sup> <sup>Pilot-Automation-Interaction</sup> <sup>(PAI)</sup> <sup>framework</sup> <sup>[55,56]</sup> <sup>which</sup> <sup>charac</sup> <sub>and</sub> <sub>a</sub> <sub>blending</sub> <sub>equation</sub> <sub>(same</sub> <sub>structure</sub> <sub>as</sub> <sub>Equation</sub> <sub>(8)</sub> <sub>in</sub> <sub>Section 2.4)</sub> <sub>given</sub> <sub>by</sub> <sub>the</sub> an aircraft real-time functional state automation dimensions, the control architectur following: <sup>work</sup> <sup>is</sup> <sup>P</sup>

$$
\mathbf {x} _ {c m d} = K _ {b} \mathbf {x} _ {c m d, c r u} + (1 - K _ {b}) \mathbf {x} _ {c m d, h o v}\tag{10}
$$

The outer loop is airframe independent since it does not consider the airframe aerody-<sub>Command</sub> <sub>Generator</sub> namics.

![](images/acb530b82a0fc56f0d1d87a72e23b11596693521d321f40c6c958787f38ad234.jpg)
gure 4. Command Generator and Outer loop ControlleFigure 4. Command generator and outer loop controller.

## The Command Generator and Ou3.2. Control Limitation and Flight Modes

<sup>Generator</sup> <sup>consists</sup> <sup>of:</sup> Control limitations and filtering applied on the commands (manual or auto mode) are <sub>manual</sub> <sub>and</sub> <sub>auto-pilot</sub> <sub>commands</sub>listed in Section 5. The various flight regime modes are as follows:

ommand limits & filtering Vertical TakeOff and Landing $( \mathrm { V T O L } ) \colon h _ { c o n t r o l }$ mode, h control, $V = 0 , \theta = 0 , \phi = 0 .$ $\psi = 0 ;$

anual-auto mode switchingClimb, Descend, Approach and Flare (CDAF): V control, λ control, θ control, $\phi = 0$ $\psi = 0 ;$

Outerloop Linear controllerCruise (CRU): V control, λ control, θ control, $\phi = 0 , \psi = 0 ;$

ontrol mode switchingHover (HOV) and Position control: $x _ { c o n t r o l }$ mode, $y _ { c o n t r o l }$ mode, $x _ { c o n t r o l }$ mode, x control, ommand blendingy control, z control, $\theta = 0 , \psi = 0 ;$

nd a blending equation (same struc<sup>Station</sup> <sup>Keeping</sup> <sup>(SK): x mode,</sup> $y _ { c o n t r o l }$ equati<sup>mode,</sup> $x _ { c o n t r o l }$ in section 2.4) given by:<sup>mode, x hold, y hold</sup> <sup>and z</sup> hold, $\theta = 0 , \psi = 0 ;$

• Forward Transition (FT): move from Hover (HOV) and position control to Cruise (CRU);

x = K x + (1 K ) x (10Backward Transition (BT): move from Cruise (CRU) to Hover (HOV) and position control;

Banking (ROL): V control, λ control, ϕ control, θ control, ψ control, $h _ { c o n t r o l }$ mode;

• Coordinated Turn (CTURN): V control, control, control, control, control.

uterloop is airframe independent since it doesn’t consider the airframe aerodynaRefer to [51] and the supplementary file for details of the command generator and outer loop controller.

As discussed in Section 1, two main challenges of canard-plan form fixed-wing VTOL aircraft are as follows:

• Severe nonlinearities in the flight dynamics due to the wide operating envelope;

• Unstable open loop operation due to canard configuration.

## 3.3. NDI Controller Law

Consider a plant model with general state equations given by the following:

$$
\dot {\mathbf {x}} = \mathbf {f} (\mathbf {x}) + \mathbf {g} (\mathbf {x}) \boldsymbol {\tau} _ {c}\tag{11}
$$

$$
\mathbf {y} = \mathbf {h} (\mathbf {x})\tag{12}
$$

For this paper, the selected CV, $\mathbf { y } = \mathbf { h } ( \mathbf { x } )$ , for the first five (surge, heave, roll, pitch, and yaw) degrees of freedom are given by the following:

$$
\mathbf {y} = (u, w, p, q, r) ^ {T}\tag{13}
$$

where $\mathbf { x } ( t ) \in \mathbb { R } ^ { n }$ is the vector of state variables, $\pmb { \tau _ { c } } \in \mathbb { R } ^ { m }$ is the (virtual) control, $\mathbf { y } ( t ) \in \mathbb { R } ^ { m }$ is the vector of outputs to be controlled and the error is given by the following:

$$
\mathbf {e} (t) = \mathbf {r} (t) - \mathbf {y} (t)\tag{14}
$$

where $\mathbf { r } ( t ) \in \mathbb { R } ^ { m }$

Based on the exposition in $[ 3 4 , 4 9 ]$ , the NDI controller is

$$
\mathbf {v} = \mathbf {K e}\tag{15}
$$

$$
\pmb {\tau} _ {c} = G ^ {- 1} (\mathbf {x}) [ - \mathbf {F} (\mathbf {x}) + \dot {\mathbf {r}} + \mathbf {v} ]\tag{16}
$$

where

$$
\mathbf {F} (\mathbf {x}) := \frac {\partial \mathbf {h}}{\partial \mathbf {x}} \mathbf {f} (\mathbf {x}), G (\mathbf {x}) := \frac {\partial \mathbf {h}}{\partial \mathbf {x}} \mathbf {g} (\mathbf {x})\tag{17}
$$

and v ensures that $\mathbf { v } = \mathbf { K } \mathbf { e }$

The controller structure is shown in Figure 5 and consists of the linear output feedback loop Equation (15) and the state feedback linearisation loop (with linear controller K in Equation (16)) which assumes all the states are observable. The feedback linearisation loop ensures that the system from $\mathbf { v } ( t )$ $\mathbf { y } ( t )$ is like a linear system with poles at the origin, thus simplifying the outer tracking loop design. The velocity feedforward term r˙ improves the tracking accuracy of the closed-loop system. This signal is generated either by the pilot inceptor for manned operation or the guidance system for automated flight.

![](images/26c9e2a543a69d6643f6870085d95267a5311fb8663f9cff0cf7f3b2231cfc99.jpg)
Proportional Feedback Loop
Figure 5. NDI problem formulation.

Since the controller in Figure 5 contains a model of the aircraft dynamics (Equation (17)), the nonlinear functions of the aircraft must be known in order to implement it.

The classical NDI inner loop is airframe-dependent, and it uses an onboard model (OBM) of aircraft to invert and cancel the airframe dynamics so that the system appears as an integrator. The outer NDI loop is airframe-independent and can also be used for flying and handling quality control design. Refer to [49] and the supplementary file for classical NDI controller details applied in this work.

## 4. Control Allocation

Consider a plant model with general state equations given by the following:

$$
\dot {\mathbf {x}} = f (\mathbf {x}) + g (\mathbf {x}) \boldsymbol {\tau} _ {c}\tag{18}
$$

$$
\mathbf {y} = l (\mathbf {x})\tag{19}
$$

$$
\pmb {\tau} = h (\mathbf {u}, \mathbf {x})\tag{20}
$$

where $\mathbf { x } ( t ) \in \mathbb { R } ^ { n }$ is the vector of state variables, $\pmb { \tau _ { c } } \in \mathbf { A } \in \mathbb { R } ^ { m }$ is the commanded virtual control (control effect and generalised forces), $\pmb { \tau } \in \mathbf { A } \in \mathbb { R } ^ { m }$ is the actual virtual control (control effect and generalised forces), $ { \mathbf { u } } \in  { \mathbf { U } } \in \mathbb { R } ^ { p }$ is the real controls, $\mathbf { y } ( t ) \in \mathbb { R } ^ { m }$ is the vector of outputs to be controlled, A is the set of attainable moments and U is the set of admissible controls.

$h ( \mathbf { u } , \mathbf { x } )$ is the effector model, which is dependent on the vehicle effector’s physical and control configuration. Effector models can be formulated and parameterized in many ways, but they must represent the actual physical forces and moments. Usually, one seeks effector models that are linear with respect to their inputs, which leaves nonlinearities to be compensated for by nonlinear mappings, or in the low-level single effector / actuator control through inversion of monotone characteristics or linearising feedback loops. A highlevel motion controller $( \mathrm { e . g . }$ . classical NDI) computes the commanded virtual controls $\tau _ { c } .$ The virtual control inputs are usually chosen as a number of forces and moments that are equal to the degrees of freedom that the high-level motion controller wants to control, $p$ such that the basic requirement of controllability is met.

If the virtual controls cannot be jointly achieved by the individual effectors due to one or more effectors reaching their physical limits (e.g. saturation or rate limits) or a failure, the CA degrades the control by prioritising control (i.e. error minimisation) in the various control channels and degrees of freedom.

Given $\mathbf { u } \in \mathbb { R } ^ { \ell } , \tau \in \mathbb { R } ^ { p } , \tau _ { c } \in \mathbb { R } ^ { p }$ then if the number of degrees of freedom that the highlevel motion controller wants to control is greater than the number of real controls (effectors) or decision variables $p \geq m$ , then the system is over-actuated and over-determined such that the inverse problem of computing $\mathbf { u } \in \mathbf { U }$ given $\tau _ { c } = \tau$ using Equation (20) is ill-posed since the solution is generally not unique.

## 4.1. Control Allocation Theory and Formulation

If $\ell = p$ then the solution is trivially given by $\mathbf { u } = \mathbf { B } _ { e } ^ { - 1 } \boldsymbol { \tau } _ { c }$ (for example, the quadrotor control problem) under the assumption that $\mathbf { B } _ { e }$ is full rank and that no effector constraints are breached. If $\ell > p$ then multiple solutions exist, for example $\mathbf { u } = \mathbf { B _ { i n v } } = \mathbf { B } _ { e } ^ { \dagger } \boldsymbol { \tau _ { c } }$ where $[ \cdot ] ^ { \dagger }$ represents the Moore–Penrose inverse of the matrix. Hence the problem is often extended to account for some other requirements (e.g. minimum power) as well as to take account of various constraints. The constraints are, for example, saturation constraints on u and slew rate constraints on u˙ .

As mentioned in the previous section, minimising the control error $\mathbf { ( s } = \pmb { \tau } _ { c } - \mathbf { B } _ { e } )$ which is known as a primary objective or equality constraint, imposing additional physical constraints in the form of affector/actuator saturation and rate limits, and introducing secondary objective(s) to the control allocation algorithm ensures that a unique solution results in the following control allocation formulation:

$$
\begin{array}{l l} \min _ {\mathbf {u} \in \mathbb {R} ^ {p}, \mathbf {s} \in \mathbb {R} ^ {m}} & (\| \mathbf {W} _ {v} \mathbf {s} \| + J (\mathbf {x}, \mathbf {u}, t)) \text {s.t.} \\ & \boldsymbol {\tau} - \mathbf {B} _ {e} = \mathbf {s}, \mathbf {u} \in \mathbf {U}, \mathbf {u} = \mathbf {u _ {0}} + \Delta \mathbf {u}, \Delta \mathbf {u} \in \mathbf {C} \end{array}\tag{21}
$$

$$
J (\mathbf {x}, \mathbf {u}, t) = \frac {1}{2} (\mathbf {u} - \mathbf {u} _ {p}) ^ {T} \mathbf {W} _ {u} (\mathbf {u} - \mathbf {u} _ {p})\tag{22}
$$

$$
J (\mathbf {x}, \mathbf {u}, t) = \| \mathbf {W} _ {u} \mathbf {u} \|\tag{23}
$$

$\mathbf { W } _ { u }$ is intentionally chosen small compared to $\mathbf { W } _ { v } \ ( \mathrm { i . e . } \ W _ { u } \ll \mathbf { W } _ { v } )$ to reflect the fact that J represents an objective that is secondary to the primary objective of minimising the slack variables weighted by $\mathbf { W } _ { \tau }$ .

## 4.2. Control Allocation Functional Architecture

As discussed in Section 1, one of the main contributions of this work is a verification of a 6DoF real-time analytical optimal control allocation, which considers both actuator saturation and rate limits using active set linear QP Weighted Least squares (WLS) formulation proposed as CA Approach 1 (CA1) in [50,51].

It retains the overall control demands (virtual control) in Cartesian form but formulates the individual effector demand (real controls) in Cartesian form (decomposing its thrust components on the body axes), thus resulting in a Linear Effector mapping, which allows the use of classical linear control allocation techniques with prioritisation. The required computation of the actuator limits, which converts the physical actuator limits from polar to Cartesian form, is, however, nonlinear and complex. It offers a real-time analytical optimal control allocation which considers both actuator saturation and rate limits using active set Priorotised linear QP Weighted Least squares (WLS) formulation as the linear control allocation algorithm. The computation of the actuator limits uses the physical (polar) actuator limits, the previous cycle value of the real controls in Cartesian form and the previous cycle value of the real controls in polar form.

The functional architecture of the control allocation architecture is shown in Figure 6 and consists of four stages:

Step 1 Formulation of real controls in Cartesian form, resulting in a linear effector mapping; Step 2 Transformation of previous cycle real controls from Cartesian to polar form;

Step 3 Conversion of the actuator constraints from polar to Cartesian;

Step 4 Application of prioritised linear control allocation technique.

![](images/9f48c9bfc0142319c8b648214cf0fe67a19e0304d41425a71a520a9a6d2746a2.jpg)
Figure 6. The 6DoF CA functional architecture.

Active set linear control allocation methods for quadratic programming (QP) address control channel prioritisation using the value of $\mathbf { W } _ { v }$ in Equation (40) to overcome a major challenge and crucial constraint of thrust control saturation due to actuator saturation for the vectored thrust eVTOL configuration which has a potential to lead to undesired behaviour, deterioration of the controller performance and even loss of control in specific flight conditions due to the limited control authority during normal and degraded operation (e.g. actuator limitation or failures). Other parameters for the prioritised constrained linear optimisation Active Set algorithm are contained in Section 5.

## 4.3. Formulation of Real Controls Resulting in a Linear Effector Mapping

A plan view of lilium-style fixed-wing over-actuated vectored thrust eVTOL aircraft configuration showing EDF sets distribution, control groupings and real controls is shown in Figure 7.

![](images/44bb4521a71ce52c65a74143849d7b92ad6db7c1e73a8806e0f6dba8492c65f1.jpg)
Figure 7. Plan view of Lilium-style fixed-wing over-actuated vectored thrust eVTOL aircraft configu ration showing EDF set distribution, control groupings, and real controls.

The vectored thrust actuator (thrust T and tilt angle Γ) limits are illustrated in Figure 8 which only shows the actuator position limits for simplicity. The unshaded regions are the set of admission controls U constrained by the effector limits.

![](images/64bfac3981331a38e55bbf3b19b0a1e92c7da81b64bea84080ba6f767191e206.jpg)
Figure 8. Vectored thrust actuator limit illustration.

Table 1 shows the turn directions of the various EDF sets in Figure 7.
Table 1. Effector model moment arm turn directions, from CG, assuming NED.

<table><tr><td rowspan="2">A/C loc</td><td rowspan="2">EDF Set</td><td rowspan="2">Turn Dir (td)</td><td colspan="3">Effector Moment Arms from CG (NED)</td></tr><tr><td>Δx (m)</td><td>Δy (m)</td><td>Δz (m)</td></tr><tr><td>canard left engine tip</td><td>Set 1</td><td>CCW(-1)</td><td>3.8334</td><td>-2.3660</td><td>0.0716</td></tr><tr><td>canard left engine root</td><td>Set 2</td><td>CCW(-1)</td><td>3.8564</td><td>-1.3680</td><td>0.0716</td></tr><tr><td>canard right engine root</td><td>Set 3</td><td>CW(+1)</td><td>3.8564</td><td>1.3680</td><td>0.0716</td></tr><tr><td>canard right engine tip</td><td>Set 4</td><td>CW(+1)</td><td>3.8334</td><td>2.3660</td><td>0.0716</td></tr><tr><td>wing left engine 1 tip</td><td>Set 5</td><td>CW(+1)</td><td>-1.1396</td><td>-5.0410</td><td>-0.5951</td></tr><tr><td>wing left engine 2 tip</td><td>Set 6</td><td>CW(+1)</td><td>-1.0956</td><td>-4.0490</td><td>-0.5951</td></tr><tr><td>wing left engine 1 root</td><td>Set 7</td><td>CW(+1)</td><td>-1.0456</td><td>-3.0660</td><td>-0.5951</td></tr><tr><td>wing left engine 2 root</td><td>Set 8</td><td>CW(+1)</td><td>-0.9926</td><td>-2.0670</td><td>-0.5951</td></tr><tr><td>wing right engine 1 root</td><td>Set 9</td><td>CCW(-1)</td><td>-0.9926</td><td>2.0670</td><td>-0.5951</td></tr><tr><td>wing right engine 2 root</td><td>Set 10</td><td>CCW(-1)</td><td>-1.0456</td><td>3.0660</td><td>-0.5951</td></tr><tr><td>wing right engine 1 tip</td><td>Set 11</td><td>CCW(-1)</td><td>-1.0956</td><td>4.0490</td><td>-0.5951</td></tr><tr><td>wing right engine 2 tip</td><td>Set 12</td><td>CCW(-1)</td><td>-1.1396</td><td>5.0410</td><td>-0.5951</td></tr></table>

$$
\mathbf {u} = \left[ \begin{array}{c} T x _ {1} \\ \vdots \\ T _ {x} n \\ T z _ {1} \\ \vdots \\ T z _ {n} \end{array} \right]\tag{24}
$$

The effector model mapping within the vehicle model is given by the following:

$$
\mathbf {u} \mapsto \boldsymbol {\tau}\tag{25}
$$

The inverse effector model mapping within the control allocation is given by the following:

$$
\boldsymbol {\tau} \mapsto \mathbf {u}\tag{26}
$$

The rolling and yawing torque moments generated due to the turn directions of the EDF sets are given by the following:

$$
L _ {T Q} = C _ {Q} T _ {x} t d\tag{27}
$$

$$
N _ {T Q} = C _ {Q} T _ {z} t d\tag{28}
$$

where $T _ { x } = T$ cos Γ and $T _ { z } = T$ sin Γ based on $t d = + 1$ for the clockwise direction (CW) and $t d = - 1$ for the counterclockwise (CCW) direction.

Using the simplified control and EDF sets distribution illustrated in the configuration shown in Figure $^ { 7 , }$ since EDFs do not produce forces in the Y-direction (i.e. Y propulsive = 0) and reference direction of the vertical force from the NDI is opposite that in the vehicle model $( \mathbf { i . e . \ T } _ { Z } = - Z )$ , the generalised forces and moments (commanded virtual controls τ or $\mathbf { g } _ { e } )$ are given by the following:

$$
\boldsymbol {\tau} = \left[ \begin{array}{c} F _ {X} \\ F _ {Z} \\ L _ {T} \\ M _ {T} \\ N _ {T} \end{array} \right] = \left[ \begin{array}{c} \mathbf {F} _ {p r o p} \\ \mathbf {M} _ {p r o p} \end{array} \right] = \mathbf {g} _ {e}\tag{29}
$$

where

$$
F _ {X} = t _ {x 1} T _ {a} \cos \Gamma_ {a} + t _ {x 2} T _ {b} \cos \Gamma_ {b} + t _ {x 3} T _ {c} \cos \Gamma_ {c} + t _ {x 4} T _ {d} \cos \Gamma_ {d}\tag{30}
$$

$$
F _ {Z} = t _ {z 1} T _ {a} \sin \Gamma_ {a} + t _ {z 2} T _ {b} \sin \Gamma_ {b} + t _ {z 3} T _ {c} \sin \Gamma_ {c} + t _ {z 4} T _ {d} \sin \Gamma_ {d}\tag{31}
$$

$$
L _ {T} = L _ {1} T _ {a} \cos \Gamma_ {a} + L _ {2} T _ {b} \cos \Gamma_ {b} + L _ {3} T _ {c} \cos \Gamma_ {c} + L _ {4} T _ {d} \cos \Gamma_ {d} + L _ {5} T _ {a} \sin \Gamma_ {a}
$$

$$
+ L _ {6} T _ {b} \sin \Gamma_ {b} + L _ {7} T _ {c} \sin \Gamma_ {c} + L _ {8} T _ {d} \sin \Gamma_ {d}\tag{32}
$$

$$
M _ {T} = M _ {1} T _ {a} \cos \Gamma_ {a} + M _ {2} T _ {b} \cos \Gamma_ {b} + M _ {3} T _ {c} \cos \Gamma_ {c} + M _ {4} T _ {d} \cos \Gamma_ {d} + M _ {5} T _ {a} \sin \Gamma_ {a}
$$

$$
+ M _ {6} T _ {b} \sin \Gamma_ {b} + M _ {7} T _ {c} \sin \Gamma_ {c} + M _ {8} T _ {d} \sin \Gamma_ {d}\tag{33}
$$

$$
N _ {T} = N _ {1} T _ {a} \cos \Gamma_ {a} + N _ {2} T _ {b} \cos \Gamma_ {b} + N _ {3} T _ {c} \cos \Gamma_ {c} + N _ {4} T _ {d} \cos \Gamma_ {d} + N _ {5} T _ {a} \sin \Gamma_ {a}
$$

$$
+ N _ {6} T _ {b} \sin \Gamma_ {b} + N _ {7} T _ {c} \sin \Gamma_ {c} + N _ {8} T _ {d} \sin \Gamma_ {d}\tag{34}
$$

with terms

$$
t _ {x 1} = 2, t _ {x 2} = 2, t _ {x 3} = 4, t _ {x 4} = 4, t _ {z 1} = - 2, t _ {z 2} = - 2, t _ {z 3} = - 4, t _ {z 4} = - 4
$$

$L _ { 1 } = 2 Q D i r 1 , L _ { 2 } = 2 Q D i r 2 , L _ { 3 } = 4 Q D i r 3 , L _ { 4 } = 4 Q D i r 4 , N _ { 5 } = - 2 Q D i r 1 , N _ { 6 } = 3 Q D i r 7 .$ $- 2 Q D i r 2 , N _ { 7 } = - 4 Q D i r 3 , N _ { 8 } = - 4 Q D i r 4$

L<sub>5</sub> = (∆y<sub>1</sub> + ∆y<sub>2</sub>), L<sub>6</sub> = (∆y<sub>3</sub> + ∆y<sub>4</sub>), L<sub>7</sub> = (∆y<sub>5</sub> + ∆y<sub>6</sub> + ∆y<sub>7</sub> + ∆y<sub>8</sub>), L<sub>8</sub> = (∆y<sub>9</sub> + ∆y<sub>10</sub> + ∆y<sub>11</sub> + ∆y<sub>12</sub>)

M<sub>1</sub> = (∆z<sub>1</sub> + ∆z<sub>2</sub>), M<sub>2</sub> = (∆z<sub>3</sub> + ∆z<sub>4</sub>), M<sub>3</sub> = (∆z<sub>5</sub> + ∆z<sub>6</sub> + ∆z<sub>7</sub> + ∆z<sub>8</sub>), M<sub>4</sub> = (∆z<sub>9</sub> + ∆z + ∆z + ∆z )

$$
\begin{array}{l} \bullet \quad M _ {5} = (\Delta x _ {1} + \Delta x _ {2}), M _ {6} = (\Delta x _ {3} + \Delta x _ {4}), M _ {7} = (\Delta x _ {5} + \Delta x _ {6} + \Delta x _ {7} + \Delta x _ {8}), M _ {8} = \\ \left(\Delta x _ {9} + \Delta x _ {1 0} + \Delta x _ {1 1} + \Delta x _ {1 2}\right) \end{array}
$$

$$
\begin{array}{l l} \bullet & N _ {1} = - (\Delta y _ {1} + \Delta y _ {2}), N _ {2} = - (\Delta y _ {3} + \Delta y _ {4}), N _ {3} = - (\Delta y _ {5} + \Delta y _ {6} + \Delta y _ {7} + \Delta y _ {8}), N _ {4} = \\ & - (\Delta y _ {9} + \Delta y _ {1 0} + \Delta y _ {1 1} + \Delta y _ {1 2}) \end{array}
$$

The linear effector mapping $\mathbf { B _ { i n v } } \big ( \Delta x , \Delta y , \Delta z \big ) \tau _ { c }$ is given by the following:

$$
\boldsymbol {\tau} _ {c} = \left[ \begin{array}{c c c c c} F _ {X} & F _ {Z} & L _ {T} & M _ {T} & N _ {T} \end{array} \right] ^ {T} = \mathbf {B} _ {e} (\Delta x, \Delta y, \Delta z) \mathbf {u} _ {x, z}\tag{35}
$$

where

$$
\mathbf {B} _ {e} (\Delta x, \Delta y, \Delta z) = \left[ \begin{array}{c c c c c c c c} t _ {x 1} & t _ {x 2} & t _ {x 3} & t _ {x 4} & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & t _ {z 1} & t _ {z 2} & t _ {z 3} & t _ {z 4} \\ L _ {1} & L _ {2} & L _ {3} & L _ {4} & L _ {5} & L _ {6} & L _ {7} & L _ {8} \\ M _ {1} & M _ {2} & M _ {3} & M _ {4} & M _ {5} & M _ {6} & M _ {7} & M _ {8} \\ N _ {1} & N _ {2} & N _ {3} & N _ {4} & N _ {5} & N _ {6} & N _ {7} & N _ {8} \end{array} \right]\tag{36}
$$

$$
\begin{array}{r} \mathbf {u} _ {x, z} = \left[ \begin{array}{c c c c} T _ {x 1, 2} & T _ {x 3, 4} & T _ {x 5 - 8} & T _ {x 9 - 1 2} \\ T _ {z 1, 2} & T _ {z 3, 4} & T _ {z 5 - 8} & T _ {z 9 - 1 2} \end{array} \right] ^ {T} \end{array}\tag{37}
$$

which in compact form becomes

$$
\left[ \begin{array}{c} F _ {X} \\ F _ {Z} \\ L _ {T} \\ M _ {T} \\ N _ {T} \end{array} \right] = \left[ \begin{array}{c c c c c c} t _ {x 1} & \dots & t _ {x 4} & 0 & \dots & 0 \\ 0 & \dots & 0 & t _ {z 1} & \dots & t _ {z 4} \\ L _ {1} & \dots & L _ {4} & L _ {5} & \dots & L _ {8} \\ M _ {1} & \dots & M _ {4} & M _ {5} & \dots & M _ {8} \\ N _ {1} & \dots & N _ {4} & N _ {5} & \dots & N _ {8} \end{array} \right] \left[ \begin{array}{c} T _ {x a} \\ \vdots \\ T _ {x d} \\ T _ {z a} \\ \vdots \\ T _ {z d} \end{array} \right]\tag{38}
$$

where

$$
\left[ \begin{array}{c} T _ {x a} \\ T _ {x b} \\ T _ {x c} \\ T _ {x d} \\ T _ {z a} \\ T _ {z b} \\ T _ {z c} \\ T _ {z d} \end{array} \right] = \left[ \begin{array}{c} T _ {x 1, 2} \\ T _ {x 3, 4} \\ T _ {x 5 - 8} \\ T _ {x 9 - 1 2} \\ T _ {z 1, 2} \\ T _ {z 3, 4} \\ T _ {z 5 - 8} \\ T _ {z 9 - 1 2} \end{array} \right]\tag{39}
$$

Details of the control allocation approach can be found in [50,51] as Control allocation 1 (CA 1) in 3DoF and can be easily extended to 6DoF. Details can also be found in the supplementary file.

## 5. Simulation Results and Discussions

The vehicle model and NDI controller are coded in MATLAB Simulink. All the simulations are conducted in MATLAB Simulink with Software version R2021b on a personal computer with an Intel Core i9-8950H 2.90 GHz CPU and 32 GB RAM (Santa Clara, $\mathrm { C A } , \mathrm { U S A } )$ ). The Simulink simulation was run at a fixed step sample time of 10 ms (100 Hz).

The high-level controller (outer linear and inner NDI) constants used for the model are given in Table 2, the Pilot Command filtering parameters are presented in Table $^ { 3 , }$ while the input constants into the prioritised constrained linear optimisation active set algorithm are listed below in Table 4. It is assumed that the measurements are noise-free. The constants (aircraft aerodynamics, pilot command limits, NDI and outer loop controller parameters and effector constraints) used for the model are given in Section 2.1, Tables 3–5. It is assumed that the measurements are noise-free.

Table 2. Controller simulation coefficients.

<table><tr><td>Parameter</td><td>Value</td></tr><tr><td>Outer loop translational position hold loop P-gain,  $K_{xp}$ </td><td> $1\mathrm{N}\mathrm{m}^{-1}$ </td></tr><tr><td>Outer loop translational position hold loop I-gain,  $K_{xi}$ </td><td> $0.005\mathrm{N}\mathrm{m}^{-1}$ </td></tr><tr><td>Outer loop translational position hold loop D-gain,  $K_{xd}$ </td><td> $0\mathrm{N}\mathrm{m}^{-1}$ </td></tr><tr><td>Outer loop side position control P-gain,  $K_{yp}$ </td><td> $1\mathrm{N}\mathrm{m}^{-1}$ </td></tr><tr><td>Outer loop side position control I-gain,  $K_{yi}$ </td><td> $0.01\mathrm{N}\mathrm{m}^{-1}$ </td></tr><tr><td>Outer loop side position control D-gain,  $K_{yi}$ </td><td> $0\mathrm{N}\mathrm{m}^{-1}$ </td></tr><tr><td>Outer loop sltitude control P-gain,  $K_{hp}$ </td><td> $2.5\mathrm{N}\mathrm{m}\mathrm{rad}^{-1}$ </td></tr><tr><td>Outer loop sltitude control I-gain,  $K_{hi}$ </td><td> $0.01\mathrm{N}\mathrm{m}\mathrm{rad}^{-1}$ </td></tr><tr><td>Outer loop altitude control D-gain,  $K_{hi}$ </td><td> $0\mathrm{N}\mathrm{m}\mathrm{rad}^{-1}$ </td></tr><tr><td>Outer loop bank angle control P-gain,  $K_{\phi p}$ </td><td> $1\mathrm{N}\mathrm{m}\mathrm{rad}^{-1}$ </td></tr><tr><td>Outer loop bank angle control P-gain,  $K_{\phi i}$ </td><td> $0\mathrm{N}\mathrm{m}\mathrm{rad}^{-1}$ </td></tr><tr><td>Outer loop bank angle control D-gain,  $K_{\phi d}$ </td><td> $0\mathrm{N}\mathrm{m}\mathrm{rad}^{-1}$ </td></tr><tr><td>Outer loop attitude control P-gain,  $K_{\theta p}$ </td><td> $1\mathrm{N}\mathrm{m}\mathrm{rad}^{-1}$ </td></tr><tr><td>Outer loop attitude control P-gain,  $K_{\theta i}$ </td><td> $0\mathrm{N}\mathrm{m}\mathrm{rad}^{-1}$ </td></tr><tr><td>Outer loop attitude control D-gain,  $K_{\theta d}$ </td><td> $0\mathrm{N}\mathrm{m}\mathrm{rad}^{-1}$ </td></tr><tr><td>Outer loop turn angle control P-gain,  $K_{\psi p}$ </td><td> $1\mathrm{N}\mathrm{m}\mathrm{rad}^{-1}$ </td></tr><tr><td>Outer loop turn angle control P-gain,  $K_{\psi i}$ </td><td> $0\mathrm{N}\mathrm{m}\mathrm{rad}^{-1}$ </td></tr><tr><td>Outer loop turn angle control D-gain,  $K_{\psi d}$ </td><td> $0\mathrm{N}\mathrm{m}\mathrm{rad}^{-1}$ </td></tr><tr><td>Outer loop side motion speed control P-gain,  $K_{vp}$ </td><td> $1\mathrm{N}\mathrm{m}\mathrm{rad}^{-1}$ </td></tr><tr><td>Outer loop side motion speed control P-gain,  $K_{vi}$ </td><td> $0\mathrm{N}\mathrm{m}\mathrm{rad}^{-1}$ </td></tr><tr><td>Outer loop side motion speed control D-gain,  $K_{vd}$ </td><td> $0\mathrm{N}\mathrm{m}\mathrm{rad}^{-1}$ </td></tr><tr><td>Inner loop NDI u tracking loop P-gain,  $K_{up}$ </td><td> $1.5\mathrm{N}\mathrm{m}^{-1}\mathrm{s}$ </td></tr><tr><td>Inner loop NDI w tracking loop P-gain,  $K_{wp}$ </td><td> $2.5\mathrm{N}\mathrm{m}^{-1}\mathrm{s}$ </td></tr><tr><td>Inner loop NDI p loop P-gain,  $K_{pp}$ </td><td> $1\mathrm{N}\mathrm{m}\mathrm{rad}^{-1}\mathrm{s}$ </td></tr><tr><td>Inner loop NDI q loop P-gain,  $K_{qp}$ </td><td> $1\mathrm{N}\mathrm{m}\mathrm{rad}^{-1}\mathrm{s}$ </td></tr><tr><td>Inner loop NDI r loop P-gain,  $K_{rp}$ </td><td> $1\mathrm{N}\mathrm{m}\mathrm{rad}^{-1}\mathrm{s}$ </td></tr></table>

Table 3. Command generator filtering simulation coefficients.

<table><tr><td>Parameter</td><td>Value</td></tr><tr><td>Total airspeed pilot command lower limit  $\underline{V}_{cmd}$ </td><td> $-5 \text{ m s}^{-1}$ </td></tr><tr><td>Total airspeed pilot command upper limit  $\overline{V}_{cmd}$ </td><td> $80 \text{ m s}^{-1}$ </td></tr><tr><td>Total airSpeed pilot command upper rate limit  $\dot{\overline{V}}_{cmd}$ </td><td> $2 \text{ m s}^{-2}$ </td></tr><tr><td>Total airSpeed pilot command lower rate limit  $\underline{V}_{cmd}$ </td><td> $-1 \text{ m s}^{-2}$ </td></tr><tr><td>Flight path angle pilot command upper limit  $\overline{\gamma}_{cmd}$ </td><td> $5^{\circ} \text{ s}^{-1}$ </td></tr><tr><td>Flight path angle pilot command lower limit  $\gamma_{cmd}$ </td><td> $-5^{\circ} \text{ s}^{-1}$ </td></tr><tr><td>Flight path angle pilot command upper rate limit  $\overline{\gamma}_{cmd}$ </td><td> $0.2^{\circ} \text{ s}^{-2}$ </td></tr><tr><td>Flight path angle pilot command lower rate limit  $\gamma_{cmd}$ </td><td> $-0.1^{\circ} \text{ s}^{-2}$ </td></tr><tr><td>Altitude pilot command lower limit  $\underline{h}_{cmd}$ </td><td> $-2000 \text{ m}$ </td></tr><tr><td>Altitude pilot command upper limit  $\overline{h}_{cmd}$ </td><td> $4000 \text{ m}$ </td></tr><tr><td>Altitude pilot command rate limit  $\dot{\underline{h}}_{cmd}$ </td><td> $\pm 2 \text{ m s}^{-1}$ </td></tr><tr><td>North (translational) position pilot command rate limit  $\dot{\underline{x}}_{cmd}$ </td><td> $\pm 2 \text{ m s}^{-1}$ </td></tr><tr><td>East (lateral) position pilot command rate limit  $\dot{\underline{y}}_{cmd}$ </td><td> $\pm 0.2 \text{ m s}^{-1}$ </td></tr><tr><td>Heading pilot command upper limit  $\overline{\zeta}_{cmd}$ </td><td> $5^{\circ} \text{ s}^{-1}$ </td></tr><tr><td>Heading pilot command lower limit  $\zeta_{cmd}$ </td><td> $-5^{\circ} \text{ s}^{-1}$ </td></tr><tr><td>Heading pilot command rate limit  $\dot{\overline{\zeta}}_{cmd}$ </td><td> $\pm 1^{\circ} \text{ s}^{-2}$ </td></tr><tr><td>Bank angle pilot command upper limit  $\overline{\phi}_{cmd}$ </td><td> $45^{\circ} \text{ s}^{-1}$ </td></tr><tr><td>Bank angle pilot command lower limit  $\phi_{cmd}$ </td><td> $-5^{\circ} \text{ s}^{-1}$ </td></tr><tr><td>Bank angle pilot command rate limit  $\dot{\overline{\phi}}_{cmd}$ </td><td> $\pm 4^{\circ} \text{ s}^{-2}$ </td></tr><tr><td>Pitch attitude pilot command upper limit  $\overline{\theta}_{cmd}$ </td><td> $10^{\circ} \text{ s}^{-1}$ </td></tr><tr><td>Pitch attitude pilot command lower limit  $\underline{\theta}_{cmd}$ </td><td> $-10^{\circ} \text{ s}^{-1}$ </td></tr><tr><td>Pitch attitude pilot command rate limit  $\dot{\overline{\theta}}_{cmd}$ </td><td> $\pm 0.2^{\circ} \text{ s}^{-2}$ </td></tr><tr><td>Yaw angle upper limit  $\overline{\psi}_{cmd}$ </td><td> $10^{\circ} \text{ s}^{-1}$ </td></tr><tr><td>Yaw angle lower limit  $\underline{\psi}_{cmd}$ </td><td> $-10^{\circ} \text{ s}^{-1}$ </td></tr><tr><td>Yaw angle rate limit  $\dot{\overline{\psi}}_{cmd}$ </td><td> $\pm 0.2^{\circ} \text{ s}^{-2}$ </td></tr></table>

Table 4. Active set CA algorithm input constants.

<table><tr><td>Input Constant Name</td><td>Symbol</td><td>Value</td></tr><tr><td>Linear Effectiveness Matrix</td><td> $B_e$ </td><td>See Equation (36)</td></tr><tr><td>Virtual control weighting matrix</td><td> $W_v$ </td><td>See Equation (40)</td></tr><tr><td>Control weighting matrix</td><td> $W_u$ </td><td>See Equation (41)</td></tr><tr><td>Primary objective weighting factor</td><td> $γ$ </td><td> $10^6$ </td></tr><tr><td>Maximum number of iterations</td><td> $i_{max}$ </td><td>10</td></tr><tr><td>WLS Algorithm Simulation Period</td><td> $Δt$ </td><td> $10^{-2}$ </td></tr></table>

The CA weighting matrices are as follows:

$$
\mathbf {W} _ {v} = \left[ \begin{array}{c c c c c} 5 0 & 0 & 0 & 0 & 0 \\ 0 & 1 0 0 & 0 & 0 & 0 \\ 0 & 0 & 1 0 0 0 & 0 & 0 \\ 0 & 0 & 0 & 1 0 0 0 & 0 \\ 0 & 0 & 0 & 0 & 1 0 0 0 \end{array} \right]\tag{40}
$$

$$
\mathbf {W} _ {u} = \mathbf {I} _ {8}\tag{41}
$$

where $\mathbf { I } _ { 8 }$ denotes the $8 \times 8$ identity matrix

There is no exact reference value for the tilting dynamics of EDF modelling. Hence, assumptions are made considering the small size and mass of EDFs. The tilting dynamics is modelled as a 2nd order transfer function with a natural frequency of 20 rad/s and a damping ratio of 1. Moreover, the rate limit of the tilting mechanism is taken as 60 deg/s. Minimum and maximum position limits of the tilt angles are considered separately for the EDFs on the front (canard) and rear (wing) sections. Wing EDFs are deflected between 0 deg and 120 deg. The minimum tilt angle is taken 0 deg since EDFs are on the trailing edge of the wing. It is noted that EDFs are parallel to the wing surface when the tilt angle is 0 deg. For the EDFs on the canards, there is no fixed surface, so the minimum limit is extended to 30 deg, whereas the maximum limit is the same as wing EDFs (i.e. 120 deg). The thrust (rpm) and tilting mechanisms actuator dynamics consisting of 2nd order transfer functions $T _ { 2 t f }$ and $\Gamma _ { 2 t f , }$ , with position and rate limits given in Table 5 are illustrated in Figure 9. The 2nd order transfer functions are given by

$$
T _ {2 t f} = \frac {1}{(s / \omega_ {n T}) ^ {2} + 2 \zeta_ {T} (s / \omega_ {n T}) + 1}\tag{42}
$$

$$
\Gamma_ {2 t f} = \frac {1}{(s / \omega_ {n \Gamma}) ^ {2} + 2 \zeta_ {\Gamma} (s / \omega_ {n \Gamma}) + 1}\tag{43}
$$

![](images/1899f919ca1e452c50ecb003567a8bd5db5977ad7fcd89471ed44fe822a89fb4.jpg)
Figure 9. Thrust (T) and tilting mechanism (Γ) actuator dynamics.

Table 5. Actuator physical limits and dynamics.

<table><tr><td>Actuator Control Input</td><td> $\omega_n$ </td><td>ζ</td><td>Minimum</td><td>Maximum</td><td>Rate Limit</td></tr><tr><td>Thrust for canard EDFs</td><td>25 rad/s</td><td>1</td><td>0 N</td><td>3000 N</td><td>±2000 N/s</td></tr><tr><td>Thrust for wing EDFs</td><td>25 rad/s</td><td>1</td><td>0 N</td><td>4000 N</td><td>±3000 N/s</td></tr><tr><td>Γ for canard EDFs</td><td>20 rad/s</td><td>1</td><td>-30°</td><td>120°</td><td>±60°/s</td></tr><tr><td>Γ for wing EDFs</td><td>20 rad/s</td><td>1</td><td>0°</td><td>120°</td><td>±60°/s</td></tr></table>

The unified controller is tested considering the full flight envelope of the eVTOL given as follows: vertical take-off, transition to forward flight, climb, cruise flight, descent, transition to hover flight and vertical landing.

To illustrate the results, four plots are used for each test scenario. The first plot gives the aircraft state response and commands. The second plot shows the commanded accelerations and actual acceleration. The third plot illustrates the actuator positions, which include both the direct control allocation output (commanded effector positions) with subscript “cmd” and actual effector positions states considering the actuator dynamics with subscript “res” (see Equations (42) and (43) and Figure 9). The fourth plot shows

$$
\mathcal {W} = \left[ \begin{array}{c c c c c c c c} W _ {1} & W _ {2} & W _ {3} & W _ {4} & W _ {5} & W _ {6} & W _ {7} & W _ {8} \end{array} \right] ^ {T}
$$

whether the control allocation is active or not; this can be explained as follows:

$W _ { * } = 0$ means no actuator saturation;

$W _ { * }$ = 1 means actuator saturation at the lower limit;

$W _ { * } = 1$ means actuator saturation at the upper limit;

$W _ { 1 }$ means canard left actuator EDFs thrust body x-axis component;

$W _ { 2 }$ means canard right actuator EDFs thrust body x-axis component;

$W _ { 3 }$ means wing left actuator EDFs thrust body x-axis component;

$W _ { 4 }$ means wing right actuator EDFs thrust body x-axis component;

$W _ { 5 }$ means canard left actuator EDFs thrust body z-axis component;

$W _ { 6 }$ means canard right actuator EDFs thrust body z-axis component;

• W<sub>7</sub> means wing left actuator EDFs thrust body z-axis component;

$W _ { 8 }$ means wing right actuator EDFs thrust body z-axis component.

To observe the CA algorithm’s computational properties, the $^ { \prime \prime } \mathrm { C A }$ Iteration Number” is also given in the plots. If the $^ { \prime \prime } \mathrm { C A }$ Iteration Number” is above 1, then the CA algorithm overwrites the NDI controller’s output to find the optimum allocation iteratively considering the actuator limits and channel prioritisation.

## 5.1. Maneouvre 1—VTO and Heading Control, Forward Transition

Transition from hover to forward flight is one of the most critical flight regimes regarding flight control due to the coupled vectored thrust control and severe nonlinearities during the transition (see Section 2.4). Note that nonlinearities occur in the aerodynamic model during the transition due to the blending between the hover and forward flight models.

EDFs on both canard and wing sections are tilted forward to accelerate the aircraft. Thrust is also adjusted to keep both pitch attitude and altitude at the desired level while accelerating the aircraft to high-speed forward flight. According to the trim data at 77.77 $\mathbf { m } / \mathbf { s } ,$ the canard EDFs generate a total of <sup>≊</sup>180 N $( 2 \times 8 9 . 8 $ for EDF Sets 1/2 and EDF Sets $3 / 4 )$ at both left and right sections, and the tilt angle is around $2 8 ^ { \circ }$ (0.52 rad), whereas the wing EDFs generate total of <sup>≊</sup>600 N $( 4 \times 1 4 9 . 6 0 1 8$ for EDF Sets 5–8 and EDF Sets 9–12) at both left and right sections with tilt angle of nearly $0 ^ { \circ }$

$\mathrm { A t } 5 \mathrm { s } , 2 9 8 0 \mathrm { m } , \mathrm { a }$ Vertical take-off (VTO) altitude command $( h _ { c m d } = 3 0 0 0 \mathrm { m } )$ is applied at a constant climb rate o $: 2 \mathrm { m } / \mathrm { s } ,$ and the controller tracks the corresponding vertical velocity command $( w _ { c m d } )$

$\mathrm { A t } 8 \mathsf { s } ,$ the aircraft heading is changed by commanding a yaw angle command $( \psi _ { c m d } )$ of $\psi = 3 0 ^ { \circ }$ to change the aircraft heading while climbing vertically, and then the heading returns back to $\psi = 0 ^ { \circ }$

Between 20 s and $7 0 \mathrm { s } ,$ forward transition is commanded by simultaneously applying velocity command in the x and z direction body axis $u _ { c m d } , w _ { c m d }$ (via airspeed $V _ { c m d }$ and flight path angle $\gamma _ { c m d }$ commands) and AoA α. An airspeed speed V of $7 7 . 8 \mathrm { m } / \mathrm { s }$ was commanded at an acceleration of 4 m $\mathbf { s } ^ { - 2 }$ , fpa of $0 ^ { \circ }$ and pitch attitude of $2 . 8 4 ^ { \circ }$ at a constant rate of 0.01 rad $\mathbf { s } ^ { - 1 }$ was commanded over a 17.5s duration.

The commanded and actual states, commanded and actual effector positions, and control authority (actuator limitation) status are shown in Figures 10–14.

![](images/6ac239aad6907c67ddbae02b2210fcc6646805fa4087126fa15caac585c2e80a.jpg)
Figure 10. VTO, forward transition—attitude.

![](images/b48623d9832a4fadf001142061b015982155642a497081aae42c2da3efe5deaf.jpg)
Figure 11. VTO, forward transition—velocity.

![](images/1582c35e59863af680e9143769088de9fd8050919c99459e45f9e30320d5feab.jpg)
Figure 12. VTO, forward transition—effector commands.

![](images/33689455909f2bd17cad6c87fc7622f8ed37cbd6466a1489c2cbcc86ce9eea1b.jpg)
Figure 13. VTO, forward transition—acceleration.

![](images/9dad126a494c36e7c0fe6c5206be3e87160360c0e5b03c5d77e3baf41917f5c0.jpg)
Figure 14. VTO, forward transition—control limitation status.

## Analysis of Manueovre 1 Results

During the forward transition, both the airspeed (V) and AoA (α) are commanded simultaneously so that the cruise trim AoA α of 2.84◦ is reached before cruise trim airspeed, thus ensuring very low tilt angles in order to reduce rotor-induced drag (resulting in high power consumption) that occurs at high airspeed and high actuator tilt angles. This is different from the forward transition performed in [48,53] where cruise trim speed was first commanded before trim AoA was commanded resulting in a period of time in between of high rotor-induced drag and high power consumption due to high tilt angle at high forward speed.

Figures 10 and 11 show the state response of the forward transition. The aircraft climbed to 3000 m at first and the commands heading initially changed to 30◦ and then back to 0◦ before accelerating to 77.77 m/s after 39 s with an approximate acceleration of 2 m $/ { \mathrm { s } } ^ { 2 } .$ , which can be seen in the u˙ plot in Figure 13.

According to the actuator position plot given in Figure 12, both front and wing section EDFs’ tilt angle decrease after 20s to provide thrust for the forward acceleration. During the forward acceleration, thrust levels decreased by <sup>≊</sup> 95% (Canard EDF Set from 3048 N to 149 N, Wing EDF Set from 3048 N to 149 N, i.e below 10% (canard 5.4% and main wing 4.89%) compared to hover on both front and wing EDFs since the lift is primarily provided by the wings and canards leaving the EDF to only provide forward thrust. The thrust decrease at the wing EDFs is higher compared to the canard EDFs. This is an expected result since with the increasing airspeed, the lift generated by the wings increases and generates positive pitch moment. To balance the lift-generated pitch moment, the front EDFs lost less thrust (generated more thrust) compared to the wing EDFs from Hover to Cruise.

The NDI controller is designed to control the body angular and translational velocities, see Equations (15) and (16). Therefore, it is possible to control the angle of attack and flight path angle. To achieve 2.84◦ the trim angle of attack, combined $2 . 8 4 ^ { \circ } , \theta _ { c m d } , 0 ^ { \circ } \gamma _ { c m d }$ and 77.8 $\mathbf { m } / \mathbf { s } V _ { c m d }$ are applied at 20 s (see Figure 10). Desired trim values are reached without any loss of altitude (See Figure 10), and trim states are achieved with very small errors such as the following: $\alpha _ { t r i m } = 2 . 8 5 ^ { \circ } , w _ { t r i m } = 3 . 8 5 \mathrm { m } / \mathrm { s } , u _ { t r i m } = 7 7 . 7 7 \mathrm { m } / \mathrm { s } , \beta _ { t r i m } \cong 0 ^ { \circ } , \gamma _ { t r i m } \cong 0 ^ { \circ }$

Based on Figure 12, thrust values at trim are 91 N (89.819 N at cruise trim) for the front EDFs and 148 N (149.60 N at cruise trim) for the wing EDFs. Moreover, tilt angles are 28.62◦ (28◦ at cruise trim) for the front EDFs and 0.46◦ (0.0013229◦ at cruise trim) for the wing EDFs.

The performance of the CA approach and Active Set algorithm is also analysed. The last plot of Figures 13 and 14 shows the CA iteration number, more explicitly whether the CA algorithm is actively working or not. At the end of the forward transition, as can be seen from Figure 12, the unconstrained CA (Moore–Penrose pseudo-inverse) output $( \Delta \mathbf { u } _ { u c }$ in Figure 6) violated the minimum tilt angle limit of the wing EDF (see $W _ { 7 }$ and $W _ { 8 } )$ limits ( 1.2◦) at 59 s resulting the Active Set Constrained (ASC) CA algorithm being active (see Figure 14) imposing actuator constraint limitations on the CA output which prevent violation of the physical actuator constraints.

The CA performance is also investigated while considering the real-time implementation. According to the last plot of Figures 13 and 14, the maximum “CA Iteration Number” are below 10 when the active set constrained (ASC) CA is active. Also, Figure 14 shows that minimum tilt angle limits were violated for the wing EDFs—see plots $7 \ ( W _ { 7 } )$ and $8 ( W _ { 8 } )$ . Therefore, the CA algorithm finds the optimal solution very fast. Moreover, the CA algorithm does not cause discontinuities/sudden jumps in the actuator states which is crucial for the real time implementation.

## 5.2. Maneouvre 2—Cruise, Reverse Transition and VL with Heading Change

Transition from high speed cruise flight to low speed hover flight refered to as "Reverse Transition" is also a critical flight regime regarding the flight control due the coupled vectored thrust control and severe nonlinearities during the transition. Note that nonlinearities occur in the aerodynamic model during the reverse transition due to the blending between the forward flight and hover models.

The reverse transition test starts at the cruise condition with $h = 3 0 0 0 \mathrm { m } , 2 . 8 4 ^ { \circ }$ degree angle of attack and $7 7 . 8 7 \mathrm { m } / \mathrm { s }$ airspeed. During the reverse transition, both the airspeed (V) and AoA (α) are commanded simultaneously so that the trim AoA α of $2 . 8 4 ^ { \circ }$ was reduced to $0 ^ { \circ }$ before cruise trim airspeed decreased from $7 7 . 8 7$ m $/ s$ to hover speed of 0 $\mathbf { m } / \mathbf { s } .$ . This ensures that the duration of high–low tilt angles is minimised in order to reduce rotorinduced drag (resulting in high power consumption) that occurs at high airspeed and high actuator tilt angles. This is different from the forward transition performed in [48,53] where the cruise trim AoA (α) was first commanded to $0 ^ { \circ }$ before the cruise trim speed was first commanded before decreasing from 77.87 m/s to a hover speed of $0 \mathrm { m } / s ,$ which resulted in a period of time in between of high rotor-induced drag and high power consumption due to high tilt angle at high forward speed.

The commanded and actual states, commanded and actual effector positions, and control authority (actuator limitation) status are shown in Figures 15–19.

![](images/e76aafe8ce0c89849405ab3dd43bfc520614eb1f2dcf986d7583a0eed0ee5cb7.jpg)
Figure 15. Cruise, reverse transition, and VL with heading change—attitude.

![](images/e564d856fbb3bb2b7be0da77517d5558ab1418cde01cbdd354b2a3cb1dc7a4f8.jpg)
Figure 16. Cruise, reverse transition, and VL with heading change—velocity.

![](images/476f62689438063945ea4a4bfc79c86c8de65f609ff7f18cedee007c202d1355.jpg)
Figure 17. Cruise, reverse transition, and VL with heading change—effector commands.

![](images/6606be3dedd27dc9fda161d7f1f5d60177a28458104b71ef8b05ef9007ebf2e7.jpg)
Figure 18. Cruise, reverse transition, and VL with heading change—acceleration.

![](images/8bd3c512300ae316540ea65837c1d9137369c76cb361a97b07ce2797907b4537.jpg)
Figure 19. Cruise, reverse transition, and VL with heading change—control limitation status.

## Analysis of Manueovre 2 Results

As expected, tilt angles and rpm/thrust increase during the reverse transition since the lift generated by the wing reduces with the decreasing airspeed. ${ \mathrm { A t ~ } } t = 8 2 { \mathrm { ~ s } } _ { \mathrm { , } }$ , the aircraft reaches the hover condition with $9 0 ^ { \circ }$ tilt angles and hover trim thrust values (Figure 17). At $t = 8 2 \mathrm { s } , h = 2 9 8 0$ m command (VL) and $( \psi = - 3 0 ^ { \circ } )$ heading change were simultaneously applied, and the aircraft lands vertically at $t = 1 0 2$ s and completed the $( - 3 0 ^ { \circ } )$ heading change at $t = 1 0 0 \mathrm { s } .$

This is different from the forward transition performed in [48,53] where the cruise trim AoA (α) was first commanded to $0 ^ { \circ }$ before the cruise trim speed was first commanded before decreasing from 77.87 m $/ s$ to hover speed of 0 m/s, which resulted in a period of time in between high rotor-induced drag and high power consumption due to high tilt angle at high forward speed.

During the initial phase of the reverse transition, as can be seen from Figure 17, the high-level controller outputs violated all the tilt angle position $( W _ { 1 } { - } W _ { 7 } )$ limits between 5 s and 7.5 s, which resulted in the CAs being active (see Figure 19) imposing actuator constraint limitations on the CA output which prevent violation of the physical actuator constraints. Also, the canard left and right tilt angle rates $( W _ { 5 }$ and $W _ { 6 } )$ limits were violated several times as can be seen from Figures 18 and 19.

In general, the controller’s performance is satisfactory during the reverse transition and vertical landing as the maximum number of control allocation iterations is 20, which is satisfactory for real-time safety-critical software applications.

## 5.3. Maneouvre 3—Cruise, Climb, Descend, Cruise

Climb and descent manoeuvers are tested at cruise conditions. The commanded and actual states, commanded and actual effector positions, and control authority (actuator limitation) status are shown in Figures 20–24. Beginning at cruise altitude 3000 m, first, $4 ^ { \circ }$ flight path angle commands $( \gamma _ { c m d } )$ ) are applied between 5 s and 26 s during which the aircraft climbed 200 m (to 3200 m). Then $- 2 ^ { \circ }$ flight path angle commands $( \gamma _ { c m d } )$ are applied between 26 s and 130 s during which the aircraft descended back to cruise altitude (3000 m) and then continues in cruise. The climb FPA ( ) of $4 ^ { \circ }$ is set by a combination of pitch attitude $( \theta _ { c m d } = 6 . 3 6 7 2 ^ { \circ } )$ and FPA while maintaining cruise $\mathrm { A o A } ( \alpha _ { c m d } = 2 . 3 6 7 2 ^ { \circ } ) ;$ the descent FPA $( \gamma ) \mathrm { o f } - 2 ^ { \circ }$ is set by a combination of pitch attitude $( \theta _ { c m d } = 0 ^ { \circ } )$ and FPA while setting an $\mathrm { A o A } \ : ( \alpha _ { c m d } = 2 ^ { \circ } )$ . This manoeuvre, though not a typical manoeuvre, is used in emergencies to clear obstacles along eVTOL air taxi flight paths, and since it shows climb and descent manoeuvres at cruise conditions, which are performed within a short time window, it is suitable for performance analysis in flight simulations.

![](images/db0318ac2d5b840d92b382f3726e6fcb4ca43e8918eb1b6cf16d2ff7666edc66.jpg)
Figure 20. Climb, descend—attitude.

![](images/634ee9eca33d40c664701e0014da9699550fb0af12ddea84d52c958ad7dfa8ea.jpg)
Figure 21. Climb, descend—velocity.

![](images/c2c9495e1d4b955f0f47b95a34035b1562c3c12f16f2e8db2d7fddc18d2fb4b3.jpg)
Figure 22. Climb, descend—effector commands.

![](images/6b3825fa9fcefa92899cd6a0aded264d3ac4554ff2fafb01a8b76cfc65211686.jpg)
Figure 23. Climb, Descend—Acceleration.

![](images/fd9b8fee4000c870d4a46b277eb449868c058364b5fa007365b135ba2a056da1.jpg)
Figure 24. Climb, descend—control limitation status.

## Analysis of Manueovre 3 Results

The Controller’s performance is satisfactory during the climb and descent manoeuvres. According to the actuator position plot given in Figure 22, the CA algorithm becomes active several times during the climb and descent manoeuvres. Lower position tilt angle limit and tilt angle rate limits are reached numerous times during the manoeuvre (see Figures 22–24). Note that the maximum iteration number of the CA algorithm is less than 20, and the actuator commands are continuous (Figures 23 and 24).

## 5.4. Maneouvre 4—Cruise and Coordinated Turn

The coordinated turn maneuvers are tested at cruise conditions. The commanded and actual states, commanded and actual effector positions, and control authority (actuator limitation) status are shown in Figures 25–29. Beginning at cruise altitude 3000 m, with a forward speed of 77.77 m/s along 0◦ flight path angle commands $( \gamma _ { c m d } )$ , a coordinated turn is performed between 5 s and 40 s by applying a $3 0 ^ { \circ }$ roll angle command (see Figures 25 and 26).

To reduce the altitude loss behaviour during the turn, the aircraft can be trimmed for a coordinated turn (level flight, turning, non-skidding flight) at cruise using a trim bank angle and corresponding turn rate and turn radius, which ensures an equilibrium state (balance of horizontal and vertical force).

![](images/831f8962bfa61455d71b591d895e41a2b3da05898d618f8e9be8536b1ac52678.jpg)
Figure 25. Cruise and coordinated turn—attitude.

![](images/2329f2691957263142244f355895938fac59e24a990222e9fa53b1832c151a30.jpg)
Figure 26. Cruise and coordinated turn—velocity.

![](images/5ac7f011de86b67016c71630d6c024f7d4c6d87b5650ebf7b0a88a1c1a3eee92.jpg)
Figure 27. Cruise and coordinated turn—effector commands.

![](images/24699e379712a9dbe0b2f8e680bb0547246c91e24e639b9e0390cd43024b2169.jpg)
Figure 28. Cruise and coordinated turn—acceleration.

![](images/6e8a916f46a477bc9f0673c77cbcb0101f4b85a8764d742ae9bfbdc7659bdf83.jpg)
Figure 29. Cruise and coordinated turn—control limitation status.

## Analysis of Manueovre 4 Results

To maintain altitude during a coordinated turn (level turn), lift must be increased via increasing AoA (α) by increasing pitch, assuming that cruise airspeed is maintained, which, in turn, will increase drag (i.e. since drag increases with lift). To compensate for the increase in drag, thrust must be increased. Figure 27 shows that all EDFs increased in thrust and tilt angles during the turn to compensate for reductions in aerodynamic lift and increases in aerodynalic drag. Since it was a right turn $( \phi _ { C t u r n } = 2 0 ^ { \circ } )$ , the right EDFs (canard from 90 N to 289 N, wing from 148 N to 483 N) experienced a higher thrust increase compared to the left EDFs (canard from 90 N to 283 N, wing from 148 N to 460 N). However, differential increase in tilt angle between the canard right EDFs (canard from <sup>≊</sup> $2 8 ^ { \circ } ~ \mathrm { t o } \approxeq 6 4 ^ { \circ } )$ and left EDFs (canard from <sup>≊</sup> $2 8 ^ { \circ } \ \mathrm { t o } \approxeq 8 4 ^ { \circ } )$ . The stall speed in turn is also increased but the bank angle is limited. The yaw rate compensation during turn suppressed adverse yaw effects ensuring a non-skidding non side-slipping turn. The roll rate compensations ensures sufficient lift due to banking. The CA prioritizes rotational channels over the translational channels to satisfy stable flight in case of actuator saturation. This prioritisation order can be seen in Figure 27. Translational commands $( u _ { c m d }$ and $w _ { c m d } )$ are not tracked well (Figure 26 shows a small increase) when the CA prioritizes the rotational channels to ensure that the rotational commands are tracked accurately; hence, the aircraft gained altitude (increased from 3000 m to 3011 m) during the turns (altitude response after 50 s) in Figure 25) as evident from $\approxeq 0 . 2 ^ { \circ }$ increase in flight path angle γ during the turn. Although $\bar { \theta } _ { C t u r n } \approx 0 ^ { \circ }$ and $\psi _ { C t u r n } \approxeq 0 ^ { \circ }$ were as expected during a coordinated turn (see Figure 28), it was not a perfect non-skidding, non-sliding, and level turn as there was small side-slipping $( \beta \approxeq 0 . 2 ^ { \circ }$ in Figure 25) and skidding (v<sup>≊</sup> 0.4 m/s in Figure 26).

The performance of the CA is also investigated considering the real-time implementation. According to Figure 29, the maximum CA software iterations $C A _ { i t e r }$ is 20 when the CA is active. Therefore, the CA algorithm finds the optimal solution very fast. Again, the CA algorithm does not cause discontinuities/sudden jumps in the actuator positions, which is crucial for real-time implementation since it considers the actuator position and rate limits.

## 6. Conclusions and Recommendations

This paper verifies a nonlinear unified controller used for full envelope control of a fixed-wing over-actuated vectored thrust eVTOL, which is inspired by the 7-seater Lilium Jet. The unified controller consists of an overall (high-level) aircraft-level controller and a control allocation. The aircraft-level controller consists of a command generator, a cascade of an outer loop (primary loop) position/attitude linear controller, which is independent of the airframe dynamics, and an inner loop (secondary loop) velocity NDI controller, which is dependent on the aircraft dynamics. The simulation results demonstrate that the NDI-based unified controller can naturally handle the eVTOL’s wide range of operating conditions, thus removing the need for gain scheduling and leading to improved levels of performance over conventional flight controller designs, which use linearising approximations. The tracking capabilities of the NDI scheme and potentially the operational safety during vertical take-off (VTO) with heading change, forward transition (FT), reverse transition, climb, descent, coordinated turn, and vertical landing with heading change are also demonstrated. The aircraft has many tilting EDFs to control the thrust vector; therefore, the system is highly over-actuated. Over-actuation and vectored thrust control (with nonlinear effector mapping) pose another challenge for flight control. The challenge is to properly allocate the limited control authority in order to guarantee stable flight in severe flight conditions, which is known as control allocation. For the studied eVTOL air taxi, the simulation results show that proper allocation of the limited control authority is very critical and must be solved carefully as part of the controller design. To allocate the limited control authority in case of actuator saturation, the mapping between the overall aircraft control demand $\pmb { \tau } _ { c }$ (virtual controls or generalised forces and moments) and the real control controls u (individual effectors/actuators), which is known as effector mapping, needs to be formulated. For the studied vectored thrust eVTOL air taxis, this mapping is a vectored thrust control, and the CA problem becomes more complicated due to the nonlinear effector mapping. First, the nonlinear effector mapping is transformed to a linear one, and then the linear optimisation-based AS WLS CCA approach is applied and integrated into the NDI controller, given the described challenges. The CA performance is also investigated, given real-time implementation. The maximum number of CA iterations “CA Iteration Number” are all below 20 when the AS CCA is active. Therefore, the CA algorithm finds the optimal solution quickly. Moreover, the CA algorithm does not cause discontinuities/sudden jumps in the actuator states, which is crucial for real-time implementation.

The next stage of this research work will be to further improve the controller (high-level motion controller and control allocation) for achieving the required flying and handling performance for this kind of vehicle by including a reference model. Also, since NDI relies on an accurate model of the system, which is not possible and hence not inherently robust to model mismatches and external disturbances, the robustness of the NDI controller should be improved by incorporating a robust control scheme. Further, more representative mission task elements (MTE) for flying and handling quality performance assessments will be tested and simulated. Finally, pilot-in-the-loop simulation trials will be conducted at

Cranfield University flight simulators to evaluate full envelope control design performance and handling qualities.

Supplementary Materials: The following supporting information can be downloaded at: https: //www.mdpi.com/article/10.3390/aerospace11120979/s1.

Author Contributions: Conceptualisation, E.E.; data curation, E.E.; formal analysis, E.E.; funding acquisition, E.E. and J.F.W.; investigation, E.E.; methodology, E.E.; project administration, J.F.W. and L.L.; resources, E.E.; software, E.E.; supervision, J.F.W. and L.L.; validation, E.E.; Visualisation, E.E.; writing—original draft, E.E.; writing—review and editing, E.E. and J.F.W. All authors have read and agreed to the published version of the manuscript.

Funding: This research did not receive any specific grant from funding agencies in the public, commercial, or not-for-profit sectors.

Data Availability Statement: MATLAB<sup>⃝R</sup> and Simulink<sup>⃝R</sup> simulation codes are available from the first author on request.

Conflicts of Interest: The authors declare no conflicts of interest.

## Abbreviations

The following abbreviations are used in this manuscript:

<table><tr><td>AS</td><td>active set</td></tr><tr><td>CA</td><td>control allocation</td></tr><tr><td>CCA</td><td>constrained control allocation</td></tr><tr><td>CTL</td><td>control laws</td></tr><tr><td>DoF</td><td>degrees of freedom</td></tr><tr><td>eVTOL</td><td>Electrical vertical take-off and landing</td></tr><tr><td>FBW</td><td>Fly-by-wire</td></tr><tr><td>LP</td><td>Linear programming</td></tr><tr><td>LTV</td><td>Linear time varying</td></tr><tr><td>QP</td><td>Quadratic programming</td></tr><tr><td>UCA</td><td>unconstrained control allocation</td></tr><tr><td>UAM</td><td>Urban air mobility</td></tr><tr><td>UAV</td><td>Unmanned air vehicle</td></tr><tr><td>VL</td><td>Vertical landing</td></tr><tr><td>VTO</td><td>Vertical take-off</td></tr><tr><td>WLS</td><td>Weighted least squares</td></tr><tr><td colspan="2">Nomenclature</td></tr><tr><td>x</td><td>vector of state variables</td></tr><tr><td>x,y,z</td><td>position of the centre of gravity of the vehicle along the north, east and down respectively from starting point</td></tr><tr><td>V</td><td>airspeed</td></tr><tr><td> $V_b$ </td><td>linear velocity vector of the body</td></tr><tr><td>ω</td><td>angular velocity vector of the body</td></tr><tr><td>X,Y,Z</td><td>net aerodynamic force acting at the centre of gravity in the body x,y and z-axis direction</td></tr><tr><td>u,v,w</td><td>velocities in the body x,y and z-axis direction</td></tr><tr><td>p,q,r</td><td>angular rates</td></tr><tr><td> $K_{out}$ </td><td>high-level CTL outer loop linear controller</td></tr><tr><td>y</td><td>NDI high-level CTL vector of outputs to be controlled</td></tr><tr><td>K</td><td>NDI outer loop (feedback) linear controller gain vector</td></tr><tr><td> $K_{inn}$ </td><td>NDI controller</td></tr><tr><td>r</td><td>high-level CTL vector of reference signals to be tracked</td></tr><tr><td>v</td><td>NDI high-level CTL outer loop controller output vector</td></tr><tr><td>e</td><td>NDI high-level CTL inner loop control error vector</td></tr><tr><td>u</td><td>vector of effector demands or real controls</td></tr><tr><td>T</td><td>effector thrust</td></tr><tr><td>*1,2,*a</td><td>front/canard left EDFs actual and control demands</td></tr><tr><td>*3,4,*b</td><td>front/canard right EDFs actual and control demands</td></tr><tr><td>*5-8,*c</td><td>rear/main wing left EDFs actual and control demands</td></tr><tr><td>*9-12,*d</td><td>rear/main wing right EDFs actual and control demands</td></tr><tr><td>f</td><td>nonlinear system plant state transition matrix</td></tr><tr><td>g</td><td>affine nonlinear control control effectiveness matrix</td></tr><tr><td>Be</td><td>linear control effectiveness matrix</td></tr><tr><td>Binv</td><td>Moore-Penrose inverse of the linear control effectiveness matrix</td></tr><tr><td></td><td>slack variable representing the control error in all control channels and degrees of freedom</td></tr><tr><td>S</td><td></td></tr><tr><td>Wv</td><td>weight matrix or control channel priority matrix</td></tr><tr><td>Wu</td><td>actuator utilisation or prioritisation weight positive definite matrix</td></tr><tr><td>U</td><td>actuator saturation constraints  $u_{min}$  and  $u_{max}$  or set of admissible controls</td></tr><tr><td>C</td><td>actuator rate constraints  $\dot{u}_{min}$  and  $\dot{u}_{max}$ </td></tr><tr><td>A</td><td>set of attainable moments</td></tr><tr><td>J</td><td>secondary objective cost function</td></tr><tr><td></td><td>maximum number of Active Set control allocation optimisation software iterations</td></tr><tr><td>imax</td><td></td></tr><tr><td>up</td><td>preferred actuator position</td></tr><tr><td>uop</td><td>optimal actuator position</td></tr><tr><td>uc</td><td>unconstrained actuator position using Moore-Penrose inverse</td></tr><tr><td>T0</td><td>actuator thrust at time t = 0 or previous software iteration</td></tr><tr><td>TX*,TY*,TZ*</td><td>rotor thrust in the body x, y, and z-directions</td></tr><tr><td>FX,FY,FZ</td><td>overall vehicle (virtual) propulsive thrust in the body x, y, and z-directions</td></tr><tr><td>m</td><td>vehicle mass</td></tr><tr><td>g</td><td>gravitational acceleration constant</td></tr><tr><td>Ixx,Iyy,Izz</td><td>moment of inertia about the body x, y, and z-axis</td></tr><tr><td>Ixz</td><td>moment of inertia about the body x - z plane</td></tr><tr><td>La,Ma,Na</td><td>net aerodynamic moment acting about the body x, y, and z-direction</td></tr><tr><td>LT,MT,NT</td><td>net vehicle pitching moment acting about the body x, y, and z-direction</td></tr><tr><td>h</td><td>output matrix</td></tr><tr><td>G(x)</td><td>product of (Jacobian  $\frac{\partial h}{\partial x}$  of output matrix h with respect to x) and u</td></tr><tr><td>F(x)</td><td>product of (Jacobian  $\frac{\partial h}{\partial x}$  of output matrix h with respect to x) and f(x)</td></tr><tr><td>Δt</td><td>sampling period or time increment from previous to current software iteration</td></tr><tr><td></td><td>vector of real controls in Cartesian form (the x- and z- coordinate rotor thrust components)</td></tr><tr><td> $\bar{x}$ </td><td>maximum limit</td></tr><tr><td> $\bar{*}$ </td><td>minimum limit</td></tr><tr><td> $\ddot{x}$ </td><td>maximum rate limit</td></tr><tr><td> $\dot{*}$ </td><td>minimum rate limit</td></tr><tr><td>In</td><td>the n × n identity matrix</td></tr><tr><td>Ib</td><td>vehicle moment of inertia matrix</td></tr><tr><td> $\phi ,\theta ,\psi$ </td><td>Euler angles—roll, pitch and yaw angles</td></tr><tr><td> $\lambda ,\zeta$ </td><td>flight path or glide angle, and course angle or heading angle</td></tr><tr><td> $\alpha ,\beta$ </td><td>angle of attack and side-slip angle</td></tr><tr><td> $\gamma$ </td><td>primary objective weighting factor ensuring Wu &lt;&lt; Wv</td></tr><tr><td>Γ</td><td>effector tilt angle</td></tr><tr><td>Γ0</td><td>actuator tilt angle at time t = 0 or previous software iteration</td></tr><tr><td>Δ</td><td>control change or increment from previous to current software iteration</td></tr><tr><td>τc</td><td>virtual or overall aircraft control command</td></tr><tr><td>τ</td><td>actual generalised forces and moments</td></tr><tr><td>Δt</td><td>sampling period or time increment from previous to current software iteration</td></tr></table>

## References

1. Enenakpogbe, E.; Whidborne, J.F.; Lu, L. Control of an over-actuated fixed-wing vectored thrust eVTOL. In Proceedings of the 14th UKACC International Conference on Control (CONTROL 2024), Winchester, UK, 10–12 April 2024; pp. 315–316. https://doi.org/10.1109/CONTROL60310.2024.10531934.

2. NASA. UAM Vision Concept of Operations (ConOps) UAM Maturity Level (UML) 4. Available online: https://ntrs.nasa.gov/ citations/20205011091 (accessed on 6 November 2021)

3. Hascaryo, R.W.; Merret, J.M. Configuration-independent initial sizing method for UAM/eVTOL vehicles. In Proceedings of the AIAA Aviation 2020 Forum, Virtual, 15–19 June 2020; Number AIAA-2020-2630. https://doi.org/10.2514/6.2020-2630.

4. Prempain, E.; Postlethwaite, I.; Vorley, D. A gain scheduled autopilot design for a bank-to-turn missile. In Proceedings of the 2001 European Control Conference (ECC), Porto, Portugal, 4–7 September 2001; pp. 2052–2057. https://doi.org/10.23919/ECC. 2001.7076224.

5. Prempain, E.; Turner, M.C.; Sandou, G.; Duc, G.; Vorley, D.; Harcaut, J.P. Dynamic controllers flight control design over a large flight envelope. In Proceedings of the Materials and Components for Missiles—Innovation & Technology Partnership Conference, Manchester, UK, 5 October 2010; pp. CD–Rom.

6. Ducard, G.J.J.; Allenspach, M. Review of designs and flight control techniques of hybrid and convertible VTOL UAVs. Aerosp. Sci. Technol. 2021, 118, 107035. https://doi.org/10.1016/j.ast.2021.107035.

7. Lombaerts, T.; Kaneshige, J.; Feary, M. Control concepts for simplified vehicle operations of a quadrotor eVTOL vehicle. In Proceedings of the AIAA AVIATION 2020 Forum, Virtual, 15–19 June 2020; Number AIAA-2020-3189. https://doi.org/10.2514 6.2020-3189.

8. Marks, A.; Whidborne, J.F.; Yamamoto, I. Control allocation for fault tolerant control of a VTOL octorotor. In Proceedings of the UKACC International Conference on Control 2012, Cardiff, UK, 3–5 September 2012; pp. 357–362. https://doi.org/10.1109/ CONTROL.2012.6334656.

9. Littell, J. Challenges in vehicle safety and occupant protection for autonomous electric vertical take-off and landing (eVTOL) vehicles. In Proceedings of the AIAA Propulsion and Energy 2019 Forum, Indianapolis, IN, USA, 19–22 August 2019; Number AIAA 2019-4504. https://doi.org/10.2514/6.2019-4504.

10. Straubinger, A.; Rothfeld, R.; Shamiyeh, M.; Büchter, K.D.; Kaiser, J.; Plötner, K.O. An overview of current research and developments in urban air mobility—Setting the scene for UAM introduction. J. Air Transp. Manag. 2020, 87, 101852. https: //doi.org/10.1016/j.jairtraman.2020.101852.

11. Ploetner, K.O.; Al Haddad, C.; Antoniou, C.; Frank, F.; Fu, M.; Kabel, S.; Llorca, C.; Moeckel, R.; Moreno, A.T.; Pukhova, A.; et al. Long-term application potential of urban air mobility complementing public transport: An upper Bavaria example. CEAS Aeronaut. J. 2020, 11, 991–1007. https://doi.org/10.1007/s13272-020-00468-5.

12. Tan, C.M.H. Multidisciplinary Modeling & Simulation Framework for Electric Vertical Take-Off & Landing (eVTOL) Vehicles. Mater’s Thesis, Esslingen University of Applied Sciences, Esslingen am Neckar, Germany, 2020.

13. Vertical Flight Society. Vertical Flight Society Announces Continued Strong Growth. Available online: https://vtol.org/files/ dmfile/vfspressrelease-2020growth\_200113.pdf (accessed on 30 October 2020).

14. Airbus. A<sup>3</sup> by Airbus. A<sup>3</sup> Vahana. Available online: https://acubed.airbus.com (accessed on 30 October 2022).

15. Lilium. Lilium GmbH. Lilium jet. Available online: https://lilium.com (accessed on 30 October 2022).

16. Padfield, G.D.; Lu, L. The potential impact of adverse aircraft-pilot couplings on the safety of tilt-rotor operations. Aeronaut. J. 2022, 126, 1617–1647. https://doi.org/10.1017/aer.2022.20.

17. Lane, S.H.; Stengel, R.F. Flight control design using non-linear inverse dynamics. Automatica 1988, 24, 471–483. https: //doi.org/10.1016/0005-1098(88)90092-1.

18. Smith, P.; Berry, A. Flight test experience of a non-linear dynamic inversion control law on the VAAC Harrier. In Proceedings of the Atmospheric Flight Mechanics Conference, Denver, CO, USA, 14–17 August 2000; Number AIAA-2000-3914. https: //doi.org/10.2514/6.2000-3914.

19. Smith, P.R. A simplified approach to nonlinear dynamic inversion based flight control. In Proceedings of the 23rd Atmospheric Flight Mechanics Conference, Boston, MA, USA, 10–12 August 1998; Number AIAA-98-4461.https://doi.org/10.2514/6.1998-446 1.

20. Ito, D.; Georgie, J.; Valasek, J.; Ward, D.T. Reentry vehicle flight controls design guidelines: Dynamic inversion. In NASA Technical Report NASA/TP-2002-210771; NASA: Washington, DC, USA, 2002.

21. Georgie, J.; Valasek, J. Evaluation of longitudinal desired dynamics for dynamic-inversion controlled generic reentry vehicles. J. Guid. Control. Dyn. 2003, 26, 811–819. https://doi.org/10.2514/2.5116.

22. Valasek, J.; Georgie, J. Selection of longitudinal desired dynamics for dynamic inversion controlled re-entry vehicles. In Proceedings of the AIAA Guidance, Navigation, and Control Conference and Exhibit, Montreal, QC, Canada, 6–9 August 2001; Number AIAA-2001-4382. https://doi.org/10.2514/6.2001-4382.

23. Miller, C. Nonlinear dynamic inversion baseline control law: Architecture and performance predictions. In Proceedings of the AIAA Guidance, Navigation, and Control Conference, Portland, OR, USA, 8–11 August 2011; Number AIAA-2011-6467. https://doi.org/10.2514/6.2011-6467.

24. Holzapfel, F.; Sachs, G. Dynamic inversion based control concept with application to an unmanned aerial vehicle. In Proceedings of the AIAA Guidance, Navigation, and Control Conference and Exhibit, Providence, RI, USA, 16–19 August 2004; Number AIAA-2004-4907. https://doi.org/10.2514/6.2004-4907.

25. Gregory, I. Modified dynamic inversion to control large flexible aircraft — What’s going on? In Proceedings of the Guidance, Navigation, and Control Conference and Exhibit, Portland, OR, USA, 9–11 August 1999; Number AIAA-99-3998, p. 3998. bttps://doi org/10.2514/6.1999-3998

26. Smith, P.R. Functional control law design using exact non-linear dynamic inversion. In Proceedings of the 19th Atmospheric Flight Mechanics Conference, Scottsdale, AZ, USA, 1–3 August 1994; Number AIAA-94-3516-CP. https://doi.org/10.2514/6.1994-3516.

27. Bugajski, D.J.; Enns, D.F.; Elgersma, M.R. A dynamic inversion based control law with application to the high angle-of-attack research vehicle. In Proceedings of the Guidance, Navigation and Control Conference, Portland, OR, USA, 20–22 August 1990; Number AIAA-90-3407-CP. https://doi.org/10.2514/6.1990-3407.

28. Enns, D.; Bugajski, D.; Hendrick, R.; Stein, G. Dynamic inversion: An evolving methodology for flight control design. Int. J. Control 1994, 59, 71–91. https://doi.org/10.1080/00207179408923070.

29. Hameduddin, I.; Bajodah, A.H. Nonlinear generalised dynamic inversion for aircraft manoeuvring control. Int. J. Control 2012, 85, 437–450. https://doi.org/10.1080/00207179.2012.656143.

30. Yang, H.; Morales, R. Robust full-envelope flight control design for an eVTOL vehicle. In Proceedings of the AIAA Scitech 2021 Forum, Virtual, 11–15 and 19–21 January 2021; Number AIAA 2021-0254. https://doi.org/10.2514/6.2021-0254.

31. Adams, R.J.; Banda, S.S. An integrated approach to flight control design using dynamic inversion and µ-synthesis. In Proceedings of the 1993 American Control Conference, San Francisco, CA, USA, 2–4 June 1993; pp. 1385–1389. https://doi.org/10.23919 /ACC.1993.4793098.

32. Adams, R.J.; Banda, S.S. Robust flight control design using dynamic inversion and structured singular value synthesis. IEEE Trans. Control Syst. Technol. 1993, 1, 80–92. https://doi.org/10.1109/87.238401.

33. Adams, R.J.; Buffington, J.M.; Banda, S.S. Design of nonlinear control laws for high-angle-of-attack flight. J. Guid. Control. Dyn. 1994, 17, 737–746. https://doi.org/10.2514/3.21262

34. Stevens, B.L.; Lewis, F.L.; Johnson, E.N. Aircraft Control and Simulation: Dynamics, Controls Design, and Autonomous Systems; John Wiley: Hoboken, NJ, USA, 2015.

35. Fossen, T.I.; Johansen, T.A. A survey of control allocation methods for ships and underwater vehicles. In Proceedings of the 14th Mediterranean Conference on Control and Automation, Ancona, Italy, 28–30 June 2006; pp. 1–6. https://doi.org/10.1109/MED. 2006.328749.

36. Johansen, T.A.; Fossen, T.I. Control allocation—A survey. Automatica 2013, 49, 1087–1103. https://doi.org/10.1016/j.automatica. 2013.01.035.

37. Durham, W.; Bordignon, K.A.; Beck, R. Aircraft Control Allocation; John Wiley: Hoboken, NJ, USA, 2017. https://doi.org/10.1002/ 9781118827789.

38. Santos, D.A.; Bezerra, J.A. On the control allocation of fully actuated multirotor aerial vehicles. Aerosp. Sci. Technol. 2022, 122, 107424. https://doi.org/10.1016/j.ast.2022.107424.

39. Bodson, M. Evaluation of optimization methods for control allocation. J. Guid. Control. Dyn. 2002, 25, 703–711. https: //doi.org/10.2514/2.4937.

40. Harkegard, O. Efficient active set algorithms for solving constrained least squares problems in aircraft control allocation. In Proceedings of the 41st IEEE Conference on Decision and Control (CDC2002), Las Vegas, NV, USA, 10–13 December 2002; pp. 1295–1300. https://doi.org/10.1109/CDC.2002.1184694.

41. Petersen, J.A.; Bodson, M. Constrained quadratic programming techniques for control allocation. IEEE Trans. Control Syst. Technol. 2005, 14, 91–98. https://doi.org/10.1109/TCST.2005.860516.

42. Petersen, J.A.; Bodson, M. Interior-point algorithms for control allocation. J. Guid. Control. Dyn. 2005, 28, 471–480. https: //doi.org/10.2514/1.5937.

43. Nocedal, J.; Wright, S.J. Numerical Optimization; Springer: Berlin/Heidelberg, Germany, 1999.

44. Dong, Y.; Shi, Z.; Chen, K.; Chen, J. Experimental investigation of the effects of sideslip on canard-configuration aircraft at high angle of attack. AIP Adv. 2019, 9, 055114. https://doi.org/10.1063/1.5093559.

45. Wibowo, S.B.; Sutrisno, S.; Rohmat, T.A. The influence of canard position on aerodynamic characteristics of aircraft in delaying stall conditions. Aip Conf. Proc. 2018, 2021, 060028. https://doi.org/10.1063/1.5062792.

46. Lombaerts, T.; Kaneshige, J.; Schuet, S.; Hardy, G.; Aponso, B.; Shish, K.H. Nonlinear dynamic inversion based attitude control for a hovering quad tiltrotor eVTOL vehicle. In Proceedings of the AIAA Scitech 2019 Forum, San Diego, CA, USA, 7–11 January 2019; Number AIAA-2019-0134. https://doi.org/10.2514/6.2019-0134.

47. Lombaerts, T.; Kaneshige, J.; Schuet, S.; Aponso, B.L.; Shish, K.H.; Hardy, G. Dynamic inversion based full envelope flight control for an eVTOL vehicle using a unified framework. In Proceedings of the AIAA Scitech 2020 Forum, Orlando, FL, USA, 6–10 January 2020; p. 1619. https://doi.org/10.2514/6.2020-1619.

48. Suiçmez, E.C. Full Envelope Nonlinear Controller Design for a Novel Electric VTOL (eVTOL) Air-Taxi via INDI Approach Combined with CA. Ph.D. Thesis, Middle East Technical University, Çankaya/Ankara, Turkey, 2021.

49. Enenakpogbe, E.; Whidborne, J.F.; Lu, L. Control of an eVTOL using nonlinear dynamic inversion. In Proceedings of the 13th UKACC International Conference on Control (CONTROL 2022), Plymouth, UK, 20–22 April 2022; pp. 158–164. https://doi.org/10.1109/Control55989.2022.9781449

50. Enenakpogbe, E.; Whidborne, J.F.; Lu, L. Control allocation problem transformation approaches for over-actuated vectored thrust VTOLs. Aerosp. Sci. Technol. 2024. Submitted.

51. Enenakpogbe, E. Integrated NDI-based Controller and Incremental Control Allocation for an eVTOL. Ph.D. Thesis, Cranfield University, Cranfield, UK, 2025; To be submitted.

52. Moses, D. Modelling and Simulation of an eVTOL with Distributed Propulsion Architecture. Master’s Thesis, Cranfield University, Cranfield, UK, 2021.

53. Suiçmez, E.C.; Kutay, A.T. Full envelope nonlinear flight controller design for a novel electric VTOL (eVTOL) air taxi. Aeronaut. J. 2023, 128, 966–993. https://doi.org/10.1017/aer.2023.87.

54. Welles, A.V. Hybrid Electric Distributed Propulsion for Vertical Takeoff and Landing Aircraft. Ph.D. Thesis, Syracuse University, Syracuse, NY, USA, 2018.

55. Kaneshige, J.; Lombaerts, T.; Shish, K.H.; Feary, M. Command and control concepts for a lift plus cruise electric vertical takeoff and landing vehicle. In Proceedings of the AIAA AVIATION 2023 Forum, San Diego, CA, USA, 12–16 June 2023; number AIAA 2023-3910. https://doi.org/10.2514/6.2023-3910

56. Feary, M.S.; Kaneshige, J.; Lombaerts, T.; Shish, K.; Haworth, L. Evaluation of novel eVTOL aircraft automation concepts. In Proceedings of the AIAA AVIATION 2023 Forum, San Diego, CA, USA, 12–16 June 2023; Number AIAA 2023-3909. https: //doi.org/10.2514/6.2023-3909.

Disclaimer/Publisher’s Note: The statements, opinions and data contained in all publications are solely those of the individual author(s) and contributor(s) and not of MDPI and/or the editor(s). MDPI and/or the editor(s) disclaim responsibility for any injury to people or property resulting from any ideas, methods, instructions or products referred to in the content.