Article

# Design and Analysis of a Launcher Flight Control System Based on Incremental Nonlinear Dynamic Inversion <sup>†</sup>

Pedro Simplício \* , Paul Acquatella and Samir Bennani <sup>‡</sup>

European Space Agency (ESA), European Space Technology Centre (ESTEC), 2201 AZ Noordwijk, The Netherlands; paul.acquatella@esa.int (P.A.)

Correspondence: pedro.simplicio@esa.int

† This article is an improved and extended version of the paper presented at the 12th International ESA Conference on Guidance, Navigation and Control Systems, Sopot, Poland, 12–16 June 2023. https://doi.org/10.5270/esa-gnc-icatt-2023-030.

<sup>‡</sup> Retired Senior GNC System Advisor.

![](images/d04d5eb63bdde3817621aab5f886b07fe32a044851862a2cea76f4be5334ef2f.jpg)

Academic Editor: Mikhai Ovchinnikov

Received: 24 January 2025 Revised: 24 March 2025 Accepted: 28 March 2025 Published: 31 March 2025

Citation: Simplício, P.; Acquatella, P.; Bennani, S. Design and Analysis of a Launcher Flight Control System Based on Incremental Nonlinear Dynamic Inversion. Aerospace 2025, 12, 296. https://doi.org/10.3390/ aerospace12040296

Copyright: © 2025 by the authors. Licensee MDPI, Basel, Switzerland. This article is an open access article distributed under the terms and conditions of the Creative Commons Attribution (CC BY) license (https://creativecommons.org/ licenses/by/4.0/).

Abstract: This paper investigates the application of Incremental Nonlinear Dynamic Inversion (INDI) for launch vehicle flight control, addressing the limited exploration of nonlinear control architectures and their potential benefits in the context of the current “New Space” era. In this context, this study aims to bridge the gap between the launcher’s traditional linear control practice and nonlinear methods, focusing on INDI, which offers the potential to enhance limits of performance while reducing mission preparation (“missionisation”) efforts. INDI control commands incremental inputs by exploiting feedback acceleration estimates in a feedback-linearised plant in order to reduce model dependency, making it easier to design and resulting in a robust closed loop as compared to nonlinear dynamic inversion. The objective of this paper is therefore to demonstrate INDI’s implementation in a representative industrial launch ascent scenario, evaluate its strengths and limitations relative to industry standards, and promote its adoption within the launcher Guidance, Navigation, and Control (GNC) community. Comparative simulations with traditional scheduled PD controllers, with and without angular acceleration feedback, are highlighted together with several trade-offs. Furthermore, this paper presents a new and practical INDI stability analysis method as applied in the context of aerospace attitude control, as well as an augmentation of the design with an outer control loop for active load relief. Results indicate that while INDI exhibits increased sensitivity to sensor noise and actuator delays as compared to the linear controllers, its advantages in robustness and performance are significant. Notably, INDI’s ability to handle nonlinearities without extensive tuning and gain-scheduling surpasses the capabilities of the traditional linear control counterparts. These results highlight the potential of INDI as a more robust and efficient alternative to state-of-practice launcher control design methodologies.

Keywords: robust control; nonlinear control; acceleration feedback; stability analysis; launch vehicles; load relief

## 1. Introduction

## 1.1. Background and Motivation

The space industry has undergone a significant transformation in recent years, characterised by the emergence of the “New Space” era. This era is marked by disruptive changes in business models, manufacturing technologies, and agile development practices within launch vehicle companies, all aimed at minimising production and operating costs in an increasingly competitive market. Despite this focus on innovation, the potential benefits of advancements in control theory, such as increasing performance limits and streamlining mission preparation (or “missionisation”) efforts, have received limited attention.

Moreover, government-led developments of recent launchers such as Ares I and VEGA still adhere to the same design approach of the Saturn V, i.e., linear control [1]. This approach relies on single-channel tuning and ad hoc gain-scheduling followed by extensive validation and verification (V&V) processes, all of which are time-consuming and resource-intensive.

In contrast to the approach presented above, the past few years have seen a growing interest in the application of artificial intelligence and machine learning methods for launcher GNC, but the industrial use of such data-driven/model-free methods remains limited by well-known issues related to training and certification of the algorithms on the full flight envelope of intended operation. In that sense, there is a clear gap between these strategies and the current state of practice, in which other techniques could bring relevant improvements; this is the case for nonlinear control algorithms, especially those based on Nonlinear Dynamic Inversion (NDI). On the one hand, agile practices of New Space companies provide the ideal opportunity to explore the benefits of this type of design approach. On the other hand, a successful adoption of nonlinear launcher control will likely facilitate the augmentation with and transition to data-driven methods in the future. This is therefore our motivation and aim for this paper, to start bridging the gap between these two approaches while presenting a potential alternative based on incremental nonlinear control.

## 1.2. Related Work

In this paper, we briefly introduce and focus on (Incremental) Nonlinear Dynamic Inversion (NDI), which is a control design method based on feedback linearisation [2,3]; it basically consists of a nonlinear (state feedback) transformation that linearises the nominal system dynamics and a linear part that imposes the desired closed-loop dynamics. NDI is a very well-known and applied (nonlinear) control technique in the aerospace field, especially in aeronautics for various flight control applications [4–8]. Successful implementation of NDI requires a match between the onboard model and the system model and accurate knowledge of all nonlinearities, which is often not the case in reality; this results in poor robustness properties because they rely on exact availability of the system dynamics. This highlights the need for robustness in these methodologies, as the inner loop of the control system is critical and can be compromised by model and sensor uncertainties, potentially affecting stability and performance. In this regard, alternative methods involving robust ness and improvements of the method for NDI-based flight control applications were considered, among many others, in [9–13].

A successful technique that became popular in recent years for aerospace applications is Incremental Nonlinear Dynamics Inversion (INDI). The concept using incremental nonlinear control was first developed in the late 1990s and was initially focused on the “implicit” dynamic inversion for DI-based flight control. The works of Smith, Bacon, and others laid the foundation for these developments [6,11], for which the term “incremental” is now more commonly used to describe this methodology as it better reflects the nature of these control laws [14–16]. Those early studies further developed the incremental approach, and since then, it has been further elaborated theoretically and successfully applied in various highperformance systems including fault-tolerant control of aircraft subjected to sensor and actuator faults [17,18], in practice for quadrotors using adaptive control [19,20], in real flight tests of small (unmanned) and business jet (Cessna Citation II, PH-LAB) aircraft [21–23]; it has also been used in studies in spacecraft attitude control [24–26], and it is planned to be flight-tested in the upcoming “Reusability Flight Experiment (ReFEx)” by DLR [27–30].

However, its applicability to launch vehicles has not been fully investigated yet but only considered in [31–34] and more recently in [35]. These related works have demonstrated INDI’s performance and robustness against aerodynamic model uncertainties and disturbance rejection for several aerospace vehicles; hence, the potential benefits of INDI are quite relevant for reusable launchers which have much tighter dynamical couplings between online-generated trajectory and attitude control during descent flight. Moreover, due to the nonlinear nature of INDI, it has proven difficult to attain analytical proof of stability, which has been derived in [36].

With this paper, which is an extended and improved version of [37], we aim to further close this gap towards the application of INDI for launchers, with special focus on the ascent of a TVC-controlled launcher, and also to present a new, practical approach for stability analysis of such INDI control laws applied for attitude control. Furthermore, the extension from [37] in this work considers an outer loop for load relief control to handle the uncontrolled drift dynamics that arise when performing attitude control only.

## 1.3. Objectives and Outline

It is therefore the objective of this paper to showcase the implementation of INDI on a representative application scenario; to highlight its strengths and limitations in the face of the industrial state of practice; and to raise awareness of the INDI method among the launcher GNC community. To achieve this, this paper provides a concise description of the NDI and INDI approaches, followed by the detailed design and comparison of different control laws: linear, linear with angular acceleration feedback, and INDI-based. Furthermore, this paper also aims to address the (main) two well-known challenges associated with the practical implementation of INDI-based control:

Sensitivity to sensor noise and actuator delay. By relying on angular acceleration and control input measurements/estimates, INDI controllers are generally more sensitive to sensor noise and actuator delay than classical controllers. To assess the severity of this challenge, this paper shows a comprehensive nonlinear simulation campaign with wind disturbances and uncertainties, as well as different levels of sensor noise and actuator delay. These simulations serve as a basis to analyse the sensitivity to sensor noise and actuator delay in comparison to more classical approaches, and we showcase how to remediate or tackle these issues properly.

Nonlinear stability analysis. The second challenge of INDI is that due to its nonlinear nature, attaining analytical proof of stability is not trivial [36]. For this second challenge, this paper proposes a simple yet insightful linearisation-based approach to evaluate stability degradation related to an inexact feedback linearisation and to deviations from the control tuning conditions. This method provides a new way to analyse and evaluate stability analysis of the nonlinear controller using linear control techniques; since INDI is designed from the theory of feedback linearisation, this approach is very intuitive in the sense that it provides a measure of degradation with respect to the feedback-linearised plant, and linear stability analysis can be performed.

Lastly, this paper also addresses improvements and extensions of [37] regarding load relief. This is necessary within this launch vehicle ascent context because of the “internal” dynamics that arises when performing (I)NDI for attitude control only; therefore, we address the following challenge:

Drift dynamics. We propose an extension to our launcher flight control system including an active load relief outer loop. Since the drift states can grow unbounded depending on the attitude control architecture, an outer loop providing the reference pitch angle for the inner-loop attitude and rate control can alleviate the drift and mitigate aerodynamic loads during ascent.

To demonstrate the benefits and challenges of the INDI approach, we showcase the method within an application scenario consisting of a launcher model during ascent flight while featuring attitude and lateral drift degrees of freedom, actuator dynamics, and moving-mass effects. All the controllers and filters are implemented at a sampling frequency that is compatible with current onboard capabilities (25 Hz).

The outline of this paper is as follows. A brief introduction to Nonlinear Dynamic Inversion (NDI) and Incremental NDI is presented in Section 2. Section 3 presents the launch flight dynamics modelling aspects and describes the simulator used for the attitude control design and testing. Launcher attitude control designs including angular acceleration feedback are presented in Section 4. Time-domain robust performance results and analysis of the obtained simulations comparing the controllers studied are presented in Section $5 ,$ while Section 6 presents the frequency-domain stability results and analysis. Section 7 presents an extension to flight control with active load relief, and finally, conclusions are presented in Section 8.

## 2. Basic Principles of (Incremental) Nonlinear Dynamic Inversion

## 2.1. Nonlinear Dynamic Inversion (NDI)

We consider a multiple-input and multiple-output (MIMO) system whose number of inputs are equal to the number of outputs without loss of generality, to avoid control allocation complexities. Let us also assume momentarily that the nonlinear system can be described affine in the inputs as

$$
\dot {x} = f (x) + g (x) u\tag{1a}
$$

$$
\boldsymbol {y} = \boldsymbol {h} (\boldsymbol {x})\tag{1b}
$$

where $\textstyle { \boldsymbol { x } } \in { \mathcal { R } } ^ { n }$ is the state vector, $u \in \mathcal { R } ^ { m }$ is the control input vector, and $y \in \mathcal { R } ^ { m }$ is the system output vector, the functions $f ( x )$ and $h ( x )$ are assumed to be smooth vector fields on $\mathcal { R } ^ { n }$ , and $g ( x ) \in \mathcal { R } ^ { n \times m }$ is a matrix whose columns are also assumed as smooth vector fields $g _ { j }$ . For these systems, the vector of relative degree $\rho$ represents the number of differentiations of each output $y _ { i } , i = 1 , \ldots , m$ , that are needed for the input to appear [2,3]. Moreover, the vector of the relative degree of the continuous-time nonlinear system satisfies $\rho = \| \rho \| _ { 1 } \leq n ,$ , where $\rho$ is the total relative degree of the system. In this brief introduction to NDI, we consider $y = x$ so that the relative degree of each output $y _ { i }$ is one, and therefore the system is said to be full-state feedback-linearisable. Whenever the total relative degree is strictly less than the order of the system $( \rho < n )$ , the nonlinear system is then decomposed into an external (input–output) part and an internal (unobservable) part which is described by a new set of $( n - \rho )$ -coordinates [26,36].

Nonlinear Dynamic Inversion (NDI) is a technique that aims to eliminate the nonlinearities present in a given nonlinear system, resulting in closed-loop dynamics that can be expressed in a linear form. To achieve this, the nonlinear system is inverted into a linear structure using state feedback, making it possible to apply conventional linear controllers. However, NDI has a significant disadvantage in that it relies on the fundamental assumption that the system model is known exactly, making it vulnerable to uncertainties. Additionally, NDI assumes that the system state is fully and accurately known, which can be challenging to achieve in practice. NDI involves applying the following input transformation [2,3]:

$$
\boldsymbol {u} _ {\mathrm{cmd}} = \boldsymbol {g} ^ {- 1} (\boldsymbol {x}) (\boldsymbol {\nu} - \boldsymbol {f} (\boldsymbol {x}))\tag{2}
$$

which cancels all nonlinearities in a closed loop, and the simple linear input–output rela tionship or “single-integrator” form $\dot { y } = \nu$ between the new virtual control input ν and the output $_ y$ is obtained. In the case that the relative degree of each output $y _ { i }$ is $^ { 2 , }$ the relationship would be a double integrator $\ddot { y } = \nu .$ , and so on. In addition to being linear, an interesting feature of this relationship is that it is also decoupled, meaning that the input $\nu _ { i }$ only affects the output $y _ { i } ;$ this property makes (2) a decoupling control law. By utilising appropriate control techniques (e.g., linear, robust), the single-integrator form can be rendered exponentially stable. The single integrator can be made exponentially stable through the use of

$$
\pmb {\nu} = \dot {\pmb {y}} _ {\mathrm{des}}\tag{3}
$$

where, for instance, $\dot { y } _ { \mathrm { d e s } } = \dot { y } _ { \mathrm { c m d } } + K _ { P } e$ defines the desired dynamics for the output vector or control variables. Here, $\dot { y } _ { \mathrm { c m d } }$ is a feedforward term, while $e = { \pmb y } _ { \mathrm { c m d } } - { \pmb y }$ represents the error vector, and $y _ { \mathrm { c m d } }$ denotes the (smooth) desired output vector which is (in this case, since the relative degree is one) at least once-differentiable. The gain matrix $K _ { P } \in \mathcal { R } ^ { m \times m }$ is used to ensure that the polynomials given by $s + K _ { P _ { i } }$ for $i = 1 , \ldots , m$ become Hurwitz. The diagonal elements $K _ { P _ { i } }$ of $K _ { P }$ are then selected accordingly. As a result of using (3), the desired error dynamics ${ \dot { e } } _ { i } + K _ { P _ { i } } e _ { i } = 0$ become exponentially stable and decoupled, leading to $e _ { i } ( t ) \to 0 { \mathrm { f o r } } i = 1 , \ldots , m$ . Here, ${ \dot { y } } _ { \mathrm { d e s } }$ can be designed in any other way as long as the closed-loop system remains stable while guaranteeing desired performance requirements.

## 2.2. Incremental Nonlinear Dynamic Inversion (INDI)

Incremental Nonlinear Dynamic Inversion (INDI) consists of the application of NDI to a system expressed in an incremental form $\left[ 1 5 , 1 6 , 3 6 \right]$ . To obtain a system in incremental form, first we introduce a sufficiently small time delay λ and define the following deviation variables $\dot { { \boldsymbol x } } _ { 0 } : = \dot { { \boldsymbol x } } ( t - \lambda ) , { \boldsymbol x } _ { 0 } : = { \boldsymbol x } ( t - \lambda )$ , and $\pmb { u } _ { 0 } : = \pmb { u } ( t - \lambda )$ ; these are the λ-time-delayed signals of the current state derivative ${ \dot { \mathbf { } } } ( t )$ , state $x ( t )$ , and control ${ \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf { } } { \mathbf } { } { \mathbf } { } \mathbf { } { \mathbf { } } { \mathbf } { } { \mathbf } { } \mathbf { }  { \mathbf { } \mathbf { } } { \mathbf { } \mathbf { } } { \mathbf { } \mathbf { } } { \mathbf } { \mathbf { } } { \mathbf } { \mathbf } { \mathbf { } } { \mathbf } { \mathbf } { } { \mathbf } { \mathbf } { \mathbf } { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } { \mathbf } { \mathbf } { \mathbf } { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf { } \mathbf  { \mathbf }  \mathbf \mathbf { } \mathbf { } \mathbf { } \mathbf \mathbf$ , respectively [25]. Moreover, we denote $\Delta \dot { \boldsymbol { x } } : = \dot { \boldsymbol { x } } - \dot { \boldsymbol { x } } _ { 0 } , \Delta \boldsymbol { x } : = \boldsymbol { x } - \boldsymbol { x } _ { 0 } ,$ , and $\Delta \boldsymbol { u } : = \boldsymbol { u } - \boldsymbol { u } _ { 0 }$ as the incremental state derivative, the incremental state, and the so-called incremental control input, respectively. Subsequently, we consider a first-order Taylor series expansion of x˙ not in the geometric sense but with respect to the newly introduced time delay λ as [15,16,25,36]

$$
\begin{aligned} \dot{x} & = \dot{x}_{0} + \frac{\partial}{\partial x}\big[f(x) + g(x)u\big]\bigg|_{\substack{x = x_{0}\\ u = u_{0}}}\Delta x + g(x_{0})\Delta u + H.O.T\\ & = \dot{x}_{0} + g(x_{0})\Delta u + N(x,\lambda) \end{aligned}\tag{4}
$$

with ${ \dot { \boldsymbol { x } } } _ { 0 } = f ( { \boldsymbol { x } } _ { 0 } ) + g ( { \boldsymbol { x } } _ { 0 } ) { \boldsymbol { u } } _ { 0 }$ and

$$
N(x,\lambda) = \frac{\partial}{\partial x}\Big[f(x) + g(x)\boldsymbol {u}\Big]\Big|_{\substack{x = x_{0}\\ u = u_{0}}}\Delta x + H.O.T\tag{5}
$$

which represents a residual containing the Jacobian linearisation of the onboard model and the higher-order terms (H.O.T) of the series expansion. Notice that the model-based control effectiveness $g ( x _ { 0 } )$ is sampled at the previous incremental time. This means an approximate linearisation about the λ−delayed signals is performed incrementally and not with respect to a particular equilibrium or operational point of interest.

Furthermore, it is assumed that during a sufficiently small time increment $\lambda ( \mathrm { i . e . }$ ., for a sufficiently high control frequency), the input u can change significantly faster than the state x [15,16,25,36]. This implies that there is a reasonable frequency separation between the dynamics of the vehicle and of the actuators, which is generally the case for aerospace systems. Under this assumption, the following approximation holds:

$$
\Delta x = x - x _ {0} \cong 0, \text {   even   if   } \Delta u = u - u _ {0} \neq 0\tag{6}
$$

which leads to

$$
\dot {\boldsymbol {x}} \cong \dot {\boldsymbol {x}} _ {0} + \boldsymbol {g} (\boldsymbol {x} _ {0}) \Delta \boldsymbol {u} + \underbrace {\boldsymbol {N} (\boldsymbol {x} , \lambda)} _ {\cong 0} \Rightarrow \Delta \dot {\boldsymbol {x}} \cong \boldsymbol {g} (\boldsymbol {x} _ {0}) \Delta \boldsymbol {u}\tag{7}
$$

This assumption, corroborated by the fact that the perturbation term $N ( x , \lambda )$ satisfies [36]:

$$
\lim _ {\lambda \to 0} \| N (x, \lambda) \| _ {2} \to 0, \forall x\tag{8}
$$

implies that the nonlinear system dynamics in its incremental form is approximated at each time-step by the model-based control effectiveness $g ( x _ { 0 } )$ . Finally, applying NDI to the system based on the approximation (7) results in a relation between the incremental control input and the output of the system:

$$
\pmb {u} = \pmb {u} _ {0} + \pmb {g} (\pmb {x} _ {0}) ^ {- 1} (\pmb {\nu} - \dot {\pmb {x}} _ {0})\tag{9}
$$

It shall be noted that the availability of $\dot { { \boldsymbol x } } _ { 0 }$ is required, and that the incremental input $\pmb { u } _ { 0 }$ is obtained from the output of the actuators or estimated from an actuator dynamical model; recall it has been assumed that a commanded control is achieved sufficiently fast with regard to the actuator dynamics. The total control command along with the obtained linearising control $\pmb { u } _ { 0 } = \pmb { u } ( t - \lambda )$ ) can be rewritten as

$$
\pmb {u} (t) = \pmb {u} (t - \lambda) + \pmb {g} (\pmb {x} (t - \lambda)) ^ {- 1} [ \pmb {\nu} - \dot {\pmb {x}} (t - \lambda) ]\tag{10}
$$

This improves the robustness of the closed-loop system as compared with conventional NDI since dependency on the accurate knowledge of the plant dynamics is reduced; more specifically, the dependency on accurate knowledge of the dynamic model in $f ( x )$ is largely decreased. Therefore, the INDI control law design is more dependent on accurate measurements or accurate estimates of ${ \dot { \mathbf { x } } } _ { 0 } ,$ , the state derivatives, and $u _ { 0 } ,$ , the incremental control input, respectively. INDI control is therefore model- and sensor-based because of the model-based control effectiveness $g ( x _ { 0 } )$ and the necessity to measure or sense the additional quantities of $\scriptstyle { \boldsymbol { \mu } } _ { 0 }$ and $\dot { x } _ { 0 }$

## 3. Launcher Flight Dynamics and Control Synthesis Models

## 3.1. Flight Dynamics

The present study relies on a conventional single-axis launcher model in ascent flight featuring lateral drift (z) and pitch (θ) dynamics, as schematised in Figure 1. These dynamics, representing the first and second time derivatives of z as $\{ \dot { z } = w , \ddot { z } = w \}$ together with the first and second time derivatives of θ as $\{ \dot { \theta } = q , \ddot { \theta } = \dot { q } \}$ }, are governed by the well-known nonlinear Newton–Euler equations:

$$
m \dot {w} = F _ {\alpha} + F _ {\mathrm{c}} + F _ {\mathrm{n}} - m g \sin \theta\tag{11a}
$$

$$
J \dot {q} = M _ {\alpha} + M _ {\mathrm{c}} + M _ {\mathrm{n}}\tag{11b}
$$

where $m , J ,$ and $g$ are the launcher’s mass, lateral moment of inertia, and gravity acceleration, respectively; $\{ F _ { \alpha } , M _ { \alpha } \}$ are the normal aerodynamic forces and moments, respectively; $\{ F _ { \mathrm { c } } , M _ { \mathrm { c } } \}$ are the TVC-induced forces and moments, respectively; and $\{ F _ { \mathrm { n } } , M _ { \mathrm { n } } \}$ are the engine nozzle moving-mass effects, also known as tail-wags-dog (TWD).

![](images/00aabe7de0b9723152c76a23220fe9ff6629b802028d961ff48c8350ebbc383d.jpg)
Figure 1. Launcher model diagram.

The aerodynamic force consists of an axial component (A in Figure 1) and a normal component $( N _ { \alpha } \alpha )$ , which varies linearly with the angle of attack α. Only the latter is considered in this single-axis launcher model, and the induced force and moment are computed as

$$
F _ {\alpha} = - N _ {\alpha} \alpha = - S C _ {N _ {\alpha}} Q \alpha\tag{12a}
$$

$$
M _ {\alpha} = - l _ {\alpha} F _ {\alpha} = l _ {\alpha} S C _ {N _ {\alpha}} Q \alpha\tag{12b}
$$

where $S , C _ { N _ { \alpha } }$ , and $l _ { \alpha }$ are the reference aerodynamic area, lateral force gradient, and aerodynamic arm (distance between the launcher’s centres of pressure and gravity). Qα is the aerodynamic load indicator, defined as the product between aerodynamic pressure and angle of attack, which are, respectively, given by

$$
Q = \frac {1}{2} \rho V ^ {2}\tag{13a}
$$

$$
\alpha = \theta + \arctan \frac {w - l _ {\alpha} q - v _ {\mathrm{w}}}{V}\tag{13b}
$$

where $\rho$ is the air density, V is the total airspeed, and $v _ { \mathrm { w } }$ is the lateral wind turbulence speed. The term $l _ { \alpha } q$ is often known as aerodynamic damping. The TVC-induced force and moment are computed as

$$
F _ {\mathrm{c}} = - T \sin \beta\tag{14a}
$$

$$
M _ {\mathrm{c}} = l _ {\mathrm{c}} F _ {\mathrm{c}} = - l _ {\mathrm{c}} T \sin \beta\tag{14b}
$$

where T is the thrust magnitude, $l _ { \mathrm { c } }$ is the TVC arm (distance between the launcher’s centre of gravity and nozzle’s pivot point), and $\beta$ is the TVC deflection angle. Finally, the nozzle TWD effects (dynamics) are computed as

$$
F _ {n} = - m _ {n} l _ {n} \ddot {\beta}\tag{15a}
$$

$$
M _ {n} = l _ {c} F _ {n} - J _ {n} \ddot {\beta} = - (m _ {n} l _ {n} l _ {c} + J _ {n}) \ddot {\beta}\tag{15b}
$$

where $m _ { \mathrm { n } }$ is the nozzle moving mass, $l _ { \mathrm { n } }$ is the moving-mass arm (distance between the nozzle’s centre of gravity and pivot point), $\ddot { \beta }$ is the TVC deflection acceleration, and $J _ { \mathrm { n } }$ is the nozzle moment of inertia with respect to the pivot point (not to the centre of gravity).

Most of the model’s parameters vary along the launcher’s trajectory (this dependence was not evidenced in the previous equations for the sake of readability) and are highly uncertain. These parameters were extracted as a function of time from the simulator presented in [38] for an 80 s trajectory. The uncertainty levels adopted in this study are summarised in Table 1 and are assumed to remain constant throughout the flight.

Table 1. Uncertainty level per type of parameter.

<table><tr><td>Type of Parameters</td><td>Variables</td><td>Uncertainty Level</td></tr><tr><td>Aerodynamics</td><td> $C_{N_{\alpha}}, l_{\alpha}, \rho, V$ </td><td>20%</td></tr><tr><td>Mass/propulsion</td><td> $m, J, l_{c}, T$ </td><td>10%</td></tr></table>

Note that while mass/propulsion parameters have an explicit dependency on time, related to the way the propellant burns, aerodynamics parameters have an implicit depen dency through intermediate quantities such as altitude and Mach number.

## 3.2. Actuator Dynamics

In addition to the launcher model described above, the present study considers the dynamical effects of TVC actuation and wind turbulence. Both effects are modelled as time-invariant transfer functions for the sake of simplicity without loss of generality. The TVC dynamics corresponds to a reduced second-order model of the system used in [39], which is given by

$$
G _ {\mathrm{TVC}} (\mathrm{s}) = \frac {6 7 . 8 ^ {2}}{\mathrm{s} ^ {2} + 9 0 . 9 \mathrm{s} + 6 7 . 8 ^ {2}}\tag{16}
$$

where s represents the Laplace variable. The wind turbulence speed $v _ { \mathrm { w } }$ is modelled by colouring a white noise signal through a first-order Dryden filter [40] given by

$$
G _ {\mathrm{w}} (\mathbf {s}) = \frac {3 . 5 4}{\mathbf {s} + 0 . 3 2}\tag{17}
$$

The launcher, TVC, and wind models were put together in a simulator that allows us to quickly analyse and compare several control systems, which is illustrated in Figure 2. In this figure, different simulation rates are highlighted using different colours: black for the continuous-time dynamics, red for GNC computations $( f _ { \mathrm { G N C } } = 2 5 \mathrm { H z } ,$ which is well representative of current onboard capabilities), and green for wind noise generation $( f _ { \mathrm { w } } = 2 0$ Hz in this case).

![](images/b5d1f9079b0f43738a69b68fc3509ee6b0db7aea764b7347958471ed9a3c5abb.jpg)
Figure 2. Launcher simulator diagram.

## 3.3. Nonlinear Model

In this section, we conveniently express the dynamics in state-space form. For that, consider the following coefficients [39]; first, relative to the rotational motion,

$$
\bar {\mu} _ {\alpha} = \frac {l _ {\alpha} Q S C _ {N _ {\alpha}}}{J} = \frac {l _ {\alpha} N _ {\alpha}}{J}\tag{18a}
$$

$$
\bar {\mu} _ {c} = \frac {l _ {c} T}{J}\tag{18b}
$$

$$
\bar {\mu} _ {n} = \frac {m _ {n} l _ {n} l _ {c} + J _ {n}}{J}\tag{18c}
$$

and second, relative to the translational motion,

$$
\bar {n} _ {\alpha} = \frac {Q S C _ {N _ {\alpha}}}{m} = \frac {N _ {\alpha}}{m}\tag{19a}
$$

$$
\bar {n} _ {c} = \frac {T}{m}\tag{19b}
$$

$$
\bar {n} _ {n} = \frac {m _ {n} l _ {n}}{m}\tag{19c}
$$

Then, considering the state vector $\pmb { x } = \left[ z , \dot { z } , \theta , \dot { \theta } \right] ^ { \top } = \left[ z , w , \theta , q \right] ^ { \top }$ , the full nonlinear system dynamics results in

$$
\Sigma_ {n o n l i n}: \left\{ \begin{array}{l l} \dot {z} & = w, \\ \ddot {z} & = \dot {w} = \bar {n} _ {\alpha} \alpha - \bar {n} _ {c} \sin \beta - \bar {n} _ {n} \ddot {\beta} - g \sin \theta , \\ \dot {\theta} & = q, \\ \ddot {\theta} & = \dot {q} = \bar {\mu} _ {\alpha} \alpha - \bar {\mu} _ {c} \sin \beta - \bar {\mu} _ {n} \ddot {\beta} \end{array} \right.\tag{20}
$$

## 3.4. Synthesis and Linear Models

For control design purposes, it is convenient to express the dynamics using an approximated synthesis model that fully captures the driving dynamics of Equations (11a) and (11b). To do so, the coefficients in Equations (18) and (19) are extracted from a reference (nominal)

trajectory and are now denoted without the bars. Using the small angles approximation, we can consider the following dynamics as the synthesis model:

$$
\Sigma_ {s y n}: \left\{ \begin{array}{l l} \dot {z} & = w \\ \ddot {z} & = \dot {w} = n _ {\alpha} \alpha - n _ {c} \beta - n _ {n} \ddot {\beta} - g \sin \theta_ {0} \\ \dot {\theta} & = q \\ \ddot {\theta} & = \dot {q} = \mu_ {\alpha} \alpha - \mu_ {c} \beta - \mu_ {n} \ddot {\beta} \end{array} \right.\tag{21}
$$

Considering the output vector $\pmb { y } = \left[ \ w , \ \theta \ \right] ^ { \top } = \left[ \ w ( \mathbf { s } ) , \ \theta ( \mathbf { s } ) \ \right] ^ { \top }$ , we obtain the following linear system dynamics in the Laplace domain:

$$
\Sigma_ {l i n}: \left\{ \begin{array}{l} s   w (s) - n _ {\alpha} \alpha (s) + g   \sin \theta_ {0} = - (n _ {n} s ^ {2} + n _ {c}) \beta (s) \\ s ^ {2} \theta (s) - \mu_ {\alpha} \alpha (s) = - (\mu_ {n} s ^ {2} + \mu_ {c}) \beta (s) \end{array} \right.\tag{22}
$$

and using the linearised angle of attack,

$$
\alpha = \theta + \frac {w - l _ {\alpha} q - v _ {\mathrm{w}}}{V} \quad \Rightarrow \quad \alpha (\mathrm{s}) = \left(1 - l _ {\alpha} \frac {1}{V} \mathrm{s}\right) \theta (\mathrm{s}) + \frac {1}{V} w (\mathrm{s}) - \frac {1}{V} v _ {\mathrm{w}} (\mathrm{s})\tag{23}
$$

together with the model in Equation (22), we obtain the transfer functions $\beta ( \mathbf { s } ) \to \theta ( \mathbf { s } )$ and $\beta ( \mathsf { s } ) \to w ( \mathsf { s } )$ , which correspond to the solutions of the system (without the wind input $v _ { \mathrm { w } } ( \mathbf { s } ) )$ :

$$
\left[ \begin{array}{c c} \mathrm{s} ^ {2} + l _ {\alpha} \frac {\mu_ {\alpha}}{V} \mathrm{s} - \mu_ {\alpha} & - \frac {\mu_ {\alpha}}{V} \\ - l _ {\alpha} \frac {n _ {\alpha}}{V} \mathrm{s} + n _ {\alpha} + g \sin \theta_ {0} & \mathrm{s} + \frac {n _ {\alpha}}{V} \end{array} \right] \left[ \begin{array}{c} \frac {\theta (\mathrm{s})}{\beta (\mathrm{s})} \\ \frac {w (\mathrm{s})}{\beta (\mathrm{s})} \end{array} \right] = - \left[ \begin{array}{c} \mu_ {\mathrm{n}} \mathrm{s} ^ {2} + \mu_ {\mathrm{c}} \\ n _ {\mathrm{n}} \mathrm{s} ^ {2} + n _ {\mathrm{c}} \end{array} \right]\tag{24}
$$

Furthermore, as a first approximation for attitude control design purposes, drift and TWD dynamics can be neglected, and the transfer function $\beta ( \mathbf { s } ) \to \theta ( \mathbf { s } )$ simplifies into

$$
\frac {\theta (\mathbf {s})}{\beta (\mathbf {s})} = - \frac {\mu_ {\mathrm{c}}}{\mathbf {s} ^ {2} + l _ {\alpha} \frac {\mu_ {\alpha}}{V} \mathbf {s} - \mu_ {\alpha}}\tag{25}
$$

## 4. Attitude Control Design Using Angular Acceleration Feedback

This section describes and justifies the four attitude control systems developed in this study.

## 4.1. Scheduled PD Controller

The baseline controller for this study is a classic proportional-derivative (PD) controller designed for the linear system approximation in Equation (25); the PD controller is therefore considered with the following structure:

$$
\beta (\mathbf {s}) = k _ {P} \left(\theta_ {\mathrm{cmd}} (\mathbf {s}) - \theta (\mathbf {s})\right) - k _ {D} q (\mathbf {s}) = k _ {P} \theta_ {\mathrm{cmd}} (\mathbf {s}) - \left(k _ {P} + \mathbf {s} k _ {D}\right) \theta (\mathbf {s})\tag{26}
$$

Despite their simplicity, PD controllers represent the industrial state of practice for the vast majority of launch vehicles [1]. The gains k and $k _ { D }$ can be tuned using a multitude of methods. Here, they are selected based on pole placement of the closed-loop transfer function, which is obtained by substituting Equation (26) in (25):

$$
\frac {\theta (s)}{\theta_ {c m d} (s)} = - \frac {\mu_ {c} k _ {P}}{s ^ {2} + \left(l _ {\alpha} \frac {\mu_ {\alpha}}{V} - \mu_ {c} k _ {D}\right) s - \left(\mu_ {\alpha} + \mu_ {c} k _ {P}\right)}\tag{27}
$$

It is clear from this equation that $k _ { P }$ and $k _ { D }$ can be chosen to enforce the desired natural frequency $\omega _ { \theta }$ and damping ratio $\zeta$ (here assumed constant throughout the flight for simplicity without loss of generality). It is also clear that this approach does not allow us to specify the steady-state gain (when $s \to 0 )$ independently of the natural frequency as they both depend on $k _ { P }$ only.

In order to handle the wide variation of the model’s parameters during the flight, the two gains need to be scheduled throughout the trajectory. To do so, they are pre-computed for a grid of $N = 9$ points (spaced every 10 seconds along the trajectory) as

$$
k _ {P} [ i ] = - \frac {1}{\mu_ {\mathrm{c}} [ i ]} \Big (\mu_ {\alpha} [ i ] + \omega_ {\theta} ^ {2} \Big), \qquad k _ {D} [ i ] = \frac {1}{\mu_ {\mathrm{c}} [ i ]} \Big (l _ {\alpha} [ i ] \frac {\mu_ {\alpha} [ i ]}{V [ i ]} - 2 \zeta \omega_ {\theta} \Big), \qquad i = 1, \ldots , N\tag{28}
$$

and then linearly interpolated online during the simulation. The robustness of this approach can be increased by scheduling the controller with respect to online measurements/estimates of some of the model’s parameters. This is the underlying idea of LPV control [41], which is outside the scope of this paper.

## 4.2. INDI Controller

In this section, an INDI-based control law is developed and applied to regulate the launcher’s attitude channel in Equation (21); therefore, we consider only the state $\pmb { x } _ { c } = \left[ \theta , \dot { \theta } \right] ^ { \top } = \left[ \theta , q \right] ^ { \top }$ , and ignoring the TWD effects, the dynamics are

$$
\dot {\boldsymbol {x}} _ {c} = \boldsymbol {f} _ {c} (\boldsymbol {x} _ {c}) + \boldsymbol {g} _ {c} (\boldsymbol {x} _ {c}) u = \left[ \begin{array}{c} q \\ \mu_ {\alpha} \alpha \end{array} \right] - \left[ \begin{array}{c} 0 \\ \mu_ {c} \end{array} \right] \beta\tag{29}
$$

where $f _ { c } ( { \boldsymbol { x } } _ { c } )$ is the control-independent part of the model, ${ { g } _ { c } } ( { { x } _ { c } } )$ expresses the influence of the controls in the system, and $u \ : = \ : \beta$ is the control input, i.e., the TVC deflection. Considering the (scalar) output as $y = h ( x ) = \theta$ , in order to apply the INDI technique, it has to be time-differentiated until an explicit dependency on the TVC deflection input appears:

$$
\ddot {y} = \ddot {\theta} = \mu_ {\alpha} \alpha - \mu_ {c} \beta\tag{30}
$$

Then, while denoting ν as a virtual control input, the NDI control law for $\beta$ is obtained as 1

$$
\beta_ {\mathrm{NDI}} = - \frac {1}{\mu_ {c}} \Big (\nu - \mu_ {\alpha} \alpha \Big)\tag{31}
$$

Since $\mu _ { c }$ is different from zero, the linear input–output relationship between $\ddot { \theta }$ and ν is obtained as a double integrator:

$$
\nu = \dot {q} = \ddot {\theta} \Rightarrow \frac {\theta (\mathrm{s})}{\nu (\mathrm{s})} = \frac {1}{\mathrm{s} ^ {2}}\tag{32}
$$

Because of the high level of uncertainty affecting $\mu _ { \alpha }$ and $\alpha ,$ it is convenient to employ the more robust INDI approach; for that, following the procedure of Section 2.2, the INDI control law results in a command signal sent to the TVC actuator given by

$$
\beta = \beta_ {0} - \frac {1}{\mu_ {\mathrm{c}}} (\nu - \dot {q} _ {0})\tag{33}
$$

where $\beta _ { 0 }$ and $\dot { q } _ { 0 }$ are measurements/estimates of the TVC command and angular acceleration at the current computation step, respectively. Because angular acceleration sensors are not common in launchers today, $\dot { q } _ { 0 }$ is estimated by passing the angular rate $q$ through a derivative filter of the form:

$$
H _ {\dot {q}} (\mathbf {s}) = \frac {\mathbf {s} \omega_ {\dot {q}}}{\mathbf {s} + \omega_ {\dot {q}}}\tag{34}
$$

where $\omega _ { \dot { q } }$ represents the filter bandwidth.

Note that after the feedback linearisation of Equation (33), there are still some degrees of internal dynamics in the system related to the drift motion and TWD effect. However, the TWD dynamics are known to be stable, and the drift motion develops slowly and can be further handled by outer control loops $( \mathrm { e . g . }$ , see Section 7). Denoting $e = \theta _ { \mathrm { c m d } } - \theta _ { \mathrm { c m d } }$ , where $\theta _ { \mathrm { c m d } }$ represents the (nonsmooth) commanded pitch angle (in this case, the only reference for the attitude motion), the double integrator can be therefore rendered exponentially stable with

$$
\nu = k _ {D} \dot {e} + k _ {P} e\tag{35}
$$

where $k _ { D }$ and $k _ { P }$ are constants chosen so that the polynomial $s ^ { 2 } + k _ { D } s + k _ { P }$ is Hurwitz. This results in the exponentially stable and decoupled error dynamics $\ddot { e } + k _ { D } \dot { e } + k _ { P } e = 0 _ { . }$ which implies that $e ( t ) \to 0 { \mathrm { a s } } t \to \infty$ . Using the virtual control and the linearised system of Equation (32), an outer PD control law is able to enforce the desired closed-loop response as follows:

$$
\nu (s) = k _ {P} \Big (\theta_ {\mathrm{cmd}} (s) - \theta (s) \Big) - k _ {D} q (s) \Rightarrow \frac {\theta (s)}{\theta_ {\mathrm{cmd}} (s)} = \frac {k _ {P}}{s ^ {2} + k _ {D} s + k _ {P}}\tag{36}
$$

with the control gains:

$$
k _ {P} = \omega_ {\theta} ^ {2}, \qquad k _ {D} = 2 \zeta \omega_ {\theta}\tag{37}
$$

Note that in contrast with the PD controller of Section 4.1, $k _ { P }$ and $k _ { D }$ do not need to be scheduled as they depend on $\omega _ { \theta }$ and $\zeta$ only, but a pre-computed grid of $\mu _ { \mathrm { c } } [ i ]$ is still required to perform the feedback linearisation. This is highlighted in the blue area of Figure 3a, which illustrates the implementation of the INDI controller in the simulator. Alternatively, $\mu _ { \mathrm { c } }$ could be estimated based on online measurements. As also shown in Figure $^ { 3 \mathbf { a } , }$ the feedback linearisation loop includes a unit delay to ensure physical causality of the signals.

![](images/becbcb58c1da7876af77ebfbe17cd2d3335bba37f6006f573f22ab8ec1996088.jpg)
Figure 3. Cont.
(a)

![](images/fc32e4cd7446cf83f0cce01c2465ec0f449571c55eb84fb7fa78ba728b3e40a8.jpg)
Figure 3. INDI controllers’ implementation diagrams. (a) Pure INDI (from Section 4.2). (b) INDI with low-pass filter (from Section 4.4).

## 4.3. Scheduled PD Controller with q Feedback˙

As explained in Section 2.2, the INDI controller of Equation (33) relies on q˙ information to reduce the impact of the launcher’s model on the achievable control performance. For a fair comparison of controllers, it is then pertinent to consider a linear controller where q˙ feedback is also employed. In this case, the control law takes the form

$$
\beta (\mathsf {s}) = k _ {P} \left(\theta_ {\mathrm{cmd}} (\mathsf {s}) - \theta (\mathsf {s})\right) - k _ {D} q (\mathsf {s}) - k _ {A} \dot {q} (\mathsf {s})\tag{38}
$$

where $k _ { A }$ is the acceleration feedback gain. Similar to Section 4.1, the three gains can be tuned via pole placement of the closed-loop transfer function, which is obtained by substituting Equation (38) in (25):

$$
\frac {\theta (\mathrm{s})}{\theta_ {\mathrm{cmd}} (\mathrm{s})} = - \frac {k _ {P}}{1 - \mu_ {\mathrm{c}} k _ {A}} \frac {\mu_ {\mathrm{c}}}{\mathrm{s} ^ {2} + \frac {l _ {\alpha} \frac {\mu_ {\alpha}}{V} - \mu_ {\mathrm{c}} k _ {D}}{1 - \mu_ {\mathrm{c}} k _ {A}} \mathrm{s} - \frac {\mu_ {\alpha} + \mu_ {\mathrm{c}} k _ {P}}{1 - \mu_ {\mathrm{c}} k _ {A}}}\tag{39}
$$

In contrast with the pure PD controller, the $\dot { q }$ feedback allows us to minimise tracking errors because the desired steady-state gain $G _ { 0 }$ is specified independently of $\omega _ { \theta }$ through the proportional gain as follows:

$$
k _ {P} [ i ] = \frac {\mu_ {\alpha} [ i ]}{\mu_ {\mathrm{c}} [ i ]} \frac {G _ {0}}{1 - G _ {0}}, \qquad i = 1, \ldots , N\tag{40}
$$

which is scheduled along a grid of $N = 9$ points along the launcher’s trajectory. The other two gains are then derived as a function of $\omega _ { \theta }$ and $\zeta$ as

$$
k _ {A} [ i ] = \frac {1}{\mu_ {\mathrm{c}} [ i ]} \Big (1 + \frac {\mu_ {\alpha} [ i ] + \mu_ {\mathrm{c}} [ i ] k _ {P} [ i ]}{\omega_ {\theta} ^ {2}} \Big), \quad k _ {D} [ i ] = \frac {1}{\mu_ {\mathrm{c}} [ i ]} \Big (l _ {\alpha} [ i ] \frac {\mu_ {\alpha} [ i ]}{V [ i ]} - 2 \zeta \omega_ {\theta} (1 - \mu_ {\mathrm{c}} [ i ] k _ {A} [ i ]) \Big)\tag{41}
$$

For the estimation of $\dot { q }$ in Equation (38), the same approach of Section 4.2, i.e., passing the angular rate q through the first-order derivative filter of Equation (34), was followed. In practice, it was verified that the performance of this controller is fairly sensitive to the filter bandwidth $\omega _ { \dot { q } } .$ . This impact is illustrated in Figure 4, which shows root-mean-square (RMS) values of pitch error $( \theta _ { \mathrm { { e r r } } } = \theta _ { \mathrm { { c m d } } } - \theta )$ vs. TVC rate (β<sup>˙</sup>) for a step command in $\theta _ { \mathrm { c m d } }$ using different controllers and nominal conditions.

![](images/f4969a93fa28632f36852341e1b6c0327e223a4f472bf87180f72aac7812ccc8.jpg)
Figure 4. Tuning trade-off of angular acceleration feedback approaches.

The blue line in Figure 4 shows results using the scheduled PD controller with q˙ feedback (FB) and varying values of the derivative filter bandwidth $\omega _ { \dot { q } } .$ . Based on the results, the selection of $\omega _ { \dot { q } }$ provides a key (and intuitive) tuning trade-off: increasing the bandwidth leads to smaller errors at the expense of more demanding TVC actuation, and vice versa. A more favourable trade-off would likely be achieved by using a higher-orderderivative filter, which is outside the scope of this paper.

## 4.4. INDI Controller with Low-Pass Filter

When applied to the pure INDI controller developed in Section 4.2, the same tuning trade-off analysis showed a much smaller sensitivity to $\omega _ { \dot { q } }$ but unacceptably high TVC rates. To address this issue, the INDI controller was augmented with a low-pass filter, as depicted in the right-hand side of Figure 3b. In this case, by taking the $\beta _ { 0 }$ signal after the low-pass filter for the feedback linearisation, the unit delay used in Figure 3a is no longer necessary.

The outer linear gains and q˙ estimation filter remain unchanged. The low-pass filter has bandwidth $\omega _ { \beta }$ and a first-order structure as follows:

$$
H _ {\beta} (\mathbf {s}) = \frac {\omega_ {\beta}}{\mathbf {s} + \omega_ {\beta}}\tag{42}
$$

The purple line in Figure 4 shows the tuning trade-off using the INDI controller with low-pass filter and varying values of its bandwidth $\omega _ { \beta }$ . Compared with the PD controller with q˙ feedback (blue line), the two controllers show a similar trend (i.e., smaller errors and larger TVC rates for higher bandwidths), yet the INDI controller leads to smaller TVC rates for the same level of error. As before, a more favourable trade-off would likely be achieved by using a higher-order low-pass filter, but this is outside the scope of this paper.

## 4.5. Control Design Summary

The four controllers in Sections 4.1–4.4 were designed to enforce the same closed-loop properties throughout the flight. These are the characteristics:

• Natural frequency $\omega _ { \theta } = 2 . 5$ rad/s;

• Damping ratio $\zeta = 0 . 8 ;$

Steady-state error of 5%, i.e., $G _ { 0 } = 1 . 0 5 ,$ , only applicable to Section 4.3.

Furthermore, the bandwidth of the filters in Sections 4.3 and $4 . 4 , \omega _ { \dot { q } }$ and $\omega _ { \beta } ,$ was tuned to provide the same pitch error in nominal conditions, as highlighted in Figure 4. The robust performance of these controllers is analysed in Section 5.

Table 2 provides an overview of each controller’s dependency on the model parameters and sensor measurements/estimates. As anticipated, from the scheduled PD controller to the INDI controller, there is a progressive reduction in model dependency and an increased use of sensor information. More specifically, the INDI controller relies on measurements/estimates of $\dot { q }$ and $\beta$ to fully circumvent the knowledge of the aerodynamics model.

Table 2. Dependencies per control design method.

<table><tr><td>Control Design Method</td><td>Dependency on Model Parameters</td><td>Dependency on Measurements/Estimates</td></tr><tr><td>Scheduled PD</td><td> $J, l_{c}, T, C_{N_{\alpha}}, l_{\alpha}, \rho, V$ </td><td> $\theta, q$ </td></tr><tr><td>Scheduled PD with  $\dot{q}$  feedback</td><td> $J, l_{c}, T, C_{N_{\alpha}}, l_{\alpha}, \rho, V$ </td><td> $\theta, q, \dot{q}$ </td></tr><tr><td>INDI with or without low-pass filter</td><td> $J, l_{c}, T$ </td><td> $\theta, q, \dot{q}, \beta$ </td></tr></table>

## 5. Time-Domain Robust Performance Analysis

This section analyses and compares the nonlinear time-domain performance the controllers developed in Section 4. Figure 5 shows dispersed responses of the $2 ^ { 8 } = 2 5 6$ corner cases within the uncertainty level of Table 1 when subjected to the same wind turbulence input $v _ { \mathrm { w } } ,$ modelled as described in Section 3. From the top to the bottom rows, the figure depicts the obtained pitch error $\theta _ { \mathrm { e r r } } ,$ TVC deflection $\beta ,$ and aerodynamic load indicator Qα along the trajectory. From left to right, the figure depicts results using the scheduled PD controller (Figure 5a, in black), scheduled PD controller with q˙ feedback (Figure 5b, in blue), and INDI controller with low-pass filter (Figure 5c, in purple). The pure INDI controller (without low-pass filter) is not shown as it leads to unacceptably high TVC rates.

From Figure 5a,b, a reduction in the dispersion of all the indicators can be observed. This joint reduction clearly demonstrates the benefit of including q˙ feedback in the control design. The pitch error (and partially the Qα) is further reduced when using the INDI controller with low-pass filter, as depicted in Figure 5c, at the expense of higher TVC deflections (although still comparable to the pure PD controller). Note that Qα minimisation was not a specific control design objective in this case but comes as a direct consequence of smaller pitch and drift errors, as indicated in Equation (13b).

In order to visualise these trends more clearly, Figure 6a shows the wind response results using the same RMS $\theta _ { \mathrm { e r r } }$ vs. $\dot { \beta }$ plot of Figure 4. Each point in Figure 6a corresponds to a single simulation from Figure 5. As anticipated, the pure PD controller (in black) provides the largest errors but the smallest TVC rates, while on the other hand, the pure INDI controller (in red) provides the smallest errors but the largest TVC rates. The PD controller with $\dot { q }$ feedback (in blue) and the INDI controller with low-pass filter (in purple) lie in between the two extremes, with the latter controller performing better than the former (i.e., with slightly smaller errors and TVC rates) but only marginally.

![](images/b5d681edafadf85898baa7e30b69f9020b2a12691496f33ac961695a8e3fd002.jpg)

![](images/93d60a55ae8a73bbb02fba6a1a938fa12260e725853090676d19305542e6b7a6.jpg)

![](images/762e99c7d736b6d94fbc300bc523e1b252f1c30e95b6a3f5db94a341a18ecb5a.jpg)

![](images/9af072f699246574be9cf0d78a3f7b3fe719a907122c0ffd04391d4e5107298f.jpg)

![](images/8d879fb2dba4720844630a2b0912f14f0becc5d3286cb5c716edb97907e9c087.jpg)

![](images/58bc24673c3b1a7115720b0f07b178994dc494954d52d43832a63ff21a9892b6.jpg)

![](images/4c261bb9a66ee2cb3a042258a0b572af9cc1da7ac81dae7481d93e6e2b6e2b1b.jpg)
(a) Scheduled PD controller.

![](images/867f1e8131f79daa70142b23e53a0469f61bfe442348e2631da716faa712e26e.jpg)

![](images/a8b01905508522417ea49c4ca940e3f0055f3e416150036b0ca829c9321b6248.jpg)
(b) Scheduled PD controller with q˙ FB. (c) INDI controller with low-pass filter.
Figure 5. Comparison of Monte Carlo wind responses using different controllers.

In order to complement the analysis, Figure 6b shows the same type of results for a step command in $\theta _ { \mathrm { c m d } }$ . As before, the pure PD controller (in black) leads by far to the largest errors and the pure INDI controller (in red) to the largest TVC rates. Performance in terms of error and TVC rate improves using either the PD controller with q˙ feedback or the INDI controller with low-pass filter, and the difference between these two controllers is now more evident than for the wind responses.

In nominal conditions, it is known from Figure 4 that for the same error, the INDI controller with low-pass filter (in purple) provides a smaller TVC rate than the PD controller with q˙ feedback (in blue). Nonetheless, Figure 6a shows that the former controller performs better also in terms of error, having a range of dispersion that is approximately four times smaller. The smaller error dispersion of the INDI controller with low-pass filter comes at the expense of a larger TVC rate dispersion, but its maximum value remains significantly lower than that of the PD controller with q˙ feedback.

![](images/8fee002bfd2b630ebb540024b3b685ee1fe8c7f1cce10198e2db80431055f0da.jpg)
(a) Wind responses.

![](images/d45c915e2bf0addd5cb49c292b6d7a5c6e76ef8562aefca163ecb0a07d6722f8.jpg)
(b) Step responses.
Figure 6. Overview of Monte Carlo results using different controllers.

INDI-based controllers, by relying on angular acceleration and control input measurements/estimates, are known to be more sensitive to sensor noise and actuator delays than classical linear controllers. In order to assess this sensitivity, Figure 7 extends Figure 6a using the INDI controller with low-pass filter, showing wind simulation results with different combinations of the following:

Gaussian noise on the angular rate signal, with $3 \sigma = \{ 0 , 0 . 0 5 , 0 . 1 \}$ deg/s, which affects the estimates of both q and q˙ through Equation (34). Note that the 3σ value of 0.1 deg/s is considered to be very pessimistic [42].

Time delay of { 0, 40, 80 } ms on the signal commanded to the TVC actuator, corresponding to a delay of { 0, 1, 2 } control samples.

![](images/938e113cbe95d267ea2d853e140fbb5e51db19caf740564c5c92284068ef3a71.jpg)
Figure 7. Impact of sensor noise and actuator delays on INDI controller (with low-pass filter).

From Figure 7, it can be observed that for the ranges considered, delays on the TVC signal have very little impact on the controller’s performance. Noise on the angular rate signal, on the other hand, leads to a more noticeable degradation, with the resulting TVC rates increasing approximately linearly with the noise variance.

This type of understanding is critical when designing and sizing INDI-based GNC software and hardware. The authors consider the scenario shown in the middle of Figure 7 (noise $3 \sigma = 0 . 0 5 \ : \mathrm { d e g / s , T V C }$ rate below $1 0 \deg / s )$ to be very representative of a potential real-world application. The impact of angular rate noise would be minimised by using a higher-order-derivative filter $H _ { \dot { q } } ( \mathbf { s } )$ or by including an angular acceleration sensor in the GNC system.

## 6. Frequency-Domain Robust Stability Analysis

Because of the nonlinear nature of INDI, attaining analytical proof of stability of INDIbased controllers [36] is much less trivial than for classical linear controllers. In order to mitigate this shortcoming, this section introduces a simple yet insightful frequency-domain approach to quantify stability degradation related to an imperfect feedback linearisation and to deviations from the control tuning conditions. This section is therefore focused on the controller developed in Section 4.4, not on a full comparison of controllers.

The proposed approach is based on linearised models of the nonlinear launcher simulator with the INDI control law in the loop at different flight conditions and on the fact that for a perfect feedback linearisation, the channel $\nu ( s ) \to \theta ( s )$ behaves as a double integrator (recall Equation (32)). The INDI controller design was carried out under this assumption.

This approach was first presented in [37], but there was an error in the computation of the linearised models in that reference, which led to invalid stability analysis results. This section serves, therefore, as a correction of the results presented in [37].

Linearised models of $\nu ( \mathbf { s } ) \to \theta ( \mathbf { s } )$ can be computed thanks to the following $\mathbf { M A T L A B } ^ { \textregistered }$ (R2024b) routine

$$
\text { linearize } (\mathrm{mdl}, \text { findop } (\mathrm{mdl}, t), \dots)
$$

where mdl is the Simulink<sup>®</sup> file instantiated with a certain configuration and t is the flight time instant. The analysis in this section considers the $2 ^ { 8 } = 2 5 6$ corner cases (within the uncertainty level of Table 1) and 33 instants (spaced every 2.5 s along the trajectory).

Figure 8a shows the frequency response of the aforementioned linearised models (in blue) together with the “perfect” double-integrator assumption (in red). This figure shows two important features:

A dispersion of the linearised models, which is caused by deviations from the control tuning conditions due to the uncertain and time-varying nature of the model’s parameters;

A mismatch between the linearised models and the double-integrator assumption, which grows in frequency ranges dominated by dynamical effects that are neglected in the feedback linearisation, i.e., drift motion at low frequencies (purple area) and TVC/TWD dynamics and $H _ { \dot { q } } ( \mathbf { s } )$ filter at high frequencies (blue area).

These models can be employed to assess the system’s stability margins when the loop is closed using Equation (36). To do so, it is convenient to plot the responses in a Nichols chart, which is depicted in Figure 8b for the nominal and corner-case samples. For a detailed explanation of the application of Nichols charts to launcher stability assessment, the reader is referred to [39].

![](images/ed55b84635a64ed660205a65d1918b8c4b9ca8e063e96234de79f3b7e9ca073a.jpg)
(a) Bode magnitude plot $\nu ( \mathbf { s } ) \to \theta ( \mathbf { s } )$

![](images/d97c580c335bdc4852b5b86a9b210fd9bf4ac6c8b8f87965fba1c1bfc4bac0e7.jpg)
(b) Nichols chart $\theta _ { \mathrm { e r r } } ( s ) \to \theta ( s )$
Figure 8. Frequency responses of linearised INDI-controlled plants for stability analysis.

The impact of the imperfect feedback linearisation on the system’s stability becomes evident in Figure 8b: the phase margin is reduced approximately by half and the system can be gain-destabilised, which is not the case under the double-integrator assumption. Nonetheless, all phase and gain margins remain substantial. When this is not the case, the linearised models of $\nu ( s ) \to \theta ( s )$ can be employed instead of Equation (32) to re-tune the INDI outer control law. The stability margins are naturally driven by the value of $\mu _ { \mathrm { c } } ,$ which is the main dependency of the INDI controller (recall Table 2). Accordingly, the margins become smaller for smaller values of $\mu _ { \mathrm { c } }$ as the system’s control effectiveness decreases, and vice versa.

In order to assess the degradation caused by uncertainties and time variations, the gain and phase margins are plotted as a function of time in Figure 9a and b, respectively. Figure 9a contains the margins related to a reduction in gain (which impacts lower frequency dynamics, often referred to as LF margin) in blue and to an increase in gain (which impacts higher-frequency dynamics, referred to as HF margin) in purple. The fig ures show the nominal margins in a continuous line, and the worst (minimum) corner-case margins (assuming the uncertainty level of Table 1) in a dashed line. The $N = 9$ control tuning points, i.e., the interpolation nodes of $\mu _ { \mathrm { c } } ,$ are indicated in the figures using circular marks. The main results are then summarised in Table 3.

From Figure 9, it can be seen that the stability margins—especially the LF gain margin—show a noticeable variation over the flight, which follows the evolution of the launcher’s dynamic pressure. This variation is caused by the impact of drift motion at low frequencies (recall Figure 8a), which is neglected in the feedback linearisation and which is directly proportional to the dynamic pressure. The HF gain margin is significantly less sensitive to variations, as already anticipated from Figure 8b.

Between control tuning points, there is also a variation in margins due to mismatches between actual and interpolated values of $\mu _ { \mathrm { c } }$ . Nonetheless, this variation is extremely limited and leads to a degradation of around 0.1 deg and 0.1 dB, as can been seen from Table 3.

Stability degradation due to uncertainties is about two orders of magnitude higher, leading to margin losses (i.e., differences between nominal and worst-case values from Table 3) of 12.7 deg and 10.9 dB (at LF). In practice, the resulting stability margins must provide enough room to accommodate the impact of dynamical effects that were not considered in this study, such as flexible modes and noncollocated sensing. Nonetheless, the worst-case margins are plentiful, which suggests the feasibility of INDI-based launcher attitude control.

![](images/d4173444b718d362d3b2f474850dbb8983afa04e7631a77ed0dff3cce18508dc.jpg)
(a) Gain margins at low and high freq. (LF and HF) vs. time.

![](images/c134b4bdbc25d225195ff8c01b234d93000ccce9a1fff39136148070708bb34e.jpg)
(b) Phase margin vs. time.
Figure 9. Nominal and worst-case margins of INDI controller (with low-pass filter).

Table 3. Stability margin budget (gain margins are indicated in absolute value).

<table><tr><td>Case</td><td>LF Gain Margin (dB)</td><td>Phase Margin (deg)</td><td>HF Gain Margin (dB)</td></tr><tr><td>Double-integrator assumption</td><td>∞</td><td>69.84</td><td>∞</td></tr><tr><td>Nominal</td><td>22.62</td><td>34.66</td><td>19.21</td></tr><tr><td>Nominal with deviation from  $\mu_c$  interp. points</td><td>22.50</td><td>34.55</td><td>19.13</td></tr><tr><td>Worst-case</td><td>11.76</td><td>21.94</td><td>18.32</td></tr><tr><td>Worst-case with deviation from  $\mu_c$  interp. points</td><td>11.64</td><td>21.86</td><td>18.18</td></tr></table>

## 7. Extension to Launcher Control with Active Load Relief

The controllers analysed in Sections 5 and 6 control only the attitude of the launcher, meaning that the lateral drift dynamics (Equation (11a)) can develop in an unbounded manner. One of the simplest approaches to control the drift motion is to augment the current design with an outer loop employing drift rate (z˙) feedback. This section aims to showcase how such an approach can be effectively applied in conjunction with the INDI controller of Section 4.4.

The idea behind the proposed control law is to adjust the commanded pitch angle in the presence of drift motion so that when $\theta  \theta _ { \mathrm { c m d } }$ (which is ensured by the faster inner attitude controller), the induced angle of attack $\alpha _ { 0 }$ is neutralised, i.e., $\alpha _ { 0 }  0$ . This leads then to an active minimisation of the induced aerodynamic load, quantified by Qα.

The induced angle of attack is given by Equation (13b) without $v _ { \mathrm { w } }$ since the wind speed is typically unknown (unless a wind estimator is included in the design [40]) and without the aerodynamic damping term due to the very high uncertainty in $l _ { \alpha } / V$ . This corresponds to

$$
\alpha_ {0} = \theta + \arctan {\frac {w}{V}} \approx \theta + \frac {w}{V}\tag{43}
$$

where $w = \dot { z }$ is the drift rate. The pitch command is then scaled by a “load relief” gain $k _ { L R }$ and by the normalised dynamic pressure $Q _ { \mathrm { n o r m } } = Q / \operatorname* { m a x } ( Q )$ , yielding the control law

$$
\theta_ {\mathrm{cmd}} (\mathbf {s}) = - k _ {L R} \frac {Q _ {\mathrm{norm}}}{V} \dot {z} (\mathbf {s})\tag{44}
$$

This pitch command is then passed to the inner loop of Equation (36). The scaling by Q<sub>norm</sub> guarantees that the load relief is tighter when the dynamic pressure is higher $( Q _ { \mathrm { n o r m } }  1 )$ and more relaxed otherwise, allowing a tighter attitude tracking when the dynamic pressure is lower $( Q _ { \mathrm { n o r m } }  0$ during the initial and final phases of the flight). Note that when $Q _ { \mathrm { n o r m } }$ or $k _ { L R }$ are zero, $\theta _ { \mathrm { c m d } } ( \mathbf { s } ) = 0$ and the outer loop is opened.

The implementation of Equation (44) is illustrated in Figure 10. A pre-computed grid of $Q _ { \mathrm { n o r m } } [ i ]$ and $V [ i ]$ is employed to handle the variation of the model’s parameters throughout the flight (though $Q _ { \mathrm { n o r m } }$ and V could alternatively be estimated from online measurements, similar to $\mu _ { \mathrm { c } }$ in Figure 3).

![](images/301d4f32d64fa5cafe9cb8cd8bbfe36ce7f887a30370e9793dff2aeb73cb769c.jpg)
Figure 10. Outer-loop controller implementation diagram.

A stepwise performance analysis of the proposed controller is provided in Figure 11. Each individual plot shows RMS values of different indicators (each point corresponding to a single simulation, similar to Figure 6) when using a different load relief gain $k _ { L R } ,$ , which is represented using different colours.

Figure 11a depicts dispersed results of $2 ^ { 7 } = 1 2 8$ corner cases per gain when subjected to the same wind input signal (these are the same conditions used in Section 5 except that the launcher’s mass is not dispersed now as its impact on the simulations is practically negligible). In Figure 11b, the launcher’s parameters are kept to their nominal values, but 128 scattered wind signals are tested per gain (this is important as wind disturbances have a major impact on the induced load). Then, Figure 11c shows the results using both uncertain parameters and scattered winds (amounting to $1 2 8 \times 1 2 8 = 1 6 3 8 4$ simulations per gain). From top to bottom, the rows of Figure 11 depict, respectively, $\theta _ { \mathrm { e r r } } ~ \mathrm { v s . } ~ \dot { z } , \theta _ { \mathrm { e r r } } ~ \mathrm { v s . } ~ Q \alpha .$ , and $\theta _ { \mathrm { e r r } }$ vs. ${ \bar { \dot { \beta } } } .$

Starting with the first plot of Figure 11a, as $k _ { L R }$ increases (from blue to purple), there is a significant reduction in drift rate at the expense of much higher pitch errors (due to the modified pitch command). The reduction in drift rate is then accompanied by a reduction in Qα (second plot), which is indeed the main aim of the designed outer-loop controller. The bottom plot shows that the impact of the load relief gain on the induced TVC rate is very little for smaller values of $k _ { L R } ,$ but a few outliers are observed with $k _ { L R } = 9 _ { . }$ , which indicates robustness issues for gains around this value. It should be noted that the higher pitch errors obtained in this case (maximum around 0.5 deg) remain comparable with what would be achieved using a traditional PD controller even without any load relief (maximum around 0.4 deg, from Figure 6a).

Figure 11b shows the same trade-off pitch error versus drift rate and $Q \alpha _ { \iota }$ , and it confirms that wind disturbances have a major impact: the plots depict much larger variations under dispersed winds than under uncertain parameters (Figure 11a). The only exception is the TVC rate, which shows very little sensitivity to the wind (bottom plot of Figure 11b).

![](images/f9f04b51391b29343728db762ad0240051862c436ea69b3581b86937f95f1328.jpg)

![](images/e13834b5e10b418be90d86bcc1702228e485ac8d9b6d56259ab7d77f86f0f007.jpg)

![](images/d7711b8af71367732bd7f8ffca5e3bff66a1ac87d52b3917d3b0793340ffd3d9.jpg)

![](images/02d7c6fe402b67bcab1ead513cee851353aa53378f3d38f0dce2693e2f7aa86e.jpg)

![](images/1a6d84d13d808ba4043121a7f87735a35dfcbc4a941c0549f5b8c075ca4d90b9.jpg)

![](images/8dcdef21076bbfb319ff6e21a33a71419dcad6ba113ea4a5abe25c6cbdc942d0.jpg)

![](images/16bc0d18d20d9f12b7581df0db07605958c1abd2a182c574d6d704d665947801.jpg)
(a) Corner cases, same wind.

![](images/5d9b2b18761438e179d245d3c6262cc5311ccecb5a8317079ccc0128cf561ba1.jpg)
(b) Nominal, several winds.

![](images/c801031c5e5563fa86480fdc69f83d56a8272f812ef2a48716b3fff31fb737c1.jpg)
(c) Corner cases, several winds.
Figure 11. Time-domain performance analysis using varying load relief gains.

Finally, Figure 11c illustrates the full dispersions caused by the combination of uncertain parameters and scattered winds. As can be seen, the conclusion concerning the trade-off pitch error versus drift rate and Qα through $k _ { L R }$ remains valid. Specifically in terms of aerodynamic load (second plot), increasing $k _ { L R }$ from 0 to 3 (from blue to yellow) has negligible impact on the minimum value of Qα, but allows to reduce its worst-case value by 14.8% (from 92.3 to 78.6 kPa·deg), which proves the effectiveness of the proposed load relief approach. For $k _ { L R } = 9$ , although the worst-case Q is even lower, the robustness issues anticipated in the bottom plot of Figure 11a become too severe under wind dispersions, as observed in Figure 11c. Hence, the optimal choice of $k _ { L R }$ shall be supported by a formal robustness analysis, which is similar to the one carried out for the inner attitude loop in Section 6 and therefore omitted for the sake of conciseness.

## 8. Conclusions

The results obtained in this study confirm that the Incremental Nonlinear Dynamic Inversion (INDI) technique provides a fast and robust launcher control design approach, which enables a stepwise transition from model-based to data-driven control design in the future. In particular, the results of a comprehensive nonlinear simulation campaign considering wind disturbances and parameter uncertainties indicate that the challenges associated with INDI-based control (e.g., sensitivity to sensor noise and actuator delay, difficulty of obtaining analytical proof of stability) are outweighed by the aforementioned benefits. Moreover, while being less disruptive than INDI, the linear control approach using angular acceleration feedback showed relevant performance improvements, and therefore, it also deserves consideration for a potential industrial application.

The outlook of this work is as follows: (1) furthering the suitability analysis of the INDI-based launcher control, (2) fine-tuning the components of the INDI architecture, and (3) fully exploiting the potential of this approach. For the first point, a thorough assessment of the impact of noncollocated sensing and flexible and sloshing modes needs to be carried out. The second point concerns the design of a better performing (higher-order) derivative filter and the combined tuning of the INDI gains and low-pass filter. As for the third point, the full exploitation of INDI will extend the current single-axis, single-TVC application to the multivariable case of controlling multiple axes (i.e., roll, pitch, and yaw) using multiple TVC actuators (potentially involving multiple engines).

Author Contributions: Conceptualization, P.S., P.A. and S.B.; Methodology, P.S. and P.A.; Formal analysis, P.S. and P.A.; Writing—original draft, P.S. and P.A.; Writing—review & editing, S.B. All authors have read and agreed to the published version of the manuscript.

Funding: This research received no external funding.

Data Availability Statement: The datasets presented in this article are not readily available due to their sensitive nature.

Acknowledgments: The authors would like to thank Massimo Casasco, head of ESA’s Guidance, Navigation & Control Systems Section, for making this feasibility study possible.

Conflicts of Interest: The authors declare no conflicts of interest.

## References

1. Marcos, A.; Navarro-Tapia, D.; Simplício, P.; Bennani, S. Robust Control for Launchers: VEGA study case. J. Soc. Instrum. Control Eng. 2020, 3, 192–202.

2. Slotine, J.J.; Li, W. Applied Nonlinear Control; Prentice Hall Inc.: Hoboken, NJ, USA, 1990.

3. Khalil, H.K. Nonlinear Systems, 3rd ed.; Prentice Hall: Hoboken, NJ, USA, 2002.

4. Enns, D.; Bugajski, D.; Hendrick, R.; Stein, G. Dynamic inversion: An evolving methodology for flight control design. Int. J. Control 1994, 59, 71–91.

5. Reiner, J.; Balas, G.J.; Garrard, W.L. Flight Control Design Using Robust Dynamic Inversion and Time-scale Separation. Automatica 1996, 32, 1493–1504. [CrossRef]

6. Smith, P.R. A Simplified Approach to Nonlinear Dynamic Inversion Based Flight Control. In Proceedings of the AIAA Atmospheric Flight Mechanics Conference, Boston, MA, USA, 10–12 August 1998.

7. Looye, G. Design of Robust Autopilot Control Laws with Nonlinear Dynamic Inversion. at–Automatisierungstechnik 2001, 49.523-531

8. Lombaerts, T.J.; Huisman, H.O.; Chu, Q.P.; Mulder, J.A.; Joosten, D.A. Flight Control Reconfiguration based on Online Physical Model Identification and Nonlinear Dynamic Inversion. In Proceedings of the AIAA Guidance, Navigation, and Control Conference and Exhibit, Honolulu, HI, USA, 18–21 August 2008.

9. Bennani, S.; Looye, G. Flight Control Law Design for a Civil Aircraft using Robust Dynamic Inversion. In Proceedings of the IEEE/SMC CESA’98, Hammamet, Tunisia, 8–12 April 1998.

10. Smith, P.R.; Berry, A. Flight Test Experience of a Nonlinear Dynamic Inversion Control Law on the VAAC Harrier. In Proceedings of the AIAA Atmospheric Flight Mechanics Conference, Toronto, ON, Canada, 2–5 August 2000.

11. Bacon, B.J.; Ostroff, A.J. Reconfigurable Flight Control using Nonlinear Dynamic Inversion with a Special Accelerometer Implementation. In Proceedings of the AIAA Guidance, Navigation, and Control Conference and Exhibit, Providence, RI, USA, 16–19 August 2000.

12. Bacon, B.J.; Ostroff, A.J.; Joshi, S.M. Nonlinear Dynamic Inversion Reconfigurable Controller Utilizing a Fault-Tolerant Accelerometer Approach; Technical Report; NASA Langley Research Center: Hampton, VA, USA, 2000.

13. Bacon, B.J.; Ostroff, A.J.; Joshi, S.M. Reconfigurable NDI Controller using Inertial Sensor Failure Detection & Isolation. IEEE Trans. Aerosp. Electron. Syst. 2001, 37, 1373–1383.

14. Chen, H.B.; Zhang, S.G. Robust Dynamic Inversion Flight Control Law Design. In Proceedings of the ISSCAA 2008, 2nd International Symposium on Systems and Control in Aerospace and Astronautics, Shenzhen, China, 10–12 December 2008.

15. Sieberling, S.; Chu, Q.P.; Mulder, J.A. Robust Flight Control Using Incremental Nonlinear Dynamic Inversion and Angular Acceleration Prediction. J. Guid. Control Dyn. 2010, 33, 1732–1742. [CrossRef]

16. Simplício, P.; Pavel, M.; van Kampen, E.; Chu, Q.P. An Acceleration Measurements-based Approach for Helicopter Nonlinear Flight Control using Incremental Nonlinear Dynamic Inversion. Control Eng. Pract. 2013, 21, 1065–1077. [CrossRef]

17. Lu, P.; van Kampen, E.; Chu, Q.P. Robustness and Tuning of Incremental Backstepping Approach. In Proceedings of the AIAA Guidance, Navigation and Control Conference, Kissimmee, FL, USA, 5–9 January 2015.

18. Lu, P.; van Kampen, E. Active Fault-Tolerant Control System using Incremental Backstepping Approach. In Proceedings of the AIAA Guidance, Navigation and Control Conference, Kissimmee, FL, USA, 5–9 January 2015.

19. Smeur, E.J.; Chu, Q.P.; de Croon, G.C. Adaptive Incremental Nonlinear Dynamic Inversion for Attitude Control of Micro Air Vehicles. J. Guid. Control Dyn. 2016, 39, 450–461.

20. Smeur, E.J.; de Croon, G.C.; Chu, Q.P. Gust Disturbance Alleviation with Incremental Nonlinear Dynamic Inversion. In Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Daejeon, Republic of Korea, 9–14 October 2016.

21. Vlaar, C. Incremental Nonlinear Dynamic Inversion Flight Control. Master’s Thesis, Delft University of Technology, Faculty of Aerospace Engineering, Delft, The Netherlands, 2014.

22. Grondman, F.; Looye, G.; Kuchar, R.O.; Chu, Q.P.; van Kampen, E. Design and Flight Testing of Incremental Nonlinear Dynamic Inversion-based Control Laws for a Passenger Aircraft. In Proceedings of the AIAA Guidance, Navigation and Control Conference, Kissimmee, FL, USA, 8–12 January 2018

23. Keijzer, T.; Looye, G.; Chu, Q.P.; van Kampen, E. Design and Flight Testing of Incremental Backstepping based Control Laws with Angular Accelerometer Feedback. In Proceedings of the AIAA SciTech Forum, San Diego, CA, USA, 7–11 January 2019.

24. Acquatella B.P.; Falkena, W.; van Kampen, E.J.; Chu, Q.P. Robust Nonlinear Spacecraft Attitude Control Using Incremental Nonlinear Dynamic Inversion. In Proceedings of the AIAA Guidance, Navigation, and Control Conference, Minneapolis, MN, USA, 13–16 August 2012.

25. Acquatella B., P.; Chu, Q.P. Agile Spacecraft Attitude Control: An Incremental Nonlinear Dynamic Inversion Approach. IFAC-PapersOnLine 2020, 53, 5709–5716.

26. Acquatella B., P.; van Kampen, E.; Chu, Q.P. A Sampled-data Form of Incremental Nonlinear Dynamic Inversion for Spacecraft Attitude Control. In Proceedings of the AIAA SciTech Forum, San Diego, CA, USA, 3–7 January 2022.

27. Rickmers, P.; Bauer, W.; Stappert, S.; Sippel, M.; Redondo, Gutierrez, J.L.; Seelbinder, D.; Bernal, Polo, P.; Razgus, B.; Theil, S.; Acquatella, B.P.; et al. The Reusability Flight Experiment–ReFEx: From Design to Flight–Hardware. In Proceedings of the International Astronautical Congress (IAC), Dubai, United Arab Emirates, 25–29 October 2021.

28. Rickmers, P.; Kottmar, S.; Wibbels, G.; Bauer, W. ReFEx: Reusability Flight Experiment–A Demonstration Experiment for Technologies for Aerodynamically Controlled RLV Stages. In Proceedings of the Aerospace Europe Conference 2023–10th EUCASS–9th CEAS, Lausanne, Switzerland, 9–13 July 2023.

29. Redondo Gutierrez, J.L.; Seelbinder, D.; Theil, S.; Bernal Polo, P.; Gäßler, B.; Robens, J.; Acquatella B.P. ReFEx: Reusability Flight Experiment–Architecture and Algorithmic Design of the GNC Subsystem. In Proceedings of the Aerospace Europe Conference 2023–10th EUCASS–9th CEAS, Lausanne, Switzerland, 9–13 July 2023.

30. Gäßler, B.: Robens, I. Incremental Nonlinear Dynamic Inversion Flight Control for the DLR Reusability Flight Experiment ReFEx In Proceedings of the AIAA SciTech Forum, Kissimmee, FL, USA, 6–10 January 2025.

31. Mooij, E. Robust Control of a Conventional Aeroelastic Launch Vehicle. In Proceedings of the AIAA SciTech Forum, Orlando, FL, USA, 6–10 January 2020.

32. Mooij, E. Slosh Observer Design for Aeroelastic Launch Vehicles. In Proceedings of the AIAA SciTech 2020 Forum, Orlando, FL, USA, 6–10 January 2020. [CrossRef]

33. Mooij, E.; Wang, X. Incremental Sliding Mode Control for Aeroelastic Launch Vehicles with Propellant Slosh. In Proceedings of the AIAA SciTech 2021 Forum, Virtual, 11–15. 19–21 January 2021. [CrossRef]

34. E. Mooij. Dynamic Inversion Heat-Flux Tracking for Hypersonic Entry. In Proceedings of the AIAA SciTech Forum, National Harbor, MD, USA, 23–27 January 2023.

35. Mooij, E.; Wang, X. Nonlinear Robust Control and Observation for Aeroelastic Launch Vehicles with Propellant Slosh in a Turbulent Atmosphere. In Proceedings of the AIAA SciTech Forum National Harbor, MD, USA, 23–27 January 2023. [CrossRef]

36. Wang, X.; van Kampen, E.; Chu, Q.P.; Lu, P. Stability Analysis for Incremental Nonlinear Dynamic Inversion Control. J. Guid. Control. Dyn. 2019, 42, 1116–1129. [CrossRef]

37. Simplício, P.; Acquatella, P.; Bennani, S. Launcher Attitude Control based on Incremental Nonlinear Dynamic Inversion: A Feasibility Study towards Fast and Robust Design Approaches. In Proceedings of the 12th International ESA Conference on Guidance, Navigation and Control Systems, Sopot, Poland, 12–16 June 2023.

38. Simplício, P.; Marcos, A.; Bennani, S. Reusable Launchers: Development of a Coupled Flight Mechanics, Guidance and Control Benchmark. J. Spacecr. Rocket. 2020, 57, 74–89. [CrossRef]

39. Simplício, P.; Bennani, S.; Marcos, A.; Roux, C.; Lefort, X. Structured Singular-Value Analysis of the Vega Launcher in Atmospheric Flight. J. Guid. Control. Dyn. 2016, 39, 1342–1355. [CrossRef]

40. Simplício, P.; Marcos, A.; Bennani, S. Launcher Flight Control Design using Robust Wind Disturbance Observation. Acta Astronaut. 2021, 186, 303–318. [CrossRef]

41. Navarro-Tapia, D.; Marcos, A.; Bennani, S.; Roux, C. Structured H-infinity and Linear Parameter Varying control design for the VEGA Launch Vehicle. In Proceedings of the The 7th European Conference for Aeronautics and Space Sciences, Milan, Italy, 3–6 July 2017.

42. Vandersteen, J.; Bennani, S.; Roux, C. Robust Rocket Navigation with Sensor Uncertainties: Vega Launcher Application. J. Spacecr. Rocket. 2018, 55, 153–166. [CrossRef]

Disclaimer/Publisher’s Note: The statements, opinions and data contained in all publications are solely those of the individual author(s) and contributor(s) and not of MDPI and/or the editor(s). MDPI and/or the editor(s) disclaim responsibility for any injury to people or property resulting from any ideas, methods, instructions or products referred to in the content.