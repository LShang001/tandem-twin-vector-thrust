# The Invariant Extended Kalman filter as a stable observer

Axel Barrau, Silvere Bonnabel\` <sup>∗</sup>

## Abstract

We analyze the convergence aspects of the invariant extended Kalman filter (IEKF), when the latter is used as a deterministic non-linear observer on Lie groups, for continuoustime systems with discrete observations. One of the main features of invariant observers for left-invariant systems on Lie groups is that the estimation error is autonomous. In this paper we first generalize this result by characterizing the (much broader) class of systems for which this property holds. Then, we leverage the result to prove for those systems the local stability of the IEKF around any trajectory, under the standard conditions of the linear case. One mobile robotics example and one inertial navigation example illustrate the interest of the approach. Simulations evidence the fact that the EKF is capable of diverging in some challenging situations, where the IEKF with identical tuning keeps converging.

## 1 Introduction

The design of non-linear observers is always a challenge, as except for a few classes of systems (e.g., [15]), no general method exists. Of course, the grail of non-linear observer design is to achieve global convergence to zero of the state estimation error, but this is a very ambitious property to pursue. As a first step, a general method is to use standard linearization techniques, such as the extended Kalman filter (EKF) that makes use of Kalman equations to stabilize the linearized estimation error, and then attempt to derive local convergence properties around any trajectory. This is yet a rare property to obtain in a non-linear setting (see, e.g., [1]), due to the fact that the linearized estimation error equation is time varying, and contrarily to the linear case it generally depends on the unknown true state we seek to estimate. The EKF, the most popular observer in the engineering world, provides an “off the shelf” candidate observer, potentially able to deal with the time-varying nature of the linearized error equation, due to its adaptive gain tuning through a Riccati equation. However, the EKF does not possess any optimality guarantee, and its efficiency is aleatory. Indeed, its main flaw lies in its very nature: the Kalman gain is computed assuming the estimation error is sufficiently small to be propagated analytically through a first-order linearization of the dynamics about the estimated trajectory. When the estimate is actually far from the true state variable, the linearization is not valid, and results in an unadapted gain that may amplify the error. In turn, such positive feedback loop may lead to divergence of the filter. This is the reason why mos of the papers dealing with the stability of the EKF (see [11, 25, 24, 10]) rely on the highly nontrivial assumption that the eigenvalues of the Kalman covariance matrix P computed about the estimated trajectory are lower and upper bounded by strictly positive scalars. To the authors’ knowledge, only a few papers deal with the stability of the EKF without invoking this assumption [19]. It is then replaced by second-order properties whose verification can prove difficult in practical situations. This lack of guarantee is also due to the fact the filter can diverge indeed in a number of applications. Note that, beyond the general theory, there are not even that many engineering examples where the EKF is proved to be (locally) stable.

The present paper builds upon the theory of symmetry preserving observers [8, 7] and notably the theory of invariant Kalman filtering [6, 9, 23, 5] in a purely deterministic context. As such, it is a contribution to the theory of non-linear observer design on Lie groups that has lately attracted considerable interest, notably for attitude estimation, see, e.g., [22, 26, 3, 17, 20, 16, 18]. The detailed contributions and organization of the paper are as follows.

In Section 2, we recall the main contribution of [8, 7] is to evidence the fact that for leftinvariant systems on Lie groups, non-linear observers may be designed in such a way that the left-invariant estimation error obeys an autonomous equation, a key property for observer design on Lie groups. We show here this property of the error equation can actually be obtained for a much broader class of systems, and we characterize this class. Very surprisingly, it turns out that, up to a suitable non-linear change of variables, the error evolution (in the absence of measurements) obeys a linear differential equation.

In Section 3, we focus on the invariant extended Kalman filter (IEKF) [6] when applied to the broad class of systems of Section 2. We consider continuous-time models with discrete observations, which best suits navigation systems where high rate sensors governing the dynamics are to be combined with low rate sensors [14]. We change a little the IEKF equations to cast them into a matrix Lie group framework, more handy to use than the usual abstract Lie group formulation of [6]. We then prove, that under the standard convergence conditions of the linear case [13], applied to the linearized model around the true state, the IEKF is an asymptotic observer around any trajectory of the system, a rare to obtain property. This way, we produce a generic observer with guaranteed local convergence properties under natural assumptions, for a broad class of systems on Lie groups, whereas this property has so far only been reserved to specific examples on Lie groups. This also allows putting on firm theoretical ground the good behavior of the IEKF in practice, as already noticed in a few papers, see e.g., [3, 5, 2].

In Section 4 we consider a mobile robotics example, where a unicycle robot (or simplified car) tries to estimate its position and orientation from GPS position (only) measurements, or alternatively landmarks range and bearing measurements. On this example of engineering interest, the IEKF is proved to converge around any trajectory using the results of the paper, which is a contribution in itself. Simulations indicate the IEKF is always superior to the EKF and may even outperform the latter in challenging situations.

In Section 5 we consider the highly relevant problem of an unmanned aerial vehicle (UAV) navigating with accelerometers and gyrometers, and range and bearing measurements of known landmarks. Although the system is not invariant in the sense of [8, 7], it is proved to fit into our framework so that the autonomous error equation property of [7] holds, a fact never noticed before to our best knowledge (except in our preliminary conference paper [4]). The IEKF is shown to converge around any trajectory using the results of the paper, which is a contribution in itself. Moreover, it is shown to outperform the EKF which even diverges when, as in high precision navigation, the user has way more trust in the inertial sensors than in the landmark measurements.

The main contributions can be summarized as follows:

• The class of systems, for which the key result of [7] about the (state) error equation autonomy holds, is completely characterized, and actually shown to be much broader than left-invariant systems.

• The autonomy of the error equation is proved to come with a very intriguing property: a well-chosen non-linear function of the non-linear error is proved to obey a linear differential equation.

• In turn, this property allows proving that, for the introduced class of systems, the IEKF used in a deterministic context possesses powerful local convergence guarantees that the standard EKF lacks.

• Two examples of navigation illustrate the applicability of the results, and simulations indicate indeed the IEKF is always superior to the EKF, and may turn out to literally outperform the latter when confronted with some challenging situations - the EKF being even capable to diverge.

## 2 A special class of multiplicative systems

## 2.1 An introductory example

Consider a linear (deterministic) system $\begin{array} { r } { \frac { d } { d t } x _ { t } = A _ { t } x _ { t } } \end{array}$ . Consider two trajectories of this system, say, a reference trajectory $( \bar { x } _ { t } ) _ { t \geq 0 }$ and another one $( x _ { t } ) _ { t \geq 0 }$ . The discrepancy between both trajectories $\Delta x _ { t } : = x _ { t } - \bar { x } _ { t }$ satisfies the linear equation $\begin{array} { r } { \frac { d } { d t } \Delta x _ { t } = A _ { t } \Delta x _ { t } } \end{array}$ . This is a key property for the design of linear convergent observers, as during the propagation step, the evolution of the error between the true state and the estimate does not depend on the true state’s trajectory.

Consider now the following non-linear standard model of the two-dimensional non-holonomic car. Its state is defined by three parameters : heading $\theta _ { t }$ and position $X _ { t } = ( x _ { t } ^ { 1 } , x _ { t } ^ { 2 } )$ . The velocity $u _ { t } \in \mathbb { R }$ is given by an odometer, the angular velocity $\omega _ { t } \in \mathbb { R }$ is measured by a differential odometer or a gyrometer. The equations read (see, e.g., [12]):

$$
\frac {d}{d t} \theta_ {t} = \omega_ {t}, \quad \frac {d}{d t} x _ {t} ^ {1} = \cos (\theta_ {t}) u _ {t}, \quad \frac {d}{d t} x _ {t} ^ {2} = \sin (\theta_ {t}) u _ {t}.
$$

Now consider a reference trajectory $( \bar { \theta } _ { t } , \bar { X } _ { t } )$ and a second trajectory $\left( \theta _ { t } , X _ { t } \right)$ with different initial conditions but same inputs. The exact propagation of the “error” $\left( \Delta \theta _ { t } , \Delta X _ { t } \right) = \left( \theta _ { t } - \bar { \theta } _ { t } \right.$

$X _ { t } - { \bar { X } } _ { t } )$ , satisfies:

$$
\begin{array}{l} \frac {d}{d t} \Delta \theta_ {t} = 0, \\ \frac {d}{d t} \Delta x _ {t} ^ {1} = \left[ \cos (\theta_ {t}) - \cos (\bar {\theta} _ {t}) \right] u _ {t}, \\ \frac {d}{d t} \Delta x _ {t} ^ {2} = \left[ \sin (\theta_ {t}) - \sin (\bar {\theta} _ {t}) \right] u _ {t}, \end{array}\tag{1}
$$

where we let $\Delta X _ { t } = ( \Delta x _ { t } ^ { 1 } , \Delta x _ { t } ^ { 2 } )$ . We see the time derivative of $\left( \Delta \theta _ { t } , \Delta X _ { t } \right)$ is not a function of $( \Delta \theta _ { t } , \Delta X _ { t } )$ only: it also involves $\bar { \theta } _ { t }$ and $\theta _ { t }$ individually. Moreover, the equation is non-linear. These features, characteristic of non-linear systems, make the design of observers way more complicated in the non-linear case. Now, let us introduce the following non-linear error, where $R ( \theta ) = \cos ( \theta ) I _ { 2 } + \sin ( \theta ) J _ { 2 }$ denotes the planar rotation matrix of angle $\theta$ (see definition of $J _ { 2 }$ below):

$$
\begin{array}{l} \xi_ {t} := \binom{\left(\theta_ {t} - \bar {\theta} _ {t}\right)}{\frac {1}{2} \left(\theta_ {t} - \bar {\theta} _ {t}\right) \left[ a (\theta_ {t} - \bar {\theta} _ {t}) I _ {2} - J _ {2} \right] R (- \bar {\theta} _ {t}) (X _ {t} - \bar {X} _ {t})}, \\ \text { with } J _ {2} = \left( \begin{array}{c c} 0 & - 1 \\ 1 & 0 \end{array} \right) \text { and } a (s) = \frac {\sin (s)}{1 - \cos (s)}, \end{array}\tag{2}
$$

which is equal to 0 indeed if and only if both trajectories coincide. We are about to prove by elementary means a surprising property that will be generalized by Theorem 2.

Proposition 1. Contrarily to the linear error obeying (1), the alternative non-linear error (2) obeys the following linear and autonomous equation although the system, and the error, are totally non-linear:

$$
\frac {d}{d t} \xi_ {t} = \left( \begin{array}{c c c} 0 & 0 & 0 \\ 0 & 0 & \omega_ {t} \\ - u _ {t} & - \omega_ {t} & 0 \end{array} \right) \xi_ {t}.\tag{3}
$$

Proof. We will use the notations $\begin{array} { r } { \delta x _ { t } = \frac { 1 } { 2 } \left( \theta _ { t } - \bar { \theta } _ { t } \right) \left[ a ( \theta _ { t } - \bar { \theta } _ { t } ) I _ { 2 } - J _ { 2 } \right] R ( - \bar { \theta } _ { t } ) ( X _ { t } - \bar { X } _ { t } ) } \end{array}$ . Since we have $\begin{array} { r } { \frac { d } { d t } \delta \theta _ { t } = 0 } \end{array}$ as in the linear case above, only the two last terms of $\delta { x } _ { t }$ change over time. Moreover, $J _ { 2 }$ commutes with $I _ { 2 } , J _ { 2 }$ and $R \big ( - \bar { \theta } _ { t } \big )$ and $\begin{array} { r } { \frac { d } { d t } R ( - \bar { \theta } _ { t } ) = - J _ { 2 } \omega _ { t } R ( - \bar { \theta } _ { t } ) } \end{array}$ . Thus we have:

$$
\begin{array}{l} \frac {d}{d t} \delta x _ {t} = - J _ {2} \omega_ {t} \delta x _ {t} \\ \qquad + \frac {1}{2} \delta \theta_ {t} [ a (\delta \theta_ {t}) I _ {2} - J _ {2} ] R (- \bar {\theta} _ {t}) [ R (\theta_ {t}) - R (\bar {\theta} _ {t}) ] (u _ {t}, 0) ^ {T} \\ = - J _ {2} \omega_ {t} \delta x _ {t} \\ \qquad + \frac {1}{2} \delta \theta_ {t} [ a (\delta \theta_ {t}) I _ {2} - J _ {2} ] [ (\cos (\delta \theta_ {t}) - 1) I _ {2} + \sin (\delta \theta_ {t}) J _ {2} ] (u _ {t}, 0) ^ {T} \\ = - J _ {2} \omega_ {t} \delta x _ {t} + \delta \theta_ {t} J _ {2} (u _ {t}, 0) ^ {T} \\ = - (\omega_ {t} J _ {2}) \delta x _ {t} + \delta \theta_ {t} (0, - u _ {t}) ^ {T}. \end{array}
$$

In the second to last equality we used the relation ${ \frac { \sin ( s ) ^ { 2 } } { 1 - \cos ( s ) } } = 1 + \cos ( s )$ . Equation (3) is proved. □

The present section provides a novel geometrical framework - encompassing this example - to characterize systems on Lie groups for which such a property holds. In turn, such a property will simplify the convergence analysis of non-linear observers, namely the IEKF, due to the implied similarities with the linear case.

## 2.2 Systems on Lie groups with state trajectory independent error propagation property

Let $G \subset \mathbb { R } ^ { N \times N }$ be a matrix Lie group whose Lie algebra is denoted g and has dimension dim g. We consider a class of dynamical systems:

$$
\frac {d}{d t} \chi_ {t} = f _ {u _ {t}} (\chi_ {t}),\tag{4}
$$

where the state $\chi _ { t }$ lives in the Lie group $G$ and $u _ { t }$ is an input variable. Consider two distinct trajectories $\chi _ { t }$ and $\bar { \chi } _ { t }$ of (4). Define the left-invariant and right-invariant errors $\eta _ { t } ^ { L }$ and $\eta _ { t } ^ { R }$ between the two trajectories as:

$$
\eta_ {t} ^ {L} = \chi_ {t} ^ {- 1} \bar {\chi} _ {t} \quad (\text { left   invariant }),\tag{5}
$$

$$
\eta_ {t} ^ {R} = \bar {\chi} _ {t} \chi_ {t} ^ {- 1} \quad (\text { right   invariant }).\tag{6}
$$

The terminology stems from the invariance of e.g., (5) to (left) multiplications $( \chi , \bar { \chi } ) $ $( \Gamma \chi , \Gamma \bar { \chi } )$ for $\Gamma \in G$

Definition 1. The left-invariant and right-invariant errors are said to have a state-trajectory independent propagation if they satisfy a differential equation of the form $\begin{array} { r } { \frac { d } { d t } \eta _ { t } = g _ { u _ { t } } ( \dot { \eta } _ { t } ) } \end{array}$

Note that, in general the time derivative of $\eta _ { t }$ is a complicated function depending on $u _ { t }$ and both $\chi _ { t }$ and $\bar { \chi } _ { t }$ in a way that does not boil down to a function of $\eta _ { t }$ , see for instance eq (1) above. The following result allows characterizing the class of systems of the form (4) for which the property holds.

Theorem 1. The threefollowing conditions are equivalentfor the dynamics (4):

i The left-invariant error (5) is state trajectory independent

ii The right-invariant error (6) is state trajectory independent

iii For all $t > 0$ and $a , b \in G$ we have (in the tangent space at ab):

$$
f _ {u _ {t}} (a b) = f _ {u _ {t}} (a) b + a f _ {u _ {t}} (b) - a f _ {u _ {t}} \left(I _ {d}\right) b,\tag{7}
$$

where $I _ { d }$ denotes the identity matrix. Moreover, ifone ofthese conditions is satisfied we have

$$
\frac {d}{d t} \eta_ {t} ^ {L} = g _ {u _ {t}} ^ {L} (\eta_ {t} ^ {L}) \quad w h e r e \quad g _ {u _ {t}} ^ {L} (\eta) = f _ {u _ {t}} (\eta) - f _ {u _ {t}} (I _ {d}) \eta ,\tag{8}
$$

$$
\frac {d}{d t} \eta_ {t} ^ {R} = g _ {u _ {t}} ^ {R} (\eta_ {t} ^ {R}) \quad w h e r e \quad g _ {u _ {t}} ^ {R} (\eta) = f _ {u _ {t}} (\eta) - \eta f _ {u _ {t}} (I _ {d}).\tag{9}
$$

Proof. Assume we have $\begin{array} { r } { \frac { d } { d t } \eta _ { t } ^ { L } = g _ { u _ { t } } ( \eta _ { t } ^ { L } ) } \end{array}$ for a certain function $g _ { u _ { t } }$ and any $\eta _ { t } ^ { L } = \chi _ { t } ^ { - 1 } \bar { \chi } _ { t }$ , where $\chi _ { t }$ and ${ \bar { \chi } } _ { t }$ are solutions of (4). We have:

$$
\begin{array}{l} g _ {u _ {t}} (\chi_ {t} ^ {- 1} \bar {\chi} _ {t}) = \frac {d}{d t} (\chi_ {t} ^ {- 1} \bar {\chi} _ {t}) = - \chi_ {t} ^ {- 1} [ \frac {d}{d t} \chi_ {t} ] \chi_ {t} ^ {- 1} \bar {\chi} _ {t} + \chi_ {t} ^ {- 1} \frac {d}{d t} \bar {\chi} _ {t} \\ \qquad = - \chi_ {t} ^ {- 1} f _ {u _ {t}} (\chi_ {t}) \eta_ {t} ^ {L} + \chi_ {t} ^ {- 1} f _ {u _ {t}} (\bar {\chi} _ {t}), \\ \text {i.e.} g _ {u _ {t}} (\eta_ {t} ^ {L}) = - \chi_ {t} ^ {- 1} f _ {u _ {t}} (\chi_ {t}) \eta_ {t} ^ {L} + \chi_ {t} ^ {- 1} f _ {u _ {t}} (\chi_ {t} \eta_ {t} ^ {L}). \end{array}\tag{10}
$$

This has to hold for any $\chi _ { t }$ and $\eta _ { t } ^ { L }$ . In the particular case where $\chi _ { t } = I _ { d }$ we obtain:

$$
g _ {u _ {t}} (\eta_ {t} ^ {L}) = f _ {u _ {t}} (\eta_ {t} ^ {L}) - f _ {u _ {t}} (I _ {d}) \eta_ {t} ^ {L}.\tag{11}
$$

Reinjecting (11) in (10) we obtain:

$$
f _ {u _ {t}} \left(\chi_ {t} \eta_ {t} ^ {L}\right) = f _ {u _ {t}} \left(\chi_ {t}\right) \eta_ {t} ^ {L} + \chi_ {t} f _ {u _ {t}} \left(\eta_ {t} ^ {L}\right) - \chi_ {t} f _ {u _ {t}} \left(I _ {d}\right) \eta_ {t} ^ {L}.
$$

The converse is trivial and the proof is analogous for right-invariant errors.

Remark 1. The particular cases ofleft-invariant and right-invariant dynamics, or the combination of both as follows, verify (7). Let $f _ { \nu _ { t } , \omega _ { t } } ( \pmb { \chi } ) = \nu _ { t } \pmb { \chi } + \pmb { \chi } \pmb { \omega } _ { t }$ . We have indeed:

$$
\begin{array}{l} f _ {v _ {t}, \omega_ {t}} (a) b + a f _ {v _ {t}, \omega_ {t}} (b) - a f _ {v _ {t}, \omega_ {t}} (I _ {d}) b \\ = (v _ {t} a + a \omega_ {t}) b + a (v _ {t} b + b \omega_ {t}) - a (v _ {t} + \omega_ {t}) b \\ = v _ {t} a b + a b \omega_ {t} = f _ {v _ {t}, \omega_ {t}} (a b). \end{array}
$$

Remark 2. In the particular case where G is a vector space with standard addition as the group composition law, the condition (7) boils down to $f _ { u _ { t } } ( a + b ) = f _ { u _ { t } } ( a ) + f _ { u _ { t } } ( b ) - f _ { u _ { t } } ( 0 )$ and we recover the affine functions. We thus see the class of system introduced here appears as a generalization ofthe linear case.

In the next section we show that the dynamics of the form (4) with additional property (7) have striking properties generalizing those of linear systems.

## 2.3 Log-linear property of the error propagation

In the sequel, we will systematically consider systems of the form (4) with the additional property (7), i.e. systems on Lie groups defined by

$$
\begin{array}{c} \frac {d}{d t} \chi_ {t} = f _ {u _ {t}} (\chi_ {t}), \\ \text { where } \forall (u, a, b) \quad f _ {u} (a b) = a f _ {u} (b) + f _ {u} (a) b - a f _ {u} (I d) b. \end{array}\tag{12}
$$

For such systems, Theorem 1 proves that the left (resp. right) invariant error is a solution to the equation $\begin{array} { r } { \frac { d } { d t } \eta _ { t } = g _ { u _ { t } } ( \eta _ { t } ) } \end{array}$ where $g _ { u _ { t } }$ is given by (8) (resp. (9)). We have the following novel and striking property.

Theorem 2. [Log-linear property ofthe error] Consider the left or right invariant error $\eta _ { t } ^ { i }$ as defined by (5) or (6) between two arbitrarilyfar trajectories of (12), the superscript i denoting indifferently L or R. Let $\mathcal { L } _ { \mathfrak { g } }$ and exp(.) be defined as in Appendix A. Let $\bar { \zeta } _ { 0 } ^ { i } \in \mathbb { R } ^ { \bar { d i m } \mathfrak { g } }$ be such that initially exp $( \xi _ { 0 } ^ { i } ) = \eta _ { 0 } ^ { i }$ . Let $A _ { u _ { t } } ^ { i }$ be defined by $g _ { u _ { t } } ( \mathbf { e x p } ( \xi ) ) = \mathcal { L } _ { \mathfrak { g } } ( A _ { u _ { t } } ^ { i } \xi ) + O ( \rVert \xi \rVert ^ { 2 } )$ $I f \xi _ { t } ^ { i }$ is definedfor $t > 0$ by the linear differential equation in R<sup>dim</sup> <sup>g</sup>

$$
\frac {d}{d t} \xi_ {t} ^ {i} = A _ {t} ^ {i} \xi_ {t} ^ {i},\tag{13}
$$

then, we have for the true non-linear error $\eta _ { t } ^ { i } ,$ the correspondence at all times and for arbitrarily large errors

$$
\forall t \geq 0 \quad \eta_ {t} ^ {i} = \exp (\xi_ {t} ^ {i}).
$$

The latter result, whose proof has been moved to the Appendix, shows that a wide range of nonlinear problems (see examples below) can lead to linear error equations provided the error variable is correctly chosen. We also see the results displayed in the previous introductory example of Section 2.1 are mere applications of the latter theorem, as the non-holonomic car example turns out to perfectly fit into our framework (see Section 4) and $\xi _ { t }$ in eq (2) actually merely is the Lie logarithm of the left-invariant error. This will be extensively used in Section 3, and in the examples to prove stability properties of IEKFs.

## 3 Invariant Extended Kalman Filtering

In this section we first recap the equations of the Invariant EKF (IEKF), a variant of the EKF devoted to Lie groups space states, that has been introduced in continuous time in [6, 9]. We derive the equations in continuous time with discrete observations here, which has already been done in a restricted setting in [5], and we propose a novel matrix (Lie group) framework to simplify the design. We then show that for the class of systems introduced in Section 2, under observability conditions, and painless conditions on the covariance matrices considered here as design parameters, the IEKF is a (deterministic) non-linear observer with local convergence properties around any trajectory, a feature extremely rare to obtain in the field of non-linear observers, due to the dependency of the estimation error to the true unknown trajectory. The notions necessary to follow Section 3 are given in Appendix A.

## 3.1 Full system and IEKF general structure

We consider in this section an equation on a matrix Lie group $G \subset \mathbb { R } ^ { N \times N }$ of the form:

$$
\frac {d}{d t} \chi_ {t} = f _ {u _ {t}} (\chi_ {t}),\tag{14}
$$

with $f _ { u } ( a b ) = a f _ { u } ( b ) + f _ { u } ( a ) b - a f _ { u } ( I d ) b$ for all $( u , a , b ) \in U \times G \times G$ . This system will be associated to two different kinds of observations.

## 3.1.1 Left-invariant observations

The first family of outputs we are interested in write:

$$
Y _ {t _ {n}} ^ {1} = \chi_ {t _ {n}} d ^ {1} \qquad , \qquad \dots \qquad , \qquad Y _ {t _ {n}} ^ {k} = \chi_ {t _ {n}} d ^ {k},\tag{15}
$$

where $( d ^ { i } ) _ { i \leq k }$ are known vectors. The Left-Invariant Extended Kalman Filter (LIEKF) is defined in this setting through the following propagation and update steps:

$$
\frac {d}{d t} \hat {\chi} _ {t} = f _ {u _ {t}} (\hat {\chi} _ {t}), \quad t _ {n - 1} \leq t <   t _ {n}, \quad \text { Propagation }\tag{16}
$$

$$
\hat {\chi} _ {t _ {n}} ^ {+} = \hat {\chi} _ {t _ {n}} \exp \left[ L _ {n} \left( \begin{array}{c} \hat {\chi} _ {t _ {n}} ^ {- 1} Y _ {t _ {n}} ^ {1} - d ^ {1} \\ ... \\ \hat {\chi} _ {t _ {n}} ^ {- 1} Y _ {t _ {n}} ^ {k} - d ^ {k} \end{array} \right) \right], \quad \text { Update }\tag{17}
$$

where the function $L _ { n } : \mathbb { R } ^ { k N } \to \mathbb { R } ^ { \dim { \mathfrak { g } } }$ is to be defined in the sequel using error linearizations. A left-invariant error between true state $\chi _ { t }$ and estimated state $\hat { \mathcal { X } } _ { t }$ can be associated to this filter:

$$
\eta_ {t} ^ {L} = \chi_ {t} ^ {- 1} \hat {\chi} _ {t}.\tag{18}
$$

During the Propagation step, $\chi _ { t }$ and $\hat { \chi } _ { t }$ are two trajectories of the system (14). Thus, the error (18) is independent from the true state trajectory from Theorem 1 and eq. (8) ! We have thus

$$
\frac {d}{d t} \eta_ {t} ^ {L} = g _ {u _ {t}} ^ {L} (\eta_ {t} ^ {L}), \quad t _ {n - 1} \leq t <   t _ {n}.\tag{19}
$$

Consider now the following linear differential equation in $\mathbb { R } ^ { \mathrm { d i m } \mathfrak { g } }$

$$
\frac {d}{d t} \xi_ {t} = A _ {u _ {t}} \xi_ {t},\tag{20}
$$

where $A _ { u _ { t } }$ is defined by $g _ { u _ { t } } ^ { L } ( \mathbf { e x p } ( \xi ) ) = \mathcal { L } _ { \mathfrak { g } } ( A _ { u _ { t } } \xi ) + O ( \| \xi _ { t } \| ^ { 2 } )$ . Theorem 2 implies the unexpected result:

Proposition 2. $I f \xi _ { t }$ is defined as a solution to the linear system (20) and $\eta _ { t }$ is defined as the solution to the nonlinear error system (19), then if at time $t _ { n - 1 }$ we have $\eta _ { t _ { n - 1 } } = \exp ( \xi _ { t _ { n - 1 } } )$ then the equality $\eta _ { t } = \exp ( \xi _ { t } ) = \exp _ { m } \left( \mathcal { L } _ { \mathfrak { g } } ( \xi _ { t } ) \right)$ is verified at all times $t _ { n - 1 } \leq t < t _ { n }$ , even for arbitrarily large initial errors.

Besides, at the update step, the evolution of the invariant error variable (18) merely writes:

$$
(\eta_ {t _ {n}} ^ {L}) ^ {+} = \chi_ {t _ {n}} ^ {- 1} \hat {\chi} _ {t _ {n}} ^ {+} = \eta_ {t _ {n}} ^ {L} \exp \left[ L _ {n} \left( \begin{array}{c} (\eta_ {t _ {n}} ^ {L}) ^ {- 1} d ^ {1} - d ^ {1} \\ ... \\ (\eta_ {t _ {n}} ^ {L}) ^ {- 1} d ^ {k} - d ^ {k} \end{array} \right) \right].\tag{21}
$$

We see that the nice geometrical structure of the LIEKF allows the updated error $( \eta _ { t _ { n } } ^ { L } ) ^ { + }$ to be here again only a function of the error just before update $\eta _ { t _ { n } } ^ { L }$ , i.e. to be independent from the true state $\mathbb { \chi } _ { t _ { n } }$

## 3.1.2 Right-invariant observations

The second family of observations we are interested in have the form:

$$
Y _ {t _ {n}} ^ {1} = \chi_ {t _ {n}} ^ {- 1} d ^ {1} \qquad , \qquad \dots \qquad , \qquad Y _ {t _ {n}} ^ {k} = \chi_ {t _ {n}} ^ {- 1} d ^ {k},\tag{22}
$$

with the same notations as in the previous section. The Right-Invariant EKF (RIEKF) is defined here as:

$$
\frac {d}{d t} \hat {\chi} _ {t} = f _ {u _ {t}} (\hat {\chi} _ {t}),\tag{23}
$$

$$
\hat {\chi} _ {t _ {n}} ^ {+} = \exp \left[ L _ {n} \left( \begin{array}{c} \hat {\chi} _ {t _ {n}} Y _ {t _ {n}} ^ {1} - d ^ {1} \\ \dots \\ \hat {\chi} _ {t _ {n}} Y _ {t _ {n}} ^ {k} - d ^ {k} \end{array} \right) \right] \hat {\chi} _ {t _ {n}}.\tag{24}
$$

A right-invariant error can be associated to this filter:

$$
\eta_ {t} ^ {R} = \hat {\chi} _ {t} \chi_ {t} ^ {- 1}.\tag{25}
$$

Once again, Theorem 1 proves the evolution of the error does not depend on the state of the system. The analog of Proposition 2 is thus easily derived for the error (25) and we skip it due to space limitations.

At the update step, the evolution of the invariant error variable reads:

$$
(\eta_ {t _ {n}} ^ {R}) ^ {+} = \hat {\chi} _ {t} ^ {+} \chi_ {t} ^ {- 1} = \exp \left[ L _ {n} \left( \begin{array}{c} \eta_ {t _ {n}} ^ {R} d ^ {1} - d ^ {1} \\ ... \\ \eta_ {t _ {n}} ^ {R} d ^ {k} - d ^ {k} \end{array} \right) \right] \eta_ {t _ {n}} ^ {R},
$$

so that the error update does not depend on the true state either.

## 3.2 IEKF gain tuning

In the standard theory of Kalman filtering, EKFs are designed for “noisy” systems associated with the deterministic considered system. In a deterministic context, the covariance matrices Q and N of the noises are left free to tune by the user, and are design parameters for the EKF used as a non-linear observer. Yet, in the spirit of [25], it is nevertheless convenient to associate a “noisy” system with the considered deterministic system consisting of dynamics (14) with outputs (15) or (22). The obtained error equations can be linearized, and the standard Kalman equations applied to make this error decrease. This way, the matrices Q and N can be interpreted as covariance matrices. And in many engineering applications, the characteristics of the noises of the sensors are approximately known, so that the engineer can use the corresponding covariance matrices as a useful guide to tune (or design) the non-linear observer, that is here the IEKF. This provides him with (at least) a first sensible tuning of the parameter matrices, which is consistent with the trust he has in each sensor. Moreover, in the same spirit, the IEKF viewed as a non-linear observer remedies a common weakness shared by numerous non-linear observers on Lie groups, as it conveys an information about its own accuracy through the computed covariance matrix $P _ { t }$ . Although it comes with no rigorous interpretation in a deterministic context, the information conveyed by $P _ { t }$ may prove useful in applications.

Note that, in mobile robotics and navigation, the sensors are attached to the earth-fixed frame (e.g., a GPS) or to the body frame (e.g., a gyrometer). To interpret them as covariance matrices in the IEKF framework (see below) those matrices $Q$ and N may have to undergo a change of frame yielding trajectory dependent tuning matrices $\hat { Q }$ and $\hat { N } ,$ , such as in the application examples in the sequel. This does not weaken the results, but however comes at the price of making the stability analysis a little more complicated.

## 3.2.1 Associated “noisy” system

To tune the IEKF (16)-(17) or (23)-(24), we associate to the system (14) the following “noisy” system:

$$
\frac {d}{d t} \chi_ {t} = f _ {u _ {t}} (\chi_ {t}) + \chi_ {t} w _ {t},\tag{26}
$$

where $w _ { t }$ is a continuous white noise belonging to g whose covariance matrix is denoted by $Q _ { t }$ (for a proper discussion on multiplicative noise for systems defined on Lie groups, see e.g. [5]).

In the same way, we associate to the family of left-invariant observations (15) the following family of “noisy” outputs:

$$
Y _ {t _ {n}} ^ {1} = \chi_ {t _ {n}} \left(d ^ {1} + B _ {n} ^ {1}\right) + V _ {n} ^ {1}, \dots , Y _ {t _ {n}} ^ {k} = \chi_ {t _ {n}} \left(d ^ {k} + B _ {n} ^ {k}\right) + V _ {n} ^ {k},\tag{27}
$$

where the $( V _ { n } ^ { i } ) _ { i \leq k } , ( B _ { n } ^ { i } ) _ { i \leq k }$ are noises with known characteristics. To the family of rightinvariant observations (22) we associate the following family of “noisy” outputs

$$
Y _ {t _ {n}} ^ {1} = \chi_ {t _ {n}} ^ {- 1} \left(d ^ {1} + V _ {n} ^ {1}\right) + B _ {n} ^ {1}, \dots , Y _ {t _ {n}} ^ {k} = \chi_ {t _ {n}} ^ {- 1} \left(d ^ {k} + V _ {n} ^ {k}\right) + B _ {n} ^ {k}.\tag{28}
$$

## 3.2.2 Linearized “noisy” estimation error equation

As in a conventional EKF, we assume the error to be small (here close to Id as it is equal to Id if $\hat { \chi } _ { t } = \chi _ { t } )$ so that the error system can be linearized to compute the gains $L _ { n }$ . By definition, the Lie algebra g represents the infinitesimal variations around $I d$ of an element of $G .$ . Thus the natural way to define a vector error variable $\xi _ { t }$ in $\mathbb { R } ^ { \mathrm { d i m } \mathfrak { g } }$ is (see Appendix A):

$$
\eta_ {t} = \exp (\xi_ {t}) = \exp_ {m} \left(\mathscr {L} _ {\mathfrak {g}} (\xi_ {t})\right).\tag{29}
$$

During the Propagation step, that is for $t _ { n - 1 } \leq t < t _ { n }$ , elementary computations based on the results of Theorem 1 show that for the noisy model (26) we have

$$
\begin{array}{l} \frac {d}{d t} \eta_ {t} ^ {L} = g _ {u _ {t}} ^ {L} (\eta_ {t} ^ {L}) - w _ {t} \eta_ {t} ^ {L}, \\ \frac {d}{d t} \eta_ {t} ^ {R} = g _ {u _ {t}} ^ {R} (\eta_ {t} ^ {R}) - (\hat {\chi} _ {t} w _ {t} \hat {\chi} _ {t} ^ {- 1}) \eta_ {t} ^ {R}. \end{array}\tag{30}
$$

Defining $\hat { w } _ { t } \in \mathbb { R } ^ { \dim { \mathfrak { g } } }$ by $\mathcal { L } _ { \mathfrak { g } } ( \hat { w } _ { t } ) = - w _ { t }$ in the first case and $\mathcal { L } _ { \mathfrak { g } } ( \hat { w } _ { t } ) = - \hat { \chi } _ { t } w _ { t } \hat { \chi } _ { t } ^ { - 1 }$ (i.e. $\hat { w } _ { t } =$ $- A d _ { \hat { \chi } _ { t } } \mathcal { L } _ { \mathfrak { g } } ^ { - 1 } ( w _ { t } ) )$ in the second case, and using the superscript i to denote indifferently L or R we end up with the linearized error equation in R<sup>dimg</sup>:

$$
\frac {d}{d t} \xi_ {t} = A _ {u _ {t}} ^ {i} \xi_ {t} + \hat {w} _ {t},\tag{31}
$$

where $A _ { u _ { t } } ^ { i }$ is defined by $g _ { u _ { t } } ^ { i } ( \mathbf { e x p } ( \xi ) ) = \mathcal { L } _ { \mathfrak { g } } ( A _ { u _ { t } } ^ { i } \xi ) + O ( \| \xi _ { t } \| ^ { 2 } )$ and where we have neglected terms of order $O ( \| \boldsymbol { \xi } _ { t } \| ^ { 2 } )$ as well as terms of order $O ( \left\| \hat { w } _ { t } \right\| \left\| \xi _ { t } \right\| )$ . The latter approximation, as well as the fact that $\hat { w } _ { t }$ can be considered as white noise, are approximations that would require a proper justification in a stochastic setting. However, they are part of the standard EKF methodology, see e.g., [21] and justifying them is not the object of the present paper. Besides, the emphasis is put here on deterministic properties of the observer.

Regarding the output, we consider for instance the case of left-invariant observations, and define $\xi _ { t _ { n } }$ through the exponential mapping (29), i.e. $\exp ( \xi _ { t _ { n } } ) = \eta _ { t _ { n } } ^ { L }$ . Moreover, for $1 \leq i \leq k$ let $\hat { V } _ { n } ^ { i }$ denote $\hat { \chi } _ { t _ { n } } ^ { - 1 } V _ { n } ^ { i }$ . The error update (21), when the LIEKF update (17) is fed with the “noisy” measurements (27) becomes

$$
\begin{array}{c} \left(\eta_ {t _ {n}} ^ {L}\right) ^ {+} = \chi_ {t _ {n}} ^ {- 1} \hat {\chi} _ {t _ {n}} ^ {+} \\ = \eta_ {t _ {n}} ^ {L} \exp \left[ L _ {n} \left( \begin{array}{c} \left(\eta_ {t _ {n}} ^ {L}\right) ^ {- 1} d ^ {1} - d ^ {1} + \hat {V} _ {n} ^ {1} + \left(\eta_ {t _ {n}} ^ {L}\right) ^ {- 1} B _ {n} ^ {1} \\ \dots \\ \left(\eta_ {t _ {n}} ^ {L}\right) ^ {- 1} d ^ {k} - d ^ {k} + \hat {V} _ {n} ^ {k} + \left(\eta_ {t _ {n}} ^ {L}\right) ^ {- 1} B _ {n} ^ {k} \end{array} \right) \right]. \end{array}\tag{32}
$$

To linearize it we proceed as follows. For $1 \leq i \leq k$ we have

$$
\begin{array}{r l} & {(\eta_ {t _ {n}}) ^ {- 1} d ^ {i} - d ^ {i} + \hat {V} _ {n} ^ {i} + (\eta_ {t _ {n}} ^ {L}) ^ {- 1} B _ {n} ^ {i}} \\ & {= \exp_ {m} (\mathcal {L} _ {\mathfrak {g}} (\xi_ {t _ {n}})) ^ {- 1} (d ^ {i} + B _ {n} ^ {i}) - d ^ {i} + \hat {V} _ {n} ^ {i}} \\ & {= (I d - \mathcal {L} _ {\mathfrak {g}} (\xi) _ {t _ {n}}) (d ^ {i} + B _ {n} ^ {i}) - d ^ {i} + \hat {V} _ {n} ^ {i} + O \| \xi_ {t _ {n}} \| ^ {2}} \\ & {= - \mathcal {L} _ {\mathfrak {g}} (\xi) _ {t _ {n}} d ^ {i} + \hat {V} _ {n} ^ {i} + B _ {n} ^ {i} + O (\| \xi_ {t _ {n}} \| ^ {2}) + O (\| \xi_ {t _ {n}} \| \| B _ {n} ^ {i} \|),} \end{array}
$$

using a simple Taylor expansion of the matrix exponential map. Expanding similarly equation (32) yields

$$
I d + \mathcal {L} _ {\mathfrak {g}} (\xi_ {t _ {n}} ^ {+}) = I d + \mathcal {L} _ {\mathfrak {g}} \left(L _ {n} \left( \begin{array}{c} - \mathcal {L} _ {\mathfrak {g}} (\xi_ {t _ {n}}) d ^ {1} + \hat {V} _ {n} ^ {1} + B _ {n} ^ {1} \\ ... \\ - \mathcal {L} _ {\mathfrak {g}} (\xi_ {t _ {n}}) d ^ {k} + \hat {V} _ {n} ^ {k} + B _ {n} ^ {k} \end{array} \right)\right) + T.\tag{33}
$$

with $T$ terms of order $O \left( \| \xi _ { t _ { n } } \| ^ { 2 } \right) + O \left( \| \xi _ { t _ { n } } \| \| B _ { n } \| \right)$ . Neglecting them we finally get the following linearized error equation in R<sup>dim</sup> <sup>g</sup>

$$
\xi_ {t _ {n}} ^ {+} = \xi_ {t _ {n}} + L _ {n} \left(H \xi_ {t _ {n}} + \hat {V} _ {n} + B _ {n}\right),\tag{34}
$$

where $H \in \mathbb { R } ^ { k N \times \mathrm { d i n } }$ <sup>mg</sup>, $\hat { V } _ { n } \in \mathbb { R } ^ { k N }$ and $B _ { n } \in \mathbb { R } ^ { k N }$ are defined by

$$
H \xi = \left( \begin{array}{c} - \mathcal {L} _ {\mathfrak {g}} (\xi) d ^ {1} \\ ... \\ - \mathcal {L} _ {\mathfrak {g}} (\xi) d ^ {k} \end{array} \right), \qquad \hat {V} _ {n} = \left( \begin{array}{c} \hat {V} _ {n} ^ {1} \\ ... \\ \hat {V} _ {n} ^ {k} \end{array} \right), \qquad B _ {n} = \left( \begin{array}{c} B _ {n} ^ {1} \\ ... \\ B _ {n} ^ {k} \end{array} \right).
$$

Now, let $\hat { Q } _ { t }$ reflect the trusted covariance of the modified process noise $\hat { w } _ { t }$ , and $\hat { N } _ { n }$ the trusted covariance of the modified measurement noise $\hat { V } _ { n } + B _ { n }$ . Note that, equations (31) and (34) mimick those of a Kalman filter designed for the following auxiliary linear system with discrete measurements: $\begin{array} { r } { \frac { d } { d t } x _ { t } = A _ { u _ { t } } x _ { t } + \hat { w } _ { t } , y _ { n } = H x _ { t _ { n } } + \hat { V } _ { n } + B _ { n } } \end{array}$ . The standard Kalman theory thus suggests to compute $L _ { n }$ through the Riccati equation

$$
\begin{array}{c} \frac {d}{d t} P _ {t} = A _ {u _ {t}} P _ {t} + P _ {t} A _ {u _ {t}} ^ {T} + \hat {Q} _ {t}, \qquad P _ {t _ {n}} ^ {+} = (I - L _ {n} H) P _ {t _ {n}}, \\ \text { with } S _ {n} = H P _ {t _ {n}} H ^ {T} + \hat {N} _ {n}, L _ {n} = P _ {t _ {n}} H ^ {T} S ^ {- 1}. \end{array}\tag{35}
$$

## 3.3 Summary of IEFK equations

In a deterministic context, the IEKF equations can be compactly recapped as follows

$$
\begin{array}{l l} & \frac {d}{d t} \hat {\chi} _ {t} = f _ {u _ {t}} (\hat {\chi} _ {t}), \quad t _ {n - 1} \leq t <   t _ {n}, \\ & \hat {\chi} _ {t _ {n}} ^ {+} = \hat {\chi} _ {t _ {n}} \exp \left[ L _ {n} \left( \begin{array}{c} \hat {\chi} _ {t _ {n}} ^ {- 1} Y _ {t _ {n}} ^ {1} - d ^ {1} \\ \dots \\ \hat {\chi} _ {t _ {n}} ^ {- 1} Y _ {t _ {n}} ^ {k} - d ^ {k} \end{array} \right) \right], (\text { LIEKF }), \\ \underline {{\mathrm{or}}} & \hat {\chi} _ {t _ {n}} ^ {+} = \exp \left[ L _ {n} \left( \begin{array}{c} \hat {\chi} _ {t _ {n}} Y _ {t _ {n}} ^ {1} - d ^ {1} \\ \dots \\ \hat {\chi} _ {t _ {n}} Y _ {t _ {n}} ^ {k} - d ^ {k} \end{array} \right) \right] \hat {\chi} _ {t _ {n}}, (\text { RIEKF }) \end{array}\tag{36}
$$

where the LIEKF (resp. RIEKF) is to be used in the case of left (resp. right) invariant outputs. The gain $L _ { n }$ is obtained in each case through the following Riccati equation

$$
\begin{array}{c} \frac {d}{d t} P _ {t} = A _ {u _ {t}} P _ {t} + P _ {t} A _ {u _ {t}} ^ {T} + \hat {Q} _ {t}, \\ S _ {n} = H P _ {t _ {n}} H ^ {T} + \hat {N} _ {n}, \quad L _ {n} = P _ {t _ {n}} H ^ {T} S ^ {- 1}, \quad P _ {t _ {n}} ^ {+} = (I - L _ {n} H) P _ {t _ {n}}. \end{array}\tag{37}
$$

As concerns the LIEKF, $A _ { u _ { t } }$ is defined by $g _ { u _ { t } } ^ { L } ( \mathbf { e x p } ( \boldsymbol { \xi } _ { } ) ) = \mathcal { L } _ { \mathfrak { g } } ( A _ { u _ { t } } \boldsymbol { \xi } _ { } ) + O ( \Vert \boldsymbol { \xi } _ { } \Vert ^ { 2 } )$ , and $H \in$ $\mathbb { R } ^ { k N }$ <sup>×dimg</sup> is defined by $\boldsymbol { H } \boldsymbol { \xi } = \left( - \left( \mathcal { L } _ { \mathfrak { g } } ( \boldsymbol { \xi } ) d ^ { 1 } \right) ^ { T } , . . . , - \left( \mathcal { L } _ { \mathfrak { g } } ( \boldsymbol { \xi } ) d ^ { k } \right) ^ { T } \right) ^ { T }$ . The design matrix parameters $\hat { Q } _ { t } , \hat { N } _ { n } , \hat { V } _ { n } , B _ { n }$ are freely assigned by the user. When sensor noise characteristics are known, they can provide the user with a first sensible tuning of those matrices by considering the associated “noisy” system $( 2 6 ) – ( 2 7 )$ of Section 3.2. In this case, the matrices can be interpreted in the following way: Q<sup>ˆ</sup><sub>t</sub> ∈ R<sup>dim</sup> <sup>g×dim</sup> <sup>g</sup> denotes the covariance of the modified process noise $\hat { w } _ { t } = - \mathcal { L } _ { \mathfrak { g } } ^ { - 1 } ( w _ { t } )$ and $\hat { N } _ { n }$ the covariance matrix of the noise $\hat { V } _ { n } + B _ { n } , \hat { V } _ { n }$ and $B _ { n }$ being defined as: $\hat { V } _ { n } \stackrel {  } { = } ( \hat { \chi } _ { t _ { n } } ^ { - 1 } V _ { n } ^ { 1 } , . . . , \hat { \chi } _ { t _ { n } } ^ { - 1 } V _ { n } ^ { k } ) ^ { T } ~ , B _ { n } = ( B _ { n } ^ { 1 } , . . . , B _ { n } ^ { k } ) ^ { T }$

As concerns the RIEKF implementation, $A _ { u _ { t } }$ is defined by $g _ { u _ { t } } ^ { R } ( \exp ( \xi ) ) = \mathcal { L } _ { \mathfrak { g } } ( A _ { u _ { t } } \xi ) +$ $O ( \| \xi \| ^ { 2 } )$ , and $H \in \mathbb { R } ^ { k N }$ <sup>×dimg</sup> by $\boldsymbol { H } \boldsymbol { \xi } = \left( \left( \mathcal { L } _ { \mathfrak { g } } ( \boldsymbol { \xi } ) d ^ { 1 } \right) ^ { T } , . . . , \left( \mathcal { L } _ { \mathfrak { g } } ( \boldsymbol { \xi } ) d ^ { k } \right) ^ { T } \right) ^ { T }$ . The design matrix parameters $\hat { Q } _ { t } , \hat { N } _ { n } , \hat { V } _ { n } , B _ { n }$ are freely assigned by the user. Considering the associated “noisy” system (26)-(28) of Section 3.2 they can be interpreted as follows. $\hat { Q } _ { t }$ denotes the covariance of the modified process noise $\hat { w } _ { t } = - A d _ { \hat { \chi } _ { t } } \mathcal { L } _ { \mathfrak { g } } ^ { - 1 } ( w _ { t } )$ and $\hat { N } _ { n }$ the covariance matrix of the noise $V _ { n } + \hat { B } _ { n } , V _ { n }$ and ${ \hat { B } } _ { n }$ being defined as: $\boldsymbol { V _ { n } } = \left( \boldsymbol { \bar { V } } _ { n } ^ { 1 } , . . . , \boldsymbol { V } _ { n } ^ { k } \right) ^ { T } , \boldsymbol { \hat { B } _ { n } } = \left( \boldsymbol { \hat { \chi } } _ { t _ { n } } \boldsymbol { B } _ { n } ^ { 1 } , . . . , \boldsymbol { \hat { \chi } } _ { t _ { n } } \boldsymbol { B } _ { n } ^ { k } \right) ^ { T }$

## 3.4 Stability properties

The aim of the present section is to study the stability properties of the IEKF as a deterministic observer for the system (14)-(15), or alternatively (14)-(22). We are about to prove the IEKF has guaranteed (local) stability properties, that rely on the error equation properties the EKF does not possess. The stability of an observer is defined as its ability to recover from a perturbation or an erroneous initialization:

Definition 2. Let $( x _ { 0 } , t _ { 0 } , t ) \to X _ { t _ { 0 } } ^ { t } ( x _ { 0 } )$ denote a continuous flow on a space $\mathcal { X }$ endowed with $a$ distance d. A flow $( z , t _ { 0 } , t )  \hat { X } _ { t _ { 0 } } ^ { t } ( z )$ is an asymptotically stable observer of X about the trajectory $\left( X _ { t _ { 0 } } ^ { t } ( x ) \right) _ { t \geq t _ { 0 } } i f$ there exists $\varepsilon > 0$ such that:

$$
d \left(x, \tilde {x}\right) <   \varepsilon \Rightarrow d \left(X _ {t _ {0}} ^ {t} (x), \hat {X} _ {t _ {0}} ^ {t} (\tilde {x})\right)\rightarrow 0 w h e n t \rightarrow + \infty .
$$

Theorem 4 below is the main result of the paper . It is a consequence of Theorem 2 of Section 2. J. J. Deyst and C. F. Price have shown in [13] the following theorem, stating sufficient conditions for the Kalman filter to be a stable observer for linear (time-varying) deterministic systems.

Theorem 3 (Deyst and Price, 1968). Consider the linear system $\begin{array} { r } { \frac { d } { d t } x _ { t } = A _ { t } x _ { t } , y _ { t _ { n } } = H x _ { t _ { n } } } \end{array}$ with $x _ { t } \in \mathbb { R } ^ { p }$ and let Φ<sup>t</sup> denote the square matrix defined by $\begin{array} { r } { \Phi _ { t _ { 0 } } ^ { t _ { 0 } } = I _ { p } , \frac { d } { d t } \Phi _ { t _ { 0 } } ^ { t } = A _ { t } \Phi _ { t _ { 0 } } ^ { t } } \end{array}$ . If there exist $\alpha _ { 1 } , \alpha _ { 2 } , \beta _ { 1 } , \beta _ { 2 } , \delta _ { 1 } , \tilde { \delta } _ { 2 } , \delta _ { 3 } , M$ such that:

$$
i \left(\Phi_ {t _ {n}} ^ {t _ {n + 1}}\right) ^ {T} \Phi_ {t _ {n}} ^ {t _ {n + 1}} \succeq \delta_ {1} I _ {p} \succeq 0,
$$

$$
i i \exists q \in \mathbb {N} ^ {*}, \forall s > 0, \exists G _ {s} \in \mathbb {R} ^ {p \times q}, Q _ {s} = G _ {s} Q ^ {\prime} G _ {s} ^ {T} w h e r e Q ^ {\prime} \succeq \delta_ {2} I _ {q} \succeq 0,
$$

iii $N _ { n } \succeq \delta _ { 3 } I \succeq 0$

$$
i v \alpha_ {1} I _ {p} \leq \int_ {s = t _ {n - M}} ^ {t _ {n}} \left(\Phi_ {s} ^ {t _ {n}}\right) Q _ {s} \left(\Phi_ {s} ^ {t _ {n}}\right) ^ {T} \leq \alpha_ {2} I _ {p},
$$

$$
v \beta_ {1} I _ {p} \leq \sum_ {i = n - M} ^ {n - 1} \left(\Phi_ {t _ {i + 1}} ^ {t _ {n}}\right) ^ {T} H ^ {T} N _ {n} ^ {- 1} H \left(\Phi_ {t _ {i + 1}} ^ {t _ {n}}\right) \leq \beta_ {2} I _ {p}.
$$

Then the linear Kalman filter tuned with covariance matrices Q and N is an asymptotically stable observerfor the Euclidean distance. More precisely there exist $\gamma _ { m i n } , \gamma _ { m a x } > 0$ such that $\gamma _ { m i n } I \preceq P _ { t } I \preceq \gamma _ { m a x } I$ for all t and $( \hat { x } _ { t } - x _ { t } ) ^ { T } P _ { t } ^ { - 1 } ( \hat { x } _ { t } - x _ { t } )$ has exponential decay.

The main theorem of the present paper is the extension of this linear result to the non-linear case when the Invariant Extended Kalman Filter is used for systems of Section 2.

Theorem 4. Consider the system (14)-(15) (respectively (14)-(22)). Suppose the stability conditions of the linear Kalman filter given in Theorem 3 are verified about the true system’s trajectory χ (i.e. are verifiedfor the linear system obtained by linearizing the system (14) with left (resp. right) invariant output about $\chi _ { t } )$ . Then the Left (resp. Right) Invariant Extended Kalman Filter estimate $\hat { \chi } _ { t }$ defined at eq. (36) is an asymptotically stable observer of $\chi _ { t }$ in the sense of Definition 2. Moreover, the convergence radius $\varepsilon > 0$ is valid over the whole trajectory (i.e. is independent ofthe initialization time $t _ { 0 } )$

Proof. The full proof is technical and has been moved to Appendix C. The rationale is to compare the evolution of the logarithmic error $\xi _ { t }$ defined as $\eta _ { t } = \exp ( \xi _ { t } )$ , with its linearization. For the general EKF, the control of second and higher order terms in the error equation is difficult because: 1- they depend on the inputs $u _ { t } \ 2 \cdot$ - they depend on the linearization point $\hat { \chi } _ { t }$ 3- the estimation error impacts the gain matrices. For the IEKF, the main difficulties vanish as during the propagation step the IEKF is built for the logarithmic error $\xi _ { t }$ whose evolution $\begin{array} { r } { \frac { d } { d t } \xi _ { t } = A _ { u _ { t } } \xi _ { t } } \end{array}$ is in fact exact (no higher order terms) due to Theorem 2. At the update step, due to the specific form of the IEKF update, second order terms can be uniformly bounded over n. And finally, due to the error equation of the IEKF, the Riccati equation depends on the estimate only through the matrices $\hat { Q } _ { t }$ and $\hat { N } _ { t }$ which affect stability in a minor way, as shown by Theorem 3.

The result displayed in Theorem 4 is in sharp contrast with the usual results available, that make the highly non-trivial assumption that the linearized system around the estimated trajectory is well-behaved [11, 25, 24, 10]. But this fact is almost impossible to predict as when the estimate is (even slightly) away from the true state, the Kalman gain becomes erroneous, which can in turn amplify the discrepancy between estimate and true state so that there is no reason the assumption should keep holding. On the other hand, when considering an actual system undergoing a realistic physical motion (see the examples below), if sufficiently many sensors are available, one can generally assert in advance the linearized system around the true trajectory possesses all the desired properties. The following consequence proves useful in practice.

Theorem 5. Assume the system linearized around the true trajectory has the following properties : the propagation matrix A is constant, there exist matrices $B , D$ such that $\hat { Q } = \bar { B Q B } ^ { T }$ and $\hat { N } = D \bar { N } D ^ { T }$ with Q and <sup>¯</sup> N upper- and lower-bounded, with <sup>¯</sup> (A,H,B,D) detectable and reachable. Then the conditions of Theorem 4 are satisfied and the IEKF is asymptotically stable.

## 4 Simplified car example

The computations require only basic knowledge about matrix Lie groups as recalled in $\mathbf { A p } \llangle$ pendix A.

## 4.1 Considered model

Consider a (non-holonomic) car evolving on the 2D plane. Its heading is denoted by an angle $\theta _ { t } \in [ - \pi , \pi ]$ and its position by a vector $\boldsymbol { x } _ { t } \in \mathbb { R } ^ { 2 }$ . They follow the classical equations (see, e.g., [12]):

$$
\frac {d}{d t} \theta_ {t} = u _ {t} v _ {t}, \quad \frac {d}{d t} x _ {t} ^ {1} = \cos (\theta_ {t}) v _ {t}, \quad \frac {d}{d t} x _ {t} ^ {2} = \sin (\theta_ {t}) v _ {t},\tag{38}
$$

where $\nu _ { t }$ is the velocity measured by an odometer and $u _ { t }$ (a function of) the steering angle. Two kinds of observations are considered:

$$
\bar {Y} _ {n} = x _ {t _ {n}}\tag{39}
$$

$$
\mathrm{or} \bar {Y} _ {n} ^ {k} = R (\theta_ {t _ {n}}) ^ {T} (x _ {t _ {n}} - p _ {k}), \qquad k \in [ 1, K ]\tag{40}
$$

where $R ( \theta )$ is a planar rotation of angle θ. Equation (39) represents a position measurement (GPS for instance) whereas (40) represents a range-and-bearing observation of a sequence of known features located at $p _ { k } \in \mathbb { R } ^ { 2 }$ for $k \in [ 1 , K ]$

## 4.2 IEKF gain tuning

To derive and tune the IEKF equations, we follow the methodology of Section 3.2 which amounts to 1- associate a “noisy ”system to the original considered system, just because it allows obtaining a sensible tuning of the design matrices from an engineering viewpoint, 2- transform it into a system defined on a matrix Lie group to make it fit into our framework, 3- linearize the “noisy” equations, and 4- use the Kalman equations to tune the observer gain.

## 4.2.1 Associated “noisy” system

Taking into account the possible noise in the measurements we get

$$
\begin{array}{c} \frac {d}{d t} \theta_ {t} = u _ {t} v _ {t} + w _ {t} ^ {\theta}, \qquad \frac {d}{d t} x _ {t} ^ {1} = \cos (\theta_ {t}) (v _ {t} + w _ {t} ^ {l}) - \sin (\theta_ {t}) w _ {t} ^ {t r}, \\ \frac {d}{d t} x _ {t} ^ {2} = \sin (\theta_ {t}) (v _ {t} + w _ {t} ^ {l}) + \cos (\theta_ {t}) w _ {t} ^ {t r}, \end{array}\tag{41}
$$

with $w _ { t } ^ { \theta }$ the differential odometry error, $w _ { t } ^ { l }$ the longitudinal odometry error and $w _ { t } ^ { t r }$ the transversal shift. By letting noise enter the measurement equations we get the following two kinds of measurements:

$$
\begin{array}{c} \bar {Y} _ {n} = x _ {t _ {n}} + V _ {n} \\ \text {or} \bar {Y} _ {n} ^ {k} = R (\boldsymbol {\theta} _ {t _ {n}}) ^ {T} (x _ {t _ {n}} - p _ {k}) + \bar {V} _ {n} ^ {k}, \qquad k \in [ 1, K ]. \end{array}\tag{42}
$$

(43)

## 4.2.2 Matrix form

This system can be embedded in the matrix Lie group SE(2) (see Appendix A.1) using the matrices:

$$
\chi_ {t} = \left( \begin{array}{c c c} \cos (\theta_ {t}) & - s i n (\theta_ {t}) & x _ {t} ^ {1} \\ \sin (\theta_ {t}) & \cos (\theta_ {t}) & x _ {t} ^ {2} \\ 0 & 0 & 1 \end{array} \right),
$$

$$
\boldsymbol {v} _ {t} = \left( \begin{array}{c c c} 0 & - u _ {t} v _ {t} & v _ {t} \\ u _ {t} v _ {t} & 0 & 0 \\ 0 & 0 & 0 \end{array} \right), \quad w _ {t} = \left( \begin{array}{c c c} 0 & - w _ {t} ^ {\theta} & w _ {t} ^ {l} \\ w _ {t} ^ {\theta} & 0 & w _ {t} ^ {t r} \\ 0 & 0 & 0 \end{array} \right).
$$

The equation (41) governing the “noisy” system evolution writes:

$$
\frac {d}{d t} \chi_ {t} = \chi_ {t} (v _ {t} + w _ {t}),\tag{44}
$$

and the observations (42) and (43) respectively have the equivalent form:

$$
Y _ {n} = \binom{x _ {t _ {n}} + V _ {n}}{1} = \chi_ {t _ {n}} \binom{0 _ {2 \times 1}}{1} + \binom{V _ {n}}{0},\tag{45}
$$

$$
Y _ {n} ^ {k} = \binom{R (\hat {\theta} _ {t _ {n}}) ^ {T} (x _ {t _ {n}} - p _ {k})}{1} + \binom{\bar {V} _ {n} ^ {k}}{0} = - \chi_ {t _ {n}} ^ {- 1} \binom{p _ {k}}{1} + \binom{\bar {V} _ {n} ^ {k}}{0}.\tag{46}
$$

The reader can verify relation (7) letting $f _ { \nu _ { t } } ( \chi _ { t } ) = \chi _ { t } \nu _ { t }$

## 4.2.3 IEKF equations for the left-invariant output (39)

The LIEKF equations (36) for the associated “noisy” system (44), (45) write:

$$
\frac {d}{d t} \hat {\chi} _ {t} = \hat {\chi} _ {t} v _ {t}, \qquad \hat {\chi} _ {t _ {n}} ^ {+} = \hat {\chi} _ {t _ {n}} \exp \left(L _ {n} \left[ \hat {\chi} _ {t _ {n}} ^ {- 1} Y _ {n} - \binom{0 _ {2 \times 1}}{1} \right]\right).
$$

As the bottom element of $\left\lceil \hat { \chi } _ { t _ { n } } ^ { - 1 } Y _ { n } - \binom { 0 _ { 2 \times 1 } } { 1 } \right\rceil$ is always zero we can conveniently use a reduced-dimension gain matrix ${ \tilde { L } } _ { n }$ defined by $L _ { n } = { \tilde { L } } _ { n } { \tilde { p } }$ with $\tilde { p } = ( I _ { 2 } , 0 _ { 2 , 1 } )$ . To compute the gains, we write the left-invariant error $\eta _ { t } = \chi _ { t } ^ { - 1 } \hat { \chi } _ { t }$ whose evolution is:

$$
\begin{array}{l} \frac {d}{d t} \eta_ {t} = \eta_ {t} v _ {t} - v _ {t} \eta_ {t} - w _ {t} \eta_ {t}, \\ \eta_ {t _ {n}} ^ {+} = \eta_ {t _ {n}} \exp \left(\tilde {L} _ {n} \tilde {p} \left[ \eta_ {t _ {n}} ^ {- 1} \binom{0 _ {2 \times 1}}{1} - \binom{0 _ {2 \times 1}}{1} + \hat {\chi} _ {t _ {n}} ^ {- 1} \binom{V _ {n}}{0} \right]\right). \end{array}\tag{47}
$$

To linearize this equation we introduce the linearized error $\xi _ { t }$ defined replacing $\eta _ { t }$ with ${ \cal I } _ { 3 } +$ $\mathcal { L } _ { \mathfrak { s e } ( 2 ) } ( \xi _ { t } )$ . Introducing the first-order approximations $\eta _ { t } = I _ { 3 } + \mathcal { L } _ { \mathfrak { s e } ( 2 ) } ( \xi _ { t } ) , \eta _ { t } ^ { + } = I _ { 3 } + \mathcal { L } _ { \mathfrak { s e } ( 2 ) } ( \xi _ { t } ^ { + } )$ $\exp ( u ) = I _ { 3 } + \mathcal { L } _ { \mathfrak { s e } ( 2 ) } ( u )$ and $\eta _ { t } ^ { - 1 } = I _ { 3 } - \mathcal { L } _ { \mathfrak { s e } ( 2 ) } ( \xi _ { t } )$ in (47) and removing the second-order terms in $\xi _ { t } , V _ { n }$ and w we obtain:

$$
\frac {d}{d t} \xi_ {t} = - \left( \begin{array}{c c c} 0 & 0 & 0 \\ 0 & 0 & - u _ {t} v _ {t} \\ - v _ {t} & u _ {t} v _ {t} & 0 \end{array} \right) \xi_ {t} - \left( \begin{array}{c} w _ {t} ^ {\theta} \\ w _ {t} ^ {l} \\ w _ {t} ^ {t r} \end{array} \right)
$$

and

$$
\boldsymbol {\xi} _ {t _ {n}} ^ {+} = \boldsymbol {\xi} _ {t _ {n}} - \tilde {L} _ {n} \left[ (0 _ {2, 1}, I _ {2}) \boldsymbol {\xi} _ {t} - R (\hat {\boldsymbol {\theta}} _ {t _ {n}}) ^ {T} V _ {n} \right].
$$

The gains ${ \tilde { L } } _ { n }$ are thus finally computed using the Riccati equation (37) with:

$$
A _ {t} = - \left( \begin{array}{c c c} 0 & 0 & 0 \\ 0 & 0 & - u _ {t} v _ {t} \\ - v _ {t} & u _ {t} v _ {t} & 0 \end{array} \right), \quad H = (0 _ {2, 1}, I _ {2}),
$$

$$
\hat {Q} _ {t} = \operatorname{Cov} \left[ \left(w _ {t} ^ {\theta}, w _ {t} ^ {l}, w _ {t} ^ {t r}\right) ^ {T} \right], \quad \hat {N} = R \left(\hat {\boldsymbol {\theta}} _ {t _ {n}}\right) \operatorname{Cov} \left(V _ {n}\right) R \left(\hat {\boldsymbol {\theta}} _ {t _ {n}}\right) ^ {T}.
$$

## 4.2.4 IEKF equations for the right-invariant output (43)

The RIEKF equations (36) for the associated “noisy” system (44), (46) write:

$$
\frac {d}{d t} \hat {\chi} _ {t} = \hat {\chi} _ {t} v _ {t}, \hat {\chi} _ {t _ {n}} ^ {+} = \exp \left(L _ {n} \left[ \hat {\chi} _ {t _ {n}} Y _ {n} ^ {1} + \binom{p _ {1}}{1};...; \hat {\chi} _ {t _ {n}} Y _ {n} ^ {K} + \binom{p _ {K}}{1} \right]\right) \hat {\chi} _ {t _ {n}}.
$$

As the bottom element of $\left[ \hat { \chi } _ { t _ { n } } ^ { - 1 } Y _ { n } ^ { k } + \binom { p _ { k } } { 1 } \right]$ is always zero we can conveniently use a reduceddimension gain matrix ${ \tilde { L } } _ { n }$ defined by $L _ { n } = { \tilde { L } } _ { n } { \tilde { p } }$ with $\tilde { p } = \left( \begin{array} { c c c } { { \left[ I _ { 2 } , 0 _ { 2 , 1 } \right] } } & { { } } & { { } } \\ { { } } & { { \ddots } } & { { } } \\ { { } } & { { } } & { { \left[ I _ { 2 } , 0 _ { 2 , 1 } \right] } } \end{array} \right)$ . To compute the gains ${ \tilde { L } } _ { n }$ we derive the evolution of the right-invariant error variable $\eta _ { t } = \hat { \chi } _ { t } \chi _ { t } ^ { - 1 }$ between the estimate and the state of the associated “noisy” system:

$$
\begin{array}{l} \frac {d}{d t} \eta_ {t} = - (\hat {\chi} _ {t} w _ {t} \hat {\chi} _ {t} ^ {- 1}) \eta_ {t}, \\ \eta_ {t _ {n}} ^ {+} = \exp \left(\tilde {L} _ {n} \tilde {p} \left( \begin{array}{c} - \eta_ {t _ {n}} \binom{p _ {1}}{1} + \binom{p _ {1}}{1} + \hat {\chi} _ {t _ {n}} \binom{V _ {n} ^ {1}}{0} \\ \dots \\ - \eta_ {t _ {n}} \binom{p _ {K}}{1} + \binom{p _ {K}}{1} + \hat {\chi} _ {t _ {n}} \binom{V _ {n} ^ {K}}{0} \end{array} \right)\right) \eta_ {t _ {n}}. \end{array}\tag{48}
$$

To linearize this equation we introduce the linearized error $\xi _ { t }$ defined as $\eta _ { t } = I _ { 3 } + \mathcal { L } _ { \mathfrak { s e } ( 2 ) } ( \xi _ { t } )$ Introducing $\eta _ { t } = I _ { 3 } + \mathcal { L } _ { \mathfrak { s e } ( 2 ) } ( \xi _ { t } ) , \eta _ { t } ^ { + } = I _ { 3 } + \mathcal { L } _ { \mathfrak { s e } ( 2 ) } ( \xi _ { t } ^ { + } ) , \exp ( u ) = I _ { 3 } + \mathcal { L } _ { \mathfrak { s e } ( 2 ) } ( u )$ and $\eta _ { t } ^ { - 1 } =$ $I _ { 3 } - \mathcal { L } _ { \mathfrak { s e } ( 2 ) } ( \xi _ { t } )$ in (48) and removing the second-order terms in $\xi _ { t } , V _ { n }$ and $w _ { t }$ we obtain:

$$
\begin{array}{l} \frac {d}{d t} \xi_ {t} = - \left( \begin{array}{c c} 1 & 0 _ {1, 2} \\ \hat {x} _ {t} ^ {2} & R (\hat {\theta} _ {t}) \\ - \hat {x} _ {t} ^ {1} & \end{array} \right) \left( \begin{array}{c} w _ {t} ^ {\theta} \\ w _ {t} ^ {l} \\ w _ {t} ^ {t r} \end{array} \right), \\ \xi_ {t _ {n}} ^ {+} = \xi_ {t _ {n}} - \tilde {L} _ {n} \left[ \left( \begin{array}{c c c} (- (p _ {1}) _ {2} & 1 & 0) \\ (p _ {1}) _ {1} & 0 & 1) \\ ... & & \\ (- (p _ {K}) _ {2} & 1 & 0) \\ (p _ {K}) _ {1} & 0 & 1) \end{array} \right) \xi_ {t _ {n}} - \left( \begin{array}{c} R (\hat {\theta} _ {t _ {n}}) V _ {n} ^ {1} \\ ... \\ R (\hat {\theta} _ {t _ {n}}) V _ {n} ^ {K} \end{array} \right) \right]. \end{array}
$$

The gains are thus computed using the Riccati equation (37) with $A _ { t } , H , \hat { Q }$ and $\hat { N }$ defined as:

$$
A _ {t} = 0 _ {3, 3}, \quad H _ {n} = \left( \begin{array}{c c c} \left( \begin{array}{c c c} - (p _ {1}) _ {2} & 1 & 0 \\ (p _ {1}) _ {1} & 0 & 1 \end{array} \right) \\ ... \\ \left( \begin{array}{c c c} - (p _ {K}) _ {2} & 1 & 0 \\ (p _ {K}) _ {1} & 0 & 1 \end{array} \right) \end{array} \right),
$$

$$
\hat {Q} _ {t} = \left( \begin{array}{c c} 1 & 0 _ {1, 2} \\ \hat {x} _ {t} ^ {2} & R (\hat {\boldsymbol {\theta}} _ {t}) \\ - \hat {x} _ {t} ^ {1} \end{array} \right) C o v \left( \begin{array}{c} w _ {t} ^ {\theta} \\ w _ {t} ^ {l} \\ w _ {t} ^ {t r} \end{array} \right) \left( \begin{array}{c c} 1 & 0 _ {1, 2} \\ \hat {x} _ {t} ^ {2} & R (\hat {\boldsymbol {\theta}} _ {t}) \\ - \hat {x} _ {t} ^ {1} \end{array} \right) ^ {T},
$$

$$
\hat {N} _ {n} = \left( \begin{array}{c c c} R (\hat {\theta} _ {t _ {n}}) C o v (N ^ {1}) R (\hat {\theta} _ {t _ {n}}) ^ {T} & & 0 \\ & \ddots & \\ 0 & & R (\hat {\theta} _ {t _ {n}}) C o v (N ^ {K}) R (\hat {\theta} _ {t _ {n}}) ^ {T} \end{array} \right).
$$

## 4.3 Stability properties of the IEKF viewed as a non-linear observer for the simplified car

## 4.3.1 Stability of the IEKF for the left-invariant output (39)

Proposition 3. Ifthere exists $\nu _ { \operatorname* { m a x } } , \nu _ { \operatorname* { m i n } } > 0$ such that the displacement satisfies $\lvert \lvert x _ { t _ { n + 1 } } - x _ { t _ { n } } \rvert \rvert \geq$ $\nu _ { \mathrm { { m i n } } } > 0$ and the input velocity satisfies $u _ { t } \le \nu _ { \operatorname* { m a x } }$ then the LIEKF derived at Section 4.2.4 is an asymptotically stable observer in the sense of Definition 2 about any trajectory.

The proof is a verification of the hypotheses of Theorem 4 and has been moved to $\mathbf { A p } \mathbf { \cdot }$ pendix D. Note that it seems very difficult to improve on the assumptions: if the car is at the same place each time its position is measured, the heading $\theta _ { t }$ becomes unobservable, and in practice an arbitrary high velocity is unfeasible.

## 4.3.2 Stability of the IEKF for the right-invariant output (40)

Proposition 4. Ifat least two distinct points are observed then the IEKF is an asymptotically stable observer in the sense ofDefinition 2 about any bounded trajectory.

Proof. According to Theorem 5 it is sufficient to show that in this case the observation matrix H is full-rank, i.e. of rank 3. This is obvious as the position and the heading are easily computed from the observation of two vectors at known locations. □

## 4.4 Simulations

The IEKF described in Section 4.2 has been implemented and compared to a classical EKF for the experimental setting described by Figure 1. The car drives along a 10-meter diameter circle for 40 seconds with high rate odometer measurements (100 Hz) and low rate GPS measurements (1 Hz).

The equations of the IEKF can be found above. The conventional EKF is based on the linear error $e _ { t } = \binom { \theta _ { t } - \hat { \theta } _ { t } } { x _ { t } ^ { 1 } - \hat { x } _ { t } ^ { 1 } }$ yielding the linearized matrices $F _ { t } = \left( \begin{array} { c c c } { 0 } & { 0 } & { 0 } \\ { - \sin ( \hat { \theta } _ { t } ) \nu _ { t } } & { 0 } & { 0 } \\ { \cos ( \hat { \theta } _ { t } ) \nu _ { t } } & { 0 } & { 0 } \end{array} \right)$ and $H _ { n } = { \binom { 0 1 0 } { 0 0 1 } }$ . Both filters are tuned with the same design parameters (which can be interpreted as odometer and GPS noise covariances) $N = I _ { 2 }$ , and $Q = d i a g ( ( \pi / 1 8 0 ) ^ { 2 } , 1 0 ^ { - 4 } , 1 0 ^ { - 4 } )$ i.e. moderate angular velocity uncertainty and highly precise linear velocity. The simulation is performed for two initial values of the heading error: $1 ^ { \circ }$ and $4 5 ^ { \circ }$ while the initial position is always assumed known. The covariance matrix $P _ { 0 }$ is consistent with the initial error (it encodes a standard deviation of the heading of $1 ^ { \circ }$ and $4 5 ^ { \circ }$ respectively).

The results are displayed on Figure 1. We see that for small initial errors both filters behave similarly for a long time, but for larger errors they soon behave differently, and we see the IEKF, whose design has been adapted to the specific structure of the system, completely outperforms the EKF.

## 5 Navigation on flat earth

In this example we estimate the orientation, velocity, and position of a rigid body in space from inertial sensors and relative observations of points having known locations (the setting of [26] but with the state including the position). To our knowledge, this is the first time the invariant observer on Lie groups based approach is applied to this full navigation problem with landmarks, except for our preliminary conference paper [4]. Indeed, this example does not fit into the usual framework leading to autonomous errors (unless we discard the position estimate as in [26]) but thanks to Theorem 1 we see it still leads to an autonomous error equation. This allows the IEKF observer to possess provable convergence properties. Note the problem at hand is different from the navigation problems using magnetometers, and velocity and position measurements of the GPS [17, 16].

Of course, the EKF, to be more precise its appropriate variant the multiplicative (M)EKF [21], is the state of the (industrial) art for this navigation example, due to its good performances, and easy tuning based on sensors’ noise covariances. But to our best knowledge it is nowhere proved to possess stability properties as a non-linear observer, and simulations below even indicate it may diverge in some situations whereas the IEKF converges. The computations require basic formulas recalled in Appendix A.

## 5.1 Considered model

We consider here the more complicated model of a vehicle evolving in the 3D space and characterized by its attitude $R _ { t }$ , velocity $\nu _ { t }$ and position $x _ { t }$ . The vehicle is endowed with accelerometers and gyroscopes whose measures are denoted respectively by $u _ { t }$ and angular velocity $\omega _ { t }$ . The dynamics read:

$$
\frac {d}{d t} R _ {t} = R _ {t} \left(\omega_ {t}\right) _ {\times}, \quad \frac {d}{d t} v _ {t} = g + R _ {t} u _ {t}, \quad \frac {d}{d t} x _ {t} = v _ {t},\tag{49}
$$

where $( \omega ) _ { \times }$ denotes the $3 \times 3$ skew-symmetric matrix associated with the cross product with $\omega ,$ , that is, for any $b \in \mathbb { R } ^ { 3 }$ we have $( \pmb { \omega } ) _ { \times } b = \pmb { \omega } \times b$ . Observations of the relative position of known features (using for instance a depth camera) are considered:

$$
(Y _ {n} ^ {1}, \dots , Y _ {n} ^ {k}) = \left(R _ {t _ {n}} ^ {T} (p _ {1} - x _ {t _ {n}}), \dots , R _ {t _ {n}} ^ {T} (p _ {k} - x _ {t _ {n}})\right),\tag{50}
$$

where $\left( p _ { 1 } , . . . , p _ { k } \right)$ denote the (assumed known) position of the features in the earth-fixed frame.

## 5.2 IEKF gain tuning

To derive and tune the IEKF equations, we follow the methodology of Section 3.2 which amounts to 1- associate a “noisy ”system to the original considered system, just because it allows obtaining a sensible tuning of the design matrices from an engineering viewpoint, 2- transform it into a system defined on a matrix Lie group to make it fit into our framework, 3- linearize the “noisy” equations, and 4- use the Kalman equations to tune the observer gain.

## 5.2.1 Associated “noisy” system

By merely introducing noise in the accelerometers’ and gyrometers’ measurements we obtain the well-known equations [14]:

$$
\frac {d}{d t} R _ {t} = R _ {t} \left(\omega_ {t} + w _ {t} ^ {\omega}\right) _ {\times}, \quad \frac {d}{d t} v _ {t} = g + R _ {t} \left(u _ {t} + w _ {t} ^ {u}\right), \quad \frac {d}{d t} x _ {t} = v _ {t}.\tag{51}
$$

Letting additive noise pollute the observations (the sensor being in the body frame) we get:

$$
(Y _ {n} ^ {1}, \dots , Y _ {n} ^ {k}) = \left(R _ {t _ {n}} ^ {T} (p _ {1} - x _ {t _ {n}}) + V _ {n} ^ {1}, \dots , R _ {t _ {n}} ^ {T} (p _ {k} - x _ {t _ {n}}) + V _ {n} ^ {k}\right),\tag{52}
$$

where $V _ { n } , V _ { n } ^ { 1 } , . . . , V _ { n } ^ { k }$ are noises in $\mathbb { R } ^ { 3 }$ .

## 5.2.2 Matrix form

As already noticed in the preliminary work [4], the system (51) can be embedded in the group of double homogeneous matrices (see Appendix A.2) using the matrices $\chi _ { t } , w _ { t }$ and function $f _ { \omega , u } \colon$

$$
\chi_ {t} = \left( \begin{array}{c c c} R _ {t} & v _ {t} & x _ {t} \\ 0 _ {3, 1} & 1 & 0 \\ 0 _ {3, 1} & 0 & 1 \end{array} \right), \qquad w _ {t} = \left( \begin{array}{c c c} (w _ {t} ^ {\omega}) _ {\times} & w _ {t} ^ {u} & 0 _ {3, 1} \\ 0 _ {1, 3} & 0 & 0 \\ 0 _ {1, 3} & 0 & 0 \end{array} \right),
$$

$$
f _ {\omega , u}: \left( \begin{array}{c c c} R & v & x \\ 0 _ {3, 1} & 1 & 0 \\ 0 _ {3, 1} & 0 & 1 \end{array} \right) \to \left( \begin{array}{c c c} R (\omega) _ {\times} & g + R u & v \\ 0 _ {3, 1} & 0 & 0 \\ 0 _ {3, 1} & 0 & 0 \end{array} \right).
$$

The equation of the dynamics becomes:

$$
\frac {d}{d t} \chi_ {t} = f _ {\omega_ {t}, u _ {t}} (\chi_ {t}) + \chi_ {t} w _ {t},\tag{53}
$$

and the observations (52) have the equivalent forms:

$$
(Y _ {n} ^ {1},..., Y _ {n} ^ {k}) = \left(\chi_ {t _ {n}} ^ {- 1} \left( \begin{array}{c} p _ {1} \\ 0 \\ 1 \end{array} \right) + \left( \begin{array}{c} V _ {n} ^ {1} \\ 0 \\ 0 \end{array} \right),..., \chi_ {t _ {n}} ^ {- 1} \left( \begin{array}{c} p _ {k} \\ 0 \\ 1 \end{array} \right) + \left( \begin{array}{c} V _ {n} ^ {k} \\ 0 \\ 0 \end{array} \right)\right).\tag{54}
$$

Proposition 5. The matricial function $f _ { \omega _ { t } , u _ { t } }$ is neither left nor right invariant. However the reader can verify relation (7) which is easy to derive.

## 5.2.3 IEKF equations

The RIEKF (36) for the associated “noisy” system (53), with right-invariant “noisy” observations (54) reads:

$$
\frac {d}{d t} \hat {\chi} _ {t} = f _ {\omega_ {t}, u _ {t}} (\hat {\chi} _ {t}) \qquad , \qquad \hat {\chi} _ {t} ^ {+} = \exp \left(L _ {n} \left( \begin{array}{c} \hat {\chi} _ {t} Y _ {n} ^ {1} \\ ... \\ \hat {\chi} _ {t} Y _ {n} ^ {k} \end{array} \right)\right) \hat {\chi} _ {t}.
$$

As the two last entries of each matrix $\hat { \chi } _ { t } ^ { - 1 } Y _ { n } ^ { j }$ are always zero, one we can conveniently use a reduced-dimension gain matrix ${ \tilde { L } } _ { n }$ defined by $L _ { n } = { \tilde { L } } _ { n } { \tilde { p } }$ with $\tilde { p } = \left( \begin{array} { c c c } { { \lbrack I _ { 3 } , 0 _ { 3 , 2 } \rbrack } } & { { } } & { { } } \\ { { } } & { { \ddots } } & { { } } \\ { { } } & { { } } & { { \lbrack I _ { 3 } , 0 _ { 3 , 2 } \rbrack } } \end{array} \right)$ The right-invariant error is $\eta _ { t } = \hat { \chi } _ { t } \chi _ { t } ^ { - 1 }$ and its evolution reads:

$$
\frac {d}{d t} \eta_ {t} = f _ {\omega_ {t}, u _ {t}} (\eta_ {t}) - \eta_ {t} f _ {\omega_ {t}, u _ {t}} (I _ {5}) - (\hat {\chi} _ {t} w _ {t} \hat {\chi} ^ {- 1}) \eta_ {t},\tag{55}
$$

$$
\eta_ {t _ {n}} ^ {+} = \exp \left(\tilde {L} _ {n} \tilde {p} \left( \begin{array}{c} \eta_ {t _ {n}} \binom{p _ {1}}{0} + \hat {\chi} _ {t _ {n}} \binom{V _ {n} ^ {1}}{0 _ {2, 1}} \\ \dots \\ \eta_ {t _ {n}} \binom{p _ {k}}{0} + \hat {\chi} _ {t _ {n}} \binom{V _ {n} ^ {k}}{0 _ {2, 1}} \end{array} \right)\right) \eta_ {t _ {n}}.\tag{56}
$$

To linearize this equation we introduce the linearized error $\xi _ { t }$ by replacing $\eta _ { t }$ with ${ \cal I } _ { 3 } +$ $\mathcal { L } _ { \mathfrak { s e } ( 3 ) , 2 } ( \xi _ { t } )$ . Using the first order approximations $\eta _ { t } = I _ { 3 } + \mathcal { L } _ { \mathfrak { s e } ( 3 ) , 2 } ( \xi _ { t } ) , \eta _ { t } ^ { + } = I _ { 3 } + \mathcal { L } _ { \mathfrak { s e } ( 3 ) , 2 } ( \xi _ { t } ^ { + } )$ e $\mathrm { x p } ( u ) = I _ { 3 } + \mathcal { L } _ { \mathfrak { s e } ( 3 ) , 2 } ( u )$ and $\eta _ { t } ^ { - 1 } = I _ { 3 } - \mathcal { L } _ { \mathfrak { s e } ( 3 ) , 2 } ( \xi _ { t } )$ in (55), (56) and removing the secondorder terms in $\xi _ { t } , \dot { V _ { n } }$ and $w _ { t }$ we obtain:

$$
\begin{array}{l} \frac {d}{d t} \xi_ {t} = \left( \begin{array}{c c c} 0 _ {3, 3} & 0 _ {3, 3} & 0 _ {3, 3} \\ (g) _ {\times} & 0 _ {3, 3} & 0 _ {3, 3} \\ 0 _ {3, 3} & I _ {3} & 0 _ {3, 3} \end{array} \right) \xi_ {t} - \left( \begin{array}{c c c} \hat {R} _ {t} & 0 _ {3, 3} & 0 _ {3, 3} \\ (\hat {v} _ {t}) _ {\times} \hat {R} _ {t} & \hat {R} _ {t} & 0 _ {3, 3} \\ (\hat {x} _ {t}) _ {\times} \hat {R} _ {t} & 0 _ {3, 3} & \hat {R} _ {t} \end{array} \right) w _ {t}, \\ \xi_ {t _ {n}} ^ {+} = \xi_ {t _ {n}} - \tilde {L} _ {n} \left[ \left( \begin{array}{c c c} (p _ {1}) _ {\times} & 0 _ {3, 3} & - I _ {3} \\ & \ldots \\ (p _ {k}) _ {\times} & 0 _ {3, 3} & - I _ {3} \end{array} \right) \xi_ {t _ {n}} - \left( \begin{array}{c} \hat {R} _ {t _ {n}} V _ {n} ^ {1} \\ \ldots \\ \hat {R} _ {t _ {n}} V _ {n} ^ {k} \end{array} \right) \right]. \end{array}
$$

The gains ${ \tilde { L } } _ { n }$ are computed using the Riccati equation (37) and matrices $A _ { t } , H , \hat { Q }$ and $\hat { N }$ defined as:

$$
A _ {t} = \left( \begin{array}{c c c} 0 _ {3, 3} & 0 _ {3, 3} & 0 _ {3, 3} \\ (g) _ {\times} & 0 _ {3, 3} & 0 _ {3, 3} \\ 0 _ {3, 3} & I _ {3} & 0 _ {3, 3} \end{array} \right), \qquad H = \left( \begin{array}{c c c} (p _ {1}) _ {\times} & 0 _ {3, 3} & - I _ {3} \\ & \ldots \\ (p _ {k}) _ {\times} & 0 _ {3, 3} & - I _ {3} \end{array} \right),
$$

$$
\hat {Q} = \left( \begin{array}{c c c} \hat {R} _ {t} & 0 _ {3, 3} & 0 _ {3, 3} \\ (\hat {v} _ {t}) _ {\times} \hat {R} _ {t} & \hat {R} _ {t} & 0 _ {3, 3} \\ (\hat {x} _ {t}) _ {\times} \hat {R} _ {t} & 0 _ {3, 3} & \hat {R} _ {t} \end{array} \right) C o v (w _ {t}) \left( \begin{array}{c c c} \hat {R} _ {t} & 0 _ {3, 3} & 0 _ {3, 3} \\ (\hat {v} _ {t}) _ {\times} \hat {R} _ {t} & \hat {R} _ {t} & 0 _ {3, 3} \\ (\hat {x} _ {t}) _ {\times} \dot {\hat {R}} _ {t} & 0 _ {3, 3} & \hat {R} _ {t} \end{array} \right) ^ {T},
$$

$$
\hat {N} = \left( \begin{array}{c c c} \hat {R} _ {t _ {n}} C o v (V _ {n} ^ {1}) \hat {R} _ {t _ {n}} ^ {T} & & \\ & \ddots & \\ & & \hat {R} _ {t _ {n}} C o v (V _ {n} ^ {k}) \hat {R} _ {t _ {n}} ^ {T} \end{array} \right).
$$

## 5.3 Stability properties of the IEKF viewed as a non-linear observer for the navigation example

Theorem 6. If three non-collinear points are observed, then the IEKF whose equations are derived in Section 5.2.3 is an asymptotically stable observer for the system (49)-(50) in the sense of Definition 2 about any bounded trajectory.

Proof. According to Theorem 5 we only have to ensure the couple (A,H) is observable. Integrating the propagation on one step we obtain the discrete propagation matrix

$$
\Phi = \left( \begin{array}{c c c} I _ {3} & 0 _ {3 \times 3} & 0 _ {3 \times 3} \\ t (g) _ {\times} & I _ {3} & 0 _ {3 \times 3} \\ \frac {1}{2} t ^ {2} (g) _ {\times} & t I d _ {3 \times 3} & I _ {3} \end{array} \right).
$$

The observation matrix is denoted H. We will show that $[ H ; H \Phi ]$ has rank 9. We can keep only the raws corresponding to the observation of three non-collinear features $p _ { 1 } , p _ { 2 } , p _ { 3 }$ and denote the remaining matrix by $\mathcal { H } _ { 1 }$ . Matrices $\mathcal { H } _ { 2 }$ and $\mathcal { H } _ { 3 }$ , obtained using elementary operations on the columns of $\mathcal { H } _ { 1 }$ , have a rank inferior or equal to the rank of $\mathcal { H } _ { 1 }$

$$
\mathcal {H} _ {1} = \left( \begin{array}{c c c} (p _ {1}) _ {\times} & 0 _ {3 \times 3} & - I _ {3} \\ (p _ {2}) _ {\times} & 0 _ {3 \times 3} & - I _ {3} \\ (p _ {3}) _ {\times} & 0 _ {3 \times 3} & - I _ {3} \\ (p _ {1}) _ {\times} - \frac {1}{2} t ^ {2} (g) _ {\times} & - t I _ {3} & - I _ {3} \\ (p _ {2}) _ {\times} - \frac {1}{2} t ^ {2} (g) _ {\times} & - t I _ {3} & - I _ {3} \\ (p _ {3}) _ {\times} - \frac {1}{2} t ^ {2} (g) _ {\times} & - t I _ {3} & - I _ {3} \end{array} \right), \qquad \mathcal {H} _ {2} = \left( \begin{array}{c c c} (p _ {1}) _ {\times} & 0 _ {3 \times 3} & - I _ {3} \\ (p _ {2}) _ {\times} & 0 _ {3 \times 3} & - I _ {3} \\ (p _ {3}) _ {\times} & 0 _ {3 \times 3} & - I _ {3}. \\ - \frac {1}{2} t ^ {2} (g) _ {\times} & - t I _ {3} & 0 _ {3 \times 3} \\ - \frac {1}{2} t ^ {2} (g) _ {\times} & - t I _ {3} & 0 _ {3 \times 3} \\ - \frac {1}{2} t ^ {2} (g) _ {\times} & - t I _ {3} & 0 _ {3 \times 3} \end{array} \right),
$$

$$
\mathcal {H} _ {3} = \left( \begin{array}{c c c} (p _ {1} - p _ {3}) _ {\times} & 0 _ {3 \times 3} & 0 _ {3 \times 3} \\ (p _ {2} - p _ {3}) _ {\times} & 0 _ {3 \times 3} & 0 _ {3 \times 3} \\ \frac {1}{2} t ^ {2} (g) _ {\times} & t I _ {3} & 0 _ {3 \times 3} \\ (p _ {3}) _ {\times} & 0 _ {3 \times 3} & - I _ {3} \end{array} \right).
$$

The diagonal blocks $\left( { \begin{array} { c c } { - ( p _ { 1 } - p _ { 3 } ) _ { \times } } & { 0 _ { 3 \times 3 } } \\ { - ( p _ { 2 } - p _ { 3 } ) _ { \times } } & { 0 _ { 3 \times 3 } } \end{array} } \right)$ $t I _ { 3 }$ and $I _ { 3 }$ have rank 3 thus the full matrix has rank 9. □

## 5.4 Simulations

The IEKF described in Section 5.2.3 has been implemented and compared to a state of the art multiplicative EKF [21] for the experimental setting described by Figure 2 (top plots). The vehicle drives a 10-meter diameter circle (green arrows) in 30 seconds and observes three features (black circles) every second while receiving high-frequency inertial measurements (100 Hz). The equations of the IEKF have already been detailed. The error variable to be linearized for the multiplicative (M)EKF is $e _ { t } = \big ( \hat { R } _ { t } R _ { t } ^ { - 1 } , \hat { \nu } _ { t } - \nu _ { t } , \hat { x } _ { t } - x _ { t } \big )$ . As $\hat { R } _ { t } R _ { t } ^ { - 1 }$ is not a vector variable, it is linearized using the first-order expansion $\hat { R } _ { t } R _ { t } ^ { - 1 } = I _ { 3 } + \left( \zeta _ { t } \right) _ { \times } , \zeta _ { t } \in \mathbb { R } ^ { 3 } \left[ 2 1 \right]$ The linearized error variable is thus a vector $\pmb { \varepsilon } _ { t } = ( \zeta , d x , d \nu ) \in \mathbb { R } ^ { 9 }$ . Expanding the propagation and observation steps up to the first order in $\varepsilon _ { t }$ give the classical $F _ { t }$ and $H _ { n }$ matrices used in the Riccati Equation of the MEKF:

$$
F _ {t} = \left( \begin{array}{c c c} 0 _ {3} & 0 _ {3} & 0 _ {3} \\ - \left(\hat {R} _ {t} u _ {t}\right) _ {\times} & 0 _ {3} & 0 _ {3} \\ 0 _ {3} & I _ {3} & 0 _ {3} \end{array} \right),
$$

$$
H _ {n} = \left( \begin{array}{c c c c} - \hat {R} _ {t _ {n}} ^ {T} \left(p ^ {1} - \hat {x} _ {t _ {n}}\right) _ {\times} & 0 _ {3} & & \hat {R} _ {t _ {n}} ^ {T} \\ & \dots & & \\ - \hat {R} _ {t _ {n}} ^ {T} \left(p ^ {k} - \hat {x} _ {t _ {n}}\right) _ {\times} & 0 _ {3} & & \hat {R} _ {t _ {n}} ^ {T} \end{array} \right).
$$

We use the following design parameters in two distinct simulations, with same N but two different matrices $Q .$

$$
N = \left( \begin{array}{c c c} 1 0 ^ {- 2} I _ {3} & 0 _ {3} & 0 _ {3} \\ 0 _ {3} & 1 0 ^ {- 2} I _ {3} & 0 _ {3} \\ 0 _ {3} & 0 _ {3} & 1 0 ^ {- 2} I _ {3} \end{array} \right),
$$

$$
Q _ {1} = \left( \begin{array}{c c c} 1 0 ^ {- 8} I _ {3} & 0 _ {3} & 0 _ {3} \\ 0 _ {3} & 1 0 ^ {- 8} I _ {3} & 0 _ {3} \\ 0 _ {3} & 0 _ {3} & 0 _ {3} \end{array} \right), \quad Q _ {2} = \left( \begin{array}{c c c} 1 0 ^ {- 4} I _ {3} & 0 _ {3} & 0 _ {3} \\ 0 _ {3} & 1 0 ^ {- 4} I _ {3} & 0 _ {3} \\ 0 _ {3} & 0 _ {3} & 0 _ {3} \end{array} \right).
$$

The initial errors are the same for both simulations: 15 degrees for attitude and 1 meter for position standard deviations. The small “process noise” matrix $Q _ { 1 }$ , although reasonable in the context of high-precision inertial navigation, has been deliberately chosen to challenge EKFlike methods: the corresponding gains are small so the errors introduced during the transitory phase due to non-linearities in the initial errors can never be corrected. Note this would not be an issue if the system was linear: the estimation errors and filter gains would decrease simultaneously. The problem is that the error does not decrease as fast as predicted by the linear Kalman theory. As shown by the plots of the left column of Figure 2 (top plots) it makes the EKF even diverge! This is probably the simplest way to make the EKF fail in a navigation problem and this is purely a problem of non-linearity as no noise has been added whatsoever. Still on the left column, we see the IEKF is not affected by the problem, due to its appropriate non-linear structure. In particular, the attitude and position errors go to zero in accordance with Theorem 6.

Usually, engineers get around those convergence problems by artificially inflating the “process noise” matrix $Q$ (see also [24]). This classical solution, sometimes referred to as robust tuning, is illustrated here by using $Q _ { 2 }$ instead. The results are displayed on the right column of Fig. 2. They illustrate the fact the EKF, as an observer, can be improved through a proper tuning, although still much slower to converge that the IEKF. But this raises issues: $Q$ and N have been chosen for a specific trajectory with no guarantee regarding robustness. Moreover, these matrices admit a physical interpretation (the accuracy of the sensors) and arbitrarily changing them by several orders of magnitude is a renouncement to use this precious information when available. In turn, this makes the matrix $P _ { t }$ loose its interpretability as an indication of the observer’s accuracy in response to the sensors’ trusted accuracy. For this relevant problem, we thus see the IEKF turns out to be a viable alternative to the EKF thanks to its guaranteed properties, and to its convincing experimental behavior reflecting way better performances than the EKF, even for challenging choices of Q and N.

## 6 Conclusion

The Invariant EKF, when used as a deterministic observer for an introduced and well characterized class of problems on Lie groups, is shown to possess theoretical stability guarantees under the simple and natural hypotheses of the linear case, a feature the EKF has never been proved to share so far. Simulations confirm the IEKF is an appealing alternative indeed, as it is always superior to the EKF and outperforms it in challenging situations, while remaining similar to EKF in terms of tuning, implementation, and computational load.

## A Matrix Lie groups useful formulas

A matrix Lie group G is a set of square invertible matrices of size $N \times N$ verifying the following properties:

$$
I d \in G, \quad \forall g \in G, g ^ {- 1} \in G, \quad \forall a, b \in G, a b \in G
$$

If $\gamma ( t )$ is a process taking values in G and verifying $\gamma ( 0 ) = I d$ , then its derivative at $t = 0$ cannot take any value in the set of squared matrices $\mathcal { M } _ { N } ( \mathbb { R } )$ . It is constrained to lie in a vector subspace g of ${ \mathcal { M } } _ { N } ( \mathbb { R } )$ called the “Lie algebra of $G ^ { \ast }$ . As it proves useful to identify g to $\mathbb { R } ^ { \mathrm { d i m } \mathfrak { g } }$ a linear mapping $\mathcal { L } _ { \mathfrak { g } } : \mathbb { R } ^ { \dim { \mathfrak { g } } } \to { \mathfrak { g } }$ is used in this paper. The vector space g can be mapped to the matrix Lie group G through the classical matrix exponential $\exp _ { m }$ . As well, $\mathbb { R } ^ { \mathrm { d i m } \mathfrak { g } }$ can be mapped to G through a function exp defined by ex $) ( \xi ) = \exp _ { m } ( \mathcal { L } _ { \mathfrak { g } } ( \xi ) )$ . For any $g \in G$ the operator $A d _ { g } : \mathbb { R } ^ { \dim { \mathfrak { g } } }  \mathbb { R } ^ { \dim { \mathfrak { g } } }$ is defined by $g \mathcal { L } _ { \mathfrak { g } } ( \xi ) g ^ { - 1 } = \mathcal { L } _ { \mathfrak { g } } ( A d _ { g } \xi )$ . For any $\boldsymbol { x } \in \mathbb { R } ^ { \dim { \mathfrak { g } } }$ the operator $A d _ { x } : \mathbb { R } ^ { \mathrm { d i m g } }  \mathbb { R } ^ { \mathrm { d i m g } }$ is defined by $\mathcal { L } _ { \mathfrak { g } } ( \bar { x } ) \mathcal { L } _ { \mathfrak { g } } ( \xi ) - \bar { \mathcal { L } } _ { \mathfrak { g } } ( \xi ) \bar { \mathcal { L } } _ { \mathfrak { g } } ( x ) = \mathcal { L } _ { \mathfrak { g } } ( A d _ { x } \xi )$ . These operators are very handy in practical computations. For all matrix Lie groups considered in this paper, no matrix exponentiation is actually needed as there exist closed formulas, given thereafter. We give now a short description of the matrix Lie groups appearing in the present paper.

## A.1 Group of direct planar isometries $S E ( 2 )$

We have here $G = S E ( 2 )$ and ${ \mathfrak { g } } = { \mathfrak { s e } } ( 2 )$ , where

$$
S E (2) = \left\{\left( \begin{array}{c c} R (\boldsymbol {\theta}) & x \\ 0 _ {1, 2} & 1 \end{array} \right), \binom{\boldsymbol {\theta}}{x} \in \mathbb {R} ^ {3} \right\},
$$

$$
\mathfrak {s e} (2) = \left\{\left( \begin{array}{c c c} 0 & - \theta & u _ {1} \\ \theta & 0 & u _ {2} \\ 0 & 0 & 0 \end{array} \right), \left( \begin{array}{c} \theta \\ u _ {1} \\ u _ {2} \end{array} \right) \in \mathbb {R} ^ {3} \right\},
$$

$$
\mathcal {L} _ {\mathfrak {s c} (2)} \left( \begin{array}{c} \theta \\ u _ {1} \\ u _ {2} \end{array} \right) = \left( \begin{array}{c c c} 0 & - \theta & u _ {1} \\ \theta & 0 & u _ {2} \\ 0 & 0 & 0 \end{array} \right),
$$

where $R ( \theta ) = { \binom { \cos ( \theta ) } { \sin ( \theta ) } } \quad \cos ( \theta ) \quad \quad$ denotes the rotation of angle $\theta ,$ and the exponential mapping is:

$$
\exp \left((\theta , u _ {1}, u _ {2}) ^ {T}\right) = \left( \begin{array}{c c} R (\theta) & x \\ 0 _ {1, 2} & 1 \end{array} \right),
$$

where

$$
x = \left( \begin{array}{c c} \frac {\sin (\theta)}{\theta} & - \frac {1 - \cos (\theta)}{\theta} \\ \frac {1 - \cos (\theta)}{\theta} & \frac {\sin (\theta)}{\theta} \end{array} \right).
$$

## A.2 Group of double direct spatial isometries $S E _ { 2 } ( 3 )$

We have here:

$$
G = S E _ {2} (3) = \left\{\left( \begin{array}{c c c} R & v & x \\ 0 _ {1, 3} & 1 & 0 \\ 0 _ {1, 3} & 0 & 1 \end{array} \right), R \in S O (3), v, x \in \mathbb {R} ^ {3} \right\},
$$

$$
\mathfrak {g} = \mathfrak {s e} _ {2} (3) = \left\{\left( \begin{array}{c c c} (\xi) _ {\times} & u & y \\ 0 _ {1, 3} & 0 & 0 \\ 0 _ {1, 3} & 0 & 0 \end{array} \right), \xi , u, y \in \mathbb {R} ^ {3} \right\}.
$$

An isomorphism between $\mathbb { R } ^ { 9 }$ and $\mathfrak { s e } _ { 2 } ( 3 )$ is given by $\mathcal { L } _ { \mathfrak { s } \mathfrak { e } _ { 2 } ( 3 ) } \left( \begin{array} { l } { \xi } \\ { u } \\ { y } \end{array} \right) = \left( \begin{array} { l l l } { ( \xi ) _ { \times } } & { u } & { y } \\ { 0 _ { 1 , 3 } } & { 0 } & { 0 } \\ { 0 _ { 1 , 3 } } & { 0 } & { 0 } \end{array} \right)$ . The exponential mapping is given by the formula:

$$
\begin{array}{l} \exp \left(\left(\xi^ {T}, u ^ {T}, y ^ {T}\right) ^ {T}\right) \\ = I _ {5} + S + \frac {1 - \cos (| | \xi | |)}{| | \xi | | ^ {2}}) S ^ {2} + \frac {| | \xi | | - \sin (| | \xi | |)}{| | \xi | | ^ {3}} S ^ {3}, \end{array}
$$

where $S = \mathcal { L } _ { \mathfrak { s e } _ { 2 } ( 3 ) } ( \xi , u , y ) ^ { T }$

## B Further explanation and proof of the log-linear property

The definition of $A _ { t }$ through $g _ { u _ { t } } ( \exp ( \xi ) ) = \mathcal { L } _ { \mathfrak { g } } ( A _ { u _ { t } } \xi ) + O ( \vert \vert \xi _ { t } \vert \vert ^ { 2 } )$ is very convenient in practice as shown in Sections 4 and 5, but it requires a quick theoretical explanation as follows. On an abstract Lie group, vector $g _ { u _ { t } } \left( \exp \left( \xi \right) \right)$ belongs to $T _ { \mathrm { e x p } ( \xi ) } G$ (tangent space at $\exp ( \xi ) )$ whereas the image of $\mathcal { L } _ { \mathfrak { g } }$ is ${ \mathfrak { g } } = T _ { I d } G$ . But as $g _ { u _ { t } } ( I d ) = \ddot { 0 }$ it is true that the first order terms of $g _ { u _ { t } } \left( \exp \left( \xi \right) \right)$ belong to ${ \mathfrak { g } } \subset \mathbb { R } ^ { N \times N }$ . Indeed as $\exp { ( \xi ) } ^ { - 1 } g ( \exp { ( \xi ) } ) \in \mathfrak { g }$ we have $g \left( \exp \left( \xi \right) \right) = \exp _ { m } ( \mathcal { L } _ { \mathfrak { g } } ( \xi ) ) \mathcal { L } _ { \mathfrak { g } } ( \omega ( \xi ) )$ for some function ω with $\omega ( 0 ) = 0$ . Thus $g \left( \exp { ( \xi ) } \right) = ( I d + \mathcal { L } _ { \mathfrak { g } } ( \xi ) ) \dot { \mathcal { L } } _ { \mathfrak { g } } ( \tilde { \omega } ( \xi ) ) + O ( \| \xi \| ^ { 2 } ) = \mathcal { L } _ { \mathfrak { g } } ( \omega ( \xi ) ) + O ( \| \xi \| ^ { 2 } )$

The proof of Theorem 2 is based upon the following lemmas.

Lemma 1. Consider the system (12) and let ${ \bar { \chi } } _ { t }$ denote a particular solution. Consider the condition

$$
\forall (u, a, b) \in G \quad g _ {u} (a b) = a g _ {u} (b) + g _ {u} (a) b\tag{57}
$$

We have thefollowing properties:

• Thefunction $g _ { u } ^ { R } ( \eta ) = f _ { u _ { t } } ( \eta ) - \eta f _ { u _ { t } } ( I d )$ verifies (57) and all the solutions of (12) have the form $\chi _ { t } = \eta _ { t } ^ { R } \bar { \chi } _ { t }$ , where $\eta _ { t } ^ { R }$ verifies $\begin{array} { r } { \frac { d } { d t } \eta _ { t } ^ { R } = g _ { u _ { t } } ^ { R } ( \eta _ { t } ^ { R } ) } \end{array}$

• The function $g _ { u _ { t } } ^ { L } ( \eta ) = f _ { u _ { t } } ( \eta ) - f _ { u _ { t } } ( I d ) \eta$ verifies (57) and all the solutions of (12) have the form $\chi _ { t } = \bar { \chi } _ { t } \eta _ { t } ^ { L }$ , where $\eta _ { t } ^ { L }$ verifies $\begin{array} { r } { \frac { d } { d t } \eta _ { t } ^ { L } = g _ { u _ { t } } ^ { L } ( \eta _ { t } ^ { L } ) } \end{array}$

The verification of these two properties is trivial. The functions $g _ { u _ { t } }$ governing the errors propagation turn out to possess an intriguing property.

Lemma 2. Let $\Phi _ { t }$ be the flow (that is the solution at time t associated to a given initial condition) associated to the system $\begin{array} { r } { \frac { d } { d t } \eta _ { t } = g _ { u _ { t } } ( \eta _ { t } ) } \end{array}$ , where $g _ { u _ { t } }$ verifies (57). Then:

$$
\forall \eta_ {0}, \eta_ {0} ^ {\prime} \in G, \Phi_ {t} (\eta_ {0} \eta_ {0} ^ {\prime}) = \Phi_ {t} (\eta_ {0}) \Phi_ {t} (\eta_ {0} ^ {\prime})
$$

Proof. We simply have to see that $\Phi _ { t } ( \eta _ { 0 } ) \Phi _ { t } ( \eta _ { 0 } ^ { \prime } )$ is solution of the system $\begin{array} { r } { \frac { d } { d t } \eta _ { t } = g _ { u _ { t } } ( \eta _ { t } ) } \end{array}$ :

$$
\begin{array}{c} \frac {d}{d t} [ \Phi_ {t} (\eta_ {0}) \Phi_ {t} (\eta_ {0} ^ {\prime}) ] = g _ {u _ {t}} (\Phi_ {t} (\eta_ {0})) \Phi_ {t} (\eta_ {0} ^ {\prime}) + \Phi_ {t} (\eta_ {0}) g _ {u _ {t}} (\Phi_ {t} (\eta_ {0} ^ {\prime})) \\ = g _ {u _ {t}} \left(\Phi_ {t} (\eta_ {0}) \Phi_ {t} (\eta_ {0} ^ {\prime})\right) \end{array}
$$

An immediate recursion gives then:

Lemma 3. We havefurthermore

$$
\forall \eta_ {0} \in G, \forall p \in \mathbb {Z}, \Phi_ {t} (\eta_ {0} ^ {p}) = \Phi_ {t} (\eta_ {0}) ^ {p}
$$

Lemmas 2 and 3 indicate the behavior of the flow infinitely close to Id dictates its behavior arbitrarily far from it, as the flow commutes with exponentiation. The use of the exponential thus allows deriving an infinitesimal version of the Lemma 3, which is an equivalent formulation of Theorem 2.

Theorem 7. Let $\Phi _ { t }$ be the flow associated to the system $\begin{array} { r } { \frac { d } { d t } \eta _ { t } = g _ { u _ { t } } ( \eta _ { t } ) } \end{array}$ satisfying (57). We have:

$$
\Phi_ {t} (\exp (\xi_ {0})) = \exp (F _ {t} \xi_ {0})
$$

where F<sub>t</sub> is the solution ofthe matrix equation $\begin{array} { r } { F _ { 0 } = I d , \frac { d } { d t } F _ { t } = A _ { t } F _ { t } } \end{array}$

Proof. Thanks to Lemma 3 we have, for any $n \in \mathbb { N } , \Phi _ { t } \left( e ^ { \xi _ { 0 } } \right) = \Phi _ { t } \left( \left[ e ^ { \frac { 1 } { n } \xi _ { 0 } } \right] ^ { n } \right) = \Phi _ { t } \left( e ^ { \frac { 1 } { n } \xi _ { 0 } } \right) ^ { n } =$ $\biggl [ e ^ { \frac { 1 } { n } F _ { t } \xi _ { 0 } + r _ { t } \left( \frac { 1 } { n } \xi _ { 0 } \right) } \biggr ] ^ { n }$ , where $F _ { t } = D \left( \exp ^ { - 1 } \circ \Phi _ { t } \circ \exp \right) \big | _ { 0 }$ and $\begin{array} { r } { r _ { t } \left( \frac { 1 } { n } \xi _ { 0 } \right) } \end{array}$ is a quadratic term, which ensures in turn Φ $\left( e ^ { \xi _ { 0 } } \right) = e ^ { F _ { t } \xi _ { 0 } + n r _ { t } \left( \frac { 1 } { n } \xi _ { 0 } \right) }$ . Letting n → ∞ we get Φ<sub>t</sub> $\biggl ( e ^ { \xi _ { 0 } } \biggr ) = e ^ { F _ { t } \xi _ { 0 } }$ . Differentiating both sides of the latter equality we obtain $\begin{array} { r } { g _ { u _ { t } } \left( \Phi _ { t } \left( e _ { 0 } ^ { \xi } \right) \right) = D \exp _ { F _ { t } \xi _ { 0 } } \left[ \frac { d } { d t } F _ { t } \xi _ { 0 } \right] } \end{array}$ . A first-order expansion in $\xi _ { 0 }$ using matrix $A _ { t }$ gives: $\mathcal { L } _ { \mathfrak { g } } \left( A _ { t } F _ { t } \xi _ { 0 } \right) + \circ \left( | | \xi _ { 0 } | | \right) = \mathcal { L } _ { \mathfrak { g } } \left( \frac { d } { d t } F _ { t } \xi _ { 0 } \right) + \circ \left( | | \xi _ { 0 } | | \right)$ , then $\begin{array} { r } { A _ { t } F _ { t } \xi _ { 0 } = \frac { d } { d t } F _ { t } \xi _ { 0 } } \end{array}$ for any $\boldsymbol { \xi } _ { 0 } \in \mathbb { R } ^ { \mathrm { d i m } \mathfrak { g } }$ and finally $\textstyle A _ { t } = { \frac { d } { d t } } F _ { t }$ □

## C Proof of theorem 4

## C.1 Proof rationale

We define the rest $r _ { n } ,$ , here for the left-invariant filter only, as follows: exp $[ ( I - L _ { n } H ) \xi _ { n } - r _ { n } ( \xi ) ]$ $= \exp [ \xi ] \exp [ - L _ { n } H ( \exp [ \xi ] b - b ) ]$ . We then introduce the flow $\Psi _ { t _ { 0 } } ^ { t }$ of the linear part of the equations governing $\xi _ { t }$ (that is, $\begin{array} { r } { \Psi _ { t _ { 0 } } ^ { t _ { 0 } } = I d , \quad \frac { d } { d t } \Psi _ { t _ { 0 } } ^ { t } = A _ { u _ { t } } \Psi _ { t _ { 0 } } ^ { t } , \quad \Psi _ { t _ { 0 } } ^ { t + } = ( I - L _ { n } H ) \Psi _ { t _ { 0 } } ^ { t } ) } \end{array}$ and decompose the solution $\xi _ { t }$ as:

$$
\forall t \geq 0, \qquad \xi_ {t} = \Psi_ {t _ {0}} ^ {t} \xi_ {0} + \sum_ {t _ {n} <   t} \Psi_ {t _ {n}} ^ {t} r _ {n} (\xi_ {t _ {n}})\tag{58}
$$

All we have to verify is that the appearance of the second-order terms $r _ { n } \big ( \xi _ { t _ { n } } \big )$ at each update is compensated by the exponential decay of $\Psi _ { t _ { 0 } } ^ { t }$ (Theorem 3).

## C.2 Review of existing linear results

Consider a linear time-varying Kalman filter and let $\boldsymbol { \Psi } _ { t _ { 0 } } ^ { t }$ denote the flow of the error variable $\xi _ { s }$ . It is proved in [13] that if the parameters of the Riccati equation verify conditions (i) - (v) then there exist $\gamma _ { \mathrm { { m a x } } } > 0$ and $\gamma _ { \mathrm { { m i n } } } > 0$ such that $\gamma _ { \mathrm { m a x } } I \succeq P _ { t } \succeq \gamma _ { \mathrm { m i n } } I$ . This pivotal property allows proving the solution of the linear error equation $\Psi _ { s } ^ { t } \xi _ { s }$ verifies for $V ( P , { \bar { \xi } } ) = \xi ^ { \bar { T } } { \bar { P ^ { - 1 } } } \xi .$

$$
V \left(P _ {t _ {n + N}} ^ {+}, \Psi_ {t _ {n}} ^ {t _ {n + N}} (\xi_ {t _ {n}} ^ {+})\right) \leq V (P _ {t _ {n}} ^ {+}, \xi_ {t _ {n}} ^ {+}) - \beta_ {3} | | \Psi_ {t _ {n}} ^ {t _ {n + N}} (\xi_ {t _ {n}} ^ {+}) | | ^ {2}\tag{59}
$$

where $\beta _ { 3 }$ only depends on $\alpha _ { 1 } , \alpha _ { 2 } , \beta _ { 1 } , \beta _ { 2 } , \delta _ { 1 } , \delta _ { 2 } , \delta _ { 3 } , N$ . Of course, the proof given in [13] holds if the inequalities are only verified on an interval $[ 0 , T ]$ . We will also use the direct consequence:

$$
V \left(P _ {t _ {n + 1}} ^ {+}, \Psi_ {t _ {n}} ^ {t _ {n + 1}} (\xi_ {t _ {n}} ^ {+})\right) <   V \left(P _ {t _ {n}} ^ {+}, \xi_ {t _ {n}} ^ {+}\right)\tag{60}
$$

## C.3 Preliminary lemmas

The proof of Theorem 4 is displayed in the next subsection. It relies on the final Lemma 7, which is proved step by step in this section through lemmas 4, 5 and 6. The time interval between two successive observations will be denoted $\Delta t . \tilde { P } _ { t }$ will denote the Kalman covariance about the true state trajectory.

Lemma 4. [modified constants for closeby trajectories] Ifthe conditions (i) to (v) are satisfied about the true trajectory, then for any $k > 1$ there exists a radius ε such that the bound $\forall s \in$ $[ 0 , t ] , | | \xi _ { t _ { 0 } + s } | | < \varepsilon$ ensures the conditions (i) to (v) are also verified on $[ t _ { 0 } , t _ { 0 } + t ]$ about the estimated trajectory, with the modified constants $\begin{array} { r } { \hat { \delta } _ { 1 } = \delta _ { 1 } , \hat { \delta } _ { 2 } = \frac { 1 } { k ^ { 2 } } \delta _ { 2 } , \hat { \delta } _ { 3 } = \frac { 1 } { k ^ { 2 } } \delta _ { 3 } , \hat { \alpha } _ { 1 } = \frac { 1 } { k ^ { 2 } } \alpha _ { 1 } , \hat { \alpha } _ { 2 } = } \end{array}$ $\begin{array} { r } { k ^ { 2 } \alpha _ { 2 } , \hat { \beta } _ { 1 } = \frac { 1 } { k ^ { 2 } } \beta _ { 1 } , \hat { \beta } _ { 2 } = k ^ { 2 } \beta _ { 2 } } \end{array}$ . Moreover, if $\begin{array} { r } { \frac { 1 } { k } \tilde { P } _ { t _ { 0 } } \leq P _ { t _ { 0 } } \leq k \tilde { P } _ { t _ { 0 } } } \end{array}$ then $\begin{array} { r } { \frac { 1 } { k } \tilde { P } _ { t _ { 0 } + s } \leq P _ { t _ { 0 } + s } \leq k \tilde { P } _ { t _ { 0 } + s } } \end{array}$ holds on [0, t].

Proof. We consider the LIEKF, the proof works the same way for the RIEKF. Matrices $\hat { Q } _ { t }$ and $\hat { N } _ { n }$ depend on the estimate $\hat { \mathcal { X } } _ { t }$ , this is why this lemma is needed. So we replace them by their values: $\hat { Q } _ { t } = C o \nu \left( w _ { t } \right)$ if the noise term has the form $\chi _ { t } w _ { t } , \hat { Q } _ { t } = A d _ { \hat { \chi } _ { t } ^ { - 1 } } C o \bar { \nu } \left( w _ { t } \right) A d _ { \hat { \chi } _ { t } ^ { - } } ^ { T }$ if the noise term has the form $w _ { t } \chi _ { t }$ , and $\hat { N } _ { n } = \hat { \chi } _ { t _ { n } } ^ { - 1 } C o \nu \left( V _ { n } \right) \hat { \chi } _ { t _ { n } } ^ { - T } + C o \nu \left( B _ { n } \right)$ . All these situations are covered if we assume there exist four (possibly time-dependent) matrices $Q _ { 1 } , Q _ { 2 } , N _ { 1 }$ and $N _ { 2 }$ such that $\hat { \mathcal { Q } } _ { t } = \boldsymbol { Q } _ { 1 } + A d _ { \hat { \chi } _ { t } ^ { - 1 } } \boldsymbol { Q } _ { 2 } A d _ { \hat { \chi } _ { t } ^ { - 1 } } ^ { T }$ and $\hat { N } _ { n } = \overline { { N _ { 1 } + \hat { \chi } _ { t _ { n } } ^ { - 1 } N _ { 2 } \hat { \chi } _ { t _ { n } } ^ { - T } } }$ . These notations will be used in the sequel but they hold only for this proof: they are not related to matrices $Q _ { 1 }$ and $Q _ { 2 }$ defined in the simulations sections. The Riccati equation computed about the true trajectory reads:

$$
\begin{array}{l} \frac {d}{d t} \tilde {P} _ {t} = A _ {t} \tilde {P} _ {t} + \tilde {P} _ {t} A _ {t} ^ {T} + Q _ {1} + A d _ {\chi_ {t}} Q _ {2} A d _ {\chi_ {t}} ^ {T}, \\ \tilde {P} _ {t _ {n}} ^ {+} = \tilde {P} _ {t _ {n}} - \tilde {P} _ {t _ {n}} H ^ {T} \left(H \tilde {P} _ {t _ {n}} H ^ {T} + N _ {1} + \chi_ {t _ {n}} ^ {- 1} N _ {2} \chi_ {t _ {n}} ^ {- T}\right) ^ {- 1} H \tilde {P} _ {t _ {n}}. \end{array}
$$

The Riccati equation computed on the estimated trajectory is obtained replacing $\chi _ { t }$ with $\hat { \mathcal { X } } _ { t }$ Recalling the error $\eta _ { t }$ and the properties of the $A d ,$ , the idea of the proof is simply to rewrite the Riccati equation computed about $\hat { \chi } _ { t }$ as a perturbation of the Riccati equation computed about $\chi _ { t }$ :

$$
\frac {d}{d t} P _ {t} = A _ {t} P _ {t} + P _ {t} A _ {t} ^ {T} + Q _ {1} + A d _ {\eta_ {t} ^ {- 1}} \left[ A d _ {\chi_ {t} ^ {- 1}} Q _ {2} A d _ {\chi_ {t} ^ {- 1}} ^ {T} \right] A d _ {\eta_ {t} ^ {- 1}} ^ {T},
$$

$$
P _ {t _ {n}} ^ {+} = P _ {t _ {n}} - P _ {t _ {n}} H ^ {T} \left(H P _ {t _ {n}} H ^ {T} + N _ {1} + \eta_ {t _ {n}} ^ {- 1} \left[ \chi_ {t _ {n}} ^ {- 1} N _ {2} \chi_ {t _ {n}} ^ {- T} \right] \eta_ {t _ {n}} ^ {- T}\right) ^ {- 1} H P _ {t _ {n}}.
$$

Controlling the perturbation is easy: matrix-valued functions $\xi  e ^ { - \xi }$ and $\xi  A d _ { e ^ { - } \xi }$ are continuous and equal to $I _ { d }$ for $\xi = 0$ , thus there exists a real $\varepsilon > 0$ depending only on k such that $| | \xi _ { t _ { 0 } + s } | | \leq \varepsilon$ ensures $\begin{array} { r } { \frac { 1 } { k } N _ { 2 } \preceq \left( e ^ { - \xi _ { t _ { 0 } + s } } \right) N _ { 2 } \left( e ^ { - \xi _ { t _ { 0 } + s } } \right) ^ { T } \preceq k N _ { 2 } } \end{array}$ and $\begin{array} { r } { \frac { 1 } { k } Q _ { 2 } \preceq A d _ { e ^ { - \xi _ { t _ { 0 } + s } } } Q _ { 2 } A d _ { e ^ { - \xi _ { t _ { 0 } + s } } } ^ { T } \preceq } \end{array}$ $k Q _ { 2 }$ . It ensures consequently

$$
\frac {1}{k} \left(Q _ {1} + A d _ {\chi_ {t _ {0} + s} ^ {- 1}} Q _ {2} A d _ {\chi_ {t _ {0} + s} ^ {- 1}} ^ {T}\right) \preceq Q _ {1} + A d _ {\hat {\chi} _ {t _ {0} + s} ^ {- 1}} Q _ {2} A d _ {\hat {\chi} _ {t _ {0} + s} ^ {- 1}} ^ {T} \preceq k \left(Q _ {1} + A d _ {\chi_ {t _ {0} + s} ^ {- 1}} Q _ {2} A d _ {\chi_ {t _ {0} + s} ^ {- 1}} ^ {T}\right)
$$

and

$$
\frac {1}{k} \left(N _ {1} + \chi_ {t _ {0} + s} ^ {- 1} N _ {2} \chi_ {t _ {0} + s} ^ {- T}\right) \preceq N _ {1} + \hat {\chi} _ {t _ {0} + s} ^ {- 1} N _ {2} \hat {\chi} _ {t _ {0} + s} ^ {- T} \preceq k \left(N _ {1} + \chi_ {t _ {0} + s} ^ {- 1} N _ {2} \chi_ {t _ {0} + s} ^ {- T}\right),
$$

and a mere look at the definitions of the constants of Theorem 3 yields the modified constants.

The inequality $\begin{array} { r } { \frac { 1 } { k } \tilde { P } _ { t _ { 0 } + s } \leq P _ { t _ { 0 } + s } \leq k \tilde { P } _ { t _ { 0 } + s } } \end{array}$ follows from the matrix inequalities above on the covariance matrices, by writing the Riccati equation verified by $k P _ { t }$ and $\scriptstyle { \frac { 1 } { k } } P _ { t }$ and using simple matrix inequalities.

Lemma 5. [first-order control of growth] Under the same conditions as in Lemma 4 (including $\begin{array} { r } { \frac { 1 } { k } \tilde { P } _ { t _ { 0 } } \leq P _ { t _ { 0 } } \leq k \tilde { P } _ { t _ { 0 } } ) } \end{array}$ and $| | \xi _ { t _ { 0 } + s } | |$ bounded by the same ε for $s \in [ 0 , 2 M \Delta T ]$ (i.e. over 2M time steps, where M is defined as in Theorem 3), there exists a continuous function l depending only on k ensuring $| | \xi _ { t _ { 0 } + s } | | \leq l _ { 1 } ( | | \xi _ { t _ { 0 } } | | )$ for any $s \in [ 0 , 2 M \Delta T ]$ and $l _ { 1 } ( x ) = O ( x )$

Proof. Using Lemma 4 and then Theorem 3 we know there exist two constants $\gamma _ { \mathrm { m i n } } > 0$ and $\gamma _ { \mathrm { { m a x } } } > 0$ such that $\gamma _ { \mathrm { m a x } } I \succeq P _ { t } \succeq \gamma _ { \mathrm { m i n } } I .$ . The non-linear rest $r _ { t _ { n } } ( \xi )$ introduced in (58) is defined by $\exp ( \xi ) \exp ( L _ { n } ( e ^ { \xi } b - b - H \xi ) ) = \exp ( ( I - L _ { n } H ) \xi + r _ { t _ { n } } ( \xi ) )$ . The Baker-Campbell-Hausdorff (BCH) formula gives $r _ { t _ { n } } ( \xi ) = O ( | | \xi | | . | | L _ { n } \xi | | )$ but $L _ { n }$ is uniformly bounded over time by $\frac { \gamma _ { \operatorname* { m a x } } | | H | | } { \delta _ { 3 } }$ as an operator. Thus $\lvert \lvert r _ { n } \rvert \rvert$ is uniformly dominated over time by a second order: there exists a continuous function $\tilde { l } ^ { k }$ (depending only on $k$ and on the true trajectory) such that $\tilde { l } ^ { k } ( x ) = O ( x ^ { 2 } )$ and $| | r _ { t _ { n } } ( \xi ) | | \leq \tilde { l } ^ { k } ( | | \xi | | )$ for any n such that $t _ { n } \leq 2 M \Delta T$

Now we can control the evolution of the error using $\tilde { l } ^ { k }$ . The propagation step is linear, thus we have the classical result $\begin{array} { r } { \frac { d } { d t } V _ { t } ( { \xi } _ { t } ) < 0 } \end{array}$ . It ensures $| | \xi _ { t _ { 0 } + s } | | < \sqrt { \frac { \gamma _ { \mathrm { m a x } } } { \gamma _ { \mathrm { m i n } } } } | | \xi _ { t _ { 0 } } | |$ as long as there is no update on $[ t _ { 0 } , t _ { 0 } + s ]$ . At each update step we have $V _ { t _ { n } } ^ { + } ( \xi _ { t _ { n } } ^ { + } ) ^ { 1 / 2 } { = } V _ { t _ { n } } ^ { + } \left( [ I - L _ { n } H ] \xi _ { t _ { n } } + r _ { n } ( \xi _ { t _ { n } } ) \right) ^ { 1 / 2 }$ $\leq V _ { t _ { n } } ^ { + } \left( \left[ I - L _ { n } H \right] \xi _ { t _ { n } } \right) ^ { 1 / 2 } + V _ { t _ { n } } ^ { + } \left( r _ { t _ { n } } ( \xi _ { t _ { n } } ) \right) ^ { 1 / 2 } \leq V _ { t _ { n } } \left( \xi _ { t _ { n } } \right) ^ { 1 / 2 } + V _ { t _ { n } } ^ { + } \left( r _ { n } ( \xi _ { t _ { n } } ) \right) ^ { 1 / 2 }$ using the triangular inequality. Thus: $\begin{array} { r } { | | \xi _ { t _ { n } } ^ { + } | | \leq \sqrt { \frac { \gamma _ { \operatorname* { m a x } } } { \gamma _ { \operatorname* { m i n } } } } \big ( | | \xi _ { t _ { n } } | | + | | r _ { n } ( \xi _ { t _ { n } } ) | | \big ) \ \leq \sqrt { \frac { \gamma _ { \operatorname* { m a x } } } { \gamma _ { \operatorname* { m i n } } } } \big ( | | \xi _ { t _ { n } } | | + \widetilde { l } ^ { k } ( | | \xi _ { t _ { n } } | | ) \big ) } \end{array}$ . Reiterating over successive propagations and updates over [0, 2M∆T], we see $| | \xi _ { t _ { 0 } + s } | |$ is uniformly bounded by a function $l _ { 1 } (   \boldsymbol { \xi } _ { t _ { 0 } }  | )$ that is first order in $| | \xi _ { t _ { 0 } } | |$

Lemma 6. [second-order control of the Lyapunov function] Under the same conditions as in Lemma 4 (including $\begin{array} { r } { \frac { 1 } { k } \tilde { P } _ { t _ { 0 } } \leq P _ { t _ { 0 } } \leq k \tilde { P } _ { t _ { 0 } } ) } \end{array}$ and for $| | \xi _ { t _ { 0 } + s } | |$ bounded by the same $\varepsilon f o r s \in$ [0, 2M∆T] (2M time steps, see Theorem 3 for the definition of M), there exists a continuous function $l _ { 2 }$ depending only on k ensuring $\begin{array} { r } { V _ { t _ { 0 } + s } \left( \xi _ { t _ { 0 } + s } \right) \le V _ { t _ { 0 } + s } ( \Psi _ { t _ { 0 } } ^ { t _ { 0 } + s } \xi _ { t _ { 0 } } ) + l _ { 2 } ( | | \xi _ { t _ { 0 } } | | ) \le } \end{array}$ $V _ { t _ { 0 } } ( \xi _ { t _ { 0 } } ) + l _ { 2 } ( | | \xi _ { t _ { 0 } } | | )$ for any $s \in [ 0 , 2 M \Delta t ]$ with $l _ { 2 } ( x ) = O ( x ^ { 2 } )$ . We also have $V _ { t _ { n } } \left( \xi _ { t _ { n } } ^ { + } \right) \leq V _ { t _ { n } } ^ { + } ( ( \Psi _ { t _ { 0 } } ^ { t _ { n } } ) ^ { + } \xi _ { t _ { 0 } } ) +$ $l _ { 2 } ( | | \xi _ { t _ { 0 } } | | ) \leq V _ { t _ { 0 } } ( \xi _ { t _ { 0 } } ) + l _ { 2 } ( | | \xi _ { t _ { 0 } } | | ) f o r t _ { 0 } \leq t _ { n } \leq t _ { 0 } + 2 M \Delta T$

Proof. The result stems from the decomposition (58) as:

$$
\begin{array}{l} V _ {t _ {0} + s} \left(\xi_ {t _ {0} + s}\right) ^ {1 / 2} = V _ {t _ {0} + s} \left(\Psi_ {t _ {0}} ^ {t _ {0} + s} \xi_ {t _ {0}} + \sum_ {t _ {0} <   t _ {n} <   t _ {0} + s} \Psi_ {t _ {n}} ^ {t _ {0} + s} r _ {n} \left(\xi_ {t _ {n}}\right)\right) ^ {1 / 2} \\ \leq V _ {t _ {0} + s} \left(\Psi_ {t _ {0}} ^ {t _ {0} + s} \xi_ {t _ {0} + s}\right) ^ {1 / 2} + \sum_ {t _ {0} <   t _ {n} <   t _ {0} + s} V _ {t _ {0} + s} \left(\Psi_ {t _ {n}} ^ {t _ {0} + s} r _ {n} \left(\xi_ {t _ {n}}\right)\right) ^ {1 / 2} \\ \text {(triangular inequality)} \\ \leq V _ {t _ {0} + s} \left(\Psi_ {t _ {0}} ^ {t _ {0} + s} \xi_ {t _ {0} + s}\right) ^ {1 / 2} + \sum_ {t _ {0} <   t _ {n} <   t _ {0} + s} V _ {t _ {n}} \left(r _ {n} \left(\xi_ {t _ {n}}\right)\right) ^ {1 / 2} \quad \text {(using (60))} \\ \leq V _ {t _ {0} + s} \left(\Psi_ {t _ {0}} ^ {t _ {0} + s} \xi_ {t _ {0} + s}\right) ^ {1 / 2} + \sum_ {t _ {0} <   t _ {n} <   t _ {0} + s} \sqrt {\frac {\gamma_ {\max}}{\gamma_ {\min}}} | | r _ {n} \left(\xi_ {t _ {n}}\right) | | ^ {1 / 2} \\ \text {(from the def. of V)} \\ \leq V _ {t _ {0} + s} \left(\Psi_ {t _ {0}} ^ {t _ {0} + s} \xi_ {t _ {0} + s}\right) ^ {1 / 2} + \sum_ {t _ {0} <   t _ {n} <   t _ {0} + s} \sqrt {\frac {\gamma_ {\max}}{\gamma_ {\min}} (\tilde {l} ^ {k} \circ l _ {1} ^ {k}) (| | \xi_ {t _ {0}} | |) ^ {1 / 2}} \\ \text {(from Lemma 5).} \end{array}
$$

As we have $( \tilde { l } ^ { k } \circ l _ { 1 } ^ { k } ) ( x ) = O ( x ^ { 2 } )$ , we obtain the result squaring the inequality and using $V _ { t _ { 0 } + s } \left( \Psi _ { t _ { 0 } } ^ { t _ { 0 } + s } \xi _ { t _ { 0 } + s } \right)$ $\leq V _ { t _ { 0 } } \left( \xi _ { t _ { 0 } } \right) \leq \frac { \gamma _ { \operatorname* { m a x } } } { \gamma _ { \operatorname* { m i n } } } | | \xi _ { t _ { 0 } } | |$ to control the crossed terms. □

Lemma 7. [final second order growth control] Under the same conditions as in Lemma 4 (including $\begin{array} { r } { \frac { 1 } { k } \tilde { P } _ { t _ { 0 } } \leq P _ { t _ { 0 } } \leq k \tilde { P } _ { t _ { 0 } } ) } \end{array}$ andfor $| | \xi _ { t _ { 0 } + s } | |$ bounded by the same ε for $s \in [ 0 , t ]$ , there exist twofunctions $l _ { 1 } ^ { k } ( \xi ) = O ( | | \xi | | ^ { 2 } )$ and $l _ { 2 } ^ { k } = o ( | | \xi | | ^ { 2 } )$ and a constant $\beta ^ { k }$ ensuring the relation:

$$
\begin{array}{l} V _ {t _ {0} + s} (\xi_ {t _ {0} + s}) \leq V _ {t _ {0}} (\xi_ {t _ {0}}) + l _ {1} ^ {k} (\xi_ {t _ {0}}) \\ - \sum_ {i = 0} ^ {J - 1} \left[ \beta^ {k} | | \xi_ {t _ {n _ {0}} + i M \Delta t} | | ^ {2} - l _ {2} ^ {k} (\xi_ {t _ {n _ {0}} + i M \Delta t}) \right] + l _ {1} ^ {k} (| | \xi_ {t _ {n _ {\max}}} ^ {+} | |), \end{array}\tag{61}
$$

(62)

where $n _ { \mathrm { m a x } }$ is the last update before $t _ { 0 } + s ( \ i . e . \ n _ { \operatorname* { m a x } } = \operatorname* { m a x } { \{ n , t _ { n } \leq t _ { 0 } + s \} } ) ,$ , J is the number of successive sequences of M updates in $\left[ t _ { 0 } + M \Delta t , t _ { n _ { \mathrm { m a x } } } \right] ( i . e . J = \operatorname* { m a x } \left\{ j , t _ { n _ { \mathrm { m a x } } - j M } \geq t _ { 0 } \right\} - 1 )$ and $n _ { 0 } = n _ { \operatorname* { m a x } } - J M . \ I f t _ { 0 } + s = t _ { n _ { \operatorname* { m a x } } }$ the last term can be removed.

Proof. For $l _ { 1 } ^ { k }$ we choose the same function as in Lemma $^ { 6 . }$ There is nothing more to prove for $s < 2 M \Delta t$ . Let $s \geq 2 M \Delta t$ . We have $V _ { t _ { 0 } + s } ( \xi _ { t _ { 0 } + s } ) - V _ { t _ { 0 } } ^ { + } ( \xi _ { t _ { 0 } } ^ { + } ) = \left( V _ { t _ { 0 } + s } ( \xi _ { t _ { 0 } + s } ) - V _ { t _ { n _ { \mathrm { m a x } } } } ^ { + } ( \xi _ { t _ { n _ { \mathrm { m a x } } } } ^ { + } ) \right) +$ $\left( V _ { t _ { n _ { \mathrm { m a x } } } } ^ { + } ( \xi _ { t _ { n _ { \mathrm { m a x } } } } ^ { + } ) - V _ { t _ { n _ { 0 } } } ^ { + } ( \xi _ { t _ { n _ { 0 } } } ^ { + } ) \right) + \left( V _ { t _ { n _ { 0 } } } ^ { + } ( \xi _ { t _ { n _ { 0 } } } ^ { + } ) - V _ { t _ { 0 } } ( \xi _ { t _ { 0 } } ) \right)$ . The first and third terms are upper bounded using Lemma 6. The second term is controlled as follows:

$$
\begin{array}{l} V _ {t _ {n _ {\max}}} ^ {+} (\xi_ {t _ {n _ {\max}}} ^ {+}) - V _ {t _ {n _ {0}}} ^ {+} (\xi_ {t _ {n _ {0}}} ^ {+}) \\ = \sum_ {i = 0} ^ {J - 1} \left[ V _ {t _ {n _ {0} + (i + 1) M}} ^ {+} (\xi_ {t _ {n _ {0} + (i + 1) M}} ^ {+}) - V _ {t _ {n _ {0} + i M}} ^ {+} (\xi_ {t _ {n _ {0} + i M}} ^ {+}) \right] \\ \leq \sum_ {i = 0} ^ {J - 1} \left[ V _ {t _ {n _ {0} + (i + 1) M}} ^ {+} (\Psi_ {t _ {n _ {0} + i M}} ^ {t _ {n _ {0} + (i + 1) M} +} \xi_ {t _ {n _ {0} + i M}} ^ {+}) - V _ {t _ {n _ {0} + i M}} ^ {+} (\xi_ {t _ {n _ {0} + i M}} ^ {+}) + l _ {2} (\xi_ {t _ {n _ {0} + i M}} ^ {+}) \right]. \end{array}
$$

And we conclude using (see [13]):

$$
\begin{array}{r l} & V _ {t _ {n _ {0} + (i + 1) M}} ^ {+} (\Psi_ {t _ {n _ {0} + i M}} ^ {t _ {n _ {0} + (i + 1) M} +} \xi_ {t _ {n _ {0} + i M}} ^ {+}) - V _ {t _ {n _ {0} + i M}} ^ {+} (\xi_ {t _ {n _ {0} + i M}} ^ {+}) \leq \\ & \quad - \tilde {\beta} ^ {k} | | \Psi_ {t _ {n _ {0} + i M}} ^ {t _ {n _ {0} + (i + 1) M} +} \xi_ {t _ {n _ {0} + i M}} ^ {+} | | ^ {2} \leq - \tilde {\beta} ^ {k} (\frac {\gamma_ {\min}}{\gamma_ {\max}} \delta_ {1}) ^ {M} | | \xi_ {t _ {n _ {0} + i M}} ^ {+} | | ^ {2}, \end{array}
$$

for a $\tilde { \beta } ^ { k }$ depending only on the modified constants of Lemma 4. The last inequality is obtained using $\Psi _ { t _ { 0 } } ^ { t _ { n } + } = ( P _ { n } ^ { + } P _ { n } ^ { - 1 } ) \Psi _ { t _ { 0 } } ^ { t _ { n } }$ and an obvious recursion over M time steps. We finally set $\beta =$ $\tilde { \beta } ^ { k } \big ( \frac { \gamma _ { \mathrm { m i n } } } { \gamma _ { \mathrm { m a x } } } \delta _ { 1 } \big ) ^ { M }$ □

Remark 3. The control we have obtained on $\xi _ { t _ { 0 } + s }$ is verified $i f | | \xi _ { t _ { 0 } + s } | |$ is already in a ball of radius ε over the whole interval $\left[ t _ { 0 } , t _ { 0 } + t \right]$ . We now prove the result holds assuming only that $\xi _ { t _ { 0 } }$ is sufficiently small.

## C.4 Proof of theorem 4

Applying Lemma 7 with $t _ { 0 } + s = t _ { n _ { \mathrm { m a x } } }$ gives for $\begin{array} { r } { \frac { 1 } { k } \tilde { P } _ { t _ { 0 } } \leq P _ { t _ { 0 } } \leq k \tilde { P } _ { t _ { 0 } } } \end{array}$ and $| | \xi _ { t _ { 0 } + s } | | < \varepsilon \mathrm { o n } \left[ 0 , t \right]$ :

$$
\begin{array}{l} | | \xi_ {t _ {n _ {\max}}} ^ {+} | | ^ {2} \leq \frac {\gamma_ {\max}}{\gamma_ {\min}} | | \xi_ {t _ {0}} | | ^ {2} + \gamma_ {\max} l _ {1} ^ {k} (| | \xi_ {t _ {0}} | |) \\ - \frac {\gamma_ {\max}}{\gamma_ {\min}} \sum_ {i = 0} ^ {J - 1} \left[ \beta^ {k} | | \xi_ {t _ {n _ {0} + i M}} ^ {+} | | ^ {2} - l _ {2} (\xi_ {t _ {n _ {0} + i M}} ^ {+}) \right]. \end{array}
$$

There exist $K > 0$ and $\varepsilon ^ { \prime } > 0$ such that for $x < \varepsilon ^ { \prime }$ , we have $l _ { 2 } ( x ) < \frac { \beta ^ { k } } { 2 } x$ and $\gamma _ { \mathrm { m a x } } l _ { 1 } ^ { k } ( x ) < K x$ (as $l _ { 2 } ( x ) = O ( x ^ { 2 } )$ and $l _ { 1 } ^ { k } ( x ) = O ( x ^ { 2 } ) )$ which gives: $\begin{array} { r } { \vert \vert \xi _ { t _ { n _ { \mathrm { m a x } } } } ^ { + } \vert \vert ^ { 2 } \leq \left( \frac { \gamma _ { \mathrm { m a x } } } { \gamma _ { \mathrm { m i n } } } + K \right) \vert \vert \xi _ { t _ { 0 } } \vert \vert ^ { 2 } } \end{array}$ . Thus, for $\begin{array} { r } { \vert \vert \xi _ { t _ { 0 } } \vert \vert < \frac { \varepsilon ^ { \prime } } { \sqrt { \frac { \gamma _ { \mathrm { m a x } } } { \gamma _ { \mathrm { m i n } } } + K } } ; } \end{array}$

$$
\begin{array}{c} | | \xi_ {t _ {0} + s} | | ^ {2} \leq \left(\frac {\gamma_ {\max}}{\gamma_ {\min}} + K + K \left(\frac {\gamma_ {\max}}{\gamma_ {\min}} + K\right)\right) | | \xi_ {t _ {0}} | | ^ {2} \\ - \frac {\gamma_ {\max}}{\gamma_ {\min}} \sum_ {i = 0} ^ {J - 1} \frac {\beta^ {k}}{2} | | \xi_ {t _ {n _ {0} + i M}} ^ {+} | | ^ {2}, \end{array}\tag{63}
$$

which finally ensures

$$
\left| \left| \xi_ {t _ {0}} \right| \right| <   \frac {1}{2} \varepsilon^ {\prime} / \left(\frac {\gamma_ {\max}}{\gamma_ {\min}} + K + K \left(\frac {\gamma_ {\max}}{\gamma_ {\min}} + K\right)\right) \Rightarrow \left| \left| \xi_ {t _ {0} + s} \right| \right| \leq \varepsilon^ {\prime} / 2
$$

Reducing $\varepsilon ^ { \prime }$ if necessary to have $\varepsilon ^ { \prime } \leq \varepsilon$ , we have obtained $| | \xi _ { t _ { 0 } + s } | | < \varepsilon ^ { \prime }$ for $s \in [ 0 , t ] \Rightarrow$ $| | \xi _ { t _ { 0 } + s } | | \leq \varepsilon ^ { \prime } / 2$ for sufficiently small $| | \xi _ { t _ { 0 } } | |$ (as Lemma 7 applies). Letting t = inf $\left\{ s , | | \xi _ { t _ { 0 } + s } | \right\} \left| \geq \frac 3 4 \varepsilon ^ { \prime } \right\}$ for sufficiently small $| | \xi _ { t _ { 0 } } | |$ we end up with a contradiction if we suppose $t < + \infty$ , which proves $t = + \infty$ . All the previous results thus hold only for sufficiently small $| | \xi _ { t _ { 0 } } | |$

Moreover, (63) shows that $\textstyle \sum _ { i = 0 } ^ { J - 1 } { \frac { \beta ^ { k } } { 2 } } \vert \vert \xi _ { t _ { n _ { 0 } + i M } } ^ { + } \vert \vert ^ { 2 }$ is bounded and has positive terms thus $\lvert \lvert \xi _ { t _ { n _ { 0 } + i M } } ^ { + } \rvert \rvert ^ { 2 }$ goes to zero. Note also that $| | P _ { t } - \tilde { P } _ { t } | | \underset { t  + \infty } { \longrightarrow } 0$ as a byproduct.

## D Proof of proposition 3

Only conditions (i) and (v) are non-trivial. Let Φ denote the flow of the dynamics. We have

$$
\frac {d}{d t} \left[ (\Phi_ {t _ {n}} ^ {t}) ^ {T} \Phi_ {t _ {n}} ^ {t} \right] = (\Phi_ {t _ {n}} ^ {t}) ^ {T} \left( \begin{array}{c c c} 0 & 0 & - v _ {t} \\ 0 & 0 & 0 \\ - v _ {t} & 0 & 0 \end{array} \right) \Phi_ {t _ {n}} ^ {t} \succeq - v _ {\max} (\Phi_ {t _ {n}} ^ {t}) ^ {T} \Phi_ {t _ {n}} ^ {t}
$$

as the eigenvalues of $\left( \begin{array} { l l l } { 0 } & { 0 } & { 1 } \\ { 0 } & { 0 } & { 0 } \\ { 1 } & { 0 } & { 0 } \end{array} \right)$ are $( 1 , - 1 , 0 )$ . Thus, $\forall z \in \mathbb { R } ^ { 3 }$ , <sub>dt</sub> <sup>d</sup> log $\left( z ^ { T } \left( \Phi _ { t _ { n } } ^ { t } \right) ^ { T } \Phi _ { t _ { n } } ^ { t } z \right) \geqslant$ $- \nu _ { \mathrm { m a x } }$ and finally $z ^ { T } ( \Phi _ { t _ { n } } ^ { i _ { n + 1 } } ) ^ { T } \Phi _ { t _ { n } } ^ { t _ { n + 1 } ^ { * } } z \geqslant \exp \left( - \nu _ { \operatorname* { m a x } } \left( t _ { n + 1 } - t _ { n } \right) \right) | | z | | ^ { 2 } \mathrm { ~ a s ~ } ( \Phi _ { t _ { n } } ^ { t _ { n } } ) ^ { T } \Phi _ { t _ { n } } ^ { t _ { n } } = I _ { 3 }$ . Thus $( \Phi _ { t _ { n } } ^ { t _ { n + 1 } } ) ^ { T } \Phi _ { t _ { n } } ^ { t _ { n + 1 } } \succeq \exp \left( - \nu _ { \operatorname* { m a x } } \left( t _ { n + 1 } - t _ { n } \right) \right) I _ { 3 }$ and (i) is verified. The difficult part of (v) is the lower bound. Denoting $C o \nu ( V _ { n } )$ by N we will show:

$$
\begin{array}{l} \exists \beta_ {1}, \forall n \in \mathbb {N}, \quad \beta_ {1} I _ {3} \leq \hat {R} _ {t _ {n}} ^ {T} N ^ {- 1} \hat {R} _ {t _ {n}} \\ \qquad + (\Phi_ {t _ {n}} ^ {t _ {n + 1}}) ^ {T} (0 _ {1, 2} I _ {2}) ^ {T} \hat {R} _ {t _ {n - 1}} ^ {T} N ^ {- 1} \hat {R} _ {t _ {n - 1}} (0 _ {1, 2} I _ {2}) \Phi_ {t _ {n}} ^ {t _ {n + 1}}. \end{array}
$$

That is to say that we want a lower bound on the quadratic form:

$$
\begin{array}{l} M \binom {\theta} {u} = \binom {\theta} {u} ^ {T} \hat {R} _ {t _ {n}} ^ {T} N ^ {- 1} \hat {R} _ {t _ {n}} \binom {\theta} {u} \\ + \binom {\theta} {u} ^ {T} (\Phi_ {t _ {n}} ^ {t _ {n + 1}}) ^ {T} (0 _ {1, 2} I _ {2}) ^ {T} \hat {R} _ {t _ {n - 1}} ^ {T} N ^ {- 1} \hat {R} _ {t _ {n - 1}} (0 _ {1, 2} I _ {2}) \Phi_ {t _ {n}} ^ {t _ {n + 1}} \binom {\theta} {u}. \end{array}
$$

We decompose $\Phi _ { t _ { n } } ^ { t _ { n + 1 } }$ as $\Phi _ { t _ { n } } ^ { t _ { n + 1 } } = \binom { 1 } { \delta V _ { n } } \stackrel { 0 } { T _ { n } } ^ { 0 } \bigg )$ . To simplify the writing we introduce the norms $| | x | | _ { N } ^ { 2 } = x ^ { T } N ^ { - 1 } x$ and the associated scalar product $\langle . , . \rangle _ { N }$ . There exists $\alpha > 0$ such that $\forall x \in \mathbb { R } ^ { 2 } , | | x | | _ { N } \geq \alpha | | x | |$ . For any ${ \binom { \theta } { u } } \in \mathbb { R } ^ { 3 }$ we have $M \left( \begin{array} { c } { { \theta } } \\ { { u } } \end{array} \right) = | | \hat { R } _ { t _ { n } } u | | _ { N } ^ { 2 } + | | \theta \hat { R } _ { t _ { n - 1 } } \delta V _ { n } +$ $\hat { R } _ { t _ { n - 1 } } T _ { n } u | | _ { N } ^ { 2 } = | | \hat { R } _ { t _ { n } } u | | _ { N } ^ { 2 } + \theta ^ { 2 } | | \hat { R } _ { t _ { n - 1 } } \delta V _ { n } | | _ { N } ^ { 2 } + 2 \theta \left. \hat { R } _ { t _ { n - 1 } } \delta V _ { n } , \hat { R } _ { t _ { n - 1 } } T _ { n } u \right. _ { N } + | | \hat { R } _ { t _ { n - 1 } } T _ { n } u | | _ { N } ^ { 2 }$ and for $\lambda \in ] 0 , 1 ]$ we have:

$$
\begin{array}{l} M \binom{\theta}{u} = | | \hat {R} _ {t _ {n}} u | | _ {N} ^ {2} + (1 - \lambda^ {2}) \theta^ {2} | | \hat {R} _ {t _ {n - 1}} \delta V _ {n} | | _ {N} ^ {2} + \lambda^ {2} \theta^ {2} | | \hat {R} _ {t _ {n - 1}} \delta V _ {n} | | _ {N} ^ {2} \\ \qquad + 2 \theta \left\langle \hat {R} _ {t _ {n - 1}} \delta V _ {n}, \hat {R} _ {t _ {n - 1}} T _ {n} u \right\rangle_ {N} + | | \hat {R} _ {t _ {n - 1}} T _ {n} u | | _ {N} ^ {2} \\ = | | \hat {R} _ {t _ {n}} u | | _ {N} ^ {2} + (1 - \lambda^ {2}) \theta^ {2} | | \hat {R} _ {t _ {n - 1}} \delta V _ {n} | | _ {N} ^ {2} + | | \lambda \theta \hat {R} _ {t _ {n - 1}} \delta V _ {n} \\ \qquad + \frac {1}{\lambda} \hat {R} _ {t _ {n - 1}} T _ {n} u | | _ {N} ^ {2} + (1 - \frac {1}{\lambda^ {2}}) | | \hat {R} _ {t _ {n - 1}} T _ {n} u | | _ {N} ^ {2} \\ \geq \alpha \bigg [ | | \hat {R} _ {t _ {n}} u | | ^ {2} + (1 - \lambda^ {2}) \theta^ {2} | | \hat {R} _ {t _ {n - 1}} \delta V _ {n} | | ^ {2} \\ \qquad + | | \lambda \theta \hat {R} _ {t _ {n - 1}} \delta V _ {n} + \frac {1}{\lambda} \hat {R} _ {t _ {n - 1}} T _ {n} u | | ^ {2} + (1 - \frac {1}{\lambda^ {2}}) | | \hat {R} _ {t _ {n - 1}} T _ {n} u | | ^ {2} \bigg ] \\ \geq \alpha \left[ | | u | | ^ {2} + (1 - \lambda^ {2}) \theta^ {2} | | \delta V _ {n} | | ^ {2} + (1 - \frac {1}{\lambda^ {2}}) | | u | | ^ {2} \right] \\ \geq \alpha \left[ (2 - \frac {1}{\lambda^ {2}}) | | u | | ^ {2} + \theta^ {2} + [ \frac {1 - \lambda^ {2}}{2 - \frac {1}{\lambda^ {2}}} | | \delta V _ {n} | | ^ {2} - 1 ] \theta^ {2} \right] \\ \geq \alpha (2 - \frac {1}{\lambda^ {2}}) \left(| | u | | ^ {2} + \theta^ {2} + [ \frac {1 - \lambda^ {2}}{2 - \frac {1}{\lambda^ {2}}} v _ {\min} ^ {2} - 1 ] \theta^ {2}\right). \end{array}
$$

As $\frac { 1 - \lambda ^ { 2 } } { 2 - \frac { 1 } { \lambda ^ { 2 } } } \longrightarrow _ { \lambda  \frac { 1 } { \sqrt { 2 } } } -$ +∞ there exists $\lambda _ { 0 }$ such that: $M \left( \begin{array} { l } { \theta } \\ { u } \end{array} \right) \geq \alpha \left( 2 - \frac { 1 } { \lambda _ { 0 } ^ { 2 } } \right) \left. \theta \right. ^ { 2 }$ and the result is true for $\begin{array} { r } { \beta _ { 1 } = \alpha ( 2 - \frac { 1 } { \lambda _ { 0 } ^ { 2 } } ) } \end{array}$

## References

[1] N. Aghannan and P. Rouchon. An intrinsic observer for a class of Lagrangian systems. IEEE Trans. on Automatic Control, 48(6):936–945, 2003.

[2] Martin Barczyk, Silvere Bonnabel, Jean-Emmanuel Deschaud, and Franc¸ois Goulette. Invariant EKF design for scan matching-aided localization. IEEE Trans. on Systems Control Technology. DOI 10.1109/TCST.2015.2413933, 2015.

[3] Martin Barczyk and Alan F Lynch. Invariant observer design for a helicopter UAV aided inertial navigation system. Control Systems Technology, IEEE Transactions on, 21(3):791–806, 2013.

[4] Axel Barrau and Silvere Bonnabel. Invariant particle filtering with application to localization. In Decision and Control (CDC), 2014 IEEE 53rd Annual Conference on, pages 5599–5605. IEEE, 2014.

[5] Axel Barrau and Silvere Bonnabel. Intrinsic filtering on Lie groups with applications to attitude estimation. Automatic Control, IEEE Transactions on, 60(2):436–449, 2015.

[6] S. Bonnabel. Left-invariant extended Kalman filter and attitude estimation. In Decision and Control, 2007 46th IEEE Conference on, pages 1027–1032. IEEE, 2007.

[7] S. Bonnabel, P. Martin, and P. Rouchon. Non-linear symmetry-preserving observers on Lie groups. IEEE Transactions on Automatic Control, 54(7):1709–1713, 2009.

[8] S. Bonnabel, Ph. Martin, and P. Rouchon. Symmetry-preserving observers. Automatic Control, IEEE Transactions on, 53(11):2514–2526, 2008.

[9] Silvere Bonnabel, Philippe Martin, and Erwan Salaun. Invariant extended Kalman filter:¨ theory and application to a velocity-aided attitude estimation problem. In Decision and Control, 2009 held jointly with the 2009 28th Chinese Control Conference. CDC/CCC 2009. Proceedings ofthe 48th IEEE Conference on, pages 1297–1304. IEEE, 2009.

[10] Silvere Bonnabel and Jean-Jacques Slotine. A contraction theory-based analysis of the stability of the deterministic extended Kalman filter. Automatic Control, IEEE Transactions on, 60(2):565–569, 2015.

[11] M. Boutayeb, H. Rafaralahy, and M. Darouach. Convergence analysis of the extended Kalman filter used as an observer for nonlinear deterministic discrete-time systems. IEEE transactions on automatic control, 42, 1997.

[12] Alessandro De Luca, Giuseppe Oriolo, and Claude Samson. Feedback control of a nonholonomic car-like robot. In Robot motion planning and control, pages 171–253. Springer, 1998.

[13] J. Deyst and C. Price. Conditions for asymptotic stability of the discrete minimumvariance linear estimator. IEEE Trans. on Automatic Control, 13:702–705, 1968.

[14] Jay Farrell. Aided navigation: GPS with high rate sensors. McGraw-Hill, Inc., 2008.

[15] J.P. Gauthier and I. Kupka. Deterministic Observation Theory and Applications. Cambridge University Press, 2001.

[16] Havard Fjær Grip, Thor I Fossen, Tor A Johansen, and Ali Saberi. Globally exponen-˚ tially stable attitude and gyro bias estimation with application to GNSS/INS integration. Automatica, 51:158–166, 2015.

[17] Minh-Duc Hua. Attitude estimation for accelerated vehicles using GPS/INS measurements. Control Engineering Practice, 18(7):723–732, 2010.

[18] Maziar Izadi and Amit K Sanyal. Rigid body attitude estimation based on the Lagrange– d- Alembert principle. Automatica, 50(10):2570–2577, 2014.

[19] Arthur J Krener. The convergence of the extended Kalman filter. In Directions in mathematical systems theory and optimization, pages 173–182. Springer, 2003.

[20] Ch. Lageman, J. Trumpf, and R. Mahony. Gradient-like observers for invariant dynamics on a Lie group. Automatic Control, IEEE Transactions on, 55(2):367–377, 2010.

[21] E. J. Lefferts, F Landis Markley, and M. D. Shuster. Kalman filtering for spacecraft attitude estimation. Journal ofGuidance, Control, and Dynamics, 5(5):417–429, 1982.

[22] Robert Mahony, Tarek Hamel, and J-M Pflimlin. Complementary filter design on the special orthogonal group SO(3). In Decision and Control, 2005 and 2005 European Control Conference. CDC-ECC’05. 44th IEEE Conference on, pages 1477–1484. IEEE, 2005.

[23] Philippe Martin and Erwan Salaun. Generalized multiplicative extended Kalman filter¨ for aided attitude and heading reference system. In AIAA Guidance, Navigation, and Control Conference, page 8300, 2010.

[24] F. Sonnemann Reif, K. and R. Unbehauen. An EKF-based nonlinear observer with a prescribed degree of stability. Automatica, 34:1119–1123, 1998.

[25] Y.K. Song and J.W. Grizzle. The extended Kalman filter as a local asymptotic observer. Estimation and Control, 5:59–78, 1995.

[26] J.F. Vasconcelos, R. Cunha, C. Silvestre, and P. Oliveira. A nonlinear position and attitude observer on SE(3) using landmark measurements. Systems Control Letters, 59:155– 166, 2010.

![](images/efec6ab5edeea1d834b1a0817f840e7520d5a62298dec2ada2b5afe58da90d25.jpg)

![](images/406861c254d89b6c647676952459989479f5a590d996a69ab9ecd89e45022ac0.jpg)

![](images/1aecc04fc3ae2ed882134900b0472efc8e7834438a00a86949a8b025bd6dcbb6.jpg)

![](images/24a639bd1b22c02c57f5d05bd3221ec787ccdc34e91470091c6b537580e7f4a8.jpg)

![](images/619d13fe9b020b1dcfed4668b60924af2f8edd770dc6957a446eccac6fa2ec62.jpg)

![](images/1d298e40112fda9090e28bd5e1c4f08d45f7053a7dd886b7ae7efa59f68ea47a.jpg)  
Figure 1: The heading and position of the car are estimated through EKF and IEKF with high rate odometry and low rate GPS measurements. Top plots illustrate the experimental setting and display the estimated trajectories, middle plots display the heading errors and bottom plots the position errors. As the starting point is assumed known in this simulation, the initial values of the latter are zero. But it increases afterwards due to initial heading error. Left column: small initial angle error (1<sup>◦</sup>). We see EKF and IEKF behave similarly (at least for a long time) as propagation steps are identical. Right column: large initial angle error $( 4 5 ^ { \circ } )$ The behaviors rapidly become different, and the EKF is outperformed. Due to its righteous use of the system’s non-linearities, the IEKF keeps ensuring rapid estimation error decrease.

![](images/8a2aefaebd002e34f6a8efbe524dc32dfc9f68d286712f9acaa9ec4797d498d0.jpg)

![](images/ab48fb622aa8fff00e264a99ff3fb7c9485a7d6d725155b70b37a1d3ba017ef2.jpg)

![](images/0619edd64811f643aa0ddf934a06eccc4e2c2a6042a6d574a923d232a0e7d24b.jpg)

![](images/f2a3c196b92524980984f0b721b7db91fb303277c25f19f1015f393f167201d2.jpg)

![](images/5559cdbe5f139659f88fc8384055a29bd59c18d2943111abfcbe5a8e5d43f219.jpg)

![](images/c625a2ba3dd6607e5e855aa77d0a559720be0f642f75894f3e0250cfa5990ea0.jpg)

![](images/f6a879b0108aab330374d4f441304378152b3c51eeffd34a263014814a0971f8.jpg)  
Figure 2: Aided inertial navigation based on high rate accelerometers’ and gyrometers’ measurements and low rate observation of known landmarks. We also displayed the orthogonal projection of the landmarks on the plane containing the trajectory (black crosses) to help imagining the $3 \mathrm { D }$ position of the landmarks. This shows the disposition of the landmarks is the same in both experiments. Top plots illustrate the experimental setting and display the EKF and IEKF estimates. Middle plots display the attitude errors and bottom plots the position errors. Left column: the tuning of $Q$ is tight $( Q = Q _ { 1 } )$ due to highly precise inertial sensors. This creates robustness issues: the gains of the EKF decrease rapidly during the transitory phase while the attitude error is not reduced enough due to non-linearities. When the position estimate is impacted, the gains have become too small to correct the errors, leading to filter’s divergence. IEKF ensures rapid decrease to 0 of the estimation error with identical tuning. Right plot: $Q$ is inflated $( Q = Q _ { 2 } )$ . This classical engineering trick prevents the EKF to diverge but IEKF still prevails in terms of time of convergence.