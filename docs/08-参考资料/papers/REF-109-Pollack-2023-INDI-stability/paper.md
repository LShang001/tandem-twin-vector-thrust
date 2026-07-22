Delft University of Technology

# Robust Stability and Performance Analysis of Incremental Dynamic Inversion-based Flight Control Laws

Pollack, T.S.C.; van Kampen, E.

DOI 10.2514/6.2022-1395

Publication date 2022

Document Version

Final published version

Published in

AIAA SCITECH 2022 Forum

## Citation (APA)

Pollack, T. S. C., & van Kampen, E. (2022). Robust Stability and Performance Analysis of Incremental Dynamic Inversion-based Flight Control Laws. In AIAA SCITECH 2022 Forum Article AIAA 2022-1395 (AIAA Science and Technology Forum and Exposition, AIAA SciTech Forum 2022). https://doi.org/10.2514/6.2022-1395

## Important note

To cite this publication, please use the final published version (if applicable).
Please check the document version above.

## Copyright

Other than for strictly personal use, it is not permitted to download, forward or distribute the text or part of it, without the consent of the author(s) and/or copyright holder(s), unless the work is under an open content license such as Creative Commons.

## Takedown policy

Please contact us and provide details if you believe this document breaches copyrights.
We will remove access to the work immediately and investigate your claim.

# Robust Stability and Performance Analysis of Incremental Dynamic Inversion-based Flight Control Laws

T.S.C. Pollack∗, and E. van Kampen† Delft University of Technology, 2629HS Delft, The Netherlands

Incremental Nonlinear Dynamic Inversion (INDI) is a sensor-based control law design strategy that is based on the principles of feedback linearization. Contrary to its non-incremental counterpart (NDI), this design method does not rely on the availability of a high-fidelity on-board model of the airframe dynamics and is robust to aerodynamic variations. Consequently, INDI brings a natural and robust design approach to desirable flying qualities. However, robustness to singular perturbations, which may arise due to transportation lags, elastic airframe effects, or other types of badly modelled or unknown dynamics, is a known challenge for INDI-based control laws. In this article, the general stability and performance robustness properties of INDI and its linear form (IDI) are described analytically and analyzed in a flight control law design study by means of the structured singular value frame framework. In addition, inversion loop augmentation solutions are investigated using automated synthesis to further improve the robustness characteristics of basic IDI designs.

## I. Introduction

he development of aircraft flight control laws is a comprehensive and multidisciplinary activity, one that is generally associated with significant effort and costs [1]. A primary reason is that flight control laws are subject to many, often conflicting design objectives and requirements associated with the intended mission and vehicle safety, which follow from overall program requirements and standards issued by airworthiness certification authorities [2, 3]. This relates to several areas, including stability, flying qualities, dynamic structural mode interaction (SMI) [4], and structural loads, and need to be complied with to various extents throughout the range of possible aircraft flight envelopes and configurations, including failure conditions [2, 3]. Consequently, the size and complexity of the development task is strongly dependent on the characteristics of the airframe and its desired capabilities.

In both the civil and military domain, industry has been able to successfully design and certify flight control laws for a wide range of vehicles and applications. Many of these designs are based on the divide-and-conquer philosophy, which relies on strictly linear design and analysis techniques applied to a range of linearized models of the airframe dynamics obtained over a grid of trim conditions throughout the flight envelope. Among these techniques are classical design methods such as one-loop-at-a-time frequency response shaping, root locus techniques, and gain scheduling, but also multivariable control law synthesis and analysis techniques that are able to capture the multivariable nature of the aircraft control problem, such as LQR/LQG and eigenstructure assignment design techniques [5–8]. However, the combination of more demanding operating capabilities and a general need for shorter and more efficient design cycles has put this proven design strategy under strain. The need for desirable flying qualities in (very) broad regions of the flight envelope results in extensive gain scheduling schemes that significantly increase the complexity of the resulting control law [9, 10]. Moreover, design and tuning of these flight control laws is strongly tied to the dynamics of the bare airframe, which prohibits re-use over multiple airframes.

Nonlinear Dynamic Inversion (NDI) is a control law design technique that can address these challenges to a considerable extent. NDI-based control laws incorporate an on-board model (OBM) representation of the airframe dynamics, which allows the flying qualities design task to be performed in a largely isolated fashion. This natural advantage has been recognized by industry as well and represents a key feature that makes NDI attractive as a production control law design technique [10, 11]. However, NDI-based control laws come with several challenges of their own, with control law complexity and robustness being particular areas of concern. For example, the aerodynamic database associated with the vehicle dynamics can grow very large in size, in particular when the aircraft features a broad flight envelope and can attain many different configurations. At the same time, the control law needs to be robust against modeling and scheduling errors, which may take serious forms especially in those conditions where the airframe aerodynamics are nonlinear and known to only a limited extent.

Incremental NDI (INDI) seeks to address these limitations by reducing the need for detailed and accurate on-board models of the airframe aerodynamics by using direct sensor measurements of the derivatives of the control variables (CVs) instead. The only model information required by such control laws relates to the control effectiveness information for control allocation purposes. Since the early work on this simplified form of NDI [12, 13], which only later became more commonly known as INDI [14], the ease of implementation of this technique and its ability to guarantee flying quality robustness in face of linear and nonlinear variations and uncertainties in the airframe aerodynamics has been demonstrated repeatedly both in-simulation and during flight tests [15–19]. In addition, analytical proofs of nominal and robust stability properties under external disturbances and regular perturbations have been established [20]. These achievements show that INDI has real benefits for use in future production aircraft.

However, compared to traditional NDI, research has also shown that INDI has a relatively small stability robustness margin when subjected to singular perturbations, such as transportation lags and other forms of unmodelled dynamics that affect the sensor feedback paths [16, 21, 22]. In the context of the general flight control law development cycle, this implies that stability and SMI requirements may be difficult to meet in practice. The contribution of this article is to illustrate the stability and performance robustness properties of INDI-based flight control laws when subjected to both regular and singular perturbations and to establish inversion loop design solutions that improve these properties.

The article is structured as follows. Section II describes the basic characteristics of INDI by investigating analytically its general robustness properties in nonlinear and linear form. This sets the stage for a numerical case study in Section III, where the robust stability and performance properties of IDI in its basic form are analyzed using the $\mu \cdot$ -framework in the context of a pitch rate control law for an open-access General Dynamics F-16 simulation model. This is followed by a discussion in Section IV focused on augmentation techniques that enhance the robustness of basic IDI in a $\mu \cdot$ -optimal sense. The article is concluded in Section V.

## II. Basic Properties of Incremental Nonlinear Dynamic Inversion

In this Section, the fundamental robustness characteristics of INDI are investigated and compared to its nonincremental counterpart based on analytical derivations and insights. First, the derivation of the control law is reviewed in Subsection II.A. The subject of robustness to parametric and dynamic uncertainties in the general nonlinear case is treated in Subsection II.B. Subsection II.C limits the discussion to the linear case to arrive at additional insights.

## A. Control Law Design

INDI-based control laws follow the general principles of feedback linearization, which enables the construction of controllers in the sense of both input-output and full state linearization for either regulation or tracking purposes for arbitrary relative degree [20, 23]. To enunciate the main ideas that makes INDI distinct from its non-incremental counterpart, only a basic derivation of the control law limited to a class of square, input-affine nonlinear dynamical systems with a common relative degree equal to one will be illustrated here. Such a system is closely representative of the equations of motion of the rotational rates of an aircraft, for which INDI has been applied successfully in the past [16]. Accordingly, consider a multi-input multi-output nonlinear system Σ of the form

$$
\Sigma : \left\{ \begin{array} { l } { \dot { { \boldsymbol { x } } } = f ( { \boldsymbol { x } } ) + G ( { \boldsymbol { x } } ) { \boldsymbol { u } } } \\ { { \boldsymbol { y } } = { \boldsymbol { h } } ( { \boldsymbol { x } } ) } \end{array} \right.\tag{1}
$$

described by the state vector ?? $\in \mathbb { R } ^ { n }$ , the input vector ?? $\in \mathbb { R } ^ { m }$ , the observation vector $\boldsymbol { y } \in \mathbb { R } ^ { m }$ , and smooth mappings ?? , ??, and ??. Leveraging the assumption on relative degree stated earlier, the output dynamics can be described as [20]

$$
\dot { \pmb { y } } = \left[ \begin{array} { c } { \mathcal { L } _ { f } h _ { 1 } ( \pmb { x } ) } \\ { \vdots } \\ { \mathcal { L } _ { f } h _ { m } ( \pmb { x } ) } \end{array} \right] + \left[ \begin{array} { c c c } { \mathcal { L } _ { g _ { 1 } } h _ { 1 } ( \pmb { x } ) } & { \hdots } & { \mathcal { L } _ { g _ { m } } h _ { 1 } ( \pmb { x } ) } \\ { \vdots } & { \ddots } & { \vdots } \\ { \mathcal { L } _ { g _ { 1 } } h _ { m } ( \pmb { x } ) } & { \hdots } & { \mathcal { L } _ { g _ { m } } h _ { m } ( \pmb { x } ) } \end{array} \right] \pmb { u } = \alpha ( \pmb { x } ) + \mathcal { B } ( \pmb { x } ) \pmb { u }\tag{2}
$$

where $\mathcal { L } _ { f } h _ { i } ( x )$ and $\mathcal { L } _ { g _ { i } } h _ { i } ( x )$ represent the Lie derivatives of the function $h _ { i }$ along the vectors fields $f$ and $g _ { i }$ with $g _ { i }$ being a column vector of the matrix ?? [23]. For traditional feedback linearization, this expression can be used

directly to construct a control law that linearizes the input-output dynamics to a set of ?? parallel integrators. Assuming that the control effectiveness matrix $\mathcal { B } ( \pmb { x } )$ is invertible,

$$
\pmb { u } = \hat { \mathcal { B } } ^ { - 1 } ( \pmb { x } ) \left[ \pmb { \nu } - \hat { \alpha } ( \pmb { x } ) \right]\tag{3}
$$

where $\hat { \alpha } ( x )$ and $\hat { \mathcal { B } } ( { \boldsymbol x } )$ represent on-board model estimates of $\alpha ( x )$ and $\mathcal B ( \pmb x )$ , respectively, and $\pmb { \nu } \in \mathbb { R } ^ { m }$ is the pseudo-control term that corresponds to the output of an auxiliary control law that has been designed to meet the control objectives. To obtain an analogous control law in incremental form instead, one common approach is to perform a Taylor expansion of the output dynamics around the system state at time $t - \Delta t$ [20, 24], where Δ?? represents the sampling interval. Denoting this condition by the subscript 0 for ease of notation yields the expression

$$
\dot { \pmb { y } } = \dot { \pmb { y } } _ { 0 } +  \frac { \partial [ \alpha ( \pmb { x } ) + \mathcal { B } ( \pmb { x } ) \pmb { u } ] } { \partial \pmb { x } } | _ { 0 } \underbrace { ( \pmb { x } - \pmb { x } _ { 0 } ) } _ { \Delta \pmb { x } } +  \mathcal { B } ( \pmb { x } _ { 0 } ) \underbrace { ( \pmb { u } - \pmb { u } _ { 0 } ) } _ { \Delta \pmb { u } } +  \pmb { R } _ { 1 } \tag{4}
$$

where $\pmb { R } _ { 1 }$ represents the expansion remainder. Consequently, the time-scale separation assumption can be leveraged to design the incremental control input $\Delta \pmb { u }$ , which assumes that all state-dependent and residual terms can be neglected [14, 16, 20, 21]. In practice, this is justified in case high sampling rates and high-bandwidth actuators are available. The control law is completed by adding the control vector $\pmb { u } _ { 0 }$ to the resulting incremental term:

$$
\pmb { u } = \pmb { u } _ { 0 } + \hat { \mathcal { B } } ^ { - 1 } ( { \pmb x } _ { 0 } ) \left[ \pmb { \nu } - \dot { \pmb y } _ { 0 } \right]\tag{5}
$$

Note that compared to its non-incremental counterpart from Equation 3, the resulting control law does not require any model information on $\alpha ( x )$ but uses sensor feedback of the previous control vector and the derivative of the control variable instead. It has been demonstrated by other authors that the incremental form can also be leveraged for the more general class of nonlinear systems that are not affine in the input [24].

## B. Robustness Properties

Let the true output dynamics associated with the system described by Equation 1 be formulated as

$$
\dot { \pmb { y } } = \left[ \hat { \pmb { \alpha } } ( \pmb { x } ) + \pmb { \xi } ( \pmb { x } ) \right] + \left[ \hat { \pmb { \beta } } ( \pmb { x } ) + \pmb { \Xi } ( \pmb { x } ) \right] \pmb { u }\tag{6}
$$

where the mappings $\pmb { \xi }$ and $\Xi$ represent additive regular perturbation terms which, by definition, do not change the order ?? of the system [20]. These terms represent known or unknown variations with respect to the model representations that are embedded in the dynamic inversion control law, and are assumed to be bounded. The system input is modelled as the control law output perturbed by an arbitrary causal perturbation ?? which smoothly maps bounded input signals $z \in \mathbb { R } ^ { m }$ to bounded outputs $\pmb { w } = \pmb { \Delta z } \in \mathbb { R } ^ { m }$ [23]. This perturbation represents a class of unmodelled or neglected dynamics that exist in cascade with the system described by Equation 1, and may be associated with actuation devices or neglected high-order structural modes, for example [7, 9]. In the case of traditional NDI, the input takes the form of

$$
\pmb { u } = ( \pmb { I } + \Delta ) \hat { \mathcal { B } } ^ { - 1 } ( \pmb { x } ) \left[ \pmb { \nu } - \hat { \pmb { \alpha } } ( \pmb { x } ) \right]\tag{7}
$$

Substituting this expression in Equation 6 results in the following description of the closed-loop dynamics [9]:

$$
\dot { \pmb { y } } = \pmb { \nu } + \left( \pmb { \xi } ( \pmb { x } ) + \pmb { \mathcal { D } } ( \pmb { x } , \Delta ) \left[ \pmb { \nu } - \hat { \alpha } ( \pmb { x } ) \right] \right) \triangleq \pmb { \nu } + \epsilon _ { N D I } ( \pmb { x } , \pmb { \nu } , \Delta )\tag{8}
$$

where the term $\mathcal { D } ( \pmb { x } , \pmb { \Delta } )$ is given by

$$
\mathcal { D } ( \pmb { x } , \pmb { \Delta } ) = \Xi ( \pmb { x } ) \hat { \mathcal { B } } ^ { - 1 } ( \pmb { x } ) + \hat { \mathcal { B } } ( \pmb { x } ) \Delta \hat { \mathcal { B } } ^ { - 1 } ( \pmb { x } ) + \Xi ( \pmb { x } ) \pmb { \Delta } \hat { \mathcal { B } } ^ { - 1 } ( \pmb { x } )\tag{9}
$$

The residual term $\epsilon _ { N D I }$ is in analogy with the terminology introduced in [20], and represents the closed-loop residual dynamics that emerge due to non-ideal dynamic inversion in the presence of regular and dynamic perturbations. In consequence of the assumptions made earlier on the boundedness of the individual terms, $\epsilon _ { N D I }$ has an upper bound $\bar { \epsilon } _ { N D I }$ under bounded virtual control ??. However, its magnitude can be relatively large, which indicates poor robustness properties. This is a widespread concern for control laws based on traditional NDI [10, 20, 23].

Considering the incremental form, a more general situation is considered to be able to analyze the effect of multiple dynamic perturbations in different locations of the feedback system. In line with the early work by Smith [12], writing $\pmb { u } _ { 0 } = \left( \pmb { I } + \pmb { \Delta } _ { 1 } \right)$ ?? and ${ \dot { \pmb y } } _ { 0 } = \left( { \pmb I } + \Delta _ { 2 } \right) { \dot { \pmb y } }$ , with $\Delta _ { i }$ representing smooth, causal mappings as before, and assuming that $\scriptstyle { \boldsymbol { x } } _ { 0 } = { \boldsymbol { x } }$ the system input associated with the INDI-based control law given by Equation 5 can be expressed as

$$
\pmb { u } = \left( \pmb { I } + \Delta _ { 1 } \right) \pmb { u } + \hat { \pmb { \mathcal { B } } } ^ { - 1 } ( \pmb { x } ) \left[ \pmb { \nu } - \left( \pmb { I } + \Delta _ { 2 } \right) \dot { \pmb { y } } \right]\tag{10}
$$

This form can be used to analyze a broad class of perturbation configurations: in particular, it enables direct analysis of the synchronization effect, which is notorious for incremental control laws and has been reported repeatedly in the literature [15, 16, 21]. By substituting the relationship between ?? and ${ \dot { \mathbf { y } } } ,$ , which is directly obtained from Equation $^ { 6 , }$ a n explicit description of the closed-loop output dynamics can be found:

$$
\begin{array} { r l } & { \dot { \pmb { y } } = \pmb { \nu } + \left( I - \mathcal { D } _ { 1 } ( \pmb { x } , \Delta _ { 1 } ) + \Delta _ { 2 } \right) ^ { - 1 } \left[ \mathcal { D } _ { 1 } ( \pmb { x } , \Delta _ { 1 } ) \left( \pmb { \nu } - \left[ \hat { \alpha } ( \pmb { x } ) + \pmb { \xi } ( \pmb { x } ) \right] \right) - \Delta _ { 2 } \pmb { \nu } \right] } \\ & { \quad \triangleq \pmb { \nu } + \pmb { S } ( \pmb { x } , \Delta _ { 1 } , \Delta _ { 2 } ) ^ { - 1 } \left[ \mathcal { D } _ { 1 } ( \pmb { x } , \Delta _ { 1 } ) \left( \pmb { \nu } - \left[ \hat { \alpha } ( \pmb { x } ) + \pmb { \xi } ( \pmb { x } ) \right] \right) - \Delta _ { 2 } \pmb { \nu } \right] } \\ & { \quad \triangleq \pmb { \nu } + \epsilon _ { I N D I } ( \pmb { x } , \pmb { \nu } , \Delta _ { 1 } , \Delta _ { 2 } ) } \end{array}\tag{11}
$$

where $\mathcal { D } _ { 1 } ( \pmb { x } , \pmb { \Delta } _ { 1 } )$ is given by

$$
\mathcal { D } _ { 1 } ( x , \Delta _ { 1 } ) = \hat { \mathcal { B } } ( { \pmb x } ) \Delta _ { 1 } \left( \hat { \mathcal { B } } ( { \pmb x } ) + \Xi ( { \pmb x } ) \right) ^ { - 1 }\tag{12}
$$

This result sheds light on several robustness properties. In particular, if ?? and $\dot { \pmb { y } }$ can be accurately measured, the closed-loop system will be highly robust against regular perturbations in the output dynamics. The assumption that the control effectiveness matrix and its on-board model estimate are invertible should be emphasized here. If the L2-gain of the perturbations $\Delta _ { i }$ is expressed as $\gamma _ { i }$ and it holds that

$$
\gamma _ { 1 } \left\| \hat { \mathcal { B } } ( \pmb { x } ) \right\| \left\| \left[ \hat { \mathcal { B } } ( \pmb { x } ) + \Xi ( \pmb { x } ) \right] ^ { - 1 } \right\| + \gamma _ { 2 } < 1\tag{13}
$$

then, the upper bound on the $\mathcal { L } _ { 2 }$ norm of the residual term $\epsilon _ { I N D I }$ can be shown to be

$$
\| \epsilon _ { I N D I } \| \leq \frac { \Big ( \gamma _ { 1 } \left\| \hat { \mathcal { B } } ( x ) \right\| \left\| \left[ \hat { \mathcal { B } } ( x ) + \Xi ( x ) \right] ^ { - 1 } \right\| + \gamma _ { 2 } \Big ) \left\| \nu \right\| + \gamma _ { 1 } \left\| \hat { \mathcal { B } } ( x ) \right\| \left\| \left[ \hat { \mathcal { B } } ( x ) + \Xi ( x ) \right] ^ { - 1 } \right\| \left\| \hat { \alpha } ( x ) + \xi ( x ) \right\| } { 1 - \gamma _ { 1 } \left\| \hat { \mathcal { B } } ( x ) \right\| \left\| \left[ \hat { \mathcal { B } } ( x ) + \Xi ( x ) \right] ^ { - 1 } \right\| - \gamma _ { 2 } }\tag{14}
$$

Consequently, it holds that the upper bound on $\| \epsilon _ { I N D I } \|$ goes towards zero as $\gamma _ { i } \to 0$ , independently of $\xi ( x )$ and $\Xi ( x )$ . This illustrates the relative benefit of INDI over its non-incremental counterpart. However, the condition given by Equation 13 is overly conservative and cannot be met in practice due to the presence of actuator dynamics and finite sampling times, for example. Therefore, the relative time-scale properties of Σ and the perturbations should be considered in general. Along this line, it has been shown by Wang et al. [25] that $\epsilon _ { I N D I }$ remains bounded independently of $\xi ( { \pmb x } ) \ \mathrm { i f } \ \big | \big | \Xi ( { \pmb x } ) \hat { \mathcal B } ( { \pmb x } ) ^ { - 1 } \big | \big | < \bar { 1 }$ in case the digital nature of the control system is considered.

Equation 11 also shows that the mapping $s$ needs to be non-singular for $\epsilon _ { I N D I }$ to remain bounded, which limits the permissible perturbation dynamics. This is a direct manifestation of the synchronization effect and requires careful consideration by the designer. Considering the extreme case where $\Delta _ { 1 }$ is zero, for example, $\epsilon _ { I N D I }$ remains bounded if and only if $\Delta _ { 2 } \neq - I$ . This is in contrast with traditional NDI, for which an upper bound on the inversion residual can always be found under the assumptions stated earlier. To ensure boundededness and therefore stability of an INDI-based control system, a possible design solution is to ensure that $\mathcal { D } _ { 1 } ( x , \Delta _ { 1 } ) = \Delta _ { 2 }$ , in which case it holds that ${ \pmb S } = { \pmb I }$ and

$$
\epsilon _ { I N D I } ( x , \Delta _ { 2 } ) = - \Delta _ { 2 } \left[ \hat { \alpha } ( { \pmb x } ) + { \pmb \xi } ( { \pmb x } ) \right]\tag{15}
$$

This idea will be referred to as the matching strategy and is in line with the main design philosophy that has been adopted in the past [15, 16, 21]. With this procedure, the inversion residual will be nonzero for nonzero $\Delta _ { 2 }$ . Nevertheless, the norm of $\epsilon _ { I N D I }$ will be small if $\Delta _ { 2 }$ is small in magnitude in the operating time-scale of $\Sigma ,$ which is in line with the time-scale separation assumption that underlies the derivation of the control law.

## C. Linear State Space Formulation

The basic robust stability and performance properties of INDI can be further understood by analyzing the state space representation of the closed-loop dynamics in case the plant Σ, the perturbations $\Delta _ { i } .$ , and the control law are considered to be linear. Accordingly, the linear open-loop system will be considered in normal form [26]:

$$
\boldsymbol { \Sigma } : \left\{ \begin{array} { r l } { \dot { \boldsymbol { \xi } } } & { = \boldsymbol { R } \boldsymbol { \xi } + \boldsymbol { T } \boldsymbol { \eta } + \boldsymbol { B } \boldsymbol { u } } \\ { \dot { \boldsymbol { \eta } } } & { = \boldsymbol { P } \boldsymbol { \xi } + \boldsymbol { Q } \boldsymbol { \eta } } \\ { \boldsymbol { y } } & { = \boldsymbol { \xi } } \end{array} \right.\tag{16}
$$

Writing the perturbation interconnections explicitly and considering the incremental dynamic inversion (IDI) control law in linear form, it follows that Equation 11 is equivalently described by

$$
\left[ \begin{array} { c } { \dot { \xi } } \\ { \dot { \eta } } \\ { \epsilon _ { I D I } } \\ { z _ { 1 } } \\ { z _ { 2 } } \end{array} \right] = \left[ \begin{array} { c c c c c c } { \textbf { 0 } } & { \textbf { 0 } } & { I } & { \hat { B } } & { - I } \\ { P } & { Q } & { \textbf { 0 } } & { \textbf { 0 } } & { \textbf { 0 } } \\ { \textbf { 0 } } & { \textbf { 0 } } & { \textbf { 0 } } & { \hat { B } } & { - I } \\ { - B ^ { - 1 } R } & { - B ^ { - 1 } T } & { B ^ { - 1 } } & { B ^ { - 1 } \hat { B } } & { - B ^ { - 1 } } \\ { \textbf { 0 } } & { \textbf { 0 } } & { I } & { \hat { B } } & { - I } \end{array} \right] \left[ \begin{array} { c } { \xi } \\ { \eta } \\ { \nu } \\ { w _ { 1 } } \\ { w _ { 2 } } \end{array} \right] , \qquad \left[ \begin{array} { c } { w _ { 1 } } \\ { w _ { 2 } } \end{array} \right] = \left[ \begin{array} { c c } { \Delta _ { 1 } } & { \textbf { 0 } } \\ { \Delta _ { 2 } } \end{array} \right] \left[ \begin{array} { c } { z _ { 1 } } \\ { z _ { 2 } } \end{array} \right]\tag{17}
$$

The resulting formulation enables direct verification of the stability of the closed-loop system and the performance properties of the mapping from ?? to $\epsilon _ { I D I }$ for all perturbations in the uncertainty set. Likewise, if $\Delta _ { i }$ represents a series interconnection of two dynamical systems $\Gamma _ { i }$ and $\bar { \Delta } _ { i } ,$ where it is known that $\left\| \dot { \bar { \Delta } } _ { i } \right\| \leq 1$ and

$$
\Gamma _ { i } : \left\{ \begin{array} { r l } { \dot { \pmb q } _ { i } } & { { } = A _ { i } \pmb q _ { i } + B _ { i } \pmb z _ { i } } \\ { \bar { \pmb z } _ { i } } & { { } = C _ { i } \pmb q _ { i } + D _ { i } \pmb z _ { i } } \end{array} \right.\tag{18}
$$

then, Equation 17 extends to

$$
\left[ \begin{array} { c } { \dot { \xi } } \\ { \dot { q } } \\ { \dot { q } } \\ { \dot { \epsilon } _ { I D I } } \\ { \dot { \tau } _ { 1 D I } } \\ { \dot { z } _ { 1 } } \\ { \dot { z } _ { 2 } } \end{array} \right] = \left[ \begin{array} { c c c c c c c } { 0 } & { 0 } & { 0 } & { 0 } & { 0 } & { \hat { \textbf {  { f } } } } & { \hat { \hat { \boldsymbol { B } } } } & { - I } \\ { \boldsymbol { P } } & { \boldsymbol { Q } } & { \hat { \textbf {  { 0 } } } } & { 0 } & { 0 } & { 0 } \\ { - R _ { 1 } \boldsymbol { B } ^ { - 1 } \boldsymbol { R } } & { - \boldsymbol { B } _ { 1 } \boldsymbol { B } ^ { - 1 } \boldsymbol { T } } & { \hat { A } _ { 1 } } & { \hat { \textbf {  { 0 } } } } \\ { \frac { \boldsymbol { Q } } { 0 } } & { 0 } & { 0 } & { A _ { 2 } } & { B _ { 2 } \hat { \textbf {  { B } } } } & { B _ { 2 } \hat { \textbf {  { B } } } } & { - \boldsymbol { B } _ { 2 } } \\ { 0 } & { 0 } & { 0 } & { 0 } & { 0 } & { \hat { \textbf {  { B } } } } & { \hat { \hat { \boldsymbol { B } } } } \\ { - \boldsymbol { D } _ { 1 } \boldsymbol { B } ^ { - 1 } \boldsymbol { R } } & { - \boldsymbol { D } _ { 1 } \boldsymbol { B } ^ { - 1 } \boldsymbol { T } } & { \boldsymbol { C } _ { 1 } } & { 0 } & { 0 } \\ { 0 } & { 0 } & { 0 } & { \hat { \textbf {  { C } } _ { 2 } } } & { \boldsymbol { D } _ { 2 } \hat { \boldsymbol { B } } } & { - \boldsymbol { D } _ { 2 } } \end{array} \right] \left[ \begin{array} { c } { \xi } \\ { \eta } \\ { q _ { 1 } } \\ { q _ { 2 } } \\ { \nu } \\ { \nu _ { 1 } } \\ { \nu _ { 2 } } \end{array} \right] .\tag{19}
$$

In this context, $\Gamma _ { i }$ represents an uncertainty weight that places an upper bound on the $\mathcal { L } _ { 2 }$ -gain of ??. This expression is an example of the basic framework that enables evaluation of the linear robustness properties of IDI-based control systems and is fundamental to the analyses presented in Sections III and IV. As a special case, consider the situation where $\bar { \Delta } _ { i }$ is described by a deterministic (unity) input-output mapping. This implies that the perturbation dynamics can be described by a known form. Then, the closed-loop dynamics can be expressed as

$$
\left[ \begin{array} { c } { \dot { \xi } } \\ { \dot { \eta } } \\ { \dot { q } _ { 1 } } \\ { \dot { q } _ { 2 } } \\ { \epsilon _ { I D I } } \end{array} \right] = \left[ \begin{array} { c c c c c } { - S ^ { - 1 } \tilde { D } _ { 1 } R } & { - S ^ { - 1 } \tilde { D } _ { 1 } T } & { S ^ { - 1 } \hat { B } C _ { 1 } } & { - S ^ { - 1 } C _ { 2 } } & { S ^ { - 1 } } \\ { P } & { Q } & { 0 } & { 0 } \\ { - B _ { 1 } \left( S B \right) ^ { - 1 } E _ { 1 } R } & { - B _ { 1 } \left( S B \right) ^ { - 1 } E _ { 1 } T } & { A _ { 1 } + B _ { 1 } \tilde { S } ^ { - 1 } C _ { 1 } } & { - B _ { 1 } \left( S B \right) ^ { - 1 } C _ { 2 } } \\ { - B _ { 2 } S ^ { - 1 } \tilde { D } _ { 1 } R } & { - B _ { 2 } S ^ { - 1 } \tilde { D } _ { 1 } T } & { B _ { 2 } S ^ { - 1 } \hat { B } C _ { 1 } } & { A _ { 2 } - B _ { 2 } S ^ { - 1 } C _ { 2 } } & { B _ { 2 } S ^ { - 1 } } \\ { - S ^ { - 1 } \tilde { D } _ { 1 } R } & { - S ^ { - 1 } \tilde { D } _ { 1 } T } & { S ^ { - 1 } \hat { B } C _ { 1 } } & { - S ^ { - 1 } C _ { 2 } } & { S ^ { - 1 } - I } \end{array} \right] \left[ \begin{array} { c } { \xi } \\ { \eta } \\ { q _ { 1 } } \\ { q _ { 2 } } \\ { \eta } \end{array} \right]\tag{20}
$$

where

$$
S \triangleq I - \tilde { D } _ { 1 } + D _ { 2 } , \tilde { S } \triangleq \hat { B } ^ { - 1 } S B , \tilde { D } _ { 1 } \triangleq \hat { B } D _ { 1 } B ^ { - 1 } , E _ { 1 } \triangleq \tilde { D } _ { 1 } + S\tag{21}
$$

Regarding the synchronization effect, several conclusions can be drawn from this result. Consider the situation where ??2 arise because of proper dynamics $K _ { 2 }$ in the feedback path, which is often the case in practice. Then, it holds that $\Gamma _ { 2 } = K _ { 2 } - I ,$ , which implies $D _ { 2 } = - I .$ . Consequently, it follows that ?? is singular if $\pmb { D _ { 1 } }$ is zero, which leads to the conclusion that $\epsilon _ { I D I }$ will be undetermined (infinitely large in the limit) in the absence of any compensating perturbation dynamics $\mathbf { { { I } _ { 1 } } }$ . As a second observation, consider the case where it holds that $\Gamma _ { 1 } = K _ { 1 } - I$ in addition, with $K _ { 1 }$ again proper, such that $D _ { 1 } = - I$ . Then, Equation 20 reduces to

$$
\left[ \begin{array} { c } { \dot { \xi } } \\ { \dot { \eta } } \\ { \dot { q } _ { 1 } } \\ { \dot { q } _ { 2 } } \\ { \epsilon _ { I D I } } \end{array} \right] = \left[ \begin{array} { c c c c c } { R } & { T } & { B C _ { 1 } } & { - K _ { B } C _ { 2 } } & { K _ { B } } \\ { P } & { Q } & { 0 } & { 0 } \\ { 0 } & { 0 } & { A _ { 1 } + B _ { 1 } C _ { 1 } } & { - B _ { 1 } B ^ { - 1 } K _ { B } C _ { 2 } } \\ { B _ { 2 } R } & { B _ { 2 } T } & { B _ { 2 } B C _ { 1 } } & { A _ { 2 } - B _ { 2 } K _ { B } C _ { 2 } } & { - B _ { 2 } K _ { B } } \\ { R } & { T } & { B C _ { 1 } } & { K _ { B } C _ { 2 } } & { K _ { B } - I } \end{array} \right] \left[ \begin{array} { c } { \xi } \\ { \eta } \\ { q _ { 1 } } \\ { q _ { 2 } } \\ { \eta } \end{array} \right]\tag{22}
$$

where $K _ { B } \triangleq B \hat { \pmb { B } } ^ { - 1 }$ . This result enables direct verification of the robust stability and performance properties of $\epsilon _ { I D I }$ when subjected to parametric uncertainties in $\mathbf { { K } _ { 1 } } , \mathbf { { K } _ { 2 } } .$ , and the system control effectiveness matrix ??. Of special interest is the lower-right part of the state transition matrix, which provides direct insight into the properties of the eigenmodes associated with synchronization:

$$
\Psi \triangleq \left[ \begin{array} { c c } { A _ { 1 } + B _ { 1 } C _ { 1 } } & { - B _ { 1 } B ^ { - 1 } K _ { B } C _ { 2 } } \\ { B _ { 2 } B C _ { 1 } } & { A _ { 2 } - B _ { 2 } K _ { B } C _ { 2 } } \end{array} \right]\tag{23}
$$

If the characteristic roots of ?? are sufficiently distant from the eigenmodes of $\Sigma ,$ which can be argued to hold if the time-scale separation assumption holds, the synchronization dynamics of the closed-loop system can be assessed independently. Furthermore, if Σ and the control law do not feature any coupling between input-output channels, which holds for diagonal control systems, using an appropriate change of variables enables direct assessment of the synchronization dynamics without any information of Σ. In this case, isolating individual channels, ?? reduces to

$$
\bar { \Psi } _ { S I S O } \triangleq \left[ \begin{array} { c c } { { A _ { 1 } + B _ { 1 } C _ { 1 } } } & { { - k _ { B } B _ { 1 } C _ { 2 } } } \\ { { B _ { 2 } C _ { 1 } } } & { { A _ { 2 } - k _ { B } B _ { 2 } C _ { 2 } } } \end{array} \right]\tag{24}
$$

where the scalar nature of each channel is reflected by the scalar scaling factor $k _ { B } .$ . This simple result can be used to immediately verify the impact of proper high-order dynamics on $\epsilon _ { I D I }$ . Since the scaling factor cannot be factored out from the expression, the synchronization dynamics will be affected if the control effectiveness undergoes unmodeled variations. This is independent from how $K _ { 1 }$ and $\pmb { K } _ { 2 }$ are selected.

## III. Basic Incremental Dynamic Inversion Robustness Assessment

The discussion from the previous Section is continued by a numerical robustness assessment and comparison of linear INDI (IDI)-based flight control laws. The scope is limited to a single flight condition that is well within the interior of the service flight envelope. The case study is described in Subsection III.A, which gives an overview of the control system elements and the design requirements. Both Dynamic Inversion (DI) and IDI designs are discussed, for which the results are described in Subsections III.B and III.C, respectively.

## A. Problem formulation

The numerical evaluation is based on a linear dynamic inversion (DI) based pitch rate control law design for an open-access simulation model of the General Dynamics F-16 [27, 28] and is closely in line with the analysis presented in [7]. The non-linear aircraft model has been trimmed and linearized around a flight condition of Mach 0.5 at 10,000 feet altitude, with the center of gravity located at 38% relative to the mean aerodynamic chord (MAC), resulting in a trim angle-of-attack of 2.6 degrees. To simplify the analysis, the linear model has been reduced to the short-period mode only. It is assumed that the horizontal tailplane is moved symmetrically by an actuation system that can be represented by a first-order lag with a time constant of 1/60 seconds. In addition, it is assumed that measurements of pitch rate, pitch acceleration, angle-of-attack, and horizontal tail position are all available to the control law.

<!-- image-->
Fig. 1 Control system interconnection structure

The bare airframe and flight control system hardware are subject to both regular and dynamic uncertainties. The pitch rate equations of motion are affected by variations in the angle of attack stability derivative $M _ { \alpha }$ and control effectiveness $M _ { \delta _ { h } }$ , which may deviate up to $\pm 7 5 \%$ and ±25% from their nominal values, respectively. These parametric variations are denoted by ??. Multiplicative dynamic uncertainties are present at the actuator (act) and bare airframe (ba) input channels and serve as lumped representations of any high-order, additional dynamics that may be present in the control system. These may be caused by processing delays, elastic deformation of the actuation mechanism or airframe, or sensor uncertainties, for example. In line with [7], the following uncertainty weights are selected:

$$
W _ { a c t } ( s ) = W _ { b a } ( s ) = K _ { 0 } \frac { \tau _ { l a g } } { \tau _ { l e a d } } \frac { s + \tau _ { l e a d } } { s + \tau _ { l a g } }\tag{25}
$$

with $K _ { 0 } = 0 . 1 , \tau _ { l a g } = 5$ , and $\tau _ { l e a d } = 1 0 0$ seconds, respectively. With these weights, each uncertainty imposes an upper bound of 10% model error in steady state conditions and grows beyond 100% error at frequencies above 50 rad/s.

The control system is subject to a range of performance requirements in terms of tracking error and disturbance rejection, which shall be robustly met given the uncertainties present in the system. A primary design goal is to ensure that the closed-loop short-period response follows the desired dynamics, which is based on existing flying quality requirements that can be found in airworthiness standards and guidelines such as MIL-STD-1797 [2]. Here, the desired dynamics are limited to the small-amplitude pitch rate response to stick input only:

$$
W _ { r e f } ( s ) = \frac { q _ { r e f } ( s ) } { \delta _ { l o n } ( s ) } = \frac { K _ { q } ( T _ { \theta _ { 2 } } s + 1 ) } { s ^ { 2 } + 2 \zeta _ { s p } \omega _ { s p } s + \omega _ { s p } ^ { 2 } }\tag{26}
$$

The desired values are set to $T _ { \theta _ { 2 } } = 1 . 0$ seconds, $\omega _ { s p } = 4 . 0$ rad/s, and $\zeta _ { s p } = 1 . 0$ , which is in line with Level-1 flying quality specifications for category A flight phases for this class of aircraft [2]. The tracking error between the desired and actual pitch rate response is penalized by a weighting filter $W _ { T E }$ , which takes the form of

$$
W _ { t e } ( s ) = \frac { \omega _ { x } } { s + K _ { 0 } \omega _ { x } } + K _ { \infty } ^ { - 1 }\tag{27}
$$

where $\omega _ { x } = 1$ rad/s represents the gain crossover frequency, and $K _ { 0 } = 0 . 2$ and $K _ { \infty } = 3 0$ determine the steady-state and high-frequency tracking error bounds, respectively. Exogenous atmospheric disturbances are appropriately shaped using the following weighting filter, which represents a first-order approximation of the Dryden gust model [7]:

$$
W _ { d i s t } ( s ) = \left( \frac { 1 8 0 } { \pi } \frac { 1 } { V _ { t _ { 0 } } } \right) \frac { 4 } { s + 0 . 2 }\tag{28}
$$

In addition to the dynamic inversion loop, the control law consists of a two degree-of-freedom (DOF) linear control structure to achieve accurate tracking of pilot stick commands. It includes a proportional-integral (PI) feedback regulator for stabilization and disturbance rejection, a command reference model that describes the desired dynamics, and a feedforward path that enables more accurate tracking of the reference signal. The regulator gains assume constant values of $K _ { P } = 7 . 0$ and $K _ { I } = 2 . 0$ and have been designed manually, with the all-loops-broken crossover frequency and the phase lag induced by the integrator as main design goals. Figure 1 shows the block diagram representation of the control system interconnection structure∗. Note that control allocation is furnished simply by inversion of the nominal control effectiveness term $M _ { \delta _ { h } }$

## B. Dynamic Inversion

The dynamic inversion variant of the control law makes use of a model-based estimate of the angular acceleration,

$$
\hat { \dot { q } } = \hat { m } _ { \alpha } \alpha + \hat { m } _ { q } q + \hat { m } _ { \delta _ { h } } \delta _ { h }\tag{29}
$$

where the on-board representations of the short-period stability and control derivatives correspond to the unperturbed airframe dynamics. With this setup, the synchronization compensation loop will cancel out the contribution of the horizontal tail position in the control law. It is assumed that the angle-of-attack measurement does not contain any additional uncertainty associated with imperfect air data measurements. Figures 2a and 2b visualize the nominal step response and all-loops broken (path A) frequency response of the resulting control architecture.

In order to quantify robust stability and performance levels, the interconnection machinery described in Subsection II.C is applied to describe the generalized DI-based control system. Given the structured and mixed nature of the uncertainty formulation, the structured singular value (??) is used as the main analysis instrument [29]. The Matlab® Robust Control Toolbox™ (version 6.9) serves as the primary software tool [30]. The upper and lower bounds of ?? as returned by the ????-scaling procedure are shown in Figures 2c and 2d. The results show that whereas robust stability is met for the entire uncertainty set, the performance requirements are not. In the low-to-medium frequency region, the robust performance deficit is dominated by parametric uncertainty in the stability and control derivatives; by contrast, robust stability and performance levels at medium-to-high frequencies are constrained by the dynamic actuator and airframe uncertainties Δ. Note that increasing the outer loop gain improves robustness to aerodynamic uncertainty at the cost of robust stability at high frequencies.

<!-- image-->

<!-- image-->

(a) Step response
<!-- image-->
(c) Stability and performance margins

(b) Broken-loop frequency response
<!-- image-->
(d) Robust performance breakdown
Fig. 2 Overview of basic Dynamic Inversion-based (DI) control system properties

## C. Incremental Dynamic Inversion

The above analysis is repeated to examine the robustness properties of a basic variant of the incremental dynamic inversion architecture. This implementation does not include any additional processing of the horizontal tail position and angular acceleration measurements, which implies that the synchronization compensation and acceleration estimation blocks in Figure 1 are unity in this case. Figures 3a and 3b show the step response and broken-loop frequency responses of the basic IDI-based control law. These are equivalent to the model-based DI control system, except for the broken-loop response after closing the actuator feedback loop(s). The latter reveals that applying IDI effectively results in a high-gain control system at the level of the bare airframe, which explains its robustness to aerodynamic uncertainties. This observation is confirmed by the ??-analysis, as shown in Figures 3c and 3d. Attainable robust performance levels are significantly improved in the low-to-medium frequency range compared to DI, which can be directly attributed to the increased robustness to aerodynamic variations. However, this comes at the cost of robust stability, which cannot be met as a result of the dynamic actuator and airframe uncertainties and represents a direct manifestation of the synchronization effect described in Sections II.B and II.C. This result also confirms that outer loop regulator gains could be reduced when INDI is adopted, which has been reported earlier in the literature [16].

<!-- image-->
(a) Step response

<!-- image-->
(b) Broken-loop frequency response

<!-- image-->
(c) Stability and performance margins

<!-- image-->
(d) Robust performance breakdown
Fig. 3 Overview of basic Incremental Dynamic Inversion-based (IDI) control system properties

## IV. ??-optimal Robust Incremental Dynamic Inversion Augmentation Techniques

In this Section, mixed ??-synthesis of the incremental inversion loop is used in the F-16 short-period design problem to identify and assess design strategies that augment the basic incremental dynamic inversion loop such that ??-optimal robust performance levels are achieved. Three principal design degrees-of-freedom form the basis of the discussion. Synchronization of filter design elements is considered first in Subsection IV.A, where full-order synthesis is used to leverage full design flexibility. This is followed by an investigation of filter structure in Subsection IV.B, where the scope is limited to synchronous designs. In addition, Subsection IV.C concentrates on alternative architectures that use supplemental model information to further augment the basic incremental inversion loop. Subsection IV.D concludes the discussion.

## A. Synchronization Compensation

The preceding Sections have made evident that adequate compensation of the synchronization effect forms a key factor in the design of incremental control laws. Subsection II.B explained the relevance of the mapping S and introduced the matching procedure as a simple, but effective method to ensure stability. In case of uncertainty, accurately matching perturbing dynamics is improbable. Therefore, it is of interest to assess how the synchronization effect shall be accounted for in a ??-optimal fashion in light of the regular and dynamic uncertainties and performance requirements that define the control system design problem. In this view, the following two design architectures are considered:

$$
\tilde { u } _ { f } ^ { ( a s y n c ) } = K ( \dot { q } _ { 0 } , \delta _ { h _ { 0 } } ) , \qquad \tilde { u } _ { f } ^ { ( s y n c ) } = K ( \tilde { u } _ { 0 } )\tag{30}
$$

where ?? represents the feedback filter augmentation system and $\tilde { u } _ { 0 }$ forms the basic combined IDI feedback signal,

<!-- image-->

<!-- image-->

(a) Broken-loop frequency response (asynchronous)
<!-- image-->
(c) Stability and performance margins (asynchronous)

(b) Broken-loop frequency response (synchronous)
<!-- image-->
(d) Stability and performance margins (synchronous)
Fig. 4 Full-order ??-optimal augmented IDI-based control system properties

$$
\tilde { u } _ { 0 } = \frac { 1 } { \hat { m } _ { \delta _ { h } } } \dot { q } _ { 0 } - \delta _ { h _ { 0 } }\tag{31}
$$

In the asynchronous variant, the control law has been configured such that the angular acceleration and horizontal tail position feedback filter design elements can be designed independently. By contrast, the synchronous variant represents a more constrained architecture in the sense that matching of the filter dynamics is enforced. By doing this comparison, an insight into the relative limitations of the synchronous implementation, which has seen successful applications in the past, can be obtained.

Mixed ??-synthesis with ??????-iteration [29] from the Matlab® Robust Control Toolbox™ [30] is used to obtain the results presented in Figure 4. A brief summary of the most relevant information can also be found in Table 1. The asynchronous and synchronous high-order control systems returned by the optimizer are obtained after 10 and 8 iterations, respectively. Figures 4c and 4d show that both design variants yield solutions that satisfy the performance requirements in a robust sense, with peak $\bar { \mu }$ not exceeding the unity threshold value. Examining the diagrams more closely reveals that the optimized robust performance levels obtained for either implementation are in fact very close, which implies that the optimizer did not manage to extract more performance from the additional design degree-of-freedom in the asynchronous case. Consequently, imposing synchronous filter dynamics appears not to impose severe limitations on the robust performance potential of the IDI-based control law, which confirms that the application of the matching strategy to filter design forms a reasonable design philosophy for managing the synchronization effect.

It is of additional interest to examine the SISO broken-loop response of the synchronous variant in Figure 4b. Compared to the basic design as shown in Figure 3b, it is evident that the all-loops broken responses are equivalent and that the loop shape after closing the actuator loop reflects the principal trade-offs associated with the design problem. In particular, the gain crossover frequency $\omega _ { c }$ has moved to a lower region, while the low-to-medium frequency high gain characteristics that govern the robustness of the IDI control law against real perturbations have been largely maintained. Adequate phase margin in the crossover region has also been preserved.

## B. Low-order Synchronous Designs

The control laws returned by the automated synthesis procedure presented in the previous Subsection are of very high order, which is a known characteristic of the ??????-iteration procedure. In this light, low-order, fixed structure filter designs that can be implemented in practice need to be considered instead. The Matlab ® Robust Control Toolbox™ offers the possibility to optimize fixed-order controller structures through the use of tunable control design blocks [30], which is the procedure followed here. In view of the preceding discussion, the synchronous architecture will be adopted. The following fixed structures are selected, which correspond to second-order lag-lead and low-pass filters:

$$
K ( s ) = { \frac { \tilde { u } _ { f } ( s ) } { \tilde { u } _ { 0 } ( s ) } } = K _ { 1 } ( s ) = { \frac { u _ { f } ( s ) } { u _ { 0 } ( s ) } } = K _ { 2 } ( s ) = { \frac { \dot { q } _ { f } ( s ) } { \dot { q } _ { 0 } ( s ) } } = { \frac { a _ { 2 } s ^ { 2 } + a _ { 1 } s + a _ { 0 } } { s ^ { 2 } + b _ { 1 } s + b _ { 0 } } }\tag{32}
$$

$$
K ( s ) = { \frac { \tilde { u } _ { f } ( s ) } { \tilde { u } _ { 0 } ( s ) } } = K _ { 1 } ( s ) = { \frac { u _ { f } ( s ) } { u _ { 0 } ( s ) } } = K _ { 2 } ( s ) = { \frac { \dot { q } _ { f } ( s ) } { \dot { q } _ { 0 } ( s ) } } = { \frac { \omega _ { n } ^ { 2 } } { s ^ { 2 } + { \sqrt { 2 } } \omega _ { n } s + \omega _ { n } ^ { 2 } } }\tag{33}
$$

The reason for selecting these forms is that they are closely in line with previous successful designs [15, 16, 21]. The lag-lead architecture offers considerably more flexibility compared to the low-pass design, for which only break frequency $\omega _ { n }$ forms the available tuning parameter. The mixed ??-synthesis results are presented in Figure 5.

With both designs, the performance requirements are met for the entire uncertainty set. However, the peak $\bar { \mu }$ of the low-pass architecture is higher compared to the lag-lead form, which shows very similar performance levels when compared to the full-order designs presented earlier. This forms a direct consequence of the additional design flexibility and is also reflected by the broken-loop response diagrams in Figures 5c and 5d, which show that both the gain crossover frequency and phase margins are lower for the low-pass filter design option. Comparing both optimized filter designs in Figure 5b leads to the conclusion that the smaller phase "reach back" associated with the lag-lead variant represents a key attribute. This demonstrates that mitigating phase distortions in the rigid-body frequency range should be considered as a design guideline, which is in line with the time-scale separation assumption that underlies the derivation of the control law. Still, these results also confirm that using synchronous low-pass filters should be viewed as a reasonable strategy when designing incremental dynamic inversion-based control laws.

<!-- image-->
(a) Step responses

<!-- image-->
(b) Optimized filter designs

<!-- image-->
(c) Broken-loop frequency response (lag-lead)

<!-- image-->
(d) Broken-loop frequency response (low-pass)

<!-- image-->
(e) Stability and performance margins (lag-lead)

<!-- image-->
(f) Stability and performance margins (low-pass)
Fig. 5 Overview of low-order ??-optimal augmented IDI-based control system properties

## C. Reinstating Model Information

Although the matching strategy to filter design is shown to be successful in improving the overall robustness properties of IDI-based control laws, this comes at the cost of an enlarged inversion residual even in the nominal situation. This raises the question if and to what extent this disadvantage can be mitigated. Equation 15 suggests that the inversion residual can be further decreased by reintroducing model information of the bare airframe dynamics in the form of a complementary augmentation element. This approach is formulated in [31] as hybrid INDI. It is of interest to investigate how this design method can improve upon the preceding sensor-based inversion strategies. Additionally, there may be circumstances where direct actuator measurements may not be available, which implies that the control system architecture from Figure 1 does not apply. In this case, another form of input signal feedback needs to be found. An internal model representation of the nominal actuator dynamics $H _ { a } ( s )$ would provide a solution here, which has been introduced in the past [15]. This strategy is illustrated in Figure 6, together with a closely related design option that adopts direct control command feedback instead. Both variants will be compared in performance to the designs presented earlier.

<!-- image-->
Fig. 6 Alternative synchronization compensation strategies

First, the hybrid inversion strategy is investigated. Any information that is distorted or lost as a result of the filter feedback augmentation system ?? is compensated for by adding complementary model information of the plant dynamics. Using this design method, it can be shown that, in the nominal case where the control effectiveness is known and the synchronous perturbation mapping $\Delta _ { 2 }$ is fully compensated, the inversion residual expressed by Equation 15 reduces to

$$
\epsilon _ { I N D I } ( x , \Delta _ { 2 } ) = - \Delta _ { 2 } \xi ( x )\tag{34}
$$

The sensitivity of this outcome in the presence of uncertainty is again considered by analyzing $\mu .$ Based on the insights from [31], the following design is used for the F-16 design problem considered here:

$$
\dot { q } _ { f b } ( s ) = K ( s ) \dot { q } ( s ) + \left( 1 - K ( s ) \right) \hat { q } ( s ) \triangleq K ( s ) \dot { q } ( s ) + K ^ { \prime } ( s ) \hat { q } ( s )\tag{35}
$$

where $K ( s )$ is selected as the low-pass filter design from Equation 33 and $\hat { \dot { q } } ( s )$ follows from Equation 29. The same form is applied to the horizontal tail position feedback signal, which leads to a unity synchronization filter. Figure 7 shows the results from a batch ??-analysis where the break frequency $\omega _ { n }$ has been selected as the running variable. This diagram confirms that as the break frequency reduces, which corresponds to closer operating time-scales between the plant and filter feedback augmentation system, the presented hybrid approach results in enhanced levels of nominal and robust performance for similar stability margins. However, the general conclusion remains that for maximum robust performance, filters shall be tuned sufficiently fast to meet the time-scale separation assumption. Therefore, in this case, adding complementary model information does not yield significant optimal performance benefits other than an improved nominal response. Also, some performance degradation can be expected for the hybrid approach in case of imperfect air data measurements, which is likely in practice [9].

<!-- image-->
Fig. 7 Peak $\bar { \mu }$ trends as function of low-pass break frequency for sensor-based (????) and hybrid (????) IDI designs

<!-- image-->
Fig. 8 Optimal $\bar { \mu }$ levels for basic and low-order synthesized IDI using actuator sensor (????), actuator model (????), or control command (????) feedback

Table 1 Summary of control system peak robust performance levels and nominal broken-loop response characteristics [actuator feedback closed (B); ???? - gain crossover frequency; ???? - phase margin]
<table><tr><td>Inversion design method</td><td>Sync. compensation</td><td> $\underline { { \bar { \mu } _ { R P } ^ { * } } }$  [-]</td><td> $\omega _ { c }$  [rad/s]</td><td>PM [deg]</td></tr><tr><td>Dynamic Inversion (DI)</td><td>n/a</td><td>2.10</td><td>5.98</td><td>83.5</td></tr><tr><td rowspan="2">Incremental Dynamic Inversion (IDI)</td><td>Actuator sensor</td><td>1.59</td><td>60.3</td><td>84.0</td></tr><tr><td>Actuator model</td><td>1.71</td><td>60.3</td><td>84.0</td></tr><tr><td>IDI augmentation - full-order synthesized designs</td><td></td><td></td><td></td><td></td></tr><tr><td>Asynchronous</td><td>Actuator sensor</td><td>0.858</td><td>12.5</td><td>53.6</td></tr><tr><td>Synchronous</td><td>Actuator sensor</td><td>0.869</td><td>15.3</td><td>48.7</td></tr><tr><td>IDI augmentation - low-order synthesized designs</td><td></td><td></td><td></td><td></td></tr><tr><td rowspan="3">Lag-lead, synchronous</td><td>Actuator sensor</td><td>0.879</td><td>17.4</td><td>59.6</td></tr><tr><td>Actuator model</td><td>0.793</td><td>14.7</td><td>79.2</td></tr><tr><td>Control command</td><td>0.789</td><td>14.6</td><td>81.2</td></tr><tr><td rowspan="3">Low-pass, synchronous</td><td>Actuator sensor</td><td>0.958</td><td>13.0</td><td>49.3</td></tr><tr><td>Actuator model</td><td>0.977</td><td>10.9</td><td>50.9</td></tr><tr><td>Control command</td><td>0.981</td><td>10.2</td><td>49.5</td></tr></table>

Secondly, the alternative synchronization strategies from Figure 6 are analyzed. Figure 8 illustrates the relative peak performance levels, where both basic (no filter augmentation) and optimized low-order designs are considered. Comparing these to the results presented earlier, it can be seen that both strategies deliver satisfactory robust performance levels if adequate filtering is in place. Therefore, viable designs may be found even in case no actuator sensor instrumentation is available. In case of sufficient bandwidth separation between the nominal actuator dynamics and augmentation filters, even the use of an internal model may not be required.

## D. Overview

An overview of the numerical results presented in the preceding Sections is shown in Table 1. The matching strategy has been found to be an adequate strategy for handling the synchronization effect in the presence of dynamic uncertainty. Robustness against aerodynamic variations is largely maintained and is not heavily affected by adopting low-order designs. This also holds in case internal actuator models are used, if adequate feedback filters are in place.

## V. Conclusion

The stability and performance robustness properties of incremental nonlinear dynamic inversion (INDI) when subjected to both regular and singular perturbations have been investigated in this article. For traditional Nonlinear Dynamic Inversion (NDI), the inversion residual that arises under these perturbations may grow large in magnitude, but can always be described by an upper bound under modest assumptions. By contrast, INDI shows improved robustness to regular perturbations, but its inversion residual may grow unbounded for particular combinations of singular perturbations. This corresponds to a loss of robust stability as a result of the synchronization effect. By introducing additional augmentation to sensor feedback signals, the overall robustness properties of INDI can be further improved.

A linear control law design study based on the structured singular value (??) framework confirms that IDI features increased levels of robustness to aerodynamic uncertainties, at the expense of robust stability due to singular perturbations. The matching strategy, which has been applied successfully in the past, is shown to be a reasonable design method to compensate for the synchronization effect and compares closely in terms of optimized robustness levels to less-constrained asynchronous design options. The use of additional model information as a form of complementary augmentation can further improve the design in case the time-scale separation assumption is violated due to low-bandwidth feedback filter designs. In case actuator sensor measurements are not available, alternative synchronization strategies can lead to adequately performing designs.

## References

[1] Anonymous, “Flight Control Design - Best Practices,” Tech. Rep. RTO-TR-029, NATO Research and Technology Organization (RTO), 2000.

[2] Anonymous, Flying Qualities of Piloted Aircraft, U.S. Department of Defense, 1997. MIL-HDBK-1797A.

[3] Anonymous, Airworthiness Certification Criteria, U.S. Department of Defense, 2014. MIL-HDBK-516C.

[4] Tauke, G., and Bordignon, K., “Structural Coupling Challenges for the X-35B,” 2002 Biennial International Powered Lift Conference and Exhibit, Williamsburg, VA, USA, 2002. https://doi.org/10.2514/6.2002-6004, AIAA-2002-6004.

[5] Moorhouse, D. J., “Lessons learned in the development of a multivariable control system (STOL and S/MTD programs),” Proceedings of the IEEE National Aerospace and Electronics Conference, Dayton, OH, USA, 1989, pp. 364–371 vol.1. https://doi.org/10.1109/NAECON.1989.40236.

[6] Blight, J. D., Dailey, R. L., and Gangsaas, D., “Practical control law design for aircraft using multivariable techniques,” International Journal of Control, Vol. 59, No. 1, 1994, pp. 93–137. https://doi.org/10.1080/00207179408923071.

[7] Honeywell Technology Center, Lockheed Martin Skunk Works and Lockheed Martin Tactical Aircraft Systems, “Application of Multivariable Control Theory to Aircraft Control Laws, Final Report : Multivariable Control Design Guidelines,” Tech. rep., 1996. WL-TR-96-3099.

[8] Balas, G. J., “Flight Control Law Design: An Industry Perspective,” European Journal of Control, Vol. 9, No. 2, 2003, pp. 207 – 226. https://doi.org/https://doi.org/10.3166/ejc.9.207-226.

[9] Enns, D., Bugajski, D., Hendrick, R., and Stein, G., “Dynamic inversion: an evolving methodology for flight control design,” International Journal of Control, Vol. 59, No. 1, 1994, pp. 71–91. https://doi.org/10.1080/00207179408923070.

[10] Harris, J. J., and Stanford, J. R., “F-35 Flight Control Law Design, Development and Verification,” 2018 Aviation Technology, Integration, and Operations Conference, Atlanta, GA, USA, 2018. https://doi.org/10.2514/6.2018-3516, AIAA-2018-3516.

[11] Canin, D., McConnell, J. K., and James, P. W., “F-35 High Angle of Attack Flight Control Development and Flight Test Results,” AIAA Aviation 2019 Forum, Dallas, TX, USA, 2019. https://doi.org/10.2514/6.2019-3227, AIAA-2019-3227.

[12] Smith, P., “A simplified approach to nonlinear dynamic inversion based flight control,” 23rd Atmospheric Flight Mechanics Conference, Boston, MA, USA, 1998. https://doi.org/10.2514/6.1998-4461, AIAA-1998-4461.

[13] Smith, P., and Berry, A., “Flight test experience of a non-linear dynamic inversion control law on the VAAC Harrier,” Atmospheric Flight Mechanics Conference, Denver, CO, USA, 2000. https://doi.org/10.2514/6.2000-3914, AIAA-2000-3914.

[14] Sieberling, S., Chu, Q. P., and Mulder, J. A., “Robust Flight Control Using Incremental Nonlinear Dynamic Inversion and Angular Acceleration Prediction,” Journal of Guidance, Control, and Dynamics, Vol. 33, No. 6, 2010, pp. 1732–1742. https://doi.org/10.2514/1.49978.

[15] Smeur, E. J. J., Chu, Q. P., and de Croon, G. C. H. E., “Adaptive Incremental Nonlinear Dynamic Inversion for Attitude Control of Micro Air Vehicles,” Journal of Guidance, Control, and Dynamics, Vol. 39, No. 3, 2016, pp. 450–461. https://doi.org/10.2514/1.G001490.

[16] Grondman, F., Looye, G., Kuchar, R. O., Chu, Q. P., and Van Kampen, E., “Design and Flight Testing of Incremental Nonlinear Dynamic Inversion-based Control Laws for a Passenger Aircraft,” 2018 AIAA Guidance, Navigation, and Control Conference, Kissimmee, FL, USA, 2018. https://doi.org/10.2514/6.2018-0385, AIAA-2018-0385.

[17] Keijzer, T., Looye, G., Chu, Q. P., and Van Kampen, E., “Design and Flight Testing of Incremental Backstepping based Control Laws with Angular Accelerometer Feedback,” AIAA Scitech 2019 Forum, San Diego, CA, USA, 2019. https: //doi.org/10.2514/6.2019-0129, AIAA-2019-0129.

[18] Pollack, T., Looye, G., and van der Linden, F., “Design and flight testing of flight control laws integrating incremental nonlinear dynamic inversion and servo current control,” AIAA Scitech 2019 Forum, San Diego, CA, USA, 2019. https: //doi.org/10.2514/6.2019-0130, AIAA-2019-0130.

[19] Smeur, E. J. J., Bronz, M., and de Croon, G. C. H. E., “Incremental Control and Guidance of Hybrid Aircraft Applied to a Tailsitter Unmanned Air Vehicle,” Journal of Guidance, Control, and Dynamics, Vol. 43, No. 2, 2020, pp. 274–287. https://doi.org/10.2514/1.G004520.

[20] Wang, X., van Kampen, E., Chu, Q., and Lu, P., “Stability Analysis for Incremental Nonlinear Dynamic Inversion Control,” Journal of Guidance, Control, and Dynamics, Vol. 42, No. 5, 2019, pp. 1116–1129. https://doi.org/10.2514/1.G003791.

[21] van ’t Veld, R., Kampen, E. V., and Chu, Q. P., “Stability and Robustness Analysis and Improvements for Incremental Nonlinear Dynamic Inversion Control,” 2018 AIAA Guidance, Navigation, and Control Conference, Kissimmee, FL, USA, 2018. https://doi.org/10.2514/6.2018-1127, AIAA-2018-1127.

[22] Pavel, M. D., Shanthakumaran, P., Chu, Q., Stroosma, O., Wolfe, M., and Cazemier, H., “Incremental Nonlinear Dynamic Inversion for the Apache AH-64 Helicopter Control,” Journal of the American Helicopter Society, Vol. 65, No. 2, 2020, pp. 1–16. https://doi.org/10.4050/JAHS.65.022006.

[23] Khalil, H., Nonlinear Systems, 3rd ed., Pearson Education, Prentice Hall, 2002.

[24] Bacon, B., and Ostroff, A., “Reconfigurable flight control using nonlinear dynamic inversion with a special accelerometer implementation,” AIAA Guidance, Navigation, and Control Conference and Exhibit, Denver, CO, USA, 2000. https: //doi.org/10.2514/6.2000-4565, AIAA-2000-4565.

[25] Wang, X., Kampen, E. v., Chu, Q., and Lu, P., “Incremental Sliding-Mode Fault-Tolerant Flight Control,” Journal of Guidance, Control, and Dynamics, Vol. 42, No. 2, 2019, pp. 244–259. https://doi.org/10.2514/1.G003497.

[26] Isidori, A., Nonlinear control systems, 3rd ed., Springer Science & Business Media, 1995.

[27] Stevens, B., Lewis, F., and Johnson, E., Aircraft Control and Simulation: Dynamics, Controls Design, and Autonomous Systems, Wiley, 2015.

[28] Russel, R., “Non-linear F-16 Simulation using Simulink and Matlab,” Tech. rep., University of Minnesota, 2003.

[29] Skogestad, S., and Postlethwaite, I., Multivariable Feedback Control: analysis and design, 2nd ed., John Wiley & Sons Ltd., 2005.

[30] Balas, G., Chiang, R., Packard, A., and Safonov, M., Robust Control Toolbox™ User’s Guide, The MathWorks, Inc., 2021.

[31] Kumtepe, Y., Pollack, T., and van Kampen, E., “Flight Control Law Design Using Hybrid Incremental Nonlinear Dynamic Inversion,” Accepted for presentation at the AIAA SciTech 2022 Forum, 2022.