This paper has been accepted for publication in IEEE Transactions on Robotics.

DOI: 10.1109/TRO.2016.2597321 IEEE Explore: http://ieeexplore.ieee.org/document/7557075/

Please cite the paper as:

Christian Forster, Luca Carlone, Frank Dellaert, Davide Scaramuzza, “On-Manifold Preintegration for Real-Time Visual-Inertial Odometry”, in IEEE Transactions on Robotics, 2016.

# On-Manifold Preintegration for Real-Time Visual-Inertial Odometry

Christian Forster, Luca Carlone, Frank Dellaert, Davide Scaramuzza

Abstract—Current approaches for visual-inertial odometry (VIO) are able to attain highly accurate state estimation via nonlinear optimization. However, real-time optimization quickly becomes infeasible as the trajectory grows over time; this problem is further emphasized by the fact that inertial measurements come at high rate, hence leading to fast growth of the number of variables in the optimization. In this paper, we address this issue by preintegrating inertial measurements between selected keyframes into single relative motion constraints. Our first contribution is a preintegration theory that properly addresses the manifold structure of the rotation group. We formally discuss the generative measurement model as well as the nature of the rotation noise and derive the expression for the maximum a posteriori state estimator. Our theoretical development enables the computation of all necessary Jacobians for the optimization and a-posteriori bias correction in analytic form. The second contribution is to show that the preintegrated IMU model can be seamlessly integrated into a visual-inertial pipeline under the unifying framework of factor graphs. This enables the application of incremental-smoothing algorithms and the use of a structureless model for visual measurements, which avoids optimizing over the 3D points, further accelerating the computation. We perform an extensive evaluation of our monocular VIO pipeline on real and simulated datasets. The results confirm that our modelling effort leads to accurate state estimation in real-time, outperforming state-of-the-art approaches.

## SUPPLEMENTARY MATERIAL

• Video of the experiments: https://youtu.be/CsJkci5lfco

• Source-code for preintegrated IMU and structureless vision factors https://bitbucket.org/gtborg/gtsam.

## I. INTRODUCTION

The use of cameras and inertial sensors for threedimensional structure and motion estimation has received considerable attention from the robotics community. Both sensor types are cheap, ubiquitous, and complementary. A single moving camera is an exteroceptive sensor that allows us to measure appearance and geometry of a three-dimensional scene, up to an unknown metric scale; an inertial measurement unit (IMU) is a proprioceptive sensor that renders metric scale of monocular vision and gravity observable [1] and provides robust and accurate inter-frame motion estimates. Applications of VIO range from autonomous navigation in GPS-denied environments, to 3D reconstruction, and augmented reality.

The existing literature on VIO imposes a trade-off between accuracy and computational efficiency (a detailed review is given in Section II). On the one hand, filtering approaches enable fast inference, but their accuracy is deteriorated by the accumulation of linearization errors. On the other hand, full smoothing approaches, based on nonlinear optimization, are accurate, but computationally demanding. Fixed-lag smoothing offers a compromise between accuracy for efficiency; however, it is not clear how to set the length of the estimation window so to guarantee a given level of performance.

In this work we show that it is possible to overcome this trade-off. We design a VIO system that enables fast incremental smoothing and computes the optimal maximum a posteriori (MAP) estimate in real time. An overview of our approach is given in Section IV.

The first step towards this goal is the development of a novel preintegration theory. The use of preintegrated IMU measurements was first proposed in [2] and consists of combining many inertial measurements between two keyframes into a single relative motion constraint. We build upon this work and present a preintegration theory that properly addresses the manifold structure of the rotation group SO(3). Our preintegration theory is presented in Sections V-VI. Compared with [2], our theory offers a more formal treatment of the rotation noise, and avoids singularities in the representation of rotations. Furthermore, we are able to derive all necessary Jacobians in analytic form: specifically, we report the analytic Jacobians of the residuals, the noise propagation, and the aposteriori bias correction in the appendix of this paper.

Our second contribution is to frame the IMU preintegration theory into a factor graph model. This enables the application of incremental smoothing algorithms, as iSAM2 [3], which avoid the accumulation of linearization errors and offer an elegant way to trade-off accuracy with efficiency. Inspired by [4, 5], we also adopt a structureless model for visual measurements, which allows eliminating a large number of variables (i.e., all 3D points) during incremental smoothing, further accelerating the computation (Section VII). In contrast to [5], we use the structureless model in an incremental smoothing framework. This has two main advantages: we do not need to delay the processing of visual measurements, and we can relinearize the visual measurements multiple times.

In order to demonstrate the effectiveness of our model, we integrated the proposed IMU preintegration in a state-of-theart VIO pipeline and tested it on real and simulated datasets (Sections VIII). Our theoretical development leads to tangible practical advantages: an implementation of the approach proposed in this paper performs full-smoothing at a rate of 100 Hz and achieves superior accuracy with respect to competitive state-of-the-art filtering and optimization approaches.

Besides the technical contribution, the paper also provides a tutorial contribution for practitioners. In Section III and across the paper, we provide a short but concise summary of uncertainty representation on manifolds and exemplary derivations for uncertainty propagation and Jacobian computation. The complete derivation of all equations and Jacobians – necessary to implement our model – are given in the appendix.

This paper is an extension of our previous work [6] with additional experiments, an in-depth discussion of related work, and comprehensive technical derivations. The results of the new experiments highlight the accuracy of bias estimation, demonstrate the consistency of our approach, and provide comparisons against full batch estimation. We release our implementation of the preintegrated IMU and structureless vision factors in the GTSAM 4.0 optimization toolbox [7].

## II. RELATED WORK

Related work on visual-inertial odometry can be sectioned along three main dimensions. The first dimension is the number of camera-poses involved in the estimation. While full smoothers (or batch nonlinear least-squares algorithms) estimate the complete history of poses, fixed-lag smoothers (or sliding window estimators) consider a window of the latest poses, and filtering approaches only estimate the latest state. Both fixed-lag smoothers and filters marginalize older states and absorb the corresponding information in a Gaussian prior.

The second dimension regards the representation of the uncertainty for the measurements and the Gaussian priors: the Extended Kalman Filter (EKF) represents the uncertainty using a covariance matrix; instead, information filters and smoothers resort to the information matrix (the inverse of the covariance) or the square-root of the information matrix [3, 8].

Finally, the third dimension distinguishes existing approaches by looking at the number of times in which the measurement model is linearized. While a standard EKF (in contrast to the iterated EKF) processes a measurement only once, a smoothing approach allows linearizing multiple times.

While the terminology is vast, the underlying algorithms are tightly related. For instance, it can be shown that the iterated Extended Kalman filter equations are equivalent to the Gauss-Newton algorithm, commonly used for smoothing [9].

## A. Filtering

Filtering algorithms enable efficient estimation by restricting the inference process to the latest state of the system. The complexity of the EKF grows quadratically in the number of estimated landmarks, therefore, a small number of landmarks (in the order of 20) are typically tracked to allow real-time operation [10–12]. An alternative is to adopt a “structureless” approach where landmark positions are marginalized out of the state vector. An elegant example of this strategy is the

Multi-State Constraint Kalman filter (MSC-KF) [5]. The structureless approach requires to keep previous poses in the state vector, by means of stochastic cloning [13].

A drawback of using a structureless approach for filtering, is that the processing of landmark measurements needs to be delayed until all measurements of a landmark are obtained [5]. This hinders accuracy as the filter cannot use all current visual information. Marginalization is also a source of errors as it locks in linearization errors and erroneous outlier measurements. Therefore, it is particularly important to filter out spurious measurements as a single outlier can irreversibly corrupt the filter [14]. Further, linearization errors introduce drift in the estimate and render the filter inconsistent. An effect of inconsistency is that the estimator becomes over-confident, resulting in non-optimal information fusion. Generally, the VIO problem has four unobservable directions: the global position and the orientation around the gravity direction (yaw) [15, 16]. In [16] it is shown that linearization at the wrong estimate results in only three unobservable directions (the global position); hence, erroneous linearization adds spurious information in yaw direction to the Gaussian prior, which renders the filter inconsistent. This problem was addressed with the first-estimates jacobian approach [17], which ensures that a state is not updated with different linearization points — a source of inconsistency. In the observability-constrained EKF (OC-EKF) an estimate of the unobservable directions is maintained which allows to update the filter only in directions that are observable [16, 18]. A thorough analysis of VIO observability properties is given in [1, 15, 19].

## B. Fixed-lag Smoothing

Fixed-lag smoothers estimate the states that fall within a given time window, while marginalizing out older states [20– 24]. In a maximum likelihood estimation setup, fixed-lag smoothers lead to an optimization problem over a set of recent states. For nonlinear problems, smoothing approaches are generally more accurate than filtering, since they relinearize past measurements [25]. Moreover, these approaches are more resilient to outliers, which can be discarded a posteriori (i.e., after the optimization), or can be alleviated by using robust cost functions. On the downside, the marginalization of the states outside the estimation window leads to dense Gaussian priors which hinder efficient inference. For this reason, it has been proposed to drop certain measurements in the interest of sparsity [24]. Furthermore, due to marginalization, fixed-lag smoothers share part of the issues of filtering (consistency, build-up of linearization errors) [18, 22, 26].

## C. Full Smoothing

Full smoothing methods estimate the entire history of the states (camera trajectory and 3D landmarks), by solving a large nonlinear optimization problem [27–31]. Full smoothing guarantees the highest accuracy; however, real-time operation quickly becomes infeasible as the trajectory and the map grow over time. Therefore, it has been proposed to discard frames except selected keyframes [24, 32–34] or to run the optimization in a parallel thread, using a tracking and mapping dual architecture [20, 35]. A breakthrough has been the development of incremental smoothing techniques (iSAM [36], iSAM2 [3]), which leverage the expressiveness offactor graphs to maintain sparsity and to identify and update only the typically small subset of variables affected by a new measurement.

Nevertheless, the high rate of inertial measurements (usually 100 Hz to 1 kHz) still constitutes a challenge for smoothing approaches. A naive implementation would require adding a new state at every IMU measurement, which quickly becomes impractically slow [37]. Therefore, inertial measurements are typically integrated between frames to form relative motion constraints [24, 30, 38–40]. For standard IMU integration between two frames, the initial condition is given by the state estimate at the first frame. However, at every iteration of the optimization, the state estimate changes, which requires to repeat the IMU integration between all frames [24]. Lupton and Sukkarieh [2] show that this repeated integration can be avoided by a reparametrization of the relative motion constraints. Such reparametrization is called IMU preintegration.

In the present work, we build upon the seminal work [2] and bring the theory of IMU preintegration to maturity by properly addressing the manifold structure of the rotation group SO(3). The work [2] adopted Euler angles as global parametrization for rotations. Using Euler angles and applying the usual averaging and smoothing techniques of Euclidean spaces for state propagation and covariance estimation is not properly invariant under the action of rigid transformations [41, 42]. Moreover, Euler angles are known to have singularities. Our work, on the other hand, provides a formal treatment of the rotation measurements (and the corresponding noise), and provides a complete derivation of the maximum a posteriori estimator. We also derive analytic expressions for the Jacobians (needed for the optimization), which, to the best of our knowledge, have not been previously reported in the literature. In the experimental section, we show that a proper representation of the rotation manifold results in higher accuracy and robustness, leading to tangible advantages over the original proposal [2].

## III. PRELIMINARIES

In this paper we formulate VIO in terms of MAP estimation. In our model, MAP estimation leads to a nonlinear optimization problem that involves quantities living on smooth manifolds (e.g., rotations, poses). Therefore, before delving into details, we conveniently review some useful geometric concepts. This section can be skipped by the expert reader.

We structure this section as follows: Section III-A provides useful notions related to two main Riemannian manifolds: the Special Orthogonal Group SO(3) and the Special Euclidean Group SE(3). Our presentation is based on [43, 44]. Section III-B describes a suitable model to describe uncertain rotations in SO(3). Section III-C reviews optimization on manifolds, following standard references [45].

## A. Notions of Riemannian geometry

a) Special Orthogonal Group: SO(3) describes the group of 3D rotation matrices and it is formally defined as $\mathrm { S O ( 3 ) } \doteq \{ \mathtt { R } \in \mathbb { R } ^ { 3 \times 3 } : \mathtt { R } ^ { \mathsf { T } } \mathtt { R } = \mathbf { I } , \mathtt { d e t } ( \mathtt { R } ) = 1 \}$ . The group operation is the usual matrix multiplication, and the inverse is the matrix transpose. The group $\mathrm { S O ( 3 ) }$ also forms a smooth manifold. The tangent space to the manifold (at the identity) is denoted as so(3), which is also called the Lie algebra and coincides with the space of $3 \times 3$ skew symmetric matrices. We can identify every skew symmetric matrix with a vector in $\mathbb { R } ^ { 3 }$ using the hat operator:

![](images/3c8661e0e0a41aab890aa206366820fcd571bae5c02472c6eabc3645e7895861.jpg)  
Fig. 1: The right Jacobian J<sub>r</sub> relates an additive perturbation δφ in the tangent space to a multiplicative perturbation on the manifold SO(3), as per Eq. (7).

$$
\boldsymbol {\omega} ^ {\wedge} = \left[ \begin{array}{c} \omega_ {1} \\ \omega_ {2} \\ \omega_ {3} \end{array} \right] ^ {\wedge} = \left[ \begin{array}{c c c} 0 & - \omega_ {3} & \omega_ {2} \\ \omega_ {3} & 0 & - \omega_ {1} \\ - \omega_ {2} & \omega_ {1} & 0 \end{array} \right] \in \mathfrak {s o} (3).\tag{1}
$$

Similarly, we can map a skew symmetric matrix to a vector in $\mathbb { R } ^ { 3 }$ using the vee operator $( \cdot ) ^ { \vee } \colon$ for a skew symmetric matrix $\mathsf { S } = \omega ^ { \wedge }$ , the vee operator is such that $\mathsf { S } ^ { \vee } = \omega . \mathrm { ~ A ~ }$ property of skew symmetric matrices that will be useful later on is:

$$
\mathbf {a} ^ {\wedge} \mathbf {b} = - \mathbf {b} ^ {\wedge} \mathbf {a}, \quad \forall \mathbf {a}, \mathbf {b} \in \mathbb {R} ^ {3}.\tag{2}
$$

The exponential map (at the identity) exp : ${ \mathfrak { s o } } ( 3 ) \to \mathrm { S O } ( 3 )$ associates an element of the Lie Algebra to a rotation and coincides with standard matrix exponential (Rodrigues’ formula):

$$
\exp (\phi^ {\wedge}) = \mathbf {I} + \frac {\sin (\| \phi \|)}{\| \phi \|} \phi^ {\wedge} + \frac {1 - \cos (\| \phi \|)}{\| \phi \| ^ {2}} \left(\phi^ {\wedge}\right) ^ {2}.\tag{3}
$$

A first-order approximation of the exponential map that we will use later on is:

$$
\exp (\phi^ {\wedge}) \approx \mathbf {I} + \phi^ {\wedge}.\tag{4}
$$

The logarithm map (at the identity) associates a matrix $\mathtt { R \neq }$ I in $\mathrm { S O ( 3 ) }$ to a skew symmetric matrix:

$$
\log (\mathrm{R}) = \frac {\varphi \cdot (\mathrm{R} - \mathrm{R} ^ {\mathrm{T}})}{2 \sin (\varphi)} \text {   with   } \varphi = \cos^ {- 1} \left(\frac {\operatorname{tr} (\mathrm{R}) - 1}{2}\right).\tag{5}
$$

Note that $\log ( \mathtt { R } ) ^ { \vee } = \mathbf { a } \varphi$ , where a and $\varphi$ are the rotation axis and the rotation angle of $\mathbb { R } ,$ respectively. If $\mathbb { R } = \mathbf { I } .$ , then $\varphi = 0$ and a is undetermined and can therefore be chosen arbitrarily.

The exponential map is a bijection if restricted to the open ball $\| \phi \| < \pi$ , and the corresponding inverse is the logarithm map. However, if we do not restrict the domain, the exponential map becomes surjective as every vector $\phi = ( \varphi + 2 k \pi ) \mathbf { a }$ $k \in \mathbb { Z }$ would be an admissible logarithm of R.

For notational convenience, we adopt “vectorized” versions of the exponential and logarithm map:

$$
\begin{array}{c c c c c c c c} \text {Exp:} & \mathbb {R} ^ {3} & \to & \text {SO(3)} & ; & \phi & \mapsto & \exp (\phi^ {\wedge}) \\ \text {Log:} & \text {SO(3)} & \to & \mathbb {R} ^ {3} & ; & \mathsf {R} & \mapsto & \log (\mathsf {R}) ^ {\vee}, \end{array}\tag{6}
$$

which operate directly on vectors, rather than on skew symmetric matrices in $\mathfrak { s o } ( 3 )$

Later, we will use the following first-order approximation:

$$
\operatorname{Exp} (\phi + \delta \phi) \approx \operatorname{Exp} (\phi) \operatorname{Exp} (\mathrm{J} _ {r} (\phi) \delta \phi).\tag{7}
$$

The term $\textstyle { \mathrm { J } } _ { r } ( \phi )$ is the right Jacobian of SO(3) [43, p.40] and relates additive increments in the tangent space to multiplicative increments applied on the right-hand-side (Fig. 1):

$$
\mathbf {J} _ {r} (\phi) = \mathbf {I} - \frac {1 - \cos (\| \phi \|)}{\| \phi \| ^ {2}} \phi^ {\wedge} + \frac {\| \phi \| - \sin (\| \phi \|)}{\| \phi^ {3} \|} (\phi^ {\wedge}) ^ {2}.\tag{8}
$$

A similar first-order approximation holds for the logarithm:

$$
\operatorname{Log} \big (\operatorname{Exp} (\phi) \operatorname{Exp} (\delta \phi) \big) \approx \phi + \mathrm{J} _ {r} ^ {- 1} (\phi) \delta \phi .\tag{9}
$$

Where the inverse of the right Jacobian is

$$
\mathrm{J} _ {r} ^ {- 1} (\phi) = \mathbf {I} + \frac {1}{2} \phi^ {\wedge} + \left(\frac {1}{\| \phi \| ^ {2}} + \frac {1 + \cos (\| \phi \|)}{2 \| \phi \| \sin (\| \phi \|)}\right) (\phi^ {\wedge}) ^ {2}.
$$

The right Jacobian $\textstyle { \mathrm { J } } _ { r } ( \phi )$ and its inverse $J _ { r } ^ { - 1 } ( \phi )$ reduce to the identity matrix for $\| \phi \| = 0$

Another useful property of the exponential map is:

$$
\mathrm{R} \operatorname{Exp} (\phi) \mathrm{R} ^ {\mathsf {T}} = \exp (\mathrm{R} \phi^ {\wedge} \mathrm{R} ^ {\mathsf {T}}) = \operatorname{Exp} (\mathrm{R} \phi)\tag{10}
$$

$$
\Leftrightarrow \quad \operatorname{Exp} (\phi) \mathsf {R} = \mathsf {R} \operatorname{Exp} (\mathsf {R} ^ {\mathsf {T}} \phi).\tag{11}
$$

b) Special Euclidean Group: SE(3) describes the group of rigid motion in 3D, which is the semi-direct product of $\mathrm { S O ( 3 ) }$ and $\mathbb { R } ^ { 3 }$ , and it is defined as ${ \mathrm { S E } } ( 3 ) \doteq \{ ( { \mathbb { R } } , { \mathbf { p } } ) : { \mathbb { R } } \in$ $\mathrm { S O } ( 3 ) , \mathbf { p } \in \mathbb { R } ^ { 3 } \}$ . Given $\mathrm { T } _ { 1 } , \mathrm { T } _ { 2 } \in \mathrm { S E } ( 3 )$ , the group operation is ${ \bf T } _ { 1 } \cdot { \bf T } _ { 2 } = ( \mathtt { R } _ { 1 } \mathtt { R } _ { 2 } \ , \ { \bf p } _ { 1 } + \mathtt { R } _ { 1 } { \bf p } _ { 2 } )$ , and the inverse is $\boldsymbol { \mathrm { T } } _ { 1 } ^ { - 1 } =$ $\left( \mathsf { R } _ { 1 } ^ { \mathsf { T } } \ , \ - \mathsf { R } _ { 1 } ^ { \mathsf { T } } \mathbf { p } _ { 1 } \right)$ . The exponential map and the logarithm map for SE(3) are defined in [44]. However, these are not needed in this paper for reasons that will be clear in Section III-C.

## B. Uncertainty Description in SO(3)

A natural definition of uncertainty in SO(3) is to define a distribution in the tangent space, and then map it to $\mathrm { S O ( 3 ) }$ via the exponential map (6) [44, 46, 47]:

$$
\tilde {\mathsf {R}} = \mathsf {R}   \mathrm{Exp} (\epsilon), \qquad \epsilon \sim \mathcal {N} (0, \Sigma),\tag{12}
$$

where R is a given noise-free rotation (the mean) and  is a small normally distributed perturbation with zero mean and covariance Σ.

To obtain an explicit expression for the distribution of ${ \tilde { \mathsf { R } } } ,$ we start from the integral of the Gaussian distribution in $\mathbb { R } ^ { 3 }$ :

$$
\int_ {\mathbb {R} ^ {3}} p (\epsilon) d \epsilon = \int_ {\mathbb {R} ^ {3}} \alpha e ^ {- \frac {1}{2} \| \epsilon \| _ {\Sigma} ^ {2}} \mathrm{d} \epsilon = 1,\tag{13}
$$

where $\alpha = 1 / \sqrt { ( 2 \pi ) ^ { 3 } \operatorname* { d e t } ( \Sigma ) }$ and $\| \epsilon \| _ { \Sigma } ^ { 2 } \doteq \epsilon ^ { \mathsf { T } } \Sigma ^ { - 1 } \epsilon$  is the squared Mahalanobis distance with covariance Σ. Then, applying the change of coordinates $\epsilon = \mathrm { L o g } ( \mathtt { R } ^ { - 1 } \tilde { \mathtt { R } } )$ (this is the inverse of (12) when $\| \epsilon \| < \pi )$ , the integral (13) becomes:

$$
\int_ {\mathrm{SO} (3)} \beta (\tilde {\mathsf {R}}) e ^ {- \frac {1}{2} \left\| \mathrm{Log} (\mathbb {R} ^ {- 1} \tilde {\mathsf {R}}) \right\| _ {\Sigma} ^ {2}} d \tilde {\mathsf {R}} = 1,\tag{14}
$$

where $\beta ( { \tilde { \tt R } } )$ is a normalization factor. The normalization factor assumes the form $\beta ( \tilde { \mathbb { R } } ) ~ = ~ \alpha / | \operatorname* { d e t } ( \mathcal { I } ( \tilde { \mathbb { R } } ) |$ |, where $\mathcal { I } ( \tilde { \mathtt { R } } ) \ \doteq$ $\mathbb { J } _ { r } ( \mathrm { L o g } ( \mathbb { R } ^ { - 1 } \tilde { \mathbb { R } } ) )$ and $\bar { \mathsf { J } } _ { r } ( \cdot )$ is the right Jacobian $( 8 ) ; { \mathcal { I } } ( { \tilde { \mathbb { R } } } )$ is a by-product of the change of variables, see [46] for a derivation.

From the argument of (14) we can directly read our “Gaussian” distribution in SO(3):

$$
p (\tilde {\mathbb {R}}) = \beta (\tilde {\mathbb {R}}) e ^ {- \frac {1}{2} \left\| \mathrm{Log} (\mathbb {R} ^ {- 1} \tilde {\mathbb {R}}) \right\| _ {\Sigma} ^ {2}}.\tag{15}
$$

For small covariances we can approximate $\beta \ \simeq \ \alpha ,$ as $\mathbb { J } _ { r } ( \mathrm { L o g } ( \mathbb { R } ^ { - 1 } { \tilde { \mathbb { R } } } ) )$ is well approximated by the identity matrix <sub>when R</sub>˜ <sub>is close to R. Note that (14) already assumes relatively</sub> a small covariance Σ, since it “clips” the probability tails outside the open ball of radius π (this is due to the reparametrization $\epsilon = \mathrm { L o g } ( \mathtt { R } ^ { - 1 } \tilde { \mathtt { R } } )$ , which restricts $\epsilon \mathrm { ~ t o ~ } \| \epsilon \| < \pi )$ Approximating $\beta$ as a constant, the negative log-likelihood of a rotation R, given a measurement $\tilde { \mathtt { R } }$ distributed as in (15), is:

$$
\mathcal {L} (\mathsf {R}) = \frac {1}{2} \left\| \operatorname{Log} \left(\mathsf {R} ^ {- 1} \tilde {\mathsf {R}}\right) \right\| _ {\Sigma} ^ {2} + \text {const} = \frac {1}{2} \left\| \operatorname{Log} \left(\tilde {\mathsf {R}} ^ {- 1} \mathsf {R}\right) \right\| _ {\Sigma} ^ {2} + \text {const},\tag{16}
$$

which geometrically can be interpreted as the squared angle (geodesic distance in $\mathrm { S O ( 3 ) } )$ <sub>between R</sub>˜ <sub>and R weighted by</sub> the inverse uncertainty $\Sigma ^ { - 1 }$

## C. Gauss-Newton Method on Manifold

A standard Gauss-Newton method in Euclidean space works by repeatedly optimizing a quadratic approximation of the (generally non-convex) objective function. Solving the quadratic approximation reduces to solving a set of linear equations (normal equations), and the solution of this local approximation is used to update the current estimate. Here we recall how to extend this approach to (unconstrained) optimization problems whose variables belong to some manifold M.

Let us consider the following optimization problem:

$$
\min _ {x \in \mathcal {M}} f (x),\tag{17}
$$

where the variable x belongs to a manifold M; for the sake of simplicity we consider a single variable in (17), while the description easily generalizes to multiple variables.

Contrarily to the Euclidean case, one cannot directly approximate (17) as a quadratic function of x. This is due to two main reasons. First, working directly on x leads to an over-parametrization of the problem (e.g., we parametrize a rotation matrix with 9 elements, while a 3D rotation is completely defined by a vector in $\mathbb { R } ^ { 3 } )$ and this can make the normal equations under-determined. Second, the solution of the resulting approximation does not belong to M in general.

A standard approach for optimization on manifold [45, 48], consists of defining a retraction $\mathcal { R } _ { x } .$ , which is a bijective map between an element δx of the tangent space (at x) and a neighborhood of $x \in \mathcal { M }$ . Using the retraction, we can reparametrize our problem as follows:

$$
\min _ {x \in \mathcal {M}} f (x) \quad \Rightarrow \quad \min _ {\delta x \in \mathbb {R} ^ {n}} f (\mathcal {R} _ {x} (\delta x)).\tag{18}
$$

The re-parametrization is usually called lifting [45]. Roughly speaking, we work in the tangent space defined at the current estimate, which locally behaves as an Euclidean space. The use of the retraction allows framing the optimization problem over an Euclidean space of suitable dimension $( \mathbf { e . g . } , \delta x \in \mathbb { R } ^ { 3 }$ when we work in $\mathrm { S O ( 3 ) } )$ ). We can now apply standard optimization techniques to the problem on the right-hand side of (18). In the Gauss-Newton framework, we square the cost around the current estimate. Then we solve the quadratic approximation to get a vector $\delta x ^ { \star }$ in the tangent space. Finally, the current guess on the manifold is updated as

![](images/001cb7dde8f0772a847df28503fdb3dffeb455443e470029de87cdacf0215a31.jpg)  
Fig. $2 \colon \mathsf { T } _ { \mathrm { W B } } \doteq ( \mathsf { R } _ { \mathrm { W B } } , \mathsf { w } \mathbf { p } )$ is the pose of the body frame B w.r.t. the world frame W. We assume that the body frame coincides with the IMU frame. T is the pose of the camera in the body frame, known from prior calibration.

$$
\hat {x} \leftarrow \mathcal {R} _ {\hat {x}} (\delta x ^ {\star}).\tag{19}
$$

This “lift-solve-retract” scheme can be generalized to any trust-region method [45]. Moreover, it provides a grounded and unifying generalization of the error state model, commonly used in aerospace literature for filtering [49] and recently adopted in robotics for optimization [23, 34].

We conclude this section by discussing the choice of the retraction $\mathcal { R } _ { x }$ . A possible retraction is the exponential map. It is known that, computationally, this may not be the most convenient choice, see [50].

In this work, we use the following retraction for SO(3),

$$
\mathcal {R} _ {\mathtt {R}} (\phi) = \mathtt {R}   \mathrm{Exp} (\delta \phi), \qquad \delta \phi \in \mathbb {R} ^ {3},\tag{20}
$$

and for SE(3), we use the retraction at ${ \boldsymbol { \mathbf { T } } } \doteq ( { \boldsymbol { \mathbf { R } } } , \mathbf { p } )$

$$
\mathcal {R} _ {\mathrm{T}} (\delta \phi , \delta \mathbf {p}) = (\mathrm{R} \operatorname{Exp} (\delta \phi), \mathbf {p} + \mathrm{R} \delta \mathbf {p}), \quad [ \delta \phi \delta \mathbf {p} ] \in \mathbb {R} ^ {6},\tag{21}
$$

which explains why in Section III-A we only defined the exponential map for SO(3): with this choice of retraction we never need to compute the exponential map for SE(3).

## IV. MAXIMUM A POSTERIORI VISUAL-INERTIAL STATE ESTIMATION

We consider a VIO problem in which we want to track the state of a sensing system (e.g., a mobile robot, a UAV, or a hand-held device), equipped with an IMU and a monocular camera. We assume that the IMU frame “B” coincides with the body frame we want to track, and that the transformation between the camera and the IMU is fixed and known from prior calibration (Fig. 2). Furthermore, we assume that a front-end provides image measurements of 3D landmarks at unknown position. The front-end also selects a subset of images, called keyframes [32], for which we want to compute a pose estimate. Section VIII-B1 discusses implementation aspects, including the choice of the front-end in our experiments.

## A. The State

The state of the system at time i is described by the IMU orientation, position, velocity and biases:

$$
\mathbf {x} _ {i} \doteq [ \mathrm{R} _ {i}, \mathbf {p} _ {i}, \mathbf {v} _ {i}, \mathbf {b} _ {i} ].\tag{22}
$$

The pose $\left( \mathtt { R } _ { i } , \mathbf { p } _ { i } \right)$ belongs to SE(3), while velocities live in a vector space, i.e., $\mathbf { v } _ { i } \in \mathbb { R } ^ { 3 }$ . IMU biases can be written as $\mathbf { b } _ { i } = [ \mathbf { b } _ { i } ^ { g } \mathbf { b } _ { i } ^ { a } ] \in \mathbb { R } ^ { 6 }$ , where $\mathbf { b } _ { i } ^ { g } , \mathbf { b } _ { i } ^ { a } \in \mathbb { R } ^ { 3 }$ are the gyroscope and accelerometer bias, respectively.

Let $\kappa _ { k }$ denote the set of all keyframes up to time $k .$ In our approach we estimate the state of all keyframes:

$$
\mathcal {X} _ {k} \doteq \{\mathbf {x} _ {i} \} _ {i \in \mathcal {K} _ {k}}.\tag{23}
$$

In our implementation, we adopt a structureless approach $( c f .$ Section VII), hence the 3D landmarks are not part of the variables to be estimated. However, the proposed approach generalizes in a straightforward manner to also estimating the landmarks and the camera intrinsic and extrinsic calibration parameters.

## B. The Measurements

The input to our estimation problem are the measurements from the camera and the IMU. We denote with $\mathcal { C } _ { i }$ the image measurements at keyframe i. At time $i ,$ the camera can observe multiple landmarks l, hence $\mathcal { C } _ { i }$ contains multiple image measurements $\mathbf { z } _ { i l }$ . With slight abuse of notation we write $l \in \mathcal { C } _ { i }$ when a landmark l is seen at time i.

We denote with $\mathcal { T } _ { i j }$ the set of IMU measurements acquired between two consecutive keyframes i and $j .$ Depending on the IMU measurement rate and the frequency of selected keyframes, each set $\mathcal { T } _ { i j }$ can contain from a small number to hundreds of IMU measurements. The set of measurements collected up to time k is

$$
\mathcal {Z} _ {k} \doteq \{\mathcal {C} _ {i}, \mathcal {I} _ {i j} \} _ {(i, j) \in \mathcal {K} _ {k}}.\tag{24}
$$

## C. Factor Graphs and MAP Estimation

The posterior probability of the variables $\mathcal { X } _ { k }$ , given the available visual and inertial measurements $\mathcal { Z } _ { k }$ and priors $p ( \mathcal { X } _ { 0 } )$ is:

$$
p (\mathcal {X} _ {k} | \mathcal {Z} _ {k}) \propto p (\mathcal {X} _ {0}) p (\mathcal {Z} _ {k} | \mathcal {X} _ {k}) \stackrel {(a)} {=} p (\mathcal {X} _ {0}) \prod_ {(i, j) \in \mathcal {K} _ {k}} p (\mathcal {C} _ {i}, \mathcal {I} _ {i j} | \mathcal {X} _ {k})
$$

$$
\stackrel {(b)} {=} p (\mathcal {X} _ {0}) \prod_ {(i, j) \in \mathcal {K} _ {k}} p (\mathcal {I} _ {i j} | \mathbf {x} _ {i}, \mathbf {x} _ {j}) \prod_ {i \in \mathcal {K} _ {k}} \prod_ {l \in \mathcal {C} _ {i}} p (\mathbf {z} _ {i l} | \mathbf {x} _ {i}).\tag{25}
$$

The factorizations (a) and (b) follow from a standard independence assumption among the measurements. Furthermore, the Markovian property is applied in (b) (e.g., an image measurement at time i only depends on the state at time i).

As the measurements $\mathcal { Z } _ { k }$ are known, we are free to eliminate them as variables and consider them as parameters of the joint probability factors over the actual unknowns. This naturally leads to the well known factor graph representation, a class of bipartite graphical models that can be used to represent such factored densities [51, 52]. A schematic representation of the connectivity of the factor graph underlying the VIO problem is given in Fig. 3 (the connectivity of the structureless vision factors will be clarified in Section VII). The factor graph is composed of nodes for unknowns and nodes for the probability factors defined on them, and the graph structure expresses which unknowns are involved in each factor.

![](images/dbe8291d83dba90af954d9425fdf26aa6632901cd38b902bf64aabf5e4385cf4.jpg)  
Fig. 3: Left: visual and inertial measurements in VIO. Right: factor graph in which several IMU measurements are summarized in a single preintegrated IMU factor and a structureless vision factor constraints keyframes observing the same landmark.

The MAP estimate $\mathcal { X } _ { k } ^ { \star }$ corresponds to the maximum of (25), or equivalently, the minimum of the negative log-posterior. Under the assumption of zero-mean Gaussian noise, the negative log-posterior can be written as a sum of squared residual errors:

$$
\begin{array}{l} \mathcal {X} _ {k} ^ {\star} \doteq \arg \min _ {\mathcal {X} _ {k}} - \log_ {e} p (\mathcal {X} _ {k} | \mathcal {Z} _ {k}) \\ = \arg \min _ {\mathcal {X} _ {k}} \| \mathbf {r} _ {0} \| _ {\boldsymbol {\Sigma} _ {0}} ^ {2} + \sum_ {(i, j) \in \mathcal {K} _ {k}} \| \mathbf {r} _ {\mathcal {I} _ {i j}} \| _ {\boldsymbol {\Sigma} _ {i j}} ^ {2} + \sum_ {i \in \mathcal {K} _ {k}} \sum_ {l \in \mathcal {C} _ {i}} \| \mathbf {r} _ {\mathcal {C} _ {i l}} \| _ {\boldsymbol {\Sigma} _ {\mathcal {C}}} ^ {2} \end{array} \tag {26}
$$

where r $) , \mathbf { r } _ { \mathcal { T } _ { i j } } , \mathbf { r } _ { \mathcal { C } _ { i l } }$ are the residual errors associated to the measurements, and $\Sigma _ { 0 } , \Sigma _ { i j }$ , and $\Sigma _ { \mathcal { C } }$ are the corresponding covariance matrices. Roughly speaking, the residual error is a function of $\mathcal { X } _ { k }$ that quantifies the mismatch between a measured quantity and the predicted value of this quantity given the state $\mathcal { X } _ { k }$ and the priors. The goal of the following sections is to provide expressions for the residual errors and the covariances.

## V. IMU MODEL AND MOTION INTEGRATION

An IMU commonly includes a 3-axis accelerometer and a 3-axis gyroscope and allows measuring the rotation rate and the acceleration of the sensor with respect to an inertial frame. The measurements, namely $\mathbf { \rho } _ { \mathrm { B } } \tilde { \mathbf { a } } ( t )$ , and $\phantom { } _ { \mathrm { B } } \tilde { \omega } _ { \mathrm { w B } } ( t )$ , are affected by additive white noise η and a slowly varying sensor bias b:

$$
{ } _ { \mathrm{B} } \tilde { \boldsymbol { \omega } } _ { \mathrm{WB} } ( t ) = { } _ { \mathrm{B} } \boldsymbol { \omega } _ { \mathrm{WB} } ( t ) + \mathbf { b } ^ { g } ( t ) + \boldsymbol { \eta } ^ { g } ( t )\tag{27}
$$

$$
{ } _ { \mathrm{B} } \tilde { \mathbf { a } } ( t ) = \mathsf { R } _ { \mathrm{WB} } ^ { \mathsf { T } } ( t ) \left( { } _ { \mathrm{w} } \mathbf { a } ( t ) - { } _ { \mathrm{w} } \mathbf { g } \right) + \mathbf { b } ^ { a } ( t ) + \boldsymbol { \eta } ^ { a } ( t ) ,\tag{28}
$$

In our notation, the prefix B denotes that the corresponding quantity is expressed in the frame B (c.f., Fig. 2). The pose of the IMU is described by the transformation $\{ \mathtt { R } _ { \mathtt { W B } } , \ O _ { \mathrm { W } } \}$ which maps a point from sensor frame B to W. The vector ${ \bf \Lambda } _ { \mathrm { B } } \omega _ { \mathrm { w B } } ( t ) \in \mathbb { R } ^ { 3 }$ is the instantaneous angular velocity of B relative to W expressed in coordinate frame B, while $\mathrm { w } \mathbf { a } ( t ) \in \mathbb { R } ^ { 3 }$ is the acceleration of the sensor; $\mathrm { { w } } \mathbf { \mathbf { g } }$ is the gravity vector in world coordinates. We neglect effects due to earth’s rotation, which amounts to assuming that W is an inertial frame.

The goal now is to infer the motion of the system from IMU measurements. For this purpose we introduce the following kinematic model [49, 53]:

$$
\dot {\mathsf {R}} _ {\mathrm{WB}} = \mathsf {R} _ {\mathrm{WB} \text { B }} \boldsymbol {\omega} _ {\mathrm{WB}} ^ {\wedge}, \qquad_ {\mathrm{w}} \dot {\mathbf {v}} = _ {\mathrm{w}} \mathbf {a}, \qquad_ {\mathrm{w}} \dot {\mathbf {p}} = _ {\mathrm{w}} \mathbf {v},\tag{29}
$$

which describes the evolution of the pose and velocity of B.

![](images/bd3686c022a21d14bce704a883b4632489e88b2ec9b04f6268bbdb63ac2d39f8.jpg)  
Fig. 4: Different rates for IMU and camera.

The state at time $t + \Delta t$ is obtained by integrating Eq. (29):

$$
\begin{array}{r l} & {\mathsf {R} _ {\mathrm{wB}} (t + \Delta t) = \mathsf {R} _ {\mathrm{wB}} (t) \operatorname{Exp} \left(\int_ {t} ^ {t + \Delta t} \omega_ {\mathrm{wB}} (\tau) d \tau\right)} \\ & {_ \mathrm{w} \mathbf {v} (t + \Delta t) = _ {\mathrm{w}} \mathbf {v} (t) + \int_ {t} ^ {t + \Delta t} _ {\mathrm{w}} \mathbf {a} (\tau) d \tau} \\ & {_ \mathrm{w} \mathbf {p} (t + \Delta t) = _ {\mathrm{w}} \mathbf {p} (t) + \int_ {t} ^ {t + \Delta t} _ {\mathrm{w}} \mathbf {v} (\tau) d \tau + \iint_ {t} ^ {t + \Delta t} _ {\mathrm{w}} \mathbf {a} (\tau) d \tau^ {2}.} \end{array}
$$

Assuming that $\mathbf { w } \mathbf { a }$ and $\phantom { } _ { \mathrm { B } } \omega _ { \mathrm { W B } }$ remain constant in the time interval $[ t , t + \Delta t ]$ , we can write:

$$
\begin{array}{l} \mathsf {R} _ {\mathrm{WB}} (t + \Delta t) = \mathsf {R} _ {\mathrm{WB}} (t)   \mathrm{Exp} \left(_ {\mathrm{B}} \boldsymbol {\omega} _ {\mathrm{WB}} (t) \Delta t\right) \\ \mathbf {\Lambda} _ {\mathrm{w}} \mathbf {v} (t + \Delta t) = \mathbf {\Lambda} _ {\mathrm{w}} \mathbf {v} (t) + \mathbf {\Lambda} _ {\mathrm{w}} \mathbf {a} (t) \Delta t \\ \mathbf {\Lambda} _ {\mathrm{w}} \mathbf {p} (t + \Delta t) = \mathbf {\Lambda} _ {\mathrm{w}} \mathbf {p} (t) + \mathbf {\Lambda} _ {\mathrm{w}} \mathbf {v} (t) \Delta t + \frac {1}{2} \mathbf {\Lambda} _ {\mathrm{w}} \mathbf {a} (t) \Delta t ^ {2}. \end{array}\tag{30}
$$

Using Eqs. (27)–(28), we can write $\mathbf { w } \mathbf { a }$ and ${ \bf \Pi } _ { \mathrm { B } } \omega _ { \mathrm { W B } }$ as a function of the IMU measurements, hence (30) becomes

$$
\begin{array}{l} \mathbb {R} (t + \Delta t) = \mathbb {R} (t) \operatorname{Exp} \left(\left(\tilde {\boldsymbol {\omega}} (t) - \mathbf {b} ^ {g} (t) - \boldsymbol {\eta} ^ {g d} (t)\right) \Delta t\right) \\ \mathbf {v} (t + \Delta t) = \mathbf {v} (t) + \mathbf {g} \Delta t + \mathbb {R} (t) \left(\tilde {\mathbf {a}} (t) - \mathbf {b} ^ {a} (t) - \boldsymbol {\eta} ^ {a d} (t)\right) \Delta t \\ \mathbf {p} (t + \Delta t) = \mathbf {p} (t) + \mathbf {v} (t) \Delta t + \frac {1}{2} \mathbf {g} \Delta t ^ {2} \\ + \frac {1}{2} \mathbb {R} (t) \left(\tilde {\mathbf {a}} (t) - \mathbf {b} ^ {a} (t) - \boldsymbol {\eta} ^ {a d} (t)\right) \Delta t ^ {2}, \end{array} \tag {3}\tag{\(\Delta t\}
$$

(31)

where we dropped the coordinate frame subscripts for readability (the notation should be unambiguous from now on). This numeric integration of the velocity and position assumes a constant orientation $\mathtt { R } ( t )$ for the time of integration between two measurements, which is not an exact solution of the differential equation (29) for measurements with non-zero rotation rate. In practice, the use of a high-rate IMU mitigates the effects of this approximation. We adopt the integration scheme (31) as it is simple and amenable for modeling and uncertainty propagation. While we show that this integration scheme performs very well in practice, we remark that for slower IMU measurement rates one may consider using higher-order numerical integration methods [54–57].

The covariance of the discrete-time noise $\eta ^ { g d }$ is a function of the sampling rate and relates to the continuous-time spectral noise $\eta ^ { g }$ via $\begin{array} { r } { \dot { \mathrm { C o v } } ( \pmb { \eta } ^ { g d } ( t ) ) = \frac { 1 } { \Delta t } \mathrm { C o v } ( \pmb { \eta } ^ { g } ( t ) ) } \end{array}$ . The same relation holds for η<sup>ad</sup> (cf., [58, Appendix]).

## VI. IMU PREINTEGRATION ON MANIFOLD

While Eq. (31) could be readily seen as a probabilistic constraint in a factor graph, it would require to include states in the factor graph at high rate. Intuitively, Eq. (31) relates states at time t and $t + \Delta t$ , where $\Delta t$ is the sampling period of the IMU, hence we would have to add new states in the estimation at every new IMU measurement [37].

Here we show that all measurements between two keyframes at times $k \ = \ i$ and k = j (see Fig. 4) can be summarized in a single compound measurement, named preintegrated IMU measurement, which constrains the motion between consecutive keyframes. This concept was first proposed in [2] using Euler angles and we extend it, by developing a suitable theory for preintegration on the manifold SO(3).

We assume that the IMU is synchronized with the camera and provides measurements at discrete times k (cf., Fig. 4).<sup>1</sup> Iterating the IMU integration (31) for all $\Delta t$ intervals between two consecutive keyframes at times $k = i$ and $k = j ~ ( c . f . , \mathrm { F i g }$ 4), we find:

$$
\begin{array}{l} \mathrm{R} _ {j} = \mathrm{R} _ {i} \prod_ {k = i} ^ {j - 1} \mathrm{Exp} \left(\left(\tilde {\boldsymbol {\omega}} _ {k} - \mathbf {b} _ {k} ^ {g} - \boldsymbol {\eta} _ {k} ^ {g d}\right) \Delta t\right), \\ \mathbf {v} _ {j} = \mathbf {v} _ {i} + \mathbf {g} \Delta t _ {i j} + \sum_ {k = i} ^ {j - 1} \mathrm{R} _ {k} \Big (\tilde {\mathbf {a}} _ {k} - \mathbf {b} _ {k} ^ {a} - \boldsymbol {\eta} _ {k} ^ {a d} \Big) \Delta t \\ \mathbf {p} _ {j} = \mathbf {p} _ {i} + \sum_ {k = i} ^ {j - 1} \left[ \mathbf {v} _ {k} \Delta t + \frac {1}{2} \mathbf {g} \Delta t ^ {2} + \frac {1}{2} \mathrm{R} _ {k} \Big (\tilde {\mathbf {a}} _ {k} - \mathbf {b} _ {k} ^ {a} - \boldsymbol {\eta} _ {k} ^ {a d} \Big) \Delta t ^ {2} \right] \end{array} \tag {32}
$$

where we introduced the shorthands $\begin{array} { r } { \Delta t { _ { i j } } \doteq \sum _ { k = i } ^ { j - 1 } \Delta t } \end{array}$ and $( \cdot ) _ { i } \doteq ( \cdot ) ( t _ { i } )$ for readability. While Eq. (32) already provides an estimate of the motion between time $t _ { i }$ and $t _ { j } ,$ , it has the drawback that the integration in (32) has to be repeated whenever the linearization point at time $t _ { i }$ changes [24] (intuitively, a change in the rotation $\mathtt { R } _ { i }$ implies a change in all future rotations $\mathtt { R } _ { k } , k = i , \dots , j - 1$ , and makes necessary to re-evaluate summations and products in (32)).

We want to avoid to recompute the above integration whenever the linearization point at time $t _ { i }$ changes. Therefore, we follow [2] and define the following relative motion increments that are independent of the pose and velocity at $t _ { i }$ :

$$
\begin{array}{l} \Delta \mathrm{R} _ {i j} \doteq \mathrm{R} _ {i} ^ {\top} \mathrm{R} _ {j} = \prod_ {k = i} ^ {j - 1} \operatorname{Exp} \left(\left(\tilde {\boldsymbol {\omega}} _ {k} - \mathbf {b} _ {k} ^ {g} - \boldsymbol {\eta} _ {k} ^ {g d}\right) \Delta t\right) \\ \Delta \mathbf {v} _ {i j} \doteq \mathrm{R} _ {i} ^ {\top} \left(\mathbf {v} _ {j} - \mathbf {v} _ {i} - \mathbf {g} \Delta t _ {i j}\right) = \sum_ {k = i} ^ {j - 1} \Delta \mathrm{R} _ {i k} \left(\tilde {\mathbf {a}} _ {k} - \mathbf {b} _ {k} ^ {a} - \boldsymbol {\eta} _ {k} ^ {a d}\right) \Delta t \\ \Delta \mathbf {p} _ {i j} \doteq \mathrm{R} _ {i} ^ {\top} \left(\mathbf {p} _ {j} - \mathbf {p} _ {i} - \mathbf {v} _ {i} \Delta t _ {i j} - \frac {1}{2} \mathbf {g} \Delta t _ {i j} ^ {2}\right) \\ = \sum_ {k = i} ^ {j - 1} \left[ \Delta \mathbf {v} _ {i k} \Delta t + \frac {1}{2} \Delta \mathrm{R} _ {i k} \left(\tilde {\mathbf {a}} _ {k} - \mathbf {b} _ {k} ^ {a} - \boldsymbol {\eta} _ {k} ^ {a d}\right) \Delta t ^ {2} \right] \end{array} (3 3)
$$

where $\Delta \mathtt { R } _ { i k } \doteq \mathtt { R } _ { i } ^ { \top } \mathtt { R } _ { k }$ and $\Delta \mathbf { v } _ { i k } \doteq \mathsf { R } _ { i } ^ { \mathsf { T } } \left( \mathbf { v } _ { k } - \mathbf { v } _ { i } - \mathbf { g } \Delta t _ { i k } \right)$ . We highlight that, in contrast to the “delta” rotation $\Delta \mathtt { R } _ { i j }$ , neither $\Delta \mathbf { v } _ { i j }$ nor $\Delta \mathbf { p } _ { i j }$ correspond to the true physical change in velocity and position but are defined in a way that make the right-hand side of (33) independent from the state at time i as well as gravitational effects. Indeed, we will be able to compute the right-hand side of (33) directly from the inertial measurements between the two keyframes.

Unfortunately, summations and products in (33) are still function of the bias estimate. We tackle this problem in two steps. In Section VI-A, we assume $\mathbf { b } _ { i }$ is known; then, in Section VI-C we show how to avoid repeating the integration when the bias estimate changes.

In the rest of the paper, we assume that the bias remains constant between two keyframes:

$$
\mathbf {b} _ {i} ^ {g} = \mathbf {b} _ {i + 1} ^ {g} = \dots = \mathbf {b} _ {j - 1} ^ {g}, \quad \mathbf {b} _ {i} ^ {a} = \mathbf {b} _ {i + 1} ^ {a} = \dots = \mathbf {b} _ {j - 1} ^ {a}.\tag{34}
$$

## A. Preintegrated IMU Measurements

Equation (33) relates the states of keyframes i and $j$ (lefthand side) to the measurements (right-hand side). In this sense, it can be already understood as a measurement model. Unfortunately, it has a fairly intricate dependence on the measurement noise and this complicates a direct application of MAP estimation; intuitively, the MAP estimator requires to clearly define the densities (and their log-likelihood) of the measurements. In this section we manipulate (33) so to make easier the derivation of the measurement log-likelihood. In practice, we isolate the noise terms of the individual inertial measurements in (33). As discussed above, across this section assume that the bias at time $t _ { i }$ is known.

Let us start with the rotation increment $\Delta \mathtt { R } _ { i j }$ in (33). We use the first-order approximation (7) (rotation noise is “small”) and rearrange the terms, by “moving” the noise to the end, using the relation (11):

$$
\begin{array}{l} \Delta \mathrm{R} _ {i j} \stackrel {\text {eq.} (7)} {\simeq} \prod_ {k = i} ^ {j - 1} \left[ \operatorname{Exp} \left(\left(\tilde {\boldsymbol {\omega}} _ {k} - \mathbf {b} _ {i} ^ {g}\right) \Delta t\right) \operatorname{Exp} \left(- \mathrm{J} _ {r} ^ {k} \boldsymbol {\eta} _ {k} ^ {g d} \Delta t\right) \right] \\ \stackrel {\text {eq.} (1 1)} {=} \Delta \tilde {\mathrm{R}} _ {i j} \prod_ {k = i} ^ {j - 1} \operatorname{Exp} \left(- \Delta \tilde {\mathrm{R}} _ {k + 1 j} ^ {\top} \mathrm{J} _ {r} ^ {k} \boldsymbol {\eta} _ {k} ^ {g d} \Delta t\right) \\ \stackrel {\cdot} {=} \Delta \tilde {\mathrm{R}} _ {i j} \operatorname{Exp} \left(- \delta \phi_ {i j}\right) \end{array} \tag {3}\tag{35}
$$

with $\begin{array} { c c l } { { \bf J } _ { r } ^ { k } } & { \doteq } & { \Im _ { r } ^ { k } \big ( \big ( \tilde { \omega } _ { k } - { \bf b } _ { i } ^ { g } \big ) \Delta t \big ) } \end{array}$ . In the last line of (35), we defined the preintegrated rotation measurement $\Delta \tilde { \mathrm { R } } _ { i j } \ \dot { = }$ $\Pi _ { k = i } ^ { j - 1 }$ Exp $( ( \tilde { \omega } _ { k } - \mathbf { b } _ { i } ^ { g } ) \Delta t )$ , and its noise $\delta \phi _ { i j }$ , which will be further analysed in the next section.

Substituting (35) back into the expression of $\Delta \mathbf { v } _ { i j }$ in (33), using the first-order approximation (4) for $\mathrm { E x p } \left( - \delta \mathbf { \bar { \phi } } _ { i j } \right)$ , and dropping higher-order noise terms, we obtain:

$$
\begin{array}{l} \Delta \mathbf {v} _ {i j} \stackrel {\text {eq. (4)}} {\simeq} \sum_ {k = i} ^ {j - 1} \Delta \tilde {\mathrm{R}} _ {i k} (\mathbf {I} - \delta \phi_ {i k} ^ {\wedge}) \left(\tilde {\mathbf {a}} _ {k} - \mathbf {b} _ {i} ^ {a}\right) \Delta t - \Delta \tilde {\mathrm{R}} _ {i k} \boldsymbol {\eta} _ {k} ^ {a d} \Delta t \\ \stackrel {\text {eq. (2)}} {=} \Delta \tilde {\mathbf {v}} _ {i j} + \sum_ {k = i} ^ {j - 1} \left[ \Delta \tilde {\mathrm{R}} _ {i k} \left(\tilde {\mathbf {a}} _ {k} - \mathbf {b} _ {i} ^ {a}\right) ^ {\wedge} \delta \phi_ {i k} \Delta t - \Delta \tilde {\mathrm{R}} _ {i k} \boldsymbol {\eta} _ {k} ^ {a d} \Delta t \right] \\ \doteq \Delta \tilde {\mathbf {v}} _ {i j} - \delta \mathbf {v} _ {i j} \end{array} \tag {36}
$$

where we defined the preintegrated velocity measurement $\begin{array} { r } { \Delta \tilde { \mathbf { v } } _ { i j } \doteq \sum _ { k = i } ^ { j - 1 } \Delta \tilde { \mathrm { R } } _ { i k } ( \tilde { \mathbf { a } } _ { k } - \mathbf { b } _ { i } ^ { a } ) \Delta t } \end{array}$ and its noise $\delta \mathbf { v } _ { i j }$

Similarly, substituting (35) and (36) in the expression of $\Delta \mathbf { p } _ { i j }$ in (33), and using the first-order approximation (4), we

obtain:

$$
\begin{array}{l} \Delta \mathbf {p} _ {i j} \stackrel {\text {eq. (4)}} {\simeq} \sum_ {k = i} ^ {j - 1} \left[ (\Delta \tilde {\mathbf {v}} _ {i k} - \delta \mathbf {v} _ {i k}) \Delta t + \frac {1}{2} \Delta \tilde {\mathbf {R}} _ {i k} (\mathbf {I} - \delta \phi_ {i k} ^ {\wedge}) (\tilde {\mathbf {a}} _ {k} - \mathbf {b} _ {i} ^ {a}) \Delta t ^ {2} \right. \\ \left. - \frac {1}{2} \Delta \tilde {\mathbf {R}} _ {i k} \boldsymbol {\eta} _ {k} ^ {a d} \Delta t ^ {2} \right] \\ \stackrel {\text {eq. (2)}} {=} \Delta \tilde {\mathbf {p}} _ {i j} + \sum_ {k = i} ^ {j - 1} \left[ - \delta \mathbf {v} _ {i k} \Delta t + \frac {1}{2} \Delta \tilde {\mathbf {R}} _ {i k} (\tilde {\mathbf {a}} _ {k} - \mathbf {b} _ {i} ^ {a}) ^ {\wedge} \delta \phi_ {i k} \Delta t ^ {2} \right. \\ \left. - \frac {1}{2} \Delta \tilde {\mathbf {R}} _ {i k} \boldsymbol {\eta} _ {k} ^ {a d} \Delta t ^ {2} \right] \\ \doteq \Delta \tilde {\mathbf {p}} _ {i j} - \delta \mathbf {p} _ {i j}, \end{array} \tag {3}\tag{37}
$$

where we defined the preintegrated position measurement $\Delta \tilde { \mathbf { p } } _ { i j }$ and its noise $\delta \mathbf { p } _ { i j }$

Substituting the expressions (35), (36), (37) back in the original definition of $\Delta \mathrm { R } _ { i j } , \Delta \mathbf { v } _ { i j } , \Delta \mathbf { p } _ { i j }$ in (33), we finally get our preintegrated measurement model (remember Exp $\left( - \delta \phi _ { i j } \right) ^ { \mathsf { T } } = \mathrm { E x p } \left( \delta \phi _ { i j } \right) ^ { \mathsf { \prime } }$ ):

$$
\begin{array}{r l} & {\Delta \tilde {\mathbf {R}} _ {i j} = \mathsf {R} _ {i} ^ {\mathsf {T}} \mathsf {R} _ {j} \mathrm{Exp} (\delta \phi_ {i j})} \\ & {\Delta \tilde {\mathbf {v}} _ {i j} = \mathsf {R} _ {i} ^ {\mathsf {T}} (\mathbf {v} _ {j} - \mathbf {v} _ {i} - \mathbf {g} \Delta t _ {i j}) + \delta \mathbf {v} _ {i j}} \\ & {\Delta \tilde {\mathbf {p}} _ {i j} = \mathsf {R} _ {i} ^ {\mathsf {T}} (\mathbf {p} _ {j} - \mathbf {p} _ {i} - \mathbf {v} _ {i} \Delta t _ {i j} - \frac {1}{2} \mathbf {g} \Delta t _ {i j} ^ {2}) + \delta \mathbf {p} _ {i j}} \end{array}\tag{38}
$$

where our compound measurements are written as a function of the (to-be-estimated) state “plus” a random noise, described by the random vector $[ \delta \phi _ { i j } ^ { \mathsf { T } } , \delta \mathbf { \bar { v } } _ { i j } ^ { \mathsf { T } } , \delta \mathbf { p } _ { i j } ^ { \mathsf { T } } ] ^ { \mathsf { T } }$

To wrap-up the discussion in this section, we manipulated the measurement model (33) and rewrote it as (38). The advantage of Eq. (38) is that, for a suitable distribution of the noise, it makes the definition of the log-likelihood straightforward. For instance the (negative) log-likelihood of measurements with zero-mean additive Gaussian noise (last two lines in (38)) is a quadratic function. Similarly, if $\delta \phi _ { i j }$ is a zero-mean Gaussian noise, we compute the (negative) loglikelihood associated with $\Delta \tilde { \mathrm { R } } _ { i j }$ . The nature of the noise terms is discussed in the following section.

## B. Noise Propagation

In this section we derive the statistics of the noise vector $[ \delta \phi _ { i j } ^ { \mathsf { T } } , \delta \mathbf { v } _ { i j } ^ { \mathsf { T } } , \delta \mathbf { p } _ { i j } ^ { \mathsf { T } } ] ^ { \mathsf { T } }$ . While we already observed that it is convenient to approximate the noise vector to be zero-mean Normally distributed, it is of paramount importance to accurately model the noise covariance. Indeed, the noise covariance has a strong influence on the MAP estimator (the inverse noise covariance is used to weight the terms in the optimization (26)). In this section, we therefore provide a derivation of the covariance $\Sigma _ { i j }$ of the preintegrated measurements:

$$
\boldsymbol {\eta} _ {i j} ^ {\Delta} \doteq [ \delta \boldsymbol {\phi} _ {i j} ^ {\mathsf {T}}, \delta \mathbf {v} _ {i j} ^ {\mathsf {T}}, \delta \mathbf {p} _ {i j} ^ {\mathsf {T}} ] ^ {\mathsf {T}} \sim \mathcal {N} (\mathbf {0} _ {9 \times 1}, \boldsymbol {\Sigma} _ {i j}).\tag{39}
$$

We first consider the preintegrated rotation noise $\delta \phi _ { i j }$ Recall from (35) that

$$
\operatorname{Exp} \left(- \delta \phi_ {i j}\right) \doteq \prod_ {k = i} ^ {j - 1} \operatorname{Exp} \left(- \Delta \tilde {\mathsf {R}} _ {k + 1 j} ^ {\mathsf {T}} \mathsf {J} _ {r} ^ {k}   \boldsymbol {\eta} _ {k} ^ {g d}   \Delta t\right).\tag{40}
$$

Taking the Log on both sides and changing signs, we get:

$$
\delta \phi_ {i j} = - \mathrm{Log} \left(\prod_ {k = i} ^ {j - 1} \mathrm{Exp} \left(- \Delta \tilde {\mathsf {R}} _ {k + 1 j} ^ {\mathsf {T}} \mathsf {J} _ {r} ^ {k}   \boldsymbol {\eta} _ {k} ^ {g d}   \Delta t\right)\right).\tag{41}
$$

Repeated application of the first-order approximation (9) (recall that $\eta _ { k } ^ { \dot { g } \dot { d } }$ as well as $\delta \phi _ { i j }$ are small rotation noises, hence the right Jacobians are close to the identity) produces:

$$
\delta \phi_ {i j} \simeq \sum_ {k = i} ^ {j - 1} \Delta \tilde {\mathsf {R}} _ {k + 1 j} ^ {\mathsf {T}} \mathsf {J} _ {r} ^ {k} \boldsymbol {\eta} _ {k} ^ {g d} \Delta t\tag{42}
$$

Up to first order, the noise $\delta \phi _ { i j }$ is zero-mean and Gaussian, as it is a linear combination of zero-mean noise terms $\eta _ { k } ^ { g d }$ . This is desirable, since it brings the rotation measurement model (38) exactly in the form (12).

Dealing with the noise terms $\delta \mathbf { v } _ { i j }$ and $\delta \mathbf { p } _ { i j }$ is now easy: these are linear combinations of the acceleration noise $\eta _ { k } ^ { \dot { a } \dot { d } }$ and the preintegrated rotation noise $\delta \phi _ { i j }$ , hence they are also zero-mean and Gaussian. Simple manipulation leads to:

$$
\begin{array}{l} \delta \mathbf {v} _ {i j} \simeq \sum_ {k = i} ^ {j - 1} \left[ - \Delta \tilde {\mathbf {R}} _ {i k} \left(\tilde {\mathbf {a}} _ {k} - \mathbf {b} _ {i} ^ {a}\right) ^ {\wedge} \delta \phi_ {i k} \Delta t + \Delta \tilde {\mathbf {R}} _ {i k} \boldsymbol {\eta} _ {k} ^ {a d} \Delta t \right] \\ \delta \mathbf {p} _ {i j} \simeq \sum_ {k = i} ^ {j - 1} \left[ \delta \mathbf {v} _ {i k} \Delta t - \frac {1}{2} \Delta \tilde {\mathbf {R}} _ {i k} \left(\tilde {\mathbf {a}} _ {k} - \mathbf {b} _ {i} ^ {a}\right) ^ {\wedge} \delta \phi_ {i k} \Delta t ^ {2} + \frac {1}{2} \Delta \tilde {\mathbf {R}} _ {i k} \boldsymbol {\eta} _ {k} ^ {a d} \Delta t ^ {2} \right] \end{array} \tag {43}
$$

where the relations are valid up to the first order.

Eqs. (42)-(43) express the preintegrated noise $\eta _ { i j } ^ { \Delta }$ as a linear function of the IMU measurement noise $\pmb { \eta } _ { k } ^ { d } \doteq [ \pmb { \eta } _ { k } ^ { g \bar { d } } , \pmb { \eta } _ { k } ^ { a d } ] , k =$ $1 , \ldots , j - 1$ . Therefore, from the knowledge of the covariance of $\eta _ { k } ^ { d }$ (given in the IMU specifications), we can compute the covariance of $\eta _ { i j } ^ { \Delta }$ , namely $\Sigma _ { i j }$ , by a simple linear propagation.

In Appendix IX-A, we provide a more clever way to compute $\Sigma _ { i j }$ . In particular, we show that $\Sigma _ { i j }$ can be conveniently computed in iterative form: as a new IMU measurement arrive we only update $\Sigma _ { i j }$ , rather than recomputing it from scratch. The iterative computation leads to simpler expressions and is more amenable for online inference.

## C. Incorporating Bias Updates

In the previous section, we assumed that the bias $\{ \bar { \mathbf { b } } _ { i } ^ { a } , \bar { \mathbf { b } } _ { i } ^ { g } \}$ that is used during preintegration between $k = i$ and $k = j$ is correct and does not change. However, more likely, the bias estimate changes by a small amount δb during optimization. One solution would be to recompute the delta measurements when the bias changes; however, that is computationally expensive. Instead, given a bias update b $ \bar { \mathbf { b } } + \delta \mathbf { b }$ , we can update the delta measurements using a first-order expansion:

$$
\begin{array}{c} \Delta \tilde {\mathbf {R}} _ {i j} (\mathbf {b} _ {i} ^ {g}) \simeq \Delta \tilde {\mathbf {R}} _ {i j} (\bar {\mathbf {b}} _ {i} ^ {g}) \operatorname{Exp} \left(\frac {\partial \Delta \bar {\mathbf {R}} _ {i j}}{\partial \mathbf {b} ^ {g}} \delta \mathbf {b} ^ {g}\right) \\ \Delta \tilde {\mathbf {v}} _ {i j} (\mathbf {b} _ {i} ^ {g}, \mathbf {b} _ {i} ^ {a}) \simeq \Delta \tilde {\mathbf {v}} _ {i j} (\bar {\mathbf {b}} _ {i} ^ {g}, \bar {\mathbf {b}} _ {i} ^ {a}) + \frac {\partial \Delta \bar {\mathbf {v}} _ {i j}}{\partial \mathbf {b} ^ {g}} \delta \mathbf {b} _ {i} ^ {g} + \frac {\partial \Delta \bar {\mathbf {v}} _ {i j}}{\partial \mathbf {b} ^ {a}} \delta \mathbf {b} _ {i} ^ {a} \\ \Delta \tilde {\mathbf {p}} _ {i j} (\mathbf {b} _ {i} ^ {g}, \mathbf {b} _ {i} ^ {a}) \simeq \Delta \tilde {\mathbf {p}} _ {i j} (\bar {\mathbf {b}} _ {i} ^ {g}, \bar {\mathbf {b}} _ {i} ^ {a}) + \frac {\partial \Delta \tilde {\mathbf {p}} _ {i j}}{\partial \mathbf {b} ^ {g}} \delta \mathbf {b} _ {i} ^ {g} + \frac {\partial \Delta \tilde {\mathbf {p}} _ {i j}}{\partial \mathbf {b} ^ {a}} \delta \mathbf {b} _ {i} ^ {a} \end{array} \tag {44}\tag{44}
$$

This is similar to the bias correction in [2] but operates directly on SO(3). The Jacobians $\begin{array} { r } { \{ \frac { \partial \Delta \bar { \bf R } _ { i j } } { \partial { \bf b } ^ { g } } , \frac { \partial \Delta \bar { \bf v } _ { i j } } { \partial { \bf b } ^ { g } } , . . . \} } \end{array}$ (computed at $\bar { \mathbf { b } } _ { i } ,$ the bias estimate at integration time) describe how the measurements change due to a change in the bias estimate. The Jacobians remain constant and can be precomputed during the preintegration. The derivation of the Jacobians is very similar to the one we used in Section VI-A to express the measurements as a large value plus a small perturbation and is given in Appendix IX-B.

## D. Preintegrated IMU Factors

Given the preintegrated measurement model in (38) and since measurement noise is zero-mean and Gaussian (with covariance $\Sigma _ { i j } )$ up to first order (39), it is now easy to write the residual errors $\mathbf { r } _ { \mathbb { Z } _ { i j } } \doteq [ \mathbf { r } _ { \Delta \mathsf { R } _ { i j } } ^ { \mathsf { T } } , \mathbf { r } _ { \Delta \mathbf { v } _ { i j } } ^ { \mathsf { T } } , \mathbf { r } _ { \Delta \mathbf { p } _ { i j } } ^ { \mathsf { T } } ] ^ { \mathsf { T } } \in \bar { \mathbb { R } } ^ { 9 }$ , where

$$
\begin{array}{r l} & {\mathbf {r} _ {\Delta \mathsf {R} _ {i j}} \doteq \mathrm{Log} \left(\left(\Delta \tilde {\mathsf {R}} _ {i j} (\bar {\mathbf {b}} _ {i} ^ {g}) \mathrm{Exp} \left(\frac {\partial \Delta \bar {\mathsf {R}} _ {i j}}{\partial \mathbf {b} ^ {g}} \delta \mathbf {b} ^ {g}\right)\right) ^ {\mathsf {T}} \mathsf {R} _ {i} ^ {\mathsf {T}} \mathsf {R} _ {j}\right)} \\ & {\mathbf {r} _ {\Delta \mathbf {v} _ {i j}} \doteq \mathsf {R} _ {i} ^ {\mathsf {T}} (\mathbf {v} _ {j} - \mathbf {v} _ {i} - \mathbf {g} \Delta t _ {i j})} \\ & {- \left[ \Delta \tilde {\mathbf {v}} _ {i j} (\bar {\mathbf {b}} _ {i} ^ {g}, \bar {\mathbf {b}} _ {i} ^ {a}) + \frac {\partial \Delta \bar {\mathbf {v}} _ {i j}}{\partial \mathbf {b} ^ {g}} \delta \mathbf {b} ^ {g} + \frac {\partial \Delta \bar {\mathbf {v}} _ {i j}}{\partial \mathbf {b} ^ {a}} \delta \mathbf {b} ^ {a} \right]} \\ & {\mathbf {r} _ {\Delta \mathbf {p} _ {i j}} \doteq \mathsf {R} _ {i} ^ {\mathsf {T}} (\mathbf {p} _ {j} - \mathbf {p} _ {i} - \mathbf {v} _ {i} \Delta t _ {i j} - \frac 12 \mathbf {g} \Delta t _ {i j} ^ {2})} \\ & {- \left[ \Delta \tilde {\mathbf {p}} _ {i j} (\bar {\mathbf {b}} _ {i} ^ {g}, \bar {\mathbf {b}} _ {i} ^ {a}) + \frac {\partial \Delta \bar {\mathbf {p}} _ {i j}}{\partial \mathbf {b} ^ {g}} \delta \mathbf {b} ^ {g} + \frac {\partial \Delta \bar {\mathbf {p}} _ {i j}}{\partial \mathbf {b} _ {a}} \delta \mathbf {b} ^ {a} \right],} \end{array}\tag{45}
$$

in which we also included the bias updates of Eq. (44).

According to the “lift-solve-retract” method (Section III-C), at each Gauss-Newton iteration we need to re-parametrize (45) using the retraction (21). Then, the “solve” step requires to linearize the resulting cost around the current estimate. For the purpose of linearization, it is convenient to compute analytic expressions of the Jacobians of the residual errors, which we derive in the Appendix IX-C.

## E. Bias Model

When presenting the IMU model (27), we said that biases are slowly time-varying quantities. Hence, we model them with a “Brownian motion”, i.e., integrated white noise:

$$
\dot {\mathbf {b}} ^ {g} (t) = \boldsymbol {\eta} ^ {b g}, \quad \dot {\mathbf {b}} ^ {a} (t) = \boldsymbol {\eta} ^ {b a}.\tag{46}
$$

Integrating (46) over the time interval $[ t _ { i } , t _ { j } ]$ between two consecutive keyframes i and $j$ we get:

$$
\mathbf {b} _ {j} ^ {g} = \mathbf {b} _ {i} ^ {g} + \boldsymbol {\eta} ^ {b g d}, \qquad \mathbf {b} _ {j} ^ {a} = \mathbf {b} _ {i} ^ {a} + \boldsymbol {\eta} ^ {b a d},\tag{47}
$$

where, as done before, we use the shorthand $\mathbf { b } _ { i } ^ { g } \doteq \mathbf { b } ^ { g } ( t _ { i } )$ and we define the discrete noises $\eta ^ { b g d }$ and $\eta ^ { b a d }$ , which have zero mean and covariance $\Sigma ^ { b g d } \doteq \bar { \Delta } t _ { i j } \mathrm { C o v } ( \dot { \pmb { \eta } } ^ { b g } )$ and $\pmb { \Sigma } ^ { b a d } \dot { = }$ $\Delta t _ { i j } \mathbf { C o v } ( \pmb { \eta } ^ { b a } )$ , respectively (cf. [58, Appendix]).

The model (47) can be readily included in our factor graph, as a further additive term in (26) for all consecutive keyframes:

$$
\left\| \mathbf {r} _ {\mathbf {b} _ {i j}} \right\| ^ {2} \doteq \left\| \mathbf {b} _ {j} ^ {g} - \mathbf {b} _ {i} ^ {g} \right\| _ {\boldsymbol {\Sigma} ^ {b g d}} ^ {2} + \left\| \mathbf {b} _ {j} ^ {a} - \mathbf {b} _ {i} ^ {a} \right\| _ {\boldsymbol {\Sigma} ^ {b a d}} ^ {2}\tag{48}
$$

## VII. STRUCTURELESS VISION FACTORS

In this section we introduce our structureless model for vision measurements. The key feature of our approach is the linear elimination of landmarks. Note that the elimination is repeated at each Gauss-Newton iteration, hence we are still guaranteed to obtain the optimal MAP estimate.

Visual measurements contribute to the cost (26) via the sum:

$$
\sum_ {i \in \mathcal {K} _ {k}} \sum_ {l \in \mathcal {C} _ {i}} \| \mathbf {r} _ {\mathcal {C} _ {i l}} \| _ {\boldsymbol {\Sigma} _ {\mathcal {C}}} ^ {2} = \sum_ {l = 1} ^ {L} \sum_ {i \in \mathcal {X} (l)} \| \mathbf {r} _ {\mathcal {C} _ {i l}} \| _ {\boldsymbol {\Sigma} _ {\mathcal {C}}} ^ {2}\tag{49}
$$

which, on the right-hand-side, we rewrote as a sum of contributions of each landmark $l = 1 , \ldots , L$ . In (49), X(l) denotes the subset of keyframes in which l is seen.

A fairly standard model for the residual error of a single image measurement $\mathbf { z } _ { i l }$ is the reprojection error:

$$
\mathbf {r} _ {\mathcal {C} _ {i l}} = \mathbf {z} _ {i l} - \pi (\mathrm{R} _ {i}, \mathbf {p} _ {i}, \rho_ {l}),\tag{50}
$$

where $\rho _ { l } \in \mathbb { R } ^ { 3 }$ denotes the position of the l-th landmark, and $\pi ( \cdot )$ is a standard perspective projection, which also encodes the (known) IMU-camera transformation $\mathrm { T } _ { \mathrm { B C } }$

Direct use of (50) would require to include the landmark positions $\rho _ { l } , l = 1 , \ldots , L$ in the optimization, and this impacts negatively on computation. Therefore, in the following we adopt a structureless approach that avoids optimization over the landmarks, thus ensuring to retrieve the MAP estimate.

As recalled in Section III-C, at each GN iteration, we lift the cost function, using the retraction (21). For the vision factors this means that the original residuals (49) become:

$$
\sum_ {l = 1} ^ {L} \sum_ {i \in \mathcal {X} (l)} \| \mathbf {z} _ {i l} - \check {\pi} (\delta \phi_ {i}, \delta \mathbf {p} _ {i}, \delta \rho_ {l}) \| _ {\boldsymbol {\Sigma} _ {\mathcal {C}}} ^ {2}\tag{51}
$$

where $\delta \phi _ { i } , \delta \mathbf { p } _ { i } , \delta \rho _ { l }$ are now Euclidean corrections, and $\check { \pi } ( \cdot )$ is the lifted cost function. The “solve” step in the GN method is based on linearization of the residuals:

$$
\sum_ {l = 1} ^ {L} \sum_ {i \in \mathcal {X} (l)} \| \mathbf {F} _ {i l} \delta \mathbf {T} _ {i} + \mathbf {E} _ {i l} \delta \rho_ {l} - \mathbf {b} _ {i l} \| ^ {2},\tag{52}
$$

where $\delta { \bf T } _ { i } \doteq [ \delta \phi _ { i } \delta { \bf p } _ { i } ] ^ { \scriptscriptstyle \dag } ;$ ; the Jacobians $\mathbf { F } _ { i l } , \mathbf { E } _ { i l }$ , and the vector $\mathbf { b } _ { i l }$ (both normalized by $\Sigma _ { \mathcal { C } } ^ { 1 / 2 } )$ result from the linearization. The vector $\mathbf { b } _ { i l }$ is the residual error at the linearization point.

Writing the second sum in (52) in matrix form we get:

$$
\sum_ {l = 1} ^ {L} \left\| \mathbf {F} _ {l} \delta \mathbf {T} _ {\mathcal {X} (l)} + \mathbf {E} _ {l} \delta \rho_ {l} - \mathbf {b} _ {l} \right\| ^ {2}\tag{53}
$$

where $\mathbf { F } _ { l } , \mathbf { E } _ { l } , \mathbf { b } _ { l }$ are obtained by stacking $\mathbf { F } _ { i l } , \mathbf { E } _ { i l } , \mathbf { b } _ { i l }$ , respectively, for all $i \in \mathcal { X } ( l )$

Since a landmark l appears in a single term of the sum (53), for any given choice of the pose perturbation $\delta \mathbf { T } _ { \mathcal { X } ( l ) }$ , the landmark perturbation $\delta \rho _ { l }$ that minimizes the quadratic cost $\| { \bf F } _ { l } \delta { \bf T } _ { { \mathcal { X } } ( l ) } + { \bf E } _ { l } \delta \rho _ { l } - { \bf b } _ { l } \| ^ { 2 }$ is:

$$
\delta \rho_ {l} = - (\mathbf {E} _ {l} ^ {\mathsf {T}} \mathbf {E} _ {l}) ^ {- 1} \mathbf {E} _ {l} ^ {\mathsf {T}} (\mathbf {F} _ {l} \delta \mathbf {T} _ {\mathcal {X} (l)} - \mathbf {b} _ {l})\tag{54}
$$

Substituting (54) back into (53) we can eliminate the variable $\delta \rho _ { l }$ from the optimization problem:

$$
\sum_ {l = 1} ^ {L} \left\| \left(\mathbf {I} - \mathbf {E} _ {l} (\mathbf {E} _ {l} ^ {\top} \mathbf {E} _ {l}) ^ {- 1} \mathbf {E} _ {l} ^ {\top}\right) \left(\mathbf {F} _ {l} \delta \mathbf {T} _ {\mathcal {X} (l)} - \mathbf {b} _ {l}\right) \right\| ^ {2},\tag{55}
$$

where $\mathbf { I } - \mathbf { E } _ { l } \bigl ( \mathbf { E } _ { l } ^ { \mathsf { T } } \mathbf { E } _ { l } \bigr ) ^ { - 1 } \mathbf { E } _ { l } ^ { \mathsf { T } }$ is an orthogonal projector of $\mathbf { E } _ { l }$ In Appendix IX-D we show that the cost (55) can be further manipulated, leading to a more efficient implementation.

This approach is well known in the bundle adjustment literature as the Schur complement trick, where a standard practice is to update the linearization point of $\rho _ { l }$ via backsubstitution [61]. In contrast, we obtain the updated landmark positions from the linearization point of the poses using a fast linear triangulation. Using this approach, we reduced a large set of factors (51) which involve poses and landmarks into a smaller set of L factors (55), which only involve poses. In particular, the factor corresponding to landmark l only involves the states $\mathcal { X } ( l )$ observing l, creating the connectivity pattern of Fig. 3. The same approach is also used in MSC-KF [5] to avoid the inclusion of landmarks in the state vector.

![](images/63d27eb38fa03e52acbe996a772b04ba0770ed023314486338917b7ebec99e8c.jpg)  
Fig. 5: Simulation setup: The camera moves along a circular trajectory while observing features (green lines) on the walls of a square environment.

However, since MSC-KF can only linearize and absorb a measurement once, the processing of measurements needs to be delayed until all measurements of the same landmark are observed. This does not apply to the proposed optimizationbased approach, which allows for multiple relinearizations and the incremental inclusion of new measurements.

## VIII. EXPERIMENTAL ANALYSIS

We tested the proposed approach on both simulated and real data. Section VIII-A reports simulation results, showing that our approach is accurate, fast, and consistent. Section VIII-B compares our approach against the state-of-the-art, confirming its superior accuracy in real indoor and outdoor experiments.

## A. Simulation Experiments

We simulated a camera following a circular trajectory of three meter radius with a sinusoidal vertical motion. The total length of the trajectory is 120 meters. While moving, the camera observes landmarks as depicted in Fig. 5. The number of landmark observations per frame is limited to 50. To simulate a realistic feature-tracker, we corrupt the landmark measurements with isotropic Gaussian noise with standard deviation $\sigma _ { \mathrm { p x } } = 1$ pixel. The camera has a focal length of 315 pixels and runs at a rate of 2.5 Hz (simulating keyframes). The simulated acceleration and gyroscope measurements are computed from the analytic derivatives of the parametric trajectory and additionally corrupted by white noise and a slowly time-varying bias terms, according to the IMU model in Eq. (27).<sup>2</sup> To evaluate our approach, we performed a Monte Carlo analysis with 50 simulation runs, each with different realizations of process and measurement noise. In each run we compute the MAP estimate using the IMU and the vision models presented in this paper. The optimization (whose solution is the MAP estimate) is solved using the incremental smoothing algorithm iSAM2 [3]. iSAM2 uses the Bayes tree [62] data structure to obtain efficient variable ordering that minimizes fill-in in the square-root information matrix and, thus, minimizes computation time. Further, iSAM2 exploits the fact that new measurements often have only local effect on the MAP estimate, hence applies incremental updates directly to the square-root information matrix, only re-solving for the variables affected by a new measurement.

![](images/db1fec14a1ae712550c01d0887141053b9b383498aa428c6c00d12de1e7289c4.jpg)  
Fig. 6: Left: CPU time required for inference, comparing batch estimation against iSAM2. Right: histogram plot of CPU time for the proposed approach.

In the following we present the results of our experiments, organized in four subsections: 1) pose estimation accuracy and timing, 2) consistency, 3) bias estimation accuracy, and 4) firstorder bias correction. Then, in Section VIII-A5 we compare our approach against the original proposal of [2].

1) Pose Estimation Accuracy and Timing: The optimal MAP estimate is given by the batch nonlinear optimization of the least-squares objective in Eq. (26). However, as shown on the left in Fig. 6, the computational cost of batch optimization quickly increases as the trajectory length grows. A key ingredient that makes our approach extremely efficient is the use of the incremental smoothing algorithm iSAM2 [3], which performs close-to-optimal inference, while preserving real-time capability. Fig. 7 shows that the accuracy of iSAM2 is practically the same as the batch estimate. In odometry problems, the iSAM2 algorithm results in approximately constant update time per frame (Fig. 6, left), which in our experiment is approximately 10 milliseconds per update (Fig. 6, right).

2) Consistency: For generic motion, the VIO problem has four unobservable degrees of freedom, three corresponding to the global translation and one to the global orientation around the gravity direction (yaw), see [16]. A VIO algorithm must preserve these observability properties and avoid inclusion of spurious information along the unobservable directions, which would result in inconsistency [16]. Fig. 8 reports orientation and position errors with the corresponding 3σ bounds, confirming that our approach is consistent. In the VIO problem, the gravity direction is observable, hence the uncertainty on roll and pitch remains bounded. In contrast, global yaw and position cannot be measured and the uncertainty slowly grows over time.

To present more substantial evidence of the fact that our estimator is consistent, we recall a standard measure of consistency, the average Normalized Estimation Error Squared (NEES) [63]. The NEES is the squared estimation error $\epsilon _ { k }$ normalized by the estimator-calculated covariance $\Sigma _ { k }$

$$
\eta_ {k} \doteq \epsilon_ {k} ^ {\mathsf {T}} \hat {\Sigma} _ {k} ^ {- 1} \epsilon_ {k}
$$

(NEES)

(56)

The error in estimating the current pose is computed as:

$$
\pmb {\epsilon} _ {k} \doteq \left[ \mathrm{Log} (\hat {\mathsf {R}} _ {k} ^ {\mathsf {T}} \mathsf {R} _ {k} ^ {\mathrm{gt}}), \hat {\mathsf {R}} _ {k} ^ {\mathsf {T}} (\hat {\mathbf {p}} _ {k} - \mathbf {p} _ {k} ^ {\mathrm{gt}}) \right] ^ {\mathsf {T}}\tag{57}
$$

where the exponent $ { \mathrm { ^ 6 \mathrm { \ : } g t } } ^ { \prime \prime }$ denotes ground-truth states and $( \hat { \textmd R } _ { k } , \hat { \textmd p } _ { k } )$ denotes the estimated pose at time k. Note that the error (57) is expressed in the body frame and it is consistent with our choice of the retraction in Eq. (21) (intuitively, the retraction applies the perturbation in the body frame).

![](images/88946891be0ab84fcacf83a6c9932482b5e21ebe01e86c44620604c4d41543b1.jpg)  
Fig. 7: Root Mean Squared Error (RMSE) averaged over 50 Monte Carlo experiments, comparing batch nonlinear optimization and iSAM2.

![](images/1fe8f4fab0a34af3bace393f3aad641e351584463116b95ae4e28b73a2453374.jpg)  
Fig. 8: Orientation and position errors with 3σ bounds (single simulation).

The average NEES over N independent Monte Carlo runs, can be computed by averaging the NEES values:

$$
\bar {\eta} _ {k} = \frac {1}{N} \sum_ {i = 1} ^ {N} \eta_ {k} ^ {(i)} \quad (\text { average   NEES })\tag{58}
$$

where $\eta _ { k } ^ { \left( i \right) }$ is the NEES computed at the i-th Monte Carlo run. If the estimator is consistent, then $N \bar { \eta } _ { k }$ is $\chi _ { n } ^ { 2 }$ chi-square distributed with $n = \dim ( \epsilon _ { k } )$ · N degrees of freedom [63, pp. 234]. We evaluate this hypothesis with a $\chi _ { n } ^ { 2 }$ acceptance test [63, pp. 235]. For a significance level $\alpha = 2 . 5 \%$ and $n = \dim ( \epsilon _ { k } ) \cdot N = 6 \cdot 5 0$ , the acceptance region of the test is given by the two-sided probability concentration region $\bar { \eta } _ { k } \in [ 5 . 0 , 7 . 0 ]$ . If $\bar { \eta } _ { k }$ rises significantly higher than the upper bound, the estimator is overconfident, if it tends below the lower bound, it is conservative. In VIO one usually wants to avoid overconfident estimators: the fact that $\bar { \eta } _ { k }$ exceeds the upper bound is an indicator of the fact that the estimator is including spurious information in the inference process.

In Fig. 9 we report the average NEES of the proposed approach. The average NEES approaches the lower bound but, more importantly, it remains below the upper bound at 7.0 (black dots), which assures that the estimator is not overconfident. We also report the average rotational and translational NEES to allow a comparison with the observabilityconstrained EKF in [16, 18], which obtains similar results by enforcing explicitly the observability properties in EKF.

3) Bias Estimation Accuracy: Our simulations allow us to compare the estimated gyroscope and accelerometer bias with the true biases that were used to corrupt the simulated inertial measurements. Fig. 10 shows that biases estimated by our approach (in blue) correctly track the ground truth biases (in red). Note that, since we use a smoothing approach, at each step, we potentially change the entire history of the bias estimates, hence we visualize the bias estimates using multiple curves. Each curve represents the history of the estimated biases from time zero (left-most extreme of the blue curve) to the current time (right-most extreme of the blue curve).

4) First-Order Bias Correction: We performed an additional Monte-Carlo analysis to evaluate the a-posteriori bias correction proposed in Section VI-C. The preintegrated measurements are computed with the bias estimate at the time of integration. However, as seen in Fig. 10, the bias estimate for an older preintegrated measurement may change when more information becomes available. To avoid repeating the integration when the bias estimate changes, we perform a firstorder correction of the preintegrated measurement according to Eq. (44). The accuracy of this first order bias correction is reported in Fig. 11. To compute the statistics, we integrated 100 random IMU measurements with a given bias <sub>estimate b</sub>¯ <sub>which results in the preintegrated measurements</sub> $\Delta \tilde { \mathrm { R } } _ { i j } ( \bar { \mathbf { b } } _ { i } ) , \Delta \tilde { \mathbf { v } } _ { i j } ( \bar { \mathbf { b } } _ { i } )$ and $\Delta \tilde { \bf p } _ { i j } ( \bar { \bf b } _ { i } )$ . Subsequently, a random perturbation δb<sub>i</sub> with magnitude between 0.04 and 0.2 was applied to both the gyroscope and accelerometer bias. We repeated the integration at $\bar { \mathbf { b } } _ { i } + \delta \mathbf { b } _ { i }$ to obtain $\Delta \tilde { \mathrm { R } } _ { i j } ( \bar { \mathbf { b } } _ { i } + $ $\delta \mathbf { b } _ { i } ) , \Delta \tilde { \mathbf { v } } _ { i j } ( \bar { \mathbf { b } } _ { i } + \delta \mathbf { b } _ { i } )$ and $\Delta \tilde { \bf p } _ { i j } ( \bar { \bf b } _ { i } + \delta { \bf b } _ { i } )$ . This ground-truth result was then compared against the first-order correction in (44) to compute the error of the approximation. The errors resulting from the first-order approximation are negligible, even for relatively large bias perturbations.

5) Advantages over the Euler-angle-based formulation: In this section we compare the proposed IMU preintegration with the original formulation of [2], based on Euler angles. We observe three main problems with the preintegration using Euler angles, which are avoided in our formulation.

The first drawback is that, in contrast to the integration using the exponential map in Eq. (30), the rotation integration based on Euler angles is only exact up to the first order. For the interested reader, we recall the rotation rate integration using Euler angles in Appendix IX-E. On the left of Fig. 12, we report the integration errors commited by the Euler angle parametrization when integrating angular rates with randomly selected rotation axes and magnitude in the range from 1 to 3 rad/s. Integration error in Euler angles accumulates quickly when the sampling time $\Delta t$ or the angular rate ω˜ are large. On the other hand, the proposed approach, which performs integration directly on the rotation manifold, is exact, regardless the values of $\Delta t$ and $\tilde { \omega }$

![](images/08d3cb2873fac9674ea9f09a8ce3a357dd7901118319fc95fb6a4f61f567509a.jpg)  
Fig. 9: Normalized Estimation Error Squared (NEES) averaged over 50 Monte Carlo runs. The average NEES is reported for the current pose (top), current position (middle), and current rotation (bottom).

![](images/edefb4cddfe57f034a9fbc96147ae1563275a2df1c41987570994cf6cdc2b755.jpg)

Fig. 10: Comparison between ground truth bias (red line) and estimated bias (blue lines) in a Monte Carlo run.  
![](images/6f40c3aa9718832585748fbdb1a54dabd1e7ec38de932bade35d8b5255d3ac05.jpg)

![](images/c3aa7b24573d191311e06b8277e17f08c667d0ce79b34e5df9acf042a00dbf0a.jpg)

![](images/6d8d5da2bf9262420432cd49e1e82e10806c597c7eee99fb407d6eb7058b4791.jpg)  
Fig. 11: Error committed when using the first-order approximation (44) instead of repeating the integration, for different bias perturbations. Left: $\Delta \tilde { \bf p } _ { i j } ( \bar { \bf b } _ { i } +$ $\delta \mathbf { b } _ { i } )$ error; Center: $\Delta \tilde { \mathbf { v } } _ { i j } ( \bar { \mathbf { b } } _ { i } + \delta \mathbf { b } _ { i } )$ error; Right: $\Delta \tilde { \mathbf { R } } _ { i j } ( \bar { \mathbf { b } } _ { i } + \delta \mathbf { b } _ { i } )$ error. Statistics are computed over 1000 Monte Carlo runs.

The second drawback is that the Euler parametrization is not fair [64], which means that, given the preintegrated Euler angles $\tilde { \pmb { \theta } } ,$ the negative log-likelihood $\begin{array} { r } { \mathcal { L } ( \pmb { \theta } ) \overset { \mathrm { ~ \tiny ~ { ~ 1 ~ } ~ } } { = } \frac { 1 } { 2 } \Vert \tilde { \pmb { \theta } } - \pmb { \theta } \Vert _ { \Sigma } ^ { 2 } } \end{array}$ is not invariant under the action of rigid body transformations. On the right of Fig. 12 we show experimentally how the loglikelihood changes when the frame of reference is rotated around randomly selected rotation axes. This essentially means that an estimator using Euler angles may give different results for different choices of the world frame (cf. with Fig. 2). On the other hand, the SO(3) parametrization can be easily seen to be fair (the negative likelihood (16) can be promptly seen to be left invariant), and this is confirmed by Fig. 12 (right).

The third drawback is the existence of so-called gimball lock singularities. For a $z y x$ Euler angle parametrization, the singularity is reached at pitch values of $\theta \ : = \ : \frac { \pi } { 2 } \ : + n \pi$ for $n ~ \in ~ \mathbb { Z }$ . To evaluate the effect of the singularity and how it affects the computation of preintegrated measurement noise, we performed the following Monte Carlo analysis. We simulated a set of trajectories that reach maximum pitch values $\theta _ { \mathrm { m a x } }$ of increasing magnitude. For each trajectory, we integrate the rotation uncertainty using the Euler parametrization and the proposed on-manifold approach. The ground-truth covariance is instead obtained through sampling. We use the Kullback-Leibler (KL) divergence to quantify the mismatch between the estimated covariances and the ground-truth one. The results of this experiment are shown in Fig. 13, where we observe that the closer we get to the singularity, the worse is the noise propagation using Euler angles. On the other hand, the proposed approach can accurately estimate the measurement covariance, independently on the motion of the platform.

## B. Real Experiments

We integrated the proposed inertial factors in a monocular VIO pipeline to benchmark its performance against the state of the art. In the following, we first discuss our implementation, and then present results from an indoor experiment with motion-capture ground-truth. Finally, we show results from longer trajectories in outdoor experiments. The results confirm that our approach is more accurate than state-of-theart filtering and fixed-lag smoothing algorithms, and enables fast inference in real-world problems.

1) Implementation: Our implementation consists of a high frame rate tracking front-end based on $\mathrm { { S V O ^ { 3 } } }$ [65] and an optimization back-end based on iSAM2 $[ 3 ] ^ { 4 }$ . The front-end tracks salient features in the image at camera rate while the back-end optimizes in parallel the state of selected keyframes as described in this paper.

![](images/591a5f7bb30e639866bd748d7075ca029f9553e1af8c418ec0f1c3d0d3bcc59e.jpg)  
(a)

![](images/8397854b43bd39253e2840558f16db164fac27be963b85ac73e17f4cca045a69.jpg)  
(b)

Fig. 12: (a) Integration errors committed with the Euler angle parametrization for angular velocities ω of increasing magnitude [rad/s]. (b) Negative loglikelihood of a rotation measurement under the action of random rigid body transformations.  
![](images/6a13768aa0ef27e6ed0943d42a4003166e8c4bcf8af59b6e3e188f7f094305a1.jpg)  
Fig. 13: Kullback-Leibler divergence between the preintegrated rotation covariance –computed using Euler angles (red) and the proposed approach (blue)– and the ground-truth covariance. The Euler angle parametrization degrades close to the singularity at $\theta _ { \mathrm { { m a x } } } = 9 0$ deg while the proposed onmanifold approach is accurate regardless of the motion.

SVO [65] is a precise and robust monocular visual odometry system that employs sparse image alignment to estimate incremental motion and tracks features by minimizing the photometric error between subsequent frames. The difference to tracking features individually, as in standard Lucas-Kanade tracking, is that we exploit the known depth of features from previous triangulations. This allows us to track all features as a bundle in a single optimization that satisfies epipolar constraints; hence, outliers only originate from erroneous triangulations. In the visual-inertial setting, we further exploit the availability of accurate rotation increments, obtained by integrating angular velocity measurements from the gyroscope. These increments are used as rotation priors in the sparseimage-alignment algorithm, and this increases the overall robustness of the system. The motion estimation is combined with an outlier resistant probabilistic triangulation method that is implemented with a recursive Bayesian filter. The high frame-rate motion estimation combined with the robust depth estimation results in increased robustness in scenes with repetitive and high frequency texture (e.g., asphalt). The output of SVO are selected keyframes with feature-tracks corresponding to triangulated landmarks. This data is passed to the back-end that computes the visual-inertial MAP estimate in Eq. (26) using iSAM2 [3].

We remark that our approach does not marginalize out past states. Therefore, while the approach is designed for fast visual-inertial odometry, if desired, it could be readily extended to incorporate loop closures.

2) Indoor Experiments: The indoor experiment shows that the proposed approach is more accurate than two competitive state-of-the-art approaches, namely ${ \mathrm { O K V I S } } ^ { 5 }$ [24], and MSCKF [5]. The experiment is performed on the 430mlong indoor trajectory of Fig. 14. The dataset was recorded with a forward-looking VI-Sensor [66] that consists of an ADIS16448 MEMS IMU and two embedded WVGA monochrome cameras (we only use the left camera). Intrinsic and extrinsic calibration was obtained using [59]. The camera runs at 20Hz and the IMU at 800Hz. Ground truth poses are provided by a Vicon system mounted in the room; the handeye calibration between the Vicon markers and the camera is computed using a least-squares method [67].

![](images/4d9fc690678984597628b0d1b21ff3daafa396f057b494bd2304573bdf3b2684.jpg)

![](images/e98b4369c9e598138ab1c796cd7faba8fb3a12060244986b318324ebdea71ab1.jpg)

![](images/15e910dc2e75d07b0b2b73460223578096e5a5326a8aed23abfa0bd40f8b58d0.jpg)  
Fig. 14: Left: two images from the indoor trajectory dataset with tracked features in green. Right: top view of the trajectory estimate produced by our approach (blue) and 3D landmarks triangulated from the trajectory (green).

![](images/01b8dad7de6cec3bf1dda658b394ca2c3d3fe60f474ea4dec20f63148855371a.jpg)

![](images/6f9bd2064913ba0c47a8bddf20faab0ad97e30b77c264752e34d0c737738649e.jpg)

![](images/d2e4130a17d30aa8279174712366c94cdbd2cc4840f8f38fbc0cde6180ebafb1.jpg)  
Fig. 15: Comparison of the proposed approach versus the OKVIS algorithm [24] and an implementation of the MSCKF filter [5]. Relative errors are measured over different segments of the trajectory, of length {10, 40, 90, 160, 250, 360}m, according to the odometric error metric in [68].

![](images/473139be2f260ebc098b463d2ebfcfdcab033bada1623eb6313b0165a427d33b.jpg)  
Fig. 16: Processing-time per keyframe for the proposed VIO approach.

Fig. 15 compares the proposed system against the OKVIS algorithm [24], and an implementation of the MSCKF filter [5]. Both these algorithms currently represent the stateof-the-art in VIO, OKVIS for optimization-based approaches, and MSCKF for filtering methods. We obtained the datasets as

y [m]

![](images/bc949d1871b5834275db696460ceffaa335722509bcd1de9d17d203cfe5ebbd8.jpg)  
Fig. 17: Orientation and position errors with 3σ bounds for the real indoor experiment in Fig. 14.

well as the trajectories computed with OKVIS and MSCKF from the authors of [24]. We use the relative error metrics proposed in [68] to obtain error statistics. The metric evaluates the relative error by averaging the drift over trajectory segments of different length ({10, 40, 90, 160, 250, 360}m in Fig. 15). Our approach exhibits less drift than the state-ofthe-art, achieving 0.3m drift on average over 360m traveled distance; OKVIS and MSCKF accumulate an average error of 0.7m. We observe significantly less drift in yaw direction in the proposed approach while the error in pitch and and roll direction is constant for all methods due to the observability of the gravity direction.

We highlight that these three algorithms use different frontend feature tracking systems, which influence the overall performance of the approach. Therefore, while in Section VIII-A we discussed only aspects related to the preintegration theory, in this section we evaluate the proposed system as a whole (SVO, preintegration, structureless vision factors, iSAM2).

Evaluating consistency in real experiments by means of analysing the average NEES is difficult as one would have to evalate and average the results of multiple runs of the same trajectory with different realizations of sensor noise. In Figure 17 we show the error plots with the 3-sigma bounds for a single run. The result is consistent as the estimation errors remain within the bounds of the estimated uncertainty. In this experiment, we aligned only the first frame of the trajectory with the vicon trajectory. Therefore, analyzing the drift over

![](images/ae18aac8fa83e14b83f187a2a51d59a8460dbf36c7c4529b3448d2f95c9f9d98.jpg)  
Fig. 18: Outdoor trajectory (length: 300m) around a building with identical start and end point at coordinates (0, 0, 0). The end-to-end error of the proposed approach is 1.0m. Google Tango accumulated 2.2m drift. The green dots are the 3D points triangulated from our trajectory estimate.

![](images/2309496a9854d10f78b3c4b084f46085acc04691d212a6a8e694f978ac35441b.jpg)

![](images/8ed69a9e1051e776f76589e3961765fb44d0a6fc77adc3b3704da14db6855f15.jpg)  
Fig. 19: Real test comparing the proposed VIO approach against Google Tango. The 160m-long trajectory starts at (0, 0, 0) (ground floor), goes up till the 3rd floor of a building, and returns to the initial point. The figure shows a side view (left) and a top view (right) of the trajectory estimates for our approach (blue) and Tango (red). Google Tango accumulates 1.4m error, while the proposed approach only has 0.5m drift. 3D points triangulated from our trajectory estimate are shown in green for visualization purposes.

400 meters is very prone to errors in the initial pose from the ground-truth or errors in the hand-eye calibration of the system.

Figure 16 illustrates the time required by the back-end to compute the full MAP estimate, by running iSAM2 with 10 optimization iterations. The experiment was performed on a standard laptop (Intel i7, 2.4 GHz). The average update time for iSAM2 is 10ms. The peak corresponds to the start of the experiment in which the camera was not moving. In this case the number of tracked features becomes very large making the back-end slightly slower. The SVO front-end requires approximately 3ms to process a frame on the laptop while the back-end runs in a parallel thread and optimizes only keyframes. Although the processing times of OKVIS were not reported, the approach is described as computationally demanding [24]. OKVIS needs to repeat IMU integration at every change of the linearization point, which we avoid by using the preintegrated IMU measurements.

3) Outdoor Experiments: The second experiment is performed on an outdoor trajectory, and compares the proposed approach against the Google Tango Peanut sensor (mapper version 3.15), which is an engineered VIO system. We rigidly attached the VI-Sensor to a Tango device and walked around an office building. Fig. 18 depicts the trajectory estimates for our approach and Google Tango. The trajectory starts and ends at the same location, hence we can report the end-to-end error which is 1.5m for the proposed approach and 2.2m for the Google Tango sensor.

In Fig. 18 we also show the estimated landmark positions (in green). 3D points are not estimated by our approach (which uses a structureless vision model), but are triangulated from our trajectory estimate for visualization purposes.

The third experiment is the one in Fig. 19. The trajectory goes across three floors of an office building and eventually returns to the initial location on the ground floor. Also in this case the proposed approach guarantees a very small end-to-end error (0.5m), while Tango accumulates 1.4m error.

We remark that Tango and our system use different sensors, hence the reported end-to-end errors only allow for a qualitative comparison. However, the IMUs of both sensors exhibit similar noise characteristics [69, 70] and the Tango camera has a significantly larger field-of-view and better shutter speed control than our sensor. Therefore, the comparison is still valuable to assess the accuracy of the proposed approach.

A video demonstrating the execution of our approach for the real experiments discussed in this section can be viewed at https://youtu.be/CsJkci5lfco

## IX. CONCLUSION

This paper proposes a novel preintegration theory, which provides a grounded way to model a large number of IMU measurements as a single motion constraint. Our proposal improves over related works that perform integration in a global frame, e.g., [5, 23], as we do not commit to a linearization point during integration. Moreover, it leverages the seminal work on preintegration [2], bringing to maturity the preintegration and uncertainty propagation in SO(3).

As a second contribution, we discuss how to use the preintegrated IMU model in a VIO pipeline; we adopt a structureless model for visual measurements which avoids optimizing over 3D landmarks. Our VIO approach uses iSAM2 to perform constant-time incremental smoothing.

An efficient implementation of our approach requires 10ms to perform inference (back-end), and 3ms for feature tracking (front-end). Experimental results also confirm that our approach is more accurate than state-of-the-art alternatives, including filtering and optimization-based techniques.

We release the source-code of the IMU preintegration and the structurless vision factors in the GTSAM 4.0 optimization toolbox [7] and provide additional theoretical derivations and implementation details in the Appendix of this paper.

Acknowledgments The authors gratefully acknowledge Stephen Williams and Richard Roberts for helping with an early implementation in GTSAM, and Simon Lynen and Stefan Leutenegger for providing datasets and results of their algorithms. Furthermore, we are thankful to Mingyu Chen for the detailed comments and suggestions on the previous version of this paper.

## APPENDIX

## A. Iterative Noise Propagation

In this section we show that the computation of the preintegrated noise covariance, discussed in Section VI-B, can be carried out in iterative form, which leads to simpler expressions and is more amenable for online inference.

Let us start from the preintegrated rotation noise in (42). To write $\delta \phi _ { i j }$ in iterative form, we simply take the last term $( k = j - 1 )$ ) out of the sum and rearrange the terms:

$$
\begin{array}{l} \delta \phi_ {i j} \simeq \sum_ {k = i} ^ {j - 1} \Delta \tilde {\mathbf {R}} _ {k + 1 j} ^ {\mathsf {T}} \mathbf {J} _ {r} ^ {k} \boldsymbol {\eta} _ {k} ^ {g d} \Delta t \\ = \sum_ {k = i} ^ {j - 2} \Delta \tilde {\mathbf {R}} _ {k + 1 j} ^ {\mathsf {T}} \mathbf {J} _ {r} ^ {k} \boldsymbol {\eta} _ {k} ^ {g d} \Delta t + \overbrace {\Delta \tilde {\mathbf {R}} _ {j j} ^ {\mathsf {T}}} ^ {= \mathbf {I} _ {3 \times 3}} \mathbf {J} _ {r} ^ {j - 1} \boldsymbol {\eta} _ {j - 1} ^ {g d} \Delta t \\ = \sum_ {k = i} ^ {j - 2} (\overbrace {\Delta \tilde {\mathbf {R}} _ {k + 1 j - 1} \Delta \tilde {\mathbf {R}} _ {j - 1 j}} ^ {= \Delta \tilde {\mathbf {R}} _ {k + 1 j}}) ^ {\mathsf {T}} \mathbf {J} _ {r} ^ {k} \boldsymbol {\eta} _ {k} ^ {g d} \Delta t + \mathbf {J} _ {r} ^ {j - 1} \boldsymbol {\eta} _ {j - 1} ^ {g d} \Delta t \\ = \Delta \tilde {\mathbf {R}} _ {j - 1 j} ^ {\mathsf {T}} \sum_ {k = i} ^ {j - 2} \Delta \tilde {\mathbf {R}} _ {k + 1 j - 1} ^ {\mathsf {T}} \mathbf {J} _ {r} ^ {k} \boldsymbol {\eta} _ {k} ^ {g d} \Delta t + \mathbf {J} _ {r} ^ {j - 1} \boldsymbol {\eta} _ {j - 1} ^ {g d} \Delta t \\ = \Delta \tilde {\mathbf {R}} _ {j - 1 j} \delta \phi_ {i j - 1} + \mathbf {J} _ {r} ^ {j - 1} \boldsymbol {\eta} _ {j - 1} ^ {g d} \Delta t. \end{array}\tag{59}
$$

Repeating the same process for $\delta \mathbf { v } _ { i j }$ in (43):

$$
\begin{array}{l} \delta \mathbf {v} _ {i j} = \sum_ {k = i} ^ {j - 1} \left[ - \Delta \tilde {\mathrm{R}} _ {i k} \left(\tilde {\mathbf {a}} _ {k} - \mathbf {b} _ {i} ^ {a}\right) ^ {\wedge} \delta \phi_ {i k} \Delta t + \Delta \tilde {\mathrm{R}} _ {i k} \boldsymbol {\eta} _ {k} ^ {a d} \Delta t \right] \\ = \sum_ {k = i} ^ {j - 2} \left[ - \Delta \tilde {\mathrm{R}} _ {i k} \left(\tilde {\mathbf {a}} _ {k} - \mathbf {b} _ {i} ^ {a}\right) ^ {\wedge} \delta \phi_ {i k} \Delta t + \Delta \tilde {\mathrm{R}} _ {i k} \boldsymbol {\eta} _ {k} ^ {a d}, \Delta t \right] \\ - \Delta \tilde {\mathrm{R}} _ {i j - 1} \left(\tilde {\mathbf {a}} _ {j - 1} - \mathbf {b} _ {i} ^ {a}\right) ^ {\wedge} \delta \phi_ {i j - 1} \Delta t + \Delta \tilde {\mathrm{R}} _ {i j - 1} \boldsymbol {\eta} _ {j - 1} ^ {a d}, \Delta t \\ = \delta \mathbf {v} _ {i j - 1} - \Delta \tilde {\mathrm{R}} _ {i j - 1} \left(\tilde {\mathbf {a}} _ {j - 1} - \mathbf {b} _ {i} ^ {a}\right) ^ {\wedge} \delta \phi_ {i j - 1} \Delta t + \Delta \tilde {\mathrm{R}} _ {i j - 1} \boldsymbol {\eta} ^ {a d}, \Delta t. \end{array} \tag {60}
$$

Doing the same for $\delta \mathbf { p } _ { i j }$ in (43), and noting that $\delta \mathbf { p } _ { i j }$ can be written as a function of $\delta { \bf v } _ { i j } \ : ( c f .$ with the expression of $\delta \mathbf { v } _ { i j }$ in (43)):

$$
\begin{array}{l} \delta \mathbf {p} _ {i j} = \sum_ {k = i} ^ {j - 1} \left[ \delta \mathbf {v} _ {i k} \Delta t - \frac {1}{2} \Delta \tilde {\mathrm{R}} _ {i k} \left(\tilde {\mathbf {a}} _ {k} - \mathbf {b} _ {i} ^ {a}\right) ^ {\wedge} \delta \phi_ {i k} \Delta t ^ {2} + \frac {1}{2} \Delta \tilde {\mathrm{R}} _ {i k} \boldsymbol {\eta} _ {k} ^ {a d} \Delta t ^ {2} \right] \\ = \sum_ {k = i} ^ {j - 2} \left[ \delta \mathbf {v} _ {i k} \Delta t - \frac {1}{2} \Delta \tilde {\mathrm{R}} _ {i k} \left(\tilde {\mathbf {a}} _ {k} - \mathbf {b} _ {i} ^ {a}\right) ^ {\wedge} \delta \phi_ {i k} \Delta t ^ {2} + \frac 12 \Delta \tilde {\mathrm{R}} _ {i k} \boldsymbol {\eta} _ {k} ^ {a d} \Delta t ^ {2} \right] \\ + \delta \mathbf {v} _ {i j - 1} \Delta t - \frac 12 \Delta \tilde {\mathrm{R}} _ {i j - 1} (\tilde {\mathbf {a}} _ {j - 1} - \mathbf {b} _ {i} ^ {a}) ^ {\wedge} \delta \phi_ {i j - 1} \Delta t ^ {2} + \frac 12 \Delta \tilde {\mathrm{R}} _ {i j - 1} \boldsymbol {\eta} _ {j - 1} ^ {a d} \Delta t ^ {2} \\ = \delta \mathbf {p} _ {i j - 1} + \delta \mathbf {v} _ {i j - 1} \Delta t - \frac 12 \Delta \tilde {\mathrm{R}} _ {i j - 1} (\tilde {\mathbf {a}} _ {j - 1} - \mathbf {b} _ {i} ^ {a}) ^ {\wedge} \delta \phi_ {i j - 1} \Delta t ^ {2} \\ + \frac 12 \Delta \tilde {\mathrm{R}} _ {i j - 1} \boldsymbol {\eta} _ {j - 1} ^ {a d} \Delta t ^ {2} \end{array} \tag {61}
$$

Recalling that $\pmb { \eta } _ { i k } ^ { \Delta } \ \doteq \ [ \delta \phi _ { i k } , \delta \mathbf { v } _ { i k } , \delta \mathbf { p } _ { i k } ]$ , and defining the IMU measurement noise $\pmb { \eta } _ { k } ^ { d } \doteq \mathsf { \bar { \Pi } } [ \pmb { \eta } _ { k } ^ { g \dot { d } } \pmb { \eta } _ { k } ^ { a d } ] , ^ { 6 }$ we can finally write Eqs. (59)- (61) in compact matrix form as:

$$
\pmb {\eta} _ {i j} ^ {\Delta} = \mathbf {A} _ {j - 1} \pmb {\eta} _ {i j - 1} ^ {\Delta} + \mathbf {B} _ {j - 1} \pmb {\eta} _ {j - 1} ^ {d},\tag{62}
$$

From the linear model (62) and given the covariance $\Sigma _ { \eta } \in \mathbb { R } ^ { 6 \times 6 }$ of the raw IMU measurements noise $\eta _ { k } ^ { d } .$ , it is now possible to compute the preintegrated measurement covariance iteratively:

$$
\boldsymbol {\Sigma} _ {i j} = \mathbf {A} _ {j - 1} \boldsymbol {\Sigma} _ {i j - 1} \mathbf {A} _ {j - 1} ^ {\top} + \mathbf {B} _ {j - 1} \boldsymbol {\Sigma} _ {\eta} \mathbf {B} _ {j - 1} ^ {\top}\tag{63}
$$

starting from initial conditions $\pmb { \Sigma } _ { i i } = \mathbf { 0 } _ { 9 \times 9 }$

<sup>6</sup>Both $\eta _ { i j } ^ { \Delta }$ and $\eta _ { k } ^ { d }$ are column vectors: we omit the transpose in the definition to keep notation simple.

## B. Bias Correction via First-Order Updates

In this section we provide a complete derivation of the first-order bias correction proposed in Section VI-C.

Let us assume that we have computed the preintegrated variables at a given bias estimate $\bar { { \bf b } } _ { i } \ \doteq \ [ \bar { { \bf b } } _ { i } ^ { g } \mathrm { ~ \bf ~ \bar { ~ b } ^ { a } _ { i } ~ } ]$ , and let us denote the corresponding preintegrated measurements as

$$
\Delta \bar {\mathbf {R}} _ {i j} \doteq \Delta \tilde {\mathbf {R}} _ {i j} (\bar {\mathbf {b}} _ {i}), \Delta \bar {\mathbf {v}} _ {i j} \doteq \Delta \tilde {\mathbf {v}} _ {i j} (\bar {\mathbf {b}} _ {i}), \Delta \bar {\mathbf {p}} _ {i j} \doteq \Delta \tilde {\mathbf {p}} _ {i j} (\bar {\mathbf {b}} _ {i}).\tag{64}
$$

In this section we want to devise an expression to “update” $\Delta \bar { \mathsf { R } } _ { i j }$ $\Delta \bar { \mathbf { v } } _ { i j } , \Delta \bar { \mathbf { p } } _ { i j }$ when our bias estimate changes.

Consider the case in which we get a new estimate $\hat { \mathbf { b } } _ { i } \gets \bar { \mathbf { b } } _ { i } + \delta \mathbf { b } _ { i }$ where $\delta \mathbf { { b } } _ { i }$ is a small correction w.r.t. the previous estimate $\mathbf { b } _ { i }$

We start with the bias correction for the preintegrated rotation measurement. The key idea here is to write $\Delta \tilde { \mathrm { R } } _ { i j } ( \hat { \mathbf { b } } _ { i } )$ (the preintegrated measurement at the new bias estimate) as a function of $\Delta \bar { \mathsf { R } } _ { i j }$ (the preintegrated measurement at the old bias estimate), $\mathrm { \ddot { \ p l u s } } ^ { \mathrm { \prime \prime } }$ a first-order correction. Recalling Eq. (35), we write $\Delta \tilde { \mathrm { R } } _ { i j } ( \hat { \mathbf { b } } _ { i } )$ as:

$$
\Delta \tilde {\mathbf {R}} _ {i j} (\hat {\mathbf {b}} _ {i}) = \prod_ {k = i} ^ {j - 1} \operatorname{Exp} \left(\left(\tilde {\boldsymbol {\omega}} _ {k} - \hat {\mathbf {b}} _ {i} ^ {g}\right) \Delta t\right)\tag{65}
$$

Substituting $\hat { \mathbf { b } } _ { i } = \bar { \mathbf { b } } _ { i } + \delta \mathbf { b } _ { i }$ in the previous expression and using the first-order approximation (4) in each factor (we assumed small $\delta \mathbf { b } _ { i } ) \colon$

$$
\begin{array}{l} \Delta \tilde {\mathbf {R}} _ {i j} (\hat {\mathbf {b}} _ {i}) = \prod_ {k = i} ^ {j - 1} \mathrm{Exp} \left(\left(\tilde {\boldsymbol {\omega}} _ {k} - (\bar {\mathbf {b}} _ {i} ^ {g} + \delta \mathbf {b} _ {i} ^ {g})\right) \Delta t\right) \\ \simeq \prod_ {k = i} ^ {j - 1} \mathrm{Exp} \left(\left(\tilde {\boldsymbol {\omega}} _ {k} - \bar {\mathbf {b}} _ {i} ^ {g}\right) \Delta t\right) \mathrm{Exp} \left(- \mathsf {J} _ {r} ^ {k}   \delta \mathbf {b} _ {i} ^ {g}   \Delta t\right). \end{array}\tag{66}
$$

Now, we rearrange the terms in the product, by “moving” the terms including $\delta \mathbf { b } _ { i } ^ { g d }$ to the end, using the relation (11):

$$
\Delta \tilde {\mathbf {R}} _ {i j} (\hat {\mathbf {b}} _ {i}) = \Delta \bar {\mathbf {R}} _ {i j} \prod_ {k = i} ^ {j - 1} \mathrm{Exp} \left(- \Delta \tilde {\mathbf {R}} _ {k + 1 j} (\bar {\mathbf {b}} _ {i}) ^ {\top} \mathsf {J} _ {r} ^ {k}   \delta \mathbf {b} _ {i} ^ {g}   \Delta t\right),\tag{67}
$$

where we used the fact that by definition it holds that $\begin{array} { r l } { \Delta \bar { \mathsf { R } } _ { i j } } & { { } = } \end{array}$ $\begin{array} { r l } {  { \prod _ { k = i } ^ { j - 1 } \mathrm { E x p } ( ( \tilde { \omega } _ { k } - \bar { \mathbf { b } } _ { i } ^ { g } ) \Delta t ) } } & { { } } \end{array}$ . Repeated application of the first-order approximation (7) (recall that $\delta \mathbf { { b } } _ { i } ^ { \hat { g } }$ is small, hence the right Jacobians are close to the identity) produces:

$$
\begin{array}{l} \Delta \tilde {\mathbf {R}} _ {i j} (\hat {\mathbf {b}} _ {i}) \simeq \Delta \bar {\mathbf {R}} _ {i j} \mathrm{Exp} \left(\sum_ {k = i} ^ {j - 1} - \Delta \tilde {\mathbf {R}} _ {k + 1 j} (\bar {\mathbf {b}} _ {i}) ^ {\mathsf {T}} \mathbf {J} _ {r} ^ {k} \delta \mathbf {b} _ {i} ^ {g} \Delta t\right) \\ = \Delta \bar {\mathbf {R}} _ {i j} \mathrm{Exp} \Big (\frac {\partial \Delta \bar {\mathbf {R}} _ {i j}}{\partial \mathbf {b} ^ {g}} \delta \mathbf {b} _ {i} ^ {g} \Big) \end{array}\tag{68}
$$

Using (68) we can now update the preintegrated rotation measurement $\Delta \tilde { \mathsf { R } } _ { i j } ( \bar { \mathsf { b } } _ { i } )$ to get $\Delta \tilde { \tt R } _ { i j } ( \hat { \bf { b } } _ { i } )$ without repeating the integration.

Let us now focus on the bias correction of the preintegrated velocity $\Delta \tilde { \mathbf { v } } _ { i j } ( \ddot { \mathbf { b } } _ { i } )$

$$
\Delta \tilde {\mathbf {v}} _ {i j} (\hat {\mathbf {b}} _ {i}) = \sum_ {k = i} ^ {j - 1} \Delta \tilde {\mathsf {R}} _ {i k} (\hat {\mathbf {b}} _ {i}) \left(\tilde {\mathbf {a}} _ {k} - \bar {\mathbf {b}} _ {i} ^ {a} - \delta \mathbf {b} _ {i} ^ {a}\right) \Delta t\tag{69}
$$

$$
\stackrel {(6 8)} {\simeq} \sum_ {k = i} ^ {j - 1} \Delta \bar {\mathbf {R}} _ {i k} \mathrm{Exp} \left(\frac {\partial \Delta \bar {\mathbf {R}} _ {i k}}{\partial \mathbf {b} ^ {g}} \delta \mathbf {b} _ {i} ^ {g}\right) \left(\tilde {\mathbf {a}} _ {k} - \bar {\mathbf {b}} _ {i} ^ {a} - \delta \mathbf {b} _ {i} ^ {a}\right) \Delta t
$$

$$
\stackrel {(4)} {\simeq} \sum_ {k = i} ^ {j - 1} \Delta \bar {\mathbf {R}} _ {i k} \left(\mathbf {I} + \left(\frac {\partial \Delta \bar {\mathbf {R}} _ {i k}}{\partial \mathbf {b} ^ {g}} \delta \mathbf {b} _ {i} ^ {g}\right) ^ {\wedge}\right) \left(\tilde {\mathbf {a}} _ {k} - \bar {\mathbf {b}} _ {i} ^ {a} - \delta \mathbf {b} _ {i} ^ {a}\right) \Delta t
$$

$$
\stackrel {(a)} {\simeq} \Delta \bar {\mathbf {v}} _ {i j} - \sum_ {k = i} ^ {j - 1} \Delta \bar {\mathbf {R}} _ {i k} \Delta t \delta \mathbf {b} _ {i} ^ {a} + \sum_ {k = i} ^ {j - 1} \Delta \bar {\mathbf {R}} _ {i k} \Big (\frac {\partial \Delta \bar {\mathbf {R}} _ {i k}}{\partial \mathbf {b} ^ {g}} \delta \mathbf {b} _ {i} ^ {g} \Big) ^ {\wedge} \left(\tilde {\mathbf {a}} _ {k} - \bar {\mathbf {b}} _ {i} ^ {a}\right)
$$

$$
\begin{array}{l} \stackrel {(2)} {=} \Delta \bar {\mathbf {v}} _ {i j} - \sum_ {k = i} ^ {j - 1} \Delta \bar {\mathbf {R}} _ {i k} \Delta t \delta \mathbf {b} _ {i} ^ {a} - \sum_ {k = i} ^ {j - 1} \Delta \bar {\mathbf {R}} _ {i k} \left(\tilde {\mathbf {a}} _ {k} - \bar {\mathbf {b}} _ {i} ^ {a}\right) ^ {\wedge} \frac {\partial \Delta \bar {\mathbf {R}} _ {i k}}{\partial \mathbf {b} ^ {g}} \Delta t \delta \mathbf {b} _ {i} ^ {g} \\ = \Delta \bar {\mathbf {v}} _ {i j} + \frac {\partial \Delta \bar {\mathbf {v}} _ {i j}}{\partial \mathbf {b} ^ {a}} \delta \mathbf {b} _ {i} ^ {a} + \frac {\partial \Delta \bar {\mathbf {v}} _ {i j}}{\partial \mathbf {b} ^ {g}} \delta \mathbf {b} _ {i} ^ {g} \end{array}
$$

Where for (a), we used $\begin{array} { r } { \Delta \bar { \bf v } _ { i j } = \sum _ { k = i } ^ { j - 1 } \Delta \bar { \bf R } _ { i k } \left( \tilde { \bf a } _ { k } - \bar { \bf b } _ { i } ^ { a } \right) \Delta t } \end{array}$ . Exactly the same derivation can be repeated for $\Delta \tilde { \bf p } _ { i j } ( \hat { \bf b } _ { i } )$ . Summarizing, the Jacobians used for the a posteriori bias update in Eq. (44) are:

$$
\begin{array}{r l r} \frac {\partial \Delta \bar {\mathbf {r}} _ {i j}}{\partial \mathbf {b} ^ {g}} & = & - \sum_ {k = i} ^ {j - 1} \left[ \Delta \tilde {\mathbf {R}} _ {k + 1 j} (\bar {\mathbf {b}} _ {i}) ^ {\top} \right. \mathbf {J} _ {r} ^ {k} \Delta t ] \\ \frac {\partial \Delta \bar {\mathbf {v}} _ {i j}}{\partial \mathbf {b} ^ {a}} & = & - \sum_ {k = i} ^ {j - 1} \Delta \bar {\mathbf {R}} _ {i k} \Delta t \\ \frac {\partial \Delta \bar {\mathbf {v}} _ {i j}}{\partial \mathbf {b} ^ {g}} & = & - \sum_ {k = i} ^ {j - 1} \Delta \bar {\mathbf {R}} _ {i k} \left(\tilde {\mathbf {a}} _ {k} - \bar {\mathbf {b}} _ {i} ^ {a}\right) ^ {\wedge} \frac {\partial \Delta \bar {\mathbf {R}} _ {i k}}{\partial \mathbf {b} ^ {g}} \Delta t \\ \frac {\partial \Delta \bar {\mathbf {p}} _ {i j}}{\partial \mathbf {b} ^ {a}} & = & \sum_ {k = i} ^ {j - 1} \frac {\partial \Delta \bar {\mathbf {v}} _ {i k}}{\partial \mathbf {b} ^ {a}} \Delta t - \frac {1}{2} \Delta \bar {\mathbf {R}} _ {i k} \Delta t ^ {2} \\ \frac {\partial \Delta \bar {\mathbf {p}} _ {i j}}{\partial \mathbf {b} ^ {g}} & = & \sum_ {k = i} ^ {j - 1} \frac {\partial \Delta \bar {\mathbf {v}} _ {i k}}{\partial \mathbf {b} ^ {g}} \Delta t - \frac {1}{2} \Delta \bar {\mathbf {R}} _ {i k} \left(\tilde {\mathbf {a}} _ {k} - \bar {\mathbf {b}} _ {i} ^ {a}\right) ^ {\wedge} \frac {\partial \Delta \bar {\mathbf {R}} _ {i k}}{\partial \mathbf {b} ^ {g}} \Delta t ^ {2} \end{array}
$$

Note that the Jacobians can be computed incrementally, as new measurements arrive.

## C. Jacobians of Residual Errors

In this section we provide analytic expressions for the Jacobian matrices of the residual errors in Eq. (45). These Jacobians are crucial when using iterative optimization techniques $( \mathrm { e . g . }$ , the Gauss-Newton method of Section III-C) to minimize the cost in Eq. (26).

“Lifting” the cost function (see Section III-C) consists in substi tuting the following retractions:

$$
\begin{array}{c c c c c c} \mathsf {R} _ {i} & \leftarrow & \mathsf {R} _ {i}   \mathrm{Exp} (\delta \phi_ {i}), & \mathsf {R} _ {j} & \leftarrow & \mathsf {R} _ {j}   \mathrm{Exp} (\delta \phi_ {j}), \\ \mathbf {p} _ {i} & \leftarrow & \mathbf {p} _ {i} + \mathsf {R} _ {i} \delta \mathbf {p} _ {i}, & \mathbf {p} _ {j} & \leftarrow & \mathbf {p} _ {j} + \mathsf {R} _ {j} \delta \mathbf {p} _ {j}, \\ \mathbf {v} _ {i} & \leftarrow & \mathbf {v} _ {i} + \delta \mathbf {v} _ {i}, & \mathbf {v} _ {j} & \leftarrow & \mathbf {v} _ {j} + \delta \mathbf {v} _ {i}, \\ \delta \mathbf {b} _ {i} ^ {g} & \leftarrow & \delta \mathbf {b} _ {i} ^ {g} + \tilde {\delta} \mathbf {b} _ {i} ^ {g}, & \delta \mathbf {b} _ {i} ^ {a} & \leftarrow & \delta \mathbf {b} _ {i} ^ {a} + \tilde {\delta} \mathbf {b} _ {i} ^ {a}, \end{array}\tag{70}
$$

The process of lifting makes the residual errors a function defined on a vector space, on which it is easy to compute Jacobians. Therefore, in the following sections we derive the Jacobians w.r.t. the vectors $\delta \phi _ { i } , \delta \mathbf { p } _ { i } , \delta \mathbf { v } _ { i } , \bar { \delta \phi } _ { j } , \delta \mathbf { p } _ { j } , \delta \mathbf { v } _ { j } , \tilde { \delta } \mathbf { b } _ { i } ^ { g } , \tilde { \delta } \mathbf { b } _ { i } ^ { a }$

1) Jacobians of $\mathbf { r } _ { \Delta \mathbf { p } _ { i j } } .$ Since $\mathbf { r } _ { \Delta \mathbf { p } _ { i j } }$ is linear in $\delta \mathbf { b } _ { i } ^ { g }$ and $\delta \mathbf { b } _ { i } ^ { a }$ and the retraction is simply a vector sum, the Jacobians of $\mathbf { r } _ { \Delta \mathbf { p } _ { i j } }$ w.r.t. $\tilde { \delta } \mathbf { b } _ { i } ^ { g } , \tilde { \delta } \mathbf { b } _ { i } ^ { a }$ are simply the matrix coefficients of $\delta \mathbf { b } _ { i } ^ { g }$ and $\delta \mathbf { b } _ { i } ^ { a }$ Moreover, $\mathtt { R } _ { j }$ and $\mathbf { v } _ { j }$ do not appear in $\mathbf { r } _ { \Delta \mathbf { p } _ { i j } } .$ , hence the Jacobians w.r.t. $\delta \phi _ { j } , \delta \mathbf { v } _ { j }$ are zero. Let us focus on the remaining Jacobians:

$$
\mathbf {r} _ {\Delta \mathbf {p} _ {i j}} (\mathbf {p} _ {i} + \mathrm{R} _ {i} \delta \mathbf {p} _ {i}) = \mathrm{R} _ {i} ^ {\top} \left(\mathbf {p} _ {j} - \mathbf {p} _ {i} - \mathrm{R} _ {i} \delta \mathbf {p} _ {i} - \mathbf {v} _ {i} \Delta t _ {i j} - \frac {1}{2} \mathbf {g} \Delta t _ {i j} ^ {2}\right) - C\tag{71}
$$

$$
\mathbf {r} _ {\Delta \mathbf {p} _ {i j}} (\mathbf {p} _ {j} + \mathrm{R} _ {j} \delta \mathbf {p} _ {j}) = \mathrm{R} _ {i} ^ {\top} \left(\mathbf {p} _ {j} + \mathrm{R} _ {j} \delta \mathbf {p} _ {j} - \mathbf {p} _ {i} - \mathbf {v} _ {i} \Delta t _ {i j} - \frac {1}{2} \mathbf {g} \Delta t _ {i j} ^ {2}\right) - C\tag{72}
$$

$$
\begin{array}{r l} \mathbf {r} _ {\Delta \mathbf {p} _ {i j}} (\mathbf {v} _ {i} + \delta \mathbf {v} _ {i}) & = \mathbb {R} _ {i} ^ {\top} \left(\mathbf {p} _ {j} - \mathbf {p} _ {i} - \mathbf {v} _ {i} \Delta t _ {i j} - \delta \mathbf {v} _ {i} \Delta t _ {i j} - \frac {1}{2} \mathbf {g} \Delta t _ {i j} ^ {2}\right) - C \\ & = \mathbf {r} _ {\Delta \mathbf {p} _ {i j}} (\mathbf {v} _ {i}) + (- \mathbb {R} _ {i} ^ {\top} \Delta t _ {i j}) \delta \mathbf {v} _ {i} \end{array} \tag {73}
$$

$$
\mathbf {r} _ {\Delta \mathbf {p} _ {i j}} (\mathbb {R} _ {i} \mathrm{Exp} (\delta \phi_ {i}))\tag{74}
$$

$$
= \left(\mathrm{R} _ {i} \operatorname{Exp} \left(\delta \phi_ {i}\right)\right) ^ {\top} \left(\mathbf {p} _ {j} - \mathbf {p} _ {i} - \mathbf {v} _ {i} \Delta t _ {i j} - \frac {1}{2} \mathbf {g} \Delta t _ {i j} ^ {2}\right) - C
$$

$$
\stackrel {(4)} {\simeq} (\mathbf {I} - \delta \phi_ {i} ^ {\wedge}) \mathsf {R} _ {i} ^ {\top} \left(\mathbf {p} _ {j} - \mathbf {p} _ {i} - \mathbf {v} _ {i} \Delta t _ {i j} - \frac {1}{2} \mathbf {g} \Delta t _ {i j} ^ {2}\right) - C
$$

$$
\stackrel {(2)} {=} \mathbf {r} _ {\Delta \mathbf {p} _ {i j}} (\mathsf {R} _ {i}) + \left(\mathsf {R} _ {i} ^ {\top} \left(\mathbf {p} _ {j} - \mathbf {p} _ {i} - \mathbf {v} _ {i} \Delta t _ {i j} - \frac {1}{2} \mathbf {g} \Delta t _ {i j} ^ {2}\right)\right) ^ {\wedge} \delta \phi_ {i}.
$$

Where we used the shorthand $\begin{array} { r } { C \doteq \Delta \tilde { \bf p } _ { i j } + \frac { \partial \Delta \bar { \bf p } _ { i j } } { \partial { \bf b } _ { i } ^ { g } } \delta { \bf b } _ { i } ^ { g } + \frac { \partial \Delta \bar { \bf p } _ { i j } } { \partial { \bf b } _ { i } ^ { a } } \delta { \bf b } _ { i } ^ { a } } \end{array}$ Summarizing, the Jacobians of $\mathbf { r } _ { \Delta \mathbf { p } _ { i _ { \perp } } }$ are:

$$
\begin{array}{r l} \frac {\partial \mathbf {r} _ {\Delta \mathbf {p} _ {i j}}}{\partial \delta \phi_ {i}} = (\mathsf {R} _ {i} ^ {\top} (\mathbf {p} _ {j} - \mathbf {p} _ {i} - \mathbf {v} _ {i} \Delta t _ {i j} - \frac {1}{2} \mathbf {g} \Delta t _ {i j} ^ {2})) ^ {\wedge} & \frac {\partial \mathbf {r} _ {\Delta \mathbf {p} _ {i j}}}{\partial \delta \phi_ {j}} = \mathbf {0} \\ \frac {\partial \mathbf {r} _ {\Delta \mathbf {p} _ {i j}}}{\partial \delta \mathbf {p} _ {i}} = - \mathbf {I} _ {3 \times 1} & \frac {\partial \mathbf {r} _ {\Delta \mathbf {p} _ {i j}}}{\partial \delta \mathbf {p} _ {j}} = \mathsf {R} _ {i} ^ {\top} \mathsf {R} _ {j} \\ \frac {\partial \mathbf {r} _ {\Delta \mathbf {p} _ {i j}}}{\partial \delta \mathbf {v} _ {i}} = - \mathsf {R} _ {i} ^ {\top} \Delta t _ {i j} & \frac {\partial \mathbf {r} _ {\Delta \mathbf {p} _ {i j}}}{\partial \delta \mathbf {v} _ {j}} = \mathbf {0} \\ \frac {\partial \mathbf {r} _ {\Delta \mathbf {p} _ {i j}}}{\partial \tilde {\delta} \mathbf {b} _ {i} ^ {a}} = - \frac {\partial \Delta \bar {\mathbf {p}} _ {i j}}{\partial \mathbf {b} _ {i} ^ {a}} & \frac {\partial \mathbf {r} _ {\Delta \mathbf {p} _ {i j}}}{\partial \tilde {\delta} \mathbf {b} _ {i} ^ {g}} = - \frac {\partial \Delta \bar {\mathbf {p}} _ {i j}}{\partial \mathbf {b} _ {i} ^ {g}} \end{array}
$$

2) Jacobians of $\mathbf { r } _ { \Delta \mathbf { v } _ { i j } } .$ As in the previous section, $\mathbf { r } _ { \Delta \mathbf { v } _ { i j } }$ is linear in $\delta \mathbf { b } _ { i } ^ { g }$ and $\delta \mathbf { b } _ { i } ^ { a }$ , hence the Jacobians of $\mathbf { r } \Delta \mathbf { v } _ { i j }$ w.r.t. $\tilde { \delta } \mathbf { b } _ { i } ^ { g }$ , ˜δb<sup>a</sup> are simply the matrix coefficients of $\delta \mathbf { b } _ { i } ^ { g }$ and $\delta \mathbf { b } _ { i } ^ { a }$ . Moreover, $\mathtt { R } _ { j } ,$ $\mathbf { p } _ { i } ,$ and $\mathbf { p } _ { j }$ do not appear in $\mathbf { r } _ { \Delta \mathbf { v } _ { i j } } ,$ hence the Jacobians w.r.t. $\delta \phi _ { j } , \delta \mathbf { p } _ { i } , \delta \mathbf { p } _ { j }$ are zero. The remaining Jacobias are computed as:

$$
\begin{array}{r} \mathbf {r} _ {\Delta \mathbf {v} _ {i j}} (\mathbf {v} _ {i} + \delta \mathbf {v} _ {i}) = \mathsf {R} _ {i} ^ {\top} (\mathbf {v} _ {j} - \mathbf {v} _ {i} - \delta \mathbf {v} _ {i} - \mathbf {g} \Delta t _ {i j}) - D \\ = \mathbf {r} _ {\Delta \mathbf {v}} (\mathbf {v} _ {i}) - \mathsf {R} _ {i} ^ {\top} \delta \mathbf {v} _ {i} \end{array}\tag{75}
$$

$$
\begin{array}{r} \mathbf {r} _ {\Delta \mathbf {v} _ {i j}} (\mathbf {v} _ {j} + \delta \mathbf {v} _ {j}) = \mathsf {R} _ {i} ^ {\top} (\mathbf {v} _ {j} + \delta \mathbf {v} _ {j} - \mathbf {v} _ {i} - \mathbf {g} \Delta t _ {i j}) - D \\ = \mathbf {r} _ {\Delta \mathbf {v}} (\mathbf {v} _ {j}) + \mathsf {R} _ {i} ^ {\top} \delta \mathbf {v} _ {j} \end{array}\tag{76}
$$

$$
\begin{array}{l} \mathbf {r} _ {\Delta \mathbf {v} _ {i j}} (\mathrm{R} _ {i} \operatorname{Exp} (\delta \phi_ {i})) = (\mathrm{R} _ {i} \operatorname{Exp} (\delta \phi_ {i})) ^ {\top} (\mathbf {v} _ {j} - \mathbf {v} _ {i} - \mathbf {g} \Delta t _ {i j}) - D \\ \stackrel {(4)} {\simeq} (\mathbf {I} - \delta \phi_ {i} ^ {\wedge}) \mathrm{R} _ {i} ^ {\top} (\mathbf {v} _ {j} - \mathbf {v} _ {i} - \mathbf {g} \Delta t _ {i j}) - D \\ \stackrel {(2)} {\simeq} \mathbf {r} _ {\Delta \mathbf {v}} (\mathrm{R} _ {i}) + \left(\mathrm{R} _ {i} ^ {\top} (\mathbf {v} _ {j} - \mathbf {v} _ {i} - \mathbf {g} \Delta t _ {i j})\right) ^ {\wedge} \delta \phi_ {i}, \end{array} \tag {77}
$$

with $\begin{array} { r l } & { \cal { D } \doteq \left[ \Delta \tilde { \bf v } _ { i j } + \frac { \partial \Delta \bar { \bf v } _ { i j } } { \partial { \bf b } _ { i } ^ { g } } \delta { \bf b } _ { i } ^ { g } + \frac { \partial \Delta \bar { \bf v } _ { i j } } { \partial { \bf b } _ { i } ^ { a } } \delta { \bf b } _ { i } ^ { a } \right] } \\ & { \mathrm { i a n s ~ o f ~ } { \bf r } _ { \Delta { \bf v } _ { i j } } \mathrm { a r e } \mathrm { : } } \end{array}$ . Summarizing, the Jaco

$$
\begin{array}{l l} \frac {\partial \mathbf {r} _ {\Delta \mathbf {v} _ {i j}}}{\partial \delta \phi_ {i}} = \left(\mathsf {R} _ {i} ^ {\top} (\mathbf {v} _ {j} - \mathbf {v} _ {i} - \mathbf {g} \Delta t _ {i j})\right) ^ {\wedge} & \frac {\partial \mathbf {r} _ {\Delta \mathbf {v} _ {i j}}}{\partial \delta \phi_ {j}} = \mathbf {0} \\ \frac {\partial \mathbf {r} _ {\Delta \mathbf {v} _ {i j}}}{\partial \delta \mathbf {p} _ {i}} = \mathbf {0} & \frac {\partial \mathbf {r} _ {\Delta \mathbf {v} _ {i j}}}{\partial \delta \mathbf {p} _ {j}} = \mathbf {0} \\ \frac {\partial \mathbf {r} _ {\Delta \mathbf {v} _ {i j}}}{\partial \delta \mathbf {v} _ {i}} = - \mathsf {R} _ {i} ^ {\top} & \frac {\partial \mathbf {r} _ {\Delta \mathbf {v} _ {i j}}}{\partial \delta \mathbf {v} _ {j}} = \mathsf {R} _ {i} ^ {\top} \\ \frac {\partial \mathbf {r} _ {\Delta \mathbf {v} _ {i j}}}{\partial \bar {\delta} \mathbf {b} _ {i} ^ {a}} = - \frac {\partial \Delta \bar {\mathbf {v}} _ {i j}}{\partial \mathbf {b} _ {i} ^ {a}} & \frac {\partial \mathbf {r} _ {\Delta \mathbf {v} _ {i j}}}{\partial \bar {\delta} \mathbf {b} _ {i} ^ {g}} = - \frac {\partial \Delta \bar {\mathbf {v}} _ {i j}}{\partial \mathbf {b} _ {i} ^ {g}} \end{array}
$$

3) Jacobians of $\mathbf { r } _ { \Delta \mathrm { R } _ { i j } } .$ The derivation of the Jacobians of $\mathbf { r } _ { \Delta \mathrm { R } _ { i j } }$ is slightly more involved. We first note that $\mathbf { p } _ { i } , \mathbf { p } _ { j } , \mathbf { v } _ { i } , \mathbf { v } _ { j } , \delta \mathbf { b } _ { i } ^ { a }$ do not appear in the expression of $\mathbf { r } _ { \Delta \mathbb { R } _ { i j } } .$ , hence the corresponding Jacobians are zero. The remaining Jacobians can be computed as follows:

$$
\begin{array}{l} \mathbf {r} _ {\Delta \mathrm{R} _ {i j}} (\mathrm{R} _ {i} \operatorname{Exp} (\delta \phi_ {i})) = \operatorname{Log} \left(\left(\Delta \tilde {\mathrm{R}} _ {i j} (\bar {\mathbf {b}} _ {i} ^ {g}) E\right) ^ {\top} (\mathrm{R} _ {i} \operatorname{Exp} (\delta \phi_ {i})) ^ {\top} \mathrm{R} _ {j}\right) \\ = \operatorname{Log} \left(\left(\Delta \tilde {\mathrm{R}} _ {i j} (\bar {\mathbf {b}} _ {i} ^ {g}) E\right) ^ {\top} \operatorname{Exp} (- \delta \phi_ {i}) \mathrm{R} _ {i} ^ {\top} \mathrm{R} _ {j}\right) \\ \stackrel {(1 1)} {=} \operatorname{Log} \left(\left(\Delta \tilde {\mathrm{R}} _ {i j} (\bar {\mathbf {b}} _ {i} ^ {g}) E\right) ^ {\top} \mathrm{R} _ {i} ^ {\top} \mathrm{R} _ {j} \operatorname{Exp} (- \mathrm{R} _ {j} ^ {\top} \mathrm{R} _ {i} \delta \phi_ {i})\right) \\ \stackrel {(9)} {\simeq} \mathbf {r} _ {\Delta \mathrm{R}} (\mathrm{R} _ {i}) - \mathrm{J} _ {r} ^ {- 1} (\mathbf {r} _ {\Delta \mathrm{R}} (\mathrm{R} _ {i})) \mathrm{R} _ {j} ^ {\top} \mathrm{R} _ {i} \delta \phi_ {i} \end{array} \tag {78}
$$

$$
\begin{array}{l} \mathbf {r} _ {\Delta \mathrm{R} _ {i j}} (\mathrm{R} _ {j} \operatorname{Exp} (\delta \phi_ {j})) = \operatorname{Log} \left(\left(\Delta \tilde {\mathrm{R}} _ {i j} (\bar {\mathbf {b}} _ {i} ^ {g}) E\right) ^ {\top} \mathrm{R} _ {i} ^ {\top} (\mathrm{R} _ {j} \operatorname{Exp} (\delta \phi_ {j}))\right) \\ \stackrel {(9)} {\simeq} \mathbf {r} _ {\Delta \mathrm{R}} (\mathrm{R} _ {j}) + \mathrm{J} _ {r} ^ {- 1} (\mathbf {r} _ {\Delta \mathrm{R}} (\mathrm{R} _ {j})) \delta \phi_ {j} \end{array} \tag {79}
$$

$$
\begin{array}{l} \mathbf {r} _ {\Delta \mathrm{R} _ {i j}} (\delta \mathbf {b} _ {i} ^ {g} + \tilde {\delta} \mathbf {b} _ {i} ^ {g}) \\ = \operatorname{Log} \left(\left(\Delta \tilde {\mathrm{R}} _ {i j} (\bar {\mathbf {b}} _ {i} ^ {g}) \operatorname{Exp} \left(\frac {\partial \Delta \bar {\mathrm{R}} _ {i j}}{\partial \mathbf {b} ^ {g}} (\delta \mathbf {b} _ {i} ^ {g} + \tilde {\delta} \mathbf {b} _ {i} ^ {g})\right)\right) ^ {\top} \mathrm{R} _ {i} ^ {\top} \mathrm{R} _ {j}\right) \\ \stackrel {(7)} {\simeq} \operatorname{Log} \left(\left(\Delta \tilde {\mathrm{R}} _ {i j} (\bar {\mathbf {b}} _ {i} ^ {g}) E \operatorname{Exp} \left(\mathrm{J} _ {r} ^ {b} \frac {\partial \Delta \bar {\mathrm{R}} _ {i j}}{\partial \mathbf {b} ^ {g}} \tilde {\delta} \mathbf {b} _ {i} ^ {g}\right)\right) ^ {\top} \mathrm{R} _ {i} ^ {\top} \mathrm{R} _ {j}\right) \\ = \operatorname{Log} \left(\operatorname{Exp} \left(- \mathrm{J} _ {r} ^ {b} \frac {\partial \Delta \bar {\mathrm{R}} _ {i j}}{\partial \mathbf {b} ^ {g}} \tilde {\delta} \mathbf {b} _ {i} ^ {g}\right) (\Delta \tilde {\mathrm{R}} _ {i j} (\bar {\mathbf {b}} _ {i} ^ {g}) E) ^ {\top} \mathrm{R} _ {i} ^ {\top} \mathrm{R} _ {j}\right) \\ = \operatorname{Log} \left(\operatorname{Exp} \left(- \mathrm{J} _ {r} ^ {b} \frac {\partial \Delta \bar {\mathrm{R}} _ {i j}}{\partial \mathbf {b} ^ {g}} \tilde {\delta} \mathcal {b} _ {i} ^ {g}\right) \operatorname{Exp} (\mathbf {r} _ {\Delta \mathrm{R} _ {i j}} (\delta \mathbf {b} _ {i} ^ {g}))\right) \\ \stackrel {(1 1)} {=} \operatorname{Log} \left(\operatorname{Exp} (\mathbf {r} _ {\Delta \mathrm{R} _ {i j}} (\delta \mathbf {b} _ {i} ^ {g})) \cdot \operatorname{Exp} (- \operatorname{Exp} (\mathbf {r} _ {\Delta \mathrm{R} _ {i j}} (\delta \mathbf {b} _ {i} ^ {g})) ^ {\top} \mathrm{J} _ {r} ^ {b} \frac {\partial \Delta \bar {\mathrm{R}} _ {i j}}{\partial \mathbf {b} ^ {g}} \tilde {\delta} \mathcal {b} _ {i} ^ {g})\right) \\ \stackrel {(9)} {\simeq} \mathbf {r} _ {\Delta \mathrm{R} _ {i j}} (\delta \mathbf {b} _ {i} ^ {g}) \\ - \mathrm{J} _ {r} ^ {- 1} (\mathbf {r} _ {\Delta \mathrm{R} _ {i j}} (\delta \mathbf {b} _ {i} ^ {g})) \operatorname{Exp} (\mathbf {r} _ {\Delta \mathrm{R} _ {i j}} (\delta \mathbf {b} _ {i} ^ {g})) ^ {\top} \mathrm{J} _ {r} ^ {b} \frac {\partial \Delta \bar {\mathrm{R}} _ {i j}}{\partial \mathbf {b} ^ {g}} \tilde {\delta} \mathcal {b}_{i}\end{array}
$$

where we used the shorthands $\begin{array} { r } { E \doteq \exp \left( \frac { \partial \Delta \bar { \bf R } _ { i j } } { \partial { \bf b } ^ { g } } \delta { \bf b } ^ { g } \right) } \end{array}$ and $\bar { \mathsf { J } } _ { r } ^ { b } \doteq \mathsf { \bar { \Phi } }$ $\mathbb { J } _ { r } \left( \frac { \partial \Delta \bar { \mathbb { R } } _ { i j } } { \partial \mathbf { b } ^ { g } } \delta \mathbf { b } _ { i } ^ { g } \right)$ . In summary, the Jacobians of $\mathbf { r } \Delta \mathtt { R } _ { i , j }$ are:

$$
\begin{array}{r l} \frac {\partial \mathbf {r} _ {\Delta \mathsf {R} _ {i j}}}{\partial \delta \phi_ {i}} = - \mathbf {J} _ {r} ^ {- 1} (\mathbf {r} _ {\Delta \mathsf {R}} (\mathsf {R} _ {i})) \mathsf {R} _ {j} ^ {\mathsf {T}} \mathsf {R} _ {i} & \frac {\partial \mathbf {r} _ {\Delta \mathsf {R} _ {i j}}}{\partial \delta \mathbf {p} _ {i}} = \mathbf {0} \\ \frac {\partial \mathbf {r} _ {\Delta \mathsf {R} _ {i j}}}{\partial \delta \mathbf {v} _ {i}} = \mathbf {0} & \frac {\partial \mathbf {r} _ {\Delta \mathsf {R} _ {i j}}}{\partial \delta \phi_ {j}} = \mathbf {J} _ {r} ^ {- 1} (\mathbf {r} _ {\Delta \mathsf {R}} (\mathsf {R} _ {j})) \\ \frac {\partial \mathbf {r} _ {\Delta \mathsf {R} _ {i j}}}{\partial \delta \mathbf {p} _ {j}} = \mathbf {0} & \frac {\partial \mathbf {r} _ {\Delta \mathsf {R} _ {i j}}}{\partial \delta \mathbf {v} _ {j}} = \mathbf {0} \\ \frac {\partial \mathbf {r} _ {\Delta \mathsf {R} _ {i j}}}{\partial \tilde {\delta} \mathbf {b} _ {i} ^ {a}} = \mathbf {0} & \frac {\partial \mathbf {r} _ {\Delta \mathsf {R} _ {i j}}}{\partial \tilde {\delta} \mathbf {b} _ {i} ^ {g}} = \alpha \end{array}\tag{81}
$$

$$
\mathrm{with} \alpha = - \mathbf {J} _ {r} ^ {- 1} \left(\mathbf {r} _ {\Delta \mathbb {R} _ {i j}} (\delta \mathbf {b} _ {i} ^ {g})\right) \mathrm{Exp} \left(\mathbf {r} _ {\Delta \mathbb {R} _ {i j}} (\delta \mathbf {b} _ {i} ^ {g})\right) ^ {\mathsf {T}} \mathbf {J} _ {r} ^ {b} \frac {\partial \Delta \bar {\mathbb {R}} _ {i j}}{\partial \mathbf {b} ^ {g}}.
$$

## D. Structureless Vision Factors: Null Space Projection

In this section we provide a more efficient implementation of the structureless vision factors, described in Section VII.

Let us consider Eq. (55). Recall that $\mathbf { Q } \doteq ( \mathbf { I } { - } \mathbf { E } _ { l } ( \mathbf { E } _ { l } ^ { \mathsf { T } } \mathbf { E } _ { l } ) ^ { - 1 } \mathbf { E } _ { l } ^ { \mathsf { T } } ) \in$ $\mathbb { R } ^ { 2 n _ { l } \times 2 n _ { l } }$ is an orthogonal projector of $\mathbf { E } _ { l }$ , where n<sub>l</sub> is the number of cameras observing landmark l. Roughly speaking, $\mathbf { Q }$ projects any vector in $\mathbb { R } ^ { 2 n _ { l } }$ to the null space of the matrix $\mathbf { E } _ { l }$ . Since $\mathbf { \dot { E } } _ { l } \in \mathbb { R } ^ { 2 n _ { l } \times \mathbf { \dot { 3 } } }$ has rank $^ { 3 , }$ the dimension of its null space is $2 n _ { l } - 3 .$ Any basis $\mathbf { E } _ { l } ^ { \perp } \in \mathbb { R } ^ { 2 \acute { n _ { l } } \times 2 n _ { l } - 3 }$ of the null space of $\mathbf { E } _ { l }$ satisfies the following relation [71]:

$$
\mathbf {E} _ {l} ^ {\perp} \left(\left(\mathbf {E} _ {l} ^ {\perp}\right) ^ {\mathsf {T}} \mathbf {E} _ {l} ^ {\perp}\right) ^ {- 1} \left(\mathbf {E} _ {l} ^ {\perp}\right) ^ {\mathsf {T}} = \mathbf {I} - \mathbf {E} _ {l} \left(\mathbf {E} _ {l} ^ {\mathsf {T}} \mathbf {E} _ {l}\right) ^ {- 1} \mathbf {E} _ {l} ^ {\mathsf {T}}.\tag{82}
$$

A basis for the null space can be easily computed from $\mathbf { E } _ { l }$ using SVD. Such basis is unitary, i.e., satisfies $( \mathbf { \check { E } } _ { l } ^ { \perp } ) ^ { \mathsf { T } } \mathbf { \check { E } } _ { l } ^ { \perp } = \mathbf { I }$ . Substituting (82) into (55), and recalling that $\mathbf { E } _ { l } ^ { \perp }$ is a unitary matrix, we obtain:

$$
\begin{array}{l} \sum_ {l = 1} ^ {L} \left\| \mathbf {E} _ {l} ^ {\perp} \left(\mathbf {E} _ {l} ^ {\perp}\right) ^ {\top} \left(\mathbf {F} _ {l} \delta \mathbf {T} _ {\mathcal {X} (l)} - \mathbf {b} _ {l}\right) \right\| ^ {2} \\ = \sum_ {l = 1} ^ {L} \left(\mathbf {E} _ {l} ^ {\perp} \left(\mathbf {E} _ {l} ^ {\perp}\right) ^ {\top} \left(\mathbf {F} _ {l} \delta \mathbf {T} _ {\mathcal {X} (l)} - \mathbf {b} _ {l}\right)\right) ^ {\top} \left(\mathbf {E} _ {l} ^ {\perp} \left(\mathbf {E} _ {l} ^ {\perp}\right) ^ {\top} \left(\mathbf {F} _ {l} \delta \mathbf {T} _ {\mathcal {X} (l)} - \mathbf {b} _ {l}\right)\right) \\ = \sum_ {l = 1} ^ {L} \left(\mathbf {F} _ {l} \delta \mathbf {T} _ {\mathcal {X} (l)} - \mathbf {b} _ {l}\right) ^ {\top} \mathbf {E} _ {l} ^ {\perp} \overbrace {\left(\mathbf {E} _ {l} ^ {\perp}\right) ^ {\top} \mathbf {E} _ {l} ^ {\perp}} ^ {= \mathbf {I} _ {3 \times 3}} \left(\mathbf {E} _ {l} ^ {\perp}\right) ^ {\top} \left(\mathbf {F} _ {l} \delta \mathbf {T} _ {\mathcal {X} (l)} - \mathbf {b} _ {l}\right) \\ = \sum_ {l = 1} ^ {L} \| (\mathbf {E} _ {l} ^ {\perp}) ^ {\top} (\mathbf {F} _ {l} \delta \mathbf {T} _ {\mathcal {X} (l)} - \mathbf {b} _ {l}) \| ^ {2} \end{array} \tag {83}
$$

which is an alternative representation of the cost function (55). This representation is usually preferable from a computational standpoint, as it does not include matrix inversion and can be computed using a smaller number of matrix multiplications.

## E. Rotation Rate Integration Using Euler Angles

In this section, we recall how to integrate rotation rate measurements using the Euler angle parametrization. Let $\tilde { \omega } _ { k }$ be the rotation rate measurement at time k and $\eta _ { k } ^ { g }$ be the corresponding noise. Then, given the vector of Euler angles at time $k ,$ namely $\pmb { \theta } _ { k } \in \mathbb { R } ^ { 3 }$ , we can integrate the rotation rate measurement $\tilde { \omega } _ { k }$ and get $\pmb { \theta } _ { k + 1 }$ as follows:

$$
\pmb {\theta} _ {k + 1} = \pmb {\theta} _ {k} + [ E ^ {\prime} (\pmb {\theta} _ {k}) ] ^ {- 1} (\tilde {\pmb {\omega}} _ {k} - \pmb {\eta} _ {k} ^ {g}) \Delta t,\tag{84}
$$

where the matrix $E ^ { \prime } ( \pmb { \theta } _ { k } )$ is the conjugate Euler angle rate matrix [72]. The covariance of $\pmb { \theta } _ { k + 1 }$ can be approximated by a firstorder propagation as:

$$
\boldsymbol {\Sigma} _ {k + 1} ^ {\text { Euler }} = \mathbf {A} _ {k} \boldsymbol {\Sigma} _ {k} ^ {\text { Euler }} \mathbf {A} _ {k} ^ {\top} + \mathbf {B} _ {k} \boldsymbol {\Sigma} _ {\eta} \mathbf {B} _ {k} ^ {\top}\tag{85}
$$

where $\begin{array} { r } { \mathbf { A } _ { k } \doteq \mathbf { I } _ { 3 \times 3 } + \frac { \partial [ E ^ { \prime } ( \pmb \theta _ { k } ) ] ^ { - 1 } } { \partial \pmb \theta _ { k } } \Delta t , \mathbf { B } _ { k } = - [ E ^ { \prime } ( \pmb \theta _ { k } ) ] ^ { - 1 } \Delta t , } \end{array}$ , and $\Sigma _ { \eta }$ is the covariance of the measurement noise $\eta _ { k } ^ { g d }$

[1] A. Martinelli. Vision and IMU data fusion: Closedform solutions for attitude, speed, absolute scale, and bias determination. IEEE Trans. Robotics, 28(1):44–60, 2012.

[2] T. Lupton and S. Sukkarieh. Visual-inertial-aided navigation for high-dynamic motion in built environments without initial conditions. IEEE Trans. Robotics, 28(1): 61–76, Feb 2012.

[3] M. Kaess, H. Johannsson, R. Roberts, V. Ila, J. Leonard, and F. Dellaert. iSAM2: Incremental smoothing and mapping using the Bayes tree. Intl. J. of Robotics Research, 31:217–236, Feb 2012.

[4] L. Carlone, Z. Kira, C. Beall, V. Indelman, and F. Dellaert. Eliminating conditionally independent sets in factor graphs: A unifying perspective based on smart factors. In IEEE Intl. Conf. on Robotics and Automation (ICRA), 2014.

[5] A.I. Mourikis and S.I. Roumeliotis. A multi-state constraint Kalman filter for vision-aided inertial navigation. In IEEE Intl. Conf. on Robotics and Automation (ICRA), pages 3565–3572, April 2007.

[6] C. Forster, L. Carlone, F. Dellaert, and D. Scaramuzza. IMU preintegration on manifold for efficient visualinertial maximum-a-posteriori estimation. In Robotics: Science and Systems (RSS), 2015.

[7] Frank Dellaert. Factor graphs and GTSAM: A handson introduction. Technical Report GT-RIM-CP&R-2012- 002, Georgia Institute of Technology, September 2012.

[8] K. J. Wu, A. M. Ahmed, G. A. Georgiou, and S. I. Roumeliotis. A square root inverse filter for efficient vision-aided inertial navigation on mobile devices. In Robotics: Science and Systems (RSS), 2015.

[9] Bradley M. Bell and Frederick W. Cathey. The iterated kalman filter update as a gauss-newton method. IEEE Transactions on Automatic Control, 38(2):294– 297, 1993.

[10] A.J. Davison, I. Reid, N. Molton, and O. Stasse. MonoSLAM: Real-time single camera SLAM. IEEE Trans. Pattern Anal. Machine Intell., 29(6):1052–1067, Jun 2007.

[11] M. Bloesch, S. Omari, M. Hutter, and R. Siegwart. Robust visual inertial odometry using a direct EKF-based approach. In IEEE/RSJ Intl. Conf. on Intelligent Robots and Systems (IROS). IEEE, 2015.

[12] E.S. Jones and S. Soatto. Visual-inertial navigation, mapping and localization: A scalable real-time causal approach. Intl. J. of Robotics Research, 30(4), Apr 2011.

[13] S. I. Roumeliotis and J. W. Burdick. Stochastic cloning: A generalized framework for processing relative state measurements. In IEEE Intl. Conf. on Robotics and Automation (ICRA). IEEE, 2002.

[14] K. Tsotsos, A. Chiuso, and S. Soatto. Robust inference for visual-inertial sensor fusion. In IEEE Intl. Conf. on Robotics and Automation (ICRA), 2015.

[15] A. Martinelli. Observability properties and deterministic algorithms in visual-inertial structure from motion. Foundations and Trends in Robotics, pages 1–75, 2013.

[16] Dimitrios G. Kottas, Joel A. Hesch, Sean L. Bowman, and Stergios I. Roumeliotis. On the consistency of visionaided inertial navigation. In Intl. Sym. on Experimental Robotics (ISER), 2012.

[17] G.P. Huang, A.I. Mourikis, and S.I. Roumeliotis. A firstestimates jacobian EKF for improving SLAM consistency. In Intl. Sym. on Experimental Robotics (ISER), 2008.

[18] J.A. Hesch, D.G. Kottas, S.L. Bowman, and S.I. Roumeliotis. Camera-imu-based localization: Observability analysis and consistency improvement. Intl. J. of Robotics Research, 33(1):182–201, 2014.

[19] J. Hernandez, K. Tsotsos, and S. Soatto. Observability, identifiability and sensitivity of vision-aided inertial navigation. In IEEE Intl. Conf. on Robotics and Automation (ICRA), 2015.

[20] A.I. Mourikis and S.I. Roumeliotis. A dual-layer estimator architecture for long-term localization. In Proc. of the Workshop on Visual Localizationfor Mobile Platforms at CVPR, Anchorage, Alaska, June 2008.

[21] G. Sibley, L. Matthies, and G. Sukhatme. Sliding window filter with application to planetary landing. J. of Field Robotics, 27(5):587–608, 2010.

[22] T-C. Dong-Si and A.I. Mourikis. Motion tracking with fixed-lag smoothing: Algorithm consistency and analysis. In IEEE Intl. Conf. on Robotics and Automation (ICRA), 2011.

[23] S. Leutenegger, P. Furgale, V. Rabaud, M. Chli, K. Konolige, and R. Siegwart. Keyframe-based visual-inertial slam using nonlinear optimization. In Robotics: Science and Systems (RSS), 2013.

[24] S. Leutenegger, S. Lynen, M. Bosse, R. Siegwart, and P. Furgale. Keyframe-based visual-inertial slam using nonlinear optimization. Intl. J. of Robotics Research, 2015.

[25] P. Maybeck. Stochastic Models, Estimation and Control, volume 1. Academic Press, New York, 1979.

[26] G.P. Huang, A.I. Mourikis, and S.I. Roumeliotis. An observability-constrained sliding window filter for SLAM. In IEEE/RSJ Intl. Conf. on Intelligent Robots and Systems (IROS), pages 65–72, 2011.

[27] S-H. Jung and C.J. Taylor. Camera trajectory estimation using inertial sensor measurements and structure fom motion results. In IEEE Conf. on Computer Vision and Pattern Recognition (CVPR), 2001.

[28] D. Sterlow and S. Singh. Motion estimation from image and inertial measurements. 2004.

[29] M. Bryson, M. Johnson-Roberson, and S. Sukkarieh. Airborne smoothing and mapping using vision and inertial sensors. In IEEE Intl. Conf. on Robotics and Automation (ICRA), pages 3143–3148, 2009.

[30] V. Indelman, S. Wiliams, M. Kaess, and F. Dellaert. Information fusion in navigation systems via factor graph based incremental smoothing. Robotics and Autonomous Systems, 61(8):721–738, August 2013.

[31] A. Patron-Perez, S. Lovegrove, and G. Sibley. A splinebased trajectory representation for sensor fusion and rolling shutter cameras. Intl. J. of Computer Vision,

February 2015.

[32] H. Strasdat, J.M.M. Montiel, and A.J. Davison. Realtime monocular SLAM: Why filter? In IEEE Intl. Conf. on Robotics and Automation (ICRA), pages 2657–2664, 2010.

[33] G. Klein and D. Murray. Parallel tracking and mapping on a camera phone. In IEEE and ACM Intl. Sym. on Mixed and Augmented Reality (ISMAR), 2009.

[34] E.D. Nerurkar, K.J. Wu, and S.I. Roumeliotis. C-KLAM: Constrained keyframe-based localization and mapping. In IEEE Intl. Conf. on Robotics and Automation (ICRA), 2014.

[35] G. Klein and D. Murray. Parallel tracking and mapping for small AR workspaces. In IEEE and ACM Intl. Sym. on Mixed and Augmented Reality (ISMAR), pages 225– 234, Nara, Japan, Nov 2007.

[36] M. Kaess, A. Ranganathan, and F. Dellaert. iSAM: Incremental smoothing and mapping. IEEE Trans. Robotics, 24(6):1365–1378, Dec 2008.

[37] V. Indelman, S. Wiliams, M. Kaess, and F. Dellaert. Factor graph based incremental smoothing in inertial navigation systems. In Intl. Conf. on Information Fusion, FUSION, 2012.

[38] V. Indelman, A. Melim, and F. Dellaert. Incremental light bundle adjustment for robotics navigation. In IEEE/RSJ Intl. Conf. on Intelligent Robots and Systems (IROS), November 2013.

[39] S. Shen. Autonomous Navigation in Complex Indoor and Outdoor Environments with Micro Aerial Vehicles. PhD Thesis, University of Pennsylvania, 2014.

[40] N. Keivan, A. Patron-Perez, and G. Sibley. Asynchronous adaptive conditioning for visual-inertial SLAM. In Intl. Sym. on Experimental Robotics (ISER), 2014.

[41] J. Hornegger and C. Tomasi. Representation issues in the ML estimation of camera motion. In Intl. Conf. on Computer Vision (ICCV), pages 640–647, Kerkyra, Greece, September 1999.

[42] M. Moakher. Means and averaging in the group of rotations. SIAM Journal on Matrix Analysis and Applications, 24(1):1–16, 2002.

[43] G. S. Chirikjian. Stochastic Models, Information Theory, and Lie Groups, Volume 2: Analytic Methods and Modern Applications (Applied and Numerical Harmonic Analysis). Birkhauser, 2012.

[44] Y. Wang and G.S. Chirikjian. Nonparametric secondorder theory of error propagation on motion groups. Intl. J. of Robotics Research, 27(11–12):1258–1273, 2008.

[45] P.A. Absil, C.G. Baker, and K.A. Gallivan. Trustregion methods on Riemannian manifolds. Foundations of Computational Mathematics, 7(3):303–330, 2007.

[46] T. D. Barfoot and P. T. Furgale. Associating uncertainty with three-dimensional poses for use in estimation problems. IEEE Trans. Robotics, 30(3):679–693, 2014.

[47] Y. Wang and G.S. Chirikjian. Error propagation on the euclidean group with applications to manipulator kinematics. IEEE Trans. Robotics, 22(4):591–602, 2006.

[48] S. T. Smith. Optimization techniques on Riemannian manifolds. Hamiltonian and Gradient Flows, Algorithms

and Control, Fields Inst. Commun., Amer. Math. Soc., 3: 113–136, 1994.

[49] J.A. Farrell. Aided Navigation: GPS with High Rate Sensors. McGraw-Hill, 2008.

[50] J. H. Manton. Optimization algorithms exploiting unitary constraints. IEEE Trans. Signal Processing, 50:63–650, 2002.

[51] F.R. Kschischang, B.J. Frey, and H-A. Loeliger. Factor graphs and the sum-product algorithm. IEEE Trans. Inform. Theory, 47(2), February 2001.

[52] F. Dellaert. Square Root SAM: Simultaneous localization and mapping via square root information smoothing. Technical Report GIT-GVU-05-11, Georgia Institute of Technology, 2005.

[53] R.M. Murray, Z. Li, and S. Sastry. A Mathematical Introduction to Robotic Manipulation. CRC Press, 1994.

[54] P. E. Crouch and R. Grossman. Numerical integration of ordinary differential equations on manifolds. J. Nonlinear Sci., 3:1–22, 1993.

[55] H. Munthe-Kaas. Higher order runge-kutta methods on manifolds. Appl. Numer. Math., 29(1):115–127, 1999.

[56] J. Park and W.-K. Chung. Geometric integration on euclidean group with application to articulated multibody systems. IEEE Trans. Robotics, 2005.

[57] M. S. Andrle and J. L. Crassidis. Geometric integration of quaternions. Journal of Guidance, Control, and Dynamics, 36(6):1762–1757, 2013.

[58] J.L. Crassidis. Sigma-point Kalman filtering for integrated GPS and inertial navigation. IEEE Trans. Aerosp. Electron. Syst., 42(2):750–756, 2006.

[59] P. Furgale, J. Rehder, and R. Siegwart. Unified temporal and spatial calibration for multi-sensor systems. In IEEE/RSJ Intl. Conf. on Intelligent Robots and Systems (IROS), 2013.

[60] M. Li and A.I. Mourikis. Online temporal calibration for camera-imu systems: Theory and algorithms. Intl. J. of Robotics Research, 33(6), 2014.

[61] R. I. Hartley and A. Zisserman. Multiple View Geometry in Computer Vision. Cambridge University Press, second edition, 2004.

[62] M. Kaess, V. Ila, R. Roberts, and F. Dellaert. The Bayes tree: Enabling incremental reordering and fluid relinearization for online mapping. Technical Report MIT-CSAIL-TR-2010-021, Computer Science and Artificial Intelligence Laboratory, MIT, Jan 2010.

[63] Y. Bar-Shalom, X. R. Li, and T. Kirubarajan. Estimation with Applications To Tracking and Navigation. John Wiley and Sons, 2001.

[64] J. Hornegger. Statistical modeling of relations for 3- D object recognition. In Intl. Conf. Acoust., Speech, and Signal Proc. (ICASSP), volume 4, pages 3173–3176, Munich, April 1997.

[65] C. Forster, M. Pizzoli, and D. Scaramuzza. SVO: Fast Semi-Direct Monocular Visual Odometry. In IEEE Intl. Conf. on Robotics and Automation (ICRA), 2014. doi: 10.1109/ICRA.2014.6906584.

[66] J. Nikolic, J., Rehderand M. Burri, P. Gohl, S. Leutenegger, P. Furgale, and R. Siegwart. A Synchronized Visual-

Inertial Sensor System with FPGA Pre-Processing for Accurate Real-Time SLAM. In IEEE Intl. Conf. on Robotics and Automation (ICRA), 2014.

[67] F.C. Park and B.J. Martin. Robot sensor calibration: Solving AX=XB on the euclidean group. 10(5), 1994.

[68] A. Geiger, P. Lenz, and R. Urtasun. Are we ready for autonomous driving? the KITTI vision benchmark suite. In IEEE Conf. on Computer Vision and Pattern Recognition (CVPR), pages 3354–3361, Providence, USA, June 2012.

[69] Tango IMU specifications. URL http://ae-bst. resource.bosch.com/media/products/dokumente/bmx055/ BST-BMX055-FL000-00 2013-05-07 onl.pdf.

[70] Adis IMU specifications. URL http://www.analog. com/media/en/technical-documentation/data-sheets/ ADIS16448.pdf.

[71] C.D. Meyer. Matrix Analysis and Applied Linear Algebra. SIAM, 2000.

[72] J. Diebel. Representing attitude: Euler angles, unit quaternions, and rotation vectors. Technical report, Stanford University, 2006.

![](images/5946423ba64c0e1fbd2c4148cc14996fff14909fc2f730756cdf23503aefde72.jpg)  
environment.

Christian Forster (1986, Swiss) obtained his Ph.D. in Computer Science (2016) at the University of Zurich under the supervision of Davide Scaramuzza. Previously, he received a B.Sc. degree in Mechanical Engineering (2009) and a M.Sc. degree in Robotics, Systems and Control (2012) at ETH Zurich, Switzerland. In 2011, he was a visiting researcher at CSIR (South Africa), and in 2014 at Georgia Tech in the group of Frank Dellaert. He is broadly interested in developing real-time computer vision algorithms that enable robots to perceive the three dimensional

![](images/142cf0ffcf64eeee9545ff602080a5c809d0d19a71f993e62bccfb108fe895b1.jpg)

Luca Carlone is a research scientist in the Laboratory for Information and Decision Systems at the Massachusetts Institute of Technology. Before joining MIT, he was a postdoctoral fellow at Georgia Tech (2013-2015), a visiting researcher at the University of California Santa Barbara (2011), and a visiting researcher at the University of Zaragoza (2010). He got his Ph.D. from Politecnico di Torino, Italy, in 2012. His research interests include nonlinear estimation, optimization, and control applied to robotics, with special focus on localization, mapping, and decision making for navigation in single and multi robot systems.

![](images/7e37ff1424814fdb3143214c7407d567221e7e10c193f7532ce593951a40c31c.jpg)

Frank Dellaert is Professor in the School of Interactive Computing at Georgia Tech. He graduated from Carnegie Mellon in 2001 with a Ph.D. in Computer Science, after obtaining degrees from Case Western Reserve University and K.U. Leuven before that. Professor Dellaert’s research focuses on large-scale inference for autonomous robot systems, on land, air, and in water. He pioneered the use of several probabilistic methods in both computer vision and robotics. With Dieter Fox and Sebastian Thrun, he has introduced the Monte Carlo localization method

for estimating and tracking the pose of robots, which is now a standard and popular tool in mobile robotics. Most recently, he has investigated 3D reconstruction in large-scale environments by taking a graph-theoretic view, and pioneered the use of sparse factor graphs as a representation in robot navigation and mapping. He and his group released an open-source largescale optimization toolbox, GTSAM.

![](images/0fdc68b918a21a61ff184efa67870f7c9dc7c90006c2d1c9adf2a476dfdbd672.jpg)

Davide Scaramuzza (1980, Italian) is Professor of Robotics at the University of Zurich, where he does research at the intersection of robotics, computer vision, and neuroscience. He did his PhD in robotics and computer Vision at ETH Zurich and a postdoc at the University of Pennsylvania. From 2009 to 2012, he led the European project sFly, which introduced the worlds first autonomous navigation of micro drones in GPS-denied environments using visualinertial sensors as the only sensor modality. For his research contributions, he was awarded an SNSF-

ERC Starting Grant, the IEEE Robotics and Automation Early Career Award, and a Google Research Award. He coauthored the book Introduction to Autonomous Mobile Robots (published by MIT Press).