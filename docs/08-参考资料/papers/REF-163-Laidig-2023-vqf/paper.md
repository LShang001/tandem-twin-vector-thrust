# VQF: Highly Accurate IMU Orientation Estimation with Bias Estimation and Magnetic Disturbance Rejection⋆

Daniel Laidiga,∗, Thomas Seelb

<sup>a</sup>Control Systems Group, Technische Universität Berlin, 10623 Berlin, Germany

<sup>b</sup>Department Artificial Intelligence in Biomedical Engineering, Friedrich-Alexander-Universität Erlangen-Nürnberg, 91052 Erlangen, Germany

## A R T I C L E I N F O

Keywords: inertial sensor inertial measurement unit IMU AHRS orientation estimation attitude estimation sensor fusion magnetic disturbances gyroscope bias estimation

## A BS T RA C T

The miniaturization of MEMS-based inertial measurement units (IMUs) facilitates their widespread use in a growing number of application domains. The fundamental sensor fusion task of orientation estimation is a prerequisite for most further data processing steps in inertial motion tracking, such as position and velocity estimation, joint angle estimation, and 3D visualization. Errors in the estimated orientations severely afect all further processing steps. Recent systematic comparisons of existing algorithms show that out-of-the-box accuracy is often low and that application-specific tuning is required to obtain high accuracy. In the present work, we propose and extensively evaluate a quaternion-based orientation estimation algorithm that is based on a novel approach of filtering the acceleration measurements in an almost-inertial frame and that includes extensions for gyroscope bias estimation and magnetic disturbance rejection, as well as a variant for ofline data processing. In contrast to all existing work, we perform an extensive evaluation, using a large collection of publicly available datasets and eight literature methods for comparison. The proposed method consistently outperforms all eight literature methods and achieves an average RMSE of 2.9°, while the errors obtained with literature methods range from 5.3° to 16.7°. This improved accuracy with respect to the state of the art is observed not only in average but also for each of several diferent motion characteristics, as well as for gyroscope bias estimation. Since the evaluation was performed with one single fixed parametrization across a very diverse dataset collection, we conclude that the proposed method provides unprecedented out-of-the-box performance for a broad range of motions, sensor hardware, and environmental conditions. This gain in orientation estimation accuracy is expected to advance the field of IMU-based motion analysis and provide performance benefits in numerous applications. The provided open-source implementation makes it easy to employ the proposed method.

## 1. Introduction

In recent years, MEMS-based inertial measurement units (IMUs) have become small, lightweight, and afordable. New application domains in which they are used include, for example, sports [1], gait analysis [2, 3], rehabilitation monitoring [4], rehabilitation robotics [5], autonomous vehicles [6], aerial vehicles [7], and kites [8]. In all these applications, IMUs are used to estimate variables of motion, such as orientations, velocities, and positions, either in real time or via postprocessing of recorded data.

IMUs measure angular rate, specific force (also called proper acceleration), and magnetic field strength, each as a time-dependent 3D vector in an intrinsic sensor coordinate system. Those measurements are processed to determine the motion parameters of interest, e.g., the orientation of an object to which the sensor is attached, the object’s velocity or position, or other application-specific motion parameters [9, 10]. As shown in Fig. 1, determining such motion parameters generally requires the prior estimation of the orientation of the sensor with respect to an inertial frame of reference, a procedure known as inertial orientation estimation (IOE).

Since IOE is such a fundamental step in IMU-based motion analysis and the accuracy of all further parame ters of interest depends on the accuracy of the orientation estimate, it is not surprising that abundant prior research has aimed at solving this task. Comprehensive reviews that classify and compare the existing solution approaches are found in [11, 12]. The existing methods widely vary with respect to the filter type, the computational complexity, the number of tuning parameters, and the additional features such as gyroscope bias estimation. For several algorithms, implementations in C++ or Matlab are available, see e.g. [12, 13, 14, 15, 16, 17].

In literature that proposes new IOE algorithms, accuracy is most commonly validated using marker-based optical motion capture (OMC) as a ground truth. However, the employed datasets are almost always non-public and highly application-specific; they vary widely in terms of movement speed, characteristics of the employed motion, magnetic environment, and sensor error characteristics. Therefore, reported performance figures cannot be compared directly. This lack of common datasets and suitable benchmarks has recently been addressed by the publication of the Sassari dataset [18] and the Berlin Robust Orientation Estimation Assessment Dataset (BROAD) [19]. Newly proposed algorithms should be validated using such publicly available datasets in comparison with other state-of-the-art algorithms [11].

VQF: Highly Accurate IMU Orientation Estimation with Bias Estimation and Magnetic Disturbance Rejection  
![](images/ec9aef3d484f4d0483b1651e853c57d12948029e010213ca01ca977f3cbbc807.jpg)  
Figure 1: Orientation estimation of IMUs is achieved by sensor fusion of the gyroscope measurements with the accelerometer and, in 9D sensor fusion, magnetometer measurements. Obtaining an accurate orientation estimate is the prerequisite for fundamenta further steps in inertial motion tracking, including velocity and position estimation, joint angle calculation, and 3D visualization.

The few existing comparative studies show that the out of-the-box performance of most algorithms is poor and that application-specific algorithm selection, as well as laborious parameter tuning, are necessary to achieve good results [12, 19], which represents a severe limitation of the state of the art. Even with optimized parameters, the root-meansquare errors achieved by the best IOE algorithms are in the range of 1° to 3° for slow and smooth motions and as much as 5° to 15° for fast and challenging motions [12, 19]. Further improving this accuracy seems highly desirable in view of numerous applications.

In summary, while there is ample work on various IOE algorithms, evaluation of the proposed methods is often limited and cannot be compared across publications. Recent comparative reviews and benchmarks show that there is no one-size-fits-all solution that works out of the box and yields high accuracy for a wide variety of application scenarios. Furthermore, the widespread adoption of novel IOE algorithms is not only driven by accuracy but also de pends on the availability of an easy-to-use implementation. In combination, this demonstrates that there is a need for an algorithm that is validated on a very large and diverse set of experimental data, provides accurate out-of-the-box orientation estimates without tuning, and is easy to use and to integrate into existing code projects.

We aim at filling this gap in two steps: We first propose a new feature-rich quaternion-based orientation estimation al gorithm and then perform an extensive validation to demon strate the exceptionally high accuracy that is achieved by this algorithm. With respect to the first step, the key diferences of the proposed algorithm with respect to the latest state of the art are best expressed by the following five features:

1. As a novel approach to sensor fusion of gyroscopes and accelerometers, the accelerometer information is low-pass filtered in an almost-inertial frame, which yields robust rejection of accelerations due to velocity changes.

2. Magnetometer-based heading correction is performed as a modular decoupled step, which eliminates the influence of magnetic disturbances on the inclination and facilitates simultaneous 6D and 9D estimation.

3. The algorithm includes extensions for online gyroscope bias estimation during rest and motion and an optional magnetic disturbance rejection strategy.

4. In contrast to the vast majority of previous approaches, an acausal ofline version is available, which further increases the accuracy in situations in which real-time capability is not required.

5. Easy-to-use open-source implementations of the proposed algorithms are provided in C++, Python, and Matlab.

With respect to the second step, the main contributions and results of the extensive accuracy evaluation are:

1. In contrast to most previous work, the proposed method is extensively evaluated using a large collection of publicly available data and in comparison with eight existing IOE algorithms.

2. The results show that the proposed method outperforms all evaluated existing methods, providing a 1.8- fold to 5-fold increase in orientation estimation accuracy.

3. For a large variety of motions, speeds, and disturbed environments, the proposed method works out of the box, and application-specific parameter tuning is not necessary.

## 2. Proposed Method for Inertial Orientation Estimation

We briefly explain the employed terminology and notation and then, building upon preliminary work in [20], propose a modular method for simultaneous 6D and 9D orientation estimation.

VQF: Highly Accurate IMU Orientation Estimation with Bias Estimation and Magnetic Disturbance Rejection

## 2.1. Terminology and Notation

As illustrated in Fig. 1, the following measurements are available in IOE: gyroscope readings $\omega ( t _ { k } ) ~ \in ~ \mathbb { R } ^ { 3 }$ accelerometer readings $\mathbf { a } ( t _ { k } ) \ \in \ \mathbb { R } ^ { 3 }$ , and magnetometer readings 𝐦 $( t _ { k } ) \ \in \ \mathbb { R } ^ { 3 }$ , sampled at times $t _ { k } ~ = ~ k T _ { \mathrm { s } } , k ~ \in$ $\{ 1 , 2 , \ldots , N \} , T _ { \mathrm { s } } \in \mathbb { R } _ { > 0 }$

If only gyroscopes and accelerometers are employed, we use the term 6D IOE, while 9D IOE additionally uses magnetometers. Therefore, 9D IOE yields the sensor orientation with respect to a fixed inertial reference frame, typically using the east-north-up (ENU) convention (i.e., 𝑧 is pointing up and 𝑦 is pointing north). In contrast, only vertical reference information is available in 6D IOE, and the resulting orientations are thus provided with respect to an almost-inertial reference frame, which has one vertical axis and slowly drifts around this axis (at a rate determined by the gyroscope bias, i.e., typically $\leq 1 ^ { \circ } / \mathrm { s } )$

We denote the moving sensor frame, i.e., the coordinate system in which the sensor readings are provided, by $S _ { i } ( t _ { k } )$ The ENU inertial reference frame used in 9D IOE is denoted $\varepsilon ,$ and the sensor-specific almost-inertial reference frame used in 6D IOE is denoted $\mathcal { E } _ { i } ( t _ { k } )$ ). Anticipating the com mon application scenario with multiple IMUs on diferent segments of a kinematic chain, a sensor index 𝑖 is used for sensor-specific quantities.

We use square brackets to specify the coordinate system in which a vector is expressed, for example, $[ \mathbf { a } ] _ { \varepsilon }$ is the accelerometer measurement transformed into frame . We denote rotations and orientations by unit quaternions [21] in vector notation, i.e., we write the quaternion $w + i x + j y + k z$ as $[ { \textbf { \em w } } \textbf { \em x } \textbf { \em y } \ z ] ^ { \intercal }$ . In the context of quaternion multiplication, which we denote by $\otimes$ , we implicitly regard 3D vectors as pure quaternions. For example, $\mathbf { \bar { \rho } } _ { [ \mathbf { a } ] _ { \varepsilon } } \mathbf { \bar { = } } \mathbf { \rho } _ { \varepsilon } ^ { s } \mathbf { \bar { q } } \otimes \mathbf { a } \otimes _ { \varepsilon } ^ { s } \mathbf { q } ^ { - 1 }$ . Here, the left upper and lower indices denote the frames between which the quaternion rotates. Quaternions that represent the rotation of an angle $\alpha \in \mathbb { R }$ around the axis $\textbf { v } \in \ \mathbb { R } ^ { 3 }$ are written as $\begin{array} { r } { \left( \alpha \circledcirc \mathbf { v } \right) : = \left[ \cos \frac { \alpha } { 2 } \frac { \mathbf { v } ^ { \intercal } } { \| \mathbf { v } \| } \sin \frac { \alpha } { 2 } \right] ^ { \intercal } } \end{array}$

## 2.2. A Modular Estimation Approach

The most fundamental state of any IOE algorithm is the current orientation estimate, which is commonly represented by a single orientation quaternion. For the proposed method, we use a more modular approach and represent the 6D estimate $\begin{array} { c } { { S _ { i } ( t _ { k } ) } } \\ { { \mathcal { E } _ { i } ( t _ { k } ) } } \end{array} { } \mathbf { q }$ as the concatenation of an inclination correction quaternion $\begin{array} { c } { { T _ { i } ( t _ { k } ) } } \\ { { \mathcal { E } _ { i } ( t _ { k } ) } } \end{array} { \bf q }$ with a gyroscope strapdown integration quaternion $\begin{array} { c } { S _ { i } ( t _ { k } ) _ { } } \\ { T _ { i } ( t _ { k } ) ^ { } } \end{array}$ , and the 9D estimate $S _ { i } ( t _ { k } ) _ { \mathcal { E } } \mathbf { \bar { q } }$ as the concatenation of a heading correction rotation $\mathcal { E } _ { i } ( t _ { k } ) _ { \mathfrak { g } }$ represented by the scalar heading ofset $\delta _ { i } ( t _ { k } )$ , with the aforementioned 6D estimate, i.e.,

$$
\underbrace {\mathcal {S} _ {i (t _ {k})} \mathbf {q}} _ {\text {9D estimate}} = \overbrace {\left(\delta_ {i} (t _ {k}) @ [ 0 0 1 ] ^ {\intercal}\right)} ^ {\text {magnetometer correction}} \otimes \underbrace {\overbrace {\mathcal {I} _ {i (t _ {k})} \mathbf {q}} ^ {\text {accelerometer correction}} \otimes \overbrace {\mathcal {I} _ {i (t _ {k})} \mathbf {q}} ^ {\text {strapdown integration}}} _ {\text {6D estimate}}.\tag{1}
$$

![](images/339eaa6294cd0f5e2ccfc2375fa0779a9cc7f7573366c2d68a1069374473b4f7.jpg)  
Figure 2: Illustration of the diferent coordinate systems used by the proposed method. The aim of IOE is to determine the orientation of the sensor $S _ { i }$ relative to an ENU reference frame $\varepsilon$ (in 9D sensor fusion) or relative to a reference frame $\mathcal { E } _ { i }$ with vertical 𝑧-axis (in 6D sensor fusion). The angle $\delta _ { i }$ describes the slowly drifting heading ofset between $\varepsilon$ and $\mathcal { E } _ { i }$ . Internally, the auxiliary $\boldsymbol { \mathscr { T } } _ { i }$ frame is used to represent the orientation obtained by pure gyroscope strapdown integration and slowly drifts due to the integration of gyroscope bias.

The introduced auxiliary coordinate system $\boldsymbol { T } _ { i } ( t _ { k } )$ , with $T _ { i } ( t _ { 0 } ) = S _ { i } ( t _ { 0 } )$ , is an almost-inertial frame that slowly drifts around arbitrary axes due to errors in gyroscope strapdown integration. See Fig. 2 for an illustration of the four distinct coordinate systems that are used in (1).

As demonstrated in [14], a drawback of many existing methods is that magnetic disturbances can severely impact the inclination estimates. While previous methods [13, 14] have already ensured that the magnetometer correction can only influence the heading but not the inclination, the proposed modular state representation makes this property very explicit by representing the heading ofset with a scalar variable $\delta _ { i } ( t _ { k } )$ and facilitates simultaneous 6D and 9D orientation estimation.

The chosen approach of separating strapdown integration, inclination correction, and heading correction in the state is also represented in the filter structure as shown in Fig. 3. Unlike conventional methods, the correction steps are decoupled from the previous steps, i.e., there is no feedback loop from the heading correction to the strapdown integration and, therefore, neither to the inclination correction.

![](images/f5814a15778d60e895dc0b652fdf71da82be0a09857efb681e32eb77dfd3365a.jpg)  
Figure 3: Illustration of conventional and proposed filter structures $\left( z ^ { - 1 } \right)$ denotes the unit delay). The proposed filter structure avoids the feedback of the 9D estimate on the strapdown integration. It thereby enables simultaneous 6D and 9D orientation estimation and ensures that the inclination cannot be influenced by magnetic disturbances.

![](images/109ab728321594ff286b4e53e493985aff1ce6bf861cb504b8ced7788daf4e5e.jpg)  
Figure 4: Variants of the proposed algorithm. BasicVQF consists of strapdown integration, inclination correction, and heading correction. The full version VQF additionally includes rest detection, gyroscope bias estimation, and magnetic disturbance rejection (which can be enabled or disabled independently).

This basic filter structure is extended by an optional gyroscope bias estimation algorithm and an algorithm for magnetic disturbance detection and rejection. The bias esti mation algorithm includes a rest detection and automatically adjusts to whether the IMU is at rest or in motion. The extended filter structure is shown in Fig. 4. Note that it is also possible, and supported by the reference implementation (Section 2.7), to independently enable or disable rest bias es timation, motion bias estimation, and magnetic disturbance rejection.

In the following, we call the extended algorithm VQF (Versatile Quaternion-based Filter) and the basic version BasicVQF. Furthermore, we introduce an acausal imple mentation called OflineVQF in Section 2.6.

## 2.3. Fusion of Gyroscope, Accelerometer, and Magnetometer Measurements

The basic filter update consists of gyroscope-based pre diction, followed by accelerometer correction and, in 9D IOE, by magnetometer correction. The algorithm is given in Algorithm 1, and details on each step are given in Appendix A. Gyroscope prediction is performed via strapdown integration of the measured angular rate. Errors due to gyroscope bias, noise, and other measurement errors lead to slow drift of the $\boldsymbol { \mathit { I } } _ { i }$ frame.

To obtain a vertical reference, we transform the measured accelerations into the almost-inertial frame $\tau _ { i }$ and then apply a second-order Butterworth low-pass filter to each component. This low-pass filter efectively averages the accelerometer measurements and allows for short-term acceleration and deceleration to cancel out, as illustrated in Fig. 5. The inclination of the orientation estimate is then corrected so that the filtered acceleration points in upward direction. In contrast to conventional IOE algorithms, which typically regard each single accelerometer sample as a 3D vector and perform a nonlinear correction step based on the comparison of this vector to a vertical reference vector, the use of a linear low-pass filter in the $\tau _ { i }$ frame more efectively and robustly separates the gravitational acceleration compo nent from the acceleration caused by velocity changes.

![](images/1c0394f5247b9f6673ce3c05716a9297fc4fed55852c13ea1c839dc74c4bdfcf.jpg)  
Figure 5: Example of unfiltered and low-pass filtered accelerations in the original sensor frame $S _ { i }$ and the almostinertial frame $\scriptstyle { \cal T } _ { i } .$ . Applying a low-pass filter with a low cutof frequency to each component in the sensor frame does not give a meaningful output. In the $\boldsymbol { \mathscr { T } } _ { i }$ frame, low-pass filtering each component of the acceleration efectively averages the measurement, allowing for acceleration and deceleration to cancel out, and the result shows the drift of the $\boldsymbol { \mathcal { T } } _ { i }$ frame due to errors in gyroscope integration.

<div class="mineru-algorithm" style="white-space: pre-wrap; font-family:monospace;">
Algorithm 1 BasicVQF
1: procedure INITIALIZEFILTER
2: $S_i$ q ← [1 0 0 0]T ▷ Gyroscope strapdown integration quaternion
3: $I_i$ q ← [1 0 0 0]T ▷ Accelerometer correction quaternion
4: δi ← 0 ▷ Magnetometer correction angle
5: initialize low-pass filter state
6: end procedure
7: procedure FILTERUPDATE(ω = ω(tk), a = a(tk), m = m(tk), fc, acc, kmag, Ts)
8: $S_i$ q ← $S_i$ q ⊗ (Ts ||ω|| @ ω) ▷ Perform gyroscope strapdown integration
9: [a]Ii ← $S_i$ q ⊗ a ⊗ $S_i$ q-1 ▷ Transform acceleration to Ii frame
10: [aLP]Ii ← lpfStep([a]Ii, fc = fc, acc) ▷ Apply low-pass filter
11: [aLP]Ei ← $I_i$ q ⊗ [aLP]Ii ⊗ $I_i$ q-1 ▷ Transform to Ei frame
12: [ax ay az]T ← $[aLP]E_i$ ▷ Normalize
13: qw ← √$\frac{a_z + 1}{2}$
14: $I_i$ q ← [qw $\frac{a_y}{2q_w}$ $-a_x$ 0]T ⊗ $I_i$ q ▷ Update correction quaternion
15: $S_i$ q ← $I_i$ q ⊗ $S_i$ q ▷ Calculate 6D orientation estimate
16: if m is given then
17: [mx my mz]T ← $S_i$ q ⊗ m ⊗ $S_i$ q-1 ▷ Transform mag. sample to Ei frame
18: δmag ← atan2(mx, my) ▷ Calculate heading offset from mag. sample
19: δi ← δi + kmag wrapToPi(δmag - δi) ▷ Update correction angle
20: end if
21: $S_i$ q ← [cos δi/2 0 0 sin δi/2]T ⊗ $S_i$ q ▷ Calculate 9D orientation estimate
22: return $S_i$ q, $S_i$ q ▷ Provide 6D and 9D orientation estimate
23: end procedure
lpfStep: update step of second-order Butterworth low-pass filter with cutoff frequency fc
wrapToPi: bring angle into the interval [-π, π] by adding integer multiples of 2π
</div>

If magnetometer measurements are given, a heading of set is derived from the projection of the magnetic field vector into the horizontal plane and tracked via an exponential filter.

As it is common in IOE algorithms, the behavior can be influenced by fusion weights that balance between rejecting gyroscope drift and rejecting disturbances in the accelerometer and magnetometer measurements. In Algorithm 1, those parameters are the cut-of frequency $f _ { \mathrm { c , a c c } }$ and the magnetometer correction gain $k _ { \mathrm { m a g } }$ . Like for many existing methods, the meaning of the values assigned to those parameters is hard to interpret, depends on the sampling time (for $k _ { \mathrm { m a g } } )$ , and does not allow for a comparison between the trust assigned to the accelerometer and the trust assigned to the magnetometer. To provide a more intuitive parametrization, we replace those parameters with time constants $\tau _ { \mathrm { a c c } }$ and $\tau _ { \mathrm { m a g } }$ that can be changed by the user to influence the algorithm behavior. A small time constant leads to fast correction and indicates high trust in the accelerometer or magnetometer measurements, while large time constants indicate trust in the gyroscope measurements.

Those time constants map to the internal values as follows (see Appendix A and Fig. 17 for more information). For the magnetometer correction first-order exponential filter, we use the time constant that is commonly used to characterize first-order systems and corresponds to the time needed for the step response to reach $1 - e ^ { - 1 } \ \approx \ 6 3 . 2 \%$ of its final value. To ensure similar behavior for the second-order Butterworth filter of the accelerometer correction, we use a time constant that corresponds to the undampened part of the step response. This leads to the mapping

![](images/eb2d74f901f4ed9bb0fec1c187b6f744788c78ad647d95bf4bbdb3077e161cf4.jpg)  
Figure 6: Step response for the Kalman filter with the proposed parametrization. The blue bands show the standard deviation 𝜎 of the estimate. $\mathsf { A t } ~ t < 0$ , the Kalman filter either starts in the initial state $( \sigma = 0 . 5 ^ { \circ } / \mathrm { s } )$ or in a converged state of either the motion or rest update with an estimate of $0 ^ { \circ } / \mathrm { s } .$ . At $t = 0$ , the measurement changes to $1 ^ { \circ } / \mathrm { s }$ . Directly after initialization, the filter converges much faster than in cases where a (contradictory) previous estimate was already obtained. In general, the motion update converges much slower than the rest update.

$$
f _ {\mathrm{c,acc}} = \frac {\sqrt {2}}{2 \pi \tau_ {\mathrm{acc}}}, k _ {\mathrm{mag}} = 1 - \exp \left(- \frac {T _ {\mathrm{s}}}{\tau_ {\mathrm{mag}}}\right),\tag{2}
$$

which allows us to derive the internal parameters $f _ { \mathrm { c , a c c } }$ and $k _ { \mathrm { m a g } }$ from the user-specified time constants $\tau _ { \mathrm { a c c } }$ and $\tau _ { \mathrm { m a g } } .$ The same parametrization via time constants is also used for the other first-order and second-order filters introduced in the following subsections. Note that we will later determine default values for $\tau _ { \mathrm { a c c } }$ and $\tau _ { \mathrm { m a g } }$ that yield excellent out-ofthe-box accuracy for a large range of application scenarios, and manual tuning by adjusting these time constants is therefore only required in rare edge cases.

## 2.4. Gyroscope Bias Estimation

To ensure high accuracy in the presence of gyroscope bias, we extend the BasicVQF algorithm from Section 2.3 by a method to estimate and compensate such bias. In existing IOE algorithms, this is commonly realized by integral action [14, 16, 17]. However, this approach requires feedback of both accelerometer and magnetometer correction [17], making the gyroscope bias estimate susceptible to magnetic disturbances. To prevent this, the proposed method for gyroscope bias estimation avoids using any information from the magnetometer correction. Instead, the bias is estimated solely from the disagreement between strapdown integration and accelerometer measurements during motion.

Moreover, note that, in many application scenarios, the IMU will occasionally be at rest for several seconds, and those phases can be detected reliably. This is leveraged by the proposed bias estimation algorithm, which determines the bias directly from low-pass-filtered gyroscope measurements whenever it detects a period of rest.

As further detailed in Appendix C and Algorithm 2, the bias estimation is realized via a single Kalman filter that performs diferent updates based on whether the IMU is currently at rest or in motion. This Kalman filter is parametrized in a way that is independent of the sampling rate and that gives much larger trust to the rest-based updates than to the motion-based updates. In each sampling step, the current Kalman filter estimate of the bias is subtracted from the gyroscope measurement before strapdown integration.

Fig. 6 shows how the Kalman filter behaves with the proposed parametrization. The relation between the uncertainty of the measurement (large during motion, small during rest) and the uncertainty of the current estimate determines how fast the bias estimation converges. After initialization without prior knowledge, the estimation uncertainty is large, which leads to fast convergence. From a converged estimate during motion (i.e., with medium uncertainty), a contradicting but much more reliable observation during rest is adopted within several seconds. From a converged rest estimate, a contradicting observation during motion is adopted much slower due to the larger uncertainty of the measurement.

Table 1  
Inertial orientation estimation algorithms used in the evaluation

<table><tr><td colspan="2">Algorithm</td><td>6D</td><td>9D</td><td>Bias</td><td>Source</td></tr><tr><td>VQF</td><td>proposed method</td><td>✓</td><td>✓</td><td>✓</td><td>https://github.com/dlaidig/vqf</td></tr><tr><td>MAH</td><td>Mahony et al. [16]</td><td>✓</td><td>✓</td><td>✓</td><td>https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/</td></tr><tr><td>MAD</td><td>Madgwick [17]</td><td>✓</td><td>✓</td><td>✗</td><td>https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/</td></tr><tr><td>VAC</td><td>Valenti et al. [13]</td><td>✓</td><td>✓</td><td>✓</td><td>https://wiki.ros.org/imu_complementary_filter</td></tr><tr><td>FKF</td><td>Guo et al. [22]</td><td>✗</td><td>✓</td><td>✗</td><td>https://github.com/zarathustr/FKF</td></tr><tr><td>SEL</td><td>Seel and Ruppin [14]</td><td>✓</td><td>✓</td><td>✓</td><td>https://github.com/dlaidig/qmt, qmt.oriEstIMU</td></tr><tr><td>MKF</td><td>Matlab</td><td>✓</td><td>✓</td><td>✓</td><td>Matlab R2021b (The MathWorks Inc., Natick, MA, USA), imufilter/ahrsfilter</td></tr><tr><td>KOK</td><td>Kok and Schön [15]</td><td>✗</td><td>✓</td><td>✓</td><td>https://github.com/manonkok/fastRobustOriEst</td></tr><tr><td>RIANN</td><td>Weber et al. [23]</td><td>✓</td><td>✗</td><td>✗</td><td>https://github.com/daniel-om-weber/riann</td></tr></table>

## 2.5. Magnetic Disturbance Rejection

We extend the proposed IOE algorithm with a set of methods that enable adaptive filtering of the magnetometer measurements with the aim of reducing the influence of temporary magnetic disturbances. The employed strategy is composed of three parts: magnetic disturbance detection, magnetic disturbance rejection, and new magnetic field acceptance. All three parts are briefly explained below, and the complete algorithm is described in detail in Algorithm 3 in Appendix F.

The magnetic disturbance detection uses a user-defined or automatically determined reference for the norm and dip angle of the local magnetic field. The magnetometer measurements are considered to be undisturbed only if they have been close to the reference for at least 0.5 s – and disturbed otherwise. Whenever the magnetic field is considered undisturbed, the reference values are slowly updated to track slow changes in the norm and dip angle.

The magnetic disturbance rejection adjusts the speed of the first-order filter used for heading correction if magnetic disturbances are detected. For disturbances of up to 60 s, the magnetometer update is fully disabled. For longer periods that are considered to be disturbed, updates are performed but at a lower speed.

Finally, the new magnetic field acceptance is used to deal with sudden changes in the environment, e.g., after changing the terrain from outdoor to indoor or moving to a diferent indoor room with a diferent local magnetic field. Whenever the magnetic field is considered to be disturbed but seems homogeneous (for a suficiently long time during which the IMU was not stationary), the norm and dip angle of the current magnetic field is used as the new reference.

## 2.6. Acausal Filtering for Ofline Data Processing

In application scenarios in which the complete time series of recorded data is available (ofline data processing), we can employ acausal signal processing methods to further improve accuracy. As commonly done in signal processing for zero-phase filtering [24], we first run the filtering steps forward and then again backward in time. This allows us to leverage the existing real-time implementation to create the ofline variant detailed in Algorithm 4 in Appendix G.

## 2.7. Open-Source Implementation

Implementations of the proposed orientation estimation algorithm are available at https://github.com/dlaidig/vqf under the MIT license. Native implementations are provided in C++, Python, and Matlab. Furthermore, the fast C++ implementation can easily be used from Python code. The Python package is available at https://pypi.org/project/ vqf/ and can be installed via pip, and documentation is available at https://vqf.readthedocs.io/.

## 3. Evaluation

In this section, we evaluate the performance of the proposed IOE algorithm on six publicly available datasets and compare the results obtained with the proposed method to results obtained with eight state-of-the-art IOE algorithms.

## 3.1. Algorithms and Datasets

Table 1 lists the algorithms used for the evaluation. This table also shows whether the algorithm can be used for 6D and/or 9D orientation estimation, whether it supports gyroscope bias estimation, and where the implementation is available.

To evaluate the accuracy of IOE, we consider publicly available datasets consisting of IMU measurements and a ground truth orientation obtained from marker-based optical motion capture (OMC). In [19], we presented the Berlin Robust Orientation Estimation Assessment Dataset (BROAD) and briefly reviewed other existing datasets that contain trial data that is suitable for IOE accuracy evaluation. In the present evaluation, we use all of these datasets, i.e.,

• BROAD [19]: 39 trials (23 undisturbed trials with diferent motion types and speeds and 16 trials with various deliberate disturbances)

• Sassari [18]: 18 trials (3 speeds, 3 IMU models, and 2 IMUs of each model)

![](images/ee8af566840a8669fbcfa210500d5a8c95d2c2ea1f2fa6c5eb9171c4caf7b2a1.jpg)

![](images/4396220e5877c7a3a18c1931143e74194bd97f2dfcf33b50e0fb0cdacfae14f8.jpg)  
Figure 7: RMSE (weighted average over all datasets) achieved with the proposed VQF algorithm and with the reduced BasicVQF variant, for diferent values of the tuning parameters. The default algorithm parameters are chosen such that the mean of both errors is minimized, i.e., $\tau _ { \mathrm { a c c } } = 3 \ : \mathrm { s }$ and $\tau _ { \mathrm { m a g } } = 9 \ : \mathrm { s }$ .  
Figure 8: RMSE (weighted average over all datasets) for the proposed VQF and all state-of-the-art algorithms. VQF outperforms all eight literature methods, and only RIANN provides similar 6D performance. Even the errors obtained with the simple BasicVQF variant are clearly lower than for the other seven algorithms. When real-time capability is not required, using the ofline variant is advised to further increase the accuracy of the orientation estimates.

• RepoIMU [25]: 21 trials (T-Stick only; test 5, test 6 trial 1, test 10 excluded due to artifacts, as explained in [19])

• OxIOD [26]: 71 trials (only handbag, handheld, pocket, running, slow walking, trolley trials)

• TUM VI [27]: 6 trials (room only; no magnetometer data)

• EuRoC MAV [28]: 6 trials (Vicon room only; no magnetometer data).

Combined, the collection of evaluation data consists of 161 trials with a total duration of 12.9 h. The data includes motions of handheld IMUs (various combinations of fast and slow rotations and translations), walking and running, as well as flight data from a micro aerial vehicle, and contains data from eight diferent IMU models, recorded at sampling rates ranging from 100 Hz to 286 Hz. For more information about the datasets, refer to the respective publications and the summary provided in [19]. This large collection of experimental data allows us to evaluate the robustness of the proposed method for diferent motion characteristics, diferent sensor hardware, and diferent sampling rates.

## 3.2. Algorithm Parametrization

Before assessing the IOE accuracy, we need to determine suitable tuning parameters for each algorithm. In [19], we introduced a metric to assess the performance of an IOE algorithm, the trial-agnostic generalized performance (TAGP). This metric is defined as the smallest possible root mean square error (RMSE), averaged over all 39 trials of the BROAD dataset, that can be obtained with a common algorithm parameterization for all trials. The associated parameterization can then be expected to provide good results for a wide variety of motions and disturbance scenarios.

However, one limitation of the BROAD dataset is that all trials are performed with the same IMU model. To find parameters that are not only robust against movement speed, type of motion, and various disturbances, but also work well for diferent sensor characteristics, we define an extended TAGP metric, the TAGP x

![](images/d896333d588d8bfe7fca7e5619f94149d06b49e26b5025ee0ba25d7dda2c64b9.jpg)

![](images/98e3da0d352a489835e67b8e4b08bb55697cdc2dffe172b9723f660f9d3986a7.jpg)

![](images/4c96e719c22b2dbd4b73dd81d41e9d00e476518d0605f6acc8c98c68d02a9c7d.jpg)

![](images/5fffb8f6d9d7d74e17a905b4aa494118d3a549a639f8b796447d292b8349aa18.jpg)  
Figure 9: Orientation estimation errors for all evaluated algorithms and for all trials of the (a,b) BROAD dataset and (c,d) the five other datasets and for (a,c) 6D and (b,d) 9D sensor fusion. The numbers below the algorithm names indicate the RMSE averaged over all trials. For (b,d), the boxplots and average values are weighted to give each dataset the same weight regardless of the number of trials. The proposed VQF algorithm consistently provides the best performance.

Similar to the TAGP, the $\mathrm { T A G P _ { x } }$ is the smallest possible RMSE, averaged over the aforementioned trials for each of the six datasets. The errors are first averaged by dataset, where the dedicated benchmark dataset BROAD is given a five times larger weight than all other datasets, which are weighted equally. For trials without magnetometer data (and for RIANN, which does not support magnetometers), the inclination error, as defined in [19], is used instead of the orientation error.

Fig. 7 shows how the weighted error defined above depends on the tuning parameters for the default and the basic variant of the proposed VQF algorithm. The $\mathrm { T A G P _ { x } }$ is the minimum value of this error, i.e., 2.59° for VQF and $3 . 4 1 ^ { \circ }$ for BasicVQF, which shows that gyroscope bias estimation and magnetic disturbance rejection lead to improved accuracy. However, the optimal values for the time constants $\tau _ { \mathrm { a c c } }$ and $\tau _ { \mathrm { m a g } }$ are similar for both variants. To avoid specifying diferent default parameters for VQF and

![](images/09bb9d07ef51f70241e9374b4ca2357e96438474fc2c25e3f09533545b6d9a92.jpg)  
Figure 10: Averaged RMSE errors for various groups of trials of the BROAD dataset. The proposed algorithm VQF is compared with the best of the seven other evaluated algorithms, i.e., the algorithm that provides the lowest orientation error for the respective group of trials. The lines originating from the center highlight the diference between the errors. For all groups except for the tapping and vibration trials, the proposed algorithm outperforms even the best-performing literature method.

BasicVQF, we simply use the average of the errors obtained with both variants to determine the default values $\tau _ { \mathrm { a c c } } = 3 \ : \mathrm { s }$ 2 and $\tau _ { \mathrm { a c c } } = 9 \mathrm { s } .$

To provide a fair comparison between the proposed and the state-of-the-art methods, we also optimize the parameters for all other algorithms according to the $\mathrm { T A G P _ { x } } ,$ , i.e., we find the parameters that allow each algorithm to provide the best possible performance across all datasets. Details on the employed search strategy as well as the resulting parameters are given in Appendix H. In the following, we always use those optimal parameters to evaluate and compare algorithm performance.

## 3.3. Orientation Estimation Accuracy

To assess the performance of all algorithms, we apply each to the data of all trials of all datasets. In the case of 6D (magnetometer-free) sensor fusion, we determine the incli nation error [19], and for 9D sensor fusion the orientation error, in each case between the IMU-based estimate and the OMC ground truth. We then calculate the RMSE while only considering the motion phases. Since the TUM VI and EuRoC MAV datasets do not include magnetometer data, they are only considered for the 6D results. We average the errors by dataset analogously to the definition of the TAGP in Section 3.2. The resulting values are presented for all algorithms in Fig. 8.

One main observation from this figure is that the proposed method VQF consistently provides considerably lower errors than the existing orientation estimation algorithms. For 9D IOE, there is a 1.8-fold to 5-fold increase in accuracy, while for 6D IOE, the increase is 17 % for RIANN and between 2.1-fold and 5-fold for the other methods. It should be noted that the only algorithm that achieves similar inclination errors, the neural network RIANN, cannot perform 9D sensor fusion and was trained on some of the datasets that are used in this evaluation (see [23] for details).

Fig. 8 also allows us to compare the variants of the proposed method, cf. Fig. 4. Unsurprisingly, the errors obtained with the BasicVQF variant (no bias estimation and no magnetic disturbance rejection) are slightly larger. However, with the aforementioned exception of RIANN, even the 6D and 9D errors of BasicVQF are still clearly lower than the corresponding errors obtained with any of the existing algorithms. The low 6D errors of BasicVQF can directly be attributed to the novel approach for inclination correction, while the comparatively small diference between 6D and 9D errors can be attributed to the decoupled filter structure and modular state representation. Compared to the real-timecapable implementation, the ofline variant OflineVQF is able to further increase the estimation accuracy by another 20 percent. Therefore, employing this variant is advisable when analyzing recorded data.

![](images/8ad7f4139a20f945a680780a369e2df46e9d3247c823a4a1fda1f4c7e84bfb5d.jpg)

![](images/c70d7f0f07037811a6dfda46722cdf5d801074e67fe6e6e61ed32e9bd525d391.jpg)  
Figure 11: Execution time for one update step on an AMD Ryzen 5 3600 CPU vs. orientation estimation RMSE (weighted average over all datasets) for the proposed VQF and all state-of-the-art algorithms. Accuracies vary largely, and execution times mainly depend on the programming language used for implementing the algorithm. The execution time of VQF is in the same order of magnitude as for the other algorithms implemented in C++, while the errors are clearly smaller.

While Fig. 8 shows a clear improvement in accuracy with respect to the state of the art when looking at errors averaged over a large number of trials, a comprehensive comparison should also include a closer look at individual trials. Fig. 9 diferentiates the errors by dataset and shows a marker for each single trial. Comparing medians, the interquartile ranges, the lengths of the whiskers, or the distributions of outliers yields the same conclusion: The proposed method not only performs better on average but consistently and robustly provides lower errors than the existing methods.

It is noticeable that unusually large errors of ∼ 23° are observed for some trials of the OxIOD dataset. As those large errors are observed across all methods, the most likely cause of those errors are irregularities in the measurement or ground truth data that have previously been noted in [19].

Beyond the level of comparing performance on diferent datasets, the BROAD dataset facilitates the investigation of algorithm performance for seven diferent motion characteristics and six diferent disturbance characteristics. Fig. 10 compares the average RMSE across diferent groups of trials and compares the proposed VQF method in each case with the best of the other 9D-capable algorithms, i.e., the algorithm that achieves the lowest errors for the respective group of trials. Except for the tapping and vibration groups, VQF always achieves lower errors than even the best of the other algorithms. For the vibration group, it is worth noticing that, while the orientation error is slightly larger than the error obtained with FKF, the inclination errors obtained with VQF are clearly lower.

In summary, compared with eight other IOE algorithms and using a collection of six publicly available datasets that cover a wide range of motions, speeds, disturbances, and diferent sensor hardware, the proposed method VQF consistently provides the best IOE accuracy, both for 6D and 9D orientation estimation.

## 3.4. Algorithm Execution Time

In addition to accuracy, the execution time of an IOE algorithm is often relevant, especially in real-time applications or when the algorithm is running on low-powered microcontrollers directly on the IMU. To compare the execution times, we repeatedly process the entire BROAD dataset with all algorithms on an AMD Ryzen 5 3600 CPU while measuring the execution time. Fig. 11 shows the average execution time for one update step in combination with the orientation estimation error for the respective algorithm. The results show that execution time mostly depends on the programming language used for the implementation and that the algorithms written in C++ are considerably faster than the algorithms written in Matlab or using the ONNX machine learning runtime. While the VQF algorithms achieve clearly higher accuracy, the execution times are in the same order of magnitude as for the existing state-of-the-art methods with a C++ implementation. VQF is fast enough for use on microcontrollers, which we verified by integrating it into an IMU firmware running on a Cortex M4 at a comparatively high sampling rate of 1600 Hz.

## 3.5. Gyroscope Bias Estimation

We now investigate the performance of the gyroscope bias estimation method. Besides the proposed algorithm, we also evaluate the performance of the five other algorithms that are able to estimate gyroscope bias.

For the BROAD and Sassari datasets, we derive a ground truth for the gyroscope bias by averaging the gyroscope measurements during the rest phases at the beginning and at the end of each trial and linearly interpolating in between to account for slow bias instability. For each IOE algorithm, we calculate the root-mean-square over time of the residual bias norm, i.e., of the norm of the diference between the estimated bias and the true bias. Fig. 12 shows the achieved relative reduction of the gyroscope bias, the true bias norm, and the residual bias norm, averaged over all trials for each dataset. Since the proposed VQF and the literature method VAC use rest detection for bias estimation, while the other algorithms do not, we also test the performance of all algorithms on a cut version of the BROAD dataset, in which the initial and final rest phases were removed.

VQF: Highly Accurate IMU Orientation Estimation with Bias Estimation and Magnetic Disturbance Rejection

<table><tr><td></td><td>BROAD</td><td>true bias: 0.332°/s</td><td>BROAD (cut)</td><td>true bias: 0.332°/s</td><td>Sassari</td><td>true bias: 0.855°/s</td><td>Parameters</td></tr><tr><td>VQF</td><td>90%</td><td>0.033°/s</td><td>39%</td><td>0.204°/s</td><td>89%</td><td>0.092°/s</td><td>default</td></tr><tr><td>OfflineVQF</td><td>97%</td><td>0.008°/s</td><td>58%</td><td>0.138°/s</td><td>96%</td><td>0.037°/s</td><td>default</td></tr><tr><td>MAH</td><td>7%</td><td>0.309°/s</td><td>1%</td><td>0.329°/s</td><td>30%</td><td>0.601°/s</td><td>optimized</td></tr><tr><td>MAD</td><td>17%</td><td>0.276°/s</td><td>3%</td><td>0.321°/s</td><td>33%</td><td>0.572°/s</td><td>optimized</td></tr><tr><td>VAC</td><td>79%</td><td>0.069°/s</td><td>18%</td><td>0.274°/s</td><td>39%</td><td>0.522°/s</td><td>optimized</td></tr><tr><td>SEL</td><td>51%</td><td>0.162°/s</td><td>6%</td><td>0.312°/s</td><td>67%</td><td>0.280°/s</td><td>optimized</td></tr><tr><td>MKF</td><td>0%</td><td>1.021°/s</td><td>0%</td><td>1.094°/s</td><td>10%</td><td>0.773°/s</td><td rowspan="2">TAGPx optimized</td></tr><tr><td>KOK</td><td>15%</td><td>0.283°/s</td><td>3%</td><td>0.323°/s</td><td>34%</td><td>0.567°/s</td></tr><tr><td></td><td>bias reduction</td><td>residual bias</td><td>bias reduction</td><td>residual bias</td><td>bias reduction</td><td>residual bias</td><td></td></tr></table>

Figure 12: Bias estimation results for different IOE algorithms. Green bars and percentages indicate the reduction of the bias norm with respect to the true bias (black) contained in the measurement data. For all datasets, VQF with default parameters surpasses the best possible performance of the existing algorithms. Using the OflineVQF variant further improves accuracy.

gyroscope measurements 𝝎 [°∕s]  
![](images/b101edf2b8a2e708fd844ad7a334cd7a6acf1fcc04e77c0d028f918c4feed99f.jpg)

estimated bias ̂𝐛 and ground truth 𝐛 [°∕s]  
![](images/9e7995782747a29937fc80f89e48b6c99906fe5ff5342bb075492c045ca88677.jpg)

estimation error ̂𝐛 − 𝐛 and uncertainty 𝜎 [°∕s]  
![](images/019cdb8b8cd3c17b795c4a272d25ee51c472453d9fae5e89a1b7f7e0d3e1a462.jpg)  
Figure 13: Behavior of the bias estimation method for the first 60 seconds of trial 4 of the BROAD dataset (undisturbed slow rotation with breaks, initial rest cut). During motion, the estimated bias slowly converges toward the true value. Once rest is automatically detected by Algorithm 2, the convergence speed increases, and the error and the estimation uncertainty suddenly drop.

For the proposed algorithms VQF and OflineVQF, we present the bias estimates obtained with the default parameters. For the existing methods, we found that parameters that yield the best orientation estimation results often do not yield the best bias estimates. We therefore optimized, separately for each dataset, all parameters of all literature methods (except MKF) across the search grid presented in Table 2, such that the bias estimation error is minimized. Despite this disparity, the proposed method clearly outperforms the bias estimation methods of all other IOE algorithms. Even though the results obtained with the cut BROAD dataset are worse than for the datasets with long rest phases, the proposed method is still able to reduce the bias by 39 %, while the best literature method only achieves a reduction of 18 %. As with orientation estimation, using the OflineVQF variant further improves the accuracy in comparison to the realtime capable VQF algorithm. MKF only achieves a slight reduction of gyroscope bias for the Sassari dataset, while for BROAD the bias norm increases compared to the original bias found in the measurement data.

magnetometer measurements 𝐦 [µT]  
![](images/c6db457b801d9dc12ea2d6c1c603602cf784940af5e005a78f587c99b8c734c2.jpg)

orientation error with and without magnetic disturbance rejection [°] mag. dist. detected by VQF  
![](images/61478c567f9818aaec0722fd298b971576bc09a09bb22ab3ece2349135b26369.jpg)

true heading disturbance of the local magnetic field (derived with OMC data) [°]  
![](images/ec9837a3ddbfc6e7bfb1b4bfb7d7a2494ce92a3e1187ffdd36704536f824449e.jpg)  
Figure 14: Performance of the magnetic disturbance rejection method for trial 37 of the BROAD dataset (disturbed ofice environment). Whenever magnetic disturbances are detected by Algorithm 3 (light red background), the magnetometer-based correction is automatically disabled. In comparison to the same algorithm with disabled magnetic disturbance rejection, the RMSE is considerably reduced, and the large error peak of 20° is avoided.

To illustrate how the bias estimation method works, Fig. 13 shows the estimated bias, the estimation error, and the estimation uncertainty for an exemplary trial of the BROAD dataset. It can be seen that the bias estimation works as intended: During the initial motion phase, the estimated bias slowly converges to the true bias. Once rest is automatically detected, the alternative update is used, causing the estimate to rapidly converge.

In summary, both in a systematic comparison, as well as in an exemplary case study, the proposed gyroscope bias estimation method was found to work reliably and clearly outperformed existing bias estimation approaches.

## 3.6. Magnetic Disturbance Rejection

To further illustrate the diference in performance between the proposed BasicVQF and VQF algorithms, we now take a brief look at the performance of the magnetic disturbance detection and rejection method. Fig. 14 shows the behavior of this extension on an example of the BROAD dataset. For the sake of evaluation, OMC data was used to determine the true disturbance of the local magnetic field caused by ferromagnetic material and electric devices in the ofice environment. This ground truth information shows that the detection is triggered whenever disturbances are present. Without magnetic disturbance rejection, the orientation estimation error reaches a maximum of 20.0°, and the

RMSE is 6.1°. In contrast, enabling magnetic disturbance rejection reduces the maximum error to just 3.7° and the RMSE to 1.8°, which translates to an at least three times better accuracy. This example demonstrates how the optional magnetic disturbance rejection can improve the reliability of 9D IOE in real-world scenarios inside buildings, near ferromagnetic material and electric devices.

## 3.7. Summary of the Results

The proposed VQF algorithm achieved an average RMSE of 2.9° for 9D IOE, while the average errors obtained with state-of-the-art methods range from 5.3° to 16.7°. For 6D IOE, VQF attained an average RMSE of 1.1°, compared to 2.4°–6.3° obtained with existing methods, and it achieved even 17 % lower errors than a neural network that was trained on large portions of the benchmark data. For the 13 characteristic trial groups of the BROAD dataset, the proposed method outperforms even the best-performing literature method for all seven motion characteristics and for four out of six disturbance characteristics. Furthermore, the gyroscope bias estimation of VQF clearly outperformed all existing state-of-the-art literature methods and compensated ∼ 90 % of the bias. Even for the challenging case without rest phases, the bias could still be reduced by ∼40–60 %, while the existing algorithms barely achieved any bias reduction. For an exemplary case in a simulated ofice environment, the magnetic disturbance rejection algorithm was shown to achieve a five-fold reduction of the maximum orientation error.

Even the variant BasicVQF, without bias estimation and magnetic disturbance rejection, was shown to provide clearly more accurate 9D orientation estimates than all state-of-the art methods. For applications in which real-time capability is not required, the variant OflineVQF can be used to further increase accuracy.

## 4. Conclusions

We proposed a novel IOE algorithm that simultaneously performs 6D and 9D sensor fusion, estimates gyroscope bias, and performs magnetic disturbance detection and rejec tion. An open-source implementation is provided in C++, Python, and Matlab, making it easy to use the algorithm. We compared the proposed algorithm VQF with eight other IOE algorithms and using a collection of six publicly avail able datasets that cover a wide range of motions, speeds, disturbances, and diferent sensor hardware. As summarized in Section 3.7, VQF consistently provided the best performance, both for 6D and 9D orientation estimation, as well as for gyroscope bias estimation, and it proved capable of magnetic disturbance rejection.

The proposed method provides a highly accurate out-of the-box performance, which means that – unlike existing literature methods – VQF requires no parameter tuning for a vast range of motions and application scenarios. For rare edge cases, the proposed method facilitates easy and intuitive tuning via the time constants $\tau _ { \mathrm { a c c } }$ and $\tau _ { \mathrm { m a g } }$

The achieved improvements in ease of use and in orien tation estimation accuracy are expected to change the way we use IOE algorithms in practice and, thereby, to advance the broad field of inertial motion tracking since it enables more accurate IMU-based position and velocity estimation, joint angle estimation, and 3D visualization. This, in turn, leads to improved performance in many existing application areas of miniature inertial sensor technology, and it likewise facilitates the applicability in novel application domains with increased accuracy demands.

Future work will focus on integrating continuous and automatic magnetometer calibration as well as on employing the VQF algorithm in various applications.

## CRediT authorship contribution statement

Daniel Laidig: Conceptualization, Methodology, Software, Validation, Formal analysis, Investigation, Data cura tion, Writing – original draft, Writing – review & editing, Visualization. Thomas Seel: Conceptualization, Methodol ogy, Formal analysis, Writing - review & editing, Supervi sion.

## References

[1] McGinnis, R.S., Perkins, N.C., 2012. A highly miniaturized, wireless inertial measurement unit for characterizing the dynamics of pitched baseballs and softballs. Sensors 12, 11933–11945. doi:10.3390/ s120911933.

[2] Laidig, D., Jocham, A.J., Guggenberger, B., Adamer, K., Fischer, M., Seel, T., 2021. Calibration-free gait assessment by foot-worn inertial sensors. Frontiers in Digital Health 3, 147. doi: 736418.

[3] Wang, Z., Zhao, H., Qiu, S., Gao, Q., 2015. Stance-phase detection for ZUPT-aided foot-mounted pedestrian navigation system. IEEE/ASME Transactions on Mechatronics 20, 3170–3181. doi:10. 1109/TMECH.2015.2430357.

[4] Nguyen, H., Lebel, K., Bogard, S., Goubault, E., Boissy, P., Duval, C., 2018. Using inertial sensors to automatically detect and segment activities of daily living in people with Parkinson’s disease. IEEE Transactions on Neural Systems and Rehabilitation Engineering 26, 197–204. doi:10.1109/TNSRE.2017.2745418.

[5] Passon, A., Schauer, T., Seel, T., 2020. Inertial-robotic motion tracking in end-efector-based rehabilitation robots. Frontiers in Robotics and AI 7. doi:10.3389/frobt.2020.554639.

[6] Rodrigo Marco, V., Kalkkuhl, J., Raisch, J., Scholte, W.J., Nijmeijer, H., Seel, T., 2020. Multi-modal sensor fusion for highly accurate vehicle motion state estimation. Control Engineering Practice 100, 104409. doi:10.1016/j.conengprac.2020.104409.

[7] Chao, H., Cao, Y., Chen, Y., 2010. Autopilots for small unmanned aerial vehicles: A survey. International Journal of Control, Automation and Systems 8, 36–44. doi:10.1007/s12555-010-0105-z.

[8] Freter, J.H., Seel, T., Elfert, C., Göhlich, D., 2020. Motion estimation for tethered airfoils with tether sag, in: 2020 IEEE International Conference on Multisensor Fusion and Integration for Intelligent Systems (MFI), pp. 114–120. doi:10.1109/MFI49285.2020.9235235.

[9] Woodman, O.J., 2007. An Introduction to Inertial Navigation. Technical Report UCAM-CL-TR-696. University of Cambridge, Computer Laboratory. doi:10.48456/tr-696.

[10] Kok, M., Hol, J.D., Schön, T.B., 2017. Using inertial sensors for position and orientation estimation. Foundations and Trends® in Signal Processing 11, 1–153. doi: , .

[11] Nazarahari, M., Rouhani, H., 2020. 40 years of sensor fusion for orientation tracking via magnetic and inertial measurement units: Methods, lessons learned, and future challenges. Information Fusion doi:10.1016/j.inffus.2020.10.018.

[12] Caruso, M., Sabatini, A.M., Laidig, D., Seel, T., Knaflitz, M., Della Croce, U., Cereatti, A., 2021. Analysis of the accuracy of ten algorithms for orientation estimation using inertial and magnetic sensing under optimal conditions: One size does not fit all. Sensors 21, 2543. doi:10.3390/s21072543.

[13] Valenti, R.G., Dryanovski, I., Xiao, J., 2015. Keeping a good attitude: A quaternion-based orientation filter for IMUs and MARGs. Sensors 15, 19302–19330. doi: .

[14] Seel, T., Ruppin, S., 2017. Eliminating the efect of magnetic disturbances on the inclination estimates of inertial sensors. IFAC-PapersOnLine 50, 8798–8803. doi:10.1016/j.ifacol.2017.08.1534.

[15] Kok, M., Schön, T., 2019. A fast and robust algorithm for orientation estimation using inertial sensors. IEEE Signal Processing Letters PP, 1–1. doi: .

[16] Mahony, R., Hamel, T., Pflimlin, J.M., 2008. Nonlinear complementary filters on the special orthogonal group. IEEE Transactions on Automatic Control 53, 1203–1218. doi: .

[17] Madgwick, S.O.H., 2010. An eficient orientation filter for inertial and inertial/magnetic sensor arrays , 32.

[18] Caruso, M., Sabatini, A.M., Knaflitz, M., Gazzoni, M., Croce, U.D., Cereatti, A., 2021. Orientation estimation through magneto-inertial sensor fusion: A heuristic approach for suboptimal parameters tuning. IEEE Sensors Journal 21, 3408–3419. doi: 3024806.

[19] Laidig, D., Caruso, M., Cereatti, A., Seel, T., 2021. BROAD—A benchmark for robust inertial orientation estimation. Data 6, article 72. doi:10.3390/data6070072.

[20] Laidig, D., Weygers, I., Bachhuber, S., Seel, T., 2022. VQF: A milestone in accuracy and versatility of 6D and 9D inertial orientation estimation, in: 2022 25th International Conference on Information Fusion (FUSION), pp. 1–6. doi:10.23919/FUSION49751.2022.9841356.

[21] Kuipers, J.B., 1999. Quaternions and rotation sequences, in: Proceedings of the International Conference on Geometry, Integrability and Quantization, Coral Press Scientific Publishing, Varna, Bulgaria. pp. 127–143. doi:10.7546/giq-1-2000-127-143.

[22] Guo, S., Wu, J., Wang, Z., Qian, J., 2017. Novel MARG-sensor orientation estimation algorithm using fast Kalman filter. Journal of Sensors 2017, e8542153. doi:10.1155/2017/8542153.

[23] Weber, D., Gühmann, C., Seel, T., 2021. RIANN—A robust neural network outperforms attitude estimation filters. AI 2, 444–463. doi:10.3390/ai2030028.

[24] Gustafsson, F., 1996. Determining the initial states in forward backward filtering. IEEE Transactions on Signal Processing 44, 988– 992. doi:10.1109/78.492552.

[25] Szczęsna, A., Skurowski, P., Pruszowski, P., Pęszor, D., Paszkuta, M., Wojciechowski, K., 2016. Reference data set for accuracy evaluation of orientation estimation algorithms for inertial motion capture sys tems, in: Chmielewski, L.J., Datta, A., Kozera, R., Wojciechowski, K. (Eds.), Computer Vision and Graphics, Springer International Publishing, Cham. pp. 509–520. doi:10.1007/978-3-319-46418-3\_45.

[26] Chen, C., Zhao, P., Lu, C.X., Wang, W., Markham, A., Trigoni, N., 2018. OxIOD: The dataset for deep inertial odometry. arXiv:1809.07491 [cs] arXiv:1809.07491.

[27] Schubert, D., Goll, T., Demmel, N., Usenko, V., Stückler, J., Cremers, D., 2018. The TUM VI benchmark for evaluating visual-inertial odometry, in: 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Madrid, Spain. pp. 1680–1687. doi:10. 1109/IROS.2018.8593419.

[28] Burri, M., Nikolic, J., Gohl, P., Schneider, T., Rehder, J., Omari, S., Achtelik, M.W., Siegwart, R., 2016. The EuRoC micro aerial vehicle datasets. The International Journal of Robotics Research 35, 1157– 1163. doi:10.1177/0278364915620033.

[29] Kalman, R.E., 1960. A new approach to linear filtering and prediction problems. Journal of Basic Engineering 82, 35–45. doi:10.1115/1. 3662552.

[30] Welch, G., Bishop, G., 2006. An Introduction to the Kalman Filter. Technical Report TR 95-041. University of North Carolina at Chapel Hill.

## A. Details on the Basic Update Step

The basic filter update given in Algorithm 1 consists of a prediction of the next orientation based on the gyroscope measurement, followed by a correction of the inclination based on the accelerometer measurement, and an (optional) correction of the heading based on the magnetometer mea surement.

## A.1. Gyroscope Prediction

Gyroscope prediction via strapdown integration is per formed by multiplying the previous estimate with a quater nion based on the norm and direction of the measured angular rate:

$$
\begin{array}{c} S _ {i} (t _ {k}) \\ I _ {i} (t _ {k}) \end{array} \mathbf {q} = \begin{array}{c} S _ {i} (t _ {k - 1}) \\ I _ {i} (t _ {k - 1}) \end{array} \mathbf {q} \otimes \left(T _ {s} \| \boldsymbol {\omega} \| @ \boldsymbol {\omega}\right).\tag{3}
$$

The rotation due to the gyroscope measurement is composed of the true change of sensor orientation and an error, due to gyroscope bias, noise, and other measurement errors $( \mathrm { e . g . }$ , scaling errors, nonlinearity, misalignment, and clip ping). This error can be regarded as a small drift in the $\tau _ { i }$ frame, i.e., as $\begin{array} { r } { T _ { i } ( t _ { k - 1 } ) _ { } } \\ { T _ { i } ( t _ { k } ) ^ { } } \end{array}$ , which can be shown via quaternion algebra (cf. Appendix B).

## A.2. Accelerometer Correction

The accelerometer measurements consist of the gravita tional acceleration, change of velocity, as well as noise, bias, and other measurement errors. Most existing methods [17, 16, 14] interpret each single accelerometer sample as a 3D vector and use the angle between this vector and the expected vertical direction to derive the correction step. In contrast, to better separate the gravitational acceleration from the other components of the measurement, we transform the measured accelerations into the almost-inertial frame $\tau _ { i }$ and then apply a linear low-pass filter to each component. The resulting signal provides a vertical reference in the $\boldsymbol { \mathit { I } } _ { i }$ frame, which slowly drifts due to errors in gyroscope integration, as shown in Fig. 5. As a low-pass filter, we use a second-order Butterworth filter. The cutof frequency $f _ { \mathrm { c , a c c } }$ of this filter defines the weight between gyroscope prediction and accelerometer correction. To ensure a fast and robust convergence when the algorithm is initialized, we calculate the arithmetic mean for the first few samples instead of using the Butterworth filter. Then, the filter state is initialized based on this mean value.

As illustrated in Fig. 15, we can use this vertical reference to correct the inclination estimate. If $[ a _ { x } a _ { y } a _ { z } ] ^ { \intercal }$ denotes the filtered and normalized acceleration measurement in the $\mathcal { E } _ { i }$ frame, the shortest rotation for inclination correction has an angle of arccos 𝑎 around the axis $[ a _ { x } a _ { y } a _ { z } ] ^ { \intercal } \times [ 0 0 1 ] ^ { \intercal } = [ a _ { y } - a _ { x } 0 ] ^ { \intercal }$ ⊺. The quaternion $\mathbf { q } _ { \mathrm { c o r r } }$ that corresponds to this rotation can be expressed without trigonometric functions as

$$
q _ {w} = \cos \left(\frac {\arccos a _ {z}}{2}\right) = \sqrt {\frac {a _ {z} + 1}{2}}\tag{4}
$$

$$
\mathbf {q} _ {\mathrm{corr}} = \left[ q _ {w} \quad \frac {a _ {y}}{2 q _ {w}} \quad \frac {- a _ {x}}{2 q _ {w}} \quad 0 \right] ^ {\intercal}.\tag{5}
$$

This quaternion is used to correct the estimate of the $\mathbf { \Pi } _ { \boldsymbol { \varepsilon } _ { i } } ^ { I _ { i } } \mathbf { q }$ quaternion, i.e.,

$$
\mathop{\mathcal{I}}_{\substack{i(t_{k})\\ \mathcal{E}_{i}(t_{k})}}\mathbf{q} = \mathbf{q}_{\text{corr}}(t_{k})\otimes \mathop{\mathcal{I}}_{\substack{i(t_{k - 1})\\ \mathcal{E}_{i}(t_{k - 1})}}\mathbf{q}.\tag{6}
$$

![](images/3ac3cb71265d9a48a236c10404a0370de47e516eb8ccf96d60b8bddf513ac131.jpg)  
Figure 15: Illustration of the inclination correction step based on the filtered accelerometer measurement. In global $\mathcal { E } _ { i }$ coordinates, the filtered and normalized acceleration $\mathbf { a } = [ a _ { x } a _ { y } a _ { z } ] ^ { \intercal }$ is expected to point in positive 𝑧-direction. This can be achieved by a correction rotation with angle arccos $a _ { z }$ and axis $[ a _ { y } ^ { \phantom { \dagger } } - a _ { x } ^ { \phantom { \dagger } } \ 0 ] ^ { \intercal }$

After the correction step, the low-pass filtered acceleration will perfectly point in upward direction, i.e., in 𝑧-direction of the $\mathcal { E } _ { i }$ and frames.

## A.3. Magnetometer Correction

If magnetometer measurements are $\mathrm { g i v e n }$ , we use them to correct the heading estimate. As shown in (1), the heading is tracked via a scalar state $\delta _ { i } ( t _ { k } )$ that represents the vertical rotation from the global frame (with the 𝑦-axis pointing north) to the $\mathcal { E } _ { i }$ frame. As illustrated in Fig. 16, we can use the current magnetometer sample to derive a measurement $\delta _ { \mathrm { m a g } } ( t _ { k } )$ for this state by projecting the magnetic field vector into the horizontal plane. The state is then corrected by a fixed fraction $k _ { \mathrm { m a g } }$ of the deviation between state and measurement, which corresponds to a first-order low-pass filter with exponential convergence.

The parameter $k _ { \mathrm { m a g } }$ defines the fusion weight between gyroscope prediction and magnetometer correction. To ensure robust and fast convergence when the filter is initialized with the default value $\delta _ { i } ( t _ { 0 } ) = 0$ , we average the first measurements by choosing the filter weight $k _ { \mathrm { m a g } }$ as $1 , { \frac { 1 } { 2 } } , { \frac { 1 } { 3 } } , \dots$ during the first $\frac { 1 } { k _ { \mathrm { m a g } } }$ steps.

## A.4. Definition of Intuitive Fusion Weights

The behavior of the algorithm can be influenced by two fusion weights $f _ { \mathrm { c , a c c } }$ and $k _ { \mathrm { m a g } } ,$ which we will now replace with a more intuitive parametrization by defining intuitive time constants $\tau _ { \mathrm { a c c } }$ and $\tau _ { \mathrm { m a g } }$ that allow the user to influ ence the fusion weights between gyroscope prediction and accelerometer correction and between gyroscope prediction and magnetometer correction, respectively. The parameters $f _ { \mathrm { c , a c c } }$ and $k _ { \mathrm { m a g } }$ that are internally used in the filter update step (Algorithm 1) are automatically derived from those time constants.

![](images/10882371a7e1ec123685c90465eda17f1a13cfb8b02b55b0c80d79a69afa2b3f.jpg)  
Figure 16: Illustration of the heading correction step based on the magnetometer measurement. In many parts of the world, Earth’s magnetic field is dominated by a vertical component, e.g., in Berlin, the magnetic field is pointing down with a dip angle of $6 8 ^ { \circ }$ . An estimate of the magnetic north direction can be obtained by projecting the measured magnetic field into the horizontal plane.

In order to specify the gain $k _ { \mathrm { m a g } }$ of the magnetometer correction first-order exponential filter, we use the time constant $\begin{array} { r } { \tau = \frac { 1 } { 2 \pi f _ { \mathrm { c } } } } \end{array}$ that is commonly used to characterize first-order systems and corresponds to the time needed for the step response to reach $1 - e ^ { - 1 } \approx 6 3 . 2 \%$ of its final value. The filter weight for the proportional update can be derived from this time constant as

$$
k _ {\mathrm{mag}} = 1 - \exp \left(- \frac {T _ {\mathrm{s}}}{\tau_ {\mathrm{mag}}}\right).\tag{7}
$$

The second-order Butterworth filter used for accelerometer correction is characterized by the cutof frequency $f _ { \mathrm { c , a c c } } .$ In order to obtain a parametrization that is similar to the parametrization of the magnetometer correction, we use a time constant that corresponds to the undampened part of the step response, i.e., $\begin{array} { r } { \tau _ { \mathrm { a c c } } = \frac { \sqrt { 2 } } { 2 \pi f _ { \mathrm { c , a c c } } } } \end{array}$ . The cutof frequency used to determine the Butterworth filter coeficients is then given as

$$
f _ {\mathrm{c,acc}} = \frac {\sqrt {2}}{2 \pi \tau_ {\mathrm{acc}}}.\tag{8}
$$

See Fig. 17 for a comparison of the step responses of both filter types in relation to the time constant 𝜏.

This mapping is used to derive the internal parameters $f _ { \mathrm { c , a c c } }$ and $k _ { \mathrm { m a g } }$ from the user-specified time constants $\tau _ { \mathrm { a c c } }$ and $\tau _ { \mathrm { m a g } } .$

## B. Gyroscope Integration Errors Cause Drift of Almost-Inertial Frame

Every gyroscope prediction step causes a small error, due to gyroscope bias, noise, and other measurement errors. Without correction by accelerometers and magnetometers, those errors will add up and lead to drift in the orientation estimates. We can show that this error can be regarded as a small drift in the $\tau _ { i }$ frame, i.e., all rotation that happens in the gyroscope prediction and that is not the true change of sensor orientation can mathematically be expressed as a rotation $\begin{array} { r } { T _ { i } ( t _ { k - 1 } ) _ { } } \\ { T _ { i } ( t _ { k } ) ^ { } } \end{array}$

![](images/a7e1b5a333d0f17b19dc0dab1129e1866ec3a3b2ded218831de0e7c7b75aa106.jpg)  
Figure 17: Step response for first- and second-order low-pass filters. The time axis is normalized by dividing by the time constant 𝜏 of the proposed parametrization. Both filter outputs are roughly close to 0.5 at = and converge up to a deviation of $\leq 5 \%$ at $t = 3 \tau$

The gyroscope prediction step consists of multiplication of the previous estimate with an update quaternion based on the measured angular rate:

$$
\mathop{\mathcal{S}}_{\substack{i(t_{k})\\ \mathcal{I}_{i}(t_{k})}}\mathbf{q} = \mathop{\mathcal{S}}_{\substack{i(t_{k - 1})\\ \mathcal{I}_{i}(t_{k - 1})}}\mathbf{q}\otimes \left(T_{s}\| \boldsymbol {\omega}\|   @   \boldsymbol {\omega}\right).\tag{9}
$$

This update quaternion can be expressed as the true change in sensor orientation multiplied with a small error quaternion $\mathbf { q } _ { \mathrm { e } } , \mathrm { i } . \mathrm { e } .$

$$
\left(T _ {s} \| \pmb {\omega} \| @ \pmb {\omega}\right) = \mathsf {\Pi} _ {\mathcal {S} _ {i} (t _ {k - 1})} ^ {\mathcal {S} _ {i} (t _ {k})} \mathbf {q} \otimes \mathbf {q} _ {\mathrm{e}}.\tag{10}
$$

We can transform the error rotation ${ \bf q } _ { \mathrm { e } }$ to any frame, here $\boldsymbol { T } _ { i } ( t _ { k - 1 } ) ;$ :

$$
\mathbf {q} _ {\mathrm{e}} = \frac {\mathcal {I} _ {i} (t _ {k - 1})}{\mathcal {S} _ {i} (t _ {k})} \mathbf {q} \otimes \left[ \mathbf {q} _ {\mathrm{e}} \right] _ {\mathcal {I} _ {i} (t _ {k - 1})} \otimes \frac {\mathcal {I} _ {i} (t _ {k - 1})}{\mathcal {S} _ {i} (t _ {k})} \mathbf {q} ^ {- 1}.\tag{11}
$$

When putting this into the prediction step, we obtain

$$
\mathbf {\Phi} _ {I _ {i} (t _ {k})} ^ {S _ {i} (t _ {k})} \mathbf {q} = \mathbf {\Phi} _ {I _ {i} (t _ {k - 1})} ^ {S _ {i} (t _ {k - 1})} \mathbf {q} \otimes \left(T _ {s} \| \boldsymbol {\omega} \| @ \boldsymbol {\omega}\right)\tag{12}
$$

$$
= \mathsf {I} _ {i (t _ {k - 1})} ^ {\mathcal {S} _ {i} (t _ {k - 1})} \mathbf {q} \otimes \mathsf {S} _ {i (t _ {k - 1})} ^ {\mathcal {S} _ {i} (t _ {k})} \mathbf {q} \otimes \mathbf {q} _ {\mathrm{e}}\tag{13}
$$

$$
= \mathop {\mathcal {I}} _ {i (t _ {k - 1})} ^ {S _ {i} (t _ {k})} \mathbf {q} \otimes \mathop {\mathcal {I}} _ {i (t _ {k - 1})} ^ {I _ {i} (t _ {k - 1})} \mathbf {q} \otimes \left[ \mathbf {q} _ {\mathrm{e}} \right] _ {I _ {i} (t _ {k - 1})} \otimes \mathop {\mathcal {I}} _ {i (t _ {k - 1})} ^ {I _ {i} (t _ {k - 1})} \mathbf {q} ^ {- 1}\tag{14}
$$

$$
= \left[ \mathbf {q} _ {\mathrm{e}} \right] _ {\mathcal {I} _ {i} (t _ {k - 1})} \otimes_ {\mathcal {I} _ {i} (t _ {k - 1})} ^ {\mathcal {S} _ {i} (t _ {k})} \mathbf {q}\tag{15}
$$

$$
\mathbf {\Pi} _ {I _ {i} (t _ {k})} ^ {I _ {i} (t _ {k - 1})} \mathbf {q}
$$

Therefore, expressed in the almost-inertial frame $\boldsymbol { T } _ { i } ( t _ { k - 1 } ) ,$ the gyroscope prediction error quaternion ${ \bf q } _ { \mathrm { e } }$ corresponds to the drift rotation $\begin{array} { r } { T _ { i } ( t _ { k - 1 } ) _ { } } \\ { T _ { i } ( t _ { k } ) ^ { } } \end{array}$ of the almost-inertial frame $\boldsymbol { \mathit { I } } _ { i }$

## C. Details on Gyroscope Bias Estimation

The full algorithm for gyroscope bias estimation is given in Algorithm 2.

To detect whether the IMU is at rest (procedure REST-DETECTION in Algorithm 2), we first filter each component of the gyroscope and accelerometer measurements with a second-order Butterworth filter and a time constant of $\tau =$ 0.5 s. Note that, unlike the low-pass filter for the acceleration used for inclination correction, we apply the filter directly in the sensor frame. We then calculate the Euclidean norm of the deviation between the current measurement and the filtered measurement. Rest is detected if, in the last 1 5 s, the gyroscope and accelerometer deviations are always less than $\bar { 2 } ^ { \circ } / \mathrm { s } \mathrm { a n d } 0 . 5 \mathrm { m } / \mathrm { s } ^ { 2 }$ , respectively. Note that we deliberately do not use magnetometer measurements for rest detection since we found the rest detection to be very reliable when only using gyroscope and accelerometer measurements, whereas using magnetometers did not add additional value.

<div class="mineru-algorithm" style="white-space: pre-wrap; font-family:monospace;">
Algorithm 2 Gyroscope Bias Estimation
1: procedure RESTDETECTION(ω, a)
2:    $T_{\text{rest}} \leftarrow T_{\text{rest}} + T_s$
3:    for each measurement ω, a do
4:    low-pass filter the measurement with τ = 0.5 s
5:    if absolute difference between current and filtered measurement value is above threshold for any component then
6:    $T_{\text{rest}} \leftarrow 0$
7:    end if
8:    end for
9:    if $T_{\text{rest}} \geq 1.5$ s then
10:    rest detected
11:    else
12:    movement detected
13:    end if
14: end procedure
15: procedure INITIALIZEKALMANFILTER
16:    $\hat{\mathbf{b}} \leftarrow [0 0 0]^{\intercal} \quad \triangleright$ Gyroscope bias estimate
17:    $\mathbf{P} \leftarrow (0.5^{\circ}/s)^{2}\mathbf{I}_{3\times3} \quad \triangleright$ Covariance matrix
18:    $v \leftarrow (0.1^{\circ}/s)^{2}T_s(100s)^{-1} \quad \triangleright$ System noise
19:    $w_{\text{motion}} \leftarrow (0.1^{\circ}/s)^{4}v^{-1} + (0.1^{\circ}/s)^{2} \quad \triangleright$ Motion update variance
20:    $w_{\text{rest}} \leftarrow (0.03^{\circ}/s)^{4}v^{-1} + (0.03^{\circ}/s)^{2} \quad \triangleright$ Rest update variance
21: end procedure
22: procedure BIASESTIMATIONSTEP($_i$ q, $a_x, a_y, a_z$)
23:    $\mathbf{R} \leftarrow$ rotation matrix corresponding to $_i$ q
24:    $R_{LP} \leftarrow$ low-pass filter R with τ = τacc
25:    $\hat{\mathbf{b}}_{\varepsilon_i,LP} \leftarrow$ low-pass filter R$\hat{\mathbf{b}}$ with τ = τacc
26:    if rest detected then
27:    $y \leftarrow \hat{\mathbf{b}}$
28:    $C \leftarrow I_{3\times3}$
29:    $W \leftarrow w_{\text{rest}}[1 1 1]^{\intercal}$
30:    else
31:    $y \leftarrow T_s^{-1}[a_y - a_x 0]^{\intercal} + \text{diag}(1, 1, 0)\hat{\mathbf{b}}_{\varepsilon_i,LP}$
32:    $C \leftarrow R_{LP}$
33:    $W \leftarrow w_{\text{motion}}[1 1 \frac{1}{0.0001}]^{\intercal}$
34:    end if
35:    $P \leftarrow P + v[1 1 1]^{\intercal} \quad \triangleright$ Kalman filter update
36:    $K \leftarrow PC^{\intercal}(W + CPC^{\intercal})^{-1}$
37:    $\hat{\mathbf{b}} \leftarrow \hat{\mathbf{b}} + K\text{clip}(y - C\hat{\mathbf{b}}, -2^{\circ}/s, 2^{\circ}/s) \quad \triangleright$ Limit disagreement to 2°/s
38:    $P \leftarrow P - KCP$
39:    $\hat{\mathbf{b}} \leftarrow \text{clip}(\hat{\mathbf{b}}, -2^{\circ}/s, 2^{\circ}/s) \quad \triangleright$ Limit bias estimate to 2°/s
40: end procedure
</div>

To estimate the gyroscope bias during rest and motion, we employ a Kalman filter [29, 30], with the gyroscope bias as state and a time-dependent output matrix:

$$
\mathbf {b} (t _ {k}) = \mathbf {b} (t _ {k - 1}) + \mathbf {v} (t _ {k}), \qquad \mathbf {v} (t _ {k}) \sim \mathcal {N} (0, \mathbf {V})\tag{16}
$$

$$
\mathbf {y} (t _ {k}) = \mathbf {C} (t _ {k}) \mathbf {b} (t _ {k}) + \mathbf {w} (t _ {k}), \quad \mathbf {w} (t _ {k}) \sim \mathcal {N} (0, \mathbf {W} (t _ {k})).\tag{17}
$$

During rest, we use the low-pass filtered gyroscope readings ${ \pmb \omega } _ { \mathrm { L P } }$ (which we calculated in the rest detection procedure) as a direct measurement of the bias, i.e., $\mathbf { C } ( t _ { k } ) =$ $\mathbf { I } _ { 3 \times 3 }$ and $\mathbf { y } ( t _ { k } ) = \pmb { \omega } _ { \mathrm { L P } } ( t _ { k } )$ . Because the IMU is at rest and gyroscope readings are already filtered, we can assign a comparatively large weight (i.e., a small covariance) to this measurement update and achieve fast convergence for the bias estimate $\hat { \mathbf { b } } ( t _ { k } )$

During motion, we estimate the gyroscope bias from the inclination correction steps. At every time step, the new error due to gyroscope bias is a local rotation $( \mathrm { i . e . , i n } S _ { i } )$ with the rotation vector $T _ { s } ( { \bf { b } } - \hat { { \bf { b } } } )$ , and the inclination correction is a global rotation $( \mathrm { i . e . }$ , in $\mathcal { E } _ { i } )$ with a horizontal rotation vector $\mathbf { c } = [ c _ { x } c _ { v } 0 ] ^ { \intercal }$ . In ideal conditions $( \mathrm { i . e . } ,$ , in a steady state and without noise or other errors), the correction rotation will exactly compensate the inclination portion of the bias rotation. As illustrated in Fig. 18, in this case, the correction is the inverse of the horizontal projection of the bias rotation.

With 𝐑 being the rotation matrix corresponding to $\mathbf { \Pi } _ { \varepsilon _ { i } } ^ { S _ { i } } \mathbf { q } .$ we can express this as

$$
\mathbf {R} (\mathbf {b} - \hat {\mathbf {b}}) = - \frac {1}{T _ {\mathrm{s}}} \left[ \begin{array}{c} c _ {x} \\ c _ {y} \\ * \end{array} \right],\tag{18}
$$

where the star indicates that the gyroscope bias in the current vertical direction is not observable by accelerometer mea surements.

To achieve slow forgetting of the bias for the special case in which the same axis is vertical for a long time, we set the corresponding measurement to zero, with a significantly larger variance to slow down convergence.

With an output matrix of $\mathbf { C } _ { k } \ = \ \mathbf { R } \ = : \ ( r _ { i j } )$ and the correction vector $[ \ : a _ { y } \ : \ : \ : - \ : a _ { x } \ : \ : 0 \ : ] ^ { \intercal }$ , we obtain the following

![](images/5a8f764b79a53969aeb65280787923d45fd0eb9a08aa2ba3c36b92f54c8c0182.jpg)  
Figure 18: Illustration of the principle behind gyroscope bias estimation from the inclination correction step. In the steady state, the correction angular rate ${ \pmb \omega } _ { \mathrm { c o r r } }$ that corresponds to the accelerometer-based correction is the (negative) horizontal projection of the remaining gyroscope bias $\mathbf { b } - \mathbf { \hat { b } }$

measurement equation:

$$
\mathbf {y} _ {k} = \left[ \begin{array}{c} - \frac {1}{T _ {\mathrm{s}}} a _ {y} + r _ {1 1} \hat {b} _ {x} + r _ {1 2} \hat {b} _ {y} + r _ {1 3} \hat {b} _ {z} \\ \frac {1}{T _ {\mathrm{s}}} a _ {x} + r _ {2 1} \hat {b} _ {x} + r _ {2 2} \hat {b} _ {y} + r _ {2 3} \hat {b} _ {z} \\ 0 \end{array} \right].\tag{19}
$$

This measurement equation does not take into account that the measured accelerations are low-pass filtered in the $\mathcal { E } _ { i }$ frame. The accuracy and robustness of the bias estimation can be significantly improved by low-pass filtering the components of 𝐑 and 𝐑̂𝐛 with the same low-pass filter used for the accelerometer measurements in the $\tau _ { i }$ frame. See $\mathsf { A p - }$ pendix D for a detailed derivation of this full measurement equation.

The tuning parameters of a Kalman filter are the initial covariance, the covariance of the system noise, and the covariance of the measurement noise. Since the real values of those covariances are hard to obtain and would furthermore depend on the sampling rate, we employ a parametrization that ensures the following properties:

1. The initial estimation uncertainty is $\sigma _ { \mathrm { i n i t } } = 0 . 5 ^ { \circ } / \mathrm { s } .$

2. During motion, the uncertainty converges to $\sigma _ { \mathrm { m o t i o n } } =$ $0 . 1 ^ { \circ } / \mathrm { s }$ (for the non-vertical axes).

3. During rest, the uncertainty converges to $\begin{array} { r l } { \sigma _ { \mathrm { r e s t } } } & { { } = } \end{array}$ $0 . 0 3 ^ { \circ } / \mathrm { s }$

4. Without updates, the estimation uncertainty increases from 0 to $0 . 1 ^ { \circ } / \mathrm { s }$ in the forgetting time $t _ { \mathrm { f o r g e t } } = 1 0 0 \mathrm { s } .$

How those parameters translate to the covariances internally used by the Kalman filter is specified in Algorithm 2 and further explained in Appendix E. Note that the absolute values of the provided parameters are arbitrary and chosen to facilitate an intuitive understanding of the estimation uncertainty. For the behavior of the Kalman filter, only the relation between the parameters is relevant.

## D. Measurement for Motion Bias Estimation

The approach for gyroscope bias estimation during motion, as shown in Fig. 18, is based on the assumption that the current inclination correction step corresponds to the gyroscope bias that is transformed to the global frame using the current sensor orientation. This works well if the sensor orientation does not change much. In reality, due to the low-pass filter used for the acceleration, the inclination correction corresponds to the bias-induced rotations from the last few seconds, taking the respective sensor orientations into account. In the following, we show that this efect is well-described by applying the same low-pass filter to the elements of the rotation matrices 𝐑 corresponding to $\mathbf { \Pi } _ { I _ { i } } ^ { S _ { i } } \mathbf { q } .$ and also to the rotated bias estimates 𝐑̂𝐛.

To simplify the notation, we introduce the rotation operator:

$$
\operatorname{rot} (\mathbf {q}, \mathbf {v}) := \mathbf {q} \otimes \mathbf {v} \otimes \mathbf {q} ^ {- 1}.\tag{20}
$$

Assume that the accelerometer measurements are always a perfect vertical vector $\begin{array} { r l r } { \left( \left[ \mathbf { a } ( t _ { k } ) \right] _ { \mathcal { E } _ { \mathrm { t r u e } } } \right. } & { { } = } & { \left[ \mathbf { v } \right] _ { \mathcal { E } _ { \mathrm { t r u e } } } = } \end{array}$ [ 0 0 1 ]⊺), i.e. there are no disturbances or measurement errors (unit length is only used to simplify the notation). In the orientation estimation update step, those accelerometer measurements are transformed into the $\boldsymbol { \cal T } _ { i } ( t _ { k } )$ frame and then low-pass filtered with an infinite impulse response (IIR) filter with impulse response $b _ { n } .$ , i.e.,

$$
\left[ \mathbf {a} _ {\mathrm{LP}} (t _ {k}) \right] _ {\mathcal {I} _ {i} (t _ {k})} = \sum_ {n = 0} ^ {\infty} b _ {n} \left[ \mathrm{rot} \left(\mathbf {\Sigma} _ {\mathcal {I} _ {i} (t _ {k - n})} ^ {\mathcal {E} _ {\mathrm{true}}} \mathbf {q}, \left[ \mathbf {v} \right] _ {\mathcal {E} _ {\mathrm{true}}}\right) \right]\tag{21}
$$

$$
= \sum_ {n = 0} ^ {\infty} b _ {n} \left[ \mathbf {v} \right] _ {\mathcal {I} _ {i} (t _ {k - n})}.\tag{22}
$$

One time step earlier, the filter output is

$$
\begin{array}{r l} \left[ \mathbf {a} _ {\mathrm{LP}} (t _ {k - 1}) \right] _ {\mathcal {I} _ {i} (t _ {k - 1})} & = \sum_ {n = 0} ^ {\infty} b _ {n} \left[ \mathbf {v} \right] _ {\mathcal {I} _ {i} (t _ {k - 1 - n})} \\ & = \sum_ {n = 0} ^ {\infty} b _ {n} \operatorname{rot} \left(\frac {\mathcal {I} _ {i} (t _ {k - n})}{\mathcal {I} _ {i} (t _ {k - 1 - n})} \mathbf {q}, \left[ \mathbf {v} \right] _ {\mathcal {I} _ {i} (t _ {k - n})}\right) \end{array} \tag {23}\tag{24}
$$

From Rodrigues’ rotation formula follows that for a small rotation 𝜃 around axis 𝐤,

$$
\mathbf {p} _ {\mathrm{rot}} \approx \mathbf {p} + \theta \mathbf {k} \times \mathbf {p}.\tag{25}
$$

We use this approximation for the rotation of the $\tau _ { i }$ frame due to gyroscope bias in one sample step $\begin{array} { r } { T _ { i } ( t _ { k - 1 - n } ) _ { \bf { \sigma } } } \\ { T _ { i } ( t _ { k - n } ) ^ { \top } = } \end{array}$ (cf. Appendix B). Note that this rotation is only caused by the uncorrected part of the gyroscope bias, i.e., by $\mathbf { b } ^ { \prime } ( t _ { k } ) : =$ $\mathbf { b } - \hat { \mathbf { b } } ( t _ { k } )$ . Therefore, we obtain

$$
\left[ \mathbf {a} _ {\mathrm{LP}} (t _ {k - 1}) \right] _ {\mathcal {I} _ {i} (t _ {k - 1})}\tag{26}
$$

$$
\approx \sum_ {n = 0} ^ {\infty} b _ {n} \left(\left[ \mathbf {v} \right] _ {\mathcal {I} _ {i} (t _ {k - n})} - T _ {\mathrm{s}} \left[ \mathbf {b} ^ {\prime} (t _ {k - n}) \right] _ {\mathcal {I} _ {i} (t _ {k - n})} \times \left[ \mathbf {v} \right] _ {\mathcal {I} _ {i} (t _ {k - n})}\right).\tag{27}
$$

The diference between two consecutive filtered accelerations is then

$$
\left[ \mathbf {a} _ {\mathrm{LP}} (t _ {k}) \right] _ {\mathcal {I} _ {i} (t _ {k})} - \left[ \mathbf {a} _ {\mathrm{LP}} (t _ {k - 1}) \right] _ {\mathcal {I} _ {i} (t _ {k - 1})}\tag{28}
$$

$$
\approx T _ {\mathrm{s}} \sum_ {n = 0} ^ {\infty} b _ {n} \left(\left[ \mathbf {b} ^ {\prime} (t _ {k - n}) \right] _ {\mathcal {I} _ {i} (t _ {k - n})} \times \left[ \mathbf {v} \right] _ {\mathcal {I} _ {i} (t _ {k - n})}\right)\tag{29}
$$

$$
= T _ {s} \sum_ {n = 0} ^ {\infty} b _ {n} \left(\operatorname{rot} \binom{\mathcal {S} _ {i} (t _ {k - n})}{\mathcal {I} _ {i} (t _ {k - n})} \mathbf {q}, \mathbf {b} ^ {\prime} (t _ {k - n})\right) \times \left[ \mathbf {v} \right] _ {\mathcal {I} _ {i} (t _ {k - n})}\left. \right).\tag{30}
$$

Now, we express this diference in the frame $\mathcal { E } _ { i } ( t _ { k - 1 } )$ that is used to perform the inclination correction step, assuming that $\mathcal { E } _ { i }$ and $\tau _ { i }$ do not change much over the duration that is relevant for the filter (since the influence of bias and the inclination correction is limited during short time spans), and assuming that the true vertical axis is approximately [ 0 0 1 ]⊺ in the $\mathcal { E } _ { i } ( t _ { k - 1 } )$ frame.

$$
\left[ \mathbf {a} _ {\mathrm{LP}} (t _ {k}) \right] _ {\mathcal {E} _ {i} (t _ {k - 1})} - \left[ \mathbf {a} _ {\mathrm{LP}} (t _ {k - 1}) \right] _ {\mathcal {E} _ {i} (t _ {k - 1})}\tag{31}
$$

$$
\approx T _ {\mathrm{s}} \sum_ {n = 0} ^ {\infty} b _ {n} \left[ \operatorname{rot} \left( \begin{array}{c} \mathcal {S} _ {i} (t _ {k - n}) \\ \mathcal {E} _ {i} (t _ {k - 1 - n}) \end{array} \mathbf {q}, \mathbf {b} ^ {\prime} (t _ {k - n})\right) \times \left[ \begin{array}{c} 0 \\ 0 \\ 1 \end{array} \right] \right]\tag{32}
$$

$$
= T _ {\mathrm{s}} \left[ \sum_ {n = 0} ^ {\infty} b _ {n} \operatorname{rot} \left(\mathcal {S} _ {i (t _ {k - n})} (\mathbf {q}, \mathbf {b} ^ {\prime} (t _ {k - n}))\right) \right] \times \left[ \begin{array}{c} 0 \\ 0 \\ 1 \end{array} \right]\tag{33}
$$

Expressing the rotation by $\begin{array} { r } { S _ { i } ( t _ { k - n } ) _ { \mathbf { q } } } \\ { \varepsilon _ { i } ( t _ { k - n - 1 } ) ^ { } { \bf q } } \end{array}$ with a rotation matrix ${ \bf R } ( t _ { k - n } )$ and introducing an LPF operator to simplify the notation of the low-pass filter yields

$$
\left[ \mathbf {a} _ {\mathrm{LP}} (t _ {k}) \right] _ {\mathcal {E} _ {i} (t _ {k - 1})} - \left[ \mathbf {a} _ {\mathrm{LP}} (t _ {k - 1}) \right] _ {\mathcal {E} _ {i} (t _ {k - 1})}\tag{34}
$$

$$
= T _ {\mathrm{s}} \left[ \sum_ {n = 0} ^ {\infty} b _ {n} \left(\mathbf {R} (t _ {k - n}) \mathbf {b} ^ {\prime} (t _ {k - n})\right) \right] \times \left[ \begin{array}{c} 0 \\ 0 \\ 1 \end{array} \right]\tag{35}
$$

$$
= T _ {\mathrm{s}} \operatorname{LPF} \left(\mathbf {R} (t _ {k}) \mathbf {b} ^ {\prime} (t _ {k})\right) \times \left[ \begin{array}{c} 0 \\ 0 \\ 1 \end{array} \right].\tag{36}
$$

Since $\left[ \mathbf { a } _ { \mathrm { L P } } ( t _ { k - 1 } ) \right] _ { \mathcal { E } _ { i } ( t _ { k - 1 } ) }$ is always $[ 0 \ 0 \ 1 ] ^ { \intercal }$ as the result of the previous inclination correction step, we get

$$
\left[ \mathbf {a} _ {\mathrm{LP}} (t _ {k}) \right] _ {\mathcal {E} _ {i} (t _ {k - 1})} = \left[ \begin{array}{c} 0 \\ 0 \\ 1 \end{array} \right] + T _ {\mathrm{s}}   \text { LPF } \left(\mathbf {R} (t _ {k}) \mathbf {b} ^ {\prime} (t _ {k})\right) \times \left[ \begin{array}{c} 0 \\ 0 \\ 1 \end{array} \right].\tag{37}
$$

The inclination correction rotation vector $\mathbf { c } ( t _ { k } )$ , i.e., expressing the correction quaternion $\mathbf { q } _ { \mathrm { c o r 1 } }$ from (5) as a rotation vector, is (neglecting the small change of the norm of $\left[ \mathbf { a } _ { \mathrm { L P } } ( t _ { k } ) \right] _ { \mathcal { E } _ { i } ( t _ { k - 1 } ) } )$

$$
\mathbf {c} (t _ {k}) = \left[ \mathbf {a} _ {\mathrm{LP}} (t _ {k - 1}) \right] _ {\mathcal {E} _ {i} (t _ {k})} \times \left[ \begin{array}{c} 0 \\ 0 \\ 1 \end{array} \right]\tag{38}
$$

$$
= \left(\left[ \begin{array}{c} 0 \\ 0 \\ 1 \end{array} \right] + T _ {\mathrm{s}} \operatorname{LPF} \left(\mathbf {R} (t _ {k}) \mathbf {b} ^ {\prime} (t _ {k})\right) \times \left[ \begin{array}{c} 0 \\ 0 \\ 1 \end{array} \right]\right) \times \left[ \begin{array}{c} 0 \\ 0 \\ 1 \end{array} \right]\tag{39}
$$

$$
= - \operatorname{diag} (1, 1, 0) T _ {\mathrm{s}} \operatorname{LPF} \left(\mathbf {R} \left(t _ {k}\right) \mathbf {b} ^ {\prime} \left(t _ {k}\right)\right)\tag{40}
$$

$$
= - \operatorname{diag} (1, 1, 0) T _ {\mathrm{s}} \left(\operatorname{LPF} (\mathbf {R} (t _ {k})) \mathbf {b} - \operatorname{LPF} (\mathbf {R} (t _ {k}) \hat {\mathbf {b}} (t _ {k}))\right).\tag{41}
$$

Therefore, when choosing the system output matrix as $\mathbf { C } ( t _ { k } ) = \mathrm { L P F } ( \mathbf { R } ( t _ { k } ) )$ , we obtain the following measurement in the horizontal plane:

$$
\operatorname{diag} (1, 1, 0) \mathbf {y} \left(t _ {k}\right) = - \frac {1}{T _ {\mathrm{s}}} \mathbf {c} \left(t _ {k}\right) + \operatorname{diag} (1, 1, 0) \operatorname{LPF} \left(\mathbf {R} \left(t _ {k}\right) \hat {\mathbf {b}} \left(t _ {k}\right)\right).\tag{42}
$$

## E. Parametrization of the Bias Estimation Method

Gyroscope bias is estimated using the system model

$$
\mathbf {b} _ {k} = \mathbf {b} _ {k - 1} + \mathbf {v} _ {k}, \qquad \mathbf {v} _ {k} \sim \mathcal {N} (0, \mathbf {V}),\tag{43}
$$

VQF: Highly Accurate IMU Orientation Estimation with Bias Estimation and Magnetic Disturbance Rejection

$$
\mathbf {y} _ {k} = \mathbf {C} _ {k} \mathbf {b} _ {k} + \mathbf {w} _ {k}, \qquad \mathbf {w} _ {k} \sim \mathcal {N} (0, \mathbf {W} _ {k}),\tag{44}
$$

where the index 𝑘 denotes sampling at $t _ { k } .$ , and the standard Kalman filter update equations for the estimated state $\hat { \mathbf { b } } _ { k }$

$$
\mathbf {P} _ {k} ^ {-} = \mathbf {P} _ {k - 1} ^ {+} + \mathbf {V}\tag{45}
$$

$$
\mathbf {K} _ {k} = \mathbf {P} _ {k} ^ {-} \mathbf {C} _ {k} ^ {\intercal} (\mathbf {W} _ {k} + \mathbf {C} _ {k} \mathbf {P} _ {k} ^ {-} \mathbf {C} _ {k} ^ {\intercal}) ^ {- 1}\tag{46}
$$

$$
\hat {\mathbf {b}} _ {k} = \hat {\mathbf {b}} _ {k - 1} + \mathbf {K} _ {k} (\mathbf {y} _ {k} - \mathbf {C} _ {k} \hat {\mathbf {b}} _ {k - 1})\tag{47}
$$

$$
\mathbf {P} _ {k} ^ {+} = \mathbf {P} _ {k} ^ {-} - \mathbf {K} _ {k} \mathbf {C} _ {k} \mathbf {P} _ {k} ^ {-}.\tag{48}
$$

The tuning parameters are the initial covariance $\mathbf { P } _ { 0 } ^ { + } ,$ the variance of the system noise 𝐕, and the variance of measurement noise $\mathbf { W } ( t _ { k } )$ . In the following, we derive an intuitive parametrization for those values that is independent of the sampling frequency. Note that scaling all parameters of the Kalman filter with the same value will not change the system behavior. For heuristically determined parame ters, the actual quantities are therefore arbitrary. However, a good parametrization still helps to make the behavior of the algorithm understandable and facilitates tuning.

A fixed amount of variance, the variance of the system noise 𝐕, is added to the covariance matrix in every update step. To be independent of the sampling frequency, scaling with the sampling time $T _ { \mathrm { s } }$ is necessary. To facilitate interpretation of the value as a forgetting time, we parametrize the system noise by the time needed for the standard deviation of the estimation uncertainty to increase from 0 to $0 . 1 ^ { \circ } / \mathrm { s }$ in the absence of measurements, i.e.,

$$
\mathbf {V} = (0. 1 ^ {\circ} / \mathrm{s}) ^ {2} \frac {T _ {\mathrm{s}}}{t _ {\mathrm{forget}}} \mathbf {I} _ {3 \times 3}.\tag{49}
$$

We use the initial estimation uncertainty, i.e., the standard deviation $\sigma _ { \mathrm { i n i t } }$ , to initialize the covariance matrix:

$$
\mathbf {P} _ {0} ^ {+} = \sigma_ {\mathrm{init}} ^ {2} \mathbf {I} _ {3 \times 3}.\tag{50}
$$

For the rest and motion updates, we provide the uncertainties $\sigma _ { \mathrm { r e s t } }$ and $\sigma _ { \mathrm { m o t i o n } }$ to which the estimate will eventually converge when the respective filter update is active instead of specifying the variance of the motion and rest update measurements directly. This ensures independence of the sampling rates and makes the parameters easy to compare to the initial standard deviation. The relation to the measurement variance $w _ { \mathrm { r e s t / m o t i o n } }$ is given by

$$
w _ {\mathrm{rest/motion}} = \frac {\sigma_ {\mathrm{rest/motion}} ^ {4}}{v} + \sigma_ {\mathrm{rest/motion}} ^ {2}.\tag{51}
$$

<div class="mineru-algorithm" style="white-space: pre-wrap; font-family:monospace;">
Algorithm 3 Magnetic Disturbance Rejection
1: procedure MAGDISTDETECTION(m, $S_i$ , q)
2: n ← ||m|| ▷ Norm of magnetic field
3: θ ← -arcsin([0 0 1]($S_i$ q ⊗ m ⊗ $S_i$ q-1) n-1) ▷ Dip angle
4: low-pass filter n and θ with τ = 0.05 s
5: if |n - nref| &lt; 0.1nref and |θ - θref| &lt; 10° then
6: Tundist ← Tundist + Ts
7: if Tundist ≥ 0.5 s then
8: disturbed ← false
9: nref ← kref (n - nref)
10: θref ← kref (θ - θref) ▷ Track slow changes of norm and dip
11: else
12: disturbed ← true
13: Tundist ← 0
14: end if
15: end if
16: end procedure
17: procedure NEWMAGFIELDACCEPTANCE(n, θ, |ω|)
18: if |n - n cand| &lt; 0.1n cand and |θ - θ cand| &lt; 10° then
19: if ||ω|| ≥ 20°/s then ▷ Only count the time if there is movement
20: Tcand ← Tcand + Ts
21: end if
22: n cand ← k ref (n - n cand)
23: θ cand ← k ref (θ - θ cand)
24: if disturbed and Tcand ≥ 20 s then ▷ Accept candidate as new reference
25: disturbed ← false
26: n ref ← n cand
27: θ ref ← θ cand
28: end if
29: else ▷ Reset candidate to current value
30: Tcand ← 0
31: n cand ← n
32: θ cand ← θ
33: end if
34: end procedure
35: procedure MAGDISTREJECTION
36: if disturbed then
37: if Treject &lt; 60 s then
38: Treject ← Treject + Ts
39: do not perform heading correction
40: else
41: perform heading correction with ½k mag
42: end if
43: else
44: Treject ← max(Treject - 2Ts, 0)
45: perform heading correction with k mag
46: end if
47: end procedure
</div>

To derive this, consider a simplified case of a Kalman filter for a system with one constant state $( x _ { k } = x _ { k - 1 } )$ and

VQF: Highly Accurate IMU Orientation Estimation with Bias Estimation and Magnetic Disturbance Rejection

direct measurement of the state (𝐶 = 1):

$$
p _ {k} ^ {-} = p _ {k - 1} ^ {+} + v\tag{52}
$$

$$
k _ {k} = \frac {p _ {k} ^ {-}}{w + p _ {k} ^ {-}}\tag{53}
$$

$$
\hat {x} _ {k} = \hat {x} _ {k - 1} + k _ {k} (y _ {k} - \hat {x} _ {k - 1})\tag{54}
$$

$$
p _ {k} ^ {+} = p _ {k} ^ {-} - k _ {k} p _ {k} ^ {-}.\tag{55}
$$

In the converged state, $k _ { k } = k _ { k - 1 }$ and $p _ { k } ^ { + } = p _ { k - 1 } ^ { + }$ . From (55) follows

$$
p _ {k} ^ {+} = p _ {k} ^ {+} + v - \frac {p _ {k} ^ {+} + v}{p _ {k} ^ {+} + v + w} (p _ {k} ^ {+} + v)\tag{56}
$$

$$
v = \frac {(p _ {k} ^ {+} + v) ^ {2}}{p _ {k} ^ {+} + v + w}\tag{57}
$$

$$
w = \frac {(p _ {k} ^ {+} + v) ^ {2}}{v} - p _ {k} ^ {+} - v = \frac {(p _ {k} ^ {+}) ^ {2}}{v} + p _ {k} ^ {+}.\tag{58}
$$

The relation given in (51) is then obtained by replacing the variance $p _ { k }$ with $\sigma ^ { 2 }$

Note that, for the 3-dimensional bias estimate, the $3 \times 3$ covariance matrix might not be close to a diagonal matrix, especially if the same sensor axis is vertical for a long time. The uncertainty 𝜎 of the bias estimate (in the worst-case direction) can be derived from the largest eigenvalue of the covariance matrix 𝐏. To avoid calculating eigenvalues, the Gershgorin circle theorem can be leveraged to obtain an upper bound estimate via the largest absolute row sum of 𝐏.

## F. Magnetic Disturbance Rejection Algorithm

The full algorithm for the magnetic disturbance detection and rejection extension as described in Section 2.5 is given in Algorithm 3.

## G. Ofline Orientation Estimation Algorithm

The full algorithm for the acausal ofline algorithm variant OflineVQF as described in Section 2.6 is given in Algorithm 4.

```txt
Algorithm 4 Offline Orientation Estimation Algorithm
1: procedure OFFLINEVQF(ω[t₁:N], a[t₁:N], m[t₁:N])
2:    ... , dist₁, b̂₁, P₁ ← VQF(ω[t₁:N], a[t₁:N], m[t₁:N])
3:    ... , dist₂, b̂₂, P₂ ← VQF(-ω[tₙ:1], a[tₙ:1], m[tₙ:1])
4:    dist[t₁:N] ← dist₁[t₁:N] ∧ dist₂[t₁:N]    ▷ Regard magnetic field as disturbed if both runs detected disturbances
5:    b̂[t₁:N] ← (P⁻¹₁ + P⁻¹₂)⁻¹(P⁻¹₁b̂₁ - P⁻¹₂b̂₂)    ▷ Average bias estimates of both filter runs via covariance
6:    Sᵢᵢq[t₁:N] ← integrateGyr(ω[t₁:N] - b̂[t₁:N])    ▷ Perform gyroscope strapdown integration
7:    [a]ᵢᵢ[t₁:N] ← Sᵢᵢq[t₁:N] ⊗ a[t₁:N] ⊗ Sᵢᵢq[t₁:N]⁻¹    ▷ Transform acceleration into Iᵢ frame
8:    [aLP]ᵢᵢ[t₁:N] ← filtfiltLPF([a]ᵢᵢ[t₁:N], τ = τ_acc)    ▷ Forward-backward low-pass filtering
9:    Iᵢεᵢq[t₁:N] ← perform inclination correction based on [aLP]ᵢᵢ[t₁:N]
10:    [mₓ mᵧ m_z]ᵀ[t₁:N] ← (Iᵢεᵢq[t₁:N] ⊗ Sᵢᵢq[t₁:N]) ⊗ m[t₁:N] ⊗ (Iᵢεᵢq[t₁:N] ⊗ Sᵢᵢq[t₁:N])⁻¹
11:    δ_mag[t₁:N] ← atan2(mₓ[t₁:N], mᵧ[t₁:N])
12:    δᵢ[t₁:N] ← run heading correction filter with magnetic disturbance rejection on δ_mag[t₁:N]
13:    δᵢ[t₁:N] ← run heading correction filter with magnetic disturbance rejection on δᵢ[tₙ:1]
14:    return Iᵢεᵢq[t₁:N] ⊗ Sᵢᵢq[t₁:N],
15:    [cos δᵢ[t₁:N]/2 0 0 sin δᵢ[t₁:N]/2]ᵀ ⊗ Iᵢεᵢq[t₁:N] ⊗ Sᵢᵢq[t₁:N]
16: end procedure
VQF: real-time implementation, returns magnetic disturbance state, bias estimate, and bias estimation covariance
integrateGyr: gyroscope strapdown integration by (3)
filtfiltLPF: forward-backward filtering with second-order Butterworth low-pass filter
```

Results of ${ \mathsf { T A G P } } _ { \mathrm { x } }$ -based parameter tuning for all IOE algorithms used in the evaluation

<table><tr><td>Algorithm</td><td>TAGPx</td><td>Param.</td><td>Value</td><td>Search grid (start:step:end)</td></tr><tr><td rowspan="2">VQF</td><td rowspan="2">2.64°</td><td> $\tau_{acc}$ </td><td>3</td><td>1:0.5:10</td></tr><tr><td> $\tau_{mag}$ </td><td>9</td><td>1:1:30</td></tr><tr><td rowspan="2">MAH [16]</td><td rowspan="2">14.83°</td><td> $K_P$ </td><td>1.44</td><td>0.02:0.02:4</td></tr><tr><td> $K_I$ </td><td>0.0027</td><td>0:0.0001:0.004</td></tr><tr><td rowspan="2">MAD [17]</td><td rowspan="2">12.01°</td><td> $\beta$ </td><td>0.29</td><td>0.01:0.01:1</td></tr><tr><td> $\zeta_{bias}$ </td><td>0</td><td>0:0.00001:0.001</td></tr><tr><td rowspan="5">VAC [13]</td><td rowspan="5">5.63°</td><td> $\alpha_{acc}$ </td><td>0.00085</td><td>0.0001:0.00005:0.001</td></tr><tr><td> $\beta_{mag}$ </td><td>0.0005</td><td>0.0001:0.00005:0.001</td></tr><tr><td> $\alpha_{bias}$ </td><td>0.00055</td><td>0.0001:0.00005:0.001</td></tr><tr><td> $b_{est}$ </td><td>true</td><td>{false, true}</td></tr><tr><td> $\alpha_{adapt}$ </td><td>false</td><td>{false, true}</td></tr><tr><td rowspan="3">FKF [22]</td><td rowspan="3">9.19°</td><td> $\sigma^{2}_{gyr}$ </td><td>0.001</td><td>0.001:0.001:0.001</td></tr><tr><td> $\sigma^{2}_{acc}$ </td><td>0.002</td><td>0.001:0.0001:0.005</td></tr><tr><td> $\sigma^{2}_{mag}$ </td><td>0.0033</td><td>0.001:0.0001:0.005</td></tr><tr><td rowspan="4">SEL [14]</td><td rowspan="4">4.58°</td><td> $\tau_{acc}$ </td><td>3.2</td><td>1:0.2:5</td></tr><tr><td> $\tau_{mag}$ </td><td>10</td><td>1:1:20</td></tr><tr><td> $\zeta_{bias}$ </td><td>5</td><td>0:1:10</td></tr><tr><td> $r_{acc}$ </td><td>2</td><td>0:1:10</td></tr><tr><td rowspan="8">MKF</td><td rowspan="8">7.58°</td><td> $\sigma^{2}_{acc}$ </td><td>0.00028171</td><td rowspan="8">MKF parameters were iteratively determined with line searches instead of a grid search.</td></tr><tr><td> $\sigma^{2}_{mag}$ </td><td>14.55188</td></tr><tr><td> $\sigma^{2}_{gyr}$ </td><td>0.15625</td></tr><tr><td> $\sigma^{2}_{gyrdrift}$ </td><td> $3 \times 10^{-21}$ </td></tr><tr><td> $\sigma^{2}_{linacc}$ </td><td>0.49128</td></tr><tr><td> $d_{linacc}$ </td><td>0.81297</td></tr><tr><td> $\sigma^{2}_{magdist}$ </td><td>0.12329</td></tr><tr><td> $d_{magdist}$ </td><td>0.51005</td></tr><tr><td rowspan="3">KOK [15]</td><td rowspan="3">11.70°</td><td> $\sigma_{gyr}$ </td><td>0.185</td><td>0.01:0.005:0.5</td></tr><tr><td> $\zeta_{bias}$ </td><td>0</td><td>0:0.00001:0.001</td></tr><tr><td> $m_{est}$ </td><td>true</td><td>{false, true}</td></tr><tr><td rowspan="2">RIANN [23]</td><td rowspan="2">1.32°</td><td colspan="3">no parameters</td></tr><tr><td colspan="3">TAGPx value not comparable to other algorithms because the inclination RMSE is used instead of the orientation RMSE.</td></tr></table>

## H. Parameter Tuning for the Evaluated IOE Algorithms

To provide a fair comparison for all evaluated algorithms, all algorithm parameters were tuned to provide the smallest possible $\mathrm { T A G P _ { x } }$ values. To determine the parameters, a grid search was performed, i.e., the algorithm per formance was evaluated on a grid defined by the Cartesian product of the linearly spaced parameter sets presented in Table 2. This search grid was iteratively adjusted to ensure that the distance between parameter values is suficiently small and that the TAGP parameters do not lie at the border of the grid. The resulting averaged error, the associated parameters, and the parameter search range are presented in Table 2.

changed while the other parameters are kept at the previ ously found minimum. The search range was also iteratively adjusted until it converged to a stable minimum.

Due to the high dimensionality of the search space and the slow implementation, the parameters for MKF were only evaluated using a line search, i.e., only one parameter was