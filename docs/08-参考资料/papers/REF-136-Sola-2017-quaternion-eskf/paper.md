# Quaternion kinematics for the error-state Kalman filter

Joan Solà

November 8, 2017

## Contents

1 Quaternion definition and properties 4   
1.1 Definition of quaternion . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 4   
1.1.1 Alternative representations of the quaternion. 5   
1.2 Main quaternion properties 6   
1.2.1 Sum 6   
1.2.2 Product 6   
1.2.3 Identity 8   
1.2.4 Conjugate 8   
1.2.5 Norm 8   
1.2.6 Inverse 9   
1.2.7 Unit or normalized quaternion 9   
1.3 Additional quaternion properties 9   
1.3.1 Quaternion commutator 9   
1.3.2 Product of pure quaternions 9   
1.3.3 Natural powers of pure quaternions 10   
1.3.4 Exponential of pure quaternions 10   
1.3.5 Exponential of general quaternions 11   
1.3.6 Logarithm of unit quaternions 11   
1.3.7 Logarithm of general quaternions 12   
1.3.8 Exponential forms of the type $\mathbf{q}^t$ 12   
2 Rotations and cross-relations 12   
2.1 The 3D vector rotation formula 12   
2.2 The rotation group $SO(3)$ 13   
2.3 The rotation group and the rotation matrix 14   
2.3.1 The exponential map 16   
2.3.2 The capitalized exponential map 18   
2.3.3 Rotation matrix and rotation vector: the Rodrigues rotation formula 18   
2.3.4 The logarithmic maps 19   
2.3.5 The rotation action 19

2.4 The rotation group and the quaternion . 20   
2.4.1 The exponential map . 20   
2.4.2 The capitalized exponential map . 21   
2.4.3 Quaternion and rotation vector . 22   
2.4.4 The logarithmic maps . 22   
2.4.5 The rotation action . 23   
2.4.6 The double cover of the manifold of $SO(3)$ . 24   
2.5 Rotation matrix and quaternion . 25   
2.6 Rotation composition . 26   
2.7 Spherical linear interpolation (SLERP) . 27   
2.8 Quaternion and isoclinic rotations: explaining the magic . 29   
3 Quaternion conventions. My choice. 33   
3.1 Quaternion flavors . 33   
3.1.1 Order of the quaternion components . 35   
3.1.2 Specification of the quaternion algebra . 36   
3.1.3 Function of the rotation operator . 36   
3.1.4 Direction of the rotation operator . 37   
4 Perturbations, derivatives and integrals 38   
4.1 The additive and subtractive operators in $SO(3)$ . 38   
4.2 The four possible derivative definitions . 39   
4.2.1 Functions from vector space to vector space . 39   
4.2.2 Functions from $SO(3)$ to $SO(3)$ . 39   
4.2.3 Functions from vector space to $SO(3)$ . 40   
4.2.4 Functions from $SO(3)$ to vector space . 40   
4.3 Useful, and very useful, Jacobians of the rotation . 40   
4.3.1 Jacobian with respect to the vector . 40   
4.3.2 Jacobian with respect to the quaternion . 41   
4.3.3 Right Jacobian of $SO(3)$ . 41   
4.3.4 Jacobian with respect to the rotation vector . 43   
4.3.5 Jacobians of the rotation composition . 43   
4.4 Perturbations, uncertainties, noise . 44   
4.4.1 Local perturbations . 44   
4.4.2 Global perturbations . 45   
4.5 Time derivatives . 45   
4.5.1 Global-to-local relations . 47   
4.5.2 Time-derivative of the quaternion product . 47   
4.5.3 Other useful expressions with the derivative . 47   
4.6 Time-integration of rotation rates . 48   
4.6.1 Zeroth order integration . 48   
4.6.2 First order integration . 50

5 Error-state kinematics for IMU-driven systems 52
5.1 Motivation . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 52
5.2 The error-state Kalman filter explained . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 53
5.3 System kinematics in continuous time. 53
5.3.1 The true-state kinematics. 54
5.3.2 The nominal-state kinematics. 56
5.3.3 The error-state kinematics. 56
5.4 System kinematics in discrete time. 59
5.4.1 The nominal state kinematics. 60
5.4.2 The error-state kinematics. 60
5.4.3 The error-state Jacobian and perturbation matrices. 61
6 Fusing IMU with complementary sensory data 62
6.1 Observation of the error state via filter correction. 62
6.1.1 Jacobian computation for the filter correction. 63
6.2 Injection of the observed error into the nominal state. 64
6.3 ESKF reset. 64
6.3.1 Jacobian of the reset operation with respect to the orientation error 65
7 The ESKF using global angular errors 66
7.1 System kinematics in continuous time. 66
7.1.1 The true- and nominal-state kinematics. 66
7.1.2 The error-state kinematics. 67
7.2 System kinematics in discrete time. 69
7.2.1 The nominal state. 69
7.2.2 The error state. 69
7.2.3 The error state Jacobian and perturbation matrices. 69
7.3 Fusing with complementary sensory data. 70
7.3.1 Error state observation. 70
7.3.2 Injection of the observed error into the nominal state. 71
7.3.3 ESKF reset. 71
A Runge-Kutta numerical integration methods 72
A.1 The Euler method. 73
A.2 The midpoint method. 73
A.3 The RK4 method. 74
A.4 General Runge-Kutta method. 74
B Closed-form integration methods 75
B.1 Integration of the angular error. 76
B.2 Simplified IMU example. 76
B.3 Full IMU example. 80

C Approximate methods using truncated series 82  
C.1 System-wise truncation . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . C.1.1 First order truncation: the finite differences method . . . . . . . . . . . . . . . . . . . . . . . C.1.2 N-th order truncation . . . . . . . . . . . . . . . . . . . . . . . . . . . C.2 Block-wise truncation. 84   
D The transition matrix via Runge-Kutta integration 86  
D.1 Error-state example. 87   
E Integration of random noise and perturbations 88  
E.1 Noise and perturbation impulses. 91  
E.2 Full IMU example. 91  
E.2.1 Noise and perturbation impulses. 92

## Abstract

This article is an exhaustive revision of concepts and formulas related to quaternions and rotations in 3D space, and their proper use in estimation engines such as the error-state Kalman filter.

The paper includes an in-depth study of the rotation group and its Lie structure, with formulations using both quaternions and rotation matrices. It makes special attention in the definition of rotation perturbations, derivatives and integrals. It provides numerous intuitions and geometrical interpretations to help the reader grasp the inner mechanisms of 3D rotation.

The whole material is used to devise precise formulations for error-state Kalman filters suited for real applications using integration of signals from an inertial measurement unit (IMU).

## 1 Quaternion definition and properties

## 1.1 Definition of quaternion

One introduction to the quaternion that I find particularly attractive is given by the Cayley-Dickson construction: If we have two complex numbers $A = a + bi$ and $C = c + di$ , then constructing $Q = A + Cj$ and defining $k \triangleq ij$ yields a number in the space of quaternions H,

$$
Q = a + b i + c j + d k \in \mathbb {H},\tag{1}
$$

where $\{a,b,c,d\} \in \mathbb{R}$ , and $\{i,j,k\}$ are three imaginary unit numbers defined so that

$$
i ^ {2} = j ^ {2} = k ^ {2} = i j k = - 1,\tag{2a}
$$

from which we can derive

$$
i j = - j i = k, \quad j k = - k j = i, \quad k i = - i k = j.\tag{2b}
$$

From (1) we see that we can embed complex numbers, and thus real and imaginary numbers, in the quaternion definition, in the sense that real, imaginary and complex numbers are indeed quaternions,

$$
Q = a \in \mathbb {R} \subset \mathbb {H}, \qquad Q = b i \in \mathbb {I} \subset \mathbb {H}, \qquad Q = a + b i \in \mathbb {Z} \subset \mathbb {H}.\tag{3}
$$

Likewise, and for the sake of completeness, we may define numbers in the tri-dimensional imaginary subspace of H. We refer to them as pure quaternions, and may note $\mathbb{H}_{p} = \operatorname{Im}(\mathbb{H})$ the space of pure quaternions,

$$
Q = b i + c j + d k \in \mathbb {H} _ {p} \subset \mathbb {H}.\tag{4}
$$

It is noticeable that, while regular complex numbers of unit length $z = e^{i\theta}$ can encode rotations in the 2D plane (with one complex product, $x' = z \cdot x$ ), “extended complex numbers” or quaternions of unit length $\mathbf{q} = e^{(u_x i + u_y j + u_z k)\theta/2}$ encode rotations in the 3D space (with a double quaternion product, $x' = q \otimes x \otimes q^*$ , as we explain later in this document).

CAUTION: Not all quaternion definitions are the same. Some authors write the products as ib instead of bi, and therefore they get the property $k = ji = -ij$ , which results in ijk = 1 and a left-handed quaternion. Also, many authors place the real part at the end position, yielding $Q = ia + jb + kc + d$ . These choices have no fundamental implications but make the whole formulation different in the details. Please refer to Section 3 for further explanations and disambiguation.

CAUTION: There are additional conventions that also make the formulation different in details. They concern the “meaning” or “interpretation” we give to the rotation operators, either rotating vectors or rotating reference frames – which, essentially, constitute opposite operations. Refer also to Section 3 for further explanations and disambiguation.

NOTE: Among the different conventions exposed above, this document concentrates on the Hamilton convention, whose most remarkable property is the definition (2). A proper and grounded disambiguation requires to first develop a significant amount of material; therefore, this disambiguation is relegated to the aforementioned Section 3.

## 1.1.1 Alternative representations of the quaternion

The real + imaginary notation $\{1, i, j, k\}$ is not always convenient for our purposes. Provided that the algebra (2) is used, a quaternion can be posed as a sum scalar + vector,

$$
Q = q _ {w} + q _ {x} i + q _ {y} j + q _ {z} k \quad \Leftrightarrow \quad Q = q _ {w} + {\bf q} _ {v},\tag{5}
$$

where $q_{w}$ is referred to as the real or scalar part, and $\mathbf{q}_{v}=q_{x}i+q_{y}j+q_{z}k=(q_{x},q_{y},q_{z})$ as the imaginary or vector part. $^{1}$ It can be also defined as an ordered pair scalar-vector

$$
Q = \left\langle q _ {w}, \mathbf {q} _ {v} \right\rangle .\tag{6}
$$

We mostly represent a quaternion $Q$ as a 4-vector $\mathbf{q}$ ,

$$
\mathbf {q} \triangleq \left[ \begin{array}{c} q _ {w} \\ \mathbf {q} _ {v} \end{array} \right] = \left[ \begin{array}{c} q _ {w} \\ q _ {x} \\ q _ {y} \\ q _ {z} \end{array} \right],\tag{7}
$$

which allows us to use matrix algebra for operations involving quaternions. At certain occasions, we may allow ourselves to mix notations by abusing of the sign “=”. Typical examples are real quaternions and pure quaternions,

$$
\text { general: } \mathbf {q} = q _ {w} + \mathbf {q} _ {v} = \left[ \begin{array}{c} q _ {w} \\ \mathbf {q} _ {v} \end{array} \right] \in \mathbb {H}, \quad \text { real: } q _ {w} = \left[ \begin{array}{c} q _ {w} \\ \mathbf {0} _ {v} \end{array} \right] \in \mathbb {R}, \quad \text { pure: } \mathbf {q} _ {v} = \left[ \begin{array}{c} 0 \\ \mathbf {q} _ {v} \end{array} \right] \in \mathbb {H} _ {p}.\tag{8}
$$

## 1.2 Main quaternion properties

## 1.2.1 Sum

The sum is straightforward,

$$
\mathbf {p} \pm \mathbf {q} = \left[ \begin{array}{c} p _ {w} \\ \mathbf {p} _ {v} \end{array} \right] \pm \left[ \begin{array}{c} q _ {w} \\ \mathbf {q} _ {v} \end{array} \right] = \left[ \begin{array}{c} p _ {w} \pm q _ {w} \\ \mathbf {p} _ {v} \pm \mathbf {q} _ {v} \end{array} \right].\tag{9}
$$

By construction, the sum is commutative and associative,

$$
\mathbf {p} + \mathbf {q} = \mathbf {q} + \mathbf {p}\tag{10}
$$

$$
\mathbf {p} + (\mathbf {q} + \mathbf {r}) = (\mathbf {p} + \mathbf {q}) + \mathbf {r}.\tag{11}
$$

## 1.2.2 Product

Denoted by $\otimes$ , the quaternion product requires using the original form (1) and the quaternion algebra (2). Writing the result in vector form gives

$$
\mathbf {p} \otimes \mathbf {q} = \left[ \begin{array}{l} p _ {w} q _ {w} - p _ {x} q _ {x} - p _ {y} q _ {y} - p _ {z} q _ {z} \\ p _ {w} q _ {x} + p _ {x} q _ {w} + p _ {y} q _ {z} - p _ {z} q _ {y} \\ p _ {w} q _ {y} - p _ {x} q _ {z} + p _ {y} q _ {w} + p _ {z} q _ {x} \\ p _ {w} q _ {z} + p _ {x} q _ {y} - p _ {y} q _ {x} + p _ {z} q _ {w} \end{array} \right].\tag{12}
$$

This can be posed also in terms of the scalar and vector parts,

$$
\mathbf {p} \otimes \mathbf {q} = \left[ \begin{array}{c} p _ {w} q _ {w} - \mathbf {p} _ {v} ^ {\top} \mathbf {q} _ {v} \\ p _ {w} \mathbf {q} _ {v} + q _ {w} \mathbf {p} _ {v} + \mathbf {p} _ {v} \times \mathbf {q} _ {v} \end{array} \right],\tag{13}
$$

where the presence of the cross-product reveals that the quaternion product is not commutative in the general case,

$$
\mathbf {p} \otimes \mathbf {q} \neq \mathbf {q} \otimes \mathbf {p}.\tag{14}
$$

Exceptions to this general non-commutativity are limited to the cases where $p_{v} \times q_{v} = 0$ , which happens whenever one quaternion is real, $p = p_{w}$ or $q = q_{w}$ , or when both vector parts are parallel, $p_{v} \|q_{v}$ . Only in these cases the quaternion product is commutative.

The quaternion product is however associative,

$$
(\mathbf {p} \otimes \mathbf {q}) \otimes \mathbf {r} = \mathbf {p} \otimes (\mathbf {q} \otimes \mathbf {r}),\tag{15}
$$

and distributive over the sum,

$$
\mathbf {p} \otimes (\mathbf {q} + \mathbf {r}) = \mathbf {p} \otimes \mathbf {q} + \mathbf {p} \otimes \mathbf {r} \quad \text { and } \quad (\mathbf {p} + \mathbf {q}) \otimes \mathbf {r} = \mathbf {p} \otimes \mathbf {r} + \mathbf {q} \otimes \mathbf {r} .\tag{16}
$$

The product of two quaternions is bi-linear and can be expressed as two equivalent matrix products, namely

$$
\mathbf {q} _ {1} \otimes \mathbf {q} _ {2} = [ \mathbf {q} _ {1} ] _ {L} \mathbf {q} _ {2} \quad \text {and} \quad \mathbf {q} _ {1} \otimes \mathbf {q} _ {2} = [ \mathbf {q} _ {2} ] _ {R} \mathbf {q} _ {1},\tag{17}
$$

where $[q]_{L}$ and $[q]_{R}$ are respectively the left- and right-quaternion-product matrices, which are derived from (12) and (17) by simple inspection,

$$
[ \mathbf {q} ] _ {L} = \left[ \begin{array}{c c c c} q _ {w} & - q _ {x} & - q _ {y} & - q _ {z} \\ q _ {x} & q _ {w} & - q _ {z} & q _ {y} \\ q _ {y} & q _ {z} & q _ {w} & - q _ {x} \\ q _ {z} & - q _ {y} & q _ {x} & q _ {w} \end{array} \right], \qquad [ \mathbf {q} ] _ {R} = \left[ \begin{array}{c c c c} q _ {w} & - q _ {x} & - q _ {y} & - q _ {z} \\ q _ {x} & q _ {w} & q _ {z} & - q _ {y} \\ q _ {y} & - q _ {z} & q _ {w} & q _ {x} \\ q _ {z} & q _ {y} & - q _ {x} & q _ {w} \end{array} \right],\tag{18}
$$

or more concisely, from (13) and (17),

$$
[ \mathbf {q} ] _ {L} = q _ {w}   \mathbf {I} + \left[ \begin{array}{c c} 0 & - \mathbf {q} _ {v} ^ {\top} \\ \mathbf {q} _ {v} & [ \mathbf {q} _ {v} ] _ {\times} \end{array} \right], \qquad [ \mathbf {q} ] _ {R} = q _ {w}   \mathbf {I} + \left[ \begin{array}{c c} 0 & - \mathbf {q} _ {v} ^ {\top} \\ \mathbf {q} _ {v} & - [ \mathbf {q} _ {v} ] _ {\times} \end{array} \right] .\tag{19}
$$

Here, the skew operator $^{2}$ $[\bullet]_{\times}$ produces the cross-product matrix,

$$
\left[ \mathbf {a} \right] _ {\times} \triangleq \left[ \begin{array}{c c c} 0 & - a _ {z} & a _ {y} \\ a _ {z} & 0 & - a _ {x} \\ - a _ {y} & a _ {x} & 0 \end{array} \right],\tag{20}
$$

which is a skew-symmetric matrix, $[a]_{\times}^{\top} = -[a]_{\times}$ , equivalent to the cross product, i.e.,

$$
[ \mathbf {a} ] _ {\times} \mathbf {b} = \mathbf {a} \times \mathbf {b}, \quad \forall \mathbf {a}, \mathbf {b} \in \mathbb {R} ^ {3}.\tag{21}
$$

Finally, since

$$
\begin{array}{c} \mathbf {q} \otimes \mathbf {x} \otimes \mathbf {p} = (\mathbf {q} \otimes \mathbf {x}) \otimes \mathbf {p} = [ \mathbf {p} ] _ {R} [ \mathbf {q} ] _ {L} \mathbf {x} \\ = \mathbf {q} \otimes (\mathbf {x} \otimes \mathbf {p}) = [ \mathbf {q} ] _ {L} [ \mathbf {p} ] _ {R} \mathbf {x}, \end{array}
$$

we have the relation

$$
[ \mathbf {p} ] _ {R} [ \mathbf {q} ] _ {L} = [ \mathbf {q} ] _ {L} [ \mathbf {p} ] _ {R},\tag{22}
$$

that is, left- and right- quaternion product matrices commute. Further properties of these matrices are provided in Section 2.8.

Quaternions endowed with the product operation $\otimes$ form a non-commutative group. The group's elements identity, $\mathbf{q}_1 = 1$ , and inverse, $\mathbf{q}^{-1}$ , are explored below.

## 1.2.3 Identity

The identity quaternion $q_{1}$ with respect to the product is such that $q_{1} \otimes q = q \otimes q_{1} = q$ . It corresponds to the real product identity ‘1’ expressed as a quaternion,

$$
\mathbf {q} _ {1} = 1 = \left[ \begin{array}{c} 1 \\ \mathbf {0} _ {v} \end{array} \right] .
$$

## 1.2.4 Conjugate

The conjugate of a quaternion is defined by

$$
\mathbf {q} ^ {*} \triangleq q _ {w} - \mathbf {q} _ {v} = \left[ \begin{array}{c} q _ {w} \\ - \mathbf {q} _ {v} \end{array} \right] .\tag{23}
$$

This has the properties

$$
\mathbf {q} \otimes \mathbf {q} ^ {*} = \mathbf {q} ^ {*} \otimes \mathbf {q} = q _ {w} ^ {2} + q _ {x} ^ {2} + q _ {y} ^ {2} + q _ {z} ^ {2} = \left[ \begin{array}{c} q _ {w} ^ {2} + q _ {x} ^ {2} + q _ {y} ^ {2} + q _ {z} ^ {2} \\ \mathbf {0} _ {v} \end{array} \right],\tag{24}
$$

and

$$
(\mathbf {p} \otimes \mathbf {q}) ^ {*} = \mathbf {q} ^ {*} \otimes \mathbf {p} ^ {*}.\tag{25}
$$

## 1.2.5 Norm

The norm of a quaternion is defined by

$$
\| \mathbf {q} \| \triangleq \sqrt {\mathbf {q} \otimes \mathbf {q} ^ {*}} = \sqrt {\mathbf {q} ^ {*} \otimes \mathbf {q}} = \sqrt {q _ {w} ^ {2} + q _ {x} ^ {2} + q _ {y} ^ {2} + q _ {z} ^ {2}} \in \mathbb {R}.\tag{26}
$$

It has the property

$$
\left\| \mathbf {p} \otimes \mathbf {q} \right\| = \left\| \mathbf {q} \otimes \mathbf {p} \right\| = \left\| \mathbf {p} \right\| \left\| \mathbf {q} \right\|.\tag{27}
$$

## 1.2.6 Inverse

The inverse quaternion $q^{-1}$ is such that the quaternion times its inverse gives the identity,

$$
\mathbf {q} \otimes \mathbf {q} ^ {- 1} = \mathbf {q} ^ {- 1} \otimes \mathbf {q} = \mathbf {q} _ {1}.\tag{28}
$$

It can be computed with

$$
\mathbf {q} ^ {- 1} = \mathbf {q} ^ {*} / \| \mathbf {q} \| ^ {2}.\tag{29}
$$

## 1.2.7 Unit or normalized quaternion

For unit quaternions, $\|q\|=1$ , and therefore

$$
\mathbf {q} ^ {- 1} = \mathbf {q} ^ {*}.\tag{30}
$$

When interpreting the unit quaternion as an orientation specification, or as a rotation operator, this property implies that the inverse rotation can be accomplished with the conjugate quaternion. Unit quaternions can always be written in the form,

$$
\mathbf {q} = \left[ \begin{array}{c} \cos \theta \\ \mathbf {u} \sin \theta \end{array} \right],\tag{31}
$$

where $\mathbf{u} = u_{x}i + u_{y}j + u_{z}k$ is a unit vector and $\theta$ is a scalar.

From (27), unit quaternions endowed with the product operation $\otimes$ form a non commutative group, where the inverse coincides with the conjugate.

## 1.3 Additional quaternion properties

## 1.3.1 Quaternion commutator

The quaternion commutator is defined as $[p,q] \triangleq p \otimes q - q \otimes p$ . We have from (13),

$$
\mathbf {p} \otimes \mathbf {q} - \mathbf {q} \otimes \mathbf {p} = 2 \mathbf {p} _ {v} \times \mathbf {q} _ {v}.\tag{32}
$$

This has as a trivial consequence,

$$
\mathbf {p} _ {v} \otimes \mathbf {q} _ {v} - \mathbf {q} _ {v} \otimes \mathbf {p} _ {v} = 2 \mathbf {p} _ {v} \times \mathbf {q} _ {v}.\tag{33}
$$

We will use this property later on.

## 1.3.2 Product of pure quaternions

Pure quaternions are those with null real or scalar part, $Q = q_{v}$ or $q = [0, q_{v}]$ . We have from (13),

$$
\mathbf {p} _ {v} \otimes \mathbf {q} _ {v} = - \mathbf {p} _ {v} ^ {\top} \mathbf {q} _ {v} + \mathbf {p} _ {v} \times \mathbf {q} _ {v} = \left[ \begin{array}{c} - \mathbf {p} _ {v} ^ {\top} \mathbf {q} _ {v} \\ \mathbf {p} _ {v} \times \mathbf {q} _ {v} \end{array} \right] .\tag{34}
$$

This implies

$$
\mathbf {q} _ {v} \otimes \mathbf {q} _ {v} = - \mathbf {q} _ {v} ^ {\top} \mathbf {q} _ {v} = - \| \mathbf {q} _ {v} \| ^ {2},\tag{35}
$$

and for pure unitary quaternions $u \in H_{p}$ , $\|u\| = 1$ ,

$$
\mathbf {u} \otimes \mathbf {u} = - 1,\tag{36}
$$

which is analogous to the standard imaginary case, $i \cdot i = -1$ .

## 1.3.3 Natural powers of pure quaternions

Let us define $q^{n}$ , $n \in N$ , as the n-th power of q using the quaternion product $\otimes$ . Then, if v is a pure quaternion and we let $v = u\theta$ , with $\theta = \|v\| \in R$ and u unitary, we get from (35) the cyclic pattern

$$
\mathbf {v} ^ {2} = - \theta^ {2}, \quad \mathbf {v} ^ {3} = - \mathbf {u} \theta^ {3}, \quad \mathbf {v} ^ {4} = \theta^ {4}, \quad \mathbf {v} ^ {5} = \mathbf {u} \theta^ {5}, \quad \mathbf {v} ^ {6} = - \theta^ {6}, \quad \dots\tag{37}
$$

and for pure unitary quaternions u, this reduces to the pattern

$$
\mathbf {u} ^ {2} = - 1, \quad \mathbf {u} ^ {3} = - \mathbf {u}, \quad \mathbf {u} ^ {4} = 1, \quad \mathbf {u} ^ {5} = \mathbf {u}, \quad \mathbf {u} ^ {6} = - 1, \quad \dots\tag{38}
$$

## 1.3.4 Exponential of pure quaternions

The quaternion exponential is a function on quaternions analogous to the ordinary exponential function. Exactly as in the real exponential case, it is defined as the absolutely convergent power series,

$$
e ^ {\mathbf {q}} \triangleq \sum_ {k = 0} ^ {\infty} \frac {1}{k !} \mathbf {q} ^ {k} \quad \in \mathbb {H}.\tag{39}
$$

Clearly, the exponential of a real quaternion coincides exactly with the ordinary exponential function.

More interestingly, the exponential of a pure quaternion $v = v_{x}i + v_{y}j + v_{z}k$ is a new quaternion defined by,

$$
e ^ {\mathbf {v}} = \sum_ {k = 0} ^ {\infty} \frac {1}{k !} \mathbf {v} ^ {k} \quad \in \mathbb {H}.\tag{40}
$$

Letting $v = u \theta$ , with $\theta = \|v\| \in R$ and u unitary, and considering (37), we group the scalar and vector terms in the series,

$$
e ^ {\mathbf {u} \theta} = \left(1 - \frac {\theta^ {2}}{2 !} + \frac {\theta^ {4}}{4 !} + \dots\right) + \left(\mathbf {u} \theta - \frac {\mathbf {u} \theta^ {3}}{3 !} + \frac {\mathbf {u} \theta^ {5}}{5 !} + \dots\right)\tag{41}
$$

and recognize in them, respectively, the series of $\cos\theta$ and $\sin\theta$ .³ This results in

$$
e ^ {\mathbf {v}} = e ^ {\mathbf {u} \theta} = \cos \theta + \mathbf {u} \sin \theta = \left[ \begin{array}{c} \cos \theta \\ \mathbf {u} \sin \theta \end{array} \right],\tag{42}
$$

which constitutes a beautiful extension of the Euler formula, $e^{i\theta} = \cos\theta + i\sin\theta$ , defined for imaginary numbers. Notice that since $\|e^{v}\|^{2} = \cos^{2}\theta + \sin^{2}\theta = 1$ , the exponential of a pure quaternion is a unit quaternion. Notice also the property,

$$
e ^ {- \mathbf {v}} = (e ^ {\mathbf {v}}) ^ {*}.\tag{43}
$$

For small angle quaternions we avoid the division by zero in $u = v / \|v\|$ by expressing the Taylor series of $\sin\theta$ and $\cos\theta$ and truncating, obtaining varying degrees of the approximation,

$$
e ^ {\mathbf {v}} \approx \left[ \begin{array}{c} 1 - \theta^ {2} / 2 \\ \mathbf {v} (1 - \theta^ {2} / 6) \end{array} \right] \approx \left[ \begin{array}{c} 1 \\ \mathbf {v} \end{array} \right] \xrightarrow [ \theta \to 0 ]{} \left[ \begin{array}{c} 1 \\ \mathbf {0} \end{array} \right] .\tag{44}
$$

## 1.3.5 Exponential of general quaternions

Due to the non-commutativity property of the quaternion product, we cannot write for general quaternions p and q that $e^{p+q} = e^{p}e^{q}$ . However, commutativity holds when any of the product members is a scalar, and therefore,

$$
e ^ {\mathbf {q}} = e ^ {q _ {w} + \mathbf {q} _ {v}} = e ^ {q _ {w}} e ^ {\mathbf {q} _ {v}}.\tag{45}
$$

Then, using (42) with $u\theta = q_{v}$ we get

$$
e ^ {\mathbf {q}} = e ^ {q _ {w}} \left[ \begin{array}{c} \cos \left\| \mathbf {q} _ {v} \right\| \\ \frac {\mathbf {q} _ {v}}{\left\| \mathbf {q} _ {v} \right\|} \sin \left\| \mathbf {q} _ {v} \right\| \end{array} \right] .\tag{46}
$$

## 1.3.6 Logarithm of unit quaternions

It is immediate to see that, if $\|q\|=1$ ,

$$
\log \mathbf {q} = \log (\cos \theta + \mathbf {u} \sin \theta) = \log (e ^ {\mathbf {u} \theta}) = \mathbf {u} \theta = \left[ \begin{array}{l} 0 \\ \mathbf {u} \theta \end{array} \right],\tag{47}
$$

that is, the logarithm of a unit quaternion is a pure quaternion. The angle-axis values are obtained easily by inverting (42),

$$
\mathbf {u} = \mathbf {q} _ {v} / \| \mathbf {q} _ {v} \|\tag{48}
$$

$$
\theta = \arctan (\| \mathbf {q} _ {v} \|, q _ {w}).\tag{49}
$$

For small angle quaternions, we avoid division by zero by expressing the Taylor series of $\arctan(x)$ and truncating, $^{4}$ obtaining varying degrees of the approximation,

$$
\log (\mathbf {q}) = \mathbf {u} \theta = \mathbf {q} _ {v} \frac {\arctan (\| \mathbf {q} _ {v} \| , q _ {w})}{\| \mathbf {q} _ {v} \|} \approx \frac {\mathbf {q} _ {v}}{q _ {w}} \left(1 - \frac {\| \mathbf {q} _ {v} \| ^ {2}}{3 q _ {w} ^ {2}}\right) \approx \mathbf {q} _ {v} \xrightarrow [ \theta \to 0 ]{} \mathbf {0} .\tag{50}
$$

## 1.3.7 Logarithm of general quaternions

By extension, if $\mathbf{q}$ is a general quaternion,

$$
\log \mathbf {q} = \log (\| \mathbf {q} \| \frac {\mathbf {q}}{\| \mathbf {q} \|}) = \log \| \mathbf {q} \| + \log \frac {\mathbf {q}}{\| \mathbf {q} \|} = \log \| \mathbf {q} \| + \mathbf {u} \theta = \left[ \begin{array}{c} \log \| \mathbf {q} \| \\ \mathbf {u} \theta \end{array} \right].\tag{51}
$$

## 1.3.8 Exponential forms of the type $q^{t}$

We have, for $q \in H$ and $t \in R$ ,

$$
\mathbf {q} ^ {t} = \exp (\log (\mathbf {q} ^ {t})) = \exp (t \log (\mathbf {q})).\tag{52}
$$

If $\|q\|=1$ , we can write $q=\left[\cos\theta,\mathbf{u}\sin\theta\right]$ , thus $\log(\mathbf{q})=\mathbf{u}\theta$ , which gives

$$
\mathbf {q} ^ {t} = \exp (t   \mathbf {u} \theta) = \left[ \begin{array}{c} \cos t \theta \\ \mathbf {u} \sin t \theta \end{array} \right]  .\tag{53}
$$

Because the exponent t has ended up as a linear multiplier of the angle $\theta$ , it can be seen as a linear angular interpolator. We will develop this idea in Section 2.7.

## 2 Rotations and cross-relations

## 2.1 The 3D vector rotation formula

![](images/0e80eaead042a618942279e3b1c59c75ed9aef517d7b3602d8d4beb495abe806.jpg)  
Figure 1: Rotation of a vector x, by an angle $\phi$ , around the axis u. See text for details.

We illustrate in Fig. 1 the rotation, following the right-hand rule, of a general 3D vector $\mathbf{x}$ , by an angle $\phi$ , around the axis defined by the unit vector $\mathbf{u}$ . This is accomplished by decomposing the vector $\mathbf{x}$ into a part $\mathbf{x}_{||}$ parallel to $\mathbf{u}$ , and a part $\mathbf{x}_{\perp}$ orthogonal to $\mathbf{u}$ , so that

$$
\mathbf {x} = \mathbf {x} _ {| |} + \mathbf {x} _ {\perp}.
$$

These parts can be computed easily ( $\alpha$ is the angle between the vector x and the axis u),

$$
\begin{array}{r} \mathbf {x} _ {| |} = \mathbf {u} \left(\| \mathbf {x} \| \cos \alpha\right) = \mathbf {u} \mathbf {u} ^ {\top} \mathbf {x} \\ \mathbf {x} _ {\perp} = \mathbf {x} - \mathbf {x} _ {| |} = \mathbf {x} - \mathbf {u} \mathbf {u} ^ {\top} \mathbf {x}. \end{array}
$$

Upon rotation, the parallel part does not rotate,

$$
\mathbf {x} _ {| |} ^ {\prime} = \mathbf {x} _ {| |},
$$

and the orthogonal part experiences a planar rotation in the plane normal to u. That is, if we create an orthogonal base $\{e_{1}, e_{2}\}$ of this plane with

$$
\begin{array}{l} \mathbf {e} _ {1} = \mathbf {x} _ {\perp} \\ \mathbf {e} _ {2} = \mathbf {u} \times \mathbf {x} _ {\perp} = \mathbf {u} \times \mathbf {x}, \end{array}
$$

satisfying $\|e_{1}\|=\|e_{2}\|$ , then $x_{\perp}=e_{1}\cdot1+e_{2}\cdot0$ . A rotation of $\phi$ rad on this plane produces,

$$
\mathbf {x} _ {\perp} ^ {\prime} = \mathbf {e} _ {1} \cos \phi + \mathbf {e} _ {2} \sin \phi ,
$$

which develops as,

$$
\mathbf {x} _ {\perp} ^ {\prime} = \mathbf {x} _ {\perp} \cos \phi + (\mathbf {u} \times \mathbf {x}) \sin \phi .
$$

Adding the parallel part yields the expression of the rotated vector, $x' = x_{||}' + x_{\perp}'$ , which is known as the vector rotation formula,

$$
\boxed {\mathbf {x} ^ {\prime} = \mathbf {x} _ {| |} + \mathbf {x} _ {\perp} \cos \phi + (\mathbf {u} \times \mathbf {x}) \sin \phi}.\tag{54}
$$

## 2.2 The rotation group $SO(3)$

In $R^{3}$ , the rotation group $SO(3)$ is the group of rotations around the origin under the operation of composition. Rotations are linear transformations that preserve vector length and relative vector orientation (i.e., handedness). Its importance in robotics is that it represents rotations of rigid bodies in 3D space: a rigid motion requires precisely that distances, angles and relative orientations within a rigid body be preserved upon motion—otherwise, if norms, angles or relative orientations are not kept, the body could not be considered rigid.

Let us then define rotations through an operator that satisfies these properties. A rotation operator $r : R^{3} \to R^{3}; v \mapsto r(v)$ acting on vectors $v \in R^{3}$ can be defined from the metrics of Euclidean space, constituted by the dot and cross products, as follows.

\- Rotation preserves the vector norm,

$$
\left\| r (\mathbf {v}) \right\| = \sqrt {\langle r (\mathbf {v}) , r (\mathbf {v}) \rangle} = \sqrt {\langle \mathbf {v} , \mathbf {v} \rangle} \triangleq \left\| \mathbf {v} \right\|, \quad \forall \mathbf {v} \in \mathbb {R} ^ {3}.\tag{55a}
$$

\- Rotation preserves angles between vectors,

$$
\langle r (\mathbf {v}), r (\mathbf {w}) \rangle = \langle \mathbf {v}, \mathbf {w} \rangle = \| \mathbf {v} \| \| \mathbf {w} \| \cos \alpha , \quad \forall \mathbf {v}, \mathbf {w} \in \mathbb {R} ^ {3}.\tag{55b}
$$

\- Rotation preserves the relative orientations of vectors,

$$
\mathbf {u} \times \mathbf {v} = \mathbf {w} \iff r (\mathbf {u}) \times r (\mathbf {v}) = r (\mathbf {w}).\tag{56}
$$

It is easily proved that the first two conditions are equivalent. We can thus define the rotation group $SO(3)$ as,

$$
S O (3): \left\{r: \mathbb {R} ^ {3} \to \mathbb {R} ^ {3} / \forall \mathbf {v}, \mathbf {w} \in \mathbb {R} ^ {3}, \| r (\mathbf {v}) \| = \| \mathbf {v} \|, r (\mathbf {v}) \times r (\mathbf {w}) = r (\mathbf {v} \times \mathbf {w}) \right\}.\tag{57}
$$

The rotation group is typically represented by the set of rotation matrices. However, quaternions constitute also a good representation of it. The aim of this chapter is to show that both representations are equally valid. They exhibit a lot of similarities, both conceptually and algebraically, as the reader will appreciate in Table 1. Perhaps, the most important difference is that the unit quaternion group constitutes a double cover of $SO(3)$ (thus technically not being $SO(3)$ itself), something that is not critical in most of our applications. $^{5}$ The table is inserted upfront for the sake of a rapid comparison and evaluation. The rotation matrix and quaternion representations of $SO(3)$ are explored in the following sections.

## 2.3 The rotation group and the rotation matrix

The operator $r()$ is linear, since it is defined from the scalar and vector products, which are linear. It can therefore be represented by a matrix $R \in R^{3 \times 3}$ , which produces rotations to vectors $v \in R^{3}$ through the matrix product,

$$
r (\mathbf {v}) = \mathbf {R} \mathbf {v}.\tag{58}
$$

Injecting it in (55a), using the dot product $\langle a,b\rangle=a^{\top}b$ and developing we have that for all v,

$$
\left(\mathbf {R} \mathbf {v}\right) ^ {\top} (\mathbf {R} \mathbf {v}) = \mathbf {v} ^ {\top} \mathbf {R} ^ {\top} \mathbf {R} \mathbf {v} = \mathbf {v} ^ {\top} \mathbf {v},\tag{59}
$$

yielding the orthogonality condition on R,

$$
\boxed {\mathbf {R} ^ {\top} \mathbf {R} = \mathbf {I} = \mathbf {R}   \mathbf {R} ^ {\top}}  .\tag{60}
$$

Table 1: The rotation matrix and the quaternion for representing $SO(3)$ .

<table><tr><td></td><td>Rotation matrix, R</td><td>Quaternion, q</td></tr><tr><td>Parameters</td><td>3 × 3 = 9</td><td>1 + 3 = 4</td></tr><tr><td>Degrees of freedom</td><td>3</td><td>3</td></tr><tr><td>Constraints</td><td>9 - 3 = 6</td><td>4 - 3 = 1</td></tr><tr><td>Constraints</td><td> $\mathbf{RR}^{\top} = \mathbf{I}$  ; det(R) = +1</td><td> $\mathbf{q} \otimes \mathbf{q}^{*} = 1$ </td></tr><tr><td>ODE</td><td> $\dot{\mathbf{R}} = \mathbf{R} [\omega]_{\times}$ </td><td> $\dot{\mathbf{q}} = \frac{1}{2}\mathbf{q} \otimes \boldsymbol{\omega}$ </td></tr><tr><td>Exponential map</td><td> $\mathbf{R} = \exp([\mathbf{u}\phi]_{\times})$ </td><td> $\mathbf{q} = \exp(\mathbf{u}\phi/2)$ </td></tr><tr><td>Logarithmic map</td><td> $\log(\mathbf{R}) = [\mathbf{u}\phi]_{\times}$ </td><td> $\log(\mathbf{q}) = \mathbf{u}\phi/2$ </td></tr><tr><td>Relation to SO(3)</td><td>Single cover</td><td>Double cover</td></tr><tr><td>Identity</td><td> $\mathbf{I}$ </td><td>1</td></tr><tr><td>Inverse</td><td> $\mathbf{R}^{\top}$ </td><td> $\mathbf{q}^{*}$ </td></tr><tr><td>Composition</td><td> $\mathbf{R}_{1} \mathbf{R}_{2}$ </td><td> $\mathbf{q}_{1} \otimes \mathbf{q}_{2}$ </td></tr><tr><td>Rotation operator</td><td> $\mathbf{R} = \mathbf{I} + \sin \phi [\mathbf{u}]_{\times} + (1 - \cos \phi) [\mathbf{u}]_{\times}^{2}$ </td><td> $\mathbf{q} = \cos \phi/2 + \mathbf{u} \sin \phi/2$ </td></tr><tr><td>Rotation action</td><td> $\mathbf{R} \times$ </td><td> $\mathbf{q} \otimes \mathbf{x} \otimes \mathbf{q}^{*}$ </td></tr><tr><td>Interpolation</td><td> $\mathbf{R}^{t} = \mathbf{I} + \sin t\phi [\mathbf{u}]_{\times} + (1 - \cos t\phi) [\mathbf{u}]_{\times}^{2}$  $\mathbf{R}_{1}(\mathbf{R}_{1}^{\top}\mathbf{R}_{2})^{t}$ </td><td> $\mathbf{q}^{t} = \cos t\phi/2 + \mathbf{u} \sin t\phi/2$  $\mathbf{q}_{1} \otimes (\mathbf{q}_{1}^{*} \otimes \mathbf{q}_{2})^{t}$  $\mathbf{q}_{1} \frac{\sin((1-t)\Delta\theta)}{\sin(\Delta\theta)} + \mathbf{q}_{2} \frac{\sin(t\Delta\theta)}{\sin(\Delta\theta)}$ </td></tr><tr><td>Cross relations</td><td colspan="2"> $\mathbf{R}\{\mathbf{q}\} = (q_{w}^{2} - \mathbf{q}_{v}^{\top}\mathbf{q}_{v}) \mathbf{I} + 2 \mathbf{q}_{v} \mathbf{q}_{v}^{\top} + 2 q_{w} [\mathbf{q}_{v}]_{\times}$  $\mathbf{R}\{-\mathbf{q}\} = \mathbf{R}\{\mathbf{q}\}$  double cover $\mathbf{R}\{1\} = \mathbf{I}$  identity $\mathbf{R}\{\mathbf{q}^{*}\} = \mathbf{R}\{\mathbf{q}\}^{\top}$  inverse $\mathbf{R}\{\mathbf{q}_{1} \otimes \mathbf{q}_{2}\} = \mathbf{R}\{\mathbf{q}_{1}\} \mathbf{R}\{\mathbf{q}_{2}\}$  composition $\mathbf{R}\{\mathbf{q}^{t}\} = \mathbf{R}\{\mathbf{q}\}^{t}$  interpolation</td></tr></table>

The condition above is indeed a condition of orthogonality, since we can observe from it that, by writing $R = [r_{1}, r_{2}, r_{3}]$ and substituting above, the column vectors $r_{i}$ of R, with $i \in \{1, 2, 3\}$ , are of unit length and orthogonal to each other,

$$
\begin{array}{l} \langle \mathbf {r} _ {i}, \mathbf {r} _ {i} \rangle = \mathbf {r} _ {i} ^ {\top} \mathbf {r} _ {i} = 1 \\ \langle \mathbf {r} _ {i}, \mathbf {r} _ {j} \rangle = \mathbf {r} _ {i} ^ {\top} \mathbf {r} _ {j} = 0, \quad \text {if} i \neq j. \end{array}
$$

The set of transformations keeping vector norms and angles is for this reason called the Orthogonal group, denoted $O(3)$ . The orthogonal group includes rotations (which are rigid motions) and reflections (which are not rigid). The notion of group here means essentially (and informally) that the product of two orthogonal matrices is always an orthogonal matrix, $^{6}$ and that each orthogonal matrix admits an inverse. In effect, the orthogonality condition (60) implies that the inverse rotation is achieved with the transposed matrix,

$$
\mathbf {R} ^ {- 1} = \mathbf {R} ^ {\top}.\tag{61}
$$

Adding the relative orientations condition (56) guarantees rigid body motion (hence discarding reflections), and results in one additional constraint on $R,^{7}$

$$
\boxed {\det (\mathbf {R}) = 1}.\tag{62}
$$

Orthogonal matrices with positive unit determinant are commonly referred to as proper or special. The set of such special orthogonal matrices is a subgroup of $O(3)$ named the Special Orthogonal group $SO(3)$ . Being a group, the product of two rotation matrices is always a rotation matrix. $^{8}$

## 2.3.1 The exponential map

The exponential map (and the logarithmic map, which we see in the next section) is a powerful mathematical tool for working in the rotational 3D space with ease and rigor. It represents the entrance door to a corpus of infinitesimal calculus suited for the rotational space. The exponential map allows us to properly define derivatives, perturbations, and velocities, and to manipulate them. It is therefore essential in estimation problems in the space of rotations or orientations.

Rotations constitute rigid motions. This rigidity implies that it is possible to define a continuous trajectory or path $r(t)$ in $SO(3)$ that continuously rotates the rigid body from its initial orientation, $r(0)$ , to its current orientation, $r(t)$ . Being continuous, it is legitimate to investigate the time-derivatives of such transformations. We do so by deriving the properties (60) and (62) that we have just seen.

First of all, we notice that it is impossible to continuously escape the unit determinant condition (62) while satisfying (60), because this would imply a jump of the determinant from +1 to -1. $^{9}$ Therefore we only need to investigate the time-derivative of the orthogonality condition (60). This reads

$$
\frac {d}{d t} (\mathbf {R} ^ {\top} \mathbf {R}) = \dot {\mathbf {R}} ^ {\top} \mathbf {R} + \mathbf {R} ^ {\top} \dot {\mathbf {R}} = 0,\tag{63}
$$

which results in

$$
\mathbf {R} ^ {\top} \dot {\mathbf {R}} = - (\mathbf {R} ^ {\top} \dot {\mathbf {R}}) ^ {\top},\tag{64}
$$

meaning that the matrix $R^{\top}\dot{R}$ is skew-symmetric (i.e., it is equal to the negative of its transpose). The set of skew-symmetric $3 \times 3$ matrices is denoted $\mathfrak{so}(3)$ , and receives the name of the Lie algebra of $SO(3)$ . Skew-symmetric $3 \times 3$ matrices have the form,

$$
[ \pmb {\omega} ] _ {\times} \triangleq \left[ \begin{array}{c c c} 0 & - \omega_ {z} & \omega_ {y} \\ \omega_ {z} & 0 & - \omega_ {x} \\ - \omega_ {y} & \omega_ {x} & 0 \end{array} \right];\tag{65}
$$

they have 3 DOF, and correspond to cross-product matrices, as we introduced already in (20). This establishes a one-to-one mapping $\omega \in \mathbb{R}^3 \leftrightarrow [\omega]_x \in \mathfrak{so}(3)$ . Let us then take a vector $\omega = (\omega_x, \omega_y, \omega_z) \in \mathbf{R}^3$ and write

$$
\mathbf {R} ^ {\top} \dot {\mathbf {R}} = [ \boldsymbol {\omega} ] _ {\times}.\tag{66}
$$

This leads to the ordinary differential equation (ODE),

$$
\dot {\mathbf {R}} = \mathbf {R} [ \boldsymbol {\omega} ] _ {\times}.\tag{67}
$$

Around the origin, we have $\mathbf{R} = \mathbf{I}$ and the equation above reduces to $\dot{\mathbf{R}} = [\pmb{\omega}]_{\times}$ . Thus, we can interpret the Lie algebra $\mathfrak{so}(3)$ as the space of the derivatives of $r(t)$ at the origin; it constitutes the tangent space to $SO(3)$ , or the velocity space. Following these facts, we can very well call $\pmb{\omega}$ the vector of instantaneous angular velocities.

If $\omega$ is constant, the differential equation above can be time-integrated as

$$
\mathbf {R} (t) = \mathbf {R} (0) e ^ {[ \boldsymbol {\omega} ] _ {\times} t} = \mathbf {R} (0) e ^ {[ \boldsymbol {\omega} t ] _ {\times}}\tag{68}
$$

where the exponential $e^{[x]_{\times}}$ is defined by its Taylor series, as we see in the following section. Since $\mathbf{R}(0)$ and $\mathbf{R}(t)$ are rotation matrices, then clearly $e^{[\omega t]_{\times}} = \mathbf{R}(0)^{\top} \mathbf{R}(t)$ is a rotation matrix. Defining the vector $\phi \triangleq \omega \Delta t$ as the rotation vector encoding the full rotation over the period $\Delta t$ , we have

$$
\boxed {\mathbf {R} = e ^ {[ \phi ] _ {\times}}}.\tag{69}
$$

This is known as the exponential map, an application from $\mathfrak{so}(3)$ to $SO(3)$ ,

$$
\exp : \mathfrak {s o} (3) \to S O (3); [ \phi ] _ {\times} \mapsto \exp ([ \phi ] _ {\times}) = e ^ {[ \phi ] _ {\times}}.\tag{70}
$$

![](images/4ced90d87e3e09ba7cfb4e78947428a31470053f722422e0771f39ce468e6e26.jpg)  
Figure 2: Exponential maps of the rotation matrix.

## 2.3.2 The capitalized exponential map

The exponential map above is sometimes expressed with some abuse of notation, i.e., confounding $\phi\in\mathbb{R}^{3}$ with $[\phi]_{\times}\in\mathfrak{so}(3)$ . To avoid possible ambiguities, we opt for writing this new application $\mathbb{R}^{3}\to SO(3)$ with an explicit notation using a capitalized Exp, having (see Fig. 2)

$$
\mathrm{Exp}: \mathbb {R} ^ {3} \to S O (3); \phi \mapsto \mathrm{Exp} (\phi) = e ^ {[ \phi ] _ {\times}}.\tag{71}
$$

Its relation with the exponential map is trivial,

$$
\mathrm{Exp} (\phi) \triangleq \mathrm{exp} ([ \phi ] _ {\times}).\tag{72}
$$

In the following sections we'll see that the vector $\phi$ , called the rotation vector or the angle-axis vector, encodes through $\phi = \omega \Delta t = \phi \mathbf{u}$ the angle $\phi$ and axis $\mathbf{u}$ of rotation.

## 2.3.3 Rotation matrix and rotation vector: the Rodrigues rotation formula

The rotation matrix is defined from the rotation vector $\phi = \phi u$ through the exponential map (69), with the cross-product matrix $[\phi]_{\times} = \phi[u]_{\times}$ as defined in (20). The Taylor expansion of (69) with $\phi = \phi u$ reads,

$$
\mathbf {R} = e ^ {\phi [ \mathbf {u} ] _ {\times}} = \mathbf {I} + \phi [ \mathbf {u} ] _ {\times} + \frac {1}{2} \phi^ {2} [ \mathbf {u} ] _ {\times} ^ {2} + \frac {1}{3 !} \phi^ {3} [ \mathbf {u} ] _ {\times} ^ {3} + \frac {1}{4 !} \phi^ {4} [ \mathbf {u} ] _ {\times} ^ {4} + \dots\tag{73}
$$

When applied to unit vectors, $\mathbf{u}$ , the matrix $[\mathbf{u}]_{\times}$ satisfies

$$
\left[ \mathbf {u} \right] _ {\times} ^ {2} = \mathbf {u u} ^ {\top} - \mathbf {I}\tag{74}
$$

$$
\left[ \mathbf {u} \right] _ {\times} ^ {3} = - \left[ \mathbf {u} \right] _ {\times},\tag{75}
$$

and thus all powers of $[u]_{\times}$ can be expressed in terms of $[u]_{\times}$ and $[u]_{\times}^{2}$ in a cyclic pattern,

$$
[ \mathbf {u} ] _ {\times} ^ {4} = - [ \mathbf {u} ] _ {\times} ^ {2} \qquad [ \mathbf {u} ] _ {\times} ^ {5} = [ \mathbf {u} ] _ {\times} \qquad [ \mathbf {u} ] _ {\times} ^ {6} = [ \mathbf {u} ] _ {\times} ^ {2} \qquad [ \mathbf {u} ] _ {\times} ^ {7} = - [ \mathbf {u} ] _ {\times} \dots .\tag{76}
$$

Then, grouping the Taylor series in terms of $[u]_{\times}$ and $[u]_{\times}^{2}$ , and identifying in them, respectively, the series of $\sin\phi$ and $\cos\phi$ , leads to a closed form to obtain the rotation matrix from the rotation vector, the so called Rodrigues rotation formula,

$$
\boxed {\mathbf {R} = \mathbf {I} + \sin \phi [ \mathbf {u} ] _ {\times} + (1 - \cos \phi) [ \mathbf {u} ] _ {\times} ^ {2}} ,\tag{77}
$$

which we denote $\mathbf{R}\{\phi\} \triangleq \operatorname{Exp}(\phi)$ . This formula admits some variants, e.g., using (74),

$$
\mathbf {R} = \mathbf {I} \cos \phi + [ \mathbf {u} ] _ {\times} \sin \phi + \mathbf {u u} ^ {\top} (1 - \cos \phi).\tag{78}
$$

## 2.3.4 The logarithmic maps

We define the logarithmic map as the inverse of the exponential map,

$$
\log : S O (3) \to \mathfrak {s o} (3); \mathbf {R} \mapsto \log (\mathbf {R}) = [ \mathbf {u} \phi ] _ {\times},\tag{79}
$$

with

$$
\phi = \arccos \left(\frac {\operatorname{trace} (\mathbf {R}) - 1}{2}\right)\tag{80}
$$

$$
\mathbf {u} = \frac {(\mathbf {R} - \mathbf {R} ^ {\top}) ^ {\vee}}{2 \sin \phi},\tag{81}
$$

where $\bullet^{\vee}$ is the inverse of $[\bullet]_{\times}$ , that is, $([v]_{\times})^{\vee} = v$ and $[V^{\vee}]_{\times} = V$ .

We also define a capitalized version Log, which allows us to recover the rotation vector $\phi = u\phi \in R^{3}$ directly from the rotation matrix,

$$
\mathrm{Log}: S O (3) \to \mathbb {R} ^ {3}; \mathbf {R} \mapsto \mathrm{Log} (\mathbf {R}) = \mathbf {u} \phi .\tag{82a}
$$

Its relation with the logarithmic map is trivial,

$$
\operatorname{Log} (\mathbf {R}) \triangleq (\log (\mathbf {R})) ^ {\vee}.\tag{83}
$$

## 2.3.5 The rotation action

Rotating a vector x by an angle $\phi$ around the unit axis u is performed with the linear product

$$
\mathbf {x} ^ {\prime} = \mathbf {R x},\tag{84}
$$

where $\mathbf{R} = \mathrm{Exp}(\mathbf{u}\phi)$ . This can be shown by developing (84), using (77), (74) and (75),

$$
\begin{array}{r l} & {\mathbf {x} ^ {\prime} = \mathbf {R x}} \\ & {\quad = (\mathbf {I} + \sin \phi [ \mathbf {u} ] _ {\times} + (1 - \cos \phi) [ \mathbf {u} ] _ {\times} ^ {2}) \mathbf {x}} \\ & {\quad = \mathbf {x} + \sin \phi [ \mathbf {u} ] _ {\times} \mathbf {x} + (1 - \cos \phi) [ \mathbf {u} ] _ {\times} ^ {2} \mathbf {x}} \\ & {\quad = \mathbf {x} + \sin \phi (\mathbf {u} \times \mathbf {x}) + (1 - \cos \phi) (\mathbf {u u} ^ {\top} - \mathbf {I}) \mathbf {x}} \\ & {\quad = \mathbf {x} _ {\parallel} + \mathbf {x} _ {\perp} + \sin \phi (\mathbf {u} \times \mathbf {x}) - (1 - \cos \phi) \mathbf {x} _ {\perp}} \\ & {\quad = \mathbf {x} _ {\parallel} + (\mathbf {u} \times \mathbf {x}) \sin \phi + \mathbf {x} _ {\perp} \cos \phi ,} \end{array}\tag{85}
$$

which is precisely the vector rotation formula (54).

## 2.4 The rotation group and the quaternion

For didactical purposes, we are interested in highlighting the connections between quaternions and rotation matrices as representations of the rotation group $SO(3)$ . For this, the well-known formula of the quaternion rotation action, which reads,

$$
r (\mathbf {v}) = \mathbf {q} \otimes \mathbf {v} \otimes \mathbf {q} ^ {*},\tag{86}
$$

is here taken initially as an hypothesis. This allows us to develop the full quaternion section with a discourse that retraces the one we used for the rotation matrix. The exactness of this hypothesis will be proved a little later, in Section 2.4.5, thus validating the approach.

Let us then inject the rotation above into the orthogonality condition (55a), and develop it using (27) as

$$
\left\| \mathbf {q} \otimes \mathbf {v} \otimes \mathbf {q} ^ {*} \right\| = \left\| \mathbf {q} \right\| ^ {2} \| \mathbf {v} \| = \| \mathbf {v} \|.\tag{87}
$$

This yields $\|q\|^{2}=1$ , that is, the unit norm condition on the quaternion, which reads,

$$
\boxed {\mathbf {q} ^ {*} \otimes \mathbf {q} = 1 = \mathbf {q} \otimes \mathbf {q} ^ {*}}.\tag{88}
$$

This condition is akin to the one we encountered for rotation matrices, see (60), which reads $R^{\top}R = I = RR^{\top}$ . We encourage the reader to stop at their similarities for a second.

Similarly, we show that the relative orientation condition (56) is satisfied by construction (we use (33) twice, as indicated below),

$$
\begin{array}{l l} r (\mathbf {v}) \times r (\mathbf {w}) = (\mathbf {q} \otimes \mathbf {v} \otimes \mathbf {q} ^ {*}) \times (\mathbf {q} \otimes \mathbf {w} \otimes \mathbf {q} ^ {*}) \\ (3 3) & = \frac {1}{2} \big ((\mathbf {q} \otimes \mathbf {v} \otimes \mathbf {q} ^ {*}) \otimes (\mathbf {q} \otimes \mathbf {w} \otimes \mathbf {q} ^ {*}) - (\mathbf {q} \otimes \mathbf {w} \otimes \mathbf {q} ^ {*}) \otimes (\mathbf {q} \otimes \mathbf {v} \otimes \mathbf {q} ^ {*}) \big) \\ & = \frac {1}{2} (\mathbf {q} \otimes \mathbf {v} \otimes \mathbf {w} \otimes \mathbf {q} ^ {*} - \mathbf {q} \otimes \mathbf {w} \otimes \mathbf {v} \otimes \mathbf {q} ^ {*}) \\ & = \frac {1}{2} (\mathbf {q} \otimes (\mathbf {v} \otimes \mathbf {w} - \mathbf {w} \otimes \mathbf {v}) \otimes \mathbf {q} ^ {*}) \\ (3 3) & = \mathbf {q} \otimes (\mathbf {v} \times \mathbf {w}) \otimes \mathbf {q} ^ {*} \\ & = r (\mathbf {v} \times \mathbf {w}). \end{array}\tag{89}
$$

The set of unit quaternions forms a group under the operation of multiplication. This group is topologically a 3-sphere, that is, the 3-dimensional surface of the unit sphere of $R^{4}$ , and is commonly noted as $S^{3}$ .

## 2.4.1 The exponential map

Let us consider a unit quaternion $q \in S^{3}$ , that is, $q^{*} \otimes q = 1$ , and let us proceed as we did for the orthogonality condition of the rotation matrix, $R^{\top}R = I$ . Taking the time derivative,

$$
\frac {d (\mathbf {q} ^ {*} \otimes \mathbf {q})}{d t} = \dot {\mathbf {q}} ^ {*} \otimes \mathbf {q} + \mathbf {q} ^ {*} \otimes \dot {\mathbf {q}} = 0,\tag{90}
$$

it follows that

$$
\mathbf {q} ^ {*} \otimes \dot {\mathbf {q}} = - (\dot {\mathbf {q}} ^ {*} \otimes \mathbf {q}) = - (\mathbf {q} ^ {*} \otimes \dot {\mathbf {q}}) ^ {*},\tag{91}
$$

which means that $q^{*} \otimes \dot{q}$ is a pure quaternion (i.e., it is equal to minus its conjugate, therefore its real part is zero). We thus take a pure quaternion $\Omega \in H_{p}$ and write,

$$
\mathbf {q} ^ {*} \otimes \dot {\mathbf {q}} = \boldsymbol {\Omega} = \left[ \begin{array}{c} 0 \\ \boldsymbol {\Omega} \end{array} \right] \in \mathbb {H} _ {p}.\tag{92}
$$

Left-multiplication by q yields the differential equation,

$$
\dot {\mathbf {q}} = \mathbf {q} \otimes \boldsymbol {\Omega}.\tag{93}
$$

Around the origin, we have q = 1 and the equation above reduces to $\dot{q} = \Omega \in H_{p}$ . Thus, the space $H_{p}$ of pure quaternions constitutes the tangent space, or the Lie Algebra, of the unit sphere $S^{3}$ of quaternions. In the quaternion case, however, this space is not directly the velocity space, but rather the space of the half-velocities, as we will see soon.

If $\Omega$ is constant, the differential equation can be integrated as

$$
\mathbf {q} (t) = \mathbf {q} (0) \otimes e ^ {\boldsymbol {\Omega} t},\tag{94}
$$

where, since $\mathbf{q}(0)$ and $\mathbf{q}(t)$ are unit quaternions, the exponential $e^{\Omega t}$ is also a unit quaternion —something we already knew from the quaternion exponential (42). Defining $V \triangleq \Omega \Delta t$ we have

$$
\boxed {\mathbf {q} = e ^ {\mathbf {V}}}.\tag{95}
$$

This is again an exponential map: an application from the space of pure quaternions to the space of rotations represented by unit quaternions,

$$
\exp : \mathbb {H} _ {p} \to S ^ {3}; \mathbf {V} \mapsto \exp (\mathbf {V}) = e ^ {\mathbf {V}}\tag{96}
$$

## 2.4.2 The capitalized exponential map

As we will see, the pure quaternion V in the exponential map (96) encodes, through $V = \theta u = \phi u / 2$ , the axis of rotation u and the half of the rotated angle, $\theta = \phi / 2$ . We will provide ample explanations to this half-angle fact very soon, mainly in Sections 2.4.5, 2.4.6 and 2.8. By now, let it suffice to say that, since the rotation action is accomplished by the double product $x' = q \otimes x \otimes q^*$ , the vector x experiences a rotation which is ‘twice’ the one encoded in q, or equivalently, the quaternion q encodes ‘half’ the intended rotation on x.

In order to express a direct relation between the angle-axis rotation parameters, $\phi = \phi u \in R^{3}$ , and the quaternion, we define a capitalized version of the exponential map, which captures the half-angle effect (see Fig. 3),

![](images/af34bf23b4deb07ece5da5ac8ebfa185f2ea1a9d29860f81bd106943af89b8d3.jpg)  
Figure 3: Exponential maps of the quaternion.

$$
\mathrm{Exp}: \mathbf {R} ^ {3} \to S ^ {3}; \phi \mapsto \mathrm{Exp} (\phi) = e ^ {\phi / 2}\tag{97}
$$

Its relation to the exponential map is trivial,

$$
\mathrm{Exp} (\phi) \triangleq \mathrm{exp} (\phi / 2).\tag{98}
$$

It is also convenient to introduce the vector of angular velocities $\omega = 2\Omega \in R^{3}$ , so that (93) and (94) become,

$$
\dot {\mathbf {q}} = \frac {1}{2} \mathbf {q} \otimes \boldsymbol {\omega}\tag{99}
$$

$$
\mathbf {q} = e ^ {\omega t / 2}.\tag{100}
$$

## 2.4.3 Quaternion and rotation vector

Let $\phi = \phi \mathbf{u}$ be a rotation vector representing a rotation of $\phi$ rad around the axis $\mathbf{u}$ . Then, the exponential map can be developed using an extension of the Euler formula (see (37-42) for a complete development),

$$
\boxed {\mathbf {q} \triangleq \mathrm{Exp} (\phi \mathbf {u}) = e ^ {\phi \mathbf {u} / 2} = \cos \frac {\phi}{2} + \mathbf {u} \sin \frac {\phi}{2} = \left[ \begin{array}{c} \cos (\phi / 2) \\ \mathbf {u} \sin (\phi / 2) \end{array} \right]}.\tag{101}
$$

We call this the rotation vector to quaternion conversion formula, and will be denoted in this document by $\mathbf{q} = \mathbf{q}\{\phi\} \triangleq \operatorname{Exp}(\phi)$ .

## 2.4.4 The logarithmic maps

We define the logarithmic map as the inverse of the exponential map,

$$
\log : S ^ {3} \to \mathbb {H} _ {p}; \mathbf {q} \mapsto \log (\mathbf {q}) = \mathbf {u} \theta ,\tag{102}
$$

which is of course the definition we gave for the quaternion logarithm in Section 1.3.6.

We also define the capitalized logarithmic map, which directly provides the angle $\phi$ and axis u of rotation in Cartesian 3-space,

$$
\mathrm{Log}: S ^ {3} \to \mathbb {R} ^ {3}; \mathbf {q} \mapsto \mathrm{Log} (\mathbf {q}) = \mathbf {u} \phi .\tag{103}
$$

Its relation with the logarithmic map is trivial,

$$
\mathrm{Log} (\mathbf {q}) \triangleq 2 \log (\mathbf {q}).\tag{104}
$$

For its implementation we use the 4-quadrant version of $\arctan(y, x)$ . From (101),

$$
\phi = 2 \arctan (\| \mathbf {q} _ {v} \|, q _ {w})\tag{105a}
$$

$$
\mathbf {u} = \mathbf {q} _ {v} / \| \mathbf {q} _ {v} \|.\tag{105b}
$$

For small-angle quaternions, (105b) diverges. We then use the a truncated Taylor series for the arctan() function, getting,

$$
\operatorname{Log} (\mathbf {q}) = \theta \mathbf {u} \approx 2 \frac {\mathbf {q} _ {v}}{q _ {w}} \left(1 - \frac {\| \mathbf {q} _ {v} \| ^ {2}}{3 q _ {w} ^ {2}}\right).\tag{106}
$$

## 2.4.5 The rotation action

We are finally in the position of proving our hypothesis (86) for the vector rotation using quaternions, thus validating all the material presented so far. Rotating a vector x by an angle $\phi$ around the axis u is performed with the double quaternion product, also known as the sandwich product,

$$
\mathbf {x} ^ {\prime} = \mathbf {q} \otimes \mathbf {x} \otimes \mathbf {q} ^ {*},\tag{107}
$$

where $\mathbf{q} = \operatorname{Exp}(\mathbf{u}\phi)$ , and where the vector x has been written in quaternion form, that is,

$$
\mathbf {x} = x i + y j + z k = \left[ \begin{array}{c} 0 \\ \mathbf {x} \end{array} \right] \in \mathbb {H} _ {p}.\tag{108}
$$

To show that this double product does perform the desired vector rotation, we use (13), (101), and basic vector and trigonometric identities, to develop (107) as follows,

$$
\begin{array}{r l} & {\mathbf {x} ^ {\prime} = \mathbf {q} \otimes \mathbf {x} \otimes \mathbf {q} ^ {*}} \\ & {\quad = \left(\cos \frac {\phi}{2} + \mathbf {u} \sin \frac {\phi}{2}\right) \otimes (0 + \mathbf {x}) \otimes \left(\cos \frac {\phi}{2} - \mathbf {u} \sin \frac {\phi}{2}\right)} \\ & {\quad = \mathbf {x} \cos^ {2} \frac {\phi}{2} + (\mathbf {u} \otimes \mathbf {x} - \mathbf {x} \otimes \mathbf {u}) \sin \frac {\phi}{2} \cos \frac {\phi}{2} - \mathbf {u} \otimes \mathbf {x} \otimes \mathbf {u} \sin^ {2} \frac {\phi}{2}} \\ & {\quad = \mathbf {x} \cos^ {2} \frac {\phi}{2} + 2 (\mathbf {u} \times \mathbf {x}) \sin \frac {\phi}{2} \cos \frac {\phi}{2} - (\mathbf {x} (\mathbf {u} ^ {\top} \mathbf {u}) - 2 \mathbf {u} (\mathbf {u} ^ {\top} \mathbf {x})) \sin^ {2} \frac {\phi}{2}} \\ & {\quad = \mathbf {x} (\cos^ {2} \frac {\phi}{2} - \sin^ {2} \frac {\phi}{2}) + (\mathbf {u} \times \mathbf {x}) (2 \sin \frac {\phi}{2} \cos \frac {\phi}{2}) + \mathbf {u} (\mathbf {u} ^ {\top} \mathbf {x}) (2 \sin^ {2} \frac {\phi}{2})} \\ & {\quad = \mathbf {x} \cos \phi + (\mathbf {u} \times \mathbf {x}) \sin \phi + \mathbf {u} (\mathbf {u} ^ {\top} \mathbf {x}) (1 - \cos \phi)} \\ & {\quad = (\mathbf {x} - \mathbf {u u} ^ {\top} \mathbf {x}) \cos \phi + (\mathbf {u} \times \mathbf {x}) \sin \phi + \mathbf {u u} ^ {\top} \mathbf {x}} \\ & {\quad = \mathbf {x} _ {\perp} \cos \phi + (\mathbf {u} \times \mathbf {x}) \sin \phi + \mathbf {x} _ {| |},} \end{array}\tag{109}
$$

which is precisely the vector rotation formula (54).

## 2.4.6 The double cover of the manifold of $SO(3)$ .

Consider a unit quaternion q. When regarded as a regular 4-vector, the angle $\theta$ between q and the identity quaternion $q_{1} = [1, 0, 0, 0]$ representing the origin of orientations is,

$$
\cos \theta = \mathbf {q} _ {1} ^ {\top} \mathbf {q} = \mathbf {q} (1) = q _ {w}.\tag{110}
$$

At the same time, the angle $\phi$ rotated by the quaternion q on objects in 3D space satisfies

$$
\mathbf {q} = \left[ \begin{array}{c} q _ {w} \\ \mathbf {q} _ {v} \end{array} \right] = \left[ \begin{array}{c} \cos \phi / 2 \\ \mathbf {u} \sin \phi / 2 \end{array} \right].\tag{111}
$$

That is, we have $q_{w} = \cos\theta = \cos\phi/2$ , so the angle between a quaternion vector and the identity in 4D space is half the angle rotated by the quaternion in 3D space,

$$
\theta = \phi / 2.\tag{112}
$$

We illustrate this double cover in Fig. 4. By the time the angle between the two quaternion vectors is $\theta = \pi /2$ , the 3D rotation has already achieved $\phi = \pi$ , which is half a turn. And by the time the quaternion vector has made a half turn, $\theta = \pi$ , the 3D rotation has completed a full turn. The second half turn of the quaternion vector, $\pi < \theta < 2\pi$ , represents a second full turn of the 3D rotation, $2\pi < \phi < 4\pi$ , that is, a second cover of the rotation manifold.

![](images/9fa15d53c01ae382b634f9b0450e091f213c5a3cb96f53679a7c6765f415c874.jpg)  
Figure 4: Double cover of the rotation manifold. Left: the quaternion q in the unit 3-sphere defines an angle $\theta$ with the identity quaternion $q_{1}$ . Center: the resulting 3D rotation $x' = q \otimes x \otimes q^{*}$ has double angle $\phi$ than that of the original quaternion. Right: Superposing the 4D and 3D rotation planes, observe how one turn of the quaternion q over the 3-sphere (red) represents two turns of the rotated vector x in 3D space (blue).

## 2.5 Rotation matrix and quaternion

As we have just seen, given a rotation vector $\phi = u\phi$ , the exponential maps for the unit quaternion and the rotation matrix produce rotation operators $\mathbf{q} = \text{Exp}(\mathbf{u}\phi)$ and $\mathbf{R} = \text{Exp}(\mathbf{u}\phi)$ that rotate vectors x exactly the same angle $\phi$ around the same axis $u.^{10}$ . That is, if

$$
\forall \phi , \mathbf {x} \in \mathbb {R} ^ {3}, \mathbf {q} = \mathrm{Exp} (\phi), \mathbf {R} = \mathrm{Exp} (\phi)\tag{113}
$$

then,

$$
\mathbf {q} \otimes \mathbf {x} \otimes \mathbf {q} ^ {*} = \mathbf {R x}.\tag{114}
$$

As both sides of this identity are linear in x, an expression of the rotation matrix equivalent to the quaternion is found by developing the left hand side and identifying terms on the right, yielding the quaternion to rotation matrix formula,

$$
\mathbf {R} = \left[ \begin{array}{c c c} q _ {w} ^ {2} + q _ {x} ^ {2} - q _ {y} ^ {2} - q _ {z} ^ {2} & 2 (q _ {x} q _ {y} - q _ {w} q _ {z}) & 2 (q _ {x} q _ {z} + q _ {w} q _ {y}) \\ 2 (q _ {x} q _ {y} + q _ {w} q _ {z}) & q _ {w} ^ {2} - q _ {x} ^ {2} + q _ {y} ^ {2} - q _ {z} ^ {2} & 2 (q _ {y} q _ {z} - q _ {w} q _ {x}) \\ 2 (q _ {x} q _ {z} - q _ {w} q _ {y}) & 2 (q _ {y} q _ {z} + q _ {w} q _ {x}) & q _ {w} ^ {2} - q _ {x} ^ {2} - q _ {y} ^ {2} + q _ {z} ^ {2} \end{array} \right],\tag{115}
$$

denoted throughout this document by $R = R\{q\}$ . The matrix form of the quaternion product (17–19) provides us with an alternative formula, since

$$
\mathbf {q} \otimes \mathbf {x} \otimes \mathbf {q} ^ {*} = [ \mathbf {q} ^ {*} ] _ {R} [ \mathbf {q} ] _ {L} \left[ \begin{array}{c} 0 \\ \mathbf {x} \end{array} \right] = \left[ \begin{array}{c} 0 \\ \mathbf {R x} \end{array} \right],\tag{116}
$$

which leads after some easy developments to

$$
\boxed {\mathbf {R} = (q _ {w} ^ {2} - \mathbf {q} _ {v} ^ {\top} \mathbf {q} _ {v})   \mathbf {I} + 2   \mathbf {q} _ {v} \mathbf {q} _ {v} ^ {\top} + 2   q _ {w}   [ \mathbf {q} _ {v} ] _ {\times}}    .\tag{117}
$$

The rotation matrix $\mathbf{R}$ has the following properties with respect to the quaternion,

$$
\mathbf {R} \{[ 1, 0, 0, 0 ] ^ {\top} \} = \mathbf {I}\tag{118}
$$

$$
\mathbf {R} \{- \mathbf {q} \} = \mathbf {R} \{\mathbf {q} \}\tag{119}
$$

$$
\mathbf {R} \{\mathbf {q} ^ {*} \} = \mathbf {R} \{\mathbf {q} \} ^ {\top}\tag{120}
$$

$$
\mathbf {R} \left\{\mathbf {q} _ {1} \otimes \mathbf {q} _ {2} \right\} = \mathbf {R} \left\{\mathbf {q} _ {1} \right\} \mathbf {R} \left\{\mathbf {q} _ {2} \right\},\tag{121}
$$

where we observe that: (118) the identity quaternion encodes the null rotation; (119) a quaternion and its negative encode the same rotation, defining a double cover of $SO(3)$ ;

(120) the conjugate quaternion encodes the inverse rotation; and (121) the quaternion product composes consecutive rotations in the same order as rotation matrices do.

Additionally, we have the property

$$
\mathbf {R} \{\mathbf {q} ^ {t} \} = \mathbf {R} \{\mathbf {q} \} ^ {t},\tag{122}
$$

which relates the spherical interpolations of the quaternion and rotation matrix over a running scalar t.

## 2.6 Rotation composition

Quaternion composition is done similarly to rotation matrices, i.e., with appropriate quaternion- and matrix-products, and in the same order (Fig. 5),

$$
\mathbf {q} _ {\mathcal {A C}} = \mathbf {q} _ {\mathcal {A B}} \otimes \mathbf {q} _ {\mathcal {B C}},
$$

$$
\mathbf {R} _ {\mathcal {A C}} = \mathbf {R} _ {\mathcal {A B}} \mathbf {R} _ {\mathcal {B C}}.\tag{123}
$$

This comes immediately from the associative property of the involved products,

$$
\begin{array}{r l r l} \mathbf {x} _ {\mathcal {A}} = \mathbf {q} _ {\mathcal {A B}} \otimes \mathbf {x} _ {\mathcal {B}} \otimes \mathbf {q} _ {\mathcal {A B}} ^ {*} & & \mathbf {x} _ {\mathcal {A}} = \mathbf {R} _ {\mathcal {A B}} \mathbf {x} _ {\mathcal {B}} \\ = \mathbf {q} _ {\mathcal {A B}} \otimes (\mathbf {q} _ {\mathcal {B C}} \otimes \mathbf {x} _ {\mathcal {C}} \otimes \mathbf {q} _ {\mathcal {B C}} ^ {*}) \otimes \mathbf {q} _ {\mathcal {A B}} ^ {*} & & = \mathbf {R} _ {\mathcal {A B}} (\mathbf {R} _ {\mathcal {B C}} \mathbf {x} _ {\mathcal {C}}) \\ = (\mathbf {q} _ {\mathcal {A B}} \otimes \mathbf {q} _ {\mathcal {B C}}) \otimes \mathbf {x} _ {\mathcal {C}} \otimes (\mathbf {q} _ {\mathcal {B C}} ^ {*} \otimes \mathbf {q} _ {\mathcal {A B}} ^ {*}) & & = (\mathbf {R} _ {\mathcal {A B}} \mathbf {R} _ {\mathcal {B C}}) \mathbf {x} _ {\mathcal {C}} \\ = (\mathbf {q} _ {\mathcal {A B}} \otimes \mathbf {q} _ {\mathcal {B C}}) \otimes \mathbf {x} _ {\mathcal {C}} \otimes (\mathbf {q} _ {\mathcal {A B}} \otimes \mathbf {q} _ {\mathcal {B C}}) ^ {*} & & = \mathbf {R} _ {\mathcal {A C}} \mathbf {x} _ {\mathcal {C}}. \\ = \mathbf {q} _ {\mathcal {A C}} \otimes \mathbf {x} _ {\mathcal {C}} \otimes \mathbf {q} _ {\mathcal {A C}} ^ {*}, & & \end{array}
$$

![](images/71a8150f190e4126121b7d921d9636e982976fd27d1c845c375077f0de1ef218.jpg)  
Figure 5: Rotation composition. In $R^{2}$ , we would simply do $\theta_{AC} = \theta_{AB} + \theta_{BC}$ , with an operation ‘sum’ that is commutative. In $R^{3}$ composition satisfies $q_{AC} = q_{AB} \otimes q_{BC}$ and, in matrix form, $R_{AC} = R_{AB} R_{BC}$ . These operators are not commutative and one must respect the order strictly —a proper notation helps: ‘AB’ chains with ‘BC’ to create ‘AC’.

A comment on notation A proper notation helps determining the right order of the factors in the composition, especially for compositions of several rotations (see Fig. 5). For example, let $\mathbf{q}_{ji}$ (resp. $\mathbf{R}_{ji}$ ) represent a rotation from situation $i$ to situation $j$ , that is, $\mathbf{x}_j = \mathbf{q}_{ji} \otimes \mathbf{x}_i \otimes \mathbf{q}_{ji}^*$ (resp. $\mathbf{x}_j = \mathbf{R}_{ji}\mathbf{x}_i$ ). Then, given a number of rotations represented by the quaternions $\mathbf{q}_{OA}, \mathbf{q}_{AB}, \mathbf{q}_{BC}, \mathbf{q}_{OX}, \mathbf{q}_{XZ}$ , we just have to chain the indices and get:

$$
\mathbf {q} _ {O C} = \mathbf {q} _ {O A} \otimes \mathbf {q} _ {A B} \otimes \mathbf {q} _ {B C} \quad \mathbf {R} _ {O C} = \mathbf {R} _ {O A} \mathbf {R} _ {A B} \mathbf {R} _ {B C},
$$

and knowing that the opposite rotation corresponds to the conjugate, $q_{ji} = q_{ij}^{*}$ , or the transpose, $R_{ji} = R_{ij}^{\top}$ , we also have

$$
\begin{array}{r l r} \mathbf {q} _ {Z A} = \mathbf {q} _ {X Z} ^ {*} \otimes \mathbf {q} _ {O X} ^ {*} \otimes \mathbf {q} _ {O A} & & \mathbf {R} _ {Z A} = \mathbf {R} _ {X Z} ^ {\top} \mathbf {R} _ {O X} ^ {\top} \mathbf {R} _ {O A} \\ = \mathbf {q} _ {Z X} \otimes \mathbf {q} _ {X O} \otimes \mathbf {q} _ {O A} & & = \mathbf {R} _ {Z X} \mathbf {R} _ {X O} \mathbf {R} _ {O A}. \end{array}
$$

## 2.7 Spherical linear interpolation (SLERP)

Quaternions are very handy for computing proper orientation interpolations. Given two orientations represented by quaternions $q_{0}$ and $q_{1}$ , we want to find a quaternion function $\mathbf{q}(t)$ , $t \in [0,1]$ , that linearly interpolates from $\mathbf{q}(0) = \mathbf{q}_{0}$ to $\mathbf{q}(1) = \mathbf{q}_{1}$ . This interpolation is such that, as t evolves from 0 to 1, a body will continuously rotate from orientation $q_{0}$ to orientation $q_{1}$ , at constant speed along a fixed axis.

Method 1 A first approach uses quaternion algebra, and follows a geometric reasoning in $R^{3}$ that should be easily related to the material presented so far. First, compute the orientation increment $\Delta q$ from $q_{0}$ to $q_{1}$ such that $q_{1} = q_{0} \otimes \Delta q$ ,

$$
\Delta \mathbf {q} = \mathbf {q} _ {0} ^ {*} \otimes \mathbf {q} _ {1}.\tag{124}
$$

Then obtain the associated rotation vector, $\Delta\phi = u\Delta\phi$ , using the logarithmic map, $^{11}$

$$
\mathbf {u} \Delta \phi = \operatorname{Log} (\Delta \mathbf {q}).\tag{125}
$$

Finally, keep the rotation axis u and take a linear fraction of the rotation angle, $\delta\phi = t\Delta\phi$ . Put it in quaternion form through the exponential map, $\delta\mathbf{q} = \operatorname{Exp}(\mathbf{u}\delta\phi)$ , and compose it with the original quaternion to get the interpolated result,

$$
\mathbf {q} (t) = \mathbf {q} _ {0} \otimes \operatorname{Exp} (t \mathbf {u} \Delta \phi).\tag{126}
$$

The whole process can be written as $\mathbf{q}(t) = \mathbf{q}_0\otimes \mathrm{Exp}(t\operatorname {Log}(\mathbf{q}_0^*\otimes \mathbf{q}_1))$ , which reduces to

$$
\boxed {\mathbf {q} (t) = \mathbf {q} _ {0} \otimes (\mathbf {q} _ {0} ^ {*} \otimes \mathbf {q} _ {1}) ^ {t}}  ,\tag{127}
$$

and which is usually implemented (see (53)) as,

$$
\mathbf {q} (t) = \mathbf {q} _ {0} \otimes \left[ \begin{array}{c} \cos (t   \Delta \phi / 2) \\ \mathbf {u} \sin (t   \Delta \phi / 2) \end{array} \right]  .\tag{128}
$$

![](images/964d5816a3679d632ce5f0fe45bfa1581192b4a703313ac0d3afaafefbd307e9.jpg)  
Figure 6: Quaternion interpolation in the unit sphere of $R^{4}$ , and a frontal view of the situation on the rotation plane $\pi$ of $R^{4}$ .

Note: An analogous procedure may be used to define Slerp for rotation matrices, yielding

$$
\mathbf {R} (t) = \mathbf {R} _ {0} \mathrm{Exp} (t \mathrm{Log} (\mathbf {R} _ {0} ^ {\top} \mathbf {R} _ {1})) = \mathbf {R} _ {0} (\mathbf {R} _ {0} ^ {\top} \mathbf {R} _ {1}) ^ {t},\tag{129}
$$

where the matrix exponential $R^{t}$ can be implemented using Rodrigues (77), leading to

$$
\mathbf {R} (t) = \mathbf {R} _ {0} \left(\mathbf {I} + \sin (t \Delta \phi) [ \mathbf {u} ] _ {\times} + (1 - \cos (t \Delta \phi)) [ \mathbf {u} ] _ {\times} ^ {2}\right).\tag{130}
$$

Method 2 Other approaches to Slerp can be developed that are independent of the inners of quaternion algebra, and even independent of the dimension of the space in which the arc is embedded. In particular, see Fig. 6, we can treat quaternions $q_{0}$ and $q_{1}$ as two unit vectors in the unit sphere, and interpolate in this same space. The interpolated $\mathbf{q}(t)$ is the unit vector that follows at a constant angular speed the shortest spherical path joining $q_{0}$ to $q_{1}$ . This path is the planar arc resulting from intersecting the unit sphere with the plane defined by $q_{0}$ , $q_{1}$ and the origin (dashed circumference in the figure). For a proof that these approaches are equivalent to the above, see Dam et al. (1998).

The first of these approaches uses vector algebra and follows literally the ideas above. Consider $q_{0}$ and $q_{1}$ as two unit vectors; the angle $^{12}$ between them is derived from the scalar product,

$$
\cos (\Delta \theta) = \mathbf {q} _ {0} ^ {\top} \mathbf {q} _ {1}
$$

$$
\Delta \theta = \arccos (\mathbf {q} _ {0} ^ {\top} \mathbf {q} _ {1}).\tag{131}
$$

We proceed as follows. We identify the plane of rotation, that we name here $\pi$ , and build its ortho-normal basis $\{q_{0}, q_{\perp}\}$ , where $q_{\perp}$ comes from ortho-normalizing $q_{1}$ against $q_{0}$ ,

$$
\mathbf {q} _ {\perp} = \frac {\mathbf {q} _ {1} - (\mathbf {q} _ {0} ^ {\top} \mathbf {q} _ {1}) \mathbf {q} _ {0}}{\left\| \mathbf {q} _ {1} - (\mathbf {q} _ {0} ^ {\top} \mathbf {q} _ {1}) \mathbf {q} _ {0} \right\|},\tag{132}
$$

so that (see Fig. 6 - right)

$$
\mathbf {q} _ {1} = \mathbf {q} _ {0} \cos \Delta \theta + \mathbf {q} _ {\perp} \sin \Delta \theta .\tag{133}
$$

Then, we just need to rotate $q_{0}$ a fraction of the angle, $t\Delta\theta$ , over the plane $\pi$ , to yield the spherical interpolation,

$$
\boxed {\mathbf {q} (t) = \mathbf {q} _ {0} \cos (t \Delta \theta) + \mathbf {q} _ {\perp} \sin (t \Delta \theta)}.\tag{134}
$$

Method 3 A similar approach, credited to Glenn Davis in Shoemake (1985), draws from the fact that any point on the great arc joining $\mathbf{q}_0$ to $\mathbf{q}_1$ must be a linear combination of its ends (since the three vectors are coplanar). Having computed the angle $\Delta\theta$ using (131), we can isolate $\mathbf{q}_{\perp}$ from (133) and inject it in (134). Applying the identity $\sin(\Delta\theta - t\Delta\theta) = \sin\Delta\theta\cos t\Delta\theta - \cos\Delta\theta\sin t\Delta\theta$ , we obtain the Davis' formula (see Eberly (2010) for an alternative derivation),

$$
\left| \mathbf {q} (t) = \mathbf {q} _ {0} \frac {\sin ((1 - t) \Delta \theta)}{\sin (\Delta \theta)} + \mathbf {q} _ {1} \frac {\sin (t \Delta \theta)}{\sin (\Delta \theta)} \right|.\tag{135}
$$

This formula has the benefit of being symmetric: defining the reverse interpolator s = 1 - t yields

$$
\mathbf {q} (s) = \mathbf {q} _ {1} \frac {\sin ((1 - s) \Delta \theta)}{\sin (\Delta \theta)} + \mathbf {q} _ {0} \frac {\sin (s \Delta \theta)}{\sin (\Delta \theta)}.
$$

which is exactly the same formula with the roles of $q_{0}$ and $q_{1}$ swapped.

All these quaternion-based SLERP methods require some care to ensure proper interpolation along the shortest path, that is, with rotation angles $\phi \leq \pi$ . Due to the quaternion double cover of $SO(3)$ (see Section 2.4.6) only the interpolation between quaternions in acute angles $\Delta\theta \leq \pi/2$ is done following the shortest path (Fig. 7). Testing for this situation and solving it is simple: if $\cos(\Delta\theta) = \mathbf{q}_{0}^{\top}\mathbf{q}_{1} < 0$ , then replace e.g. $q_{1}$ by $-q_{1}$ and start over.

## 2.8 Quaternion and isoclinic rotations: explaining the magic

This section provides geometrical insights to the two intriguing questions about quaternions, what we call the ‘magic’:

\- How is it that the product $\mathbf{q} \otimes \mathbf{x} \otimes \mathbf{q}^*$ rotates the vector $\mathbf{x}$ ?

\- Why do we need to consider half-angles when constructing the quaternion through $\mathbf{q} = e^{\phi /2} = [\cos \phi /2,\mathbf{u}\sin \phi /2]$ ?

![](images/7e94918e5de1ca8793a4f4fe88f781fa50a95ed1a1e749d344e01369c8ee825b.jpg)  
Figure 7: Ensuring Slerp along the shortest path between the orientations represented by $\mathbf{q}_0$ and $\mathbf{q}_1$ . Left: quaternion rotation plane in 4D space, showing initial and final orientation quaternions, and two possible interpolations, $\mathbf{q}(t)$ from $\mathbf{q}_0$ to $\mathbf{q}_1$ , and $\mathbf{q}'(t)$ from $\mathbf{q}_0$ to $-\mathbf{q}_1$ . Right: vector rotation plane in 3D space: since $\mathbf{q}_1 = -\mathbf{q}_1$ , we have $\mathbf{x}_1 = \mathbf{q}_1 \otimes \mathbf{x}_0 \otimes \mathbf{q}_1^* = \mathbf{q}_1' \otimes \mathbf{x}_0 \otimes \mathbf{q}_1'$ , that is, both quaternions produce the same rotation. However, the interpolated quaternion $\mathbf{q}(t)$ produces the vector $\mathbf{x}(t)$ which takes the long path from $\mathbf{x}_0$ to $\mathbf{x}_1$ , while the corrected $\mathbf{q}_1' = -\mathbf{q}_1$ yields $\mathbf{q}'(t)$ , producing the vector $\mathbf{x}'(t)$ along the shortest path from $\mathbf{x}_0$ to $\mathbf{x}_1$ .

We want a geometrical explanation, that is, some rationale that goes beyond the algebraic demonstration (109) and the double cover facts in Section 2.4.6.

To start, let us reproduce here equation (116) expressing the quaternion rotation action through the quaternion product matrices $[q]_{L}$ and $[q^{*}]_{R}$ , defined in (19),

$$
\mathbf {q} \otimes \mathbf {x} \otimes \mathbf {q} ^ {*} = [ \mathbf {q} ^ {*} ] _ {R} [ \mathbf {q} ] _ {L} \left[ \begin{array}{c} 0 \\ \mathbf {x} \end{array} \right] = \left[ \begin{array}{c} 0 \\ \mathbf {R x} \end{array} \right].
$$

For unit quaternions $\mathbf{q}$ , the quaternion product matrices $[\mathbf{q}]_L$ and $[\mathbf{q}^*]_R$ satisfy two remarkable properties,

$$
[ \mathbf {q} ] [ \mathbf {q} ] ^ {\top} = \mathbf {I} _ {4}\tag{136}
$$

$$
\det ([ \mathbf {q} ]) = + 1,\tag{137}
$$

and are therefore elements of $SO(4)$ , that is, proper rotation matrices in the $R^{4}$ space. To be more specific, they represent a particular type of rotation, named isoclinic rotation, as we explain hereafter. Thus, according to (116), a quaternion rotation corresponds to two chained isoclinic rotations in $R^{4}$ .

In order to explain the insights of quaternion rotation, we need to understand isoclinic rotations in $R^{4}$ . For this, we first need to understand general rotations in $R^{4}$ . And to understand rotations in $R^{4}$ , we need to go back to $R^{3}$ , whose rotations are in fact planar rotations. Let us walk all these steps one by one.

![](images/16fae62e687a3200418c40d2bbb42651905ec5f90248f9aee4127a04c00974d0.jpg)  
Figure 8: Rotation in $R^{3}$ . A rotation of a vector x around an axis u describes a circumference in a plane orthogonal to the axis. The component of x parallel to the axis does not move, and is represented by the small red dot on the axis. The sketch on the right illustrates the radically different behaviors of the rotating point on the plane and axis subspaces.

![](images/9e27dd4d23c7a60b86b67e648c7770475003265c25288c50b9120389516e4e8d.jpg)  
Figure 9: Rotations in $R^{4}$ . Two orthogonal rotations are possible, on two orthogonal planes $\pi_{1}$ and $\pi_{2}$ . Rotations of a vector x (not drawn) in the plane $\pi_{1}$ cause the two components of the vector parallel to this plane (red dot on $\pi_{1}$ ) to describe a circumference (red circle), leaving the other two components in $\pi_{2}$ unchanged (the red dot). Conversely, rotations in the plane $\pi_{2}$ (blue dot on blue circle in $\pi_{2}$ ) leave the components in $\pi_{1}$ unchanged (blue dot). The sketch on the right better illustrates the situation by resigning to draw unrepresentable perspectives in $R^{4}$ , which might be misleading.

Rotations in $R^{3}$ : In $R^{3}$ , let us consider the rotations of a vector x around an arbitrary axis represented by the vector u —see Fig. 8, and recall Fig. 1. Upon rotation, vectors parallel to the axis of rotation u do not move, and vectors perpendicular to the axis rotate in the plane $\pi$ perpendicular to the axis. For general vectors x, the two components of the vector in the plane rotate in this plane, while the axial component remains static.

Rotations in $R^{4}$ : In $R^{4}$ , see Fig. 9, due to the extra dimension, the one-dimensional axis of rotation in $R^{3}$ becomes a new two-dimensional plane. This second plane provides room for a second rotation. Indeed, rotations in $R^{4}$ encompass two independent rotations in two orthogonal planes of the 4-space. This means that every 4-vector of each of these planes rotates in its own plane, and that rotations of general 4-vectors with respect to one plane leave unaffected the vector components in the other plane. These planes are for this

![](images/b054fa3baa6bca165342f0fab38c3cce42471347762af67b42df805dc19c8dff.jpg)  
Figure 10: Quaternion rotation in $R^{4}$ . Two chained isoclinic rotations, one left (with equal half-angles), and one right (with opposing half-angles), produce a pure rotation by the full angle in only one of the invariant planes.

## reason called ‘invariant’.

Isoclinic rotations in $R^{4}$ : Isoclinic rotations (from Greek, iso: “equal”, klinein: “to incline”) are those rotations in $R^{4}$ where the angles of rotation in the two invariant planes have the same magnitude. Then, when the two angles have also the same sign, $^{13}$ we speak of left-isoclinic rotations. And when they have opposite signs, we speak of right-isoclinic rotations. A remarkable property of isoclinic rotations, that we had already seen in (22), is that left- and right-isoclinic rotations commute,

$$
[ \mathbf {p} ] _ {R} [ \mathbf {q} ] _ {L} = [ \mathbf {q} ] _ {L} [ \mathbf {p} ] _ {R}.\tag{138}
$$

Quaternion rotations in $R^{4}$ and $R^{3}$ : Given a unit quaternion $q = e^{u\theta/2}$ , representing a rotation in $R^{3}$ of an angle $\theta$ around the axis u, the matrix $[q]_{L}$ is a left-isoclinic rotation in $R^{4}$ corresponding to the left-multiplication by the quaternion q, and $[q^{*}]_{R}$ is a right-isoclinic rotation corresponding to the right-multiplication by the quaternion $q^{*}$ . The angles of these isoclinic rotations are exactly of magnitude $\theta/2$ , $^{14}$ and the invariant planes are the same. Then, the rotation expression (116), reproduced once again here,

$$
\left[ \begin{array}{c} 0 \\ \mathbf {x} ^ {\prime} \end{array} \right] = \mathbf {q} \otimes \mathbf {x} \otimes \mathbf {q} ^ {*} = [ \mathbf {q} ^ {*} ] _ {R} [ \mathbf {q} ] _ {L} \left[ \begin{array}{c} 0 \\ \mathbf {x} \end{array} \right],
$$

represents two chained isoclinic rotations to the 4-vector $(0,\mathbf{x})^{\top}$ , one left- and one right-, each by half the desired rotation angle in $R^{3}$ . In one of the invariant planes of $R^{4}$ (see Fig. 10), the two half angles cancel out, because they have opposite signs. In the other plane, they sum up to yield the total rotation angle $\theta$ . If we define from (116) the resulting rotation matrix, $R_{4}$ , one easily realizes that (see also (138)),

$$
\mathbf {R} _ {4} \triangleq [ \mathbf {q} ^ {*} ] _ {R} [ \mathbf {q} ] _ {L} = [ \mathbf {q} ] _ {L} [ \mathbf {q} ^ {*} ] _ {R} = \left[ \begin{array}{c c} 1 & \mathbf {0} \\ \mathbf {0} & \mathbf {R} \end{array} \right],\tag{139}
$$

where R is the rotation matrix in $R^{3}$ , which clearly rotates vectors in the $R^{3}$ subspace of $R^{4}$ , leaving the fourth dimension unchanged.

This discourse is somewhat beyond the scope of the present document. It is also incomplete, for it does not provide, beyond the result in (139), an intuition or geometrical explanation for why we need to do $q \otimes x \otimes q^{*}$ instead of e.g. $q \otimes x \otimes q$ . $^{15}$ We include it here just as a means for providing yet another way to interpret rotations by quaternions, with the hope that the reader grasps more intuition about its mechanisms. The interested reader is suggested to consult the appropriate literature on isoclinic rotations in $R^{4}$ .

## 3 Quaternion conventions. My choice.

## 3.1 Quaternion flavors

There are several ways to determine the quaternion. They are basically related to four binary choices:

\- The order of its elements — real part first or last:

$$
\mathbf {q} = \left[ \begin{array}{c} q _ {w} \\ \mathbf {q} _ {v} \end{array} \right] \quad v s. \quad \mathbf {q} = \left[ \begin{array}{c} \mathbf {q} _ {v} \\ q _ {w} \end{array} \right].\tag{140}
$$

\- The multiplication formula — definition of the quaternion algebra:

$$
i j = - j i = k \qquad v s. \qquad j i = - i j = k,\tag{141a}
$$

which correspond to different handedness, respectively:

$$
r i g h t - h a n d e d \quad v s. \quad l e f t - h a n d e d.\tag{141b}
$$

This means that, given a rotation axis u, one quaternion $q_{right}\{u\theta\}$ rotates vectors an angle $\theta$ around u using the right hand rule, while the other quaternion $q_{left}\{u\theta\}$ uses the left hand rule.

\- The function of the rotation operator — rotating frames or rotating vectors:

$$
\text { Passive } \quad \text { vs. } \quad \text { Active. }\tag{142}
$$

Table 2: Hamilton vs. JPL quaternion conventions with respect to the 4 binary choices

<table><tr><td colspan="2">Quaternion type</td><td>Hamilton</td><td>JPL</td></tr><tr><td>1</td><td>Components order</td><td> $(q_w, \mathbf{q}_v)$ </td><td> $(\mathbf{q}_v, q_w)$ </td></tr><tr><td rowspan="2">2</td><td>Algebra</td><td> $ij = k$ </td><td> $ij = -k$ </td></tr><tr><td>Handedness</td><td>Right-handed</td><td>Left-handed</td></tr><tr><td>3</td><td>Function</td><td>Passive</td><td>Passive</td></tr><tr><td rowspan="3">4</td><td>Right-to-left products mean</td><td>Local-to-Global</td><td>Global-to-Local</td></tr><tr><td>Default notation,  $\mathbf{q}$ </td><td> $\mathbf{q} \triangleq \mathbf{q}_{\mathcal{G}\mathcal{L}}$ </td><td> $\mathbf{q} \triangleq \mathbf{q}_{\mathcal{L}\mathcal{G}}$ </td></tr><tr><td>Default operation</td><td> $\mathbf{x}_{\mathcal{G}} = \mathbf{q} \otimes \mathbf{x}_{\mathcal{L}} \otimes \mathbf{q}^*$ </td><td> $\mathbf{x}_{\mathcal{L}} = \mathbf{q} \otimes \mathbf{x}_{\mathcal{G}} \otimes \mathbf{q}^*$ </td></tr></table>

\- In the passive case, the direction of the operation — local-to-global or global-to-local:

$$
\mathbf {x} _ {g l o b a l} = \mathbf {q} \otimes \mathbf {x} _ {l o c a l} \otimes \mathbf {q} ^ {*} \qquad v s. \qquad \mathbf {x} _ {l o c a l} = \mathbf {q} \otimes \mathbf {x} _ {g l o b a l} \otimes \mathbf {q} ^ {*}\tag{143}
$$

This variety of choices leads to 12 different combinations. Historical developments have favored some conventions over others (Chou, 1992; Yazell, 2009). Today, in the available literature, we find many quaternion flavors such as the Hamilton, the STS $^{16}$ , the JPL $^{17}$ , the ISS $^{18}$ , the ESA $^{19}$ , the Engineering, the Robotics, and possibly a lot more denominations. Many of these forms might be identical, others not, but this fact is rarely explicitly stated, and many works simply lack a sufficient description of their quaternion with regard to the four choices above.

These differences impact the respective formulas for rotation, composition, etc., in non-obvious ways. The formulas are thus not compatible, and we need to make a clear choice from the very start.

The two most commonly used conventions, which are also the best documented, are Hamilton (the options on the left in (140–143)) and JPL (the options on the right, with the exception of (142)). Table 2 shows a summary of their characteristics. JPL is mostly used in the aerospace domain, while Hamilton is more common to other engineering areas such as robotics — though this should not be taken as a rule.

My choice, that has been taken as early as in equation (2), is to take the Hamilton convention, which is right-handed and coincides with many software libraries of widespread use in robotics, such as Eigen, ROS, Google Ceres, and with a vast amount of literature on Kalman filtering for attitude estimation using IMUs (Chou, 1992; Kuipers, 1999; Piniés et al., 2007; Roussillon et al., 2011; Martinelli, 2012, and many others).

The JPL convention is possibly less commonly used, at least in the robotics field. It is extensively described in (Trawny and Roumeliotis, 2005), a reference work that has an aim and scope very close to the present one, but that concentrates exclusively in the JPL convention. The JPL quaternion is used in the JPL literature (obviously) and in key papers by Li, Mourikis, Roumeliotis, and colleagues (see e.g. (Li and Mourikis, 2012; Li et al., 2014)), which draw from Trawny and Roumeliotis' document. These works are a primary source of inspiration when dealing with visual-inertial odometry and SLAM —which is what we do.

In the rest of this section we analyze these two quaternion conventions with a little more depth.

## 3.1.1 Order of the quaternion components

Though not the most fundamental, the most salient difference between Hamilton and JPL quaternions is in the order of the components, with the scalar part being either in first (Hamilton) or last (JPL) position. The implications of such change are quite obvious and should not represent a great challenge of interpretation. In fact, some works with the quaternion's real component at the end (e.g., the C++ library Eigen) are still considered as using the Hamilton convention, as long as the other three aspects are maintained.

We have used the subscripts $(w, x, y, z)$ for the quaternion components for increased clarity, instead of the other commonly used $(0, 1, 2, 3)$ . When changing the order, $q_{w}$ will always denote the real part, while it is not clear whether $q_{0}$ would also do —in some occasions, one might find things such as $\mathbf{q} = (q_{1}, q_{2}, q_{3}, q_{0})$ , with $q_{0}$ real and last, but in the general case of $\mathbf{q} = (q_{0}, q_{1}, q_{2}, q_{3})$ , the real part at the end would be $q_{3}$ . $^{20}$ When passing from one convention to the other, we must be careful of formulas involving full $4 \times 4$ or $3 \times 4$ quaternion-related matrices, for their rows and/or columns need to be swapped. This is not difficult to do, but it might be difficult to detect and therefore prone to error.

Two curiosities about the components' order are:

\- With real part first, the quaternion is naturally interpreted as an extended complex number, of the familiar form $real + imaginary$ . Some of us are comfortable with this representation probably because of this.

\- With real part last, the quaternion expressed in vector form, $\mathbf{q} = [x,y,z,w] \in \mathbb{H}$ , has a format absolutely equivalent to the homogeneous vector in the projective 3D space, $\mathbf{p} = [x,y,z,w] \in \mathbb{P}^3$ , where in both cases $x,y,z$ are clearly identified with the three Cartesian axes. When dealing with geometric problems in 3D, this makes the algebra for operating on quaternions and homogeneous vectors more uniform, especially (but not only) if the homogeneous vector is constrained to the unit sphere $\| \mathbf{p} \| = 1$ .

## 3.1.2 Specification of the quaternion algebra

The Hamilton convention defines ij = k and therefore,

$$
i ^ {2} = j ^ {2} = k ^ {2} = i j k = - 1, \quad i j = - j i = k, \quad j k = - k j = i, \quad k i = - i k = j,\tag{144}
$$

whereas the JPL convention defines ji = k and hence its quaternion algebra becomes,

$$
i ^ {2} = j ^ {2} = k ^ {2} = - i j k = - 1, \quad - i j = j i = k, \quad - j k = k j = i, \quad - k i = i k = j.\tag{145}
$$

Interestingly, these subtle sign changes preserve the basic properties of quaternions as rotation operators. Mathematically, the key consequence is the change of the sign of the cross-product in (13), which induces a change in the quaternion handedness (Shuster, 1993): Hamilton uses $ij = k$ and is therefore right-handed, i.e., it turns vectors following the right-hand rule; JPL uses $ji = k$ and is left-handed (Trawny and Roumeliotis, 2005). Being left- and right- handed rotations of opposite signs, we can say that their quaternions $\mathbf{q}_{left}$ and $\mathbf{q}_{right}$ are related by,

$$
\mathbf {q} _ {l e f t} = \mathbf {q} _ {r i g h t} ^ {*}.\tag{146}
$$

## 3.1.3 Function of the rotation operator

We have seen how to rotate vectors in 3D. This is referred to in (Shuster, 1993) as the active interpretation, because operators (this affects all rotation operators) actively rotate vectors,

$$
\mathbf {x} ^ {\prime} = \mathbf {q} _ {a c t i v e} \otimes \mathbf {x} \otimes \mathbf {q} _ {a c t i v e} ^ {*},
$$

$$
\mathbf {x} ^ {\prime} = \mathbf {R} _ {a c t i v e} \mathbf {x}.\tag{147}
$$

Another way of seeing the effect of q and R over a vector x is to consider that the vector is steady but it is us who have rotated our point of view by an amount specified by q or R. This is called here frame transformation and it is referred to in (Shuster, 1993) as the passive interpretation, because vectors do not move,

$$
\mathbf {x} _ {\mathcal {B}} = \mathbf {q} _ {p a s s i v e} \otimes \mathbf {x} _ {\mathcal {A}} \otimes \mathbf {q} _ {p a s s i v e} ^ {*},
$$

$$
\mathbf {x} _ {\mathcal {B}} = \mathbf {R} _ {p a s s i v e} \mathbf {x} _ {\mathcal {A}},\tag{148}
$$

where A and B are two Cartesian reference frames, and $x_{A}$ and $x_{B}$ are expressions of the same vector x in these frames. See further down for explanations and proper notations.

The active and passive interpretations are governed by operators inverse of each other, that is,

$$
\mathbf {q} _ {a c t i v e} = \mathbf {q} _ {p a s s i v e} ^ {*},
$$

$$
\mathbf {R} _ {a c t i v e} = \mathbf {R} _ {p a s s i v e} ^ {\top}.
$$

Both Hamilton and JPL use the passive convention.

Direction cosine matrix A few authors understand the passive operator as not being a rotation operator, but rather an orientation specification, named the direction cosine matrix,

$$
\mathbf {C} = \left[ \begin{array}{c c c} c _ {x x} & c _ {x y} & c _ {z x} \\ c _ {x y} & c _ {y y} & c _ {z y} \\ c _ {x z} & c _ {y z} & c _ {z z} \end{array} \right],\tag{149}
$$

where each component $c_{ij}$ is the cosine of the angle between the axis i in the source frame and the axis j in the target frame. We have the identity,

$$
\mathbf {C} \equiv \mathbf {R} _ {\text {passive}}.\tag{150}
$$

## 3.1.4 Direction of the rotation operator

In the passive case, a second source of interpretation is related to the direction in which the rotation matrix and quaternion operate, either converting from local to global frames, or from global to local.

Given two Cartesian frames G and L, we identify G and L as being the global and local frames. “Global” and “local” are relative definitions, i.e., G is global with respect to L, and L is local with respect to G – in other words, L is a frame specified in the reference frame $G.^{21}$ . We specify $q_{GL}$ and $R_{GL}$ as being respectively the quaternion and rotation matrix transforming vectors from frame L to frame G, in the sense that a vector $x_{L}$ in frame L is expressed in frame G with the quaternion- and matrix-products

$$
\mathbf {x} _ {\mathcal {G}} = \mathbf {q} _ {\mathcal {G L}} \otimes \mathbf {x} _ {\mathcal {L}} \otimes \mathbf {q} _ {\mathcal {G L}} ^ {*},
$$

$$
\mathbf {x} _ {\mathcal {G}} = \mathbf {R} _ {\mathcal {G L}} \mathbf {x} _ {\mathcal {L}}.\tag{151}
$$

The opposite conversion, from G to L, is done with

$$
\mathbf {x} _ {\mathcal {L}} = \mathbf {q} _ {\mathcal {L G}} \otimes \mathbf {x} _ {\mathcal {G}} \otimes \mathbf {q} _ {\mathcal {L G}} ^ {*},
$$

$$
\mathbf {x} _ {\mathcal {L}} = \mathbf {R} _ {\mathcal {L G}} \mathbf {x} _ {\mathcal {G}},\tag{152}
$$

where

$$
\mathbf {q} _ {\mathcal {L G}} = \mathbf {q} _ {\mathcal {G L}} ^ {*},
$$

$$
\mathbf {R} _ {\mathcal {L G}} = \mathbf {R} _ {\mathcal {G L}} ^ {\top}.\tag{153}
$$

Hamilton uses local-to-global as the default specification of a frame L expressed in frame G,

$$
\mathbf {q} _ {H a m i l t o n} \triangleq \mathbf {q} _ {[ w i t h r e s p e c t t o ] [ o f ]} = \mathbf {q} _ {[ t o ] [ f r o m ]} = \mathbf {q} _ {\mathcal {G L}},\tag{154}
$$

while JPL uses the opposite, global-to-local conversion,

$$
\mathbf {q} _ {J P L} \triangleq \mathbf {q} _ {[ o f ] [ w i t h r e s p e c t t o ]} = \mathbf {q} _ {[ t o ] [ f r o m ]} = \mathbf {q} _ {\mathcal {L G}}.\tag{155}
$$

![](images/b878215f2a4576ef4da2cbf68590ab887e238130b86ea95f67dd6afd0c83c8cc.jpg)  
Figure 11: The S3 manifold is a unit sphere in $\mathbb{R}^4$ , here represented by a unit circle (blue), where all unit quaternions live. The tangent space to the manifold is the hyperplane $\mathbb{R}^3$ , here represented by a line (red). Left: The Exp() and Log() operators map elements of $\mathbb{R}^3$ to/from elements of S3. Right: The $\oplus$ and $\ominus$ operators relate elements of the manifold with elements in the tangent space. (Likewise, these figures illustrate the $SO(3)$ manifold.)

Notice that

$$
\mathbf {q} _ {J P L} \triangleq \mathbf {q} _ {\mathcal {L G}, l e f t} = \mathbf {q} _ {\mathcal {L G}, r i g h t} ^ {*} = \mathbf {q} _ {\mathcal {G L}, r i g h t} \triangleq \mathbf {q} _ {H a m i l t o n},\tag{156}
$$

which is not particularly useful, but illustrates how easy it is to get confused when mixing conventions. Notice also that we can conclude that $q_{JPL} = q_{Hamilton}$ , but this, far from being a beautiful result, is just the source of great confusion, because the equality is only present in the quaternion values, but the two quaternions, when employed in formulas, mean and represent different things.

## 4 Perturbations, derivatives and integrals

## 4.1 The additive and subtractive operators in $SO(3)$

In vector spaces $R^{n}$ , the addition and subtraction operations are performed with the regular sum ‘+’ and minus ‘−’ operations. In $SO(3)$ this is not possible, but equivalent operators can be defined for establishing a proper calculus corpus.

We thus define the plus and minus operators, $\oplus,\ominus$ , between elements $\mathsf{R}\in SO(3)$ , and elements $\theta\in R^{3}$ of the tangent space at R, as follows.

The plus operator. The ‘plus’ operator $\oplus: SO(3) \times \mathbb{R}^{3} \to SO(3)$ produces an element S of $SO(3)$ which is the result of composing a reference element R of $SO(3)$ with a (often small) rotation. This rotation is specified by a vector of $\theta \in R^{3}$ in the vector space tangent to the $SO(3)$ manifold at the reference element R, that is,

$$
\mathsf {S} = \mathsf {R} \oplus \boldsymbol {\theta} \triangleq \mathsf {R} \circ \operatorname{Exp} (\boldsymbol {\theta})
$$

$$
\mathsf {R}, \mathsf {S} \in S O (3), \boldsymbol {\theta} \in \mathbb {R} ^ {3}.\tag{157}
$$

Notice that this operator may be defined for any representation of $SO(3)$ . In particular, for the quaternion and rotation matrix we have,

$$
\mathbf {q} _ {\mathrm{S}} = \mathbf {q} _ {\mathrm{R}} \oplus \boldsymbol {\theta} = \mathbf {q} _ {\mathrm{R}} \otimes \operatorname{Exp} (\boldsymbol {\theta})\tag{158}
$$

$$
\mathbf {R} _ {S} = \mathbf {R} _ {R} \oplus \boldsymbol {\theta} = \mathbf {R} _ {R} \cdot \mathrm{Exp} (\boldsymbol {\theta}) .\tag{159}
$$

The minus operator. The ‘minus’ operator $\ominus: SO(3) \times SO(3) \to \mathbb{R}^{3}$ is the inverse of the above. It returns the vectorial angular difference $\theta \in R^{3}$ between two elements of $SO(3)$ . This difference is expressed in the vector space tangent to the reference element R,

$$
\boldsymbol {\theta} = \mathsf {S} \ominus \mathsf {R} \triangleq \operatorname{Log} (\mathsf {R} ^ {- 1} \circ \mathsf {S}) \quad \mathsf {R}, \mathsf {S} \in S O (3), \boldsymbol {\theta} \in \mathbb {R} ^ {3},\tag{160}
$$

which for the quaternion and rotation matrix reads,

$$
\boldsymbol {\theta} = \mathbf {q} _ {S} \ominus \mathbf {q} _ {R} = \operatorname{Log} \left(\mathbf {q} _ {R} ^ {*} \otimes \mathbf {q} _ {S}\right)\tag{161}
$$

$$
\boldsymbol {\theta} = \mathbf {R} _ {S} \ominus \mathbf {R} _ {R} = \operatorname{Log} \left(\mathbf {R} _ {R} ^ {\top} \mathbf {R} _ {S}\right).\tag{162}
$$

In both cases, notice that even though the vector difference $\theta$ is typically supposed to be small, the definitions above hold for any value of $\theta$ (up to the first coverage of the $SO(3)$ manifold, that is, for angles $\theta < \pi$ ).

## 4.2 The four possible derivative definitions

## 4.2.1 Functions from vector space to vector space

The scalar and vector cases follow the classical definition of the derivative: given a function $f: \mathbb{R}^m \to \mathbb{R}^n$ , we use $\{+, -\}$ to define the derivative as

$$
\frac {\partial f (\mathbf {x})}{\partial \mathbf {x}} \triangleq \lim _ {\delta \mathbf {x} \to 0} \frac {f (\mathbf {x} + \delta \mathbf {x}) - f (\mathbf {x})}{\delta \mathbf {x}}\tag{163}
$$

Euler integration produces linear expressions of the form

$$
f (\mathbf {x} + \Delta \mathbf {x}) \approx f (\mathbf {x}) + \frac {\partial f (\mathbf {x})}{\partial \mathbf {x}} \Delta \mathbf {x}
$$

## 4.2.2 Functions from $SO(3)$ to $SO(3)$

Given a function $f: SO(3) \to SO(3)$ with $\mathsf{R} \in SO(3)$ and a local, small angular variation $\boldsymbol{\theta} \in \mathbb{R}^3$ , we use $\{\oplus, \ominus\}$ to define the derivative as

$$
\begin{array}{l} \frac {\partial f (\mathsf {R})}{\partial \boldsymbol {\theta}} \triangleq \lim _ {\delta \boldsymbol {\theta} \to 0} \frac {f (\mathsf {R} \oplus \delta \boldsymbol {\theta}) \ominus f (\mathsf {R})}{\delta \boldsymbol {\theta}} \\ = \lim _ {\delta \boldsymbol {\theta} \to 0} \frac {\operatorname{Log} \left(f ^ {- 1} (\mathsf {R})   f (\mathsf {R}   \operatorname{Exp} (\delta \boldsymbol {\theta}))\right)}{\delta \boldsymbol {\theta}} \end{array}
$$

$$
\in \mathbb {R} ^ {3 \times 3}\tag{164}
$$

(165)

Euler integration produces expressions of the form,

$$
f (\mathsf {R} \oplus \Delta \boldsymbol {\theta}) \approx f (\mathsf {R}) \oplus \frac {\partial f (\mathsf {R})}{\partial \boldsymbol {\theta}} \Delta \boldsymbol {\theta} \triangleq f (\mathsf {R}) \operatorname{Exp} \left(\frac {\partial f (\mathsf {R})}{\partial \boldsymbol {\theta}} \Delta \boldsymbol {\theta}\right) \quad \in S O (3)
$$

## 4.2.3 Functions from vector space to $SO(3)$

For the case of a function $f : R^{m} \to SO(3)$ , we use ‘+’ for the vector perturbations, and ‘ $\ominus$ ’ for the $SO(3)$ difference,

$$
\begin{array}{r l} & {\frac {\partial f (\mathbf {x})}{\partial \mathbf {x}} \triangleq \lim _ {\delta \mathbf {x} \to 0} \frac {f (\mathbf {x} + \delta \mathbf {x}) \ominus f (\mathbf {x})}{\delta \mathbf {x}}} \\ & {\quad = \lim _ {\delta \mathbf {x} \to 0} \frac {\mathrm{Log} (f ^ {- 1} (\mathbf {x}) f (\mathbf {x} + \delta \mathbf {x}))}{\delta \mathbf {x}}} \end{array}
$$

$$
\in \mathbb {R} ^ {3 \times m}\tag{166}
$$

(167)

Euler integration produces expressions of the form,

$$
f (\mathbf {x} + \Delta \mathbf {x}) \approx f (\mathbf {x}) \oplus \frac {\partial f (\mathbf {x})}{\partial \mathbf {x}} \Delta \mathbf {x} \triangleq f (\mathbf {x}) \mathrm{Exp} \left(\frac {\partial f (\mathbf {x})}{\partial \mathbf {x}} \Delta \mathbf {x}\right) \qquad \in S O (3)
$$

## 4.2.4 Functions from $SO(3)$ to vector space

For the case of a function $f : SO(3) \to \mathbb{R}^{n}$ , we use ‘ $\oplus$ ’ for the $SO(3)$ perturbations, and ‘—’ for the vector difference,

$$
\begin{array}{c} \frac {\partial f (\mathsf {R})}{\partial \boldsymbol {\theta}} \triangleq \lim _ {\delta \boldsymbol {\theta} \to 0} \frac {f (\mathsf {R} \oplus \delta \boldsymbol {\theta}) - f (\mathsf {R})}{\delta \boldsymbol {\theta}} \\ = \lim _ {\delta \boldsymbol {\theta} \to 0} \frac {f (\mathsf {R}   \mathrm{Exp} (\delta \boldsymbol {\theta})) - f (\mathsf {R})}{\delta \boldsymbol {\theta}} \end{array}
$$

$$
\in \mathbb {R} ^ {n \times 3}\tag{168}
$$

(169)

Euler integration produces expressions of the form,

$$
f (\mathsf {R} \oplus \delta \boldsymbol {\theta}) \approx f (\mathsf {R}) + \frac {\partial f (\mathsf {R})}{\partial \boldsymbol {\theta}}   \Delta \boldsymbol {\theta} \triangleq f (\mathsf {R}) + \operatorname{Exp} \left(\frac {\partial f (\mathsf {R})}{\partial \boldsymbol {\theta}} \Delta \boldsymbol {\theta}\right) \quad \in S O (3)
$$

## 4.3 Useful, and very useful, Jacobians of the rotation

Let us consider a rotation to a vector $\mathbf{a}$ , of $\theta$ radians around the unit axis $\mathbf{u}$ . Let us express the rotation specification in three equivalent forms, namely $\boldsymbol{\theta} = \theta \mathbf{u}$ , $\mathbf{q} = \mathbf{q}\{\boldsymbol{\theta}\}$ and $\mathbf{R} = \mathbf{R}\{\boldsymbol{\theta}\}$ . We are interested in the Jacobians of the rotated result with respect to different magnitudes.

## 4.3.1 Jacobian with respect to the vector

The derivative of the rotation of a vector $\mathbf{a}$ with respect to this vector is trivial,

$$
\boxed {\frac {\partial (\mathbf {q} \otimes \mathbf {a} \otimes \mathbf {q} *)}{\partial \mathbf {a}} = \frac {\partial (\mathbf {R a})}{\partial \mathbf {a}} = \mathbf {R}}.\tag{170}
$$

## 4.3.2 Jacobian with respect to the quaternion

On the contrary, the derivative of the rotation with respect to the quaternion q is tricky. For convenience, we use a lighter notation for the quaternion, $q = [w \ v] = w + v$ . We make use of (34), (33), and the identity $\mathbf{a} \times (\mathbf{b} \times \mathbf{c}) = (\mathbf{c} \times \mathbf{b}) \times \mathbf{a} = (\mathbf{a}^{\top} \mathbf{c}) \mathbf{b} - (\mathbf{a}^{\top} \mathbf{b}) \mathbf{c}$ , to develop the quaternion-based rotation (107) as follows,

$$
\begin{array}{l} \mathbf {a} ^ {\prime} = \mathbf {q} \otimes \mathbf {a} \otimes \mathbf {q} * \\ \quad = (w + \mathbf {v}) \otimes \mathbf {a} \otimes (w - \mathbf {v}) \\ \quad = w ^ {2} \mathbf {a} + w (\mathbf {v} \otimes \mathbf {a} - \mathbf {a} \otimes \mathbf {v}) - \mathbf {v} \otimes \mathbf {a} \otimes \mathbf {v} \\ \quad = w ^ {2} \mathbf {a} + 2 w (\mathbf {v} \times \mathbf {a}) - [ (- \mathbf {v} ^ {\top} \mathbf {a} + \mathbf {v} \times \mathbf {a}) \otimes \mathbf {v} ] \\ \quad = w ^ {2} \mathbf {a} + 2 w (\mathbf {v} \times \mathbf {a}) - [ (- \mathbf {v} ^ {\top} \mathbf {a}) \mathbf {v} + (\mathbf {v} \times \mathbf {a}) \otimes \mathbf {v} ] \\ \quad = w ^ {2} \mathbf {a} + 2 w (\mathbf {v} \times \mathbf {a}) - [ (- \mathbf {v} ^ {\top} \mathbf {a}) \mathbf {v} - (\mathbf {v} \times \mathbf {a}) ^ {\top} \overline {{\mathbf {v}}} + (\mathbf {v} \times \mathbf {a}) \times \mathbf {v} ] \\ \quad = w ^ {2} \mathbf {a} + 2 w (\mathbf {v} \times \mathbf {a}) - [ (- \mathbf {v} ^ {\top} \mathbf {a}) \mathbf {v} + (\mathbf {v} ^ {\top} \mathbf {v}) \mathbf {a} - (\mathbf {v} ^ {\top} \mathbf {a}) \mathbf {v} ] \\ \quad = w ^ {2} \mathbf {a} + 2 w (\mathbf {v} \times \mathbf {a}) + 2 (\mathbf {v} ^ {\top} \mathbf {a})   \mathbf {v} - (\mathbf {v} ^ {\top} \mathbf {v})   \mathbf {a}. \end{array}\tag{171}
$$

With this, we can extract the derivatives $\partial a^{\prime}/\partial w$ and $\partial a^{\prime}/\partial v$ ,

$$
\begin{array}{l} \frac {\partial \mathbf {a} ^ {\prime}}{\partial w} = 2 (w \mathbf {a} + \mathbf {v} \times \mathbf {a}) \\ \frac {\partial \mathbf {a} ^ {\prime}}{\partial \mathbf {v}} = - 2 w [ \mathbf {a} ] _ {\times} + 2 (\mathbf {v} ^ {\top} \mathbf {a I} + \mathbf {v a} ^ {\top}) - 2 \mathbf {a v} ^ {\top} \\ = 2 (\mathbf {v} ^ {\top} \mathbf {a I} + \mathbf {v a} ^ {\top} - \mathbf {a v} ^ {\top} - w [ \mathbf {a} ] _ {\times}), \end{array}\tag{172}
$$

(173)

yielding

$$
\left| \frac {\partial (\mathbf {q} \otimes \mathbf {a} \otimes \mathbf {q} *)}{\partial \mathbf {q}} = 2 \left[ w \mathbf {a} + \mathbf {v} \times \mathbf {a} \mid \mathbf {v} ^ {\top} \mathbf {a} \mathbf {I} _ {3} + \mathbf {v} \mathbf {a} ^ {\top} - \mathbf {a} \mathbf {v} ^ {\top} - w [ \mathbf {a} ] _ {\times} \right] \in \mathbb {R} ^ {3 \times 4} \right|\tag{174}
$$

## 4.3.3 Right Jacobian of $SO(3)$

Let us consider (see Fig. 12) an element $R \in SO(3)$ and a rotation vector $\theta \in R^{3}$ such that $R = \text{Exp}(\theta)$ . When $\theta$ is altered by an amount $\delta\theta$ , the element R varies. Expressing the variations of R in the tangent space of $SO(3)$ at R with a rotation vector $\delta\phi \in R^{3}$ , we have that (please see the figure, I am not inventing anything here)

$$
\operatorname{Exp} (\boldsymbol {\theta}) \oplus \delta \boldsymbol {\phi} = \operatorname{Exp} (\boldsymbol {\theta} + \delta \boldsymbol {\theta})\tag{175}
$$

which might be written also as,

$$
\mathrm{Exp} (\pmb {\theta}) \circ \mathrm{Exp} (\delta \pmb {\phi}) = \mathrm{Exp} (\pmb {\theta} + \delta \pmb {\theta}),\tag{176}
$$

and even

$$
\delta \boldsymbol {\phi} = \mathrm{Log} \left(\mathrm{Exp} (\boldsymbol {\theta}) ^ {- 1} \circ \mathrm{Exp} (\boldsymbol {\theta} + \delta \boldsymbol {\theta})\right) = \mathrm{Exp} (\boldsymbol {\theta} + \delta \boldsymbol {\theta}) \ominus \mathrm{Exp} (\boldsymbol {\theta}).\tag{177}
$$

![](images/4130dedaab56f0788fd18288698a0b3f66ec8fd25f9122a54bc4541ffca253f8.jpg)  
Figure 12: The right Jacobian $J_{r} = \partial\delta\phi/\partial\delta\theta$ maps variations $\delta\theta$ around the parameter $\theta$ into variations $\delta\phi$ on the vector space tangent to the manifold at the point Exp $\theta$ .

In the limit, the variation of $\delta\phi$ as a function of $\delta\theta$ defines a Jacobian matrix

$$
\frac {\partial \delta \phi}{\partial \delta \boldsymbol {\theta}} = \lim _ {\delta \boldsymbol {\theta} \rightarrow 0} \frac {\delta \phi}{\delta \boldsymbol {\theta}} = \lim _ {\delta \boldsymbol {\theta} \rightarrow 0} \frac {\operatorname{Exp} (\boldsymbol {\theta} + \delta \boldsymbol {\theta}) \ominus \operatorname{Exp} (\boldsymbol {\theta})}{\delta \boldsymbol {\theta}},\tag{178}
$$

whose expression is a particular case of (166), that is, it is the derivative of the function $f(\boldsymbol{\theta}) = \operatorname{Exp}(\boldsymbol{\theta})$ , from $R^{3}$ to $SO(3)$ . This Jacobian matrix is known as the right Jacobian of $SO(3)$ , and is defined as,

$$
\mathbf {J} _ {r} (\pmb {\theta}) \triangleq \frac {\partial \operatorname{Exp} (\pmb {\theta})}{\partial \pmb {\theta}}.\tag{179}
$$

Its expression is independent of the parametrization used, though it can indeed be expressed particularly for each parametrization. Using (166) we have,

$$
\mathbf {J} _ {r} (\boldsymbol {\theta}) = \lim _ {\delta \boldsymbol {\theta} \rightarrow 0} \frac {\operatorname{Exp} (\boldsymbol {\theta} + \delta \boldsymbol {\theta}) \ominus \operatorname{Exp} (\boldsymbol {\theta})}{\delta \boldsymbol {\theta}}\tag{180}
$$

$$
= \lim _ {\delta \boldsymbol {\theta} \rightarrow 0} \frac {\operatorname{Log} (\operatorname{Exp} (\boldsymbol {\theta}) ^ {\top} \operatorname{Exp} (\boldsymbol {\theta} + \delta \boldsymbol {\theta}))}{\delta \boldsymbol {\theta}}
$$

$$
\mathrm{ifusing} \mathbf {R}\tag{181}
$$

$$
= \lim _ {\delta \boldsymbol {\theta} \rightarrow 0} \frac {\operatorname{Log} (\operatorname{Exp} (\boldsymbol {\theta}) ^ {*} \otimes \operatorname{Exp} (\boldsymbol {\theta} + \delta \boldsymbol {\theta}))}{\delta \boldsymbol {\theta}}
$$

$$
\text { if   using } \mathbf {q} .\tag{182}
$$

The right Jacobian and its inverse can be computed in closed form (Chirikjian, 2012, page 40),

$$
\mathbf {J} _ {r} (\boldsymbol {\theta}) = \mathbf {I} - \frac {1 - \cos \| \boldsymbol {\theta} \|}{\| \boldsymbol {\theta} \| ^ {2}} [ \boldsymbol {\theta} ] _ {\times} + \frac {\| \boldsymbol {\theta} \| - \sin \| \boldsymbol {\theta} \|}{\| \boldsymbol {\theta} \| ^ {3}} [ \boldsymbol {\theta} ] _ {\times} ^ {2}\tag{183}
$$

$$
\mathbf {J} _ {r} ^ {- 1} (\pmb {\theta}) = \mathbf {I} + \frac {1}{2} [ \pmb {\theta} ] _ {\times} + \left(\frac {1}{\| \pmb {\theta} \| ^ {2}} - \frac {1 + \cos \| \pmb {\theta} \|}{2 \| \pmb {\theta} \| \sin \| \pmb {\theta} \|}\right) [ \pmb {\theta} ] _ {\times} ^ {2}\tag{184}
$$

The right Jacobian of $SO(3)$ has the following properties, for any $\theta$ and small $\delta\theta$ ,

$$
\operatorname{Exp} (\boldsymbol {\theta} + \delta \boldsymbol {\theta}) \approx \operatorname{Exp} (\boldsymbol {\theta}) \operatorname{Exp} (\mathbf {J} _ {r} (\boldsymbol {\theta}) \delta \boldsymbol {\theta})\tag{185}
$$

$$
\mathrm{Exp} (\pmb {\theta}) \mathrm{Exp} (\delta \pmb {\theta}) \approx \mathrm{Exp} (\pmb {\theta} + \mathbf {J} _ {r} ^ {- 1} (\pmb {\theta}) \delta \pmb {\theta})\tag{186}
$$

$$
\mathrm{Log} (\mathrm{Exp} (\pmb {\theta}) \mathrm{Exp} (\delta \pmb {\theta})) \approx \pmb {\theta} + \mathbf {J} _ {r} ^ {- 1} (\pmb {\theta}) \delta \pmb {\theta}\tag{187}
$$

## 4.3.4 Jacobian with respect to the rotation vector

The rotation of a vector $a' = R\{\theta\}$ a with respect to the rotation vector $\theta$ is a function from $R^{3}$ to $R^{3}$ . Its derivative with respect to the rotation vector $\theta$ uses (163) and is developed from the previous result, using (185),

$$
\begin{array}{r l}\frac {\partial (\mathbf {q} \otimes \mathbf {a} \otimes \mathbf {q} ^ {*})}{\partial \delta \boldsymbol {\theta}}&= \frac {\partial (\mathbf {R a})}{\partial \delta \boldsymbol {\theta}} = \lim _ {\delta \boldsymbol {\theta} \rightarrow 0} \frac {\mathbf {R} \{\boldsymbol {\theta} + \delta \boldsymbol {\theta} \} \mathbf {a} - \mathbf {R} \{\boldsymbol {\theta} \} \mathbf {a}}{\delta \boldsymbol {\theta}}\\&= \lim _ {\delta \boldsymbol {\theta} \rightarrow 0} \frac {(\mathbf {R} \{\boldsymbol {\theta} \} \operatorname{Exp} (\mathbf {J} _ {r} (\boldsymbol {\theta}) \delta \boldsymbol {\theta}) - \mathbf {R} \{\boldsymbol {\theta} \}) \mathbf {a}}{\delta \boldsymbol {\theta}}\\&= \lim _ {\delta \boldsymbol {\theta} \rightarrow 0} \frac {(\mathbf {R} \{\boldsymbol {\theta} \} (\mathbf {I} + [ \mathbf {J} _ {r} (\boldsymbol {\theta}) \delta \boldsymbol {\theta} ] _ {\times}) - \mathbf {R} \{\boldsymbol {\theta} \}) \mathbf {a}}{\delta \boldsymbol {\theta}}\\&= \lim _ {\delta \boldsymbol {\theta} \rightarrow 0} \frac {\mathbf {R} \{\boldsymbol {\theta} \} [ \mathbf {J} _ {r} (\boldsymbol {\theta}) \delta \boldsymbol {\theta} ] _ {\times} \mathbf {a}}{\delta \boldsymbol {\theta}}\\&= \lim _ {\delta \boldsymbol {\theta} \rightarrow 0} - \frac {\mathbf {R} \{\boldsymbol {\theta} \} [ \mathbf {a} ] _ {\times} \mathbf {J} _ {r} (\boldsymbol {\theta}) \delta \boldsymbol {\theta}}{\delta \boldsymbol {\theta}}\\&= - \mathbf {R} \{\boldsymbol {\theta} \} [ \mathbf {a} ] _ {\times} \mathbf {J} _ {r} (\boldsymbol {\theta}),\end{array}
$$

where $\mathbf{R}\{\pmb {\theta}\} \triangleq \mathrm{Exp}(\pmb {\theta})$ . Summarizing,

$$
\left| \frac {\partial (\mathbf {q} \otimes \mathbf {a} \otimes \mathbf {q} ^ {*})}{\partial \delta \boldsymbol {\theta}} = \frac {\partial (\mathbf {R a})}{\partial \delta \boldsymbol {\theta}} = - \mathbf {R} \{\boldsymbol {\theta} \} [ \mathbf {a} ] _ {\times} \mathbf {J} _ {r} (\boldsymbol {\theta}) \right|.\tag{188}
$$

## 4.3.5 Jacobians of the rotation composition

Consider the SO(3) composition $P = Q \circ R$ , which can be implemented in either quaternion or matrix form,

$$
\mathbf {p} = \mathbf {q} _ {\theta} \otimes \mathbf {r} _ {\phi}
$$

$$
\mathbf {P} = \mathbf {Q} _ {\theta} \mathbf {R} _ {\phi}\tag{189}
$$

where the subindices indicate the name of the vector perturbations in the tangent space. These are functions from $SO(3)$ to $SO(3)$ , and therefore we use (165) to write the derivatives,

$$
\begin{array}{l} \frac {\partial \mathbf {Q} \circ \mathbf {R}}{\partial \mathbf {Q}} = \frac {\partial \mathbf {q} _ {\theta} \otimes \mathbf {r} _ {\phi}}{\partial \boldsymbol {\theta}} = \frac {\partial \mathbf {Q} _ {\theta} \mathbf {R} _ {\phi}}{\partial \boldsymbol {\theta}} = \lim _ {\delta \boldsymbol {\theta} \to 0} \frac {((\mathbf {Q} _ {\theta} \oplus \delta \boldsymbol {\theta}) \mathbf {R} _ {\phi}) \ominus (\mathbf {Q} _ {\theta} \mathbf {R} _ {\phi})}{\delta \boldsymbol {\theta}} \\ \qquad = \lim _ {\delta \boldsymbol {\theta} \to 0} \frac {\mathrm{Log} [ (\mathbf {Q} _ {\theta} \mathbf {R} _ {\phi}) ^ {\top} (\mathbf {Q} _ {\theta}   \mathrm{Exp} (\delta \boldsymbol {\theta}) \mathbf {R} _ {\phi}) ]}{\delta \boldsymbol {\theta}} \\ \qquad = \lim _ {\delta \boldsymbol {\theta} \to 0} \frac {\mathrm{Log} [ \mathbf {R} _ {\phi} ^ {\top}   \mathrm{Exp} (\delta \boldsymbol {\theta}) \mathbf {R} _ {\phi} ]}{\delta \boldsymbol {\theta}} \\ \qquad = \lim _ {\delta \boldsymbol {\theta} \to 0} \frac {\mathrm{Log} [ \mathrm{Exp} (\mathbf {R} _ {\phi} ^ {\top} \delta \boldsymbol {\theta}) ]}{\delta \boldsymbol {\theta}} \\ \qquad = \lim _ {\delta \boldsymbol {\theta} \to 0} \frac {\mathbf {R} _ {\phi} ^ {\top} \delta \boldsymbol {\theta}}{\delta \boldsymbol {\theta}} = \mathbf {R} _ {\phi} ^ {\top} \end{array}
$$

$$
\begin{array}{l} \frac {\partial \mathbf {Q} \circ \mathbf {R}}{\partial \mathbf {R}} = \frac {\partial \mathbf {q} _ {\theta} \otimes \mathbf {r} _ {\phi}}{\partial \phi} = \frac {\partial \mathbf {Q} _ {\theta} \mathbf {R} _ {\phi}}{\partial \phi} = \lim _ {\delta \phi \to 0} \frac {(\mathbf {Q} _ {\theta} (\mathbf {R} _ {\phi} \oplus \delta \phi)) \ominus (\mathbf {Q} _ {\theta} \mathbf {R} _ {\phi})}{\delta \phi} \\ \qquad = \lim _ {\delta \phi \to 0} \frac {\mathrm{Log} [ (\mathbf {Q} _ {\theta} \mathbf {R} _ {\phi}) ^ {\top} (\mathbf {Q} _ {\theta} \mathbf {R} _ {\phi}   \mathrm{Exp} (\delta \phi)) ]}{\delta \phi} \\ \qquad = \lim _ {\delta \phi \to 0} \frac {\mathrm{Log} [ \mathrm{Exp} (\delta \phi) ]}{\delta \phi} \\ \qquad = \lim _ {\delta \phi \to 0} \frac {\delta \phi}{\delta \phi} = \mathbf {I} \end{array}
$$

## 4.4 Perturbations, uncertainties, noise

## 4.4.1 Local perturbations

A perturbed orientation $\tilde{q}$ may be expressed as the composition of the unperturbed orientation q with a small local perturbation $\Delta q_{L}$ . Because of the Hamilton convention, this local perturbation appears at the right hand side of the composition product —we give also the matrix equivalent for comparison,

$$
\tilde {\mathbf {q}} = \mathbf {q} \otimes \Delta \mathbf {q} _ {\mathcal {L}},
$$

$$
\tilde {\mathbf {R}} = \mathbf {R} \Delta \mathbf {R} _ {\mathcal {L}}.\tag{190}
$$

These local perturbation $\Delta\mathbf{q}_{\mathcal{L}}$ (or $\Delta\mathbf{R}_{\mathcal{L}}$ ) is easily obtained from its equivalent vector form $\Delta\phi_{\mathcal{L}} = u\Delta\phi_{\mathcal{L}}$ , defined in the tangent space, using the exponential map. This gives

$$
\tilde {\mathbf {q}} _ {\mathcal {L}} = \mathbf {q} _ {\mathcal {L}} \otimes \operatorname{Exp} (\Delta \phi_ {\mathcal {L}}),
$$

$$
\tilde {\mathbf {R}} _ {\mathcal {L}} = \mathbf {R} _ {\mathcal {L}} \cdot \mathrm{Exp} (\Delta \phi_ {\mathcal {L}})\tag{191}
$$

leading to an expression of the local perturbation

$$
\Delta \phi_ {\mathcal {L}} = \mathrm{Log} (\mathbf {q} _ {\mathcal {L}} ^ {*} \otimes \tilde {\mathbf {q}} _ {\mathcal {L}}) = \mathrm{Log} (\mathbf {R} _ {\mathcal {L}} ^ {\top} \cdot \tilde {\mathbf {R}} _ {\mathcal {L}})\tag{192}
$$

If the perturbation angle $\Delta\phi_{L}$ is small then the perturbation in quaternion and rotation matrix forms can be approximated by the Taylor expansions of (101) and (69) up to the linear terms,

$$
\Delta \mathbf {q} _ {\mathcal {L}} \approx \left[ \begin{array}{c} 1 \\ \frac {1}{2} \Delta \phi_ {\mathcal {L}} \end{array} \right], \quad \Delta \mathbf {R} _ {\mathcal {L}} \approx \mathbf {I} + [ \Delta \phi_ {\mathcal {L}} ] _ {\times}.\tag{193}
$$

Perturbations can therefore be specified in the local vector space $\Delta\phi_{L}$ tangent to the $SO(3)$ manifold at the actual orientation. It is convenient, for example, to express the covariances matrix of these perturbations in this vectorial space, that is, with a regular $3\times3$ covariance matrix.

## 4.4.2 Global perturbations

It is possible and indeed interesting to consider globally-defined perturbations, and likewise for the related derivatives. Global perturbations appear at the left hand side of the composition product, namely,

$$
\tilde {\mathbf {q}} _ {\mathcal {G}} = \mathrm{Exp} (\Delta \phi_ {\mathcal {G}}) \otimes \mathbf {q} _ {\mathcal {G}},
$$

$$
\tilde {\mathbf {R}} _ {\mathcal {G}} = \mathrm{Exp} (\Delta \phi_ {\mathcal {G}}) \cdot \mathbf {R} _ {\mathcal {G}}\tag{194}
$$

leading to an expression of the global perturbation

$$
\Delta \phi_ {\mathcal {G}} = \mathrm{Log} (\tilde {\mathbf {q}} _ {\mathcal {G}} \otimes \mathbf {q} _ {\mathcal {G}} ^ {*}) = \mathrm{Log} (\tilde {\mathbf {R}} _ {\mathcal {G}} \cdot \mathbf {R} _ {\mathcal {G}} ^ {\top})\tag{195}
$$

Again, these perturbations can be specified in the vector space $\Delta\phi_{G}$ tangent to the $SO(3)$ manifold at the origin.

## 4.5 Time derivatives

Expressing the local perturbations in a vector space we can easily develop expressions for the time-derivatives. Just consider $\mathbf{q} = \mathbf{q}(t)$ as the original state, $\tilde{\mathbf{q}} = \mathbf{q}(t + \Delta t)$ as the perturbed state, and apply the definition of the derivative

$$
\frac {d \mathbf {q} (t)}{d t} \triangleq \lim _ {\Delta t \to 0} \frac {\mathbf {q} (t + \Delta t) - \mathbf {q} (t)}{\Delta t},\tag{196}
$$

to the above, with

$$
\pmb {\omega} _ {\mathcal {L}} (t) \triangleq \frac {d \phi_ {\mathcal {L}} (t)}{d t} \triangleq \lim _ {\Delta t \to 0} \frac {\Delta \phi_ {\mathcal {L}}}{\Delta t},\tag{197}
$$

which, being $\Delta\phi_{L}$ a local angular perturbation, corresponds to the angular rates vector in the local frame defined by q.

The development of the time-derivative of the quaternion follows (an analogous reasoning would be used for the rotation matrix)

$$
\begin{array}{l} \dot {\mathbf {q}} \triangleq \lim _ {\Delta t \to 0} \frac {\mathbf {q} (t + \Delta t) - \mathbf {q} (t)}{\Delta t} \\ = \lim _ {\Delta t \to 0} \frac {\mathbf {q} \otimes \Delta \mathbf {q} _ {\mathcal {L}} - \mathbf {q}}{\Delta t} \\ = \lim _ {\Delta t \to 0} \frac {\mathbf {q} \otimes \left(\left[ \begin{array}{c} 1 \\ \Delta \phi_ {\mathcal {L}} / 2 \end{array} \right] - \left[ \begin{array}{c} 1 \\ \mathbf {0} \end{array} \right]\right)}{\Delta t} \\ = \lim _ {\Delta t \to 0} \frac {\mathbf {q} \otimes \left[ \begin{array}{c} 0 \\ \Delta \phi_ {\mathcal {L}} / 2 \end{array} \right]}{\Delta t} \\ = \frac {1}{2}   \mathbf {q} \otimes \left[ \begin{array}{c} 0 \\ \boldsymbol {\omega} _ {\mathcal {L}} \end{array} \right]. \end{array}\tag{198}
$$

Defining

$$
\Omega (\boldsymbol {\omega}) \triangleq [ \boldsymbol {\omega} ] _ {R} = \left[ \begin{array}{c c} 0 & - \boldsymbol {\omega} ^ {\top} \\ \boldsymbol {\omega} & - [ \boldsymbol {\omega} ] _ {\times} \end{array} \right] = \left[ \begin{array}{c c c c} 0 & - \omega_ {x} & - \omega_ {y} & - \omega_ {z} \\ \omega_ {x} & 0 & \omega_ {z} & - \omega_ {y} \\ \omega_ {y} & - \omega_ {z} & 0 & \omega_ {x} \\ \omega_ {z} & \omega_ {y} & - \omega_ {x} & 0 \end{array} \right],\tag{199}
$$

we get from (198) and (17) (we give also its matrix equivalent)

$$
\dot {\mathbf {q}} = \frac {1}{2} \boldsymbol {\Omega} (\boldsymbol {\omega} _ {\mathcal {L}}) \mathbf {q} = \frac {1}{2} \mathbf {q} \otimes \boldsymbol {\omega} _ {\mathcal {L}},
$$

$$
\dot {\mathbf {R}} = \mathbf {R} \left[ \pmb {\omega} _ {\mathcal {L}} \right] _ {\times}.\tag{200}
$$

These expressions are of course identical to (99) and (67), developed in the framework of the rotation group $SO(3)$ . Here, however, and interestingly, we are able to clearly refer the angular rate $\omega_{L}$ to a particular reference frame, which in this case is the local frame defined by the orientation q or R. This has been possible now because we have given the operators q and R a precise geometrical meaning. From this viewpoint, (200) expresses the evolution of the orientation of a reference frame, when the angular rates are expressed locally in this frame.

The time-derivatives associated to global perturbations follow from a development analogous to (198), which results in

$$
\boxed {\dot {\mathbf {q}} = \frac {1}{2} \boldsymbol {\omega} _ {\mathcal {G}} \otimes \mathbf {q},}
$$

$$
\dot {\mathbf {R}} = [ \pmb {\omega} _ {\mathcal {G}} ] _ {\times} \mathbf {R},\tag{201}
$$

where

$$
\pmb {\omega} _ {\mathcal {G}} (t) \triangleq \frac {d \pmb {\phi} _ {\mathcal {G}} (t)}{d t}\tag{202}
$$

is the angular rates vector expressed in the global frame. Eq. (201) expresses the evolution of the orientation of a reference frame, when the angular rates are expressed in the global reference frame.

## 4.5.1 Global-to-local relations

From the previous paragraph, it is worth noticing the following relation between local and global angular rates,

$$
\frac {1}{2} \boldsymbol {\omega} _ {\mathcal {G}} \otimes \mathbf {q} = \dot {\mathbf {q}} = \frac {1}{2} \mathbf {q} \otimes \boldsymbol {\omega} _ {\mathcal {L}}.\tag{203}
$$

Then, post-multiplying by the conjugate quaternion we have

$$
\boldsymbol {\omega} _ {\mathcal {G}} = \mathbf {q} \otimes \boldsymbol {\omega} _ {\mathcal {L}} \otimes \mathbf {q} ^ {*} = \mathbf {R} \boldsymbol {\omega} _ {\mathcal {L}}.\tag{204}
$$

Likewise, considering that $\Delta \phi_{R}\approx \omega \Delta t$ for small $\Delta t$ , we have that

$$
\Delta \phi_ {\mathcal {G}} = \mathbf {q} \otimes \Delta \phi_ {\mathcal {L}} \otimes \mathbf {q} ^ {*} = \mathbf {R} \Delta \phi_ {\mathcal {L}}.\tag{205}
$$

That is, we can transform angular rates vectors $\omega$ and small angular perturbations $\Delta\phi$ via frame transformation, using the quaternion or the rotation matrix, as if they were regular vectors. The same can be seen by posing $\omega = u\omega$ , or $\Delta\phi = u\Delta\phi$ , and noticing that the rotation axis vector u transforms normally, with

$$
\mathbf {u} _ {\mathcal {G}} = \mathbf {q} \otimes \mathbf {u} _ {\mathcal {L}} \otimes \mathbf {q} ^ {*} = \mathbf {R}   \mathbf {u} _ {\mathcal {L}}  .\tag{206}
$$

## 4.5.2 Time-derivative of the quaternion product

We use the regular formula for the derivative of the product,

$$
\left(\mathbf {q} _ {1} \dot {\otimes} \mathbf {q} _ {2}\right) = \dot {\mathbf {q}} _ {1} \otimes \mathbf {q} _ {2} + \mathbf {q} _ {1} \otimes \dot {\mathbf {q}} _ {2}, \quad \left(\mathbf {R} _ {1} \dot {\mathbf {R}} _ {2}\right) = \dot {\mathbf {R}} _ {1} \mathbf {R} _ {2} + \mathbf {R} _ {1} \dot {\mathbf {R}} _ {2},\tag{207}
$$

but noticing that, since the products are non commutative, we need to respect the order of the operands strictly. This means that $(\dot{\mathbf{q}}^{2}) \neq 2\mathbf{q} \otimes \dot{\mathbf{q}}$ , as it would be in the scalar case, but rather

$$
(\dot {\mathbf {q}} ^ {2}) = \dot {\mathbf {q}} \otimes \mathbf {q} + \mathbf {q} \otimes \dot {\mathbf {q}}.\tag{208}
$$

## 4.5.3 Other useful expressions with the derivative

We can derive an expression for the local rotation rate

$$
\boldsymbol {\omega} _ {\mathcal {L}} = 2 \mathbf {q} ^ {*} \otimes \dot {\mathbf {q}},
$$

$$
\left[ \boldsymbol {\omega} _ {\mathcal {L}} \right] _ {\times} = \mathbf {R} ^ {\top} \dot {\mathbf {R}}.\tag{209}
$$

and the global rotation rate,

$$
\boldsymbol {\omega} _ {\mathcal {G}} = 2 \dot {\mathbf {q}} \otimes \mathbf {q} ^ {*},
$$

$$
\left[ \pmb {\omega} _ {\mathcal {G}} \right] _ {\times} = \dot {\mathbf {R}} \mathbf {R} ^ {\top}.\tag{210}
$$

## 4.6 Time-integration of rotation rates

Accumulating rotation over time in quaternion form is done by integrating the differential equation appropriate to the rotation rate definition, that is, (200) for a local rotation rate definition, and (201) for a global one. In the cases we are interested in, the angular rates are measured by local sensors, thus providing local measurements $\omega(t_{n})$ at discrete times $t_{n}=n\Delta t$ . We concentrate here on this case only, for which we reproduce the differential equation (200),

$$
\dot {\mathbf {q}} (t) = \frac {1}{2} \mathbf {q} (t) \otimes \boldsymbol {\omega} (t).\tag{211}
$$

We develop zeroth- and first-order integration methods (Figs. 13 and 14), all based on the Taylor series of $\mathbf{q}(t_n + \Delta t)$ around the time $t = t_n$ . We note $\mathbf{q} \triangleq \mathbf{q}(t)$ and $\mathbf{q}_n \triangleq \mathbf{q}(t_n)$ , and the same for $\omega$ . The Taylor series reads,

$$
\mathbf {q} _ {n + 1} = \mathbf {q} _ {n} + \dot {\mathbf {q}} _ {n} \Delta t + \frac {1}{2 !} \ddot {\mathbf {q}} _ {n} \Delta t ^ {2} + \frac {1}{3 !} \dddot {\mathbf {q}} _ {n} \Delta t ^ {3} + \frac {1}{4 !} \dddot {\mathbf {q}} _ {n} \Delta t ^ {4} + \dots .\tag{212}
$$

The successive derivatives of $q_{n}$ above are easily obtained by repeatedly applying the expression of the quaternion derivative, (211), with $\ddot{\omega}=0$ . We obtain

$$
\dot {\mathbf {q}} _ {n} = \frac {1}{2} \mathbf {q} _ {n} \boldsymbol {\omega} _ {n}\tag{213a}
$$

$$
\ddot {\mathbf {q}} _ {n} = \frac {1}{2 ^ {2}} \mathbf {q} _ {n} \boldsymbol {\omega} _ {n} ^ {2} + \frac {1}{2} \mathbf {q} _ {n} \dot {\boldsymbol {\omega}}\tag{213b}
$$

$$
\ddot {\mathbf {q}} _ {n} = \frac {1}{2 ^ {3}} \mathbf {q} _ {n} \boldsymbol {\omega} _ {n} ^ {3} + \frac {1}{4} \mathbf {q} _ {n} \dot {\boldsymbol {\omega}} \boldsymbol {\omega} _ {n} + \frac {1}{2} \mathbf {q} \boldsymbol {\omega} _ {n} \dot {\boldsymbol {\omega}}\tag{213c}
$$

$$
\mathbf {q} _ {n} ^ {(i \ge 4)} = \frac {1}{2 ^ {i}} \mathbf {q} _ {n} \boldsymbol {\omega} _ {n} ^ {i} + \dots ,\tag{213d}
$$

where we have omitted the $\otimes$ signs for economy of notation, that is, all products and the powers of $\omega$ must be interpreted in terms of the quaternion product.

## 4.6.1 Zeroth order integration

Forward integration In the case where the angular rate $\omega_{n}$ is held constant over the period $[t_{n}, t_{n+1}]$ , we have $\dot{\omega} = 0$ and (212) reduces to,

$$
\mathbf {q} _ {n + 1} = \mathbf {q} _ {n} \otimes \left(1 + \frac {1}{2} \boldsymbol {\omega} _ {n} \Delta t + \frac {1}{2 !} \left(\frac {1}{2} \boldsymbol {\omega} _ {n} \Delta t\right) ^ {2} + \frac {1}{3 !} \left(\frac {1}{2} \boldsymbol {\omega} _ {n} \Delta t\right) ^ {3} + \dots\right),\tag{214}
$$

where we identify the Taylor series (40) of the exponential $e^{\omega_{n}\Delta t/2}$ . From (101), this exponential corresponds to the quaternion representing the incremental rotation $\Delta\theta = \omega_{n}\Delta t$ ,

$$
e ^ {\omega \Delta t / 2} = \mathrm{Exp} (\pmb {\omega} \Delta t) = \mathbf {q} \{\pmb {\omega} \Delta t \} = \left[ \begin{array}{c} \cos (\| \pmb {\omega} \| \Delta t / 2) \\ \frac {\pmb {\omega}}{\| \pmb {\omega} \|} \sin (\| \pmb {\omega} \| \Delta t / 2) \end{array} \right],
$$

![](images/7ff36b0c711576898211571519c17cf42bc04284ee254b9ddb23919d072383a4.jpg)  
Figure 13: Angular velocity approximations for the integral: Red: true velocity. Blue: zero-th order approximations (bottom to top: forward, midward and backward). Green: first order approximation.

![](images/6efb263867dfaa4d0d87727d2e9af3253353262d0c286b720df0a452d3d6fad1.jpg)  
Figure 14: Integration schemes for two consecutive time steps (gray and black arrow sets), where variables sharing the same time stamp have been organized in columns. Left: forward integration. Center: midward and first-order integrations. Right: backward integration.

therefore,

$$
\mathbf {q} _ {n + 1} = \mathbf {q} _ {n} \otimes \mathbf {q} \{\boldsymbol {\omega} _ {n} \Delta t \}\tag{215}
$$

Backward integration We can also consider that the constant velocity over the period $\Delta t$ corresponds to $\omega_{n+1}$ , the velocity measured at the end of the period. This can be developed in a similar manner with a Taylor expansion of $q_{n}$ around $t_{n+1}$ , leading to

$$
\mathbf {q} _ {n + 1} \approx \mathbf {q} _ {n} \otimes \mathbf {q} \{\boldsymbol {\omega} _ {n + 1} \Delta t \}.\tag{216}
$$

We want to remark here that this is the typical integration method when the arriving motion measurements are to be processed in real time, because the integration horizon corresponds to the last measurement (in this case, $t_{n+1}$ , see Fig. 14). To make this more salient, we can re-label the time indices to use $\{n-1,n\}$ instead of $\{n,n+1\}$ , and write,

$$
\boxed {\mathbf {q} _ {n} = \mathbf {q} _ {n - 1} \otimes \mathbf {q} \{\boldsymbol {\omega} _ {n} \Delta t \}}\tag{217}
$$

Midward integration Similarly, if the velocity is considered constant at the median rate over the period $\Delta t$ (which is not necessary the velocity at the midpoint of the period),

$$
\overline {{\omega}} = \frac {\omega_ {n + 1} + \omega_ {n}}{2},\tag{218}
$$

we have,

$$
\mathbf {q} _ {n + 1} = \mathbf {q} _ {n} \otimes \mathbf {q} \{\overline {{\boldsymbol {\omega}}} \Delta t \}\tag{219}
$$

## 4.6.2 First order integration

The angular rate $\omega(t)$ is now linear with time. Its first derivative is constant, and all higher ones are zero,

$$
\dot {\boldsymbol {\omega}} = \frac {\boldsymbol {\omega} _ {n + 1} - \boldsymbol {\omega} _ {n}}{\Delta t}\tag{220}
$$

$$
\ddot {\omega} = \dddot {\omega} = \dots = 0.\tag{221}
$$

We can write the median rate $\overline{\omega}$ in terms of $\omega_{n}$ and $\dot{\omega}$ ,

$$
\overline {{\omega}} = \omega_ {n} + \frac {1}{2} \dot {\omega} \Delta t,\tag{222}
$$

and derive the expression of the powers of $\omega_{n}$ appearing in the quaternion derivatives (213), in terms of the more convenient $\overline{\omega}$ and $\dot{\omega}$ ,

$$
\omega_ {n} = \overline {{\omega}} - \frac {1}{2} \dot {\omega} \Delta t\tag{223a}
$$

$$
\omega_ {n} ^ {2} = \overline {{\omega}} ^ {2} - \frac {1}{2} \overline {{\omega}} \dot {\omega} \Delta t - \frac {1}{2} \dot {\omega} \overline {{\omega}} \Delta t + \frac {1}{4} \dot {\omega} ^ {2} \Delta t ^ {2}\tag{223b}
$$

$$
\pmb {\omega} _ {n} ^ {3} = \overline {{\pmb {\omega}}} ^ {3} - \frac {3}{2} \overline {{\pmb {\omega}}} ^ {2} \dot {\pmb {\omega}} \Delta t + \frac {3}{4} \overline {{\pmb {\omega}}} \dot {\pmb {\omega}} ^ {2} \Delta t ^ {2} + \frac {1}{8} \dot {\pmb {\omega}} ^ {3} \Delta t ^ {3}\tag{223c}
$$

$$
\omega_ {n} ^ {4} = \overline {{\omega}} ^ {4} + \dots .\tag{223d}
$$

Injecting them in the quaternion derivatives, and substituting in the Taylor series (212), we have after proper reordering,

$$
\mathbf {q} _ {n + 1} = \mathbf {q} \left(1 + \frac {1}{2} \overline {{\boldsymbol {\omega}}} \Delta t + \frac {1}{2 !} \left(\frac {1}{2} \overline {{\boldsymbol {\omega}}} \Delta t\right) ^ {2} + \frac {1}{3 !} \left(\frac {1}{2} \overline {{\boldsymbol {\omega}}} \Delta t\right) ^ {3} + \dots\right)\tag{224a}
$$

$$
+ \mathbf {q} \left(- \frac {1}{4} \dot {\boldsymbol {\omega}} + \frac {1}{4} \dot {\boldsymbol {\omega}}\right) \Delta t ^ {2}\tag{224b}
$$

$$
+ \mathbf {q} \left(- \frac {1}{1 6} \overline {{\omega}} \dot {\omega} - \frac {1}{1 6} \dot {\omega} \overline {{\omega}} + \frac {1}{2 4} \dot {\omega} \overline {{\omega}} + \frac {1}{1 2} \overline {{\omega}} \dot {\omega}\right) \Delta t ^ {3}\tag{224c}
$$

$$
+ \mathbf {q} (\dots) \Delta t ^ {4} + \dots ,\tag{224d}
$$

where in (224a) we recognize the exponential series $e^{\overline{\omega}\Delta t/2} = \mathbf{q}\{\overline{\omega}\Delta t\}$ , (224b) vanishes, and (224d) represents terms of high multiplicity that we are going to neglect. This yields after simplification (we recover now the normal $\otimes$ notation),

$$
\mathbf {q} _ {n + 1} = \mathbf {q} _ {n} \otimes \mathbf {q} \{\overline {{\boldsymbol {\omega}}} \Delta t \} + \frac {\Delta t ^ {3}}{4 8} \mathbf {q} _ {n} \otimes (\overline {{\boldsymbol {\omega}}} \otimes \dot {\boldsymbol {\omega}} - \dot {\boldsymbol {\omega}} \otimes \overline {{\boldsymbol {\omega}}}) + \dots .\tag{225}
$$

Substituting $\dot{\omega}$ and $\overline{\omega}$ by their definitions (220) and (218) we get,

$$
\mathbf {q} _ {n + 1} = \mathbf {q} _ {n} \otimes \mathbf {q} \{\overline {{\boldsymbol {\omega}}} \Delta t \} + \frac {\Delta t ^ {2}}{4 8} \mathbf {q} _ {n} \otimes (\boldsymbol {\omega} _ {n} \otimes \boldsymbol {\omega} _ {n + 1} - \boldsymbol {\omega} _ {n + 1} \otimes \boldsymbol {\omega} _ {n}) + \dots ,\tag{226}
$$

which is a result equivalent to (Trawny and Roumeliotis, 2005)'s, but using the Hamilton convention, and the quaternion product form instead of the matrix product form. Finally, since $\mathbf{a}_v\otimes \mathbf{b}_v - \mathbf{b}_v\otimes \mathbf{a}_v = 2\mathbf{a}_v\times \mathbf{b}_v$ , see (33), we have the alternative form,

$$
\mathbf {q} _ {n + 1} \approx \mathbf {q} _ {n} \otimes \left(\mathbf {q} \{\overline {{\boldsymbol {\omega}}} \Delta t \} + \frac {\Delta t ^ {2}}{2 4} \left[ \begin{array}{c} 0 \\ \boldsymbol {\omega} _ {n} \times \boldsymbol {\omega} _ {n + 1} \end{array} \right]\right)  .\tag{227}
$$

In this expression, the first term of the sum is the midward zeroth order integrator (219). The second term is a second-order correction that vanishes when $\omega_{n}$ and $\omega_{n+1}$ are collinear, $^{22}$ i.e., when the axis of rotation has not changed from $t_{n}$ to $t_{n+1}$ .

Case of fixed rotation axis Let us write $\boldsymbol{\omega}(t) = \mathbf{u}(t) \omega(t)$ and call u the axis of rotation. In the case of a constant rotation axis $\mathbf{u}(t) = \mathbf{u}$ , we have $\omega_{n} \times \omega_{n+1} = 0$ and therefore,

$$
\mathbf {q} _ {n + 1} = \mathbf {q} _ {n} \otimes \mathbf {q} \{\mathbf {u} \overline {{\omega}} \Delta t \}.\tag{228}
$$

This result is in fact interesting for cases not limited to first-order derivatives of $\omega(t)$ . In effect, if the axis of rotation is constant, the infinitesimal contributions of rotation into the quaternion commute, i.e.,

$$
\exp (\mathbf {u} \omega_ {1} \delta t _ {1}) \exp (\mathbf {u} \omega_ {2} \delta t _ {2}) = \exp (\mathbf {u} \omega_ {2} \delta t _ {2}) \exp (\mathbf {u} \omega_ {1} \delta t _ {1}) = \exp (\mathbf {u} (\omega_ {1} \delta t _ {1} + \omega_ {2} \delta t _ {2})) ,
$$

and thus we have the identity,

$$
\mathbf {q} _ {n + 1} = \mathbf {q} _ {n} \otimes \exp \left(\frac {\mathbf {u}}{2} \int_ {t _ {n}} ^ {t _ {n + 1}} \omega (t) \delta t\right)\tag{229a}
$$

$$
= \mathbf {q} _ {n} \otimes \exp (\mathbf {u} \Delta \theta_ {n} / 2)\tag{229b}
$$

$$
= \mathbf {q} _ {n} \otimes \mathbf {q} \{\mathbf {u} \Delta \theta_ {n} \}.\tag{229c}
$$

with $\Delta\theta_{n}=\int_{t_{n}}^{t_{n+1}}\omega(t)dt\in\mathbb{R}$ the total angle rotated during the interval $[t_{n},t_{n+1}]$ .

Case of varying rotation axis Clearly, the second term of the sum in (227) captures through $\omega_{n} \times \omega_{n+1} \neq 0$ the effect that a varying rotation axis has on the integrated orientation. For its practical usage, we notice that given usual IMU sampling times $\Delta t \leq 0.01s$ , and the usual near-collinearity of $\omega_{n}$ and $\omega_{n+1}$ due to inertia, this second-order term takes values of the order of $10^{-6}\|\omega\|^{2}$ , or easily smaller. Terms with higher multiplicities of $\omega\Delta t$ are even smaller and have been neglected.

Please note also that, while all zeroth-order integrators result in unit quaternions by construction (because they are computed as the product of two unit quaternions), this is not the case for the first-order integrator due to the sum in $(227)$ . Hence, when using the first-order integrator, and even if the summed term is small as stated, users should take care to check the evolution of the quaternion norm over time, and eventually re-normalize the quaternion if needed, using quaternion updates of the form $q \leftarrow q/\|q\|$ . Only if the constant axis assumption holds, then $(228)$ holds too and this normalization is no longer necessary.

## 5 Error-state kinematics for IMU-driven systems

## 5.1 Motivation

We wish to write the error-estate equations of the kinematics of an inertial system integrating accelerometer and gyrometer readings with bias and noise, using the Hamilton quaternion to represent the orientation in space or attitude.

Accelerometer and gyrometer readings come typically from an Inertial Measurement Unit (IMU). Integrating IMU readings leads to dead-reckoning positioning systems, which drift with time. Avoiding drift is a matter of fusing this information with absolute position readings such as GPS or vision.

The error-state Kalman filter (ESKF) is one of the tools we may use for this purpose. Within the Kalman filtering paradigm, these are the most remarkable assets of the ESKF (Madyastha et al., 2011):

\- The orientation error-state is minimal (i.e., it has the same number of parameters as degrees of freedom), avoiding issues related to over-parametrization (or redundancy) and the consequent risk of singularity of the involved covariances matrices, resulting typically from enforcing constraints.

\- The error-state system is always operating close to the origin, and therefore far from possible parameter singularities, gimbal lock issues, or the like, providing a guarantee that the linearization validity holds at all times.

\- The error-state is always small, meaning that all second-order products are negligible. This makes the computation of Jacobians very easy and fast. Some Jacobians may even be constant or equal to available state magnitudes.

\- The error dynamics are slow because all the large-signal dynamics have been integrated in the nominal-state. This means that we can apply KF corrections (which are the only means to observe the errors) at a lower rate than the predictions.

## 5.2 The error-state Kalman filter explained

In error-state filter formulations, we speak of true-, nominal- and error-state values, the true-state being expressed as a suitable composition (linear sum, quaternion product or matrix product) of the nominal- and the error- states. The idea is to consider the nominal-state as large-signal (integrable in non-linear fashion) and the error-state as small signal (thus linearly integrable and suitable for linear-Gaussian filtering).

The error-state filter can be explained as follows. On one side, high-frequency IMU data $\mathbf{u}_m$ is integrated into a nominal-state $\mathbf{x}$ . This nominal state does not take into account the noise terms $\mathbf{w}$ and other possible model imperfections. As a consequence, it will accumulate errors. These errors are collected in the error-state $\delta \mathbf{x}$ and estimated with the Error-State Kalman Filter (ESKF), this time incorporating all the noise and perturbations. The error-state consists of small-signal magnitudes, and its evolution function is correctly defined by a (time-variant) linear dynamic system, with its dynamic, control and measurement matrices computed from the values of the nominal-state. In parallel with integration of the nominal-state, the ESKF predicts a Gaussian estimate of the error-state. It only predicts, because by now no other measurement is available to correct these estimates. The filter correction is performed at the arrival of information other than IMU (e.g. GPS, vision, etc.), which is able to render the errors observable and which happens generally at a much lower rate than the integration phase. This correction provides a posterior Gaussian estimate of the error-state. After this, the error-state's mean is injected into the nominal-state, then reset to zero. The error-state's covariances matrix is conveniently updated to reflect this reset. The system goes on like this forever.

## 5.3 System kinematics in continuous time

The definition of all the involved variables is summarized in Table 3. Two important decisions regarding conventions are worth mentioning:

\- The angular rates $\omega$ are defined locally with respect to the nominal quaternion. This allows us to use the gyrometer measurements $\omega_{m}$ directly, as they provide body-referenced angular rates.

\- The angular error $\delta\boldsymbol{\theta}$ is also defined locally with respect to the nominal orientation. This is not necessarily the optimal way to proceed, but it corresponds to the choice in most IMU-integration works —what we could call the classical approach. There exists evidence (Li and Mourikis, 2012) that a globally-defined angular error has better properties. This will be explored too in the present document, Section 7, but most of the developments, examples and algorithms here are based in this locally-defined angular error.

Table 3: All variables in the error-state Kalman filter.

<table><tr><td>Magnitude</td><td>True</td><td>Nominal</td><td>Error</td><td>Composition</td><td>Measured</td><td>Noise</td></tr><tr><td>Full state (1)</td><td> $\mathbf{x}_{t}$ </td><td> $\mathbf{x}$ </td><td> $\delta\mathbf{x}$ </td><td> $\mathbf{x}_{t} = \mathbf{x} \oplus \delta\mathbf{x}$ </td><td></td><td></td></tr><tr><td>Position</td><td> $\mathbf{p}_{t}$ </td><td> $\mathbf{p}$ </td><td> $\delta\mathbf{p}$ </td><td> $\mathbf{p}_{t} = \mathbf{p} + \delta\mathbf{p}$ </td><td></td><td></td></tr><tr><td>Velocity</td><td> $\mathbf{v}_{t}$ </td><td> $\mathbf{v}$ </td><td> $\delta\mathbf{v}$ </td><td> $\mathbf{v}_{t} = \mathbf{v} + \delta\mathbf{v}$ </td><td></td><td></td></tr><tr><td>Quaternion (2,3)</td><td> $\mathbf{q}_{t}$ </td><td> $\mathbf{q}$ </td><td> $\delta\mathbf{q}$ </td><td> $\mathbf{q}_{t} = \mathbf{q} \otimes \delta\mathbf{q}$ </td><td></td><td></td></tr><tr><td>Rotation matrix (2,3)</td><td> $\mathbf{R}_{t}$ </td><td> $\mathbf{R}$ </td><td> $\delta\mathbf{R}$ </td><td> $\mathbf{R}_{t} = \mathbf{R} \delta\mathbf{R}$  $\delta\mathbf{q} = e^{\delta\boldsymbol{\theta}/2}$  $\delta\mathbf{R} = e^{[\delta\boldsymbol{\theta}]_{\times}}$ </td><td></td><td></td></tr><tr><td>Angles vector (4)</td><td></td><td></td><td> $\delta\boldsymbol{\theta}$ </td><td></td><td></td><td></td></tr><tr><td>Accelerometer bias</td><td> $\mathbf{a}_{bt}$ </td><td> $\mathbf{a}_{b}$ </td><td> $\delta\mathbf{a}_{b}$ </td><td> $\mathbf{a}_{bt} = \mathbf{a}_{b} + \delta\mathbf{a}_{b}$ </td><td></td><td> $\mathbf{a}_{w}$ </td></tr><tr><td>Gyrometer bias</td><td> $\boldsymbol{\omega}_{bt}$ </td><td> $\boldsymbol{\omega}_{b}$ </td><td> $\delta\boldsymbol{\omega}_{b}$ </td><td> $\boldsymbol{\omega}_{bt} = \boldsymbol{\omega}_{b} + \delta\boldsymbol{\omega}_{b}$ </td><td></td><td> $\boldsymbol{\omega}_{w}$ </td></tr><tr><td>Gravity vector</td><td> $\mathbf{g}_{t}$ </td><td> $\mathbf{g}$ </td><td> $\delta\mathbf{g}$ </td><td> $\mathbf{g}_{t} = \mathbf{g} + \delta\mathbf{g}$ </td><td></td><td></td></tr><tr><td>Acceleration</td><td> $\mathbf{a}_{t}$ </td><td></td><td></td><td></td><td> $\mathbf{a}_{m}$ </td><td> $\mathbf{a}_{n}$ </td></tr><tr><td>Angular rate</td><td> $\boldsymbol{\omega}_{t}$ </td><td></td><td></td><td></td><td> $\boldsymbol{\omega}_{m}$ </td><td> $\boldsymbol{\omega}_{n}$ </td></tr></table>

$^{(1)}$ the symbol $\oplus$ indicates a generic composition  
$^{(2)}$ indicates non-minimal representations

$^{(3)}$ see Table 4 for the composition formula in case of globally-defined angular errors

$^{(4)}$ exponentials defined as in (101) and (69, 77)

## 5.3.1 The true-state kinematics

The true kinematic equations are

$$
\dot {\mathbf {p}} _ {t} = \mathbf {v} _ {t}
$$

$$
\dot {\mathbf {v}} _ {t} = \mathbf {a} _ {t}\tag{230a}
$$

(230b)

$$
\dot {\mathbf {q}} _ {t} = \frac {1}{2} \mathbf {q} _ {t} \otimes \boldsymbol {\omega} _ {t}\tag{230c}
$$

$$
\dot {\mathbf {a}} _ {b t} = \mathbf {a} _ {w}\tag{230d}
$$

$$
\dot {\pmb {\omega}} _ {b t} = \pmb {\omega} _ {w}
$$

$$
\dot {\mathbf {g}} _ {t} = 0\tag{230e}
$$

(230f)

Here, the true acceleration $a_{t}$ and angular rate $\omega_{t}$ are obtained from an IMU in the form of noisy sensor readings $a_{m}$ and $\omega_{m}$ in body frame, namely $^{23}$

$$
\mathbf {a} _ {m} = \mathbf {R} _ {t} ^ {\top} (\mathbf {a} _ {t} - \mathbf {g} _ {t}) + \mathbf {a} _ {b t} + \mathbf {a} _ {n}\tag{231}
$$

$$
\pmb {\omega} _ {m} = \pmb {\omega} _ {t} + \pmb {\omega} _ {b t} + \pmb {\omega} _ {n}\tag{232}
$$

with $R_{t} \triangleq R\{q_{t}\}$ . With this, the true values can be isolated (this means that we have inverted the measurement equations),

$$
\mathbf {a} _ {t} = \mathbf {R} _ {t} (\mathbf {a} _ {m} - \mathbf {a} _ {b t} - \mathbf {a} _ {n}) + \mathbf {g} _ {t}\tag{233}
$$

$$
\pmb {\omega} _ {t} = \pmb {\omega} _ {m} - \pmb {\omega} _ {b t} - \pmb {\omega} _ {n}.\tag{234}
$$

Substituting above yields the kinematic system

$$
\dot {\mathbf {p}} _ {t} = \mathbf {v} _ {t}\tag{235a}
$$

$$
\dot {\mathbf {v}} _ {t} = \mathbf {R} _ {t} (\mathbf {a} _ {m} - \mathbf {a} _ {b t} - \mathbf {a} _ {n}) + \mathbf {g} _ {t}\tag{235b}
$$

$$
\dot {\mathbf {q}} _ {t} = \frac {1}{2} \mathbf {q} _ {t} \otimes (\pmb {\omega} _ {m} - \pmb {\omega} _ {b t} - \pmb {\omega} _ {n})\tag{235c}
$$

$$
\dot {\mathbf {a}} _ {b t} = \mathbf {a} _ {w}\tag{235d}
$$

$$
\dot {\pmb {\omega}} _ {b t} = \pmb {\omega} _ {w}\tag{235e}
$$

$$
\dot {\mathbf {g}} _ {t} = 0\tag{235f}
$$

which we may name $\dot{\mathbf{x}}_{t}=f_{t}(\mathbf{x}_{t},\mathbf{u},\mathbf{w})$ . This system has state $x_{t}$ , is governed by IMU noisy readings $u_{m}$ , and is perturbed by white Gaussian noise w, all defined by

$$
\mathbf {x} _ {t} = \left[ \begin{array}{l} \mathbf {p} _ {t} \\ \mathbf {v} _ {t} \\ \mathbf {q} _ {t} \\ \mathbf {a} _ {b t} \\ \boldsymbol {\omega} _ {b t} \\ \mathbf {g} _ {t} \end{array} \right] \quad \mathbf {u} = \left[ \begin{array}{l} \mathbf {a} _ {m} - \mathbf {a} _ {n} \\ \boldsymbol {\omega} _ {m} - \boldsymbol {\omega} _ {n} \end{array} \right] \quad \mathbf {w} = \left[ \begin{array}{l} \mathbf {a} _ {w} \\ \boldsymbol {\omega} _ {w} \end{array} \right].\tag{236}
$$

It is to note in the above formulation that the gravity vector $g_{t}$ is going to be estimated by the filter. It has a constant evolution equation, (235f), as corresponds to a magnitude that is known to be constant. The system starts at a fixed and arbitrarily known initial orientation $\mathbf{q}_{t}(t=0)=\mathbf{q}_{0}$ , which, being generally not in the horizontal plane, makes the initial gravity vector generally unknown. For simplicity it is usually taken $\mathbf{q}_{0}=(1,0,0,0)$

and thus $R_{0} = R\{q_{0}\} = I$ . We estimate $g_{t}$ expressed in frame $q_{0}$ , and not $q_{t}$ expressed in a horizontal frame, so that the initial uncertainty in orientation is transferred to an initial uncertainty on the gravity direction. We do so to improve linearity: indeed, equation (235b) is now linear in g, which carries all the uncertainty, and the initial orientation $q_{0}$ is known without uncertainty, so that q starts with no uncertainty. Once the gravity vector is estimated the horizontal plane can be recovered and, if desired, the whole state and recovered motion trajectories can be re-oriented to reflect the estimated horizontal. See (Lupton and Sukkarieh, 2009) for further justification. This is of course optional, and the reader is free to remove all equations related to gravity from the system and adopt a more classical approach of considering $g \triangleq (0, 0, -9.8xx)$ , with xx the appropriate decimal digits of the gravity vector on the site of the experiment, and an uncertain initial orientation $q_{0}$ .

## 5.3.2 The nominal-state kinematics

The nominal-state kinematics corresponds to the modeled system without noises or perturbations,

$$
\dot {\mathbf {p}} = \mathbf {v}\tag{237a}
$$

$$
\dot {\mathbf {v}} = \mathbf {R} (\mathbf {a} _ {m} - \mathbf {a} _ {b}) + \mathbf {g}\tag{237b}
$$

$$
\dot {\mathbf {q}} = \frac {1}{2} \mathbf {q} \otimes (\boldsymbol {\omega} _ {m} - \boldsymbol {\omega} _ {b})\tag{237c}
$$

$$
\dot {\mathbf {a}} _ {b} = 0\tag{237d}
$$

$$
\dot {\pmb {\omega}} _ {b} = 0\tag{237e}
$$

$$
\dot {\mathbf {g}} = 0.\tag{237f}
$$

## 5.3.3 The error-state kinematics

The goal is to determine the linearized dynamics of the error-state. For each state equation, we write its composition (in Table 3), solving for the error state and simplifying all second-order infinitesimals. We give here the full error-state dynamic system and proceed afterwards with comments and proofs.

$$
\dot {\delta \mathbf {p}} = \delta \mathbf {v}\tag{238a}
$$

$$
\dot {\delta \mathbf {v}} = - \mathbf {R} [ \mathbf {a} _ {m} - \mathbf {a} _ {b} ] _ {\times} \delta \pmb {\theta} - \mathbf {R} \delta \mathbf {a} _ {b} + \delta \mathbf {g} - \mathbf {R} \mathbf {a} _ {n}\tag{238b}
$$

$$
\dot {\delta \pmb {\theta}} = - \left[ \pmb {\omega} _ {m} - \pmb {\omega} _ {b} \right] _ {\times} \delta \pmb {\theta} - \delta \pmb {\omega} _ {b} - \pmb {\omega} _ {n}\tag{238c}
$$

$$
\dot {\delta \mathbf {a}} _ {b} = \mathbf {a} _ {w}\tag{238d}
$$

$$
\dot {\delta \pmb {\omega}} _ {b} = \pmb {\omega} _ {w}\tag{238e}
$$

$$
\dot {\delta \mathbf {g}} = 0.\tag{238f}
$$

Equations (238a), (238d), (238e) and (238f), respectively of position, both biases, and gravity errors, are derived from linear equations and their error-state dynamics is trivial.

As an example, consider the true and nominal position equations (235a) and (237a), their composition $p_{t} = p + \delta p$ from Table 3, and solve for $\delta\dot{p}$ to obtain (238a).

Equations (238b) and (238c), of velocity and orientation errors, require some non-trivial manipulations of the non-linear equations (235b) and (235c) to obtain the linearized dynamics. Their proofs are developed in the following two sections.

Equation (238b): The linear velocity error. We wish to determine $\delta\dot{v}$ , the dynamics of the velocity errors. We start with the following relations

$$
\mathbf {R} _ {t} = \mathbf {R} (\mathbf {I} + [ \delta \pmb {\theta} ] _ {\times}) + O (\| \delta \pmb {\theta} \| ^ {2})\tag{239}
$$

$$
\dot {\mathbf {v}} = \mathbf {R} \mathbf {a} _ {\mathcal {B}} + \mathbf {g},\tag{240}
$$

where (239) is the small-signal approximation of $R_{t}$ , and in (240) we rewrote (237b) but introducing $a_{B}$ and $\delta a_{B}$ , defined as the large- and small-signal accelerations in body frame,

$$
\mathbf {a} _ {\mathcal {B}} \triangleq \mathbf {a} _ {m} - \mathbf {a} _ {b}\tag{241}
$$

$$
\delta \mathbf {a} _ {\mathcal {B}} \triangleq - \delta \mathbf {a} _ {b} - \mathbf {a} _ {n}\tag{242}
$$

so that we can write the true acceleration in inertial frame as a composition of large- and small-signal terms,

$$
\mathbf {a} _ {t} = \mathbf {R} _ {t} (\mathbf {a} _ {\mathcal {B}} + \delta \mathbf {a} _ {\mathcal {B}}) + \mathbf {g} + \delta \mathbf {g}.\tag{243}
$$

We proceed by writing the expression (235b) of $\dot{v}_{t}$ in two different forms (left and right developments), where the terms $O(\|\delta\boldsymbol{\theta}\|^{2})$ have been ignored,

$$
\begin{array}{r l r} & & {\dot {\mathbf {v}} + \dot {\delta \mathbf {v}} = \boxed {\dot {\mathbf {v}} _ {t}} = \mathbf {R} (\mathbf {I} + [ \delta \pmb {\theta} ] _ {\times}) (\mathbf {a} _ {\mathcal {B}} + \delta \mathbf {a} _ {\mathcal {B}}) + \mathbf {g} + \delta \mathbf {g}} \\ & & {\mathbf {R} \mathbf {a} _ {\mathcal {B}} + \mathbf {g} + \dot {\delta \mathbf {v}} = \qquad = \mathbf {R} \mathbf {a} _ {\mathcal {B}} + \mathbf {R} \delta \mathbf {a} _ {\mathcal {B}} + \mathbf {R} [ \delta \pmb {\theta} ] _ {\times} \mathbf {a} _ {\mathcal {B}} + \mathbf {R} [ \delta \pmb {\theta} ] _ {\times} \delta \mathbf {a} _ {\mathcal {B}} + \mathbf {g} + \delta \mathbf {g}} \end{array}
$$

This leads after removing $Ra_{B} + g$ from left and right to

$$
\dot {\delta \mathbf {v}} = \mathbf {R} (\delta \mathbf {a} _ {\mathcal {B}} + [ \delta \pmb {\theta} ] _ {\times} \mathbf {a} _ {\mathcal {B}}) + \mathbf {R} [ \delta \pmb {\theta} ] _ {\times} \delta \mathbf {a} _ {\mathcal {B}} + \delta \mathbf {g}\tag{244}
$$

Eliminating the second order terms and reorganizing some cross-products (with $[a]_{\times}b = -[b]_{\times}a$ ), we get

$$
\dot {\delta \mathbf {v}} = \mathbf {R} (\delta \mathbf {a} _ {\mathcal {B}} - [ \mathbf {a} _ {\mathcal {B}} ] _ {\times} \delta \pmb {\theta}) + \delta \mathbf {g},\tag{245}
$$

then, recalling (241) and (242),

$$
\dot {\delta \mathbf {v}} = \mathbf {R} \big (- \left[ \mathbf {a} _ {m} - \mathbf {a} _ {b} \right] _ {\times} \delta \pmb {\theta} - \delta \mathbf {a} _ {b} - \mathbf {a} _ {n} \big) + \delta \mathbf {g}\tag{246}
$$

which after proper rearranging leads to the dynamics of the linear velocity error,

$$
\boxed {\dot {\delta \mathbf {v}} = - \mathbf {R} \left[ \mathbf {a} _ {m} - \mathbf {a} _ {b} \right] _ {\times} \delta \boldsymbol {\theta} - \mathbf {R} \delta \mathbf {a} _ {b} + \delta \mathbf {g} - \mathbf {R} \mathbf {a} _ {n}}\tag{247}
$$

To further clean up this expression, we can often times assume that the accelerometer noise is white, uncorrelated and isotropic $^{24}$ ,

$$
\mathbb {E} [ \mathbf {a} _ {n} ] = 0 \qquad \mathbb {E} [ \mathbf {a} _ {n} \mathbf {a} _ {n} ^ {\top} ] = \sigma_ {a} ^ {2} \mathbf {I},\tag{248}
$$

that is, the covariance ellipsoid is a sphere centered at the origin, which means that its mean and covariances matrix are invariant upon rotations (Proof: $E[Ra_{n}] = R\mathbb{E}[a_{n}] = 0$ and $\mathbf{E}[(\mathbf{Ra}_{n})(\mathbf{Ra}_{n})^{\top}] = \mathbf{R}\mathbb{E}[\mathbf{a}_{n}\mathbf{a}_{n}^{\top}]\mathbf{R}^{\top} = \mathbf{R}\sigma_{a}^{2}\mathbf{I}\mathbf{R}^{\top} = \sigma_{a}^{2}\mathbf{I}$ ). Then we can redefine the accelerometer noise vector, with absolutely no consequences, according to

$$
\mathbf {a} _ {n} \leftarrow \mathbf {R a} _ {n}\tag{249}
$$

which gives

$$
\boxed {\dot {\delta \mathbf {v}} = - \mathbf {R} \left[ \mathbf {a} _ {m} - \mathbf {a} _ {b} \right] _ {\times} \delta \boldsymbol {\theta} - \mathbf {R} \delta \mathbf {a} _ {b} + \delta \mathbf {g} - \mathbf {a} _ {n}}.\tag{250}
$$

Equation (238c): The orientation error. We wish to determine $\delta\theta$ , the dynamics of the angular errors. We start with the following relations

$$
\dot {\mathbf {q}} _ {t} = \frac {1}{2} \mathbf {q} _ {t} \otimes \boldsymbol {\omega} _ {t}\tag{251}
$$

$$
\dot {\mathbf {q}} = \frac {1}{2} \mathbf {q} \otimes \boldsymbol {\omega},\tag{252}
$$

which are the true- and nominal- definitions of the quaternion derivatives.

As we did with the acceleration, we group large- and small-signal terms in the angular rate for clarity,

$$
\omega \triangleq \omega_ {m} - \omega_ {b}\tag{253}
$$

$$
\delta \pmb {\omega} \triangleq - \delta \pmb {\omega} _ {b} - \pmb {\omega} _ {n},\tag{254}
$$

so that $\omega_{t}$ can be written with a nominal part and an error part,

$$
\pmb {\omega} _ {t} = \pmb {\omega} + \delta \pmb {\omega}.\tag{255}
$$

We proceed by computing $\dot{q}_{t}$ by two different means (left and right developments)

$$
\begin{array}{c} (\mathbf {q} \otimes \dot {\delta \mathbf {q}}) = \boxed {\dot {\mathbf {q}} _ {t}} = \frac {1}{2} \mathbf {q} _ {t} \otimes \boldsymbol {\omega} _ {t} \\ \dot {\mathbf {q}} \otimes \delta \mathbf {q} + \mathbf {q} \otimes \dot {\delta \mathbf {q}} = \quad = \frac {1}{2} \mathbf {q} \otimes \delta \mathbf {q} \otimes \boldsymbol {\omega} _ {t} \\ \frac {1}{2} \mathbf {q} \otimes \boldsymbol {\omega} \otimes \delta \mathbf {q} + \mathbf {q} \otimes \dot {\delta \mathbf {q}} = \end{array}
$$

simplifying the leading $\mathbf{q}$ and isolating $\delta \mathbf{q}$ we obtain

$$
\begin{array}{r l} \left[ \begin{array}{c} 0 \\ \dot {\delta \pmb {\theta}} \end{array} \right] = & \boxed {2 \dot {\delta \mathbf {q}}} = \delta \mathbf {q} \otimes \pmb {\omega} _ {t} - \pmb {\omega} \otimes \delta \mathbf {q} \\ & = [ \mathbf {q} ] _ {R} (\pmb {\omega} _ {t}) \delta \mathbf {q} - [ \mathbf {q} ] _ {L} (\pmb {\omega}) \delta \mathbf {q} \\ & = \left[ \begin{array}{c c} 0 & - (\pmb {\omega} _ {t} - \pmb {\omega}) ^ {\top} \\ (\pmb {\omega} _ {t} - \pmb {\omega}) & - [ \pmb {\omega} _ {t} + \pmb {\omega} ] _ {\times} \end{array} \right] \left[ \begin{array}{c} 1 \\ \delta \pmb {\theta} / 2 \end{array} \right] + O (\| \delta \pmb {\theta} \| ^ {2}) \\ & = \left[ \begin{array}{c c} 0 & - \delta \pmb {\omega} ^ {\top} \\ \delta \pmb {\omega} & - [ 2 \pmb {\omega} + \delta \pmb {\omega} ] _ {\times} \end{array} \right] \left[ \begin{array}{c} 1 \\ \delta \pmb {\theta} / 2 \end{array} \right] + O (\| \delta \pmb {\theta} \| ^ {2}) \end{array}\tag{256}
$$

which results in one scalar- and one vector- equalities

$$
0 = \delta \boldsymbol {\omega} ^ {\top} \delta \boldsymbol {\theta} + O (| \delta \boldsymbol {\theta} | ^ {2})\tag{257a}
$$

$$
\dot {\delta \boldsymbol {\theta}} = \delta \boldsymbol {\omega} - [ \boldsymbol {\omega} ] _ {\times} \delta \boldsymbol {\theta} - \frac {1}{2} [ \delta \boldsymbol {\omega} ] _ {\times} \delta \boldsymbol {\theta} + O (\| \delta \boldsymbol {\theta} \| ^ {2}).\tag{257b}
$$

The first equation leads to $\delta\boldsymbol{\omega}^{\top}\delta\boldsymbol{\theta}=O(\|\delta\boldsymbol{\theta}\|^{2})$ , which is formed by second-order infinitesimals, not very useful. The second equation yields, after neglecting all second-order terms,

$$
\dot {\delta \pmb {\theta}} = - \left[ \pmb {\omega} \right] _ {\times} \delta \pmb {\theta} + \delta \pmb {\omega}\tag{258}
$$

and finally, recalling (253) and (254), we get the linearized dynamics of the angular error,

$$
\boxed {\delta \dot {\boldsymbol {\theta}} = - \left[ \boldsymbol {\omega} _ {m} - \boldsymbol {\omega} _ {b} \right] _ {\times} \delta \boldsymbol {\theta} - \delta \boldsymbol {\omega} _ {b} - \boldsymbol {\omega} _ {n}}.\tag{259}
$$

## 5.4 System kinematics in discrete time

The differential equations above need to be integrated into differences equations to account for discrete time intervals $\Delta t > 0$ . The integration methods may vary. In some cases, one will be able to use exact closed-form solutions. In other cases, numerical integration of varying degree of accuracy may be employed. Please refer to the Appendices for pertinent details on integration methods.

Integration needs to be done for the following sub-systems:

1. The nominal state.

2. The error-state.

(a) The deterministic part: state dynamics and control.

(b) The stochastic part: noise and perturbations.

## 5.4.1 The nominal state kinematics

We can write the differences equations of the nominal-state as

$$
\mathbf {p} \leftarrow \mathbf {p} + \mathbf {v} \Delta t + \frac {1}{2} (\mathbf {R} (\mathbf {a} _ {m} - \mathbf {a} _ {b}) + \mathbf {g}) \Delta t ^ {2}\tag{260a}
$$

$$
\mathbf {v} \leftarrow \mathbf {v} + (\mathbf {R} (\mathbf {a} _ {m} - \mathbf {a} _ {b}) + \mathbf {g}) \Delta t\tag{260b}
$$

$$
\mathbf {q} \leftarrow \mathbf {q} \otimes \mathbf {q} \{(\boldsymbol {\omega} _ {m} - \boldsymbol {\omega} _ {b}) \Delta t \}\tag{260c}
$$

$$
\mathbf {a} _ {b} \leftarrow \mathbf {a} _ {b}\tag{260d}
$$

$$
\omega_ {b} \leftarrow \omega_ {b}\tag{260e}
$$

$$
\mathbf {g} \leftarrow \mathbf {g},\tag{260f}
$$

where $x \leftarrow f(x, \bullet)$ stands for a time update of the type $x_{k+1} = f(x_k, \bullet_k)$ , $R \triangleq R\{q\}$ is the rotation matrix associated to the current nominal orientation q, and $q\{v\}$ is the quaternion associated to the rotation v, according to (101).

We can also use more precise integration, please see the Appendices for more information.

## 5.4.2 The error-state kinematics

The deterministic part is integrated normally (in this case we follow the methods in App. C.2), and the integration of the stochastic part results in random impulses (see App. E), thus,

$$
\delta \mathbf {p} \leftarrow \delta \mathbf {p} + \delta \mathbf {v} \Delta t\tag{261a}
$$

$$
\delta \mathbf {v} \leftarrow \delta \mathbf {v} + (- \mathbf {R} [ \mathbf {a} _ {m} - \mathbf {a} _ {b} ] _ {\times} \delta \boldsymbol {\theta} - \mathbf {R} \delta \mathbf {a} _ {b} + \delta \mathbf {g}) \Delta t + \mathbf {v} _ {\mathrm{i}}\tag{261b}
$$

$$
\delta \pmb {\theta} \leftarrow \mathbf {R} ^ {\top} \{(\pmb {\omega} _ {m} - \pmb {\omega} _ {b}) \Delta t \} \delta \pmb {\theta} - \delta \pmb {\omega} _ {b} \Delta t + \pmb {\theta} _ {\mathbf {i}}\tag{261c}
$$

$$
\delta \mathbf {a} _ {b} \leftarrow \delta \mathbf {a} _ {b} + \mathbf {a _ {i}}\tag{261d}
$$

$$
\delta \pmb {\omega} _ {b} \gets \delta \pmb {\omega} _ {b} + \pmb {\omega} _ {\mathbf {i}}\tag{261e}
$$

$$
\delta \mathbf {g} \leftarrow \delta \mathbf {g}.\tag{261f}
$$

Here, $v_{i}$ , $\theta_{i}$ , $a_{i}$ and $\omega_{i}$ are the random impulses applied to the velocity, orientation and bias estimates, modeled by white Gaussian processes. Their mean is zero, and their covariances matrices are obtained by integrating the covariances of $a_{n}$ , $\omega_{n}$ , $a_{w}$ and $\omega_{w}$ over the step time $\Delta t$ (see App. E),

$$
\mathbf {V _ {i}} = \sigma_ {\tilde {\mathbf {a}} _ {n}} ^ {2} \Delta t ^ {2} \mathbf {I}
$$

$$
[ m ^ {2} / s ^ {2} ]\tag{262}
$$

$$
\Theta_ {\mathbf {i}} = \sigma_ {\tilde {\omega} _ {n}} ^ {2} \Delta t ^ {2} \mathbf {I}
$$

$$
[ r a d ^ {2} ]\tag{263}
$$

$$
\mathbf {A _ {i}} = \sigma_ {\mathbf {a} _ {w}} ^ {2} \Delta t \mathbf {I}
$$

$$
[ m ^ {2} / s ^ {4} ]\tag{264}
$$

$$
\Omega_ {\mathbf {i}} = \sigma_ {\omega_ {w}} ^ {2} \Delta t \mathbf {I}
$$

$$
[ r a d ^ {2} / s ^ {2} ]\tag{265}
$$

where $\sigma_{\tilde{a}_{n}}[m/s^{2}]$ , $\sigma_{\tilde{\omega}_{n}}[rad/s]$ , $\sigma_{a_{w}}[m/s^{2}\sqrt{s}]$ and $\sigma_{\omega_{w}}[rad/s\sqrt{s}]$ are to be determined from the information in the IMU datasheet, or from experimental measurements.

## 5.4.3 The error-state Jacobian and perturbation matrices

The Jacobians are obtained by simple inspection of the error-state differences equations in the previous section.

To write these equations in compact form, we consider the nominal state vector x, the error state vector $\delta x$ , the input vector $u_{m}$ , and the perturbation impulses vector i, as follows (see App. E.1 for details and justifications),

$$
\mathbf {x} = \left[ \begin{array}{l} \mathbf {p} \\ \mathbf {v} \\ \mathbf {q} \\ \mathbf {a} _ {b} \\ \boldsymbol {\omega} _ {b} \\ \mathbf {g} \end{array} \right], \quad \delta \mathbf {x} = \left[ \begin{array}{l} \delta \mathbf {p} \\ \delta \mathbf {v} \\ \delta \boldsymbol {\theta} \\ \delta \mathbf {a} _ {b} \\ \delta \boldsymbol {\omega} _ {b} \\ \delta \mathbf {g} \end{array} \right], \quad \mathbf {u} _ {m} = \left[ \begin{array}{l} \mathbf {a} _ {m} \\ \boldsymbol {\omega} _ {m} \end{array} \right], \quad \mathbf {i} = \left[ \begin{array}{l} \mathbf {v} _ {\mathrm{i}} \\ \boldsymbol {\theta} _ {\mathrm{i}} \\ \mathbf {a} _ {\mathrm{i}} \\ \boldsymbol {\omega} _ {\mathrm{i}} \end{array} \right]\tag{266}
$$

The error-state system is now

$$
\delta \mathbf {x} \gets f (\mathbf {x}, \delta \mathbf {x}, \mathbf {u} _ {m}, \mathbf {i}) = \mathbf {F} _ {\mathbf {x}} (\mathbf {x}, \mathbf {u} _ {m}) \cdot \delta \mathbf {x} + \mathbf {F} _ {\mathbf {i}} \cdot \mathbf {i},\tag{267}
$$

The ESKF prediction equations are written:

$$
\hat {\delta \mathbf {x}} \leftarrow \mathbf {F} _ {\mathbf {x}} (\mathbf {x}, \mathbf {u} _ {m}) \cdot \hat {\delta \mathbf {x}}\tag{268}
$$

$$
\mathbf {P} \leftarrow \mathbf {F _ {x}} \mathbf {P} \mathbf {F _ {x} ^ {\top}} + \mathbf {F _ {i}} \mathbf {Q _ {i}} \mathbf {F _ {i} ^ {\top}},\tag{269}
$$

where $\delta\mathbf{x}\sim\mathcal{N}\{\hat{\delta\mathbf{x}},\mathbf{P}\}^{25};\mathbf{F}_{\mathbf{x}}$ and $F_{i}$ are the Jacobians of $f()$ with respect to the error and perturbation vectors; and $Q_{i}$ is the covariances matrix of the perturbation impulses.

The expressions of the Jacobian and covariances matrices above are detailed below. All state-related values appearing herein are extracted directly from the nominal state.

$$
\mathbf {F} _ {\mathbf {x}} = \left. \frac {\partial f}{\partial \delta \mathbf {x}} \right| _ {\mathbf {x}, \mathbf {u} _ {m}} = \left[ \begin{array}{c c c c c c} \mathbf {I} & \mathbf {I} \Delta t & 0 & 0 & 0 & 0 \\ 0 & \mathbf {I} & - \mathbf {R} \left[ \mathbf {a} _ {m} - \mathbf {a} _ {b} \right] _ {\times} \Delta t & - \mathbf {R} \Delta t & 0 & \mathbf {I} \Delta t \\ 0 & 0 & \mathbf {R} ^ {\top} \{(\boldsymbol {\omega} _ {m} - \boldsymbol {\omega} _ {b}) \Delta t \} & 0 & - \mathbf {I} \Delta t & 0 \\ 0 & 0 & 0 & \mathbf {I} & 0 & 0 \\ 0 & 0 & 0 & 0 & \mathbf {I} & 0 \\ 0 & 0 & 0 & 0 & 0 & \mathbf {I} \end{array} \right]\tag{270}
$$

$$
\mathbf {F} _ {\mathbf {i}} = \left. \frac {\partial f}{\partial \mathbf {i}} \right| _ {\mathbf {x}, \mathbf {u} _ {m}} = \left[ \begin{array}{c c c c} 0 & 0 & 0 & 0 \\ \mathbf {I} & 0 & 0 & 0 \\ 0 & \mathbf {I} & 0 & 0 \\ 0 & 0 & \mathbf {I} & 0 \\ 0 & 0 & 0 & \mathbf {I} \\ 0 & 0 & 0 & 0 \end{array} \right], \quad \mathbf {Q} _ {\mathbf {i}} = \left[ \begin{array}{c c c c} \mathbf {V} _ {\mathbf {i}} & 0 & 0 & 0 \\ 0 & \boldsymbol {\Theta} _ {\mathbf {i}} & 0 & 0 \\ 0 & 0 & \mathbf {A} _ {\mathbf {i}} & 0 \\ 0 & 0 & 0 & \boldsymbol {\Omega} _ {\mathbf {i}} \end{array} \right].\tag{271}
$$

Please note particularly that $\mathbf{F}_{\mathbf{x}}$ is the system's transition matrix, which can be approximated to different levels of precision in a number of ways. We showed here one of its simplest forms (the Euler form). Se Appendices B to D for further reference.

Please note also that, being the mean of the error $\delta x$ initialized to zero, the linear equation (268) always returns zero. You should of course skip line (268) in your code. I recommend that you write it, though, but that you comment it out so that you are sure you did not forget anything.

And please note, finally, that you should NOT skip the covariance prediction (269)!! In effect, the term $F_{i}Q_{i}F_{i}^{\top}$ is not null and therefore this covariance grows continuously – as it must be in any prediction step.

## 6 Fusing IMU with complementary sensory data

At the arrival of other kind of information than IMU, such as GPS or vision, we proceed to correct the ESKF. In a well-designed system, this should render the IMU biases observable and allow the ESKF to correctly estimate them. There are a myriad of possibilities, the most popular ones being GPS + IMU, monocular vision + IMU, and stereo vision + IMU. In recent years, the combination of visual sensors with IMU has attracted a lot of attention, and thus generated a lot of scientific activity. These vision + IMU setups are very interesting for use in GPS-denied environments, and can be implemented on mobile devices (typically smart phones), but also on UAVs and other small, agile platforms.

While the IMU information has served so far to make predictions to the ESKF, this other information is used to correct the filter, and thus observe the IMU bias errors. The correction consists of three steps:

1. observation of the error-state via filter correction,

2. injection of the observed errors into the nominal state, and

3. reset of the error-state.

These steps are developed in the following sections.

## 6.1 Observation of the error state via filter correction

Suppose as usual that we have a sensor that delivers information that depends on the state, such as

$$
\mathbf {y} = h (\mathbf {x} _ {t}) + v,\tag{272}
$$

where $h()$ is a general nonlinear function of the system state (the true state), and v is a white Gaussian noise with covariance V,

$$
v \sim \mathcal {N} \{0, \mathbf {V} \}.\tag{273}
$$

Our filter is estimating the error state, and therefore the filter correction equations $^{26}$ ,

$$
\mathbf {K} = \mathbf {P H} ^ {\top} (\mathbf {H P H} ^ {\top} + \mathbf {V}) ^ {- 1}\tag{274}
$$

$$
\hat {\delta \mathbf {x}} \leftarrow \mathbf {K} (\mathbf {y} - h (\hat {\mathbf {x}} _ {t}))\tag{275}
$$

$$
\mathbf {P} \leftarrow (\mathbf {I} - \mathbf {K H}) \mathbf {P}\tag{276}
$$

require the Jacobian matrix H to be defined with respect to the error state $\delta x$ , and evaluated at the best true-state estimate $\hat{x}_{t} = x \oplus \delta \hat{x}$ . As the error state mean is zero at this stage (we have not observed it yet), we have $\hat{x}_{t} = x$ and we can use the nominal error x as the evaluation point, leading to

$$
\mathbf {H} \equiv \left. \frac {\partial h}{\partial \delta \mathbf {x}} \right| _ {\mathbf {x}}.\tag{277}
$$

## 6.1.1 Jacobian computation for the filter correction

The Jacobian above might be computed in a number of ways. The most illustrative one is by making use of the chain rule,

$$
\mathbf {H} \triangleq \left. \frac {\partial h}{\partial \delta \mathbf {x}} \right| _ {\mathbf {x}} = \left. \frac {\partial h}{\partial \mathbf {x} _ {t}} \right| _ {\mathbf {x}} \left. \frac {\partial \mathbf {x} _ {t}}{\partial \delta \mathbf {x}} \right| _ {\mathbf {x}} = \mathbf {H} _ {\mathbf {x}} \mathbf {X} _ {\delta \mathbf {x}}.\tag{278}
$$

Here, $H_{x} \triangleq \left. \frac{\partial h}{\partial x_{t}} \right|_{x}$ is the standard Jacobian of $h()$ with respect to its own argument (i.e., the Jacobian one would use in a regular EKF). This first part of the chain rule depends on the measurement function of the particular sensor used, and is not presented here.

The second part, $X_{\delta x} \triangleq \left. \frac{\partial x_{t}}{\partial \delta x} \right|_{x}$ , is the Jacobian of the true state with respect to the error state. This part can be derived here as it only depends on the ESKF composition of states. We have the derivatives,

$$
\mathbf {X} _ {\delta \mathbf {x}} = \left[ \begin{array}{c c c c c} \frac {\partial (\mathbf {p} + \delta \mathbf {p})}{\partial \delta \mathbf {p}} & & & & \\ & \frac {\partial (\mathbf {v} + \delta \mathbf {v})}{\partial \delta \mathbf {v}} & & 0 & \\ & & \frac {\partial (\mathbf {q} \otimes \delta \mathbf {q})}{\partial \delta \boldsymbol {\theta}} & \\ & & & \frac {\partial (\mathbf {a} _ {b} + \delta \mathbf {a} _ {b})}{\partial \delta \mathbf {a} _ {b}} & \\ & 0 & & & \frac {\partial (\boldsymbol {\omega} _ {b} + \delta \boldsymbol {\omega} _ {b})}{\partial \delta \boldsymbol {\omega} _ {b}} \\ & & & & \frac {\partial (\mathbf {g} + \delta \mathbf {g})}{\partial \delta \mathbf {g}} \end{array} \right]\tag{279}
$$

which results in all identity $3 \times 3$ blocks (for example, $\frac{\partial(\mathbf{p}+\delta\mathbf{p})}{\partial\delta\mathbf{p}} = \mathbf{I}_{3}$ ) except for the $4 \times 3$ quaternion term $\mathbf{Q}_{\delta\boldsymbol{\theta}} = \partial(\mathbf{q} \otimes \delta\mathbf{q})/\partial\delta\boldsymbol{\theta}$ . Therefore we have the form,

$$
\mathbf {X} _ {\delta \mathbf {x}} \triangleq \left. \frac {\partial \mathbf {x} _ {t}}{\partial \delta \mathbf {x}} \right| _ {\mathbf {x}} = \left[ \begin{array}{c c c} \mathbf {I} _ {6} & 0 & 0 \\ 0 & \mathbf {Q} _ {\delta \pmb {\theta}} & 0 \\ 0 & 0 & \mathbf {I} _ {9} \end{array} \right]\tag{280}
$$

Using the chain rule, equations (17–19), and the limit $\delta\mathbf{q}\underset{\delta\boldsymbol{\theta}\to0}{\longrightarrow}\left[\frac{1}{2}\delta\boldsymbol{\theta}\right]$ , the quaternion term $Q_{\delta\theta}$ may be derived as follows,

$$
\begin{array}{r l} \mathbf {Q} _ {\delta \boldsymbol {\theta}} \triangleq & \frac {\partial (\mathbf {q} \otimes \delta \mathbf {q})}{\partial \delta \boldsymbol {\theta}} \bigg | _ {\mathbf {q}} = \left. \frac {\partial (\mathbf {q} \otimes \delta \mathbf {q})}{\partial \delta \mathbf {q}} \right| _ {\mathbf {q}} \frac {\partial \delta \mathbf {q}}{\partial \delta \boldsymbol {\theta}} \bigg | _ {\delta \hat {\boldsymbol {\theta}} = 0} \\ & = \left. \frac {\partial ([ \mathbf {q} ] _ {L} \delta \mathbf {q})}{\partial \delta \mathbf {q}} \right| _ {\mathbf {q}} \frac {\partial \left[ \begin{array}{c} 1 \\ \frac {1}{2} \delta \boldsymbol {\theta} \end{array} \right]}{\partial \delta \boldsymbol {\theta}} \bigg | _ {\delta \hat {\boldsymbol {\theta}} = 0} \\ & = [ \mathbf {q} ] _ {L} \frac {1}{2} \left[ \begin{array}{c c c} 0 & 0 & 0 \\ 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{array} \right], \end{array}
$$

which leads to

$$
\mathbf {Q} _ {\delta \pmb {\theta}} = \frac {1}{2} \left[ \begin{array}{c c c} - q _ {x} & - q _ {y} & - q _ {z} \\ q _ {w} & - q _ {z} & q _ {y} \\ q _ {z} & q _ {w} & - q _ {x} \\ - q _ {y} & q _ {x} & q _ {w} \end{array} \right].\tag{281}
$$

## 6.2 Injection of the observed error into the nominal state

After the ESKF update, the nominal state gets updated with the observed error state using the appropriate compositions (sums or quaternion products, see Table 3),

$$
\mathbf {x} \leftarrow \mathbf {x} \oplus \hat {\delta \mathbf {x}},\tag{282}
$$

that is,

$$
\mathbf {p} \leftarrow \mathbf {p} + \hat {\delta \mathbf {p}}\tag{283a}
$$

$$
\mathbf {v} \leftarrow \mathbf {v} + \hat {\delta \mathbf {v}}\tag{283b}
$$

$$
\mathbf {q} \leftarrow \mathbf {q} \otimes \mathbf {q} \{\hat {\delta \boldsymbol {\theta}} \}\tag{283c}
$$

$$
\mathbf {a} _ {b} \leftarrow \mathbf {a} _ {b} + \delta \hat {\mathbf {a}} _ {b}\tag{283d}
$$

$$
\pmb {\omega} _ {b} \gets \pmb {\omega} _ {b} + \delta \hat {\pmb {\omega}} _ {b}\tag{283e}
$$

$$
\mathbf {g} \leftarrow \mathbf {g} + \hat {\delta \mathbf {g}}\tag{283f}
$$

## 6.3 ESKF reset

After error injection into the nominal state, the error state mean $\delta \hat{\mathbf{x}}$ gets reset. This is especially relevant for the orientation part, as the new orientation error will be expressed locally with respect to the orientation frame of the new nominal state. To make the ESKF update complete, the covariance of the error needs to be updated according to this modification.

Let us call the error reset function $g()$ . It is written as follows,

$$
\delta \mathbf {x} \leftarrow g (\delta \mathbf {x}) = \delta \mathbf {x} \ominus \hat {\delta \mathbf {x}},\tag{284}
$$

where $\ominus$ stands for the composition inverse of $\oplus$ . The ESKF error reset operation is thus,

$$
\hat {\delta \mathbf {x}} \gets 0\tag{285}
$$

$$
\mathbf {P} \leftarrow \mathbf {G P G} ^ {\top}.\tag{286}
$$

where $\mathbf{G}$ is the Jacobian matrix defined by,

$$
\mathbf {G} \triangleq \left. \frac {\partial g}{\partial \delta \mathbf {x}} \right| _ {\hat {\delta_ {\mathbf {x}}}}.\tag{287}
$$

Similarly to what happened with the update Jacobian above, this Jacobian is the identity on all diagonal blocks except in the orientation error. We give here the full expression and proceed in the following section with the derivation of the orientation error block, $\partial \delta \pmb{\theta}^{+} / \partial \delta \pmb{\theta} = \mathbf{I} - \left[\frac{1}{2}\delta \hat{\pmb{\theta}}\right]_{\times}$ ,

$$
\mathbf {G} = \left[ \begin{array}{c c c} \mathbf {I} _ {6} & 0 & 0 \\ 0 & \mathbf {I} - \left[ \frac {1}{2} \hat {\delta \pmb {\theta}} \right] _ {\times} & 0 \\ 0 & 0 & \mathbf {I} _ {9} \end{array} \right].\tag{288}
$$

In major cases, the error term $\delta\theta$ can be neglected, leading simply to a Jacobian $G = I_{18}$ , and thus to a trivial error reset. This is what most implementations of the ESKF do. The expression here provided should produce more precise results, which might be of interest for reducing long-term error drift in odometry systems.

## 6.3.1 Jacobian of the reset operation with respect to the orientation error

We want to obtain the expression of the new angular error $\delta\theta^{+}$ with respect to the old error $\delta\theta$ and the observed error $\delta\hat{\theta}$ . Consider these facts:

\- The true orientation does not change on error reset, i.e., $\mathbf{q}_t^+ = \mathbf{q}_t$ . This gives:

$$
\mathbf {q} ^ {+} \otimes \delta \mathbf {q} ^ {+} = \mathbf {q} \otimes \delta \mathbf {q}.\tag{289}
$$

\- The observed error mean has been injected into the nominal state (see (283c) and (123)):

$$
\mathbf {q} ^ {+} = \mathbf {q} \otimes \hat {\delta \mathbf {q}}.\tag{290}
$$

Combining both identities we obtain an expression of $\delta q^{+}$ ,

$$
\delta \mathbf {q} ^ {+} = (\mathbf {q} ^ {+}) ^ {*} \otimes \mathbf {q} \otimes \delta \mathbf {q} = (\mathbf {q} \otimes \hat {\delta \mathbf {q}}) ^ {*} \otimes \mathbf {q} \otimes \delta \mathbf {q} = \hat {\delta \mathbf {q}} ^ {*} \otimes \delta \mathbf {q} = [ \hat {\delta \mathbf {q}} ^ {*} ] _ {L} \cdot \delta \mathbf {q}.\tag{291}
$$

Considering that $\hat{\delta\mathbf{q}}^{*}\approx \left[1-\frac{1}{2}\hat{\delta\pmb{\theta}}\right]$ , the identity above can be expanded as

$$
\left[ \begin{array}{c} 1 \\ \frac {1}{2} \delta \boldsymbol {\theta} ^ {+} \end{array} \right] = \left[ \begin{array}{c c} 1 & \frac {1}{2} \delta \hat {\boldsymbol {\theta}} ^ {\top} \\ - \frac {1}{2} \delta \hat {\boldsymbol {\theta}} & \mathbf {I} - \left[ \frac {1}{2} \delta \hat {\boldsymbol {\theta}} \right] _ {\times} \end{array} \right] \cdot \left[ \begin{array}{c} 1 \\ \frac {1}{2} \delta \boldsymbol {\theta} \end{array} \right] + \mathcal {O} (\| \delta \boldsymbol {\theta} \| ^ {2}) ,\tag{292}
$$

which gives one scalar- and one vector- equations,

$$
\frac {1}{4} \hat {\delta \pmb {\theta}} ^ {\top} \delta \pmb {\theta} = \mathcal {O} (\| \delta \pmb {\theta} \| ^ {2})\tag{293a}
$$

$$
\delta \boldsymbol {\theta} ^ {+} = - \hat {\delta \boldsymbol {\theta}} + \left(\mathbf {I} - \left[ \frac {1}{2} \hat {\delta \boldsymbol {\theta}} \right] _ {\times}\right) \delta \boldsymbol {\theta} + \mathcal {O} (\| \delta \boldsymbol {\theta} \| ^ {2}),\tag{293b}
$$

among which the first one is not very informative in that it is only a relation of infinitesimals. One can show from the second equation that $\delta\hat{\theta}^{+}=0$ , which is what we expect from the reset operation. The Jacobian is obtained by simple inspection,

$$
\boxed {\frac {\partial \delta \boldsymbol {\theta} ^ {+}}{\partial \delta \boldsymbol {\theta}} = \mathbf {I} - \left[ \frac {1}{2} \hat {\delta \boldsymbol {\theta}} \right] _ {\times}}.\tag{294}
$$

## 7 The ESKF using global angular errors

We explore in this section the implications of having the angular error defined in the global reference, as opposed to the local definition we have used so far. We retrace the development of sections 5 and 6, and particularize the subsections that present changes with respect to the new definition.

A global definition of the angular error $\delta\theta$ implies a composition on the left hand side, i.e.,

$$
\mathbf {q} _ {t} = \delta \mathbf {q} \otimes \mathbf {q} = \mathbf {q} \{\delta \boldsymbol {\theta} \} \otimes \mathbf {q}.
$$

We remark for the sake of completeness that we keep the local definition of the angular rates vector $\omega$ , i.e., $\dot{q} = \frac{1}{2}q \otimes \omega$ in continuous time, and therefore $q \leftarrow q \otimes q\{\omega \Delta t\}$ in discrete time, regardless of the angular error being defined globally. This is so for convenience, as the measure of the angular rates provided by the gyrometers is in body frame, that is, local.

## 7.1 System kinematics in continuous time

## 7.1.1 The true- and nominal-state kinematics

True and nominal kinematics do not involve errors and their equations are unchanged.

## 7.1.2 The error-state kinematics

We start by writing the equations of the error-state kinematics, and proceed afterwards with comments and proofs.

$$
\dot {\delta \mathbf {p}} = \delta \mathbf {v}\tag{295a}
$$

$$
\dot {\delta \mathbf {v}} = - \left[ \mathbf {R} (\mathbf {a} _ {m} - b \mathbf {a} _ {b}) \right] _ {\times} \delta \pmb {\theta} - \mathbf {R} \delta \mathbf {a} _ {b} + \delta \mathbf {g} - \mathbf {R} \mathbf {a} _ {n}\tag{295b}
$$

$$
\dot {\delta \pmb {\theta}} = - \mathbf {R} \delta \pmb {\omega} _ {b} - \mathbf {R} \pmb {\omega} _ {n}\tag{295c}
$$

$$
\dot {\delta \mathbf {a}} _ {b} = \mathbf {a} _ {w}\tag{295d}
$$

$$
\dot {\delta \pmb {\omega}} _ {b} = \pmb {\omega} _ {w}\tag{295e}
$$

$$
\dot {\delta \mathbf {g}} = 0,\tag{295f}
$$

where, again, all equations except those of $\delta\dot{v}$ and $\delta\dot{\theta}$ are trivial. The non-trivial expressions are developed below.

Equation (295b): The linear velocity error. We wish to determine $\delta\dot{v}$ , the dynamics of the velocity errors. We start with the following relations

$$
\mathbf {R} _ {t} = (\mathbf {I} + [ \delta \pmb {\theta} ] _ {\times}) \mathbf {R} + O (\| \delta \pmb {\theta} \| ^ {2})\tag{296}
$$

$$
\dot {\mathbf {v}} = \mathbf {R} \mathbf {a} _ {\beta} + \mathbf {g},\tag{297}
$$

where (296) is the small-signal approximation of $R_{t}$ using a globally defined error, and in (297) we introduced $a_{B}$ and $\delta a_{B}$ as the large- and small-signal accelerations in body frame, defined in(241) and (242), as we did for the locally-defined case.

We proceed by writing the expression (235b) of $\dot{v}_{t}$ in two different forms (left and right developments), where the terms $O(\|\delta\boldsymbol{\theta}\|^{2})$ have been ignored,

$$
\begin{array}{r l} \dot {\mathbf {v}} + \dot {\delta \mathbf {v}} = & \boxed {\dot {\mathbf {v}} _ {t}} = (\mathbf {I} + [ \delta \pmb {\theta} ] _ {\times}) \mathbf {R} (\mathbf {a} _ {\mathcal {B}} + \delta \mathbf {a} _ {\mathcal {B}}) + \mathbf {g} + \delta \mathbf {g} \\ \mathbf {R} \mathbf {a} _ {\mathcal {B}} + \mathbf {g} + \dot {\delta \mathbf {v}} = & = \mathbf {R} \mathbf {a} _ {\mathcal {B}} + \mathbf {R} \delta \mathbf {a} _ {\mathcal {B}} + [ \delta \pmb {\theta} ] _ {\times} \mathbf {R} \mathbf {a} _ {\mathcal {B}} + [ \delta \pmb {\theta} ] _ {\times} \mathbf {R} \delta \mathbf {a} _ {\mathcal {B}} + \mathbf {g} + \delta \mathbf {g} \end{array}
$$

This leads after removing $Ra_{B} + g$ from left and right to

$$
\dot {\delta \mathbf {v}} = \mathbf {R} \delta \mathbf {a} _ {\beta} + [ \delta \pmb {\theta} ] _ {\times} \mathbf {R} (\mathbf {a} _ {\beta} + \delta \mathbf {a} _ {\beta}) + \delta \mathbf {g}\tag{298}
$$

Eliminating the second order terms and reorganizing some cross-products (with $[a]_{\times}b = -[b]_{\times}a$ ), we get

$$
\dot {\delta \mathbf {v}} = \mathbf {R} \delta \mathbf {a} _ {\mathcal {B}} - \left[ \mathbf {R} \mathbf {a} _ {\mathcal {B}} \right] _ {\times} \delta \pmb {\theta} + \delta \mathbf {g},\tag{299}
$$

and finally, recalling (241) and (242) and rearranging, we obtain the expression of the derivative of the velocity error,

$$
\dot {\delta \mathbf {v}} = - \left[ \mathbf {R} (\mathbf {a} _ {m} - \mathbf {a} _ {b}) \right] _ {\times} \delta \pmb {\theta} - \mathbf {R} \delta \mathbf {a} _ {b} + \delta \mathbf {g} - \mathbf {R} \mathbf {a} _ {n}\tag{300}
$$

Equation (295c): The orientation error. We start by writing the true- and nominal-definitions of the quaternion derivatives,

$$
\dot {\mathbf {q}} _ {t} = \frac {1}{2} \mathbf {q} _ {t} \otimes \boldsymbol {\omega} _ {t}\tag{301}
$$

$$
\dot {\mathbf {q}} = \frac {1}{2} \mathbf {q} \otimes \boldsymbol {\omega},\tag{302}
$$

and remind that we are using a globally-defined angular error, i.e.,

$$
\mathbf {q} _ {t} = \delta \mathbf {q} \otimes \mathbf {q}.\tag{303}
$$

As we did for the locally-defined error case, we also group large- and small-signal angular rates (253–254). We proceed by computing $\dot{q}_{t}$ by two different means (left and right developments),

$$
\begin{array}{r l r} & & {(\delta \mathbf {q} \overset {\cdot} {\otimes} \mathbf {q}) = \boxed {\dot {\mathbf {q}} _ {t}} = \frac {1}{2} \mathbf {q} _ {t} \otimes \boldsymbol {\omega} _ {t}} \\ & & {\dot {\delta \mathbf {q}} \otimes \mathbf {q} + \delta \mathbf {q} \otimes \dot {\mathbf {q}} =} \\ & & {\dot {\delta \mathbf {q}} \otimes \mathbf {q} + \frac {1}{2} \delta \mathbf {q} \otimes \mathbf {q} \otimes \boldsymbol {\omega} =} \end{array}
$$

Having $\omega_{t} = \omega +\delta \omega$ , this reduces to

$$
\dot {\delta \mathbf {q}} \otimes \mathbf {q} = \frac {1}{2} \delta \mathbf {q} \otimes \mathbf {q} \otimes \delta \boldsymbol {\omega}.\tag{304}
$$

Right-multiplying left and right terms by $q^{*}$ , and recalling that $q \otimes \delta\omega \otimes q^{*} \equiv R\delta\omega$ , we can further develop as follows,

$$
\begin{array}{r l} \dot {\delta \mathbf {q}} & = \frac {1}{2} \delta \mathbf {q} \otimes \mathbf {q} \otimes \delta \boldsymbol {\omega} \otimes \mathbf {q} ^ {*} \\ & = \frac {1}{2} \delta \mathbf {q} \otimes (\mathbf {R} \delta \boldsymbol {\omega}) \\ & = \frac {1}{2} \delta \mathbf {q} \otimes \delta \boldsymbol {\omega} _ {G}, \end{array}\tag{305}
$$

with $\delta \pmb{\omega}_G\triangleq \mathbf{R}\delta \pmb{\omega}$ the small-signal angular rate expressed in the global frame. Then,

$$
\begin{array}{r l} \left[ \begin{array}{l} 0 \\ \dot {\delta \pmb {\theta}} \end{array} \right] = & \boxed {2 \dot {\delta \mathbf {q}}} = \delta \mathbf {q} \otimes \delta \pmb {\omega} _ {G} \\ & = \Omega (\delta \pmb {\omega} _ {G}) \delta \mathbf {q} \\ & = \left[ \begin{array}{c c} 0 & - \delta \pmb {\omega} _ {G} ^ {\top} \\ \delta \pmb {\omega} _ {G} & - [ \delta \pmb {\omega} _ {G} ] _ {\times} \end{array} \right] \left[ \begin{array}{c} 1 \\ \delta \pmb {\theta} / 2 \end{array} \right] + O (\| \delta \pmb {\theta} \| ^ {2}), \end{array}\tag{306}
$$

which results in one scalar- and one vector- equalities

$$
0 = \delta \pmb {\omega} _ {G} ^ {\top} \delta \pmb {\theta} + O (| \delta \pmb {\theta} | ^ {2})\tag{307a}
$$

$$
\dot {\delta \pmb {\theta}} = \delta \pmb {\omega} _ {G} - \frac {1}{2} \left[ \delta \pmb {\omega} _ {G} \right] _ {\times} \delta \pmb {\theta} + O (\| \delta \pmb {\theta} \| ^ {2}).\tag{307b}
$$

The first equation leads to $\delta\boldsymbol{\omega}_{G}^{\top}\delta\boldsymbol{\theta}=O(\|\delta\boldsymbol{\theta}\|^{2})$ , which is formed by second-order infinitesimals, not very useful. The second equation yields, after neglecting all second-order terms,

$$
\dot {\delta \pmb {\theta}} = \delta \pmb {\omega} _ {G} = \mathbf {R} \delta \pmb {\omega}.\tag{308}
$$

Finally, recalling (254), we obtain the linearized dynamics of the global angular error,

$$
\boxed {\dot {\delta \boldsymbol {\theta}} = - \mathbf {R} \delta \boldsymbol {\omega} _ {b} - \mathbf {R} \boldsymbol {\omega} _ {n}}.\tag{309}
$$

## 7.2 System kinematics in discrete time

## 7.2.1 The nominal state

The nominal state equations do not involve errors and are therefore the same as in the case where the orientation error is defined locally.

## 7.2.2 The error state

Using Euler integration, we obtain the following set of differences equations,

$$
\delta \mathbf {p} \leftarrow \delta \mathbf {p} + \delta \mathbf {v} \Delta t\tag{310a}
$$

$$
\delta \mathbf {v} \leftarrow \delta \mathbf {v} + \left(- \left[ \mathbf {R} (\mathbf {a} _ {m} - \mathbf {a} _ {b}) \right] _ {\times} \delta \pmb {\theta} - \mathbf {R} \delta \mathbf {a} _ {b} + \delta \mathbf {g}\right) \Delta t + \mathbf {v _ {i}}\tag{310b}
$$

$$
\delta \pmb {\theta} \leftarrow \delta \pmb {\theta} - \mathbf {R} \delta \pmb {\omega} _ {b} \Delta t + \pmb {\theta} _ {\mathrm{i}}\tag{310c}
$$

$$
\delta \mathbf {a} _ {b} \leftarrow \delta \mathbf {a} _ {b} + \mathbf {a _ {i}}\tag{310d}
$$

$$
\delta \boldsymbol {\omega} _ {b} \leftarrow \delta \boldsymbol {\omega} _ {b} + \boldsymbol {\omega} _ {\mathbf {i}}\tag{310e}
$$

$$
\delta \mathbf {g} \leftarrow \delta \mathbf {g}.\tag{310f}
$$

## 7.2.3 The error state Jacobian and perturbation matrices

The Transition matrix is obtained by simple inspection of the equations above,

$$
\mathbf {F} _ {\mathbf {x}} = \left[ \begin{array}{c c c c c c} \mathbf {I} & \mathbf {I} \Delta t & 0 & 0 & 0 & 0 \\ 0 & \mathbf {I} & \boxed {- [ \mathbf {R} (\mathbf {a} _ {m} - \mathbf {a} _ {b}) ] _ {\times} \Delta t} & - \mathbf {R} \Delta t & 0 & \mathbf {I} \Delta t \\ 0 & 0 & \boxed {\mathbf {I}} & 0 & \boxed {- \mathbf {R} \Delta t} & 0 \\ 0 & 0 & 0 & \mathbf {I} & 0 & 0 \\ 0 & 0 & 0 & 0 & \mathbf {I} & 0 \\ 0 & 0 & 0 & 0 & 0 & \mathbf {I} \end{array} \right].\tag{311}
$$

We observe three changes with respect to the case with a locally-defined angular error (compare the boxed terms in the Jacobian above to the ones in $(270)$ ); these changes are summarized in Table 4.

The perturbation Jacobian and the perturbation matrix are unchanged after considering isotropic noises and the developments of App. E,

$$
\mathbf {F} _ {\mathrm{i}} = \left[ \begin{array}{l l l l} 0 & 0 & 0 & 0 \\ \mathbf {I} & 0 & 0 & 0 \\ 0 & \mathbf {I} & 0 & 0 \\ 0 & 0 & \mathbf {I} & 0 \\ 0 & 0 & 0 & \mathbf {I} \\ 0 & 0 & 0 & 0 \end{array} \right], \quad \mathbf {Q} _ {\mathrm{i}} = \left[ \begin{array}{l l l l} \mathbf {V} _ {\mathrm{i}} & 0 & 0 & 0 \\ 0 & \boldsymbol {\Theta} _ {\mathrm{i}} & 0 & 0 \\ 0 & 0 & \mathbf {A} _ {\mathrm{i}} & 0 \\ 0 & 0 & 0 & \boldsymbol {\Omega} _ {\mathrm{i}} \end{array} \right].\tag{312}
$$

## 7.3 Fusing with complementary sensory data

The fusing equations involving the ESKF machinery vary only slightly when considering global angular errors. We revise these variations in the error state observation via ESKF correction, the injection of the error into the nominal state, and the reset step.

## 7.3.1 Error state observation

The only difference with respect to the local error definition is in the Jacobian block of the observation function that relates the orientation to the angular error. This new block is developed below.

Using (17–19) and the first-order approximation $\delta\mathbf{q}\rightarrow\begin{bmatrix}1\\ \frac{1}{2}\delta\boldsymbol{\theta}\end{bmatrix}$ , the quaternion term $Q_{\delta\theta}$ may be derived as follows,

$$
\mathbf {Q} _ {\delta \boldsymbol {\theta}} \triangleq \left. \frac {\partial (\delta \mathbf {q} \otimes \mathbf {q})}{\partial \delta \boldsymbol {\theta}} \right| _ {\mathbf {q}} = \left. \frac {\partial (\delta \mathbf {q} \otimes \mathbf {q})}{\partial \delta \mathbf {q}} \right| _ {\mathbf {q}} \left. \frac {\partial \delta \mathbf {q}}{\partial \delta \boldsymbol {\theta}} \right| _ {\hat {\delta} \boldsymbol {\theta} = 0}\tag{313a}
$$

$$
= \left[ \mathbf {q} \right] _ {R} \frac {1}{2} \left[ \begin{array}{c c c} 0 & 0 & 0 \\ 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{array} \right]\tag{313b}
$$

$$
= \frac {1}{2} \left[ \begin{array}{c c c} - q _ {x} & - q _ {y} & - q _ {z} \\ q _ {w} & q _ {z} & - q _ {y} \\ - q _ {z} & q _ {w} & q _ {x} \\ q _ {y} & - q _ {x} & q _ {w} \end{array} \right].\tag{313c}
$$

## 7.3.2 Injection of the observed error into the nominal state

The composition $\mathbf{x} \leftarrow \mathbf{x} \oplus \hat{\delta\mathbf{x}}$ of the nominal and error states is depicted as follows,

$$
\mathbf {p} \leftarrow \mathbf {p} + \delta \mathbf {p}\tag{314a}
$$

$$
\mathbf {v} \gets \mathbf {v} + \delta \mathbf {v}\tag{314b}
$$

$$
\mathbf {q} \leftarrow \mathbf {q} \{\hat {\delta \pmb {\theta}} \} \otimes \mathbf {q}\tag{314c}
$$

$$
\mathbf {a} _ {b} \leftarrow \mathbf {a} _ {b} + \delta \mathbf {a} _ {b}\tag{314d}
$$

$$
\pmb {\omega} _ {b} \leftarrow \pmb {\omega} _ {b} + \delta \pmb {\omega} _ {b}\tag{314e}
$$

$$
\mathbf {g} \leftarrow \mathbf {g} + \delta \mathbf {g}.\tag{314f}
$$

where only the equation for the quaternion update has been affected. This is summarized in Table 4.

## 7.3.3 ESKF reset

The ESKF error mean is reset, and the covariance updated, according to,

$$
\hat {\delta \mathbf {x}} \gets 0\tag{315}
$$

$$
\mathbf {P} \leftarrow \mathbf {G P G} ^ {\top}\tag{316}
$$

with the Jacobian

$$
\mathbf {G} = \left[ \begin{array}{c c c} \mathbf {I} _ {6} & 0 & 0 \\ 0 & \mathbf {I} + \left[ \frac {1}{2} \hat {\delta} \boldsymbol {\theta} \right] _ {\times} & 0 \\ 0 & 0 & \mathbf {I} _ {9} \end{array} \right]\tag{317}
$$

whose non-trivial term is developed as follows. Our goal is to obtain the expression of the new angular error $\delta\theta^{+}$ with respect to the old error $\delta\theta$ . We consider these facts:

\- The true orientation does not change on error reset, i.e., $\mathbf{q}_t^+ \equiv \mathbf{q}_t$ . This gives:

$$
\delta \mathbf {q} ^ {+} \otimes \mathbf {q} ^ {+} = \delta \mathbf {q} \otimes \mathbf {q}.\tag{318}
$$

\- The observed error mean has been injected into the nominal state (see (283c) and (123)):

$$
\mathbf {q} ^ {+} = \hat {\delta \mathbf {q}} \otimes \mathbf {q}.\tag{319}
$$

Combining both identities we obtain an expression of the new orientation error with respect to the old one and the observed error $\delta\hat{q}$ ,

$$
\delta \mathbf {q} ^ {+} = \delta \mathbf {q} \otimes \hat {\delta \mathbf {q}} ^ {*} = [ \hat {\delta \mathbf {q}} ^ {*} ] _ {R} \cdot \delta \mathbf {q}.\tag{320}
$$

Considering that $\hat{\delta\mathbf{q}}^*\approx \left[1-\frac{1}{2}\hat{\delta\pmb{\theta}}\right]$ , the identity above can be expanded as

$$
\left[ \begin{array}{c} 1 \\ \frac {1}{2} \delta \boldsymbol {\theta} ^ {+} \end{array} \right] = \left[ \begin{array}{c c} 1 & \frac {1}{2} \hat {\delta \boldsymbol {\theta}} ^ {\top} \\ - \frac {1}{2} \hat {\delta \boldsymbol {\theta}} & \mathbf {I} + \left[ \frac {1}{2} \hat {\delta \boldsymbol {\theta}} \right] _ {\times} \end{array} \right] \cdot \left[ \begin{array}{c} 1 \\ \frac {1}{2} \delta \boldsymbol {\theta} \end{array} \right] + \mathcal {O} (\| \delta \boldsymbol {\theta} \| ^ {2})\tag{321}
$$

Table 4: Algorithm modifications related to the definition of the orientation errors.

<table><tr><td>Context</td><td>Item</td><td>local angular error</td><td>global angular error</td></tr><tr><td>Error composition</td><td> $\mathbf{q}_t$ </td><td> $\mathbf{q}_t = \mathbf{q} \otimes \delta \mathbf{q}$ </td><td> $\mathbf{q}_t = \delta \mathbf{q} \otimes \mathbf{q}$ </td></tr><tr><td>Euler integration</td><td> $\partial \delta \mathbf{v}^{+}/\partial \delta \boldsymbol{\theta}$  $\partial \delta \boldsymbol{\theta}^{+}/\partial \delta \boldsymbol{\theta}$  $\partial \delta \boldsymbol{\theta}^{+}/\partial \delta \boldsymbol{\omega}_{b}$ </td><td> $-\mathbf{R}\left[\mathbf{a}_{m}-\mathbf{a}_{b}\right]_{\times} \Delta t$  $\mathbf{R}^{\top}\{(\boldsymbol{\omega}_{m}-\boldsymbol{\omega}_{b})\Delta t\}$  $-\mathbf{I}\Delta t$ </td><td> $-\left[\mathbf{R}(\mathbf{a}_{m}-\mathbf{a}_{b})\right]_{\times} \Delta t$  $\mathbf{I}$  $-\mathbf{R}\Delta t$ </td></tr><tr><td>Error observation</td><td> $\mathbf{Q}_{\delta \boldsymbol{\theta}}$ </td><td> $\frac{1}{2} \begin{bmatrix} -q_x & -q_y & -q_z \\ q_w & -q_z & q_y \\ q_z & q_w & -q_x \\ -q_y & q_x & q_w \end{bmatrix}$ </td><td> $\frac{1}{2} \begin{bmatrix} -q_x & -q_y & -q_z \\ q_w & q_z & -q_y \\ -q_z & q_w & q_x \\ q_y & -q_x & q_w \end{bmatrix}$ </td></tr><tr><td>Error injection</td><td></td><td> $\mathbf{q} \leftarrow \mathbf{q} \otimes \mathbf{q}\{\hat{\delta \boldsymbol{\theta}}\}$ </td><td> $\mathbf{q} \leftarrow \mathbf{q}\{\hat{\delta \boldsymbol{\theta}}\} \otimes \mathbf{q}$ </td></tr><tr><td>Error reset</td><td> $\partial \delta \boldsymbol{\theta}^{+}/\partial \delta \boldsymbol{\theta}$ </td><td> $\mathbf{I} - \left[\frac{1}{2}\hat{\delta \boldsymbol{\theta}}\right]_{\times}$ </td><td> $\mathbf{I} + \left[\frac{1}{2}\hat{\delta \boldsymbol{\theta}}\right]_{\times}$ </td></tr></table>

which gives one scalar- and one vector- equations,

$$
\frac {1}{4} \hat {\delta \pmb {\theta}} ^ {\top} \delta \pmb {\theta} = \mathcal {O} (\| \delta \pmb {\theta} \| ^ {2})\tag{322a}
$$

$$
\delta \pmb {\theta} ^ {+} = - \hat {\delta \pmb {\theta}} + \left(\mathbf {I} + \left[ \frac {1}{2} \hat {\delta \pmb {\theta}} \right] _ {\times}\right) \delta \pmb {\theta} + \mathcal {O} (\| \delta \pmb {\theta} \| ^ {2})\tag{322b}
$$

among which the first one is not very informative in that it is only a relation of infinitesimals. One can show from the second equation that $\delta\hat{\theta}^{+}=0$ , which is what we expect from the reset operation. The Jacobian is obtained by simple inspection,

$$
\boxed {\frac {\partial \delta \boldsymbol {\theta} ^ {+}}{\partial \delta \boldsymbol {\theta}} = \mathbf {I} + \left[ \frac {1}{2} \hat {\delta \boldsymbol {\theta}} \right] _ {\times}}.\tag{323}
$$

The difference with respect to the local error case is summarized in Table 4.

## A Runge-Kutta numerical integration methods

We aim at integrating nonlinear differential equations of the form

$$
\dot {\mathbf {x}} = f (t, \mathbf {x})\tag{324}
$$

over a limited time interval $\Delta t$ , in order to convert them to a differences equation, i.e.,

$$
\mathbf {x} (t + \Delta t) = \mathbf {x} (t) + \int_ {t} ^ {t + \Delta t} f (\tau , \mathbf {x} (\tau)) d \tau ,\tag{325}
$$

or equivalently, if we assume that $t_n = n\Delta t$ and $\mathbf{x}_n \triangleq \mathbf{x}(t_n)$ ,

$$
\mathbf {x} _ {n + 1} = \mathbf {x} _ {n} + \int_ {n \Delta t} ^ {(n + 1) \Delta t} f (\tau , \mathbf {x} (\tau)) d \tau .\tag{326}
$$

One of the most utilized family of methods is the Runge-Kutta methods (from now on, RK). These methods use several iterations to estimate the derivative over the interval, and then use this derivative to integrate over the step $\Delta t$ .

In the sections that follow, several RK methods are presented, from the simplest one to the most general one, and are named according to their most common name.

NOTE: All the material here is taken from the Runge-Kutta method entry in the English Wikipedia.

## A.1 The Euler method

The Euler method assumes that the derivative $f(\cdot)$ is constant over the interval, and therefore

$$
\boxed {\mathbf {x} _ {n + 1} = \mathbf {x} _ {n} + \Delta t \cdot f (t _ {n}, \mathbf {x} _ {n}) .}\tag{327}
$$

Put as a general RK method, this corresponds to a single-stage method, which can be depicted as follows. Compute the derivative at the initial point,

$$
k _ {1} = f (t _ {n}, \mathbf {x} _ {n}),\tag{328}
$$

and use it to compute the integrated value at the end point,

$$
\mathbf {x} _ {n + 1} = \mathbf {x} _ {n} + \Delta t \cdot k _ {1}.\tag{329}
$$

## A.2 The midpoint method

The midpoint method assumes that the derivative is the one at the midpoint of the interval, and makes one iteration to compute the value of x at this midpoint, i.e.,

$$
\left| \mathbf {x} _ {n + 1} = \mathbf {x} _ {n} + \Delta t \cdot f \left(t _ {n} + \frac {1}{2} \Delta t, \mathbf {x} _ {n} + \frac {1}{2} \Delta t \cdot f (t _ {n}, \mathbf {x} _ {n})\right) \right|.\tag{330}
$$

The midpoint method can be explained as a two-step method as follows. First, use the Euler method to integrate until the midpoint, using $k_{1}$ as defined previously,

$$
k _ {1} = f (t _ {n}, \mathbf {x} _ {n})\tag{331}
$$

$$
\mathbf {x} (t _ {n} + \frac {1}{2} \Delta t) = \mathbf {x} _ {n} + \frac {1}{2} \Delta t \cdot k _ {1}.\tag{332}
$$

Then use this value to evaluate the derivative at the midpoint, $k_{2}$ , leading to the integration

$$
k _ {2} = f (t _ {n} + \frac {1}{2} \Delta t, \mathbf {x} (t _ {n} + \frac {1}{2} \Delta t))\tag{333}
$$

$$
\mathbf {x} _ {n + 1} = \mathbf {x} _ {n} + \Delta t \cdot k _ {2}.\tag{334}
$$

## A.3 The RK4 method

This is usually referred to as simply the Runge-Kutta method. It assumes evaluation values for $f()$ at the start, midpoint and end of the interval. And it uses four stages or iterations to compute the integral, with four derivatives, $k_{1} \ldots k_{4}$ , that are obtained sequentially. These derivatives, or slopes, are then weight-averaged to obtain the 4th-order estimate of the derivative in the interval.

The RK4 method is better specified as a small algorithm than a one-step formula like the two methods above. The RK4 integration step is,

$$
\boxed {\mathbf {x} _ {n + 1} = \mathbf {x} _ {n} + \frac {\Delta t}{6} \Big (k _ {1} + 2 k _ {2} + 2 k _ {3} + k _ {4} \Big)} ,\tag{335}
$$

that is, the increment is computed by assuming a slope which is the weighted average of the slopes $k_{1}, k_{2}, k_{3}, k_{4}$ , with

$$
k _ {1} = f (t _ {n}, \mathbf {x} _ {n})\tag{336}
$$

$$
k _ {2} = f \Big (t _ {n} + \frac {1}{2} \Delta t, \mathbf {x} _ {n} + \frac {\Delta t}{2} k _ {1} \Big)\tag{337}
$$

$$
k _ {3} = f \Big (t _ {n} + \frac {1}{2} \Delta t, \mathbf {x} _ {n} + \frac {\Delta t}{2} k _ {2} \Big)\tag{338}
$$

$$
k _ {4} = f \left(t _ {n} + \Delta t, \mathbf {x} _ {n} + \Delta t \cdot k _ {3}\right).\tag{339}
$$

The different slopes have the following interpretation:

\- $k_{1}$ is the slope at the beginning of the interval, using $\mathbf{x}_n$ , (Euler's method);

\- $k_{2}$ is the slope at the midpoint of the interval, using $\mathbf{x}_n + \frac{1}{2}\Delta t\cdot k_1$ , (midpoint method);

\- $k_{3}$ is again the slope at the midpoint, but now using $\mathbf{x}_n + \frac{1}{2}\Delta t\cdot k_2$ ;

\- $k_{4}$ is the slope at the end of the interval, using $\mathbf{x}_n + \Delta t \cdot k_3$ .

## A.4 General Runge-Kutta method

More elaborated RK methods are possible. They aim at either reduce the error and/or increase stability. They take the general form

$$
\boxed {\mathbf {x} _ {n + 1} = \mathbf {x} _ {n} + \Delta t \sum_ {i = 1} ^ {s} b _ {i} k _ {i}} ,\tag{340}
$$

where

$$
k _ {i} = f \left(t _ {n} + \Delta t \cdot c _ {i}, \mathbf {x} _ {n} + \Delta t \sum_ {j = 1} ^ {s} a _ {i j} k _ {j}\right),\tag{341}
$$

that is, the number of iterations (the order of the method) is s, the averaging weights are defined by $b_{i}$ , the evaluation time instants by $c_{i}$ , and the slopes $k_{i}$ are determined using the values $a_{ij}$ . Depending on the structure of the terms $a_{ij}$ , one can have explicit or implicit RK methods.

\- In explicit methods, all $k_{i}$ are computed sequentially, i.e., using only previously computed values. This implies that the matrix $[a_{ij}]$ is lower triangular with zero diagonal entries (i.e., $a_{ij} = 0$ for $j \geq i$ ). Euler, midpoint and RK4 methods are explicit.

\- Implicit methods have a full $[a_{ij}]$ matrix and require the solution of a linear set of equations to determine all $k_i$ . They are therefore costlier to compute, but they are able to improve on accuracy and stability with respect to explicit methods.

Please refer to specialized documentation for more detailed information.

## B Closed-form integration methods

In many cases it is possible to arrive to a closed-form expression for the integration step. We consider now the case of a first-order linear differential equation,

$$
\dot {\mathbf {x}} (t) = \mathbf {A} \cdot \mathbf {x} (t),\tag{342}
$$

that is, the relation is linear and constant over the interval. In such cases, the integration over the interval $[t_{n}, t_{n} + \Delta t]$ results in

$$
\mathbf {x} _ {n + 1} = e ^ {\mathbf {A} \cdot \Delta t} \mathbf {x} _ {n} = \Phi \mathbf {x} _ {n},\tag{343}
$$

where $\Phi$ is known as the transition matrix. The Taylor expansion of this transition matrix is

$$
\Phi = e ^ {\mathbf {A} \Delta t} = \mathbf {I} + \mathbf {A} \Delta t + \frac {1}{2} \mathbf {A} ^ {2} \Delta t ^ {2} + \frac {1}{3 !} \mathbf {A} ^ {3} \Delta t ^ {3} + \dots = \sum_ {k = 0} ^ {\infty} \frac {1}{k !} \mathbf {A} ^ {k} \Delta t ^ {k}.\tag{344}
$$

When writing this series for known instances of A, it is sometimes possible to identify known series in the result. This allows writing the resulting integration in closed form. A few examples follow.

## B.1 Integration of the angular error

For example, consider the angular error dynamics without bias and noise (a cleaned version of Eq. (238c)),

$$
\dot {\delta \pmb {\theta}} = - \left[ \pmb {\omega} \right] _ {\times} \delta \pmb {\theta}\tag{345}
$$

Its transition matrix can be written as a Taylor series,

(346)

$$
= \mathbf {I} - [ \boldsymbol {\omega} ] _ {\times} \Delta t + \frac {1}{2} [ \boldsymbol {\omega} ] _ {\times} ^ {2} \Delta t ^ {2} - \frac {1}{3 !} [ \boldsymbol {\omega} ] _ {\times} ^ {3} \Delta t ^ {3} + \frac {1}{4 !} [ \boldsymbol {\omega} ] _ {\times} ^ {4} \Delta t ^ {4} - \dots\tag{347}
$$

Now defining $\omega\Delta t \triangleq u\Delta\theta$ , the unitary axis of rotation and the rotated angle, and applying (75), we can group terms and get

$$
\begin{array}{r l} & {\Phi = \mathbf {I} - [ \mathbf {u} ] _ {\times} \Delta \theta + \frac {1}{2} [ \mathbf {u} ] _ {\times} ^ {2} \Delta \theta^ {2} - \frac {1}{3 !} [ \mathbf {u} ] _ {\times} ^ {3} \Delta \theta^ {3} + \frac {1}{4 !} [ \mathbf {y} ] _ {\times} ^ {4} \Delta \theta^ {4} - \dots} \\ & {\qquad = \mathbf {I} - [ \mathbf {u} ] _ {\times} \left(\Delta \theta - \frac {\Delta \theta^ {3}}{3 !} + \frac {\Delta \theta^ {5}}{5 !} - \dots\right) + [ \mathbf {u} ] _ {\times} ^ {2} \left(\frac {\Delta \theta^ {2}}{2 !} - \frac {\Delta \theta^ {4}}{4 !} + \frac {\Delta \theta^ {6}}{6 !} - \dots\right)} \\ & {\qquad = \mathbf {I} - [ \mathbf {u} ] _ {\times} \sin \Delta \theta + [ \mathbf {u} ] _ {\times} ^ {2} (1 - \cos \Delta \theta),} \end{array}\tag{348}
$$

which is a closed-form solution.

This solution corresponds to a rotation matrix, $\Phi = R\{-u\Delta\theta\} = R\{\omega\Delta t\}^{\top}$ , according to the Rodrigues rotation formula (77), a result that could be obtained by direct inspection of (346) and recalling (69). Let us therefore write this as the final closed-form result,

$$
\boxed {\Phi = \mathbf {R} \{\boldsymbol {\omega} \Delta t \} ^ {\top}}.\tag{349}
$$

## B.2 Simplified IMU example

Consider the simplified, IMU driven system with error-state dynamics governed by,

$$
\dot {\delta \mathbf {p}} = \delta \mathbf {v}\tag{350a}
$$

$$
\dot {\delta \mathbf {v}} = - \mathbf {R} [ \mathbf {a} ] _ {\times} \delta \pmb {\theta}\tag{350b}
$$

$$
\dot {\delta \pmb {\theta}} = - [ \pmb {\omega} ] _ {\times} \delta \pmb {\theta},\tag{350c}
$$

where $(\mathbf{a},\omega)$ are the IMU readings, and we have obviated gravity and sensor biases. This system is defined by the state vector and the dynamic matrix,

$$
\mathbf {x} = \left[ \begin{array}{l} \delta \mathbf {p} \\ \delta \mathbf {v} \\ \delta \boldsymbol {\theta} \end{array} \right] \quad \mathbf {A} = \left[ \begin{array}{c c c} 0 & \mathbf {P} _ {\mathrm{v}} & 0 \\ 0 & 0 & \mathbf {V} _ {\boldsymbol {\theta}} \\ 0 & 0 & \Theta_ {\boldsymbol {\theta}} \end{array} \right].\tag{351}
$$

with

$$
\mathbf {P _ {v}} = \mathbf {I}\tag{352}
$$

$$
\mathbf {V} _ {\boldsymbol {\theta}} = - \mathbf {R} [ \mathbf {a} ] _ {\times}\tag{353}
$$

$$
\Theta_ {\theta} = - \left[ \omega \right] _ {\times}\tag{354}
$$

Its integration with a step time $\Delta t$ is $\mathbf{x}_{n+1}=e^{(\mathbf{A}\Delta t)}\cdot\mathbf{x}_{n}=\Phi\cdot\mathbf{x}_{n}$ . The transition matrix $\Phi$ admits a Taylor development (344), in increasing powers of $A\Delta t$ . We can write a few powers of A to get an illustration of their general form,

$$
\mathbf {A} = \left[ \begin{array}{c c c} 0 & \mathbf {P _ {v}} & 0 \\ 0 & 0 & \mathbf {V _ {\theta}} \\ 0 & 0 & \Theta_ {\theta} \end{array} \right], \mathbf {A} ^ {2} = \left[ \begin{array}{c c c} 0 & 0 & \mathbf {P _ {v}} \mathbf {V _ {\theta}} \\ 0 & 0 & \mathbf {V _ {\theta}} \Theta_ {\theta} \\ 0 & 0 & \Theta_ {\theta} ^ {2} \end{array} \right], \mathbf {A} ^ {3} = \left[ \begin{array}{c c c} 0 & 0 & \mathbf {P _ {v}} \mathbf {V _ {\theta}} \Theta_ {\theta} \\ 0 & 0 & \mathbf {V _ {\theta}} \Theta_ {\theta} ^ {2} \\ 0 & 0 & \Theta_ {\theta} ^ {3} \end{array} \right], \mathbf {A} ^ {4} = \left[ \begin{array}{c c c} 0 & 0 & \mathbf {P _ {v}} \mathbf {V _ {\theta}} \Theta_ {\theta} ^ {2} \\ 0 & 0 & \mathbf {V _ {\theta}} \Theta_ {\theta} ^ {3} \\ 0 & 0 & \Theta_ {\theta} ^ {4} \end{array} \right],\tag{355}
$$

from which it is now visible that, for $k > 1$ ,

$$
\mathbf {A} ^ {k > 1} = \left[ \begin{array}{c c c} 0 & 0 & \mathbf {P} _ {v} \mathbf {V} _ {\boldsymbol {\theta}} \Theta_ {\boldsymbol {\theta}} ^ {k - 2} \\ 0 & 0 & \mathbf {V} _ {\boldsymbol {\theta}} \Theta_ {\boldsymbol {\theta}} ^ {k - 1} \\ 0 & 0 & \Theta_ {\boldsymbol {\theta}} ^ {k} \end{array} \right]\tag{356}
$$

We can observe that the terms in the increasing powers of $\mathbf{A}$ have a fixed part and an increasing power of $\Theta_{\theta}$ . These powers can lead to closed form solutions, as in the previous section.

Let us partition the matrix $\Phi$ as follows,

$$
\Phi = \left[ \begin{array}{c c c} \mathbf {I} & \Phi_ {\mathbf {p v}} & \Phi_ {\mathbf {p} \theta} \\ 0 & \mathbf {I} & \Phi_ {\mathbf {v} \theta} \\ 0 & 0 & \Phi_ {\theta \theta} \end{array} \right],\tag{357}
$$

and let us advance step by step, exploring all the non-zero blocks of $\Phi$ one by one.

Trivial diagonal terms Starting by the two upper terms in the diagonal, they are the identity as shown.

Rotational diagonal term Next is the rotational diagonal term $\Phi_{\theta\theta}$ , relating the new angular error to the old angular error. Writing the full Taylor series for this term leads to

$$
\Phi_ {\boldsymbol {\theta} \boldsymbol {\theta}} = \sum_ {k = 0} ^ {\infty} \frac {1}{k !} \Theta_ {\boldsymbol {\theta}} ^ {k} \Delta t ^ {k} = \sum_ {k = 0} ^ {\infty} \frac {1}{k !} [ - \boldsymbol {\omega} ] _ {\times} ^ {k} \Delta t ^ {k},\tag{358}
$$

which corresponds, as we have seen in the previous section, to our well-known rotation matrix,

$$
\boxed {\Phi_ {\theta \theta} = \mathbf {R} \{\boldsymbol {\omega} \Delta t \} ^ {\top}}.\tag{359}
$$

Position-vs-velocity term The simplest off-diagonal term is $\Phi_{pv}$ , which is

$$
\boxed {\Phi_ {\mathbf {p v}} = \mathbf {P} _ {\mathbf {v}} \Delta t = \mathbf {I} \Delta t}.\tag{360}
$$

Velocity-vs-angle term Let us now move to the term $\Phi_{v,\theta}$ , by writing its series,

$$
\Phi_ {\mathbf {v} \theta} = \mathbf {V} _ {\theta} \Delta t + \frac {1}{2} \mathbf {V} _ {\theta} \Theta_ {\theta} \Delta t ^ {2} + \frac {1}{3 !} \mathbf {V} _ {\theta} \Theta_ {\theta} ^ {2} \Delta t ^ {3} + \dots\tag{361}
$$

$$
\Phi_ {\mathbf {v} \pmb {\theta}} = \Delta t \mathbf {V} _ {\pmb {\theta}} (\mathbf {I} + \frac {1}{2} \Theta_ {\pmb {\theta}} \Delta t + \frac {1}{3 !} \Theta_ {\pmb {\theta}} ^ {2} \Delta t ^ {2} + \dots)\tag{362}
$$

which reduces to

$$
\Phi_ {\mathbf {v} \boldsymbol {\theta}} = \Delta t \mathbf {V} _ {\boldsymbol {\theta}} \left(\mathbf {I} + \sum_ {k \geq 1} \frac {(\Theta_ {\boldsymbol {\theta}} \Delta t) ^ {k}}{(k + 1) !}\right)\tag{363}
$$

At this point we have two options. We can truncate the series at the first significant term, obtaining $\Phi_{v\theta} = V_{\theta}\Delta t$ , but this would not be a closed-form. See next section for results using this simplified method. Alternatively, let us factor $V_{\theta}$ out and write

$$
\Phi_ {\mathbf {v} \theta} = \mathbf {V} _ {\theta} \Sigma_ {1}\tag{364}
$$

with

$$
\Sigma_ {1} = \mathbf {I} \Delta t + \frac {1}{2} \Theta_ {\theta} \Delta t ^ {2} + \frac {1}{3 !} \Theta_ {\theta} ^ {2} \Delta t ^ {3} + \dots .\tag{365}
$$

The series $\Sigma_{1}$ resembles the series we wrote for $\Phi_{\theta\theta}$ , (358), with two exceptions:

\- The powers of $\Theta_{\theta}$ in $\Sigma_{1}$ do not match with the rational coefficients $\frac{1}{k!}$ and with the powers of $\Delta t$ . In fact, we remark here that the subindex “1” in $\Sigma_{1}$ denotes the fact that one power of $\Theta_{\theta}$ is missing in each of the members.

\- Some terms at the start of the series are missing. Again, the subindex “1” indicates that one such term is missing.

The first issue may be solved by applying (75) to (354), which yields the identity

$$
\Theta_ {\boldsymbol {\theta}} = \frac {[ \boldsymbol {\omega} ] _ {\times} ^ {3}}{\| \boldsymbol {\omega} \| ^ {2}} = \frac {- \Theta_ {\boldsymbol {\theta}} ^ {3}}{\| \boldsymbol {\omega} \| ^ {2}}.\tag{366}
$$

This expression allows us to increase the exponents of $\Theta_{\theta}$ in the series by two, and write, if $\omega \neq 0$ ,

$$
\Sigma_ {1} = \mathbf {I} \Delta t - \frac {\Theta_ {\pmb {\theta}}}{\| \pmb {\omega} \| ^ {2}} \left(\frac {1}{2} \Theta_ {\pmb {\theta}} ^ {2} \Delta t ^ {2} + \frac {1}{3 !} \Theta_ {\pmb {\theta}} ^ {3} \Delta t ^ {3} + \dots\right),\tag{367}
$$

and $\Sigma_{1}=I\Delta t$ otherwise. All the powers in the new series match with the correct coefficients. Of course, and as indicated before, some terms are missing. This second issue can be solved by adding and subtracting the missing terms, and substituting the full series by its closed form. We obtain

$$
\Sigma_ {1} = \mathbf {I} \Delta t - \frac {\Theta_ {\pmb {\theta}}}{\| \pmb {\omega} \| ^ {2}} \left(\mathbf {R} \{\pmb {\omega} \Delta t \} ^ {\top} - \mathbf {I} - \Theta_ {\pmb {\theta}} \Delta t\right),\tag{368}
$$

which is a closed-form solution valid if $\omega \neq 0$ . Therefore we can finally write

$$
\omega \rightarrow 0\tag{369a}
$$

$$
\Phi_ {\mathbf {v} \theta} = \left\{- \mathbf {R} [ \mathbf {a} ] _ {\times} \left(\mathbf {I} \Delta t + \frac {[ \boldsymbol {\omega} ] _ {\times}}{\| \boldsymbol {\omega} \| ^ {2}} \left(\mathbf {R} \{\boldsymbol {\omega} \Delta t \} ^ {\top} - \mathbf {I} + [ \boldsymbol {\omega} ] _ {\times} \Delta t\right)\right) \quad \boldsymbol {\omega} \neq 0 \right.\tag{369b}
$$

Position-vs-angle term Let us finally board the term $\Phi_{p\theta}$ . Its Taylor series is,

$$
\Phi_ {\mathbf {p} \theta} = \frac {1}{2} \mathbf {P _ {v}} \mathbf {V} _ {\theta} \Delta t ^ {2} + \frac {1}{3 !} \mathbf {P _ {v}} \mathbf {V} _ {\theta} \Theta_ {\theta} \Delta t ^ {3} + \frac {1}{4 !} \mathbf {P _ {v}} \mathbf {V} _ {\theta} \Theta_ {\theta} ^ {2} \Delta t ^ {4} + \dots\tag{370}
$$

We factor out the constant terms and get,

$$
\Phi_ {\mathbf {p} \theta} = \mathbf {P _ {v}} \mathbf {V} _ {\theta} \Sigma_ {2},\tag{371}
$$

with

$$
\Sigma_ {2} = \frac {1}{2} \mathbf {I} \Delta t ^ {2} + \frac {1}{3 !} \Theta_ {\theta} \Delta t ^ {3} + \frac {1}{4 !} \Theta_ {\theta} ^ {2} \Delta t ^ {4} + \dots .\tag{372}
$$

where we note the subindex “2” in $\Sigma_{2}$ admits the following interpretation:

\- Two powers of $\Theta_{\theta}$ are missing in each term of the series,

\- The first two terms of the series are missing.

Again, we use (366) to increase the exponents of $\Theta_{\theta}$ , yielding

$$
\Sigma_ {2} = \frac {1}{2} \mathbf {I} \Delta t ^ {2} - \frac {1}{\| \boldsymbol {\omega} \| ^ {2}} \left(\frac {1}{3 !} \Theta_ {\boldsymbol {\theta}} ^ {3} \Delta t ^ {3} + \frac {1}{4 !} \Theta_ {\boldsymbol {\theta}} ^ {4} \Delta t ^ {4} + \dots\right).\tag{373}
$$

We substitute the incomplete series by its closed form,

$$
\Sigma_ {2} = \frac {1}{2} \mathbf {I} \Delta t ^ {2} - \frac {1}{\| \boldsymbol {\omega} \| ^ {2}} \left(\mathbf {R} \{\boldsymbol {\omega} \Delta t \} ^ {\top} - \mathbf {I} - \Theta_ {\boldsymbol {\theta}} \Delta t - \frac {1}{2} \Theta_ {\boldsymbol {\theta}} ^ {2} \Delta t ^ {2}\right),\tag{374}
$$

which leads to the final result

$$
\Phi_ {\mathbf {p} \theta} = \left\{ \begin{array}{l l} - \mathbf {R}   [ \mathbf {a} ] _ {\times}   \frac {\Delta t ^ {2}}{2} & \omega \to 0 \\ - \mathbf {R}   [ \mathbf {a} ] _ {\times} \left(\frac {1}{2} \mathbf {I} \Delta t ^ {2} - \frac {1}{\| \boldsymbol {\omega} \| ^ {2}} \left(\mathbf {R} \{\boldsymbol {\omega} \Delta t \} ^ {\top} - \sum_ {k = 0} ^ {2} \frac {(- [ \boldsymbol {\omega} ] _ {\times}   \Delta t) ^ {k}}{k !}\right)\right) & \omega \neq 0 \end{array} \right.\tag{375a}
$$

(375b)

## B.3 Full IMU example

In order to give means to generalize the methods exposed in the simplified IMU example, we need to examine the full IMU case from a little closer.

Consider the full IMU system (238), which can be posed as

$$
\dot {\delta \mathbf {x}} = \mathbf {A} \delta \mathbf {x} + \mathbf {B} \mathbf {w},\tag{376}
$$

whose discrete-time integration requires the transition matrix

$$
\Phi = \sum_ {k = 0} ^ {\infty} \frac {1}{k !} \mathbf {A} ^ {k} \Delta t ^ {k} = \mathbf {I} + \mathbf {A} \Delta t + \frac {1}{2} \mathbf {A} ^ {2} \Delta t ^ {2} + \dots ,\tag{377}
$$

which we wish to compute. The dynamic matrix A is block-sparse, and its blocks can be easily determined by examining the original equations (238),

$$
\mathbf {A} = \left[ \begin{array}{l l l l l l} 0 & \mathbf {P _ {v}} & 0 & 0 & 0 & 0 \\ 0 & 0 & \mathbf {V _ {\theta}} & \mathbf {V _ {a}} & 0 & \mathbf {V _ {g}} \\ 0 & 0 & \Theta_ {\theta} & 0 & \Theta_ {\omega} & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \end{array} \right].\tag{378}
$$

As we did before, let us write a few powers of A,

$$
\mathbf {A} ^ {2} = \left[ \begin{array}{c c c c c c} 0 & 0 & \mathrm{P} _ {\mathrm{v}} \mathrm{V} _ {\theta} & \mathrm{P} _ {\mathrm{v}} \mathrm{V} _ {\mathrm{a}} & 0 & \mathrm{P} _ {\mathrm{v}} \mathrm{V} _ {\mathrm{g}} \\ 0 & 0 & \mathrm{V} _ {\theta} \Theta_ {\theta} & 0 & \mathrm{V} _ {\theta} \Theta_ {\omega} & 0 \\ 0 & 0 & \Theta_ {\theta} ^ {2} & 0 & \Theta_ {\theta} \Theta_ {\omega} & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \end{array} \right]
$$

$$
\mathbf {A} ^ {3} = \left[ \begin{array}{c c c c c c} 0 & 0 & \mathbf {P _ {v}} \mathbf {V} _ {\theta} \Theta_ {\theta} & 0 & \mathbf {P _ {v}} \mathbf {V} _ {\theta} \Theta_ {\omega} & 0 \\ 0 & 0 & \mathbf {V} _ {\theta} \Theta_ {\theta} ^ {2} & 0 & \mathbf {V} _ {\theta} \Theta_ {\theta} \Theta_ {\omega} & 0 \\ 0 & 0 & \Theta_ {\theta} ^ {3} & 0 & \Theta_ {\theta} ^ {2} \Theta_ {\omega} & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \end{array} \right]
$$

$$
\mathbf {A} ^ {4} = \left[ \begin{array}{c c c c c c} 0 & 0 & \mathbf {P _ {v}} \mathbf {V} _ {\theta} \Theta_ {\theta} ^ {2} & 0 & \mathbf {P _ {v}} \mathbf {V} _ {\theta} \Theta_ {\theta} \Theta_ {\omega} & 0 \\ 0 & 0 & \mathbf {V} _ {\theta} \Theta_ {\theta} ^ {3} & 0 & \mathbf {V} _ {\theta} \Theta_ {\theta} ^ {2} \Theta_ {\omega} & 0 \\ 0 & 0 & \Theta_ {\theta} ^ {4} & 0 & \Theta_ {\theta} ^ {3} \Theta_ {\omega} & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \end{array} \right].
$$

Basically, we observe the following,

\- The only term in the diagonal of $\mathbf{A}$ , the rotational term $\Theta_{\theta}$ , propagates right and up in the sequence of powers $\mathbf{A}^k$ . All terms not affected by this propagation vanish. This propagation affects the structure of the sequence $\{\mathbf{A}^k\}$ in the three following aspects:

\- The sparsity of the powers of $\mathbf{A}$ is stabilized after the 3rd power. That is to say, there are no more non-zero blocks appearing or vanishing for powers of $\mathbf{A}$ higher than 3.

\- The upper-left $3 \times 3$ block, corresponding to the simplified IMU model in the previous example, has not changed with respect to that example. Therefore, its closed-form solution developed before holds.

\- The terms related to the gyrometer bias error (those of the fifth column) introduce a similar series of powers of $\Theta_{\theta}$ , which can be solved with the same techniques we used in the simplified example.

We are interested at this point in finding a generalized method to board the construction of the closed-form elements of the transition matrix $\Phi$ . Let us recall the remarks we made about the series $\Sigma_{1}$ and $\Sigma_{2}$ ,

\- The subindex coincides with the lacking powers of $\Theta_{\theta}$ in each of the members of the series.

\- The subindex coincides with the number of terms missing at the beginning of the series.

Taking care of these properties, let us introduce the series $\Sigma_{n}(\mathbf{X},y)$ , defined by $^{27}$

$$
\Sigma_ {n} (\mathbf {X}, y) \triangleq \sum_ {k = n} ^ {\infty} \frac {1}{k !} \mathbf {X} ^ {k - n} y ^ {k} = \sum_ {k = 0} ^ {\infty} \frac {1}{(k + n) !} \mathbf {X} ^ {k} y ^ {k + n} = y ^ {n} \sum_ {k = 0} ^ {\infty} \frac {1}{(k + n) !} \mathbf {X} ^ {k} y ^ {k}\tag{379}
$$

in which the sum starts at term n and the terms lack n powers of the matrix X. It follows immediately that $\Sigma_{1}$ and $\Sigma_{2}$ respond to

$$
\Sigma_ {n} = \Sigma_ {n} (\Theta_ {\pmb {\theta}}, \Delta t),\tag{380}
$$

and that $\Sigma_{0}=\mathbf{R}\{\omega\Delta t\}^{\top}$ . We can now write the transition matrix (377) as a function of these series,

$$
\Phi = \left[ \begin{array}{c c c c c c} \mathbf {I} & \mathbf {P _ {v}} \Delta t & \mathbf {P _ {v}} \mathbf {V _ {\theta}} \Sigma_ {2} & \frac {1}{2} \mathbf {P _ {v}} \mathbf {V _ {a}} \Delta t ^ {2} & \mathbf {P _ {v}} \mathbf {V _ {\theta}} \Sigma_ {3} \boldsymbol {\theta_ {\omega}} & \frac {1}{2} \mathbf {P _ {v}} \mathbf {V _ {g}} \Delta t ^ {2} \\ 0 & \mathbf {I} & \mathbf {V _ {\theta}} \Sigma_ {1} & \mathbf {V _ {a}} \Delta t & \mathbf {V _ {\theta}} \Sigma_ {2} \boldsymbol {\theta_ {\omega}} & \mathbf {V _ {g}} \Delta t \\ 0 & 0 & \Sigma_ {0} & 0 & \Sigma_ {1} \boldsymbol {\theta_ {\omega}} & 0 \\ 0 & 0 & 0 & \mathbf {I} & 0 & 0 \\ 0 & 0 & 0 & 0 & \mathbf {I} & 0 \\ 0 & 0 & 0 & 0 & 0 & \mathbf {I} \end{array} \right]\tag{381}
$$

Our problem has now derived to the problem of finding a general, closed-form expression for $\Sigma_{n}$ . Let us observe the closed-form results we have obtained so far,

$$
\Sigma_ {0} = \mathbf {R} \{\boldsymbol {\omega} \Delta t \} ^ {\top}\tag{382}
$$

$$
\Sigma_ {1} = \mathbf {I} \Delta t - \frac {\Theta_ {\pmb {\theta}}}{\| \pmb {\omega} \| ^ {2}} \left(\mathbf {R} \{\pmb {\omega} \Delta t \} ^ {\top} - \mathbf {I} - \Theta_ {\pmb {\theta}} \Delta t\right)\tag{383}
$$

$$
\Sigma_ {2} = \frac {1}{2} \mathbf {I} \Delta t ^ {2} - \frac {1}{\| \boldsymbol {\omega} \| ^ {2}} \left(\mathbf {R} \{\boldsymbol {\omega} \Delta t \} ^ {\top} - \mathbf {I} - \Theta_ {\boldsymbol {\theta}} \Delta t - \frac {1}{2} \Theta_ {\boldsymbol {\theta}} ^ {2} \Delta t ^ {2}\right) .\tag{384}
$$

In order to develop $\Sigma_{3}$ , we need to apply the identity (366) twice (because we lack three powers, and each application of (366) increases this number by only two), getting

$$
\Sigma_ {3} = \frac {1}{3 !} {\bf I} \Delta t ^ {3} + \frac {\Theta_ {\theta}}{\| \pmb {\omega} \| ^ {4}} \left(\frac {1}{4 !} \Theta_ {\theta} ^ {4} \Delta t ^ {4} + \frac {1}{5 !} \Theta_ {\theta} ^ {5} \Delta t ^ {5} + \dots\right) ,\tag{385}
$$

which leads to

$$
\Sigma_ {3} = \frac {1}{3 !} \mathbf {I} \Delta t ^ {3} + \frac {\Theta_ {\pmb {\theta}}}{\| \pmb {\omega} \| ^ {4}} \left(\mathbf {R} \{\pmb {\omega} \Delta t \} ^ {\top} - \mathbf {I} - \Theta_ {\pmb {\theta}} \Delta t - \frac {1}{2} \Theta_ {\pmb {\theta}} ^ {2} \Delta t ^ {2} - \frac {1}{3 !} \Theta_ {\pmb {\theta}} ^ {3} \Delta t ^ {3}\right) .\tag{386}
$$

By careful inspection of the series $\Sigma_0\ldots \Sigma_3$ , we can now derive a general, closed-form expression for $\Sigma_{n}$ , as follows,

$$
\int \frac {1}{n !} \mathbf {I} \Delta t ^ {n}
$$

$$
\omega \rightarrow 0\tag{387a}
$$

$$
n = 0\tag{387b}
$$

$$
\Sigma_ {n} = \left\{\frac {1}{n !} \mathbf {I} \Delta t ^ {n} - \frac {(- 1) ^ {\frac {n + 1}{2}} [ \boldsymbol {\omega} ] _ {\times}}{\| \boldsymbol {\omega} \| ^ {n + 1}} \left(\mathbf {R} \{\boldsymbol {\omega} \Delta t \} ^ {\top} - \sum_ {k = 0} ^ {n} \frac {(- [ \boldsymbol {\omega} ] _ {\times} \Delta t) ^ {k}}{k !}\right) \right.
$$

$$
n \mathrm{odd}\tag{387c}
$$

$$
\left\lfloor \frac {1}{n !} \mathbf {I} \Delta t ^ {n} + \frac {(- 1) ^ {\frac {n}{2}}}{\| \boldsymbol {\omega} \| ^ {n}} \left(\mathbf {R} \{\boldsymbol {\omega} \Delta t \} ^ {\top} - \sum_ {k = 0} ^ {n} \frac {(- [ \boldsymbol {\omega} ] _ {\times} \Delta t) ^ {k}}{k !}\right) \right.
$$

$$
n \mathrm{even}\tag{387d}
$$

The final result for the transition matrix $\Phi$ follows immediately by substituting the appropriate values of $\Sigma_{n}$ , $n \in \{0,1,2,3\}$ , in the corresponding positions of (381).

It might be worth noticing that the series now appearing in these new expressions of $\Sigma_{n}$ have a finite number of terms, and thus that they can be effectively computed. That is to say, the expression of $\Sigma_{n}$ is a closed form as long as $n < \infty$ , which is always the case. For the current example, we have $n \leq 3$ as can be observed in (381).

## C Approximate methods using truncated series

In the previous section, we have devised closed-form expressions for the transition matrix of complex, IMU-driven dynamic systems written in their linearized, error-state form $\delta\dot{x}=$

Aδx. Closed form expressions may always be of interest, but it is unclear up to which point we should be worried about high order errors and their impact on the performance of real algorithms. This remark is particularly relevant in systems where IMU integration errors are observed (and thus compensated for) at relatively high rates, such as visual-inertial or GPS-inertial fusion schemes.

In this section we devise methods for approximating the transition matrix. They start from the same assumption that the transition matrix can be expressed as a Taylor series, and then truncate these series at the most significant terms. This truncation can be done system-wise, or block-wise.

## C.1 System-wise truncation

## C.1.1 First order truncation: the finite differences method

A typical, widely used integration method for systems of the type

$$
\dot {\mathbf {x}} = f (t, \mathbf {x})
$$

is based on the finite-differences method for the computation of the derivative, i.e.,

$$
\dot {\mathbf {x}} \triangleq \lim _ {\delta t \to 0} \frac {\mathbf {x} (t + \delta t) - \mathbf {x} (t)}{\delta t} \approx \frac {\mathbf {x} _ {n + 1} - \mathbf {x} _ {n}}{\Delta t}.\tag{388}
$$

This leads immediately to

$$
\mathbf {x} _ {n + 1} \approx \mathbf {x} _ {n} + \Delta t f (t _ {n}, \mathbf {x} _ {n}),\tag{389}
$$

which is precisely the Euler method. Linearization of the function $f()$ at the beginning of the integration interval leads to

$$
\mathbf {x} _ {n + 1} \approx \mathbf {x} _ {n} + \Delta t \mathbf {A x} _ {n},\tag{390a}
$$

where $\mathbf{A} \triangleq \frac{\partial f}{\partial \mathbf{x}}(t_{n}, \mathbf{x}_{n})$ is a Jacobian matrix. This is strictly equivalent to writing the exponential solution to the linearized differential equation and truncating the series at the linear term (i.e., the following relation is identical to the previous one),

$$
\mathbf {x} _ {n + 1} = e ^ {\mathbf {A} \Delta t} \mathbf {x} _ {n} \approx (\mathbf {I} + \Delta t \mathbf {A}) \mathbf {x} _ {n}.\tag{390b}
$$

This means that the Euler method (App. A.1), the finite-differences method, and the first-order system-wise Taylor truncation method, are all the same. We get the approximate transition matrix,

$$
\boxed {\Phi \approx \mathbf {I} + \Delta t \mathbf {A}}.\tag{391}
$$

For the simplified IMU example of Section B.2, the finite-differences method results in the approximated transition matrix

$$
\Phi \approx \left[ \begin{array}{c c c} {\bf I} & {\bf I} \Delta t & 0 \\ 0 & {\bf I} & - {\bf R} [ {\bf a} ] _ {\times} \Delta t \\ 0 & 0 & {\bf I} - [ {\pmb \omega} \Delta t ] _ {\times} \end{array} \right].\tag{392}
$$

However, we already know from Section B.1 that the rotational term has a compact, closed-form solution, $\Phi_{\theta\theta} = \mathbf{R}(\omega\Delta t)^{\top}$ . It is convenient to re-write the transition matrix according to it,

$$
\Phi \approx \left[ \begin{array}{c c c} {\bf I} & {\bf I} \Delta t & 0 \\ 0 & {\bf I} & - {\bf R} [ {\bf a} ] _ {\times} \Delta t \\ 0 & 0 & {\bf R} \{\pmb {\omega} \Delta t \} ^ {\top} \end{array} \right].\tag{393}
$$

## C.1.2 N-th order truncation

Truncating at higher orders will increase the precision of the approximated transition matrix. A particularly interesting order of truncation is that which exploits the sparsity of the result to its maximum. In other words, the order after which no new non-zero terms appear.

For the simplified IMU example of Section B.2, this order is 2, resulting in

$$
\boldsymbol {\Phi} \approx \mathbf {I} + \mathbf {A} \Delta t + \frac {1}{2} \mathbf {A} ^ {2} \Delta t ^ {2} = \left[ \begin{array}{c c c} \mathbf {I} & \mathbf {I} \Delta t & - \frac {1}{2} \mathbf {R} [ \mathbf {a} ] _ {\times} \Delta t ^ {2} \\ 0 & \mathbf {I} & - \mathbf {R} [ \mathbf {a} ] _ {\times} (\mathbf {I} - \frac {1}{2} [ \boldsymbol {\omega} ] _ {\times} \Delta t) \Delta t \\ 0 & 0 & \mathbf {R} \{\boldsymbol {\omega} \Delta t \} ^ {\top} \end{array} \right].\tag{394}
$$

In the full IMU example of Section B.3, the is order 3, resulting in

$$
\boldsymbol {\Phi} \approx \mathbf {I} + \mathbf {A} \Delta t + \frac {1}{2} \mathbf {A} ^ {2} \Delta t ^ {2} + \frac {1}{6} \mathbf {A} ^ {3} \Delta t ^ {3},\tag{395}
$$

whose full form is not given here for space reasons. The reader may consult the expressions of A, $A^{2}$ and $A^{3}$ in Section B.3.

## C.2 Block-wise truncation

A fairly good approximation to the closed forms previously explained results from truncating the Taylor series of each block of the transition matrix at the first significant term. That is, instead of truncating the series in full powers of A, as we have just made above, we regard each block individually. Therefore, truncation needs to be analyzed in a per-block basis. We explore it with two examples.

For the simplified IMU example of Section B.2, we had series $\Sigma_{1}$ and $\Sigma_{2}$ , which we can truncate as follows

$$
\Sigma_ {1} = \mathbf {I} \Delta t + \frac {1}{2} \Theta_ {\theta} \Delta t ^ {2} + \dots \approx \mathbf {I} \Delta t\tag{396}
$$

$$
\Sigma_ {2} = \frac {1}{2} \mathbf {I} \Delta t ^ {2} + \frac {1}{3 !} \Theta_ {\theta} \Delta t ^ {3} + \dots \approx \frac {1}{2} \mathbf {I} \Delta t ^ {2}.\tag{397}
$$

This leads to the approximate transition matrix

$$
\Phi \approx \left[ \begin{array}{c c c} \mathbf {I} & \mathbf {I} \Delta t & - \frac {1}{2} \mathbf {R} [ \mathbf {a} ] _ {\times} \Delta t ^ {2} \\ 0 & \mathbf {I} & - \mathbf {R} [ \mathbf {a} ] _ {\times} \Delta t \\ 0 & 0 & \mathbf {R} (\pmb {\omega} \Delta t) ^ {\top} \end{array} \right],\tag{398}
$$

which is more accurate than the one in the system-wide first-order truncation above (because of the upper-right term which has now appeared), yet it remains easy to obtain and compute, especially when compared to the closed forms developed in Section B. Again, observe that we have taken the closed-form for the lowest term, i.e., $\Phi_{\theta\theta} = \mathbf{R}(\omega\Delta t)^{\top}$ .

In the general case, it suffices to approximate each $\Sigma_{n}$ except $\Sigma_{0}$ by the first term of its series, i.e.,

$$
\boxed {\Sigma_ {0} = \mathbf {R} \{\boldsymbol {\omega} \Delta t \} ^ {\top}, \qquad \Sigma_ {n > 0} \approx \frac {1}{n !} \mathbf {I} \Delta t ^ {n}}.\tag{399}
$$

For the full IMU example, feeding the previous $\Sigma_{n}$ into (381) yields the approximated transition matrix,

$$
\Phi \approx \left[ \begin{array}{c c c c c c} \mathbf {I} & \mathbf {I} \Delta t & - \frac {1}{2} \mathbf {R} [ \mathbf {a} ] _ {\times} \Delta t ^ {2} & - \frac {1}{2} \mathbf {R} \Delta t ^ {2} & \frac {1}{3 !} \mathbf {R} [ \mathbf {a} ] _ {\times} \Delta t ^ {3} & \frac {1}{2} \mathbf {I} \Delta t ^ {2} \\ 0 & \mathbf {I} & - \mathbf {R} [ \mathbf {a} ] _ {\times} \Delta t & - \mathbf {R} \Delta t & \frac {1}{2} \mathbf {R} [ \mathbf {a} ] _ {\times} \Delta t ^ {2} & \mathbf {I} \Delta t \\ 0 & 0 & \mathbf {R} \{\boldsymbol {\omega} \Delta t \} ^ {\top} & 0 & - \mathbf {I} \Delta t & 0 \\ 0 & 0 & 0 & \mathbf {I} & 0 & 0 \\ 0 & 0 & 0 & 0 & \mathbf {I} & 0 \\ 0 & 0 & 0 & 0 & 0 & \mathbf {I} \end{array} \right]\tag{400}
$$

with (see (238))

$$
\mathbf {a} = \mathbf {a} _ {m} - \mathbf {a} _ {b}, \quad \boldsymbol {\omega} = \boldsymbol {\omega} _ {m} - \boldsymbol {\omega} _ {b}, \quad \mathbf {R} = \mathbf {R} \{\mathbf {q} \},
$$

and where we have substituted the matrix blocks by their appropriate values (see also (238)),

$$
\mathbf {P} _ {\mathbf {v}} = \mathbf {I}, \quad \mathbf {V} _ {\boldsymbol {\theta}} = - \mathbf {R} [ \mathbf {a} ] _ {\times}, \quad \mathbf {V} _ {\mathbf {a}} = - \mathbf {R}, \quad \mathbf {V} _ {\mathbf {g}} = \mathbf {I}, \quad \Theta_ {\boldsymbol {\theta}} = - [ \boldsymbol {\omega} ] _ {\times}, \quad \Theta_ {\boldsymbol {\omega}} = - \mathbf {I}
$$

A slight simplification of this method is to limit each block in the matrix to a certain maximum order n. For n = 1 we have,

$$
\Phi \approx \left[ \begin{array}{c c c c c c} \mathbf {I} & \mathbf {I} \Delta t & 0 & 0 & 0 & 0 \\ 0 & \mathbf {I} & - \mathbf {R} [ \mathbf {a} ] _ {\times} \Delta t & - \mathbf {R} \Delta t & 0 & \mathbf {I} \Delta t \\ 0 & 0 & \mathbf {R} \{\pmb {\omega} \Delta t \} ^ {\top} & 0 & - \mathbf {I} \Delta t & 0 \\ 0 & 0 & 0 & \mathbf {I} & 0 & 0 \\ 0 & 0 & 0 & 0 & \mathbf {I} & 0 \\ 0 & 0 & 0 & 0 & 0 & \mathbf {I} \end{array} \right],\tag{401}
$$

which is the Euler method, whereas for $n = 2$ ,

$$
\Phi \approx \left[ \begin{array}{c c c c c c} \mathbf {I} & \mathbf {I} \Delta t & - \frac {1}{2} \mathbf {R} [ \mathbf {a} ] _ {\times} \Delta t ^ {2} & - \frac {1}{2} \mathbf {R} \Delta t ^ {2} & 0 & \frac {1}{2} \mathbf {I} \Delta t ^ {2} \\ 0 & \mathbf {I} & - \mathbf {R} [ \mathbf {a} ] _ {\times} \Delta t & - \mathbf {R} \Delta t & \frac {1}{2} \mathbf {R} [ \mathbf {a} ] _ {\times} \Delta t ^ {2} & \mathbf {I} \Delta t \\ 0 & 0 & \mathbf {R} \{\pmb {\omega} \Delta t \} ^ {\top} & 0 & - \mathbf {I} \Delta t & 0 \\ 0 & 0 & 0 & \mathbf {I} & 0 & 0 \\ 0 & 0 & 0 & 0 & \mathbf {I} & 0 \\ 0 & 0 & 0 & 0 & 0 & \mathbf {I} \end{array} \right]\tag{402}
$$

For $n \geq 3$ we have the full form (400).

## D The transition matrix via Runge-Kutta integration

Still another way to approximate the transition matrix is to use Runge-Kutta integration. This might be necessary in cases where the dynamic matrix A cannot be considered constant along the integration interval, i.e.,

$$
\dot {\mathbf {x}} (t) = \mathbf {A} (t) \mathbf {x} (t).\tag{403}
$$

Let us rewrite the following two relations defining the same system in continuous- and discrete-time. They involve the dynamic matrix A and the transition matrix $\Phi$ ,

$$
\dot {\mathbf {x}} (t) = \mathbf {A} (t) \cdot \mathbf {x} (t)\tag{404}
$$

$$
\mathbf {x} (t _ {n} + \tau) = \Phi (t _ {n} + \tau | t _ {n}) \cdot \mathbf {x} (t _ {n}).\tag{405}
$$

These equations allow us to develop $\dot{\mathbf{x}}(t_{n}+\tau)$ in two ways as follows (left and right developments, please note the tiny dots indicating the time-derivatives),

$$
\begin{array}{r l r} & & {(\Phi (t _ {n} + \dot {\tau} | t _ {n}) \mathbf {x} (t _ {n})) = \boxed {\dot {\mathbf {x}} (t _ {n} + \tau)} = \mathbf {A} (t _ {n} + \tau) \mathbf {x} (t _ {n} + \tau)} \\ & & {\dot {\Phi} (t _ {n} + \tau | t _ {n}) \mathbf {x} (t _ {n}) + \Phi (t _ {n} + \tau | t _ {n}) \dot {\mathbf {x}} (t _ {n}) =} \\ & & {\dot {\Phi} (t _ {n} + \tau | t _ {n}) \mathbf {x} (t _ {n}) =} \end{array}\tag{406}
$$

Here, (406) comes from noticing that $\dot{\mathbf{x}}(t_{n}) = \dot{\mathbf{x}}_{n} = 0$ , because it is a sampled value. Then,

$$
\dot {\Phi} (t _ {n} + \tau | t _ {n}) = \mathbf {A} (t _ {n} + \tau) \Phi (t _ {n} + \tau | t _ {n})\tag{407}
$$

which is the same ODE as (404), now applied to the transition matrix instead of the state vector. Mind that, because of the identity $\mathbf{x}(t_{n}) = \Phi_{t_{n}|t_{n}}\mathbf{x}(t_{n})$ , the transition matrix at the beginning of the interval, $t = t_{n}$ , is always the identity,

$$
\Phi_ {t _ {n} | t _ {n}} = \mathbf {I}.\tag{408}
$$

Using RK4 with $f(t, \Phi(t)) = \mathbf{A}(t) \Phi(t)$ , we have

$$
\Phi \triangleq \Phi (t _ {n} + \Delta t | t _ {n}) = \mathbf {I} + \frac {\Delta t}{6} (\mathbf {K} _ {1} + 2 \mathbf {K} _ {2} + 2 \mathbf {K} _ {3} + \mathbf {K} _ {4})\tag{409}
$$

with

$$
\mathbf {K} _ {1} = \mathbf {A} (t _ {n})\tag{410}
$$

$$
\mathbf {K} _ {2} = \mathbf {A} \left(t _ {n} + \frac {1}{2} \Delta t\right) \left(\mathbf {I} + \frac {\Delta t}{2} \mathbf {K} _ {1}\right)\tag{411}
$$

$$
\mathbf {K} _ {3} = \mathbf {A} \left(t _ {n} + \frac {1}{2} \Delta t\right) \left(\mathbf {I} + \frac {\Delta t}{2} \mathbf {K} _ {2}\right)\tag{412}
$$

$$
\mathbf {K} _ {4} = \mathbf {A} \left(t _ {n} + \Delta t\right) \left(\mathbf {I} + \Delta t \cdot \mathbf {K} _ {3}\right).\tag{413}
$$

## D.1 Error-state example

Let us consider the error-state Kalman filter for the non-linear, time-varying system

$$
\dot {\mathbf {x}} _ {t} (t) = f (t, \mathbf {x} _ {t} (t), \mathbf {u} (t))\tag{414}
$$

where $x_{t}$ denotes the true state, and u is a control input. This true state is a composition, denoted by $\oplus$ , of a nominal state x and the error state $\delta x$ ,

$$
\mathbf {x} _ {t} (t) = \mathbf {x} (t) \oplus \delta \mathbf {x} (t)\tag{415}
$$

where the error-state dynamics admits a linear form which is time-varying depending on the nominal state x and the control u, i.e.,

$$
\dot {\delta \mathbf {x}} = \mathbf {A} (\mathbf {x} (t), \mathbf {u} (t)) \cdot \delta \mathbf {x}\tag{416}
$$

that is, the error-state dynamic matrix in (403) has the form $\mathbf{A}(t)=\mathbf{A}(\mathbf{x}(t),\mathbf{u}(t))$ . The dynamics of the error-state transition matrix can be written,

$$
\dot {\Phi} (t _ {n} + \tau | t _ {n}) = \mathbf {A} (\mathbf {x} (t), \mathbf {u} (t)) \cdot \Phi (t _ {n} + \tau | t _ {n}).\tag{417}
$$

In order to RK-integrate this equation, we need the values of $\mathbf{x}(t)$ and $\mathbf{u}(t)$ at the RK evaluation points, which for RK4 are $\{t_{n}, t_{n} + \Delta t/2, t_{n} + \Delta t\}$ . Starting by the easy ones, the control inputs $\mathbf{u}(t)$ at the evaluation points can be obtained by linear interpolation of the current and last measurements,

$$
\mathbf {u} (t _ {n}) = \mathbf {u} _ {n}\tag{418}
$$

$$
\mathbf {u} (t _ {n} + \Delta t / 2) = \frac {\mathbf {u} _ {n} + \mathbf {u} _ {n + 1}}{2}\tag{419}
$$

$$
\mathbf {u} (t _ {n} + \Delta t) = \mathbf {u} _ {n + 1}\tag{420}
$$

The nominal state dynamics should be integrated using the best integration practicable. For example, using RK4 integration we have,

$$
\begin{array}{l} \mathbf {k} _ {1} = f (\mathbf {x} _ {n}, \mathbf {u} _ {n}) \\ \mathbf {k} _ {2} = f (\mathbf {x} _ {n} + \frac {\Delta t}{2} \mathbf {k} _ {1}, \frac {\mathbf {u} _ {n} + \mathbf {u} _ {n + 1}}{2}) \\ \mathbf {k} _ {3} = f (\mathbf {x} _ {n} + \frac {\Delta t}{2} \mathbf {k} _ {2}, \frac {\mathbf {u} _ {n} + \mathbf {u} _ {n + 1}}{2}) \\ \mathbf {k} _ {4} = f (\mathbf {x} _ {n} + \Delta t \mathbf {k} _ {3}, \mathbf {u} _ {n + 1}) \\ \mathbf {k} = (\mathbf {k} _ {1} + 2 \mathbf {k} _ {2} + 2 \mathbf {k} _ {3} + \mathbf {k} _ {4}) / 6, \end{array}
$$

which gives us the estimates at the evaluation points,

$$
\mathbf {x} (t _ {n}) = \mathbf {x} _ {n}\tag{421}
$$

$$
\mathbf {x} (t _ {n} + \Delta t / 2) = \mathbf {x} _ {n} + \frac {\Delta t}{2} \mathbf {k}\tag{422}
$$

$$
\mathbf {x} (t _ {n} + \Delta t) = \mathbf {x} _ {n} + \Delta t \mathbf {k}.\tag{423}
$$

We notice here that $\mathbf{x}(t_{n}+\Delta t/2)=\frac{\mathbf{x}_{n}+\mathbf{x}_{n+1}}{2}$ , the same linear interpolation we used for the control. This should not be surprising given the linear nature of the RK update.

Whichever the way we obtained the nominal state values, we can now compute the RK4 matrices for the integration of the transition matrix,

$$
\begin{array}{l} \mathbf {K} _ {1} = \mathbf {A} (\mathbf {x} _ {n}, \mathbf {u} _ {n}) \\ \mathbf {K} _ {2} = \mathbf {A} \left(\mathbf {x} _ {n} + \frac {\Delta t}{2} \mathbf {k}, \frac {\mathbf {u} _ {n} + \mathbf {u} _ {n + 1}}{2}\right) \left(\mathbf {I} + \frac {\Delta t}{2} \mathbf {K} _ {1}\right) \\ \mathbf {K} _ {3} = \mathbf {A} \left(\mathbf {x} _ {n} + \frac {\Delta t}{2} \mathbf {k}, \frac {\mathbf {u} _ {n} + \mathbf {u} _ {n + 1}}{2}\right) \left(\mathbf {I} + \frac {\Delta t}{2} \mathbf {K} _ {2}\right) \\ \mathbf {K} _ {4} = \mathbf {A} \left(\mathbf {x} _ {n} + \Delta t \mathbf {k}, \mathbf {u} _ {n + 1}\right) \left(\mathbf {I} + \Delta t \mathbf {K} _ {3}\right) \\ \mathbf {K} = (\mathbf {K} _ {1} + 2 \mathbf {K} _ {2} + 2 \mathbf {K} _ {3} + \mathbf {K} _ {4}) / 6 \end{array}
$$

which finally lead to,

$$
\Phi \triangleq \Phi_ {t _ {n} + \Delta t | t _ {n}} = \mathbf {I} + \Delta t \mathbf {K}\tag{424}
$$

## E Integration of random noise and perturbations

We aim now at giving appropriate methods for the integration of random variables within dynamic systems. Of course, we cannot integrate unknown random values, but we can integrate their variances and covariances for the sake of uncertainty propagation. This is needed in order to establish the covariances matrices in estimators for systems that are of continuous nature (and specified in continuous time) but estimated in a discrete manner.

Consider the continuous-time dynamic system,

$$
\dot {\mathbf {x}} = f (\mathbf {x}, \mathbf {u}, \mathbf {w}),\tag{425}
$$

where x is the state vector, u is a vector of control signals containing noise $\tilde{u}$ , so that the control measurements are $u_{m} = u + \tilde{u}$ , and w is a vector of random perturbations. Both noise and perturbations are assumed white Gaussian processes, specified by,

$$
\tilde {\mathbf {u}} \sim \mathcal {N} \{0, \mathbf {U} ^ {c} \}, \quad \mathbf {w} ^ {c} \sim \mathcal {N} \{0, \mathbf {W} ^ {c} \},\tag{426}
$$

where the super-index $\bullet^{c}$ indicates a continuous-time uncertainty specification, which we want to integrate.

There exists an important difference between the natures of the noise levels in the control signals, $\tilde{u}$ , and the random perturbations, w:

\- On discretization, the control signals are sampled at the time instants $n\Delta t$ , having $\mathbf{u}_{m,n} \triangleq \mathbf{u}_m(n\Delta t) = \mathbf{u}(n\Delta t) + \tilde{\mathbf{u}}(n\Delta t)$ . The measured part is obviously considered constant over the integration interval, i.e., $\mathbf{u}_m(t) = \mathbf{u}_{m,n}$ , and therefore the noise level at the sampling time $n\Delta t$ is also held constant,

$$
\tilde {\mathbf {u}} (t) = \tilde {\mathbf {u}} (n \Delta t) = \tilde {\mathbf {u}} _ {n}, \quad n \Delta t <   t <   (n + 1) \Delta t.\tag{427}
$$

\- The perturbations $\mathbf{w}$ are never sampled.

As a consequence, the integration over $\Delta t$ of these two stochastic processes differs. Let us examine it.

The continuous-time error-state dynamics (425) can be linearized to

$$
\dot {\delta \mathbf {x}} = \mathbf {A} \delta \mathbf {x} + \mathbf {B} \tilde {\mathbf {u}} + \mathbf {C} \mathbf {w},\tag{428}
$$

with

$$
\mathbf {A} \triangleq \left. \frac {\partial f}{\partial \delta \mathbf {x}} \right| _ {\mathbf {x}, \mathbf {u} _ {m}}, \quad \mathbf {B} \triangleq \left. \frac {\partial f}{\partial \tilde {\mathbf {u}}} \right| _ {\mathbf {x}, \mathbf {u} _ {m}}, \quad \mathbf {C} \triangleq \left. \frac {\partial f}{\partial \mathbf {w}} \right| _ {\mathbf {x}, \mathbf {u} _ {m}},\tag{429}
$$

and integrated over the sampling period $\Delta t$ , giving,

$$
\begin{array}{r l} & {\delta \mathbf {x} _ {n + 1} = \delta \mathbf {x} _ {n} + \int_ {n \Delta t} ^ {(n + 1) \Delta t} \left(\mathbf {A} \delta \mathbf {x} (\tau) + \mathbf {B} \tilde {\mathbf {u}} (\tau) + \mathbf {C} \mathbf {w} ^ {c} (\tau)\right) d \tau} \\ & {\qquad = \delta \mathbf {x} _ {n} + \int_ {n \Delta t} ^ {(n + 1) \Delta t} \mathbf {A} \delta \mathbf {x} (\tau) d \tau + \int_ {n \Delta t} ^ {(n + 1) \Delta t} \mathbf {B} \tilde {\mathbf {u}} (\tau) d \tau + \int_ {n \Delta t} ^ {(n + 1) \Delta t} \mathbf {C} \mathbf {w} ^ {c} (\tau) d \tau} \end{array}\tag{430}
$$

(431)

which has three terms of very different nature. They can be integrated as follows:

1. From App. B we know that the dynamic part is integrated giving the transition matrix,

$$
\delta \mathbf {x} _ {n} + \int_ {n \Delta t} ^ {(n + 1) \Delta t} \mathbf {A} \delta \mathbf {x} (\tau) d \tau = \Phi \cdot \delta \mathbf {x} _ {n}\tag{432}
$$

where $\Phi = e^{A\Delta t}$ can be computed in closed-form or approximated at different levels of accuracy.

2. From (427) we have

$$
\int_ {n \Delta t} ^ {(n + 1) \Delta t} \mathbf {B} \tilde {\mathbf {u}} (\tau) d \tau = \mathbf {B} \Delta t \tilde {\mathbf {u}} _ {n}\tag{433}
$$

which means that the measurement noise, once sampled, is integrated in a deterministic manner because its behavior inside the integration interval is known.

3. From Probability Theory we know that the integration of continuous white Gaussian noise over a period $\Delta t$ produces a discrete white Gaussian impulse $w_{n}$ described by

$$
\mathbf {w} _ {n} \triangleq \int_ {n \Delta t} ^ {(n + 1) \Delta t} \mathbf {w} (\tau) d \tau , \quad \mathbf {w} _ {n} \sim \mathcal {N} \{0, \mathbf {W} \}, \quad \mathrm{with} \mathbf {W} = \mathbf {W} ^ {c} \Delta t\tag{434}
$$

We observe that, contrary to the measurement noise just above, the perturbation does not have a deterministic behavior inside the integration interval, and hence it must be integrated stochastically.

Table 5: Effect of integration on system and covariances matrices.

<table><tr><td>Description</td><td>Continuous time t</td><td>Discrete time nΔt</td></tr><tr><td>state</td><td> $\dot{\mathbf{x}} = f^{c}(\mathbf{x}, \mathbf{u}, \mathbf{w})$ </td><td> $\mathbf{x}_{n+1} = f(\mathbf{x}_{n}, \mathbf{u}_{n}, \mathbf{w}_{n})$ </td></tr><tr><td>error-state</td><td> $\delta\dot{\mathbf{x}} = \mathbf{A}\delta\mathbf{x} + \mathbf{B}\tilde{\mathbf{u}} + \mathbf{C}\mathbf{w}$ </td><td> $\delta\mathbf{x}_{n+1} = \mathbf{F}_{\mathbf{x}}\delta\mathbf{x}_{n} + \mathbf{F}_{\mathbf{u}}\tilde{\mathbf{u}}_{n} + \mathbf{F}_{\mathbf{w}}\mathbf{w}_{n}$ </td></tr><tr><td>system matrix</td><td> $\mathbf{A}$ </td><td> $\mathbf{F}_{\mathbf{x}} = \Phi = e^{\mathbf{A}\Delta t}$ </td></tr><tr><td>control matrix</td><td> $\mathbf{B}$ </td><td> $\mathbf{F}_{\mathbf{u}} = \mathbf{B}\Delta t$ </td></tr><tr><td>perturbation matrix</td><td> $\mathbf{C}$ </td><td> $\mathbf{F}_{\mathbf{w}} = \mathbf{C}$ </td></tr><tr><td>control covariance</td><td> $\mathbf{U}^{c}$ </td><td> $\mathbf{U} = \mathbf{U}^{c}$ </td></tr><tr><td>perturbation covariance</td><td> $\mathbf{W}^{c}$ </td><td> $\mathbf{W} = \mathbf{W}^{c}\Delta t$ </td></tr></table>

Therefore, the discrete-time, error-state dynamic system can be written as

$$
\delta \mathbf {x} _ {n + 1} = \mathbf {F} _ {\mathbf {x}} \delta \mathbf {x} _ {n} + \mathbf {F} _ {\mathbf {u}} \tilde {\mathbf {u}} _ {n} + \mathbf {F} _ {\mathbf {w}} \mathbf {w} _ {n}\tag{435}
$$

with transition, control and perturbation matrices given by

$$
\mathbf {F} _ {\mathbf {x}} = \Phi = e ^ {\mathbf {A} \Delta t}, \quad \mathbf {F} _ {\mathbf {u}} = \mathbf {B} \Delta t, \quad \mathbf {F} _ {\mathbf {w}} = \mathbf {C},\tag{436}
$$

with noise and perturbation levels defined by

$$
\tilde {\mathbf {u}} _ {n} \sim \mathcal {N} \{0, \mathbf {U} \}, \quad \mathbf {w} _ {n} \sim \mathcal {N} \{0, \mathbf {W} \}\tag{437}
$$

with

$$
\mathbf {U} = \mathbf {U} ^ {c}, \quad \mathbf {W} = \mathbf {W} ^ {c} \Delta t.\tag{438}
$$

These results are summarized in Table 5. The prediction stage of an EKF would propagate the error state's mean and covariances matrix according to

$$
\begin{array}{r l} & {\hat {\delta \mathbf {x}} _ {n + 1} = \mathbf {F} _ {\mathbf {x}} \hat {\delta \mathbf {x}} _ {n}} \\ & {\mathbf {P} _ {n + 1} = \mathbf {F} _ {\mathbf {x}} \mathbf {P} _ {n} \mathbf {F} _ {\mathbf {x}} ^ {\top} + \mathbf {F} _ {\mathbf {u}} \mathbf {U} \mathbf {F} _ {\mathbf {u}} ^ {\top} + \mathbf {F} _ {\mathbf {w}} \mathbf {W} \mathbf {F} _ {\mathbf {w}} ^ {\top}} \\ & {\quad = e ^ {\mathbf {A} \Delta t} \mathbf {P} _ {n} (e ^ {\mathbf {A} \Delta t}) ^ {\top} + \Delta t ^ {2} \mathbf {B} \mathbf {U} ^ {c} \mathbf {B} ^ {\top} + \Delta t \mathbf {C} \mathbf {W} ^ {c} \mathbf {C} ^ {\top}} \end{array}\tag{439}
$$

(440)

It is important and illustrative here to observe the different effects of the integration interval, $\Delta t$ , on the three terms of the covariance update (440): the dynamic error term is exponential, the measurement error term is quadratic, and the perturbation error term is linear.

## E.1 Noise and perturbation impulses

One is oftentimes confronted (for example when reusing existing code or when interpreting other authors' documents) with EKF prediction equations of a simpler form than those that we used here, namely,

$$
\mathbf {P} _ {n + 1} = \mathbf {F} _ {\mathbf {x}} \mathbf {P} _ {n} \mathbf {F} _ {\mathbf {x}} ^ {\top} + \mathbf {Q}.\tag{441}
$$

This corresponds to the general discrete-time dynamic system,

$$
\delta \mathbf {x} _ {n + 1} = \mathbf {F} _ {\mathbf {x}} \delta \mathbf {x} _ {n} + \mathbf {i}\tag{442}
$$

where

$$
\mathbf {i} \sim \mathcal {N} \{0, \mathbf {Q} \}\tag{443}
$$

is a vector of random (white, Gaussian) impulses that are directly added to the state vector at time $t_{n+1}$ . The matrix Q is simply considered the impulses covariances matrix. From what we have seen, we should compute this covariances matrix as follows,

$$
\mathbf {Q} = \Delta t ^ {2} \mathbf {B} \mathbf {U} ^ {c} \mathbf {B} ^ {\top} + \Delta t \mathbf {C} \mathbf {W} ^ {c} \mathbf {C} ^ {\top}.\tag{444}
$$

In the case where the impulses do not affect the full state, as it is often the case, the matrix Q is not full-diagonal and may contain a significant amount of zeros. One can then write the equivalent form

$$
\delta \mathbf {x} _ {n + 1} = \mathbf {F} _ {\mathbf {x}} \delta \mathbf {x} _ {n} + \mathbf {F} _ {\mathbf {i}} \mathbf {i}\tag{445}
$$

with

$$
\mathbf {i} \sim \mathcal {N} \{0, \mathbf {Q _ {i}} \},\tag{446}
$$

where the matrix $F_{i}$ simply maps each individual impulse to the part of the state vector it affects to. The associated covariance $Q_{i}$ is then smaller and full-diagonal. Please refer to the next section for an example. In such case the ESKF time-update becomes

$$
\hat {\delta \mathbf {x}} _ {n + 1} = \mathbf {F} _ {\mathbf {x}} \hat {\delta \mathbf {x}} _ {n}\tag{447}
$$

$$
\mathbf {P} _ {n + 1} = \mathbf {F} _ {\mathbf {x}} \mathbf {P} _ {n} \mathbf {F} _ {\mathbf {x}} ^ {\top} + \mathbf {F} _ {\mathbf {i}} \mathbf {Q} _ {\mathbf {i}} \mathbf {F} _ {\mathbf {i}} ^ {\top}.\tag{448}
$$

Obviously, all these forms are equivalent, as it can be seen in the following double identity for the general perturbation Q,

$$
\mathbf {F} _ {\mathbf {i}} \mathbf {Q} _ {\mathbf {i}} \mathbf {F} _ {\mathbf {i}} ^ {\top} = \boxed {\mathbf {Q}} = \Delta t ^ {2} \mathbf {B} \mathbf {U} ^ {c} \mathbf {B} ^ {\top} + \Delta t \mathbf {C} \mathbf {W} ^ {c} \mathbf {C} ^ {\top}.\tag{449}
$$

## E.2 Full IMU example

We study the construction of an error-state Kalman filter for an IMU. The error-state system is defined in (238) and involves a nominal state x, an error-state $\delta x$ , a noisy control signal $u_{m} = u + \tilde{u}$ and a perturbation w, specified by,

$$
\mathbf {x} = \left[ \begin{array}{l} \mathbf {p} \\ \mathbf {v} \\ \mathbf {q} \\ \mathbf {a} _ {b} \\ \boldsymbol {\omega} _ {b} \\ \mathbf {g} \end{array} \right], \quad \delta \mathbf {x} = \left[ \begin{array}{l} \delta \mathbf {p} \\ \delta \mathbf {v} \\ \delta \boldsymbol {\theta} \\ \delta \mathbf {a} _ {b} \\ \delta \boldsymbol {\omega} _ {b} \\ \delta \mathbf {g} \end{array} \right], \quad \mathbf {u} _ {m} = \left[ \begin{array}{l} \mathbf {a} _ {m} \\ \boldsymbol {\omega} _ {m} \end{array} \right], \quad \tilde {\mathbf {u}} = \left[ \begin{array}{l} \tilde {\mathbf {a}} \\ \tilde {\boldsymbol {\omega}} \end{array} \right], \quad \mathbf {w} = \left[ \begin{array}{l} \mathbf {a} _ {w} \\ \boldsymbol {\omega} _ {w} \end{array} \right]\tag{450}
$$

In a model of an IMU like the one we are considering throughout this document, the control noise corresponds to the additive noise in the IMU measurements. The perturbations affect the biases, thus producing their random-walk behavior. The dynamic, control and perturbation matrices are (see (428), (378) and (238)),

$$
\mathbf {A} = \left[ \begin{array}{c c c c c c} 0 & \mathbf {P _ {v}} & 0 & 0 & 0 & 0 \\ 0 & 0 & \mathbf {V} _ {\theta} & \mathbf {V _ {a}} & 0 & \mathbf {V _ {g}} \\ 0 & 0 & \Theta_ {\theta} & 0 & \Theta_ {\omega} & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \\ 0 & 0 & 0 & 0 & 0 & 0 \end{array} \right], \quad \mathbf {B} = \left[ \begin{array}{c c} 0 & 0 \\ - \mathbf {R} & 0 \\ 0 & - \mathbf {I} \\ 0 & 0 \\ 0 & 0 \\ 0 & 0 \end{array} \right], \quad \mathbf {C} = \left[ \begin{array}{c c} 0 & 0 \\ 0 & 0 \\ 0 & 0 \\ \mathbf {I} & 0 \\ 0 & \mathbf {I} \\ 0 & 0 \end{array} \right]\tag{451}
$$

In the regular case of IMUs with accelerometer and gyrometer triplets of the same kind on the three axes, noise and perturbations are isotropic. Their standard deviations are specified as scalars as follows

$$
\sigma_ {\tilde {\mathbf {a}}} \left[ m / s ^ {2} \right] \quad , \quad \sigma_ {\tilde {\boldsymbol {\omega}}} \left[ r a d / s \right] \quad , \quad \sigma_ {\mathbf {a} _ {w}} \left[ m / s ^ {2} \sqrt {s} \right] \quad , \quad \sigma_ {\boldsymbol {\omega} _ {w}} \left[ r a d / s \sqrt {s} \right]\tag{452}
$$

and their covariances matrices are purely diagonal, giving

$$
\mathbf {U} ^ {c} = \left[ \begin{array}{c c} \sigma_ {\tilde {\mathbf {a}}} ^ {2} \mathbf {I} & 0 \\ 0 & \sigma_ {\tilde {\boldsymbol {\omega}}} ^ {2} \mathbf {I} \end{array} \right] \qquad , \qquad \mathbf {W} ^ {c} = \left[ \begin{array}{c c} \sigma_ {\mathbf {a} _ {w}} ^ {2} \mathbf {I} & 0 \\ 0 & \sigma_ {\boldsymbol {\omega} _ {w}} ^ {2} \mathbf {I} \end{array} \right].\tag{453}
$$

The system evolves with sampled measures at intervals $\Delta t$ , following (435–438), where the transition matrix $F_{x} = \Phi$ can be computed in a number of ways – see previous appendices.

## E.2.1 Noise and perturbation impulses

In the case of a perturbation specification in the form of impulses i, we can re-define our system as follows,

$$
\delta \mathbf {x} _ {n + 1} = \mathbf {F} _ {\mathbf {x}} (\mathbf {x} _ {n}, \mathbf {u} _ {m}) \cdot \delta \mathbf {x} _ {n} + \mathbf {F} _ {\mathbf {i}} \cdot \mathbf {i}\tag{454}
$$

with the nominal-state, error-state, control, and impulses vectors defined by,

$$
\mathbf {x} = \left[ \begin{array}{l} \mathbf {p} \\ \mathbf {v} \\ \mathbf {q} \\ \mathbf {a} _ {b} \\ \boldsymbol {\omega} _ {b} \\ \mathbf {g} \end{array} \right], \quad \delta \mathbf {x} = \left[ \begin{array}{l} \delta \mathbf {p} \\ \delta \mathbf {v} \\ \delta \boldsymbol {\theta} \\ \delta \mathbf {a} _ {b} \\ \delta \boldsymbol {\omega} _ {b} \\ \delta \mathbf {g} \end{array} \right], \quad \mathbf {u} _ {m} = \left[ \begin{array}{l} \mathbf {a} _ {m} \\ \boldsymbol {\omega} _ {m} \end{array} \right], \quad \mathbf {i} = \left[ \begin{array}{l} \mathbf {v} _ {\mathrm{i}} \\ \boldsymbol {\theta} _ {\mathrm{i}} \\ \mathbf {a} _ {\mathrm{i}} \\ \boldsymbol {\omega} _ {\mathrm{i}} \end{array} \right],\tag{455}
$$

the transition and perturbations matrices defined by,

$$
\mathbf {F} _ {\mathbf {x}} = \boldsymbol {\Phi} = e ^ {\mathbf {A} \Delta t} \qquad , \qquad \mathbf {F} _ {\mathbf {i}} = \left[ \begin{array}{l l l l} 0 & 0 & 0 & 0 \\ \mathbf {I} & 0 & 0 & 0 \\ 0 & \mathbf {I} & 0 & 0 \\ 0 & 0 & \mathbf {I} & 0 \\ 0 & 0 & 0 & \mathbf {I} \\ 0 & 0 & 0 & 0 \end{array} \right] \quad ,\tag{456}
$$

and the impulses variances specified by

$$
\mathbf {i} \sim \mathcal {N} \{0, \mathbf {Q _ {i}} \} \quad , \quad \mathbf {Q _ {i}} = \left[ \begin{array}{c c c} \sigma_ {\tilde {\mathbf {a}}} ^ {2} \Delta t ^ {2} \mathbf {I} & 0 & \\ & \sigma_ {\tilde {\omega}} ^ {2} \Delta t ^ {2} \mathbf {I} & \\ & & \sigma_ {\mathbf {a} _ {w}} ^ {2} \Delta t \mathbf {I} \\ 0 & & \sigma_ {\boldsymbol {\omega} _ {w}} ^ {2} \Delta t \mathbf {I} \end{array} \right] .\tag{457}
$$

The trivial specification of $F_{i}$ may appear surprising given especially that of B in (451). What happens is that the errors are defined isotropic in $Q_{i}$ , and therefore $-\mathbf{R}\sigma^{2}\mathbf{I}(-\mathbf{R})^{\top} = \sigma^{2}\mathbf{I}$ and $-\mathbf{I}\sigma^{2}\mathbf{I}(-\mathbf{I})^{\top} = \sigma^{2}\mathbf{I}$ , leading to the expression given for $F_{i}$ . This is not possible when considering non-isotropic IMUs, where a proper Jacobian $F_{i} = [B C]$ should be used together with a proper specification of $Q_{i}$ .

We can of course use full-state perturbation impulses,

$$
\delta \mathbf {x} _ {n + 1} = \mathbf {F} _ {\mathbf {x}} (\mathbf {x} _ {n}, \mathbf {u} _ {m}) \cdot \delta \mathbf {x} _ {n} + \mathbf {i}\tag{458}
$$

with

$$
\mathbf {i} = \left[ \begin{array}{l} 0 \\ \mathbf {v _ {i}} \\ \boldsymbol {\theta_ {i}} \\ \mathbf {a _ {i}} \\ \omega_ {i} \\ 0 \end{array} \right] \quad , \quad \mathbf {i} \sim \mathcal {N} \{0, \mathbf {Q} \} \quad , \quad \mathbf {Q} = \left[ \begin{array}{c c c c} 0 & & & \\ & \sigma_ {\tilde {\mathbf {a}}} ^ {2} \Delta t ^ {2} \mathbf {I} & & 0 \\ & & \sigma_ {\tilde {\omega}} ^ {2} \Delta t ^ {2} \mathbf {I} & \\ & & & \sigma_ {\mathbf {a} _ {w}} ^ {2} \Delta t \mathbf {I} \\ 0 & & & \sigma_ {\omega_ {w}} ^ {2} \Delta t \mathbf {I} \\ & & & & 0 \end{array} \right]\tag{459}
$$

Bye bye.

## References

Chirikjian, G. S. (2012). Stochastic Models, Information Theory, and Lie Groups, volume 2: Analytic Methods and Modern Applications of Applied and Numerical Harmonic Analysis. Birkhäuser,, Basel.

Chou, J. (1992). Quaternion kinematic and dynamic differential equations. Robotics and Automation, IEEE Transactions on, 8(1):53–64.

Dam, E. B., Koch, M., and Lillholm, M. (1998). Quaternions, interpolation and animation. Technical report, Department of Computer Science, University of Copenhagen, Denmark.

Eberly, D. (2010). Quaternion algebra and calculus. Technical report, Geometric Tools, LLC.

Kuipers, J. B. (1999). Quaternions and rotation sequences: a primer with applications to orbits, aerospace, and virtual reality. Princeton University Press., Princeton, N.J.

Li, M. and Mourikis, A. (2012). Improving the accuracy of EKF-based visual-inertial odometry. In Robotics and Automation (ICRA), 2012 IEEE International Conference on, pages 828–835.

Li, M., Yu, H., Zheng, X., and Mourikis, A. (2014). High-fidelity sensor modeling and self-calibration in vision-aided inertial navigation. In Robotics and Automation (ICRA), 2014 IEEE International Conference on, pages 409–416.

Lupton, T. and Sukkarieh, S. (2009). Efficient integration of inertial observations into visual slam without initialization. In IEEE/RSJ Int. Conf. on Intelligent Robots and Systems.

Madyastha, V., Ravindra, V., Mallikarjunan, S., and Goyal, A. (2011). Extended Kalman Filter vs. Error State Kalman Filter for Aircraft Attitude Estimation. In AIAA Guidance, Navigation, and Control Conference, Portland, Oregon. American Institute of Aeronautics and Astronautics.

Martinelli, A. (2012). Vision and imu data fusion: Closed-form solutions for attitude, speed, absolute scale, and bias determination. IEEE Transactions on Robotics, 28(1):44–60.

Piniés, P., Lupton, T., Sukkarieh, S., and Tardós, J. D. (2007). Inertial aiding of inverse depth SLAM using a monocular camera. In Int. Conf. on Robotics and Automation.

Roussillon, C., Gonzalez, A., Solà, J., Codol, J. M., Mansard, N., Lacroix, S., and Devy, M. (2011). RT-SLAM: A generic and real-time visual SLAM implementation. In Crowley, J., Draper, B., and Thonnat, M., editors, Computer Vision Systems, volume 6962 of Lecture Notes in Computer Science, pages 31–40. Springer Berlin Heidelberg.

Shoemake, K. (1985). Animating rotation with quaternion curves. SIGGRAPH Comput. Graph., 19(3):245–254.

Shuster, M. D. (1993). Survey of attitude representations. Journal of the Astronautical Sciences, 41:439–517.

Trawny, N. and Roumeliotis, S. I. (2005). Indirect Kalman filter for 3D attitude estimation. Technical Report 2005-002, University of Minnesota, Dept. of Comp. Sci. & Eng.

Yazell, D. (2009). Origins of the unusual space shuttle quaternion definition. In Aerospace Sciences Meetings, pages –. American Institute of Aeronautics and Astronautics.