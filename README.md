# LQG Missile Autopilot – Control & Navigation Project

## Overview

This project presents the modeling, control, and state estimation of a missile **longitudinal dynamics** using modern control techniques implemented in **MATLAB** and **Simulink**.

The work combines:
- State-space modeling of missile dynamics
- Linear Quadratic Regulator (LQR) synthesis
- Linear Quadratic Gaussian (LQG) control via Kalman filtering
- Closed-loop stability analysis
- Kinematic propagation of missile position
- A guidance law based on angular geometry (atan) and No-Fly Zone (NFZ) constraints
- Integration within a simplified missile–environment scenario

The objective is to demonstrate a complete and coherent **control design workflow**, suitable for Guidance, Navigation, and Control (GNC) applications.



## 1. System Modeling

### 1.1 State-Space Representation

The missile longitudinal dynamics are modeled in continuous-time state-space form:


$\dot{x} = Ax + Bu $

$\ y = Cx + Du $

with the following definitions:

**States**
- Angle of Attack \( $AoA$ )
- Pitch rate \( $q$ )

**Input**
- Control surface deflection \( $\delta_c$ )

**Outputs**
- Normal acceleration ( $A_z$ )
- Pitch rate \( $q$ )

The system matrices are:

$$
A =
\begin{bmatrix}
-1.064 & 1 \\
290.26 & 0
\end{bmatrix},
\quad
B =
\begin{bmatrix}
-0.25 \\
-331.40
\end{bmatrix}
$$

$$
C =
\begin{bmatrix}
-123.34 & 0 \\
0 & 1
\end{bmatrix},
\quad
D =
\begin{bmatrix}
-13.51 \\
0
\end{bmatrix}
$$

This reduced-order model focuses on the **short-period longitudinal dynamics**, which dominate the pitch stability and control behavior of the missile and therefore represent the primary target of the autopilot design.



### 1.2 Open-Loop Analysis

The transfer function from the control input \( $\delta_c $) to the pitch rate \( $q$ ) is extracted from the state-space model.

Pole analysis reveals an **open-loop unstable behavior**, motivating the use of active feedback control.



## 2. LQR Control Design

### 2.1 Control Objective

The LQR controller is designed to:
- Stabilize the missile longitudinal dynamics
- Penalize excessive angle of attack and pitch rate
- Limit control effort

The quadratic cost function minimized in infinite time is:

$$
J = \int_{0}^{\infty} \left( x^T Q x + u^T R u \right) \ dt
$$

with weighting matrices:

$$
Q =
\begin{bmatrix}
1 & 0 \\
0 & 1
\end{bmatrix},
\quad
R = 1.4
$$



### 2.2 LQR Gain and Stability

The optimal state feedback control law is:

$$
u = -Kx
$$

The gain matrix \( $K$ ) is obtained by solving the Algebraic Riccati Equation :

$$
A^T S + S A - S B R^{-1} B^T S + Q = 0
$$

The closed-loop system matrix:

$$
A_{cl} = A - BK
$$

has eigenvalues strictly located in the left-half complex plane, confirming **closed-loop asymptotic stability**.



### 2.3 Closed-Loop Dynamics

The closed-loop transfer function from \( $\delta_c $) to \( $q$ ) is analyzed.

Compared to the open-loop system, the closed-loop response exhibits:
- Improved damping
- Faster settling time
- Robust stabilization of pitch dynamics



## 3. State Estimation – Kalman Filter

### 3.1 Motivation

In practical applications, not all states are directly measurable and sensor measurements are corrupted by noise.  
A **Kalman filter** is therefore designed to provide optimal state estimation.
A Kalman filter is used to estimate :

$$
\dot{\hat{x}} = 
\begin{bmatrix}
\hat{AoA} \\
\hat{q}
\end{bmatrix}
$$



### 3.2 Noise Modeling

The augmented system includes process and measurement noise with the following covariances:

**- Process noise covariance**

$$
W = 0.01 I
$$

**- Measurement noise covariance**

$$
V = 0.5 I
$$

This choice reflects moderate sensor noise and low model uncertainty.



### 3.3 Observer Dynamics

The Kalman filter provides the observer gain \( $L$ ).  
The observer dynamics are given by:

$$
\begin{aligned}
\dot{\hat{x}} &= A\hat{x} + Bu + L(y - C\hat{x}) \\
              &= (A - LC)\hat{x} + Bu + Ly
\end{aligned}
$$

The eigenvalues of \( $A - LC$ ) lie in the left-half complex plane, ensuring **fast and stable state estimation**.



## 4. LQG Control Architecture

The LQG controller combines:
- **LQR state feedback** for control
- **Kalman filter** for state estimation

Thanks to the separation principle, overall closed-loop stability is guaranteed as long as both the controller and the observer are individually stable.

This architecture is well-suited for real-world missile autopilot implementations.



## 5. Missile kinematics & Navigation model

### 5.1 Translational motion

The missile position is propagated in an Earth-fixed local frame:

$$
\begin{cases}
X = V_T \cos(\theta) \\
Z = V_T \sin(\theta) \\
\dot{\theta} = q
\end{cases}
$$

with $V_T$ the speed of the missile, assumed constant.
These equations are implemented in Simulink to reconstruct the missile trajectory (x,y,z) and then converted in geographic coordinates (Latitude, Longitude, Altitude) to generate guidance command. This conversion is applied using the **Aerospace toolbox**.



## 6. Guidance Law (Outer loop)

### 6.1 Line-of-Sight Geometry

The guidance loop computes the desired flight path angle based on relative geometry:

$\gamma_{cmd} = \arctan\left(\frac{z_{target} - z_{missile}}{R_{horizontal}}\right) $

This really simple atan-based guidance law ensures:
- Smooth angular commands
- Robust behavior near terminal phase
- Compatibility with autopilot inner-loop dynamics

### 6.2 No-Fly Zone (NFZ) Constraint





## 7. Implementation

The project is implemented using:
- **MATLAB** for analytical design and verification
- **Simulink** for block-diagram modeling and closed-loop simulation

The modular structure enables future extensions such as:
- Nonlinear aerodynamic modeling
- Actuator dynamics
- 3-DOF missile models



## 8. Conclusion

This project demonstrates a complete **GNC-oriented control design workflow**, from modeling to LQG synthesis and environmental integration.

Key outcomes include:
- Successful stabilization of an unstable missile model using LQR
- Robust state estimation via Kalman filtering
- Clear separation between control and estimation
- Realistic initialization using geodetic navigation principles

The project forms a solid foundation for advanced missile guidance and control developments and is well-suited for inclusion in a professional GitHub portfolio.

---

## Keywords

`LQR` · `LQG` · `Kalman Filter` · `Missile Autopilot` · `State-Space Control` · `MATLAB` · `Simulink` · `GNC`
