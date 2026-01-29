# LQG Missile Autopilot – Control & Navigation Project

## Overview

This project presents the modeling, control, and state estimation of a missile **longitudinal dynamics** using modern control techniques implemented in **MATLAB** and **Simulink**.

The work combines:
- State-space modeling of missile dynamics
- Linear Quadratic Regulator (LQR) synthesis
- Linear Quadratic Gaussian (LQG) control via Kalman filtering
- Closed-loop stability analysis
- Integration within a simplified missile–environment scenario

The objective is to demonstrate a complete and coherent **control design workflow**, suitable for Guidance, Navigation, and Control (GNC) applications.

---

## 1. System Modeling

### 1.1 State-Space Representation

The missile longitudinal dynamics are modeled in continuous-time state-space form:

\[
\dot{x} = Ax + Bu
\]
\[
y = Cx + Du
\]

with the following definitions:

**States**
- Angle of Attack (AoA)
- Pitch rate \( q \)

**Input**
- Control surface deflection \( \delta_c \)

**Outputs**
- Normal acceleration \( A_z \)
- Pitch rate \( q \)

The system matrices are:

\[
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
\]

\[
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
\]

This reduced-order model focuses on the **short-period longitudinal dynamics**, which dominate the pitch stability and control behavior of the missile and therefore represent the primary target of the autopilot design.

---

### 1.2 Open-Loop Analysis

The transfer function from the control input \( \delta_c \) to the pitch rate \( q \) is extracted from the state-space model.

Pole analysis reveals an **open-loop unstable behavior**, motivating the use of active feedback control.

---

## 2. LQR Control Design

### 2.1 Control Objective

The LQR controller is designed to:
- Stabilize the missile longitudinal dynamics
- Penalize excessive angle of attack and pitch rate
- Limit control effort

The quadratic cost function minimized is:

\[
J = \int_{0}^{\infty} \left( x^T Q x + u^T R u \right) \, dt
\]

with weighting matrices:

\[
Q =
\begin{bmatrix}
1 & 0 \\
0 & 1
\end{bmatrix},
\quad
R = 1.4
\]

---

### 2.2 LQR Gain and Stability

The optimal state feedback control law is:

\[
u = -Kx
\]

The gain matrix \( K \) is obtained by solving the Algebraic Riccati Equation.  
The closed-loop system matrix:

\[
A_{cl} = A - BK
\]

has eigenvalues strictly located in the left-half complex plane, confirming **closed-loop asymptotic stability**.

---

### 2.3 Closed-Loop Dynamics

The closed-loop transfer function from \( \delta_c \) to \( q \) is analyzed.

Compared to the open-loop system, the closed-loop response exhibits:
- Improved damping
- Faster settling time
- Robust stabilization of pitch dynamics

---

## 3. State Estimation – Kalman Filter

### 3.1 Motivation

In practical applications, not all states are directly measurable and sensor measurements are corrupted by noise.  
A **Kalman filter** is therefore designed to provide optimal state estimation.

---

### 3.2 Noise Modeling

The augmented system includes process and measurement noise with the following covariances:

**Process noise covariance**
\[
W = 0.01 I
\]

**Measurement noise covariance**
\[
V = 0.5 I
\]

This choice reflects moderate sensor noise and low model uncertainty.

---

### 3.3 Observer Dynamics

The Kalman filter provides the observer gain \( L \).  
The observer dynamics are given by:

\[
\dot{\hat{x}} = (A - LC)\hat{x} + Bu + Ly
\]

The eigenvalues of \( A - LC \) lie in the left-half complex plane, ensuring **fast and stable state estimation**.

---

## 4. LQG Control Architecture

The LQG controller combines:
- **LQR state feedback** for control
- **Kalman filter** for state estimation

Thanks to the separation principle, overall closed-loop stability is guaranteed as long as both the controller and the observer are individually stable.

This architecture is well-suited for real-world missile autopilot implementations.

---

## 5. Missile–Environment Scenario

### 5.1 Earth and Kinematic Parameters

A simplified engagement scenario is defined using:
- Earth radius: 6371 km
- Missile speed: 1021 m/s

---

### 5.2 Initial and Target Conditions

Geodetic coordinates (latitude, longitude, altitude) are used for:
- Initial missile position
- Target position
- Obstacle location

The **Haversine formula** is employed to compute the horizontal distance, which is combined with altitude difference to estimate the initial slant range.

---

### 5.3 Initial Guidance Geometry

- Initial azimuth angle is computed from geodetic coordinates
- Initial flight path angle (FPA) is derived from altitude difference and ground distance

These parameters provide realistic initial conditions for guidance and control simulations.

---

## 6. Implementation

The project is implemented using:
- **MATLAB** for analytical design and verification
- **Simulink** for block-diagram modeling and closed-loop simulation

The modular structure enables future extensions such as:
- Nonlinear aerodynamic modeling
- Actuator dynamics
- 3-DOF or 6-DOF missile models

---

## 7. Conclusion

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
