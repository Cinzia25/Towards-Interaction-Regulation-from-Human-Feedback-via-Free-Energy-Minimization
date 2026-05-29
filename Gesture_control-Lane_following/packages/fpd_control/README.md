# fpd_control

This package presents a probabilistic control framework for lane following in Duckietown, integrating:

- human commands (gesture-based commands)
- model-based prediction (kinematic model)
- learned dynamics (linear regression model)
- a probabilistic action selection mechanism
- a smooth barrier function for safety awareness


## Method

At each control step, the controller evaluates a discrete set of actions:

\[
u \in \mathcal{U}
\]

Each action is assigned a probability according to:

\[
p(u_k) \propto q(u_k)\,\exp\left(-\left[c(x_k) + D(\mu_p, \mu_q)\right]\right)
\]

where:

- \( q(u_k) \): prior centered around the nominal human command  
- \( c(x_k) \): cost function on the predicted next state  
- \( \mu_p \): kinematic prediction  
- \( \mu_q \): learned model prediction  
- \( D(\mu_p, \mu_q) \): mismatch penalty (KL-like divergence)

The selected action is:

\[
u^* = \arg\max_u p(u)
\]

---

## Components

### 1. Prior (Human Feedback)

The prior is centered on gesture commands:

- `F`: forward  
- `L`: left  
- `R`: right  
- `S`: stop  


---

### 2. Kinematic Model

A one-step prediction based on a simple differential-drive approximation:

\[
x_k = f(x_{k-1}, u_k)
\]

Used for evaluating the cost function.

---

### 3. Learned Model

A linear regression model predicts the next state:

\[
x_k = f_\theta(x_{k-1}, u_k)
\]

Used to penalize mismatch with the kinematic model.

---

### 4. Cost Function

The cost includes:

- quadratic penalties on lateral deviation \(d\) and heading error \(\phi\)
- a cross-term between \(d\) and \(\phi\)
- a smooth barrier for large deviations

\[
J = w_d d^2 + w_\phi \phi^2 + w_c d\phi + \text{barrier}(d)
\]

The barrier is implemented via a smooth approximation (softplus).

---

### 5. Model Discrepancy

A KL term penalizes disagreement between models:

\[
D(\mu_p, \mu_q) \propto \|\mu_q - \mu_p\|^2
\]
