import numpy as np
import rospy


class FullyProbabilisticController:
    """
    Fully probabilistic one-step controller for lane following.

    This controller evaluates a discrete action space by combining:
    - a prior distribution centered around the nominal user command
    - a one-step kinematic prediction
    - a learned linear regression model
    - a state-dependent cost function
    - a KL-inspired penalty between analytical and learned predictions

    The final policy is a normalized probability distribution over actions,
    and the selected action is the one with maximum probability.

    Notes
    -----
    The implementation is hybrid:
    - the kinematic model is used for cost evaluation
    - the learned model is used to penalize model mismatch

    Therefore, this is neither a purely model-based controller nor a purely
    learned controller. It is a mixed probabilistic decision rule.
    """

    def __init__(self, lr_model, action_space, sleep_time, sigma_kl=0.1, max_v=None, max_omega=None):
        """
        Initialize the controller.

        Parameters
        ----------
        lr_model : sklearn-like regression model
            Learned model used to predict the next state.
        action_space : list
            Discrete set of feasible actions, each action being [v, omega].
        sleep_time : float
            Sampling time used by the controller.
        sigma_kl : float, optional
            Scaling factor used inside the KL-inspired mismatch penalty.
        max_v : float, optional
            Maximum allowed linear velocity.
        max_omega : float, optional
            Maximum allowed angular velocity.
        """
        self.lr = lr_model
        self.actions = action_space
        self.sleep_time = sleep_time
        self.sigma_kl = sigma_kl
        self.max_v = max_v
        self.max_omega = max_omega

        # Nominal command must be assigned externally by the ROS node.
        # These values define the center of the prior over actions.
        # They are not initialized here, so the caller must set them before use.
        # self.v_nominal = ...
        # self.omega_nominal = ...

    def cost(self, x_next, state):
        """
        Compute the stage cost associated with a predicted next state.

        Parameters
        ----------
        x_next : array-like
            Predicted next state [d_next, phi_next].
        state : array-like
            Current state. It is currently passed for interface consistency,
            but it is not used inside this function.

        Returns
        -------
        float
            Scalar cost value.
        int
            Barrier flag:
            - 1 if |dbar| exceeds the threshold
            - 0 otherwise

        Notes
        -----
        The cost contains:
        - quadratic penalty on lateral error d
        - quadratic penalty on heading error phi
        - cross term between d and phi
        - smooth barrier on large lateral deviations

        The smooth barrier is implemented with a softplus-like function so that
        the cost remains continuous, unlike a hard discontinuous constraint.
        """
        d_next, phi_next = x_next

        # Normalization scales for the state components.
        d_scale = 0.105
        phi_scale = 0.4

        dbar = d_next / d_scale
        phibar = phi_next / phi_scale

        # Cost weights
        w_d = 7
        w_phi0 = 2
        w_cross = 2.5  # The sign depends on the adopted state/sign convention

        # Base quadratic cost
        J = (
            w_d * dbar**2 +
            w_phi0 * phibar**2 +
            w_cross * dbar * phibar
        )

        # Smooth barrier term on normalized lateral displacement.
        # The barrier activates progressively when |dbar| exceeds the threshold.
        threshold = 0.8
        beta = 10.0       # Larger beta -> closer to a hard constraint
        k_barrier = 20.0  # Barrier penalty gain

        x = abs(dbar) - threshold
        soft = np.log1p(np.exp(beta * x)) / beta

        J += k_barrier * soft**2

        # Hard logical flag derived from the same threshold.
        barrier = int(abs(dbar) > threshold)

        return float(J), barrier

    def lr_predict(self, state, action):
        """
        Predict the next state using the learned linear regression model.

        Parameters
        ----------
        state : array-like
            Current controller state, expected in the form [d, phi, dt].
        action : array-like
            Action [v, omega].

        Returns
        -------
        np.ndarray
            Predicted next state.

        Notes
        -----
        The input fed to the linear model is:
            [d, phi, v, omega, dt]

        The original state already contains dt, but it is removed and then
        appended again explicitly. This is logically valid, but not elegant.
        """
        state = state.reshape(3,)
        state = np.delete(state, 2)  # Remove dt from [d, phi, dt]
        input_array = np.append(state, action)
        input_array = np.append(input_array, self.sleep_time).reshape(1, -1)

        new_state = self.lr.predict(input_array)
        return new_state.reshape(-1)

    def kinematic_predict(self, d, phi, action):
        """
        Predict the next state using a simple one-step kinematic model.

        Parameters
        ----------
        d : float
            Current lateral displacement.
        phi : float
            Current heading error.
        action : array-like
            Action [v, omega].

        Returns
        -------
        np.ndarray
            Predicted next state [new_d, new_phi].

        Notes
        -----
        A special case is used for omega ~ 0 to avoid numerical instability
        due to division by a very small angular velocity.
        """
        v, omega = action
        dt = self.sleep_time

        if abs(omega) < 1e-9:
            new_d = d + v * np.sin(phi) * dt
        else:
            new_d = d + (v / omega) * (np.cos(phi) - np.cos(phi + omega * dt))

        new_phi = phi + omega * dt
        return np.array([new_d, new_phi])

    def kl_divergence(self, mu_p, mu_q):
        """
        Compute a simplified KL-like divergence between two mean vectors.

        Parameters
        ----------
        mu_p : array-like
            First predicted mean.
        mu_q : array-like
            Second predicted mean.

        Returns
        -------
        float
            Quadratic mismatch penalty.

        Notes
        -----
        This is not the full KL divergence between two generic Gaussians.
        It is a simplified isotropic quadratic form:
            0.5 * ||mu_q - mu_p||^2 / sigma_kl^2

        In practice, it acts as a disagreement penalty between:
        - the analytical kinematic predictor
        - the learned linear regression predictor
        """
        diff = mu_q - mu_p
        return 0.5 * np.sum(diff**2) / (self.sigma_kl ** 2)

    def compute_q_prior(self):
        """
        Compute the prior distribution over actions.

        Returns
        -------
        np.ndarray
            Normalized prior distribution over the discrete action space.

        Notes
        -----
        The prior is centered around the nominal command:
            [v_nominal, omega_nominal]

        Each action is weighted according to its scaled Euclidean distance from
        the nominal command. Therefore, actions closer to the nominal reference
        are preferred a priori.
        """
        u_nominal = [self.v_nominal, self.omega_nominal]
        q_vals = []

        # Prior variance hyperparameter
        sigma_q = 1

        # Scaling factors used to normalize action components before distance computation.
        # This prevents omega from dominating v only because they have different units/scales.
        v_sigma = 0.2
        omega_sigma = 2

        for u in self.actions:
            u = np.array(u)
            du = u - u_nominal

            du_scaled = np.array([du[0] / v_sigma, du[1] / omega_sigma])
            diff = np.linalg.norm(du_scaled)

            q_val = np.exp(-diff**2 / (2 * sigma_q**2))
            q_vals.append(q_val)

        q_vals = np.array(q_vals)

        # Safe normalization fallback.
        # If numerical issues produce an invalid sum, fall back to a uniform prior.
        s = np.sum(q_vals)
        if not np.isfinite(s) or s <= 0:
            rospy.logwarn("[FPCNode] Invalid prior normalization. Falling back to a uniform prior.")
            return np.ones_like(q_vals) / len(q_vals)

        return q_vals / s

    def compute_policy(self, state):
        """
        Compute the posterior policy over actions.

        Parameters
        ----------
        state : array-like
            Current state, expected in the form [d, phi, dt].

        Returns
        -------
        np.ndarray
            Normalized probability distribution over actions.

        Method
        ------
        For each action:
        1. compute the prior probability q(u)
        2. predict the next state with the kinematic model
        3. predict the next state with the learned model
        4. compute the state cost on the kinematic prediction
        5. compute the KL-like disagreement penalty
        6. assign weight:
               q(u) * exp(-(cost + KL))

        Then normalize all weights.

        Notes
        -----
        The cost is computed only on the kinematic prediction (`mu_p`), not on
        the learned prediction (`mu_q`). The learned model enters only through
        the mismatch penalty.
        """
        q_prior = self.compute_q_prior()
        weights = []

        d, phi = state.reshape(-1)[:2]
        rospy.loginfo("[FPCNode] Sensor data: d=%.3f, phi=%.3f", d, phi)

        for idx, u in enumerate(self.actions):
            u = np.array(u)

            # One-step analytical prediction
            mu_p = self.kinematic_predict(d, phi, u)

            # One-step learned prediction
            mu_q = self.lr_predict(state, u)

            # Cost evaluated on the analytical prediction
            c_step, _ = self.cost(mu_p, state)

            # Mismatch penalty between analytical and learned models
            d_kl = self.kl_divergence(mu_p, mu_q)

            # Final unnormalized weight
            weight = q_prior[idx] * np.exp(-(c_step + d_kl))
            weights.append(weight)

        weights = np.array(weights)
        S = np.sum(weights)

        # Fallback in case of numerical underflow/overflow
        if S <= 0 or not np.isfinite(S):
            return q_prior

        return weights / S

    def check_nominal_barrier(self, state):
        """
        Evaluate the barrier condition for the nominal command only.

        Parameters
        ----------
        state : array-like
            Current state.

        Returns
        -------
        int
            Barrier flag associated with the one-step kinematic prediction of
            the nominal action.

        Notes
        -----
        This check is performed on the nominal command, not on the selected action.
        As a consequence, the returned barrier flag does not necessarily describe
        the safety status of the action that will actually be applied.
        """
        d, phi = state.reshape(-1)[:2]
        nominal_u = np.array([self.v_nominal, self.omega_nominal])

        # One-step prediction under the nominal action
        x_nominal_next = self.kinematic_predict(d, phi, nominal_u)

        # Barrier evaluation
        _, barrier_flag = self.cost(x_nominal_next, state)
        return barrier_flag

    def select_action(self, state):
        """
        Select the control action for the current state.

        Parameters
        ----------
        state : array-like
            Current controller state.

        Returns
        -------
        list or tuple
            Selected action [v, omega].
        int
            Barrier flag.

        Logic
        -----
        - If the nominal command is stop, return zero action immediately.
        - Otherwise:
            1. compute the action policy
            2. select the action with maximum probability
            3. evaluate the barrier on the nominal action
            4. return selected action and nominal barrier flag

        Notes
        -----
        This method uses deterministic selection (`argmax`), not sampling.
        Therefore, despite computing a probabilistic policy, the final controller
        behaves greedily.

        Also note that the barrier flag does not refer to the selected action,
        but to the nominal one.
        """
        # If the nominal command is stop, force stop
        if self.v_nominal == 0 and self.omega_nominal == 0:
            return [0.0, 0.0], 0

        policy_probs = self.compute_policy(state)
        best_idx = np.argmax(policy_probs)

        # Alternative stochastic selection:
        # best_idx = np.random.choice(len(self.actions), p=policy_probs)

        best_action = self.actions[best_idx]
        barrier_flag = self.check_nominal_barrier(state)

        return best_action, barrier_flag