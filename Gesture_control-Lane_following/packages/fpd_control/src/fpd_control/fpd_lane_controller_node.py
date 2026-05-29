"""
Free Energy Principle Based Controller Node.

This ROS node implements a probabilistic lane-following controller for a
Duckietown robot. It combines:

- lane pose estimation from the lane filter
- gesture-based nominal commands
- a pretrained linear regression model
- a discrete feasible action space
- a probabilistic controller with barrier-based supervision

The node publishes velocity commands, logs runtime data, and provides LED
feedback when the barrier is active.

This implementation is mainly suitable for experimentation and evaluation.
It is not a fully hardened production controller.
"""

import os
import csv
from pathlib import Path

import numpy as np
import rospkg
import rospy
from sklearn.linear_model import LinearRegression

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import LanePose, Twist2DStamped, LEDPattern
from std_msgs.msg import String, ColorRGBA

# Custom controller
from fpd_control.fpd_lane_controller.controller import FullyProbabilisticController
# from fpd_control.fpd_lane_controller.controller2 import FullyProbabilisticControllerLangevin


# Control loop sampling time [s]
SLEEP_TIME = 0.2
RATE = int(1 / SLEEP_TIME)


def generate_action_space(max_v, max_omega):
    """
    Generate a discrete feasible action space (v, omega).

    The action space is filtered through simple differential-drive constraints:
    - wheel velocity saturation
    - wheel deadband avoidance for turning motions

    Parameters
    ----------
    max_v : float
        Maximum allowed linear velocity. It defines the intended operating range,
        even though the current discretization is explicitly hardcoded.
    max_omega : float
        Maximum allowed angular velocity. It defines the intended operating range,
        even though the current discretization is explicitly hardcoded.

    Returns
    -------
    list of tuple
        List of feasible (v, omega) actions.

    Notes
    -----
    The discretization is handcrafted:
    - v in [0, 0.25) with step 0.025
    - omega in [-2.5, 2.5) with step 0.25

    Each candidate (v, omega) is mapped into left/right wheel speeds and
    discarded if it violates basic hardware-inspired constraints.
    """
    v_values = np.arange(0, 0.25, 0.025)
    omega_values = np.arange(-2.5, 2.5, 0.25)

    # Approximate wheel speed limits [m/s]
    wheel_min = 0.07
    wheel_max = 0.3

    # Approximate wheelbase [m]
    L = 0.1

    actions = []

    for v in v_values:
        for omega in omega_values:
            # Differential-drive mapping:
            # vr = v + (L/2)*omega
            # vl = v - (L/2)*omega
            vr = v + 0.5 * L * omega
            vl = v - 0.5 * L * omega

            # Saturation constraint
            if abs(vr) > wheel_max or abs(vl) > wheel_max:
                continue

            # Straight motion is accepted if within saturation
            if abs(vr - vl) < 1e-6:
                pass
            else:
                # Reject actions that would place one wheel inside the motor
                # deadband while still being nonzero.
                if (0 < abs(vr) < wheel_min) or (0 < abs(vl) < wheel_min):
                    continue

            actions.append((v, omega))

    return actions


class FPCNode(DTROS):
    """
    Fully Probabilistic Controller Node.

    This node:
    - subscribes to lane pose estimates
    - subscribes to gesture commands
    - loads a pretrained linear regression model
    - creates a probabilistic controller
    - periodically computes and publishes actions
    - logs state, commands, and barrier activity
    - uses LEDs to show barrier activation

    Design assumptions
    ------------------
    - Lane pose is provided by Duckietown lane_filter_node.
    - Gesture commands are symbolic commands: F, L, R, S.
    - The controller exposes:
          select_action(state) -> (action, barrier)
    """

    def __init__(self, node_name):
        """
        Initialize the node, publishers/subscribers, model, controller, and logger.
        """
        self.v_nominal = 0.0
        self.omega_nominal = 0.0
        self.step_count = 0

        rospy.loginfo("[FPCNode] __init__: IN")
        super(FPCNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        self.namespace = rospy.get_namespace()
        rospy.loginfo("[FPCNode] namespace='%s'", self.namespace)

        sub_topic_lane = str(self.namespace + "lane_filter_node/lane_pose")
        sub_topic_gesture = "/gesture_command"
        pub_topic = str(self.namespace + "joy_mapper_node/car_cmd")

        rospy.loginfo("[FPCNode] subscribing to: %s", sub_topic_lane)
        rospy.loginfo("[FPCNode] subscribing to: %s", sub_topic_gesture)
        rospy.loginfo("[FPCNode] publishing to:  %s", pub_topic)

        # LED publisher used to visualize barrier activation
        self.led_pub = rospy.Publisher(
            f"{self.namespace}led_emitter_node/led_pattern",
            LEDPattern,
            queue_size=1
        )

        # Subscribers
        self.lane_pose_sub = rospy.Subscriber(sub_topic_lane, LanePose, self.lane_pose_cb)
        self.gesture_sub = rospy.Subscriber(sub_topic_gesture, String, self.gesture_cb)

        # Velocity command publisher
        self.cmd_pub = rospy.Publisher(pub_topic, Twist2DStamped, queue_size=1)

        rospy.loginfo("[FPCNode] calling load_model()")
        self.load_model()
        rospy.loginfo("[FPCNode] load_model() DONE")

        self.max_v = 0.25
        self.max_omega = 2.5
        rospy.loginfo(
            "[FPCNode] limits: max_v=%.3f max_omega=%.3f",
            self.max_v,
            self.max_omega
        )

        # Build the discrete action space used by the controller
        self.actions = generate_action_space(self.max_v, self.max_omega)

        rospy.loginfo("[FPCNode] creating FullyProbabilisticController...")
        self.controller = FullyProbabilisticController(
            self.lr,
            self.actions,
            sleep_time=SLEEP_TIME,
            max_v=self.max_v,
            max_omega=self.max_omega
        )
        # self.controller = FullyProbabilisticControllerLangevin(
        #     self.lr,
        #     sleep_time=SLEEP_TIME,
        #     max_v=self.max_v,
        #     max_omega=self.max_omega
        # )

        # Initialize the controller with the current nominal command
        self.controller.v_nominal = self.v_nominal
        self.controller.omega_nominal = self.omega_nominal
        rospy.loginfo("[FPCNode] controller created.")

        # Current lane pose [d, phi]
        self.lane_pose = np.array([0.0, 0.0], dtype=float)

        # Timestamp of the latest lane measurement
        self._last_lane_t = None

        # -------------------------
        # Logging setup
        # -------------------------
        self.log_dir = os.path.dirname(os.path.abspath(__file__))
        self.log_path = os.path.join(self.log_dir, "fpd_run_log.csv")

        self.log_file = open(self.log_path, "w", newline="")
        self.log_writer = csv.writer(self.log_file)

        self.log_writer.writerow([
            "time",
            "step",
            "d",
            "phi",
            "gesture_v",
            "gesture_omega",
            "chosen_v",
            "chosen_omega",
            "barrier"
        ])
        self.log_file.flush()

        rospy.loginfo("[FPCNode] logging to file: %s", self.log_path)

        # Wait for the first valid measurements before starting the timer.
        # This is convenient for controlled experiments, but it is a blocking
        # design choice and may be too rigid for production environments.
        rospy.loginfo("[FPCNode] Waiting for first LanePose on %s ...", sub_topic_lane)
        rospy.wait_for_message(sub_topic_lane, LanePose)
        rospy.loginfo("[FPCNode] First LanePose received.")

        rospy.loginfo("[FPCNode] Waiting for first GestureCommand on %s ...", sub_topic_gesture)
        rospy.wait_for_message(sub_topic_gesture, String)
        rospy.loginfo("[FPCNode] First GestureCommand received. Starting control loop...")

        rospy.loginfo("[FPCNode] __init__: OUT")

        rospy.on_shutdown(self.stop_robot)

        # Timer-based periodic control loop
        self.timer = rospy.Timer(rospy.Duration(SLEEP_TIME), self.control_step)

    def control_step(self, event):
        """
        Periodic control step.

        At each timer tick:
        1. read the latest lane state
        2. build the controller input
        3. query the probabilistic controller
        4. publish the selected action
        5. append one row to the CSV log

        Notes
        -----
        The controller state currently includes:
            [d, phi, dt]

        This is a simplified state representation and ignores quantities such as:
        - previous control input
        - velocity history
        - filtered derivatives
        """
        self.step_count += 1

        d = float(self.lane_pose[0])
        phi = float(self.lane_pose[1])

        state = np.array([d, phi, SLEEP_TIME], dtype=float).reshape(1, 3)

        try:
            action, barrier = self.controller.select_action(state)
        except Exception as e:
            rospy.logerr(f"[FPCNode] control_step: select_action failed: {e}")
            self.publish_action([0.0, 0.0], 0)
            return

        self.publish_action(action, barrier)

        t = rospy.Time.now().to_sec()
        self.log_writer.writerow([
            f"{t:.6f}",
            self.step_count,
            f"{d:.6f}",
            f"{phi:.6f}",
            f"{self.v_nominal:.6f}",
            f"{self.omega_nominal:.6f}",
            f"{float(action[0]):.6f}",
            f"{float(action[1]):.6f}",
            int(barrier)
        ])

        # Flush at each step for robustness.
        # This is reliable but not efficient for long runs.
        self.log_file.flush()

    def load_model(self):
        """
        Load the pretrained linear regression model from disk.

        The model is stored as a NumPy archive (.npz) containing:
        - coef
        - intercept

        The sklearn LinearRegression object is reconstructed manually.

        Raises
        ------
        FileNotFoundError
            If the model file does not exist.

        Notes
        -----
        This assumes that the stored arrays are dimensionally consistent with
        the controller. No explicit validation is performed.
        """
        rospy.loginfo("[FPCNode] load_model: IN")
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("fpd_control")
        rospy.loginfo("[FPCNode] load_model: pkg_path=%s", pkg_path)

        model_path = Path(pkg_path) / "config" / "model.npz"
        rospy.loginfo("[FPCNode] load_model: model_path=%s", str(model_path))

        if not model_path.exists():
            rospy.logerr("[FPCNode] load_model: missing model file at %s", str(model_path))
            raise FileNotFoundError(f"Model file not found at: {model_path}")

        model_arrays = np.load(str(model_path))
        self.lr = LinearRegression()
        self.lr.coef_ = model_arrays["coef"]
        self.lr.intercept_ = model_arrays["intercept"]

        rospy.loginfo(
            "[FPCNode] load_model: loaded coef_shape=%s intercept_shape=%s",
            str(np.shape(self.lr.coef_)),
            str(np.shape(self.lr.intercept_))
        )
        rospy.loginfo("[FPCNode] Linear regression model loaded from %s", model_path)
        rospy.loginfo("[FPCNode] load_model: OUT")

    def lane_pose_cb(self, msg):
        """
        Lane pose callback.

        Parameters
        ----------
        msg : LanePose
            ROS message containing:
            - d   : lateral displacement from the lane center
            - phi : heading error with respect to the lane direction

        Notes
        -----
        The measurement is stored directly without filtering. This keeps the
        implementation simple, but exposes the controller to raw sensor noise.
        """
        self.lane_pose = np.array([msg.d, msg.phi], dtype=float)
        self._last_lane_t = rospy.Time.now()
        rospy.logdebug("[FPCNode] lane_pose_cb: d=%.4f phi=%.4f", msg.d, msg.phi)

    def gesture_cb(self, msg):
        """
        Gesture command callback.

        Supported commands:
        - F : forward
        - L : turn left
        - R : turn right
        - S : stop

        Any unknown command is treated as stop.

        Parameters
        ----------
        msg : String
            ROS string message containing the gesture command.
        """
        command = msg.data.upper()

        if command == "F":
            self.v_nominal = 0.2
            self.omega_nominal = 0.0
        elif command == "L":
            self.v_nominal = 0.0
            self.omega_nominal = 2.0
        elif command == "R":
            self.v_nominal = 0.0
            self.omega_nominal = -2.0
        elif command == "S":
            self.v_nominal = 0.0
            self.omega_nominal = 0.0
        else:
            self.v_nominal = 0.0
            self.omega_nominal = 0.0

        self.controller.v_nominal = self.v_nominal
        self.controller.omega_nominal = self.omega_nominal

    def led_red(self):
        """
        Turn all LEDs red.

        This is used as a visual indicator that the barrier condition is active.
        """
        msg = LEDPattern()
        msg.rgb_vals = [ColorRGBA(1.0, 0.0, 0.0, 1.0)] * 5
        msg.color_mask = [1, 1, 1, 1, 1]
        msg.frequency = 0
        msg.frequency_mask = [0, 0, 0, 0, 0]
        self.led_pub.publish(msg)

    def led_off(self):
        """
        Turn all LEDs off.
        """
        msg = LEDPattern()
        msg.rgb_vals = [ColorRGBA(0.0, 0.0, 0.0, 1.0)] * 5
        msg.color_mask = [1, 1, 1, 1, 1]
        msg.frequency = 0
        msg.frequency_mask = [0, 0, 0, 0, 0]
        self.led_pub.publish(msg)

    def publish_action(self, action, barrier):
        """
        Publish the selected control action and update LEDs.

        Parameters
        ----------
        action : sequence
            Selected control action [v, omega].
        barrier : bool or int
            Barrier activation flag. If true, LEDs are turned red.

        Notes
        -----
        The barrier flag is currently only used for visualization.
        It does not forcibly override the control action here.
        """
        if barrier:
            self.led_red()
        else:
            self.led_off()

        twist_msg = Twist2DStamped()
        twist_msg.v = float(action[0])
        twist_msg.omega = float(action[1])
        twist_msg.header.stamp = rospy.Time.now()

        self.cmd_pub.publish(twist_msg)

        rospy.loginfo(
            "[FPCNode] Published action: v=%.3f, omega=%.3f",
            action[0],
            action[1]
        )
        rospy.loginfo(
            "[FPCNode] Gesture command -> v_nominal=%.3f omega_nominal=%.3f",
            self.v_nominal,
            self.omega_nominal
        )

    def stop_robot(self, n_times: int = 5, dt: float = 0.05):
        """
        Stop the robot by repeatedly publishing zero-velocity commands.

        This repeated publication is more reliable than sending a single stop
        command, especially during shutdown.

        Parameters
        ----------
        n_times : int, optional
            Number of zero-velocity commands to send.
        dt : float, optional
            Delay between consecutive stop commands [s].
        """
        if hasattr(self, "log_file") and self.log_file is not None and not self.log_file.closed:
            try:
                self.log_file.flush()
                self.log_file.close()
                rospy.loginfo("[FPCNode] log file closed successfully")
            except Exception as e:
                rospy.logwarn(f"[FPCNode] error while closing log file: {e}")

        if not hasattr(self, "cmd_pub"):
            return

        twist_msg = Twist2DStamped()
        twist_msg.v = 0.0
        twist_msg.omega = 0.0

        for i in range(n_times):
            twist_msg.header.stamp = rospy.Time.now()
            try:
                self.cmd_pub.publish(twist_msg)
            except Exception as e:
                rospy.logwarn(f"[FPCNode] stop_robot: publish failed at i={i}: {e}")
            rospy.sleep(dt)

        rospy.loginfo("[FPCNode] stop_robot: sent %d zero-velocity commands", n_times)


if __name__ == '__main__':
    rospy.loginfo("[FPCNode] main: creating node ...")
    node = None
    try:
        node = FPCNode("FPCNode")
        rospy.loginfo("[FPCNode] main: node created, entering spin()")
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"[FPCNode] main: exception: {e}")
        if node is not None:
            node.stop_robot()
        raise
    finally:
        if node is not None:
            node.stop_robot()
        rospy.loginfo("[FPCNode] main: exiting.")