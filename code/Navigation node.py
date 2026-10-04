#!/usr/bin/env python3

import math
import time
import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Bool

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy
)


class NavigationNode(Node):

    def __init__(self):
        super().__init__("navigation_node")

        # =================================================
        # ROS parameter
        # =================================================

        self.declare_parameter("mode", "OPEN")
        self.mode = self.get_parameter("mode").value.upper()

        # =================================================
        # Logging
        # =================================================

        self.log_path = os.path.expanduser("~/robot_log.txt")

        with open(self.log_path, "w") as f:
            f.write("=== ROBOT LOG STARTED ===\n")

        self.last_log_time = time.monotonic()

        # =================================================
        # IMU / heading values
        # =================================================

        self.yaw = 0.0
        self.filtered_yaw = 0.0
        self.yaw_offset = None
        self.target_yaw = None

        # Lower = smoother but slower yaw response.
        self.filter_alpha = 0.15

        # Normal heading controller.
        self.kp = 0.027
        self.max_steering = 0.70

        # =================================================
        # TF-Luna distances, in metres
        # =================================================

        self.front_distance = 999.0
        self.rear_distance = 999.0
        self.magenta_distance = 999.9
        # Used when the robot must back away from a front wall.
        self.reverse_enter = 0.30
        self.reverse_threshold = 0.60

        # =================================================
        # Normal lap driving values
        # =================================================

        self.drive_speed = 0.30
        self.reverse_speed = 0.40
        self.turn_throttle = 0.30
        self.turn_speed = 0.60

        self.turn_distance = 0.80
        self.turn_disarm_distance = 5.0
        self.turn_target_angle = 87.0

        self.turn_start_yaw = 0.0
        self.turn_count = 0
        self.turn_reverse_ticks = 0

        # Main ordinary-navigation state.
        self.state = "DRIVING"

        # =================================================
        # Camera: obstacle data
        # =================================================

        self.green_detected = False
        self.green_x = -1.0

        self.red_detected = False
        self.red_x = -1.0

        # =================================================
        # Camera: magenta parking data
        # =================================================

        self.magenta_detected = False
        self.magenta_x = -1.0
        self.magenta_area = 0.0

        self.magenta_last_seen = 0.0
        self.magenta_stable_ticks = 0

        # Camera detection must remain true and valid.
        self.magenta_required_ticks = 5
        self.magenta_min_area = 300.0
        self.magenta_timeout_s = 0.30

        # =================================================
        # Parking state
        # =================================================

        self.in_parking_straight = True

        self.parking_attempted = False
        self.parking_state = "DISABLED"

        # Heading captured while driving parallel to parking wall.
        self.parking_heading = None

        self.parking_start_time = 0.0
        self.parking_state_start_time = 0.0

        # -------------------------------------------------
        # PARKING TUNING VALUES
        # Start slow. Tune on your real WRO mat.
        # -------------------------------------------------

        # Forward movement after magenta is detected.
        self.parking_approach_speed = 0.12
        self.parking_forward_time = 0.35

        # Reverse manoeuvre speeds.
        self.parking_reverse_speed = -0.08
        self.parking_straighten_speed = -0.06

        # Reverse target heading = parking_heading + this angle.
        #
        # Start at +35.0.
        # If robot turns away from the parking slot, use -35.0.
        self.parking_entry_angle = 35.0

        # IMU heading tolerance to count as parallel.
        self.parking_parallel_yaw_tolerance = 3.0

        # Rear TF-Luna values in metres.
        #
        # Tune these based on sensor-to-bumper offset.
        self.parking_straighten_distance = 0.20
        self.parking_stop_distance = 0.11
        self.parking_emergency_stop_distance = 0.07

        # Absolute safety timeout for the whole manoeuvre.
        self.parking_timeout_s = 8.0

        # =================================================
        # Obstacle avoidance state
        # =================================================

        self.obstacle_state = "NONE"
        self.obstacle_direction = None

        self.obstacle_start_yaw = None
        self.obstacle_target_yaw = None

        self.obstacle_reverse_ticks = 0

        self.max_obstacle_turn_angle = 90.0
        self.obstacle_turn_speed = 0.60
        self.obstacle_drive_speed = 0.35
        self.obstacle_clear_distance = 1.00
        
        ##parking##
        self.parking_search_speed = 0.16
        self.parking_turn_angle = 45.0
        self.parking_turn_timeout_s = 10
        self.parking_turn_start_yaw = 0.0

        # =================================================
        # ROS publisher
        # =================================================

        self.motor_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        # =================================================
        # TF-Luna QoS
        # =================================================

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # =================================================
        # Sensor subscriptions
        # =================================================

        self.create_subscription(
            Imu,
            "/imu/data",
            self.imu_callback,
            10
        )

        self.create_subscription(
            Float32,
            "/tf_luna/front",
            self.front_callback,
            sensor_qos
        )

        self.create_subscription(
            Float32,
            "/tf_luna/rear",
            self.rear_callback,
            10
        )

        # =================================================
        # Camera subscriptions: obstacle colours
        # =================================================

        self.create_subscription(
            Bool,
            "/camera/green_detected",
            self.green_callback,
            10
        )

        self.create_subscription(
            Float32,
            "/camera/green_x",
            self.green_x_callback,
            10
        )

        self.create_subscription(
            Bool,
            "/camera/red_detected",
            self.red_callback,
            10
        )

        self.create_subscription(
            Float32,
            "/camera/red_x",
            self.red_x_callback,
            10
        )

        # =================================================
        # Camera subscriptions: parking magenta
        # =================================================

        self.create_subscription(
            Bool,
            "/camera/magenta_detected",
            self.magenta_callback,
            10
        )

        self.create_subscription(
            Float32,
            "/camera/magenta_x",
            self.magenta_x_callback,
            10
        )

        self.create_subscription(
            Float32,
            "/camera/magenta_area",
            self.magenta_area_callback,
            10
        )

        # =================================================
        # Main control loop: 50 Hz
        # =================================================

        self.timer = self.create_timer(
            0.02,
            self.navigation_loop
        )

        self.get_logger().info(
            "Navigation node started."
        )

    # =====================================================
    # General helper methods
    # =====================================================

    def normalize_angle(self, angle):
        while angle > 180.0:
            angle -= 360.0

        while angle < -180.0:
            angle += 360.0

        return angle

    def publish(self, throttle, steering):
        cmd = Twist()

        # Your motor_node uses these as throttle and steering.
        cmd.linear.x = float(throttle)
        cmd.angular.z = float(steering)

        self.motor_pub.publish(cmd)

    def stop(self):
        self.publish(0.0, 0.0)

    def heading_hold(self, target_yaw, speed):
        error = self.normalize_angle(
            target_yaw - self.yaw
        )

        steering = self.kp * error

        steering = max(
            -self.max_steering,
            min(self.max_steering, steering)
        )

        self.publish(speed, steering)

    def log(self, message):
        timestamp = datetime.now().strftime(
            "%H:%M:%S.%f"
        )[:-3]

        with open(self.log_path, "a") as f:
            f.write(
                f"[{timestamp}] {message}\n"
            )

    # =====================================================
    # IMU callback
    # =====================================================

    def imu_callback(self, msg):
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        siny = 2.0 * (
            w * z + x * y
        )

        cosy = 1.0 - 2.0 * (
            y * y + z * z
        )

        raw_yaw = math.degrees(
            math.atan2(siny, cosy)
        )

        raw_yaw = self.normalize_angle(
            raw_yaw
        )

        # On first IMU reading, define current direction as 0 degrees.
        if self.yaw_offset is None:
            self.yaw_offset = raw_yaw
            self.yaw = 0.0
            self.filtered_yaw = 0.0
            self.target_yaw = 0.0
            return

        relative_yaw = self.normalize_angle(
            raw_yaw - self.yaw_offset
        )

        difference = self.normalize_angle(
            relative_yaw - self.filtered_yaw
        )

        self.filtered_yaw += (
            self.filter_alpha * difference
        )

        self.yaw = self.normalize_angle(
            self.filtered_yaw
        )

    # =====================================================
    # TF-Luna callbacks
    # =====================================================

    def front_callback(self, msg):
        self.front_distance = msg.data

    def rear_callback(self, msg):
        self.rear_distance = msg.data

    # =====================================================
    # Camera callbacks
    # =====================================================

    def green_callback(self, msg):
        self.green_detected = msg.data

    def green_x_callback(self, msg):
        self.green_x = msg.data

    def red_callback(self, msg):
        self.red_detected = msg.data

    def red_x_callback(self, msg):
        self.red_x = msg.data

    def magenta_callback(self, msg):
        self.magenta_detected = msg.data

        if msg.data:
            self.magenta_last_seen = time.monotonic()

    def magenta_x_callback(self, msg):
        self.magenta_x = msg.data

    def magenta_area_callback(self, msg):
        self.magenta_area = msg.data

    # =====================================================
    # Red/green obstacle trigger
    # =====================================================

    def colour_detection(self):

        # Do not begin a fresh obstacle manoeuvre while already avoiding.
        if self.obstacle_state != "NONE":
            return

        # GREEN obstacle: pass on its right.
        if (
            self.green_detected
            and 150 < self.green_x < 400
        ):
            self.obstacle_direction = "RIGHT"

            self.obstacle_start_yaw = self.yaw

            self.obstacle_target_yaw = (
                self.normalize_angle(
                    self.obstacle_start_yaw + 45.0
                )
            )

            self.obstacle_state = "TURN_OUT"

            self.get_logger().info(
                f"GREEN -> RIGHT | "
                f"Start={self.obstacle_start_yaw:.1f} | "
                f"Target={self.obstacle_target_yaw:.1f}"
            )

        # RED obstacle: pass on its left.
        elif (
            self.red_detected
            and 150 < self.red_x < 400
        ):
            self.obstacle_direction = "LEFT"

            self.obstacle_start_yaw = self.yaw

            self.obstacle_target_yaw = (
                self.normalize_angle(
                    self.obstacle_start_yaw - 45.0
                )
            )

            self.obstacle_state = "TURN_OUT"

            self.get_logger().info(
                f"RED -> LEFT | "
                f"Start={self.obstacle_start_yaw:.1f} | "
                f"Target={self.obstacle_target_yaw:.1f}"
            )

    # =====================================================
    # Obstacle state machine
    # =====================================================

    def obstacle_logic(self):

        if self.obstacle_state == "NONE":
            return False

        # If an obstacle is too close ahead, reverse before turning.
        if (
            self.front_distance <= self.reverse_enter
            and self.obstacle_state != "REVERSE"
        ):
            self.obstacle_state = "REVERSE"
            self.obstacle_reverse_ticks = 0

        # -------------------------------------------------
        # Reverse until front distance becomes safe.
        # -------------------------------------------------
        if self.obstacle_state == "REVERSE":

            self.publish(-0.20, 0.0)

            if self.front_distance >= self.reverse_threshold:
                self.obstacle_reverse_ticks += 1
            else:
                self.obstacle_reverse_ticks = 0

            if self.obstacle_reverse_ticks >= 5:

                self.obstacle_state = "TURN_OUT"
                self.obstacle_start_yaw = self.yaw

                if self.obstacle_direction == "RIGHT":
                    direction_angle = 45.0
                else:
                    direction_angle = -45.0

                self.obstacle_target_yaw = (
                    self.normalize_angle(
                        self.obstacle_start_yaw
                        + direction_angle
                    )
                )

            return True

        # -------------------------------------------------
        # Turn out around the obstacle.
        # -------------------------------------------------
        if self.obstacle_state == "TURN_OUT":

            error = self.normalize_angle(
                self.obstacle_target_yaw - self.yaw
            )

            turned_angle = abs(
                self.normalize_angle(
                    self.yaw - self.obstacle_start_yaw
                )
            )

            if (
                turned_angle >= self.max_obstacle_turn_angle
                or abs(error) <= 3.0
            ):
                self.obstacle_state = "PASS"

                self.get_logger().info(
                    "Obstacle turn out complete."
                )

                return True

            if error > 0:
                steering = self.obstacle_turn_speed
            else:
                steering = -self.obstacle_turn_speed

            self.publish(
                self.turn_throttle,
                steering
            )

            return True

        # -------------------------------------------------
        # Drive beyond obstacle.
        # -------------------------------------------------
        if self.obstacle_state == "PASS":

            self.publish(
                self.obstacle_drive_speed,
                0.0
            )

            if (
                self.front_distance
                >= self.obstacle_clear_distance
            ):
                self.obstacle_target_yaw = (
                    self.obstacle_start_yaw
                )

                self.obstacle_state = "TURN_BACK"

                self.get_logger().info(
                    f"Obstacle passed. "
                    f"Returning to heading "
                    f"{self.obstacle_start_yaw:.1f}"
                )

            return True

        # -------------------------------------------------
        # Return to original heading.
        # -------------------------------------------------
        if self.obstacle_state == "TURN_BACK":

            error = self.normalize_angle(
                self.obstacle_target_yaw - self.yaw
            )

            if abs(error) <= 3.0:

                self.obstacle_state = "NONE"

                self.obstacle_direction = None
                self.obstacle_start_yaw = None
                self.obstacle_target_yaw = None

                self.target_yaw = self.yaw
                self.state = "DRIVING"

                self.get_logger().info(
                    "Obstacle manoeuvre complete."
                )

                return True

            if error > 0:
                steering = self.obstacle_turn_speed
            else:
                steering = -self.obstacle_turn_speed

            self.publish(
                self.turn_throttle,
                steering
            )

            return True

        return False

    # =====================================================
    # Normal driving / lap state machine
    # =====================================================

    def start_turn(self):
        self.state = "TURNING"
        self.turn_start_yaw = self.yaw

        self.get_logger().info(
            f"Turn started at yaw={self.turn_start_yaw:.1f}"
        )

    def turn_logic(self):

        angle = self.normalize_angle(
            self.yaw - self.turn_start_yaw
        )

        # Corner has completed.
        if (
            abs(angle) >= self.turn_target_angle
            or self.front_distance
            >= self.turn_disarm_distance
        ):
            self.state = "TURN_EXIT"
            self.target_yaw = self.yaw
            return

        # Too close to wall: reverse safely.
        if self.front_distance <= self.reverse_enter:
            self.state = "TURN_REVERSE"
            return

        self.publish(
            self.turn_throttle,
            self.turn_speed
        )

    def turn_reverse_logic(self):

        self.publish(-self.reverse_speed, -0.75)

        if self.front_distance >= self.reverse_threshold:
            self.turn_reverse_ticks += 1
        else:
            self.turn_reverse_ticks = 0

        if self.turn_reverse_ticks >= 5:
            self.turn_reverse_ticks = 0
            self.state = "TURNING"
            self.stop()

    def turn_exit_logic(self):

        self.turn_count += 1

        self.state = "DRIVING"
        self.target_yaw = self.yaw

        self.get_logger().info(
            f"Turn complete: {self.turn_count}"
        )

        # Assumption: four corners per lap.
        # The twelfth turn exits onto the parking/start straight.
        if self.turn_count == 12:
            self.in_parking_straight = True
            self.obstacle_state = "NONE"
            self.obstacle_direction = None
            self.obstacle_start_yaw = None
            self.obstacle_target_yaw = None

            self.get_logger().info(
                "Third lap complete. "
                "Parking straight entered."
            )
#
    def reverse_logic(self):

        self.publish(-self.reverse_speed, 0.0)

        if self.front_distance >= self.reverse_threshold:
            self.state = "TURNING"

    def normal_driving(self):

        self.colour_detection()

        if self.obstacle_state != "NONE":
            self.state = "OBSTACLE"
            return

        if self.target_yaw is None:
            self.target_yaw = self.yaw

        # Front-wall safety.
        if self.front_distance < self.reverse_enter:
            self.state = "REVERSE"
            return

        # Begin a corner.
        if self.front_distance <= self.turn_distance:
            self.start_turn()
            return

        # Drive straight while holding heading.
        self.heading_hold(
            self.target_yaw,
            self.drive_speed
        )

    # =====================================================
    # Magenta parking trigger
    # =====================================================

    def magenta_marker_is_fresh(self):
        return (
            time.monotonic() - self.magenta_last_seen
        ) < self.magenta_timeout_s


    def update_magenta_marker_stability(self):
        marker_is_valid = (
            self.magenta_detected
            and self.magenta_area >= self.magenta_min_area
            and self.magenta_marker_is_fresh()
        )

        if marker_is_valid:
            self.magenta_stable_ticks = min(
                self.magenta_stable_ticks + 1,
                50
            )
        else:
            self.magenta_stable_ticks = 0


    def magenta_marker_confirmed(self):
        return (
            self.magenta_stable_ticks
            >= self.magenta_required_ticks
        )


    def drive_while_searching_for_parking(self):
        # Hold the final straight heading and do not call normal_driving().
        if self.target_yaw is None:
            self.target_yaw = self.yaw

        self.heading_hold(
            self.target_yaw,
            self.parking_search_speed
        )


    def begin_parking_turn(self):
        # Called once when stable magenta has been detected after lap 3.
        self.parking_attempted = True
        self.parking_state = "TURN_IN"

        # Keep a separate reference from normal track turn_start_yaw.
        self.parking_heading = self.yaw
        self.parking_turn_start_yaw = self.yaw
        self.parking_start_time = time.monotonic()

        self.get_logger().info(
            f"PARKING TURN STARTED | "
            f"start_yaw={self.parking_turn_start_yaw:.1f} | "
            f"magenta_x={self.magenta_x:.1f} | "
            f"area={self.magenta_area:.0f}"
        )


    def run_parking_turn(self):
        # This is the 45-degree turn-only parking test.
        if self.parking_state == "TURN_IN":
            angle_turned = self.normalize_angle(
                self.yaw - self.parking_turn_start_yaw
            )

            if (
                time.monotonic() - self.parking_start_time
                > self.parking_turn_timeout_s
            ):
                self.parking_state = "ABORT"
                self.stop()
                self.get_logger().warn(
                    "Parking turn timeout. Robot stopped."
                )
                return

            if abs(angle_turned) >= self.parking_turn_angle:
                self.parking_state = "PARKED"
                self.stop()
                self.get_logger().info(
                    f"PARKING 45 DEG TURN COMPLETE | "
                    f"angle={angle_turned:.1f}"
                )
                return

            # Same movement style as your existing normal turn_logic().
            self.publish(
                self.turn_throttle,
                self.turn_speed
            )
            return

        if self.parking_state in ("PARKED", "ABORT"):
            self.stop()

    # -----------------------------------------------------
    # 5. ADD THIS TO THE TOP OF navigation_loop(), AFTER
    #    LOGGING AND BEFORE NORMAL TURN/DRIVING LOGIC
    # -----------------------------------------------------
        

        if self.turn_count == 12 and self.in_parking_straight:
            if self.parking_state == "DISABLED":
                self.parking_state = "SEARCHING"
                self.get_logger().info(
                    "Three laps complete. Searching for magenta parking wall."
                )

            if self.parking_state == "SEARCHING":
                self.update_magenta_marker_stability()

                if (
                    not self.parking_attempted
                    and self.magenta_marker_confirmed()
                ):
                    self.begin_parking_turn()
                    

                self.drive_while_searching_for_parking()
                

        # -----------------------------------------------------
        # 6. ADD THIS TO turn_exit_logic(), IMMEDIATELY AFTER:
        #    self.turn_count += 1
        # -----------------------------------------------------

                
        
        # =====================================================
        # Main navigation loop
        # =====================================================

    def navigation_loop(self):
        
    
        now = time.monotonic()

        # Log at 10 Hz, not every 50 Hz control tick.
        if now - self.last_log_time >= 0.10:

            self.last_log_time = now

            self.log(
                f"STATE={self.state} | "
                f"PARKING={self.parking_state} | "
                f"Yaw={self.yaw:.2f} | "
                f"TargetYaw={self.target_yaw} | "
                f"Front={self.front_distance:.2f} | "
                f"Rear={self.rear_distance:.2f} | "
                f"Magenta={self.magenta_detected} | "
                f"MagentaArea={self.magenta_area:.0f} | "
                f"MagentaTicks={self.magenta_stable_ticks}"
            )

        # Parking owns control once it has begun.
        if self.parking_state in ("TURN_IN", "PARKED", "ABORT"):
            self.parking_logic()
            return

        # After three laps, search only on parking straight.
        if (
            self.turn_count >= 12
            and self.in_parking_straight
        ):
            if self.parking_state == "DISABLED":

                self.parking_state = "SEARCHING"

                self.get_logger().info(
                    "Three laps complete. "
                    "Searching for magenta parking wall."
                )

            if self.parking_state == "SEARCHING":

                self.update_magenta_stability()

                if (
                    not self.parking_attempted
                    and self.magenta_marker_is_stable()
                ):
                    self.start_parking()
                    return

                self.parking_search_drive()
                return

        # Normal driving before parking begins.
        if self.state == "TURNING":
            self.turn_logic()

        elif self.state == "TURN_EXIT":
            self.turn_exit_logic()

        elif self.state == "REVERSE":
            self.reverse_logic()

        elif self.state == "OBSTACLE":
            self.obstacle_logic()

        elif self.state == "TURN_REVERSE":
            self.turn_reverse_logic()

        elif self.state == "FINISHED":
            self.stop()

        else:
            self.normal_driving()


def main(args=None):

    rclpy.init(args=args)

    node = NavigationNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
