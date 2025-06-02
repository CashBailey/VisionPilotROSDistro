import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode

class OffboardNode(Node):
    def __init__(self):
        super().__init__('offb_node_py')
        self.current_state = State()
        self.state_sub = self.create_subscription(State, 'mavros/state', self.state_cb, 10)
        self.raw_pub = self.create_publisher(PositionTarget, 'mavros/setpoint_raw/local', 10)
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.offb_set_mode = SetMode.Request()
        self.offb_set_mode.custom_mode = 'OFFBOARD'
        self.arm_cmd = CommandBool.Request()
        self.arm_cmd.value = True
        self.last_req = self.get_clock().now()
        self.altitude = 1.0
        self.speed = 0.5
        self.duration = 15.0
        self.movements = [
            (self.speed, 0.0),
            (0.0, self.speed),
            (-self.speed, 0.0),
            (0.0, -self.speed),
        ]

    def state_cb(self, msg: State):
        self.current_state = msg

    def publish_velocity(self, vx, vy, z_fixed, yaw_rate):
        target = PositionTarget()
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = (
            PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_YAW_RATE
        )
        target.velocity.x = float(vx)
        target.velocity.y = float(vy)
        target.position.z = float(z_fixed)
        target.yaw = float(yaw_rate)
        self.raw_pub.publish(target)

    def run(self):
        rate = self.create_rate(20)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            if not self.current_state.connected:
                rate.sleep()
                continue
            now = self.get_clock().now()
            if self.current_state.mode != 'OFFBOARD' and (now - self.last_req).nanoseconds > 5e9:
                if self.set_mode_client.call_async(self.offb_set_mode):
                    self.get_logger().info('OFFBOARD enabled')
                self.last_req = now
            elif not self.current_state.armed and (now - self.last_req).nanoseconds > 5e9:
                if self.arming_client.call_async(self.arm_cmd):
                    self.get_logger().info('Vehicle armed')
                self.last_req = now
            for vx, vy in self.movements:
                for t in range(int(self.duration * 20)):
                    factor = 1.0 - t / (self.duration * 20)
                    self.publish_velocity(vx * factor, vy * factor, self.altitude, 0.0)
                    rate.sleep()
            rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    node = OffboardNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
