import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
import can
  
HEARTBEAT_ID = 0x2052C80  
HEARTBEAT_DATA = [255] * 8

class HeartbeatNode(LifecycleNode):
  def __init__(self):
    super().__init__("Heartbeat_node")
    self.get_logger().info("IN constructor")
    self.timer_ = None
    self.bus = None

  def on_configure(self, state: LifecycleState):
    self.get_logger().info("IN on_configure")

    self.bus = can.interface.Bus(interface='socketcan', channel='vcan0', bitrate=1000000)

    self.timer_ = self.create_timer(0.2, self.send_heartbeat_frame) 
    self.timer_.cancel() #Don't start yet

    return TransitionCallbackReturn.SUCCESS

  def on_cleanup(self, state: LifecycleState):
    self.get_logger().info("IN on_cleanup")
    self.destroy_timer(self.timer_)
    self.bus.shutdown()
    return TransitionCallbackReturn.SUCCESS

  def on_activate(self, state: LifecycleState):
    self.get_logger().info("IN on_activate")
    self.timer_.reset()
    return super().on_activate(state)

  def on_deactivate(self, state: LifecycleState):
    self.get_logger().info("IN on_deactivate")
    self.timer_.cancel()
    return super().on_deactivate(state)

  def on_shutdown(self, state: LifecycleState):
    self.get_logger().info("IN on_shutdown")
    self.destroy_timer(self.timer_)   
    self.bus.shutdown()
    return TransitionCallbackReturn.SUCCESS

  def send_heartbeat_frame(self):
    msg = can.Message(
      arbitration_id = HEARTBEAT_ID, data = HEARTBEAT_DATA, is_extended_id = True
    )

    try:
      self.bus.send(msg)
      self.get_logger().info("Heartbeat sent")
    except can.CanError:
        self.get_logger().info("Heartbeat could not be send")


  def main(args=None):
    rclpy.init(args=args)
    lifecycle_node = HeartbeatNode()
    rclpy.spin(lifecycle_node)
    rclpy.shutdown()

if __name__ == "__main__":
  main()
