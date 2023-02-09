import rclpy
import time
import math
from rclpy.node import Node
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector

# Interface with rumble events
import evdev
from evdev import ecodes, InputDevice, ff


class Rumbler:
    def __init__(self):
        # Find first EV_FF capable event device
        dev = None
        for name in evdev.list_devices():
            dev = InputDevice(name)
            if ecodes.EV_FF in dev.capabilities():
                break

        if dev is None:
            raise Exception("No rumble-compatible device found!")
        self.dev = dev

        rumble = ff.Rumble(strong_magnitude=0x0000, weak_magnitude=0xffff)
        effect_type = ff.EffectType(ff_rumble_effect=rumble)

        self._default_duration_ms = 100
        self.effect = ff.Effect(
            type=ecodes.FF_RUMBLE,
            id=-1,
            direction=0,
            ff_trigger=ff.Trigger(0, 0),
            ff_replay=ff.Replay(self._default_duration_ms, 0),
            u=effect_type,
        )

    def send_rumble(self, duration_ms=100):
        # How many times to repeat write?
        repeat = math.ceil(duration_ms / self._default_duration_ms)

        effect_id = self.dev.upload_effect(self.effect)
        self.dev.write(ecodes.EV_FF, effect_id, repeat)
        time.sleep(duration_ms / 1000.0)
        self.dev.erase_effect(effect_id)


class BumpSubscriber(Node):
    def __init__(self):
        super().__init__("bump_subscriber")
        self.subscription = self.create_subscription(
            HazardDetectionVector,
            'hazard_detection',
            self.listener_callback,
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        )

        self.subscription
        self.rumbler = Rumbler()

        name = self.rumbler.dev.name
        path = self.rumbler.dev.path
        self.get_logger().info(f"Sending rumble commands to device {name} at {path}")

    def handle_hazard_msg(self, msg):
        if msg.type == msg.BUMP:
            log = self.get_logger()
            log.info('bump!', throttle_duration_sec=1)

            try:
                self.rumbler.send_rumble()
            except Exception as e:
                log.error(f'{e}', throttle_duration_sec=1)

    def listener_callback(self, msg):
        for hazard in msg.detections:
            self.handle_hazard_msg(hazard)


def main(args=None):
    rclpy.init(args=args)

    sub = BumpSubscriber()
    rclpy.spin(sub)

    sub.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    main()
