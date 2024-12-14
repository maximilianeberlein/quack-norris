#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA
import os

class HeadlightChanger:
    def __init__(self):
        rospy.init_node('headlight_changer', anonymous=True)

        self.bot_name = os.environ.get("VEHICLE_NAME")
        self.led_pub = rospy.Publisher(f'/{self.bot_name}/led_emitter_node/led_pattern', LEDPattern, queue_size=10)
        self.rate = rospy.Rate(2)  # 10 Hz
        self.current_led_pair = 0
        self.led_pairs = [(0, 1), (3, 4)]
        self.emergency = 1

        rospy.on_shutdown(self.shutdown_hook)


    def change_color(self, r, g, b):
        led_pattern = LEDPattern()
        color = ColorRGBA(r, g, b, 1.0)
        led_pattern.rgb_vals = [ColorRGBA(0, 0, 0, 1.0)] * 5  # Assuming 5 LEDs
        for i in self.led_pairs[self.current_led_pair]:
            led_pattern.rgb_vals[i] = color 
        led_pattern.color_mask = [1] * 5  # Enable all LEDs
        led_pattern.frequency = 0  # No blinking
        led_pattern.frequency_mask = [0] * 5  # No blinking
        self.led_pub.publish(led_pattern)

    def normal_pattern(self):
        led_pattern = LEDPattern()
        led_pattern.rgb_vals = [
            ColorRGBA(1.0, 1.0, 1.0, 1.0),  # White
            ColorRGBA(1.0, 0, 0, 1.0),      # Red
            ColorRGBA(0, 0, 0, 1.0),        # Off
            ColorRGBA(1.0, 0, 0, 1.0),      # Red
            ColorRGBA(1.0, 1.0, 1.0, 1.0)  # White

        ]
        led_pattern.color_mask = [1] * 5  # Enable all LEDs
        led_pattern.frequency = 0  # No blinking
        led_pattern.frequency_mask = [0] * 5  # No blinking
        self.led_pub.publish(led_pattern)

    def blue_pattern(self):
        self.change_color(0.0, 0.0, 1.0)
        self.current_led_pair = (self.current_led_pair + 1) % 2  # Move to the next LED

    def violet_pattern(self):
        self.change_color(1.0, 0.0, 1.0)
        self.current_led_pair = (self.current_led_pair + 1) % 2  # Move to the next LED

    def shutdown_hook(self):
        # Turn off all LEDs
        led_pattern = LEDPattern()
        led_pattern.rgb_vals = [ColorRGBA(0, 0, 0, 1.0)] * 5  # Turn off all LEDs
        led_pattern.color_mask = [1] * 5  # Enable all LEDs
        led_pattern.frequency = 0  # No blinking
        led_pattern.frequency_mask = [0] * 5  # No blinking
        self.led_pub.publish(led_pattern)

    def run(self):
        while not rospy.is_shutdown():
            if self.emergency == 0:
                self.normal_pattern()
            elif self.emergency == 1:
                self.blue_pattern()
            elif self.emergency == 2:
                self.violet_pattern()
            self.rate.sleep()

if __name__ == "__main__":
    try:
        headlight_changer = HeadlightChanger()
        headlight_changer.run()
    except rospy.ROSInterruptException:
        pass