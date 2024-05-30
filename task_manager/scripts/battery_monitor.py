import rclpy
from rclpy.node import Node
import sys
import os
import yaml

from crazyswarm_application.msg import AgentsStateFeedback

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class BatterDisplay(Node):
    
    def __init__(self):
        super().__init__('BatterDisplay')
        # load crazyflies config
        crazyflies_yaml = os.path.join(
            "../../",
            "crazyswarm2",
            "crazyflie",
            'config',
            'crazyflies.yaml')

        with open(crazyflies_yaml, 'r') as ymlfile:
            crazyflies = yaml.safe_load(ymlfile)

        self.critical = \
            crazyflies['robot_types']['cf21']['battery']['voltage_critical']

        self.create_subscription(
            AgentsStateFeedback, 
            "/agents", 
            self.listener_callback, 
            10)
        self.agents_dict = {}
        self.timer_freq = 0.5  # seconds
        self.timer = self.create_timer(
            (1/self.timer_freq), self.timer_callback)

    def listener_callback(self, msg):
        for agent in msg.agents:
            self.agents_dict[agent.id] = agent.battery
        return

    def timer_callback(self):
        os.system("clear")
        format_count = 1
        divisor = 3
        for name, batt in self.agents_dict.items():
            batt_dp = "{:.2f}".format(batt)
            critical_dp = "{:.2f}".format(self.critical)
            print(f"{name}:", end=" ")
            if (batt_dp > critical_dp):
                print(f"{bcolors.OKGREEN}{batt_dp}V{bcolors.ENDC}", end="    ")
            else:
                print(f"{bcolors.FAIL}{batt_dp}V{bcolors.ENDC}", end="    ")
            if (format_count%divisor == 0 or format_count == len(self.agents_dict)):
                print("")
            format_count += 1;

def main():
        
    rclpy.init(args=None)

    battery_display = BatterDisplay()
    rclpy.spin(battery_display)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    battery_display.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()