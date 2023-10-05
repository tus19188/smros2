# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):

    def __init__(self):
        super().__init__('Mayo_listener')
        self.sub = self.create_subscription(String, 'chatter', self.chatter_callback, 10)
        self.my_first_name = "Stephanie"

        # Define a dictionary to map text representations of numbers to numerical values
        self.number_mapping = {
            "one": 1,
            "two": 2,
            "three": 3,
            # Add more mappings as needed
        }

    def chatter_callback(self, msg):
        # Split the received message into words
        words = msg.data.split()
        converted_message = []

        for word in words:
            # Check if the word is in the number mapping dictionary
            if word in self.number_mapping:
                # If it's a known number in text format, replace it with the numerical value
                converted_message.append(str(self.number_mapping[word]))
            else:
                # If not a known number, keep the word as it is
                converted_message.append(word)

        # Join the words back together to form the converted message
        converted_msg = " ".join(converted_message)

        self.get_logger().info(f'{self.my_first_name} heard: {self.my_first_name} -> [{converted_msg}]')

def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
