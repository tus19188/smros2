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
from words2numbers import words_to_numbers

class Listener(Node):

    def __init__(self):
        super().__init__('Mayo_listener')
        self.sub = self.create_subscription(String, 'chatter', self.chatter_callback, 10)
        self.my_first_name = "Stephanie"

    def convert_to_number(self, word):
        try:
            # Use words2numbers library to convert text to number
            return str(words_to_numbers(word))
        except ValueError:
            # If the word cannot be converted, leave it as is
            return word

    def split_message(self, message):
        words = message.split()
        return words

    def chatter_callback(self, msg):
        words = self.split_message(msg.data)
        converted_message = []

        for word in words:
            converted_word = self.convert_to_number(word)
            converted_message.append(converted_word)

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
