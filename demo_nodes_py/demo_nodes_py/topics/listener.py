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
from word2number import w2n


class Listener(Node):

    def __init__(self):
        super().__init__('Mayo_listener')
        self.sub = self.create_subscription(String, 'chatter', self.chatter_callback, 10)
        self.my_first_name = "Stephanie"

    def convert_to_number(self, word):  # Added 'self' parameter
        if "hundred" in word:
            parts = word.split()
            if "and" in parts:
                # Handle numbers like "one hundred and eight"
                idx_and = parts.index("and")
                before_and = parts[:idx_and]
                after_and = parts[idx_and + 1:]
                if len(before_and) == 2 and before_and[0] == "one" and before_and[1] == "hundred":
                    return str(w2n.word_to_num(" ".join(after_and)))
                else:
                    return word
            elif parts[0] == "one" and parts[1] == "hundred":
                return "100"
            else:
                return word
        try:
            return str(w2n.word_to_num(word))
        except ValueError:
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



