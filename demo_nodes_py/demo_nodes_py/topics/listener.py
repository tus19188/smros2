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
import re
from word2number import w2n

class Listener(Node):

    def __init__(self):
        super().__init__('Mayo_listener')
        self.sub = self.create_subscription(String, 'chatter', self.chatter_callback, 10)
        self.my_first_name = "Stephanie"

    def convert_to_number(self, word):
        try:
            # Use word2number library to convert text to number
            return str(w2n.word_to_num(word))
        except ValueError:
            # If conversion is not possible, leave the word as is
            return word

    def split_message(self, message):
        # Use a regular expression to tokenize the message
        words = re.findall(r'\S+', message)
        return words

    def handle_and_number(self, words, index):
        # Handle numbers with "and" (e.g., one hundred and twenty-five)
        number_parts = []

        # Check if the current word is "and"
        if words[index] == "and":
            index += 1

            # Keep parsing until the end of the words or a non-number word is encountered
            while index < len(words) and words[index].isdigit():
                number_parts.append(words[index])
                index += 1

        return index, number_parts

    def chatter_callback(self, msg):
        # Split the received message into words
        words = self.split_message(msg.data)
        converted_message = []

        index = 0
        while index < len(words):
            word = words[index]

            # Check if the current word is "and"
            if word == "and":
                index, number_parts = self.handle_and_number(words, index)
                if number_parts:
                    converted_word = " ".join(number_parts)
                else:
                    converted_word = word
            else:
                converted_word = self.convert_to_number(word)

            converted_message.append(converted_word)
            index += 1

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

