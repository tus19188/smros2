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

thon
Copy code
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import re  # Import the 're' module

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
            "four": 4,
            "five": 5,
            "six": 6,
            "seven": 7,
            "eight": 8,
            "nine": 9,
            "ten": 10,
            "eleven": 11,
            "twelve": 12,
            "thirteen": 13,
            "fourteen": 14,
            "fifteen": 15,
            "sixteen": 16,
            "seventeen": 17,
            "eighteen": 18,
            "nineteen": 19,
            "twenty": 20,
            "thirty": 30,
            "forty": 40,
            "fifty": 50,
            "sixty": 60,
            "seventy": 70,
            "eighty": 80,
            "ninety": 90,
            "hundred": 100,
            "thousand": 1000
        }

    def convert_to_number(self, word):
        # Check if the word is in the number mapping dictionary
        if word.lower() in self.number_mapping:
            return str(self.number_mapping[word.lower()])
        # If not found, leave the word as is
        return word

    def split_message(self, message):
        # Use a regular expression to tokenize the message
        words = re.findall(r'\w+|-', message)
        return words

    def chatter_callback(self, msg):
        # Split the received message into words
        words = self.split_message(msg.data)
        converted_message = []

        for word in words:
            converted_word = self.convert_to_number(word)
            converted_message.append(converted_word)

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

