// Copyright (c) 2021 Xeni Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* **********************************************************************
 * Publishes maxsonar/range as sensor_msgs/msg/Range.
 * ***********************************************************************/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

#include <maxsonar/maxsonar.h>

using namespace std::chrono_literals;

class MaxsonarPublisher : public rclcpp::Node
{
  public:
    MaxsonarPublisher()
    : Node("maxsonar_node")
    {
      // Initialize i2c peripheral in the cpu core
      mySonar.init();

      publisher_ = this->create_publisher<sensor_msgs::msg::Range>("maxsonar/range", 5);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MaxsonarPublisher::timer_callback, this));
    }

  private:
    MaxSonar mySonar;
    
    void timer_callback()
    {      
    
      int distance = mySonar.readDistance();  // Range measurment in centimeters
            
      rclcpp::Time now = this->get_clock()->now();
      auto message = sensor_msgs::msg::Range();
      message.header.frame_id = "sonar";
      message.header.stamp = now;
      message.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
      message.field_of_view = 0.1;             
      message.min_range = 0.25;                  // 25 cm.
      message.max_range = 7.65;                  // 765 cm. Range, 70% reflective target
        
      message.range = (float) distance / 100.0;  // range in meters from centimeters

      // from https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/Range.msg
      // # (Note: values < range_min or > range_max should be discarded)
      if((message.range >= message.min_range) && (message.range <= message.max_range)) {
        publisher_->publish(message);
      }
      
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MaxsonarPublisher>());
  rclcpp::shutdown();
  return 0;
}
