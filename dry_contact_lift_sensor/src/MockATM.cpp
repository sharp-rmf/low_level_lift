/*TODO:
  Created by : Pallavi
  Created on : 25-09-2018
  Description: This file subscribes to the ROS2 topic and if there is an open door command over the topic, it sets the GPIO pin on the Pi
*/

#include "dry_contact_lift_sensor/MockATM.hpp"

using namespace std;
using std::placeholders::_1;

MockAtm::MockAtm() : Node("mock_atm_node")
{
  msg_lift_request = std::make_shared<dry_contact_lift_sensor::msg::LiftRequest>();
  alarming_event_subscriber_ = this->create_subscription<dry_contact_lift_sensor::msg::AlarmingEvent>("/dry_contact_lift_sensor/alarming_event_topic", std::bind(&MockAtm::set_in_operation, this, _1));
  lift_request_publisher_ = this->create_publisher<dry_contact_lift_sensor::msg::LiftRequest>("/dry_contact_lift_sensor/lift_request_topic");
  lift_state_subscription_ = this->create_subscription<dry_contact_lift_sensor::msg::LiftState>("/dry_contact_lift_sensor/lift_state_topic", std::bind(&MockAtm::read_status, this, _1));
  make_lift_request();
}

MockAtm::~MockAtm()
{
}

void MockAtm::set_in_operation(const dry_contact_lift_sensor::msg::AlarmingEvent::SharedPtr alarm_event)
{
  if ((alarm_event->fire_alarm) || (alarm_event->code_blue))
  {
    active_operation = false;
  }
}

void MockAtm::make_lift_request()
{
  RCLCPP_INFO(this->get_logger(), "Starting the request. Checking if any alarm event is raised");
  // auto msg_lift_request = dry_contact_lift_sensor::msg::LiftRequest();
  if (active_operation)
  {
    RCLCPP_INFO(this->get_logger(), "In active operation");
    msg_lift_request->lift_name = lift_id;
    msg_lift_request->exclusive = true;
    msg_lift_request->destination_floor = -1;
    lift_request_publisher_->publish(msg_lift_request);
    RCLCPP_INFO(this->get_logger(), "Made the request");
  }
}

void MockAtm::read_status(const dry_contact_lift_sensor::msg::LiftState::SharedPtr lift_state)
{
  RCLCPP_INFO(this->get_logger(), "Received new lift_state");
  // auto msg_lift_request = dry_contact_lift_sensor::msg::LiftRequest();
  msg_lift_request->lift_name = lift_id;
  msg_lift_request->exclusive = true;
  if (lift_state->exclusive)
  {
    RCLCPP_INFO(this->get_logger(), "lift_state->current_floor : '%d'", lift_state->current_floor);
    RCLCPP_INFO(this->get_logger(), "msg_lift_request->destination_floor : '%d'", lift_state->destination_floor);
    if (!agv_in_lift)
    {
      RCLCPP_INFO(this->get_logger(), "in AGV_MODE and outside the lift");
      if (initial_setup)
      {
        cout << "Enter the current floor of the AGV" /*<< current_floor_ */ << endl;
        cin >> current_floor_;
        msg_lift_request->current_floor = current_floor_;
        cout << "Enter the destination floor of the AGV" /*<< destination_floor_*/ << endl;
        cin >> destination_floor_;
        msg_lift_request->destination_floor = destination_floor_;
        cout << "Requesting to hold the door in open state" << endl;
        msg_lift_request->door_state = 2;
        msg_lift_request->lift_name = lift_id;
        msg_lift_request->exclusive = true;
        lift_request_publisher_->publish(msg_lift_request);
        initial_setup = false;
      }
      if (lift_state->door_state == 2)
      {
        RCLCPP_INFO(this->get_logger(), "AGV is moving inside the lift");
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        RCLCPP_INFO(this->get_logger(), "AGV moved inside the lift");
        msg_lift_request->current_floor = current_floor_;
        msg_lift_request->destination_floor = destination_floor_;
        msg_lift_request->agv_inside_lift = true;
        agv_in_lift = true;
        lift_request_publisher_->publish(msg_lift_request);
      }
    }
    else if (lift_state->current_floor == lift_state->destination_floor)
    {
      RCLCPP_INFO(this->get_logger(), "Lift is in destination floor");
      if (lift_state->door_state == 2)
      {
        RCLCPP_INFO(this->get_logger(), "AGV is moving outside the lift");
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        RCLCPP_INFO(this->get_logger(), "AGV moved outside the lift");
        msg_lift_request->agv_inside_lift = false;
        agv_in_lift = false;
        msg_lift_request->lift_name = lift_id;
        msg_lift_request->exclusive = false;
        lift_request_publisher_->publish(msg_lift_request);
        cout << "Terminating now" << endl;
        rclcpp::shutdown();
      }
    }
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Lift is not in AGV_MODE");
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockAtm>());
  cout << "Terminating now" << endl;
  rclcpp::shutdown();
  return 0;
}
