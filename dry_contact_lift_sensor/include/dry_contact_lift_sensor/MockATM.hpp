/*
  Created by : Pallavi
  Created on : 28-08-2018
  Description: This file creates an agv_publisher node. This is a simulator script that simulates the AGV publishings to open the door.

*/

#ifndef MOCK_ATM_HPP
#define MOCK_ATM_HPP

#include <iostream>
#include <string>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "dry_contact_lift_sensor/msg/lift_request.hpp"
#include "dry_contact_lift_sensor/msg/lift_state.hpp"
#include "dry_contact_lift_sensor/msg/alarming_event.hpp"

class MockAtm : public rclcpp::Node
{
  public:
    MockAtm();
    virtual ~MockAtm();
    void set_in_operation(const dry_contact_lift_sensor::msg::AlarmingEvent::SharedPtr alarm_event);
    void make_lift_request();
    void read_status(const dry_contact_lift_sensor::msg::LiftState::SharedPtr state);

  private:
    rclcpp::Subscription<dry_contact_lift_sensor::msg::AlarmingEvent>::SharedPtr alarming_event_subscriber_;
    rclcpp::Publisher<dry_contact_lift_sensor::msg::LiftRequest>::SharedPtr lift_request_publisher_;
    rclcpp::Subscription<dry_contact_lift_sensor::msg::LiftState>::SharedPtr lift_state_subscription_;
    int current_floor_; // = 1;
    int destination_floor_; // = 2;
    std::string lift_id = "AAA";
    bool active_operation = true;
    bool initial_setup = true;
    bool agv_in_lift = false;
    dry_contact_lift_sensor::msg::LiftRequest::SharedPtr msg_lift_request;
};

#endif