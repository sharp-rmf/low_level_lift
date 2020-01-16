/*TODO:
  Created by : Pallavi, You Liang
  Created on : 01-01-2019, restructured on Dec 2019
  Description: DryContactLiftWrap node receives the request from the AGV over ROS2 DDS once it approaches the lift and sends the command to the lift control board via the dry contact and receives the status from the lift controller board and publishes it over ROS2 DDS. 
*/

#ifndef DRY_CONTACT_LIFT_SENSOR_HPP
#define DRY_CONTACT_LIFT_SENSOR_HPP

#include <iostream>
#include <string>
#include <cstring>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rmf_lift_msgs/msg/lift_request.hpp"
#include "rmf_lift_msgs/msg/lift_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"

//Macro to enable/disable BCM2835
#ifndef BCM2835
#define BCM2835
#endif

#ifdef BCM2835
#include <bcm2835.h>
#endif

#define FIRE_ALARM RPI_BPLUS_GPIO_J8_07  //Read pin - Fire alarm - corresponds to GPIO pin 7
#define CODE_BLUE RPI_BPLUS_GPIO_J8_11   //Read pin - Code Blue - corresponds to GPIO pin 11
#define AGV_MODE_REQUEST RPI_BPLUS_GPIO_J8_12  //Write pin - AGV Mode request - corresponds to GPIO pin 12
#define AGV_MODE_STATUS 2 //Read pin - AGv Mode Status - currently not present as the control board doesn't have this
#define LIFT_CALL_1 RPI_BPLUS_GPIO_J8_15   //Write pin - call Lift to Floor 1 - corresponds to GPIO pin 15
#define LIFT_CALL_2 RPI_BPLUS_GPIO_J8_16   //Write pin - call Lift to Floor 2 - corresponds to GPIO pin 16
#define LIFT_CALL_3 RPI_BPLUS_GPIO_J8_18   //Write pin - call Lift to Floor 3 - corresponds to GPIO pin 18
#define LIFT_CALL_4 RPI_BPLUS_GPIO_J8_22   //Write pin - call Lift to Floor 4 - corresponds to GPIO pin 22
#define ARRIVAL_FLOOR_36 RPI_BPLUS_GPIO_J8_29 //Read pin - pin 36 
#define ARRIVAL_FLOOR_37 RPI_BPLUS_GPIO_J8_31 //Read pin - pin 37
#define ARRIVAL_FLOOR_38 RPI_BPLUS_GPIO_J8_32 //Read pin - pin 38
#define ARRIVAL_FLOOR_39 RPI_BPLUS_GPIO_J8_33 //Read pin - pin 39
#define DOOR_STATUS RPI_BPLUS_GPIO_J8_35   //Read pin - Door Status, 1 closed, 0 open - corresponds to GPIO pin 35
#define DOOR_OPEN_COMMAND RPI_BPLUS_GPIO_J8_36  //Write pin - Door Open - corresponds to GPIO pin 36
#define DOOR_CLOSE_SIGNAL RPI_BPLUS_GPIO_J8_37  //Write pin - Door Open - corresponds to GPIO pin 37

class DryContactLiftController : public rclcpp::Node
{
public:
  DryContactLiftController(const rclcpp::NodeOptions & options = (
      rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true)
  ));
  virtual ~DryContactLiftController();

  void param_initialize();

private:

  void read_alarm_signals_callback();
  void publish_lift_state_callback();

  bool check_destination_floor(std::string req_floor);
  bool go_to_destination_floor(std::string req_floor);
  std::string get_current_floor();

  void lift_request_cb(const rmf_lift_msgs::msg::LiftRequest::SharedPtr request);
  
  int convertBinaryToDecimal(uint16_t array[4]);

  void gpiosetup();

  std::string lift_id;
  char buffer[5];
  int door_timeout;
  int lift_car_timeout;

  std::vector<int> lift_lvl_gpio_write_;

  //rclcpp::Publisher<rmf_lift_msgs::msg::AlarmingEvent>::SharedPtr alarming_event_publisher_;
  rclcpp::Clock ros_clock_;

  // callback groups
  rclcpp::callback_group::CallbackGroup::SharedPtr fire_alarm_pub_cbg_;
  rclcpp::callback_group::CallbackGroup::SharedPtr lift_state_pub_cbg_;
  rclcpp::callback_group::CallbackGroup::SharedPtr lift_req_sub_cbg_;

  rclcpp::Subscription<rmf_lift_msgs::msg::LiftRequest>::SharedPtr lift_request_subscription_;
  rclcpp::Publisher<rmf_lift_msgs::msg::LiftState>::SharedPtr lift_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fire_alarm_publisher_;
  std_msgs::msg::Bool::SharedPtr fire_alarm_message_;
  rmf_lift_msgs::msg::LiftState::SharedPtr lift_state_message_;
    
  rclcpp::TimerBase::SharedPtr fire_alarm_timer_cb_;
  rclcpp::TimerBase::SharedPtr lift_state_timer_cb_;

};

#endif
