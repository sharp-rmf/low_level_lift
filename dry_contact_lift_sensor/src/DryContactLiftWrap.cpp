/*TODO:
  Created by : Pallavi, You Liang
  Created on : 25-09-2018, restructured on Dec 2019
  Description: This file subscribes to the ROS2 topic and if there is an open door command over the topic, it sets the GPIO pin on the Pi
*/


// HUGE TODO, code refactoring and add one seperate lib for pins manipulation

#include "dry_contact_lift_sensor/DryContactLiftWrap.hpp"

using namespace std;
using std::placeholders::_1;

DryContactLiftController::DryContactLiftController(const rclcpp::NodeOptions & options) : Node("dry_contact_lift_node")
{
  gpiosetup();
  
  // Calback groups
  fire_alarm_pub_cbg_ = this->create_callback_group(
      rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  lift_state_pub_cbg_ = this->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  lift_req_sub_cbg_ = this->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

  // Timer cb groups, Read alarm signals every 1 seconds
  fire_alarm_timer_cb_ = this->create_wall_timer(
    1000ms, std::bind(&DryContactLiftController::read_alarm_signals_callback, this), fire_alarm_pub_cbg_ );
  lift_state_timer_cb_  = this->create_wall_timer(
    1000ms, std::bind(&DryContactLiftController::publish_lift_state_callback, this), lift_state_pub_cbg_ );

  lift_state_message_ = std::make_shared<rmf_lift_msgs::msg::LiftState>();
  fire_alarm_message_ = std::make_shared<std_msgs::msg::Bool>();

  // Listen when AGV wants to use the lift
  auto lif_req_sub_option = rclcpp::SubscriptionOptions();
  lif_req_sub_option.callback_group = lift_req_sub_cbg_;
  lift_request_subscription_ = this->create_subscription<rmf_lift_msgs::msg::LiftRequest>(
      "lift_requests", 10, std::bind(&DryContactLiftController::lift_request_cb, this, _1), lif_req_sub_option);

  lift_state_publisher_ = this->create_publisher<rmf_lift_msgs::msg::LiftState>("lift_states", 10);
  fire_alarm_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/fire_alarm_trigger", 10); //absolute topic name
}

DryContactLiftController::~DryContactLiftController()
{
    cout << " Trigger 'DryContactLiftController' destructor !! " << endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup and Initialization
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void DryContactLiftController::gpiosetup()
{
  RCLCPP_INFO(this->get_logger(), "setup wiringPi");
#ifdef BCM2835
  if (!bcm2835_init())
  {
    RCLCPP_INFO(this->get_logger(), "setup BCM2835 failed");
  }
  bcm2835_gpio_fsel(FIRE_ALARM, BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_set_pud(FIRE_ALARM, BCM2835_GPIO_PUD_DOWN);

  bcm2835_gpio_fsel(CODE_BLUE, BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_set_pud(CODE_BLUE, BCM2835_GPIO_PUD_DOWN);

  bcm2835_gpio_fsel(AGV_MODE_REQUEST, BCM2835_GPIO_FSEL_OUTP);

  bcm2835_gpio_fsel(AGV_MODE_STATUS, BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_set_pud(AGV_MODE_STATUS, BCM2835_GPIO_PUD_DOWN);

  bcm2835_gpio_fsel(LIFT_CALL_1, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(LIFT_CALL_2, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(LIFT_CALL_3, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(LIFT_CALL_4, BCM2835_GPIO_FSEL_OUTP);

  // gpio off pins
  bcm2835_gpio_write(AGV_MODE_REQUEST, LOW);
  bcm2835_gpio_write(LIFT_CALL_1, LOW);
  bcm2835_gpio_write(LIFT_CALL_2, LOW);
  bcm2835_gpio_write(LIFT_CALL_3, LOW);
  bcm2835_gpio_write(LIFT_CALL_4, LOW);
  bcm2835_gpio_write(DOOR_OPEN_COMMAND, LOW);
  bcm2835_gpio_write(DOOR_CLOSE_SIGNAL, LOW);

  bcm2835_gpio_fsel(ARRIVAL_FLOOR_36, BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_set_pud(ARRIVAL_FLOOR_36, BCM2835_GPIO_PUD_DOWN);

  bcm2835_gpio_fsel(ARRIVAL_FLOOR_37, BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_set_pud(ARRIVAL_FLOOR_37, BCM2835_GPIO_PUD_DOWN);

  bcm2835_gpio_fsel(ARRIVAL_FLOOR_38, BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_set_pud(ARRIVAL_FLOOR_38, BCM2835_GPIO_PUD_DOWN);

  bcm2835_gpio_fsel(ARRIVAL_FLOOR_39, BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_set_pud(ARRIVAL_FLOOR_39, BCM2835_GPIO_PUD_DOWN);

  bcm2835_gpio_fsel(DOOR_STATUS, BCM2835_GPIO_FSEL_INPT);
  bcm2835_gpio_set_pud(DOOR_STATUS, BCM2835_GPIO_PUD_DOWN);

  bcm2835_gpio_fsel(DOOR_OPEN_COMMAND, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(DOOR_CLOSE_SIGNAL, BCM2835_GPIO_FSEL_OUTP);

#endif
}

void DryContactLiftController::param_initialize()
{
  RCLCPP_INFO(this->get_logger(), "initialising parameters..");
  lift_id = this->declare_parameter("lift_id_param", "LF002");
  lift_car_timeout = this->declare_parameter("lift_car_timeout_param", 60);
  door_timeout = this->declare_parameter("door_timeout_param", 10);
  cout << "Fetched lift id : " << lift_id << endl;
  cout << "Fetched lift car_timeout and : " << lift_car_timeout << endl;
  std::vector<std::string> default_available_floors = {"L1", "L2", "L3", "L4"};
  std::vector<std::string> avail_floors = this->declare_parameter("available_floors_param", default_available_floors);
  for (int i =0; i < avail_floors.size(); i++)
  {
    cout << avail_floors.at(i) << endl;
  }
  lift_state_message_->available_floors = avail_floors;
  std::vector<std::string> default_available_modes = {"0", "1", "2", "3"};
  std::vector<uint8_t> lift_msg_available_modes;
  std::vector<std::string> available_modes = this->declare_parameter("available_modes_param", default_available_modes);
  for (int i =0; i < available_modes.size(); i++)
  {
    cout << available_modes.at(i) << endl;
    char avail_modes_ = (available_modes.at(i)).back();
    uint8_t lift_avail_modes_ = avail_modes_;
    lift_avail_modes_ -= 48;
    cout << lift_avail_modes_ << endl;
    lift_msg_available_modes.push_back(lift_avail_modes_);
    cout << lift_msg_available_modes.at(i) <<endl;
  }

  //todo: convert ascii to numbers
  lift_state_message_->available_modes = lift_msg_available_modes;  
  lift_state_message_->lift_name = lift_id;
  lift_state_message_->motion_state = lift_state_message_->MOTION_UNKNOWN;
  lift_state_message_->current_mode = lift_state_message_->MODE_UNKNOWN;

  // hardcoded for now: TODO
  lift_lvl_gpio_write_ =  {LIFT_CALL_1, LIFT_CALL_2, LIFT_CALL_3, LIFT_CALL_4};
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Timer callbacksssss
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// publish a heartbeat
void DryContactLiftController::read_alarm_signals_callback()
{
#ifdef BCM2835
  if (bcm2835_gpio_lev(FIRE_ALARM) == 0)
  {
    RCLCPP_INFO(this->get_logger(), "Received fire alarm event");
    lift_state_message_->current_mode = lift_state_message_->MODE_FIRE;
    fire_alarm_message_->data = 1;  
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "No fire alarm detected");
    fire_alarm_message_->data = 0;
    if ( lift_state_message_->current_mode == lift_state_message_->MODE_FIRE)
      lift_state_message_->current_mode = lift_state_message_->MODE_UNKNOWN;
  }
  if (bcm2835_gpio_lev(CODE_BLUE) == 0)
  {
    RCLCPP_INFO(this->get_logger(), "Received code blue event");
    lift_state_message_->current_mode = lift_state_message_->MODE_EMERGENCY;
  }
#endif
  fire_alarm_publisher_->publish(*fire_alarm_message_);
}

// publish a heartbeat
void DryContactLiftController::publish_lift_state_callback(){
  lift_state_message_->lift_time = ros_clock_.now();

  #ifdef BCM2835

  // Update lift's door state
  if ( bcm2835_gpio_lev(DOOR_STATUS) == 0)
    lift_state_message_->door_state = lift_state_message_->DOOR_OPEN;
  else if ( bcm2835_gpio_lev(DOOR_STATUS) == 1)
    lift_state_message_->door_state = lift_state_message_->DOOR_CLOSED;
  else // impossible
    lift_state_message_->door_state = lift_state_message_->DOOR_OPEN;

  #endif

  lift_state_message_->current_floor = get_current_floor();
  
  lift_state_publisher_->publish(*lift_state_message_);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// functions for lift floor manipulation
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



bool DryContactLiftController::go_to_destination_floor(std::string req_floor)
{
  RCLCPP_INFO(this->get_logger(), "Calling lift '%s' floor", req_floor.c_str());
  time_t start_time = time(0);
  char int_req_floor = req_floor.back();
  // convert char number to int
  int req_floor_bin = (int)int_req_floor - 48; 
  // Lift level is mapped to lift call pin in init
  int lift_call_pin = lift_lvl_gpio_write_[req_floor_bin - 1];

  // check if going up or down
  if(lift_state_message_->destination_floor < lift_state_message_->current_floor){
    lift_state_message_->motion_state = lift_state_message_->MOTION_DOWN;
  }
  else{
    lift_state_message_->motion_state = lift_state_message_->MOTION_UP;
  }
  
  // Wait till lift reaches the requested floor
  while (lift_state_message_->destination_floor != lift_state_message_->current_floor)
  {
    if ( time(0) > (start_time + lift_car_timeout) )
    {
      RCLCPP_INFO(this->get_logger(), "Received lift car timeout");
      lift_state_message_->destination_floor = "";
      lift_state_message_->current_mode = lift_state_message_->MODE_UNKNOWN;
      return false;
    }

    // move to requested floor
    bcm2835_gpio_write(lift_call_pin, HIGH);
    // close door
    bcm2835_gpio_write(DOOR_OPEN_COMMAND, HIGH);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }

  // reached floor. hurray!
  RCLCPP_INFO(this->get_logger(), "Called lift '%s' floor", lift_state_message_->current_floor.c_str());
  lift_state_message_->motion_state = lift_state_message_->MOTION_STOPPED;
  #ifdef BCM2835
    bcm2835_gpio_write(lift_call_pin, LOW);
  #endif

  return true;
}

// @return: if door is valid
bool DryContactLiftController::check_destination_floor(std::string req_floor)
{ 
  for(int idx = 0; idx < lift_state_message_->available_floors.size(); idx++){
    if(req_floor == lift_state_message_->available_floors[idx])
    {
      RCLCPP_INFO(this->get_logger(), "lift floor is avail");
      return true;
    }
  }
  
  RCLCPP_ERROR(this->get_logger(), "This lift floor is not available");
  lift_state_message_->destination_floor = "";
  lift_state_message_->motion_state = lift_state_message_->MOTION_STOPPED;
  return false;
}

int DryContactLiftController::convertBinaryToDecimal(uint16_t array[4])
{
  int decimalNumber = 0;
  int i = 0;
  for (int n = 0; n < 4; n++)
  {
    int remainder_ = array[n] % 10;
    decimalNumber += remainder_ * pow(2, i);
    ++i;
  }
  return decimalNumber;
}

std::string DryContactLiftController::get_current_floor()
{
  uint16_t current_floor_decimal[4] = {0};
  //todo: get current floor
  #ifdef BCM2835
    if (bcm2835_gpio_lev(ARRIVAL_FLOOR_36))
    {
      current_floor_decimal[0] = 1;
    }
    if (bcm2835_gpio_lev(ARRIVAL_FLOOR_37))
    {
      current_floor_decimal[1] = 1;
    }
    if (bcm2835_gpio_lev(ARRIVAL_FLOOR_38))
    {
      current_floor_decimal[2] = 1;
    }
    if (bcm2835_gpio_lev(ARRIVAL_FLOOR_39))
    {
      current_floor_decimal[3] = 1;
    }
  #endif
  int floor_num = convertBinaryToDecimal(current_floor_decimal) + 1;
  return "L" + std::to_string(floor_num);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// MAIN Callback when receive lift request!
void DryContactLiftController::lift_request_cb(const rmf_lift_msgs::msg::LiftRequest::SharedPtr request)
{
  RCLCPP_INFO(this->get_logger(), "Received new request");
  lift_state_message_->session_id = request->session_id;
  lift_state_message_->current_mode = lift_state_message_->MODE_AGV;
  lift_state_message_->motion_state = lift_state_message_->MOTION_STOPPED;
  lift_state_message_->destination_floor = request->destination_floor;

  // Check Lift ID
  RCLCPP_INFO(this->get_logger(), "Local Lift ID: '%s'", lift_id.c_str());
  if ((request->lift_name) != lift_id)
  {
    RCLCPP_INFO(this->get_logger(), "The request received is not for this Lift ID");
    return;
  }

  // Check Lift if request type is AGV Mode, release lift!!!
  if (request->request_type != request->REQUEST_AGV_MODE)
  {
    RCLCPP_INFO(this->get_logger(), "Received AGV Release mode");
    #ifdef BCM2835
    bcm2835_gpio_write(DOOR_OPEN_COMMAND, HIGH);
    bcm2835_gpio_write(AGV_MODE_REQUEST, LOW);
    cout << " Current bcm2835_gpio_lev(AGV_MODE_STATUS) = " << bcm2835_gpio_lev(AGV_MODE_STATUS) << endl;
    #endif

    lift_state_message_->current_mode = lift_state_message_->MODE_HUMAN;
    gpiosetup();
    RCLCPP_INFO(this->get_logger(), "Finished the reset activity, waiting for the next activity");
    return;
  }

  // ============================= Start managing valid lift request, in AGV mode ==============================
  RCLCPP_INFO(this->get_logger(), "Requested for AGV Mode. Need to check the status");

  // Check floor!! 
  if (request->destination_floor != lift_state_message_->current_floor){
    //Call lift to current floor
    if( !check_destination_floor(request->destination_floor) )
      return;
    if( !go_to_destination_floor(request->destination_floor) )
      return;
  }

  // reached to the requested floor, check door state!
  if (request->door_state != lift_state_message_->door_state){
    RCLCPP_INFO(this->get_logger(), "Reached destination floor, now control the door!!!");
    lift_state_message_->motion_state = lift_state_message_->MOTION_STOPPED;

    if (request->door_state == request->DOOR_OPEN)
    {
      RCLCPP_INFO(this->get_logger(), "Received lift's door open request, sending door open signal to control board");
      #ifdef BCM2835
        bcm2835_gpio_write(DOOR_OPEN_COMMAND, LOW);
      #endif
    }
    else if (request->door_state == request->DOOR_CLOSED)
    {
      RCLCPP_INFO(this->get_logger(), "Received lift's door close request, sending door close signal to control board");
      #ifdef BCM2835
        bcm2835_gpio_write(DOOR_OPEN_COMMAND, HIGH);
      #endif
    }
    RCLCPP_INFO(this->get_logger(), "Process DONE!");
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char *argv[])
{
  // signal(SIGINT, h_sig_sigint);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DryContactLiftController>();
  node->param_initialize();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  exec.cancel();

  rclcpp::shutdown();
  return 0;
}
