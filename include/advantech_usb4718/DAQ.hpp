
// ROS datatypes
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/u_int8.hpp>
#include "interfaces_advantech_usb4718/srv/reset_do.hpp"
#include "interfaces_advantech_usb4718/srv/set_do_high.hpp"
#include "interfaces_advantech_usb4718/srv/set_do_low.hpp"
#include "interfaces_advantech_usb4718/srv/print_do_status.hpp"

// Standard lib
#include "string"
#include <bitset>
#include <chrono>
#include <functional>
#include <memory>

// DAQ 
#include "bdaqctrl.h"

using namespace std::chrono_literals;

class DAQ : public rclcpp::Node
{
public:
  // --- Board --- // 

  DAQ(const wchar_t* board_name) : Node("DAQ_usb4718")
  {
      InitialiseBoard(board_name);
  }

  // --- Ros --- //

  // Spinner
  void Spinner(std::shared_ptr<DAQ> pObj);

private:

  // --- Ros --- //
  
  // Publishers
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr publisher_di;

  // Services
  bool ResetDoSrv(const std::shared_ptr<interfaces_advantech_usb4718::srv::ResetDo::Request > request,
                      std::shared_ptr<interfaces_advantech_usb4718::srv::ResetDo::Response> response);
  
  bool SetDoHighSrv(const std::shared_ptr<interfaces_advantech_usb4718::srv::SetDoHigh::Request > request,
                        std::shared_ptr<interfaces_advantech_usb4718::srv::SetDoHigh::Response> response); 

  bool SetDoLowSrv(const std::shared_ptr<interfaces_advantech_usb4718::srv::SetDoLow::Request > request,
                      std::shared_ptr<interfaces_advantech_usb4718::srv::SetDoLow::Response> response); 

  bool PrintDoStatusSrv(const std::shared_ptr<interfaces_advantech_usb4718::srv::PrintDoStatus::Request > request,
                            std::shared_ptr<interfaces_advantech_usb4718::srv::PrintDoStatus::Response> response);

  // Callbacks
  void UpdateDigitalInputValuesCallback();

  rclcpp::Service<interfaces_advantech_usb4718::srv::ResetDo>::SharedPtr       service_server_reset_do;
  rclcpp::Service<interfaces_advantech_usb4718::srv::SetDoHigh>::SharedPtr     service_server_set_do_high;
  rclcpp::Service<interfaces_advantech_usb4718::srv::SetDoLow>::SharedPtr      service_server_set_do_low;
  rclcpp::Service<interfaces_advantech_usb4718::srv::PrintDoStatus>::SharedPtr service_server_print_do_status;

  // Timer
  rclcpp::TimerBase::SharedPtr timer;

  // --- Board --- // 
  bool SetDoHigh(int port);
  bool SetDoLow(int port);

  // DI/DO handlers
  Automation::BDaq::InstantDoCtrl* pInstantDoCtrl; 
  Automation::BDaq::InstantDiCtrl* pInstantDiCtrl;

  // Error flag
  Automation::BDaq::ErrorCode	ret = Automation::BDaq::Success;

  // DO memory value
  char do_memory = 0x03; // In binary 00000011 
  uint8_t di_memory = 0x00; // In binary 00000000 

  // Set and reset operator
  char set = 0x01; // 00000001

  bool InitialiseBoard(const wchar_t* board_name);

};