#include <cstdio>
#include <../include/advantech_usb4718/DAQ.hpp>

int main(int argc, char ** argv)
{
  // Reading the input args 
  if(argc < 2) 
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ERROR: Wrong number of inputs");
    return 0;
  }

  char* board_name_c = argv[1];

  RCLCPP_INFO_STREAM_ONCE(rclcpp::get_logger("rclcpp"), "Board name " << std::string(board_name_c));

  // Turning char* to w_char
  const size_t cSize = strlen(board_name_c) + 1;
  wchar_t* board_name_wc = new wchar_t[cSize];
  mbstowcs(board_name_wc, board_name_c, cSize);

  rclcpp::init(argc, argv);
  std::shared_ptr<DAQ> p_DAQ = std::make_shared<DAQ>(board_name_wc);
  p_DAQ->Spinner(p_DAQ);
  rclcpp::shutdown();

  return 0;
}
