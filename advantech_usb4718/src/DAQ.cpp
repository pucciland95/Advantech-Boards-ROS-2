#include <../include/advantech_usb4718/DAQ.hpp>

// --------------------------------------------------------- //
// --------------------- Digital Output -------------------- //
// --------------------------------------------------------- //

bool DAQ::SetDoHigh(int port)
{
	char operation = this->set << port; // Shifting the set operation to the right port
	this->do_memory = this->do_memory | operation; // | is the set operation
	this->ret = this->pInstantDoCtrl->Write(0, this->do_memory);

	if(BioFailed(this->ret))  // Checking for board error
	{
		RCLCPP_ERROR(this->get_logger(), "Cannot Set the given port high. Board Error!");
		return false;
	}
	return true;
}

bool DAQ::SetDoLow(int port)
{
	char operation = ~(this->set << port); // Shifting the set operation to the right port
	this->do_memory = this->do_memory & operation; // & is the reset operation
	this->ret = this->pInstantDoCtrl->Write(0, this->do_memory);

	if(BioFailed(this->ret)) // Checking for board error
	{
		RCLCPP_ERROR(this->get_logger(), "Cannot Set the given port high. Board Error!");
		return false;
	}
	return true;
}

bool DAQ::ResetDoSrv(const std::shared_ptr<interfaces_advantech_usb4718::srv::ResetDo::Request > request,
                                       std::shared_ptr<interfaces_advantech_usb4718::srv::ResetDo::Response> response)
{
	char operation = 0x00; // Reset all ports
	this->do_memory = this->do_memory & operation; // & is the reset operation
	this->ret = this->pInstantDoCtrl->Write(0, this->do_memory);

	if(BioFailed(this->ret)) // Checking for board error
	{
		RCLCPP_ERROR(this->get_logger(), "Cannot Reset all ports. Board Error!");
		response->result = "FAIL";
		return false;
	}

	// Printing the new values of the DO
	std::bitset<8> data(this->do_memory);
	RCLCPP_INFO_STREAM(this->get_logger(), "Digital output values set to: " <<  data << "\n");
	response->result = "SUCCESS";

	return true;
}

bool DAQ::SetDoHighSrv(const std::shared_ptr<interfaces_advantech_usb4718::srv::SetDoHigh::Request > request,
                                         std::shared_ptr<interfaces_advantech_usb4718::srv::SetDoHigh::Response> response)
{
	uint8_t port = request->port;

	// Check if port < 0 or > 7
	if(port > 7)
	{
		RCLCPP_ERROR(this->get_logger(), "Port out of bounds");
		return false;
	}

	if(!this->SetDoHigh(port))
		return false;

	// Printing the new values of the DO
	std::bitset<8> data(this->do_memory);
	RCLCPP_INFO_STREAM(this->get_logger(), "Digital output values set to: " <<  data << "\n");
	response->result = "SUCCESS";

	return true;
}

bool DAQ::SetDoLowSrv(const std::shared_ptr<interfaces_advantech_usb4718::srv::SetDoLow::Request > request,
                                        std::shared_ptr<interfaces_advantech_usb4718::srv::SetDoLow::Response> response)
{
	uint8_t port = request->port;

	// Check if port < 0 or > 7
	if(port > 7)
	{
			RCLCPP_ERROR(this->get_logger(), "Port out of bounds");
			return false;
	}

	SetDoLow(port);

	// Printing the new values of the DO
	std::bitset<8> data(this->do_memory);
	RCLCPP_INFO_STREAM(this->get_logger(), "Digital output values set to: " <<  data << "\n");

	response->result = "SUCCESS";

	return true;
}

bool DAQ::PrintDoStatusSrv(const std::shared_ptr<interfaces_advantech_usb4718::srv::PrintDoStatus::Request > request,
                                             std::shared_ptr<interfaces_advantech_usb4718::srv::PrintDoStatus::Response> response)
{
	std::bitset<8> data(this->do_memory);
	RCLCPP_INFO_STREAM(this->get_logger(), "Digital output values: " <<  data << "\n");
	response->result = "SUCCESS";

	return true;
}

// --------------------------------------------------------- //
// --------------------- Digital Input --------------------- //
// --------------------------------------------------------- //
void DAQ::UpdateDigitalInputValuesCallback()
{
	this->pInstantDiCtrl->Read(0, this->di_memory);

	// Publishing the digital inputs
	std_msgs::msg::UInt8 msg;
	msg.data = this->di_memory;
	this->publisher_di->publish(msg);
}

bool DAQ::InitialiseBoard(const wchar_t* board_name)
{
	// DAQ initialization DO
	this->pInstantDoCtrl = Automation::BDaq::AdxInstantDoCtrlCreate();

	// DAQ initialization DI
	this->pInstantDiCtrl = Automation::BDaq::AdxInstantDiCtrlCreate();

	Automation::BDaq::DeviceInformation devInfo(board_name);
	this->ret = pInstantDoCtrl->setSelectedDevice(devInfo);

	if (BioFailed(this->ret))
	{
		RCLCPP_ERROR(this->get_logger(), "Cannot initialise DO board");
		return false;
	}

	this->ret = pInstantDiCtrl->setSelectedDevice(devInfo);

	if (BioFailed(this->ret))
	{
		RCLCPP_ERROR(this->get_logger(), "Cannot initialise DI board");
			return false;
	}
	else
		RCLCPP_INFO(this->get_logger(), "Board correctly initialised");

	// Starting advertising the services
	this->service_server_reset_do        = this->create_service<interfaces_advantech_usb4718::srv::ResetDo>("reset_do",  std::bind(&DAQ::ResetDoSrv, this, std::placeholders::_1, std::placeholders::_2));
	this->service_server_set_do_high     = this->create_service<interfaces_advantech_usb4718::srv::SetDoHigh>("set_do_high",  std::bind(&DAQ::SetDoHighSrv, this, std::placeholders::_1, std::placeholders::_2));
	this->service_server_set_do_low      = this->create_service<interfaces_advantech_usb4718::srv::SetDoLow>("set_do_low",  std::bind(&DAQ::SetDoLowSrv, this, std::placeholders::_1, std::placeholders::_2));
	this->service_server_print_do_status = this->create_service<interfaces_advantech_usb4718::srv::PrintDoStatus>("print_do_status",  std::bind(&DAQ::PrintDoStatusSrv, this, std::placeholders::_1, std::placeholders::_2));

	// Starting publishing
	this->publisher_di = this->create_publisher<std_msgs::msg::UInt8>("DI_board_status", 1);
	this->timer = this->create_wall_timer(100ms, std::bind(&DAQ::UpdateDigitalInputValuesCallback, this));

	// Setting initial board state to 0x03 -> 00000011
	this->SetDoHigh(0);
	this->SetDoHigh(1);

	// Printing board initial status
	std::bitset<8> data(this->do_memory);
	RCLCPP_INFO_STREAM(this->get_logger(), "Digital output initial values: " <<  data << "\n");

	return true;
}

void DAQ::Spinner(std::shared_ptr<DAQ> pObj)
{
	rclcpp::spin(pObj);
}
