#include "../include/linear_delta_hardware_plugin/linear_delta_hardware_plugin.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>
#include <stdexcept>
#include <tuple>

hardware_interface::CallbackReturn LinearDeltaHardwarePlugin::on_init(const hardware_interface::HardwareComponentInterfaceParams & params) {
  
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) return hardware_interface::CallbackReturn::ERROR;

  RCLCPP_INFO(this->get_logger(), "####### INITIALIZING");
  size_t num = info_.joints.size();
  RCLCPP_INFO(this->get_logger(), "###### NUMBER OF JOINTS: %zu", num);

  hw_positions.resize(num, 0.0);
  hw_commands.resize(num, 0.0);
  last_hw_commands.resize(num, -9999.0);

  try {

      serial_port.Open("/dev/ttyACM0");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      serial_port.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

      std::string msg = "START\n";
      serial_port.Write(msg); 

      return hardware_interface::CallbackReturn::SUCCESS;

    } catch (const std::exception& e) {
      
      RCLCPP_ERROR(this->get_logger(), "####### SETUP FAILED: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;

  } 

  return hardware_interface::CallbackReturn::FAILURE;

}

// Implementation of export_state_interfaces
std::vector<hardware_interface::StateInterface> LinearDeltaHardwarePlugin::export_state_interfaces() {

  RCLCPP_INFO(this->get_logger(), "####### MOTOR STATE");
  
  std::vector<hardware_interface::StateInterface> positions;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {

    positions.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions[i]);

  }

  return positions; 

}

// Implementation of export_command_interfaces
std::vector<hardware_interface::CommandInterface> LinearDeltaHardwarePlugin::export_command_interfaces() {
    
  RCLCPP_INFO(this->get_logger(), "####### MOTOR COMMAND");

  std::vector<hardware_interface::CommandInterface> commands;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {

    commands.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands[i]);

  }

  return commands; 

}

hardware_interface::CallbackReturn LinearDeltaHardwarePlugin::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{

  RCLCPP_INFO(this->get_logger(), "####### ACTIVATING");

  sync_node = std::make_shared<rclcpp::Node>("delta_sync_node");

  homing_service = sync_node->create_service<std_srvs::srv::Trigger>("/real/homing", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        
        RCLCPP_INFO(this->get_logger(), "####### HardwareInterface: Motion node requesting homing");
        
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));

        std::string msg = "HOMING\n";

        while(rclcpp::ok()) 
        {
          {
  
            std::lock_guard<std::mutex> lock(serial_mutex);
            if (home) {
              RCLCPP_INFO(this->get_logger(), "####### HardwareInterface: Limit switches reports homing success");
              response->success = true;
              break;
            }
          }

          try {
            std::lock_guard<std::mutex> lock(serial_mutex);
            serial_port.Write(msg); 
            
          }
          catch (const std::exception& e) {
            continue;
          }
          
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
    });

    point_service = sync_node->create_service<std_srvs::srv::Trigger>("/real/point", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {

      RCLCPP_INFO(this->get_logger(), "####### Motion Node: Requesting permission to send next point");

      std::lock_guard<std::mutex> lock(serial_mutex);
      if (next)
      {
        response->success = true;
        response->message = "Arduino reached next point!";
        next = false;

      }
      else
      { 
        response->success = false;
        response->message = "Arduino working!";
      }
    
    });

    std::thread([this]() { rclcpp::spin(sync_node); }).detach();

    return hardware_interface::CallbackReturn::SUCCESS;

}

// Implementation of read
hardware_interface::return_type LinearDeltaHardwarePlugin::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

  std::lock_guard<std::mutex> lock(serial_mutex);

  std::string response;

  while (serial_port.IsDataAvailable())
  {
    try {
      char byte_in;
      serial_port.ReadByte(byte_in); 

      if (byte_in != '\n') {
        read_buffer_ += byte_in;
      } 
      else {

        std::string response = read_buffer_;
        read_buffer_ = ""; 

        while (!response.empty() && (response.back() == '\n' || response.back() == '\r')) {
          response.pop_back();
        }

        if (response.empty()) continue;

        if (home) {
          if (response == "NEXT") {
            next = true;
          }
          else if (response.front() == '<' && response.back() == '>') {
            response = response.substr(1, response.size() - 2);
            std::stringstream ss(response);
            std::string token;

            std::getline(ss, token, ','); hw_positions[0] = std::stod(token); 
            std::getline(ss, token, ','); hw_positions[1] = std::stod(token); 
            std::getline(ss, token, ','); hw_positions[2] = std::stod(token); 
            std::getline(ss, token, ','); hw_positions[3] = std::stod(token);

            RCLCPP_INFO(this->get_logger(), "Arduino Sending: X=%.3f, Y=%.3f, Z=%.3f, E=%.3f", hw_positions[0], hw_positions[1], hw_positions[2], hw_positions[3]);
          }
        }
        else if (response == "HOME") {
          home = true;
        }
        else if (!home)
        {

          if (response.front() == '<' && response.back() == '>')
          {

            response = response.substr(1, response.size() - 2);
            std::stringstream ss(response);
            std::string token;

            std::getline(ss, token, ','); hw_positions[0] = std::stod(token); 
            std::getline(ss, token, ','); hw_positions[1] = std::stod(token); 
            std::getline(ss, token, ','); hw_positions[2] = std::stod(token); 
            std::getline(ss, token, ','); hw_positions[3] = std::stod(token);

          }

        }
      }
    }
    catch (const LibSerial::ReadTimeout&) {
        continue;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Serial read failed: %s", e.what());
        break;
    }
  }

  if (home && !next) {
    for (size_t i = 0; i < hw_positions.size(); i++) {
        hw_positions[i] = hw_commands[i];
    }
  }

  return hardware_interface::return_type::OK;

}

// Implementation of write
hardware_interface::return_type LinearDeltaHardwarePlugin::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {

  std::lock_guard<std::mutex> lock(serial_mutex);

  if (home)
  {

    bool coordinates_changed = false;
    for (size_t i = 0; i < hw_commands.size(); i++) {
        if (std::abs(hw_commands[i] - last_hw_commands[i]) > 0.0001) {
            coordinates_changed = true;
            break;
        }
    }

    if (coordinates_changed) {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "<%.3f,%.3f,%.3f,%.3f>\n", hw_commands[0] * 1000, hw_commands[1] * 1000, hw_commands[2] * 1000, hw_commands[3] * 1000);
        
        try {
            serial_port.Write(buffer);
            last_hw_commands = hw_commands;
        } 
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Serial write failed: %s", e.what());
        }
    }
  }

  return hardware_interface::return_type::OK;

}

PLUGINLIB_EXPORT_CLASS(LinearDeltaHardwarePlugin, hardware_interface::SystemInterface);