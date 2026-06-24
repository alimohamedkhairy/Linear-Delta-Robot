#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <libserial/SerialPort.h>
#include <std_srvs/srv/trigger.hpp>

class LinearDeltaHardwarePlugin : public hardware_interface::SystemInterface
{

public:

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

    std::vector<double> hw_positions;
    std::vector<double> hw_commands;
    std::vector<double> last_hw_commands;
    std::vector<std::vector<double>> buffer;
    LibSerial::SerialPort serial_port;

    std::mutex serial_mutex;

    std::shared_ptr<rclcpp::Node> sync_node;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr homing_service;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr point_service;

    bool home = false;
    bool next = false;
    bool start = false;

    std::string read_buffer_ = "";
};