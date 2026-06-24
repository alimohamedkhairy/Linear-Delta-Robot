#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "../src/stl_reader.h"
#include <fstream>
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;


std::vector<std::vector<double>> load_gcode(const std::string& filename)
{
    
    std::vector<std::vector<double>> decoded;
    
    std::ifstream file(filename);
    std::string line;

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double f = 1800.0;

    while (std::getline(file, line))
    {
        
        if (line.empty())
            continue;

        if (line.rfind("G0", 0) == 0 || line.rfind("G1", 0) == 0)
        {
            std::stringstream ss(line);
            std::string token;

            double e = 0.0;
            bool coordinates_updated = false;
            while (ss >> token)
            {
                if (token[0] == 'X') { x = std::stod(token.substr(1)); coordinates_updated = true; }
                else if (token[0] == 'Y') { y = std::stod(token.substr(1)); coordinates_updated = true; }
                else if (token[0] == 'Z') { z = std::stod(token.substr(1)); coordinates_updated = true; }
                else if (token[0] == 'E') { e = std::stod(token.substr(1)); coordinates_updated = true; }
                else if (token[0] == 'F') { f = std::stod(token.substr(1));}
            }

            if (coordinates_updated) {
                decoded.push_back({x, y, z, e, f});
            }
        }
    }

    return decoded;

}

std::tuple<double, double, double> get_inverse_kinematics(double x, double y, double z, double L, double r, double R)
{
      double z1 = NAN, z2 = NAN, z3 = NAN;

      double Delta = r - R;

      double s1 = (L * L) - (x * x) - ((y + Delta) * (y + Delta));
      double s2 = (L * L) - ((x - ((std::sqrt(3.0) / 2.0) * Delta)) *
                              (x - ((std::sqrt(3.0) / 2.0) * Delta)))
                            - ((y - (0.5 * Delta)) * (y - (0.5 * Delta)));

      double s3 = (L * L) - ((x + ((std::sqrt(3.0) / 2.0) * Delta)) *
                              (x + ((std::sqrt(3.0) / 2.0) * Delta)))
                            - ((y - (0.5 * Delta)) * (y - (0.5 * Delta)));

      if (s1 < 0 || s2 < 0 || s3 < 0) {
          throw std::runtime_error("Point is outside the workspace");
      }

      double z1_root1 = z + std::sqrt(s1);
      //double z1_root2 = z - std::sqrt(s1);

      double z2_root1 = z + std::sqrt(s2);
      //double z2_root2 = z - std::sqrt(s2);

      double z3_root1 = z + std::sqrt(s3);
      //  double z3_root2 = z - std::sqrt(s3);

      // Same selection as MATLAB (currently choosing + root)
      z1 = z1_root1 - 0.196;
      z2 = z2_root1 - 0.196;
      z3 = z3_root1 - 0.196;

      return std::make_tuple(z1, z2, z3);

}

class TrajectoryCreator : public rclcpp::Node
{

public:

    TrajectoryCreator() : Node("trajectory_creator_node") 
    {

        try
        {
            
            //std::string model_path = "/home/ali/Desktop/Projects/linear_delta_workspace/src/models/model.stl"; 
            //std::string gcode_file = "/tmp/model.gcode";

            //std::string command =
            //    "prusa-slicer "
            //    "--slice " + model_path +
            //    " --use-relative-e-distances" +
            //    " --output " + gcode_file;
                
            //int result = std::system(command.c_str());

            //if (result != 0) RCLCPP_ERROR(this->get_logger(), "PrusaSlicer Failure!");
            //else RCLCPP_ERROR(this->get_logger(), "PrusaSlice Success!");

            positions.push_back({  0.0,  0.0,   5.0, 0.0});
            positions.push_back({-15.0, -15.0,   5.0, 0.0}); 
            positions.push_back({-15.0, -15.0,   2.5, 1.0}); 
            positions.push_back({-15.0,  15.0,   2.5, 1.0}); 
            positions.push_back({ 15.0,  15.0,   2.5, 1.0}); 
            positions.push_back({ 15.0, -15.0,   2.5, 1.0}); 
            positions.push_back({-15.0, -15.0,   2.5, 0.0}); 
            positions.push_back({-15.0, -15.0,   5.0, 0.0}); 
            positions.push_back({  0.0,  0.0, 40.0, 0.0}); 

            //positions = load_gcode(gcode_file);

            RCLCPP_INFO(this->get_logger(), "Loading Model Success!");

        }
        catch(const std::exception& e)
        {
            RCLCPP_INFO(this->get_logger(), "Loading Model Failure!");
        }

        publisher_of_points_sim = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/sim/joint_trajectory_controller/joint_trajectory", 10);
        publisher_of_points_real = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/real/joint_trajectory_controller/joint_trajectory", 10);
        homing_delta = this->create_client<std_srvs::srv::Trigger>("/real/homing");
        point_delta = this->create_client<std_srvs::srv::Trigger>("/real/point");

        homing();

    }

private:

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_of_points_sim;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_of_points_real;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr homing_delta;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr point_delta;
    rclcpp::TimerBase::SharedPtr timer;
    std::vector<std::vector<double>> positions;
    size_t index = 0;

    double last_x = 0.0, last_y = 0.0, last_z = 0.0;

    double x = 0;
    double y = 0;
    double z = 0;
    double e = 0;
    double L = 0.225;   // arm length
    double r = 0.060;    // platform radius
    double R = 0.170;   // base radius

    void homing()
    {

        RCLCPP_INFO(this->get_logger(), "####### Motion Node: Sending homing request");

        while(rclcpp::ok() && !homing_delta->wait_for_service(1s)) RCLCPP_INFO(this->get_logger(), "####### Motion Node: Waiting for homing service to exist");

        if (!rclcpp::ok()) {
        
            RCLCPP_INFO(this->get_logger(), "####### Motion Node: Shutting down");
            return;
    
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

        homing_delta->async_send_request(request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
        {

            try {

                auto response  = future.get();

                if (response->success)
                {
                    
                    RCLCPP_INFO(this->get_logger(), "####### Motion Node: Homing reached"); 
                    RCLCPP_INFO(this->get_logger(), "####### Motion Node: Activating motion generation"); 
                    std::thread(&TrajectoryCreator::send_point, this).detach();

                }
                else
                {

                    RCLCPP_INFO(this->get_logger(), "####### HardwareInterface: %s", response->message.c_str());

                }
            }
            catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "SERVICE ERROR: %s", e.what());
            }

        });

    }

    void send_point()
    {
        
        while (rclcpp::ok())
        {

            if (positions.empty() || index >= positions.size()) {

                RCLCPP_INFO(this->get_logger(), "Printing Completed!");
                return;
            
            }

            auto sim_msg = trajectory_msgs::msg::JointTrajectory();
            auto real_msg = trajectory_msgs::msg::JointTrajectory();

            sim_msg.joint_names = {
                "rod_0_to_dynamic_stabilizer_0_joint",
                "rod_1_to_dynamic_stabilizer_1_joint",
                "rod_2_to_dynamic_stabilizer_2_joint"
            };

            real_msg.joint_names = {
                "rod_0_to_dynamic_stabilizer_0_joint",
                "rod_1_to_dynamic_stabilizer_1_joint",
                "rod_2_to_dynamic_stabilizer_2_joint",
                "extrusion_joint"
            };

            auto current_position = positions[index];
            double raw_x = current_position[0];
            double raw_y = current_position[1];
            double raw_z = current_position[2];
            double raw_e = current_position[3];
            
            double duration_sec = 0.1;

            x = raw_x / 1000.0;
            y = raw_y / 1000.0;
            z = raw_z / 1000.0;
            e = raw_e / 1000.0;

            try
            {

                auto [z1, z2, z3] = get_inverse_kinematics(x, y, z, L, r, R);

                auto sim_point = trajectory_msgs::msg::JointTrajectoryPoint();
                sim_point.positions = {z1, z2, z3};
                sim_point.time_from_start = rclcpp::Duration::from_seconds(duration_sec + 0.5);

                auto real_point = trajectory_msgs::msg::JointTrajectoryPoint();
                real_point.positions = {z1, z2, z3, e};
                real_point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);

                RCLCPP_INFO(this->get_logger(), "Motion Sending: Z1_joint=%.3f, Z2_joint=%.3f, Z3_joint=%.3f, E=%.2f", 
                real_point.positions[0], real_point.positions[1], real_point.positions[2], real_point.positions[3]);

                sim_msg.points.push_back(sim_point);
                real_msg.points.push_back(real_point);

                publisher_of_points_sim->publish(sim_msg);
                publisher_of_points_real->publish(real_msg);

                auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
                auto point_next = point_delta->async_send_request(request);

                if (point_next.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
                {

                    auto response = point_next.get();
                    if (response->success)
                    {

                        RCLCPP_INFO(this->get_logger(), "####### HardwareInterface: %s", response->message.c_str());
                        RCLCPP_INFO(this->get_logger(), "####### Motion Node: Next point reached"); 
                        RCLCPP_INFO(this->get_logger(), "####### Motion Node: Sending new point"); 
                        last_x = raw_x;
                        last_y = raw_y;
                        last_z = raw_z;
                        index++;

                    }
                    else 
                    {
                        
                        RCLCPP_INFO(this->get_logger(), "####### HardwareInterface: %s", response->message.c_str());
                        std::this_thread::sleep_for(20ms);
                    
                    }

                }

            }
            catch(const std::runtime_error& e)
            {

                RCLCPP_ERROR(this->get_logger(), "Motion Node: %s" , e.what());
                index++;

            }

        }
    }

};

int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryCreator>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    executor.spin();
    rclcpp::shutdown();

}