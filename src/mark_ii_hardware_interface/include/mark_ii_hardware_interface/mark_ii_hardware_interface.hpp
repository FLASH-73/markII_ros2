#ifndef MARK_II_HARDWARE_INTERFACE_HPP_
#define MARK_II_HARDWARE_INTERFACE_HPP_

#include <vector>
#include <string>
#include <map>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "visibility_control.h"

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>


namespace mark_ii_hardware_interface
{
class MarkIIHardwareInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MarkIIHardwareInterface)

    MARK_II_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    MARK_II_HARDWARE_INTERFACE_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    MARK_II_HARDWARE_INTERFACE_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    MARK_II_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    MARK_II_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    MARK_II_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    MARK_II_HARDWARE_INTERFACE_PUBLIC
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    std::vector<double> hw_commands_;
    std::vector<double> hw_states_;

    // Hardware connection details
    std::string serial_port_;
    long baudrate_;
    std::string ament_prefix_path_ = std::getenv("AMENT_PREFIX_PATH");

    // Python driver object
    pybind11::object driver_;

    // Joint mappings and calibration data
    std::map<std::string, int> single_servo_joints_;
    std::map<std::string, std::vector<int>> dual_servo_joints_;
    std::map<std::string, std::map<std::string, double>> calibration_;

    // Conversion functions
    double _convert_ticks_to_rad(int ticks, const std::string &joint_name);
    int _convert_rad_to_ticks(double rad, const std::string &joint_name);
    double _convert_ticks_to_gripper_dist(int ticks);
    int _convert_gripper_dist_to_ticks(double dist);
};

}  // namespace mark_ii_hardware_interface

#endif  // MARK_II_HARDWARE_INTERFACE_HPP_
