#include "mark_ii_hardware_interface/mark_ii_hardware_interface.hpp"
#include <dlfcn.h>
#include <vector>
#include <string>
#include <sstream> // Required for std::stringstream

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // Required for automatic type casting

namespace py = pybind11;

namespace mark_ii_hardware_interface
{

// --- FIX: Correctly manage interpreter lifecycle ---
MarkIIHardwareInterface::MarkIIHardwareInterface()
{
    // Force libpython into the GLOBAL symbol table
    void * handle = dlopen(PYTHON_LIBRARY_PATH, RTLD_NOW | RTLD_GLOBAL);
    if (!handle) {
        RCLCPP_ERROR(rclcpp::get_logger("MarkIIHardwareInterface"),
                    "dlopen(%s) failed: %s", PYTHON_LIBRARY_PATH, dlerror());
    }
    py::initialize_interpreter();
    // Allow Python threads to be created
    PyEval_InitThreads(); 
}

MarkIIHardwareInterface::~MarkIIHardwareInterface()
{
    // Acquire the GIL to safely finalize
    py::gil_scoped_acquire acquire;
    driver_.release();
    py::finalize_interpreter();
}


hardware_interface::CallbackReturn MarkIIHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    serial_port_ = info_.hardware_parameters["serial_port"];
    baudrate_ = std::stol(info_.hardware_parameters["baudrate"]);

    hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    single_servo_joints_ = {
        {"base_link_Revolute-1", 1}, {"link3_Revolute-5", 6},
        {"link4_Revolute-6", 7}, {"link5_Revolute-7", 8},
        {"link6_Slider-8", 9}
    };
    dual_servo_joints_ = {
        {"link1_Revolute-3", {2, 3}}, {"link2_Revolute-4", {4, 5}}
    };
    calibration_ = {
        {"base_link_Revolute-1", {{"scale", 644.9}, {"offset", 3453}}},
        {"link1_Revolute-3", {{"scale", 651}, {"offset", 3072}}},
        {"link2_Revolute-4", {{"scale", 647.2}, {"offset", 985}}},
        {"link3_Revolute-5", {{"scale", 628}, {"offset", 3112}}},
        {"link4_Revolute-6", {{"scale", 640}, {"offset", 702}}},
        {"link5_Revolute-7", {{"scale", 654}, {"offset", 99}}},
        {"link6_Slider-8", {{"scale", 14427}, {"offset", 2908}}}
    };

    RCLCPP_INFO(rclcpp::get_logger("MarkIIHardwareInterface"), "Initialization successful. Port: %s, Baudrate: %ld", serial_port_.c_str(), baudrate_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MarkIIHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MarkIIHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    }
    return command_interfaces;
}


// --- FIX: Create a new private helper function for Python initialization ---
void MarkIIHardwareInterface::_initialize_python_driver()
{
    // This function assumes the GIL is already held
    if (python_driver_initialized_) {
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("MarkIIHardwareInterface"), "Initializing Python driver in real-time thread...");

    try {
        auto sys = py::module::import("sys");
        const char* python_path_env = std::getenv("PYTHONPATH");
        if (python_path_env)
        {
            std::string python_path_str(python_path_env);
            std::stringstream ss(python_path_str);
            std::string path;
            while (std::getline(ss, path, ':')) {
                sys.attr("path").attr("append")(path);
            }
        }
        
        // --- FIX: Import the module from its new, correct package ---
        auto sts_driver_module = py::module_::import("sts_driver_py.sts_driver");
        driver_ = sts_driver_module.attr("ServoController")(serial_port_, baudrate_);
        RCLCPP_INFO(rclcpp::get_logger("MarkIIHardwareInterface"), "Python driver loaded and connected successfully.");

        py::list all_servo_ids;
        for(auto const& [key, val] : single_servo_joints_) all_servo_ids.append(val);
        for(auto const& [key, val] : dual_servo_joints_) {
            all_servo_ids.append(val[0]);
            all_servo_ids.append(val[1]);
        }
        for (const auto id : all_servo_ids) {
            driver_.attr("set_torque_enable")(id, true);
        }
        RCLCPP_INFO(rclcpp::get_logger("MarkIIHardwareInterface"), "Torque enabled on all servos.");

        python_driver_initialized_ = true;

    } catch (const py::error_already_set& e) {
        RCLCPP_ERROR(rclcpp::get_logger("MarkIIHardwareInterface"), "Failed to initialize Python driver: %s", e.what());
        // We don't set the flag, so it will retry on the next cycle.
    }
}


hardware_interface::CallbackReturn MarkIIHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("MarkIIHardwareInterface"), "Activating... Please wait.");

    for (size_t i = 0; i < hw_states_.size(); i++)
    {
       if (std::isnan(hw_states_[i])) {
           hw_states_[i] = 0;
           hw_commands_[i] = 0;
       } else {
           hw_commands_[i] = hw_states_[i];
       }
    }

    // --- FIX: Remove all python-related calls from on_activate ---
    
    // The initial read will now handle the first-time initialization.
    if (read(rclcpp::Time(0), rclcpp::Duration(0, 0)) != hardware_interface::return_type::OK)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MarkIIHardwareInterface"), "Initial read failed during activation.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    hw_commands_ = hw_states_;

    RCLCPP_INFO(rclcpp::get_logger("MarkIIHardwareInterface"), "System successfully activated.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MarkIIHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("MarkIIHardwareInterface"), "Deactivating... Please wait.");
    
    py::gil_scoped_acquire acquire;

    // --- FIX: Only run deactivation logic if the driver was actually initialized ---
    if (python_driver_initialized_) {
        try {
            py::list all_servo_ids;
            for(auto const& [key, val] : single_servo_joints_) all_servo_ids.append(val);
            for(auto const& [key, val] : dual_servo_joints_) {
                all_servo_ids.append(val[0]);
                all_servo_ids.append(val[1]);
            }
            for (const auto id : all_servo_ids) {
                driver_.attr("set_torque_enable")(id, false);
            }
            driver_.attr("close")();
            
        } catch (const py::error_already_set& e) {
            RCLCPP_ERROR(rclcpp::get_logger("MarkIIHardwareInterface"), "Python error on deactivation: %s", e.what());
        }
        python_driver_initialized_ = false; // Reset for next activation
    }
    RCLCPP_INFO(rclcpp::get_logger("MarkIIHardwareInterface"), "System successfully deactivated.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// --- The read() and write() functions are already correct, as they use gil_scoped_acquire ---
hardware_interface::return_type MarkIIHardwareInterface::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  py::gil_scoped_acquire acquire;
  
  // --- FIX: Ensure driver is initialized before use ---
  _initialize_python_driver();
  if (!python_driver_initialized_) {
      return hardware_interface::return_type::ERROR; // Don't proceed if init failed
  }

  try {
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      const auto& joint_name = info_.joints[i].name;

      if (joint_name == "link7_Slider-9") {
          for (size_t j = 0; j < info_.joints.size(); ++j) {
              if (info_.joints[j].name == "link6_Slider-8") {
                  hw_states_[i] = hw_states_[j];
                  break;
              }
          }
          continue;
      }

      py::object result;

      if (single_servo_joints_.count(joint_name)) {
        int servo_id = single_servo_joints_.at(joint_name);
        result = driver_.attr("get_position")(servo_id);
      } else if (dual_servo_joints_.count(joint_name)) {
        const auto& ids = dual_servo_joints_.at(joint_name);
        result = driver_.attr("get_position")(ids[0]);
      } else {
          continue;
      }

      if (result.is_none()) {
        RCLCPP_WARN(
          rclcpp::get_logger("MarkIIHardwareInterface"),
          "Failed to read position for joint %s. Skipping update.",
          joint_name.c_str());
        continue;
      }
      int ticks = result.cast<int>();

      if (joint_name == "link6_Slider-8") {
        hw_states_[i] = _convert_ticks_to_gripper_dist(ticks);
      } else {
        hw_states_[i] = _convert_ticks_to_rad(ticks, joint_name);
      }
    }
  } catch (const pybind11::error_already_set & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("MarkIIHardwareInterface"),
      "Exception in MarkIIHardwareInterface::read(): %s",
      ex.what());
    PyErr_Clear();
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MarkIIHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    py::gil_scoped_acquire acquire;

    // --- FIX: Ensure driver is initialized before use ---
    _initialize_python_driver();
    if (!python_driver_initialized_) {
        return hardware_interface::return_type::ERROR; // Don't proceed if init failed
    }

    try
    {
        py::dict commands_to_sync;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            const auto& joint_name = info_.joints[i].name;
            if (std::isnan(hw_commands_[i]))
            {
                continue;
            }

            int position_ticks = 0;
            if (joint_name == "link6_Slider-8") {
                position_ticks = _convert_gripper_dist_to_ticks(hw_commands_[i]);
            } else if (calibration_.count(joint_name)) {
                position_ticks = _convert_rad_to_ticks(hw_commands_[i], joint_name);
            } else {
                continue;
            }

            if (single_servo_joints_.count(joint_name)) {
                commands_to_sync[py::cast(single_servo_joints_.at(joint_name))] = position_ticks;
            } else if (dual_servo_joints_.count(joint_name)) {
                const auto& ids = dual_servo_joints_.at(joint_name);
                commands_to_sync[py::cast(ids[0])] = position_ticks;
                commands_to_sync[py::cast(ids[1])] = 4095 - position_ticks;
            }
        }
        if (!commands_to_sync.empty())
        {
            driver_.attr("sync_write_positions")(commands_to_sync);
        }
    }
    catch(const pybind11::error_already_set &e)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("MarkIIHardwareInterface"),
            "Exception in MarkIIHardwareInterface::write(): %s",
            e.what());
        PyErr_Clear();
        return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

double MarkIIHardwareInterface::_convert_ticks_to_rad(int ticks, const std::string &joint_name) {
    auto cal = calibration_.at(joint_name);
    double rad = (static_cast<double>(ticks) - cal.at("offset")) / cal.at("scale");
    if (joint_name == "base_link_Revolute-1") {
        rad /= 3.0;
    }
    return rad;
}

int MarkIIHardwareInterface::_convert_rad_to_ticks(double rad, const std::string &joint_name) {
    if (joint_name == "base_link_Revolute-1") {
        rad *= 3.0;
    }
    auto cal = calibration_.at(joint_name);
    return static_cast<int>(cal.at("offset") + rad * cal.at("scale"));
}

double MarkIIHardwareInterface::_convert_ticks_to_gripper_dist(int ticks) {
    double min_dist = 0.0, max_dist = 0.024;
    int min_ticks = 2903, max_ticks = 1518;
    double position = min_dist + (static_cast<double>(ticks - min_ticks) / (max_ticks - min_ticks)) * (max_dist - min_dist);
    return std::max(min_dist, std::min(position, max_dist));
}

int MarkIIHardwareInterface::_convert_gripper_dist_to_ticks(double dist) {
    double min_dist = 0.0, max_dist = 0.024;
    int min_ticks = 2903, max_ticks = 1518;
    dist = std::max(min_dist, std::min(dist, max_dist));
    return static_cast<int>(min_ticks + ((dist - min_dist) / (max_dist - min_dist)) * (max_ticks - min_ticks));
}


} // namespace mark_ii_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    mark_ii_hardware_interface::MarkIIHardwareInterface,
    hardware_interface::SystemInterface)
