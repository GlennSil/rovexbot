#include "rovexbot_firmware/rovexbot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>


namespace rovexbot_firmware
{
RovexbotInterface::RovexbotInterface()
{
}


RovexbotInterface::~RovexbotInterface()
{
  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("RovexbotInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }
}

CallbackReturn RovexbotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  try
  {
    port_ = info_.hardware_parameters.at("port");
  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("RovexbotInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  // resize (et non reserve) : les vecteurs ont vraiment une case par roue
  velocity_commands_.resize(info_.joints.size(), 0.0);
  position_states_.resize(info_.joints.size(), 0.0);
  velocity_states_.resize(info_.joints.size(), 0.0);
  last_run_ = rclcpp::Clock().now();

  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> RovexbotInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> RovexbotInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }

  return command_interfaces;
}


CallbackReturn RovexbotInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("RovexbotInterface"), "Starting robot hardware ...");

  // Remise à zéro SANS réallouer les vecteurs (les pointeurs exportés restent valides)
  std::fill(velocity_commands_.begin(), velocity_commands_.end(), 0.0);
  std::fill(position_states_.begin(), position_states_.end(), 0.0);
  std::fill(velocity_states_.begin(), velocity_states_.end(), 0.0);

  try
  {
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("RovexbotInterface"),
                        "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }

  last_run_ = rclcpp::Clock().now();
  RCLCPP_INFO(rclcpp::get_logger("RovexbotInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}


CallbackReturn RovexbotInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("RovexbotInterface"), "Stopping robot hardware ...");

  if (arduino_.IsOpen())
  {
    try

    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("RovexbotInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("RovexbotInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}


hardware_interface::return_type RovexbotInterface::read(const rclcpp::Time &,
                                                        const rclcpp::Duration &)
{
  // Format attendu : "rp04.73,ln02.10,\r\n"
  if (arduino_.IsDataAvailable())
  {
    auto dt = (rclcpp::Clock().now() - last_run_).seconds();
    std::string message;
    arduino_.ReadLine(message);
    std::stringstream ss(message);
    std::string res;

    while (std::getline(ss, res, ','))
    {
      // Ignore les morceaux trop courts ("\r\n", ligne tronquée...)
      if (res.size() < 3)
      {
        continue;
      }

      try
      {
        int multiplier = res.at(1) == 'p' ? 1 : -1;
        double value = multiplier * std::stod(res.substr(2));

        if (res.at(0) == 'r')
        {
          velocity_states_.at(0) = value;
          position_states_.at(0) += value * dt;
        }
        else if (res.at(0) == 'l')
        {
          velocity_states_.at(1) = value;
          position_states_.at(1) += value * dt;
        }
      }
      catch (const std::exception &e)
      {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("RovexbotInterface"),
                           "Malformed serial chunk ignored: '" << res << "'");
      }
    }
    last_run_ = rclcpp::Clock().now();
  }
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type RovexbotInterface::write(const rclcpp::Time &,
                                                         const rclcpp::Duration &)
{
  // Format envoyé : "rp05.00,ln03.20,"  (largeur fixe pour faciliter le parsing côté Arduino)
  std::stringstream message_stream;
  char right_wheel_sign = velocity_commands_.at(0) >= 0 ? 'p' : 'n';
  char left_wheel_sign = velocity_commands_.at(1) >= 0 ? 'p' : 'n';
  std::string compensate_zeros_right = std::abs(velocity_commands_.at(0)) < 10.0 ? "0" : "";
  std::string compensate_zeros_left = std::abs(velocity_commands_.at(1)) < 10.0 ? "0" : "";

  message_stream << std::fixed << std::setprecision(2)
                 << "r" << right_wheel_sign << compensate_zeros_right << std::abs(velocity_commands_.at(0))
                 << ",l" << left_wheel_sign << compensate_zeros_left << std::abs(velocity_commands_.at(1))
                 << ",";

  try
  {
    arduino_.Write(message_stream.str());
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("RovexbotInterface"),
                        "Something went wrong while sending the message "
                            << message_stream.str() << " to the port " << port_);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}
}  // namespace rovexbot_firmware

PLUGINLIB_EXPORT_CLASS(rovexbot_firmware::RovexbotInterface, hardware_interface::SystemInterface)