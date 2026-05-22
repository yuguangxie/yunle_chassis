#include "chassis_interfaces/ScuControlCommand.h"

#include <ros/ros.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <termios.h>
#include <unistd.h>

namespace
{

std::atomic_bool g_shutdown_requested{false};

void signalHandler(int)
{
  g_shutdown_requested.store(true);
}

/** Manage terminal raw mode so key presses can be read without pressing Enter. */
/** 管理终端 raw 模式，使按键无需回车即可被读取。 */
class TerminalRawMode
{
public:
  TerminalRawMode()
  {
    if (!isatty(STDIN_FILENO)) {
      return;
    }
    if (tcgetattr(STDIN_FILENO, &original_) != 0) {
      return;
    }
    termios raw = original_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
      enabled_ = true;
    }
  }

  ~TerminalRawMode()
  {
    if (enabled_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original_);
    }
  }

  bool readChar(char & c) const
  {
    const ssize_t n = ::read(STDIN_FILENO, &c, 1);
    return n == 1;
  }

  bool enabled() const { return enabled_; }

private:
  termios original_{};
  bool enabled_{false};
};

/** Publish ScuControlCommand messages from keyboard input for manual chassis checkout. */
/** 根据键盘输入发布 ScuControlCommand 消息，用于人工底盘联调检查。 */
class KeyboardScuControlNode
{
public:
  KeyboardScuControlNode()
  : pnh_("~")
  {
    pnh_.param<std::string>("topic_name", topic_name_, "/yunle_chassis/control/scu_control_command");
    pnh_.param<double>("publish_rate_hz", publish_rate_hz_, 20.0);
    pnh_.param<double>("speed_step_kmh", speed_step_kmh_, 0.5);
    pnh_.param<double>("default_speed_kmh", default_speed_kmh_, 1.0);
    pnh_.param<double>("max_speed_kmh", max_speed_kmh_, 15.0);
    pnh_.param<double>("steering_step_deg", steering_step_deg_, 2.0);
    pnh_.param<double>("max_steering_angle_deg", max_steering_angle_deg_, 27.0);
    pnh_.param<double>("rear_steering_ratio", rear_steering_ratio_, 0.0);
    pnh_.param<bool>("auto_publish_zero_on_exit", auto_publish_zero_on_exit_, true);

    publish_rate_hz_ = sanitizePositive(publish_rate_hz_, 20.0);
    speed_step_kmh_ = sanitizePositive(speed_step_kmh_, 0.5);
    max_speed_kmh_ = sanitizePositive(max_speed_kmh_, 15.0);
    default_speed_kmh_ = clamp(default_speed_kmh_, 0.0, max_speed_kmh_);
    steering_step_deg_ = sanitizePositive(steering_step_deg_, 2.0);
    max_steering_angle_deg_ = sanitizePositive(max_steering_angle_deg_, 27.0);

    publisher_ = nh_.advertise<chassis_interfaces::ScuControlCommand>(topic_name_, 10);
    terminal_ = std::make_unique<TerminalRawMode>();
    if (!terminal_->enabled()) {
      ROS_WARN("stdin is not a TTY or raw mode failed; keyboard input may be unavailable");
    }

    resetCommand();
    printHelp();
    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_hz_), [this](const ros::TimerEvent &) { tick(); });
  }

  ~KeyboardScuControlNode()
  {
    if (auto_publish_zero_on_exit_ && ros::ok()) {
      publishZeroCommand(true);
    }
  }

private:
  static double clamp(double value, double low, double high)
  {
    return std::max(low, std::min(value, high));
  }

  static double sanitizePositive(double value, double fallback)
  {
    return std::isfinite(value) && value > 0.0 ? value : fallback;
  }

  void resetCommand()
  {
    command_ = chassis_interfaces::ScuControlCommand{};
    command_.scu_shift_level_request = chassis_interfaces::ScuControlCommand::SHIFT_LEVEL_N;
    command_.scu_steering_angle_front = 0.0F;
    command_.scu_steering_angle_rear = 0.0F;
    command_.scu_target_speed = 0.0F;
    command_.scu_brake_enable = false;
    command_.gw_left_turn_light_request = 0U;
    command_.gw_right_turn_light_request = 0U;
    command_.gw_position_light_request = 0U;
    command_.gw_low_beam_request = 0U;
    command_.scu_torque_or_speed_mode = 0U;
    command_.steering_angle_speed_valid = false;
    command_.brake_force_command_valid = false;
  }

  void tick()
  {
    char key = 0;
    bool handled = false;
    while (terminal_ && terminal_->readChar(key)) {
      handled = handleKey(key) || handled;
    }

    applyRearSteering();
    publisher_.publish(command_);

    if (handled) {
      printState();
    }
    if (g_shutdown_requested.load()) {
      if (auto_publish_zero_on_exit_) {
        publishZeroCommand(true);
      }
      ros::shutdown();
    }
  }

  bool handleKey(char key)
  {
    switch (key) {
      case 'w':
      case 'W':
        command_.scu_brake_enable = false;
        if (command_.scu_target_speed <= 0.0F) {
          command_.scu_target_speed = static_cast<float>(default_speed_kmh_);
        } else {
          adjustSpeed(speed_step_kmh_);
        }
        return true;
      case 's':
      case 'S':
        adjustSpeed(-speed_step_kmh_);
        return true;
      case '1':
        command_.scu_shift_level_request = chassis_interfaces::ScuControlCommand::SHIFT_LEVEL_D;
        return true;
      case '2':
        command_.scu_shift_level_request = chassis_interfaces::ScuControlCommand::SHIFT_LEVEL_N;
        return true;
      case '3':
        command_.scu_shift_level_request = chassis_interfaces::ScuControlCommand::SHIFT_LEVEL_R;
        return true;
      case 'e':
      case 'E':
        adjustSpeed(speed_step_kmh_);
        command_.scu_brake_enable = false;
        return true;
      case 'q':
      case 'Q':
        adjustSpeed(-speed_step_kmh_);
        return true;
      case 'a':
      case 'A':
        adjustSteering(steering_step_deg_);
        return true;
      case 'd':
      case 'D':
        adjustSteering(-steering_step_deg_);
        return true;
      case 'c':
      case 'C':
        command_.scu_steering_angle_front = 0.0F;
        command_.scu_steering_angle_rear = 0.0F;
        return true;
      case 'x':
      case 'X':
        resetCommand();
        return true;
      case ' ':
        command_.scu_target_speed = 0.0F;
        command_.scu_shift_level_request = chassis_interfaces::ScuControlCommand::SHIFT_LEVEL_N;
        command_.scu_brake_enable = true;
        return true;
      case 'b':
      case 'B':
        command_.scu_brake_enable = !command_.scu_brake_enable;
        if (command_.scu_brake_enable) {
          command_.scu_target_speed = 0.0F;
        }
        return true;
      case 'j':
      case 'J':
        command_.gw_left_turn_light_request = toggleRequest(command_.gw_left_turn_light_request);
        return true;
      case 'k':
      case 'K':
        command_.gw_right_turn_light_request = toggleRequest(command_.gw_right_turn_light_request);
        return true;
      case 'u':
      case 'U':
        command_.gw_position_light_request = toggleRequest(command_.gw_position_light_request);
        return true;
      case 'i':
      case 'I':
        command_.gw_low_beam_request = toggleRequest(command_.gw_low_beam_request);
        return true;
      case 'm':
      case 'M':
        command_.scu_torque_or_speed_mode = command_.scu_torque_or_speed_mode == 0U ? 1U : 0U;
        return true;
      case 'v':
      case 'V':
        command_.steering_angle_speed_valid = !command_.steering_angle_speed_valid;
        command_.brake_force_command_valid = command_.steering_angle_speed_valid;
        return true;
      case 'h':
      case 'H':
        printHelp();
        return true;
      case '\x03':
        if (auto_publish_zero_on_exit_) {
          publishZeroCommand(true);
        }
        ros::shutdown();
        return true;
      default:
        return false;
    }
  }

  static uint8_t toggleRequest(uint8_t value)
  {
    return value == 0U ? 1U : 0U;
  }

  void adjustSpeed(double delta)
  {
    const double current = static_cast<double>(command_.scu_target_speed);
    command_.scu_target_speed = static_cast<float>(clamp(current + delta, 0.0, max_speed_kmh_));
  }

  void adjustSteering(double delta)
  {
    const double current = static_cast<double>(command_.scu_steering_angle_front);
    command_.scu_steering_angle_front = static_cast<float>(
      clamp(current + delta, -max_steering_angle_deg_, max_steering_angle_deg_));
  }

  void applyRearSteering()
  {
    const double rear = static_cast<double>(command_.scu_steering_angle_front) * rear_steering_ratio_;
    command_.scu_steering_angle_rear = static_cast<float>(clamp(rear, -max_steering_angle_deg_, max_steering_angle_deg_));
  }

  void publishZeroCommand(bool brake)
  {
    resetCommand();
    command_.scu_brake_enable = brake;
    publisher_.publish(command_);
  }

  std::string shiftName() const
  {
    switch (command_.scu_shift_level_request) {
      case chassis_interfaces::ScuControlCommand::SHIFT_LEVEL_D:
        return "D";
      case chassis_interfaces::ScuControlCommand::SHIFT_LEVEL_R:
        return "R";
      default:
        return "N";
    }
  }

  void printHelp() const
  {
    std::cout
      << "\nKeyboard SCU control keys / 键盘 SCU 控制按键:\n"
      << "  w/s: increase/decrease target speed / 增加/降低目标速度\n"
      << "  1/2/3: select D/N/R shift level / 选择 D/N/R 档位\n"
      << "  q/e: decrease/increase target speed / 降低/提高目标速度\n"
      << "  a/d: steer left/right / 左转/右转\n"
      << "  c: center steering / 转角回正\n"
      << "  x: neutral zero command / N 档零速命令\n"
      << "  SPACE: brake and neutral / N 档制动\n"
      << "  b: toggle brake enable / 切换制动使能\n"
      << "  j/k/u/i: toggle left/right/position/low-beam light request / 切换左转/右转/位置灯/近光灯请求\n"
      << "  m: toggle torque-or-speed mode / 切换扭矩或速度模式\n"
      << "  v: toggle valid flags / 切换有效位\n"
      << "  h: print this help / 打印本帮助\n"
      << "  Ctrl-C: exit / 退出\n"
      << std::endl;
  }

  void printState() const
  {
    std::cout << "\r"
      << "shift=" << shiftName()
      << " speed=" << command_.scu_target_speed << "km/h"
      << " front=" << command_.scu_steering_angle_front << "deg"
      << " rear=" << command_.scu_steering_angle_rear << "deg"
      << " brake=" << (command_.scu_brake_enable ? "on" : "off")
      << " lights(L/R/P/Low)=" << static_cast<int>(command_.gw_left_turn_light_request)
      << "/" << static_cast<int>(command_.gw_right_turn_light_request)
      << "/" << static_cast<int>(command_.gw_position_light_request)
      << "/" << static_cast<int>(command_.gw_low_beam_request)
      << "      " << std::flush;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::string topic_name_;
  double publish_rate_hz_{20.0};
  double speed_step_kmh_{0.5};
  double default_speed_kmh_{1.0};
  double max_speed_kmh_{15.0};
  double steering_step_deg_{2.0};
  double max_steering_angle_deg_{27.0};
  double rear_steering_ratio_{0.0};
  bool auto_publish_zero_on_exit_{true};

  chassis_interfaces::ScuControlCommand command_;
  ros::Publisher publisher_;
  ros::Timer timer_;
  std::unique_ptr<TerminalRawMode> terminal_;
};

}  // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "keyboard_scu_control_node");
  std::signal(SIGINT, signalHandler);
  KeyboardScuControlNode node;
  ros::spin();
  return 0;
}
