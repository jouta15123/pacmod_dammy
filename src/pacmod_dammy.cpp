#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/engage.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>
#include <pacmod3_msgs/msg/system_rpt_float.hpp>
#include <pacmod3_msgs/msg/wheel_speed_rpt.hpp>
#include <pacmod3_msgs/msg/system_rpt_int.hpp>
#include <pacmod3_msgs/msg/global_rpt.hpp>
#include <cmath>

class AutowareToPacmodDummyPublisher : public rclcpp::Node
{
public:
  AutowareToPacmodDummyPublisher() : Node("autoware_to_pacmod_dummy_publisher")
  {
    // Autowareからの入力トピックのサブスクライバーの初期化
    control_cmd_sub_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
        "/control/command/control_cmd", 10,
        std::bind(&AutowareToPacmodDummyPublisher::controlCmdCallback, this, std::placeholders::_1));

    gear_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::GearCommand>(
        "/control/command/gear_cmd", 10,
        std::bind(&AutowareToPacmodDummyPublisher::gearCmdCallback, this, std::placeholders::_1));

    turn_indicators_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>(
        "/control/command/turn_indicators_cmd", 10,
        std::bind(&AutowareToPacmodDummyPublisher::turnIndicatorsCmdCallback, this, std::placeholders::_1));

    hazard_lights_cmd_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>(
        "/control/command/hazard_lights_cmd", 10,
        std::bind(&AutowareToPacmodDummyPublisher::hazardLightsCmdCallback, this, std::placeholders::_1));

    // engage_sub_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::Engage>(
    //     "/vehicle/engage", 10, std::bind(&AutowareToPacmodDummyPublisher::engageCallback, this, std::placeholders::_1));

    actuation_cmd_sub_ = this->create_subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>(
        "/control/command/actuation_cmd", 10,
        std::bind(&AutowareToPacmodDummyPublisher::actuationCmdCallback, this, std::placeholders::_1));

    emergency_cmd_sub_ = this->create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
        "/control/command/emergency_cmd", 10,
        std::bind(&AutowareToPacmodDummyPublisher::emergencyCmdCallback, this, std::placeholders::_1));

    // Pacmodトピックのパブリッシャーの初期化
    steering_rpt_pub_ = this->create_publisher<pacmod3_msgs::msg::SystemRptFloat>("/pacmod/steering_rpt", 10);
    wheel_speed_rpt_pub_ = this->create_publisher<pacmod3_msgs::msg::WheelSpeedRpt>("/pacmod/wheel_speed_rpt", 10);
    accel_rpt_pub_ = this->create_publisher<pacmod3_msgs::msg::SystemRptFloat>("/pacmod/accel_rpt", 10);
    brake_rpt_pub_ = this->create_publisher<pacmod3_msgs::msg::SystemRptFloat>("/pacmod/brake_rpt", 10);
    shift_rpt_pub_ = this->create_publisher<pacmod3_msgs::msg::SystemRptInt>("/pacmod/shift_rpt", 10);
    turn_rpt_pub_ = this->create_publisher<pacmod3_msgs::msg::SystemRptInt>("/pacmod/turn_rpt", 10);
    global_rpt_pub_ = this->create_publisher<pacmod3_msgs::msg::GlobalRpt>("/pacmod/global_rpt", 10);
  }

private:
  void controlCmdCallback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received control command.");
    // ステアリング角度をAutowareからPacmodに変換（ラジアン）
    auto steering_rpt_msg = pacmod3_msgs::msg::SystemRptFloat();
    steering_rpt_msg.header.stamp = this->get_clock()->now();
    steering_rpt_msg.enabled = true;
    steering_rpt_msg.command = msg->lateral.steering_tire_angle;
    steering_rpt_msg.output = msg->lateral.steering_tire_angle;
    steering_rpt_pub_->publish(steering_rpt_msg);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Published steering command.");


    // 車速をAutowareからPacmodに変換（ホイール速度）
    auto wheel_speed_rpt_msg = pacmod3_msgs::msg::WheelSpeedRpt();
    wheel_speed_rpt_msg.header.stamp = this->get_clock()->now();
    double wheel_speed = msg->longitudinal.speed / tire_radius_;
    wheel_speed_rpt_msg.front_left_wheel_speed = wheel_speed;
    wheel_speed_rpt_msg.front_right_wheel_speed = wheel_speed;
    wheel_speed_rpt_msg.rear_left_wheel_speed = wheel_speed;
    wheel_speed_rpt_msg.rear_right_wheel_speed = wheel_speed;
    wheel_speed_rpt_pub_->publish(wheel_speed_rpt_msg);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Published wheel speed command.");
  }

  void gearCmdCallback(const autoware_auto_vehicle_msgs::msg::GearCommand::SharedPtr msg)
  {
    // ギアをAutowareからPacmodに変換
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received gear command.");
    auto shift_rpt_msg = pacmod3_msgs::msg::SystemRptInt();
    shift_rpt_msg.header.stamp = this->get_clock()->now();
    shift_rpt_msg.enabled = true;
    switch (msg->command)
    {
      case autoware_auto_vehicle_msgs::msg::GearCommand::PARK:
        shift_rpt_msg.output = pacmod3_msgs::msg::SystemRptInt::SHIFT_PARK;
        break;
      case autoware_auto_vehicle_msgs::msg::GearCommand::REVERSE:
        shift_rpt_msg.output = pacmod3_msgs::msg::SystemRptInt::SHIFT_REVERSE;
        break;
      case autoware_auto_vehicle_msgs::msg::GearCommand::DRIVE:
        shift_rpt_msg.output = pacmod3_msgs::msg::SystemRptInt::SHIFT_FORWARD;
        break;
      default:
        shift_rpt_msg.output = pacmod3_msgs::msg::SystemRptInt::SHIFT_PARK;
    }
    shift_rpt_pub_->publish(shift_rpt_msg);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Published shift command.");
  }

  void turnIndicatorsCmdCallback(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::SharedPtr msg)
  {
    // ウィンカーをAutowareからPacmodに変換
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received turn indicators command.");
    auto turn_rpt_msg = pacmod3_msgs::msg::SystemRptInt();
    turn_rpt_msg.header.stamp = this->get_clock()->now();
    turn_rpt_msg.enabled = true;
    switch (msg->command)
    {
      case autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::NO_COMMAND:
        turn_rpt_msg.output = pacmod3_msgs::msg::SystemRptInt::TURN_NONE;
        break;
      case autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_LEFT:
        turn_rpt_msg.output = pacmod3_msgs::msg::SystemRptInt::TURN_LEFT;
        break;
      case autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand::ENABLE_RIGHT:
        turn_rpt_msg.output = pacmod3_msgs::msg::SystemRptInt::TURN_RIGHT;
        break;
      default:
        turn_rpt_msg.output = pacmod3_msgs::msg::SystemRptInt::TURN_NONE;
    }
    turn_rpt_pub_->publish(turn_rpt_msg);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Published turn indicators command.");
  }

  void hazardLightsCmdCallback(const autoware_auto_vehicle_msgs::msg::HazardLightsCommand::SharedPtr msg)
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received hazard lights command.");
    auto global_rpt_msg = pacmod3_msgs::msg::GlobalRpt();
    global_rpt_msg.header.stamp = this->get_clock()->now();
    global_rpt_msg.override_active = msg->command == autoware_auto_vehicle_msgs::msg::HazardLightsCommand::ENABLE;
    global_rpt_pub_->publish(global_rpt_msg);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Published hazard lights command.");
  }

//   void engageCallback(const autoware_auto_vehicle_msgs::msg::Engage::SharedPtr msg)
//   {
//     // エンゲージ状態に基づいて、Pacmodトピックの有効/無効を設定
//     RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received engage command.");
//     auto steering_rpt_msg = pacmod3_msgs::msg::SystemRptFloat();
//     steering_rpt_msg.header.stamp = this->get_clock()->now();
//     steering_rpt_msg.enabled = msg->engage;
//     steering_rpt_pub_->publish(steering_rpt_msg);

//     auto accel_rpt_msg = pacmod3_msgs::msg::SystemRptFloat();
//     accel_rpt_msg.header.stamp = this->get_clock()->now();
//     accel_rpt_msg.enabled = msg->engage;
//     accel_rpt_pub_->publish(accel_rpt_msg);

//     auto brake_rpt_msg = pacmod3_msgs::msg::SystemRptFloat();
//     brake_rpt_msg.header.stamp = this->get_clock()->now();
//     brake_rpt_msg.enabled = msg->engage;
//     brake_rpt_pub_->publish(brake_rpt_msg);

//     auto shift_rpt_msg = pacmod3_msgs::msg::SystemRptInt();
//     shift_rpt_msg.header.stamp = this->get_clock()->now();
//     shift_rpt_msg.enabled = msg->engage;
//     shift_rpt_pub_->publish(shift_rpt_msg);

//     auto turn_rpt_msg = pacmod3_msgs::msg::SystemRptInt();
//     turn_rpt_msg.header.stamp = this->get_clock()->now();
//     turn_rpt_msg.enabled = msg->engage;
//     turn_rpt_pub_->publish(turn_rpt_msg);
//   }

  void actuationCmdCallback(const tier4_vehicle_msgs::msg::ActuationCommandStamped::SharedPtr msg)
  {
    // アクセルペダルをAutowareからPacmodに変換（パーセント）
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received actuation command.");
    auto accel_rpt_msg = pacmod3_msgs::msg::SystemRptFloat();
    accel_rpt_msg.header.stamp = this->get_clock()->now();
    accel_rpt_msg.enabled = true;
    accel_rpt_msg.command = msg->actuation.accel_cmd * 100.0;
    accel_rpt_msg.output = msg->actuation.accel_cmd * 100.0;
    accel_rpt_pub_->publish(accel_rpt_msg);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Published accel command.");

    // ブレーキペダルをAutowareからPacmodに変換（パーセント）
    auto brake_rpt_msg = pacmod3_msgs::msg::SystemRptFloat();
    brake_rpt_msg.header.stamp = this->get_clock()->now();
    brake_rpt_msg.enabled = true;
    brake_rpt_msg.command = msg->actuation.brake_cmd * 100.0;
    brake_rpt_msg.output = msg->actuation.brake_cmd * 100.0;
    brake_rpt_pub_->publish(brake_rpt_msg);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Published brake command.");
  }

  void emergencyCmdCallback(const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::SharedPtr msg)
  {
    // 緊急コマンドに基づいて、Pacmodトピックの有効/無効を設定
    auto global_rpt_msg = pacmod3_msgs::msg::GlobalRpt();
    global_rpt_msg.header.stamp = this->get_clock()->now();
    global_rpt_msg.enabled = !msg->emergency;
    global_rpt_pub_->publish(global_rpt_msg);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Published emergency command.");
  }

  // サブスクライバー
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr control_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr turn_indicators_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr hazard_lights_cmd_sub_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::Engage>::SharedPtr engage_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::ActuationCommandStamped>::SharedPtr actuation_cmd_sub_;
  rclcpp::Subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr emergency_cmd_sub_;

  // パブリッシャー
  rclcpp::Publisher<pacmod3_msgs::msg::SystemRptFloat>::SharedPtr steering_rpt_pub_;
  rclcpp::Publisher<pacmod3_msgs::msg::WheelSpeedRpt>::SharedPtr wheel_speed_rpt_pub_;
  rclcpp::Publisher<pacmod3_msgs::msg::SystemRptFloat>::SharedPtr accel_rpt_pub_;
  rclcpp::Publisher<pacmod3_msgs::msg::SystemRptFloat>::SharedPtr brake_rpt_pub_;
  rclcpp::Publisher<pacmod3_msgs::msg::SystemRptInt>::SharedPtr shift_rpt_pub_;
  rclcpp::Publisher<pacmod3_msgs::msg::SystemRptInt>::SharedPtr turn_rpt_pub_;
  rclcpp::Publisher<pacmod3_msgs::msg::GlobalRpt>::SharedPtr global_rpt_pub_;

  // 車両パラメータ
  double tire_radius_ = 0.35;  // タイヤ半径（例）
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutowareToPacmodDummyPublisher>());
  rclcpp::shutdown();
  return 0;
}