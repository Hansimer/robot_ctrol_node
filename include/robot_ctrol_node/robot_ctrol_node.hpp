#ifndef ROBOT_CTROL_NODE__ROBOT_CTROL_NODE_HPP_
#define ROBOT_CTROL_NODE__ROBOT_CTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <my_interfaces/srv/srv_move_axis.hpp>
#include <my_interfaces/action/act_motor_cmd.hpp>
#include <robot_controller/action/arm_motion.hpp>
#include <robot_controller/srv/execute_command.hpp>
#include <ethercat_control/msg/bringup_health.hpp>

#include <memory>
#include <map>
#include <string>
#include <vector>
#include <functional>
#include <sstream>
#include <ctime>
#include <iomanip>
#include "../include/datedef.hpp"
#include "../getparxml.hpp"
#include "modbustcp_server.hpp"
#include "transform.hpp"

using namespace std;
namespace robot_ctrol_node
{

// ==================== 类型别名 ====================
using ArmMotion = robot_controller::action::ArmMotion;
using ArmGoalHandle = rclcpp_action::ClientGoalHandle<ArmMotion>;
using MotorCmd = my_interfaces::action::ActMotorCmd;
using GoalHandleMotor = rclcpp_action::ClientGoalHandle<MotorCmd>;

// ==================== 机械臂关节话题订阅模块 ====================
struct TopicJoint_arm
{
  /// @brief 初始化机械臂关节话题订阅器
  /// @param node 节点原始指针（生命周期由 RobotCtrol 保证）
  void init(rclcpp::Node * node);

  /// @brief /joint_states 话题回调
  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // 成员变量
  rclcpp::Node * node_ = nullptr;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

  std::map<std::string,joint_state_> InfoJoint_arm; //保存数据


};

// ==================== 升降伺服话题订阅模块 ====================
struct TopicJoint_lift
{
  /// @brief 初始化升降伺服话题订阅器
  /// @param node 节点原始指针（生命周期由 RobotCtrol 保证）
  void init(rclcpp::Node * node);

  /// @brief /joint_state/lift_servo 话题回调
  void lift_servo_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // 成员变量
  rclcpp::Node * node_ = nullptr;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr lift_servo_sub_;
  joint_state_ InfoJoint_lift;
};

// ==================== arm话题订阅模块 ====================
struct TopicHealth_arms
{
  /// @brief 初始化升降伺服话题订阅器
  /// @param node 节点原始指针（生命周期由 RobotCtrol 保证）
  void init(rclcpp::Node * node);

  /// @brief /joint_state/lift_servo 话题回调
  void health_arms_callback(const ethercat_control::msg::BringupHealth msg);

  // 成员变量
  rclcpp::Node * node_ = nullptr;
  rclcpp::Subscription<ethercat_control::msg::BringupHealth>::SharedPtr health_arms_sub_;
  arms_health_ InfoHealth_arms;
};


// ==================== ExecuteCommand 服务客户端模块 ====================
using ExecArmsSrvCmd = robot_controller::srv::ExecuteCommand;

struct ServiceExecCmd_arms
{
  /// @brief 初始化 ExecuteCommand 服务客户端
  /// @param node 节点原始指针（生命周期由 RobotCtrol 保证）
  void init(rclcpp::Node * node);

  /// @brief 根据命令配置构建命令字符串
  /// @param config 命令配置结构体
  /// @return 构建好的命令字符串，可直接传给 send_command
  static std::string build_command(const exec_cmd_config_ & config);

  /// @brief 基于配置发送命令（内部调用 build_command + send_command，使用成员变量 cmd_config_）
  /// @return 是否发送成功
  bool send_command_from_config();

  /// @brief 发送执行命令
  /// @param command 命令字符串
  /// @param wait_for_result 是否等待结果
  /// @param timeout 超时时间（秒）
  /// @return 是否发送成功
  bool send_command(const std::string & command, bool wait_for_result = true, double timeout = 30.0);

  /// @brief 服务响应回调
  void handle_response(rclcpp::Client<ExecArmsSrvCmd>::SharedFuture future);

  // 成员变量
  rclcpp::Node * node_ = nullptr;
  rclcpp::Client<ExecArmsSrvCmd>::SharedPtr exec_cmd_client_;

  // 命令配置（供外部设置，send_command_from_config 使用）
  exec_cmd_config_ cmd_config_;
  // 最近一次服务调用结果
  exec_ArmsMoveCmd_info_ last_result_;
  bool last_call_success_ = false;
};

// ==================== 服务端模块 ====================
struct ServiceTest_MoveAxis
{
  /// @brief 初始化服务端
  /// @param node 节点原始指针（生命周期由 RobotCtrol 保证）
  void init(rclcpp::Node * node);

  /// @brief move_axis 服务回调
  void move_axis_service_callback(
    const std::shared_ptr<my_interfaces::srv::SrvMoveAxis::Request> request,
    std::shared_ptr<my_interfaces::srv::SrvMoveAxis::Response> response);

  // 成员变量
  rclcpp::Node * node_ = nullptr;
  rclcpp::Service<my_interfaces::srv::SrvMoveAxis>::SharedPtr move_axis_service_;
};

// ==================== ArmMotion Action 客户端模块 ====================
struct ActMotion_arm
{
  /// @brief 初始化 ArmMotion Action 客户端
  /// @param node 节点原始指针（生命周期由 RobotCtrol 保证）
  void init(rclcpp::Node * node);

  /// @brief 发送机械臂关节运动目标 (ArmMotion action)
  /// @param group      运动组名称，如 "dual_arms"
  /// @param joint_names 关节名称列表
  /// @param joint_positions 关节目标位置
  /// @param execute    是否立即执行
  /// @param timeout    超时时间
  /// @return 是否发送成功
  bool send_arm_joint_motion(
    const std::string & group,
    const std::vector<std::string> & joint_names,
    const std::vector<double> & joint_positions,
    bool execute,
    double timeout = 10.0);

  /// @brief ArmMotion 反馈回调
  void arm_feedback_callback(
    std::shared_ptr<ArmGoalHandle> goal_handle,
    const std::shared_ptr<const ArmMotion::Feedback> feedback);

  /// @brief ArmMotion 结果回调
  void arm_result_callback(const ArmGoalHandle::WrappedResult & result);

  // 成员变量
  rclcpp::Node * node_ = nullptr;
  rclcpp_action::Client<ArmMotion>::SharedPtr arm_action_client_;
  bool arm_motion_finished_;
  bool arm_motion_success_;
};

// ==================== MotorCmd Action 客户端模块 ====================
struct ActMotion_lift
{
  /// @brief 初始化 MotorCmd Action 客户端
  /// @param node 节点原始指针（生命周期由 RobotCtrol 保证）
  void init(rclcpp::Node * node);

  /// @brief 发送伺服电机运动目标 (ActMotorCmd action)
  /// @param target_pos 目标位置
  /// @param vel        目标速度
  /// @param acc        加速度
  /// @param timeout    超时时间
  /// @return 是否发送成功
  bool send_motor_pos_cmd(
    double target_pos,
    double vel = 1.0,
    double acc = 0.5,
    double timeout = 10.0);

  /// @brief MotorCmd 反馈回调
  void motor_feedback_callback(
    std::shared_ptr<GoalHandleMotor> goal_handle,
    const std::shared_ptr<const MotorCmd::Feedback> feedback);

  /// @brief MotorCmd 结果回调
  void motor_result_callback(const GoalHandleMotor::WrappedResult & result);

  // 成员变量
  rclcpp::Node * node_ = nullptr;
  rclcpp_action::Client<MotorCmd>::SharedPtr motor_action_client_;
};

// ==================== 主节点类 ====================
class RobotCtrol : public rclcpp::Node
{
public:
  explicit RobotCtrol(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RobotCtrol() = default;
  bool init();
  void run();

private:
  /// @brief 10ms 周期定时器回调任务
  void taskpool();
  void updata_pos_mutex(); //互斥获取位置信息

  bool bis_stop;

  rclcpp::TimerBase::SharedPtr timer_taskpool_;  ///< 10ms 周期定时器


  TopicJoint_arm     topic_arm_module_;
  TopicJoint_lift    topic_lift_module_;
  TopicHealth_arms   topic_health_arms_module_;
  ServiceTest_MoveAxis   service_testAxis_;
  ServiceExecCmd_arms     service_execdmd_arms_;
  ActMotion_arm Act_arm_motion_;
  ActMotion_lift  Act_liftmotor_motion_;
  std::shared_ptr<ModbusTcpServerCpp> pobj_mdtcpserver;

  getparxml param_robot; //获取设备参数
  std::shared_ptr<mdreg_data> pobj_mdpar;

//定义互斥变量
  std::mutex mutex_arm_data,mutex_liftservo_data,mutex_handlservo_data,
  mutex_handrservo_data,mutex_rotatorservo_data,mutex_headservo_data;
  
  

};

}  // namespace robot_ctrol_node

#endif  // ROBOT_CTROL_NODE__ROBOT_CTROL_NODE_HPP_