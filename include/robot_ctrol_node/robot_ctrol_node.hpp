#ifndef ROBOT_CTROL_NODE__ROBOT_CTROL_NODE_HPP_
#define ROBOT_CTROL_NODE__ROBOT_CTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <my_interfaces/msg/msg_joint_state_cmd.hpp>
#include <my_interfaces/srv/srv_move_axis.hpp>
#include <my_interfaces/srv/srv_servo_cmd.hpp>
#include <my_interfaces/action/act_motor_cmd.hpp>
#include <robot_controller/action/arm_motion.hpp>
#include <robot_controller/srv/execute_command.hpp>
#include <ethercat_control/msg/bringup_health.hpp>
#include <robot_controller/msg/command_result.hpp>
#include <robot_controller/msg/end_effector_pose.hpp>

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

#include <mutex>
#include <condition_variable>
#include <thread>
#include <chrono>

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
{// @brief 初始化机械臂关节话题订阅器
  /// @param node 节点原始指针（生命周期由 RobotCtrol 保证）
  void init(rclcpp::Node * node);

  /// @brief /joint_states 话题回调
  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // 成员变量
  rclcpp::Node * node_ = nullptr;
  std::mutex mutex_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

  std::map<std::string,joint_state_> InfoJoint_arm; //保存数据


};

// ==================== 升降伺服话题订阅模块 ====================
struct TopicJoint_lift
{
  /// @brief 初始化升降伺服话题订阅器
  /// @param node 节点原始指针（生命周期由 RobotCtrol 保证）mutex_进行数据互锁
  void init(rclcpp::Node * node );

  /// @brief /joint_state/lift_servo 话题回调
  void lift_servo_callback(const my_interfaces::msg::MsgJointStateCmd::SharedPtr msg);

  // 成员变量
  rclcpp::Node * node_ = nullptr;
  std::mutex mutex_;
  rclcpp::Subscription<my_interfaces::msg::MsgJointStateCmd>::SharedPtr lift_servo_sub_;
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
  std::mutex mutex_; //同步信号
  bool last_call_success_ = false;
};

// ==================== lift 服务客户端模块 ====================
using SrvServoCmd = my_interfaces::srv::SrvServoCmd;

struct ServiceExecCmd_lift
{
  /// @brief 初始化 SrvServoCmd 服务客户端
  /// @param node 节点原始指针（生命周期由 RobotCtrol 保证）
  void init(rclcpp::Node * node);

  /// @brief 发送伺服命令（同步等待结果）
  /// @param req_config 请求配置结构体
  /// @param timeout  超时时间（秒）
  /// @return 是否发送成功并收到响应
  bool send_command(const srv_servo_cmd_request_ & req_config, uint8_t timeout = 5);

  /// @brief 使用成员变量 cmd_request_ 发送命令
  /// @param timeout  超时时间（秒）
  /// @return 是否发送成功并收到响应
  bool send_command_from_config(double timeout = 5.0);

  /// @brief 服务响应回调
  void handle_response(rclcpp::Client<SrvServoCmd>::SharedFuture future);

  // 成员变量
  rclcpp::Node * node_ = nullptr;
  rclcpp::Client<SrvServoCmd>::SharedPtr srv_servo_cmd_client_;

  std::mutex mutex_; //同步信号

  // 请求配置（供外部设置，send_command_from_config 使用）
  srv_servo_cmd_request_ cmd_request_;
  // 最近一次服务调用结果
  srv_servo_cmd_response_ last_result_;
};


// ==================== arm_motion_controller 结果话题订阅模块 ====================
struct TopicArmMotionResult
{
  /// @brief 初始化 arm_motion_controller/result 话题订阅器
  /// @param node 节点原始指针（生命周期由 RobotCtrol 保证）
  void init(rclcpp::Node * node);

  /// @brief /arm_motion_controller/result 话题回调
  void result_callback(const robot_controller::msg::CommandResult::SharedPtr msg);

  // 成员变量
  rclcpp::Node * node_ = nullptr;
  std::mutex mutex_;
  rclcpp::Subscription<robot_controller::msg::CommandResult>::SharedPtr result_sub_;

  arm_motion_result_info_ InfoArmMotionResult;  // 保存数据
};

// ==================== 左手末端位姿话题订阅模块 ====================
struct TopicEefpos_handleft
{
  /// @brief 初始化左手末端位姿话题订阅器
  /// @param node 节点原始指针（生命周期由 RobotCtrol 保证）
  void init(rclcpp::Node * node);

  /// @brief /eef_pose_publisher/left_hand 话题回调
  void eef_pose_callback(const robot_controller::msg::EndEffectorPose::SharedPtr msg);

  // 成员变量
  rclcpp::Node * node_ = nullptr;
  std::mutex mutex_;
  rclcpp::Subscription<robot_controller::msg::EndEffectorPose>::SharedPtr eef_pose_sub_;

  eef_pose_info_ InfoEefpos_handleft;  // 保存数据
};

// ==================== 右手末端位姿话题订阅模块 ====================
struct TopicEefpos_handright
{
  /// @brief 初始化右手末端位姿话题订阅器
  /// @param node 节点原始指针（生命周期由 RobotCtrol 保证）
  void init(rclcpp::Node * node);

  /// @brief /eef_pose_publisher/right_hand 话题回调
  void eef_pose_callback(const robot_controller::msg::EndEffectorPose::SharedPtr msg);

  // 成员变量
  rclcpp::Node * node_ = nullptr;
  std::mutex mutex_;
  rclcpp::Subscription<robot_controller::msg::EndEffectorPose>::SharedPtr eef_pose_sub_;

  eef_pose_info_ InfoEefpos_handright;  // 保存数据
};

// ==================== 主节点类 ====================
class RobotCtrol : public rclcpp::Node
{
public:
  explicit RobotCtrol(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RobotCtrol();
  bool init();
  void run();

private:
  /// @brief 10ms 周期定时器回调任务
  void taskpool();
  void mode_auto();//自动运行程序
  void mode_manu();//手动运行程序
  bool mode_manu_lift();
  bool mode_manu_arm_rotate_head_hand(const action_info_ &par);
  void monitor();//系统监控程序
  void updata_pos_mutex(); //互斥获取位置信息
  bool decode_action(array_actions_info_ &par);//解析运动指令

  bool bis_stop;

  // rclcpp::TimerBase::SharedPtr timer_taskpool_;  ///< 10ms 周期定时器（已替换为线程）
  std::thread taskpool_thread_;  ///< taskpool 工作线程


  TopicJoint_arm     topic_arm_module_;
  TopicJoint_lift    topic_lift_module_;
  TopicHealth_arms   topic_health_arms_module_;
  TopicArmMotionResult topic_arm_motion_result_module_;
  TopicEefpos_handleft   topic_eefpos_handleft_module_;
  TopicEefpos_handright  topic_eefpos_handright_module_;
  ServiceExecCmd_arms     service_execdmd_arms_;
  ServiceExecCmd_lift      service_srv_servo_cmd_;
  std::shared_ptr<ModbusTcpServerCpp> pobj_mdtcpserver;

  getparxml param_robot; //获取设备参数
  std::shared_ptr<mdreg_data> pobj_mdpar;

  

};

}  // namespace robot_ctrol_node

#endif  // ROBOT_CTROL_NODE__ROBOT_CTROL_NODE_HPP_