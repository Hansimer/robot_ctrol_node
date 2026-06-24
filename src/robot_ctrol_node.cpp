#include "robot_ctrol_node/robot_ctrol_node.hpp"
#include "../include/log.hpp"
#include "../include/datedef.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include "../include/robot_ctrol_node/transform.hpp"

namespace robot_ctrol_node
{

  
#define CODEAXIS_RATIO_RPM 65536
#define RATIO_VEL_DEC_RPM 512.0*CODEAXIS_RATIO_RPM/1875
#define RATIO_ACC_DEC_RPS 65536.0*CODEAXIS_RATIO_RPM/4000000


#define STAWORD_MASK 0x03FF
#define STAWORD_REACH_MASK 0x0637
#define STAWORD_INI 0x0250
#define STAWORD_INI2READY_6BACK 0x0231
#define STAWORD_READY2OPEN_7BACK 0x0233
#define STAWORD_OPEN2RUN_FBACK 0x0237
#define STAWORD_ERR2NO_8BACK 0x0250
#define STAWORD_ERR2STOP 0x021F
#define STAWORD_ERR2ERR 0x0218

    #define SERVO_POWERON 1 
    #define SERVO_POWEROFF 2
    #define SERVO_RSET 3
    #define SERVO_SETVEL 4
    #define SERVO_MVPOS_ABS 5
    #define SERVO_STOP 6

// ======================== 机械臂关节话题订阅模块 ========================
void TopicJoint_arm::init(rclcpp::Node * node,std::mutex* mute)
{
  node_ = node;
  mutex_ = mute;

  // 订阅 /joint_states 话题
  joint_states_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    10,
    std::bind(&TopicJoint_arm::joint_states_callback, this, std::placeholders::_1));
  LOG_INFO("已订阅 /joint_states");
}

void TopicJoint_arm::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  size_t joint_cnt = std::min(msg->name.size(), msg->position.size());
  // LOG_INFO("收到 /joint_states 关节状态，共 %zu 个关节", joint_cnt);
   std::unique_lock<std::mutex> lock1(*mutex_, std::defer_lock);
  if(lock1.try_lock()) //获取到进行数据更新
  {
    for (size_t i = 0; i < joint_cnt; i++)
    {
      const std::string & jname = msg->name[i];
      joint_state_ js;
      js.name = jname;
      js.rad_pose_current = static_cast<float>(msg->position[i]);
      js.rad_vel_current = (i < msg->velocity.size()) ? static_cast<float>(msg->velocity[i]) : 0.0f;
      js.mode = 0;
      js.statues = 0;
      js.error = 0;

      // 保存到 InfoJoint_arm map
      InfoJoint_arm[jname] = js;

      // LOG_INFO("[InfoJoint_arm] %s: pose=%.6f rad, vel=%.6f rad/s",
      //   js.name.c_str(), js.rad_pose_current, js.rad_vel_current);
    }
    lock1.unlock();//解锁
  }


  // 输出当前 map 中所有关节数据
  // LOG_INFO("[InfoJoint_arm] 当前共 %zu 个关节数据:", InfoJoint_arm.size());
  // for (const auto & kv : InfoJoint_arm)
  // {
  //   const auto & js = kv.second;
  //   LOG_INFO("  [%s] pose=%.6f, vel=%.6f", js.name.c_str(), js.rad_pose_current, js.rad_vel_current);
  // }
}

// ======================== 升降伺服话题订阅模块 ========================
void TopicJoint_lift::init(rclcpp::Node * node,std::mutex* mute)
{
  node_ = node;
  mutex_ = mute;

  // 订阅 /joint_state/lift_servo 话题
  lift_servo_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_state/lift_servo",
    10,
    std::bind(&TopicJoint_lift::lift_servo_callback, this, std::placeholders::_1));
  LOG_INFO("已订阅 /joint_state/lift_servo");
}

void TopicJoint_lift::lift_servo_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    size_t joint_cnt = std::min(msg->name.size(), msg->position.size());
    if(joint_cnt != 1)
    {
      return;
    }
  // lift_servo 通常只有1个关节，保存到 InfoJoint_lift
    std::unique_lock<std::mutex> lock1(*mutex_, std::defer_lock);
    if(lock1.try_lock()) //获取到进行数据更新
    {
      InfoJoint_lift.name = msg->name[0];
      InfoJoint_lift.rad_pose_current = static_cast<float>(msg->position[0]);
      InfoJoint_lift.rad_vel_current = (msg->velocity.size() > 0) ? static_cast<float>(msg->velocity[0]) : 0.0f;
      InfoJoint_lift.mode = 0;
      InfoJoint_lift.statues = 0;
      InfoJoint_lift.error = 0;
      lock1.unlock();//解锁
    }
    
    // for (size_t i = 0; i < joint_cnt; i++)
    // {
    //   LOG_INFO("收到 /joint_state/lift_servo 关节状态  %s: %.6f rad", msg->name[i].c_str(), msg->position[i]);
    // }
}

// ======================== arm健康话题订阅模块 ========================
void TopicHealth_arms::init(rclcpp::Node * node)
{
  node_ = node;

  // 订阅 /health/arms 话题
  health_arms_sub_ = node_->create_subscription<ethercat_control::msg::BringupHealth>(
    "/health/arms",
    10,
    std::bind(&TopicHealth_arms::health_arms_callback, this, std::placeholders::_1));
  LOG_INFO("已订阅 /health/arms");
}

void TopicHealth_arms::health_arms_callback(const ethercat_control::msg::BringupHealth msg)
{
  // 将 BringupHealth 消息字段映射到 arms_health_ 结构体
  InfoHealth_arms.robot_control_ready = msg.robot_control_ready;
  InfoHealth_arms.moveit_ready = msg.moveit_ready;
  InfoHealth_arms.l_arm_ready = msg.l_arm_ready;
  InfoHealth_arms.r_arm_ready = msg.r_arm_ready;
  InfoHealth_arms.head_ready = msg.head_ready;
  InfoHealth_arms.l_hand_ready = msg.l_hand_ready;
  InfoHealth_arms.r_hand_ready = msg.r_hand_ready;

  LOG_INFO("[InfoHealth_arms] robot_control=%d, moveit=%d, l_arm=%d, r_arm=%d, head=%d, l_hand=%d, r_hand=%d",
    InfoHealth_arms.robot_control_ready,
    InfoHealth_arms.moveit_ready,
    InfoHealth_arms.l_arm_ready,
    InfoHealth_arms.r_arm_ready,
    InfoHealth_arms.head_ready,
    InfoHealth_arms.l_hand_ready,
    InfoHealth_arms.r_hand_ready);
}

// ======================== ExecuteCommand 服务客户端模块 ========================
void ServiceExecCmd_arms::init(rclcpp::Node * node)
{
  node_ = node;

  exec_cmd_client_ = node_->create_client<ExecArmsSrvCmd>("/cli_controller/execute_command");
  LOG_INFO("已创建服务客户端: /cli_controller/execute_command");
}

bool ServiceExecCmd_arms::send_command(const std::string & command, bool wait_for_result, double timeout)
{
  if (!exec_cmd_client_->wait_for_service(std::chrono::seconds(2)))
  {
    LOG_WARN("服务 /cli_controller/execute_command 不可用");
    last_call_success_ = false;
    return false;
  }

  auto request = std::make_shared<ExecArmsSrvCmd::Request>();
  request->command = command;
  request->wait_for_result = wait_for_result;
  request->timeout = timeout;

  last_call_success_ = true;

  // 异步发送请求
  exec_cmd_client_->async_send_request(
    request,
    std::bind(&ServiceExecCmd_arms::handle_response, this, std::placeholders::_1));

  LOG_INFO("已发送 ExecuteCommand 请求: command=%s, wait_for_result=%d, timeout=%.1f",
    command.c_str(), wait_for_result, timeout);
  return true;
}

std::string ServiceExecCmd_arms::build_command(const exec_cmd_config_ & config)
{
  // 若 raw_command 非空，直接返回
  if (!config.raw_command.empty())
    return config.raw_command;

  // 将 goal_type 统一转为小写
  std::string goal_type = config.goal_type;
  std::transform(goal_type.begin(), goal_type.end(), goal_type.begin(), ::tolower);

  // 构建 key=value 参数列表
  std::vector<std::string> parts;

  // --- 基本参数 ---
  if (!config.group.empty())
    parts.push_back("group=" + config.group);
  if (!goal_type.empty())
    parts.push_back("type=" + goal_type);

  // --- 按 goal_type 添加特定参数 ---
  if (goal_type == "named")
  {
    if (!config.named_target.empty())
      parts.push_back("target=" + config.named_target);
  }
  else if (goal_type == "joints")
  {
    // 构建 "joint1:val1,joint2:val2,..." 格式
    if (!config.joints.empty())
    {
      std::stringstream ss;
      for (size_t i = 0; i < config.joints.size(); ++i)
      {
        if (i > 0) ss << ",";
        ss << config.joints[i].joint_name << ":" << config.joints[i].value;
      }
      parts.push_back("joints=" + ss.str());
    }
  }
  else if (goal_type == "pose")
  {
    if (!config.pose.empty())
      parts.push_back("pose=" + config.pose);
  }
  else if (goal_type == "multi_pose" || goal_type == "multipose" || goal_type == "dual_pose")
  {
    if (!config.links.empty())
      parts.push_back("links=" + config.links);
    if (!config.poses.empty())
      parts.push_back("poses=" + config.poses);
  }
  else if (goal_type == "cart" || goal_type == "cartesian")
  {
    if (!config.poses.empty())
      parts.push_back("poses=" + config.poses);
  }
  else if (goal_type == "stop")
  {
    // stop 只需要基本参数
    std::stringstream ss;
    for (size_t i = 0; i < parts.size(); ++i)
    {
      if (i > 0) ss << " ";
      ss << parts[i];
    }
    return ss.str();
  }

  // --- 公共可选参数 ---
  if (!config.frame.empty())
    parts.push_back("frame=" + config.frame);
  if (!config.end_effector_link.empty())
    parts.push_back("end_effector_link=" + config.end_effector_link);
  if (!config.planner.empty())
    parts.push_back("planner=" + config.planner);
  if (!config.pipeline_id.empty())
    parts.push_back("pipeline=" + config.pipeline_id);
  if (!config.planner_id.empty())
    parts.push_back("planner_id=" + config.planner_id);

  parts.push_back("vel=" + std::to_string(config.velocity_scaling));
  parts.push_back("acc=" + std::to_string(config.acceleration_scaling));
  parts.push_back("planning_time=" + std::to_string(config.planning_time));
  parts.push_back("exec=" + std::string(config.execute_motion ? "1" : "0"));

  // --- cart/cartesian 专用参数 ---
  if (goal_type == "cart" || goal_type == "cartesian")
  {
    parts.push_back("eef_step=" + std::to_string(config.eef_step));
    parts.push_back("jump_threshold=" + std::to_string(config.jump_threshold));
    parts.push_back("avoid=" + std::string(config.avoid_collisions ? "1" : "0"));
    parts.push_back("fraction_min=" + std::to_string(config.fraction_min));
  }

  // --- pose/multi_pose/cart 通用参数 ---
  if (goal_type == "pose" || goal_type == "multi_pose" || goal_type == "multipose"
      || goal_type == "dual_pose" || goal_type == "cart" || goal_type == "cartesian")
  {
    parts.push_back("lin_position_only=" + std::string(config.lin_position_only ? "1" : "0"));
  }

  // 用空格拼接所有参数
  std::stringstream result;
  for (size_t i = 0; i < parts.size(); ++i)
  {
    if (i > 0) result << " ";
    result << parts[i];
  }
  return result.str();
}

bool ServiceExecCmd_arms::send_command_from_config()
{
  std::string command = build_command(cmd_config_);


  LOG_INFO("[send_command_from_config] 构建命令: %s", command.c_str());

  return send_command(command, cmd_config_.wait_for_result, cmd_config_.timeout);
}

void ServiceExecCmd_arms::handle_response(rclcpp::Client<ExecArmsSrvCmd>::SharedFuture future)
{
  auto response = future.get();
  if (!response)
  {
    LOG_ERROR("ExecuteCommand 响应为空");
    last_call_success_ = false;
    return;
  }

  // 将响应映射到 exec_ArmsMoveCmd_info_ 结构体
  last_result_.accepted = response->accepted;
  last_result_.command_id = response->command_id;
  last_result_.success = response->success;
  last_result_.result_received = response->result_received;
  last_result_.timed_out = response->timed_out;
  last_result_.message = response->message;

  // 映射 CommandResult 子字段
  last_result_.result_command_id = response->result.command_id;
  last_result_.result_group = response->result.group;
  last_result_.result_goal_type = response->result.goal_type;
  last_result_.result_success = response->result.success;
  last_result_.result_moveit_code = response->result.moveit_code;
  last_result_.result_cartesian_fraction = response->result.cartesian_fraction;
  last_result_.result_executed = response->result.executed;
  last_result_.result_message = response->result.message;

  last_call_success_ = true;

  LOG_INFO("[ExecuteCommand] accepted=%d, command_id=%s, success=%d, result_received=%d, timed_out=%d, message=%s",
    last_result_.accepted,
    last_result_.command_id.c_str(),
    last_result_.success,
    last_result_.result_received,
    last_result_.timed_out,
    last_result_.message.c_str());


}

// ======================== SrvServoCmd 服务客户端模块 ========================
void ServiceSrvServoCmd::init(rclcpp::Node * node)
{
  node_ = node;

  srv_servo_cmd_client_ = node_->create_client<SrvServoCmd>("/servocmd/srvservocmd");
  LOG_INFO("已创建服务客户端: /servocmd/srvservocmd");
}

bool ServiceSrvServoCmd::send_command(const srv_servo_cmd_request_ & req_config, double timeout)
{
  if (!srv_servo_cmd_client_->wait_for_service(std::chrono::seconds(20)))
  {
    LOG_WARN("服务 /servocmd/srvservocmd 不可用");
    last_result_.call_success = false;
    return false;
  }

  auto request = std::make_shared<SrvServoCmd::Request>();
  request->master_name = req_config.master_name;
  request->node_id = req_config.node_id;
  request->command_id = req_config.command_id;
  request->seq = req_config.seq;
  request->aim_pos = req_config.aim_pos;
  request->aim_vel = req_config.aim_vel;
  request->dec = req_config.dec;

  last_result_.call_success = true;

  // 重置响应标志
  {
    std::lock_guard<std::mutex> lk(mutex_);
    response_received_ = false;
  }

  // 异步发送请求，响应回调为 handle_response
  srv_servo_cmd_client_->async_send_request(
    request,
    std::bind(&ServiceSrvServoCmd::handle_response, this, std::placeholders::_1));

  LOG_INFO("已发送 SrvServoCmd 请求: master=%s, node_id=%u, command_id=%u, aim_pos=%.3f, aim_vel=%.3f，阻塞等待响应...",
    req_config.master_name.c_str(), req_config.node_id, req_config.command_id,
    req_config.aim_pos, req_config.aim_vel);

  // 阻塞等待 handle_response 回调通知，最多等待 timeout 秒
  std::unique_lock<std::mutex> lk(mutex_);
  bool success = cv_.wait_for(lk, std::chrono::duration<double>(timeout), [this] {
    return response_received_;
  });

  if (!success)
  {
    LOG_ERROR("SrvServoCmd 请求超时（%.1f秒），未收到响应", timeout);
    last_result_.call_success = false;
    return false;
  }

  LOG_INFO("SrvServoCmd 阻塞等待完成，收到响应");
  return last_result_.call_success;
}

bool ServiceSrvServoCmd::send_command_from_config(double timeout)
{
  return send_command(cmd_request_, timeout);
}

void ServiceSrvServoCmd::handle_response(rclcpp::Client<SrvServoCmd>::SharedFuture future)
{
  auto response = future.get();
  if (!response)
  {
    LOG_ERROR("SrvServoCmd 响应为空");
    last_result_.call_success = false;
    // 通知等待线程，即使是空响应也要解除阻塞
    std::lock_guard<std::mutex> lk(mutex_);
    response_received_ = true;
    cv_.notify_one();
    return;
  }

  last_result_.statues = response->statues;
  last_result_.act_pos = response->act_pos;
  last_result_.errid = response->errid;
  last_result_.call_success = true;

  LOG_INFO("[SrvServoCmd 异步响应] statues=%u, act_pos=%.3f, errid=%u",
    last_result_.statues, last_result_.act_pos, last_result_.errid);

  // 通知阻塞的 send_command 线程：响应已收到
  {
    std::lock_guard<std::mutex> lk(mutex_);
    response_received_ = true;
    cv_.notify_one();
  }
}

// ======================== 服务端模块 ========================
void ServiceTest_MoveAxis::init(rclcpp::Node * node)
{
  node_ = node;

  move_axis_service_ = node_->create_service<my_interfaces::srv::SrvMoveAxis>(
    "/my_interfaces/move_axis",
    std::bind(&ServiceTest_MoveAxis::move_axis_service_callback, this,
      std::placeholders::_1, std::placeholders::_2));
  LOG_INFO( "已启动服务: /my_interfaces/move_axis");
}

void ServiceTest_MoveAxis::move_axis_service_callback(
  const std::shared_ptr<my_interfaces::srv::SrvMoveAxis::Request> request,
  std::shared_ptr<my_interfaces::srv::SrvMoveAxis::Response> response)
{
  LOG_INFO(
    "收到服务请求 -> id: %u, name: %s",
    static_cast<unsigned int>(request->id),
    request->name.c_str());

  // 模拟业务处理：id=1 返回100，其他返回200
  if (request->id == 1)
  {
    response->statues = 100;
  }
  else
  {
    response->statues = 200;
  }

  LOG_INFO(
    "发送服务响应 -> statues: %u",
    static_cast<unsigned int>(response->statues));
}

// ======================== ArmMotion Action 客户端模块 ========================
void ActMotion_arm::init(rclcpp::Node * node)
{
  node_ = node;
  arm_motion_finished_ = false;
  arm_motion_success_ = false;

  // 创建 ArmMotion action 客户端
  arm_action_client_ = rclcpp_action::create_client<ArmMotion>(
    node_, "/arm_motion_controller/arm_motion");
  LOG_INFO( "ArmMotion Action 客户端已创建");
}

bool ActMotion_arm::send_arm_joint_motion(
  const std::string & group,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & joint_positions,
  bool execute,
  double /*timeout*/)
{
  // 等待 action 服务端上线
  if (!arm_action_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    LOG_ERROR( "ArmMotion Action 服务未启动");
    return false;
  }

  // 填充 Goal
  ArmMotion::Goal goal;
  goal.group = group;
  goal.goal_type = "joints";
  goal.execute = execute;
  goal.joint_names = joint_names;
  goal.joint_positions = joint_positions;

  // 生成唯一 command_id
  auto ts = std::time(nullptr);
  std::stringstream cmd_ss;
  cmd_ss << "motion_ctrol_" << ts;
  goal.command_id = cmd_ss.str();

  // 重置状态标记
  arm_motion_finished_ = false;
  arm_motion_success_ = false;

  // 配置回调
  rclcpp_action::Client<ArmMotion>::SendGoalOptions send_opt;
  send_opt.feedback_callback = std::bind(
    &ActMotion_arm::arm_feedback_callback, this,
    std::placeholders::_1, std::placeholders::_2);
  send_opt.result_callback = std::bind(
    &ActMotion_arm::arm_result_callback, this,
    std::placeholders::_1);

  // 异步发送 goal
  arm_action_client_->async_send_goal(goal, send_opt);
  LOG_INFO( "已发送 ArmMotion 目标，等待执行...");
  return true;
}

void ActMotion_arm::arm_feedback_callback(
  std::shared_ptr<ArmGoalHandle>,
  const std::shared_ptr<const ArmMotion::Feedback> feedback)
{
  LOG_INFO(
    "[Arm反馈] 阶段: %s, 进度: %.3f",
    feedback->stage.c_str(), feedback->progress);
}

void ActMotion_arm::arm_result_callback(const ArmGoalHandle::WrappedResult & result)
{
  arm_motion_finished_ = true;
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
  {
    LOG_INFO( "ArmMotion 执行成功");
    arm_motion_success_ = true;
  }
  else
  {
    LOG_ERROR(
      "ArmMotion 执行失败，错误码: %d",
      static_cast<int>(result.code));
    arm_motion_success_ = false;
  }
}

// ======================== MotorCmd Action 客户端模块 ========================
void ActMotion_lift::init(rclcpp::Node * node)
{
  node_ = node;

  // 创建 MotorCmd action 客户端
  motor_action_client_ = rclcpp_action::create_client<MotorCmd>(
    node_, "/motor/lift_motioncmd");
  LOG_INFO( "MotorCmd Action 客户端已创建");
}

bool ActMotion_lift::send_motor_pos_cmd(
  double target_pos, double vel, double acc, double timeout)
{
  // 等待 action 服务端上线
  if (!motor_action_client_->wait_for_action_server(std::chrono::seconds(5)))
  {
    LOG_ERROR( "MotorCmd Action 服务未启动");
    return false;
  }

  // 填充 Goal
  MotorCmd::Goal goal;
  goal.mode = "pos";
  goal.target_pos = target_pos;
  goal.target_vel = vel;
  goal.acc = acc;
  goal.execute = true;
  goal.timeout = timeout;

  // 配置回调
  rclcpp_action::Client<MotorCmd>::SendGoalOptions send_opt;
  send_opt.feedback_callback = std::bind(
    &ActMotion_lift::motor_feedback_callback, this,
    std::placeholders::_1, std::placeholders::_2);
  send_opt.result_callback = std::bind(
    &ActMotion_lift::motor_result_callback, this,
    std::placeholders::_1);

  // 异步发送 goal
  motor_action_client_->async_send_goal(goal, send_opt);
  LOG_INFO(
    "已发送 MotorCmd 目标: pos=%.3f, vel=%.3f, acc=%.3f",
    target_pos, vel, acc);
  return true;
}

void ActMotion_lift::motor_feedback_callback(
  std::shared_ptr<GoalHandleMotor>,
  const std::shared_ptr<const MotorCmd::Feedback> feedback)
{
  LOG_INFO(
    "[Motor反馈] 位置: %.3f, 速度: %.3f, 进度: %.3f, 阶段: %s",
    feedback->current_pos, feedback->current_vel,
    feedback->progress, feedback->stage.c_str());
}

void ActMotion_lift::motor_result_callback(const GoalHandleMotor::WrappedResult & result)
{
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      LOG_INFO(
        "Motor运动成功，终点位置: %.3f", result.result->final_pos);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(node_->get_logger(), "Motor运动被取消");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      LOG_ERROR(
        "Motor运动异常中止: %s", result.result->msg.c_str());
      break;
    default:
      LOG_ERROR( "Motor运动未知状态");
      break;
  }
}

// ======================== 主节点类 ========================
RobotCtrol::RobotCtrol(const rclcpp::NodeOptions & options)
: Node("robot_ctrol_node", options)
{
    bis_stop = false;
   

}

RobotCtrol::~RobotCtrol()
{
    // 停止 taskpool 线程
    bis_stop = true;
    if (taskpool_thread_.joinable())
    {
        taskpool_thread_.join();
    }
    LOG_INFO("RobotCtrol 析构完成，taskpool 线程已停止");
}

bool RobotCtrol::init()
{
    LOG_INFO("===== RobotCtrol init =====");
    std::string pkg_prefix = ament_index_cpp::get_package_share_directory(PROJECT_NAME);
    std::string xml_path = pkg_prefix + "/launch/dev_par.xml";

    if(param_robot.load_dev_xmlconfig(xml_path))
    {
      LOG_INFO("xml file read success");
    }
    else
    {
      LOG_WARN("xml file read failed");

      return false;
    }

    //创建tcpsever
    pobj_mdtcpserver = std::make_shared<ModbusTcpServerCpp>(param_robot.para_net_tcp.ip, 
    param_robot.para_net_tcp.port,param_robot.para_net_tcp.connmax,param_robot.para_net_tcp.intertime);
    //创建tcp 参数对象
    pobj_mdpar =  std::make_shared<mdreg_data>();

    //tcp 初始化
    pobj_mdtcpserver->init();
    // 1. 初始化机械臂关节话题订阅模块
    topic_arm_module_.init(this,&mutex_arm_data);

    // 2. 初始化升降伺服话题订阅模块
    topic_lift_module_.init(this , &mutex_liftservo_data);

    // 3. 初始化arm健康话题订阅模块
    topic_health_arms_module_.init(this);

    // 4. 初始化服务端模块
    // service_testAxis_.init(this);

    // 5. 初始化 ArmMotion Action 客户端模块
    // Act_arm_motion_.init(this);

    // 6. 初始化 MotorCmd Action 客户端模块
    // Act_liftmotor_motion_.init(this);

    // 7. 初始化 ExecuteCommand 服务客户端模块
    service_execdmd_arms_.init(this);

    // 8. 初始化 SrvServoCmd 服务客户端模块
    service_srv_servo_cmd_.init(this);



    LOG_INFO("RobotCtrol 节点已启动，等待事件...");





    return true;
}
void RobotCtrol::taskpool()
{
  // 10ms 周期任务：在此添加周期性业务逻辑
  //获取参数
  LOG_INFO("taskpool is running ");
  pobj_mdpar->show_reg.Reg.sta_system = 0;
  LOG_INFO("system state is %d ",pobj_mdpar->show_reg.Reg.sta_system);
  while (!bis_stop)
  {
     pobj_mdtcpserver->getdown_Input_reg(0,pobj_mdpar->hold_reg.data,REGINDEX_HOLDREG_START);
    
      pobj_mdpar->xch_word2bool_holdreg1();//获取按键信息

      //cycle
      switch (pobj_mdpar->show_reg.Reg.sta_system ) //系统总状态机
      {
        case 0: //初始状态
          pobj_mdpar->show_reg.Reg.sta_system = 5;
          LOG_INFO("system state is %d ",pobj_mdpar->show_reg.Reg.sta_system);
          break;

        case 5: //给lift servo进行op
          service_srv_servo_cmd_.cmd_request_.master_name = "can0";
          service_srv_servo_cmd_.cmd_request_.node_id = 1;
          service_srv_servo_cmd_.cmd_request_.command_id = SERVO_POWERON;  // 例: 使能命令
          service_srv_servo_cmd_.cmd_request_.seq = 0;
          // 使用 send_command_from_config 异步发送请求，响应回调为 handle_response
          service_srv_servo_cmd_.send_command_from_config(20.0);

          if(service_srv_servo_cmd_.last_result_.errid == 0 && 
            (service_srv_servo_cmd_.last_result_.statues & STAWORD_MASK) == STAWORD_OPEN2RUN_FBACK)
            {
              pobj_mdpar->show_reg.Reg.sta_system = 10;
              LOG_INFO("system state is %d ",pobj_mdpar->show_reg.Reg.sta_system);
            }
          
          break;

          case 10: //给lift servo进行op
            service_srv_servo_cmd_.cmd_request_.master_name = "can0";
            service_srv_servo_cmd_.cmd_request_.node_id = 1;
            service_srv_servo_cmd_.cmd_request_.command_id = SERVO_MVPOS_ABS;  // 例: 使能命令
            service_srv_servo_cmd_.cmd_request_.seq = 0;
            service_srv_servo_cmd_.cmd_request_.aim_pos = 20*CODEAXIS_RATIO_RPM;
            service_srv_servo_cmd_.cmd_request_.aim_vel = 180*RATIO_VEL_DEC_RPM;
            service_srv_servo_cmd_.cmd_request_.dec = 180*RATIO_VEL_DEC_RPM/60/1;
            service_srv_servo_cmd_.send_command_from_config(50.0);

            if(service_srv_servo_cmd_.last_result_.errid == 0 && 
              (service_srv_servo_cmd_.last_result_.statues & STAWORD_MASK) == STAWORD_OPEN2RUN_FBACK)
              {
                pobj_mdpar->show_reg.Reg.sta_system = 15;
                LOG_INFO("system state is %d ",pobj_mdpar->show_reg.Reg.sta_system);
              }
          
          break;

                case 15: //给lift servo进行op
            service_srv_servo_cmd_.cmd_request_.master_name = "can0";
            service_srv_servo_cmd_.cmd_request_.node_id = 1;
            service_srv_servo_cmd_.cmd_request_.command_id = SERVO_MVPOS_ABS;  // 例: 使能命令
            service_srv_servo_cmd_.cmd_request_.seq = 0;
            service_srv_servo_cmd_.cmd_request_.aim_pos = 0;
            service_srv_servo_cmd_.cmd_request_.aim_vel = 180*RATIO_VEL_DEC_RPM;
            service_srv_servo_cmd_.cmd_request_.dec = 180*RATIO_VEL_DEC_RPM/60/1;
            service_srv_servo_cmd_.send_command_from_config(50.0);

            if(service_srv_servo_cmd_.last_result_.errid == 0 && 
              (service_srv_servo_cmd_.last_result_.statues & STAWORD_MASK) == STAWORD_OPEN2RUN_FBACK)
              {
                pobj_mdpar->show_reg.Reg.sta_system = 10;
                LOG_INFO("system state is %d ",pobj_mdpar->show_reg.Reg.sta_system);
              }
          
          break;
        
        default:
          break;
      }






      //上行参数
      pobj_mdpar->hold_reg_last_save();//保持last状态
      updata_pos_mutex();
      pobj_mdtcpserver->update_Input_reg(0,pobj_mdpar->show_reg.data,REGINDEX_HOLDREG_START);
      // 延时20毫秒
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  
 
}

void RobotCtrol::updata_pos_mutex()
{
  //获取手臂位置
  std::unique_lock<std::mutex> lock(mutex_arm_data, std::defer_lock);
  if(lock.try_lock()) //获取到进行数据更新
  {
        pobj_mdpar->write_float_to_word(pobj_mdpar->out_armL.x,pobj_mdpar->show_reg.Reg.pos_arml_x_f2w_1, pobj_mdpar->show_reg.Reg.pos_arml_x_f2w_2 );
        pobj_mdpar->write_float_to_word(pobj_mdpar->out_armL.y,pobj_mdpar->show_reg.Reg.pos_arml_y_f2w_1, pobj_mdpar->show_reg.Reg.pos_arml_y_f2w_2 );
        pobj_mdpar->write_float_to_word(pobj_mdpar->out_armL.z,pobj_mdpar->show_reg.Reg.pos_arml_z_f2w_1, pobj_mdpar->show_reg.Reg.pos_arml_z_f2w_2 );
        pobj_mdpar->write_float_to_word(pobj_mdpar->out_armL.rx,pobj_mdpar->show_reg.Reg.pos_arml_rx_f2w_1, pobj_mdpar->show_reg.Reg.pos_arml_rx_f2w_2 );
        pobj_mdpar->write_float_to_word(pobj_mdpar->out_armL.ry,pobj_mdpar->show_reg.Reg.pos_arml_ry_f2w_1, pobj_mdpar->show_reg.Reg.pos_arml_ry_f2w_2 );
        pobj_mdpar->write_float_to_word(pobj_mdpar->out_armL.rz,pobj_mdpar->show_reg.Reg.pos_arml_rz_f2w_1, pobj_mdpar->show_reg.Reg.pos_arml_rz_f2w_2 );

        pobj_mdpar->write_float_to_word(pobj_mdpar->out_armR.x,pobj_mdpar->show_reg.Reg.pos_armr_x_f2w_1, pobj_mdpar->show_reg.Reg.pos_armr_x_f2w_2 );
        pobj_mdpar->write_float_to_word(pobj_mdpar->out_armR.y,pobj_mdpar->show_reg.Reg.pos_armr_y_f2w_1, pobj_mdpar->show_reg.Reg.pos_armr_y_f2w_2 );
        pobj_mdpar->write_float_to_word(pobj_mdpar->out_armR.z,pobj_mdpar->show_reg.Reg.pos_armr_z_f2w_1, pobj_mdpar->show_reg.Reg.pos_armr_z_f2w_2 );
        pobj_mdpar->write_float_to_word(pobj_mdpar->out_armR.rx,pobj_mdpar->show_reg.Reg.pos_armr_rx_f2w_1, pobj_mdpar->show_reg.Reg.pos_armr_rx_f2w_2 );
        pobj_mdpar->write_float_to_word(pobj_mdpar->out_armR.ry,pobj_mdpar->show_reg.Reg.pos_armr_ry_f2w_1, pobj_mdpar->show_reg.Reg.pos_armr_ry_f2w_2 );
        pobj_mdpar->write_float_to_word(pobj_mdpar->out_armR.rz,pobj_mdpar->show_reg.Reg.pos_armr_rz_f2w_1, pobj_mdpar->show_reg.Reg.pos_armr_rz_f2w_2 );

    lock.unlock();//解锁
  }

  //获取手臂位置
  std::unique_lock<std::mutex> lock1(mutex_liftservo_data, std::defer_lock);
  if(lock1.try_lock()) //获取到进行数据更新
  {
    pobj_mdpar->write_float_to_word(pobj_mdpar->out_lift,pobj_mdpar->show_reg.Reg.pos_servo_lift_f2w_1,pobj_mdpar->show_reg.Reg.pos_servo_lift_f2w_2);
    lock1.unlock();//解锁
  }



}
void RobotCtrol::run()
{
  LOG_INFO("===== tcp server run =====");
  pobj_mdtcpserver->start();
  pobj_mdtcpserver->g_modbus_map->tab_registers[1] =10;

  
  
  // 8. 创建 taskpool 工作线程，使用 bind 绑定 taskpool，10ms 轮询延时
  taskpool_thread_ = std::thread(&RobotCtrol::taskpool, this);
  LOG_INFO("已创建 taskpool 工作线程 (20ms 周期)");
  // LOG_INFO("===== RobotCtrol run =====");

  // 根据命令字符串:
  // group=r_arm type=joints joints=right_arm_7_joint:0.1,right_arm_6_joint:0.1,right_arm_5_joint:0.1
  //   pipeline=ompl planner_id=RRTConnectkConfigDefault exec=1
  // wait_for_result=true, timeout=0

  // --- 基本参数 ---
  // service_execdmd_arms_.cmd_config_.group = "r_arm";
  // service_execdmd_arms_.cmd_config_.goal_type = "joints";
  // service_execdmd_arms_.cmd_config_.execute_motion = true;       // exec=1
  // service_execdmd_arms_.cmd_config_.wait_for_result = true;
  // service_execdmd_arms_.cmd_config_.timeout = 0.0;

  // // --- 规划器参数 ---
  // service_execdmd_arms_.cmd_config_.pipeline_id = "ompl";
  // service_execdmd_arms_.cmd_config_.planner_id = "RRTConnectkConfigDefault";

  // // --- joints 参数: right_arm_7_joint:0.1, right_arm_6_joint:0.1, right_arm_5_joint:0.1 ---
  // service_execdmd_arms_.cmd_config_.joints.clear();
  // {
  //   joint_name_value_ j1;
  //   j1.joint_name = "right_arm_1_joint";
  //   j1.value = 0.5;
  //   service_execdmd_arms_.cmd_config_.joints.push_back(j1);
  // }
  // {
  //   joint_name_value_ j2;
  //   j2.joint_name = "right_arm_2_joint";
  //   j2.value = 0.5;
  //   service_execdmd_arms_.cmd_config_.joints.push_back(j2);
  // }
  // {
  //   joint_name_value_ j3;
  //   j3.joint_name = "right_arm_5_joint";
  //   j3.value = 0.1;
  //   service_execdmd_arms_.cmd_config_.joints.push_back(j3);
  // }

  

  // service_execdmd_arms_.send_command_from_config();

  // ======================== SrvServoCmd 服务调用示例 ========================
  // 设置请求参数到 cmd_request_
  // service_srv_servo_cmd_.cmd_request_.master_name = "can0";
  // service_srv_servo_cmd_.cmd_request_.node_id = 1;
  // service_srv_servo_cmd_.cmd_request_.command_id = 0x01;  // 例: 使能命令
  // service_srv_servo_cmd_.cmd_request_.seq = 0;
  // service_srv_servo_cmd_.cmd_request_.aim_pos = 0.0f;
  // service_srv_servo_cmd_.cmd_request_.aim_vel = 100.0f;
  // service_srv_servo_cmd_.cmd_request_.dec = 0.0f;

  // // 使用 send_command_from_config 异步发送请求，响应回调为 handle_response
  // service_srv_servo_cmd_.send_command_from_config();

}
} // namespace robot_ctrol_node

// ======================== main ========================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_ctrol_node::RobotCtrol>();
  // node->init();

  // // 在后台线程中执行 spin，使回调（如 handle_response）能够被处理
  // std::thread spin_thread([node]() {
  //   rclcpp::spin(node);
  // });

  // // 在主线程中执行 run()（包含阻塞的 send_command_from_config）
  // node->run();

  // // run() 结束后，不要调用 rclcpp::shutdown()，否则节点会立即退出
  // // 等待 spin 线程结束（SIGINT 时 ROS2 会自动调用 rclcpp::shutdown）
  // if (spin_thread.joinable())
  // {
  //   spin_thread.join();
  // }

  node->init();
  node->run();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
