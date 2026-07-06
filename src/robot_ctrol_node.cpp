#include "robot_ctrol_node/robot_ctrol_node.hpp"
#include "../include/log.hpp"
#include "../include/datedef.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include "../include/robot_ctrol_node/transform.hpp"



namespace robot_ctrol_node
{


  #define BOOLSHOW_REG1 1
  #define BIT_IS_AUTO 0
  #define BIT_IS_AUTO_DONE 1
  #define BIT_IS_MANU 2
  #define BIT_IS_MANU_DONE 3
 
// ======================== 机械臂关节话题订阅模块 ========================
void TopicJoint_arm::init(rclcpp::Node * node)
{
  node_ = node;
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
   std::unique_lock<std::mutex> lock1(mutex_, std::defer_lock);
  if(lock1.try_lock()) //获取到进行数据更新
  {
    for (size_t i = 0; i < joint_cnt; i++)
    {
      const std::string & jname = msg->name[i];
      joint_state_ js;
      js.name = jname;
      js.rad_pose_current = static_cast<float>(msg->position[i]);
      js.rad_vel_current = (i < msg->velocity.size()) ? static_cast<float>(msg->velocity[i]) : 0.0f;
      js.command_id = 0;
      js.statues = 0;
      js.error = 0;

      // 保存到 InfoJoint_arm map
      InfoJoint_arm[jname] = js;

      // LOG_INFO("[InfoJoint_arm] %s: pose=%.6f rad, vel=%.6f rad/s",
      //   js.name.c_str(), js.rad_pose_current, js.rad_vel_current);
    }
    lock1.unlock();//解锁
  }

}

// ======================== 升降伺服话题订阅模块 ========================
void TopicJoint_lift::init(rclcpp::Node * node)
{
  node_ = node;


  // 订阅 /joint_state/lift_servo 话题
  lift_servo_sub_ = node_->create_subscription<my_interfaces::msg::MsgJointStateCmd>(
    "/joint_state/lift_servo",
    10,
    std::bind(&TopicJoint_lift::lift_servo_callback, this, std::placeholders::_1));
  LOG_INFO("已订阅 /joint_state/lift_servo");
}

void TopicJoint_lift::lift_servo_callback(const my_interfaces::msg::MsgJointStateCmd::SharedPtr msg)
{
    size_t joint_cnt = std::min(msg->joint_state.name.size(), msg->joint_state.position.size());
    if(joint_cnt != 1)
    {
      return;
    }
  // lift_servo 通常只有1个关节，保存到 InfoJoint_lift
    std::unique_lock<std::mutex> lock1(mutex_, std::defer_lock);
    if(lock1.try_lock()) //获取到进行数据更新
    {
      InfoJoint_lift.name = msg->joint_state.name[0];
      InfoJoint_lift.rad_pose_current = static_cast<float>(msg->joint_state.position[0]);
      InfoJoint_lift.rad_vel_current = (msg->joint_state.velocity.size() > 0) ? static_cast<float>(msg->joint_state.velocity[0]) : 0.0f;
      InfoJoint_lift.command_id = msg->commanding;
      InfoJoint_lift.error = msg->errid ;
      InfoJoint_lift.bbusy = msg->bbusy;
      InfoJoint_lift.bdone = msg->bdone;
      InfoJoint_lift.req = msg->req;
//       LOG_INFO("收到 /joint_state/lift_servo 关节状态  %s: %.6f rad", msg->joint_state.name[0].c_str(), msg->joint_state.position[0]);
      lock1.unlock();//解锁
    }
    
    // for (size_t i = 0; i < joint_cnt; i++)
    // {
   
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

// ======================== arm_motion_controller 结果话题订阅模块 ========================
void TopicArmMotionResult::init(rclcpp::Node * node)
{
  node_ = node;

  // 订阅 /arm_motion_controller/result 话题
  result_sub_ = node_->create_subscription<robot_controller::msg::CommandResult>(
    "/arm_motion_controller/result",
    10,
    std::bind(&TopicArmMotionResult::result_callback, this, std::placeholders::_1));
  LOG_INFO("已订阅 /arm_motion_controller/result");
}

void TopicArmMotionResult::result_callback(const robot_controller::msg::CommandResult::SharedPtr msg)
{
  std::unique_lock<std::mutex> lock1(mutex_, std::defer_lock);
  if(lock1.try_lock()) //获取到进行数据更新
  {
    InfoArmMotionResult.command_id = msg->command_id;
    InfoArmMotionResult.group = msg->group;
    InfoArmMotionResult.goal_type = msg->goal_type;
    InfoArmMotionResult.success = msg->success;
    InfoArmMotionResult.moveit_code = msg->moveit_code;
    InfoArmMotionResult.cartesian_fraction = msg->cartesian_fraction;
    InfoArmMotionResult.executed = msg->executed;
    InfoArmMotionResult.message = msg->message;
    InfoArmMotionResult.start_time_ns = static_cast<int64_t>(msg->start_time.sec) * 1000000000LL + msg->start_time.nanosec;
    InfoArmMotionResult.end_time_ns = static_cast<int64_t>(msg->end_time.sec) * 1000000000LL + msg->end_time.nanosec;

    LOG_INFO("[InfoArmMotionResult] command_id=%s, group=%s, goal_type=%s, success=%d, moveit_code=%d, executed=%d, message=%s",
      InfoArmMotionResult.command_id.c_str(),
      InfoArmMotionResult.group.c_str(),
      InfoArmMotionResult.goal_type.c_str(),
      InfoArmMotionResult.success,
      InfoArmMotionResult.moveit_code,
      InfoArmMotionResult.executed,
      InfoArmMotionResult.message.c_str());

    lock1.unlock();//解锁
  }
}

// ======================== arm 服务客户端模块 ========================
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
  std::unique_lock<std::mutex> lock1( mutex_ );
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

// ======================== liftServoCmd 服务客户端模块 ========================
void ServiceExecCmd_lift::init(rclcpp::Node * node)
{
  node_ = node;
  cmd_request_.seq =0;

  srv_servo_cmd_client_ = node_->create_client<SrvServoCmd>("/servocmd/srvservocmd");
  LOG_INFO("已创建服务客户端: /servocmd/srvservocmd");
}

bool ServiceExecCmd_lift::send_command(const srv_servo_cmd_request_ & req_config, uint8_t timeout)
{
  if (!srv_servo_cmd_client_->wait_for_service(std::chrono::seconds(timeout)))
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


  // 异步发送请求，响应回调为 handle_response
  srv_servo_cmd_client_->async_send_request(
    request,
    std::bind(&ServiceExecCmd_lift::handle_response, this, std::placeholders::_1));

  LOG_INFO("已发送 SrvServoCmd 请求: master=%s, node_id=%u, command_id=%u, aim_pos=%.3f, aim_vel=%.3f，阻塞等待响应...",
    req_config.master_name.c_str(), req_config.node_id, req_config.command_id,
    req_config.aim_pos, req_config.aim_vel);

  return last_result_.call_success;
}

bool ServiceExecCmd_lift::send_command_from_config(double timeout)
{
  return send_command(cmd_request_, timeout);
}

void ServiceExecCmd_lift::handle_response(rclcpp::Client<SrvServoCmd>::SharedFuture future)
{
  auto response = future.get();
  if (!response)
  {
    LOG_ERROR("SrvServoCmd 响应为空");
    last_result_.call_success = false; 
    return;
  }

  last_result_.statues = response->statues;
  last_result_.act_pos = response->act_pos;
  last_result_.errid = response->errid;
  last_result_.call_success = true;

  LOG_INFO("[SrvServoCmd 异步响应] statues=%u, act_pos=%.3f, errid=%u",
    last_result_.statues, last_result_.act_pos, last_result_.errid);

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
    topic_arm_module_.init(this);

    // 2. 初始化升降伺服话题订阅模块
    topic_lift_module_.init(this);

    // 3. 初始化arm健康话题订阅模块
    topic_health_arms_module_.init(this);

    // 4. 初始化 arm_motion_controller 结果话题订阅模块
    topic_arm_motion_result_module_.init(this);

    // 4.1 初始化左手末端位姿话题订阅模块
    topic_eefpos_handleft_module_.init(this);

    // 4.2 初始化右手末端位姿话题订阅模块
    topic_eefpos_handright_module_.init(this);

    // 5. 初始化服务端模块
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
/// @brief 
void RobotCtrol::taskpool()
{
  // 10ms 周期任务：在此添加周期性业务逻辑
  //获取参数
  LOG_INFO("taskpool is running ");
  pobj_mdpar->hold_reg.Reg.sta_system = 0;
  outtime_ wcdog_liftservo_op= {5,0,1200,0 };//延时监控 12s
 
  while (!bis_stop)
  {
    
     pobj_mdtcpserver->getdown_Input_reg(0,pobj_mdpar->hold_reg.data,REGLEN_MAX_HOLD*2);
     pobj_mdpar->download_floatpar();//获取float类型     
     pobj_mdpar->xch_word2bool_holdreg1();//获取按键信息

      LOG_INFO("system state is %d ",pobj_mdpar->hold_reg.Reg.sta_system);

      //cycle
      switch (pobj_mdpar->hold_reg.Reg.sta_system ) //系统总状态机
      {
        case 0: //初始状态
          //waiting for arm ready
          // if(topic_health_arms_module_.InfoHealth_arms.l_arm_ready ==true 
          // && topic_health_arms_module_.InfoHealth_arms.r_arm_ready ==true
          // && topic_health_arms_module_.InfoHealth_arms.moveit_ready ==true
          // && topic_health_arms_module_.InfoHealth_arms.robot_control_ready ==true
          // )
          // {
              pobj_mdpar->hold_reg.Reg.sta_system = 5;
              pobj_mdpar->hold_reg.Reg.sta_sub_action =0;          
              
          // }
          break;

        case 5: //等待提升电机到位
          if(topic_lift_module_.InfoJoint_lift.name == "lift_servo")
          {
            pobj_mdpar->hold_reg.Reg.sta_system = 10;
            pobj_mdpar->hold_reg.Reg.sta_manu_disp = 0;
            wcdog_liftservo_op.retry_cnt =0 ;
            wcdog_liftservo_op.outtime_cnt =0;

          }

          break;

        case 10: //提升电机使能
          if(pobj_mdpar->hold_reg.Reg.sta_manu_disp ==0) //发送指令
          {
              wcdog_liftservo_op.outtime_cnt =0;             
              service_srv_servo_cmd_.cmd_request_.seq+=1;
              srv_servo_cmd_request_ reque;
              action_info_ template_action;//使能
              template_action.index = 0;
              template_action.type = 61;
              template_action.sub_index = 0;
              transform::getliftMotionCmd(template_action, service_srv_servo_cmd_.cmd_request_.seq,service_srv_servo_cmd_.cmd_request_);
              // 使用 send_command_from_config 
              service_srv_servo_cmd_.send_command_from_config(2);

              if(service_srv_servo_cmd_.last_result_.call_success == true )
              {
                  pobj_mdpar->hold_reg.Reg.sta_manu_disp =10;
                  LOG_INFO("system state is %d ",pobj_mdpar->hold_reg.Reg.sta_manu_disp);
              }
          }
          else if(pobj_mdpar->hold_reg.Reg.sta_manu_disp ==10) //等待返回
          {
            if(topic_lift_module_.InfoJoint_lift.req == service_srv_servo_cmd_.cmd_request_.seq &&
              topic_lift_module_.InfoJoint_lift.bdone ==  true && topic_lift_module_.InfoJoint_lift.error ==0)
              {
                pobj_mdpar->hold_reg.Reg.sta_sub_action =20;
                pobj_mdpar->hold_reg.Reg.sta_system = 15;
                LOG_INFO("system state is %d ",pobj_mdpar->hold_reg.Reg.sta_manu_disp);
                

              } 
            else
            {
              if(wcdog_liftservo_op.outtime_cnt++ > wcdog_liftservo_op.outtime_set)
              {
                if(wcdog_liftservo_op.retry_cnt ++ > wcdog_liftservo_op.retry_set) //等待10s 重新下发
                {
                  LOG_ERROR("Lift servo op failed");
                  exit(0);//系统退出
                }
                pobj_mdpar->hold_reg.Reg.sta_manu_disp = 0;
              }
              
            }

          }
          break;
        case 15: //大循环
          if( pobj_mdpar->hold_reg.Reg.set_mode_run ==0) //自动模式
          {
            mode_auto();
          }
          else if(pobj_mdpar->hold_reg.Reg.set_mode_run ==1) //手动模式
          {
            mode_manu();
          }

          break;

        case 100: //给lift servo进行op
          // if(decode_action(param_robot.actions_init) == true)
          // {
          //   pobj_mdpar->hold_reg.Reg.sta_system = 15;
          // }

          break;
         
        case 150:
          break;  
        
        default:
          break;
      }




      monitor();

      //上行参数
      pobj_mdpar->hold_reg_last_save();//保持last状态
      updata_pos_mutex();
      pobj_mdtcpserver->update_Input_reg(0,pobj_mdpar->hold_reg.data,REGLEN_MAX_HOLD*2);
            // 延时20毫秒
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  
 
}

void RobotCtrol::mode_auto()
{
  if( pobj_mdpar->hold_BtnReg0_curr.btn_2start == true &&
   pobj_mdpar->hold_BtnReg0_last.btn_2start == false) //沿检测
   {
    pobj_mdpar->hold_BtnReg1_cur.bfalg_isAuto = true;
    pobj_mdpar->hold_BtnReg1_cur.bfalg_AutoTask_done =false;
    pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu = false;
    pobj_mdpar->hold_BtnReg1_cur.bfalg_ManuTask_done =false;
    pobj_mdpar->hold_reg.Reg.sta_sub_action =0;
    pobj_mdpar->hold_reg.Reg.sta_action_disp = pobj_mdpar->hold_reg.Reg.sta_action_set;
    LOG_INFO("start auto action mode ,action index is %d ",pobj_mdpar->hold_reg.Reg.sta_action_disp);    
   }

   if(pobj_mdpar->hold_BtnReg1_cur.bfalg_isAuto == true)
   {
      if(decode_action(param_robot.actions[pobj_mdpar->hold_reg.Reg.sta_action_disp]) == true)
      {
        pobj_mdpar->hold_BtnReg1_cur.bfalg_isAuto = false;
        pobj_mdpar->hold_BtnReg1_cur.bfalg_AutoTask_done =true;
        LOG_INFO("auto action mode done ");
      }
   }
}

void RobotCtrol::mode_manu()
{

  //提升点动
  if( pobj_mdpar->hold_BtnReg0_curr.btn_lift_2run == true && pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu==false &&
   pobj_mdpar->hold_BtnReg0_last.btn_lift_2run == false) //手动进行提升操作
   {
    pobj_mdpar->hold_BtnReg1_cur.bfalg_isAuto = false;
    pobj_mdpar->hold_BtnReg1_cur.bfalg_AutoTask_done =false;
    pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu = true;
    pobj_mdpar->hold_BtnReg1_cur.bfalg_ManuTask_done =false;
    pobj_mdpar->hold_reg.Reg.sta_manu_disp = 0;
    pobj_mdpar->hold_reg.Reg.type_manu_disp =1;
    //获取目标位置 与速度信息
    LOG_INFO("start manu mode ,action index is lift servo ");
   }

    if(pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu == true && pobj_mdpar->hold_reg.Reg.type_manu_disp==1)
    {
      if(mode_manu_lift() == true)
      {
        pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu = false;
        pobj_mdpar->hold_BtnReg1_cur.bfalg_ManuTask_done =true;
        pobj_mdpar->hold_reg.Reg.type_manu_disp =0;
        LOG_INFO("manu action mode done ");
      }
    }

  //左臂 ptp
  if( pobj_mdpar->hold_BtnReg0_curr.btn_armL_2run == true && pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu==false &&
   pobj_mdpar->hold_BtnReg0_last.btn_armL_2run == false) //手动进行左臂操作
   {
    pobj_mdpar->hold_BtnReg1_cur.bfalg_isAuto = false;
    pobj_mdpar->hold_BtnReg1_cur.bfalg_AutoTask_done =false;
    pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu = true;
    pobj_mdpar->hold_BtnReg1_cur.bfalg_ManuTask_done =false;
    pobj_mdpar->hold_reg.Reg.sta_manu_disp = 0;
    pobj_mdpar->hold_reg.Reg.type_manu_disp = 2;
    LOG_INFO("start manu mode ,action index is left arm ");
   }

    if(pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu == true && pobj_mdpar->hold_reg.Reg.type_manu_disp ==2)
    {
      action_info_ par;
      par.index = 0;
      if(pobj_mdpar->hold_BtnReg0_curr.sw_type_armrun == false)
      {
        par.type = 22; //左臂
      }
      else
      {
        par.type = 32; //左臂
      }      
      par.sub_index = 0;
      par.info_.action_data.d[0] = pobj_mdpar->getfloat_par.par.aim_arml_x;
      par.info_.action_data.d[1] = pobj_mdpar->getfloat_par.par.aim_arml_y;
      par.info_.action_data.d[2] = pobj_mdpar->getfloat_par.par.aim_arml_z;
      par.info_.action_data.d[3] = pobj_mdpar->getfloat_par.par.aim_arml_rx;
      par.info_.action_data.d[4] = pobj_mdpar->getfloat_par.par.aim_arml_ry;
      par.info_.action_data.d[5] = pobj_mdpar->getfloat_par.par.aim_arml_rz;

      if(mode_manu_arm_rotate_head_hand(par) == true)
      {
        pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu = false;
        pobj_mdpar->hold_BtnReg1_cur.bfalg_ManuTask_done =true;
        pobj_mdpar->hold_reg.Reg.type_manu_disp =0;
         pobj_mdpar->hold_reg.Reg.sta_manu_disp = 0;
        LOG_INFO("manu action mode done ");
      }
    }

  //右臂 ptp
  if( pobj_mdpar->hold_BtnReg0_curr.btn_armR_2run == true && pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu==false &&
   pobj_mdpar->hold_BtnReg0_last.btn_armR_2run == false) //手动进行右臂操作
   {
    pobj_mdpar->hold_BtnReg1_cur.bfalg_isAuto = false;
    pobj_mdpar->hold_BtnReg1_cur.bfalg_AutoTask_done =false;
    pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu = true;
    pobj_mdpar->hold_BtnReg1_cur.bfalg_ManuTask_done =false;
    pobj_mdpar->hold_reg.Reg.sta_manu_disp = 0;
    pobj_mdpar->hold_reg.Reg.type_manu_disp = 3;
    LOG_INFO("start manu mode ,action index is right arm ");
   }

   if(pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu == true && pobj_mdpar->hold_reg.Reg.type_manu_disp ==3)
    {
      action_info_ par;
      par.index = 0;
      if(pobj_mdpar->hold_BtnReg0_curr.sw_type_armrun == false)
      {
        par.type = 23; //右臂
      }
      else
      {
        par.type = 33; //右臂
      }   
      par.sub_index = 0;
      par.info_.action_data.d[0] = pobj_mdpar->getfloat_par.par.aim_arml_x;
      par.info_.action_data.d[1] = pobj_mdpar->getfloat_par.par.aim_arml_y;
      par.info_.action_data.d[2] = pobj_mdpar->getfloat_par.par.aim_arml_z;
      par.info_.action_data.d[3] = pobj_mdpar->getfloat_par.par.aim_arml_rx;
      par.info_.action_data.d[4] = pobj_mdpar->getfloat_par.par.aim_arml_ry;
      par.info_.action_data.d[5] = pobj_mdpar->getfloat_par.par.aim_arml_rz;

      if(mode_manu_arm_rotate_head_hand(par) == true)
      {
        pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu = false;
        pobj_mdpar->hold_BtnReg1_cur.bfalg_ManuTask_done =true;
        pobj_mdpar->hold_reg.Reg.type_manu_disp =0;
         pobj_mdpar->hold_reg.Reg.sta_manu_disp = 0;
        LOG_INFO("manu action mode done ");
      }
    }

    //左手运动
    if( pobj_mdpar->hold_BtnReg0_curr.btn_handL_2run == true && pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu==false &&
   pobj_mdpar->hold_BtnReg0_last.btn_handL_2run == false) //手动进行右臂操作
   {
    pobj_mdpar->hold_BtnReg1_cur.bfalg_isAuto = false;
    pobj_mdpar->hold_BtnReg1_cur.bfalg_AutoTask_done =false;
    pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu = true;
    pobj_mdpar->hold_BtnReg1_cur.bfalg_ManuTask_done =false;
    pobj_mdpar->hold_reg.Reg.sta_manu_disp = 0;
    pobj_mdpar->hold_reg.Reg.type_manu_disp = 4;
    LOG_INFO("start manu mode ,action index is handl");
   }

   if(pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu == true && pobj_mdpar->hold_reg.Reg.type_manu_disp ==4)
    {
      action_info_ par;
      par.index = 0;
      par.type = 42; //右臂
     
      par.sub_index = 0;
      par.info_.action_data.d[0] = pobj_mdpar->getfloat_par.par.aim_handl;
      

      if(mode_manu_arm_rotate_head_hand(par) == true)
      {
        pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu = false;
        pobj_mdpar->hold_BtnReg1_cur.bfalg_ManuTask_done =true;
        pobj_mdpar->hold_reg.Reg.type_manu_disp =0;
         pobj_mdpar->hold_reg.Reg.sta_manu_disp = 0;
        LOG_INFO("manu handl action mode done ");
      }
    }


        //右手运动
    if( pobj_mdpar->hold_BtnReg0_curr.btn_handR_2run == true && pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu==false &&
   pobj_mdpar->hold_BtnReg0_last.btn_handR_2run == false) //手动进行右臂操作
   {
    pobj_mdpar->hold_BtnReg1_cur.bfalg_isAuto = false;
    pobj_mdpar->hold_BtnReg1_cur.bfalg_AutoTask_done =false;
    pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu = true;
    pobj_mdpar->hold_BtnReg1_cur.bfalg_ManuTask_done =false;
    pobj_mdpar->hold_reg.Reg.sta_manu_disp = 0;
    pobj_mdpar->hold_reg.Reg.type_manu_disp = 5;
    LOG_INFO("start manu mode ,action index is handl");
   }

   if(pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu == true && pobj_mdpar->hold_reg.Reg.type_manu_disp ==5)
    {
      action_info_ par;
      par.index = 0;
      par.type = 43; //右臂
     
      par.sub_index = 0;
      par.info_.action_data.d[0] = pobj_mdpar->getfloat_par.par.aim_handr;
      

      if(mode_manu_arm_rotate_head_hand(par) == true)
      {
        pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu = false;
        pobj_mdpar->hold_BtnReg1_cur.bfalg_ManuTask_done =true;
        pobj_mdpar->hold_reg.Reg.type_manu_disp =0;
         pobj_mdpar->hold_reg.Reg.sta_manu_disp = 0;
        LOG_INFO("manu handl action mode done ");
      }
    }

            //旋转运动
    if( pobj_mdpar->hold_BtnReg0_curr.btn_rotat_2run == true && pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu==false &&
   pobj_mdpar->hold_BtnReg0_last.btn_rotat_2run == false) //手动进行右臂操作
   {
    pobj_mdpar->hold_BtnReg1_cur.bfalg_isAuto = false;
    pobj_mdpar->hold_BtnReg1_cur.bfalg_AutoTask_done =false;
    pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu = true;
    pobj_mdpar->hold_BtnReg1_cur.bfalg_ManuTask_done =false;
    pobj_mdpar->hold_reg.Reg.sta_manu_disp = 0;
    pobj_mdpar->hold_reg.Reg.type_manu_disp = 6;
    LOG_INFO("start manu mode ,action index is handl");
   }

   if(pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu == true && pobj_mdpar->hold_reg.Reg.type_manu_disp ==6)
    {
      action_info_ par;
      par.index = 0;
      par.type = 52; //右臂
     
      par.sub_index = 0;
      par.info_.action_data.d[0] = pobj_mdpar->getfloat_par.par.aim_rotate;
      

      if(mode_manu_arm_rotate_head_hand(par) == true)
      {
        pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu = false;
        pobj_mdpar->hold_BtnReg1_cur.bfalg_ManuTask_done =true;
        pobj_mdpar->hold_reg.Reg.type_manu_disp =0;
         pobj_mdpar->hold_reg.Reg.sta_manu_disp = 0;
        LOG_INFO("manu handl action mode done ");
      }
    }

                //头运动
    if( pobj_mdpar->hold_BtnReg0_curr.btn_head_2run == true && pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu==false &&
   pobj_mdpar->hold_BtnReg0_last.btn_head_2run == false) //手动进行右臂操作
   {
    pobj_mdpar->hold_BtnReg1_cur.bfalg_isAuto = false;
    pobj_mdpar->hold_BtnReg1_cur.bfalg_AutoTask_done =false;
    pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu = true;
    pobj_mdpar->hold_BtnReg1_cur.bfalg_ManuTask_done =false;
    pobj_mdpar->hold_reg.Reg.sta_manu_disp = 0;
    pobj_mdpar->hold_reg.Reg.type_manu_disp = 7;
    LOG_INFO("start manu mode ,action index is handl");
   }

   if(pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu == true && pobj_mdpar->hold_reg.Reg.type_manu_disp ==7)
    {
      action_info_ par;
      par.index = 0;
      par.type = 53; //右臂
     
      par.sub_index = 0;
      par.info_.action_data.d[0] = pobj_mdpar->getfloat_par.par.aim_head;
      

      if(mode_manu_arm_rotate_head_hand(par) == true)
      {
        pobj_mdpar->hold_BtnReg1_cur.bfalg_isManu = false;
        pobj_mdpar->hold_BtnReg1_cur.bfalg_ManuTask_done =true;
        pobj_mdpar->hold_reg.Reg.type_manu_disp =0;
         pobj_mdpar->hold_reg.Reg.sta_manu_disp = 0;
        LOG_INFO("manu handl action mode done ");
      }
    }
}

/// @brief 手动提升
/// @return 速度由xml配置文件中获取，目标位置由modbus寄存器获取
bool RobotCtrol::mode_manu_lift()
{
  if(pobj_mdpar->hold_reg.Reg.sta_manu_disp ==0) //发送指令
      {
          service_srv_servo_cmd_.cmd_request_.seq+=1;
          srv_servo_cmd_request_ reque;
          action_info_ template_action;//使能
          template_action.index = 0;
          template_action.type = 62;
          template_action.sub_index = 0;

          if(param_robot.para_axis_lift.speed_convert_factor == 0)
          {
            LOG_WARN("speed_convert_factor is 0");
            return false;
          }

          if(param_robot.para_axis_lift.time_acc_s == 0)
          {
            LOG_WARN("time_acc_s is 0");
            return false;
          }

          template_action.info_.action_data.d[0] = pobj_mdpar->getfloat_par.par.aim_lift / param_robot.para_axis_lift.speed_convert_factor; //转换为多少转
          template_action.info_.action_data.d[1] = param_robot.para_axis_lift.veljog_mpm / param_robot.para_axis_lift.speed_convert_factor;
          template_action.info_.action_data.d[2] = template_action.info_.action_data.d[1] / param_robot.para_axis_lift.time_acc_s / 60;

          transform::getliftMotionCmd(template_action, service_srv_servo_cmd_.cmd_request_.seq,service_srv_servo_cmd_.cmd_request_);
          // 使用 send_command_from_config 
          service_srv_servo_cmd_.send_command_from_config(2);

          if(service_srv_servo_cmd_.last_result_.call_success == true )
          {
              pobj_mdpar->hold_reg.Reg.sta_manu_disp =10;
              LOG_INFO("lift servo running in manu mode,aim pos is %.2f ,vel is %.2f", template_action.info_.Axis_servo_pose.dis, template_action.info_.Axis_servo_pose.vel);
          }
      }
      else if(pobj_mdpar->hold_reg.Reg.sta_manu_disp ==10) //等待返回
      {
        if(topic_lift_module_.InfoJoint_lift.req == service_srv_servo_cmd_.cmd_request_.seq &&
          topic_lift_module_.InfoJoint_lift.bdone ==  true && topic_lift_module_.InfoJoint_lift.error ==0)
          {
            pobj_mdpar->hold_reg.Reg.sta_manu_disp =20;
            
            
            LOG_INFO("lift servo running done");
            return true;
           

          } 

      }
    return false;
}
/// @brief 
/// @param type 22 左臂 ；23 右臂； 42 左手；43 右手； 52：rotate 53：head

bool RobotCtrol::mode_manu_arm_rotate_head_hand(const action_info_ &par)
{
     if(pobj_mdpar->hold_reg.Reg.sta_manu_disp ==0) //发送指令
      {
          transform::getArmMotionCmd(par, service_execdmd_arms_.cmd_config_);
          service_execdmd_arms_.send_command_from_config();         
          pobj_mdpar->hold_reg.Reg.sta_manu_disp =5;
          std::unique_lock<std::mutex> lock1( service_execdmd_arms_.mutex_ );
          service_execdmd_arms_.last_result_.success =false ;
          service_execdmd_arms_.last_call_success_= false;

      }
      else if(pobj_mdpar->hold_reg.Reg.sta_manu_disp ==5) //waiting for callback
      {
        std::unique_lock<std::mutex> lock1( service_execdmd_arms_.mutex_ );
        if(service_execdmd_arms_.last_result_.success == true && service_execdmd_arms_.last_result_.result_group ==service_execdmd_arms_.cmd_config_.group)
          {
            pobj_mdpar->hold_reg.Reg.sta_manu_disp =10;
            LOG_INFO("sta_manu state is %d ",pobj_mdpar->hold_reg.Reg.sta_manu_disp);
          }
        

      }
      else if(pobj_mdpar->hold_reg.Reg.sta_manu_disp ==10) //等待返回
      {
        LOG_INFO("InfoArmMotionResult.command_id is %d ",topic_arm_motion_result_module_.InfoArmMotionResult.command_id);
        LOG_INFO("service_execdmd_arms_.last_result_.command_id is %d ",service_execdmd_arms_.last_result_.command_id);
        if(topic_arm_motion_result_module_.InfoArmMotionResult.command_id == service_execdmd_arms_.last_result_.command_id
        && topic_arm_motion_result_module_.InfoArmMotionResult.executed ==true && service_execdmd_arms_.last_result_.result_group ==topic_arm_motion_result_module_.InfoArmMotionResult.group)
           
          {
            pobj_mdpar->hold_reg.Reg.sta_manu_disp =15;
            LOG_INFO("sta_manu_disp2 state is %d ",pobj_mdpar->hold_reg.Reg.sta_manu_disp);
            return true;
          } 

      }
      return false;
}
void RobotCtrol::monitor()
{
  //心跳设置
   if(pobj_mdpar->hold_reg.Reg.system_heartbeat_cnt++ >10000)
   {
     pobj_mdpar->hold_reg.Reg.system_heartbeat_cnt = 0;
   }

   //
}
void RobotCtrol::updata_pos_mutex()
{
  //获取手臂位置
  std::unique_lock<std::mutex> lock(topic_arm_module_.mutex_, std::defer_lock);
  if(lock.try_lock()) //获取到进行数据更新
  {
      //  pobj_mdpar->write_float_to_word(pobj_mdpar->out_armL.x,pobj_mdpar->hold_reg.Reg., pobj_mdpar->hold_reg.Reg.pos_arml_x_f2w_2 );
       

    lock.unlock();//解锁
  }

  //获取提升电机位置
  std::unique_lock<std::mutex> lock_1(topic_lift_module_.mutex_, std::defer_lock);
  if(lock_1.try_lock()) //获取到进行数据更新
  {
    float value = topic_lift_module_.InfoJoint_lift.rad_pose_current/CODEAXIS_RATIO_RPM_LIFTSERVO*param_robot.para_axis_lift.speed_convert_factor;
    pobj_mdpar->write_float_to_word(topic_lift_module_.InfoJoint_lift.rad_pose_current,pobj_mdpar->hold_reg.Reg.currpos_lift_1_1,pobj_mdpar->hold_reg.Reg.currpos_lift_1_2);
    lock_1.unlock();//解锁
  }

    //获取左臂坐标
  std::unique_lock<std::mutex> lock1(topic_eefpos_handleft_module_.mutex_, std::defer_lock);
  if(lock1.try_lock()) //获取到进行数据更新
  {
    pobj_mdpar->write_float_to_word(topic_eefpos_handleft_module_.InfoEefpos_handleft.x, pobj_mdpar->hold_reg.Reg.currpos_arml_x_1,pobj_mdpar->hold_reg.Reg.currpos_arml_x_2);
    pobj_mdpar->write_float_to_word(topic_eefpos_handleft_module_.InfoEefpos_handleft.y, pobj_mdpar->hold_reg.Reg.currpos_arml_y_1,pobj_mdpar->hold_reg.Reg.currpos_arml_y_2);
    pobj_mdpar->write_float_to_word(topic_eefpos_handleft_module_.InfoEefpos_handleft.z, pobj_mdpar->hold_reg.Reg.currpos_arml_z_1,pobj_mdpar->hold_reg.Reg.currpos_arml_z_2);
    pobj_mdpar->write_float_to_word(topic_eefpos_handleft_module_.InfoEefpos_handleft.roll, pobj_mdpar->hold_reg.Reg.currpos_arml_rx_1,pobj_mdpar->hold_reg.Reg.currpos_arml_rx_2);
    pobj_mdpar->write_float_to_word(topic_eefpos_handleft_module_.InfoEefpos_handleft.pitch, pobj_mdpar->hold_reg.Reg.currpos_arml_ry_1,pobj_mdpar->hold_reg.Reg.currpos_arml_ry_2);
    pobj_mdpar->write_float_to_word(topic_eefpos_handleft_module_.InfoEefpos_handleft.yaw, pobj_mdpar->hold_reg.Reg.currpos_arml_rz_1,pobj_mdpar->hold_reg.Reg.currpos_arml_rz_2);

    lock1.unlock();//解锁
  }

      //获取右臂坐标
  std::unique_lock<std::mutex> lock2(topic_eefpos_handright_module_.mutex_, std::defer_lock);
  if(lock2.try_lock()) //获取到进行数据更新
  {
    pobj_mdpar->write_float_to_word(topic_eefpos_handright_module_.InfoEefpos_handright.x, pobj_mdpar->hold_reg.Reg.currpos_armr_x_1,pobj_mdpar->hold_reg.Reg.currpos_armr_x_2);
    pobj_mdpar->write_float_to_word(topic_eefpos_handright_module_.InfoEefpos_handright.y, pobj_mdpar->hold_reg.Reg.currpos_armr_y_1,pobj_mdpar->hold_reg.Reg.currpos_armr_y_2);
    pobj_mdpar->write_float_to_word(topic_eefpos_handright_module_.InfoEefpos_handright.z, pobj_mdpar->hold_reg.Reg.currpos_armr_z_1,pobj_mdpar->hold_reg.Reg.currpos_armr_z_2);
    pobj_mdpar->write_float_to_word(topic_eefpos_handright_module_.InfoEefpos_handright.roll, pobj_mdpar->hold_reg.Reg.currpos_armr_rx_1,pobj_mdpar->hold_reg.Reg.currpos_armr_rx_2);
    pobj_mdpar->write_float_to_word(topic_eefpos_handright_module_.InfoEefpos_handright.pitch, pobj_mdpar->hold_reg.Reg.currpos_armr_ry_1,pobj_mdpar->hold_reg.Reg.currpos_armr_ry_2);
    pobj_mdpar->write_float_to_word(topic_eefpos_handright_module_.InfoEefpos_handright.yaw, pobj_mdpar->hold_reg.Reg.currpos_armr_rz_1,pobj_mdpar->hold_reg.Reg.currpos_armr_rz_2);

    lock2.unlock();//解锁
  }


}

/// @brief 
bool RobotCtrol::decode_action(array_actions_info_ &par)
{
  //当运动自状态机大于运动条数+5，以5为步长进行设置状态机
  
  if(pobj_mdpar->hold_reg.Reg.sta_sub_action >= (par.actions.size()*10))
  {
    return true;
  }
   auto index = pobj_mdpar->hold_reg.Reg.sta_sub_action /10;
   auto type = par.actions.at(index).type; //获取类型

   LOG_INFO("index is %d type is %d date is %f", index,type,par.actions.at(index).info_.action_data.d[0]);

   if(type < 10 ) //延时
   {

   }
   else if(type < 60) //双臂控制
   {
      if(pobj_mdpar->hold_reg.Reg.sta_sub_action %10 ==0) //发送指令
      {
          transform::getArmMotionCmd(par.actions.at(index), service_execdmd_arms_.cmd_config_);
          service_execdmd_arms_.send_command_from_config();         
          pobj_mdpar->hold_reg.Reg.sta_sub_action +=2;
          std::unique_lock<std::mutex> lock1( service_execdmd_arms_.mutex_ );
          service_execdmd_arms_.last_result_.success =false ;
          service_execdmd_arms_.last_call_success_= false;

      }
      else if(pobj_mdpar->hold_reg.Reg.sta_sub_action %10 ==2) //waiting for callback
      {
        std::unique_lock<std::mutex> lock1( service_execdmd_arms_.mutex_ );
        if(service_execdmd_arms_.last_result_.success == true && service_execdmd_arms_.last_result_.result_group ==service_execdmd_arms_.cmd_config_.group)
          {
            pobj_mdpar->hold_reg.Reg.sta_sub_action +=3;
            LOG_INFO("sta_sub_action1 state is %d ",pobj_mdpar->hold_reg.Reg.sta_sub_action && service_execdmd_arms_.last_result_.result_group ==service_execdmd_arms_.cmd_config_.group);
          }
        else if(service_execdmd_arms_.last_call_success_== true )
        {
          //没有接收成功，需要重新发送
          pobj_mdpar->hold_reg.Reg.sta_sub_action =(pobj_mdpar->hold_reg.Reg.sta_sub_action /10)*10;
            LOG_INFO("retry cmd arm %d ",pobj_mdpar->hold_reg.Reg.sta_sub_action);

        }

      }
      else if(pobj_mdpar->hold_reg.Reg.sta_sub_action %10 ==5) //等待返回
      {
        LOG_INFO("InfoArmMotionResult.command_id is %d ",topic_arm_motion_result_module_.InfoArmMotionResult.command_id);
        LOG_INFO("service_execdmd_arms_.last_result_.command_id is %d ",service_execdmd_arms_.last_result_.command_id);
        if(topic_arm_motion_result_module_.InfoArmMotionResult.command_id == service_execdmd_arms_.last_result_.command_id
        && topic_arm_motion_result_module_.InfoArmMotionResult.executed ==true && service_execdmd_arms_.last_result_.result_group ==topic_arm_motion_result_module_.InfoArmMotionResult.group)
           
          {
            pobj_mdpar->hold_reg.Reg.sta_sub_action +=5;
            LOG_INFO("sta_sub_action2 state is %d ",pobj_mdpar->hold_reg.Reg.sta_sub_action);
          } 

      }
   }
   else if(type < 70) //提升电机
   {
      if(pobj_mdpar->hold_reg.Reg.sta_sub_action %10 ==0) //发送指令
      {
          service_srv_servo_cmd_.cmd_request_.seq+=1;
          srv_servo_cmd_request_ reque;
          transform::getliftMotionCmd(par.actions.at(index), service_srv_servo_cmd_.cmd_request_.seq,service_srv_servo_cmd_.cmd_request_);
          // 使用 send_command_from_config 
          service_srv_servo_cmd_.send_command_from_config(2);

          if(service_srv_servo_cmd_.last_result_.call_success == true )
          {
              pobj_mdpar->hold_reg.Reg.sta_sub_action +=5;
              LOG_INFO("system state is %d ",pobj_mdpar->hold_reg.Reg.sta_system);
          }
      }
      else if(pobj_mdpar->hold_reg.Reg.sta_sub_action %10 ==5) //等待返回
      {
        if(topic_lift_module_.InfoJoint_lift.req == service_srv_servo_cmd_.cmd_request_.seq &&
          topic_lift_module_.InfoJoint_lift.bdone ==  true && topic_lift_module_.InfoJoint_lift.error ==0)
          {
            pobj_mdpar->hold_reg.Reg.sta_sub_action +=5;
            LOG_INFO("system state is %d ",pobj_mdpar->hold_reg.Reg.sta_system);
          } 

      }

    }
    return false;
  
  

}


// ======================== 左手末端位姿话题订阅模块 ========================
void TopicEefpos_handleft::init(rclcpp::Node * node)
{
  node_ = node;
  eef_pose_sub_ = node_->create_subscription<robot_controller::msg::EndEffectorPose>(
    "/eef_pose_publisher/left_hand",
    10,
    std::bind(&TopicEefpos_handleft::eef_pose_callback, this, std::placeholders::_1));
  LOG_INFO("已订阅 /eef_pose_publisher/left_hand");
}

void TopicEefpos_handleft::eef_pose_callback(const robot_controller::msg::EndEffectorPose::SharedPtr msg)
{
  std::unique_lock<std::mutex> lock1(mutex_, std::defer_lock);
  if(lock1.try_lock())
  {
    InfoEefpos_handleft.arm = msg->arm;
    InfoEefpos_handleft.base_frame = msg->base_frame;
    InfoEefpos_handleft.ee_frame = msg->ee_frame;
    InfoEefpos_handleft.x = msg->x;
    InfoEefpos_handleft.y = msg->y;
    InfoEefpos_handleft.z = msg->z;
    InfoEefpos_handleft.roll = msg->roll;
    InfoEefpos_handleft.pitch = msg->pitch;
    InfoEefpos_handleft.yaw = msg->yaw;
    InfoEefpos_handleft.tf_data_age_sec = msg->tf_data_age_sec;
    InfoEefpos_handleft.valid = msg->valid;
    InfoEefpos_handleft.status = msg->status;
    InfoEefpos_handleft.stamp_sec = msg->header.stamp.sec;
    InfoEefpos_handleft.stamp_nanosec = msg->header.stamp.nanosec;

    LOG_INFO("[InfoEefpos_handleft] arm=%s, x=%.4f, y=%.4f, z=%.4f, roll=%.4f, pitch=%.4f, yaw=%.4f, valid=%d",
      InfoEefpos_handleft.arm.c_str(),
      InfoEefpos_handleft.x, InfoEefpos_handleft.y, InfoEefpos_handleft.z,
      InfoEefpos_handleft.roll, InfoEefpos_handleft.pitch, InfoEefpos_handleft.yaw,
      InfoEefpos_handleft.valid);
    lock1.unlock();
  }
}

// ======================== 右手末端位姿话题订阅模块 ========================
void TopicEefpos_handright::init(rclcpp::Node * node)
{
  node_ = node;
  eef_pose_sub_ = node_->create_subscription<robot_controller::msg::EndEffectorPose>(
    "/eef_pose_publisher/right_hand",
    10,
    std::bind(&TopicEefpos_handright::eef_pose_callback, this, std::placeholders::_1));
  LOG_INFO("已订阅 /eef_pose_publisher/right_hand");
}

void TopicEefpos_handright::eef_pose_callback(const robot_controller::msg::EndEffectorPose::SharedPtr msg)
{
  std::unique_lock<std::mutex> lock1(mutex_, std::defer_lock);
  if(lock1.try_lock())
  {
    InfoEefpos_handright.arm = msg->arm;
    InfoEefpos_handright.base_frame = msg->base_frame;
    InfoEefpos_handright.ee_frame = msg->ee_frame;
    InfoEefpos_handright.x = msg->x;
    InfoEefpos_handright.y = msg->y;
    InfoEefpos_handright.z = msg->z;
    InfoEefpos_handright.roll = msg->roll;
    InfoEefpos_handright.pitch = msg->pitch;
    InfoEefpos_handright.yaw = msg->yaw;
    InfoEefpos_handright.tf_data_age_sec = msg->tf_data_age_sec;
    InfoEefpos_handright.valid = msg->valid;
    InfoEefpos_handright.status = msg->status;
    InfoEefpos_handright.stamp_sec = msg->header.stamp.sec;
    InfoEefpos_handright.stamp_nanosec = msg->header.stamp.nanosec;

    LOG_INFO("[InfoEefpos_handright] arm=%s, x=%.4f, y=%.4f, z=%.4f, roll=%.4f, pitch=%.4f, yaw=%.4f, valid=%d",
      InfoEefpos_handright.arm.c_str(),
      InfoEefpos_handright.x, InfoEefpos_handright.y, InfoEefpos_handright.z,
      InfoEefpos_handright.roll, InfoEefpos_handright.pitch, InfoEefpos_handright.yaw,
      InfoEefpos_handright.valid);
    lock1.unlock();
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
  

}
} // namespace robot_ctrol_node

// ======================== main ========================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_ctrol_node::RobotCtrol>();

  node->init();
  node->run();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
