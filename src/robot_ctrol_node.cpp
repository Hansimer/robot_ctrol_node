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


  // 输出当前 map 中所有关节数据
  // LOG_INFO("[InfoJoint_arm] 当前共 %zu 个关节数据:", InfoJoint_arm.size());
  // for (const auto & kv : InfoJoint_arm)
  // {
  //   const auto & js = kv.second;
  //   LOG_INFO("  [%s] pose=%.6f, vel=%.6f", js.name.c_str(), js.rad_pose_current, js.rad_vel_current);
  // }
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
      InfoJoint_lift.rad_pose_current = static_cast<float>(msg->joint_state.position[0])/CODEAXIS_RATIO_RPM;
      InfoJoint_lift.rad_vel_current = (msg->joint_state.velocity.size() > 0) ? static_cast<float>(msg->joint_state.velocity[0]) : 0.0f;
      InfoJoint_lift.command_id = msg->commanding;
      InfoJoint_lift.error = msg->errid ;
      InfoJoint_lift.bbusy = msg->bbusy;
      InfoJoint_lift.bdone = msg->bdone;
      InfoJoint_lift.req = msg->req;
       LOG_INFO("收到 /joint_state/lift_servo 关节状态  %s: %.6f rad", msg->joint_state.name[0].c_str(), msg->joint_state.position[0]);
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
          service_srv_servo_cmd_.cmd_request_.seq = 1;
          // 使用 send_command_from_config 
          service_srv_servo_cmd_.send_command_from_config(2);

          if(service_srv_servo_cmd_.last_result_.call_success == true )
          {
              pobj_mdpar->show_reg.Reg.sta_system = 10;
              LOG_INFO("system state is %d ",pobj_mdpar->show_reg.Reg.sta_system);
          }
          break;

        case 10:
            LOG_INFO("InfoJoint_lift.req is %d cmd_request_.seq is %d bbond is %d errid is %d",
              topic_lift_module_.InfoJoint_lift.req,service_srv_servo_cmd_.cmd_request_.seq,
            topic_lift_module_.InfoJoint_lift.bdone,topic_lift_module_.InfoJoint_lift.error);
            if(topic_lift_module_.InfoJoint_lift.req == service_srv_servo_cmd_.cmd_request_.seq &&
            topic_lift_module_.InfoJoint_lift.bdone ==  true && topic_lift_module_.InfoJoint_lift.error ==0)
            {
              pobj_mdpar->show_reg.Reg.sta_system = 15;
              LOG_INFO("system state is %d ",pobj_mdpar->show_reg.Reg.sta_system);
            } 
            break;


          case 15: //给lift servo进行op
            service_srv_servo_cmd_.cmd_request_.master_name = "can0";
            service_srv_servo_cmd_.cmd_request_.node_id = 1;
            service_srv_servo_cmd_.cmd_request_.command_id = SERVO_MVPOS_ABS;  // 例: 使能命令
            service_srv_servo_cmd_.cmd_request_.seq = service_srv_servo_cmd_.cmd_request_.seq+1;
            service_srv_servo_cmd_.cmd_request_.aim_pos = 20*CODEAXIS_RATIO_RPM;
            service_srv_servo_cmd_.cmd_request_.aim_vel = 180*RATIO_VEL_DEC_RPM;
            service_srv_servo_cmd_.cmd_request_.dec = 180*RATIO_VEL_DEC_RPM/60/1;
            service_srv_servo_cmd_.send_command_from_config(1.0);
            if(service_srv_servo_cmd_.last_result_.call_success == true )
            {
                pobj_mdpar->show_reg.Reg.sta_system = 20;
                LOG_INFO("system state is %d ",pobj_mdpar->show_reg.Reg.sta_system);
            }
            break;
     

          case 20: //给lift servo进行op
            if(topic_lift_module_.InfoJoint_lift.req == service_srv_servo_cmd_.cmd_request_.seq &&
            topic_lift_module_.InfoJoint_lift.bdone ==  true && topic_lift_module_.InfoJoint_lift.error ==0)
            {
              pobj_mdpar->show_reg.Reg.sta_system = 25;
              LOG_INFO("system state is %d ",pobj_mdpar->show_reg.Reg.sta_system);
            } 
            break;

          case 25:

             service_srv_servo_cmd_.cmd_request_.master_name = "can0";
            service_srv_servo_cmd_.cmd_request_.node_id = 1;
            service_srv_servo_cmd_.cmd_request_.command_id = SERVO_MVPOS_ABS;  // 例: 使能命令
            service_srv_servo_cmd_.cmd_request_.seq = service_srv_servo_cmd_.cmd_request_.seq+1;
            service_srv_servo_cmd_.cmd_request_.aim_pos = 0;
            service_srv_servo_cmd_.cmd_request_.aim_vel = 180*RATIO_VEL_DEC_RPM;
            service_srv_servo_cmd_.cmd_request_.dec = 180*RATIO_VEL_DEC_RPM/60/1;
            service_srv_servo_cmd_.send_command_from_config(1.0);
            if(service_srv_servo_cmd_.last_result_.call_success == true )
            {
                pobj_mdpar->show_reg.Reg.sta_system = 30;
                LOG_INFO("system state is %d ",pobj_mdpar->show_reg.Reg.sta_system);
            }
            break;
     
          case 30: //给lift servo进行op
            if(topic_lift_module_.InfoJoint_lift.req == service_srv_servo_cmd_.cmd_request_.seq &&
            topic_lift_module_.InfoJoint_lift.bdone ==  true && topic_lift_module_.InfoJoint_lift.error ==0)
            {
              pobj_mdpar->show_reg.Reg.sta_system = 15;
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
  std::unique_lock<std::mutex> lock(topic_arm_module_.mutex_, std::defer_lock);
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
  std::unique_lock<std::mutex> lock1(topic_lift_module_.mutex_, std::defer_lock);
  if(lock1.try_lock()) //获取到进行数据更新
  {
    pobj_mdpar->write_float_to_word(topic_lift_module_.InfoJoint_lift.rad_pose_current,pobj_mdpar->show_reg.Reg.pos_servo_lift_f2w_1,pobj_mdpar->show_reg.Reg.pos_servo_lift_f2w_2);
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
