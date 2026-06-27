
#pragma once

#include <string>
#include <chrono>   
#include <map>
#include <vector>

#define REGINDEX_SET_MODE_1 0x0001 //机械臂模式
#define REGINDEX_SWITCH_MODE_2 0x0002 //切换模式 从00 切换到FF

#define REGINDEX_INPUTREG_START 0 //显示寄存器开始索引0
#define REGINDEX_HOLDREG_START 200
#define REGLEN_MAX_INPUT 200
#define REGLEN_MAX_HOLD 200




using namespace std;

namespace robot_ctrol_node
{

    /////寄存器 显示数据结构
       
 /////////////////////////////////机器人状态       
        struct joint_state_
        {
            string name;
            float rad_pose_current;
            float rad_vel_current;
            uint8_t command_id;
            uint8_t statues;
            uint8_t error;
            uint8_t req; //正在执行的序列
            bool bbusy;
            bool bdone;
        };

        struct pose_
        {
            float x;
            float y;
            float z;
            float rx;
            float ry;
            float rz;
        };

        struct robot_info
        {
            joint_state_ joint_neck; //脖子
            joint_state_ joint_rotate; //旋转
            joint_state_ joint_lift; //提升
            joint_state_ joint_arm_left[8]; //机械臂左边8个关节
            joint_state_ joint_arm_right[8]; //机械臂右边8个关节
            pose_ pose_neck;
            pose_ pose_rotate;
            pose_ pose_lift;
            pose_ pose_arm_left;
            pose_ pose_arm_right;

        };

        //原始数据
        struct action_data_
        {
            float d[14];            
        };

        //运动类型单臂节点运动
        struct single_arm_joint_
        {
            float joint1_rad;
            float joint2_rad;
            float joint3_rad;
            float joint4_rad;
            float joint5_rad;
            float joint6_rad;
            float joint7_rad;
        };

        //运动类型单臂节点运动
        struct arms_health_
        {
            bool robot_control_ready;
            bool moveit_ready;
            bool l_arm_ready;
            bool r_arm_ready;
            bool head_ready;
            bool l_hand_ready;
            bool r_hand_ready;
        };



        //运动类型双臂节点运动
        struct dual_arm_joint_
        {
            float armL_joint1_rad;
            float armL_joint2_rad;
            float armL_joint3_rad;
            float armL_joint4_rad;
            float armL_joint5_rad;
            float armL_joint6_rad;
            float armL_joint7_rad;
            float armR_joint1_rad;
            float armR_joint2_rad;
            float armR_joint3_rad;
            float armR_joint4_rad;
            float armR_joint5_rad;
            float armR_joint6_rad;
            float armR_joint7_rad;
            
        };

        //运动类型等待时间
        struct wait_delay_
        {
            uint16_t delay_ms;
        };

        //运动类型单臂点对点运动
        struct single_arm_PizP2P_
        {
            float x;
            float y;
            float z;
            float rx;
            float ry;
            float rz;
        };
        //运动类型双臂点对点运动
        struct dual_arm_PizP2P_
        {
            float armL_x;
            float armL_y;
            float armL_z;
            float armL_rx;
            float armL_ry;
            float armL_rz;
            float armR_x;
            float armR_y;
            float armR_z;
            float armR_rx;
            float armR_ry;
            float armR_rz;
        };
        //运动类型单臂线运动
        struct single_arm_Pizline_
        {
            float x;
            float y;
            float z;
            float rx;
            float ry;
            float rz;
        };
        //运动类型双臂线运动
        struct dual_arm_Pizline_
        {
            float armL_x;
            float armL_y;
            float armL_z;
            float armL_rx;
            float armL_ry;
            float armL_rz;
            float armR_x;
            float armR_y;
            float armR_z;
            float armR_rx;
            float armR_ry;
            float armR_rz;
        };
        //运动类型单臂轴运动
        struct Axis_servo_pose_
        {
            float dis;   
            float vel;
            float dec;
        };


        struct action_info_
        {
            std::int16_t index;
            std::int16_t sub_index;
            std::uint8_t type;

            union Data
            {
                action_data_ action_data;
                single_arm_joint_ single_arm_joint;
                dual_arm_joint_ dual_arm_joint;
                wait_delay_ wait_delay;
                single_arm_PizP2P_ single_arm_PizP2P;
                dual_arm_PizP2P_ dual_arm_PizP2P;
                single_arm_Pizline_ single_arm_Pizline;
                dual_arm_Pizline_ dual_arm_Pizline;
                Axis_servo_pose_ Axis_servo_pose;
            } info_;
        };


        struct dev_params_info_
        {
            std::string mode;
            float speed_convert_factor;
        };

        struct tcp_params_info_
        {
            std::string ip;
            int port;
            int connmax;
            int intertime;            
        };

        struct array_actions_info_
        {
            std::vector<action_info_> actions;
        };

///////////////////////////////////////////////fb块相关
        struct fb_delay_
        {
            bool in_execut;//执行
            uint16_t in_time_set;//一轮询周期为基数

            bool out_busy;//正在执行
            bool out_err;//报错
            bool out_done;//完成


            bool temp_execut_last;
            uint16_t temp_time_cnt;//一轮询周期为基数

        };

        /// @brief 获取机器人模型
        struct fb_getbotmodel_
        {
            bool in_execut;//执行
            string str_checkmodel;//验证的运动模型

            bool out_busy;//正在执行
            bool out_err;//报错
            bool out_done;//完成


            bool temp_execut_last;           

        };

        /// @brief 获取机器人模型
        // struct fb_getbotmodel_
        // {
        //     bool in_execut;//执行
        //     string str_checkmodel;//验证的运动模型

        //     bool out_busy;//正在执行
        //     bool out_err;//报错
        //     bool out_done;//完成


        //     bool temp_execut_last;           

        // };

        // ===================== ExecuteCommand 命令配置结构体 =====================
        /// @brief 关节名-值对，用于 joints 类型运动
        struct joint_name_value_
        {
            std::string joint_name;   // 关节名称，如 "right_arm_7_joint"
            double value;             // 关节目标位置（弧度）
        };

        /// @brief ExecuteCommand 命令配置，支持不同运动类型
        /// 运动类型(goal_type): "joints" | "pose" | "named" | "multi_pose" | "cart" | "stop"
        struct exec_cmd_config_
        {
            // --- 基本参数 ---
            std::string group;              // 运动组: "r_arm", "l_arm", "dual_arms" 等
            std::string goal_type;          // 运动类型: "joints", "pose", "named", "multi_pose", "cart", "stop"
            bool execute_motion = true;     // 是否立即执行
            bool wait_for_result = true;    // 是否等待结果
            double timeout = 0.0;           // 超时时间(秒)，0表示不超时

            // --- 规划器参数 ---
            std::string frame = "base_link";        // 参考坐标系
            std::string pipeline_id = "ompl";       // 规划管线: "ompl", "pilz_industrial_motion_planner"
            std::string planner_id = "RRTConnectkConfigDefault"; // 规划器ID
            double velocity_scaling = 0.2;          // 速度缩放因子
            double acceleration_scaling = 0.2;      // 加速度缩放因子
            double planning_time = 5.0;             // 规划时间限制(秒)

            // --- pose 类型参数 ---
            // pose 字符串格式: "x,y,z,rx,ry,rz"
            std::string pose;               // 位姿值，如 "0.6,-0.18,0.80,0,-1.57,3.14"

            // --- joints 类型参数 ---
            // 多个关节名-值对，构建时生成 "joint1:val1,joint2:val2,..." 格式
            std::vector<joint_name_value_> joints;

            // --- named 类型参数 ---
            std::string named_target;       // 预定义位姿名称

            // --- multi_pose 类型参数 ---
            std::string links;              // 链路名称列表
            std::string poses;              // 多位姿值

            // --- cart (笛卡尔) 类型参数 ---
            double eef_step = 0.01;         // 末端执行器步长
            double jump_threshold = 0.0;    // 跳跃阈值
            bool avoid_collisions = true;   // 是否避障
            double fraction_min = 0.0;      // 最小轨迹分数

            // --- 其他可选参数 ---
            std::string end_effector_link;  // 末端执行器链接
            std::string planner;            // 规划器名称(旧接口)
            bool lin_position_only = false; // 仅位置(线性运动)
            bool avoid = true;              // 是否避障(cart/pose类型)

            // --- 原始命令(直接使用，跳过build) ---
            std::string raw_command;        // 若非空，直接使用此命令字符串
        };

        // ===================== SrvServoCmd 服务请求/响应结构体 =====================
        /// @brief SrvServoCmd 服务请求配置
        struct srv_servo_cmd_request_
        {
            std::string master_name;    // CAN主站名称
            uint8_t node_id = 0;        // 节点ID
            uint8_t command_id = 0;     // 命令ID
            uint32_t seq = 0;           // 序列号
            float aim_pos = 0.0f;       // 目标位置
            float aim_vel = 0.0f;       // 目标速度
            float dec = 0.0f;           // 减速度
        };

        /// @brief SrvServoCmd 服务响应结果
        struct srv_servo_cmd_response_
        {
            uint16_t statues = 0;       // 状态字
            float act_pos = 0.0f;       // 实际位置
            uint8_t errid = 0;          // 错误码
            bool call_success = false;  // 调用是否成功
        };

        // ===================== ArmMotion CommandResult 订阅结果结构体 =====================
        struct arm_motion_result_info_
        {
            std::string command_id;
            std::string group;
            std::string goal_type;
            bool success = false;
            int32_t moveit_code = 0;
            double cartesian_fraction = 0.0;
            bool executed = false;
            std::string message;
            int64_t start_time_ns = 0;  // 起始时间（纳秒时间戳）
            int64_t end_time_ns = 0;    // 结束时间（纳秒时间戳）
        };

        // ===================== ExecuteCommand 服务响应结构体 =====================
        struct exec_ArmsMoveCmd_info_
        {
            bool accepted;          // 命令是否被接受
            std::string command_id; // 命令ID
            bool success;           // 执行是否成功
            bool result_received;   // 是否收到结果
            bool timed_out;         // 是否超时
            std::string message;    // 消息

            // CommandResult 子字段
            std::string result_command_id;
            std::string result_group;
            std::string result_goal_type;
            bool result_success;
            int32_t result_moveit_code;
            double result_cartesian_fraction;
            bool result_executed;
            std::string result_message;
        };


}
