#pragma once

#include "../datedef.hpp"
#include "modbustcp_server.hpp"
#include <sstream>


namespace robot_ctrol_node
{
    class transform
    {
        public:
            /*
                joint_name:
                    right_arm_1_joint
                    left_arm_1_joint


                action_info_.type 分配如下：
                1： timedelay           

                //joint运动端                
                12：左臂arm
                13：右臂arm
                14：双臂运动

                //ptp运动端
                22：左臂arm
                23：右臂arm
                24：双臂arm

                //pline运动端
                32：左臂arm
                33：右臂arm
                34：双臂arm   
                
                //hand joint move
                42: 左手
                43：右手
            

                //head joint move
                52：neck
                53: head
                54: neck / head

                //提升电机
                61：提升电机使能 commandid=1
                62：abs move commandid=5
            
            */
            static bool getArmMotionCmd(const action_info_ &par , exec_cmd_config_& command )
            {
                if(par.type != 12 && par.type != 13 && par.type != 14 && 
                    par.type != 22 && par.type != 23 && par.type != 24 && 
                    par.type != 32 && par.type != 33 && par.type != 34 && 
                    par.type != 42 && par.type != 43   && 
                    par.type != 52 && par.type != 53 && par.type != 54               
                
                )
                {
                    return false;
                }

                //获取group
                if(par.type == 12 || par.type == 22 || par.type == 32)
                {
                    command.group = "l_arm";
                }
                else if(par.type == 13 || par.type == 23 || par.type == 33)
                {
                    command.group = "r_arm";
                }
                else if(par.type == 14 || par.type == 24 || par.type == 34)
                {
                    command.group = "dual_arms";
                }
                else if(par.type == 42 )
                {
                    command.group = "l_hand";
                }
                else if(par.type == 43 )
                {
                    command.group = "r_hand";
                }
                else if(par.type == 52 || par.type == 53 || par.type == 54)
                {
                    command.group = "head";
                }

                command.execute_motion = true; //执行
                command.wait_for_result = true; //等待结果
                command.timeout = 30.0; //延时时长

                switch(par.type )
                {
                    case 12: //左臂joint 运动
                        command.goal_type = "joints";
                        command.links="left_hand";
                        command.pipeline_id = "ompl";
                        command.planner_id = "RRTConnectkConfigDefault";
                        command.joints.clear();
                        {
                            joint_name_value_ j1;
                            j1.joint_name = "left_arm_1_joint";
                            j1.value = par.info_.single_arm_joint.joint1_rad;
                            command.joints.push_back(j1);
                        }
                        {
                            joint_name_value_ j2;
                            j2.joint_name = "left_arm_2_joint";
                            j2.value = par.info_.single_arm_joint.joint2_rad;
                            command.joints.push_back(j2);
                        }
                        {
                            joint_name_value_ j3;
                            j3.joint_name = "left_arm_3_joint";
                            j3.value = par.info_.single_arm_joint.joint3_rad;
                            command.joints.push_back(j3);
                        }
                        {
                            joint_name_value_ j4;
                            j4.joint_name = "left_arm_4_joint";
                            j4.value = par.info_.single_arm_joint.joint4_rad;
                            command.joints.push_back(j4);
                        }
                        {
                            joint_name_value_ j5;
                            j5.joint_name = "left_arm_5_joint";
                            j5.value = par.info_.single_arm_joint.joint5_rad;
                            command.joints.push_back(j5);
                        }
                        {
                            joint_name_value_ j6;
                            j6.joint_name = "left_arm_6_joint";
                            j6.value = par.info_.single_arm_joint.joint6_rad;
                            command.joints.push_back(j6);
                        }
                        {
                            joint_name_value_ j7;
                            j7.joint_name = "left_arm_7_joint";
                            j7.value = par.info_.single_arm_joint.joint7_rad;
                            command.joints.push_back(j7);
                        }
                        break;

                    case 13: //右臂joint
                        command.goal_type = "joints";                        
                        command.pipeline_id = "ompl";
                        command.links="right_hand";
                        command.planner_id = "RRTConnectkConfigDefault";
                        command.joints.clear();
                        {
                            joint_name_value_ j1;
                            j1.joint_name = "right_arm_1_joint";
                            j1.value = par.info_.single_arm_joint.joint1_rad;
                            command.joints.push_back(j1);
                        }
                        {
                            joint_name_value_ j2;
                            j2.joint_name = "right_arm_2_joint";
                            j2.value = par.info_.single_arm_joint.joint2_rad;
                            command.joints.push_back(j2);
                        }
                        {
                            joint_name_value_ j3;
                            j3.joint_name = "right_arm_3_joint";
                            j3.value = par.info_.single_arm_joint.joint3_rad;
                            command.joints.push_back(j3);
                        }
                        {
                            joint_name_value_ j4;
                            j4.joint_name = "right_arm_4_joint";
                            j4.value = par.info_.single_arm_joint.joint4_rad;
                            command.joints.push_back(j4);
                        }
                        {
                            joint_name_value_ j5;
                            j5.joint_name = "right_arm_5_joint";
                            j5.value = par.info_.single_arm_joint.joint5_rad;
                            command.joints.push_back(j5);
                        }
                        {
                            joint_name_value_ j6;
                            j6.joint_name = "right_arm_6_joint";
                            j6.value = par.info_.single_arm_joint.joint6_rad;
                            command.joints.push_back(j6);
                        }
                        {
                            joint_name_value_ j7;
                            j7.joint_name = "right_arm_7_joint";
                            j7.value = par.info_.single_arm_joint.joint7_rad;
                            command.joints.push_back(j7);
                        }
                        break;

                    case 14:
                        command.goal_type = "joints";                        
                        command.pipeline_id = "ompl";
                        command.planner_id = "RRTConnectkConfigDefault";
                        command.links="left_hand,right_hand";
                        command.joints.clear();
                        {
                            joint_name_value_ j1;
                            j1.joint_name = "right_arm_1_joint";
                            j1.value = par.info_.dual_arm_joint.armR_joint1_rad;
                            command.joints.push_back(j1);
                        }
                        {
                            joint_name_value_ j2;
                            j2.joint_name = "right_arm_2_joint";
                            j2.value = par.info_.dual_arm_joint.armR_joint2_rad;
                            command.joints.push_back(j2);
                        }
                        {
                            joint_name_value_ j3;
                            j3.joint_name = "right_arm_3_joint";
                            j3.value = par.info_.dual_arm_joint.armR_joint3_rad;
                            command.joints.push_back(j3);
                        }
                        {
                            joint_name_value_ j4;
                            j4.joint_name = "right_arm_4_joint";
                            j4.value = par.info_.dual_arm_joint.armR_joint4_rad;
                            command.joints.push_back(j4);
                        }
                        {
                            joint_name_value_ j5;
                            j5.joint_name = "right_arm_5_joint";
                            j5.value = par.info_.dual_arm_joint.armR_joint5_rad;
                            command.joints.push_back(j5);
                        }
                        {
                            joint_name_value_ j6;
                            j6.joint_name = "right_arm_6_joint";
                            j6.value = par.info_.dual_arm_joint.armR_joint6_rad;
                            command.joints.push_back(j6);
                        }
                        {
                            joint_name_value_ j7;
                            j7.joint_name = "right_arm_7_joint";
                            j7.value = par.info_.dual_arm_joint.armR_joint7_rad;
                            command.joints.push_back(j7);
                        }
                        {
                            joint_name_value_ j8;
                            j8.joint_name = "left_arm_1_joint";
                            j8.value = par.info_.dual_arm_joint.armL_joint1_rad;
                            command.joints.push_back(j8);
                        }
                        {
                            joint_name_value_ j9;
                            j9.joint_name = "left_arm_2_joint";
                            j9.value = par.info_.dual_arm_joint.armL_joint2_rad;
                            command.joints.push_back(j9);
                        }
                        {
                            joint_name_value_ j10;
                            j10.joint_name = "left_arm_3_joint";
                            j10.value = par.info_.dual_arm_joint.armL_joint3_rad;
                            command.joints.push_back(j10);
                        }
                        {
                            joint_name_value_ j11;
                            j11.joint_name = "left_arm_4_joint";
                            j11.value = par.info_.dual_arm_joint.armL_joint4_rad;
                            command.joints.push_back(j11);
                        }
                        {
                            joint_name_value_ j12;
                            j12.joint_name = "left_arm_5_joint";
                            j12.value = par.info_.dual_arm_joint.armL_joint5_rad;
                            command.joints.push_back(j12);
                        }
                        {
                            joint_name_value_ j13;
                            j13.joint_name = "left_arm_6_joint";
                            j13.value = par.info_.dual_arm_joint.armL_joint6_rad;
                            command.joints.push_back(j13);
                        }
                        {
                            joint_name_value_ j14;
                            j14.joint_name = "left_arm_7_joint";
                            j14.value = par.info_.dual_arm_joint.armL_joint7_rad;
                            command.joints.push_back(j14);
                        }
                        break;
                    //ptp
                    case 22: //左臂
                        command.goal_type = "pose";    
                        command.links = "left_hand";                    
                        command.pipeline_id = "pilz_industrial_motion_planner";
                        command.planner_id = "PTP";
                        {
                            std::stringstream ss;
                            for (size_t i = 0; i < 7; ++i)
                            {
                                if (i > 0) ss << ",";
                                ss << par.info_.action_data.d[i] ;
                            }
                            command.pose = ss.str();
                        }
                        
                        break;

                    case 23: //右臂
                        command.goal_type = "pose";  
                        command.links = "right_hand";                       
                        command.pipeline_id = "pilz_industrial_motion_planner";
                        command.planner_id = "PTP";
                        {
                            std::stringstream ss;
                            for (size_t i = 0; i < 7; ++i)
                            {
                                if (i > 0) ss << ",";
                                ss << par.info_.action_data.d[i] ;
                            }
                            command.pose = ss.str();
                        }
                        break;

                    case 24: //双臂
                        command.goal_type = "multi_pose";   
                        command.links = "right_hand,left_hand";                       
                        command.pipeline_id = "pilz_industrial_motion_planner";
                        command.planner_id = "PTP";
                        {
                            std::stringstream ss;
                            for (size_t i = 0; i < 7; ++i)
                            {
                                if (i > 0) ss << ",";
                                ss << par.info_.action_data.d[i] ;
                            }
                            ss << ";";
                            for (size_t i = 7; i < 14; ++i)
                            {
                                if (i > 7) ss << ",";
                                ss << par.info_.action_data.d[i] ;
                            }
                            command.pose = ss.str();
                        }
                        break;

                    //pline    
                    case 32: //左臂
                        command.goal_type = "pose";     
                        command.links = "left_hand";                     
                        command.pipeline_id = "pilz_industrial_motion_planner";
                        command.planner_id = "lin";
                        command.lin_position_only =1;
                        {
                            std::stringstream ss;
                            for (size_t i = 0; i < 7; ++i)
                            {
                                if (i > 0) ss << ",";
                                ss << par.info_.action_data.d[i] ;
                            }
                            command.pose = ss.str();
                        }
                      
                        break;
                    case 33: //右臂
                        command.goal_type = "pose";      
                        command.links = "right_hand";                    
                        command.pipeline_id = "pilz_industrial_motion_planner";
                        command.planner_id = "lin";
                        command.lin_position_only =1;
                        {
                            std::stringstream ss;
                            for (size_t i = 0; i < 7; ++i)
                            {
                                if (i > 0) ss << ",";
                                ss << par.info_.action_data.d[i] ;
                            }
                            command.pose = ss.str();
                        }
                        break;
                    case 34: //双臂
                        command.goal_type = "pose"; 
                        command.links = "right_hand,left_hand";                         
                        command.pipeline_id = "pilz_industrial_motion_planner";
                        command.planner_id = "lin";
                        command.lin_position_only =1;
                        {
                            std::stringstream ss;
                            for (size_t i = 0; i < 7; ++i)
                            {
                                if (i > 0) ss << ",";
                                ss << par.info_.action_data.d[i] ;
                            }
                            ss << ";";
                            for (size_t i = 7; i < 14; ++i)
                            {
                                if (i > 7) ss << ",";
                                ss << par.info_.action_data.d[i] ;
                            }
                            command.pose = ss.str();
                        }
                        break;
                    
                    case 42: //左手
                        command.goal_type = "joints"; 
                        command.joints.clear();
                        {
                            joint_name_value_ j1;
                            j1.joint_name = "left_hand_virtual_joint";
                            j1.value = par.info_.Axis_servo_pose.dis;
                            command.joints.push_back(j1);
                        }
                        break;

                    case 43: //右手
                        command.goal_type = "joints"; 
                        command.joints.clear();
                        {
                            joint_name_value_ j1;
                            j1.joint_name = "right_hand_virtual_joint";
                            j1.value = par.info_.Axis_servo_pose.dis;
                            command.joints.push_back(j1);
                        }
                        break;

                    case 52://neck
                        command.goal_type = "joints"; 
                        command.joints.clear();
                        {
                            joint_name_value_ j1;
                            j1.joint_name = "neck_joint";
                            j1.value = par.info_.Axis_servo_pose.dis;
                            command.joints.push_back(j1);
                        }
                        break;
                        
                    case 53://head
                        command.goal_type = "joints"; 
                        command.joints.clear();
                        {
                            joint_name_value_ j1;
                            j1.joint_name = "head_joint";
                            j1.value = par.info_.Axis_servo_pose.dis;
                            command.joints.push_back(j1);
                        }
                        break;

                    case 54://head
                        command.goal_type = "joints"; 
                        command.joints.clear();
                        {
                            joint_name_value_ j1;
                            j1.joint_name = "head_joint";
                            j1.value = par.info_.Axis_servo_pose.dis;
                            command.joints.push_back(j1);
                        }
                        {
                            joint_name_value_ j2;
                            j2.joint_name = "head_joint";
                            j2.value = par.info_.Axis_servo_pose.dis;
                            command.joints.push_back(j2);
                        }
                        break;
                        
                    default:
                        return false;
                        

                }

                

                return true;
            };

           
            #define CODEAXIS_RATIO_RPM_LIFTSERVO 65536.0
            #define RATIO_VEL_DEC_RPM_LIFTSERVO 512.0*CODEAXIS_RATIO_RPM_LIFTSERVO/1875
            #define RATIO_ACC_DEC_RPS_LIFTSERVO 65536.0*CODEAXIS_RATIO_RPM_LIFTSERVO/4000000


            #define SERVO_POWERON 1 
            #define SERVO_POWEROFF 2
            #define SERVO_RSET 3
            #define SERVO_SETVEL 4
            #define SERVO_MVPOS_ABS 5
            #define SERVO_STOP 6

            static bool getliftMotionCmd(const action_info_ &par , uint32_t &req ,srv_servo_cmd_request_ & command )
            {
                if(par.type != 61 && par.type !=62)
                {
                    return false;
                }
                switch(par.type )
                {
                    case 61:
                        command.master_name = "can0";
                        command.command_id = SERVO_POWERON;
                        command.node_id = 1;
                        command.seq =req;
                        break;

                    case 62:
                        command.master_name = "can0";
                        command.command_id = SERVO_MVPOS_ABS;
                        command.aim_pos = par.info_.Axis_servo_pose.dis *CODEAXIS_RATIO_RPM_LIFTSERVO;
                        command.aim_vel = par.info_.Axis_servo_pose.vel * RATIO_VEL_DEC_RPM_LIFTSERVO;
                        command.dec = par.info_.Axis_servo_pose.dec * RATIO_ACC_DEC_RPS_LIFTSERVO;
                        command.node_id = 1;
                        command.seq =req;
                        break;

                    default:
                        break;
                }


                return true;
            };

            // void switch_code2dis_liftservo(float &src, float &dec)
            // {
            //     dec = src /RATIO_ROTATE2LINE_M2R_LIFTSERVO/CODEAXIS_RATIO_RPM_LIFTSERVO;
            // };
    };

//获取制定
#define GET_REG_BIT_FAST(val, bit)  (((val) & (1U << (bit))) != 0)
// 置1指定bit
#define SET_REG_BIT(val, bit)      ((val) | (1U << (bit)))
// 清零指定bit
#define CLR_REG_BIT(val, bit)      ((val) & ~(1U << (bit)))

            




//////////////modbus 通信保持型数据
        struct modbus_btn_reg0_
        {
            bool btn_2moveit;//无动力拖动or op
            bool btn_2reset; //复位
            bool btn_2stop; //停机
            bool btn_2next; //单步运行
            bool btn_2start;//开始运行
            bool btn_lift_2run;//提升电机运行
            bool btn_rotat_2run;//旋转电机运行
            bool btn_head_2run;//头部运行
            bool btn_handL_2run;//手部运行
            bool btn_handR_2run;//手部运行
            bool btn_armL_2run;//手部运行
            bool btn_armR_2run;//手部运行
            bool sw_type_armrun;//高为pline di为ptp
        };
        struct modbus_btn_reg1_
        {
            bool bfalg_isAuto;//无动力拖动or op
            bool bfalg_AutoTask_done; //复位
            bool bfalg_isManu; //停机
            bool bfalg_ManuTask_done; //单步运行

        };

        #define FLOATPAR_AIMPOS_START 83 //aimpos 开始索引 
#pragma pack(push, 1)
         struct modbus_hold_date_
        {
            uint16_t btn_reg[10];//系统总状态机 0-9
            uint16_t sta_system;//系统总状态机 10
            uint16_t sta_action_disp;//运动状态 11
            uint16_t sta_sub_action;//运动子状态 12
            uint16_t errid_system;//系统总报警id 13
            uint16_t errid_arm;//arm报警id 14
            uint16_t errid_axis_lift;//升降电机报警 15
            uint16_t set_mode_run;// 16 1 自动运行  2 手动  
            uint16_t sta_action_set; //执行指令 17
            uint16_t sta_manu_disp; //执行指令 18
            uint16_t type_manu_disp; //手动操作类型 19
            uint16_t system_heartbeat_cnt; //系统运行心跳计数器 20
            uint16_t currpos_arml_1_1; // 当前位置信息，左臂1关节 21
            uint16_t currpos_arml_1_2; // 当前位置信息，左臂1关节 22
            uint16_t currpos_arml_2_1; // 当前位置信息，左臂1关节 23
            uint16_t currpos_arml_2_2; // 当前位置信息，左臂1关节 24
            uint16_t currpos_arml_3_1; // 当前位置信息，左臂1关节 25
            uint16_t currpos_arml_3_2; // 当前位置信息，左臂1关节 26
            uint16_t currpos_arml_4_1; // 当前位置信息，左臂1关节 27
            uint16_t currpos_arml_4_2; // 当前位置信息，左臂1关节 28
            uint16_t currpos_arml_5_1; // 当前位置信息，左臂1关节 29
            uint16_t currpos_arml_5_2; // 当前位置信息，左臂1关节 30
            uint16_t currpos_arml_6_1; // 当前位置信息，左臂1关节 31
            uint16_t currpos_arml_6_2; // 当前位置信息，左臂1关节 32
            uint16_t currpos_arml_7_1; // 当前位置信息，左臂1关节 33
            uint16_t currpos_arml_7_2; // 当前位置信息，左臂1关节 34

            uint16_t currpos_armr_1_1; // 当前位置信息，左臂1关节 35
            uint16_t currpos_armr_1_2; // 当前位置信息，左臂1关节 36
            uint16_t currpos_armr_2_1; // 当前位置信息，左臂1关节 37
            uint16_t currpos_armr_2_2; // 当前位置信息，左臂1关节 38
            uint16_t currpos_armr_3_1; // 当前位置信息，左臂1关节 39
            uint16_t currpos_armr_3_2; // 当前位置信息，左臂1关节 40
            uint16_t currpos_armr_4_1; // 当前位置信息，左臂1关节 41
            uint16_t currpos_armr_4_2; // 当前位置信息，左臂1关节 42
            uint16_t currpos_armr_5_1; // 当前位置信息，左臂1关节 43
            uint16_t currpos_armr_5_2; // 当前位置信息，左臂1关节 44
            uint16_t currpos_armr_6_1; // 当前位置信息，左臂1关节 45
            uint16_t currpos_armr_6_2; // 当前位置信息，左臂1关节 46
            uint16_t currpos_armr_7_1; // 当前位置信息，左臂1关节 47
            uint16_t currpos_armr_7_2; // 当前位置信息，左臂1关节 48

            uint16_t currpos_handl_1_1; // 当前位置信息，左手 49
            uint16_t currpos_handl_1_2; // 当前位置信息，左手 50
            uint16_t currpos_handr_1_1; // 当前位置信息，右手 51
            uint16_t currpos_handr_1_2; // 当前位置信息，右手 52
            uint16_t currpos_head_1_1; // 当前位置信息，左手 53
            uint16_t currpos_head_1_2; // 当前位置信息，左手 54
            uint16_t currpos_rotate_1_1; // 当前位置信息，左手 55
            uint16_t currpos_rotate_1_2; // 当前位置信息，左手 56
            uint16_t currpos_lift_1_1; // 当前位置信息，左手 57
            uint16_t currpos_lift_1_2; // 当前位置信息，左手 58

            uint16_t currpos_arml_x_1; // 当前位置信息，左臂1关节 59
            uint16_t currpos_arml_x_2; // 当前位置信息，左臂1关节 60
            uint16_t currpos_arml_y_1; // 当前位置信息，左臂1关节 61
            uint16_t currpos_arml_y_2; // 当前位置信息，左臂1关节 62
            uint16_t currpos_arml_z_1; // 当前位置信息，左臂1关节 63
            uint16_t currpos_arml_z_2; // 当前位置信息，左臂1关节 64
            uint16_t currpos_arml_rx_1; // 当前位置信息，左臂1关节 65
            uint16_t currpos_arml_rx_2; // 当前位置信息，左臂1关节 66
            uint16_t currpos_arml_ry_1; // 当前位置信息，左臂1关节 67
            uint16_t currpos_arml_ry_2; // 当前位置信息，左臂1关节 68
            uint16_t currpos_arml_rz_1; // 当前位置信息，左臂1关节 69
            uint16_t currpos_arml_rz_2; // 当前位置信息，左臂1关节 70

            uint16_t currpos_armr_x_1; // 当前位置信息，左臂1关节 71
            uint16_t currpos_armr_x_2; // 当前位置信息，左臂1关节 72
            uint16_t currpos_armr_y_1; // 当前位置信息，左臂1关节 73
            uint16_t currpos_armr_y_2; // 当前位置信息，左臂1关节 74
            uint16_t currpos_armr_z_1; // 当前位置信息，左臂1关节 75
            uint16_t currpos_armr_z_2; // 当前位置信息，左臂1关节 76
            uint16_t currpos_armr_rx_1; // 当前位置信息，左臂1关节 77
            uint16_t currpos_armr_rx_2; // 当前位置信息，左臂1关节 78
            uint16_t currpos_armr_ry_1; // 当前位置信息，左臂1关节 79
            uint16_t currpos_armr_ry_2; // 当前位置信息，左臂1关节 80
            uint16_t currpos_armr_rz_1; // 当前位置信息，左臂1关节 81
            uint16_t currpos_armr_rz_2; // 当前位置信息，左臂1关节 82

            uint16_t aimpos_handl_1_1; // 当前位置信息，左手 83
            uint16_t aimpos_handl_1_2; // 当前位置信息，左手 84
            uint16_t aimpos_handr_1_1; // 当前位置信息，右手 85
            uint16_t aimpos_handr_1_2; // 当前位置信息，右手 86
            uint16_t aimpos_head_1_1; // 当前位置信息，左手 87
            uint16_t aimpos_head_1_2; // 当前位置信息，左手 88
            uint16_t aimpos_rotate_1_1; // 当前位置信息，左手 89
            uint16_t aimpos_rotate_1_2; // 当前位置信息，左手 90
            uint16_t aimpos_lift_1_1; // 当前位置信息，左手 91
            uint16_t aimpos_lift_1_2; // 当前位置信息，左手 92

            uint16_t aimpos_arml_x_1; // 当前位置信息，左臂1关节 93
            uint16_t aimpos_arml_x_2; // 当前位置信息，左臂1关节 94
            uint16_t aimpos_arml_y_1; // 当前位置信息，左臂1关节 95
            uint16_t aimpos_arml_y_2; // 当前位置信息，左臂1关节 96
            uint16_t aimpos_arml_z_1; // 当前位置信息，左臂1关节 97
            uint16_t aimpos_arml_z_2; // 当前位置信息，左臂1关节 98
            uint16_t aimpos_arml_rx_1; // 当前位置信息，左臂1关节 99
            uint16_t aimpos_arml_rx_2; // 当前位置信息，左臂1关节 100
            uint16_t aimpos_arml_ry_1; // 当前位置信息，左臂1关节 101
            uint16_t aimpos_arml_ry_2; // 当前位置信息，左臂1关节 102
            uint16_t aimpos_arml_rz_1; // 当前位置信息，左臂1关节 103
            uint16_t aimpos_arml_rz_2; // 当前位置信息，左臂1关节 104

            uint16_t aimpos_armr_x_1; // 当前位置信息，左臂1关节 105
            uint16_t aimpos_armr_x_2; // 当前位置信息，左臂1关节 106
            uint16_t aimpos_armr_y_1; // 当前位置信息，左臂1关节 107
            uint16_t aimpos_armr_y_2; // 当前位置信息，左臂1关节 108
            uint16_t aimpos_armr_z_1; // 当前位置信息，左臂1关节 109
            uint16_t aimpos_armr_z_2; // 当前位置信息，左臂1关节 110
            uint16_t aimpos_armr_rx_1; // 当前位置信息，左臂1关节 111
            uint16_t aimpos_armr_rx_2; // 当前位置信息，左臂1关节 112
            uint16_t aimpos_armr_ry_1; // 当前位置信息，左臂1关节 113
            uint16_t aimpos_armr_ry_2; // 当前位置信息，左臂1关节 114
            uint16_t aimpos_armr_rz_1; // 当前位置信息，左臂1关节 115
            uint16_t aimpos_armr_rz_2; // 当前位置信息，左臂1关节 116

        };

        union hold_reg_
        {
            uint16_t data[REGLEN_MAX_HOLD];//
            modbus_hold_date_ Reg;
        }; 

#pragma pack(pop) // 恢复默认对齐
        #define FLOATPAR_LEN 17 
        // 1字节紧凑对齐，关闭填充
#pragma pack(push, 1)
        struct float_getpar_
        {
            float aim_handl;
            float aim_handr;
            float aim_head;
            float aim_rotate;
            float aim_lift;
            float aim_arml_x;
            float aim_arml_y;
            float aim_arml_z;
            float aim_arml_rx;
            float aim_arml_ry;
            float aim_arml_rz;
            float aim_armr_x;
            float aim_armr_y;
            float aim_armr_z;
            float aim_armr_rx;
            float aim_armr_ry;
            float aim_armr_rz;
        };

        union getpar_float_
        {
            float data[FLOATPAR_LEN];
            float_getpar_ par;
        }; 
#pragma pack(pop) // 恢复默认对齐

    class mdreg_data
    {
        public:
            mdreg_data(){};
            ~mdreg_data(){};            
            hold_reg_ hold_reg;//写入数据
            getpar_float_ getfloat_par;//将数据转换为float
            modbus_btn_reg0_ hold_BtnReg0_curr,hold_BtnReg0_last ;//reg 1 实时数据
            modbus_btn_reg1_ hold_BtnReg1_cur;            
            pose_ out_armL,out_armR;//用于显示左右臂位姿
            float out_rotate,out_head,out_handL,out_handR;
            pose_ in_armL,in_armR;//用于设置左右臂位姿
            float in_lift,in_rotate,in_head,in_handL,in_handR;
            
            

            void hold_reg_last_save()
            {
                hold_BtnReg0_last = hold_BtnReg0_curr;
            };

            void xch_word2bool_holdreg1()
            {
                hold_BtnReg0_curr.btn_2moveit = GET_REG_BIT_FAST(hold_reg.data[0],0);
                hold_BtnReg0_curr.btn_2reset = GET_REG_BIT_FAST(hold_reg.data[0],1);
                hold_BtnReg0_curr.btn_2stop = GET_REG_BIT_FAST(hold_reg.data[0],2);
                hold_BtnReg0_curr.btn_2next = GET_REG_BIT_FAST(hold_reg.data[0],3);
                hold_BtnReg0_curr.btn_2start = GET_REG_BIT_FAST(hold_reg.data[0],4);
                hold_BtnReg0_curr.btn_armL_2run = GET_REG_BIT_FAST(hold_reg.data[0],5);
                hold_BtnReg0_curr.btn_armR_2run = GET_REG_BIT_FAST(hold_reg.data[0],6);
                hold_BtnReg0_curr.btn_handL_2run = GET_REG_BIT_FAST(hold_reg.data[0],7);
                hold_BtnReg0_curr.btn_handR_2run = GET_REG_BIT_FAST(hold_reg.data[0],8);
                hold_BtnReg0_curr.btn_head_2run = GET_REG_BIT_FAST(hold_reg.data[0],9);
                hold_BtnReg0_curr.btn_lift_2run = GET_REG_BIT_FAST(hold_reg.data[0],10);
                hold_BtnReg0_curr.btn_rotat_2run = GET_REG_BIT_FAST(hold_reg.data[0],11);

            };

            void write_float_to_word(float& fvalue, uint16_t& value_L,uint16_t& value_H)
            {
                union {
                    float f;
                    uint8_t bytes[4];
                } u;
                u.f = fvalue;

                // 大端模式：
                // 寄存器1（base_addr）：高16位（u.bytes[0], u.bytes[1]）
                // 寄存器2（base_addr+1）：低16位（u.bytes[2], u.bytes[3]）
                value_L = (static_cast<uint16_t>(u.bytes[1]) << 8) | u.bytes[0];
                value_H = (static_cast<uint16_t>(u.bytes[3]) << 8) | u.bytes[2];

            };

            float read_float_from_word(uint16_t& value_L,uint16_t& value_H)
            {
                uint16_t reg_high = value_L;
                uint16_t reg_low  = value_H;

                union {
                    float f;
                    uint8_t bytes[4];
                } u;

                // 大端模式还原
                u.bytes[1] = (reg_high >> 8) & 0xFF;
                u.bytes[0] = reg_high & 0xFF;
                u.bytes[3] = (reg_low >> 8) & 0xFF;
                u.bytes[2] = reg_low & 0xFF;

                return u.f;

            };

            void download_floatpar()
            {
                for(int8_t i =0 ;i <FLOATPAR_LEN ;i++)
                {
                    getfloat_par.data[i] = read_float_from_word(hold_reg.data[FLOATPAR_AIMPOS_START+2*i],hold_reg.data[FLOATPAR_AIMPOS_START+2*i+1]);
                  
                }

            };






    };

}


