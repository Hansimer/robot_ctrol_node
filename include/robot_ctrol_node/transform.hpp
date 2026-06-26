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

            #define RATIO_ROTATE2LINE_M2R_LIFTSERVO 1.0
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
                        command.aim_pos = par.info_.Axis_servo_pose.dis * RATIO_ROTATE2LINE_M2R_LIFTSERVO*CODEAXIS_RATIO_RPM_LIFTSERVO;
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

            void switch_code2dis_liftservo(float &src, float &dec)
            {
                dec = src /RATIO_ROTATE2LINE_M2R_LIFTSERVO/CODEAXIS_RATIO_RPM_LIFTSERVO;
            };
    };

//获取制定
#define GET_REG_BIT_FAST(val, bit)  (((val) & (1U << (bit))) != 0)
// 置1指定bit
#define SET_REG_BIT(val, bit)      ((val) | (1U << (bit)))
// 清零指定bit
#define CLR_REG_BIT(val, bit)      ((val) & ~(1U << (bit)))
     struct modbus_input_date_
        {
            uint16_t sta_system;//系统总状态机 1
            uint16_t sta_action;//运动状态 2
            uint16_t sta_sub_action;//运动子状态 3
            uint16_t errid_system;//系统总报警id 4
            uint16_t errid_arm;//arm报警id 5
            uint16_t errid_axis_lift;//升降电机报警 6
            uint16_t errid_servers1; //7
            uint16_t errid_servers2; //8
            uint16_t errid_servers3; //9
            /////////////////左臂
            uint16_t pos_arml_x_f2w_1;//左臂位置信息10
            uint16_t pos_arml_x_f2w_2;//左臂位置信息11
            uint16_t pos_arml_y_f2w_1;//左臂位置信息12
            uint16_t pos_arml_y_f2w_2;//左臂位置信息13
            uint16_t pos_arml_z_f2w_1;//左臂位置信息14
            uint16_t pos_arml_z_f2w_2;//左臂位置信息15
            uint16_t pos_arml_rx_f2w_1;//左臂位置信息16
            uint16_t pos_arml_rx_f2w_2;//左臂位置信息17
            uint16_t pos_arml_ry_f2w_1;//左臂位置信息18
            uint16_t pos_arml_ry_f2w_2;//左臂位置信息19
            uint16_t pos_arml_rz_f2w_1;//左臂位置信息20
            uint16_t pos_arml_rz_f2w_2;//左臂位置信息21
            /////////////////////右臂
            uint16_t pos_armr_x_f2w_1;//左臂位置信息22
            uint16_t pos_armr_x_f2w_2;//左臂位置信息23
            uint16_t pos_armr_y_f2w_1;//左臂位置信息24
            uint16_t pos_armr_y_f2w_2;//左臂位置信息25
            uint16_t pos_armr_z_f2w_1;//左臂位置信息26
            uint16_t pos_armr_z_f2w_2;//左臂位置信息27
            uint16_t pos_armr_rx_f2w_1;//左臂位置信息28
            uint16_t pos_armr_rx_f2w_2;//左臂位置信息29
            uint16_t pos_armr_ry_f2w_1;//左臂位置信息30
            uint16_t pos_armr_ry_f2w_2;//左臂位置信息31
            uint16_t pos_armr_rz_f2w_1;//左臂位置信息32
            uint16_t pos_armr_rz_f2w_2;//左臂位置信息33

            //提升电机位置
            uint16_t pos_servo_lift_f2w_1;//左臂位置信息34
            uint16_t pos_servo_lift_f2w_2;//左臂位置信息35

            //旋转
            uint16_t pos_servo_rotate_f2w_1;//左臂位置信息36
            uint16_t pos_servo_rotate_f2w_2;//左臂位置信息37

            //头部
            uint16_t pos_servo_head_f2w_1;//左臂位置信息38
            uint16_t pos_servo_head_f2w_2;//左臂位置信息39

            //左手
            uint16_t pos_servo_handL_f2w_1;//左臂位置信息40
            uint16_t pos_servo_handL_f2w_2;//左臂位置信息41

            //右手
            uint16_t pos_servo_handR_f2w_1;//左臂位置信息42
            uint16_t pos_servo_handR_f2w_2;//左臂位置信息43

        };

        union input_reg_
        {
            uint16_t data[REGLEN_MAX_INPUT];
            modbus_input_date_ Reg;
        }; 

//////////////modbus 通信保持型数据
        struct modbus_btn_reg1_
        {
            bool bfalg_2moveit;//无动力拖动or op
            bool bfalg_2reset; //复位
            bool bfalg_2stop; //停机
            bool bflag_2next; //单步运行
            bool bflag_2start;//开始运行
            bool btn_lift_2run;//提升电机运行
            bool btn_rotat_2run;//旋转电机运行
            bool btn_head_2run;//头部运行
            bool btn_handL_2run;//手部运行
            bool btn_handR_2run;//手部运行
            bool btn_armL_2run;//手部运行
            bool btn_armR_2run;//手部运行
        };


         struct modbus_hold_date_
        {
            uint16_t btn_reg[8];//系统总状态机 0-7 
            uint16_t set_mode_run;// 1 自动运行  2 手动  3 半自动 8
            uint16_t set_cmd; //执行指令 9

            uint16_t pos_arml_x_f2w_1;//左臂位置信息10
            uint16_t pos_arml_x_f2w_2;//左臂位置信息11
            uint16_t pos_arml_y_f2w_1;//左臂位置信息12
            uint16_t pos_arml_y_f2w_2;//左臂位置信息13
            uint16_t pos_arml_z_f2w_1;//左臂位置信息14
            uint16_t pos_arml_z_f2w_2;//左臂位置信息15
            uint16_t pos_arml_rx_f2w_1;//左臂位置信息16
            uint16_t pos_arml_rx_f2w_2;//左臂位置信息17
            uint16_t pos_arml_ry_f2w_1;//左臂位置信息18
            uint16_t pos_arml_ry_f2w_2;//左臂位置信息19
            uint16_t pos_arml_rz_f2w_1;//左臂位置信息20
            uint16_t pos_arml_rz_f2w_2;//左臂位置信息21
            /////////////////////右臂
            uint16_t pos_armr_x_f2w_1;//左臂位置信息22
            uint16_t pos_armr_x_f2w_2;//左臂位置信息23
            uint16_t pos_armr_y_f2w_1;//左臂位置信息24
            uint16_t pos_armr_y_f2w_2;//左臂位置信息25
            uint16_t pos_armr_z_f2w_1;//左臂位置信息26
            uint16_t pos_armr_z_f2w_2;//左臂位置信息27
            uint16_t pos_armr_rx_f2w_1;//左臂位置信息28
            uint16_t pos_armr_rx_f2w_2;//左臂位置信息29
            uint16_t pos_armr_ry_f2w_1;//左臂位置信息30
            uint16_t pos_armr_ry_f2w_2;//左臂位置信息31
            uint16_t pos_armr_rz_f2w_1;//左臂位置信息32
            uint16_t pos_armr_rz_f2w_2;//左臂位置信息33

            //提升电机位置
            uint16_t pos_servo_lift_f2w_1;//左臂位置信息34
            uint16_t pos_servo_lift_f2w_2;//左臂位置信息35

            //旋转
            uint16_t pos_servo_rotate_f2w_1;//左臂位置信息36
            uint16_t pos_servo_rotate_f2w_2;//左臂位置信息37

            //头部
            uint16_t pos_servo_head_f2w_1;//左臂位置信息38
            uint16_t pos_servo_head_f2w_2;//左臂位置信息39

            //左手
            uint16_t pos_servo_handL_f2w_1;//左臂位置信息40
            uint16_t pos_servo_handL_f2w_2;//左臂位置信息41

            //右手
            uint16_t pos_servo_handR_f2w_1;//左臂位置信息42
            uint16_t pos_servo_handR_f2w_2;//左臂位置信息43
            


        };

        union hold_reg_
        {
            uint16_t data[REGLEN_MAX_HOLD];
            modbus_hold_date_ Reg;
        }; 


    class mdreg_data
    {
        public:
            mdreg_data(){};
            ~mdreg_data(){};
            input_reg_ show_reg;//更新显示数据
            hold_reg_ hold_reg;//写入数据
            hold_reg_ hold_reg_last;//写入数据
            modbus_btn_reg1_ hold_reg1_curr,hold_reg1_last ;//reg 1 实时数据            
            pose_ out_armL,out_armR;//用于显示左右臂位姿
            float out_rotate,out_head,out_handL,out_handR;
            pose_ in_armL,in_armR;//用于设置左右臂位姿
            float in_lift,in_rotate,in_head,in_handL,in_handR;
            
            

            void hold_reg_last_save()
            {
                hold_reg_last = hold_reg;
                hold_reg1_last = hold_reg1_curr;
            };

            void xch_word2bool_holdreg1()
            {
                hold_reg1_curr.bfalg_2moveit = GET_REG_BIT_FAST(hold_reg.data[0],0);
                hold_reg1_curr.bfalg_2reset = GET_REG_BIT_FAST(hold_reg.data[0],1);
                hold_reg1_curr.bfalg_2stop = GET_REG_BIT_FAST(hold_reg.data[0],2);
                hold_reg1_curr.bflag_2next = GET_REG_BIT_FAST(hold_reg.data[0],3);
                hold_reg1_curr.bflag_2start = GET_REG_BIT_FAST(hold_reg.data[0],4);
                hold_reg1_curr.btn_armL_2run = GET_REG_BIT_FAST(hold_reg.data[0],5);
                hold_reg1_curr.btn_armR_2run = GET_REG_BIT_FAST(hold_reg.data[0],6);
                hold_reg1_curr.btn_handL_2run = GET_REG_BIT_FAST(hold_reg.data[0],7);
                hold_reg1_curr.btn_handR_2run = GET_REG_BIT_FAST(hold_reg.data[0],8);
                hold_reg1_curr.btn_head_2run = GET_REG_BIT_FAST(hold_reg.data[0],9);
                hold_reg1_curr.btn_lift_2run = GET_REG_BIT_FAST(hold_reg.data[0],10);
                hold_reg1_curr.btn_rotat_2run = GET_REG_BIT_FAST(hold_reg.data[0],11);

            }

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

            void download_par()
            {
                in_armL.x = read_float_from_word(hold_reg.Reg.pos_arml_x_f2w_1, hold_reg.Reg.pos_arml_x_f2w_2 );
                in_armL.y = read_float_from_word(hold_reg.Reg.pos_arml_y_f2w_1, hold_reg.Reg.pos_arml_y_f2w_2 );
                in_armL.z = read_float_from_word(hold_reg.Reg.pos_arml_z_f2w_1, hold_reg.Reg.pos_arml_z_f2w_2 );
                in_armL.rx = read_float_from_word(hold_reg.Reg.pos_arml_rx_f2w_1, hold_reg.Reg.pos_arml_rx_f2w_2 );
                in_armL.ry = read_float_from_word(hold_reg.Reg.pos_arml_ry_f2w_1, hold_reg.Reg.pos_arml_ry_f2w_2 );
                in_armL.rz = read_float_from_word(hold_reg.Reg.pos_arml_rz_f2w_1, hold_reg.Reg.pos_arml_rz_f2w_2 );

                in_armR.x = read_float_from_word(hold_reg.Reg.pos_armr_x_f2w_1, hold_reg.Reg.pos_armr_x_f2w_2 );
                in_armR.y = read_float_from_word(hold_reg.Reg.pos_armr_y_f2w_1, hold_reg.Reg.pos_armr_y_f2w_2 );
                in_armR.z = read_float_from_word(hold_reg.Reg.pos_armr_z_f2w_1, hold_reg.Reg.pos_armr_z_f2w_2 );
                in_armR.rx = read_float_from_word(hold_reg.Reg.pos_armr_rx_f2w_1, hold_reg.Reg.pos_armr_rx_f2w_2 );
                in_armR.ry = read_float_from_word(hold_reg.Reg.pos_armr_ry_f2w_1, hold_reg.Reg.pos_armr_ry_f2w_2 );
                in_armR.rz = read_float_from_word(hold_reg.Reg.pos_armr_rz_f2w_1, hold_reg.Reg.pos_armr_rz_f2w_2 );

                in_handL = read_float_from_word(hold_reg.Reg.pos_servo_handL_f2w_1,hold_reg.Reg.pos_servo_handL_f2w_2);
                in_handR = read_float_from_word(hold_reg.Reg.pos_servo_handR_f2w_1,hold_reg.Reg.pos_servo_handR_f2w_2);

                in_lift = read_float_from_word(hold_reg.Reg.pos_servo_lift_f2w_1,hold_reg.Reg.pos_servo_lift_f2w_2);
                in_head = read_float_from_word(hold_reg.Reg.pos_servo_head_f2w_1,hold_reg.Reg.pos_servo_head_f2w_2 );
                in_rotate = read_float_from_word(hold_reg.Reg.pos_servo_rotate_f2w_1,hold_reg.Reg.pos_servo_rotate_f2w_2 );




            };






    };

}


