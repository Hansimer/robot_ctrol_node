#pragma once

#include "../datedef.hpp"

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
                2： 视觉触发
                3：提升电机
                4：旋转电机
                5: neck电机
                6：头部电机
                7：right hand
                8：left hand

                //joint运动端                
                12：左臂arm
                13：右臂arm
                14：双臂运动

                //ptp运动端
                21：左臂arm
                22：右臂arm
                23：双臂arm

                //pline运动端
                31：左臂arm
                32：右臂arm
                33：双臂arm            
            
            */
            bool getArmMotionCmd(const action_info_ &par , exec_cmd_config_& command )
            {


            };
    }

}


