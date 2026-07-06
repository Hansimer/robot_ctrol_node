#pragma once
#include <tinyxml2.h>
#include <iostream>
#include <string>
#include <map>
#include <vector>
#include "json.hpp"  // JSON解析（ROS2自带）
#include "log.hpp"
#include "datedef.hpp"
#include "config_par.hpp"

#define MAX_ACTIONS 40

namespace robot_ctrol_node
{
    class getparxml
    {
        public:
            getparxml(){

               
            };
            ~getparxml()
            {

            };
                    //获取动作
            array_actions_info_ actions[MAX_ACTIONS]; //0:初始化动作 1:扫描动作
            

            //获取参数
            dev_params_info_ para_axis_lift;
            dev_params_info_ para_dual_arms;
            tcp_params_info_ para_net_tcp;//网络配置

            string xml_path;
            ConfigParser config_parser_;

            bool load_dev_xmlconfig(const string& xml_path)
            {
                
                if (config_parser_.loadFile(xml_path)) {
                    LOG_INFO( "XML configuration loaded successfully."); 
                    // 1. 先获取目标设备的 params
                const auto& device_params = config_parser_.devices_cfg;
                LOG_INFO("===== Device params list =====");
                for(auto& par :device_params)
                {
                    if (par.device_name =="lift_motor" ) //提升电机参数
                    {
                        auto& tmap = par.params;
                        auto it = tmap.find("mode");
                        if (it != tmap.end()) {
                            // it->second 就是 "mode" 对应的 value
                            para_axis_lift.mode = it->second;
                        } 

                        it = tmap.find("speed_convert_factor");
                        if (it != tmap.end()) {
                            // it->second 就是 "mode" 对应的 value
                            para_axis_lift.speed_convert_factor = std::stof(it->second);
                        } 

                        it = tmap.find("veljog_mpm");
                        if (it != tmap.end()) {
                            // it->second 就是 "mode" 对应的 value
                            para_axis_lift.veljog_mpm = std::stof(it->second);
                        } 
                        
                        it = tmap.find("time_acc_s");
                        if (it != tmap.end()) {
                            // it->second 就是 "mode" 对应的 value
                            para_axis_lift.time_acc_s = std::stof(it->second);
                        } 


                        LOG_INFO(" lift_motor mode is %s , factor is %.8f, veljog_mpm is %.2f",para_axis_lift.mode.c_str(),para_axis_lift.speed_convert_factor,para_axis_lift.veljog_mpm);
                    }

                    if (par.device_name =="dual_arm" ) //提升电机参数
                    {
                        auto& tmap = par.params;
                        auto it = tmap.find("mode");
                        if (it != tmap.end()) {
                            // it->second 就是 "mode" 对应的 value
                            para_axis_lift.mode = it->second;
                        } 

                        it = tmap.find("speed_convert_factor");
                        if (it != tmap.end()) {
                            // it->second 就是 "mode" 对应的 value
                            para_axis_lift.speed_convert_factor = std::stof(it->second);
                        } 
                        LOG_INFO(" dual_arm mode is %s , factor is %.2f",para_axis_lift.mode.c_str(),para_axis_lift.speed_convert_factor);
                    }

                    if (par.device_name =="mdtcpserver" ) //提升电机参数
                    {
                        auto& tmap = par.params;
                        auto it = tmap.find("ip");
                        if (it != tmap.end()) {
                            // it->second 就是 "mode" 对应的 value
                            para_net_tcp.ip = it->second;
                        } 

                        it = tmap.find("port");
                        if (it != tmap.end()) {
                            // it->second 就是 "mode" 对应的 value
                            para_net_tcp.port = std::stof(it->second);
                        } 

                        it = tmap.find("intertime_ms");
                        if (it != tmap.end()) {
                            // it->second 就是 "mode" 对应的 value
                            para_net_tcp.intertime = std::stof(it->second);
                        } 


                        it = tmap.find("maxconnect");
                        if (it != tmap.end()) {
                            // it->second 就是 "mode" 对应的 value
                            para_net_tcp.connmax = std::stof(it->second);
                        } 
                        LOG_INFO(" tcp ip is %s , port is %d",para_net_tcp.ip.c_str(),para_net_tcp.port);
                    }
                    
                }

                LOG_INFO("===== actions list =====");
                const auto& pose_params = config_parser_.actions;
                for(auto &pose_param : pose_params)
                {
                    auto &tmap1 = pose_param.action_entries_map;
                    auto it = tmap1.find("init_action");
                    if(it != tmap1.end())
                    {
                        auto it2 = it->second;
                        // 关键：提前扩容，和源 vector 大小保持一致
                        actions[0].actions.resize(it2.size());
                        for (size_t idx = 0; idx < it2.size(); ++idx)
                        {
                        actions[0].actions.at(idx).index = it2.at(idx).index;
                        actions[0].actions.at(idx).sub_index =  it2.at(idx).sub_index;
                        actions[0].actions.at(idx).type = it2.at(idx).type;
                        actions[0].actions.at(idx).info_.action_data.d[0] = it2.at(idx).d1;
                        actions[0].actions.at(idx).info_.action_data.d[1] = it2.at(idx).d2;
                        actions[0].actions.at(idx).info_.action_data.d[2] = it2.at(idx).d3;
                        actions[0].actions.at(idx).info_.action_data.d[3] = it2.at(idx).d4;
                        actions[0].actions.at(idx).info_.action_data.d[4] = it2.at(idx).d5;
                        actions[0].actions.at(idx).info_.action_data.d[5] = it2.at(idx).d6;
                        actions[0].actions.at(idx).info_.action_data.d[6] = it2.at(idx).d7;
                        actions[0].actions.at(idx).info_.action_data.d[7] = it2.at(idx).d8;
                        actions[0].actions.at(idx).info_.action_data.d[8] = it2.at(idx).d9;
                        actions[0].actions.at(idx).info_.action_data.d[9] = it2.at(idx).d10;
                        actions[0].actions.at(idx).info_.action_data.d[10] = it2.at(idx).d11;
                        actions[0].actions.at(idx).info_.action_data.d[11] = it2.at(idx).d12;
                        actions[0].actions.at(idx).info_.action_data.d[12] = it2.at(idx).d13;
                        actions[0].actions.at(idx).info_.action_data.d[13] = it2.at(idx).d14;

                        
                        LOG_INFO("  [%zu] index: %u, sub_index: %u, type: %u",
                                idx, actions[0].actions.at(idx).index, actions[0].actions.at(idx).sub_index, actions[0].actions.at(idx).type);
                     
                        }
                    }


                    
                    it = tmap1.find("scan_action");
                    if(it != tmap1.end())
                    {
                        auto it2 = it->second;
                        // 关键：提前扩容，和源 vector 大小保持一致
                        actions[1].actions.resize(it2.size());
                        for (size_t idx = 0; idx < it2.size(); ++idx)
                        {
                        actions[1].actions.at(idx).index = it2.at(idx).index;
                        actions[1].actions.at(idx).sub_index =  it2.at(idx).sub_index;
                        actions[1].actions.at(idx).type = it2.at(idx).type;
                        actions[1].actions.at(idx).info_.action_data.d[0] = it2.at(idx).d1;
                        actions[1].actions.at(idx).info_.action_data.d[1] = it2.at(idx).d2;
                        actions[1].actions.at(idx).info_.action_data.d[2] = it2.at(idx).d3;
                        actions[1].actions.at(idx).info_.action_data.d[3] = it2.at(idx).d4;
                        actions[1].actions.at(idx).info_.action_data.d[4] = it2.at(idx).d5;
                        actions[1].actions.at(idx).info_.action_data.d[5] = it2.at(idx).d6;
                        actions[1].actions.at(idx).info_.action_data.d[6] = it2.at(idx).d7;
                        actions[1].actions.at(idx).info_.action_data.d[7] = it2.at(idx).d8;
                        actions[1].actions.at(idx).info_.action_data.d[8] = it2.at(idx).d9;
                        actions[1].actions.at(idx).info_.action_data.d[9] = it2.at(idx).d10;
                        actions[1].actions.at(idx).info_.action_data.d[10] = it2.at(idx).d11;
                        actions[1].actions.at(idx).info_.action_data.d[11] = it2.at(idx).d12;
                        actions[1].actions.at(idx).info_.action_data.d[12] = it2.at(idx).d13;
                        actions[1].actions.at(idx).info_.action_data.d[13] = it2.at(idx).d14;

                        
                        LOG_INFO("  [%zu] index: %u, sub_index: %u, type: %u",
                                idx, actions[1].actions.at(idx).index, actions[1].actions.at(idx).sub_index, actions[1].actions.at(idx).type);
                       
                        }
                    }
                    


                }          
                
                    LOG_INFO("=================================");
                
                
                } else {
                    LOG_ERROR("Failed to load XML configuration from %s", xml_path.c_str());
                    return false;
                }
                return true;

            };
    };


}
