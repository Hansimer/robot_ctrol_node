#pragma once    


#include <rclcpp/rclcpp.hpp>
#include <my_interfaces/srv/srv_move_axis.hpp>
#include <modbus/modbus.h>
#include <thread>
#include <mutex>
#include <cstdint>
#include <cerrno>
#include <cstring>
#include "../log.hpp"
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "../config_par.hpp"
#include "../datedef.hpp"

using namespace std::chrono_literals;
using namespace std;

namespace robot_ctrol_node
{

  
// 全局配置 & Modbus 资源
constexpr int MODBUS_TCP_PORT     = 5020;
constexpr int MAX_CLIENT_NUM      = 5;
constexpr int REGISTER_COUNT      = 300;
constexpr int POLL_INTERVAL_MS    = 50;
constexpr int RECONNECT_DELAY_MS  = 1000;





class ModbusTcpServerCpp 
{
    public:
        explicit ModbusTcpServerCpp(string& net_ip_ , int& net_port_, int& net_max_clients_,int& net_task_interval_ms_);
        ~ModbusTcpServerCpp();
        bool init();//初始化
        void start();//运行
        void stop();//停止
        modbus_mapping_t* g_modbus_map = nullptr;
        void update_Input_reg(uint8_t index_start ,uint16_t* src, uint16_t len); //更新寄存器数据
        void getdown_Input_reg(uint8_t index_start , uint16_t* des, uint16_t len); //获取寄存器数据




    private:

       
        std::thread modbus_server_thread_;
        // 存储所有位姿数据

        std::string net_ip_;
        int net_port_;
        int net_max_clients_;
        int net_task_interval_ms_;

        
        std::mutex        g_reg_mutex;
        uint16_t          g_prev_reg0 = 0;
        bool              is_stop_ = false;
        bool              is_run_ = false;

        // 单个客户端处理
        void handle_single_client(modbus_t* ctx, int client_fd);


        // Modbus TCP 监听主循环
        void server_loop();
        void write_float_to_modbus_registers(modbus_mapping_t* mb_map, int base_addr, float value);
        float read_float_from_modbus_registers(modbus_mapping_t* mb_map, int base_addr);
    
    };
} // namespace modbus_tcp_server_cpp


