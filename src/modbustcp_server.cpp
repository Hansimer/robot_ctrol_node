#include "../include/robot_ctrol_node/modbustcp_server.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include "../include/robot_ctrol_node/modbustcp_server.hpp"


using namespace std::chrono_literals;
using namespace std;
namespace robot_ctrol_node
{
     ModbusTcpServerCpp::ModbusTcpServerCpp(string& net_ip , int& net_port, int& net_max_clients,int& net_task_interval_ms)
  {
    LOG_INFO("==== Modbus TCP Server C++ Node Started ====");
    net_ip_=net_ip;
    net_port_=net_port;
    net_max_clients_=net_max_clients;
    net_task_interval_ms_ = net_task_interval_ms;

    is_stop_ = false;
    is_run_ = false;
  }

  

  ModbusTcpServerCpp::~ModbusTcpServerCpp()
  {
    stop();
    if (modbus_server_thread_.joinable())
    {
      modbus_server_thread_.join();
    }
    if (g_modbus_map)
    {
      modbus_mapping_free(g_modbus_map);
      g_modbus_map = nullptr;
    }
  }

  void ModbusTcpServerCpp::start()
  {
    // 启动 Modbus 监听线程
    modbus_server_thread_ = std::thread(&ModbusTcpServerCpp::server_loop, this);

    // 上升沿检测定时器
    

  }

  bool ModbusTcpServerCpp::init()
  {
    
       // 初始化 Modbus 寄存器
    if (!g_modbus_map)
    {
      g_modbus_map = modbus_mapping_new(0, 0, REGISTER_COUNT, 0);
      if (!g_modbus_map)
      {
        LOG_INFO("modbus_mapping_new failed");        
        return false;
      }
    }
    return true;
  }

  void ModbusTcpServerCpp::stop()
  {
    is_stop_ = true ;
    is_run_ = false ;
  }

  /// @brief 对只读寄存器进行数据更新
  /// @param index_start 
  /// @param src 
  void ModbusTcpServerCpp::update_Input_reg(uint8_t index_start ,uint16_t* src, uint16_t len)
  {
    std::lock_guard<std::mutex> lock(g_reg_mutex);
    memcpy(g_modbus_map->tab_registers+index_start,src,len);
  }

  /// @brief 获取寄存器数据
  /// @param index_start 
  /// @param src 
  void ModbusTcpServerCpp::getdown_Input_reg(uint8_t index_start , uint16_t* des, uint16_t len)
  {
      std::lock_guard<std::mutex> lock(g_reg_mutex);
      memcpy(des,g_modbus_map->tab_registers+index_start,len);
  }

  // 单个客户端处理
  void ModbusTcpServerCpp::handle_single_client(modbus_t* ctx, int client_fd)
  {
    uint8_t recv_buffer[MODBUS_TCP_MAX_ADU_LENGTH];
    bool client_online = true;

    // ========== 新增：获取客户端 IP + 端口 ==========
    struct sockaddr_in client_addr{};
    socklen_t addr_len = sizeof(client_addr);
    char client_ip[INET_ADDRSTRLEN] = {0};
    uint16_t client_port = 0;

    // 获取对端 socket 地址
    if (getpeername(client_fd, (struct sockaddr*)&client_addr, &addr_len) == 0)
    {
        // 二进制IP转字符串
        inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
        // 网络字节序转主机字节序端口
        client_port = ntohs(client_addr.sin_port);
        LOG_INFO("New client connected, fd: %d | IP: %s | Port: %d",
                 client_fd, client_ip, client_port);
    }
    else
    {
        LOG_WARN("New client connected, fd: %d | Get client IP failed, errno: %d",
                 client_fd, errno);
        LOG_INFO("New client connected, fd: %d", client_fd);
    }
    

    while (client_online)
    {
      //std::this_thread::sleep_for(std::chrono::milliseconds(POLL_INTERVAL_MS));
      try
      {
        int recv_len = modbus_receive(ctx, recv_buffer);
        if (recv_len <= 0)
        {
          LOG_WARN(
            "Client fd:%d disconnected | errno: %d | info: %s",
            client_fd, errno, strerror(errno));
          client_online = false;
          break;
        }

        std::lock_guard<std::mutex> lock(g_reg_mutex);
        modbus_reply(ctx, recv_buffer, recv_len, g_modbus_map);
      }
      catch (...)
      {
        LOG_ERROR("Client fd:%d communication exception", client_fd);
        client_online = false;
        break;
      }
    }

    close(client_fd);
    LOG_INFO("Client fd:%d has been closed", client_fd);
    std::this_thread::sleep_for(std::chrono::milliseconds(RECONNECT_DELAY_MS));
  }

  // Modbus TCP 监听主循环
  void ModbusTcpServerCpp::server_loop()
  {
    modbus_t* ctx = modbus_new_tcp(net_ip_.c_str(), net_port_);
    if (!ctx)
    {
      LOG_ERROR("Create modbus tcp context failed!");
      return; 
    }
    modbus_set_debug(ctx, false);

    int server_fd = modbus_tcp_listen(ctx, MAX_CLIENT_NUM);
    if (server_fd == -1)
    {
      LOG_ERROR("Listen port %d failed!", MODBUS_TCP_PORT);
      modbus_free(ctx);
      return;
    }

    LOG_INFO("Modbus TCP Listen: %s:%d | Max Client: %d",
      net_ip_.c_str(), net_port_, net_max_clients_);

    while (!is_stop_)
    {
      int client_fd = modbus_tcp_accept(ctx, &server_fd);
      if (client_fd == -1)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(RECONNECT_DELAY_MS));
        continue;
      }
      handle_single_client(ctx, client_fd);
    }

    close(server_fd);
    modbus_free(ctx);
  }


  float ModbusTcpServerCpp::read_float_from_modbus_registers(modbus_mapping_t* mb_map, int base_addr)
{
    uint16_t reg_high = mb_map->tab_registers[base_addr];
    uint16_t reg_low  = mb_map->tab_registers[base_addr + 1];

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
}


// 把 float 写入 Modbus 寄存器（大端字序，标准Modbus格式：高字在前，高字节在前）
void ModbusTcpServerCpp::write_float_to_modbus_registers(modbus_mapping_t* mb_map, int base_addr, float value)
{
    union {
        float f;
        uint8_t bytes[4];
    } u;
    u.f = value;

    // 大端模式：
    // 寄存器1（base_addr）：高16位（u.bytes[0], u.bytes[1]）
    // 寄存器2（base_addr+1）：低16位（u.bytes[2], u.bytes[3]）
    uint16_t reg_high = (static_cast<uint16_t>(u.bytes[1]) << 8) | u.bytes[0];
    uint16_t reg_low  = (static_cast<uint16_t>(u.bytes[3]) << 8) | u.bytes[2];

    mb_map->tab_registers[base_addr ]     = reg_high;
    mb_map->tab_registers[base_addr +1] = reg_low;
}

   
}