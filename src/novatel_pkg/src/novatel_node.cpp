#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "udp_interface.h"
// 替换为标准ROS消息
#include "nav_msgs/msg/odometry.hpp"
#include "novatel_pkg/msg/net_param.hpp"
#include "novatel_pkg/msg/radar_mount_info.hpp"
#include <vector>
#include <string>
#include <sstream>
#include <cstdint>

#include <regex>
#include <map>

#define TEST_MODE       (0U)  //测试开关

//定义全局的json数据键值关系
typedef struct{
  std::string json_key;
  std::string json_value;
}jsonKV_t;

class novatel_imu : public rclcpp::Node {

public:
  novatel_imu(const std::string& node_name) : Node(node_name) {
    //尝试多个本地端口
    int ret = -1;
    int max_attempts = 65535;
    for (int attempt = 50000; attempt < max_attempts; attempt++) {
      ret = udp_io.initUdpUnicastClient("192.168.2.61", 42404, attempt);
      if (ret == 0) {
            RCLCPP_INFO(get_logger(), "成功绑定本地端口: %d", attempt);
            break;
      } 
      if (attempt == max_attempts - 1) {
          RCLCPP_ERROR(get_logger(), "所有端口尝试失败");
          return;
      }
    }

    if(ret == 0) {
        // 创建订阅者 获取姿态信息 - 改用Odometry消息
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom",  // 根据实际话题名修改
        10,
        std::bind(&novatel_imu::topic_callback, this, std::placeholders::_1)
        );

        //创建订阅这 重置雷达的ip设置信息至默认状态
        resetIp_subscription_ = this->create_subscription<std_msgs::msg::String>(
        "radar_223f/resetIP",
        10,
        std::bind(&novatel_imu::reset_radarIp,this,std::placeholders::_1)
        );

        //创建设置IP地址信息的订阅请求，默认地址掩码为255.255.255.0
        //创建订阅这 重置雷达的ip设置信息至默认状态
        setIp_subscription_ = this->create_subscription<novatel_pkg::msg::NetParam>(
        "radar_223f/setIP",
        10,
        std::bind(&novatel_imu::set_radarIp,this,std::placeholders::_1)
        );

        //订阅话题，设置雷达静态安装参数
        radarMountInfo_subscription_ = this->create_subscription<novatel_pkg::msg::RadarMountInfo>(
        "radar_223f/setRadarMountInfo",
        10,
        std::bind(&novatel_imu::set_RadarMountInfo,this,std::placeholders::_1)
        );

        //订阅话题，获取汽车的档位信息
        egoGearStatus_subscription_ = this->create_subscription<std_msgs::msg::String>(
          "loaderStatus",
          10,
          std::bind(&novatel_imu::set_EgoGearStatus,this,std::placeholders::_1)
        );

#if (TEST_MODE == 1U)
        pub_resetIpPtr_ = this->create_publisher<std_msgs::msg::String>("/radar_223f/resetIP", 10);
        pub_setIpPtr_ = this->create_publisher<novatel_pkg::msg::NetParam>("/radar_223f/setIP", 10);
        pub_radarMountPtr_ = this->create_publisher<novatel_pkg::msg::RadarMountInfo>("/radar_223f/setRadarMountInfo", 10);

        // 创建定时器 - 每隔1秒执行一次
        timer_ = this->create_wall_timer(
          std::chrono::seconds(1),  // 周期：1秒
          std::bind(&novatel_imu::timer_callback, this)
        );
#endif

    }
  }

  //析构函数释放资源
  ~novatel_imu() {
    // udp_io.closeUdpSocket();    //释放系统资源

#if (TEST_MODE == 1U)
    if(timer_) {
      timer_->cancel();
      timer_->reset();
    }
#endif
  }

private:
#if (TEST_MODE == 1U)
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_resetIpPtr_;    //测试重置IP测试器
    rclcpp::Publisher<novatel_pkg::msg::NetParam>::SharedPtr pub_setIpPtr_; //测试设置ip
    rclcpp::Publisher<novatel_pkg::msg::RadarMountInfo>::SharedPtr pub_radarMountPtr_; //测试雷达挂载信息
    rclcpp::TimerBase::SharedPtr timer_;

    //创建定时器回调函数用于测试任务
  void timer_callback() {
    static int count = 0;
    count++;
    
    // 创建并发布消息
    auto msg = std_msgs::msg::String();
    msg.data = "Periodic message " + std::to_string(count) + 
               " at time: " + std::to_string(this->now().seconds());
    pub_resetIpPtr_->publish(msg);
    
    //测试设置IP信息
    novatel_pkg::msg::NetParam ipInfo_;
    ipInfo_.radar_ip = "10.13.2.115";
    ipInfo_.local_ip = "10.13.2.250";
    ipInfo_.gateway = "255.255.255.0";
    ipInfo_.radar_receive_port = 4004;
    ipInfo_.pc_receive_port = 42104;
    ipInfo_.radar_send_port = 4003;
    pub_setIpPtr_->publish(ipInfo_);

    //创建配置雷达的静止安装信息
    novatel_pkg::msg::RadarMountInfo radar_mountInfo_;
    radar_mountInfo_.car_height = 2.f;
    radar_mountInfo_.car_length = 10.f;

    pub_radarMountPtr_->publish(radar_mountInfo_);
  }
#endif
  
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr resetIp_subscription_;                    //重置雷达的ip信息
    rclcpp::Subscription<novatel_pkg::msg::NetParam>::SharedPtr setIp_subscription_;                 //设置雷达的ip信息
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;                          //车辆的姿态信息(改用Odometry)
    rclcpp::Subscription<novatel_pkg::msg::RadarMountInfo>::SharedPtr radarMountInfo_subscription_;  //雷达静态挂载参数
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr egoGearStatus_subscription_;              //获取车辆档位信息

    UdpInterface  udp_io;
    uint16_t    packageNum = 0;
    // 移除了不再需要的变量: azimuthPre, preTime, deltaT (因为Odometry直接提供yaw rate)
    int         udp_socket; 
    uint16_t    resetIp_count = 0;
    uint16_t    setIp_count = 0;
    uint16_t    radarMount_count = 0;
    volatile int16_t ego_gearStatus = 1;  //默认空档

    //实现内部自定义的函数，完成内部操作的相关转换
private:
    //进行字符串的ip信息转成bytes字节数组
  std::vector<uint8_t> ip_string_to_bytes(const std::string& ip_str) {
    std::vector<uint8_t> bytes;
    std::istringstream iss(ip_str);
    std::string segment;
    
    while (std::getline(iss, segment, '.')) {
        bytes.push_back(static_cast<uint8_t>(std::stoi(segment)));
    }
    return bytes;
  }

  //计算指定数组内的制定字节checksum
  uint8_t calc_checkSum(uint8_t* p_data_buf, uint16_t data_buf_size) {
    uint16_t index;
    uint16_t check_sum = 0;

    for (index = 0U; index <= data_buf_size; index++) {
      check_sum += p_data_buf[index];
    }

    return (uint8_t)check_sum;
  }

  //从json数据流中解析出制定的字符串信息
  bool parse_json_data(const std::string& json_str, jsonKV_t& data) {

    //定义待解析的键值信息，如需要新增其他的键值对解析需求，可以在此处进行追加
    std::map<std::string, std::regex> patterns = {
        {"gear", std::regex("\"gear\":\\s*\"([^\"]+)\"")},
    };
    
    //使用正则表达式进行提取字符串中的相关所关系内容
    std::smatch matches;
    bool found_any = false;
    
    // 解析 gear
    if (std::regex_search(json_str, matches, patterns["gear"]) && matches.size() > 1) {
        data.json_key = "gear";
        data.json_value = matches[1].str();
        found_any = true;
    }
      
    return found_any;
}

private:

  //获取汽车的档位信息
  void set_EgoGearStatus(const std_msgs::msg::String::SharedPtr p_msg) {
    (void)p_msg;
    jsonKV_t gearJson;
    uint8_t gearStatus = 1;  //默认空档

    if(parse_json_data(p_msg->data, gearJson)) {
      //进行档位信息的重新映射
      if(gearJson.json_value == "r") {
        gearStatus = 0;
      }
      else if(gearJson.json_value == "n") {
        gearStatus = 1;
      }
      else if(gearJson.json_value == "p"){
        gearStatus = 3;
      }
      else {
        gearStatus = 2;
      }

      this->ego_gearStatus = gearStatus;
    }
  }

  //设置雷达的静态挂载信息
  void set_RadarMountInfo(const novatel_pkg::msg::RadarMountInfo::SharedPtr p_msg) {
    uint8_t data_buf[64] = {0x00};

    data_buf[0] = 0xAA;
    data_buf[1] = 0x30;
    data_buf[2] = 0x00;
    data_buf[3] = 32;                     //数据有效长度

    data_buf[4] = radarMount_count >> 8;       //设置重置计数器
    data_buf[5] = radarMount_count & 0x00ff;

    //设置雷达挂载坐标
    int16_t mount_pos = (int16_t)(p_msg->mount_xpos * 100.f);
    data_buf[6] = mount_pos >> 8;
    data_buf[7] = mount_pos & 0xff;

    mount_pos = (int16_t)(p_msg->mount_ypos * 100.f);
    data_buf[8] = mount_pos >> 8;
    data_buf[9] = mount_pos & 0xff;

    mount_pos = (int16_t)(p_msg->mount_zpos * 100.f);
    data_buf[10] = mount_pos >> 8;
    data_buf[11] = mount_pos & 0xff;

    //雷达的安装角度
    mount_pos = (int16_t)(p_msg->raw_yaw_angle * 100.f);
    data_buf[12] = mount_pos >> 8;
    data_buf[13] = mount_pos & 0xff;

    mount_pos = (int16_t)(p_msg->raw_pitch_angle * 100.f);
    data_buf[14] = mount_pos >> 8;
    data_buf[15] = mount_pos & 0xff;

    //设置车辆宽度
    mount_pos = (int16_t)(p_msg->car_width * 100.f);
    data_buf[16] = mount_pos >> 8;
    data_buf[17] = mount_pos & 0xff;

    mount_pos = (int16_t)(p_msg->car_length * 100.f);
    data_buf[18] = mount_pos >> 8;
    data_buf[19] = mount_pos & 0xff;

    mount_pos = (int16_t)(p_msg->car_height * 100.f);
    data_buf[20] = mount_pos >> 8;
    data_buf[21] = mount_pos & 0xff;

    mount_pos = (int16_t)(p_msg->car_wheelbase * 100.f);
    data_buf[22] = mount_pos >> 8;
    data_buf[23] = mount_pos & 0xff;

    data_buf[24] = 0x00;  //装订操作

    //计算checksum
    data_buf[31] = this->calc_checkSum(&data_buf[0],31);
    this->udp_io.sendToRadar((char*)data_buf,32,200);
    radarMount_count ++;
    RCLCPP_INFO(this->get_logger(), "设置雷达的静态挂载信息");
  }

  //设置雷达的ip地址
  void set_radarIp(const novatel_pkg::msg::NetParam::SharedPtr p_msg) {
    (void)p_msg;
    RCLCPP_INFO(this->get_logger(), "请求雷达设置IP信息");

    uint8_t data_buf[64] = {0x00};

    data_buf[0] = 0xAA;
    data_buf[1] = 0x8A;
    data_buf[2] = 0x00;
    data_buf[3] = 64;                     //数据有效长度

    data_buf[4] = setIp_count >> 8;       //设置重置计数器
    data_buf[5] = setIp_count & 0x00ff;

    //设置IP地址
    auto radar_ip_bytes = this->ip_string_to_bytes(p_msg->radar_ip);
    data_buf[6] = radar_ip_bytes[0];            //设置雷达的ip地址
    data_buf[7] = radar_ip_bytes[1];
    data_buf[8] = radar_ip_bytes[2];
    data_buf[9] = radar_ip_bytes[3];

    //设置本地ip地址
    auto local_ip_bytes = this->ip_string_to_bytes(p_msg->local_ip);
    data_buf[10] = local_ip_bytes[0];     //设置雷达的ip地址
    data_buf[11] = local_ip_bytes[1];
    data_buf[12] = local_ip_bytes[2];
    data_buf[13] = local_ip_bytes[3];

    //设置网关
    auto gateway_bytes = this->ip_string_to_bytes(p_msg->gateway);
    data_buf[14] = gateway_bytes[0];            //设置雷达的网关
    data_buf[15] = gateway_bytes[1];
    data_buf[16] = gateway_bytes[2];
    data_buf[17] = gateway_bytes[3];

    //设置子网掩码
    data_buf[18] = 255;
    data_buf[19] = 255;
    data_buf[20] = 255;
    data_buf[21] = 0;

    //设置pc接受端口
    data_buf[22] = (p_msg->pc_receive_port >> 8);        
    data_buf[23] = (p_msg->pc_receive_port & 0xff);

    //设置雷达的接受端口
    data_buf[24] = (p_msg->radar_receive_port >> 8);        
    data_buf[25] = (p_msg->radar_receive_port & 0xff);

    //设置雷达的发送端口
    data_buf[26] = (p_msg->radar_send_port >> 8);        
    data_buf[27] = (p_msg->radar_send_port & 0xff);

    //设置报文模式
    data_buf[28] = 0x00;                  //进行ip设置
    
    //计算校验码
    data_buf[63] = this->calc_checkSum(&data_buf[0], 63);
    this->udp_io.sendToRadar((char*)data_buf,64,200);
    setIp_count++;
  }

  //重置雷达的ip为默认状态
  void reset_radarIp(const std_msgs::msg::String::SharedPtr msg) {
    (void) msg;
    RCLCPP_INFO(this->get_logger(), "请求雷达复位IP信息");

    uint8_t data_buf[64] = {0x00};

    data_buf[0] = 0xAA;
    data_buf[1] = 0x8A;
    data_buf[2] = 0x00;
    data_buf[3] = 64;                     //数据有效长度

    data_buf[4] = resetIp_count >> 8;     //设置重置计数器
    data_buf[5] = resetIp_count & 0x00ff;

    data_buf[28] = 0x01;                  //进行ip复位

    //计算校验码
    data_buf[63] = this->calc_checkSum(&data_buf[0], 63);
    this->udp_io.sendToRadar((char*)data_buf,64,200);
    resetIp_count++;
  }

  
  //订阅Odometry获取速度和yaw rate信息 (替换原INSPVA回调)
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr p_odom) {
    uint8_t data_buf[32] = {0x00};
    data_buf[0] = 0xAA;
    data_buf[1] = 0x31; 
    data_buf[2] = 0x00;
    data_buf[3] = 32;           //数据有效长度

    float vel = 0.f;            //车辆速度
    float yawRate = 0.f;        //车辆的yawRate
    int16_t yawRateInt= 0;
    int16_t speedInt = 0;

    //进行话题数据组包处理
    data_buf[4] = (packageNum & 0xFF00)  >> 8; 
    data_buf[5] = packageNum & 0x00FF;

    // ============ 核心修改部分 ============
    // 从Odometry的twist中直接获取速度和yaw rate
    // twist.twist.linear 是线速度 (x: 前进, y: 横向, z: 垂直)
    // twist.twist.angular.z 是绕z轴的角速度，即yaw rate
    
    // 方式1: 如果Odometry是车体坐标系(常见情况)，直接用linear.x作为前进速度
    // vel = std::abs(p_odom->twist.twist.linear.x);
    
    // 方式2: 如果Odometry是世界坐标系(UTM)，需要合成x和y分量
    vel = std::sqrt(
        p_odom->twist.twist.linear.x * p_odom->twist.twist.linear.x + 
        p_odom->twist.twist.linear.y * p_odom->twist.twist.linear.y
    );

    // yaw rate 直接从angular.z获取，单位已经是rad/s，无需额外计算
    yawRate = p_odom->twist.twist.angular.z;
    // ============ 核心修改结束 ============

    //打包整车的偏航率
    yawRateInt = (int16_t)(yawRate * 10000.f);
    data_buf[6] = (yawRateInt & 0xFF00)  >> 8; 
    data_buf[7] = yawRateInt & 0x00FF;

    //打包整车的车速
    speedInt = (int16_t)(vel * 100.f);
    data_buf[8] = (speedInt & 0xFF00)  >> 8; 
    data_buf[9] = speedInt & 0x00FF;

    //设置车辆的档位信息
    data_buf[16] = this->ego_gearStatus >> 8; 
    data_buf[17] = this->ego_gearStatus & 0x00ff;

    //计算校验码
    data_buf[31] = this->calc_checkSum(&data_buf[0],31);

    udp_io.sendToRadar((char*) data_buf, 32,200);

    packageNum = packageNum +1;

    // RCLCPP_INFO(get_logger(),"vel=%.2f m/s, yawRate=%.2f rad/s", vel, yawRate);
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<novatel_imu>("novatel_imu_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
}
