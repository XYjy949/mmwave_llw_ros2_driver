#include "rclcpp/rclcpp.hpp"

#include "ra223f_pkg/msg/object_att.hpp"
#include "ra223f_pkg/msg/object_list.hpp"
#include "ra223f_pkg/msg/point_cloud.hpp"
#include "ra223f_pkg/msg/point_cloud_list.hpp"
#include "ra223f_pkg/msg/radar_status.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "udp_interface.h"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <thread>
#include <math.h>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <vector>
#include <unistd.h>

#include <chrono>
#include <ctime>
#include <builtin_interfaces/msg/time.hpp>

// 在现有头文件下方添加
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// 补充命名空间（可选，但能简化代码）
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

// 将秒和纳秒转换为系统时间
rclcpp::Time ptp_to_system_time(int64_t seconds, int64_t nanoseconds) {
    // 创建 ROS 2 Time 对象
    rclcpp::Time ptp_time(seconds, nanoseconds);
    
    // 如果需要转换为系统时间，ROS 2 时间默认就是系统时间
    // 或者进行必要的时间偏移调整
    return ptp_time;
}

std::string rclcpp_time_to_string(const rclcpp::Time& ros_time) {
    // 将 rclcpp::Time 转换为 time_t
    std::time_t time = static_cast<std::time_t>(ros_time.seconds());
    
    // 转换为本地时间
    std::tm* local_time = std::localtime(&time);
    
    // 格式化输出
    std::stringstream ss;
    ss << std::put_time(local_time, "%Y-%m-%d %H:%M:%S");
    
    // 添加纳秒部分
    ss << "." << std::setw(9) << std::setfill('0') << ros_time.nanoseconds();
    
    return ss.str();
}



//完成不同数据类型之间的转换
typedef union{
    uint8_t data_u8[4];
    float data_float;
}union_conv_t; 

class ra223f_radar:public rclcpp::Node {
private:
    struct sockaddr_in source_address;
    char rec_data[40000];
    int recv_len;
    int ret;

    std::queue<std::vector<char>> data_queue_;          // 数据队列
    std::mutex queue_mutex_;                            // 队列互斥锁
    std::condition_variable queue_cv_;                  // 条件变量

    UdpInterface udp_io;                                 //udp 网络配置

    //设置ra223f的点云和目标话题发布对象
    rclcpp::Publisher<ra223f_pkg::msg::PointCloudList>::SharedPtr pub_pointPtr_;
    rclcpp::Publisher<ra223f_pkg::msg::ObjectList>::SharedPtr pub_objectPtr_;

    //设置ra223f基于pointCloud2的a点云和航迹话题
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointPtr_PointCloud2_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_object_pc2_;

    // 新增：航迹方框 MarkerArray 发布者（RViz 绘制方框用）
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_object_marker_;

    //构建udp的生产者与消费者线程
    std::thread udp_thread;
    std::thread udp_doWork_thread;

public:
    ~ra223f_radar(){
        udp_io.closeAllSockets();

        //释放线程资源，线程中有超时机，等待线程结束
        if(udp_thread.joinable()){
            udp_thread.join();
        }

        if(udp_doWork_thread.joinable()) {
            udp_doWork_thread.join();
        }
    }

    explicit ra223f_radar(const std::string& node_name, std::string groupAdd, uint16_t groupPort):Node(node_name) {
        // RCLCPP_INFO(get_logger(), "step0");
        //创建话题
        pub_pointPtr_ = this->create_publisher<ra223f_pkg::msg::PointCloudList>("/ra223f_pointCloud_topic", 10);
        pub_objectPtr_ = this->create_publisher<ra223f_pkg::msg::ObjectList>("/ra223f_objectList_topic", 10);

        //创建基于sensor_msgs_pointcloud2类型的点云话题
        pub_pointPtr_PointCloud2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ra223f/pointcloud_topic", 10);
        
        // 初始化航迹 PointCloud2 发布者（话题名：/ra223f/object_pointcloud2）
        pub_object_pc2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ra223f/objectList_topic", 10);

        // 初始化航迹方框 MarkerArray 发布者（话题名：/ra223f/object_marker）
        pub_object_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/ra223f/object_marker", 10);


        //配置组播通信
        int ret = udp_io.initUdpMulticastServer(groupAdd,groupPort);
        if(ret == 0) {
            //创建udp数据接受与处理的生产者与消费者模式线程
            udp_thread = std::thread(&ra223f_radar::udp_recv_callback,this);
            udp_doWork_thread = std::thread(&ra223f_radar::udp_recv_doWork,this);
        }
        else {
            RCLCPP_INFO(get_logger(), "网络配置失败");
        }
    }

    //udp数据接受处理线程（生产者）
    void udp_recv_callback(void) {
        udp_io.setNonBlocking(0);

        //接受udp缓冲区中的相关数据并将这些数据添加到队列中
        while(rclcpp::ok()) {
            udp_io.receiveWithTimeout(&source_address, rec_data, recv_len, 1000);
            
            if(recv_len > 0) {
                std::vector<char> buffer(rec_data, rec_data + recv_len);
                {
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    data_queue_.push(buffer);
                }
                queue_cv_.notify_one();
            }
            else {
                continue;
            }
        }
    }

    //执行数据的解析操作 (消费者)
    void udp_recv_doWork(void) {
        ra223f_pkg::msg::PointCloudList pointCloudList;
        ra223f_pkg::msg::PointCloudList *p_pointCloudList = &pointCloudList;
        ra223f_pkg::msg::PointCloud t_PointCloud;
        ra223f_pkg::msg::PointCloud *p_PointCloud = nullptr;

        ra223f_pkg::msg::ObjectList objectList;
        ra223f_pkg::msg::ObjectList *p_objectList = &objectList;
        ra223f_pkg::msg::ObjectAtt t_ObjectAtt;
        ra223f_pkg::msg::ObjectAtt *p_ObjectAtt = nullptr ;

        uint16_t messageType;       //报文类型
        uint16_t messageLen;        //报文数据长度

        // uint8_t checkSum = 0x00; //校验和

        union_conv_t union_conv;    //多字节数据转换工具

        uint16_t savePointCloudCount = 0;
        uint16_t saveObjectNum = 0;

        uint16_t numPointCloudInPackage = 0;
        uint16_t numObjectInPackage = 0;

        uint8_t *p_data = nullptr;

        volatile uint8_t findPointCloudHeader = 0x00;
        volatile uint8_t findObjectHeader = 0x00;
        uint8_t *data_recvBuf = nullptr;

        uint64_t ptp_s;
        uint32_t ptp_ns;

        //系统时间
        rclcpp::Time radar_ptp_time;
        std::string time_str;

        //解析线程主题
        while (rclcpp::ok()) {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            bool hs_data = queue_cv_.wait_for(lock,std::chrono::milliseconds(100),[this]{
                return !data_queue_.empty(); 
            });

            if(!hs_data) {
                //超时触发，没有数据继续检查推出条件
                lock.unlock();
                continue;  
            }
            
            //有数据，读取出相关数据进行数据处理解析操作
            auto data = data_queue_.front();
            data_queue_.pop();
            lock.unlock();

            //进行数据解析逻辑
            data_recvBuf = (uint8_t*)&data[0];
            messageType = (uint16_t)(data_recvBuf[0] << 8 |  data_recvBuf[1]);
            messageLen = (uint16_t) (data_recvBuf[2] << 8 |  data_recvBuf[3]);
            //checkSum = (uint8_t) data_recvBuf[messageLen - 1U];   //获取当前帧的校验和

            switch(messageType) {
                //解析点云头信息
                case 0xAA80:
                    //清空点云临时缓冲区
                    pointCloudList.frame_id = 0;
                    pointCloudList.numof_pointcloud = 0;
                    pointCloudList.pointcloud.clear();

                    //设置当前帧接受到的点云数量与帧id
                    pointCloudList.frame_id = data_recvBuf[6] << 8 | data_recvBuf[7];           //循环计数0~65535
                    pointCloudList.numof_pointcloud = data_recvBuf[8] << 8 | data_recvBuf[9];   //当前帧最大点云数量
                    pointCloudList.timestamp = data_recvBuf[10] << 8 | data_recvBuf[11];        //内部计数器时间戳,0~65535 精度1ms

                    //嵌入式传入错误，嵌入式为小端模式(历史数据兼容问题)
                    union_conv.data_u8[0] = data_recvBuf[15]; 
                    union_conv.data_u8[1] = data_recvBuf[14]; 
                    union_conv.data_u8[2] = data_recvBuf[13]; 
                    union_conv.data_u8[3] = data_recvBuf[12]; 
                    pointCloudList.maxunambiguationspeed = union_conv.data_float;               //当前帧最大非模糊速度 m/s
                    
                    //解析雷达的工作模式
                    pointCloudList.workmode = data_recvBuf[16];                                 //工作模式 0 1

                    //解析ptp时间 秒(s)
                    pointCloudList.ptp_s =  ((uint64_t)data_recvBuf[52] << 56) |
                                            ((uint64_t)data_recvBuf[53] << 48) |
                                            ((uint64_t)data_recvBuf[54] << 40) |
                                            ((uint64_t)data_recvBuf[55] << 32) |

                                            ((uint64_t)data_recvBuf[56] << 24) |
                                            ((uint64_t)data_recvBuf[57] << 16) |
                                            ((uint64_t)data_recvBuf[58] << 8)  |
                                            (uint64_t)data_recvBuf[59];

                    //解析ptp时间ns(ns)
                    pointCloudList.ptp_ns = ((uint32_t)data_recvBuf[60] << 24) |
                                            ((uint32_t)data_recvBuf[61] << 16) |
                                            ((uint32_t)data_recvBuf[62] << 8) |
                                            ((uint32_t)data_recvBuf[63]);

                    //同步保存点云的ptp时间信息用于objectList的ptp时间戳填充
                    ptp_s = pointCloudList.ptp_s;
                    ptp_ns = pointCloudList.ptp_ns;

                    //输出时间戳信息,调试用
                    // RCLCPP_INFO(this->get_logger(),"s: %ld, ns: %d", ptp_s,ptp_ns);
                    radar_ptp_time = ptp_to_system_time(ptp_s,ptp_ns);
                    time_str = rclcpp_time_to_string(radar_ptp_time);
                    // RCLCPP_INFO(this->get_logger(), "%s",time_str.c_str());

                    savePointCloudCount = 0;
                    findPointCloudHeader = 0x01;
                break;

                //解析点云信息
                case 0xAA81:
                    if(0x01 == findPointCloudHeader) {
                        //从多个0xaa81报文中获取对应的点云信息
                        numPointCloudInPackage = (messageLen - 9) / 20;   //获取当前报文里面含有点云个数
                        if(savePointCloudCount < pointCloudList.numof_pointcloud) {
                            for(uint16_t idx = 0; idx < numPointCloudInPackage; idx++) {
                                p_PointCloud = &t_PointCloud;
                                //设置点云的相应熟悉
                                p_data = &data_recvBuf[8 + idx * 20];

                                p_PointCloud->range = (uint32_t)(p_data[0] << 24 | p_data[1] << 16 | p_data[2] << 8 | p_data[3]) * 0.01f; // 径向距离 m
                                p_PointCloud->amb_rangerate = (int16_t)(p_data[4] << 8 | p_data[5]) * 0.01f;        //模糊速度   m/s 点云算法处理结果
                                p_PointCloud->unamb_rangerate = (int16_t)(p_data[6] << 8 | p_data[7]) * 0.01f;      //非模糊速度  m/s 数据处理算法结果 解模糊操作
                                p_PointCloud->azimuth_deg = (int16_t)(p_data[8] << 8 | p_data[9]) * 0.01f;          //方位角 度
                                p_PointCloud->elevation_deg = (int16_t)(p_data[10] << 8 | p_data[11]) *0.01f;       //抚养角 度
                                p_PointCloud->snr = (int16_t)(p_data[12] << 8 | p_data[13]) * 0.01f;                //信噪比 RD域结果
                                p_PointCloud->rcs = (int16_t)(p_data[14] << 8 | p_data[15]) * 0.0039f;              //rcs
                                p_PointCloud->confidence = p_data[16] * 0.01f;  
                                p_PointCloud->unamb_rangeratemask = p_data[17];                                     //数据处理点云是否速度解模糊成功标志 0x00:成功 0x01:可能成功也可能失败 0x02:失败
                                p_PointCloud->snr_azi = (int16_t)((p_data[18] << 8) | (p_data[19])) * 0.01f;        //信噪比 方位维

                                //预留两个字节无内容咱不使用
                                p_pointCloudList->pointcloud.push_back(t_PointCloud);
                            }

                            savePointCloudCount += numPointCloudInPackage;
                        }
                        
                        //点云数据解析完毕，发送相应的话题
                        if(savePointCloudCount == p_pointCloudList->numof_pointcloud) {   
                            pointCloudList.numof_pointcloud = savePointCloudCount;
                            pub_pointPtr_->publish(pointCloudList);
                            findPointCloudHeader = 0x00;

                            
                            // 检查点数量是否有效
                            if (pointCloudList.numof_pointcloud <= 0) {
                                RCLCPP_WARN(this->get_logger(), "No valid points, skip publishing PointCloud2");
                                break;
                            }

                            sensor_msgs::msg::PointCloud2 cloud_msg;    //用于发布点云相关信息

                            // 初始化PointCloud2消息
                            cloud_msg.header.stamp = radar_ptp_time;
                            cloud_msg.header.frame_id = "map";
                            cloud_msg.width = pointCloudList.numof_pointcloud;
                            cloud_msg.height = 1;
                            cloud_msg.is_dense = false;
                            cloud_msg.is_bigendian = false;

                            // 定义x/y/z字段
                            sensor_msgs::msg::PointField field_x;
                            field_x.name = "x";
                            field_x.offset = 0;
                            field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
                            field_x.count = 1;

                            sensor_msgs::msg::PointField field_y;
                            field_y.name = "y";
                            field_y.offset = 4;
                            field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
                            field_y.count = 1;

                            sensor_msgs::msg::PointField field_z;
                            field_z.name = "z";
                            field_z.offset = 8;
                            field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
                            field_z.count = 1;

                            // 新增 snr 字段（float32，占4字节）
                            sensor_msgs::msg::PointField field_snr;
                            field_snr.name = "snr";
                            field_snr.offset = 12;       // 偏移12字节（x+y+z占12字节）
                            field_snr.datatype = sensor_msgs::msg::PointField::FLOAT32;
                            field_snr.count = 1;

                            // 新增 rcs 字段（float32，占4字节）
                            sensor_msgs::msg::PointField field_rcs;
                            field_rcs.name = "rcs";
                            field_rcs.offset = 16;       // 偏移16字节（x+y+z+snr占16字节）
                            field_rcs.datatype = sensor_msgs::msg::PointField::FLOAT32;
                            field_rcs.count = 1;

                            // 新增 range字段（float32，占4字节）
                            sensor_msgs::msg::PointField field_range_rate;
                            field_range_rate.name = "rangerate";
                            field_range_rate.offset = 20;       // 偏移16字节（x+y+z+snr占16字节）
                            field_range_rate.datatype = sensor_msgs::msg::PointField::FLOAT32;
                            field_range_rate.count = 1;

                            // unamb_rangeratemask 字段（速度解模糊标志，uint8 转 FLOAT32 存储）
                            sensor_msgs::msg::PointField field_rangerate_mask;
                            field_rangerate_mask.name = "unamb_rangeratemask";
                            field_rangerate_mask.offset = 24;  // 前 6 个字段占 24 字节（6×4），偏移 24
                            field_rangerate_mask.datatype = sensor_msgs::msg::PointField::FLOAT32;
                            field_rangerate_mask.count = 1;

                            // 清空并添加字段
                            cloud_msg.fields.clear();
                            cloud_msg.fields.push_back(field_x);
                            cloud_msg.fields.push_back(field_y);
                            cloud_msg.fields.push_back(field_z);

                            cloud_msg.fields.push_back(field_snr);              // 添加snr字段
                            cloud_msg.fields.push_back(field_rcs);              // 添加rcs字段
                            cloud_msg.fields.push_back(field_range_rate);       // 添加rangerate字段
                            cloud_msg.fields.push_back(field_rangerate_mask);   //添加速度解模糊标志

                            // 校验字段是否添加成功
                            bool has_x = false;
                            for (const auto& field : cloud_msg.fields) {
                                if (field.name == "x") {
                                    has_x = true;
                                    break;
                                }
                            }
                            if (!has_x) {
                                RCLCPP_ERROR(this->get_logger(), "Failed to add 'x' field!");
                                break;
                            }

                            // 设置步长和数据内存
                            cloud_msg.point_step = 28;
                            cloud_msg.row_step = cloud_msg.width * cloud_msg.point_step;
                            cloud_msg.data.resize(cloud_msg.row_step * cloud_msg.height);

                            // 创建迭代器（此时字段已确认存在）
                            sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
                            sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
                            sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
                            
                            sensor_msgs::PointCloud2Iterator<float> iter_snr(cloud_msg, "snr");                             // snr迭代器
                            sensor_msgs::PointCloud2Iterator<float> iter_rcs(cloud_msg, "rcs");                             // rcs迭代器
                            sensor_msgs::PointCloud2Iterator<float> iter_rangerate(cloud_msg, "rangerate");                 // rangerate迭代器
                            sensor_msgs::PointCloud2Iterator<float> iter_rangerate_mask(cloud_msg, "unamb_rangeratemask");  //rangeratemask迭代器

                            // 填充数据
                            for (size_t i = 0; i < cloud_msg.width; ++i) {  // 显式定义i为size_t
                                t_PointCloud = p_pointCloudList->pointcloud[i];
                                float azimuth_rad = M_PI * (90.0-t_PointCloud.azimuth_deg )/ 180.0f;            //注意此处方位角和雷达定义不一致，适配画图
                                float elevation_rad = M_PI * t_PointCloud.elevation_deg / 180.0f;
                                float range = t_PointCloud.range;

                                *iter_x = range * cos(elevation_rad) * sin(azimuth_rad);
                                *iter_y = range * cos(elevation_rad) * cos(azimuth_rad);
                                *iter_z = range * sin(elevation_rad);

                                *iter_snr = t_PointCloud.snr;                                                   // snr：信噪比
                                *iter_rcs = t_PointCloud.rcs;                                                   // rcs：雷达截面积
                                *iter_rangerate = t_PointCloud.unamb_rangerate;                                 // rangerate:径向速度
                                *iter_rangerate_mask = static_cast<float>(t_PointCloud.unamb_rangeratemask);    //更新速度解模糊标志

                                ++iter_x;
                                ++iter_y;
                                ++iter_z;
                                ++iter_snr;
                                ++iter_rcs;
                                ++iter_rangerate;
                                ++iter_rangerate_mask;
                            }

                            // ====================== 关键修改3：发布PointCloud2消息 ======================
                            pub_pointPtr_PointCloud2_->publish(cloud_msg);
                        }
                    }
                break;

                //解析objectList报文
                case 0xAA90:
                    objectList.object_att.clear();
                    objectList.numof_object = 0;
                    objectList.frame_id = 0;
                    p_objectList->frame_id  = (data_recvBuf[6] << 8) | (data_recvBuf[7]); //帧计数 0~65535 循环计数
                    p_objectList->numof_object = data_recvBuf[8];   //获取得到object个数

                    //获取ptp时间
                    p_objectList->ptp_s = ptp_s;
                    p_objectList->ptp_ns = ptp_ns;

                    saveObjectNum = 0;
                    findObjectHeader = 0x01;
                break;

                //解析object报文
                case 0xAA91:
                if(findObjectHeader == 0x01) {
                    numObjectInPackage  = (messageLen - 10) / 43;   //获取当前报文里面含有点云个数
                    if(saveObjectNum <  p_objectList->numof_object ) {
                        p_ObjectAtt = &t_ObjectAtt ;
                        for(uint16_t idx = 0; idx < numObjectInPackage; idx++) {
                            //设置点云的i相应属性
                            p_data = &data_recvBuf[9 + idx * 43];

                            p_ObjectAtt->track_id = p_data[0];      //航迹的id号
                            p_ObjectAtt->xpos_vcs =  (int16_t)( p_data[1] << 8 | p_data[2]) * 0.01f;      //纵向距离 m
                            p_ObjectAtt->ypos_vcs =  (int16_t)( p_data[3] << 8 | p_data[4]) * 0.01f;      //横向距离 m
                            p_ObjectAtt->zpos_vcs =  (int16_t)( p_data[5] << 8 | p_data[6]) * 0.01f;      //垂直距离 m

                            p_ObjectAtt->xvel_abs =  (int16_t)( p_data[7] << 8 | p_data[8]) * 0.01f;      //纵向速度 m/s
                            p_ObjectAtt->yvel_abs =  (int16_t)( p_data[9] << 8 | p_data[10]) * 0.01f;     //横向速度 m/s

                            p_ObjectAtt->xacc_abs =  (int16_t)( p_data[11] << 8 | p_data[12]) * 0.01f;    //纵向加速度 m/s^2
                            p_ObjectAtt->yacc_abs =  (int16_t)( p_data[13] << 8 | p_data[14]) * 0.01f;    //横向加速度 m/s^2

                            p_ObjectAtt->heading_angle =  (int16_t)( p_data[15] << 8 | p_data[16]) * 0.01f; //航向角度 左正右负

                            p_ObjectAtt->box_width =  (uint16_t)( p_data[17] << 8 | p_data[18]) * 0.01f;    //航迹的包围框长宽高
                            p_ObjectAtt->box_length =  (uint16_t)( p_data[19] << 8 | p_data[20]) * 0.01f;
                            p_ObjectAtt->box_heigh =  (uint16_t)( p_data[21] << 8 | p_data[22]) * 0.01f;

                            p_ObjectAtt->classify_type = p_data[23];       //目标分类 0:未知 1：行人 2：自行车 3：小汽车 4：大卡车                             
                            p_ObjectAtt->classify_prob = p_data[24];       //目标分类对应的概率

                            p_ObjectAtt->objmotion_status =  (uint16_t)( p_data[25] << 8 | p_data[26]) ;  //航迹动静状态 0：静止 1：运动
                            p_ObjectAtt->obstacle_prob =  (uint16_t)( p_data[27] << 8 | p_data[28]) ;     //障碍物概率

                            p_objectList->object_att.push_back(t_ObjectAtt);
                        }

                        saveObjectNum += numObjectInPackage;
                    }

                    //航迹接受到指定的个数
                    if(saveObjectNum == p_objectList->numof_object ) {
                        objectList.numof_object = saveObjectNum;
                        pub_objectPtr_->publish(objectList);
                        findObjectHeader = 0x00;

                        // ====================== 发布航迹 PointCloud2（中心坐标+尺寸）======================
                        sensor_msgs::msg::PointCloud2 object_pc2;
                        object_pc2.header.stamp = radar_ptp_time;
                        object_pc2.header.frame_id = "map";                 // 与点云坐标系一致
                        object_pc2.width = objectList.numof_object;         // 航迹数量
                        object_pc2.height = 1;
                        object_pc2.is_dense = false;
                        object_pc2.is_bigendian = false;

                        // 逐个创建 PointField 
                        std::vector<sensor_msgs::msg::PointField> object_fields;

                        // 1. x 字段（中心x坐标）
                        sensor_msgs::msg::PointField field_ox;
                        field_ox.name = "x";
                        field_ox.offset = 0;  // 偏移0字节
                        field_ox.datatype = sensor_msgs::msg::PointField::FLOAT32;
                        field_ox.count = 1;
                        object_fields.push_back(field_ox);

                        // 2. y 字段（中心y坐标）
                        sensor_msgs::msg::PointField field_oy;
                        field_oy.name = "y";
                        field_oy.offset = 4;  // 偏移4字节（x占4字节）
                        field_oy.datatype = sensor_msgs::msg::PointField::FLOAT32;
                        field_oy.count = 1;
                        object_fields.push_back(field_oy);

                        // 3. z 字段（中心z坐标）
                        sensor_msgs::msg::PointField field_oz;
                        field_oz.name = "z";
                        field_oz.offset = 8;  // 偏移8字节（x+y占8字节）
                        field_oz.datatype = sensor_msgs::msg::PointField::FLOAT32;
                        field_oz.count = 1;
                        object_fields.push_back(field_oz);

                        // 4. width 字段（航迹宽度）
                        sensor_msgs::msg::PointField field_ow;
                        field_ow.name = "width";
                        field_ow.offset = 12;  // 偏移12字节（x+y+z占12字节）
                        field_ow.datatype = sensor_msgs::msg::PointField::FLOAT32;
                        field_ow.count = 1;
                        object_fields.push_back(field_ow);

                        // 5. length 字段（航迹长度）
                        sensor_msgs::msg::PointField field_ol;
                        field_ol.name = "length";
                        field_ol.offset = 16;  // 偏移16字节（x+y+z+width占16字节）
                        field_ol.datatype = sensor_msgs::msg::PointField::FLOAT32;
                        field_ol.count = 1;
                        object_fields.push_back(field_ol);

                        // 6. height 字段（航迹高度）
                        sensor_msgs::msg::PointField field_oh;
                        field_oh.name = "height";
                        field_oh.offset = 20;  // 偏移20字节（x+y+z+width+length占20字节）
                        field_oh.datatype = sensor_msgs::msg::PointField::FLOAT32;
                        field_oh.count = 1;
                        object_fields.push_back(field_oh);

                        // 7. xvel_abs（纵向速度）
                        sensor_msgs::msg::PointField field_xvel;
                        field_xvel.name = "xvel_abs";
                        field_xvel.offset = 24;  // 前 6 个字段占 20 字节，下一个偏移 24
                        field_xvel.datatype = sensor_msgs::msg::PointField::FLOAT32;
                        field_xvel.count = 1;
                        object_fields.push_back(field_xvel);

                        // 8. yvel_abs（横向速度）
                        sensor_msgs::msg::PointField field_yvel;
                        field_yvel.name = "yvel_abs";
                        field_yvel.offset = 28;
                        field_yvel.datatype = sensor_msgs::msg::PointField::FLOAT32;
                        field_yvel.count = 1;
                        object_fields.push_back(field_yvel);

                        // 9. xacc_abs（纵向加速度）
                        sensor_msgs::msg::PointField field_xacc;
                        field_xacc.name = "xacc_abs";
                        field_xacc.offset = 32;
                        field_xacc.datatype = sensor_msgs::msg::PointField::FLOAT32;
                        field_xacc.count = 1;
                        object_fields.push_back(field_xacc);

                        // 10. yacc_abs（横向加速度）
                        sensor_msgs::msg::PointField field_yacc;
                        field_yacc.name = "yacc_abs";
                        field_yacc.offset = 36;
                        field_yacc.datatype = sensor_msgs::msg::PointField::FLOAT32;
                        field_yacc.count = 1;
                        object_fields.push_back(field_yacc);

                        // 11. heading_angle（航向角）
                        sensor_msgs::msg::PointField field_heading;
                        field_heading.name = "heading_angle";
                        field_heading.offset = 40;
                        field_heading.datatype = sensor_msgs::msg::PointField::FLOAT32;
                        field_heading.count = 1;
                        object_fields.push_back(field_heading);

                        // 12. classify_type（目标分类）
                        sensor_msgs::msg::PointField field_cls_type;
                        field_cls_type.name = "classify_type";
                        field_cls_type.offset = 44;
                        field_cls_type.datatype = sensor_msgs::msg::PointField::FLOAT32;  // 用FLOAT32存储uint8，兼容RViz显示
                        field_cls_type.count = 1;
                        object_fields.push_back(field_cls_type);

                        // 13. classify_prob（分类概率）
                        sensor_msgs::msg::PointField field_cls_prob;
                        field_cls_prob.name = "classify_prob";
                        field_cls_prob.offset = 48;
                        field_cls_prob.datatype = sensor_msgs::msg::PointField::FLOAT32;
                        field_cls_prob.count = 1;
                        object_fields.push_back(field_cls_prob);

                        // 14. objmotion_status（动静状态）
                        sensor_msgs::msg::PointField field_motion;
                        field_motion.name = "objmotion_status";
                        field_motion.offset = 52;
                        field_motion.datatype = sensor_msgs::msg::PointField::FLOAT32;  // 用FLOAT32存储uint16，兼容显示
                        field_motion.count = 1;
                        object_fields.push_back(field_motion);

                        // 15. obstacle_prob（障碍物概率）
                        sensor_msgs::msg::PointField field_obstacle_prob;
                        field_obstacle_prob.name = "obstacle_prob";
                        field_obstacle_prob.offset = 56;
                        field_obstacle_prob.datatype = sensor_msgs::msg::PointField::FLOAT32;  // 用FLOAT32存储uint16，兼容显示
                        field_obstacle_prob.count = 1;
                        object_fields.push_back(field_obstacle_prob);

                        // 赋值字段到 PointCloud2
                        object_pc2.fields = object_fields;
                        object_pc2.point_step = 60;  // 15个FLOAT32 × 4字节 = 60字节
                        object_pc2.row_step = object_pc2.width * object_pc2.point_step;
                        object_pc2.data.resize(object_pc2.row_step * object_pc2.height);

                        // 创建迭代器填充数据
                        sensor_msgs::PointCloud2Iterator<float> iter_ox(object_pc2, "x");
                        sensor_msgs::PointCloud2Iterator<float> iter_oy(object_pc2, "y");
                        sensor_msgs::PointCloud2Iterator<float> iter_oz(object_pc2, "z");
                        sensor_msgs::PointCloud2Iterator<float> iter_ow(object_pc2, "width");
                        sensor_msgs::PointCloud2Iterator<float> iter_ol(object_pc2, "length");
                        sensor_msgs::PointCloud2Iterator<float> iter_oh(object_pc2, "height");

                        sensor_msgs::PointCloud2Iterator<float> iter_xvel(object_pc2, "xvel_abs");
                        sensor_msgs::PointCloud2Iterator<float> iter_yvel(object_pc2, "yvel_abs");
                        sensor_msgs::PointCloud2Iterator<float> iter_xacc(object_pc2, "xacc_abs");
                        sensor_msgs::PointCloud2Iterator<float> iter_yacc(object_pc2, "yacc_abs");
                        sensor_msgs::PointCloud2Iterator<float> iter_heading(object_pc2, "heading_angle");
                        sensor_msgs::PointCloud2Iterator<float> iter_cls_type(object_pc2, "classify_type");
                        sensor_msgs::PointCloud2Iterator<float> iter_cls_prob(object_pc2, "classify_prob");
                        sensor_msgs::PointCloud2Iterator<float> iter_motion(object_pc2, "objmotion_status");
                        sensor_msgs::PointCloud2Iterator<float> iter_obstacle_prob(object_pc2, "obstacle_prob");

                        // ====================== 生成航迹方框 MarkerArray（RViz 绘制用）======================
                        visualization_msgs::msg::MarkerArray marker_array;
                        int marker_id = 0;  // 每个航迹的唯一ID（避免RViz重复渲染）

                        for (const auto& obj : objectList.object_att) {
                            // 填充 PointCloud2 数据（中心坐标+尺寸）
                            *iter_ox = obj.xpos_vcs;
                            *iter_oy = obj.ypos_vcs;
                            *iter_oz = obj.zpos_vcs;
                            *iter_ow = obj.box_width;
                            *iter_ol = obj.box_length;
                            *iter_oh = obj.box_heigh;

                            *iter_xvel = obj.xvel_abs;                                      // 纵向速度
                            *iter_yvel = obj.yvel_abs;                                      // 横向速度
                            *iter_xacc = obj.xacc_abs;                                      // 纵向加速度
                            *iter_yacc = obj.yacc_abs;                                      // 横向加速度
                            *iter_heading = obj.heading_angle;                              // 航向角
                            *iter_cls_type = static_cast<float>(obj.classify_type);         // uint8转float，兼容显示
                            *iter_cls_prob = obj.classify_prob;                             // 分类概率
                            *iter_motion = static_cast<float>(obj.objmotion_status);        // uint16转float，兼容显示
                            *iter_obstacle_prob = static_cast<float>(obj.obstacle_prob);    // uint16转float，兼容显示

                            // 生成单个航迹的方框 Marker
                            visualization_msgs::msg::Marker marker;
                            marker.header = object_pc2.header;  // 同坐标系、同时间戳
                            marker.id = marker_id++;  // 唯一ID
                            marker.type = visualization_msgs::msg::Marker::CUBE;  // 类型：立方体（方框）
                            marker.action = visualization_msgs::msg::Marker::ADD;

                            // 方框中心坐标（与航迹中心一致）
                            // 对应xpos_vcs，y对应ypos_vcs）
                            marker.pose.position.x = obj.xpos_vcs;  // 纵向距离 → x轴
                            marker.pose.position.y = obj.ypos_vcs;  // 横向距离 → y轴
                            marker.pose.position.z = obj.zpos_vcs;  // 垂直距离 → z轴

                            // 添加航向角旋转
                            float heading_rad = obj.heading_angle * M_PI / 180.0f;  // 度转弧度
                            // 关键：ROS 坐标系中，航向角左正右负，对应绕 Z 轴逆时针旋转
                            // 四元数计算（绕 Z 轴旋转 θ：w=cos(θ/2), z=sin(θ/2)）
                            marker.pose.orientation.w = cos(heading_rad / 2.0f);
                            marker.pose.orientation.z = sin(heading_rad / 2.0f);
                            marker.pose.orientation.x = 0.0f;
                            marker.pose.orientation.y = 0.0f;

                            // 方框尺寸（直接使用航迹的宽/长/高）
                            marker.scale.x = obj.box_length;  // 注意：RViz的scale.x对应航迹length（纵向）
                            marker.scale.y = obj.box_width;   // scale.y对应航迹width（横向）
                            marker.scale.z = obj.box_heigh;   // scale.z对应航迹height（垂直）

                            // 方框颜色（半透明蓝色，rgba：红,绿,蓝,透明度）
                            marker.color.r = 0.0f;
                            marker.color.g = 0.0f;
                            marker.color.b = 1.0f;
                            marker.color.a = 0.5f;  // 透明度0.5，避免遮挡其他数据

                            // 方框生命周期（1秒，避免航迹消失后残留）
                            marker.lifetime = rclcpp::Duration::from_seconds(1.0);

                            // 添加到 MarkerArray
                            marker_array.markers.push_back(marker);

                            // 迭代器自增
                            ++iter_ox; ++iter_oy; ++iter_oz; ++iter_ow; ++iter_ol; ++iter_oh;
                            ++iter_xvel; ++iter_yvel; ++iter_xacc; ++iter_yacc; ++iter_heading;
                            ++iter_cls_type; ++iter_cls_prob; ++iter_motion; ++iter_obstacle_prob;
                        }

                        // 发布航迹 PointCloud2 和 MarkerArray
                        pub_object_pc2_->publish(object_pc2);
                        pub_object_marker_->publish(marker_array);
                    }
                } 
                break;
                
                //接受IP地址装订状态报文
                case 0xAA8B:
                if(messageLen == 8U) {
                    if(data_recvBuf[6] == 0x00) {
                        RCLCPP_INFO(this->get_logger(),"雷达IP操作失败!..........");
                    }
                    else if(data_recvBuf[6] == 0x01) {
                        RCLCPP_INFO(this->get_logger(),"雷达IP装订操作成功!..........");
                    }
                    else if(data_recvBuf[6] == 0x02) {
                        RCLCPP_INFO(this->get_logger(),"雷达IP复位操作成功!..........");
                    }
                    else {
                        /*qac*/
                    }
                } 
                break;

                //雷达系统的工作状态
                // case 0xAA7F:
                // if(messageLen == 16) {
                //     //解析awr2243芯片的工作状态
                //     if((data_recvBuf[6] & 0x01) != 0x00) {
                //         RCLCPP_INFO(this->get_logger(), "从片0故障");
                //     }

                //     if((data_recvBuf[6] & 0x02) == 0x00) {
                //         RCLCPP_INFO(this->get_logger(), "从片1故障");
                //     }

                //     if((data_recvBuf[6] & 0x04) == 0x00) {
                //         RCLCPP_INFO(this->get_logger(), "从片2故障");
                //     }

                //     if((data_recvBuf[6] & 0x08) == 0x00) {
                //         RCLCPP_INFO(this->get_logger(),"从片3故障");
                //     }

                //     //获取rx_error
                //     if((data_recvBuf[7] & 0x01) == 0x00) {
                //         RCLCPP_INFO(this->get_logger(),"awr2243接受故障");
                //     }

                //     if((data_recvBuf[8] & 0x01) == 0x00) {
                //         RCLCPP_INFO(this->get_logger(),"awr2243发射故障");
                //     }

                //     if((data_recvBuf[9] & 0x01) == 0x00) {
                //          RCLCPP_INFO(this->get_logger(),"雷达受到了干扰");
                //     }

                //     if((data_recvBuf[10] & 0x01) == 0x00) {
                //         RCLCPP_INFO(this->get_logger(),"雷达天线罩被覆盖");
                //     }
                // }
                // break;

                default:
                break;
            }
        }  
    }
};

//主入口
int main(int argc, char **argv) {
    setlocale(LC_ALL, "en_US.UTF-8");

    rclcpp::init(argc,argv);
    auto node = std::make_shared<ra223f_radar>("ra223f_radar_node","224.0.2.3", 42104);
      
    rclcpp::spin(node);
    rclcpp::shutdown();

    return  0;
}


