uint16      frame_id                # 帧ID (0~65535)循环
uint16      numof_pointcloud        # 当前帧包含的点云数量
uint16      timestamp               # 当前帧雷达的时间戳(0~65535) 1ms精度的计数器，循环计数器
float32     maxunambiguationspeed   # 当前帧对应的最大非模糊速度
uint8       workmode                # 当前雷达的波型工作模式

float32     ego_speed               # 雷达内部当前帧使用的车辆速度信息 m/s
float32     ego_yawrate             # 雷达内部当前帧使用的车辆偏航率信息 rad/s

float32     dy_alignment_process    # 雷达安装角度动态校准进程

#ptp 系统时间戳 秒(s)
uint64      ptp_s                   # 获取雷达系统上传的ptp系统时间秒s
uint64      ptp_ns                  # 获取雷达系统上传的ptp系统时间纳秒ns

PointCloud[] pointcloud             # 当前帧包含的雷达点云数据集
