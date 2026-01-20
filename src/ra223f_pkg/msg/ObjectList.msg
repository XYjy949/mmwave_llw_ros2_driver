uint16          frame_id        # 当前帧id（0～65535）循环
uint8           numof_object    # 当前帧上传的目标个数
uint8           work_mode       # 雷达的工作模式

#ptp 系统时间戳 秒(s)
uint64          ptp_s           # 获取雷达系统上传的ptp系统时间秒s
uint64          ptp_ns          # 获取雷达系统上传的ptp系统时间纳秒ns

ObjectAtt[]     object_att      # 当前帧上传的目标属性