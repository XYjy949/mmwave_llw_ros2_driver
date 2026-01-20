float32     range                       # 点云径向距离
float32     amb_rangerate               # 点云径向速度 感知算法输出结果(未进行速度解模糊，结果可能是模糊的也可能不是模糊的)
float32     unamb_rangerate             # 点云径向速度 数据处理算法处理之后的结果(进行了速度解模糊处理，结果也)存在一定概率是模糊的
float32     azimuth_deg                 # 点云方位角度(度) 左正右负
float32     elevation_deg               # 点云俯仰角度(度) 上正下负
float32     snr                         # 点云RD域信噪比
float32     rcs                         # 点云的rcs
float32     confidence                  # 点云的置信度(0~100)
uint8       unamb_rangeratemask         # 点云速度解模糊完成标志(0x00:不模糊 0x01:可能模糊也可能不模糊 0x02:模糊)由数据处理算法提供
float32     snr_azi                     # 点云方位维信噪比