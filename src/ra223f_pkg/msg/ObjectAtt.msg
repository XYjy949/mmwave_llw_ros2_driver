uint8       track_id         # 跟踪目标的批号

float32     xpos_vcs         # 整车坐标系下的目标纵向位置
float32     ypos_vcs         # 整车坐标系的目标横向位置
float32     zpos_vcs         # 整车坐标下的高度位置

float32     xvel_abs         # 目标在整车坐标系下的纵向对地速度
float32     yvel_abs         # 目标在整车坐标系下的横向对地速度

float32     xacc_abs         # 目标在整车坐标下的纵向对地加速度
float32     yacc_abs         # 目标在整车坐标下的横向对地加速度

float32     heading_angle    # 目标在跟踪在在坐标系下的航向角

float32     box_width        # 目标的包围框宽度
float32     box_length       # 目标包围框长度
float32     box_heigh        # 目标包围框高度

uint8       classify_type    # 目标分类 0x00:未知 0x01:行人 0x02:两轮车 0x03：小汽车 0x04: 大卡车
uint8       classify_prob    # 目标的分类概率

uint8       objmotion_status # 目标的动静状态 0x00:静止  0x01:运动
uint8       obstacle_prob    # 目标的障碍物概率