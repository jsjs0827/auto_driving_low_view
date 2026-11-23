int64 time
uint8 collision_signal   # 撞板信号---左触发：0X01;右触发;0X02;双触发：0X03；解除0X00
uint8 suspend_signal     # 悬空信号---左触发：0X01;右触发;0X02;双触发：0X03；解除0X00
uint8 emergency_stop_o   # 急停原始;---触发：0X01；解除0X00
uint8 emergency_stop_h   # 急停处理;----按下：0X03;松开0X01;解除：0X00
uint8 rain_signal        # 雨淋信号;---触发：0X01；解除0X00
uint8 charging           # 充电状态;---充电中：0X01；非充电：0X00
