uint8 CMD_CLOSE=0       # 整机开关机
uint8 CMD_REBOOT=1      # 整机重启
uint8 CMD_LOW_POWER=2   # 整机低功耗
uint8 CMD_MOTOR_POWER=3 # 电机电源开关
uint8 CMD_STOP=4        # 急停状态设置
uint8 CMD_FILL_LIGHT=5  # 补光灯开关

uint8 STATUS_CLOSE=0     # 关闭
uint8 STATUS_OPEN=1    # 打开

int64 time
uint8 cmd
uint8 status