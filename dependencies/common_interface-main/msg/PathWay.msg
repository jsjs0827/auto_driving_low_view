# 通道（连接两个 zone 的路径），对应 WorkMapPathway
uint32 pathway_id
string pathway_name
uint32 parent_id      # 地图id
uint32 from_zone_id   # 起始区域id
uint32 to_zone_id     # 到达区域id
WorkMapPoint[] polygon
float64 pathway_width
string description