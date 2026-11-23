#ifndef AI_FUSION_H
#define AI_FUSION_H

#include "percep_demo/ai_fusion_safety.hpp"
#include "percep_demo/pcl_common.hpp"

#include <stdexcept>
#include <utility>

struct CloudDATA {
    PointCloudT ground;
    PointCloudT obs;
    PointCloudT ai;
};

class AiFusion:public PclCommon{
    public:
        int rows,cols;
        bool debug;
        // 点云 label 到栅格 type 的会话级契约，由当前模型 class_names
        // 与 passable/nongrass 策略在 DNN 初始化完成后注入。
        std::map<int,int> map_label;

        AiFusion(double width,double height,double res,double obs_height_thr,int fill_grass_thr,int remove_noise_thr,double x_cali,double y_cali,float min_x)
        {
            const auto geometry =
                ai_node_percep_demo::detail::TryMakeGridGeometry(width, height, res);
            if (!geometry.has_value()) {
                throw std::invalid_argument("invalid or oversized AI fusion grid");
            }
            map_width = width;
            map_height = height;
            grid_geometry = *geometry;
            map_resolution = grid_geometry.resolution_m;
            ObsHeightThr = obs_height_thr;
            FillGrassThr = fill_grass_thr;
            RemoveNoiseThr = remove_noise_thr;
            x_cam_cali = x_cali;
            y_cam_cali = y_cali;
            fov_min_x = min_x;
            cols = static_cast<int>(grid_geometry.cols);  //列数
            rows = static_cast<int>(grid_geometry.rows);  //行数
            total_grids = static_cast<int>(grid_geometry.cell_count);
            this->clean_variable();
        }

        void setLabelMap(std::map<int, int> labels)
        {
            map_label = std::move(labels);
        }

        void initData(PointCloudT::Ptr ground_cloud,PointCloudT::Ptr obs_cloud,PointCloudT::Ptr ai_cloud)
        {
            if (ground_cloud) cloud_data.ground = *ground_cloud;
            else cloud_data.ground.clear();
            if (ai_cloud) cloud_data.ai = *ai_cloud;
            else cloud_data.ai.clear();
            if (obs_cloud) cloud_data.obs = *obs_cloud;
            else cloud_data.obs.clear();
        }
        void integrate(OccMapData &data)
        {
            vector<int> idx;
            // init_map(fusion_map);
            project_to_map();

            for (int i=0;i<total_grids;i++)
            {
                this->fuse(ai_map[i],pcl_map[i],fusion_map[i]);
            }
            fill_holes();
            remove_noise();
            // 未命中的格子保持 MapPoint 默认 min_h=999/max_h=-999；*100 会溢出 int16
            auto height_cm = [](float h_m) -> int16_t {
                if (!(h_m <= 900.f && h_m >= -900.f)) {
                    return 0;  // 默认哨兵，避免 int16 截断成 ±31172
                }
                const float cm = h_m * 100.f;
                if (cm > 32767.f) return 32767;
                if (cm < -32768.f) return -32768;
                return static_cast<int16_t>(cm);
            };
            std::vector<int16_t> pcl_min_h,pcl_max_h,ai_min_h,ai_max_h;
            pcl_min_h.resize(total_grids,0);
            pcl_max_h.resize(total_grids,0);
            ai_min_h.resize(total_grids,0);
            ai_max_h.resize(total_grids,0);

            for (int idx = 0; idx < total_grids; idx++)
            {
                map_avg_h[idx]= height_cm(fusion_map[idx].avg_h);
                map_min_h[idx] = height_cm(fusion_map[idx].min_h);
                map_max_h[idx] = height_cm(fusion_map[idx].max_h);
                type[idx] =   static_cast<uint8_t>(fusion_map[idx].type);

                pcl_min_h[idx] = height_cm(pcl_map[idx].min_h);
                pcl_max_h[idx] = height_cm(pcl_map[idx].max_h);
                ai_min_h[idx] = height_cm(ai_map[idx].min_h);
                ai_max_h[idx] = height_cm(ai_map[idx].max_h);
            }

            float dist_min = 999.f;
            int index_min = -1;
            float x_min = 0.f, y_min = 0.f;
            for (int idx = 0; idx < total_grids; idx++)
            {
                if (type[idx] >= 2 && type[idx]!=3)
                {
                    float half_res = map_resolution * 0.5f;
                    int grid_y = idx / cols;
                    int grid_x = idx % cols;
                    float x = (grid_y + half_res) * map_resolution;
                    float y = map_width/2.0f - (grid_x + half_res)* map_resolution;
                    float dist_to_origin = std::sqrt(x * x + y * y);
                    if (dist_to_origin<dist_min) {
                        dist_min = dist_to_origin;
                        index_min = idx;
                        x_min = x;
                        y_min = y;
                    }
                }
            }

            const size_t n_ground = cloud_data.ground.size();
            const size_t n_obs = cloud_data.obs.size();
            const size_t n_ai = cloud_data.ai.size();

            if (index_min < 0) {
                LOG_INFO("[percepDemo][fusion] nearest: none (no type>=2)");
            } else {
                const MapPoint& ai_p = ai_map[index_min];
                const MapPoint& pcl_p = pcl_map[index_min];
                const MapPoint& fus_p = fusion_map[index_min];
                const int ai_valid = (ai_p.min_h <= ai_p.max_h) ? 1 : 0;
                const int pcl_valid = (pcl_p.min_h <= pcl_p.max_h) ? 1 : 0;
                cout<<"最近障碍物点x:"<<x_min<<" y:"<<y_min<<" map_min_h:"<<map_min_h[index_min]<<" map_max_h:"<<map_max_h[index_min]<<endl;
                cout<<"pcl_min_h:"<<pcl_min_h[index_min]
                    <<" pcl_max_h:"<<pcl_max_h[index_min]
                    <<" ai_min_h:"<<ai_min_h[index_min]
                    <<" ai_max_h:"<<ai_max_h[index_min]<<endl;
                LOG_INFO("[percepDemo] 最近障碍物点 x:%f, y:%f,map_min_h:%d,map_max_h:%d",
                         x_min, y_min, map_min_h[index_min], map_max_h[index_min]);
            }

            data.ai_map = ai_map;
            data.pcl_map = pcl_map;
            data.fusion_map = fusion_map;
            data.map_min_h = map_min_h;
            data.map_max_h = map_max_h;
            data.map_avg_h = map_avg_h;
            data.type = type;
            this->clean_variable();
        }
        
    public:
        CloudDATA cloud_data;
        vector<MapPoint> ai_map,pcl_map,fusion_map;
        double map_resolution,map_width,map_height;
        double ObsHeightThr = 0.05f;
        int FillGrassThr = 5;
        int RemoveNoiseThr = 6;
        double x_cam_cali,y_cam_cali;
        float fov_min_x;
        int total_grids;
        ai_node_percep_demo::detail::GridGeometry grid_geometry;
        std::vector<int16_t> map_min_h,map_max_h,map_avg_h;
        std::vector<uint8_t> type;
        
    private:
        void init_map(vector<MapPoint> &map)
        {
            for (int i=0;i<map.size();i++)
            {
                double origin_x= i / cols;
                double origin_y= -i % cols + map_width/2.0/map_resolution;
                if(is_inside_fov(origin_x,origin_y))
                {
                    map[i].type = 1;
                }
            }
        }
        void clean_variable()
        {
            ai_map.clear();
            pcl_map.clear();
            fusion_map.clear();
            map_min_h.clear();
            map_max_h.clear();
            map_avg_h.clear();
            type.clear();

            ai_map.resize(total_grids); 
            pcl_map.resize(total_grids);
            fusion_map.resize(total_grids);
            map_min_h.resize(total_grids,INT16_MAX);
            map_max_h.resize(total_grids,INT16_MIN);
            map_avg_h.resize(total_grids,0);
            type.resize(total_grids,0);
        }

        bool is_inside_fov(double x,double y)
        {
            //双目HFOV：98度
            double fov_angle = angle_to_rad(98.0);
            double min_k = -tan(fov_angle/2.0);
            double max_k = -min_k;
  
            if (y-min_k*x-x_cam_cali/map_resolution>=0 && y-max_k*x+x_cam_cali/map_resolution<=0 && x>=(fov_min_x+x_cam_cali)/map_resolution) 
                {return true;}
            else
                { return false;}

        }
        void fuse(MapPoint &ai_point,MapPoint &pcl_point,MapPoint &fusion_point)
        {
            int ai_type = ai_point.type;
            int pcl_type = pcl_point.type;

            fusion_point.avg_h = pcl_point.avg_h;
            fusion_point.max_h = pcl_point.max_h;
            fusion_point.min_h = pcl_point.min_h;
            //ai: 0=未知, 1=非草,3 =草，4 = 人，5 = 猫， 6 = 狗 ， 7 = 充电站
            //pcl:0=未知, 1=地面, 2=其他障碍
            //fusion: 0=未知, 1=非草, 2=其他障碍，3 =草，4 = 人，5 = 猫， 6 = 狗 ， 7 = 充电站
            if (ai_type == 0) //unknown
            {
                if (pcl_type > 0) {fusion_point.type = pcl_type;}
            }            
            if (pcl_type == 0) //unknown
            {
                if (ai_type > 0) {fusion_point.type = ai_type;}
            }   
            
            if (ai_type == 3) //grass
            {
                fusion_point.type = 3;
            }
            if (ai_type == 1) //nonegrass
            {
                if (pcl_type == 0) {fusion_point.type = 0;}
                if (pcl_type == 1) {fusion_point.type = 1;}
                if (pcl_type == 2) {fusion_point.type = 2;}

            }
            if (ai_type == 2) // generic detection obstacle
            {
                if (pcl_type == 1) {fusion_point.type = 1;}
                else {fusion_point.type = 2;}
            }
            if (ai_type >=4)  //obs
            {
                if (pcl_type == 1) {fusion_point.type = 1;}
                else{
                    fusion_point.type = ai_type;
                }
            }

        }
        void fill_holes()
        {
            for (std::size_t i = 0; i < fusion_map.size(); ++i)
            {
                if (fusion_map[i].type == 0)  //type=="unknown"
                {
                    int count_grass = 0;
                    const auto neighborhood =
                        ai_node_percep_demo::detail::MakeGridNeighborhood(
                            grid_geometry, i);
                    for (std::size_t n = 0; n < neighborhood.count; ++n)
                    {
                        const std::size_t neighbor_idx = neighborhood.cells[n];
                        if (fusion_map[neighbor_idx].type == 3) {count_grass++;} // type == "grass"
                    }             
                    if (count_grass>=FillGrassThr){
                        fusion_map[i].type = 3;  //"grass"

                    }       
                }
            }
        }
        void remove_noise()
        {
            for (std::size_t i = 0; i < fusion_map.size(); ++i)
            {
                const auto neighborhood =
                    ai_node_percep_demo::detail::MakeGridNeighborhood(
                        grid_geometry, i);
                if (fusion_map[i].type>0)
                {
                    int count_unknown = 0;
                    for (std::size_t n = 0; n < neighborhood.count; ++n)
                    {
                        const std::size_t neighbor_idx = neighborhood.cells[n];
                        if (fusion_map[neighbor_idx].type == 0) {count_unknown++;} //label == "unknown"
                    }             
                    if (count_unknown>=RemoveNoiseThr){
                        fusion_map[i].type = 0;
                    }       
                }

                if(fusion_map[i].type == 1) //nongrass
                {
                    int count_grass = 0;
                    for (std::size_t n = 0; n < neighborhood.count; ++n)
                    {
                        const std::size_t neighbor_idx = neighborhood.cells[n];
                        if (fusion_map[neighbor_idx].type == 3) {count_grass++;}
                    }
                    if (count_grass >=7){
                        fusion_map[i].type = 3;
                    }                     
                }
                if (fusion_map[i].type>0)//非unknown
                {
                    int count_self = 0,count_grass =0,count_nonegrass =0,count_obs=0,count_ai_obs=0;
                    int ai_type = 0;
                    for (std::size_t n = 0; n < neighborhood.count; ++n)
                    {
                        const std::size_t neighbor_idx = neighborhood.cells[n];
                        if (fusion_map[neighbor_idx].type == fusion_map[i].type) {count_self++;} 
                        if (fusion_map[neighbor_idx].type == 1) {count_nonegrass++;} 
                        if (fusion_map[neighbor_idx].type == 2) {count_obs++;} 
                        if (fusion_map[neighbor_idx].type == 3) {count_grass++;} 
                        if (fusion_map[neighbor_idx].type > 3) {count_ai_obs++;ai_type = fusion_map[neighbor_idx].type;} 
                    }             
                    if (count_self<=2){
                        int max_vote = std::max({count_nonegrass, count_obs, count_grass, count_ai_obs});
                        int new_type = 0;
                        // 按优先级匹配
                        if (count_nonegrass == max_vote) { new_type = 1; }
                        else if (count_obs == max_vote) { new_type = 2; }
                        else if (count_grass == max_vote) { new_type = 3; }
                        else if (count_ai_obs == max_vote) { new_type = ai_type; } 
                        fusion_map[i].type = new_type;
                    }       
                }
            }
        }
        void project_to_map()
        {
            //行存储地图：idx = y * 列数 + x
            //点云坐标系:x向上，y向左，原点在底部map_width/2.0处
            //地图坐标系:x向右，y向上，原点在左下角
            std::vector<float> sum_h_pcl(total_grids, 0.0f); 
            std::vector<int> point_count_pcl(total_grids, 0);
  
            for (const auto &point : cloud_data.ground)
            {
                ai_node_percep_demo::detail::GridCellIndex cell;
                if (ai_node_percep_demo::detail::TryProjectPointToGrid(
                        point.x, point.y, point.z, map_width, map_resolution,
                        grid_geometry, &cell))
                {
                    const std::size_t cur_index = cell.linear;
                    sum_h_pcl[cur_index] += point.z; 
                    point_count_pcl[cur_index] += 1;
                    pcl_map[cur_index].type = 1;
                    if (point.z > pcl_map[cur_index].max_h){pcl_map[cur_index].max_h = point.z;}
                    if (point.z < pcl_map[cur_index].min_h){pcl_map[cur_index].min_h = point.z;}
                }
            }

            for (const auto &point : cloud_data.obs)
            {
                ai_node_percep_demo::detail::GridCellIndex cell;
                if (ai_node_percep_demo::detail::TryProjectPointToGrid(
                        point.x, point.y, point.z, map_width, map_resolution,
                        grid_geometry, &cell))
                {
                    const std::size_t cur_index = cell.linear;
                    sum_h_pcl[cur_index] += point.z; 
                    point_count_pcl[cur_index] += 1;
                    pcl_map[cur_index].type = 2;
                    if (point.z > pcl_map[cur_index].max_h){pcl_map[cur_index].max_h = point.z;}
                    if (point.z < pcl_map[cur_index].min_h){pcl_map[cur_index].min_h = point.z;}

                }
            }

            for (int idx = 0; idx < total_grids; idx++)
            {
                if (point_count_pcl[idx] > 0)
                {
                    pcl_map[idx].avg_h = sum_h_pcl[idx] / point_count_pcl[idx];
       
                }
            }
                    
            std::vector<float> sum_h_ai(total_grids, 0.0f); 
            std::vector<int> point_count_ai(total_grids, 0);
            for (const auto &point : cloud_data.ai)
            {
                const int point_label = point.label;
                const auto candidate_type =
                    ai_node_percep_demo::detail::ResolveAiGridType(
                        map_label, point_label);
                if (!candidate_type.has_value()) {
                    continue;
                }
                ai_node_percep_demo::detail::GridCellIndex cell;
                if (ai_node_percep_demo::detail::TryProjectPointToGrid(
                        point.x, point.y, point.z, map_width, map_resolution,
                        grid_geometry, &cell))
                {
                    const std::size_t cur_index = cell.linear;
                    sum_h_ai[cur_index] += point.z; 
                    point_count_ai[cur_index] += 1;
                    if (point.z > ai_map[cur_index].max_h){ai_map[cur_index].max_h = point.z;}
                    if (point.z < ai_map[cur_index].min_h){ai_map[cur_index].min_h = point.z;}
                    // Unknown labels were filtered above. Semantic candidates
                    // are merged deterministically so detector obstacles cannot
                    // be overwritten by grass/non-grass due to point order.
                    ai_map[cur_index].type =
                        ai_node_percep_demo::detail::MergeAiGridType(
                            ai_map[cur_index].type, *candidate_type);
                }
            }      
            for (int idx = 0; idx < total_grids; idx++)
            {
                if (point_count_ai[idx] > 0)
                {
                    ai_map[idx].avg_h = sum_h_ai[idx] / point_count_ai[idx];      
                }
            }
            
        }
};


#endif
