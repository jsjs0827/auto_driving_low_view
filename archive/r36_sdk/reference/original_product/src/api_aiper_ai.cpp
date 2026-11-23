#define STB_IMAGE_WRITE_IMPLEMENTATION
/*
 * @Description: AI inference API - using real MoGeneralSDK
 *              Fixed to match daye_s316_record.cpp SDK call style.
 */

#include "percep_demo/api_aiper_ai.h"


namespace Aiper_AI {

// MoGeneralSDK API 结构体。
// 关键：函数指针类型必须与原始可运行的 daye_s316_record.cpp 保持一致。
struct SDKApi {
    void* handle = nullptr;

    int (*setConfigPath)(const char*) = nullptr;
    int (*openCamera)(int*) = nullptr;
    int (*closeCamera)(const int) = nullptr;

    int (*getRgbdFrame)(const int, MoFrame*, MoFrame*, MoFrame*) = nullptr;
    int (*releaseRgbdFrame)(const int, MoFrame*, MoFrame*, MoFrame*) = nullptr;

    int (*getGdcFrame)(const int, MoFrame *, MoFrame *) = nullptr;
    int (*releaseGdcFrame)(const int, MoFrame *, MoFrame *) = nullptr;

    int (*getOrgFrame)(const int, MoFrame *, MoFrame *) = nullptr;
    int (*releaseOrgFrame)(const int, MoFrame *, MoFrame *) = nullptr;

    int (*getPointCloud)(const int, MoFrame*, std::vector<MoPoint>&) = nullptr;

    int (*getBinocularParam)(const int, const MoBinocularCameraParams **) = nullptr;
    int (*getParam)(const int, const MoMonoCameraParams *,double *,double *,int *,int *,int *) = nullptr;

};

// 全局状态
static SDKApi g_sdk;
static GridConfig g_grid_cfg;
static FrameResult g_frame_result;
static bool g_initialized = false;
static int g_camera_handle = 0;
const MoBinocularCameraParams *params = nullptr;
// 全局锁
static std::mutex g_frame_mtx;
// RGBDFrame 左/右图为 640x544 yuv420p，视差图为 640x544 uint16_t。
constexpr size_t FRAME_BUF_MAX = 640 * 544 * 3 / 2;
constexpr size_t DISP_BUF_MAX = 640 * 544 * sizeof(uint16_t);
constexpr size_t GDC_BUF_MAX = 1280 * 1088 * 3 / 2;
// 左/右/视差固定缓存
static uint8_t g_buf_left[FRAME_BUF_MAX];
static uint8_t g_buf_right[FRAME_BUF_MAX];
static uint8_t g_buf_disp[DISP_BUF_MAX];
static uint8_t g_buf_color[GDC_BUF_MAX];


static void FreeSingleMoFrame(MoFrame& frame)
{
    if (frame.Data) {
        uint8_t* buf = static_cast<uint8_t*>(frame.Data);
        // P5: 静态缓冲不可 delete[]，仅释放堆分配的帧数据
        if (buf != g_buf_left && buf != g_buf_right && buf != g_buf_disp && buf != g_buf_color) {
            delete[] buf;
        }
        frame.Data = nullptr;
    }
}

// P5: 支持静态缓冲复用，避免每帧 new/delete 堆分配
// static_buf 非 nullptr 且容量足够时，直接使用静态缓冲（不可 delete）；
// 否则降级为堆分配（如帧超出预设尺寸）。
static bool DeepCopyMoFrame(const MoFrame& src, MoFrame& dst,
                             uint8_t* static_buf, size_t static_buf_size)
{
    // 清空 dst 原有内存（区分静态/堆）
    if (dst.Data != nullptr) {
        uint8_t* old_buf = static_cast<uint8_t*>(dst.Data);
        if (old_buf != g_buf_left && old_buf != g_buf_right && old_buf != g_buf_disp && old_buf != g_buf_color) {
            delete[] old_buf;
        }
        dst.Data = nullptr;
    }

    dst.TimeStamp         = src.TimeStamp;
    dst.FrameId           = src.FrameId;
    dst.StreamType        = src.StreamType;
    dst.SensorId          = src.SensorId;
    dst.ResolutionWidth   = src.ResolutionWidth;
    dst.ResolutionHeight  = src.ResolutionHeight;
    dst.BitsPerPixel      = src.BitsPerPixel;
    dst.BytesUsed         = src.BytesUsed;
    dst.FillLightStatus   = src.FillLightStatus;
    dst.FillLightBrightness = src.FillLightBrightness;
    dst.exp_time          = src.exp_time;
    dst.analog_gain       = src.analog_gain;
    memcpy(dst.Reserved, src.Reserved, sizeof(dst.Reserved));

    if (src.Data != nullptr && src.BytesUsed > 0) {
        if (static_buf != nullptr && src.BytesUsed <= static_buf_size) {
            // 使用静态缓冲，零分配器压力
            memcpy(static_buf, static_cast<const uint8_t*>(src.Data), src.BytesUsed);
            dst.Data = static_buf;
        } else {
            // 降级：帧超出静态缓冲容量，回退到堆分配
            uint8_t* new_img_buf = new(std::nothrow) uint8_t[src.BytesUsed];
            if (!new_img_buf) {
                printf("[ERR] DeepCopyMoFrame malloc failed, size=%u\n", src.BytesUsed);
                return false;
            }
            memcpy(new_img_buf, static_cast<const uint8_t*>(src.Data), src.BytesUsed);
            dst.Data = new_img_buf;
        }
    } else {
        dst.Data = nullptr;
    }
    return true;
}
// 初始化 g_grid_cfg
static void initGridConfig() {
    g_grid_cfg.resolution_mm = 20.0;
    g_grid_cfg.front_range_mm = 2000.0;
    g_grid_cfg.side_range_mm = 1000.0;
    g_grid_cfg.camera_height_mm = 230.0;
    g_grid_cfg.ground_threshold_mm = 80.0;
    g_grid_cfg.min_height_mm = -300.0;
    g_grid_cfg.max_height_mm = 1000.0;
    g_grid_cfg.obstacle_height_mm = 150.0;
    g_grid_cfg.height_jump_mm = 180.0;
    g_grid_cfg.min_points_per_cell = 1;

    g_grid_cfg.side_cells =
        static_cast<int>(std::round(g_grid_cfg.side_range_mm /
                                    g_grid_cfg.resolution_mm));

    g_grid_cfg.front_cells =
        static_cast<int>(std::round(g_grid_cfg.front_range_mm /
                                    g_grid_cfg.resolution_mm));

    g_grid_cfg.rows = g_grid_cfg.side_cells * 2 + 1;
    g_grid_cfg.cols = g_grid_cfg.front_cells + 1;
}

static bool loadSymbol(void* handle, const char* name, void** fn) {
    *fn = dlsym(handle, name);
    if (!*fn) {
        std::cerr << "[Aiper_AI] missing symbol: " << name << std::endl;
        return false;
    }
    return true;
}

static SDKApi loadSdk(const std::string& so_path) {
    SDKApi api{};

    api.handle = dlopen(so_path.c_str(), RTLD_NOW | RTLD_GLOBAL);
    if (!api.handle) {
        std::cerr << "[Aiper_AI] dlopen failed: " << dlerror() << std::endl;
        return api;
    }

    if (!loadSymbol(api.handle, "moSetConfigPath",
                    reinterpret_cast<void**>(&api.setConfigPath))) {
        return api;
    }

    if (!loadSymbol(api.handle, "moOpenCamera",
                    reinterpret_cast<void**>(&api.openCamera))) {
        return api;
    }

    if (!loadSymbol(api.handle, "moCloseCamera",
                    reinterpret_cast<void**>(&api.closeCamera))) {
        return api;
    }

    api.getRgbdFrame =
        reinterpret_cast<int (*)(const int, MoFrame*, MoFrame*, MoFrame*)>(
            dlsym(api.handle, "moGetOneRGBDFrame"));

    api.releaseRgbdFrame =
        reinterpret_cast<int (*)(const int, MoFrame*, MoFrame*, MoFrame*)>(
            dlsym(api.handle, "moReleaseRGBDFrame"));

    api.getGdcFrame =
        reinterpret_cast<int (*)(const int, MoFrame*, MoFrame*)>(
            dlsym(api.handle, "moGetOneGdcFrame"));            

    api.releaseGdcFrame =
        reinterpret_cast<int (*)(const int, MoFrame*, MoFrame*)>(
            dlsym(api.handle, "moReleaseGdcFrame"));    

    api.getOrgFrame =
        reinterpret_cast<int (*)(const int, MoFrame*, MoFrame*)>(
            dlsym(api.handle, "moGetOneOrgFrame"));

    api.releaseOrgFrame =
        reinterpret_cast<int (*)(const int, MoFrame*, MoFrame*)>(
            dlsym(api.handle, "moReleaseOrgFrame"));

    api.getPointCloud =
        reinterpret_cast<int (*)(const int, MoFrame*, std::vector<MoPoint>&)>(
            dlsym(api.handle, "moGetPointCloudData"));

    api.getBinocularParam = 
        reinterpret_cast<int (*)(const int, const MoBinocularCameraParams **)>(
            dlsym(api.handle, "moGetOrgBinocularCameraParameter"));

    api.getParam = 
        reinterpret_cast<int (*)(const int, const MoMonoCameraParams *,double *,double *,int *,int *,int *)>(
            dlsym(api.handle, "moGetCameraParams"));

    if (!api.getRgbdFrame) {
        std::cerr << "[Aiper_AI] missing symbol: moGetOneRGBDFrame" << std::endl;
    }

    if (!api.releaseRgbdFrame) {
        std::cerr << "[Aiper_AI] missing symbol: moReleaseRGBDFrame" << std::endl;
    }
    if (!api.getGdcFrame) {
        std::cerr << "[Aiper_AI] missing symbol: moGetOneGdcFrame" << std::endl;
    }

    if (!api.releaseGdcFrame) {
        std::cerr << "[Aiper_AI] missing symbol: moReleaseGdcFrame" << std::endl;
    }

    if (!api.getPointCloud) {
        std::cerr << "[Aiper_AI] missing symbol: moGetPointCloudData" << std::endl;
    }

    return api;
}

// 灰度计算
static uint8_t clamp_u8_int(int v) {
    if (v < 0) return 0;
    if (v > 255) return 255;
    return static_cast<uint8_t>(v);
}

static uint8_t heightToGray(int mean_h_mm,
                            int diff_h_mm,
                            const GridConfig& cfg) {
    const double ground_th = std::max(1.0, cfg.ground_threshold_mm);
    const double obs_th = std::max(ground_th + 1.0, cfg.obstacle_height_mm);
    const double jump_th = std::max(1.0, cfg.height_jump_mm);

    if (diff_h_mm >= jump_th) {
        return 255;
    }

    if (std::abs(mean_h_mm) <= ground_th) {
        double t = std::abs(static_cast<double>(mean_h_mm)) / ground_th;
        return clamp_u8_int(static_cast<int>(std::round(60.0 + t * 60.0)));
    }

    if (mean_h_mm < -ground_th) {
        double t = std::min(
            1.0,
            (std::abs(static_cast<double>(mean_h_mm)) - ground_th) / obs_th);
        return clamp_u8_int(static_cast<int>(std::round(30.0 + t * 40.0)));
    }

    if (mean_h_mm < obs_th) {
        double t = (static_cast<double>(mean_h_mm) - ground_th) /
                   (obs_th - ground_th);
        t = std::max(0.0, std::min(1.0, t));
        return clamp_u8_int(static_cast<int>(std::round(120.0 + t * 90.0)));
    }

    double t = (static_cast<double>(mean_h_mm) - obs_th) / jump_th;
    t = std::max(0.0, std::min(1.0, t));
    return clamp_u8_int(static_cast<int>(std::round(210.0 + t * 45.0)));
}


// 转换点云格式：SDK 内部用 MoPoint，对外输出 AiperPoint。
static void convertPoints(PointCloudT::Ptr& cloud,const std::vector<MoPoint>& mo_points, const PointCloudPubConfig& box) {
    cloud->reserve(mo_points.size());
    const int32_t min_z_mm = static_cast<int32_t>(box.box_min_z * 1000.f);
    const int32_t max_z_mm = static_cast<int32_t>(box.box_max_z * 1000.f);
    const int32_t min_x_mm = static_cast<int32_t>(box.box_min_x * 1000.f);
    const int32_t max_x_mm = static_cast<int32_t>(box.box_max_x * 1000.f);
    const int32_t min_y_mm = static_cast<int32_t>(box.box_min_y * 1000.f);
    const int32_t max_y_mm = static_cast<int32_t>(box.box_max_y * 1000.f);
    constexpr float scale = 0.001f;
    for (const auto& sp : mo_points) {
        if (sp.Z < min_z_mm || sp.Z > max_z_mm ||
            sp.X < min_x_mm || sp.X > max_x_mm ||
            sp.Y < min_y_mm || sp.Y > max_y_mm)
        {
            continue;
        }
        AiperPclPoint p;
        p.x = static_cast<float>(sp.X) * scale;
        p.y = static_cast<float>(sp.Y) * scale;
        p.z = static_cast<float>(sp.Z) * scale;
        p.label = 0.0f;
        p.confidence = 0.9f;
        cloud->push_back(p);
    }
}


// static void convertPoints(PointCloudT::Ptr& cloud,
//                           const std::vector<MoPoint>& mo_points,
//                           const PointCloudPubConfig& box)
// {
      
//     const int32_t min_z_mm = static_cast<int32_t>(box.box_min_z * 1000.f);
//     const int32_t max_z_mm = static_cast<int32_t>(box.box_max_z * 1000.f);
//     const int32_t min_x_mm = static_cast<int32_t>(box.box_min_x * 1000.f);
//     const int32_t max_x_mm = static_cast<int32_t>(box.box_max_x * 1000.f);
//     const int32_t min_y_mm = static_cast<int32_t>(box.box_min_y * 1000.f);
//     const int32_t max_y_mm = static_cast<int32_t>(box.box_max_y * 1000.f);
//     constexpr float scale = 0.001f;
//     constexpr float label_val = 0.0f;
//     constexpr float conf_val = 0.9f;
//     const size_t total = mo_points.size();
//     std::vector<size_t> thread_valid(omp_get_max_threads(), 0);

//     // 多线程分片统计有效点
//     #pragma omp parallel
//     {
//         int tid = omp_get_thread_num();
//         size_t cnt = 0;
//         #pragma omp for nowait
//         for (size_t i = 0; i < total; i++) {
//             const auto& sp = mo_points[i];
//             if (sp.Z >= min_z_mm && sp.Z <= max_z_mm &&
//                 sp.X >= min_x_mm && sp.X <= max_x_mm &&
//                 sp.Y >= min_y_mm && sp.Y <= max_y_mm)
//                 cnt++;
//         }
//         thread_valid[tid] = cnt;
//     }
//     // 计算总有效点 + 每个线程写入偏移
//     std::vector<size_t> offset(thread_valid.size(), 0);
//     size_t valid_cnt = 0;
//     for (int i = 0; i < offset.size(); i++) {
//         offset[i] = valid_cnt;
//         valid_cnt += thread_valid[i];
//     }

//     auto& pts = cloud->points;
//     pts.resize(valid_cnt);

//     // 多线程并行填充点云
//     #pragma omp parallel
//     {
//         int tid = omp_get_thread_num();
//         size_t idx = offset[tid];
//         #pragma omp for nowait
//         for (size_t i = 0; i < total; i++) {
//             const auto& sp = mo_points[i];
//             if (sp.Z < min_z_mm || sp.Z > max_z_mm ||
//                 sp.X < min_x_mm || sp.X > max_x_mm ||
//                 sp.Y < min_y_mm || sp.Y > max_y_mm)
//                 continue;
//             auto& p = pts[idx++];
//             p.x = static_cast<float>(sp.X) * scale;
//             p.y = static_cast<float>(sp.Y) * scale;
//             p.z = static_cast<float>(sp.Z) * scale;
//             p.label = label_val;
//             p.confidence = conf_val;
//         }
//     }
//     cloud->width = valid_cnt;
//     cloud->height = 1;
// }
// 关键：完全复刻原始 daye_s316_record.cpp 的取点云顺序。
// getRgbdFrame -> getPointCloud -> releaseRgbdFrame
static bool getPointCloudOnce(std::vector<MoPoint>* points,
                              MoFrame* left_out,
                              MoFrame* right_out,
                              MoFrame* disp_out,
                              MoFrame* color_out) {
    if (!g_initialized || !g_sdk.handle) {
        std::cerr << "[Aiper_AI] getPointCloudOnce failed: not initialized"
                  << std::endl;
        return false;
    }

    if (!g_sdk.getRgbdFrame ||
        !g_sdk.releaseRgbdFrame ||
        !g_sdk.getGdcFrame ||
        !g_sdk.releaseGdcFrame ||
        !g_sdk.getPointCloud ||
        !points) {
        std::cerr << "[Aiper_AI] getPointCloudOnce failed: sdk function null"
                  << std::endl;
        return false;
    }

    MoFrame left_ori{};
    MoFrame right_ori{};
    MoFrame disp{};

    const int ret_frame =
        g_sdk.getRgbdFrame(g_camera_handle, &left_ori, &right_ori, &disp);

    //计算频率低于100ms每秒会报错
    if (ret_frame != MO_REV_OK) {
        // std::cerr << "[Aiper_AI] getRgbdFrame failed, ret="
        //           << ret_frame
        //           << ", disp.Data=" << static_cast<void*>(disp.Data)
        //           << std::endl;
        g_sdk.releaseRgbdFrame(g_camera_handle, &left_ori, &right_ori, &disp);
        return false;
    }
    if ( !left_ori.Data  || !right_ori.Data || !disp.Data)
    {
        std::cerr << "[Aiper_AI ERR] sdk get Frame data null"<< std::endl;
        return false;
    }

    // P5: 使用静态缓冲，避免每帧堆分配
    if (!DeepCopyMoFrame(left_ori, *left_out,   g_buf_left,  FRAME_BUF_MAX) ||
        !DeepCopyMoFrame(right_ori, *right_out, g_buf_right, FRAME_BUF_MAX) ||
        !DeepCopyMoFrame(disp,      *disp_out,  g_buf_disp,  DISP_BUF_MAX))
    {
        g_sdk.releaseRgbdFrame(g_camera_handle, &left_ori, &right_ori, &disp);
        return false;
    }

    // points->clear();
    // const int ret_cloud =
    //     g_sdk.getPointCloud(g_camera_handle, &disp, *points);

    // // 关键：release 必须放在 getPointCloud 后面。
    g_sdk.releaseRgbdFrame(g_camera_handle, &left_ori, &right_ori, &disp);

    // if (ret_cloud != MO_REV_OK || points->empty()) {
    //     std::cerr << "[Aiper_AI] getPointCloud failed, ret="
    //               << ret_cloud
    //               << ", points=" << points->size()
    //               << std::endl;
    //     return false;
    // }

    if (color_out) {
        MoFrame left_color{};
        MoFrame right_color{};
        bool got_color = false;
        bool color_from_org = false;

        // OrgFrame 是 SDK 明确标注的彩色原图；GdcFrame 在部分固件上可能接近灰度。
        // if (g_sdk.getOrgFrame && g_sdk.releaseOrgFrame) {
        //     const int ret_org = g_sdk.getOrgFrame(g_camera_handle, &left_color, &right_color);
        //     got_color = (ret_org == MO_REV_OK && left_color.Data);
        //     color_from_org = got_color;
        //     if (!got_color && (left_color.Data || right_color.Data)) {
        //         g_sdk.releaseOrgFrame(g_camera_handle, &left_color, &right_color);
        //         left_color = MoFrame{};
        //         right_color = MoFrame{};
        //     }
        // }

        // if (!got_color) {
            const int ret_gdc = g_sdk.getGdcFrame(g_camera_handle, &left_color, &right_color);
            got_color = (ret_gdc == MO_REV_OK && left_color.Data);
            color_from_org = false;
        // }

        if (got_color) {
            if (!DeepCopyMoFrame(left_color, *color_out, g_buf_color, GDC_BUF_MAX)) {
                color_out->Data = nullptr;
            }
        } else {
            color_out->Data = nullptr;
        }

        if (left_color.Data || right_color.Data) {
            if (color_from_org) {
                g_sdk.releaseOrgFrame(g_camera_handle, &left_color, &right_color);
            } else {
                g_sdk.releaseGdcFrame(g_camera_handle, &left_color, &right_color);
            }
        }
    }

    return true;
}

}  // namespace Aiper_AI

// C 风格 API 函数
extern "C" {

// 初始化 - 加载 SDK 并打开相机
int aiper_init(const char* sdk_so_path, const char* cfg_path) {
    if (Aiper_AI::g_initialized) {
        std::cout << "[Aiper_AI] already initialized" << std::endl;
        return 0;
    }

    const std::string so_path =
        sdk_so_path ? sdk_so_path : "/app/data/metoak/lib/libMoGeneralSDK.so";

    Aiper_AI::g_sdk = Aiper_AI::loadSdk(so_path);

    if (!Aiper_AI::g_sdk.handle ||
        !Aiper_AI::g_sdk.getBinocularParam ||
        !Aiper_AI::g_sdk.getParam ||
        !Aiper_AI::g_sdk.setConfigPath ||
        !Aiper_AI::g_sdk.openCamera ||
        !Aiper_AI::g_sdk.closeCamera ||
        !Aiper_AI::g_sdk.getRgbdFrame ||
        !Aiper_AI::g_sdk.releaseRgbdFrame ||
        !Aiper_AI::g_sdk.getGdcFrame ||
        !Aiper_AI::g_sdk.releaseGdcFrame ||
        !Aiper_AI::g_sdk.getPointCloud) {
        std::cerr << "[Aiper_AI] SDK load failed" << std::endl;
        dlclose(Aiper_AI::g_sdk.handle);
        Aiper_AI::g_sdk.handle = nullptr;
        return -1;
    }

    if (cfg_path && cfg_path[0]) {
        const int ret_cfg = Aiper_AI::g_sdk.setConfigPath(cfg_path);
        if (ret_cfg != MO_REV_OK) {
            std::cerr << "[Aiper_AI] setConfigPath failed, ret="
                      << ret_cfg
                      << ", path="
                      << cfg_path
                      << std::endl;
            return -1;
        }
    }   

    // if (Aiper_AI::g_sdk.getBinocularParam(Aiper_AI::g_camera_handle, &Aiper_AI::params) != MO_REV_OK) {
    //     std::cout << "[INFO] binocular params not available" << std::endl;
    //     return -1; 
    // }    

    // const int ret_open =
    //     Aiper_AI::g_sdk.openCamera(&Aiper_AI::g_camera_handle);

    // if (ret_open != MO_REV_OK) {
    //     std::cerr << "[Aiper_AI] openCamera failed, ret="
    //               << ret_open
    //               << std::endl;
    //     return -1;
    // }

    Aiper_AI::g_initialized = true;

    std::cout << "[Aiper_AI] initialized, camera handle="
              << Aiper_AI::g_camera_handle
              << std::endl;

    return 0;
}
Aiper_AI::CameraParams* aiper_getCameraParams()
{
    MoMonoCameraParams pParams{};
    double pBxf,pBaseline;
    int pResolutionWidth,pResolutionHeight,pDspMaxValue;
    int ret = Aiper_AI::g_sdk.getParam(Aiper_AI::g_camera_handle, &pParams,&pBxf,&pBaseline,&pResolutionWidth,&pResolutionHeight,&pDspMaxValue);
     cout<<"DspMaxValue:"<<pDspMaxValue<<"  bxf:"<<pBxf<<endl;
    std::cout<<"ret from moGetCameraParams:"<<ret<<std::endl;
    if (ret != MO_REV_OK) {
        std::cout << "[INFO] binocular params not available" << std::endl;
        return nullptr;
    }   
    static Aiper_AI::CameraParams cameraParams;
    cameraParams.fx = pParams.fx;
    cameraParams.fy = pParams.fy;
    cameraParams.cx = pParams.cx;
    cameraParams.cy = pParams.cy;    
    cameraParams.pBxf = pBxf;
    cameraParams.pBaseline = pBaseline;
    cameraParams.pResolutionWidth = pResolutionWidth;
    cameraParams.pResolutionHeight = pResolutionHeight;
    cameraParams.pDspMaxValue = pDspMaxValue;

    return &cameraParams;
}
const MoBinocularCameraParams * aiper_saveCameraParams(const std::string &path) 
{
    FILE *fp = fopen(path.c_str(), "w");
    if (!fp) return nullptr; 

    fprintf(fp, "# Left Camera Intrinsics\n");
    fprintf(fp, "left.fx=%.9f\n", (double)Aiper_AI::params->leftCamera.fx);
    fprintf(fp, "left.fy=%.9f\n", (double)Aiper_AI::params->leftCamera.fy);
    fprintf(fp, "left.cx=%.9f\n", (double)Aiper_AI::params->leftCamera.cx);
    fprintf(fp, "left.cy=%.9f\n", (double)Aiper_AI::params->leftCamera.cy);

    for (int i = 0; i < 6; ++i)
    {
        fprintf(fp, "left.k%d=%.9f\n", i, (double)Aiper_AI::params->leftCamera.radial_distortion_k[i]);
    }
        
    fprintf(fp, "left.p1=%.9f\n", (double)Aiper_AI::params->leftCamera.tangential_distortion_p[0]);
    fprintf(fp, "left.p2=%.9f\n", (double)Aiper_AI::params->leftCamera.tangential_distortion_p[1]);

    fprintf(fp, "\n# Right Camera Intrinsics\n");
    fprintf(fp, "right.fx=%.9f\n", (double)Aiper_AI::params->rightCamera.fx);
    fprintf(fp, "right.fy=%.9f\n", (double)Aiper_AI::params->rightCamera.fy);
    fprintf(fp, "right.cx=%.9f\n", (double)Aiper_AI::params->rightCamera.cx);
    fprintf(fp, "right.cy=%.9f\n", (double)Aiper_AI::params->rightCamera.cy);

    for (int i = 0; i < 6; ++i)
    {
        fprintf(fp, "right.k%d=%.9f\n", i, (double)Aiper_AI::params->rightCamera.radial_distortion_k[i]);
    }
        
    fprintf(fp, "right.p1=%.9f\n", (double)Aiper_AI::params->rightCamera.tangential_distortion_p[0]);
    fprintf(fp, "right.p2=%.9f\n", (double)Aiper_AI::params->rightCamera.tangential_distortion_p[1]);

    fprintf(fp, "\n# Extrinsics (R|t)\n");
    for (int i = 0; i < 3; ++i)
        fprintf(fp, "rotation.r%d=%.9f\n", i, (double)Aiper_AI::params->rotation_r[i]);
    for (int i = 0; i < 3; ++i)
        fprintf(fp, "translation.t%d=%.9f\n", i, (double)Aiper_AI::params->translation_t[i]);

    fprintf(fp, "\n# Resolution\n");
    fprintf(fp, "width=%u\n", Aiper_AI::params->resolution_width);
    fprintf(fp, "height=%u\n", Aiper_AI::params->resolution_height);

    fclose(fp);
    std::cout << "[INFO] camera params saved: " << path << std::endl;
    return Aiper_AI::params;
}

// 处理一帧 - 获取相机各路数据
Aiper_AI::FrameResult* aiper_processFrame(const PointCloudPubConfig& box) {
    // std::lock_guard<std::mutex> lock(Aiper_AI::g_frame_mtx);
    Aiper_AI::FrameResult* res = new Aiper_AI::FrameResult{};
    res->mapped_points.reset(new PointCloudT);
    // res->mapped_points.clear();

    std::vector<MoPoint> mo_points;
    MoFrame left_frame{}, right_frame{}, disp_frame{}, color_frame{};

auto start = std::chrono::high_resolution_clock::now();
    if (!Aiper_AI::getPointCloudOnce(&mo_points, &left_frame, &right_frame, &disp_frame, &color_frame))
    {
        return res;
    }
auto end = std::chrono::high_resolution_clock::now();
// auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end- start);
//     std::cout<<"getPointCloudOnce Time taken: "<<duration.count()<<"ms"<<std::endl;
    res->left_frame  = left_frame;
    res->right_frame = right_frame;
    res->disp_frame  = disp_frame;
    res->color_frame = color_frame;
// auto start1 = std::chrono::high_resolution_clock::now();
//      Aiper_AI::convertPoints(res->mapped_points,mo_points,box);
// auto end1 = std::chrono::high_resolution_clock::now();
// auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end1- start1);
//     std::cout<<"convertPoints Time taken: "<<duration1.count()<<"ms"<<std::endl;
    return res;
}

void aiper_freeResult(Aiper_AI::FrameResult* result) {
    // 当前返回的是全局静态结果，不需要 delete。
    // 保持接口兼容，暂时只清理 mapped_points。
    if (!result) return;

    Aiper_AI::FreeSingleMoFrame(result->left_frame);
    Aiper_AI::FreeSingleMoFrame(result->right_frame);
    Aiper_AI::FreeSingleMoFrame(result->disp_frame);
    Aiper_AI::FreeSingleMoFrame(result->color_frame);

    delete result;
}

bool aiper_openCamera()
{
    
    if (Aiper_AI::g_sdk.openCamera)
    {
        const int ret_open =
            Aiper_AI::g_sdk.openCamera(&Aiper_AI::g_camera_handle);

        if (ret_open != MO_REV_OK) {
            std::cerr << "[Aiper_AI] openCamera failed, ret="<< ret_open<< std::endl;
            LOG_ERROR("[percepDemo]sdk open camera failed: ret = %d",ret_open);
            return false;
        }        
    }else{
        LOG_ERROR("[percepDemo]sdk can not open camera!!!");
        return false;}

    return true;
}
bool aiper_closeCamera()
{
            
    if (Aiper_AI::g_camera_handle != 0 && Aiper_AI::g_sdk.closeCamera) 
    {
        const int ret_close = Aiper_AI::g_sdk.closeCamera(Aiper_AI::g_camera_handle);
        if (ret_close != MO_REV_OK) {
            std::cerr << "[Aiper_AI] closeCamera failed, ret="<< ret_close<< std::endl;
            LOG_ERROR("[percepDemo]sdk close camera failed: ret = %d",ret_close);
            return false;
        }    
    }else{
        LOG_ERROR("[percepDemo]sdk can not close camera!!!");
        return false;
    }
    return true;
}

void aiper_destroy() {
    if (Aiper_AI::g_sdk.handle) {
        if (Aiper_AI::g_camera_handle != 0 &&
            Aiper_AI::g_sdk.closeCamera) {
                LOG_INFO("[percepDemo]close camera completed!");
            Aiper_AI::g_sdk.closeCamera(Aiper_AI::g_camera_handle);
        }
        LOG_INFO("[percepDemo]camera sdk released completed!");
        dlclose(Aiper_AI::g_sdk.handle);
        Aiper_AI::g_sdk.handle = nullptr;
    }
    Aiper_AI::g_camera_handle = 0;
    Aiper_AI::g_initialized = false;
    
}

}  // extern "C"

namespace Aiper_AI {

// 工厂函数
AIApi* createAiperAI(const std::string& so_path) {
    (void)so_path;

    AIApi* api = new AIApi();

    api->handle = nullptr;

    api->init =
        [](const char* sdk_so, const char* cfg) {
            return aiper_init(sdk_so, cfg);
        };

    api->processFrame =
        [](const PointCloudPubConfig& box) {
            
            return aiper_processFrame(box);
        };

    api->freeResult =
        [](FrameResult* r) {
            aiper_freeResult(r);
        };

    api->saveMonoCameraParams = 
        [](const std::string path) {
            return aiper_saveCameraParams(path);
        };
    api->getCameraParams = 
        []() {
            return aiper_getCameraParams();
        };
    
    api->openCamera = 
        []() {
            return aiper_openCamera();
        };
    api->closeCamera = 
        []() {
            return aiper_closeCamera();
        };
    api->destroy =
        []() {
            aiper_destroy();
        };

    return api;
}

}  // namespace Aiper_AI
