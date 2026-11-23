#include "r33/mapping.hpp"
#include "mapping_algorithms.hpp"
#include "percep_demo/pre_pointcloud_types.hpp"

#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace r33 {
MappingRgbImage renderOriginalMap(const std::vector<MapPoint>& map,
    double width, double height, double resolution, double x, double y);
namespace {
using namespace ai_node_percep_demo;

std::string readFile(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("cannot read mapping config: " + path);
  return std::string(std::istreambuf_iterator<char>(input), {});
}
rapidjson::Document parse(const std::string& json, const char* name) {
  rapidjson::Document doc;
  doc.Parse(json.c_str());
  if (doc.HasParseError() || !doc.IsObject())
    throw std::invalid_argument(std::string("invalid mapping JSON: ") + name);
  return doc;
}
double number(const rapidjson::Value& v, const char* n, double f) {
  return v.HasMember(n) && v[n].IsNumber() ? v[n].GetDouble() : f;
}
int integer(const rapidjson::Value& v, const char* n, int f) {
  return v.HasMember(n) && v[n].IsInt() ? v[n].GetInt() : f;
}
bool boolean(const rapidjson::Value& v, const char* n, bool f) {
  return v.HasMember(n) && v[n].IsBool() ? v[n].GetBool() : f;
}
std::string stringValue(const rapidjson::Value& v, const char* n, const std::string& f) {
  return v.HasMember(n) && v[n].IsString() ? v[n].GetString() : f;
}
const rapidjson::Value* object(const rapidjson::Value& v, const char* n) {
  return v.HasMember(n) && v[n].IsObject() ? &v[n] : nullptr;
}
void copyCloud(const PointCloudT::Ptr& source, std::vector<MappingPoint>& dest) {
  dest.clear();
  if (!source) return;
  dest.reserve(source->size());
  for (const auto& p : *source)
    dest.push_back({p.x,p.y,p.z,p.label,p.confidence,p.r,p.g,p.b});
}
std::vector<MappingCell> copyMap(const std::vector<MapPoint>& map) {
  std::vector<MappingCell> out;
  out.reserve(map.size());
  for (const auto& p : map) out.push_back({p.min_h,p.max_h,p.avg_h,p.type});
  return out;
}
}  // namespace

class MappingPipeline::Impl {
 public:
  explicit Impl(const MappingConfig& c) : config(c) {
    if (config.sample_step < 1 || config.sample_step > 8192)
      throw std::invalid_argument("mapping sample_step must be in [1,8192]");
    const auto start = config.start_config_json.empty()
        ? readFile(config.start_config_path) : config.start_config_json;
    const auto extrinsics = config.extrinsics_json.empty()
        ? readFile(config.extrinsics_path) : config.extrinsics_json;
    start_doc = parse(start, "start_config");
    extrinsics_doc = parse(extrinsics, "extrinsics");
    const auto* e = object(extrinsics_doc, "extrinsics");
    if (!e) throw std::invalid_argument("extrinsics object missing");
    alg.cali.x = number(*e,"x",0);
    alg.cali.y = number(*e,"y",0);
    alg.cali.z = number(*e,"z",0);
    alg.cali.roll = angle_to_rad(static_cast<float>(number(*e,"roll",0)));
    alg.cali.pitch = angle_to_rad(static_cast<float>(number(*e,"pitch",0)));
    alg.cali.yaw = angle_to_rad(static_cast<float>(number(*e,"yaw",0)));
    if (!std::isfinite(alg.cali.x) || !std::isfinite(alg.cali.y) ||
        !std::isfinite(alg.cali.z) || !std::isfinite(alg.cali.roll) ||
        !std::isfinite(alg.cali.pitch) || !std::isfinite(alg.cali.yaw))
      throw std::invalid_argument("non-finite mapping extrinsics");
    // The original stores translation through float casts as well.
    alg.cali.x = static_cast<float>(alg.cali.x);
    alg.cali.y = static_cast<float>(alg.cali.y);
    alg.cali.z = static_cast<float>(alg.cali.z);
    auto_cali = boolean(start_doc,"auto_cali",false);
    if (auto_cali && !config.run_calibration)
      throw std::invalid_argument("auto_cali requires an enabled graph calibration feature");
    if (const auto* v = object(start_doc,"debug"))
      visualize = boolean(*v,"visualize",false);
    if (const auto* v = object(start_doc,"map")) {
      width = number(*v,"MapWidth",width);
      height = number(*v,"MapHeight",height);
      resolution = number(*v,"MapResolution",resolution);
    }
    const auto geometry = detail::TryMakeGridGeometry(width,height,resolution);
    if (!geometry || !detail::TryMakeGridVisualizationGeometry(*geometry))
      throw std::invalid_argument("invalid original map geometry");
    resolution = geometry->resolution_m;
    if (const auto* v = object(start_doc,"point_cloud_pub")) {
      if (v->HasMember("max_points") && (*v)["max_points"].IsUint())
        box.max_points = (*v)["max_points"].GetUint();
#define R33_BOX(N) box.N = static_cast<float>(number(*v,#N,box.N))
      R33_BOX(box_min_x); R33_BOX(box_max_x); R33_BOX(box_min_y);
      R33_BOX(box_max_y); R33_BOX(box_min_z); R33_BOX(box_max_z);
#undef R33_BOX
    }
    if (!(std::isfinite(box.box_min_x) && std::isfinite(box.box_max_x) &&
          std::isfinite(box.box_min_y) && std::isfinite(box.box_max_y) &&
          std::isfinite(box.box_min_z) && std::isfinite(box.box_max_z) &&
          box.box_min_x <= box.box_max_x && box.box_min_y <= box.box_max_y &&
          box.box_min_z <= box.box_max_z))
      throw std::invalid_argument("invalid mapping cloud bounds");
    if (const auto* v = object(start_doc,"point_cloud_color")) {
      color.enabled = boolean(*v,"enabled",color.enabled);
      color.use_gdc_image = boolean(*v,"use_gdc_image",color.use_gdc_image);
      color.height_color_fallback = boolean(*v,"height_color_fallback",color.height_color_fallback);
    }
    if (const auto* v = object(start_doc,"patchwork")) {
      auto& pw = alg.patchwork_cfg_;
      pw.enabled = boolean(*v,"enabled",pw.enabled);
      pw.verbose = boolean(*v,"verbose",pw.verbose);
      pw.publish_ground = boolean(*v,"publish_ground",pw.publish_ground);
      pw.publish_nonground = boolean(*v,"publish_nonground",pw.publish_nonground);
      pw.sensor_height = number(*v,"sensor_height",pw.sensor_height);
      pw.min_range = number(*v,"min_range",pw.min_range);
      pw.max_range = number(*v,"max_range",pw.max_range);
      pw.th_dist = number(*v,"th_dist",pw.th_dist);
      pw.th_seeds = number(*v,"th_seeds",pw.th_seeds);
      pw.log_every_n = integer(*v,"log_every_n",pw.log_every_n);
    }
    const auto& pw = alg.patchwork_cfg_;
    if (!std::isfinite(pw.sensor_height) || !std::isfinite(pw.min_range) ||
        !std::isfinite(pw.max_range) || !std::isfinite(pw.th_dist) ||
        !std::isfinite(pw.th_seeds) || pw.min_range < 0 ||
        pw.max_range <= pw.min_range || pw.th_dist < 0 || pw.th_seeds < 0)
      throw std::invalid_argument("invalid original Patchwork configuration");
    if (const auto* v = object(start_doc,"dnn")) {
      alg.dnn_cfg_.enabled = boolean(*v,"enabled",false);
      alg.det_match_cfg_.enabled = boolean(*v,"publish_det_cloud",alg.det_match_cfg_.enabled);
      alg.det_match_cfg_.score_min = static_cast<float>(number(*v,"det_match_score_min",alg.det_match_cfg_.score_min));
      alg.det_match_cfg_.depth_span_m = static_cast<float>(number(*v,"det_depth_span_m",alg.det_match_cfg_.depth_span_m));
      alg.det_match_cfg_.max_depth_m = static_cast<float>(number(*v,"det_max_depth_m",alg.det_match_cfg_.max_depth_m));
      if (const auto* d = object(*v,"detect")) {
        alg.dnn_cfg_.detect.enabled = boolean(*d,"enabled",true);
        alg.dnn_cfg_.detect.class_names = stringValue(*d,"class_names",alg.dnn_cfg_.detect.class_names);
      }
      if (const auto* s = object(*v,"seg")) {
        alg.dnn_cfg_.seg.enabled = boolean(*s,"enabled",true);
        alg.dnn_cfg_.seg.class_names = stringValue(*s,"class_names",alg.dnn_cfg_.seg.class_names);
      }
    }
    alg.parseLabelFusionConfig(start_doc);
    alg.aiFusionPtr.reset(new AiFusion(width,height,resolution,0.05,5,6,
        alg.cali.x,alg.cali.y,box.box_min_z));
    alg.initLabelFusionClassIds();
    if (pw.enabled) {
      patchwork::Params params;
      params.verbose = pw.verbose;
      params.sensor_height = pw.sensor_height;
      params.min_range = pw.min_range;
      params.max_range = pw.max_range;
      params.th_dist = pw.th_dist;
      params.th_seeds = pw.th_seeds;
      params.enable_RNR = false;
      params.num_iter = 3; params.num_lpr = 20; params.num_min_pts = 0;
      params.th_seeds_v = 0.8; params.th_dist_v = 0.05;
      params.uprightness_thr = 0.101;
      params.num_zones = 1; params.num_rings_of_interest = 1;
      alg.patchwork_.reset(new patchwork::PatchWorkpp(params));
    }
  }

  bool stageEnabled(int stage) const {
    switch (stage) {
      case 2: return config.run_calibration;
      case 3: return config.run_labels;
      case 4: return config.run_transform;
      case 5: return config.run_ground;
      case 6: return config.run_maps;
      default: return true;
    }
  }
  void skipDisabled() { while (next_stage < 7 && !stageEnabled(next_stage)) ++next_stage; }
  void stage(int expected, const char* name) {
    skipDisabled();
    if (!stageEnabled(expected))
      throw std::logic_error(std::string("disabled mapping stage invoked: ") + name);
    if (next_stage != expected)
      throw std::logic_error(std::string("mapping stage order error: ") + name);
    ++next_stage;
    skipDisabled();
  }
  void updateCalibrationResult() {
    auto& out = output->calibration;
    out.active = auto_cali;
    out.accumulated_frames = cali_count;
    out.x = alg.cali.x; out.y = alg.cali.y; out.z = alg.cali.z;
    out.roll_degrees = rad_to_angle(alg.cali.roll);
    out.pitch_degrees = rad_to_angle(alg.cali.pitch);
    out.yaw_degrees = rad_to_angle(alg.cali.yaw);
  }
  MappingConfig config;
  rapidjson::Document start_doc, extrinsics_doc;
  ai_node_percep_demo::MappingAlgorithms alg;
  PointCloudPubConfig box;
  PointCloudColorConfig color;
  bool auto_cali = false, visualize = false;
  double width = 1.6, height = 2.0, resolution = 0.02;
  int next_stage = 0, cali_count = 0;
  float sum_roll = 0, sum_pitch = 0, sum_h = 0;
  MappingFrame frame;
  std::vector<std::uint16_t> disparity;
  std::vector<std::uint8_t> rgb, mask;
  std::vector<ai_node_percep_demo::DnnDetection> detections;
  PointCloudT::Ptr raw{new PointCloudT}, camera{new PointCloudT};
  Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
  std::shared_ptr<MappingResult> output;
  struct Semantic {
    std::vector<std::uint8_t> mask;
    std::vector<ai_node_percep_demo::DnnDetection> detections;
  };
  static Semantic copySemantic(const MappingFrame& f) {
    if (f.semantic_result_valid && f.semantic_frame_id != f.frame_id)
      throw std::invalid_argument("mapping semantic result frame mismatch");
    const auto pixels=static_cast<std::size_t>(f.width)*f.height;
    if (f.segmentation_bytes && (!f.segmentation_mask ||
        f.segmentation_width != f.width || f.segmentation_height != f.height ||
        f.segmentation_bytes != pixels))
      throw std::invalid_argument("mapping semantic mask size mismatch");
    Semantic semantic;
    if (!f.semantic_result_valid) return semantic;
    if (f.segmentation_bytes)
      semantic.mask.assign(f.segmentation_mask,f.segmentation_mask+f.segmentation_bytes);
    semantic.detections.reserve(f.detections.size());
    for (const auto& d:f.detections) {
      if (!(std::isfinite(d.confidence) && std::isfinite(d.x1) &&
            std::isfinite(d.y1) && std::isfinite(d.x2) && std::isfinite(d.y2)))
        throw std::invalid_argument("non-finite mapping detection");
      ai_node_percep_demo::DnnDetection copy;
      copy.class_id=d.class_id;copy.confidence=d.confidence;
      copy.x1=d.x1;copy.y1=d.y1;copy.x2=d.x2;copy.y2=d.y2;
      semantic.detections.push_back(std::move(copy));
    }
    return semantic;
  }
  void acceptSemantic(const MappingFrame& f,Semantic semantic) {
    mask=std::move(semantic.mask);detections=std::move(semantic.detections);
    frame.semantic_result_valid=f.semantic_result_valid;
    frame.semantic_frame_id=f.semantic_frame_id;
    // Only native owned copies are used by the original fusion algorithms.
    frame.detections.clear();
    frame.segmentation_mask=mask.empty()?nullptr:mask.data();
    frame.segmentation_bytes=mask.size();
    frame.segmentation_width=mask.empty()?0:frame.width;
    frame.segmentation_height=mask.empty()?0:frame.height;
  }
};

MappingPipeline::MappingPipeline(const MappingConfig& config) : impl_(new Impl(config)) {}
MappingPipeline::~MappingPipeline() = default;

void MappingPipeline::beginFrame(const MappingFrame& f) {
  auto& p = *impl_;
  const auto validation = ai_node_percep_demo::validateDisparityBuffer(
      f.disparity,f.width,f.height,f.disparity_bytes);
  if (!validation || f.disparity_stride_samples != static_cast<int>(validation.layout.stride_samples))
    throw std::invalid_argument("invalid Q8 disparity buffer or stride");
  const auto& k = f.intrinsics;
  if (!std::isfinite(k.fx) || !std::isfinite(k.fy) || !std::isfinite(k.cx) ||
      !std::isfinite(k.cy) || !std::isfinite(k.bxf_m) || k.fx <= 0 || k.fy <= 0 ||
      k.cx < 0 || k.cy < 0 || k.cx >= f.width || k.cy >= f.height || k.bxf_m <= 0)
    throw std::invalid_argument("invalid same-resolution mapping intrinsics");
  const std::size_t pixels = static_cast<std::size_t>(f.width)*f.height;
  if (f.rgb_bytes && (!f.rgb || f.rgb_bytes != pixels*3))
    throw std::invalid_argument("same-frame RGB buffer size mismatch");
  auto semantic=Impl::copySemantic(f);
  p.frame = f;
  p.disparity.resize(validation.layout.sample_count);
  std::memcpy(p.disparity.data(),f.disparity,f.disparity_bytes);
  p.rgb.clear();
  if (f.rgb_bytes) p.rgb.assign(f.rgb,f.rgb+f.rgb_bytes);
  p.acceptSemantic(f,std::move(semantic));
  p.raw.reset(new PointCloudT); p.camera.reset(new PointCloudT);
  p.alg.ground_cloud.reset(new PointCloudT);
  p.alg.no_ground_cloud.reset(new PointCloudT);
  p.alg.obs_cloud.reset(new PointCloudT);
  p.alg.last_semantic_cloud_.reset(new PointCloudT);
  p.output = std::make_shared<MappingResult>();
  p.output->frame_id = f.frame_id;
  p.next_stage = 1;
  // Disabling auto calibration must not discard the configured extrinsics.
  if (!p.config.run_calibration) p.alg.pclCommonPtr->EulerToMatrix(p.alg.cali,p.transform_matrix);
  p.updateCalibrationResult();
}

void MappingPipeline::setSemantic(const MappingFrame& f) {
  auto& p=*impl_;
  if (!p.output || p.next_stage<1 || p.next_stage>3)
    throw std::logic_error("mapping semantic injection requires beginFrame and must precede labels");
  if (f.frame_id!=p.frame.frame_id || f.width!=p.frame.width || f.height!=p.frame.height)
    throw std::invalid_argument("mapping semantic result frame or dimensions mismatch");
  // Validation/copy is transactional: rejection preserves any accepted result.
  auto semantic=Impl::copySemantic(f);
  p.acceptSemantic(f,std::move(semantic));
}

void MappingPipeline::cloud() {
  auto& p = *impl_;
  p.stage(1,"cloud");
  const auto& f = p.frame;
  const auto& k = f.intrinsics;
  const int step = p.config.sample_step;
  const auto rows = (static_cast<std::size_t>(f.height)+step-1U)/step;
  const auto cols = (static_cast<std::size_t>(f.width)+step-1U)/step;
  auto reserve = rows*cols;
  if (p.box.max_points > 0) reserve = std::min(reserve,p.box.max_points);
  p.raw->reserve(reserve);
  // Original dense2sparse_pointcloud numeric loop, without ROS disparity
  // publication or auxiliary file-record vectors.
  bool reached_limit = false;
  for (int row=0; row<f.height && !reached_limit; row+=step) {
    for (int column=0; column<f.width; column+=step) {
      const auto index = static_cast<std::size_t>(row)*f.disparity_stride_samples+column;
      const float disparity_pixels = static_cast<float>(p.disparity[index])/256.0F;
      if (!std::isfinite(disparity_pixels) || disparity_pixels <= 0.0F) continue;
      const float z = k.bxf_m/disparity_pixels;
      if (!std::isfinite(z) || z < 0.001F) continue;
      const float x = (static_cast<float>(column)-k.cx)*z/k.fx;
      const float y = (static_cast<float>(row)-k.cy)*z/k.fy;
      if (!std::isfinite(x) || !std::isfinite(y) ||
          z<p.box.box_min_z || z>p.box.box_max_z || x<p.box.box_min_x ||
          x>p.box.box_max_x || y<p.box.box_min_y || y>p.box.box_max_y) continue;
      p.raw->emplace_back(x,y,z,0,0,0,-1,0.0F);
      if (p.box.max_points>0 && p.raw->size()>=p.box.max_points) {
        reached_limit=true; break;
      }
    }
  }
  *p.camera = *p.raw;
  p.output->camera_points = p.raw->size();
  if (p.config.retain_clouds) copyCloud(p.camera,p.output->camera_cloud);
}

void MappingPipeline::calibration() {
  auto& p = *impl_;
  p.stage(2,"calibration");
  p.alg.pclCommonPtr->EulerToMatrix(p.alg.cali,p.transform_matrix);
  if (!p.auto_cali) return;
  PointCloudT calibration_cloud = *p.raw;
  Trans initial;
  initial.roll = angle_to_rad(-90.0); initial.yaw = angle_to_rad(-90.0);
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  p.alg.pclCommonPtr->EulerToMatrix(initial,matrix);
  pcl::transformPointCloud(calibration_cloud,calibration_cloud,matrix);
  // The original left these values uninitialized when RANSAC failed. Surface
  // that failure instead of persisting undefined angles; valid fits are exact.
  float roll=std::numeric_limits<float>::quiet_NaN(),pitch=roll,yaw=roll;
  const double h = p.alg.pclCommonPtr->calculate_by_normal(calibration_cloud,roll,pitch,yaw);
  if (!std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(h))
    throw std::runtime_error("auto calibration could not fit a ground plane");
  const float roll_cali = -rad_to_angle(pitch)-90;
  const float pitch_cali = rad_to_angle(roll);
  p.sum_roll = roll_cali+p.sum_roll;
  p.sum_pitch = pitch_cali+p.sum_pitch;
  p.sum_h = h+p.sum_h;
  ++p.cali_count;
  p.updateCalibrationResult();
  if (p.cali_count>5) {
    auto& out = p.output->calibration;
    out.completed = true;
    out.z = roundDouble(p.sum_h/p.cali_count,2);
    out.roll_degrees = roundDouble(p.sum_roll/p.cali_count,2);
    out.pitch_degrees = roundDouble(p.sum_pitch/p.cali_count,2);
    out.yaw_degrees = -90;
    auto& e = p.extrinsics_doc["extrinsics"];
    auto set = [&](const char* name,double value) {
      if (e.HasMember(name)) e[name].SetDouble(value);
      else e.AddMember(rapidjson::Value(name,p.extrinsics_doc.GetAllocator()).Move(),
                       rapidjson::Value(value).Move(),p.extrinsics_doc.GetAllocator());
    };
    set("z",out.z); set("roll",out.roll_degrees);
    set("pitch",out.pitch_degrees); set("yaw",out.yaw_degrees);
    const auto path = p.config.calibration_output_path.empty()
        ? p.config.extrinsics_path : p.config.calibration_output_path;
    if (!path.empty()) {
      rapidjson::StringBuffer buffer;
      rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
      writer.SetIndent(' ',4); p.extrinsics_doc.Accept(writer);
      std::ofstream file(path,std::ios::binary|std::ios::trunc);
      if (!file) throw std::runtime_error("cannot persist auto calibration: "+path);
      file << buffer.GetString() << '\n';
      if (!file) throw std::runtime_error("failed writing auto calibration: "+path);
    }
    p.cali_count=0; p.sum_roll=0; p.sum_pitch=0; p.sum_h=0;
  }
}

void MappingPipeline::labels() {
  auto& p = *impl_;
  p.stage(3,"labels");
  if (p.auto_cali || !p.frame.semantic_result_valid || !p.alg.dnn_cfg_.enabled) return;
  const auto& k = p.frame.intrinsics;
  if (p.alg.label_fusion_cfg_.enabled) {
    p.alg.postprocessSegMask(p.mask,p.frame.width,p.frame.height);
    p.alg.fuseSemanticLabels(p.raw,p.detections,p.mask,p.frame.width,p.frame.height,
        p.frame.width,p.frame.height,k.fx,k.fy,k.cx,k.cy);
  } else {
    p.alg.matchDetectionLabels(p.raw,p.detections,p.frame.width,p.frame.height,k.fx,k.fy,k.cx,k.cy);
  }
  p.output->semantic_points=p.alg.last_semantic_cloud_->size();
  if (p.config.retain_clouds) copyCloud(p.alg.last_semantic_cloud_,p.output->semantic_cloud);
}

void MappingPipeline::transform() {
  auto& p = *impl_;
  p.stage(4,"transform");
  if (p.auto_cali) return;
  const auto& k = p.frame.intrinsics;
  // Original postprocess colors the raw cloud only with debug.visualize.
  if (p.visualize && p.color.enabled && !p.rgb.empty())
    p.alg.pclCommonPtr->ProjectRGBFromBuffer(p.raw,p.rgb,p.frame.width,p.frame.height,k.fx,k.fy,k.cx,k.cy);
  pcl::transformPointCloud(*p.raw,*p.raw,p.transform_matrix);
  if (p.config.retain_clouds) copyCloud(p.raw,p.output->robot_cloud);
}

void MappingPipeline::ground() {
  auto& p = *impl_;
  p.stage(5,"ground");
  if (p.auto_cali) return;
  if (p.alg.patchwork_cfg_.enabled) p.alg.runGroundSegmentation(p.raw);
  p.output->ground_points=p.alg.ground_cloud->size();
  p.output->obstacle_points=p.alg.obs_cloud->size();
  if (p.config.retain_clouds) {
    copyCloud(p.alg.ground_cloud,p.output->ground_cloud);
    copyCloud(p.alg.obs_cloud,p.output->obstacle_cloud);
  }
}

void MappingPipeline::maps() {
  auto& p = *impl_;
  p.stage(6,"maps");
  if (p.auto_cali) return;
  PointCloudT::Ptr semantic;
  if (p.alg.label_fusion_cfg_.enabled && p.alg.label_fusion_cfg_.fuse_into_grid)
    semantic=p.alg.last_semantic_cloud_;
  OccMapData map;
  p.alg.aiFusionPtr->initData(p.alg.ground_cloud,p.alg.obs_cloud,semantic);
  p.alg.aiFusionPtr->integrate(map);
  const auto geometry=ai_node_percep_demo::detail::TryMakeGridGeometry(p.width,p.height,p.resolution);
  auto& out=*p.output;
  out.has_maps=true;
  out.rows=geometry->rows;out.cols=geometry->cols;out.resolution_mm=geometry->resolution_mm;
  out.width_m=p.width;out.height_m=p.height;out.resolution_m=geometry->resolution_m;
  out.pcl_map=copyMap(map.pcl_map);out.ai_map=copyMap(map.ai_map);out.fusion_map=copyMap(map.fusion_map);
  out.avg_h_cm=std::move(map.map_avg_h);out.min_h_cm=std::move(map.map_min_h);
  out.max_h_cm=std::move(map.map_max_h);out.type=std::move(map.type);
  // Preserve the original AiFusion nearest-grid computation exactly,
  // including its half_res term (not a reinterpreted cell-center formula).
  for (std::size_t i=0;i<out.type.size();++i) {
    if (out.type[i]>=2 && out.type[i]!=3) {
      const float half_res=p.resolution*0.5f;
      const int grid_y=i/out.cols,grid_x=i%out.cols;
      const float x=(grid_y+half_res)*p.resolution;
      const float y=p.width/2.0f-(grid_x+half_res)*p.resolution;
      const float distance=std::sqrt(x*x+y*y);
      if (distance<out.nearest_obstacle_distance_m) {
        out.nearest_obstacle_valid=true;out.nearest_obstacle_index=static_cast<int>(i);
        out.nearest_obstacle_x_m=x;out.nearest_obstacle_y_m=y;
        out.nearest_obstacle_distance_m=distance;
      }
    }
  }
  if (p.config.render_maps) {
    out.pcl_rgb=renderOriginalMap(map.pcl_map,p.width,p.height,p.resolution,p.alg.cali.x,p.alg.cali.y);
    out.ai_rgb=renderOriginalMap(map.ai_map,p.width,p.height,p.resolution,p.alg.cali.x,p.alg.cali.y);
    out.fusion_rgb=renderOriginalMap(map.fusion_map,p.width,p.height,p.resolution,p.alg.cali.x,p.alg.cali.y);
  }
}

std::shared_ptr<const MappingResult> MappingPipeline::result() const {
  if (!impl_->output || impl_->next_stage!=7)
    throw std::logic_error("mapping result requested before all stages completed");
  return impl_->output;
}

std::string MappingPipeline::effectiveConfigJson() const {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
  writer.StartObject();
  writer.Key("start_config");impl_->start_doc.Accept(writer);
  writer.Key("extrinsics");impl_->extrinsics_doc.Accept(writer);
  writer.Key("sample_step");writer.Int(impl_->config.sample_step);
  writer.Key("retain_clouds");writer.Bool(impl_->config.retain_clouds);
  writer.Key("render_maps");writer.Bool(impl_->config.render_maps);
  writer.Key("effective_resolution_m");writer.Double(impl_->resolution);
  writer.Key("effective_label_fusion_enabled");writer.Bool(impl_->alg.label_fusion_cfg_.enabled);
  writer.EndObject();
  return buffer.GetString();
}
}  // namespace r33
