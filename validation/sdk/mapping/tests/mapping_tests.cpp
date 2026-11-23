#include "r33/mapping.hpp"
#include "mapping_algorithms.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace r33 {
MappingRgbImage renderOriginalMap(const std::vector<MapPoint>&,double,double,double,double,double);
}
namespace {
void require(bool test,const char* message) {
  if (!test) throw std::runtime_error(message);
}
void compareMap(const std::vector<r33::MappingCell>& result,const std::vector<MapPoint>& oracle) {
  require(result.size()==oracle.size(),"map size differs from original AiFusion");
  for (std::size_t i=0;i<result.size();++i) {
    require(result[i].type==oracle[i].type,"map label differs from original AiFusion");
    require(result[i].min_h==oracle[i].min_h && result[i].max_h==oracle[i].max_h &&
            result[i].avg_h==oracle[i].avg_h,"map height differs from original AiFusion");
  }
}
r33::MappingConfig config(bool ground=false) {
  r33::MappingConfig c;
  c.sample_step=1;c.retain_clouds=true;c.render_maps=false;
  c.start_config_json=std::string(R"({
    "auto_cali":false,"map":{"MapWidth":1.6,"MapHeight":2.0,"MapResolution":0.02},
    "point_cloud_pub":{"max_points":0,"box_min_x":-3,"box_max_x":3,
      "box_min_y":-3,"box_max_y":3,"box_min_z":0.1,"box_max_z":3},
    "patchwork":{"enabled":)")+(ground?"true":"false")+R"(,"sensor_height":0.2,
      "min_range":0.1,"max_range":3,"th_dist":0.05,"th_seeds":0.025},
    "label_fusion":{"enabled":true,"fuse_into_grid":true,"passable_classes":"grass"},
    "dnn":{"enabled":true,"publish_det_cloud":false,"detect":{"enabled":false},
      "seg":{"enabled":true,"class_names":"no_grass,grass,flagstone"}}
  })";
  c.extrinsics_json=R"({"extrinsics":{"x":0,"y":0,"z":0.2,"roll":-90,"pitch":0,"yaw":-90}})";
  return c;
}
void execute(r33::MappingPipeline& pipeline,const r33::MappingFrame& f) {
  pipeline.beginFrame(f);pipeline.cloud();pipeline.calibration();pipeline.labels();
  pipeline.transform();pipeline.ground();pipeline.maps();
}
void projectionAndAiFusionOracle() {
  const int width=32,height=32,stride=40;
  std::vector<std::uint16_t> disparity(stride*height,65535);
  std::vector<std::uint8_t> mask(width*height,1);
  for (int y=0;y<height;++y) for (int x=0;x<width;++x)
    disparity[y*stride+x]=2560-32*y;  // first row depth 1 m, then increasing
  r33::MappingFrame f;
  f.frame_id=41;f.width=width;f.height=height;f.disparity=disparity.data();
  f.disparity_bytes=disparity.size()*2;f.disparity_stride_samples=stride;
  f.intrinsics={80,80,16,16,10};f.semantic_result_valid=true;f.semantic_frame_id=41;
  f.segmentation_mask=mask.data();f.segmentation_bytes=mask.size();
  f.segmentation_width=width;f.segmentation_height=height;
  r33::MappingPipeline pipeline(config());execute(pipeline,f);
  const auto result=pipeline.result();
  require(result->camera_points==width*height,"padding pixels entered original projection");
  require(result->camera_cloud[0].x==-0.2f && result->camera_cloud[0].y==-0.2f &&
          result->camera_cloud[0].z==1.f,"Q8 disparity projection differs from analytical pinhole");
  require(result->semantic_cloud.size()==result->camera_cloud.size(),"semantic cloud lost original point correspondence");
  require(result->rows==100 && result->cols==80 && result->resolution_mm==20,"original grid metadata differs");
  PointCloudT::Ptr ai(new PointCloudT),empty(new PointCloudT);
  for (const auto& p:result->semantic_cloud)
    ai->emplace_back(p.x,p.y,p.z,p.r,p.g,p.b,p.label,p.confidence);
  // AiFusion is compiled unchanged from the original header. This is a direct
  // oracle for the bridge's semantic-grid policy, axes, and output conversion.
  AiFusion original(1.6,2.0,0.02,0.05,5,6,0,0,0.1f);
  original.setLabelMap(ai_node_percep_demo::detail::BuildAiGridLabelMap(
      {"person","cat","dog","charger"},{"no_grass","grass","flagstone"},{},{1},false));
  original.initData(empty,empty,ai);
  OccMapData expected;original.integrate(expected);
  compareMap(result->pcl_map,expected.pcl_map);compareMap(result->ai_map,expected.ai_map);
  compareMap(result->fusion_map,expected.fusion_map);
  require(result->avg_h_cm==expected.map_avg_h && result->min_h_cm==expected.map_min_h &&
          result->max_h_cm==expected.map_max_h && result->type==expected.type,"published original centimetre arrays differ");
  // The next frame has no DNN result: previous semantic points must disappear.
  f.frame_id=42;f.semantic_result_valid=false;f.segmentation_mask=nullptr;
  f.segmentation_bytes=0;execute(pipeline,f);
  const auto next=pipeline.result();
  require(next->semantic_cloud.empty(),"stale semantic cloud crossed frame boundary");
  for (const auto& c:next->ai_map) require(c.type==0,"stale AI map crossed frame boundary");
  require(result->frame_id==41 && !result->semantic_cloud.empty(),"returned snapshot changed on next frame");
  bool rejected=false;f.semantic_result_valid=true;f.semantic_frame_id=41;
  try {pipeline.beginFrame(f);} catch (const std::invalid_argument&) {rejected=true;}
  require(rejected,"cross-frame semantic result accepted");
}
void patchworkGroundOracle() {
  const int width=96,height=80;
  std::vector<std::uint16_t> disparity(width*height);
  for (int y=41;y<height;++y) for (int x=0;x<width;++x) {
    const float depth=0.2f*80.f/(y-40.f);
    disparity[y*width+x]=static_cast<std::uint16_t>(std::round(10.f/depth*256.f));
  }
  r33::MappingFrame f;
  f.frame_id=99;f.width=width;f.height=height;f.disparity=disparity.data();
  f.disparity_bytes=disparity.size()*2;f.disparity_stride_samples=width;
  f.intrinsics={80,80,48,40,10};
  r33::MappingPipeline pipeline(config(true));execute(pipeline,f);
  const auto result=pipeline.result();
  require(result->camera_points>500,"ground fixture did not generate a usable cloud");
  require(result->ground_points>0,"Patchwork stage produced no ground for the plane fixture");
  ai_node_percep_demo::MappingAlgorithms original;
  original.patchwork_cfg_.enabled=true;
  patchwork::Params pw;
  pw.verbose=false;pw.sensor_height=0.2;pw.min_range=0.1;pw.max_range=3;
  pw.th_dist=0.05;pw.th_seeds=0.025;pw.enable_RNR=false;
  pw.num_iter=3;pw.num_lpr=20;pw.num_min_pts=0;pw.th_seeds_v=0.8;
  pw.th_dist_v=0.05;pw.uprightness_thr=0.101;pw.num_zones=1;pw.num_rings_of_interest=1;
  original.patchwork_.reset(new patchwork::PatchWorkpp(pw));
  PointCloudT::Ptr robot(new PointCloudT);
  for (const auto& p:result->robot_cloud) robot->emplace_back(p.x,p.y,p.z);
  original.runGroundSegmentation(robot);
  require(result->ground_points==original.ground_cloud->size() &&
          result->obstacle_points==original.obs_cloud->size(),"bridge differs from original Patchwork+PCL postfilters");
  AiFusion fusion(1.6,2.0,0.02,0.05,5,6,0,0,0.1f);
  fusion.initData(original.ground_cloud,original.obs_cloud,{});
  OccMapData expected;fusion.integrate(expected);
  compareMap(result->pcl_map,expected.pcl_map);compareMap(result->fusion_map,expected.fusion_map);
}
void renderOrientationAndPalette() {
  std::vector<MapPoint> map(16*20);
  map[3*16+2].type=2; // forward row 3 becomes image row 16
  map[15*16+12].type=3;
  const auto image=r33::renderOriginalMap(map,1.6,2,0.1,0.45,0);
  require(image.width==640 && image.height==800,"original display scaling changed");
  auto pixel=[&](int x,int y,int c) {return image.pixels[(y*image.width+x)*3+c];};
  require(pixel(2*40+20,16*40+20,0)==255 && pixel(2*40+20,16*40+20,1)==0,
          "original obstacle red or row reversal changed");
  require(pixel(12*40+20,4*40+20,0)==0 && pixel(12*40+20,4*40+20,1)==255,
          "original grass green changed");
}
void partialGraphCloud() {
  auto settings=config();settings.run_calibration=false;settings.run_labels=false;
  settings.run_transform=false;settings.run_ground=false;settings.run_maps=false;
  const int width=16,height=16;std::vector<std::uint16_t> q(width*height,2560);
  r33::MappingFrame frame;frame.frame_id=77;frame.width=width;frame.height=height;
  frame.disparity=q.data();frame.disparity_bytes=q.size()*2;frame.disparity_stride_samples=width;
  frame.intrinsics={80,80,8,8,10};
  r33::MappingPipeline camera(settings);camera.beginFrame(frame);camera.cloud();
  const auto cloud=camera.result();
  require(cloud->camera_cloud.size()==q.size()&&!cloud->has_maps,"Cloud-only graph did not expose real camera points");
  require(cloud->robot_cloud.empty()&&cloud->ground_cloud.empty()&&cloud->semantic_cloud.empty(),"Disabled mapping features produced data");
  settings.run_transform=true;r33::MappingPipeline robot(settings);
  robot.beginFrame(frame);robot.cloud();robot.transform();const auto transformed=robot.result();
  require(transformed->robot_cloud.size()==q.size()&&!transformed->has_maps,"Transform-only graph failed");
  require(std::abs(transformed->robot_cloud[0].z-cloud->camera_cloud[0].z)>0.1,"Disabling auto-calibration discarded configured extrinsics");
}
void compareCloud(const std::vector<r33::MappingPoint>& a,const std::vector<r33::MappingPoint>& b) {
  require(a.size()==b.size(),"late semantic injection changed cloud size");
  for (std::size_t i=0;i<a.size();++i) {
    const auto& x=a[i];const auto& y=b[i];
    require(x.x==y.x && x.y==y.y && x.z==y.z && x.label==y.label &&
            x.confidence==y.confidence && x.r==y.r && x.g==y.g && x.b==y.b,
            "late semantic injection differs from beginFrame semantic result");
  }
}
void lateSemanticInjection() {
  const int width=32,height=32;
  std::vector<std::uint16_t> q(width*height,2560);
  std::vector<std::uint8_t> mask(width*height,1);
  r33::MappingFrame f;f.frame_id=301;f.width=width;f.height=height;
  f.disparity=q.data();f.disparity_bytes=q.size()*2;f.disparity_stride_samples=width;
  f.intrinsics={80,80,16,16,10};f.semantic_result_valid=true;f.semantic_frame_id=301;
  f.segmentation_mask=mask.data();f.segmentation_bytes=mask.size();
  f.segmentation_width=width;f.segmentation_height=height;
  r33::MappingPipeline reference(config());execute(reference,f);
  r33::MappingPipeline late(config());
  bool rejected=false;
  try {late.setSemantic(f);} catch (const std::logic_error&) {rejected=true;}
  require(rejected,"semantic injection accepted before beginFrame");
  auto geometry=f;geometry.semantic_result_valid=false;
  geometry.segmentation_mask=nullptr;geometry.segmentation_bytes=0;
  // These geometry stages must finish before an AI result is available.
  // result() intentionally remains unavailable until the selected chain joins;
  // partialGraphCloud separately verifies a configured cloud-only snapshot.
  late.beginFrame(geometry);late.cloud();late.calibration();
  auto semantic=f;
  // A separate semantic carrier has no disparity or RGB buffer.
  semantic.disparity=nullptr;semantic.disparity_bytes=0;semantic.rgb=nullptr;semantic.rgb_bytes=0;
  semantic.frame_id=300;rejected=false;
  try {late.setSemantic(semantic);} catch (const std::invalid_argument&) {rejected=true;}
  require(rejected,"late cross-frame semantic result accepted");
  semantic.frame_id=301;semantic.width=31;rejected=false;
  try {late.setSemantic(semantic);} catch (const std::invalid_argument&) {rejected=true;}
  require(rejected,"late semantic result with wrong image dimensions accepted");
  semantic.width=width;late.setSemantic(semantic);
  auto malformed=semantic;malformed.segmentation_bytes-=1;rejected=false;
  try {late.setSemantic(malformed);} catch (const std::invalid_argument&) {rejected=true;}
  require(rejected,"late semantic result with truncated mask accepted");
  // Accepted results own their mask. A later rejected injection must preserve it.
  std::fill(mask.begin(),mask.end(),0);
  late.labels();late.transform();late.ground();late.maps();
  const auto expected=reference.result(),actual=late.result();
  compareCloud(expected->camera_cloud,actual->camera_cloud);
  compareCloud(expected->semantic_cloud,actual->semantic_cloud);
  compareCloud(expected->robot_cloud,actual->robot_cloud);
  require(expected->type==actual->type && expected->avg_h_cm==actual->avg_h_cm,
          "late semantic injection changed the fusion map");
  rejected=false;
  try {late.setSemantic(semantic);} catch (const std::logic_error&) {rejected=true;}
  require(rejected,"semantic injection accepted after labels executed");
  geometry.frame_id=302;execute(late,geometry);
  require(late.result()->semantic_cloud.empty(),"late semantic result leaked into next frame");
}
void lateDetectionOwnership() {
  auto settings=config();
  settings.start_config_json=R"({
    "auto_cali":false,"patchwork":{"enabled":false},
    "point_cloud_pub":{"box_min_x":-3,"box_max_x":3,"box_min_y":-3,"box_max_y":3,"box_min_z":0.1,"box_max_z":3},
    "label_fusion":{"enabled":false},
    "dnn":{"enabled":true,"publish_det_cloud":true,"det_max_depth_m":3,"detect":{"enabled":true}}
  })";
  const int width=16,height=16;std::vector<std::uint16_t> q(width*height,2560);
  r33::MappingFrame f;f.frame_id=401;f.width=width;f.height=height;
  f.disparity=q.data();f.disparity_bytes=q.size()*2;f.disparity_stride_samples=width;
  f.intrinsics={80,80,8,8,10};f.semantic_result_valid=true;f.semantic_frame_id=401;
  f.detections.push_back({0,0.9f,0,0,15,15});
  r33::MappingPipeline reference(settings);execute(reference,f);
  r33::MappingPipeline late(settings);auto geometry=f;
  geometry.semantic_result_valid=false;geometry.detections.clear();
  late.beginFrame(geometry);late.cloud();late.calibration();late.setSemantic(f);
  auto invalid=f;invalid.detections[0].confidence=std::numeric_limits<float>::quiet_NaN();
  bool rejected=false;
  try {late.setSemantic(invalid);} catch (const std::invalid_argument&) {rejected=true;}
  require(rejected,"non-finite late detection accepted");
  f.detections[0]={1,0.1f,0,0,0,0};f.detections.clear();
  late.labels();late.transform();late.ground();late.maps();
  compareCloud(reference.result()->semantic_cloud,late.result()->semantic_cloud);
  require(!late.result()->semantic_cloud.empty() && late.result()->semantic_cloud[0].label!=-1,
          "detection ownership fixture did not label any point");
}
}
int main() {
  try {
    projectionAndAiFusionOracle();patchworkGroundOracle();renderOrientationAndPalette();partialGraphCloud();
    lateSemanticInjection();lateDetectionOwnership();
    std::cout<<"original mapping projection, Patchwork/PCL, AiFusion, frame isolation, render oracle passed\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr<<e.what()<<'\n';return 1;
  }
}
