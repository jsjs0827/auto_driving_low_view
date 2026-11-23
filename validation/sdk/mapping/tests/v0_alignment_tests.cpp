#include "r33/mapping.hpp"
#include "mapping_algorithms.hpp"
#include "v0_1104_oracle.generated.hpp"
#include <rapidjson/document.h>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <utility>

// These declarations use the existing public C++ backend API unchanged. No
// test hook, extra public field or altered result layout is required.
static_assert(sizeof(r33::MappingCell)==16,"Existing MappingCell wire layout changed");
static_assert(offsetof(r33::MappingCell,min_h)==0 && offsetof(r33::MappingCell,max_h)==4 &&
              offsetof(r33::MappingCell,avg_h)==8 && offsetof(r33::MappingCell,type)==12,
              "Existing public cell offsets changed");
static_assert(std::is_same<decltype(&r33::MappingPipeline::cloud),void(r33::MappingPipeline::*)()>::value,
              "Existing public cloud API changed");
static_assert(std::is_same<decltype(&r33::MappingPipeline::result),std::shared_ptr<const r33::MappingResult>(r33::MappingPipeline::*)() const>::value,
              "Existing public result API changed");
namespace {
int checks=0;
void require(bool test,const std::string& message) {
  ++checks;if(!test)throw std::runtime_error(message);
}
bool near(double a,double b,double tolerance=1e-11) {return std::abs(a-b)<=tolerance;}
void samePose(const Trans& a,const Trans& b,const char* context) {
  require(near(a.x,b.x)&&near(a.y,b.y)&&near(a.z,b.z)&&near(a.roll,b.roll)&&
          near(a.pitch,b.pitch)&&near(a.yaw,b.yaw),context);
}
void sameCloud(const PointCloudT::Ptr& a,const PointCloudT::Ptr& b,const char* context) {
  require(a->size()==b->size(),std::string(context)+": different point count");
  for(std::size_t i=0;i<a->size();++i) {
    const auto& x=(*a)[i];const auto& y=(*b)[i];
    require(x.x==y.x&&x.y==y.y&&x.z==y.z,std::string(context)+": different ordered XYZ");
  }
}
template<class ExpectedCell>
void sameMap(const std::vector<r33::MappingCell>& a,const std::vector<ExpectedCell>& b,const char* context) {
  require(a.size()==b.size(),std::string(context)+": map shape differs");
  for(std::size_t i=0;i<a.size();++i)
    require(a[i].type==b[i].type&&a[i].min_h==b[i].min_h&&a[i].max_h==b[i].max_h&&a[i].avg_h==b[i].avg_h,context);
}
const char* extrinsics=R"({"extrinsics":{"x":0.45,"y":0,"z":0.2,"roll":-90,"pitch":0,"yaw":-90}})";
r33::MappingConfig config(const std::string& start) {
  r33::MappingConfig c;c.sample_step=1;c.retain_clouds=true;c.render_maps=false;
  c.start_config_json=start;c.extrinsics_json=extrinsics;return c;
}
struct Frame {
  std::vector<std::uint16_t> q;
  std::vector<std::uint8_t> mask;
  r33::MappingFrame f;
  Frame(int w,int h):q(w*h,2560),mask(w*h,1) {
    f.frame_id=1;f.width=w;f.height=h;f.disparity=q.data();f.disparity_bytes=q.size()*2;
    f.disparity_stride_samples=w;f.intrinsics={80,80,float(w)/2,float(h)/2,10};
  }
  void semantic() {
    f.semantic_result_valid=true;f.semantic_frame_id=f.frame_id;
    f.segmentation_mask=mask.data();f.segmentation_bytes=mask.size();
    f.segmentation_width=f.width;f.segmentation_height=f.height;
  }
};
void execute(r33::MappingPipeline& p,const r33::MappingFrame& f) {
  p.beginFrame(f);p.cloud();p.calibration();p.labels();p.transform();p.ground();p.maps();
}
std::string semanticConfig(bool enabled,bool fuse) {
  std::ostringstream s;s<<R"({"auto_cali":false,"obs_height_thr":0.01,
    "map":{"MapWidth":1.6,"MapHeight":2,"MapResolution":0.02},
    "point_cloud_pub":{"max_points":0,"box_min_x":-3,"box_max_x":3,"box_min_y":-3,"box_max_y":3,"box_min_z":0.1,"box_max_z":3},
    "patchwork":{"enabled":false},"label_fusion":{"enabled":)"<<(enabled?"true":"false")
    <<R"(,"fuse_into_grid":)"<<(fuse?"true":"false")<<R"(,"passable_classes":"grass","cluster_filter":false,
      "seg_cluster_filter":false,"seg_mask_erode_px":0,"max_range_m":10,"max_height_m":2},
    "dnn":{"enabled":true,"publish_det_cloud":true,"detect":{"enabled":true,"class_names":"person"},
      "seg":{"enabled":true,"class_names":"no_grass,grass,flagstone"}}})";
  return s.str();
}
void falseFlagStillFeedsV0SemanticMap() {
  Frame frame(32,32);frame.semantic();
  r33::MappingPipeline enabled(config(semanticConfig(true,true)));
  r33::MappingPipeline falseFlag(config(semanticConfig(true,false)));
  execute(enabled,frame.f);execute(falseFlag,frame.f);
  auto a=enabled.result(),b=falseFlag.result();
  require(a->semantic_points>0&&b->semantic_points>0,"semantic fixture must generate labeled points");
  require(std::any_of(b->ai_map.begin(),b->ai_map.end(),[](const auto& c){return c.type!=0;}),
          "V0 false fuse_into_grid still passes semantic cloud to maps");
  sameMap(a->ai_map,b->ai_map,"fuse_into_grid=false changed V0 AI map");
  sameMap(a->fusion_map,b->fusion_map,"fuse_into_grid=false changed V0 fusion map");
  // With label fusion disabled, V0 can still create detection-matched semantic
  // points. The map handoff is unconditional, not gated by label_fusion.enabled.
  Frame det(32,32);det.f.semantic_result_valid=true;det.f.semantic_frame_id=1;
  det.f.detections.push_back({0,.99f,0,0,31,31});
  r33::MappingPipeline fallback(config(semanticConfig(false,false)));execute(fallback,det.f);
  require(fallback.result()->semantic_points>0,"disabled label fusion must exercise detection fallback");
  require(std::any_of(fallback.result()->ai_map.begin(),fallback.result()->ai_map.end(),[](const auto& c){return c.type!=0;}),
          "V0 detection fallback semantic cloud disappeared at map handoff");
  require(b->ground_points==0&&b->obstacle_points==0,"explicit Patchwork disable must not reuse geometry");
  require(std::all_of(b->pcl_map.begin(),b->pcl_map.end(),[](const auto& c){return c.type==0;}),
          "explicit Patchwork disable unexpectedly invents PCL cells");
}
const char* bounds=R"({"box_min_x":-3,"box_max_x":3,"box_min_y":0,"box_max_y":3,"box_min_z":0.1,"box_max_z":3})";
std::size_t cloudCount(const std::string& fields) {
  auto c=config("{\"auto_cali\":false,\"patchwork\":{\"enabled\":false},"+fields+"}");
  c.run_calibration=false;c.run_labels=false;c.run_transform=false;c.run_ground=false;c.run_maps=false;
  Frame f(4,4);f.f.intrinsics={1,1,1.5f,1.5f,10};
  r33::MappingPipeline p(c);p.beginFrame(f.f);p.cloud();return p.result()->camera_points;
}
void fovCompatibilityAndAmbiguity() {
  require(cloudCount(std::string("\"interestFov\":")+bounds)==8,"1104 interestFov not applied before transform");
  require(cloudCount(std::string("\"point_cloud_pub\":")+bounds)==8,"legacy V2 point_cloud_pub compatibility lost");
  require(cloudCount(std::string("\"interestFov\":")+bounds+",\"point_cloud_pub\":"+bounds)==8,
          "identical duplicate FOV bounds should remain compatible");
  bool rejected=false;
  try { cloudCount(std::string("\"interestFov\":")+bounds+R"(,"point_cloud_pub":{"box_min_y":-3})"); }
  catch(const std::invalid_argument&) {rejected=true;}
  require(rejected,"different interestFov and point_cloud_pub silently selected a contract");
}
void configurationDefaultsObserved() {
  auto c=config(R"({"auto_cali":false,"obs_height_thr":0.01})");
  c.extrinsics_json=R"({"extrinsics":{"x":0.45,"y":0,"z":0.27,"roll":-90,"pitch":0,"yaw":-90}})";
  r33::MappingPipeline p(c);rapidjson::Document effective;
  effective.Parse(p.effectiveConfigJson().c_str());
  require(!effective.HasParseError()&&effective.HasMember("effective_patchwork"),"effective numeric configuration is missing");
  const auto& pw=effective["effective_patchwork"];
  require(pw["enabled"].GetBool(),"actual1104 omitted Patchwork block must enable ground segmentation");
  require(near(pw["sensor_height"].GetDouble(),double(float(.27)),1e-8),"default Patchwork sensor height must use actual e.z");
  require(near(pw["th_dist"].GetDouble(),.01,1e-8)&&near(effective["effective_obs_height_thr"].GetDouble(),.01,1e-8),"actual1104 obs_height_thr=.01 must drive Patchwork and z split");
  require(near(pw["min_range"].GetDouble(),.1)&&near(pw["max_range"].GetDouble(),3)&&near(pw["th_seeds"].GetDouble(),.025),"actual1104 Patchwork default ranges/seeds differ");
  auto legacy=config(R"({"auto_cali":false,"patchwork":{"enabled":false,"th_dist":0.03}})");
  r33::MappingPipeline oldExplicit(legacy);rapidjson::Document oldEffective;
  oldEffective.Parse(oldExplicit.effectiveConfigJson().c_str());
  require(!oldEffective["effective_patchwork"]["enabled"].GetBool(),"explicit disabled Patchwork compatibility lost");
  require(near(oldEffective["effective_obs_height_thr"].GetDouble(),.03,1e-8),"legacy explicit th_dist fallback ignored");
  auto priority=config(R"({"auto_cali":false,"obs_height_thr":0.01,"patchwork":{"th_dist":0.04}})");
  r33::MappingPipeline sourcePriority(priority);rapidjson::Document priorityEffective;
  priorityEffective.Parse(sourcePriority.effectiveConfigJson().c_str());
  require(near(priorityEffective["effective_patchwork"]["th_dist"].GetDouble(),.01,1e-8),"source obs_height_thr did not take precedence over legacy th_dist");
}
void configureV0State(ai_node_percep_demo::MappingAlgorithms& a,double height=.2) {
  a.cali={};a.cali.x=double(float(.45));a.cali.z=double(float(height));a.cali.roll=-M_PI/2;a.cali.yaw=-M_PI/2;
  a.cali_default=a.cali;a.gadapt_enable=true;a.gadapt_ema_alpha=1;
  a.gadapt_max_roll_dev=10;a.gadapt_max_pitch_dev=10;a.gadapt_max_yaw_dev=5;
  a.gadapt_max_h_dev=.1;a.gadapt_max_resid_cm=4;a.gadapt_min_inliers=3000;
  a.gadapt_verbose=false;a.cali_max_range=4;a.gadapt_inited_=false;a.gadapt_ok_=a.gadapt_rej_=0;
}
PointCloudT cameraPlane(double height,double xSlope=0,double zSlope=0,int nx=90,int nz=90) {
  PointCloudT cloud;cloud.reserve(nx*nz);
  for(int iz=0;iz<nz;++iz) for(int ix=0;ix<nx;++ix) {
    const float x=-.8f+1.6f*ix/(nx-1),z=.4f+2.f*iz/(nz-1);
    const float y=float(height+xSlope*x+zSlope*z);cloud.emplace_back(x,y,z);
  }
  return cloud;
}
void adaptiveOriginalFunctionOracle() {
  ai_node_percep_demo::MappingAlgorithms actual;v0_1104_oracle::Oracle oracle;
  require(actual.gadapt_enable&&actual.gadapt_ema_alpha==1&&actual.gadapt_min_inliers==3000&&
          actual.gadapt_max_resid_cm==4&&actual.cali_max_range==4,"1104 adaptive defaults changed");
  configureV0State(actual);configureV0State(oracle);
  const auto plane=cameraPlane(.21,.015,.02);
  Trans aa,oo;double ah=0,oh=0,at=0,ot=0,ar=0,orr=0;size_t ai=0,oi=0;
  const bool okA=actual.solveExtrinsicsFromGround(plane,4,aa,ah,at,ar,ai);
  const bool okO=oracle.solveExtrinsicsFromGround(plane,4,oo,oh,ot,orr,oi);
  require(okA&&okO&&ai>=3000,"adaptive success fixture did not meet real V0 guards");
  samePose(aa,oo,"independent1104 solver pose differs");
  require(near(ah,oh)&&near(at,ot)&&near(ar,orr)&&ai==oi,"independent1104 solver quality differs");
  auto expected=oracle.adaptGroundTrans(plane),got=actual.adaptGroundTrans(plane);
  samePose(got,expected,"accepted adaptive pose differs from actual V0 function");
  require(actual.gadapt_ok_==1&&actual.gadapt_rej_==0,"valid plane was not accepted");
  require(std::abs(got.roll-actual.cali_default.roll)>1e-4,"success fixture did not alter pose");
  require(got.yaw==actual.cali_default.yaw&&got.x==actual.cali_default.x&&got.y==actual.cali_default.y,
          "1104 adaptive mode changed frozen yaw/x/y");
  // Model caller behavior: the returned pose is stored in cali. A rejection
  // returns the original static pose, not the previous accepted estimate.
  actual.cali=got;oracle.cali=expected;
  PointCloudT tooSmall;tooSmall.emplace_back(0,.2f,1.f);
  got=actual.adaptGroundTrans(tooSmall);expected=oracle.adaptGroundTrans(tooSmall);
  samePose(got,expected,"too-few-points fallback differs from actual V0 function");
  samePose(got,actual.cali_default,"failed fit did not return static cali_default");
  require(actual.gadapt_rej_==1,"failed fit was not counted");
  actual.cali=got;oracle.cali=expected;
  const auto impossibleHeight=cameraPlane(.5);
  got=actual.adaptGroundTrans(impossibleHeight);expected=oracle.adaptGroundTrans(impossibleHeight);
  samePose(got,expected,"height guard differs from actual V0 function");
  require(actual.gadapt_rej_==2,"out-of-height plane should be rejected");
  // Nondefault EMA exercises persistent state, independently through real V0
  // code. It does not claim production's default alpha1 performs low-pass.
  configureV0State(actual);configureV0State(oracle);
  actual.gadapt_ema_alpha=oracle.gadapt_ema_alpha=.25;
  for(const auto& sample:{cameraPlane(.205,.01,.01),cameraPlane(.215,.02,.025),cameraPlane(.21,-.01,.02)}) {
    expected=oracle.adaptGroundTrans(sample);got=actual.adaptGroundTrans(sample);
    samePose(got,expected,"EMA sequence differs from actual V0 function");
    require(actual.gadapt_ok_==oracle.gadapt_ok_&&actual.gadapt_rej_==oracle.gadapt_rej_,"adaptive sequence counter differs");
    actual.cali=got;oracle.cali=expected;
  }
  actual.gadapt_enable=false;oracle.gadapt_enable=false;
  samePose(actual.adaptGroundTrans(plane),oracle.adaptGroundTrans(plane),"disabled adaptive mode fallback differs");
}
void setupPatchwork(ai_node_percep_demo::MappingAlgorithms& a,double height=.2,double threshold=.01) {
  a.obs_height_thr=threshold;a.patchwork_cfg_.enabled=true;
  patchwork::Params pw;pw.verbose=false;pw.sensor_height=height;pw.min_range=.1;pw.max_range=3;
  pw.th_dist=threshold;pw.th_seeds=.025;pw.enable_RNR=false;pw.num_iter=3;pw.num_lpr=20;
  pw.num_min_pts=0;pw.th_seeds_v=.8;pw.th_dist_v=.05;pw.uprightness_thr=.101;
  pw.num_zones=1;pw.num_rings_of_interest=1;a.patchwork_.reset(new patchwork::PatchWorkpp(pw));
}
PointCloudT::Ptr groundFixture(bool largeObstacle=false) {
  PointCloudT::Ptr cloud(new PointCloudT);
  for(int ix=0;ix<80;++ix) for(int iy=0;iy<80;++iy)
    cloud->emplace_back(.3f+1.9f*ix/79.f,-.7f+1.4f*iy/79.f,0.f);
  const int side=largeObstacle?60:12;
  for(int ix=0;ix<side;++ix) for(int iy=0;iy<side;++iy)
    cloud->emplace_back(.9f+.35f*ix/(side-1),.12f+.35f*iy/(side-1),.03f);
  return cloud;
}
void groundOriginalFunctionOracle() {
  for(bool large:{false,true}) {
    ai_node_percep_demo::MappingAlgorithms actual;v0_1104_oracle::Oracle oracle;
    setupPatchwork(actual);setupPatchwork(oracle);
    auto cloud=groundFixture(large);
    actual.runGroundSegmentation(cloud);oracle.runGroundSegmentation(cloud);
    sameCloud(actual.ground_cloud,oracle.ground_cloud,"independent1104 ground points");
    sameCloud(actual.obs_cloud,oracle.obs_cloud,"independent1104 obstacle points");
    require(!actual.ground_cloud->empty(),"ground fixture lost its zero-height plane");
    require(std::any_of(actual.obs_cloud->begin(),actual.obs_cloud->end(),[](const auto& p){return std::abs(p.z-.03f)<1e-5f;}),
            "3cm obstacle fixture did not exercise retained nonground above1cm");
    require(std::none_of(actual.ground_cloud->begin(),actual.ground_cloud->end(),[](const auto& p){return std::abs(p.z-.03f)<1e-5f;}),
            "old V2 signed-plane5cm augmentation still reclassifies3cm obstacle as ground");
  }
}
void v0PatchworkDefaultsThroughPublicPipeline() {
  const int width=160,height=120;
  Frame f(width,height);
  std::fill(f.q.begin(),f.q.end(),0);
  f.f.intrinsics={120,120,80,60,10};
  for(int y=69;y<height;++y) for(int x=0;x<width;++x) {
    const float depth=.2f*120/(y-60.f);
    f.q[y*width+x]=std::uint16_t(std::round(10/depth*256));
  }
  auto c=config(R"({"auto_cali":false,"obs_height_thr":0.01,
    "map":{"MapWidth":1.6,"MapHeight":2,"MapResolution":0.02},
    "interestFov":{"box_min_x":-3,"box_max_x":3,"box_min_y":-3,"box_max_y":3,"box_min_z":0.1,"box_max_z":3},
    "dnn":{"enabled":false}})"); // no patchwork object: actual1104 constructs it.
  c.run_calibration=false;
  r33::MappingPipeline pipeline(c);pipeline.beginFrame(f.f);pipeline.cloud();pipeline.labels();
  pipeline.transform();pipeline.ground();pipeline.maps();auto result=pipeline.result();
  require(result->ground_points>0,"omitted Patchwork block disabled actual1104 ground defaults");
  // Start from the same camera points, then independently execute actual1104
  // adaptive pose + transform + full ground function before projecting maps.
  // This does not borrow the public pipeline's already-transformed points.
  v0_1104_oracle::Oracle oracle;configureV0State(oracle);
  setupPatchwork(oracle,double(float(.2)),.01);
  PointCloudT::Ptr robot(new PointCloudT);PointCloudT camera;
  for(const auto& p:result->camera_cloud)camera.emplace_back(p.x,p.y,p.z);
  auto expectedPose=oracle.adaptGroundTrans(camera);oracle.cali=expectedPose;
  Eigen::Matrix4f transform=Eigen::Matrix4f::Identity();
  oracle.pclCommonPtr->EulerToMatrix(expectedPose,transform);
  pcl::transformPointCloud(camera,*robot,transform);
  oracle.runGroundSegmentation(robot);
  auto publicCloud=[](const std::vector<r33::MappingPoint>& points) {
    PointCloudT::Ptr out(new PointCloudT);
    for(const auto& p:points)out->emplace_back(p.x,p.y,p.z);
    return out;
  };
  sameCloud(publicCloud(result->robot_cloud),robot,"public transform vs actual1104 pose oracle");
  sameCloud(publicCloud(result->ground_cloud),oracle.ground_cloud,"public ground vs actual1104 function oracle");
  sameCloud(publicCloud(result->obstacle_cloud),oracle.obs_cloud,"public obstacles vs actual1104 function oracle");
  AiFusion projection(1.6,2.0,.02,.05,5,6,double(float(.45)),0,.1f);
  projection.initData(oracle.ground_cloud,oracle.obs_cloud,{});
  OccMapData expectedMaps;projection.integrate(expectedMaps);
  sameMap(result->pcl_map,expectedMaps.pcl_map,"public PCL type/min/max/avg differ from actual1104 pose+ground oracle");
  sameMap(result->ai_map,expectedMaps.ai_map,"public AI type/min/max/avg differ in geometry-only cohort");
  sameMap(result->fusion_map,expectedMaps.fusion_map,"public fusion type/min/max/avg differ from actual1104 pose+ground oracle");
  // cloud() must expose the pose it applied even when the separate auto-cali
  // graph task is disabled, matching the ordinary1104 mapping path.
  require(near(result->calibration.z,expectedPose.z,1e-8)&&
          near(result->calibration.roll_degrees,rad_to_angle(expectedPose.roll),1e-7)&&
          near(result->calibration.pitch_degrees,rad_to_angle(expectedPose.pitch),1e-7),
          "run_calibration=false skipped ordinary adaptive pose or failed to expose applied pose");
}
}
int main() {
  try {
    fovCompatibilityAndAmbiguity();falseFlagStillFeedsV0SemanticMap();configurationDefaultsObserved();
    adaptiveOriginalFunctionOracle();groundOriginalFunctionOracle();v0PatchworkDefaultsThroughPublicPipeline();
    std::cout<<"v0_alignment_checks="<<checks<<" actual1104_function_oracle=true public_mapping_signatures_and_cell_layout_checked=true\n";
    return 0;
  } catch(const std::exception& e) {std::cerr<<"V0 ALIGNMENT FAILED: "<<e.what()<<'\n';return 1;}
}
