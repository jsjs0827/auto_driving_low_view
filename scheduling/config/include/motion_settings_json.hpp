#pragma once
#include "json.hpp"
#include "visual_motion.hpp"
#include "motion_reference.hpp"
#include "slip_detection.hpp"
#include <type_traits>

namespace r33::full {
inline std::string motionJson(const json::JsonValue& v) {
  using K=json::JsonValue::Kind;
  if(v.kind==K::Null)return "null";
  if(v.kind==K::Boolean)return v.boolean?"true":"false";
  if(v.kind==K::Number)return v.text;
  if(v.kind==K::String)return json::quote(v.text);
  std::string out=v.kind==K::Array?"[":"{";bool first=true;
  if(v.kind==K::Array)for(const auto& item:v.array){if(!first)out+=',';first=false;out+=motionJson(item);}
  else for(const auto& item:v.object){if(!first)out+=',';first=false;out+=json::quote(item.first)+':'+motionJson(item.second);}
  return out+(v.kind==K::Array?"]":"}");
}
inline perception::motion::VisualSettings visualSettingsFromJson(const json::JsonValue& v) {
  using namespace json;perception::motion::VisualSettings s;
  fields(v,{"roi","patch_radius","search_radius","grid_step","max_features","min_matches","min_inliers","ransac_iterations",
    "min_texture","max_patch_error","match_ratio","min_disparity_px","max_depth_m","inlier_threshold_m","max_residual_m",
    "min_inlier_ratio","min_dt_ms","max_gap_ms","max_translation_m","max_rotation_rad","extrinsics"});
  const auto count=[&](const char* key,auto& target){if(auto p=optional(v,key))target=static_cast<std::decay_t<decltype(target)>>(integer(*p,0,1000000));};
  const auto real=[&](const char* key,double& target){if(auto p=optional(v,key))target=number(*p);};
  count("patch_radius",s.patch_radius);count("search_radius",s.search_radius);count("grid_step",s.grid_step);
  count("max_features",s.max_features);count("min_matches",s.min_matches);count("min_inliers",s.min_inliers);count("ransac_iterations",s.ransac_iterations);
  real("min_texture",s.min_texture);real("max_patch_error",s.max_patch_error);real("match_ratio",s.match_ratio);
  real("min_disparity_px",s.min_disparity_px);real("max_depth_m",s.max_depth_m);real("inlier_threshold_m",s.inlier_threshold_m);
  real("max_residual_m",s.max_residual_m);real("min_inlier_ratio",s.min_inlier_ratio);real("min_dt_ms",s.min_dt_ms);
  real("max_gap_ms",s.max_gap_ms);real("max_translation_m",s.max_translation_m);real("max_rotation_rad",s.max_rotation_rad);
  if(auto roi=optional(v,"roi")){fields(*roi,{"x","y","width","height"});
    const auto value=[&](const char* k){auto p=optional(*roi,k);return p?integer(*p,0,8192):0;};
    s.roi={value("x"),value("y"),value("width"),value("height")};}
  if(auto e=optional(v,"extrinsics")){fields(*e,{"rotation","translation_m","confirmed"});
    if(auto p=optional(*e,"confirmed"))s.extrinsics.confirmed=boolean(*p);
    const auto array=[&](const char* key,auto& target){if(auto p=optional(*e,key)){
      if(p->kind!=JsonValue::Kind::Array||p->array.size()!=target.size())throw std::invalid_argument(std::string("extrinsics.")+key+" has incorrect array length");
      for(std::size_t i=0;i<target.size();++i)target[i]=number(p->array[i]);}};
    array("rotation",s.extrinsics.rotation);array("translation_m",s.extrinsics.translation_m);}
  perception::motion::VisualMotion validate(s);return s;
}
inline perception::motion::ReferenceSettings referenceSettingsFromJson(const json::JsonValue& v) {
  using namespace json;perception::motion::ReferenceSettings s;
  fields(v,{"max_samples","max_age_ms","max_sample_gap_ms","max_abs_speed_mps","max_abs_yaw_rate_rps","input_csv","odom_topic","clock_domain","same_clock_confirmed","wait_timeout_ms"});
  if(auto p=optional(v,"max_samples"))s.max_samples=std::size_t(integer(*p,2,1000000));
  const auto real=[&](const char* key,double& target){if(auto p=optional(v,key))target=number(*p);};
  real("max_age_ms",s.max_age_ms);real("max_sample_gap_ms",s.max_sample_gap_ms);real("max_abs_speed_mps",s.max_abs_speed_mps);real("max_abs_yaw_rate_rps",s.max_abs_yaw_rate_rps);
  if(auto p=optional(v,"input_csv"))if(p->kind!=JsonValue::Kind::Null)(void)string(*p);
  if(auto p=optional(v,"odom_topic"))if(p->kind!=JsonValue::Kind::String||p->text.find('\0')!=std::string::npos)throw std::invalid_argument("motion_reference.odom_topic must be a string");
  if(auto p=optional(v,"clock_domain"))if(p->kind!=JsonValue::Kind::String||p->text.find('\0')!=std::string::npos)throw std::invalid_argument("motion_reference.clock_domain must be a string");
  if(auto p=optional(v,"same_clock_confirmed"))(void)boolean(*p);
  if(auto p=optional(v,"wait_timeout_ms"))(void)integer(*p,0,2000);
  perception::motion::MotionBuffer validate(s);return s;
}
inline perception::motion::SlipSettings slipSettingsFromJson(const json::JsonValue& v) {
  using namespace json;perception::motion::SlipSettings s;
  fields(v,{"min_speed_mps","min_reference_distance_m","max_yaw_rad","max_yaw_rate_rps","min_visual_quality","enter_ratio","exit_ratio","enter_duration_ms","exit_duration_ms","max_gap_ms"});
  const auto real=[&](const char* key,double& target){if(auto p=optional(v,key))target=number(*p);};
  real("min_speed_mps",s.min_speed_mps);real("min_reference_distance_m",s.min_reference_distance_m);real("max_yaw_rad",s.max_yaw_rad);
  real("max_yaw_rate_rps",s.max_yaw_rate_rps);real("min_visual_quality",s.min_visual_quality);real("enter_ratio",s.enter_ratio);real("exit_ratio",s.exit_ratio);
  real("enter_duration_ms",s.enter_duration_ms);real("exit_duration_ms",s.exit_duration_ms);real("max_gap_ms",s.max_gap_ms);
  perception::motion::SlipDetector validate(s);return s;
}
} // namespace r33::full
