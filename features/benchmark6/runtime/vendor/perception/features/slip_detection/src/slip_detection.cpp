#include "slip_detection.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace perception::motion {
SlipDetector::SlipDetector(SlipSettings s):settings_(s) {
    const double values[]={s.min_speed_mps,s.min_reference_distance_m,s.max_yaw_rad,s.max_yaw_rate_rps,s.min_visual_quality,
        s.enter_ratio,s.exit_ratio,s.enter_duration_ms,s.exit_duration_ms,s.max_gap_ms};
    for(double v:values)if(!std::isfinite(v)||v<0)throw std::invalid_argument("Invalid slip settings");
    if(s.min_speed_mps<=0||s.min_reference_distance_m<=0||s.max_yaw_rad<=0||s.max_yaw_rate_rps<=0||
       s.min_visual_quality>1||s.exit_ratio>=s.enter_ratio||s.enter_ratio>=1||
       s.enter_duration_ms<=0||s.exit_duration_ms<=0||s.max_gap_ms<=0)
        throw std::invalid_argument("Invalid slip thresholds/hysteresis");
}
void SlipDetector::reset()noexcept {slipping_=false;last_end_ns_=0;enter_ms_=exit_ms_=0;}
SlipResult SlipDetector::update(const VisualResult&v,const ReferenceResult&r) {
    SlipResult out;out.t0_ns=v.t0_ns;out.t1_ns=v.t1_ns;
    const auto unavailable=[&](const char* reason){reset();out.reason=reason;return out;};
    if(!v.available)return unavailable("visual_motion_unavailable");
    if(!v.body_confirmed)return unavailable("camera_body_extrinsics_unconfirmed");
    if(!r.available)return unavailable("motion_reference_unavailable");
    if(!v.t0_ns||v.t1_ns<=v.t0_ns||r.t0_ns!=v.t0_ns||r.t1_ns!=v.t1_ns)return unavailable("motion_intervals_differ");
    const double dt=double(v.t1_ns-v.t0_ns)*1e-9;
    const double values[]={v.forward_m,v.yaw_rad,v.quality,r.forward_m,r.yaw_rad,dt};
    for(double x:values)if(!std::isfinite(x))return unavailable("nonfinite_motion");
    if(dt*1000>settings_.max_gap_ms)return unavailable("motion_time_gap");
    if(v.quality<settings_.min_visual_quality)return unavailable("visual_quality_below_threshold");
    if(std::abs(v.yaw_rad)>settings_.max_yaw_rad||std::abs(r.yaw_rad)>settings_.max_yaw_rad||
       std::abs(v.yaw_rad)/dt>settings_.max_yaw_rate_rps||std::abs(r.yaw_rad)/dt>settings_.max_yaw_rate_rps)
        return unavailable("turn_too_large");
    out.reference_speed_mps=r.forward_m/dt;out.visual_speed_mps=v.forward_m/dt;
    if(std::abs(out.reference_speed_mps)<settings_.min_speed_mps||std::abs(r.forward_m)<settings_.min_reference_distance_m)
        return unavailable("reference_speed_too_low");
    // Forward/reverse use the reference travel direction. Opposite visual travel
    // can produce > 1; negative values mean vision moved farther than the wheels.
    out.slip_ratio=1.-v.forward_m/r.forward_m;
    if(last_end_ns_&&v.t1_ns<=last_end_ns_)return unavailable("motion_timestamp_not_increasing");
    if(last_end_ns_&&v.t0_ns!=last_end_ns_){reset();out.reason="window_discontinuity";}
    last_end_ns_=v.t1_ns;
    if(!slipping_){
        exit_ms_=0;
        if(out.slip_ratio>=settings_.enter_ratio){enter_ms_+=dt*1000;
            if(enter_ms_>=settings_.enter_duration_ms){slipping_=true;enter_ms_=0;out.state=SlipState::Slipping;}
            else out.state=SlipState::Suspected;
        }else{enter_ms_=0;out.state=SlipState::Normal;}
    }else{
        enter_ms_=0;
        if(out.slip_ratio<=settings_.exit_ratio){exit_ms_+=dt*1000;
            if(exit_ms_>=settings_.exit_duration_ms){slipping_=false;exit_ms_=0;out.state=SlipState::Normal;}
            else out.state=SlipState::Recovering;
        }else{exit_ms_=0;out.state=SlipState::Slipping;}
    }
    out.enter_elapsed_ms=enter_ms_;out.exit_elapsed_ms=exit_ms_;out.available=true;
    if(out.reason=="not_evaluated")out.reason="ok";return out;
}
}
