#include "motion_reference.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
namespace perception::motion {
MotionBuffer::MotionBuffer(ReferenceSettings s):settings_(s) {
    if(s.max_samples<2||s.max_samples>1000000||!std::isfinite(s.max_age_ms)||s.max_age_ms<=0||
       !std::isfinite(s.max_sample_gap_ms)||s.max_sample_gap_ms<=0||
       !std::isfinite(s.max_abs_speed_mps)||s.max_abs_speed_mps<=0||
       !std::isfinite(s.max_abs_yaw_rate_rps)||s.max_abs_yaw_rate_rps<=0)
        throw std::invalid_argument("Invalid motion reference settings");
}
bool MotionBuffer::push(MotionSample s,std::string* reason) {
    const auto reject=[&](const char* r){if(reason)*reason=r;return false;};
    // Break interpolation continuity on corrupt observations. Dropping only the
    // bad value would silently draw a valid-looking line across invalid input.
    if(!s.timestamp_ns||!std::isfinite(s.forward_mps)||!std::isfinite(s.yaw_rate_rps)){samples_.clear();return reject("invalid_reference_sample");}
    if(std::abs(s.forward_mps)>settings_.max_abs_speed_mps||std::abs(s.yaw_rate_rps)>settings_.max_abs_yaw_rate_rps){samples_.clear();return reject("reference_sample_out_of_range");}
    if(!samples_.empty()&&s.timestamp_ns<=samples_.back().timestamp_ns)return reject("reference_timestamp_not_increasing");
    samples_.push_back(s);
    while(samples_.size()>settings_.max_samples||
          (samples_.size()>1&&double(samples_.back().timestamp_ns-samples_.front().timestamp_ns)/1e6>settings_.max_age_ms))samples_.pop_front();
    if(reason)reason->clear();return true;
}
ReferenceResult MotionBuffer::integrate(std::uint64_t t0,std::uint64_t t1)const {
    ReferenceResult out;out.t0_ns=t0;out.t1_ns=t1;
    const auto fail=[&](const char* r){out.reason=r;return out;};
    if(!t0||t1<=t0)return fail("invalid_reference_interval");
    out.dt_s=double(t1-t0)*1e-9;
    if(samples_.size()<2||samples_.front().timestamp_ns>t0||samples_.back().timestamp_ns<t1)return fail("reference_does_not_cover_interval");
    std::vector<MotionSample> knots;
    const auto interpolate=[](const MotionSample&a,const MotionSample&b,std::uint64_t t){
        const double w=double(t-a.timestamp_ns)/double(b.timestamp_ns-a.timestamp_ns);
        return MotionSample{t,a.forward_mps+(b.forward_mps-a.forward_mps)*w,a.yaw_rate_rps+(b.yaw_rate_rps-a.yaw_rate_rps)*w};
    };
    for(std::size_t i=1;i<samples_.size();++i){const auto&a=samples_[i-1];const auto&b=samples_[i];
        if(b.timestamp_ns<=t0||a.timestamp_ns>=t1)continue;
        if(double(b.timestamp_ns-a.timestamp_ns)/1e6>settings_.max_sample_gap_ms)return fail("reference_sample_gap");
        const auto begin=std::max(a.timestamp_ns,t0),end=std::min(b.timestamp_ns,t1);
        if(knots.empty())knots.push_back(interpolate(a,b,begin));
        knots.push_back(interpolate(a,b,end));
    }
    if(knots.size()<2||knots.front().timestamp_ns!=t0||knots.back().timestamp_ns!=t1)return fail("reference_does_not_cover_interval");
    out.samples_used=static_cast<unsigned>(knots.size());
    // Piecewise-linear velocity interpolation, integrated in <= 5 ms SE(2) steps.
    // The signed path distance and yaw use the exact trapezoidal integral.
    for(std::size_t i=1;i<knots.size();++i){const auto&a=knots[i-1];const auto&b=knots[i];
        const double dt=double(b.timestamp_ns-a.timestamp_ns)*1e-9;
        out.distance_m+=(a.forward_mps+b.forward_mps)*.5*dt;
        if(!std::isfinite(dt)||dt>60)return fail("reference_integration_interval_too_large");
        const unsigned steps=std::max(1U,static_cast<unsigned>(std::ceil(dt/.005)));
        const double h=dt/steps;
        for(unsigned j=0;j<steps;++j){const double u=(j+.5)/steps;
            const double v=a.forward_mps+(b.forward_mps-a.forward_mps)*u;
            const double w=a.yaw_rate_rps+(b.yaw_rate_rps-a.yaw_rate_rps)*u;
            const double turn=w*h,heading=out.yaw_rad+turn*.5;
            const double sinc=std::abs(turn)<1e-8?1.:std::sin(turn*.5)/(turn*.5);
            out.forward_m+=v*h*sinc*std::cos(heading);out.lateral_m+=v*h*sinc*std::sin(heading);
            out.yaw_rad+=turn;
        }
    }
    out.available=true;out.reason="ok";return out;
}
}
