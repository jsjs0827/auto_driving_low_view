#include "features/raw_lux/include/raw_lux.hpp"
#include "../backend/include/percep_main/raw_lux_runner.hpp"
#include "perception_full_sdk.h"
#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace perception::raw_lux_feature {
namespace {
using r33::json::quote;
void paired(std::uint64_t a_id,std::uint64_t a_time,std::uint64_t b_id,std::uint64_t b_time,const char* what) {
    if(!a_id||!b_id||!a_time||!b_time||a_id!=b_id||a_time!=b_time)
        throw std::invalid_argument(std::string(what)+" requires equal nonzero capture frame IDs and timestamps");
}
std::uint64_t distance(std::uint64_t a,std::uint64_t b){return a>=b?a-b:b-a;}
std::string signedSkew(std::uint64_t a,std::uint64_t b){return a>=b?std::to_string(a-b):"-"+std::to_string(b-a);}
struct CrossStreamPolicy {
    std::string mode="exact_timestamps";
    std::uint64_t maximum=0,period=0;
    void check(std::uint64_t a_id,std::uint64_t a_time,std::uint64_t b_id,std::uint64_t b_time,const char* what)const {
        if(!a_id||!b_id||!a_time||!b_time||a_id!=b_id)
            throw std::invalid_argument(std::string(what)+" requires equal nonzero frame IDs and nonzero original timestamps");
        if(mode=="exact_timestamps") {if(a_time!=b_time)throw std::invalid_argument(std::string(what)+" requires exact timestamps under the default policy");}
        else if(distance(a_time,b_time)>maximum)
            throw std::invalid_argument(std::string(what)+" exceeds explicit cross-stream timestamp skew bound");
    }
    std::string diagnostic(const psdk_frame& current,const psdk_raw_stereo_input* raw)const {
        std::ostringstream out;out<<"{\"policy\":"<<quote(mode)<<",\"association\":"<<quote(mode=="sdk_frame_id_bounded"?"bounded_unverified":"exact_identity")
            <<",\"max_cross_stream_skew_ns\":"<<maximum<<",\"capture_frame_period_ns\":"<<period
            <<",\"cross_stream_sync_verified\":false,\"original_timestamps_preserved\":true"
            <<",\"provenance\":"<<quote(mode=="sdk_frame_id_bounded"?"SDK public frame ID plus bounded original timestamps; SDK counter grouping is not physical exposure proof":"Exact public IDs and timestamps; RAW exposure metadata remains an ORG proxy")
            <<",\"yuv\":{\"frame_id\":"<<current.frame_id<<",\"timestamp_ns\":"<<current.timestamp_ns<<'}';
        const auto identity=[&](const char* name,std::uint64_t id,std::uint64_t stamp,std::uint32_t sensor,std::uint32_t stream){
            out<<','<<quote(name)<<":{\"frame_id\":"<<id<<",\"timestamp_ns\":"<<stamp<<",\"sensor_id\":"<<sensor<<",\"stream_type\":"<<stream<<'}';};
        if(raw){
            identity("raw_left",raw->left.frame_id,raw->left.timestamp_ns,raw->left.sensor_id,raw->left.stream_type);
            identity("raw_right",raw->right.frame_id,raw->right.timestamp_ns,raw->right.sensor_id,raw->right.stream_type);
            identity("org_left",raw->left_org.frame_id,raw->left_org.timestamp_ns,raw->left_org.sensor_id,raw->left_org.stream_type);
            identity("org_right",raw->right_org.frame_id,raw->right_org.timestamp_ns,raw->right_org.sensor_id,raw->right_org.stream_type);
            out<<",\"observed_skew_ns\":{\"raw_minus_yuv\":"<<signedSkew(raw->left.timestamp_ns,current.timestamp_ns)
                <<",\"raw_minus_org_left\":"<<signedSkew(raw->left.timestamp_ns,raw->left_org.timestamp_ns)
                <<",\"raw_minus_org_right\":"<<signedSkew(raw->right.timestamp_ns,raw->right_org.timestamp_ns)
                <<",\"org_minus_yuv\":"<<signedSkew(raw->left_org.timestamp_ns,current.timestamp_ns)<<'}';
        }
        out<<'}';return out.str();
    }
};
RawLuxImageView image(const psdk_raw_image& raw,const psdk_org_exposure& org,const CrossStreamPolicy& sync) {
    if(raw.struct_size!=sizeof(raw)||raw.abi_version!=PSDK_RAW_INPUT_ABI_VERSION||!raw.data||raw.bytes!=2785280u||
       raw.width!=1088||raw.height!=1280||raw.stride_bytes!=2176||raw.format!=PSDK_RAW_FORMAT_UINT16_LE||
       raw.valid_bits!=10||raw.bayer!=PSDK_RAW_BAYER_BGGR)
        throw std::invalid_argument("RAW lux needs native BGGR1088x1280 uint16LE DN10 stride2176; no resize, rotation or bit shifting");
    if(org.struct_size!=sizeof(org)||org.abi_version!=PSDK_RAW_INPUT_ABI_VERSION||org.present!=1||org.reserved||
       !std::isfinite(org.exposure_ms)||org.exposure_ms<=0||!std::isfinite(org.analog_gain)||org.analog_gain<=0)
        throw std::invalid_argument("RAW lux needs each eye's positive finite ORG exposure_ms and linear analog_gain");
    sync.check(raw.frame_id,raw.timestamp_ns,org.frame_id,org.timestamp_ns,"RAW/ORG metadata");
    const double us=org.exposure_ms*1000.0;
    if(!std::isfinite(us))throw std::invalid_argument("ORG milliseconds-to-microseconds overflow");
    RawLuxImageView v;v.data=raw.data;v.bytes=static_cast<std::size_t>(raw.bytes);
    v.width=1088;v.height=1280;v.stride_bytes=2176;v.format=RawLuxPixelFormat::UInt16LE;
    v.frame_id=raw.frame_id;v.timestamp_ns=raw.timestamp_ns;v.exposure_us=us;v.analog_gain=org.analog_gain;return v;
}
class RawLightFeature final:public runtime::Feature {
    algorithms::Invocation invocation_;
    RawLuxRunner runner_;
    bool include_features_=true;
    CrossStreamPolicy sync_;
    void publishInvalid(runtime::FeatureContext& context,const psdk_frame& frame,const char* status,const std::string& reason,const psdk_raw_stereo_input* raw=nullptr)const {
        std::ostringstream out;out<<"{\"algorithm\":\"raw_lux\",\"valid\":false,\"status\":"<<quote(status)
            <<",\"frame_id\":"<<frame.frame_id<<",\"timestamp_ns\":"<<frame.timestamp_ns
            <<",\"metadata_source\":\"org_proxy\",\"reason\":"<<quote(reason)
            <<",\"capture_association\":"<<sync_.diagnostic(frame,raw)
            <<",\"timing_ms\":{\"feature\":0,\"classification\":0,\"total\":0}}";
        context.set(algorithms::resultKey(context.taskId()),algorithms::Output{out.str()});
    }
public:
    explicit RawLightFeature(algorithms::Invocation invocation):invocation_(std::move(invocation)) {
        using namespace r33::json;
        RawLuxConfig config;config.model_name=string(required(invocation_.parameters,"model_name"));
        config.metadata_source=string(required(invocation_.parameters,"metadata_source"));config.max_session_pairs=1;
        include_features_=boolean(required(invocation_.parameters,"include_features"));
        if(const auto p=optional(invocation_.parameters,"cross_stream_sync"))sync_.mode=string(*p);
        if(const auto p=optional(invocation_.parameters,"max_cross_stream_skew_ns"))sync_.maximum=integer(*p,0,20000000);
        if(const auto p=optional(invocation_.parameters,"capture_frame_period_ns"))sync_.period=integer(*p,0,1000000000);
        if(sync_.mode!="exact_timestamps"&&sync_.mode!="sdk_frame_id_bounded")throw std::invalid_argument("Unknown RAW cross-stream synchronization policy");
        if(sync_.mode=="exact_timestamps"&&sync_.maximum)throw std::invalid_argument("Exact RAW synchronization must not specify a nonzero skew tolerance");
        if(sync_.mode=="sdk_frame_id_bounded"&&(!sync_.maximum||!sync_.period||sync_.maximum>=sync_.period))
            throw std::invalid_argument("Bounded RAW association needs explicit positive skew <=20ms and a larger actual capture frame period");
        if(config.model_name!="raw_hist_org_logistic"||config.metadata_source!="org_proxy")
            throw std::invalid_argument("DAG raw_lux requires raw_hist_org_logistic with explicit org_proxy; no automatic fallback");
        if(!runner_.init(config))throw std::runtime_error("RAW lux init: "+runner_.lastError());
    }
    void execute(runtime::FeatureContext& context)override {
        const auto current=context.get<psdk_frame>("input.left");
        const auto key=invocation_.inputs.at("raw");
        if(!context.contains(key)){publishInvalid(context,*current,"missing_input","RAW/ORG input artifact is absent");return;}
        RawLuxResult result;
        std::optional<psdk_raw_stereo_input> observed;
        try {
            const auto packet=context.get<psdk_raw_stereo_input>(key);
            observed=*packet;
            if(packet->struct_size!=sizeof(*packet)||packet->abi_version!=PSDK_RAW_INPUT_ABI_VERSION||packet->reserved||
               packet->metadata_source!=PSDK_RAW_METADATA_ORG_PROXY)
                throw std::invalid_argument("RAW input requires valid ABI and ORG_PROXY metadata source");
            if(!packet->left.data||!packet->right.data){publishInvalid(context,*current,"missing_input","One or both RAW eyes are absent",&*observed);return;}
            if(packet->left.data==packet->right.data)throw std::invalid_argument("RAW left and right must use separate image buffers");
            paired(packet->left.frame_id,packet->left.timestamp_ns,packet->right.frame_id,packet->right.timestamp_ns,"RAW stereo pair");
            paired(packet->left_org.frame_id,packet->left_org.timestamp_ns,packet->right_org.frame_id,packet->right_org.timestamp_ns,"ORG stereo pair");
            sync_.check(packet->left.frame_id,packet->left.timestamp_ns,current->frame_id,current->timestamp_ns,"RAW/current DAG input");
            sync_.check(packet->left_org.frame_id,packet->left_org.timestamp_ns,current->frame_id,current->timestamp_ns,"ORG/current DAG input");
            const auto left=image(packet->left,packet->left_org,sync_),right=image(packet->right,packet->right_org,sync_);
            if(!runner_.predictPair(std::to_string(current->frame_id),left,right,&result))
                throw std::invalid_argument(runner_.lastError());
        }catch(const std::exception& error){publishInvalid(context,*current,"invalid_input",error.what(),observed?&*observed:nullptr);return;}
        std::ostringstream out;out<<std::setprecision(17)
            <<"{\"algorithm\":\"raw_lux\",\"valid\":true,\"status\":\"ok\",\"frame_id\":"<<current->frame_id
            <<",\"timestamp_ns\":"<<current->timestamp_ns<<",\"predicted_class\":"<<quote(result.predicted_class)
            <<",\"class_index\":"<<result.class_index<<",\"metadata_source\":"<<quote(result.metadata_source)
            <<",\"model_name\":"<<quote(result.model_name)<<",\"model_source_sha256\":"<<quote(result.model_source_sha256)
            <<",\"probabilities\":{";
        for(std::size_t i=0;i<result.class_names.size();++i){if(i)out<<',';out<<quote(result.class_names[i])<<':'<<result.probabilities[i];}
        out<<"},\"probabilities_calibrated\":false,\"single_sample_accuracy_unvalidated\":true,\"stereo_same_exposure_verified\":false"
            <<",\"metadata_source_transfer_unvalidated\":"<<(result.metadata_source_transfer_unvalidated?"true":"false")
            <<",\"warning\":"<<quote(result.warning)<<",\"capture_association\":"<<sync_.diagnostic(*current,&*observed)<<",\"outside_training\":[";
        for(std::size_t i=0;i<result.outside_training.size();++i){if(i)out<<',';out<<quote(result.outside_training[i]);}
        out<<']';
        if(include_features_){out<<",\"features\":{";for(std::size_t i=0;i<result.features.size();++i){if(i)out<<',';out<<quote(result.feature_names.at(i))<<':'<<result.features[i];}out<<'}';}
        out<<",\"timing_ms\":{\"feature\":"<<result.timing.feature_ms<<",\"classification\":"<<result.timing.classification_ms
            <<",\"total\":"<<result.timing.total_ms<<",\"lock_wait\":"<<result.timing.lock_wait_ms<<"}}";
        context.set(algorithms::resultKey(context.taskId()),algorithms::Output{out.str()});
    }
};
}
std::unique_ptr<runtime::Feature> create(const algorithms::Invocation& invocation){return std::make_unique<RawLightFeature>(invocation);}
}
