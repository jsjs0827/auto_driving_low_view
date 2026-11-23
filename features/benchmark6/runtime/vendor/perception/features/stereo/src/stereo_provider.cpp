#include "perception_ros_so_demo/stereo_provider.hpp"
#include "perception_ros_so_demo/stereo_inputs.hpp"
#include "compare_util.hpp"
#include "perception/inference_task.hpp"
#ifdef R36_ONLINE_WITH_HORIZON
#include "r36_stereo/horizon_stereo.hpp"
#include "r36_stereo/nv12.hpp"
#endif
#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <vector>

namespace ros_so_demo {
struct StereoProvider::Impl {
    using Clock=std::chrono::steady_clock;
    StereoSettings settings;
    psdk_full_hooks hooks{};
    psdk_frame left{},right{};
    bool paired=false,was_called=false;
    double elapsed_ms=0;
    cv::Size model_size,output_size;
    r36compare::StereoInputAdaptation adaptation;
    std::vector<std::uint16_t> q8;
    perception::inference::StagedInvocation invocation;
#ifdef R36_ONLINE_WITH_HORIZON
    std::unique_ptr<r36stereo::HorizonStereoModel> model;
#endif
    explicit Impl(const StereoSettings& value):settings(value) {
        r36compare::validateStereoInputRequest(settings.input_mode,{settings.input_width,settings.input_height});
        if(settings.model.empty())throw std::invalid_argument("stereo.model is required for new Stereo; no fallback is permitted");
#ifdef R36_ONLINE_WITH_HORIZON
        model=std::make_unique<r36stereo::HorizonStereoModel>(settings.model,true);
        model_size=model->input_size();
        // Validate metadata assertions at startup, before accepting any source.
        (void)r36compare::stereoInputAdaptation(settings.input_mode,model_size,model_size,
            {settings.input_width,settings.input_height});
#else
        throw std::runtime_error("This demo was built without Horizon Stereo support; rebuild with R36_BUILD_HORIZON_STEREO_RUNNER=ON on X5/AArch64");
#endif
        hooks.struct_size=sizeof(hooks);hooks.abi_version=PSDK_FULL_ABI_VERSION;
        hooks.userdata=this;hooks.disparity_provider=&Impl::invoke;
    }
    static std::uint32_t invoke(void* userdata,const psdk_frame* frame,
                                psdk_disparity_q8* out,psdk_status* status) noexcept {
        const auto start=Clock::now();
        auto* self=static_cast<Impl*>(userdata);
        auto finish=[&] {
            if(self)self->elapsed_ms=std::chrono::duration<double,std::milli>(Clock::now()-start).count();
        };
        auto failure=[&](const char* detail) {
            if(out)*out={};
            if(status) {
                *status={};status->struct_size=sizeof(*status);status->abi_version=PSDK_ABI_VERSION;
                status->code=1002;
                if(frame){status->frame_id=frame->frame_id;status->timestamp_ns=frame->timestamp_ns;}
                std::snprintf(status->feature,sizeof(status->feature),"%s","ros_demo.stereo.provider");
                std::snprintf(status->task,sizeof(status->task),"%s","disparity");
                std::snprintf(status->detail,sizeof(status->detail),"%s",detail);
            }
            finish();return std::uint32_t(1002);
        };
        try {
            if(!self||!frame||!out)throw std::invalid_argument("Null Stereo disparity callback argument");
            self->was_called=true;
            if(!self->paired)throw std::logic_error("Stereo callback has no registered same-frame left/right pair");
            stereo_inputs::validateHookFrame(self->left,*frame);
#ifdef R36_ONLINE_WITH_HORIZON
            using perception::inference::Stage;
            const perception::FrameInfo identity{frame->frame_id,frame->timestamp_ns};
            self->invocation.begin("stereo",identity,
                {{self->left.frame_id,self->left.timestamp_ns},{self->right.frame_id,self->right.timestamp_ns}},2);
            cv::Mat left,right,prediction;
            self->invocation.execute(Stage::Preprocess,[&]{
                const auto convert=[](const psdk_frame& input) {
                    return r36stereo::copy_yuv420_to_nv12(input.data,int(input.width),int(input.height),
                        input.stride,std::size_t(input.bytes),input.format==PSDK_FORMAT_NV12);
                };
                // Crop preserves original Y/UV samples by row copies. I420 only
                // needs U/V interleaving; resize is a separate explicit mode.
                const auto prepared=r36compare::prepareStereoNv12(convert(self->left),convert(self->right),self->adaptation);
                left=prepared.left;right=prepared.right;
            });
            self->invocation.execute(Stage::Backend,[&]{prediction=self->model->infer_nv12(left,right);});
            self->invocation.execute(Stage::Postprocess,[&]{
                if(prediction.cols!=self->model_size.width)
                    throw std::runtime_error("Stereo disparity output width differs from actual model input width");
                if(self->output_size.width&&self->output_size!=prediction.size())
                    throw std::runtime_error("Stereo model output dimensions changed between frames");
                self->output_size=prediction.size();
                // Crop writes native predictions into the source ROI and leaves
                // other rows invalid. Source YUV, intrinsics and bxf stay intact.
                self->q8=r36compare::restoreStereoQ8(prediction,self->adaptation);
            });
#else
            throw std::logic_error("Horizon Stereo support is disabled");
#endif
            *out={};out->struct_size=sizeof(*out);out->abi_version=PSDK_FULL_ABI_VERSION;
            out->width=self->left.width;out->height=self->left.height;out->stride_bytes=self->left.width*2;
            out->data=self->q8.data();out->bytes=self->q8.size()*sizeof(std::uint16_t);
            out->frame_id=frame->frame_id;out->timestamp_ns=frame->timestamp_ns;
            if(status){*status={};status->struct_size=sizeof(*status);status->abi_version=PSDK_ABI_VERSION;}
            finish();return 0;
        } catch(const std::exception& error) {return failure(error.what());}
          catch(...) {return failure("Unknown exception in Stereo disparity callback");}
    }
};
StereoProvider::StereoProvider(const StereoSettings& settings):impl_(std::make_unique<Impl>(settings)){}
StereoProvider::~StereoProvider()=default;
void StereoProvider::setPair(const psdk_frame& left,const psdk_frame& right) {
    clearPair();
    impl_->was_called=false;impl_->elapsed_ms=0;
    impl_->invocation={};
    stereo_inputs::validatePair(left,right);
    const auto adaptation=r36compare::stereoInputAdaptation(impl_->settings.input_mode,
        {int(left.width),int(left.height)},impl_->model_size,
        {impl_->settings.input_width,impl_->settings.input_height});
    impl_->left=left;impl_->right=right;impl_->adaptation=adaptation;impl_->paired=true;
}
void StereoProvider::clearPair() {
    impl_->paired=false;impl_->left={};impl_->right={};
}
const psdk_full_hooks* StereoProvider::hooks() const{return &impl_->hooks;}
double StereoProvider::elapsedMs() const{return impl_->elapsed_ms;}
bool StereoProvider::called() const{return impl_->was_called;}
std::string StereoProvider::metadataJson() const {
    return "{\"model\":"+r36compare::quote(impl_->settings.model)+
        ",\"input_mode\":"+r36compare::quote(impl_->settings.input_mode)+
        ",\"model_input_size\":"+r36compare::sizeJson(impl_->model_size)+
        ",\"input_adaptation\":"+(impl_->adaptation.source_size.width?
            r36compare::stereoInputAdaptationJson(impl_->adaptation,impl_->output_size):"null")+
        ",\"model_contract\":"+
#ifdef R36_ONLINE_WITH_HORIZON
        impl_->model->contract_json()+
#else
        std::string("null")+
#endif
        ",\"staged_adapter\":{\"preprocess_ms\":"+std::to_string(impl_->invocation.timings()[0].elapsed_ms)+
        ",\"backend_ms\":"+std::to_string(impl_->invocation.timings()[1].elapsed_ms)+
        ",\"postprocess_ms\":"+std::to_string(impl_->invocation.timings()[2].elapsed_ms)+
        ",\"complete\":"+(impl_->invocation.complete()?"true":"false")+
        ",\"backend_scope\":\"native_model_pack_infer_wait_decode\"}"+
        ",\"input_preprocessing\":\"direct_NV12_explicit_crop_resize_or_strict; crop_Y_UV_row_memcpy; I420_interleave_only; no_BGR_round_trip; no_host_255_scaling\""
        ",\"compute_scope\":\"YUV420_to_NV12_explicit_transform_model_pack_infer_decode_restore_Q8_inside_SDK_disparity_task\"}";
}
} // namespace ros_so_demo
