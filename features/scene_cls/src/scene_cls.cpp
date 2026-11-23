#include "features/scene_cls/include/scene_cls.hpp"
#include "perception/backend.hpp"
#include "perception/preprocess.hpp"
#include "perception_sdk.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <sstream>

namespace perception::scene_cls {
namespace {
using Clock=std::chrono::steady_clock;
double ms(Clock::time_point start) {return std::chrono::duration<double,std::milli>(Clock::now()-start).count();}
class SceneClassifier final : public runtime::Feature {
    algorithms::Invocation invocation_;
    std::unique_ptr<Backend> backend_;
    std::unique_ptr<PreprocessPlan> preprocess_;
    int top_k_,output_index_;
    bool softmax_;
    std::vector<std::string> class_names_;
public:
    explicit SceneClassifier(algorithms::Invocation invocation):invocation_(std::move(invocation)) {
        using namespace r33::json;
        const auto& p=invocation_.parameters; ModelConfig config;
        config.source.path=string(required(p,"model"));
        config.backend.device=string(required(p,"backend"))=="bpu"?Device::BPU:Device::CPU;
        config.backend.cpu_threads=integer(required(p,"cpu_threads"),1,32);
        config.backend.bpu_timeout_ms=integer(required(p,"bpu_timeout_ms"),1,60000);
        config.preprocess.width=integer(required(p,"input_width"),0,4096);
        config.preprocess.height=integer(required(p,"input_height"),0,4096);
        config.preprocess.letterbox=boolean(required(p,"letterbox"));
        config.preprocess.input_bgr=boolean(required(p,"input_bgr"));
        for(int i=0;i<3;++i) {
            config.preprocess.mean[i]=float(number(required(p,"mean").array.at(i)));
            config.preprocess.stddev[i]=float(number(required(p,"stddev").array.at(i)));
        }
        config.output_layout=Layout::NCHW;
        top_k_=integer(required(p,"top_k"),1,1024);
        output_index_=integer(required(p,"output_index"),0,63);
        softmax_=boolean(required(p,"softmax"));
        for(const auto& name:required(p,"class_names").array)class_names_.push_back(string(name));
        backend_=makeBackend(config);
        const auto shape=backend_->inputSize();
        config.preprocess.width=shape[0];config.preprocess.height=shape[1];
        preprocess_=std::make_unique<PreprocessPlan>(config.preprocess,backend_->acceptsNv12());
    }
    ~SceneClassifier() override {if(backend_)try{backend_->close();}catch(...) {}}
    void execute(runtime::FeatureContext& context) override {
        const auto source=context.get<psdk_frame>(invocation_.inputs.at("image"));
        if(source->format!=PSDK_FORMAT_NV12&&source->format!=PSDK_FORMAT_I420)
            throw std::invalid_argument("scene_cls expects NV12 or I420 input");
        const ImageView image{source->data,std::size_t(source->bytes),source->stride,
            int(source->width),int(source->height),source->format==PSDK_FORMAT_NV12?PixelFormat::NV12:PixelFormat::I420};
        const auto start=Clock::now();const auto prepared=preprocess_->execute(image);const auto pre_ms=ms(start);
        const auto infer=Clock::now();auto outputs=prepared.nv12?backend_->runNv12(prepared.bytes):backend_->run(prepared.bytes);
        const auto infer_ms=ms(infer);const auto post=Clock::now();
        if(std::size_t(output_index_)>=outputs.size())throw std::invalid_argument("scene_cls output_index is missing");
        const auto& logits=outputs.at(output_index_);logits.validate();
        if(logits.height!=1||logits.width!=1)throw std::invalid_argument("scene_cls needs [1,C] or [1,C,1,1] classification logits");
        if(top_k_>logits.channels)throw std::invalid_argument("scene_cls top_k exceeds model classes");
        if(!class_names_.empty()&&class_names_.size()!=std::size_t(logits.channels))throw std::invalid_argument("scene_cls class_names must match model classes");
        std::vector<double> scores(logits.data.begin(),logits.data.end());
        for(const auto score:scores)if(!std::isfinite(score))throw std::invalid_argument("Nonfinite classification score");
        if(softmax_) {
            const auto maximum=*std::max_element(scores.begin(),scores.end());double total=0;
            for(auto& score:scores){score=std::exp(score-maximum);total+=score;}
            for(auto& score:scores)score/=total;
        } else {
            double total=0;for(const auto score:scores){if(score<0||score>1)throw std::invalid_argument("scene_cls probability is outside [0,1]");total+=score;}
            if(std::abs(total-1)>0.001)throw std::invalid_argument("scene_cls probabilities do not sum to one");
        }
        std::vector<int> order(scores.size());std::iota(order.begin(),order.end(),0);
        std::partial_sort(order.begin(),order.begin()+top_k_,order.end(),[&](int a,int b){return scores[a]==scores[b]?a<b:scores[a]>scores[b];});
        std::ostringstream result;result<<std::setprecision(12)<<"{\"algorithm\":\"scene_cls\",\"frame_id\":"<<source->frame_id
            <<",\"timestamp_ns\":"<<source->timestamp_ns<<",\"top_k\":[";
        for(int i=0;i<top_k_;++i){if(i)result<<',';const auto c=order[i];result<<"{\"class_id\":"<<c<<",\"score\":"<<scores[c];
            if(!class_names_.empty())result<<",\"label\":"<<r33::json::quote(class_names_[c]);result<<'}';}
        result<<"],\"timing_ms\":{\"preprocess\":"<<pre_ms<<",\"inference\":"<<infer_ms<<",\"postprocess\":"<<ms(post)<<"}}";
        context.set(algorithms::resultKey(context.taskId()),algorithms::Output{result.str()});
    }
};
}
std::unique_ptr<runtime::Feature> create(const algorithms::Invocation& invocation) {
    return std::make_unique<SceneClassifier>(invocation);
}
}
