#include "percep_main/dnn_config_json.hpp"
#include <cassert>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
extern "C" {
void mock_reset(int);int mock_peak();int mock_active();int mock_created();int mock_released();
int mock_allocations();int mock_models();void mock_fail_wait();void mock_fail_submit();
size_t mock_input(int,uint8_t*,size_t);
}
using namespace perception;
static void check_timing(const DnnInferTiming& t,bool seg) {
    assert(t.detect_pre_ms>0 && t.detect_bpu_ms>0 && t.detect_post_ms>0);
    assert(t.detect_pre_ms+t.detect_bpu_ms+t.detect_post_ms<=t.detect_ms+0.01);
    if(seg) {
        assert(t.seg_pre_ms>0 && t.seg_bpu_ms>0 && t.seg_post_ms>0);
        assert(t.seg_pre_ms+t.seg_bpu_ms+t.seg_post_ms<=t.seg_ms+0.01);
    } else assert(t.seg_ms==0 && t.seg_pre_ms==0 && t.seg_bpu_ms==0 && t.seg_post_ms==0);
}
static std::string names(int n,const std::string& prefix){std::string s;for(int i=0;i<n;++i){if(i)s+=',';s+=prefix+std::to_string(i);}return s;}
static DnnConfig config(int nc=5,int seg_nc=3){
    DnnConfig c;c.enabled=true;c.parallel_det_seg=true;c.detect.model_path="det:"+std::to_string(nc);
    c.detect.classes_num=nc;c.detect.class_names=names(nc,"configured_detection_");
    c.seg.model_path="seg:"+std::to_string(seg_nc);c.seg.num_classes=seg_nc;
    c.seg.class_names=names(seg_nc,"configured_segmentation_");return c;
}
static bool run(HorizonDnnRunner& runner,const std::vector<uint8_t>& input,bool seg=true,bool nv12=true,int stride=16){
    std::vector<DnnDetection> dets;std::vector<uint8_t> mask;DnnInferTiming timing;
    const bool ok=runner.inferNv12(input.data(),16,16,stride,input.size(),nv12,&dets,seg?&mask:nullptr,nullptr,&timing);
    if(ok){check_timing(timing,seg);assert(!dets.empty());for(const auto& d:dets){assert(d.class_id==runner.config().detect.classes_num-1);
        assert(d.class_name=="configured_detection_"+std::to_string(d.class_id));}
        if(seg){assert(mask.size()==256);for(auto id:mask)assert(id==runner.seg_num_classes()-1);}}
    return ok;
}
int main(){
    std::vector<uint8_t> input(16*16*3/2);for(size_t i=0;i<input.size();++i)input[i]=uint8_t(i*37);
    {
        auto cfg=config();HorizonDnnRunner runner;assert(runner.init(cfg));
        mock_reset(1);assert(run(runner,input));assert(mock_peak()==2&&mock_active()==0);
        assert(mock_created()==2&&mock_released()==2);
        assert(runner.config().seg.class_names==cfg.seg.class_names);
        // Reuse model Tensor and cached maps across a second frame.
        assert(run(runner,input));
        mock_reset(0);assert(run(runner,input,false));assert(mock_created()==1&&mock_peak()==1);
        // Padding cache must be invalidated by an intervening RGB inference.
        std::vector<uint8_t> rgb(16*16*3,128),mask;std::vector<DnnDetection> boxes;
        DnnInferTiming rgb_timing;
        mock_reset(1);assert(runner.infer(rgb.data(),16,16,&boxes,&mask,nullptr,&rgb_timing));assert(mock_peak()==2);
        check_timing(rgb_timing,true);
        // Reusing a timing object for detection-only clears old segmentation values.
        mock_reset(0);assert(runner.infer(rgb.data(),16,16,&boxes,nullptr,nullptr,&rgb_timing));
        check_timing(rgb_timing,false);
        mock_reset(1);assert(run(runner,input));
        // All task handles are released even on wait failure or failed submit with a handle.
        mock_reset(0);mock_fail_wait();assert(!run(runner,input));assert(mock_active()==0&&mock_created()==mock_released());
        mock_reset(0);mock_fail_submit();assert(!run(runner,input));assert(mock_active()==0&&mock_created()==mock_released());
        auto bad=cfg;bad.detect.classes_num=4;assert(!runner.init(bad));
        assert(runner.init(cfg));bad=cfg;bad.detect.class_names="wrong,count";assert(!runner.init(bad));
        assert(runner.init(cfg));bad=cfg;bad.seg.num_classes=9;assert(!runner.init(bad));
        assert(runner.init(cfg));bad=cfg;bad.seg.class_names="bad,map";assert(!runner.init(bad));
        assert(runner.init(cfg));
    }
    assert(mock_allocations()==0&&mock_models()==0);
    {
        auto cfg=config(7,9);cfg.parallel_det_seg=false;
        HorizonDnnRunner runner;assert(runner.init(cfg));mock_reset(0);assert(run(runner,input));assert(mock_peak()==1);
        // Class counts and names are loaded from JSON, not fixed at 24/3.
        const auto path=std::filesystem::temp_directory_path()/"percep_dnn_dynamic_classes_test.json";
        {std::ofstream json(path);json<<"{\"dnn\":{\"enabled\":true,\"parallel_det_seg\":true,\"detect\":{\"model_path\":\"det:7\",\"classes_num\":7,\"class_names\":\""
            <<cfg.detect.class_names<<"\"},\"seg\":{\"model_path\":\"seg:9\",\"num_classes\":9,\"class_names\":\""<<cfg.seg.class_names<<"\"}}}";}
        const auto loaded=LoadDnnConfigJson(path.string());std::filesystem::remove(path);
        assert(loaded.parallel_det_seg&&loaded.detect.classes_num==7&&loaded.seg.num_classes==9);
        assert(runner.init(loaded));mock_reset(1);assert(run(runner,input));assert(mock_peak()==2);
        assert(runner.config().detect.class_names==loaded.detect.class_names);
        assert(runner.config().seg.class_names==loaded.seg.class_names);
    }
    assert(mock_allocations()==0&&mock_models()==0);
    {
        // Distinct runner instances and raw layouts may execute at the same time.
        HorizonDnnRunner a,b;assert(a.init(config(5,3)));assert(b.init(config(7,4)));
        std::vector<uint8_t> padded(24*16*3/2,128);bool ok_a=false,ok_b=false;
        mock_reset(1);
        std::thread ta([&]{ok_a=run(a,input);});
        std::thread tb([&]{ok_b=run(b,padded,true,false,24);});
        ta.join();tb.join();assert(ok_a&&ok_b&&mock_peak()>=2&&mock_active()==0);
    }
    assert(mock_allocations()==0&&mock_models()==0);
    std::cout<<"PASS: overlapping detector/segmenter BPU tasks, JSON-defined 5/7 detection and 3/4/9 segmentation classes, serial toggle, det-only, RGB/YUV reuse, mismatched config rejection, task cleanup, independent instances.\n";
}
