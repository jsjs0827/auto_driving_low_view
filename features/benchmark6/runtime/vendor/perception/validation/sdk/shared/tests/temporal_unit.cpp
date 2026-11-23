#include "perception_temporal.hpp"
#include "full_graph.hpp"
#include "config.hpp"
#include <iostream>
#include <stdexcept>
using namespace r36::temporal;
void need(bool v,const char*s){if(!v)throw std::runtime_error(s);}
int main(){try{
    Settings s;s.capacity=2;s.frame_interval=2;s.difference_threshold=1;s.match=true;
    History h(s);std::vector<std::uint8_t> data(6,1);
    psdk_full_frame f{};f.left={sizeof(psdk_frame),1,PSDK_FORMAT_NV12,2,2,2,data.data(),data.size(),0,0};
    std::shared_ptr<const Frame> held;
    for(unsigned i=1;i<=5;++i){f.left.frame_id=i;f.left.timestamp_ns=i*100;data[0]=std::uint8_t(i);auto p=h.prepare(f);
        need(p.window.sampled==(i%2==1),"fixed interval sampling");h.complete(p,nullptr,{});h.commit(p);
        if(i==1)held=p.window.frames.back();if(i==5)need(p.window.frames.size()==2&&p.window.frames[0]->left.metadata.frame_id==3,"bounded oldest eviction");}
    data[0]=200;need(held->left.view().data[0]==1,"source ownership after reuse/eviction");
    auto dup=h.prepare(f);need(!dup.window.sampled&&!dup.new_capture,"duplicate capture cannot advance history");
    f.left.frame_id=6;f.left.timestamp_ns=600;auto p6=h.prepare(f);h.commit(p6);
    f.left.frame_id=7;f.left.timestamp_ns=700;auto failed=h.prepare(f);auto retry=h.prepare(f);
    need(failed.window.sampled&&retry.window.sampled&&failed.unique_count==retry.unique_count,"uncommitted failure rollback");h.commit(retry);
    f.left.frame_id=1;f.left.timestamp_ns=50;auto reset=h.prepare(f);need(reset.window.frames.size()==1&&reset.window.reset_reason=="timestamp_discontinuity","camera restart resets window");h.commit(reset);
    f.left.frame_id=0;f.left.timestamp_ns=100;auto wrap=h.prepare(f);need(wrap.window.reset_reason.empty(),"frame ID wrap with monotonic time");h.commit(wrap);
    f.left.timestamp_ns=150;f.intrinsics.fx=1;auto calibration=h.prepare(f);need(calibration.window.reset_reason=="calibration_changed"&&calibration.window.frames.size()==1,"calibration reset");
    h.clear();f.intrinsics={};f.left.frame_id=10;f.left.timestamp_ns=1000;data={1,2,3,4,128,128};
    auto before=h.prepare(f);psdk_detection det{};det.class_id=2;det.x2=10;det.y2=10;
    std::uint8_t mask1[]={0,1,2,1};psdk_result_view ai{};ai.width=2;ai.height=2;ai.frame_id=10;ai.timestamp_ns=1000;ai.detections=&det;ai.detection_count=1;ai.segmentation_mask=mask1;ai.segmentation_bytes=4;
    h.complete(before,&ai,{});h.commit(before);
    f.left.frame_id=11;f.left.timestamp_ns=1100;auto gap=h.prepare(f);h.commit(gap);
    f.left.frame_id=12;f.left.timestamp_ns=1200;data={2,4,6,8,128,128};auto after=h.prepare(f);ai.frame_id=12;ai.timestamp_ns=1200;mask1[0]=1;det.x1=1;det.x2=11;
    h.complete(after,&ai,{});Analysis a;difference(after.window,s,a);match(after.window,s,a);
    need(a.difference_valid&&a.mean_abs_luma==2.5&&a.changed_luma==3&&a.mask_pixels==4&&a.changed_mask==1,"known luma/mask differences");
    need(a.matching_valid&&a.matches.size()==1&&a.matches[0].class_id==2&&a.matches[0].iou>.81f,"known same-class one-to-one IoU match");
    s.match_iou_threshold=.9;Analysis strict;match(after.window,s,strict);need(strict.matches.empty(),"matching threshold");
    Settings bounded;bounded.capacity=4;bounded.max_memory_mb=1;History small(bounded);std::vector<std::uint8_t> large(640*544*3/2);std::vector<std::uint8_t> mask(640*544);
    f.left={sizeof(psdk_frame),1,PSDK_FORMAT_NV12,640,544,640,large.data(),large.size(),1,100};
    ai={};ai.width=640;ai.height=544;ai.frame_id=1;ai.timestamp_ns=100;ai.segmentation_mask=mask.data();ai.segmentation_bytes=mask.size();
    auto one=small.prepare(f);small.complete(one,&ai,{});small.commit(one);f.left.frame_id=2;f.left.timestamp_ns=200;ai.frame_id=2;ai.timestamp_ns=200;
    auto two=small.prepare(f);small.complete(two,&ai,{});need(two.window.frames.size()==1&&two.window.memory_limited&&two.window.payload_bytes<=1024*1024,"payload budget eviction");
    f.left.height=1088;f.left.width=1280;f.left.stride=1280;bool refused=false;try{small.prepare(f);}catch(const std::invalid_argument&){refused=true;}need(refused,"oversized single capture rejected");
    Settings padded_settings;padded_settings.capacity=2;padded_settings.difference_threshold=0;History padded(padded_settings);
    std::vector<std::uint8_t> pad1={1,2,200,200,3,4,200,200,128,128,200,200},pad2={2,3,0,0,4,5,0,0,128,128,0,0};
    f.left={sizeof(psdk_frame),1,PSDK_FORMAT_NV12,2,2,4,pad1.data(),pad1.size(),21,2100};
    auto pad_before=padded.prepare(f);padded.complete(pad_before,nullptr,{});padded.commit(pad_before);
    f.left.data=pad2.data();f.left.frame_id=22;f.left.timestamp_ns=2200;auto pad_after=padded.prepare(f);
    auto q8=std::make_shared<const std::vector<std::uint16_t>>(std::initializer_list<std::uint16_t>{256,512,1024,2048});
    padded.complete(pad_after,nullptr,q8);Analysis pad_analysis;difference(pad_after.window,padded_settings,pad_analysis);
    need(pad_analysis.mean_abs_luma==1&&pad_analysis.changed_luma==4,"Y stride respected, padding ignored in difference");
    auto q8_view=pad_after.window.frames.back()->view();need(q8_view.has_disparity&&q8_view.disparity.data[3]==2048&&q8_view.disparity.frame_id==22,"owned Q8 identity and values");
    psdk_frame wrong=f.left;wrong.frame_id=999;f.recharge_hires=&wrong;bool mismatch=false;try{padded.prepare(f);}catch(const std::invalid_argument&){mismatch=true;}
    need(mismatch,"same-capture GDC mismatch rejected");f.recharge_hires=nullptr;
    Settings stress_settings;stress_settings.capacity=4;History stress(stress_settings);std::weak_ptr<const Frame> expired;
    for(unsigned i=1;i<=1000;++i){f.left.frame_id=i;f.left.timestamp_ns=i*100;auto capture=stress.prepare(f);stress.complete(capture,nullptr,{});
        if(i==1)expired=capture.window.frames.back();stress.commit(capture);need(capture.window.frames.size()<=4&&capture.window.payload_bytes<=48,"bounded stress payload");}
    need(expired.expired(),"evicted frame memory released when no result retains it");
    r33::full::Graph graph;r33::full::Graph::ContextBindings work;unsigned seen=0;
    auto original=std::make_shared<const Window>(after.window);
    work["dnn"]=[&](perception::runtime::FeatureContext&c){need(c.get<Window>(input_artifact_key)->frames.size()==2,"DAG producer sees owned input window");};
    work["history"]=[&](perception::runtime::FeatureContext&c){auto w=*c.get<Window>(input_artifact_key);w.current_id=999;c.set(artifact_key,std::move(w));};
    work["temporal_extension"]=[&](perception::runtime::FeatureContext&c){seen=unsigned(c.get<Window>(artifact_key)->current_id);};
    perception::runtime::GraphReport report;r28::Status status;
    need(graph.runContext(work,{12,1200},report,status,original)==0&&seen==999,"DAG transactional history reaches downstream Feature");
    need(original->current_id==12,"initial immutable window remains unchanged");
    std::cout<<"Temporal unit passed: sampling, eviction, rollback, restart/wrap, calibration, owned bytes, difference, IoU, budget and actual DAG artifact transactions.\n";return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
