#include "product/product_dag.hpp"
#include "product/disparity_projection.hpp"
#include "product/status_api.hpp"
#include <cstring>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <iostream>
#include <stdexcept>

void check(bool value,const char* detail) {if(!value) throw std::runtime_error(detail);}
int main(int argc,char** argv) {
    using namespace r28::product;
    ProductDag dag(argc>1?argv[1]:"");
    check(dag.registeredFeatures().size()==11,"real product feature inventory");
    perception::runtime::GraphReport report; r28::Status status;
    std::vector<std::string> events;
    ProductDag::Bindings bindings;
    for (const auto& name : {"dnn","rgb","cloud","labels","transform","ground","maps","publish","record"})
        bindings[name]=[&,name]{events.push_back(name);};
    perception::FrameInfo frame; frame.id=42; frame.timestamp_ns=123456789;
    check(dag.run(bindings,frame,report,status)==0,"map graph succeeds");
    check(events==std::vector<std::string>({"dnn","rgb","cloud","labels","transform","ground","maps","publish","record"}),"product original order");
    events.clear();
    bindings["ground"]=[&]{events.push_back("ground"); r28::Status error;
        error.code=1011;error.native_code=-71;error.detail="vendor failure";throw r28::Failure(error);};
    check(dag.run(bindings,frame,report,status)==1011,"business code returned unchanged");
    check(status.native_code==-71&&status.frame.id==42&&status.task=="ground"&&status.feature=="product.pointcloud.patchwork","failure context survives scheduler");
    check(events==std::vector<std::string>({"dnn","rgb","cloud","labels","transform","ground"}),"no maps, publication, recording after failure");
    setLastStatus(status);
    std::size_t required=0;
    check(perception_product_last_status(nullptr,0,&required)==1011,"C boundary preserves uint32 code");
    std::vector<char> json(required);
    check(perception_product_last_status(json.data(),json.size(),nullptr)==1011&&std::string(json.data()).find("-71")!=std::string::npos,"full JSON status boundary");
    ProductDag::Bindings recharge;
    events.clear();recharge["dnn"]=[&]{events.push_back("det");};recharge["recharge"]=[&]{events.push_back("mark");};
    frame.id=43;
    check(dag.run(recharge,frame,report,status)==0&&status.code==0,"next mode clears old failure");
    check(events==std::vector<std::string>({"det","mark"}),"recharge only skips map chain");
    ProductDag::Bindings invalid;
    check(dag.run(invalid,frame,report,status)==2001,"empty bindings rejected");
    invalid["typo"]=[]{throw std::runtime_error("must not execute");};
    check(dag.run(invalid,frame,report,status)==2001,"unknown binding rejected");
    invalid.clear();invalid["dnn"]={};
    check(dag.run(invalid,frame,report,status)==2001,"empty implementation rejected");
    check(dag.run(recharge,frame,report,status)==0,"invalid bindings do not leak frame closure");
    ProductDag::Bindings reduced;
    bool published=false;
    reduced["cloud"]=[]{throw std::runtime_error("cloud fails without optional labels/ground");};
    reduced["publish"]=[&]{published=true;};
    check(dag.run(reduced,frame,report,status)==1002&&!published,"disabled tasks retain transitive failure dependencies");
    ProductDag::Bindings calibration;events.clear();
    calibration["rgb"]=[&]{events.push_back("rgb");};calibration["cloud"]=[&]{events.push_back("cloud");};
    calibration["calibration"]=[&]{events.push_back("calibration");};
    check(dag.run(calibration,frame,report,status)==0&&events.size()==3,"calibration has no map publication");
    const auto temp=std::filesystem::temp_directory_path()/
        ("product_dag_test_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(temp);
    {
        ProductDag measured(argc>1?argv[1]:"",temp.string());
        check(measured.run(calibration,frame,report,status)==0,"measured graph succeeds");
        r28::Status async;async.code=2002;async.frame=frame;async.feature="product.output.record_io";
        measured.recordExternalStatus(async,measured.currentRunId());
    }
    std::ifstream metric_file(temp/"product_metrics.csv"),status_file(temp/"product_status.jsonl");
    std::string metrics((std::istreambuf_iterator<char>(metric_file)),{}),statuses((std::istreambuf_iterator<char>(status_file)),{});
    check(metrics.find("run_id,frame_id,timestamp_ns,")==0&&metrics.find(",calibration,succeeded,")!=std::string::npos,"per-task metrics emitted");
    check(statuses.find("\"run_id\":1")!=std::string::npos&&statuses.find("2002")!=std::string::npos,"async status carries run correlation");
    metric_file.close();status_file.close();
    std::filesystem::remove_all(temp);

    // Exhaustively compare all Q8.8 raw words at representative image/FOV
    // coordinates to the frozen scalar expressions from the original product.
    for (const auto& uv : {std::pair<int,int>{0,0},{639,543},{312,278}}) {
        for(std::uint32_t value=0;value<=65535;++value) {
            const auto raw=static_cast<std::uint16_t>(value);
            const float fx=359.3753967285f,fy=359.4878234865f,cx=312.170349121f,cy=278.223236084f;
            const double bxf=17.3211;
            float disp=static_cast<float>(raw)/256.0;
            float z=static_cast<float>(bxf)/disp;
            bool valid=std::isfinite(z)&&z>=0.001f;
            float x=0,y=0;
            if(valid) {x=(static_cast<float>(uv.first)-cx)*z/fx;y=(static_cast<float>(uv.second)-cy)*z/fy;valid=std::isfinite(x)&&std::isfinite(y);}
            ProjectedDisparity out;
            check(projectOriginalDisparity(raw,uv.first,uv.second,fx,fy,cx,cy,bxf,out)==valid,"raw disparity validity equivalence");
            if(valid) check(std::memcmp(&x,&out.x,sizeof(float))==0&&std::memcmp(&y,&out.y,sizeof(float))==0&&std::memcmp(&z,&out.z,sizeof(float))==0&&std::memcmp(&disp,&out.disparity,sizeof(float))==0,"bitwise original cloud scalar equivalence");
        }
    }
    std::cout<<"product DAG ordering, failure skip, modes, full status, Q8.8 exhaustive scalar equivalence passed\n";
}
