#include "perception/inference_task.hpp"
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition,const char* detail) {if(!condition)throw std::runtime_error(detail);}
template<class Function> void rejects(Function&& function) {
    bool failed=false;try{function();}catch(const std::exception&){failed=true;}
    require(failed,"Invalid staged inference operation was accepted");
}
}
int main() {try {
    using namespace perception::inference;
    const perception::FrameInfo frame{17,1001};
    for(const auto& algorithm:{"det","seg","stereo"}) {
        StagedInvocation task;
        const auto count=std::string(algorithm)=="stereo"?2U:1U;
        task.begin(algorithm,frame,std::vector<perception::FrameInfo>(count,frame),count);
        int calls=0;
        rejects([&]{task.execute(Stage::Backend,[&]{++calls;});});
        task.execute(Stage::Preprocess,[&]{++calls;});
        task.execute(Stage::Backend,[&]{++calls;});
        task.execute(Stage::Postprocess,[&]{++calls;});
        require(task.complete()&&calls==3,"A model adapter skipped or duplicated an execution stage");
        for(const auto& timing:task.timings())require(timing.succeeded&&timing.elapsed_ms>=0,"Stage timing was not retained");
        rejects([&]{task.execute(Stage::Postprocess,[]{});});
    }
    StagedInvocation task;
    rejects([&]{task.begin("stereo",frame,{frame},2);});
    rejects([&]{task.begin("stereo",frame,{frame,{18,1001}},2);});
    rejects([&]{task.begin("stereo",frame,{frame,{17,1002}},2);});
    task.begin("det",frame,{frame},1);
    rejects([&]{task.execute(Stage::Preprocess,[]{throw std::runtime_error("model preprocessing rejected");});});
    rejects([&]{task.execute(Stage::Backend,[]{});});
    require(!task.complete()&&!task.timings()[0].succeeded,"Failed stage was exposed as a completed inference");
    task.begin("det",{18,1002},{{18,1002}},1);
    task.execute(Stage::Preprocess,[]{});task.execute(Stage::Backend,[]{});task.execute(Stage::Postprocess,[]{});
    require(task.complete()&&task.frame().id==18,"A new frame could not reset its invocation state");
    std::cout<<"Shared det/seg/stereo staged invocation contracts passed (no model/hardware inference).\n";
    return 0;
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
