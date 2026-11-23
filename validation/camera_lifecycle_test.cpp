#include "perception_ros_so_demo/mo_camera.hpp"
#include <dlfcn.h>
#include <iostream>
#include <stdexcept>
namespace raw=ros_so_demo::raw;
void expect(bool value,const char* why) { if (!value) throw std::runtime_error(why); }
int main(int argc,char** argv) {
    expect(argc==2,"Pass TEST camera fixture path");
    void* library=dlopen(argv[1],RTLD_NOW);expect(library,"Fixture did not load");
    auto reset=reinterpret_cast<void(*)(int)>(dlsym(library,"fixture_reset"));
    auto count=reinterpret_cast<int(*)(int)>(dlsym(library,"fixture_count"));expect(reset&&count,"Fixture controls missing");
    raw::CameraOptions opts;opts.sdk=argv[1];opts.config="test-fixture.cfg";opts.need_q8=true;opts.need_gdc=true;opts.timeout_ms=100;
    reset(0);
    { raw::Camera camera(opts);auto frame=camera.next([]{return true;});
      expect(frame.left.bytes[0]==17 && frame.disparity.bytes[0]==1 && frame.disparity.bytes[1]==2 && frame.gdc.bytes[0]==23,"SDK release invalidated owned output");
      expect(count(1)==1 && count(2)==1,"Successful planes not released exactly once");
      camera.close();camera.close();expect(count(3)==1 && count(4)==1,"Close not idempotent or mode not restored"); }
    reset(1);
    { raw::Camera camera(opts);bool failed=false;try{camera.next([]{return true;});}catch(const std::runtime_error&){failed=true;}
      expect(failed && count(1)==1,"Rejected metadata leaked or was accepted"); }
    reset(2);opts.need_q8=false;opts.need_gdc=false;
    { raw::Camera camera(opts);auto frame=camera.next([]{return true;});
      expect(frame.left.bytes[0]==17 && count(0)==2 && count(1)==2 && count(2)==0,"Partial RGBD retry/release failed in infer mode"); }
    reset(3);
    { raw::Camera camera(opts);bool failed=false;try{camera.next([]{return true;});}catch(const std::runtime_error&){failed=true;}
      expect(failed && count(0)==count(1),"Timeout leaked partial frames"); }
    reset(0);opts.disparity_mode=1;
    { raw::Camera camera(opts);expect(count(4)==1,"Stereo mode did not disable SDK disparity");
      const auto frame=camera.next([]{return true;});
      expect(frame.left.bytes[0]==17&&frame.right.bytes[0]==19&&frame.disparity.bytes.empty(),"Stereo mode lost owned left/right pair");
      const auto k=camera.parameters();expect(k.width==1280&&k.height==1088&&k.bxf==36000,"SDK calibration not queried");
      camera.close();expect(count(4)==1,"Stereo mode did not restore original SDK mode"); }
    opts.need_q8=true;bool rejected=false;try{raw::Camera camera(opts);}catch(const std::invalid_argument&){rejected=true;}
    expect(rejected,"Original Q8 was allowed with disabled disparity producer");opts.need_q8=false;opts.disparity_mode=2;
    reset(4);
    { raw::Camera camera(opts);bool fatal=false;try{camera.next([]{return true;});}catch(const raw::FatalCameraError&){fatal=true;}
      expect(fatal && count(1)==1,"Release failure not fatal"); }
    reset(5);
    { raw::Camera camera(opts);bool failed=false;try{camera.close();}catch(const raw::FatalCameraError&){failed=true;}
      expect(failed && count(3)==1,"Close failure hidden");camera.close();expect(count(3)==1,"Failed close repeated"); }
    // The production adapter intentionally pins the SDK after a close failure.
    dlclose(library);
    std::cout<<"camera lifecycle fixture: release, partial retry, rejection, timeout, fatal release and close passed\n";
}
