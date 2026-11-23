#include "camera_compare.hpp"
#include <iostream>
#include <set>
#include <stdexcept>

int main(int argc,char** argv) {
  try {
    camera_compare::Options options;std::set<std::string> seen;
    for(int i=1;i<argc;++i) {
      const std::string key=argv[i];
      if(key=="--help"||key=="-h") {
        std::cout<<"Pure C++ live X5 camera experiment controller\n"
          "r36_camera_compare --config camera_benchmark.json --output NEW\n"
          "  [--method detseg_only|original|stereo|all] [--prepare-only]\n"
          "Each method opens the camera in a separate serial runner process.\n"
          "Reports are unpaired_live_runs, not same-frame accuracy comparisons.\n";return 0;
      }
      if(!seen.insert(key).second)throw std::invalid_argument("Duplicate argument "+key);
      if(key=="--prepare-only"){options.prepare_only=true;continue;}
      if(key!="--config"&&key!="--output"&&key!="--method")throw std::invalid_argument("Unknown argument "+key);
      if(++i>=argc)throw std::invalid_argument("Missing value for "+key);
      if(key=="--config")options.configuration=std::filesystem::u8path(argv[i]);
      else if(key=="--output")options.output=std::filesystem::u8path(argv[i]);
      else options.method=argv[i];
    }
    if(options.configuration.empty()||options.output.empty())throw std::invalid_argument("--config and --output NEW are required");
    return camera_compare::run(options);
  } catch(const std::exception& e){std::cerr<<"Camera comparison error: "<<e.what()<<'\n';return 2;}
}
