#include "benchmark6.hpp"
#include <iostream>
int main(int argc,char **argv) {
  try {
    const std::string command=argc>1?argv[1]:"";
    if(command=="--prepare"&&argc==4){b6::preparePlan(b6::loadSuite(b6::fs::u8path(argv[2])),b6::fs::u8path(argv[3]));return 0;}
    if(command=="--report"&&argc==3){std::cout<<v3::jsonText(b6::report(b6::fs::u8path(argv[2])))<<'\n';return 0;}
    if(command=="--value"&&argc==4){auto j=v3::readJson(argv[2]);std::string path=argv[3];std::size_t begin=0;
      while(begin<path.size()){auto end=path.find('.',begin);auto next=b6::get(j,path.substr(begin,end-begin));j=std::move(next);if(end==std::string::npos)break;begin=end+1;}
      std::cout<<(j.kind==b6::Json::Kind::String?j.text:v3::jsonText(j,0))<<'\n';return 0;}
    std::cerr<<"r36_benchmark6 --prepare SUITE.json NEW_OUTPUT | --report OUTPUT | --value FILE.json dotted.key\n";return 2;
  }catch(const std::exception &e){std::cerr<<"benchmark6: "<<e.what()<<'\n';return 1;}
}
