#include "disparity_compare.hpp"
#include <chrono>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <sys/wait.h>
#include <unistd.h>

using namespace perception_compare;
using namespace perception_compare::detail;
namespace {
void check(bool condition,const char* why){if(!condition)throw std::runtime_error(why);}
template<class F>void rejects(F f,const char* why){bool rejected=false;try{f();}catch(const std::exception&){rejected=true;}check(rejected,why);}
double get(const Json& j,const char* key){const auto& v=field(j,key);check(v.kind==Json::Kind::Number,"Expected numeric statistic");return std::stod(v.text);}
void near(double a,double b,const char* why){check(std::abs(a-b)<1e-7,why);}
void q8(const fs::path& p,const std::vector<std::uint16_t>& values){std::string s;for(auto v:values){s+=char(v&255);s+=char(v>>8);}writeText(p,s);}
int execute(const std::vector<std::string>& args){
  const auto pid=fork();if(pid<0)throw std::runtime_error("fork failed");
  if(!pid){std::vector<char*> a;for(const auto& v:args)a.push_back(const_cast<char*>(v.c_str()));a.push_back(nullptr);execv(a[0],a.data());_exit(127);}
  int status=0;if(waitpid(pid,&status,0)<0)throw std::runtime_error("waitpid failed");return WIFEXITED(status)?WEXITSTATUS(status):128;
}
// Contract-only C++ runner: records dispatch arguments, never loads models or
// claims hardware execution. The enclosing recording is synthetic_validation.
int fakeRunner(int argc,char** argv){
  std::map<std::string,std::string> args;for(int i=1;i+1<argc;i+=2)args[argv[i]]=argv[i+1];
  const auto output=fs::path(args.at("--output"));fs::create_directory(output);
  std::ifstream manifest(args.at("--manifest"));std::string line;std::getline(manifest,line);const auto stem=line.substr(0,line.find('\t'));
  writeText(output/"frames.jsonl",dump(obj({{"method",str(args.at("--method"))},{"stem",str(stem)},
    {"repeat",integer(0)},{"warmup",boolean(false)},{"compute_wall_ms",number(10)},{"total_wall_ms",number(12)},
    {"cpu_ms",number(2)},{"cpu_wall_ms",number(10)},{"online_cpus",integer(8)},
    {"rss_kib",integer(1000)},{"output_dir",Json{}},{"task_trace",obj({{"tasks",array()}})}}))+"\n");
  writeJson(output/"complete.json",obj({{"complete",boolean(true)},{"measured_frames",integer(1)},
    {"measured_wall_ms",number(13)},{"measured_cpu_ms",number(3)},{"online_cpus",integer(8)}}));
  return 0;
}
void helpers(){
  const Intrinsics source{640,544,300,300,320,272,18000};
  const auto r=comparisonRoi("crop",640,352,source);
  check(r.x==0&&r.y==192&&r.width==640&&r.height==352,"Bottom crop is not source-aligned 0,192,640,352");
  const auto meta=comparisonRoiJson(r,source);
  near(get(field(meta,"source_intrinsics"),"cy"),272,"Source cy changed");
  near(get(field(meta,"crop_intrinsics"),"cy"),80,"Crop cy must subtract ROI y");
  near(get(field(meta,"crop_intrinsics"),"bxf"),18000,"Crop must not scale bxf");
  std::vector<std::uint16_t> original(640*544);for(std::size_t i=0;i<original.size();++i)original[i]=std::uint16_t((i%640)+1);
  original[192*640]=4321;const auto cut=sliceDisparity(original,r);
  check(cut.size()==640*352&&cut[0]==4321&&cut[640]==1,"ROI is not sliced at identical source pixel coordinates");
  rejects([&]{comparisonRoi("crop",0,0,source);},"Unspecified crop model dimensions accepted");
  rejects([&]{comparisonRoi("crop",632,352,source);},"Width-changing crop accepted");
  rejects([&]{comparisonRoi("crop",640,546,source);},"Crop taller than source accepted");
  rejects([&]{comparisonRoi("crop",640,351,source);},"Odd NV12 crop accepted");
  auto old=obj({{"intrinsics",field(meta,"source_intrinsics")}}),full=old,crop=old;
  full.object["comparison_roi"]=comparisonRoiJson(comparisonRoi("resize",0,0,source),source);
  crop.object["comparison_roi"]=meta;
  check(dump(comparisonRoiIdentity(old))==dump(comparisonRoiIdentity(full)),"Legacy full-frame identity compatibility broken");
  check(dump(comparisonRoiIdentity(full))!=dump(comparisonRoiIdentity(crop)),"Crop and full-frame version identities considered equal");
  const auto empty=compareDisparity({0,256},{256,0});
  check(get(empty,"both_valid_pixels")==0&&field(empty,"mae_px").kind==Json::Kind::Null,"No joint validity must produce null differences");
}
void quality(const fs::path& root){
  const auto output=root/"quality";fs::create_directories(output/"original/frameA");fs::create_directories(output/"stereo/frameA");
  fs::create_directories(output/"original/frameB");fs::create_directories(output/"stereo/frameB");fs::create_directories(output/"prepared/pairA");fs::create_directories(output/"prepared/pairB");
  const Intrinsics k{8,6,3,3,4,3,180};const auto r=comparisonRoi("crop",8,4,k);
  auto metadata=obj({{"intrinsics",field(comparisonRoiJson(r,k),"source_intrinsics")},{"comparison_roi",comparisonRoiJson(r,k)}});
  std::vector<std::uint16_t>a(48,2560),b(48,2816);for(int i=0;i<16;++i){a[i]=50000;b[i]=1;}
  a[16]=0;b[16]=256;a[17]=2560;b[17]=0;a[18]=0;b[18]=0;b[19]=3584;b[20]=2048;
  q8(output/"original/frameA/disparity_q8.u16",a);q8(output/"stereo/frameA/disparity_q8.u16",b);
  q8(output/"original/frameB/disparity_q8.u16",std::vector<std::uint16_t>(48,2560));q8(output/"stereo/frameB/disparity_q8.u16",std::vector<std::uint16_t>(48,0));
  cv::Mat left(6,8,CV_8UC3);for(int y=0;y<6;++y)left.row(y).setTo(cv::Scalar(y*20,y*20,y*20));
  check(cv::imwrite((output/"prepared/pairA/left.png").string(),left),"Fixture left PNG failed");
  check(cv::imwrite((output/"prepared/pairB/left.png").string(),left),"Fixture left PNG failed");
  auto row=[](const char* stem,const char* directory){return obj({{"stem",str(stem)},{"repeat",integer(0)},{"warmup",boolean(false)},{"output_dir",directory?str(directory):Json{}}});};
  const auto rows=dump(row("pairA","frameA"))+"\n"+dump(row("pairB","frameB"))+"\n"+dump(row("pairC",nullptr))+"\n";
  writeText(output/"original/frames.jsonl",rows);writeText(output/"stereo/frames.jsonl",rows);
  const auto report=generateDisparityComparison(output,metadata);
  check(get(report,"common_measured_pairs")==3&&get(report,"saved_pair_count")==2&&get(report,"analyzed_pair_count")==2,"Saved/common analyzed pair counts differ");
  check(!field(report,"ground_truth").boolean&&textValue(field(report,"reference"))=="original_q8_reference_not_ground_truth","Reference must not be advertised as GT");
  const auto& pair=report.object.at("pairs").array[0];const auto& stats=field(pair,"metrics");
  near(get(stats,"roi_pixels"),32,"Full image leaked into crop metrics");
  near(get(stats,"original_valid_pixels"),30,"Reference validity count wrong");near(get(stats,"stereo_valid_pixels"),30,"Stereo validity count wrong");
  near(get(stats,"both_valid_pixels"),29,"Difference must use both-valid intersection");
  near(get(stats,"mae_px"),33.0/29,"Q8 /256 MAE wrong");near(get(stats,"rmse_px"),std::sqrt(47.0/29),"RMSE wrong");
  near(get(stats,"median_abs_difference_px"),1,"Median wrong");near(get(stats,"p95_abs_difference_px"),1.6,"Interpolated p95 wrong");
  near(get(stats,"bias_stereo_minus_original_px"),1,"Signed bias wrong");
  near(get(stats,"abs_difference_gt_1px_ratio"),2.0/29,"Difference >1 ratio wrong");
  near(get(stats,"abs_difference_gt_2px_ratio"),1.0/29,"Difference >2 ratio wrong");
  near(get(stats,"abs_difference_gt_3px_ratio"),1.0/29,"Difference >3 ratio wrong");
  check(field(field(report,"pairs").array[1],"metrics").object.at("mae_px").kind==Json::Kind::Null,"Empty intersection fabricated zero MAE");
  near(get(field(report,"aggregate"),"roi_pixels"),64,"Aggregate ROI count wrong");near(get(field(report,"aggregate"),"mae_px"),33.0/29,"Empty intersection contaminated aggregate");
  const auto roi_left=cv::imread((output/"disparity_roi/pair-1/left_roi.png").string());
  check(roi_left.size()==cv::Size(8,4)&&roi_left.at<cv::Vec3b>(0,0)[0]==40,"Left ROI has wrong source origin");
  check(fs::file_size(output/"disparity_roi/pair-1/original_q8.u16")==64,"Cropped raw Q8 bytes wrong");
  const auto npy=readText(output/"disparity_roi/pair-1/original_px.npy");check(npy.compare(0,6,"\x93NUMPY")==0&&npy.find("'<f4'")!=std::string::npos,"Float32 NPY missing");
  const auto page=readText(output/"disparity_compare.html");check(page.find("不是 Ground Truth")!=std::string::npos&&page.find("0–128 px")!=std::string::npos,"HTML scale/GT scope missing");
  const auto perf=root/"save0";fs::create_directories(perf/"original");fs::create_directory(perf/"stereo");
  const auto unsaved=dump(row("pairA",nullptr))+"\n";writeText(perf/"original/frames.jsonl",unsaved);writeText(perf/"stereo/frames.jsonl",unsaved);
  const auto noquality=generateDisparityComparison(perf,metadata);
  check(textValue(field(noquality,"status"))=="not_saved"&&field(noquality,"aggregate").kind==Json::Kind::Null,"save0 must not fabricate quality");
}
void cli(const fs::path& root,const std::string& app,const std::string& fake_runner){
  const auto capture=root/"capture";fs::create_directory(capture);
  writeJson(capture/"metadata.json",obj({{"validation_only",boolean(true)},{"intrinsics",obj({{"width",integer(8)},{"height",integer(6)},
    {"fx",number(3)},{"fy",number(3)},{"cx",number(4)},{"cy",number(3)},{"bxf",number(180)}})}}));
  writeText(capture/"left.nv12",std::string(72,'\x60'));writeText(capture/"right.nv12",std::string(72,'\x70'));writeText(capture/"q8.u16",std::string(96,'\x01'));
  writeText(capture/"manifest.jsonl",dump(obj({{"frame_id",integer(1)},{"timestamp_ns",integer(100)},
    {"width",integer(8)},{"height",integer(6)},{"left_format",str("nv12")},{"right_format",str("nv12")},
    {"disparity_format",str("q8_le")},{"left",str("left.nv12")},{"right",str("right.nv12")},{"disparity",str("q8.u16")}}))+"\n");
  writeJson(root/"mapping.json",object());
  writeJson(root/"full.json",obj({{"mapping",obj({{"enabled",boolean(true)},{"original_config",str("mapping.json")}})},
    {"inference",obj({{"enabled",boolean(false)}})}}));
  auto args=[&](const char* output){return std::vector<std::string>{app,"compare","--capture",capture.string(),"--sdk-config",(root/"full.json").string(),
    "--output",(root/output).string(),"--method","original","--model",(root/"does-not-exist.bin").string(),"--input-mode","crop","--prepare-only"};};
  auto accepted=args("crop_cli");accepted.insert(accepted.end(),{"--input-width","8","--input-height","4"});
  check(execute(accepted)==0,"Actual native CLI rejected crop or unnecessarily loaded Stereo model for original");
  const auto run=readJson(root/"crop_cli/run.json");near(get(field(run,"comparison_roi"),"y"),2,"CLI metadata ROI y wrong");
  check(field(run,"stereo_model").kind==Json::Kind::Null,"Original-only crop must not identify/load new model");
  check(execute(args("crop_cli_missing"))==2&&!fs::exists(root/"crop_cli_missing"),"CLI accepted crop with undeclared dims");
  auto bad=args("crop_cli_bad_width");bad.insert(bad.end(),{"--input-width","6","--input-height","4"});
  check(execute(bad)==2&&!fs::exists(root/"crop_cli_bad_width"),"CLI accepted width-changing crop");
  writeText(root/"det.bin","synthetic fixture only");writeText(root/"seg.bin","synthetic fixture only");writeText(root/"stereo.bin","synthetic fixture only");
  writeJson(root/"dag.json",parse(R"({"tasks":{"det":{"features":["ai.exact.det.backend"]},"seg":{"features":["ai.exact.seg.backend"]}}})","test DAG"));
  auto full=readJson(root/"full.json");full.object["inference"]=obj({{"enabled",boolean(true)},{"dag",str("dag.json")},
    {"detection_model",str("det.bin")},{"segmentation_model",str("seg.bin")}});
  full.object["graph"]=parse(R"({"nodes":[{"id":"det","feature":"inference","algorithm":"det"},
    {"id":"seg","feature":"inference","algorithm":"seg"},{"id":"stereo","feature":"inference","algorithm":"stereo"},
    {"id":"cloud","feature":"cloud","depends_on":["stereo"]},{"id":"publish","feature":"publish","depends_on":["det","seg","cloud"]}]})","test graph");
  writeJson(root/"triple.json",full);
  Options o;o.capture=capture;o.sdk_config=root/"triple.json";o.output=root/"dispatch_crop";o.model=root/"stereo.bin";
  o.runner=fake_runner;o.all=true;o.order="detseg_only,original,stereo";o.input_mode="crop";o.input_width=8;o.input_height=4;
  o.warmup=0;o.repeat=1;o.save_every=0;o.bpu_enabled=false;
  runCompare(o);
  const auto executions=readJson(o.output/"executions.json");check(executions.array.size()==3,"Three method dispatch coverage missing");
  for(const auto& execution:executions.array){const auto& argv=field(execution,"arguments").array;
    std::size_t found=0;bool model=false;for(std::size_t i=0;i<argv.size();++i){
      if(textValue(argv[i])=="--comparison-roi"){++found;check(i+1<argv.size()&&textValue(argv[i+1])=="0,2,8,4","A method received a different comparison ROI");}
      if(textValue(argv[i])=="--model")model=true;
    }
    check(found==1,"Each method must receive exactly one same comparison ROI");
    check(model==(textValue(field(execution,"method"))=="stereo"),"Original/detseg unnecessarily receive Stereo model");
  }
  o.input_mode="resize";o.output=root/"dispatch_resize";runCompare(o);
  for(const auto& execution:readJson(o.output/"executions.json").array)
    for(const auto& arg:field(execution,"arguments").array)check(textValue(arg)!="--comparison-roi","Legacy resize dispatch unexpectedly cropped inputs");
}
} // namespace
int main(int argc,char** argv){try{if(argc>1&&std::string(argv[1])=="--manifest")return fakeRunner(argc,argv);
  if(argc!=3)throw std::runtime_error("Provide output directory and native CLI");
  const auto root=fs::absolute(argv[1])/("crop-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));fs::create_directories(root);
  helpers();quality(root);cli(root,argv[2],fs::canonical(argv[0]).string());std::cout<<"PASS: crop ROI, pixel alignment, validity intersection, statistics, raw/NPY/PNG, no-GT scope, saved coverage, version identity, real CLI crop dimensions and all-method same-ROI dispatch.\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
