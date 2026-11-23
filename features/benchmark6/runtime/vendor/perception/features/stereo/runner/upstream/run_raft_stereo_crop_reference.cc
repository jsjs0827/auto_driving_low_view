// Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
// RAFT-Stereo (ResNet50, 640x544, iter32) X5 BPU x86 simulation inference.
// Dual-input model: left_rgb + right_rgb (RGB, NCHW, 1x3x544x640), input_type_rt=rgb.
// Output: disparity (float32, 1x1x544x640) in 640-wide pixel units.
// Preprocess: BGR->RGB, resize 640x544 (INTER_LINEAR), HWC->CHW float32, no /255
//             (scale 1/255 is fused in HZ_PREPROCESS_FOR_* on BPU).
// Supports single pair (--left/--right) and batch dir mode (--left_dir/--right_dir).

#include <algorithm>
#include <cmath>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unordered_map>
#include <vector>
#include <cctype>
#include <chrono>

#include "dnn/hb_dnn.h"
#include "dnn/hb_sys.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#define EMPTY ""
DEFINE_string(model_file, EMPTY, "Path to RAFT-Stereo *.bin (e.g. stereo_r50_640x544_iter32_sim_x5default0909.bin)");
DEFINE_string(left, EMPTY, "Left image path (single-pair mode)");
DEFINE_string(right, EMPTY, "Right image path (single-pair mode)");
DEFINE_string(left_dir, EMPTY, "Left image directory (batch mode: paired with --right_dir)");
DEFINE_string(right_dir, EMPTY, "Right image directory (batch mode)");
DEFINE_string(left_nv12_dir, EMPTY, "Left NV12 directory (sim camera mode: *.nv12 640x544, skips JPG decode/cvtColor)");
DEFINE_string(right_nv12_dir, EMPTY, "Right NV12 directory (sim camera mode)");
DEFINE_string(pre_mode, "resize", "NV12预处理: resize(640x544->640x352全图缩放) | crop(取底部352行, 无插值)");
DEFINE_string(output, "disparity.npy", "Single-pair: output .npy path. Batch: ignored, uses --output_dir");
DEFINE_string(output_dir, EMPTY, "Batch mode: output directory for disparity .npy files (default: <left_dir>_result_disparity)");
DEFINE_string(output_png_dir, EMPTY, "Optional: also save colorized disparity PNGs to this dir");
DEFINE_string(gt_dir, EMPTY, "Optional GT disparity dir (.npy, 1280x1088 float32) for online EPE metrics");
DEFINE_string(calibration, EMPTY, "Optional camera .conf (org_binocular_camera_2533.conf) to export depth_mm PNG");
DEFINE_bool(restore_original_resolution, false, "If true, resize disparity to 1280x1088 and scale x2 for original Q");
DEFINE_double(depth_min, 0.1, "Depth min (m) for depth_mm export sanity");
DEFINE_double(depth_max, 30.0, "Depth max (m) for depth_mm export sanity");
DEFINE_int32(log_level, 2, "glog VLOG level");

enum VLOG_LEVEL { EXAMPLE_SYSTEM=0, EXAMPLE_REPORT=1, EXAMPLE_DETAIL=2, EXAMPLE_DEBUG=3 };

#define HB_CHECK_SUCCESS(v,msg) do{auto rc=v; if(rc!=0){VLOG(EXAMPLE_SYSTEM)<<msg<<", rc:"<<rc; return rc;}}while(0)

struct EvalStats{
  int evaluated=0, missing_gt=0, failed=0;
  double sum_epe=0, sum_bad1=0, sum_bad2=0, sum_bad3=0, sum_d1=0;
  std::vector<std::string> per_image_lines;
};

static std::string ToLowerCopy(const std::string& s){ std::string o=s; for(auto &c:o) c=tolower((unsigned char)c); return o; }
static bool IsImageFile(const std::string& f){
  auto e=ToLowerCopy(f.find('.')==std::string::npos?"":f.substr(f.find_last_of('.')));
  return e==".jpg"||e==".jpeg"||e==".png"||e==".bmp"||e==".webp";
}
static std::string JoinPath(const std::string& d,const std::string& n){
  if(d.empty()) return n; if(d.back()=='/'||d.back()=='\\') return d+n; return d+"/"+n;
}
static std::string BaseName(const std::string& p){
  size_t s=p.find_last_of("/\\"); std::string n=s==std::string::npos?p:p.substr(s+1);
  size_t d=n.find_last_of('.'); return d==std::string::npos?n:n.substr(0,d);
}
static bool EnsureDir(const std::string& d){
  struct stat st; if(stat(d.c_str(),&st)==0) return S_ISDIR(st.st_mode);
  std::string cmd="mkdir -p \""+d+"\""; return system(cmd.c_str())==0;
}
static std::vector<std::string> ListImages(const std::string& dir){
  std::vector<std::string> out; DIR* dp=opendir(dir.c_str()); if(!dp){LOG(ERROR)<<"opendir: "<<dir; return out;}
  for(dirent* e=readdir(dp);e;e=readdir(dp)){
    std::string n=e->d_name; if(n=="."||n=="..") continue;
    if(!IsImageFile(n)) continue; out.push_back(JoinPath(dir,n));
  } closedir(dp); std::sort(out.begin(),out.end()); return out;
}

static bool SaveNpyFloat32(const std::string& path, const float* data, int h,int w){
  std::ofstream ofs(path, std::ios::binary); if(!ofs) return false;
  std::string header = "{'descr': '<f4', 'fortran_order': False, 'shape': ("+std::to_string(h)+", "+std::to_string(w)+"), }";
  int header_len = (int)header.size()+1;
  int pad = 64 - (10 + header_len)%64;
  header += std::string(pad, ' ');
  header += "\n";
  uint16_t hlen = (uint16_t)header.size();
  char magic[6]={char(0x93),'N','U','M','P','Y'};
  ofs.write(magic,6);
  uint8_t major=1, minor=0;
  ofs.write((char*)&major,1); ofs.write((char*)&minor,1);
  ofs.write((char*)&hlen,2);
  ofs.write(header.c_str(), header.size());
  ofs.write((char*)data, (size_t)h*w*sizeof(float));
  return true;
}
static bool LoadNpyFloat32(const std::string& path, std::vector<float>& out, int& h,int& w){
  std::ifstream ifs(path, std::ios::binary); if(!ifs) return false;
  char magic[6]; ifs.read(magic,6); if(magic[0]!=char(0x93)) return false;
  uint8_t major,minor; ifs.read((char*)&major,1); ifs.read((char*)&minor,1);
  uint16_t hlen; ifs.read((char*)&hlen,2);
  std::string header(hlen,' '); ifs.read(&header[0], hlen);
  auto p = header.find("shape");
  if(p==std::string::npos) return false;
  auto l = header.find('(',p); auto r=header.find(')',l);
  std::string shape = header.substr(l+1, r-l-1);
  std::vector<int> dims;
  std::stringstream ss(shape);
  std::string tok;
  while(std::getline(ss,tok,',')){
    size_t a=tok.find_first_not_of(" \t"), b=tok.find_last_not_of(" \t");
    if(a==std::string::npos) continue;
    tok=tok.substr(a,b-a+1);
    if(tok.empty()) continue;
    dims.push_back(std::stoi(tok));
  }
  if(dims.size()==1){ h=dims[0]; w=1; }
  else if(dims.size()==2){ h=dims[0]; w=dims[1]; }
  else if(dims.size()==3 && dims[0]==1){ h=dims[1]; w=dims[2]; }
  else if(dims.size()==4 && dims[1]==1){ h=dims[2]; w=dims[3]; }
  else return false;
  size_t n=(size_t)h*w;
  out.resize(n);
  ifs.read((char*)out.data(), n*sizeof(float));
  return true;
}

static bool ParseConf(const std::string& path, std::unordered_map<std::string,double>& kv){
  std::ifstream ifs(path); if(!ifs) return false;
  std::string line;
  while(std::getline(ifs,line)){
    auto c=line.find('#'); if(c!=std::string::npos) line=line.substr(0,c);
    auto k=line.find(':'); if(k==std::string::npos) continue;
    std::string key=line.substr(0,k), val=line.substr(k+1);
    auto trim=[](std::string s){
      size_t a=s.find_first_not_of(" \t\r\n"), b=s.find_last_not_of(" \t\r\n");
      return a==std::string::npos?std::string():s.substr(a,b-a+1);
    };
    key=trim(key); val=trim(val);
    if(key.empty()||val.empty()) continue;
    kv[key]=std::stod(val);
  }
  return true;
}

static float ReadOutputFloat(const hbDNNTensor& t, int idx){
  const auto& pr=t.properties;
  void* base=t.sysMem[0].virAddr;
  if(pr.quantiType==NONE && pr.tensorType==HB_DNN_TENSOR_TYPE_F32) return ((float*)base)[idx];
  int32_t raw=0;
  if(pr.tensorType==HB_DNN_TENSOR_TYPE_S8) raw=((int8_t*)base)[idx];
  else if(pr.tensorType==HB_DNN_TENSOR_TYPE_U8) raw=((uint8_t*)base)[idx];
  else if(pr.tensorType==HB_DNN_TENSOR_TYPE_S16) raw=((int16_t*)base)[idx];
  else if(pr.tensorType==HB_DNN_TENSOR_TYPE_S32) raw=((int32_t*)base)[idx];
  else if(pr.tensorType==HB_DNN_TENSOR_TYPE_F32) return ((float*)base)[idx];
  else raw=((int32_t*)base)[idx];
  if(pr.quantiType==SHIFT){
    int32_t shift=pr.shift.shiftLen>0?pr.shift.shiftData[0]:0;
    if(pr.shift.shiftLen>idx) shift=pr.shift.shiftData[idx];
    return (float)raw / (float)(1<<shift);
  }
  if(pr.quantiType==SCALE){
    float sc=pr.scale.scaleLen>0?pr.scale.scaleData[0]:1.0f;
    if(pr.scale.scaleLen>idx) sc=pr.scale.scaleData[idx];
    return (float)raw * sc;
  }
  return (float)raw;
}

static cv::Mat ColorizeDisparity(const cv::Mat& disp, float max_disp=192){
  cv::Mat norm, color;
  cv::Mat clamped;
  disp.convertTo(clamped, CV_32F);
  for(int y=0;y<clamped.rows;y++) for(int x=0;x<clamped.cols;x++) clamped.at<float>(y,x)=std::max(0.f,std::min(max_disp, clamped.at<float>(y,x)));
  clamped.convertTo(norm, CV_8U, 255.0/max_disp);
  // JET: OpenCV 3.4 compatible (TURBO needs OpenCV>=4.1)
  cv::applyColorMap(norm, color, cv::COLORMAP_JET);
  return color;
}

static bool FillTensorFromNv12File(const std::string& nv12_path, int src_w, int src_h,
                                     hbDNNTensor& tensor, int expect_h, int expect_w,
                                     double* resize_ms_out = nullptr,
                                     const std::string& pre_mode = "resize"){
  auto t0 = std::chrono::steady_clock::now();
  size_t src_size = (size_t)src_w * src_h * 3 / 2;
  std::vector<uint8_t> src(src_size);
  {
    std::ifstream ifs(nv12_path, std::ios::binary);
    if(!ifs){ LOG(ERROR)<<"open nv12 failed: "<<nv12_path; return false; }
    ifs.read((char*)src.data(), src_size);
    if((size_t)ifs.gcount() != src_size){ LOG(ERROR)<<"nv12 size mismatch: "<<nv12_path; return false; }
  }
  size_t dst_size = (size_t)expect_w * expect_h * 3 / 2;
  std::vector<uint8_t> dst(dst_size);
  if(pre_mode == "crop" && src_w == expect_w && src_h >= expect_h){
    // 底部裁剪: Y取最后expect_h行, UV取最后expect_h/2行, 纯memcpy无插值
    int y_off = src_h - expect_h;
    memcpy(dst.data(), src.data() + (size_t)y_off * src_w, (size_t)expect_w * expect_h);
    const uint8_t* src_uv = src.data() + (size_t)src_w * src_h;
    int uv_off = (y_off / 2) * src_w;
    memcpy(dst.data() + (size_t)expect_w * expect_h, src_uv + uv_off,
           (size_t)expect_w * expect_h / 2);
  } else {
  const uint8_t* src_y = src.data();
  const uint8_t* src_uv = src.data() + (size_t)src_w * src_h;
  uint8_t* dst_y = dst.data();
  uint8_t* dst_uv = dst.data() + (size_t)expect_w * expect_h;
  for(int y=0; y<expect_h; y++){
    float sy = (y + 0.5f) * src_h / expect_h - 0.5f;
    int y0 = std::max(0, (int)std::floor(sy)), y1 = std::min(src_h-1, y0+1);
    float wy = sy - y0;
    for(int x=0; x<expect_w; x++){
      float sx = (x + 0.5f) * src_w / expect_w - 0.5f;
      int x0 = std::max(0, (int)std::floor(sx)), x1 = std::min(src_w-1, x0+1);
      float wx = sx - x0;
      float v = src_y[y0*src_w+x0]*(1-wx)*(1-wy) + src_y[y0*src_w+x1]*wx*(1-wy)
              + src_y[y1*src_w+x0]*(1-wx)*wy + src_y[y1*src_w+x1]*wx*wy;
      dst_y[y*expect_w+x] = (uint8_t)(v + 0.5f);
    }
  }
  int src_uvw = src_w/2, src_uvh = src_h/2, dst_uvw = expect_w/2, dst_uvh = expect_h/2;
  for(int y=0; y<dst_uvh; y++){
    float sy = (y + 0.5f) * src_uvh / dst_uvh - 0.5f;
    int y0 = std::max(0, (int)std::floor(sy)), y1 = std::min(src_uvh-1, y0+1);
    float wy = sy - y0;
    for(int x=0; x<dst_uvw; x++){
      float sx = (x + 0.5f) * src_uvw / dst_uvw - 0.5f;
      int x0 = std::max(0, (int)std::floor(sx)), x1 = std::min(src_uvw-1, x0+1);
      float wx = sx - x0;
      for(int k=0; k<2; k++){
        float v = src_uv[(y0*src_w+x0*2)+k]*(1-wx)*(1-wy) + src_uv[(y0*src_w+x1*2)+k]*wx*(1-wy)
                + src_uv[(y1*src_w+x0*2)+k]*(1-wx)*wy + src_uv[(y1*src_w+x1*2)+k]*wx*wy;
        dst_uv[y*expect_w+x*2+k] = (uint8_t)(v + 0.5f);
      }
    }
  }
  }  // end resize branch
  if(resize_ms_out){
    *resize_ms_out = std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now() - t0).count();
  }
  if(tensor.sysMem[0].virAddr == nullptr) return false;
  size_t cap = tensor.properties.alignedByteSize;
  memcpy(tensor.sysMem[0].virAddr, dst.data(), std::min(dst_size, cap));
  return true;
}

static std::vector<std::string> ListNv12(const std::string& dir){
  std::vector<std::string> out; DIR* dp=opendir(dir.c_str()); if(!dp){LOG(ERROR)<<"opendir: "<<dir; return out;}
  for(dirent* e=readdir(dp);e;e=readdir(dp)){
    std::string n=e->d_name; if(n=="."||n=="..") continue;
    auto p=n.find_last_of('.'); std::string ext=p==std::string::npos?"":ToLowerCopy(n.substr(p));
    if(ext!=".nv12" && ext!=".yuv") continue; out.push_back(JoinPath(dir,n));
  } closedir(dp); std::sort(out.begin(),out.end()); return out;
}

static bool FillTensorFromImage(const std::string& image_path, hbDNNTensor& tensor, int expect_h,int expect_w){
  cv::Mat bgr=cv::imread(image_path, cv::IMREAD_COLOR);
  if(bgr.empty()){ LOG(ERROR)<<"imread failed: "<<image_path; return false;}
  int tt = (int)tensor.properties.tensorType;
  // NV12 (type 1, HB_DNN_IMG_TYPE_NV12): Y(HW) + interleaved VU(HW/2), 1.5*HW bytes
  if(tt==1 || tt==2){
    cv::Mat resized_bgr; cv::resize(bgr,resized_bgr,cv::Size(expect_w,expect_h),0,0,cv::INTER_LINEAR);
    cv::Mat yuv; cv::cvtColor(resized_bgr, yuv, cv::COLOR_BGR2YUV_I420);
    // yuv I420: Y(HW) + U(HW/4) + V(HW/4), total 1.5*HW
    uint8_t* dst=reinterpret_cast<uint8_t*>(tensor.sysMem[0].virAddr);
    int hw=expect_h*expect_w;
    const uint8_t* src=yuv.ptr<uint8_t>(0);
    memcpy(dst, src, (size_t)hw);  // Y
    const uint8_t* u_plane=src+hw;
    const uint8_t* v_plane=src+hw+hw/4;
    uint8_t* vu=dst+hw;
    // I420 U/V planar -> NV12 interleaved: order depends on tt (NV12=UV, NV21=XU=VU)
    bool vu_order = (tt==2);  // assume type2 = NV21/XU
    for(int i=0;i<hw/4;i++){
      if(vu_order){ vu[2*i]=v_plane[i]; vu[2*i+1]=u_plane[i]; }
      else { vu[2*i]=u_plane[i]; vu[2*i+1]=v_plane[i]; }
    }
    return true;
  }
  cv::Mat rgb; cv::cvtColor(bgr,rgb,cv::COLOR_BGR2RGB);
  cv::Mat resized; cv::resize(rgb,resized,cv::Size(expect_w,expect_h),0,0,cv::INTER_LINEAR);
  // input_type_rt=rgb + data_scale=1/255 fused in HZ_PREPROCESS_FOR_*: sysMem holds
  // uint8 RGB planar (NCHW bytes). Do NOT write float 0-255 here, otherwise the
  // model sees a near-black image and disparity collapses to a narrow 2~15 range.
  if(tensor.properties.tensorType==HB_DNN_IMG_TYPE_RGB ||
     tensor.properties.tensorType==HB_DNN_IMG_TYPE_BGR){
    uint8_t* mem=reinterpret_cast<uint8_t*>(tensor.sysMem[0].virAddr);
    int hw=expect_h*expect_w;
    uint8_t* ch0=mem, *ch1=mem+hw, *ch2=mem+hw*2;
    // resized is RGB HWC interleaved -> planar NCHW bytes
    for(int y=0;y<expect_h;y++){
      const uint8_t* row=resized.ptr<uint8_t>(y);
      for(int x=0;x<expect_w;x++){
        ch0[y*expect_w+x]=row[x*3+0];
        ch1[y*expect_w+x]=row[x*3+1];
        ch2[y*expect_w+x]=row[x*3+2];
      }
    }
    return true;
  }
  // fallback: true float tensor path
  float* dst=(float*)tensor.sysMem[0].virAddr;
  int hw=expect_h*expect_w;
  for(int y=0;y<expect_h;y++){
    for(int x=0;x<expect_w;x++){
      cv::Vec3b px=resized.at<cv::Vec3b>(y,x);
      dst[0*hw + y*expect_w + x]= (float)px[0];
      dst[1*hw + y*expect_w + x]= (float)px[1];
      dst[2*hw + y*expect_w + x]= (float)px[2];
    }
  }
  return true;
}

static void EvaluateOne(const std::string& stem, const float* pred,int ph,int pw,
                        const std::string& gt_dir, EvalStats& stats){
  if(gt_dir.empty()) return;
  std::string gt_path=JoinPath(gt_dir, stem+".npy");
  std::vector<float> gt; int gh,gw;
  if(!LoadNpyFloat32(gt_path, gt, gh,gw)){
    stats.missing_gt++; return;
  }
  // GT is 1280x1088, pred is 640x544: upsample pred then scale disparity by width ratio
  cv::Mat pred_mat(ph,pw,CV_32F,(void*)pred);
  cv::Mat pred_up;
  cv::resize(pred_mat, pred_up, cv::Size(gw,gh), 0,0, cv::INTER_LINEAR);
  pred_up *= (float)gw / (float)pw;
  double sum_epe=0; int valid=0, bad1=0,bad2=0,bad3=0,d1=0;
  for(int y=0;y<gh;y++) for(int x=0;x<gw;x++){
    float g=gt[y*gw+x]; float p=pred_up.at<float>(y,x);
    if(!std::isfinite(g) || g<=0 || g>192) continue;
    float e=std::abs(p-g);
    sum_epe+=e; valid++;
    if(e>1) bad1++; if(e>2) bad2++; if(e>3) bad3++;
    if(e>3 && e/std::max(g,1.f)>0.05) d1++;
  }
  if(valid==0){ stats.missing_gt++; return; }
  double epe=sum_epe/valid;
  double b1=(double)bad1/valid*100, b2=(double)bad2/valid*100, b3=(double)bad3/valid*100, d1p=(double)d1/valid*100;
  stats.evaluated++; stats.sum_epe+=epe; stats.sum_bad1+=b1; stats.sum_bad2+=b2; stats.sum_bad3+=b3; stats.sum_d1+=d1p;
  char buf[512];
  snprintf(buf,sizeof(buf),"%s  EPE=%.3f  Bad1=%.2f%%  Bad2=%.2f%%  Bad3=%.2f%%  D1=%.2f%%  valid=%d/%d", stem.c_str(), epe,b1,b2,b3,d1p, valid, gh*gw);
  stats.per_image_lines.push_back(buf);
  VLOG(EXAMPLE_REPORT)<<buf;
}

int main(int argc,char** argv){
  gflags::SetUsageMessage("RAFT-Stereo X5 BPU x86 simulation (640x544, dual-input).");
  gflags::ParseCommandLineFlags(&argc,&argv,true);
  std::cout<<gflags::GetArgv()<<std::endl;
  google::InitGoogleLogging(""); google::SetStderrLogging(0);
  google::SetVLOGLevel("*", FLAGS_log_level);
  FLAGS_colorlogtostderr=true; FLAGS_minloglevel=google::INFO; FLAGS_logtostderr=true;

  if(FLAGS_model_file.empty()){ LOG(ERROR)<<"--model_file required"; return -1; }
  bool single = !FLAGS_left.empty() && !FLAGS_right.empty();
  bool batch = !FLAGS_left_dir.empty() && !FLAGS_right_dir.empty();
  bool nv12batch = !FLAGS_left_nv12_dir.empty() && !FLAGS_right_nv12_dir.empty();
  // NV12源尺寸: 相机直出(默认640x544, 与resized数据集一致); 由文件名无关, 按实际字节数自适应
  const int kNv12SrcW = 640, kNv12SrcH = 544;
  if(!single && !batch && !nv12batch){ LOG(ERROR)<<"Specify --left/--right (single) or --left_dir/--right_dir (batch) or --left_nv12_dir/--right_nv12_dir (nv12 batch)"; return -1; }
  if((single?1:0)+(batch?1:0)+(nv12batch?1:0) > 1){ LOG(ERROR)<<"Use only one of single/batch/nv12batch mode"; return -1; }

  std::vector<std::pair<std::string,std::string>> pairs;
  if(single){
    pairs.emplace_back(FLAGS_left, FLAGS_right);
  } else if(nv12batch){
    auto lefts=ListNv12(FLAGS_left_nv12_dir);
    if(lefts.empty()){ LOG(ERROR)<<"No nv12 in "<<FLAGS_left_nv12_dir; return -1; }
    for(auto& lp: lefts){
      std::string name=lp.substr(lp.find_last_of("/\\")+1);
      std::string rp=JoinPath(FLAGS_right_nv12_dir, name);
      struct stat st; if(stat(rp.c_str(),&st)!=0){ LOG(WARNING)<<"No paired right for "<<name<<" skip"; continue; }
      pairs.emplace_back(lp,rp);
    }
    LOG(INFO)<<"Batch NV12 pairs: "<<pairs.size()<<" src="<<kNv12SrcW<<"x"<<kNv12SrcH;
  } else {
    auto lefts=ListImages(FLAGS_left_dir);
    if(lefts.empty()){ LOG(ERROR)<<"No images in "<<FLAGS_left_dir; return -1; }
    for(auto& lp: lefts){
      std::string name=lp.substr(lp.find_last_of("/\\")+1);
      std::string rp=JoinPath(FLAGS_right_dir, name);
      struct stat st; if(stat(rp.c_str(),&st)!=0){
        std::string stem=BaseName(lp);
        bool found=false;
        DIR* dp=opendir(FLAGS_right_dir.c_str());
        if(dp){ for(dirent* e=readdir(dp);e;e=readdir(dp)){ std::string n=e->d_name; if(BaseName(n)==stem){ rp=JoinPath(FLAGS_right_dir,n); found=true; break; } } closedir(dp); }
        if(!found){ LOG(WARNING)<<"No paired right for "<<name<<" skip"; continue; }
      }
      pairs.emplace_back(lp,rp);
    }
    LOG(INFO)<<"Batch pairs: "<<pairs.size();
  }

  std::string out_dir = FLAGS_output_dir;
  bool need_dir = batch || nv12batch;
  if(need_dir && out_dir.empty()){
    // default: <left_dir>_result_disparity (same as run_raft_stereo.sh)
    out_dir = nv12batch ? FLAGS_left_nv12_dir : FLAGS_left_dir;
    if(!out_dir.empty() && out_dir.back()=='/') out_dir.pop_back();
    out_dir += "_result_disparity";
  }
  if(need_dir) EnsureDir(out_dir);
  if(!FLAGS_output_png_dir.empty()) EnsureDir(FLAGS_output_png_dir);

  hbPackedDNNHandle_t packed; hbDNNHandle_t dnn; const char** names; int cnt=0;
  const char* mf=FLAGS_model_file.c_str();
  HB_CHECK_SUCCESS(hbDNNInitializeFromFiles(&packed,&mf,1),"hbDNNInitializeFromFiles");
  HB_CHECK_SUCCESS(hbDNNGetModelNameList(&names,&cnt,packed),"GetModelNameList");
  HB_CHECK_SUCCESS(hbDNNGetModelHandle(&dnn,packed,names[0]),"GetModelHandle");
  VLOG(EXAMPLE_REPORT)<<"Model: "<<names[0]<<"  packed_cnt="<<cnt;
  VLOG(EXAMPLE_DEBUG)<<"DNN version: "<<hbDNNGetVersion();

  int ic=0,oc=0;
  HB_CHECK_SUCCESS(hbDNNGetInputCount(&ic,dnn),"GetInputCount");
  HB_CHECK_SUCCESS(hbDNNGetOutputCount(&oc,dnn),"GetOutputCount");
  if(ic!=2){ LOG(ERROR)<<"Expected 2 inputs, got "<<ic; return -1; }
  if(oc!=1){ LOG(ERROR)<<"Expected 1 output, got "<<oc; return -1; }

  std::vector<hbDNNTensor> inputs(ic);
  for(int i=0;i<ic;i++){
    HB_CHECK_SUCCESS(hbDNNGetInputTensorProperties(&inputs[i].properties,dnn,i),"GetInputProps");
    HB_CHECK_SUCCESS(hbSysAllocCachedMem(&inputs[i].sysMem[0], inputs[i].properties.alignedByteSize),"Alloc input");
    inputs[i].properties.alignedShape = inputs[i].properties.validShape;
    const char* n=nullptr; hbDNNGetInputName(&n,dnn,i);
    VLOG(EXAMPLE_REPORT)<<"Input["<<i<<"] "<<(n?n:"?")<<" shape=["<<inputs[i].properties.validShape.dimensionSize[0]<<","<<inputs[i].properties.validShape.dimensionSize[1]<<","<<inputs[i].properties.validShape.dimensionSize[2]<<","<<inputs[i].properties.validShape.dimensionSize[3]<<"] type="<<inputs[i].properties.tensorType<<" layout="<<inputs[i].properties.tensorLayout;
  }
  std::vector<hbDNNTensor> outputs(oc);
  for(int i=0;i<oc;i++){
    HB_CHECK_SUCCESS(hbDNNGetOutputTensorProperties(&outputs[i].properties,dnn,i),"GetOutputProps");
    HB_CHECK_SUCCESS(hbSysAllocCachedMem(&outputs[i].sysMem[0], outputs[i].properties.alignedByteSize),"Alloc output");
    const char* n=nullptr; hbDNNGetOutputName(&n,dnn,i);
    VLOG(EXAMPLE_REPORT)<<"Output["<<i<<"] "<<(n?n:"?")<<" shape=["<<outputs[i].properties.validShape.dimensionSize[0]<<","<<outputs[i].properties.validShape.dimensionSize[1]<<","<<outputs[i].properties.validShape.dimensionSize[2]<<","<<outputs[i].properties.validShape.dimensionSize[3]<<"] type="<<outputs[i].properties.tensorType;
  }
  int in_h = inputs[0].properties.validShape.dimensionSize[2];
  int in_w = inputs[0].properties.validShape.dimensionSize[3];
  int out_h = outputs[0].properties.validShape.dimensionSize[2];
  int out_w = outputs[0].properties.validShape.dimensionSize[3];
  VLOG(EXAMPLE_REPORT)<<"Model I/O: input "<<in_h<<"x"<<in_w<<"  output "<<out_h<<"x"<<out_w;

  std::unordered_map<std::string,double> calib;
  bool has_calib=false;
  if(!FLAGS_calibration.empty()){
    if(!ParseConf(FLAGS_calibration, calib)){ LOG(ERROR)<<"Failed to parse calibration: "<<FLAGS_calibration; return -1; }
    has_calib=true;
    VLOG(EXAMPLE_REPORT)<<"Calibration: "<<FLAGS_calibration<<"  fx="<<calib["left.fx"]<<"  baseline~"<<calib["translation_t_0"]<<"mm";
  }

  EvalStats eval;
  int failed=0;
  double sum_pre_ms=0, sum_infer_ms=0, sum_post_ms=0, sum_get_ms=0;
  double max_infer_ms=0, max_get_ms=0;
  int timed_frames=0;
  auto now_ms = [](){
    return std::chrono::steady_clock::now();
  };
  auto elapsed_ms = [](std::chrono::steady_clock::time_point a,
                       std::chrono::steady_clock::time_point b){
    return std::chrono::duration<double, std::milli>(b - a).count();
  };
  for(size_t idx=0; idx<pairs.size(); idx++){
    const auto& pr=pairs[idx];
    std::string stem=BaseName(pr.first);
    LOG(INFO)<<"["<<(idx+1)<<"/"<<pairs.size()<<"] "<<pr.first<<" | "<<pr.second;
    auto t_pre0 = now_ms();
    double nv12_resize_ms = 0;
    if(nv12batch){
      double r0=0, r1=0;
      if(!FillTensorFromNv12File(pr.first, kNv12SrcW, kNv12SrcH, inputs[0], in_h, in_w, &r0, FLAGS_pre_mode)){ failed++; eval.failed++; continue; }
      if(!FillTensorFromNv12File(pr.second, kNv12SrcW, kNv12SrcH, inputs[1], in_h, in_w, &r1, FLAGS_pre_mode)){ failed++; eval.failed++; continue; }
      nv12_resize_ms = r0 + r1;
    } else {
      if(!FillTensorFromImage(pr.first, inputs[0], in_h,in_w)){ failed++; eval.failed++; continue; }
      if(!FillTensorFromImage(pr.second, inputs[1], in_h,in_w)){ failed++; eval.failed++; continue; }
    }
    for(int i=0;i<ic;i++) hbSysFlushMem(&inputs[i].sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
    auto t_pre1 = now_ms();

    hbDNNTaskHandle_t task=nullptr;
    hbDNNTensor* out_ptr=outputs.data();
    hbDNNInferCtrlParam ctrl; HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&ctrl);
    auto t_inf0 = now_ms();
    int rc=hbDNNInfer(&task, &out_ptr, inputs.data(), dnn, &ctrl);
    if(rc!=0){ LOG(ERROR)<<"hbDNNInfer failed rc="<<rc; failed++; eval.failed++; continue; }
    rc=hbDNNWaitTaskDone(task,0);
    auto t_inf1 = now_ms();
    if(rc!=0){ LOG(ERROR)<<"WaitTaskDone rc="<<rc; hbDNNReleaseTask(task); failed++; eval.failed++; continue; }
    hbSysFlushMem(&outputs[0].sysMem[0], HB_SYS_MEM_CACHE_INVALIDATE);

    int h=out_h, w=out_w;
    if(outputs[0].properties.tensorLayout==HB_DNN_LAYOUT_NHWC){ h=outputs[0].properties.validShape.dimensionSize[1]; w=outputs[0].properties.validShape.dimensionSize[2]; }
    auto t_post0 = now_ms();
    std::vector<float> disp((size_t)h*w);
    for(int i=0;i<h*w;i++) disp[i]=ReadOutputFloat(outputs[0], i);
    for(auto& v: disp) if(v<0) v=0;
    auto t_got = now_ms();  // 到视差为止(含反量化+clip), 不含落盘/png/EPE

    std::string npy_path;
    if(single) npy_path=FLAGS_output;
    else npy_path=JoinPath(out_dir, stem+".npy");
    if(!SaveNpyFloat32(npy_path, disp.data(), h,w)){ LOG(ERROR)<<"Save npy failed: "<<npy_path; }
    else VLOG(EXAMPLE_REPORT)<<"Saved disparity: "<<npy_path<<"  "<<w<<"x"<<h;

    if(!FLAGS_output_png_dir.empty()){
      cv::Mat dm(h,w,CV_32F,disp.data());
      cv::Mat color=ColorizeDisparity(dm, 192);
      std::string png=JoinPath(FLAGS_output_png_dir, stem+"_disp.png");
      cv::imwrite(png, color);
    }

    if(has_calib){
      double fx = calib["left.fx"];
      double baseline_m = std::abs(calib["translation_t_0"])*0.001;
      cv::Mat disp_mat(h,w,CV_32F,disp.data());
      cv::Mat disp_for_depth = disp_mat;
      double fx_used = fx;
      if(FLAGS_restore_original_resolution){
        cv::resize(disp_mat, disp_for_depth, cv::Size(1280,1088), 0,0, cv::INTER_LINEAR);
        disp_for_depth *= (1280.0 / w);
        fx_used = fx;
      } else {
        fx_used = fx * (w / 1280.0);
      }
      cv::Mat depth_m(h,w,CV_32F);
      for(int y=0;y<disp_for_depth.rows;y++) for(int x=0;x<disp_for_depth.cols;x++){
        float d=disp_for_depth.at<float>(y,x);
        float z = d>0.5f ? (float)(fx_used*baseline_m/d) : 0.f;
        if(z<FLAGS_depth_min*1000) z=0; if(z>FLAGS_depth_max*1000) z=0;
        depth_m.at<float>(y,x)=z;
      }
      cv::Mat depth_mm;
      depth_m.convertTo(depth_mm, CV_16U, 1000.0);
      std::string depth_path = batch? JoinPath(out_dir, "depth_mm_"+stem+".png") : (BaseName(npy_path)+"_depth_mm.png");
      cv::imwrite(depth_path, depth_mm);
    }

    EvaluateOne(stem, disp.data(), h,w, FLAGS_gt_dir, eval);

    hbDNNReleaseTask(task);
    {
      double pre_ms = elapsed_ms(t_pre0, t_pre1);
      double infer_ms = elapsed_ms(t_inf0, t_inf1);
      double get_ms = elapsed_ms(t_pre0, t_got);  // 预处理+推理+反量化=到视差总耗时
      auto t_end = now_ms();
      double post_ms = elapsed_ms(t_inf1, t_end);
      sum_pre_ms += pre_ms; sum_infer_ms += infer_ms; sum_post_ms += post_ms; sum_get_ms += get_ms;
      if(infer_ms > max_infer_ms) max_infer_ms = infer_ms;
      if(get_ms > max_get_ms) max_get_ms = get_ms;
      timed_frames++;
      LOG(INFO)<<"[time] "<<stem<<" pre="<<std::fixed<<std::setprecision(1)<<pre_ms<<"ms"
               <<" infer="<<infer_ms<<"ms get_disp="<<get_ms<<"ms post="<<post_ms<<"ms"
               <<" total="<<(pre_ms+infer_ms+post_ms)<<"ms"
               <<(nv12batch?(" nv12_resize=" + std::to_string(nv12_resize_ms).substr(0,5) + "ms"):"");
    }
  }

  if(timed_frames>0){
    double m_pre=sum_pre_ms/timed_frames, m_inf=sum_infer_ms/timed_frames, m_post=sum_post_ms/timed_frames, m_get=sum_get_ms/timed_frames;
    std::cout<<"\n===== Timing (avg over "<<timed_frames<<" frames) =====\n";
    std::cout<<std::fixed<<std::setprecision(1);
    std::cout<<"preprocess: "<<m_pre<<" ms/frame\n";
    std::cout<<"infer(BPU): "<<m_inf<<" ms/frame  (max "<<max_infer_ms<<" ms)\n";
    std::cout<<"get_disp(pre+infer+dequant): "<<m_get<<" ms/frame  (max "<<max_get_ms<<" ms)\n";
    std::cout<<"postprocess(save/eval): "<<m_post<<" ms/frame\n";
    std::cout<<"end-to-end: "<<(m_pre+m_inf+m_post)<<" ms/frame  (~"<<(1000.0/(m_pre+m_inf+m_post))<<" FPS)\n";
  }

  if(!FLAGS_gt_dir.empty()){
    std::cout<<"\n===== Stereo Evaluation (GT: "<<FLAGS_gt_dir<<") =====\n";
    std::cout<<"Pairs: "<<pairs.size()<<"  Evaluated: "<<eval.evaluated<<"  Missing GT: "<<eval.missing_gt<<"  Failed: "<<eval.failed<<"\n";
    if(eval.evaluated>0){
      double mEPE=eval.sum_epe/eval.evaluated;
      double mB1=eval.sum_bad1/eval.evaluated, mB2=eval.sum_bad2/eval.evaluated, mB3=eval.sum_bad3/eval.evaluated, mD1=eval.sum_d1/eval.evaluated;
      std::cout<<std::fixed<<std::setprecision(3);
      std::cout<<"Mean EPE: "<<mEPE<<"  Bad1(>1px): "<<mB1<<"%  Bad2: "<<mB2<<"%  Bad3: "<<mB3<<"%  D1: "<<mD1<<"%\n";
      std::cout<<"Per-image:\n"; for(auto& l: eval.per_image_lines) std::cout<<"  "<<l<<"\n";
      std::string eval_txt = batch? JoinPath(out_dir, "stereo_evaluation.txt") : (BaseName(FLAGS_output)+"_evaluation.txt");
      std::ofstream ofs(eval_txt);
      if(ofs){
        ofs<<"RAFT-Stereo Evaluation\nGT: "<<FLAGS_gt_dir<<"\nModel: "<<FLAGS_model_file<<"\n";
        ofs<<"Pairs: "<<pairs.size()<<"  Evaluated: "<<eval.evaluated<<"\n";
        ofs<<std::fixed<<std::setprecision(4);
        ofs<<"Mean EPE: "<<mEPE<<"\nBad1: "<<mB1<<"%  Bad2: "<<mB2<<"%  Bad3: "<<mB3<<"%  D1: "<<mD1<<"%\n";
        for(auto& l: eval.per_image_lines) ofs<<l<<"\n";
        std::cout<<"Saved: "<<eval_txt<<"\n";
      }
    }
  }

  for(auto& t: inputs) hbSysFreeMem(&t.sysMem[0]);
  for(auto& t: outputs) hbSysFreeMem(&t.sysMem[0]);
  hbDNNRelease(packed);
  if(failed>0) LOG(ERROR)<<failed<<"/"<<pairs.size()<<" failed";
  else LOG(INFO)<<"All "<<pairs.size()<<" done";
  return failed? -1:0;
}