#include "ab/config.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <thread>
namespace {
using Image=sensor_msgs::msg::Image;
using Cloud=sensor_msgs::msg::PointCloud2;
cv::Vec3b color(int id){static const cv::Vec3b colors[]={{45,45,45},{55,210,55},{50,100,240}};return id<3?colors[id]:cv::Vec3b((id*73)%256,(id*131)%256,(id*197)%256);}
void save(const ab::fs::path&p,const cv::Mat&m){if(!cv::imwrite(p.u8string(),m))throw std::runtime_error("Cannot save "+p.u8string());}
double numeric(const ab::Json&j){if(j.kind!=ab::Json::Kind::Number)throw std::invalid_argument("Expected numeric intrinsics");return std::stod(j.text);}
}
int main(int argc,char**argv){try{
  if(argc<6||argc>7)throw std::invalid_argument("ab_result_replay QUALITY_DIR TOPIC_PREFIX HZ CLOUD_STRIDE DISP_MAX_PX [--derive-only|--record]");
  const auto dir=ab::fs::absolute(ab::fs::u8path(argv[1]));const std::string prefix=argv[2];const double hz=std::stod(argv[3]),maximum=std::stod(argv[5]);const int stride=std::stoi(argv[4]);const std::string mode=argc==7?argv[6]:"";const bool publish=mode!="--derive-only",record=mode=="--record";
  if(!mode.empty()&&mode!="--derive-only"&&mode!="--record")throw std::invalid_argument("Unknown replay option");
  if(hz<.1||hz>100||stride<1||stride>64||maximum<=0||maximum>256||prefix.empty()||prefix[0]!='/')throw std::invalid_argument("Invalid replay settings");
  rclcpp::Node::SharedPtr node;std::map<std::string,rclcpp::Publisher<Image>::SharedPtr> images;rclcpp::Publisher<Cloud>::SharedPtr clouds;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr results;
  if(publish){rclcpp::init(0,nullptr);node=std::make_shared<rclcpp::Node>("benchmark6_result_replay");auto qos=rclcpp::QoS(4).reliable();
    for(auto name:{"left","detection","segmentation","segmentation_overlay","disparity_color","disparity_q8"})images[name]=node->create_publisher<Image>(prefix+"/"+name,qos);
    clouds=node->create_publisher<Cloud>(prefix+"/cloud",qos);results=node->create_publisher<std_msgs::msg::String>(prefix+"/result",qos);
    auto ready=std::chrono::steady_clock::now()+std::chrono::seconds(record?30:2);
    while(rclcpp::ok()){
      rclcpp::spin_some(node);bool subscribed=clouds->get_subscription_count()>0&&results->get_subscription_count()>0;for(auto &p:images)subscribed=subscribed&&p.second->get_subscription_count()>0;
      if(record&&subscribed)break;
      if(std::chrono::steady_clock::now()>=ready){if(record)throw std::runtime_error("Recorder did not subscribe to all output topics within 30 seconds");break;}
      std::this_thread::sleep_for(std::chrono::milliseconds(20));}}
  std::ifstream rows(dir/"predictions.jsonl");if(!rows)throw std::runtime_error("Missing predictions.jsonl");std::string line;std::size_t count=0;
  while(std::getline(rows,line)){if(line.empty())continue;if(publish&&!rclcpp::ok())break;auto start=std::chrono::steady_clock::now();auto row=ab::j::JsonParser(line,"predictions.jsonl").parse();
    const auto maskpath=ab::resolve(dir,ab::str(row,"segmentation_mask"));const auto frame=maskpath.parent_path();auto left=cv::imread((frame/"left.png").u8string(),cv::IMREAD_COLOR);auto mask=cv::imread(maskpath.u8string(),cv::IMREAD_GRAYSCALE);
    const int w=ab::num(row,"width",2,4096),h=ab::num(row,"height",2,4096);if(left.cols!=w||left.rows!=h||mask.size()!=left.size())throw std::runtime_error("Saved image shape mismatch");
    cv::Mat q8(h,w,CV_16UC1);std::ifstream q(ab::resolve(dir,ab::str(row,"disparity_q8")),std::ios::binary);for(int y=0;y<h;++y)for(int x=0;x<w;++x){unsigned char bytes[2]={0,0};q.read(reinterpret_cast<char*>(bytes),2);q8.at<std::uint16_t>(y,x)=std::uint16_t(bytes[0])|(std::uint16_t(bytes[1])<<8);}if(!q||q.peek()!=EOF)throw std::runtime_error("Wrong saved Q8 byte count");
    auto detection=left.clone();for(const auto &b:ab::j::required(row,"detections").array){auto c=color(ab::num(b,"class_id"));int x1=int(ab::real(b,"x1")),y1=int(ab::real(b,"y1")),x2=int(ab::real(b,"x2")),y2=int(ab::real(b,"y2"));
      cv::rectangle(detection,{x1,y1},{x2,y2},cv::Scalar(c),2);std::ostringstream label;label<<ab::num(b,"class_id")<<' '<<std::fixed<<std::setprecision(2)<<ab::real(b,"score");cv::putText(detection,label.str(),{x1,std::max(12,y1-4)},cv::FONT_HERSHEY_SIMPLEX,.45,cv::Scalar(c),1);}
    cv::Mat rgbmask(h,w,CV_8UC3);for(int y=0;y<h;++y)for(int x=0;x<w;++x)rgbmask.at<cv::Vec3b>(y,x)=color(mask.at<std::uint8_t>(y,x));cv::Mat overlay;cv::addWeighted(left,.65,rgbmask,.35,0,overlay);
    cv::Mat scaled,disparity;q8.convertTo(scaled,CV_8U,255.0/(maximum*256));cv::applyColorMap(scaled,disparity,cv::COLORMAP_JET);disparity.setTo(cv::Scalar(0,0,0),q8==0);
    save(frame/"detection.png",detection);save(frame/"segmentation_color.png",rgbmask);save(frame/"segmentation_overlay.png",overlay);save(frame/"disparity_color.png",disparity);save(frame/"disparity_q8.png",q8);
    const auto &k=ab::j::required(row,"intrinsics").array;if(k.size()!=7)throw std::invalid_argument("Saved intrinsics require w,h,fx,fy,cx,cy,bxf");double fx=numeric(k[2]),fy=numeric(k[3]),cx=numeric(k[4]),cy=numeric(k[5]),bxf=numeric(k[6])/1000.;if(fx<=0||fy<=0||bxf<=0)throw std::invalid_argument("Invalid saved calibration");
    struct Point{float x,y,z;std::uint8_t r,g,b;};std::vector<Point> points;for(int y=0;y<h;y+=stride)for(int x=0;x<w;x+=stride){auto d=q8.at<std::uint16_t>(y,x);if(!d)continue;const double z=bxf/(d/256.);auto c=left.at<cv::Vec3b>(y,x);points.push_back({float((x-cx)*z/fx),float((y-cy)*z/fy),float(z),c[2],c[1],c[0]});}
    std::ofstream ply(frame/"camera_cloud.ply");ply<<"ply\nformat ascii 1.0\nelement vertex "<<points.size()<<"\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";for(auto &p:points)ply<<p.x<<' '<<p.y<<' '<<p.z<<' '<<unsigned(p.r)<<' '<<unsigned(p.g)<<' '<<unsigned(p.b)<<'\n';ply.close();if(!ply)throw std::runtime_error("Point cloud save failed");
    if(publish){std_msgs::msg::Header header;const auto stamp=ab::u64(ab::str(row,"timestamp_ns"));header.stamp.sec=std::int32_t(stamp/1000000000);header.stamp.nanosec=stamp%1000000000;header.frame_id="camera_optical_frame";
      auto send=[&](const char*name,const cv::Mat&m,const char*encoding){Image image;image.header=header;image.width=w;image.height=h;image.encoding=encoding;image.step=std::uint32_t(m.step);image.is_bigendian=false;image.data.assign(m.ptr(),m.ptr()+m.total()*m.elemSize());images[name]->publish(image);};
      send("left",left,"bgr8");send("detection",detection,"bgr8");send("segmentation",mask,"mono8");send("segmentation_overlay",overlay,"bgr8");send("disparity_color",disparity,"bgr8");send("disparity_q8",q8,"16UC1");
      Cloud cloud;cloud.header=header;cloud.height=1;cloud.width=std::uint32_t(points.size());cloud.is_dense=true;sensor_msgs::PointCloud2Modifier modifier(cloud);modifier.setPointCloud2FieldsByString(2,"xyz","rgb");modifier.resize(points.size());sensor_msgs::PointCloud2Iterator<float> px(cloud,"x"),py(cloud,"y"),pz(cloud,"z");sensor_msgs::PointCloud2Iterator<std::uint8_t> pr(cloud,"r"),pg(cloud,"g"),pb(cloud,"b");
      for(const auto &p:points){*px=p.x;*py=p.y;*pz=p.z;*pr=p.r;*pg=p.g;*pb=p.b;++px;++py;++pz;++pr;++pg;++pb;}clouds->publish(cloud);std_msgs::msg::String result;result.data=line;results->publish(result);rclcpp::spin_some(node);
      std::this_thread::sleep_until(start+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1/hz)));}
    ++count;
  }
  if(publish){auto end=std::chrono::steady_clock::now()+std::chrono::seconds(1);while(rclcpp::ok()&&std::chrono::steady_clock::now()<end){rclcpp::spin_some(node);std::this_thread::sleep_for(std::chrono::milliseconds(20));}rclcpp::shutdown();}
  if(!count)throw std::runtime_error("No saved predictions to visualize");std::cout<<"Saved visualizations for "<<count<<" frames in "<<dir<<'\n';return 0;
}catch(const std::exception&e){std::cerr<<"Visualization: "<<e.what()<<'\n';return 1;}}
