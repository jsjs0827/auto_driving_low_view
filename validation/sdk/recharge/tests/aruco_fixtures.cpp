#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

int main(int argc,char** argv) {
    try {
        if(argc!=2)throw std::runtime_error("usage: aruco_fixtures NEW_DIRECTORY");
        const std::filesystem::path dir(argv[1]);std::filesystem::create_directories(dir);
        auto dict=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        const char* names[]={"dual","offset","left","right","blank","wrong_id","dual_repeat","small_dual"};
        for(int i=0;i<8;++i){
            cv::Mat gray(1088,1280,CV_8UC1,cv::Scalar(255));
            const int size=i==7?56:98,sep=i==7?96:168;
            const int x=i==1?660:500,y=i==1?550:450;
            auto marker=[&](int id,int left){cv::Mat m;cv::aruco::drawMarker(dict,id,size,m);m.copyTo(gray(cv::Rect(left,y,size,size)));};
            if(i==5)marker(12,x);
            else if(i!=4){if(i!=3)marker(0,x);if(i!=2)marker(1,x+sep);}
            std::ofstream f(dir/(std::string(names[i])+".ppm"),std::ios::binary);
            f<<"P6\n1280 1088\n255\n";
            for(int row=0;row<gray.rows;++row)for(int col=0;col<gray.cols;++col){const char v=char(gray.at<unsigned char>(row,col));f.write(&v,1);f.write(&v,1);f.write(&v,1);}
            if(!f)throw std::runtime_error("fixture write failed");
        }
        std::cout<<"Generated 8 ArUco fixtures: dual/single/blank/wrong-id and scale/offset sequences\n";
        return 0;
    }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}
