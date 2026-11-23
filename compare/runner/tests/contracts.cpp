#include "compare_util.hpp"
#include "replay_passes.hpp"
#include <iostream>
#include <limits>

int main() {
  try {
    auto check = [](bool condition, const char* message) { if (!condition) throw std::runtime_error(message); };
    // Simulate a retained warmup frame and two full dataset passes. Frame zero
    // in either measured pass must see an empty window, while frame one keeps
    // the history produced by frame zero in that same pass.
    std::vector<int> history{99}, observed_history;
    int history_resets=0;
    r36compare::runMeasuredPasses(2,2,true,[&]{history.clear();++history_resets;},
      [&](std::size_t frame,int repeat,bool warmup){
        check(!warmup,"Measured pass must not be reported as warmup");
        observed_history.push_back(int(history.size()));history.push_back(int(frame)+repeat*2);
      });
    check(history_resets==2&&observed_history==std::vector<int>({0,1,0,1}),
          "History is isolated from warmup and previous repeats, but retained within each pass");
    history={99};observed_history.clear();history_resets=0;
    r36compare::runMeasuredPasses(2,2,false,[&]{history.clear();++history_resets;},
      [&](std::size_t frame,int,bool){observed_history.push_back(int(history.size()));history.push_back(int(frame));});
    check(history_resets==0&&observed_history==std::vector<int>({1,2,3,4}),
          "Disabled history must not call the SDK history-reset operation");
    int after_failed_reset=0;bool reset_failure=false;
    try {
      r36compare::runMeasuredPasses(2,1,true,[]{throw std::runtime_error("reset failed");},
        [&](std::size_t,int,bool){++after_failed_reset;});
    }catch(const std::runtime_error&){reset_failure=true;}
    check(reset_failure&&after_failed_reset==0,"Failed history reset aborts before measuring frames");
    check(r36compare::tsv("a\tleft\tbgr\tright\tq8").size() == 5, "Manifest five fields");
    check(r36compare::tsv("a\tleft_nv12\tleft_png\tright_png\tq8\tright_nv12").size()==6,
          "Direct NV12 manifest retains genuine right image as sixth column");
    for (const auto& bad : {"a\tb\tc\td", "../a\tb\tc\td\te", "a\t\tc\td\te", "a\tb\tc\td\te\r"}) {
      bool failed = false; try { r36compare::tsv(bad); } catch (const std::invalid_argument&) { failed = true; }
      check(failed, "Reject malformed manifest");
    }
    check(r36compare::quote("a\n\"\\") == "\"a\\u000a\\\"\\\\\"", "JSON escaping");
    check(r36compare::decodeQ8({0, 1, 255, 255, 1, 0}) == std::vector<std::uint16_t>({256, 65535, 1}), "Q8 little endian");
    cv::Mat d(1, 8, CV_32FC1);
    const float source[] = {1.f, 0.f, -1.f, std::numeric_limits<float>::quiet_NaN(), 256.f, .5f / 256.f, 1.5f / 256.f, 65535.f / 256.f};
    std::copy(std::begin(source), std::end(source), d.ptr<float>());
    check(r36compare::normalizeQ8(d, d.size()) == std::vector<std::uint16_t>({256, 0, 0, 0, 0, 1, 2, 65535}), "Q8 rounding and invalid preservation");
    cv::Mat low(1, 2, CV_32FC1, cv::Scalar(4));
    check(r36compare::normalizeQ8(low, {4, 2}) == std::vector<std::uint16_t>(8, 2048), "Horizontal disparity scale");
    cv::Mat hole(1, 2, CV_32FC1); hole.at<float>(0, 0) = 4; hole.at<float>(0, 1) = 0;
    check(r36compare::normalizeQ8(hole, {4, 1}) == std::vector<std::uint16_t>({2048, 2048, 0, 0}), "Invalid neighbors must not dilute/create disparity");
    auto rejects = [&](const auto& operation, const char* message) {
      bool failed = false; try { operation(); } catch (const std::invalid_argument&) { failed = true; }
      check(failed, message);
    };
    const cv::Size recording(640, 544), model352(640, 352);
    rejects([&] { r36compare::stereoInputAdaptation("strict", recording, model352); }, "Default strict mode must reject a differently sized real model");
    rejects([&] { r36compare::stereoInputAdaptation("letterbox", recording, model352); }, "Unsupported image adaptation mode");
    rejects([&] { r36compare::stereoInputAdaptation("resize", recording, model352, {640, 0}); }, "Both assertion dimensions are required");
    rejects([&] { r36compare::stereoInputAdaptation("resize", recording, model352, {640, 544}); }, "Metadata assertion must not override actual model shape");
    rejects([&] { r36compare::stereoInputAdaptation("resize", recording, model352, {-1, 352}); }, "Negative model metadata assertion");
    rejects([&] { r36compare::stereoInputAdaptation("resize", recording, model352, {641, 352}); }, "Odd model metadata assertion is rejected consistently with JSON");
    rejects([&] { r36compare::stereoInputAdaptation("resize", recording, model352, {640, 1}); }, "One-pixel model metadata assertion is rejected consistently with JSON");
    const auto adapt352 = r36compare::stereoInputAdaptation("resize", recording, model352, model352);
    check(adapt352.scale_x == 1 && std::abs(adapt352.scale_y - 352.0 / 544) < 1e-15 && adapt352.disparity_x_scale == 1,
          "Vertical-only input resize must not scale horizontal disparity");
    const auto metadata = r36compare::stereoInputAdaptationJson(adapt352, {640, 176});
    check(metadata.find("\"model_input_size\":{\"width\":640,\"height\":352}") != std::string::npos &&
          metadata.find("\"output_size\":{\"width\":640,\"height\":176}") != std::string::npos &&
          metadata.find("\"restored_q8_size\":{\"width\":640,\"height\":544}") != std::string::npos,
          "Actual model input, actual output and restored source dimensions need separate provenance");
    const auto crop352=r36compare::stereoInputAdaptation("crop",recording,model352,model352);
    check(crop352.source_roi==cv::Rect(0,192,640,352) && crop352.scale_x==1 && crop352.scale_y==1 &&
          crop352.disparity_x_scale==1,"Bottom crop keeps source pixel scale in both axes");
    const auto crop_metadata=r36compare::stereoInputAdaptationJson(crop352,model352);
    check(crop_metadata.find("\"source_roi\":{\"x\":0,\"y\":192,\"width\":640,\"height\":352}")!=std::string::npos &&
          crop_metadata.find("\"crop_applied\":true")!=std::string::npos &&
          crop_metadata.find("\"resize_applied\":false")!=std::string::npos &&
          crop_metadata.find("\"resize_interpolation\":null")!=std::string::npos,
          "Crop provenance must identify exact source ROI and absence of interpolation");
    rejects([&]{r36compare::stereoInputAdaptation("crop",recording,{320,352});},"Crop cannot silently scale source width");
    rejects([&]{r36compare::stereoInputAdaptation("crop",{640,350},model352);},"Crop cannot upscale a shorter source");
    rejects([&]{r36compare::stereoInputAdaptation("crop",{640,543},model352);},"Crop requires even NV12 source height");
    rejects([&]{r36compare::stereoInputAdaptation("crop",recording,{640,351});},"Crop requires even NV12 model height");
    cv::Mat crop_plane(model352,CV_32FC1,cv::Scalar(8));
    crop_plane.at<float>(0,0)=std::numeric_limits<float>::quiet_NaN();
    crop_plane.at<float>(0,1)=0;crop_plane.at<float>(0,2)=256;
    crop_plane.at<float>(0,3)=.5f/256;
    const auto crop_q8=r36compare::restoreStereoQ8(crop_plane,crop352);
    check(crop_q8.size()==640*544 &&
          std::all_of(crop_q8.begin(),crop_q8.begin()+192*640,[](auto q){return q==0;}) &&
          crop_q8[192*640]==0 && crop_q8[192*640+1]==0 && crop_q8[192*640+2]==0 &&
          crop_q8[192*640+3]==1 && crop_q8[192*640+4]==2048 && crop_q8.back()==2048,
          "Native crop Q8 writes only corresponding ROI, preserves invalids and never multiplies disparity");
    rejects([&]{r36compare::restoreStereoQ8(cv::Mat(176,640,CV_32FC1,cv::Scalar(8)),crop352);},
            "Crop cannot silently stretch a lower-resolution model output");
    rejects([&]{r36compare::restoreStereoQ8(cv::Mat(352,320,CV_32FC1,cv::Scalar(4)),crop352);},
            "Crop cannot silently scale output width/disparity");
    auto malformed_crop=crop352;malformed_crop.source_roi.y=190;
    rejects([&]{r36compare::restoreStereoQ8(crop_plane,malformed_crop);},"Public crop ROI must be revalidated before writing");
    malformed_crop=crop352;malformed_crop.disparity_x_scale=2;
    rejects([&]{r36compare::restoreStereoQ8(crop_plane,malformed_crop);},"Crop output cannot carry a disparity scale");

    cv::Mat raw_left(12,4,CV_8UC1),raw_right(12,4,CV_8UC1);
    for(int y=0;y<12;++y)for(int x=0;x<4;++x) {
      raw_left.ptr(y)[x]=std::uint8_t(10*y+x);raw_right.ptr(y)[x]=std::uint8_t(100+10*y+x);
    }
    const auto small_crop=r36compare::stereoInputAdaptation("crop",{4,8},{4,4});
    const auto raw_crop=r36compare::prepareStereoNv12(raw_left,raw_right,small_crop);
    check(raw_crop.left.ptr(0)[0]==40 && raw_crop.left.ptr(3)[3]==73 &&
          raw_crop.left.ptr(4)[0]==100 && raw_crop.left.ptr(5)[3]==113 &&
          raw_crop.right.ptr(0)[0]==140 && raw_crop.right.ptr(4)[0]==200,
          "Both NV12 sides crop matching bottom Y rows and half-offset UV rows");
    malformed_crop=small_crop;malformed_crop.source_roi={0,2,4,6};
    rejects([&]{r36compare::prepareStereoNv12(raw_left,raw_right,malformed_crop);},
            "Preparation rejects a bottom ROI differing from model native dimensions");

    cv::Mat left(4, 4, CV_8UC3), right(4, 4, CV_8UC3);
    for (int y = 0; y < 4; ++y) for (int x = 0; x < 4; ++x) {
      const auto value = std::uint8_t(20 * y + 4 * x);
      left.at<cv::Vec3b>(y, x) = {value, std::uint8_t(value + 1), std::uint8_t(value + 2)};
      right.at<cv::Vec3b>(y, x) = {std::uint8_t(value + 100), std::uint8_t(value + 101), std::uint8_t(value + 102)};
    }
    const auto source_left = left.clone(), source_right = right.clone();
    const auto small = r36compare::stereoInputAdaptation("resize", {4, 4}, {2, 2});
    const auto resized = r36compare::prepareStereoBgr(left, right, small);
    check(resized.left.size() == cv::Size(2, 2) && resized.right.size() == cv::Size(2, 2) &&
          resized.left.at<cv::Vec3b>(0, 0) == cv::Vec3b(12, 13, 14) && resized.left.at<cv::Vec3b>(1, 1) == cv::Vec3b(60, 61, 62) &&
          resized.right.at<cv::Vec3b>(0, 0) == cv::Vec3b(112, 113, 114), "Both stereo sides preserve BGR and use linear interpolation");
    check(cv::norm(left, source_left, cv::NORM_INF) == 0 && cv::norm(right, source_right, cv::NORM_INF) == 0,
          "Input adaptation must not modify either original source image");
    const auto bgr_crop=r36compare::prepareStereoBgr(left,right,r36compare::stereoInputAdaptation("crop",{4,4},{4,2}));
    check(bgr_crop.left.size()==cv::Size(4,2) && bgr_crop.left.at<cv::Vec3b>(0,0)==left.at<cv::Vec3b>(2,0) &&
          bgr_crop.right.at<cv::Vec3b>(1,3)==right.at<cv::Vec3b>(3,3),"Optional BGR crop preserves corresponding source samples");
    const auto identical = r36compare::prepareStereoBgr(left, right, r36compare::stereoInputAdaptation("strict", {4, 4}, {4, 4}));
    check(identical.left.data == left.data && identical.right.data == right.data, "Strict identical sizes need no resize or copy");
    rejects([&] { r36compare::prepareStereoBgr(left, cv::Mat(), small); }, "Missing right view must fail before inference");
    rejects([&] { r36compare::prepareStereoBgr(left, cv::Mat(4, 4, CV_8UC1), small); }, "Wrong right image type must fail");
    rejects([&] { r36compare::prepareStereoBgr(left, cv::Mat(2, 2, CV_8UC3), small); }, "Right source dimensions cannot be silently reinterpreted");

    // A plane at Z=4.5m has d=fx*baseline_mm/(Z*1000)=8px for
    // fx=600px and baseline=60mm. Compressing only image height cannot change d.
    const double bxf_pixel_mm = 600.0 * 60;
    // A cropped output pixel must project to the original source pixel. Using
    // local ROI coordinates is equivalent only when cy is shifted by ROI.y.
    const double fx=600,fy=601,cx=319.5,cy=271.5,local_x=320,local_y=80;
    const double cropped_z=bxf_pixel_mm/(crop_q8[std::size_t(192+80)*640+320]/256.0)/1000;
    const double source_y=local_y+crop352.source_roi.y;
    const double y_source=(source_y-cy)*cropped_z/fy;
    const double y_roi=(local_y-(cy-crop352.source_roi.y))*cropped_z/fy;
    const double x_source=(local_x-cx)*cropped_z/fx;
    check(std::abs(cropped_z-4.5)<1e-12 && std::abs(y_source-y_roi)<1e-12 &&
          std::abs(x_source-.00375)<1e-12,"Crop ROI offset with unchanged source intrinsics yields the same 3D point as shifted cropped cy");
    for (const auto raw_size : {model352, cv::Size(640, 176)}) {
      cv::Mat plane(raw_size, CV_32FC1, cv::Scalar(8));
      const auto restored = r36compare::normalizeQ8(plane, recording);
      check(restored == std::vector<std::uint16_t>(640 * 544, 2048), "352/176-height model outputs restore source geometry without a height disparity multiplier");
      const double z_m = bxf_pixel_mm / (double(restored[272 * 640 + 320]) / 256) / 1000;
      check(std::abs(z_m - 4.5) < 1e-12, "Unchanged source intrinsics and bxf must recover the same depth after vertical resize");
    }
    // Halving input width halves predicted disparity; restore by width only,
    // even when model/output heights differ by unrelated factors.
    const auto narrow = r36compare::stereoInputAdaptation("resize", recording, {320, 352});
    check(narrow.disparity_x_scale == 2 && narrow.scale_x == .5, "Horizontal resize provenance is reciprocal");
    cv::Mat narrow_plane(88, 320, CV_32FC1, cv::Scalar(4));
    check(r36compare::normalizeQ8(narrow_plane, recording) == std::vector<std::uint16_t>(640 * 544, 2048),
          "Horizontal source-to-model disparity scaling must recover the same 3D plane independently of output height");
    std::cout << "Comparison manifest, JSON, stereo crop/resize metadata and source-plane geometry contracts passed\n";
    return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
