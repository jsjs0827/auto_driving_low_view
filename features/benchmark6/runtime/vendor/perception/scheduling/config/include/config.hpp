#pragma once
#include "perception_sdk.h"
#include "perception_temporal.hpp"
#include "product_graph.hpp"
#include "motion_types.hpp"
#include <string>
namespace r33::full {
struct Config {
    std::string source, backend="bpu", original_profile, ai_dag, cpu_profile, det_model, seg_model;
    std::string detection_dag, segmentation_dag;
    std::string disparity_source="auto"; // Only legacy SDK JSON may omit the selector.
    std::string mapping_config, extrinsics, calibration_output, recharge_config, output_directory;
    bool inference=true, mapping=true, recharge=true, clouds=false, render_maps=true;
    bool save_overlay=true, save_inputs=false, allow_recharge_source_fallback=false;
    int command=3, sample_step=4, record_every_n=0;
    r36::temporal::Settings history;
    perception::motion::VisualSettings visual_motion;
    perception::motion::ReferenceSettings motion_reference;
    perception::motion::SlipSettings slip_detection;
    bool visual_motion_enabled=false,motion_reference_enabled=false,slip_detection_enabled=false;
    bool motion_same_clock_confirmed=false;
    std::string visual_motion_json="{}",motion_reference_json="{}",slip_detection_json="{}";
    std::string motion_reference_csv,motion_clock_domain;
    ProductGraphPlan graph;
};
Config loadConfig(const std::string&);
std::string effectiveJson(const Config&);
}
