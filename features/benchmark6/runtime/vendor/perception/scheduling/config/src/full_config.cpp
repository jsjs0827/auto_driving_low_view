#include "config.hpp"
#include "json.hpp"
#include "motion_settings_json.hpp"
namespace r33::full {
Config loadConfig(const std::string& file) {
    using namespace json;
    const auto root=load(file);fields(root,{"schema_version","mode","input","inference","mapping","recharge","output","history","graph","visual_motion","motion_reference","slip_detection"});
    if(integer(required(root,"schema_version"),1,1)!=1)throw std::invalid_argument("Unsupported full SDK JSON version");
    Config c;c.source=std::filesystem::absolute(std::filesystem::u8path(file)).u8string();
    if(auto input=optional(root,"input")) {
        fields(*input,{"disparity_source"});
        c.disparity_source=string(required(*input,"disparity_source"));
        if(c.disparity_source!="original_q8"&&c.disparity_source!="raft_stereo")
            throw std::invalid_argument("input.disparity_source must be original_q8 or raft_stereo");
    }
    auto path=[&](const JsonValue& v){auto p=std::filesystem::u8path(string(v));return (p.is_absolute()?p:std::filesystem::u8path(c.source).parent_path()/p).lexically_normal().u8string();};
    const auto mode=string(required(root,"mode"));
    if(mode=="map")c.command=1;else if(mode=="recharge")c.command=2;else if(mode=="map_and_recharge")c.command=3;else if(mode=="stopped")c.command=21;else throw std::invalid_argument("mode must be map/recharge/map_and_recharge/stopped");
    const auto& ai=required(root,"inference");fields(ai,{"enabled","backend","original_profile","dag","detection_dag","segmentation_dag","cpu_profile","detection_model","segmentation_model"});
    if(auto p=optional(ai,"enabled"))c.inference=boolean(*p);
    c.backend=string(required(ai,"backend"));if(c.backend!="bpu"&&c.backend!="cpu")throw std::invalid_argument("backend must be bpu or cpu");
    c.original_profile=path(required(ai,"original_profile"));c.ai_dag=path(required(ai,"dag"));
    if(auto p=optional(ai,"detection_dag"))c.detection_dag=path(*p);
    if(auto p=optional(ai,"segmentation_dag"))c.segmentation_dag=path(*p);
    if(auto p=optional(ai,"cpu_profile"))c.cpu_profile=path(*p);
    if(auto p=optional(ai,"detection_model"))c.det_model=path(*p);
    if(auto p=optional(ai,"segmentation_model"))c.seg_model=path(*p);
    if(c.inference&&c.backend=="cpu"&&c.cpu_profile.empty())throw std::invalid_argument("CPU inference requires explicit cpu_profile");
    const auto& map=required(root,"mapping");fields(map,{"enabled","original_config","extrinsics","sample_step","retain_clouds","render_maps","calibration_output"});
    c.mapping=boolean(required(map,"enabled"));
    if(auto p=optional(map,"original_config"))c.mapping_config=path(*p);
    if(auto p=optional(map,"extrinsics"))c.extrinsics=path(*p);
    if(auto p=optional(map,"sample_step"))c.sample_step=integer(*p,1,32);
    if(auto p=optional(map,"retain_clouds"))c.clouds=boolean(*p);
    if(auto p=optional(map,"render_maps"))c.render_maps=boolean(*p);
    if(auto p=optional(map,"calibration_output"))c.calibration_output=path(*p);
    if(c.mapping&&(c.mapping_config.empty()||c.extrinsics.empty()))throw std::invalid_argument("Mapping requires original_config and extrinsics");
    const auto& recharge=required(root,"recharge");fields(recharge,{"enabled","config","allow_source_fallback"});
    c.recharge=boolean(required(recharge,"enabled"));
    if(auto p=optional(recharge,"config"))c.recharge_config=path(*p);
    if(auto p=optional(recharge,"allow_source_fallback"))c.allow_recharge_source_fallback=boolean(*p);
    if(c.recharge&&c.recharge_config.empty())throw std::invalid_argument("Recharge requires config");
    if(auto p=optional(root,"output")) {
        fields(*p,{"directory","every_n","save_overlay","save_inputs"});
        if(auto v=optional(*p,"directory"))c.output_directory=path(*v);
        if(auto v=optional(*p,"every_n"))c.record_every_n=integer(*v,0,1000000);
        if(auto v=optional(*p,"save_overlay"))c.save_overlay=boolean(*v);
        if(auto v=optional(*p,"save_inputs"))c.save_inputs=boolean(*v);
    }
    if(auto h=optional(root,"history")) {
        fields(*h,{"capacity","frame_interval","max_memory_mb","difference","match","difference_threshold","match_iou_threshold","match_max_detections"});
        if(auto v=optional(*h,"capacity"))c.history.capacity=integer(*v,1,32);
        if(auto v=optional(*h,"frame_interval"))c.history.frame_interval=integer(*v,1,1000000);
        if(auto v=optional(*h,"max_memory_mb"))c.history.max_memory_mb=integer(*v,1,4096);
        if(auto v=optional(*h,"difference"))c.history.difference=boolean(*v);
        if(auto v=optional(*h,"match"))c.history.match=boolean(*v);
        if(auto v=optional(*h,"difference_threshold"))c.history.difference_threshold=integer(*v,0,255);
        if(auto v=optional(*h,"match_max_detections"))c.history.match_max_detections=integer(*v,1,1024);
        if(auto v=optional(*h,"match_iou_threshold")) {
            if(v->kind!=JsonValue::Kind::Number)throw std::invalid_argument("match_iou_threshold must be numeric");
            c.history.match_iou_threshold=std::stod(v->text);
        }
        r36::temporal::History validate(c.history);
    }
    if(auto graph=optional(root,"graph")) {
        c.graph=parseProductGraph(*graph);
        const bool inferred=c.graph.enabled("dnn")||c.graph.enabled("det")||c.graph.enabled("seg");
        if(c.inference!=inferred)throw std::invalid_argument("inference.enabled must match graph inference features");
        if(c.graph.enabled("det")&&c.detection_dag.empty())throw std::invalid_argument("det inference requires inference.detection_dag");
        if(c.graph.enabled("seg")&&c.segmentation_dag.empty())throw std::invalid_argument("seg inference requires inference.segmentation_dag");
        if(c.mapping!=c.graph.enabled("cloud"))throw std::invalid_argument("mapping.enabled must match graph cloud feature");
        if(c.recharge!=c.graph.enabled("recharge"))throw std::invalid_argument("recharge.enabled must match graph recharge feature");
        if(c.history.difference!=c.graph.enabled("temporal_difference"))throw std::invalid_argument("history.difference must match graph temporal_difference feature");
        if(c.history.match!=c.graph.enabled("temporal_match"))throw std::invalid_argument("history.match must match graph temporal_match feature");
        if(c.record_every_n&&!c.graph.enabled("record"))throw std::invalid_argument("SDK recording requires an enabled graph record node");
    }
    if(auto p=optional(root,"visual_motion")){c.visual_motion=visualSettingsFromJson(*p);c.visual_motion_json=motionJson(*p);}
    if(auto p=optional(root,"motion_reference")){
        c.motion_reference=referenceSettingsFromJson(*p);c.motion_reference_json=motionJson(*p);
        if(auto csv=optional(*p,"input_csv"))if(csv->kind!=JsonValue::Kind::Null)c.motion_reference_csv=path(*csv);
        if(auto domain=optional(*p,"clock_domain"))c.motion_clock_domain=domain->text;
        if(auto confirmed=optional(*p,"same_clock_confirmed"))c.motion_same_clock_confirmed=boolean(*confirmed);
        if(!c.motion_reference_csv.empty()&&(!c.motion_same_clock_confirmed||c.motion_clock_domain.empty()))
            throw std::invalid_argument("Motion reference CSV requires confirmed shared clock_domain");
    }
    if(auto p=optional(root,"slip_detection")){c.slip_detection=slipSettingsFromJson(*p);c.slip_detection_json=motionJson(*p);}
    c.visual_motion_enabled=c.graph.configured&&c.graph.enabled("visual_motion");
    c.motion_reference_enabled=c.graph.configured&&c.graph.enabled("motion_reference");
    c.slip_detection_enabled=c.graph.configured&&c.graph.enabled("slip_detection");
    if(c.visual_motion_enabled&&c.history.capacity<2)throw std::invalid_argument("visual_motion requires history.capacity >= 2");
    if((c.history.difference||c.history.match)&&c.history.capacity<2)
        throw std::invalid_argument("Temporal difference/match require history.capacity >= 2");
    if(c.record_every_n&&!c.output_directory.size())throw std::invalid_argument("record every_n requires output.directory");
    if((c.command==1||c.command==3)&&!c.mapping) {
        // Map work may explicitly be inference-only (useful for the r32 ABI
        // regression). Its result clearly reports zero maps, never placeholders.
        if(!c.inference&&!(c.graph.configured&&(c.graph.enabled("rgb")||c.graph.enabled("disparity"))))throw std::invalid_argument("Map mode has no enabled work");
    }
    if((c.command==2||c.command==3)&&!c.recharge)throw std::invalid_argument("Recharge mode requires recharge.enabled=true");
    return c;
}
std::string effectiveJson(const Config& c) {
    using json::quote;
    std::ostringstream o;o<<"{\"revision\":\"r12-v36-temporal\",\"source\":"<<quote(c.source)
      <<",\"backend\":"<<quote(c.backend)<<",\"command\":"<<c.command
      <<",\"disparity_source\":"<<quote(c.disparity_source)
      <<",\"inference_enabled\":"<<(c.inference?"true":"false")<<",\"mapping_enabled\":"<<(c.mapping?"true":"false")
      <<",\"recharge_enabled\":"<<(c.recharge?"true":"false")
      <<",\"visual_motion_enabled\":"<<(c.visual_motion_enabled?"true":"false")
      <<",\"motion_reference_enabled\":"<<(c.motion_reference_enabled?"true":"false")
      <<",\"slip_detection_enabled\":"<<(c.slip_detection_enabled?"true":"false")
      <<",\"visual_motion_settings\":"<<c.visual_motion_json<<",\"motion_reference_settings\":"<<c.motion_reference_json
      <<",\"slip_detection_settings\":"<<c.slip_detection_json
      <<",\"user_graph_configured\":"<<(c.graph.configured?"true":"false")
      <<",\"original_profile\":"<<quote(c.original_profile)<<",\"ai_dag\":"<<quote(c.ai_dag)
      <<",\"detection_dag\":"<<quote(c.detection_dag)<<",\"segmentation_dag\":"<<quote(c.segmentation_dag)
      <<",\"detection_model_override\":"<<quote(c.det_model)<<",\"segmentation_model_override\":"<<quote(c.seg_model)
      <<",\"mapping_config\":"<<quote(c.mapping_config)<<",\"extrinsics\":"<<quote(c.extrinsics)
      <<",\"recharge_config\":"<<quote(c.recharge_config)<<",\"record_every_n\":"<<c.record_every_n
      <<",\"history\":{\"capacity\":"<<c.history.capacity<<",\"frame_interval\":"<<(c.history.capacity==1?1:c.history.frame_interval)<<",\"max_memory_mb\":"<<c.history.max_memory_mb<<",\"difference\":"<<(c.history.difference?"true":"false")<<",\"match\":"<<(c.history.match?"true":"false")<<"}"
      <<",\"double_buffer\":false,\"original_input_default\":\"640x544 NV12 RGBD-left\"}";return o.str();
}
}
