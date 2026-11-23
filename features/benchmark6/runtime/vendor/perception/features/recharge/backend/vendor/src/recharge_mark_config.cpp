#include "recharge_mark_config.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

#include <rapidjson/document.h>

namespace recharge_cfg {
namespace {

bool JsonBool(const rapidjson::Value& obj, const char* key, bool def) {
    if (!obj.HasMember(key) || !obj[key].IsBool()) return def;
    return obj[key].GetBool();
}

double JsonNum(const rapidjson::Value& obj, const char* key, double def) {
    if (!obj.HasMember(key)) return def;
    if (obj[key].IsDouble()) return obj[key].GetDouble();
    if (obj[key].IsInt()) return static_cast<double>(obj[key].GetInt());
    if (obj[key].IsUint()) return static_cast<double>(obj[key].GetUint());
    return def;
}

int JsonInt(const rapidjson::Value& obj, const char* key, int def) {
    if (!obj.HasMember(key)) return def;
    if (obj[key].IsInt()) return obj[key].GetInt();
    if (obj[key].IsUint()) return static_cast<int>(obj[key].GetUint());
    return def;
}

uint32_t JsonU32(const rapidjson::Value& obj, const char* key, uint32_t def) {
    if (!obj.HasMember(key)) return def;
    if (obj[key].IsUint()) return obj[key].GetUint();
    if (obj[key].IsInt() && obj[key].GetInt() >= 0) {
        return static_cast<uint32_t>(obj[key].GetInt());
    }
    return def;
}

void CopyIntrinsics(char* dst, size_t n, const char* src) {
    if (!dst || n == 0) return;
    const char* s = src ? src : "dataset";
    std::snprintf(dst, n, "%s", s);
}

void ParseObject(const rapidjson::Value& rm, Parsed* out) {
    auto& h = out->host;
    h.publish_enabled = JsonBool(rm, "publish_enabled", h.publish_enabled);
    h.run_every_n = std::max(1u, JsonU32(rm, "run_every_n", h.run_every_n));
    h.min_interval_ms = JsonU32(rm, "min_interval_ms", h.min_interval_ms);
    h.use_hires = JsonBool(rm, "use_hires", h.use_hires);
    h.publish_overlay = JsonBool(rm, "publish_overlay", h.publish_overlay);
    h.overlay_max_width =
        JsonInt(rm, "overlay_max_width", h.overlay_max_width);
    if (rm.HasMember("intrinsics_source") && rm["intrinsics_source"].IsString()) {
        CopyIntrinsics(h.intrinsics_source, sizeof(h.intrinsics_source),
                       rm["intrinsics_source"].GetString());
    }

    if (rm.HasMember("roi") && rm["roi"].IsObject()) {
        const auto& roi = rm["roi"];
        out->roi_enabled = JsonBool(roi, "enabled", out->roi_enabled);
        out->roi_x_min = JsonNum(roi, "x_min", out->roi_x_min);
        out->roi_x_max = JsonNum(roi, "x_max", out->roi_x_max);
        out->roi_y_min = JsonNum(roi, "y_min", out->roi_y_min);
        out->roi_y_max = JsonNum(roi, "y_max", out->roi_y_max);
    }
    if (rm.HasMember("checks") && rm["checks"].IsObject()) {
        const auto& ck = rm["checks"];
        out->check_aspect = JsonBool(ck, "aspect_ratio", out->check_aspect);
        out->check_sym = JsonBool(ck, "strict_symmetry", out->check_sym);
        out->check_area = JsonBool(ck, "area_perimeter", out->check_area);
    }
    if (rm.HasMember("pose_refine") && rm["pose_refine"].IsObject()) {
        const auto& pr = rm["pose_refine"];
        out->pose_refine = JsonBool(pr, "enabled", out->pose_refine);
        out->camera_height_mm =
            JsonNum(pr, "camera_height_mm", out->camera_height_mm);
        out->mark_height_mm =
            JsonNum(pr, "mark_height_mm", out->mark_height_mm);
        out->camera_pitch_deg =
            JsonNum(pr, "camera_pitch_deg", out->camera_pitch_deg);
    }
    if (rm.HasMember("save_raw") && rm["save_raw"].IsObject()) {
        const auto& sr = rm["save_raw"];
        out->save_raw = JsonBool(sr, "enabled", out->save_raw);
        if (sr.HasMember("dir") && sr["dir"].IsString()) {
            out->save_raw_dir = sr["dir"].GetString();
        }
        out->save_every_n = std::max(1u, JsonU32(sr, "every_n", out->save_every_n));
        out->save_max = JsonU32(sr, "max_count", out->save_max);
    }
    if (rm.HasMember("aruco") && rm["aruco"].IsObject()) {
        const auto& a = rm["aruco"];
        out->aruco_dictionary_id = JsonInt(a, "dictionary_id", out->aruco_dictionary_id);
        out->aruco_target_id = JsonInt(a, "marker_id", out->aruco_target_id);
        out->aruco_dual_marker = JsonBool(a, "dual_marker", out->aruco_dual_marker);
        if (a.HasMember("layout") && a["layout"].IsString()) {
            std::string lay = a["layout"].GetString();
            for (char& c : lay) {
                c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            }
            out->aruco_layout_horizontal = (lay != "vertical");
        }
        out->aruco_id_left = JsonInt(a, "marker_id_left", out->aruco_id_left);
        out->aruco_id_right = JsonInt(a, "marker_id_right", out->aruco_id_right);
        out->aruco_id_top = JsonInt(a, "marker_id_top", out->aruco_id_top);
        out->aruco_id_bottom = JsonInt(a, "marker_id_bottom", out->aruco_id_bottom);
        out->aruco_marker_size_m = JsonNum(a, "marker_size_m", out->aruco_marker_size_m);
        out->aruco_gap_m = JsonNum(a, "gap_m", out->aruco_gap_m);
        out->aruco_margin_m = JsonNum(a, "margin_m", out->aruco_margin_m);
        out->aruco_card_w_m = JsonNum(a, "card_w_m", out->aruco_card_w_m);
        out->aruco_card_h_m = JsonNum(a, "card_h_m", out->aruco_card_h_m);
        out->aruco_max_reproj_px = JsonNum(a, "max_reproj_px", out->aruco_max_reproj_px);
        out->aruco_enable_lm_refine =
            JsonBool(a, "enable_lm_refine", out->aruco_enable_lm_refine);
        out->aruco_thresh_win_min =
            JsonInt(a, "adaptive_thresh_win_size_min", out->aruco_thresh_win_min);
        out->aruco_thresh_win_max =
            JsonInt(a, "adaptive_thresh_win_size_max", out->aruco_thresh_win_max);
        out->aruco_thresh_win_step =
            JsonInt(a, "adaptive_thresh_win_size_step", out->aruco_thresh_win_step);
        out->aruco_min_perimeter_rate =
            JsonNum(a, "min_marker_perimeter_rate", out->aruco_min_perimeter_rate);
        out->aruco_max_perimeter_rate =
            JsonNum(a, "max_marker_perimeter_rate", out->aruco_max_perimeter_rate);
        out->aruco_poly_approx_rate =
            JsonNum(a, "polygonal_approx_accuracy_rate", out->aruco_poly_approx_rate);
        out->aruco_min_corner_dist_rate =
            JsonNum(a, "min_corner_distance_rate", out->aruco_min_corner_dist_rate);
        out->aruco_min_border =
            JsonInt(a, "min_distance_to_border", out->aruco_min_border);
        out->aruco_pixel_per_cell =
            JsonInt(a, "perspective_remove_pixel_per_cell", out->aruco_pixel_per_cell);
        out->aruco_refine_method =
            JsonInt(a, "corner_refinement_method", out->aruco_refine_method);
        out->aruco_refine_win =
            JsonInt(a, "corner_refinement_win_size", out->aruco_refine_win);
        out->aruco_refine_iters =
            JsonInt(a, "corner_refinement_max_iterations", out->aruco_refine_iters);
        out->aruco_refine_min_acc =
            JsonNum(a, "corner_refinement_min_accuracy", out->aruco_refine_min_acc);
    }
}

}  // namespace

bool LoadFile(const char* path, Parsed* out, std::string* err) {
    if (!out) return false;
    *out = Parsed{};
    if (!path || !path[0]) {
        if (err) *err = "empty recharge config path";
        return false;
    }
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open()) {
        if (err) *err = std::string("open failed: ") + path;
        return false;
    }
    std::stringstream ss;
    ss << ifs.rdbuf();
    rapidjson::Document doc;
    doc.Parse(ss.str().c_str());
    if (doc.HasParseError() || !doc.IsObject()) {
        if (err) *err = std::string("json parse failed: ") + path;
        return false;
    }
    const rapidjson::Value* obj = &doc;
    if (doc.HasMember("recharge_mark") && doc["recharge_mark"].IsObject()) {
        const auto& rm = doc["recharge_mark"];
        const bool pointer_only =
            rm.HasMember("config_file") && rm["config_file"].IsString() &&
            !rm.HasMember("enabled") && !rm.HasMember("roi") &&
            !rm.HasMember("aruco");
        if (pointer_only) {
            if (err) {
                *err = std::string(
                           "got start_config pointer, need recharge_mark.json: ") +
                       path;
            }
            return false;
        }
        obj = &rm;
    }
    ParseObject(*obj, out);
    return true;
}

namespace {

std::string DirnameOf(const std::string& path) {
    const auto pos = path.find_last_of('/');
    if (pos == std::string::npos) return ".";
    if (pos == 0) return "/";
    return path.substr(0, pos);
}

std::string JoinPath(const std::string& dir, const std::string& rel) {
    if (rel.empty()) return {};
    if (rel[0] == '/') return rel;
    if (dir.empty() || dir == ".") return rel;
    if (dir.back() == '/') return dir + rel;
    return dir + "/" + rel;
}

bool ReadJsonObjectFile(const char* path, rapidjson::Document* doc,
                        std::string* err) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open()) {
        if (err) *err = std::string("open failed: ") + path;
        return false;
    }
    std::stringstream ss;
    ss << ifs.rdbuf();
    doc->Parse(ss.str().c_str());
    if (doc->HasParseError() || !doc->IsObject()) {
        if (err) *err = std::string("json parse failed: ") + path;
        return false;
    }
    return true;
}

bool IsMarkConfigObject(const rapidjson::Value& o) {
    return o.HasMember("enabled") || o.HasMember("publish_enabled") ||
           o.HasMember("roi") || o.HasMember("aruco");
}

}  // namespace

std::string ResolveMarkJsonPath(const char* start_or_cfg, std::string* err) {
    constexpr const char* kDefaultStart =
        "/app/daye/config/ai_node/start_config.json";
    constexpr const char* kDefaultMark =
        "/app/daye/config/ai_node/recharge_mark.json";
    const bool explicit_in = start_or_cfg && start_or_cfg[0];
    const char* in = explicit_in ? start_or_cfg : kDefaultStart;

    rapidjson::Document doc;
    std::string read_err;
    if (!ReadJsonObjectFile(in, &doc, &read_err)) {
        if (explicit_in) {
            const std::string s = in;
            const bool start_name =
                s.size() >= 16 &&
                s.compare(s.size() - 16, 16, "start_config.json") == 0;
            if (!start_name) {
                if (err) *err = read_err;
                return s;
            }
        }
        if (err) err->clear();
        return kDefaultMark;
    }

    if (doc.HasMember("recharge_mark") && doc["recharge_mark"].IsObject()) {
        const auto& rm = doc["recharge_mark"];
        if (rm.HasMember("config_file") && rm["config_file"].IsString() &&
            !IsMarkConfigObject(rm)) {
            if (err) err->clear();
            return JoinPath(DirnameOf(in), rm["config_file"].GetString());
        }
        if (IsMarkConfigObject(rm) || IsMarkConfigObject(doc)) {
            if (err) err->clear();
            return in;
        }
    }
    if (IsMarkConfigObject(doc)) {
        if (err) err->clear();
        return in;
    }
    if (doc.HasMember("sdk_so_dir") || doc.HasMember("dnn") ||
        doc.HasMember("map")) {
        if (err) err->clear();
        return JoinPath(DirnameOf(in), "recharge_mark.json");
    }
    if (err) err->clear();
    return in;
}

}  // namespace recharge_cfg
