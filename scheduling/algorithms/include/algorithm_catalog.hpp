#pragma once
#include "json.hpp"
#include <filesystem>
#include <map>
#include <string>

namespace perception::algorithms {
using Json = r33::json::JsonValue;
struct Descriptor {
    std::string id, feature;
    Json schema;
    std::map<std::string, std::string> inputs;
};
// Generated from features/*/algorithm.json. No model is loaded by this catalog.
const std::map<std::string, Descriptor>& catalog();
const Descriptor* find(const std::string& id_or_feature);
std::string nodeAlgorithm(const Json& node);
Json normalizeParameters(const Descriptor&, const Json&, const std::filesystem::path& base,
                         bool enabled = true);
Json compileConfigurations(const Json& graph, const Json& feature_settings,
                           const std::filesystem::path& base);
Json validateConfigurations(const Json& graph, const Json& configurations,
                            const std::filesystem::path& base);
bool hasWork(const Json& configurations);
std::string jsonText(const Json&);
}
