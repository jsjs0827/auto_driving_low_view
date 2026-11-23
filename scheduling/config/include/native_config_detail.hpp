#pragma once
#include "native_pipeline.hpp"
#include <initializer_list>
#include <set>
namespace v3::detail {
Json object();
Json array();
Json str(const std::string &value);
Json num(double value);
Json integer(int value);
Json boolean(bool value);
bool has(const Json &value, const std::string &key);
bool explicitValue(const Json &value, const std::string &key);
const Json &get(const Json &value, const std::string &key);
Json get(const Json &value, const std::string &key, Json fallback);
void keys(const Json &value, std::initializer_list<const char *> allowed, const std::string &name);
void keys(const Json &value, const std::set<std::string> &allowed, const std::string &name);
std::string text(const Json &value, const std::string &name, bool empty = false);
std::string choice(const Json &value, std::initializer_list<const char *> choices,
                   const std::string &name);
bool flag(const Json &value, const std::string &name);
int count(const Json &value, const std::string &name, int low = 1, int high = 1000000);
double number(const Json &value, const std::string &name, double low, double high,
              bool exclusive_low = false);
std::string topic(const Json &value, const std::string &name);
void merge(Json &target, const Json &overrides);
Path projectRoot(const Path &configuration);
struct Context {
  Path base, project;
  std::string mode, method;
  bool cloud = false, disparity = false, recharge = false;
  bool inference = false, detection = false, segmentation = false;
  bool visual_motion=false,motion_reference=false,slip_detection=false;
};
Json loadFeatures(const Json &requested, const Context &context, Json &source_files);
void validateFeatureSettings(const Json &features, const Context &context);
void applyFeatures(const Json &features, const Context &context, const Path &run, Json &full,
                   Json &original, std::map<Path, std::string> &generated);
Json generatedInferenceDag(const Context &context, int worker_threads);
Json cameraSettings(const Json &requested, const Context &context, bool check_files,
                    const std::string &override_sdk = {}, const std::string &override_config = {},
                    int forced_timeout = 0);
} // namespace v3::detail
