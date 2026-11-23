#include "perception/runtime_engine.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void check(bool value, const char *message) {
    if (!value)
        throw std::runtime_error(message);
}

std::string document(const std::string &tasks, const std::string &dag,
                     const std::string &extra = {}) {
    return "{\"config_version\":3,\"engine\":{\"worker_threads\":3},"
           "\"tasks\":{" + tasks + "},\"dag\":" + dag + extra + "}";
}

std::string task(const std::string &id, const std::string &extra = {}) {
    return "\"" + id + "\":{\"features\":[\"ok\"]" + extra + "}";
}

std::string repeatedArray(std::size_t count, const std::string &value) {
    std::string result = "[";
    for (std::size_t index = 0; index < count; ++index) {
        if (index)
            result.push_back(',');
        result += value;
    }
    result.push_back(']');
    return result;
}

std::string numberedObject(std::size_t count) {
    std::string result = "{";
    for (std::size_t index = 0; index < count; ++index) {
        if (index)
            result.push_back(',');
        result += "\"p" + std::to_string(index) + "\":0";
    }
    result.push_back('}');
    return result;
}

class Fixture {
  public:
    Fixture() {
        const auto unique = std::chrono::high_resolution_clock::now()
                                .time_since_epoch()
                                .count();
        directory_ = std::filesystem::temp_directory_path() /
                     ("perception-r12-v3-json-" + std::to_string(unique));
        std::filesystem::create_directories(directory_);
    }

    ~Fixture() {
        std::error_code ignored;
        std::filesystem::remove_all(directory_, ignored);
    }

    std::filesystem::path write(const std::string &name, const std::string &content) {
        const auto path = directory_ / std::filesystem::u8path(name);
        std::ofstream output(path, std::ios::binary);
        if (!output)
            throw std::runtime_error("cannot create JSON test fixture");
        output.write(content.data(), static_cast<std::streamsize>(content.size()));
        if (!output)
            throw std::runtime_error("cannot write JSON test fixture");
        return path;
    }

  private:
    std::filesystem::path directory_;
};

perception::runtime::TaskGraphConfig load(const std::filesystem::path &path) {
    return perception::runtime::loadPerceptionEngineConfig(path.u8string());
}

void reject(Fixture &fixture, const std::string &name, const std::string &content,
            const std::string &expected) {
    try {
        (void)load(fixture.write(name, content));
    } catch (const std::exception &error) {
        if (std::string(error.what()).find(expected) == std::string::npos)
            throw std::runtime_error("JSON rejection omitted '" + expected +
                                     "': " + error.what());
        return;
    }
    throw std::runtime_error("invalid JSON configuration was accepted: " + name);
}

const perception::runtime::TaskSpec &findTask(
    const perception::runtime::TaskGraphConfig &config, const std::string &id) {
    for (const auto &item : config.tasks)
        if (item.id == id)
            return item;
    throw std::runtime_error("missing task: " + id);
}

void nestedSequenceAndParallel(Fixture &fixture) {
    const auto tasks = task("a") + "," + task("b") + "," + task("c") + "," +
                       task("d") + "," + task("e");
    const auto dag =
        R"({"sequence":["a",{"parallel":[{"sequence":["b","c"]},"d"]},"e"]})";
    const auto config = load(fixture.write("nested.json", document(tasks, dag)));
    check(config.worker_threads == 3 && config.tasks.size() == 5,
          "nested JSON graph size changed");
    const std::vector<std::string> order{"a", "b", "c", "d", "e"};
    for (std::size_t index = 0; index < order.size(); ++index)
        check(config.tasks[index].id == order[index], "JSON traversal order changed");
    check(findTask(config, "a").depends_on.empty(), "root task acquired a dependency");
    check(findTask(config, "b").depends_on == std::vector<std::string>{"a"},
          "first sequence branch dependency is wrong");
    check(findTask(config, "c").depends_on == std::vector<std::string>{"b"},
          "nested sequence dependency is wrong");
    check(findTask(config, "d").depends_on == std::vector<std::string>{"a"},
          "parallel branch dependency is wrong");
    check(findTask(config, "e").depends_on ==
              std::vector<std::string>({"c", "d"}),
          "parallel join did not wait for both exits");
}

void explicitEdgesAndParameters(Fixture &fixture) {
    const std::string tasks =
        R"("a":{"features":[{"type":"ok","params":{"matrix":[1,2.5,-3],"flag":true}}],"params":{"empty":[],"label":"camera"}},)" +
        task("b") + "," + task("c");
    const auto dag = R"({"parallel":["a","b","c"]})";
    const auto extra = R"(,"edges":[{"from":"a","to":"c"},{"from":"b","to":"c"}])";
    const auto config = load(fixture.write("edges.json", document(tasks, dag, extra)));
    const auto &a = findTask(config, "a");
    check(a.parameters.at("empty").empty() && a.parameters.at("label") == "camera",
          "task-level JSON parameters were not converted");
    check(a.feature_parameters.size() == 1 &&
              a.feature_parameters[0].at("matrix") == "1,2.5,-3" &&
              a.feature_parameters[0].at("flag") == "true",
          "feature-level JSON parameters were not converted");
    check(findTask(config, "c").depends_on ==
              std::vector<std::string>({"a", "b"}),
          "explicit JSON edges were not applied in stable order");
}

void disabledIsFailClosed(Fixture &fixture) {
    const auto tasks = task("a") + "," + task("off", ",\"enabled\":false") +
                       "," + task("b");
    reject(fixture, "disabled-middle.json",
           document(tasks, R"({"sequence":["a","off","b"]})"),
           "depends on disabled task");

    const auto leaf = load(fixture.write(
        "disabled-leaf.json",
        document(tasks, R"({"sequence":["a",{"parallel":["b","off"]}]})")));
    check(!findTask(leaf, "off").enabled &&
              findTask(leaf, "off").depends_on == std::vector<std::string>{"a"},
          "disabled leaf was bypassed instead of retained in the graph");
}

void structuralFailures(Fixture &fixture) {
    const auto two = task("a") + "," + task("b");
    reject(fixture, "empty-sequence.json", document(two, R"({"sequence":[]})"),
           "must not be empty");
    reject(fixture, "single-parallel.json", document(two, R"({"parallel":["a"]})"),
           "at least two");
    reject(fixture, "two-operators.json",
           document(two, R"({"sequence":["a"],"parallel":["a","b"]})"),
           "exactly one");
    reject(fixture, "unknown-node.json", document(two, R"({"choice":["a","b"]})"),
           "unknown dag node");
    reject(fixture, "unknown-ref.json", document(two, R"({"sequence":["missing"]})"),
           "unknown task");
    reject(fixture, "duplicate-ref.json",
           document(two, R"({"sequence":["a","b","a"]})"), "more than once");
    reject(fixture, "unreferenced.json", document(two, R"({"task":"a"})"),
           "not referenced");
}

void edgeFailures(Fixture &fixture) {
    const auto two = task("a") + "," + task("b");
    reject(fixture, "unknown-edge.json",
           document(two, R"({"parallel":["a","b"]})",
                    R"(,"edges":[{"from":"missing","to":"b"}])"),
           "unknown source");
    reject(fixture, "self-edge.json",
           document(two, R"({"parallel":["a","b"]})",
                    R"(,"edges":[{"from":"a","to":"a"}])"),
           "self edge");
    reject(fixture, "duplicate-edge.json",
           document(two, R"({"parallel":["a","b"]})",
                    R"(,"edges":[{"from":"a","to":"b"},{"from":"a","to":"b"}])"),
           "duplicates an explicit edge");
    reject(fixture, "structured-edge.json",
           document(two, R"({"sequence":["a","b"]})",
                    R"(,"edges":[{"from":"a","to":"b"}])"),
           "duplicates a structured DAG edge");
    reject(fixture, "cycle.json",
           document(two, R"({"sequence":["a","b"]})",
                    R"(,"edges":[{"from":"b","to":"a"}])"),
           "contains a cycle");
}

void schemaAndSyntaxFailures(Fixture &fixture) {
    const auto one = task("a");
    reject(fixture, "unknown-root.json",
           document(one, R"({"task":"a"})", R"(,"typo":1)"),
           "unknown field 'typo'");
    reject(fixture, "unknown-task-field.json",
           document(task("a", ",\"featurez\":[]"), R"({"task":"a"})"),
           "featurez");
    reject(fixture, "unknown-feature-field.json",
           document(R"("a":{"features":[{"type":"ok","typo":1}]})",
                    R"({"task":"a"})"),
           "typo");
    reject(fixture, "no-features.json",
           document(R"("a":{"features":[]})", R"({"task":"a"})"),
           "has no features");
    reject(fixture, "null-param.json",
           document(R"("a":{"features":["ok"],"params":{"x":null}})",
                    R"({"task":"a"})"),
           "must be a string");
    reject(fixture, "mixed-array.json",
           document(R"("a":{"features":["ok"],"params":{"x":[1,true]}})",
                    R"({"task":"a"})"),
           "numeric array");
    reject(fixture, "wrong-version.json",
           R"({"config_version":2,"engine":{},"tasks":{"a":{"features":["ok"]}},"dag":"a"})",
           "config_version 3");
    reject(fixture, "duplicate-key.json",
           R"({"config_version":3,"config_version":3,"engine":{},"tasks":{"a":{"features":["ok"]}},"dag":"a"})",
           "duplicate object field");
    reject(fixture, "trailing-comma.json",
           R"({"config_version":3,"engine":{},"tasks":{"a":{"features":["ok"],}},"dag":"a"})",
           "object field name");
    reject(fixture, "trailing-content.json", document(one, "\"a\"") + "x",
           "unexpected content");
}

void bomUnicodeAndInvalidUtf8(Fixture &fixture) {
    const auto valid = document(task("a", ",\"params\":{\"label\":\"相机\"}"),
                                R"({"task":"a"})");
    const auto config = load(fixture.write("配置.json", std::string("\xEF\xBB\xBF") + valid));
    check(findTask(config, "a").parameters.at("label") == "相机",
          "UTF-8/BOM configuration was not preserved");
    reject(fixture, "invalid-utf8.json", std::string("{\"") +
               static_cast<char>(0xFF) + "\":1}", "invalid UTF-8");
}

void resourceLimitFailures(Fixture &fixture) {
    const auto one = task("a");
    const auto dag = R"({"task":"a"})";

    reject(fixture, "worker-zero.json",
           R"({"config_version":3,"engine":{"worker_threads":0},"tasks":{"a":{"features":["ok"]}},"dag":"a"})",
           "positive integer");
    reject(fixture, "worker-too-large.json",
           R"({"config_version":3,"engine":{"worker_threads":257},"tasks":{"a":{"features":["ok"]}},"dag":"a"})",
           "1..256");
    const auto maximum_workers = load(fixture.write(
        "worker-maximum.json",
        R"({"config_version":3,"engine":{"worker_threads":256},"tasks":{"a":{"features":["ok"]}},"dag":"a"})"));
    check(maximum_workers.worker_threads == 256,
          "maximum documented worker count was rejected");

    reject(fixture, "string-limit.json",
           document(one, dag,
                    ",\"$schema\":\"" + std::string(65537U, 'x') + "\""),
           "string exceeds the limit");
    reject(fixture, "array-limit.json",
           document(one, dag,
                    ",\"overflow\":" + repeatedArray(16385U, "0")),
           "array exceeds the limit");
    reject(fixture, "object-limit.json",
           document(one, dag,
                    ",\"overflow\":" + numberedObject(1025U)),
           "object exceeds the limit");

    const auto excessive_features =
        "\"a\":{\"features\":" + repeatedArray(257U, "\"ok\"") + "}";
    reject(fixture, "feature-limit.json", document(excessive_features, dag),
           "limit of 256 features");
    const auto excessive_parameters =
        "\"a\":{\"features\":[\"ok\"],\"params\":" +
        numberedObject(257U) + "}";
    reject(fixture, "parameter-limit.json", document(excessive_parameters, dag),
           "limit of 256 parameters");
    const auto excessive_parameter_list =
        "\"a\":{\"features\":[\"ok\"],\"params\":{\"values\":" +
        repeatedArray(4097U, "0") + "}}";
    reject(fixture, "parameter-list-limit.json",
           document(excessive_parameter_list, dag), "limit of 4096 numeric values");

    std::string many_nodes = "[";
    for (std::size_t index = 0; index < 1024U; ++index) {
        if (index)
            many_nodes.push_back(',');
        many_nodes += repeatedArray(64U, "0");
    }
    many_nodes.push_back(']');
    reject(fixture, "node-limit.json",
           document(one, dag, ",\"overflow\":" + many_nodes),
           "value count exceeds the limit");
}

} // namespace

int main() {
    try {
        Fixture fixture;
        nestedSequenceAndParallel(fixture);
        explicitEdgesAndParameters(fixture);
        disabledIsFailClosed(fixture);
        structuralFailures(fixture);
        edgeFailures(fixture);
        schemaAndSyntaxFailures(fixture);
        bomUnicodeAndInvalidUtf8(fixture);
        resourceLimitFailures(fixture);
        std::cout << "PASS r12-v3 strict nested JSON DAG configuration\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
