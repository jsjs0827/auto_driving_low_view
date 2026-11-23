#include "runtime_config_json.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace perception::runtime {
namespace {

constexpr std::size_t kMaximumConfigBytes = 4U * 1024U * 1024U;
constexpr std::size_t kMaximumJsonDepth = 64U;
constexpr std::size_t kMaximumJsonNodes = 65536U;
constexpr std::size_t kMaximumStringBytes = 65536U;
constexpr std::size_t kMaximumArrayItems = 16384U;
constexpr std::size_t kMaximumObjectMembers = 1024U;
constexpr std::size_t kMaximumTasks = 512U;
constexpr std::size_t kMaximumEdges = 16384U;
constexpr std::size_t kMaximumFeaturesPerTask = 256U;
constexpr std::size_t kMaximumParameters = 256U;
constexpr std::size_t kMaximumParameterListItems = 4096U;

struct JsonValue {
    enum class Kind { Null, Boolean, Number, String, Array, Object };

    Kind kind = Kind::Null;
    bool boolean = false;
    std::string text;
    std::vector<JsonValue> array;
    std::map<std::string, JsonValue> object;
};

class JsonParser {
  public:
    JsonParser(std::string source, std::string path)
        : source_(std::move(source)), path_(std::move(path)) {}

    JsonValue parse() {
        skipWhitespace();
        auto result = value(0);
        skipWhitespace();
        if (position_ != source_.size())
            fail("unexpected content after the JSON value");
        return result;
    }

  private:
    [[noreturn]] void fail(const std::string &message) const {
        std::size_t line = 1;
        std::size_t column = 1;
        for (std::size_t index = 0; index < position_ && index < source_.size(); ++index) {
            if (source_[index] == '\n') {
                ++line;
                column = 1;
            } else {
                ++column;
            }
        }
        throw std::runtime_error(path_ + ":" + std::to_string(line) + ":" +
                                 std::to_string(column) + ": " + message);
    }

    void skipWhitespace() {
        while (position_ < source_.size()) {
            const char character = source_[position_];
            if (character != ' ' && character != '\t' && character != '\r' &&
                character != '\n')
                break;
            ++position_;
        }
    }

    bool consume(char expected) {
        if (position_ >= source_.size() || source_[position_] != expected)
            return false;
        ++position_;
        return true;
    }

    void expect(char expected, const char *description) {
        if (!consume(expected))
            fail(std::string("expected ") + description);
    }

    JsonValue value(std::size_t depth) {
        if (depth > kMaximumJsonDepth)
            fail("JSON nesting exceeds 64 levels");
        if (++node_count_ > kMaximumJsonNodes)
            fail("JSON value count exceeds the limit of 65536");
        if (position_ >= source_.size())
            fail("unexpected end of JSON input");
        switch (source_[position_]) {
        case '{':
            return object(depth + 1);
        case '[':
            return array(depth + 1);
        case '"': {
            JsonValue result;
            result.kind = JsonValue::Kind::String;
            result.text = string();
            return result;
        }
        case 't':
            literal("true");
            return boolean(true);
        case 'f':
            literal("false");
            return boolean(false);
        case 'n':
            literal("null");
            return {};
        default:
            if (source_[position_] == '-' ||
                (source_[position_] >= '0' && source_[position_] <= '9'))
                return number();
            fail("expected a JSON value");
        }
    }

    JsonValue object(std::size_t depth) {
        expect('{', "'{'");
        JsonValue result;
        result.kind = JsonValue::Kind::Object;
        skipWhitespace();
        if (consume('}'))
            return result;
        for (;;) {
            if (result.object.size() >= kMaximumObjectMembers)
                fail("JSON object exceeds the limit of 1024 members");
            if (position_ >= source_.size() || source_[position_] != '"')
                fail("expected an object field name");
            auto key = string();
            skipWhitespace();
            expect(':', "':' after an object field name");
            skipWhitespace();
            auto inserted = result.object.emplace(std::move(key), value(depth));
            if (!inserted.second)
                fail("duplicate object field: " + inserted.first->first);
            skipWhitespace();
            if (consume('}'))
                return result;
            expect(',', "',' or '}' in an object");
            skipWhitespace();
        }
    }

    JsonValue array(std::size_t depth) {
        expect('[', "'['");
        JsonValue result;
        result.kind = JsonValue::Kind::Array;
        skipWhitespace();
        if (consume(']'))
            return result;
        for (;;) {
            if (result.array.size() >= kMaximumArrayItems)
                fail("JSON array exceeds the limit of 16384 items");
            result.array.push_back(value(depth));
            skipWhitespace();
            if (consume(']'))
                return result;
            expect(',', "',' or ']' in an array");
            skipWhitespace();
        }
    }

    static int hexadecimal(char value) {
        if (value >= '0' && value <= '9')
            return value - '0';
        if (value >= 'a' && value <= 'f')
            return value - 'a' + 10;
        if (value >= 'A' && value <= 'F')
            return value - 'A' + 10;
        return -1;
    }

    std::uint32_t unicodeEscape() {
        if (position_ + 4 > source_.size())
            fail("incomplete Unicode escape");
        std::uint32_t result = 0;
        for (int index = 0; index < 4; ++index) {
            const int digit = hexadecimal(source_[position_++]);
            if (digit < 0)
                fail("invalid hexadecimal digit in Unicode escape");
            result = (result << 4U) | static_cast<std::uint32_t>(digit);
        }
        return result;
    }

    void appendUtf8(std::string &target, std::uint32_t codepoint) {
        if (codepoint <= 0x7FU) {
            target.push_back(static_cast<char>(codepoint));
        } else if (codepoint <= 0x7FFU) {
            target.push_back(static_cast<char>(0xC0U | (codepoint >> 6U)));
            target.push_back(static_cast<char>(0x80U | (codepoint & 0x3FU)));
        } else if (codepoint <= 0xFFFFU) {
            target.push_back(static_cast<char>(0xE0U | (codepoint >> 12U)));
            target.push_back(static_cast<char>(0x80U | ((codepoint >> 6U) & 0x3FU)));
            target.push_back(static_cast<char>(0x80U | (codepoint & 0x3FU)));
        } else {
            target.push_back(static_cast<char>(0xF0U | (codepoint >> 18U)));
            target.push_back(static_cast<char>(0x80U | ((codepoint >> 12U) & 0x3FU)));
            target.push_back(static_cast<char>(0x80U | ((codepoint >> 6U) & 0x3FU)));
            target.push_back(static_cast<char>(0x80U | (codepoint & 0x3FU)));
        }
    }

    std::string string() {
        expect('"', "string");
        std::string result;
        while (position_ < source_.size()) {
            const unsigned char character =
                static_cast<unsigned char>(source_[position_++]);
            if (character == '"')
                return result;
            if (character < 0x20U)
                fail("unescaped control character in a string");
            if (character != '\\') {
                result.push_back(static_cast<char>(character));
                if (result.size() > kMaximumStringBytes)
                    fail("JSON string exceeds the limit of 65536 bytes");
                continue;
            }
            if (position_ >= source_.size())
                fail("incomplete string escape");
            const char escape = source_[position_++];
            switch (escape) {
            case '"': result.push_back('"'); break;
            case '\\': result.push_back('\\'); break;
            case '/': result.push_back('/'); break;
            case 'b': result.push_back('\b'); break;
            case 'f': result.push_back('\f'); break;
            case 'n': result.push_back('\n'); break;
            case 'r': result.push_back('\r'); break;
            case 't': result.push_back('\t'); break;
            case 'u': {
                std::uint32_t codepoint = unicodeEscape();
                if (codepoint >= 0xD800U && codepoint <= 0xDBFFU) {
                    if (position_ + 2 > source_.size() || source_[position_] != '\\' ||
                        source_[position_ + 1] != 'u')
                        fail("high surrogate is not followed by a low surrogate");
                    position_ += 2;
                    const std::uint32_t low = unicodeEscape();
                    if (low < 0xDC00U || low > 0xDFFFU)
                        fail("invalid low surrogate");
                    codepoint = 0x10000U + ((codepoint - 0xD800U) << 10U) +
                                (low - 0xDC00U);
                } else if (codepoint >= 0xDC00U && codepoint <= 0xDFFFU) {
                    fail("unexpected low surrogate");
                }
                appendUtf8(result, codepoint);
                break;
            }
            default:
                fail("invalid string escape");
            }
            if (result.size() > kMaximumStringBytes)
                fail("JSON string exceeds the limit of 65536 bytes");
        }
        fail("unterminated string");
    }

    JsonValue number() {
        const std::size_t begin = position_;
        consume('-');
        if (position_ >= source_.size())
            fail("incomplete number");
        if (source_[position_] == '0') {
            ++position_;
            if (position_ < source_.size() && std::isdigit(
                    static_cast<unsigned char>(source_[position_])))
                fail("leading zero in a number");
        } else {
            if (source_[position_] < '1' || source_[position_] > '9')
                fail("invalid integer part");
            while (position_ < source_.size() && std::isdigit(
                       static_cast<unsigned char>(source_[position_])))
                ++position_;
        }
        if (consume('.')) {
            if (position_ >= source_.size() || !std::isdigit(
                    static_cast<unsigned char>(source_[position_])))
                fail("fraction requires at least one digit");
            while (position_ < source_.size() && std::isdigit(
                       static_cast<unsigned char>(source_[position_])))
                ++position_;
        }
        if (position_ < source_.size() &&
            (source_[position_] == 'e' || source_[position_] == 'E')) {
            ++position_;
            if (position_ < source_.size() &&
                (source_[position_] == '+' || source_[position_] == '-'))
                ++position_;
            if (position_ >= source_.size() || !std::isdigit(
                    static_cast<unsigned char>(source_[position_])))
                fail("exponent requires at least one digit");
            while (position_ < source_.size() && std::isdigit(
                       static_cast<unsigned char>(source_[position_])))
                ++position_;
        }
        JsonValue result;
        result.kind = JsonValue::Kind::Number;
        result.text = source_.substr(begin, position_ - begin);
        return result;
    }

    JsonValue boolean(bool value) {
        JsonValue result;
        result.kind = JsonValue::Kind::Boolean;
        result.boolean = value;
        return result;
    }

    void literal(const char *expected) {
        const std::string token(expected);
        if (source_.compare(position_, token.size(), token) != 0)
            fail("invalid JSON literal");
        position_ += token.size();
    }

    std::string source_;
    std::string path_;
    std::size_t position_ = 0;
    std::size_t node_count_ = 0;
};

const JsonValue &field(const JsonValue &object, const std::string &name,
                       const std::string &context) {
    const auto found = object.object.find(name);
    if (found == object.object.end())
        throw std::runtime_error(context + " requires field '" + name + "'");
    return found->second;
}

const JsonValue *optionalField(const JsonValue &object, const std::string &name) {
    const auto found = object.object.find(name);
    return found == object.object.end() ? nullptr : &found->second;
}

void requireKind(const JsonValue &value, JsonValue::Kind expected,
                 const std::string &context) {
    if (value.kind != expected)
        throw std::runtime_error(context + " has the wrong JSON type");
}

void rejectUnknownFields(const JsonValue &value, const std::set<std::string> &allowed,
                         const std::string &context) {
    requireKind(value, JsonValue::Kind::Object, context);
    for (const auto &entry : value.object)
        if (!allowed.count(entry.first))
            throw std::runtime_error(context + " contains unknown field '" +
                                     entry.first + "'");
}

std::string requireString(const JsonValue &value, const std::string &context) {
    requireKind(value, JsonValue::Kind::String, context);
    if (value.text.empty())
        throw std::runtime_error(context + " must not be empty");
    return value.text;
}

bool optionalBoolean(const JsonValue &object, const std::string &name, bool fallback,
                     const std::string &context) {
    const auto *value = optionalField(object, name);
    if (!value)
        return fallback;
    requireKind(*value, JsonValue::Kind::Boolean, context + "." + name);
    return value->boolean;
}

std::size_t positiveInteger(const JsonValue &value, const std::string &context) {
    requireKind(value, JsonValue::Kind::Number, context);
    if (value.text.empty() || value.text.front() == '-' ||
        value.text.find_first_of(".eE") != std::string::npos)
        throw std::runtime_error(context + " must be a positive integer");
    std::size_t used = 0;
    unsigned long long parsed = 0;
    try {
        parsed = std::stoull(value.text, &used);
    } catch (const std::exception &) {
        throw std::runtime_error(context + " is outside the supported integer range");
    }
    if (used != value.text.size() || parsed == 0 ||
        parsed > std::numeric_limits<std::size_t>::max())
        throw std::runtime_error(context + " must be a positive integer");
    return static_cast<std::size_t>(parsed);
}

std::string scalarParameter(const JsonValue &value, const std::string &context) {
    switch (value.kind) {
    case JsonValue::Kind::String:
    case JsonValue::Kind::Number:
        return value.text;
    case JsonValue::Kind::Boolean:
        return value.boolean ? "true" : "false";
    default:
        throw std::runtime_error(context +
                                 " must be a string, number, boolean, or flat array");
    }
}

std::string parameterValue(const JsonValue &value, const std::string &context) {
    if (value.kind != JsonValue::Kind::Array)
        return scalarParameter(value, context);
    if (value.array.size() > kMaximumParameterListItems)
        throw std::runtime_error(context +
                                 " exceeds the limit of 4096 numeric values");
    std::string result;
    for (std::size_t index = 0; index < value.array.size(); ++index) {
        if (index)
            result.push_back(',');
        if (value.array[index].kind != JsonValue::Kind::Number)
            throw std::runtime_error(context + " must be a flat numeric array");
        result += value.array[index].text;
    }
    return result;
}

Parameters parameters(const JsonValue *value, const std::string &context) {
    Parameters result;
    if (!value)
        return result;
    requireKind(*value, JsonValue::Kind::Object, context);
    if (value->object.size() > kMaximumParameters)
        throw std::runtime_error(context + " exceeds the limit of 256 parameters");
    for (const auto &entry : value->object) {
        if (entry.first.empty())
            throw std::runtime_error(context + " contains an empty parameter name");
        result.emplace(entry.first, parameterValue(entry.second,
                                                    context + "." + entry.first));
    }
    return result;
}

std::vector<std::string> stringArray(const JsonValue *value, const std::string &context) {
    std::vector<std::string> result;
    if (!value)
        return result;
    requireKind(*value, JsonValue::Kind::Array, context);
    result.reserve(value->array.size());
    for (std::size_t index = 0; index < value->array.size(); ++index)
        result.push_back(requireString(value->array[index], context + "[" +
                                                               std::to_string(index) + "]"));
    return result;
}

bool validIdentifier(const std::string &value) {
    if (value.empty() || value.size() > 128U)
        return false;
    const auto first = static_cast<unsigned char>(value.front());
    if (!(std::isalpha(first) || value.front() == '_'))
        return false;
    return std::all_of(value.begin() + 1, value.end(), [](char character) {
        const auto byte = static_cast<unsigned char>(character);
        return std::isalnum(byte) || character == '_' || character == '-' ||
               character == '.';
    });
}

TaskSpec taskDefinition(const std::string &id, const JsonValue &value) {
    if (!validIdentifier(id))
        throw std::runtime_error("invalid task id '" + id + "'");
    const std::string context = "tasks." + id;
    rejectUnknownFields(value, {"enabled", "features", "params"}, context);
    TaskSpec result;
    result.id = id;
    result.enabled = optionalBoolean(value, "enabled", true, context);
    result.parameters = parameters(optionalField(value, "params"), context + ".params");
    const auto &features = field(value, "features", context);
    requireKind(features, JsonValue::Kind::Array, context + ".features");
    if (features.array.size() > kMaximumFeaturesPerTask)
        throw std::runtime_error(context +
                                 ".features exceeds the limit of 256 features");
    result.feature_parameters.reserve(features.array.size());
    for (std::size_t index = 0; index < features.array.size(); ++index) {
        const auto &feature = features.array[index];
        const auto feature_context = context + ".features[" + std::to_string(index) + "]";
        if (feature.kind == JsonValue::Kind::String) {
            result.features.push_back(requireString(feature, feature_context));
            result.feature_parameters.emplace_back();
            continue;
        }
        rejectUnknownFields(feature, {"params", "type"}, feature_context);
        result.features.push_back(requireString(field(feature, "type", feature_context),
                                                feature_context + ".type"));
        result.feature_parameters.push_back(
            parameters(optionalField(feature, "params"), feature_context + ".params"));
    }
    if (result.features.empty())
        throw std::runtime_error(context + " has no features");
    return result;
}

struct Definition {
    TaskSpec task;
    bool referenced = false;
};

struct NodeSpan {
    std::vector<std::string> entries;
    std::vector<std::string> exits;
};

void appendUnique(std::vector<std::string> &target, const std::string &value) {
    if (std::find(target.begin(), target.end(), value) == target.end())
        target.push_back(value);
}

void addDependency(std::map<std::string, Definition> &definitions,
                   const std::string &task, const std::string &dependency) {
    auto &dependencies = definitions.at(task).task.depends_on;
    appendUnique(dependencies, dependency);
}

NodeSpan expandNode(const JsonValue &node,
                    std::map<std::string, Definition> &definitions,
                    std::vector<std::string> &task_order, std::size_t depth);

NodeSpan taskReference(const std::string &id,
                       std::map<std::string, Definition> &definitions,
                       std::vector<std::string> &task_order) {
    const auto found = definitions.find(id);
    if (found == definitions.end())
        throw std::runtime_error("dag references unknown task '" + id + "'");
    if (found->second.referenced)
        throw std::runtime_error("dag references task more than once: " + id);
    found->second.referenced = true;
    task_order.push_back(id);
    return {{id}, {id}};
}

NodeSpan expandSequence(const JsonValue &children,
                        std::map<std::string, Definition> &definitions,
                        std::vector<std::string> &task_order, std::size_t depth) {
    requireKind(children, JsonValue::Kind::Array, "dag.sequence");
    if (children.array.empty())
        throw std::runtime_error("dag.sequence must not be empty");
    NodeSpan result;
    std::vector<std::string> previous_exits;
    for (const auto &child : children.array) {
        auto span = expandNode(child, definitions, task_order, depth + 1);
        if (span.entries.empty())
            continue;
        if (result.entries.empty())
            result.entries = span.entries;
        for (const auto &entry : span.entries)
            for (const auto &previous : previous_exits)
                addDependency(definitions, entry, previous);
        previous_exits = std::move(span.exits);
    }
    result.exits = std::move(previous_exits);
    return result;
}

NodeSpan expandParallel(const JsonValue &branches,
                        std::map<std::string, Definition> &definitions,
                        std::vector<std::string> &task_order, std::size_t depth) {
    requireKind(branches, JsonValue::Kind::Array, "dag.parallel");
    if (branches.array.size() < 2U)
        throw std::runtime_error("dag.parallel requires at least two branches");
    NodeSpan result;
    for (const auto &branch : branches.array) {
        auto span = expandNode(branch, definitions, task_order, depth + 1);
        for (const auto &entry : span.entries)
            appendUnique(result.entries, entry);
        for (const auto &exit : span.exits)
            appendUnique(result.exits, exit);
    }
    return result;
}

NodeSpan expandNode(const JsonValue &node,
                    std::map<std::string, Definition> &definitions,
                    std::vector<std::string> &task_order, std::size_t depth) {
    if (depth > kMaximumJsonDepth)
        throw std::runtime_error("dag nesting exceeds 64 levels");
    if (node.kind == JsonValue::Kind::String)
        return taskReference(requireString(node, "dag task reference"), definitions,
                             task_order);
    requireKind(node, JsonValue::Kind::Object, "dag node");
    if (node.object.size() != 1U)
        throw std::runtime_error(
            "each dag node must contain exactly one of task, sequence, or parallel");
    const auto &entry = *node.object.begin();
    if (entry.first == "task")
        return taskReference(requireString(entry.second, "dag.task"), definitions,
                             task_order);
    if (entry.first == "sequence")
        return expandSequence(entry.second, definitions, task_order, depth);
    if (entry.first == "parallel")
        return expandParallel(entry.second, definitions, task_order, depth);
    throw std::runtime_error("unknown dag node '" + entry.first + "'");
}

void validateDependencies(const TaskGraphConfig &config) {
    std::map<std::string, std::size_t> indices;
    for (std::size_t index = 0; index < config.tasks.size(); ++index)
        indices.emplace(config.tasks[index].id, index);
    std::vector<int> indegree(config.tasks.size(), 0);
    std::vector<std::vector<std::size_t>> successors(config.tasks.size());
    std::size_t edge_count = 0;
    for (std::size_t index = 0; index < config.tasks.size(); ++index) {
        const auto &task = config.tasks[index];
        std::set<std::string> unique;
        for (const auto &dependency : task.depends_on) {
            const auto found = indices.find(dependency);
            if (found == indices.end())
                throw std::runtime_error("unknown dependency '" + dependency +
                                         "' for task '" + task.id + "'");
            if (!unique.insert(dependency).second)
                throw std::runtime_error("duplicate dependency '" + dependency +
                                         "' for task '" + task.id + "'");
            if (dependency == task.id)
                throw std::runtime_error("task depends on itself: " + task.id);
            if (task.enabled && !config.tasks[found->second].enabled)
                throw std::runtime_error("enabled task '" + task.id +
                                         "' depends on disabled task '" + dependency + "'");
            ++indegree[index];
            if (++edge_count > kMaximumEdges)
                throw std::runtime_error("task dependency graph exceeds 16384 edges");
            successors[found->second].push_back(index);
        }
    }
    std::queue<std::size_t> ready;
    for (std::size_t index = 0; index < indegree.size(); ++index)
        if (indegree[index] == 0)
            ready.push(index);
    std::size_t visited = 0;
    while (!ready.empty()) {
        const auto current = ready.front();
        ready.pop();
        ++visited;
        for (const auto next : successors[current])
            if (--indegree[next] == 0)
                ready.push(next);
    }
    if (visited != config.tasks.size())
        throw std::runtime_error("task dependency graph contains a cycle");
}

void addExplicitEdges(const JsonValue *edges,
                      std::map<std::string, Definition> &definitions) {
    if (!edges)
        return;
    requireKind(*edges, JsonValue::Kind::Array, "edges");
    std::set<std::pair<std::string, std::string>> unique;
    for (std::size_t index = 0; index < edges->array.size(); ++index) {
        const auto &edge = edges->array[index];
        const auto context = "edges[" + std::to_string(index) + "]";
        rejectUnknownFields(edge, {"from", "to"}, context);
        const auto from = requireString(field(edge, "from", context), context + ".from");
        const auto to = requireString(field(edge, "to", context), context + ".to");
        if (!definitions.count(from))
            throw std::runtime_error(context + " references unknown source task '" + from + "'");
        if (!definitions.count(to))
            throw std::runtime_error(context + " references unknown target task '" + to + "'");
        if (from == to)
            throw std::runtime_error(context + " is a self edge");
        if (!unique.emplace(from, to).second)
            throw std::runtime_error(context + " duplicates an explicit edge");
        auto &dependencies = definitions.at(to).task.depends_on;
        if (std::find(dependencies.begin(), dependencies.end(), from) != dependencies.end())
            throw std::runtime_error(context + " duplicates a structured DAG edge");
        dependencies.push_back(from);
    }
}

void validateUtf8(const std::string &content, const std::string &path) {
    std::size_t index = 0;
    while (index < content.size()) {
        const auto first = static_cast<unsigned char>(content[index]);
        if (first <= 0x7FU) {
            ++index;
            continue;
        }
        std::size_t length = 0;
        std::uint32_t codepoint = 0;
        if (first >= 0xC2U && first <= 0xDFU) {
            length = 2;
            codepoint = first & 0x1FU;
        } else if (first >= 0xE0U && first <= 0xEFU) {
            length = 3;
            codepoint = first & 0x0FU;
        } else if (first >= 0xF0U && first <= 0xF4U) {
            length = 4;
            codepoint = first & 0x07U;
        } else {
            throw std::runtime_error(path + ": invalid UTF-8 at byte " +
                                     std::to_string(index));
        }
        if (index + length > content.size())
            throw std::runtime_error(path + ": truncated UTF-8 at byte " +
                                     std::to_string(index));
        for (std::size_t offset = 1; offset < length; ++offset) {
            const auto continuation =
                static_cast<unsigned char>(content[index + offset]);
            if ((continuation & 0xC0U) != 0x80U)
                throw std::runtime_error(path + ": invalid UTF-8 continuation at byte " +
                                         std::to_string(index + offset));
            codepoint = (codepoint << 6U) | (continuation & 0x3FU);
        }
        const bool overlong = (length == 2 && codepoint < 0x80U) ||
                              (length == 3 && codepoint < 0x800U) ||
                              (length == 4 && codepoint < 0x10000U);
        if (overlong || codepoint > 0x10FFFFU ||
            (codepoint >= 0xD800U && codepoint <= 0xDFFFU))
            throw std::runtime_error(path + ": invalid UTF-8 code point at byte " +
                                     std::to_string(index));
        index += length;
    }
}

std::string readConfig(const std::string &path) {
    std::ifstream stream(std::filesystem::u8path(path), std::ios::binary | std::ios::ate);
    if (!stream)
        throw std::runtime_error("cannot open JSON task graph config: " + path);
    const auto end = stream.tellg();
    if (end < 0)
        throw std::runtime_error("cannot determine JSON config size: " + path);
    const auto size = static_cast<std::uintmax_t>(end);
    if (size > kMaximumConfigBytes)
        throw std::runtime_error("JSON task graph config exceeds 4 MiB: " + path);
    std::string content(static_cast<std::size_t>(size), '\0');
    stream.seekg(0, std::ios::beg);
    if (!content.empty() && !stream.read(content.data(),
                                         static_cast<std::streamsize>(content.size())))
        throw std::runtime_error("cannot read JSON task graph config: " + path);
    if (content.compare(0, 3, "\xEF\xBB\xBF") == 0)
        content.erase(0, 3);
    validateUtf8(content, path);
    return content;
}

} // namespace

TaskGraphConfig loadNestedJsonTaskGraphConfig(const std::string &path) {
    const auto root = JsonParser(readConfig(path), path).parse();
    rejectUnknownFields(root,
                        {"$schema", "config_version", "dag", "edges", "engine", "tasks"},
                        "root");
    if (const auto *schema = optionalField(root, "$schema"))
        (void)requireString(*schema, "$schema");
    const auto &version = field(root, "config_version", "root");
    if (positiveInteger(version, "config_version") != 3U)
        throw std::runtime_error("JSON task graph requires config_version 3");

    TaskGraphConfig config;
    const auto &engine = field(root, "engine", "root");
    rejectUnknownFields(engine, {"result_artifacts", "worker_threads"}, "engine");
    if (const auto *workers = optionalField(engine, "worker_threads"))
        config.worker_threads = positiveInteger(*workers, "engine.worker_threads");
    else
        config.worker_threads = 3;
    if (config.worker_threads > 256U)
        throw std::runtime_error("engine.worker_threads must be 1..256");
    config.result_artifacts = stringArray(optionalField(engine, "result_artifacts"),
                                          "engine.result_artifacts");

    const auto &task_values = field(root, "tasks", "root");
    requireKind(task_values, JsonValue::Kind::Object, "tasks");
    if (task_values.object.empty())
        throw std::runtime_error("tasks must not be empty");
    if (task_values.object.size() > kMaximumTasks)
        throw std::runtime_error("tasks exceeds the limit of 512");
    std::map<std::string, Definition> definitions;
    for (const auto &entry : task_values.object)
        definitions.emplace(entry.first,
                            Definition{taskDefinition(entry.first, entry.second), false});

    std::vector<std::string> task_order;
    (void)expandNode(field(root, "dag", "root"), definitions, task_order, 0);
    addExplicitEdges(optionalField(root, "edges"), definitions);
    for (const auto &entry : definitions)
        if (!entry.second.referenced)
            throw std::runtime_error("task is defined but not referenced by dag: " +
                                     entry.first);

    config.tasks.reserve(task_order.size());
    for (const auto &id : task_order)
        config.tasks.push_back(std::move(definitions.at(id).task));
    validateDependencies(config);
    return config;
}

} // namespace perception::runtime
