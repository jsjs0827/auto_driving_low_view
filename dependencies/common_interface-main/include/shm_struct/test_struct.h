#pragma once

#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#if 1
#pragma pack(push, 1)

typedef struct {
  int64_t time_ms;
  char name[64];
  uint8_t data[100000];
  uint8_t data2[400000];
} TestDataFat;

#pragma pack(pop)
#else
struct TestData
{
  int64_t time_ms;
  std::string name;
  int32_t len;
  std::vector<uint8_t> data;
  int32_t len2;
  std::vector<uint16_t> data2;
};

#pragma pack(push, 1)
struct TestDataWireHeader {
  uint32_t magic;
  uint16_t version;
  uint16_t header_size;
  int64_t time_ms;
  uint32_t name_size;
  uint32_t data_size;
  uint32_t data2_size;
};

#pragma pack(pop)

static_assert(sizeof(TestDataFat) == 50072, "TestDataFat size must be 30008");

inline constexpr uint32_t TEST_DATA_MAGIC = 0x54445354;  // "TSDT"
inline constexpr uint16_t TEST_DATA_VERSION = 1;

static_assert(sizeof(TestDataWireHeader) == 28, "TestDataWireHeader size must be 28");

inline bool canSerializeTestData(const TestData & value)
{
  return value.name.size() <= std::numeric_limits<uint32_t>::max() &&
         value.data.size() <= std::numeric_limits<uint32_t>::max() &&
         value.data2.size() <= std::numeric_limits<uint32_t>::max();
}

inline uint32_t testDataSerializedSize(const TestData & value)
{
  return static_cast<uint32_t>(
    sizeof(TestDataWireHeader) +
    value.name.size() +
    value.data.size() +
    value.data2.size() * sizeof(uint16_t));
}

inline bool serializeTestData(const TestData & value, std::vector<uint8_t> & out)
{
  if (!canSerializeTestData(value)) {
    return false;
  }

  TestDataWireHeader header{};
  header.magic = TEST_DATA_MAGIC;
  header.version = TEST_DATA_VERSION;
  header.header_size = static_cast<uint16_t>(sizeof(TestDataWireHeader));
  header.time_ms = value.time_ms;
  header.name_size = static_cast<uint32_t>(value.name.size());
  header.data_size = static_cast<uint32_t>(value.data.size());
  header.data2_size = static_cast<uint32_t>(value.data2.size());

  out.resize(testDataSerializedSize(value));

  std::size_t offset = 0;
  std::memcpy(out.data() + offset, &header, sizeof(header));
  offset += sizeof(header);

  if (!value.name.empty()) {
    std::memcpy(out.data() + offset, value.name.data(), value.name.size());
    offset += value.name.size();
  }

  if (!value.data.empty()) {
    std::memcpy(out.data() + offset, value.data.data(), value.data.size());
    offset += value.data.size();
  }

  const std::size_t data2_bytes = value.data2.size() * sizeof(uint16_t);
  if (data2_bytes > 0) {
    std::memcpy(out.data() + offset, value.data2.data(), data2_bytes);
  }

  return true;
}

inline bool deserializeTestData(const uint8_t * bytes, std::size_t size, TestData & out)
{
  if (!bytes || size < sizeof(TestDataWireHeader)) {
    return false;
  }

  TestDataWireHeader header{};
  std::memcpy(&header, bytes, sizeof(header));

  if (header.magic != TEST_DATA_MAGIC ||
      header.version != TEST_DATA_VERSION ||
      header.header_size != sizeof(TestDataWireHeader)) {
    return false;
  }

  const std::size_t name_size = header.name_size;
  const std::size_t data_size = header.data_size;
  const std::size_t data2_size = header.data2_size;
  if (data2_size > std::numeric_limits<std::size_t>::max() / sizeof(uint16_t)) {
    return false;
  }

  const std::size_t data2_bytes = data2_size * sizeof(uint16_t);
  const std::size_t total_size = sizeof(TestDataWireHeader) + name_size + data_size + data2_bytes;
  if (total_size > size) {
    return false;
  }

  std::size_t offset = sizeof(TestDataWireHeader);

  out.time_ms = header.time_ms;
  out.name.assign(reinterpret_cast<const char *>(bytes + offset), name_size);
  offset += name_size;

  out.data.resize(data_size);
  if (data_size > 0) {
    std::memcpy(out.data.data(), bytes + offset, data_size);
    offset += data_size;
  }

  out.data2.resize(data2_size);
  if (data2_bytes > 0) {
    std::memcpy(out.data2.data(), bytes + offset, data2_bytes);
  }

  out.len = static_cast<int32_t>(out.data.size());
  out.len2 = static_cast<int32_t>(out.data2.size());
  return true;
}

#endif