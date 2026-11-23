#pragma once
#include <string>

#if defined(_WIN32)
#define PERCEPTION_PRODUCT_IDENTITY_EXPORT __declspec(dllexport)
#elif defined(__GNUC__)
#define PERCEPTION_PRODUCT_IDENTITY_EXPORT __attribute__((visibility("default")))
#else
#define PERCEPTION_PRODUCT_IDENTITY_EXPORT
#endif

extern "C" PERCEPTION_PRODUCT_IDENTITY_EXPORT
const char* perception_product_build_id() noexcept;

namespace r31::product {
// A receipt proves that this module reached interface initialization. It does
// not prove that a frame was received, inferred, or published successfully.
// The destination's parent directory must already exist. A configured write
// failure throws r28::Failure with initialization code 1004.
void writeRosIoReceipt(const std::string& destination);
void writeRosIoReceiptFromEnvironment();
}
