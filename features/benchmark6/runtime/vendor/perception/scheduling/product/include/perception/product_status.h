#ifndef PERCEPTION_PRODUCT_STATUS_H
#define PERCEPTION_PRODUCT_STATUS_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* Stable full business code, 0 for success. Query required bytes (including NUL)
 * with a NULL buffer, then copy the same structured status with sufficient space.
 * The snapshot can change between calls; retry if required grows. */
uint32_t perception_product_last_status(char* json, size_t capacity, size_t* required);
#ifdef __cplusplus
}
#endif
#endif
