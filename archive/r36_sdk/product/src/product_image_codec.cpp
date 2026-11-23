// The root preview codec uses private/static no-stdio symbols. The original
// product JPEG writer needs this separate real stdio implementation.
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
