#pragma once

#include <serial_bridge/encodings.hpp>

using namespace acrobat::serial_bridge;

namespace testing_utils {

Buffer create_buffer(const char* contents, size_t length);

} // namespace testing_utils