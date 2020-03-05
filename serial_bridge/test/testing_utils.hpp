#pragma once

#include <string>

#include <serial_bridge/encodings.hpp>

using namespace acrobat::serial_bridge;

namespace testing_utils {

Buffer create_buffer(std::string contents);

} // namespace testing_utils