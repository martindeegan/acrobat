#include <string>

#include <gtest/gtest.h>

#include "testing_utils.hpp"
#include <serial_bridge/encodings.hpp>

using namespace testing_utils;

namespace {

TEST(ChecksumTest, EmptyMessage) {
    const std::string contents = "";
    Buffer            buffer   = create_buffer(contents);
    EXPECT_EQ(compute_checksum(buffer), 0x1C89);
}

} // namespace