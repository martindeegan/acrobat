#include <string>

#include <gtest/gtest.h>

#include "testing_utils.hpp"
#include <serial_bridge/encodings.hpp>

using namespace testing_utils;

namespace {

TEST(FinalizeMessageTest, EmptyMessage) {
    constexpr std::string_view contents = "";
    Buffer                     buffer   = create_buffer(contents);
    finalize_message(buffer, buffer.begin() + checksum_size + strlen(""));
}

} // namespace