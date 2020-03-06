#include <string>

#include <gtest/gtest.h>

#include "testing_utils.hpp"
#include <serial_bridge/encodings.hpp>

using namespace testing_utils;

namespace {

TEST(FinalizeMessageTest, EmptyMessage) {
    const char* contents = "";
    Buffer      buffer   = create_buffer(contents, 0);
    finalize_message(buffer, buffer.begin() + checksum_size + strlen(""));

    // Check checksum
    const char* expected_message =
        "baaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

    boost::crc_optimal<16, 0x8005, 0xFFFF, 0xFFFF, true, true> crc_cipher;
    crc_cipher.process_bytes(expected_message, 77);
    const short* message_checksum_data = reinterpret_cast<const short*>(buffer.data());
    EXPECT_EQ(*message_checksum_data, crc_cipher.checksum());

    // Check end character
    EXPECT_EQ(buffer[2], '\0');
}

} // namespace