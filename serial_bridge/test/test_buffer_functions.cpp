#include <cmath>
#include <numeric>
#include <string>

#include <gtest/gtest.h>

#include "testing_utils.hpp"

using namespace acrobat::serial_bridge;

namespace {

// Test that clear buffer sets the whole message part to zero. We don't care about the checksum part
TEST(TestBufferFunctions, BufferClear) {
    char buffer[buffer_size];
    clear_buffer(buffer);

    for (size_t i = checksum_size; i < buffer_size; i++) {
        EXPECT_EQ(buffer[i], 0);
    }
}

TEST(TestBufferFunctions, IndexFunctions) {
    char buffer[buffer_size];
    std::iota(buffer, buffer + buffer_size, 0);

    EXPECT_EQ(*checksum_begin(buffer), 0);
    EXPECT_EQ(*checksum_end(buffer), checksum_size);
    EXPECT_EQ(*message_begin(buffer), checksum_size);
    EXPECT_EQ(buffer + buffer_size, message_end(buffer));
}

void compare_bytes(const char* b1, const char* b2, size_t n) {
    for (size_t i = 0; i < n; i++) {
        EXPECT_EQ(*(b1 + i), *(b2 + i));
    }
}

TEST(TestBufferFunctions, InsertChecksum) {
    {
        char buffer[buffer_size];
        std::iota(buffer, buffer + buffer_size, 0);
        insert_checksum(buffer);
        EXPECT_TRUE(checksum_valid(buffer));
    }

    // clang-format off
    constexpr size_t num_messages = 5;
    const char* buffer_messages[num_messages] = {
        "JDXQH5AQQFxMPiYxSHpKAtbJtx73vleacZud9y0OT1HbDx20BFOxjhd9V29v8MwaWraEczEkbgn3zxhSsPnMCa5iZKRAB1PZIYp3owswAbQGQcJpGYBZPSRd4rd1b3DTIsJrQa8kKHarxdsp3xq045SGvveyuk2QEKGVTFGYLp0eccvmcEIV2S1BUqD1FTbeBNtETZLYHTefwA0idESxziNTSgfT58vT",
        "4k41JUqc3AAs4q7Fw4LukVpDlWf3fbRDknsDt7B7kmeSay6xfnPM2GMK5KLcBMvOsLzqTLOok3EM9fghmDbHVpPYfQVHBDPBr5aB5pTAeNlJ8pYHhAqoTbIrMVRaDruNKLJ6dDwptE0oPSYv6rpO1zBrSH0u0ZYvYCkTonLlBnjs98gA2BAeGz4SwmVUKetwiF1GjqSSp6asHzWk0lD7nathFPf8IxP3",
        "6avI9HCGlyLr6YWtysHxmeXXjQUCBAGeDZHytO8LkMkydrIQi5HMqFvjP6VDARWGAzl75alGx7s0aUtSgbxbXwOIaUt86Av9AGJ0vGJAK25BRAAR7gtfXnBRnXiqRXuKQ92DVRco175JjYq2HUXjADTDIHMq8jj1RN9x5E79Q9M2vOAk1spsKCQLcjMqdCAk7TwFT9HDupbfzVi7liocd3QGJsC0FOst",
        "PuPZ1WTaiQ9aU4GjPhAT4itcrwNwHL25lVq8FfSoShWivaCYvoqZWVA3WxvkkegepzfcuBG28UVTSxJ6JtwZIY6XMF2cE96WFjUAqDO2QcdKpcsPOqpoSGrNp5zsaxdb8pILkUw6GeUpB9ViKbfsDzrA5mJHHaXmpYMMiiQ5SY20VxynZR2jo3lv3hMzl2Ae9FIGwM887Zhnlg6YChrccYs7EDqOQy9G",
        "JhRAfZiartV0tpDBoKCHIX7h4G7RVgqSO2eKso0npR2jJmdJBi0wIFOyxvYBBlrBw32TE8wAaMIldUP3ndi4OiXNGCzhMllzWUG1Ldnr3bVpzfa5Qv1R1ktz20kU3GbJDlAO0V35Sh4xtR6hkRMOVGjbQDoBjbvjxUEWu8FDP6S3RKuVLj7eiTjfDl6XP5g8N87d8zredDTi2loaCpeqyzp9gsMERS9I"
    };

    const std::string expected_digests[num_messages] = {
        "6d1a361e0fa4b6a3a7993634e1a4b0bb6f25bc0c17f6ac78b5d85e450415446e",
        "2cd5ceedc84a3be388f8384f9d6c10335aad5af2c5b01dfdfd7af75b5474e330",
        "2b8a86625b8ee5ced33c7ad4d611cdbbfe20073336214d91c5144ad7463a8a23",
        "5e4480703d6266849ef46e7893e8589a486cac833b9fef0bb8a40cb0e61e89c6",
        "873e92d44cea3fcb85cbb55f3aec3cf537726db2cc68fab4d695da2615bb6141"
    };
    // clang-format on

    for (size_t message_idx = 0; message_idx < num_messages; message_idx++) {
        char        buffer[buffer_size];
        const auto* buffer_message = buffer_messages[message_idx];
        memcpy(message_begin(buffer), buffer_message, buffer_size - checksum_size);
        insert_checksum(buffer);

        auto* buffer_unsigned = reinterpret_cast<const unsigned char*>(buffer);

        std::string digest_hex_string =
            picosha2::bytes_to_hex_string(buffer_unsigned, buffer_unsigned + checksum_size);

        // Check that the checksum was properly placed within the buffer
        EXPECT_TRUE(checksum_valid(buffer));
        // Check that the message was properly hashed
        EXPECT_EQ(digest_hex_string, expected_digests[message_idx]);
    }
}

TEST(TestBufferFunctions, ValidationFailInvalidMessage) {
    {
        char buffer[buffer_size];
        std::iota(buffer, buffer + buffer_size, 0);
        insert_checksum(buffer);

        // Change message bytes slightly
        *message_begin(buffer) += 1;

        EXPECT_FALSE(checksum_valid(buffer));
    }

    {
        char buffer[buffer_size];
        std::iota(buffer, buffer + buffer_size, 0);
        insert_checksum(buffer);

        // Change message bytes slightly
        *(message_begin(buffer) + 20) += 1;
        *(message_begin(buffer) + 30) += 20;

        EXPECT_FALSE(checksum_valid(buffer));
    }
}

TEST(TestBufferFunctions, ValidationFailInvalidChecksum) {
    {
        char buffer[buffer_size];
        std::iota(buffer, buffer + buffer_size, 0);
        insert_checksum(buffer);

        // Change checksum bytes slightly
        *checksum_begin(buffer) += 1;

        EXPECT_FALSE(checksum_valid(buffer));
    }

    {
        char buffer[buffer_size];
        std::iota(buffer, buffer + buffer_size, 0);
        insert_checksum(buffer);

        // Change checksum bytes slightly
        *(checksum_begin(buffer) + 5) += 1;
        *(checksum_begin(buffer) + 10) += 20;

        EXPECT_FALSE(checksum_valid(buffer));
    }
}

TEST(TestBufferFunctions, ValidationFailInvalidMessageAndInvalidChecksum) {
    {
        char buffer[buffer_size];
        std::iota(buffer, buffer + buffer_size, 0);
        insert_checksum(buffer);

        // Change checksum bytes slightly
        *checksum_begin(buffer) += 1;
        // Change message bytes slightly
        *message_begin(buffer) += 1;

        EXPECT_FALSE(checksum_valid(buffer));
    }

    {
        char buffer[buffer_size];
        std::iota(buffer, buffer + buffer_size, 0);
        insert_checksum(buffer);

        // Change checksum bytes slightly
        *(checksum_begin(buffer) + 5) += 1;
        *(checksum_begin(buffer) + 10) += 20;
        // Change message bytes slightly
        *(message_begin(buffer) + 20) += 1;
        *(message_begin(buffer) + 30) += 20;

        EXPECT_FALSE(checksum_valid(buffer));
    }
}

} // namespace