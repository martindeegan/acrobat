#include "testing_utils.hpp"

namespace testing_utils {

Buffer create_buffer(const char* contents, size_t length) {
    Buffer buffer;
    buffer.fill('a');
    auto buffer_begin = buffer.begin() + checksum_size;
    std::copy(contents, contents + length, buffer_begin);
    return buffer;
}

} // namespace testing_utils
