#include "testing_utils.hpp"

namespace testing_utils {

Buffer create_buffer(std::string contents) {
    Buffer buffer{0};
    auto   buffer_begin = buffer.begin() + checksum_size;
    std::copy(contents.begin(), contents.end(), buffer_begin);
    return buffer;
}

} // namespace testing_utils
