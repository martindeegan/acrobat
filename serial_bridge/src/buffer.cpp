#include <cstring>

#include <serial_bridge/buffer.hpp>

namespace acrobat::serial_bridge {

uint8_t* checksum_begin(uint8_t* buffer) {
    return buffer;
}

uint8_t* checksum_end(uint8_t* buffer) {
    return buffer + checksum_size;
}

uint8_t* message_begin(uint8_t* buffer) {
    return buffer + checksum_size;
}

uint8_t* message_end(uint8_t* buffer) {
    return buffer + buffer_size;
}

void compute_checksum(uint8_t* buffer, uint8_t* output) {
    picosha2::hash256(
        message_begin(buffer), message_end(buffer), checksum_begin(output), checksum_end(output));
}

void insert_checksum(uint8_t* buffer) {
    compute_checksum(buffer, checksum_begin(buffer));
}

bool checksum_valid(uint8_t* buffer) {
    uint8_t checksum[checksum_size];
    compute_checksum(buffer, checksum_begin(checksum));

    return std::memcmp(checksum_begin(buffer), checksum_begin(checksum), checksum_size) == 0;
}

void clear_buffer(uint8_t* buffer) {
    std::memset(message_begin(buffer), 0, buffer_size - checksum_size);
}

} // namespace acrobat::serial_bridge