#include <boost/crc.hpp>

#include <serial_bridge/encodings.hpp>

namespace acrobat::serial_bridge {

short compute_checksum(const Buffer& buffer) {
    // USB crc16
    boost::crc_optimal<16, 0x8005, 0xFFFF, 0, true, true> result;
    result.process_bytes(buffer.data() + checksum_size, buffer_size - checksum_size);
    return result.checksum();
}

void finalize_message(Buffer& buffer, Buffer::iterator buffer_begin) {
    // Set end of message
    constexpr char end_of_message = '\0';
    *buffer_begin                 = end_of_message;

    // Compute checksum and copy into message
    const auto  checksum       = compute_checksum(buffer);
    const char* checksum_bytes = reinterpret_cast<const char*>(&checksum);
    std::copy(checksum_bytes, checksum_bytes + checksum_size, buffer.begin());
}

bool check_message(const Buffer& buffer) {
    const short* checksum_bytes    = reinterpret_cast<const short*>(buffer.data());
    const short  message_checksum  = *checksum_bytes;
    const auto   computed_checksum = compute_checksum(buffer);

    return computed_checksum == message_checksum;
}

} // namespace acrobat::serial_bridge