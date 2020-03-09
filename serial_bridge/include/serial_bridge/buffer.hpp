#pragma once

#include "picosha2.h"

namespace acrobat::serial_bridge {

constexpr size_t checksum_size         = picosha2::k_digest_size;
constexpr size_t end_of_message_length = sizeof(uint8_t);
constexpr size_t buffer_size           = 256;

uint8_t* checksum_begin(uint8_t* buffer);
uint8_t* checksum_end(uint8_t* buffer);
uint8_t* message_begin(uint8_t* buffer);
uint8_t* message_end(uint8_t* buffer);

void compute_checksum(uint8_t* buffer, uint8_t* output);
void insert_checksum(uint8_t* buffer);
bool checksum_valid(uint8_t* buffer);

void clear_buffer(uint8_t* buffer);

} // namespace acrobat::serial_bridge
