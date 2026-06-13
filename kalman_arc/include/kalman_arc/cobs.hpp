#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

std::vector<uint8_t> cobs_encode(const uint8_t* input, size_t length);
std::vector<uint8_t> cobs_decode(const uint8_t* input, size_t length);
