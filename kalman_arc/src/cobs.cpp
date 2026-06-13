#include "kalman_arc/cobs.hpp"

// Code from https://github.com/anatolianroverchallenge/rscp
std::vector<uint8_t> cobs_encode(const uint8_t *input, size_t length) {
	std::vector<uint8_t> output;
	output.reserve(length + length / 254 + 1);

	size_t  code_index = 0;
	uint8_t code       = 1;
	output.push_back(0);

	for (size_t index = 0; index < length; ++index) {
		if (input[index] == 0) {
			output[code_index] = code;
			code_index         = output.size();
			output.push_back(0);
			code = 1;
		} else {
			output.push_back(input[index]);
			++code;

			if (code == 0xFF) {
				output[code_index] = code;
				code_index         = output.size();
				output.push_back(0);
				code = 1;
			}
		}
	}

	output[code_index] = code;
	return output;
}

std::vector<uint8_t> cobs_decode(const uint8_t *input, size_t length) {
	std::vector<uint8_t> output;
	output.reserve(length); // Reserve enough space for the decoded data

	size_t index = 0;
	while (index < length) {
		uint8_t code = input[index];

		// Stop decoding on invalid code byte or if bounds are exceeded
		if (code == 0 || index + code > length + 1) {
			break;
		}

		// Copy the next (code - 1) bytes as literal data
		for (uint8_t i = 1; i < code; ++i) {
			output.push_back(input[index + i]);
		}

		// Append a zero byte if the code is not 0xFF (which means no zero was
		// encoded)
		if (code != 0xFF && index + code < length) {
			output.push_back(0);
		}

		index += code;
	}

	return output;
}
