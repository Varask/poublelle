//
// Created by guillaume.benhamou on 04/05/2023.
//

#ifndef POUBLELLE_COMMON_FUNCTION_H
#define POUBLELLE_COMMON_FUNCTION_H
#include "common_function.h"
#include <stdint.h>

uint16_t fuse_bytes(uint8_t byte1, uint8_t byte2) {
    uint16_t result = 0;

    // Shift byte1 by 8 bits to the left and bitwise OR with byte2
    result = (byte1 << 8) | byte2;

    return result;
}

#endif //POUBLELLE_COMMON_FUNCTION_H