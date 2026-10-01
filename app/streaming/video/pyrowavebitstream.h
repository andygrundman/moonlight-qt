#pragma once

#include <Limelight.h>
#include "pyrowaveframing.h"
#include <cstring>
#include <new>
#include <vector>

// Compact intact frames in either Vibeshine framing, removing envelope lengths
// and padding while keeping the codec records unchanged.
inline bool unpackPyroWaveBitstream(std::vector<uint32_t>& words, int width, int height, bool yuv444)
{
    PyroWaveFraming::Frame frame;
    std::string error;
    if (!PyroWaveFraming::parse(reinterpret_cast<const uint8_t*>(words.data()),
                                words.size() * sizeof(uint32_t), {width, height, yuv444}, frame, error)) {
        return false;
    }
    size_t output = 0;
    for (const auto& span : frame.spans) {
        std::memmove(words.data() + output, reinterpret_cast<const uint8_t*>(words.data()) + span.offset, span.size);
        output += span.size / sizeof(uint32_t);
    }
    words.resize(output);
    return true;
}

// RTP fragments may split PyroWave blocks, so parse the complete decode unit.
inline bool assemblePyroWaveDecodeUnit(PDECODE_UNIT du, std::vector<uint32_t>& words,
                                       std::vector<PyroWaveFraming::Segment>* segments = nullptr)
{
    constexpr int MaxFrameBytes = 64 * 1024 * 1024;
    if (!du || du->fullLength < 8 || du->fullLength > MaxFrameBytes ||
            du->fullLength % sizeof(uint32_t) != 0) {
        return false;
    }

    size_t length = 0;
    for (auto entry = du->bufferList; entry; entry = entry->next) {
        if (!entry->data || entry->length <= 0 ||
                size_t(entry->length) > size_t(du->fullLength) - length) {
            return false;
        }
        length += entry->length;
    }
    if (length != size_t(du->fullLength)) {
        return false;
    }
    try {
        words.resize(length / sizeof(uint32_t));
        if (segments) {
            segments->clear();
            size_t offset = 0;
            for (auto entry = du->bufferList; entry; entry = entry->next) {
                segments->push_back({offset, size_t(entry->length), entry->bufferType == BUFFER_TYPE_LOST,
                                     entry->bufferType == BUFFER_TYPE_RECORD_START});
                offset += entry->length;
            }
        }
    }
    catch (const std::bad_alloc&) {
        return false;
    }
    auto output = reinterpret_cast<uint8_t*>(words.data());
    for (auto entry = du->bufferList; entry; entry = entry->next) {
        std::memcpy(output, entry->data, entry->length);
        output += entry->length;
    }
    return true;
}
