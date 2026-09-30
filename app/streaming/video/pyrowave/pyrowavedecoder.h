#pragma once

#include "pyrowaveframing.h"

#include <memory>
#include <string>
#include <vector>

extern "C" {
#include <libavutil/frame.h>
}

// Native Metal decoder. Decode is called from one thread; output AVFrames
// may be released on any thread. Textures stay alive until their GPU users finish.
class PyroWaveMetalDecoder
{
public:
    struct Config {
        int width = 0;
        int height = 0;
        bool chroma444 = false;
        bool tenBit = false;
        void* metalDevice = nullptr;
    };

    PyroWaveMetalDecoder();
    ~PyroWaveMetalDecoder();

    PyroWaveMetalDecoder(const PyroWaveMetalDecoder&) = delete;
    PyroWaveMetalDecoder& operator=(const PyroWaveMetalDecoder&) = delete;

    bool initialize(const Config& config);

    // Submits decode work and returns retained GPU textures. The renderer waits
    // for readyEvent before sampling. Input is validated before GPU submission.
    bool decode(const uint8_t* data, size_t size,
                const std::vector<PyroWaveFraming::Segment>& packets, size_t criticalPackets,
                AVFrame* frame);

    // Why the last call failed, for logging.
    const std::string& lastError() const { return m_LastError; }

private:
    struct Impl;
    std::unique_ptr<Impl> m_Impl;
    std::string m_LastError;
};
