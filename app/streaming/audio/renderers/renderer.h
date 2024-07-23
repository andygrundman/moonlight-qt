#pragma once

#include <Limelight.h>
#include <QtGlobal>
#include <SDL.h>

typedef struct _AUDIO_STATS {
    uint32_t opusBytesReceived;
    uint32_t totalPackets;
    uint32_t networkDroppedPackets;
    uint32_t fellBehindAndDroppedPackets;
    uint32_t decodedPackets;
    uint32_t renderedPackets;
    uint32_t deviceOverloadCount;
    uint32_t lastRtt;
    uint32_t lastRttVariance;
    uint32_t measurementStartTimestamp;
    float opusBitsPerSec;
    float packetsPerSec;
} AUDIO_STATS, *PAUDIO_STATS;

class IAudioRenderer
{
public:
    IAudioRenderer() {
        SDL_zero(m_ActiveWndAudioStats);
        SDL_zero(m_LastWndAudioStats);
        SDL_zero(m_GlobalAudioStats);
    }

    virtual ~IAudioRenderer() {}

    virtual bool prepareForPlayback(const OPUS_MULTISTREAM_CONFIGURATION* opusConfig) = 0;

    virtual void* getAudioBuffer(int* size) = 0;

    // Return false if an unrecoverable error has occurred and the renderer must be reinitialized
    virtual bool submitAudio(int bytesWritten) = 0;

    virtual int getCapabilities() = 0;

    virtual void remapChannels(POPUS_MULTISTREAM_CONFIGURATION) {
        // Use default channel mapping:
        // 0 - Front Left
        // 1 - Front Right
        // 2 - Center
        // 3 - LFE
        // 4 - Surround Left
        // 5 - Surround Right
    }

    enum class AudioFormat {
        Sint16NE,  // 16-bit signed integer (native endian)
        Float32NE, // 32-bit floating point (native endian)
    };
    virtual AudioFormat getAudioBufferFormat() = 0;

    int getAudioBufferSampleSize() {
        switch (getAudioBufferFormat()) {
        case IAudioRenderer::AudioFormat::Sint16NE:
            return sizeof(short);
        case IAudioRenderer::AudioFormat::Float32NE:
            return sizeof(float);
        default:
            Q_UNREACHABLE();
        }
    }

    AUDIO_STATS & getActiveWndAudioStats() {
        return m_ActiveWndAudioStats;
    }

    // optional for backends wishing to report stats
    virtual void snapshotAudioStats(AUDIO_STATS &) { return; }
    virtual void addAudioStats(AUDIO_STATS &, AUDIO_STATS &) { return; }
    virtual void stringifyAudioStats(AUDIO_STATS &, char *, int) { return; }
    virtual void logGlobalAudioStats() { return; }
    virtual void statsIncRenderedPackets() { return; }
    virtual void statsAddOpusBytesReceived(int) { return; }
    virtual void statsIncDeviceOverload() { return; }

protected:
    AUDIO_STATS m_ActiveWndAudioStats;
    AUDIO_STATS m_LastWndAudioStats;
    AUDIO_STATS m_GlobalAudioStats;
};
