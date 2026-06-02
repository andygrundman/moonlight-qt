#pragma once

#include "renderer.h"
#include "SDL_compat.h"

class SdlAudioRenderer : public IAudioRenderer
{
public:
    SdlAudioRenderer();

    virtual ~SdlAudioRenderer();

    virtual bool prepareForPlayback(const OPUS_MULTISTREAM_CONFIGURATION* opusConfig) override;

    virtual void* getAudioBuffer(int* size) override;

    virtual bool submitAudio(int bytesWritten) override;

    virtual AudioFormat getAudioBufferFormat() override;

    virtual void updateMetrics() override;

private:
    SDL_AudioDeviceID m_AudioDevice;
    void* m_AudioBuffer;
    Uint32 m_FrameSize;
    Uint32 m_FrameDurationMs;
    Uint32 m_DropCount;
    std::atomic<int> m_QueuedAudioSize;
};
