#pragma once

#include "TPCircularBuffer.h"

#include <AudioUnit/AudioUnit.h>
#include <AudioToolbox/AudioToolbox.h>

#include <Limelight.h>

typedef void (^SimpleBlock)();

class AUSpatialRenderer
{
public:
    AUSpatialRenderer();
    ~AUSpatialRenderer();
    void cleanup();
    void clearCallback();

    double getAudioUnitLatency();
    bool getHeadTracking();
    void setHeadTracking(bool enabled);
    void setRingBufferPtr(const TPCircularBuffer* __nonnull buffer);
    void setStatsTrackRenderBlock(SimpleBlock _Nonnull);
    bool setup(AUSpatialMixerOutputType outputType, float sampleRate, int inChannelCount);
    OSStatus setStreamFormatAndACL(float inSampleRate, AudioChannelLayoutTag inLayoutTag, AudioUnitScope inScope, AudioUnitElement inElement);
    OSStatus setOutputType(AUSpatialMixerOutputType outputType);
    OSStatus process(AudioBufferList* __nullable outputABL, AudioUnitRenderActionFlags* __nonnull ioActionFlags, const AudioTimeStamp* __nullable inTimestamp, uint32_t inNumberFrames);

    friend OSStatus inputCallback(void * _Nonnull,
                    AudioUnitRenderActionFlags *_Nullable,
                    const AudioTimeStamp * _Nullable,
                    uint32_t, uint32_t,
                    AudioBufferList * _Nonnull);

    uint32_t m_PersonalizedHRTF;

private:
    AudioUnit _Nonnull m_Mixer;
    const TPCircularBuffer* _Nonnull m_RingBufferPtr; // pointer to RingBuffer in the outer CoreAudioRenderer
    SimpleBlock _Nonnull m_StatsTrackRenderBlock;

    bool m_Initialized = false;
    double m_AudioUnitLatency;
};
