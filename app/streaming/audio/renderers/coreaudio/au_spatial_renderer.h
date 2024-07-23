#pragma once

#include "TPCircularBuffer.h"

#include <AudioUnit/AudioUnit.h>
#include <AudioToolbox/AudioToolbox.h>

class AUSpatialRenderer
{
public:
    AUSpatialRenderer();
    ~AUSpatialRenderer();

    void setRingBufferPtr(const TPCircularBuffer* __nonnull buffer);
    bool setup(AUSpatialMixerOutputType outputType, float sampleRate, int inChannelCount);
    OSStatus setStreamFormatAndACL(float inSampleRate, AudioChannelLayoutTag inLayoutTag, AudioUnitScope inScope, AudioUnitElement inElement);
    OSStatus setOutputType(AUSpatialMixerOutputType outputType);
    void process(AudioBufferList* __nullable outputABL, const AudioTimeStamp* __nullable inTimeStamp, float inNumberFrames);

    friend OSStatus inputCallback(void * _Nonnull,
                    AudioUnitRenderActionFlags *_Nullable,
                    const AudioTimeStamp * _Nullable,
                    uint32_t, uint32_t,
                    AudioBufferList * _Nonnull);

    uint32_t m_HeadTracking;
    uint32_t m_PersonalizedHRTF;

private:
    AudioUnit _Nonnull m_Mixer;
    const TPCircularBuffer* _Nonnull m_RingBufferPtr; // pointer to RingBuffer in the outer CoreAudioRenderer
};
