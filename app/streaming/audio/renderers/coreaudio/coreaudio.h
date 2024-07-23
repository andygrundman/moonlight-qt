#pragma once

#include "../renderer.h"
#include "au_spatial_renderer.h"
#include "AllocatedAudioBufferList.h"
#include "TPCircularBuffer.h"

#include <AudioUnit/AudioUnit.h>
#include <AudioToolbox/AudioToolbox.h>

class CoreAudioRenderer : public IAudioRenderer
{
public:
    CoreAudioRenderer();
    ~CoreAudioRenderer();

    bool prepareForPlayback(const OPUS_MULTISTREAM_CONFIGURATION* opusConfig) override;
    virtual void updateMetrics() override;
    virtual void* getAudioBuffer(int* size) override;
    virtual bool submitAudio(int bytesWritten) override;
    virtual int getCapabilities();
    virtual AudioFormat getAudioBufferFormat() override;
    const char * getRendererName() { return "CoreAudio"; }

    friend OSStatus renderCallbackDirect(void *, AudioUnitRenderActionFlags *, const AudioTimeStamp *, uint32_t, uint32_t, AudioBufferList *);
    friend OSStatus renderCallbackSpatial(void *, AudioUnitRenderActionFlags *, const AudioTimeStamp *, uint32_t, uint32_t, AudioBufferList *);
    friend OSStatus onDeviceOverload(AudioObjectID, UInt32, const AudioObjectPropertyAddress *, void *);
    friend OSStatus onAudioNeedsReinit(AudioObjectID, UInt32, const AudioObjectPropertyAddress *, void *);
    friend OSStatus onAudioNeedsReinit(UInt32, AudioObjectID, UInt32, const AudioObjectPropertyAddress *, void *);

private:
    bool initAudioUnit();
    bool initRingBuffer();
    bool initListeners();
    void deinitListeners();
    bool setCallback(AURenderCallback);
    void clearCallback();
    void cleanup();
    AUSpatialMixerOutputType getSpatialMixerOutputType();
    void setOutputDeviceName(CFStringRef);

    AudioUnit m_OutputAU;
    AUSpatialRenderer m_SpatialAU;

    // output device metadata
    AudioDeviceID m_OutputDeviceID;
    AudioStreamBasicDescription m_OutputASBD;
    char *m_OutputDeviceName;
    char m_OutputTransportType[5];
    char m_OutputDataSource[5];
    const OPUS_MULTISTREAM_CONFIGURATION* m_opusConfig;

    // buffers
    TPCircularBuffer m_RingBuffer;
    AllocatedAudioBufferList m_SpatialBuffer;
    double m_AudioPacketDuration;
    uint32_t m_BufferFrameSize;

    // latency
    double m_OutputHardwareLatency;
    double m_TotalSoftwareLatency;
    double m_OutputSoftwareLatencyMin;
    double m_OutputSoftwareLatencyMax;

    // internal device state
    bool m_needsReinit;
    bool m_Spatial;
    uint32_t m_SpatialOutputType;
    uint64_t m_LastDebugOutputTime;

    // stats
    double m_LastSampleTime;
    uint32_t m_LastNumFrames;
    uint32_t m_BufferSize;
    uint32_t m_BufferFilledBytes;
    void statsIncDeviceOverload();
    void statsTrackRender(uint64_t, const AudioTimeStamp *, uint32_t);
};
