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

    bool prepareForPlayback(const OPUS_MULTISTREAM_CONFIGURATION* opusConfig);
    virtual void* getAudioBuffer(int* size);
    virtual bool submitAudio(int bytesWritten);
    virtual int getCapabilities();
    virtual AudioFormat getAudioBufferFormat();

    virtual void snapshotAudioStats(AUDIO_STATS &snapshot);
    virtual void addAudioStats(AUDIO_STATS &src, AUDIO_STATS &dst);
    virtual void stringifyAudioStats(AUDIO_STATS &stats, char* output, int length);
    virtual void logGlobalAudioStats();

    friend OSStatus renderCallbackDirect(void *, AudioUnitRenderActionFlags *, const AudioTimeStamp *, uint32_t, uint32_t, AudioBufferList *);
    friend OSStatus renderCallbackSpatial(void *, AudioUnitRenderActionFlags *, const AudioTimeStamp *, uint32_t, uint32_t, AudioBufferList *);
    friend OSStatus onDeviceOverload(AudioObjectID, UInt32, const AudioObjectPropertyAddress *, void *);
    friend OSStatus onAudioNeedsReinit(AudioObjectID, UInt32, const AudioObjectPropertyAddress *, void *);

private:
    bool initAudioUnit();
    bool initRingBuffer();
    bool initListeners();
    void deinitListeners();
    bool setCallback(AURenderCallback);
    void stop();
    void cleanup();
    AUSpatialMixerOutputType getSpatialMixerOutputType();
    void setOutputDeviceName(CFStringRef);

    AudioUnit m_OutputAU;
    AUSpatialRenderer m_SpatialAU;

    // input stream metadata
    const OPUS_MULTISTREAM_CONFIGURATION* m_opusConfig;

    // output device metadata
    AudioDeviceID m_OutputDeviceID;
    AudioStreamBasicDescription m_OutputASBD;
    char *m_OutputDeviceName;
    double m_OutputHardwareLatency;
    char m_OutputTransportType[5];
    char m_OutputDataSource[5];

    // buffers
    TPCircularBuffer m_RingBuffer;
    AllocatedAudioBufferList m_SpatialBuffer;
    double m_AudioPacketDuration;
    double m_OutputSoftwareLatency;
    double m_OutputSoftwareLatencyCurrent;
    double m_OutputSoftwareLatencyMin;
    double m_OutputSoftwareLatencyMax;

    // internal device state
    bool m_needsReinit;
    bool m_Spatial;
    uint32_t m_SpatialOutputType;

    // stats
    uint32_t m_BufferSize;
    uint32_t m_BufferFilledBytes;
    void statsIncRenderedPackets();
    void statsAddOpusBytesReceived(int);
    void statsIncDeviceOverload();
};
