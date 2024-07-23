#include "coreaudio.h"
#include "coreaudio_helpers.h"

#if TARGET_OS_OSX
#include <IOKit/audio/IOAudioTypes.h>
#endif

#include <QtGlobal>
#include <SDL.h>
#include <string>

#define kRingBufferMaxSeconds 0.030

CoreAudioRenderer::CoreAudioRenderer()
    : m_SpatialBuffer(2, 4096)
{
    DEBUG_TRACE("CoreAudioRenderer construct");

    AudioComponentDescription description;
    description.componentType = kAudioUnitType_Output;
#if TARGET_OS_IPHONE
    description.componentSubType = kAudioUnitSubType_RemoteIO;
#elif TARGET_OS_OSX
    description.componentSubType = kAudioUnitSubType_HALOutput;
#endif
    description.componentManufacturer = kAudioUnitManufacturer_Apple;
    description.componentFlags = 0;
    description.componentFlagsMask = 0;

    AudioComponent comp = AudioComponentFindNext(NULL, &description);
    if (!comp) {
        return;
    }

    OSStatus status = AudioComponentInstanceNew(comp, &m_OutputAU);
    if (status != noErr) {
        CA_LogError(status, "Failed to create an instance of HALOutput or RemoteIO");
        throw std::runtime_error("Failed to create an instance of HALOutput or RemoteIO");
    }
}

CoreAudioRenderer::~CoreAudioRenderer()
{
    DEBUG_TRACE("CoreAudioRenderer destruct");
    cleanup();
}

void CoreAudioRenderer::stop()
{
    DEBUG_TRACE("CoreAudioRenderer stop");
    if (m_OutputAU != nullptr) {
        AudioOutputUnitStop(m_OutputAU);
    }
}

void CoreAudioRenderer::cleanup()
{
    DEBUG_TRACE("CoreAudioRenderer cleanup");
    stop();

    if (m_OutputAU != nullptr) {
        AudioUnitUninitialize(m_OutputAU);
        AudioComponentInstanceDispose(m_OutputAU);

        // Must be destroyed after the stream is stopped
        TPCircularBufferCleanup(&m_RingBuffer);
    }

    if (m_OutputDeviceID) {
        deinitListeners();
        m_OutputDeviceID = 0;
    }

    if (m_OutputDeviceName) {
        free(m_OutputDeviceName);
    }
}

int CoreAudioRenderer::getCapabilities()
{
    return 0; // CAPABILITY_DIRECT_SUBMIT feels worse than decoding in a separate thread
}

IAudioRenderer::AudioFormat CoreAudioRenderer::getAudioBufferFormat()
{
    return AudioFormat::Float32NE;
}

// called frequently for the stats overlay to provide realtime recent stats
void CoreAudioRenderer::snapshotAudioStats(AUDIO_STATS &snapshot)
{
    addAudioStats(m_LastWndAudioStats, snapshot);
    addAudioStats(m_ActiveWndAudioStats, snapshot);
}

void CoreAudioRenderer::statsIncRenderedPackets()
{
    m_ActiveWndAudioStats.renderedPackets++;
}

void CoreAudioRenderer::statsAddOpusBytesReceived(int size)
{
    m_ActiveWndAudioStats.opusBytesReceived += size;
}

void CoreAudioRenderer::statsIncDeviceOverload()
{
    m_ActiveWndAudioStats.deviceOverloadCount++;
}

void CoreAudioRenderer::addAudioStats(AUDIO_STATS& src, AUDIO_STATS& dst)
{
    dst.opusBytesReceived += src.opusBytesReceived;
    dst.totalPackets      += src.totalPackets;
    dst.networkDroppedPackets += src.networkDroppedPackets;
    dst.decodedPackets    += src.decodedPackets;
    dst.renderedPackets   += src.renderedPackets;

    dst.fellBehindAndDroppedPackets = dst.totalPackets - dst.decodedPackets;

    if (!LiGetEstimatedRttInfo(&dst.lastRtt, &dst.lastRttVariance)) {
        dst.lastRtt = 0;
        dst.lastRttVariance = 0;
    }
    else {
        // Our logic to determine if RTT is valid depends on us never
        // getting an RTT of 0. ENet currently ensures RTTs are >= 1.
        SDL_assert(dst.lastRtt > 0);
    }

    uint32_t now = SDL_GetTicks();

    // Initialize the measurement start point if this is the first video stat window
    if (!dst.measurementStartTimestamp) {
        dst.measurementStartTimestamp = src.measurementStartTimestamp;
    }

    // // The following code assumes the global measure was already started first
    SDL_assert(dst.measurementStartTimestamp <= src.measurementStartTimestamp);

    dst.packetsPerSec = (float)dst.totalPackets / ((float)(now - dst.measurementStartTimestamp));
    dst.opusBitsPerSec = (float)dst.opusBytesReceived * 8 / (float)(now - dst.measurementStartTimestamp);
}

void CoreAudioRenderer::stringifyAudioStats(AUDIO_STATS& stats, char *output, int length)
{
    double opusFrameSize = (double)m_opusConfig->samplesPerFrame / 48.0;
    double bufferSizeInPackets = (double)m_BufferSize / (m_opusConfig->samplesPerFrame * m_opusConfig->channelCount * 4);
    double bufferSizeInMs = bufferSizeInPackets * opusFrameSize;

    snprintf(
        output, length,
        "Audio stream: %s-channel Opus low-delay @ 48 kHz\n"
        "Output device: %s @ %.1f kHz, %u-channel\n"
        "Render mode: %s, device type: %s %s, %s\n"
        "Opus config: %s, frame size: %.1fms, bitrate: %dkbps\n"
        "Buffer: length: %0.1f packets (%.1fms), current %.1f%%\n"
        "Packet loss from network: %.2f%%, packets dropped due to resync: %.2f%%\n"
        "Latency: %0.2fms (network %dms, buffer %dms, hardware: %dms)\n",

        m_opusConfig->channelCount == 6 ? "5.1" : m_opusConfig->channelCount == 8 ? "7.1" : "2",

        m_OutputDeviceName,
        m_OutputASBD.mSampleRate / 1000.0,
        m_OutputASBD.mChannelsPerFrame,

        m_Spatial ? (m_SpatialAU.m_PersonalizedHRTF ? "personalized spatial" : "spatial") : "passthrough",
        !strcmp(m_OutputTransportType, "blue") ? "Bluetooth"
            : !strcmp(m_OutputTransportType, "bltn") ? "built-in"
            : !strcmp(m_OutputTransportType, "usb ") ? "USB"
            : !strcmp(m_OutputTransportType, "hdmi") ? "HDMI"
            : !strcmp(m_OutputTransportType, "airp") ? "AirPlay"
            : m_OutputTransportType,
        !strcmp(m_OutputDataSource      , "hdpn") ? "headphones"
            : !strcmp(m_OutputDataSource, "ispk") ? "internal speakers"
            : !strcmp(m_OutputDataSource, "espk") ? "external speakers"
            : m_OutputDataSource,
        m_Spatial && m_SpatialAU.m_HeadTracking ? "head-tracking: yes" : "",

        // Work out if we're getting high or low quality from Sunshine. coupled surround is designed for physical speakers
        ((m_opusConfig->channelCount == 2 && stats.opusBitsPerSec > 128) || !m_opusConfig->coupledStreams)
            ? "high quality (LAN)" // 512k stereo coupled, 1.5mbps 5.1 uncoupled, 2mbps 7.1 uncoupled
            : "normal quality",    // 96k stereo coupled, 256k 5.1 coupled, 450k 7.1 coupled
        opusFrameSize,
        (int)stats.opusBitsPerSec,

        bufferSizeInPackets,
        bufferSizeInMs,
        (double)m_BufferFilledBytes / m_BufferSize * 100.0,
        stats.totalPackets ? (float)stats.networkDroppedPackets / stats.totalPackets * 100 : 0.0,
        stats.totalPackets ? (float)stats.fellBehindAndDroppedPackets / stats.totalPackets * 100 : 0.0,

        stats.lastRtt + (m_OutputSoftwareLatencyCurrent + m_OutputHardwareLatency) * 1000.0,
        stats.lastRtt,
        (int)(m_OutputSoftwareLatencyCurrent * 1000.0),
        (int)(m_OutputHardwareLatency * 1000.0)
    );
}

void CoreAudioRenderer::logGlobalAudioStats()
{
    if (m_GlobalAudioStats.renderedPackets > 0) {
        char audioStatsStr[1024];
        stringifyAudioStats(m_GlobalAudioStats, audioStatsStr, sizeof(audioStatsStr));

        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "%s", "Global audio stats");
        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                    "----------------------------------------------------------\n%s",
                    audioStatsStr);
    }
}

OSStatus renderCallbackDirect(void *inRefCon,
                                     AudioUnitRenderActionFlags * ioActionFlags,
                                     const AudioTimeStamp * /*inTimeStamp*/,
                                     uint32_t /*inBusNumber*/,
                                     uint32_t /*inNumberFrames*/,
                                     AudioBufferList * ioData)
{
    //auto me = static_cast<CoreAudioRenderer *>(inRefCon);
    CoreAudioRenderer *me = (CoreAudioRenderer *)inRefCon;
    int bytesToCopy = ioData->mBuffers[0].mDataByteSize;
    float *targetBuffer = (float *)ioData->mBuffers[0].mData;

    // Pull audio from playthrough buffer
    uint32_t availableBytes;
    float *buffer = (float *)TPCircularBufferTail(&me->m_RingBuffer, &availableBytes);

    if ((int)availableBytes < bytesToCopy) {
        // write silence if not enough buffered data is available
        memset(targetBuffer, 0, bytesToCopy);
        *ioActionFlags |= kAudioUnitRenderAction_OutputIsSilence;

       // This underrun is not always a problem, so it's not included in stats currently
    } else {
        memcpy(targetBuffer, buffer, qMin(bytesToCopy, (int)availableBytes));
        TPCircularBufferConsume(&me->m_RingBuffer, qMin(bytesToCopy, (int)availableBytes));

        me->statsIncRenderedPackets();
    }

    return noErr;
}

OSStatus renderCallbackSpatial(void *inRefCon,
                                      AudioUnitRenderActionFlags * /*ioActionFlags*/,
                                      const AudioTimeStamp *inTimeStamp,
                                      uint32_t /*inBusNumber*/,
                                      uint32_t inNumberFrames,
                                      AudioBufferList * ioData)
{
    CoreAudioRenderer *me = (CoreAudioRenderer *)inRefCon;
    AudioBufferList *spatialBuffer = me->m_SpatialBuffer.get();

    // Set the byte size with the output audio buffer list.
    for (uint32_t i = 0; i < spatialBuffer->mNumberBuffers; i++) {
        spatialBuffer->mBuffers[i].mDataByteSize = inNumberFrames * 4;
    }

    // Process the input frames with the audio unit spatial mixer.
    me->m_SpatialAU.process(spatialBuffer, inTimeStamp, inNumberFrames);

    // Copy the temporary buffer to the output.
    for (uint32_t i = 0; i < spatialBuffer->mNumberBuffers; i++) {
        memcpy(ioData->mBuffers[i].mData, spatialBuffer->mBuffers[i].mData, inNumberFrames * 4);
    }

    me->statsIncRenderedPackets();

    return noErr;
}

bool CoreAudioRenderer::prepareForPlayback(const OPUS_MULTISTREAM_CONFIGURATION* opusConfig)
{
    OSStatus status = noErr;
    m_opusConfig = opusConfig;

    // Request the OS set our buffer close to the Opus packet size
    m_AudioPacketDuration = (opusConfig->samplesPerFrame / (opusConfig->sampleRate / 1000)) / 1000.0;
    m_OutputSoftwareLatency = m_AudioPacketDuration;

    if (!initAudioUnit()) {
        DEBUG_TRACE("initAudioUnit failed");
        return false;
    }

    if (!initRingBuffer()) {
        DEBUG_TRACE("initRingBuffer failed");
        return false;
    }

    if (!initListeners()) {
        DEBUG_TRACE("initListeners failed");
        return false;
    }

    m_Spatial = false;
    AUSpatialMixerOutputType outputType = getSpatialMixerOutputType();

    DEBUG_TRACE("CoreAudioRenderer getSpatialMixerOutputType = %d", outputType);

    // XXX: let the user choose this if they want
    if (opusConfig->channelCount > 2) {
        if (outputType != kSpatialMixerOutputType_ExternalSpeakers) {
            m_Spatial = true;
        }
    }

    // indicate the format our callback will provide samples in
    // If necessary, the OS takes care of resampling (but not downmixing, hmm)
    AudioStreamBasicDescription streamDesc;
    memset(&streamDesc, 0, sizeof(AudioStreamBasicDescription));
    streamDesc.mSampleRate       = m_opusConfig->sampleRate;
    streamDesc.mFormatID         = kAudioFormatLinearPCM;
    streamDesc.mFormatFlags      = kAudioFormatFlagIsFloat | kAudioFormatFlagsNativeEndian | kAudioFormatFlagIsPacked;
    streamDesc.mFramesPerPacket  = 1;
    streamDesc.mChannelsPerFrame = (uint32_t)opusConfig->channelCount;
    streamDesc.mBitsPerChannel   = 32;
    streamDesc.mBytesPerPacket   = 4 * opusConfig->channelCount;
    streamDesc.mBytesPerFrame    = streamDesc.mBytesPerPacket;

    if (m_Spatial) {
        // render audio for binaural headphones or built-in laptop speakers
        setCallback(renderCallbackSpatial);

        // this callback is non-interleaved
        streamDesc.mFormatFlags    |= kAudioFormatFlagIsNonInterleaved;
        streamDesc.mBytesPerPacket = 4;
        streamDesc.mBytesPerFrame  = 4;

        m_SpatialOutputType = outputType;

        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "CoreAudioRenderer is using spatial audio output");

        if (!m_SpatialAU.setup(outputType, opusConfig->sampleRate, opusConfig->channelCount)) {
            DEBUG_TRACE("m_SpatialAU.setup failed");
            return false;
        }
    } else {
        // direct passthrough of all channels for stereo and HDMI
        setCallback(renderCallbackDirect);

        SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "CoreAudioRenderer is using passthrough mode");
    }

    status = AudioUnitSetProperty(m_OutputAU, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Input, 0, &streamDesc, sizeof(streamDesc));
    if (status != noErr) {
        CA_LogError(status, "Failed to set output stream format");
        return false;
    }

    DEBUG_TRACE("CoreAudioRenderer start");
    status = AudioOutputUnitStart(m_OutputAU);
    if (status != noErr) {
        CA_LogError(status, "Failed to start output audio unit");
        return false;
    }

    return true;
}

bool CoreAudioRenderer::initAudioUnit()
{
    // Initialize the audio unit interface to begin configuring it.
    OSStatus status = AudioUnitInitialize(m_OutputAU);
    if (status != noErr) {
        CA_LogError(status, "Failed to initialize the output audio unit");
        return false;
    }

    /* macOS:
     * disable OutputAU input IO
     * enable OutputAU output IO
     * get system default output AudioDeviceID  (todo: allow user to choose specific device from list)
     * set OutputAU to AudioDeviceID
     * get device's AudioStreamBasicDescription (format, bit depth, samplerate, etc)
     * get device name
     * get output buffer frame size
     * get output buffer min/max
     * set output buffer frame size to
     */

#if TARGET_OS_OSX
    constexpr AudioUnitElement outputElement{0};
    constexpr AudioUnitElement inputElement{1};

    {
        uint32_t enableIO = 0;
        status = AudioUnitSetProperty(m_OutputAU, kAudioOutputUnitProperty_EnableIO, kAudioUnitScope_Input, inputElement, &enableIO, sizeof(enableIO));
        if (status != noErr) {
            CA_LogError(status, "Failed to disable the input on AUHAL");
            return false;
        }

        enableIO = 1;
        status = AudioUnitSetProperty(m_OutputAU, kAudioOutputUnitProperty_EnableIO, kAudioUnitScope_Output, outputElement, &enableIO, sizeof(enableIO));
        if (status != noErr) {
            CA_LogError(status, "Failed to enable the output on AUHAL");
            return false;
        }
    }

    {
        uint32_t size = sizeof(AudioDeviceID);
        AudioObjectPropertyAddress addr{kAudioHardwarePropertyDefaultOutputDevice, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
        status = AudioObjectGetPropertyData(AudioObjectID(kAudioObjectSystemObject), &addr, outputElement, nil, &size, &m_OutputDeviceID);
        if (status != noErr) {
            CA_LogError(status, "Failed to get the default output device");
            return false;
        }
        DEBUG_TRACE("CoreAudioRenderer default output device = %d", m_OutputDeviceID);
    }

    {
        // Set the current device to the default output device.
        // This should be done only after I/O is enabled on the output audio unit.
        status = AudioUnitSetProperty(m_OutputAU, kAudioOutputUnitProperty_CurrentDevice, kAudioUnitScope_Global, outputElement, &m_OutputDeviceID, sizeof(AudioDeviceID));
        if (status != noErr) {
            CA_LogError(status, "Failed to set the default output device");
            return false;
        }
    }

    {
        uint32_t streamFormatSize = sizeof(AudioStreamBasicDescription);
        AudioObjectPropertyAddress addr{kAudioDevicePropertyStreamFormat, kAudioDevicePropertyScopeOutput, kAudioObjectPropertyElementMain};
        status = AudioObjectGetPropertyData(m_OutputDeviceID, &addr, 0, nil, &streamFormatSize, &m_OutputASBD);
        if (status != noErr) {
            CA_LogError(status, "Failed to get output device AudioStreamBasicDescription");
            return false;
        }
        CA_PrintASBD("CoreAudioRenderer output format:", &m_OutputASBD);
    }

    {
        CFStringRef name;
        uint32_t nameSize = sizeof(CFStringRef);
        AudioObjectPropertyAddress addr{kAudioObjectPropertyName, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
        status = AudioObjectGetPropertyData(m_OutputDeviceID, &addr, 0, nil, &nameSize, &name);
        if (status != noErr) {
            CA_LogError(status, "Failed to get name of output device");
            return false;
        }
        setOutputDeviceName(name);
        CFRelease(name);
    }

    // Buffer:
    // The goal here is to set the system buffer to our desired value, which is currently in m_OutputSoftwareLatency.
    // First we get the current value, and the range of allowed values, set our value, and then query to find the actual value.
    // We also query the hardware latency (e.g. Bluetooth delay for AirPods), but this is just for fun

    {
        uint32_t bufferFrameSize = 0;
        uint32_t size = sizeof(uint32_t);
        AudioObjectPropertyAddress addr{kAudioDevicePropertyBufferFrameSize, kAudioObjectPropertyScopeOutput, kAudioObjectPropertyElementMain};
        status = AudioObjectGetPropertyData(m_OutputDeviceID, &addr, 0, nil, &size, &bufferFrameSize);
        if (status != noErr) {
            CA_LogError(status, "Failed to get the output device buffer frame size");
            return false;
        }
        m_OutputSoftwareLatencyCurrent = (double)bufferFrameSize / m_OutputASBD.mSampleRate;
        DEBUG_TRACE("CoreAudioRenderer output current BufferFrameSize %d (%0.2f s)", bufferFrameSize, m_OutputSoftwareLatencyCurrent);
    }

    {
        AudioValueRange avr;
        uint32_t size = sizeof(AudioValueRange);
        AudioObjectPropertyAddress addr{kAudioDevicePropertyBufferFrameSizeRange, kAudioObjectPropertyScopeOutput, kAudioObjectPropertyElementMain};
        status = AudioObjectGetPropertyData(m_OutputDeviceID, &addr, 0, nil, &size, &avr);
        if (status != noErr) {
            CA_LogError(status, "Failed to get the output device buffer frame size range");
            return false;
        }
        m_OutputSoftwareLatencyMin = avr.mMinimum / m_OutputASBD.mSampleRate;
        m_OutputSoftwareLatencyMax = avr.mMaximum / m_OutputASBD.mSampleRate;
        DEBUG_TRACE("CoreAudioRenderer output BufferFrameSizeRange: %.0f - %.0f", avr.mMinimum, avr.mMaximum);
    }

    {
        uint32_t latencyFrames;
        uint32_t size = sizeof(uint32_t);
        AudioObjectPropertyAddress addr{kAudioDevicePropertyLatency, kAudioObjectPropertyScopeOutput, kAudioObjectPropertyElementMain};
        status = AudioObjectGetPropertyData(m_OutputDeviceID, &addr, 0, nil, &size, &latencyFrames);
        if (status != noErr) {
            CA_LogError(status, "Failed to get the output device hardware latency");
            return false;
        }
        m_OutputHardwareLatency = (double)latencyFrames / m_OutputASBD.mSampleRate;
        DEBUG_TRACE("CoreAudioRenderer output hardware latency: %d (%0.2f s)", latencyFrames, m_OutputHardwareLatency);
    }

    {
        if (m_OutputSoftwareLatency == 0.0) {
            m_OutputSoftwareLatency = m_OutputSoftwareLatencyCurrent;
        }
        m_OutputSoftwareLatency = qMax(qMin(m_OutputSoftwareLatency, m_OutputSoftwareLatencyMax), m_OutputSoftwareLatencyMin);

        uint32_t bufferFrameSize = m_OutputSoftwareLatency * m_OutputASBD.mSampleRate;
        AudioObjectPropertyAddress addrSet{kAudioDevicePropertyBufferFrameSize, kAudioObjectPropertyScopeInput, kAudioObjectPropertyElementMain};
        status = AudioObjectSetPropertyData(m_OutputDeviceID, &addrSet, 0, NULL, sizeof(uint32_t), &bufferFrameSize);
        if (status != noErr) {
            CA_LogError(status, "Failed to set the output device buffer frame size");
            return false;
        }
        DEBUG_TRACE("CoreAudioRenderer output requested BufferFrameSize of %d (%0.3f s)", bufferFrameSize, m_OutputSoftwareLatency);

        // see what we got
        uint32_t size = sizeof(uint32_t);
        AudioObjectPropertyAddress addrGet{kAudioDevicePropertyBufferFrameSize, kAudioObjectPropertyScopeOutput, kAudioObjectPropertyElementMain};
        status = AudioObjectGetPropertyData(m_OutputDeviceID, &addrGet, 0, nil, &size, &bufferFrameSize);
        if (status != noErr) {
            CA_LogError(status, "Failed to get the output device buffer frame size");
            return false;
        }
        m_OutputSoftwareLatencyCurrent = (double)bufferFrameSize / m_OutputASBD.mSampleRate;
        m_OutputSoftwareLatency = m_OutputSoftwareLatencyCurrent;
        DEBUG_TRACE("CoreAudioRenderer output now has actual BufferFrameSize of %d (%0.3f s)", bufferFrameSize, m_OutputSoftwareLatency);
    }
#endif

    return true;
}

bool CoreAudioRenderer::initRingBuffer()
{
    // Always buffer at least 2 packets, up to 30ms worth of packets
    int packetsToBuffer = qMax(2, (int)ceil(kRingBufferMaxSeconds / m_OutputSoftwareLatency));

    bool ok = TPCircularBufferInit(&m_RingBuffer,
                                   sizeof(float) *
                                   m_opusConfig->channelCount *
                                   m_opusConfig->samplesPerFrame *
                                   packetsToBuffer);
    if (!ok) return false;

    // Spatial mixer code needs to be able to read from the ring buffer
    m_SpatialAU.setRingBufferPtr(&m_RingBuffer);

    // real length will be larger than requested due to memory page alignment
    m_BufferSize = m_RingBuffer.length;
    DEBUG_TRACE("CoreAudioRenderer ring buffer init, %d packets (%d bytes)", packetsToBuffer, m_BufferSize);

    return true;
}

OSStatus onDeviceOverload(AudioObjectID /*inObjectID*/,
                          UInt32 /*inNumberAddresses*/,
                          const AudioObjectPropertyAddress * /*inAddresses*/,
                          void *inClientData)
{
    CoreAudioRenderer *me = (CoreAudioRenderer *)inClientData;
    SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "CoreAudioRenderer output device overload");
    me->statsIncDeviceOverload();
    return noErr;
}

OSStatus onAudioNeedsReinit(AudioObjectID /*inObjectID*/,
                            UInt32 /*inNumberAddresses*/,
                            const AudioObjectPropertyAddress * /*inAddresses*/,
                            void *inClientData)
{
    CoreAudioRenderer *me = (CoreAudioRenderer *)inClientData;
    SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "CoreAudioRenderer output device had a change, will reinit");
    me->m_needsReinit = true;
    return noErr;
}

bool CoreAudioRenderer::initListeners()
{
    // events we care about on our output device

    AudioObjectPropertyAddress addr{kAudioDeviceProcessorOverload, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
    OSStatus status = AudioObjectAddPropertyListener(m_OutputDeviceID, &addr, onDeviceOverload, this);
    if (status != noErr) {
        CA_LogError(status, "Failed to add listener for kAudioDeviceProcessorOverload");
        return false;
    }

    addr.mSelector = kAudioDevicePropertyDeviceHasChanged;
    status = AudioObjectAddPropertyListener(m_OutputDeviceID, &addr, onAudioNeedsReinit, this);
    if (status != noErr) {
        CA_LogError(status, "Failed to add listener for kAudioDevicePropertyDeviceHasChanged");
        return false;
    }

    // non-device-specific listeners
    addr.mSelector = kAudioHardwarePropertyServiceRestarted;
    status = AudioObjectAddPropertyListener(kAudioObjectSystemObject, &addr, onAudioNeedsReinit, this);
    if (status != noErr) {
        CA_LogError(status, "Failed to add listener for kAudioHardwarePropertyServiceRestarted");
        return false;
    }

    addr.mSelector = kAudioHardwarePropertyDefaultOutputDevice;
    status = AudioObjectAddPropertyListener(kAudioObjectSystemObject, &addr, onAudioNeedsReinit, this);
    if (status != noErr) {
        CA_LogError(status, "Failed to add listener for kAudioDevicePropertyIOStoppedAbnormally");
        return false;
    }

    return true;
}

void CoreAudioRenderer::deinitListeners()
{
    AudioObjectPropertyAddress addr{kAudioDeviceProcessorOverload, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
    AudioObjectRemovePropertyListener(m_OutputDeviceID, &addr, onDeviceOverload, this);

    addr.mSelector = kAudioDevicePropertyDeviceHasChanged;
    AudioObjectRemovePropertyListener(m_OutputDeviceID, &addr, onAudioNeedsReinit, this);

    addr.mSelector = kAudioHardwarePropertyServiceRestarted;
    AudioObjectRemovePropertyListener(kAudioObjectSystemObject, &addr, onAudioNeedsReinit, this);

    addr.mSelector = kAudioHardwarePropertyDefaultOutputDevice;
    AudioObjectRemovePropertyListener(kAudioObjectSystemObject, &addr, onAudioNeedsReinit, this);
}

bool CoreAudioRenderer::setCallback(AURenderCallback callback)
{
    AURenderCallbackStruct callbackStruct;
    callbackStruct.inputProc = callback;
    callbackStruct.inputProcRefCon = this;

    OSStatus status = AudioUnitSetProperty(m_OutputAU, kAudioUnitProperty_SetRenderCallback, kAudioUnitScope_Output, 0, &callbackStruct, sizeof(callbackStruct));
    if (status != noErr) {
        CA_LogError(status, "Failed to set output render callback");
        return false;
    }

    return true;
}

void* CoreAudioRenderer::getAudioBuffer(int* size)
{
    // We must always write a full frame of audio. If we don't,
    // the reader will get out of sync with the writer and our
    // channels will get all mixed up. To ensure this is always
    // the case, round our bytes free down to the next multiple
    // of our frame size.
    uint32_t bytesFree;
    void *ptr = TPCircularBufferHead(&m_RingBuffer, &bytesFree);
    int bytesPerFrame = m_opusConfig->channelCount * sizeof(float);
    *size = qMin(*size, (int)(bytesFree / bytesPerFrame) * bytesPerFrame);

    m_BufferFilledBytes = m_RingBuffer.length - bytesFree;

    return ptr;
}

bool CoreAudioRenderer::submitAudio(int bytesWritten)
{
    // We'll be fully recreated after any changes to the audio device, default output, etc.
    if (m_needsReinit) {
        return false;
    }

    // flip our stats window, no great place to put this but this is similar to how video does it.
    // When no audio is playing or audio is muted, stats will pause their updates.
    if (SDL_TICKS_PASSED(SDL_GetTicks(), m_ActiveWndAudioStats.measurementStartTimestamp + 1000)) {
        // Accumulate these values into the global stats
        addAudioStats(m_ActiveWndAudioStats, m_GlobalAudioStats);

        // Move this window into the last window slot and clear it for next window
        SDL_memcpy(&m_LastWndAudioStats, &m_ActiveWndAudioStats, sizeof(m_ActiveWndAudioStats));
        SDL_zero(m_ActiveWndAudioStats);
        m_ActiveWndAudioStats.measurementStartTimestamp = SDL_GetTicks();
    }

    if (bytesWritten == 0) {
        // Nothing to do
        return true;
    }

    if (!m_ActiveWndAudioStats.totalPackets++) {
        // when totalPackets is 0, we're starting a new stream
        m_ActiveWndAudioStats.measurementStartTimestamp = SDL_GetTicks();
    }

    // drop packet if we've fallen behind Moonlight's queue by at least 30 ms
    if (LiGetPendingAudioDuration() > 30) {
        // stat handled by totalPackets - decodedPackets
        return true;
    }

    // Advance the write pointer
    TPCircularBufferProduce(&m_RingBuffer, bytesWritten);
    m_ActiveWndAudioStats.decodedPackets++;

    return true;
}

AUSpatialMixerOutputType CoreAudioRenderer::getSpatialMixerOutputType()
{
#if TARGET_OS_OSX
    // Check if headphones are plugged in.
    uint32_t dataSource{};
    uint32_t size = sizeof(dataSource);

    AudioObjectPropertyAddress addTransType{kAudioDevicePropertyTransportType, kAudioObjectPropertyScopeOutput, kAudioObjectPropertyElementMain};
    OSStatus status = AudioObjectGetPropertyData(m_OutputDeviceID, &addTransType, 0, nullptr, &size, &dataSource);
    if (status != noErr) {
        CA_LogError(status, "Failed to get the transport type of output device");
        return kSpatialMixerOutputType_ExternalSpeakers;
    }

    CA_FourCC(dataSource, m_OutputTransportType);
    DEBUG_TRACE("CoreAudioRenderer output transport type %s", m_OutputTransportType);

    if (dataSource == kAudioDeviceTransportTypeHDMI) {
        dataSource = kIOAudioOutputPortSubTypeExternalSpeaker;
    } else if (dataSource == kAudioDeviceTransportTypeBluetooth || dataSource == kAudioDeviceTransportTypeUSB) {
        dataSource = kIOAudioOutputPortSubTypeHeadphones;
    } else {
        AudioObjectPropertyAddress theAddress{kAudioDevicePropertyDataSource, kAudioDevicePropertyScopeOutput, kAudioObjectPropertyElementMain};

        status = AudioObjectGetPropertyData(m_OutputDeviceID, &theAddress, 0, nullptr, &size, &dataSource);
        if (status != noErr) {
            CA_LogError(status, "Couldn't determine default audio device type, defaulting to ExternalSpeakers");
            return kSpatialMixerOutputType_ExternalSpeakers;
        }
    }

    CA_FourCC(dataSource, m_OutputDataSource);
    DEBUG_TRACE("CoreAudioRenderer output data source %s", m_OutputDataSource);

    switch (dataSource) {
        case kIOAudioOutputPortSubTypeInternalSpeaker:
            return kSpatialMixerOutputType_BuiltInSpeakers;
            break;

        case kIOAudioOutputPortSubTypeHeadphones:
            return kSpatialMixerOutputType_Headphones;
            break;

        case kIOAudioOutputPortSubTypeExternalSpeaker:
            return kSpatialMixerOutputType_ExternalSpeakers;
            break;

        default:
            return kSpatialMixerOutputType_Headphones;
            break;
    }
#else
    AVAudioSession *audioSession = [AVAudioSession sharedInstance];

    if ([audioSession.currentRoute.outputs count] != 1) {
        return kSpatialMixerOutputType_ExternalSpeakers;
    } else {
        NSString* pType = audioSession.currentRoute.outputs.firstObject.portType;
        if ([pType isEqualToString:AVAudioSessionPortHeadphones] || [pType isEqualToString:AVAudioSessionPortBluetoothA2DP] || [pType isEqualToString:AVAudioSessionPortBluetoothLE] || [pType isEqualToString:AVAudioSessionPortBluetoothHFP]) {
            return kSpatialMixerOutputType_Headphones;
        } else if ([pType isEqualToString:AVAudioSessionPortBuiltInSpeaker]) {
            return kSpatialMixerOutputType_BuiltInSpeakers;
        } else {
            return kSpatialMixerOutputType_ExternalSpeakers;
        }
    }
#endif
}

static void replace_fancy_quote(char *str)
{
    char *pos;
    while ((pos = strstr(str, "\xe2\x80\x99")) != NULL) {
        *pos = '\'';
        memmove(pos + 1, pos + 3, strlen(pos + 3) + 1);
    }
}

void CoreAudioRenderer::setOutputDeviceName(const CFStringRef cfstr)
{
    if (cfstr) {
        CFIndex size = CFStringGetMaximumSizeForEncoding(CFStringGetLength(cfstr), kCFStringEncodingUTF8) + 1;
        char *buffer = (char *)malloc(size);
        CFStringGetCString(cfstr, buffer, size, kCFStringEncodingUTF8);

        // it's very likely we'll get a name like "Andy’s AirPods Pro"
        // with a UTF8 quote, and our overlay font is only ASCII
        replace_fancy_quote(buffer);

        if (m_OutputDeviceName) {
            free(m_OutputDeviceName);
        }

        m_OutputDeviceName = buffer;
    }
}
