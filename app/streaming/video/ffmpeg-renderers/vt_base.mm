// Nasty hack to avoid conflict between AVFoundation and
// libavutil both defining AVMediaType
#define AVMediaType AVMediaType_FFmpeg
#include "vt.h"
#undef AVMediaType

#include "imgui/devui.h"

#import <Cocoa/Cocoa.h>
#import <VideoToolbox/VideoToolbox.h>
#import <AVFoundation/AVFoundation.h>
#import <Metal/Metal.h>

#include <mach/machine.h>
#include <sys/sysctl.h>

VTBaseRenderer::VTBaseRenderer(IFFmpegRenderer::RendererType type) :
    IFFmpegRenderer(type),
    m_HdrMetadataChanged(false),
    m_MasteringDisplayColorVolume(nullptr),
    m_ContentLightLevelInfo(nullptr),
    m_MinNits(0.0f),
    m_MaxNits(0.0f),
    m_OverrideNits(false) {

}

VTBaseRenderer::~VTBaseRenderer() {
    if (m_MasteringDisplayColorVolume != nullptr) {
        CFRelease(m_MasteringDisplayColorVolume);
    }

    if (m_ContentLightLevelInfo != nullptr) {
        CFRelease(m_ContentLightLevelInfo);
    }
}

bool VTBaseRenderer::isAppleSilicon() {
    static uint32_t cpuType = 0;
    if (cpuType == 0) {
        size_t size = sizeof(cpuType);
        int err = sysctlbyname("hw.cputype", &cpuType, &size, NULL, 0);
        if (err != 0) {
            SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION,
                        "sysctlbyname(hw.cputype) failed: %d", err);
            return false;
        }
    }

    // Apple Silicon Macs have CPU_ARCH_ABI64 set, so we need to mask that off.
    // For some reason, 64-bit Intel Macs don't seem to have CPU_ARCH_ABI64 set.
    return (cpuType & ~CPU_ARCH_MASK) == CPU_TYPE_ARM;
}

bool VTBaseRenderer::checkDecoderCapabilities(id<MTLDevice> device, PDECODER_PARAMETERS params) {
    if (params->videoFormat & VIDEO_FORMAT_MASK_H264) {
        if (!VTIsHardwareDecodeSupported(kCMVideoCodecType_H264)) {
            SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION,
                        "No HW accelerated H.264 decode via VT");
            return false;
        }
    }
    else if (params->videoFormat & VIDEO_FORMAT_MASK_H265) {
        if (!VTIsHardwareDecodeSupported(kCMVideoCodecType_HEVC)) {
            SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION,
                        "No HW accelerated HEVC decode via VT");
            return false;
        }

        // HEVC Main10 requires more extensive checks because there's no
        // simple API to check for Main10 hardware decoding, and if we don't
        // have it, we'll silently get software decoding with horrible performance.
        if (params->videoFormat == VIDEO_FORMAT_H265_MAIN10) {
            // Exclude all GPUs earlier than macOSGPUFamily2
            // https://developer.apple.com/documentation/metal/mtlfeatureset/mtlfeatureset_macos_gpufamily2_v1
            if ([device supportsFamily:MTLGPUFamilyMac2]) {
                if ([device.name containsString:@"Intel"]) {
                    // 500-series Intel GPUs are Skylake and don't support Main10 hardware decoding
                    if ([device.name containsString:@" 5"]) {
                        SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION,
                                    "No HEVC Main10 support on Skylake iGPU");
                        return false;
                    }
                }
                else if ([device.name containsString:@"AMD"]) {
                    // FirePro D, M200, and M300 series GPUs don't support Main10 hardware decoding
                    if ([device.name containsString:@"FirePro D"] ||
                            [device.name containsString:@" M2"] ||
                            [device.name containsString:@" M3"]) {
                        SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION,
                                    "No HEVC Main10 support on AMD GPUs until Polaris");
                        return false;
                    }
                }
            }
            else {
                SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION,
                            "No HEVC Main10 support on macOS GPUFamily1 GPUs");
                return false;
            }
        }
    }
    else if (params->videoFormat & VIDEO_FORMAT_MASK_AV1) {
    #if __MAC_OS_X_VERSION_MAX_ALLOWED >= 130000
        if (!VTIsHardwareDecodeSupported(kCMVideoCodecType_AV1)) {
            SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION,
                        "No HW accelerated AV1 decode via VT");
            return false;
        }

        // 10-bit is part of the Main profile for AV1, so it will always
        // be present on hardware that supports 8-bit.
    #else
        SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION,
                    "AV1 requires building with Xcode 14 or later");
        return false;
    #endif
    }

    return true;
}

void VTBaseRenderer::setHdrMode(bool enabled) {
    SS_HDR_METADATA metadata = {};
    bool available = enabled && LiGetHdrMetadata(&metadata);
    std::lock_guard<std::mutex> guard(m_HostHdrMetadataLock);
    m_HostHdrMetadata = metadata;
    m_HostHdrMetadataValid = available;
}

void VTBaseRenderer::updateHdrMetadataForFrame(const AVFrame* frame) {
    SS_HDR_METADATA host = {};
    bool available;
    {
        std::lock_guard<std::mutex> guard(m_HostHdrMetadataLock);
        host = m_HostHdrMetadata;
        available = m_HostHdrMetadataValid;
    }
    auto metadata = vtHdrMetadataForFrame(frame, available ? &host : nullptr);
    if (m_FrameHdrMetadataInitialized && metadata == m_FrameHdrMetadata) return;
    m_FrameHdrMetadataInitialized = true;
    m_FrameHdrMetadata = metadata;

    if (m_MasteringDisplayColorVolume) CFRelease(m_MasteringDisplayColorVolume);
    if (m_ContentLightLevelInfo) CFRelease(m_ContentLightLevelInfo);
    m_MasteringDisplayColorVolume = metadata.hasDisplay ?
        CFDataCreate(nullptr, metadata.display.data(), metadata.display.size()) : nullptr;
    m_ContentLightLevelInfo = metadata.hasContent ?
        CFDataCreate(nullptr, metadata.content.data(), metadata.content.size()) : nullptr;
    if (!m_OverrideNits) {
        m_MinNits = metadata.minNits;
        m_MaxNits = metadata.maxNits;
        DevUISettings::instance().SetConfig([=](DevUIConfig& config) {
            config.minNits = m_MinNits;
            config.maxNits = m_MaxNits;
        });
    }
    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION,
                "Frame HDR metadata: min/max nits %.4f/%.2f, mastering %s, content light %s",
                metadata.minNits, metadata.maxNits,
                metadata.hasDisplay ? "present" : "absent", metadata.hasContent ? "present" : "absent");
    m_HdrMetadataChanged = true;
}
