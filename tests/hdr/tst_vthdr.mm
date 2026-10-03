// No NSApplication, window, drawable, input device or network connection.
// Compile and execute the production shader on offscreen Metal textures.
#define AVMediaType AVMediaType_FFmpeg
#include "streaming/video/ffmpeg-renderers/renderer.h"
#include "streaming/video/ffmpeg-renderers/vt_colorspace.h"
#undef AVMediaType
#import <Foundation/Foundation.h>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <vector>

static unsigned checks = 0, failures = 0;
static bool legacy = false;
static void check(bool okay, const char* what) {
    ++checks;
    if (!okay) { ++failures; std::fprintf(stderr, "FAIL: %s\n", what); }
}
static double pqEncode(double nits) {
    double p = std::pow(std::clamp(nits / 10000.0, 0.0, 1.0), 2610.0 / 16384.0);
    return std::pow((3424.0/4096.0 + 2413.0/128.0*p) / (1.0 + 2392.0/128.0*p), 2523.0/32.0);
}
static double pqDecode(double v) {
    double p = std::pow(std::clamp(v, 0.0, 1.0), 32.0/2523.0);
    return 10000 * std::pow(std::max(p - 3424.0/4096.0, 0.0) / (2413.0/128.0 - 2392.0/128.0*p), 16384.0/2610.0);
}
class CscProvider : public IFFmpegRenderer {
public:
    CscProvider() : IFFmpegRenderer(RendererType::VTMetal) {}
    bool initialize(PDECODER_PARAMETERS) override { return false; }
    bool prepareDecoderContext(AVCodecContext*, AVDictionary**) override { return false; }
    void renderFrame(AVFrame*) override {}
};
struct Vertex { simd_float4 position; simd_float2 texCoord; };
struct LegacyParams { simd_half3x3 matrix; simd_half3 offsets; simd_half2 chromaOffset; _Float16 scale; };

struct GPU {
    id<MTLDevice> device;
    id<MTLCommandQueue> queue;
    id<MTLLibrary> library;
    GPU(const char* shader) {
        device = MTLCreateSystemDefaultDevice();
        if (!device) throw std::runtime_error("Metal device unavailable");
        queue = [device newCommandQueue];
        NSError* error = nil;
        NSString* source = [NSString stringWithContentsOfFile:[NSString stringWithUTF8String:shader]
            encoding:NSUTF8StringEncoding error:&error];
        library = [device newLibraryWithSource:source options:nil error:&error];
        if (!library) throw std::runtime_error(error.localizedDescription.UTF8String);
        std::printf("GPU: %s, production shader: %s\n", device.name.UTF8String, shader);
    }
    ~GPU() { [library release]; [queue release]; [device release]; }
    id<MTLTexture> texture(MTLPixelFormat format, int width, int height) {
        auto desc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:format width:width height:height mipmapped:NO];
        desc.storageMode = MTLStorageModeShared;
        desc.usage = MTLTextureUsageShaderRead | MTLTextureUsageRenderTarget;
        return [device newTextureWithDescriptor:desc];
    }
    std::array<float,4> render(const char* shader, const VTMetalCscParams& csc,
        const std::vector<id<MTLTexture>>& textures, float white, MTLPixelFormat output,
        const VTMetalOverlayParams* overlay = nullptr, float headroom = 2) {
        auto desc = [[MTLRenderPipelineDescriptor new] autorelease];
        desc.vertexFunction = [[library newFunctionWithName:@"vs_draw"] autorelease];
        desc.fragmentFunction = [[library newFunctionWithName:[NSString stringWithUTF8String:shader]] autorelease];
        desc.colorAttachments[0].pixelFormat = output;
        NSError* error = nil;
        auto pipeline = [device newRenderPipelineStateWithDescriptor:desc error:&error];
        if (!pipeline) throw std::runtime_error(error.localizedDescription.UTF8String);
        auto target = texture(output, 2, 2);
        auto pass = [MTLRenderPassDescriptor renderPassDescriptor];
        pass.colorAttachments[0].texture = target;
        pass.colorAttachments[0].loadAction = MTLLoadActionClear;
        pass.colorAttachments[0].storeAction = MTLStoreActionStore;
        auto command = [queue commandBuffer];
        auto encoder = [command renderCommandEncoderWithDescriptor:pass];
        [encoder setRenderPipelineState:pipeline];
        Vertex vertices[] = {{{-1,-1,0,1},{0,0}},{{-1,1,0,1},{0,1}},{{1,-1,0,1},{1,0}},{{1,1,0,1},{1,1}}};
        [encoder setVertexBytes:vertices length:sizeof(vertices) atIndex:0];
        if (legacy) {
            LegacyParams old = {};
            for (int i=0;i<3;++i) for (int j=0;j<3;++j) old.matrix.columns[i][j] = csc.matrix.columns[i][j];
            old.offsets = simd_make_half3(csc.offsets.x,csc.offsets.y,csc.offsets.z);
            old.chromaOffset = simd_make_half2(csc.chromaOffset.x,csc.chromaOffset.y);
            old.scale = csc.bitnessScaleFactor;
            [encoder setFragmentBytes:&old length:sizeof(old) atIndex:0];
        }
        else [encoder setFragmentBytes:&csc length:sizeof(csc) atIndex:0];
        // Legacy shader compressed to headroom before OS tone mapping.
        float peak=1000;
        [encoder setFragmentBytes:&headroom length:sizeof(headroom) atIndex:1];
        [encoder setFragmentBytes:&white length:sizeof(white) atIndex:2];
        [encoder setFragmentBytes:&peak length:sizeof(peak) atIndex:3];
        if (overlay) [encoder setFragmentBytes:overlay length:sizeof(*overlay) atIndex:4];
        for (size_t i=0;i<textures.size();++i) [encoder setFragmentTexture:textures[i] atIndex:i];
        [encoder drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:4];
        [encoder endEncoding]; [command commit]; [command waitUntilCompleted];
        if (command.status != MTLCommandBufferStatusCompleted) throw std::runtime_error("GPU command failed");
        std::array<float,4> pixel = {};
        if (output == MTLPixelFormatRGBA16Float) {
            _Float16 data[16]; [target getBytes:data bytesPerRow:16 fromRegion:MTLRegionMake2D(0,0,2,2) mipmapLevel:0];
            for (int i=0;i<4;++i) pixel[i]=data[i];
        }
        else if (output == MTLPixelFormatBGR10A2Unorm) {
            uint32_t data[4]; [target getBytes:data bytesPerRow:8 fromRegion:MTLRegionMake2D(0,0,2,2) mipmapLevel:0];
            pixel = {float((data[0] >> 20) & 1023)/1023, float((data[0] >> 10) & 1023)/1023, float(data[0] & 1023)/1023, float(data[0] >> 30)/3};
        }
        else {
            uint8_t data[16]; [target getBytes:data bytesPerRow:8 fromRegion:MTLRegionMake2D(0,0,2,2) mipmapLevel:0];
            pixel={data[2]/255.0f,data[1]/255.0f,data[0]/255.0f,data[3]/255.0f};
        }
        [target release]; [pipeline release];
        return pixel;
    }
};

static void testGPU(GPU& gpu) {
    CscProvider provider;
    double worst=0;
    for (bool planar : {false,true}) for (bool full : {false,true})
        for (int storage : {0,1,2}) for (float white : {100.0f,203.0f}) {
        // Native PyroWave UNORM, VideoToolbox P010 (MSB), software planar (LSB).
        AVFrame frame = {}; frame.format=AV_PIX_FMT_YUV444P10; frame.colorspace=AVCOL_SPC_BT2020_NCL;
        frame.color_range=full ? AVCOL_RANGE_JPEG : AVCOL_RANGE_MPEG;
        std::array<float,9> matrix; std::array<float,3> offsets;
        provider.getFramePremultipliedCscConstants(&frame,matrix,offsets);
        VTMetalCscParams params = {};
        params.matrix=simd_matrix(simd_make_float3(matrix[0],matrix[3],matrix[6]),
            simd_make_float3(matrix[1],matrix[4],matrix[7]),simd_make_float3(matrix[2],matrix[5],matrix[8]));
        params.offsets=simd_make_float3(offsets[0],offsets[1],offsets[2]);
        params.bitnessScaleFactor=storage==0 ? 1 : vtUnormScale(10,16,storage==1 ? 6 : 0);
        if (legacy && storage) params.bitnessScaleFactor=storage==1 ? 1 : 64;
        auto y=gpu.texture(MTLPixelFormatR16Unorm,2,2);
        auto u=gpu.texture(planar ? MTLPixelFormatR16Unorm : MTLPixelFormatRG16Unorm,2,2);
        auto v=planar ? gpu.texture(MTLPixelFormatR16Unorm,2,2) : nil;
        auto unorm=[&](unsigned code) {return uint16_t(storage==0 ? std::lround(code*65535.0/1023) : code << (storage==1 ? 6 : 0));};
        uint16_t chroma=unorm(512), neutral[8]; std::fill_n(neutral,8,chroma);
        [u replaceRegion:MTLRegionMake2D(0,0,2,2) mipmapLevel:0 withBytes:neutral bytesPerRow:planar ? 4 : 8];
        if (v) [v replaceRegion:MTLRegionMake2D(0,0,2,2) mipmapLevel:0 withBytes:neutral bytesPerRow:4];
        std::vector<id<MTLTexture>> textures={y,u}; if(v) textures.push_back(v);
        for (double nits : {0.0,0.1,1.0,10.0,80.0,100.0,203.0,400.0,1000.0,4000.0,10000.0}) {
            unsigned code=std::lround(full ? pqEncode(nits)*1023 : 64+pqEncode(nits)*876);
            uint16_t data[4]; std::fill_n(data,4,unorm(code));
            [y replaceRegion:MTLRegionMake2D(0,0,2,2) mipmapLevel:0 withBytes:data bytesPerRow:4];
            double expected=pqDecode(full ? code/1023.0 : (int(code)-64)/876.0);
            auto result=gpu.render(planar ? "ps_draw_linear_triplanar" : "ps_draw_linear",params,textures,white,MTLPixelFormatRGBA16Float);
            double error=0; for(int i=0;i<3;++i) error=std::max(error,std::abs(result[i]*white-expected));
            worst=std::max(worst,error/std::max(expected,0.01));
            check(error <= std::max(0.004,expected*0.0015), "PQ absolute luminance, black and reference white");
            // Pixel values handed to Core Animation must not vary with display headroom.
            auto sdr=gpu.render(planar ? "ps_draw_linear_triplanar" : "ps_draw_linear",params,textures,white,MTLPixelFormatRGBA16Float,nullptr,1);
            check(result==sdr,"system tone mapping occurs once, independent of shader headroom");
            auto pq=gpu.render(planar ? "ps_draw_triplanar" : "ps_draw_biplanar",params,textures,white,MTLPixelFormatBGR10A2Unorm);
            double encoded=full ? code/1023.0 : (int(code)-64)/876.0;
            for(int i=0;i<3;++i) check(std::abs(pq[i]-encoded)<=1.01/1023,"PQ passthrough preserves 10-bit signal");
        }
        [y release]; [u release]; [v release];
    }
    std::printf("PQ maximum relative luminance error: %.4f%%\n",100*worst);
    auto texture=gpu.texture(MTLPixelFormatRGBA8Unorm,2,2);
    const uint8_t whitePixels[16]={255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255};
    [texture replaceRegion:MTLRegionMake2D(0,0,2,2) mipmapLevel:0 withBytes:whitePixels bytesPerRow:8];
    AVFrame frame={};frame.color_trc=AVCOL_TRC_SMPTE2084;frame.color_primaries=AVCOL_PRI_BT2020;
    for(bool linear : {false,true}) {
        auto params=vtMetalOverlayParams(&frame,COLORSPACE_REC_2020,linear,203);
        auto result=gpu.render("ps_draw_rgb",{}, {texture},203,linear ? MTLPixelFormatRGBA16Float : MTLPixelFormatBGR10A2Unorm,&params);
        for(int i=0;i<3;++i) check(std::abs(result[i]-(linear ? 1 : pqEncode(203))) < 1.1/1023,"SDR overlay white stays at reference white in HDR");
    }
    // Color and alpha, not just white: verify the Rec.709-to-2020 matrix and
    // sRGB decoding before HDR encoding. Test HLG diffuse-white placement too.
    for(std::array<uint8_t,4> color : {std::array<uint8_t,4>{255,0,0,128},{0,255,0,255},
        {0,0,255,255},{128,128,128,255},{0,0,0,255},{255,255,255,255}}) {
        uint8_t pixels[16];for(int i=0;i<4;++i)std::copy(color.begin(),color.end(),pixels+4*i);
        [texture replaceRegion:MTLRegionMake2D(0,0,2,2) mipmapLevel:0 withBytes:pixels bytesPerRow:8];
        std::array<double,3> linear709;
        for(int i=0;i<3;++i){double v=color[i]/255.0;linear709[i]=v<=.04045 ? v/12.92 : std::pow((v+.055)/1.055,2.4);}
        std::array<double,3> linear2020={.627404*linear709[0]+.329282*linear709[1]+.0433136*linear709[2],
            .069097*linear709[0]+.919540*linear709[1]+.0113612*linear709[2],
            .0163916*linear709[0]+.0880132*linear709[1]+.895595*linear709[2]};
        for(int transfer : {1,2,3}) {
            frame.color_trc=transfer==3 ? AVCOL_TRC_ARIB_STD_B67 : AVCOL_TRC_SMPTE2084;
            auto params=vtMetalOverlayParams(&frame,COLORSPACE_REC_2020,transfer==2,203);
            auto result=gpu.render("ps_draw_rgb",{}, {texture},203,transfer==2 ? MTLPixelFormatRGBA16Float : MTLPixelFormatBGR10A2Unorm,&params);
            for(int i=0;i<3;++i) {
                double scene=linear2020[i]*.26496256;
                double hlg=scene<=1.0/12.0 ? std::sqrt(3*scene) : .17883277*std::log(12*scene-.28466892)+.55991073;
                double expected=transfer==1 ? pqEncode(linear2020[i]*203) : transfer==2 ? linear2020[i] : hlg;
                check(std::abs(result[i]-expected)<1.1/1023,"SDR overlay colors retain gamut and transfer in HDR");
            }
            double alphaExpected=transfer==2 ? color[3]/255.0 : std::round(color[3]*3.0/255)/3.0;
            check(std::abs(result[3]-alphaExpected)<.001,"HDR overlay preserves alpha");
        }
    }
    [texture release];
}

static void testMetadata() {
    AVFrame* frame=av_frame_alloc(); frame->color_trc=AVCOL_TRC_SMPTE2084;
    SS_HDR_METADATA host={}; host.maxDisplayLuminance=1015;host.minDisplayLuminance=5;
    host.displayPrimaries[0]={34000,16000};host.displayPrimaries[1]={13250,34500};host.displayPrimaries[2]={7500,3000};host.whitePoint={15635,16450};
    host.maxContentLightLevel=2000;host.maxFrameAverageLightLevel=0;
    auto result=vtHdrMetadataForFrame(frame,&host);
    check(result.hasDisplay && result.hasContent && result.maxNits==1015,"host fallback and partial content-light metadata");
    check(result.display[0]==uint8_t(13250>>8) && result.display[1]==uint8_t(13250),"MDCV big-endian GBR primary order");
    auto md=av_mastering_display_metadata_create_side_data(frame);
    md->has_luminance=1;md->min_luminance=av_make_q(1,1000);md->max_luminance=av_make_q(4000,1);
    auto content=av_content_light_metadata_create_side_data(frame);content->MaxCLL=6000;
    result=vtHdrMetadataForFrame(frame,&host);
    check(result.maxNits==4000 && !result.hasDisplay && result.content[0]==uint8_t(6000>>8),"per-frame metadata wins over asynchronous host metadata");
    md->max_luminance={0,0};
    check(vtHdrMetadataForFrame(frame,&host).maxNits==1000,"invalid metadata produces a finite fallback");
    frame->color_trc=AVCOL_TRC_BT709;
    result=vtHdrMetadataForFrame(frame,&host);
    check(!result.hasDisplay && !result.hasContent,"SDR clears HDR metadata regardless of host state");
    CVPixelBufferRef buffer=nullptr;CVPixelBufferCreate(nullptr,2,2,kCVPixelFormatType_32BGRA,nullptr,&buffer);
    uint8_t displayData[24]={}, contentData[4]={};
    auto display=CFDataCreate(nullptr,displayData,24), light=CFDataCreate(nullptr,contentData,4);
    vtAttachHdrMetadata(buffer,display,light);
    check(CVBufferHasAttachment(buffer,kCVImageBufferMasteringDisplayColorVolumeKey),"attach HDR metadata");
    vtAttachHdrMetadata(buffer,nullptr,nullptr);
    check(!CVBufferHasAttachment(buffer,kCVImageBufferMasteringDisplayColorVolumeKey) &&
          !CVBufferHasAttachment(buffer,kCVImageBufferContentLightLevelInfoKey),"recycled SDR buffers lose stale HDR attachments");
    CFRelease(display);CFRelease(light);CFRelease(buffer);av_frame_free(&frame);
}

static void testColorPatches(GPU& gpu) {
    CscProvider provider;
    double maxSignalError = 0;
    for (AVColorSpace space : {AVCOL_SPC_SMPTE170M,AVCOL_SPC_BT709,AVCOL_SPC_BT2020_NCL})
        for (int bits : {8,10}) for (bool full : {false,true}) for (bool planar : {false,true}) {
        double kr = space==AVCOL_SPC_SMPTE170M ? .299 : space==AVCOL_SPC_BT709 ? .2126 : .2627;
        double kb = space==AVCOL_SPC_SMPTE170M ? .114 : space==AVCOL_SPC_BT709 ? .0722 : .0593;
        double kg = 1-kr-kb;
        int max = (1<<bits)-1, factor=1<<(bits-8), mid=1<<(bits-1);
        double yMin=full ? 0 : 16*factor, yScale=full ? max : 219*factor, uvScale=full ? max : 224*factor;
        AVFrame frame={};frame.format=bits==8 ? AV_PIX_FMT_YUV444P : AV_PIX_FMT_YUV444P10;
        frame.colorspace=space;frame.color_range=full ? AVCOL_RANGE_JPEG : AVCOL_RANGE_MPEG;
        std::array<float,9> matrix;std::array<float,3> offsets;
        provider.getFramePremultipliedCscConstants(&frame,matrix,offsets);
        VTMetalCscParams params={};
        params.matrix=simd_matrix(simd_make_float3(matrix[0],matrix[3],matrix[6]),
            simd_make_float3(matrix[1],matrix[4],matrix[7]),simd_make_float3(matrix[2],matrix[5],matrix[8]));
        params.offsets=simd_make_float3(offsets[0],offsets[1],offsets[2]);params.bitnessScaleFactor=1;
        auto format=bits==8 ? MTLPixelFormatR8Unorm : MTLPixelFormatR16Unorm;
        auto y=gpu.texture(format,2,2),u=gpu.texture(planar ? format : bits==8 ? MTLPixelFormatRG8Unorm : MTLPixelFormatRG16Unorm,2,2);
        auto v=planar ? gpu.texture(format,2,2) : nil;
        auto upload=[&](id<MTLTexture> texture,unsigned code,unsigned second,bool interleaved) {
            unsigned codes[2]={code,second};
            uint8_t bytes[8];uint16_t words[8];
            for(int i=0;i<8;++i) {bytes[i]=codes[interleaved ? i%2 : 0];words[i]=std::lround(codes[interleaved ? i%2 : 0]*65535.0/max);}
            [texture replaceRegion:MTLRegionMake2D(0,0,2,2) mipmapLevel:0 withBytes:bits==8 ? (void*)bytes : (void*)words
                bytesPerRow:2*(bits==8 ? 1 : 2)*(interleaved ? 2 : 1)];
        };
        for(std::array<double,3> rgb : {std::array<double,3>{0,0,0},{1,1,1},{.18,.18,.18},
            {.8,.15,.1},{.15,.8,.1},{.1,.15,.8},{.75,.6,.2},{.3,.7,.8}}) {
            double luma=kr*rgb[0]+kg*rgb[1]+kb*rgb[2];
            unsigned yc=std::lround(yMin+yScale*luma),uc=std::lround(mid+uvScale*(rgb[2]-luma)/(2*(1-kb))),vc=std::lround(mid+uvScale*(rgb[0]-luma)/(2*(1-kr)));
            upload(y,yc,0,false);upload(u,uc,vc,!planar);if(v) upload(v,vc,0,false);
            std::vector<id<MTLTexture>> textures={y,u};if(v)textures.push_back(v);
            // Independent inverse derived from the standard's Kr/Kb coefficients.
            double yq=(yc-yMin)/yScale,cb=(int(uc)-mid)/uvScale,cr=(int(vc)-mid)/uvScale;
            std::array<double,3> expected={yq+2*(1-kr)*cr,yq-2*kb*(1-kb)/kg*cb-2*kr*(1-kr)/kg*cr,yq+2*(1-kb)*cb};
            auto signal=gpu.render(planar ? "ps_draw_triplanar" : "ps_draw_biplanar",params,textures,203,
                bits==8 ? MTLPixelFormatBGRA8Unorm : MTLPixelFormatBGR10A2Unorm);
            for(int i=0;i<3;++i) {
                double error=std::abs(signal[i]-std::clamp(expected[i],0.0,1.0));
                maxSignalError=std::max(maxSignalError,error);
                check(error <= 1.01/max,"SDR/HDR color patches match independent BT.601/709/2020 conversion");
            }
            if(bits==10 && space==AVCOL_SPC_BT2020_NCL) {
                auto linear=gpu.render(planar ? "ps_draw_linear_triplanar" : "ps_draw_linear",params,textures,203,MTLPixelFormatRGBA16Float);
                for(int i=0;i<3;++i) {
                    double nits=pqDecode(expected[i]);
                    check(std::abs(linear[i]*203-nits) <= std::max(.005,nits*.0045),"colored PQ values preserve luminance and gamut");
                }
            }
        }
        [y release];[u release];[v release];
    }
    std::printf("Color-patch maximum signal error: %.6f\n",maxSignalError);
}

static void testCoreVideoP010(GPU& gpu) {
    CVMetalTextureCacheRef cache=nullptr;
    if(CVMetalTextureCacheCreate(nullptr,nullptr,gpu.device,nullptr,&cache)!=kCVReturnSuccess)
        throw std::runtime_error("CoreVideo Metal texture cache unavailable");
    CscProvider provider;
    for(bool full : {false,true}) {
        const void* keys[]={kCVPixelBufferMetalCompatibilityKey};const void* values[]={kCFBooleanTrue};
        auto attributes=CFDictionaryCreate(nullptr,keys,values,1,&kCFTypeDictionaryKeyCallBacks,&kCFTypeDictionaryValueCallBacks);
        CVPixelBufferRef buffer=nullptr;
        auto status=CVPixelBufferCreate(nullptr,4,4,full ? kCVPixelFormatType_420YpCbCr10BiPlanarFullRange :
            kCVPixelFormatType_420YpCbCr10BiPlanarVideoRange,attributes,&buffer);
        CFRelease(attributes);
        if(status!=kCVReturnSuccess) throw std::runtime_error("P010 pixel buffer unavailable");
        CVMetalTextureRef planes[2]={};
        for(int i=0;i<2;++i) {
            status=CVMetalTextureCacheCreateTextureFromImage(nullptr,cache,buffer,nullptr,
                i ? MTLPixelFormatRG16Unorm : MTLPixelFormatR16Unorm,
                CVPixelBufferGetWidthOfPlane(buffer,i),CVPixelBufferGetHeightOfPlane(buffer,i),i,&planes[i]);
            if(status!=kCVReturnSuccess) throw std::runtime_error("P010 texture mapping failed");
        }
        AVFrame frame={};frame.format=AV_PIX_FMT_P010;frame.colorspace=AVCOL_SPC_BT2020_NCL;
        frame.color_range=full ? AVCOL_RANGE_JPEG : AVCOL_RANGE_MPEG;
        std::array<float,9> matrix;std::array<float,3> offsets;
        provider.getFramePremultipliedCscConstants(&frame,matrix,offsets);
        VTMetalCscParams params={};
        params.matrix=simd_matrix(simd_make_float3(matrix[0],matrix[3],matrix[6]),
            simd_make_float3(matrix[1],matrix[4],matrix[7]),simd_make_float3(matrix[2],matrix[5],matrix[8]));
        params.offsets=simd_make_float3(offsets[0],offsets[1],offsets[2]);
        params.bitnessScaleFactor=legacy ? 1 : vtUnormScale(10,16,6);
        for(double nits : {0.0,203.0,1000.0,10000.0}) {
            unsigned code=std::lround(full ? pqEncode(nits)*1023 : 64+pqEncode(nits)*876);
            if(CVPixelBufferLockBaseAddress(buffer,0)!=kCVReturnSuccess) throw std::runtime_error("P010 buffer lock failed");
            for(int i=0;i<2;++i) for(size_t row=0;row<CVPixelBufferGetHeightOfPlane(buffer,i);++row) {
                auto line=reinterpret_cast<uint16_t*>(static_cast<uint8_t*>(CVPixelBufferGetBaseAddressOfPlane(buffer,i))+
                    row*CVPixelBufferGetBytesPerRowOfPlane(buffer,i));
                std::fill_n(line,CVPixelBufferGetWidthOfPlane(buffer,i)*(i ? 2 : 1),uint16_t((i ? 512 : code)<<6));
            }
            CVPixelBufferUnlockBaseAddress(buffer,0);
            auto result=gpu.render("ps_draw_linear",params,
                {CVMetalTextureGetTexture(planes[0]),CVMetalTextureGetTexture(planes[1])},203,MTLPixelFormatRGBA16Float);
            double expected=pqDecode(full ? code/1023.0 : (int(code)-64)/876.0);
            for(int i=0;i<3;++i) check(std::abs(result[i]*203-expected)<=std::max(.004,expected*.0015),
                "actual CoreVideo P010 mapping preserves PQ black, diffuse white and highlights");
        }
        CFRelease(planes[0]);CFRelease(planes[1]);CFRelease(buffer);
    }
    CFRelease(cache);
}
static void testTransitions() {
    CscProvider provider;AVFrame frame={};frame.width=2;frame.height=2;frame.format=AV_PIX_FMT_YUV444P10;
    frame.colorspace=AVCOL_SPC_BT2020_NCL;frame.color_primaries=AVCOL_PRI_BT2020;frame.color_trc=AVCOL_TRC_BT2020_10;
    check(provider.hasFrameFormatChanged(&frame),"first frame invalidates colorspace");
    check(vtColorSpaceName(&frame,COLORSPACE_REC_2020)==kCGColorSpaceITUR_2020,"BT.2020 SDR color tag");
    check(vtMetalPixelFormat(&frame,10,false)==MTLPixelFormatBGR10A2Unorm,"10-bit SDR retains precision");
    frame.color_trc=AVCOL_TRC_SMPTE2084;
    check(provider.hasFrameFormatChanged(&frame),"transfer-only SDR to HDR invalidates color tag");
    check(vtColorSpaceName(&frame,COLORSPACE_REC_2020)==kCGColorSpaceITUR_2100_PQ,"PQ color tag");
    check(vtMetalPixelFormat(&frame,10,true)==MTLPixelFormatRGBA16Float,"linear HDR render target");
    frame.color_primaries=AVCOL_PRI_BT709;
    check(vtColorSpaceName(&frame,COLORSPACE_REC_2020,true)==kCGColorSpaceExtendedLinearSRGB,"RGB gamut independent of YUV matrix");
    frame.color_trc=AVCOL_TRC_ARIB_STD_B67;frame.color_primaries=AVCOL_PRI_BT2020;
    check(vtColorSpaceName(&frame,COLORSPACE_REC_2020)==kCGColorSpaceITUR_2100_HLG,"HLG is not mislabeled SDR");
    frame.color_trc=AVCOL_TRC_BT709;frame.color_primaries=AVCOL_PRI_BT709;
    check(provider.hasFrameFormatChanged(&frame),"HDR to SDR invalidates format");
    frame.color_trc=AVCOL_TRC_IEC61966_2_1;
    check(vtColorSpaceName(&frame,COLORSPACE_REC_709)==kCGColorSpaceSRGB,"sRGB transfer independent of YUV matrix");
    check(std::abs(1023*64/65535.0*vtUnormScale(10,16,6)-1)<1e-6,"P010 normalization preserves full white");
}
int main(int argc,char** argv) { @autoreleasepool {
    try {
        if(argc<2) throw std::runtime_error("usage: tst_vthdr vt_renderer.metal [--legacy]");
        legacy=argc>2 && !std::strcmp(argv[2],"--legacy");
        testTransitions();testMetadata();GPU gpu(argv[1]);testGPU(gpu);testColorPatches(gpu);testCoreVideoP010(gpu);
        std::printf("%u checks, %u failures\n",checks,failures);
        return failures ? 1 : 0;
    }
    catch(const std::exception& e) {std::fprintf(stderr,"ERROR: %s\n",e.what());return 2;}
}}
