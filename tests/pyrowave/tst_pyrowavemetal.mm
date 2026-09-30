// Exercises the production Metal decoder, frame ownership and render shaders
// on the GPU. A missing Apple Silicon GPU fails rather than skipping coverage.
#include "../../app/streaming/video/pyrowave/pyrowavedecoder.h"
#include "../../app/streaming/video/pyrowave/pyrowavemetal.h"
#include <pyrowave_metal.h>
#include <QCoreApplication>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <thread>
#include <vector>

void require(bool condition, const char* what) {
    if (!condition) throw std::runtime_error(what);
}
void okay(pyrowave_result result, const char* what) {
    if (result != PYROWAVE_SUCCESS) throw std::runtime_error(std::string(what) + ": " + pyrowave_result_to_string(result));
}
void appendU32(std::vector<uint8_t>& out, uint32_t value) {
    for (int i = 0; i < 4; ++i) out.push_back(uint8_t(value >> (8 * i)));
}
std::vector<uint8_t> encode(pyrowave_device device, int width, int height, bool c444, bool lengthPrefix,
                           std::vector<uint8_t> (&planes)[3]) {
    pyrowave_encoder encoder = nullptr;
    pyrowave_encoder_create_info info = {device, width, height,
        c444 ? PYROWAVE_CHROMA_SUBSAMPLING_444 : PYROWAVE_CHROMA_SUBSAMPLING_420};
    okay(pyrowave_encoder_create(&info, &encoder), "create encoder");
    pyrowave_cpu_buffer input = {};
    input.width = width; input.height = height;
    input.format = c444 ? PYROWAVE_CPU_BUFFER_FORMAT_YUV444P : PYROWAVE_CPU_BUFFER_FORMAT_YUV420P;
    for (int i = 0; i < 3; ++i) {
        int w = i && !c444 ? width / 2 : width, h = i && !c444 ? height / 2 : height;
        planes[i].resize(w * h);
        for (int row = 0; row < h; ++row) for (int col = 0; col < w; ++col) {
            planes[i][row * w + col] = i ? 96 + (i == 1 ? col : row * 2) % 64 :
                std::clamp((col * 3 + row * 2 + 17) & 255, 16, 235);
        }
        input.data[i] = planes[i].data(); input.row_stride_in_bytes[i] = w;
        input.plane_size_in_bytes[i] = planes[i].size();
    }
    pyrowave_rate_control rate = {size_t(width * height * 4)};
    okay(pyrowave_encoder_encode_cpu_synchronous(encoder, &input, &rate), "encode");
    size_t count = 0;
    okay(pyrowave_encoder_compute_num_packets(encoder, 1376, &count), "count packets");
    std::vector<pyrowave_packet> packets(count);
    std::vector<uint8_t> data(rate.maximum_bitstream_size + 1024 * 1024);
    okay(pyrowave_encoder_packetize(encoder, packets.data(), 1376, &count, data.data(), data.size()), "packetize");
    std::vector<uint8_t> framed;
    if (lengthPrefix) appendU32(framed, uint32_t(count));
    for (size_t i = 0; i < count; ++i) {
        if (lengthPrefix) appendU32(framed, uint32_t(packets[i].size));
        framed.insert(framed.end(), data.begin() + packets[i].offset, data.begin() + packets[i].offset + packets[i].size);
    }
    pyrowave_encoder_destroy(encoder);
    return framed;
}
void verifyPlanes(AVFrame* frame, const std::vector<uint8_t> (&source)[3], bool tenBit) {
    auto ref = pyroWaveMetalFrame(frame);
    require(ref && pyroWaveMetalWait(ref), "decode GPU completion");
    auto queue = [ref->textures[0].device newCommandQueue];
    for (int i = 0; i < 3; ++i) {
        auto texture = ref->textures[i];
        size_t pixelBytes = tenBit ? 2 : 1, stride = (texture.width * pixelBytes + 255) & ~size_t(255);
        auto buffer = [texture.device newBufferWithLength:stride * texture.height options:MTLResourceStorageModeShared];
        auto command = [queue commandBuffer];
        auto blit = [command blitCommandEncoder];
        [blit copyFromTexture:texture sourceSlice:0 sourceLevel:0 sourceOrigin:MTLOriginMake(0,0,0)
            sourceSize:MTLSizeMake(texture.width, texture.height, 1) toBuffer:buffer destinationOffset:0
            destinationBytesPerRow:stride destinationBytesPerImage:stride * texture.height];
        [blit endEncoding]; [command commit]; [command waitUntilCompleted];
        require(command.status == MTLCommandBufferStatusCompleted, "texture download");
        double squareError = 0;
        for (size_t row = 0; row < texture.height; ++row) for (size_t col = 0; col < texture.width; ++col) {
            auto line = static_cast<uint8_t*>(buffer.contents) + stride * row;
            double value = tenBit ? reinterpret_cast<uint16_t*>(line)[col] * 255.0 / 65535.0 : line[col];
            double delta = value - source[i][row * texture.width + col]; squareError += delta * delta;
        }
        double psnr = squareError == 0 ? 99 : 10 * log10(255.0 * 255 * source[i].size() / squareError);
        std::printf("  plane %d: %.2f dB\n", i, psnr);
        require(psnr > 35, "decoded pixels differ from encoded source");
        [buffer release];
    }
    [queue release];
}
int main(int argc, char** argv) { @autoreleasepool {
    QCoreApplication app(argc, argv);
    try {
        require(pyroWaveMetalSupported(), "Apple Silicon Metal device unavailable");
        PyroWaveMetalCalibrationRenderer renderer;
        require(renderer.create(640, 360), "create native renderer");
        pyrowave_device device = nullptr;
        pyrowave_device_create_info di = {}; di.mtl_device = renderer.device();
        okay(pyrowave_device_create(&di, &device), "create encoder device");
        for (bool c444 : {false, true}) for (bool tenBit : {false, true}) for (bool lp : {false, true}) {
            std::printf("Testing %s %d-bit %s framing\n", c444 ? "4:4:4" : "4:2:0", tenBit ? 10 : 8, lp ? "length" : "record");
            const int width = 258, height = 130; // Non block-aligned dimensions
            std::vector<uint8_t> planes[3];
            auto data = encode(device, width, height, c444, lp, planes);
            PyroWaveMetalDecoder decoder;
            PyroWaveMetalDecoder::Config config; config.width = width; config.height = height;
            config.chroma444 = c444; config.tenBit = tenBit; config.metalDevice = renderer.device();
            require(decoder.initialize(config), decoder.lastError().c_str());
            require(renderer.prepare(width, height, c444, tenBit), "prepare Metal render target");
            AVFrame* frame = av_frame_alloc();
            require(decoder.decode(data.data(), data.size(), {}, 0, frame), decoder.lastError().c_str());
            verifyPlanes(frame, planes, tenBit);
            require(renderer.present(frame, tenBit), "native render of decoded textures");
            AVFrame* copy = av_frame_clone(frame); require(copy, "clone GPU frame");
            av_frame_free(&frame);
            require(renderer.present(copy, tenBit), "render cloned GPU frame");
            av_frame_free(&copy);
            // Hold every surface, then release a frame on another thread and reuse it.
            std::vector<AVFrame*> held;
            for (int i = 0; i < 8; ++i) {
                frame = av_frame_alloc();
                require(decoder.decode(data.data(), data.size(), {}, 0, frame), decoder.lastError().c_str());
                held.push_back(frame);
            }
            frame = av_frame_alloc();
            require(!decoder.decode(data.data(), data.size(), {}, 0, frame), "surface pool must apply bounded backpressure");
            auto released = held.back(); held.pop_back();
            require(pyroWaveMetalWait(pyroWaveMetalFrame(released)), "finish decode before explicit surface reuse");
            std::thread release([&] { av_frame_free(&released); }); release.join();
            require(decoder.decode(data.data(), data.size(), {}, 0, frame), "reuse surface freed on another thread");
            require(renderer.present(frame, tenBit), "render reused surface");
            av_frame_free(&frame);
            for (auto& f : held) av_frame_free(&f);
            frame = av_frame_alloc();
            require(!decoder.decode(data.data(), 3, {}, 0, frame), "reject truncated framing");
            require(decoder.decode(data.data(), data.size(), {}, 0, frame), "recover after malformed frame");
            require(renderer.present(frame, tenBit), "render recovered frame");
            av_frame_free(&frame);
            // Frames must retain their textures after decoder teardown.
            {
                PyroWaveMetalDecoder temporary;
                require(temporary.initialize(config), "temporary decoder");
                frame = av_frame_alloc();
                require(temporary.decode(data.data(), data.size(), {}, 0, frame), "temporary decode");
            }
            require(renderer.present(frame, tenBit), "frame outlives decoder");
            av_frame_free(&frame);
        }
        pyrowave_device_destroy(device);
        std::puts("PASS: native Metal decode/render, 8 format/framing combinations, pool and frame lifetime");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FAIL: %s\n", e.what()); return 1;
    }
}}
