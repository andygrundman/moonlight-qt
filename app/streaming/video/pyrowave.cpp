/**
 * @file app/streaming/video/pyrowave.cpp
 * @brief PyroWave client decoder — Stage B4 GPU zero-copy: decode into exportable dmabuf planes on
 * PyroWave's device, import into libplacebo, YUV->RGB + swapchain present. No CPU readback.
 */
#ifdef HAVE_PYROWAVE

  #include "pyrowave.h"  // this decoder's header (pulls vulkan + libplacebo + SDL_vulkan)

  // PyroWave C API. vulkan.h is already included by our header; the angle-bracket form resolves to
  // the pyrowave submodule (not this same-named local header).
  #include <pyrowave.h>

  #include "streaming/session.h"

  #include <drm_fourcc.h>

  #include <cstring>
  #include <vector>

namespace {
  uint32_t findMemoryType(VkPhysicalDevice pd, uint32_t bits, VkMemoryPropertyFlags props) {
    VkPhysicalDeviceMemoryProperties mp;
    vkGetPhysicalDeviceMemoryProperties(pd, &mp);
    for (uint32_t i = 0; i < mp.memoryTypeCount; i++) {
      if ((bits & (1u << i)) && (mp.memoryTypes[i].propertyFlags & props) == props) {
        return i;
      }
    }
    return UINT32_MAX;
  }

  std::vector<uint64_t> storageModifiers(VkPhysicalDevice pd, VkFormat format) {
    VkDrmFormatModifierPropertiesListEXT list{VK_STRUCTURE_TYPE_DRM_FORMAT_MODIFIER_PROPERTIES_LIST_EXT};
    VkFormatProperties2 fp{VK_STRUCTURE_TYPE_FORMAT_PROPERTIES_2};
    fp.pNext = &list;
    vkGetPhysicalDeviceFormatProperties2(pd, format, &fp);
    std::vector<VkDrmFormatModifierPropertiesEXT> props(list.drmFormatModifierCount);
    list.pDrmFormatModifierProperties = props.data();
    vkGetPhysicalDeviceFormatProperties2(pd, format, &fp);
    std::vector<uint64_t> out;
    for (auto& m : props) {
      if ((m.drmFormatModifierTilingFeatures & VK_FORMAT_FEATURE_STORAGE_IMAGE_BIT) &&
          (m.drmFormatModifierTilingFeatures & VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT)) {
        out.push_back(m.drmFormatModifier);
      }
    }
    return out;
  }
}  // namespace

PyroWaveVideoDecoder::PyroWaveVideoDecoder(bool testOnly)
    : m_TestOnly(testOnly), m_Width(0), m_Height(0), m_YUV444(false), m_TenBit(false),
      m_HdrEnabled(false), m_LastColorspace{}, m_Window(nullptr),
      m_PyroDevice(nullptr), m_Decoder(nullptr), m_VkDev(VK_NULL_HANDLE), m_VkPhys(VK_NULL_HANDLE),
      m_VkQueue(VK_NULL_HANDLE), m_VkFamily(0), m_VkPool(VK_NULL_HANDLE), m_VkCmd(VK_NULL_HANDLE),
      m_GetMemoryFd(nullptr), m_GetImgMod(nullptr),
      m_Log(nullptr), m_PlVkInstance(nullptr), m_VkSurface(VK_NULL_HANDLE), m_Vulkan(nullptr),
      m_Swapchain(nullptr), m_Renderer(nullptr), m_DestroySurface(nullptr),
      m_PlSem(VK_NULL_HANDLE), m_SyncObj(nullptr), m_PwSem(VK_NULL_HANDLE),
      m_TlNext(0), m_LastHoldVal(0), m_LastReleaseVal(0), m_TimelineReady(false),
      m_FrameReady(false), m_LastFrameNumber(0), m_RenderedFrames(0), m_TotalRenderTimeUs(0),
      m_OverlayLock(0) {}

PyroWaveVideoDecoder::~PyroWaveVideoDecoder() {
    // Stop overlay updates before we tear down the libplacebo GPU that owns the overlay textures.
    // Only the real (non-testOnly) decoder registers itself, and only while a Session is active
    // (the testOnly probe runs during validateLaunch before Session::get() is set).
    if (!m_TestOnly && Session::get() != nullptr) {
        Session::get()->getOverlayManager().setOverlayRenderer(nullptr);
    }

    if (m_Decoder) {
        pyrowave_decoder_destroy(m_Decoder);  // ensures PyroWave GPU is idle
    }
    if (m_SyncObj) {
        pyrowave_sync_object_destroy(m_SyncObj);
    }
    // libplacebo cleanup (also releases imported plane textures + their dmabuf fds).
    if (m_Vulkan) {
        pl_gpu_finish(m_Vulkan->gpu);
        for (auto& o : m_Overlays) {
            pl_tex_destroy(m_Vulkan->gpu, &o.overlay.tex);
            pl_tex_destroy(m_Vulkan->gpu, &o.stagingOverlay.tex);
        }
    }
    for (auto& p : m_Planes) {
        if (p.plTex) {
            pl_tex_destroy(m_Vulkan->gpu, &p.plTex);
        }
    }
    if (m_PlSem && m_Vulkan) {
        vkDestroySemaphore(m_Vulkan->device, m_PlSem, nullptr);
    }
    if (m_Renderer) {
        pl_renderer_destroy(&m_Renderer);
    }
    if (m_Swapchain) {
        pl_swapchain_destroy(&m_Swapchain);
    }
    if (m_Vulkan) {
        pl_vulkan_destroy(&m_Vulkan);
    }
    if (m_VkSurface && m_DestroySurface) {
        m_DestroySurface(m_PlVkInstance->instance, m_VkSurface, nullptr);
    }
    if (m_PlVkInstance) {
        pl_vk_inst_destroy(&m_PlVkInstance);
    }
    if (m_Log) {
        pl_log_destroy(&m_Log);
    }
    // PyroWave device cleanup.
    if (m_VkDev) {
        vkDeviceWaitIdle(m_VkDev);
        for (auto& p : m_Planes) {
            if (p.view) vkDestroyImageView(m_VkDev, p.view, nullptr);
            if (p.image) vkDestroyImage(m_VkDev, p.image, nullptr);
            if (p.mem) vkFreeMemory(m_VkDev, p.mem, nullptr);
        }
        if (m_VkPool) vkDestroyCommandPool(m_VkDev, m_VkPool, nullptr);
    }
    if (m_PyroDevice) {
        pyrowave_device_destroy(m_PyroDevice);
    }
}

bool PyroWaveVideoDecoder::createPyroDevice() {
    if (pyrowave_create_device_by_compat(0, 0, nullptr, nullptr, nullptr, &m_PyroDevice) != PYROWAVE_SUCCESS) {
        return false;
    }
    pyrowave_device_get_vk_device_handles(m_PyroDevice, nullptr, &m_VkPhys, &m_VkDev);
    m_GetMemoryFd = (PFN_vkGetMemoryFdKHR) vkGetDeviceProcAddr(m_VkDev, "vkGetMemoryFdKHR");
    m_GetImgMod = (PFN_vkGetImageDrmFormatModifierPropertiesEXT) vkGetDeviceProcAddr(m_VkDev, "vkGetImageDrmFormatModifierPropertiesEXT");
    if (!m_GetMemoryFd || !m_GetImgMod) {
        return false;
    }
    uint32_t qn = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(m_VkPhys, &qn, nullptr);
    std::vector<VkQueueFamilyProperties> qf(qn);
    vkGetPhysicalDeviceQueueFamilyProperties(m_VkPhys, &qn, qf.data());
    m_VkFamily = UINT32_MAX;
    for (uint32_t i = 0; i < qn; i++) {
        if ((qf[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) && (qf[i].queueFlags & VK_QUEUE_COMPUTE_BIT)) {
            m_VkFamily = i;
            break;
        }
    }
    if (m_VkFamily == UINT32_MAX) {
        return false;
    }
    vkGetDeviceQueue(m_VkDev, m_VkFamily, 0, &m_VkQueue);
    VkCommandPoolCreateInfo pci{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    pci.queueFamilyIndex = m_VkFamily;
    pci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    if (vkCreateCommandPool(m_VkDev, &pci, nullptr, &m_VkPool) != VK_SUCCESS) {
        return false;
    }
    VkCommandBufferAllocateInfo cai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    cai.commandPool = m_VkPool;
    cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cai.commandBufferCount = 1;
    return vkAllocateCommandBuffers(m_VkDev, &cai, &m_VkCmd) == VK_SUCCESS;
}

bool PyroWaveVideoDecoder::createPlanes() {
    VkFormat planeFmt = m_TenBit ? VK_FORMAT_R16_UNORM : VK_FORMAT_R8_UNORM;
    auto mods = storageModifiers(m_VkPhys, planeFmt);

    // Restrict to modifiers libplacebo can import for this format, or the pl_tex_create import
    // in importPlanes() will reject whatever the driver picked (seen on Van Gogh/RDNA2).
    int depth = m_TenBit ? 16 : 8;
    pl_fmt plFmt = pl_find_fmt(m_Vulkan->gpu, PL_FMT_UNORM, 1, depth, depth, PL_FMT_CAP_SAMPLEABLE);
    if (!plFmt) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "PyroWave: no sampleable %d-bit plane format in libplacebo", depth);
        return false;
    }
    std::vector<uint64_t> importable;
    for (uint64_t m : mods) {
        for (int i = 0; i < plFmt->num_modifiers; i++) {
            if (plFmt->modifiers[i] == m) {
                importable.push_back(m);
                break;
            }
        }
    }
    if (importable.empty()) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                     "PyroWave: no DRM modifier is both storage-capable and libplacebo-importable "
                     "(%zu storage, %d importable)", mods.size(), plFmt->num_modifiers);
        return false;
    }
    mods = std::move(importable);
    int chromaW = m_YUV444 ? m_Width : m_Width / 2;
    int chromaH = m_YUV444 ? m_Height : m_Height / 2;
    int dims[3][2] = {{m_Width, m_Height}, {chromaW, chromaH}, {chromaW, chromaH}};
    for (int i = 0; i < 3; i++) {
        Plane& p = m_Planes[i];
        p.w = dims[i][0];
        p.h = dims[i][1];

        VkExternalMemoryImageCreateInfo ext{VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO};
        ext.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT;
        VkImageDrmFormatModifierListCreateInfoEXT ml{VK_STRUCTURE_TYPE_IMAGE_DRM_FORMAT_MODIFIER_LIST_CREATE_INFO_EXT};
        ml.drmFormatModifierCount = (uint32_t) mods.size();
        ml.pDrmFormatModifiers = mods.data();
        ml.pNext = &ext;
        VkImageCreateInfo ci{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
        ci.pNext = &ml;
        ci.imageType = VK_IMAGE_TYPE_2D;
        ci.format = planeFmt;
        ci.extent = {(uint32_t) p.w, (uint32_t) p.h, 1};
        ci.mipLevels = 1;
        ci.arrayLayers = 1;
        ci.samples = VK_SAMPLE_COUNT_1_BIT;
        ci.tiling = VK_IMAGE_TILING_DRM_FORMAT_MODIFIER_EXT;
        ci.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        if (vkCreateImage(m_VkDev, &ci, nullptr, &p.image) != VK_SUCCESS) {
            return false;
        }
        VkMemoryRequirements mr;
        vkGetImageMemoryRequirements(m_VkDev, p.image, &mr);
        p.size = mr.size;
        VkExportMemoryAllocateInfo emai{VK_STRUCTURE_TYPE_EXPORT_MEMORY_ALLOCATE_INFO};
        emai.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT;
        VkMemoryDedicatedAllocateInfo ded{VK_STRUCTURE_TYPE_MEMORY_DEDICATED_ALLOCATE_INFO};
        ded.image = p.image;
        ded.pNext = &emai;
        VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        ai.pNext = &ded;
        ai.allocationSize = mr.size;
        ai.memoryTypeIndex = findMemoryType(m_VkPhys, mr.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        if (ai.memoryTypeIndex == UINT32_MAX || vkAllocateMemory(m_VkDev, &ai, nullptr, &p.mem) != VK_SUCCESS) {
            return false;
        }
        vkBindImageMemory(m_VkDev, p.image, p.mem, 0);
        VkImageViewCreateInfo vi{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
        vi.image = p.image;
        vi.viewType = VK_IMAGE_VIEW_TYPE_2D;
        vi.format = planeFmt;
        vi.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        if (vkCreateImageView(m_VkDev, &vi, nullptr, &p.view) != VK_SUCCESS) {
            return false;
        }

        // Query the actual modifier + plane layout, then export the dmabuf fd.
        VkImageDrmFormatModifierPropertiesEXT mp{VK_STRUCTURE_TYPE_IMAGE_DRM_FORMAT_MODIFIER_PROPERTIES_EXT};
        m_GetImgMod(m_VkDev, p.image, &mp);
        p.modifier = mp.drmFormatModifier;
        VkImageSubresource sr{VK_IMAGE_ASPECT_MEMORY_PLANE_0_BIT_EXT, 0, 0};
        VkSubresourceLayout sl{};
        vkGetImageSubresourceLayout(m_VkDev, p.image, &sr, &sl);
        p.offset = sl.offset;
        p.pitch = sl.rowPitch;
        VkMemoryGetFdInfoKHR gfi{VK_STRUCTURE_TYPE_MEMORY_GET_FD_INFO_KHR};
        gfi.memory = p.mem;
        gfi.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT;
        if (m_GetMemoryFd(m_VkDev, &gfi, &p.fd) != VK_SUCCESS) {
            return false;
        }
    }

    // Transition all planes to GENERAL so PyroWave's decode compute can write them.
    VkCommandBufferBeginInfo bg{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    bg.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(m_VkCmd, &bg);
    for (auto& p : m_Planes) {
        VkImageMemoryBarrier b{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
        b.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        b.newLayout = VK_IMAGE_LAYOUT_GENERAL;
        b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.image = p.image;
        b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        b.dstAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        vkCmdPipelineBarrier(m_VkCmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                             0, 0, nullptr, 0, nullptr, 1, &b);
    }
    vkEndCommandBuffer(m_VkCmd);
    VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    si.commandBufferCount = 1;
    si.pCommandBuffers = &m_VkCmd;
    vkQueueSubmit(m_VkQueue, 1, &si, VK_NULL_HANDLE);
    vkQueueWaitIdle(m_VkQueue);
    return true;
}

bool PyroWaveVideoDecoder::createLibplacebo(PDECODER_PARAMETERS params) {
    pl_log_params logParams = pl_log_default_params;
    logParams.log_cb = [](void*, pl_log_level lvl, const char* msg) {
        if (lvl <= PL_LOG_WARN) SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "[pl] %s", msg);
    };
    logParams.log_level = PL_LOG_INFO;
    m_Log = pl_log_create(PL_API_VER, &logParams);

    unsigned int extCount = 0;
    if (!SDL_Vulkan_GetInstanceExtensions(params->window, &extCount, nullptr)) {
        return false;
    }
    std::vector<const char*> exts(extCount);
    if (!SDL_Vulkan_GetInstanceExtensions(params->window, &extCount, exts.data())) {
        return false;
    }
    pl_vk_inst_params ip = pl_vk_inst_default_params;
    ip.get_proc_addr = (PFN_vkGetInstanceProcAddr) SDL_Vulkan_GetVkGetInstanceProcAddr();
    ip.extensions = exts.data();
    ip.num_extensions = (int) exts.size();
    m_PlVkInstance = pl_vk_inst_create(m_Log, &ip);
    if (!m_PlVkInstance) {
        return false;
    }
    m_DestroySurface = (PFN_vkDestroySurfaceKHR) m_PlVkInstance->get_proc_addr(m_PlVkInstance->instance, "vkDestroySurfaceKHR");

    if (!SDL_Vulkan_CreateSurface(params->window, m_PlVkInstance->instance, &m_VkSurface)) {
        return false;
    }

    pl_vulkan_params vp = pl_vulkan_default_params;
    vp.instance = m_PlVkInstance->instance;
    vp.get_proc_addr = m_PlVkInstance->get_proc_addr;
    vp.surface = m_VkSurface;
    m_Vulkan = pl_vulkan_create(m_Log, &vp);
    if (!m_Vulkan) {
        return false;
    }

    pl_vulkan_swapchain_params sp = {};
    sp.surface = m_VkSurface;
    sp.present_mode = VK_PRESENT_MODE_FIFO_KHR;
    sp.swapchain_depth = 1;
    m_Swapchain = pl_vulkan_create_swapchain(m_Vulkan, &sp);
    if (!m_Swapchain) {
        return false;
    }
    m_Renderer = pl_renderer_create(m_Log, m_Vulkan->gpu);
    return m_Renderer != nullptr;
}

bool PyroWaveVideoDecoder::importPlanes() {
    int depth = m_TenBit ? 16 : 8;
    pl_fmt fmt = pl_find_fmt(m_Vulkan->gpu, PL_FMT_UNORM, 1, depth, depth, PL_FMT_CAP_SAMPLEABLE);
    if (!fmt) {
        return false;
    }
    for (auto& p : m_Planes) {
        pl_tex_params tp = {};
        tp.w = p.w;
        tp.h = p.h;
        tp.format = fmt;
        tp.sampleable = true;
        tp.import_handle = PL_HANDLE_DMA_BUF;
        tp.shared_mem.handle.fd = p.fd;  // libplacebo takes ownership of the fd
        tp.shared_mem.size = (size_t) p.size;
        tp.shared_mem.offset = (size_t) p.offset;
        tp.shared_mem.drm_format_mod = p.modifier;
        tp.shared_mem.stride_w = (size_t) p.pitch;
        p.plTex = pl_tex_create(m_Vulkan->gpu, &tp);
        if (!p.plTex) {
            return false;
        }
    }
    return true;
}

bool PyroWaveVideoDecoder::createSharedTimeline() {
    // libplacebo creates an exportable timeline semaphore; PyroWave imports the fd. Both then
    // signal/wait the same timeline for cross-device sync (replaces the full-device vkDeviceWaitIdle).
    union pl_handle handle{};
    struct pl_vulkan_sem_params sp{};
    sp.type = VK_SEMAPHORE_TYPE_TIMELINE;
    sp.initial_value = 0;
    sp.export_handle = PL_HANDLE_FD;
    sp.out_handle = &handle;
    m_PlSem = pl_vulkan_sem_create(m_Vulkan->gpu, &sp);
    if (!m_PlSem) {
        return false;
    }
    pyrowave_sync_object_create_info si{};
    si.device = m_PyroDevice;
    si.external_handle = (pyrowave_os_handle) handle.fd;
    si.handle_type = VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_OPAQUE_FD_BIT;
    si.semaphore_type = VK_SEMAPHORE_TYPE_TIMELINE;
    si.import_flags = 0;
    if (pyrowave_sync_object_create(&si, &m_SyncObj) != PYROWAVE_SUCCESS) {
        return false;
    }
    m_PwSem = pyrowave_sync_object_get_semaphore(m_SyncObj);

    // Put the plane textures into external-owned ("held") state so the first decode can write them.
    // The first decode's acquire waits on the last hold value published here.
    for (auto& p : m_Planes) {
        uint64_t v = ++m_TlNext;
        struct pl_vulkan_hold_params hp{};
        hp.tex = p.plTex;
        hp.layout = VK_IMAGE_LAYOUT_GENERAL;
        hp.qf = VK_QUEUE_FAMILY_IGNORED;
        hp.semaphore = {m_PlSem, v};
        pl_vulkan_hold_ex(m_Vulkan->gpu, &hp);
        m_LastHoldVal.store(v);
    }
    return true;
}

bool PyroWaveVideoDecoder::initialize(PDECODER_PARAMETERS params) {
    if (!(params->videoFormat & VIDEO_FORMAT_MASK_PYROWAVE)) {
        return false;
    }
    m_YUV444 = !!(params->videoFormat & (VIDEO_FORMAT_PYROWAVE_444 | VIDEO_FORMAT_PYROWAVE10_444));
    m_TenBit = !!(params->videoFormat & VIDEO_FORMAT_MASK_10BIT);
    m_Width = params->width & ~1;
    m_Height = params->height & ~1;
    m_Window = params->window;

    if (!createPyroDevice()) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "PyroWave: device init failed");
        return false;
    }

    pyrowave_decoder_create_info ci{};
    ci.device = m_PyroDevice;
    ci.width = m_Width;
    ci.height = m_Height;
    ci.chroma = m_YUV444 ? PYROWAVE_CHROMA_SUBSAMPLING_444 : PYROWAVE_CHROMA_SUBSAMPLING_420;
    ci.fragment_path = pyrowave_decoder_device_prefers_fragment_path(m_PyroDevice);
    if (pyrowave_decoder_create(&ci, &m_Decoder) != PYROWAVE_SUCCESS) {
        return false;
    }

    if (m_TestOnly) {
        return true;
    }

    // libplacebo first: plane images must be created with a DRM modifier libplacebo can import
    // (createPlanes intersects the storage-capable modifiers with pl_fmt's import list — on some
    // GPUs, e.g. Steam Deck's Van Gogh, radv otherwise picks a storage modifier pl can't sample).
    if (!createLibplacebo(params) || !createPlanes() || !importPlanes() || !createSharedTimeline()) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "PyroWave: GPU present pipeline init failed");
        return false;
    }
    m_TimelineReady = true;
    // Composite the performance/status overlay on top of the video (same as the FFmpeg renderers).
    if (Session::get() != nullptr) {
        Session::get()->getOverlayManager().setOverlayRenderer(this);
    }

    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "PyroWave GPU zero-copy decoder ready: %dx%d %s %s", m_Width, m_Height, m_YUV444 ? "4:4:4" : "4:2:0", m_TenBit ? "10-bit" : "8-bit");
    return true;
}

void PyroWaveVideoDecoder::overlayUploadComplete(void* opaque) {
    SDL_FreeSurface((SDL_Surface*) opaque);
}

// Upload an overlay text surface (ARGB8888) to a libplacebo texture. Ported from PlVkRenderer.
bool PyroWaveVideoDecoder::createOverlay(pl_overlay* overlay, SDL_Surface* surface) {
    pl_fmt texFormat = pl_find_named_fmt(m_Vulkan->gpu, "bgra8");
    if (!texFormat) {
        SDL_FreeSurface(surface);
        return false;
    }
    pl_tex_params texParams = {};
    texParams.w = surface->w;
    texParams.h = surface->h;
    texParams.format = texFormat;
    texParams.sampleable = true;
    texParams.host_writable = true;
    texParams.blit_src = !!(texFormat->caps & PL_FMT_CAP_BLITTABLE);
    if (!pl_tex_recreate(m_Vulkan->gpu, &overlay->tex, &texParams)) {
        pl_tex_destroy(m_Vulkan->gpu, &overlay->tex);
        SDL_zerop(overlay);
        SDL_FreeSurface(surface);
        return false;
    }
    pl_tex_transfer_params xfer = {};
    xfer.tex = overlay->tex;
    xfer.row_pitch = (size_t) surface->pitch;
    xfer.ptr = surface->pixels;
    xfer.callback = overlayUploadComplete;
    xfer.priv = surface;
    if (!pl_tex_upload(m_Vulkan->gpu, &xfer)) {
        pl_tex_destroy(m_Vulkan->gpu, &overlay->tex);
        SDL_zerop(overlay);
        SDL_FreeSurface(surface);
        return false;
    }
    // surface is now owned by the upload (freed in overlayUploadComplete).
    overlay->mode = PL_OVERLAY_NORMAL;
    overlay->coords = PL_OVERLAY_COORDS_DST_FRAME;
    overlay->repr = pl_color_repr_rgb;
    overlay->color = pl_color_space_srgb;
    return true;
}

void PyroWaveVideoDecoder::notifyOverlayUpdated(Overlay::OverlayType type) {
    SDL_Surface* newSurface = Session::get()->getOverlayManager().getUpdatedOverlaySurface(type);
    if (newSurface == nullptr && Session::get()->getOverlayManager().isOverlayEnabled(type)) {
        return;  // enabled but no new surface: keep the existing texture
    }

    SDL_AtomicLock(&m_OverlayLock);
    m_Overlays[type].hasStagingOverlay = false;
    SDL_AtomicUnlock(&m_OverlayLock);

    if (newSurface == nullptr) {
        pl_tex_destroy(m_Vulkan->gpu, &m_Overlays[type].stagingOverlay.tex);
        SDL_zero(m_Overlays[type].stagingOverlay);
        return;
    }

    if (!createOverlay(&m_Overlays[type].stagingOverlay, newSurface)) {
        return;
    }

    SDL_AtomicLock(&m_OverlayLock);
    m_Overlays[type].hasStagingOverlay = true;
    SDL_AtomicUnlock(&m_OverlayLock);
}

bool PyroWaveVideoDecoder::isHardwareAccelerated() { return true; }
bool PyroWaveVideoDecoder::isAlwaysFullScreen() { return false; }
bool PyroWaveVideoDecoder::isHdrSupported() { return true; }
int PyroWaveVideoDecoder::getDecoderCapabilities() {
    // Decode directly on the receive thread: submitDecodeUnit only pushes packets and submits
    // GPU work (cross-device sync is via timeline semaphores, no CPU waits), so skipping the
    // decode-unit queue removes a thread hop + wakeup jitter from the latency path.
    return CAPABILITY_DIRECT_SUBMIT;
}
int PyroWaveVideoDecoder::getDecoderColorspace() { return COLORSPACE_REC_601; }
int PyroWaveVideoDecoder::getDecoderColorRange() { return COLOR_RANGE_LIMITED; }
QSize PyroWaveVideoDecoder::getDecoderMaxResolution() { return QSize(0, 0); }
void PyroWaveVideoDecoder::setHdrMode(bool enabled) {
    // Consumed on the render thread: switches the pl_frame colorspace + swapchain hint.
    m_HdrEnabled.store(enabled);
    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "PyroWave: HDR mode %s", enabled ? "enabled" : "disabled");
}
bool PyroWaveVideoDecoder::notifyWindowChanged(PWINDOW_STATE_CHANGE_INFO) { return false; }

void PyroWaveVideoDecoder::addVideoStats(VIDEO_STATS& src, VIDEO_STATS& dst) {
    dst.receivedFrames += src.receivedFrames;
    dst.decodedFrames += src.decodedFrames;
    dst.renderedFrames += src.renderedFrames;
    dst.totalFrames += src.totalFrames;
    dst.networkDroppedFrames += src.networkDroppedFrames;
    dst.totalReassemblyTimeUs += src.totalReassemblyTimeUs;
    dst.totalDecodeTimeUs += src.totalDecodeTimeUs;
    dst.totalRenderTimeUs += src.totalRenderTimeUs;
    if (dst.minHostProcessingLatency == 0) {
        dst.minHostProcessingLatency = src.minHostProcessingLatency;
    } else if (src.minHostProcessingLatency != 0) {
        dst.minHostProcessingLatency = qMin(dst.minHostProcessingLatency, src.minHostProcessingLatency);
    }
    dst.maxHostProcessingLatency = qMax(dst.maxHostProcessingLatency, src.maxHostProcessingLatency);
    dst.totalHostProcessingLatency += src.totalHostProcessingLatency;
    dst.framesWithHostProcessingLatency += src.framesWithHostProcessingLatency;

    if (!LiGetEstimatedRttInfo(&dst.lastRtt, &dst.lastRttVariance)) {
        dst.lastRtt = 0;
        dst.lastRttVariance = 0;
    }
    if (!dst.measurementStartUs) {
        dst.measurementStartUs = src.measurementStartUs;
    }
    double timeDiffSecs = (double) (LiGetMicroseconds() - dst.measurementStartUs) / 1000000.0;
    if (timeDiffSecs > 0) {
        dst.totalFps = (double) dst.totalFrames / timeDiffSecs;
        dst.receivedFps = (double) dst.receivedFrames / timeDiffSecs;
        dst.decodedFps = (double) dst.decodedFrames / timeDiffSecs;
        dst.renderedFps = (double) dst.renderedFrames / timeDiffSecs;
    }
}

void PyroWaveVideoDecoder::stringifyVideoStats(VIDEO_STATS& stats, char* output, int length) {
    int offset = 0;
    output[0] = 0;
    int ret;

    if (stats.receivedFps > 0) {
        ret = snprintf(&output[offset], length - offset,
                       "Video stream: %dx%d %.2f FPS (Codec: PyroWave %s%s)\n"
                       "Bitrate: %.1f Mbps, Peak (%us): %.1f\n"
                       "Incoming frame rate from network: %.2f FPS\n"
                       "Decoding frame rate: %.2f FPS\n"
                       "Rendering frame rate: %.2f FPS\n",
                       m_Width, m_Height, stats.totalFps, m_YUV444 ? "4:4:4" : "4:2:0",
                       m_TenBit ? (m_HdrEnabled.load() ? " 10-bit HDR" : " 10-bit") : "",
                       m_BwTracker.GetAverageMbps(), m_BwTracker.GetWindowSeconds(), m_BwTracker.GetPeakMbps(),
                       stats.receivedFps, stats.decodedFps, stats.renderedFps);
        if (ret < 0 || ret >= length - offset) return;
        offset += ret;
    }
    if (stats.framesWithHostProcessingLatency > 0) {
        ret = snprintf(&output[offset], length - offset,
                       "Host processing latency min/max/average: %.1f/%.1f/%.1f ms\n",
                       (float) stats.minHostProcessingLatency / 10,
                       (float) stats.maxHostProcessingLatency / 10,
                       (float) stats.totalHostProcessingLatency / 10 / stats.framesWithHostProcessingLatency);
        if (ret < 0 || ret >= length - offset) return;
        offset += ret;
    }
    if (stats.renderedFrames != 0) {
        char rttString[32];
        if (stats.lastRtt != 0) {
            snprintf(rttString, sizeof(rttString), "%u ms (variance: %u ms)", stats.lastRtt, stats.lastRttVariance);
        } else {
            snprintf(rttString, sizeof(rttString), "N/A");
        }
        ret = snprintf(&output[offset], length - offset,
                       "Frames dropped by your network connection: %.2f%%\n"
                       "Average network latency: %s\n"
                       "Average reassembly/decoding time: %.2f/%.2f ms\n"
                       "Average rendering time (including monitor V-sync latency): %.2f ms\n",
                       stats.totalFrames ? (float) stats.networkDroppedFrames / stats.totalFrames * 100 : 0.0f,
                       rttString,
                       stats.decodedFrames ? (double) (stats.totalReassemblyTimeUs / 1000.0) / stats.decodedFrames : 0.0,
                       stats.decodedFrames ? (double) (stats.totalDecodeTimeUs / 1000.0) / stats.decodedFrames : 0.0,
                       stats.renderedFrames ? (double) (stats.totalRenderTimeUs / 1000.0) / stats.renderedFrames : 0.0);
        if (ret < 0 || ret >= length - offset) return;
        offset += ret;
    }
}

int PyroWaveVideoDecoder::submitDecodeUnit(PDECODE_UNIT du) {
    // Per-frame performance stats + overlay text (mirrors FFmpegVideoDecoder).
    if (!m_LastFrameNumber) {
        m_ActiveWndVideoStats.measurementStartUs = LiGetMicroseconds();
        m_LastFrameNumber = du->frameNumber;
    } else {
        m_ActiveWndVideoStats.networkDroppedFrames += du->frameNumber - (m_LastFrameNumber + 1);
        m_ActiveWndVideoStats.totalFrames += du->frameNumber - (m_LastFrameNumber + 1);
        m_LastFrameNumber = du->frameNumber;
    }

    // Flip stats windows roughly every second and refresh the overlay text if enabled.
    if (LiGetMicroseconds() > m_ActiveWndVideoStats.measurementStartUs + 1000000) {
        m_ActiveWndVideoStats.renderedFrames = m_RenderedFrames.exchange(0);
        m_ActiveWndVideoStats.totalRenderTimeUs = m_TotalRenderTimeUs.exchange(0);

        if (Session::get()->getOverlayManager().isOverlayEnabled(Overlay::OverlayDebug)) {
            VIDEO_STATS lastTwoWndStats = {};
            addVideoStats(m_LastWndVideoStats, lastTwoWndStats);
            addVideoStats(m_ActiveWndVideoStats, lastTwoWndStats);
            stringifyVideoStats(lastTwoWndStats,
                                Session::get()->getOverlayManager().getOverlayText(Overlay::OverlayDebug),
                                Session::get()->getOverlayManager().getOverlayMaxTextLength());
            Session::get()->getOverlayManager().setOverlayTextUpdated(Overlay::OverlayDebug);
        }

        SDL_memcpy(&m_LastWndVideoStats, &m_ActiveWndVideoStats, sizeof(m_ActiveWndVideoStats));
        SDL_zero(m_ActiveWndVideoStats);
        m_ActiveWndVideoStats.measurementStartUs = LiGetMicroseconds();
    }

    if (du->frameHostProcessingLatency != 0) {
        if (m_ActiveWndVideoStats.minHostProcessingLatency != 0) {
            m_ActiveWndVideoStats.minHostProcessingLatency = qMin(m_ActiveWndVideoStats.minHostProcessingLatency, du->frameHostProcessingLatency);
        } else {
            m_ActiveWndVideoStats.minHostProcessingLatency = du->frameHostProcessingLatency;
        }
        m_ActiveWndVideoStats.framesWithHostProcessingLatency += 1;
    }
    m_ActiveWndVideoStats.maxHostProcessingLatency = qMax(m_ActiveWndVideoStats.maxHostProcessingLatency, du->frameHostProcessingLatency);
    m_ActiveWndVideoStats.totalHostProcessingLatency += du->frameHostProcessingLatency;
    m_ActiveWndVideoStats.receivedFrames++;
    m_ActiveWndVideoStats.totalFrames++;

    m_BwTracker.AddBytes(du->fullLength);
    // Reassembly (receive-to-enqueue) happens on the receive thread and is otherwise invisible here.
    m_ActiveWndVideoStats.totalReassemblyTimeUs += (du->enqueueTimeUs - du->receiveTimeUs);

    uint64_t decodeStartUs = LiGetMicroseconds();

    // Host frames the PyroWave packets as [u32 count]{[u32 size][bytes]}*. Parse the buffer
    // chain in place instead of flattening it: only packets straddling an entry boundary are
    // copied, into a persistent scratch (packets are ~1 KB by host packetization, so the copy
    // is small and the scratch stops growing after the first frames).
    PLENTRY entry = du->bufferList;
    size_t entryOff = 0;
    size_t remaining = (size_t) du->fullLength;

    // Returns n contiguous bytes and advances the cursor: in place from the current chain
    // entry when possible, else gathered into dst.
    auto readBytes = [&](uint8_t* dst, size_t n) -> const uint8_t* {
        if (n > remaining) {
            return nullptr;
        }
        remaining -= n;
        if (entry && entryOff + n <= (size_t) entry->length) {
            const uint8_t* p = reinterpret_cast<const uint8_t*>(entry->data) + entryOff;
            entryOff += n;
            if (entryOff == (size_t) entry->length) {
                entry = entry->next;
                entryOff = 0;
            }
            return p;
        }
        for (size_t got = 0; got < n; ) {
            if (!entry) {
                return nullptr;
            }
            size_t take = qMin((size_t) entry->length - entryOff, n - got);
            SDL_memcpy(dst + got, reinterpret_cast<const uint8_t*>(entry->data) + entryOff, take);
            got += take;
            entryOff += take;
            if (entryOff == (size_t) entry->length) {
                entry = entry->next;
                entryOff = 0;
            }
        }
        return dst;
    };
    auto u32 = [&](uint32_t& v) -> bool {
        uint8_t tmp[4];
        const uint8_t* p = readBytes(tmp, 4);
        if (!p) {
            return false;
        }
        v = (uint32_t) p[0] | ((uint32_t) p[1] << 8) | ((uint32_t) p[2] << 16) | ((uint32_t) p[3] << 24);
        return true;
    };
    uint32_t count = 0;
    if (!u32(count)) return DR_OK;
    for (uint32_t i = 0; i < count; i++) {
        uint32_t sz = 0;
        if (!u32(sz) || sz > remaining) break;
        if (sz > m_PacketScratch.size()) {
            m_PacketScratch.resize(sz);
        }
        const uint8_t* pkt = readBytes(m_PacketScratch.data(), sz);
        if (!pkt) break;
        pyrowave_decoder_push_packet(m_Decoder, pkt, sz);
    }
    if (!pyrowave_decoder_decode_is_ready(m_Decoder, true)) {
        return DR_OK;
    }

    VkFormat planeFmt = m_TenBit ? VK_FORMAT_R16_UNORM : VK_FORMAT_R8_UNORM;
    auto view = [&](Plane& p) {
        pyrowave_image_view v{};
        v.image = p.image;
        v.width = (uint32_t) p.w;
        v.height = (uint32_t) p.h;
        v.image_format = planeFmt;
        v.view_format = planeFmt;
        v.aspect = VK_IMAGE_ASPECT_COLOR_BIT;
        v.swizzle = VK_COMPONENT_SWIZZLE_IDENTITY;
        v.layout = VK_IMAGE_LAYOUT_GENERAL;
        return v;
    };
    pyrowave_gpu_buffers gb{};
    gb.planes[0] = view(m_Planes[0]);
    gb.planes[1] = view(m_Planes[1]);
    gb.planes[2] = view(m_Planes[2]);

    // Cross-device sync via the shared timeline: the decode GPU-waits until libplacebo finished its
    // last read of these planes (WAR), and signals decode-done for libplacebo to wait on (RAW).
    // No CPU stall — the waits happen on the GPU.
    uint64_t waitVal = m_LastHoldVal.load();
    uint64_t sigVal = ++m_TlNext;
    pyrowave_gpu_sync_operation acquire{};
    acquire.sync = {m_PwSem, waitVal};
    pyrowave_gpu_sync_operation release{};
    release.sync = {m_PwSem, sigVal};
    {
        std::lock_guard<std::mutex> lock(m_FrameLock);
        if (pyrowave_decoder_decode_gpu_buffer(m_Decoder, &acquire, &release, &gb) != PYROWAVE_SUCCESS) {
            return DR_OK;
        }
        m_LastReleaseVal.store(sigVal);
        m_FrameReady = true;
    }

    m_ActiveWndVideoStats.totalDecodeTimeUs += LiGetMicroseconds() - decodeStartUs;
    m_ActiveWndVideoStats.decodedFrames++;

    SDL_Event event;
    SDL_zero(event);
    event.type = SDL_USEREVENT;
    event.user.code = SDL_CODE_FRAME_READY;
    SDL_PushEvent(&event);
    return DR_OK;
}

void PyroWaveVideoDecoder::renderFrameOnMainThread() {
    uint64_t renderStartUs = LiGetMicroseconds();
    {
        std::lock_guard<std::mutex> lock(m_FrameLock);
        if (!m_FrameReady) {
            return;
        }
        m_FrameReady = false;
    }

    // Reacquire the plane textures from external (PyroWave) use, GPU-waiting on the decode-done
    // timeline value (RAW). Always paired with the hold_ex below so ownership state stays balanced.
    uint64_t decodeVal = m_LastReleaseVal.load();
    for (auto& p : m_Planes) {
        struct pl_vulkan_release_params rp{};
        rp.tex = p.plTex;
        rp.layout = VK_IMAGE_LAYOUT_GENERAL;
        rp.qf = VK_QUEUE_FAMILY_IGNORED;
        rp.semaphore = {m_PlSem, decodeVal};
        pl_vulkan_release_ex(m_Vulkan->gpu, &rp);
    }

    // Source colorspace + representation follow the host's HDR state (control-stream driven, can
    // change mid-stream). Must match the encoder's shader convention:
    //   SDR: BT.601 limited on sRGB R'G'B'.  HDR: BT.2020 NCL FULL range on PQ R'G'B'.
    bool hdr = m_HdrEnabled.load();
    struct pl_color_space csp = {};
    struct pl_color_repr repr = {};
    if (hdr) {
        csp.primaries = PL_COLOR_PRIM_BT_2020;
        csp.transfer = PL_COLOR_TRC_PQ;
        SS_HDR_METADATA md;
        if (LiGetHdrMetadata(&md)) {
            csp.hdr.prim.red = {md.displayPrimaries[0].x / 50000.0f, md.displayPrimaries[0].y / 50000.0f};
            csp.hdr.prim.green = {md.displayPrimaries[1].x / 50000.0f, md.displayPrimaries[1].y / 50000.0f};
            csp.hdr.prim.blue = {md.displayPrimaries[2].x / 50000.0f, md.displayPrimaries[2].y / 50000.0f};
            csp.hdr.prim.white = {md.whitePoint.x / 50000.0f, md.whitePoint.y / 50000.0f};
            csp.hdr.max_luma = md.maxDisplayLuminance;
            // Sub-nit precision; PL_COLOR_HDR_BLACK (infinite contrast) when unset, like plvk.
            csp.hdr.min_luma = md.minDisplayLuminance ? md.minDisplayLuminance / 10000.0f : PL_COLOR_HDR_BLACK;
            csp.hdr.max_cll = md.maxContentLightLevel;
            csp.hdr.max_fall = md.maxFrameAverageLightLevel;
        }
        repr.sys = PL_COLOR_SYSTEM_BT_2020_NC;
        repr.levels = PL_COLOR_LEVELS_FULL;
    } else {
        // YUV matrix must match the encoder (BT.601 limited). Primaries/transfer describe the
        // RGB volume, which is the captured desktop = sRGB.
        csp.primaries = PL_COLOR_PRIM_BT_709;
        csp.transfer = PL_COLOR_TRC_SRGB;
        repr.sys = PL_COLOR_SYSTEM_BT_601;
        repr.levels = PL_COLOR_LEVELS_LIMITED;
    }
    if (!pl_color_space_equal(&csp, &m_LastColorspace)) {
        m_LastColorspace = csp;
        // libplacebo picks the closest supported swapchain colorspace (HDR10 when available) and
        // tonemaps otherwise, so an SDR-only display still presents HDR streams correctly.
        pl_swapchain_colorspace_hint(m_Swapchain, &csp);
    }

    int dw = 0, dh = 0;
    SDL_Vulkan_GetDrawableSize(m_Window, &dw, &dh);
    if (pl_swapchain_resize(m_Swapchain, &dw, &dh)) {
        struct pl_swapchain_frame scFrame;
        if (pl_swapchain_start_frame(m_Swapchain, &scFrame)) {
            struct pl_frame target;
            pl_frame_from_swapchain(&target, &scFrame);

            // Promote any staged overlays and build the overlay list to composite (perf/status).
            pl_overlay_part overlayParts[Overlay::OverlayMax] = {};
            std::vector<pl_overlay> overlays;
            std::vector<pl_tex> overlayTexToDestroy;
            SDL_AtomicLock(&m_OverlayLock);
            for (int i = 0; i < Overlay::OverlayMax; i++) {
                if (m_Overlays[i].hasStagingOverlay) {
                    if (m_Overlays[i].hasOverlay) {
                        overlayTexToDestroy.push_back(m_Overlays[i].overlay.tex);
                    }
                    m_Overlays[i].overlay = m_Overlays[i].stagingOverlay;
                    m_Overlays[i].hasStagingOverlay = false;
                    SDL_zero(m_Overlays[i].stagingOverlay);
                    m_Overlays[i].hasOverlay = true;
                }
                if (m_Overlays[i].hasOverlay &&
                    !Session::get()->getOverlayManager().isOverlayEnabled((Overlay::OverlayType) i)) {
                    overlayTexToDestroy.push_back(m_Overlays[i].overlay.tex);
                    SDL_zero(m_Overlays[i].overlay);
                    m_Overlays[i].hasOverlay = false;
                }
                if (m_Overlays[i].hasOverlay) {
                    overlayParts[i].src = {0, 0, (float) m_Overlays[i].overlay.tex->params.w, (float) m_Overlays[i].overlay.tex->params.h};
                    overlayParts[i].dst.x0 = 0;
                    if (i == Overlay::OverlayStatusUpdate) {
                        overlayParts[i].dst.y0 = SDL_max(0.0f, target.crop.y1 - overlayParts[i].src.y1);  // bottom-left
                    } else {
                        overlayParts[i].dst.y0 = 0;  // top-left
                    }
                    overlayParts[i].dst.x1 = overlayParts[i].dst.x0 + overlayParts[i].src.x1;
                    overlayParts[i].dst.y1 = overlayParts[i].dst.y0 + overlayParts[i].src.y1;
                    m_Overlays[i].overlay.parts = &overlayParts[i];
                    m_Overlays[i].overlay.num_parts = 1;
                    overlays.push_back(m_Overlays[i].overlay);
                }
            }
            SDL_AtomicUnlock(&m_OverlayLock);

            struct pl_frame src = {};
            src.num_planes = 3;
            for (int i = 0; i < 3; i++) {
                src.planes[i].texture = m_Planes[i].plTex;
                src.planes[i].components = 1;
                src.planes[i].component_mapping[0] = i;  // 0=Y, 1=Cb, 2=Cr
            }
            src.repr = repr;
            src.color = csp;
            src.crop.x0 = 0;
            src.crop.y0 = 0;
            src.crop.x1 = m_Width;
            src.crop.y1 = m_Height;

            target.num_overlays = (int) overlays.size();
            target.overlays = overlays.data();

            pl_render_image(m_Renderer, &src, &target, &pl_render_fast_params);
            pl_swapchain_submit_frame(m_Swapchain);
            pl_swapchain_swap_buffers(m_Swapchain);

            for (auto t : overlayTexToDestroy) {
                pl_tex_destroy(m_Vulkan->gpu, &t);
            }
        }
    }

    // Hand the plane textures back to external (PyroWave) use, signaling render-done values that the
    // next decode's acquire waits on (WAR). Must always run to keep hold/release balanced.
    for (auto& p : m_Planes) {
        uint64_t v = ++m_TlNext;
        struct pl_vulkan_hold_params hp{};
        hp.tex = p.plTex;
        hp.layout = VK_IMAGE_LAYOUT_GENERAL;
        hp.qf = VK_QUEUE_FAMILY_IGNORED;
        hp.semaphore = {m_PlSem, v};
        pl_vulkan_hold_ex(m_Vulkan->gpu, &hp);
        m_LastHoldVal.store(v);
    }

    m_TotalRenderTimeUs.fetch_add(LiGetMicroseconds() - renderStartUs);
    m_RenderedFrames.fetch_add(1);
}

#endif  // HAVE_PYROWAVE
