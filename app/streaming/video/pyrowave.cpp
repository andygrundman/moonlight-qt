/**
 * @file app/streaming/video/pyrowave.cpp
 * @brief PyroWave client decoder — Stage B4 GPU zero-copy, YUV->RGB + swapchain present via
 * libplacebo, no CPU readback. Linux decodes into exportable dmabuf planes on PyroWave's own
 * device; macOS (MoltenVK, no dmabuf) shares a single VkDevice between PyroWave and libplacebo.
 */
#ifdef HAVE_PYROWAVE

  #include "pyrowave.h"  // this decoder's header (pulls vulkan + libplacebo + SDL_vulkan)

  // PyroWave C API. vulkan.h is already included by our header; the angle-bracket form resolves to
  // the pyrowave submodule (not this same-named local header).
  #include <pyrowave.h>

  #include "streaming/session.h"

  #ifndef __APPLE__
    #include <drm_fourcc.h>
  #endif

  #ifndef IMGUI_DISABLE
    #include "imgui.h"
    #include "imgui_impl_sdl2.h"
    #include "imgui_impl_vulkan.h"
    #include "imgui/imgui_plots.h"
    #include "imgui/devui.h"
    #include "imgui/gamepadmenu.h"
    #include "streaming/stats.h"
  #endif

  #include <cstring>
  #include <vector>

#ifndef __APPLE__
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
#endif  // !__APPLE__

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
      m_FrameReady(false), m_LastFrameNumber(0),
      m_OverlayLock(0) {}

PyroWaveVideoDecoder::~PyroWaveVideoDecoder() {
    // Stop overlay updates before we tear down the libplacebo GPU that owns the overlay textures.
    // Only the real (non-testOnly) decoder registers itself, and only while a Session is active
    // (the testOnly probe runs during validateLaunch before Session::get() is set).
    if (!m_TestOnly && Session::get() != nullptr) {
        Session::get()->getOverlayManager().setOverlayRenderer(nullptr);
    }

#ifndef IMGUI_DISABLE
    // Tear down the ImGui context + Vulkan backend before the libplacebo GPU that backs it.
    if (m_ImGuiInited) {
        ImGuiPlots::instance().ImGui_deinit(this);
    }
#endif

    if (m_Decoder) {
        pyrowave_decoder_destroy(m_Decoder);  // ensures PyroWave GPU is idle
    }
#ifndef __APPLE__
    if (m_SyncObj) {
        pyrowave_sync_object_destroy(m_SyncObj);
    }
#endif
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
        pl_vulkan_sem_destroy(m_Vulkan->gpu, &m_PlSem);
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
#ifdef __APPLE__
    // Shared-device teardown: PyroWave's wrapper goes before the VkDevice it wraps, and the
    // device (created from the libplacebo instance) goes before that instance below.
    if (m_PyroDevice) {
        pyrowave_device_destroy(m_PyroDevice);
    }
    if (m_VkDev && m_DeviceWaitIdle) {
        m_DeviceWaitIdle(m_VkDev);
    }
    if (m_VkDev && m_DestroyDevice) {
        m_DestroyDevice(m_VkDev, nullptr);
    }
#endif
    if (m_VkSurface && m_DestroySurface) {
        m_DestroySurface(m_PlVkInstance->instance, m_VkSurface, nullptr);
    }
    if (m_PlVkInstance) {
        pl_vk_inst_destroy(&m_PlVkInstance);
    }
    if (m_Log) {
        pl_log_destroy(&m_Log);
    }
#ifndef __APPLE__
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
#endif
}

#ifndef __APPLE__
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
#endif  // !__APPLE__

#ifdef __APPLE__
// Shared-device planes: ordinary libplacebo textures (storage + sampled) whose VkImage is handed
// to PyroWave's compute decode. No export/import; layout transitions happen via hold/release.
bool PyroWaveVideoDecoder::createPlanes() {
    int depth = m_TenBit ? 16 : 8;
    pl_fmt fmt = pl_find_fmt(m_Vulkan->gpu, PL_FMT_UNORM, 1, depth, depth,
                             (enum pl_fmt_caps) (PL_FMT_CAP_SAMPLEABLE | PL_FMT_CAP_STORABLE));
    if (!fmt) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                     "PyroWave: no storage-capable %d-bit plane format in libplacebo", depth);
        return false;
    }
    int chromaW = m_YUV444 ? m_Width : m_Width / 2;
    int chromaH = m_YUV444 ? m_Height : m_Height / 2;
    int dims[3][2] = {{m_Width, m_Height}, {chromaW, chromaH}, {chromaW, chromaH}};
    for (int i = 0; i < 3; i++) {
        Plane& p = m_Planes[i];
        p.w = dims[i][0];
        p.h = dims[i][1];
        pl_tex_params tp = {};
        tp.w = p.w;
        tp.h = p.h;
        tp.format = fmt;
        tp.sampleable = true;
        tp.storable = true;
        p.plTex = pl_tex_create(m_Vulkan->gpu, &tp);
        if (!p.plTex) {
            return false;
        }
        p.image = pl_vulkan_unwrap(m_Vulkan->gpu, p.plTex, nullptr, nullptr);
        if (!p.image) {
            return false;
        }
    }
    return true;
}
#else
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
#endif  // __APPLE__

#ifdef __APPLE__
// Create the single VkDevice shared by PyroWave and libplacebo. All entry points come from the
// libplacebo instance's get_proc_addr (SDL-loaded MoltenVK); both consumers serialize their queue
// submissions through m_QueueMutex.
bool PyroWaveVideoDecoder::createSharedDevice() {
    PFN_vkGetInstanceProcAddr gipa = m_PlVkInstance->get_proc_addr;
    VkInstance inst = m_PlVkInstance->instance;

    // Granite requires a Vulkan 1.3 instance/device (subgroup size control etc.).
    if (m_PlVkInstance->api_version < VK_API_VERSION_1_3) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                     "PyroWave: Vulkan 1.3 instance required, got 0x%x (MoltenVK too old?)",
                     m_PlVkInstance->api_version);
        return false;
    }

  #define PW_INST_PFN(name) \
    auto pfn_##name = (PFN_##name) gipa(inst, #name); \
    if (!pfn_##name) return false
    PW_INST_PFN(vkEnumeratePhysicalDevices);
    PW_INST_PFN(vkGetPhysicalDeviceProperties);
    PW_INST_PFN(vkGetPhysicalDeviceQueueFamilyProperties);
    PW_INST_PFN(vkGetPhysicalDeviceSurfaceSupportKHR);
    PW_INST_PFN(vkGetPhysicalDeviceFeatures2);
    PW_INST_PFN(vkEnumerateDeviceExtensionProperties);
    PW_INST_PFN(vkCreateDevice);
    PW_INST_PFN(vkGetDeviceProcAddr);
  #undef PW_INST_PFN

    // Pick the first physical device with a graphics+compute queue family that can present to our
    // surface (MoltenVK exposes one all-purpose family per GPU).
    uint32_t devCount = 0;
    pfn_vkEnumeratePhysicalDevices(inst, &devCount, nullptr);
    std::vector<VkPhysicalDevice> devs(devCount);
    pfn_vkEnumeratePhysicalDevices(inst, &devCount, devs.data());
    m_VkPhys = VK_NULL_HANDLE;
    for (VkPhysicalDevice pd : devs) {
        uint32_t qn = 0;
        pfn_vkGetPhysicalDeviceQueueFamilyProperties(pd, &qn, nullptr);
        std::vector<VkQueueFamilyProperties> qf(qn);
        pfn_vkGetPhysicalDeviceQueueFamilyProperties(pd, &qn, qf.data());
        for (uint32_t i = 0; i < qn; i++) {
            constexpr VkQueueFlags needed = VK_QUEUE_GRAPHICS_BIT | VK_QUEUE_COMPUTE_BIT;
            VkBool32 present = VK_FALSE;
            if ((qf[i].queueFlags & needed) == needed &&
                pfn_vkGetPhysicalDeviceSurfaceSupportKHR(pd, i, m_VkSurface, &present) == VK_SUCCESS &&
                present) {
                m_VkPhys = pd;
                m_VkFamily = i;
                break;
            }
        }
        if (m_VkPhys) {
            break;
        }
    }
    if (!m_VkPhys) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "PyroWave: no presentable graphics+compute Vulkan device");
        return false;
    }
    VkPhysicalDeviceProperties props;
    pfn_vkGetPhysicalDeviceProperties(m_VkPhys, &props);
    if (props.apiVersion < VK_API_VERSION_1_3) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "PyroWave: GPU only supports Vulkan 0x%x, need 1.3", props.apiVersion);
        return false;
    }

    // Device extensions: swapchain, portability subset (mandatory to enable when advertised),
    // plus whatever of libplacebo's recommended extensions the device offers.
    uint32_t extCount = 0;
    pfn_vkEnumerateDeviceExtensionProperties(m_VkPhys, nullptr, &extCount, nullptr);
    std::vector<VkExtensionProperties> extProps(extCount);
    pfn_vkEnumerateDeviceExtensionProperties(m_VkPhys, nullptr, &extCount, extProps.data());
    auto hasExt = [&](const char* name) {
        for (const auto& e : extProps) {
            if (strcmp(e.extensionName, name) == 0) return true;
        }
        return false;
    };
    if (!hasExt(VK_KHR_SWAPCHAIN_EXTENSION_NAME)) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "PyroWave: device lacks VK_KHR_swapchain");
        return false;
    }
    m_DevExtNames.clear();
    m_DevExtNames.push_back(VK_KHR_SWAPCHAIN_EXTENSION_NAME);
    bool hasPortability = hasExt(VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME);
    if (hasPortability) {
        m_DevExtNames.push_back(VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME);
    }
    for (int i = 0; i < pl_vulkan_num_recommended_extensions; i++) {
        const char* name = pl_vulkan_recommended_extensions[i];
        if (hasExt(name) && strcmp(name, VK_KHR_SWAPCHAIN_EXTENSION_NAME) != 0 &&
            strcmp(name, VK_KHR_PORTABILITY_SUBSET_EXTENSION_NAME) != 0) {
            m_DevExtNames.push_back(name);
        }
    }

    // Enable every core feature the device supports; this covers PyroWave's decode requirements
    // (shaderInt16, 8-bit storage, subgroup size control) and libplacebo's required features in
    // one go. robustBufferAccess is left off as pure overhead.
    m_Feat13 = {VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES};
    m_Feat12 = {VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES, &m_Feat13};
    m_Feat11 = {VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_1_FEATURES, &m_Feat12};
    m_FeatPortability = {VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PORTABILITY_SUBSET_FEATURES_KHR, &m_Feat11};
    m_Feat2 = {VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2,
               hasPortability ? (void*) &m_FeatPortability : (void*) &m_Feat11};
    pfn_vkGetPhysicalDeviceFeatures2(m_VkPhys, &m_Feat2);
    m_Feat2.features.robustBufferAccess = VK_FALSE;
    if (!m_Feat2.features.shaderInt16 || !m_Feat12.storageBuffer8BitAccess ||
        !m_Feat12.timelineSemaphore || !m_Feat13.subgroupSizeControl) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION,
                     "PyroWave: GPU lacks required decode features (int16=%d 8bit=%d timeline=%d subgroupCtl=%d)",
                     m_Feat2.features.shaderInt16, m_Feat12.storageBuffer8BitAccess,
                     m_Feat12.timelineSemaphore, m_Feat13.subgroupSizeControl);
        return false;
    }

    m_QueueCreateInfo = {VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
    m_QueueCreateInfo.queueFamilyIndex = m_VkFamily;
    m_QueueCreateInfo.queueCount = 1;
    m_QueueCreateInfo.pQueuePriorities = &m_QueuePriority;

    m_DevCreateInfo = {VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
    m_DevCreateInfo.pNext = &m_Feat2;
    m_DevCreateInfo.queueCreateInfoCount = 1;
    m_DevCreateInfo.pQueueCreateInfos = &m_QueueCreateInfo;
    m_DevCreateInfo.enabledExtensionCount = (uint32_t) m_DevExtNames.size();
    m_DevCreateInfo.ppEnabledExtensionNames = m_DevExtNames.data();
    if (pfn_vkCreateDevice(m_VkPhys, &m_DevCreateInfo, nullptr, &m_VkDev) != VK_SUCCESS) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "PyroWave: vkCreateDevice failed");
        return false;
    }

    auto pfn_vkGetDeviceQueue = (PFN_vkGetDeviceQueue) pfn_vkGetDeviceProcAddr(m_VkDev, "vkGetDeviceQueue");
    m_DestroyDevice = (PFN_vkDestroyDevice) pfn_vkGetDeviceProcAddr(m_VkDev, "vkDestroyDevice");
    m_DeviceWaitIdle = (PFN_vkDeviceWaitIdle) pfn_vkGetDeviceProcAddr(m_VkDev, "vkDeviceWaitIdle");
    if (!pfn_vkGetDeviceQueue || !m_DestroyDevice || !m_DeviceWaitIdle) {
        return false;
    }
    pfn_vkGetDeviceQueue(m_VkDev, m_VkFamily, 0, &m_VkQueue);

    // Wrap the instance+device for PyroWave. Granite reads extensions/features straight out of
    // these create infos (see MyInstanceFactory/MyDeviceFactory in pyrowave_c.cpp), so they and
    // everything they point to are class members that outlive m_PyroDevice.
    m_AppInfo = {VK_STRUCTURE_TYPE_APPLICATION_INFO};
    m_AppInfo.apiVersion = m_PlVkInstance->api_version;
    m_InstCreateInfo = {VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    m_InstCreateInfo.pApplicationInfo = &m_AppInfo;
    m_InstCreateInfo.enabledExtensionCount = (uint32_t) m_PlVkInstance->num_extensions;
    m_InstCreateInfo.ppEnabledExtensionNames = m_PlVkInstance->extensions;
    m_QueueInfoDesc.queue = m_VkQueue;
    m_QueueInfoDesc.familyIndex = m_VkFamily;
    m_QueueInfoDesc.index = 0;

    pyrowave_device_create_info di{};
    di.GetInstanceProcAddr = gipa;
    di.instance = inst;
    di.physical_device = m_VkPhys;
    di.device = m_VkDev;
    di.instance_create_info = &m_InstCreateInfo;
    di.device_create_info = &m_DevCreateInfo;
    di.queue_info = &m_QueueInfoDesc;
    di.queue_info_count = 1;
    di.queue_lock_callback = [](void* ud) { static_cast<PyroWaveVideoDecoder*>(ud)->m_QueueMutex.lock(); };
    di.queue_unlock_callback = [](void* ud) { static_cast<PyroWaveVideoDecoder*>(ud)->m_QueueMutex.unlock(); };
    di.userdata = this;
    if (pyrowave_create_device(&di, &m_PyroDevice) != PYROWAVE_SUCCESS) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "PyroWave: shared-device wrap failed");
        return false;
    }

    // libplacebo imports the same device; queue access on both sides funnels through m_QueueMutex.
    struct pl_vulkan_import_params ip = {};
    ip.instance = inst;
    ip.get_proc_addr = gipa;
    ip.phys_device = m_VkPhys;
    ip.device = m_VkDev;
    ip.extensions = m_DevExtNames.data();
    ip.num_extensions = (int) m_DevExtNames.size();
    ip.queue_graphics = {m_VkFamily, 1, 0};
    ip.features = &m_Feat2;
    ip.lock_queue = [](void* ctx, uint32_t, uint32_t) { static_cast<PyroWaveVideoDecoder*>(ctx)->m_QueueMutex.lock(); };
    ip.unlock_queue = [](void* ctx, uint32_t, uint32_t) { static_cast<PyroWaveVideoDecoder*>(ctx)->m_QueueMutex.unlock(); };
    ip.queue_ctx = this;
    m_Vulkan = pl_vulkan_import(m_Log, &ip);
    if (!m_Vulkan) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "PyroWave: pl_vulkan_import failed");
        return false;
    }
    return true;
}
#endif  // __APPLE__

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

#ifdef __APPLE__
    // Shared-device mode: create the VkDevice ourselves so PyroWave can wrap it too, then
    // import it into libplacebo (also creates m_PyroDevice).
    if (!createSharedDevice()) {
        return false;
    }
#else
    pl_vulkan_params vp = pl_vulkan_default_params;
    vp.instance = m_PlVkInstance->instance;
    vp.get_proc_addr = m_PlVkInstance->get_proc_addr;
    vp.surface = m_VkSurface;
    m_Vulkan = pl_vulkan_create(m_Log, &vp);
    if (!m_Vulkan) {
        return false;
    }
#endif

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

#ifndef __APPLE__
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
#endif  // !__APPLE__

bool PyroWaveVideoDecoder::createSharedTimeline() {
#ifdef __APPLE__
    // Same VkDevice on both sides: PyroWave uses the libplacebo-created timeline semaphore
    // directly, no export/import.
    struct pl_vulkan_sem_params sp{};
    sp.type = VK_SEMAPHORE_TYPE_TIMELINE;
    sp.initial_value = 0;
    m_PlSem = pl_vulkan_sem_create(m_Vulkan->gpu, &sp);
    if (!m_PlSem) {
        return false;
    }
    m_PwSem = m_PlSem;
#else
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
#endif

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

#ifdef __APPLE__
    // Shared-device mode: the pyrowave device is created inside createLibplacebo (it wraps the
    // VkDevice we make there). The testOnly probe has no window/surface, so it lets PyroWave
    // bring up its own headless device via its internal loader (finds the bundled MoltenVK).
    if (m_TestOnly) {
        if (pyrowave_create_default_device(&m_PyroDevice) != PYROWAVE_SUCCESS) {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "PyroWave: device init failed");
            return false;
        }
    } else if (!createLibplacebo(params)) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "PyroWave: GPU present pipeline init failed");
        return false;
    }
#else
    if (!createPyroDevice()) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "PyroWave: device init failed");
        return false;
    }
#endif

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

#ifdef __APPLE__
    if (!createPlanes() || !createSharedTimeline()) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "PyroWave: GPU present pipeline init failed");
        return false;
    }
#else
    // libplacebo first: plane images must be created with a DRM modifier libplacebo can import
    // (createPlanes intersects the storage-capable modifiers with pl_fmt's import list — on some
    // GPUs, e.g. Steam Deck's Van Gogh, radv otherwise picks a storage modifier pl can't sample).
    if (!createLibplacebo(params) || !createPlanes() || !importPlanes() || !createSharedTimeline()) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "PyroWave: GPU present pipeline init failed");
        return false;
    }
#endif
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

#ifndef IMGUI_DISABLE

// (Re)create the offscreen ImGui render target at the current drawable size. The render pass is
// created once on first use (the pl_fmt, and thus VkFormat, never changes afterwards).
bool PyroWaveVideoDecoder::ensureImGuiTarget(int w, int h) {
    if (m_ImGuiTex && m_ImGuiTex->params.w == w && m_ImGuiTex->params.h == h) {
        return true;
    }
    VkDevice dev = m_Vulkan->device;

    // Drop any previous target. Our last draw into it is fenced; libplacebo defers destruction
    // of in-flight textures internally.
    if (m_ImGuiFenceArmed) {
        m_ImGuiFn.WaitForFences(dev, 1, &m_ImGuiFence, VK_TRUE, UINT64_MAX);
    }
    if (m_ImGuiFb) {
        m_ImGuiFn.DestroyFramebuffer(dev, m_ImGuiFb, nullptr);
        m_ImGuiFb = VK_NULL_HANDLE;
    }
    if (m_ImGuiView) {
        m_ImGuiFn.DestroyImageView(dev, m_ImGuiView, nullptr);
        m_ImGuiView = VK_NULL_HANDLE;
    }
    if (m_ImGuiTex) {
        pl_tex_destroy(m_Vulkan->gpu, &m_ImGuiTex);
    }

    pl_fmt fmt = pl_find_fmt(m_Vulkan->gpu, PL_FMT_UNORM, 4, 8, 8,
                             (enum pl_fmt_caps) (PL_FMT_CAP_SAMPLEABLE | PL_FMT_CAP_RENDERABLE));
    if (!fmt) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "PyroWave: no renderable RGBA8 format for ImGui");
        return false;
    }
    pl_tex_params tp = {};
    tp.w = w;
    tp.h = h;
    tp.format = fmt;
    tp.sampleable = true;
    tp.renderable = true;
    m_ImGuiTex = pl_tex_create(m_Vulkan->gpu, &tp);
    if (!m_ImGuiTex) {
        return false;
    }
    VkFormat vkFmt = VK_FORMAT_UNDEFINED;
    VkImage img = pl_vulkan_unwrap(m_Vulkan->gpu, m_ImGuiTex, &vkFmt, nullptr);
    if (!img) {
        return false;
    }

    if (!m_ImGuiRp) {
        VkAttachmentDescription att{};
        att.format = vkFmt;
        att.samples = VK_SAMPLE_COUNT_1_BIT;
        att.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;  // clear to transparent every frame
        att.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        att.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        att.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        att.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        att.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;  // must match release_ex below
        VkAttachmentReference ref{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
        VkSubpassDescription sub{};
        sub.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        sub.colorAttachmentCount = 1;
        sub.pColorAttachments = &ref;
        VkSubpassDependency dep{};
        dep.srcSubpass = VK_SUBPASS_EXTERNAL;
        dep.dstSubpass = 0;
        dep.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dep.srcAccessMask = 0;
        dep.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        dep.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        VkRenderPassCreateInfo rci{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
        rci.attachmentCount = 1;
        rci.pAttachments = &att;
        rci.subpassCount = 1;
        rci.pSubpasses = &sub;
        rci.dependencyCount = 1;
        rci.pDependencies = &dep;
        if (m_ImGuiFn.CreateRenderPass(dev, &rci, nullptr, &m_ImGuiRp) != VK_SUCCESS) {
            return false;
        }
    }

    VkImageViewCreateInfo vci{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    vci.image = img;
    vci.viewType = VK_IMAGE_VIEW_TYPE_2D;
    vci.format = vkFmt;
    vci.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    if (m_ImGuiFn.CreateImageView(dev, &vci, nullptr, &m_ImGuiView) != VK_SUCCESS) {
        return false;
    }
    VkFramebufferCreateInfo fci{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
    fci.renderPass = m_ImGuiRp;
    fci.attachmentCount = 1;
    fci.pAttachments = &m_ImGuiView;
    fci.width = (uint32_t) w;
    fci.height = (uint32_t) h;
    fci.layers = 1;
    return m_ImGuiFn.CreateFramebuffer(dev, &fci, nullptr, &m_ImGuiFb) == VK_SUCCESS;
}

void PyroWaveVideoDecoder::ImGui_initBackend() {
    PFN_vkGetInstanceProcAddr gipa = m_PlVkInstance->get_proc_addr;
    VkInstance inst = m_PlVkInstance->instance;
    VkDevice dev = m_Vulkan->device;

  #define PW_IMGUI_PFN(name) \
    m_ImGuiFn.name = (PFN_vk##name) gipa(inst, "vk" #name); \
    if (!m_ImGuiFn.name) return
    PW_IMGUI_PFN(GetDeviceQueue);
    PW_IMGUI_PFN(CreateCommandPool);
    PW_IMGUI_PFN(DestroyCommandPool);
    PW_IMGUI_PFN(AllocateCommandBuffers);
    PW_IMGUI_PFN(CreateFence);
    PW_IMGUI_PFN(DestroyFence);
    PW_IMGUI_PFN(WaitForFences);
    PW_IMGUI_PFN(ResetFences);
    PW_IMGUI_PFN(CreateRenderPass);
    PW_IMGUI_PFN(DestroyRenderPass);
    PW_IMGUI_PFN(CreateImageView);
    PW_IMGUI_PFN(DestroyImageView);
    PW_IMGUI_PFN(CreateFramebuffer);
    PW_IMGUI_PFN(DestroyFramebuffer);
    PW_IMGUI_PFN(BeginCommandBuffer);
    PW_IMGUI_PFN(EndCommandBuffer);
    PW_IMGUI_PFN(CmdBeginRenderPass);
    PW_IMGUI_PFN(CmdEndRenderPass);
    PW_IMGUI_PFN(QueueSubmit);
  #undef PW_IMGUI_PFN

    m_ImGuiFamily = m_Vulkan->queue_graphics.index;
    m_ImGuiFn.GetDeviceQueue(dev, m_ImGuiFamily, 0, &m_ImGuiQueue);

    VkCommandPoolCreateInfo pci{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    pci.queueFamilyIndex = m_ImGuiFamily;
    pci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    if (m_ImGuiFn.CreateCommandPool(dev, &pci, nullptr, &m_ImGuiPool) != VK_SUCCESS) {
        return;
    }
    VkCommandBufferAllocateInfo cai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    cai.commandPool = m_ImGuiPool;
    cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cai.commandBufferCount = 1;
    if (m_ImGuiFn.AllocateCommandBuffers(dev, &cai, &m_ImGuiCmd) != VK_SUCCESS) {
        return;
    }
    VkFenceCreateInfo fci{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
    if (m_ImGuiFn.CreateFence(dev, &fci, nullptr, &m_ImGuiFence) != VK_SUCCESS) {
        return;
    }

    int dw = 0, dh = 0;
    SDL_Vulkan_GetDrawableSize(m_Window, &dw, &dh);
    if (!ensureImGuiTarget(SDL_max(dw, 1), SDL_max(dh, 1))) {
        return;
    }

    struct LoaderCtx {
        PFN_vkGetInstanceProcAddr gipa;
        VkInstance inst;
    } lctx{gipa, inst};
    if (!ImGui_ImplVulkan_LoadFunctions(m_Vulkan->api_version,
            [](const char* name, void* ud) {
                LoaderCtx* c = (LoaderCtx*) ud;
                return c->gipa(c->inst, name);
            }, &lctx)) {
        return;
    }

    ImGui_ImplVulkan_InitInfo ii = {};
    ii.ApiVersion = m_Vulkan->api_version;
    ii.Instance = inst;
    ii.PhysicalDevice = m_Vulkan->phys_device;
    ii.Device = dev;
    ii.QueueFamily = m_ImGuiFamily;
    ii.Queue = m_ImGuiQueue;
    ii.DescriptorPoolSize = 64;  // backend creates its own pool
    ii.MinImageCount = 2;
    ii.ImageCount = 2;
    ii.PipelineInfoMain.RenderPass = m_ImGuiRp;
    ii.PipelineInfoMain.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
    if (!ImGui_ImplVulkan_Init(&ii)) {
        return;
    }
    ImGui_ImplSDL2_InitForVulkan(m_Window);
    m_ImGuiInited = true;
    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "PyroWave: ImGui Vulkan backend initialized");
}

void PyroWaveVideoDecoder::ImGui_deinitBackend() {
    if (!m_Vulkan) {
        return;
    }
    VkDevice dev = m_Vulkan->device;

    // Drain libplacebo's use of the overlay texture and our own fenced draw before destroying
    // anything ImGui's pipeline or our render pass reference.
    pl_gpu_finish(m_Vulkan->gpu);
    if (m_ImGuiFenceArmed) {
        m_ImGuiFn.WaitForFences(dev, 1, &m_ImGuiFence, VK_TRUE, UINT64_MAX);
        m_ImGuiFenceArmed = false;
    }

    if (ImGui::GetIO().BackendRendererUserData) {
        ImGui_ImplVulkan_Shutdown();
    }
    if (ImGui::GetIO().BackendPlatformUserData) {
        ImGui_ImplSDL2_Shutdown();
    }

    if (m_ImGuiFb) {
        m_ImGuiFn.DestroyFramebuffer(dev, m_ImGuiFb, nullptr);
        m_ImGuiFb = VK_NULL_HANDLE;
    }
    if (m_ImGuiView) {
        m_ImGuiFn.DestroyImageView(dev, m_ImGuiView, nullptr);
        m_ImGuiView = VK_NULL_HANDLE;
    }
    if (m_ImGuiTex) {
        pl_tex_destroy(m_Vulkan->gpu, &m_ImGuiTex);
    }
    if (m_ImGuiRp) {
        m_ImGuiFn.DestroyRenderPass(dev, m_ImGuiRp, nullptr);
        m_ImGuiRp = VK_NULL_HANDLE;
    }
    if (m_ImGuiFence) {
        m_ImGuiFn.DestroyFence(dev, m_ImGuiFence, nullptr);
        m_ImGuiFence = VK_NULL_HANDLE;
    }
    if (m_ImGuiPool) {
        m_ImGuiFn.DestroyCommandPool(dev, m_ImGuiPool, nullptr);  // frees m_ImGuiCmd too
        m_ImGuiPool = VK_NULL_HANDLE;
        m_ImGuiCmd = VK_NULL_HANDLE;
    }
    m_ImGuiInited = false;
}

// Build this frame's dev UI and render it into m_ImGuiTex. Returns true (and fills *overlay)
// when there is something to composite. The hold/release pair keeps ownership of the texture
// balanced with libplacebo: hold's semaphore fires when pl's last sampling finished (WAR), our
// submit waits it and signals a new value that release hands to pl to wait on (RAW).
bool PyroWaveVideoDecoder::renderImGuiOverlay(int dw, int dh, pl_overlay* overlay, pl_overlay_part* part) {
    if (!m_ImGuiInited || !ensureImGuiTarget(dw, dh)) {
        return false;
    }

    ImGui_ImplVulkan_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    // The overlay is declared sRGB; libplacebo adapts it to the swapchain colorspace (incl. HDR).
    DevUIColors.InitColors(OUTPUT_IS_SDR);
    Stats::instance().RenderGraphs();
    DevUISettings::instance().Render();
    GamepadMenu::instance().Render();

    ImGui::Render();
    ImDrawData* drawData = ImGui::GetDrawData();
    if (!drawData || drawData->CmdListsCount == 0) {
        return false;  // no visible UI this frame; skip the GPU work entirely
    }

    VkDevice dev = m_Vulkan->device;

    uint64_t waitVal = ++m_TlNext;
    struct pl_vulkan_hold_params hp{};
    hp.tex = m_ImGuiTex;
    hp.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    hp.qf = VK_QUEUE_FAMILY_IGNORED;
    hp.semaphore = {m_PlSem, waitVal};
    if (!pl_vulkan_hold_ex(m_Vulkan->gpu, &hp)) {
        return false;
    }

    // Single command buffer in flight; recycle once the previous draw retired.
    if (m_ImGuiFenceArmed) {
        m_ImGuiFn.WaitForFences(dev, 1, &m_ImGuiFence, VK_TRUE, UINT64_MAX);
        m_ImGuiFenceArmed = false;
    }
    m_ImGuiFn.ResetFences(dev, 1, &m_ImGuiFence);

    VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    m_ImGuiFn.BeginCommandBuffer(m_ImGuiCmd, &bi);
    VkClearValue clear{};  // transparent black
    VkRenderPassBeginInfo rbi{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
    rbi.renderPass = m_ImGuiRp;
    rbi.framebuffer = m_ImGuiFb;
    rbi.renderArea = {{0, 0}, {(uint32_t) dw, (uint32_t) dh}};
    rbi.clearValueCount = 1;
    rbi.pClearValues = &clear;
    m_ImGuiFn.CmdBeginRenderPass(m_ImGuiCmd, &rbi, VK_SUBPASS_CONTENTS_INLINE);
    ImGui_ImplVulkan_RenderDrawData(drawData, m_ImGuiCmd);
    m_ImGuiFn.CmdEndRenderPass(m_ImGuiCmd);
    m_ImGuiFn.EndCommandBuffer(m_ImGuiCmd);

    uint64_t sigVal = ++m_TlNext;
    VkTimelineSemaphoreSubmitInfo tsi{VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO};
    tsi.waitSemaphoreValueCount = 1;
    tsi.pWaitSemaphoreValues = &waitVal;
    tsi.signalSemaphoreValueCount = 1;
    tsi.pSignalSemaphoreValues = &sigVal;
    VkPipelineStageFlags waitStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    si.pNext = &tsi;
    si.waitSemaphoreCount = 1;
    si.pWaitSemaphores = &m_PlSem;
    si.pWaitDstStageMask = &waitStage;
    si.commandBufferCount = 1;
    si.pCommandBuffers = &m_ImGuiCmd;
    si.signalSemaphoreCount = 1;
    si.pSignalSemaphores = &m_PlSem;
    m_Vulkan->lock_queue(m_Vulkan, m_ImGuiFamily, 0);
    VkResult res = m_ImGuiFn.QueueSubmit(m_ImGuiQueue, 1, &si, m_ImGuiFence);
    m_Vulkan->unlock_queue(m_Vulkan, m_ImGuiFamily, 0);

    struct pl_vulkan_release_params rp{};
    rp.tex = m_ImGuiTex;
    rp.qf = VK_QUEUE_FAMILY_IGNORED;
    if (res != VK_SUCCESS) {
        // Nothing was submitted: the image is still as hold left it. Give it straight back.
        rp.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        pl_vulkan_release_ex(m_Vulkan->gpu, &rp);
        return false;
    }
    m_ImGuiFenceArmed = true;
    rp.layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;  // render pass finalLayout
    rp.semaphore = {m_PlSem, sigVal};
    pl_vulkan_release_ex(m_Vulkan->gpu, &rp);

    *part = {};
    part->src = {0, 0, (float) dw, (float) dh};
    part->dst = {0, 0, (float) dw, (float) dh};
    *overlay = {};
    overlay->tex = m_ImGuiTex;
    overlay->mode = PL_OVERLAY_NORMAL;
    overlay->coords = PL_OVERLAY_COORDS_DST_FRAME;
    overlay->repr = pl_color_repr_rgb;
    overlay->repr.alpha = PL_ALPHA_INDEPENDENT;  // ImGui renders straight (non-premultiplied) alpha
    overlay->color = pl_color_space_srgb;
    overlay->parts = part;
    overlay->num_parts = 1;
    return true;
}

#endif  // !IMGUI_DISABLE

bool PyroWaveVideoDecoder::isHardwareAccelerated() { return true; }
bool PyroWaveVideoDecoder::isAlwaysFullScreen() { return false; }
bool PyroWaveVideoDecoder::isHdrSupported() { return true; }
int PyroWaveVideoDecoder::getDecoderCapabilities() { return 0; }
int PyroWaveVideoDecoder::getDecoderColorspace() { return COLORSPACE_REC_601; }
int PyroWaveVideoDecoder::getDecoderColorRange() { return COLOR_RANGE_LIMITED; }
QSize PyroWaveVideoDecoder::getDecoderMaxResolution() { return QSize(0, 0); }
void PyroWaveVideoDecoder::setHdrMode(bool enabled) {
    // Consumed on the render thread: switches the pl_frame colorspace + swapchain hint.
    m_HdrEnabled.store(enabled);
    SDL_LogInfo(SDL_LOG_CATEGORY_APPLICATION, "PyroWave: HDR mode %s", enabled ? "enabled" : "disabled");
}
bool PyroWaveVideoDecoder::notifyWindowChanged(PWINDOW_STATE_CHANGE_INFO) { return false; }

int PyroWaveVideoDecoder::submitDecodeUnit(PDECODE_UNIT du) {
    // Detect breaks in the frame sequence indicating dropped packets
	uint32_t droppedFramesNetwork = 0;

    // Per-frame performance stats + overlay text (mirrors FFmpegVideoDecoder).
    if (!m_LastFrameNumber) {
        m_LastFrameNumber = du->frameNumber;
    } else {
        // Any frame number greater than m_LastFrameNumber + 1 represents a dropped frame
        if (m_LastFrameNumber > 0 && du->frameNumber > (m_LastFrameNumber + 1)) {
		    droppedFramesNetwork = du->frameNumber - (m_LastFrameNumber + 1);
        }
        m_LastFrameNumber = du->frameNumber;
    }

    Stats::instance().SubmitVideoBytesAndReassemblyTime(du, droppedFramesNetwork);

    // Flip stats windows roughly every second
    if (Stats::instance().ShouldUpdateDisplay(
        Session::get()->getOverlayManager().isOverlayEnabled(Overlay::OverlayDebug),
        Session::get()->getOverlayManager().getOverlayText(Overlay::OverlayDebug),
        Session::get()->getOverlayManager().getOverlayMaxTextLength()))
    {
        Session::get()->getOverlayManager().setOverlayTextUpdated(Overlay::OverlayDebug);
    }

    // Host frames the PyroWave packets as [u32 count]{[u32 size][bytes]}*; reassemble + re-push.
    std::vector<uint8_t> frame;
    frame.reserve((size_t) du->fullLength);
    for (PLENTRY e = du->bufferList; e; e = e->next) {
        const uint8_t* d = reinterpret_cast<const uint8_t*>(e->data);
        frame.insert(frame.end(), d, d + e->length);
    }
    size_t pos = 0;
    auto u32 = [&](uint32_t& v) -> bool {
        if (pos + 4 > frame.size()) return false;
        v = (uint32_t) frame[pos] | ((uint32_t) frame[pos+1] << 8) | ((uint32_t) frame[pos+2] << 16) | ((uint32_t) frame[pos+3] << 24);
        pos += 4;
        return true;
    };
    uint32_t count = 0;
    if (!u32(count)) return DR_OK;
    for (uint32_t i = 0; i < count; i++) {
        uint32_t sz = 0;
        if (!u32(sz) || pos + sz > frame.size()) break;
        pyrowave_decoder_push_packet(m_Decoder, frame.data() + pos, sz);
        pos += sz;
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

    Stats::instance().SubmitDecodeTimeUs(LiGetMicroseconds() - du->enqueueTimeUs);

    SDL_Event event;
    SDL_zero(event);
    event.type = SDL_USEREVENT;
    event.user.code = SDL_CODE_FRAME_READY;
    SDL_PushEvent(&event);
    return DR_OK;
}

void PyroWaveVideoDecoder::renderFrameOnMainThread() {
    {
        std::lock_guard<std::mutex> lock(m_FrameLock);
        if (!m_FrameReady) {
            return;
        }
        m_FrameReady = false;
    }

#ifndef IMGUI_DISABLE
    // Lazy ImGui bring-up on the render (main) thread, mirroring FramePacer's renderThread init
    // for the FFmpeg renderers. ImGui_init creates the context and calls ImGui_initBackend().
    if (!m_ImGuiInited && !m_ImGuiFailed) {
        ImGuiPlots::instance().ImGui_init(this);
        if (!m_ImGuiInited) {
            SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "PyroWave: ImGui Vulkan backend init failed");
            ImGuiPlots::instance().ImGui_deinit(this);
            m_ImGuiFailed = true;
        }
    }
#endif

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

#ifndef IMGUI_DISABLE
            // Dev UI (graphs/tools) rendered via imgui_impl_vulkan into its own texture, then
            // composited by libplacebo along with the text overlays above.
            pl_overlay imguiOverlay;
            pl_overlay_part imguiPart;
            if (m_ImGuiInited && renderImGuiOverlay(dw, dh, &imguiOverlay, &imguiPart)) {
                overlays.push_back(imguiOverlay);
            }
#endif

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
}

#endif  // HAVE_PYROWAVE
