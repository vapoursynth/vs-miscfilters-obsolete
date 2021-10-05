/*
* Copyright (c) 2016 Fredrik Mellbin & other contributors
*
* This file is part of VapourSynth's miscellaneous filters package.
*
* VapourSynth is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* VapourSynth is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with VapourSynth; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
*/

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <cfloat>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>
#include <VapourSynth4.h>
#include <VSHelper4.h>

namespace {
std::string operator""_s(const char *str, size_t len) { return{ str, len }; }
} // namespace

using namespace vsh;

///////////////////////////////////////
// Shared

template<typename T>
struct DualNodeData : public T {
private:
    const VSAPI *vsapi;
public:
    VSNode *node1 = nullptr;
    VSNode *node2 = nullptr;

    explicit DualNodeData(const VSAPI *vsapi) noexcept : T(), vsapi(vsapi) {
    }

    ~DualNodeData() {
        vsapi->freeNode(node1);
        vsapi->freeNode(node2);
    }
};

template<typename T>
static void VS_CC filterFree(void *instanceData, VSCore *core, const VSAPI *vsapi) {
    delete reinterpret_cast<T *>(instanceData);
}

static bool is8to16orFloatFormat(const VSVideoFormat &fi, bool allowVariable = false, bool allowCompat = false) {
    if (fi.colorFamily == cfUndefined && !allowVariable)
        return false;

    if ((fi.sampleType == stInteger && fi.bitsPerSample > 16) || (fi.sampleType == stFloat && fi.bitsPerSample != 32))
        return false;

    return true;
}

static inline void getPlanesArg(const VSMap *in, bool *process, const VSAPI *vsapi) {
    int m = vsapi->mapNumElements(in, "planes");

    for (int i = 0; i < 3; i++)
        process[i] = (m <= 0);

    for (int i = 0; i < m; i++) {
        int o = vsapi->mapGetIntSaturated(in, "planes", i, nullptr);

        if (o < 0 || o >= 3)
            throw std::runtime_error("plane index out of range");

        if (process[o])
            throw std::runtime_error("plane specified twice");

        process[o] = true;
    }
}

///////////////////////////////////////
// SCDetect

typedef struct {
    double threshold;
} SCDetectDataExtra;

typedef DualNodeData<SCDetectDataExtra> SCDetectData;

static const VSFrame *VS_CC scDetectGetFrame(int n, int activationReason, void *instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    SCDetectData *d = static_cast<SCDetectData *>(instanceData);

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->node1, frameCtx);
        vsapi->requestFrameFilter(std::max(n - 1, 0), d->node2, frameCtx);
        vsapi->requestFrameFilter(n, d->node2, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrame *src = vsapi->getFrameFilter(n, d->node1, frameCtx);
        const VSFrame *prevframe = vsapi->getFrameFilter(std::max(n - 1, 0), d->node2, frameCtx);
        const VSFrame *nextframe = vsapi->getFrameFilter(n, d->node2, frameCtx);

        double prevdiff = vsapi->mapGetFloat(vsapi->getFramePropertiesRO(prevframe), "SCPlaneStatsDiff", 0, nullptr);
        double nextdiff = vsapi->mapGetFloat(vsapi->getFramePropertiesRO(nextframe), "SCPlaneStatsDiff", 0, nullptr);

        VSFrame *dst = vsapi->copyFrame(src, core);
        VSMap *rwprops = vsapi->getFramePropertiesRW(dst);
        vsapi->mapSetInt(rwprops, "_SceneChangePrev", prevdiff > d->threshold, maReplace);
        vsapi->mapSetInt(rwprops, "_SceneChangeNext", nextdiff > d->threshold, maReplace);
        vsapi->freeFrame(src);
        vsapi->freeFrame(prevframe);
        vsapi->freeFrame(nextframe);

        return dst;
    }

    return nullptr;
}

static void VS_CC scDetectCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    std::unique_ptr<SCDetectData> d(new SCDetectData(vsapi));
    int err;
    d->threshold = vsapi->mapGetFloat(in, "threshold", 0, &err);
    if (err)
        d->threshold = 0.1;
    d->node1 = vsapi->mapGetNode(in, "clip", 0, nullptr);
    const VSVideoInfo *vi = vsapi->getVideoInfo(d->node1);

    try {
        if (d->threshold < 0.0 || d->threshold > 1.0)
            throw std::runtime_error("threshold must be between 0 and 1");
        if (!is8to16orFloatFormat(vi->format))
            throw std::runtime_error("clip must be constant format and of integer 8-16 bit type or 32 bit float");
        if (vi->numFrames == 1)
            throw std::runtime_error("clip must have more than one frame");

        VSPlugin *stdplugin = vsapi->getPluginByID(VSH_STD_PLUGIN_ID, core);
        VSMap *invmap = vsapi->createMap();
        VSMap *invmap2 = nullptr;
        vsapi->mapSetNode(invmap, "clip", d->node1, maAppend);
        vsapi->mapSetInt(invmap, "first", 1, maAppend);
        invmap2 = vsapi->invoke(stdplugin, "Trim", invmap);
        vsapi->clearMap(invmap);
        vsapi->mapSetNode(invmap, "clipa", d->node1, maAppend);
        vsapi->mapConsumeNode(invmap, "clipb", vsapi->mapGetNode(invmap2, "clip", 0, nullptr), maAppend);
        vsapi->mapSetData(invmap, "prop", "SCPlaneStats", -1, dtUtf8, maAppend);
        vsapi->mapSetInt(invmap, "plane", 0, maAppend);
        vsapi->freeMap(invmap2);
        invmap2 = vsapi->invoke(stdplugin, "PlaneStats", invmap);
        vsapi->freeMap(invmap);
        d->node2 = vsapi->mapGetNode(invmap2, "clip", 0, nullptr);
        vsapi->freeMap(invmap2);
    } catch (const std::runtime_error &e) {
        vsapi->mapSetError(out, ("SCDetect: "_s + e.what()).c_str());
        return;
    }

    VSFilterDependency deps[] = {{ d->node1, rpStrictSpatial }, { d->node2, rpGeneral }};
    vsapi->createVideoFilter(out, "SCDetect", vi, scDetectGetFrame, filterFree<SCDetectData>, fmParallel, deps, 2, d.release(), core);
}

///////////////////////////////////////
// AverageFrames

static void VS_CC averageFramesCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    VSPlugin *plugin = vsapi->getPluginByID(VSH_STD_PLUGIN_ID, core);
    VSMap *result = vsapi->invoke(plugin, "AverageFrames", in);
    if (vsapi->mapGetError(result)) {
        vsapi->mapSetError(out, vsapi->mapGetError(result));
    } else {
        vsapi->mapConsumeNode(out, "clip", vsapi->mapGetNode(result, "clip", 0, nullptr), maAppend);
    }
    vsapi->freeMap(result);
}

///////////////////////////////////////
// Hysteresis

struct HysteresisExtraData {
    bool process[3];
    uint16_t peak;
    size_t labelSize;
};

typedef DualNodeData<HysteresisExtraData> HysteresisData;

template<typename T>
static void process_frame_hysteresis(const VSFrame * src1, const VSFrame * src2, VSFrame * dst, const VSVideoFormat *fi, const HysteresisData * d, const VSAPI * vsapi) VS_NOEXCEPT {
    uint8_t * VS_RESTRICT label = nullptr;

    for (int plane = 0; plane < fi->numPlanes; plane++) {
        if (d->process[plane]) {
            if (!label)
                label = new uint8_t[d->labelSize]();
            const int width = vsapi->getFrameWidth(src1, plane);
            const int height = vsapi->getFrameHeight(src1, plane);
            const ptrdiff_t stride = vsapi->getStride(src1, plane) / sizeof(T);
            const T * srcp1 = reinterpret_cast<const T *>(vsapi->getReadPtr(src1, plane));
            const T * srcp2 = reinterpret_cast<const T *>(vsapi->getReadPtr(src2, plane));
            T * VS_RESTRICT dstp = reinterpret_cast<T *>(vsapi->getWritePtr(dst, plane));

            T lower, upper;
            if (std::is_integral<T>::value) {
                lower = 0;
                upper = d->peak;
            } else {
                lower = 0.f;
                upper = 1.f;
            }

            std::fill_n(dstp, stride * height, lower);

            std::vector<std::pair<int, int>> coordinates;

            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    if (!label[width * y + x] && srcp1[stride * y + x] > lower && srcp2[stride * y + x] > lower) {
                        label[width * y + x] = std::numeric_limits<uint8_t>::max();
                        dstp[stride * y + x] = upper;

                        coordinates.emplace_back(std::make_pair(x, y));

                        while (!coordinates.empty()) {
                            const auto pos = coordinates.back();
                            coordinates.pop_back();

                            for (int yy = std::max(pos.second - 1, 0); yy <= std::min(pos.second + 1, height - 1); yy++) {
                                for (int xx = std::max(pos.first - 1, 0); xx <= std::min(pos.first + 1, width - 1); xx++) {
                                    if (!label[width * yy + xx] && srcp2[stride * yy + xx] > lower) {
                                        label[width * yy + xx] = std::numeric_limits<uint8_t>::max();
                                        dstp[stride * yy + xx] = upper;

                                        coordinates.emplace_back(std::make_pair(xx, yy));
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    delete[] label;
}

static const VSFrame *VS_CC hysteresisGetFrame(int n, int activationReason, void *instanceData, void **frameData, VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    HysteresisData * d = static_cast<HysteresisData *>(instanceData);

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->node1, frameCtx);
        vsapi->requestFrameFilter(n, d->node2, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrame * src1 = vsapi->getFrameFilter(n, d->node1, frameCtx);
        const VSFrame * src2 = vsapi->getFrameFilter(n, d->node2, frameCtx);
        const VSFrame * fr[]{ d->process[0] ? nullptr : src1, d->process[1] ? nullptr : src1, d->process[2] ? nullptr : src1 };
        const int pl[]{ 0, 1, 2 };
        const VSVideoFormat *fi = vsapi->getVideoFrameFormat(src1);
        VSFrame * dst = vsapi->newVideoFrame2(fi, vsapi->getFrameWidth(src1, 0), vsapi->getFrameHeight(src1, 0), fr, pl, src1, core);

        if (fi->bytesPerSample == 1)
            process_frame_hysteresis<uint8_t>(src1, src2, dst, fi, d, vsapi);
        else if (fi->bytesPerSample == 2)
            process_frame_hysteresis<uint16_t>(src1, src2, dst, fi, d, vsapi);
        else
            process_frame_hysteresis<float>(src1, src2, dst, fi, d, vsapi);

        vsapi->freeFrame(src1);
        vsapi->freeFrame(src2);
        return dst;
    }

    return nullptr;
}

static void VS_CC hysteresisCreate(const VSMap *in, VSMap *out, void *userData, VSCore *core, const VSAPI *vsapi) {
    std::unique_ptr<HysteresisData> d(new HysteresisData(vsapi));

    d->node1 = vsapi->mapGetNode(in, "clipa", 0, nullptr);
    d->node2 = vsapi->mapGetNode(in, "clipb", 0, nullptr);
    const VSVideoInfo *vi = vsapi->getVideoInfo(d->node1);

    try {
        
        if (!isConstantVideoFormat(vi) || !is8to16orFloatFormat(vi->format))
            throw std::runtime_error("only constant format 8-16 bits integer and 32 bits float input supported");

        if (!isSameVideoInfo(vi, vsapi->getVideoInfo(d->node2)))
            throw std::runtime_error("both clips must have the same dimensions and the same format");

        getPlanesArg(in, d->process, vsapi);

        if (vi->format.sampleType == stInteger) {
            d->peak = (1 << vi->format.bitsPerSample) - 1;
        }

        d->labelSize = vi->width * vi->height;

    } catch (const std::runtime_error &e) {
        vsapi->mapSetError(out, ("Hysteresis: "_s + e.what()).c_str());
        return;
    }

    VSFilterDependency deps[] = {{d->node1, rpStrictSpatial}, {d->node2, (vi->numFrames <= vsapi->getVideoInfo(d->node2)->numFrames) ? rpStrictSpatial : rpGeneral}};
    vsapi->createVideoFilter(out, "Hysteresis", vi, hysteresisGetFrame, filterFree<HysteresisData>, fmParallel, deps, 2, d.release(), core);
}

///////////////////////////////////////
// Init

VS_EXTERNAL_API(void) VapourSynthPluginInit2(VSPlugin *plugin, const VSPLUGINAPI *vspapi) {
    vspapi->configPlugin("com.vapoursynth.misc", "misc", "Miscellaneous filters", 1, VAPOURSYNTH_API_VERSION, 0, plugin);
    vspapi->registerFunction("SCDetect", "clip:vnode;threshold:float:opt;", "clip:vnode;", scDetectCreate, 0, plugin);
    vspapi->registerFunction("AverageFrames", "clips:vnode[];weights:float[];scale:float:opt;scenechange:int:opt;planes:int[]:opt;", "clip:vnode;", averageFramesCreate, 0, plugin);
    vspapi->registerFunction("Hysteresis", "clipa:vnode;clipb:vnode;planes:int[]:opt;", "clip:vnode;", hysteresisCreate, nullptr, plugin);
}
