
#ifndef AR_CORE_H_
#define AR_CORE_H_

#include "ar_la.h"
#include <vector>
#include <cstring>
#include <cstdint>
#include <cmath>


namespace toyar {

struct CameraIntrinsics {
    float fx, fy, cx, cy;
};

struct GreyFrame {
    int w, h;
    const uint8_t* data;
};

class Tracker {
public:
    void init(int w, int h, const CameraIntrinsics& K) {
        _w = w; _h = h; _K = K;
        _prev.resize(w * h, 0);
        _pose = Mat4::identity();
        _havePrev = false;
    }

    bool track(const GreyFrame& f) {
        if (!_havePrev) {
            std::memcpy(_prev.data(), f.data, _prev.size());
            _havePrev = true;
            return false;
        }

        int64_t sumX = 0, sumY = 0, diffCount = 0;
        const uint8_t* cur = f.data;
        const uint8_t* prv = _prev.data();
        for (int y = 0; y < _h; ++y) {
            for (int x = 0; x < _w; ++x, ++cur, ++prv) {
                int d = int(*cur) - int(*prv);
                if (std::abs(d) > 15) {
                    sumX += x;
                    sumY += y;
                    ++diffCount;
                }
            }
        }

        if (diffCount == 0) {
            std::memcpy(_prev.data(), f.data, _prev.size());
            return false;
        }

        float cx = float(sumX) / diffCount;
        float cy = float(sumY) / diffCount;

        // Convert pixel shift to camera-space translation
        float dx = (cx - _K.cx) / _K.fx;
        float dy = (cy - _K.cy) / _K.fy;

        constexpr float depth = 0.5f;
        Vec3 t { -dx * depth, dy * depth, 0};

        // update the pose and swap the buffers for next iteration
        _pose = Mat4::translation(t) * _pose;
        std::memcpy(_prev.data(), f.data, _prev.size());
        return true;

    }

private:
    int _w = 0, _h = 0;
    CameraIntrinsics _K {0,0,0,0};
    std::vector<uint8_t> _prev;
    bool _havePrev = false;
    Mat4 _pose;
};

}

#endif
