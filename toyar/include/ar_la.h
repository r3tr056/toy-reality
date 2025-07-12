#ifndef AR_MATH_H_
#define AR_MATH_H_

#include <array>
#include <cmath>
#include <cstring>

/* ───────────────── Platform / visibility macros ───────────────── */

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#define AR_API EMSCRIPTEN_KEEPALIVE           // exported to JS / other Wasm
#else
#define AR_API                                // no decoration on native
#endif

#if defined(__clang__) || defined(__GNUC__)
#define AR_FORCE_INLINE inline __attribute__((always_inline))
#define AR_ALIGN16    __attribute__((aligned(16)))
#else
#define AR_FORCE_INLINE inline
#define AR_ALIGN16    __declspec(align(16))
#endif

/* Mathematical constant as an object-like macro (see [2][3]) */
#ifndef AR_PI
#define AR_PI 3.14159265358979323846f
#endif

namespace toyar {

struct Vec3 {
    float x, y, z;

    constexpr Vec3() : x(0), y(0), z(0) {}
    constexpr Vec3(float X, float Y, float Z) : x(X), y(Y), z(Z) {}

    AR_FORCE_INLINE Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    AR_FORCE_INLINE Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    AR_FORCE_INLINE Vec3 operator*(float s)       const { return {x*s,   y*s,   z*s};   }
    AR_FORCE_INLINE float dot   (const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    AR_FORCE_INLINE Vec3  cross (const Vec3& o) const {
        return { y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x };
    }
    AR_FORCE_INLINE float norm() const { return std::sqrt(dot(*this)); }
    AR_FORCE_INLINE Vec3 normalized() const { float n = norm(); return n ? (*this)*(1.0f/n): *this; }
};

struct AR_ALIGN16 Mat4 {
    std::array<float, 16> m;

    constexpr Mat4( float m00, float m01, float m02, float m03,
                   float m10, float m11, float m12, float m13,
                   float m20, float m21, float m22, float m23,
                   float m30, float m31, float m32, float m33 )
        : m{ m00,m01,m02,m03,
            m10,m11,m12,m13,
            m20,m21,m22,m23,
            m30,m31,m32,m33 } {}

    constexpr explicit Mat4(const std::array<float,16>& arr)
        : m(arr) {}

    explicit Mat4(const float* arr) { std::memcpy(m.data(), arr, 16*sizeof(float)); }

    constexpr Mat4() : m{1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1} {}
    AR_FORCE_INLINE const float* data() const { return m.data(); }
    float* data() { return m.data(); }

    AR_FORCE_INLINE Mat4 operator*(const Mat4& b) const {
        Mat4 r{}; r.m.fill(0);
        for (int c=0;c<4;++c)
            for (int r_=0;r_<4;++r_)
                for (int k=0;k<4;++k)
                    r.m[c*4+r_] += m[k*4+r_]*b.m[c*4+k];

        return r;
    }

    static AR_FORCE_INLINE Mat4 identity() { return Mat4(); }

    static AR_FORCE_INLINE Mat4 translation(const Vec3& t) {
        Mat4 r = identity();
        r.m[12] = t.x; r.m[13] = t.y; r.m[14] = t.z;
        return r;
    }

    static AR_FORCE_INLINE Mat4 scale(const Vec3& s) {
        Mat4 r{};
        r.m = {s.x,0,0,0, 0,s.y,0,0, 0,0,s.z,0, 0,0,0,1};
        return r;
    }

    static AR_FORCE_INLINE Mat4 rotationXYZ(float pitch, float yaw, float roll) {
        float cp = std::cos(pitch), sp = std::sin(pitch);
        float cy = std::cos(yaw), sy = std::sin(yaw);
        float cr = std::cos(roll), sr = std::sin(roll);

        Mat4 rx{{1,0,0,0, 0,cp,sp,0, 0,-sp,cp,0, 0,0,0,1}};
        Mat4 ry{{cy,0,-sy,0, 0,1,0,0, sy,0,cy,0, 0,0,0,1}};
        Mat4 rz{{cr,sr,0,0, -sr,cr,0,0, 0,0,1,0, 0,0,0,1}};
        return rz*ry*rx;
    }

    static AR_FORCE_INLINE Mat4 perspective(float fovy,float aspect,
                                            float zNear,float zFar){
        float f = 1.f/std::tan(fovy*0.5f);
        Mat4 r{};
        r.m = {f/aspect,0,0,0,
               0,f,0,0,
               0,0,(zFar+zNear)/(zNear-zFar),-1,
               0,0,(2*zFar*zNear)/(zNear-zFar),0};
        return r;
    }

    AR_FORCE_INLINE Vec3 transformPoint(const Vec3& v) const {
        float x = m[0]*v.x + m[4]*v.y + m[8]*v.z  + m[12];
        float y = m[1]*v.x + m[5]*v.y + m[9]*v.z  + m[13];
        float z = m[2]*v.x + m[6]*v.y + m[10]*v.z + m[14];
        float w = m[3]*v.x + m[7]*v.y + m[11]*v.z + m[15];
        if (w!=0.f && w!=1.f) { x/=w; y/=w; z/=w; }
        return {x,y,z};
    }

};

} // namespace toyar

/* ────────────────── Minimal C ABI exports for Wasm callers ───────────
   (symbol names stay unmangled in extern "C")                 */
extern "C" {

AR_API const float* ar_mat4_identity();
AR_API void ar_mat4_mul(const float* a, const float* b, float* out);
AR_API void ar_vec3_transform(const float* m, const float* v, float* out);

} // extern "C"

#endif
