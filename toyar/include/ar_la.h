#ifndef AR_LA_H_
#define AR_LA_H_

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <cassert>

/* ───────────────── Platform / visibility macros ───────────────── */

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#define AR_API EMSCRIPTEN_KEEPALIVE           // exported to JS / other Wasm
#else
#define AR_API                                // no decoration on native
#endif

/* Mathematical constants using modern C++ idioms */
namespace toyar {
    inline constexpr float PI = 3.14159265358979323846f;
    inline constexpr float PI_2 = PI * 0.5f;
    inline constexpr float PI_4 = PI * 0.25f;
    inline constexpr float EPSILON = std::numeric_limits<float>::epsilon() * 10.0f;
    inline constexpr float DEG_TO_RAD = PI / 180.0f;
    inline constexpr float RAD_TO_DEG = 180.0f / PI;
}

namespace toyar {

struct Vec3 {
    float x, y, z;

    constexpr Vec3() : x(0), y(0), z(0) {}
    constexpr Vec3(float X, float Y, float Z) : x(X), y(Y), z(Z) {}

    constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator*(float s) const { return {x*s, y*s, z*s}; }
    constexpr Vec3 operator/(float s) const { 
        assert(std::abs(s) > EPSILON && "Division by zero or near-zero");
        return {x/s, y/s, z/s}; 
    }
    
    constexpr Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    constexpr Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    constexpr Vec3& operator*=(float s) { x *= s; y *= s; z *= s; return *this; }
    constexpr Vec3& operator/=(float s) { 
        assert(std::abs(s) > EPSILON && "Division by zero or near-zero");
        x /= s; y /= s; z /= s; return *this; 
    }

    constexpr bool operator==(const Vec3& o) const {
        return std::abs(x - o.x) < EPSILON && 
               std::abs(y - o.y) < EPSILON && 
               std::abs(z - o.z) < EPSILON;
    }
    
    constexpr bool operator!=(const Vec3& o) const { return !(*this == o); }

    constexpr float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    constexpr Vec3 cross(const Vec3& o) const {
        return { y*o.z - z*o.y, z*o.x - x*o.z, x*o.y - y*o.x };
    }
    
    constexpr float lengthSquared() const { return dot(*this); }
    float length() const { return std::sqrt(lengthSquared()); }
    
    Vec3 normalized() const { 
        const float len = length();
        if (len < EPSILON) {
            assert(false && "Cannot normalize zero-length vector");
            return *this;
        }
        return *this * (1.0f / len);
    }
    
    constexpr float* data() { return &x; }
    constexpr const float* data() const { return &x; }
    
    // Legacy compatibility
    [[deprecated("Use length() instead")]]
    float norm() const { return length(); }
    
    static constexpr Vec3 zero() { return {0, 0, 0}; }
    static constexpr Vec3 one() { return {1, 1, 1}; }
    static constexpr Vec3 up() { return {0, 1, 0}; }
    static constexpr Vec3 right() { return {1, 0, 0}; }
    static constexpr Vec3 forward() { return {0, 0, -1}; }
};

constexpr Vec3 operator*(float s, const Vec3& v) { return v * s; }

struct alignas(16) Mat4 {
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
    
    constexpr const float* data() const { return m.data(); }
    float* data() { return m.data(); }
    
    constexpr float& operator[](size_t index) { return m[index]; }
    constexpr const float& operator[](size_t index) const { return m[index]; }
    
    constexpr float& operator()(size_t row, size_t col) { return m[col*4 + row]; }
    constexpr const float& operator()(size_t row, size_t col) const { return m[col*4 + row]; }

    // Optimized matrix multiplication using cache-friendly ordering
    Mat4 operator*(const Mat4& b) const {
        Mat4 result{};
        result.m.fill(0);
        
        // Cache-friendly i-k-j ordering for better memory access pattern
        for (int i = 0; i < 4; ++i) {
            for (int k = 0; k < 4; ++k) {
                const float a_ik = m[k*4 + i];
                for (int j = 0; j < 4; ++j) {
                    result.m[j*4 + i] += a_ik * b.m[j*4 + k];
                }
            }
        }
        return result;
    }
    
    constexpr bool operator==(const Mat4& other) const {
        for (size_t i = 0; i < 16; ++i) {
            if (std::abs(m[i] - other.m[i]) > EPSILON) return false;
        }
        return true;
    }
    
    constexpr bool operator!=(const Mat4& other) const { return !(*this == other); }

    static constexpr Mat4 identity() { return Mat4(); }

    static constexpr Mat4 translation(const Vec3& t) {
        Mat4 r = identity();
        r.m[12] = t.x; r.m[13] = t.y; r.m[14] = t.z;
        return r;
    }

    static constexpr Mat4 scale(const Vec3& s) {
        Mat4 r{};
        r.m = {s.x,0,0,0, 0,s.y,0,0, 0,0,s.z,0, 0,0,0,1};
        return r;
    }

    static Mat4 rotationX(float angle) {
        const float c = std::cos(angle), s = std::sin(angle);
        return Mat4{1,0,0,0, 0,c,s,0, 0,-s,c,0, 0,0,0,1};
    }
    
    static Mat4 rotationY(float angle) {
        const float c = std::cos(angle), s = std::sin(angle);
        return Mat4{c,0,-s,0, 0,1,0,0, s,0,c,0, 0,0,0,1};
    }
    
    static Mat4 rotationZ(float angle) {
        const float c = std::cos(angle), s = std::sin(angle);
        return Mat4{c,s,0,0, -s,c,0,0, 0,0,1,0, 0,0,0,1};
    }

    static Mat4 rotationXYZ(float pitch, float yaw, float roll) {
        // Intrinsic rotations: Z(roll) * Y(yaw) * X(pitch)
        return rotationZ(roll) * rotationY(yaw) * rotationX(pitch);
    }
    
    // Left-handed perspective projection (consistent with DirectX convention)
    static Mat4 perspective(float fovy, float aspect, float zNear, float zFar) {
        assert(aspect > EPSILON && "Invalid aspect ratio");
        assert(zFar > zNear && "Invalid depth range");
        assert(zNear > EPSILON && "Invalid near plane");
        
        const float f = 1.0f / std::tan(fovy * 0.5f);
        const float depth_range = zFar - zNear;
        
        Mat4 r{};
        r.m = {f/aspect, 0, 0, 0,
               0, f, 0, 0,
               0, 0, zFar/depth_range, 1,
               0, 0, -(zFar*zNear)/depth_range, 0};
        return r;
    }
    
    // Right-handed perspective projection (consistent with OpenGL convention)
    static Mat4 perspectiveRH(float fovy, float aspect, float zNear, float zFar) {
        assert(aspect > EPSILON && "Invalid aspect ratio");
        assert(zFar > zNear && "Invalid depth range");
        assert(zNear > EPSILON && "Invalid near plane");
        
        const float f = 1.0f / std::tan(fovy * 0.5f);
        const float depth_range = zNear - zFar;
        
        Mat4 r{};
        r.m = {f/aspect, 0, 0, 0,
               0, f, 0, 0,
               0, 0, (zFar+zNear)/depth_range, -1,
               0, 0, (2*zFar*zNear)/depth_range, 0};
        return r;
    }
    
    static Mat4 orthographic(float left, float right, float bottom, float top, float zNear, float zFar) {
        assert(right > left && top > bottom && zFar > zNear && "Invalid orthographic parameters");
        
        const float width = right - left;
        const float height = top - bottom;
        const float depth = zFar - zNear;
        
        Mat4 r{};
        r.m = {2/width, 0, 0, 0,
               0, 2/height, 0, 0,
               0, 0, -2/depth, 0,
               -(right+left)/width, -(top+bottom)/height, -(zFar+zNear)/depth, 1};
        return r;
    }
    
    static Mat4 lookAt(const Vec3& eye, const Vec3& center, const Vec3& up) {
        const Vec3 f = (center - eye).normalized();
        const Vec3 s = f.cross(up).normalized();
        const Vec3 u = s.cross(f);
        
        Mat4 result{};
        result.m = {s.x, u.x, -f.x, 0,
                   s.y, u.y, -f.y, 0,
                   s.z, u.z, -f.z, 0,
                   -s.dot(eye), -u.dot(eye), f.dot(eye), 1};
        return result;
    }

    constexpr Vec3 transformPoint(const Vec3& v) const {
        const float x = m[0]*v.x + m[4]*v.y + m[8]*v.z  + m[12];
        const float y = m[1]*v.x + m[5]*v.y + m[9]*v.z  + m[13];
        const float z = m[2]*v.x + m[6]*v.y + m[10]*v.z + m[14];
        const float w = m[3]*v.x + m[7]*v.y + m[11]*v.z + m[15];
        
        if (std::abs(w) > EPSILON && std::abs(w - 1.0f) > EPSILON) {
            return {x/w, y/w, z/w};
        }
        return {x, y, z};
    }
    
    constexpr Vec3 transformVector(const Vec3& v) const {
        // Transform as direction vector (ignore translation)
        return {m[0]*v.x + m[4]*v.y + m[8]*v.z,
                m[1]*v.x + m[5]*v.y + m[9]*v.z,
                m[2]*v.x + m[6]*v.y + m[10]*v.z};
    }
    
    // Extract translation component
    constexpr Vec3 getTranslation() const {
        return {m[12], m[13], m[14]};
    }
    
    // Extract scale component (assumes no skew)
    Vec3 getScale() const {
        const Vec3 x_axis{m[0], m[1], m[2]};
        const Vec3 y_axis{m[4], m[5], m[6]};
        const Vec3 z_axis{m[8], m[9], m[10]};
        return {x_axis.length(), y_axis.length(), z_axis.length()};
    }

    // GLM compatibility (if GLM is available)
    #ifdef GLM_VERSION
    Mat4(const glm::mat4& glm_matrix) {
        std::memcpy(m.data(), glm::value_ptr(glm_matrix), 16 * sizeof(float));
    }
    
    operator glm::mat4() const {
        return glm::make_mat4(m.data());
    }
    
    glm::mat4 toGLM() const {
        return glm::make_mat4(m.data());
    }
    
    static Mat4 fromGLM(const glm::mat4& glm_matrix) {
        Mat4 result;
        std::memcpy(result.m.data(), glm::value_ptr(glm_matrix), 16 * sizeof(float));
        return result;
    }
    #endif

};

} // namespace toyar

/* ────────────────── Minimal C ABI exports for Wasm callers ───────────
   (symbol names stay unmangled in extern "C")                 */
extern "C" {

AR_API inline const float* ar_mat4_identity() {
    static toyar::Mat4 I = toyar::Mat4::identity();   // static to keep address stable
    return I.data();
}

AR_API inline void ar_mat4_mul(const float* a, const float* b, float* out) {
    const toyar::Mat4& A = *reinterpret_cast<const toyar::Mat4*>(a);
    const toyar::Mat4& B = *reinterpret_cast<const toyar::Mat4*>(b);
    *reinterpret_cast<toyar::Mat4*>(out) = A * B;
}

AR_API inline void ar_vec3_transform(const float* m, const float* v, float* out) {
    const toyar::Mat4& M = *reinterpret_cast<const toyar::Mat4*>(m);
    const toyar::Vec3& V = *reinterpret_cast<const toyar::Vec3*>(v);
    toyar::Vec3 r = M.transformPoint(V);
    std::memcpy(out, &r, sizeof(toyar::Vec3));
}

} // extern "C"

#endif
