#include "ar_la.h"
#include <cstring>

/* ────────────────── C ABI exports implementation ─────────────── */

extern "C" {

AR_API const float* ar_mat4_identity() {
    static toyar::Mat4 I = toyar::Mat4::identity();   // static to keep address stable
    return I.data();
}

AR_API void ar_mat4_mul(const float* a, const float* b, float* out) {
    const toyar::Mat4& A = *reinterpret_cast<const toyar::Mat4*>(a);
    const toyar::Mat4& B = *reinterpret_cast<const toyar::Mat4*>(b);
    *reinterpret_cast<toyar::Mat4*>(out) = A * B;
}

AR_API void ar_vec3_transform(const float* m, const float* v, float* out) {
    const toyar::Mat4& M = *reinterpret_cast<const toyar::Mat4*>(m);
    const toyar::Vec3& V = *reinterpret_cast<const toyar::Vec3*>(v);
    toyar::Vec3 r = M.transformPoint(V);
    std::memcpy(out, &r, sizeof(toyar::Vec3));
}

} // extern "C"
