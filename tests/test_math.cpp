#include <gtest/gtest.h>
#include "ar_la.h"

using namespace toyar;

class Vec3Test : public ::testing::Test {
protected:
    void SetUp() override {
        v1 = Vec3(1.0f, 2.0f, 3.0f);
        v2 = Vec3(4.0f, 5.0f, 6.0f);
        zero_vec = Vec3::zero();
    }

    Vec3 v1, v2, zero_vec;
};

class Mat4Test : public ::testing::Test {
protected:
    void SetUp() override {
        identity = Mat4::identity();
        translation_mat = Mat4::translation(Vec3(1.0f, 2.0f, 3.0f));
        scale_mat = Mat4::scale(Vec3(2.0f, 3.0f, 4.0f));
    }

    Mat4 identity, translation_mat, scale_mat;
};

// Vec3 Tests
TEST_F(Vec3Test, DefaultConstructor) {
    Vec3 v;
    EXPECT_FLOAT_EQ(v.x, 0.0f);
    EXPECT_FLOAT_EQ(v.y, 0.0f);
    EXPECT_FLOAT_EQ(v.z, 0.0f);
}

TEST_F(Vec3Test, ParametrizedConstructor) {
    EXPECT_FLOAT_EQ(v1.x, 1.0f);
    EXPECT_FLOAT_EQ(v1.y, 2.0f);
    EXPECT_FLOAT_EQ(v1.z, 3.0f);
}

TEST_F(Vec3Test, Addition) {
    Vec3 result = v1 + v2;
    EXPECT_FLOAT_EQ(result.x, 5.0f);
    EXPECT_FLOAT_EQ(result.y, 7.0f);
    EXPECT_FLOAT_EQ(result.z, 9.0f);
}

TEST_F(Vec3Test, Subtraction) {
    Vec3 result = v2 - v1;
    EXPECT_FLOAT_EQ(result.x, 3.0f);
    EXPECT_FLOAT_EQ(result.y, 3.0f);
    EXPECT_FLOAT_EQ(result.z, 3.0f);
}

TEST_F(Vec3Test, ScalarMultiplication) {
    Vec3 result = v1 * 2.0f;
    EXPECT_FLOAT_EQ(result.x, 2.0f);
    EXPECT_FLOAT_EQ(result.y, 4.0f);
    EXPECT_FLOAT_EQ(result.z, 6.0f);
    
    // Test commutativity
    Vec3 result2 = 2.0f * v1;
    EXPECT_EQ(result, result2);
}

TEST_F(Vec3Test, DotProduct) {
    float result = v1.dot(v2);
    EXPECT_FLOAT_EQ(result, 32.0f); // 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
}

TEST_F(Vec3Test, CrossProduct) {
    Vec3 result = v1.cross(v2);
    EXPECT_FLOAT_EQ(result.x, -3.0f); // 2*6 - 3*5 = 12 - 15 = -3
    EXPECT_FLOAT_EQ(result.y, 6.0f);  // 3*4 - 1*6 = 12 - 6 = 6
    EXPECT_FLOAT_EQ(result.z, -3.0f); // 1*5 - 2*4 = 5 - 8 = -3
}

TEST_F(Vec3Test, Length) {
    Vec3 v(3.0f, 4.0f, 0.0f);
    EXPECT_FLOAT_EQ(v.length(), 5.0f);
    EXPECT_FLOAT_EQ(v.lengthSquared(), 25.0f);
}

TEST_F(Vec3Test, Normalization) {
    Vec3 v(3.0f, 4.0f, 0.0f);
    Vec3 normalized = v.normalized();
    EXPECT_FLOAT_EQ(normalized.length(), 1.0f);
    EXPECT_FLOAT_EQ(normalized.x, 0.6f);
    EXPECT_FLOAT_EQ(normalized.y, 0.8f);
    EXPECT_FLOAT_EQ(normalized.z, 0.0f);
}

TEST_F(Vec3Test, Equality) {
    Vec3 v_same(1.0f, 2.0f, 3.0f);
    Vec3 v_different(1.1f, 2.0f, 3.0f);
    
    EXPECT_EQ(v1, v_same);
    EXPECT_NE(v1, v_different);
}

TEST_F(Vec3Test, StaticConstants) {
    EXPECT_EQ(Vec3::zero(), Vec3(0, 0, 0));
    EXPECT_EQ(Vec3::one(), Vec3(1, 1, 1));
    EXPECT_EQ(Vec3::up(), Vec3(0, 1, 0));
    EXPECT_EQ(Vec3::right(), Vec3(1, 0, 0));
    EXPECT_EQ(Vec3::forward(), Vec3(0, 0, -1));
}

// Mat4 Tests
TEST_F(Mat4Test, DefaultConstructor) {
    Mat4 m;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (i == j) {
                EXPECT_FLOAT_EQ(m(i, j), 1.0f);
            } else {
                EXPECT_FLOAT_EQ(m(i, j), 0.0f);
            }
        }
    }
}

TEST_F(Mat4Test, Identity) {
    EXPECT_EQ(identity, Mat4::identity());
    
    // Test that identity times any matrix equals that matrix
    Mat4 result = identity * translation_mat;
    EXPECT_EQ(result, translation_mat);
}

TEST_F(Mat4Test, Translation) {
    Vec3 point(1.0f, 1.0f, 1.0f);
    Vec3 translated = translation_mat.transformPoint(point);
    
    EXPECT_FLOAT_EQ(translated.x, 2.0f);
    EXPECT_FLOAT_EQ(translated.y, 3.0f);
    EXPECT_FLOAT_EQ(translated.z, 4.0f);
}

TEST_F(Mat4Test, Scale) {
    Vec3 point(1.0f, 1.0f, 1.0f);
    Vec3 scaled = scale_mat.transformPoint(point);
    
    EXPECT_FLOAT_EQ(scaled.x, 2.0f);
    EXPECT_FLOAT_EQ(scaled.y, 3.0f);
    EXPECT_FLOAT_EQ(scaled.z, 4.0f);
}

TEST_F(Mat4Test, Rotation) {
    // Test 90-degree rotation around Y axis
    Mat4 rot_y = Mat4::rotationY(PI_2);
    Vec3 point(1.0f, 0.0f, 0.0f);
    Vec3 rotated = rot_y.transformPoint(point);
    
    EXPECT_NEAR(rotated.x, 0.0f, EPSILON);
    EXPECT_NEAR(rotated.y, 0.0f, EPSILON);
    EXPECT_NEAR(rotated.z, 1.0f, EPSILON);
}

TEST_F(Mat4Test, MatrixMultiplication) {
    // Test that scale followed by translation works correctly
    Mat4 combined = translation_mat * scale_mat;
    Vec3 point(1.0f, 1.0f, 1.0f);
    Vec3 result = combined.transformPoint(point);
    
    // First scale (2,3,4), then translate (1,2,3)
    EXPECT_FLOAT_EQ(result.x, 3.0f); // 1*2 + 1 = 3
    EXPECT_FLOAT_EQ(result.y, 5.0f); // 1*3 + 2 = 5
    EXPECT_FLOAT_EQ(result.z, 7.0f); // 1*4 + 3 = 7
}

TEST_F(Mat4Test, Perspective) {
    Mat4 proj = Mat4::perspectiveRH(PI_4, 16.0f/9.0f, 0.1f, 100.0f);
    
    // Test that the projection matrix has reasonable values
    EXPECT_GT(proj(0, 0), 0.0f); // X scale should be positive
    EXPECT_GT(proj(1, 1), 0.0f); // Y scale should be positive
    EXPECT_LT(proj(2, 2), 0.0f); // Z scale should be negative for RH
    EXPECT_FLOAT_EQ(proj(2, 3), -1.0f); // W component should be -1 for RH
}

TEST_F(Mat4Test, LookAt) {
    Vec3 eye(0, 0, 0);
    Vec3 center(0, 0, -1);
    Vec3 up(0, 1, 0);
    
    Mat4 view = Mat4::lookAt(eye, center, up);
    
    // For a standard view matrix looking down -Z with Y up,
    // we expect the identity transformation for this specific case
    Vec3 transformed = view.transformPoint(Vec3(1, 0, 0));
    EXPECT_NEAR(transformed.x, 1.0f, EPSILON);
}

TEST_F(Mat4Test, TransformVector) {
    Vec3 vector(1.0f, 0.0f, 0.0f);
    Vec3 transformed = translation_mat.transformVector(vector);
    
    // Vector transformation should ignore translation
    EXPECT_FLOAT_EQ(transformed.x, 1.0f);
    EXPECT_FLOAT_EQ(transformed.y, 0.0f);
    EXPECT_FLOAT_EQ(transformed.z, 0.0f);
}

TEST_F(Mat4Test, ExtractComponents) {
    Vec3 translation = translation_mat.getTranslation();
    EXPECT_FLOAT_EQ(translation.x, 1.0f);
    EXPECT_FLOAT_EQ(translation.y, 2.0f);
    EXPECT_FLOAT_EQ(translation.z, 3.0f);
    
    Vec3 scale = scale_mat.getScale();
    EXPECT_FLOAT_EQ(scale.x, 2.0f);
    EXPECT_FLOAT_EQ(scale.y, 3.0f);
    EXPECT_FLOAT_EQ(scale.z, 4.0f);
}

// Performance and numerical stability tests
TEST(MathPerformanceTest, MatrixMultiplicationStability) {
    Mat4 m = Mat4::identity();
    
    // Accumulate many small rotations - should remain stable
    for (int i = 0; i < 1000; ++i) {
        m = m * Mat4::rotationY(0.001f);
    }
    
    // After 1000 * 0.001 = 1 radian rotation, we should be close to expected result
    Vec3 point(1, 0, 0);
    Vec3 rotated = m.transformPoint(point);
    Vec3 expected = Mat4::rotationY(1.0f).transformPoint(point);
    
    EXPECT_NEAR(rotated.x, expected.x, 0.01f);
    EXPECT_NEAR(rotated.y, expected.y, 0.01f);
    EXPECT_NEAR(rotated.z, expected.z, 0.01f);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
