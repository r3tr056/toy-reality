#ifndef AR_LIGHTING_H_
#define AR_LIGHTING_H_

#include "ar_la.h"
#include <GL/glew.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

using namespace toyar;

// Environmental light estimation data
struct LightEstimate {
    Vec3 ambientColor;          // RGB ambient light color
    float ambientIntensity;     // Overall ambient light intensity
    Vec3 directionalLightDir;   // Primary directional light direction
    Vec3 directionalLightColor; // Directional light color
    float directionalIntensity; // Directional light intensity
    float colorTemperature;     // Light color temperature in Kelvin
    float confidence;           // Estimation confidence [0-1]
    
    LightEstimate() 
        : ambientColor(0.4f, 0.4f, 0.4f)
        , ambientIntensity(0.3f)
        , directionalLightDir(0.0f, -1.0f, 0.0f)
        , directionalLightColor(1.0f, 1.0f, 1.0f)
        , directionalIntensity(0.8f)
        , colorTemperature(5500.0f)
        , confidence(0.5f) {}
};

// Shadow mapping configuration
struct ShadowMapConfig {
    int shadowMapSize;
    float shadowBias;
    float shadowRadius;
    float lightFrustumSize;
    float lightNearPlane;
    float lightFarPlane;
    
    ShadowMapConfig()
        : shadowMapSize(2048)
        , shadowBias(0.005f)
        , shadowRadius(2.0f)
        , lightFrustumSize(10.0f)
        , lightNearPlane(0.1f)
        , lightFarPlane(20.0f) {}
};

class ARLightingEstimator {
public:
    ARLightingEstimator();
    ~ARLightingEstimator();
    
    bool initialize();
    void cleanup();
    
    // Main light estimation from camera frame
    LightEstimate estimateLight(const cv::Mat& frame, const Mat4& cameraTransform);
    
    // Get current light estimate
    const LightEstimate& getCurrentEstimate() const { return currentEstimate; }
    
    // Configuration
    void setAutoEstimation(bool enabled) { autoEstimationEnabled = enabled; }
    void setManualAmbient(const Vec3& color, float intensity);
    void setManualDirectional(const Vec3& direction, const Vec3& color, float intensity);
    
private:
    // Light estimation algorithms
    float analyzeAmbientIntensity(const cv::Mat& frame);
    Vec3 analyzeColorTemperature(const cv::Mat& frame);
    Vec3 estimateDirectionalLight(const cv::Mat& frame);
    float calculateConfidence(const cv::Mat& frame);
    
    // Image processing helpers
    cv::Mat preprocessFrame(const cv::Mat& frame);
    std::vector<cv::Point2f> detectSpecularHighlights(const cv::Mat& frame);
    float analyzeShadowDirections(const cv::Mat& frame);
    
    LightEstimate currentEstimate;
    LightEstimate previousEstimate;
    
    bool autoEstimationEnabled;
    bool isInitialized;
    
    // Smoothing and temporal filtering
    std::vector<LightEstimate> estimateHistory;
    static const int MAX_HISTORY_SIZE = 30;
    
    // Image processing state
    cv::Mat grayFrame;
    cv::Mat blurredFrame;
    cv::Mat gradientFrame;
};

class ARShadowRenderer {
public:
    ARShadowRenderer();
    ~ARShadowRenderer();
    
    bool initialize(const ShadowMapConfig& config = ShadowMapConfig());
    void cleanup();
    
    // Shadow map generation
    void beginShadowMapGeneration(const LightEstimate& lightEst, const Vec3& sceneCenter);
    void endShadowMapGeneration();
    
    // Render shadows for objects
    void renderObjectShadows(const std::vector<class ARObject*>& objects, 
                           const Mat4& lightViewMatrix, const Mat4& lightProjMatrix);
    
    // Get shadow map for shaders
    GLuint getShadowMapTexture() const { return shadowMapTexture; }
    Mat4 getLightViewMatrix() const { return lightViewMatrix; }
    Mat4 getLightProjectionMatrix() const { return lightProjMatrix; }
    
    // Configuration
    void updateConfig(const ShadowMapConfig& config);
    const ShadowMapConfig& getConfig() const { return config; }
    
private:
    bool createShadowMapFramebuffer();
    void setupShadowShaders();
    Mat4 calculateLightViewMatrix(const Vec3& lightDir, const Vec3& sceneCenter);
    Mat4 calculateLightProjectionMatrix();
    
    ShadowMapConfig config;
    
    // OpenGL resources
    GLuint shadowMapFBO;
    GLuint shadowMapTexture;
    GLuint shadowMapDepthTexture;
    
    GLuint shadowVertexShader;
    GLuint shadowFragmentShader;
    GLuint shadowShaderProgram;
    
    // Light matrices
    Mat4 lightViewMatrix;
    Mat4 lightProjMatrix;
    
    bool isInitialized;
};

class AREnvironmentalRenderer {
public:
    AREnvironmentalRenderer();
    ~AREnvironmentalRenderer();
    
    bool initialize();
    void cleanup();
    
    // Environment cube map for reflections
    bool createEnvironmentMap(int size = 512);
    void updateEnvironmentMap(const cv::Mat& cameraFrame, const Mat4& cameraTransform);
    
    // Image-based lighting
    Vec3 sampleEnvironmentLighting(const Vec3& direction) const;
    GLuint getEnvironmentTexture() const { return environmentCubeMap; }
    
    // Real-world occlusion estimation
    cv::Mat estimateOcclusionMask(const cv::Mat& currentFrame, const cv::Mat& backgroundFrame);
    void updateBackgroundModel(const cv::Mat& frame);
    
private:
    bool setupCubeMapFramebuffer();
    void renderEnvironmentFace(int face, const Mat4& viewMatrix);
    
    // Environment mapping
    GLuint environmentCubeMap;
    GLuint environmentFBO;
    GLuint environmentDepthBuffer;
    
    // Background subtraction for occlusion
    cv::BackgroundSubtractorMOG2* backgroundSubtractor;
    cv::Mat backgroundModel;
    cv::Mat occlusionMask;
    
    bool isInitialized;
    int environmentMapSize;
};

// Utility functions for lighting calculations
namespace LightingUtils {
    // Color temperature conversion
    Vec3 kelvinToRGB(float kelvin);
    float rgbToKelvin(const Vec3& rgb);
    
    // Light direction estimation from shadows
    Vec3 estimateLightDirectionFromShadows(const cv::Mat& frame);
    
    // Exposure and intensity analysis
    float calculateExposureValue(const cv::Mat& frame);
    float calculateContrastRatio(const cv::Mat& frame);
    
    // Smoothing and filtering
    LightEstimate smoothLightEstimate(const std::vector<LightEstimate>& history);
    
    // Matrix utilities for lighting
    Mat4 createShadowMatrix(const Mat4& lightView, const Mat4& lightProj);
    Vec3 transformDirection(const Vec3& dir, const Mat4& transform);
}

#endif // AR_LIGHTING_H_
