#ifndef AR_OCCLUSION_H_
#define AR_OCCLUSION_H_

#include "ar_la.h"
#include <GL/glew.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>
#include <vector>
#include <memory>

using namespace toyar;

// Occlusion detection methods
enum class OcclusionMethod {
    DEPTH_ESTIMATION,     // Monocular depth estimation
    BACKGROUND_SUBTRACTION, // Background subtraction
    MOTION_DETECTION,     // Motion-based occlusion
    SEMANTIC_SEGMENTATION // Semantic segmentation (if available)
};

// Occlusion region data
struct OcclusionRegion {
    cv::Rect boundingBox;
    cv::Mat mask;           // Binary mask of occluding region
    float confidence;       // Confidence of occlusion detection
    float depth;           // Estimated depth of occluding object
    int trackingId;        // For tracking across frames
    
    OcclusionRegion() : confidence(0.0f), depth(1.0f), trackingId(-1) {}
};

// Depth estimation configuration
struct DepthEstimationConfig {
    std::string modelPath;
    int inputWidth;
    int inputHeight;
    float depthScale;
    float minDepth;
    float maxDepth;
    
    DepthEstimationConfig()
        : modelPath("")
        , inputWidth(384)
        , inputHeight(288)
        , depthScale(1.0f)
        , minDepth(0.1f)
        , maxDepth(10.0f) {}
};

class ARDepthEstimator {
public:
    ARDepthEstimator();
    ~ARDepthEstimator();
    
    bool initialize(const DepthEstimationConfig& config = DepthEstimationConfig());
    void cleanup();
    
    // Estimate depth from single image
    cv::Mat estimateDepth(const cv::Mat& frame);
    
    // Get depth at specific point
    float getDepthAtPoint(const cv::Point2f& point, const cv::Mat& depthMap);
    
    // Configuration
    void setDepthRange(float minDepth, float maxDepth);
    bool isInitialized() const { return initialized; }
    
private:
    bool initializeDepthModel();
    cv::Mat preprocessForDepth(const cv::Mat& frame);
    cv::Mat postprocessDepth(const cv::Mat& rawDepth);
    
    DepthEstimationConfig config;
    cv::dnn::Net depthNet;
    bool initialized;
    bool useSimpleDepth;  // Fallback to simple depth estimation
    
    // Fallback depth estimation using image features
    cv::Mat estimateDepthSimple(const cv::Mat& frame);
    cv::Mat calculateDefocusMap(const cv::Mat& frame);
    cv::Mat calculateSizeBasedDepth(const cv::Mat& frame);
};

class ARBackgroundSubtractor {
public:
    ARBackgroundSubtractor();
    ~ARBackgroundSubtractor();
    
    bool initialize();
    void cleanup();
    
    // Update background model
    void updateBackground(const cv::Mat& frame);
    
    // Get foreground mask (potential occluders)
    cv::Mat getForegroundMask(const cv::Mat& frame);
    
    // Reset background model
    void resetBackground();
    
    // Configuration
    void setLearningRate(double rate) { learningRate = rate; }
    void setSensitivity(int threshold) { this->threshold = threshold; }
    
private:
    cv::Ptr<cv::BackgroundSubtractorMOG2> backgroundSubtractor;
    cv::Mat morphKernel;
    double learningRate;
    int threshold;
    bool initialized;
    int frameCount;
};

class AROcclusionRenderer {
public:
    AROcclusionRenderer();
    ~AROcclusionRenderer();
    
    bool initialize(int width, int height);
    void cleanup();
    
    // Create occlusion mask texture from detected regions
    void updateOcclusionMask(const std::vector<OcclusionRegion>& regions, 
                            const cv::Mat& depthMap, 
                            int frameWidth, int frameHeight);
    
    // Render objects with occlusion
    void renderWithOcclusion(const std::vector<class ARObject*>& objects, 
                            const Mat4& viewMatrix, 
                            const Mat4& projMatrix,
                            const Mat4& cameraTransform);
    
    // Get occlusion textures for use in shaders
    GLuint getOcclusionMaskTexture() const { return occlusionMaskTexture; }
    GLuint getDepthTexture() const { return depthTexture; }
    
    // Configuration
    void setOcclusionStrength(float strength) { occlusionStrength = strength; }
    void setDepthTolerance(float tolerance) { depthTolerance = tolerance; }
    
private:
    bool setupOcclusionFramebuffers();
    bool createOcclusionShaders();
    void uploadOcclusionData(const cv::Mat& mask, const cv::Mat& depth);
    
    // OpenGL resources
    GLuint occlusionFBO;
    GLuint occlusionMaskTexture;
    GLuint depthTexture;
    GLuint colorTexture;
    
    // Shaders for occlusion rendering
    GLuint occlusionVertexShader;
    GLuint occlusionFragmentShader;
    GLuint occlusionShaderProgram;
    
    // Render parameters
    int renderWidth, renderHeight;
    float occlusionStrength;
    float depthTolerance;
    bool initialized;
};

class AROcclusionTracker {
public:
    AROcclusionTracker();
    ~AROcclusionTracker();
    
    bool initialize(OcclusionMethod method = OcclusionMethod::BACKGROUND_SUBTRACTION);
    void cleanup();
    
    // Main processing function
    std::vector<OcclusionRegion> detectOcclusion(const cv::Mat& frame, 
                                                 const cv::Mat& depthMap = cv::Mat());
    
    // Track occluders across frames
    void updateTracking(std::vector<OcclusionRegion>& regions);
    
    // Configuration
    void setOcclusionMethod(OcclusionMethod method);
    void setMinOccluderSize(int minSize) { minOccluderSize = minSize; }
    void setConfidenceThreshold(float threshold) { confidenceThreshold = threshold; }
    
    // State queries
    bool hasActiveOccluders() const { return !trackedRegions.empty(); }
    const std::vector<OcclusionRegion>& getTrackedRegions() const { return trackedRegions; }
    
private:
    // Detection methods
    std::vector<OcclusionRegion> detectByBackgroundSubtraction(const cv::Mat& frame);
    std::vector<OcclusionRegion> detectByDepthEstimation(const cv::Mat& frame, const cv::Mat& depthMap);
    std::vector<OcclusionRegion> detectByMotion(const cv::Mat& frame);
    
    // Tracking helpers
    void associateRegions(std::vector<OcclusionRegion>& newRegions);
    float calculateRegionSimilarity(const OcclusionRegion& a, const OcclusionRegion& b);
    void filterRegions(std::vector<OcclusionRegion>& regions);
    
    OcclusionMethod currentMethod;
    
    // Component instances
    std::unique_ptr<ARDepthEstimator> depthEstimator;
    std::unique_ptr<ARBackgroundSubtractor> backgroundSubtractor;
    
    // Tracking state
    std::vector<OcclusionRegion> trackedRegions;
    int nextTrackingId;
    
    // Configuration
    int minOccluderSize;
    float confidenceThreshold;
    int maxTrackingAge;
    
    // Motion detection state
    cv::Mat previousFrame;
    cv::Mat flowMask;
    
    bool initialized;
};

// Utility functions for occlusion handling
namespace OcclusionUtils {
    // Convert OpenCV mask to OpenGL texture
    GLuint createTextureFromMask(const cv::Mat& mask);
    
    // Depth map utilities
    cv::Mat normalizeDepthMap(const cv::Mat& depthMap, float minDepth, float maxDepth);
    cv::Mat resizeDepthMap(const cv::Mat& depthMap, int width, int height);
    
    // Region processing
    std::vector<cv::Rect> findOcclusionRegions(const cv::Mat& mask, int minSize);
    cv::Mat createRegionMask(const std::vector<OcclusionRegion>& regions, int width, int height);
    
    // Geometric utilities
    bool isPointOccluded(const Vec3& worldPoint, const Mat4& viewMatrix, const Mat4& projMatrix,
                        const cv::Mat& depthMap, float tolerance = 0.1f);
    
    Vec3 projectToScreen(const Vec3& worldPoint, const Mat4& viewMatrix, const Mat4& projMatrix,
                        int screenWidth, int screenHeight);
    
    // Filtering and smoothing
    cv::Mat morphologicalCleanup(const cv::Mat& mask);
    cv::Mat temporalFilter(const cv::Mat& currentMask, const cv::Mat& previousMask, float alpha = 0.7f);
}

#endif // AR_OCCLUSION_H_
