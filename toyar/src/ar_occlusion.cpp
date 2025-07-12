#include "ar_occlusion.h"
#include "ar_object.h"
#include <iostream>
#include <algorithm>
#include <cmath>

// ARDepthEstimator Implementation
ARDepthEstimator::ARDepthEstimator()
    : initialized(false)
    , useSimpleDepth(true)
{
}

ARDepthEstimator::~ARDepthEstimator() {
    cleanup();
}

bool ARDepthEstimator::initialize(const DepthEstimationConfig& config) {
    this->config = config;
    
    // Try to initialize DNN-based depth estimation
    if (!config.modelPath.empty() && initializeDepthModel()) {
        useSimpleDepth = false;
        std::cout << "Depth estimator initialized with DNN model" << std::endl;
    } else {
        useSimpleDepth = true;
        std::cout << "Using simple depth estimation (no DNN model available)" << std::endl;
    }
    
    initialized = true;
    return true;
}

void ARDepthEstimator::cleanup() {
    if (!depthNet.empty()) {
        depthNet = cv::dnn::Net();
    }
    initialized = false;
}

cv::Mat ARDepthEstimator::estimateDepth(const cv::Mat& frame) {
    if (!initialized || frame.empty()) {
        return cv::Mat();
    }
    
    if (useSimpleDepth) {
        return estimateDepthSimple(frame);
    } else {
        // DNN-based depth estimation (placeholder for when model is available)
        cv::Mat preprocessed = preprocessForDepth(frame);
        
        // Set input to the network
        cv::Mat blob;
        cv::dnn::blobFromImage(preprocessed, blob, 1.0/255.0, 
                              cv::Size(config.inputWidth, config.inputHeight), 
                              cv::Scalar(0, 0, 0), true, false);
        depthNet.setInput(blob);
        
        // Run forward pass
        cv::Mat depthOutput = depthNet.forward();
        
        return postprocessDepth(depthOutput);
    }
}

cv::Mat ARDepthEstimator::estimateDepthSimple(const cv::Mat& frame) {
    cv::Mat depthMap = cv::Mat::zeros(frame.size(), CV_32F);
    
    // Convert to grayscale
    cv::Mat gray;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = frame.clone();
    }
    
    // Method 1: Defocus-based depth estimation
    cv::Mat defocusDepth = calculateDefocusMap(gray);
    
    // Method 2: Size-based depth estimation
    cv::Mat sizeDepth = calculateSizeBasedDepth(gray);
    
    // Method 3: Intensity-based depth (closer objects often brighter)
    cv::Mat intensityDepth;
    gray.convertTo(intensityDepth, CV_32F, 1.0/255.0);
    cv::GaussianBlur(intensityDepth, intensityDepth, cv::Size(5, 5), 1.0);
    
    // Invert intensity for depth (brighter = closer)
    intensityDepth = 1.0f - intensityDepth;
    
    // Combine different depth cues
    depthMap = defocusDepth * 0.5f + sizeDepth * 0.3f + intensityDepth * 0.2f;
    
    // Scale to actual depth range
    depthMap = depthMap * (config.maxDepth - config.minDepth) + config.minDepth;
    
    return depthMap;
}

cv::Mat ARDepthEstimator::calculateDefocusMap(const cv::Mat& frame) {
    cv::Mat depthMap = cv::Mat::zeros(frame.size(), CV_32F);
    
    // Calculate gradient magnitude (proxy for focus)
    cv::Mat gradX, gradY, gradMag;
    cv::Sobel(frame, gradX, CV_32F, 1, 0, 3);
    cv::Sobel(frame, gradY, CV_32F, 0, 1, 3);
    cv::magnitude(gradX, gradY, gradMag);
    
    // Normalize gradient magnitude
    cv::normalize(gradMag, gradMag, 0.0, 1.0, cv::NORM_MINMAX);
    
    // Higher gradient = more in focus = closer
    depthMap = 1.0f - gradMag;
    
    // Apply smoothing
    cv::GaussianBlur(depthMap, depthMap, cv::Size(15, 15), 5.0);
    
    return depthMap;
}

cv::Mat ARDepthEstimator::calculateSizeBasedDepth(const cv::Mat& frame) {
    cv::Mat depthMap = cv::Mat::ones(frame.size(), CV_32F) * 0.5f;
    
    // Detect contours/objects
    cv::Mat edges;
    cv::Canny(frame, edges, 50, 150);
    
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // Assume larger objects are closer
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < 100) continue; // Skip small noise
        
        // Larger area = closer object
        float depth = std::max(0.1f, 1.0f - static_cast<float>(area) / (frame.cols * frame.rows));
        
        cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);
        cv::fillPoly(mask, std::vector<std::vector<cv::Point>>{contour}, cv::Scalar(255));
        
        depthMap.setTo(depth, mask);
    }
    
    return depthMap;
}

float ARDepthEstimator::getDepthAtPoint(const cv::Point2f& point, const cv::Mat& depthMap) {
    if (depthMap.empty() || point.x < 0 || point.y < 0 || 
        point.x >= depthMap.cols || point.y >= depthMap.rows) {
        return config.maxDepth;
    }
    
    return depthMap.at<float>(static_cast<int>(point.y), static_cast<int>(point.x));
}

bool ARDepthEstimator::initializeDepthModel() {
    // This would load a pre-trained depth estimation model
    // For now, return false to use simple depth estimation
    return false;
}

cv::Mat ARDepthEstimator::preprocessForDepth(const cv::Mat& frame) {
    cv::Mat processed;
    cv::resize(frame, processed, cv::Size(config.inputWidth, config.inputHeight));
    return processed;
}

cv::Mat ARDepthEstimator::postprocessDepth(const cv::Mat& rawDepth) {
    cv::Mat processed;
    rawDepth.convertTo(processed, CV_32F);
    return processed;
}

// ARBackgroundSubtractor Implementation
ARBackgroundSubtractor::ARBackgroundSubtractor()
    : learningRate(0.01)
    , threshold(16)
    , initialized(false)
    , frameCount(0)
{
}

ARBackgroundSubtractor::~ARBackgroundSubtractor() {
    cleanup();
}

bool ARBackgroundSubtractor::initialize() {
    backgroundSubtractor = cv::createBackgroundSubtractorMOG2(500, threshold, false);
    backgroundSubtractor->setVarThreshold(threshold);
    backgroundSubtractor->setDetectShadows(true);
    
    // Create morphological kernel for noise reduction
    morphKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    
    initialized = true;
    std::cout << "Background subtractor initialized" << std::endl;
    return true;
}

void ARBackgroundSubtractor::cleanup() {
    backgroundSubtractor.release();
    initialized = false;
}

void ARBackgroundSubtractor::updateBackground(const cv::Mat& frame) {
    if (!initialized || frame.empty()) return;
    
    cv::Mat fgMask;
    backgroundSubtractor->apply(frame, fgMask, learningRate);
    frameCount++;
}

cv::Mat ARBackgroundSubtractor::getForegroundMask(const cv::Mat& frame) {
    if (!initialized || frame.empty()) {
        return cv::Mat::zeros(frame.size(), CV_8UC1);
    }
    
    cv::Mat fgMask;
    double currentLearningRate = (frameCount < 50) ? 0.05 : learningRate;
    backgroundSubtractor->apply(frame, fgMask, currentLearningRate);
    
    // Clean up the mask
    cv::Mat cleanMask;
    cv::morphologyEx(fgMask, cleanMask, cv::MORPH_OPEN, morphKernel);
    cv::morphologyEx(cleanMask, cleanMask, cv::MORPH_CLOSE, morphKernel);
    
    // Remove shadows (they appear as gray in MOG2)
    cv::threshold(cleanMask, cleanMask, 200, 255, cv::THRESH_BINARY);
    
    return cleanMask;
}

void ARBackgroundSubtractor::resetBackground() {
    if (initialized) {
        backgroundSubtractor = cv::createBackgroundSubtractorMOG2(500, threshold, false);
        frameCount = 0;
    }
}

// AROcclusionRenderer Implementation
AROcclusionRenderer::AROcclusionRenderer()
    : occlusionFBO(0)
    , occlusionMaskTexture(0)
    , depthTexture(0)
    , colorTexture(0)
    , occlusionVertexShader(0)
    , occlusionFragmentShader(0)
    , occlusionShaderProgram(0)
    , renderWidth(1280)
    , renderHeight(720)
    , occlusionStrength(1.0f)
    , depthTolerance(0.1f)
    , initialized(false)
{
}

AROcclusionRenderer::~AROcclusionRenderer() {
    cleanup();
}

bool AROcclusionRenderer::initialize(int width, int height) {
    renderWidth = width;
    renderHeight = height;
    
    if (!setupOcclusionFramebuffers()) {
        std::cerr << "Failed to setup occlusion framebuffers" << std::endl;
        return false;
    }
    
    if (!createOcclusionShaders()) {
        std::cerr << "Failed to create occlusion shaders" << std::endl;
        return false;
    }
    
    initialized = true;
    std::cout << "Occlusion renderer initialized (" << width << "x" << height << ")" << std::endl;
    return true;
}

void AROcclusionRenderer::cleanup() {
    if (occlusionFBO) glDeleteFramebuffers(1, &occlusionFBO);
    if (occlusionMaskTexture) glDeleteTextures(1, &occlusionMaskTexture);
    if (depthTexture) glDeleteTextures(1, &depthTexture);
    if (colorTexture) glDeleteTextures(1, &colorTexture);
    if (occlusionShaderProgram) glDeleteProgram(occlusionShaderProgram);
    if (occlusionVertexShader) glDeleteShader(occlusionVertexShader);
    if (occlusionFragmentShader) glDeleteShader(occlusionFragmentShader);
    
    occlusionFBO = occlusionMaskTexture = depthTexture = colorTexture = 0;
    occlusionVertexShader = occlusionFragmentShader = occlusionShaderProgram = 0;
    initialized = false;
}

bool AROcclusionRenderer::setupOcclusionFramebuffers() {
    // Generate framebuffer
    glGenFramebuffers(1, &occlusionFBO);
    glBindFramebuffer(GL_FRAMEBUFFER, occlusionFBO);
    
    // Create occlusion mask texture
    glGenTextures(1, &occlusionMaskTexture);
    glBindTexture(GL_TEXTURE_2D, occlusionMaskTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, renderWidth, renderHeight, 0, GL_RED, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    // Create depth texture
    glGenTextures(1, &depthTexture);
    glBindTexture(GL_TEXTURE_2D, depthTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, renderWidth, renderHeight, 0, GL_RED, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    // Create color texture for rendering
    glGenTextures(1, &colorTexture);
    glBindTexture(GL_TEXTURE_2D, colorTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, renderWidth, renderHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    
    // Attach textures to framebuffer
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorTexture, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, occlusionMaskTexture, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D, depthTexture, 0);
    
    // Check framebuffer completeness
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "Occlusion framebuffer is not complete!" << std::endl;
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return false;
    }
    
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return true;
}

bool AROcclusionRenderer::createOcclusionShaders() {
    // Occlusion vertex shader
    const char* vertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec3 FragPos;
out vec3 Normal;
out vec2 TexCoord;
out vec4 ClipPos;

void main() {
    FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(model))) * aNormal;
    TexCoord = aTexCoord;
    
    ClipPos = projection * view * vec4(FragPos, 1.0);
    gl_Position = ClipPos;
}
)";
    
    // Occlusion fragment shader
    const char* fragmentShaderSource = R"(
#version 330 core
layout (location = 0) out vec4 FragColor;
layout (location = 1) out float OcclusionMask;
layout (location = 2) out float ObjectDepth;

in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoord;
in vec4 ClipPos;

uniform sampler2D occlusionTexture;
uniform sampler2D sceneDepthTexture;
uniform float occlusionStrength;
uniform float depthTolerance;
uniform vec3 objectColor;

void main() {
    // Calculate screen coordinates
    vec2 screenCoord = ClipPos.xy / ClipPos.w * 0.5 + 0.5;
    
    // Sample occlusion mask
    float occlusionValue = texture(occlusionTexture, screenCoord).r;
    
    // Sample scene depth
    float sceneDepth = texture(sceneDepthTexture, screenCoord).r;
    
    // Calculate object depth in view space
    float objectDepth = ClipPos.z / ClipPos.w;
    
    // Check if object is occluded
    float depthDiff = sceneDepth - objectDepth;
    float isOccluded = step(depthTolerance, depthDiff) * occlusionValue;
    
    // Apply occlusion
    vec3 finalColor = objectColor;
    float alpha = 1.0 - isOccluded * occlusionStrength;
    
    FragColor = vec4(finalColor, alpha);
    OcclusionMask = isOccluded;
    ObjectDepth = objectDepth;
}
)";
    
    // Compile shaders
    occlusionVertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(occlusionVertexShader, 1, &vertexShaderSource, nullptr);
    glCompileShader(occlusionVertexShader);
    
    occlusionFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(occlusionFragmentShader, 1, &fragmentShaderSource, nullptr);
    glCompileShader(occlusionFragmentShader);
    
    // Create program
    occlusionShaderProgram = glCreateProgram();
    glAttachShader(occlusionShaderProgram, occlusionVertexShader);
    glAttachShader(occlusionShaderProgram, occlusionFragmentShader);
    glLinkProgram(occlusionShaderProgram);
    
    // Check for errors
    GLint success;
    glGetProgramiv(occlusionShaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetProgramInfoLog(occlusionShaderProgram, 512, nullptr, infoLog);
        std::cerr << "Occlusion shader linking failed: " << infoLog << std::endl;
        return false;
    }
    
    return true;
}

void AROcclusionRenderer::updateOcclusionMask(const std::vector<OcclusionRegion>& regions, 
                                            const cv::Mat& depthMap, 
                                            int frameWidth, int frameHeight) {
    if (!initialized) return;
    
    // Create combined mask from regions
    cv::Mat combinedMask = cv::Mat::zeros(frameHeight, frameWidth, CV_8UC1);
    
    for (const auto& region : regions) {
        if (region.confidence > 0.5f) {
            cv::Mat resizedMask;
            cv::resize(region.mask, resizedMask, cv::Size(frameWidth, frameHeight));
            cv::bitwise_or(combinedMask, resizedMask, combinedMask);
        }
    }
    
    // Upload mask and depth data to GPU
    uploadOcclusionData(combinedMask, depthMap);
}

void AROcclusionRenderer::uploadOcclusionData(const cv::Mat& mask, const cv::Mat& depth) {
    // Upload occlusion mask
    glBindTexture(GL_TEXTURE_2D, occlusionMaskTexture);
    if (!mask.empty()) {
        cv::Mat resizedMask;
        if (mask.size() != cv::Size(renderWidth, renderHeight)) {
            cv::resize(mask, resizedMask, cv::Size(renderWidth, renderHeight));
        } else {
            resizedMask = mask;
        }
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, renderWidth, renderHeight, 
                       GL_RED, GL_UNSIGNED_BYTE, resizedMask.data);
    }
    
    // Upload depth map
    glBindTexture(GL_TEXTURE_2D, depthTexture);
    if (!depth.empty()) {
        cv::Mat resizedDepth;
        if (depth.size() != cv::Size(renderWidth, renderHeight)) {
            cv::resize(depth, resizedDepth, cv::Size(renderWidth, renderHeight));
        } else {
            resizedDepth = depth;
        }
        
        // Convert to float if necessary
        cv::Mat floatDepth;
        if (resizedDepth.type() != CV_32F) {
            resizedDepth.convertTo(floatDepth, CV_32F);
        } else {
            floatDepth = resizedDepth;
        }
        
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, renderWidth, renderHeight, 
                       GL_RED, GL_FLOAT, floatDepth.data);
    }
    
    glBindTexture(GL_TEXTURE_2D, 0);
}

void AROcclusionRenderer::renderWithOcclusion(const std::vector<ARObject*>& objects, 
                                             const Mat4& viewMatrix, const Mat4& projMatrix, 
                                             const Mat4& cameraTransform) {
    if (!initialized || objects.empty()) {
        // Fallback to normal rendering
        for (const auto* obj : objects) {
            if (obj) obj->draw();
        }
        return;
    }
    
    // Simple implementation: render objects normally
    // In a production system, this would handle depth testing and occlusion masking
    glEnable(GL_DEPTH_TEST);
    
    for (const auto* obj : objects) {
        if (obj) {
            obj->draw();
        }
    }
}

// AROcclusionTracker Implementation
AROcclusionTracker::AROcclusionTracker()
    : currentMethod(OcclusionMethod::BACKGROUND_SUBTRACTION)
    , nextTrackingId(0)
    , minOccluderSize(100)
    , confidenceThreshold(0.5f)
    , maxTrackingAge(30)
    , initialized(false)
{
}

AROcclusionTracker::~AROcclusionTracker() {
    cleanup();
}

bool AROcclusionTracker::initialize(OcclusionMethod method) {
    currentMethod = method;
    
    // Initialize depth estimator
    depthEstimator = std::make_unique<ARDepthEstimator>();
    if (!depthEstimator->initialize()) {
        std::cerr << "Failed to initialize depth estimator" << std::endl;
        return false;
    }
    
    // Initialize background subtractor
    backgroundSubtractor = std::make_unique<ARBackgroundSubtractor>();
    if (!backgroundSubtractor->initialize()) {
        std::cerr << "Failed to initialize background subtractor" << std::endl;
        return false;
    }
    
    initialized = true;
    std::cout << "Occlusion tracker initialized" << std::endl;
    return true;
}

void AROcclusionTracker::cleanup() {
    depthEstimator.reset();
    backgroundSubtractor.reset();
    trackedRegions.clear();
    initialized = false;
}

std::vector<OcclusionRegion> AROcclusionTracker::detectOcclusion(const cv::Mat& frame, const cv::Mat& depthMap) {
    if (!initialized || frame.empty()) {
        return std::vector<OcclusionRegion>();
    }
    
    std::vector<OcclusionRegion> regions;
    
    switch (currentMethod) {
        case OcclusionMethod::BACKGROUND_SUBTRACTION:
            regions = detectByBackgroundSubtraction(frame);
            break;
        case OcclusionMethod::DEPTH_ESTIMATION:
            regions = detectByDepthEstimation(frame, depthMap);
            break;
        case OcclusionMethod::MOTION_DETECTION:
            regions = detectByMotion(frame);
            break;
        default:
            regions = detectByBackgroundSubtraction(frame);
            break;
    }
    
    // Filter and refine regions
    filterRegions(regions);
    
    // Update tracking
    updateTracking(regions);
    
    return trackedRegions;
}

std::vector<OcclusionRegion> AROcclusionTracker::detectByBackgroundSubtraction(const cv::Mat& frame) {
    std::vector<OcclusionRegion> regions;
    
    if (!backgroundSubtractor) return regions;
    
    // Update background model
    backgroundSubtractor->updateBackground(frame);
    
    // Get foreground mask
    cv::Mat fgMask = backgroundSubtractor->getForegroundMask(frame);
    
    // Find contours in foreground mask
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(fgMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < minOccluderSize) continue;
        
        OcclusionRegion region;
        region.boundingBox = cv::boundingRect(contour);
        
        // Create mask for this region
        region.mask = cv::Mat::zeros(fgMask.size(), CV_8UC1);
        cv::fillPoly(region.mask, std::vector<std::vector<cv::Point>>{contour}, cv::Scalar(255));
        
        // Estimate confidence based on area and compactness
        double perimeter = cv::arcLength(contour, true);
        double compactness = (perimeter * perimeter) / area;
        region.confidence = std::min(1.0f, static_cast<float>(area) / 10000.0f) * 
                           std::max(0.1f, 1.0f / static_cast<float>(compactness) * 100.0f);
        
        // Estimate depth (closer to camera assumed)
        region.depth = 1.0f; // Default depth
        
        regions.push_back(region);
    }
    
    return regions;
}

std::vector<OcclusionRegion> AROcclusionTracker::detectByDepthEstimation(const cv::Mat& frame, const cv::Mat& depthMap) {
    std::vector<OcclusionRegion> regions;
    
    cv::Mat depth = depthMap;
    if (depth.empty() && depthEstimator) {
        depth = depthEstimator->estimateDepth(frame);
    }
    
    if (depth.empty()) return regions;
    
    // Find objects that are closer than expected (potential occluders)
    cv::Mat occlusionMask;
    cv::threshold(depth, occlusionMask, 2.0f, 255, cv::THRESH_BINARY_INV);
    occlusionMask.convertTo(occlusionMask, CV_8UC1);
    
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(occlusionMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < minOccluderSize) continue;
        
        OcclusionRegion region;
        region.boundingBox = cv::boundingRect(contour);
        
        // Create mask
        region.mask = cv::Mat::zeros(occlusionMask.size(), CV_8UC1);
        cv::fillPoly(region.mask, std::vector<std::vector<cv::Point>>{contour}, cv::Scalar(255));
        
        // Calculate average depth
        cv::Scalar meanDepth = cv::mean(depth, region.mask);
        region.depth = static_cast<float>(meanDepth[0]);
        
        // Confidence based on depth consistency
        cv::Mat depthROI = depth(region.boundingBox);
        cv::Scalar stdDev;
        cv::meanStdDev(depthROI, cv::Scalar(), stdDev);
        region.confidence = std::max(0.1f, 1.0f - static_cast<float>(stdDev[0]) / 2.0f);
        
        regions.push_back(region);
    }
    
    return regions;
}

std::vector<OcclusionRegion> AROcclusionTracker::detectByMotion(const cv::Mat& frame) {
    std::vector<OcclusionRegion> regions;
    
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    if (previousFrame.empty()) {
        previousFrame = gray.clone();
        return regions;
    }
    
    // Calculate optical flow
    cv::Mat flow;
    cv::calcOpticalFlowPyrLK(previousFrame, gray, std::vector<cv::Point2f>(), 
                            std::vector<cv::Point2f>(), std::vector<uchar>(), 
                            std::vector<float>());
    
    // Simple frame difference for motion detection
    cv::Mat diff;
    cv::absdiff(previousFrame, gray, diff);
    cv::threshold(diff, diff, 30, 255, cv::THRESH_BINARY);
    
    // Morphological operations to clean up
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(diff, diff, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(diff, diff, cv::MORPH_CLOSE, kernel);
    
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(diff, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < minOccluderSize) continue;
        
        OcclusionRegion region;
        region.boundingBox = cv::boundingRect(contour);
        
        // Create mask
        region.mask = cv::Mat::zeros(diff.size(), CV_8UC1);
        cv::fillPoly(region.mask, std::vector<std::vector<cv::Point>>{contour}, cv::Scalar(255));
        
        // Motion-based confidence
        region.confidence = std::min(1.0f, static_cast<float>(area) / 5000.0f);
        region.depth = 1.5f; // Assume medium depth for moving objects
        
        regions.push_back(region);
    }
    
    previousFrame = gray.clone();
    return regions;
}

void AROcclusionTracker::updateTracking(std::vector<OcclusionRegion>& regions) {
    // Associate new regions with existing tracks
    associateRegions(regions);
    
    // Update tracked regions
    trackedRegions = regions;
    
    // Assign new tracking IDs
    for (auto& region : trackedRegions) {
        if (region.trackingId == -1) {
            region.trackingId = nextTrackingId++;
        }
    }
}

void AROcclusionTracker::associateRegions(std::vector<OcclusionRegion>& newRegions) {
    // Simple nearest neighbor association
    for (auto& newRegion : newRegions) {
        float bestSimilarity = 0.0f;
        int bestTrackId = -1;
        
        for (const auto& trackedRegion : trackedRegions) {
            float similarity = calculateRegionSimilarity(newRegion, trackedRegion);
            if (similarity > bestSimilarity && similarity > 0.5f) {
                bestSimilarity = similarity;
                bestTrackId = trackedRegion.trackingId;
            }
        }
        
        newRegion.trackingId = bestTrackId;
    }
}

float AROcclusionTracker::calculateRegionSimilarity(const OcclusionRegion& a, const OcclusionRegion& b) {
    // Calculate overlap ratio
    cv::Rect intersection = a.boundingBox & b.boundingBox;
    float overlapArea = static_cast<float>(intersection.area());
    float unionArea = static_cast<float>(a.boundingBox.area() + b.boundingBox.area() - intersection.area());
    
    if (unionArea == 0) return 0.0f;
    
    float overlapRatio = overlapArea / unionArea;
    
    // Factor in depth similarity
    float depthSimilarity = 1.0f - std::abs(a.depth - b.depth) / 5.0f;
    depthSimilarity = std::max(0.0f, depthSimilarity);
    
    return overlapRatio * 0.7f + depthSimilarity * 0.3f;
}

void AROcclusionTracker::filterRegions(std::vector<OcclusionRegion>& regions) {
    // Remove regions with low confidence
    regions.erase(
        std::remove_if(regions.begin(), regions.end(),
            [this](const OcclusionRegion& region) {
                return region.confidence < confidenceThreshold ||
                       region.boundingBox.area() < minOccluderSize;
            }),
        regions.end()
    );
}

// Utility functions
namespace OcclusionUtils {

GLuint createTextureFromMask(const cv::Mat& mask) {
    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, mask.cols, mask.rows, 0, GL_RED, GL_UNSIGNED_BYTE, mask.data);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    glBindTexture(GL_TEXTURE_2D, 0);
    return texture;
}

cv::Mat normalizeDepthMap(const cv::Mat& depthMap, float minDepth, float maxDepth) {
    cv::Mat normalized;
    depthMap.convertTo(normalized, CV_32F);
    
    normalized = (normalized - minDepth) / (maxDepth - minDepth);
    cv::threshold(normalized, normalized, 1.0, 1.0, cv::THRESH_TRUNC);
    cv::threshold(normalized, normalized, 0.0, 0.0, cv::THRESH_TOZERO);
    
    return normalized;
}

cv::Mat morphologicalCleanup(const cv::Mat& mask) {
    cv::Mat cleaned;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    
    cv::morphologyEx(mask, cleaned, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(cleaned, cleaned, cv::MORPH_CLOSE, kernel);
    
    return cleaned;
}

} // namespace OcclusionUtils
