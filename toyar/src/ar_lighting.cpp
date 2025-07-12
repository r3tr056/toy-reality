#include "ar_lighting.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// ARLightingEstimator Implementation
ARLightingEstimator::ARLightingEstimator()
    : autoEstimationEnabled(true)
    , isInitialized(false)
{
    estimateHistory.reserve(MAX_HISTORY_SIZE);
}

ARLightingEstimator::~ARLightingEstimator() {
    cleanup();
}

bool ARLightingEstimator::initialize() {
    if (isInitialized) return true;
    
    // Initialize OpenCV processing matrices
    grayFrame = cv::Mat();
    blurredFrame = cv::Mat();
    gradientFrame = cv::Mat();
    
    // Set default light estimate
    currentEstimate = LightEstimate();
    previousEstimate = currentEstimate;
    
    isInitialized = true;
    
    std::cout << "Light estimator initialized successfully" << std::endl;
    return true;
}

void ARLightingEstimator::cleanup() {
    estimateHistory.clear();
    isInitialized = false;
}

LightEstimate ARLightingEstimator::estimateLight(const cv::Mat& frame, const Mat4& cameraTransform) {
    if (!isInitialized || frame.empty()) {
        return currentEstimate;
    }
    
    if (!autoEstimationEnabled) {
        return currentEstimate;
    }
    
    // Preprocess frame for analysis
    cv::Mat processedFrame = preprocessFrame(frame);
    
    // Analyze ambient lighting
    float ambientIntensity = analyzeAmbientIntensity(processedFrame);
    
    // Analyze color temperature
    Vec3 ambientColor = analyzeColorTemperature(processedFrame);
    
    // Estimate directional light
    Vec3 directionalDir = estimateDirectionalLight(processedFrame);
    
    // Calculate confidence
    float confidence = calculateConfidence(processedFrame);
    
    // Create new estimate
    LightEstimate newEstimate;
    newEstimate.ambientIntensity = ambientIntensity;
    newEstimate.ambientColor = ambientColor;
    newEstimate.directionalLightDir = directionalDir;
    newEstimate.directionalLightColor = Vec3(1.0f, 1.0f, 1.0f); // Assume white light
    newEstimate.directionalIntensity = std::max(0.1f, 1.0f - ambientIntensity);
    newEstimate.colorTemperature = LightingUtils::rgbToKelvin(ambientColor);
    newEstimate.confidence = confidence;
    
    // Add to history for temporal smoothing
    estimateHistory.push_back(newEstimate);
    if (estimateHistory.size() > MAX_HISTORY_SIZE) {
        estimateHistory.erase(estimateHistory.begin());
    }
    
    // Apply temporal smoothing
    if (estimateHistory.size() > 5) {
        currentEstimate = LightingUtils::smoothLightEstimate(estimateHistory);
    } else {
        currentEstimate = newEstimate;
    }
    
    previousEstimate = currentEstimate;
    
    return currentEstimate;
}

cv::Mat ARLightingEstimator::preprocessFrame(const cv::Mat& frame) {
    cv::Mat processed;
    
    // Convert to grayscale if needed
    if (frame.channels() == 3) {
        cv::cvtColor(frame, grayFrame, cv::COLOR_BGR2GRAY);
    } else {
        grayFrame = frame.clone();
    }
    
    // Apply Gaussian blur to reduce noise
    cv::GaussianBlur(grayFrame, blurredFrame, cv::Size(5, 5), 1.0);
    
    return blurredFrame;
}

float ARLightingEstimator::analyzeAmbientIntensity(const cv::Mat& frame) {
    if (frame.empty()) return 0.3f;
    
    // Calculate mean intensity
    cv::Scalar meanIntensity = cv::mean(frame);
    float intensity = meanIntensity[0] / 255.0f;
    
    // Calculate histogram for better analysis
    std::vector<cv::Mat> channels;
    cv::split(frame, channels);
    
    cv::Mat hist;
    int histSize = 256;
    float range[] = {0, 256};
    const float* histRange = {range};
    
    cv::calcHist(&channels[0], 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);
    
    // Find the mode (most common intensity)
    int maxBin = 0;
    float maxVal = 0;
    for (int i = 0; i < histSize; i++) {
        float val = hist.at<float>(i);
        if (val > maxVal) {
            maxVal = val;
            maxBin = i;
        }
    }
    
    float modeIntensity = maxBin / 255.0f;
    
    // Combine mean and mode with weights
    float ambientIntensity = (intensity * 0.7f + modeIntensity * 0.3f);
    
    // Apply realistic bounds
    return std::clamp(ambientIntensity, 0.05f, 0.95f);
}

Vec3 ARLightingEstimator::analyzeColorTemperature(const cv::Mat& frame) {
    if (frame.empty() || frame.channels() < 3) {
        return Vec3(0.4f, 0.4f, 0.4f);
    }
    
    // Calculate average color
    cv::Scalar avgColor = cv::mean(frame);
    
    // Normalize to [0,1] range
    Vec3 color(
        avgColor[2] / 255.0f,  // OpenCV uses BGR
        avgColor[1] / 255.0f,
        avgColor[0] / 255.0f
    );
    
    // Apply white balance correction
    float maxChannel = std::max({color.x, color.y, color.z});
    if (maxChannel > 0.01f) {
        color.x /= maxChannel;
        color.y /= maxChannel;
        color.z /= maxChannel;
    }
    
    // Apply realistic ambient color bounds
    color.x = std::clamp(color.x, 0.2f, 1.0f);
    color.y = std::clamp(color.y, 0.2f, 1.0f);
    color.z = std::clamp(color.z, 0.2f, 1.0f);
    
    return color;
}

Vec3 ARLightingEstimator::estimateDirectionalLight(const cv::Mat& frame) {
    if (frame.empty()) {
        return Vec3(0.0f, -1.0f, 0.0f); // Default: light from above
    }
    
    // Calculate gradient to find dominant light direction
    cv::Mat gradX, gradY;
    cv::Sobel(frame, gradX, CV_32F, 1, 0, 3);
    cv::Sobel(frame, gradY, CV_32F, 0, 1, 3);
    
    // Calculate average gradient direction
    cv::Scalar avgGradX = cv::mean(gradX);
    cv::Scalar avgGradY = cv::mean(gradY);
    
    // Convert gradient to light direction (opposite of shadow direction)
    Vec3 lightDir(
        -static_cast<float>(avgGradX[0]),
        -static_cast<float>(avgGradY[0]),
        -1.0f  // Assume light is coming from above the scene
    );
    
    // Normalize
    float length = std::sqrt(lightDir.x * lightDir.x + lightDir.y * lightDir.y + lightDir.z * lightDir.z);
    if (length > 0.001f) {
        lightDir.x /= length;
        lightDir.y /= length;
        lightDir.z /= length;
    } else {
        lightDir = Vec3(0.0f, -1.0f, 0.0f);
    }
    
    return lightDir;
}

float ARLightingEstimator::calculateConfidence(const cv::Mat& frame) {
    if (frame.empty()) return 0.5f;
    
    // Base confidence on image quality metrics
    cv::Mat laplacian;
    cv::Laplacian(frame, laplacian, CV_64F);
    
    cv::Scalar mean, stddev;
    cv::meanStdDev(laplacian, mean, stddev);
    
    float sharpness = stddev[0] * stddev[0];
    float confidence = std::min(1.0f, sharpness / 1000.0f);
    
    // Ensure minimum confidence
    return std::max(0.1f, confidence);
}

void ARLightingEstimator::setManualAmbient(const Vec3& color, float intensity) {
    currentEstimate.ambientColor = color;
    currentEstimate.ambientIntensity = intensity;
    autoEstimationEnabled = false;
}

void ARLightingEstimator::setManualDirectional(const Vec3& direction, const Vec3& color, float intensity) {
    currentEstimate.directionalLightDir = direction;
    currentEstimate.directionalLightColor = color;
    currentEstimate.directionalIntensity = intensity;
    autoEstimationEnabled = false;
}

// ARShadowRenderer Implementation
ARShadowRenderer::ARShadowRenderer()
    : shadowMapFBO(0)
    , shadowMapTexture(0)
    , shadowMapDepthTexture(0)
    , shadowVertexShader(0)
    , shadowFragmentShader(0)
    , shadowShaderProgram(0)
    , isInitialized(false)
{
}

ARShadowRenderer::~ARShadowRenderer() {
    cleanup();
}

bool ARShadowRenderer::initialize(const ShadowMapConfig& config) {
    this->config = config;
    
    if (!createShadowMapFramebuffer()) {
        std::cerr << "Failed to create shadow map framebuffer" << std::endl;
        return false;
    }
    
    setupShadowShaders();
    
    isInitialized = true;
    std::cout << "Shadow renderer initialized with " << config.shadowMapSize << "x" << config.shadowMapSize << " shadow map" << std::endl;
    
    return true;
}

void ARShadowRenderer::cleanup() {
    if (shadowMapFBO) glDeleteFramebuffers(1, &shadowMapFBO);
    if (shadowMapTexture) glDeleteTextures(1, &shadowMapTexture);
    if (shadowMapDepthTexture) glDeleteTextures(1, &shadowMapDepthTexture);
    if (shadowShaderProgram) glDeleteProgram(shadowShaderProgram);
    if (shadowVertexShader) glDeleteShader(shadowVertexShader);
    if (shadowFragmentShader) glDeleteShader(shadowFragmentShader);
    
    shadowMapFBO = shadowMapTexture = shadowMapDepthTexture = 0;
    shadowVertexShader = shadowFragmentShader = shadowShaderProgram = 0;
    isInitialized = false;
}

bool ARShadowRenderer::createShadowMapFramebuffer() {
    // Generate framebuffer
    glGenFramebuffers(1, &shadowMapFBO);
    glBindFramebuffer(GL_FRAMEBUFFER, shadowMapFBO);
    
    // Create depth texture for shadow map
    glGenTextures(1, &shadowMapDepthTexture);
    glBindTexture(GL_TEXTURE_2D, shadowMapDepthTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, 
                 config.shadowMapSize, config.shadowMapSize, 0, 
                 GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    
    float borderColor[] = {1.0f, 1.0f, 1.0f, 1.0f};
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
    
    // Attach depth texture to framebuffer
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, shadowMapDepthTexture, 0);
    
    // No color buffer needed for shadow mapping
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    
    // Check framebuffer completeness
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cerr << "Shadow map framebuffer is not complete!" << std::endl;
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return false;
    }
    
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return true;
}

void ARShadowRenderer::setupShadowShaders() {
    // Shadow mapping vertex shader
    const char* shadowVertSource = R"(
#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 lightSpaceMatrix;
uniform mat4 model;

void main() {
    gl_Position = lightSpaceMatrix * model * vec4(aPos, 1.0);
}
)";
    
    // Shadow mapping fragment shader
    const char* shadowFragSource = R"(
#version 330 core

void main() {
    // OpenGL automatically writes depth
}
)";
    
    // Compile shaders
    shadowVertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(shadowVertexShader, 1, &shadowVertSource, nullptr);
    glCompileShader(shadowVertexShader);
    
    shadowFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(shadowFragmentShader, 1, &shadowFragSource, nullptr);
    glCompileShader(shadowFragmentShader);
    
    // Create program
    shadowShaderProgram = glCreateProgram();
    glAttachShader(shadowShaderProgram, shadowVertexShader);
    glAttachShader(shadowShaderProgram, shadowFragmentShader);
    glLinkProgram(shadowShaderProgram);
    
    // Check for compilation/linking errors
    GLint success;
    glGetProgramiv(shadowShaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetProgramInfoLog(shadowShaderProgram, 512, nullptr, infoLog);
        std::cerr << "Shadow shader linking failed: " << infoLog << std::endl;
    }
}

void ARShadowRenderer::beginShadowMapGeneration(const LightEstimate& lightEst, const Vec3& sceneCenter) {
    if (!isInitialized) return;
    
    // Calculate light matrices
    lightViewMatrix = calculateLightViewMatrix(lightEst.directionalLightDir, sceneCenter);
    lightProjMatrix = calculateLightProjectionMatrix();
    
    // Bind shadow map framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, shadowMapFBO);
    glViewport(0, 0, config.shadowMapSize, config.shadowMapSize);
    
    // Clear depth buffer
    glClear(GL_DEPTH_BUFFER_BIT);
    
    // Enable depth testing
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    
    // Use shadow shader
    glUseProgram(shadowShaderProgram);
    
    // Set light space matrix
    Mat4 lightSpaceMatrix = lightProjMatrix * lightViewMatrix;
    GLint lightSpaceLocation = glGetUniformLocation(shadowShaderProgram, "lightSpaceMatrix");
    glUniformMatrix4fv(lightSpaceLocation, 1, GL_FALSE, lightSpaceMatrix.data());
}

void ARShadowRenderer::endShadowMapGeneration() {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glUseProgram(0);
}

Mat4 ARShadowRenderer::calculateLightViewMatrix(const Vec3& lightDir, const Vec3& sceneCenter) {
    // Position light far from scene center
    Vec3 lightPos = sceneCenter - lightDir * config.lightFarPlane * 0.5f;
    
    // Create view matrix looking towards scene center
    glm::vec3 lightPosition(lightPos.x, lightPos.y, lightPos.z);
    glm::vec3 target(sceneCenter.x, sceneCenter.y, sceneCenter.z);
    glm::vec3 up(0.0f, 1.0f, 0.0f);
    
    // If light direction is too close to up vector, choose different up
    if (std::abs(lightDir.y) > 0.9f) {
        up = glm::vec3(1.0f, 0.0f, 0.0f);
    }
    
    glm::mat4 lightView = glm::lookAt(lightPosition, target, up);
    
    Mat4 result;
    std::memcpy(result.data(), glm::value_ptr(lightView), 16 * sizeof(float));
    
    return result;
}

Mat4 ARShadowRenderer::calculateLightProjectionMatrix() {
    float size = config.lightFrustumSize;
    glm::mat4 lightProj = glm::ortho(-size, size, -size, size, 
                                    config.lightNearPlane, config.lightFarPlane);
    
    Mat4 result;
    std::memcpy(result.data(), glm::value_ptr(lightProj), 16 * sizeof(float));
    
    return result;
}

// Lighting utility functions
namespace LightingUtils {

Vec3 kelvinToRGB(float kelvin) {
    // Simplified Kelvin to RGB conversion
    kelvin = std::clamp(kelvin, 1000.0f, 12000.0f);
    kelvin /= 100.0f;
    
    float red, green, blue;
    
    // Calculate red
    if (kelvin <= 66) {
        red = 255;
    } else {
        red = kelvin - 60;
        red = 329.698727446 * std::pow(red, -0.1332047592);
        red = std::clamp(red, 0.0f, 255.0f);
    }
    
    // Calculate green
    if (kelvin <= 66) {
        green = kelvin;
        green = 99.4708025861 * std::log(green) - 161.1195681661;
    } else {
        green = kelvin - 60;
        green = 288.1221695283 * std::pow(green, -0.0755148492);
    }
    green = std::clamp(green, 0.0f, 255.0f);
    
    // Calculate blue
    if (kelvin >= 66) {
        blue = 255;
    } else if (kelvin <= 19) {
        blue = 0;
    } else {
        blue = kelvin - 10;
        blue = 138.5177312231 * std::log(blue) - 305.0447927307;
        blue = std::clamp(blue, 0.0f, 255.0f);
    }
    
    return Vec3(red / 255.0f, green / 255.0f, blue / 255.0f);
}

float rgbToKelvin(const Vec3& rgb) {
    // Simplified RGB to Kelvin conversion
    // This is an approximation
    float r = rgb.x;
    float g = rgb.y;
    float b = rgb.z;
    
    float temperature;
    
    if (r > g && r > b) {
        // Warm light (reddish)
        temperature = 2000.0f + (1.0f - r) * 2500.0f;
    } else if (b > r && b > g) {
        // Cool light (bluish)
        temperature = 5500.0f + b * 4000.0f;
    } else {
        // Neutral light
        temperature = 5500.0f;
    }
    
    return std::clamp(temperature, 2000.0f, 12000.0f);
}

LightEstimate smoothLightEstimate(const std::vector<LightEstimate>& history) {
    if (history.empty()) return LightEstimate();
    if (history.size() == 1) return history[0];
    
    LightEstimate smoothed;
    float totalWeight = 0.0f;
    
    // Exponential moving average with more weight on recent estimates
    for (size_t i = 0; i < history.size(); ++i) {
        float weight = std::pow(0.8f, history.size() - 1 - i);
        totalWeight += weight;
        
        smoothed.ambientColor.x += history[i].ambientColor.x * weight;
        smoothed.ambientColor.y += history[i].ambientColor.y * weight;
        smoothed.ambientColor.z += history[i].ambientColor.z * weight;
        smoothed.ambientIntensity += history[i].ambientIntensity * weight;
        
        smoothed.directionalLightDir.x += history[i].directionalLightDir.x * weight;
        smoothed.directionalLightDir.y += history[i].directionalLightDir.y * weight;
        smoothed.directionalLightDir.z += history[i].directionalLightDir.z * weight;
        smoothed.directionalIntensity += history[i].directionalIntensity * weight;
        
        smoothed.colorTemperature += history[i].colorTemperature * weight;
        smoothed.confidence += history[i].confidence * weight;
    }
    
    if (totalWeight > 0) {
        smoothed.ambientColor.x /= totalWeight;
        smoothed.ambientColor.y /= totalWeight;
        smoothed.ambientColor.z /= totalWeight;
        smoothed.ambientIntensity /= totalWeight;
        
        smoothed.directionalLightDir.x /= totalWeight;
        smoothed.directionalLightDir.y /= totalWeight;
        smoothed.directionalLightDir.z /= totalWeight;
        smoothed.directionalIntensity /= totalWeight;
        
        smoothed.colorTemperature /= totalWeight;
        smoothed.confidence /= totalWeight;
    }
    
    // Normalize directional light direction
    float length = std::sqrt(smoothed.directionalLightDir.x * smoothed.directionalLightDir.x +
                           smoothed.directionalLightDir.y * smoothed.directionalLightDir.y +
                           smoothed.directionalLightDir.z * smoothed.directionalLightDir.z);
    if (length > 0.001f) {
        smoothed.directionalLightDir.x /= length;
        smoothed.directionalLightDir.y /= length;
        smoothed.directionalLightDir.z /= length;
    }
    
    smoothed.directionalLightColor = Vec3(1.0f, 1.0f, 1.0f);
    
    return smoothed;
}

Mat4 createShadowMatrix(const Mat4& lightView, const Mat4& lightProj) {
    // Bias matrix to transform from [-1,1] to [0,1]
    Mat4 biasMatrix;
    biasMatrix.m[0] = 0.5f; biasMatrix.m[4] = 0.0f; biasMatrix.m[8]  = 0.0f; biasMatrix.m[12] = 0.5f;
    biasMatrix.m[1] = 0.0f; biasMatrix.m[5] = 0.5f; biasMatrix.m[9]  = 0.0f; biasMatrix.m[13] = 0.5f;
    biasMatrix.m[2] = 0.0f; biasMatrix.m[6] = 0.0f; biasMatrix.m[10] = 0.5f; biasMatrix.m[14] = 0.5f;
    biasMatrix.m[3] = 0.0f; biasMatrix.m[7] = 0.0f; biasMatrix.m[11] = 0.0f; biasMatrix.m[15] = 1.0f;
    
    return biasMatrix * lightProj * lightView;
}

} // namespace LightingUtils
