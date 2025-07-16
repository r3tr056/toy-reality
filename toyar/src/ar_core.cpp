#include "ar_core.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cstring>

namespace toyar {

// Logger implementation
Logger::LogCallback Logger::callback_ = [](LogLevel level, const std::string& message) {
    const char* level_str = "UNKNOWN";
    switch (level) {
        case LogLevel::DEBUG: level_str = "DEBUG"; break;
        case LogLevel::INFO:  level_str = "INFO";  break;
        case LogLevel::WARN:  level_str = "WARN";  break;
        case LogLevel::ERROR: level_str = "ERROR"; break;
    }
    
    auto time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    
    std::cout << "[" << std::put_time(std::localtime(&time_t), "%H:%M:%S") 
              << "] [" << level_str << "] " << message << std::endl;
};

void Logger::setCallback(LogCallback callback) {
    callback_ = std::move(callback);
}

void Logger::log(LogLevel level, const std::string& message) {
    if (callback_) {
        callback_(level, message);
    }
}

// Tracker implementation
Tracker::Tracker() : Tracker(Config{}) {}

Tracker::Tracker(const Config& config) : config_(config) {
    Logger::info("Tracker created with config: motion_threshold=%d", config_.motion_threshold);
}

void Tracker::initialize(int width, int height, const CameraIntrinsics& intrinsics) {
    if (width <= 0 || height <= 0) {
        throw CameraException("Invalid frame dimensions: " + std::to_string(width) + "x" + std::to_string(height));
    }
    
    if (!intrinsics.isValid()) {
        throw CameraException("Invalid camera intrinsics");
    }
    
    width_ = width;
    height_ = height;
    intrinsics_ = intrinsics;
    
    // Allocate frame buffer
    previous_frame_.resize(width * height);
    
    // Reset state
    reset();
    
    initialized_ = true;
    
    Logger::info("Tracker initialized: %dx%d, fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                 width, height, intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy);
}

TrackingState Tracker::track(const Frame& frame) {
    if (!initialized_) {
        throw TrackingException("Tracker not initialized");
    }
    
    if (frame.width() != width_ || frame.height() != height_) {
        throw TrackingException("Frame size mismatch");
    }
    
    if (frame.format() != Frame::Format::GRAYSCALE) {
        throw TrackingException("Only grayscale frames are supported");
    }
    
    if (frame.empty()) {
        Logger::warn("Received empty frame");
        state_.status = TrackingState::Status::LOST;
        return state_;
    }
    
    auto start_time = std::chrono::steady_clock::now();
    
    try {
        bool success = processFrame(frame);
        
        if (success) {
            state_.status = TrackingState::Status::TRACKING;
            state_.confidence = std::min(1.0f, state_.confidence + 0.1f);
        } else {
            state_.confidence = std::max(0.0f, state_.confidence - 0.2f);
            if (state_.confidence < 0.3f) {
                state_.status = TrackingState::Status::LOST;
                Logger::warn("Tracking lost due to low confidence");
            }
        }
        
        state_.timestamp = std::chrono::steady_clock::now();
        
        // Update performance metrics
        auto processing_time = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - start_time).count();
        average_processing_time_ = 0.9f * average_processing_time_ + 0.1f * processing_time;
        
        if (processing_time > 16.0f) { // More than one frame at 60fps
            Logger::warn("Slow tracking frame: %.2fms", processing_time);
        }
        
    } catch (const std::exception& e) {
        Logger::error("Tracking error: %s", e.what());
        state_.status = TrackingState::Status::LOST;
        state_.confidence = 0.0f;
    }
    
    return state_;
}

bool Tracker::processFrame(const Frame& frame) {
    const uint8_t* current_data = frame.data();
    
    // First frame - just store it
    if (state_.status == TrackingState::Status::UNINITIALIZED) {
        std::memcpy(previous_frame_.data(), current_data, previous_frame_.size());
        state_.status = TrackingState::Status::TRACKING;
        state_.confidence = 0.5f;
        state_.pose = Mat4::identity();
        return true;
    }
    
    // Simple motion detection based on pixel differences
    int64_t sum_x = 0, sum_y = 0, diff_count = 0;
    const uint8_t* prev_data = previous_frame_.data();
    
    // Process frame with some optimization
    for (int y = 1; y < height_ - 1; ++y) {
        for (int x = 1; x < width_ - 1; ++x) {
            const int idx = y * width_ + x;
            const int diff = static_cast<int>(current_data[idx]) - static_cast<int>(prev_data[idx]);
            
            if (std::abs(diff) > config_.motion_threshold) {
                sum_x += x;
                sum_y += y;
                ++diff_count;
            }
        }
    }
    
    // Update previous frame
    std::memcpy(previous_frame_.data(), current_data, previous_frame_.size());
    
    if (static_cast<size_t>(diff_count) < config_.min_features_for_tracking) {
        Logger::debug("Insufficient features for tracking: %ld", diff_count);
        return false;
    }
    
    // Calculate centroid of motion
    const float centroid_x = static_cast<float>(sum_x) / diff_count;
    const float centroid_y = static_cast<float>(sum_y) / diff_count;
    
    // Convert pixel motion to camera space
    const float dx = (centroid_x - intrinsics_.cx) / intrinsics_.fx;
    const float dy = (centroid_y - intrinsics_.cy) / intrinsics_.fy;
    
    // Estimate translation (assuming constant depth for now)
    constexpr float estimated_depth = 1.0f;
    const Vec3 translation{-dx * estimated_depth * 0.01f, dy * estimated_depth * 0.01f, 0.0f};
    
    // Simple rotation estimation (this is very basic)
    const Vec3 rotation{0.0f, 0.0f, 0.0f};
    
    // Validate motion
    if (!validateMotion(translation, rotation)) {
        Logger::debug("Motion validation failed");
        return false;
    }
    
    // Update pose
    updatePose(translation, rotation);
    
    Logger::debug("Motion detected: features=%ld, translation=(%.3f,%.3f,%.3f)", 
                  diff_count, translation.x, translation.y, translation.z);
    
    return true;
}

void Tracker::updatePose(const Vec3& translation, const Vec3& rotation) {
    // Apply incremental transformation
    const Mat4 delta_translation = Mat4::translation(translation);
    const Mat4 delta_rotation = Mat4::rotationXYZ(rotation.x, rotation.y, rotation.z);
    const Mat4 delta_transform = delta_translation * delta_rotation;
    
    state_.pose = delta_transform * state_.pose;
}

bool Tracker::validateMotion(const Vec3& translation, const Vec3& rotation) const {
    // Check translation bounds
    if (translation.length() > config_.max_translation_per_frame) {
        Logger::debug("Translation too large: %.3f > %.3f", 
                      translation.length(), config_.max_translation_per_frame);
        return false;
    }
    
    // Check rotation bounds
    if (rotation.length() > config_.max_rotation_per_frame) {
        Logger::debug("Rotation too large: %.3f > %.3f", 
                      rotation.length(), config_.max_rotation_per_frame);
        return false;
    }
    
    return true;
}

void Tracker::reset() {
    state_.status = TrackingState::Status::UNINITIALIZED;
    state_.pose = Mat4::identity();
    state_.confidence = 0.0f;
    state_.timestamp = std::chrono::steady_clock::now();
    
    if (!previous_frame_.empty()) {
        std::fill(previous_frame_.begin(), previous_frame_.end(), 0);
    }
    
    average_processing_time_ = 0.0f;
    
    Logger::info("Tracker reset");
}

} // namespace toyar
