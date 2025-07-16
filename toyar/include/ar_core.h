
#ifndef AR_CORE_H_
#define AR_CORE_H_

#include "ar_la.h"
#include <vector>
#include <memory>
#include <optional>
#include <functional>
#include <chrono>
#include <stdexcept>
#include <string>

namespace toyar {

// Forward declarations
class Logger;
class ErrorHandler;

/**
 * @brief Exception types for AR operations
 */
class ARException : public std::runtime_error {
public:
    explicit ARException(const std::string& msg) : std::runtime_error(msg) {}
};

class CameraException : public ARException {
public:
    explicit CameraException(const std::string& msg) : ARException("Camera Error: " + msg) {}
};

class TrackingException : public ARException {
public:
    explicit TrackingException(const std::string& msg) : ARException("Tracking Error: " + msg) {}
};

/**
 * @brief Camera intrinsic parameters with validation
 */
struct CameraIntrinsics {
    float fx, fy, cx, cy;
    float k1 = 0.0f, k2 = 0.0f, p1 = 0.0f, p2 = 0.0f; // Distortion coefficients
    
    constexpr CameraIntrinsics() : fx(0), fy(0), cx(0), cy(0) {}
    constexpr CameraIntrinsics(float fx_, float fy_, float cx_, float cy_)
        : fx(fx_), fy(fy_), cx(cx_), cy(cy_) {}
    
    bool isValid() const {
        return fx > EPSILON && fy > EPSILON && cx >= 0 && cy >= 0;
    }
    
    Mat4 getProjectionMatrix(float width, float height, float near_plane, float far_plane) const {
        if (!isValid()) {
            throw CameraException("Invalid camera intrinsics");
        }
        
        // Convert camera intrinsics to OpenGL projection matrix
        const float A = (fx + fy) * 0.5f; // Average focal length
        const float fov_y = 2.0f * std::atan(height / (2.0f * A));
        const float aspect = width / height;
        
        return Mat4::perspectiveRH(fov_y, aspect, near_plane, far_plane);
    }
};

/**
 * @brief RAII wrapper for image data with metadata
 */
class Frame {
public:
    enum class Format { GRAYSCALE, RGB, BGR, RGBA };
    
    Frame() = default;
    Frame(int width, int height, Format format, std::vector<uint8_t> data)
        : width_(width), height_(height), format_(format), data_(std::move(data))
        , timestamp_(std::chrono::steady_clock::now()) {
        validate();
    }
    
    // Move-only semantics for performance
    Frame(const Frame&) = delete;
    Frame& operator=(const Frame&) = delete;
    Frame(Frame&&) = default;
    Frame& operator=(Frame&&) = default;
    
    int width() const { return width_; }
    int height() const { return height_; }
    Format format() const { return format_; }
    const uint8_t* data() const { return data_.data(); }
    size_t size() const { return data_.size(); }
    auto timestamp() const { return timestamp_; }
    
    size_t channels() const {
        switch (format_) {
            case Format::GRAYSCALE: return 1;
            case Format::RGB:
            case Format::BGR: return 3;
            case Format::RGBA: return 4;
            default: return 0;
        }
    }
    
    bool empty() const { return data_.empty(); }

private:
    void validate() const {
        const size_t expected_size = width_ * height_ * channels();
        if (data_.size() != expected_size) {
            throw std::invalid_argument("Frame data size mismatch");
        }
    }
    
    int width_ = 0, height_ = 0;
    Format format_ = Format::GRAYSCALE;
    std::vector<uint8_t> data_;
    std::chrono::steady_clock::time_point timestamp_;
};

/**
 * @brief Tracking state and pose estimation
 */
struct TrackingState {
    enum class Status { UNINITIALIZED, TRACKING, LOST };
    
    Status status = Status::UNINITIALIZED;
    Mat4 pose = Mat4::identity();
    float confidence = 0.0f;
    std::chrono::steady_clock::time_point timestamp;
    
    bool isTracking() const { return status == Status::TRACKING; }
    bool isLost() const { return status == Status::LOST; }
    
    // Get time since last update
    std::chrono::duration<float> getAge() const {
        return std::chrono::steady_clock::now() - timestamp;
    }
};

/**
 * @brief Enhanced tracker with better architecture and error handling
 */
class Tracker {
public:
    struct Config {
        int motion_threshold = 15;
        float max_translation_per_frame = 0.1f;
        float max_rotation_per_frame = 0.1f;
        size_t min_features_for_tracking = 10;
        std::chrono::milliseconds tracking_timeout{5000};
    };
    
    Tracker();
    explicit Tracker(const Config& config);
    ~Tracker() = default;
    
    // Non-copyable but movable
    Tracker(const Tracker&) = delete;
    Tracker& operator=(const Tracker&) = delete;
    Tracker(Tracker&&) = default;
    Tracker& operator=(Tracker&&) = default;
    
    /**
     * @brief Initialize tracker with camera parameters
     * @param width Frame width
     * @param height Frame height
     * @param intrinsics Camera intrinsic parameters
     * @throws CameraException if parameters are invalid
     */
    void initialize(int width, int height, const CameraIntrinsics& intrinsics);
    
    /**
     * @brief Process a new frame and update tracking state
     * @param frame Input frame (must be grayscale)
     * @return Updated tracking state
     * @throws TrackingException if tracking fails critically
     */
    TrackingState track(const Frame& frame);
    
    /**
     * @brief Reset tracking state
     */
    void reset();
    
    /**
     * @brief Get current tracking state
     */
    const TrackingState& getState() const { return state_; }
    
    /**
     * @brief Check if tracker is initialized
     */
    bool isInitialized() const { return initialized_; }

private:
    bool processFrame(const Frame& frame);
    void updatePose(const Vec3& translation, const Vec3& rotation);
    bool validateMotion(const Vec3& translation, const Vec3& rotation) const;
    
    Config config_;
    bool initialized_ = false;
    int width_ = 0, height_ = 0;
    CameraIntrinsics intrinsics_;
    
    std::vector<uint8_t> previous_frame_;
    TrackingState state_;
    
    // Performance tracking
    std::chrono::steady_clock::time_point last_frame_time_;
    float average_processing_time_ = 0.0f;
};

/**
 * @brief Simple logging interface
 */
enum class LogLevel { DEBUG, INFO, WARN, ERROR };

class Logger {
public:
    using LogCallback = std::function<void(LogLevel, const std::string&)>;
    
    static void setCallback(LogCallback callback);
    static void log(LogLevel level, const std::string& message);
    
    template<typename... Args>
    static void debug(const std::string& format, Args&&... args) {
        log(LogLevel::DEBUG, formatString(format, std::forward<Args>(args)...));
    }
    
    template<typename... Args>
    static void info(const std::string& format, Args&&... args) {
        log(LogLevel::INFO, formatString(format, std::forward<Args>(args)...));
    }
    
    template<typename... Args>
    static void warn(const std::string& format, Args&&... args) {
        log(LogLevel::WARN, formatString(format, std::forward<Args>(args)...));
    }
    
    template<typename... Args>
    static void error(const std::string& format, Args&&... args) {
        log(LogLevel::ERROR, formatString(format, std::forward<Args>(args)...));
    }

private:
    template<typename... Args>
    static std::string formatString(const std::string& format, Args&&... args) {
        if constexpr (sizeof...(args) == 0) {
            return format;
        } else {
            size_t size = std::snprintf(nullptr, 0, format.c_str(), args...) + 1;
            std::unique_ptr<char[]> buf(new char[size]);
            std::snprintf(buf.get(), size, format.c_str(), args...);
            return std::string(buf.get(), buf.get() + size - 1);
        }
    }
    
    static LogCallback callback_;
};

} // namespace toyar

#endif // AR_CORE_H_
