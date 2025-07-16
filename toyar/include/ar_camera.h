#ifndef AR_CAMERA_H_
#define AR_CAMERA_H_

#include "ar_core.h"
#include "ar_la.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <queue>
#include <functional>
#include <GL/glew.h>

namespace toyar {

/**
 * @brief RAII OpenGL texture wrapper
 */
class GLTexture {
public:
    GLTexture(int width, int height, GLenum internal_format = GL_RGB8);
    ~GLTexture();
    
    // Non-copyable but movable
    GLTexture(const GLTexture&) = delete;
    GLTexture& operator=(const GLTexture&) = delete;
    GLTexture(GLTexture&& other) noexcept;
    GLTexture& operator=(GLTexture&& other) noexcept;
    
    void upload(const void* data, GLenum format, GLenum type);
    void bind(unsigned int unit = 0) const;
    void unbind() const;
    
    GLuint getId() const { return texture_id_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    
    void renderFullscreen() const;

private:
    void destroy();
    
    GLuint texture_id_ = 0;
    int width_ = 0, height_ = 0;
    GLenum internal_format_ = GL_RGB8;
};

/**
 * @brief ArUco marker detection result
 */
struct MarkerDetection {
    int id;
    std::vector<cv::Point2f> corners;
    Mat4 pose; // Marker pose in camera space
    float confidence = 0.0f;
    
    bool isValid() const { return id >= 0 && corners.size() == 4; }
};

/**
 * @brief Camera capture and tracking
 */
class ARCamera {
public:
    struct Config {
        int camera_index = 0;
        int desired_width = 640;
        int desired_height = 480;
        double desired_fps = 30.0;
        float marker_size = 0.05f; // 5cm markers by default
        bool enable_async_capture = true;
        size_t frame_queue_size = 3;
        
        // ArUco parameters
        cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name = cv::aruco::DICT_6X6_250;
        
        // Camera calibration file path (optional)
        std::string calibration_file;
    };
    
    using FrameCallback = std::function<void(const Frame&)>;
    using MarkerCallback = std::function<void(const std::vector<MarkerDetection>&)>;
    
    ARCamera();
    explicit ARCamera(const Config& config);
    ~ARCamera();
    
    // Non-copyable but movable
    ARCamera(const ARCamera&) = delete;
    ARCamera& operator=(const ARCamera&) = delete;
    ARCamera(ARCamera&&) = default;
    ARCamera& operator=(ARCamera&&) = default;
    
    /**
     * @brief Initialize and open camera
     * @throws CameraException if initialization fails
     */
    void initialize();
    
    /**
     * @brief Start capturing frames
     */
    void start();
    
    /**
     * @brief Stop capturing frames
     */
    void stop();
    
    /**
     * @brief Check if camera is currently running
     */
    bool isRunning() const { return running_; }
    
    /**
     * @brief Get the latest frame (blocking)
     * @param timeout_ms Maximum time to wait for a frame
     * @return Frame if available, empty frame otherwise
     */
    Frame getFrame(int timeout_ms = 100);
    
    /**
     * @brief Set callback for new frames
     */
    void setFrameCallback(FrameCallback callback) { frame_callback_ = std::move(callback); }
    
    /**
     * @brief Set callback for marker detections
     */
    void setMarkerCallback(MarkerCallback callback) { marker_callback_ = std::move(callback); }
    
    /**
     * @brief Detect ArUco markers in frame
     */
    std::vector<MarkerDetection> detectMarkers(const Frame& frame) const;
    
    /**
     * @brief Get camera intrinsics
     */
    const CameraIntrinsics& getIntrinsics() const { return intrinsics_; }
    
    /**
     * @brief Load camera calibration from file
     * @param filename Path to calibration file (OpenCV format)
     * @return true if successful
     */
    bool loadCalibration(const std::string& filename);
    
    /**
     * @brief Get camera pose in world space
     */
    const Mat4& getPose() const { return pose_; }
    
    /**
     * @brief Set camera pose in world space
     */
    void setPose(const Mat4& pose) { pose_ = pose; }
    
    /**
     * @brief Get view matrix (inverse of pose)
     */
    Mat4 getViewMatrix() const;
    
    /**
     * @brief Get projection matrix
     */
    Mat4 getProjectionMatrix(float near_plane = 0.1f, float far_plane = 100.0f) const;
    
    /**
     * @brief Create texture for camera background rendering
     */
    std::unique_ptr<GLTexture> createBackgroundTexture() const;
    
    /**
     * @brief Update background texture with latest frame
     */
    void updateBackgroundTexture(GLTexture& texture, const Frame& frame) const;

private:
    void captureLoop();
    void processFrame(cv::Mat& cv_frame);
    Frame convertToFrame(const cv::Mat& cv_frame) const;
    bool initializeCapture();
    void destroyCapture();
    
    Config config_;
    CameraIntrinsics intrinsics_;
    Mat4 pose_ = Mat4::identity();
    
    // OpenCV components
    cv::VideoCapture capture_;
    cv::Mat camera_matrix_;
    cv::Mat distortion_coeffs_;
    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    
    // Threading for async capture
    std::atomic<bool> running_{false};
    std::atomic<bool> should_stop_{false};
    std::thread capture_thread_;
    
    // Frame queue for async operation
    mutable std::mutex frame_queue_mutex_;
    std::condition_variable frame_available_;
    std::queue<Frame> frame_queue_;
    
    // Callbacks
    FrameCallback frame_callback_;
    MarkerCallback marker_callback_;
    
    // Performance metrics
    std::atomic<size_t> frames_captured_{0};
    std::atomic<size_t> frames_dropped_{0};
    std::chrono::steady_clock::time_point last_stats_time_;
};

} // namespace toyar

#endif // AR_CAMERA_H_
