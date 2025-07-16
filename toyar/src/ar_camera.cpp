#include "ar_camera.h"
#include "ar_error.h"
#include <iostream>
#include <fstream>
#include <GL/glew.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace toyar {

// GLTexture implementation
GLTexture::GLTexture(int width, int height, GLenum internal_format)
    : width_(width), height_(height), internal_format_(internal_format) {
    TOYAR_GL_CHECK(glGenTextures(1, &texture_id_));
    TOYAR_GL_CHECK(glBindTexture(GL_TEXTURE_2D, texture_id_));
    TOYAR_GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
    TOYAR_GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    TOYAR_GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE));
    TOYAR_GL_CHECK(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE));
    
    // Allocate texture storage
    TOYAR_GL_CHECK(glTexImage2D(GL_TEXTURE_2D, 0, internal_format_, width_, height_, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr));
}

GLTexture::~GLTexture() {
    destroy();
}

GLTexture::GLTexture(GLTexture&& other) noexcept
    : texture_id_(other.texture_id_), width_(other.width_), height_(other.height_), 
      internal_format_(other.internal_format_) {
    other.texture_id_ = 0;
}

GLTexture& GLTexture::operator=(GLTexture&& other) noexcept {
    if (this != &other) {
        destroy();
        texture_id_ = other.texture_id_;
        width_ = other.width_;
        height_ = other.height_;
        internal_format_ = other.internal_format_;
        other.texture_id_ = 0;
    }
    return *this;
}

void GLTexture::destroy() {
    if (texture_id_ != 0) {
        glDeleteTextures(1, &texture_id_);
        texture_id_ = 0;
    }
}

void GLTexture::upload(const void* data, GLenum format, GLenum type) {
    TOYAR_GL_CHECK(glBindTexture(GL_TEXTURE_2D, texture_id_));
    TOYAR_GL_CHECK(glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width_, height_, format, type, data));
}

void GLTexture::bind(unsigned int unit) const {
    TOYAR_GL_CHECK(glActiveTexture(GL_TEXTURE0 + unit));
    TOYAR_GL_CHECK(glBindTexture(GL_TEXTURE_2D, texture_id_));
}

void GLTexture::unbind() const {
    TOYAR_GL_CHECK(glBindTexture(GL_TEXTURE_2D, 0));
}

void GLTexture::renderFullscreen() const {
    bind();
    
    // Render fullscreen quad using immediate mode (for compatibility)
    TOYAR_GL_CHECK(glBegin(GL_QUADS));
        glTexCoord2f(0.0f, 1.0f); glVertex2f(-1.0f, -1.0f);
        glTexCoord2f(1.0f, 1.0f); glVertex2f(1.0f, -1.0f);
        glTexCoord2f(1.0f, 0.0f); glVertex2f(1.0f, 1.0f);
        glTexCoord2f(0.0f, 0.0f); glVertex2f(-1.0f, 1.0f);
    TOYAR_GL_CHECK(glEnd());
    
    unbind();
}

// ARCamera implementation
ARCamera::ARCamera() : ARCamera(Config{}) {}

ARCamera::ARCamera(const Config& config) : config_(config) {
    TOYAR_INFO("Camera", "Creating ARCamera with config");
}

ARCamera::~ARCamera() {
    stop();
    destroyCapture();
}

void ARCamera::initialize() {
    if (!initializeCapture()) {
        throw CameraException("Failed to initialize camera capture");
    }
    
    // Set up default intrinsics
    intrinsics_.fx = config_.desired_width * 0.7f;
    intrinsics_.fy = config_.desired_height * 0.7f;
    intrinsics_.cx = config_.desired_width * 0.5f;
    intrinsics_.cy = config_.desired_height * 0.5f;
    
    // Initialize ArUco detection
    aruco_dict_ = cv::aruco::getPredefinedDictionary(config_.dictionary_name);
    detector_params_ = cv::aruco::DetectorParameters::create();
}

bool ARCamera::initializeCapture() {
    try {
        capture_.open(config_.camera_index);
        if (!capture_.isOpened()) {
            return false;
        }
        
        capture_.set(cv::CAP_PROP_FRAME_WIDTH, config_.desired_width);
        capture_.set(cv::CAP_PROP_FRAME_HEIGHT, config_.desired_height);
        capture_.set(cv::CAP_PROP_FPS, config_.desired_fps);
        
        return true;
        
    } catch (const cv::Exception& e) {
        return false;
    }
}

void ARCamera::destroyCapture() {
    if (capture_.isOpened()) {
        capture_.release();
    }
}

void ARCamera::start() {
    if (running_) {
        return;
    }
    
    should_stop_ = false;
    running_ = true;
    
    if (config_.enable_async_capture) {
        capture_thread_ = std::thread(&ARCamera::captureLoop, this);
    }
}

void ARCamera::stop() {
    if (!running_) {
        return;
    }
    
    should_stop_ = true;
    running_ = false;
    
    if (capture_thread_.joinable()) {
        frame_available_.notify_all();
        capture_thread_.join();
    }
}

void ARCamera::captureLoop() {
    cv::Mat cv_frame;
    
    while (!should_stop_) {
        try {
            if (!capture_.read(cv_frame)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            
            if (cv_frame.empty()) {
                continue;
            }
            
            processFrame(cv_frame);
            frames_captured_++;
            
        } catch (const std::exception& e) {
            // Log error
        }
    }
}

void ARCamera::processFrame(cv::Mat& cv_frame) {
    Frame frame = convertToFrame(cv_frame);
    
    {
        std::lock_guard<std::mutex> lock(frame_queue_mutex_);
        if (frame_queue_.size() >= config_.frame_queue_size) {
            frame_queue_.pop();
            frames_dropped_++;
        }
        frame_queue_.emplace(std::move(frame));
    }
    frame_available_.notify_one();
    
    if (frame_callback_) {
        frame_callback_(frame_queue_.back());
    }
    
    if (marker_callback_) {
        auto markers = detectMarkers(frame_queue_.back());
        if (!markers.empty()) {
            marker_callback_(markers);
        }
    }
}

Frame ARCamera::convertToFrame(const cv::Mat& cv_frame) const {
    cv::Mat rgb_frame;
    cv::cvtColor(cv_frame, rgb_frame, cv::COLOR_BGR2RGB);
    
    std::vector<uint8_t> data(rgb_frame.data, rgb_frame.data + rgb_frame.total() * rgb_frame.channels());
    
    return Frame(rgb_frame.cols, rgb_frame.rows, Frame::Format::RGB, std::move(data));
}

Frame ARCamera::getFrame(int timeout_ms) {
    std::unique_lock<std::mutex> lock(frame_queue_mutex_);
    
    if (frame_available_.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
                                 [this] { return !frame_queue_.empty(); })) {
        Frame frame = std::move(frame_queue_.front());
        frame_queue_.pop();
        return frame;
    }
    
    return Frame{};
}

std::vector<MarkerDetection> ARCamera::detectMarkers(const Frame& frame) const {
    std::vector<MarkerDetection> detections;
    
    if (!aruco_dict_ || !detector_params_) {
        return detections;
    }
    
    try {
        cv::Mat cv_frame(frame.height(), frame.width(), CV_8UC3, 
                        const_cast<uint8_t*>(frame.data()));
        cv::Mat gray_frame;
        cv::cvtColor(cv_frame, gray_frame, cv::COLOR_RGB2GRAY);
        
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        
        cv::aruco::detectMarkers(gray_frame, aruco_dict_, marker_corners, marker_ids, detector_params_);
        
        if (!marker_ids.empty()) {
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 
                intrinsics_.fx, 0, intrinsics_.cx,
                0, intrinsics_.fy, intrinsics_.cy,
                0, 0, 1);
            cv::Mat dist_coeffs = (cv::Mat_<double>(5,1) << 
                intrinsics_.k1, intrinsics_.k2, intrinsics_.p1, intrinsics_.p2, 0);
            
            cv::aruco::estimatePoseSingleMarkers(marker_corners, config_.marker_size, 
                                               camera_matrix, dist_coeffs, rvecs, tvecs);
            
            for (size_t i = 0; i < marker_ids.size(); ++i) {
                MarkerDetection detection;
                detection.id = marker_ids[i];
                detection.corners = marker_corners[i];
                
                cv::Mat rotation_matrix;
                cv::Rodrigues(rvecs[i], rotation_matrix);
                
                Mat4 pose = Mat4::identity();
                for (int r = 0; r < 3; ++r) {
                    for (int c = 0; c < 3; ++c) {
                        pose(r, c) = static_cast<float>(rotation_matrix.at<double>(r, c));
                    }
                }
                pose(0, 3) = static_cast<float>(tvecs[i][0]);
                pose(1, 3) = static_cast<float>(tvecs[i][1]);
                pose(2, 3) = static_cast<float>(tvecs[i][2]);
                
                detection.pose = pose;
                detection.confidence = 1.0f;
                
                detections.push_back(detection);
            }
        }
        
    } catch (const cv::Exception& e) {
        // Log error
    }
    
    return detections;
}

bool ARCamera::loadCalibration(const std::string& filename) {
    try {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            return false;
        }
        
        cv::Mat camera_matrix, dist_coeffs;
        fs["camera_matrix"] >> camera_matrix;
        fs["distortion_coefficients"] >> dist_coeffs;
        
        if (camera_matrix.rows == 3 && camera_matrix.cols == 3) {
            intrinsics_.fx = static_cast<float>(camera_matrix.at<double>(0, 0));
            intrinsics_.fy = static_cast<float>(camera_matrix.at<double>(1, 1));
            intrinsics_.cx = static_cast<float>(camera_matrix.at<double>(0, 2));
            intrinsics_.cy = static_cast<float>(camera_matrix.at<double>(1, 2));
        }
        
        if (dist_coeffs.rows >= 4) {
            intrinsics_.k1 = static_cast<float>(dist_coeffs.at<double>(0));
            intrinsics_.k2 = static_cast<float>(dist_coeffs.at<double>(1));
            intrinsics_.p1 = static_cast<float>(dist_coeffs.at<double>(2));
            intrinsics_.p2 = static_cast<float>(dist_coeffs.at<double>(3));
        }
        
        return true;
        
    } catch (const cv::Exception& e) {
        return false;
    }
}

Mat4 ARCamera::getViewMatrix() const {
    return Mat4::identity(); // TODO: implement proper view matrix
}

Mat4 ARCamera::getProjectionMatrix(float near_plane, float far_plane) const {
    return intrinsics_.getProjectionMatrix(
        static_cast<float>(config_.desired_width), 
        static_cast<float>(config_.desired_height), 
        near_plane, far_plane);
}

std::unique_ptr<GLTexture> ARCamera::createBackgroundTexture() const {
    return std::make_unique<GLTexture>(config_.desired_width, config_.desired_height, GL_RGB8);
}

void ARCamera::updateBackgroundTexture(GLTexture& texture, const Frame& frame) const {
    if (frame.format() == Frame::Format::RGB) {
        texture.upload(frame.data(), GL_RGB, GL_UNSIGNED_BYTE);
    }
}

} // namespace toyar
