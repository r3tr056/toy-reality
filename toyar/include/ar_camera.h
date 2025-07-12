#ifndef AR_CAMERA_H_
#define AR_CAMERA_H_

#include "ar_core.h"
#include "ar_la.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <glm/glm.hpp>
#include <GL/glew.h>

using namespace toyar;

class Texture;

class ARCamera {
public:
    ARCamera();
    ~ARCamera();

    bool openCamera(int cameraIndex = 0);
    void closeCamera();
    bool readFrame(cv::Mat& frame);
    void renderBackground(const cv::Mat& frame) const;

    void detectMarkers(
        const cv::Mat& frame,
        std::vector<int>& markerIds,
        std::vector<std::vector<cv::Point2f>>& markerCorners
    ) const;

    bool estimatePoseSingleMarkers(
        const std::vector<std::vector<cv::Point2f>>& markerCorners,
        float markerSize,
        std::vector<cv::Vec3d>& rvecs,
        std::vector<cv::Vec3d>& tvecs
    ) const;

    void setPosition(const Vec3& pos);
    Vec3 getPosition() const;
    void setOrientation(const Vec3& orientation);
    Vec3 getOrientation() const;
    glm::mat4 getViewMatrix() const;

private:
    void updateViewMatrix();

    int frameWidth, frameHeight;
    Vec3 position;
    Vec3 orientation;
    glm::mat4 viewMatrix;
    
    cv::VideoCapture capture;
    Texture* imageTexture;
    
    // Camera calibration
    float cameraIntrinsics[3][3];
    float distortionCoeffs[5];
    
    // ArUco detection
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams;
};

class Texture {
public:
    Texture(int width, int height, GLenum format);
    ~Texture();
    
    void Upload(const void* data, GLenum inputFormat, GLenum type);
    void RenderToViewportFlipY() const;
    
private:
    GLuint textureId;
    int width, height;
    GLenum format;
};

#endif // AR_CAMERA_H_
