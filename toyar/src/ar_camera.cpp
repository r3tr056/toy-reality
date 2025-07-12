
#include "ar_camera.h"
#include <iostream>
#include <glm/gtc/matrix_transform.hpp>
#include <GL/glew.h>
#include <opencv2/calib3d.hpp>

// Texture implementation
Texture::Texture(int w, int h, GLenum fmt) : width(w), height(h), format(fmt) {
    glGenTextures(1, &textureId);
    glBindTexture(GL_TEXTURE_2D, textureId);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

Texture::~Texture() {
    glDeleteTextures(1, &textureId);
}

void Texture::Upload(const void* data, GLenum inputFormat, GLenum type) {
    glBindTexture(GL_TEXTURE_2D, textureId);
    glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, inputFormat, type, data);
}

void Texture::RenderToViewportFlipY() const {
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, textureId);
    
    glBegin(GL_QUADS);
        glTexCoord2f(0.0f, 1.0f); glVertex2f(-1.0f, -1.0f);
        glTexCoord2f(1.0f, 1.0f); glVertex2f(1.0f, -1.0f);
        glTexCoord2f(1.0f, 0.0f); glVertex2f(1.0f, 1.0f);
        glTexCoord2f(0.0f, 0.0f); glVertex2f(-1.0f, 1.0f);
    glEnd();
    
    glDisable(GL_TEXTURE_2D);
}

ARCamera::ARCamera() : frameWidth(640), frameHeight(480) {
	this->position = {0.0, 0.0, 0.0};
	this->orientation = {0.0, 0.0, -1.0};
	updateViewMatrix();

	// Initialize texture
	imageTexture = new Texture(frameWidth, frameHeight, GL_RGB8);

	// load camera intrinsics (replace with the actual calibration code)
	cameraIntrinsics[0][0] = 420.0f; cameraIntrinsics[0][1] = 0.0f; cameraIntrinsics[0][2] = 320.0f;
	cameraIntrinsics[1][0] = 0.0f; cameraIntrinsics[1][1] = 420.0f; cameraIntrinsics[1][2] = 240.0f;
	cameraIntrinsics[2][0] = 0.0f; cameraIntrinsics[2][1] = 0.0f; cameraIntrinsics[2][2] = 1.0f;
	
	distortionCoeffs[0] = 0.0f; distortionCoeffs[1] = 0.0f; distortionCoeffs[2] = 0.0f; 
	distortionCoeffs[3] = 0.0f; distortionCoeffs[4] = 0.0f;

	dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	detectorParams = cv::aruco::DetectorParameters::create();
}

ARCamera::~ARCamera() {
	closeCamera();
	delete imageTexture;
}


bool ARCamera::openCamera(int cameraIndex) {
	capture.open(cameraIndex);
	if (!capture.isOpened()) {
		std::cerr << "Error: Could not open camera." << std::endl;
		return false;
	}

	capture.set(cv::CAP_PROP_FRAME_WIDTH, frameWidth);
	capture.set(cv::CAP_PROP_FRAME_HEIGHT, frameHeight);
	return true;
}


void ARCamera::closeCamera() {
	
	if (capture.isOpened()) {
		capture.release();
	}
}

bool ARCamera::readFrame(cv::Mat& frame) {

	if (!capture.isOpened()) {
		std::cerr << "Error: Camera not opened." << std::endl;
		return false;
	}
	return capture.read(frame);
}

void ARCamera::renderBackground(const cv::Mat& frame) const {
	if (!frame.empty()) {
		imageTexture->Upload(frame.data, GL_BGR, GL_UNSIGNED_BYTE);
		glColor3f(1.0f, 1.0f, 1.0f);
		imageTexture->RenderToViewportFlipY();
	}
}

void ARCamera::detectMarkers(
	const cv::Mat& frame,
	std::vector<int>& markerIds,
	std::vector<std::vector<cv::Point2f>>& markerCorners
) const {

	cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, detectorParams);
}

bool ARCamera::estimatePoseSingleMarkers(
	const std::vector<std::vector<cv::Point2f>>& markerCorners,
	float markerSize,
	std::vector<cv::Vec3d>& rvecs,
	std::vector<cv::Vec3d>& tvecs
) const {

	if (markerCorners.empty()) return false;
	cv::Mat cameraMatrix(3, 3, CV_32FC1);
	cv::Mat distCoeffs(5, 1, CV_32FC1);

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {
			cameraMatrix.at<float>(i, j) = cameraIntrinsics[i][j];
		}
	}

	for (int i = 0; i < 5; ++i) {
		distCoeffs.at<float>(i) = distortionCoeffs[i];
	}

	cv::aruco::estimatePoseSingleMarkers(markerCorners, markerSize, cameraMatrix, distCoeffs, rvecs, tvecs);
	return true;
}

void ARCamera::setPosition(const Vec3& pos) {
	this->position = pos;
	updateViewMatrix();
}

Vec3 ARCamera::getPosition() const {
	return this->position;
}

void ARCamera::setOrientation(const Vec3& orientation) {
	this->orientation = orientation;
	updateViewMatrix();
}

Vec3 ARCamera::getOrientation() const {
	return this->orientation;
}

glm::mat4 ARCamera::getViewMatrix() const {
	return this->viewMatrix;
}

void ARCamera::updateViewMatrix() {
	glm::vec3 cameraPos(position.x, position.y, position.z);
	glm::vec3 cameraDirection(orientation.x, orientation.y, orientation.z);
	glm::vec3 cameraUp(0.0f, 1.0f, 0.0f);

	this->viewMatrix = glm::lookAt(cameraPos, cameraPos + cameraDirection, cameraUp);
}

