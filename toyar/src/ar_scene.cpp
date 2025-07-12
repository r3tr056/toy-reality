#include "ar_scene.h"
#include <iostream>
#include <chrono>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <GL/glew.h>
#include <opencv2/calib3d.hpp>

// Vertex shader source
const char* vertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec3 FragPos;
out vec3 Normal;

void main() {
    FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(model))) * aNormal;
    
    gl_Position = projection * view * vec4(FragPos, 1.0);
}
)";

// Fragment shader source
const char* fragmentShaderSource = R"(
#version 330 core
out vec4 FragColor;

in vec3 FragPos;
in vec3 Normal;

uniform vec3 lightPos;
uniform vec3 lightColor;
uniform vec3 objectColor;

void main() {
    // ambient
    float ambientStrength = 0.1;
    vec3 ambient = ambientStrength * lightColor;
    
    // diffuse
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;
    
    vec3 result = (ambient + diffuse) * objectColor;
    FragColor = vec4(result, 1.0);
}
)";

ARScene::ARScene() 
    : environmentScanned(false)
    , motionTrackingEnabled(true)
    , lightingEnabled(true)
    , occlusionEnabled(true)
    , scanTimer(0.0f)
{
    // Initialize lighting and occlusion systems
    lightingEstimator = std::make_unique<ARLightingEstimator>();
    shadowRenderer = std::make_unique<ARShadowRenderer>();
    occlusionTracker = std::make_unique<AROcclusionTracker>();
    occlusionRenderer = std::make_unique<AROcclusionRenderer>();
}

ARScene::~ARScene() {
	cleanup();
}

void ARScene::cleanup() {
    // Cleanup lighting and occlusion systems
    if (lightingEstimator) lightingEstimator->cleanup();
    if (shadowRenderer) shadowRenderer->cleanup();
    if (occlusionTracker) occlusionTracker->cleanup();
    if (occlusionRenderer) occlusionRenderer->cleanup();
    
	for (ARObject* obj : objects) {
		delete obj;
	}
	objects.clear();
	
	for (ARPlane* plane : planes) {
		delete plane;
	}
	planes.clear();
	
	gui.cleanup();
	
	if (shaderProgram != 0) {
		glDeleteProgram(shaderProgram);
	}
	if (vertexShader != 0) {
		glDeleteShader(vertexShader);
	}
	if (fragmentShader != 0) {
		glDeleteShader(fragmentShader);
	}
}

void ARScene::addObject(ARObject* object) {
	objects.push_back(object);
}

void ARScene::removeObject(ARObject* object) {
	for (auto it = objects.begin(); it != objects.end(); ++it) {
		if (*it == object) {
			objects.erase(it);
			delete object;
			return;
		}
	}

	std::cout << "Object not found in the scene." << std::endl;
}

void ARScene::addPlane(ARPlane* plane) {
	planes.push_back(plane);
}

void ARScene::removePlane(ARPlane* plane) {
	for (auto it = planes.begin(); it != planes.end(); ++it) {
		if (*it == plane) {
			planes.erase(it);
			delete plane;
			return;
		}
	}
}

void ARScene::drawScene() const {
	Vec3 camPos = camera.getPosition();
	Vec3 camOrientation = camera.getOrientation();
	
	std::cout << "Camera pos : (" << camPos.x << ", " << camPos.y << ", " << camPos.z << ")" << std::endl;
	std::cout << "Camera orientation : (" << camOrientation.x << ", " << camOrientation.y << ", " << camOrientation.z << ")" << std::endl;

	for (const ARObject* obj : objects) {
		obj->draw();
	}
}

void ARScene::renderScene() {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	cv::Mat frame;
	if (camera.readFrame(frame)) {
		camera.renderBackground(frame);
		
		// Process frame for motion tracking
		if (motionTrackingEnabled) {
			auto now = std::chrono::steady_clock::now();
			double timestamp = std::chrono::duration<double>(now.time_since_epoch()).count();
			
			bool trackingSuccess = motionTracker.processFrame(frame, timestamp);
			if (trackingSuccess) {
				updateCameraPoseFromTracking();
			}
		}

		// detect and estimate marker pose
		std::vector<int> markerIds;
		std::vector<std::vector<cv::Point2f>> markerCorners;
		camera.detectMarkers(frame, markerIds, markerCorners);

		if (!markerIds.empty()) {
			std::vector<cv::Vec3d> rvecs, tvecs;
			if (camera.estimatePoseSingleMarkers(markerCorners, 0.1f, rvecs, tvecs)) {
				for (size_t i = 0; i < markerIds.size(); ++i) {
					
					// Convert OpenCV's pose to a GL model matrix
					cv::Mat rotationMatrix;
					cv::Rodrigues(rvecs[i], rotationMatrix);

					glm::mat4 modelMatrix = glm::mat4(1.0f);
					for (int row = 0; row < 3; ++row) {
						for (int col = 0; col < 3; ++col) {
							modelMatrix[col][row] = rotationMatrix.at<double>(row, col);
						}
					}

					modelMatrix[3][0] = tvecs[i][0];
					modelMatrix[3][1] = tvecs[i][1];
					modelMatrix[3][2] = tvecs[i][2];

					// find the cube object and update its model matrix
					for (ARObject* obj : objects) {
						if (obj->name == "Cube") {
							obj->setModelMatrix(modelMatrix);
							break;
						}
					}
					
					// If we detect a marker, consider this a surface detection
					Vec3 markerPos(tvecs[i][0], tvecs[i][1], tvecs[i][2]);
					handleSurfaceDetection(markerPos);
				}
			}
		}
	}
	
	// Render 3D content
	glm::mat4 glmViewMatrix = camera.getViewMatrix();
	Mat4 toyarViewMatrix(glm::value_ptr(glmViewMatrix));
	
	// Render grid if enabled
	if (environmentScanned) {
		gui.renderGrid(toyarViewMatrix, Mat4::perspective(45.0f * 3.14159f / 180.0f, 
		                                                  1280.0f/720.0f, 0.1f, 100.0f));
	}
	
	// Draw virtual objects with lighting and occlusion
	if (lightingEnabled || occlusionEnabled) {
		renderWithLightingAndOcclusion();
	} else {
		// Fallback to standard rendering
		for (const ARObject* obj : objects) {
			obj->draw();
		}
		
		for (const ARPlane* plane : planes) {
			plane->draw();
		}
	}
	
	// Render GUI overlays
	gui.renderCrosshair();
	gui.renderHUD();
	
	// Render motion tracking debug info
	if (motionTrackingEnabled) {
		renderTrackingDebugInfo();
	}
}

bool ARScene::initialize(int windowWidth, int windowHeight) {
	if (!initializeOpenGL()) {
		std::cerr << "Failed to initialize OpenGL" << std::endl;
		return false;
	}
	
	if (!createShaders()) {
		std::cerr << "Failed to create shaders" << std::endl;
		return false;
	}
	
	if (!gui.initialize(windowWidth, windowHeight)) {
		std::cerr << "Failed to initialize GUI" << std::endl;
		return false;
	}
	
	// Initialize motion tracking
	if (!initializeMotionTracking()) {
		std::cerr << "Failed to initialize motion tracking" << std::endl;
		motionTrackingEnabled = false; // Continue without motion tracking
	}
	
	// Initialize lighting and occlusion systems
	if (!initializeLightingAndOcclusion()) {
		std::cerr << "Warning: Failed to initialize lighting and occlusion systems" << std::endl;
		// Continue execution - these are optional features
	}
	
	setupLighting();
	updateProjectionMatrix(45.0f, (float)windowWidth/windowHeight, 0.1f, 100.0f);
	
	// Start the scanning animation
	startEnvironmentScanning();
	
	return true;
}

bool ARScene::initializeOpenGL() {
	// Initialize GLEW
	if (glewInit() != GLEW_OK) {
		std::cerr << "Failed to initialize GLEW" << std::endl;
		return false;
	}
	
	// Enable depth testing
	glEnable(GL_DEPTH_TEST);
	
	// Set clear color
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	
	return true;
}

bool ARScene::createShaders() {
	// Compile vertex shader
	vertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
	glCompileShader(vertexShader);
	
	// Check for shader compile errors
	int success;
	char infoLog[512];
	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
	if (!success) {
		glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
		std::cerr << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
		return false;
	}
	
	// Compile fragment shader
	fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
	glCompileShader(fragmentShader);
	
	glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
	if (!success) {
		glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
		std::cerr << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
		return false;
	}
	
	// Link shaders
	shaderProgram = glCreateProgram();
	glAttachShader(shaderProgram, vertexShader);
	glAttachShader(shaderProgram, fragmentShader);
	glLinkProgram(shaderProgram);
	
	glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
	if (!success) {
		glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
		std::cerr << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
		return false;
	}
	
	return true;
}

void ARScene::setupLighting() {
	glUseProgram(shaderProgram);
	
	// Set light properties
	glUniform3f(glGetUniformLocation(shaderProgram, "lightPos"), 1.2f, 1.0f, 2.0f);
	glUniform3f(glGetUniformLocation(shaderProgram, "lightColor"), 1.0f, 1.0f, 1.0f);
}

void ARScene::updateProjectionMatrix(float fov, float aspect, float nearPlane, float farPlane) {
	projectionMatrix = glm::perspective(glm::radians(fov), aspect, nearPlane, farPlane);
}

glm::mat4 ARScene::getProjectionMatrix() const {
	return projectionMatrix;
}

void ARScene::startEnvironmentScanning() {
	environmentScanned = false;
	scanTimer = 0.0f;
	gui.showMessage("Looking for surfaces...");
	std::cout << "Starting environment scanning..." << std::endl;
}

void ARScene::handleSurfaceDetection(const Vec3& position) {
	if (!environmentScanned) {
		environmentScanned = true;
		gui.setSurfaceDetected(true, position);
		std::cout << "Surface detected at position: (" 
		          << position.x << ", " << position.y << ", " << position.z << ")" << std::endl;
	}
}

void ARScene::update(float deltaTime) {
	scanTimer += deltaTime;
	
	// Auto-complete scanning after a certain time if no surface is detected
	if (scanTimer > 5.0f && !environmentScanned) {
		// Simulate surface detection at origin
		handleSurfaceDetection(Vec3(0.0f, 0.0f, 0.0f));
	}
}

bool ARScene::initializeMotionTracking() {
	// Create camera matrix from intrinsics (simplified)
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	cameraMatrix.at<double>(0, 0) = 420.0; // fx
	cameraMatrix.at<double>(1, 1) = 420.0; // fy
	cameraMatrix.at<double>(0, 2) = 320.0; // cx
	cameraMatrix.at<double>(1, 2) = 240.0; // cy
	
	cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
	
	bool success = motionTracker.initialize(cameraMatrix, distCoeffs, 640, 480);
	
	if (success) {
		std::cout << "Motion tracking initialized successfully" << std::endl;
		motionTracker.setMaxFeatures(300);
		motionTracker.setMinFeatures(50);
	}
	
	return success;
}

bool ARScene::initializeLightingAndOcclusion() {
    std::cout << "Initializing lighting and occlusion systems..." << std::endl;
    
    // Initialize lighting estimator
    if (lightingEnabled && lightingEstimator) {
        if (!lightingEstimator->initialize()) {
            std::cerr << "Failed to initialize lighting estimator" << std::endl;
            lightingEnabled = false;
        }
    }
    
    // Initialize shadow renderer
    if (lightingEnabled && shadowRenderer) {
        ShadowMapConfig shadowConfig;
        shadowConfig.shadowMapSize = 2048;
        shadowConfig.shadowBias = 0.005f;
        shadowConfig.lightFrustumSize = 8.0f;
        
        if (!shadowRenderer->initialize(shadowConfig)) {
            std::cerr << "Failed to initialize shadow renderer" << std::endl;
            // Continue without shadows
        }
    }
    
    // Initialize occlusion tracker
    if (occlusionEnabled && occlusionTracker) {
        if (!occlusionTracker->initialize(OcclusionMethod::BACKGROUND_SUBTRACTION)) {
            std::cerr << "Failed to initialize occlusion tracker" << std::endl;
            occlusionEnabled = false;
        }
    }
    
    // Initialize occlusion renderer
    if (occlusionEnabled && occlusionRenderer) {
        if (!occlusionRenderer->initialize(1280, 720)) {
            std::cerr << "Failed to initialize occlusion renderer" << std::endl;
            occlusionEnabled = false;
        }
    }
    
    std::cout << "Lighting enabled: " << (lightingEnabled ? "Yes" : "No") << std::endl;
    std::cout << "Occlusion enabled: " << (occlusionEnabled ? "Yes" : "No") << std::endl;
    
    return true;
}

void ARScene::updateLighting(const cv::Mat& frame) {
    if (!lightingEnabled || !lightingEstimator || frame.empty()) return;
    
    // Get current camera transform from motion tracking
    Mat4 cameraTransform = Mat4::identity();
    if (motionTrackingEnabled) {
        Pose6DOF pose = motionTracker.getCurrentPose();
        cameraTransform = pose.transformMatrix;
    }
    
    // Estimate lighting from current frame
    lightingEstimator->estimateLight(frame, cameraTransform);
}

void ARScene::updateOcclusion(const cv::Mat& frame) {
    if (!occlusionEnabled || !occlusionTracker || frame.empty()) return;
    
    // Detect occlusion regions
    std::vector<OcclusionRegion> regions = occlusionTracker->detectOcclusion(frame);
    
    // Update occlusion renderer with detected regions
    if (occlusionRenderer && !regions.empty()) {
        cv::Mat depthMap; // Could be provided by depth estimator
        occlusionRenderer->updateOcclusionMask(regions, depthMap, frame.cols, frame.rows);
    }
}

const LightEstimate& ARScene::getCurrentLighting() const {
    static LightEstimate defaultLighting;
    if (lightingEnabled && lightingEstimator) {
        return lightingEstimator->getCurrentEstimate();
    }
    return defaultLighting;
}

void ARScene::updateCameraPoseFromTracking() {
	Pose6DOF pose = motionTracker.getCurrentPose();
	TrackingState state = motionTracker.getTrackingState();
	
	if (state == TrackingState::TRACKING) {
		// Update camera position and orientation from motion tracking
		camera.setPosition(pose.position);
		
		// Convert rotation vector to orientation (simplified)
		Vec3 orientation = pose.rotation.normalized();
		camera.setOrientation(orientation);
		
		std::cout << "Motion tracking pose: pos(" << pose.position.x 
		          << ", " << pose.position.y << ", " << pose.position.z 
		          << ") confidence: " << pose.confidence << std::endl;
	}
}

void ARScene::renderTrackingDebugInfo() {
	// Get tracked features for visualization
	std::vector<TrackedFeature> features = motionTracker.getTrackedFeatures();
	TrackingState state = motionTracker.getTrackingState();
	
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	
	// Render tracking state
	glColor4f(1.0f, 1.0f, 1.0f, 0.8f);
	
	// Convert screen coordinates and render feature points
	for (const auto& feature : features) {
		if (feature.isValid) {
			float x = (feature.imagePoint.x / 640.0f) * 2.0f - 1.0f;
			float y = 1.0f - (feature.imagePoint.y / 480.0f) * 2.0f;
			
			// Color based on feature age and confidence
			float green = feature.confidence;
			float red = 1.0f - feature.confidence;
			glColor4f(red, green, 0.0f, 0.7f);
			
			glPointSize(3.0f + feature.age * 0.1f);
			glBegin(GL_POINTS);
			glVertex2f(x, y);
			glEnd();
		}
	}
	
	// Render tracking state text (simplified)
	const char* stateText = "";
	switch (state) {
		case TrackingState::NOT_TRACKING: stateText = "NOT_TRACKING"; break;
		case TrackingState::INITIALIZING: stateText = "INITIALIZING"; break;
		case TrackingState::TRACKING: stateText = "TRACKING"; break;
		case TrackingState::LOST: stateText = "LOST"; break;
	}
	
	// Display feature count and tracking state
	glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
	// In a real implementation, you'd render text here
	// For now, we output to console periodically
	static int frameCounter = 0;
	if (++frameCounter % 60 == 0) { // Every 60 frames
		std::cout << "Tracking: " << stateText << " Features: " << features.size() << std::endl;
	}
	
	glDisable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
}

void ARScene::renderWithLightingAndOcclusion() {
    // Get current lighting estimate
    LightEstimate lighting = getCurrentLighting();
    
    // Render shadow map pass if lighting is enabled
    if (lightingEnabled && shadowRenderer) {
        renderShadowPass();
    }
    
    // Render main scene with lighting and occlusion
    renderMainPass();
}

void ARScene::renderShadowPass() {
    if (!shadowRenderer) return;
    
    LightEstimate lighting = getCurrentLighting();
    Vec3 sceneCenter(0.0f, 0.0f, 0.0f); // Could be calculated from objects
    
    // Begin shadow map generation
    shadowRenderer->beginShadowMapGeneration(lighting, sceneCenter);
    
    // Render all shadow-casting objects
    Mat4 lightView = shadowRenderer->getLightViewMatrix();
    Mat4 lightProj = shadowRenderer->getLightProjectionMatrix();
    
    for (const ARObject* obj : objects) {
        // Render object for shadow mapping (depth only)
        obj->renderForShadows(lightView, lightProj);
    }
    
    // End shadow map generation
    shadowRenderer->endShadowMapGeneration();
}

void ARScene::renderMainPass() {
    // Get matrices
    glm::mat4 glmViewMatrix = camera.getViewMatrix();
    Mat4 toyarViewMatrix(glm::value_ptr(glmViewMatrix));
    Mat4 projMatrix = Mat4::perspective(45.0f * 3.14159f / 180.0f, 1280.0f/720.0f, 0.1f, 100.0f);
    
    // Get current lighting
    LightEstimate lighting = getCurrentLighting();
    
    // Render objects with lighting and occlusion
    for (const ARObject* obj : objects) {
        if (occlusionEnabled && occlusionRenderer) {
            // Render with occlusion handling
            std::vector<ARObject*> singleObj = {const_cast<ARObject*>(obj)};
            Mat4 cameraTransform = Mat4::identity();
            if (motionTrackingEnabled) {
                Pose6DOF pose = motionTracker.getCurrentPose();
                cameraTransform = pose.transformMatrix;
            }
            
            occlusionRenderer->renderWithOcclusion(singleObj, toyarViewMatrix, projMatrix, cameraTransform);
        } else {
            // Standard rendering with lighting
            obj->drawWithLighting(toyarViewMatrix, projMatrix, lighting, 
                                shadowRenderer ? shadowRenderer->getShadowMapTexture() : 0);
        }
    }
    
    // Render planes
    for (const ARPlane* plane : planes) {
        plane->drawWithLighting(toyarViewMatrix, projMatrix, lighting);
    }
}