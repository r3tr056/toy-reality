#ifndef AR_SCENE_H_
#define AR_SCENE_H_

#include "ar_camera.h"
#include "ar_object.h"
#include "ar_plane.h"
#include "ar_gui.h"
#include "ar_motion_tracking.h"
#include "ar_plane_detection.h"
#include "ar_lighting.h"
#include "ar_occlusion.h"
#include "ar_la.h"
#include <vector>
#include <memory>
#include <glm/glm.hpp>

using namespace toyar;

class ARScene {
public:
    ARScene();
    ~ARScene();

    void addObject(ARObject* object);
    void removeObject(ARObject* object);
    void addPlane(ARPlane* plane);
    void removePlane(ARPlane* plane);

    void drawScene() const;
    void renderScene();
    
    bool initialize(int windowWidth = 1280, int windowHeight = 720);
    void cleanup();
    
    void updateProjectionMatrix(float fov, float aspect, float nearPlane, float farPlane);
    glm::mat4 getProjectionMatrix() const;

    // GUI and interaction
    void startEnvironmentScanning();
    void handleSurfaceDetection(const Vec3& position);
    void toggleGrid() { gui.setGridEnabled(!environmentScanned); }
    
    // Motion tracking
    bool initializeMotionTracking();
    void updateCameraPoseFromTracking();
    void renderTrackingDebugInfo();
    
    // Lighting and occlusion
    bool initializeLightingAndOcclusion();
    void updateLighting(const cv::Mat& frame);
    void updateOcclusion(const cv::Mat& frame);
    const LightEstimate& getCurrentLighting() const;
    
    // Animation and visual effects
    bool isScanning() const { return !environmentScanned; }
    void update(float deltaTime);

    ARCamera camera;
    ARGui gui;
    ARMotionTracker motionTracker;

private:
    std::vector<ARObject*> objects;
    std::vector<ARPlane*> planes;
    
    glm::mat4 projectionMatrix;
    
    // OpenGL state
    GLuint shaderProgram;
    GLuint vertexShader, fragmentShader;
    
    // Lighting and occlusion systems
    std::unique_ptr<ARLightingEstimator> lightingEstimator;
    std::unique_ptr<ARShadowRenderer> shadowRenderer;
    std::unique_ptr<AROcclusionTracker> occlusionTracker;
    std::unique_ptr<AROcclusionRenderer> occlusionRenderer;
    
    // Application state
    bool environmentScanned;
    bool motionTrackingEnabled;
    bool lightingEnabled;
    bool occlusionEnabled;
    float scanTimer;
    
    bool initializeOpenGL();
    bool createShaders();
    void setupLighting();
    void renderWithLightingAndOcclusion();
    void renderShadowPass();
    void renderMainPass();
};

#endif // AR_SCENE_H_
