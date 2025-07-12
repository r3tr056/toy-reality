#ifndef AR_GUI_H_
#define AR_GUI_H_

#include "ar_la.h"
#include <GL/glew.h>
#include <string>
#include <vector>

using namespace toyar;

class ARGui {
public:
    ARGui();
    ~ARGui();

    bool initialize(int screenWidth, int screenHeight);
    void cleanup();

    // Main GUI rendering
    void renderGrid(const Mat4& viewMatrix, const Mat4& projMatrix);
    void renderSurfaceIndicator(const Vec3& position, bool surfaceFound = false);
    void renderHUD();
    void renderCrosshair();
    
    // Surface detection
    void setSurfaceDetected(bool detected, const Vec3& position = Vec3(0, 0, 0));
    void showMessage(const std::string& message);
    
    // Settings
    void setGridEnabled(bool enabled) { gridEnabled = enabled; }
    void setHUDEnabled(bool enabled) { hudEnabled = enabled; }
    void setCrosshairEnabled(bool enabled) { crosshairEnabled = enabled; }

private:
    // Rendering setup
    void setupShaders();
    void setupGeometry();
    void renderQuad();
    void renderText(const std::string& text, float x, float y, float scale = 1.0f);
    
    // Shader management
    GLuint createShader(GLenum type, const char* source);
    GLuint createProgram(const char* vertexSource, const char* fragmentSource);
    
    // Screen dimensions
    int screenWidth, screenHeight;
    
    // OpenGL resources
    GLuint gridShader, hudShader;
    GLuint gridVAO, gridVBO;
    GLuint quadVAO, quadVBO;
    
    // Surface detection
    bool surfaceDetected;
    Vec3 surfacePosition;
    std::string currentMessage;
    
    // UI state
    bool gridEnabled;
    bool hudEnabled;
    bool crosshairEnabled;
    
    // Grid parameters
    float gridSpacing;
    float gridAlpha;
    Vec3 gridColor;
};

#endif // AR_GUI_H_
