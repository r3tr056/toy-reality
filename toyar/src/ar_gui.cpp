#include "ar_gui.h"
#include <iostream>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// Simple grid shader without animations
const char* gridVertexShader = R"(
#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 mvpMatrix;

void main() {
    gl_Position = mvpMatrix * vec4(aPos, 1.0);
}
)";

const char* gridFragmentShader = R"(
#version 330 core
out vec4 FragColor;

uniform vec3 gridColor;
uniform float alpha;

void main() {
    FragColor = vec4(gridColor, alpha);
}
)";

// Simple HUD shader for text and UI elements
const char* hudVertexShader = R"(
#version 330 core
layout (location = 0) in vec2 aPos;

void main() {
    gl_Position = vec4(aPos, 0.0, 1.0);
}
)";

const char* hudFragmentShader = R"(
#version 330 core
out vec4 FragColor;

uniform vec3 color;
uniform float alpha;

void main() {
    FragColor = vec4(color, alpha);
}
)";

ARGui::ARGui() 
    : screenWidth(1280), screenHeight(720)
    , surfaceDetected(false)
    , gridEnabled(true), hudEnabled(true), crosshairEnabled(true)
    , gridSpacing(0.1f), gridAlpha(0.6f)
    , gridColor(0.3f, 0.7f, 1.0f)
{
    gridShader = hudShader = 0;
    gridVAO = gridVBO = 0;
    quadVAO = quadVBO = 0;
}

ARGui::~ARGui() {
    cleanup();
}

bool ARGui::initialize(int width, int height) {
    screenWidth = width;
    screenHeight = height;
    
    setupShaders();
    setupGeometry();
    
    return true;
}

void ARGui::cleanup() {
    if (gridShader) glDeleteProgram(gridShader);
    if (hudShader) glDeleteProgram(hudShader);
    
    if (gridVAO) glDeleteVertexArrays(1, &gridVAO);
    if (gridVBO) glDeleteBuffers(1, &gridVBO);
    if (quadVAO) glDeleteVertexArrays(1, &quadVAO);
    if (quadVBO) glDeleteBuffers(1, &quadVBO);
}

void ARGui::setupShaders() {
    gridShader = createProgram(gridVertexShader, gridFragmentShader);
    hudShader = createProgram(hudVertexShader, hudFragmentShader);
}

void ARGui::setupGeometry() {
    // Setup simple grid geometry
    std::vector<float> gridVertices;
    int gridResolution = 20;
    
    // Create grid lines
    for (int i = 0; i <= gridResolution; ++i) {
        float pos = (i / float(gridResolution) - 0.5f) * 4.0f;
        
        // Horizontal lines
        gridVertices.push_back(-2.0f); gridVertices.push_back(0.0f); gridVertices.push_back(pos);
        gridVertices.push_back(2.0f);  gridVertices.push_back(0.0f); gridVertices.push_back(pos);
        
        // Vertical lines
        gridVertices.push_back(pos); gridVertices.push_back(0.0f); gridVertices.push_back(-2.0f);
        gridVertices.push_back(pos); gridVertices.push_back(0.0f); gridVertices.push_back(2.0f);
    }
    
    glGenVertexArrays(1, &gridVAO);
    glGenBuffers(1, &gridVBO);
    
    glBindVertexArray(gridVAO);
    glBindBuffer(GL_ARRAY_BUFFER, gridVBO);
    glBufferData(GL_ARRAY_BUFFER, gridVertices.size() * sizeof(float), 
                 gridVertices.data(), GL_STATIC_DRAW);
    
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    
    // Setup simple quad for HUD elements
    float quadVertices[] = {
        -1.0f, -1.0f,
         1.0f, -1.0f,
         1.0f,  1.0f,
        -1.0f,  1.0f
    };
    
    unsigned int quadIndices[] = {
        0, 1, 2,
        2, 3, 0
    };
    
    GLuint quadEBO;
    glGenVertexArrays(1, &quadVAO);
    glGenBuffers(1, &quadVBO);
    glGenBuffers(1, &quadEBO);
    
    glBindVertexArray(quadVAO);
    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices, GL_STATIC_DRAW);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, quadEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(quadIndices), quadIndices, GL_STATIC_DRAW);
    
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    
    glBindVertexArray(0);
}

void ARGui::setSurfaceDetected(bool detected, const Vec3& position) {
    surfaceDetected = detected;
    surfacePosition = position;
    
    if (detected) {
        showMessage("Surface found. Tap to create surface anchor.");
    }
}

void ARGui::showMessage(const std::string& message) {
    currentMessage = message;
}

void ARGui::renderGrid(const Mat4& viewMatrix, const Mat4& projMatrix) {
    if (!gridEnabled) return;
    
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    glUseProgram(gridShader);
    
    // Create MVP matrix
    glm::mat4 model = glm::mat4(1.0f);
    glm::mat4 view = glm::make_mat4(viewMatrix.data());
    glm::mat4 proj = glm::make_mat4(projMatrix.data());
    glm::mat4 mvp = proj * view * model;
    
    glUniformMatrix4fv(glGetUniformLocation(gridShader, "mvpMatrix"), 1, GL_FALSE, glm::value_ptr(mvp));
    glUniform3f(glGetUniformLocation(gridShader, "gridColor"), gridColor.x, gridColor.y, gridColor.z);
    glUniform1f(glGetUniformLocation(gridShader, "alpha"), gridAlpha);
    
    glBindVertexArray(gridVAO);
    glLineWidth(1.0f);
    glDrawArrays(GL_LINES, 0, 84); // 21 * 4 lines
    glBindVertexArray(0);
    
    glDisable(GL_BLEND);
}

void ARGui::renderSurfaceIndicator(const Vec3& /*position*/, bool surfaceFound) {
    if (!surfaceFound) return;
    
    // Render a circular indicator at the surface position
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // This would render a 3D circle at the detected surface
    // Implementation would involve creating geometry for a circle
    // and rendering it at the given position
    
    glDisable(GL_BLEND);
}

void ARGui::renderHUD() {
    if (!hudEnabled) return;
    
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // Render current message if any
    if (!currentMessage.empty()) {
        renderText(currentMessage, -0.5f, -0.1f, 1.2f);
    }
    
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
}

void ARGui::renderCrosshair() {
    if (!crosshairEnabled) return;
    
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    // Simple crosshair in the center
    glLineWidth(2.0f);
    glColor4f(1.0f, 1.0f, 1.0f, 0.8f);
    
    glBegin(GL_LINES);
        // Horizontal line
        glVertex2f(-0.05f, 0.0f);
        glVertex2f(0.05f, 0.0f);
        // Vertical line
        glVertex2f(0.0f, -0.05f);
        glVertex2f(0.0f, 0.05f);
    glEnd();
    
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
}

void ARGui::renderText(const std::string& /*text*/, float /*x*/, float /*y*/, float /*scale*/) {
    // Simple text rendering - in a real implementation, you'd use
    // a proper text rendering library like FreeType
    // For now, this is a placeholder
}

GLuint ARGui::createShader(GLenum type, const char* source) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);
    
    int success;
    char infoLog[512];
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(shader, 512, NULL, infoLog);
        std::cerr << "Shader compilation failed: " << infoLog << std::endl;
    }
    
    return shader;
}

GLuint ARGui::createProgram(const char* vertexSource, const char* fragmentSource) {
    GLuint vertexShader = createShader(GL_VERTEX_SHADER, vertexSource);
    GLuint fragmentShader = createShader(GL_FRAGMENT_SHADER, fragmentSource);
    
    GLuint program = glCreateProgram();
    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);
    glLinkProgram(program);
    
    int success;
    char infoLog[512];
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(program, 512, NULL, infoLog);
        std::cerr << "Program linking failed: " << infoLog << std::endl;
    }
    
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    
    return program;
}
