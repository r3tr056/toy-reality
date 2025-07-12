#ifndef AR_OBJECT_H_
#define AR_OBJECT_H_

#include "ar_la.h"
#include <string>
#include <glm/glm.hpp>
#include <GL/glew.h>

using namespace toyar;

// Forward declaration
struct LightEstimate;

class ARObject {
public:
    ARObject(const std::string& objName, const Vec3& pos = Vec3(0, 0, 0));
    virtual ~ARObject();

    virtual void draw() const;
    virtual void render() const;
    
    // Advanced rendering with lighting and shadows
    virtual void drawWithLighting(const Mat4& viewMatrix, const Mat4& projMatrix, 
                                 const LightEstimate& lighting, GLuint shadowMap = 0) const;
    virtual void renderForShadows(const Mat4& lightView, const Mat4& lightProj) const;

    void setPosition(const Vec3& pos);
    Vec3 getPosition() const;
    
    void setModelMatrix(const glm::mat4& matrix);
    glm::mat4 getModelMatrix() const;
    
    std::string getType() const;
    
    // Material properties
    void setMaterial(const Vec3& ambient, const Vec3& diffuse, const Vec3& specular, float shininess);
    void setColor(const Vec3& color) { this->color = color; }
    
    std::string name;
    std::string type;

protected:
    Vec3 position;
    glm::mat4 modelMatrix;
    Vec3 scale;
    Vec3 rotation;
    Vec3 color;
    
    // Material properties for lighting
    Vec3 materialAmbient;
    Vec3 materialDiffuse;
    Vec3 materialSpecular;
    float materialShininess;
    
    // OpenGL rendering data
    GLuint VAO, VBO, EBO;
    GLuint lightingShader;
    GLuint shadowShader;
    
    void setupMesh();
    void setupCube();
    void setupSphere();
    void setupLightingShaders();
    void updateModelMatrix();
};

#endif // AR_OBJECT_H_
