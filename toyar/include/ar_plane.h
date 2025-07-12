#ifndef AR_PLANE_H_
#define AR_PLANE_H_

#include "ar_la.h"
#include <GL/glew.h>

using namespace toyar;

// Forward declaration
struct LightEstimate;

class ARPlane {
public:
    ARPlane(const Vec3& pos = Vec3(0, 0, 0), const Vec3& norm = Vec3(0, 1, 0));
    virtual ~ARPlane();

    virtual void draw() const;
    virtual void render() const;
    
    // Advanced rendering with lighting and shadows
    virtual void drawWithLighting(const Mat4& viewMatrix, const Mat4& projMatrix, 
                                 const LightEstimate& lighting, GLuint shadowMap = 0) const;
    virtual void renderForShadows(const Mat4& lightView, const Mat4& lightProj) const;

    void setPosition(const Vec3& pos);
    Vec3 getPosition() const;
    
    void setNormal(const Vec3& norm);
    Vec3 getNormal() const;
    
    // Material properties for lighting
    void setMaterial(const Vec3& ambient, const Vec3& diffuse, const Vec3& specular, float shininess);
    void setColor(const Vec3& color) { this->color = color; }

protected:
    Vec3 position;
    Vec3 normal;
    float size;
    Vec3 color;
    
    // Material properties
    Vec3 materialAmbient;
    Vec3 materialDiffuse;
    Vec3 materialSpecular;
    float materialShininess;
    
    // OpenGL rendering data
    GLuint VAO, VBO;
    GLuint lightingShader;
    GLuint shadowShader;
    
    void setupMesh();
    bool initializeShaders();
};

#endif // AR_PLANE_H_
