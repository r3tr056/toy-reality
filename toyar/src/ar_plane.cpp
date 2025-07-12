

#include "ar_plane.h"
#include "ar_lighting.h"
#include <iostream>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <GL/glew.h>

ARPlane::ARPlane(const Vec3& pos, const Vec3& norm) 
    : position(pos), normal(norm), size(1.0f), color(0.5f, 0.5f, 0.5f), 
      materialAmbient(0.3f, 0.3f, 0.3f),
      materialDiffuse(0.7f, 0.7f, 0.7f),
      materialSpecular(0.2f, 0.2f, 0.2f),
      materialShininess(32.0f),
      VAO(0), VBO(0),
      lightingShader(0), shadowShader(0) {
    setupMesh();
    initializeShaders();
}

ARPlane::~ARPlane() {
    if (VAO != 0) {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
    }
    if (lightingShader != 0) {
        glDeleteProgram(lightingShader);
    }
    if (shadowShader != 0) {
        glDeleteProgram(shadowShader);
    }
}

void ARPlane::setupMesh() {
    // Simple plane vertices (quad)
    float vertices[] = {
        // positions          // normals
        -0.5f, 0.0f, -0.5f,   0.0f, 1.0f, 0.0f,
         0.5f, 0.0f, -0.5f,   0.0f, 1.0f, 0.0f,
         0.5f, 0.0f,  0.5f,   0.0f, 1.0f, 0.0f,
         
         0.5f, 0.0f,  0.5f,   0.0f, 1.0f, 0.0f,
        -0.5f, 0.0f,  0.5f,   0.0f, 1.0f, 0.0f,
        -0.5f, 0.0f, -0.5f,   0.0f, 1.0f, 0.0f
    };

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}

void ARPlane::draw() const {
	std::cout << "Drawing plane at (" << position.x << ", " << position.y << ", " << position.z << ") with normal (" << normal.x << ", " << normal.y << ", " << normal.z << ")" << std::endl;
}

void ARPlane::render() const {
	glColor3f(color.x, color.y, color.z);

	glBegin(GL_QUADS);
		glNormal3f(normal.x, normal.y, normal.z);
		glVertex3f(position.x - size*0.5f, position.y, position.z - size*0.5f);
		glVertex3f(position.x + size*0.5f, position.y, position.z - size*0.5f);
		glVertex3f(position.x + size*0.5f, position.y, position.z + size*0.5f);
		glVertex3f(position.x - size*0.5f, position.y, position.z + size*0.5f);
	glEnd();
}

void ARPlane::setPosition(const Vec3& pos) {
    this->position = pos;
}

Vec3 ARPlane::getPosition() const {
    return this->position;
}

void ARPlane::setNormal(const Vec3& norm) {
    this->normal = norm;
}

Vec3 ARPlane::getNormal() const {
    return this->normal;
}

void ARPlane::setMaterial(const Vec3& ambient, const Vec3& diffuse, const Vec3& specular, float shininess) {
    this->materialAmbient = ambient;
    this->materialDiffuse = diffuse;
    this->materialSpecular = specular;
    this->materialShininess = shininess;
}

void ARPlane::drawWithLighting(const Mat4& viewMatrix, const Mat4& projMatrix, 
                              const LightEstimate& lighting, GLuint shadowMap) const {
    if (lightingShader == 0) {
        // Fallback to standard drawing
        draw();
        return;
    }
    
    glUseProgram(lightingShader);
    
    // Convert matrices to glm
    glm::mat4 view = glm::make_mat4(viewMatrix.data());
    glm::mat4 proj = glm::make_mat4(projMatrix.data());
    
    // Create model matrix for plane transformation
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(position.x, position.y, position.z));
    model = glm::scale(model, glm::vec3(size, 1.0f, size));
    
    // Set matrices
    GLint modelLoc = glGetUniformLocation(lightingShader, "model");
    GLint viewLoc = glGetUniformLocation(lightingShader, "view");
    GLint projLoc = glGetUniformLocation(lightingShader, "projection");
    
    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(proj));
    
    // Set lighting parameters
    GLint ambientLoc = glGetUniformLocation(lightingShader, "ambientColor");
    GLint ambientIntensityLoc = glGetUniformLocation(lightingShader, "ambientIntensity");
    GLint lightDirLoc = glGetUniformLocation(lightingShader, "lightDirection");
    GLint lightColorLoc = glGetUniformLocation(lightingShader, "lightColor");
    GLint lightIntensityLoc = glGetUniformLocation(lightingShader, "lightIntensity");
    
    glUniform3f(ambientLoc, lighting.ambientColor.x, lighting.ambientColor.y, lighting.ambientColor.z);
    glUniform1f(ambientIntensityLoc, lighting.ambientIntensity);
    glUniform3f(lightDirLoc, lighting.directionalLightDir.x, lighting.directionalLightDir.y, lighting.directionalLightDir.z);
    glUniform3f(lightColorLoc, lighting.directionalLightColor.x, lighting.directionalLightColor.y, lighting.directionalLightColor.z);
    glUniform1f(lightIntensityLoc, lighting.directionalIntensity);
    
    // Set material properties
    GLint matAmbientLoc = glGetUniformLocation(lightingShader, "material.ambient");
    GLint matDiffuseLoc = glGetUniformLocation(lightingShader, "material.diffuse");
    GLint matSpecularLoc = glGetUniformLocation(lightingShader, "material.specular");
    GLint matShininessLoc = glGetUniformLocation(lightingShader, "material.shininess");
    
    glUniform3f(matAmbientLoc, materialAmbient.x, materialAmbient.y, materialAmbient.z);
    glUniform3f(matDiffuseLoc, materialDiffuse.x * color.x, materialDiffuse.y * color.y, materialDiffuse.z * color.z);
    glUniform3f(matSpecularLoc, materialSpecular.x, materialSpecular.y, materialSpecular.z);
    glUniform1f(matShininessLoc, materialShininess);
    
    // Bind shadow map if available
    if (shadowMap != 0) {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, shadowMap);
        GLint shadowMapLoc = glGetUniformLocation(lightingShader, "shadowMap");
        glUniform1i(shadowMapLoc, 0);
        
        GLint useShadowsLoc = glGetUniformLocation(lightingShader, "useShadows");
        glUniform1i(useShadowsLoc, 1);
    } else {
        GLint useShadowsLoc = glGetUniformLocation(lightingShader, "useShadows");
        glUniform1i(useShadowsLoc, 0);
    }
    
    // Render the plane mesh
    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);
    
    glUseProgram(0);
}

void ARPlane::renderForShadows(const Mat4& lightView, const Mat4& lightProj) const {
    if (shadowShader == 0) return;
    
    glUseProgram(shadowShader);
    
    // Convert matrices
    glm::mat4 lightV = glm::make_mat4(lightView.data());
    glm::mat4 lightP = glm::make_mat4(lightProj.data());
    
    // Create model matrix for plane transformation
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(position.x, position.y, position.z));
    model = glm::scale(model, glm::vec3(size, 1.0f, size));
    
    glm::mat4 lightSpaceMatrix = lightP * lightV;
    
    // Set uniforms
    GLint lightSpaceLoc = glGetUniformLocation(shadowShader, "lightSpaceMatrix");
    GLint modelLoc = glGetUniformLocation(shadowShader, "model");
    
    glUniformMatrix4fv(lightSpaceLoc, 1, GL_FALSE, glm::value_ptr(lightSpaceMatrix));
    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
    
    // Render the plane for shadow mapping
    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);
    
    glUseProgram(0);
}

bool ARPlane::initializeShaders() {
    // Vertex shader source for lighting
    const char* lightingVertexShaderSource = R"(
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
    
    // Fragment shader source for lighting
    const char* lightingFragmentShaderSource = R"(
        #version 330 core
        out vec4 FragColor;
        
        in vec3 FragPos;
        in vec3 Normal;
        
        uniform vec3 ambientColor;
        uniform float ambientIntensity;
        uniform vec3 lightDirection;
        uniform vec3 lightColor;
        uniform float lightIntensity;
        
        struct Material {
            vec3 ambient;
            vec3 diffuse;
            vec3 specular;
            float shininess;
        };
        uniform Material material;
        
        uniform bool useShadows;
        uniform sampler2D shadowMap;
        
        void main() {
            // Ambient lighting
            vec3 ambient = ambientColor * ambientIntensity * material.ambient;
            
            // Diffuse lighting
            vec3 norm = normalize(Normal);
            vec3 lightDir = normalize(-lightDirection);
            float diff = max(dot(norm, lightDir), 0.0);
            vec3 diffuse = lightColor * lightIntensity * diff * material.diffuse;
            
            // Specular lighting (simple Phong)
            vec3 viewDir = normalize(-FragPos); // Simplified view direction
            vec3 reflectDir = reflect(-lightDir, norm);
            float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
            vec3 specular = lightColor * lightIntensity * spec * material.specular;
            
            vec3 result = ambient + diffuse + specular;
            FragColor = vec4(result, 1.0);
        }
    )";
    
    // Shadow vertex shader source
    const char* shadowVertexShaderSource = R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;
        
        uniform mat4 lightSpaceMatrix;
        uniform mat4 model;
        
        void main() {
            gl_Position = lightSpaceMatrix * model * vec4(aPos, 1.0);
        }
    )";
    
    // Shadow fragment shader source
    const char* shadowFragmentShaderSource = R"(
        #version 330 core
        
        void main() {
            // OpenGL automatically writes depth
        }
    )";
    
    // Compile lighting shaders
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &lightingVertexShaderSource, NULL);
    glCompileShader(vertexShader);
    
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &lightingFragmentShaderSource, NULL);
    glCompileShader(fragmentShader);
    
    lightingShader = glCreateProgram();
    glAttachShader(lightingShader, vertexShader);
    glAttachShader(lightingShader, fragmentShader);
    glLinkProgram(lightingShader);
    
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    
    // Compile shadow shaders
    GLuint shadowVertShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(shadowVertShader, 1, &shadowVertexShaderSource, NULL);
    glCompileShader(shadowVertShader);
    
    GLuint shadowFragShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(shadowFragShader, 1, &shadowFragmentShaderSource, NULL);
    glCompileShader(shadowFragShader);
    
    shadowShader = glCreateProgram();
    glAttachShader(shadowShader, shadowVertShader);
    glAttachShader(shadowShader, shadowFragShader);
    glLinkProgram(shadowShader);
    
    glDeleteShader(shadowVertShader);
    glDeleteShader(shadowFragShader);
    
    return (lightingShader != 0 && shadowShader != 0);
}
