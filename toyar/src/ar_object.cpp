#include "ar_object.h"
#include "ar_lighting.h"
#include <iostream>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <GL/glut.h>

ARObject::ARObject(const std::string& objName, const Vec3& pos) 
    : name(objName), position(pos), scale(1.0f, 1.0f, 1.0f), 
      rotation(0.0f, 0.0f, 0.0f), color(1.0f, 1.0f, 1.0f),
      materialAmbient(0.2f, 0.2f, 0.2f), materialDiffuse(0.8f, 0.8f, 0.8f),
      materialSpecular(1.0f, 1.0f, 1.0f), materialShininess(32.0f),
      lightingShader(0), shadowShader(0) {
    
    // Set type based on name (simple mapping)
    if (objName == "Cube" || objName == "cube") {
        type = "cube";
    } else if (objName == "Sphere" || objName == "sphere") {
        type = "sphere";
    } else {
        type = "cube"; // default
    }
    
    modelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(pos.x, pos.y, pos.z));
    setupMesh();
    setupLightingShaders();
}

ARObject::~ARObject() {
    if (VAO != 0) {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
        if (EBO != 0) glDeleteBuffers(1, &EBO);
    }
    
    if (lightingShader != 0) glDeleteProgram(lightingShader);
    if (shadowShader != 0) glDeleteProgram(shadowShader);
}

void ARObject::setupMesh() {
    if (type == "cube") {
        setupCube();
    } else if (type == "sphere") {
        setupSphere();
    }
}

void ARObject::setupCube() {
    // Simple cube vertices
    float vertices[] = {
        // positions          // normals
        -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
         0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
         0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
         0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
        -0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,
        -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,

        -0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
         0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
        -0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,
        -0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,

        -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
        -0.5f,  0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
        -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
        -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,
        -0.5f, -0.5f,  0.5f, -1.0f,  0.0f,  0.0f,
        -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,

         0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
         0.5f,  0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
         0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
         0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,
         0.5f, -0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
         0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,

        -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
         0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,
         0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
         0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
        -0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,
        -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,

        -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,
         0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
         0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
        -0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,
        -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f
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

void ARObject::setupSphere() {
    // For now, use the same cube setup - could be improved with actual sphere geometry
    setupCube();
}

void ARObject::draw() const {
	std::cout << "Drawing object: " << name << " at (" << position.x << ", " << position.y << ", " << position.z << ")" << std::endl;
}

void ARObject::render() const {
	if (type == "cube") {
		glColor3f(1.0f, 0.0f, 0.0f);
		glTranslatef(position.x, position.y, position.z);
		glutSolidCube(0.5f);
	} else if (type == "sphere") {
		glColor3f(0.0f, 0.0f, 1.0f);
		glTranslatef(position.x, position.y, position.z);
		glutSolidSphere(0.3f, 20, 20);
	}
}


void ARObject::setPosition(const Vec3& pos) {
	this->position = pos;
	this->modelMatrix = glm::translate(glm::mat4(1.0f), glm::vec3(pos.x, pos.y, pos.z));
}

Vec3 ARObject::getPosition() const {
	return this->position;
}

void ARObject::setModelMatrix(const glm::mat4& matrix) {
    this->modelMatrix = matrix;
}

std::string ARObject::getType() const {
	return this->type;
}

glm::mat4 ARObject::getModelMatrix() const {
	return this->modelMatrix;
}

void ARObject::drawWithLighting(const Mat4& viewMatrix, const Mat4& projMatrix, 
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
    glm::mat4 model = modelMatrix;
    
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
    
    // Render the mesh
    glBindVertexArray(VAO);
    if (EBO != 0) {
        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
    } else {
        glDrawArrays(GL_TRIANGLES, 0, 36);
    }
    glBindVertexArray(0);
    
    glUseProgram(0);
}

void ARObject::renderForShadows(const Mat4& lightView, const Mat4& lightProj) const {
    if (shadowShader == 0) return;
    
    glUseProgram(shadowShader);
    
    // Convert matrices
    glm::mat4 lightV = glm::make_mat4(lightView.data());
    glm::mat4 lightP = glm::make_mat4(lightProj.data());
    glm::mat4 model = modelMatrix;
    glm::mat4 lightSpaceMatrix = lightP * lightV;
    
    // Set uniforms
    GLint lightSpaceLoc = glGetUniformLocation(shadowShader, "lightSpaceMatrix");
    GLint modelLoc = glGetUniformLocation(shadowShader, "model");
    
    glUniformMatrix4fv(lightSpaceLoc, 1, GL_FALSE, glm::value_ptr(lightSpaceMatrix));
    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
    
    // Render depth only
    glBindVertexArray(VAO);
    if (EBO != 0) {
        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
    } else {
        glDrawArrays(GL_TRIANGLES, 0, 36);
    }
    glBindVertexArray(0);
    
    glUseProgram(0);
}

void ARObject::setMaterial(const Vec3& ambient, const Vec3& diffuse, const Vec3& specular, float shininess) {
    materialAmbient = ambient;
    materialDiffuse = diffuse;
    materialSpecular = specular;
    materialShininess = shininess;
}

void ARObject::setupLightingShaders() {
    // Lighting vertex shader
    const char* lightingVertexSource = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

out vec3 FragPos;
out vec3 Normal;
out vec4 FragPosLightSpace;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform mat4 lightSpaceMatrix;

void main() {
    FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(model))) * aNormal;
    FragPosLightSpace = lightSpaceMatrix * vec4(FragPos, 1.0);
    
    gl_Position = projection * view * vec4(FragPos, 1.0);
}
)";
    
    // Lighting fragment shader
    const char* lightingFragmentSource = R"(
#version 330 core
out vec4 FragColor;

in vec3 FragPos;
in vec3 Normal;
in vec4 FragPosLightSpace;

struct Material {
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    float shininess;
};

uniform Material material;
uniform vec3 ambientColor;
uniform float ambientIntensity;
uniform vec3 lightDirection;
uniform vec3 lightColor;
uniform float lightIntensity;
uniform vec3 viewPos;
uniform sampler2D shadowMap;
uniform bool useShadows;

float ShadowCalculation(vec4 fragPosLightSpace) {
    if (!useShadows) return 0.0;
    
    vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
    projCoords = projCoords * 0.5 + 0.5;
    
    if (projCoords.z > 1.0) return 0.0;
    
    float closestDepth = texture(shadowMap, projCoords.xy).r;
    float currentDepth = projCoords.z;
    
    float bias = 0.005;
    float shadow = currentDepth - bias > closestDepth ? 1.0 : 0.0;
    
    return shadow;
}

void main() {
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(-lightDirection);
    
    // Ambient
    vec3 ambient = ambientColor * ambientIntensity * material.ambient;
    
    // Diffuse
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = lightColor * lightIntensity * diff * material.diffuse;
    
    // Specular
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    vec3 specular = lightColor * lightIntensity * spec * material.specular;
    
    // Shadow
    float shadow = ShadowCalculation(FragPosLightSpace);
    vec3 lighting = ambient + (1.0 - shadow) * (diffuse + specular);
    
    FragColor = vec4(lighting, 1.0);
}
)";
    
    // Shadow vertex shader
    const char* shadowVertexSource = R"(
#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 lightSpaceMatrix;
uniform mat4 model;

void main() {
    gl_Position = lightSpaceMatrix * model * vec4(aPos, 1.0);
}
)";
    
    // Shadow fragment shader
    const char* shadowFragmentSource = R"(
#version 330 core

void main() {
    // OpenGL automatically writes depth
}
)";
    
    // Compile and link lighting shader
    GLuint lightingVert = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(lightingVert, 1, &lightingVertexSource, nullptr);
    glCompileShader(lightingVert);
    
    GLuint lightingFrag = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(lightingFrag, 1, &lightingFragmentSource, nullptr);
    glCompileShader(lightingFrag);
    
    lightingShader = glCreateProgram();
    glAttachShader(lightingShader, lightingVert);
    glAttachShader(lightingShader, lightingFrag);
    glLinkProgram(lightingShader);
    
    glDeleteShader(lightingVert);
    glDeleteShader(lightingFrag);
    
    // Compile and link shadow shader
    GLuint shadowVert = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(shadowVert, 1, &shadowVertexSource, nullptr);
    glCompileShader(shadowVert);
    
    GLuint shadowFrag = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(shadowFrag, 1, &shadowFragmentSource, nullptr);
    glCompileShader(shadowFrag);
    
    shadowShader = glCreateProgram();
    glAttachShader(shadowShader, shadowVert);
    glAttachShader(shadowShader, shadowFrag);
    glLinkProgram(shadowShader);
    
    glDeleteShader(shadowVert);
    glDeleteShader(shadowFrag);
}
