#include "ar_resource.h"
#include "ar_error.h"
#include <GL/glew.h>
#include <fstream>
#include <sstream>
#include <iostream>

namespace toyar {

// ═══════════════════════════════════════════════════════════════════════════
// ShaderProgram Implementation
// ═══════════════════════════════════════════════════════════════════════════

bool ShaderProgram::loadFromSource(const std::string& vertex_source, 
                                   const std::string& fragment_source,
                                   const std::string& geometry_source) {
    // Create program
    GLuint program_id = glCreateProgram();
    if (program_id == 0) {
        TOYAR_ERROR("Shader", "Failed to create shader program");
        return false;
    }
    
    // Create and compile vertex shader
    GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    if (!compileShader(vertex_shader, vertex_source)) {
        glDeleteShader(vertex_shader);
        glDeleteProgram(program_id);
        return false;
    }
    glAttachShader(program_id, vertex_shader);
    
    // Create and compile fragment shader
    GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    if (!compileShader(fragment_shader, fragment_source)) {
        glDeleteShader(vertex_shader);
        glDeleteShader(fragment_shader);
        glDeleteProgram(program_id);
        return false;
    }
    glAttachShader(program_id, fragment_shader);
    
    // Create and compile geometry shader if provided
    GLuint geometry_shader = 0;
    if (!geometry_source.empty()) {
        geometry_shader = glCreateShader(GL_GEOMETRY_SHADER);
        if (!compileShader(geometry_shader, geometry_source)) {
            glDeleteShader(vertex_shader);
            glDeleteShader(fragment_shader);
            glDeleteShader(geometry_shader);
            glDeleteProgram(program_id);
            return false;
        }
        glAttachShader(program_id, geometry_shader);
    }
    
    // Link program
    glLinkProgram(program_id);
    
    // Check linking status
    GLint success;
    glGetProgramiv(program_id, GL_LINK_STATUS, &success);
    if (!success) {
        GLchar info_log[1024];
        glGetProgramInfoLog(program_id, 1024, nullptr, info_log);
        TOYAR_ERROR("Shader", std::string("Program linking failed: ") + info_log);
        glDeleteShader(vertex_shader);
        glDeleteShader(fragment_shader);
        if (geometry_shader != 0) glDeleteShader(geometry_shader);
        glDeleteProgram(program_id);
        return false;
    }
    
    // Cleanup shaders (they're linked now)
    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);
    if (geometry_shader != 0) glDeleteShader(geometry_shader);
    
    // Store the program
    program_ = GLProgram(program_id);
    return true;
}

bool ShaderProgram::loadFromFiles(const std::string& vertex_path,
                                  const std::string& fragment_path,
                                  const std::string& geometry_path) {
    try {
        std::string vertex_source = readFile(vertex_path);
        std::string fragment_source = readFile(fragment_path);
        std::string geometry_source = geometry_path.empty() ? "" : readFile(geometry_path);
        
        return loadFromSource(vertex_source, fragment_source, geometry_source);
    } catch (const std::exception& e) {
        TOYAR_ERROR("Shader", std::string("Failed to load shader files: ") + e.what());
        return false;
    }
}

void ShaderProgram::use() const {
    if (!isValid()) {
        TOYAR_ERROR("Shader", "Attempting to use invalid shader program");
        return;
    }
    glUseProgram(program_.get());
}

GLint ShaderProgram::getUniformLocation(const std::string& name) const {
    auto it = uniform_cache_.find(name);
    if (it != uniform_cache_.end()) {
        return it->second;
    }
    
    GLint location = glGetUniformLocation(program_.get(), name.c_str());
    uniform_cache_[name] = location;
    
    if (location == -1) {
        TOYAR_WARN("Shader", "Uniform not found: " + name);
    }
    
    return location;
}

void ShaderProgram::setUniform(const std::string& name, float value) const {
    GLint location = getUniformLocation(name);
    if (location >= 0) {
        glUniform1f(location, value);
    }
}

void ShaderProgram::setUniform(const std::string& name, int value) const {
    GLint location = getUniformLocation(name);
    if (location >= 0) {
        glUniform1i(location, value);
    }
}

void ShaderProgram::setUniform(const std::string& name, const Vec3& value) const {
    GLint location = getUniformLocation(name);
    if (location >= 0) {
        glUniform3f(location, value.x, value.y, value.z);
    }
}

void ShaderProgram::setUniform(const std::string& name, const Mat4& value) const {
    GLint location = getUniformLocation(name);
    if (location >= 0) {
        glUniformMatrix4fv(location, 1, GL_FALSE, value.data());
    }
}

void ShaderProgram::setUniform(const std::string& name, bool value) const {
    setUniform(name, static_cast<int>(value));
}

bool ShaderProgram::compileShader(GLuint shader, const std::string& source) const {
    const char* src = source.c_str();
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);
    
    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        GLchar info_log[1024];
        glGetShaderInfoLog(shader, 1024, nullptr, info_log);
        TOYAR_ERROR("Shader", std::string("Shader compilation failed: ") + info_log);
        return false;
    }
    
    return true;
}

std::string ShaderProgram::readFile(const std::string& path) const {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + path);
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

// ═══════════════════════════════════════════════════════════════════════════
// Mesh Implementation (simplified stub to allow linking)
// ═══════════════════════════════════════════════════════════════════════════

void Mesh::create(const std::vector<Vertex>& vertices, 
                  const std::vector<unsigned int>& indices) {
    // Simplified implementation for now
    vao_ = GLVertexArray(0);
    vbo_ = GLBuffer(0);
    ebo_ = GLBuffer(0);
    
    GLuint vao, vbo, ebo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);
    
    vao_ = GLVertexArray(vao);
    vbo_ = GLBuffer(vbo);
    ebo_ = GLBuffer(ebo);
    
    glBindVertexArray(vao);
    
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), 
                 vertices.data(), GL_STATIC_DRAW);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int),
                 indices.data(), GL_STATIC_DRAW);
    
    // Position attribute
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
    
    // Normal attribute
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), 
                         (void*)(3 * sizeof(float)));
    
    // Texture coordinate attribute
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), 
                         (void*)(6 * sizeof(float)));
    
    glBindVertexArray(0);
    
    index_count_ = indices.size();
}

void Mesh::render() const {
    if (vao_.get() != 0) {
        glBindVertexArray(vao_.get());
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(index_count_), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// ResourceManager Implementation (simplified stub)
// ═══════════════════════════════════════════════════════════════════════════

ResourceManager& ResourceManager::getInstance() {
    static ResourceManager instance;
    return instance;
}

std::shared_ptr<ShaderProgram> ResourceManager::getShader(const std::string& name,
                                                          const std::string& vertex_path,
                                                          const std::string& fragment_path,
                                                          const std::string& geometry_path) {
    auto it = shaders_.find(name);
    if (it != shaders_.end()) {
        return it->second;
    }
    
    auto shader = std::make_shared<ShaderProgram>();
    if (shader->loadFromFiles(vertex_path, fragment_path, geometry_path)) {
        shaders_[name] = shader;
        return shader;
    }
    
    TOYAR_ERROR("ResourceManager", "Failed to create shader: " + name);
    return nullptr;
}

std::shared_ptr<ShaderProgram> ResourceManager::getShader(const std::string& name) {
    auto it = shaders_.find(name);
    if (it != shaders_.end()) {
        return it->second;
    }
    
    TOYAR_WARN("ResourceManager", "Shader not found: " + name);
    return nullptr;
}

} // namespace toyar
