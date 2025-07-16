#ifndef AR_RESOURCE_H_
#define AR_RESOURCE_H_

#include "ar_la.h"
#include <GL/glew.h>
#include <memory>
#include <vector>
#include <unordered_map>
#include <string>
#include <functional>

namespace toyar {

/**
 * @brief RAII wrapper for OpenGL resources
 */
template<typename T, void(*Deleter)(T)>
class GLResource {
public:
    explicit GLResource(T resource = T{}) : resource_(resource) {}
    
    ~GLResource() {
        if (resource_ != T{}) {
            Deleter(resource_);
        }
    }
    
    // Non-copyable but movable
    GLResource(const GLResource&) = delete;
    GLResource& operator=(const GLResource&) = delete;
    
    GLResource(GLResource&& other) noexcept : resource_(other.release()) {}
    
    GLResource& operator=(GLResource&& other) noexcept {
        if (this != &other) {
            reset(other.release());
        }
        return *this;
    }
    
    T get() const { return resource_; }
    T release() {
        T tmp = resource_;
        resource_ = T{};
        return tmp;
    }
    
    void reset(T resource = T{}) {
        if (resource_ != T{}) {
            Deleter(resource_);
        }
        resource_ = resource;
    }
    
    explicit operator bool() const { return resource_ != T{}; }

private:
    T resource_;
};

// Specific OpenGL resource types
namespace detail {
    inline void deleteBuffer(GLuint buffer) { if (buffer) glDeleteBuffers(1, &buffer); }
    inline void deleteTexture(GLuint texture) { if (texture) glDeleteTextures(1, &texture); }
    inline void deleteShader(GLuint shader) { if (shader) glDeleteShader(shader); }
    inline void deleteProgram(GLuint program) { if (program) glDeleteProgram(program); }
    inline void deleteVertexArray(GLuint vao) { if (vao) glDeleteVertexArrays(1, &vao); }
    inline void deleteFramebuffer(GLuint fbo) { if (fbo) glDeleteFramebuffers(1, &fbo); }
    inline void deleteRenderbuffer(GLuint rbo) { if (rbo) glDeleteRenderbuffers(1, &rbo); }
}

using GLBuffer = GLResource<GLuint, detail::deleteBuffer>;
using GLTextureHandle = GLResource<GLuint, detail::deleteTexture>;
using GLShader = GLResource<GLuint, detail::deleteShader>;
using GLProgram = GLResource<GLuint, detail::deleteProgram>;
using GLVertexArray = GLResource<GLuint, detail::deleteVertexArray>;
using GLFramebuffer = GLResource<GLuint, detail::deleteFramebuffer>;
using GLRenderbuffer = GLResource<GLuint, detail::deleteRenderbuffer>;

/**
 * @brief Shader program builder with error checking
 */
class ShaderProgram {
public:
    ShaderProgram() = default;
    ~ShaderProgram() = default;
    
    // Non-copyable but movable
    ShaderProgram(const ShaderProgram&) = delete;
    ShaderProgram& operator=(const ShaderProgram&) = delete;
    ShaderProgram(ShaderProgram&&) = default;
    ShaderProgram& operator=(ShaderProgram&&) = default;
    
    /**
     * @brief Load shader from source code
     */
    bool loadFromSource(const std::string& vertex_source, 
                       const std::string& fragment_source,
                       const std::string& geometry_source = "");
    
    /**
     * @brief Load shader from files
     */
    bool loadFromFiles(const std::string& vertex_path,
                      const std::string& fragment_path,
                      const std::string& geometry_path = "");
    
    /**
     * @brief Use this shader program
     */
    void use() const;
    
    /**
     * @brief Get OpenGL program ID
     */
    GLuint getId() const { return program_.get(); }
    
    /**
     * @brief Check if program is valid
     */
    bool isValid() const { return program_.get() != 0; }
    
    // Uniform setters
    void setUniform(const std::string& name, float value) const;
    void setUniform(const std::string& name, int value) const;
    void setUniform(const std::string& name, const Vec3& value) const;
    void setUniform(const std::string& name, const Mat4& value) const;
    void setUniform(const std::string& name, bool value) const;
    
    /**
     * @brief Get uniform location (cached)
     */
    GLint getUniformLocation(const std::string& name) const;

private:
    bool compileShader(GLuint shader, const std::string& source) const;
    bool linkProgram() const;
    std::string readFile(const std::string& path) const;
    
    GLProgram program_;
    mutable std::unordered_map<std::string, GLint> uniform_cache_;
};

/**
 * @brief Mesh data and rendering
 */
class Mesh {
public:
    struct Vertex {
        Vec3 position;
        Vec3 normal;
        float u, v; // texture coordinates
        
        Vertex() = default;
        Vertex(const Vec3& pos, const Vec3& norm = Vec3::up(), float tex_u = 0, float tex_v = 0)
            : position(pos), normal(norm), u(tex_u), v(tex_v) {}
    };
    
    Mesh() = default;
    ~Mesh() = default;
    
    // Non-copyable but movable
    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;
    Mesh(Mesh&&) = default;
    Mesh& operator=(Mesh&&) = default;
    
    /**
     * @brief Create mesh from vertex and index data
     */
    void create(const std::vector<Vertex>& vertices, 
               const std::vector<unsigned int>& indices = {});
    
    /**
     * @brief Render the mesh
     */
    void render() const;
    
    /**
     * @brief Check if mesh is ready for rendering
     */
    bool isValid() const { return vao_.get() != 0 && vertex_count_ > 0; }
    
    /**
     * @brief Get vertex count
     */
    size_t getVertexCount() const { return vertex_count_; }
    
    /**
     * @brief Get index count
     */
    size_t getIndexCount() const { return index_count_; }
    
    // Factory methods for common shapes
    static Mesh createQuad(float size = 1.0f);
    static Mesh createCube(float size = 1.0f);
    static Mesh createSphere(float radius = 1.0f, int segments = 32);

private:
    void setupBuffers(const std::vector<Vertex>& vertices,
                     const std::vector<unsigned int>& indices);
    
    GLVertexArray vao_;
    GLBuffer vbo_;
    GLBuffer ebo_;
    
    size_t vertex_count_ = 0;
    size_t index_count_ = 0;
};

/**
 * @brief Resource manager for caching and sharing resources
 */
class ResourceManager {
public:
    static ResourceManager& getInstance();
    
    /**
     * @brief Load and cache a shader program
     */
    std::shared_ptr<ShaderProgram> getShader(const std::string& name,
                                            const std::string& vertex_path,
                                            const std::string& fragment_path,
                                            const std::string& geometry_path = "");
    
    /**
     * @brief Get cached shader by name
     */
    std::shared_ptr<ShaderProgram> getShader(const std::string& name);
    
    /**
     * @brief Clear all cached resources
     */
    void clear();
    
    /**
     * @brief Get memory usage statistics
     */
    struct Stats {
        size_t shader_count = 0;
        size_t memory_usage_bytes = 0;
    };
    
    Stats getStats() const;

private:
    ResourceManager() = default;
    ~ResourceManager() = default;
    
    std::unordered_map<std::string, std::shared_ptr<ShaderProgram>> shaders_;
};

} // namespace toyar

#endif // AR_RESOURCE_H_
