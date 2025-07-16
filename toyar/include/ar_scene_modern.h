#ifndef AR_SCENE_MODERN_H_
#define AR_SCENE_MODERN_H_

#include "ar_core.h"
#include "ar_camera.h"
#include "ar_resource.h"
#include "ar_error.h"
#include <vector>
#include <memory>
#include <unordered_map>

namespace toyar {

/**
 * @brief 3D Object representation for the scene
 */
class SceneObject {
public:
    SceneObject() = default;
    virtual ~SceneObject() = default;
    
    virtual void render(const Mat4& view_matrix, const Mat4& projection_matrix) = 0;
    virtual void update(float delta_time) {}
    
    void setTransform(const Mat4& transform) { transform_ = transform; }
    const Mat4& getTransform() const { return transform_; }
    
    void setVisible(bool visible) { visible_ = visible; }
    bool isVisible() const { return visible_; }

protected:
    Mat4 transform_ = Mat4::identity();
    bool visible_ = true;
};

/**
 * @brief Simple cube object for testing
 */
class CubeObject : public SceneObject {
public:
    CubeObject(float size = 1.0f);
    void render(const Mat4& view_matrix, const Mat4& projection_matrix) override;

private:
    std::unique_ptr<Mesh> mesh_;
    std::shared_ptr<ShaderProgram> shader_;
    float size_;
};

/**
 * @brief Modern scene management with performance optimization
 */
class Scene {
public:
    struct Config {
        float near_plane = 0.1f;
        float far_plane = 100.0f;
        bool enable_frustum_culling = true;
        bool enable_depth_testing = true;
        bool enable_face_culling = true;
        Vec3 clear_color = Vec3(0.1f, 0.1f, 0.1f);
        
        static Config defaultConfig() { return Config{}; }
    };
    
    explicit Scene(const Config& config = Config::defaultConfig());
    ~Scene() = default;
    
    // Non-copyable but movable
    Scene(const Scene&) = delete;
    Scene& operator=(const Scene&) = delete;
    Scene(Scene&&) = default;
    Scene& operator=(Scene&&) = default;
    
    /**
     * @brief Initialize the scene
     */
    void initialize();
    
    /**
     * @brief Add object to scene
     */
    void addObject(std::shared_ptr<SceneObject> object, const std::string& name = "");
    
    /**
     * @brief Remove object from scene
     */
    void removeObject(const std::string& name);
    
    /**
     * @brief Get object by name
     */
    std::shared_ptr<SceneObject> getObject(const std::string& name);
    
    /**
     * @brief Update all objects
     */
    void update(float delta_time);
    
    /**
     * @brief Render the scene
     */
    void render(const Mat4& view_matrix, const Mat4& projection_matrix);
    
    /**
     * @brief Set background frame for AR
     */
    void setBackgroundFrame(const Frame& frame);
    
    /**
     * @brief Update marker poses
     */
    void updateMarkerPose(int marker_id, const Mat4& pose);
    
    /**
     * @brief Clear the scene
     */
    void clear();
    
    /**
     * @brief Get render statistics
     */
    struct RenderStats {
        size_t objects_rendered = 0;
        size_t triangles_rendered = 0;
        float render_time_ms = 0.0f;
        float update_time_ms = 0.0f;
    };
    
    RenderStats getLastFrameStats() const { return last_frame_stats_; }

private:
    void setupOpenGLState();
    void renderBackground();
    void renderObjects(const Mat4& view_matrix, const Mat4& projection_matrix);
    
    Config config_;
    std::unordered_map<std::string, std::shared_ptr<SceneObject>> objects_;
    std::unordered_map<int, Mat4> marker_poses_;
    
    // Background rendering
    std::unique_ptr<GLTexture> background_texture_;
    std::shared_ptr<ShaderProgram> background_shader_;
    std::unique_ptr<Mesh> background_quad_;
    
    // Statistics
    RenderStats last_frame_stats_;
    size_t frame_counter_ = 0;
    
    bool initialized_ = false;
};

/**
 * @brief Main AR application class
 */
class ARApp {
public:
    struct Config {
        int window_width = 1280;
        int window_height = 720;
        std::string window_title = "ToyAR Application";
        bool fullscreen = false;
        bool vsync = true;
        
        ARCamera::Config camera_config;
        Scene::Config scene_config;
        
        static Config defaultConfig() { return Config{}; }
    };
    
    explicit ARApp(const Config& config = Config::defaultConfig());
    ~ARApp();
    
    /**
     * @brief Initialize the application
     */
    void initialize();
    
    /**
     * @brief Run the main loop
     */
    void run();
    
    /**
     * @brief Shutdown the application
     */
    void shutdown();
    
    /**
     * @brief Get the scene
     */
    Scene& getScene() { return scene_; }
    
    /**
     * @brief Get the camera
     */
    ARCamera& getCamera() { return camera_; }

private:
    void initializeWindow();
    void initializeOpenGL();
    void mainLoop();
    void handleEvents();
    void render();
    
    Config config_;
    ARCamera camera_;
    Scene scene_;
    
    // Window management (we'll use GLFW)
    void* window_ = nullptr; // GLFWwindow*
    bool should_close_ = false;
    
    // Timing
    std::chrono::steady_clock::time_point last_frame_time_;
    float delta_time_ = 0.0f;
    float fps_ = 0.0f;
};

} // namespace toyar

#endif // AR_SCENE_MODERN_H_
