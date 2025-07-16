#include "ar_scene_modern.h"
#include "ar_error.h"
#include <GLFW/glfw3.h>
#include <GL/glew.h>
#include <iostream>

namespace toyar {

// CubeObject implementation
CubeObject::CubeObject(float size) : size_(size) {
    mesh_ = std::make_unique<Mesh>();
    
    // Create a simple cube
    std::vector<Mesh::Vertex> vertices = {
        // Front face
        {{-size/2, -size/2,  size/2}, {0, 0, 1}, 0, 0},
        {{ size/2, -size/2,  size/2}, {0, 0, 1}, 1, 0},
        {{ size/2,  size/2,  size/2}, {0, 0, 1}, 1, 1},
        {{-size/2,  size/2,  size/2}, {0, 0, 1}, 0, 1},
        
        // Back face
        {{-size/2, -size/2, -size/2}, {0, 0, -1}, 1, 0},
        {{-size/2,  size/2, -size/2}, {0, 0, -1}, 1, 1},
        {{ size/2,  size/2, -size/2}, {0, 0, -1}, 0, 1},
        {{ size/2, -size/2, -size/2}, {0, 0, -1}, 0, 0}
    };
    
    std::vector<unsigned int> indices = {
        // Front face
        0, 1, 2, 2, 3, 0,
        // Back face
        4, 5, 6, 6, 7, 4,
        // Left face
        0, 3, 5, 5, 4, 0,
        // Right face
        1, 7, 6, 6, 2, 1,
        // Top face
        3, 2, 6, 6, 5, 3,
        // Bottom face
        0, 4, 7, 7, 1, 0
    };
    
    mesh_->create(vertices, indices);
    
    // Get basic shader from resource manager
    shader_ = ResourceManager::getInstance().getShader("basic", 
        "shaders/basic.vert", "shaders/basic.frag");
}

void CubeObject::render(const Mat4& view_matrix, const Mat4& projection_matrix) {
    if (!mesh_->isValid() || !shader_ || !shader_->isValid()) {
        return;
    }
    
    shader_->use();
    shader_->setUniform("u_model", transform_);
    shader_->setUniform("u_view", view_matrix);
    shader_->setUniform("u_projection", projection_matrix);
    shader_->setUniform("u_color", Vec3(1.0f, 0.5f, 0.0f)); // Orange cube
    
    mesh_->render();
}

// Scene implementation
Scene::Scene(const Config& config) : config_(config) {
    TOYAR_INFO("Scene", "Creating scene");
}

void Scene::initialize() {
    TOYAR_PROFILE_FUNCTION();
    
    if (initialized_) {
        return;
    }
    
    setupOpenGLState();
    
    // Create background rendering resources
    background_texture_ = std::make_unique<GLTexture>(1920, 1080, GL_RGB8);
    background_quad_ = std::make_unique<Mesh>();
    
    // Create fullscreen quad for background
    std::vector<Mesh::Vertex> quad_vertices = {
        {{-1, -1, 0}, {0, 0, 1}, 0, 0},
        {{ 1, -1, 0}, {0, 0, 1}, 1, 0},
        {{ 1,  1, 0}, {0, 0, 1}, 1, 1},
        {{-1,  1, 0}, {0, 0, 1}, 0, 1}
    };
    
    std::vector<unsigned int> quad_indices = {0, 1, 2, 2, 3, 0};
    background_quad_->create(quad_vertices, quad_indices);
    
    // Get background shader
    background_shader_ = ResourceManager::getInstance().getShader("background",
        "shaders/background.vert", "shaders/background.frag");
    
    initialized_ = true;
    TOYAR_INFO("Scene", "Scene initialized successfully");
}

void Scene::setupOpenGLState() {
    if (config_.enable_depth_testing) {
        TOYAR_GL_CHECK(glEnable(GL_DEPTH_TEST));
        TOYAR_GL_CHECK(glDepthFunc(GL_LESS));
    }
    
    if (config_.enable_face_culling) {
        TOYAR_GL_CHECK(glEnable(GL_CULL_FACE));
        TOYAR_GL_CHECK(glCullFace(GL_BACK));
        TOYAR_GL_CHECK(glFrontFace(GL_CCW));
    }
    
    TOYAR_GL_CHECK(glClearColor(config_.clear_color.x, config_.clear_color.y, 
                               config_.clear_color.z, 1.0f));
}

void Scene::addObject(std::shared_ptr<SceneObject> object, const std::string& name) {
    if (!object) {
        TOYAR_WARN("Scene", "Attempted to add null object");
        return;
    }
    
    std::string object_name = name.empty() ? "object_" + std::to_string(objects_.size()) : name;
    objects_[object_name] = object;
    
    TOYAR_DEBUG("Scene", "Added object: " + object_name);
}

void Scene::removeObject(const std::string& name) {
    auto it = objects_.find(name);
    if (it != objects_.end()) {
        objects_.erase(it);
        TOYAR_DEBUG("Scene", "Removed object: " + name);
    } else {
        TOYAR_WARN("Scene", "Object not found: " + name);
    }
}

std::shared_ptr<SceneObject> Scene::getObject(const std::string& name) {
    auto it = objects_.find(name);
    return (it != objects_.end()) ? it->second : nullptr;
}

void Scene::update(float delta_time) {
    TOYAR_PROFILE("Scene::update");
    
    auto start_time = std::chrono::steady_clock::now();
    
    for (auto& [name, object] : objects_) {
        if (object) {
            object->update(delta_time);
        }
    }
    
    auto end_time = std::chrono::steady_clock::now();
    last_frame_stats_.update_time_ms = 
        std::chrono::duration<float, std::milli>(end_time - start_time).count();
}

void Scene::render(const Mat4& view_matrix, const Mat4& projection_matrix) {
    TOYAR_PROFILE("Scene::render");
    
    if (!initialized_) {
        TOYAR_ERROR("Scene", "Scene not initialized");
        return;
    }
    
    auto start_time = std::chrono::steady_clock::now();
    
    // Clear buffers
    TOYAR_GL_CHECK(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));
    
    // Render background (camera feed)
    renderBackground();
    
    // Render 3D objects
    renderObjects(view_matrix, projection_matrix);
    
    auto end_time = std::chrono::steady_clock::now();
    last_frame_stats_.render_time_ms = 
        std::chrono::duration<float, std::milli>(end_time - start_time).count();
    
    frame_counter_++;
}

void Scene::renderBackground() {
    if (!background_texture_ || !background_shader_ || !background_quad_) {
        return;
    }
    
    TOYAR_GL_CHECK(glDisable(GL_DEPTH_TEST));
    
    background_shader_->use();
    background_texture_->bind(0);
    background_shader_->setUniform("u_texture", 0);
    
    background_quad_->render();
    
    background_texture_->unbind();
    
    if (config_.enable_depth_testing) {
        TOYAR_GL_CHECK(glEnable(GL_DEPTH_TEST));
    }
}

void Scene::renderObjects(const Mat4& view_matrix, const Mat4& projection_matrix) {
    last_frame_stats_.objects_rendered = 0;
    last_frame_stats_.triangles_rendered = 0;
    
    for (auto& [name, object] : objects_) {
        if (object && object->isVisible()) {
            object->render(view_matrix, projection_matrix);
            last_frame_stats_.objects_rendered++;
        }
    }
}

void Scene::setBackgroundFrame(const Frame& frame) {
    if (background_texture_ && !frame.empty()) {
        if (frame.format() == Frame::Format::RGB) {
            background_texture_->upload(frame.data(), GL_RGB, GL_UNSIGNED_BYTE);
        }
    }
}

void Scene::updateMarkerPose(int marker_id, const Mat4& pose) {
    marker_poses_[marker_id] = pose;
    
    // Update objects attached to this marker
    std::string marker_object_name = "marker_" + std::to_string(marker_id);
    auto object = getObject(marker_object_name);
    if (object) {
        object->setTransform(pose);
    }
}

void Scene::clear() {
    objects_.clear();
    marker_poses_.clear();
    last_frame_stats_ = {};
    frame_counter_ = 0;
}

// ARApp implementation
ARApp::ARApp(const Config& config) : config_(config), camera_(config.camera_config), scene_(config.scene_config) {
    TOYAR_INFO("App", "Creating ARApp");
}

ARApp::~ARApp() {
    shutdown();
}

void ARApp::initialize() {
    TOYAR_PROFILE_FUNCTION();
    
    try {
        initializeWindow();
        initializeOpenGL();
        
        camera_.initialize();
        scene_.initialize();
        
        // Set up camera callbacks
        camera_.setFrameCallback([this](const Frame& frame) {
            scene_.setBackgroundFrame(frame);
        });
        
        camera_.setMarkerCallback([this](const std::vector<MarkerDetection>& markers) {
            for (const auto& marker : markers) {
                if (marker.isValid()) {
                    scene_.updateMarkerPose(marker.id, marker.pose);
                }
            }
        });
        
        TOYAR_INFO("App", "ARApp initialized successfully");
        
    } catch (const std::exception& e) {
        TOYAR_ERROR("App", std::string("Failed to initialize ARApp: ") + e.what());
        throw;
    }
}

void ARApp::initializeWindow() {
    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW");
    }
    
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    
    GLFWwindow* window = glfwCreateWindow(config_.window_width, config_.window_height,
                                         config_.window_title.c_str(), 
                                         config_.fullscreen ? glfwGetPrimaryMonitor() : nullptr,
                                         nullptr);
    if (!window) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }
    
    window_ = window;
    glfwMakeContextCurrent(window);
    
    if (config_.vsync) {
        glfwSwapInterval(1);
    }
    
    // Set callbacks
    glfwSetWindowUserPointer(window, this);
    glfwSetKeyCallback(window, [](GLFWwindow* window, int key, int scancode, int action, int mods) {
        if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        }
    });
}

void ARApp::initializeOpenGL() {
    if (glewInit() != GLEW_OK) {
        throw std::runtime_error("Failed to initialize GLEW");
    }
    
    TOYAR_INFO("App", std::string("OpenGL Version: ") + reinterpret_cast<const char*>(glGetString(GL_VERSION)));
    TOYAR_INFO("App", std::string("GLSL Version: ") + reinterpret_cast<const char*>(glGetString(GL_SHADING_LANGUAGE_VERSION)));
}

void ARApp::run() {
    if (!window_) {
        TOYAR_ERROR("App", "Application not initialized");
        return;
    }
    
    camera_.start();
    last_frame_time_ = std::chrono::steady_clock::now();
    
    TOYAR_INFO("App", "Starting main loop");
    
    while (!glfwWindowShouldClose(static_cast<GLFWwindow*>(window_))) {
        mainLoop();
    }
    
    camera_.stop();
    TOYAR_INFO("App", "Main loop ended");
}

void ARApp::mainLoop() {
    TOYAR_PROFILE("ARApp::mainLoop");
    
    auto current_time = std::chrono::steady_clock::now();
    delta_time_ = std::chrono::duration<float>(current_time - last_frame_time_).count();
    last_frame_time_ = current_time;
    
    // Calculate FPS
    fps_ = 1.0f / delta_time_;
    
    handleEvents();
    
    scene_.update(delta_time_);
    render();
    
    glfwSwapBuffers(static_cast<GLFWwindow*>(window_));
}

void ARApp::handleEvents() {
    glfwPollEvents();
    
    if (glfwGetKey(static_cast<GLFWwindow*>(window_), GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        should_close_ = true;
        glfwSetWindowShouldClose(static_cast<GLFWwindow*>(window_), GLFW_TRUE);
    }
}

void ARApp::render() {
    TOYAR_PROFILE("ARApp::render");
    
    // Get camera matrices
    Mat4 view_matrix = camera_.getViewMatrix();
    Mat4 projection_matrix = camera_.getProjectionMatrix();
    
    scene_.render(view_matrix, projection_matrix);
}

void ARApp::shutdown() {
    if (window_) {
        glfwDestroyWindow(static_cast<GLFWwindow*>(window_));
        window_ = nullptr;
    }
    glfwTerminate();
    
    TOYAR_INFO("App", "ARApp shutdown complete");
}

} // namespace toyar
