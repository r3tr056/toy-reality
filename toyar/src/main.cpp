#include "ar_scene.h"
#include "ar_object.h"
#include "ar_la.h"
#include <iostream>
#include <GL/glut.h>
#include <chrono>

using namespace toyar;

ARScene* scene = nullptr;
std::chrono::steady_clock::time_point lastFrameTime;

void display() {
    if (scene) {
        // Calculate delta time
        auto currentTime = std::chrono::steady_clock::now();
        auto deltaTime = std::chrono::duration<float>(currentTime - lastFrameTime).count();
        lastFrameTime = currentTime;
        
        scene->update(deltaTime);
        scene->renderScene();
    }
    glutSwapBuffers();
}

void reshape(int width, int height) {
    glViewport(0, 0, width, height);
    if (scene) {
        scene->updateProjectionMatrix(45.0f, (float)width/height, 0.1f, 100.0f);
    }
}

void keyboard(unsigned char key, int /*x*/, int /*y*/) {
    if (key == 27) { // ESC
        delete scene;
        exit(0);
    } else if (key == 'g' || key == 'G') { // Toggle grid
        if (scene) {
            scene->toggleGrid();
        }
    } else if (key == 's' || key == 'S') { // Start scanning
        if (scene) {
            scene->startEnvironmentScanning();
        }
    } else if (key == 'm' || key == 'M') { // Reset motion tracking
        if (scene) {
            scene->motionTracker.reset();
            std::cout << "Motion tracking reset" << std::endl;
        }
    } else if (key == 't' || key == 'T') { // Show tracking info
        if (scene) {
            TrackingState state = scene->motionTracker.getTrackingState();
            Pose6DOF pose = scene->motionTracker.getCurrentPose();
            std::cout << "Tracking state: " << (int)state 
                      << " Position: (" << pose.position.x 
                      << ", " << pose.position.y 
                      << ", " << pose.position.z << ")" << std::endl;
        }
    }
}

void mouse(int button, int state, int /*x*/, int /*y*/) {
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        // Handle surface anchor creation when scanning is complete
        if (scene && !scene->isScanning()) {
            // Convert screen coordinates to world coordinates and create surface anchor
            Vec3 surfacePos(0.0f, 0.0f, 0.0f); // Simplified for demo
            scene->handleSurfaceDetection(surfacePos);
        }
    }
}

void idle() {
    glutPostRedisplay();
}

int main(int argc, char** argv) {
    // Initialize GLUT
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(1280, 720);
    glutCreateWindow("ToyAR - Augmented Reality Platform");

    // Set up callbacks
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouse);
    glutIdleFunc(idle);

    // Create the AR scene
    scene = new ARScene();
    if (!scene->initialize(1280, 720)) {
        std::cerr << "Failed to initialize AR scene" << std::endl;
        return -1;
    }

    Vec3 camPos = {0.0, 0.0, 2.0};
    Vec3 camOrientation = {0.0, 0.0, -1.0};

    scene->camera.setPosition(camPos);
    scene->camera.setOrientation(camOrientation);

    // Try to open camera
    if (!scene->camera.openCamera(0)) {
        std::cout << "Warning: Could not open camera. Running in demo mode with simulated surface detection." << std::endl;
    }

    // Add some demo objects
    ARObject* obj1 = new ARObject("Cube", {1.0, 0.0, 0.0});
    ARObject* obj2 = new ARObject("Sphere", {0.0, 1.0, 0.0});

    scene->addObject(obj1);
    scene->addObject(obj2);

    // move the cube
    obj1->setPosition({2.0, 0.0, 0.0});

    // Initialize timing
    lastFrameTime = std::chrono::steady_clock::now();

    // Print controls
    std::cout << "\n=== ToyAR Controls ===" << std::endl;
    std::cout << "ESC - Exit application" << std::endl;
    std::cout << "G   - Toggle grid display" << std::endl;
    std::cout << "S   - Restart environment scanning" << std::endl;
    std::cout << "M   - Reset motion tracking" << std::endl;
    std::cout << "T   - Show tracking information" << std::endl;
    std::cout << "Left Click - Create surface anchor (when surface detected)" << std::endl;
    std::cout << "=====================\n" << std::endl;

    // Print initial scene state
    scene->drawScene();

    // Start the main loop
    glutMainLoop();

    return 0;
}