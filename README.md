# ToyAR - Augmented Reality Platform

A comprehensive, real-world ready AR platform built in C++ with OpenCV, OpenGL, and modern C++ features.

## Features

- **Camera Integration**: Real-time camera feed processing using OpenCV
- **Marker Detection**: ArUco marker detection and pose estimation
- **3D Object Rendering**: OpenGL-based 3D object rendering with proper lighting
- **Scene Management**: Complete scene graph with camera, objects, and planes
- **Mathematical Foundation**: Custom linear algebra library optimized for AR applications
- **Advanced GUI System**: Clean and functional interface with grid overlays
- **Surface Detection**: Real-time surface detection with visual feedback
- **Interactive Controls**: Intuitive controls for environment scanning and surface anchoring
- **Cross-Platform**: Supports Linux, Windows, and can be compiled to WebAssembly

## Architecture

### Core Components

1. **ar_la.h**: Mathematical foundation with Vec3, Mat4, and transformation utilities
2. **ar_core.h**: Core AR functionality including camera tracking and intrinsics
3. **ar_camera.h/cpp**: Camera management, marker detection, and pose estimation
4. **ar_object.h/cpp**: 3D object representation and rendering
5. **ar_plane.h/cpp**: Plane detection and rendering for surface tracking
6. **ar_scene.h/cpp**: Scene management and rendering pipeline
7. **ar_gui.h/cpp**: Clean GUI system with functional overlays

### Key Features

- **Real-time Performance**: Optimized for real-time AR applications
- **Modern OpenGL**: Uses modern OpenGL with shaders for efficient rendering
- **ArUco Markers**: Supports ArUco marker detection for robust tracking
- **Clean GUI Interface**: Professional-grade interface with grid overlay
- **Surface Detection Feedback**: Real-time visual feedback for detected surfaces
- **Extensible Design**: Easy to add new object types and rendering methods

## Dependencies

- **OpenCV 4.x**: For computer vision and marker detection
- **OpenGL 3.3+**: For 3D rendering
- **GLEW**: OpenGL extension wrangler
- **GLUT/FreeGLUT**: Window management and input handling
- **GLM**: OpenGL Mathematics library
- **CMake 3.10+**: Build system

## Building

### Ubuntu/Debian
```bash
# Install dependencies
sudo apt update
sudo apt install build-essential cmake
sudo apt install libopencv-dev libgl1-mesa-dev libglu1-mesa-dev
sudo apt install libglew-dev freeglut3-dev libglm-dev

# Build the project
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Running
```bash
# Run the demo application
./toyar_demo
```

## Usage

### Basic Scene Setup
```cpp
#include "ar_scene.h"
#include "ar_object.h"

// Create scene
ARScene scene;
scene.initialize(1280, 720);

// Set up camera
Vec3 camPos = {0.0, 0.0, 2.0};
scene.camera.setPosition(camPos);
scene.camera.openCamera(0); // Open default camera

// Add objects
ARObject* cube = new ARObject("Cube", {1.0, 0.0, 0.0});
scene.addObject(cube);

// Render loop
scene.renderScene();
```

### Marker-based Tracking
The platform automatically detects ArUco markers in the camera feed and can track their pose in 3D space, allowing for robust augmented reality experiences.

### Basic Scene Setup with GUI
```cpp
#include "ar_scene.h"
#include "ar_object.h"

// Create scene with GUI
ARScene scene;
scene.initialize(1280, 720);

// Set up camera
Vec3 camPos = {0.0, 0.0, 2.0};
scene.camera.setPosition(camPos);
scene.camera.openCamera(0); // Open default camera

// The application will automatically start looking for surfaces
// in the environment

// Add objects
ARObject* cube = new ARObject("Cube", {1.0, 0.0, 0.0});
scene.addObject(cube);

// Main render loop with GUI
while (running) {
    scene.update(deltaTime);
    scene.renderScene(); // Includes GUI overlays
}
```

### Clean GUI Features
The ToyAR platform includes a clean and functional GUI system that provides:

- **Grid Overlay**: Simple grid overlay for visualizing detected surfaces
- **Surface Detection**: Visual indicators when surfaces are detected
- **Real-time Feedback**: Status messages and progress indicators
- **Interactive Elements**: Crosshair targeting and surface anchor creation

## Controls

- **ESC**: Exit the application
- **G**: Toggle grid display on/off
- **S**: Restart environment scanning
- **Left Click**: Create surface anchor when surface is detected
- **Camera Input**: Automatic marker detection and tracking

## Project Structure

```
toyar/
├── CMakeLists.txt          # Build configuration
├── README.md              # This file
├── toyar/
│   ├── include/           # Header files
│   │   ├── ar_camera.h
│   │   ├── ar_core.h
│   │   ├── ar_gui.h
│   │   ├── ar_la.h
│   │   ├── ar_object.h
│   │   ├── ar_plane.h
│   │   └── ar_scene.h
│   └── src/               # Source files
│       ├── ar_camera.cpp
│       ├── ar_gui.cpp
│       ├── ar_la.cpp
│       ├── ar_object.cpp
│       ├── ar_plane.cpp
│       ├── ar_scene.cpp
│       └── main.cpp
└── build/                 # Build output (generated)
```

## Future Enhancements

- SLAM (Simultaneous Localization and Mapping)
- Multiple marker tracking
- Physics integration
- Mobile platform support (Android/iOS)
- Advanced lighting and materials
- Occlusion handling
- Performance optimization for mobile devices

## License

This project is open source and available under the MIT License.

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.
