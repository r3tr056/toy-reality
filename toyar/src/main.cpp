#include "ar_scene_modern.h"
#include "ar_error.h"
#include <iostream>
#include <memory>

int main(int argc, char* argv[]) {
    TOYAR_INFO("Main", "Starting ToyAR Demo Application");
    
    try {
        // Create the AR application
        auto app = std::make_unique<toyar::ARApp>();
        
        // Initialize the application
        app->initialize();
        TOYAR_INFO("Main", "AR application initialized successfully");
        
        // Run the main loop
        app->run();
        
        TOYAR_INFO("Main", "AR application shutting down");
        
    } catch (const std::exception& e) {
        TOYAR_CRITICAL("Main", std::string("Unhandled exception: ") + e.what());
        return -1;
    }
    
    return 0;
}
