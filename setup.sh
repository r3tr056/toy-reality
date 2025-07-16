#!/bin/bash
set -e

# ToyAR System Setup Script
# Installs dependencies and configures the development environment
# Supports Ubuntu 20.04+, Debian 11+, and other Debian-based systems

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Helper functions
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "${CYAN}$1${NC}"
}

# Check if running as root
check_root() {
    if [[ $EUID -eq 0 ]]; then
        print_error "This script should not be run as root"
        print_info "Run without sudo, the script will ask for permissions when needed"
        exit 1
    fi
}

# Detect OS and version
detect_os() {
    if [[ -f /etc/os-release ]]; then
        . /etc/os-release
        OS=$ID
        VER=$VERSION_ID
        print_info "Detected OS: $PRETTY_NAME"
    else
        print_error "Cannot detect OS version"
        exit 1
    fi
}

# Check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Install package if not already installed
install_package() {
    local package=$1
    local check_cmd=${2:-$package}
    
    if command_exists "$check_cmd"; then
        print_info "$package is already installed"
        return 0
    fi
    
    print_info "Installing $package..."
    sudo apt-get install -y "$package"
}

# Install packages from list
install_packages() {
    local packages=("$@")
    print_info "Installing packages: ${packages[*]}"
    sudo apt-get install -y "${packages[@]}"
}

# Main setup function
main() {
    echo "============================================================"
    print_header "ToyAR Development Environment Setup"
    echo "============================================================"
    echo ""
    
    check_root
    detect_os
    
    # Check if supported OS
    case $OS in
        ubuntu|debian|pop|elementary|linuxmint)
            print_success "Supported OS detected"
            ;;
        *)
            print_warning "Untested OS: $OS"
            print_info "This script is designed for Debian-based systems"
            read -p "Continue anyway? (y/N): " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                exit 1
            fi
            ;;
    esac
    
    echo ""
    print_header "Step 1: Updating package lists"
    echo "============================================================"
    
    print_info "Updating apt package lists..."
    sudo apt-get update
    
    echo ""
    print_header "Step 2: Installing build tools"
    echo "============================================================"
    
    # Essential build tools
    BUILD_TOOLS=(
        "build-essential"
        "cmake"
        "ninja-build"
        "pkg-config"
        "git"
        "wget"
        "curl"
    )
    
    install_packages "${BUILD_TOOLS[@]}"
    
    echo ""
    print_header "Step 3: Installing OpenGL and graphics libraries"
    echo "============================================================"
    
    # OpenGL and graphics
    GRAPHICS_LIBS=(
        "libgl1-mesa-dev"
        "libglu1-mesa-dev"
        "freeglut3-dev"
        "libglew-dev"
        "libglfw3-dev"
        "libxrandr-dev"
        "libxinerama-dev"
        "libxcursor-dev"
        "libxi-dev"
    )
    
    install_packages "${GRAPHICS_LIBS[@]}"
    
    echo ""
    print_header "Step 4: Installing OpenCV"
    echo "============================================================"
    
    # OpenCV and computer vision
    OPENCV_LIBS=(
        "libopencv-dev"
        "libopencv-contrib-dev"
        "python3-opencv"
    )
    
    install_packages "${OPENCV_LIBS[@]}"
    
    # Verify OpenCV installation
    if pkg-config --exists opencv4; then
        OPENCV_VERSION=$(pkg-config --modversion opencv4)
        print_success "OpenCV $OPENCV_VERSION installed successfully"
    else
        print_warning "OpenCV pkg-config not found, but packages were installed"
    fi
    
    echo ""
    print_header "Step 5: Installing GLM (OpenGL Mathematics)"
    echo "============================================================"
    
    install_package "libglm-dev" "glm"
    
    echo ""
    print_header "Step 6: Installing testing frameworks"
    echo "============================================================"
    
    # Testing libraries
    TEST_LIBS=(
        "libgtest-dev"
        "libgmock-dev"
        "libbenchmark-dev"
    )
    
    for lib in "${TEST_LIBS[@]}"; do
        if ! dpkg -l | grep -q "$lib"; then
            print_info "Installing $lib..."
            sudo apt-get install -y "$lib" || print_warning "Failed to install $lib"
        else
            print_info "$lib is already installed"
        fi
    done
    
    # Build and install GTest if needed
    if [[ ! -f /usr/lib/libgtest.a && ! -f /usr/local/lib/libgtest.a ]]; then
        print_info "Building GTest from source..."
        TEMP_DIR=$(mktemp -d)
        cd "$TEMP_DIR"
        
        if [[ -d /usr/src/gtest ]]; then
            sudo cp -r /usr/src/gtest .
            cd gtest
            sudo cmake .
            sudo make
            sudo cp lib/*.a /usr/local/lib/ 2>/dev/null || sudo cp *.a /usr/local/lib/
            print_success "GTest built and installed"
        else
            print_warning "GTest source not found, skipping manual build"
        fi
        
        cd - > /dev/null
        rm -rf "$TEMP_DIR"
    fi
    
    echo ""
    print_header "Step 7: Installing optional development tools"
    echo "============================================================"
    
    # Development tools
    DEV_TOOLS=(
        "clang"
        "clang-format"
        "clang-tidy"
        "cppcheck"
        "valgrind"
        "gdb"
        "doxygen"
        "graphviz"
    )
    
    for tool in "${DEV_TOOLS[@]}"; do
        if ! command_exists "$tool"; then
            print_info "Installing $tool..."
            sudo apt-get install -y "$tool" || print_warning "Failed to install $tool"
        else
            print_info "$tool is already installed"
        fi
    done
    
    echo ""
    print_header "Step 8: Installing additional AR/CV libraries"
    echo "============================================================"
    
    # Additional libraries for AR development
    AR_LIBS=(
        "libeigen3-dev"
        "libceres-dev"
        "libg2o-dev"
        "libpcl-dev"
        "libassimp-dev"
        "libfreeimage-dev"
    )
    
    for lib in "${AR_LIBS[@]}"; do
        if ! dpkg -l | grep -q "$lib"; then
            print_info "Installing $lib..."
            sudo apt-get install -y "$lib" || print_warning "Failed to install $lib (optional)"
        else
            print_info "$lib is already installed"
        fi
    done
    
    echo ""
    print_header "Step 9: Configuring development environment"
    echo "============================================================"
    
    # Create useful aliases
    BASHRC_ADDITIONS="
# ToyAR Development Aliases
alias toyar-build='./build.sh'
alias toyar-debug='./build.sh Debug'
alias toyar-clean='rm -rf build && mkdir build'
alias toyar-run='./build/toyar_demo'
alias toyar-test='cd build && make test'
"
    
    if ! grep -q "ToyAR Development Aliases" ~/.bashrc; then
        print_info "Adding development aliases to ~/.bashrc"
        echo "$BASHRC_ADDITIONS" >> ~/.bashrc
        print_success "Aliases added to ~/.bashrc"
    else
        print_info "Development aliases already exist in ~/.bashrc"
    fi
    
    # Set up git hooks directory (if in git repo)
    if [[ -d .git ]]; then
        print_info "Setting up git hooks..."
        mkdir -p .git/hooks
        
        # Pre-commit hook for code formatting
        cat > .git/hooks/pre-commit << 'EOF'
#!/bin/bash
# Check if clang-format is available
if command -v clang-format >/dev/null 2>&1; then
    # Format all staged C++ files
    for file in $(git diff --cached --name-only --diff-filter=ACM | grep -E '\.(cpp|hpp|cc|hh|c\+\+|h\+\+|cxx|hxx|c|h)$'); do
        if [[ -f "$file" ]]; then
            clang-format -i "$file"
            git add "$file"
        fi
    done
fi
EOF
        chmod +x .git/hooks/pre-commit
        print_success "Git pre-commit hook installed"
    fi
    
    echo ""
    print_header "Step 10: Verification"
    echo "============================================================"
    
    # Verify installations
    print_info "Verifying installations..."
    
    # Check essential tools
    ESSENTIAL_TOOLS=("cmake" "ninja" "pkg-config" "g++" "clang++")
    for tool in "${ESSENTIAL_TOOLS[@]}"; do
        if command_exists "$tool"; then
            if [[ "$tool" == "cmake" ]]; then
                VERSION=$(cmake --version | head -n1 | cut -d' ' -f3)
                print_success "$tool is installed (version $VERSION)"
            elif [[ "$tool" == "g++" ]]; then
                VERSION=$(g++ --version | head -n1 | cut -d' ' -f3)
                print_success "$tool is installed (version $VERSION)"
            else
                print_success "$tool is installed"
            fi
        else
            print_error "$tool is not installed or not in PATH"
        fi
    done
    
    # Check libraries
    print_info "Checking library installations..."
    
    # OpenCV
    if pkg-config --exists opencv4; then
        OPENCV_VER=$(pkg-config --modversion opencv4)
        print_success "OpenCV $OPENCV_VER is properly configured"
    else
        print_warning "OpenCV pkg-config not found"
    fi
    
    # OpenGL
    if [[ -f /usr/include/GL/gl.h ]]; then
        print_success "OpenGL headers found"
    else
        print_warning "OpenGL headers not found"
    fi
    
    # GLFW
    if pkg-config --exists glfw3; then
        GLFW_VER=$(pkg-config --modversion glfw3)
        print_success "GLFW $GLFW_VER is properly configured"
    else
        print_warning "GLFW pkg-config not found"
    fi
    
    echo ""
    print_header "Setup Summary"
    echo "============================================================"
    
    print_success "ToyAR development environment setup completed!"
    echo ""
    print_info "Next steps:"
    echo "  1. Source your bashrc: source ~/.bashrc"
    echo "  2. Build the project: ./build.sh"
    echo "  3. Run the demo: ./build/toyar_demo"
    echo ""
    print_info "Development aliases added:"
    echo "  toyar-build   - Build the project"
    echo "  toyar-debug   - Build in debug mode"
    echo "  toyar-clean   - Clean build directory"
    echo "  toyar-run     - Run the demo application"
    echo ""
    print_info "For more information, see README.md"
    
    echo "============================================================"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --help|-h)
            echo "ToyAR Development Environment Setup"
            echo ""
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  --help, -h     Show this help message"
            echo "  --minimal      Install only essential packages"
            echo "  --no-optional  Skip optional development tools"
            echo ""
            echo "This script installs all necessary dependencies for ToyAR development"
            echo "including OpenCV, OpenGL, CMake, and testing frameworks."
            exit 0
            ;;
        --minimal)
            MINIMAL_INSTALL=true
            shift
            ;;
        --no-optional)
            NO_OPTIONAL=true
            shift
            ;;
        *)
            print_error "Unknown option: $1"
            print_info "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Run main setup
main
