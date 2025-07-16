#include "ar_error.h"
#include <iostream>
#include <fstream>
#include <thread>
#include <unordered_map>
#include <iomanip>
#include <GL/glew.h>

namespace toyar {

// ARError implementation
ARError::ARError(Category category, Severity severity, const std::string& message, 
                const std::string& context, const std::string& suggestion)
    : std::runtime_error(message), category_(category), severity_(severity), 
      context_(context), suggestion_(suggestion),
      timestamp_(std::chrono::steady_clock::now()) {
}

std::string ARError::getFormattedMessage() const {
    std::ostringstream oss;
    oss << "[" << static_cast<int>(severity_) << "] " << what();
    if (!context_.empty()) {
        oss << " (Context: " << context_ << ")";
    }
    if (!suggestion_.empty()) {
        oss << " (Suggestion: " << suggestion_ << ")";
    }
    return oss.str();
}

// OpenGL error checking
namespace gl {

void checkError(const std::string& operation) {
    GLenum error = glGetError();
    if (error != GL_NO_ERROR) {
        std::string error_str = getErrorString(error);
        std::string msg = "OpenGL error: " + error_str;
        if (!operation.empty()) {
            msg += " (during: " + operation + ")";
        }
        throw ARError(ARError::Category::OPENGL, ARError::Severity::ERROR, 
                     msg, operation, "Check OpenGL state and parameters");
    }
}

std::string getErrorString(unsigned int error) {
    switch (error) {
        case GL_NO_ERROR: return "GL_NO_ERROR";
        case GL_INVALID_ENUM: return "GL_INVALID_ENUM";
        case GL_INVALID_VALUE: return "GL_INVALID_VALUE";
        case GL_INVALID_OPERATION: return "GL_INVALID_OPERATION";
        case GL_OUT_OF_MEMORY: return "GL_OUT_OF_MEMORY";
        default: return "Unknown OpenGL error";
    }
}

ErrorGuard::ErrorGuard(const std::string& operation) 
    : operation_(operation), had_error_on_entry_(glGetError() != GL_NO_ERROR) {
}

ErrorGuard::~ErrorGuard() {
    if (!had_error_on_entry_) {
        checkError(operation_);
    }
}

} // namespace gl

// LogSystem implementation
LogSystem& LogSystem::getInstance() {
    static LogSystem instance;
    return instance;
}

LogSystem::LogSystem() {
    addHandler([this](const LogEntry& entry) { consoleHandler(entry); });
}

void LogSystem::addHandler(LogHandler handler) {
    std::lock_guard<std::mutex> lock(mutex_);
    handlers_.push_back(std::move(handler));
}

void LogSystem::clearHandlers() {
    std::lock_guard<std::mutex> lock(mutex_);
    handlers_.clear();
}

void LogSystem::log(Level level, const std::string& category, const std::string& message,
                   const char* file, int line, const char* function) {
    if (level < min_level_) {
        return;
    }
    
    LogEntry entry;
    entry.level = level;
    entry.message = message;
    entry.category = category;
    entry.timestamp = std::chrono::steady_clock::now();
    entry.thread_id = std::this_thread::get_id();
    entry.file = file ? file : "";
    entry.line = line;
    entry.function = function ? function : "";
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Add to recent entries
    recent_entries_.push_back(entry);
    if (recent_entries_.size() > max_recent_entries_) {
        recent_entries_.erase(recent_entries_.begin());
    }
    
    // Call handlers
    for (const auto& handler : handlers_) {
        try {
            handler(entry);
        } catch (...) {
            // Ignore handler exceptions to prevent infinite loops
        }
    }
}

std::vector<LogSystem::LogEntry> LogSystem::getRecentEntries(size_t count) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (count >= recent_entries_.size()) {
        return recent_entries_;
    }
    
    auto start_it = recent_entries_.end() - count;
    return std::vector<LogEntry>(start_it, recent_entries_.end());
}

void LogSystem::setConsoleOutput(bool enabled) {
    console_enabled_ = enabled;
}

void LogSystem::setFileOutput(const std::string& filename) {
    std::lock_guard<std::mutex> lock(mutex_);
    log_file_ = std::make_unique<std::ofstream>(filename, std::ios::app);
}

void LogSystem::consoleHandler(const LogEntry& entry) {
    if (!console_enabled_) {
        return;
    }
    
    const char* level_str = "UNKNOWN";
    switch (entry.level) {
        case Level::TRACE: level_str = "TRACE"; break;
        case Level::DEBUG: level_str = "DEBUG"; break;
        case Level::INFO:  level_str = "INFO";  break;
        case Level::WARN:  level_str = "WARN";  break;
        case Level::ERROR: level_str = "ERROR"; break;
        case Level::CRITICAL: level_str = "CRITICAL"; break;
    }
    
    auto time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    
    std::cout << "[" << std::put_time(std::localtime(&time_t), "%H:%M:%S") 
              << "] [" << level_str << "] [" << entry.category << "] " 
              << entry.message << std::endl;
}

void LogSystem::fileHandler(const LogEntry& entry) {
    if (!log_file_ || !log_file_->is_open()) {
        return;
    }
    
    const char* level_str = "UNKNOWN";
    switch (entry.level) {
        case Level::TRACE: level_str = "TRACE"; break;
        case Level::DEBUG: level_str = "DEBUG"; break;
        case Level::INFO:  level_str = "INFO";  break;
        case Level::WARN:  level_str = "WARN";  break;
        case Level::ERROR: level_str = "ERROR"; break;
        case Level::CRITICAL: level_str = "CRITICAL"; break;
    }
    
    auto time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    
    *log_file_ << "[" << std::put_time(std::localtime(&time_t), "%H:%M:%S") 
               << "] [" << level_str << "] [" << entry.category << "] " 
               << entry.message << std::endl;
    log_file_->flush();
}

// Profiler implementation
Profiler& Profiler::getInstance() {
    static Profiler instance;
    return instance;
}

Profiler::ScopedTimer::ScopedTimer(const std::string& name) 
    : name_(name), start_time_(std::chrono::steady_clock::now()) {
}

Profiler::ScopedTimer::~ScopedTimer() {
    auto end_time = std::chrono::steady_clock::now();
    auto duration = end_time - start_time_;
    Profiler::getInstance().record(name_, duration);
}

void Profiler::record(const std::string& name, std::chrono::nanoseconds duration) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& data = data_[name];
    
    data.name = name;
    data.total_time += duration;
    data.call_count++;
    data.min_time = std::min(data.min_time, duration);
    data.max_time = std::max(data.max_time, duration);
}

std::vector<Profiler::ProfileData> Profiler::getData() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<ProfileData> result;
    result.reserve(data_.size());
    
    for (const auto& pair : data_) {
        result.push_back(pair.second);
    }
    
    return result;
}

void Profiler::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    data_.clear();
}

void Profiler::printSummary() const {
    auto data = getData();
    if (data.empty()) {
        TOYAR_INFO("Profiler", "No profiling data available");
        return;
    }
    
    TOYAR_INFO("Profiler", "=== Performance Summary ===");
    for (const auto& entry : data) {
        std::stringstream ss;
        ss << entry.name << ": avg=" << std::fixed << std::setprecision(3) 
           << entry.getAverageMs() << "ms, total=" << entry.getTotalMs() 
           << "ms, calls=" << entry.call_count;
        TOYAR_INFO("Profiler", ss.str());
    }
}

} // namespace toyar
