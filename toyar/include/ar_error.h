#ifndef AR_ERROR_H_
#define AR_ERROR_H_

#include <stdexcept>
#include <string>
#include <sstream>
#include <chrono>
#include <functional>
#include <mutex>
#include <vector>
#include <memory>
#include <thread>
#include <iomanip>

namespace toyar {

/**
 * @brief Enhanced error handling with context and recovery suggestions
 */
class ARError : public std::runtime_error {
public:
    enum class Severity {
        INFO,
        WARNING,
        ERROR,
        CRITICAL
    };
    
    enum class Category {
        GENERAL,
        CAMERA,
        TRACKING,
        RENDERING,
        IO,
        MEMORY,
        OPENGL
    };
    
    ARError(Category category, Severity severity, const std::string& message, 
           const std::string& context = "", const std::string& suggestion = "");
    
    Category getCategory() const { return category_; }
    Severity getSeverity() const { return severity_; }
    const std::string& getContext() const { return context_; }
    const std::string& getSuggestion() const { return suggestion_; }
    auto getTimestamp() const { return timestamp_; }
    
    std::string getFormattedMessage() const;

private:
    Category category_;
    Severity severity_;
    std::string context_;
    std::string suggestion_;
    std::chrono::steady_clock::time_point timestamp_;
};

/**
 * @brief Macro helpers for throwing specific errors
 */
#define TOYAR_THROW_CAMERA_ERROR(msg, context, suggestion) \
    throw ARError(ARError::Category::CAMERA, ARError::Severity::ERROR, msg, context, suggestion)

#define TOYAR_THROW_TRACKING_ERROR(msg, context, suggestion) \
    throw ARError(ARError::Category::TRACKING, ARError::Severity::ERROR, msg, context, suggestion)

#define TOYAR_THROW_RENDER_ERROR(msg, context, suggestion) \
    throw ARError(ARError::Category::RENDERING, ARError::Severity::ERROR, msg, context, suggestion)

#define TOYAR_THROW_GL_ERROR(msg, context, suggestion) \
    throw ARError(ARError::Category::OPENGL, ARError::Severity::ERROR, msg, context, suggestion)

/**
 * @brief OpenGL error checking utilities
 */
namespace gl {
    
    /**
     * @brief Check for OpenGL errors and throw if found
     */
    void checkError(const std::string& operation = "");
    
    /**
     * @brief Get string description of OpenGL error
     */
    std::string getErrorString(unsigned int error);
    
    /**
     * @brief RAII guard for OpenGL error checking
     */
    class ErrorGuard {
    public:
        explicit ErrorGuard(const std::string& operation);
        ~ErrorGuard();
        
    private:
        std::string operation_;
        bool had_error_on_entry_;
    };
    
} // namespace gl

/**
 * @brief Macro for automatic OpenGL error checking
 */
#define TOYAR_GL_CHECK(operation) \
    do { \
        operation; \
        toyar::gl::checkError(#operation); \
    } while(0)

#define TOYAR_GL_GUARD(operation) \
    toyar::gl::ErrorGuard guard(#operation); \
    operation

/**
 * @brief Enhanced logging system with multiple outputs and filtering
 */
class LogSystem {
public:
    enum class Level {
        TRACE = 0,
        DEBUG = 1,
        INFO = 2,
        WARN = 3,
        ERROR = 4,
        CRITICAL = 5
    };
    
    struct LogEntry {
        Level level;
        std::string message;
        std::string category;
        std::chrono::steady_clock::time_point timestamp;
        std::thread::id thread_id;
        std::string file;
        int line;
        std::string function;
    };
    
    using LogHandler = std::function<void(const LogEntry&)>;
    
    static LogSystem& getInstance();
    
    /**
     * @brief Set minimum log level
     */
    void setLevel(Level level) { min_level_ = level; }
    Level getLevel() const { return min_level_; }
    
    /**
     * @brief Add a log handler
     */
    void addHandler(LogHandler handler);
    
    /**
     * @brief Remove all handlers
     */
    void clearHandlers();
    
    /**
     * @brief Log a message
     */
    void log(Level level, const std::string& category, const std::string& message,
            const char* file = __builtin_FILE(), int line = __builtin_LINE(),
            const char* function = __builtin_FUNCTION());
    
    /**
     * @brief Get recent log entries
     */
    std::vector<LogEntry> getRecentEntries(size_t count = 100) const;
    
    /**
     * @brief Enable/disable console output
     */
    void setConsoleOutput(bool enabled);
    
    /**
     * @brief Enable/disable file output
     */
    void setFileOutput(const std::string& filename);

private:
    LogSystem();
    ~LogSystem() = default;
    
    std::vector<LogHandler> handlers_;
    Level min_level_ = Level::INFO;
    mutable std::mutex mutex_;
    std::vector<LogEntry> recent_entries_;
    size_t max_recent_entries_ = 1000;
    
    // Built-in handlers
    void consoleHandler(const LogEntry& entry);
    void fileHandler(const LogEntry& entry);
    
    bool console_enabled_ = true;
    std::unique_ptr<std::ofstream> log_file_;
};

/**
 * @brief Logging macros with automatic file/line information
 */
#define TOYAR_LOG(level, category, message) \
    toyar::LogSystem::getInstance().log(level, category, message, __FILE__, __LINE__, __FUNCTION__)

#define TOYAR_TRACE(category, message) TOYAR_LOG(toyar::LogSystem::Level::TRACE, category, message)
#define TOYAR_DEBUG(category, message) TOYAR_LOG(toyar::LogSystem::Level::DEBUG, category, message)
#define TOYAR_INFO(category, message) TOYAR_LOG(toyar::LogSystem::Level::INFO, category, message)
#define TOYAR_WARN(category, message) TOYAR_LOG(toyar::LogSystem::Level::WARN, category, message)
#define TOYAR_ERROR(category, message) TOYAR_LOG(toyar::LogSystem::Level::ERROR, category, message)
#define TOYAR_CRITICAL(category, message) TOYAR_LOG(toyar::LogSystem::Level::CRITICAL, category, message)

/**
 * @brief Performance profiler for tracking bottlenecks
 */
class Profiler {
public:
    struct ProfileData {
        std::string name;
        std::chrono::nanoseconds total_time{0};
        std::chrono::nanoseconds min_time{std::chrono::nanoseconds::max()};
        std::chrono::nanoseconds max_time{0};
        size_t call_count = 0;
        
        double getAverageMs() const {
            return call_count > 0 ? 
                std::chrono::duration<double, std::milli>(total_time).count() / call_count : 0.0;
        }
        
        double getTotalMs() const {
            return std::chrono::duration<double, std::milli>(total_time).count();
        }
    };
    
    class ScopedTimer {
    public:
        ScopedTimer(const std::string& name);
        ~ScopedTimer();
        
    private:
        std::string name_;
        std::chrono::steady_clock::time_point start_time_;
    };
    
    static Profiler& getInstance();
    
    /**
     * @brief Record timing data
     */
    void record(const std::string& name, std::chrono::nanoseconds duration);
    
    /**
     * @brief Get profiling data
     */
    std::vector<ProfileData> getData() const;
    
    /**
     * @brief Clear all data
     */
    void clear();
    
    /**
     * @brief Print summary to log
     */
    void printSummary() const;

private:
    Profiler() = default;
    
    mutable std::mutex mutex_;
    std::unordered_map<std::string, ProfileData> data_;
};

/**
 * @brief Macro for automatic profiling
 */
#define TOYAR_PROFILE(name) toyar::Profiler::ScopedTimer timer(name)
#define TOYAR_PROFILE_FUNCTION() TOYAR_PROFILE(__FUNCTION__)

} // namespace toyar

#endif // AR_ERROR_H_
