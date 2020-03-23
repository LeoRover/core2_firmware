#include <cstdarg>
#include <cstdio>

#include <hFramework.h>

#include <ros.h>

#include <leo_firmware/config.h>
#include <leo_firmware/logging.h>

extern ros::NodeHandle nh;

void logDebug(const char *format, ...) {
  if (conf.debug_logging) {
    char buffer[256];
    va_list arg;
    va_start(arg, format);
    vsnprintf(buffer, sizeof(buffer), format, arg);
    va_end(arg);
    nh.logdebug(buffer);
#ifdef DEBUG
    sys.log("[DEBUG] %s\r\n", buffer);
#endif
  }
}

void logInfo(const char *format, ...) {
  char buffer[256];
  va_list arg;
  va_start(arg, format);
  vsnprintf(buffer, sizeof(buffer), format, arg);
  va_end(arg);
  nh.loginfo(buffer);
#ifdef DEBUG
  sys.log("[INFO] %s\r\n", buffer);
#endif
}

void logWarn(const char *format, ...) {
  char buffer[256];
  va_list arg;
  va_start(arg, format);
  vsnprintf(buffer, sizeof(buffer), format, arg);
  va_end(arg);
  nh.logwarn(buffer);
#ifdef DEBUG
  sys.log("[WARN] %s\r\n", buffer);
#endif
}

void logError(const char *format, ...) {
  char buffer[256];
  va_list arg;
  va_start(arg, format);
  vsnprintf(buffer, sizeof(buffer), format, arg);
  va_end(arg);
  nh.logerror(buffer);
#ifdef DEBUG
  sys.log("[ERROR] %s\r\n", buffer);
#endif
}

void logFatal(const char *format, ...) {
  char buffer[256];
  va_list arg;
  va_start(arg, format);
  vsnprintf(buffer, sizeof(buffer), format, arg);
  va_end(arg);
  nh.logfatal(buffer);
#ifdef DEBUG
  sys.log("[FATAL] %s\r\n", buffer);
#endif
}
