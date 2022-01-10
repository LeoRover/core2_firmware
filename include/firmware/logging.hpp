#ifndef LEO_FIRMWARE_INCLUDE_LOGGING_H_
#define LEO_FIRMWARE_INCLUDE_LOGGING_H_

void logDebug(const char *format, ...);
void logInfo(const char *format, ...);
void logWarn(const char *format, ...);
void logError(const char *format, ...);
void logFatal(const char *format, ...);

#endif  // LEO_FIRMWARE_INCLUDE_LOGGING_H_
