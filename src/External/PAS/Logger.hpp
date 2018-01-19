/****************************************************************************
Copyright (c) 2013, Ryan Luna, Athanasios Krontiris, Kostas Bekris
All rights reserved.

The Software is available for download and use subject to the terms and
conditions of this License. Access or use of the Software constitutes
acceptance and agreement to the terms and conditions of this License.

Permission to use, copy, modify, and distribute this software and its
documentation for educational, research, and non-profit purposes, without fee,
and without a written agreement is hereby granted.  Permission to incorporate
this software into commercial products may be obtained by contacting the authors.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
  * Neither the names of the authors nor the names of its contributors may be
    used to endorse or promote products derived from this software without
    specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
****************************************************************************/

// Author: Ryan Luna

#ifndef PAS_LOGGER_HPP
#define PAS_LOGGER_HPP

// NOTE: These logging macros are NOT thread safe.
#define PaS_ERROR(fmt, ...)  PaS::log(__FILE__, __LINE__, PaS::LOG_ERROR, fmt, ##__VA_ARGS__)
#define PaS_DEBUG(fmt, ...)  PaS::log(__FILE__, __LINE__, PaS::LOG_DEBUG, fmt, ##__VA_ARGS__)

namespace PaS
{
    // Enumeration of logging levels.
    enum LogLevel
    {
        LOG_ERROR = 0,
        LOG_DEBUG = 1
    };

    // Abstract Logger class.  Instances must implement the log method which records a message
    class Logger
    {
        public:
            Logger() {};
            virtual ~Logger() {};
            virtual void log(const char* text, LogLevel level, const char *source_file, int line) = 0;
    };

    // Default Logger which writes to stdout/stderr.
    class ConsoleLogger : public Logger
    {
        public:
            virtual void log(const char* text, LogLevel level, const char *source_file, int line);
    };

    // Top level log method.  Do not invoke this method directly.  Instead, use one of the
    // logging macros: PAS_ERROR, PAS_DEBUG, etc.
    // NOTE: This method is NOT thread safe.
    void log(const char *file, int line, LogLevel level, const char* m, ...);

    // Set the current logger instance
    void useLogger(Logger* logger);
    // Use the default (console) logger
    void useDefaultLogger(void);
    // Set the minimum threshold for logging.  Default is to log everything
    void setLogLevel(LogLevel level);
}

#endif
