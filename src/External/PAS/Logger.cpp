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

#include "Logger.hpp"
using namespace PaS;

#include <iostream>
#include <cstdio>
#include <cstdarg>

#define MAX_BUFFER_SIZE 256

static PaS::ConsoleLogger defaultLogger;
static PaS::Logger *currentLogger = &defaultLogger;
static PaS::LogLevel currentLevel = PaS::LOG_DEBUG;

void PaS::log(const char *file, int line, LogLevel level, const char* m, ...)
{
    if (currentLogger && level <= currentLevel)
    {
        va_list __ap;
        va_start(__ap, m);
        char buf[MAX_BUFFER_SIZE];
        vsnprintf(buf, sizeof(buf), m, __ap);
        va_end(__ap);
        buf[MAX_BUFFER_SIZE - 1] = '\0';

        currentLogger->log(buf, level, file, line);
    }
}

void PaS::setLogLevel(LogLevel level)
{
    currentLevel = level;
}

void PaS::useLogger(PaS::Logger* logger)
{
    currentLogger = logger;
}

void PaS::useDefaultLogger()
{
    currentLogger = &defaultLogger;
}

void PaS::ConsoleLogger::log(const char* text, LogLevel level, const char *source_file, int line)
{
    if (level == LOG_ERROR)
        std::cerr << "[ERROR] " << text << "  [" << source_file << " line " << line << "]" << std::endl;
    else
        std::cout << text << "  [" << source_file << " line " << line << "]" << std::endl;
}
