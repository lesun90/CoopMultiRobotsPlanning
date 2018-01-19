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

#ifndef PUSH_AND_SWAP_INSTANCE_HPP
#define PUSH_AND_SWAP_INSTANCE_HPP

#include "Agent.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/trim_all.hpp>
#include <boost/lexical_cast.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cassert>

// Definition of an instance of a multi-agent path planning problem
// Reads an instance file defining the problem.  The class
// contains a graph filename, and a list of agents, their starts,
// and their goals on the graph
class Instance
{
    public:
        Instance(const char* filename) { parse(filename); }

        void parse(const char* filename)
        {
            std::ifstream fin;
            fin.open(filename);
            if (!fin)
            {
                std::cerr << "ERROR: Could not open " << filename << std::endl;
                exit(1);
            }

            std::string line;
            while (!fin.eof())
            {
                std::getline(fin, line);
                parseLine (line);
            }
            fin.close();
        }

        void parseLine(std::string& line)
        {
            boost::trim_all(line);
            if (line.size() == 0)
                return;

            std::vector<std::string> strs;
            boost::split(strs, line, boost::is_any_of(":"));  // split by colon

            if (strs.size() == 1)
            {
                boost::split(strs, line, boost::is_any_of("="));  // split by equals
                boost::trim_all(strs[0]);
                if (strs[0] == "graph" && strs.size() == 2)
                {
                    boost::trim_all(strs[1]);
                    graphFilename = strs[1];
                }
                else
                {
                    std::cerr << "WARNING: Unknown line: " << strs[0] << std::endl;
                }
            }
            else
            {
                assert(strs.size() == 2);
                if (strs[0] == "agent")
                {
                    std::vector<std::string> conf;
                    boost::trim_all(strs[1]);
                    boost::split(conf, strs[1], boost::is_any_of(","));  // split by comma
                    assert(conf.size() == 2);

                    // Start
                    std::string start = conf[0];
                    std::vector<std::string> stconf;
                    boost::split(stconf, start, boost::is_any_of("="));
                    assert(stconf.size() == 2);
                    boost::trim_all(stconf[0]);
                    assert(stconf[0] == "start");
                    boost::trim_all(stconf[1]);
                    unsigned int st = boost::lexical_cast<unsigned int> (stconf[1]);

                    // Goal
                    std::string goal = conf[1];
                    std::vector<std::string> glconf;
                    boost::split(glconf, goal, boost::is_any_of("="));
                    assert(glconf.size() == 2);
                    boost::trim_all(glconf[0]);
                    assert(glconf[0] == "goal");
                    boost::trim_all(glconf[1]);
                    unsigned int gl = boost::lexical_cast<unsigned int> (glconf[1]);

                    agents.push_back(PaS::Agent(agents.size(), st, gl));
                }
            }
        }

        unsigned int numAgents(void) const { return agents.size(); }
        PaS::Agent getAgent(unsigned int idx) { return agents[idx]; }
        std::vector<PaS::Agent> getAgents(void) { return agents; }

        std::string getGraphFilename (void) const { return graphFilename; }

    protected:
        std::string graphFilename;
        std::vector<PaS::Agent> agents;
};

#endif
