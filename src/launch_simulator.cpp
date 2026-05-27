// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "simulator.h"

#include <fstream>
#include <iostream>

using namespace xbot_mujoco;

namespace {

bool file_exists(const std::string& path)
{
    return std::ifstream(path).good();
}

void print_usage(const char* executable)
{
    std::cerr << "Usage: " << executable << " <mujoco_xml_path> <xbot2_config_path>\n";
}

} // namespace

// run the full simulation loop
int main(int argc, char** argv)
{
    if(argc == 2 && (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help"))
    {
        print_usage(argv[0]);
        return 0;
    }

    // request loadmodel if file given (otherwise drag-and-drop)
    if(argc < 3)
    {
        print_usage(argv[0]);
        return 2;
    }

    const std::string model_path = argv[1];
    const std::string xbot2_cfg_path = argv[2];

    if(!file_exists(model_path))
    {
        std::cerr << "MuJoCo XML file does not exist: " << model_path << "\n";
        return 2;
    }

    if(!file_exists(xbot2_cfg_path))
    {
        std::cerr << "XBot2 config file does not exist: " << xbot2_cfg_path << "\n";
        return 2;
    }

    char filename[mj::Simulate::kMaxFilenameLength] = {};
    mju_strncpy(filename, model_path.c_str(), mj::Simulate::kMaxFilenameLength);

    bool headless = false;
    // ros::init(argc, argv, "mujoco_ros");
    // ros::NodeHandle nh("");
    // run(filename,xbot2_cfg_path,nh,headless); // run everything
    run(filename,xbot2_cfg_path,headless); // run everything

    return 0;
}
