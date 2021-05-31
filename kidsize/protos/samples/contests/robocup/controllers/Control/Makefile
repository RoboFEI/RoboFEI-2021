# Copyright 1996-2020 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

### Generic Makefile.include for Webots controllers, physics plugins, robot
### window libraries, remote control libraries and other libraries
### to be used with GNU make
###
### Platforms: Windows, macOS, Linux
### Languages: C, C++
###
### Authors: Olivier Michel, Yvan Bourquin, Fabien Rohrer
###          Edmund Ronald, Sergei Poskriakov
###
###-----------------------------------------------------------------------------
###
### This file is meant to be included from the Makefile files located in the
### Webots projects subdirectories. It is possible to set a number of variables
### to customize the build process, i.e., add source files, compilation flags,
### include paths, libraries, etc. These variables should be set in your local
### Makefile just before including this Makefile.include. This Makefile.include
### should never be modified.
###
### Here is a description of the variables you may set in your local Makefile:
###
### ---- C Sources ----
### if your program uses several C source files:
### C_SOURCES = my_plugin.c my_clever_algo.c my_graphics.c
###
### ---- C++ Sources ----
### if your program uses several C++ source files:
### CXX_SOURCES = my_plugin.cc my_clever_algo.cpp my_graphics.c++
###
### ---- Compilation options ----
### if special compilation flags are necessary:
### CFLAGS = -Wno-unused-result
###
### ---- Linked libraries ----
### if your program needs additional libraries:
### INCLUDE = -I"/my_library_path/include"
### LIBRARIES = -L"/path/to/my/library" -lmy_library -lmy_other_library
###
### ---- Linking options ----
### if special linking flags are needed:
### LFLAGS = -s
###
### ---- Webots included libraries ----
### if you want to use the Webots C API in your C++ controller program:
### USE_C_API = true
###
### ---- Debug mode ----
### if you want to display the gcc command line for compilation and link, as
### well as the rm command details used for cleaning:
### VERBOSE = 1
###
###-----------------------------------------------------------------------------

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
INCLUDE = -I"./Linux/include" -I"./Framework/include"
LIBRARIES = -Wno-write-strings -lpthread -lrt -lboost_filesystem#-L"./Linux/project/control/main.cpp" -L"./Linux/project/control/kbhit.cpp" -L"./Linux/project/control/ReadConfig.cpp" -L"./Linux/project/control/ActionMove.cpp" -L"./Linux/project/control/GaitMove.cpp"
CXXFLAGS=-std=c++98
CXX_SOURCES = $(wildcard *.cpp) 

include $(WEBOTS_HOME_PATH)/resources/Makefile.include


# @author Isaac Jesus da Silva - ROBOFEI-HT - FEI
# @version V0.0.3
# @created 04/07/2014
# @Modified 14/07/2016
# 
# Para Help acesse http://www.cmake.org/cmake/help/v2.8.0/cmake.html#command:link_directories

#find_package(Boost REQUIRED COMPONENTS program_options)

#include_directories(
#  include
#  #${RoboFEI-HT_SOURCE_DIR}/Blackboard/include
#  ./Control/Linux/include
#  ./Framework/include
#  ./Linux/project/control
#  ./Framework/Framework_Dynamixel/include/dynamixel_sdk
#)

#include_directories(include)
#add_library(libcontrol SHARED ./Linux/project/control/main.cpp ./Linux/project/control/kbhit.cpp ./Linux/project/control/ReadConfig.cpp ./Linux/project/control/ActionMove.cpp ./Linux/project/control/GaitMove.cpp)
#install(TARGETS libcontrol DESTINATION ${RoboFEI-HT_BINARY_DIR}/lib)

#link_directories(
#  ${RoboFEI-HT_BINARY_DIR}/Blackboard}
#  ${RoboFEI-HT_BINARY_DIR}/Dynamixel}
#)

#MESSAGE(STATUS "CMAKE_SOURCE_DIR:         " ${RoboFEI-HT_SOURCE_DIR})
#MESSAGE(STATUS "CMAKE_BINARY_DIR:         " ${RoboFEI-HT_BINARY_DIR})
#execute_process(COMMAND pwd)

##cria executável
#add_executable(control ./Linux/project/control/main.cpp)

##adiciona as dependências
#add_dependencies(control libcontrol blackboard)

#set(LIB_DARWIN ${RoboFEI-HT_SOURCE_DIR}/Control/Linux/lib/darwin.a)
#set(LIB_DYNAMIXEL ${RoboFEI-HT_SOURCE_DIR}/Control/Framework/Framework_Dynamixel/libdxl_x64_cpp.so)

#target_link_libraries(control libcontrol blackboard ${LIB_DARWIN} ${LIB_DYNAMIXEL} -Wno-write-strings -lpthread -lrt ${Boost_LIBRARIES})

#install(TARGETS control DESTINATION ${RoboFEI-HT_BINARY_DIR}/bin)
#install(TARGETS control DESTINATION /usr/local/bin)
