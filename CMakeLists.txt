cmake_minimum_required (VERSION 3.2)
project(Space_Linux)

#设定CXX的标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

#添加源文件到src_init
file(GLOB src_init ${PROJECT_SOURCE_DIR}/init/*.cpp) #GLOB是通配符路径的意思

#添加需要include的h文件路径
#include_directories()
 
add_executable(Space_Linux ${src_init})