cmake_minimum_required (VERSION 3.2)
project(Space_AI)

#设定CXX的标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

#设定编译项目名（放在usr目录下的项目目录）
set(fork YOLO) #编译YOLO分支
#set(fork PID_OrbitCtl) #编译PID轨道导航分支

find_package(OpenCV REQUIRED)

if(OPENMP_FOUND)
	message("OPENMP FOUND!")
endif()

set(ncnn_DIR "/home/kdp-pi/ncnn/build/install/lib/cmake/ncnn")
find_package(ncnn REQUIRED)


#添加源文件到src_init GLOB是通配符模板名称
file(GLOB src_init ${CMAKE_CURRENT_SOURCE_DIR}/proc/*.cpp
					${CMAKE_CURRENT_SOURCE_DIR}/lib/*.cpp
					${CMAKE_CURRENT_SOURCE_DIR}/usr/${fork}/*.cpp
					${CMAKE_CURRENT_SOURCE_DIR}/usr/${fork}/lib/*.cpp)

#添加需要include的h文件路径
set(hFile ${CMAKE_CURRENT_SOURCE_DIR}/include/
				  ${CMAKE_CURRENT_SOURCE_DIR}/usr/${fork}/include/
				  /usr/include/python3.11/
				  /usr/include/eigen3/
					)

include_directories(${hFile})

#添加library的文件路径
set(lib_path /usr/lib/python3.11/config-3.11-aarch64-linux-gnu/
			/usr/lib/python3/dist-packages/numpy/core/lib)

link_directories(${lib_path})

#编译文件
add_executable(Space_AI ${src_init})
target_link_libraries(Space_AI python3.11 #python调用
								${OpenCV_LIBS} #opencv库
								ncnn           #ncnn
								)

#打印变量
#message(STATUS "${hFile}")
