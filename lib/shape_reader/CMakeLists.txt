cmake_minimum_required(VERSION 3.15.0 FATAL_ERROR)

set(ROOT_PATH ${SOURCE_DIR}../..)

project(shape_reader)
project_protect()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(HEADERS
  ./include/shape_reader.h

)

set(SOURCES
  ./src/shape_reader.cpp

)

source_group ("Header Files" FILES ${HEADERS})
source_group ("Source Files" FILES ${SOURCES})

add_subproject(shapefile_utils)
add_subproject(shape_tiff_model)

add_library(${PROJECT_NAME} STATIC ${HEADERS} ${SOURCES})

target_link_libraries(${PROJECT_NAME} 
	PRIVATE
	shapefile_utils
	shape_tiff_model
)

target_include_directories(${PROJECT_NAME} PUBLIC ./include)

if(MSVC)
	target_compile_options(${PROJECT_NAME} PRIVATE /bigobj)
endif(MSVC)

# require C++ 20
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
