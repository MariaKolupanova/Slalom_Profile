cmake_minimum_required(VERSION 3.15.0 FATAL_ERROR)

set(ROOT_PATH ${SOURCE_DIR}../..)

project(shape_tiff_model)
project_protect()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(HEADERS
  ./include/shape_tiff_model.h
)

set(SOURCES
  ./src/shape_tiff_model.cpp
)

source_group ("Header Files" FILES ${HEADERS})
source_group ("Source Files" FILES ${SOURCES})

add_library( ${PROJECT_NAME} STATIC ${SOURCES} ${HEADERS} )
target_include_directories(${PROJECT_NAME} PUBLIC ./include)

# require C++ 20
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

