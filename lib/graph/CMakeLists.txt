cmake_minimum_required(VERSION 3.15.0 FATAL_ERROR)

set(ROOT_PATH ${SOURCE_DIR}../..)

project(graph)
project_protect()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

#seismic_solver
set(SOURCES
   	src/Graph.cpp
)
set(HEADERS
	include/Graph.h
)
add_subproject(shape_tiff_model)


add_library(${PROJECT_NAME} STATIC ${HEADERS} ${SOURCES})

target_link_libraries(${PROJECT_NAME} 
	PUBLIC
	shape_tiff_model
)


target_include_directories(${PROJECT_NAME} PUBLIC ./include)

if(MSVC)
	target_compile_options(${PROJECT_NAME} PRIVATE /bigobj)
endif(MSVC)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)


