cmake_minimum_required(VERSION 3.15.0 FATAL_ERROR)

set(ROOT_PATH ${SOURCE_DIR}../..)

project(tiff_reader)
project_protect()

find_package(GeoTIFF REQUIRED)
find_package(PROJ REQUIRED)
find_package(SQLite3 REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(HEADERS
  ./include/tiff_reader.h
)

set(SOURCES
  ./src/tiff_reader.cpp
)

source_group ("Header Files" FILES ${HEADERS})
source_group ("Source Files" FILES ${SOURCES})

add_library( ${PROJECT_NAME} STATIC ${SOURCES} ${HEADERS} )
target_include_directories(${PROJECT_NAME} PUBLIC ./include)

# require C++ 20
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

add_subproject(shape_tiff_model)

target_include_directories(${PROJECT_NAME}
    PUBLIC
	${GeoTIFF_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
	shape_tiff_model
	${GeoTIFF_LIBRARIES}
)
