cmake_minimum_required(VERSION 3.15.0 FATAL_ERROR)

if(DEFINED shapelib_GUARD)
    if(NOT shapelib_GUARD STREQUAL ${CMAKE_CURRENT_BINARY_DIR})
        return()
    endif()
else()
    set(shapelib_GUARD ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "shapelib guard")
endif()

project(shapelib C)


set(CMAKE_AUTOMOC OFF)
set(CMAKE_AUTORCC OFF)

set(SHP_C_HEADERS
	./shapelib/shapefil.h
)

set(SHP_C_SOURCES
	./shapelib/shpopen.c
	./shapelib/dbfopen.c
	./shapelib/safileio.c
	./shapelib/shptree.c
	#./shapelib/sbnsearch.c
)

add_library(${PROJECT_NAME} STATIC ${SHP_C_HEADERS} ${SHP_C_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC ./shapelib)

#target_compile_definitions(${PROJECT_NAME} PUBLIC SHAPELIB_DLLEXPORT)
