cmake_minimum_required(VERSION 3.15.0 FATAL_ERROR)

project(Tree)

set(ROOT_PATH ${SOURCE_DIR}../)

macro(project_protect)
    if(DEFINED ${PROJECT_NAME}_PROTECT)
        if(NOT ${PROJECT_NAME}_PROTECT STREQUAL ${CMAKE_CURRENT_BINARY_DIR})
          return()
        endif()
    else()
        set(${PROJECT_NAME}_PROTECT ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "${PROJECT_NAME} protect")
    endif()
endmacro()

macro(add_subproject name)
  set(REPOSITORY_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
  set(REPOSITORY_ROOT_DIR_EXISTS FALSE)
  while(EXISTS ${REPOSITORY_ROOT_DIR})
    if(EXISTS "${REPOSITORY_ROOT_DIR}/cmake_project")
      set(REPOSITORY_ROOT_DIR_EXISTS TRUE)
      break()
    else()
      set(REPOSITORY_ROOT_DIR ${REPOSITORY_ROOT_DIR}/..)
    endif()
  endwhile()

  if (NOT ${REPOSITORY_ROOT_DIR_EXISTS})
    message(FATAL_ERROR cmake_project " DOES NOT EXIST FOR " ${CMAKE_CURRENT_SOURCE_DIR})
	return()
  else()
  endif()

  # Наличие extern
  string(REPLACE "/" ";" SUBPROJECT_DIRECTORY_LIST ${name})
  list(GET SUBPROJECT_DIRECTORY_LIST 0 SUBPROJECT_NAME)
  if(${SUBPROJECT_NAME} STREQUAL "extern")
    list(GET SUBPROJECT_DIRECTORY_LIST 1 EXTERN_REPOSITORY_NAME)
    string(REPLACE "extern/${EXTERN_REPOSITORY_NAME}/" "" SUBPROJECT_INFO_FILE_PATH ${name})

    if(EXISTS "${REPOSITORY_ROOT_DIR}/extern")
      if(EXISTS "${REPOSITORY_ROOT_DIR}/extern/${EXTERN_REPOSITORY_NAME}/cmake_project/${SUBPROJECT_INFO_FILE_PATH}.cmake")
	    # Read subproject path
        file(STRINGS "${REPOSITORY_ROOT_DIR}/extern/${EXTERN_REPOSITORY_NAME}/cmake_project/${SUBPROJECT_INFO_FILE_PATH}.cmake" SUBPROJECT_DIRECTORY)

	    # Get subproject name
        string(REPLACE "/" ";" SUBPROJECT_DIRECTORY_LIST ${name})
        list(REVERSE SUBPROJECT_DIRECTORY_LIST)
        list(GET SUBPROJECT_DIRECTORY_LIST 0 SUBPROJECT_NAME)

        # Add subproject
        add_subdirectory(${REPOSITORY_ROOT_DIR}/extern/${EXTERN_REPOSITORY_NAME}/${SUBPROJECT_DIRECTORY} ${CMAKE_BINARY_DIR}/subproject/${SUBPROJECT_NAME}/${PROJECT_NAME})
      else()
        message(FATAL_ERROR ${REPOSITORY_ROOT_DIR}/extern/${EXTERN_REPOSITORY_NAME}/cmake_project/${SUBPROJECT_INFO_FILE_PATH}.cmake " DOES NOT EXIST")
      endif()
    else()
	  if(EXISTS "${REPOSITORY_ROOT_DIR}/../${EXTERN_REPOSITORY_NAME}/cmake_project/${SUBPROJECT_INFO_FILE_PATH}.cmake")
	    # Read subproject path
        file(STRINGS "${REPOSITORY_ROOT_DIR}/../${EXTERN_REPOSITORY_NAME}/cmake_project/${SUBPROJECT_INFO_FILE_PATH}.cmake" SUBPROJECT_DIRECTORY)

	    # Get subproject name
        string(REPLACE "/" ";" SUBPROJECT_DIRECTORY_LIST ${name})
        list(REVERSE SUBPROJECT_DIRECTORY_LIST)
        list(GET SUBPROJECT_DIRECTORY_LIST 0 SUBPROJECT_NAME)

        # Add subproject
        add_subdirectory(${REPOSITORY_ROOT_DIR}/../${EXTERN_REPOSITORY_NAME}/${SUBPROJECT_DIRECTORY} ${CMAKE_BINARY_DIR}/subproject/${SUBPROJECT_NAME}/${PROJECT_NAME})
	  else()
        message(FATAL_ERROR ${REPOSITORY_ROOT_DIR}/../${EXTERN_REPOSITORY_NAME}/cmake_project/${SUBPROJECT_INFO_FILE_PATH}.cmake " DOES NOT EXIST")
      endif()
    endif()
  else()
    if(EXISTS "${REPOSITORY_ROOT_DIR}/cmake_project/${name}.cmake")
      # Read subproject path
      file(STRINGS "${REPOSITORY_ROOT_DIR}/cmake_project/${name}.cmake" SUBPROJECT_DIRECTORY)

      # Get subproject name
      string(REPLACE "/" ";" SUBPROJECT_DIRECTORY_LIST ${name})
      list(REVERSE SUBPROJECT_DIRECTORY_LIST)
      list(GET SUBPROJECT_DIRECTORY_LIST 0 SUBPROJECT_NAME)

      # Add subproject
      add_subdirectory(${REPOSITORY_ROOT_DIR}/${SUBPROJECT_DIRECTORY} ${CMAKE_BINARY_DIR}/subproject/${SUBPROJECT_NAME}/${PROJECT_NAME})
    else()
      message(FATAL_ERROR ${REPOSITORY_ROOT_DIR}/cmake_project/${name}.cmake " DOES NOT EXIST")
    endif()
  endif()
endmacro()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# add_subproject(tiff_reader)
add_subproject(shape_reader)
add_subproject(graph)

set(HEADERS

)

# Populate a CMake variable with the sources
set(SOURCES
  src/main.cpp
)

source_group ("Header Files" FILES ${HEADERS})
source_group ("Source Files" FILES ${SOURCES})

add_executable(${PROJECT_NAME} ${SOURCES}  ${MOC_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE ./include)

target_link_libraries(${PROJECT_NAME} 
	PRIVATE 
	shape_reader
	graph
	# tiff_reader
)

# require C++ 20
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)