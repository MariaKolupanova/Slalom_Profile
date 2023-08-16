cmake_minimum_required(VERSION 3.15.0 FATAL_ERROR)

project(shapefile_utils)
project_protect()

set(EXTERN_PATH ../../..)

set(HEADERS
	./shapefilecpp/lib/include/Bounds.hpp
	./shapefilecpp/lib/include/Feature.hpp
	./shapefilecpp/lib/include/FeatureIterator.hpp
	./shapefilecpp/lib/include/Field.hpp
	./shapefilecpp/lib/include/FieldType.hpp
	./shapefilecpp/lib/include/Geometry.hpp
	./shapefilecpp/lib/include/GeometryType.hpp
	./shapefilecpp/lib/include/Line.hpp
	./shapefilecpp/lib/include/MultiLine.hpp
	./shapefilecpp/lib/include/MultiPoint.hpp
	./shapefilecpp/lib/include/MultiPolygon.hpp
	./shapefilecpp/lib/include/Point.hpp
	./shapefilecpp/lib/include/Polygon.hpp
	./shapefilecpp/lib/include/Ring.hpp
	./shapefilecpp/lib/include/Schema.hpp
    ./shapefilecpp/lib/include/ShapefileReader.hpp
)

set(SOURCES
	./shapefilecpp/lib/src/Bounds.cpp
	./shapefilecpp/lib/src/Feature.cpp
	./shapefilecpp/lib/src/FeatureIterator.cpp
	./shapefilecpp/lib/src/Field.cpp
	./shapefilecpp/lib/src/FieldType.cpp
	./shapefilecpp/lib/src/Geometry.cpp
	./shapefilecpp/lib/src/GeometryType.cpp
	./shapefilecpp/lib/src/Line.cpp
	./shapefilecpp/lib/src/MultiLine.cpp
	./shapefilecpp/lib/src/MultiPoint.cpp
	./shapefilecpp/lib/src/MultiPolygon.cpp
	./shapefilecpp/lib/src/Point.cpp
	./shapefilecpp/lib/src/Polygon.cpp
	./shapefilecpp/lib/src/Ring.cpp
	./shapefilecpp/lib/src/Schema.cpp
	./shapefilecpp/lib/src/ShapefileReader.cpp
)

add_library(${PROJECT_NAME} STATIC ${HEADERS} ${SOURCES})

add_subproject(shapelib)

target_include_directories(${PROJECT_NAME} PUBLIC ./shapefilecpp/lib/include)

target_link_libraries(${PROJECT_NAME}
	PUBLIC
	shapelib
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
