cmake_minimum_required(VERSION 3.0.0)
project(NOTgraphVIZ VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_PREFIX_PATH "/usr/bin/qmake")

find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)

file(GLOB SOURCES
	GraphicClasses/*.cpp
	AbstractClasses/*.cpp
	*.cpp
	


)

include_directories(.)

add_executable(NOTgraphVIZ
        ${SOURCES}
        )

target_link_libraries(NOTgraphVIZ Qt5::Core Qt5::Widgets Qt5::Gui)
