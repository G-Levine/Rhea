cmake_minimum_required(VERSION 3.22.2)
set (CMAKE_CXX_STANDARD 17)

FIND_PACKAGE(Boost)
IF (Boost_FOUND)
    INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
    ADD_DEFINITIONS( "-DHAS_BOOST" )
ENDIF()

project (rhea)
include_directories(include)
file(GLOB SOURCES "src/*.cpp"
                    "include/*.cpp")
file(GLOB HEADERS "include/*.h")
add_executable(rhea ${SOURCES})

