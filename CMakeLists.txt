cmake_minimum_required(VERSION 3.10)

set(projectName SoftRender)

project(${projectName})

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
endif (NOT CMAKE_BUILD_TYPE)

list(APPEND CMAKE_CXX_FLAGS "-std=c++17")

include(FindPkgConfig)
PKG_SEARCH_MODULE(SDL2 REQUIRED sdl2)
message(STATUS "Found SDL2 in ${SDL2_INCLUDE_DIRS}")
PKG_SEARCH_MODULE(SDL2IMAGE REQUIRED SDL2_image>=2.0.0)
message(STATUS "Found SDL2IMAGE in ${SDL2IMAGE_INCLUDE_DIRS}")

if (APPLE)
    include_directories(${SDL2_INCLUDE_DIR} ${SDL2IMAGE_INCLUDE_DIRS})
    SET(APPLE_LIBS ${SDL2_LIBRARIES} ${SDL2IMAGE_LIBRARIES})
    set(LIBS ${LIBS} ${APPLE_LIBS})
endif (APPLE)

file(GLOB SOURCE
        "src/*.h"
        "src/*.cpp"
        )

        add_executable(${projectName} ${SOURCE} )
        target_link_libraries(${projectName} ${LIBS})
        if (WIN32)
            set_target_properties(${projectName} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin/${CHAPTER}")
        elseif (UNIX AND NOT APPLE)
            set_target_properties(${projectName} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin/${CHAPTER}")
        elseif (APPLE)
            set_target_properties(${projectName} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin/${CHAPTER}")
            set_target_properties(${projectName} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_BINARY_DIR}/bin/${CHAPTER}")
            set_target_properties(${projectName} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_BINARY_DIR}/bin/${CHAPTER}")
        endif (WIN32)

include_directories(${CMAKE_SOURCE_DIR}/includes)