cmake_minimum_required(VERSION 3.10)

set(projectName ofxProfiler)

project(${projectName})

file(GLOB SOURCE
        "src/*.h"
        "src/*.cpp"
		"src/ofxProfiler/*.h"
		"src/ofxProfiler/*.cpp"
        )

set(LIBOFXPROFILER_SRC ${SOURCE})

add_library(${projectName} STATIC ${LIBOFXPROFILER_SRC})
include_directories(/src)