## Use the variable PROJECT_NAME for changing the target name
get_filename_component(ProjectId ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
set( PROJECT_NAME ${ProjectId} )

cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
project(${PROJECT_NAME})

# include_directories(include)

## Define libraries
# target_include_directories(${PROJECT_NAME}_lib include)
# add_library(${PROJECT_NAME}_lib
# 
#)

## Define the executable
set(SRCS 
  src/main.cpp
)
add_executable(${PROJECT_NAME} ${SRCS})
target_include_directories(${PROJECT_NAME} PUBLIC
 	include
)
# target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)