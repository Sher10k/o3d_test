#========================================================================================

project( o3d_test LANGUAGES CXX )

#========================================================================================

cmake_minimum_required( VERSION 3.14 )

#========================================================================================

option( DEBUG FALSE )

if ( DEBUG )
    set( CMAKE_BUILD_TYPE "Debug" )
    add_definitions ( -Wall )
else()
    set( CMAKE_BUILD_TYPE "Release" )
endif()

#========================================================================================

set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

#========================================================================================

# include( ${CMAKE_CURRENT_SOURCE_DIR}/external/external.cmake )

#========================================================================================

include_directories( ${CMAKE_SOURCE_DIR} )

# include( ${CMAKE_CURRENT_SOURCE_DIR}/cfg/cfg.cmake )
include( ${CMAKE_CURRENT_SOURCE_DIR}/src/src.cmake )
include( ${CMAKE_CURRENT_SOURCE_DIR}/libs/libs.cmake )

#========================================================================================

add_executable( ${PROJECT_NAME}
    src/main.cpp
#     ${CFG}
    ${SRC}
#     ${EXTERNAL}
    )

target_link_libraries( ${PROJECT_NAME}
#     ${V_LIBRARIES}
    ${LIBS}
    )

#========================================================================================
