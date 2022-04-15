#========================================================================================

include_directories( ${CMAKE_CURRENT_LIST_DIR} )

#========================================================================================

include( ${CMAKE_CURRENT_LIST_DIR}/eigen_libs.cmake )
include( ${CMAKE_CURRENT_LIST_DIR}/opencv_libs.cmake )
include( ${CMAKE_CURRENT_LIST_DIR}/open3d_libs.cmake )
# include( ${CMAKE_CURRENT_LIST_DIR}/json_libs.cmake )
# include( ${CMAKE_CURRENT_LIST_DIR}/pcl_libs.cmake )

#========================================================================================

set( LIBS
    ${EIGEN_LIBS}
    ${OpenCV_LIBS}
    ${Open3D_LIBS}
#     ${JSON_LIBS}
#     ${PCL_LIB}
#    -lrdkafka++
#    -lglib-2.0
#    -lgobject-2.0
#    -lgio-2.0
#     -lzcm
#    -lcurl
#    -lcpr
    )

#========================================================================================
