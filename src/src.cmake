#========================================================================================

include_directories( ${CMAKE_CURRENT_LIST_DIR} )

#========================================================================================

# include( ${CMAKE_CURRENT_LIST_DIR}/config/config.cmake )
# include( ${CMAKE_CURRENT_LIST_DIR}/core/core.cmake )
# include( ${CMAKE_CURRENT_LIST_DIR}/subscriber/subscriber.cmake )
# include( ${CMAKE_CURRENT_LIST_DIR}/publisher/publisher.cmake )
# include( ${CMAKE_CURRENT_LIST_DIR}/defs/defs.cmake )
include( ${CMAKE_CURRENT_LIST_DIR}/utils/utils.cmake )

#========================================================================================

set( SRC
#     ${CONFIG}
#     ${CORE}
#     ${SUBSCRIBER}
#     ${PUBLISHER}
#     ${DEFS}
    ${UTILS}
    )

#========================================================================================
