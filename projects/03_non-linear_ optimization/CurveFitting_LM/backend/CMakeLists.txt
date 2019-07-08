list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake) 

# find_package(GLOG REQUIRED)
# include_directories(${GLOG_INCLUDE_DIRS})

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIR})


add_library(${PROJECT_NAME}_backend
        vertex.cc
        edge.cc
        problem.cc
        )
target_link_libraries(
        ${PROJECT_NAME}_backend
        # ${GLOG_LIBRARIES}
        ${Eigen_LIBRARIES}
)        