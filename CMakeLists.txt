cmake_minimum_required(VERSION 3.17)
project(registration)

set(CMAKE_CXX_STANDARD 11)

# EDIT PATHS ACCORDINGLY
############################
set(EIGEN_INCLUDE_DIR /usr/local/include)
set(FLANN_INCLUDE_DIR /usr/local/include)
set(OMP_INCLUDE_DIR /usr/local/include)
set(JSONCPP_INCLUDE_DIR /usr/local/opt/jsoncpp/include)
set(FLANN_LIB_DIR /usr/local/lib)
set(OMP_LIB_DIR /usr/local/lib)
set(JSONCPP_LIB_DIR /usr/local/opt/jsoncpp/lib)
set(FLANN_LFLAG -lflann)
set(OMP_LFLAG -lomp)
set(JSONCPP_LFLAG -ljsoncpp)
############################

file(GLOB REGISTRATION_SOURCES "source/*.cpp")
add_library(registration SHARED ${REGISTRATION_SOURCES})
set_target_properties(registration PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(registration PRIVATE
        include/registration)
target_include_directories(registration PRIVATE
        ${EIGEN_INCLUDE_DIR}
        ${FLANN_INCLUDE_DIR}
        ${OMP_INCLUDE_DIR})
target_link_directories(registration PRIVATE
        ${FLANN_LIB_DIR}
        ${OMP_LIB_DIR})
target_link_libraries(registration PRIVATE
        ${FLANN_LFLAG}
        ${OMP_LFLAG})

add_executable(stitch stitch.cpp)
target_include_directories(stitch PRIVATE ./include)
target_link_libraries(stitch PRIVATE registration)
target_include_directories(stitch PRIVATE ${EIGEN_INCLUDE_DIR} ${JSONCPP_INCLUDE_DIR})
target_link_directories(stitch PRIVATE ${JSONCPP_LIB_DIR})
target_link_libraries(stitch PRIVATE ${JSONCPP_LFLAG})

install(TARGETS registration stitch DESTINATION ${PROJECT_SOURCE_DIR}/bin)
