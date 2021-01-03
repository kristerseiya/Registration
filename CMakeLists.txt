cmake_minimum_required(VERSION 3.17)
project(registration)

if (NOT DEFINED CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 11)
endif()

# EDIT PATHS ACCORDINGLY
############################
if (NOT DEFINED EIGEN_INCLUDE_DIR)
set(EIGEN_INCLUDE_DIR /usr/local/include)
endif()
if (NOT DEFINED FLANN_INCLUDE_DIR)
set(FLANN_INCLUDE_DIR /usr/local/include)
endif()
if (NOT DEFINED OMP_INCLUDE_DIR)
set(OMP_INCLUDE_DIR /usr/local/include)
endif()
if (NOT DEFINED FLANN_LIB_DIR)
set(FLANN_LIB_DIR /usr/local/lib)
endif()
if (NOT DEFINED OMP_LIB_DIR)
set(OMP_LIB_DIR /usr/local/lib)
endif()
if (NOT DEFINED FLANN_LFLAG)
set(FLANN_LFLAG -lflann)
endif()
if (NOT DEFINED OMP_LFLAG)
set(OMP_LFLAG -lomp)
endif()
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
