cmake_minimum_required(VERSION 3.16)

project(open_vins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(NOFORTRAN "1")
set(DYNAMIC_ARCH OFF)
set(BUILD_WITHOUT_LAPACK ON)
set(BUILD_TESTING OFF)
set(BUILD_BENCHMARKS OFF)
set(TARGET "RISCV64_GENERIC")
add_definitions(-DGEMMINI_BACKEND)
add_definitions(-DEIGEN_USE_BLAS=1)

add_subdirectory(${CMAKE_SOURCE_DIR}/external/OpenBLAS)
add_subdirectory(${CMAKE_SOURCE_DIR}/external/eigen)

list(APPEND custom_eigen_blas
    openblas_static
    Eigen3::Eigen
)

add_subdirectory(ov_core)
# add_subdirectory(ov_eval)
add_subdirectory(ov_msckf)


