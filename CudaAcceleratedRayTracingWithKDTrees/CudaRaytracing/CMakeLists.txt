cmake_minimum_required (VERSION 3.8)

add_library(CudaRaytracing CudaRaytracing.cu)

target_include_directories(${PROJECT_NAME} PUBLIC
"${PROJECT_BINARY_DIR}"
"${PROJECT_SOURCE_DIR}")