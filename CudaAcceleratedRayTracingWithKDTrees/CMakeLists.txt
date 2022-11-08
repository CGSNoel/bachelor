# CMakeList.txt: CMake-Projekt für "CudaAcceleratedRayTracingWithKDTrees". Schließen Sie die Quelle ein, und definieren Sie
# projektspezifische Logik hier.
#
cmake_minimum_required (VERSION 3.8)

# Fügen Sie der ausführbaren Datei dieses Projekts eine Quelle hinzu.
add_executable (CudaAcceleratedRayTracingWithKDTrees "CudaAcceleratedRayTracingWithKDTrees.cpp" "CudaAcceleratedRayTracingWithKDTrees.h" "hit.cpp" "hit.h" "image.cpp" "image.h" "kdnode.cpp" "kdnode.h" "light.cpp" "light.h" "lodepng.cpp" "lodepng.h" "object.cpp" "object.h" "objparser.cpp" "objparser.h" "ray.cpp" "ray.h" "triangle.cpp" "triangle.h" "vector.cpp" "vector.h")

add_subdirectory ("CudaRaytracing")

# TODO: Fügen Sie bei Bedarf Tests hinzu, und installieren Sie Ziele.
