cmake_minimum_required(VERSION 3.20)
project(ConstellationSim VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(MSVC)
  add_compile_options(/W4 /O2)
else()
  add_compile_options(-Wall -Wextra -O3 -march=native)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    add_compile_options(-fopenmp-simd)
  endif()
endif()

# --------------------------------------------------------------------------
# Dependencies
# --------------------------------------------------------------------------
include(FetchContent)

FetchContent_Declare(
  json
  URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
  URL_HASH SHA256=d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d
)
FetchContent_MakeAvailable(json)

find_package(Threads REQUIRED)

# --------------------------------------------------------------------------
# Source groups
# --------------------------------------------------------------------------
set(CORE_SOURCES
  src/core/ConfigLoader.cpp
  src/core/SimulationEngine.cpp
)

set(ORBIT_SOURCES
  src/orbit/OrbitalElements.cpp
  src/orbit/Satellite.cpp
  src/orbit/Propagator.cpp
)

set(ENVIRONMENT_SOURCES
  src/environment/SunModel.cpp
  src/environment/MoonModel.cpp
  src/environment/AtmosphereModel.cpp
  src/environment/EclipseModel.cpp
)

set(PHYSICS_SOURCES
  src/physics/Gravity.cpp
  src/physics/J2Perturbation.cpp
  src/physics/AtmosphericDrag.cpp
  src/physics/SolarRadiationPressure.cpp
)

set(CONSTELLATION_SOURCES
  src/constellation/Plane.cpp
  src/constellation/Constellation.cpp
  src/constellation/Deployment.cpp
)

set(METRICS_SOURCES
  src/metrics/MetricsCollector.cpp
)

set(MONTECARLO_SOURCES
  src/montecarlo/ParameterSweep.cpp
  src/montecarlo/SimulationBatch.cpp
  src/montecarlo/ExperimentManager.cpp
)

set(OUTPUT_SOURCES
  src/output/CsvWriter.cpp
  src/output/OutputManager.cpp
)

# --------------------------------------------------------------------------
# Core simulation library (headless, no GL dependency)
# --------------------------------------------------------------------------
add_library(constellation_core STATIC
  ${CORE_SOURCES}
  ${ORBIT_SOURCES}
  ${ENVIRONMENT_SOURCES}
  ${PHYSICS_SOURCES}
  ${CONSTELLATION_SOURCES}
  ${METRICS_SOURCES}
  ${MONTECARLO_SOURCES}
  ${OUTPUT_SOURCES}
)

target_include_directories(constellation_core PUBLIC src)
target_link_libraries(constellation_core PUBLIC
  nlohmann_json::nlohmann_json
  Threads::Threads
)

# --------------------------------------------------------------------------
# Main executable
# --------------------------------------------------------------------------
add_executable(ConstellationSim src/main.cpp)
target_link_libraries(ConstellationSim PRIVATE constellation_core)

# --------------------------------------------------------------------------
# Optional visualization (requires VGL)
# --------------------------------------------------------------------------
option(BUILD_VIZ "Build with VGL visualization support" OFF)

if(BUILD_VIZ)
  set(VGL_LOCAL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../VGL")
  if(EXISTS "${VGL_LOCAL_PATH}/CMakeLists.txt")
    add_subdirectory(${VGL_LOCAL_PATH} vgl_build)
  else()
    FetchContent_Declare(
      vgl
      GIT_REPOSITORY https://github.com/vcampos4545/VGL.git
      GIT_TAG main
    )
    FetchContent_MakeAvailable(vgl)
  endif()

  target_sources(constellation_core PRIVATE
    src/viz/Renderer.cpp
    src/viz/SatelliteRenderer.cpp
  )
  target_link_libraries(constellation_core PUBLIC vgl)
  target_compile_definitions(constellation_core PUBLIC CONSTELLATION_VIZ_ENABLED)
endif()

# --------------------------------------------------------------------------
# Copy config directories to build tree for convenience
# --------------------------------------------------------------------------
add_custom_command(TARGET ConstellationSim POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_SOURCE_DIR}/config"
    "$<TARGET_FILE_DIR:ConstellationSim>/config"
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${CMAKE_SOURCE_DIR}/montecarlo"
    "$<TARGET_FILE_DIR:ConstellationSim>/montecarlo"
  COMMENT "Copying config files to build directory"
)
