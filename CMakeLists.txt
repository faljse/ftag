# ----------------------------------------------------------------------------
#   Basic Configuration
# ----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.0)
set(CMAKE_INSTALL_PREFIX /opt/ftag)
project(aruco VERSION "3.0.4" LANGUAGES CXX)
set(PROJECT_SOVERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11

 
# find_package(OpenCV REQUIRED)
# target_include_directories(main PRIVATE ${OpenCV_INCLUDE_DIRS})
# target_link_libraries(main PRIVATE ${OpenCV_LIBS})

include(cmake/options.cmake)

include(cmake/findDependencies.cmake)
include(cmake/compilerOptions.cmake)


add_definitions(-DUSE_SVM_LABELER)
add_subdirectory(src)
add_subdirectory(ftag)

if(GL_FOUND AND BUILD_GLSAMPLES)
  add_subdirectory(utils_gl)
endif()
if(BUILD_TESTS)
  add_subdirectory(tests)
endif()


if(BUILD_UTILS)
  add_subdirectory(utils)
  add_subdirectory(utils_markermap)
  add_subdirectory(utils_calibration)
  ADD_SUBDIRECTORY(utils_svm)
endif()
include(cmake/printInfo.cmake)
include(cmake/installOptions.cmake)

