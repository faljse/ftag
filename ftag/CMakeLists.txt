INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src ${GNULIBS_INCLUDE_DIR})
LINK_LIBRARIES(${PROJECT_NAME} ${ARUCO_REQUIRED_LIBRARIES} )

add_executable(ftag ftag.cpp)
target_link_libraries(ftag         aruco opencv_highgui opencv_imgproc opencv_calib3d)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(ftag Threads::Threads)

if(WIN32)
  target_link_libraries(ftag wsock32 ws2_32)
endif()

if (OpenCV_VERSION VERSION_GREATER "3.0.0")
#    target_link_libraries(image2contourfile     opencv_imgcodecs)
#    target_link_libraries(extract_chilitag_dict opencv_imgcodecs)
endif()

INSTALL(TARGETS ftag RUNTIME DESTINATION bin)
