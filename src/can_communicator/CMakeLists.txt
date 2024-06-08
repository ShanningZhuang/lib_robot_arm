include_directories(${PROJECT_SOURCE_DIR}/include/motor_controller)
include_directories(${PROJECT_SOURCE_DIR}/include/pcan)
include_directories(${PROJECT_SOURCE_DIR}/include/can_communicator)

add_library(can_communicator STATIC ${CMAKE_CURRENT_SOURCE_DIR}/can_communicator.cpp)

# check arch and os
message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
endif()
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64.*")
  set(ARCH arm64)
endif()

target_link_libraries(can_communicator ${PROJECT_SOURCE_DIR}/lib/pcan/${ARCH}/libpcan.so) 
