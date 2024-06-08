# file(GLOB_RECURSE test_list ${CMAKE_CURRENT_LIST_DIR}/*.cpp)

include_directories(${PROJECT_SOURCE_DIR}/include/lkmotor_controller)

add_executable(test_get_multi_encoder test_get_multi_encoder.cpp)
target_link_libraries(test_get_multi_encoder lkmotor_controller)
target_link_libraries(test_get_multi_encoder can_communicator)
target_link_libraries(test_get_multi_encoder motor_config)
target_link_libraries(test_get_multi_encoder utils)


# add_executable(test_set_zero test_set_zero.cpp)
# target_link_libraries(test_set_zero lkmotor_controller)
# target_link_libraries(test_set_zero can_communicator)
# target_link_libraries(test_set_zero motor_config)
# target_link_libraries(test_set_zero utils)


add_executable(test_servo test_servo.cpp)
target_link_libraries(test_servo lkmotor_controller)
target_link_libraries(test_servo can_communicator)
target_link_libraries(test_servo motor_config)
target_link_libraries(test_servo utils)

find_package(yaml-cpp REQUIRED)

add_executable(test_recorder test_recorder.cpp)
target_link_libraries(test_recorder lkmotor_controller)
target_link_libraries(test_recorder can_communicator)
target_link_libraries(test_recorder motor_config)
target_link_libraries(test_recorder utils)
target_link_libraries(test_recorder yaml-cpp)

add_executable(test_replay test_replay.cpp)
target_link_libraries(test_replay lkmotor_controller)
target_link_libraries(test_replay can_communicator)
target_link_libraries(test_replay motor_config)
target_link_libraries(test_replay utils)
target_link_libraries(test_replay yaml-cpp)

# add_executable(test_process_feedback test_process_feedback.cpp)
# target_link_libraries(test_process_feedback lkmotor_controller)
# target_link_libraries(test_process_feedback can_communicator)
# target_link_libraries(test_process_feedback motor_config)
# target_link_libraries(test_process_feedback utils)

# add_executable(test_atomic_signal test_atomic_signal.cpp)
# target_link_libraries(test_atomic_signal lkmotor_controller)
# target_link_libraries(test_atomic_signal can_communicator)
# target_link_libraries(test_atomic_signal motor_config)
# target_link_libraries(test_atomic_signal utils) 