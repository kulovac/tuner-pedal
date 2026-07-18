enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(firmware_tests
    test_dsp.cpp

    # Target source files under test:
    ${CMSIS_DSP_SOURCES}
)

target_include_directories(firmware_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/inc
    ${cmsis_core_SOURCE_DIR}/Include
    ${cmsis_dsp_SOURCE_DIR}/Include
)

target_compile_definitions(firmware_tests PRIVATE
    LOG_LEVEL=LOG_LEVEL_NONE
)

target_link_libraries(firmware_tests PRIVATE
    GTest::gtest_main
    m
)

include(GoogleTest)
gtest_discover_tests(firmware_tests)
