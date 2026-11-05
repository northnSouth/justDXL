find_package(GTest QUIET)
if(NOT GTest_FOUND)
    include(FetchContent)
    if(POLICY CMP0135)
        cmake_policy(SET CMP0135 NEW)
    endif()
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz)
    FetchContent_MakeAvailable(googletest)
endif()
include(GoogleTest)

add_executable(jdxl_tests test_ph1.cpp test_ph2.cpp)
target_compile_features(jdxl_tests PRIVATE cxx_std_14)
set_target_properties(jdxl_tests PROPERTIES CXX_EXTENSIONS OFF)
target_link_libraries(jdxl_tests PRIVATE jdxl GTest::gtest_main)

if(MSVC)
    target_compile_options(jdxl_tests PRIVATE /W4)
else()
    target_compile_options(jdxl_tests PRIVATE -Wall -Wextra)
endif()

if(JDXL_SANITIZE_ACTIVE)
    target_compile_options(jdxl_tests PRIVATE ${JDXL_SAN_FLAGS})
    target_link_options(jdxl_tests    PRIVATE ${JDXL_SAN_LINK_FLAGS})
endif()

gtest_discover_tests(jdxl_tests
    PROPERTIES TIMEOUT 10
               ENVIRONMENT "UBSAN_OPTIONS=print_stacktrace=1")