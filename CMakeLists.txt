cmake_minimum_required(VERSION 3.16)
project(jdxl VERSION 0.7.0
        DESCRIPTION "DYNAMIXEL Protocol 1.0 / 2.0 packet handlers"
        LANGUAGES C)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# ---------------------------------------------------------------------------
# Top-level detection. As a subproject we must not change the consumer's build
# type, build our tests, sanitizer-instrument the archive, or add install rules.
# ---------------------------------------------------------------------------
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(JDXL_IS_TOP_LEVEL ON)
else()
    set(JDXL_IS_TOP_LEVEL OFF)
endif()

get_property(_jdxl_multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(JDXL_IS_TOP_LEVEL)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    if(NOT _jdxl_multi_config AND NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    endif()
endif()

if(JDXL_IS_TOP_LEVEL AND CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(_jdxl_tests_default ON)
else()
    set(_jdxl_tests_default OFF)
endif()

option(JDXL_BUILD_TESTS "Build the GoogleTest unit tests"                     ${_jdxl_tests_default})
option(JDXL_SANITIZE    "Build library + tests with ASan/UBSan (tests only)" ON)
option(JDXL_INSTALL     "Generate install/export rules"                       ${JDXL_IS_TOP_LEVEL})

if(JDXL_IS_TOP_LEVEL)
    message(STATUS "jdxl: build type  = ${CMAKE_BUILD_TYPE}")
endif()
message(STATUS "jdxl: build tests = ${JDXL_BUILD_TESTS}")

# ---------------------------------------------------------------------------
# Library
# ---------------------------------------------------------------------------
add_library(jdxl STATIC
    src/jdxl_common.c
    src/jdxl_ph1.c
    src/jdxl_ph2.c)
add_library(jdxl::jdxl ALIAS jdxl)

target_include_directories(jdxl
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_compile_features(jdxl PUBLIC c_std_99)
set_target_properties(jdxl PROPERTIES C_EXTENSIONS OFF)

if(MSVC)
    target_compile_options(jdxl PRIVATE /W4)
else()
    target_compile_options(jdxl PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Sizes. They dimension the contexts, so they are part of the ABI: always
# defined PUBLIC so every consumer compiles with the library's values. Ranges
# are checked by the headers. An empty default makes the size required.
# Precedence: -D on the command line / set() by a parent project / default.
# ---------------------------------------------------------------------------
function(jdxl_size name default help)
    if(NOT DEFINED ${name})
        if("${default}" STREQUAL "")
            message(FATAL_ERROR
                "${name} is required (${help}).\n"
                "  Standalone:  -D${name}=<n>\n"
                "  Subproject:  set(${name} <n>) before add_subdirectory()/FetchContent_MakeAvailable()")
        endif()
        set(${name} ${default} CACHE STRING "${help}")
    endif()
    if(NOT "${${name}}" MATCHES "^[1-9][0-9]*$")
        message(FATAL_ERROR "${name} must be a positive integer, got '${${name}}'.")
    endif()
    message(STATUS "jdxl: ${name} = ${${name}}")
    target_compile_definitions(jdxl PUBLIC ${name}=${${name}})
endfunction()

jdxl_size(JDXL_PH1_PKT_MAX_LEN 259 "Protocol 1.0 packet buffer, bytes (6..259)")
jdxl_size(JDXL_PH1_MAX_STATUS  8   "Protocol 1.0 status replies per instruction (1..254)")
jdxl_size(JDXL_PH2_PKT_MAX_LEN ""  "Protocol 2.0 packet buffer, bytes (14..65542)")
jdxl_size(JDXL_PH2_MAX_STATUS  8   "Protocol 2.0 status replies per instruction (1..253)")

# ---------------------------------------------------------------------------
# Sanitizers — on the library itself so code under test is instrumented.
# PRIVATE so flags never leak into the exported target.
# ---------------------------------------------------------------------------
set(JDXL_SANITIZE_ACTIVE OFF)
if(JDXL_BUILD_TESTS AND JDXL_SANITIZE)
    if(MSVC)
        set(JDXL_SAN_FLAGS      /fsanitize=address)
        set(JDXL_SAN_LINK_FLAGS "")
    elseif(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
        set(JDXL_SAN_FLAGS
            -fsanitize=address,undefined
            -fno-sanitize-recover=undefined
            -fno-omit-frame-pointer)
        set(JDXL_SAN_LINK_FLAGS -fsanitize=address,undefined)
    else()
        message(WARNING "JDXL_SANITIZE: no sanitizer flags known for ${CMAKE_C_COMPILER_ID}")
    endif()
    if(DEFINED JDXL_SAN_FLAGS)
        set(JDXL_SANITIZE_ACTIVE ON)
        target_compile_options(jdxl PRIVATE ${JDXL_SAN_FLAGS})
    endif()
endif()

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if(JDXL_BUILD_TESTS)
    enable_language(CXX)
    enable_testing()
    add_subdirectory(test)
endif()

# ---------------------------------------------------------------------------
# Install + export  =>  find_package(jdxl) -> jdxl::jdxl
# Skipped for sanitizer builds. src/jdxl_internal.h is not installed.
# ---------------------------------------------------------------------------
if(JDXL_INSTALL AND NOT JDXL_SANITIZE_ACTIVE)
    set(_jdxl_cmake_dir ${CMAKE_INSTALL_LIBDIR}/cmake/jdxl)

    install(TARGETS jdxl
        EXPORT  jdxlTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

    install(DIRECTORY include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.h")

    install(EXPORT jdxlTargets
        FILE        jdxlTargets.cmake
        NAMESPACE   jdxl::
        DESTINATION ${_jdxl_cmake_dir})

    configure_package_config_file(
        cmake/jdxlConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/jdxlConfig.cmake
        INSTALL_DESTINATION ${_jdxl_cmake_dir})

    # Pre-1.0: a minor bump may break the API/ABI, so only the same 0.MINOR matches.
    write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/jdxlConfigVersion.cmake
        VERSION       ${PROJECT_VERSION}
        COMPATIBILITY SameMinorVersion)

    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/jdxlConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/jdxlConfigVersion.cmake
        DESTINATION ${_jdxl_cmake_dir})

elseif(JDXL_INSTALL AND JDXL_SANITIZE_ACTIVE)
    message(STATUS "jdxl: install skipped for sanitizer build")
endif()