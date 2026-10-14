cmake_minimum_required(VERSION 3.25)

# The library's identity. The family's CMake modules and tools read these.
set(MAUL_LIBRARY maul-anim)
set(MAUL_API_PREFIX mani)
set(MAUL_MACRO_PREFIX MANI)
set(MAUL_TITLE "Maul Anim")
set(MAUL_PREFIX MAUL_ANIM)

include(cmake/ReadVersion.cmake)
maul_read_version(${CMAKE_CURRENT_SOURCE_DIR}/include/maul-anim/base.h MANI MAUL_VERSION)
project(maul-anim
    VERSION ${MAUL_VERSION}
    DESCRIPTION "Skeletal animation: sampling, blending, graphs, IK and compression"
    LANGUAGES C)

option(MAUL_ANIM_BUILD_TESTS "Build the test suites" ${PROJECT_IS_TOP_LEVEL})
option(MAUL_ANIM_BUILD_BENCH "Build the benchmarks" ${PROJECT_IS_TOP_LEVEL})
option(MAUL_ANIM_BUILD_SHARED "Build a shared library instead of a static one" OFF)
option(MAUL_ANIM_INSTALL "Generate install rules" ${PROJECT_IS_TOP_LEVEL})
option(MAUL_ANIM_WERROR "Treat compiler warnings as errors" OFF)
option(MAUL_ANIM_SANITIZE "Instrument with AddressSanitizer and UBSan" OFF)
option(MAUL_ANIM_TSAN "Instrument with ThreadSanitizer" OFF)
option(MAUL_ANIM_COVERAGE "Instrument for gcov coverage" OFF)
option(MAUL_ANIM_FUZZ "Build the libFuzzer targets (Clang)" OFF)
option(MAUL_ANIM_OFFLINE "Build the offline side: builders, compression, import" ON)

# Every executable (tests, benchmarks, tools) lands in the top of the
# build tree, so scripts and CI find them in one place.
if(PROJECT_IS_TOP_LEVEL)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
endif()

include(GNUInstallDirs)
include(cmake/CompilerFlags.cmake)
include(cmake/Library.cmake)
include(cmake/Testing.cmake)

if(MAUL_ANIM_BUILD_SHARED)
    set(MAUL_LIBRARY_KIND SHARED)
else()
    set(MAUL_LIBRARY_KIND STATIC)
endif()
add_library(maul-anim ${MAUL_LIBRARY_KIND}
    src/base.c
    src/container.c
    src/decompose.c
    src/elementary_float.c
    src/fpenv.c
    src/hash.c
    src/pose.c
    src/scalar.c
    src/skeleton.c
    src/skin.c
    src/text.c
    src/vecmath.c)
# The offline side (builders, compression, import) is part of the one
# library, compiled when MAUL_ANIM_OFFLINE is on; tools/modules.txt keeps
# the runtime from using it.
if(MAUL_ANIM_OFFLINE)
    target_sources(maul-anim PRIVATE src/allocator.c src/builder.c src/inflate.c
                                     src/skeleton_build.c src/skin_build.c)
endif()
maul_configure_library(maul-anim)
# The exactly rounded functions the sources may call
# (tools/libm-allowed.txt) come from libm (mani-0001).
if(NOT WIN32 AND NOT APPLE)
    target_link_libraries(maul-anim PUBLIC m)
endif()

if(MAUL_ANIM_BUILD_TESTS)
    enable_testing()
    maul_add_test(base)
    maul_add_test(container WHITEBOX)
    maul_add_test(decompose WHITEBOX)
    maul_add_test(hash)
    maul_add_test(math_float WHITEBOX)
    maul_add_test(fpenv WHITEBOX)
    maul_add_test(scalar WHITEBOX)
    maul_add_test(text WHITEBOX)
    maul_add_test(vecmath WHITEBOX)
    if(MAUL_ANIM_OFFLINE)
        maul_add_test(inflate WHITEBOX)
        maul_add_test(skeleton WHITEBOX)
        maul_add_test(pose)
        maul_add_test(skin WHITEBOX)
    endif()
    maul_add_header_tests()
endif()

# The fuzz targets: test/fuzz_<area>.c, run as fuzz_<area> <corpus dir>,
# over the bytes programs hand the library; test_skeleton --seed <file>
# writes fuzz_skeleton's first seed.
if(MAUL_ANIM_FUZZ)
    # The library is instrumented too: its coverage guides libFuzzer, and
    # the sanitizers watch its code. Undefined behaviour stops the run.
    target_compile_options(maul-anim PRIVATE -fsanitize=fuzzer-no-link,address,undefined
                                             -fno-sanitize-recover=all)
    target_link_options(maul-anim PUBLIC -fsanitize=address,undefined)
    foreach(area skeleton skin)
        add_executable(fuzz_${area} test/fuzz_${area}.c)
        target_link_libraries(fuzz_${area} PRIVATE maul-anim)
        maul_apply_flags(fuzz_${area})
        target_compile_options(fuzz_${area} PRIVATE -fsanitize=fuzzer,address,undefined
                                                      -fno-sanitize-recover=all)
        target_link_options(fuzz_${area} PRIVATE -fsanitize=fuzzer,address,undefined)
    endforeach()
endif()

if(MAUL_ANIM_BUILD_BENCH AND EXISTS ${PROJECT_SOURCE_DIR}/bench/bench_main.c)
    add_subdirectory(bench)
endif()

if(MAUL_ANIM_INSTALL)
    # Without the offline side, its header is not installed.
    if(NOT MAUL_ANIM_OFFLINE)
        file(GLOB MAUL_PUBLIC_HEADERS RELATIVE ${PROJECT_SOURCE_DIR}/include/maul-anim
            ${PROJECT_SOURCE_DIR}/include/maul-anim/*.h)
        list(REMOVE_ITEM MAUL_PUBLIC_HEADERS build.h)
    endif()
    include(cmake/Install.cmake)
endif()
