# The package test (MAUL_ANIM_TEST_PACKAGE): installs the build into
# BINARY_DIR/package-test/prefix, then configures, builds and runs
# test/consumer against it with nothing else on its search path, and,
# where pkg-config is found, compiles the consumer from the pkg-config
# module's flags alone. The consumer makes the CPU check through the
# installed header, which proves the package exports it (a shared
# library's too) and names its SIMD backend. Run by CTest as the test
# "package"; the build must be complete.
#
#   cmake -DBINARY_DIR=... -DSOURCE_DIR=... -DGENERATOR=... -DC_COMPILER=...
#         -DBACKEND=<the build's SIMD backend> [-DCONFIG=Release]
#         [-DTOOLSET=ClangCL] -P PackageTest.cmake

set(root ${BINARY_DIR}/package-test)
file(REMOVE_RECURSE ${root})

function(run)
    execute_process(COMMAND ${ARGN} RESULT_VARIABLE result)
    if(NOT result EQUAL 0)
        list(JOIN ARGN " " command)
        message(FATAL_ERROR "failed (${result}): ${command}")
    endif()
endfunction()

set(config_args)
set(exe ${root}/build/consumer)
if(CONFIG)
    set(config_args --config ${CONFIG})
    set(exe ${root}/build/${CONFIG}/consumer)
endif()
if(WIN32)
    string(APPEND exe .exe)
endif()
set(configure_args -G ${GENERATOR} -DCMAKE_PREFIX_PATH=${root}/prefix)
if(TOOLSET)
    list(APPEND configure_args -T ${TOOLSET})
else()
    list(APPEND configure_args -DCMAKE_C_COMPILER=${C_COMPILER})
endif()

run(${CMAKE_COMMAND} --install ${BINARY_DIR} --prefix ${root}/prefix ${config_args})
run(${CMAKE_COMMAND} -S ${SOURCE_DIR}/test/consumer -B ${root}/build ${configure_args})
run(${CMAKE_COMMAND} --build ${root}/build ${config_args})
# A shared library is found on the loader's search path: lib on Linux,
# bin (on PATH) on Windows. Set here rather than through cmake -E env, as
# a list argument would split Windows' PATH at its semicolons.
if(WIN32)
    set(ENV{PATH} "${root}/prefix/bin;$ENV{PATH}")
else()
    set(ENV{LD_LIBRARY_PATH} "${root}/prefix/lib:${root}/prefix/lib64")
endif()
run(${exe})

find_program(PKG_CONFIG pkg-config)
if(PKG_CONFIG AND NOT WIN32)
    set(ENV{PKG_CONFIG_PATH} "${root}/prefix/lib/pkgconfig:${root}/prefix/lib64/pkgconfig")
    execute_process(COMMAND ${PKG_CONFIG} --cflags --libs maul-anim OUTPUT_VARIABLE flags
                    OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE result)
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "pkg-config does not find the installed maul-anim")
    endif()
    separate_arguments(flags UNIX_COMMAND "${flags}")
    run(${C_COMPILER} -std=c17 ${SOURCE_DIR}/test/consumer/main.c
        "-DCONSUMER_BACKEND=\"${BACKEND}\"" ${flags} -o ${root}/consumer-pc)
    run(${root}/consumer-pc)
endif()
