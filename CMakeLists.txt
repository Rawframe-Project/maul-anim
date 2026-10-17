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
option(MAUL_ANIM_DIFFERENTIAL
       "Build the ufbx, ACL and ozz differentials (fetch ufbx, ACL, RTM and ozz-animation; mani-0018, mani-0019, mani-0020)" OFF)

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
    src/clip.c
    src/root_motion.c
    src/clip_codec.c
    src/additive.c
    src/blend.c
    src/clip_compressed.c
    src/clip_cursor.c
    src/clip_tracks.c
    src/container.c
    src/decompose.c
    src/elementary_float.c
    src/fpenv.c
    src/graph.c
    src/graph_bind.c
    src/graph_instance.c
    src/graph_nodes.c
    src/graph_machine_file.c
    src/graph_machine.c
    src/graph_machine_step.c
    src/graph_machine_listen.c
    src/graph_shot.c
    src/graph_shot_step.c
    src/graph_dead_blend.c
    src/graph_inertia.c
    src/graph_inertia_apply.c
    src/graph_emit.c
    src/graph_match.c
    src/graph_match_step.c
    src/graph_space.c
    src/graph_update.c
    src/graph_evaluate.c
    src/hash.c
    src/influences.c
    src/mask.c
    src/match.c
    src/match_search.c
    src/match_move.c
    src/trajectory.c
    src/learned.c
    src/learned_eval.c
    src/pose.c
    src/scalar.c
    src/skeleton.c
    src/skin.c
    src/skinning.c
    src/morph.c
    src/text.c
    src/vecmath.c)
# The offline side (builders, compression, import) is part of the one
# library, compiled when MAUL_ANIM_OFFLINE is on; tools/modules.txt keeps
# the runtime from using it.
if(MAUL_ANIM_OFFLINE)
    target_sources(maul-anim PRIVATE src/allocator.c src/builder.c src/clip_build.c src/quat64.c src/root_motion_build.c
                                     src/clip_build_named.c src/clip_compress.c src/clip_measure.c src/clip_plan.c src/clip_search.c src/clip_shell.c src/decimal.c src/decimal_format.c src/inflate.c
                                     src/arena.c src/gltf.c src/gltf_animation.c src/basis.c src/gltf_basis.c
                                     src/gltf_clip.c src/gltf_mesh.c
                                     src/gltf_buffer.c src/gltf_layout.c
                                     src/gltf_names.c src/hierarchy.c src/names.c src/gltf_node.c src/gltf_property.c
                                     src/gltf_rest.c src/gltf_skeleton.c src/gltf_spring.c src/json.c src/skeleton_build.c
                                     src/fbx.c src/fbx_anim.c src/fbx_ascii.c src/fbx_binary.c src/fbx_blend.c src/fbx_clip.c src/fbx_clip_state.c src/fbx_curve.c src/fbx_helpers.c src/fbx_influences.c src/fbx_morph.c src/fbx_keys.c src/fbx_legacy.c src/fbx_model.c src/fbx_object.c src/fbx_options.c src/fbx_sample.c src/fbx_skeleton.c src/fbx_skin.c src/fbx_take.c src/fbx_tracks.c src/fbx_transform.c src/fbx_units.c src/fbx_value.c src/fbx_world.c src/sort.c
                                     src/skin_build.c src/influences_build.c src/centers_build.c
                                     src/morph_build.c src/graph_check.c src/graph_space_check.c src/graph_machine_check.c src/graph_match_check.c src/exact.c src/triangulate.c src/graph_build.c src/graph_inline.c src/graph_json.c src/graph_json_read.c src/graph_json_graph.c src/graph_json_machine.c
                                     src/match_sample.c src/match_build.c src/learned_build.c
                                     src/train_random.c src/train_net.c src/train_adam.c src/train_tasks.c
                                     src/train_data.c src/train_decompress.c src/train_decompress_loss.c src/train_kinematics.c
                                     src/train_solo.c src/train_stepper.c src/train_projector.c
                                     src/learned_train.c src/learned_train_run.c src/learned_checkpoint.c src/learned_restore.c)
endif()
maul_configure_library(maul-anim)
# sqrtf need not set errno, which the library never reads: square roots
# then run four or eight at a time; no value changes (mani-0020).
if(NOT MSVC)
    target_compile_options(maul-anim PRIVATE -fno-math-errno)
endif()
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
        maul_add_test(pose WHITEBOX)
        maul_add_test(skin WHITEBOX)
        maul_add_test(influences WHITEBOX)
        maul_add_test(skinning WHITEBOX)
        maul_add_test(morph WHITEBOX)
        maul_add_test(clip WHITEBOX)
        maul_add_test(clip_compress WHITEBOX)
        maul_add_test(root_motion WHITEBOX)
        maul_add_test(graph WHITEBOX)
        maul_add_test(graph_json)
        maul_add_test(graph_update WHITEBOX)
        maul_add_test(graph_build WHITEBOX)
        maul_add_test(graph_space WHITEBOX)
        maul_add_test(graph_triangles WHITEBOX)
        maul_add_test(exact WHITEBOX)
        maul_add_test(graph_machine WHITEBOX)
        maul_add_test(graph_listen WHITEBOX)
        maul_add_test(graph_shot WHITEBOX)
        maul_add_test(graph_inertia WHITEBOX)
        maul_add_test(graph_dead_blend WHITEBOX)
        maul_add_test(match WHITEBOX)
        maul_add_test(match_search WHITEBOX)
        maul_add_test(graph_match WHITEBOX)
        maul_add_test(graph_match_file WHITEBOX)
        maul_add_test(graph_models WHITEBOX)
        maul_add_test(trajectory WHITEBOX)
        maul_add_test(learned WHITEBOX)
        maul_add_test(train WHITEBOX)
        maul_add_test(train_decompress WHITEBOX)
        maul_add_test(train_stages WHITEBOX)
        maul_add_test(learned_train WHITEBOX)
        maul_add_test(blend WHITEBOX)
        maul_add_test(json WHITEBOX)
        maul_add_test(gltf WHITEBOX)
        maul_add_test(gltf_skeleton WHITEBOX)
        maul_add_test(gltf_clip WHITEBOX)
        maul_add_test(gltf_spring WHITEBOX)
        maul_add_test(arena WHITEBOX)
        maul_add_test(fbx WHITEBOX)
        maul_add_test(fbx_object WHITEBOX)
        maul_add_test(fbx_transform WHITEBOX)
        maul_add_test(fbx_model WHITEBOX)
        maul_add_test(fbx_world WHITEBOX)
        maul_add_test(fbx_units WHITEBOX)
        maul_add_test(fbx_skeleton WHITEBOX)
        maul_add_test(fbx_skin WHITEBOX)
        maul_add_test(fbx_curve WHITEBOX)
        maul_add_test(fbx_anim WHITEBOX)
        maul_add_test(fbx_clip WHITEBOX)
        maul_add_test(fbx_blend WHITEBOX)
        maul_add_test(fbx_keys WHITEBOX)
        maul_add_test(fbx_take WHITEBOX)
        maul_add_test(fbx_legacy WHITEBOX)
        maul_add_test(basis WHITEBOX)
        maul_add_test(hierarchy WHITEBOX)
        # Khronos sample models in test/data/gltf; FBX scenes in
        # test/data/fbx against ufbx's values (mani-0018).
        maul_add_test(gltf_fixtures WHITEBOX)
        maul_add_test(gltf_mesh WHITEBOX)
        maul_add_test(fbx_influences)
        maul_add_test(fbx_morph)
        maul_add_test(fbx_fixtures)
        foreach(fixtures IN ITEMS test_gltf_fixtures test_gltf_mesh test_fbx_fixtures)
            if(TARGET ${fixtures})
                target_compile_definitions(${fixtures} PRIVATE MANI_TEST_DATA="${PROJECT_SOURCE_DIR}/test/data")
                if(EMSCRIPTEN)
                    target_link_options(${fixtures} PRIVATE -sNODERAWFS=1)
                endif()
                if(WIN32)
                    target_compile_definitions(${fixtures} PRIVATE _CRT_SECURE_NO_WARNINGS)
                endif()
            endif()
        endforeach()
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
    foreach(area skeleton skin influences morph clip graph graph_state graph_json match learned json gltf fbx)
        add_executable(fuzz_${area} test/fuzz_${area}.c)
        target_link_libraries(fuzz_${area} PRIVATE maul-anim)
        # Internal readers (JSON) are fuzzed through their own headers.
        target_include_directories(fuzz_${area} PRIVATE ${PROJECT_SOURCE_DIR}/src)
        maul_apply_flags(fuzz_${area})
        target_compile_options(fuzz_${area} PRIVATE -fsanitize=fuzzer,address,undefined
                                                      -fno-sanitize-recover=all)
        target_link_options(fuzz_${area} PRIVATE -fsanitize=fuzzer,address,undefined)
    endforeach()
endif()

# The ufbx differential (mani-0018): ufbx at its pinned commit, with its
# corpus, fetched for the tool alone and never linked into the library.
if(MAUL_ANIM_DIFFERENTIAL)
    if(NOT MAUL_ANIM_OFFLINE)
        message(FATAL_ERROR "MAUL_ANIM_DIFFERENTIAL needs MAUL_ANIM_OFFLINE")
    endif()
    include(FetchContent)
    FetchContent_Declare(ufbx GIT_REPOSITORY https://github.com/ufbx/ufbx.git
                              GIT_TAG 5955c5c0b042ac2dc6f32955ab206aaec0620cfc)
    FetchContent_MakeAvailable(ufbx)
    add_executable(fbx_differential tools/fbx_differential.c tools/fbx_differential_mesh.c
        ${ufbx_SOURCE_DIR}/ufbx.c)
    target_include_directories(fbx_differential SYSTEM PRIVATE ${ufbx_SOURCE_DIR})
    target_link_libraries(fbx_differential PRIVATE maul-anim)
    maul_apply_flags(fbx_differential)
    if(NOT MSVC)
        target_link_libraries(fbx_differential PRIVATE m)
    endif()
    # ufbx is someone else's code: built as it is, without our warnings.
    set_source_files_properties(${ufbx_SOURCE_DIR}/ufbx.c PROPERTIES COMPILE_OPTIONS -w)
    set(MAUL_ANIM_UFBX_DATA ${ufbx_SOURCE_DIR}/data CACHE INTERNAL "ufbx's corpus")
    # The ACL differential, the one C++ program: ACL and RTM are header
    # only, read at the commits research measured them at; their own
    # projects are not built (SOURCE_SUBDIR names none).
    enable_language(CXX)
    FetchContent_Declare(acl GIT_REPOSITORY https://github.com/nfrechette/acl.git
                             GIT_TAG 3ee568542eca4428e1041908b4a644b98e7885bd
                             GIT_SUBMODULES "" SOURCE_SUBDIR not-built)
    FetchContent_Declare(rtm GIT_REPOSITORY https://github.com/nfrechette/rtm.git
                             GIT_TAG dd384b40c4a8f15ceff4f743c4f3ae7ce4730981
                             GIT_SUBMODULES "" SOURCE_SUBDIR not-built)
    FetchContent_MakeAvailable(acl rtm)
    add_executable(acl_differential tools/acl_differential.cc)
    set_target_properties(acl_differential PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON
                                                      CXX_EXTENSIONS OFF)
    target_include_directories(acl_differential SYSTEM PRIVATE ${acl_SOURCE_DIR}/includes
                                                               ${rtm_SOURCE_DIR}/includes)
    target_include_directories(acl_differential PRIVATE ${PROJECT_SOURCE_DIR}/src)
    target_link_libraries(acl_differential PRIVATE maul-anim)
    maul_apply_flags(acl_differential)
    # The ozz differential: the few ozz-animation sources blending and
    # building a skeleton need, compiled as they are; its own project
    # (CMake 3.30, samples, tools) is not built.
    FetchContent_Declare(ozz GIT_REPOSITORY https://github.com/guillaumeblanc/ozz-animation.git
                             GIT_TAG 744eb9d99f606eda849acb0b1204f7a3dc20bca1
                             GIT_SUBMODULES "" SOURCE_SUBDIR not-built)
    FetchContent_MakeAvailable(ozz)
    set(MAUL_OZZ_SOURCES
        src/base/memory/allocator.cc src/base/platform.cc src/base/log.cc src/base/io/archive.cc
        src/base/io/stream.cc src/base/containers/string_archive.cc
        src/base/maths/simd_math_archive.cc src/base/maths/soa_math_archive.cc
        src/base/maths/math_archive.cc src/base/maths/simd_math.cc
        src/animation/runtime/skeleton.cc src/animation/runtime/blending_job.cc
        src/animation/offline/raw_skeleton.cc src/animation/offline/skeleton_builder.cc)
    list(TRANSFORM MAUL_OZZ_SOURCES PREPEND ${ozz_SOURCE_DIR}/)
    add_executable(ozz_differential tools/ozz_differential.cc ${MAUL_OZZ_SOURCES})
    set_target_properties(ozz_differential PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON
                                                      CXX_EXTENSIONS OFF)
    target_include_directories(ozz_differential SYSTEM PRIVATE ${ozz_SOURCE_DIR}/include
                                                               ${ozz_SOURCE_DIR}/src)
    target_link_libraries(ozz_differential PRIVATE maul-anim)
    maul_apply_flags(ozz_differential)
    set_source_files_properties(${MAUL_OZZ_SOURCES} PROPERTIES COMPILE_OPTIONS -w)
endif()

if(MAUL_ANIM_BUILD_BENCH AND EXISTS ${PROJECT_SOURCE_DIR}/bench/bench_main.c)
    add_subdirectory(bench)
endif()

if(MAUL_ANIM_INSTALL)
    # Without the offline side, its headers are not installed.
    if(NOT MAUL_ANIM_OFFLINE)
        file(GLOB MAUL_PUBLIC_HEADERS RELATIVE ${PROJECT_SOURCE_DIR}/include/maul-anim
            ${PROJECT_SOURCE_DIR}/include/maul-anim/*.h)
        list(REMOVE_ITEM MAUL_PUBLIC_HEADERS build.h import.h)
    endif()
    include(cmake/Install.cmake)
endif()
