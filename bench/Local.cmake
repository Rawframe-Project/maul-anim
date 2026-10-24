# The benchmark's scenes, built once at its start.
target_sources(${PROJECT_NAME}_bench PRIVATE scenes.c)

# Motion matching's search by bounds against an exact k-d tree, on
# synthetic databases of several sizes and feature counts.
add_executable(${PROJECT_NAME}_bench_match bench_match.c)
target_link_libraries(${PROJECT_NAME}_bench_match PRIVATE ${PROJECT_NAME})
maul_apply_flags(${PROJECT_NAME}_bench_match)
