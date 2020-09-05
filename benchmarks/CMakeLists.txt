
add_executable(benchmark_single single.cpp benchmarks.h)
add_executable(benchmark_full full.cpp benchmarks.h)

set_target_properties(benchmark_single benchmark_full PROPERTIES
	CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

target_link_libraries(benchmark_single PUBLIC box2d)
target_link_libraries(benchmark_full PUBLIC box2d)

# Place the test executable at the project binary directory instead of in the nested subfolder
set_target_properties(benchmark_single benchmark_full PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES single.cpp full.cpp benchmarks.h)
