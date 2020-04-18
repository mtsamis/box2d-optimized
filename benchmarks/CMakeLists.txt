set (BENCHMARK_SOURCE_FILES
    main.cpp
)

add_executable(benchmark ${BENCHMARK_SOURCE_FILES})

set_target_properties(benchmark PROPERTIES
	CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

target_link_libraries(benchmark PUBLIC box2d)

# Place the test executable at the project binary directory instead of in the nested subfolder
set_target_properties(benchmark PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${BENCHMARK_SOURCE_FILES})
