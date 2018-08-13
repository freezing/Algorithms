if (LIBALGORITHM_INCLUDE_GOOGLETEST)
    add_subdirectory(googletest)
endif()

if (LIBALGORITHM_INCLUDE_GOOGLEBENCHMARK)
    add_subdirectory(benchmark)
endif()