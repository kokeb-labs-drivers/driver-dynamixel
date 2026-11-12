# =============================================================================
# Fuzz targets (docs/testing.md, "Fuzz tests")
# =============================================================================
# Each tests/fuzz/fuzz_<name>.cpp defines LLVMFuzzerTestOneInput() and is built twice:
#
#   fuzz_replay_<name>   any compiler; a CTest that replays tests/fuzz/corpus/fuzz_<name>/
#   fuzz_<name>          Clang with -DDYNAMIXEL_BUILD_FUZZERS=ON: libFuzzer with ASan and UBSan
#
#   cmake -S . -B build-fuzz -DCMAKE_CXX_COMPILER=clang++ -DDYNAMIXEL_BUILD_TESTS=ON -DDYNAMIXEL_BUILD_FUZZERS=ON
#   cmake --build build-fuzz --target fuzz_dynamixel
#   ./build-fuzz/tests/fuzz/fuzz_dynamixel -max_total_time=60 tests/fuzz/corpus/fuzz_dynamixel
# =============================================================================

file(GLOB DYNAMIXEL_FUZZ_TARGETS CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/fuzz_*.cpp)
foreach(src ${DYNAMIXEL_FUZZ_TARGETS})
  get_filename_component(name ${src} NAME_WE)

  add_executable(fuzz_replay_${name} ${src} replay_main.cpp)
  dynamixel_test_target(fuzz_replay_${name})
  target_include_directories(fuzz_replay_${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
  add_test(NAME dynamixel_${name}_corpus
    COMMAND fuzz_replay_${name} ${CMAKE_CURRENT_SOURCE_DIR}/corpus/${name})

  if(name STREQUAL "fuzz_dynamixel")
    # The same target with one edit (the port stops counting writes). Its corpus run must abort,
    # which proves the promise "a stop or a reboot writes at least once" is checked under every
    # compiler, whatever order it evaluates the arguments of a call in.
    add_executable(fuzz_mutant_${name} ${src} replay_main.cpp)
    dynamixel_test_target(fuzz_mutant_${name})
    target_include_directories(fuzz_mutant_${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
    target_compile_definitions(fuzz_mutant_${name} PRIVATE DYNAMIXEL_FUZZ_MUTANT_UNCOUNTED_WRITES)
    add_test(NAME dynamixel_${name}_mutant_uncounted_writes
      COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:fuzz_mutant_${name}>
        -DCORPUS=${CMAKE_CURRENT_SOURCE_DIR}/corpus/${name}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/expect_broken_promise.cmake)
  endif()

  if(DYNAMIXEL_BUILD_FUZZERS)
    add_executable(${name} ${src})
    dynamixel_test_target(${name})
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
    target_compile_options(${name} PRIVATE -fsanitize=fuzzer,address,undefined -fno-sanitize-recover=all -g -O1)
    target_link_options(${name} PRIVATE -fsanitize=fuzzer,address,undefined)
  endif()
endforeach()
