# Runs the host example and compares its output with examples/host/expected_output.txt.
# Usage: cmake -DEXE=<binary> -DEXPECTED=<file> -P check_example_output.cmake
execute_process(COMMAND ${EXE} OUTPUT_VARIABLE actual RESULT_VARIABLE code)
if(NOT code EQUAL 0)
  message(FATAL_ERROR "the host example exited with ${code}")
endif()
file(READ ${EXPECTED} expected)
# The version line follows the build: compare everything after it.
string(REGEX REPLACE "^[^\n]*\n" "" actual_rest "${actual}")
string(REGEX REPLACE "^[^\n]*\n" "" expected_rest "${expected}")
if(NOT actual_rest STREQUAL expected_rest)
  message(FATAL_ERROR "host example output differs from expected_output.txt:\n${actual}")
endif()
