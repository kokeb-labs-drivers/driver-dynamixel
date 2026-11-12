# Runs a fuzz replay built with a deliberate mutation and passes only when it aborts naming a broken
# promise. CTest alone cannot do this: it reports an aborted process as a failure even with WILL_FAIL.
#   cmake -DEXE=<replay> -DCORPUS=<dir> -P expect_broken_promise.cmake
execute_process(COMMAND ${EXE} ${CORPUS}
  RESULT_VARIABLE status OUTPUT_VARIABLE out ERROR_VARIABLE err)
if(status EQUAL 0 OR NOT err MATCHES "promise broken at line [0-9]+")
  message(FATAL_ERROR "the mutant was not caught (status ${status}): ${out}${err}")
endif()
