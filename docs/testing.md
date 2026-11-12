# Testing and CI

How the driver is checked: host tests against a model of the servo bus, two fuzzers that cut, corrupt and replace what
the servos send, a mutation run over the driver, compiled documentation, an install test, Cortex-M builds, static
analysis and measured quality gates, on every pull request.

## Host tests

The tests build the real driver against a model of a DYNAMIXEL bus and run 204 test cases in one executable
(`dynamixel_host_tests`), among 22 CTest entries. They build with the firmware's flags, `-fno-exceptions -fno-rtti`,
and warnings as errors, on GCC and Clang.

```bash
cmake -S . -B build -DDYNAMIXEL_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

| File | Cases | Covers |
| --- | --- | --- |
| [`tests/test_driver.cpp`](../tests/test_driver.cpp) | 29 | Lifecycle, discovery, raw read and write, reply rules by Status Return Level, reset and reboot |
| [`tests/test_faults.cpp`](../tests/test_faults.cpp) | 39 | Reply validity, resynchronisation, retries, line faults, echo, port failures, alert policy, the safe stop (flush failure, no answer to confirm, start-up order, family against protocol) |
| [`tests/test_multi.cpp`](../tests/test_multi.cpp) | 26 | Broadcast ping, ID scan, Sync and Bulk read and write, the fast replies, per-device outcomes |
| [`tests/test_typed.cpp`](../tests/test_typed.cpp) | 14 | The typed register API, feedback, ID and baud changes, the status return level |
| [`tests/test_protocol.cpp`](../tests/test_protocol.cpp) | 15 | CRC and checksum against the e-Manual's examples and an independent computation, byte stuffing, the reply search |
| [`tests/test_model.cpp`](../tests/test_model.cpp) | 8 | Configuration checks, the control-table catalogue, the error names |
| [`tests/test_boundaries.cpp`](../tests/test_boundaries.cpp) | 58 | The edges of every check: exact argument and buffer limits, which timeout each read gets, retry and settle counts, per-device outcomes of Sync and Fast reads |
| [`tests/test_boundaries_more.cpp`](../tests/test_boundaries_more.cpp) | 15 | The survivors of the independent mutation run: stuffing, short status packets, protocol 1.0 limits, echo, broadcast ping, a stuck line in a Sync Read, the stop on an invalid configuration, the time a stop can take, the protocol 1.0 stop bytes. A second file because GCC's condition coverage gives up above 64 registrations in one file; the shared helpers are in [`tests/boundary_support.hpp`](../tests/boundary_support.hpp) |
| [`tests/test_kconfig.cpp`](../tests/test_kconfig.cpp), [`tests/test_version.cpp`](../tests/test_version.cpp) | | `KconfigDefaults()` with and without `CONFIG_` values; the version getters follow the generated header |
| [`tests/compile_fail/`](../tests/compile_fail/port_without_delay.cpp) | 10 | Code that must not compile, each for its `EXPECT:` diagnostic: a port without `DelayUs()`, a narrowed or non-`noexcept` hook, a wrong `Log()` or `SetBaudRate()`, an invalid `CheckedConfig`, a class that is not a port |
| [`tests/fuzz/`](../tests/fuzz/fuzz_dynamixel.cpp) | 2 | CTest replays the corpus of both fuzz targets (`dynamixel_fuzz_dynamixel_corpus`, `dynamixel_fuzz_scan_corpus`), see below |
| [`examples/host/`](../examples/host/README.md) | 2 | The host example, built with the test flags, run, and its output compared with `expected_output.txt` |
| [`tests/check_install.cmake`](../tests/check_install.cmake) | 1 | Installs into a scratch prefix and builds [`tests/install_consumer/`](../tests/install_consumer/CMakeLists.txt) with `find_package(dynamixel)` |
| [`tests/check_doxygen.cmake`](../tests/check_doxygen.cmake) | 1 | Doxygen with `_config/Doxyfile` and warnings as errors, when `doxygen` is installed |

### The bus model stands alone

[`tests/mock_servo_bus.hpp`](../tests/mock_servo_bus.hpp) is written from the e-Manual, not from the driver: it does
not include a driver header. Its CRC, byte stuffing, packet layout and status packets are its own, so a wrong
constant or a misread rule in the driver cannot hide behind the same mistake in the model (handbook KB-TST-030). The
driver's CRC and checksum are also checked against the worked examples of the e-Manual and against an independent
bitwise computation in `tests/test_protocol.cpp`.

The model has a servo per ID with a memory map and a fault knob for each thing a real bus does: a silent servo, a
forced error byte, an alert, dropped replies, a corrupted CRC, a truncated packet, garbage before a reply, a stuck
line, a reply from the wrong ID, a reply split into pieces, the echo of a single-wire bus, an extra parameter, an
oversized read. Every behaviour found on hardware becomes a knob and a test that fails on the code before the fix.

## Fuzz tests

Everything the driver reads comes from the servos over the bus, so the bytes read are what a noisy cable, a wrong
servo or a broken one controls. Two libFuzzer targets, built with ASan and UBSan, check the driver's promises after
every call and abort, naming the line, when one breaks:

- [`fuzz_dynamixel.cpp`](../tests/fuzz/fuzz_dynamixel.cpp) chooses a configuration and runs up to 30 operations that
  cover the whole API on the bus model, and on every transfer chooses what the bus does: pass it on, fail it, cut it,
  flip bits or replace it. It checks that every error is one the enum defines, that a failed call returns `T{}`,
  that nothing is sent for `InvalidArgument`, `Unsupported`, `NotPermitted` or `NotInitialized`, that the statistics
  never go backwards, that a multi-read that returns `Ok` has a defined outcome per device, and that every packet
  written fits the buffer. It also checks the safety rule: `DisableTorque()`, `DisableTorqueAll()`,
  `DisableTorqueEach()`, `DisableTorqueAt()` and `Reboot()` are called in any state (not initialised, a failing
  flush, failing reads, an invalid configuration) and are never refused: unless the argument is invalid or the port
  cannot start, at least one packet is written.
- [`fuzz_scan.cpp`](../tests/fuzz/fuzz_scan.cpp) feeds raw bytes as the receive line to Ping, Read, Write,
  BroadcastPing, SyncRead, FastSyncRead, BulkRead and FastBulkRead, and checks that every call ends, that nothing is
  written outside its buffer, and that the same promises hold.

```bash
# libFuzzer needs Clang (on macOS, Homebrew's LLVM)
cmake -S . -B build-fuzz -DCMAKE_CXX_COMPILER=clang++ -DDYNAMIXEL_BUILD_TESTS=ON -DDYNAMIXEL_BUILD_FUZZERS=ON
cmake --build build-fuzz --target fuzz_dynamixel fuzz_scan
./build-fuzz/tests/fuzz/fuzz_dynamixel -max_total_time=120 tests/fuzz/corpus/fuzz_dynamixel
./build-fuzz/tests/fuzz/fuzz_scan -max_total_time=120 tests/fuzz/corpus/fuzz_scan
```

The corpora ([`fuzz_dynamixel`](../tests/fuzz/corpus/fuzz_dynamixel), 867 inputs, and
[`fuzz_scan`](../tests/fuzz/corpus/fuzz_scan), 104 inputs) were built by a libFuzzer run of 240 s and 100 s and
reduced with set-cover merge. A later run of 120 s per target found one broken promise, a stop on an invalid
configuration that read with a zero timeout; it is fixed and its input is kept as `regress-stop-invalid-config`. The
run after the fix (120 s per target) found nothing. The same files build without
libFuzzer as `fuzz_replay_*`, and CTest replays every input on GCC and Clang. An input that ever broke a promise
is added under the name `regress-*` so that a fixed bug stays fixed. CI fuzzes for 120 s per target on every pull
request, from the corpus plus the one it keeps between runs (job `Host quality / fuzz`).

## Mutation testing

A line coverage figure says a line ran, not that a test would notice if it were wrong. To check that, a small harness
(kept outside the repository) changes the driver one place at a time, builds the tests and runs them: a mutant that
the tests do not fail ("survives") marks a check nothing tests. The mutations are the usual ones: a comparison
operator flipped or off by one, a constant changed by one, `&&` and `||` swapped, a condition negated, a `return`
value changed, a statement removed.

The run on the driver of that time (663 mutants of `inc/` and `src/`, one per line, built and run one at a time):

| Outcome | Mutants |
| --- | --- |
| Killed by a test | 500 |
| Did not compile (not counted) | 97 |
| Survived | 66 |

500 of the 566 mutants that compile were killed (88 %). The first run killed 394 of 567; the survivors were read, and
[`tests/test_boundaries.cpp`](../tests/test_boundaries.cpp) was written from them (the exact limits of every
argument and buffer check, which timeout each read gets, per-device outcomes of damaged Fast replies, the limits
of `Validate()`), which killed 106 more. The 66 that remain were read and are accepted, not proven equivalent one by
one:

- About 8 are `len == 0` checks after `FrameWriter::Finish()` that cannot trigger with the argument checks before them
  (the packet always fits the buffer by then); they are defence in depth.
- About 8 change only a comment or the initial value of a flag that every path overwrites before it is read.
- About 20 change a start value or step of a loop that walks the same entries again (`k + 0` for `k + 1`), or an
  initial value in `Scan` that the next statement overwrites.
- About 15 move a buffer-fit limit by one (`>` for `>=`), reachable only with a receive buffer of exactly that size.
- The rest are in the walk through a damaged Fast packet (a part that stopped at the very byte of its ID or CRC); the
  by-hand tests cover the cases that matter, and the fuzz targets cover the rest.

A later pass, by hand, took the survivors an independent review found in 59 mutants of the safety, scan, stuffing,
limit and Sync/Bulk code, and the mutants of the fixes of that review. Each row is one edit; every one is killed by a
test written for it (the corpus replay of the fuzz target also kills the first row):

| Mutant (one edit) | Killed by |
| --- | --- |
| The input flush refuses a stop again | `A stop is sent even when the input flush fails`, fuzz corpus replay |
| A stop confirmed or sent once when no answer can confirm it; `kStopAttempts` 3 to 2 | `A stop that no answer can confirm is sent three times` and the retry tests |
| An answer trusted after a failed flush; the flush fault not reported after a stop or a reboot | the flush tests |
| A stop does not set the baud rate; `Reboot()` refused before initialisation | `A stop before EnsureInitialized sets the bus baud rate`, the fuzz corpus replay |
| The family against protocol check removed or its bits swapped | `A family that does not speak the bus protocol is refused` |
| `FrameWriter::Put` run clamp 2 to 3, or stuffing after one `FF` | `FrameWriter 2.0: FF FF FF FD is stuffed` |
| `Destuff` pattern after one `FF`, or run clamp 2 to 3 | `Destuff: ...`, `A reply whose data is FF FD is read as data` |
| V2 status length guard 4 to 3 | `Scan 2.0: a length below 4 ...`, `A length-3 status packet ...` |
| V1 ID bound dropped; V1 exact-fit status; V1 exact-fit write | `Scan 1.0: only IDs up to 253 can answer ...`, `FrameWriter 1.0: a buffer that holds the checksum ...` |
| Echo oversize read tolerated by one byte | `Echo mode: a read that claims more than was asked for is a bus failure` |
| Broadcast ping ID bound dropped; the scan ends on one error servo; no reply count bound | `Broadcast ping: ...` (three tests) |
| Sync Read stops the rest only on `Bus`, not on `LineFault` | `Sync read: a stuck line marks the first device and every one after it a line fault` |
| A refusal in the answer ends the attempts after a flush fault; the flush fault not kept across attempts; the "sent, no answer" outcome dropped; the raw stop's address or ID check loosened | `A refusal in the answer does not end the attempts ...`, `A flush that failed once taints the stop`, `A stop that no answer can confirm is Ok ...`, `DisableTorqueAt ...` |
| The first read of a receive using the gap timeout; the baud rate set again on every stop; a two-byte V1 stop address; the V1 address bound | `The time a stop can take ...`, `The stop on protocol 1.0 ...`, `A stop on an initialised driver does not set the baud rate again` |
| The stop's timeouts or settle wait not limited on an invalid configuration | `The stop on an invalid configuration never polls ...`, the fuzz regression input `regress-stop-invalid-config` |
| `SetStatusReturnLevel()` adopts a silent answer without a log line | `SetStatusReturnLevel says so in the log ...` |

The dead non-exact branch of the reply check was removed instead of tested.

The counts are for the code before the last refactoring (the longest functions were split into helpers to meet the
complexity limit); the tests are the same, the helpers carry the same statements. The harness is not part of the
repository: it changes one source line at a time in a copy of the tree and runs `dynamixel_host_tests`.

## Coverage

Measured locally with GCC 15 (`--coverage -O0 -fcondition-coverage`) over every test that runs, gcovr 8.8,
`inc/` and `src/` only, template lines merged across instances:

| Metric | Result | Limit at `high` |
| --- | --- | --- |
| Lines | 97.3 % (1164 of 1196) | 85 % |
| Branches | 94.2 % (858 of 911) | 75 % |
| Conditions (MC/DC input) | 94.6 % (808 of 854) | measured, not gated |

The figures were measured before the fixes of the independent review and have not been re-measured since.

The figure leaves out the objects that are only compiled and never run (`dynamixel_header_checks`,
`dynamixel_optimised_build`, the documentation snippets): they instantiate every template once more with a count of
zero, and gcovr counts each such instance's branches separately, which would show 50 % branches. If the CI gate
counts them, its result will differ from this one; the gate is not run locally.

## Quality gates

The `Host quality` job measures the library code, `inc/` and `src/` only (tests and examples are left out), and
fails when a value is over the limits of this repository's assurance level, `high` (set in
[`.kokeb/handbook.yml`](../.kokeb/handbook.yml), defined by the handbook's ADR-0020):

| Gate | Tool and how it is measured | Limit at `high` |
| --- | --- | --- |
| Complexity | lizard 1.24.1, modified cyclomatic complexity (`-m`: a `switch` counts once) per function | CCN at most 15 |
| Function length | lizard, lines of code per function | at most 60 |
| Parameters | lizard, per function | at most 6 |
| Line coverage | GCC 14 `--coverage -O0`, every CTest, gcovr 8.7 `--merge-lines` (a template's lines count once across its instances), `--exclude-unreachable-branches --exclude-throw-branches` | at least 85 % |
| Branch coverage | same run | at least 75 % |
| MC/DC | GCC `-fcondition-coverage`, same run | measured, not gated at `high` (100 % at `safety`) |
| Sanitizers | Clang, `-fsanitize=address,undefined -fno-sanitize-recover=all`, every CTest | no report |
| Fuzzing | libFuzzer with ASan and UBSan, 120 s per target | no broken promise |

The reports are in the run's `host-quality-reports` artifact and its step summary. To run the gates locally:

```bash
# Coverage (GCC 14 or later), then the complexity limits
cmake -S . -B build-cov -DCMAKE_CXX_COMPILER=g++-14 -DDYNAMIXEL_BUILD_TESTS=ON \
  -DCMAKE_CXX_FLAGS="--coverage -O0 -fcondition-coverage"
cmake --build build-cov && ctest --test-dir build-cov
gcovr --root . --filter inc/ --filter src/ --merge-lines --exclude-unreachable-branches --txt build-cov
lizard -m -C 15 -L 60 -a 6 inc src

# Sanitizers (Clang)
cmake -S . -B build-san -DCMAKE_CXX_COMPILER=clang++ -DDYNAMIXEL_BUILD_TESTS=ON \
  -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined -fno-sanitize-recover=all -O1 -g"
cmake --build build-san && ctest --test-dir build-san --output-on-failure
```

The fuzz gate runs locally as shown in [Fuzz tests](#fuzz-tests).

clang-tidy 20 ([`_config/.clang-tidy`](../_config/.clang-tidy): bugprone, CERT, clang-analyzer,
cppcoreguidelines, modernize, performance, portability and readability, each disabled check with its reason) and
cppcheck (strict) run in the `C++ lint` and `Repository checks` jobs and fail on any finding.

The `MISRA` job ([`ci-misra.yml`](../.github/workflows/ci-misra.yml)) runs the shared `ru-misra` workflow of
ci-tools at level `high`: it traces a host build and the Cortex-M build (`dynamixel_optimised_build`) of every
translation unit, checks them against MISRA C++:2023 and the handbook's C++20 profile (the CodeQL coding-standards
and profile packs, plus the validated clang-tidy and cppcheck checks), and reports each finding against the
handbook's compliance matrix and deviation records. A finding is fixed in the code; a deliberate exception names
its deviation on the line before it (`// codeql::misra_deviation_next_line(MD-CPP-009)` above the `.ipp` include
of `inc/dynamixel.hpp`). Until the repository is public the job runs with `open_source_prerelease: true`, the
declaration that the code is released under the licence in `LICENSE` and is to be published.

## Documentation snippets

Every C++ block on this site is included from [`tests/docs/`](../tests/docs/my_port.hpp), which the host build
compiles (target `dynamixel_doc_snippets`) with the driver instantiated over the documented port. A change to
the API that breaks a page breaks the build first. The README's example is a generated copy of the same file;
`kokeb-docs check` fails when it drifts.

## CI workflows

Everything runs from [`ci.yml`](../.github/workflows/ci.yml), which calls one workflow file per check and ends in
the `CI gate` job, the single required check (`CI gate / gate`).

| Job | What it checks |
| --- | --- |
| Host tests | GCC, then Clang (one job), build and run every test above, then build and run `examples/host` on its own |
| Host quality | The gates above: complexity, coverage, sanitizers, fuzzing |
| Cross-compile | `arm-none-eabi-g++` for Cortex-M0+, M4F and M7F with the strict flags, warnings as errors (nothing linked) |
| C++ lint | clang-format and clang-tidy over the sources |
| MISRA | MISRA C++:2023 and the C++20 profile on traced host and Cortex-M builds, at level `high` (see [Quality gates](#quality-gates)) |
| Docs | Strict site build (`_config/mkdocs.yml`), figure and generated-region drift, Doxygen (`_config/Doxyfile`), offline link check, Lighthouse |
| Repository checks | One job: DCO on pull requests, the org standard, Markdown and YAML lint, the [Kokeb Labs handbook](https://github.com/kokeb-labs/handbook) rules for `.kokeb/handbook.yml` (by rule ID), template drift (warns only) and cppcheck over the driver sources; a second job runs the handbook's source checks (include guards, bus requirements, raw access) |
| CI gate | Passes only if every job above passed: the one required check |

The **ESP32 examples** job builds every app in `examples/esp32/app_config.yml`, for each listed ESP-IDF version and
build type, with the repository root as the driver's component, and lints the example sources. The gate needs it
too.

The workflows call the shared, versioned workflows of
[kokeb-labs/ci-tools](https://github.com/kokeb-labs/ci-tools) at `@v1`.

## On hardware

Nothing here has been run against a servo. The ESP32 example ([Examples](examples.md)) is the bench test: it finds the
servos on the bus and logs the feedback of each twice a second with the torque off. On the first run, check the four
open points of the [Contract](contract.md#open-points) (a Fast Sync Read of two or more servos, a write of the ID
register, a Sync Read with one servo unplugged, the reply deadline) and record the board, the servo models, the
firmware versions and the result in the pull request's bench-status section.
