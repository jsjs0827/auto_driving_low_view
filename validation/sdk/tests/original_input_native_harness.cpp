// Link the real original scalar helper against existing host-only API fixtures.
// The fixture donor's main is deliberately not run. No model/BPU inference is
// executed by this CPU input contract; it does not prove X5 numerical parity.
#define main r26_unused_native_fixture_main
#include "runner_harness.cpp"
#undef main
#define R26_NATIVE_REFERENCE_TEST 1
#include "original_input_contract_tests.cpp"
