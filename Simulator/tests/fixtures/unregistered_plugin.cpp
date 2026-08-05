// Test-only .so with no REGISTER_* call — dlopen succeeds, factory stays empty.

extern "C" int fixture_unregistered_marker() { return 1; }
