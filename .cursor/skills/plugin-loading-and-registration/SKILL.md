---
name: plugin-loading-and-registration
description: Implements or debugs the ex3 shared-library plugin path - the registrar singleton, REGISTER_ macros, dlopen/dlclose lifetime, symbol export, and .so naming. Use when working on plugin loading, factory registration, undefined-symbol errors at dlopen, or a plugin landing in the report's errors list.
---

# Plugin Loading and Registration

The simulator discovers algorithms and mission controls by `dlopen`ing `.so` files whose static
initializers register a factory with the simulator. This is the part of assignment 3 with the most ways to
fail silently.

Contract summary: `.cursor/rules/plugin-architecture.mdc`. Header details:
`docs/api-delta-ex2-to-ex3.md`.

## How the pieces fit

```text
common/MappingAlgorithmRegistration.h   declares  struct MappingAlgorithmRegistration { explicit ctor };
                                        defines   REGISTER_MAPPING_ALGORITHM(class_name)
                            (published, must not change)

Algorithm_<ids>.so   contains a file-scope MappingAlgorithmRegistration object
                     -> its ctor runs during dlopen
                     -> the ctor symbol is UNDEFINED in the .so and resolved from the executable

simulator_<ids>      defines MappingAlgorithmRegistration::MappingAlgorithmRegistration(factory)
                     -> forwards the factory into a singleton registrar
                     -> must be linked with ENABLE_EXPORTS so the .so can see that symbol
```

The macro expands to:

```cpp
[[maybe_unused]] ::common::MappingAlgorithmRegistration register_me_##class_name{
    [](::common::MappingAlgorithmDependencies dependencies)
        -> std::unique_ptr<::common::IMappingAlgorithm> {
        return std::make_unique<class_name>(std::move(dependencies));
    }};
```

So the concrete class needs a constructor taking the dependency struct **by value**, and the macro must
appear at **global scope** in a `.cpp` (not inside a namespace block).

## Implementation steps

### 1. Registrar singleton (Simulator only)

One registrar holding the most recently registered factory of each kind, plus the association to a `.so`
filename. The plugin has no idea which file it came from, so the loader must attribute it:

```cpp
// Simulator/src — sketch
// PluginRegistrar::instance().takePendingAlgorithmFactory()  -> std::optional<MappingAlgorithmFactory>
// PluginRegistrar::instance().setPendingAlgorithmFactory(f)  <- called from the registration ctor
```

Write the registration constructor bodies in Simulator-owned `.cpp` files (one for
`MappingAlgorithmRegistration`, one for `MissionControlRegistration`). **Never** compile these into a
plugin — that would satisfy the symbol locally and break the registration handshake.

### 2. Loader

```text
for each candidate .so in the folder (or the single .so argument):
    clear the registrar's pending slot
    handle = dlopen(path, RTLD_NOW | RTLD_LOCAL)
    if !handle                                   -> record filename in report errors; log dlerror(); continue
    factory = registrar.takePendingFactory()
    if !factory                                  -> record filename in report errors; dlclose; continue
    store { filename, handle, factory }
```

- `RTLD_NOW` surfaces undefined symbols at load time instead of mid-run.
- `RTLD_LOCAL` keeps two teams' plugins from binding to each other's internal symbols.
- Load on the **main thread, before spawning workers** — registration is a global side effect, so
  concurrent `dlopen` needs serialization anyway (`.cursor/rules/threading-model.mdc`).
- A folder argument with **zero** `.so` files is a CLI validation error, not an empty run
  (`.cursor/rules/simulator-cli-and-outputs.mdc`).

### 3. Shutdown order

Getting this wrong is a use-after-`dlclose` crash, usually at process exit and usually blamed on something
else:

```text
1. join all worker threads
2. destroy every run's object graph (all IMappingAlgorithm / IMissionControl instances)
3. clear the registrar's factory map  <- the std::function targets live inside the .so
4. dlclose every handle
5. write the report / return from main
```

The assignment is explicit: `dlclose` before the program ends, and never while objects from that library
are still alive.

### 4. Instances, not libraries, are recreated

Call the factory for **every run**. Do not cache instances (assignment forbids it; they are cheap by
design). Do cache the `.so` handle — never `dlclose` and re-`dlopen` the same file.

Lazy load-once-and-unload-when-unused is a **bonus**; build the eager path first.

### 5. CMake

```cmake
# plugin
set_target_properties(<plugin> PROPERTIES
    OUTPUT_NAME "Algorithm_207190406_209543255"   # or MissionControl_...
    PREFIX "")                                    # no "lib" prefix

# simulator
set_target_properties(<sim> PROPERTIES
    OUTPUT_NAME "simulator_207190406_209543255"
    ENABLE_EXPORTS ON)                            # -Wl,--export-dynamic
target_link_libraries(<sim> PRIVATE common::common ${CMAKE_DL_LIBS})
```

## Debugging

| Symptom | Cause |
|---------|-------|
| `dlopen` fails: `undefined symbol: _ZN6common32MappingAlgorithmRegistrationC1E...` | `ENABLE_EXPORTS` missing on the simulator target, or the registration `.cpp` isn't in the simulator |
| `dlopen` succeeds but no factory registered | macro is inside a namespace instead of global scope; or the plugin's `.cpp` with the macro was optimized out of a static lib — put it directly in the `SHARED` target |
| Plugin file is `libAlgorithm_<ids>.so` | `PREFIX ""` missing |
| Two plugins loaded, second one's factory overwrites the first | registrar's pending slot not taken/cleared between `dlopen` calls |
| Crash at exit, valgrind points into freed mapping | `dlclose` before instances or factories were destroyed |
| Works alone, breaks with another team's `.so` | `RTLD_GLOBAL` instead of `RTLD_LOCAL`, or our classes aren't in the ID-suffixed namespace |

Useful checks:

```bash
nm -D --defined-only build/default/Simulator/simulator_207190406_209543255 | grep Registration
nm -DC --undefined-only build/default/Algorithm/Algorithm_207190406_209543255.so | grep Registration
ldd build/default/Algorithm/Algorithm_207190406_209543255.so
```

The plugin should list the registration constructor as **undefined**, and the executable should **define**
it. Anything else means the two ends aren't wired.
