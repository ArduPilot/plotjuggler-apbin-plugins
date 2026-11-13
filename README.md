# plotjuggler-apbin-plugins

ArduPilot [DataFlash binary log](https://ardupilot.org/copter/docs/common-logs.html)
loader for [PlotJuggler 4](https://github.com/PlotJuggler/PlotJuggler).
Plugin version **0.2.0** targets PlotJuggler **4.1.0** and its **plotjuggler_sdk 0.40.0**.
It uses the SDK's C ABI data-source interface, C++20, and Qt 6 Core.
PlotJuggler 3 / Qt 5 binaries are incompatible with this version.

## Get the plugin and PlotJuggler

Clone this repository to build the plugin:

```sh
git clone https://github.com/ArduPilot/plotjuggler-apbin-plugins
cd plotjuggler-apbin-plugins
```

Prebuilt Ubuntu and Windows plugins are available on the
[GitHub Releases](https://github.com/ArduPilot/plotjuggler-apbin-plugins/releases)
page. Choose a release built for PlotJuggler 4; older PlotJuggler 3 plugins
cannot be used with the current application.

To run the plugin, install a compatible PlotJuggler application using its
[installation instructions](https://github.com/PlotJuggler/PlotJuggler#installation).
The application is not required to compile the plugin.

## Build

Install a C++20 compiler, CMake 3.22+, Python 3, Conan 2, and Qt 6 development
files. Use a Qt 6 kit compatible with the application: the target PlotJuggler checkout
pins Qt 6.12.0. On Windows use the same architecture and MSVC runtime as the app.
The SDK is Qt independent; the plugin only uses Qt Core and creates no widgets.
You do not need to build or install the PlotJuggler application to build this plugin.

On Ubuntu, install `build-essential cmake qt6-base-dev python3 python3-venv`.
Then, from this repository:

```sh
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install 'conan>=2,<3'
conan profile detect  # once, if no default profile exists
conan remote add plotjuggler https://plotjuggler.jfrog.io/artifactory/api/conan/plotjuggler-conan
conan install . -of build --build=missing -s build_type=Release -s compiler.cppstd=20
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
cmake --install build --config Release --prefix plugin_install
```

Pass `-DCMAKE_PREFIX_PATH=<Qt6-kit>` if Qt is not found. On Windows run in a
Visual Studio developer shell, create the environment with `python -m venv .venv`,
and activate it with `.venv\Scripts\Activate.ps1` in PowerShell or
`.venv\Scripts\activate.bat` in cmd. Continue from the pip installation step,
using `python` instead of `python3`. The virtual environment also works on Ubuntu
24.04 and other distributions that prevent pip from changing system Python.
An existing installed SDK can also be used directly with
`-Dplotjuggler_sdk_DIR=<sdk>/lib/cmake/plotjuggler_sdk`, without Conan.

### Build against SDK sources

`-DPJ_SDK_SOURCE_DIR=<plotjuggler_sdk-0.40.0-checkout>` builds the SDK in-tree.
Its CMake dependencies (`fmt`, `FastFloat`, `nlohmann_json`) must be installed
or supplied through `CMAKE_PREFIX_PATH`. The SDK's own tests and ported plugins
are disabled; this plugin's tests remain enabled. This is useful when validating
against the SDK release pinned in PlotJuggler's `conanfile.txt`.

### Build in a ROS workspace

ROS 1 catkin and ROS 2 ament integration are retained. Source your ROS environment
and place the plugin repository under the workspace's `src/` directory. Install
Qt 6 and SDK 0.40.0 first; the plugin no longer links against the PlotJuggler
application. For an installed SDK, build from the workspace root.

Use the system Python that provides your ROS modules. The commands below use
`/usr/bin/python3` on Ubuntu and explicitly select it for CMake, including when
Conan's virtual environment is still active.
Build with:

```sh
# ROS 1, using catkin_tools
catkin build plotjuggler_apbin_plugins --cmake-args \
  -Dplotjuggler_sdk_DIR=/absolute/path/to/sdk/lib/cmake/plotjuggler_sdk \
  -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPython3_EXECUTABLE=/usr/bin/python3

# ROS 2
colcon build --packages-select plotjuggler_apbin_plugins --cmake-args \
  -Dplotjuggler_sdk_DIR=/absolute/path/to/sdk/lib/cmake/plotjuggler_sdk \
  -DPython3_EXECUTABLE=/usr/bin/python3
```

Alternatively, run the Conan setup above through `conan install` in the plugin
repository, then pass the absolute path to `build/conan_toolchain.cmake` with
`-DCMAKE_TOOLCHAIN_FILE=...`
in the workspace command's `--cmake-args`, replacing `-Dplotjuggler_sdk_DIR=...`
and retaining the Python options above. This keeps ROS's `catkin_pkg` and
`ament_package` modules visible; Conan's isolated Python does not include them.
You can also run `deactivate` before the workspace build. If a package was
already configured with the virtual environment's Python, retain the explicit
Python options to replace the cached interpreter paths.

Add `-DCMAKE_PREFIX_PATH=<Qt6-kit>` if needed. The runtime dependency requires
PlotJuggler 4.1.0 or newer; a ROS
distribution's PlotJuggler 3 package is incompatible.

Both ROS install trees place the plugin and its manifest under
`lib/plotjuggler_apbin_plugins`. Add that directory to PlotJuggler's plugin folders
as described below. The catkin devel tree places the shared library under `lib`.

### Notes for building on macOS

Qt dependencies may not be found automatically. With Homebrew Qt 6, pass
`-DCMAKE_PREFIX_PATH="$(brew --prefix qt)"` to CMake; also include the installed
SDK prefix if you are building without Conan. The current official PlotJuggler
app bundle permits third-party plugin libraries, so the old workaround that
re-signed the application is no longer needed.

### Optional labels

Both options default to `OFF`:

* `-DADD_UNITS=ON` appends units to field names.
* `-DADD_RCOU_FUNCTION_LABELS=ON` appends servo functions to RCOU/RCO2 channel names.

Build and run CTest for each configuration you use. **Be careful:** changing
either option changes series names. A saved layout created without units or servo
labels cannot resolve those series after enabling the corresponding option, and
vice versa. Use the same label configuration when opening a saved layout.

### Docker

```sh
mkdir -p artifacts
docker build --output type=local,dest=artifacts .
```

Build arguments: `BASE_IMAGE` (default `ubuntu:22.04`), `ADD_UNITS`, and
`ADD_RCOU_FUNCTION_LABELS` (both default `OFF`). The image builds the SDK through
Conan and tests the plugin; it does not compile the PlotJuggler GUI.
Pass an argument with `--build-arg`, for example `--build-arg ADD_UNITS=ON`.
The exported `libDataAPBin.so` and `DataAPBin.pjmanifest.json` are placed in
`artifacts/`; copy them to a PlotJuggler plugin folder.

Choose a base image whose libstdc++ and glibc requirements are supported by the
host running PlotJuggler. A plugin built on a newer distribution may fail to
load on an older one. The default Ubuntu 22.04 base preserves compatibility with
22.04 and newer hosts; `tests/test_plugin.sh` reports the binary's ABI floor.

## Install and load logs

The standalone install tree contains `lib/libDataAPBin.so` on Linux,
`lib/DataAPBin.dll` on Windows, or `lib/libDataAPBin.dylib` on macOS, plus the
SDK-generated `DataAPBin.pjmanifest.json` sidecar in the same directory.
Check **App -> Preferences -> Plugins** to find the default plugin folders or add
a custom folder. Copy the binary and sidecar there, then restart PlotJuggler so
it scans the folder. This also applies when adding a folder in Preferences.

You can instead launch the PlotJuggler executable with `--plugin-dir <folder>`.
For a source build, use the launch scripts **from the PlotJuggler repository**:

```sh
/absolute/path/to/PlotJuggler/run.sh --plugin-dir /absolute/path/to/plugin_install/lib
```

On Windows use `C:\path\to\PlotJuggler\run.bat --plugin-dir <folder>`.
Open an ArduPilot `.bin` or `.BIN`
file with the **ArduPilot Bin** data source. The host passes its path through the
SDK's `filepath` configuration property.

The loader preserves message/instance/field paths, multipliers, GPS time
synchronization, parameter defaults, and message text. Timestamps passed to the
SDK are nanoseconds. When GPS time is unavailable, TimeUS uses seconds since boot,
including older logs without FMTU. Every import resets the decoding state.

The message timeline is now `/2. Message Timeline/index`, since PlotJuggler 4
represents scalar series as a topic plus field. The SDK's scalar write API has
no per-field tooltip setter; descriptions are retained in dataset metadata under
`field_descriptions`, available in the metadata inspector.

Copter and Plane descriptions are bundled for offline use from compressed JSON
snapshots in `DataLoadAPBin/LogMessageDescriptions`. The Copter snapshot can be
refreshed from [ArduPilot's published metadata](https://firmware.ardupilot.org/Copter/latest/__METADATA__/LogMessages.json.xz);
the Plane snapshot comes from the corresponding Plane firmware metadata. Save
them as `Copter_LogMessages.json.xz` and `Plane_LogMessages.json.xz`. Their
`firmware.git_sha` identifies the snapshot's source revision. Rebuild the plugin
after updating either file. Small legacy JSON files retain descriptions and enum
entries for older logs that are absent from the newer snapshots; the loader uses
the descriptions, while the enum entries are kept for reference. When a log
contains an ArduCopter or ArduPlane firmware banner, the matching JSON takes
precedence; otherwise the snapshots are merged, with Plane taking precedence for
shared fields. Legacy
descriptions and the other vehicle's metadata fill missing fields. These snapshots
may differ from the log's firmware version.

## Test a downloaded or locally built binary

The headless helper loads the binary through the same SDK library loader as
PlotJuggler 4 and binds recording write/runtime services. It verifies the real
log in `tests/logs`, repeated imports, parameter/message-only logs, old and new
MSG layouts, timestamp units, configuration roundtrips, cancellation, and
file/host failures.

```sh
build/tests/pj_plugin_smoketest build/libDataAPBin.so tests/logs/00000001.BIN --verify
```

Use `--expect-units` and/or `--expect-rcou` together with `--verify` for enabled
label options. On Windows the DLL is under `build/Release` and the helper is
under `build/tests/Release` when using a multi-configuration generator.

To build the helper independently, for example to test a downloaded plugin:

```sh
cmake -S tests -B tests/build -DCMAKE_PREFIX_PATH=<installed-sdk-and-Qt6> -DCMAKE_BUILD_TYPE=Release
cmake --build tests/build --config Release --parallel
```

With a multi-configuration Windows generator, this helper is under
`tests/build/Release`.

On Linux, `tests/test_plugin.sh <plugin> [log]` uses this independent helper and
also reports the libstdc++ ABI floor. To use the helper from the main build,
select its path explicitly:

```sh
SMOKETEST=build/tests/pj_plugin_smoketest tests/test_plugin.sh build/libDataAPBin.so --verify
```
