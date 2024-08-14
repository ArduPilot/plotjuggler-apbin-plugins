# plotjuggler-apbin-plugins

This repository contains the [ArduPilot Dataflash](https://ardupilot.org/copter/docs/common-logs.html) plugin for [PlotJuggler](https://github.com/facontidavide/PlotJuggler).

:construction: While you should be able to compile the plugin for Windows, this guide is tested for an Ubuntu workflow.

## Install PlotJuggler

To build any plugin for PlotJuggler, PlotJuggler must be installed on your system.
For detailed instructions on how to install PlotJuggler please have a look at the [Installation](https://github.com/facontidavide/PlotJuggler#installation) section of the PlotJuggler repository.

If you have ROS installed, you can install PlotJuggler using:

    sudo apt install ros-$ROS_DISTRO-plotjuggler-ros

## (Optional) Build plotjuggler-apbin-plugin

If you wish to build `plotjuggler-apbin-plugin` follow these steps.

### 1. Clone the repository

```
git clone https://github.com/ArduPilot/plotjuggler-apbin-plugins
```
    
### 2a. Compile via Docker

This requires that you have Docker installed in your system.

1. `cd` into the cloned repository.

2. Prepare a new folder with
    ```bash
    mkdir artifacts
    ```

3. Build the plugin with
    ```bash
    docker build -o type=local,dest=artifacts .
    ```
    This command will clone and build PlotJuggler anew, which takes a lot of time.

    You can pass build arguments with the `--build-arg` option.
    Build arguments include:
    - `ADD_UNITS[=OFF]`: Set to `ON` to enable the display of units in the logged fields. Read at the end for more information.
    - `BASE_IMAGE[=ubuntu:22.04]`: Specify the OS image to build off of. It is known that using a different OS than your host OS may result in the plugin not working.
    - `PJ_TAG[=3.9.2]`: The PlotJuggler git branch or tag to use when cloing and compiling PlotJuggler.

Once compilation is finished, you will find your `.so` plugin in the `artifacts` folder you created previously.

### 2b. Compile locally

1. Install the dependencies:  
    The plugin uses Qt as dependency. Since PlotJuggler also depends on Qt, chances are high that you already have it installed. On Ubuntu Qt can be installed with:

    ```
    sudo apt -y install qtbase5-dev libqt5svg5-dev
    ```

2. If you want the units to be displayed in PlotJuggler (read at the end), you need to edit `dataload_apbin.cpp` and activate `#define LABEL_WITH_UNIT`.  

3. Compile using cmake:

    ```
    mkdir build; cd build
    cmake ..
    make
    sudo make install
    ```

## Install plotjuggler-apbin-plugin

PlotJuggler looks for plugins in specific folders. 
Check **App->Preferences->Plugins** in PlotJuggler to find out which they are and add more if you wish to.

If you have added a folder, you will need to restart PlotJuggler for the chage to take effect.

### For prebuilt binaries

You can find a pre-built plugin for Ubuntu in the [Github Releases](https://github.com/ArduPilot/plotjuggler-apbin-plugins/releases) page.
Copy that binary to one of the PlotJuggler plugin folders.

### For Docker-built binaries

If you used Docker to compile the plugin, copy the build artifact from the `artifacts` folder to one of the PlotJuggler plugin folders.

### For locally-built binaries

If you compiled the plugin in your host system, the plugin has been installed to `/usr/local/bin/`.  

Ensure that PlotJuggler scans for plugins in this folder or copy the plugin in one of the folders PlotJuggler already scans.

## Displaying units

This plugin allows the units of logged fields to be appended to the logged field names.

**Be carefull:**  

If you created a PlotJuggler layout without units and then enable the units, the layout will be unusable and vice-versa.
This is because the units are part of the field name; hence, the original field name no longer exists.