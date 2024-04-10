# plotjuggler-apbin-plugins

This repository contains [ArduPilot Dataflash](https://ardupilot.org/copter/docs/common-logs.html) plugin for [PlotJuggler](https://github.com/facontidavide/PlotJuggler).


## Install PlotJuggler

To build any plugin for PlotJuggler, PlotJuggler must be installed on your system.
For detailled instructions on how to install PlotJuggler please have a look at the [Installation](https://github.com/facontidavide/PlotJuggler#installation) section of the PlotJuggler repository.

If you have ROS installed, you can install PlotJuggler using:

    sudo apt install ros-$ROS_DISTRO-plotjuggler-ros


## Install plotjuggler-apbin-plugin
The installation of the plotjuggler-apbin-plugin is straightforward.

1. Clone the repository:

    ```
    git clone https://github.com/khancyr/plotjuggler-apbin-plugins
    ```
    
2. Install the dependencies:  
    The plugin uses Qt as dependency. Since PlotJuggler also depends on Qt, chances are high that you already have it installed. On Ubuntu Qt can be installed with:

    ```
    sudo apt -y install qtbase5-dev libqt5svg5-dev
    ```

3. Compile using cmake:

    ```
    mkdir build; cd build
    cmake ..
    make
    sudo make install
    ```

If you run `sudo make install` the plugin gets installed to `/usr/local/bin/`.  
Remember that PlotJuggler needs to find the plugin files at startup. Therefore make sure that the folder containing the plugin files is added to PlotJuggler in the settings.
Check **App->Preferences->Plugins** in PlotJuggler to learn more.


## Configuration
If you want the units to be displayed in PlotJuggler, you need to edit `dataload_apbin.cpp` and activate `#define LABEL_WITH_UNIT`.  
**Be carefull:**  
This is not a nice solution because the units are simply appended to the field names.
This renders your already created visualization layouts unusable. It would be a better solution if PlotJuggler supported unit handling directly.
