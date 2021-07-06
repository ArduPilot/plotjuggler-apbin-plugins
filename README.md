# plotjuggler-apbin-plugins

This repository contains [ArduPilot Dataflash](https://ardupilot.org/copter/docs/common-logs.html) plugin for Plotjuggler.


## Build

To build any plugin, PlotJuggler must be installed in your system.

For instance, in Linux, you should perform a full compilation and installation:

```
git clone --recurse-submodules https://github.com/facontidavide/PlotJuggler.git
cd PlotJuggler
mkdir build; cd build
cmake ..
make -j
sudo make install
```

Look at the [CMakeLists.txt](CMakeLists.txt) file to learn how to
find **Qt** and PlotJuggler.

## Plugin installation

Remember that PlotJugglers need to find the plugin files at startup.

The best way to do that is to install/copy the plugin in the same folder
where the executable `plotjuggler`is located.

Alternatively, there is a number of additional folders which will be
used to load plugins. Check **App->Preferences...** in PlotJuggler to learn more.

## Note for ROS users

The provide **CMakeLists.txt** should find the necessary dependencies even
when compiled with `catkin` or `ament`.

Anyway, remember that the primary goal of this repo is **not** to
support the development of ROS specific plugins.

You can find those in the repository
[PlotJuggler/plotjuggler-ros-plugins](https://github.com/PlotJuggler/plotjuggler-ros-plugins)
