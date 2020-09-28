# Gerber File Viewer

For the moment this is just a gerber file viewer.

## Requirements

This project uses GLFW as graphics library. You can dowload it here: https://www.glfw.org/download.html. CMake is also used to build the project (you can find it here: https://cmake.org/download/). If you are using a Debian distribution you can use the following command to install it:
```bash
sudo apt install libglfw3-dev cmake
```

## Build and run

This is how to build the project:
```bash
mkdir build
cd build
cmake ..
make -j4
```
The executable takes as many inputs as you want. Each input has to be a path to a gerber file. Here is an example:
```bash
./gerber ../gb_files/DIY_detector-PTH.drl
```