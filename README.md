## 2 Prerequisited

### 2.1 Ubuntu and ROS

Ubuntu 18.04~20.04, [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 2.2 PCL and Eigen

PCL >= 1.6, follow [PCL Installation](https://pointclouds.org)

Eigen >= 3.3.4, follow [Eigen Installation](https://eigen.tuxfamily.org/index.php?title=Main_Page)

fmt: 

Download fmt.zip at https://github.com/hku-mars/IPC/releases/tag/v0.1.

Then Build and install by:

```bash
mkdir build
cd build/
cmake -DBUILD_SHARED_LIBS=TRUE ..
make
sudo make install
sudo cp /usr/local/lib/libfmt.so.8 /usr/lib
```

### 2.3 OSQP 

* [osqp-github](https://github.com/osqp/osqp)
* [osqp-document](https://osqp.org/docs/get_started/sources.html)

Install OSQP (please selete the version less than [0.6.3](https://github.com/osqp/osqp/releases/tag/v0.6.3))
```
git clone --recursive https://github.com/osqp/osqp
cd osqp
mkdir build
cd build
cmake ..
sudo make install
```

<!-- * [osqp-eigen-github](https://github.com/robotology/osqp-eigen)

Install OSQP-Eigen
```
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build
cd build
cmake ..
sudo make
sudo make install
``` -->

### 2.4 Other

A debug tool: *backward.cpp*

Installation
```
sudo apt-get install libdw-dev
wget https://raw.githubusercontent.com/bombela/backward-cpp/master/backward.hpp
sudo mv backward.hpp /usr/include
```

A print visualization tool：rosfmt

Installation
```
sudo apt-get install ros-noetic-rosfmt
```


## Install
### install system dependences
```
sudo apt-get install cmake libopenblas-dev liblapack-dev libarpack-dev libarpack2-dev libsuperlu-dev
```
### install Armadillo<br>
if not decompressed yet：<br>
```
xz -d armadillo-9.870.2.tar.xz
tar -xvf armadillo-9.870.2.tar
```
Go into armadillo directory:
```
cd armadillo-9.870.2
```
if "build" directory already exits:<br>
```
rm -rf build
```
Then make:
```
mkdir build  
cd build  
cmake ..  
make  
sudo make install
```
>Debug: if cmake failed because of python version(recommend python3.8,3.9) ,<br>
      do `cmake -B . -S .. -DCMAKE_INSTALL_PREFIX=/usr/local -DPYTHON_EXECUTABLE=/usr/bin/python3 --debug-output`<br>
      change the term after "-DPYTHON_EXECUTABLE=" to your own python path<br>
     `-B` : explicitly tells CMake to put build files in the current directory<br>
     `-S`: specifies where the source files (including CMakeLists.txt) are located <br>

Then go to workspce directory
```
catkin build
```
