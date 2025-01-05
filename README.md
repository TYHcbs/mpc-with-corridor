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

