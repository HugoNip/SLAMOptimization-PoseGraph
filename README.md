## Introduction
This project shows backend optimization in SLAM. In details, it shows how to use g2o and graph optimization to optimize pose graph.

## Requirements

### Eigen Package (Version >= 3.0.0)
#### Source
http://eigen.tuxfamily.org/index.php?title=Main_Page

#### Compile and Install
```
cd [path-to-Eigen]
mkdir build
cd build
cmake ..
make 
sudo make install 
```

#### Search Installing Location
```
sudo updatedb
locate eigen3
```

default location "/usr/include/eigen3"

### Sophus Package
#### Download
https://github.com/HugoNip/Sophus

#### Compile and Install
```
cd [path-to-pangolin]
mkdir build
cd build
cmake ..
make 
sudo make install 
```


## Compile this Project
```
mkdir build
cd build
cmake ..
make 
```

## Run
```
./build/pose_graph_g2o_lie_algbra
./build/pose_graph_g2o_SE3
```



## Reference
[Source](https://github.com/HugoNip/slambook2/tree/master/ch10)
