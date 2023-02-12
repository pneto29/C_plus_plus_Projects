# RND_Sample #

This repository contains an implementation of the Random DOwnsampling. 

## Dependencies ##

The dependencies are header-only and are all included in the ext directory. As a consequence, there is nothing to do.

* PCL (Point cloud library) - 1.7 version
* Linux Ubuntu 16.04 or 18.04
* Qt Creator 4.5.2

## Usage ##

* Create a folder named build (for example)
mkdir build
* Enter folder
cd build/
* Run "CmakeLists.txt" which is a path before build
cmake ..
* Run make
make
* When the program has been built, run: 
./random_fixed argv[1] argv[2] argv[3]

## Parameters ##
* argv[1]: source cloud
* argv[2]: number of output points ( if you decide for the variant with 50% of the points, pass as a parameter that of the points in half ... if you choose 70%, pass the value referring to total -0.7 * num_points)
* argv[3]: output to rnd_sampled_cloud

## Reference ##

Vitter, Jeffrey Scott. "Faster methods for random sampling." Communications of the ACM 27.7 (1984): 703-718.
