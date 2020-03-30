# scat_libs

## Brief description

This package contains a number of handy tools, used throughout the scatwheel_drivers repository. 
It contains the several source files described below. 
More detailed documentation for all methods is provided in doxygen format, see the docs folder or source code. 
If you want to use some feature from these libraries, do the following:
- add dependency to package.xml: <depend>scat_libs</depend>
- add dependency scat_libs to find_package and caktin_package in CMakeLists


## base_utils
Provides a number of basic essential functions, such as distance and angle calculations.

## geom_utils
Provides a number of basic or commonly used geometric calculations, such as 
- transformations between coordinate frames
- triangle center calculations for triangulation
- line intersection, e.g. for obstacle based path planning
- footprint methods, for collision checking

## rosmsg
Provides convenient methods to create ros messages in a single line of code. 

## obst_dist
Provides a single object which calculates the distance to closest obstacle based on K nearest neighbors. The object can be updated with scan data, map data, or both. 



