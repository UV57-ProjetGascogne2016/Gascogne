===================
Secure Zone Image Processing
===================


------------
Requirements
------------

 - OpenCV 3 library.
 - PYIBEX library. Available: https://github.com/benEnsta/pyIbex
 - VIBES. Available: http://enstabretagnerobotics.github.io/VIBES/

------------
Description
------------

This code returns the pavage of the secure zone.
The log file contains the robots positions along the time.
For each robot we have time, longitude and latitude values.
It uses a SIVIA algorithm defined in SIVIA.py.
The frame origine, the scale and the precision of the pavage are defined by the variables (i0, j0), pixelScale and epsilon.
    

