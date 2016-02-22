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

This code returns the pavage of the secure zone. It contains the fusion of the different codes for the processing of the secure zone image. A SIVIA test for the Gascogne Area, a SIVIA test for the robots secure area with incertitude and a SIVIA test for the trail of the robots.
It uses a SIVIA algorithm defined in SIVIA.py.
The frame origine, the scale and the precision of the pavage are defined by the variables (i0, j0), pixelScale and epsilon.

The log file contains the robots positions along the time.
For each robot we have time, longitude and latitude values.

    

