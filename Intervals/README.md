===================
TestCases Intervals
===================


------------
Introduction
------------

This folder group work on calculating Set of Interval Vector:

 - testCSIVIA.py use a SIVIA in C++ to compute the right set of boxes

 - testASIVIA.py same as fisrt one but saving all the boxes

 - test2.py use only test in C++ to compute all paving (option to not compute some boxes to gain speed)

 - sivia_comp.py clean example on how to call sivia computing function

 - fusion_sivia.py testcase fusion unchanging image with the precedente sivia

----------------------
How to use with pyIbex
----------------------

Those script use a differente version of pyIbex,
the good version is link in this repository as a submodule.

To compile this pyIbex you will need a C++11 compliant compiler
and add the flags in the CMakeList.txt (there is on the one from the correct version 
of pyIbex, but there are some personal computer self adding so it may not work with yours
so download the CMakeList from benEnsta/pyIbex then add the c11++ flags - an example will be found in my version of the CMakeList)

 
