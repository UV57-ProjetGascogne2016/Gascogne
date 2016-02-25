===================
TestCases Intervals
===================


------------
Introduction
------------

This folder group work on calculating Set of Interval Vector:

 - testCSIVIA.py use a SIVIA in C++ to compute the right set of boxes

 - testASIVIA.py same as fisrt one but saving all the boxes

 - SimuSecureZone_live.py test with controled robot around an ellipse

 - Simulation_Server.py server that send positions of robots upon request (robots controled as in SimuSecureZone__live.py)

 - Simulation_Client.py client which ask for position then proceed to compute the SecureZone (using Client.py for connexion to the server)

 - Client_real.py client for test in ENSTA Bretagne (expecting UTM coords)

----------------------
How to use with pyIbex
----------------------

Those script use a differente version of pyIbex,
the good version is link in this repository as a submodule.

To compile this pyIbex you will need a C++11 compliant compiler
and add the flags in the CMakeList.txt (there is on the one from the correct version 
of pyIbex, but there are some personal computer self adding so it may not work with yours
so download the CMakeList from benEnsta/pyIbex then add the c11++ flags - an example will be found in my version of the CMakeList)


Available Functions in python in relation to Gascogne Project:

 - testR(IntervalVector\[2\] box to test,\[\[Interval,Interval\]\] watch robot positions,range2,is_efficient)
 - class clSIVIA(image as list of list,size of erode Kernel (2*i+1),type of erosion (0,1,2) as (rectangle,cross,ellipse),iteration of erode)

		setRecord(name,fps,do_draw_rectangle) recording of video
		setErode(do_erode) (procede to erosion after SIVIA or not)
		setScreening(do_draw_rectangle) showing of result
		stopScreening()
                limSIVIA (proceed to a SIVIA without returning boxes)
                imSIVIA (proceed to a SIVIA with returning boxes) this function can cuse memory leak because of failure or the pytho garbage collector
 
