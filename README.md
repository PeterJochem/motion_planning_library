# MotionPlanning

## Road Map

1) Model robot as a 
	a) list of cylinders or 
	b) list of triangles from a mesh file
		1) FCL has support for loading mesh files	

2) Read in an STL file into a 
list of triangles

3) Convert the list of triangles to 
a collision object in FCL

4) Use FCL to check for 
	a) self collision of the robot
	b) robot collision with the environment







Mesh files for UR5 are found [here](https://github.com/ros-industrial/universal_robot/blob/kinetic-devel/ur_description/urdf/ur5.urdf.xacro#L101) as STL files


You can have FCL load obj files as shown [here](https://github.com/flexible-collision-library/fcl/blob/7980061604ca8df4872a1c5139074b780d82fe08/test/test_fcl_utility.h#L194)


You can convert STL files to OBJ files [here](https://all3dp.com/2/how-to-convert-stl-files-to-obj/)




