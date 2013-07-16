keerthi-masters

===============

Milestones:

1) 3D map of "room"/surrounding area - using Octomap or similar. This provides a 3D volumetric representation of the space

Status: Done

Approach/Steps followed:

-> 3d environment and navigation using Morse simulator.

-> Pcl data is received from the morse simualtor.

-> Octomap is used to construct the 3d Map.

Note: A default sample kitchen environment present in Morse simulator is used currently.

2) Find supporting planes in the 3D map. Plane fitting using PCL or you hand code which areas of the map we will consider. Outcome of this will be a list of voxels in the map.

Status: Done

Approach/Steps followed:
-> Use of RANSAC and Perpendicular plane model for segmentation.

-> The segmented pcl is fed into the octomap server to generate a 3D map having only horizontal planar surfaces.

-> May need further refining of parameters based on the need.

3) Distribute probability over the identified voxels.

Status: Work in progress

Approach/Steps followed:

-> Initially consider uniform probability and visualise in rviz.

-> Add probabilities for different regions. (initially hard code the probabilities)

4) Plan to take views which maximise probability of seeing object. (follow Alper's approach)
Not started


Note: Refer the tutorials for detailed instructions.
