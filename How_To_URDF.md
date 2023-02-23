## How to make a URDF from CAD?
This document will be a brief summary of how I created the URDF for the avatar gripper from CAD designs (and some tricks and lessons learned).

For a more comprehensive tutorial on URDF, here is a very good reference: http://wiki.ros.org/urdf/Tutorials.

Given a CAD file of a robot/part, what information do we need from it to create the URDF? 
1. The mesh file for visualization and possibly for collision checking too,
2. the relative positions and orientations between consecutive links. 

Since URDF describes a robot/part in a kinematic tree strucutre (unlike SDF). Every robot/part starts with a base **link**
as the root and grows into a kinematic tree. Every two consecutive links are connected by a **joint**. The two main joint
types supported by URDF are prismatic joint and revolute joint. So it's helpful to start by expressing the robot/part as 
a kinematic tree and identify what are the links and joints.

### Link
For a link element, three major tags need to be defined: inertial, visual and collision. 
Full explaination [here](http://wiki.ros.org/urdf/XML/link). 

The **inertial** values can be calculated with Solidwork or use standard type such as cylinder or sphere as an approximation. 

The **visual** origin defines the transformation between the visual frame and the link frame. Ususally it's all 0. If you 
are importing the link from a CAD model, **the reference frame attached to the link mesh file is the coordinate frame defined
in Solidwork when exporting the mesh file (obj, stl or dae)**. I would recommend creating a coordinate frame at each joint 
location in Solidwork and exporting each link with respect to the corresponding coordinate frame so that it's much easier 
to define joint tags in URDF later.

An example flow to generate a STL mesh file for a body:
1. Add a Coordinate System under Feature->Reference Geometry at the center of joint
1. select the body you want to export as the mesh file
2. click file->save as->select stl format-> options
3. In the pop-up window, change the Output coordinate system at bottom to the coordinate frame you just created.

As for **collision**, the most straightforward way is to use the visual mesh file directly, however it is not recommended to do so as the number of triangles are oftentimes too many. A raw exported stl mesh file could contain 30000 triangles while it's normally recommended to have less 1000 triangles for each link. There are two ways here to reduce the number of triangles. A common tool to use is under `Filters -> Remeshing, Simplification and Reconstruction -> Simplification: Quadric Edge Collapse Decimation`. This would half the number of triangles. You can run it a few times until you hit the balance between simplicity and detailedness. 

The first is to create a simpler CAD model in the begining, with only necessary geometry information (no holes, no curves, etc). The second is to use [MeshLab](https://www.meshlab.net/) to remesh the file and reduce the number of triangles. 


### Joint
For a joint element, the most important tags are origin, parent link, child link and type. Full explaination [here](http://wiki.ros.org/urdf/XML/joint).
Parent link, child link and type are easy to figure out. Origin could be a bit trickier if the transformation between two links 
involves rotation too. If it only contains translation, we can use measure tool in Solidwork to easily get it. If it also contains 
orientation, we need to obtain the transformation matrix between two frames and calculate the roll, pitch and yaw value.
In Solidwork there is no existing tool to obtain the transformation matrix between two frames, so someone wrote a macro 
[script](https://www.codestack.net/solidworks-api/geometry/transformation/get-coordinate-system-transform/) in VBA to do that. It's not convenient but it works. 

You also need to set the joint limit but that's straightforward. I used `<mimic>` tag to approximate the linear relationship between joints, more information could also found on the page I linked above.

### Use Xacro to generate URDF 
Above are the basics concepts that are important to understand. Hopefully you haven't directly jumped into writing a URDF from scratch already, as nobody would actually do that directly. It is recommended to write in [xacro](http://wiki.ros.org/xacro) instead. It is easier to maintain and also more convenient to generate multiple URDFs for parts with only slight differences. 

You can create your own macro using `<xacro:macro name="xxxx">` and use them anywhere in your xacro file by calling `<xacro:xxxx>`. There are a lot to cover to cover for xacro but I will skip here as I'm no expert and I mostly only use macro. Feel free to click the link above to learn more!
