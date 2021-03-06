# ECE 470 Object Sorting
ECE 470 Final Project: Object Sorting based on Size

We will be implementing a simple object sorting based on the size of objects. Limitation: The objects are expected to be regular in shape.

Robot: UR3
Simulator: V-REP

Team Members: Lohitaksh Gupta, Kehan Long, Miguel Jiménez Aparicio
Team Name: Jarvis (Inspired from Iron Man)

Checkpoint 1: We used Jupiter and the intial code provided by the professor. We modified the intial code to move all the six joints. We basically copied the code for the handlers and the snippet which set the angles for each one of the joints.


# Installing and Using V-REP with the python remote API

The steps I took to install and use V-REP with the python remote API
Folks, here is what I did to install and use V-REP with the python remote API.

Download V-REP PRO EDU from the Coppelia Robotics website:
http://www.coppeliarobotics.com

I suggest using version 3.4.0 instead of the most recent version:
http://coppeliarobotics.com/files/V-REP_PRO_EDU_V3_4_0_Mac.zip

I renamed this folder vrep because I am efficient and want to type fewer characters.

I suggest running V-REP from the command line (e.g., in a terminal). Inside vrep, type:


./vrep.app/Contents/MacOS/vrep
You should see the GUI appear. In the terminal, you should also see these lines (among others):


Plugin 'RemoteApi': loading...
Starting a remote API server on port 19997
Plugin 'RemoteApi': load succeeded.
Note the port number. Your code will communicate with V-REP over a socket with this port number.

Drag a robot into the GUI. For example, I might want to work with a UR3:



 

Robots often come with demo "scripts" that execute when you start a simulation. You'll likely want to remove these scripts. Right-click on the robot you just inserted and remove the "Associated Child Script", as shown above.

 

At this point, you'll likely want to "File -> Save Scene As..." so that you don't have to re-insert the robot or remove the script again in future. I called my scene my_ur3.ttt, for example. The next time you open V-REP, you can just load this scene.

 

To write python code, first install python. I recommend conda (Products -> Download -> Python 3.6 version):

https://www.anaconda.com

 

Conda allows you to create "environments" for package management. Here is the environment I use right now for ece470:

ece470.yml

 

If you like, you can download and import this environment for your own use like so:

https://conda.io/docs/user-guide/tasks/manage-environments.html#creating-an-environment-from-an-environment-yml-file

 

Once you have installed python (and, optionally, activated an ece470-specific environment), create a new folder outside of vrep (e.g., vrep_code). Copy these things into the folder (in my case, into vrep_code):

 

vrep/programming/remoteApiBindings/python/python/vrep.py
vrep/programming/remoteApiBindings/python/python/vrepConst.py
vrep/programming/remoteApiBindings/lib/lib/remoteApi.dylib
 

Download and put this file (based on vrep/programming/remoteApiBindings/python/python/simpleTest.py) in the folder as well:

test.py

 

Run this code in a terminal with this command (inside the folder vrep_code):

 

python test.py
 

In the GUI, you should see your robot move. In the terminal, you should see:

 

current value of first joint variable: theta = 0.000000
current value of first joint variable: theta = 1.570797
 

A lot more information can be found in the documentation (e.g., look at "Writing code in and around V-REP"):

http://www.coppeliarobotics.com/helpFiles/index.html

 

In particular, here are the commands available to you in the python remote API:

http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm

# Demonstrating Forward Kinematics: Checkpoint 2

1) Drew a schematic of the robot. We used the one in lab manual.

2) Derived the forward kinematics of your robot, from the schematic.

3) Wrote code that implements the forward kinematics (i.e., a function that takes joint variables as input and returns the pose of the tool frame as output).

4) Wrote code that (1) moves the robot in the simulator to a given set of joint variables, and (2) drew a frame in the simulator at the pose that is predicted by our implementation of the forward kinematics.

# Demonstrating Inverse Kinematics: Checkpoint 3

1) Wrote a code that implements numerical inverse kinematics (i.e., a function that takes a goal pose as input and returns either a set of joint variables that achieve the goal pose or an indication - e.g., "None" in python or "[]" in matlab - that the goal pose is not reachable).

2) Wrote a code that (1) generates a goal pose either at random or in response to some kind of user input, (2) draws a frame in the simulator at the goal pose, (3) either moves the robot in the simulator to a set of joint variables that achieves the goal pose or indicates in some way that the goal pose is not reachable.

3) Showed the robot achieving several different goal poses - selected either at random or in response to user input - and highlighting agreement between the goal pose and the actual pose of the tool frame in each case. Your video should also show the result of asking the robot to achieve at least one goal pose that is not reachable.

# Demonstrating Collision Detection: Checkpoint 4

1) Wrote code that decides if a given set of joint variables (i.e., a configuration) places the robot in collision, either with itself or with something else in the environment. Note that one type of "self-collision" is violating joint limits (i.e., bounds on the value of a joint variable).

2) Showed the robot in many different configurations and that indicates, in some way, which of these configurations place the robot in collision. Your video must show at least a few configurations each that result in no collision, in self-collision, and in collision with other things.

# Demonstrating Motion Planning: Checkpoint 5

1) Write code that either returns a collision-free path between given start and goal configurations (i.e., a sequence of straight-line segments that is described by a list of nodes q1,…,qn where q1=θstart and qn=θgoal) or that returns failure if such a path could not be found. Your code must consider both self-collision and collision with obstacles.

2) Showed the robot moving along at least 3 different collision-free paths. Each path must be non-trivial, in the sense that it (1) consists of more than one straight-line segment, and (2) could not have been replaced by a path consisting of only one straight-line segment from θstart to θgoal. In other words, your video should demonstrate that your path planner is really working, that it really does allow your robot to avoid both self-collision and obstacles.

# Final Project: Jarvis - Object Handling for Industrial and Household Applications

We are going to use V-REP simulator to show how our robotic arm, UR-3, will suck a cube and a glass full of balls from their initial pose in a table. The first one will be reallocated in a conveyor belt and the balls will be dropped in a basin. Concepts such as Forward Kinematics, Inverse Kinematics and Motion Planning will be used in order to implement the sorting of objects. 

“Created a dynamic simulation (i.e., a simulation of real physics) in which at least one robot - with at least one arm that has at least six joints - moves at least one object (e.g., by pushing or grasping) from a random initial pose to a given final pose.”
	Going beyond that, we decided to attach a suction gripper to our robot and pick two objects - a cube and a cup full of balls - from their initial position and, in the case of the cube, release it in a conveyer belt. For the glass, the robot takes it and moves until it is located above the basin. Then, it rotates the last joint so all the balls fall from the glass. After that, the robot leaves the empty glass in its initial place. 

The final goal of our project is to show the capability of the robot to perform tasks both in industrial and household taks, such as reallocation and manipulation of objects and automated search of a path without collision with itself and the environment.

For this project, we have used the following items: 
 Robot: UR-3
 Simulator: V-REP
 Programming Language: Python
 
 # Final Project Report
 Link: https://github.com/lohitakshgupta/ece470_object-sorting/blob/master/Final%20Project%20Report.pdf
 
 We followed IEEE standards for making the Final Project Report
