# ece470_object-sorting
ECE 470 Final Project: Object Sorting based on Size

We will be implementing a simple object sorting based on the size of objects. Limitation: The objects are expected to be regular in shape.

Robot: UR3
Simulator: V-REP

Team Members: Lohitaksh Gupta, Kehan Long, Miguel Jiménez Aparicio
Team Name: Jarvis (Inspired from Iron Man)

Checkpoint 1: We used Jupiter and the intial code provided by the professor. We modified the intial code to move all the six joints. We basically copied the code for the handlers and the snippet which set the angles for each one of the joints.



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
