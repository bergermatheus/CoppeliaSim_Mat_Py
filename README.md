## Project to translate Matlab to Python code

The class developed here was created to communicate
Coppelia Simulator with Matlab Software. Now, we
want to improve the code and make it available in Python.

<img src="https://media.giphy.com/media/dU0jVmU13fQuDCmntQ/giphy.gif" height="250" width="350">

## Learning purposes

Feel free to colaborate and use everything.


## Lib RemoteApi

- To use the API correctly, insert on the project folder the remoteApi file compatible with the operating system: 

.dll for windows, .so linux, and .dylib for mac 

## Folders and Files

There are three different folders in this project.

1. Lib Folder:

    Contains the remoteApi library that makes the bridge between Matlab and Python with the simulator.

2. Matlab Folder:

    Contains the Class to comunicate with CoppeliaSim and Matlab. The main file is VREP.m (Constructor Class).

3. Python Folder:

    Contains the Class to comunicate with CoppeliaSim and Python. The main file is CoppeliaSimClass.py (Contructor Class).

## TO-DO

1. Load the scene, try to start and pause the simulation from the code (.py) ☑
2. Load the mobile robot Pioneer 3DX and if possible, move it (.py) ☑
3. Start implement the Class and organize the functions ☑
4. Run a strategy of position controller (.py)
5. Map environment with 2D Laser Scanner (.py)
