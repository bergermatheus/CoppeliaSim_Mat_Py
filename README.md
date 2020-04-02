## Project to translate Matlab to Python code

The class developed here was created to communicate
Coppelia Simulator with Matlab Software. Now, we
want to improve the code and make it available in Python.

<img src="https://media.giphy.com/media/UuZMcNDe2mhHvPrUNF/giphy.gif" height="250" width="450">

## Learning purposes

Feel free to colaborate and use everything.


## Dependencies

Coppelia Robotics: you can download the educational version in [link](https://coppeliarobotics.com/downloads)

## Lib RemoteApi

- To use the API correctly, insert on the project folder the remoteApi file compatible with the operating system: 

.dll for windows, .so linux, and .dylib for mac

**OBS:** We suggest you to get the earliest API version in the Coppelia Robotics folder `/programming/remoteApiBindings/lib/lib`

## Folders and Files

There are three different folders in this project.

1. Lib Folder:

    Contains the remoteApi library that makes the bridge between Matlab and Python with the simulator.

2. Matlab Folder:

    Contains the Class to comunicate with CoppeliaSim and Matlab. The main file is VREP.m (Constructor Class).

3. Python Folder:

    Contains the Class to comunicate with CoppeliaSim and Python. The main file is CoppeliaSimClass.py (Constructor Class).

## TO-DO

1. Load the scene, try to start and pause the simulation from the code (.py) ☑
2. Load the mobile robot Pioneer 3DX and if possible, move it (.py) ☑
3. Start implement the Class and organize the functions ☑
4. Run a strategy of position controller (.py) ☑
5. Map environment with 2D Laser Scanner (.py) ☑
6. Implement avoidance algorithm (Potential Fields) ☑
7. Implement the detection of objects on the scene

## Contact

- Matheus Berger Quemelli: matheus.quemelli@gmail.com
- Emows Lemos: emowslemos@gmail.com
- André Lobo : andreloboteixeira@gmail.com