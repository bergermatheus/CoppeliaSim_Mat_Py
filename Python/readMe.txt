Make sure you have following files in this directory, in order to run the examples:

remoteApi located on Lib folder, based on your OS.
Coppelia.py located on Python folder, communicate Coppelia simulator and Python
-------------------------------------------------------------------
Class Constructor is: Copellia
Followed by the properties and methods.
1- Properties
-- clientID: connection between Python and CoppeliaSim

2- Methods
-- start_Simulation()
-- stop_Simulation()

-------------------------------------------------------------------
Class Constructor for mobile Robot: Pioneer3DX
Followed by the properties and methods.
1- Properties
-- clientID: connection between Python and CoppeliaSim
-- pioneer3DX_array: tags used to hangle the object on the simulation
-- position_coordX: controlling coordinates (X Y Z)
-- position_coordXc: center coordinates (Xc Yc Zc) of the mobile robot
-- orientation: the angle of orientation regarding to Z axis.
-- velocity: array with linear and angular velocity
-- name: name of the object on the scene
-- left_motor: name of the object on the scene
-- right_motor: name of the object on the scene
-- ultrasonic_sensors: name of the object on the scene

2- Methods
-- send_ControlSignals()
-- get_PositionData()
-- get_UltrasonicData()