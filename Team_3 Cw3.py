# import all classes needed
from roboticstoolbox import Bicycle, RandomPath, VehicleIcon, RangeBearingSensor, LandmarkMap
from math import pi, atan2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

# create a function that checks for obstacle within a specified range and stops if an obstacle is detected.
def CheckObstacles(SensorReadings, Robot):
    Threat = False
    Obstacles = SensorReadings.h(Robot.x)
    ObstaclesR = Obstacles[:, 0]
    ObstaclesTheta = (Obstacles[:, 1] / pi) * 180
    for n in range(len(ObstaclesR)):
        if ObstaclesR[n] < 8:
                if abs(ObstaclesTheta[n]) < 45:
                    print('There is a Threat')
                    Threat = True
    return [Threat, ObstaclesR[n], ObstaclesTheta[n]]                
                    

# user inputs to show where the robot starts and where it stops
x = int(input("please enter x coordinate "))
y = int(input("please enter y coordinate "))
angle = int(input("please enter an angle "))
obs = int(input("please enter the number of obstacles "))
goal_x = int(input("please enter the x coordinate for the goal "))
goal_y = int(input("please enter the y coordinate for the goal "))

# VehicleIcon is a class that enables the animation of the vehicle used.
anim = VehicleIcon ("/home/hussein/Desktop/Hussein/robot.png", scale = 6)
Robot = Bicycle(
    animation = anim,
    control = RandomPath,
    dim = 10,
    x0 = (x, y, angle),
)
Robot.init(plot = True)
Robot._animation.update(Robot.x)

# loading the map with the obs being the number of landmarks and 50 units grids
map = LandmarkMap(obs, 50)
map.plot()

# The image module also includes two useful methods which are imread which is used to read images and imshow which is used to display the image.
image = mpimg.imread("/home/hussein/Desktop/Hussein/map.png")
plt.imshow(image, extent=[-50, 50, -50, 50])

# Adding the rangbearing sensor
sensor = RangeBearingSensor(robot = Robot, map = map, animate = True)
# Reading the output of the sensor
print('Snesor Readings: \n', sensor.h(Robot.x))

# Create a target point, and plot in on your grid. The target point is simply a list of its x and y coordinates. The marker can be customized, where it is a diamond-shaped marker or size 6 and color red.
Goal = [goal_x , goal_y]
GoalMarker = {
    "marker" : "D",
    "markersize" : 6,
    "color" : "r",
}

# Plot the vehicle and the target point
plt.plot(Goal[0],Goal[1],**GoalMarker)
plt.pause(1)

#create an array
goal_arr=[[-40,-25],[-25,-20],[-20,0],[0,25],[25,20]]
# create a path that the vehicle will follow from its initial position to the target position
goal_arr.append(Goal)
goal_arr.insert(0, [x, y])
print(goal_arr)
# Split the x and y coordinates into separate arrays
x_arr = [item[0] for item in goal_arr]
y_arr = [item[1] for item in goal_arr]


# create a nested loop that loops over the length of the points along the path (len(t)) and then we have the while loop that allows the vehicle to reach the consecutive point to whichever point it is currently located at.
for n in range(len(goal_arr)-1):
    run = True
    Goal = [x_arr[n+1], y_arr[n+1]]

#GoalAngle variable computes the angle of the of the target with respect to the vehicle throughout its movement.   
run = True
while (run):
    D2 = Goal[1] - Robot.x[1]
    D1 = Goal[0] - Robot.x[0]
    GoalAngle = atan2(D2, D1)
    # steer angle needed for the vehicle will change as well. It can easily be computed as the angle difference between the vehicle and the target point at a given instant
    SteeringAngle = GoalAngle - Robot.x[2]
    # If the absolute difference between the x-y coordinates of the vehicle and the target point does not exceed a given tolerance, here 0.02, the run flag is set to True, and the program keeps running. Otherwise, the run flag is set to False and then the loop check for obstacles.
    if abs(D2)>0.02 and abs(D1)>0.02:
        run = True
    else:
        run = False
    [Threat, ObsR, ObsA] = CheckObstacles(sensor, Robot)
    if Threat:
        if ObsA >= 0:
            SteeringAngle = SteeringAngle - (pi/4)
        elif ObsA < 0:
            SteeringAngle = SteeringAngle + (pi/4)            
    Robot.step(3,SteeringAngle)
    Robot._animation.update(Robot.x)
    plt.pause(0.005)
plt.pause(100)   