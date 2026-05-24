# Aruco Search Algorithm

## Pre-knowledge
1. Currently the drone that does not do anything if it is above a battery threshold
and not part of the chain that tracks the AGV
2. We have the octomap of the entire area but the octomap is given by an external node on runtime so we cannot hardcode values from the current ones
3. We can derive the position of the centre of poles using the octomap provided in the start of the run

## Algorithm

1. Locate all the pole centres in the beginning of the run.
2. Divide each pole into 8 zones, i.e in 4 cardinal directions one at 75% of the height
the other at 25% of the height.
3. Each zone will have a viewing area i.e a particular volume in which the drone if inside we can assume it to be searched.
4. Find all the viewing areas the bystanding drone can visit without breaking the line of sight constraint(which is that it should be in line of sight of any one the drones or the base station)
5. Use the most suitable algorithm to move through each of the viewing zones. Make sure when moving into a viewing zone the face of the drone must be towards the centre of the pole.