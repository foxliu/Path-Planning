# CarND-Path-Planning-Project
### Summary
In this project, I utilized a behavior planner and spline function in order to drive a car around a simulated track, including performing lane changes as necessary when around traffic.

The car is required to be able to drive at least 4.32 miles around the track without going over the speed limit, leaving the road, colliding with another car, spending too long between lanes, and not exceeding certain acceleration and jerk thresholds (The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3).

### Detailed Implementation

In `main.cpp`, after the data from the simulator is pulled in, i first check whether there is any remaining path left.
If there is a remaining path,
this will get pushed back into the `next_x_vals` and `next_y_vals`
which to be fed the simulator agin (lines 239-244).
If it is the first time run, i have to create my points to be used in spline function to be
equal to where the car is. Additionally, i calculate the x, y, yaw and velocity based off the end
values of the remaining path, inordef to have the behavior planner begin from the end of the old path (lines 249-279)

Next, I need to change my x & y coordinates into frenet coordinates, s & d (line 281).
In frenet, "s" is the distance along a lane while "d" is the distance away from the center
dividing line of the road (the yellow line). By converting these,
it makes it easier to calculate where I want to be on the road,
as I can tell it exactly where in each lane to be.

Now it's time for the behavior planner itself.
In `behavior.cpp`, I start by calculating which lane I am in based off of "d" -
left, middle, right lane ware marked as 0, 1, 2.

I then get the closest car in the front of my vehicle in current lane
with `ClosetsVehicle()` which can be set to look eighter in front of or behind of my vehicle's "s" position in
diffent lanes.

If the closest vehicle in current lane is more than 20 meters away,
I will to drive near the speed limit and hold my lane. Otherwise
my vehicle does have another car close in front of it. This is where `laneScore()` come in.
This iterates through all three lanes, and checks both in front of and behind of my vehicle for the closest vehicles.
I add a slight benefit to holding my lane for priority is not to change the lane in the case that it is not very good.
There are then benefits (or costs) depending on how much distance there is in front and behind the vehicles in
the given lane and the speed difference between these vehicles and my vehicle (lines 76-83).

These scores are the taken an  average of the last 10 scores for that lane (lines 84-90), and then the best scoring
lane is returned from the function. Back into `lanePlanner()`, I then make sure to keep the vehicle's speed in
front of my vehicle in the lane i want to move to (or keep) so that i can match it. Finally, `lanePlanner()` returns
a move in meters: 0 represents keeping the same lane, -4 represents changing lane left, and 4 represents changing
lane right.

This brings it back into `main.cpp`, where the next "d" value gets
calculated (line 286) to be used in setting waypoints for the spline function.
However, there is still one last check performed to make sure that
the lane desired is actually going to be open for our move (lines 288-297) - if it isn't,
then the lane gets reset to the current one, and the target speed is reset to the old vehicle being followed.

Finally, it's time to make waypoints for the spline function. Based off the new desired "d",
I equally space out the waypoints by fifty meters - too much shorter sometimes caused it
to exceed max acceleration/jerk. These, along with any from the old path
(or the car starting point if initializing) are then shifted and rotated so
that they are local to my own vehicle (lines 311-318).
This helps to ensure the spline can work correctly, as the x-values need to be in order,
and we want to the be correctly spaced out going forward.

After setting the spline, I use the spline to come up with new points along that spline.
For each point, I space them out over the target distance using the current (per the calc)
velocity over 0.02 seconds, as the simulator moves every 20 milliseconds. For each point,
I compare the velocity (`ref_vel`) to my target vehicle's speed, and either accelerate or
deccelerate based on where I am in comparison. The "x" value is then changed by the distance
traveled at that speed, and "y" is calculated off the spline value for that "x".
These points are then rotated and shifted back to global coordinates, and fed to the simulator to drive!