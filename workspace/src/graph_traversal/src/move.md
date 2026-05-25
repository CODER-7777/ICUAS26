struct AStarNode -> In this the operator > is overwritten by the comparison of the f score

inflation_radius = 0.3 
z_target = 1
max_speed = 12.0

Multiple Concurrent Operations
Your node runs simultaneously:
Subscriber callbacks (drone poses, battery status, waypoints) - constantly receiving data
Planning timer (100ms) - runs runSwarmSystem() - heavy computation
Control timer (50ms) - runs publishCommands() - sends commands to drones
2. Without Reentrant Callback Group
   With a default (non-reentrant) callback group, callbacks would be serialized (executed one-at-a-time), causing:
   ❌ Subscribers blocked while planning runs (100ms delay)
   ❌ Commands delayed while subscriptions process
   ❌ Stale sensor data by the time timers execute
   ❌ Overall system latency and poor responsiveness
3. With Reentrant Callback Group
   Callbacks can run concurrently on multiple threads:
   ✅ Pose updates arrive while planning is running
   ✅ Battery status updates don't block command publishing
   ✅ Latest sensor data available when timers fire
   ✅ Real-time swarm coordination

What is a Mutex?
A M utex (Mutual Exclusion lock) is a synchronization tool that ensures only one thread can access a shared resource at a time.

hasGridLineOfSight : 

It uses Bresenham's line algorithm 