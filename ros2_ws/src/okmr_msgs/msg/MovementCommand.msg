int8 command
#defines which of the commands defined below to execute

geometry_msgs/Vector3 translation #for relative move commands
geometry_msgs/Vector3 rotation 

float32 timeout_sec #max time to complete a command

float32 radius_of_acceptance 
float32 angle_threshold
#determines how close we have to be to the goal before its considered done
#used in rel move and abs move commands

okmr_msgs/GoalVelocity goal_velocity #for set velocity commands
okmr_msgs/GoalPose goal_pose #for absolute move commands

float32 altitude

std_msgs/Header header

int8 FREEZE = 0
    #hold current pose, stop all movements going on
    #this will automatically set all pid systems into position mode
    #equivlant to a move_relative command with no translation or rotation

int8 MOVE_RELATIVE = 1 
    #does a movement relative to the current (at time received) pose, 
    #ex. move 5m forward, rotate 90 degrees cw, etc
    #
    #translation (x, y, z), defines how many meters to move
    #rotation (r, p, y) in degrees NOT radians
    
    #NOTE: command is relative to the pose the auv was in when the command was received by the action server

int8 MOVE_ABSOLUTE = 2
    #move to the absolute position defined by the goal_pose field
    #can also define translation field, which will move the goal position accordingly
    #direction of translation can be defined by rotation field
    #will be useful for aligning type movements
    #can figure out a surface normal for the gate, slolumn, torpedo, then base rotation field off those
    #ALGORITHM ROUGH IDEA
    #take position from goal pose, then rotate a point 
    #(translation distance away) by roll pitch yaw of the rotation field
    #essentially setting goal pose position as the origin, placing the point defined by the translation field
    #then rotating that point around the origin (goal position) by the amount defined by rotation
    #this plan defintely has some flaws and will need to be better defined
    #maybe this method will never need to be implemented who knows

int8 SMOOTH_MOVE_ABSOLUTE = 9
    # wrapper of move absolute but with cubic spline curved path


int8 SET_VELOCITY = 3 #uses goal_velocity to request a velocity based movement, not looking at pose

#applies to both pitch and roll:
#will need to selectively enable pid systems 
#(disable all translation, disable other rotational PID systems, disable dead reckoning)
#inside the action server, need to check if command goal_velocity.twist.angular.x or .y is non zero

int8 LOOK_AT = 4
    #could be used for looking at a specific location
    #location to look at defined by "pose"
    #possibly useful for gripper, torpedo, dropper tasks
    #how to do this kind of thing with the bottom facing camera?

int8 SET_DEPTH = 5
    #go down dont #sink

int8 SURFACE_PASSIVE = 6
    #surface by turning off all motors
    #let buoyancy do its thing

int8 BARREL_ROLL = 7
    #disable dead reckoning, execute barrel roll using goal_velocity, then re-enable dead reckoning
    #uses goal_velocity field for barrel roll parameters


int8 SET_ALTITUDE = 8

int8 BBL_DVL = 9
    # Beam based 'lignment: Align back two beams on the DVL to the wall of the pool. Ideal for orienting in a cardinal direction
    # before a course
    # 
