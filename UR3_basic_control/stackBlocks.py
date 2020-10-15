# click(button): button
# drag(wp): physically move robot to waypointWP
# waypointTopA : very top configuration above A (5x block height)
# waypointTopB : very top configuration above B (5x block height)
# waypointTopC : very top configuration above C (5x block height)
# waypointTopD : very top configuration above D (5x block height)
# waypointA : on top of first pickup square (block height)
# waypointB : on top of second pickup square (block height)
# waypointC : on top of third pickup square (block height)
# waypointD1 : on top of drop off square (block height)
# waypointD2 : on top of drop off square (2x block height)
# change(type) : change move type
# weightBoth: weight tool + weight payload
# weightTool: weight tool
# setTool (signal) : signal High means pick up and signal Low means let go
# setPayload (weight) : change the payload weight so that the robot does not shut down
# setTime (time) : wait for "time" seconds
# feedback : True if it actually picks something up


# The first half of the python file contains the program tree with just an overall structure 
# of what steps are being taken and waypoints that are reached while the second half has a 
# more detailed approach of what to press on the UI in order to actually carry out these actions


############## #
# PROGRAM TREE #
############## #

# A
MoveJ
    move(waypointTopA)
    blockPlaced = False
MoveL
    move(waypointA)
    tool = High
    if (feedback)
        weight = tool + payload
        blockPlaced = True
    move(waypointTopA)
MoveP # linear and constant speed to avoid dropping the block
    waypointTopD
MoveL
    move(waypointD1)
    tool = Low
    wait = 1
    weight = tool
    move(waypointTopD)

# B
MoveJ
    move(waypointTopB)
MoveL
    move(waypointB)
    tool = High
    if (feedback)
        weight = tool + payload
    move(waypointTopA)
MoveP # linear and constant speed to avoid dropping the block
    waypointTopD
MoveL
    if (blockPlaced)
        move(waypointD2)
    else
        move(waypointD1)
    tool = Low
    wait = 1
    weight = tool
    move(waypointTopD)

# C
MoveJ
    move(waypointTopC)
MoveL
    move(waypointC)
    tool = High
    if (feedback)
        weight = tool + payload
    move(waypointTopC)
MoveP # linear and constant speed to avoid dropping the block
    waypointTopD
MoveL
    move(waypointD2)
    tool = Low
    wait = 1
    weight = tool
    move(waypointTopD)



############# #
# STEPS TO DO #
############# #
click("start")
click("Program Robot")
click("Empty Program")

# Pick up block A
click("Structure/Move")
    setWaypoint()
    drag(waypointTopA)
    assign(blockPlaced:False)
click("MoveJ")
click("Structure/Move")
    change(L)
    setWaypoint()
    drag(waypointA)
    click("Structure/Set")
    setTool(High)
    if (feedback == True)
        setPayload(weightBoth)
        setTime(1)
        assign(blockPlaced:True)
    click("Add waypoint after")
    drag(waypointTopA)
click("MoveL")
click("Structure/Move")
    click("Command")
    change(P)
    click("waypoint")
    click("Set Waypoint")
    drag(waypointTopD)
click("MoveP")
click("Structure/Move")
    change(L)
    setWaypoint()
    drag(waypointD1)
    setTool(Low)
    setPayload(weightTool)
    click("Structure/Set")
    setTime(1)
    click("Add Waypoint after")
    drag(WaypointTopD)
click("MoveL")

#Pick up block B
click("Structure/Move")
    setWaypoint()
    drag(waypointTopB)
click("MoveJ")
click("Structure/Move")
    change(L)
    setWaypoint()
    drag(waypointB)
    click("Structure/Set")
    setTool(High)
    if (feedback == True)
        setPayload(weightBoth)
        setTime(1)
    click("Add waypoint after")
    drag(waypointTopB)
click("MoveL")
click("Structure/Move")
    click("Command")
    change(P)
    click("waypoint")
    click("Set Waypoint")
    drag(waypointTopD)
click("MoveP")
click("Structure/Move")
    change(L)
    if (blockPlaced == True)
        setWaypoint()
        drag(waypointD2)
    else
        setWaypoint()
        drag(waypointD1)
    setTool(Low)
    setPayload(weightTool)
    click("Structure/Set")
    setTime(1)
    click("Add Waypoint after")
    drag(WaypointTopD)
click("MoveL")

#Pick up block C
click("Structure/Move")
    setWaypoint()
    drag(waypointTopC)
click("MoveJ")
click("Structure/Move")
    change(L)
    setWaypoint()
    drag(waypointC)
    click("Structure/Set")
    setTool(High)
    if (feedback == True)
        setPayload(weightBoth)
        setTime(1)
    click("Add waypoint after")
    drag(waypointTopC)
click("MoveL")
click("Structure/Move")
    click("Command")
    change(P)
    click("waypoint")
    click("Set Waypoint")
    drag(waypointTopD)
click("MoveP")
click("Structure/Move")
    change(L)
    setWaypoint()
    drag(waypointD2)
    setTool(Low)
    setPayload(weightTool)
    click("Structure/Set")
    setTime(1)
    click("Add Waypoint after")
    drag(WaypointTopD)
click("MoveL")