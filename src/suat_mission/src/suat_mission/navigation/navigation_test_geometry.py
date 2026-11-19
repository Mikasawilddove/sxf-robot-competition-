"""Bounded short-goal geometry and arrival checks, independent of ROS."""
import math


def short_goal(start, forward, left=0.0):
    if len(start)!=3 or not all(math.isfinite(v) for v in (*start,forward,left)):
        raise ValueError('non-finite test geometry')
    if not .15<=forward<=.50 or abs(left)>.15 or abs(math.atan2(left,forward))>.35:
        raise ValueError('short goal exceeds distance or heading envelope')
    x,y,a=start
    return (x+forward*math.cos(a)-left*math.sin(a),
            y+forward*math.sin(a)+left*math.cos(a),a)


def arrival_error(goal,current):
    if len(goal)!=3 or len(current)!=3 or not all(math.isfinite(v) for v in (*goal,*current)):
        raise ValueError('non-finite arrival pose')
    return (math.hypot(goal[0]-current[0],goal[1]-current[1]),
            abs(math.atan2(math.sin(goal[2]-current[2]),math.cos(goal[2]-current[2]))))


def arrival_confirmed(goal,current,linear,angular,age,xy_tolerance=.08,yaw_tolerance=.12):
    if not all(math.isfinite(v) for v in (linear,angular,age)) or not 0<=age<=.4:
        return False
    xy,yaw=arrival_error(goal,current)
    return xy<=xy_tolerance and yaw<=yaw_tolerance and abs(linear)<.015 and abs(angular)<.03
