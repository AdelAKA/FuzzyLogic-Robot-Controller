"""fuzzy_control controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, Keyboard, Supervisor

import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

def init_fuzzy_target_control():
    err_angle = ctrl.Antecedent(np.arange(-200, 201, 1), 'err_angle')
    distance = ctrl.Antecedent(np.arange(0, 13, 1), 'distance')
    velocity_l = ctrl.Consequent(np.arange(0, 13, 1), 'velocity_l')
    velocity_r = ctrl.Consequent(np.arange(0, 13, 1), 'velocity_r')
    
    err_angle['N'] = fuzz.trapmf(err_angle.universe, [-200,-200,-180 ,-90])
    err_angle['SN'] = fuzz.trimf(err_angle.universe, [-180,-90,-45])
    err_angle['NNZ'] = fuzz.trimf(err_angle.universe, [-90,-45,0])
    err_angle['Z'] = fuzz.trimf(err_angle.universe, [-45,0,45])
    err_angle['NPZ'] = fuzz.trimf(err_angle.universe, [0,45,90])
    err_angle['SP'] = fuzz.trimf(err_angle.universe, [45,90,180])
    err_angle['P'] = fuzz.trapmf(err_angle.universe, [90,180,200,200])

    distance['Z'] = fuzz.trimf(distance.universe, [0,0,2])
    distance['NZ'] = fuzz.trimf(distance.universe, [0,2,4])
    distance['NM'] = fuzz.trimf(distance.universe, [2,4,6])
    distance['M'] = fuzz.trimf(distance.universe, [4,6,8])
    distance['NF'] = fuzz.trimf(distance.universe, [6,8,10])
    distance['F'] = fuzz.trimf(distance.universe, [8,10,12])
    distance['VF'] = fuzz.trimf(distance.universe, [10,12,12])

    velocity_l['Z'] = fuzz.trimf(velocity_l.universe, [0,0,2])
    velocity_l['S'] = fuzz.trimf(velocity_l.universe, [0,2,4])
    velocity_l['NM'] = fuzz.trimf(velocity_l.universe, [2,4,6])
    velocity_l['M'] = fuzz.trimf(velocity_l.universe, [4,6,8])
    velocity_l['NH'] = fuzz.trimf(velocity_l.universe, [6,8,10])
    velocity_l['H'] = fuzz.trimf(velocity_l.universe, [8,10,12])
    velocity_l['VH'] = fuzz.trimf(velocity_l.universe, [10,12,12])
    
    velocity_r['Z'] = fuzz.trimf(velocity_r.universe, [0,0,2])
    velocity_r['S'] = fuzz.trimf(velocity_r.universe, [0,2,4])
    velocity_r['NM'] = fuzz.trimf(velocity_r.universe, [2,4,6])
    velocity_r['M'] = fuzz.trimf(velocity_r.universe, [4,6,8])
    velocity_r['NH'] = fuzz.trimf(velocity_r.universe, [6,8,10])
    velocity_r['H'] = fuzz.trimf(velocity_r.universe, [8,10,12])
    velocity_r['VH'] = fuzz.trimf(velocity_r.universe, [10,12,12])
    
    z, s, nm, m, nh, h, vh = ['Z', 'S', 'NM', 'M', 'NH', 'H', 'VH']
    
    distance_values = ['Z', 'NZ', 'NM', 'M', 'NF', 'F', 'VF']
    angle_values = ['N', 'SN', 'NNZ', 'Z', 'NPZ', 'SP', 'P']
    velocity_l_values = [[None] * 7] * 7
    velocity_l_values[0] = [z, z, z, z, nm, nm, m]
    velocity_l_values[1] = [s, s, s, s, m, nm, h]
    velocity_l_values[2] = [s, s, s, nm, nh, h, vh]
    velocity_l_values[3] = [s, s, s, m, h, h, vh]
    velocity_l_values[4] = [s, s, nm, nh, nh, h, vh]
    velocity_l_values[5] = [s, s, m, h, nh, h, vh]
    velocity_l_values[6] = [s, s, nm, vh, nh, h, vh]
    
    rules = [None] * 49
    for i in range(7):
        for j in range(7):
            rules [i * 7 + j] = ctrl.Rule(distance[distance_values[i]] & err_angle[angle_values[j]], (velocity_l[velocity_l_values[i][j]] ,velocity_r[velocity_l_values[i][6-j]]))
    velocity_ctrl = ctrl.ControlSystem(rules)
    velocity = ctrl.ControlSystemSimulation(velocity_ctrl)
    return velocity
    

def init_fuzzy_obstacle_avoidance_control():
    obstacle_distance_l = ctrl.Antecedent(np.arange(0, 80, 1), 'obstacle_distance_l')
    obstacle_distance_f = ctrl.Antecedent(np.arange(0, 80, 1), 'obstacle_distance_f')
    obstacle_distance_r = ctrl.Antecedent(np.arange(0, 80, 1), 'obstacle_distance_r')
    avoid_velocity_l = ctrl.Consequent(np.arange(-10, 15, 1), 'avoid_velocity_l')
    avoid_velocity_r = ctrl.Consequent(np.arange(-10, 15, 1), 'avoid_velocity_r')
    
    obstacle_distance_l['N'] = fuzz.trapmf(obstacle_distance_l.universe, [0,10,20 ,30])
    obstacle_distance_l['M'] = fuzz.trapmf(obstacle_distance_l.universe, [20,30,40,50])
    obstacle_distance_l['F'] = fuzz.trapmf(obstacle_distance_l.universe, [40, 50,60,70])
    obstacle_distance_l['I'] = fuzz.trapmf(obstacle_distance_l.universe, [70, 70,80,80])
    
    obstacle_distance_f['N'] = fuzz.trapmf(obstacle_distance_f.universe, [0,10,20 ,30])
    obstacle_distance_f['M'] = fuzz.trapmf(obstacle_distance_f.universe, [20,30,40,50])
    obstacle_distance_f['F'] = fuzz.trapmf(obstacle_distance_f.universe, [40, 50,60,70])
    obstacle_distance_f['I'] = fuzz.trapmf(obstacle_distance_f.universe, [71, 71,80,80])
    
    obstacle_distance_r['N'] = fuzz.trapmf(obstacle_distance_r.universe, [0,10,20 ,30])
    obstacle_distance_r['M'] = fuzz.trapmf(obstacle_distance_r.universe, [20,30,40,50])
    obstacle_distance_r['F'] = fuzz.trapmf(obstacle_distance_r.universe, [40, 50,60,70])
    obstacle_distance_r['I'] = fuzz.trapmf(obstacle_distance_r.universe, [71, 71,80,80])
    
    avoid_velocity_l['NH'] = fuzz.trimf(avoid_velocity_l.universe, [-10,-5,0])
    avoid_velocity_l['N'] = fuzz.trimf(avoid_velocity_l.universe, [-5,-2,0])
    avoid_velocity_l['P'] = fuzz.trimf(avoid_velocity_l.universe, [0,2,2])
    avoid_velocity_l['HP'] = fuzz.trimf(avoid_velocity_l.universe, [0,6,8])
    avoid_velocity_l['VHP'] = fuzz.trimf(avoid_velocity_l.universe, [6,8,12])
    
    avoid_velocity_r['NH'] = fuzz.trimf(avoid_velocity_r.universe, [-10,-5,0])
    avoid_velocity_r['N'] = fuzz.trimf(avoid_velocity_r.universe, [-5,-2,0])
    avoid_velocity_r['P'] = fuzz.trimf(avoid_velocity_r.universe, [0,2,2])
    avoid_velocity_r['HP'] = fuzz.trimf(avoid_velocity_r.universe, [0,6,8])
    avoid_velocity_r['VHP'] = fuzz.trimf(avoid_velocity_r.universe, [6,8,12])
    
    nh, n, p, hp, vhp = ['NH', 'N', 'P', 'HP', 'VHP']
    avoid_velocity_l_values = [hp, hp, hp, vhp, hp, hp, vhp, hp, hp, 
                                nh, hp, hp, p, hp, hp, p, vhp, vhp, 
                                nh, nh, hp, p, p, hp, p, p, vhp]
    
    avoid_velocity_r_values = [nh, nh, nh, vhp, p, p, vhp, p, p, 
                                hp, nh, nh, hp, p, p, hp, p, p, 
                                hp, vhp, nh, hp, p, p, hp, vhp, vhp]
    rules = [None] * 39
    index = 0
    for i in ['N', 'M', 'F']:
        for j in ['N', 'M', 'F']:
            for k in ['N', 'M', 'F']:
                rules [index] = ctrl.Rule(obstacle_distance_l[i] & obstacle_distance_f[j] & obstacle_distance_r[k], (avoid_velocity_l[avoid_velocity_l_values[index]] ,avoid_velocity_r[avoid_velocity_r_values[index]]))
                index += 1   
    avoid_velocity_l_values = [hp, p, vhp, p, vhp, hp, vhp]
    avoid_velocity_r_values = [hp, vhp, p, vhp, p, hp, p]
    rules[27] = ctrl.Rule(obstacle_distance_l['I'] & obstacle_distance_f['I'] & obstacle_distance_r['I'], (avoid_velocity_l[avoid_velocity_l_values[0]], avoid_velocity_r[avoid_velocity_r_values[0]]))
    rules[28] = ctrl.Rule(obstacle_distance_l['I'] & obstacle_distance_f['I'], (avoid_velocity_l[avoid_velocity_l_values[1]], avoid_velocity_r[avoid_velocity_r_values[1]]))
    rules[29] = ctrl.Rule(obstacle_distance_l['I'] & obstacle_distance_r['I'], (avoid_velocity_l[avoid_velocity_l_values[2]], avoid_velocity_r[avoid_velocity_r_values[2]]))
    rules[30] = ctrl.Rule(obstacle_distance_l['I'], (avoid_velocity_l[avoid_velocity_l_values[3]], avoid_velocity_r[avoid_velocity_r_values[3]]))
    rules[31] = ctrl.Rule(obstacle_distance_f['I'] & obstacle_distance_r['I'], (avoid_velocity_l[avoid_velocity_l_values[4]], avoid_velocity_r[avoid_velocity_r_values[4]]))
    rules[32] = ctrl.Rule(obstacle_distance_f['I'], (avoid_velocity_l[avoid_velocity_l_values[5]], avoid_velocity_r[avoid_velocity_r_values[5]]))
    rules[33] = ctrl.Rule(obstacle_distance_r['I'], (avoid_velocity_l[avoid_velocity_l_values[6]], avoid_velocity_r[avoid_velocity_r_values[6]]))
    rules[34] = ctrl.Rule(obstacle_distance_l['N'], (avoid_velocity_l[p], avoid_velocity_r[n]))
    rules[35] = ctrl.Rule(obstacle_distance_f['N'], (avoid_velocity_l[vhp], avoid_velocity_r[nh]))
    rules[36] = ctrl.Rule(obstacle_distance_r['N'], (avoid_velocity_l[n], avoid_velocity_r[p]))
    rules[37] = ctrl.Rule(obstacle_distance_l['M'], avoid_velocity_l[p])
    rules[38] = ctrl.Rule(obstacle_distance_r['M'], avoid_velocity_r[p])       
    avoid_velocity_ctrl = ctrl.ControlSystem(rules)
    avoid_velocity = ctrl.ControlSystemSimulation(avoid_velocity_ctrl)
    return avoid_velocity

def get_err_angle(gp, gp2, target):
    X, _, Z = gp.getValues()
    Xref, _, Zref = gp2.getValues()
    XT, ZT = target
    vec1 = [X - Xref, Z - Zref]
    vec2 = [XT - Xref, ZT - Zref]
    Norm = np.sqrt(vec1[0] * vec1[0] + vec1[1] * vec1[1])
    vec1[0] /= Norm
    vec1[1] /= Norm
    Norm = np.sqrt(0.81 + 0.36)
    vec2[0] /= Norm
    vec2[1] /= Norm
    # print("1 ", vec1[0], ", ", vec1[1])
    # print("2 ", vec2[0], ", ", vec2[1])
    
    dot = vec1[0]*vec2[0] + vec1[1]*vec2[1]
    det = vec1[0]*vec2[1] - vec1[1]*vec2[0];
    #print(dot, ", ", det)
    angle = np.arctan2(det, dot)
    # print("angle", angle)
    return angle

def get_distance(gp, target):
    X, _, Z = gp.getValues()
    XT, ZT = target
    dist = np.linalg.norm([XT - X, ZT - Z])
    return dist

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
kb = Keyboard()
kb.enable(timestep)

velocity = init_fuzzy_target_control()
avoid_velocity = init_fuzzy_obstacle_avoidance_control()

gp = robot.getGPS("gps")
gp.enable(timestep)

gp2 = robot.getGPS("gps2")
gp2.enable(timestep)

ds = []
dsNames = ["so0", "so3", "so7"];
for i in range(3):
  ds.append(robot.getDistanceSensor(dsNames[i]));
  ds[i].enable(timestep);

wheels = []
wheels_names = ["left wheel", "right wheel"]
for i in range(2):
    wheels.append(robot.getMotor(wheels_names[i]))
    wheels[i].setPosition(float('inf'));
    wheels[i].setVelocity(0.0);

targets = {}
targets["red"] = [1.6, -1.6]
targets["blue"] = [-1.6, -1.6]
targets["green"] = [1.6, 1.6]
targets["yellow"] = [-1.6, 1.6]
target = targets["red"]

# Main loop:
# - perform simulation steps until Webots is stopping the controller
oaflc = False # Obstacles Avoiding Fuzzy Logic Controller
max_speed = 6.0
l_speed = 0
r_speed = 0
debuging = False
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    key = kb.getKey()
    if key == ord('R'):
      target = targets["red"]
    elif key == ord('G'):
      target = targets["green"]
    elif key == ord('B'):
      target = targets["blue"]
    elif key == ord('Y'):
      target = targets["yellow"]
    if debuging:
        print("sensors", ds[0].getValue(), ds[1].getValue(), ds[2].getValue())
    if any(sensor.getValue() < 60 for sensor in ds):
        oaflc = True
    else:
        oaflc = False

    # print("err_angle = ", err_angle)
    # print("distance = ", distance)
    
    # Process sensor data here.
    if oaflc:
        avoid_velocity.input['obstacle_distance_l'] = ds[0].getValue()
        avoid_velocity.input['obstacle_distance_f'] = ds[1].getValue()
        avoid_velocity.input['obstacle_distance_r'] = ds[2].getValue()
        avoid_velocity.compute()
        if debuging:
            print(avoid_velocity.output['avoid_velocity_l'], avoid_velocity.output['avoid_velocity_r'])
        l_speed = avoid_velocity.output['avoid_velocity_l'] 
        r_speed = avoid_velocity.output['avoid_velocity_r']
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    err_angle = get_err_angle(gp, gp2, target)
    distance = get_distance(gp, target) * 3
    if debuging:
        print("distance :", distance)
    
    if not oaflc:
        velocity.input['distance'] = distance
        velocity.input['err_angle'] = np.degrees(err_angle)
        velocity.compute()
        if debuging:
            print(velocity.output['velocity_l'], velocity.output['velocity_r'])
        l_speed = velocity.output['velocity_l']
        r_speed = velocity.output['velocity_r']
        if distance < 0.001:
            l_speed, r_speed = (0, 0)
            
    if key == ord('A'):
      r_speed += 0.5
      l_speed = 0
    elif key == ord('D'):
      l_speed += 0.5
      r_speed = 0
    elif key == ord('W'):
      r_speed = 2
      l_speed = 2
    elif key == ord('S'):
      r_speed = 2
      l_speed = 2 
    
    r_speed = max_speed if r_speed > max_speed else r_speed
    l_speed = max_speed if l_speed > max_speed else l_speed
        
    wheels[0].setVelocity( l_speed)
    wheels[1].setVelocity( r_speed)
    pass

# Enter here exit cleanup code.
