import cv2
import camera
import serial
import time
import numpy as np

from particle import Particle, estimate_pose, add_uncertainty
from landmark import Landmark

DEMO = True
GUI = False

# Some colors constants
CRED = (0, 0, 255)
CGREEN = (0, 255, 0)
CBLUE = (255, 0, 0)
CCYAN = (255, 255, 0)
CYELLOW = (0, 255, 255)
CMAGENTA = (255, 0, 255)
CWHITE = (255, 255, 255)
CBLACK = (0, 0, 0)

# RGB Colors
RED = (0.52, 0.11, 0.34)
GREEN = (0.21, 0.42, 0.36)

# Landmarks.
# The robot knows the position of 4 landmarks. Their coordinates are in cm.
landmarks = [(0, 0), (0, 300), (400, 0), (400, 300)]
target = None # should we specify an angle for the target? Should be nil for exam!

# Exam landmarks
goals = [Landmark('L1', (0,300), RED, 'vertical'), Landmark('L2', (0,0), GREEN, 'vertical'), Landmark('L3', (300,400), GREEN, 'horizontal'), Landmark('L4', (400,0), RED, 'horizontal')]
avoid = []

port = '/dev/ttyACM0'

if DEMO:
    serialRead = serial.Serial(port,9600, timeout=1)

movestarttime = 0.0
currentmove = "0"
nextmove = []
waittime = 0.0
trying_to_find = 0
bubble = 75.0

landmark_x = 0
landmark_y = 300

cm_per_sec = None
deg_per_sec = None

def rad_2_deg(rads):
    return rads * (180.0 / np.pi)

def deg_2_rad(degs):
    return degs * (np.pi / 180.0)

def is_calibrated():
    return (cm_per_sec is not None) and (deg_per_sec is not None)

def jet(x):
    """Colour map for drawing particles. This function determines the colour of 
    a particle from its weight."""
    r = (x >= 3.0/8.0 and x < 5.0/8.0) * (4.0 * x - 3.0/2.0) + (x >= 5.0/8.0 and x < 7.0/8.0) + (x >= 7.0/8.0) * (-4.0 * x + 9.0/2.0)
    g = (x >= 1.0/8.0 and x < 3.0/8.0) * (4.0 * x - 1.0/2.0) + (x >= 3.0/8.0 and x < 5.0/8.0) + (x >= 5.0/8.0 and x < 7.0/8.0) * (-4.0 * x + 7.0/2.0)
    b = (x < 1.0/8.0) * (4.0 * x + 1.0/2.0) + (x >= 1.0/8.0 and x < 3.0/8.0) + (x >= 3.0/8.0 and x < 5.0/8.0) * (-4.0 * x + 5.0/2.0)

    return (255.0*r, 255.0*g, 255.0*b)

def draw_world(est_pose, particles, world):
    """Visualization.
    This functions draws robots position in the world."""
    
    offset = 50;
    
    world[:] = CWHITE # Clear background to white
    
    # Find largest weight
    max_weight = 0
    for particle in particles:
        max_weight = max(max_weight, particle.getWeight())

    # Draw particles
    for particle in particles:
        x = int(particle.getX()) + offset
        y = int(particle.getY()) + offset
        colour = jet(particle.getWeight() / max_weight)
        cv2.circle(world, (x,y), 2, colour, 2)
    
    # Draw landmarks
    lm1 = (landmarks[0][0]+offset, landmarks[0][1]+offset)
    lm2 = (landmarks[1][0]+offset, landmarks[1][1]+offset)
    lm3 = (landmarks[2][0]+offset, landmarks[2][1]+offset)
    lm4 = (landmarks[3][0]+offset, landmarks[3][1]+offset)
    cv2.circle(world, lm1, 5, CRED, 2)
    cv2.circle(world, lm2, 5, CGREEN, 2)
    cv2.circle(world, lm3, 5, CGREEN, 2)
    cv2.circle(world, lm4, 5, CRED, 2)
        
    # Draw estimated robot pose
    a = (int(est_pose.getX())+offset, int(est_pose.getY())+offset)
    b = (int(est_pose.getX() + 15.0*np.cos(est_pose.getTheta()))+offset, 
                                 int(est_pose.getY() + 15.0*np.sin(est_pose.getTheta()))+offset)
    cv2.circle(world, a, 5, CMAGENTA, 2)
    cv2.line(world, a, b, CMAGENTA, 2)


### Main program ###

if DEMO and GUI:
    # Open windows
    WIN_RF1 = "Robot view";
    cv2.namedWindow(WIN_RF1);
    cv2.moveWindow(WIN_RF1, 50, 50);

    WIN_World = "World view";
    cv2.namedWindow(WIN_World);
    cv2.moveWindow(WIN_World, 500, 50);

# Initialize particles
print "Initializing particles .."
num_particles = 1000
particles = []
for i in range(num_particles):
    # Random starting points. (x,y) \in [-1000, 1000]^2, theta \in [-pi, pi].
    p = Particle(2000.0*np.random.ranf() - 1000, 2000.0*np.random.ranf() - 1000, 2.0*np.pi*np.random.ranf() - np.pi, 1.0/num_particles)
    particles.append(p)

est_pose = estimate_pose(particles) # The estimate of the robots current pose

# Driving parameters
velocity = 0.0 # cm/sec
angular_velocity = 0.0 # radians/sec

# Initialize the robot (XXX: You do this)

# Allocate space for world map
world = np.zeros((500,500,3), dtype=np.uint8)

# Draw map
draw_world(est_pose, particles, world)

print "Opening and initializing camera"

if DEMO:
    #cam = camera.Camera(0, 'macbookpro')
    cam = camera.Camera(0, 'frindo')

def gaussian_distribution(x, mu, sigma):
    delta = x - mu
    Q = 2 * sigma * sigma
    denote = np.sqrt(Q * np.pi)
    return (np.exp(-(delta*delta) / Q))/denote

def checkpath(offroad, drivetime):
    global est_pose
    global bubble

    travel = forward_time_2_cm(drivetime)
    travel_frags = travel * 3.0
    ownx = est_pose.getX()
    owny = est_pose.getY()
    owntheta = est_pose.getTheta() + offroad
    badpath = False
    for i in range(int(travel_frags)):
        distance = (i * 3)
        dx = ownx + np.cos(owntheta) * distance
        dy = owny + -np.sin(owntheta) * distance
        for obstical in avoid:
            
            minx = minimum(dx, obstical.getUX()) 
            miny = minimum(dy, obstical.getUY())
            maxx = maximum(dx, obstical.getUX())
            maxy = maximum(dy, obstical.getUY())
            a = maxx - minx
            b = maxy - miny
            c = np.sqrt(a * a + b * b)
            if c < bubble:
                badpath = True
                break
        if badpath:
            break
    return not badpath

def inside_bubble():
    global est_pose
    ownx = est_pose.getX()
    owny = est_pose.getY()
    inabubble = False
    for obstical in avoid:
        
        minx = minimum(ownx, obstical.getUX()) 
        miny = minimum(owny, obstical.getUY())
        maxx = maximum(ownx, obstical.getUX())
        maxy = maximum(owny, obstical.getUY())
        a = maxx - minx
        b = maxy - miny
        c = np.sqrt(a * a + b * b)
        if c < bubble:
            inabubble = True
            break
    return inabubble
def stop(wait):
    global nextmove
    global currentmove
    global est_pose

    # send stop command to the adrino
    # calculate and apply the movemt to the particles
    msg = 's'
    serialRead.write(msg.encode('ascii'))
    runtime = time.time() - movestarttime

    waittime = time.time() + wait
    if runtime > 1.0:
        if currentmove == "F":
            if runtime < 1.2:
                distance = 0
            else:

                distance = forward_time_2_cm(runtime)

                for p in particles:
                    x = p.getX() + (np.cos(p.getTheta()) * distance)
                    y = p.getY() + ((-np.sin(p.getTheta())) * distance)
                    p.setX(x)
                    p.setY(y)
                add_uncertainty(particles, 5, 0.2)
        elif currentmove == "H":
            distance = cw_time_2_deg(runtime) * (np.pi/180)
            for p in particles:
                theta = p.getTheta()
                p.setTheta(theta - distance)
            add_uncertainty(particles, 1, 0.2)
        elif currentmove == "L":
            distance = ccw_time_2_deg(runtime) * (np.pi/180)
            for p in particles:
                theta = p.getTheta()
                p.setTheta(theta + distance)
            add_uncertainty(particles, 1, 0.2)
    
    currentmove = "0"
    est_pose = estimate_pose(particles)

def movecommand (command, wait):
    global nextmove
    global waittime
    global currentmove
    global movestarttime
    
    emergency = False
    #sends command to the adrino, saves the current time and command.
    if command == "F":
        msg = 'r1\n'
        serialRead.write(msg.encode('ascii'))
        front = serialRead.readline()
        print "infrared sensor read: ", front
        front_val = float(front)
        if front_val > 10.0 and front_val < 80:
            est_pose = est_pose = estimate_pose(particles)
            a = np.cos(est_pose.getTheta()) * front_val + est_pose.getX()
            b = -np.sin(est_pose.getTheta()) * front_val + est_pose.getY()
            too_close = False
            for goal in goals:
                minx = minimum(a, goal.getUX()) 
                miny = minimum(b, goal.getUY())
                maxx = maximum(a, goal.getUX())
                maxy = maximum(b, goal.getUY())

                da = maxx - minx
                db = maxy - miny
                dc = np.sqrt(a * a + b * b)
                if dc < 75:
                    too_close = True
                    break

            if not too_close:
		avoid.append(Landmark("box", (a,b), GREEN, "irrelvant"))
        if not checkpath(0.0, wait):
            notrouble = False
            if inside_bubble:
               msg = 'p 8 1.0'
            else:
               offroad = 0
               while not notrouble and offroad < 4:
                   if offroad < 0:
                       offroad = (-offroad) + 1
                   else:
                       offroad = -offroad
                   notrouble = checkpath(offroad,wait)
            if notrouble:
               if offroad < 0:
                   turn = "H"
                   turnlength = cw_deg_2_time(rad_2_deg(offroad))
               else:
                   turn = "L"
                   turnlength = ccw_deg_2_time(rad_2_deg(offroad))
               nextmove = [[turn, turnlength],["F",wait]]
               emergency = True
            else:
               msg = 'p 8.0 1.0'
        msg = 'p 8 1.0'
    elif command == "L":
        msg = 'p 300.0 1.0'
    elif command == "H":
        msg = 'p -310.0 1.0'
    else:
        print "WARNING: unknown move command!"
    if not emergency: 
        if DEMO:
            serialRead.write(msg.encode('ascii'))

        movestarttime = time.time()

        if len(nextmove) == 0:
            nextmove = [["S", 0.3]]
        else:
            nextmove.pop(0)
            nextmove.insert(0, ["S", 0.3])

        currentmove = command
        waittime = time.time() + wait

def minimum(a,b):
    if a < b:
        return a
    return b

def maximum(a,b):
    if a > b:
        return a
    return b

def getDeltaFromBot(goal):
    global est_pose
    minx = minimum(est_pose.getX(), goal.getUX()) 
    miny = minimum(est_pose.getY(), goal.getUY())
    maxx = maximum(est_pose.getX(), goal.getUX())
    maxy = maximum(est_pose.getY(), goal.getUY())

    a = maxx - minx
    b = maxy - miny
    c = np.sqrt(a * a + b * b)
    
    theta = np.arccos(a/c)
    
    return (c, theta)

def ccw_deg_2_time(theta):
    return 1.2449 * (theta / 360.0) + 1.0902

def cw_deg_2_time(theta):
    return 1.3423 * (theta / 360.0) + 0.9646

def ccw_time_2_deg(time):
    return (0.8026 * time - 0.8746) * 360

def cw_time_2_deg(time):
    return (0.7433 * time - 0.7159) * 360

def forward_cm_2_time(cm):
    return 0.0291 * cm + 1.2083

def forward_time_2_cm(time):
    return 34.339 * time - 41.4407


def resample(particel_list, sample_random):
    RS_particles = []
    total_weight = 0.0
    num_sample = num_particles - sample_random
    for p in particel_list:
        total_weight += p.getWeight()
    for i in range(num_sample):
        random_number =  total_weight * np.random.random_sample()
        random_number -= particel_list[0].getWeight()
        n = 0
        while random_number > 0.0:
            n += 1
            random_number -= particles[n].getWeight()
        RS_particles.append(particles[n])
    for i in range(sample_random):
        p = Particle(2000.0*np.random.ranf() - 1000, 2000.0*np.random.ranf() - 1000, 2.0*np.pi*np.random.ranf() - np.pi, 1.0/num_particles)
        RS_particles.append(p)
    return RS_particles

def detect_objects():
    global goals
    global target
    global particles
    global landmark_x
    global landmark_y

    # Detect objects
    if DEMO:
        objectType, measured_distance, measured_angle, colourProb = cam.get_object(colour)
    else:
        objectType = 'vertical'
        measured_distance = 100
        measured_angle = deg_2_rad(90.0)
        colourProb = (0,0,0)
    
    if objectType != 'none':
        """ FOR EXAM """
        for goal in goals:

            # if we find one of the goals
            if goal.match(objectType, colourProb):
                
                # calculate position of the goal
                x = np.cos(measured_angle) * measured_distance
                y = -np.sin(measured_angle) * measured_distance
                print measured_distance
                # set its position
                goal.setPosition(x, y)
                goal.setTheta(measured_angle)

                # announce that we're clever enough to identify which goal and where it is :)
                print "Found", goal.getIdentifier(), "at (", goal.getX(), ",", goal.getY(), "), theta: ", goal.getTheta()

                landmark_x = goal.getUX()
                landmark_y = goal.getUY()

                # is it the one we want to visit?
                if goal == goals[0]:
                    target = goal

        # Compute particle weights
        sigma_D = 15 
        sigma_A = 0.2
        for p in particles:
            X = p.getX()
            Y = p.getY()
            if (X - landmark_x) == 0 and (Y - landmark_y) == 0:
                p.setWeight(0)
            else:
                distance = np.sqrt((X - landmark_x) * (X - landmark_x) + (Y - landmark_y) * (Y - landmark_y))
                theta = p.getTheta()
                dx = np.cos(theta)
                dy = -np.sin(theta)
                lx = landmark_x - X
                ly = landmark_y - Y 
                angle = (lx*dx + ly*dy) / (np.sqrt(lx * lx + ly * ly) * np.sqrt(lx * lx + ly * ly))
                if angle < -1:
                    angle = -1
                elif angle > 1:
                    angle = 1
                D = gaussian_distribution(measured_distance, distance, sigma_D)
                A = gaussian_distribution(measured_angle, np.arccos(angle), sigma_A)
                p.setWeight(D * A)

        # Resampling
        particles = resample(particles, 100)

        # Draw detected pattern
        if DEMO:
            cam.draw_object(colour)
    else:
        # No observation - reset weights to uniform distribution
        for p in particles:
            p.setWeight(1.0/num_particles)

    est_pose = estimate_pose(particles) # The estimate of the robots current pose

while len(goals) > 0:

    action = cv2.waitKey(10)
    if action == ord('q'):
        break
    """ EXAM: Robot movement """
    # XXX: Make the robot drive

    # fallback when we have no target, it will try to go/rotate there
    delta = est_pose.getDeltaForTarget(Particle(0, 0, 90))

    # executing move orders
    if waittime > time.time():
        if currentmove == "F":
            msg = 'r1\n'
            serialRead.write(msg.encode('ascii'))
            front_val = float(serialRead.readline())
            if front_val < 30.0:
                stop(0.3)
                nextmove = [["L", ccw_deg_2_time(90)],["F", forward_cm_2_time(50)],["H", cw_deg_2_time(90)]]
            
        continue
    elif len(nextmove) > 0:
        if nextmove[0][0] == "S":
            stop(nextmove[0][1])
            nextmove.pop(0)
            print "list after stop:", nextmove
        else:
            movecommand(nextmove[0][0],nextmove[0][1])
            print "list after move:", nextmove
        continue

    pic_taken = 0
    while target == None and pic_taken < 3: 
        if DEMO:
            # Fetch next frame
            colour, distorted = cam.get_colour()    
    
        detect_objects()
        pic_taken += 1

    # if we have a target
    if target is not None:
        trying_to_find = 0
        delta = est_pose.getDeltaForTarget(target)
        print "Landmark is: ", target.getIdentifier()
        print "distance: ", target.getDistance()
        print "angle in deg: ", deg_2_rad(target.getTheta())
        
        # if we're within "visiting distance" of the goal
        if target.getDistance() < 60: # visiting range is < 75cm, so 60cm should be close enough
            print "We've visited ", target.getIdentifier()
            avoid.append(goals.pop(0))
            target = None
        else:
            # if we're looking more or less directly at it
            
            print "Moving toward ", target.getIdentifier()
            d = target.getDistance() - 45.0
            if d > 75.0:
                d = 75.0
            nextmove.append(["F", forward_cm_2_time(d)])

                
            # otherwise we can move toward the target
        target = None
        
    else:
        print "Looking for target .."
        # look for a target subroutine take picture 3 times, if no landmark is found, turn and try again. 
        if trying_to_find < 8:
            nextmove.append(["L", ccw_deg_2_time(45)])
            trying_to_find += 1
        else:
            # can't find by rotatting time to guess based on there we are
            trying_to_find = 0
            est_pose = estimate_pose(particles)
            targ = goals[0]
            length, theta =  getDeltaFromBot(targ)
            if length > 75.0:
                length = 75.0
            
            if theta < 0:
                direction = "H"
                dirlength = cw_deg_2_time(rad_2_deg(theta))
            else:
                direction = "L"
                dirlength = cw_deg_2_time(rad_2_deg(theta))
            nextmove = [[direction, dirlength],["F",forward_cm_2_time(length)]]
            print "estimated theta: ", theta

    if DEMO and GUI:
        # Draw map
        draw_world(est_pose, particles, world)
        
        # Show frame
        cv2.imshow(WIN_RF1, colour);

        # Show world
        cv2.imshow(WIN_World, world);

# TODO: for exam, measure time and do; print "Finished in x seconds!"

# Close all windows
stop(0.0)

if GUI:
    cv2.destroyAllWindows()
