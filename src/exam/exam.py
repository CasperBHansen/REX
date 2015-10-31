import cv2
import camera
import serial
import time
import numpy as np

from particle import Particle, estimate_pose
from landmark import Landmark

DEMO = False


# Some colors constants
CRED = (0, 0, 255)
CGREEN = (0, 255, 0)
CBLUE = (255, 0, 0)
CCYAN = (255, 255, 0)
CYELLOW = (0, 255, 255)
CMAGENTA = (255, 0, 255)
CWHITE = (255, 255, 255)
CBLACK = (0, 0, 0)

# Landmarks.
# The robot knows the position of 2 landmarks. Their coordinates are in cm.
landmarks = [(0, 0), (0, 300), (400, 0), (400, 300)]
target = None # should we specify an angle for the target? Should be nil for exam!

# Exam landmarks
goals = [Landmark('L1', CRED, 'vertical'), Landmark('L2', CGREEN, 'vertical'), Landmark('L3', CGREEN, 'horizontal'), Landmark('L4', CRED, 'horizontal')]
avoid = []

port = '/dev/ttyACM0'

if DEMO:
    serialRead = serial.Serial(port,9600, timeout=1)

movestarttime = 0.0
currentmove = "0"
nextmove = "0"
waittime = 0.0

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
    
    offset = 100;
    
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
    lm0 = (landmarks[0][0]+offset, landmarks[0][1]+offset)
    lm1 = (landmarks[1][0]+offset, landmarks[1][1]+offset)
    cv2.circle(world, lm0, 5, CRED, 2)
    cv2.circle(world, lm1, 5, CGREEN, 2)
    
    # Draw estimated robot pose
    a = (int(est_pose.getX())+offset, int(est_pose.getY())+offset)
    b = (int(est_pose.getX() + 15.0*np.cos(est_pose.getTheta()))+offset, 
                                 int(est_pose.getY() + 15.0*np.sin(est_pose.getTheta()))+offset)
    cv2.circle(world, a, 5, CMAGENTA, 2)
    cv2.line(world, a, b, CMAGENTA, 2)


### Main program ###

if DEMO:
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

def stop():
    # send stop command to the adrino
    # calculate and apply the movemt to the particles
    msg = 's'
    serialRead.write(msg.encode('ascii'))
    runtime = time.time() - movestarttime
    nextmove = "0"
    waittime = time.time() + 0.4
    if currentmove == "F":
        print "F"
    elif currentmove == "H":
        print "H"
    elif currentmove == "L":
        print "L"

def movecommand (command):
    #sends command to the adrino, saves the current time and command.
    if command == "F":
        msg = 'p 10.5 1.0'
    elif command == "L":
        msg = 'p 90.0 1.0'
    elif command == "H":
        msg = 'p 270.0 1.0'
    elif command == "CCW":
        msg = 'p 120.0 1.0'
    else:
        print "WARNING: unknown move command!"

    if DEMO:
        serialRead.write(msg.encode('ascii'))

    movestarttime = time.time()
    currentmove = command

def ccw_deg_2_time(theta):
    return 1.3423 * (theta / 360.0) + 0.9646

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
    global target
    global particles

    # Detect objects
    if DEMO:
        objectType, measured_distance, measured_angle, colourProb = cam.get_object(colour)
    else:
        objectType = 'vertical'
        measured_distance = 100
        measured_angle = deg_2_rad(90.0)
        colourProb = (0,0,0)

    if objectType != 'horizontal' and objectType != 'vertical':
        print "Unknown landmark type"
        return

    if objectType != 'none':
        print "Object type =", objectType
        print "Measured distance =", measured_distance
        print "Measured angle =", measured_angle
        print "Colour probabilities =", colourProb

        """ FOR EXAM """
        for goal in goals:

            # if we find one of the goals
            if goal.match(objectType, colourProb):
                
                # calculate position of the goal
                x = np.cos(measured_angle) * measured_distance
                y = np.sin(measured_angle) * measured_distance

                # set its position
                goal.setPosition(x, y)

                # announce that we're clever enough to identify which goal and where it is :)
                print "Found", goal.getIdentifier(), "at (", goal.getX(), ",", goal.getY(), ")"

                # is it the one we want to visit?
                if goal == goals[0]:
                    target = goal

        # Compute particle weights
        sigma = 1 # testing with sigma = 1
        for p in particles:
            D = gaussian_distribution(measured_distance, p.getDistance(), sigma)
            A = gaussian_distribution(measured_angle, p.getTheta(), sigma)
            p.setWeight(D * A)

        # Resampling
        particles = resample(particles, 200)

        # Draw detected pattern
        if DEMO:
            cam.draw_object(colour)
    else:
        # No observation - reset weights to uniform distribution
        for p in particles:
            p.setWeight(1.0/num_particles)

    est_pose = estimate_pose(particles) # The estimate of the robots current pose

while True: # for exam, change to len(goals) > 0

    action = cv2.waitKey(10)
    if action == ord('q'):
	break

    """ EXAM: Robot movement """
    # XXX: Make the robot drive

    print "ROBOT ESTIMATION"
    print "X: ", est_pose.getX()
    print "Y: ", est_pose.getY()
    print "Theta: ", est_pose.getTheta()
    print ""

    # fallback when we have no target, it will try to go/rotate there
    delta = est_pose.getDeltaForTarget(Particle(0, 0, 90))

    # if we have a target
    if target is not None:

        delta = est_pose.getDeltaForTarget(target)

        # if we're within "visiting distance" of the goal
        if delta.getDistance() < 50: # visiting range is < 75cm, so 50cm should be close enough
            print "We've visited ", target.getIdentifier()
            avoid(goals.pop())
            target = None
        else:
            # if we're looking more or less directly at it
            if np.abs(delta.getTheta()) < 10:
                print "Moving toward ", target.getIdentifier()
                movecommand("F")
            # otherwise we can move toward the target
            else:
                print "Centering ", target.getIdentifier()
                movecommand("CCW") # begins to rotate counter-clockwise
                waittime = time.time() + ccw_deg_2_time(delta.getTheta())
    else:
        print "Looking for target .."
        # TODO: look for a target, strategy?
        movecommand("L")

    # Read odometry, see how far we have moved, and update particles.
    # Or use motor controls to update particles
    for particle in particles:
        s = np.sin(delta.getTheta())
        c = np.cos(delta.getTheta())
        
        x = particle.getX()
        y = particle.getY()

        particle.setX((x * c - y * s) - delta.getX())
        particle.setY((x * s + y * c) - delta.getY())

    if DEMO:
        # Fetch next frame
        colour, distorted = cam.get_colour()    
    
    if waittime < time.time():
        if nextmove == "S":
            stop()
        else:
            detect_objects()

    if DEMO:
        # Draw map
        draw_world(est_pose, particles, world)
        
        # Show frame
        cv2.imshow(WIN_RF1, colour);

        # Show world
        cv2.imshow(WIN_World, world);

# TODO: for exam, measure time and do; print "Finished in x seconds!"

# Close all windows
cv2.destroyAllWindows()
