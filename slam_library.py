# .This file contains helper functions for the EKF SLAM class
from math import sin, cos, pi


# .For each scan/camera-feed, returns a list of landmarks and their
# .local x, y coordinates and an estimated identification
# .signature (gate, torpedo,.. etc.).
def find_landmarks(scan):
    pass

# .from "lego_robot.py"
def scanner_to_world(pose, point):
    pass


# This function does all processing needed to obtain the cylinder observations.
# It matches the cylinders and returns distance and angle observations together
# with the cylinder coordinates in the world system, the scanner
# system, and the corresponding cylinder index (in the list of estimated parameters).
# In detail:
# - It takes scan data and detects cylinders.
# - For every detected cylinder, it computes its world coordinate using
#   the polar coordinates from the cylinder detection and the robot's pose,
#   taking into account the scanner's displacement.
# - Using the world coordinate, it finds the closest cylinder in the
#   list of current (estimated) landmarks, which are part of the current state.
#
# - If there is such a closest cylinder, the (distance, angle) pair from the
#   scan measurement (these are the two observations), the (x, y) world
#   coordinates of the cylinder as determined by the measurement, the (x, y)
#   coordinates of the same cylinder in the scanner's coordinate system,
#   and the index of the matched cylinder are added to the output list.
#   The index is the cylinder number in the robot's current state.
# - If there is no matching cylinder, the returned index will be -1.

# .Our get_observations function should obtain and analyse readings of the
# .camera or sonar in order to detect measured landmarks, covert their local
# .coordinates to global ones, compare a newly found landmark to ones in the
# .list of landmarks included in our state vector then add it to that list
# .if it doesn't already exist and then call the filter function to update
# .our robot's pose
# .from "slam_f_library.py"
def get_observations(scan, jump, min_dist, cylinder_offset,
                     robot,
                     max_cylinder_distance):

    # Compute scanner pose from robot pose.
    # .This function should be replaced by one that calculates the
    # .transform between the robot pose (coordinates) and the
    # .camera/sonar coordinates
    scanner_pose = (
        robot.state[0] + cos(robot.state[2]) * robot.scanner_displacement,
        robot.state[1] + sin(robot.state[2]) * robot.scanner_displacement,
        robot.state[2])

    # .This function should analyse our scan data (camera or sonar)
    # .and return a list of newly found landmarks (in local coordinates)
    landmarks = find_landmarks(scan)

    # .the returnable variable
    result = []

    for l in landmarks:
        # .Compute x, y of landmark in global coordinates
        # .Find the closest landmark in the state vector if any
        # .If found, compare the signature of the landmark found
        # .Append landmark to a list of found landmarks (output list),
        # .with its original measurement data, its global coordinates,
        # .its local coordinates, and the index of the matched landmark
        # .if any. If there was none matched, return an index of -1

    return 0



