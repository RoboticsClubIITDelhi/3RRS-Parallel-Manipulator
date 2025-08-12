import math
import numpy as np
from scipy.optimize import fsolve

def angles_to_cosines(theta, phi):
    # Convert to radians
    theta_x = math.radians(theta)
    theta_y = math.radians(phi)

    # Construct two vectors lying on the plane
    # v1 lies in xz-plane, making angle theta_x with x-axis
    v1 = np.array([1, 0, math.tan(theta_x)])

    # v2 lies in yz-plane, making angle theta_y with y-axis
    v2 = np.array([0, 1, math.tan(theta_y)])

    # Normal vector is cross product of v1 and v2
    n = np.cross(v1, v2)

    # Normalize the normal vector
    norm = np.linalg.norm(n)
    direction_cosines = n / norm

    return direction_cosines  # lx, ly, lz

def normal_vector_angle_to_cosines(theta, phi):
    # Convert to radians
    alpha = math.radians(theta)
    beta = math.radians(phi)

    # Direction cosines l = cos(α), m = cos(β), n = cos(γ)
    l = math.cos(alpha)
    m = math.cos(beta)

    # Since l² + m² + n² = 1, solve for n
    l2_m2 = l**2 + m**2
    if l2_m2 > 1:
        raise ValueError("Invalid angles: cos²(θₓ) + cos²(θᵧ) > 1, cannot compute direction cosine for z-axis.")

    n = math.sqrt(1 - l2_m2)

    return [l, m, n]

from scipy.optimize import fsolve

def find_theta1_rad(l_1, l_2, l_3, l_4, l, n, height):
    def equation(theta_1_rad):
        _ = math.sqrt(l**2 + n**2)
        x_B = l_1 - l_2 * np.sin(theta_1_rad)
        z_B = -l_2 * np.cos(theta_1_rad)

        x_D = l_4 * n / _
        z_D = height + (l_4 * l / _)

        dist = np.sqrt((x_B - x_D)**2 + (z_B - z_D)**2)
        return dist - l_3  # We want this to be zero

    # Provide an initial guess, say 45 degrees in radians
    initial_guess = math.radians(45)
    theta_solution = fsolve(equation, initial_guess)[0]
    return theta_solution

def find_theta2_rad(l_1, l_2, l_3, l_4, l, m, n, height):
    root_3 = math.sqrt(3)
    def equation(theta_2_rad):
        sin_theta = np.sin(theta_2_rad)
        cos_theta = np.cos(theta_2_rad)

        # Compute B_2
        x_B = -(l_1 - l_2 * sin_theta) / 2
        y_B = root_3 * (l_1 - l_2 * sin_theta) / 2
        z_B = -l_2 * cos_theta

        # Compute D_2
        _den = np.sqrt(4 * n**2 + ((root_3 * m) - l)**2)
        x_D = -l_4 * abs(n) / _den
        y_D = root_3 * l_4 * abs(n) / _den
        z_D = height - l_4 * ((root_3 * m) - l) / _den

        # Distance between B_2 and D_2
        dist = np.sqrt((x_B - x_D)**2 + (y_B - y_D)**2 + (z_B - z_D)**2)
        return dist - l_3  # Target is l_3

    # Initial guess
    initial_guess = math.radians(45)
    theta_solution = fsolve(equation, initial_guess)[0]
    return theta_solution

def find_theta3_rad(l_1, l_2, l_3, l_4, l, m, n, height):
    root_3 = math.sqrt(3)
    def equation(theta_2_rad):  # Note: still theta_2_rad due to symmetry
        sin_theta = np.sin(theta_2_rad)
        cos_theta = np.cos(theta_2_rad)

        # Compute B_3
        x_B = -(l_1 - l_2 * sin_theta) / 2
        y_B = -root_3 * (l_1 - l_2 * sin_theta) / 2
        z_B = -l_2 * cos_theta

        # Compute D_3
        _den = np.sqrt(4 * n**2 + (root_3 * m + l)**2)
        x_D = -l_4 * abs(n) / _den
        y_D = -root_3 * l_4 * abs(n) / _den
        z_D = height + l_4 * (root_3 * m + l) / _den

        # Distance between B_3 and D_3
        dist = np.sqrt((x_B - x_D)**2 + (y_B - y_D)**2 + (z_B - z_D)**2)
        return dist - l_3  # Target is l_3

    # Initial guess
    initial_guess = math.radians(45)
    theta_solution = fsolve(equation, initial_guess)[0]
    return theta_solution

def inv_kin(height, theta, phi, mode = angles_to_cosines):
    l_1 = 32.33 # base triangle circumradius
    l_2 = 10.0 # link 1 length
    l_3 = 16.0 # link 2 length
    l_4 = 27.71 # end effector triangle circumradius
    root_3 = math.sqrt(3)

    [l, m, n] = angles_to_cosines(theta, phi)
    theta_1_rad = 0.0
    theta_2_rad = 0.0
    theta_3_rad = 0.0

    # X-axis defined in direction of Motor 1
    # Y-axis perpendicular to X-axis in the plane 

    origin = [0,0,0] #circumcentre of the triangle formed by motor revolute joints
    C = [0,0,height] #circumcentre of the triangle formed by end effector
    
    # A point refers to the location of the motor joint
    # B point refers to the location of the revolute joint
    # D point refers to the location of the spherical joint 

    A_1 = [l_1, 0, 0]
    B_1 = [l_1 - (l_2*math.sin(theta_1_rad)), 0, -l_2*math.cos(theta_1_rad)]
    _ = math.sqrt(l**2+n**2)
    D_1 = [l_4*n/_, 0, height + (l_4*l/_)]
    theta_1_rad = find_theta1_rad(l_1, l_2, l_3, l_4, l, n, height)


    A_2 = [-l_1/2, root_3*l_1/2, 0]
    B_2 = [-(l_1 - (l_2*math.sin(theta_2_rad)))/2, root_3*(l_1 - (l_2*math.sin(theta_2_rad)))/2, -l_2*math.cos(theta_2_rad)]
    _ = math.sqrt((4*n**2) + ((root_3*m) - l)**2)
    D_2 = [-l_4*abs(n)/_, root_3*l_4*abs(n)/_, height - (l_4*((root_3*m)-l)/_)]
    theta_2_rad = find_theta2_rad(l_1, l_2, l_3, l_4, l, m, n, height)

    A_3 = [-l_1/2, -root_3*l_1/2, 0]
    B_3 = [-(l_1 - (l_2*math.sin(theta_2_rad)))/2, -root_3*(l_1 - (l_2*math.sin(theta_2_rad)))/2, -l_2*math.cos(theta_2_rad)]
    _ = math.sqrt((4*n**2) + ((root_3*m) + l)**2)
    D_3 = [-l_4*abs(n)/_, -root_3*l_4*abs(n)/_, height + (l_4*((root_3*m)+l)/_)]
    theta_3_rad = find_theta3_rad(l_1, l_2, l_3, l_4, l, m, n, height)
    
    # Convert to degrees
    theta_1 = math.degrees(theta_1_rad)
    theta_2 = math.degrees(theta_2_rad)
    theta_3 = math.degrees(theta_3_rad)

    # Adjust angles to lie between 0 and 180, reflecting if negative
    def adjust_angle(angle):
        if angle < 0:
            angle = 180 - abs(angle)
        return min(angle, 180)  # clamp in case it's >180 due to overshoot

    init_theta_1 = 13
    init_theta_2 = 13
    init_theta_3 = 13
    theta_1 -= init_theta_1
    theta_2 -= init_theta_2
    theta_3 -= init_theta_3
    return theta_1, theta_2, theta_3


# Constants
init_height = 6.0
init_theta = 0.0
init_phi = 0.0

# Input theta with validation (-45° to +45°)
while True:
    try:
        theta = float(input("Enter theta (angle about X-axis) in degrees [-45 to 45]: "))
        if -45 <= theta <= 45:
            break
        else:
            print("❌ Theta must be between -45 and 45 degrees.")
    except ValueError:
        print("❌ Please enter a valid number for theta.")

# Input phi with validation (-45° to +45°)
while True:
    try:
        phi = float(input("Enter phi (angle about Y-axis) in degrees [-45 to 45]: "))
        if -45 <= phi <= 45:
            break
        else:
            print("❌ Phi must be between -45 and 45 degrees.")
    except ValueError:
        print("❌ Please enter a valid number for phi.")


# Apply initial offsets
height = init_height
theta += init_theta
phi += init_phi

print("Final Input Values:")
print(f"Theta: {theta:.2f} degrees")
print(f"Phi: {phi:.2f} degrees")

theta_1, theta_2, theta_3 = inv_kin(height, theta, phi)
while True:
    if theta_1<0 or theta_2<0 or theta_3<0 or theta_1>160 or theta_2>160 or theta_3>160:
        height+=1
        theta_1, theta_2, theta_3 = inv_kin(height, theta, phi)
    elif height>22:
        print("No valid solution exists")
        break
    else:
        print(f"Height: {height} cm")
        print(f"Motor Angles:\n  theta_1: {theta_1:.2f}\n  theta_2: {theta_2:.2f}\n  theta_3: {theta_3:.2f}")
        break
