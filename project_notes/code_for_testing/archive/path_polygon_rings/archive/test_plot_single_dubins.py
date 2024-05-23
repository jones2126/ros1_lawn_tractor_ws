# python3 /home/tractor/ros1_lawn_tractor_ws/project_notes/code_for_testing/archive/path_polygon_rings/test_plot_single_dubins.py
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import minimize

# Data in x, y format
data = [
    [19.9997435329616, 0.1999999747017954],
    [20.0, 0.2],
    [20.29734435921107, 0.16553796151887862],
    [20.57892396034633, 0.06397897237747466],
    [20.829809869482975, -0.0992924600520656],
    [21.036700486844026, -0.31561992594319704],
    [21.239460037045745, -0.5358415920787252],
    [21.487136348811447, -0.703941992557295],
    [21.76669594925137, -0.810936057125313],
    [22.063317001891477, -0.8511511187514023],
    [22.361273097300394, -0.822455034356673],
    [22.644767044054014, -0.7263692278867686],
    [22.89876841245666, -0.5679880266648518],
    [23.10981042568965, -0.3557085676927622],
    [23.266703948397332, -0.10078559386872749],
    [23.36113071809203, 0.18326525583241338],
    [23.388084367126257, 0.4813840255445122],
    [23.34613585289967, 0.7777648990925488],
    [23.23750922358381, 1.0566942009853229],
    [23.067963702377995, 1.3033835122453856],
    [22.84648834201468, 1.5047537305073135],
    [22.584825438480735, 1.6501285046777305],
    [22.296847971854103, 1.7318002792771823],
    [21.997824081421843, 1.7454389376301802],
    [21.703607571520642, 1.6903213782459674],
    [21.429797366282802, 1.5693698525899038],
    [21.190910477753608, 1.388997031633543],
    [20.99567353786622, 1.1616947657941823],
    [20.77759166838075, 0.956654286247731],
    [20.51839281103003, 0.8069303656505316],
    [20.23181930580834, 0.7204611447920679],
    [20.0, 0.7],
    [19.700000000875303, 0.700001599622784]
]

# Convert data to numpy array for ease of calculation
data_np = np.array(data)

# Function to calculate the distance from a point to the circle center
def distance_from_circle_center(center, points):
    x0, y0 = center
    distances = np.sqrt((points[:, 0] - x0) ** 2 + (points[:, 1] - y0) ** 2)
    return distances

# Function to calculate the mean squared error between distances
def mean_squared_error(center, points):
    distances = distance_from_circle_center(center, points)
    mean_distance = np.mean(distances)
    return np.mean((distances - mean_distance) ** 2)

# Initial guess for circle center
initial_guess = [np.mean(data_np[:, 0]), np.mean(data_np[:, 1])]

# Minimize the mean squared error to find the circle center
result = minimize(mean_squared_error, initial_guess, args=(data_np,))

# Extract the circle center and calculate the mean radius
circle_center = result.x
mean_radius = np.mean(distance_from_circle_center(circle_center, data_np))

# Last point in the data set
last_point = data_np[-1]
print(f"last point: {last_point}")

# Calculate the angle using atan2 (y2-y1, x2-x1)
# Here, (x1, y1) is the circle center and (x2, y2) is the last point
angle = np.arctan2(last_point[1] - circle_center[1], last_point[0] - circle_center[0])

# Ensure the angle is within the range [0, 2*pi)
if angle < 0:
    angle += 2 * np.pi

print(f"the angle of the last point: {angle}")

# Plotting
plt.figure(figsize=(10, 8))
plt.plot(data_np[:, 0], data_np[:, 1], 'o-', label="Path")
plt.plot(circle_center[0], circle_center[1], 'ro', label="Circle Center")
circle = plt.Circle(circle_center, mean_radius, color='r', fill=False, linestyle='--')
plt.gca().add_artist(circle)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Dubins Path with Circle Center')
plt.legend()
plt.grid(True)
plt.show()

print(circle_center, mean_radius)
