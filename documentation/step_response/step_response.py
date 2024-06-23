import numpy as np
from scipy import signal

# System parameters
V = 4.5
J = 0.000000807
b = 0.000000414
K = 0.0059
R = 1.72
L = 0.000106

# Transfer function numerator and denominator coefficients
numerator = [V * K]
denominator = [J * L, (J * R + b * L), (b * R + K**2)]

# Create the transfer function
P_motor = signal.TransferFunction(numerator, denominator)

# Time vector for simulation
total_time = 0.25  # Total simulation time in seconds
num_points = 10000  # Number of points for simulation

# Compute the step response
time = np.linspace(0, total_time, num_points)
time, response = signal.step(P_motor, T=time)

# Calculate settling time to 2% tolerance
final_value = response[-1]  # Final value of the step response
tolerance = 0.02 * final_value  # 2% of the final value
settling_time = None

# Find the index where the response is within the tolerance
for idx, y in enumerate(response):
    if abs(y - final_value) < tolerance:
        settling_time = time[idx]
        break

# Find the coordinates around settling time
settling_idx = np.abs(time - settling_time).argmin()
settling_time_coord = time[settling_idx]
settling_response_coord = response[settling_idx]

# Save time and response coordinates to file
with open('step_response.txt', 'w') as file:
    file.write("Time Amplitude\n")
    for t, y in zip(time, response):
        file.write(f"{t:.6f} {y:.6f}\n")

# Print the calculated settling time and its corresponding coordinates
print(f"Settling time to 2% tolerance: {settling_time:.4f} seconds")
print(f"Settling time coordinate: ({settling_time_coord:.4f}, {settling_response_coord:.4f})")

