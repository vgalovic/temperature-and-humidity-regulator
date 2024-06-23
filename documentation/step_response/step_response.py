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
num_points = 1000  # Number of points for simulation
time = np.linspace(0, total_time, num_points)  # Time vector

# Compute the step response
time, response = signal.step(P_motor, T=time)

# Save time and response coordinates to file
with open('step_response.txt', 'w') as file:
    file.write("Time Amplitude\n")
    for t, y in zip(time, response):
        file.write(f"{t:.6f} {y:.6f}\n")

