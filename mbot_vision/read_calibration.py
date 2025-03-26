import numpy as np

# Load the .npz file
data = np.load('cam_calibration_data.npz')

# Display the list of arrays (keys) inside the .npz file
print("Arrays in the .npz file:", data.files)

# Access each array by name
for array_name in data.files:
    array_data = data[array_name]  # Get the array associated with the name
    print(f"Data for {array_name}:")
    print(array_data)

# Close the .npz file after use
data.close()
