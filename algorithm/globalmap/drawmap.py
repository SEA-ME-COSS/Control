import numpy as np
import matplotlib.pyplot as plt

# color mapping for each element
number_to_color = {
    0: (1.0, 1.0, 1.0),  # Driveway
    1: (37/255, 37/255, 0),  # Solid Line
    2: (255/255, 110/255, 255/255),  # Parking Spot
    3: (218/255, 218/255, 255/255),  # Dotted Line
    4: (0, 145/255, 0),  # Stop Line
    5: (0, 109/255, 109/255),  # Crosswalk
    6: (255/255, 146/255, 146/255),  # Roundabout
}

def plot_map_from_file(file_path):
    with open(file_path, 'r') as file:
        map_data = [[int(num) for num in line.split()] for line in file]

    map_array = np.array(map_data)

    color_array = np.zeros((map_array.shape[0], map_array.shape[1], 3))

    for number, color in number_to_color.items():
        color_array[map_array == number] = color

    plt.imshow(color_array, interpolation='nearest')
    plt.axis('off')
    plt.show()


# File Path
file_path = "flipped-track.txt"
plot_map_from_file(file_path)