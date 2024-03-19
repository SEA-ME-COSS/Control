import numpy as np
from PIL import Image

color_mapping = {
    (0, 0, 0, 255): 0
}

def map_colors_to_numbers(image_path, output_file):
    img = Image.open(image_path)
    img_data = np.array(img)
    print(img_data.shape)

    result = np.zeros(img_data.shape[:2], dtype=int)

    for y in range(img_data.shape[0]):
        for x in range(img_data.shape[1]):
            pixel = tuple(img_data[y, x])
            result[y, x] = color_mapping.get(pixel, 1)

    with open(output_file, 'w') as file:
        for row in result:
            file.write(' '.join(map(str, row)) + '\n')

image_path = "test-track.bmp"
output_file = "test-track.txt"
map_colors_to_numbers(image_path, output_file)
