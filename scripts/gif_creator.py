"""Script to create a gif from a list of images."""

import os
import sys
import imageio


def create_gif(image_list, gif_name):
    """Create a gif from a list of images."""
    images = []
    for file_name in image_list:
        images.append(imageio.imread(file_name))
    imageio.mimsave(gif_name, images, duration=0.1)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python gif_creator.py <image_directory> <gif_name>")
        exit(1)

    image_directory = sys.argv[1]
    gif_name = sys.argv[2]

    image_list = [
        os.path.join(image_directory, f)
        for f in os.listdir(image_directory)
        if f.endswith(".png")
    ]
    image_list.sort(key=lambda x: int(x.split("/")[-1].split(".")[0]))

    create_gif(image_list, gif_name)
