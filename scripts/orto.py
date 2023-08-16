import requests
import time
from copy import deepcopy
import zipfile
from urllib.parse import urlencode
from urllib.request import urlretrieve
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.image as mpimg


base_url = "https://ags.cuzk.cz/arcgis1/rest/services/ORTOFOTO/MapServer/export"

arg_names = [
"bbox",
"bboxSR",
"layers",
"layerDefs",
"size",
"imageSR",
"historicMoment",
"format",
"transparent",
"dpi",
"time",
"timeRelation",
"layerTimeOptions",
"dynamicLayers",
"gdbVersion",
"mapScale",
"rotation",
"datumTransformations",
"layerParameterValues",
"mapRangeValues",
"layerRangeValues",
"clipping",
"spatialFilter",	
"f"]

arg_vals = [
"-743189,-1044300,-743089,-1044200",
"",
"",
"",
"400,400",
"",
"",
"png",
"false",
"",
"",
"esriTimeRelationOverlaps",
"",
"",
"",
"",
"",
"",
"",
"",
"",
"",
"",
"json",
]

def get_img(coords):
    global base_url, arg_names, arg_vals

    arg_vals_updated = deepcopy(arg_vals)
    arg_vals_updated[arg_names.index("bbox")] = ','.join(list(map(str, coords)))

    args = dict()
    for i in range(len(arg_names)):
        args[arg_names[i]] = arg_vals_updated[i]

    encoded_params = urlencode(args)
    full_url = f"{base_url}?{encoded_params}"

    print(full_url)

    response = requests.get(full_url)
    response = response.json()

    png_url = response['href']

    png_path = '/home/aherold/ws/src/cuzk_tools/images/orto.png'

    start_t = time.time()
    while True:
        try:
            urlretrieve(png_url, png_path)
            print("T = {} s: Job's done.".format(int(time.time() - start_t)))
            break
        except:
            print("T = {} s: Job's not done yet.".format(int(time.time() - start_t)))
            time.sleep(0.5)

        if time.time() - start_t >= 10:
            raise TimeoutError("Job did not get done in under 10 s. Terminating.")

    return png_path

def plot_image(path):
    # Load the PNG image using matplotlib's imread function
    image = mpimg.imread(path)

    # Create a figure and axis
    fig, ax = plt.subplots()

    # Display the image using imshow
    ax.imshow(image)

    # Hide the axis ticks and labels
    ax.axis('off')

    # Show the image
    plt.show()


if __name__ == "__main__":
    plot_image(get_img([-774250.24, -1048609.67, -773950.24, -1048309.67]))