from PIL import Image
import numpy as np
from rtree import index
from scipy.spatial import cKDTree

def coords2pixel(coords, im, im_bounds):
    w, h = im.size
    w_step = (im_bounds[2] - im_bounds[0])/w
    h_step = (im_bounds[3] - im_bounds[1])/h

    pix_coords = np.empty(coords.shape)

    pix_coords[:,1] = coords[:,0] - im_bounds[0]
    pix_coords[:,0] = - coords[:,1] + im_bounds[1]

    pix_coords[:,0] /= w_step
    pix_coords[:,1] /= h_step

    pix_coords = pix_coords.astype(np.int)

    return pix_coords

def find_nearest_pixel_color(im, im_array, im_bounds, coords):
    pix_coords = coords2pixel(coords ,im, im_bounds)

    pix_rgb = im_array[pix_coords[:,0], pix_coords[:,1]]

    return pix_rgb

def img2rgb(im_path, im_bounds, coords):
    im = Image.open(im_path)
    im = im.convert('RGB')
    im_array = np.array(im)

    rgb_colors = find_nearest_pixel_color(im, im_array, im_bounds, coords) 

    return rgb_colors

# Example usage
if __name__ == "__main__":
    im_path = '/home/aherold/ws/src/cuzk_tools/images/orto.png'
    coords = np.array([[1000,2000],[1500,3000],[3200,1000]])
    im_bounds = [500,500,5500,5500]

    rgb_colors = img2rgb(im_path, im_bounds, coords)
    print(rgb_colors)
