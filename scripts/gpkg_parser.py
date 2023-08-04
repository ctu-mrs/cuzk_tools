import fiona
import numpy as np
from shapely.geometry import MultiLineString, MultiPolygon, Point


""" 
    Czechia SJTSK bounds are:
        XMin: -927817.5799999982
        YMin: -1236550.607999999
        XMax: -422329.256000001
        YMax: -933179.3900000006

    Split the one file containing the complete information of Czechia into rectangles similarly
    as with the DMR5G data. In fact use the exact same rectangles.

    If an object lies between multiple rectangles, assign it to only one of them.
"""

def list_gpkg_layers(gpkg_file):
    layer_names = fiona.listlayers(gpkg_file)

    """ for layer_name in layer_names:
        with fiona.open(gpkg_file, layer=layer_name) as layer:
            for feature in layer:
                pass
                #print(feature['geometry']) """
    
    for layer_name in layer_names:
        with fiona.open(gpkg_file, layer=layer_name) as layer:

            print(layer_name)
            print(len(layer))
            
            geom_type = feature['geometry']['type']
            coords = feature['geometry']['coordinates']

            for feature in layer:
                if geom_type == 'MultiLineString':
                    pass
                elif geom_type == 'MultiPolygon':
                    pass
                elif geom_type == 'Point':
                    pass
                else:
                    raise TypeError("Expected types are MultiLineString, MultiPolygon and Point. Got {} instead.".format(geom_type))


if __name__ == "__main__":
    gpkg_file_path = "src/cuzk_tools/data/data.gpkg"
    list_gpkg_layers(gpkg_file_path)