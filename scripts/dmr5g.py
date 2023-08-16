import urllib.request
import xml.etree.ElementTree as ET
from shapely.geometry import Polygon, MultiPolygon, Point
from rtree import index
from urllib.request import urlretrieve
import zipfile
import pylas
import matplotlib.pyplot as plt
import pyproj
import os
import numpy as np

SJTSK = "EPSG:5514"
WGS = "EPSG:4326"
UTM_33U = "EPSG:32633"

WGS_TO_SJTSK = pyproj.Transformer.from_crs(WGS,SJTSK)
SJTSK_TO_WGS = pyproj.Transformer.from_crs(SJTSK,WGS)
SJTSK_TO_UTM_33U = pyproj.Transformer.from_crs(SJTSK,UTM_33U)

def get_sjtsk_to_utm_trans(utm_letter, utm_num):
    if utm_letter == "N":
        utm = "EPSG:326" + str(utm_num)
    elif utm_letter == "S":
        utm = "EPSG:327" + str(utm_num)
    else:
        raise UTMZoneError("utm letter '{}' is not one of (N,S).".format(utm_letter, utm_num))
    return pyproj.Transformer.from_crs(SJTSK,utm)

class UTMZoneError(Exception):
    pass

class PointOutOfTileError(Exception):
    pass

class Circle():
    def __init__(self,center,radius):
        self.x = center.x
        self.y = center.y
        self.r = radius

class Rectangle():
    def __init__(self,points):
        l = points[0][0]
        b = points[1][0]
        r = points[0][1]
        t = points[1][2]
        self.x = (l+r)/2
        self.y = (b+t)/2
        self.w = r-l
        self.h = t-b
 

class Dmr5gParser():
    def __init__(self, cache_dir, fetch_xml=False) -> None:
        self.cache_dir = cache_dir
        self.xml_url = "https://atom.cuzk.cz/DMR5G-SJTSK/DMR5G-SJTSK.xml"
        self.xml_fn = self.cache_dir + "DMR5G-SJTSK.xml"
        
        if fetch_xml or not os.path.exists(self.xml_fn):
            _, self.xml_data = self.open_url(self.xml_url)
            self.root = ET.fromstring(self.xml_data)
        else:
            tree = ET.parse(self.xml_fn)
            self.root = tree.getroot()

        self.namespace = {
            'atom': 'http://www.w3.org/2005/Atom',
            'georss': 'http://www.georss.org/georss'
        }

        self.n = len(self.root.findall(f"{{{self.namespace['atom']}}}entry"))

        self.tile_idx, self.tile_polygons = self.get_tiles()


    def open_url(self, url):
        response = urllib.request.urlopen(url)
        xml_data = response.read().decode('utf-8')
        
        return response, xml_data

    def get_tiles(self):
        polygons = [((0.,0.),(0.,0.),(0.,0.),(0.,0.),(0.,0.))] * self.n
        i = 0

        idx = index.Index()

        # Access elements and extract data
        for entry in self.root.iter(f"{{{self.namespace['atom']}}}entry"):
            
            polygon = entry.find(f"{{{self.namespace['georss']}}}polygon")

            # Split the string by spaces to get individual float values
            float_list = polygon.text.split()

            # Convert the list of strings to a list of floats
            float_list = [float(value) for value in float_list]

            # Create a tuple of pairs of floats
            pairs = tuple((float_list[i], float_list[i + 1]) for i in range(0, len(float_list), 2))

            polygons[i] = pairs
            
            left, bottom, right, top = (float_list[1], float_list[0], float_list[3], float_list[4])
            idx.insert(i, (left, bottom, right, top))

            i += 1

        return idx,polygons
    
    def get_intersection_tile_ids(self, point):
        tile_ids = list(self.tile_idx.intersection(point))
        return tile_ids
    
    def get_tile(self, id):
        if id >= self.n:
            raise IndexError("id {} out of bounds of polygons list of len {}".format(id, self.n))

        return self.tile_polygons[id]
    
    def fix_tile_coords(self,tile):
        x,y = np.array(tile.exterior.coords.xy)
        
        x_fixed = np.copy(x)
        y_fixed = np.copy(y)

        x_mask = x%2500 < 1250
        
        x_fixed[x_mask] -= (x%2500)[x_mask]
        x_fixed[~x_mask] += 2500-(x%2500)[~x_mask]

        y_mask = y%2000 < 1000
        
        y_fixed[y_mask] -= (y%2000)[y_mask]
        y_fixed[~y_mask] += 2000-(y%2000)[~y_mask]

        return Polygon(zip(x_fixed,y_fixed))
    
    def get_tile_id(self,point):
        ids = self.get_intersection_tile_ids(point)

        point_sjtsk = Point(WGS_TO_SJTSK.transform(point[1], point[0]))


        if len(ids) > 1:
            for id in ids:
                tile = self.get_tile(id)
                tile_sjtsk = Polygon(np.array(WGS_TO_SJTSK.transform(np.array(tile)[:,0], np.array(tile)[:,1])).T)
                tile_sjtsk_fixed = self.fix_tile_coords(tile_sjtsk)
                
                if tile_sjtsk_fixed.contains(point_sjtsk):
                    return id
                else:
                    pass

            raise PointOutOfTileError("None of selected tiles contains the point {}.".format(point))
        
        elif len(ids) == 1:
            id = ids[0]
            tile = self.get_tile(id)
            tile_sjtsk = Polygon(np.array(WGS_TO_SJTSK.transform(np.array(tile)[:,0], np.array(tile)[:,1])).T)
            tile_sjtsk_fixed = self.fix_tile_coords(tile_sjtsk)
            
            if not tile_sjtsk_fixed.contains(point_sjtsk):
                raise PointOutOfTileError("Only one tile selected and it is not containing point {}.".format(point))
            else:
                return id
        else:
            raise PointOutOfTileError("No tile found which could contain the point {}.".format(point))
        
    def c_r_intersects(self, circle, rect):
        # https://stackoverflow.com/questions/401847/circle-rectangle-collision-detection-intersection
        circleDistanceX = abs(circle.x - rect.x)
        circleDistanceY = abs(circle.y - rect.y)

        if circleDistanceX > (rect.w / 2 + circle.r):
            return False
        if circleDistanceY > (rect.h / 2 + circle.r):
            return False

        if circleDistanceX <= (rect.w / 2):
            return True
        if circleDistanceY <= (rect.h / 2):
            return True

        cornerDistance_sq = (circleDistanceX - rect.w / 2) ** 2 + (circleDistanceY - rect.h / 2) ** 2

        return cornerDistance_sq <= (circle.r ** 2)

        
    def get_tile_ids(self, point, radius):        
        
        necessary_num_near_tiles = int((2+2*np.floor((1.1+np.floor(radius/1000))/2))**2)
        tile_ids = list(self.tile_idx.nearest(point, necessary_num_near_tiles))

        point_sjtsk = Point(WGS_TO_SJTSK.transform(point[1], point[0]))

        ids = []

        for id in tile_ids:
            tile = self.get_tile(id)
            tile_sjtsk = Polygon(np.array(WGS_TO_SJTSK.transform(np.array(tile)[:,0], np.array(tile)[:,1])).T)
            tile_sjtsk_fixed = self.fix_tile_coords(tile_sjtsk)

            if self.c_r_intersects(Circle(point_sjtsk, radius), Rectangle(tile_sjtsk_fixed.exterior.coords.xy)):
                ids.append(id)

        return ids
        
    def get_tile_code(self,id):
        tile_url = self.get_tile_xml(id)
        tile_id_index = tile_url.find("CUZK_DMR5G-SJTSK_") + len("CUZK_DMR5G-SJTSK_")
        tile_id = tile_url[tile_id_index:-4]

        return tile_id

    def get_tile_xml(self,id):
        for i,entry in enumerate(self.root.iter(f"{{{self.namespace['atom']}}}entry")):
            if i == id:
                tile_url = entry.find(f"{{{self.namespace['atom']}}}id").text
                return tile_url
            
    def get_tile_zip(self,id):
        tile_url = self.get_tile_xml(id)
        response, tile_xml_data = self.open_url(tile_url)
        tile_root = ET.fromstring(tile_xml_data)
        entry = tile_root.find(f"{{{self.namespace['atom']}}}entry")
        zip_url = entry.find(f"{{{self.namespace['atom']}}}id").text
        
        return zip_url
    
    def get_tile_update_date(self,id):
        for i,entry in enumerate(self.root.iter(f"{{{self.namespace['atom']}}}entry")):
            if i == id:
                update_date = entry.find(f"{{{self.namespace['atom']}}}updated").text
                return update_date
            
    def download_tile(self,id):
        #id = self.get_tile_id(point)
        tile_update_date = self.get_tile_update_date(id)
        tile_zip = self.get_tile_zip(id)
        tile_code = self.get_tile_code(id)
        tile_zip_fn = self.cache_dir + tile_code + ".zip"
        tile_laz_fn = self.cache_dir + tile_code + ".laz"

        urlretrieve(tile_zip, tile_zip_fn)
        
        with zipfile.ZipFile(tile_zip_fn, 'r') as zip_ref:
            file_name = zip_ref.namelist()[0]
            zip_ref.extract(file_name,self.cache_dir)
            os.rename(self.cache_dir + file_name, tile_laz_fn)

        return tile_laz_fn
    
    def get_tile_data(self, fn):
        with pylas.open(self.cache_dir + fn) as f:
            las = f.read()

        pcd_data = np.zeros(f.header.point_count, dtype=[
            ('x', np.float64),
            ('y', np.float64),
            ('z', np.float64)])
        
        pcd_data['x'] = las.x#(las.x - np.mean(las.x))/np.std(las.x)
        pcd_data['y'] = las.y#(las.y - np.mean(las.y))/np.std(las.y)
        pcd_data['z'] = las.z#(las.z - np.mean(las.z))/np.std(las.z)

        return pcd_data

    def visualize_laz(self,fn):
        with pylas.open(fn) as f:
            print('Num points:', f.header.point_count)
            las = f.read()
            x = las.x
            y = las.y
            z = las.z

            print(max(x),min(x))
            print(max(y),min(y))

            plt.figure()
            plt.scatter(x, y, s=1, c=z, cmap='viridis')
            plt.colorbar(label='Elevation')
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.title('Point Cloud Visualization')
            plt.show()




