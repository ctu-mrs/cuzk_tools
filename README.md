# CUZK-TOOLS

## Description
The ROS package **cuzk_tools** contains ROS nodes providing some services using open data from the [Czech State Administration of Land Surveying and Cadastre (**CUZK**)](https://geoportal.cuzk.cz/(S(1bww4u03zr1k4oogfysuwvpu))/Default.aspx?lng=EN&head_tab=sekce-02-gp&mode=TextMeta&text=dSady_uvod&menu=20&news=yes).

Here is an overview of the provided functionalities:
 - node: **elevation**
    - service: **elevation**
    - topic: **elevation_pcd**
 - node: **topography**
    - service: **topography**
    - topics: **topography/*category***, where *category* is one of the following:
        - buildings
        - roads
        - rails
        - footways
        - water
        - forest
        - antiforest
        - agriculture
        - untraversable
        - traversable
        - obstacles

## Nodes
### Elevation
#### Data

This node uses the [DMR 5G data](https://geoportal.cuzk.cz/(S(5tcbldqpcpjqzpc4gvmqdw5a))/Default.aspx?lng=EN&mode=TextMeta&side=vyskopis&metadataID=CZ-CUZK-DMR5G-V&mapid=8&menu=302), which is a digital elevation model of the Czech Republic. The data is in the form of 3D points with the x and y coordinates given in the [S-JTSK coordinate system](https://cs.wikipedia.org/wiki/Syst%C3%A9m_jednotn%C3%A9_trigonometrick%C3%A9_s%C3%ADt%C4%9B_katastr%C3%A1ln%C3%AD) ([EPSG:5514](https://epsg.io/5514)) and the z coordinate representing the altitude in the Balt height reference system after levelling ([Bpv](https://cs.wikipedia.org/wiki/Baltsk%C3%BD_po_vyrovn%C3%A1n%C3%AD)).

The stated mean height error of DMR 5G is 0.18 m in exposed terrain and 0.3 m in forested terrain. The data itself does not contain any additional information regarding the error of individual points.

The DMR 5G data is stored in the form of 2.5 km wide and 2 km high tiles. These tiles are freely available for download for example by [zooming on an area in a map](https://ags.cuzk.cz/geoprohlizec/?atom=dmr5g) or choosing from an [XML file](https://atom.cuzk.cz/DMR5G-SJTSK/DMR5G-SJTSK.xml) more suited for machine access.

The latter is also used in the elevation service provided by this node.

#### Workflow

1. Launch the node with:
`roslaunch cuzk_tools elevation.launch`

2. Call the elevation service:
```
rosservice call /elevation "point:
  x: 15.740
  y: 50.736
  z: 0.0
radius:
  data: 1000.0"
```
where **x** is the **longitude**, **y** is the **latitude** (WGS84), z is arbitrary and **radius** is given in **meters**.

3. The node then calculates which tiles are needed and either retrieves them from cache or downloads them. Afterwards it filters out points outside the given radius.

4. The elevation data is then published to the **elevation_pcd** topic as [PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) and can be viewed in rviz.


### Topography
#### Data
This node uses the The Fundamental Base of Geographic Data of the Czech Republic ([ZABAGED](https://geoportal.cuzk.cz/(S(1bww4u03zr1k4oogfysuwvpu))/Default.aspx?lng=EN&mode=TextMeta&text=dSady_zabaged&side=zabaged&menu=24)). The data describes objects in a vectorized form either as a Point, (Multi)LineString or (Multi)Polygon. These objects are split into 139 categories such as 'Building, block of buildings', 'Road, motorway', 'Railway line', 'Castle' etc. Each object is usually listed in only one category, so for example the Hněvín Castle is a 'Castle' but not a 'Building, block of buildings'.

The data is stored in a single file, containing the entirety of Czechia (with the size of 12.3 GB).
#### Workflow