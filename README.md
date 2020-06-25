# SumoRinger

## Import OpenStreetMap (OSM) data
Execute the following command to download OSM data for Osnabrück. For that the environment variable `SUMO_HOME` has to be set accordingly ([tutorial](https://sumo.dlr.de/docs/Basics/Basic_Computer_Skills.html#configuring_path_settings)).
```
python $SUMO_HOME/tools/osmGet.py -b 8.0274 52.2590 8.0703 52.2873 -d path/to/output/dir/
```
After that, convert the OSM data to SUMO data using
```
netconvert --osm-files osna.osm.xml -o osna.net.xml
```
This also requires the `SUMO_HOME` environment variable to be set.

You can load the `.net.xml` file into SUMO using the "Open Network..." button or `Crtl+N`.

The `osna.osm.xml` and `osna.net.xml` files can already be found in the [`maps` directory](https://github.com/geronimocharlie/SumoRinger/tree/master/maps).
