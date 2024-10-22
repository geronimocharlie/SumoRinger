# SumoRinger

## Code
Our code can be found in the [scripts directory](https://github.com/geronimocharlie/SumoRinger/tree/master/scripts). In the file `genetic_algorithm.py`, we have implemented a bio-inspired algorithm called Genetic Algorithm, which mimics survival of the fittest in an evolutionary setting. We use this to optimize both durations of traffic light phases, as well as the traffic light states at each intersection. Additionally, we provide tools for plotting the results and visualizing the traffic flow on Open Street Map data ([`plotting.py`](https://github.com/geronimocharlie/SumoRinger/blob/master/scripts/plotting.py), [`run_simulation.py`](https://github.com/geronimocharlie/SumoRinger/blob/master/scripts/run_simulation.py)).

## Generate random trips
Given a `.net.xml` (road network) file, we can generate random trips using
```
python $SUMO_HOME/tools/randomTrips.py -n sumo_data/maps/simple.net.xml -o sumo_data/trips/simple.trips.xml --route-file sumo_data/routes/simple.rou.xml -p 20 -v
```
This will generate a `trips.xml` and `rou.xml` file with random trips. To run a simulation with these trips, insert the file names for the road network file (`.net.xml`) and routes (`.rou.xml`) into the SUMO config file (`.sumo.cfg`).

## Import OpenStreetMap (OSM) data
Execute the following command to download OSM data for Osnabrück. For that the environment variable `SUMO_HOME` has to be set accordingly ([tutorial](https://sumo.dlr.de/docs/Basics/Basic_Computer_Skills.html#configuring_path_settings)).
```
python $SUMO_HOME/tools/osmGet.py -b 8.0274,52.2590,8.0703,52.2873 -d path/to/output/dir/
```
After that, convert the OSM data to SUMO data using
```
netconvert --osm-files osna.osm.xml -o osna.net.xml
```
This also requires the `SUMO_HOME` environment variable to be set.
You can load the `.net.xml` file into SUMO using the "Open Network..." button or `Crtl+N`.

The `osna.osm.xml` and `osna.net.xml` files can already be found in the [`maps` directory](https://github.com/geronimocharlie/SumoRinger/tree/master/maps).

## Sources
[SUMO](https://www.eclipse.org/sumo/) - traffic simulation software

[Optimization of Traffic Signal Light Timing Using Simulation](https://www.researchgate.net/publication/221524934_Optimization_of_Traffic_Signal_Light_Timing_Using_Simulation) - Similar work using SUMO
