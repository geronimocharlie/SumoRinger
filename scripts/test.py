import traci
import sumolib
import tqdm
import numpy as np
from matplotlib import pyplot as plt

sumo_binary = sumolib.checkBinary('sumo')

sumoCmd = [sumo_binary, '-c', 'sumocfg/random.sumo.cfg']
traci.start(sumoCmd)

tlights = traci.trafficlight.getIDList()
lanes = {tl: traci.trafficlight.getControlledLanes(tl) for tl in tlights}

emissions = []

for step in tqdm.tqdm(range(1000)):
	traci.simulationStep()
	emissions.append(np.sum([traci.lane.getCO2Emission(lane) for tl_lanes in lanes.values()  for lane in tl_lanes]))

plt.plot(emissions)
plt.show()

traci.close()
