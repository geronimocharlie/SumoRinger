import os
import traci
import sumolib
import time
import pickle
import subprocess
import numpy as np
from joblib import Parallel, delayed
from matplotlib import pyplot as plt

# define hyperparameters
n_steps = 2000
model_file = 'model-23-3.87.pkl' # name of a file in the models directory

# the simulation config file
cfg_name = 'martini.sumocfg'

# define SUMO commands for starting the simulation
sumo_binary = sumolib.checkBinary('sumo')
sumo_binary_gui = sumolib.checkBinary('sumo-gui')
sumo_cmd = ['-c', os.path.join('sumo_data', cfg_name), '--quit-on-end', '--start']

def start_sumo(binary):
	'''
	Instantiates a SUMO simulation instance and return a connection object.

	Args:
		binary: path to the SUMO binary file (path to "sumo" or "sumo-gui")
	'''
	port = sumolib.miscutils.getFreeSocketPort()
	# start the simulation and pipe its outputs into /dev/null
	sumoProc = subprocess.Popen([binary] + sumo_cmd + ['--remote-port', str(port)],
								stdout=open(os.devnull, 'w'),
								stderr=open(os.devnull, 'w'))
	return traci.connect(port)

def get_durations(conn):
	'''
	Grabs the phase durations for all traffic light systems in the road network.

	Args:
		conn: traci connection object
	'''
	durations = []
	for tl in tlights:
		definitions = conn.trafficlight.getCompleteRedYellowGreenDefinition(tl)
		for definition in definitions:
			for phase in definition.phases:
				durations.append(phase.minDur)
	return durations

def get_states(conn):
	'''
	Grabs the state definitions for all traffic light systems in the road network.

	Args:
		conn: traci connection object
	'''
	states = []
	for tl in tlights:
		definitions = conn.trafficlight.getCompleteRedYellowGreenDefinition(tl)
		for definition in definitions:
			for phase in definition.phases:
				states.append(phase.state)
	return states

def set_genome(durations, states, conn):
	'''
	Set the genome (traffic light phase durations and states) for the current simulation.

	Args:
		durations: a list of durations for all traffic light phases in the simulation
		states: a list of state strings for all traffic light phases in the simulation
		conn: traci connection object
	'''
	idx = 0
	# iterate over all traffic light systems in the simulation
	for tl in tlights:
		definition = conn.trafficlight.getCompleteRedYellowGreenDefinition(tl)[0]
		for phase in definition.phases:
			# set the current phase's duration and state
			phase.minDur = phase.maxDur = durations[idx]
			phase.state = states[idx]
			idx += 1
		# replace the durations and phases in the simulation instance
		logic = conn.trafficlight.Logic(conn.trafficlight.getProgram(tl), 0, 0, phases=definition.phases)
		conn.trafficlight.setCompleteRedYellowGreenDefinition(tl, logic)

# extract traffic light information from the road network
conn = start_sumo(sumo_binary)
tlights = conn.trafficlight.getIDList()
lanes = {tl: conn.trafficlight.getControlledLanes(tl) for tl in tlights}
n_links = {tl: len(conn.trafficlight.getControlledLinks(tl)) for tl in tlights}

with open(os.path.join('models', model_file), 'rb') as file:
	durs_loaded, states_loaded = pickle.load(file)
print(f'Finished loading the model file "{model_file}"')
conn.close()

# initialize
conn = start_sumo(sumo_binary_gui)
set_genome(durs_loaded, states_loaded, conn)
# run simulation
for step in range(n_steps):
	conn.simulationStep()
	time.sleep(50 / 1000)
conn.close()
