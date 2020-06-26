import os
import traci
import sumolib
import time
import subprocess
import numpy as np
from joblib import Parallel, delayed
from matplotlib import pyplot as plt

# define hyperparameters
pop_size = 10
n_steps = 2000
n_show_steps = 500
show_its = 10
random_init = True
show_only_min = False
duration_mutation_rate = 0.3
duration_mutation_strength = 15
states_mutation_rate = 0.2
collision_penalty = 10
n_jobs = -1

# the simulation config file
cfg_name = 'martini.sumocfg'

# weighting factors for CO2 emissions and vehicle waiting timess
emissions_weight = 1 / 32_000_000
waiting_weight = 1 / 140_000

# possible traffic light states
light_options = ['G', 'y', 'r']

# define SUMO commands for starting the simulation
sumo_binary = sumolib.checkBinary('sumo')
sumo_binary_gui = sumolib.checkBinary('sumo-gui')
sumo_cmd = ['-c', os.path.join('sumo_data', cfg_name), '--quit-on-end', '--start']

def start_sumo(binary=sumo_binary):
	'''
	Instantiates a SUMO simulation instance.

	Args:
		binary: path to the SUMO binary file (path to "sumo" or "sumo-gui")
	'''
	port = sumolib.miscutils.getFreeSocketPort()
	# start the simulation and pipe its outputs into /dev/null
	sumoProc = subprocess.Popen([binary] + sumo_cmd + ['--remote-port', str(port)],
								stdout=open(os.devnull, 'w'),
								stderr=open(os.devnull, 'w'))
	traci.init(port)

def get_durations():
	'''Grabs the phase durations for all traffic light systems in the road network.'''
	durations = []
	for tl in tlights:
		definitions = traci.trafficlight.getCompleteRedYellowGreenDefinition(tl)
		for definition in definitions:
			for phase in definition.phases:
				durations.append(phase.minDur)
	return durations

def get_states():
	'''Grabs the state definitions for all traffic light systems in the road network.'''
	states = []
	for tl in tlights:
		definitions = traci.trafficlight.getCompleteRedYellowGreenDefinition(tl)
		for definition in definitions:
			for phase in definition.phases:
				states.append(phase.state)
	return states

def set_genome(durations, states):
	'''
	Set the genome (traffic light phase durations and states) for the current simulation.

	Args:
		durations: a list of durations for all traffic light phases in the simulation
		states: a list of state strings for all traffic light phases in the simulation
	'''
	idx = 0
	# iterate over all traffic light systems in the simulation
	for tl in tlights:
		definition = traci.trafficlight.getCompleteRedYellowGreenDefinition(tl)[0]
		for phase in definition.phases:
			# set the current phase's duration and state
			phase.minDur = phase.maxDur = durations[idx]
			phase.state = states[idx]
			idx += 1
		# replace the durations and phases in the simulation instance
		logic = traci.trafficlight.Logic(traci.trafficlight.getProgram(tl), 0, 0, phases=definition.phases)
		traci.trafficlight.setCompleteRedYellowGreenDefinition(tl, logic)

def mutation(durations, states, p_dur=duration_mutation_rate, p_stat=states_mutation_rate, strength=duration_mutation_strength):
	'''
	Returns a mutated copy of durations and states. Durations are mutated by adding random numbers
	from a normal distribution with mean=0 and std=strength. States are mutated by replacing
	values in one state by a random other state.

	Args:
		durations: a list of durations (float) for each traffic light phase
		states: a list of state strings for individual traffic light phases (e.g. ['GGrr', 'rrGG'])
		p_dur: mutation rate for the phase durations
		p_stat: mutation rate for the traffic light states
		strength: mutation strength for phase durations
	'''
	# make a copy of durations and states to not modify the original lists
	durations = list(durations)
	states = list(states)

	# randomly choose indices to mutate the durations
	dur_idxs = np.where(np.random.uniform(size=len(durations)) < p_dur)[0]
	# mutate the durations
	for i in dur_idxs:
		durations[i] += np.random.normal(scale=strength)

	# iterate over the list of state strings
	for i in range(len(states)):
		# split string up into a list of individual characters
		state = list(states[i])
		for j in np.where(np.random.uniform(size=len(state)) < p_stat)[0]:
			# randomly replace traffic light states
			state[j] = np.random.choice(light_options)
		states[i] = ''.join(state)
	return durations, states

def eval(durs, states):
	# instantiate a new SUMO instance and set insert the current genome
	start_sumo()
	set_genome(durs, states)

	emissions = []
	waiting = []
	fitness = 0
	for step in range(n_steps):
		# take one step in the simulation
		traci.simulationStep()
		# check for collisions in the current time step
		if traci.simulation.getCollidingVehiclesNumber() > 0:
			# break the current simulation and penalize the genome's fitness
			fitness += collision_penalty
			break

		# compute the total emissions and waiting time for all lanes in the simulation
		lane_emissions = 0
		lane_waiting = 0
		for tl_lanes in lanes.values():
			for lane in tl_lanes:
				# compute per lane scores
				lane_emissions += traci.lane.getCO2Emission(lane)
				lane_waiting += traci.lane.getWaitingTime(lane)
		emissions.append(lane_emissions)
		waiting.append(lane_waiting)

	# compute the total fitness score for the current genome
	fitness += emissions_weight * np.sum(emissions) + waiting_weight * np.sum(waiting)
	# close the simulation instance and return the fitness
	traci.close()
	return fitness

# extract traffic light information from the road network
start_sumo()
tlights = traci.trafficlight.getIDList()
lanes = {tl: traci.trafficlight.getControlledLanes(tl) for tl in tlights}
n_links = {tl: len(traci.trafficlight.getControlledLinks(tl)) for tl in tlights}

# initialize the population
if random_init:
	# random initialization
	population = [mutation(get_durations(), get_states(), 1, 1) for _ in range(pop_size)]
else:
	# hardcoded initialization from the road network file
	population = [mutation(get_durations(), get_states(), 1, 1) for _ in range(pop_size - 1)]
	population += [(get_durations(), get_states())]
traci.close()

# initialize the fitness plot
plt.ion()
fig, ax = plt.subplots()
title = ax.set_title(f'Epoch 0')
fitness_graph_min = ax.plot([], label='min')[0]
if not show_only_min:
	fitness_graph_max = ax.plot([], label='max')[0]
	fitness_graph_mean = ax.plot([], label='mean')[0]
ax.legend()
fig.canvas.draw()
fig.canvas.flush_events()

# run the optimization loop
fitnesses_all = []
for epoch in range(100):
	print('=======================================================================================')
	print(f'======================================= Epoch {epoch+1} =======================================')
	print('=======================================================================================')

	# evaluate the fitness scores for the current population in parallel
	fitnesses = Parallel(n_jobs=n_jobs)(delayed(eval)(durs, states) for durs, states in population)
	fitnesses_all.append(fitnesses)

	print(f'=================================== Min fitness: {np.min(fitnesses)} ===================================')
	print()

	# run the genetic algorithm
	# TODO: Implement cross-over, n survivors
	best = np.argmin(fitnesses)
	durs_best, states_best = population[best]
	population = [mutation(durs_best, states_best) for _ in range(pop_size - 1)] + [(durs_best, states_best)]

	# update the fitness plot
	title.set_text(f'Epoch {epoch + 1}')
	fitness_graph_min.set_data(np.arange(len(fitnesses_all))+1, np.array(fitnesses_all).min(axis=1))
	if not show_only_min:
		fitness_graph_max.set_data(np.arange(len(fitnesses_all))+1, np.array(fitnesses_all).max(axis=1))
		fitness_graph_mean.set_data(np.arange(len(fitnesses_all))+1, np.array(fitnesses_all).mean(axis=1))
	ax.relim()
	ax.autoscale_view()
	fig.canvas.draw()
	fig.canvas.flush_events()

	# run the current best genome in a SUMO simulation with GUI
	if show_its > 0 and (epoch + 1) % show_its == 0:
		try:
			# initialize
			print('Visualizing the current best')
			start_sumo(sumo_binary_gui)
			set_genome(durs_best, states_best)
			# run simulation
			for step in range(n_show_steps):
				traci.simulationStep()
				time.sleep(40 / 1000)
			traci.close()
		except traci.exceptions.FatalTraCIError:
			# user manually closed the simulation window, just proceed with the optimization
			print('Manually closed TraCI visualization')
			traci.close()

print('=================================== Done! ===================================')
plt.ioff()
plt.show()
