import os
import traci
import sumolib
import tqdm
import time
import subprocess
import numpy as np
from joblib import Parallel, delayed
from matplotlib import pyplot as plt

pop_size = 10
n_steps = 2000
n_show_steps = 500
show_its = 3
random_init = True
show_only_min = False
duration_mutation_rate = 0.3
duration_mutation_strength = 15
states_mutation_rate = 0.2

emissions_weight = 1 / 32_000_000
waiting_weight = 1 / 140_000

light_options = ['G', 'y', 'r']

cfg_name = 'martini.sumocfg'

sumo_binary = sumolib.checkBinary('sumo')
sumo_binary_gui = sumolib.checkBinary('sumo-gui')
sumo_cmd = ['-c', os.path.join('sumo_data', cfg_name), '--quit-on-end', '--start']

def start_sumo(binary=sumo_binary):
	PORT = sumolib.miscutils.getFreeSocketPort()
	sumoProc = subprocess.Popen([binary] + sumo_cmd + ['--remote-port', str(PORT)], stdout=open(os.devnull, 'w'), stderr=open(os.devnull, 'w'))
	traci.init(PORT)

def get_durations():
	durations = []
	for tl in tlights:
		definitions = traci.trafficlight.getCompleteRedYellowGreenDefinition(tl)
		for definition in definitions:
			for phase in definition.phases:
				durations.append(phase.minDur)
	return durations

def get_states():
	states = []
	for tl in tlights:
		definitions = traci.trafficlight.getCompleteRedYellowGreenDefinition(tl)
		for definition in definitions:
			for phase in definition.phases:
				states.append(phase.state)
	return states

def set_genome(durations, states):
	idx = 0
	for tl in tlights:
		n_links = len(traci.trafficlight.getControlledLinks(tl))
		definition = traci.trafficlight.getCompleteRedYellowGreenDefinition(tl)[0]
		for phase in definition.phases:
			phase.minDur = phase.maxDur = durations[idx]
			phase.state = states[idx]
			idx += 1
		logic = traci.trafficlight.Logic(traci.trafficlight.getProgram(tl), 0, 0, phases=definition.phases)
		traci.trafficlight.setCompleteRedYellowGreenDefinition(tl, logic)

def mutation(durations, states, p_dur=duration_mutation_rate, p_stat=states_mutation_rate, strength=duration_mutation_strength):
	durations = list(durations)
	states = list(states)

	dur_idxs = np.where(np.random.uniform(size=len(durations)) < p_dur)[0]
	for i in dur_idxs:
		durations[i] += np.random.normal(scale=strength)

	for i in range(len(states)):
		states[i] = list(states[i])
		for j in np.where(np.random.uniform(size=len(states[i])) < p_stat)[0]:
			states[i][j] = np.random.choice(light_options)
		states[i] = ''.join(states[i])
	return durations, states

def eval(durs, states):
	start_sumo()
	set_genome(durs, states)
	emissions = []
	waiting = []
	for step in range(n_steps):
		traci.simulationStep()
		emissions.append(np.sum([traci.lane.getCO2Emission(lane) for tl_lanes in lanes.values()  for lane in tl_lanes]))
		waiting.append(np.sum([traci.lane.getWaitingTime(lane) for tl_lanes in lanes.values()  for lane in tl_lanes]))
	fitness = emissions_weight * np.sum(emissions) + waiting_weight * np.sum(waiting)
	traci.close()
	return fitness

start_sumo()
tlights = traci.trafficlight.getIDList()
lanes = {tl: traci.trafficlight.getControlledLanes(tl) for tl in tlights}
n_links = {tl: len(traci.trafficlight.getControlledLinks(tl)) for tl in tlights}

if random_init:
	population = [mutation(get_durations(), get_states(), 1, 1) for _ in range(pop_size)]
else:
	population = [mutation(get_durations(), get_states(), 1, 1) for _ in range(pop_size - 1)]
	population += [(get_durations(), get_states())]
traci.close()

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

fitnesses_all = []
for epoch in range(100):
	print('=======================================================================================')
	print(f'======================================= Epoch {epoch+1} =======================================')
	print('=======================================================================================')

	fitnesses = Parallel(n_jobs=-1)(delayed(eval)(durs, states) for durs, states in population)
	fitnesses_all.append(fitnesses)

	print(f'=================================== Min fitness: {np.min(fitnesses)} ===================================')
	print()

	best = np.argmin(fitnesses)
	durs_best, states_best = population[best]
	population = [mutation(durs_best, states_best) for _ in range(pop_size - 1)] + [(durs_best, states_best)]

	title.set_text(f'Epoch {epoch + 1}')
	fitness_graph_min.set_data(np.arange(len(fitnesses_all))+1, np.array(fitnesses_all).min(axis=1))
	if not show_only_min:
		fitness_graph_max.set_data(np.arange(len(fitnesses_all))+1, np.array(fitnesses_all).max(axis=1))
		fitness_graph_mean.set_data(np.arange(len(fitnesses_all))+1, np.array(fitnesses_all).mean(axis=1))
	ax.relim()
	ax.autoscale_view()
	fig.canvas.draw()
	fig.canvas.flush_events()

	if show_its > 0 and (epoch + 1) % show_its == 0:
		try:
			print('Visualizing the current best')
			start_sumo(sumo_binary_gui)
			set_genome(durs_best, states_best)
			for step in range(n_show_steps):
				traci.simulationStep()
				time.sleep(40 / 1000)
			traci.close()
		except traci.exceptions.FatalTraCIError:
			print('Manually closed TraCI visualization')
			traci.close()

print('=================================== Done! ===================================')
plt.ioff()
plt.show()
