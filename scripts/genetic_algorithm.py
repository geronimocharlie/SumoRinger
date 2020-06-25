import os
import traci
import sumolib
import tqdm
import subprocess
import numpy as np
from joblib import Parallel, delayed
from matplotlib import pyplot as plt

pop_size = 5

emissions_weight = 1 / 32000000
waiting_weight = 1 / 140000

light_options = ['G', 'y', 'r']

# cfg_name = 'simple.sumocfg'
# cfg_name = 'random.sumocfg'
cfg_name = 'single-intersection.sumocfg'

sumo_binary = sumolib.checkBinary('sumo')
sumoCmd = [sumo_binary, '-c', os.path.join('sumo_data', cfg_name), '--collision.action', 'remove']

def start_sumo():
	PORT = sumolib.miscutils.getFreeSocketPort()
	sumoProc = subprocess.Popen(sumoCmd + ['--remote-port', str(PORT)], stdout=open(os.devnull, 'w'))
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

def mutation(durations, states, p_dur=0.3, p_stat=0.2, strength=15):
	durations = list(durations)
	states = list(states)

	dur_idxs = np.where(np.random.uniform(size=len(durations)) < p_dur)[0]
	for i in dur_idxs:
		durations[i] += np.random.normal(scale=strength)

	for i in range(len(states)):
		states[i] = list(states[i])
		for j in np.where(np.random.uniform(size=len(durations)) < p_stat)[0]:
			states[i][j] = np.random.choice(light_options)
		states[i] = ''.join(states[i])
	return durations, states

def eval(durs, states):
	start_sumo()
	set_genome(durs, states)
	emissions = []
	waiting = []
	for step in range(2000):
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

# population = [mutation(get_durations(), get_states(), 1, 1) for _ in range(pop_size)]
population = [mutation(get_durations(), get_states(), 1, 1) for _ in range(pop_size - 1)]
population += [(get_durations(), get_states())]
traci.close()

plt.ion()
fig, ax = plt.subplots()
fitness_graph_min = ax.plot([], label='min')[0]
fitness_graph_mean = ax.plot([], label='mean')[0]
fitness_graph_max = ax.plot([], label='max')[0]
ax.legend()
fig.canvas.draw()
fig.canvas.flush_events()

fitnesses_all = []
for epoch in range(15):
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

	fitness_graph_min.set_data(range(len(fitnesses_all)), np.array(fitnesses_all).min(axis=1))
	fitness_graph_mean.set_data(range(len(fitnesses_all)), np.array(fitnesses_all).mean(axis=1))
	fitness_graph_max.set_data(range(len(fitnesses_all)), np.array(fitnesses_all).max(axis=1))
	ax.relim()
	ax.autoscale_view()
	fig.canvas.draw()
	fig.canvas.flush_events()

print('=================================== Done! ===================================')
plt.ioff()
plt.show()
