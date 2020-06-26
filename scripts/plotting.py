import matplotlib.pyplot as plt
import pandas as pd

fname = "data/simulation_data_26-06-2020_04-01-26_PM.csv"

df = pd.read_csv(fname)

fig, axes = plt.subplots(2,1,sharex=True, figsize=(10,8))
fig.suptitle("Improvements over time")
axes[0].plot(df["Waiting Time"], color="b")
axes[0].set_xlabel("Time steps")
axes[0].set_ylabel("Waiting Time")
axes[1].plot(df["Emissions"], color="r")
axes[1].set_ylabel("Emissions")
axes[1].set_xlabel("Time steps")

plt.tight_layout()
fig.subplots_adjust(top=0.95)
plt.show()
