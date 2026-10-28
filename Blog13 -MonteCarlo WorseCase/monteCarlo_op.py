import matplotlib            
matplotlib.use('Qt5Agg')      

import numpy as np
import matplotlib.pyplot as plt

import LTSpice_Tol as lt
from LTSpice_Tol import distributionType




# --- Simulation parts that we want to varry for the Monte Carlo simulation

r1 = 200
r2 = 200
c1 = 1e-9
c2 = 1e-9

res_tol = 0.05
cap_tol = 0.20


parts = {
    "R1": {"value": r1, "tol": res_tol, "type": distributionType.GAUSIAN},
    "R2": {"value": r2, "tol": res_tol, "type": distributionType.UNIFORM}
}

# --- Simulation outputs/nodes that we want the data from
outputs = [
    "V(out1)",
    "V(out2)"
]

# Number of simulations to run
numberOfSimulations = 10

# Results from each simulation run:
# DC voltage at Vout1
dc_vout1 = []
# DC voltage at Vout2
dc_vout2 = []
# Number of simulations
simulation_number = []





def plot_init():
    # Setup subplots
    num_plots = len(outputs)
    # sharex=True is useful for transient plots to keep time axes synced
    fig, axes = plt.subplots(num_plots, 1, sharex=True, figsize=(8, 4 * num_plots))

    # Handle the case where there is only one subplot (axes is not an array)
    if num_plots == 1:
        axes = [axes]

    return axes

def opPlot(axes,simulation_number, vout1, vout2):

    axes[0].plot(simulation_number, vout1, color='blue')
    axes[0].set_ylabel(f"Voltage (V)")
    axes[0].set_title(f"Vout1")
    axes[0].grid(True)

    axes[1].plot(simulation_number, vout2, color='blue')
    axes[1].set_ylabel(f"Voltage (V)")
    axes[1].set_title(f"Vout2")
    axes[1].grid(True)

    axes[-1].set_xlabel("Time (s)")

def opHistogram(voltage):
    # Create the figure
    plt.figure(num=2,figsize=(10, 6))

    # Plot the histogram
    # 'bins' defines how many bars to show; 20 is a good starting point for 100+ sims
    plt.hist(voltage, bins=30, color='skyblue', edgecolor='black')

    # Add labels and title
    plt.title('Distribution of Output Voltages')
    plt.xlabel('Output Voltage VOUT2 (V)')
    plt.ylabel('Frequency (Number of Simulations)')

    # Show the plot
    plt.grid(axis='y', alpha=0.75)
    
def printData(simNumber,simPartInfo, voltage1, voltage2 ):
    print(f"Sim {simNumber}", end="")

    for part_name, part_value in simPartInfo.items():
        print(f", {part_name}={part_value:.4e}", end="")

    print(f" | Vout1={voltage1:4f}, Vout2={voltage2:4f}")




# Initialize the monte carlo simulation
mc = lt.monteCarlo(
    simulatorLocation=r"D:\Programs\LTspice\LTspice.exe", 
    schematicName='RC_op'
)

# Initialize my plotting
axes = plot_init()

# Run the monte carlo simulation
for simNumber, simPartInfo, data in mc.run(parts=parts,  numSimulations=numberOfSimulations):

    # The .op simulation just simulates DC at one point. For plotting we will just have our x-axis increment by the simNumber
    simulation_number.append(simNumber)
    
    # Append the new simulation output voltage to arrays
    dc_vout1.append(mc.getYdata(data,"V(out1)")[0])
    dc_vout2.append(mc.getYdata(data,"V(out2)")[0])


    printData(simNumber, simPartInfo, dc_vout1[-1], dc_vout2[-1])

# Plot the VOUT1 and VOUT2 results from each simulation
opPlot(axes, simulation_number, dc_vout1, dc_vout2)

# Plot a histogram of VOUT2
opHistogram(dc_vout2)

plt.show()















