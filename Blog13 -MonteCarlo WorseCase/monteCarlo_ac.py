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
    "R2": {"value": r2, "tol": res_tol, "type": distributionType.GAUSIAN},
    "C1": {"value": c1, "tol": cap_tol, "type": distributionType.UNIFORM},
    "C2": {"value": c2, "tol": cap_tol, "type": distributionType.UNIFORM}
}

# --- Simulation outputs/nodes that we want the data from
outputs = [
    "V(out1)",
    "V(out2)"
]

# Number of simulations to run
numberOfSimulations = 10






def plot_init():
    # Setup subplots
    num_plots = len(outputs)
    # sharex=True is useful for transient plots to keep time axes synced
    fig, axes = plt.subplots(num_plots, 1, sharex=True, figsize=(8, 4 * num_plots))

    # Handle the case where there is only one subplot (axes is not an array)
    if num_plots == 1:
        axes = [axes]

    return axes

def acPlot(axes, i, xData, yData):
    axes[i].plot(np.real(xData), 20*np.log10(np.abs(yData)), alpha=0.2, color='blue', linewidth=0.5)
    axes[i].set_ylabel(f"{node_name} (V)")
    axes[i].set_title(f"AC Response: {node_name}")
    axes[i].grid(True)
    axes[i].set_xscale("log")

    axes[-1].set_xlabel("Freqiency (Hz)")

def bwHistogram(filterBW):
    # Create the figure
    plt.figure(num=2,figsize=(10, 6))

    # Plot the histogram
    # 'bins' defines how many bars to show; 20 is a good starting point for 100+ sims
    plt.hist(filterBW, bins=30, color='skyblue', edgecolor='black')

    # Add labels and title
    plt.title('Distribution of Filter BW')
    plt.xlabel('Bandwidth (Hz)')
    plt.ylabel('Frequency (Number of Simulations)')

    # Show the plot
    plt.grid(axis='y', alpha=0.75)

def resHistogram(res):
    # Create the figure
    plt.figure(num=3,figsize=(10, 6))

    # Plot the histogram
    # 'bins' defines how many bars to show; 20 is a good starting point for 100+ sims
    plt.hist(res, bins=30, color='skyblue', edgecolor='black')

    # Add labels and title
    plt.title('Distribution of R1')
    plt.xlabel('Resistance (Ohms)')
    plt.ylabel('Frequency (Number of Simulations)')

    # Show the plot
    plt.grid(axis='y', alpha=0.75)

def calculateBW(xTime, yData, filterBW):

    # Convert to numpy array first
    y_array = np.array(yData)

    # If your data is already in dB:
    bw_idx = np.argmin(np.abs(20*np.log10(np.abs(yData)) - (-3)))
    

    filterBW.append(np.real(xTime[bw_idx]))
    
def printData(simNumber,simPartInfo, filterBW ):
    print(f"Sim {simNumber}", end="")

    for part_name, part_value in simPartInfo.items():
        print(f", {part_name}={part_value:.4e}", end="")

    print(f" | BW={filterBW/1e3:.5f}kHz")




# Initialize the monte carlo simulation
mc = lt.monteCarlo(
    simulatorLocation=r"D:\Programs\LTspice\LTspice.exe", 
    schematicName='RC_ac'
)

# Initialize my plotting
axes = plot_init()

# List of simulated R1 resistor values (to check if it is gaussian)
res_r1 = []

# Calculated filder -3dB bandwidth results from each simulation run
filterBW = []


# Run the monte carlo simulation
for simNumber, simPartInfo, data in mc.run(parts=parts,  numSimulations=numberOfSimulations):

    # Get x data (will be the same for all outputs/nodes)
    xData = mc.getXdata(data)
    
    # Loop through outputs and plot to separate subplots
    for i, node_name in enumerate(outputs):
        
        # Get y data
         yData = mc.getYdata(data,node_name)
         
         acPlot(axes,i, xData, yData)

    res_r1.append(simPartInfo["R1"])
    calculateBW(xData, mc.getYdata(data,"V(out2)"), filterBW)
    printData(simNumber, simPartInfo, filterBW[-1])

resHistogram(res_r1)
bwHistogram(filterBW)

plt.show()















