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



def plot_init():
    # Setup subplots
    num_plots = len(outputs)
    # sharex=True is useful for transient plots to keep time axes synced
    fig, axes = plt.subplots(num_plots, 1, sharex=True, figsize=(8, 4 * num_plots))

    # Handle the case where there is only one subplot (axes is not an array)
    if num_plots == 1:
        axes = [axes]

    return axes

def dcPlot(axes, i, xData, yData):
    axes[i].plot(xData, yData, alpha=0.2, color='blue', linewidth=0.5)
    axes[i].set_ylabel(f"{node_name} (V)")
    axes[i].set_title(f"DC Sweep: {node_name}")
    axes[i].grid(True)

    axes[-1].set_xlabel("VIN (V)")

    
def printData(simNumber,simPartInfo ):
    print(f"Sim {simNumber}", end="")

    for part_name, part_value in simPartInfo.items():
        print(f", {part_name}={part_value:.4e}", end="")

    print()




# Initialize the monte carlo simulation
mc = lt.monteCarlo(
    simulatorLocation=r"D:\Programs\LTspice\LTspice.exe", 
    schematicName='RC_dc'
)

# Initialize my plotting
axes = plot_init()

# Run the monte carlo simulation
for simNumber, simPartInfo, data in mc.run(parts=parts,  numSimulations=numberOfSimulations):

    # Get x data (will be the same for all outputs/nodes)
    xData = mc.getXdata(data)
    
    # Loop through outputs and plot to separate subplots
    for i, node_name in enumerate(outputs):
        
        # Get y data
        yData = mc.getYdata(data,node_name)

        dcPlot(axes, i, xData, yData)


    printData(simNumber, simPartInfo)


plt.show()















