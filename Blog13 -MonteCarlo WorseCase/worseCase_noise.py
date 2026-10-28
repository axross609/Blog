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
    "R1": {"value": r1, "tol": res_tol},
    "R2": {"value": r2, "tol": res_tol},
    "C1": {"value": c1, "tol": cap_tol},
    "C2": {"value": c2, "tol": cap_tol}
}

# --- Simulation outputs/nodes that we want the data from
# Note for noise simulation the V(onoise) noise is defined in the simulation .NOISE setup
outputs = [
    "V(onoise)"
]

# Calculated Total RMS Noise results from each simulation run
totalRmsNoise = []




def plot_init():
    # Setup subplots
    num_plots = len(outputs)
    # sharex=True is useful for transient plots to keep time axes synced
    fig, axes = plt.subplots(num_plots, 1, sharex=True, figsize=(8, 4 * num_plots))

    # Handle the case where there is only one subplot (axes is not an array)
    if num_plots == 1:
        axes = [axes]

    return axes

def noisePlot(axes, i, xData, yData):
    axes[i].plot(xData, yData, alpha=0.2, color='blue', linewidth=0.5)
    axes[i].set_ylabel(f"{node_name} (V)")
    axes[i].set_title(f"Noise: {node_name}")
    axes[i].grid(True)
    axes[i].set_xscale("log")

    axes[-1].set_xlabel("Frequency (Hz)")

def calculateRmsNoise(xFreq, yData, totalRmsNoise:list):

    power_density = yData ** 2
    integrated_power = np.trapezoid(power_density, x=xFreq)
    total_rms_noise = np.sqrt(integrated_power)

    totalRmsNoise.append(total_rms_noise)
    
def printData(simNumber,simPartInfo, totalRmsNoise:list ):
    print(f"Sim {simNumber}", end="")

    for part_name, part_value in simPartInfo.items():
        print(f", {part_name}={part_value:.4e}", end="")

    print(f" | Total RMS Noise={totalRmsNoise:.3e}")




# Initialize the monte carlo simulation
mc = lt.worseCase(
    simulatorLocation=r"D:\Programs\LTspice\LTspice.exe", 
    schematicName='RC_noise'
)

# Initialize my plotting
axes = plot_init()

# Run the monte carlo simulation
for simNumber, simPartInfo, data in mc.run(parts=parts):

    # Get x data (will be the same for all outputs/nodes)
    xData = mc.getXdata(data)
    
    # Loop through outputs and plot to separate subplots
    for i, node_name in enumerate(outputs):
        
        # Get y data
         yData = mc.getYdata(data,node_name)
         
         noisePlot(axes, i, xData, yData)

    calculateRmsNoise(xData, mc.getYdata(data,"V(onoise)"), totalRmsNoise)
    printData(simNumber, simPartInfo, totalRmsNoise[-1])

print(f"Max Noise = {max(totalRmsNoise):.3e}Vrms")
print(f"Min Noise = {min(totalRmsNoise):.3e}Vrms")


plt.show()















