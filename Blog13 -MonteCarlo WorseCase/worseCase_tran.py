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
    "R2": {"value": r2, "tol": res_tol, "type": distributionType.UNIFORM},
    "C1": {"value": c1, "tol": cap_tol, "type": distributionType.UNIFORM},
    "C2": {"value": c2, "tol": cap_tol, "type": distributionType.UNIFORM}
}

# --- Simulation outputs/nodes that we want the data from
outputs = [
    "V(out1)",
    "V(out2)"
]

# Calculated 10%-90% rise time results from each simulation run
risetime_10to90 = []



def plot_init():
    # Setup subplots
    num_plots = len(outputs)
    # sharex=True is useful for transient plots to keep time axes synced
    fig, axes = plt.subplots(num_plots, 1, sharex=True, figsize=(8, 4 * num_plots))

    # Handle the case where there is only one subplot (axes is not an array)
    if num_plots == 1:
        axes = [axes]

    return axes

def tranPlot(axes, i, xData, yData):
    axes[i].plot(xData, yData, alpha=1, color='blue', linewidth=0.5)
    axes[i].set_ylabel(f"{node_name} (V)")
    axes[i].set_title(f"Transient Response: {node_name}")
    axes[i].grid(True)

    axes[-1].set_xlabel("Time (s)")

def calculateRiseTime(xTime, yData, risetime_10to90):
    vout2_90percent = 0.9 * 1
    vout2_10percent = 0.1 * 1
    
    vout2_90percent_index = np.argmax(yData >= vout2_90percent)
    vout2_10percent_index = np.argmax(yData >= vout2_10percent)

    risetime = xTime[vout2_90percent_index] - xTime[vout2_10percent_index]

    risetime_10to90.append(risetime)
    
def printData(simNumber,simPartInfo, risetime_10to90 ):
    print(f"Sim {simNumber}", end="")

    for part_name, part_value in simPartInfo.items():
        print(f", {part_name}={part_value:.4e}", end="")

    print(f" | RT={risetime_10to90}")




# Initialize the monte carlo simulation
mc = lt.worseCase(
    simulatorLocation=r"D:\Programs\LTspice\LTspice.exe", 
    schematicName='RC_tran'
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
         
         tranPlot(axes, i, xData, yData)

    calculateRiseTime(xData, mc.getYdata(data,"V(out2)"), risetime_10to90)
    printData(simNumber, simPartInfo, risetime_10to90[-1])


print(f"Max Rise Time = {max(risetime_10to90):.3e}")
print(f"Min Rise Time = {min(risetime_10to90):.3e}")

plt.show()















