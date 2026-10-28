import matplotlib            
matplotlib.use('Qt5Agg')      

import numpy as np
import matplotlib.pyplot as plt

import LTSpice_Tol as lt


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
outputs = [
    "V(out1)",
    "V(out2)"
]



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
    axes[i].plot(np.real(xData), 20*np.log10(np.abs(yData)), alpha=1, color='blue', linewidth=0.5)
    axes[i].set_ylabel(f"{node_name} (V)")
    axes[i].set_title(f"AC Response: {node_name}")
    axes[i].grid(True)
    axes[i].set_xscale("log")

    axes[-1].set_xlabel("Freqiency (Hz)")

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
mc = lt.worseCase(
    simulatorLocation=r"D:\Programs\LTspice\LTspice.exe", 
    schematicName='RC_ac'
)

# Initialize my plotting
axes = plot_init()

# Calculated filder -3dB bandwidth results from each simulation run
filterBW = []


# Run the monte carlo simulation
for simNumber, simPartInfo, data in mc.run(parts=parts):

    # Get x data (will be the same for all outputs/nodes)
    xData = mc.getXdata(data)
    
    # Loop through outputs and plot to separate subplots
    for i, node_name in enumerate(outputs):
        
        # Get y data
         yData = mc.getYdata(data,node_name)
         
         acPlot(axes,i, xData, yData)

    calculateBW(xData, mc.getYdata(data,"V(out2)"), filterBW)
    printData(simNumber, simPartInfo, filterBW[-1])

print(f"Max BW = {max(filterBW)/1e3:.5f}kHz")
print(f"Min BW = {min(filterBW)/1e3:.5f}kHz")


plt.show()















