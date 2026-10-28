
import PyLTSpice
import os
import random
from enum import Enum
import numpy as np
import itertools


class distributionType(Enum):
    UNIFORM = 1
    GAUSIAN = 2
    NONE = 3

class monteCarlo:

    def __init__(self, simulatorLocation, schematicName,  numberOfSigma=6):

        self.schematicName = schematicName
        self.numberOfSigma = numberOfSigma

        # Create a handler to perform LTspice specific operations
        # Set the location of outputted files into a temp folder
        self.ltsim = PyLTSpice.SimRunner(output_folder='./temp', simulator=simulatorLocation)

        # Create a netlist from the LTspice schematic
        self.ltsim.create_netlist('.\\' + schematicName + '.asc')

        # Create a handler for the newly created netlist
        self.netlist = PyLTSpice.SpiceEditor('.\\' + schematicName + '.net')


    def run(self, parts:dict, numSimulations):

        # Remove old temporary files to prevent Python reading old RAW files
        # with the same name as the new ones. There is a delay between running
        # the LTspice simulation and the new RAW file being generated
        _clearTempFiles()

        simPartInfo = {}

        # Cycle through each simulation
        for simNumber in range(1,numSimulations+1):
            
            # Cycle through each part that we want to generate a new 
            # randomize value (based on its tolderance 
            # and update each part in the netlist
            for part_name, part_info in parts.items():

                # Generate a value for the part based on its tolerance
                value = self._generatePartValue(part_info)

                # Update the component in the netlist object with the generated part value 
                self.netlist.set_component_value(part_name, str(value))

                simPartInfo[part_name] = value


            # Run LTspice simulation of netlist
            sim_task = self.ltsim.run(self.netlist)
            
            # Wait for simulation to complete
            sim_task.wait_results()
            
            ltraw = PyLTSpice.RawRead('.\\temp\\' + self.schematicName + '_' + str(simNumber) +  '.raw')

            # PAUSE HERE: Give the data to the user's custom plot/manipulation code.
            # Once the user's plotting logic finishes, Python resumes this loop 
            # and forgets the old 'raw_data' object, keeping memory perfectly lean.        
            yield simNumber, simPartInfo, ltraw


    # Get x data based on simulation type
    def getXdata(self,data:PyLTSpice.RawRead) -> np.ndarray | None:
        return data.get_trace(0).get_wave(0)

        
    # Get y data
    def getYdata(self, data:PyLTSpice.RawRead, nodeName)-> np.ndarray | None:
        return data.get_trace(str(nodeName)).get_wave(0)


    # Generate a value for the part based on its tolerance
    def _generatePartValue(self, part_info):
        
        # If the part has a normal distribution
        if part_info["type"] == distributionType.UNIFORM:
            nominal = part_info["value"]
            tol = part_info["tol"]
            return nominal * random.uniform(1 - tol, 1 + tol)

        # If the part has a gausian distrubution 
        elif part_info["type"] == distributionType.GAUSIAN:
            nominal = part_info["value"]
            tol = part_info["tol"]
            return nominal + random.gauss(0, self._calculateSigma(nominal, tol))

        # If we just want to return the nominal value
        else:
            return part_info["value"]



    # Calculate the sigma of the part based on tolerance and the number of sigmas
    def _calculateSigma(self,value, tolerance, numberOfSigma = 6):

        # If user enters value of signma that is too large or too small make it 6.6sigma
        if (numberOfSigma > 6.6) or (numberOfSigma < 1):
            numberOfSigma = 6

        return (value * tolerance) / (numberOfSigma / 2)


class worseCase:

    # Note: the _=None is just used to allow us to easily switch between monteCarlo and worseCase easier.
    def __init__(self, simulatorLocation, schematicName, _=None):

        self.schematicName = schematicName

        # Create a handler to perform LTspice specific operations
        # Set the location of outputted files into a temp folder
        self.ltsim = PyLTSpice.SimRunner(output_folder='./temp', simulator=simulatorLocation)

        # Create a netlist from the LTspice schematic
        self.ltsim.create_netlist('.\\' + schematicName + '.asc')

        # Create a handler for the newly created netlist
        self.netlist = PyLTSpice.SpiceEditor('.\\' + schematicName + '.net')


    def run(self, parts:dict):

        # Remove old temporary files to prevent Python reading old RAW files
        # with the same name as the new ones. There is a delay between running
        # the LTspice simulation and the new RAW file being generated
        _clearTempFiles()

        simPartInfo = {}

        part_wc_range = self._generatePartRange(parts)

        # Generate keys and related values from the parts dictionary
        keys = list(part_wc_range.keys())
        values = list(part_wc_range.values())


        simNumber = 0

        for combo in itertools.product(*values):
            # Zip keys and the current combination together: [('R1', 1), ('R2', 2), ('R3', 3)]
            labeled_combo = dict(zip(keys, combo))   

            # Increment current simulation number
            simNumber += 1

            # Set part values for simulation
            for part_name, value in labeled_combo.items():
                self.netlist.set_component_value(str(part_name), str(value))
                simPartInfo[part_name] = value

            # Run LTspice simulation of netlist
            sim_task = self.ltsim.run(self.netlist)
            
            # Wait for simulation to complete
            sim_task.wait_results()
            
            ltraw = PyLTSpice.RawRead('.\\temp\\' + self.schematicName + '_' + str(simNumber) +  '.raw')

            # PAUSE HERE: Give the data to the user's custom plot/manipulation code.
            # Once the user's plotting logic finishes, Python resumes this loop 
            # and forgets the old 'raw_data' object, keeping memory perfectly lean.        
            yield simNumber, simPartInfo, ltraw

    # Get x data based on simulation type
    def getXdata(self,data:PyLTSpice.RawRead) -> np.ndarray | None:
        return data.get_trace(0).get_wave(0)

        
    # Get y data
    def getYdata(self, data:PyLTSpice.RawRead, nodeName)-> np.ndarray | None:
        return data.get_trace(str(nodeName)).get_wave(0)


    # Generate a value for the part based on its tolerance
    def _generatePartRange(self, parts:dict):

        part_range = {}

        # For each part, find the min and max value it can be for a given tolerance
        for name, info in parts.items():

            value_min = info["value"]*(1-info["tol"])
            value_max = info["value"]*(1+info["tol"])

            part_range[name] = [value_min, value_max]

        return part_range




# Remove all files from temp folder
def _clearTempFiles():
    folder = './temp'
    
    # Create folder if it doesn't exist to prevent os.listdir crashing
    if not os.path.exists(folder):
        os.makedirs(folder)
        return

    for filename in os.listdir(folder):
        filepath = os.path.join(folder, filename)
        if os.path.isfile(filepath):
            os.remove(filepath)








