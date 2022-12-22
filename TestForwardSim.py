
import os
import sys
import opensim
import numpy as np
import casadi as ca
import shutil
import importlib
import pandas as pd

pathMain = os.getcwd()

# %% User inputs.
# Provide path to the directory where you want to save your results.
pathExample = os.path.join(pathMain, 'examples')
# Provide path to OpenSim model.
pathOpenSimModel = os.path.join(pathExample, 'Gait2D_v43.osim')

# Run forward simulation from initial state




osimModel = opensim.Model(pathOpenSimModel)

KinematicRep = opensim.TableReporter()
KinematicRep.setName('kinematic_repoter')
KinematicRep.set_report_time_interval(0.01)

nCoords =osimModel.getCoordinateSet().getSize()
for ic in range(nCoords):
    KinematicRep.addToReport(osimModel.getCoordinateSet().get(ic).getOutput('value'),
                             osimModel.getCoordinateSet().get(ic).getName())
    KinematicRep.addToReport(osimModel.getCoordinateSet().get(ic).getOutput('speed'),
                             osimModel.getCoordinateSet().get(ic).getName() +'_dot')
    KinematicRep.addToReport(osimModel.getCoordinateSet().get(ic).getOutput('acceleration'),
                             osimModel.getCoordinateSet().get(ic).getName() + '_ddot')
osimModel.addComponent(KinematicRep)
s = osimModel.initSystem()

# run forward simulation
finalTime = 0.5
thisState = opensim.State(s)
manager = opensim.Manager(osimModel)
manager.initialize(thisState)
state = manager.integrate(finalTime)

# get the kinematics
Report = KinematicRep.getTable()
opensim.STOFileAdapter.write(Report, 'TestForwardDat.sto')

# get coordinate information
coordinateSet = osimModel.getCoordinateSet()
nCoordinates = coordinateSet.getSize()
nCoordinatesAll = coordinateSet.getSize()
coordinates = []
for coor in range(nCoordinates):
    coordinates.append(coordinateSet.get(coor).getName())

# Test create a dummy motion
from utilities import numpy2storage
DummyData = np.zeros((10,nCoordinates+1))
for coor in range(nCoordinates):
    DummyData[:,coor+1] = np.random.rand()
DummyData[:,0] = np.linspace(0.01,0.1,10)
labelsDummy = []
labelsDummy.append("time")
for coor in range(nCoordinates):
    labelsDummy.append(coordinateSet.get(coor).getName())
numpy2storage(labelsDummy, DummyData, os.path.join(pathMain, "DummyDat.sto"))


# solve inverse dynamics with this model
pathID = pathExample = os.path.join(pathMain, 'InverseDynamics')
pathGenericIDSetupFile = os.path.join(pathID, "SetupID.xml")
idTool = opensim.InverseDynamicsTool(pathGenericIDSetupFile)
idTool.setName("ID_withOsimAndIDTool")
idTool.setModelFileName(pathOpenSimModel)
#idTool.setResultsDir(pathID)
idTool.setCoordinatesFileName(os.path.join(pathMain, "DummyDat.sto"))
#idTool.setCoordinatesFileName(os.path.join(pathMain, "TestForwardDat.sto"))
idTool.setOutputGenForceFileName("ID_withOsimAndIDTool.sto")
pathSetupID = os.path.join(pathID, "SetupID.xml")
idTool.printToXML(pathSetupID)

command = 'opensim-cmd' + ' run-tool ' + pathSetupID
os.system(command)

# Extract torques from .osim + ID tool.
headers = []
for coord in range(nCoordinatesAll):
    if (coordinateSet.get(coord).getName() == "pelvis_tx" or
            coordinateSet.get(coord).getName() == "pelvis_ty" or
            coordinateSet.get(coord).getName() == "pelvis_tz" or
            coordinateSet.get(coord).getName() == "knee_angle_r_beta" or
            coordinateSet.get(coord).getName() == "knee_angle_l_beta"):
        suffix_header = "_force"
    else:
        suffix_header = "_moment"
    headers.append(coordinateSet.get(coord).getName() + suffix_header)

from utilities import storage2df
ID_osim_df = storage2df(os.path.join(pathID, "ID_withOsimAndIDTool.sto"), headers)
ID_osim = np.zeros((nCoordinates))
iframe = 1
for count, coordinate in enumerate(coordinates):
    if (coordinate == "pelvis_tx" or
            coordinate == "pelvis_ty" or
            coordinate == "pelvis_tz"):
        suffix_header = "_force"
    else:
        suffix_header = "_moment"
    ID_osim[count] = ID_osim_df.iloc[iframe][coordinate + suffix_header]


# get headers for velocities and accelerations
coordinates_dot = []
coordinates_ddot = []
for coor in range(nCoordinates):
    coordinates_dot.append(coordinateSet.get(coor).getName() +'_dot')
    coordinates_ddot.append(coordinateSet.get(coor).getName() + '_ddot')

# Extract torques from external function.
Qosim = storage2df(os.path.join(pathMain, "TestForwardDat.sto"), coordinates)
Qosim_dot = storage2df(os.path.join(pathMain, "TestForwardDat.sto"), coordinates_dot)
Qosim_ddot = storage2df(os.path.join(pathMain, "TestForwardDat.sto"), coordinates_ddot)

# create a vector for input in the external function
vecInput = np.zeros((nCoordinates*3, 1))
for coor in range(nCoordinates):
    vecInput[coor * 2] = Qosim.iloc[iframe][coordinates[coor]]
    vecInput[coor * 2 + 1] = Qosim_dot.iloc[iframe][coordinates_dot[coor]]
    vecInput[nCoordinates * 2 + coor ] = Qosim_ddot.iloc[iframe][coordinates_ddot[coor]]

# create input vector based on dummy motion
vecInput2 = np.zeros((nCoordinates*3, 1))
for coor in range(nCoordinates):
    vecInput2[coor * 2] = DummyData[iframe,coor+1]

F = ca.external('F', os.path.join(pathMain, 'examples','Gait2D.dll'))
ID_F = (F(vecInput2)).full().flatten()[:nCoordinates]
print(np.abs(ID_osim - ID_F))
print(np.max(np.abs(ID_osim - ID_F)))
print('finished')