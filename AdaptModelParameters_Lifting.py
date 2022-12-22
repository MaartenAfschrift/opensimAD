# This scripts adapts some model parameters and saves it as osim models
#--------------------------------------------------------------------------


import os
import opensim
import numpy as np
pathOpenSimModel = 'C:/Users/mat950/Documents/Software/Sim/OptimalLifting/OpenSimModel/Gait2D_v43_mtp_arm.osim'
PathOutModels = 'C:/Users/mat950/Documents/Software/Sim/OptimalLifting/OpenSimModel/Batch'

# test if we have to create a folder
if not os.path.exists(PathOutModels):
    os.makedirs(PathOutModels)

# loop over different masses for hand
MassHand = np.arange(0, 15, 0.5) + 0.02
for i in MassHand:
    # get the opensim model
    Mod = opensim.Model(pathOpenSimModel)
    # set the mass of the hand
    Mod.getBodySet().get('hand_l').setMass(i)
    Mod.getBodySet().get('hand_r').setMass(i)
    # save the model
    ModelOutName = 'Gait2D_v43_mtp_arm_m' + str(round(i*2)) + '.osim'
    Mod.printToXML(os.path.join(PathOutModels,ModelOutName))





