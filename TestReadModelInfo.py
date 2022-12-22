import os
import numpy as np
import casadi as ca

pathMain = os.getcwd()
pathExample = os.path.join(pathMain, 'examples')
m1 = np.load(os.path.join(pathExample, 'Gait2DModel_map.npy'), allow_pickle=True).item()

# Test Loading .dll files
nCoordinates = 10
F = ca.external('F', os.path.join(pathExample, 'Gait2D_v43.dll'))
vecInput = np.zeros((nCoordinates * 3, 1))
vecInput[5] = 0.8
ID_F = (F(vecInput)).full().flatten()
print(ID_F)

# Test Loading .dll files
vecInput = np.zeros((nCoordinates * 3, 1))
vecInput[5] = 0.8
vecInput[2] = 50
ID_F2 = (F(vecInput)).full().flatten()
print(ID_F2)
print('verschil')
print(ID_F2 - ID_F)

print(np.max(np.abs(ID_F2 - ID_F)))



# ID with push at multiple segments
#F2 = ca.external('F', os.path.join(pathExample, 'Gait2D_v43_Fext.dll'))
#vecInput = np.zeros((nCoordinates * 3+6, 1))
#vecInput[5] = 1
#vecInput[30] = 100
#vecInput[31] = 200
#ID_F2= (F2(vecInput)).full().flatten()[:nCoordinates]

#print('ID default model')
#print(ID_F)
#print('  ')
#print('ID external force model')
#print(ID_F2)




