import os
import sys
sys.path.append("../..") # utilities in child directory
import opensim
import numpy as np
import casadi as ca
import shutil
import importlib

# # %% Paths.
# scriptDir = os.getcwd()
# dcDir = os.path.dirname(scriptDir) 
# baseDir = os.path.dirname(dcDir) 

# # %% User settings.
# mobilecap = False
# augmenter = '_Augmenter1'
# # OpenSimModel = "gait2392_withArms_weldMTP_weldHand_FK"
# OpenSimModel = "LaiArnoldModified2017_poly_withArms_weldMTP_weldHand"
# session = "Session20210422_0003"

def generateExternalFunction(pathOpenSimModel, pathModelFolder, 
                             OpenSimModel="gait2392", 
                             build_externalFunction=True,
                             compiler="Visual Studio 15 2017 Win64",
                             verifyID=True):
    
    # dcDir = os.path.dirname(scriptDir) 
    # baseDir = os.path.dirname(dcDir) 

    # %% Default settings.
    # Set True to build the external function (.dll). This assumes you have cmake
    # and visual studio installed.
    # build_externalFunction = True
    # compiler="Visual Studio 15 2017 Win64"
    # Set True to verify that the external function returns the same torques as
    # the ID tool together with the .osim file. This check ensures that the
    # model has been built (programmatically) correctly.
    verifyID = True
    # We generate two external functions: a nominal one returning
    # joint torques, and a post-processing one returning joint torques,
    # overall GRF/Ms, and GRF/Ms per sphere.
    functions_toGenerate = ["nominal", "pp"]
        
    '''
        Specify the joint order you will use for the direct collocation problem.
        Also, specify the corresponding coordinate order for sanity check purpose.
    '''
    jointsOrder = ['ground_pelvis', 'hip_l', 'hip_r', 'knee_l', 'knee_r',
                   'ankle_l', 'ankle_r', 'subtalar_l', 'subtalar_r', 'mtp_l',
                   'mtp_r', 'back', 'acromial_l', 'acromial_r', 'elbow_l',
                   'elbow_r', 'radioulnar_l', 'radioulnar_r', 'radius_hand_l',
                   'radius_hand_r']
    
    if 'Rajagopal' in OpenSimModel or 'Lai' in OpenSimModel:
        idx_knee_r = jointsOrder.index('knee_r')
        idx_knee_l = jointsOrder.index('knee_l')
        jointsOrder[idx_knee_r] = 'walker_knee_r'
        jointsOrder[idx_knee_l] = 'walker_knee_l'    
    
    coordinatesOrder = ['pelvis_tilt', 'pelvis_list', 'pelvis_rotation',
                        'pelvis_tx', 'pelvis_ty', 'pelvis_tz', 'hip_flexion_l',
                        'hip_adduction_l', 'hip_rotation_l', 'hip_flexion_r',
                        'hip_adduction_r', 'hip_rotation_r',
                        'knee_angle_l', 'knee_adduction_l', 
                        'knee_angle_r', 'knee_adduction_r',
                        'ankle_angle_l', 'ankle_angle_r',
                        'subtalar_angle_l', 'subtalar_angle_r',
                        'mtp_angle_l', 'mtp_angle_r',
                        'lumbar_extension', 'lumbar_bending', 'lumbar_rotation',
                        'arm_flex_l', 'arm_add_l', 'arm_rot_l',
                        'arm_flex_r', 'arm_add_r', 'arm_rot_r',
                        'elbow_flex_l', 'elbow_flex_r']
    if not 'KA' in OpenSimModel:
        coordinatesOrder.pop(coordinatesOrder.index('knee_adduction_l'))
        coordinatesOrder.pop(coordinatesOrder.index('knee_adduction_r'))
          
    # No longer supported ########################################################
    '''
        Set True to output the position of the markers specified in
        response_markers.
    '''
    exportMarkerPositions = False
    if exportMarkerPositions:
        raise ValueError("No longer supported")
    response_markers = ["C7_study", "r_shoulder_study", "L_shoulder_study",
                        "r.ASIS_study", "L.ASIS_study", "r.PSIS_study", 
                        "L.PSIS_study", "r_knee_study", "L_knee_study",
                        "r_mknee_study", "L_mknee_study", "r_ankle_study", 
                        "L_ankle_study", "r_mankle_study", "L_mankle_study",
                        "r_calc_study", "L_calc_study", "r_toe_study", 
                        "L_toe_study", "r_5meta_study", "L_5meta_study"]
    if "withArms" in OpenSimModel:
        response_markers.append("r_lelbow_study")
        response_markers.append("L_lelbow_study")
        response_markers.append("r_melbow_study")
        response_markers.append("L_melbow_study")
        response_markers.append("r_lwrist_study")
        response_markers.append("L_lwrist_study")
        response_markers.append("r_mwrist_study")
        response_markers.append("L_mwrist_study")
    ##############################################################################
    
    suffix_model = '_contacts'
    outputModelFileName = (OpenSimModel + "_scaled" + suffix_model)
    pathOutputFiles = os.path.join(pathModelFolder, outputModelFileName)
    pathModel = pathOutputFiles + ".osim"
    pathOutputExternalFunctionFolder = os.path.join(pathModelFolder,
                                                    "ExternalFunction")
    os.makedirs(pathOutputExternalFunctionFolder, exist_ok=True)
    
    for function_toGenerate in functions_toGenerate:
        
        generate_pp = False
        generate_grf = False    
        if function_toGenerate == "grf":
            generate_grf = True
        elif function_toGenerate == "pp":
            generate_pp = True
        else:
            if not function_toGenerate == "nominal":
                raise ValueError("Not supported: options are nominal, grf, and pp")
                
        if generate_pp and generate_grf:
            raise ValueError("One or the other (pp vs grf)")
        
        outputCPPFileName = "nominal"
        suffix_cpp = ""
        if exportMarkerPositions:
            suffix_cpp = suffix_cpp + "_markers"
        if generate_pp:
            suffix_cpp = suffix_cpp + "_pp"
        if generate_grf:
            suffix_cpp = suffix_cpp + "_gr"
        outputCPPFileName += suffix_cpp
        
        pathOutputFile = os.path.join(pathOutputExternalFunctionFolder,
                                      outputCPPFileName + ".cpp")
        
        # %% Generate external Function (.cpp file)
        model = opensim.Model(pathModel)
        model.initSystem()
        bodySet = model.getBodySet()
        jointSet = model.get_JointSet()
        nJoints = jointSet.getSize()
        geometrySet = model.get_ContactGeometrySet()
        forceSet = model.get_ForceSet()
        coordinateSet = model.getCoordinateSet()
        nCoordinates = coordinateSet.getSize()
        coordinates = []
        for coor in range(nCoordinates):
            coordinates.append(coordinateSet.get(coor).getName())
        sides = ['r', 'l']
        for side in sides:
            if 'knee_angle_{}_beta'.format(side) in coordinates:
                nCoordinates -= 1
                nJoints -= 1
        
        nContacts = 0
        for i in range(forceSet.getSize()):        
            c_force_elt = forceSet.get(i)        
            if c_force_elt.getConcreteClassName() == "SmoothSphereHalfSpaceForce":  
                nContacts += 1
        
        with open(pathOutputFile, "w") as f:
            
            f.write('#include <OpenSim/Simulation/Model/Model.h>\n')
            f.write('#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>\n')
            f.write('#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>\n')
            f.write('#include <OpenSim/Simulation/SimbodyEngine/Joint.h>\n')
            f.write('#include <OpenSim/Simulation/SimbodyEngine/SpatialTransform.h>\n')
            f.write('#include <OpenSim/Simulation/SimbodyEngine/CustomJoint.h>\n')
            f.write('#include <OpenSim/Common/LinearFunction.h>\n')
            f.write('#include <OpenSim/Common/PolynomialFunction.h>\n')
            f.write('#include <OpenSim/Common/MultiplierFunction.h>\n')
            f.write('#include <OpenSim/Common/Constant.h>\n')
            f.write('#include <OpenSim/Simulation/Model/SmoothSphereHalfSpaceForce.h>\n')
            f.write('#include "SimTKcommon/internal/recorder.h"\n\n')
            
            f.write('#include <iostream>\n')
            f.write('#include <iterator>\n')
            f.write('#include <random>\n')
            f.write('#include <cassert>\n')
            f.write('#include <algorithm>\n')
            f.write('#include <vector>\n')
            f.write('#include <fstream>\n\n')
            
            f.write('using namespace SimTK;\n')
            f.write('using namespace OpenSim;\n\n')
        
            f.write('constexpr int n_in = 2; \n')
            f.write('constexpr int n_out = 1; \n')
            
            f.write('constexpr int nCoordinates = %i; \n' % nCoordinates)
            f.write('constexpr int NX = nCoordinates*2; \n')
            f.write('constexpr int NU = nCoordinates; \n')
            
            if exportMarkerPositions:
                nMarkers = len(response_markers)
                if generate_pp:
                    f.write('constexpr int NR = nCoordinates + 3*%i + 3*4 + 3*2*%i; \n\n' % (nMarkers, nContacts))        
                else:
                    f.write('constexpr int NR = nCoordinates + 3*%i; \n\n' % (nMarkers))        
            else:
                if generate_pp:
                    f.write('constexpr int NR = nCoordinates + 3*4 + 3*2*%i; \n\n' % (nContacts))
                elif generate_grf:
                    f.write('constexpr int NR = nCoordinates + 3*4; \n\n')
                else:
                    f.write('constexpr int NR = nCoordinates; \n\n')
            
            f.write('template<typename T> \n')
            f.write('T value(const Recorder& e) { return e; }; \n')
            f.write('template<> \n')
            f.write('double value(const Recorder& e) { return e.getValue(); }; \n\n')
            
            f.write('SimTK::Array_<int> getIndicesOSInSimbody(const Model& model) { \n')
            f.write('\tauto s = model.getWorkingState(); \n')
            f.write('\tconst auto svNames = model.getStateVariableNames(); \n')
            f.write('\tSimTK::Array_<int> idxOSInSimbody(s.getNQ()); \n')
            f.write('\ts.updQ() = 0; \n')
            f.write('\tfor (int iy = 0; iy < s.getNQ(); ++iy) { \n')
            f.write('\t\ts.updQ()[iy] = SimTK::NaN; \n')
            f.write('\t\tconst auto svValues = model.getStateVariableValues(s); \n')
            f.write('\t\tfor (int isv = 0; isv < svNames.size(); ++isv) { \n')
            f.write('\t\t\tif (SimTK::isNaN(svValues[isv])) { \n')
            f.write('\t\t\t\ts.updQ()[iy] = 0; \n')
            f.write('\t\t\t\tidxOSInSimbody[iy] = isv/2; \n')
            f.write('\t\t\t\tbreak; \n')
            f.write('\t\t\t} \n')
            f.write('\t\t} \n')
            f.write('\t} \n')
            f.write('\treturn idxOSInSimbody; \n')
            f.write('} \n\n')
            
            f.write('SimTK::Array_<int> getIndicesSimbodyInOS(const Model& model) { \n')
            f.write('\tauto idxOSInSimbody = getIndicesOSInSimbody(model); \n')
            f.write('\tauto s = model.getWorkingState(); \n')
            f.write('\tSimTK::Array_<int> idxSimbodyInOS(s.getNQ()); \n')
            f.write('\tfor (int iy = 0; iy < s.getNQ(); ++iy) { \n')
            f.write('\t\tfor (int iyy = 0; iyy < s.getNQ(); ++iyy) { \n')
            f.write('\t\t\tif (idxOSInSimbody[iyy] == iy) { \n')
            f.write('\t\t\t\tidxSimbodyInOS[iy] = iyy; \n')
            f.write('\t\t\t\tbreak; \n')
            f.write('\t\t\t} \n')
            f.write('\t\t} \n')
            f.write('\t} \n')	
            f.write('\treturn idxSimbodyInOS; \n')
            f.write('} \n\n')
            
            f.write('template<typename T>\n')
            f.write('int F_generic(const T** arg, T** res) {\n')
            
            f.write('\tOpenSim::Model* model;\n')
            f.write('\tmodel = new OpenSim::Model();;\n\n')
            
            # Bodies
            f.write('\t// Definition of bodies\n')
            for i in range(bodySet.getSize()):        
                c_body = bodySet.get(i)
                c_body_name = c_body.getName()
                
                if (c_body_name == 'patella_l' or c_body_name == 'patella_r'):
                    continue
                
                c_body_mass = c_body.get_mass()
                c_body_mass_center = c_body.get_mass_center().to_numpy()
                c_body_inertia = c_body.get_inertia()
                c_body_inertia_vec3 = np.array([c_body_inertia.get(0), c_body_inertia.get(1), c_body_inertia.get(2)])        
                f.write('\tOpenSim::Body* %s;\n' % c_body_name)
                f.write('\t%s = new OpenSim::Body(\"%s\", %.20f, Vec3(%.20f, %.20f, %.20f), Inertia(%.20f, %.20f, %.20f, 0., 0., 0.));\n' % (c_body_name, c_body_name, c_body_mass, c_body_mass_center[0], c_body_mass_center[1], c_body_mass_center[2], c_body_inertia_vec3[0], c_body_inertia_vec3[1], c_body_inertia_vec3[2]))
                f.write('\tmodel->addBody(%s);\n' % (c_body_name))
                f.write('\n')   
            
            # Joints
            f.write('\t// Definition of joints\n')
            for i in range(jointSet.getSize()): 
                c_joint = jointSet.get(i)
                c_joint_type = c_joint.getConcreteClassName()
                
                c_joint_name = c_joint.getName()
                if (c_joint_name == 'patellofemoral_l' or 
                    c_joint_name == 'patellofemoral_r'):
                    continue
                
                nJointCoordinates = int(c_joint.getNumStateVariables() / 2)
                
                parent_frame = c_joint.get_frames(0)
                parent_frame_name = parent_frame.getParentFrame().getName()
                parent_frame_trans = parent_frame.get_translation().to_numpy()
                parent_frame_or = parent_frame.get_orientation().to_numpy()
                
                child_frame = c_joint.get_frames(1)
                child_frame_name = child_frame.getParentFrame().getName()
                child_frame_trans = child_frame.get_translation().to_numpy()
                child_frame_or = child_frame.get_orientation().to_numpy()
                
                # Custom joints
                if c_joint_type == "CustomJoint":
                    
                    f.write('\tSpatialTransform st_%s;\n' % c_joint.getName())
                    
                    cObj = opensim.CustomJoint.safeDownCast(c_joint)    
                    spatialtransform = cObj.get_SpatialTransform()
                    
                    # Transform axis.
                    # Rotation 1
                    rot1 = spatialtransform.get_rotation1()
                    rot1_axis = rot1.get_axis().to_numpy()
                    rot1_f = rot1.get_function()
                    coord = 0
                    if rot1_f.getConcreteClassName() == 'LinearFunction':  
                        rot1_f_obj = opensim.LinearFunction.safeDownCast(rot1_f)                          
                        rot1_f_slope = rot1_f_obj.getSlope()
                        rot1_f_intercept = rot1_f_obj.getIntercept()                
                        c_coord = c_joint.get_coordinates(coord)
                        c_coord_name = c_coord.getName()
                        f.write('\tst_%s[%i].setCoordinateNames(OpenSim::Array<std::string>(\"%s\", 1, 1));\n' % (c_joint.getName(), coord, c_coord_name))
                        f.write('\tst_%s[%i].setFunction(new LinearFunction(%.4f, %.4f));\n' % (c_joint.getName(), coord, rot1_f_slope, rot1_f_intercept))                
                    elif rot1_f.getConcreteClassName() == 'PolynomialFunction':
                        f.write('\tst_%s[%i].setCoordinateNames(OpenSim::Array<std::string>(\"%s\", 1, 1));\n' % (c_joint.getName(), coord, c_coord_name))
                        rot1_f_obj = opensim.PolynomialFunction.safeDownCast(rot1_f)                
                        rot1_f_coeffs = rot1_f_obj.getCoefficients().to_numpy()
                        c_nCoeffs = rot1_f_coeffs.shape[0]                
                        if c_nCoeffs == 2:
                            f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, rot1_f_coeffs[0], rot1_f_coeffs[1]))
                        elif c_nCoeffs == 3:
                            f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, rot1_f_coeffs[0], rot1_f_coeffs[1], rot1_f_coeffs[2]))
                        elif c_nCoeffs == 4:
                            f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, rot1_f_coeffs[0], rot1_f_coeffs[1], rot1_f_coeffs[2], rot1_f_coeffs[3]))  
                        elif c_nCoeffs == 5:
                            f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, rot1_f_coeffs[0], rot1_f_coeffs[1], rot1_f_coeffs[2], rot1_f_coeffs[3], rot1_f_coeffs[4]))                    
                        else:
                            raise ValueError("TODO")
                        f.write('\tVector st_%s_%i_coeffs_vec(%i); \n' % (c_joint.getName(), coord, c_nCoeffs))
                        f.write('\tfor (int i = 0; i < %i; ++i) st_%s_%i_coeffs_vec[i] = st_%s_%i_coeffs[i]; \n' % (c_nCoeffs, c_joint.getName(), coord, c_joint.getName(), coord))
                        f.write('\tst_%s[%i].setFunction(new PolynomialFunction(st_%s_%i_coeffs_vec));\n' % (c_joint.getName(), coord, c_joint.getName(), coord))
                    elif rot1_f.getConcreteClassName() == 'MultiplierFunction':
                        rot1_f_obj = opensim.MultiplierFunction.safeDownCast(rot1_f)
                        rot1_f_obj_scale = rot1_f_obj.getScale()
                        rot1_f_obj_f = rot1_f_obj.getFunction()
                        rot1_f_obj_f_name = rot1_f_obj_f.getConcreteClassName()
                        if rot1_f_obj_f_name == 'Constant':
                            rot1_f_obj_f_obj = opensim.Constant.safeDownCast(rot1_f_obj_f)
                            rot1_f_obj_f_obj_value = rot1_f_obj_f_obj.getValue()
                            f.write('\tst_%s[%i].setFunction(new MultiplierFunction(new Constant(%.20f), %.20f));\n' % (c_joint.getName(), coord, rot1_f_obj_f_obj_value, rot1_f_obj_scale))
                        elif rot1_f_obj_f_name == 'PolynomialFunction':
                            f.write('\tst_%s[%i].setCoordinateNames(OpenSim::Array<std::string>(\"%s\", 1, 1));\n' % (c_joint.getName(), coord, c_coord_name))
                            rot1_f_obj_f_obj = opensim.PolynomialFunction.safeDownCast(rot1_f_obj_f)
                            rot1_f_obj_f_coeffs = rot1_f_obj_f_obj.getCoefficients().to_numpy()
                            c_nCoeffs = rot1_f_obj_f_coeffs.shape[0]
                            if c_nCoeffs == 2:
                                f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, rot1_f_obj_f_coeffs[0], rot1_f_obj_f_coeffs[1]))
                            elif c_nCoeffs == 3:
                                f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, rot1_f_obj_f_coeffs[0], rot1_f_obj_f_coeffs[1], rot1_f_obj_f_coeffs[2]))
                            elif c_nCoeffs == 4:
                                f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, rot1_f_obj_f_coeffs[0], rot1_f_obj_f_coeffs[1], rot1_f_obj_f_coeffs[2], rot1_f_obj_f_coeffs[3]))  
                            elif c_nCoeffs == 5:
                                f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, rot1_f_obj_f_coeffs[0], rot1_f_obj_f_coeffs[1], rot1_f_obj_f_coeffs[2], rot1_f_obj_f_coeffs[3], rot1_f_obj_f_coeffs[4]))                    
                            else:
                                raise ValueError("TODO")
                            f.write('\tVector st_%s_%i_coeffs_vec(%i); \n' % (c_joint.getName(), coord, c_nCoeffs))
                            f.write('\tfor (int i = 0; i < %i; ++i) st_%s_%i_coeffs_vec[i] = st_%s_%i_coeffs[i]; \n' % (c_nCoeffs, c_joint.getName(), coord, c_joint.getName(), coord))
                            f.write('\tst_%s[%i].setFunction(new MultiplierFunction(new PolynomialFunction(st_%s_%i_coeffs_vec), %.20f));\n' % (c_joint.getName(), coord, c_joint.getName(), coord, rot1_f_obj_scale))
                        else:
                            raise ValueError("Not supported")
                    elif rot1_f.getConcreteClassName() == 'Constant':
                        raise ValueError("TODO")
                        # f.write('\tst_%s[%i].setFunction(new Constant(0));\n' % (c_joint.getName(), coord))
                    f.write('\tst_%s[%i].setAxis(Vec3(%.20f, %.20f, %.20f));\n' % (c_joint.getName(), coord, rot1_axis[0], rot1_axis[1], rot1_axis[2]))
                    
                    # Rotation 2
                    rot2 = spatialtransform.get_rotation2()
                    rot2_axis = rot2.get_axis().to_numpy()
                    rot2_f = rot2.get_function()
                    coord = 1
                    if rot2_f.getConcreteClassName() == 'LinearFunction':
                        rot2_f_obj = opensim.LinearFunction.safeDownCast(rot2_f)
                        rot2_f_slope = rot2_f_obj.getSlope()
                        rot2_f_intercept = rot2_f_obj.getIntercept()                
                        c_coord = c_joint.get_coordinates(coord)
                        c_coord_name = c_coord.getName()
                        f.write('\tst_%s[%i].setCoordinateNames(OpenSim::Array<std::string>(\"%s\", 1, 1));\n' % (c_joint.getName(), coord, c_coord_name))
                        f.write('\tst_%s[%i].setFunction(new LinearFunction(%.4f, %.4f));\n' % (c_joint.getName(), coord, rot2_f_slope, rot2_f_intercept))
                    elif rot2_f.getConcreteClassName() == 'PolynomialFunction':
                        f.write('\tst_%s[%i].setCoordinateNames(OpenSim::Array<std::string>(\"%s\", 1, 1));\n' % (c_joint.getName(), coord, c_coord_name))
                        rot2_f_obj = opensim.PolynomialFunction.safeDownCast(rot2_f)                
                        rot2_f_coeffs = rot2_f_obj.getCoefficients().to_numpy()
                        c_nCoeffs = rot2_f_coeffs.shape[0]                
                        if c_nCoeffs == 2:
                            f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, rot2_f_coeffs[0], rot2_f_coeffs[1]))
                        elif c_nCoeffs == 3:
                            f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, rot2_f_coeffs[0], rot2_f_coeffs[1], rot2_f_coeffs[2]))
                        elif c_nCoeffs == 4:
                            f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, rot2_f_coeffs[0], rot2_f_coeffs[1], rot2_f_coeffs[2], rot2_f_coeffs[3]))  
                        elif c_nCoeffs == 5:
                            f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, rot2_f_coeffs[0], rot2_f_coeffs[1], rot2_f_coeffs[2], rot2_f_coeffs[3], rot2_f_coeffs[4]))                    
                        else:
                            raise ValueError("TODO")
                        f.write('\tVector st_%s_%i_coeffs_vec(%i); \n' % (c_joint.getName(), coord, c_nCoeffs))
                        f.write('\tfor (int i = 0; i < %i; ++i) st_%s_%i_coeffs_vec[i] = st_%s_%i_coeffs[i]; \n' % (c_nCoeffs, c_joint.getName(), coord, c_joint.getName(), coord))
                        f.write('\tst_%s[%i].setFunction(new PolynomialFunction(st_%s_%i_coeffs_vec));\n' % (c_joint.getName(), coord, c_joint.getName(), coord))
                    elif rot2_f.getConcreteClassName() == 'MultiplierFunction':
                        rot2_f_obj = opensim.MultiplierFunction.safeDownCast(rot2_f)
                        rot2_f_obj_scale = rot2_f_obj.getScale()
                        rot2_f_obj_f = rot2_f_obj.getFunction()
                        rot2_f_obj_f_name = rot2_f_obj_f.getConcreteClassName()
                        if rot2_f_obj_f_name == 'Constant':
                            rot2_f_obj_f_obj = opensim.Constant.safeDownCast(rot2_f_obj_f)
                            rot2_f_obj_f_obj_value = rot2_f_obj_f_obj.getValue()
                            f.write('\tst_%s[%i].setFunction(new MultiplierFunction(new Constant(%.20f), %.20f));\n' % (c_joint.getName(), coord, rot2_f_obj_f_obj_value, rot2_f_obj_scale)) 
                        elif rot2_f_obj_f_name == 'PolynomialFunction':
                            f.write('\tst_%s[%i].setCoordinateNames(OpenSim::Array<std::string>(\"%s\", 1, 1));\n' % (c_joint.getName(), coord, c_coord_name))
                            rot2_f_obj_f_obj = opensim.PolynomialFunction.safeDownCast(rot2_f_obj_f)
                            rot2_f_obj_f_coeffs = rot2_f_obj_f_obj.getCoefficients().to_numpy()
                            c_nCoeffs = rot2_f_obj_f_coeffs.shape[0]
                            if c_nCoeffs == 2:
                                f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, rot2_f_obj_f_coeffs[0], rot2_f_obj_f_coeffs[1]))
                            elif c_nCoeffs == 3:
                                f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, rot2_f_obj_f_coeffs[0], rot2_f_obj_f_coeffs[1], rot2_f_obj_f_coeffs[2]))
                            elif c_nCoeffs == 4:
                                f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, rot2_f_obj_f_coeffs[0], rot2_f_obj_f_coeffs[1], rot2_f_obj_f_coeffs[2], rot2_f_obj_f_coeffs[3]))  
                            elif c_nCoeffs == 5:
                                f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, rot2_f_obj_f_coeffs[0], rot2_f_obj_f_coeffs[1], rot2_f_obj_f_coeffs[2], rot2_f_obj_f_coeffs[3], rot2_f_obj_f_coeffs[4]))                    
                            else:
                                raise ValueError("TODO")
                            f.write('\tVector st_%s_%i_coeffs_vec(%i); \n' % (c_joint.getName(), coord, c_nCoeffs))
                            f.write('\tfor (int i = 0; i < %i; ++i) st_%s_%i_coeffs_vec[i] = st_%s_%i_coeffs[i]; \n' % (c_nCoeffs, c_joint.getName(), coord, c_joint.getName(), coord))
                            f.write('\tst_%s[%i].setFunction(new MultiplierFunction(new PolynomialFunction(st_%s_%i_coeffs_vec), %.20f));\n' % (c_joint.getName(), coord, c_joint.getName(), coord, rot2_f_obj_scale))
                        else:
                            raise ValueError("Not supported")
                    elif rot2_f.getConcreteClassName() == 'Constant':
                        raise ValueError("TODO")
                        # f.write('\tst_%s[%i].setFunction(new Constant(0));\n' % (c_joint.getName(), coord))
                    f.write('\tst_%s[%i].setAxis(Vec3(%.20f, %.20f, %.20f));\n' % (c_joint.getName(), coord, rot2_axis[0], rot2_axis[1], rot2_axis[2]))
                    
                    # Rotation 3
                    rot3 = spatialtransform.get_rotation3()
                    rot3_axis = rot3.get_axis().to_numpy()
                    rot3_f = rot3.get_function()
                    coord = 2
                    if rot3_f.getConcreteClassName() == 'LinearFunction': 
                        rot3_f_obj = opensim.LinearFunction.safeDownCast(rot3_f)
                        rot3_f_slope = rot3_f_obj.getSlope()
                        rot3_f_intercept = rot3_f_obj.getIntercept()                
                        c_coord = c_joint.get_coordinates(coord)
                        c_coord_name = c_coord.getName()
                        f.write('\tst_%s[%i].setCoordinateNames(OpenSim::Array<std::string>(\"%s\", 1, 1));\n' % (c_joint.getName(), coord, c_coord_name))
                        f.write('\tst_%s[%i].setFunction(new LinearFunction(%.4f, %.4f));\n' % (c_joint.getName(), coord, rot3_f_slope, rot3_f_intercept))
                    elif rot3_f.getConcreteClassName() == 'PolynomialFunction':
                        f.write('\tst_%s[%i].setCoordinateNames(OpenSim::Array<std::string>(\"%s\", 1, 1));\n' % (c_joint.getName(), coord, c_coord_name))
                        rot3_f_obj = opensim.PolynomialFunction.safeDownCast(rot3_f)                
                        rot3_f_coeffs = rot3_f_obj.getCoefficients().to_numpy()
                        c_nCoeffs = rot3_f_coeffs.shape[0]                
                        if c_nCoeffs == 2:
                            f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, rot3_f_coeffs[0], rot3_f_coeffs[1]))
                        elif c_nCoeffs == 3:
                            f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, rot3_f_coeffs[0], rot3_f_coeffs[1], rot3_f_coeffs[2]))
                        elif c_nCoeffs == 4:
                            f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, rot3_f_coeffs[0], rot3_f_coeffs[1], rot3_f_coeffs[2], rot3_f_coeffs[3]))  
                        elif c_nCoeffs == 5:
                            f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, rot3_f_coeffs[0], rot3_f_coeffs[1], rot3_f_coeffs[2], rot3_f_coeffs[3], rot3_f_coeffs[4]))                    
                        else:
                            raise ValueError("TODO")
                        f.write('\tVector st_%s_%i_coeffs_vec(%i); \n' % (c_joint.getName(), coord, c_nCoeffs))
                        f.write('\tfor (int i = 0; i < %i; ++i) st_%s_%i_coeffs_vec[i] = st_%s_%i_coeffs[i]; \n' % (c_nCoeffs, c_joint.getName(), coord, c_joint.getName(), coord))
                        f.write('\tst_%s[%i].setFunction(new PolynomialFunction(st_%s_%i_coeffs_vec));\n' % (c_joint.getName(), coord, c_joint.getName(), coord))
                    elif rot3_f.getConcreteClassName() == 'MultiplierFunction':
                        rot3_f_obj = opensim.MultiplierFunction.safeDownCast(rot3_f)
                        rot3_f_obj_scale = rot3_f_obj.getScale()
                        rot3_f_obj_f = rot3_f_obj.getFunction()
                        rot3_f_obj_f_name = rot3_f_obj_f.getConcreteClassName()
                        if rot3_f_obj_f_name == 'Constant':
                            rot3_f_obj_f_obj = opensim.Constant.safeDownCast(rot3_f_obj_f)
                            rot3_f_obj_f_obj_value = rot3_f_obj_f_obj.getValue()
                            f.write('\tst_%s[%i].setFunction(new MultiplierFunction(new Constant(%.20f), %.20f));\n' % (c_joint.getName(), coord, rot3_f_obj_f_obj_value, rot3_f_obj_scale))
                        elif rot3_f_obj_f_name == 'PolynomialFunction':
                            f.write('\tst_%s[%i].setCoordinateNames(OpenSim::Array<std::string>(\"%s\", 1, 1));\n' % (c_joint.getName(), coord, c_coord_name))
                            rot3_f_obj_f_obj = opensim.PolynomialFunction.safeDownCast(rot3_f_obj_f)
                            rot3_f_obj_f_coeffs = rot3_f_obj_f_obj.getCoefficients().to_numpy()
                            c_nCoeffs = rot3_f_obj_f_coeffs.shape[0]
                            if c_nCoeffs == 2:
                                f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, rot3_f_obj_f_coeffs[0], rot3_f_obj_f_coeffs[1]))
                            elif c_nCoeffs == 3:
                                f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, rot3_f_obj_f_coeffs[0], rot3_f_obj_f_coeffs[1], rot3_f_obj_f_coeffs[2]))
                            elif c_nCoeffs == 4:
                                f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, rot3_f_obj_f_coeffs[0], rot3_f_obj_f_coeffs[1], rot3_f_obj_f_coeffs[2], rot3_f_obj_f_coeffs[3]))  
                            elif c_nCoeffs == 5:
                                f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, rot3_f_obj_f_coeffs[0], rot3_f_obj_f_coeffs[1], rot3_f_obj_f_coeffs[2], rot3_f_obj_f_coeffs[3], rot3_f_obj_f_coeffs[4]))                    
                            else:
                                raise ValueError("TODO")
                            f.write('\tVector st_%s_%i_coeffs_vec(%i); \n' % (c_joint.getName(), coord, c_nCoeffs))
                            f.write('\tfor (int i = 0; i < %i; ++i) st_%s_%i_coeffs_vec[i] = st_%s_%i_coeffs[i]; \n' % (c_nCoeffs, c_joint.getName(), coord, c_joint.getName(), coord))
                            f.write('\tst_%s[%i].setFunction(new MultiplierFunction(new PolynomialFunction(st_%s_%i_coeffs_vec), %.20f));\n' % (c_joint.getName(), coord, c_joint.getName(), coord, rot3_f_obj_scale))
                        else:
                            raise ValueError("Not supported")
                    elif rot3_f.getConcreteClassName() == 'Constant':
                        raise ValueError("TODO")
                        # f.write('\tst_%s[%i].setFunction(new Constant(0));\n' % (c_joint.getName(), coord))            
                    f.write('\tst_%s[%i].setAxis(Vec3(%.20f, %.20f, %.20f));\n' % (c_joint.getName(), coord, rot3_axis[0], rot3_axis[1], rot3_axis[2]))
                    
                    # Translation 1
                    tr1 = spatialtransform.get_translation1()
                    tr1_axis = tr1.get_axis().to_numpy()
                    tr1_f = tr1.get_function()
                    coord = 3
                    if tr1_f.getConcreteClassName() == 'LinearFunction':    
                        tr1_f_obj = opensim.LinearFunction.safeDownCast(tr1_f)
                        tr1_f_slope = tr1_f_obj.getSlope()
                        tr1_f_intercept = tr1_f_obj.getIntercept()                
                        c_coord = c_joint.get_coordinates(coord)
                        c_coord_name = c_coord.getName()
                        f.write('\tst_%s[%i].setCoordinateNames(OpenSim::Array<std::string>(\"%s\", 1, 1));\n' % (c_joint.getName(), coord, c_coord_name))
                        f.write('\tst_%s[%i].setFunction(new LinearFunction(%.4f, %.4f));\n' % (c_joint.getName(), coord, tr1_f_slope, tr1_f_intercept))
                    elif tr1_f.getConcreteClassName() == 'PolynomialFunction':
                        f.write('\tst_%s[%i].setCoordinateNames(OpenSim::Array<std::string>(\"%s\", 1, 1));\n' % (c_joint.getName(), coord, c_coord_name))
                        tr1_f_obj = opensim.PolynomialFunction.safeDownCast(tr1_f)                
                        tr1_f_coeffs = tr1_f_obj.getCoefficients().to_numpy()
                        c_nCoeffs = tr1_f_coeffs.shape[0]                
                        if c_nCoeffs == 2:
                            f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, tr1_f_coeffs[0], tr1_f_coeffs[1]))
                        elif c_nCoeffs == 3:
                            f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, tr1_f_coeffs[0], tr1_f_coeffs[1], tr1_f_coeffs[2]))
                        elif c_nCoeffs == 4:
                            f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, tr1_f_coeffs[0], tr1_f_coeffs[1], tr1_f_coeffs[2], tr1_f_coeffs[3]))  
                        elif c_nCoeffs == 5:
                            f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, tr1_f_coeffs[0], tr1_f_coeffs[1], tr1_f_coeffs[2], tr1_f_coeffs[3], tr1_f_coeffs[4]))                    
                        else:
                            raise ValueError("TODO")
                        f.write('\tVector st_%s_%i_coeffs_vec(%i); \n' % (c_joint.getName(), coord, c_nCoeffs))
                        f.write('\tfor (int i = 0; i < %i; ++i) st_%s_%i_coeffs_vec[i] = st_%s_%i_coeffs[i]; \n' % (c_nCoeffs, c_joint.getName(), coord, c_joint.getName(), coord))
                        f.write('\tst_%s[%i].setFunction(new PolynomialFunction(st_%s_%i_coeffs_vec));\n' % (c_joint.getName(), coord, c_joint.getName(), coord))
                    elif tr1_f.getConcreteClassName() == 'MultiplierFunction':
                        tr1_f_obj = opensim.MultiplierFunction.safeDownCast(tr1_f)
                        tr1_f_obj_scale = tr1_f_obj.getScale()
                        tr1_f_obj_f = tr1_f_obj.getFunction()
                        tr1_f_obj_f_name = tr1_f_obj_f.getConcreteClassName()
                        if tr1_f_obj_f_name == 'Constant':
                            tr1_f_obj_f_obj = opensim.Constant.safeDownCast(tr1_f_obj_f)
                            tr1_f_obj_f_obj_value = tr1_f_obj_f_obj.getValue()
                            f.write('\tst_%s[%i].setFunction(new MultiplierFunction(new Constant(%.20f), %.20f));\n' % (c_joint.getName(), coord, tr1_f_obj_f_obj_value, tr1_f_obj_scale))
                        elif tr1_f_obj_f_name == 'PolynomialFunction':
                            f.write('\tst_%s[%i].setCoordinateNames(OpenSim::Array<std::string>(\"%s\", 1, 1));\n' % (c_joint.getName(), coord, c_coord_name))
                            tr1_f_obj_f_obj = opensim.PolynomialFunction.safeDownCast(tr1_f_obj_f)
                            tr1_f_obj_f_coeffs = tr1_f_obj_f_obj.getCoefficients().to_numpy()
                            c_nCoeffs = tr1_f_obj_f_coeffs.shape[0]
                            if c_nCoeffs == 2:
                                f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, tr1_f_obj_f_coeffs[0], tr1_f_obj_f_coeffs[1]))
                            elif c_nCoeffs == 3:
                                f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, tr1_f_obj_f_coeffs[0], tr1_f_obj_f_coeffs[1], tr1_f_obj_f_coeffs[2]))
                            elif c_nCoeffs == 4:
                                f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, tr1_f_obj_f_coeffs[0], tr1_f_obj_f_coeffs[1], tr1_f_obj_f_coeffs[2], tr1_f_obj_f_coeffs[3]))  
                            elif c_nCoeffs == 5:
                                f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, tr1_f_obj_f_coeffs[0], tr1_f_obj_f_coeffs[1], tr1_f_obj_f_coeffs[2], tr1_f_obj_f_coeffs[3], tr1_f_obj_f_coeffs[4]))                    
                            else:
                                raise ValueError("TODO")
                            f.write('\tVector st_%s_%i_coeffs_vec(%i); \n' % (c_joint.getName(), coord, c_nCoeffs))
                            f.write('\tfor (int i = 0; i < %i; ++i) st_%s_%i_coeffs_vec[i] = st_%s_%i_coeffs[i]; \n' % (c_nCoeffs, c_joint.getName(), coord, c_joint.getName(), coord))
                            f.write('\tst_%s[%i].setFunction(new MultiplierFunction(new PolynomialFunction(st_%s_%i_coeffs_vec), %.20f));\n' % (c_joint.getName(), coord, c_joint.getName(), coord, tr1_f_obj_scale))
                        else:
                            raise ValueError("Not supported")
                    elif tr1_f.getConcreteClassName() == 'Constant':
                        raise ValueError("TODO")
                        # f.write('\tst_%s[%i].setFunction(new Constant(0));\n' % (c_joint.getName(), coord))                
                    f.write('\tst_%s[%i].setAxis(Vec3(%.20f, %.20f, %.20f));\n' % (c_joint.getName(), coord, tr1_axis[0], tr1_axis[1], tr1_axis[2]))            
                    
                    # Translation 2
                    tr2 = spatialtransform.get_translation2()
                    tr2_axis = tr2.get_axis().to_numpy()
                    tr2_f = tr2.get_function()
                    coord = 4
                    if tr2_f.getConcreteClassName() == 'LinearFunction': 
                        tr2_f_obj = opensim.LinearFunction.safeDownCast(tr2_f)
                        tr2_f_slope = tr2_f_obj.getSlope()
                        tr2_f_intercept = tr2_f_obj.getIntercept()                
                        c_coord = c_joint.get_coordinates(coord)
                        c_coord_name = c_coord.getName()
                        f.write('\tst_%s[%i].setCoordinateNames(OpenSim::Array<std::string>(\"%s\", 1, 1));\n' % (c_joint.getName(), coord, c_coord_name))
                        f.write('\tst_%s[%i].setFunction(new LinearFunction(%.4f, %.4f));\n' % (c_joint.getName(), coord, tr2_f_slope, tr2_f_intercept))
                    elif tr2_f.getConcreteClassName() == 'PolynomialFunction':
                        f.write('\tst_%s[%i].setCoordinateNames(OpenSim::Array<std::string>(\"%s\", 1, 1));\n' % (c_joint.getName(), coord, c_coord_name))
                        tr2_f_obj = opensim.PolynomialFunction.safeDownCast(tr2_f)                
                        tr2_f_coeffs = tr2_f_obj.getCoefficients().to_numpy()
                        c_nCoeffs = tr2_f_coeffs.shape[0]                
                        if c_nCoeffs == 2:
                            f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, tr2_f_coeffs[0], tr2_f_coeffs[1]))
                        elif c_nCoeffs == 3:
                            f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, tr2_f_coeffs[0], tr2_f_coeffs[1], tr2_f_coeffs[2]))
                        elif c_nCoeffs == 4:
                            f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, tr2_f_coeffs[0], tr2_f_coeffs[1], tr2_f_coeffs[2], tr2_f_coeffs[3]))  
                        elif c_nCoeffs == 5:
                            f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, tr2_f_coeffs[0], tr2_f_coeffs[1], tr2_f_coeffs[2], tr2_f_coeffs[3], tr2_f_coeffs[4]))                    
                        else:
                            raise ValueError("TODO")
                        f.write('\tVector st_%s_%i_coeffs_vec(%i); \n' % (c_joint.getName(), coord, c_nCoeffs))
                        f.write('\tfor (int i = 0; i < %i; ++i) st_%s_%i_coeffs_vec[i] = st_%s_%i_coeffs[i]; \n' % (c_nCoeffs, c_joint.getName(), coord, c_joint.getName(), coord))
                        f.write('\tst_%s[%i].setFunction(new PolynomialFunction(st_%s_%i_coeffs_vec));\n' % (c_joint.getName(), coord, c_joint.getName(), coord))
                    elif tr2_f.getConcreteClassName() == 'MultiplierFunction':
                        tr2_f_obj = opensim.MultiplierFunction.safeDownCast(tr2_f)
                        tr2_f_obj_scale = tr2_f_obj.getScale()
                        tr2_f_obj_f = tr2_f_obj.getFunction()
                        tr2_f_obj_f_name = tr2_f_obj_f.getConcreteClassName()
                        if tr2_f_obj_f_name == 'Constant':
                            tr2_f_obj_f_obj = opensim.Constant.safeDownCast(tr2_f_obj_f)
                            tr2_f_obj_f_obj_value = tr2_f_obj_f_obj.getValue()
                            f.write('\tst_%s[%i].setFunction(new MultiplierFunction(new Constant(%.20f), %.20f));\n' % (c_joint.getName(), coord, tr2_f_obj_f_obj_value, tr2_f_obj_scale))
                        elif tr2_f_obj_f_name == 'PolynomialFunction':
                            f.write('\tst_%s[%i].setCoordinateNames(OpenSim::Array<std::string>(\"%s\", 1, 1));\n' % (c_joint.getName(), coord, c_coord_name))
                            tr2_f_obj_f_obj = opensim.PolynomialFunction.safeDownCast(tr2_f_obj_f)
                            tr2_f_obj_f_coeffs = tr2_f_obj_f_obj.getCoefficients().to_numpy()
                            c_nCoeffs = tr2_f_obj_f_coeffs.shape[0]
                            if c_nCoeffs == 2:
                                f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, tr2_f_obj_f_coeffs[0], tr2_f_obj_f_coeffs[1]))
                            elif c_nCoeffs == 3:
                                f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, tr2_f_obj_f_coeffs[0], tr2_f_obj_f_coeffs[1], tr2_f_obj_f_coeffs[2]))
                            elif c_nCoeffs == 4:
                                f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, tr2_f_obj_f_coeffs[0], tr2_f_obj_f_coeffs[1], tr2_f_obj_f_coeffs[2], tr2_f_obj_f_coeffs[3]))  
                            elif c_nCoeffs == 5:
                                f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, tr2_f_obj_f_coeffs[0], tr2_f_obj_f_coeffs[1], tr2_f_obj_f_coeffs[2], tr2_f_obj_f_coeffs[3], tr2_f_obj_f_coeffs[4]))                    
                            else:
                                raise ValueError("TODO")
                            f.write('\tVector st_%s_%i_coeffs_vec(%i); \n' % (c_joint.getName(), coord, c_nCoeffs))
                            f.write('\tfor (int i = 0; i < %i; ++i) st_%s_%i_coeffs_vec[i] = st_%s_%i_coeffs[i]; \n' % (c_nCoeffs, c_joint.getName(), coord, c_joint.getName(), coord))
                            f.write('\tst_%s[%i].setFunction(new MultiplierFunction(new PolynomialFunction(st_%s_%i_coeffs_vec), %.20f));\n' % (c_joint.getName(), coord, c_joint.getName(), coord, tr2_f_obj_scale))
                        else:
                            raise ValueError("Not supported")
                    elif tr2_f.getConcreteClassName() == 'Constant':
                        raise ValueError("TODO")
                        # f.write('\tst_%s[%i].setFunction(new Constant(0));\n' % (c_joint.getName(), coord))
                    f.write('\tst_%s[%i].setAxis(Vec3(%.20f, %.20f, %.20f));\n' % (c_joint.getName(), coord, tr2_axis[0], tr2_axis[1], tr2_axis[2]))
                    
                    # Translation 3
                    tr3 = spatialtransform.get_translation3()
                    tr3_axis = tr3.get_axis().to_numpy()
                    tr3_f = tr3.get_function()
                    coord = 5
                    if tr3_f.getConcreteClassName() == 'LinearFunction':     
                        tr3_f_obj = opensim.LinearFunction.safeDownCast(tr3_f)
                        tr3_f_slope = tr3_f_obj.getSlope()
                        tr3_f_intercept = tr3_f_obj.getIntercept()                
                        c_coord = c_joint.get_coordinates(coord)
                        c_coord_name = c_coord.getName()
                        f.write('\tst_%s[%i].setCoordinateNames(OpenSim::Array<std::string>(\"%s\", 1, 1));\n' % (c_joint.getName(), coord, c_coord_name))
                        f.write('\tst_%s[%i].setFunction(new LinearFunction(%.4f, %.4f));\n' % (c_joint.getName(), coord, tr3_f_slope, tr3_f_intercept))
                    elif tr3_f.getConcreteClassName() == 'PolynomialFunction':
                        f.write('\tst_%s[%i].setCoordinateNames(OpenSim::Array<std::string>(\"%s\", 1, 1));\n' % (c_joint.getName(), coord, c_coord_name))
                        tr3_f_obj = opensim.PolynomialFunction.safeDownCast(tr3_f)                
                        tr3_f_coeffs = tr3_f_obj.getCoefficients().to_numpy()
                        c_nCoeffs = tr3_f_coeffs.shape[0]                
                        if c_nCoeffs == 2:
                            f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, tr3_f_coeffs[0], tr3_f_coeffs[1]))
                        elif c_nCoeffs == 3:
                            f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, tr3_f_coeffs[0], tr3_f_coeffs[1], tr3_f_coeffs[2]))
                        elif c_nCoeffs == 4:
                            f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, tr3_f_coeffs[0], tr3_f_coeffs[1], tr3_f_coeffs[2], tr3_f_coeffs[3]))  
                        elif c_nCoeffs == 5:
                            f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, tr3_f_coeffs[0], tr3_f_coeffs[1], tr3_f_coeffs[2], tr3_f_coeffs[3], tr3_f_coeffs[4]))                    
                        else:
                            raise ValueError("TODO")
                        f.write('\tVector st_%s_%i_coeffs_vec(%i); \n' % (c_joint.getName(), coord, c_nCoeffs))
                        f.write('\tfor (int i = 0; i < %i; ++i) st_%s_%i_coeffs_vec[i] = st_%s_%i_coeffs[i]; \n' % (c_nCoeffs, c_joint.getName(), coord, c_joint.getName(), coord))
                        f.write('\tst_%s[%i].setFunction(new PolynomialFunction(st_%s_%i_coeffs_vec));\n' % (c_joint.getName(), coord, c_joint.getName(), coord))
                    elif tr3_f.getConcreteClassName() == 'MultiplierFunction':
                        tr3_f_obj = opensim.MultiplierFunction.safeDownCast(tr3_f)
                        tr3_f_obj_scale = tr3_f_obj.getScale()
                        tr3_f_obj_f = tr3_f_obj.getFunction()
                        tr3_f_obj_f_name = tr3_f_obj_f.getConcreteClassName()
                        if tr3_f_obj_f_name == 'Constant':
                            tr3_f_obj_f_obj = opensim.Constant.safeDownCast(tr3_f_obj_f)
                            tr3_f_obj_f_obj_value = tr3_f_obj_f_obj.getValue()
                            f.write('\tst_%s[%i].setFunction(new MultiplierFunction(new Constant(%.20f), %.20f));\n' % (c_joint.getName(), coord, tr3_f_obj_f_obj_value, tr3_f_obj_scale))
                        elif tr3_f_obj_f_name == 'PolynomialFunction':
                            f.write('\tst_%s[%i].setCoordinateNames(OpenSim::Array<std::string>(\"%s\", 1, 1));\n' % (c_joint.getName(), coord, c_coord_name))
                            tr3_f_obj_f_obj = opensim.PolynomialFunction.safeDownCast(tr3_f_obj_f)
                            tr3_f_obj_f_coeffs = tr3_f_obj_f_obj.getCoefficients().to_numpy()
                            c_nCoeffs = tr3_f_obj_f_coeffs.shape[0]
                            if c_nCoeffs == 2:
                                f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, tr3_f_obj_f_coeffs[0], tr3_f_obj_f_coeffs[1]))
                            elif c_nCoeffs == 3:
                                f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, tr3_f_obj_f_coeffs[0], tr3_f_obj_f_coeffs[1], tr3_f_obj_f_coeffs[2]))
                            elif c_nCoeffs == 4:
                                f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, tr3_f_obj_f_coeffs[0], tr3_f_obj_f_coeffs[1], tr3_f_obj_f_coeffs[2], tr3_f_obj_f_coeffs[3]))  
                            elif c_nCoeffs == 5:
                                f.write('\tosim_double_adouble st_%s_%i_coeffs[%i] = {%.20f, %.20f, %.20f, %.20f, %.20f}; \n' % (c_joint.getName(), coord, c_nCoeffs, tr3_f_obj_f_coeffs[0], tr3_f_obj_f_coeffs[1], tr3_f_obj_f_coeffs[2], tr3_f_obj_f_coeffs[3], tr3_f_obj_f_coeffs[4]))                    
                            else:
                                raise ValueError("TODO")
                            f.write('\tVector st_%s_%i_coeffs_vec(%i); \n' % (c_joint.getName(), coord, c_nCoeffs))
                            f.write('\tfor (int i = 0; i < %i; ++i) st_%s_%i_coeffs_vec[i] = st_%s_%i_coeffs[i]; \n' % (c_nCoeffs, c_joint.getName(), coord, c_joint.getName(), coord))
                            f.write('\tst_%s[%i].setFunction(new MultiplierFunction(new PolynomialFunction(st_%s_%i_coeffs_vec), %.20f));\n' % (c_joint.getName(), coord, c_joint.getName(), coord, tr3_f_obj_scale))
                        else:
                            raise ValueError("Not supported") 
                    elif tr3_f.getConcreteClassName() == 'Constant':
                        raise ValueError("TODO")
                        # f.write('\tst_%s[%i].setFunction(new Constant(0));\n' % (c_joint.getName(), coord))
                    f.write('\tst_%s[%i].setAxis(Vec3(%.20f, %.20f, %.20f));\n' % (c_joint.getName(), coord, tr3_axis[0], tr3_axis[1], tr3_axis[2]))          
                    
                    # Joint.
                    f.write('\tOpenSim::%s* %s;\n' % (c_joint_type, c_joint.getName()))
                    if parent_frame_name == "ground":
                        f.write('\t%s = new OpenSim::%s(\"%s\", model->getGround(), Vec3(%.20f, %.20f, %.20f), Vec3(%.20f, %.20f, %.20f), *%s, Vec3(%.20f, %.20f, %.20f), Vec3(%.20f, %.20f, %.20f), st_%s);\n' % (c_joint.getName(), c_joint_type, c_joint.getName(), parent_frame_trans[0], parent_frame_trans[1], parent_frame_trans[2], parent_frame_or[0], parent_frame_or[1], parent_frame_or[2], child_frame_name, child_frame_trans[0], child_frame_trans[1], child_frame_trans[2], child_frame_or[0], child_frame_or[1], child_frame_or[2], c_joint.getName()))     
                    else:
                        f.write('\t%s = new OpenSim::%s(\"%s\", *%s, Vec3(%.20f, %.20f, %.20f), Vec3(%.20f, %.20f, %.20f), *%s, Vec3(%.20f, %.20f, %.20f), Vec3(%.20f, %.20f, %.20f), st_%s);\n' % (c_joint.getName(), c_joint_type, c_joint.getName(), parent_frame_name, parent_frame_trans[0], parent_frame_trans[1], parent_frame_trans[2], parent_frame_or[0], parent_frame_or[1], parent_frame_or[2], child_frame_name, child_frame_trans[0], child_frame_trans[1], child_frame_trans[2], child_frame_or[0], child_frame_or[1], child_frame_or[2], c_joint.getName()))
                    
                elif c_joint_type == 'PinJoint' or c_joint_type == 'WeldJoint' :
                    f.write('\tOpenSim::%s* %s;\n' % (c_joint_type, c_joint.getName()))
                    if parent_frame_name == "ground":
                        f.write('\t%s = new OpenSim::%s(\"%s\", model->getGround(), Vec3(%.20f, %.20f, %.20f), Vec3(%.20f, %.20f, %.20f), *%s, Vec3(%.20f, %.20f, %.20f), Vec3(%.20f, %.20f, %.20f));\n' % (c_joint.getName(), c_joint_type, c_joint.getName(), parent_frame_trans[0], parent_frame_trans[1], parent_frame_trans[2], parent_frame_or[0], parent_frame_or[1], parent_frame_or[2], child_frame_name, child_frame_trans[0], child_frame_trans[1], child_frame_trans[2], child_frame_or[0], child_frame_or[1], child_frame_or[2]))     
                    else:
                        f.write('\t%s = new OpenSim::%s(\"%s\", *%s, Vec3(%.20f, %.20f, %.20f), Vec3(%.20f, %.20f, %.20f), *%s, Vec3(%.20f, %.20f, %.20f), Vec3(%.20f, %.20f, %.20f));\n' % (c_joint.getName(), c_joint_type, c_joint.getName(), parent_frame_name, parent_frame_trans[0], parent_frame_trans[1], parent_frame_trans[2], parent_frame_or[0], parent_frame_or[1], parent_frame_or[2], child_frame_name, child_frame_trans[0], child_frame_trans[1], child_frame_trans[2], child_frame_or[0], child_frame_or[1], child_frame_or[2])) 
                else:
                    raise ValueError("TODO: joint type not yet supported")
                
                f.write('\n')
            # Add joints to model in pre-defined order
            if jointsOrder:
                for jointOrder in jointsOrder: 
                    f.write('\tmodel->addJoint(%s);\n' % (jointOrder))
                    try:
                        c_joint = jointSet.get(jointOrder)
                    except:
                        raise ValueError("Joint from jointOrder not in jointSet")                
                assert(len(jointsOrder) == nJoints), "jointsOrder and jointSet have different sizes"
            f.write('\n')    
                    
            # Contacts
            f.write('\t// Definition of contacts\n')   
            for i in range(forceSet.getSize()):        
                c_force_elt = forceSet.get(i)        
                if c_force_elt.getConcreteClassName() == "SmoothSphereHalfSpaceForce":            
                    c_force_elt_obj =  opensim.SmoothSphereHalfSpaceForce.safeDownCast(c_force_elt) 	
                    
                    socket0Name = c_force_elt.getSocketNames()[0]
                    socket0 = c_force_elt.getSocket(socket0Name)
                    socket0_obj = socket0.getConnecteeAsObject()
                    socket0_objName = socket0_obj.getName()            
                    geo0 = geometrySet.get(socket0_objName)
                    geo0_loc = geo0.get_location().to_numpy()
                    geo0_or = geo0.get_orientation().to_numpy()
                    geo0_frameName = geo0.getFrame().getName()
                    
                    socket1Name = c_force_elt.getSocketNames()[1]
                    socket1 = c_force_elt.getSocket(socket1Name)
                    socket1_obj = socket1.getConnecteeAsObject()
                    socket1_objName = socket1_obj.getName()            
                    geo1 = geometrySet.get(socket1_objName)
                    geo1_loc = geo1.get_location().to_numpy()
                    geo1_or = geo1.get_orientation().to_numpy()
                    geo1_frameName = geo1.getFrame().getName()
                    obj = opensim.ContactSphere.safeDownCast(geo1) 	
                    geo1_radius = obj.getRadius()            
                    
                    f.write('\tOpenSim::%s* %s;\n' % (c_force_elt.getConcreteClassName(), c_force_elt.getName()))
                    if geo0_frameName == "ground":
                        f.write('\t%s = new %s(\"%s\", *%s, model->getGround());\n' % (c_force_elt.getName(), c_force_elt.getConcreteClassName(), c_force_elt.getName(), geo1_frameName))
                    else:
                        f.write('\t%s = new %s(\"%s\", *%s, *%s);\n' % (c_force_elt.getName(), c_force_elt.getConcreteClassName(), c_force_elt.getName(), geo1_frameName, geo0_frameName))
                        
                    f.write('\tVec3 %s_location(%.20f, %.20f, %.20f);\n' % (c_force_elt.getName(), geo1_loc[0], geo1_loc[1], geo1_loc[2]))
                    f.write('\t%s->set_contact_sphere_location(%s_location);\n' % (c_force_elt.getName(), c_force_elt.getName()))
                    f.write('\tdouble %s_radius = (%.20f);\n' % (c_force_elt.getName(), geo1_radius))
                    f.write('\t%s->set_contact_sphere_radius(%s_radius );\n' % (c_force_elt.getName(), c_force_elt.getName()))
                    f.write('\t%s->set_contact_half_space_location(Vec3(%.20f, %.20f, %.20f));\n' % (c_force_elt.getName(), geo0_loc[0], geo0_loc[1], geo0_loc[2]))
                    f.write('\t%s->set_contact_half_space_orientation(Vec3(%.20f, %.20f, %.20f));\n' % (c_force_elt.getName(), geo0_or[0], geo0_or[1], geo0_or[2]))
                    
                    f.write('\t%s->set_stiffness(%.20f);\n' % (c_force_elt.getName(), c_force_elt_obj.get_stiffness()))
                    f.write('\t%s->set_dissipation(%.20f);\n' % (c_force_elt.getName(), c_force_elt_obj.get_dissipation()))
                    f.write('\t%s->set_static_friction(%.20f);\n' % (c_force_elt.getName(), c_force_elt_obj.get_static_friction()))
                    f.write('\t%s->set_dynamic_friction(%.20f);\n' % (c_force_elt.getName(), c_force_elt_obj.get_dynamic_friction()))
                    f.write('\t%s->set_viscous_friction(%.20f);\n' % (c_force_elt.getName(), c_force_elt_obj.get_viscous_friction()))
                    f.write('\t%s->set_transition_velocity(%.20f);\n' % (c_force_elt.getName(), c_force_elt_obj.get_transition_velocity()))
                    
                    f.write('\t%s->connectSocket_sphere_frame(*%s);\n' % (c_force_elt.getName(), geo1_frameName))
                    if geo0_frameName == "ground":
                        f.write('\t%s->connectSocket_half_space_frame(model->getGround());\n' % (c_force_elt.getName()))                
                    else:
                        f.write('\t%s->connectSocket_half_space_frame(*%s);\n' % (c_force_elt.getName(), geo0_frameName))
                    f.write('\tmodel->addComponent(%s);\n' % (c_force_elt.getName()))
                    f.write('\n')
                    
            # Markers
            if exportMarkerPositions:
                markerSet = model.get_MarkerSet()
                for marker in response_markers: 
                    if "." in marker:
                        marker_adj = marker.replace(".", "_")
                    else:
                        marker_adj = marker                
                    c_marker_loc = markerSet.get(marker).get_location().to_numpy()
                    c_marker_parent = markerSet.get(marker).getParentFrame().getName()            
                    f.write('\tOpenSim::Station* %s;\n' % marker_adj)
                    f.write('\t%s = new Station(*%s, Vec3(%.20f, %.20f, %.20f));\n' % (marker_adj, c_marker_parent, c_marker_loc[0], c_marker_loc[1], c_marker_loc[2]))
                    f.write('\tmodel->addComponent(%s);\n' % (marker_adj))
                f.write('\n')
                    
            f.write('\t// Initialize system.\n')
            f.write('\tSimTK::State* state;\n')
            f.write('\tstate = new State(model->initSystem());\n\n')
        
            f.write('\t// Read inputs.\n')
            f.write('\tstd::vector<T> x(arg[0], arg[0] + NX);\n')
            f.write('\tstd::vector<T> u(arg[1], arg[1] + NU);\n\n')
            
            f.write('\t// States and controls.\n')
            f.write('\tT ua[NU];\n')
            f.write('\tVector QsUs(NX);\n')
            f.write('\t/// States\n')
            f.write('\tfor (int i = 0; i < NX; ++i) QsUs[i] = x[i];\n') 	
            f.write('\t/// Controls\n')
            f.write('\t/// OpenSim and Simbody have different state orders.\n')
            f.write('\tauto indicesOSInSimbody = getIndicesOSInSimbody(*model);\n')
            f.write('\tfor (int i = 0; i < NU; ++i) ua[i] = u[indicesOSInSimbody[i]];\n\n')
        
            f.write('\t// Set state variables and realize.\n')
            f.write('\tmodel->setStateVariableValues(*state, QsUs);\n')
            f.write('\tmodel->realizeVelocity(*state);\n\n')
            
            f.write('\t// Compute residual forces.\n')
            f.write('\t/// Set appliedMobilityForces (# mobilities).\n')
            f.write('\tVector appliedMobilityForces(nCoordinates);\n')
            f.write('\tappliedMobilityForces.setToZero();\n')
            f.write('\t/// Set appliedBodyForces (# bodies + ground).\n')
            f.write('\tVector_<SpatialVec> appliedBodyForces;\n')
            f.write('\tint nbodies = model->getBodySet().getSize() + 1;\n')
            f.write('\tappliedBodyForces.resize(nbodies);\n')
            f.write('\tappliedBodyForces.setToZero();\n')
            f.write('\t/// Set gravity.\n')
            f.write('\tVec3 gravity(0);\n')
            f.write('\tgravity[1] = %.20f;\n' % model.get_gravity()[1])
            f.write('\t/// Add weights to appliedBodyForces.\n')
            f.write('\tfor (int i = 0; i < model->getBodySet().getSize(); ++i) {\n')
            f.write('\t\tmodel->getMatterSubsystem().addInStationForce(*state,\n')
            f.write('\t\tmodel->getBodySet().get(i).getMobilizedBodyIndex(),\n')
            f.write('\t\tmodel->getBodySet().get(i).getMassCenter(),\n')
            f.write('\t\tmodel->getBodySet().get(i).getMass()*gravity, appliedBodyForces);\n')
            f.write('\t}\n')    
            f.write('\t/// Add contact forces to appliedBodyForces.\n')
            
            count = 0
            for i in range(forceSet.getSize()):        
                c_force_elt = forceSet.get(i)     
                
                if c_force_elt.getConcreteClassName() == "SmoothSphereHalfSpaceForce":
                    c_force_elt_name = c_force_elt.getName()    
                    
                    f.write('\tArray<osim_double_adouble> Force_%s = %s->getRecordValues(*state);\n' % (str(count), c_force_elt_name))
                    f.write('\tSpatialVec GRF_%s;\n' % (str(count)))           
                    
                    f.write('\tGRF_%s[0] = Vec3(Force_%s[3], Force_%s[4], Force_%s[5]);\n' % (str(count), str(count), str(count), str(count)))
                    f.write('\tGRF_%s[1] = Vec3(Force_%s[0], Force_%s[1], Force_%s[2]);\n' % (str(count), str(count), str(count), str(count)))
                    
                    socket1Name = c_force_elt.getSocketNames()[1]
                    socket1 = c_force_elt.getSocket(socket1Name)
                    socket1_obj = socket1.getConnecteeAsObject()
                    socket1_objName = socket1_obj.getName()            
                    geo1 = geometrySet.get(socket1_objName)
                    geo1_frameName = geo1.getFrame().getName()
                    
                    f.write('\tint c_idx_%s = model->getBodySet().get("%s").getMobilizedBodyIndex();\n' % (str(count), geo1_frameName))            
                    f.write('\tappliedBodyForces[c_idx_%s] += GRF_%s;\n' % (str(count), str(count)))
                    count += 1
                    f.write('\n')
                    
            f.write('\t/// knownUdot.\n')
            f.write('\tVector knownUdot(nCoordinates);\n')
            f.write('\tknownUdot.setToZero();\n')
            f.write('\tfor (int i = 0; i < nCoordinates; ++i) knownUdot[i] = ua[i];\n')
            f.write('\t/// Calculate residual forces.\n')
            f.write('\tVector residualMobilityForces(nCoordinates);\n')
            f.write('\tresidualMobilityForces.setToZero();\n')
            f.write('\tmodel->getMatterSubsystem().calcResidualForceIgnoringConstraints(*state,\n')
            f.write('\t\t\tappliedMobilityForces, appliedBodyForces, knownUdot, residualMobilityForces);\n\n')
            
            if exportMarkerPositions:
                f.write('\t/// Marker positions.\n')
                for marker in response_markers:
                    if "." in marker:
                        marker_adj = marker.replace(".", "_")
                    else:
                        marker_adj = marker                       
                    f.write('\tVec3 %s_location = %s->getLocationInGround(*state);\n' % (marker_adj, marker_adj))
                f.write('\n')
                
            if generate_pp or generate_grf:
                f.write('\t/// Ground reaction forces and moments.\n')
                f.write('\tSpatialVec GRF_r;\n')
                f.write('\tSpatialVec GRF_l;\n')
                f.write('\tSpatialVec GRM_r;\n')
                f.write('\tSpatialVec GRM_l;\n')
                f.write('\tVec3 normal(0, 1, 0);\n')
                count = 0
                geo1_frameNames = []
                for i in range(forceSet.getSize()):        
                    c_force_elt = forceSet.get(i)  
                    if c_force_elt.getConcreteClassName() == "SmoothSphereHalfSpaceForce":
                        c_force_elt_name = c_force_elt.getName() 
                        socket1Name = c_force_elt.getSocketNames()[1]
                        socket1 = c_force_elt.getSocket(socket1Name)
                        socket1_obj = socket1.getConnecteeAsObject()
                        socket1_objName = socket1_obj.getName()            
                        geo1 = geometrySet.get(socket1_objName)
                        geo1_frameName = geo1.getFrame().getName() 
                        
                        if not geo1_frameName in geo1_frameNames:
                            f.write('\tSimTK::Transform TR_GB_%s = %s->getMobilizedBody().getBodyTransform(*state);\n' % (geo1_frameName, geo1_frameName))    
                            geo1_frameNames.append(geo1_frameName)
                            
                        f.write('\tVec3 %s_location_G = %s->findStationLocationInGround(*state, %s_location);\n' % (c_force_elt_name, geo1_frameName, c_force_elt_name))                
                        f.write('\tVec3 %s_locationCP_G = %s_location_G - %s_radius * normal;\n' % (c_force_elt_name, c_force_elt_name, c_force_elt_name))
                        f.write('\tVec3 locationCP_G_adj_%i = %s_locationCP_G - 0.5*%s_locationCP_G[1] * normal;\n' % (count, c_force_elt_name, c_force_elt_name))
                        f.write('\tVec3 %s_locationCP_B = model->getGround().findStationLocationInAnotherFrame(*state, locationCP_G_adj_%i, *%s);\n' % (c_force_elt_name, count, geo1_frameName))
                        f.write('\tVec3 GRM_%i = (TR_GB_%s*%s_locationCP_B) %% GRF_%s[1];\n' % (count, geo1_frameName, c_force_elt_name, str(count)))
                        
                        if c_force_elt_name[-2:] == "_r":
                            f.write('\tGRF_r += GRF_%s;\n'  % (str(count)))
                            f.write('\tGRM_r += GRM_%i;\n'  % (count))   
                        elif c_force_elt_name[-2:] == "_l":
                            f.write('\tGRF_l += GRF_%s;\n'  % (str(count)))  
                            f.write('\tGRM_l += GRM_%i;\n'  % (count))   
                        else:
                            raise ValueError("Cannot identify contact side")
                        f.write('\n')                   
                        count += 1
                f.write('\n')
            
            f.write('\t/// Residual forces.\n')
            f.write('\t/// OpenSim and Simbody have different state orders so we need to adjust\n')
            f.write('\tauto indicesSimbodyInOS = getIndicesSimbodyInOS(*model);\n')
            f.write('\tfor (int i = 0; i < NU; ++i) res[0][i] =\n')
            f.write('\t\t\tvalue<T>(residualMobilityForces[indicesSimbodyInOS[i]]);\n')
            
            if exportMarkerPositions:
                f.write('\t/// Marker positions.\n')
                f.write('\tint nc = 3;\n')
                for count, marker in enumerate(response_markers):
                    if "." in marker:
                        marker_adj = marker.replace(".", "_")
                    else:
                        marker_adj = marker  
                    f.write('\tfor (int i = 0; i < nc; ++i) res[0][i + NU + %i * nc] = value<T>(%s_location[i]);\n' % (count, marker_adj))
                count_acc = count
            else:
                count_acc = -1
            f.write('\n')
            
            if generate_pp or generate_grf:
                if not exportMarkerPositions:
                    f.write('\tint nc = 3;\n')
                f.write('\tfor (int i = 0; i < nc; ++i) res[0][i + NU + %i * nc] = value<T>(GRF_r[1][i]);\n' % (count_acc + 1))
                f.write('\tfor (int i = 0; i < nc; ++i) res[0][i + NU + %i * nc] = value<T>(GRF_l[1][i]);\n' % (count_acc + 2))
                f.write('\tfor (int i = 0; i < nc; ++i) res[0][i + NU + %i * nc] = value<T>(GRM_r[1][i]);\n' % (count_acc + 3))
                f.write('\tfor (int i = 0; i < nc; ++i) res[0][i + NU + %i * nc] = value<T>(GRM_l[1][i]);\n' % (count_acc + 4))
                f.write('\n')
                if generate_pp:
                    count_acc += 4
                    count = 0
                    for i in range(forceSet.getSize()):
                        c_force_elt = forceSet.get(i) 
                        if c_force_elt.getConcreteClassName() == "SmoothSphereHalfSpaceForce":
                            f.write('\tfor (int i = 0; i < nc; ++i) res[0][i + NU + %i * nc] = value<T>(GRF_%i[1][i]);\n' % (count_acc + 1, count))
                            f.write('\tfor (int i = 0; i < nc; ++i) res[0][i + NU + %i * nc] = value<T>(locationCP_G_adj_%i[i]);\n' % (count_acc + 2, count))
                            f.write('\n')                       
                           	# f.write('\tfor (int i = 0; i < nc; ++i) res[0][i + NU + %i * nc] = value<T>(GRF_6_l[1][i]);\n' % (count_acc + 1))
                           	# f.write('\tfor (int i = 0; i < nc; ++i) res[0][i + NU + %i * nc] = value<T>(contactPointpos_InGround_HC_s6_l_adj[i]);\n' % (count_acc + 1))
                            count_acc += 2
                            count += 1
            
            f.write('\treturn 0;\n')
            f.write('}\n\n')
            
            f.write('int main() {\n')
            f.write('\tRecorder x[NX];\n')
            f.write('\tRecorder u[NU];\n')
            f.write('\tRecorder tau[NR];\n')
            f.write('\tfor (int i = 0; i < NX; ++i) x[i] <<= 0;\n')
            f.write('\tfor (int i = 0; i < NU; ++i) u[i] <<= 0;\n')
            f.write('\tconst Recorder* Recorder_arg[n_in] = { x,u };\n')
            f.write('\tRecorder* Recorder_res[n_out] = { tau };\n')
            f.write('\tF_generic<Recorder>(Recorder_arg, Recorder_res);\n')
            f.write('\tdouble res[NR];\n')
            f.write('\tfor (int i = 0; i < NR; ++i) Recorder_res[0][i] >>= res[i];\n')
            f.write('\tRecorder::stop_recording();\n')
            f.write('\treturn 0;\n')
            f.write('}\n')
            
        # %% Build external Function (.dll file)
        if build_externalFunction:
            from buildExternalFunction import buildExternalFunction
            buildExternalFunction(outputCPPFileName, pathOutputExternalFunctionFolder,
                                  3*nCoordinates,
                                  compiler="Visual Studio 15 2017 Win64")
            
        # %% Verification
        if verifyID:    
            # Run ID with the .osim file
            pathGenericIDFolder = os.path.join(pathOpenSimModel, "ID")
            pathGenericIDSetupFile = os.path.join(pathGenericIDFolder, "Setup_ID.xml")
            
            idTool = opensim.InverseDynamicsTool(pathGenericIDSetupFile)
            idTool.setName("ID_withOsimAndIDTool")
            idTool.setModelFileName(pathModel)
            idTool.setResultsDir(pathOutputExternalFunctionFolder)
            if 'Rajagopal' in OpenSimModel or 'Lai' in OpenSimModel:
                idTool.setCoordinatesFileName(os.path.join(pathGenericIDFolder,
                                                        "DefaultPosition_rajagopal.mot"))        
            else:
                idTool.setCoordinatesFileName(os.path.join(pathGenericIDFolder,
                                                            "DefaultPosition.mot"))
            idTool.setOutputGenForceFileName("ID_withOsimAndIDTool.sto")       
            pathSetupID = os.path.join(pathOutputExternalFunctionFolder, "SetupID.xml")
            idTool.printToXML(pathSetupID)
            
            command = 'opensim-cmd' + ' run-tool ' + pathSetupID
            os.system(command)
            
            # Extract torques from .osim + ID tool.    
            headers = []
            nCoordinatesAll = coordinateSet.getSize()
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
            ID_osim_df = storage2df(os.path.join(pathOutputExternalFunctionFolder,
                                          "ID_withOsimAndIDTool.sto"), headers)
            ID_osim = np.zeros((nCoordinates))
            for count, coordinateOrder in enumerate(coordinatesOrder):
                if (coordinateOrder == "pelvis_tx" or 
                    coordinateOrder == "pelvis_ty" or 
                    coordinateOrder == "pelvis_tz"):
                    suffix_header = "_force"
                else:
                    suffix_header = "_moment"
                ID_osim[count] = ID_osim_df.iloc[0][coordinateOrder + suffix_header]
            
            # Extract torques from external function
            import casadi as ca
            F = ca.external('F', os.path.join(pathOutputExternalFunctionFolder, 
                                              outputCPPFileName + '.dll')) 
            
            if 'Rajagopal' in OpenSimModel or 'Lai' in OpenSimModel:
                vec1 = np.zeros((nCoordinates*2, 1))
                vec1[::2, :] = 0.05   
                vec1[8, :] = -0.05
                vec2 = np.zeros((nCoordinates, 1))
            else:
                vec1 = np.zeros((len(headers)*2, 1))
                vec1[::2, :] = -1     
                vec2 = np.zeros((len(headers), 1))
            vec3 = np.concatenate((vec1,vec2))
            ID_F = (F(vec3)).full().flatten()[:nCoordinates]       
            assert(np.max(np.abs(ID_osim - ID_F)) < 1e-6), "error F vs ID tool & osim"

# %%
def generateF(dim):
    import foo
    importlib.reload(foo)
    cg = ca.CodeGenerator('foo_jac')
    arg = ca.SX.sym('arg', dim)
    y,_,_ = foo.foo(arg)
    F = ca.Function('F',[arg],[y])
    cg.add(F)
    cg.add(F.jacobian())
    cg.generate()

# %%
def buildExternalFunction(filename, CPP_DIR, nInputs, 
                          compiler="Visual Studio 15 2017 Win64"):       
    
    # %% Part 1: build expression graph (i.e., generate foo.py).
    pathMain = os.getcwd()
    pathBuildExpressionGraph = os.path.join(pathMain, 'buildExpressionGraph')
    pathBuild = os.path.join(pathMain, 'build-ExpressionGraph' + filename)
    os.makedirs(pathBuild, exist_ok=True)    
    
    OpenSimAD_DIR = os.path.join(pathMain, 'OpenSimAD-install')
    SDK_DIR = os.path.join(OpenSimAD_DIR, 'sdk')
    BIN_DIR = os.path.join(OpenSimAD_DIR, 'bin')
    
    os.chdir(pathBuild)    
    cmd1 = 'cmake "' + pathBuildExpressionGraph + '" -G "' + compiler + '" -DTARGET_NAME:STRING="' + filename + '" -DSDK_DIR:PATH="' + SDK_DIR + '" -DCPP_DIR:PATH="' + CPP_DIR + '"'
    os.system(cmd1)
    cmd2 = "cmake --build . --config RelWithDebInfo"
    os.system(cmd2)
    
    os.chdir(BIN_DIR)
    path_EXE = os.path.join(pathBuild, 'RelWithDebInfo', filename + '.exe')
    os.system(path_EXE)
    
    # %% Part 2: build external function (i.e., build .dll).
    fooName = "foo.py"
    pathBuildExternalFunction = os.path.join(pathMain, 'buildExternalFunction')
    path_external_filename_foo = os.path.join(BIN_DIR, fooName)
    path_external_functions_filename_build = os.path.join(pathMain, 'build-ExternalFunction' + filename)
    path_external_functions_filename_install = os.path.join(pathMain, 'install-ExternalFunction' + filename)
    os.makedirs(path_external_functions_filename_build, exist_ok=True) 
    os.makedirs(path_external_functions_filename_install, exist_ok=True)
    shutil.copy2(path_external_filename_foo, pathBuildExternalFunction)
    
    sys.path.append(pathBuildExternalFunction)
    os.chdir(pathBuildExternalFunction)
    
    generateF(nInputs)
    
    os.chdir(path_external_functions_filename_build)
    cmd1 = 'cmake "' + pathBuildExternalFunction + '" -G "' + compiler + '" -DTARGET_NAME:STRING="' + filename + '" -DINSTALL_DIR:PATH="' + path_external_functions_filename_install + '"'
    os.system(cmd1)
    cmd2 = "cmake --build . --config RelWithDebInfo --target install"
    os.system(cmd2)    
    os.chdir(pathMain)
    
    shutil.copy2(os.path.join(path_external_functions_filename_install, 'bin', filename + '.dll'), CPP_DIR)
    os.remove(os.path.join(pathBuildExternalFunction, "foo_jac.c"))
    os.remove(os.path.join(pathBuildExternalFunction, fooName))
    os.remove(path_external_filename_foo)
    shutil.rmtree(pathBuild)
    shutil.rmtree(path_external_functions_filename_install)
    shutil.rmtree(path_external_functions_filename_build)    