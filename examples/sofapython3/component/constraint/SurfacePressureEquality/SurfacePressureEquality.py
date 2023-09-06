from PythonScripts.EqController import Controller
import ConstantsAccordeon as Const

import os
MeshesPath = os.path.dirname(os.path.abspath(__file__)) + '/GeneratedMeshes/'


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', pluginName='SofaPython3 SoftRobots SoftRobots.Inverse')
    rootNode.addObject('VisualStyle',
                       displayFlags='hideWireframe showBehaviorModels hideCollisionModels '
                                    'hideBoundingCollisionModels showForceFields '
                                    'showInteractionForceFields')

    rootNode.findData('gravity').value = [0, 0, 0]
    rootNode.findData('dt').value = 0.02
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver', printLog=0, epsilon=1e-1, maxIterations=1000, tolerance=1e-5)

    rootNode.addObject('BackgroundSetting', color=[1, 1, 1, 1])
    rootNode.addObject('LightManager')
    rootNode.addObject('PositionalLight', name="light1", color=[0.8, 0.8, 0.8, 1], position=[0, 60, -50])
    rootNode.addObject('PositionalLight', name="light2", color=[0.8, 0.8, 0.8, 1], position=[0, -60, 50])

    VolumetricMeshPath = MeshesPath + 'Accordeon_Volumetric.vtk'
    SurfaceMeshPath = MeshesPath + 'Accordeon_Surface.stl'
    CavitySurfaceMeshPath = MeshesPath + 'Accordeon_Cavity.stl'

    ##########################################
    # Mechanical Model                       #
    ##########################################
    model = rootNode.addChild('model')
    model.addObject('EulerImplicitSolver')
    model.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixd")
    model.addObject('MeshVTKLoader', name='loader', filename=VolumetricMeshPath, scale3d=[1, 1, 1])
    model.addObject('MeshTopology', src='@loader', name='container')
    model.addObject('MechanicalObject')
    model.addObject('UniformMass', totalMass=0.1)
    model.addObject('TetrahedronFEMForceField',
                    poissonRatio=Const.PoissonRation, youngModulus=Const.YoungsModulus)
    model.addObject('BoxROI', box=Const.FixedBoxCoords, drawBoxes=True)
    model.addObject('RestShapeSpringsForceField', points=model.BoxROI.indices.linkpath, stiffness=1e12)
    model.addObject('LinearSolverConstraintCorrection')

    ##########################################
    # Pressure/Volume Equality               #
    ##########################################
    accordeonCavity = model.addChild('AccordeonCavity')
    accordeonCavity.addObject('MeshSTLLoader', name='loader', filename=CavitySurfaceMeshPath)
    accordeonCavity.addObject('MeshTopology', name='topology', src='@loader')
    accordeonCavity.addObject('MechanicalObject', src="@topology")
    # Here you can set the desired volume to reach
    accordeonCavity.addObject('SurfacePressureEquality', template='Vec3', triangles='@topology.triangles',
                              eqVolumeGrowth=500)
    accordeonCavity.addObject('BarycentricMapping')

    ##########################################
    # Visualization                          #
    ##########################################
    modelVisu = model.addChild('visu')
    modelVisu.addObject('MeshSTLLoader', filename=SurfaceMeshPath, name="loader")
    modelVisu.addObject('OglModel', src="@loader", scale3d=[1, 1, 1],
                        material="Default Diffuse 1 0.8 0.8 0.8 0.95 Ambient 0 0.2 0 0 1 "
                                 "Specular 0 1 0 0 1 Emissive 0 1 0 0 1 Shininess 0 45")
    modelVisu.addObject('BarycentricMapping')

    rootNode.addObject(Controller(name="EqualityController", RootNode=rootNode))

    return rootNode
