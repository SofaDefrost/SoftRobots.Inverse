import os

path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', pluginName='SoftRobots SoftRobots.Inverse')
    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels showBehaviorModels showCollisionModels '
                                    'hideBoundingCollisionModels hideForceFields showInteractionForceFields '
                                    'hideWireframe')

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver')

    ##########################################
    # FEM Model                              #
    ##########################################
    model = rootNode.addChild('model')
    model.addObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
    model.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixMat3x3d")
    model.addObject('MeshVTKLoader', name='loader', filename=path + 'PneuNets.vtk')
    model.addObject('MeshTopology', src='@loader', name='container')
    model.addObject('MechanicalObject')
    model.addObject('UniformMass', totalMass=0.5)
    model.addObject('TetrahedronFEMForceField', poissonRatio=0.3, youngModulus=200)
    model.addObject('BoxROI', name='boxROI', box=[-5, 0, -20, 0, 30, 20], drawBoxes=True)
    model.addObject('BoxROI', name='boxROISubTopo', box=[-175, 22.5, -8, -19, 28, 8], drawBoxes=False)
    model.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12)
    model.addObject('LinearSolverConstraintCorrection')

    ##########################################
    # Effector                               #
    ##########################################
    cable = model.addChild('cable')
    cable.addObject('MechanicalObject', name='cable',
                    position=[[0, 0, 0], [-24, 0, 0], [-26, 0, 0], [-37, 0, 0], [-39, 0, 0],
                              [-50, 0, 0], [-52, 0, 0], [-63, 0, 0], [-65, 0, 0], [-76, 0, 0], [-78, 0, 0]])
    cable.addObject('CableEffector', desiredLength=100, indices=list(range(11)))
    cable.addObject('BarycentricMapping')

    ##########################################
    # Actuator                               #
    ##########################################
    cavity = model.addChild('cavity')
    cavity.addObject('MeshSTLLoader', name='loader', filename=path + 'PneuNets_Cavity.stl')
    cavity.addObject('MeshTopology', src='@loader', name='topo')
    cavity.addObject('MechanicalObject', name='cavity')
    cavity.addObject('SurfacePressureActuator', template='Vec3', triangles='@topo.triangles', minPressure=0,
                     maxVolumeGrowthVariation=500)
    cavity.addObject('BarycentricMapping')

    ##########################################
    # Visualization                          #
    ##########################################
    modelVisu = model.addChild('visu')
    modelVisu.addObject('MeshOBJLoader', filename=path + "PneuNets.stl", name="loader")
    modelVisu.addObject('OglModel', src="@loader", template='Vec3', color=[0.7, 0.7, 0.7, 0.6])
    modelVisu.addObject('BarycentricMapping')

    return rootNode
