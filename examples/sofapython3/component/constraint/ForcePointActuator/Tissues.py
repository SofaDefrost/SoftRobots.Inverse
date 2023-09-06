import os
path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='SoftRobots.Inverse')
    rootNode.addObject('VisualStyle',
                       displayFlags='hideWireframe showVisualModels showBehaviorModels hideCollisionModels '
                                    'hideBoundingCollisionModels hideForceFields showInteractionForceFields')

    rootNode.gravity = [0, -9180, 0]
    rootNode.dt = 0.02

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver', printLog=False, epsilon=1e-3, tolerance=1e-4, maxIterations=2500)
    rootNode.addObject('DefaultPipeline')
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('DefaultContactManager', response="FrictionContactConstraint", responseParams="mu=0")
    rootNode.addObject('LocalMinDistance', name="Proximity", alarmDistance=5, contactDistance=3)

    ##########################################
    # Effector goal for interactive control  #
    ##########################################
    goal = rootNode.addChild('goal')
    goal.addObject('VisualStyle', displayFlags='showCollisionModels')
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver', iterations=100)
    goal.addObject('MechanicalObject', name='goalMO', showObject=True, showObjectScale=10, drawMode=1,
                   showColor=[255, 255, 255, 255], position=[40, 30, 30])
    goal.addObject('SphereCollisionModel', radius=10, group=[0, 1, 2])
    goal.addObject('UncoupledConstraintCorrection')

    ##########################################
    # FEM Model                              #
    ##########################################
    model = rootNode.addChild('model')
    model.addObject('EulerImplicitSolver')
    model.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixMat3x3d")
    model.addObject('MeshVTKLoader', name='loader', filename=path + 'sphere.vtk')
    model.addObject('MeshTopology', src='@loader', name='container')
    model.addObject('MechanicalObject')
    model.addObject('UniformMass', totalMass=0.03)
    model.addObject('TetrahedronFEMForceField', poissonRatio=0.3, youngModulus=180)
    model.addObject('BoxROI', name='boxROI', box=[0, 50, -20, 10, 70, 20], drawBoxes=False)
    model.addObject('FixedConstraint', indices='@boxROI.indices')
    model.addObject('LinearSolverConstraintCorrection')

    ##########################################
    # FEM Model                              #
    ##########################################
    skin = rootNode.addChild('skin')
    skin.addObject('EulerImplicitSolver')
    skin.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixMat3x3d")
    skin.addObject('MeshVTKLoader', name='loader', filename=path + 'skin.vtk')
    skin.addObject('MeshTopology', src='@loader', name='container')
    skin.addObject('MechanicalObject')
    skin.addObject('UniformMass', totalMass=0.03)
    skin.addObject('TetrahedronFEMForceField', poissonRatio=0.3, youngModulus=180)
    skin.addObject('BoxROI', name='boxROI1', box=[-100, -5, -30, -90, 5, 30], drawBoxes=False)
    skin.addObject('BoxROI', name='boxROI2', box=[100, -5, -30, 90, 5, 30], drawBoxes=False)
    skin.addObject('FixedConstraint', name='fix_1', indices='@boxROI1.indices')
    skin.addObject('FixedConstraint', name='fix_2', indices='@boxROI2.indices')
    skin.addObject('LinearSolverConstraintCorrection')

    ##########################################
    # Effector                               #
    ##########################################
    effector = model.addChild('effector')
    effector.addObject('MechanicalObject', name="effectorPoint", position=[40, 30, 10])
    effector.addObject('PositionEffector', template='Vec3',
                       indices=[0],
                       effectorGoal="@../../goal/goalMO.position",
                       useDirections=[1, 1, 0])
    effector.addObject('BarycentricMapping')

    ##########################################
    # Actuator                               #
    ##########################################
    pointForce = skin.addChild('pointForce')
    pointForce.addObject('VisualStyle', displayFlags="showInteractionForceFields")
    pointForce.addObject('MechanicalObject', name="MO", position=[[72, 68, -2], [70, 70, 2], [70, 70, 0]])
    pointForce.addObject('ForcePointActuator', showForce=True, visuScale=10,
                         direction=[-1, -1, 0], indices=[0, 1, 2], maxForce=90, minForce=0, maxForceVariation=5)
    pointForce.addObject('BarycentricMapping')

    ##########################################
    # Visualization                          #
    ##########################################
    modelVisu = model.addChild('visu')
    modelVisu.addObject('MeshSTLLoader', name="loader", filename=path + "sphereVisu.stl")
    modelVisu.addObject('OglModel', src="@loader", template='Vec3', color=[0.6, 0.1, 0.1, 1.])
    modelVisu.addObject('BarycentricMapping')

    ##########################################
    # Visualization                          #
    ##########################################
    skinVisu = skin.addChild('skinVisu')
    skinVisu.addObject('MeshSTLLoader', name="loader", filename=path + "skinVisu.stl")
    skinVisu.addObject('OglModel', src="@loader", template='Vec3', color=[0.7, 0.5, 0.5, 1.])
    skinVisu.addObject('BarycentricMapping')

    ##########################################
    # Contact                                #
    ##########################################
    modelContact = model.addChild('contact')
    modelContact.addObject('MeshSTLLoader', name='loader', filename=path + 'sphere.stl')
    modelContact.addObject('MeshTopology', src='@loader', name='topo')
    modelContact.addObject('MechanicalObject')
    modelContact.addObject('TriangleCollisionModel', group=1)
    modelContact.addObject('LineCollisionModel', group=1)
    modelContact.addObject('PointCollisionModel', group=1)
    modelContact.addObject('BarycentricMapping')

    ##########################################
    # Contact                                #
    ##########################################
    skinContact = skin.addChild('skinContact')
    skinContact.addObject('MeshSTLLoader', name='loader', filename=path + 'skin.stl')
    skinContact.addObject('MeshTopology', src='@loader', name='topo')
    skinContact.addObject('MechanicalObject')
    skinContact.addObject('TriangleCollisionModel', group=2)
    skinContact.addObject('LineCollisionModel', group=2)
    skinContact.addObject('PointCollisionModel', group=2)
    skinContact.addObject('BarycentricMapping')

    return rootNode
