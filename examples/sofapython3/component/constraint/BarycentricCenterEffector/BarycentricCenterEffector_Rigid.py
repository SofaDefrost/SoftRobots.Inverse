from math import sin, cos, pi

def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='SoftRobots.Inverse')
    rootNode.addObject('RequiredPlugin', name='BeamAdapter')
    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels showBehaviorModels showCollisionModels '
                                    'hideBoundingCollisionModels showForceFields '
                                    'showInteractionForceFields hideWireframe')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.AnimationLoop')  # Needed to use components [FreeMotionAnimationLoop]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Correction')  # Needed to use components [GenericConstraintCorrection,UncoupledConstraintCorrection]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Direct')  # Needed to use components [SparseLDLSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Iterative')  # Needed to use components [CGLinearSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mapping.NonLinear')  # Needed to use components [RigidMapping]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward')  # Needed to use components [EulerImplicitSolver]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.StateContainer')  # Needed to use components [MechanicalObject]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant')  # Needed to use components [MeshTopology]
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Visual')  # Needed to use components [VisualStyle]

    rootNode.findData('dt').value = 0.01
    rootNode.findData('gravity').value = [0, -9810, 0]
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver', epsilon=1e-3, tolerance=1e-8, maxIterations=2500)

    ##########################################
    # goal
    ##########################################
    goal = rootNode.addChild('goal')
    goal.addObject('VisualStyle', displayFlags="showCollisionModels")
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
    goal.addObject('MechanicalObject', name='goalMO', template="Rigid3", showObject=True,
                   showObjectScale=3, position=[[10, 0, 0, 0, 0, -sin(pi/16), cos(pi/16)]])
    goal.addObject('UncoupledConstraintCorrection', defaultCompliance=[1e-7, 0, 0, 0, 0, 0, 0])

    ##########################################
    # solver
    ##########################################
    solverNode = rootNode.addChild('Solver')
    solverNode.addObject('EulerImplicitSolver', firstOrder=False, rayleighStiffness=1)
    solverNode.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixd")
    solverNode.addObject('GenericConstraintCorrection')

    ##########################################
    # Beam Model
    ##########################################
    positions = [[i*10, 0, 0] for i in range(3)]
    edges = [[i, i+1] for i in range(2)]
    FramesNode = solverNode.addChild("framesNode")
    FramesNode.addObject('MeshTopology', position=positions, edges=edges)
    FramesNode.addObject('MechanicalObject', template="Rigid3")
    FramesNode.addObject('BarycentricCenterEffector', name="BCEPosition", template="Rigid3", useDirections=[0, 1, 0, 0, 0, 0],
                         effectorGoal=goal.getMechanicalState().position.linkpath, drawBarycenter=True)
    FramesNode.addObject('BarycentricCenterEffector', name="BCEOrientation", template="Rigid3", useDirections=[0, 0, 0, 0, 0, 1], weight=100,
                         effectorGoal=goal.getMechanicalState().position.linkpath)

    topo = FramesNode.addChild("topo")
    topo.addObject('MeshTopology', position=positions, edges=edges)
    topo.addObject('BeamInterpolation', name="BeamInterpolation", defaultYoungModulus=1e7,
                   dofsAndBeamsAligned=True, straight=True, radius=5)
    topo.addObject('AdaptiveBeamForceFieldAndMass', computeMass=True, massDensity=0.0001)

    ##########################################
    # Cable Model
    ##########################################
    cable = FramesNode.addChild('Cables')
    cable.addObject('VisualStyle', displayFlags="showInteractionForceFields")
    cable.addObject('MechanicalObject', template='Vec3', position=[[0., 0., 0.], [0., 0., 0.]])
    for i in range(2):
        cable.addObject('CableActuator', name="cable" + str(i), indices=i,
                        pullPoint=[[0, 20, 0], [20, 20, 0]][i], maxDispVariation=0.1)
    cable.addObject('RigidMapping', rigidIndexPerPoint=[0, 2])

    return rootNode
