def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='SoftRobots.Inverse')
    rootNode.addObject('RequiredPlugin', name='BeamAdapter')
    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels showBehaviorModels showCollisionModels '
                                    'hideBoundingCollisionModels showForceFields '
                                    'hideInteractionForceFields hideWireframe')

    rootNode.findData('dt').value = 0.01
    rootNode.findData('gravity').value = [0, -9810, 0]
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver', epsilon=1, tolerance=1e-8, maxIterations=2500)

    #########################################
    # goal
    ##########################################
    goal = rootNode.addChild('goal')
    goal.addObject('VisualStyle', displayFlags="showCollisionModels")
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
    goal.addObject('MechanicalObject', name='goalMO', template="Vec3", showObject=True, drawMode=1,
                   showObjectScale=3, position=[10, 0, 0])
    goal.addObject('UncoupledConstraintCorrection')

    #########################################
    # solver
    ##########################################
    solverNode = rootNode.addChild('Solver')
    solverNode.addObject('EulerImplicitSolver', firstOrder=False, rayleighStiffness=40.0, rayleighMass=40.0)
    solverNode.addObject('SparseLDLSolver')
    solverNode.addObject('GenericConstraintCorrection')

    ##########################################
    # Beam Model                             #
    ##########################################
    posNode = [[0, 0, 0], [10, 0, 0], [20, 0, 0]]
    edgeList = [[0, 1], [1, 2]]
    FramesNode = solverNode.addChild("framesNode")
    FramesNode.addObject('MeshTopology', position=posNode, edges=edgeList)
    FramesNode.addObject('MechanicalObject', template="Rigid3",
                         showObject=True,
                         showObjectScale=0.001)
    FramesNode.addObject('BarycentricCenterEffector', template="Rigid3",
                         effectorGoal=goal.goalMO.position.linkpath,
                         axis=[1, 1, 1])

    topo = FramesNode.addChild("topo")
    topo.addObject('MeshTopology', position=posNode, edges=edgeList)
    topo.addObject('BeamInterpolation', name="BeamInterpolation", defaultYoungModulus=1e6,
                   dofsAndBeamsAligned=True, straight=True)
    topo.addObject('AdaptiveBeamForceFieldAndMass', computeMass=True, massDensity=0.0001)

    ##########################################
    # Cable Model                            #
    ##########################################
    cable = FramesNode.addChild('Cables')
    cable.addObject('VisualStyle', displayFlags="showInteractionForceFields")
    cable.addObject('MechanicalObject', template='Vec3', position=[[0., 0., 0.], [0., 0., 0.]])
    for i in range(4):
        cable.addObject('CableActuator', name="cable" + str(i), indices=i % 2,
                        pullPoint=[[-20, 20, 20],
                                   [40, 20, 20],
                                   [-20, 20, -20],
                                   [40, 20, -20]][i],
                        minForce=0,
                        maxPositiveDisp=10,
                        maxDispVariation=0.1)
    cable.addObject('RigidMapping', rigidIndexPerPoint=[0, 2])

    return rootNode
