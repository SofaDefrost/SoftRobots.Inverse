
def createScene(rootNode):

    settings = rootNode.addChild("Settings")
    settings.addObject('RequiredPlugin', name='SoftRobots')
    settings.addObject('RequiredPlugin', name='SoftRobots.Inverse')
    settings.addObject('RequiredPlugin', name='Sofa.Component.AnimationLoop')  # Needed to use components [FreeMotionAnimationLoop]
    settings.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Algorithm')  # Needed to use components [BVHNarrowPhase,BruteForceBroadPhase,CollisionPipeline]
    settings.addObject('RequiredPlugin', name='Sofa.Component.Collision.Detection.Intersection')  # Needed to use components [LocalMinDistance]
    settings.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry')  # Needed to use components [SphereCollisionModel]
    settings.addObject('RequiredPlugin', name='Sofa.Component.Collision.Response.Contact')  # Needed to use components [CollisionResponse]
    settings.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Correction')  # Needed to use components [UncoupledConstraintCorrection]
    settings.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh')  # Needed to use components [MeshVTKLoader]  
    settings.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Direct')  # Needed to use components [SparseLDLSolver]
    settings.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Iterative')  # Needed to use components [CGLinearSolver,ShewchukPCGLinearSolver]
    settings.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward')  # Needed to use components [EulerImplicitSolver]
    settings.addObject('RequiredPlugin', name='Sofa.Component.StateContainer')  # Needed to use components [MechanicalObject]
    settings.addObject('RequiredPlugin', name='Sofa.Component.Visual')  # Needed to use components [VisualStyle]  
    settings.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Projective')  # Needed to use components [FixedProjectiveConstraint]
    settings.addObject('RequiredPlugin', name='Sofa.Component.Engine.Select')  # Needed to use components [BoxROI]  
    settings.addObject('RequiredPlugin', name='Sofa.Component.Mapping.Linear')  # Needed to use components [BarycentricMapping]
    settings.addObject('RequiredPlugin', name='Sofa.Component.Mass')  # Needed to use components [UniformMass]  
    settings.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.FEM.Elastic')  # Needed to use components [TetrahedronFEMForceField]
    settings.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Dynamic')  # Needed to use components [TetrahedronSetGeometryAlgorithms,TetrahedronSetTopologyContainer]

    rootNode.findData('dt').value=1
    rootNode.findData('gravity').value=[0, 0, -9810]
    rootNode.addObject('VisualStyle', displayFlags='showCollision showVisualModels showForceFields showInteractionForceFields')
    rootNode.addObject('DefaultVisualManagerLoop')

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver')
    rootNode.addObject('CollisionPipeline')
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    rootNode.addObject('CollisionResponse', response='FrictionContactConstraint')
    rootNode.addObject('LocalMinDistance', alarmDistance=3, contactDistance=0.5)

    #goal
    goal = rootNode.addChild('goal')
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver', iterations=100,threshold=1e-5, tolerance=1e-5)
    goal.addObject('MechanicalObject', name='goalMO', position=[[0, 0, 125]])
    goal.addObject('SphereCollisionModel', radius=5, group=1)
    goal.addObject('UncoupledConstraintCorrection')

    #robot
    robot = rootNode.addChild('robot')
    robot.addObject('EulerImplicitSolver')
    robot.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixd")
    robot.addObject('GenericConstraintCorrection')
    robot.addObject('MeshVTKLoader', name="loader", filename="mesh/diamond.vtk")
    robot.addObject('MeshTopology', src="@loader")
    robot.addObject('MechanicalObject', showIndicesScale=4e-5, rx=90, dz=35)
    robot.addObject('UniformMass', totalMass=0.5)
    robot.addObject('TetrahedronFEMForceField', youngModulus=180, poissonRatio=0.45)
    robot.addObject('BoxROI', box=[-15, -15, -40, 15, 15, 10], drawBoxes=True)
    robot.addObject('FixedProjectiveConstraint', indices="@BoxROI.indices")

    #robot/controlledPoints
    controlledPoints = robot.addChild('controlledPoints')
    controlledPoints.addObject('MechanicalObject', name="actuatedPoints", 
                               position=[[0, 0, 125], [0, 97, 45],   
                                         [-97, 0, 45], [0, -97, 45],  
                                         [97, 0, 45],  [0, 0, 115]])

    controlledPoints.addObject('PositionEffector', indices=[0], effectorGoal="@../../goal/goalMO.position")

    for i in range(4):
        controlledPoints.addObject('CableActuator', name="cable"+str(i), indices=i+1,
                                   pullPoint=[[0, 10, 30], [-10, 0, 30], [0, -10, 30], [10, 0, 30]][i],
                                   maxPositiveDisp=20, minForce=0)

    controlledPoints.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

    return rootNode
