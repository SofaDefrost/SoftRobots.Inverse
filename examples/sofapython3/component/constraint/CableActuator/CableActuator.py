import os

path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', pluginName='SoftRobots SoftRobots.Inverse')
    rootNode.addObject('VisualStyle', displayFlags='showVisualModels hideBehaviorModels showCollisionModels '
                                                   'hideBoundingCollisionModels hideForceFields '
                                                   'showInteractionForceFields hideWireframe')
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver')

    # goal
    goal = rootNode.addChild('goal')
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
    goal.addObject('MechanicalObject', name='goalMO', position=[-5.72055, 1.13543, 3.10608])
    goal.addObject('SphereCollisionModel', radius=0.25, group=3)
    goal.addObject('UncoupledConstraintCorrection')

    # bunny
    bunny = rootNode.addChild('bunny')
    bunny.addObject('EulerImplicitSolver')
    bunny.addObject('SparseLDLSolver')
    bunny.addObject('MeshVTKLoader', name='loader', filename=path + 'Hollow_Stanford_Bunny.vtu')
    bunny.addObject('MeshTopology', src='@loader', name='container')
    bunny.addObject('MechanicalObject')
    bunny.addObject('UniformMass', totalMass=0.5)
    bunny.addObject('TetrahedronFEMForceField', poissonRatio=0.3, youngModulus=18000)
    bunny.addObject('BoxROI', name='boxROI', box=[-5, -6, -5,  5, -4.5, 5], drawBoxes=True)
    bunny.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e12)
    bunny.addObject('LinearSolverConstraintCorrection')

    # bunny/constraints
    constraints = bunny.addChild('constraints')
    constraints.addObject('MechanicalObject', name="points", position=[-4.72055, 1.13543, 3.10608])
    constraints.addObject('PositionEffector', indices=[0], effectorGoal="@../../goal/goalMO.position",
                          useDirections=[0, 1, 0])
    constraints.addObject('CableActuator', template='Vec3', indices=[0], pullPoint=[-4.72055, -5, 3.10608],
                          maxPositiveDisp=4, minForce=0)
    constraints.addObject('BarycentricMapping')

    # bunny/bunnyVisu
    bunnyVisu = bunny.addChild('visu')
    bunnyVisu.addObject('TriangleSetTopologyContainer', name='container')
    bunnyVisu.addObject('TriangleSetTopologyModifier')
    bunnyVisu.addObject('Tetra2TriangleTopologicalMapping', name='Mapping')
    bunnyVisu.addObject('OglModel', color=[0.7, 0.4, 0.4, 1])
    bunnyVisu.addObject('IdentityMapping')

    return rootNode
