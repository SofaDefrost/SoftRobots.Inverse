import os

path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', pluginName='SoftRobots SoftRobots.Inverse')
    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels hideBehaviorModels showCollisionModels'
                                    ' hideBoundingCollisionModels hideForceFields '
                                    'showInteractionForceFields hideWireframe')

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver')

    # goal
    goal = rootNode.addChild('goal')
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-05, threshold=1e-05)
    goal.addObject('MechanicalObject', name='goalMO', position=[-5.72055, 1.13543, 3.10608])
    goal.addObject('SphereCollisionModel', radius=0.25, group=3)
    goal.addObject('UncoupledConstraintCorrection')

    # bunny
    bunny = rootNode.addChild('bunny')
    bunny.addObject('EulerImplicitSolver')
    bunny.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixMat3x3d")
    bunny.addObject('MeshVTKLoader', name='loader', filename=path + 'Hollow_Stanford_Bunny.vtu')
    bunny.addObject('MeshTopology', src=bunny.loader.linkpath, name='container')
    bunny.addObject('MechanicalObject')
    bunny.addObject('UniformMass', totalMass=0.5)
    bunny.addObject('TetrahedronFEMForceField', template='Vec3', poissonRatio=0.3, youngModulus=18000)
    bunny.addObject('BoxROI', name='boxROI', box=[-5, -6, -5, 5, -4.5, 5], drawBoxes=True)
    bunny.addObject('RestShapeSpringsForceField', points=bunny.boxROI.indices.linkpath, stiffness=1e12)
    bunny.addObject('LinearSolverConstraintCorrection')

    # bunny/effector
    effector = bunny.addChild('effector')
    effector.addObject('MechanicalObject', position=[-4.72055, 1.13543, 3.10608])
    effector.addObject('PositionEffector', indices=0, effectorGoal=goal.goalMO.position.linkpath)
    effector.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

    # bunny/cavity
    cavity = bunny.addChild('cavity')
    cavity.addObject('MeshOBJLoader', name='loader', filename=path + 'Hollow_Bunny_Body_Cavity.obj')
    cavity.addObject('MeshTopology', src=cavity.loader.linkpath, name='topo')
    cavity.addObject('MechanicalObject')
    cavity.addObject('SurfacePressureActuator', template='Vec3',
                     triangles=cavity.topo.triangles.linkpath, maxPressure=30,
                     minPressure=0, drawPressure=True, drawScale=0.02)
    cavity.addObject('BarycentricMapping')

    # bunny/cable
    cable = bunny.addChild('cable')
    cable.addObject('MechanicalObject', name="points", position=[-4.72055, 1.13543, 3.10608])
    cable.addObject('CableActuator', template='Vec3', indices=[0], pullPoint=[-4.72055, -5, 3.10608],
                          maxPositiveDisp=4, minForce=0)
    cable.addObject('BarycentricMapping')

    # bunny/bunnyVisu
    bunnyVisu = bunny.addChild('visu')
    bunnyVisu.addObject('TriangleSetTopologyContainer', name='container')
    bunnyVisu.addObject('TriangleSetTopologyModifier')
    bunnyVisu.addObject('Tetra2TriangleTopologicalMapping',
                        input=bunny.container.linkpath, output=bunnyVisu.container.linkpath)
    bunnyVisu.addObject('OglModel', color=[0.3, 0.2, 0.2, 0.6])
    bunnyVisu.addObject('IdentityMapping')

    return rootNode
