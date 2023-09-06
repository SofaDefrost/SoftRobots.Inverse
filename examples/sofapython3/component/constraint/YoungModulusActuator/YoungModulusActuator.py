import os
path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', pluginName=['SoftRobots', 'SoftRobots.Inverse'])
    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels hideBehaviorModels showCollisionModels '
                                    'hideBoundingCollisionModels hideForceFields '
                                    'hideInteractionForceFields hideWireframe')

    rootNode.findData('gravity').value = [0, -981.0, 0]
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver')
    rootNode.addObject('VisualGrid', size=10, nbSubdiv=10)

    # goal
    goal = rootNode.addChild('goal')
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
    goal.addObject('MechanicalObject', name='goalMO', position=[[-2.87, 1.6, -3.46], [0.10, 2.19, -1.25]])
    goal.addObject('SphereCollisionModel', radius=0.25, group=3)
    goal.addObject('UncoupledConstraintCorrection')

    # bunny
    bunny = rootNode.addChild('bunny')
    bunny.addObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
    bunny.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixMat3x3d')
    bunny.addObject('MeshVTKLoader', name='loader', filename=path + 'bunny.vtu', translation=[0.1, 0, 1])
    bunny.addObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
    bunny.addObject('TetrahedronSetTopologyModifier')
    bunny.addObject('MechanicalObject')
    bunny.addObject('UniformMass', totalMass=2.)
    bunny.addObject('TetrahedronFEMForceField', poissonRatio=0.45, youngModulus=2000)
    bunny.addObject('BoxROI', name='boxROIFixed', box=[[-4.5, 2, -5, -2.5, 6, -1],
                                                       [-2, 2, -2, 1, 6, 1]], drawBoxes=True)
    bunny.addObject('ComplementaryROI', name="outROI", template="Vec3", position="@loader.position", nbSet=1,
                    setIndices1="@boxROIFixed.indices")
    bunny.addObject('RestShapeSpringsForceField', points='@outROI.indices', stiffness=1e12)
    bunny.addObject('BoxROI', name='boxROI1', box=[-4.5, 2, -5, -2.5, 6, -1], drawBoxes=True)
    bunny.addObject('BoxROI', name='boxROI2', box=[-2, 2, -2, 1, 6, 1], drawBoxes=True)
    bunny.addObject('LinearSolverConstraintCorrection', linearSolver=bunny.SparseLDLSolver.getLinkPath())

    # subtopologies
    modelSubTopo1 = bunny.addChild('modelSubTopo1')
    modelSubTopo1.addObject('TetrahedronSetTopologyContainer', position='@../loader.position',
                            tetrahedra="@../boxROI1.tetrahedraInROI", name='container')
    modelSubTopo1.addObject('TetrahedronFEMForceField', template='Vec3', poissonRatio=0.45, youngModulus=20000)
    modelSubTopo1.addObject('YoungModulusActuator', template='Vec3', name='actuator', maxYoungVariationRatio=0.02,
                            minYoung=10)

    modelSubTopo2 = bunny.addChild('modelSubTopo2')
    modelSubTopo2.addObject('TetrahedronSetTopologyContainer', position='@../loader.position',
                            tetrahedra="@../boxROI2.tetrahedraInROI", name='container')
    modelSubTopo2.addObject('TetrahedronFEMForceField', template='Vec3', poissonRatio=0.45, youngModulus=20000)
    modelSubTopo2.addObject('YoungModulusActuator', template='Vec3', name='actuator', maxYoungVariationRatio=0.02,
                            minYoung=10)

    # bunny/effector
    effector = bunny.addChild('effector')
    effector.addObject('MechanicalObject', name="effectorPoint", position=[[-3., 4., -3.5], [0., 4., -1.]])
    effector.addObject('PositionEffector', template='Vec3', indices=[0, 1],
                       effectorGoal=goal.goalMO.position.getLinkPath())
    effector.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

    # bunny/bunnyVisu
    bunnyVisu = bunny.addChild('visu')
    bunnyVisu.addObject('MeshOBJLoader', name="loader", filename=path + "bunnyVisu.obj")
    bunnyVisu.addObject('OglModel', src="@loader")
    bunnyVisu.addObject('BarycentricMapping')

    return rootNode
