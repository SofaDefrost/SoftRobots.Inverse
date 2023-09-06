import os

path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', pluginName='SoftRobots SoftRobots.Inverse')
    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels hideBehaviorModels showCollisionModels '
                                    'hideBoundingCollisionModels hideForceFields '
                                    'showInteractionForceFields hideWireframe')

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver')

    # bunny
    bunny = rootNode.addChild('bunny')
    bunny.addObject('EulerImplicitSolver')
    bunny.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixd")
    bunny.addObject('MeshVTKLoader', name='loader', filename=path + 'Hollow_Stanford_Bunny.vtu')
    bunny.addObject('MeshTopology', src='@loader', name='container')
    bunny.addObject('MechanicalObject')
    bunny.addObject('UniformMass', totalMass=0.5)
    bunny.addObject('TetrahedronFEMForceField', poissonRatio=0.45, youngModulus=60000)
    bunny.addObject('BoxROI', name='boxROI', box=[-5, -5.5, -5, 5, -4.5, 5], drawBoxes=True)
    bunny.addObject('RestShapeSpringsForceField', points=bunny.boxROI.indices.linkpath, stiffness=1e12)
    bunny.addObject('LinearSolverConstraintCorrection')

    # bunny/effector
    effector = bunny.addChild('effector')
    effector.addObject('MeshOBJLoader', name='loader', filename=path + 'Hollow_Bunny_Body_Cavity.obj')
    effector.addObject('MeshTopology', src='@loader', name='topo')
    effector.addObject('MechanicalObject', name='cavity')
    effector.addObject('VolumeEffector', template='Vec3', triangles='@topo.triangles', desiredVolume=60)
    effector.addObject('BarycentricMapping')

    # bunny/actuator
    actuator = bunny.addChild('actuator')
    actuator.addObject('MechanicalObject', name="effectorPoint",
                       position=[[0, 1.2, 1.],
                                 [0.5, 1.2, 0.5],
                                 [0.5, 1.2, 1.],
                                 [0, 1.2, 0.5]],
                       showObject=1, showObjectScale=0.2, drawMode=1)
    actuator.addObject('ForcePointActuator', indices=[0, 1, 2, 3], direction=[0, -1, 0], minForce=0,
                       showForce=1, visuScale=0.01)
    actuator.addObject('BarycentricMapping')

    # bunny/bunnyVisu
    bunnyVisu = bunny.addChild('visu')
    bunnyVisu.addObject('TriangleSetTopologyContainer', name='container')
    bunnyVisu.addObject('TriangleSetTopologyModifier')
    bunnyVisu.addObject('Tetra2TriangleTopologicalMapping', name='Mapping',
                        input=bunny.container.linkpath, output=bunnyVisu.container.linkpath)
    bunnyVisu.addObject('OglModel', color=[0.3, 0.2, 0.2, 0.6])
    bunnyVisu.addObject('IdentityMapping')

    return rootNode
