import os

path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'
pathDir = os.path.dirname(os.path.abspath(__file__)) + '/'

objectModel = "cylinder"

visuMeshFile = path + objectModel + "_visu.stl"
mecaMeshFile = path + objectModel + ".vtk"
forceMeshFile = path + objectModel + "_forces.stl"
translation = [0., 0., 0.]


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name="ExternalPlugins", pluginName=['SoftRobots', 'SoftRobots.Inverse'])
    rootNode.addObject('VisualStyle', displayFlags='hideWireframe hideVisualModels hideBehaviorModels '
                                                   'showCollisionModels hideBoundingCollisionModels '
                                                   'showForceFields showInteractionForceFields')

    rootNode.findData('gravity').value = [0, 0, -9180]
    rootNode.findData('dt').value = 0.02

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver', epsilon=1e-3, tolerance=1e-2, maxIterations=500)

    ##########################################
    # Effector goal for interactive control  #
    ##########################################
    goal = rootNode.addChild('goal')
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver', iterations=100, tolerance=0.0001, threshold=0.0001)
    goal.addObject('MechanicalObject', name='dofs', showObject=True, showObjectScale=2, drawMode=1,
                   position=[[50, 5, 130], [50, 5, 70], [50, 5, 15]])
    trajectoryFile = pathDir + "trajectoryCylinder.tkt"
    goal.addObject('AnimationEditor', filename=trajectoryFile, loop=True, listening=True)
    goal.addObject('SphereCollisionModel', radius=2, group=1)
    goal.addObject('UncoupledConstraintCorrection')

    ##########################################
    # OBJECT                                 #
    ##########################################

    ##########################################
    # FEM Model                              #
    ##########################################
    model = rootNode.addChild('model')
    model.addObject('EulerImplicitSolver')
    model.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixMat3x3d")
    model.addObject('MeshVTKLoader', name='loader', filename=mecaMeshFile, scale3d=[1.5, 1.5, 1.5],
                    translation=translation)
    model.addObject('MeshTopology', src='@loader', name='container')
    model.addObject('MechanicalObject')
    model.addObject('UniformMass', totalMass=0.1)
    model.addObject('TetrahedronFEMForceField', poissonRatio=0.3, youngModulus=150)
    model.addObject('BoxROI', name='boxROI', box=[-10, -15, 50, 10, 15, 90], drawBoxes=True)
    model.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e2)
    model.addObject('LinearSolverConstraintCorrection')

    ##########################################
    # Effector                               #
    ##########################################
    if objectModel == "cylinder":
        positionEffector = [[30, 5, 130], [30, 5, 70], [30, 5, 15]]
    elif objectModel == "sphere":
        positionEffector = [[30, 5, 100], [30, 5, 70], [30, 5, 40]]

    effector = model.addChild('effector')
    effector.addObject('MechanicalObject', name="effectorPoint",
                       position=positionEffector)
    effector.addObject('PositionEffector', template='Vec3',
                       indices=[0, 1, 2],
                       effectorGoal="@../../goal/dofs.position",
                       useDirections=[0, 1, 1])
    effector.addObject('BarycentricMapping')

    ##########################################
    # Visualization                          #
    ##########################################
    modelVisu = model.addChild('visu')
    modelVisu.addObject('MeshSTLLoader', filename=visuMeshFile, name="loader")
    modelVisu.addObject('OglModel', src="@loader", scale3d=[1.5, 1.5, 1.5], translation=translation)
    modelVisu.addObject('BarycentricMapping')

    ##########################################
    # Actuation                              #
    ##########################################
    centers = [[0, 30, 120], [0, 30, 20], [0, -30, 70]]
    modelActuation = model.addChild('actuation')
    modelActuation.addObject('MeshSTLLoader', name='loader', filename=forceMeshFile, scale3d=[1.5, 1.5, 1.5],
                             translation=translation)
    modelActuation.addObject('MeshTopology', src='@loader', name='topo')
    modelActuation.addObject('MechanicalObject')
    modelActuation.addObject('ForceSurfaceActuator', maxForce=0, minForce=-80, centers=centers, radii=[10, 10, 15],
                             drawForces=True, drawSpheres=False, drawSurfaces=True, updateNormals=True)
    modelActuation.addObject('BarycentricMapping')

    return rootNode
