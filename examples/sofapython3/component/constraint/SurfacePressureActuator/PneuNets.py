import os

path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', pluginName=['SoftRobots', 'SoftRobots.Inverse'])
    rootNode.addObject('VisualStyle',
                       displayFlags='showVisualModels showBehaviorModels showCollisionModels '
                                    'hideBoundingCollisionModels hideForceFields '
                                    'showInteractionForceFields hideWireframe')

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver')

    ##########################################
    # Goal                                   #
    ##########################################
    goal = rootNode.addChild('goal')
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
    goal.addObject('MechanicalObject', name='goalMO', position=[[-230, 15, 0]])
    goal.addObject('SphereCollisionModel', radius=5, group=3)
    goal.addObject('UncoupledConstraintCorrection')

    ##########################################
    # FEM Model                              #
    ##########################################
    model = rootNode.addChild('model')
    model.addObject('EulerImplicitSolver', rayleighStiffness=0.2, rayleighMass=0.2)
    model.addObject('SparseLDLSolver')

    model.addObject('MeshVTKLoader', name='loader', filename=path + 'PneuNets.vtk')
    model.addObject('MeshTopology', src='@loader', name='container')

    model.addObject('MechanicalObject')
    model.addObject('UniformMass', totalMass=0.5)
    model.addObject('TetrahedronFEMForceField', template='Vec3', poissonRatio=0.3,
                    youngModulus=100)

    model.addObject('BoxROI', name='boxROI', box=[-10, 0, -20, 0, 30, 20], drawBoxes=True,
                    position=model.MechanicalObject.position.linkpath,
                    tetrahedra=model.container.tetrahedra.linkpath)
    model.addObject('BoxROI', name='boxROISubTopo', box=[-175, 22.5, -8, -19, 28, 8], drawBoxes=False,
                    position="@tetras.rest_position", tetrahedra=model.container.tetrahedra.linkpath)
    model.addObject('RestShapeSpringsForceField', points=model.boxROI.indices.linkpath, stiffness=1e12)
    model.addObject('LinearSolverConstraintCorrection')

    ##########################################
    # Sub topology                           #
    ##########################################
    modelSubTopo = model.addChild('modelSubTopo')
    modelSubTopo.addObject('TetrahedronSetTopologyContainer', position='@loader.position',
                           tetrahedra=model.boxROISubTopo.tetrahedraInROI.linkpath, name='container')
    modelSubTopo.addObject('TetrahedronFEMForceField', template='Vec3', poissonRatio=0.3,
                           youngModulus=200)

    ##########################################
    # Effector                               #
    ##########################################
    effector = model.addChild('effector')
    effector.addObject('MechanicalObject', position=[-195, 15, 0])
    effector.addObject('PositionEffector', template='Vec3', indices=[0], effectorGoal=goal.goalMO.position.linkpath,
                       useDirections=[1, 1, 0])
    effector.addObject('BarycentricMapping')

    ##########################################
    # Actuator                               #
    ##########################################
    cavity = model.addChild('cavity')
    cavity.addObject('MeshSTLLoader', name='loader', filename=path + 'PneuNets_Cavity.stl')
    cavity.addObject('MeshTopology', src=cavity.loader.linkpath, name='topo')
    cavity.addObject('MechanicalObject', name='cavity')
    cavity.addObject('SurfacePressureActuator', template='Vec3', triangles=cavity.topo.triangles.linkpath,
                     maxVolumeGrowthVariation=500, minPressure=0)
    cavity.addObject('BarycentricMapping')

    ##########################################
    # Visualization                          #
    ##########################################
    modelVisu = model.addChild('visu')
    modelVisu.addObject('MeshSTLLoader', filename=path + "PneuNets.stl", name="loader")
    modelVisu.addObject('OglModel', src=modelVisu.loader.linkpath, color=[0.7, 0.7, 0.7, 0.6])
    modelVisu.addObject('BarycentricMapping')

    return rootNode
