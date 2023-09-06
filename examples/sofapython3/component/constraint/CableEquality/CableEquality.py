import os
import numpy as np

MeshesPath = os.path.dirname(os.path.abspath(__file__)) + '/GeneratedMeshes/'

from PythonScripts.EqController import Controller
import ConstantsAccordeon as Const


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', pluginName='SofaPython3 SoftRobots SoftRobots.Inverse')
    rootNode.addObject('VisualStyle', displayFlags='hideWireframe showBehaviorModels hideCollisionModels '
                                                   'hideBoundingCollisionModels showForceFields '
                                                   'showInteractionForceFields')
    rootNode.findData('gravity').value = [0, 0, 0]
    rootNode.findData('dt').value = 0.02
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver', printLog=0, epsilon=1e-3, maxIterations=1000, tolerance=1e-4)

    VolumetricMeshPath = MeshesPath + 'Accordeon_Volumetric.vtk'
    SurfaceMeshPath = MeshesPath + 'Accordeon_Surface.stl'

    ##########################################
    # Mechanical Model                       #
    ##########################################
    model = rootNode.addChild('model')
    model.addObject('EulerImplicitSolver')
    model.addObject('SparseLDLSolver', template='CompressedRowSparseMatrixMat3x3d')
    model.addObject('MeshVTKLoader', name='loader', filename=VolumetricMeshPath)
    model.addObject('MeshTopology', src='@loader', name='container')
    model.addObject('MechanicalObject')
    model.addObject('UniformMass', totalMass=0.1)
    model.addObject('TetrahedronFEMForceField', poissonRatio=Const.PoissonRation, youngModulus=Const.YoungsModulus)
    model.addObject('BoxROI', name='BoxROI1', box=Const.FixedBoxCoordsBack, drawBoxes=True)
    model.addObject('RestShapeSpringsForceField', points='@BoxROI1.indices', stiffness=1e12)
    model.addObject('LinearSolverConstraintCorrection')

    ##########################################
    # Actuation                              #
    ##########################################
    cables = model.addChild('Cables')
    cable1 = cables.addChild('Cable1')

    TotalHeight = (Const.NSegments - 1) * Const.SegmentHeight
    CableHeights = np.linspace(0, TotalHeight, Const.NSegments)

    XCoords = np.ones(CableHeights.shape) * Const.Radius * 3 / 4
    ZCoords = np.zeros(CableHeights.shape)

    CablePositions = np.column_stack((XCoords, CableHeights, ZCoords))

    cable1.addObject('MechanicalObject', position=CablePositions.tolist())
    # Here you can set the desired cable length to reach
    cable1.addObject('CableEquality', template='Vec3d', name='CableEquality', indices=list(range(0, len(CableHeights))),
                     pullPoint=[Const.Radius * 3 / 4, -10, 0], eqDisp=5, printLog=True)
    cable1.addObject('BarycentricMapping')

    ##########################################
    # Visualization                          #
    ##########################################
    modelVisu = model.addChild('visu')
    modelVisu.addObject('MeshSTLLoader', filename=SurfaceMeshPath, name="loader")
    modelVisu.addObject('OglModel', src="@loader")
    modelVisu.addObject('BarycentricMapping')

    rootNode.addObject(Controller(name="EqualityController", RootNode=rootNode))

    return rootNode
