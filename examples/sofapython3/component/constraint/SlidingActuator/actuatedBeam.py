import Sofa
from math import sin, cos
import os

path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='SofaPython3')
    rootNode.addObject('RequiredPlugin', name='BeamAdapter')
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='SoftRobots.Inverse')
    rootNode.addObject('VisualStyle', displayFlags='showVisualModels showBehaviorModels showCollisionModels '
                                                   'showForceFields showInteractionForceFields')

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver', epsilon=0.001, printLog=False, tolerance=1e-10, maxIterations=1000)

    #########################################
    # Goal for end effector                 #
    #########################################
    goal = rootNode.addChild('goal')
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-4)
    goal.addObject('MechanicalObject', name='dofs', template='Rigid3', showObject=True, drawMode=2, showObjectScale=1,
                   position=[30.0, 0.0, 0.0, 0, 0, 0, 1])
    goal.addObject('UncoupledConstraintCorrection')

    ##########################################
    # Beam Model                             #
    ##########################################
    model = rootNode.addChild('model')
    model.addObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
    model.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixMat3x3d")
    model.addObject('GenericConstraintCorrection')
    model.addObject('MeshTopology', position=[[0, 0, 0], [5, 0, 0], [10, 0, 0], [15, 0, 0], [20, 0, 0]],
                    lines=[[0, 1], [1, 2], [2, 3], [3, 4]])

    teta = 3.14 / 8;
    model.addObject('MechanicalObject', template='Rigid3', name='dofs',
                    position=[[0, 0, 0, 0, 0, 0, 1], [5, 0, 0, 0, 0, 0, 1], [10, 0, 0, 0, 0, 0, 1],
                              [15, 0, 0, 0, 0, 0, 1], [20, 0, 0, 0, 0, 0, 1]],
                    rest_position=[[0, 0, 0, 0, 0, 0, 1], [5, 0, 0, 0, 0, 0, 1], [10, 0, 0, 0, 0, 0, 1],
                                   [15, 0, 0, 0, 0, 0, 1], [19, 2, 0, 0, 0, sin(teta), cos(teta)]],
                    showObject=True, drawMode=1, showObjectScale=0.5)

    model.addObject('BeamInterpolation', dofsAndBeamsAligned=True, straight=False, defaultYoungModulus=100, radius=0.5)
    model.addObject('AdaptiveBeamForceFieldAndMass', computeMass=True, massDensity=0.000001)
    # Unfortunatly, the component SlidingActuator needs to be used with a PartialFixedConstraint.
    # Ex: if SlidingActuator simulates a single dof (e.g. translation along x),
    # the other dofs must be fixed using this component
    model.addObject('PartialFixedConstraint', indices=0, fixedDirections=[0, 1, 1, 1, 1, 0])
    model.addObject('RestShapeSpringsForceField', points=0, angularStiffness=1000, stiffness=1000)

    ##########################################
    # Actuator                               #
    ##########################################
    # The actuation consist of two dofs at one extremity of the beam
    actuator = model.addChild('actuator')
    actuator.addObject('MechanicalObject', template='Rigid3', position=[0, 0, 0, 0, 0, 0, 1])
    # 1 - Translation on x direction
    actuator.addObject('SlidingActuator', template='Rigid3', name="actuator0", indices=0, direction=[1, 0, 0, 0, 0, 0],
                       maxNegativeDisp=10, maxPositiveDisp=10, maxDispVariation=0.1, maxForce=1000, minForce=-1000)
    # 2 - Rotation around z axis
    actuator.addObject('SlidingActuator', template='Rigid3', name="actuator1", indices=0, direction=[0, 0, 0, 0, 0, 1],
                       maxNegativeDisp=1, maxPositiveDisp=1, maxDispVariation=0.1, maxForce=100)
    actuator.addObject('AdaptiveBeamMapping', name='mapping', mapForces=False, mapMasses=False)

    ##########################################
    # Effector                               #
    ##########################################
    effector = model.addChild('effector')
    effector.addObject('MechanicalObject', position=[0.0, 0.0, 0.0])
    effector.addObject('PositionEffector', indices=0, effectorGoal=goal.dofs.findData('position').getLinkPath(),
                       useDirections=[1, 1, 0])
    # No need to follow the target in z direction because the beam working space is in the [x,y] plane
    effector.addObject('RigidMapping', index=4)

    return rootNode
