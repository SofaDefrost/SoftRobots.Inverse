import os

dirPath = os.path.dirname(os.path.abspath(__file__)) + '/'


# Example of joint actuator usage.
# In this simulation, there is one articulation, on which a servo arm is attached.
# We use the component JointActuator, with PositionEffector to define the desired
# position of the end effector (tip of the arm),
# and the solver QPInverseProblemSolver to solve the inverse problem.


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='SoftRobots.Inverse')

    rootNode.dt = 0.1
    rootNode.gravity = [0., -981., 0.]
    rootNode.addObject('VisualStyle', displayFlags='showBehaviorModels')
    rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765, 1.])
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('QPInverseProblemSolver', epsilon=0)

    # Target position of the end effector
    goal = rootNode.addChild('Goal')
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver')
    goal.addObject('MechanicalObject', name='dofs', template='Rigid3', position=[5, 0, 0, 0, 0, 0, 1],
                   showObject=1, showObjectScale=1, drawMode=1)
    goal.addObject('UncoupledConstraintCorrection')

    # Simulation node
    simulation = rootNode.addChild('Simulation')
    simulation.addObject('EulerImplicitSolver')
    simulation.addObject('CGLinearSolver')

    object = simulation.addChild('Object')
    object.addObject('MechanicalObject', name='dofs', template='Vec1', position=0.)
    object.addObject('JointActuator', template='Vec1', index=0,
                     maxAngle=1., minAngle=-1., maxAngleVariation=0.005)

    rigid = object.addChild('Rigid')
    rigid.addObject('MechanicalObject', template='Rigid3', name='dofs',
                    position=[[0., 0., 0., 0., 0., 0., 1.], [4.5, 0., 0., 0., 0., 0., 1.]])
    rigid.addObject('PositionEffector', template='Rigid3', indices=1,
                    effectorGoal=goal.dofs.findData('position').getLinkPath(), useDirections=[1, 1, 0, 0, 0, 0])
    rigid.addObject('BeamFEMForceField', name='FEM', radius=0.2, youngModulus=1e3, poissonRatio=0.45)
    rigid.addObject('MeshTopology', lines=[0, 1])

    visual = rigid.addChild('VisualModel')
    visual.addObject('MeshSTLLoader', name='loader', filename=dirPath + 'mesh/arm.stl', translation=[-4.5, 0., 0.],
                     scale3d=[0.1, 0.1, 0.1])
    visual.addObject('MeshTopology', src='@loader')
    visual.addObject('OglModel', color=[0.9, 0.9, 0.9, 1.])
    visual.addObject('RigidMapping', index=1)

    rigid.addObject('GenerateRigidMass', name='mass', density=0.002, src=visual.loader.getLinkPath())
    rigid.addObject('FixedConstraint', template='Rigid3', indices=0)
    rigid.addObject('ArticulatedSystemMapping', input1=object.dofs.getLinkPath(), output=rigid.dofs.getLinkPath())
    rigid.addObject('UniformMass', vertexMass=rigid.mass.findData('rigidMass').getLinkPath())

    object.addObject('UncoupledConstraintCorrection')
    object.addObject('ArticulatedHierarchyContainer')

    articulationCenters = object.addChild('Articulation')
    articulationCenter = articulationCenters.addChild('ArticulationCenter')
    articulationCenter.addObject('ArticulationCenter', parentIndex=0, childIndex=1, posOnParent=[0., 0., 0.],
                                 posOnChild=[-4.5, 0., 0.], articulationProcess=0)
    articulation = articulationCenter.addChild('Articulations')
    articulation.addObject('Articulation', translation=False, rotation=True, rotationAxis=[0, 0, 1],
                           articulationIndex=0)

    return rootNode
