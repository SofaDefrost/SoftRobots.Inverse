def createScene(rootnode):
    from splib3.animation import animate, AnimationManager
    rootnode.addObject(AnimationManager(rootnode))
    rootnode.addObject('RequiredPlugin', pluginName=['SoftRobots', 'SoftRobots.Inverse', 'SofaPython3'])
    rootnode.gravity = [0, 0, 0]
    rootnode.addObject('DefaultVisualManagerLoop')
    rootnode.addObject('FreeMotionAnimationLoop')
    # Add the inverse solver
    rootnode.addObject('QPInverseProblemSolver')

    cube = rootnode.addChild('Cube')
    cube.addObject('EulerImplicitSolver')
    cube.addObject('SparseLDLSolver')
    cube.addObject('GenericConstraintCorrection')
    cube.addObject('MechanicalObject', template='Rigid3', position=[-10, 0, 0, 0, 0, 0, 1])
    cube.addObject('UniformMass', totalMass=0.01)
    # SlidingActuator needs a PartialFixedConstraint to fix the other directions
    cube.addObject('PartialFixedConstraint', template='Rigid3', fixedDirections=[0, 1, 1, 1, 1, 1])
    cube.addObject('SlidingActuator', template='Rigid3', indices=0, direction=[1, 0, 0, 0, 0, 0], maxDispVariation=0.1)

    visual = cube.addChild('Visu')
    visual.addObject('MeshOBJLoader', filename='mesh/cube.obj', scale=10)
    visual.addObject('OglModel', src='@MeshOBJLoader')
    visual.addObject('RigidMapping')

    # Set an effector for the inverse resolution
    effector = cube.addChild('Effector')
    effector.addObject('MechanicalObject', position=[10, 0, 0],  # front of the cube
                       showObject=True, drawMode=2, showObjectScale=1)
    effector.addObject('PositionEffector', indices=0, effectorGoal=[0, 0, 0])
    effector.addObject('RigidMapping')

    # Set a target for the effector that can be moved interactively
    target = rootnode.addChild('Target')
    target.addObject('EulerImplicitSolver', firstOrder=True)
    target.addObject('CGLinearSolver', tolerance=1e-5, iterations=20, threshold=1e-5)
    target.addObject('MechanicalObject', position=[30, 0, 0], showObject=True, drawMode=2, showObjectScale=3)
    target.addObject('PartialFixedConstraint', fixedDirections=[0, 1, 1])
    target.addObject('UniformMass', totalMass=0.1)
    target.addObject('UncoupledConstraintCorrection')

    # Link the effector goal to the target
    effector.PositionEffector.effectorGoal.setParent(target.MechanicalObject.position)

