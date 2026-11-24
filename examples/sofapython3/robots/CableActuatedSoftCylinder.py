import Sofa

from Sofa.Units.Definitions import s, cm, m, g, Pa
from Sofa.Units.UnitSystem import MechanicalUnitSystem
import numpy as np

scene_unit = MechanicalUnitSystem(s, m, g)

cylinder_radius = scene_unit(1, cm)
cylinder_height = scene_unit(10, cm)
cylinder_mass = scene_unit(20, g)
cylinder_volume = np.pi * cylinder_radius**2 * cylinder_height
cylinder_density = cylinder_mass / cylinder_volume

class TargetController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.state = kwargs.get("state")
        self.t = 0.0

        # Sphere centered at the base of the cylinder
        self.center = np.array([0.0, 0.0, 0.0])
        self.radius = 0.95 * cylinder_height
        self.amplitude = 0.2
        self.period = 4.0
        self.ramp_time = 3.0

    def onAnimateBeginEvent(self, event):
        self.t += event["dt"]
        w = 2 * np.pi / self.period
        s = min(self.t / self.ramp_time, 1.0)
        ramp = s * s * (3 - 2 * s)
        R = cylinder_height * (1 - 0.05 * ramp)
        u = ramp * self.amplitude * np.sin(w * self.t)
        v = ramp * self.amplitude * np.sin(w * self.t) * np.cos(w * self.t)
        z = np.sqrt(1.0 - u**2 - v**2)
        self.state.position.value = [(R * np.array([u, v, z])).tolist()]

def createScene(rootNode):

    rootNode.gravity = [0, 0, 0]
    rootNode.dt = scene_unit(0.01, s)

    with rootNode.addChild("plugins") as plugins:
        plugins.addObject('RequiredPlugin', pluginName=[
            'SoftRobots',
            'SoftRobots.Inverse',
            'Sofa.Component.AnimationLoop',
            'Sofa.Component.Constraint.Lagrangian.Correction',
            'Sofa.Component.Constraint.Projective',
            'Sofa.Component.Engine.Generate',
            'Sofa.Component.Engine.Select',
            'Sofa.Component.IntegrationScheme.Backward',
            'Sofa.Component.LinearSolver.Direct',
            'Sofa.Component.Mapping.Linear',
            'Sofa.Component.Mass',
            'Sofa.Component.SolidMechanics.FEM.HyperElastic',
            'Sofa.Component.StateContainer',
            'Sofa.Component.Topology.Container.Dynamic',
            'Sofa.Component.Visual'
            ])
  
    rootNode.addObject("FreeMotionAnimationLoop")
    rootNode.addObject('QPInverseProblemSolver')

    with rootNode.addChild("geometry") as geometry:
        geometry.addObject("GenerateCylinder", name="loader",
                           radius=cylinder_radius, height=cylinder_height,
                           resHeight=12, resCircumferential=12, resRadial=3)

    with rootNode.addChild('target') as target:
        target_state = target.addObject('MechanicalObject', template="Vec3", name="target_state", position=[0, 0, cylinder_height])
        target.addObject('VisualPointCloud', position="@target_state.position", drawMode="Sphere", sphereRadius=cylinder_radius/10, color="orange")
        target.addObject(TargetController(state=target_state))
        target.addObject('TrailRenderer', template='Vec3', position="@target_state.position", nbSteps=300, thickness=10, color="black")

    with rootNode.addChild("physics") as physics:

        physics.addObject('EulerImplicitIntegrationScheme')
        physics.addObject('SparseLDLSolver', template="CompressedRowSparseMatrixMat3x3")

        physics.addObject("MechanicalObject", src="@../geometry/loader", name="state", position="@../geometry/loader.output_position")
        physics.addObject("TetrahedronSetTopologyContainer", tetrahedra="@../geometry/loader.tetrahedra", position="@../geometry/loader.output_position", createTriangleArray=True)
        physics.addObject('NodalMassDensity', property=cylinder_density)
        physics.addObject("FEMMass", template="Vec3,Tetrahedron")

        physics.addObject('StVenantKirchhoffMaterial', name='material', youngModulus=scene_unit(5e4, Pa), poissonRatio=0.45)
        physics.addObject('HyperelasticityFEMForceField', template="Vec3,Tetrahedron")

        physics.addObject('BoxROI', name='box', box=[-cylinder_radius, -cylinder_radius, -0.001, cylinder_radius, cylinder_radius, 0.001], drawBoxes=True)
        physics.addObject('FixedProjectiveConstraint', indices="@box.indices")

        with physics.addChild('effector') as effector:
            effector.addObject('MechanicalObject', template="Vec3", name="effector_state", position=[0, 0, cylinder_height])
            effector.addObject('BarycentricMapping', template="Vec3,Vec3", input="@../state", output="@effector_state", mapForces=False, mapMasses=False)
            effector.addObject('PositionEffector', template="Vec3", name="effector", indices=[0], effectorGoal="@/target/target_state.position")
            effector.addObject('TrailRenderer', template='Vec3', position="@effector_state.position", nbSteps=300, thickness=5, color="orange")

        with physics.addChild("actuator") as actuator:

            nb_subdivisions = 5
            nb_cables = 4

            for i in range(nb_cables):
                with actuator.addChild(f"cable{i}") as cable:
                    starting_point = [np.sin(i * 2 * np.pi / nb_cables) * cylinder_radius * 2 / 3,
                                      np.cos(i * 2 * np.pi / nb_cables) * cylinder_radius * 2 / 3,
                                      0]
                    cable_position = [
                        [starting_point[0], starting_point[1], j * cylinder_height / nb_subdivisions]
                        for j in range(nb_subdivisions + 1)]
                    cable.addObject('MechanicalObject', template="Vec3", name="actuator_state", position=cable_position)
                    cable.addObject('BarycentricMapping', template="Vec3,Vec3", input="@../../state", output="@actuator_state", mapForces=False, mapMasses=False)
                    cable.addObject('CableActuator', template="Vec3", name="actuator",
                                    indices=list(range(1, nb_subdivisions + 1)), pullPoint=starting_point,
                                    maxPositiveDisp=cylinder_radius, maxNegativeDisp=0, minForce=0,
                                    maxDispVariation=cylinder_radius / 20)
                    cable.addObject('VisualPointCloud', position="@actuator_state.position", drawMode="Sphere", sphereRadius=cylinder_radius/10, color="gold")

        physics.addObject('LinearSolverConstraintCorrection')

        physics.addObject("VisualMesh", position="@state.position")


    return rootNode
