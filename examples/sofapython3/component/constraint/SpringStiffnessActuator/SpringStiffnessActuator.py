import Sofa
import numpy as np

def random_unit_ball_point():
    while True:
        point = np.random.uniform(-1, 1, size=3)
        if np.sum(point**2) <= 1:
            return point

class TargetController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

    def onAnimateEndEvent(self, *args, **kwargs):
        root = self.getContext().getRoot()
        loss = root.inverse_solver.objective.value
        if loss < 0.01:
            new_target = np.array([ + np.random.uniform(-1, 1), 0 + np.random.uniform(-1, 1), 11 + np.random.uniform(-1, 1)])
            root.Target.state.position.value = new_target[None, :].tolist()


def createScene(root):
    root.gravity = [0, 0, 0]
    root.dt = 0.01

    with root.addChild("Plugins") as plugins:
        plugins.addObject("RequiredPlugin", pluginName=[
            'SoftRobots.Inverse',
            'Sofa.Component.AnimationLoop', # Needed to use components [FreeMotionAnimationLoop]
            'Sofa.Component.Constraint.Lagrangian.Correction', # Needed to use components [GenericConstraintCorrection]
            'Sofa.Component.Constraint.Projective', # Needed to use components [FixedProjectiveConstraint]
            'Sofa.Component.Engine.Select', # Needed to use components [BoxROI]
            'Sofa.Component.IntegrationScheme.Backward', # Needed to use components [StaticEquilibriumIntegrationScheme]
            'Sofa.Component.LinearSolver.Direct', # Needed to use components [SparseLDLSolver]
            'Sofa.Component.LinearSystem', # Needed to use components [MatrixLinearSystem]
            'Sofa.Component.Mapping.Linear', # Needed to use components [BarycentricMapping]
            'Sofa.Component.Mass', # Needed to use components [FEMMass,NodalMassDensity]
            'Sofa.Component.SolidMechanics.FEM.Elastic', # Needed to use components [VonMisesStress]
            'Sofa.Component.SolidMechanics.FEM.HyperElastic', # Needed to use components [HyperelasticityFEMForceField,StVenantKirchhoffMaterial]
            'Sofa.Component.SolidMechanics.Spring', # Needed to use components [SpringForceField]
            'Sofa.Component.StateContainer', # Needed to use components [MechanicalObject]
            'Sofa.Component.Topology.Container.Dynamic', # Needed to use components [HexahedronSetTopologyContainer,HexahedronSetTopologyModifier]
            'Sofa.Component.Topology.Container.Grid', # Needed to use components [RegularGridTopology]
            'Sofa.Component.Visual', # Needed to use components [VisualMesh,VisualPointCloud,VisualStyle]
        ])

    root.addObject("VisualStyle", displayFlags="showBehavior showBehaviorModels showMappings")

    root.addObject("FreeMotionAnimationLoop")
    root.addObject("QPInverseProblemSolver", name="inverse_solver")
    root.addObject(TargetController(root))

    n = 10  # Number of cubes in the beam
    r = 3  # Subdivision of cubes
    m = 5  # Number of springs

    with root.addChild("Fixed") as fixed:
        fixed.addObject("MechanicalObject", template="Vec3", name="state", position=(0, 0, 20, -10, 0, 5, 10, 0, 5, 0, -10, 5, 0, 10, 5), showObject="true", drawMode="1", showObjectScale="0.08")

    with root.addChild("Target") as target:
        target.addObject("MechanicalObject", template="Vec3", name="state", position=(0, 0, 11))
        target.addObject("VisualPointCloud", position="@state.position", drawMode="Sphere", sphereRadius=0.5, color="purple")

    root.addObject("RegularGridTopology", name="grid", min=(-0.5, -0.5, -0.5), max=(0.5, 0.5, n-0.5), n=(r+1, r+1, n*r+1))
    with root.addChild("Physics") as physics:
        physics.addObject("StaticEquilibriumIntegrationScheme", name="static", maxNbIterationsNewton=1, maxNbIterationsLineSearch=1)
        # physics.addObject("EulerImplicitIntegrationScheme", name="solver")
        physics.addObject("MatrixLinearSystem", template="CompressedRowSparseMatrixMat3x3d", name="system", printLog="false")
        physics.addObject("SparseLDLSolver", template="CompressedRowSparseMatrixMat3x3d", name="linear_solver", printLog="false")

        physics.addObject("HexahedronSetTopologyContainer", name="topology", src="@../grid")
        physics.addObject("HexahedronSetTopologyModifier", name="topology_modifier")
        physics.addObject("MechanicalObject", template="Vec3", name="state", position="@../grid.position")

        physics.addObject("NodalMassDensity", property=1)
        physics.addObject("FEMMass", name="mass", template="Vec3,Hexahedron")
        physics.addObject("StVenantKirchhoffMaterial", name="material", youngModulus="1e6", poissonRatio=0.45)
        physics.addObject(
            "HyperelasticityFEMForceField",
            template="Vec3,Hexahedron",
            name="elasticity"
        )
        physics.addObject("VonMisesStress", template="Vec3,Hexahedron", name="stress", topology="@grid",
                   stressEvaluator="@material", colorMap="green yellow orange red purple #221C35")

        physics.addObject("GenericConstraintCorrection", linearSolver="@linear_solver")

        with physics.addChild("Constraint") as constraint:
            constraint.addObject("BoxROI", name="box", box=((-2, -2, -0.6), (2, 2, -0.4)))
            constraint.addObject("FixedProjectiveConstraint", indices="@box.indices")

        with physics.addChild("Extremity") as extremity:
            extremity.addObject("MechanicalObject", template="Vec3", name="state", position=(0, 0, n-0.5))
            extremity.addObject("BarycentricMapping", input="@../state", output="@state")
            extremity.addObject('PositionEffector', template='Vec3', name="effector", indices=[0],
                               effectorGoal="@/Target/state.position")
            extremity.addObject("VisualPointCloud", position="@state.position", drawMode="Sphere", sphereRadius=0.5, color="orange")

            extremity.addObject(
                "SpringForceField", name="spring",
                stiffness=(1.0,)*m,
                length=(0.0,)*m,  damping=(0,)*m, elongationOnly=(0,)*m, enabled=(1,)*m,
                object1="@/Physics/Extremity/state", indices1=(0, 0, 0, 0, 0),
                object2="@/Fixed/state", indices2=(0, 1, 2, 3, 4),
            )
            extremity.addObject('SpringStiffnessActuator', template='Vec3', name='actuator', minValue=0., maxValue=1e12)

        physics.addObject("VisualMesh", position="@state.position", topology="@grid")

