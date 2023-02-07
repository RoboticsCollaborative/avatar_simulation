import numpy as np
import os

from pydrake.common import FindResourceOrThrow
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.geometry import (
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    Role,
    StartMeshcat,
)
from pydrake.visualization import ModelVisualizer

meshcat = StartMeshcat()
test_mode = True if "TEST_SRCDIR" in os.environ else False

visualizer = ModelVisualizer(meshcat=meshcat)
visualizer.parser().package_map().AddPackageXml("../package.xml")
# print(parser.package_map().GetPackageNames())
visualizer.parser().AddModels("../urdf/avatar_gripper_3f.urdf")


visualizer.Run(loop_once=test_mode)
