# Copyright (c) 2022-2024, The ORBIT Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

from omni.isaac.orbit.utils import configclass

from ..asset_base_cfg import AssetBaseCfg
from .deform_object import DeformObject


@configclass
class DeformObjectCfg(AssetBaseCfg):
    """Configuration parameters for a deformable object."""

    @configclass
    class InitialStateCfg(AssetBaseCfg.InitialStateCfg):
        """Initial state of the deformable body."""

        lin_vel: tuple[float, float, float] = (0.0, 0.0, 0.0)
        """Linear velocity of the root in simulation world frame. Defaults to (0.0, 0.0, 0.0)."""
        ang_vel: tuple[float, float, float] = (0.0, 0.0, 0.0)
        """Angular velocity of the root in simulation world frame. Defaults to (0.0, 0.0, 0.0)."""

    ##
    # Initialize configurations.
    ##

    class_type: type = DeformObject

    init_state: InitialStateCfg = InitialStateCfg()
    """Initial state of the deformable object. Defaults to identity pose with zero velocity."""
