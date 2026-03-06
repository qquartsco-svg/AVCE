"""Control: 퍼텐셜·경로·Cerebellum 프로파일
6-DOF 확장: DepthController, IceInterface
"""
from .potential_field import (
    U_goal,
    U_obs,
    U_obs_gaussian,
    U_obs_distance,
    U_ice_linear,
    U_total,
    GradientMethod,
    gradient_U_analytic,
    gradient_U_numeric,
    gradient_U,
    psi_ref_from_gradient,
)
from .path_controller import path_controller, PathOutput
from .cerebellum_profile import CerebellumProfile, ProfilePoint, exponential_profile
# ── 6-DOF 확장 ────────────────────────────────────────────────────────────────
from .depth_controller import DepthControlConfig, DepthController
from .ice_interface import IceInterfaceConfig, IceContactResult, IceInterface

__all__ = [
    # ── potential field — scalar ──────────────────────────────────────────────
    "U_goal",
    "U_obs",
    "U_obs_gaussian",
    "U_obs_distance",
    "U_ice_linear",
    "U_total",
    # ── potential field — gradient ───────────────────────────────────────────
    "GradientMethod",
    "gradient_U_analytic",
    "gradient_U_numeric",
    "gradient_U",
    "psi_ref_from_gradient",
    # ── path controller ──────────────────────────────────────────────────────
    "path_controller",
    "PathOutput",
    # ── cerebellum ───────────────────────────────────────────────────────────
    "CerebellumProfile",
    "ProfilePoint",
    "exponential_profile",
    # ── 6-DOF depth control ──────────────────────────────────────────────────
    "DepthControlConfig",
    "DepthController",
    # ── 6-DOF ice interface ──────────────────────────────────────────────────
    "IceInterfaceConfig",
    "IceContactResult",
    "IceInterface",
]
