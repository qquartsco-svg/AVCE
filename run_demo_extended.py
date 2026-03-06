#!/usr/bin/env python3
"""
확장 데모: Cerebellum 프로파일, 램 외란, 폐루프 시뮬레이션.
"""
import sys
from pathlib import Path
_root = Path(__file__).resolve().parent
if str(_root) not in sys.path:
    sys.path.insert(0, str(_root))

from avce import (
    VesselState,
    Waypoint,
    Obstacle,
    VesselController,
    CerebellumProfile,
    ProfilePoint,
    RammingImpact,
    RammingDisturbance,
    run_simulation,
    DynamicsParams,
)

def main():
    state0 = VesselState(0.0, 0.0, 0.0, 2.0, 0.0, 0.0)
    wp = Waypoint(800.0, 400.0, U_d_ms=5.0)
    obstacles = [Obstacle(300.0, 150.0, A=1.5, sigma_m=120.0)]

    # Cerebellum: 0~20s 선형 감쇠 프로파일
    profile = CerebellumProfile(T_psi_s=8.0, T_U_s=12.0)
    ctrl = VesselController(
        waypoint=wp,
        obstacles=obstacles,
        cerebellum_profile=profile,
        profile_blend=0.3,
        orbit_stabilizer_axes=["psi"],
    )

    # 램 이벤트: t=5s에 충돌
    ramming = RammingDisturbance()
    ramming.add_impact(RammingImpact(t_impact_s=5.0, T_ram_nm=2.0e5, sigma_ram_s=0.5))

    results = run_simulation(
        state0,
        ctrl,
        T_s=30.0,
        dt_s=0.1,
        dynamics_params=DynamicsParams(),
        ramming=ramming,
    )

    print(f"Steps: {len(results)}")
    last = results[-1].state
    print(f"Final: xi={last.xi_m:.1f} m, eta={last.eta_m:.1f} m, psi={last.psi_rad:.3f} rad")
    print(f"Goal:  xi={wp.xi_m} m, eta={wp.eta_m} m")
    dist = ((last.xi_m - wp.xi_m)**2 + (last.eta_m - wp.eta_m)**2)**0.5
    print(f"Distance to goal: {dist:.1f} m")
    print("Done.")

if __name__ == "__main__":
    main()
