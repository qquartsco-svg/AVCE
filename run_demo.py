#!/usr/bin/env python3
"""
자율선박 엔진 데모: Potential Field → ψ_ref, U_ref; (선택) OrbitStabilizer 보정.
OrbitStabilizer 미설치 시에도 경로 제어만으로 동작.
"""
import sys
from pathlib import Path

# 패키지 루트 (run_demo.py 기준 상위 = avce)
_root = Path(__file__).resolve().parent
if str(_root) not in sys.path:
    sys.path.insert(0, str(_root))

from avce import (
    VesselState,
    Waypoint,
    Obstacle,
    VesselController,
    path_controller,
    OrbitStabilizerAdapter,
)

def main():
    # 상태·웨이포인트·장애물
    state = VesselState(
        xi_m=0.0, eta_m=0.0, psi_rad=0.0,
        u_ms=2.0, v_ms=0.0, r_rads=0.0,
    )
    wp = Waypoint(xi_m=1000.0, eta_m=500.0, U_d_ms=5.0)
    obstacles = [
        Obstacle(xi_m=400.0, eta_m=200.0, A=2.0, sigma_m=150.0),
    ]

    # 경로만 (OrbitStabilizer 없이)
    out = path_controller(state, wp, obstacles)
    print("Path only:")
    print(f"  psi_ref [rad] = {out.psi_ref_rad:.4f}, U_ref [m/s] = {out.U_ref_ms:.2f}, F_pf = {out.F_pf_mag:.2f}")

    # 통합 컨트롤러 (OrbitStabilizer 축은 넣되, 미설치면 보정 0)
    ctrl = VesselController(
        waypoint=wp,
        obstacles=obstacles,
        orbit_stabilizer_axes=["psi"],
    )
    result = ctrl.step(state)
    print("VesselController.step():")
    print(f"  psi_ref = {result.psi_ref_rad:.4f}, U_ref = {result.U_ref_ms:.2f}")
    print(f"  corrections: {list(result.corrections.keys())}")
    if result.corrections:
        for aid, corr in result.corrections.items():
            print(f"    {aid}: correction_force={corr.correction_force:.4f}, confidence={corr.confidence:.2f}")

    # OrbitStabilizer 사용 가능 여부
    adapter = OrbitStabilizerAdapter(axis_ids=["psi"])
    print(f"OrbitStabilizer available: {adapter.available}")

    print("Done.")

if __name__ == "__main__":
    main()
