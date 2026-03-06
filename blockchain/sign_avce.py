#!/usr/bin/env python3
"""
AVCE 모듈 블록체인 서명 — 핵심 파일 해시 기록.
실행: 패키지 루트(avce/)에서
  python3 blockchain/sign_avce.py
→ blockchain/signature.json, blockchain/SIGNATURE.md 생성.
"""
import hashlib
import json
from pathlib import Path
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parent.parent
AVCE = ROOT / "avce"
BLOCKCHAIN = ROOT / "blockchain"

FILES = [
    "avce/__init__.py",
    "avce/controller.py",
    "avce/core/__init__.py",
    "avce/core/state.py",
    "avce/core/constants.py",
    "avce/core/ramming.py",
    "avce/core/estimator.py",
    "avce/control/__init__.py",
    "avce/control/potential_field.py",
    "avce/control/path_controller.py",
    "avce/control/cerebellum_profile.py",
    "avce/memory/__init__.py",
    "avce/memory/well_memory.py",
    "avce/simulation/__init__.py",
    "avce/simulation/dynamics.py",
    "avce/simulation/loop.py",
    "avce/integration/__init__.py",
    "avce/integration/orbit_stabilizer_adapter.py",
    "avce/integration/grid5d_adapter.py",
]

def main():
    now = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    entries = []
    for rel in FILES:
        p = ROOT / rel
        if not p.exists():
            continue
        raw = p.read_bytes()
        h = hashlib.sha256(raw).hexdigest()
        entries.append({"file": rel, "sha256": h, "bytes": len(raw)})
    out = {
        "module": "AVCE",
        "description": "Autonomous Vessel Control Engine — 자율선박·쇄빙선 제어",
        "signed_at": now,
        "entries": entries,
    }
    BLOCKCHAIN.mkdir(parents=True, exist_ok=True)
    (BLOCKCHAIN / "signature.json").write_text(
        json.dumps(out, indent=2, ensure_ascii=False), "utf-8"
    )
    lines = [
        "# AVCE — 블록체인 서명",
        "",
        f"**서명 시각**: {now}",
        "",
        "| 파일 | SHA256 |",
        "|------|--------|",
    ]
    for e in entries:
        lines.append(f"| `{e['file']}` | `{e['sha256']}` |")
    (BLOCKCHAIN / "SIGNATURE.md").write_text("\n".join(lines), "utf-8")
    print(f"Signed {len(entries)} files -> blockchain/signature.json, blockchain/SIGNATURE.md")

if __name__ == "__main__":
    main()
