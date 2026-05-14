"""
gen_pyrightconfig.py — Write pyrightconfig.json from build.bat
==============================================================
Extracted from build.bat to avoid cmd.exe multi-line quoting issues.
Run from Satellite_GNC root:
    python sim_python/gen_pyrightconfig.py
"""
import json
import pathlib
import os

# Resolve paths relative to this script's location (sim_python/) -> repo root
_HERE     = pathlib.Path(__file__).resolve().parent        # sim_python/
_SIL_ROOT = _HERE.parent                                   # Satellite_GNC/
_FLIGHT_SIM = _SIL_ROOT.parent / "flight sim"             # ../flight sim/

cfg = {
    "pythonVersion": "3.11",
    "pythonPlatform": "Windows",
    "extraPaths": [
        str(_FLIGHT_SIM).replace("\\", "/"),
        str(_SIL_ROOT).replace("\\", "/"),
        str(_SIL_ROOT / "sim_python").replace("\\", "/"),
    ],
    "exclude": [
        "**/node_modules",
        "**/__pycache__"
    ],
    # Suppress squiggles for flight sim modules (cw_dynamics, spacecraft, etc.)
    # Resolved at runtime via sys.path; Pylance cannot index them because
    # "flight sim/" has no py.typed marker or stubs.
    "reportMissingImports": "none",
    "reportMissingModuleSource": "none",
    "reportUnknownMemberType": "none",
    "reportUnknownVariableType": "none",
    "reportUnknownArgumentType": "none",
    "useLibraryCodeForTypes": True,
    "ignore": [
        str(_FLIGHT_SIM).replace("\\", "/"),
    ],
}

out = _SIL_ROOT / "pyrightconfig.json"
out.write_text(json.dumps(cfg, indent=4))
print(f"  Written: {out}")