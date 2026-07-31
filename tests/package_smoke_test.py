from pathlib import Path

import a1z


package_root = Path(a1z.__file__).parent
typing_marker = package_root / "py.typed"
urdf = package_root / "robot_models" / "a1z" / "A1Z_Flange.urdf"

assert typing_marker.is_file(), f"PEP 561 marker is missing: {typing_marker}"
assert urdf.is_file(), f"Packaged URDF is missing: {urdf}"
assert "<robot" in urdf.read_text(encoding="utf-8")
