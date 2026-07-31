from pathlib import Path

import a1z


package_root = Path(a1z.__file__).parent
urdf = package_root / "robot_models" / "a1z" / "A1Z_Flange.urdf"

assert urdf.is_file(), f"Packaged URDF is missing: {urdf}"
assert "<robot" in urdf.read_text(encoding="utf-8")
