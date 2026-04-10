# Satellite GNC — SIL Framework
## From Python Sim to C Flight Code

### Folder layout
```
C:\Users\Venkat\OneDrive\Desktop\appex\
│
├── flight sim\          ← existing Python sim (never touched by this repo)
│   ├── main.py
│   ├── lambert_controller.py
│   ├── th_ekf.py
│   └── ...
│
└── Satellite_GNC\       ← this repo
    ├── src_c\
    │   ├── linalg.h         matrix math, no malloc
    │   ├── th_ekf.h         TH-EKF header
    │   └── th_ekf.c         TH-EKF implementation
    ├── sim_python\
    │   ├── wrapper.py       ctypes bridge (drop-in for THEKF)
    │   └── verify_sil.py    Python golden model comparison
    ├── tests\
    │   └── test_thekf.c     standalone C verification
    ├── build.bat
    └── README.md
```

### Step 1: Install compiler
```
winget install MSYS2.MSYS2
```
Open **MSYS2 MinGW64** terminal (not regular MSYS2):
```
pacman -S mingw-w64-x86_64-gcc
```
Add to Windows PATH: `C:\msys64\mingw64\bin`

Verify in a normal Command Prompt:
```
gcc --version
```

### Step 2: Build and verify
From `Satellite_GNC\`:
```
build.bat
```
This compiles `gnc_lib.dll`, runs the C unit test, then runs the Python SIL comparison.

### Step 3: Use in your sim (optional one-line swap)
In `flight sim\main.py`, change the import:
```python
# Before:
from th_ekf import THEKF

# After — C when compiled, falls back to Python automatically:
import sys
sys.path.insert(0, r"C:\Users\Venkat\OneDrive\Desktop\appex\Satellite_GNC")
from sim_python.wrapper import THEKF_C as THEKF
```

### Pass criteria (verify_sil.py)
| Metric | Threshold |
|--------|-----------|
| Position divergence | < 0.1 mm |
| Velocity divergence | < 0.1 µm/s |
| Covariance error | < 1e-6 |

### Porting roadmap
| Priority | File | Status |
|----------|------|--------|
| 1 | `th_ekf.py` → `src_c/th_ekf.c` | ✅ Done |
| 2 | `mekf.py` → `src_c/mekf.c` | 🔜 Next |
| 3 | `lambert_controller.py` → `src_c/rpod_ctrl.c` | 🔜 Later |