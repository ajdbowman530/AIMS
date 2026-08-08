import os
import sys
import subprocess
from pathlib import Path
from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext

class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=""):
        Extension.__init__(self, name, sources=["telemetry.cpp", "scheduler.cpp", "aero_controller.cpp", "autothrottle.cpp", "flight_executive.cpp"])
        self.sourcedir = os.path.abspath(sourcedir)

class CMakeBuild(build_ext):
    def run(self):
        print("\n--- [AIMS Build Pipeline Starting] ---")
        try:
            subprocess.check_call(["cmake", "--version"], stdout=subprocess.DEVNULL)
        except (OSError, subprocess.CalledProcessError):
            raise RuntimeError("CMake must be installed and added to your system PATH to build aims_telemetry")

        self.build_temp = os.path.join(os.path.dirname(os.path.abspath(__file__)), "build_fresh")
        
        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        if not extdir.endswith(os.path.sep):
            extdir += os.path.sep

        cfg = os.environ.get("AIMS_BUILD_TYPE", "Release")
        print(f"[Build Profile] Activating target: {cfg}")

        current_src_dir = Path(__file__).resolve().parent
        extern_dir = current_src_dir / "extern"

        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}",
            f"-DPython_EXECUTABLE={sys.executable}",
            f"-DCMAKE_BUILD_TYPE={cfg}",
            f"-DEXTERN_SOURCE_DIR={extern_dir.as_posix()}"
        ]
        build_args = ["--config", cfg]

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        print(f"[CMake] Configuring workspace files in: {self.build_temp}")
        subprocess.check_call(["cmake", ext.sourcedir] + cmake_args, cwd=self.build_temp)
        
        print("[CMake] Building binary objects...")
        subprocess.check_call(["cmake", "--build", "."] + build_args, cwd=self.build_temp)
        print("--- [AIMS Build Pipeline Completed Successfully] ---\n")

setup(
    name="aims_telemetry",
    ext_modules=[CMakeExtension("aims_telemetry", sourcedir=".")],
    cmdclass=dict(build_ext=CMakeBuild),
    zip_safe=False,
)


# Remove-Item -Recurse -Force .\build_fresh -ErrorAction SilentlyContinue
# Remove-Item -Recurse -Force .\build -ErrorAction SilentlyContinue
# Remove-Item *.pyd -ErrorAction SilentlyContinue

# $env:AIMS_BUILD_TYPE="Release"

# python -B setup.py build_ext --inplace