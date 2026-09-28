import subprocess
import sys

def run_command(command):
    print(f"============================================================")
    print(f"Running: {command}")
    print(f"============================================================")
    result = subprocess.run(command, shell=True)
    if result.returncode != 0:
        print(f"\n[ERROR] Command failed with exit code {result.returncode}: {command}")
        sys.exit(result.returncode)
    print("\n[SUCCESS] Command completed successfully.\n")

def main():
    # Commands extracted from WALKTHROUGH.md under "Run the demonstration" and "Run checks"
    commands = [
        # Run all five scenarios, save dashboard metrics and a final-route PNG for each
        f'"{sys.executable}" -m sih_sim.cli --scenario all --device auto --video --output outputs',
        
        # Force CUDA and fail early if it cannot be used
        f'"{sys.executable}" -m sih_sim.cli --scenario market --device cuda --video',
        
        # Run village scenario with CUDA
        f'"{sys.executable}" -m sih_sim.cli --scenario village --device cuda --video',
        
        # Run checks
        f'"{sys.executable}" -m pytest -q'
    ]

    for cmd in commands:
        run_command(cmd)

if __name__ == "__main__":
    main()
