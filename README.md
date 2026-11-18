# TrackingAntennaSystem
Repository will store all resources for the tracking antenna system which will dynamically point at the rocket mid-flight.

## Run the Python simulations

Follow the [step-by-step simulation instructions](OldCode/README.md#start-here-run-the-current-simulation-yourself).
They cover PowerShell setup, both mount-geometry studies, selected-design runs,
plots, peak motor velocity/acceleration, and regression tests.

From PowerShell on this checkout:

```powershell
cd "C:\Users\mattm\Documents\GitHub\TrackingAntennaSystem\OldCode"
& ".\.venv\Scripts\python.exe" -B mount_study.py
```

The current complete workflow saves results to `OldCode/results/mount-study`.
See the linked guide if the Python environment is not installed yet. Simulation
run instructions are maintained with each simulation update.
