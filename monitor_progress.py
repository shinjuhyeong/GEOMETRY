import time
import os

# Path to the .sta file
sta_file = "ThermalAnalysis.sta"

def read_progress():
    """Read the progress from the .sta file."""
    if not os.path.exists(sta_file):
        print(".sta file not found. Waiting for the simulation to start...")
        return None

    with open(sta_file, "r") as file:
        lines = file.readlines()

    # Look for the last line containing progress information
    for line in reversed(lines):
        if "STEP" in line and "INCREMENT" in line:
            return line.strip()

    return "Progress information not found."

def monitor():
    """Monitor the simulation progress."""
    print("Monitoring simulation progress. Press Ctrl+C to stop.")
    try:
        while True:
            progress = read_progress()
            if progress:
                print(progress)
            time.sleep(5)  # Check every 5 seconds
    except KeyboardInterrupt:
        print("Monitoring stopped.")

if __name__ == "__main__":
    monitor()
