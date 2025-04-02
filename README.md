# Ardupilot Log Reader

This package is a bit like mavlogdump in pymavlink. It reads an ardupilot log to a pandas dataframe.

## usage

Set up script
```
from ardupilot_log_reader import Ardupilot
import sys

def read_bin_file(file_path):
    # Create a parser instance with specific message types we want to read
    types = ['ATT', 'BAT', 'AHR2',  'CTUN', 'CTRL', 'ESC', 'IMU', 'RCIN', 'VIBE']
    parser = Ardupilot.parse(file_path, types=types)
    
    # Print available message types and their columns
    print("\nAvailable message types and their columns:")
    for msg_type, df in parser.dfs.items():
        print(f"\n{msg_type} columns:")
        for col in df.columns:
            print(f"  - {col}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python test_log_reader.py <path_to_bin_file>")
        sys.exit(1)
    
    bin_file_path = sys.argv[1]
    read_bin_file(bin_file_path)
```

Run script:
```
python <script-name>.py <path-to-bin-file>
```
