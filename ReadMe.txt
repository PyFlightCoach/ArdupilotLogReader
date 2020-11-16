This package wraps untidily around mavlogdump in pymavlink, to read an ardupilot log to a pandas dataframe.

build:
conda config --append channels thomasdavid
conda build .


usage:
#set up the parser
from ArdupilotLogReader import Ardupilot
parser = Ardupilot(
    log_file, # the log file, .bin
    types = ['ARSP', 'ATT', 'BARO', 'GPS', 'IMU', 'RCIN', 'RCOU', 'BAT', 'MODE', 'NKF1', 'STAT', 'XKF1'],  # fields to read from the log
    zero_time_base=True # arguments passed to pymavlink
    )

parser.dfs # a dict containing a dataframe of log data for each field requested.
parser.join_logs(['ARSP', 'ATT']) #returns a pandas dataframe containing the ARSP and ATT data
parser.parms # returns the parameters read from the top of the log

