#!/usr/bin/env python

'''
example program that dumps a Mavlink log file. The log file is
assumed to be in the format that qgroundcontrol uses, which consists
of a series of MAVLink packets, each with a 64 bit timestamp
header. The timestamp is in microseconds since 1970 (unix epoch)

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>.
'''

from __future__ import print_function

import array
import fnmatch
import json
import os
import struct
import sys
import time

import pandas as pd

try:
    from pymavlink.mavextra import *
except:
    print("WARNING: Numpy missing, mathematical notation will not be supported..")

from argparse import ArgumentParser

import inspect

from pymavlink import mavutil


class Ardupilot(object):

    def __init__(
        self, 
        bin_file, 
        no_timestamps=False,
        planner=False,
        robust=False,
        condition=None,
        types=None,
        nottypes=None,
        dialect="ardupilotmega",
        zero_time_base=False,
        source_system=None,
        source_component=None,
        link=None,
        mav10=False
        ):
        ''' 
            parser.add_argument("--no-timestamps", dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
            parser.add_argument("--planner", action='store_true', help="use planner file format")
            parser.add_argument("--robust", action='store_true', help="Enable robust parsing (skip over bad data)")
            parser.add_argument("--condition", default=None, help="select packets by condition")
            parser.add_argument("--types", default=None, help="types of messages (comma separated with wildcard)")
            parser.add_argument("--nottypes", default=None, help="types of messages not to include (comma separated with wildcard)")
            parser.add_argument("--dialect", default="ardupilotmega", help="MAVLink dialect")
            parser.add_argument("--zero-time-base", action='store_true', help="use Z time base for DF logs")
            parser.add_argument("--source-system", type=int, default=None, help="filter by source system ID")
            parser.add_argument("--source-component", type=int, default=None, help="filter by source component ID")
            parser.add_argument("--link", type=int, default=None, help="filter by comms link ID")
            parser.add_argument("--mav10", action='store_true', help="parse as MAVLink1")
            parser.add_argument("log", metavar="LOG")
        '''

        self._parms = None
        if not mav10:
            os.environ['MAVLINK20'] = '1'

        filename = bin_file
        mlog = mavutil.mavlink_connection(filename, planner_format=planner,
                                  notimestamps=no_timestamps,
                                  robust_parsing=robust,
                                  dialect=dialect,
                                  zero_time_base=zero_time_base)


        types = list(set(types + ['PARM']))

        if nottypes is not None:
            nottypes = nottypes.split(',')

        def match_type(mtype, patterns):
            '''return True if mtype matches pattern'''
            for p in patterns:
                if fnmatch.fnmatch(mtype, p):
                    return True
            return False

        # for DF logs pre-calculate types list
        match_types=None
        if types is not None and hasattr(mlog, 'name_to_id'):
            for k in mlog.name_to_id.keys():
                if match_type(k, types):
                    if nottypes is not None and match_type(k, nottypes):
                        continue
                    if match_types is None:
                        match_types = []
                    match_types.append(k)

        dfs_dicts = {}

        self.read_types = types

        while True:
            m = mlog.recv_match(blocking=False, type=match_types)
            if m is None:
                break

            if not mavutil.evaluate_condition(condition, mlog.messages):
                continue
            if source_system is not None and source_system != m.get_srcSystem():
                continue
            if source_component is not None and source_component != m.get_srcComponent():
                continue
            if link is not None and link != m._link:
                continue
            if types is not None and m.get_type() != 'BAD_DATA' and not match_type(m.get_type(), types):
                continue
            if nottypes is not None and match_type(m.get_type(), nottypes):
                continue

            # Ignore BAD_DATA messages is the user requested or if they're because of a bad prefix. The
            # latter case is normally because of a mismatched MAVLink version.
            if m.get_type() == 'BAD_DATA':
                continue

            key=m.get_type()

            if not key in dfs_dicts:
                dfs_dicts[key] = {}
                dfs_dicts[key]['timestamp'] = []
                for field in m.get_fieldnames():
                    dfs_dicts[key][field] = []
            
            dfs_dicts[key]['timestamp'].append( getattr(m,'_timestamp', 0.0) )
            
            for field in m.get_fieldnames():
                dfs_dicts[key][field].append( getattr(m,field) )
        
        mlog.filehandle.close()

        self.dfs = {}
        for k, v in dfs_dicts.items():
            self.dfs[k] = pd.DataFrame(v)
            #self.dfs[k].columns = list(v.keys())#[val if val == "timestamp" else k + val for val in v.keys()]        
        self.parms = self.dfs['PARM'].set_index('Name')['Value'].to_dict()

    def __getattr__(self, name):
        if name in self.dfs:
            return self.dfs[name]
        raise AttributeError(f"No such attribute: {name}")
    