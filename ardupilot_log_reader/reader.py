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

        ext = os.path.splitext(filename)[1]
        isbin = ext in ['.bin', '.BIN', '.px4log']
        islog = ext in ['.log', '.LOG'] # NOTE: "islog" does not mean a tlog
        istlog = ext in ['.tlog', '.TLOG']

        # Track types found
        available_types = set()

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

            # Grab the timestamp.
            timestamp = getattr(m, '_timestamp', 0.0)

            try:
                dfs_dicts[m.get_type()]
            except KeyError:
                dfs_dicts[m.get_type()] = {}
                dfs_dicts[m.get_type()]['timestamp'] = []
                for field in m.get_fieldnames():
                    dfs_dicts[m.get_type()][field] = []
            
            dfs_dicts[m.get_type()]['timestamp'].append( getattr(m,'_timestamp', 0.0) )
            for field in m.get_fieldnames():
                dfs_dicts[m.get_type()][field].append( getattr(m,field) )

        self._dfs = {}
        for msgType in dfs_dicts.keys():
            self._dfs[msgType] = pd.DataFrame(data=dfs_dicts[msgType])

            new_cols = []
            for val in self._dfs[msgType].columns:
                if val == 'timestamp':
                    new_cols.append(val)
                else:
                    new_cols.append(msgType + val)

            self._dfs[msgType].columns = new_cols

        mlog.filehandle.close()

    @property
    def dfs(self):
        return self._dfs
    
    @property
    def parms(self):
        if not self._parms:
            self._parms = self.dfs['PARM'].set_index('PARMName')['PARMValue'].to_dict()
        return self._parms

    def join_logs(self, titles):
        """Merge logs on timestamp 
        """
        available_titles = [title for title in titles if title in self.dfs.keys()]
        joined_log = self.dfs[available_titles[0]]
        for title in available_titles[1:]:
            if title=='PARM':
                continue
            ln= len(self.dfs[title])
            l0 = len(joined_log)
            joined_log = pd.merge_asof(
                joined_log, 
                self.dfs[title], 
                on='timestamp'
            )
            l1 = len(joined_log)
            if l1 < l0:
                pass
        return joined_log

    def full_df(self):
        return self.join_logs(list(self.dfs.keys())) # TODO remove PARA from here.