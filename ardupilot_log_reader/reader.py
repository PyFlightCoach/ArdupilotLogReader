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
            
            key = f"{m.get_type()}_{str(m.C)}" if hasattr(m, "C") else m.get_type()

            if not key in dfs_dicts:
                dfs_dicts[key] = {}
                dfs_dicts[key]['timestamp'] = []
                for field in m.get_fieldnames():
                    dfs_dicts[key][field] = []
            
            dfs_dicts[key]['timestamp'].append( getattr(m,'_timestamp', 0.0) )
            for field in m.get_fieldnames():
                dfs_dicts[key][field].append( getattr(m,field) )
        
        mlog.filehandle.close()

        self._dfs = {}
        self.dfs = {}
        for k, v in dfs_dicts.items():
            self._dfs[k] = pd.DataFrame(v)
            core, name = Ardupilot._get_core(k)
            self._dfs[k].columns =[val if val == "timestamp" else name + val for val in v.keys()]
            #for back compatibility
            if not core:
                self.dfs[k.split("_")[0]] = self._dfs[k]
        
        self.parms = self.dfs['PARM'].set_index('PARMName')['PARMValue'].to_dict()


    @staticmethod
    def _get_core(k):
        """returns the core if it exists, otherwise None"""
        try:
            spl = k.split("_")
            assert len(spl) == 2
            return int(spl[1]), spl[0]
        except Exception as e:
            return None, k

    def __getattr__(self, name):
        if name in self.dfs:
            return self.dfs[name]
        elif name in self._dfs:
            return self._dfs[name]

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
        dfnames = list(self.dfs.keys())
        dfnames.remove("PARM")
        dfnames = [dfn for dfn in dfnames if Ardupilot._get_core(dfn) is None]
        return self.join_logs(dfnames)