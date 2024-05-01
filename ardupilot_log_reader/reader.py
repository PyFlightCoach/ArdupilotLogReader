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

from __future__ import annotations
import fnmatch
import os
import pandas as pd
from  dataclasses import dataclass
from pymavlink.DFReader import DFReader_binary


@dataclass
class Ardupilot(object):
    filename: str
    dfs: dict[str, pd.DataFrame]

    def __getattr__(self, name):
        if name in self.dfs:
            return self.dfs[name]
        raise AttributeError(f"No such attribute: {name}")

    @staticmethod
    def process_patterns(available: list[str], patterns:list[str] = None, exclude_patterns: list[str]=None):
        def match_type(mtype, patterns):
            for p in patterns:
                if fnmatch.fnmatch(mtype, p):
                    return True
            return False
        patterns = available if patterns is None else patterns
        exclude_patterns = [] if exclude_patterns is None else exclude_patterns
        return [k for k in available if match_type(k, patterns) and not match_type(k, exclude_patterns)]

    @staticmethod
    def parse( 
            bin_file, types=None, nottypes=None, zero_time_base=False, 
            source_system=None, source_component=None, link=None, mav10=False
        ) -> Ardupilot:
        """
        Parses a binary file into an Ardupilot object.

        Parameters:
        bin_file (str): The binary file to parse.
        types (list[str], optional): List of types or patterns to include in the parsing. Defaults to None.
        nottypes (list[str], optional): List of types or patterns to exclude from the parsing. Defaults to None.
        zero_time_base (bool, optional): If True, sets the time base to zero. Defaults to False.
        source_system (int, optional): The source system ID to filter messages by. Defaults to None (all systems in log).
        source_component (int, optional): The source component ID to filter messages by. Defaults to None (all components in log).
        link (int, optional): The link to filter messages by. Defaults to None.
        mav10 (bool, optional): If True, uses MAVLink 1.0. Defaults to False.

        Returns:
        Ardupilot: The parsed Ardupilot object.
        """
        if not mav10:
            os.environ['MAVLINK20'] = '1'

        mlog: DFReader_binary = DFReader_binary(str(bin_file), zero_time_base=zero_time_base)
        
        match_types = Ardupilot.process_patterns(
            list(mlog.name_to_id.keys()), 
            list(set(types + ['PARM'])), 
            nottypes
        )

        log = Ardupilot._parse(mlog, match_types, source_system, source_component, link)
        
        mlog.filehandle.close()

        return log
    
    @staticmethod
    def _parse(mlog, cols: list[str], src_system=None, src_component=None, link=None):
        dfs_dicts = {}

        while True:
            m = mlog.recv_match(blocking=False, type=cols)
            if m is None:
                break
            if src_system is not None and src_system != m.get_srcSystem():
                continue
            if src_component is not None and src_component != m.get_srcComponent():
                continue
            if link is not None and link != m._link:
                continue

            key=m.get_type()
            
            if key not in dfs_dicts:
                if key == 'BAD_DATA':
                    continue          
                dfs_dicts[key] = {}
                dfs_dicts[key]['timestamp'] = []
                for field in m.get_fieldnames():
                    dfs_dicts[key][field] = []
            
            dfs_dicts[key]['timestamp'].append( getattr(m,'_timestamp', 0.0) )
            
            for field in m.get_fieldnames():
                dfs_dicts[key][field].append( getattr(m,field) )
        
        return Ardupilot(mlog.filehandle.name, {k: pd.DataFrame(v) for k, v in dfs_dicts.items()})

    def parameters(self) -> dict[str, pd.DataFrame]:
        gb = self.PARM.groupby('Name')

        parms = {}
        for gn in gb.groups.keys():
            gr = gb.get_group(gn)
            parms[gn] = gr.loc[abs(gr.Value.diff().fillna(1)) > 0, ["timestamp", "TimeUS", "Value"]].set_index('timestamp')
        return parms
        
    