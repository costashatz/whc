#!/usr/bin/env python
# encoding: utf-8
#|
#|    Copyright (C) 2019 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
#|    Author/Maintainer:  Konstantinos Chatzilygeroudis
#|    email:   costashatz@gmail.com
#|    website: lasa.epfl.ch
#|
#|    This file is part of whc.
#|
#|    whc is free software: you can redistribute it and/or modify
#|    it under the terms of the GNU General Public License as published by
#|    the Free Software Foundation, either version 3 of the License, or
#|    (at your option) any later version.
#|
#|    whc is distributed in the hope that it will be useful,
#|    but WITHOUT ANY WARRANTY; without even the implied warranty of
#|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#|    GNU General Public License for more details.
#|
"""
Quick n dirty robot_dart detection
"""

import os
from waflib import Utils, Logs
from waflib.Configure import conf


def options(opt):
  opt.add_option('--robot_dart', type='string', help='path to robot_dart', dest='robot_dart')


@conf
def check_robot_dart(conf, *k, **kw):
    def get_directory(filename, dirs):
        res = conf.find_file(filename, dirs)
        return res[:-len(filename)-1]

    required = kw.get('required', False)

    msg = ''
    if not required:
        msg = ' [optional]'

    includes_check = ['/usr/local/include', '/usr/include']
    libs_check = ['/usr/local/lib', '/usr/lib']

    # OSX/Mac uses .dylib and GNU/Linux .so
    lib_suffix = 'dylib' if conf.env['DEST_OS'] == 'darwin' else 'so'

    # # You can customize where you want to check
    # # e.g. here we search also in a folder defined by an environmental variable
    # if 'RESIBOTS_DIR' in os.environ:
    # 	includes_check = [os.environ['RESIBOTS_DIR'] + '/include'] + includes_check
    # 	libs_check = [os.environ['RESIBOTS_DIR'] + '/lib'] + libs_check

    if conf.options.robot_dart:
        includes_check = [conf.options.robot_dart + '/include']
        libs_check = [conf.options.robot_dart + '/lib']

    try:
        conf.start_msg('Checking for robot_dart includes' + msg)
        dirs = []
        dirs.append(get_directory('robot_dart/robot.hpp', includes_check))
        dirs.append(get_directory('robot_dart/control/robot_control.hpp', includes_check))
        dirs.append(get_directory('robot_dart/robot_dart_simu.hpp', includes_check))

        # remove duplicates
        dirs = list(set(dirs))

        conf.end_msg(dirs)
        conf.env.INCLUDES_ROBOT_DART = dirs

        conf.start_msg('Checking for robot_dart library' + msg)
        libs_ext = ['.a', lib_suffix]
        lib_found = False
        type_lib = '.a'
        for lib in libs_ext:
            try:
                lib_dir = get_directory('libRobotDARTSimu' + lib, libs_check)
                lib_found = True
                type_lib = lib
                break
            except:
                lib_found = False
        conf.end_msg('libRobotDARTSimu' + type_lib)

        conf.env.LIBPATH_ROBOT_DART = lib_dir
        if type_lib == '.a':
            conf.env.STLIB_ROBOT_DART = ['RobotDARTSimu']
        else:
            conf.env.LIB_ROBOT_DART = ['RobotDARTSimu']

        # Check for GUI library
        conf.start_msg('Checking for robot_dart GUI includes' + msg)
        dirs = []
        dirs.append(get_directory('robot_dart/gui/magnum/base_application.hpp', includes_check))

        # remove duplicates
        dirs = list(set(dirs))

        conf.end_msg(dirs)
        conf.env.INCLUDES_ROBOT_DART_GRAPHIC = dirs

        conf.start_msg('Checking for robot_dart GUI library' + msg)
        libs_ext = ['.a', lib_suffix]
        lib_found = False
        type_lib = '.a'
        for lib in libs_ext:
            try:
                lib_dir = get_directory('libRobotDARTMagnum' + lib, libs_check)
                lib_found = True
                type_lib = lib
                break
            except:
                lib_found = False
        conf.end_msg('libRobotDARTMagnum' + type_lib)

        conf.env.LIBPATH_ROBOT_DART_GRAPHIC = lib_dir
        if type_lib == '.a':
            conf.env.STLIB_ROBOT_DART_GRAPHIC = 'RobotDARTMagnum'
        else:
            conf.env.LIB_ROBOT_DART_GRAPHIC = 'RobotDARTMagnum'
    except:
        if required:
            conf.fatal('Not found')
        conf.end_msg('Not found', 'RED')
        return
    return 1