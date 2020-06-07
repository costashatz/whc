#!/usr/bin/env python
# encoding: utf-8
#|
#|    Copyright (C) 2019 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
#|    Author/Maintainer:  Konstantinos Chatzilygeroudis
#|    email:   konstantinos.chatzilygeroudis@epfl.ch
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
Quick n dirty osqp detection
"""

import os
from waflib import Utils, Logs
from waflib.Configure import conf


def options(opt):
  opt.add_option('--osqp', type='string', help='path to osqp', dest='osqp')


@conf
def check_osqp(conf, *k, **kw):
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

    if conf.options.osqp:
        includes_check = [conf.options.osqp + '/include']
        libs_check = [conf.options.osqp + '/lib']

    try:
        conf.start_msg('Checking for OSQP includes' + msg)
        dirs = []
        dirs.append(get_directory('osqp/osqp.h', includes_check)+'/osqp')

        conf.end_msg(dirs)

        conf.start_msg('Checking for OSQP library' + msg)
        libs_ext = [lib_suffix] #, '.a']
        lib_found = False
        type_lib = lib_suffix #'.a'
        for lib in libs_ext:
            try:
                lib_dir = get_directory('libosqp.' + lib, libs_check)
                lib_found = True
                type_lib = lib
                break
            except:
                lib_found = False
        conf.end_msg('libosqp.' + type_lib)
        lib_dirs = [lib_dir]
        
        conf.start_msg('Checking for OsqpEigen includes' + msg)
        dirs.append(get_directory('OsqpEigen/OsqpEigen.h', includes_check))
        conf.end_msg(dirs[-1])

        conf.start_msg('Checking for OsqpEigen library' + msg)
        lib_dir = get_directory('libOsqpEigen.' + lib_suffix, libs_check)
        conf.end_msg('libOsqpEigen.' + lib_suffix)
        lib_dirs.append(lib_dir)

        # remove duplicates
        dirs = list(set(dirs))
        lib_dirs = list(set(lib_dirs))

        conf.env.INCLUDES_OSQP = dirs

        conf.env.LIBPATH_OSQP = lib_dirs
        if type_lib == '.a':
            conf.env.STLIB_OSQP = ['osqp']
            conf.env.LIB_OSQP = ['OsqpEigen']
        else:
            conf.env.LIB_OSQP = ['osqp', 'OsqpEigen']
    except:
        if required:
            conf.fatal('Not found')
        conf.end_msg('Not found', 'RED')
        return
    return 1