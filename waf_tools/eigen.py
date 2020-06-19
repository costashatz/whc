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
#! /usr/bin/env python
# JB Mouret - 2009
# Federico Allocati - 2015
# Konstantinos Chatzilygeroudis - 2016, 2017, 2018

"""
Quick n dirty eigen3 detection
"""

import os, glob, types
from waflib.Configure import conf


def options(opt):
    opt.add_option('--eigen', type='string', help='path to eigen', dest='eigen')
    # to-do: rename this to --eigen_lapacke
    opt.add_option('--lapacke_blas', action='store_true', help='enable lapacke/blas if found (required Eigen>=3.3)', dest='lapacke_blas')


@conf
def check_eigen(conf, *k, **kw):
    def get_directory(filename, dirs):
        res = conf.find_file(filename, dirs)
        return res[:-len(filename)-1]
    includes_check = ['/usr/include/eigen3', '/usr/local/include/eigen3', '/usr/include', '/usr/local/include']

    required = kw.get('required', False)

    # OSX/Mac uses .dylib and GNU/Linux .so
    suffix = 'dylib' if conf.env['DEST_OS'] == 'darwin' else 'so'

    if conf.options.eigen:
        includes_check = [conf.options.eigen]

    try:
        conf.start_msg('Checking for Eigen')
        incl = get_directory('Eigen/Core', includes_check)
        conf.env.INCLUDES_EIGEN = [incl]
        conf.end_msg(incl)
        if conf.options.lapacke_blas:
            conf.start_msg('Checking for LAPACKE/BLAS (optional)')
            world_version = -1
            major_version = -1
            minor_version = -1

            config_file = conf.find_file('Eigen/src/Core/util/Macros.h', includes_check)
            with open(config_file) as f:
                config_content = f.readlines()
            for line in config_content:
                world = line.find('#define EIGEN_WORLD_VERSION')
                major = line.find('#define EIGEN_MAJOR_VERSION')
                minor = line.find('#define EIGEN_MINOR_VERSION')
                if world > -1:
                    world_version = int(line.split(' ')[-1].strip())
                if major > -1:
                    major_version = int(line.split(' ')[-1].strip())
                if minor > -1:
                    minor_version = int(line.split(' ')[-1].strip())
                if world_version > 0 and major_version > 0 and minor_version > 0:
                    break

            if world_version == 3 and major_version >= 3:
                # Check for lapacke and blas
                extra_libs = ['/usr/lib', '/usr/local/lib', '/usr/local/opt/openblas/lib']
                blas_libs = ['blas', 'openblas']
                blas_lib = ''
                blas_path = ''
                for b in blas_libs:
                    try:
                        blas_path = get_directory('lib'+b+'.'+suffix, extra_libs)
                    except:
                        continue
                    blas_lib = b
                    break

                lapacke = False
                lapacke_path = ''
                try:
                    lapacke_path = get_directory('liblapacke.'+suffix, extra_libs)
                    lapacke = True
                except:
                    lapacke = False

                if lapacke or blas_lib != '':
                    conf.env.DEFINES_EIGEN = []
                    if lapacke_path != blas_path:
                        conf.env.LIBPATH_EIGEN = [lapacke_path, blas_path]
                    else:
                        conf.env.LIBPATH_EIGEN = [lapacke_path]
                    conf.env.LIB_EIGEN = []
                    conf.end_msg('LAPACKE: \'%s\', BLAS: \'%s\'' % (lapacke_path, blas_path))
                elif lapacke:
                    conf.end_msg('Found only LAPACKE: %s' % lapacke_path, 'YELLOW')
                elif blas_lib != '':
                    conf.end_msg('Found only BLAS: %s' % blas_path, 'YELLOW')
                else:
                    conf.end_msg('Not found in %s' % str(extra_libs), 'RED')
                if lapacke:
                    conf.env.DEFINES_EIGEN.append('EIGEN_USE_LAPACKE')
                    conf.env.LIB_EIGEN.append('lapacke')
                if blas_lib != '':
                    conf.env.DEFINES_EIGEN.append('EIGEN_USE_BLAS')
                    conf.env.LIB_EIGEN.append(blas_lib)
            else:
                conf.end_msg('Found Eigen version %s: LAPACKE/BLAS can be used only with Eigen>=3.3' % (str(world_version) + '.' + str(major_version) + '.' + str(minor_version)), 'RED')
    except:
        if required:
            conf.fatal('Not found in %s' % str(includes_check))
        conf.end_msg('Not found in %s' % str(includes_check), 'RED')
    return 1
