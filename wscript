#!/usr/bin/env python
# encoding: utf-8
import sys
import os
import fnmatch
import glob
sys.path.insert(0, sys.path[0]+'/waf_tools')

VERSION = '1.0.0'
APPNAME = 'robot_dart'

srcdir = '.'
blddir = 'build'

from waflib.Build import BuildContext
from waflib import Logs
from waflib.Tools import waf_unit_test
import dart
import boost
import eigen
import robot_dart


def options(opt):
    opt.load('compiler_cxx')
    opt.load('compiler_c')
    opt.load('boost')
    opt.load('eigen')
    opt.load('dart')
    opt.load('robot_dart')

    # opt.add_option('--shared', action='store_true', help='build shared library', dest='build_shared')
    # opt.add_option('--tests', action='store_true', help='compile tests or not', dest='tests')


def configure(conf):
    conf.get_env()['BUILD_GRAPHIC'] = False

    conf.load('compiler_cxx')
    conf.load('compiler_c')
    conf.load('waf_unit_test')
    conf.load('boost')
    conf.load('eigen')
    conf.load('dart')
    conf.load('robot_dart')

    conf.check_boost(lib='regex system filesystem unit_test_framework', min_version='1.46')
    conf.check_eigen(required=True)
    conf.check_dart(required=True)
    conf.check_robot_dart(required=True)

    # conf.env['lib_type'] = 'cxxstlib'
    # if conf.options.build_shared:
    #     conf.env['lib_type'] = 'cxxshlib'

    if conf.env.CXX_NAME in ["icc", "icpc"]:
        common_flags = "-Wall -std=c++14"
        opt_flags = " -O3 -xHost -mtune=native -unroll -g"
    elif conf.env.CXX_NAME in ["clang"]:
        common_flags = "-Wall -std=c++14"
        opt_flags = " -O3 -march=native -g -faligned-new"
    else:
        gcc_version = int(conf.env['CC_VERSION'][0]+conf.env['CC_VERSION'][1])
        if gcc_version < 47:
            conf.fatal('You need gcc version >= 4.7 for this project.')
        else:
            common_flags = "-Wall -std=c++14"
        opt_flags = " -O3 -march=native -g"
        if gcc_version >= 71:
            opt_flags = opt_flags + " -faligned-new"

    all_flags = common_flags + opt_flags
    conf.env['CXXFLAGS'] = conf.env['CXXFLAGS'] + all_flags.split(' ')
    print(conf.env['CXXFLAGS'])

def summary(bld):
    lst = getattr(bld, 'utest_results', [])
    total = 0
    tfail = 0
    if lst:
        total = len(lst)
        tfail = len([x for x in lst if x[1]])
    waf_unit_test.summary(bld)
    if tfail > 0:
        bld.fatal("Build failed, because some tests failed!")

def build(bld):
    libs = 'BOOST EIGEN DART ROBOT_DART '
    libs_graphics = libs + ' DART_GRAPHIC'

    bld.env['whc_libs'] = libs
    bld.env['whc_graphic_libs'] = libs_graphics

    cxxflags = bld.get_env()['CXXFLAGS']

    files = []
    for root, dirnames, filenames in os.walk(bld.path.abspath()+'/src/whc'):
        for filename in fnmatch.filter(filenames, '*.cpp'):
            files.append(os.path.join(root, filename))

    files = [f[len(bld.path.abspath())+1:] for f in files]
    whc_srcs = " ".join(files)

    # build qpOASES
    qp_files = []
    for root, dirnames, filenames in os.walk(bld.path.abspath()+'/external/qpOASES/src/'):
        for filename in fnmatch.filter(filenames, '*.cpp'):
            qp_files.append(os.path.join(root, filename))

    qp_files = [f[len(bld.path.abspath())+1:] for f in qp_files]
    qp_srcs = " ".join(qp_files)

    bld.program(features = 'cxx cxxstlib',
                install_path = None,
                source = qp_srcs,
                includes = './external/qpOASES/include',
                cxxflags = cxxflags + ['-DLINUX'],
                target = 'qpoases')

    bld.program(features = 'cxx cxxstlib',
                install_path = None,
                source = whc_srcs,
                includes = './src ./external/qpOASES/include',
                uselib = libs,
                use = 'qpoases',
                cxxflags = cxxflags,
                target = 'whc')
    
    bld.recurse('./src/examples')

    bld.add_post_fun(summary)