#!/usr/bin/env python
# encoding: utf-8
import sys
import os
import fnmatch
import glob
sys.path.insert(0, sys.path[0]+'/waf_tools')

VERSION = '1.0.0'
APPNAME = 'whc'

srcdir = '.'
blddir = 'build'

from waflib.Build import BuildContext
from waflib import Logs
from waflib.Tools import waf_unit_test
import dart
import boost
import eigen
import robot_dart
import avx

def options(opt):
    opt.load('compiler_fc')
    opt.load('compiler_cxx')
    opt.load('compiler_c')
    opt.load('boost')
    opt.load('eigen')
    opt.load('dart')
    opt.load('robot_dart')

    opt.add_option('--shared', action='store_true', help='build shared library', dest='build_shared')
    # opt.add_option('--tests', action='store_true', help='compile tests or not', dest='tests')


def configure(conf):
    conf.get_env()['BUILD_GRAPHIC'] = False

    conf.load('compiler_fc')
    conf.load('compiler_cxx')
    conf.load('compiler_c')
    conf.load('waf_unit_test')
    conf.load('boost')
    conf.load('eigen')
    conf.load('dart')
    conf.load('robot_dart')
    conf.load('avx')

    conf.check_boost(lib='regex system filesystem unit_test_framework', min_version='1.46')
    conf.check_eigen(required=True)
    conf.check_dart(required=True)
    conf.check_robot_dart(required=True)

    avx_dart = conf.check_avx(lib='dart', required=['dart', 'dart-utils', 'dart-utils-urdf'])
    avx_robot_dart = conf.check_avx(lib='robot_dart', required=['RobotDARTSimu'], lib_type='static')
    native = ''
    native_icc = ''
    if avx_dart and avx_robot_dart:
        conf.msg('-march=native (AVX support)', 'yes', color='GREEN')
        native = '-march=native'
        native_icc = 'mtune=native'
    else:
        if avx_dart or avx_robot_dart:
            conf.msg('-march=native (AVX support)', 'no (optional) --- some libraries are compiled with avx and others not; your programs might not run!', color='RED')
        else:
            conf.msg('-march=native (AVX support)', 'no (optional)', color='YELLOW')


    conf.env['lib_type'] = 'cxxstlib'
    if conf.options.build_shared:
        conf.env['lib_type'] = 'cxxshlib'

    if conf.env.CXX_NAME in ["icc", "icpc"]:
        common_flags = "-Wall -std=c++14"
        opt_flags = " -O3 -xHost -unroll -g " +  native_icc
    elif conf.env.CXX_NAME in ["clang"]:
        common_flags = "-Wall -std=c++14"
        opt_flags = " -O3 -g -faligned-new " + native
    else:
        gcc_version = int(conf.env['CC_VERSION'][0]+conf.env['CC_VERSION'][1])
        if gcc_version < 47:
            conf.fatal('You need gcc version >= 4.7 for this project.')
        else:
            common_flags = "-Wall -std=c++14"
        opt_flags = " -O3 -g " + native
        if gcc_version >= 71:
            opt_flags = opt_flags + " -faligned-new"

    all_flags = common_flags + opt_flags + ' -fPIC'
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
    bld.env.LIB_GFORTRAN = ['gfortran']
    libs = 'BOOST EIGEN DART ROBOT_DART GFORTRAN '
    libs_graphics = libs + ' DART_GRAPHIC'

    bld.env['whc_libs'] = libs
    bld.env['whc_graphic_libs'] = libs_graphics

    cxxflags = bld.get_env()['CXXFLAGS']

    # build qld fortran
    bld.env.append_unique('FCFLAGS', '-O3')

    bld.program(features = 'fc fcstlib',
                source = './src/external/qld/qld.f',
                includes = './src/external/qld',
                fcflags = ['-fPIC'],
                target = 'fqld')
    
    bld.program(features = 'cxx cxxstlib',
                source = './src/external/qld/qld.cpp',
                includes = './src/external/qld',
                uselib = 'EIGEN',
                use = 'fqld',
                cxxflags = cxxflags,
                target = 'qld')

    files = []
    for root, dirnames, filenames in os.walk(bld.path.abspath()+'/src/whc'):
        for filename in fnmatch.filter(filenames, '*.cpp'):
            files.append(os.path.join(root, filename))

    files = [f[len(bld.path.abspath())+1:] for f in files]
    whc_srcs = " ".join(files)

    # build qpOASES
    qp_files = []
    for root, dirnames, filenames in os.walk(bld.path.abspath()+'/src/external/qpOASES/src/'):
        for filename in fnmatch.filter(filenames, '*.cpp'):
            qp_files.append(os.path.join(root, filename))

    qp_files = [f[len(bld.path.abspath())+1:] for f in qp_files]
    qp_srcs = " ".join(qp_files)

    lib_name = 'whc'
    if bld.env['lib_type'] == 'cxxstlib':
        lib_name = 'whc_thin'
    bld.env['lib_name'] = lib_name

    bld.program(features = 'cxx ' + bld.env['lib_type'],
                source = whc_srcs + ' ' + qp_srcs,
                includes = './src ./src/external/qpOASES/include ./src/external/qld',
                uselib = libs,
                use = 'fqld qld',
                cxxflags = cxxflags,
                target = lib_name)

    if bld.env['lib_type'] == 'cxxstlib':
        bld(rule='ar -rcT ${TGT} ${SRC}', source='libfqld.a libqld.a lib' + lib_name + '.a', target='libwhc.a', use=lib_name)

    bld.recurse('./src/examples')

    bld.add_post_fun(summary)

    install_files = []
    for root, dirnames, filenames in os.walk(bld.path.abspath()+'/src/whc/'):
        for filename in fnmatch.filter(filenames, '*.hpp'):
            install_files.append(os.path.join(root, filename))
    install_files = [f[len(bld.path.abspath())+1:] for f in install_files]

    for f in install_files:
        end_index = f.rfind('/')
        if end_index == -1:
            end_index = len(f)
        bld.install_files('${PREFIX}/include/' + f[4:end_index], f)
    if bld.env['lib_type'] == 'cxxstlib':
        bld.install_files('${PREFIX}/lib', blddir + '/libwhc.a')
    else:
        bld.install_files('${PREFIX}/lib', blddir + '/libwhc.so')
    bld.install_files('${PREFIX}/lib', blddir + '/libfqld.a')
    bld.install_files('${PREFIX}/lib', blddir + '/libqld.a')