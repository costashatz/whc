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
import corrade
import magnum
import magnum_integration
import magnum_plugins
import osqp
import pybind

def options(opt):
    opt.load('compiler_cxx')
    opt.load('compiler_c')
    opt.load('boost')
    opt.load('eigen')
    opt.load('dart')
    opt.load('robot_dart')
    opt.load('corrade')
    opt.load('magnum')
    opt.load('magnum_integration')
    opt.load('magnum_plugins')
    opt.load('osqp')
    opt.load('pybind')

    opt.add_option('--shared', action='store_true', help='build shared library', dest='build_shared')
    # opt.add_option('--tests', action='store_true', help='compile tests or not', dest='tests')
    opt.add_option('--python', action='store_true', help='compile python bindings', dest='pybind')


def configure(conf):
    conf.get_env()['BUILD_GRAPHIC'] = False

    conf.load('compiler_cxx')
    conf.load('compiler_c')
    conf.load('waf_unit_test')
    conf.load('boost')
    conf.load('eigen')
    conf.load('dart')
    conf.load('robot_dart')
    conf.load('avx')
    conf.load('corrade')
    conf.load('magnum')
    conf.load('magnum_integration')
    conf.load('magnum_plugins')
    conf.load('osqp')
    if conf.options.pybind:
        conf.load('python')
        conf.load('pybind')

    conf.check_boost(lib='regex system filesystem unit_test_framework', min_version='1.58')
    conf.check(features='cxx cxxprogram', lib=['pthread'], uselib_store='PTHREAD')
    conf.check_eigen(required=True)
    conf.check_dart(required=True)
    conf.check_robot_dart(required=False)
    conf.check_corrade(components='Utility PluginManager', required=False)
    conf.env['magnum_dep_libs'] = 'MeshTools Primitives Shaders SceneGraph GlfwApplication'
    if conf.env['DEST_OS'] == 'darwin':
        conf.env['magnum_dep_libs'] += ' WindowlessCglApplication'
    else:
        conf.env['magnum_dep_libs'] += ' WindowlessGlxApplication'
    conf.check_magnum(components=conf.env['magnum_dep_libs'], required=False)
    conf.check_magnum_plugins(components='AssimpImporter', required=False)
    conf.check_magnum_integration(components='Dart', required=False)
    conf.check_osqp(required=False)

    conf.env['py_flags'] = ''
    conf.env['BUILD_PYTHON'] = False
    if conf.options.pybind:
        conf.check_python_version((2, 7))
        conf.check_python_headers(features='pyext')
        conf.check_python_module('numpy')
        conf.check_python_module('dartpy')
        conf.check_pybind11(required=True)
        conf.env['BUILD_PYTHON'] = True
        if conf.env.CXX_NAME in ["gcc", "g++"]:
            conf.env['py_flags'] = ' -fPIC' # we need -fPIC in some Linux/gcc combinations

    if len(conf.env.INCLUDES_MagnumIntegration) > 0:
        conf.get_env()['BUILD_MAGNUM'] = True
        conf.env['magnum_libs'] = magnum.get_magnum_dependency_libs(conf, conf.env['magnum_dep_libs']) + magnum_integration.get_magnum_integration_dependency_libs(conf, 'Dart')

    avx_dart = conf.check_avx(lib='dart', required=['dart', 'dart-utils', 'dart-utils-urdf'])
    robot_dart_libs = ['RobotDARTSimu']
    if len(conf.env.INCLUDES_MagnumIntegration) > 0:
        robot_dart_libs.append('RobotDARTMagnum')
    avx_robot_dart = conf.check_avx(lib='robot_dart', required=robot_dart_libs, lib_type='static')
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

    all_flags = common_flags + conf.env['py_flags'] + opt_flags
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
    libs = 'PTHREAD BOOST EIGEN DART '

    defines = []
    if len(bld.env.INCLUDES_OSQP) > 0:
        libs = libs + 'OSQP '
        defines = ['USE_OSQP']

    bld.env['whc_libs'] = libs
    bld.env['whc_graphic_libs'] = bld.env['magnum_libs'] + ' ROBOT_DART_GRAPHIC'

    cxxflags = bld.get_env()['CXXFLAGS']

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

    bld.program(features = 'cxx ' + bld.env['lib_type'],
                source = whc_srcs + ' ' + qp_srcs,
                includes = './src ./src/external/qpOASES/include',
                uselib = libs,
                cxxflags = cxxflags + ['-DLINUX'],
                defines = defines,
                target = 'whc')

    bld.recurse('./src/examples')

    #### compilation of the Python3 bindings
    if bld.env['BUILD_PYTHON'] == True:
        # fix for native flags from pyext
        native_flags = ['-march=x86-64', '-mtune=generic']
        for flag in native_flags:
            if flag in bld.env['CXXFLAGS_PYEXT']:
                bld.env['CXXFLAGS_PYEXT'].remove(flag)
        for flag in bld.env['CXXFLAGS']:
            if flag in bld.env['CXXFLAGS_PYEXT']:
                bld.env['CXXFLAGS_PYEXT'].remove(flag)

        py_files = []
        for root, dirnames, filenames in os.walk(bld.path.abspath()+'/src/python/'):
            for filename in fnmatch.filter(filenames, '*.cpp'):
                ffile = os.path.join(root, filename)
                py_files.append(ffile)

        py_files = [f[len(bld.path.abspath())+1:] for f in py_files]
        py_srcs = " ".join(py_files)

        bld.program(features = 'c cshlib pyext',
                    source = './src/python/whc.cc ' + py_srcs,
                    includes = './src',
                    uselib = 'PYBIND11 ' + libs,
                    use = 'whc',
                    defines = defines,
                    target = 'pywhc')

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