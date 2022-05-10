import os, sys

from distutils.core import setup, Extension
from distutils import sysconfig

cpp_args = ['-std=c++11'] #, '-stdlib=libc++', '-mmacosx-version-min=10.7']

from pathlib import Path

src_path = Path("../octomap/src")

def explore_more(dir : str):
    return_file_list = []
    for root, folders, files in os.walk(dir):
        if len(folders) != 0:
            for f in folders:
                new_dir = Path(root).joinpath(f)
                return_file_list.extend(explore_more(new_dir.as_posix()))
        else:
            dir_p = Path(root)
            for f in files:
                f_name = dir_p.joinpath(f)
                if f_name.suffix == "cpp":
                    return_file_list.append(f_name.as_posix())
    
    return return_file_list

files = explore_more("../octomap/src")
files.append('octomath.cpp')

print(files)

ext_modules = [
    Extension(
    'Octomap',
        # files,
        ['octomath.cpp', "../octomap/src/math/Vector3.cpp"],
        # ['funcs.cpp', 'wrap.cpp'],
        include_dirs=['pybind11/include', "../octomap/include"],
    language='c++',
    extra_compile_args = cpp_args,
    ),
]

setup(
    name='Octomap',
    version='0.0.1',
    author='rxu',
    author_email='ruoyangx@gmail.com',
    description='Example',
    ext_modules=ext_modules,
)
