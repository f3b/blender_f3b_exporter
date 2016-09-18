# This file is part of blender_io_f3b.  blender_io_f3b is free software: you can
# redistribute it and/or modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation, version 2.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc., 51
# Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#
# Copyright David Bernard

# <pep8 compliant>


bl_info = {
    "name": "F3b Exporter",
    "author": "Riccardo Balbo, based on xbuf exporter by David Bernard",
    "version": (0, 3,2),
    "blender": (2, 73, 0),
    # "location": "Render > Engine > F3b Render",
    "description": "f3b exporter",
    "wiki_url": "",
    "tracker_url": "",
    "category": "Import-Export"}

import sys
import os

#Load dependencies
_modules_path = os.path.join(os.path.dirname(__file__), "libs")
for path in os.listdir(_modules_path):
    p=os.path.join(_modules_path,path)
    print("Load library ",p)
    sys.path.append(p)
del _modules_path
print(sys.path)

# Use F8 to reload (see http://wiki.blender.org/index.php/Dev:2.5/Py/Scripts/Cookbook/Code_snippets/Multi-File_packages)
from . import helpers
from . import f3b_export

"""If the module is reloaded, reload all submodules as well
   This will reload all modules at the initial import as well but
   that should not be a problem
"""
import imp
import types
locals_copy = dict(locals())
for var in locals_copy:
    tmp = locals_copy[var]
    if isinstance(tmp, types.ModuleType) and tmp.__package__ == __name__:
        # print("Reloading: %s" % (var))
        imp.reload(tmp)

import bpy

def menu_func_exporter(self, context):
    self.layout.operator(f3b_export.f3bExporter.bl_idname, text="f3b (.f3b)")


def register_exporter():
    bpy.utils.register_class(f3b_export.f3bExporter)
    bpy.types.INFO_MT_file_export.append(menu_func_exporter)


def unregister_exporter():
    bpy.types.INFO_MT_file_export.remove(menu_func_exporter)
    bpy.utils.unregister_class(f3b_export.f3bExporter)


def register():
    register_exporter()


def unregister():
    unregister_exporter()

def main():
    try:
        unregister()
    except (RuntimeError, ValueError):
        pass
    register()


# This allows you to run the script directly from blenders text editor
# to test the addon without having to install it.
if __name__ == "__main__":
    main()
