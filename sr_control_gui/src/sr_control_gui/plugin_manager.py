#!/usr/bin/env python
#
# Copyright 2011 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
#

import roslib; roslib.load_manifest('sr_control_gui')
import os, sys, imp

class Plugin(object):
    """
    A Plugin.
    """
    #plugin name
    plugin_name = None
    category = "Default"
    class_name = None
    parent = None
    description = None
    # the dependencies are loaded in the children
    dependencies = None

    def __init__( self ):
        """
        Reads the given file, init the plugin and return its category.
        """

    def get_category(self):
        return self.category

    def set_parent(self, parent):
        self.parent = parent

    def activate(self):
        print "Virtual Function, must be implemented in your plugin"

    def depends(self):
        return self.dependencies

class PluginManager(object):
    """
    Loads the available plugins.
    """
    # dict containing the categories and their associated plugins
    dict_plugins = None
    # list of paths to the plugins
    list_of_paths = []

    def __init__(self):
        """
        """
        self.dict_plugins = {}

    def setPluginPlaces(self, paths):
        """
        Set the list of paths to the directories containing the plugins.
        """
        for path in paths:
            if path[-1] != "/":
                path += "/"
            self.list_of_paths.append(path)

    def loadPlugins(self):
        """
        Load the plugins

        @return the number of plugins found.
        """
        if self.list_of_paths == []:
            print "No paths defined. Aborting."
            return -1

        nb_plugins_found = 0
        for path in self.list_of_paths:
            #add the path to the sys path in order to be able to load the modules
            sys.path.append(path)

            dirList=os.listdir(path)
            for fname in dirList:
                if fname.split(".")[-1] == "sr_plugin":
                    plugin_name, category, class_name, description = self.parse_file( path + fname )
                    plugin_tmp = self.load_plugin(plugin_name, class_name, description, category)

                    if not self.dict_plugins.has_key(category):
                        self.dict_plugins[category] = []

                    self.dict_plugins[category].append(plugin_tmp)
                    nb_plugins_found += 1

        return nb_plugins_found

    def getPluginsOfCategory(self, category_names):
        """
        Return a list of the plugins in the given categories.
        """
        plugins_to_return = []
        for name in category_names:
            tmp_plugins = self.dict_plugins.get(name)
            if tmp_plugins != None:
                plugins_to_return += tmp_plugins
            else:
                print "WARNING: couldn't find a category named ",name

        return plugins_to_return


    def parse_file(self, full_path_to_plugin_file):
        plugin_name = ""
        category = "Default"
        class_name = ""
        description = ""

        file_tmp = open(full_path_to_plugin_file)
        for line in file_tmp.readlines():
            line = line.strip()
            if "=" not in line:
                continue
            splitted_line = line.split(" = ")
            if splitted_line[0] == "Name":
                plugin_name = splitted_line[1]
            elif splitted_line[0] == "Category":
                category = splitted_line[1]
            elif splitted_line[0] == "Module":
                class_name = splitted_line[1]
            elif splitted_line[0] == "Description":
                description = splitted_line[1]
        file_tmp.close()

        return plugin_name, category, class_name, description

    def load_plugin(self, plugin_name, class_name, description, category):
        plugin_tmp = None
        if class_name != None:
            #import the module
            #try:
            exec "from %s import %s" % (plugin_name, class_name)
            class_tmp_str = class_name + "()"
            plugin_tmp = eval(class_tmp_str)

            plugin_tmp.plugin_name = plugin_name
            plugin_tmp.class_name = class_name
            plugin_tmp.description = description
            plugin_tmp.category = category
            #except:
            #    print "[", plugin_name, "]: not imported: ", sys.exc_info()[0]

        else:
            print "[",plugin_name ,"]", "The Module name was not defined in the config file. Aborting."
        return plugin_tmp


if __name__ == "__main__":
    pm = PluginManager()
    pm.setPluginPlaces(["/home/ugo/Projects/ROS_interfaces/sr-ros-interface/trunk/shadow_robot/sr_control_gui/src/sr_control_gui/plugins"])
    pm.loadPlugins()
