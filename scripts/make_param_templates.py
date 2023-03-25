#!/usr/bin/env python

import yaml
import sys
import os

def run():
    if len(sys.argv) != 2:
        print("Usage: {} PACKAGE_CONFIG.yaml".format(__file__))
        sys.exit(1)
    config_file = sys.argv[1]
    with open(config_file, 'r') as file:
        d = yaml.safe_load(file)
        for line in d["config"]:
            print(line)
            name = line[2]
            param = os.path.join(os.path.dirname(config_file),name + ".h")
            with open(param, "w") as param_file:
                param_file.write("#pragma once\n\n")
                param_file.write(".name = \"{}\",\n".format(name))

if __name__ == "__main__":
    run()
