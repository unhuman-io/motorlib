#!/usr/bin/env python

import subprocess
import yaml
import sys
import os
import shutil

def run():
    if len(sys.argv) != 2:
        print("Usage: {} PACKAGE_CONFIG.yaml".format(__file__))
        sys.exit(1)
    config_file = sys.argv[1]
    os.mkdir("package")
    with open(config_file, 'r') as file:
        d = yaml.safe_load(file)
        for line in d["config"]:
            # for example:
            # config:
            #  - ["207539635356", "motor_aksim", "J1", "R4"]
            print(line)
            sn = line[0]
            typ = line[1]
            name = line[2]
            param = os.path.join(os.path.dirname(config_file),name + ".h")
            c_defs = line[3]
            subprocess.call(["make", "-j", "CONFIG=" + typ, "C_DEFS=-D"+c_defs + " -DPARAM_OVERRIDES="+param])
            
            os.rename(os.path.join("build", typ), os.path.join("package", typ + "-" + name + "-" + sn))
    shutil.copy(os.path.join(os.path.dirname(__file__), "load.sh"), "package")

            

if __name__ == "__main__":
    run()
