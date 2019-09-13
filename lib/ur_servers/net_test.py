# net_test.py
# James Watson , 2019 Sptember
# Monitor a set of ports and log latency times

from rm_config import rm_config
from ur_server_utils import ensure_dir
import shlex
import time
import socket
import sys
import subprocess
import os
from subprocess import Popen, PIPE, STDOUT
from datetime import datetime

# ~~ Path Additions ~~
# URL, dir containing source file: http://stackoverflow.com/a/7783326
SOURCEDIR = os.path.dirname(os.path.abspath('__file__'))
PARENTDIR = os.path.dirname(SOURCEDIR)
# Might need this to fetch a lib in a parent directory
sys.path.insert(0, PARENTDIR)


_DEBUG = 0


def get_simple_cmd_output(cmd, stderr=STDOUT):
    """
    Execute a simple external command and get its output.
    """
    args = shlex.split(cmd)
    return Popen(args, stdout=PIPE, stderr=stderr).communicate()[0]


def get_ping_time(host):
    """ Return a tuple of ( <port open bool> , <ms latency float> ) """

    host, port = host.split(':')
    print("Attempt connection:", host, port)

    # 1. Check that the port is open
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        result = sock.connect_ex((host, int(port)))
        if result == 0:
            print("Port {}:\t\tOpen".format(port))
        else:
            print(result, "Port {}:\t\tClosed".format(port))
    except Exception as ex:
        print("Error:", ex)

    sock.close()

    # 2. Report latency on this port
    cmd = "fping {host} -C 3 -q".format(host=host)
    if _DEBUG:
        print("cmd:", "fping {host} -C 3 -q".format(host=host))
    msg = str(get_simple_cmd_output(cmd).strip()).split(':')[-1].split()
    if _DEBUG:
        print(type(msg), msg)
    res = [float(x) for x in [numStr.replace("'", "") for numStr in str(
        get_simple_cmd_output(cmd).strip()).split(':')[-1].split()] if x != '-']
    if len(res) > 0:
        return sum(res) / len(res)
    else:
        return 999999


interval_s = 1.0
duration_s = 60.0
logDir = "logs"
logPath = "logfile.txt"

_DEBUG = 1

if __name__ == '__main__':

    # 0. create the logging directory
    ensure_dir(logDir)

    # 1. Get a list of things to log

    # 2. Create a logfile for each entry

    t_bgn = time.time()
    print("Timing started at:", t_bgn)
    # 3. While logging is ongoing
    while 1:
        # 4. For each connection
        t_res = get_ping_time("127.0.0.1:4957")
        t_i = time.time()
        print("time:", t_i, ", Average response in [ms]:", t_res)
        time.sleep(1)
        if (t_i - t_bgn) > duration_s:
            break

        if _DEBUG:
            break
    print("Timing completed at:", t_i)
