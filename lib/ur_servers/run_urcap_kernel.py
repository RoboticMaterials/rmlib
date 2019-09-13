import sys
import os.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))

import rmlib
rm = rmlib.RMLib()
rm.run_xmlrpc()