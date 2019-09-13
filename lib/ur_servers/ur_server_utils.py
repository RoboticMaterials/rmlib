
import datetime , os

# FIXME: Problems with TCP
# https://docs.python.org/3/library/xmlrpc.client.html#protocolerror-objects

dayTimeStamp = lambda: datetime.datetime.now().strftime('%Y-%m-%d') 
""" Return a formatted timestamp string, useful for logging and debugging """

nowTimeStampFine = lambda: datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S-%f') 
""" Return a formatted timestamp string, useful for logging and debugging """

def ensure_dir( path ):
    """ Create the directory if it does not already exist """
    if not os.path.exists( path ):
        os.mkdir( path )
        print( "Successfully created the directory %s " % path )
    else:
        print( "Path '%s' already exists" % path )

def create_log_file( logPath , prefix ):
    """ Return a file that has been opened for writing text """
    fName = str( prefix ) + dayTimeStamp() + ".txt"
    fPath = os.path.join( logPath , fName )
    print( "Created a log file at" , fPath )
    return open( fPath , 'a+' ) # Open the file in append mode

def log_print( logF , *args , **kwargs ):
    """ Print `args` to both `logF` and STDOUT , ending with newline """
    logStr = ""
    for ar in args:
#         print( ar , end = " " ) 
        logStr += ' ' + str( ar )
    print
    if 'end' in kwargs:
        logStr += kwargs['end']
    else:
        logStr += '\n'
    logF.write( logStr )

def test_server():
    """ Test the python server(s) that manage the connection to the robot """
    # FIXME: 
    try:
        proxy.some_method()
    except xmlrpc.client.ProtocolError as err:
        print("A protocol error occurred")
        print("URL: %s" % err.url)
        print("HTTP/HTTPS headers: %s" % err.headers)
        print("Error code: %d" % err.errcode)
        print("Error message: %s" % err.errmsg)