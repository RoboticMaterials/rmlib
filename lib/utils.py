# utils.py
# James Watson , 2019 August for Robotic Materials
# General purpose helper functions for robot capabilities

import numpy as np
np_choice = np.random.choice

# = Geo Functions =


def vec_diff_mag(v1, v2):
    """ Return the magnitude of the difference between two vectors """
    return np.linalg.norm(np.subtract(v1, v2))

# _ End Geo _


# = Statistical Functions =

def rand_sample_list_wo_replace(origList, N):
    """ Return a list that is composed of `N` random samples of `origList`, drawn without replacement """
    N = min(len(origList), N)  # Clamp `N` to the actual size of the input array
    rtnLst = []
    indices = np_choice(list(range(len(origList))), N, replace=False)
    for i in indices:
        rtnLst.append(origList[i])
    return rtnLst

# _ End Stats _

# == Recording ==


def dayTimeStamp(): return datetime.datetime.now().strftime(
    '%Y-%m-%d')  # http://stackoverflow.com/a/5215012/893511


""" Return a formatted timestamp string, useful for logging and debugging """


def nowTimeStamp(): return datetime.datetime.now().strftime(
    '%Y-%m-%d_%H-%M-%S')  # http://stackoverflow.com/a/5215012/893511

""" Return a formatted timestamp string, useful for logging and debugging """


def nowTimeStampFine(): return datetime.datetime.now().strftime(
    '%Y-%m-%d_%H-%M-%S-%f')  # http://stackoverflow.com/a/5215012/893511

""" Return a formatted timestamp string, useful for logging and debugging """


def verify_directory(nPath):
    """ If the directory does not exist, then create it, Else do nothing """
    if os.path.isdir(nPath):
        print(nPath, "exists! No action.")
    else:
        try:
            os.mkdir(nPath)
        except OSError:
            print("Creation of the directory %s failed" % nPath)
        else:
            print("Successfully created the directory %s " % nPath)


def mkdir_of_the_day(parentPath):
    """ Create a directory under 'parentPath' named after the calendar day: YYYY-MM-DD """
    desDir = None
    verify_directory(parentPath)
    try:
        desDir = os.path.join(parentPath, dayTimeStamp())
        if not os.path.exists(desDir):
            os.mkdir(desDir)
            print("Successfully created the directory %s " % desDir)
        else:
            print("Path '%s' already exists" % desDir)
    except OSError:
        print("Creation of the directory %s failed" % desDir)
    return desDir


def tcp_test(self):
    """ Test comm issues """
    return self.get_tcp_pose()


class Counter:
    """ Keeps tracks of calls """

    def __init__(self):
        """ Set the counter to zero """
        self.count = 0

    def reset(self):
        """ Set the counter to zero """
        self.__init__()

    def __call__(self):
        """ Increment the counter """
        self.count += 1
        return self.count

    def __str__(self):
        """ Return the count as a string """
        return str(self.count)

    def set_count(self, i):
        """ Manually set the counter """
        self.count = int(i)

# __ End Record __

# = Container Functions =


def any_A_in_B(A, B):
    """ Return True if any element of A is in B, Otherwise return False """
    for a in A:
        if a in B:
            return True
    return False

# _ End Container _
