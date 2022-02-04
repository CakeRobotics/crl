import sys

def is_running_as_test():
    return "pytest" in sys.modules
