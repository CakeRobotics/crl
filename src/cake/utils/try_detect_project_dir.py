import inspect
import os.path
import re

# Detects project_dir by searching the call stack for the most recent script
# named main.py or test_*.py
def try_detect_project_dir():
    stack = inspect.stack()
    for i in (range(0, len(stack))):
        path = os.path.abspath(stack[i].filename)
        if 'pytest' in path:
            continue
        basename = os.path.basename(path)
        if re.match(r'^(main\.py|test\_.+\.py)$', basename):
            return os.path.dirname(path)
    return None
