import os
import shlex
import subprocess

# Credit: https://stackoverflow.com/questions/3503719
def source(script_path):
    command = shlex.split(f"bash -c 'source {script_path} > /dev/null && env'")
    proc = subprocess.Popen(command, stdout=subprocess.PIPE, encoding='utf-8')
    for line in proc.stdout:
        key, _, value = line.rstrip().partition("=")
        os.environ[key] = value
    proc.communicate()
