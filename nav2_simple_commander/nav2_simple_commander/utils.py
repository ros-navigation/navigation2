import os
import signal
import subprocess


def find_gz_sim_processes():
    """ Find all the processes that are running gz sim."""
    ps_output = subprocess.check_output(['ps', 'aux'], text=True)
    ps_lines = ps_output.split('\n')
    gz_sim_processes = []
    for line in ps_lines:
        if 'gz sim' in line:
            columns = line.split()
            pid = columns[1]
            command = ' '.join(columns[10:])
            if command.startswith('gz sim'):
                gz_sim_processes.append((pid, command))
    
    return gz_sim_processes

def kill_process(pid):
    """Kill a process with a given PID."""
    try:
        os.kill(int(pid), signal.SIGKILL)
        print(f'Successfully killed process with PID: {pid}')
    except Exception as e:
        print(f'Failed to kill process with PID: {pid}. Error: {e}')