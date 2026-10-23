"""Run native host logic, controller, configuration, and wire-format checks."""
from pathlib import Path
import os
import shlex
import subprocess
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[2]
tests={
    'independent_steering':['independent_steering'],
    'cornering_control':['cornering_control'],
    'drivetrain_control':['drivetrain_control'],
    'steering_input_filter':['steering_input_filter'],
    'remote_command':['remote_command','independent_steering'],
}
with tempfile.TemporaryDirectory() as tmp:
    for test,modules in tests.items():
        exe=Path(tmp)/test
        subprocess.run([*shlex.split(os.environ.get('CC','cc')),'-std=c11','-Wall','-Wextra','-Werror',
                        '-I',str(ROOT/'main/include'),*[str(ROOT/'main'/f'{m}.c') for m in modules],
                        str(ROOT/'tests/host'/f'test_{test}.c'),'-lm','-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
        print(f'{test}: passed',flush=True)
for test in ['controller','config_store','wire_formats']:
    subprocess.run([sys.executable,str(ROOT/'tests/host'/f'test_{test}.py')],check=True,cwd=ROOT)
