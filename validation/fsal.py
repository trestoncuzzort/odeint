from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time

root = Path.cwd()
out = root / 'validation-output'
out.mkdir()
compiler = os.environ['VALIDATION_COMPILER']
fixed_sha = 'e72b242592a546ea33496a6262af5a4e9f6b89bf'
original_sha = 'efa150fb027847c3d85b2ffb9e5e0056ea358383'
records = {'scope': 'Targeted FSAL regression and default-controller trajectory comparison; not the entire upstream CI.',
           'compiler': compiler, 'dependencies': [], 'steps': []}
def save():
    (out / 'results.json').write_text(json.dumps(records, indent=2) + '\n')
def dependency(item):
    name = item['repository'].split('/')[-1]
    directory = root / 'deps' / name
    directory.mkdir(parents=True)
    with (out / ('dependency-' + name + '.log')).open('w') as stream:
        for command in [['git', 'init', str(directory)],
                        ['git', '-C', str(directory), 'fetch', '--depth=1',
                         'https://github.com/' + item['repository'] + '.git', item['head']],
                        ['git', '-C', str(directory), 'checkout', '--detach', 'FETCH_HEAD']]:
            subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, check=True)
    head = subprocess.check_output(['git', '-C', str(directory), 'rev-parse', 'HEAD'], text=True).strip()
    assert head == item['head'], (item, head)
    return {'repository': item['repository'], 'head': head, 'include': str(directory / 'include')}
items = [x for x in json.loads((root / 'validation/dependencies.json').read_text()) if not x['owner']]
with ThreadPoolExecutor(max_workers=6) as pool:
    records['dependencies'] = list(pool.map(dependency, items))
for label, expected in [('original', original_sha), ('fixed', fixed_sha)]:
    actual = subprocess.check_output(['git', '-C', str(root / label), 'rev-parse', 'HEAD'], text=True).strip()
    assert actual == expected, (label, actual)
records['original_sha'] = original_sha
records['fixed_sha'] = fixed_sha
save()
fixture = root / 'fixed/test/controlled_runge_kutta_fsal.cpp'
header_name = 'include/boost/numeric/odeint/stepper/controlled_runge_kutta.hpp'
header = (root / 'original' / header_name).read_text()
anchor = 'class controlled_runge_kutta< ErrorStepper , ErrorChecker , StepAdjuster , Resizer , explicit_error_stepper_fsal_tag >'
start = header.index(anchor)
old = 'return try_step( system , in , m_dxdt.m_v , t , out , dt );'
assert header[start:].count(old) == 1
alias_header = out / 'alias' / header_name
alias_header.parent.mkdir(parents=True)
alias_header.write_text(header[:start] + header[start:].replace(old,
    'return try_step( system , in , m_dxdt.m_v , t , out , m_dxdt.m_v , dt );'))
includes = [Path(x['include']) for x in records['dependencies']]
def execute(command, label):
    begin = time.monotonic()
    log = out / (label + '.log')
    with log.open('w') as stream:
        result = subprocess.run(command, cwd=out, stdin=subprocess.DEVNULL,
                                stdout=stream, stderr=subprocess.STDOUT)
    content = log.read_text(errors='replace')
    records['steps'].append({'label': label, 'command': command, 'returncode': result.returncode,
                             'seconds': time.monotonic() - begin, 'output_tail': content[-2500:]})
    save()
    print(label, 'exit', result.returncode, flush=True)
    return result.returncode, content
def build(source, owner, label, alias=False, compile_only=False):
    executable = out / (label + ('.exe' if os.name == 'nt' else ''))
    paths = ([out / 'alias/include'] if alias else []) + [root / owner / 'include'] + includes
    if compiler == 'msvc':
        command = ['cl', '/nologo', '/EHsc', '/std:c++14', '/Zc:__cplusplus', '/permissive-',
                   '/fp:strict', '/DBOOST_ALL_NO_LIB=1', '/Od',
                   '/Fo:' + str(out / (label + '.obj'))]
        command += ['/I' + str(p) for p in paths]
        command += ['/c'] if compile_only else ['/Fe:' + str(executable)]
        command += [str(source)]
    else:
        cxx = 'g++-11' if compiler == 'gcc11' else 'clang++'
        command = [cxx, '-std=c++14', '-O1', '-g', '-ffp-contract=off', '-DBOOST_ALL_NO_LIB=1']
        if compiler == 'clang-sanitized':
            command += ['-fsanitize=address,undefined', '-fno-sanitize-recover=all', '-fno-omit-frame-pointer']
        command += ['-I' + str(p) for p in paths]
        if compile_only:
            command += ['-c']
            executable = out / (label + '.o')
        command += [str(source), '-o', str(executable)]
    result, text = execute(command, label + '-build')
    return result, text, executable

result, text, _ = build(fixture, 'original', 'original', compile_only=True)
assert result != 0 and 'try_step' in text and (
    'no matching' in text or 'C2672' in text or 'C2660' in text or 'C2661' in text), text[-4000:]
result, text, program = build(fixture, 'original', 'alias-control', alias=True)
assert result == 0, text[-4000:]
result, text = execute([str(program)], 'alias-control-run')
assert result != 0 and re.search(r'[1-9][0-9]* errors detected\.', text), text[-4000:]
result, text, program = build(fixture, 'fixed', 'fixed')
assert result == 0, text[-4000:]
result, text = execute([str(program)], 'fixed-run')
assert result == 0 and 'No errors detected.' in text, text[-4000:]
result, text, program = build(root / 'validation/trajectory-oracle.cpp', 'fixed', 'trajectory')
assert result == 0, text[-4000:]
result, text = execute([str(program)], 'trajectory-run')
assert result == 0 and 'trajectories=4200' in text and 'mismatches=0' in text, text[-4000:]
records['source_sha256'] = {label: hashlib.sha256((root / label / header_name).read_bytes()).hexdigest()
                            for label in ['original', 'fixed']}
records['test_sha256'] = hashlib.sha256(fixture.read_bytes()).hexdigest()
records['oracle_sha256'] = hashlib.sha256((root / 'validation/trajectory-oracle.cpp').read_bytes()).hexdigest()
for label in ['original', 'fixed']:
    assert not subprocess.check_output(['git', '-C', str(root / label), 'status', '--porcelain',
                                       '--untracked-files=no'], text=True)
records['all_checks_passed'] = True
save()
print('Pinned original compile failure, unsafe-alias rejection, fixed regression and4200trajectories verified.', flush=True)
