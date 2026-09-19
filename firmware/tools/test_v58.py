import json,shutil,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'analysis/v58_tests'
def call(cmd):
 r=subprocess.run([str(x) for x in cmd],cwd=ROOT,capture_output=True,text=True,encoding='utf-8',errors='replace')
 if r.returncode:raise RuntimeError(r.stdout+r.stderr)
 return r.stdout
if __name__=='__main__':
 OUT.mkdir(parents=True,exist_ok=True)
 exe=OUT/'fixed_probe.exe'
 call([shutil.which('g++'),'-std=c++11','-Wall','-Wextra','-Itools/native_v58','-Itools/native_manager','-Isrc','tools/native_v58/test_fixed_probe.cpp','src/roller485_manager.cpp','src/fixed_probe_manager.cpp','-o',exe])
 result=call([exe]);assert result.strip().endswith('PASS')
 print(result)
 metaexe=OUT/'metadata.exe'
 call([shutil.which('g++'),'-std=c++11','-Itools/native_v58','-Isrc','tools/native_v58/test_metadata.cpp','src/fixed_probe_json.cpp','-o',metaexe])
 serialized=call([metaexe]);metadata=json.loads(serialized);protocol=metadata['paired_probe_v58']
 assert len(protocol['trials'])==200
 assert all(len(t)==len(protocol['trial_columns']) for t in protocol['trials'])
 assert len(serialized.encode())<.8*262144
 (OUT/'METADATA_TEST_RESULT.json').write_text(json.dumps(dict(passed=True,synthetic_only=True,trials=200,columns=len(protocol['trial_columns']),serialized_bytes=len(serialized.encode()),reserve_bytes=262144),indent=2))

 print(call([sys.executable,'-B','tools/test_v58_offline.py']))
 (OUT/'TEST_RESULT.json').write_text(json.dumps({'passed':True,'native_output':result,'hardware_validated':False},indent=2),encoding='utf-8')
