import glob, json, os
files = sorted(glob.glob(os.path.join('data','evaluation_tests','exp_inj0.2_af0.2_*_seed*.json')))
summary = []
count_has = 0
for f in files:
    with open(f,'r') as fh:
        j=json.load(fh)
    v = j.get('auth_attacker_packets_received', None)
    v = 0 if v is None else int(v)
    summary.append((os.path.basename(f), v))
    if v>0:
        count_has+=1
print('total files', len(files))
print('files with auth_attacker_packets_received>0:', count_has)
for s in summary:
    print(s[0], s[1])
