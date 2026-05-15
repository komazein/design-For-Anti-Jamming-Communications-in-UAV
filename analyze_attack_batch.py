import os
import json
import glob
import numpy as np
import csv

DATA_DIR = os.path.join('data', 'evaluation_tests')
OUT_DIR = os.path.join('data', 'plots')
os.makedirs(OUT_DIR, exist_ok=True)

pattern = os.path.join(DATA_DIR, 'exp_inj0.2_af0.2_*_seed*.json')
files = sorted(glob.glob(pattern))

# group by mode
groups = {'hmac_on': [], 'hmac_off': []}
for f in files:
    if 'hmac_on' in f:
        groups['hmac_on'].append(f)
    elif 'hmac_off' in f:
        groups['hmac_off'].append(f)

metrics = ['auth_detection_rate', 'auth_replay_detection_rate', 'auth_false_reject_rate', 'delivery_ratio']
summary_rows = []

for mode, flist in groups.items():
    vals = {m: [] for m in metrics}
    for f in flist:
        with open(f, 'r') as fh:
            j = json.load(fh)
        for m in metrics:
            v = j.get(m)
            if v is None:
                vals[m].append(np.nan)
            else:
                try:
                    vals[m].append(float(v))
                except Exception:
                    vals[m].append(np.nan)
    n = len(flist)
    for m in metrics:
        arr = np.array(vals[m], dtype=np.float64)
        # ignore nan
        arr_nonan = arr[~np.isnan(arr)]
        mean = float(np.nan) if arr_nonan.size == 0 else float(np.mean(arr_nonan))
        stderr = float(np.nan) if arr_nonan.size == 0 else float(np.std(arr_nonan, ddof=1) / np.sqrt(arr_nonan.size))
        ci95 = float(np.nan) if arr_nonan.size == 0 else 1.96 * stderr
        summary_rows.append({'mode': mode, 'metric': m, 'n': int(arr_nonan.size), 'mean': mean, 'ci95': ci95})

# write CSV
csvfile = os.path.join(OUT_DIR, 'hmac_batch_summary.csv')
with open(csvfile, 'w', newline='') as cf:
    writer = csv.DictWriter(cf, fieldnames=['mode', 'metric', 'n', 'mean', 'ci95'])
    writer.writeheader()
    for r in summary_rows:
        writer.writerow(r)

print('Wrote summary to', csvfile)
print('Files processed:')
for k, v in groups.items():
    print(k, len(v))

# print table
for row in summary_rows:
    print(row)
