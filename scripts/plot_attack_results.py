import os
import glob
import json
import re
import pandas as pd
import matplotlib.pyplot as plt

RESULTS_GLOB = "data/evaluation_tests/exp_inj*_af*.json"
OUT_PLOT = "data/plots/attack_results.png"
os.makedirs(os.path.dirname(OUT_PLOT), exist_ok=True)

rows = []
for path in sorted(glob.glob(RESULTS_GLOB)):
    try:
        with open(path, 'r') as f:
            data = json.load(f)
    except Exception as e:
        print(f"Failed to load {path}: {e}")
        continue

    # infer params from filename
    m = re.search(r"exp_inj(?P<inj>[0-9\.]+)_af(?P<af>[0-9\.]+)", os.path.basename(path))
    inj = float(m.group('inj')) if m else None
    af = float(m.group('af')) if m else None

    row = {
        'file': os.path.basename(path),
        'attack_injection_rate': inj,
        'attacker_fraction': af,
        'auth_attacker_packets_sent': data.get('auth_attacker_packets_sent'),
        'auth_dropped_hmac': data.get('auth_dropped_hmac'),
        'auth_dropped_replay': data.get('auth_dropped_replay'),
        'auth_total_incoming': data.get('auth_total_incoming'),
        'auth_false_rejects': data.get('auth_false_rejects'),
        'auth_accepted': data.get('auth_accepted'),
        'auth_detection_rate': data.get('auth_detection_rate'),
        'auth_replay_detection_rate': data.get('auth_replay_detection_rate'),
        'auth_false_reject_rate': data.get('auth_false_reject_rate'),
        'delivery_ratio': data.get('delivery_ratio'),
        'number_of_generated_events': data.get('number_of_generated_events'),
        'number_of_events_to_depot': data.get('number_of_events_to_depot')
    }
    rows.append(row)

if len(rows) == 0:
    print("No result files found matching", RESULTS_GLOB)
    exit(1)

df = pd.DataFrame(rows)
# print table
pd.set_option('display.float_format', '{:.4f}'.format)
print('\nSummary table:')
print(df[['file','attack_injection_rate','attacker_fraction','auth_attacker_packets_sent','auth_dropped_hmac','auth_dropped_replay','auth_detection_rate','auth_replay_detection_rate','auth_false_reject_rate','delivery_ratio']])

# plotting
fig, axes = plt.subplots(2, 2, figsize=(10,8))
ax = axes.ravel()

# x labels
x = df['file']

# Detection rate
ax[0].bar(x, df['auth_detection_rate'])
ax[0].set_title('Auth Detection Rate (dropped_hmac / attacker_sent)')
ax[0].set_ylim(0,1)
ax[0].set_xticklabels(x, rotation=45, ha='right')

# Replay detection rate
ax[1].bar(x, df['auth_replay_detection_rate'])
ax[1].set_title('Replay Detection Rate (dropped_replay / attacker_sent)')
ax[1].set_ylim(0,1)
ax[1].set_xticklabels(x, rotation=45, ha='right')

# False reject rate
ax[2].bar(x, df['auth_false_reject_rate'])
ax[2].set_title('False Reject Rate (false_rejects / total_incoming)')
ax[2].set_ylim(0,1)
ax[2].set_xticklabels(x, rotation=45, ha='right')

# Delivery ratio
ax[3].bar(x, df['delivery_ratio'])
ax[3].set_title('Delivery Ratio')
ax[3].set_ylim(0,1)
ax[3].set_xticklabels(x, rotation=45, ha='right')

plt.tight_layout()
plt.savefig(OUT_PLOT)
print(f"Saved plot to {OUT_PLOT}")

# also save a CSV summary
csv_out = 'data/plots/attack_results_summary.csv'
df.to_csv(csv_out, index=False)
print(f"Saved CSV summary to {csv_out}")
