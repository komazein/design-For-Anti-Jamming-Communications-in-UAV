import os
import json
from src.simulation.simulator import Simulator
from src.utilities import config

# Batch realistic runs: FORCE_INJECT_DELIVERED=False, multiple seeds
OUT_DIR = config.ROOT_EVALUATION_DATA
os.makedirs(OUT_DIR, exist_ok=True)

EXPERIMENT = {"attack_injection_rate": 0.2, "attacker_fraction": 0.2}
SEEDS = list(range(200, 210))  # 10 different seeds

# simulation settings
config.ENABLE_ATTACKS = True
config.ATTACK_TYPES = ['spoof']
config.ATTACK_INJECTION_RATE = EXPERIMENT['attack_injection_rate']
config.ATTACKER_FRACTION = EXPERIMENT['attacker_fraction']
config.FORCE_INJECT_DELIVERED = False
config.CAPTURED_PACKET_CACHE_SIZE = 500

for seed in SEEDS:
    for use_hmac in (True, False):
        config.USE_HMAC = use_hmac
        mode = 'hmac_on' if use_hmac else 'hmac_off'
        # create simulator with explicit seed
        sim = Simulator(len_simulation=40, n_drones=4, seed=seed)
        # ensure at least one attacker close to a victim to increase chance of reachability
        try:
            if hasattr(sim, 'attackers') and len(sim.attackers) > 0:
                attacker = sim.attackers[0]
                victim = next((d for d in sim.drones if not getattr(d, 'is_attacker', False)), None)
                if victim is not None:
                    attacker.coords = (victim.coords[0] + 1, victim.coords[1] + 1)
        except Exception:
            pass

        sim.run()
        sim.metrics.other_metrics()
        fname = os.path.join(OUT_DIR, f"exp_inj{EXPERIMENT['attack_injection_rate']}_af{EXPERIMENT['attacker_fraction']}_{mode}_seed{seed}.json")
        sim.metrics.save_as_json(fname)
        print(f"Saved: {fname}")

print('Batch runs finished')
