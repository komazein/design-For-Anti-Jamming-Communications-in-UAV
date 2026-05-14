import os
from src.simulation.simulator import Simulator
from src.utilities import config

# targeted spoof experiment (higher injection rate to collect enough attack samples)
EXPERIMENTS = [
    {"attack_injection_rate": 0.2, "attacker_fraction": 0.2},
]

OUT_DIR = config.ROOT_EVALUATION_DATA
os.makedirs(OUT_DIR, exist_ok=True)

for exp in EXPERIMENTS:
    config.ENABLE_ATTACKS = True
    # only spoof attacks (invalid HMAC) for clearer authentication evaluation
    config.ATTACK_TYPES = ['spoof']
    # enable debug prints to help diagnose HMAC behavior
    config.DEBUG = True
    # force injected packets to be delivered (bypass channel) for detection evaluation
    config.FORCE_INJECT_DELIVERED = True
    config.ATTACK_INJECTION_RATE = exp["attack_injection_rate"]
    config.ATTACKER_FRACTION = exp["attacker_fraction"]
    for use_hmac in (True, False):
        config.USE_HMAC = use_hmac
        # small simulation settings to prove authentication capability
        # reduce simulation size to avoid MemoryError under high injection rate
        config.CAPTURED_PACKET_CACHE_SIZE = 200
        sim = Simulator(len_simulation=40, n_drones=4)
        # ensure at least one attacker is placed near a target so injected packets can be received
        try:
            if hasattr(sim, 'attackers') and len(sim.attackers) > 0:
                attacker = sim.attackers[0]
                # pick a victim that is not the attacker
                victim = next((d for d in sim.drones if not getattr(d, 'is_attacker', False)), None)
                if victim is not None:
                    # move attacker very close to victim to guarantee reachability
                    attacker.coords = (victim.coords[0] + 1, victim.coords[1] + 1)
        except Exception:
            pass
        sim.run()
        # ensure metrics computed
        sim.metrics.other_metrics()

        # custom output filename
        mode = 'hmac_on' if use_hmac else 'hmac_off'
        fname = os.path.join(OUT_DIR, f"exp_inj{exp['attack_injection_rate']}_af{exp['attacker_fraction']}_{mode}_{sim.simulation_name}.json")
        sim.metrics.save_as_json(fname)
        print(f"Saved results to {fname}")

print('All experiments done')
