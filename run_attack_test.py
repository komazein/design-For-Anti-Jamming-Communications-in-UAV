from src.simulation.simulator import Simulator
from src.utilities import config

config.ENABLE_ATTACKS = True
config.ATTACK_INJECTION_RATE = 0.3

if __name__ == '__main__':
    sim = Simulator(len_simulation=20, n_drones=6)
    sim.run()
    sim.close()
    print('Done')
