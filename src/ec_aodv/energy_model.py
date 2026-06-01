"""Simple energy model for UAV nodes"""

# ：
# 本模块提供一个非常简化的能量模型，用于模拟无人机节点的能量消耗。
# - `initial_energy`：初始能量值（用于归一化计算 NRE）。
# - `tx_cost` / `rx_cost`：每次发送/接收消耗的能量成本。
# - `consume_tx` / `consume_rx`：消耗能量并保证能量下限为 0。
# - `is_alive`：判断节点是否还有能量（>0）。
# - `nre`：归一化剩余能量（NRE），返回值范围 [0,1]。


class EnergyModel:
    def __init__(self, initial_energy=100.0, tx_cost=0.5, rx_cost=0.2):
        self.initial_energy = float(initial_energy)
        self.current_energy = float(initial_energy)
        self.tx_cost = float(tx_cost)
        self.rx_cost = float(rx_cost)

    def consume_tx(self, packets=1):
        cost = self.tx_cost * packets
        self.current_energy -= cost
        if self.current_energy < 0:
            self.current_energy = 0.0
        return cost

    def consume_rx(self, packets=1):
        cost = self.rx_cost * packets
        self.current_energy -= cost
        if self.current_energy < 0:
            self.current_energy = 0.0
        return cost

    def is_alive(self):
        return self.current_energy > 0.0

    def nre(self):
        # Normalized residual energy NRE = Er(t) / Ei in [0,1]
        if self.initial_energy <= 0:
            return 0.0
        return max(0.0, min(1.0, self.current_energy / self.initial_energy))
