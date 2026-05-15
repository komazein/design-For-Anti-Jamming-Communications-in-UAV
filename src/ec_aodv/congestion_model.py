"""Simple queue-based congestion model for UAV nodes"""


class CongestionModel:
    def __init__(self, queue_capacity=10):
        self.queue_capacity = int(queue_capacity)
        self.tx_queue = []

    def enqueue(self, pkt):
        if len(self.tx_queue) >= self.queue_capacity:
            # overflow
            return False
        self.tx_queue.append(pkt)
        return True

    def dequeue(self):
        if not self.tx_queue:
            return None
        return self.tx_queue.pop(0)

    def overflow(self):
        return len(self.tx_queue) >= self.queue_capacity

    def ncd(self):
        # normalized congestion degree NCD = Lf(t) / LT
        if self.queue_capacity <= 0:
            return 1.0
        return max(0.0, min(1.0, len(self.tx_queue) / float(self.queue_capacity)))
