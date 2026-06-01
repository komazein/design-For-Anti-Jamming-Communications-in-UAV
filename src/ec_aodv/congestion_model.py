"""Simple queue-based congestion model for UAV nodes"""

# ：
# 本模块提供一个简单的基于队列的拥塞模型，用于模拟无人机节点的发送队列行为。
# - 每个节点维护一个发送队列 `tx_queue`，队列上限由 `queue_capacity` 指定。
# - 方法 `enqueue` 将报文加入队列并返回是否成功（当队列满时返回 False 表示溢出）。
# - 方法 `dequeue` 模拟取出并发送队首报文。
# - `overflow` 和 `ncd` 分别用于判断队列是否溢出以及计算归一化拥塞度（Normalized Congestion Degree）。


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
