"""EC-AODV protocol implementation for simulated nodes"""
from .packet import RREQ, RREP, RERR, RERRStatus
from .energy_model import EnergyModel
from .congestion_model import CongestionModel
from .routing_table import RoutingTable, RouteEntry
from .route_manager import overall_metric
import time

# ：
# 本模块实现了 EC-AODV 协议在节点级别的模拟：
# - 每个 `Node` 对象维护能量模型（`EnergyModel`）、拥塞模型（`CongestionModel`）、路由表（`RoutingTable`）和邻居列表。
# - 节点支持发送/接收报文、广播、处理 RREQ/RREP/RERR 控制报文，以及路由维护（主/备路由切换）。
# - 关键函数：
#   - `send_packet`：模拟发送行为（消耗发送能量、入队、溢出处理并立即传输给邻居）。
#   - `receive_packet`：接收并分派到不同的控制包处理器 `_handle_rreq/_handle_rrep/_handle_rerr`。
#   - `_handle_rreq`：处理收到的 RREQ，计算并传播平均能量/拥塞信息；当节点为目的地时生成 RREP。
#   - `_handle_rrep`：RREP 返程时更新路由表（主/备），并将 RREP 继续向 origin 传回。
#   - `_handle_rerr`：收到 RERR 时删除包含不可达节点的路由并尝试切换到备份路由。


ENERGY_THRESHOLD = 0.3


class Node:
    def __init__(self, node_id, initial_energy=100.0, queue_capacity=10):
        self.id = node_id
        self.energy = EnergyModel(initial_energy=initial_energy)
        self.congestion = CongestionModel(queue_capacity=queue_capacity)
        self.routing_table = RoutingTable()
        self.neighbors = []  # list of Node instances
        self.seq = 0
        self.processed_rreq = set()
        self.logs = []

    def log(self, tag, msg):
        s = f"[{tag}] Node {self.id}: {msg}"
        print(s)
        self.logs.append(s)

    def add_neighbor(self, other):
        if other not in self.neighbors:
            self.neighbors.append(other)

    def send_packet(self, pkt, dst_node):
        # simulate tx cost and enqueue
        if not self.energy.is_alive():
            self.log('ENERGY', f"dead cannot send to {dst_node.id}")
            return False
        self.energy.consume_tx()
        enqueued = self.congestion.enqueue(pkt)
        if not enqueued:
            self.log('CONGESTION', f"queue overflow sending to {dst_node.id}")
            # overflow triggers maintenance
            self.handle_queue_overflow()
            return False
        # immediate transmit for simulator
        self.congestion.dequeue()
        dst_node.receive_packet(pkt, self)
        return True

    # 说明：本实现为简化的模拟：发送操作会立刻将包从队列中出队并传递给目标节点的 `receive_packet`，
    # 因此这里把队列主要作为拥塞/溢出检测手段，而非精确的物理传输建模。

    def broadcast(self, pkt):
        for n in list(self.neighbors):
            # send a shallow copy conceptually
            self.send_packet(pkt, n)

    # 说明：broadcast 在模拟中将包发给所有当前邻居（逐个调用 send_packet）。

    def receive_packet(self, pkt, sender):
        # simulate rx cost
        if not self.energy.is_alive():
            self.log('ENERGY', 'dead cannot receive')
            return
        self.energy.consume_rx()
        # dispatch by packet type
        if isinstance(pkt, RREQ):
            self._handle_rreq(pkt, sender)
        elif isinstance(pkt, RREP):
            self._handle_rrep(pkt, sender)
        elif isinstance(pkt, RERR):
            self._handle_rerr(pkt, sender)

    # 说明：receive_packet 会根据收到报文的类型调度到具体处理函数。接收时也会消耗接收能量。

    def _handle_rreq(self, rreq, sender):
        key = (rreq.origin, rreq.seq, tuple(rreq.path))
        if key in self.processed_rreq:
            return
        self.processed_rreq.add(key)
        # energy threshold check
        nre_local = self.energy.nre()
        ncd_local = self.congestion.ncd()
        if nre_local < ENERGY_THRESHOLD:
            self.log('RREQ', f"drop RREQ from {rreq.origin} due low energy {nre_local:.2f}")
            return
        # update averages and hop count then forward or reply
        new_rreq = rreq.copy_for_forward(self.id, nre_local, ncd_local)
        self.log('RREQ', f"recv from {sender.id} origin={rreq.origin} dest={rreq.dest} hop={new_rreq.hop_count} avg_e={new_rreq.avg_residual:.3f} avg_c={new_rreq.avg_congestion:.3f}")
        # if I am destination, send RREP back
        if self.id == rreq.dest:
            metric = overall_metric(new_rreq.avg_residual, new_rreq.avg_congestion, new_rreq.hop_count)
            rrep = RREP(origin=self.id, dest=rreq.origin, hop_count=new_rreq.hop_count,
                        residual_energy=nre_local, congestion_degree=ncd_local,
                        avg_residual=new_rreq.avg_residual, avg_congestion=new_rreq.avg_congestion,
                        path=list(new_rreq.path), overall_metric=metric)
            self.log('RREP', f"send to {rreq.origin} metric={metric:.4f} path={rrep.path}")
            # send RREP back along reversed path
            self._send_rrep_back(rrep)
            return
        # otherwise decide whether to forward based on metric progression
        # compute tentative overall metric
        metric = overall_metric(new_rreq.avg_residual, new_rreq.avg_congestion, new_rreq.hop_count)
        # simple rule: forward if metric not worse than very small threshold
        # (paper requires propagation of metric; we forward to let origin collect best)
        self.log('RREQ', f"forwarding origin={rreq.origin} tentative_metric={metric:.4f}")
        self.broadcast(new_rreq)

    # 说明：
    # - RREQ 在传播过程中记录并更新路径的平均残余能量与平均拥塞度量；
    # - 只有当中继节点能量高于阈值时才继续转发；目的节点会基于累计度量计算整体 metric 并生成 RREP。

    def _send_rrep_back(self, rrep):
        # path contains nodes from origin to this node; need to traverse back
        path = list(rrep.path)
        if not path:
            return
        # current node should be last in path; send to previous hop towards origin
        cur_index = path.index(self.id) if self.id in path else len(path)-1
        # walk back to origin
        for i in range(cur_index-1, -1, -1):
            next_hop = path[i]
            # find neighbor instance
            neighbor = next((n for n in self.neighbors if n.id == next_hop), None)
            if neighbor:
                neighbor.receive_packet(rrep, self)
            else:
                # cannot reach previous hop -> RERR
                rerr = RERR(origin=self.id, dest=rrep.dest, unreachable_node=next_hop, status_flag=RERRStatus.LINK_BREAK)
                self.log('RERR', f"cannot send RREP back, emit RERR unreachable={next_hop}")
                self.broadcast(rerr)
                return

    # 说明：_send_rrep_back 将 RREP 沿着 RREQ 的反向路径逐跳发送回原始发起者；若中间某跳不可达则生成并广播 RERR。

    def _handle_rrep(self, rrep, sender):
        # RREP traverses back to origin; each intermediate node updates routing table
        # If I am the origin of the RREQ, collect RREP and choose best route
        self.log('RREP', f"recv from {sender.id} origin={rrep.origin} dest={rrep.dest} metric={rrep.overall_metric:.4f} path={rrep.path}")
        # create route entry
        entry = RouteEntry(dest=rrep.origin, path=rrep.path, avg_residual=rrep.avg_residual,
                           avg_congestion=rrep.avg_congestion, hop_count=rrep.hop_count,
                           overall_metric=rrep.overall_metric)
        # update routing tables along path
        # if I'm destination of this RREP (i.e., origin of RREQ), decide primary/backup
        if self.id == rrep.dest:
            # origin decides route selection
            existing = self.routing_table.get_primary(rrep.origin)
            if not existing or entry.path_metric > existing.path_metric:
                # demote old primary to backup
                if existing:
                    self.routing_table.add_backup(rrep.origin, existing)
                self.routing_table.update_primary(rrep.origin, entry)
                self.log('ROUTE_SWITCH', f"primary route to {rrep.origin} set metric={entry.path_metric:.4f} path={entry.path}")
            else:
                # add as backup if good
                self.routing_table.add_backup(rrep.origin, entry)
                self.log('BACKUP_ROUTE', f"added backup to {rrep.origin} metric={entry.path_metric:.4f} path={entry.path}")
            return
        # otherwise forward RREP towards origin using next hop in path list
        # find my index in path
        if self.id in rrep.path:
            idx = rrep.path.index(self.id)
            if idx-1 >= 0:
                prev_hop = rrep.path[idx-1]
                neighbor = next((n for n in self.neighbors if n.id == prev_hop), None)
                if neighbor:
                    neighbor.receive_packet(rrep, self)
                else:
                    # cannot forward back -> RERR
                    rerr = RERR(origin=self.id, dest=rrep.dest, unreachable_node=prev_hop, status_flag=RERRStatus.LINK_BREAK)
                    self.log('RERR', f"RREP forward failed to {prev_hop}, emit RERR")
                    self.broadcast(rerr)

    # 说明：当 RREP 到达原始 RREQ 发起节点时，发起节点会在路由表中安装主路由或将其作为备份。
    # 中间节点在转发 RREP 时也会安装到目的节点（RREP.origin）的路由条目，便于后续数据转发。

    def _handle_rerr(self, rerr, sender):
        self.log('RERR', f"recv from {sender.id} unreachable={rerr.unreachable_node} status={rerr.status_flag}")
        # remove any route containing unreachable node
        self.routing_table.remove_routes_containing(rerr.unreachable_node)
        # attempt failover using backup
        primary = self.routing_table.get_primary(rerr.dest)
        if primary is None:
            backup = self.routing_table.pop_best_backup(rerr.dest)
            if backup:
                self.routing_table.update_primary(rerr.dest, backup)
                self.log('ROUTE_SWITCH', f"switched to backup for {rerr.dest} path={backup.path}")
                return
        # otherwise propagate RERR
        self.broadcast(rerr)

    # 说明：收到 RERR 表明某节点不可达，节点会清除受影响的路由并尝试用备份路由替换主路由，若无备份则继续广播 RERR。

    # Route maintenance triggers
    def handle_queue_overflow(self):
        self.log('CONGESTION', 'queue overflow detected, attempting backup switch')
        # for all affected destinations attempt backup
        for dest in list(self.routing_table.primary.keys()):
            backup = self.routing_table.pop_best_backup(dest)
            if backup:
                self.routing_table.update_primary(dest, backup)
                self.log('BACKUP_ROUTE', f"switched to backup for {dest} path={backup.path}")

    # 说明：当本节点检测到发送队列溢出时，会尝试为所有受影响目标切换到备份路由以缓解拥塞。

    def handle_energy_threshold(self):
        # if my energy below threshold, inform neighbors via RERR (warning)
        if self.energy.nre() < ENERGY_THRESHOLD:
            self.log('ENERGY', f'below threshold {self.energy.nre():.3f}, sending LINK_WARNING')
            # notify neighbors
            for dest in list(self.routing_table.primary.keys()):
                rerr = RERR(origin=self.id, dest=dest, unreachable_node=self.id, status_flag=RERRStatus.LINK_WARNING)
                self.broadcast(rerr)

    # 说明：当本节点能量低于阈值时，会通知邻居（通过 `RERR` 状态为 LINK_WARNING），以便其他节点在路由选择时避开低能量节点。
