from src.entities.uav_entities import DataPacket, ACKPacket, HelloPacket, Packet
from src.utilities import utilities as util
from src.utilities import config

from src.routing_algorithms.BASE_routing import BASE_routing

# 中文说明：
# 本文件实现了一个简化的 AODV-like 路由算法，适配于本仿真器场景。
# 主要通过 RREQ（路由请求）/RREP（路由回复）包发现到 depot（汇聚点）的路径，
# 并在本地安装下一跳路由用于后续的数据转发。代码还包含可选的 HMAC 验证逻辑，
# 用于仿真安全机制（可在 `src.utilities.config` 中启用/禁用）。


class RREQPacket(Packet):
    # 路由请求包（Reverse-Path 安装）：当节点需要到 depot 的路由但本地无路由时，
    # 将广播该包以发现可达 depot 的节点。包中包含 origin、rreq id、目标 id 与跳数信息。
    def __init__(self, origin_drone, origin_id, rreq_id, dest_id, simulator, hop_count=0, time_step_creation=None):
        super().__init__(time_step_creation, simulator, None)
        self.origin_drone = origin_drone
        self.origin_id = origin_id
        self.rreq_id = rreq_id
        self.dest_id = dest_id
        self.hop_count = hop_count


class RREPPacket(Packet):
    # 路由回复包：当某节点（通常能直接到达 depot）接收到 RREQ 时，会生成 RREP 沿着 RREQ 的反向路径回传，
    # 从而让原始发起节点安装到目标（depot）的正向路由。RREP 包含生成者、目标 id、原始发起 id 与跳数。
    def __init__(self, src_drone, dest_id, origin_id, simulator, hop_count=0, time_step_creation=None):
        super().__init__(time_step_creation, simulator, None)
        self.src_drone = src_drone  # who generated the RREP (a node that can reach dest)
        self.dest_id = dest_id      # destination identifier (e.g., depot id)
        self.origin_id = origin_id  # original RREQ origin id (to route back)
        self.hop_count = hop_count


class AODVRouting(BASE_routing):
    """Simplified AODV-like routing implementation.

    Notes:
    - This is a simplified AODV tailored to this simulator: it discovers routes to the depot
      using RREQ/RREP packets and installs a one-hop-next routing table entry.
    - It keeps track of seen RREQs to avoid rebroadcast storms.
    """

    def __init__(self, drone, simulator):
        BASE_routing.__init__(self, drone, simulator)
        self.routing_table = {}           # dest_id -> { 'next_hop': drone, 'hop_count': int }
        self._rreq_counter = 0
        self._seen_rreq = set()           # (origin_id, rreq_id)
        # 说明：
        # - `routing_table` 存放目标到下一跳的映射（本实现只需到 depot 的路由），
        #   next_hop 为具体邻居 Drone 对象；hop_count 用于记录路径长度或转发时递增。
        # - `_seen_rreq` 用于防止重复处理同一个 RREQ（通过 (origin_id, rreq_id) 唯一标识）。

    def relay_selection(self, opt_neighbors):
        """Select next-hop according to routing table (prefer installed route to depot)."""
        dest_id = self.simulator.depot.identifier
        # opt_neighbors is list of tuples (HelloPacket, drone)
        neighs = [n[1] for n in opt_neighbors]

        if dest_id in self.routing_table:
            nh = self.routing_table[dest_id]['next_hop']
            # if next hop in current reachable neighbors, choose it
            for n in neighs:
                if n.identifier == nh.identifier:
                    return nh

        # no known route or next hop not reachable
        return None

    # 说明：本方法用于在发送数据时选择合适的中继节点。
    # 如果本地已经安装了到 depot 的主路由且下一跳仍在当前可达邻居列表中，则返回该下一跳；否则返回 None，
    # 上层逻辑会触发 RREQ 洪泛以发现新路由。

    def send_packets(self, cur_step):
        """Override send_packets to trigger RREQ when no route to depot exists."""
        # reuse base checks: no transmission or empty buffer
        if self.no_transmission or self.drone.buffer_length() == 0:
            return

        # if close enough to depot transfer
        if self.drone.distance_from_depot <= min(self.drone.communication_range,
                                                 self.drone.depot.communication_range):
            self.transfer_to_depot(self.drone.depot, cur_step)
            self.drone.move_routing = False
            self.current_n_transmission = 0
            return

        if cur_step % self.simulator.drone_retransmission_delta == 0:
            opt_neighbors = []
            for hpk_id in self.hello_messages:
                hpk: HelloPacket = self.hello_messages[hpk_id]
                if hpk.time_step_creation < cur_step - config.OLD_HELLO_PACKET:
                    continue
                opt_neighbors.append((hpk, hpk.src_drone))

            if len(opt_neighbors) > 0:
                self.simulator.metrics.mean_numbers_of_possible_relays.append(len(opt_neighbors))

                # try to pick next hop from routing table
                best_neighbor = self.relay_selection(opt_neighbors)

                if best_neighbor is None:
                    # 无已知路由：构造 RREQ 广播以发现到 depot 的路径（并安装反向路由）
                    self._rreq_counter += 1
                    rreq = RREQPacket(self.drone, self.drone.identifier, self._rreq_counter,
                                       self.simulator.depot.identifier, self.simulator, hop_count=0,
                                       time_step_creation=cur_step)

                    # broadcast to reachable neighbors
                    neighs = [n[1] for n in opt_neighbors]
                    self.broadcast_message(rreq, self.drone, neighs, cur_step)
                else:
                    # forward data packets to best_neighbor
                    for pkd in self.drone.all_packets():
                        self.unicast_message(pkd, self.drone, best_neighbor, cur_step)

            self.current_n_transmission += 1

    # 说明：send_packets 是路由器周期性触发的发送逻辑。当存在可达下一跳时直接转发数据；
    # 当无路由时，发起 RREQ 洪泛以发现路径。此逻辑与仿真时序（`drone_retransmission_delta`）相关。

    def drone_reception(self, src_drone, packet: Packet, current_ts):
        # HMAC authentication + replay protection:
        # metrics: count every incoming packet for auth
        try:
            self.simulator.metrics.auth_total_incoming += 1
        except Exception:
            pass

        pkt_hmac = getattr(packet, 'auth_hmac', None)
        pkt_nonce = getattr(packet, 'auth_nonce', None)

        if config.USE_HMAC:
            # 计算并验证 HMAC：若配置启用 HMAC，则根据共享密钥和包内 nonce 计算期望 HMAC，
            # 与接收包的 `auth_hmac` 做常量时间比较以防止计时攻击。
            try:
                secret = config.GROUP_SHARED_KEY.encode()
                algo = __import__('hashlib').__getattribute__(config.HMAC_ALGO)
                expected = __import__('hmac').new(secret, f"{packet.identifier}:{str(pkt_nonce if pkt_nonce is not None else 0)}".encode(), algo).hexdigest()
            except Exception:
                if config.DEBUG:
                    print("AODV: error computing expected HMAC")
                try:
                    self.simulator.metrics.auth_false_rejects += 1
                except Exception:
                    pass
                return

            # constant-time compare
            # debug: print expected vs received for attacker-injected packets
            try:
                if getattr(src_drone, 'is_attacker', False) and config.DEBUG:
                    print(f"AODV DEBUG: src={getattr(src_drone,'identifier',src_drone)} pkt_hmac={pkt_hmac} expected={expected} pkt_nonce={pkt_nonce}")
            except Exception:
                pass
            if not __import__('hmac').compare_digest(expected, str(pkt_hmac)):
                if config.DEBUG:
                    print(f"AODV: dropped packet from {getattr(src_drone,'identifier',src_drone)} due to HMAC mismatch")
                try:
                    self.simulator.metrics.auth_dropped_hmac += 1
                except Exception:
                    pass
                return

            # replay protection: if packet timestamp exists, require it to be within window
            if pkt_nonce is not None:
                try:
                    if abs(current_ts - int(pkt_nonce)) > config.HMAC_TIME_WINDOW:
                        if config.DEBUG:
                            print(f"AODV: dropped packet from {getattr(src_drone,'identifier',src_drone)} due to HMAC timestamp outside window")
                        try:
                            self.simulator.metrics.auth_dropped_replay += 1
                        except Exception:
                            pass
                        return
                except Exception:
                    # if nonce not int-castable, drop
                    if config.DEBUG:
                        print("AODV: dropped packet due to invalid HMAC nonce")
                    return

            # handle AODV control packets first
            try:
                self.simulator.metrics.auth_accepted += 1
            except Exception:
                pass
        else:
            # HMAC disabled: accept packets (for comparison experiments)
            try:
                self.simulator.metrics.auth_accepted += 1
            except Exception:
                pass
        if isinstance(packet, RREQPacket):
            key = (packet.origin_id, packet.rreq_id)
            if key in self._seen_rreq:
                return
            # mark seen
            self._seen_rreq.add(key)

            # install reverse route towards origin
            self.routing_table[packet.origin_id] = {'next_hop': src_drone, 'hop_count': packet.hop_count + 1}

            # if I can reach the destination (depot) directly, send RREP back
            depot = self.simulator.depot
            dist_to_depot = util.euclidean_distance(self.drone.coords, depot.coords)
            if dist_to_depot <= min(self.drone.communication_range, depot.communication_range):
                rrep = RREPPacket(self.drone, packet.dest_id, packet.origin_id, self.simulator,
                                   hop_count=0, time_step_creation=current_ts)
                # unicast back to the node that sent me the RREQ (src_drone)
                self.unicast_message(rrep, self.drone, src_drone, current_ts)
                return

            # otherwise, increment hop count and rebroadcast the RREQ
            packet.hop_count += 1
            # build list of neighbor drones from stored hello messages (except the one I received from)
            neighs = [h.src_drone for h in self.hello_messages.values() if hasattr(h, 'src_drone')]
            neighs = [n for n in neighs if n.identifier != src_drone.identifier]
            if len(neighs) > 0:
                self.broadcast_message(packet, self.drone, neighs, current_ts)

        # 说明：收到 RREQ 时，节点会：
        # 1) 使用 (origin_id, rreq_id) 防止重复处理；
        # 2) 在本地安装到 origin 的反向路由（便于后续将 RREP 送回 origin）；
        # 3) 如果能直达 depot，则生成 RREP 单播回发送该 RREQ 的节点；否则将 RREQ 洪泛转发。

        elif isinstance(packet, RREPPacket):
            # install forward route to destination
            self.routing_table[packet.dest_id] = {'next_hop': src_drone, 'hop_count': packet.hop_count + 1}

            # if I'm the original RREQ source, we're done (route installed)
            if packet.origin_id == self.drone.identifier:
                return

            # otherwise forward RREP towards origin using reverse route
            if packet.origin_id in self.routing_table:
                next_hop = self.routing_table[packet.origin_id]['next_hop']
                # increment hop and forward
                packet.hop_count += 1
                self.unicast_message(packet, self.drone, next_hop, current_ts)

        # 说明：收到 RREP 时，节点将安装到 RREP 指向的 destination（通常为 depot）的前向路由，
        # 并将 RREP 沿反向路径继续传送回原始 RREQ 发起者，直到原始发起者安装到 depot 的路由为止。

        else:
            # fallback to base behavior for Hello/Data/ACK
            super().drone_reception(src_drone, packet, current_ts)
