from src.entities.uav_entities import Packet
from src.routing_algorithms.BASE_routing import BASE_routing
from src.utilities import config, utilities as util

# 中文说明:
# 本模块实现了 EC-AODV（Energy-Conscious AODV）路由算法的控制包和路由逻辑。
# EC-AODV 在传统 AODV 的基础上引入了能量与拥塞相关的度量，用于优先选择能量更充足且拥塞较低的路径。
# 这里定义了三种控制报文: EC_RREQPacket（路由请求）、EC_RREPPacket（路由回复）、EC_RERRPacket（路由错误）；
# 以及 `EC_AODVRouting` 路由器类（继承自 `BASE_routing`），负责处理广播、回复、路由安装与故障处理。


class EC_RREQPacket(Packet):
    # 路由请求包（扩展字段包含平均残余能量 avg_residual 与平均拥塞 avg_congestion）
    def __init__(self, origin_id, rreq_id, dest_id, simulator, hop_count=0,
                 avg_residual=1.0, avg_congestion=0.0, path=None, time_step_creation=None):
        super().__init__(time_step_creation, simulator, None)
        self.origin_id = origin_id
        self.rreq_id = rreq_id
        self.dest_id = dest_id
        self.hop_count = hop_count
        self.avg_residual = avg_residual
        self.avg_congestion = avg_congestion
        self.path = path or [origin_id]


class EC_RREPPacket(Packet):
    # 路由回复包：当某节点能够直接到达 depot 时，沿着 RREQ 报文记录的路径生成 RREP 并回传。
    def __init__(self, src_id, dest_id, origin_id, simulator, hop_count=0,
                 avg_residual=1.0, avg_congestion=0.0, path=None, overall_metric=0.0, time_step_creation=None):
        super().__init__(time_step_creation, simulator, None)
        self.src_id = src_id
        self.dest_id = dest_id
        self.origin_id = origin_id
        self.hop_count = hop_count
        self.avg_residual = avg_residual
        self.avg_congestion = avg_congestion
        self.path = path or []
        self.overall_metric = overall_metric


class EC_RERRPacket(Packet):
    # 路由错误包：用于通知链路断开或节点不可达（通知源节点清理/切换到备份路由）。
    def __init__(self, origin_id, dest_id, unreachable_node, status_flag, simulator, time_step_creation=None):
        super().__init__(time_step_creation, simulator, None)
        self.origin_id = origin_id
        self.dest_id = dest_id
        self.unreachable_node = unreachable_node
        self.status_flag = status_flag


class RouteEntry:
    # 路由表项：存储目的地、路径、平均残余能量、平均拥塞、跳数和路径整体度量值（用于主备路由选择）
    def __init__(self, dest, path, avg_residual, avg_congestion, hop_count, overall_metric):
        self.dest = dest
        self.path = list(path)
        self.avg_residual = avg_residual
        self.avg_congestion = avg_congestion
        self.hop_count = hop_count
        self.path_metric = overall_metric


class EC_AODVRouting(BASE_routing):
    ENERGY_THRESHOLD = 0.3

    def __init__(self, drone, simulator):
        super().__init__(drone, simulator)
        self.routing_table = {}  # dest_id -> RouteEntry (primary)
        self.backups = {}        # dest_id -> list of RouteEntry
        self._rreq_counter = 0
        self._seen_rreq = set()

    # 说明: routing_table 存放到各目的地的主路由（优先级最高），backups 存放同一目的地的备份路由列表。
    # _seen_rreq 用于避免重复处理相同的 RREQ（通过 origin_id, rreq_id, path 三元组识别）。

    def relay_selection(self, geo_neighbors):
        # 选择中继：如果本地已有到 depot 的主路由，则按主路由选择下一跳；否则返回 None（触发 RREQ 洪泛）。
        dest_id = self.simulator.depot.identifier
        neighs = [n[1] for n in geo_neighbors]
        if dest_id in self.routing_table:
            nh_id = self.routing_table[dest_id].path[1] if len(self.routing_table[dest_id].path) > 1 else None
            if nh_id:
                for n in neighs:
                    if n.identifier == nh_id:
                        return n
        # otherwise, no selection (we rely on RREQ flooding)
        return None

    def send_packets(self, cur_step):
        # 发包逻辑：类似 AODV，但在没有可用主路由时使用 EC-RREQ 洪泛以发现基于能量/拥塞度量的路径；
        # 当存在主路由且能选出最佳中继时，直接单播数据包给该中继。
        if self.no_transmission or self.drone.buffer_length() == 0:
            return

        if self.drone.distance_from_depot <= min(self.drone.communication_range,
                                                 self.drone.depot.communication_range):
            self.transfer_to_depot(self.drone.depot, cur_step)
            self.drone.move_routing = False
            self.current_n_transmission = 0
            return

        if cur_step % self.simulator.drone_retransmission_delta == 0:
            opt_neighbors = []
            for hpk_id in self.hello_messages:
                hpk = self.hello_messages[hpk_id]
                if hpk.time_step_creation < cur_step - config.OLD_HELLO_PACKET:
                    continue
                opt_neighbors.append((hpk, hpk.src_drone))

            if len(opt_neighbors) > 0:
                self.simulator.metrics.mean_numbers_of_possible_relays.append(len(opt_neighbors))
                best_neighbor = self.relay_selection(opt_neighbors)
                if best_neighbor is None:
                    # 没有主路由或无法确定下一跳时，构造 EC-RREQ 包并对邻居洪泛，以发现到 depot 的路径（携带平均残余能量与拥塞信息）。
                    self._rreq_counter += 1
                    local_nre = self.drone.residual_energy / float(self.simulator.drone_max_energy)
                    local_ncd = self.drone.buffer_length() / float(max(1, self.drone.buffer_max_size))
                    rreq = EC_RREQPacket(origin_id=self.drone.identifier, rreq_id=self._rreq_counter,
                                         dest_id=self.simulator.depot.identifier, simulator=self.simulator,
                                         hop_count=0, avg_residual=local_nre, avg_congestion=local_ncd,
                                         path=[self.drone.identifier], time_step_creation=cur_step)
                    neighs = [n[1] for n in opt_neighbors]
                    self.broadcast_message(rreq, self.drone, neighs, cur_step)
                else:
                    # 已有候选最佳中继，直接将缓冲中的数据包逐个单播给该中继
                    for pkd in self.drone.all_packets():
                        self.unicast_message(pkd, self.drone, best_neighbor, cur_step)

            self.current_n_transmission += 1

    def drone_reception(self, src_drone, packet: Packet, current_ts):
        # Authenticate via BASE implementation
        super().drone_reception(src_drone, packet, current_ts)

        # handle EC-AODV control packets
        try:
            from src.routing_algorithms.ec_aodv_routing import EC_RREQPacket, EC_RREPPacket, EC_RERRPacket
        except Exception:
            EC_RREQPacket = None

        if isinstance(packet, EC_RREQPacket):
            # 处理收到的 EC-RREQ：
            # 1) 使用 (origin_id, rreq_id, path) 作为唯一键避免重复处理。
            key = (packet.origin_id, packet.rreq_id, tuple(packet.path))
            if key in self._seen_rreq:
                return
            self._seen_rreq.add(key)

            # 2) 计算本节点的局部度量（残余能量与缓冲拥塞度），并判断是否低于能量阈值（若低则丢弃 RREQ）。
            local_nre = self.drone.residual_energy / float(max(1, self.simulator.drone_max_energy))
            local_ncd = self.drone.buffer_length() / float(max(1, self.drone.buffer_max_size))

            if local_nre < self.ENERGY_THRESHOLD:
                # 能量不足，放弃转发 RREQ
                return

            # 3) 更新路径上的平均残余能量与拥塞（简单取均值），并把自己加入路径
            new_avg_res = (packet.avg_residual + local_nre) / 2.0
            new_avg_cong = (packet.avg_congestion + local_ncd) / 2.0
            packet.hop_count += 1
            packet.avg_residual = new_avg_res
            packet.avg_congestion = new_avg_cong
            packet.path = list(packet.path) + [self.drone.identifier]

            # 4) 如果当前节点能直接到达 depot，则根据当前累计的度量计算总体 metric，生成 EC-RREP 并回复给 RREQ 源
            depot = self.simulator.depot
            dist_to_depot = util.euclidean_distance(self.drone.coords, depot.coords)
            if dist_to_depot <= min(self.drone.communication_range, depot.communication_range):
                # 计算整体度量：残余能量 / (与跳数相关的拥塞因子)
                hp = packet.hop_count
                denom = max(packet.avg_congestion * (0.1 if hp<=2 else 0.2 if hp<=4 else 0.4 if hp<=6 else 0.6 if hp<=8 else 1.0), 1e-6)
                overall = packet.avg_residual / denom
                rrep = EC_RREPPacket(src_id=self.drone.identifier, dest_id=packet.origin_id, origin_id=packet.origin_id,
                                      simulator=self.simulator, hop_count=packet.hop_count,
                                      avg_residual=packet.avg_residual, avg_congestion=packet.avg_congestion,
                                      path=list(packet.path), overall_metric=overall, time_step_creation=current_ts)
                # 将 RREP 单播回发送该 RREQ 的上一个节点
                self.unicast_message(rrep, self.drone, src_drone, current_ts)
                return

            # 5) 否则继续将 RREQ 转发给除 src_drone 外的其他邻居（洪泛）
            neighs = [h.src_drone for h in self.hello_messages.values() if hasattr(h, 'src_drone')]
            neighs = [n for n in neighs if n.identifier != src_drone.identifier]
            if len(neighs) > 0:
                self.broadcast_message(packet, self.drone, neighs, current_ts)

        elif isinstance(packet, EC_RREPPacket):
            # 处理 EC-RREP：将收到的路径信息转为路由表项，并选择是否更新本地路由表或添加为备份路由。
            entry = RouteEntry(dest=packet.src_id, path=packet.path, avg_residual=packet.avg_residual,
                               avg_congestion=packet.avg_congestion, hop_count=packet.hop_count,
                               overall_metric=packet.overall_metric)

            # 如果我是 RREP 的目的地（即最初发起 RREQ 的节点），则根据 path_metric 判断是否替换主路由或加入备份。
            if packet.dest_id == self.drone.identifier:
                existing = self.routing_table.get(packet.src_id)
                if not existing or entry.path_metric > existing.path_metric:
                    # 新路径更优：将原主路由移到备份，并安装新的主路由
                    if existing:
                        self.backups.setdefault(packet.src_id, []).append(existing)
                    self.routing_table[packet.src_id] = entry
                else:
                    # 新路径较差：作为备份加入
                    self.backups.setdefault(packet.src_id, []).append(entry)
                return

            # 否则按照 RREP 中记录的路径向源方向转发 RREP（沿 path 逆序前进）
            if self.drone.identifier in packet.path:
                idx = packet.path.index(self.drone.identifier)
                if idx-1 >= 0:
                    next_hop_id = packet.path[idx-1]
                    # 在 hello_messages 中查找对应下一跳的邻居对象并单播
                    neigh = None
                    for h in self.hello_messages.values():
                        if getattr(h, 'src_drone', None) and h.src_drone.identifier == next_hop_id:
                            neigh = h.src_drone
                            break
                    if neigh:
                        self.unicast_message(packet, self.drone, neigh, current_ts)
                    else:
                        # 无法找到下一跳：发出 RERR 广播，通知邻居该节点不可达
                        rerr = EC_RERRPacket(origin_id=self.drone.identifier, dest_id=packet.dest_id,
                                              unreachable_node=next_hop_id, status_flag='LINK_BREAK', simulator=self.simulator,
                                              time_step_creation=current_ts)
                        neighs = [h.src_drone for h in self.hello_messages.values()]
                        self.broadcast_message(rerr, self.drone, neighs, current_ts)

        elif isinstance(packet, EC_RERRPacket):
            # 处理 EC-RERR：任何路由表项如果包含不可达节点，则删除该项并尝试从备份中恢复路由。
            to_remove = []
            for d, r in list(self.routing_table.items()):
                if packet.unreachable_node in r.path:
                    to_remove.append(d)
            for d in to_remove:
                del self.routing_table[d]
                # 如果存在备份路由，则从备份中选择第一个（假设按优先级已排序或 FIFO）作为新的主路由
                b = self.backups.get(d, [])
                if b:
                    best = b.pop(0)
                    self.routing_table[d] = best
