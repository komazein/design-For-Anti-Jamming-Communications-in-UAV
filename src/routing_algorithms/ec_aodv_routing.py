from src.entities.uav_entities import Packet
from src.routing_algorithms.BASE_routing import BASE_routing
from src.utilities import config, utilities as util


class EC_RREQPacket(Packet):
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
    def __init__(self, origin_id, dest_id, unreachable_node, status_flag, simulator, time_step_creation=None):
        super().__init__(time_step_creation, simulator, None)
        self.origin_id = origin_id
        self.dest_id = dest_id
        self.unreachable_node = unreachable_node
        self.status_flag = status_flag


class RouteEntry:
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

    def relay_selection(self, geo_neighbors):
        # choose next hop according to primary route if available
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
        # similar logic to AODV but using EC-AODV fields
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
                    # flood EC-RREQ to neighbors
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
                    # forward data packets to best_neighbor
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
            key = (packet.origin_id, packet.rreq_id, tuple(packet.path))
            if key in self._seen_rreq:
                return
            self._seen_rreq.add(key)

            # compute local metrics
            local_nre = self.drone.residual_energy / float(max(1, self.simulator.drone_max_energy))
            local_ncd = self.drone.buffer_length() / float(max(1, self.drone.buffer_max_size))

            if local_nre < self.ENERGY_THRESHOLD:
                # drop RREQ
                return

            # update averages
            new_avg_res = (packet.avg_residual + local_nre) / 2.0
            new_avg_cong = (packet.avg_congestion + local_ncd) / 2.0
            packet.hop_count += 1
            packet.avg_residual = new_avg_res
            packet.avg_congestion = new_avg_cong
            packet.path = list(packet.path) + [self.drone.identifier]

            # if this node can reach depot directly, reply
            depot = self.simulator.depot
            dist_to_depot = util.euclidean_distance(self.drone.coords, depot.coords)
            if dist_to_depot <= min(self.drone.communication_range, depot.communication_range):
                # compute overall metric
                hp = packet.hop_count
                denom = max(packet.avg_congestion * (0.1 if hp<=2 else 0.2 if hp<=4 else 0.4 if hp<=6 else 0.6 if hp<=8 else 1.0), 1e-6)
                overall = packet.avg_residual / denom
                rrep = EC_RREPPacket(src_id=self.drone.identifier, dest_id=packet.origin_id, origin_id=packet.origin_id,
                                      simulator=self.simulator, hop_count=packet.hop_count,
                                      avg_residual=packet.avg_residual, avg_congestion=packet.avg_congestion,
                                      path=list(packet.path), overall_metric=overall, time_step_creation=current_ts)
                # send back to the node that sent me the RREQ
                self.unicast_message(rrep, self.drone, src_drone, current_ts)
                return

            # otherwise forward RREQ to neighbors except src_drone
            neighs = [h.src_drone for h in self.hello_messages.values() if hasattr(h, 'src_drone')]
            neighs = [n for n in neighs if n.identifier != src_drone.identifier]
            if len(neighs) > 0:
                self.broadcast_message(packet, self.drone, neighs, current_ts)

        elif isinstance(packet, EC_RREPPacket):
            # install route to src_id using the path
            entry = RouteEntry(dest=packet.src_id, path=packet.path, avg_residual=packet.avg_residual,
                               avg_congestion=packet.avg_congestion, hop_count=packet.hop_count,
                               overall_metric=packet.overall_metric)

            # if I'm the origin, select primary/backup
            if packet.dest_id == self.drone.identifier:
                existing = self.routing_table.get(packet.src_id)
                if not existing or entry.path_metric > existing.path_metric:
                    if existing:
                        self.backups.setdefault(packet.src_id, []).append(existing)
                    self.routing_table[packet.src_id] = entry
                else:
                    self.backups.setdefault(packet.src_id, []).append(entry)
                return

            # otherwise forward RREP along path towards origin
            if self.drone.identifier in packet.path:
                idx = packet.path.index(self.drone.identifier)
                if idx-1 >= 0:
                    next_hop_id = packet.path[idx-1]
                    # find neighbor with that id
                    neigh = None
                    for h in self.hello_messages.values():
                        if getattr(h, 'src_drone', None) and h.src_drone.identifier == next_hop_id:
                            neigh = h.src_drone
                            break
                    if neigh:
                        self.unicast_message(packet, self.drone, neigh, current_ts)
                    else:
                        # can't forward, emit RERR
                        rerr = EC_RERRPacket(origin_id=self.drone.identifier, dest_id=packet.dest_id,
                                              unreachable_node=next_hop_id, status_flag='LINK_BREAK', simulator=self.simulator,
                                              time_step_creation=current_ts)
                        neighs = [h.src_drone for h in self.hello_messages.values()]
                        self.broadcast_message(rerr, self.drone, neighs, current_ts)

        elif isinstance(packet, EC_RERRPacket):
            # remove routes containing unreachable node and attempt backup
            to_remove = []
            for d, r in list(self.routing_table.items()):
                if packet.unreachable_node in r.path:
                    to_remove.append(d)
            for d in to_remove:
                del self.routing_table[d]
                # try backup
                b = self.backups.get(d, [])
                if b:
                    best = b.pop(0)
                    self.routing_table[d] = best
