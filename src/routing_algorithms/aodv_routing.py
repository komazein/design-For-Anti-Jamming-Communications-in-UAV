from src.entities.uav_entities import DataPacket, ACKPacket, HelloPacket, Packet
from src.utilities import utilities as util
from src.utilities import config

from src.routing_algorithms.BASE_routing import BASE_routing


class RREQPacket(Packet):
    def __init__(self, origin_drone, origin_id, rreq_id, dest_id, simulator, hop_count=0, time_step_creation=None):
        super().__init__(time_step_creation, simulator, None)
        self.origin_drone = origin_drone
        self.origin_id = origin_id
        self.rreq_id = rreq_id
        self.dest_id = dest_id
        self.hop_count = hop_count


class RREPPacket(Packet):
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
                    # no route known: broadcast RREQ targeted to depot
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

    def drone_reception(self, src_drone, packet: Packet, current_ts):
        # HMAC authentication + replay protection:
        pkt_hmac = getattr(packet, 'auth_hmac', None)
        pkt_nonce = getattr(packet, 'auth_nonce', None)

        try:
            secret = config.GROUP_SHARED_KEY.encode()
            algo = __import__('hashlib').__getattribute__(config.HMAC_ALGO)
            expected = __import__('hmac').new(secret, f"{packet.identifier}:{str(pkt_nonce if pkt_nonce is not None else 0)}".encode(), algo).hexdigest()
        except Exception:
            if config.DEBUG:
                print("AODV: error computing expected HMAC")
            return

        # constant-time compare
        if not __import__('hmac').compare_digest(expected, str(pkt_hmac)):
            if config.DEBUG:
                print(f"AODV: dropped packet from {getattr(src_drone,'identifier',src_drone)} due to HMAC mismatch")
            return

        # replay protection: if packet timestamp exists, require it to be within window
        if pkt_nonce is not None:
            try:
                if abs(current_ts - int(pkt_nonce)) > config.HMAC_TIME_WINDOW:
                    if config.DEBUG:
                        print(f"AODV: dropped packet from {getattr(src_drone,'identifier',src_drone)} due to HMAC timestamp outside window")
                    return
            except Exception:
                # if nonce not int-castable, drop
                if config.DEBUG:
                    print("AODV: dropped packet due to invalid HMAC nonce")
                return

        # handle AODV control packets first
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

        else:
            # fallback to base behavior for Hello/Data/ACK
            super().drone_reception(src_drone, packet, current_ts)
