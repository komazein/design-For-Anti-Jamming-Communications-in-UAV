
import src.utilities.utilities as util
from src.entities.uav_entities import DataPacket
from src.simulation.metrics import Metrics
from src.utilities import config


class MediumDispatcher:

    def __init__(self, metric_class: Metrics):
        self.packets = []
        self.metric_class = metric_class
        # captured packets on the medium (for attacker replay)
        self.captured_packets = []

    def send_packet_to_medium(self, packet, src_drone, dst_drone, to_send_ts):
        if isinstance(packet, DataPacket):
            self.metric_class.all_data_packets_in_simulation += 1
        else:
            self.metric_class.all_control_packets_in_simulation += 1

        # capture the packet (shallow record) for potential attacker replay
        try:
            self.captured_packets.append((packet, src_drone, dst_drone, to_send_ts))
            # cap buffer to avoid unbounded memory growth
            cap = getattr(config, 'CAPTURED_PACKET_CACHE_SIZE', None)
            if cap is not None and len(self.captured_packets) > cap:
                # drop oldest until within cap
                while len(self.captured_packets) > cap:
                    self.captured_packets.pop(0)
        except Exception:
            pass

        self.packets.append((packet, src_drone, dst_drone, to_send_ts))

    def run_medium(self, current_ts):
        to_drop_indices = []
        original_self_packets = self.packets[:]
        self.packets = []

        for i in range(len(original_self_packets)):
            packet, src_drone, dst_drone, to_send_ts = original_self_packets[i]

            # possibly inject attacker packets at this time step
            if self.metric_class.simulator is not None and config.ENABLE_ATTACKS:
                sim = self.metric_class.simulator
                # for each attacker, with some probability inject a malicious packet
                for attacker in getattr(sim, 'attackers', []):
                    if sim.rnd_routing.rand() <= config.ATTACK_INJECTION_RATE:
                        # pick a random target drone among those currently reachable by the attacker
                        all_targets = [d for d in sim.drones if d.identifier != attacker.identifier]
                        # filter by in-range (symmetric)
                        targets = [d for d in all_targets if util.euclidean_distance(attacker.coords, d.coords) <= min(attacker.communication_range, d.communication_range)]
                        if len(targets) == 0:
                            # no reachable targets now: skip injecting to avoid sent/received mismatch
                            continue
                        dst = targets[sim.rnd_routing.randint(len(targets))]
                        atk_type = config.ATTACK_TYPES[sim.rnd_routing.randint(len(config.ATTACK_TYPES))]
                        # build a malicious packet
                        if atk_type == 'spoof':
                            # create a fake data packet with invalid HMAC
                            fake_pkt = DataPacket(current_ts, sim)
                            fake_pkt.auth_nonce = current_ts
                            fake_pkt.auth_hmac = 'deadbeef'
                            # mark as injected by attacker for correct accounting at reception
                            try:
                                fake_pkt._injected_by_attacker = True
                            except Exception:
                                pass
                            # send from attacker to dst
                            # if configured, force direct delivery (bypass channel) for evaluation
                            if getattr(config, 'FORCE_INJECT_DELIVERED', False):
                                try:
                                    self.metric_class.auth_attacker_packets_sent += 1
                                    # directly invoke reception to ensure delivery
                                    dst.routing_algorithm.drone_reception(attacker, fake_pkt, current_ts)
                                    # count as received (injected)
                                    self.metric_class.auth_attacker_packets_received += 1
                                except Exception:
                                    pass
                            else:
                                self.packets.append((fake_pkt, attacker, dst, current_ts))
                                self.metric_class.auth_attacker_packets_sent += 1
                        elif atk_type == 'replay' and len(self.captured_packets) > 0:
                            # pick a captured packet and replay (possibly old nonce)
                            idx = sim.rnd_routing.randint(len(self.captured_packets))
                            cap_pkt, cap_src, cap_dst, cap_ts = self.captured_packets[idx]
                            # create a shallow replay of captured packet preserving auth fields
                            replay_pkt = DataPacket(cap_pkt.time_step_creation, sim, event_ref=cap_pkt.event_ref)
                            replay_pkt.auth_nonce = cap_pkt.auth_nonce
                            replay_pkt.auth_hmac = cap_pkt.auth_hmac
                            try:
                                replay_pkt._injected_by_attacker = True
                            except Exception:
                                pass
                            if getattr(config, 'FORCE_INJECT_DELIVERED', False):
                                try:
                                    self.metric_class.auth_attacker_packets_sent += 1
                                    dst.routing_algorithm.drone_reception(attacker, replay_pkt, current_ts)
                                    self.metric_class.auth_attacker_packets_received += 1
                                except Exception:
                                    pass
                            else:
                                self.packets.append((replay_pkt, attacker, dst, current_ts))
                                self.metric_class.auth_attacker_packets_sent += 1

            if to_send_ts == current_ts:  # time to send this packet
                to_drop_indices.append(i)

                if src_drone.identifier != dst_drone.identifier:
                    drones_distance = util.euclidean_distance(src_drone.coords, dst_drone.coords)
                    if drones_distance <= min(src_drone.communication_range, dst_drone.communication_range):
                        if dst_drone.routing_algorithm.channel_success(drones_distance):
                            # count attacker packets that actually reach receivers
                            try:
                                # count only packets that were injected by attackers (not honest packets originating from attacker nodes)
                                if hasattr(packet, '_injected_by_attacker') and getattr(packet, '_injected_by_attacker'):
                                    self.metric_class.auth_attacker_packets_received += 1
                            except Exception:
                                pass
                            dst_drone.routing_algorithm.drone_reception(src_drone, packet, current_ts)  # reception of a packet

        original_self_packets = [original_self_packets[i] for i in range(len(original_self_packets)) if i not in to_drop_indices]
        self.packets = original_self_packets + self.packets
