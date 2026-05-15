"""EC-AODV packet definitions: RREQ, RREP, RERR"""
from enum import Enum
import time


class RERRStatus(Enum):
    LINK_BREAK = 1
    LINK_WARNING = 2


class RREQ:
    def __init__(self, origin, dest, seq=0, hop_count=0,
                 residual_energy=1.0, congestion_degree=0.0,
                 avg_residual=1.0, avg_congestion=0.0, path=None,
                 timestamp=None):
        self.origin = origin
        self.dest = dest
        self.seq = seq
        self.hop_count = hop_count
        # the sending node's metrics
        self.residual_energy = residual_energy
        self.congestion_degree = congestion_degree
        # accumulated path averages
        self.avg_residual = avg_residual
        self.avg_congestion = avg_congestion
        # path list of node ids
        self.path = path or [origin]
        self.timestamp = timestamp or time.time()

    def copy_for_forward(self, sender_id, sender_residual, sender_cong):
        # compute new averages
        new_avg_res = (self.avg_residual + sender_residual) / 2.0
        new_avg_cong = (self.avg_congestion + sender_cong) / 2.0
        new_path = list(self.path) + [sender_id]
        return RREQ(
            origin=self.origin,
            dest=self.dest,
            seq=self.seq,
            hop_count=self.hop_count + 1,
            residual_energy=sender_residual,
            congestion_degree=sender_cong,
            avg_residual=new_avg_res,
            avg_congestion=new_avg_cong,
            path=new_path,
            timestamp=self.timestamp,
        )


class RREP:
    def __init__(self, origin, dest, hop_count=0,
                 residual_energy=1.0, congestion_degree=0.0,
                 avg_residual=1.0, avg_congestion=0.0, path=None,
                 overall_metric=0.0, timestamp=None):
        # origin is the node that sends RREP (the destination of RREQ)
        self.origin = origin
        self.dest = dest
        self.hop_count = hop_count
        self.residual_energy = residual_energy
        self.congestion_degree = congestion_degree
        self.avg_residual = avg_residual
        self.avg_congestion = avg_congestion
        self.path = path or []
        self.overall_metric = overall_metric
        self.timestamp = timestamp or time.time()


class RERR:
    def __init__(self, origin, dest, unreachable_node, status_flag=RERRStatus.LINK_BREAK, info=None):
        self.origin = origin
        self.dest = dest
        self.unreachable_node = unreachable_node
        self.status_flag = status_flag
        self.info = info
        self.timestamp = time.time()
