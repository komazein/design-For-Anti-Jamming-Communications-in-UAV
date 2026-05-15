"""Demo for EC-AODV protocol"""
from ec_aodv.ec_aodv import Node
from ec_aodv.packet import RREQ


def build_linear_topology():
    # create nodes A,B,C,D
    A = Node('A', initial_energy=100.0)
    B = Node('B', initial_energy=100.0)
    C = Node('C', initial_energy=100.0)
    D = Node('D', initial_energy=100.0)
    # neighbors (bidirectional)
    A.add_neighbor(B); B.add_neighbor(A)
    B.add_neighbor(C); C.add_neighbor(B)
    C.add_neighbor(D); D.add_neighbor(C)
    # alternate longer path A-E-D
    E = Node('E', initial_energy=100.0)
    A.add_neighbor(E); E.add_neighbor(A)
    E.add_neighbor(D); D.add_neighbor(E)
    return {'A':A,'B':B,'C':C,'D':D,'E':E}


def demo():
    nodes = build_linear_topology()
    A = nodes['A']
    # A initiates RREQ to D
    A.seq += 1
    rreq = RREQ(origin='A', dest='D', seq=A.seq, hop_count=0,
                residual_energy=A.energy.nre(), congestion_degree=A.congestion.ncd(),
                avg_residual=A.energy.nre(), avg_congestion=A.congestion.ncd(), path=['A'])
    A.log('RREQ', 'originating RREQ to D')
    A.broadcast(rreq)

    # simulate some time; now simulate node C failure to cause primary failover
    # assume primary will be A-B-C-D path; kill C
    print('\n--- Simulating node C energy drain to 0 (death) to trigger RERR and backup) ---')
    C = nodes['C']
    C.energy.current_energy = 0.0
    # neighbors detect unreachable and emit RERR
    # We simulate by letting C broadcast an RERR
    from ec_aodv.packet import RERR, RERRStatus
    rerr = RERR(origin='C', dest='D', unreachable_node='C', status_flag=RERRStatus.LINK_BREAK)
    C.broadcast(rerr)


if __name__ == '__main__':
    demo()
