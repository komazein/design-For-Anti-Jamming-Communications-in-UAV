"""Helper functions for EC-AODV: metric calc and HP normalization"""

def hp_score(hops):
    # Hop Count ?????
    if hops <= 2:
        return 0.1
    if 3 <= hops <= 4:
        return 0.2
    if 5 <= hops <= 6:
        return 0.4
    if 7 <= hops <= 8:
        return 0.6
    return 1.0


def overall_metric(nre, ncd, hops):
    # Overall Metric = NRE / (NCD × HP)
    hp = hp_score(hops)
    denom = max(ncd * hp, 1e-6)
    return float(nre) / denom
