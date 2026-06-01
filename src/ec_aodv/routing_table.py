"""Routing table supporting primary and backup routes with metrics"""
from collections import defaultdict

# 
# 本模块实现一个支持主/备路由的路由表数据结构：
# - `RouteEntry`：表示单条路由（目的地、路径、平均能量/拥塞、跳数与路径度量）。
# - `RoutingTable`：维护 `primary`（每个目的地的主路由）和 `backups`（每个目的地的备份路由列表），
#    并提供更新主路由、添加备份、获取/弹出最佳备份及移除包含指定节点的路由等操作。


class RouteEntry:
    def __init__(self, dest, path, avg_residual, avg_congestion, hop_count, overall_metric):
        self.dest = dest
        self.path = list(path)
        self.avg_residual = float(avg_residual)
        self.avg_congestion = float(avg_congestion)
        self.hop_count = int(hop_count)
        self.path_metric = float(overall_metric)


class RoutingTable:
    def __init__(self):
        # dest -> primary route
        self.primary = {}
        # dest -> list of backup RouteEntry
        self.backups = defaultdict(list)

    def update_primary(self, dest, route_entry):
        self.primary[dest] = route_entry

    def add_backup(self, dest, route_entry, max_backups=3):
        lst = self.backups[dest]
        # avoid duplicates by path
        paths = {tuple(r.path) for r in lst}
        if tuple(route_entry.path) in paths:
            return
        lst.append(route_entry)
        # keep sorted by metric desc
        lst.sort(key=lambda r: r.path_metric, reverse=True)
        if len(lst) > max_backups:
            lst[:] = lst[:max_backups]

    def get_primary(self, dest):
        return self.primary.get(dest)

    def get_backup(self, dest):
        lst = self.backups.get(dest, [])
        return lst[0] if lst else None

    def pop_best_backup(self, dest):
        lst = self.backups.get(dest, [])
        if not lst:
            return None
        best = lst.pop(0)
        return best

    def remove_routes_containing(self, node_id):
        # remove any route that contains node_id
        to_del = []
        for d, r in list(self.primary.items()):
            if node_id in r.path:
                to_del.append(d)
        for d in to_del:
            del self.primary[d]
        # clean backups
        for d, lst in list(self.backups.items()):
            newlst = [r for r in lst if node_id not in r.path]
            if newlst:
                self.backups[d] = newlst
            else:
                del self.backups[d]

# 说明：RoutingTable 提供对主/备路由的管理策略：备份路由按 path_metric 降序排列，弹出或移除操作
# 有助于在主路由失效时快速切换到质量最好的备份路由。
