"""地理计算工具：坐标系转换、距离、点在多边形判定。"""
import math

PI = math.pi
A = 6378245.0            # 克拉索夫斯基椭球长半轴
EE = 0.00669342162296594323
R_EARTH = 6371.0088      # km，平均地球半径


# ---------- 坐标系 ----------
def _t_lat(x, y):
    r = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
    r += (20.0 * math.sin(6.0 * x * PI) + 20.0 * math.sin(2.0 * x * PI)) * 2.0 / 3.0
    r += (20.0 * math.sin(y * PI) + 40.0 * math.sin(y / 3.0 * PI)) * 2.0 / 3.0
    r += (160.0 * math.sin(y / 12.0 * PI) + 320 * math.sin(y * PI / 30.0)) * 2.0 / 3.0
    return r


def _t_lon(x, y):
    r = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
    r += (20.0 * math.sin(6.0 * x * PI) + 20.0 * math.sin(2.0 * x * PI)) * 2.0 / 3.0
    r += (20.0 * math.sin(x * PI) + 40.0 * math.sin(x / 3.0 * PI)) * 2.0 / 3.0
    r += (150.0 * math.sin(x / 12.0 * PI) + 300.0 * math.sin(x / 30.0 * PI)) * 2.0 / 3.0
    return r


def wgs84_to_gcj02(lon, lat):
    """WGS84 -> GCJ-02（火星坐标）。OSM 源为 WGS84，高德/DataV 为 GCJ-02。"""
    dlat = _t_lat(lon - 105.0, lat - 35.0)
    dlon = _t_lon(lon - 105.0, lat - 35.0)
    rad = lat / 180.0 * PI
    magic = math.sin(rad)
    magic = 1 - EE * magic * magic
    sq = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((A * (1 - EE)) / (magic * sq) * PI)
    dlon = (dlon * 180.0) / (A / sq * math.cos(rad) * PI)
    return lon + dlon, lat + dlat


def gcj02_to_wgs84(lon, lat):
    """GCJ-02 -> WGS84（迭代反解）。"""
    wlng, wlat = lon, lat
    for _ in range(3):
        mlng, mlat = wgs84_to_gcj02(wlng, wlat)
        wlng += (lon - mlng)
        wlat += (lat - mlat)
    return wlng, wlat


# ---------- 距离 ----------
def haversine_km(lon1, lat1, lon2, lat2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R_EARTH * math.asin(math.sqrt(a))


# ---------- 点在多边形 ----------
def _in_ring(x, y, ring):
    inside = False
    n = len(ring)
    j = n - 1
    for i in range(n):
        xi, yi = ring[i]
        xj, yj = ring[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-18) + xi):
            inside = not inside
        j = i
    return inside


def point_in_polygon(x, y, geom):
    """geom: GeoJSON geometry (Polygon / MultiPolygon)。"""
    t = geom.get("type")
    if t == "Polygon":
        polys = [geom["coordinates"]]
    elif t == "MultiPolygon":
        polys = geom["coordinates"]
    else:
        return False
    for poly in polys:
        if not poly:
            continue
        if not _in_ring(x, y, poly[0]):
            continue
        hole = False
        for h in poly[1:]:
            if _in_ring(x, y, h):
                hole = True
                break
        if not hole:
            return True
    return False


class DistrictIndex:
    """按 (lng, lat) 查询所属行政区（GCJ-02 坐标系）。"""

    def __init__(self, features):
        self.items = []
        for f in features:
            pr = f.get("properties", {})
            self.items.append({
                "code": str(pr.get("adcode", "")),
                "name": pr.get("name", ""),
                "geom": f.get("geometry", {}),
            })
        # 预计算包围盒加速
        for it in self.items:
            xs, ys = [], []
            g = it["geom"]
            polys = [g["coordinates"]] if g.get("type") == "Polygon" else g.get("coordinates", [])
            for poly in polys:
                for ring in poly:
                    for pt in ring:
                        xs.append(pt[0]); ys.append(pt[1])
            it["bbox"] = (min(xs), min(ys), max(xs), max(ys)) if xs else None

    def lookup(self, lng, lat):
        for it in self.items:
            b = it["bbox"]
            if b and not (b[0] <= lng <= b[2] and b[1] <= lat <= b[3]):
                continue
            if point_in_polygon(lng, lat, it["geom"]):
                return it["code"], it["name"]
        return "", "（区外/未匹配）"
