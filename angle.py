import math
from haversine import haversine, Unit

point1= [15,60]
point2= [27.9944, 40.95229]
def angle(src,dst):
    src_r=[0,0]
    dst_r=[0,0]
    src_r[0] = math.degrees(src[0])
    src_r[1] = math.degrees(src[1])
    dst_r[0] = math.degrees(dst[1])
    dst_r[1] = math.degrees(dst[1])
    dLon = dst_r[1] - src_r[1]
    y = math.sin(dLon) * math.cos(dst_r[0])
    x = math.cos(src_r[0]) * math.sin(dst_r[0]) - math.sin(src_r[0]) * math.cos(dst_r[0]) * math.cos(dLon)
    brng = math.atan2(y,x)
    brng = math.radians(brng)
    brng = math.pi/2- brng
    dist = haversine(src,dst)*1000
    return dist,brng
# print(angle(point1,point2))
def findAngle(src,dst):
    lat1 = math.radians(src[0])
    lon1 = math.radians(src[1])
    lat2 = math.radians(dst[0])
    lon2 = math.radians(dst[1])
    dLon = lon2 - lon1
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
    brng = math.atan2(y,x)
    print("before ", brng)
    if brng > -math.pi/2:
        brng = math.pi/2-brng
    else:
        brng = -math.pi*3/2-brng
        
    return brng
print(angle(point1,point2))
print(findAngle(point1,point2))

