# Determine if a point is inside a given polygon or not
# Polygon is a list of (x,y) pairs. This fuction
# returns True or False.  The algorithm is called
# "Ray Casting Method".

def point_in_poly(x,y,poly):

    n = len(poly)
    inside = False

    p1x = poly[0][0]
    p1y = poly[0][1]
     
    for i in range(n+1):
        p2x = poly[i % n][0]
        p2y = poly[i % n][1]
        if y > min(p1y,p2y):
            if y <= max(p1y,p2y):
                if x <= max(p1x,p2x):
                    if p1y != p2y:
                        xinters = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x,p1y = p2x,p2y

    return inside

## Test

polygon = [[0,0], [10,10],[10,0]]

point_x = 4.5
point_y = 4.5

## Call the fuction with the points and the polygon
print point_in_poly(point_x,point_y,polygon)
