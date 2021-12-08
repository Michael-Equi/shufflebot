
### Given image dimensions (pixels), real world dimensions (m), and list of points 
### find distance to each point from bottom right corner in real units
###  (HEIGHT, Width)

def findDistances(imageDimesions, realDimensions, points):
    heightConversion =  float(realDimensions[0]) / float(imageDimesions[0]) #meter / pixel
    widthConversion =  float(realDimensions[1]) / float(imageDimesions[1]) #meter / pixel
    convertedPoints = [(0. , 0.)]*len(points)
    for i in range(len(points)):
        x =  abs(points[i][1] - imageDimesions[0]) * heightConversion
        y = abs(points[i][0] - imageDimesions[1]) * widthConversion
        convertedPoints[i] = (x, y)
    return convertedPoints
