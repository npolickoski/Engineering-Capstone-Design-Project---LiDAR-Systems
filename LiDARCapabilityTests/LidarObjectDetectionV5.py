#                            LidarObjectDetection
#                              29 March 2023
#  1. Scans Lidar .
#  2. Detect: Filters scan and stores points within range limits.
#  3. Separate:  Divides filtered scan into objects, base on angle gaps between scan points.
#  4. Obstacles: Processes ojbects to obtain the average distance, the center angle and object size.


def ObjectDetect(fHandle, NSize, minRange, maxRange, scanDist=[], scanAngle=[]):
    DegToRad = 3.14159/180
    jPts = 0                     #  number of object detections  
    kPts = 0                     #  number of gaps
    objDist = []                 #  detected objects
    objAngle = []
    

    print("  ****  Detect  ****", NSize, minRange, maxRange)
    fHandle.write("  ****  Detect  ****" + str(NSize) + ', ' + str(minRange) + ', ' + str(maxRange) + '\n')

    #filter scans for objects in range interval
    for i in range(NSize):
        if ((scanDist[i] <= maxRange) and (scanDist[i] >= minRange)):
          objDist.append(scanDist[i])
          objAngle.append(scanAngle[i])  
          jPts += 1

  
#  none found   -  a diagnostic print
    if(jPts == 0): 
        print(" No Objects Found")
        fHandle.write(" No Objects Found\n")
        return

#  add a point to the end of detections to mark the end of a scan
    objDist.append(maxRange+10)
    objAngle.append(270)

#  total number of detections
    print(" num detections  = ", jPts)
    fHandle.write(" num detections  = " + str(jPts) + '\n')

#  print angle and distances for detections, plus added last point
    for i in range(jPts+1):    
     print(objAngle[i], objDist[i])  
     fHandle.write(str(objAngle[i]) + ', ' + str(objDist[i]) + '\n')
  
#    ****  Separate Objects  ****

    N = 0        # number of obstacles
    k = 0        # number of single obstacle detections
    numDetect = jPts
    minObjAngle = []         #  object min angle
    maxObjAngle = []         #  object max angle
    minGapAngle = []
    maxGapAngle = []
    objPts = []             #  number of points/object
    objStart = []           #  object beginning index
    objEnd = []             #  object ending index

    print("  ****  Separate Objects  ****")
    fHandle.write("  ****  Separate Objects  ****\n")

    if((scanDist[0] > 100) or (scanDist[0] < 10)): 
        minGapAngle.append(scanAngle[0])
  
    else: 
#  first object min angle
        minGapAngle.append(objAngle[0])
  
    if((scanDist[0] <= 100) and (scanDist[0] >= 10)):
       minObjAngle.append(scanAngle[0])
  
    else:
       minObjAngle.append(objAngle[0])
  
# search entire scan for angle breaks in object detections   
    for j in range(numDetect):             
# find maxAngle for object N      
      if((objAngle[j+1] - objAngle[j]) < 5):   # angle jump is end of a object
        k += 1     
# next obstacle      
      else:      
# present obstacle
        objPts.append(k+1)
        objEnd.append(j)
        maxObjAngle.append(objAngle[j])     
# next obstacle             
        N += 1
        objStart.append(j+1)
        minObjAngle.append(objAngle[j+1])
        k = 0
    print("num obstacle = ", N)
    fHandle.write("num obstacle = " + str(N) + '\n')
    for i in range(N):
      print(objPts[i])
      fHandle.write(str(objPts[i]) + "\n")

    print("  ****  Obstacles  ****  ")
    fHandle.write("  ****  Obstacles  ****  \n")

#average object distances perpendicular to line-of-sight loop over object number
    obsDir = []
    obsSize = []
    obsDist = []

# loop over num obstacle points  
    for i in range(N):
        sum = 0 
    # find average distance of an obstacle    
        for j in range(objStart[i], objEnd[i]+1): 
            sum += objDist[j]
            
        obsDist.append(sum/objPts[i])   

    # find the width of the obstacle and direction to midpoint
        obsDir.append(minObjAngle[i] + (maxObjAngle[i] - minObjAngle[i])/2)
        obsSize.append((maxObjAngle[i] - minObjAngle[i])*DegToRad*obsDist[i])
  
  
# obstace distances, directions, and width   
    for m in range(N):
        print(objPts[m], objStart[m], objEnd[m])
        fHandle.write(str(objPts[m]) + ', ' + str(objStart[m]) + ', ' + str(objEnd[m]) + '\n')
        print(minObjAngle[m], maxObjAngle[m])
        fHandle.write(str(minObjAngle[m]) + ', ' + str(maxObjAngle[m]) + '\n')
        print(obsDist[m], obsDir[m], obsSize[m])
        fHandle.write(str(obsDist[m]) + ', ' + str(obsDir[m]) + ', ' + str(obsSize[m]) + '\n')
    
    return #     objPts, objStart, objEnd, minObjAngle, obsDir, obsSize
   
