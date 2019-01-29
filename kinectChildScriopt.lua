function sysCall_init() 
    depthCam=sim.getObjectHandle('kinect_depth')
    depthView=sim.floatingViewAdd(0.9,0.9,0.2,0.2,0)
    sim.adjustView(depthView,depthCam,64)

    colorCam=sim.getObjectHandle('kinect_rgb')
    colorView=sim.floatingViewAdd(0.69,0.9,0.2,0.2,0)
    sim.adjustView(colorView,colorCam,64)

    closestPtDummy=sim.getObjectHandle('closestDetectedPoint')

    -- The camera was set to resolution 64X48 on purpose (faster processing in Lua)
    camXAngleInDegrees=57
    camXResolution=256
    camYResolution=192
    camXHalfAngle=camXAngleInDegrees*0.5*math.pi/180
    camYHalfAngle=(camXAngleInDegrees*0.5*math.pi/180)*camYResolution/camXResolution
    nearClippingPlane=0.2
    depthAmplitude=3.3

    cx = camXResolution / 2 - 0.5
    cy = camYResolution / 2 - 0.5
    fx = cx / math.tan(camXHalfAngle)
    fy = cy / math.tan(camYHalfAngle)

    -- ROS related stuff:
    pubRgbImage = simROS.advertise('/kinect1/rgb/image', 'sensor_msgs/Image')
    simROS.publisherTreatUInt8ArrayAsString(pubRgbImage)

    pubDepthImage = simROS.advertise('/kinect1/depth/image', 'sensor_msgs/Image')
    pubDepthInfo = simROS.advertise('/kinect1/depth/camera_info', 'sensor_msgs/CameraInfo')
    --pubDepthPoints = simROS.advertise('/kinect1/depth_points', 'sensor_msgs/PointCloud2')
    --simROS.publisherTreatUInt8ArrayAsString(pubDepthImage)

    seq = 0
    depthBufferInt = {}
    --depthPoints = {}
end

function sysCall_cleanup() 
    simROS.shutdownPublisher(pubRgbImage)
    simROS.shutdownPublisher(pubDepthImage)
    simROS.shutdownPublisher(pubDepthInfo)
    --simROS.shutdownPublisher(pubDepthPoints)
end 

function sysCall_sensing() 
    local rgbBuffer = sim.getVisionSensorCharImage(colorCam)
    sim.transformImage(rgbBuffer, {camXResolution, camYResolution}, 4)
    local depthBuffer = sim.getVisionSensorDepthBuffer(depthCam)
    seq = seq + 1
    timestamp = simROS.getTime()
    
    for i=1,camXResolution,1 do
        local xAngle=((32-i-0.5)/32)*camXHalfAngle
        for j=1,camYResolution,1 do

            local yAngle=((j-24+0.5)/24)*camYHalfAngle
            local depthValue = depthBuffer[i+(camYResolution-j)*camXResolution]
            local zCoord=nearClippingPlane+depthAmplitude*depthValue
            --local xCoord=math.tan(xAngle)*zCoord
            --local yCoord=math.tan(yAngle)*zCoord
            --depthPoints[(i+(j-1)*64)*3 - 2] = xCoord
            --depthPoints[(i+(j-1)*64)*3 - 1] = yCoord
            --depthPoints[(i+(j-1)*64)*3 - 0] = zCoord
            
            depthBufferInt[(i+(j-1)*camXResolution)*2-1] = math.floor(1000*zCoord%256)
            depthBufferInt[(i+(j-1)*camXResolution)*2] = math.floor(1000*zCoord/256)
            
        end
    end
    --print(depthBufferInt)

    msgRgbImage = {}
    msgRgbImage['header']={seq=seq,stamp=timestamp, frame_id="kinect1_link"}
    msgRgbImage['height'] = camYResolution
    msgRgbImage['width'] = camXResolution
    msgRgbImage['encoding'] = 'rgb8'
    msgRgbImage['is_bigendian'] = 0
    msgRgbImage['step'] = camXResolution*3
    msgRgbImage['data'] = rgbBuffer

    msgDepthImage = {}
    msgDepthImage['header']={seq=seq,stamp=timestamp, frame_id="kinect1_link"}
    msgDepthImage['height'] = camYResolution
    msgDepthImage['width'] = camXResolution
    msgDepthImage['encoding'] = '16UC1' --'16UC1'  '8UC1'
    msgDepthImage['is_bigendian'] = 0
    msgDepthImage['step'] = camXResolution*2 --*2  *1
    msgDepthImage['data'] = depthBufferInt

    msgDepthInfo = {}
    msgDepthInfo['header']={seq=seq,stamp=timestamp, frame_id="kinect1_link"}
    msgDepthInfo['height'] = camYResolution
    msgDepthInfo['width'] = camXResolution
    --msgDepthInfo['distortion_model'] = 'plumb_bob'
    --msgDepthInfo['D'] = {0,0,0,0,0}
    msgDepthInfo['K'] = {fx, 0, cx, 0, fy, cy, 0, 0, 1}
    --msgDepthInfo[''] 

    --[[
    msgDepthPoints={}
    msgDepthPoints['header']={seq=seq,stamp=timestamp, frame_id="kinect1_link"}
    msgDepthPoints['height'] = camYResolution
    msgDepthPoints['width'] = camXResolution
    msgDepthPoints['is_bigendian'] = true
    msgDepthPoints['point_step'] = 3
    msgDepthPoints['row_step'] = camXResolution * 3
    msgDepthPoints['data'] = depthPoints
    --]]
    
    simROS.publish(pubRgbImage, msgRgbImage)
    simROS.publish(pubDepthImage, msgDepthImage)
    simROS.publish(pubDepthInfo, msgDepthInfo)
    --simROS.publish(pubDepthPoints, msgDepthPoints)

    --[[
    
    local closestDist=10
    local closestXYZ={0,0,0}
    for i=1,64,1 do
        local xAngle=((32-i-0.5)/32)*camXHalfAngle
        for j=1,48,1 do
            local yAngle=((j-24+0.5)/24)*camYHalfAngle
            local depthValue=depthBuffer[i+(j-1)*64]
            local zCoord=nearClippingPlane+depthAmplitude*depthValue
            local xCoord=math.tan(xAngle)*zCoord
            local yCoord=math.tan(yAngle)*zCoord
            local dist=math.sqrt(xCoord*xCoord+yCoord*yCoord+zCoord*zCoord)
    
            if (dist<closestDist) then
                closestDist=dist
                closestXYZ[1]=xCoord
                closestXYZ[2]=yCoord
                closestXYZ[3]=zCoord
            end
        end
    end
    
    if (closestDist>3.4) then
        sim.setObjectInt32Parameter(closestPtDummy,sim.objintparam_visibility_layer,0) -- hide the sphere!
        sim.setObjectPosition(closestPtDummy,depthCam,{0,0,0})
    else
        sim.setObjectInt32Parameter(closestPtDummy,sim.objintparam_visibility_layer,1) -- show the sphere in layer 1!
        sim.setObjectPosition(closestPtDummy,depthCam,closestXYZ)
    end
    
    if (sim.getSimulationState()==sim.simulation_advancing_lastbeforestop) then
        sim.setObjectInt32Parameter(closestPtDummy,sim.objintparam_visibility_layer,0) -- hide the sphere!
    end
    --]]
end 
