function sysCall_init() 
    usensors={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}
    for i=1,16,1 do
        usensors[i]=sim.getObjectHandle("Pioneer_p3dx_ultrasonicSensor"..i)
    end
    motorLeft=sim.getObjectHandle("Pioneer_p3dx_leftMotor")
    motorRight=sim.getObjectHandle("Pioneer_p3dx_rightMotor")
    noDetectionDist=0.5
    maxDetectionDist=0.2
    detect={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
    braitenbergL={-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
    braitenbergR={-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}
    v0=2

    -- ROS related stuff:
    robotHandle=sim.getObjectHandle('Pioneer_p3dx')
    pubTf = simROS.advertise('/tf', 'tf/tfMessage')
    subCmd = simROS.subscribe('/cmd_vel', 'geometry_msgs/Twist', 'cmd_callback')
    vel=nil
    omega=nil
    controlMode=sim.getScriptSimulationParameter(sim.handle_self,'controlMode')
end
-- This is a very simple EXAMPLE navigation program, which avoids obstacles using the Braitenberg algorithm


function sysCall_cleanup() 
    simROS.shutdownPublisher(pubTf)
    simROS.shutdownSubscriber(subCmd)
end 

function sysCall_sensing()
    
    local p={x=0,y=0,z=0.179}
    local q={x=0,y=0,z=0,w=1}
    tfStamped1={}
    tfStamped1['header']={seq=0,stamp=simROS.getTime(), frame_id="base_link"}
    tfStamped1['child_frame_id']="laser_link"
    tfStamped1['transform']={translation=p,rotation=q}

    pArr=sim.getObjectPosition(robotHandle,-1)
    qArr=sim.getObjectQuaternion(robotHandle,-1)
    p={x=pArr[1],y=pArr[2],z=pArr[3]}
    q={x=qArr[1],y=qArr[2],z=qArr[3],w=qArr[4]}
    --print(q)
    tfStamped2={}
    tfStamped2['header']={seq=0,stamp=simROS.getTime(), frame_id="odom"}
    tfStamped2['child_frame_id']="base_link"
    tfStamped2['transform']={translation=p,rotation=q}
    d={}
    d['transforms']={tfStamped1,tfStamped2}
    --print(d)
    simROS.publish(pubTf,d)
end

function cmd_callback(msg)
    vel=msg['linear']['x']
    omega=msg['angular']['z']
    --print(vel, omega)
end

function sysCall_actuation() 
    for i=1,16,1 do
        res,dist=sim.readProximitySensor(usensors[i])
        if (res>0) and (dist<noDetectionDist) then
            if (dist<maxDetectionDist) then
                dist=maxDetectionDist
            end
            detect[i]=1-((dist-maxDetectionDist)/(noDetectionDist-maxDetectionDist))
        else
            detect[i]=0
        end
    end
    
    vLeft=v0
    vRight=v0
    if (controlMode==1) and (vel~=nil) then
        vLeft = (vel - 0.17 * omega)*8
        vRight = (vel + 0.17 * omega)*8
    end
    
    for i=1,16,1 do
        vLeft=vLeft+braitenbergL[i]*detect[i]
        vRight=vRight+braitenbergR[i]*detect[i]
    end
    

    sim.setJointTargetVelocity(motorLeft,vLeft)
    sim.setJointTargetVelocity(motorRight,vRight)
end 
