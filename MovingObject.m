classdef MovingObject
    properties
        position (:,3)
        velocity (:,2)
        width (1,:)
        height (1,:)
        posVar
        velVar
        sizeVar
        startTime
        stopTime
        id
        arrivalTime
        recordedTime
        trackId
        age
        totalVisibleCount
        consecutiveInvisibleCount
        zoomedInVisibleCount
        cameraSpecificZoomedInVisibleCount
    end
    methods
        function obj = MovingObject(posX, posY, posZ, velX, velY, w, h, ...
                processVars, sTime, eTime, id)
            obj.position(1,1) = posX;
            obj.position(1,2) = posY;
            obj.position(1,3) = posZ;
            obj.velocity(1,1) = velX;
            obj.velocity(1,2) = velY;
            obj.height(1,1) = h;
            obj.width(1,1) = w;
            obj.posVar = [processVars(1), processVars(3)];
            obj.velVar = [processVars(2), processVars(4)];
            obj.sizeVar = processVars(5:6);
            obj.startTime = sTime;
            obj.stopTime = eTime;
            obj.id = id;
            obj.arrivalTime = -1;
            obj.recordedTime = -1;
            obj.trackId = -1;
            obj.age = 0;
            obj.totalVisibleCount = 0;
            obj.consecutiveInvisibleCount = 0;
            obj.zoomedInVisibleCount = 0;
            obj.cameraSpecificZoomedInVisibleCount = [0;0;0];
        end
        function obj = setProps(obj, posX, posY, posZ, velX, velY, w, h, i)
            obj.position(i,1) = posX;
            obj.position(i,2) = posY;
            obj.position(i,3) = posZ;
            obj.velocity(i,1) = velX;
            obj.velocity(i,2) = velY;
            obj.height(i) = h;
            obj.width(i) = w;
        end
        function obj = move(obj, deltaT, i)
            obj.position(i, 1) = obj.position(i-1, 1) + obj.velocity(i-1, 1) * deltaT + normrnd(0, obj.posVar(1));
            obj.position(i, 2) = obj.position(i-1, 2) + obj.velocity(i-1, 2) * deltaT + normrnd(0, obj.posVar(2));
            obj.velocity(i, 1) = obj.velocity(i-1, 1) + normrnd(0, obj.velVar(1));
            obj.velocity(i, 2) = obj.velocity(i-1, 2) + normrnd(0, obj.velVar(2));
            obj.width(i) = obj.width(i-1) + normrnd(0, obj.sizeVar(1) * deltaT);
            obj.height(i) = obj.height(i-1) + normrnd(0, obj.sizeVar(2) * deltaT);
        end
        function props = getProps(obj,j)
            props(1:3) = obj.position(j,:);
            props(4) = obj.width(j);
            props(5) = obj.height(j);
            props(6) = obj.id;
            props(7:8) = obj.velocity(j,1:2);
        end
    end
end