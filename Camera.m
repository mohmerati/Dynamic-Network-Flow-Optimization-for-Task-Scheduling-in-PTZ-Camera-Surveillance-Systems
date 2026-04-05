classdef Camera
    properties
        color
        id
        position (1,3)
        initialViewCntr
        viewAngle
        aspectRatio
        zoomBox 
        zoomState 
        targetTrackId
        zoomLevel
        stateCounter
        viewCenter
        alpha
        beta
        blindAngle
        blindSpotRadius
        memory
        secondaryCoveredTracks
        coveredObjectIds
        jobs = struct([])
    end
    methods
        function newZoomBox = findZoomBox(obj, viewCntr, zoomX)
            global viewBox
            originalDims = [viewBox(3), viewBox(4)]; 
            CamtoCntrDist = norm(viewCntr(1:2) - obj.position(1:2));
            x = floor(CamtoCntrDist * tan(obj.viewAngle / (2*zoomX)));
            y = x / obj.aspectRatio;
            initialX = viewCntr(1) - x;
            initialY = viewCntr(2) - y;
            endX = viewCntr(1) + x;
            endY = viewCntr(2) + y;
            if initialX < 0
                initialX = 0;
            end 
            if initialY < 0
                initialY = 0;
            end
            if endX > originalDims(1)
               endX = originalDims(1); 
            end
            if endY > originalDims(2)
               endY = originalDims(2);
            end
            newZoomBox = [initialX, initialY, endX - initialX, endY - initialY];
        end
        function [alpha, beta] = findAngles(obj, viewCenter)
            camPos = obj.position;
            alpha = atan(abs(viewCenter(1) - camPos(1))/ ...
                sqrt(sum(camPos(2:3) - viewCenter(2:3)).^2));
            frac = (viewCenter(3) - camPos(3))/sqrt(sum((camPos(1:2) - viewCenter(1:2)).^2));
            beta = atan(frac);
        end
        function obj = Camera(color, idx, camPos, viewingAngle, aspectR, viewCntr, ...
                zoomX, blindAngle)
            obj.color = color;
            obj.id = idx;
            obj.position = camPos;
            obj.viewAngle = viewingAngle;
            obj.aspectRatio = aspectR;
            obj.initialViewCntr = viewCntr;
            obj.viewCenter = viewCntr;
            obj.zoomLevel = zoomX;
            obj.zoomState = ZoomState.ZOOMED_OUT;
            obj.stateCounter = 0;
            obj.targetTrackId = 0;
            obj.zoomBox = obj.findZoomBox(viewCntr, zoomX);
            obj.alpha = 0;
            obj.beta = pi/2 - atan((camPos(3)-viewCntr(3))/(camPos(2)-viewCntr(2)));
            obj.blindAngle = blindAngle;
            obj.blindSpotRadius = camPos(3) * tan(blindAngle);
            obj.jobs = struct([]);
        end
        function obj = reset(obj)
            obj.stateCounter = 0;
        end
        function obj = changeToZoomInMode(obj, zoomCenter, zoomLevel, targetId, ...
                coveredTrackIds, coveredObjectIds)
            newZoomBox = obj.findZoomBox(zoomCenter, zoomLevel);
            obj.memory = [newZoomBox, zoomCenter, zoomLevel, targetId];
            obj.zoomState = ZoomState.ZOOMING_IN;
            obj.viewCenter = [];
            obj.zoomBox = [];
            obj.zoomLevel = [];
            obj.secondaryCoveredTracks = coveredTrackIds;
            obj.coveredObjectIds = coveredObjectIds;
            obj.stateCounter = 0;
        end
        function obj = changeToZoomOutMode(obj, newViewCenter)
            obj.zoomLevel = 1;
            newZoomBox = obj.findZoomBox(newViewCenter, obj.zoomLevel);
            obj.memory = [newZoomBox, newViewCenter];
            obj.zoomState = ZoomState.ZOOMING_OUT;
            obj.stateCounter = 0;
            obj.zoomBox = [];
            obj.targetTrackId = 0;
            obj.viewCenter = [];
            obj.secondaryCoveredTracks = [];
            obj.coveredObjectIds = [];
        end
        function obj = changeZoomedInCenter(obj, newViewCenter)
            newZoomBox = obj.findZoomBox(newViewCenter, obj.zoomLevel);
            obj.zoomBox = newZoomBox;
        end
        function obj = updateCameraState(obj)
            global transitTime
            global viewBox
            
            obj.stateCounter = obj.stateCounter + 1;
            
            if obj.zoomState == ZoomState.ZOOMING_IN && obj.stateCounter <= transitTime
                obj.zoomState = ZoomState.ZOOMING_IN;
                obj.zoomBox = viewBox;
                obj.zoomLevel = 1;
            elseif obj.zoomState == ZoomState.ZOOMING_IN && obj.stateCounter > transitTime
                obj.zoomState = ZoomState.ZOOMED_IN;
                obj.zoomBox = obj.memory(1:4);
                obj.viewCenter = obj.memory(5:7);
                obj.zoomLevel = obj.memory(8);
                obj.targetTrackId = obj.memory(9);
%                 obj.secondaryCoveredTracks = obj.memory(10:end);
                obj.stateCounter = 1;
                obj.alpha = atan(abs(obj.viewCenter(1) - obj.position(1))/ ...
                    sqrt(sum(obj.position(2:3) - obj.viewCenter(2:3)).^2));
                obj.beta = pi/2 - atan((obj.position(3) - obj.viewCenter(3))/ ...
                    (obj.position(2) - obj.viewCenter(2)));
                obj.memory = [];
            elseif obj.zoomState == ZoomState.ZOOMING_OUT && obj.stateCounter > transitTime
                obj.zoomState = ZoomState.ZOOMED_OUT;
                obj.zoomBox = obj.memory(1:4);
                obj.viewCenter = obj.memory(5:7);
                obj.zoomLevel = 1;
                obj.targetTrackId = 0;
                obj.secondaryCoveredTracks = [];
                obj.coveredObjectIds = [];
                obj.stateCounter = 1;
                obj.alpha = atan(abs(obj.viewCenter(1) - obj.position(1))/ ...
                    sqrt(sum(obj.position(2:3) - obj.viewCenter(2:3)).^2));
                obj.beta = pi/2 - atan((obj.position(2)-obj.viewCenter(2))/ ...
                    (obj.position(3)-obj.viewCenter(3)));
                obj.memory = [];
            else
                return
            end
        end
        function obj = createJobBuffer(obj, jobs)
            obj.jobs = jobs;
        end
        function [obj, poppedJob] = loadNextJob(obj)
            if isempty(obj.jobs)
                poppedJob = [];
            else
                poppedJob = obj.jobs(1);
                obj.jobs(1) = [];
            end
        end
    end

end