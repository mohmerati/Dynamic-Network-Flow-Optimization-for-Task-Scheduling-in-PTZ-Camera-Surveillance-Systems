function [movingObjects, listOfWatchableIds] = generateObjects(numRep, numFrames, numObjects, ...
    processVars, genProb)

global vidXDim
global deltaT
global viewBox

numBuckets = 10;
xPosBuckets = linspace(0, vidXDim - vidXDim/numBuckets, numBuckets);
xPosBuckets(end+1) = vidXDim - 50;
xPosBuckets(1) = 50;

objects = cell(numRep, numObjects);
movingObjects = cell(numRep, numObjects);

% if an element stays zero, it means that one of the objects can't be
% zoomed into.
listOfWatchableIds = zeros(numRep, numObjects);

poissonRate = 0.1 * genProb;

% poissonRate = genProb;

% xposes = [300, 1200, 2400];
% xvels = [9/4, 0, -3]*10;
% 
% for rep = 1:numRep
%     generatedObjectNo = 0;
%     for i = 1:numFrames
%         if generatedObjectNo < numObjects
%             numArrivals = poissrnd(poissonRate);
%             for p = 1:numArrivals
%                 for bucketNum = 1:3
%                     generatedObjectNo = generatedObjectNo + 1;
%                     xPos = xposes(bucketNum);
%                     yPos = 0;
%                     zPos = 0;
%                     width = randi([30 35]);
%                     height = randi([25 30]);
%                     xVelocity = xvels(bucketNum);
%                     yVelocity = 2*10;
%                     obj = MovingObject(xPos, yPos, zPos, xVelocity, yVelocity, width, ...
%                         height, processVars, i, numFrames, generatedObjectNo);
%                     objects{rep, generatedObjectNo} = obj;
%                 end
%             end
%         end
%     end

for rep = 1:numRep
    generatedObjectNo = 0;
    for i = 1:numFrames
        if generatedObjectNo < numObjects
            numArrivals = poissrnd(poissonRate);
            for p = 1:numArrivals
                for bucketNum = 1:numBuckets
                    generatedObjectNo = generatedObjectNo + 1;
                    xPos = randi([xPosBuckets(bucketNum) xPosBuckets(bucketNum+1)]);
                    yPos = -250;
                    zPos = 0;
                    width = randi([20 35]);
                    height = randi([20 30]);
                    xVelocity = randi([-2 2]);
                    yVelocity = randi([13 26]);
                    obj = MovingObject(xPos, yPos, zPos, xVelocity, yVelocity, width, ...
                        height, processVars, i, numFrames, generatedObjectNo);
                    objects{rep, generatedObjectNo} = obj;
                end
            end
        end
    end

% for rep = 1:numRep
%     generatedObjectNo = 0;
%     for i = 1:numFrames
%         if generatedObjectNo < numObjects
%             numArrivals = poissrnd(poissonRate);
%             for p = 1:numArrivals
%                 for bucketNum = 1:numBuckets
%                     generatedObjectNo = generatedObjectNo + 1;
%                     xPos = randi([xPosBuckets(bucketNum) xPosBuckets(bucketNum+1)]);
%                     yPos = 1400;
%                     zPos = 0;
%                     width = randi([20 35]);
%                     height = randi([20 30]);
%                     xVelocity = 0.1 * randi([-2 2]);
%                     yVelocity = 0.1 * randi([13 26]);
%                     obj = MovingObject(xPos, yPos, zPos, xVelocity, yVelocity, width, ...
%                         height, processVars, i, numFrames, generatedObjectNo);
%                     objects{rep, generatedObjectNo} = obj;
%                 end
%             end
%         end
%     end
    
    for i = 1:generatedObjectNo
        movingObjects{rep,i} = MovingObject(-inf,-inf,-inf,-10,-10,1,1,processVars, ...
            objects{rep,i}.startTime,objects{rep,i}.stopTime, objects{rep,i}.id);
    end
    
    for i = 1:generatedObjectNo
        for j = 1:numFrames
            if j < movingObjects{rep,i}.startTime
                movingObjects{rep,i} = movingObjects{rep,i}.setProps(-inf,-inf,-inf,-10,-10, 1, 1, j);
            elseif j == movingObjects{rep,i}.startTime
                movingObjects{rep,i} = movingObjects{rep,i}.setProps(objects{rep,i}.position(1) ...
                    ,objects{rep,i}.position(2),objects{rep,i}.position(3),objects{rep,i}.velocity(1) ...
                    ,objects{rep,i}.velocity(2),objects{rep,i}.width,objects{rep,i}.height,j);
            elseif j > movingObjects{rep,i}.startTime && j <= movingObjects{rep,i}.stopTime
                movingObjects{rep,i} = movingObjects{rep,i}.move(deltaT,j);
                alreadyCheckedFlag = listOfWatchableIds(rep,:) == movingObjects{rep,i}.id;
                if ~any(alreadyCheckedFlag)
                    objProps = movingObjects{rep,i}.getProps(j);
                    cond1 = objProps(1:2) >= 0;
                    cond2 = objProps(1:2) + objProps(4:5) <= viewBox(3:4);
                    if all([cond1,cond2])
                        movingObjects{rep,i}.arrivalTime = j;
                        watchableFlag = isWatchable(movingObjects{rep,i},j,numFrames);
                        if watchableFlag
                            listOfWatchableIds(rep, i) = movingObjects{rep,i}.id;
                        end
                    end
                end
            elseif j > movingObjects{rep,i}.stopTime
                movingObjects{rep,i} = movingObjects{rep,i}.setProps(-inf,-inf,-inf,-10,-10, 1, 1, j);
            end
        end
    end
end

end



function  watchableFlag = isWatchable(object, FrameNo, numFrames)

global viewBox
global focusTime
global transitTime
global frameRate

watchableFlag = 0;

props = object.getProps(FrameNo);
bbox = [props(1:2), props(4:5)];
% centroid = [props(1) + props(4)/2, props(2) + props(5)/2, props(3)];
vel = [props(7), props(8)]/frameRate;
leaveTime = [-bbox(1)/vel(1), Inf, (viewBox(3) - (bbox(1)+bbox(3)))/vel(1), ...
    (viewBox(4) - (bbox(2)+bbox(4)))/vel(2)];
leaveTime(leaveTime < 0) = Inf;
actionTime = focusTime + transitTime;
if numFrames - FrameNo >= actionTime && min(leaveTime) >= actionTime
    watchableFlag = 1;
end

end