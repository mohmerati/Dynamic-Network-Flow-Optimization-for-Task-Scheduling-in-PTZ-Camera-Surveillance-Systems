function [watchedObjectsRatio, objectsMeanDelayTime, missedObjects, ...
    inefficientInterrogationRatio, interrogationCounter, swappedTrackCounter, ...
    meanOfInterrogatedObjectsPerInterrogation] = ...
    runNetworkFlowDecisionRule(movingObjects, listOfWatchableIds, ...
    numFrames, measurementVars, surveillanceFreq, overlapIntervalCheck)

global focusTime
global transitTime
global frameRate

global viewBox
global vidXDim
global vidYDim

global deltaT
deltaT = 1/frameRate;

% mainFile = 'newNetworkFlowDecisionRule_Dataset5.avi';
% writerObj = VideoWriter(mainFile,'Uncompressed AVI');
% writerObj.FrameRate = frameRate;
% open(writerObj);

originalDims = [vidYDim, vidXDim];

Horizon = 2 * surveillanceFreq;
numRepetition = size(movingObjects, 1);
numOfObjects = size(movingObjects, 2);

taskTime = focusTime + transitTime;
samplingTime = taskTime;
newTaskGenerationTime = taskTime * surveillanceFreq;

freq = floor(numFrames/samplingTime);

watchedTracksRatio = zeros(numRepetition, freq);
meanDelayTime = zeros(numRepetition, freq);
missedTracks = zeros(numRepetition, freq);

watchedObjectsRatio = zeros(numRepetition, freq);
objectsMeanDelayTime = zeros(numRepetition, freq);
missedObjects = zeros(numRepetition, freq);
inefficientInterrogationRatio = zeros(numRepetition, 1);
interrogationCounter = zeros(numRepetition, 1);

for rep = 1:numRepetition
    disp(rep)
    tracks = initializeTracks();
    watchedObjects = initializeWatchedObjects();
    interrogatedObjectSets = initializeInterrogatedObjectSets();
    nextId = 1;
    % [Position(1:3), viewingAngle, aspectRatio, viewCenter, zoomLevel]
    blindAngle = 20 * pi/180;
    cameraProps = [[vidXDim/2, vidYDim, 500], pi/2, 1, [vidXDim/2, vidYDim/2, 0], 1, blindAngle;
               [ 700, vidYDim, 500], pi/2, 1, [700, vidYDim/2, 0], 1, blindAngle;
               [2300, vidYDim, 500], pi/2, 1, [2300,vidYDim/2, 0], 1, blindAngle];
    cameras = setCameras(cameraProps);
    watchables = listOfWatchableIds(rep,:);
    watchables = watchables(watchables > 0);
    swappedTrackCounter = 0;
    inefficientInterrogationsNum = 0;
    meanOfInterrogatedObjectsPerInterrogation = 0;
    allMissedObjects = zeros(numFrames,length(movingObjects));
    allObjectsDetected = [];
    allFalseAlarms = [];
    allFakeCentroids = [];
    rng(2,"twister")
    for frameNumber = 1:numFrames
        existingObjects = findLegitObjects(movingObjects(rep,:), frameNumber);
        [centroids, bboxes, ids, allMissedObjects, allObjectsDetected, allFalseAlarms, ...
            allFakeCentroids] = generateDetections(existingObjects, ...
            measurementVars, frameNumber, cameras, allMissedObjects, allObjectsDetected, ...
            allFalseAlarms, allFakeCentroids);
        groundTruthIds = generateGroundTruthDetections(existingObjects, ...
            cameras, frameNumber);
        [tracks, nextId, existingObjects] = updateTracks(cameras, ...
            tracks, nextId, centroids, bboxes, ids, existingObjects, ...
            frameNumber, overlapIntervalCheck);
        [movingObjects(rep,:), interrogatedObjectSets, swappedTrackCounter, ...
            interrogationCounter(rep)] = updateObjects(movingObjects(rep,:), ...
            groundTruthIds, cameras, frameNumber, interrogatedObjectSets, ...
            newTaskGenerationTime, tracks, swappedTrackCounter, interrogationCounter(rep));
        newCameras = controlCameras(cameras, tracks, frameNumber, Horizon, surveillanceFreq);
        watchedObjects = CheckForWatchedTracks(watchedObjects, tracks, ...
            existingObjects);
        if ~mod(frameNumber-1, samplingTime)
%             watchedTracksRatio(rep, floor(frameNumber/samplingTime)+1) = ...
%                 length(watchedObjects)/length(watchables);
%             meanDelayTime(rep, floor(frameNumber/samplingTime)+1) = ...
%                 calculateMeanDelayTime(watchedObjects);
%             missedTracks(rep, floor(frameNumber/samplingTime)+1) = ...
%                 length(findMissedObjects(existingObjects, watchables, ...
%                 frameNumber, watchedObjects))/length(watchables);
            [watchedObjectsRatio(rep, floor(frameNumber/samplingTime)+1), ...
                objectsMeanDelayTime(rep, floor(frameNumber/samplingTime)+1), ...
                missedObjects(rep, floor(frameNumber/samplingTime)+1)] = ...
                calculateMetrics(movingObjects(rep,:), watchables, ...
                frameNumber, existingObjects);
%             mainFrame = generateMainFrame(originalDims, existingObjects, frameNumber, ...
%                 cameras, centroids, bboxes);
%             mainFrame = displayFullTrackingResults(tracks, mainFrame, cameras);
%             writeVideo(writerObj, mainFrame);
%             imshow(mainFrame)
%             close()
        end
        if ~mod(frameNumber-1, newTaskGenerationTime)
            disp(frameNumber-1)
            meanOfInterrogatedObjectsPerInterrogation = ...
                calculateMeanObjectInterrogatedPerInterrogation(interrogatedObjectSets, ...
                interrogationCounter(rep), meanOfInterrogatedObjectsPerInterrogation);
            inefficientInterrogationsNum = inefficientInterrogationsNum + ...
                calculateSurplusInterrogations(interrogatedObjectSets);
            interrogatedObjectSets = initializeInterrogatedObjectSets();
        end
        cameras = updateCamerasStates(newCameras);
    end
    inefficientInterrogationRatio(rep) = ...
        inefficientInterrogationsNum/interrogationCounter(rep);
end

% close(writerObj);
% implay(mainFile)

end


function totalMeanInterrogatedObjectsPerInterrogation = ...
    calculateMeanObjectInterrogatedPerInterrogation(interrogatedObjectSets, ...
    totalInterrogations, meanInterrogatedSoFar)

numInterrogations = length(interrogatedObjectSets);

if ~totalInterrogations || ~numInterrogations
    totalMeanInterrogatedObjectsPerInterrogation = meanInterrogatedSoFar;
    return
end

totalInterrogationsBefore = totalInterrogations - numInterrogations;
objectsInterrogated = 0;

for i = 1:length(interrogatedObjectSets)
    objectsInterrogated = objectsInterrogated + length(interrogatedObjectSets(i).objectIds);
end

newestMeanInterrogatedObjectsPerInterrogation = objectsInterrogated/numInterrogations;

totalMeanInterrogatedObjectsPerInterrogation = (totalInterrogationsBefore * meanInterrogatedSoFar + ...
    newestMeanInterrogatedObjectsPerInterrogation * numInterrogations)/ ...
    (totalInterrogations);

end


function efficientSets = runSetCoverforEfficiencyCounting(interrogatedObjectSets)

allNeighborLists = {interrogatedObjectSets.objectIds};
combinedList = cat(2, allNeighborLists{:});
uniqueNodeIds = unique(combinedList);
selectedSetsIndices = [];
values = zeros(1,length(interrogatedObjectSets));

while ~isempty(uniqueNodeIds)
    for i = 1:length(interrogatedObjectSets)
        setIntersect = intersect(uniqueNodeIds, interrogatedObjectSets(i).objectIds);
        if ~isempty(setIntersect)
            values(i) = interrogatedObjectSets(i).weight / length(setIntersect);
        else
            values(i) = 0;
        end
    end
    newestPickedSetIndex = -1;
    while 1
        [~, maxIndex] = max(values);
        if ~ismember(maxIndex, selectedSetsIndices)
            selectedSetsIndices(end+1) = maxIndex;
            newestPickedSetIndex = maxIndex;
            break
        else
            values(maxIndex) = 0;
        end
    end
    commonElements = intersect(uniqueNodeIds, ...
        interrogatedObjectSets(newestPickedSetIndex).objectIds);
    uniqueNodeIds = uniqueNodeIds(~ismember(uniqueNodeIds, commonElements));
end

efficientSets = interrogatedObjectSets(selectedSetsIndices);

end


function inefficientInterrogationNum = calculateSurplusInterrogations( ...
    interrogatedObjectSets)

efficientInterrogations = runSetCoverforEfficiencyCounting(interrogatedObjectSets);
inefficientInterrogationNum = length(interrogatedObjectSets) - ...
    length(efficientInterrogations);

end


function [watchedObjectsRatio, objectsMeanDelayTime, missedObjectsRatio] = ...
                calculateMetrics(allObjects, watchables, frameNumber, ...
                existingObjects)

global frameRate

objectsDelay = [];

watchedObjects = 0;
for i = 1:length(allObjects)
    if allObjects{i}.recordedTime ~= -1
        watchedObjects = watchedObjects + 1;
        objectsDelay(end+1) = (allObjects{i}.recordedTime - ...
            allObjects{i}.arrivalTime)/frameRate;
    end
end

objectsMeanDelayTime = mean(objectsDelay);
watchedObjectsRatio = watchedObjects/length(watchables);

missedObjects = 0;
for i = 1:length(existingObjects)
    objProperties = existingObjects{i}.getProps(frameNumber);
    objPosition = objProperties(1:3);
    objDimensions = objProperties(4:5);
    exitFlag = checkObjectExistance([objPosition(1:2), objDimensions]);
    watchable = any(watchables == existingObjects{i}.id);
    watchedBefore = existingObjects{i}.recordedTime ~= -1;
    if exitFlag && watchable && ~watchedBefore
        missedObjects = missedObjects + 1;
    end
end

missedObjectsRatio = missedObjects/length(watchables);

end


function ids = generateGroundTruthDetections(existingObjects, cameras, frameNumber)

ids = [];

for i = 1:length(existingObjects)
    for j = 1:length(cameras)
        if cameras(j).zoomState == ZoomState.ZOOMING_IN || cameras(j).zoomState == ZoomState.ZOOMING_OUT
            continue
        else
            box = cameras(j).zoomBox;
            objProperties = existingObjects{i}.getProps(frameNumber);
            objPosition = objProperties(1:3);
            objDimensions = objProperties(4:5);
            cond1 = objPosition(1) >= box(1) && objPosition(2) >= box(2);
            cond2 = objPosition(1) + objDimensions(1) <= box(1)+box(3) && ...
                objPosition(2) + objDimensions(2) <= box(2) + box(4);
            if cond1 && cond2
                ids(end+1) = objProperties(6);
                break
            end
        end
    end
end

end


% function allObjects = updateNotDetectedObjects(allObjects, ids)
% 
% for i = 1:length(allObjects)
%     if allObjects{i}.zoomedInVisibleCount > 0
%         existenceFlag = ismember(i, ids);
%         if ~any(existenceFlag)
%             allObjects{i}.consecutiveInvisibleCount = ...
%                 allObjects{i}.consecutiveInvisibleCount + 1;
%         end
%     end
% end
% 
% end

function [allObjects, interrogatedObjectSets] = determineWatchedObjects(allObjects, ...
    frameNum, cameras, interrogatedObjectSets, newTaskGenerationTime)

global focusTime

recentlyInterrogatedObjects = [];

modulusPeriod = floor(frameNum/newTaskGenerationTime);

watchedThreshold = 0.6;
for i = 1:length(allObjects)
    watchedRatio = allObjects{i}.zoomedInVisibleCount/focusTime;
    if watchedRatio >= watchedThreshold && allObjects{i}.recordedTime == -1
        allObjects{i}.recordedTime = frameNum;
    end
    for j = 1:length(cameras)
%         if cameras(j).zoomState == ZoomState.ZOOMED_IN
        if cameras(j).zoomState == ZoomState.ZOOMED_IN && ...
            allObjects{i}.cameraSpecificZoomedInVisibleCount(j)/focusTime >= watchedThreshold && ...
            floor(allObjects{i}.recordedTime/newTaskGenerationTime) == modulusPeriod
%             objProperties = allObjects{i}.getProps(frameNum);
%             objectBox = [objProperties(1:2), objProperties(4:5)];
%             inBoxFlag = isInsideBox(objectBox, cameras(j).zoomBox);
%             if inBoxFlag && floor(allObjects{i}.recordedTime/newTaskGenerationTime) ...
%                     == modulusPeriod
            recentlyInterrogatedObjects(end+1,:) = [j, allObjects{i}.id];
%             end
        end
    end
    allObjects{i}.zoomedInVisibleCount = 0;
    allObjects{i}.cameraSpecificZoomedInVisibleCount = zeros(length(cameras),1);
end

for j = 1:length(cameras)
    if cameras(j).zoomState == ZoomState.ZOOMED_IN
        groupedObjectIds = [];
        for k = 1:size(recentlyInterrogatedObjects, 1)
            if recentlyInterrogatedObjects(k, 1) == j
                groupedObjectIds(end+1) = recentlyInterrogatedObjects(k, 2);
            end
        end
        interrogatedObjectSet = struct('objectIds', groupedObjectIds, ...
            'weight', exp(length(groupedObjectIds)));
        interrogatedObjectSets(end+1) = interrogatedObjectSet;
    end
end

end


function [allObjects, swappedTrackCounter] = updateDetectedObjects(allObjects, ids, cameras, ...
    frameNumber, tracks, swappedTrackCounter)

for i = 1:length(ids)
    allObjects{ids(i)}.age = allObjects{ids(i)}.age + 1;
    allObjects{ids(i)}.totalVisibleCount = ...
        allObjects{ids(i)}.totalVisibleCount + 1;
    allObjects{ids(i)}.consecutiveInvisibleCount = 0;
%     trackIdIndex = [tracks(:).objectID] == ids(i);
%     if allObjects{ids(i)}.trackId ~= -1 && allObjects{ids(i)}.trackId ~= tracks(trackIdIndex).id
%         swappedTrackCounter = swappedTrackCounter + 1;
%     end

%     for j = 1:length(cameras)
%         if cameras(j).zoomState == ZoomState.ZOOMED_IN
%             objProperties = allObjects{ids(i)}.getProps(frameNumber);
%             objectBox = [objProperties(1:2),objProperties(4:5)];
%             inBoxFlag = isInsideBox(objectBox, cameras(j).zoomBox);
%             if inBoxFlag
%                 allObjects{ids(i)}.zoomedInVisibleCount = ...
%                     allObjects{ids(i)}.zoomedInVisibleCount + 1;
%                 break
%             end
%         end
%     end

    insideZoomedInView = zeros(1,length(cameras));
    for j = 1:length(cameras)
        if cameras(j).zoomState == ZoomState.ZOOMED_IN
            objProperties = allObjects{ids(i)}.getProps(frameNumber);
            objectBox = [objProperties(1:2), objProperties(4:5)];
            inBoxFlag = isInsideBox(objectBox, cameras(j).zoomBox);
            insideZoomedInView(j) = inBoxFlag;
            if inBoxFlag
                allObjects{ids(i)}.cameraSpecificZoomedInVisibleCount(j) = ...
                    allObjects{ids(i)}.cameraSpecificZoomedInVisibleCount(j) + 1;
            end
        end
    end
    if any(insideZoomedInView)
        allObjects{ids(i)}.zoomedInVisibleCount = ...
                    allObjects{ids(i)}.zoomedInVisibleCount + 1;
    end
end

end


function [allObjects, interrogatedObjectSets, swappedTrackCounter, ...
    interrogationCounter] = updateObjects(allObjects, ids, cameras, ...
    frameNumber, interrogatedObjectSets, newTaskGenerationTime, tracks, ...
    swappedTrackCounter, interrogationCounter)

global focusTime

ids = ids(ids ~= -1);

[allObjects, swappedTrackCounter] = updateDetectedObjects(allObjects, ids, ...
    cameras, frameNumber, tracks, swappedTrackCounter);
% allObjects = updateNotDetectedObjects(allObjects, ids);
% allObjects = updateLostObjects(allObjects, ids);

cameraZoomStates = zeros(1,length(cameras));
for j = 1:length(cameras)
    if cameras(j).zoomState == ZoomState.ZOOMED_IN && ...
        cameras(j).stateCounter == focusTime
        cameraZoomStates(j) = 1;
    end
end

interrogationCounter = interrogationCounter + sum(cameraZoomStates);

for j = 1:length(cameras)
    if cameras(j).zoomState == ZoomState.ZOOMED_IN && ...
        cameras(j).stateCounter == focusTime
        [allObjects, interrogatedObjectSets] = determineWatchedObjects( ...
            allObjects, frameNumber,cameras, interrogatedObjectSets, ...
            newTaskGenerationTime);
        break
    end
end

end


function missedObjectsIds = findMissedObjects(existingObjects, listOfWatchableIds, ...
    frameNumber, watchedObjects)

missedObjectsIds = [];

for i = 1:length(existingObjects)
    objProperties = existingObjects{i}.getProps(frameNumber);
    objPosition = objProperties(1:3);
    objDimensions = objProperties(4:5);
    exitFlag = checkObjectExistance([objPosition(1:2), objDimensions]);
    watchable = any(listOfWatchableIds == existingObjects{i}.id);
    watchedBefore = any([watchedObjects(:).id] == existingObjects{i}.id);
    if exitFlag && watchable && ~watchedBefore
        missedObjectsIds(end+1) = existingObjects{i}.id;
    end
end

end

function exitFlag = checkObjectExistance(bbox)

global viewBox

exitFlag = 0;
initialX = bbox(1);
initialY = bbox(2);
endX = bbox(1) + bbox(3);
endY = bbox(2) + bbox(4);

if initialX < 0 || endX > viewBox(3) || endY > viewBox(4)
    exitFlag = 1;
end

end


function interrogatedSets = initializeInterrogatedObjectSets()


% I'm using coveredTrackIds since the runSetCoverGreedyAlgo requires this
% naming. It's actually the list of covered object Ids
interrogatedSets = struct(...
    'objectIds', {}, ...
    'weight', {});

end


function watchedObjects = initializeWatchedObjects()

watchedObjects = struct(...
        'id', {}, ...
        'arrivalTime', {}, ...
        'recordedTime', {});

end


function meanDelay = calculateMeanDelayTime(watchedObjectsIds)

global frameRate

objectsDelay = [];

for i = 1:length(watchedObjectsIds)
     objectsDelay(end+1) = (watchedObjectsIds(i).recordedTime - ...
        watchedObjectsIds(i).arrivalTime)/frameRate;
end

meanDelay = mean(objectsDelay);

end

function newCameras = updateCamerasStates(cameras)

for i = 1:length(cameras)
    newCameras(i) = cameras(i).updateCameraState();
end

end


function  watchableFlag = isWatchable(object,frameNo,numFrames)

global viewBox
global focusTime
global transitTime
global frameRate

watchableFlag = 0;

props = object.getProps(frameNo);
centroid = [props(1) + props(4)/2, props(2) + props(5)/2, props(3)];
vel = [props(7), props(8)]/frameRate;
leaveTime = [-centroid(1)/vel(1), Inf, (viewBox(3) - centroid(1))/vel(1), ...
    (viewBox(4) - centroid(2))/vel(2)];
leaveTime(leaveTime < 0) = Inf;

actionTime = focusTime + transitTime;
if numFrames - frameNo >= actionTime && min(leaveTime) >= actionTime
    watchableFlag = 1;
end

end


function watchedObjects = CheckForWatchedTracks(watchedObjects, tracks, objects)

if isempty(tracks)
    return
end

tracksToEval = struct([]);

if isempty(watchedObjects)
    tracksToEval = tracks;
else
    newTracksIdx = ismember([tracks(:).objectID], [watchedObjects(:).id]);
    tracksToEval = tracks(~newTracksIdx);
end

for i = 1:length(tracksToEval)
    if tracksToEval(i).watched
        newWatchedObject = tracksToEval(i).objectID;
        % detect the false alarms
        if newWatchedObject ~= -1
            objectFoundIndex = cellfun(@(x) isequal(x.id, newWatchedObject) ...
                , objects);
            watchedObject = struct( ...
                'id', objects{objectFoundIndex}.id, ...
                'arrivalTime', objects{objectFoundIndex}.arrivalTime, ...
                'recordedTime', tracksToEval(i).recordedTime);
            watchedObjects(end+1) = watchedObject;
        end
    end
end

end


function cameras = setCameras(camerasPropersties)

colors = ["magenta", "cyan", "blue"];
for i=1:size(camerasPropersties,1)
    camProps = camerasPropersties(i,:);
    cameras(i) = Camera(colors(i), i, camProps(1:3), camProps(4), ...
        camProps(5),camProps(6:8), camProps(9), camProps(end));
end

end


function [tracks, nextId, objects] = updateTracks(cameras, tracks, nextId, ...
    centroids, bboxes, ids, objects, frameNum, overlapIntervalCheck)

global focusTime

if isempty(tracks) && isempty(centroids)
    return
end

tracks = predictNewLocationsOfTracks(tracks);
[assignments, unassignedTracks, unassignedDetections] = ...
            detectionToTrackAssignment(tracks, centroids, cameras);
tracks = updateAssignedTracks(assignments, centroids, tracks, bboxes, cameras ,ids);
tracks = updateUnassignedTracks(tracks, unassignedTracks, cameras);
tracks = deleteLostTracks(tracks);
[tracks, nextId] = createNewTracks(tracks, unassignedDetections, centroids, ...
    bboxes, nextId, ids);

for j = 1:length(cameras)
    if cameras(j).zoomState == ZoomState.ZOOMED_IN && ...
        cameras(j).stateCounter == focusTime
        [tracks, objects] = determineWatchedTracks(tracks, objects, frameNum);
        break
    end
end

if ~overlapIntervalCheck
    return
else
    overlapThreshold = 0.3;
    if isempty(tracks)
        return
    elseif ~mod(frameNum, overlapIntervalCheck)
        newZoomedInFlags = findTracksUnderInterrogation(tracks, cameras);
        previousOverlapFlags = [tracks(:).overlapFlag];
        tracks = checkOverlappingTracks(tracks, overlapThreshold);
        newOverlapFlags = [tracks(:).overlapFlag];
        tracksToBeReinterrogatedInds = (~newOverlapFlags) & ...
            previousOverlapFlags & ~newZoomedInFlags;
        for i = 1:length(tracks)
            if tracksToBeReinterrogatedInds(i)
                tracks(i).watched = 0;
            end
        end
    end
end

end


function zoomedInFlag = findTracksUnderInterrogation(tracks, cameras)

zoomedInFlag = zeros(1, length(tracks));
for i = 1:length(tracks)
    for j = 1:length(cameras)
        if cameras(j).zoomState == ZoomState.ZOOMED_IN && ...
                isInsideBox(tracks(i).bbox, cameras(j).zoomBox)
            zoomedInFlag(i) = 1;
            break
        end
    end
end

end


function tracks = checkOverlappingTracks(tracks, threshold)

trackNum = length(tracks);
overlapList = zeros(1, trackNum);
for i = 1:trackNum
    if ~overlapList(i)
        for j = i + 1:trackNum
            iou = calculateIntersectionOverUnion(tracks(i).bbox, tracks(j).bbox);
            if iou > threshold
                overlapList(i) = 1;
                overlapList(j) = 1;
                break
            end
        end
    end
end

for k = 1:trackNum
    tracks(k).overlapFlag = overlapList(k);
end

end


function iou = calculateIntersectionOverUnion(box1, box2)

x1 = max(box1(1), box2(1));
y1 = max(box1(2), box2(2));
x2 = min(box1(1) + box1(3), box2(1) + box2(3));
y2 = min(box1(2) + box1(4), box2(2) + box2(4));
intersectionArea = max(0, x2 - x1 + 1) * max(0, y2 - y1 + 1);

box1Area = (box1(3) + 1) * (box1(4) + 1);
box2Area = (box2(3) + 1) * (box2(4) + 1);

iou = intersectionArea / (box1Area + box2Area - intersectionArea);

end

% It's working since all the cameras are synchronized.
function [tracks, objects] = determineWatchedTracks(tracks, objects, frameNum)

global focusTime

watchedThreshold = 0.6;
for i = 1:length(tracks)
    watchedRatio = tracks(i).zoomedInVisibleCount/focusTime;
    if watchedRatio >= watchedThreshold
        tracks(i).watched = 1;
        objectFoundIndex = cellfun(@(x) isequal(x.id, tracks(i).objectID), objects);
        if any(objectFoundIndex) && tracks(i).recordedTime == -1
            tracks(i).recordedTime = frameNum;
        end
    end
    tracks(i).zoomedInVisibleCount = 0;
end

end


function [vals, trackNodeValues, finalValues, trackNodeTrackCountValue] = ...
    generateValueVector(fixedNodes, trackNodes, cameras, numPeriods, surveilFreq)

global transitTime
global focusTime

taskTime = focusTime + transitTime;

numTrackLocations = size(trackNodes,1);
numFixedLocations = size(fixedNodes,1);
numCameras = length(cameras);

M = numCameras * numTrackLocations * numPeriods + ...
    numCameras * numFixedLocations * numPeriods + ...
    numTrackLocations * numPeriods + ...
    numFixedLocations * numPeriods + ...
    numFixedLocations * ceil(numPeriods/surveilFreq);

vals = zeros(M,1);

for t = 1:numPeriods
    for i = 1:numFixedLocations
        for j = 1:numCameras
            viewCenter = fixedNodes(i,t).viewCenter;
            vecB = [0, 1, 0];
            degree = acos((cameras(j).position - viewCenter) * vecB'/ ...
                norm(cameras(i).position - viewCenter) * norm(vecB));
            varIndex = numCameras * numTrackLocations * numPeriods + ...
                (j - 1) * numFixedLocations * numPeriods + ...
                (t - 1) * numFixedLocations + i;
            if degree <= pi/6
                vals(varIndex) = fixedNodes(i,t).numTracksInside + 3;
            elseif degree > pi/6 && degree <= pi/3
                vals(varIndex) = fixedNodes(i,t).numTracksInside + 2;
            elseif degree > pi/3 && degree <= pi/2
                vals(varIndex) = fixedNodes(i,t).numTracksInside + 1;
            else
                vals(varIndex) = fixedNodes(i,t).numTracksInside;
            end
        end
    end
end

baseValue = max(vals);

maxValue = numTrackLocations * numPeriods;
trackNodeValues = zeros(numTrackLocations, numPeriods);
finalValues = zeros(numTrackLocations, numPeriods, numCameras);
trackNodeTrackCountValue = zeros(numTrackLocations, numPeriods, numCameras);
camerasAngleConstraints = ones(numTrackLocations, numPeriods, numCameras);

if ~isempty(trackNodes)
    for t = 1:numPeriods
        featureValues = arrayfun(@(x) x.leaveTime, trackNodes(:, t));
        [~, sortedIndices] = sort(featureValues, 'ascend');
        for i = 1:length(sortedIndices)
            if trackNodes(sortedIndices(i), t).leaveTime == -1 || ...
                trackNodes(sortedIndices(i), t).leaveTime == inf
                trackNodeValues(sortedIndices(i), t) = -1;
            elseif trackNodes(sortedIndices(i), t).leaveTime < taskTime || ...
                    trackNodes(sortedIndices(i), t).watched == 1
                trackNodeValues(sortedIndices(i), t) = 0;
            else
                trackNodeValues(sortedIndices(i), t) = maxValue - ...
                    (t-1) * numTrackLocations - i + 1;
            end
        end
    end

    trackNodeCameraValues = zeros(numTrackLocations, numPeriods, numCameras);    
    
    for i = 1:numTrackLocations
        for j = 1:numPeriods
            track = trackNodes(i,j);
            if track.leaveTime < taskTime || track.watched == 1
                continue
            else
                trackCenter = [track.bbox(1) + track.bbox(3)/2, ...
                    track.bbox(2) + track.bbox(4)/2, track.bbox(5)];
                vel = [track.velocity, 0];
                for k = 1:numCameras
                    num = (cameras(k).viewCenter - trackCenter) * vel';
                    den = norm(cameras(k).viewCenter - trackCenter) * norm(vel);
                    degree = acos(num/den);
                    if degree <= pi/6
                        trackNodeCameraValues(i,j,k) = 3;
                    elseif degree > pi/6 && degree <= pi/3
                        trackNodeCameraValues(i,j,k) = 2;
                    elseif degree > pi/3 && degree <= pi/2
                        trackNodeCameraValues(i,j,k) = 1;
                    else
                        trackNodeCameraValues(i,j,k) = 0;
                    end
                end
            end
        end
    end

    for i = 1:numTrackLocations
        for t = 1:numPeriods
            mainTrackBox = trackNodes(i,t).bbox;
            mainTrackCenter = [mainTrackBox(1:2) + mainTrackBox(3:4)/2, mainTrackBox(5)];
            for j = 1:numCameras
                zoomLevel = 10;
                mainTrackCameraBox = cameras(j).findZoomBox(mainTrackCenter, zoomLevel);
                for l = 1:numTrackLocations
                    trackBox = trackNodes(l, t).bbox;
                    inBoxFlag = isInsideBox(trackBox, mainTrackCameraBox);
                    if inBoxFlag && ~trackNodes(l, t).watched
                        trackNodeTrackCountValue(i, t, j) = ...
                            trackNodeTrackCountValue(i, t, j) + 1;
                    end
                end
            end
        end
    end

    for i = 1:numTrackLocations
        for t = 1:numPeriods
            mainTrackBox = trackNodes(i,t).bbox;
            mainTrackCenter = [mainTrackBox(1:2) + mainTrackBox(3:4)/2, mainTrackBox(5)];
            for j = 1:numCameras
                [~, beta] = cameras(j).findAngles(mainTrackCenter);
%                 angleDegree = beta * 180/pi;
                if pi/2 + beta <= cameras(j).blindAngle
%                     disp("damn!")
                    camerasAngleConstraints(i,t,j) = 0;
                end
            end
        end
    end

    for i = 1:numCameras
        for t = 1:numPeriods
            for j = 1:numTrackLocations
                varIndex = (i - 1) * numTrackLocations * numPeriods + ...
                        (t - 1) * numTrackLocations + j;
                if trackNodeValues(j,t) > 0
                    vals(varIndex) = ((trackNodeValues(j,t) + trackNodeCameraValues(j,t,i) ...
                        + baseValue) + trackNodeTrackCountValue(j,t,i)) * ...
                        camerasAngleConstraints(j,t,i);
                    finalValues(j,t,i) = vals(varIndex);
                elseif trackNodeValues(j,t) < 0
                    vals(varIndex) = trackNodeValues(j,t);
                    finalValues(j,t,i) = vals(varIndex);
                end
            end
        end
    end
end

end


function [Aeq, beq, lb, ub] = generateConstraints(cameras, trackNodes, ...
    numPeriods, surveilFreq)

numCameras = length(cameras);
numTrackLocations = size(trackNodes,1);
numFixedLocations = length(cameras);

numDemandNodes = ceil(numPeriods/surveilFreq) * numFixedLocations;

M = numCameras * numTrackLocations * numPeriods + ...
    numCameras * numFixedLocations * numPeriods + ...
    numTrackLocations * numPeriods + ...
    numFixedLocations * numPeriods + ...
    numFixedLocations * ceil(numPeriods/surveilFreq);

N = (numCameras + numTrackLocations + numFixedLocations) * numPeriods ...
    + numDemandNodes + 1;

Aeq = zeros(N, M);
beq = zeros(N, 1);
lb = zeros(M,1);
ub = ones(M,1);

for i = 1:numCameras
    for t = 1:numPeriods
        nodeIndex = (i - 1) * numPeriods + t;
        % Outgoing flows to track locations
        for j = 1:numTrackLocations
            varIndex = (i - 1) * numTrackLocations * numPeriods + (t - 1) * numTrackLocations + j;
            Aeq(nodeIndex, varIndex) = 1;
        end
        % Outgoing flows to fixed locations
        for j = 1:numFixedLocations
            varIndex = numCameras * numTrackLocations * numPeriods + ...
                (i - 1) * numFixedLocations * numPeriods + (t - 1) * numFixedLocations + j;
            Aeq(nodeIndex, varIndex) = 1;
        end
        beq(nodeIndex) = 1;
    end
end

for i = 1:numTrackLocations
    for t = 1:numPeriods
        nodeIndex = numCameras * numPeriods + (i - 1) * numPeriods + t;
        % Incoming flows from camera nodes
        for j = 1:numCameras
            varIndex = (j - 1) * numTrackLocations * numPeriods + ...
                (t - 1) * numTrackLocations + i;
            Aeq(nodeIndex, varIndex) = -1;
        end
        % Incoming flow from the same node in the previous period
        if t > 1
            varIndex = numCameras * numPeriods * (numFixedLocations + numTrackLocations) + ...
                (i - 1) * (numPeriods - 1) + t - 1;
            Aeq(nodeIndex, varIndex) = -1;
        end
        % Outgoing flow to the same node in the next period (or to the sink node for the last period)
        if t < numPeriods
            varIndex = numCameras * numPeriods * (numFixedLocations + numTrackLocations) + ...
                (i - 1) * (numPeriods - 1) + t;
            Aeq(nodeIndex, varIndex) = 1;
        else
            % Last period's track location nodes flow to the sink node
            varIndex = numCameras * numTrackLocations * numPeriods + ... % Skip camera-track flows
               numCameras * numFixedLocations * numPeriods + ... % Skip camera-fixed flows
               numTrackLocations * (numPeriods - 1) + ... % Skip track-track flows
               numFixedLocations * numPeriods + ... % Skip fixed-demand flows
               i; % Offset for the specific track location node
            Aeq(nodeIndex, varIndex) = 1;
        end
    end
end

% Fixed Location Nodes
for i = 1:numFixedLocations
    for t = 1:numPeriods
        nodeIndex = (numCameras + numTrackLocations) * numPeriods + (i - 1) * numPeriods + t;
        % Incoming flows from camera nodes
        for j = 1:numCameras
            varIndex = numCameras * numTrackLocations * numPeriods + ...
                (j - 1) * numFixedLocations * numPeriods + (t - 1) * numFixedLocations + i;
            Aeq(nodeIndex, varIndex) = -1;
        end
        varIndex = numCameras * (numTrackLocations + numFixedLocations) * numPeriods + ...
            numTrackLocations * (numPeriods - 1) + (i - 1) * numPeriods + t;
        % Outgoing flow to demand nodes
        Aeq(nodeIndex, varIndex) = 1;
    end
end

% Demand Nodes
for i = 1:numFixedLocations
    for k = 1:ceil(numPeriods/surveilFreq)
        nodeIndex = numCameras * numPeriods + numTrackLocations * numPeriods + ...
            numFixedLocations * numPeriods + (i-1) * ceil(numPeriods/surveilFreq) + k;
        % Incoming flows from fixed location nodes
        for t = surveilFreq*k-(surveilFreq-1):surveilFreq*k
            varIndex = numCameras * (numTrackLocations + numFixedLocations) * numPeriods + ...
                numTrackLocations * (numPeriods - 1) + (i - 1) * numPeriods + t;
            Aeq(nodeIndex, varIndex) = -1;
        end
        % Outgoing flows from demand nodes to sink node
        varIndex = numCameras * (numTrackLocations + numFixedLocations) * numPeriods + ...
            numTrackLocations * numPeriods + numFixedLocations * numPeriods + ...
            (i-1) * ceil(numPeriods/surveilFreq) + k;
        Aeq(nodeIndex, varIndex) = 1;
        ub(varIndex) = Inf;
        % Demand of 1 at each demand node
        beq(nodeIndex) = -1;
    end
end

% Sink Node
sinkNodeIndex = N;
% Incoming flows from demand nodes
for i = 1:numDemandNodes
    varIndex = M - numDemandNodes + i;
    Aeq(sinkNodeIndex, varIndex) = -1;
end
% Incoming flows from last period's track location nodes
for i = 1:numTrackLocations
    varIndex = M - numDemandNodes - numTrackLocations + i;
    Aeq(sinkNodeIndex, varIndex) = -1;
end

beq(sinkNodeIndex) = -(numPeriods*numCameras - ceil(numPeriods/surveilFreq) * numFixedLocations);

end


function [trackNodes, fixedNodes] = generatePredictedNodes(originalTracks, ...
    cameras, T)

global measurementVars
global processVars

global transitTime
global focusTime
global viewBox

nodeFixedStruct = struct('viewCenter',{}, ...
    'numTracksInside', {});

nodeTrackStruct = struct('id', {}, ...
    'velocity', {}, ...
    'bbox', {}, ...
    'leaveTime', {}, ...
    'watched', {});

numFixedLocations = length(cameras);

tempFilters = cell(1,length(originalTracks));

for i = 1:length(originalTracks)
    bbox = originalTracks(i).bbox;
    centroid = [bbox(1) + bbox(3)/2, bbox(2) + bbox(4)/2];
%     kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
%         centroid, [1e-3, 1e-5], [0.5, 0.1]/30, 0.1/30);
    kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
        centroid, max(measurementVars) * [1, 1e4], ...
        [max(processVars(1), processVars(3)), max(processVars(2), processVars(4))], ...
        max(measurementVars));
    kalmanFilter.State = originalTracks(i).kalmanFilter.State;
    kalmanFilter.StateCovariance = originalTracks(i).kalmanFilter.StateCovariance;
    tempFilters{i} = kalmanFilter;
end

trackNodes = repmat(nodeTrackStruct, length(tempFilters), T);
fixedNodes = repmat(nodeFixedStruct, numFixedLocations, T);

taskTime = focusTime + transitTime;

for t = 1:T
    for j = 1:numFixedLocations
        viewCenter = cameras(j).initialViewCntr;
        fixedNodes(j, t).viewCenter = viewCenter;
        fixedNodes(j, t).numTracksInside = 0;
    end
end

for i = 1:length(tempFilters)
    t = 1;
    for numFrame = 1:(taskTime * T) - 1
        bbox = originalTracks(i).bbox;
        predictedCentroid = predict(tempFilters{i});
        if numFrame == transitTime + (t - 1) * taskTime
            predictedCentroid = predictedCentroid - bbox(3:4) / 2;
            bbox = [predictedCentroid, bbox(3:4), bbox(5)];
            for j = 1:length(numFixedLocations)
                viewCenter = cameras(j).initialViewCntr;
                fixedNodes(j, t).viewCenter = viewCenter;
                zoomLevel = cameras(j).zoomLevel;
                box = cameras(j).findZoomBox(viewCenter, zoomLevel);
                if isInsideBox(bbox, box)
                    fixedNodes(j, t).numTracksInside = ...
                        fixedNodes(j, t).numTracksInside + 1;
                end
            end
            kalFilter = tempFilters{i};
            vel = [kalFilter.State(2), kalFilter.State(4)];
%             centroid = [bbox(1) + bbox(3)/2, bbox(2) + bbox(4)/2, bbox(5)];
            leaveTime = [-bbox(1)/vel(1), Inf, (viewBox(3) - (bbox(1)+bbox(3)))/vel(1), ...
                (viewBox(4) - (bbox(2)+bbox(4)))/vel(2)];
%             leaveTime = [-centroid(1)/vel(1), Inf, (viewBox(3) - centroid(1))/vel(1), ...
%                     (viewBox(4) - centroid(2))/vel(2)];
            leaveTime(leaveTime < 0) = Inf;
            minLeaveTime = min(leaveTime);
            cond1 = bbox(1) >= viewBox(1) && bbox(2) >= viewBox(2);
            cond2 = bbox(1) + bbox(3) <= viewBox(1)+viewBox(3) && ...
                bbox(2) + bbox(4) <= viewBox(2)+viewBox(4);
            if ~(cond1 && cond2)
                minLeaveTime = -1;
            end
            trackNodes(i, t) = struct(...
                'id', originalTracks(i).id, ...
                'velocity', vel, ... 
                'bbox', bbox, ...
                'leaveTime', minLeaveTime, ...
                'watched', originalTracks(i).watched);
            t = t + 1;
        end
    end
end

tracksNotbeDeleted = [];
for i = 1:length(originalTracks)
    if ~originalTracks(i).watched
        tracksNotbeDeleted(end+1) = i;
    end
end

trackNodes = trackNodes(tracksNotbeDeleted,:);

end

function inBoxFlag = isInsideBox(bbox, box)

cond1 = bbox(1) >= box(1) && bbox(2) >= box(2);
cond2 = bbox(1) + bbox(3) <= box(1) + box(3) && ...
            bbox(2)+bbox(4) <= box(2) + box(4);

inBoxFlag = cond1 && cond2;

end


function jobBuffers = createCameraJobs(decisions, trackNodes, fixedNodes, ...
    cameras, numPeriods, trackNodeTrackCountValues, vals)

numTrackLocations = size(trackNodes,1);
numFixedLocations = size(fixedNodes,1);
numCameras = length(cameras);

jobBufferStruct = struct('viewCenter', {}, ...
    'zoomLevel', {}, ...
    'time', {}, ...
    'targetId', {}, ...
    'numCoveredTracks', {}, ...
    'value', {});

jobBuffers = repmat(jobBufferStruct, numCameras, numPeriods);

for t = 1:numPeriods
    for i = 1:numFixedLocations
        for j = 1:numCameras
            varIndex = numCameras * numTrackLocations * numPeriods + ...
                (j - 1) * numFixedLocations * numPeriods + ...
                (t - 1) * numFixedLocations + i;
            targetId = -1;
            if decisions(varIndex) == 1
                job = struct('viewCenter', fixedNodes(i,t).viewCenter, ...
                        'zoomLevel', 1, ...
                        'time', t, ...
                        'targetId', targetId, ...
                        'numCoveredTracks', fixedNodes(i,t).numTracksInside, ...
                        'value', vals(varIndex));
                jobBuffers(j,t) = job;
            end
        end
    end
end

for i = 1:numCameras
    for t = 1:numPeriods
        for j = 1:numTrackLocations
            varIndex = (i - 1) * numTrackLocations * numPeriods + ...
                (t - 1) * numTrackLocations + j;
            if decisions(varIndex) == 1
                bbox = trackNodes(j,t).bbox;
                viewCenter = [bbox(1)+bbox(3)/2, bbox(2)+bbox(4)/2, bbox(5)];
                job = struct('viewCenter', viewCenter, ...
                    'zoomLevel', 10, ...
                    'time', t, ...
                    'targetId', trackNodes(j,t).id, ...
                    'numCoveredTracks', trackNodeTrackCountValues(j,t,i), ...
                    'value', vals(varIndex));
                jobBuffers(i,t) = job;
            end
        end
    end
end

end


function [newCameras, interrogationCounter] = controlCameras(cameras, tracks, numFrame, T, ...
    surveillanceFreq, interrogationCounter)

global focusTime
global transitTime
global frameRate

newCameras = cameras;

taskTime = focusTime + transitTime;
newTaskGenTime = taskTime * ceil(T/2);

if ~mod(numFrame-1, newTaskGenTime) || numFrame == 1
    [trackNodes, fixedNodes] = generatePredictedNodes(tracks, cameras, T);
    [Aeq, beq, lb, ub] = generateConstraints(cameras, trackNodes, T, surveillanceFreq);
    [f, trackNodeValues, finalValues, trackNodeTrackCountValue] = ...
        generateValueVector(fixedNodes, trackNodes, cameras, T, surveillanceFreq);
    % Not part of the code
    if ~isempty(trackNodes)
        tempTrackNode = trackNodes(:,1);
        leaveTimes = [tempTrackNode.leaveTime];
        [~, sortedIndices] = sort(leaveTimes);
        sortedTrackNodes = trackNodes(sortedIndices);
        maxFinalValues = finalValues(sortedIndices,:,:);
        sortedTrackCovers = trackNodeTrackCountValue(sortedIndices,:,:);
    end
    x = linprog(-f,[],[],Aeq,beq,lb,ub);
    jobBuffers = createCameraJobs(x, trackNodes, fixedNodes, cameras, T, ...
        trackNodeTrackCountValue, f);
    for i = 1:length(cameras)
        newCameras(i) = cameras(i).createJobBuffer(jobBuffers(i,:));
    end
end
if ~mod(numFrame-1, taskTime)
    for i = 1:length(cameras)
        [newCameras(i), job] = newCameras(i).loadNextJob();
        % Change to Zoom-out process
        if isequal(job.viewCenter, newCameras(i).viewCenter)
            continue
        elseif ~isequal(job.viewCenter, newCameras(i).viewCenter) && job.targetId == -1
            newCameras(i) = newCameras(i).changeToZoomOutMode(job.viewCenter);
        % Change to Zoom-in process
        else
            newCameras(i) = newCameras(i).changeToZoomInMode(job.viewCenter, ...
                job.zoomLevel, job.targetId, [], []);
        end
    end

else
    for i = 1:length(cameras)
        if cameras(i).zoomState == ZoomState.ZOOMED_IN && cameras(i).stateCounter < focusTime
            targetTrackIndex = [tracks(:).id] == cameras(i).targetTrackId;
            if ~all(targetTrackIndex(:) == 0)
                trackBox = tracks(targetTrackIndex).bbox;
                trackCenter = [trackBox(1) + trackBox(3)/2, trackBox(2) + trackBox(4)/2, 0];
                newCameras(i) = cameras(i).changeZoomedInCenter(trackCenter);
            end
        end
    end
end

end


function nodeTrackSets = generateSets(trackNodes, middleCamera)

numTrackLocations = size(trackNodes, 1);
numPeriods = size(trackNodes, 2);
nodeTrackSets = trackNodes;

for i = 1:numTrackLocations
    for t = 1:numPeriods
        nodeTrackSets(i,t).coveredTrackIds = [];
        nodeTrackSets(i,t).weight = 0;
    end
end

tracksNotTobeDeleted = [];

for i = 1:numTrackLocations
    t = 1;
    if ~nodeTrackSets(i,t).watched
        tracksNotTobeDeleted(end+1) = i;
    end
end

nodeTrackSets = nodeTrackSets(tracksNotTobeDeleted,:);

velocityThreshold = 0.2;

for i = 1:size(nodeTrackSets,1)
    t = 1;
    mainTrackBox = nodeTrackSets(i,t).bbox;
    mainTrackCenter = [mainTrackBox(1:2) + mainTrackBox(3:4)/2, mainTrackBox(5)];
    zoomLevel = 10;
    mainTrackCameraBox = middleCamera.findZoomBox(mainTrackCenter, zoomLevel);
    for l = 1:size(nodeTrackSets,1)
        inBoxFlag = isInsideBox(nodeTrackSets(l,t).bbox, mainTrackCameraBox);
        velocityDifference = norm(nodeTrackSets(l,t).velocity - ...
            nodeTrackSets(i,t).velocity);
        if inBoxFlag && nodeTrackSets(l,t).watched ~= 1 && ...
                velocityDifference <= velocityThreshold
            nodeTrackSets(i,t).coveredTrackIds = ...
                [nodeTrackSets(i,t).coveredTrackIds, nodeTrackSets(l, t).id];
            nodeTrackSets(i,t).weight = ...
                nodeTrackSets(i,t).weight + 1;
        end
    end
end

for i = 1:size(nodeTrackSets,1)
    for t = 1:numPeriods
        nodeTrackSets(i,t).coveredTrackIds = nodeTrackSets(i,1).coveredTrackIds;
        nodeTrackSets(i,t).weight = exp(nodeTrackSets(i,1).weight);
    end
end

end


function remainingNodeTrackSets = runSetCoverGreedyAlgo(nodeTrackSets)

tempTrackSets = nodeTrackSets(:,1);

allNeighborLists = {tempTrackSets.coveredTrackIds};
combinedList = cat(2, allNeighborLists{:});
uniqueNodeIds = unique(combinedList);
selectedSetsIndices = [];
values = zeros(1,length(tempTrackSets));

while ~isempty(uniqueNodeIds)
    for i = 1:length(tempTrackSets)
        setIntersect = intersect(uniqueNodeIds, tempTrackSets(i).coveredTrackIds);
        if ~isempty(setIntersect)
            values(i) = tempTrackSets(i).weight / length(setIntersect);
        else
            values(i) = 0;
        end
    end
    newestPickedSetIndex = -1;
    while 1
        [~, maxIndex] = max(values);
        if ~ismember(maxIndex, selectedSetsIndices)
            selectedSetsIndices(end+1) = maxIndex;
            newestPickedSetIndex = maxIndex;
            break
        else
            values(maxIndex) = 0;
        end
    end
    commonElements = intersect(uniqueNodeIds, ...
        tempTrackSets(newestPickedSetIndex).coveredTrackIds);
    uniqueNodeIds = uniqueNodeIds(~ismember(uniqueNodeIds, commonElements));
end

remainingNodeTrackSets = nodeTrackSets(selectedSetsIndices, :);

end


function newTrackNodes = generateTrackSets(trackNodes, cameras)

middleCameraIndex = 3;
nodeTrackSets = generateSets(trackNodes, cameras(middleCameraIndex));
newTrackNodes = runSetCoverGreedyAlgo(nodeTrackSets);

end


function newZoomCenter = predictZoomCenter(track)

global transitTime

for i=1:transitTime
    bbox = track.bbox;
    predictedCentroid = predict(track.kalmanFilter);
    predictedCentroid = predictedCentroid - bbox(3:4) / 2;
    bbox = [predictedCentroid, bbox(3:4), bbox(5)];
end

targetPredictedCentroids = [bbox(1:2) + bbox(3:4)/2, bbox(5)];
newZoomCenter = targetPredictedCentroids;

end


function existingObjects = findLegitObjects(objects, frameNumber)

existingObjects = objects;
deleteIdx = [];
numObjects = length(objects);

for j = 1:numObjects
    if isempty(objects{j})
        deleteIdx(end+1) = j;
    else
        temp = objects{j}.getProps(frameNumber);
        if temp(1) == -inf
            deleteIdx(end+1) = j;
        end
    end
end

existingObjects(deleteIdx) = [];

end


function [centroids, bboxes ,ids, allMissedObjects, allObjectsDetected, allFalseAlarms, ...
    allFakeCentroids] = generateDetections(objects, ...
    mearusementVars, frameNumber, cameras, allMissedObjects, allObjectsDetected, ...
    allFalseAlarms, allFakeCentroids)

global viewBox

centroids = [[], [], []];
bboxes = [[], [], [], [], []];
ids = [];

missDetectionProb = 0.1;
allObjectsDetected(end+1) = length(objects);
missed = binornd(1,missDetectionProb,1,length(objects));
for i = 1:length(objects)
    objProperties = objects{i}.getProps(frameNumber);
    objPosition = objProperties(1:3);
%     objDimensions = objProperties(4:5);
    detection = DetectedObject(objProperties, mearusementVars); 
    detectedBBox = detection.getBBox();
    if missed(i)
        continue
    else
        if any(checkTrackInCameraViews(detectedBBox,cameras))
            bboxes(end+1,:) = detectedBBox;
            centroids(end+1,:) = [detectedBBox(1) + detectedBBox(3)/2, ...
                detectedBBox(2) + detectedBBox(4)/2, ...
                objPosition(3)];
            ids(end+1) = objProperties(6);
        end
    end
end

falseAlarmRate = 0.1;
numFalseAlarms = poissrnd(falseAlarmRate);
for j=1:length(cameras)
    for i = 1:numFalseAlarms
        fakeCentroid = [randi([round(viewBox(1)) round(viewBox(1) + viewBox(3))]), ...
            randi([viewBox(2) viewBox(2) + viewBox(4)]), 0];
        fakeWidth = randi([10 30]);
        fakeHeight = randi([10 30]);
        fakeBox = [fakeCentroid(1:2), fakeWidth, fakeHeight];
        if any(checkTrackInCameraViews(fakeBox, cameras))
            centroids(end+1,:) = fakeCentroid;
            bboxes(end+1,:) = [fakeCentroid(1:2) - [fakeWidth, fakeHeight]/2, fakeWidth, fakeHeight, 0];
            ids(end+1) = -1;
        end
        allFakeCentroids(end+1,:) = fakeCentroid;
    end
end

allFalseAlarms(end+1) = numFalseAlarms;
allMissedObjects(frameNumber,1:length(objects)) = missed;

end


% function [centroids, bboxes ,ids, allMissedObjects, allObjectsDetected] = generateDetections(objects, ...
%     mearusementVars, frameNumber, cameras, allMissedObjects, allObjectsDetected)
% 
% centroids = [[], [], []];
% bboxes = [[], [], [], [], []];
% ids = [];
% 
% missDetectionProb = 0.1;
% objectSaved = zeros(1,length(objects));
% allObjectsDetected(end+1) = length(objects);
% missed = binornd(1, missDetectionProb, 1, length(objects));
% for i = 1:length(objects)
%     if missed(i)
%         objectSaved(i) = 1;
%         continue
%     else
%         for j = 1:length(cameras)
%             if cameras(j).zoomState == ZoomState.ZOOMING_IN || cameras(j).zoomState == ZoomState.ZOOMING_OUT
%                 continue
%             else
%                 if ~objectSaved(i)
%                     box = cameras(j).zoomBox;
%                     objProperties = objects{i}.getProps(frameNumber);
%                     objPosition = objProperties(1:3);
%                     objDimensions = objProperties(4:5);
%                     cond1 = objPosition(1) >= box(1) && objPosition(2) >= box(2);
%                     cond2 = objPosition(1)+objDimensions(1) <= box(1)+box(3) && ...
%                         objPosition(2)+objDimensions(2) <= box(2)+box(4);
%                     if cond1 && cond2 
%                         detection = DetectedObject(objProperties, mearusementVars); 
%                         bboxes(end+1,:) = detection.getBBox();
%                         centroids(end+1,:) = [bboxes(end,1) + bboxes(end,3)/2, ...
%                             bboxes(end,2) + bboxes(end,4)/2, ...
%                             objPosition(3)];
%                         objectSaved(i) = 1;
%                         ids(end+1) = objProperties(6);
%                     end
%                 end
%             end
%         end
%     end
% end
% 
% falseAlarmRate = 0.1;
% 
% for j=1:length(cameras)
%     if cameras(j).zoomState == ZoomState.ZOOMED_IN
%         falseAlarmRate = falseAlarmRate / (cameras(j).zoomLevel)^2;
%     end
%     numFalseAlarms = poissrnd(falseAlarmRate);
%     if cameras(j).zoomState == ZoomState.ZOOMING_IN || cameras(j).zoomState == ZoomState.ZOOMING_OUT
%         continue
%     else
%         box = cameras(j).zoomBox;
%         for i = 1:numFalseAlarms
%             fakeCentroid = [randi([round(box(1)) round(box(1) + box(3))]), ...
%                 randi([round(box(2)) round(box(2) + box(4))]), 0];
%             fakeWidth = randi([10 30]);
%             fakeHeight = randi([10 30]);
%             centroids(end+1,:) = fakeCentroid;
%             bboxes(end+1,:) = [fakeCentroid(1:2) - [fakeWidth, fakeHeight]/2, fakeWidth, fakeHeight, 0];
%             ids(end+1) = -1;
%         end
%     end
% end
% 
% allMissedObjects(frameNumber,1:length(objects)) = missed;
% 
% end


function frame = generateMainFrame(originalDims, objects, frameNumber, cameras, centroids, bboxes)

vidFrame = zeros(originalDims(1),originalDims(2),3,'uint8');
frame = insertShape(vidFrame,'Filledcircle',[1, 1, 1],'color','black');

for i = 1:length(cameras)
    camPos = cameras(i).position(1:2);
    coords = [camPos(1)-10, camPos(2)-10, 20, 20];
    frame = insertShape(frame,'FilledRectangle',coords,'color',cameras(i).color);
    frame = insertShape(frame,'FilledCircle', [camPos, cameras(i).blindSpotRadius], ...
        'color', 'red');
    if ~isempty(cameras(i).zoomBox)
        label = strcat('Camera ', int2str(cameras(i).id));
        if any(cameras(i).zoomBox < 0)
            disp(cameras(i).zoomBox)
            disp(cameras(i).viewCenter)
        end
        frame = insertObjectAnnotation(frame,'rectangle',cameras(i).zoomBox, ...
            label, LineWidth=3, Color=cameras(i).color);
    end
end

for i = 1:length(objects)
    objProperties = objects{i}.getProps(frameNumber);
    if objects{i}.recordedTime == -1
        frame = insertShape(frame,'FilledRectangle', ...
            [objProperties(1:2),objProperties(4:5)],'color','white');
    else
        centroid = objProperties(1:2) + objProperties(4:5)/2;
        frame = insertText(frame, centroid, num2str(objects{i}.recordedTime), ...
            FontSize=18, BoxColor='black', BoxOpacity=0.4, TextColor="white");
        frame = insertText(frame, centroid - [10,10], num2str(objects{i}.id), ...
            FontSize=18, BoxColor='black', BoxOpacity=0.4, TextColor="red");
        frame = insertShape(frame,'FilledRectangle', ...
            [objProperties(1:2),objProperties(4:5)],'color','magenta');
    end
end

for k = 1:size(centroids,1)
    frame = insertShape(frame,'FilledCircle',[centroids(k,1:2), 5],'color','red');
    frame = insertShape(frame,'Rectangle',bboxes(k,1:4),'color','red');
end

end


function tracks = initializeTracks()

tracks = struct(...
        'id', {}, ...
        'bbox', {}, ...
        'kalmanFilter', {}, ...
        'age', {}, ...
        'totalVisibleCount', {}, ...
        'consecutiveInvisibleCount', {}, ...
        'watched', {}, ...
        'zoomedInVisibleCount', {}, ...
        'covRadius', {}, ...
        'objectID', {}, ...
        'recordedTime', {}, ...
        'overlapFlag', {});
end


function tracks = predictNewLocationsOfTracks(tracks)
    for i = 1:length(tracks) 
        bbox = tracks(i).bbox;
        [predictedCentroid, ~, ~] = predict(tracks(i).kalmanFilter);
        predictedCentroid = predictedCentroid - bbox(3:4) / 2;
        tracks(i).bbox = [predictedCentroid, bbox(3:4), bbox(5)];
    end
end


function [assignments, unassignedTracks, unassignedDetections] = ...
            detectionToTrackAssignment(tracks, centroids, cameras)

nTracks = length(tracks);
nDetections = size(centroids, 1);
% Compute the cost of assigning each detection to each track.
distCost = zeros(nTracks, nDetections);
% sizeCost = zeros(nTracks, nDetections);
costOfNonAssignment = 10;

if nDetections == 0
    unassignedTracks = length(tracks);
    assignments = [];
    unassignedDetections = [];
else
    for i = 1:nTracks
        trackStatus = checkTrackInCameraViews(tracks(i).bbox, cameras);
        if trackStatus
            distCost(i, :) = distance(tracks(i).kalmanFilter, centroids(:,1:2));
        else
            distCost(i, :) = Inf;
        end
    end
%     for i = 1:nTracks
%         distCost(i, :) = distance(tracks(i).kalmanFilter, centroids(:,1:2));
%     end
    % Solve the assignment problem.
    [assignments, unassignedTracks, unassignedDetections] = ...
        assignDetectionsToTracks(distCost, costOfNonAssignment);
end

end


function tracks = updateAssignedTracks(assignments, centroids, tracks, bboxes, ...
    cameras, ids)

numAssignedTracks = size(assignments, 1);

for i = 1:numAssignedTracks
    trackIdx = assignments(i, 1);
    detectionIdx = assignments(i, 2);
    centroid = centroids(detectionIdx, :);
    id = ids(detectionIdx);
    bbox = bboxes(detectionIdx, :);
    % Correct the estimate of the object's location
    % using the new detection.
    correct(tracks(trackIdx).kalmanFilter, centroid(1:2));
    % Replace predicted bounding box with detected
    % bounding box.
    tracks(trackIdx).bbox = bbox;
    % Update track's age.
    tracks(trackIdx).age = tracks(trackIdx).age + 1;
    % Update track's object ID
    tracks(trackIdx).objectID = id;
    % Update visibility.
    tracks(trackIdx).totalVisibleCount = ...
        tracks(trackIdx).totalVisibleCount + 1;
    tracks(trackIdx).consecutiveInvisibleCount = 0;
    tracks(trackIdx).covRadius = sqrt(max(tracks(trackIdx).kalmanFilter.StateCovariance, [], "all"));
    for j = 1:length(cameras)
        if cameras(j).zoomState == ZoomState.ZOOMED_IN
            inBoxFlag = isInsideBox(bbox,cameras(j).zoomBox);
            if inBoxFlag
                tracks(trackIdx).zoomedInVisibleCount = ...
                    tracks(trackIdx).zoomedInVisibleCount + 1;
                break
            end
        end
    end
end

end


function tracks = updateUnassignedTracks(tracks, unassignedTracks, cameras)

global viewBox

for k = 1:length(unassignedTracks)
    ind = unassignedTracks(k);
    trackBox = tracks(ind).bbox;
    tracks(ind).covRadius = sqrt(max(tracks(ind).kalmanFilter.StateCovariance, [], "all"));
    trackInCameraViews = checkTrackInCameraViews(trackBox, cameras);
    cond3 = trackBox(1) >= viewBox(1) && trackBox(2) >= viewBox(2);
    cond4 = trackBox(1) + trackBox(3) <= viewBox(3) && ...
                trackBox(2) + trackBox(4) <= viewBox(4);
    if trackInCameraViews || ~(cond3 && cond4)
        tracks(ind).age = tracks(ind).age + 1;
        tracks(ind).consecutiveInvisibleCount = ...
            tracks(ind).consecutiveInvisibleCount + 1;
    end
end

end


function trackStatus = checkTrackInCameraViews(trackBox, cameras)

trackInCameraView = zeros(1,length(cameras));

for j = 1:length(cameras)
    if cameras(j).zoomState == ZoomState.ZOOMING_IN || ...
        cameras(j).zoomState == ZoomState.ZOOMING_OUT
        trackInCameraView(j) = 0;
    else
        camBox = cameras(j).zoomBox;
        cond1 = trackBox(1) >= camBox(1) && trackBox(2) >= camBox(2);
        cond2 = trackBox(1)+trackBox(3) <= camBox(1)+camBox(3) && ...
            trackBox(2)+trackBox(4) <= camBox(2)+camBox(4);
        trackInCameraView(j) = (cond1 && cond2);
    end
end

trackStatus = any(trackInCameraView);

end


function tracks = deleteLostTracks(tracks)

if isempty(tracks)
    return;
end

invisibleForTooLong = 8;
ageThreshold = 8;
visibilityThreshold = 0.6;
covThreshold = 200;

% Compute the fraction of the track's age for which it was visible.
ages = [tracks(:).age];
covariances = [tracks(:).covRadius];
totalVisibleCounts = [tracks(:).totalVisibleCount];
visibility = totalVisibleCounts ./ ages;

% Find the indices of 'lost' tracks.
lostInds = (ages < ageThreshold & visibility < visibilityThreshold) | ...
    [tracks(:).consecutiveInvisibleCount] >= invisibleForTooLong | ...
    covariances >= covThreshold;

tracks = tracks(~lostInds);

end


function [tracks, nextId] = createNewTracks(tracks, unassignedDetections, ...
    centroids, bboxes, nextId, objectIds)

global processVars
global measurementVars

% measurementNoise = 1.5;
% motionNoise = [0.75, 0.75];
% initialEstimateError = [measurementNoise, 1e4];
centroids = centroids(unassignedDetections, :);
bboxes = bboxes(unassignedDetections, :);
objectIds = objectIds(unassignedDetections);

for k = 1:size(centroids, 1)
    centroid = centroids(k,1:2);
    bbox = bboxes(k, :);
    objectId = objectIds(k);
%     kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
%         centroid, [1e-3, 1e-5], [0.5, 0.1]/30, 0.1/30);
    kalmanFilter = configureKalmanFilter('ConstantVelocity', ...
        centroid, max(measurementVars) * [1, 1e4], ...
        [max(processVars(1), processVars(3)), max(processVars(2), processVars(4))], ...
        max(measurementVars));
    newTrack = struct(...
        'id', nextId, ...
        'bbox', bbox, ...
        'kalmanFilter', kalmanFilter, ...
        'age', 1, ...
        'totalVisibleCount', 1, ...
        'consecutiveInvisibleCount', 0, ...
        'watched', 0, ...
        'zoomedInVisibleCount', 0, ...
        'covRadius', 0, ...
        'objectID', objectId, ...
        'recordedTime', -1, ...
        'overlapFlag', 0);
    tracks(end + 1) = newTrack;
    nextId = nextId + 1;
end

end


function frame = displayFullTrackingResults(tracks, frame, cameras)

% Convert the frame and the mask to uint8 RGB.
frame = im2uint8(frame);
minVisibleCount = 0;

if ~isempty(tracks)
    for i=1:length(tracks)
        bbox = tracks(i).bbox;
        centroid = [bbox(1) + bbox(3)/2, bbox(2) + bbox(4)/2];
        covRadius = sqrt(max(tracks(i).kalmanFilter.StateCovariance, [], "all"));
        frame = insertShape(frame,'Filledcircle',[centroid, covRadius],'color','green');
    end
    reliableTrackInds = ...
        [tracks(:).totalVisibleCount] > minVisibleCount;
    reliableTracks = tracks(reliableTrackInds);
    if ~isempty(reliableTracks)
        bboxes = cat(1, reliableTracks.bbox);
        ids = int32([reliableTracks(:).id]);
        labels = cellstr(int2str(ids'));
        predictedTrackInds = [];
        for i = 1:length(reliableTracks)
            trackBox = reliableTracks(i).bbox;
            trackInCamerasView = checkTrackInCameraViews(trackBox, cameras);
            if ~trackInCamerasView || reliableTracks(i).consecutiveInvisibleCount > 0
                predictedTrackInds(end+1) = i;
            end
        end
        isPredicted = cell(size(labels));
        isPredicted(predictedTrackInds) = {' predicted'};
        labels = strcat(labels, isPredicted);
        frame = insertObjectAnnotation(frame, 'rectangle', ...
            bboxes(:,1:4), labels);
    end
end

end