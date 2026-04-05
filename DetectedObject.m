classdef DetectedObject
    properties
        position (1,3)
        size (1,2)
        centerVar (1,2)
        sizeVar (1,2)
        id
    end
    methods
        function obj = DetectedObject(objProps, measureNoise)
            obj.position = objProps(1:3);
            obj.size = objProps(4:5);
            obj.centerVar = measureNoise(1:2);
            obj.sizeVar = measureNoise(3:4);
            obj.id = objProps(6);
        end
        function bbox = getBBox(obj)
            bbox(1) = obj.position(1) + normrnd(0, obj.centerVar(1));
            bbox(2) = obj.position(2) + normrnd(0, obj.centerVar(2));
            bbox(3) = obj.size(1) + normrnd(0, [obj.sizeVar(1)]);
            bbox(4) = obj.size(2) + normrnd(0, [obj.sizeVar(2)]);
            bbox(5) = obj.position(3);
        end
    end
end