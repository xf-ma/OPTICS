classdef Camera < handle & dynamicprops
    properties
        cameraID
        intrinsic= [];
        extrinsic= [];
        pose= [];
        referenceImage_raw= [];
        referenceImage_undistorted= [];
        referenceImage_detected= [];
        referenceImagePoints= [];
        referenceImagePoints_matched= [];
        roughPerspectiveMatrix= [];
        areaCorners= [];
        imageTrackPoints = [];
       

    end

    methods
        function self = Camera(camID,oldInstance)
            

            if exist("oldInstance","class") && isa(oldInstance, 'Camera')
                % Copy-constructor: merge from old object

                metaNew = metaclass(self);
                metaOld = metaclass(oldInstance);
                
                % Loop over all properties of the new class
                for p = metaNew.PropertyList'
                    name = p.Name;
                    if ~p.Dependent
                        if any(strcmp(name, {metaOld.PropertyList.Name}))
                            % Copy from old object if property exists
                            self.(name) = oldInstance.(name);
                        end
                    end
                end
            end
                
            if ~isempty(camID)
                self.cameraID = camID;
            end


            
        end
    end
end