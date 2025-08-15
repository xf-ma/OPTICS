classdef OPTICS < handle & dynamicprops
    properties
        cameras Camera
        cameraIDs      
        
        MDSmethods = {"original","linesearch","GD","LM"};
        config
        distanceMatrix = [];
        distanceWeights = [];
        distanceMatrixProcessed = [];
        referenceCoords_true = [];
        referenceCoords_sortedtrue = [];
        referenceCoords_InitialGuess = [];
        referenceCoords_estimates = [];
        normalizedCorners= [0,0;0,6;4,6;4,0];
        markerDetect_sizeLimit = [0.2, 350] 
        markerDetect_ratioLimit = [0.5, 1.5]

        reconstructedTrajectory = [];
        

        fig
        editBoxes     
        editLabels
        resultAxes
        resultTexts
        resultsPanel
        imageAxes
        imagesShow
        

    end

    methods
        function self = OPTICS(N_cam, givenIntrinsics, distanceMatrix, imagePath, refPointsTruth)
            camIDs = cell(1,N_cam);         
            for k = 1:N_cam
                camID = char('A' + k - 1);
                camIDs{k} = camID;
                self.cameras(k) = Camera(camID);
            end
            self.cameraIDs = camIDs;
            self.setIntrinsic(givenIntrinsics);
            self.distanceMatrix = distanceMatrix;   
            self.processDistanceMatrix();


            if ~isempty(imagePath)
                filesInfo = dir(imagePath);
                for i = 1:length(filesInfo)
                    filename = fullfile(filesInfo(i).folder, filesInfo(i).name);
                    self.cameras(i).referenceImage_raw = imread(filename);
                    fprintf('Loaded: %s\n', filesInfo(i).name);
                end
               
            end

            if ~isempty(refPointsTruth)
                self.referenceCoords_true = refPointsTruth;
                [~, sortIdx] = sort(refPointsTruth(:, 2), 'ascend');
                self.referenceCoords_sortedtrue = refPointsTruth(sortIdx, :);
                figure;hold on
                plot(refPointsTruth(:,1),refPointsTruth(:,2),'-o',"DisplayName","reference points truth")
                plot(self.referenceCoords_sortedtrue(:,1),self.referenceCoords_sortedtrue(:,2),'-x',"DisplayName","sorted reference points truth")
                legend();
            
            end
            

            self.config.maxIteration =100;
            self.config.tolerence = 1e-15;
            self.config.lambda = 1;%e-3;

            self.imagesShow = self.collectCameraSubFields("referenceImage_raw");

            self.createGUI();
            self.displayImages("original");
            

        end
        function processDistanceMatrix(self)
            
            D_noisy = self.distanceMatrix;
            

            W = double(~isnan(D_noisy)); % Set weights to 0 for missing data
            D_noisy(isnan(D_noisy)) = 0; % Replace NaN with 0 for computation

            self.distanceMatrixProcessed = D_noisy;
            
            self.distanceWeights = W;
            disp("Distance matrix and weights are updated.")

            
        end



        function camIDs = getCameraIDs(self)
            camIDs = self.cameraIDs; 
        end


        function setIntrinsic(self,givenIntrinsics)
            if exist("givenIntrinsics","var") && ~isempty(givenIntrinsics)
                camIDs = self.getCameraIDs();
                for i = 1:length(camIDs)
                    self.cameras(i).intrinsic = givenIntrinsics{i};                   
                end
                disp("Intrinsics are stored to your OPTICS instance.");

            else
                Dlg = uifigure('Name', 'Important!!!', 'Position', [500 500 580 150]);
            

                uilabel(Dlg, 'Position', [20 130 570 22], 'Text', 'Please export your data from "Camera Calibrator" in order, and the store the intrinsics at');
                uilabel(Dlg, 'Position', [20 110 570 22], 'Text', '        OPTICSinstance.cameraX.intrinsic');
                uilabel(Dlg, 'Position', [20 90 570 22], 'Text', 'For example: ');
                uilabel(Dlg, 'Position', [20 70 570 22], 'Text', 'if your OPTICS instance is "optics", and you export your first camera data as cameraParameters1,');
                uilabel(Dlg, 'Position', [20 50 570 22], 'Text', 'you need to run the following command:');
                uilabel(Dlg, 'Position', [20 30 570 22], 'Text', '         optics.cameras(1).intrinsic = cameraParameters1.intrinsics');

                cameraCalibrator;
            end


        end

        function createGUI(self)
            self.fig = figure('Name', 'Approach Comparison Tool', ...
                           'Position', [100, 100, 1200, 800], ...
                           'CloseRequestFcn', @(src,evt) self.onGUIClose(src));
            l
            myPanel = uipanel('Parent', self.fig, ...
                                'Title', 'Configuration Parameters', ...
                                'Position', [0.02, 0.7, 0.3, 0.28]);
            
            paramSettings = {
                {'maxIteration', self.config.maxIteration};
                {'tolerence', self.config.tolerence};
                {'stepSize', self.config.lambda};
                {'Area width and length', self.normalizedCorners(3,:)}
            };
            

            self.editBoxes = cell(5, 1);  
            self.editLabels = cell(5, 1);
            
            for i = 1:4
                self.editLabels{i} = uicontrol('Parent', myPanel, ...
                    'Style', 'text', ...
                    'String', paramSettings{i}{1}, ...
                    'Position', [10, 220-i*50, 80, 20], ...
                    'HorizontalAlignment', 'left');
                
                self.editBoxes{i} = uicontrol('Parent', myPanel, ...
                    'Style', 'edit', ...
                    'String', num2str(paramSettings{i}{2}), ...
                    'Position', [100, 220-i*50, 100, 25], ...
                    'HorizontalAlignment', 'center', ...
                    'BackgroundColor', 'white', ...
                    'Callback', @(src,evt) self.editBoxCallback(i, src));
            end

                self.editLabels{5} = uicontrol('Parent', self.fig, ...
                    'Style', 'text', ...
                    'String', 'camera ', ...
                    'Position', [210, 250-40*2, 50, 30], ...
                    'HorizontalAlignment', 'left');
                
                self.editBoxes{5} = uicontrol('Parent', self.fig, ...
                    'Style', 'edit', ...
                    'String', '', ...
                    'Position', [250, 250-40*2, 50, 30], ...
                    'HorizontalAlignment', 'center', ...
                    'BackgroundColor', 'white', ...
                    'Callback', @(src,evt) self.editBoxCallback(5, src));
         


            self.resultsPanel = uipanel('Parent', self.fig, ...
                                 'Title', 'Display Area', ...
                                 'Position', [0.3, 0.02, 0.68, 0.96]);
            
            button1heigt = 250;
            uicontrol('Parent', self.fig, ...
                     'Style', 'pushbutton', ...
                     'String', 'Show original images', ...
                     'Position', [50, button1heigt, 150, 30], ...
                     'Callback', @(src,evt) self.displayImages("original"));

            uicontrol('Parent', self.fig, ...
                     'Style', 'pushbutton', ...
                     'String', 'Show undistorted images', ...
                     'Position', [50, button1heigt-40, 150, 30], ...
                     'Callback', @(src,evt) self.displayImages("undistorted"));

            uicontrol('Parent', self.fig, ...
                     'Style', 'pushbutton', ...
                     'String', 'detect reference markers', ...
                     'Position', [50, button1heigt-40*2, 150, 30], ...
                     'Callback', @(src,evt) self.detectAllReferenceImagePoints());

            uicontrol('Parent', self.fig, ...
                     'Style', 'pushbutton', ...
                     'String', 'match reference markers', ...
                     'Position', [50, button1heigt-40*3, 150, 30], ...
                     'Callback', @(src,evt) self.matchAllReferenceImagePoints());

            uicontrol('Parent', self.fig, ...
                     'Style', 'pushbutton', ...
                     'String', 'Show MDS results', ...
                     'Position', [50, button1heigt-40*4, 150, 30], ...
                     'Callback', @(src,evt) self.computeRefMarkerCoords());

            uicontrol('Parent', self.fig, ...
                     'Style', 'pushbutton', ...
                     'String', 'Show cameras poses', ...
                     'Position', [50, button1heigt-40*5, 150, 30], ...
                     'Callback', @(src,evt) self.computeAllCamerasExreinsics()); 

            uicontrol('Parent', self.fig, ...
                     'Style', 'pushbutton', ...
                     'String', 'Show reconstructed trajectory', ...
                     'Position', [50, button1heigt-40*6, 150, 30], ...
                     'Callback', @(src,evt) self.reconstrcut3Dtrajectory());   
                      
        end
        
        function editBoxCallback(self, paramIndex, editHandle)
            inputStr = get(editHandle, 'String');
            
            value = str2double(inputStr);

            switch paramIndex
                case 1
                    self.config.maxIteration = value;
                case 2
                    self.config.tolerence = value;
                case 3
                    self.config.lambda = value;
                case 4
                    self.normalizedCorners = [0,0;0,value(2);value(1),value(2);value(1),0];
                case 5
                    self.detectReflectiveMaker(inputStr);
                    self.displayImages('detected');


            end

        end
        function onGUIClose(self, src)
            % This is called when user closes the GUI window
            fprintf('GUI window closed. Your class instance remains alive.\n');
            
            % Just close the figure, don't delete the selfect
            delete(src);
            
            % Clear GUI-related properties but keep the selfect alive
            self.fig = [];
            self.editBoxes = [];
            self.editLabels = [];
            self.resultAxes = [];
            self.resultTexts = [];
        end
        function reopenGUI(self)
            % Method to reopen the GUI if it was closed
            if isempty(self.fig) || ~isvalid(self.fig)
                self.createGUI();
                self.displayImages("original");
            else
                figure(self.fig);  % Bring existing GUI to front
            end
        end 

        function undistortImages(self)
            if isempty(self.cameras(1).intrinsic)
                warndlg("Please set the intrinsics first!")
                return;
            end
            camIDs = self.getCameraIDs();
            for i = 1:length(camIDs)
                self.cameras(i).referenceImage_undistorted = ...
                undistortImage(self.cameras(i).referenceImage_raw, self.cameras(i).intrinsic, OutputView="same");
            end
        end

        function detectAllReferenceImagePoints(self)
            if isempty(self.cameras(1).referenceImage_undistorted)
                warndlg("You need to undistort the reference images before detect the reference markers! Now I have done this for you.");
                self.undistortImages();
            end
            if isempty(self.cameras(1).referenceImage_detected)
                camIDs = self.getCameraIDs();
                for i = 1:length(camIDs)
                    self.detectReflectiveMaker(camIDs{i});
                end
            end
            self.displayImages('detected');
            
        end
        function displayImages(self,type)            
            children = get(self.resultsPanel, 'Children');
            delete(children);
            switch type
                case 'original'
                    self.imagesShow = self.collectCameraSubFields("referenceImage_raw");                
                case 'undistorted'
                    if isempty(self.cameras(1).referenceImage_undistorted)
                        self.undistortImages();
                    end

                    self.imagesShow = self.collectCameraSubFields("referenceImage_undistorted");
                case 'detected'
                    self.imagesShow = self.collectCameraSubFields("referenceImage_detected");
                case 'matched'
                    self.imagesShow = self.collectCameraSubFields("referenceImage_undistorted");
                   
                 otherwise
             end


            set(self.resultsPanel, 'Title', 'Image Display');
            if isempty(self.imagesShow)
                % No images to display
                uicontrol('Parent', self.resultsPanel, ...
                    'Style', 'text', ...
                    'String', 'No images loaded', ...
                    'Position', [50, 200, 200, 50], ...
                    'FontSize', 14, ...
                    'HorizontalAlignment', 'center');
                return;
            end
            
            numImages = length(self.imagesShow);
            
            % Calculate optimal grid layout
            if numImages == 1
                rows = 1; cols = 1;
            elseif numImages <= 4
                rows = 2; cols = 2;
            elseif numImages <= 6
                rows = 2; cols = 3;
            elseif numImages <= 9
                rows = 3; cols = 3;
            elseif numImages <= 12
                rows = 3; cols = 4;
            else
                rows = ceil(sqrt(numImages));
                cols = ceil(numImages / rows);
            end
            
            % Calculate subplot positions
            for i = 1:numImages
                row = ceil(i / cols);
                col = mod(i-1, cols) + 1;
                
                % Calculate position [left, bottom, width, height]
                left = 0.02 + (col-1) * (0.96/cols);
                bottom = 0.98 - row * (0.96/rows);
                width = 0.96/cols - 0.02;
                height = 0.96/rows - 0.02;
                
                % Create axes for this image
                ax = axes('Parent', self.resultsPanel, ...
                         'Position', [left, bottom, width, height]);
                
                % Display the image
                try
                    if ischar(self.imagesShow{i}) || isstring(self.imagesShow{i})
                        % Image is a filename
                        img = imread(self.imagesShow{i});
                        imshow(img, 'Parent', ax);
                        title(ax, sprintf('Image %s: %s', self.cameraIDs{i}, self.imagesShow{i}), 'Interpreter', 'none');
                    else
                        % Image is already loaded data
                        
                        switch type
                            case "detected"
                                imshow(self.imagesShow{i}, 'Parent', ax);hold(ax, 'on');
                                plot(self.cameras(i).referenceImagePoints(:,1),self.cameras(i).referenceImagePoints(:,2),'o-w','Parent',ax)
                                title(ax, sprintf('Image %s: %d markers', self.cameraIDs{i},length(self.cameras(i).referenceImagePoints)));
                            case "matched"
                                imshow(self.imagesShow{i}, 'Parent', ax);hold(ax, 'on');
                                plot(self.cameras(i).referenceImagePoints(:,1),self.cameras(i).referenceImagePoints(:,2),'o:w','Parent',ax,'DisplayName',"unmatched")
                                plot(self.cameras(i).referenceImagePoints_matched(:,1),self.cameras(i).referenceImagePoints_matched(:,2),'x-r','Parent',ax,'DisplayName',"matched")
                                plot(self.cameras(i).areaCorners(:,1),self.cameras(i).areaCorners(:,2),'b-s','Parent',ax)
                                for j = 1:4
                                    text(self.cameras(i).areaCorners(j,1) + 0.02, self.cameras(i).areaCorners(j,2) + 2, num2str(j), ...
                                        'Color', 'b', 'FontSize',12,'Parent',ax);
                                end
                                title(ax, sprintf('Image %s: %d markers', self.cameraIDs{i},length(self.cameras(i).referenceImagePoints)));
                                legend(ax);

                            otherwise
                                imshow(self.imagesShow{i}, 'Parent', ax);hold(ax, 'on');
                            title(ax, sprintf('Image %s', self.cameraIDs{i}));
                        end
                    end
                catch ME
                    % Error loading image - show placeholder
                    text(0.5, 0.5, sprintf('Error loading\nImage %d', i), ...
                         'Parent', ax, 'HorizontalAlignment', 'center', ...
                         'VerticalAlignment', 'middle');
                    set(ax, 'XLim', [0 1], 'YLim', [0 1]);
                    title(ax, sprintf('Image %d (Error)', i));
                end
            end
        end
        

        function computeRefMarkerCoords(self)
            rng(50)
            X0 = randn(size(self.distanceMatrix,1), 2); % Initialize X randomly
            self.referenceCoords_InitialGuess = X0;
            
            
            children = get(self.resultsPanel, 'Children');
            delete(children);

            set(self.resultsPanel, 'Title', 'Results Visualization');
            self.resultAxes = cell(2, 2);
            self.resultTexts = cell(4, 1);

            positions = {[0.05, 0.55, 0.4, 0.4], [0.55, 0.55, 0.4, 0.4], ...
                        [0.05, 0.05, 0.4, 0.4], [0.55, 0.05, 0.4, 0.4]};

            self.resultAxes{1, 1} = axes('Parent', self.resultsPanel, ...
                    'Position', positions{1});
            title(self.resultAxes{1, 1}, "Estimated reference coordinates");

            self.resultAxes{1, 2} = axes('Parent', self.resultsPanel, ...
                    'Position', positions{2});
            title(self.resultAxes{1, 2}, "Stress");

            self.resultAxes{2, 1} = axes('Parent', self.resultsPanel, ...
                    'Position', positions{3});
            title(self.resultAxes{2, 1}, "Cameras poses");

            self.resultAxes{2, 2} = axes('Parent', self.resultsPanel, ...
                    'Position', positions{4});
            title(self.resultAxes{2, 2}, "3D reconstruction of tracking");

           
            
            % Update with actual results
            MDSestimates = struct();
            finalStress = zeros(length(self.MDSmethods),1)+Inf;

            for i = 1:length(self.MDSmethods)
                method = self.MDSmethods{i};
                self.config.method = method;
                [MDSestimates.(method), stress.(method), iteration.(method)] = self.smacof_mds();
                MDSestimates.(method) = self.transformToSameDirection(self.referenceCoords_sortedtrue,MDSestimates.(method),'proc');
                finalStress(i) = stress.(method)(end);
            end
            [min_stress, idx] = min(finalStress);           
            MDSestimates.best = MDSestimates.(self.MDSmethods{idx});
            MDSestimates.bestname = self.MDSmethods{idx};
            self.referenceCoords_estimates = MDSestimates;
            
            % Update visualizations
            self.updateVisualization(MDSestimates, stress, [], []);
   
        end

        
        function updateVisualization(self, MDSestimates, MDSstress, cameraPoses, trajectory3D)
            approaches = self.MDSmethods;
            shapes = {"o","+","x","^"};
            
            if ~isempty(MDSestimates)
                axes(self.resultAxes{1, 1});
                cla; hold on
                if ~isempty(self.referenceCoords_sortedtrue)
                    plot(self.referenceCoords_sortedtrue(:,1),self.referenceCoords_sortedtrue(:,2),"go","MarkerFaceColor","g","MarkerSize",10)
                end
                
                for i = 1:length(approaches)
                    data = MDSestimates.(approaches{i});               
                    plot(data(:,1),data(:,2),"Marker",shapes{i},"DisplayName",approaches{i});
                
                end
                grid on;
                legend();
                ylabel('Y (m)');
                xlabel('X (m)');  

            end
            if ~isempty(MDSstress)
                axes(self.resultAxes{1, 2});
                cla; hold on
                
                for i = 1:length(approaches)
                    data = MDSstress.(approaches{i});
                    plot(data,"DisplayName",approaches{i});                
                end
                grid on;
                legend();
                ylabel('stress');
                xlabel('iteration');                 
            end
            if ~isempty(cameraPoses)
                axes(self.resultAxes{2, 1});
                cla; hold on 
                worldCoords = self.referenceCoords_estimates.best;
                pcshow([worldCoords,zeros(size(worldCoords,1),1)], ...
                      VerticalAxisDir="Up",MarkerSize=40,BackgroundColor=[1 1 1]);hold on
                if ~isempty(self.referenceCoords_sortedtrue)
                    worldCoordsTrue = self.referenceCoords_sortedtrue;
                    pcshow([worldCoordsTrue(:,1:2),zeros(size(worldCoords,1),1)],"g", ...
                      VerticalAxisDir="Up",MarkerSize=40,BackgroundColor=[1 1 1]);hold on
                end              
                               
                for i = 1:length(self.cameraIDs)
                    plotCamera(AbsolutePose=cameraPoses{i},Size=0.5); hold on   
                    camLocation = cameraPoses{i}.Translation; 
                    text(camLocation(1), camLocation(2), camLocation(3),sprintf('%s', self.cameraIDs{i}), ...
          'FontSize', 20, 'Color', 'red', 'HorizontalAlignment', 'left');
             
                end
            end
            if ~isempty(trajectory3D)
                axes(self.resultAxes{2, 2});
                cla; hold on
                plot3(trajectory3D(:,1),trajectory3D(:,2),trajectory3D(:,3),"DisplayName","Reconstructed trajectory")
                view(3);
                ylabel('Y (m)');
                xlabel('X (m)'); 
                zlabel('Z (m)');


            end
                
            drawnow;
        end

        function matchedTransformedTOBEPoints = transformToSameDirection(~,ref,tobe,type)
            ref2D = ref(:,1:2); tobe2D = tobe(:,1:2);
            switch type
                case 'proc'
                    [~,~, tr] = procrustes(ref2D, tobe2D);
                    tobe2D_transformed = tobe2D * tr.T + repmat(tr.c(1,:), length(tobe2D), 1);
                    hfig = figure;ax = axes('Parent', hfig);hold on
                    plot(ax,ref2D(:,1),ref2D(:,2),'b:o',"DisplayName",'ref');
                    plot(ax,tobe2D(:,1),tobe2D(:,2),'r:x',"DisplayName",'original TOBE')        
                    plot(ax,tobe2D_transformed(:,1),tobe2D_transformed(:,2),"Color",'r',"DisplayName",'transformed TOBE');    
                case 'boundry'
  
                    % Define distance threshold for inclusion
                    shrinkStep = 0.05; 
                    shrinkFactor = 0;
                    
                    convRef = boundary(ref2D,shrinkFactor);
                    convTOBE = boundary(tobe2D,shrinkFactor);
                                        
                    refBoundrySize = size(convRef,1);
                    tobeBoundrySize = size(convTOBE,1);
                    
                    while refBoundrySize ~= tobeBoundrySize
                        shrinkFactor = shrinkFactor + shrinkStep;
                        if refBoundrySize < tobeBoundrySize
                            convRef = boundary(ref2D,shrinkFactor);
                            refBoundrySize = size(convRef);
                        elseif refBoundrySize > tobeBoundrySize
                            convTOBE = boundary(tobe2D,shrinkFactor);
                            tobeBoundrySize = size(convTOBE);
                        end
                    end
                    
                    refHullPoints = ref2D(convRef, :);
                    tobeHullPoints = tobe2D(convTOBE, :);                    
                    
                    [d, Z, transform] = procrustes(ref2D(convRef,:), tobe2D(convTOBE,:));
                    tobe2D_transformed = transform.b*tobe2D*transform.T + transform.c(1,:);

                    hfig = figure;ax = axes('Parent', hfig);hold on
                    plot(ax,ref2D(:,1),ref2D(:,2),'b:o',"DisplayName",'ref');
                    plot(ax,refHullPoints(:,1),refHullPoints(:,2),"Color",'b',"DisplayName",'ref Boundary');      
                    plot(ax,tobe2D(:,1),tobe2D(:,2),'r:x',"DisplayName",'original TOBE')        
                    plot(ax,tobeHullPoints(:,1),tobeHullPoints(:,2), '--r',"DisplayName",'original TOBE boundary');                    
                    plot(ax,tobe2D_transformed(:,1),tobe2D_transformed(:,2),"Color",'r',"DisplayName",'transformed TOBE');
                                     
                otherwise
            end

            % matching by minimial distance
           matchedTransformedTOBEPoints = zeros(size(tobe));
            for i = 1:length(ref2D)
                distances = sqrt(sum((tobe2D_transformed(:,1:2) - ref2D(i, 1:2)).^2, 2));
                [~, closestIdx] = min(distances);
                matchedTransformedTOBEPoints(i, :) = tobe2D_transformed(closestIdx, :);
                
            end

            plot(ax,matchedTransformedTOBEPoints(:,1),matchedTransformedTOBEPoints(:,2),'r:o',"DisplayName",'matched and transformed');
            legend()
            
            
        end

        function data = collectCameraSubFields(self,XXX)
            data = {self.cameras(:).(XXX)};
        end


        function computeAllCamerasExreinsics(self)
            camIDs = self.getCameraIDs();
            figure;hold on
            camPoses = cell(length(camIDs),1);
            for i = 1:length(camIDs)
                self.computeNExtrinsicsByRefCoords(i);
                camPoses{i} = self.cameras(i).pose;
                                  
            end
            self.updateVisualization([], [], camPoses, []);

        end

        function [P, extrinsic, cameraPose] = computeNExtrinsicsByRefCoords(self,camID) 
            %{
            require: the matched image points (pixel) and matched world
            coordinates (meter) of the reference markers, and intrinsic.
            Note that the image points should be undistorted            
            %}

            if ischar(camID)
                camNo = double(camID) - double('A') + 1;
            else
                camNo = camID;
            end
            worldCoords = self.referenceCoords_estimates.best;
            intrinsic = self.cameras(camNo).intrinsic;
            imagePoints = self.cameras(camNo).referenceImagePoints_matched;

            % Compute extrinsic
            % extrinsic = estimateExtrinsics(imagePoints,worldCoords,intrinsic);
            % cameraPose = extr2pose(extrinsic)
            cameraPose = estworldpose(imagePoints,[worldCoords,zeros(length(worldCoords),1)],intrinsic);
            extrinsic = pose2extr(cameraPose);

            % Calculate the camera projection matrix P
            P = cameraProjection(intrinsic,extrinsic);

            % Reprojection to verify
            % reprojectedImgPoints = P*[worldCoords,zeros(length(worldCoords),1),ones(length(worldCoords),1)]';
            % reprojectedImgPoints = (reprojectedImgPoints./reprojectedImgPoints(3,:))';
            reprojectedImgPoints = world2img([worldCoords,zeros(length(worldCoords),1)],extrinsic,intrinsic,ApplyDistortion=false);
            disp("Reprojection error: " + num2str(mean(vecnorm(reprojectedImgPoints(:,1:2) - imagePoints,2,2))))     

            self.cameras(camNo).extrinsic = extrinsic;
            self.cameras(camNo).pose = cameraPose;
            

        end


        function matchAllReferenceImagePoints(self)
            camIDs = getCameraIDs(self);                       
            % transform the image points roughly to rectangle area
            for i = 1:length(camIDs)
                if isempty(self.cameras(i).roughPerspectiveMatrix) || isempty(self.cameras(i).areaCorners) 
                    image = self.cameras(i).referenceImage_undistorted;   
                    self.computeTransformationMatrix(i, image);    

                end
            end



            % --------- Mohamed
            for i = 1:length(camIDs)
                T = self.cameras(i).roughPerspectiveMatrix;
                pts = self.cameras(i).referenceImagePoints;
                [pts_x,pts_y] = transformPointsForward(T, pts(:, 1), pts(:, 2));
                imagePointsTransformed.(camIDs{i}) = [pts_x,pts_y];
     
            end
            figure;hold on
            for j = 1:length(camIDs)
                plot(imagePointsTransformed.(camIDs{j})(:,1),imagePointsTransformed.(camIDs{j})(:,2),'o-','DisplayName',camIDs{j})
            end
            title("Transformed unmatched images points")
            legend();


            % Regard A as reference, sort markers for camera A
            refPts = imagePointsTransformed.A;
            [~, sortIdx] = sort(refPts(:, 2), 'ascend');
            refPts_sorted = refPts(sortIdx, :);

            matchedTransformedCoords.A = refPts_sorted;

            figure;hold on
            plot(imagePointsTransformed.A(:,1),imagePointsTransformed.A(:,2),'-o',"DisplayName","image Points Transformed A")
            plot(refPts_sorted(:,1),refPts_sorted(:,2),'-',"DisplayName","matchingRef points sorted")
            legend();

            matchStart = 2;


            
            
            % Matches markers between cameras by finding the closest points in the coordinate system.
               
            for j = matchStart:length(camIDs)
                matchedTransformedCoords.(camIDs{j}) = zeros(length(refPts_sorted),2);
                for i = 1:size(refPts_sorted, 1)
                    distances = sqrt(sum((imagePointsTransformed.(camIDs{j}) - refPts_sorted(i, 1:2)).^2, 2));
                    [~, closestIdx] = min(distances);
                    matchedTransformedCoords.(camIDs{j})(i, :) = imagePointsTransformed.(camIDs{j})(closestIdx, :);
                end
            end


            figure;hold on
            for j = 1:length(camIDs)
            plot(matchedTransformedCoords.(camIDs{j})(:,1),matchedTransformedCoords.(camIDs{j})(:,2),'o-','DisplayName',camIDs{j})
            end
            title("Matched images points")
            legend();
        
            % Get Original Sorted Coordinates

            for j = 1:length(camIDs)
                P_Tmatched = matchedTransformedCoords.(camIDs{j});
                T = self.cameras(j).roughPerspectiveMatrix;
                [X_matched, Y_matched] = transformPointsInverse(T, P_Tmatched(:,1), P_Tmatched(:,2));
                self.cameras(j).referenceImagePoints_matched = [X_matched, Y_matched];
            end
            % ---------


            self.displayImages("matched")
            
        end

        function computeTransformationMatrix(self, cameraID, img)
            % Function: computeTransformationMatrices
            % Purpose: Computes projective transformation matrices for each camera based on 
            % the manual selection of rectangle corners in images.
            % Inputs:
            %   - cameraIDs: Cell array of camera identifiers.
            %   - imageFiles: Cell array of image file paths.
            %   - normalizedCorners: Normalized coordinates of rectangle corners.
            % Outputs:
            %   - str_tr: Structure containing transformation matrices and selected corners for each camera.
          
            if ischar(cameraID)
                cameraNo = double(upper(cameraID)) - double('A') + 1;
            else
                cameraNo = cameraID;
                
            end
            if ~exist("img","var")
                img = self.cameras(cameraID).referenceImage_undistorted;
            end
        
            str_tr = struct();
                
            figure('Name', sprintf('Select Rectangle Corners for %s', cameraID));
            imshow(img);
            title(sprintf('Select 4 Corners of the Rectangle in Order: 1, 2, 3, 4'));
            
            % Let user manually select 4 corners
            [x, y] = ginput(4);
            str_tr.corners =  [x, y];
           
            hold on;
            plot(x, y, 'ro-', 'LineWidth', 2, 'MarkerSize', 10);
            hold off;
            
            % Calculate transformation matrix to the normalized coordinates
            str_tr.cams_tr =  fitgeotrans([x, y], self.normalizedCorners, 'projective');
           
            close;
            
            fprintf('Transformation matrix for %s computed .\n', cameraID);

            self.cameras(cameraNo).areaCorners = str_tr.corners;
            self.cameras(cameraNp).roughPerspectiveMatrix = str_tr.cams_tr;

        end

        function reconstrcut3Dtrajectory(self,interval)
           
            
            camIDs = self.getCameraIDs(); 
            imgPoints = self.collectCameraSubFields("imageTrackPoints");
            if ~exist("interval","var")
                interval = 1:length(imgPoints{1});
            end
            
            
            if length(camIDs) ~= length(imgPoints)
                error("Number of images not matching cameras")
            end
            for i = 1:length(camIDs)
                trackerImgPts_distorted{i} = imgPoints{i}(interval,:);
               
            end
            
            % remove Nan
            allSets = cat(2, trackerImgPts_distorted{:});
            nanRows = any(any(isnan(allSets), 3), 2);
            for i = 1:length(camIDs)
                trackerImgPts_distorted{i} = trackerImgPts_distorted{i}(~nanRows, :);
            end
            % pos_truth = pos_cell{5}(interval,:)/100;
            % pos_truth = pos_truth(~nanRows, :);
            
            for i = 1:length(camIDs)
             trackerImgPts_undistorted{i} = undistortPoints(trackerImgPts_distorted{i},self.cameras(i).intrinsic);
            end


            P = cell(length(length(camIDs)),1);pts = cell(length(length(camIDs)),1);
            for i = 1:length(camIDs)
            P{i} = cameraProjection(self.cameras(i).intrinsic,self.cameras(i).extrinsic);
            pts{i} = trackerImgPts_undistorted{i};
            end
            [points3D,nan_idx] = self.triangulateDLTnan(pts,P);axis equal;
            points3D = (dcm('z',-pi/2)*points3D(:,1:3)')';
            
            
            figure;
            hold on
            plot3(points3D(:,1),points3D(:,2),points3D(:,3),'.-')
            scatter3(points3D(1,1),points3D(1,2),points3D(1,3),'gs')
            
                        

          
        end


        function [Xw,p_nan_idx] = triangulateDLTnan(self,undistortedImgPts,P_all)
            % 3D reconstruction by direct linear transformation (DLT)
            % given known image points and corresponding camera matrix P
            % at least two views/cameras
            % return world 3D corrdinates in meters
            
            % Xiaofeng Ma @ Aalto University 
            % 09/2023
            

            
            N_cam = length(undistortedImgPts);
            N = length(undistortedImgPts{1});
            X = zeros(4,N);
            p_nan_idx = [];
            
            % A = zeros(2*CAM,4);
            
            for j = 1:N          
                % compute jth 3D coordinates by N correspondences 
                A = [];
            
                for i = 1:N_cam
                    x = [];
                    x = undistortedImgPts{i}(j,:);
                    
                    if ~isnan(x)
                        P = P_all{i};
                        p1 = P(1,:)';p2 = P(2,:)';p3 = P(3,:)';
                    
                        A = [A;x(2)*p3'-p2';
                               p1'-x(1)*p3'];
                    end
                end
            
            if isempty(A)
                X(:,j) = NaN([4,1]);
                p_nan_idx = [p_nan_idx;j];
            else
                [U,S,V] = svd(A);
                X(:,j) = V(:,size(V,2));
            end
            
            end
            Xw = (X./X(end,:))';
            
            figure;plot3(Xw(:,1),Xw(:,2),Xw(:,3))
            xlabel('X (mm)');ylabel('Y (mm)');zlabel('Z (mm)')
            
            % figure;scatter(p_nan_idx,1);
            disp(['Total correspondences: ',num2str(N)])
            disp(['Nan correspondences: ',num2str(length(p_nan_idx))])
            
        end
        function [Xt, stress_all, iter_converg] = smacof_mds(self)
            % SMACOF-MDS: Performs multidimensional scaling using the SMACOF algorithm.
            %
            % Inputs:
            %   D         - (n x n) dissimilarity matrix (must be symmetric and zero-diagonal)
            %   X0        - initial configuration matrix
            %
            % Outputs:
            %   X         - (n x p) configuration matrix (embedding)
            %   stress_all    - all stress value
            % Xiaofeng Ma
            
            D = self.distanceMatrixProcessed;
            X0 = self.referenceCoords_InitialGuess; 
            W = self.distanceWeights;

            max_iter = self.config.maxIteration;
            tol = self.config.tolerence;
            
            % Validate D
            if any(diag(D) ~= 0)
                error('Dissimilarity matrix must have zeros on the diagonal.');
            end
            
            stress_all = zeros(max_iter,1);
            
            % Initialization
            stress = inf;
            Xt = X0;
            
            flag = 1;iter_converg = max_iter;
            
            V = computeV(W);         
            
            for iter = 1:max_iter
                % fprintf('Iteration %d:\n', iter);
            
                B = computeB(Xt,D,W);
                X_Guttman = pinv(V) * (B * Xt);
            
                switch self.config.method
                    case "GD"
                        func = @(in,alpha) in.Y + alpha * in.direction;
                        grad = 2 * V * Xt - 2 * B * Xt;
                        para = struct('Y', Xt, 'direction', grad, 'D', D, 'W', W);
                        a = -5; b = 2; tol = 0.001;
            
                        [best_lambda, X_new, best_stress, search_count] = golden_section_search(func, para, a, b, tol);
           
            
                        % figure(trail*100);subplot(3,3,1);hold on
                        % scatter(iter, best_lambda, '.');
                        % 
                        % subplot(3,3,4);hold on
                        % scatter(iter, search_count, '.');
            
            
                    case "linesearch"
                        func = @(in,alpha) in.Y + alpha * in.direction;
            
                        delta = X_Guttman - Xt;
                        para = struct('Y', Xt, 'direction', delta, 'D', D, 'W', W);
                        a = -2; b = 5; tol = 0.001;
            
                        [best_lambda, X_new, best_stress, search_count] = golden_section_search(func, para, a, b, tol);
            
                        % figure(trail*100);subplot(3,3,2);hold on
                        % scatter(iter, best_lambda, '.');
                        % 
                        % subplot(3,3,5);hold on
                        % scatter(iter, search_count, '.');
            
            
                    case "LM"
            
                        para = struct('Y', Xt, 'D', D, 'W', W, 'B', B, 'V', V);
                        func = @(in,alpha) pinv(in.V + alpha * eye(size(in.D, 1))) * (in.B*in.Y + alpha*in.Y);
                        a = -10; b = 10; tol = 0.001;
            
                        [best_lambda, X_new, best_stress, search_count] = golden_section_search(func, para, a, b, tol);

            
                        % figure(trail*100);subplot(3,3,3);hold on
                        % scatter(iter, best_lambda, '.');
                        % 
                        % subplot(3,3,6);hold on
                        % scatter(iter, search_count, '.');
            
            
                    otherwise
                        X_new = X_Guttman;
                        best_stress = compute_stress(D, W, X_new);
            
                end
            
                stress_new = best_stress;
                if stress_new<0.01 && flag == 1
                    flag = 0;
                    iter_converg = iter;
                end
            
                if abs(stress - stress_new) < tol && stress_new < 0.001
                    break;
                end
            
                Xt = X_new;
                stress = stress_new;
                stress_all(iter,1) = stress;
            
            end
        
        
        % fprintf('SMACOF converged in %d iterations with final stress: %.6f\n', iter, stress);
        
            function stress = compute_stress(D, W, X)
            dist = squareform(pdist(X));
            stress = sum(sum(W .* ((D - dist).^2))) / 2;% sqrt(sum(sum(W .* ((D - dist_new).^2))) / 2) / sum(W(:) .* D(:).^2));
            end
            
            
            function B = computeB(X,D,W)
            
            epsilon = 1e-12;
            
            dist = squareform(pdist(X));
            dist(dist < epsilon) = epsilon;
            B = - W .* D ./ dist;
            B(1:size(D, 1)+1:end) = 0;
            B = B - diag(sum(B, 2));
            
            end
            
            function V = computeV(W)
            
            V = diag(sum(W, 2)) - W;
            
            end
            
            function [best_alpha, X_new, best_stress, search_count] = golden_section_search(fx, in, a, b, tol)
            % Inputs:
            % D, W, Y, direction: Problem-specific inputs
            % a, b: Initial interval
            % tol: Tolerance for convergence
            
            search_count = 0;
            
            gr = (sqrt(5) - 1) / 2; % Golden ratio
            c = b - gr * (b - a);
            d = a + gr * (b - a);
            
            while abs(c - d) > tol
                X_c = fx(in,c);
                X_d = fx(in,d);
                stress_c = compute_stress(in.D, in.W, X_c);
                stress_d = compute_stress(in.D, in.W, X_d);
            
                search_count = search_count + 2;
            
                if stress_c < stress_d
                    b = d;
                else
                    a = c;
                end
            
                c = b - gr * (b - a);
                d = a + gr * (b - a);
            end
            
            best_alpha = (a + b) / 2; % Optimal step size
            X_new = fx(in,best_alpha);
            best_stress = compute_stress(in.D, in.W, X_new);
            search_count = search_count + 1;
            end
        end



        function detectReflectiveMaker(self,camID)
            if ischar(camID)
                camID = double(camID) - double('A') + 1;
            end
            % Define paths and image
            undistortedImage = self.cameras(camID).referenceImage_undistorted;
      
            
            marker_pos = []; % Initialize marker positions
            

            % % Display the image and allow ROI selection
            % figure, imshow(Image_undistorted);
            % title('Select the ROI using brush');
            % h = drawfreehand(gca, 'Closed', true); % Freehand ROI selection
            % roiMask = createMask(h); % Create binary mask from ROI
            % roiMask = gather(roiMask);
            % close;
            roiMask = self.selectROIsmear(undistortedImage, 50);
            
            
            % Apply the mask to the image
            frame = bsxfun(@times, undistortedImage, cast(roiMask, 'like', undistortedImage));
            
            % Gaussian blur
            gs_frame = imgaussfilt(frame, 1);
            
            % Convert to grayscale
            gray = rgb2gray(gs_frame);
            
            % Threshold the grayscale image
            white_mask = gray > 230;
            white_mask = imdilate(white_mask, strel('disk', 1)); % Expand by 1 pixel
            figure;imshow(white_mask);pause(2);close
            % Find contours
            contours = bwboundaries(white_mask, 'noholes');
            % figure;hold on
            if ~isempty(contours)
                for i = 1:length(contours)
                    c = contours{i};
                    % Fit a minimum area rectangle
                    msk = poly2mask(c(:,2), c(:,1), size(white_mask,1), size(white_mask,2));              
                    rect = regionprops(msk, 'BoundingBox', 'Area');
                    box = rect.BoundingBox;
                    width = box(3);
                    height = box(4);
                    aspect_ratio = width / height;

                    % Check if the contour is approximately a square
                    if rect.Area > self.markerDetect_sizeLimit(1) && rect.Area < self.markerDetect_sizeLimit(2)...  
                        && aspect_ratio >= self.markerDetect_ratioLimit(1) %&& aspect_ratio <= self.markerDetect_ratioLimit(2)
                        centerX = box(1) + width / 2;
                        centerY = box(2) + height / 2;
                        marker_pos = [marker_pos; centerX, centerY];
                        % Draw the rectangle on the image
                        gs_frame = insertShape(gs_frame, 'Rectangle', box, 'Color', 'red', 'LineWidth', 2);
                        disp(['Square detected at: ', num2str(centerX), ', ', num2str(centerY)]);
                    else
                        disp('Not a square or outlier');
                        disp(['Aspect ratio: ', num2str(aspect_ratio), ', Contour area: ', num2str(rect.Area)]);
                    end
                    % stem(i,aspect_ratio,"red")
                    % scatter(i,rect.Area,'blue')
                end
            else
                disp('No contours detected');
            end
            
            % Draw circles on the detected marker positions
            marked_frame = gs_frame;
            for i = 1:size(marker_pos, 1)
                marked_frame = insertShape(marked_frame, 'Circle', [marker_pos(i,1), marker_pos(i,2), 2], ...
                    'Color', 'blue', 'LineWidth', 2);
            end
            
            % Display the final image
            figure
            imshow(marked_frame);
            title('Detected Markers');
            pause(2);close;

            self.cameras(camID).referenceImagePoints = marker_pos;
            self.cameras(camID).referenceImage_detected = marked_frame;
            

           

        end
        function mask = selectROIsmear(~,image, brushSize)
        % INTERACTIVEBRUSHSELECTION Allows user to paint a mask on an image
        %   mask = interactiveBrushSelection(image, brushSize)
        %   Inputs:
        %       image - the input image (RGB or grayscale)
        %       brushSize - radius of the painting brush in pixels
        %   Output:
        %       mask - binary mask of the painted area
        
            % Initialize variables
            drawing = false;
            hFig = figure('Name', 'Select the ROI using brush - Press Q to quit', ...
                         'NumberTitle', 'off', ...
                         'WindowKeyPressFcn', @keyPressCallback);
            hIm = imshow(image);
            hold on;
            
            % Create initial blank mask
            mask = false(size(image,1), size(image,2));
            hMask = imshow(zeros(size(image)));
            set(hMask, 'AlphaData', mask*0.5); % Show mask with transparency
            
            % Set up mouse callbacks
            set(ax, 'WindowButtonDownFcn', @mouseDownCallback);
            set(ax, 'WindowButtonMotionFcn', @mouseMoveCallback);
            set(ax, 'WindowButtonUpFcn', @mouseUpCallback);
            
            % Store data in figure's UserData
            figData = struct(...
                'drawing', drawing, ...
                'mask', mask, ...
                'brushSize', brushSize, ...
                'hMask', hMask);
            set(ax, 'UserData', figData);
            
            % Wait for user to finish (press 'q')
            uiwait(hFig);
            
            % After figure is closed, get the final mask
            if ishandle(hFig)
                figData = get(ax, 'UserData');
                mask = figData.mask;
                close(hFig);
            else
                mask = [];
            end
            
            % Callback functions
            function mouseDownCallback(~, ~)
                figData = get(gcf, 'UserData');
                figData.drawing = true;
                set(gcf, 'UserData', figData);
                updateMask();
            end
        
            function mouseMoveCallback(~, ~)
                figData = get(gcf, 'UserData');
                if figData.drawing
                    updateMask();
                end
            end
        
            function mouseUpCallback(~, ~)
                figData = get(gcf, 'UserData');
                figData.drawing = false;
                set(gcf, 'UserData', figData);
            end
        
            function keyPressCallback(~, event)
                if strcmpi(event.Key, 'q')
                    uiresume(gcf);
                end
            end
        
            function updateMask()
                figData = get(gcf, 'UserData');
                currentPoint = get(gca, 'CurrentPoint');
                x = round(currentPoint(1,1));
                y = round(currentPoint(1,2));
                
                % Create brush circle
                [xx, yy] = meshgrid(1:size(figData.mask,2), 1:size(figData.mask,1));
                circle = (xx - x).^2 + (yy - y).^2 <= figData.brushSize^2;
                
                % Add to mask
                figData.mask = figData.mask | circle;
                
                % Update display
                set(figData.hMask, 'AlphaData', figData.mask*0.5);
                set(gcf, 'UserData', figData);
            end
        end






    end




end