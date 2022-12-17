classdef agent
   properties
      size {mustBeNumeric}
      freeCellList
      currentWorkingCell
      currentLocation
      currentDirection
      rgb
   end
   methods
     function obj = agent(size,freeCellList,rgb)
        if nargin == 3
            obj.size = size;
            obj.currentWorkingCell = freeCellList(1);
            obj.freeCellList = freeCellList(2:end);
            obj.currentLocation = [obj.currentWorkingCell.xStart ...
            + floor(0.5*obj.size), obj.currentWorkingCell.yStart + floor(0.5*obj.size)];
            obj.currentDirection = 1;
            obj.rgb = rgb;
        else
            disp("Error in initializing agent! Incorrect Number of input argments.");
        end
    end
       
      function [currentLocation,img] = getCurrentLocation(obj,img)
            currentLocation = obj.currentLocation;
            
            x = obj.currentLocation(1);
            y = obj.currentLocation(2);
            
            if rem(obj.size,2) == 0
                idx1 = (x - obj.size/2): (x + obj.size/2 - 1);
                idx2 = (y - obj.size/2): (y + obj.size/2 - 1);
            else
                idx1 = (x - floor(obj.size/2)) : (x + floor(obj.size/2));
                idx2 = (y - floor(obj.size/2)) : (y + floor(obj.size/2));
            end
           update = zeros(obj.size,obj.size,3);
           update(:,:,1) = repmat(obj.rgb(1),[obj.size, obj.size]);
           update(:,:,2) = repmat(obj.rgb(2),[obj.size, obj.size]);
           update(:,:,3) = repmat(obj.rgb(3),[obj.size, obj.size]);
           
           modify = img(idx2,idx1,:);
           if nnz(modify(:) ~= 255) ==0
               img(idx2,idx1,:) = update;
           else
               img(idx2,idx1,:) = modify + update;
           end
           
            
        end

      function [obj,state,currentLocation,img] = moveOneStep(obj,img)
           state = 0; 
            if obj.currentDirection > 0
               spaceLeft = obj.currentWorkingCell.yEnd - ...
                   obj.currentLocation(2) - obj.currentDirection*ceil(0.5*obj.size);
           else 
               spaceLeft = obj.currentLocation(2) + obj.currentDirection*floor(0.5*obj.size) ...
                   - obj.currentWorkingCell.yStart;
           end

           if spaceLeft > obj.size
               obj.currentLocation(2) = obj.currentLocation(2) + obj.currentDirection * obj.size;
           elseif spaceLeft > 0
               obj.currentLocation(2) = obj.currentLocation(2) + obj.currentDirection * spaceLeft;
           elseif obj.currentLocation(1) + ceil(0.5*obj.size) < obj.currentWorkingCell.xEnd
               obj.currentDirection = -obj.currentDirection;
               xSpaceLeft = obj.currentWorkingCell.xEnd - obj.currentLocation(1) - ceil(0.5*obj.size);
               if  xSpaceLeft > obj.size
                 obj.currentLocation(1) = obj.currentLocation(1) + obj.size;
               else
                  obj.currentLocation(1) = obj.currentLocation(1) + xSpaceLeft;
               end
           else
               if ~isempty(obj.freeCellList)
                  [obj.currentWorkingCell, obj.currentLocation, obj.currentDirection, obj.freeCellList] = ...
                      findClosetEntryPoint(obj.currentLocation,obj.freeCellList,obj.size);
               else
                    state = 1;
                    currentLocation = obj.currentLocation;
                    return;
               end
           end
 
           currentLocation = obj.currentLocation;
           
           x = obj.currentLocation(1);
           y = obj.currentLocation(2);
            
           if rem(obj.size,2) == 0
               idx1 = (x - obj.size/2): (x + obj.size/2 - 1);
               idx2 = (y - obj.size/2): (y + obj.size/2 - 1);
           else
               idx1 = (x - floor(obj.size/2)) : (x + floor(obj.size/2));
               idx2 = (y - floor(obj.size/2)) : (y + floor(obj.size/2));
           end
           
   
           update = cat(3,repmat(obj.rgb(1),[obj.size, obj.size]),...
               repmat(obj.rgb(2),[obj.size, obj.size]),repmat(obj.rgb(3),[obj.size, obj.size]));
           
           modify = img(idx2,idx1,:);
           if nnz(modify(:) ~= 255) ==0
               img(idx2,idx1,:) = update;
           else
               mask1 = (modify(:,:,1) == 255);
               mask2 = (modify(:,:,2) == 255);
               mask3 = (modify(:,:,3) == 255);
               maskWhite = mask1 & mask2 & mask3;
               
               maskRGB = cat(3,maskWhite,maskWhite,maskWhite);
               modify(maskRGB) = 0;
               img(idx2,idx1,:) = modify + update;
           end
           
      end
      
   end
end

function [cell, location, direction, cellList] = findClosetEntryPoint(currLoc,cellList,size)
    minDistance = 100000000;
    direction = 1;
    %id = -1;
    
    for i = 1:length(cellList)
        entryPoint1 = [cellList(i).xStart + floor(0.5*size), cellList(i).yStart + floor(0.5*size)];
        entryPoint2 = [cellList(i).xStart + floor(0.5*size) ,cellList(i).yEnd - ceil(0.5*size)];
        
        dist1 = l2Distance(entryPoint1,currLoc);
        dist2 = l2Distance(entryPoint2,currLoc);
        
        if dist1 < minDistance
            id = i;
            location = entryPoint1;
            direction = 1;
            minDistance = dist1;
        end
        if dist2 < minDistance
            id = i;
            location = entryPoint2;
            direction = -1;
            minDistance = dist2;
        end
    end
    
    cell = cellList(id);
    cellList(id) = [];
end

function distance = l2Distance(loc1,loc2)
      distance = ((loc1(1) - loc2(1))^2 +  (loc1(2) - loc2(2))^2)^0.5;
end