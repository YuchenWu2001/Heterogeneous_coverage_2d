classdef agentGroup
    properties
        agentNum {mustBeNumeric}
        agentList
        agentSizeList
        ListofCellLists
        rgbList
        agentStates
        img
    end
    
    methods
        function obj = agentGroup(num,sizeList,...
                listOfCellLists,rgbList,img)
            if nargin == 5
                obj.agentNum = num;
                obj.agentSizeList = sizeList;
                obj.ListofCellLists = listOfCellLists;
                obj.rgbList = rgbList;
                obj.img = img;
                obj.agentList = [];
                obj.agentStates = zeros(1,obj.agentNum);
                for i  = 1:num
                    obj.agentList = [obj.agentList ...
                        agent(obj.agentSizeList(i),obj.ListofCellLists{i}, ...
                        obj.rgbList(i,:))];
                end
            else
                disp("Error in initializing agent! Incorrect Number of input argments.");
            end
        end
        
        function [obj,currentLocationList] = initializeAgents(obj)
            currentLocationList = [];
            for i = 1:obj.agentNum
                [location,obj.img] = getCurrentLocation(obj.agentList(i),obj.img);
                currentLocationList = [currentLocationList;location];
            end
            
        end
        
        function result = getImg(obj)
            result = obj.img;
        end
        
        function [obj,state,currentLocationLists] = moveOneStep(obj)
             currentLocationLists = zeros(obj.agentNum,2);  
            for i = 1:obj.agentNum
                  if obj.agentStates(i) ~= 1
                      [obj.agentList(i), obj.agentStates(i),loc,obj.img] = ...
                      moveOneStep(obj.agentList(i),obj.img);
                      currentLocationLists(i,:) = loc;
                  else
                      currentLocationLists(i,:) = obj.agentList(i).currentLocation;
                  end
            end
            
            if nnz(obj.agentStates) ~= obj.agentNum
                state = 0;
            else
                state = 1;
            end
            
        end
    end
    
    
    
end
