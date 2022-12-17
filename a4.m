%% Set up the environment

mat = zeros(100, 100);

for i = 1:100
    for j = 1:100
        if (i - 27)^2 + (j-27)^2 <= 200
            mat(i,j) = 1;
        end
    end
end



mat(30:50,[22:23 26:27 30:31 34:35]) = 1;

mat(65:85,65:85) = 1;

mat = logical(mat);

mat(1,:) = 1;
mat(end,:) = 1;
mat(:,1) = 1;
mat(:,end) = 1;
R=255*(~mat);
G=255*(~mat);
B=255*(~mat);
rawImg = cat(3,R,G,B);
imshow(rawImg);
hold on;
 area([0.01 0.01],'FaceColor','k');
  lgd = legend('Obstacle','Position',[0.45 0.1 0.1 0.2]);
    lgd.FontSize = 3;
title("Demo Environment with Non-rectilinear Obstacle",'FontSize', 4);
exportgraphics(gcf,'demo4_envir.png','Resolution',600)

%% Find obstacle and apply mask

shapeList = findShape(mat);

rectMat = mat;
mask = ones(size(mat));

for i = 1:length(shapeList)
    rectMat(shapeList(i).yMin:shapeList(i).yMax, ...
        shapeList(i).xMin:shapeList(i).xMax) = 1;
    mask(shapeList(i).yMin:shapeList(i).yMax, ...
        shapeList(i).xMin:shapeList(i).xMax) = 0;
end

rectMat(1,:) = 1;
rectMat(end,:) = 1;
rectMat(:,1) = 1;
rectMat(:,end) = 1;

nonRectMat = mat + mask;

%% Slice the environment into free segments
rectSegmentList = {};
rectSegmentList = slice(rectMat,rectSegmentList);

nonRectSegList = {};
nonRectSegList = slice(nonRectMat, nonRectSegList);

disp("done slicing");

%% Fuse the free segments to free Cells
freeCellList = [];

bigList = [];
smallList = [];

bigList = fuse(bigList,rectSegmentList);
smallList = fuse(smallList,nonRectSegList);

freeCellList = [bigList smallList];

disp("done fusing");




% the result;
mat(1,:) = 1;
mat(end,:) = 1;
mat(:,1) = 1;
mat(:,end) = 1;

R=255*(~mat);
G=255*(~mat);
B=255*(~mat);
rawImg = cat(3,R,G,B);

for i = 1:length(freeCellList)
   
   
   R(freeCellList(i).yStart : freeCellList(i).yEnd -1,freeCellList(i).xStart) = 0;
   R(freeCellList(i).yStart : freeCellList(i).yEnd -1,freeCellList(i).xEnd - 1) = 0;
   R(freeCellList(i).yStart,freeCellList(i).xStart : freeCellList(i).xEnd - 1) = 0;
   R(freeCellList(i).yEnd -1,freeCellList(i).xStart : freeCellList(i).xEnd - 1) = 0;
 
   B = R;
   
  
end

figure(1);
imgWithBorder= cat(3,R,G,B);
% plot result
imshow(imgWithBorder)
disp("done plotting");
%area([0.01 0.01],'FaceColor','k');
%area([0.02 0.02],'FaceColor','g');
%lgd = legend('Obstacle','Free Cell Border','Position',[0.45 0.1 0.1 0.2]);
%lgd.FontSize = 3;
%exportgraphics(gcf,'demo4_border.png','Resolution',600)

%% Assign the cells

bigCellList1 = [];
smallCellList1 = [];
bigSize = 10;
smallSize = 1;

for i = 1:length(freeCellList)
    if freeCellList(i).xLength < bigSize || freeCellList(i).yLength < bigSize
        smallCellList1 = [smallCellList1 freeCellList(i)];
    else
        bigCellList1 = [bigCellList1 freeCellList(i)];
    end
    
end

%% big & small agent Moving

bigCellList2 = flip(bigCellList1);
smallCellList2 = flip(smallCellList1);

group1 = agentGroup(4,[10,1,10,1],{bigCellList1,smallCellList1,bigCellList2,smallCellList2},...
    [[0 0 128];[255 0 0];[0 255 0];[0 0 128]],rawImg);

disp('set up completed');

[group1,list] = initializeAgents(group1);

disp('initialization completed');

result = 0;

bigPath1 = [list(1,:)];
smallPath1 = [list(2,:)];
bigPath2 = [list(3,:)];
smallPath2 = [list(4,:)];


loop = 1; 

while result ~= 1
    [group1,result,list] = moveOneStep(group1);
    img = getImg(group1);
    hold on;
    imshow(img);
    title("2D Demo with 2 big agents and 2 small agents",'FontSize', 4);
    bigPath1 = [bigPath1;list(1,:)];
    smallPath1 = [smallPath1;list(2,:)];
    bigPath2 = [bigPath2;list(3,:)];
    smallPath2 = [smallPath2;list(4,:)];
    
    area([0.01 0.01],'FaceColor','k');
    plot(list(1,1),list(1,2),'.','MarkerSize',15);
    plot(list(2,1),list(2,2),'.', 'MarkerSize', 7.5);
    plot(list(3,1),list(3,2),'.','MarkerSize',15);
    plot(list(4,1),list(4,2),'.', 'MarkerSize', 7.5);
    
    plot(bigPath1(:,1),bigPath1(:,2),'MarkerSize',1,'Color','black');
    plot(smallPath1(:,1),smallPath1(:,2),'MarkerSize',1,'Color','black');
    plot(bigPath2(:,1),bigPath2(:,2),'MarkerSize',1,'Color','black');
    plot(smallPath2(:,1),smallPath2(:,2),'MarkerSize',1,'Color','black');
    
    lgd = legend('Obstacle','agent I with 10 units width',...
        'agent II with 1 unit width',...
        'agent III with 10 unit width',...
        'agent IV with 1 unit width',...
        'agent path','Position',[0.45 0.1 0.1 0.2]);
    lgd.FontSize = 3;
    

    
    filename = sprintf('%s_%d%s','demo',loop,'.png');
    loop = loop + 1;
    pause(0.5);
    %exportgraphics(gcf,filename,'Resolution',600);
 
    
   
    
end

finalImg = getImg(group1);
figure(2);
imshow(finalImg);

%% Helper Function Definitions

function shapeList = findShape(mat)
    shapeList = [];
    [row,col] = find(mat);
    
    for a = 1:length(row)
        shapeContained = 0;
        
        %check if previous shape contain this obstacle
        for b = 1 : length(shapeList)
            if shapeList(b).yMin <= row(a) && shapeList(b).yMax >= row(a) && ...
                    shapeList(b).xMin <= col(a) && shapeList(b).xMax >= col(a)
                shapeContained = 1;
                break;
            end
        end
        
        if shapeContained
            continue;
        else
            newShape = obstacle();
            
            newShape.xMin = col(a);
            newShape.xMax = col(a);
            newShape.yMin = row(a);
            newShape.yMax = row(a);
            
            newShape = findNeighbour(mat,newShape);
            
            shapeList = [shapeList newShape];
        end
    end

end

function shape = findNeighbour(mat, shape)
    queue = [shape.yMin,shape.xMin];
    
    existMat = zeros(size(mat),'like',mat);
    
    existMat(shape.yMin,shape.xMin) = 1; %this mat will mark if a node is in the queue or not
    
    while ~isempty(queue)
        current = queue(1,:);
        
        currentRow = current(1);
        currentCol = current(2);
        
        % update the shape's min and max 
        if currentRow < shape.yMin
            shape.yMin = currentRow;
        elseif currentRow > shape.yMax
            shape.yMax = currentRow;
        end
        
        if currentCol < shape.xMin
            shape.xMin = currentCol;
        elseif currentCol > shape.xMax
            shape.xMax = currentCol;
        end
        
        queue(1,:) = []; % pop the first element;
        
        for i = 1:8
            nextRow = currentRow;
            nextCol = currentCol;
            
            if i == 1
                nextRow = nextRow + 1;
            elseif i == 2
                if nextRow == 1
                    continue;
                end
                nextRow = nextRow - 1;
            elseif i == 3
                nextCol = nextCol + 1;
            elseif i == 4
                if nextCol == 1
                    continue;
                end
                nextCol = nextCol - 1;
            elseif i == 5
                nextRow = nextRow + 1;
                nextCol = nextCol + 1;
            elseif i == 6
                if nextCol == 1
                    continue;
                end
                nextRow = nextRow + 1;
                nextCol = nextCol - 1;
            elseif i == 7
                if nextRow == 1
                    continue;
                end
                nextRow = nextRow - 1;
                nextCol = nextCol + 1;
            else
                if nextRow == 1
                    continue;
                end
                if nextCol == 1
                    continue;
                end
                nextRow = nextRow - 1;
                nextCol = nextCol - 1;
            end
                
            if mat(nextRow,nextCol) == 1 && existMat(nextRow,nextCol) == 0
                 existMat(nextRow,nextCol) = 1;
                 queue = [queue ; [nextRow, nextCol]];
                
            end
                
        end
    end
end

function d = findFreeSpace(c,i)
    tuple = size(c);
    rowSize = tuple(1);
    columnSize = tuple(2);
    d = 1;
    
    if rowSize < i
        d = 1;
    else
        for j = 1:columnSize
            if isobject(c{i,j}) == 0
                d = j;
                break;
            end
        end
    
        if d ==1 && columnSize ~= 0
            d = columnSize + 1;
        end
    end
end


function segmentList = slice(mat,segmentList)
    
    for i = 1 : length(mat)
        segmentConnectivity = 0;
        for j = 1 : length(mat(1,:))
            if mat(j,i) == 0
                if segmentConnectivity == 1
                    lastIndex = findFreeSpace(segmentList,i) - 1;
                    a = segmentList{i, lastIndex};
                    a.yEnd = j +1;
                    %disp('i');
                    i;
                    %disp('j');
                    j;
                    a.length = a.yEnd - a.yStart;
                    segmentList{i,lastIndex} = a;
                else
                    a = Segment;
                    a.x = i;
                    a.yStart = j;
                    a.yEnd = j + 1;
                    a.length = a.yEnd-a.yStart;
                    newIndex = findFreeSpace(segmentList,i);
                    segmentList{i,newIndex} = a;
                    segmentConnectivity = 1;
                end
            else
                segmentConnectivity = 0;
            end
        end
    end

end


function freeCellList = fuse(freeCellList,segmentList)
    
    while nnz(cellfun(@isobject,segmentList)) ~= 0
        [row, col] = find(cellfun(@isobject,segmentList),1);

        %Initiate a free cell
        newCell = FreeCell;
        currentSegment = segmentList{row,col};
        segmentList{row,col} = 0;
        newCell.xStart = currentSegment.x;
        newCell.xEnd = currentSegment.x + 1;
        newCell.yStart = currentSegment.yStart;
        newCell.yEnd = currentSegment.yEnd;
        newCell.xLength = 1;
        newCell.yLength = currentSegment.length;

        [result,newCell,segmentList] = fuseable(newCell,segmentList);

        while result ~= 0
            [result,newCell,segmentList] = fuseable(newCell,segmentList);
        end

        freeCellList = [freeCellList, newCell];
    end

end


%This function will check if next slice have fusable segments to the
%current free cell.
%If found, this will modify the segment being fused as well as the
%currentCell
function [t,currentCell,segmentList] = fuseable(currentCell,segmentList)
    t = 0;
    exploreX = currentCell.xEnd;
    tuple= size(segmentList);
    rowSize = tuple(1);
    columnSize = tuple(2);
    
    if exploreX > rowSize
        return
    end

    
    for i = 1:columnSize
        if isobject(segmentList{exploreX,i}) == 0
            continue;
        end
        
        if segmentList{exploreX,i}.length < currentCell.yLength
            continue;
        elseif segmentList{exploreX,i}.yStart <= currentCell.yStart ...
                && segmentList{exploreX,i}.yEnd >= currentCell.yEnd
            
            before = currentCell.yStart - segmentList{exploreX,i}.yStart;
            after = segmentList{exploreX,i}.yEnd - currentCell.yEnd;
            
            if before == 0 && after == 0
                segmentList{exploreX,i} = 0;
            elseif before > 0 
                segmentList{exploreX,i}.yEnd = currentCell.yStart;
                segmentList{exploreX,i}.length = before;
            elseif after > 0
                segmentList{exploreX,i}.yStart = currentCell.yEnd;
                segmentList{exploreX,i}.length = after;
            end
            
            currentCell.xLength = currentCell.xLength + 1;
            currentCell.xEnd = exploreX + 1;
            t = 1;
        end
        
    end
    
end



