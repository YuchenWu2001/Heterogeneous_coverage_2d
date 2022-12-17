%% Set up the environment
map = binaryOccupancyMap(10, 10, 10);
occ = zeros(100, 100);

occ(1,:) = 1;
occ(end,:) = 1;
occ(:,1) = 1;
occ(:,end) = 1;

occ(50,22:79) = 1;
occ(30:70,[22:23 29:30 36:37 43:44 50:51 57:58 64:65 71:72 78:79]) = 1;

setOccupancy(map, occ)

figure
%show(map)
%title('Canvas 2')
%saveas(gcf,'demo2.png');

mat = occupancyMatrix(map);
segmentList = {};
segmentConnectivity = 0;


%% Slice the environment into free segments
for i = 1 : length(mat)
    segmentConnectivity = 0;
    for j = 1 : length(mat(1,:))
        if mat(j,i) == 0
            if segmentConnectivity == 1
                lastIndex = findFreeSpace(segmentList,i) - 1;
                a = segmentList{i, lastIndex};
                a.yEnd = j +1;
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

disp("done slicing");

%% Fuse the free segments to free Cells
freeCellList = [];

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

disp("done fusing");

%show(map);

figure(1);
hold on;


% the result;
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
hold on;
area([0.01 0.01],'FaceColor','k');
area([0.02 0.02],'FaceColor','g');
lgd = legend('Obstacle','Free Cell Border','Position',[0.45 0.1 0.1 0.2]);
lgd.FontSize = 3;
exportgraphics(gcf,'demo2_border.png','Resolution',600)

%% Assign the cells

bigCellList = [];
smallCellList = [];
bigSize = 10;
smallSize = 5;

for i = 1:length(freeCellList)
    if freeCellList(i).xLength < bigSize || freeCellList(i).yLength < bigSize
        smallCellList = [smallCellList freeCellList(i)];
    else
        bigCellList = [bigCellList freeCellList(i)];
    end
    
end

%% big & small agent Moving

group1 = agentGroup(2,[10,5],{bigCellList,smallCellList},...
    [[0 0 0];[0 0 0]],rawImg);

[group1,list] = initializeAgents(group1);

result = 0;

bigPath = [list(1,:)];
smallPath = [list(2,:)];

loop = 1;

while result ~= 1
    [group1,result,list] = moveOneStep(group1);
    img = getImg(group1);
    hold on;
    imshow(img);
    
    bigPath = [bigPath;list(1,:)];
    smallPath = [smallPath;list(2,:)];
    
    plot(list(1,1),list(1,2),'.','MarkerSize',15,'Color','cyan');
    plot(list(2,1),list(2,2),'.', 'MarkerSize', 10,'Color','red');
    plot(bigPath(:,1),bigPath(:,2),'MarkerSize',1,'Color','cyan');
    plot(smallPath(:,1),smallPath(:,2),'MarkerSize',1,'Color','red');
    lgd = legend('big agent with 1m^2 size',...
        'small agent with 0.25m^2 size',...
        'big agent path','small agent path','Position',[0.45 0.1 0.1 0.2]);
    lgd.FontSize = 3;
    
    pause(1)
    filename = sprintf('%s_%d%s','demo',loop,'.png');
    loop = loop + 1;
    %%exportgraphics(gcf,filename,'Resolution',600);
    
    
end

finalImg = getImg(group1);
figure(2);
imshow(finalImg);

%% Helper Function Definitions

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
            end
        end
    
        if d ==1 && columnSize ~= 0
            d = columnSize + 1;
        end
    end
end

%This function will check if next slice have fusable segments to the
%current free cell.
%If found, this will modify the segment being fused as well as the
%currentCell
function [t,currentCell,segmentList] = fuseable(currentCell,segmentList);
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



