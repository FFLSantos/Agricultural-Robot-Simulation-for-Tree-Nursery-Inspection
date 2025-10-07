function t = treeDetector(XY,Xmax,Ymax,R,C,RL,N,RS)
global probmap2;
%% Perform the tree identification by means of image processing

D = 2;
% Crop the image to eliminate border noise
[imap_1, jmap_1] = XYtoIJ(XY(2,1)-RL/2,XY(2,2),Xmax,Ymax,R,C); 
[imap_2, jmap_2] = XYtoIJ(XY(2,1)+RL/2,XY(2*N+1,2),Xmax,Ymax,R,C);
I1 = imcrop(probmap2,[jmap_1, imap_2, jmap_2, imap_1]);
probmap_binary = im2bw(I1,0.7); % Converting the image into a binary one
I_median = medfilt2(probmap_binary); % Using a median filter to eliminate the noise (tall grass or twigs)

tree_edge = edge(I_median,'Canny'); % Canny edge detection method
SE = strel('disk',3,0); % Structuring element
map_processed = imclose(tree_edge,SE); % Fill the empty spaces of the tree objects
r_min = 1; r_max = 40;
[centers,radius] = imfindcircles(map_processed,[r_min r_max]); % Search circular objects in the image
% that have radius between r_min and r_max
gridRes = 50/R; % Grid resolution (m/pixels)

%% Converting center coordinates and radius to meters and to the world frame
XY_m = centers*gridRes;
X_m = XY_m(:,1)+XY(2,1)-RL/2;
Y_m = XY_m(:,2)+XY(2,2)-2;
XY_m = [X_m Y_m];
rad_m = radius*gridRes;

%% Labeling the data
tolerance = 1;
row_id = [];
tree_id = [];
report = [];
row1 = []; row2 = []; row3 = [];

for i = 1:length(XY_m)
    if XY_m(i,2)>= XY(2,2)-tolerance && XY_m(i,2)<= XY(2,2)+6*D+tolerance
        if XY_m(i,1)<= XY(2,1)+RS && XY_m(i,1)>= XY(2,1)
            row_id = [row_id; 1 XY_m(i,:) 2*rad_m(i)]; 
        elseif XY_m(i,1)<= XY(3,1)+RS && XY_m(i,1)>= XY(3,1)
            row_id = [row_id; 2 XY_m(i,:) 2*rad_m(i)]; 
        elseif XY_m(i,1)<= XY(4,1)+RS && XY_m(i,1)>= XY(4,1)
            row_id = [row_id; 3 XY_m(i,:) 2*rad_m(i)];  
        end
    end
end

for i = 1:length(row_id)
    if row_id(i,1) == 1  
        if row_id(i,3)<= XY(2,2)+tolerance && row_id(i,3)>= XY(2,2)-tolerance
            tree_id = [tree_id; 1 row_id(i,:)]; 
        elseif row_id(i,3)<= XY(2,2)+D+tolerance && row_id(i,3)>= XY(2,2)+D-tolerance
            tree_id = [tree_id; 2 row_id(i,:)]; 
        elseif row_id(i,3)<= XY(2,2)+2*D+tolerance && row_id(i,3)>= XY(2,2)+2*D-tolerance
            tree_id = [tree_id; 3 row_id(i,:)]; 
        elseif row_id(i,3)<= XY(2,2)+3*D+tolerance && row_id(i,3)>= XY(2,2)+3*D-tolerance
            tree_id = [tree_id; 4 row_id(i,:)]; 
        elseif row_id(i,3)<= XY(2,2)+4*D+tolerance && row_id(i,3)>= XY(2,2)+4*D-tolerance
            tree_id = [tree_id; 5 row_id(i,:)];        
        elseif row_id(i,3)<= XY(2,2)+5*D+tolerance && row_id(i,3)>= XY(2,2)+5*D-tolerance
            tree_id = [tree_id; 6 row_id(i,:)]; 
        elseif row_id(i,3)<= XY(2,2)+6*D+tolerance && row_id(i,3)>= XY(2,2)+6*D-tolerance
            tree_id = [tree_id; 7 row_id(i,:)];             
        end
    end
    if row_id(i,1) == 2  
        if row_id(i,3)<= XY(2,2)+tolerance && row_id(i,3)>= XY(2,2)-tolerance
            tree_id = [tree_id; 1 row_id(i,:)]; 
        elseif row_id(i,3)<= XY(2,2)+D+tolerance && row_id(i,3)>= XY(2,2)+D-tolerance
            tree_id = [tree_id; 2 row_id(i,:)]; 
        elseif row_id(i,3)<= XY(2,2)+2*D+tolerance && row_id(i,3)>= XY(2,2)+2*D-tolerance
            tree_id = [tree_id; 3 row_id(i,:)]; 
        elseif row_id(i,3)<= XY(2,2)+3*D+tolerance && row_id(i,3)>= XY(2,2)+3*D-tolerance
            tree_id = [tree_id; 4 row_id(i,:)]; 
        elseif row_id(i,3)<= XY(2,2)+4*D+tolerance && row_id(i,3)>= XY(2,2)+4*D-tolerance
            tree_id = [tree_id; 5 row_id(i,:)];        
        elseif row_id(i,3)<= XY(2,2)+5*D+tolerance && row_id(i,3)>= XY(2,2)+5*D-tolerance
            tree_id = [tree_id; 6 row_id(i,:)]; 
        elseif row_id(i,3)<= XY(2,2)+6*D+tolerance && row_id(i,3)>= XY(2,2)+6*D-tolerance
            tree_id = [tree_id; 7 row_id(i,:)];             
        end
    end
    if row_id(i,1) == 3  
        if row_id(i,3)<= XY(2,2)+tolerance && row_id(i,3)>= XY(2,2)-tolerance
            tree_id = [tree_id; 1 row_id(i,:)]; 
        elseif row_id(i,3)<= XY(2,2)+D+tolerance && row_id(i,3)>= XY(2,2)+D-tolerance
            tree_id = [tree_id; 2 row_id(i,:)]; 
        elseif row_id(i,3)<= XY(2,2)+2*D+tolerance && row_id(i,3)>= XY(2,2)+2*D-tolerance
            tree_id = [tree_id; 3 row_id(i,:)]; 
        elseif row_id(i,3)<= XY(2,2)+3*D+tolerance && row_id(i,3)>= XY(2,2)+3*D-tolerance
            tree_id = [tree_id; 4 row_id(i,:)]; 
        elseif row_id(i,3)<= XY(2,2)+4*D+tolerance && row_id(i,3)>= XY(2,2)+4*D-tolerance
            tree_id = [tree_id; 5 row_id(i,:)];        
        elseif row_id(i,3)<= XY(2,2)+5*D+tolerance && row_id(i,3)>= XY(2,2)+5*D-tolerance
            tree_id = [tree_id; 6 row_id(i,:)]; 
        elseif row_id(i,3)<= XY(2,2)+6*D+tolerance && row_id(i,3)>= XY(2,2)+6*D-tolerance
            tree_id = [tree_id; 7 row_id(i,:)];             
        end
    end    
end

[~,idx] = sort(tree_id(:,2)); % Sort just the second column
sortedmat = tree_id(idx,:);
r1 = sortedmat(sortedmat(:,2)==1); l_r1 = length(r1);
r2 = sortedmat(sortedmat(:,2)==2); l_r2 = length(r2);
r3 = sortedmat(sortedmat(:,2)==3); l_r3 = length(r3);

row1 = sortedmat(1:l_r1,:); row2 = sortedmat(l_r1+1:l_r1+l_r2,:); row3 = sortedmat(l_r1+l_r2+1:l_r1+l_r2+l_r3,:);
% Sort the splitted matrices individually
[~,idx] = sort(row1(:,1)); 
row1_sorted = row1(idx,:);
[~,idx] = sort(row2(:,1)); 
row2_sorted = row2(idx,:);  
[~,idx] = sort(row3(:,1)); 
row3_sorted = row3(idx,:);
report = [row1_sorted; row2_sorted; row3_sorted];


%% Generate report

fileID = fopen('Report.txt','w');
fprintf(fileID, 'Number of the tree - Row number - X coordinate (m) - Y coordinate (m) - Diameter (m)\n\n');
fprintf(fileID,'%2d %2d %2.2f %2.2f %2.2f\n',report');

fclose(fileID);

t = report;
end


