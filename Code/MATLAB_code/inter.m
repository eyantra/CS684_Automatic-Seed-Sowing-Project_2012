
clear all;
clc;
I = imread('image.jpg');
% doing image segmentation
% all the pixels values other than pink (with some threshold) are made black 
% and pixel values in the threshold are made white
for i = 1:1:1536
for j = 1:1:2048
if (I(i,j,1)>95 && I(i,j,1)<135 && I(i,j,2)>0 && I(i,j,2)<30  && I(i,j,3)>30 && I(i,j,3)<80)
BW1(i,j) = 255;
else
BW1(i,j) = 0;
end
end
end
imshow(BW1);
% distance = zeros(1,25);
% temp1 = 0;
% temp2 = 0;
% n = 1;
% k = 1;
% l = 1;
% count_h = 1;
% count_v = 1;
% flag = 0;
% while (k < 711)
%     while (l < 1575)
%         
%         if (BW1(k,l) == 255)
%             if (flag == 1)
%             distance(n) = ((temp1 - k)^2 + (temp2 - l)^2)^0.5;
%             n = n + 1;
%             end
%             temp1 = k;
%             hor(count_h) = k;
%             count_h = count_h + 1;
%             temp2 = l;
%             ver(count_v) = l;
%             count_v = count_v + 1;
%             flag = 1;
%             l = l + 40;
%         else
%             l = l + 1;
%         end
%     end
%       l = 1;
%       if (temp1 == k)
%           k = k + 40;
%       else
%       k = k + 1;
%       end
% end

% finding mean and variance
row = 1;
col = 1;
col_inc = 150;
col_lt = 150;
m = 1;
index = 1;
flag = 0;
while (index <= (2048/col_inc))
    while (col < col_lt)

        while (row < 1536)
            
                if (BW1(row,col) == 255)
                    xcor(m) = col;
                    ycor(m) = row;
                    m = m + 1;
                    flag = 1;
                    break;
                else
                    row = row + 1;
                end
        end
             if (flag == 1)
             flag = 0;
             break;
             end
             row = 1;
             col = col + 1;
    end
    col = col_lt;
    col_lt = col_lt + col_inc;
    row = 1;
    index = index + 1;
end

distance = zeros(1,(length(xcor)-1));
for i=1:1:(length(xcor)-1)
    distance(i) = ((xcor(i+1)-xcor(i))^2 + (ycor(i+1)-ycor(i))^2)^0.5;
end

dist_in_cm = (distance/141)*5;

j = 1;

for i = 1:1:length(dist_in_cm)
    if ((dist_in_cm(i) < 15) && (dist_in_cm(i) > 2))
        pdist_in_cm(j) = dist_in_cm(i);
        j = j+1;
    end
end

pdist_in_cm

mean_cm = mean(pdist_in_cm)
std_cm = std(pdist_in_cm)