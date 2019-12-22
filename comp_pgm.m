%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Compare 2 pgm images            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;

% load the images
ground_truth = imread('2019-12-12-12-54-25.pgm');
map = imread('2019-11-18-15-32-44.pgm');

% get the data as a 1D array
ground_truth = ground_truth(:);
map = map(:);

% pixel values for every state
unknown = 205;
occupied = 0;
free = 254;

% get total positives, negatives and unknowns
total_occupied = sum(ground_truth == occupied);
total_free = sum(ground_truth == free);
total_unknown = sum(ground_truth == unknown);

false_positive = 0;
true_positive = 0;
false_negative = 0;
true_negative = 0;
true_unknown = 0;
false_unknown = 0;

% compare the two maps
for i = 1:length(ground_truth)
    if map(i) == occupied && ground_truth(i) == occupied
        true_positive = true_positive + 1;
    elseif map(i) == occupied && ground_truth(i) ~= occupied
        false_positive = false_positive + 1;
    elseif map(i) == free && ground_truth(i) == free
        true_negative = true_negative + 1;
    elseif map(i) == free && ground_truth(i) ~= free
        false_negative = false_negative + 1;
    elseif map(i) == unknown && ground_truth(i) == unknown
        true_unknown = true_unknown + 1;
    elseif map(i) == unknown && ground_truth(i) ~= unknown
        false_unknown = false_unknown + 1;
    end
end

% calculate the rates
TPR = true_positive / total_occupied;
FPR = false_positive / total_free;
OE = (false_positive + false_negative + false_unknown) / length(ground_truth);