function [data] = LoadData(fname)
[A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14] = textread(fname, "%s %f %f %f %f %f %f %f %f %f %f %f %f %f");
data = [A2';A3';A4';A5';A6';A7';A8';A9';A10';A11';A12';A13';A14']';

scene2 = data(1:1:6, :);
scene3 = data(7:1:12, :);
scene1 = data(13:1:18, :);


data(1:1:6, :) = scene1;
data(7:1:12, :)= scene2;
data(13:1:18, :)= scene3;


end

