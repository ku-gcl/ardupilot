clear all;
%% FileData = load("../testData/fltTest.mat");
%% csvwrite("fltTest.csv", FileData.M);

M = load("../testData/fltTest.mat");
csvwrite("fltTest.csv", M.array);
