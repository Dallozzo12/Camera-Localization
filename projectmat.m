close all; clear all;

fid = fopen('/home/dallo/Desktop/LA SAPIENZA/SECOND YEAR/FIRST SEMESTER/PROBABILISTIC ROBOTICS/Project/01AB-3DCamLocalization/1b-3D-Camera-Localization.txt');
tline = fgetl(fid);
while ischar(tline)
    disp(tline)
    tline = fgetl(fid);
end



fclose(fid);