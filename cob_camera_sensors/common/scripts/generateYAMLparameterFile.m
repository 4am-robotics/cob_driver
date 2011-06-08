% generates a YAML file for ROS containing the paramters of the camera
%
% author: Richard Bormann
%
% image_width: 640
% image_height: 480
% camera_name: left
% camera_matrix:
%   rows: 3
%   cols: 3
%   data: [1261.56689, 0, 301.99007, 0, 1257.15536, 232.40367, 0, 0, 1]
% distortion_coefficients:
%   rows: 1
%   cols: 5
%   data: [-0.21662, 0.12327, 0, 0, 0]
% rectification_matrix:
%   rows: 3
%   cols: 3
%   data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
% projection_matrix:
%   rows: 3
%   cols: 4
%   data: [1261.56689, 0, 0, 301.99007, 0, 1257.15536, 0, 232.40367, 0, 0, 1, 0]


% generates the parameter file(s) for a single camera or a stereo pair
% stereo = set 0 if a single camera is saved, otherwise it's stereo and you will receive two output files
% filenamePrefix = additional filename prefix (e.g. cob3-3)
% cameraName = the camera name in the config file, only necessary in single camera mode
function generateYAMLparameterFile(stereo, filenamePrefix, cameraName)
if (stereo ~= 0)
    saveCameraParametersYAMLStereo(filenamePrefix);
else
    saveCameraParametersYAMLSingle(filenamePrefix, cameraName);
end
end



% generates a YAML file from the calibration data after calibrating a
% single camera
function saveCameraParametersYAMLSingle(filenamePrefix, cameraName)
filename = [filenamePrefix, '_ros.yaml'];
writeParameterFile(filename, cameraName, evalin('base', 'fc'), evalin('base', 'cc'), evalin('base', 'alpha_c'), evalin('base', 'kc'));
end


% generates two YAML files from the calibration data after calibrating a
% stereo rig
function saveCameraParametersYAMLStereo(additionalFilename)
% save left camera 
filename = ['left_', additionalFilename, '_ros.yaml'];
writeParameterFile(filename, 'left', evalin('base', 'fc_left'), evalin('base', 'cc_left'), evalin('base', 'alpha_c_left'), evalin('base', 'kc_left'));

% save right camera
filename = ['right_', additionalFilename, '_ros.yaml'];
writeParameterFile(filename, 'right', evalin('base', 'fc_right'), evalin('base', 'cc_right'), evalin('base', 'alpha_c_right'), evalin('base', 'kc_right'));

end


function writeParameterFile(filename, cameraName, fc, cc, alpha_c, kc)
fid = fopen(filename, 'w');
fprintf(fid, 'image_width: %d\nimage_height: %d\ncamera_name: %s\n', evalin('base', 'nx'), evalin('base', 'ny'), cameraName);
fprintf(fid, 'camera_matrix:\n  rows: 3\n  cols: 3\n  data: [%f, %f, %f, 0, %f, %f, 0, 0, 1]\n', fc(1), alpha_c, cc(1), fc(2), cc(2));
fprintf(fid, 'distortion_coefficients:\n  rows: 1\n  cols: 5\n  data: [%f, %f, %f, %f, %f]\n', kc(1), kc(2), kc(3), kc(4), kc(5));
fprintf(fid, 'rectification_matrix:\n  rows: 3\n  cols: 3\n  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n');
fprintf(fid, 'projection_matrix:\n  rows: 3\n  cols: 4\n  data: [%f, %f, %f, 0, 0, %f, %f, 0, 0, 0, 1, 0]', fc(1), alpha_c, cc(1), fc(2), cc(2));
fclose(fid);
end