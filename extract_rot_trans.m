function [Pose] = extract_rot_trans(filename)
    % Open the file for reading
    fileID = fopen(filename, 'r');
    
    % Check if the file is opened successfully
    if fileID == -1
        error('Cannot open the file.');
    end
    
    % Read the data from the file
    data = fscanf(fileID, '%f %f %f %f\n%f %f %f %f\n%f %f %f %f\n \n', [12 Inf]);
    
    % Close the file
    fclose(fileID);

    % Transpose the data matrix
    data = data';
    
    % Initialize the cell arrays for rotations and translations
    num_matrices = size(data, 1);
    Pose = zeros(num_matrices,6);
    
    % Extract the rotation and translation matrices
    for i = 1:num_matrices
        R = [data(i,1:3);data(i,5:7);data(i,9:11)];
        t = [data(i,4);data(i,8);data(i,12)];
        T = [R,t;
            0,0,0,1];
        [euler_angles,trans] = se3_to_euler_angles_translation(T);
        Pose(i,1:3) = euler_angles;
        Pose(i,4:6) = trans';
    end
end