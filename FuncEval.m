%% Coding by Yingyu Wang

% Evalute RMSE and MSE of 2D poses w.r.t. translation part and rotation part 

function FuncEval(Pose,Pose_GT,txt)

fid = fopen(txt,'a');

our_abs_diff_t = abs(Pose_GT(:,1:2) - Pose(:,1:2));
our_MAE_t = sum(mean(our_abs_diff_t))/2;
fprintf('Translation MAE of Our Method is %.4f\n', our_MAE_t);
fprintf(fid,'Translation MAE of Our Method is %.4f\n', our_MAE_t);

our_RMSE_t = sum(sqrt(mean((Pose_GT(:,1:2) - Pose(:,1:2)).^2)))/2;
fprintf('Translation RMSE of Our Method is %.4f\n', our_RMSE_t);
fprintf(fid,'Translation RMSE of Our Method is %.4f\n', our_RMSE_t);

% rotation error
our_diff_ang = Pose_GT(:,3) - Pose(:,3);
our_diff_ang = batchwrap(our_diff_ang);  % wrap diff_ang tp [-pi,pi]
our_abs_diff_ang = abs(our_diff_ang);
our_MAE_ang = mean(our_abs_diff_ang);
fprintf('Rotation MAE of Our Method is %.4f\n', our_MAE_ang);
fprintf(fid,'Rotation MAE of Our Method is %.4f\n', our_MAE_ang);

our_RMSE_ang = sqrt(mean((our_diff_ang).^2));
fprintf('Rotation RMSE of Our Method is %.4f\n\n', our_RMSE_ang);
fprintf(fid,'Rotation RMSE of Our Method is %.4f\n\n', our_RMSE_ang);
fclose(fid);

end