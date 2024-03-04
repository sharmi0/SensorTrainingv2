function prMat = calc_pr(rowVec) %takes in unit normals and returns pitch and roll euler angles

%project to zy plane
[r,~] = size(rowVec);
proj_zy = [zeros(r,1) rowVec(:,2:3)];
x_sgn = sign(rowVec(:,1));
%normalize the projected vector
% proj_zy = normr(proj_zy);
proj_zy = proj_zy./(vecnorm(proj_zy'))'; %if deep learning toolbox is installed, proj_zy = normr(proj_zy)
%calculate pitch
pitch = -atan2(proj_zy(:,2),proj_zy(:,3)); %per hardware configuration, rotation about x axis has to be flipped
%calculate roll 
roll = real(x_sgn.*acos(diag(proj_zy * rowVec')));

prMat = [pitch roll];

end
