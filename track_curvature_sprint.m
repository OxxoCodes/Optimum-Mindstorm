function [c,ceq] = track_curvature(path_positions)
global path_boundaries_ax track_width
tw = track_width
r_min = 14.75*2/tw;
t = 1:1:length(path_positions);

for i = 1:1:length(path_positions)
    coeff = path_boundaries_ax(i,1:2);
    x2 = max(path_boundaries_ax(i,3:4));
    x1 = min(path_boundaries_ax(i,3:4));
    position = path_positions(i);
    x3 = x1+position*(x2-x1);
    y3 = polyval(coeff,x3);
    path_points(i,:) = [x3 y3];
%     if i >= 3
%         [L,R,K] = curvature([path_points(i-2:i,1) path_points(i-2:i,2)]);
%         R = R(2);
%         while  R < r_min
%             
end
% x = linspace(1,t(end-1),length(path_points)*5);
% ppv = interp1([1:length(path_points)],path_points,x,'makima');
% vehicle_path = ppv;
% [L,R1,K] = curvature(vehicle_path);
% %R1 = R1(~isnan(R1));
% for i = 1:length(path_points)
%     for j = 1:5
%         ind =5*i-5+j;
%         radii(j) = R1(ind);
%     end
%     R(i) = min(radii);
% end
        

[L,R,K] = curvature(path_points);
R = R(~isnan(R));
R = -R+.75*r_min;
c = R;     % Compute nonlinear inequalities at x.
ceq = zeros(length(R),1);   % Compute nonlinear equalities at x.