function [lap_time time_elapsed velocity acceleration lateral_accel gear_counter path_length weights distance] = lap_information(path_positions)
global path_boundaries_ax r_min r_max cornering accel grip deccel lateral...
    shift_points top_speed shift_time

% See "lap_information.m" for code annotations, I am too lazy to do it
% twice
interval = 5;
sections = 3000;
VMAX = top_speed;
t = 1:1:length(path_positions);
clear path_points
for i = 1:1:length(path_positions)
    coeff = path_boundaries_ax(i,1:2);
    x2 = max(path_boundaries_ax(i,3:4));
    x1 = min(path_boundaries_ax(i,3:4));
    position = path_positions(i);
    x3 = x1+position*(x2-x1);
    y3 = polyval(coeff,x3);
    path_points(i,:) = [x3 y3];             
end

x = linspace(1,t(end-1),sections);
ppv = pchip(t,path_points');
vehicle_path = ppval(ppv,x);
path_length = arclength(vehicle_path(1,:),vehicle_path(2,:));
%figure
%fnplt(ppv)
% x = linspace(1,t(end-1),1000);
% ppv = interp1([1:length(path_points)],path_points,x,'makima');
% vehicle_path = ppv';

[L,R,K] = curvature(vehicle_path');
%% Traverse the track
track_points = vehicle_path;
track_points = [track_points(:,length(vehicle_path)-2) track_points(:,1:end-1)];
[LT,RT,KT] = curvature(track_points');
KT = KT(:,2);
KT = KT(~isnan(RT));
RT = RT(~isnan(RT));
for i = 1:length(RT)
    segment(i) = i;
    r = max(r_min,RT(i));
    r = min(r,r_max);
    RT(i) = r;
    Vmax(i) = min(VMAX,fnval(cornering,r));
    x1(i) = track_points(1,i+1);
    x2(i) = track_points(1,i+2);
    y1(i) = track_points(2,i+1);
    y2(i) = track_points(2,i+2);
    dist(i) = sqrt((x1(i)-x2(i))^2+(y2(i)-y1(i))^2);
end
%%
% initiate reverse sim
% count = length(segment)*interval+1;
%     v = 20;
%     vel = v;
% for i = length(segment):-1:1
%     d = dist(i);
% 
%     r = RT(i);
% 
% 
%     vmax = min(116,fnval(cornering,r));
%     if vmax < 0
%         vmax = VMAX;
%     end
%     AX = fnval(deccel,vel);
%     AY = fnval(lateral,vel);
%     dd = d/interval;
%     for j = 1:1:interval
%         count = count-1;
%         ay_r(count) = vel^2/(r*32.2);
%         if vel < vmax
%             ax_r(count) = AX*(1-(min(AY,ay_r(count))/AY)^2);
%             tt = roots([0.5*32.2*ax_r(count) vel -dd]);
%             dt_r(count) = max(tt);
%             dv = 32.2*ax_r(count)*dt_r(count);
%             dvmax = vmax-vel;
%             dv_r(count) = min(dv,dvmax);
%             v_r(count) = vel+dv_r(count); 
%             vel = v_r(count);
%         else
%             vel = vmax;
%             dt_r(count) = dd/vel;
%             ax_r(count) = 0;
%             v_r(count) = vel;
%             dv_r(count) = 0;
%         end
%     end
% 
%     
% end
%% Initiate forward sim

count = 0;
    v = 30;
    vel = v;
    gears = find((shift_points-vel)>0);
    gear = gears(1)-1;
    newgear = gear;
    time_shifting = 0;
for i = 1:1:length(segment)
    d = dist(i);

    r = RT(i);
    %gear = newgear;
    gears = find((shift_points-vel)>0);
    newgear = gears(1)-1;
    
    if newgear > gear
        shifting = 1;
    else
        shifting = 0;
    end

    vmax = min(VMAX,fnval(cornering,r));
    if vmax < 0
        vmax = VMAX;
    end
    AX = fnval(accel,vel);
    AY = fnval(lateral,vel);
    dd = d/interval;
    for j = 1:1:interval
        count = count+1;
        vehicle_gear(count) = gear;
        ay_f(count) = vel^2/(r*32.2);
        if shifting == 1 & vel < vmax;
            dt_f(count) = dd/vel;
            time_shifting = time_shifting+dt_f(count);
            ax_f(count) = 0;
            v_f(count) = vel;
            dv_f(count) = 0;
            vel = vel;
        elseif vel < vmax
            ax_f(count) = AX*(1-(min(AY,ay_f(count))/AY)^2);
            tt = roots([0.5*32.2*ax_f(count) vel -dd]);
            dt_f(count) = max(tt);
            dv = 32.2*ax_f(count)*dt_f(count);
            dvmax = vmax-vel;
            dv_f(count) = min(dv,dvmax);
            v_f(count) = vel+dv_f(count); 
            vel = v_f(count);
            gears = find((shift_points-vel)>0);
            newgear = gears(1)-1;
            if newgear > gear
                shifting = 1;
            end
        else
            vel = vmax;
            dt_f(count) = dd/vel;
            ax_f(count) = 0;
            v_f(count) = vel;
            dv_f(count) = 0;
        end
        if time_shifting > shift_time
            shifting = 0;
            time_shifting = 0;
            gear = newgear;
        end
    end
    if shifting == 1
        gear = gear;
    else
        gear = newgear;
    end
end

dtot = 0;    
for i = 1:1:count
    j = ceil(i/interval);
    dd = dist(j)/interval;
    dtot = dtot+dd;
    distance(i) = dtot;
end
%% Re run, with new starting velocity
V0 = v_f(end);
% initiate reverse sim
count = length(segment)*interval+1;
    v = V0;
    vel = v;
for i = length(segment):-1:1
    d = dist(i);

    r = RT(i);


    vmax = min(VMAX,fnval(cornering,r));
    if vmax < 0
        vmax = VMAX;
    end
    AX = fnval(deccel,vel);
    AY = fnval(lateral,vel);
    dd = d/interval;
    for j = 1:1:interval
        count = count-1;
        ay_r(count) = vel^2/(r*32.2);
        if vel < vmax
            ax_r(count) = AX*(1-(min(AY,ay_r(count))/AY)^2);
            tt = roots([0.5*32.2*ax_r(count) vel -dd]);
            dt_r(count) = max(tt);
            dv = 32.2*ax_r(count)*dt_r(count);
            dvmax = vmax-vel;
            dv_r(count) = min(dv,dvmax);
            v_r(count) = vel+dv_r(count); 
            vel = v_r(count);
        else
            vel = vmax;
            dt_r(count) = dd/vel;
            ax_r(count) = 0;
            v_r(count) = vel;
            dv_r(count) = 0;
        end
    end

    
end

% Initiate forward sim

% count = 0;
%     v = V0;
%     vel = v;
%     gears = find((shift_points-vel)>0);
%     gear = gears(1)-1;
%     newgear = gear;
%     time_shifting = 0;
% for i = 1:1:length(segment)
%     d = dist(i);
% 
%     r = RT(i);
%     %gear = newgear;
%     gears = find((shift_points-vel)>0);
%     newgear = gears(1)-1;
%     
%     if newgear > gear
%         shifting = 1;
%     else
%         shifting = 0;
%     end
% 
%     vmax = min(VMAX,fnval(cornering,r));
%     if vmax < 0
%         vmax = VMAX;
%     end
%     AX = fnval(accel,vel);
%     AY = fnval(lateral,vel);
%     dd = d/interval;
%     for j = 1:1:interval
%         count = count+1;
%         vehicle_gear(count) = gear;
%         ay_f(count) = vel^2/(r*32.2);
%         if shifting == 1 & vel < vmax;
%             dt_f(count) = dd/vel;
%             time_shifting = time_shifting+dt_f(count);
%             ax_f(count) = 0;
%             v_f(count) = vel;
%             dv_f(count) = 0;
%             vel = vel;
%         elseif vel < vmax
%             ax_f(count) = AX*(1-(min(AY,ay_f(count))/AY)^2);
%             tt = roots([0.5*32.2*ax_f(count) vel -dd]);
%             dt_f(count) = max(tt);
%             dv = 32.2*ax_f(count)*dt_f(count);
%             dvmax = vmax-vel;
%             dv_f(count) = min(dv,dvmax);
%             v_f(count) = vel+dv_f(count); 
%             vel = v_f(count);
%             gears = find((shift_points-vel)>0);
%             newgear = gears(1)-1;
%             if newgear > gear
%                 shifting = 1;
%             end
%         else
%             vel = vmax;
%             dt_f(count) = dd/vel;
%             ax_f(count) = 0;
%             v_f(count) = vel;
%             dv_f(count) = 0;
%         end
%         if time_shifting > shift_time
%             shifting = 0;
%             time_shifting = 0;
%             gear = newgear;
%         end
%     end
%     if shifting == 1
%         gear = gear;
%     else
%         gear = newgear;
%     end
% end

%% combine results
VD = v_f-v_r;
forw = find(VD>=0);
back = find(VD<0);
velocity = zeros(1,length(VD));
t_elapsed = 0;
classifier = [];
for i = 1:1:length(VD)
    if VD(i)<0
        velocity(i) = v_f(i);
        dtime(i) = dt_f(i);
        acceleration(i) = ax_f(i);
        lateral_accel(i) = ay_f(i);
        %shift_timer(i) = shift_timer(i);
    else
        velocity(i) = v_r(i);
        dtime(i) = dt_r(i);
        acceleration(i) = -ax_r(i);
        lateral_accel(i) = ay_r(i);
        %shift_timer(i) = 0;
    end
    t_elapsed = t_elapsed+dtime(i);
    time_elapsed(i) = t_elapsed;
end
AY_outlier = find(lateral_accel > fnval(lateral,116));
lateral_accel(AY_outlier) = fnval(lateral,116);
throttle = 0;
brake = 0;
corner = 0;
for i = 1:1:length(VD)
    if acceleration(i)>0
        throttle = throttle+acceleration(i)*dtime(i);
    elseif acceleration(i) < 0
        brake = brake-acceleration(i)*dtime(i);
    end
    corner = corner + lateral_accel(i)*dtime(i);
end
summ = throttle+brake+corner;
weights = [throttle/summ brake/summ corner/summ];
tloc = find(acceleration>.25);
t_t = sum(dtime(tloc));
bloc = find(acceleration<-.25);
t_b = sum(dtime(bloc));
cloc = find(lateral_accel>.25);
t_c = sum(dtime(cloc));
summ = t_t+t_b+t_c;
weights = [t_t/summ t_b/summ t_c/summ];
%figure
%plot(distance,velocity)

%% Plot Results
figure
for i = 1:1:length(track_points)-2
    V_plot(i) = mean(velocity(i*interval-interval+1:i*interval));
end
pointsize = 5;
scatter(track_points(1,2:end-1),track_points(2,2:end-1),100,V_plot,'marker','.')
title('2019 Michigan Autocross Simulation Track Summary')
h = colorbar;
set(get(h,'title'),'string','Velocity (V) [ft/s]');
set(gca,'XTick',[], 'YTick', [])
%% Gear Counter
for i = 1:1:length(velocity)
V = velocity(i);
    gears = find((shift_points-V)>0);
    gear = gears(1)-1;
gear_counter(i) = gear;
end
lap_time = t_elapsed;

for i = 1:1:length(lateral_accel)
    index = floor((i-1)/interval)+1;
    axis(i) = sign(KT(index));
end
lateral_accel = lateral_accel.*axis;