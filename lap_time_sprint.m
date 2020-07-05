function [Time] = lap_time(path_positions)
global path_boundaries_ax r_min r_max cornering accel grip deccel lateral...
    shift_points top_speed shift_time
interval = 5;
sections = 3000;

VMAX = top_speed.top_speed;

t = 1:1:length(path_positions);
for i = 1:1:length(path_positions)
    coeff = path_boundaries(i,1:2);
    x2 = max(path_boundaries(i,3:4));
    x1 = min(path_boundaries(i,3:4));
    position = path_positions(i);
    x3 = x1+position*(x2-x1);
    y3 = polyval(coeff,x3);
    path_points(i,:) = [x3 y3];             
end

x = linspace(1,t(end-1),sections);
ppv = pchip(t,path_points');
vehicle_path = ppval(ppv,x);
% x = linspace(1,t(end-1),1000);
% ppv = interp1([1:length(path_points)],path_points,x,'makima');
% vehicle_path = ppv';

[L,R,K] = curvature(vehicle_path');
%% Traverse the track
track_points = vehicle_path;
track_points = [track_points(:,length(vehicle_path)-2) track_points(:,1:end-1)];
[LT,RT,KT] = curvature(track_points');
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
%             dt_r(count) = dd/vel;
%             ax_r(count) = 0;
%             v_r(count) = vel;
%             dv_r(count) = 0;
%             vel = vmax;
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
            dt_f(count) = dd/vel;
            ax_f(count) = 0;
            v_f(count) = vel;
            dv_f(count) = 0;
            vel = vmax;
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
shiftercount = 0;
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
            dt_r(count) = dd/vel;
            ax_r(count) = 0;
            v_r(count) = vel;
            dv_r(count) = 0;
            vel = vmax;
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
%             dt_f(count) = dd/vel;
%             ax_f(count) = 0;
%             v_f(count) = vel;
%             dv_f(count) = 0;
%             vel = vmax;
%         end
%         if time_shifting > shift_time
%             shiftercount = shiftercount+1;
%             ts(shiftercount) = time_shifting;
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

%err = mean(ts/shift_time);
% disp(err)
%% combine results
VD = v_f-v_r;
forw = find(VD>=0);
back = find(VD<0);
velocity = zeros(1,length(VD));
t_elapsed = 0;
for i = 1:1:length(VD)
    if VD(i)<0
        velocity(i) = v_f(i);
        dtime(i) = dt_f(i);
    else
        velocity(i) = v_r(i);
        dtime(i) = dt_r(i);
    end
    t_elapsed = t_elapsed+dtime(i);
    time_elapsed(i) = t_elapsed;
end
Time = t_elapsed;
%toc