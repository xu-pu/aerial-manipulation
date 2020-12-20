%{
This script outputs a series of anchor points which a robot end-effector
(assumed attached to the cone's apex) must follow to create a straight line
rock and walk motion of the cone.
%}

load vector_field_pos.mat
load vector_field_neg.mat
load params.mat

figure('units','normalized','outerposition',[0 0 1 1])

%Integration Params
steps = 10;
step_length = 0.005;


%Transcribing rocking values to a variable.
filename = 'lr_rocking.mat';
total_rocking_steps = 10; %Keep it even number
right_rock = zeros(total_rocking_steps-1, 2);

left_rock = zeros(total_rocking_steps, 2);


rocking_angle = deg2rad(26);

start_pt = flowGivenPhi(rocking_angle, AX, h, 90);
new_start_pt = double([start_pt(1); start_pt(2)]);

anchor_point = [0;0];
% gait_rot_angle = deg2rad(14.0713);

% right_rock(1,:) = anchor_point';



[stream_x, stream_y, current_stream_pt] = flowFieldCCW(del_lb, start_pt,...
X_ann_pos+anchor_point(1), Y_ann_pos+anchor_point(2), U_ann_pos, V_ann_pos,...
X_ann_neg+anchor_point(1), Y_ann_neg+anchor_point(2), U_ann_neg, V_ann_neg,...
anchor_point,...
steps, step_length);


drawAnchorPoints(anchor_point, 'k*')
% prev_anchor_point = anchor_point;
% drawAnnulus(del_lb, del_ub, anchor_point, 0);
drawStreamLines(stream_x, stream_y, 'k')


% cla


anchor_point = anchorPoint(anchor_point, current_stream_pt, new_start_pt)
new_start_pt = current_stream_pt;
left_rock(1,:) = anchor_point';



[stream_x, stream_y, current_stream_pt] = flowFieldCW(del_lb, current_stream_pt',...
    X_ann_pos+anchor_point(1), Y_ann_pos+anchor_point(2), U_ann_pos, V_ann_pos,...
    X_ann_neg+anchor_point(1), Y_ann_neg+anchor_point(2), U_ann_neg, V_ann_neg,...
    anchor_point,...
    steps, step_length);



% drawAnnulus(del_lb, del_ub, anchor_point, 0.1);
drawAnchorPoints(anchor_point, 'k*')
% drawAnchorPointArrows(prev_anchor_point, anchor_point)
% prev_anchor_point = anchor_point;
drawStreamLines(stream_x, stream_y, 'k')

anchor_rot_dir = -1;

% cla


for rocking_step = 2:total_rocking_steps
    
        
    anchor_point = anchorPoint(anchor_point, current_stream_pt, new_start_pt)
    new_start_pt = current_stream_pt;
    right_rock(rocking_step-1,:) = anchor_point';
    anchor_rot_dir = -1*anchor_rot_dir;

    
    [stream_x, stream_y, current_stream_pt] = flowFieldCCW(del_lb, current_stream_pt',...
    X_ann_pos+anchor_point(1), Y_ann_pos+anchor_point(2), U_ann_pos, V_ann_pos,...
    X_ann_neg+anchor_point(1), Y_ann_neg+anchor_point(2), U_ann_neg, V_ann_neg,...
    anchor_point,...
    steps, step_length);





%     drawAnnulus(del_lb, del_ub, anchor_point, 0.1);
    drawAnchorPoints(anchor_point, 'k*')
%     drawAnchorPointArrows(prev_anchor_point, anchor_point)
%     prev_anchor_point = anchor_point;
    drawStreamLines(stream_x, stream_y, 'b')
    
    
%     cla
    

    anchor_point = anchorPoint(anchor_point, current_stream_pt, new_start_pt)
    new_start_pt = current_stream_pt;
    left_rock(rocking_step,:) = anchor_point';
    anchor_rot_dir = -1*anchor_rot_dir;

    
    [stream_x, stream_y, current_stream_pt] = flowFieldCW(del_lb, current_stream_pt',...
        X_ann_pos+anchor_point(1), Y_ann_pos+anchor_point(2), U_ann_pos, V_ann_pos,...
        X_ann_neg+anchor_point(1), Y_ann_neg+anchor_point(2), U_ann_neg, V_ann_neg,...
        anchor_point,...
        steps, step_length);

    
%     drawAnnulus(del_lb, del_ub, anchor_point, 0.1);
    drawAnchorPoints(anchor_point, 'k*')
%     drawAnchorPointArrows(prev_anchor_point, anchor_point)
%     prev_anchor_point = anchor_point;
    drawStreamLines(stream_x, stream_y, 'k')
    
%     clf('reset')

%     cla
    
end

save(filename, 'right_rock', 'left_rock')


clear all

function drawStreamLines(stream_x, stream_y, color)
%     grid on
    grid minor
    plot(stream_x, stream_y, ...
    'LineWidth', 2.0, ...
    'Color', color)
    axis ([-0.2 0.1 -.5 .7])
    hold on
    drawnow
    pause(0.50)

end

function drawAnchorPoints(anchor_point, color)
%     grid on
    grid minor
    plot(anchor_point(1), anchor_point(2), color, 'MarkerSize', 10)
%     axis ([-1 1 -1 1])
    hold on
    drawnow
%     pause(0.50)

end

function drawAnchorPointArrows(prev_anchor_point, curr_anchor_point)

    grid on 
    del = curr_anchor_point - prev_anchor_point;
    q = quiver(prev_anchor_point(1), prev_anchor_point(2), del(1), del(2));
    q.Color = [0.1, 0.1, 0.1];
    q.LineWidth = 1.1;
    q.MaxHeadSize = 0.30;    
    hold on

end


function drawAnnularVectorField(X_ann_pos, Y_ann_pos, U_ann_pos, V_ann_pos, ...
            X_ann_neg, Y_ann_neg, U_ann_neg, V_ann_neg, ...
            anchor_point)
    quiver(X_ann_pos+anchor_point(1), Y_ann_pos+anchor_point(2), U_ann_pos, V_ann_pos, 0.3, ...
        'LineWidth', 1.0, ... %previously 0.05
        'Color', [.2 .2 .2])
    hold on
    
    quiver(X_ann_neg+anchor_point(1), Y_ann_neg+anchor_point(2), U_ann_neg, V_ann_neg, 0.3, ...
        'LineWidth', 1.0, ... %previously 0.05
        'Color', [.8 .8 .8])
    hold on
    
end



function new_anchor_point = anchorPoint(anchor_point, current_stream_pt, new_start_pt)

    motion_dir =  current_stream_pt-new_start_pt;
    new_anchor_point = anchor_point + motion_dir
    
end


function new_anchor_point = anchorPoint2(anchor_point, current_stream_pt, rot_dir)

    rot_angle = rot_dir*atan(abs(anchor_point(1) -current_stream_pt(1))/ abs(anchor_point(2) -current_stream_pt(2)));
    
    rot_matrix = [cos(-rot_angle) -sin(-rot_angle); sin(-rot_angle) cos(-rot_angle)];
    
    vec_stream_anchor = anchor_point - current_stream_pt;
    
    new_anchor_curr_stream = rot_matrix*[vec_stream_anchor(1); vec_stream_anchor(2)];       

    new_anchor_point = current_stream_pt + new_anchor_curr_stream;
    
end


function [stream_x, stream_y, current_stream_pt] = flowFieldCW(del_lb, start_pt,...
        x_curve_set_pos, y_curve_set_pos, vector_field_u_pos, vector_field_v_pos,...
        x_curve_set_neg, y_curve_set_neg, vector_field_u_neg, vector_field_v_neg,...
        anchor_point,...
        steps, step_length)

    start_pt = double(start_pt);
    del_lb = double(del_lb);

    stream_y = zeros(1, ceil(steps/step_length));
    stream_x = zeros(1, ceil(steps/step_length));
    
    stream_x(1,1) = start_pt(1);
    stream_y(1,1) = start_pt(2);

    
    current_stream_pt = start_pt';


    interpolant_u = scatteredInterpolant(x_curve_set_neg(:), y_curve_set_neg(:), -vector_field_u_neg(:), 'natural', 'none');
    interpolant_v = scatteredInterpolant(x_curve_set_neg(:), y_curve_set_neg(:), -vector_field_v_neg(:), 'natural', 'none');
    
    current_direction(1,1) = interpolant_u(start_pt(1), start_pt(2));
    current_direction(2,1) = interpolant_v(start_pt(1), start_pt(2));

    
    epsilon = 0.005;
    dir = 1;
    
    for stp = 2:ceil(steps/step_length)

        
        
        next_stream_pt = current_stream_pt + step_length*current_direction;

        next_vector_u = interpolant_u(next_stream_pt(1,1), next_stream_pt(2,1));
        next_vector_v = interpolant_v(next_stream_pt(1,1), next_stream_pt(2,1));
        


        next_direction = [next_vector_u; next_vector_v];

        stream_x(1, stp) = next_stream_pt(1,1);
        stream_y(1, stp) = next_stream_pt(2,1);

        current_stream_pt = next_stream_pt;
        current_direction = next_direction;
        
        
        if norm(current_stream_pt-anchor_point) < del_lb + epsilon
            interpolant_u = scatteredInterpolant(x_curve_set_pos(:), y_curve_set_pos(:), -vector_field_u_pos(:), 'natural', 'none');
            interpolant_v = scatteredInterpolant(x_curve_set_pos(:), y_curve_set_pos(:), -vector_field_v_pos(:), 'natural', 'none');
            
            dir = -1;
        end
        
        if dir == -1 && norm(current_stream_pt-anchor_point) > norm(start_pt-anchor_point')
            break
        end
       
    end
    
    stream_x = stream_x(1, 1:stp);
    stream_y = stream_y(1, 1:stp);
        
end

function [stream_x, stream_y, current_stream_pt] = flowFieldCCW(del_lb, start_pt,...
        x_curve_set_pos, y_curve_set_pos, vector_field_u_pos, vector_field_v_pos,...
        x_curve_set_neg, y_curve_set_neg, vector_field_u_neg, vector_field_v_neg,...
        anchor_point,...
        steps, step_length)
    
    start_pt = double(start_pt);
    del_lb = double(del_lb);

    stream_y = zeros(1, ceil(steps/step_length));
    stream_x = zeros(1, ceil(steps/step_length));
    
    stream_x(1,1) = start_pt(1);
    stream_y(1,1) = start_pt(2);

    
    current_stream_pt = start_pt';


    interpolant_u = scatteredInterpolant(x_curve_set_pos(:), y_curve_set_pos(:), vector_field_u_pos(:), 'natural', 'none');
    interpolant_v = scatteredInterpolant(x_curve_set_pos(:), y_curve_set_pos(:), vector_field_v_pos(:), 'natural', 'none');
    
    current_direction(1,1) = interpolant_u(start_pt(1), start_pt(2));
    current_direction(2,1) = interpolant_v(start_pt(1), start_pt(2));

    
    epsilon = 0.005;
    dir = 1;
    
    for stp = 2:ceil(steps/step_length)

        
        
        next_stream_pt = current_stream_pt + step_length*current_direction;

        next_vector_u = interpolant_u(next_stream_pt(1,1), next_stream_pt(2,1));
        next_vector_v = interpolant_v(next_stream_pt(1,1), next_stream_pt(2,1));
        


        next_direction = [next_vector_u; next_vector_v];

        stream_x(1, stp) = next_stream_pt(1,1);
        stream_y(1, stp) = next_stream_pt(2,1);

        current_stream_pt = next_stream_pt;
        current_direction = next_direction;
        
        
        if norm(current_stream_pt-anchor_point) < del_lb + epsilon
            interpolant_u = scatteredInterpolant(x_curve_set_neg(:), y_curve_set_neg(:), vector_field_u_neg(:), 'natural', 'none');
            interpolant_v = scatteredInterpolant(x_curve_set_neg(:), y_curve_set_neg(:), vector_field_v_neg(:), 'natural', 'none');
            
            dir = -1;
        end
        
        if dir == -1 && norm(current_stream_pt-anchor_point) > norm(start_pt-anchor_point')
            break
        end
       
    end
        
    stream_x = stream_x(1, 1:stp);
    stream_y = stream_y(1, 1:stp);
        
end

function start_pt = flowGivenPhi(phi, AX, h, rot_angle)

    subs_phi_AX = subs(AX, phi); 

    norm_point = sqrt(norm(subs_phi_AX)^2 - h^2);
    
    start_pt_x = norm_point*cosd(rot_angle);
    start_pt_y = norm_point*sind(rot_angle);

    
    start_pt = [start_pt_x start_pt_y];
    
    
end

function drawAnnulus(del_lb,del_ub, anchor_point, color)

    th = deg2rad(60):pi/50:deg2rad(120);

    x_unit_inn = del_lb * cos(th) + anchor_point(1);
    y_unit_inn = del_lb * sin(th) + anchor_point(2);

    x_unit_out = del_ub * cos(th) + anchor_point(1);
    y_unit_out = del_ub * sin(th) + anchor_point(2);

    inn_circ = plot(x_unit_inn, y_unit_inn, '--', ...
        'LineWidth', 2.0, ...
        'Color', [.3+color .3+color .3+color]);
    
    hold on 
    
    out_circ = plot(x_unit_out, y_unit_out, '--', ...
        'LineWidth', 2.0, ...
        'Color', [.3+color .3+color .3+color]);
    
    hold on

    pbaspect([1 1 1])
    hold on

end

