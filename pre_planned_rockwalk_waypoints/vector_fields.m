%{
This scripts outputs the positive and negative vector fields together with
the parameters. The vector fields (over an annulus) correspond to the 
instaneous trajectory of the ground contact point of an oblique cone when
its apex is fixed and the cone is allowed to roll on a flat ground over its
base rim.
%}


%Set parameter values
R = sym(0.35); % Base radius of the cone
hc = sym(1.35*sind(90)); % Vertical height of the cone
r = sym(0.35 + 1.5*cosd(90)); % Oblique cone's eccentricity. Horizontal distance between 
h = sym(1.26); % Height of the cone's apex (assumed fixed) above ground 

%Compute bounds on delta
dist_AB = sqrt((r-R)^2 + hc^2);
dist_AC = sqrt((r+R)^2 + hc^2);

del_lb = sqrt(dist_AB^2 - h^2)
del_ub = sqrt(dist_AC^2 - h^2)

drawAnnulus(del_lb, del_ub);

syms t w1 w2 w3 %theta and three components of the axis

sym_rot_axang_mat = [...
    cos(t)+(1-cos(t))*w1^2, w1*w2*(1-cos(t))-w3*sin(t), w1*w3*(1-cos(t))+w2*sin(t);...
    w1*w2*(1-cos(t))+w3*sin(t), cos(t)+(1-cos(t))*w2^2, w2*w3*(1-cos(t))-w1*sin(t);...
    w1*w3*(1-cos(t))-w2*sin(t), w2*w3*(1-cos(t))+w1*sin(t), cos(t)+(1-cos(t))*w3^2];


syms phi
assume(phi>=0 & phi<=pi)


OX = [-R*cos(phi) -R*sin(phi) 0];
%tangent vector at X
X_tangent = [-R*sin(phi) R*cos(phi) 0]; 
%Compute Initial Location of O
AO = [r 0 -hc];
AX = AO + OX;



% Set parameters for radial and angular resolution of vector fields.
radial_res = 10;
angular_res = 80;

% Containers for annular vector field;
% pos field corresponds to ground contact approaching inner circle
% neg field corresponds to ground contact receding inner circle

X_ann_pos = zeros(radial_res, angular_res);
Y_ann_pos = zeros(radial_res, angular_res);
U_ann_pos = zeros(radial_res, angular_res);
V_ann_pos = zeros(radial_res, angular_res);

X_ann_neg = zeros(radial_res, angular_res);
Y_ann_neg = zeros(radial_res, angular_res);
U_ann_neg = zeros(radial_res, angular_res);
V_ann_neg = zeros(radial_res, angular_res);

radial_count = 1;

for del_query = linspace(del_lb, del_ub, radial_res)

    AP = [del_query 0 -h];
    unit_AP = AP/norm(AP);

    %Based on the distance constraint, solve for variable phi in AX
    sol_phi = solve(AX(1)^2 + AX(2)^2 + AX(3)^2 - norm(AP)^2 == 0,...
        phi, 'Real', true, 'PrincipalValue', true);

    subs_AX = subs(AX, sol_phi(1)); 
    subs_X_tangent = subs(X_tangent, sol_phi(1));

    %Compute Unit Vectors
    unit_X_tangent = subs_X_tangent/norm(subs_X_tangent);
    unit_AX = subs_AX/norm(subs_AX);

    first_rot_axis = cross(unit_AX, unit_AP)/ norm(cross(unit_AX, unit_AP));
    first_rot_angle = acos(dot(unit_AX, unit_AP));

    first_rot_mat = subs(sym_rot_axang_mat,...
        [w1 w2 w3 t], ...
        double([first_rot_axis first_rot_angle]));

   transformed_unit_tangent = first_rot_mat*unit_X_tangent';

   second_rot_mat_t = subs(sym_rot_axang_mat, ...
       [w1 w2 w3], double(unit_AP));

    sol_t = solve(...
        second_rot_mat_t(3,:)*double(transformed_unit_tangent) == 0, ...
        t >=0, t <=pi, ... %these limits are different for positive and negative fields
        t,...
        'Real', true, 'PrincipalValue', false, 'IgnoreAnalyticConstraints', false);

    second_rot_mat = subs(second_rot_mat_t, t, sol_t(1));

    rot_mat = second_rot_mat*first_rot_mat;

    n_base = rot_mat*[0 0 1]';   

    tan_vec = cross(n_base, [0 0 1]);
    tan_unit_vec = tan_vec/norm(tan_vec);
    
    [X_ann_pos, Y_ann_pos, U_ann_pos, V_ann_pos]= ...
    fillAnnulusVectorField(X_ann_pos, Y_ann_pos, U_ann_pos, V_ann_pos, ...
    del_query, tan_unit_vec, radial_count, angular_res);

    radial_count = radial_count + 1;

end

radial_count = 1; 

for del_query = linspace(del_lb, del_ub, radial_res)

    AP = [del_query 0 -h];
    unit_AP = AP/norm(AP);

    %Based on the distance constraint, solve for variable phi in AX
    sol_phi = solve(AX(1)^2 + AX(2)^2 + AX(3)^2 - norm(AP)^2 == 0,...
        phi, 'Real', true, 'PrincipalValue', true);

    subs_AX = subs(AX, sol_phi(1)); 
    subs_X_tangent = subs(X_tangent, sol_phi(1));

    %Compute Unit Vectors
    unit_X_tangent = subs_X_tangent/norm(subs_X_tangent);
    unit_AX = subs_AX/norm(subs_AX);

    first_rot_axis = cross(unit_AX, unit_AP)/ norm(cross(unit_AX, unit_AP));
    first_rot_angle = acos(dot(unit_AX, unit_AP));


    first_rot_mat = subs(sym_rot_axang_mat,...
        [w1 w2 w3 t], ...
        double([first_rot_axis first_rot_angle]));

   transformed_unit_tangent = first_rot_mat*unit_X_tangent';

   second_rot_mat_t = subs(sym_rot_axang_mat, ...
       [w1 w2 w3], double(unit_AP));

    sol_t = solve(...
        second_rot_mat_t(3,:)*double(transformed_unit_tangent) == 0, ...
        t >=pi, t <=2*pi, ...
        t,...
        'Real', true, 'PrincipalValue', false, 'IgnoreAnalyticConstraints', false);

    second_rot_mat = subs(second_rot_mat_t, t, sol_t(1));

    rot_mat = second_rot_mat*first_rot_mat;

    n_base = rot_mat*[0 0 1]';   

    tan_vec = cross(n_base, [0 0 1]);
    tan_unit_vec = tan_vec/norm(tan_vec);

    [X_ann_neg, Y_ann_neg, U_ann_neg, V_ann_neg]= ...
    fillAnnulusVectorField(X_ann_neg, Y_ann_neg, U_ann_neg, V_ann_neg, ...
    del_query, tan_unit_vec, radial_count, angular_res);

    radial_count = radial_count + 1;

end

start_pt = flowGivenPhi(pi-0.9, AX, h, deg2rad(0), del_lb); 

[stream_x, stream_y] = flowField(...
     start_pt, X_ann_pos, Y_ann_pos, U_ann_pos, V_ann_pos);
 
plot(stream_x, stream_y, ...
    'LineWidth', 1.40, ...
    'Color', [.1 .1 .1])
hold on

quiver(X_ann_pos, Y_ann_pos, U_ann_pos, V_ann_pos, 0.3, ...
    'LineWidth', 1.0, ... %previously 0.05
    'Color', [.4 .4 .4])
hold on


%Save vector fields data and parameters
filename = 'vector_field_pos.mat';
save(filename, 'X_ann_pos', 'Y_ann_pos', 'U_ann_pos', 'V_ann_pos')

filename = 'vector_field_neg.mat';
save(filename, 'X_ann_neg', 'Y_ann_neg', 'U_ann_neg', 'V_ann_neg')

filename = 'params.mat';
save(filename, 'AX', 'h', 'R', 'hc', 'r', 'del_lb', 'del_ub')


clear all



function start_pt = flowGivenPhi(phi, AX, h, rot_angle, del_lb)

    subs_phi_AX = subs(AX, phi); 

    start_pt_x = sqrt(norm(subs_phi_AX)^2 - h^2);
    
    start_pt_y = del_lb*sin(rot_angle);
    
    start_pt = [start_pt_x start_pt_y];
end


function drawAnnulus(del_lb,del_ub)

    th = 0:pi/50:2*pi;

    x_unit_inn = del_lb * cos(th);
    y_unit_inn = del_lb * sin(th);

    x_unit_out = del_ub * cos(th);
    y_unit_out = del_ub * sin(th);

    inn_circ = plot(x_unit_inn, y_unit_inn, ...
        'LineWidth', 2.5, ...
        'Color', 'b');
    
    hold on 
    
    out_circ = plot(x_unit_out, y_unit_out, ...
        'LineWidth', 2.5, ...
        'Color', 'g');
    
    hold on

    %Draw line

    line_lb_ub = linspace(del_lb, del_ub, 50);

    plot(line_lb_ub, zeros(1,50), ....
        'LineWidth', 2.5, ...
        'Color', 'm')

    pbaspect([1 1 1])
    hold on

end


function [X_ann, Y_ann, U_ann, V_ann]= ...
    fillAnnulusVectorField(X_ann, Y_ann, U_ann, V_ann, ...
    del_query, tan_unit_vec, radial_count, angular_res)

    %{
    Given a vector field along a line, this vector rotates it (about
    origin) to fill the entire annulus   
    %}
    
    angular_count = 1;
    for theta = linspace(0,2*pi,angular_res)
        
                
        R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
        
        rot_tan_unit_vec = R*[tan_unit_vec(1) tan_unit_vec(2)]';
        
        rot_query_pt = R*[del_query 0]';
        
        
        X_ann(radial_count, angular_count) = rot_query_pt(1);
        Y_ann(radial_count, angular_count) = rot_query_pt(2);
        U_ann(radial_count, angular_count) = rot_tan_unit_vec(1);
        V_ann(radial_count, angular_count) = rot_tan_unit_vec(2);


    angular_count = angular_count + 1;
        
    end

end


function [stream_x, stream_y] = flowField(start_pt, x_curve_set, y_curve_set, vector_field_u, vector_field_v)

    %{
    This function computes the stream line over the annular vector field.

    Input: start point, vector field over the annulus

    Params: steps and step length

    Returns: stream line in terms of its x and y coordinates
    %}

    start_pt = double(start_pt);
    steps = 10;
    step_length = 0.005;

    stream_y = zeros(1, ceil(steps/step_length));
    stream_x = zeros(1, ceil(steps/step_length));
    
    stream_x(1,1) = start_pt(1);
    stream_y(1,1) = start_pt(2);

    
    current_stream_pt = start_pt';


    interpolant_u = scatteredInterpolant(x_curve_set(:), y_curve_set(:), vector_field_u(:), 'natural', 'none');
    interpolant_v = scatteredInterpolant(x_curve_set(:), y_curve_set(:), vector_field_v(:), 'natural', 'none');
    
    current_direction(1,1) = interpolant_u(start_pt(1), start_pt(2));
    current_direction(2,1) = interpolant_v(start_pt(1), start_pt(2));

        
    for stp = 2:ceil(steps/step_length)       
        
        next_stream_pt = current_stream_pt + step_length*current_direction;

        next_vector_u = interpolant_u(next_stream_pt(1,1), next_stream_pt(2,1));
        next_vector_v = interpolant_v(next_stream_pt(1,1), next_stream_pt(2,1));
        


        next_direction = [next_vector_u; next_vector_v];

        stream_x(1, stp) = next_stream_pt(1,1);
        stream_y(1, stp) = next_stream_pt(2,1);

        current_stream_pt = next_stream_pt;
        current_direction = next_direction;
       
    end
    
    stream_x = stream_x(1, 1:stp);
    stream_y = stream_y(1, 1:stp);
        
end
