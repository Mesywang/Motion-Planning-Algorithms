function [Aieq, bieq] = getAbieq(n_seg, n_order, corridor_range, ts, v_max, a_max)
    n_all_poly = n_seg * (n_order + 1);
    

    %#####################################################
    % STEP 3.2.1 p constraint
    
    % start and end points and the start points of each segment are excluded,
    % they are guaranteed by Boundary Constraints and Continuity Constraints
    pos_constraints = n_seg * n_order - 1;
    
    Aieq_p = zeros(2*pos_constraints, n_all_poly);% row_num is the number of pos constraints, col_num is the number of segments
    bieq_p = zeros(2*pos_constraints, 1);
  
    for A_con_index = 1:n_seg-1
        raw_index = 1 + 7 * (A_con_index - 1);
        col_index = 2 + 8 * (A_con_index - 1);
        Aieq_p(raw_index:raw_index+6,col_index:col_index+6) = eye(7);% x < b 
        
%         Aieq_p_one_segment = [first_col, eye(order)];
    end
    Aieq_p(pos_constraints-5:pos_constraints,n_all_poly-6:n_all_poly-1) = eye(6);% for the last segment pos
    Aieq_p(pos_constraints+1:2*pos_constraints,:) = -Aieq_p(1:pos_constraints,:);% for the other part, % -x < -a 
    
    % x < b 
    for b_con_index = 1:n_seg
        first_coeff_index = 1 + 7 * (b_con_index - 1);
        bieq_p(first_coeff_index:first_coeff_index+5,1) = corridor_range(b_con_index,2);% middle control points
        if(b_con_index ~= n_seg)
            if(corridor_range(b_con_index+1,2) > corridor_range(b_con_index,2))
                bieq_p(first_coeff_index+6,1) = max(corridor_range(b_con_index,2),corridor_range(b_con_index+1,1));% continuity waypoints
            else
                bieq_p(first_coeff_index+6,1) = max(corridor_range(b_con_index,1),corridor_range(b_con_index+1,2));% continuity waypoints
            end
            
        end
    end
    % -x < -a 
    for b_con_index = 1:n_seg
        first_coeff_index = pos_constraints + 1 + 7 * (b_con_index - 1);
        bieq_p(first_coeff_index:first_coeff_index+5,1) = -corridor_range(b_con_index,1);% middle control points
        if(b_con_index ~= n_seg)
            if(corridor_range(b_con_index+1,2) > corridor_range(b_con_index,2))
                bieq_p(first_coeff_index+6,1) = -min(corridor_range(b_con_index,2),corridor_range(b_con_index+1,1));% continuity waypoints
            else
                bieq_p(first_coeff_index+6,1) = -min(corridor_range(b_con_index,1),corridor_range(b_con_index+1,2));% continuity waypoints
            end  
        end
    end
    

    %#####################################################
    % STEP 3.2.2 v constraint
    vel_constraints = n_seg * n_order;
    Aieq_v = zeros(2*vel_constraints, n_all_poly);
    bieq_v = zeros(2*vel_constraints, 1);
    d1 = n_order;
    d2 = n_order * (n_order - 1);
    for v_con_index = 1:n_seg
        first_coeff_raw_index = 1 + 7 * (v_con_index-1);
        first_coeff_col_index = 1 + 8 * (v_con_index-1);
        Aieq_v(first_coeff_raw_index:first_coeff_raw_index+6,first_coeff_col_index:first_coeff_col_index+7) ...
        = [-d1, d1,  0,   0,   0,   0,   0,  0;
            0, -d1, d1,   0,   0,   0,   0,  0; 
            0,  0, -d1,  d1,   0,   0,   0,  0; 
            0,  0,   0, -d1,  d1,   0,   0,  0; 
            0,  0,   0,   0, -d1,  d1,   0,  0; 
            0,  0,   0,   0,   0, -d1,  d1,  0; 
            0,  0,   0,   0,   0,   0, -d1, d1];
    end
    Aieq_v(vel_constraints+1:2*vel_constraints,:) = -Aieq_v(1:vel_constraints,:);%for the other part, % -x < Vmax 
    bieq_v(:) = repmat(v_max,2*vel_constraints,1);% -Vmax < x < Vmax

    %#####################################################
    % STEP 3.2.3 a constraint
    acc_constraints = n_seg * (n_order - 1);
    Aieq_a = zeros(2*acc_constraints, n_all_poly);
    bieq_a = zeros(2*acc_constraints, 1);
    
    for a_con_index = 1:n_seg
        first_coeff_raw_index = 1 + 6 * (a_con_index-1);
        first_coeff_col_index = 1 + 8 * (a_con_index-1);
        Aieq_a(first_coeff_raw_index:first_coeff_raw_index+5,first_coeff_col_index:first_coeff_col_index+7) ...
        = [ d2, -2*d2,    d2,     0,     0,     0,     0,  0;
             0,    d2, -2*d2,    d2,     0,     0,     0,  0; 
             0,     0,    d2, -2*d2,    d2,     0,     0,  0; 
             0,     0,     0,    d2, -2*d2,    d2,     0,  0; 
             0,     0,     0,     0,    d2, -2*d2,    d2,  0; 
             0,     0,     0,     0,     0,    d2, -2*d2,  d2];
    end
    Aieq_a(acc_constraints+1:2*acc_constraints,:) = -Aieq_a(1:acc_constraints,:);%for the other part, % -x < Amax 
    bieq_a(:) = repmat(a_max,2*acc_constraints,1);% -Amax < x < Amax
    
    %#####################################################
    % combine all components to form Aieq and bieq
    Aieq = [Aieq_p; Aieq_v; Aieq_a];
    bieq = [bieq_p; bieq_v; bieq_a];
%     Aieq = Aieq_p;
%     bieq = bieq_p;
end