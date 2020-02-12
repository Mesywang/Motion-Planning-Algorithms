function Ct = getCt(n_seg, n_order)
    %#####################################################
    % STEP 2.1: finish the expression of Ct
    
    num_d = 8 * n_seg;% number of vector d
    num_df_and_dp = 4 * (n_seg + 1);% number of vector df+dp
    num_midwp = n_seg - 1;% number of middle waypoints
    num_df = 4 * 2 + num_midwp;% number of vector df
    num_dp = 3 * num_midwp;% number of vector dp
    
    Ct = zeros(num_d, num_df_and_dp);
    
    % Ct for first pos,vel,acc,jerk
    Ct_start = zeros(4, num_df_and_dp);
    Ct_start(:,1:4) = eye(4);
    Ct(1:4,:) = Ct_start;
    
    % Ct for last pos,vel,acc,jerk
    Ct_end = zeros(4, num_df_and_dp);
    Ct_end(:,(num_df-3):num_df) = eye(4);
    Ct((num_d-3):num_d,:) = Ct_end;
    
    % Ct for middle pos
    for mid_pos_index = 1:num_midwp
        % Ct for middle pos
        Ct(5+8*(mid_pos_index-1),4+mid_pos_index) = 1;
        Ct(4+(5+8*(mid_pos_index-1)),4+mid_pos_index) = 1;
        
        % Ct for middle vel
        Ct(6+8*(mid_pos_index-1),num_df+1+3*(mid_pos_index-1)) = 1;
        Ct(4+(6+8*(mid_pos_index-1)),num_df+1+3*(mid_pos_index-1)) = 1;
        
        % Ct for middle acc
        Ct(7+8*(mid_pos_index-1),num_df+2+3*(mid_pos_index-1)) = 1;
        Ct(4+(7+8*(mid_pos_index-1)),num_df+2+3*(mid_pos_index-1)) = 1;
        
        % Ct for middle jerk
        Ct(8+8*(mid_pos_index-1),num_df+3+3*(mid_pos_index-1)) = 1;
        Ct(4+(8+8*(mid_pos_index-1)),num_df+3+3*(mid_pos_index-1)) = 1;
    end
end
