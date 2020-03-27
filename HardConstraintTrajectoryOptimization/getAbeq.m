function [Aeq, beq] = getAbeq(n_seg, n_order, ts, start_cond, end_cond)
    n_coeff = n_order+1
    n_all_poly = n_seg*(n_coeff);
    %#####################################################
    % STEP 2.1 p,v,a constraint in start 
    Aeq_start = zeros(3, n_all_poly);
    beq_start = zeros(3, 1); 
    
    d1 = n_order;
    d2 = n_order * (n_order - 1);
    Aeq_start(1:1:3, 1:1:n_coeff) = [  1,    0,  0, 0, 0, 0, 0, 0;
                                       -d1,   d1,  0, 0, 0, 0, 0, 0;
                                       d2, -2*d2, d2, 0, 0, 0, 0, 0];
    beq_start = start_cond';% p,v,a

    %#####################################################
    % STEP 2.2 p,v,a constraint in end
    Aeq_end = zeros(3, n_all_poly);
    Aeq_end = zeros(3, 1); 
    Aeq_end(1:1:3, n_all_poly-n_order:1:n_all_poly) = [0, 0, 0, 0, 0,  0,     0,   1;
                                                       0, 0, 0, 0, 0,  0,   -d1,  d1;
                                                       0, 0, 0, 0, 0, d2, -2*d2,  d2];
    beq_end = end_cond';% p,v,a
    
    %#####################################################
    % Init array for continuity constrains
    Aeq_con_p = zeros(n_seg-1, n_all_poly);
    beq_con_p = zeros(n_seg-1, 1);
    Aeq_con_v = zeros(n_seg-1, n_all_poly);
    beq_con_v = zeros(n_seg-1, 1);
    Aeq_con_a = zeros(n_seg-1, n_all_poly);
    beq_con_a = zeros(n_seg-1, 1);
    
    for con_index = 1:n_seg-1
        index = n_coeff + n_coeff * (con_index - 1);
        
        % STEP 2.3 position continuity constrain between 2 segments
        Aeq_con_p(con_index,index) = 1;% the end of previous segment
        Aeq_con_p(con_index,index+1) = -1;% the begin of next segment
        
        % STEP 2.4 velocity continuity constrain between 2 segments
        Aeq_con_v(con_index,index-1:index) = [-d1, d1];% the end of previous segment
        Aeq_con_v(con_index,index+1:index+2) = -[-d1, d1];% the begin of next segment
        
        % STEP 2.5 acceleration continuity constrain between 2 segments
        Aeq_con_a(con_index,index-2:index) = [d2, -2*d2, d2];% the end of previous segment
        Aeq_con_a(con_index,index+1:index+3) = -[d2, -2*d2, d2];% the begin of next segment
    end

    %#####################################################
    % combine all components to form Aeq and beq
    Aeq_con = [Aeq_con_p; Aeq_con_v; Aeq_con_a];
    beq_con = [beq_con_p; beq_con_v; beq_con_a];
    Aeq = [Aeq_start; Aeq_end; Aeq_con];
    beq = [beq_start; beq_end; beq_con];
end