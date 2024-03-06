function [x, y, z, gamma] = read_curr_pos(port_num, PROTOCOL_VERSION,ADDR_PRO_PRESENT_POSITION)
    ID_BASE                     = 11;
    ID_1                        = 12;
    ID_2                        = 13;
    ID_3                        = 14;
    ID_CLAW                     = 15;

    pos_base = read4ByteTxRx(port_num, PROTOCOL_VERSION, ID_BASE, ADDR_PRO_PRESENT_POSITION);
    pos_1 = read4ByteTxRx(port_num, PROTOCOL_VERSION, ID_1, ADDR_PRO_PRESENT_POSITION);
    pos_2 = read4ByteTxRx(port_num, PROTOCOL_VERSION, ID_2, ADDR_PRO_PRESENT_POSITION);
    pos_3 = read4ByteTxRx(port_num, PROTOCOL_VERSION, ID_3, ADDR_PRO_PRESENT_POSITION);
    pos_claw = read4ByteTxRx(port_num, PROTOCOL_VERSION, ID_CLAW, ADDR_PRO_PRESENT_POSITION);
    
    curr_theta = [inverse_mapping_angle('tbase', pos_base), inverse_mapping_angle('t1', pos_1), inverse_mapping_angle('t2', pos_2), inverse_mapping_angle('t3', pos_3)];
    [~, ~, ~, T3, T4] = FK(curr_theta);
    [x, y, z, gamma] = end_effector_state(T3, T4);
end