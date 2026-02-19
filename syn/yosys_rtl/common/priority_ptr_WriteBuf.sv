`timescale 1ns / 1ps

import MemoryController_Definitions::*;

module priority_ptr_WriteBuf (
    input logic [54-1:0][128-1:0] vector,
    output logic [8-1:0] index
    );

    always_comb begin           // LSB priority setup
        index = 0;
        for(int i = 8 - 1; i >=0; i--) begin
            if((vector[i].valid==1) && (vector[i].issued == 1)) index = i; 
        end        
    end
endmodule
