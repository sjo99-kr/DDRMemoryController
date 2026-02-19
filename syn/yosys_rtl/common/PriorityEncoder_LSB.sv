`timescale 1ns / 1ps

import MemoryController_Definitions::*;

module PriorityEncoder_LSB #(
    parameter int vector_length = 8,
    parameter int index_length = $clog2(vector_length)
)(
    input logic [vector_length-1:0] vector,
    output logic [index_length-1:0] index
    );

    always_comb begin           // LSB priority setup
        index = 0;
        for(int i = vector_length - 1; i >=0; i--) begin
            if(vector[i]) index = i[index_length-1:0]; 
        end        
    end
endmodule
