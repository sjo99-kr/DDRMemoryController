`timescale 1ns / 1ps


module priority_ptr_WriteBuf#(
    parameter int vector_length = 8,
    parameter int index_length = $clog2(vector_length),
    parameter type vector_type = logic
    )(
    input vector_type  vector [vector_length-1:0],
    output logic [index_length-1:0] index
    );

    always_comb begin           // LSB priority setup
        index = 0;
        for(int i = vector_length - 1; i >=0; i--) begin
            if((vector[i].valid==1) && (vector[i].issued == 1)) index = i; 
        end        
    end
endmodule
