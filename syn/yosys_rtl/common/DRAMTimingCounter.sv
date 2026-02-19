`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////////////
//      DRAMTimingCounter
//
//      Role:
//          Generic timing counter used for enforcing DRAM timing constraints (Especially in Rank FSM Load Timer)
//          (e.g., tRCD, tRP, tWR, tRFC, etc.).
//
//      Functionality:
//          - Loads a timing value when 'setup' is asserted.
//          - Decrements the counter every cycle.
//          - Asserts 'timeUp' for one cycle when the counter expires.
//
//      Usage Model:
//          - setup : asserted to (re)initialize the counter with 'load'.
//          - load  : timing value in cycles.
//          - timeUp: asserted when timing constraint has been satisfied.
//
//      Design Assumptions:
//          - Counter expiration and re-initialization must not occur
//            in the same cycle.
//          - Counter is inactive when countLoad == 0.
//
//      Author  : Seongwon Jo
//      Created : 2026.02
//////////////////////////////////////////////////////////////////////////////////////////
import MemoryController_Definitions::*;

module DRAMTimingCounter(
    input logic clk, 
    input logic rst,
    input logic setup,
    input logic[5:0] load,
    output logic timeUp
    );

    logic [5:0] countLoad;
    logic countEnd;
    
    always_ff@(posedge clk or negedge rst) begin
        if(!rst)begin
            countLoad <= 0;
            countEnd  <= 0;
        end else begin
            if(setup) begin
                countLoad <= load;
            end
            else begin
                if(countLoad != 0) begin
                    countLoad <= countLoad - 1;
                    if(countLoad == 1) begin
                        countEnd <= 1;
                    end
                end
                else begin
                    countLoad <= 0;
                    countEnd <= 0;
                end
            end
        end
        `ifdef ASSERTION
            assert (!(setup && countEnd))
                else $fatal("[0%0t] CountTimer | Illegal overlap: setup coincides with timing expiration", $time);
        `endif
    end

    assign timeUp = countEnd;
endmodule
