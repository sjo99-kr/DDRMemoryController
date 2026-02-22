`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////////////
//      CMDTurnaroundCalculator
//
//      Role:
//          Enforces rank-to-rank CMD bus turnaround timing (tRTR).
//
//      Functionality:
//          - Detects rank transitions on the CMD bus.
//          - Blocks CMD issuance for tRTRS cycles after a rank transition.
//          - Exposes CMDTurnaroundFree to indicate when CMD bus can be used.
//
//      Usage:
//          - Triggered by rankTransition from CMDGrantScheduler if the target arbitrated rank is changed.
//          - Used by RankControllers to gate CMD bus grants.
//
//      Notes:
//          - This module does not perform arbitration.
//          - It only tracks channel-level CMD bus timing constraints.
//
//      Author  : Seongwon Jo
//      Created : 2026.02
//////////////////////////////////////////////////////////////////////////////////////////

module CMDTurnaroundGrant #(
        parameter int tRTRS = 2
    )(
    input logic clk, rst,
    input logic rankTransition,
    output logic CMDTurnaroundFree
);

    logic flag;
    logic [$clog2(tRTRS)-1:0] cnt;

    //------------------------------------------------------------------------------
    //      Rank-to-Rank Turnaround Counter
    //
    //       - Loaded with (tRTRS - 1) on rank transition.
    //       - Decrements while flag is asserted.
    //------------------------------------------------------------------------------   
    always_ff@(posedge clk or negedge rst) begin :tRTRsCounterSetup
        if(!rst) begin
            cnt <= 0;
        end
        else begin
            if(flag) begin
                cnt <= cnt -1;
            end
            else if(rankTransition) begin
                cnt <= tRTRS -1;
                `ifdef DISPLAY
                    $display("[%0t] CMDTurnaroundCalculator | tRTR TIMING CONSTRAINTS FREE", $time);
                `endif
            end else begin
                cnt <= 0;
            end
        end
    end : tRTRsCounterSetup

    assign flag = ((cnt == 0) && rankTransition) ? 1 : (cnt != 0 ) ? 1: 0;
    // CMD bus is free only when no turnaround timing is in progress
    assign CMDTurnaroundFree = !flag;

endmodule
