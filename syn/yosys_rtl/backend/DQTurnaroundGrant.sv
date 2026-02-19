`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////////////
//      DQTurnaroundCalculator
//
//      Role:
//          Enforces read/write turnaround timing constraints on the DQ bus.
//
//      Functionality:
//          - Detects RD <-> WR mode transitions at channel level.
//          - Applies turnaround delays:
//              * tRTW  : Read -> Write
//              * tWTRS : Write -> Read (same rank)
//              * tWTRL : Write -> Read (different rank)
//          - Exposes DQTurnaroundFree to ChannelController.
//
//      Notes:
//          - CAS-to-CAS timing (tCCD) is handled separately.
//          - This module is channel-wide and independent of bank timing.
//
//      Author  : Seongwon Jo
//      Created : 2026.02
//////////////////////////////////////////////////////////////////////////////////////////
import MemoryController_Definitions::*;

module DQTurnaroundGrant 
(
    input logic clk, rst,
    input logic channelMode,
    input logic rankChanged,

    output logic DQTurnaroundFree
);

    localparam int DQTA_CNT_WIDTH = $clog2(
        (tRTW > tWTRL) ? ((tRTW > tWTRS) ? tRTW : tWTRS)
                    : ((tWTRL > tWTRS) ? tWTRL : tWTRS)
    );

    logic flag;
    logic [DQTA_CNT_WIDTH-1:0] cnt; // need to cover tRTW, tWTRS, tWTRL
    logic trackMode_currrent;
    logic trackMode_prev;
    logic diff_sig;

    always_ff@(posedge clk or negedge rst)begin
        if(!rst) begin
            trackMode_currrent <= 0;
            trackMode_prev <= 0;
        end else begin
            trackMode_prev <= trackMode_currrent;
            trackMode_currrent <= channelMode;
        end
    end

    assign diff_sig = (trackMode_prev != channelMode);

    always_ff@(posedge clk or negedge rst) begin
        if(!rst) begin
            flag <= 0;
        end else begin
            if(diff_sig) begin
                flag <= 1;

            end
            if(cnt == 0 && flag == 1)begin
                flag <= 0;
                `ifdef DISPLAY  
                    $display("[%0t] DQTurnaroundCalculator | TIMING CONSTRAINT OFF"); 
                `endif
            end
        end
    end

    always_ff@(posedge clk or negedge rst) begin
        if(!rst) begin
            cnt <= 0;
        end else begin
            if(diff_sig) begin
                if(channelMode) begin
                    cnt <= tRTW-1;
                    `ifdef DISPLAY  
                        $display("[%0t] DQTurnaroundCalculator | tRTW TIMING CONSTRAINT ON"); 
                    `endif
                end else begin
                    if(rankChanged) begin
                        cnt <= tWTRL-1; 
                        `ifdef DISPLAY  
                            $display("[%0t] DQTurnaroundCalculator | tWTRL TIMING CONSTRAINT ON"); 
                        `endif
                    end
                    else begin
                        cnt <= tWTRS-1;
                        `ifdef DISPLAY  
                            $display("[%0t] DQTurnaroundCalculator | tWTRS TIMING CONSTRAINT ON"); 
                        `endif
                    end
                end
            end
            if(flag) begin
                cnt <= cnt -1;
            end
        end
    end

    assign DQTurnaroundFree = !flag;
endmodule
