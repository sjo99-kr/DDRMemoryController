`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////////////
//      DQRdWrGrantArbiter
//
//      Role:
//          Enforces CAS-to-CAS (tCCD) timing constraints on the DQ bus.
//
//      Functionality:
//          - Starts a timing window upon RD/WR CAS acknowledgment (chRdWrACK).
//          - Blocks further CAS issuance for:
//              * tCCDS cycles (same bank-group access)
//              * tCCDL cycles (different bank-group access)
//          - Exposes DQ availability signal to ChannelController.
//
//      Notes:
//          - This module handles ONLY tCCD constraints.
//          - Read/Write turnaround (tRTW / tWTR) is handled separately.
//
//      Author  : Seongwon Jo
//      Created : 2026.02
//////////////////////////////////////////////////////////////////////////////////////////

module DQRdWrCCDGrant #(
    parameter int tCCDS = 4,
    parameter int tCCDL = 6
)(   // tCCD consideration -> CMDDQArbiter
    input logic clk, rst,

    input logic chRdWrACK,
    input logic CCDType, // 1  -> tCCDS / 0 -> tCCDL
    output logic chRdWrAvailabe
    );

    localparam int CCD_CNT_WIDTH = $clog2(
        (tCCDS > tCCDL) ? tCCDS : tCCDL
    );

    logic [CCD_CNT_WIDTH-1:0] count;
    logic cnt_flag;
    logic DQAvailable;

    //------------ CAS-to-CAS (Short/Long) Timing Scheduling --------------//
    always_ff@(posedge clk or negedge rst) begin
        if(!rst) begin
            cnt_flag <= 0;
            DQAvailable <= 1;
            count <= 0;
        end else begin
            if(chRdWrACK) begin
                cnt_flag <= 1;
                DQAvailable <= 0;
                                                //  Subtract 2 cycles:
                if(CCDType) count <= tCCDS-2;   //  - 1 cycle for the issuing CAS itself
                else count <= tCCDL-2;          //  - 1 cycle for current cycle alignment
                `ifdef DISPLAY
                    $display("[%0t] DQRdWrCCDGrant | CCD Timing Constraint ON", $time);
                `endif
            end else begin 
                if(cnt_flag ==1)begin
                    count <= count -1;
                    if(count == 0) begin
                        count <= 0;
                        cnt_flag <= 0;
                        DQAvailable <= 1;
                        `ifdef DISPLAY
                        $display("[%0t] DQRdWrCCDGrant | CCD Timing Constraint OFF", $time);
                        `endif
                    end
                end
            end
        end
    end
    
    assign chRdWrAvailabe = DQAvailable;
    //-------------------------------------------------------------------//

endmodule
