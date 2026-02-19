`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////////////
//      APTimingCounter
//
//      Author : Seongwon Jo
//      Date   : 2026.02
//
//      1) Manages Auto-Precharge (AP) timing based on tRP and tWR constraints
//          in the Rank Scheduler.
//
//      2) While Auto-Precharge is in progress, the corresponding Rank FSM
//          must not issue CMD/ADDR to the target bank.
//
//      Auto-Precharge Management Policy:
//         - Reservation for Auto-Precharge:
//             * Rank FSM asserts apSetup for a specific bank (BG/BK)
//         - Start of Auto-Precharge Timing:
//             * Timing counter starts upon receiving AP ACK from PHY controller
//         - Timing Granularity:
//             * Managed per-bank (row-buffer–level precharge)
//
//
// Notes:
//   - This module does NOT issue DRAM commands.
//   - It only tracks AP-in-progress states and exposes per-bank blocking signals.
//
// Mode Selection:
//   - mode = 0 : READ COMMAND WITH AUTO PRECHARGE  (TIME : tRP)
//   - mode = 1 : WRITE COMMAND WITH AUTO PRECHARGE (TIME : tWR+tRP)
//////////////////////////////////////////////////////////////////////////////////////////
import MemoryController_Definitions::*;

module APTimingCounter( 
        input logic clk,
        input logic rst,
        input logic mode,
        input logic apSetup,
        input logic apACk,
        input logic [$clog2(16) - 1 : 0] BGBKfromFSM, 
        input logic [$clog2(16) - 1 : 0] BGBKfromBUF,
        output logic [16 - 1 : 0] bankState
    );

    localparam int COUNTER_WIDTH = $clog2(8+16+1);
    localparam int BK_W = $clog2(4);
    localparam int BG_W = $clog2(4);
    
    
    

    logic [16 - 1 :0] bankReqState;
//  bankReqState[i] == 1 : Auto-Precharge is pending or in progress for bank i
    
    logic [COUNTER_WIDTH-1:0] load;
    logic [COUNTER_WIDTH-1:0] [16 - 1 :0] bankCounter;
    logic [16 - 1 :0] countFlag;


    assign load =(mode) ? (tRP+tWR) - 1 : tRP-1;
 
    always_ff@(posedge clk or negedge rst) begin : BANKSTATEWINDOW
        if(!rst) begin
            for(int i = 0; i< 16; i++) begin
                bankReqState[i] <= 0;                    // Initialization for BankState
            end
        end else begin
            if(apSetup)begin                             // If APSETUP comes from FSM, BankState Setup WAITING Auto-Precharge
                bankReqState[BGBKfromFSM] <= 1;
            end
            for(int i =0; i < 16; i++) begin     // BankState Reset AFTER Auto-Precharge
                if(countFlag[i] == 1 && bankCounter[i] == 0 && (bankReqState[i] == 1))begin
                    bankReqState[i] <= 0;
                end
            end 
        end
    end : BANKSTATEWINDOW

    always_ff@(posedge clk or negedge rst) begin : PerBANKCounterSetup
        if(!rst) begin
            for(int i = 0; i < 16; i++)begin
                bankCounter[i] <= 0;                        // Initialization for BankCounter
            end
        end else begin
            if(apSetup)begin
                bankCounter[BGBKfromFSM] <= load;           // If APSETUP comes from FSM, then Set up the counter.
                `ifdef DISPLAY
                    $display("[%0t] APTimingCounter | AP setup: BG=%0d BK=%0d",
                            $time,
                            BGBKfromFSM[BK_W+BG_W-1 : BK_W],
                            BGBKfromFSM[BK_W-1 : 0]);
                `endif
            end    
            for(int i = 0; i < 16; i++)begin
                if(countFlag[i]) begin
                    bankCounter[i] <= bankCounter[i] - 1;   // If Flag is Set, Then Counting for tRP (Precharge Delay)
                end
            end       
        end
    end : PerBANKCounterSetup

    always_ff@(posedge clk or negedge rst )begin : PerBANKCounterFlagSetup
        if(!rst) begin
            for(int i = 0; i< 16; i ++)begin
                countFlag[i] <= 0;
            end
        end
        else begin
            if(bankReqState[BGBKfromBUF] == 1 && apACk)begin
                countFlag[BGBKfromBUF] <= 1;            //  If APACK comes from FSM, then countFlag is on for counting.
                `ifdef DISPLAY
                    $display("[%0t] APTimingCounter | AP TIMING COUNTING START : BG=%0d BK=%0d",
                            $time,
                            BGBKfromBUF[BK_W+BG_W-1 : BK_W],
                            BGBKfromBUF[BK_W-1 : 0]);
                `endif
            end
            for(int i = 0; i <  16; i++)begin
                if(bankCounter[i] == 0)begin
                    countFlag[i] <= 0;                  // When bankCounter[i] ==0, then countFlag off.
                    `ifdef DISPLAY
                    $display("[%0t] APTimingCounter | AP TIMING COUNTING END : BG=%0d bk=%0d", 
                            $time,
                            i / NUMBANK, i % NUMBANK);
                    `endif
                end
            end
        end
    end : PerBANKCounterFlagSetup

    assign bankState = bankReqState; 
endmodule
