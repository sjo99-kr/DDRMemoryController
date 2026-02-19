`timescale 1ns / 1ps

//------------------------------------------------------------------------------
//      PHYReadMode
//
//      Role:
//          Physical READ-mode data handler inside the PHYController.
//          This module models the DDR4 READ data path at burst granularity.
//
//      Architecture Overview:
//
//          ChannelController
//                 ∧
//                 |
//            PHYController
//                 ∧
//                 |
//            +-------------+
//            | PHYReadMode |   <-- This module
//            +-------------+
//                 ∧
//                 |
//            DDR IF DQ BUS
//
//      Responsibilities:
//          - Capture READ data from DDR4 DQ bus using DQS timing.
//          - Buffer burst-length data inside PHY-local FIFO.
//          - Convert DRAM-side timing (clk2x, DQS) to controller clock domain.
//          - Generate READ data ACK when a full burst is received.
//          - Stream buffered data toward Read Buffer with proper LAST signaling.
//
//      Data Flow:
//          DRAM DQ/DQS
//              |
//              V
//          [ clk2x domain ]
//              |
//          PHY Read FIFO
//              |
//              V
//          [ clk domain ]
//              |
//              V
//          Read Buffer
//
//      What this module DOES:
//          - Burst-aware capture of DDR READ data.
//          - Clock-domain abstraction between DQ sampling and controller logic.
//          - FIFO-based decoupling between DRAM timing and backend consumption.
//          - Precise generation of burst completion ACK.
//
//      What this module DOES NOT do:
//          - No command scheduling or timing decision logic.
//          - No bank/rank arbitration.
//          - No DDR electrical modeling (timing-only abstraction).
//
//      Design Assumptions:
//          - inflag is asserted only when READ data is expected from DRAM.
//          - outflag is asserted only when backend is ready to consume data.
//          - BURST_LENGTH is fixed and power-of-two.
//          - FIFO depth is sufficient to absorb burst traffic.
//
//      Notes:
//          - readDataACK is asserted only once per READ burst.
//          - outDataLast strictly follows BURST_LENGTH boundaries.
//          - This module is PHY-local and channel-wide.
//
//      Author  : Seongwon Jo
//      Created : 2026.02
//------------------------------------------------------------------------------


module PHYReadMode #(
    parameter int PHY_CHANNEL = 0,
    parameter int MEM_DATAWIDTH = 64,
    parameter int PHYFIFODEPTH = 32,
    parameter int PHYFIFOMAXENTRY = 4,
    parameter int BURST_LENGTH = 8
)(
                                                             //             Read Mode (PHY)               //
    input logic clk, rst, clk2x,                          
                                                            //          INPUT FROM  DRAM-side             //
    input logic dqs_t, dqs_c,                               //  1. Diff. signals for DQ BUS               //
    input logic [MEM_DATAWIDTH-1:0] inData,                 //  2. Data from DQ-BUS WHEN Read Process     //

                                                            //          INPUT FROM PHYCONTROLLER          //
    input logic inflag,                                     //  1. Valid for Receiving Data from DQ-BUS   //
    input logic outflag,                                    //  2. Valid for Sending Data to Read Buffer  //

                                                            //          OUTPUT TO PHYCONTROLLER           //
    output logic readDataACK,                               //  1. Read Data ACK WHEN PHY RECEIVES DATA   //

                                                            //           OUTPUT TO PHYCONTROLLER          //
    output logic [MEM_DATAWIDTH-1:0] outData,                      //  1. Data to Read Buffer                    //
    output logic outDataValid,                              //  2. Valid signal for Read Buffer           //
    output logic outDataLast                                //  3. Last signal for Read Buffer            //
);

    logic [$clog2(PHYFIFODEPTH)-1:0] burst_cnt_dram;        //  Burst Counter for PHY <- DRAM         //
    logic [$clog2(PHYFIFODEPTH)-1:0] burst_cnt_host;        //  Burst Counter for PHY -> READ BUFFER  //
    logic [MEM_DATAWIDTH-1:0] readModeFIFO [PHYFIFODEPTH-1:0];     //  Burst Data FIFO   in PHY              //


    //----  Burst Counter Setup for DQ-BUS Data (Counter) ------//
    always_ff @(posedge clk2x or negedge rst) begin : FIFOPUSHCounter
        if(!rst) begin
            burst_cnt_dram <= 0;
        end else begin
            if(inflag) begin                            // Inflag is triggered by PHYController
                if(burst_cnt_dram == PHYFIFODEPTH-1)begin
                    burst_cnt_dram <= 0;
                end else begin
                    burst_cnt_dram <= burst_cnt_dram + 1;
                end
            end
        end
    end : FIFOPUSHCounter

    always@(posedge clk or negedge rst) begin : InflightReadInDataCounter
        if(!rst) begin
            readDataACK <= 0;
        end else begin
            if(inflag) begin
                    if(burst_cnt_dram[2:0] == (BURST_LENGTH -1))begin
                        readDataACK <= 1;
                        `ifdef DISPLAY
                            $display("[%0t] PHYReadMode | READ DATA RECEIVED FROM DRAM", $time);
                        `endif
                    end else begin
                        readDataACK <= 0;
                    end
            end else begin
                readDataACK <= 0;
            end
        end
    end : InflightReadInDataCounter

    /////////////////////////////////////////////////////////////
    //--------  Burst InData Setup for DQ-BUS Data  (PUSH) ------//
    always_ff @(posedge clk2x or negedge rst) begin : FIFOPush
       if(!rst) begin
            for(int i = 0; i < PHYFIFODEPTH; i++) begin
                readModeFIFO[i] <= '0;
            end
       end  else begin
            if(inflag) begin
                readModeFIFO[burst_cnt_dram] <= inData;     
                `ifdef DISPLAY
                    $display("[%0t] PHYReadMode | RECEIVED READ DATA : %h | Bead: %d", $time, inData, burst_cnt_dram[$clog2(BURST_LENGTH)-1:0]);
                `endif

            end
       end
    end : FIFOPush
    /////////////////////////////////////////////////////////////


    //----  Burst Counter Setup for Read Buffer (Counter) -----//
    always_ff@(posedge clk or negedge rst) begin : FIFOPOPCounter
        if(!rst) begin
            burst_cnt_host <= 0;
        end else begin
            if(outDataValid) begin
                if (burst_cnt_host == PHYFIFODEPTH-1) begin
                    burst_cnt_host <= 0;
                end else begin
                    burst_cnt_host <= burst_cnt_host + 1;
                end
            end 
        end
    end : FIFOPOPCounter
    /////////////////////////////////////////////////////////////

    assign outDataValid = outflag;
    assign outDataLast  = (burst_cnt_host[2:0] == BURST_LENGTH-1) ? 1 : 0;
    assign outData      = (outflag) ? readModeFIFO[burst_cnt_host] : 0; 
    /////////////////////////////////////////////////////////////


endmodule
