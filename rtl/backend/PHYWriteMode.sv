`timescale 1ns / 1ps

//------------------------------------------------------------------------------
//      PHYWriteMode
//
//      Role:
//          Physical WRITE-mode data handler inside the PHYController.
//          This module models the DDR4 WRITE data path at burst granularity.
//
//      Position in Architecture:
//
//          ChannelController
//                 |
//                 V
//            PHYController
//                 |
//                 V
//           +--------------+
//           | PHYWriteMode |
//           +--------------+
//                 |
//                 V
//              DDR4 DQ/DQS
//
//      Responsibilities:
//          - Accept WRITE data from Write Buffer via PHYController.
//          - Buffer burst-length data inside PHY-local FIFO.
//          - Generate DQS toggling synchronized with clk2x.
//          - Drive DQ/DQS/DM signals toward DRAM.
//          - Generate burst-level ACK to PHYController.
//
//      Data Flow:
//           Write Buffer
//               |
//               V
//         [ clk domain ]
//               |
//         PHY Write FIFO
//               |
//               V
//         [ clk2x domain (Abstract PHY) ]
//               |
//               V
//          DDR4 DQ / DQS
//
//      What this module DOES:
//          - Burst-aware WRITE data buffering.
//          - DQS generation and alignment using clk2x.
//          - Controlled DQ/DM driving toward DRAM.
//          - Precise WRITE burst completion detection.
//
//      What this module DOES NOT do:
//          - No command scheduling or arbitration.
//          - No timing decision logic (tCWL handled in PHYController).
//          - No DDR electrical accuracy modeling.
//
//      Design Assumptions:
//          - inflag is asserted only when WRITE data is valid from buffer.
//          - outflag is asserted only when PHYController allows DRAM driving.
//          - BURST_LENGTH is fixed and power-of-two.
//          - Only one WRITE burst is active at a time per channel.
//
//      Notes:
//          - DQS toggles on every clk2x edge during WRITE burst.
//          - outACK is asserted exactly once per WRITE burst.
//          - DM is inverted to match DDR4 active-low convention.
//          - FIFO is reset when inflag is deasserted.
//
//      Author  : Seongwon Jo
//      Created : 2026.02
//------------------------------------------------------------------------------

module PHYWriteMode #(
    parameter int PHY_CHANNEL   = 0,
    parameter int MEM_DATAWIDTH = 64,
    parameter int PHYFIFODEPTH  = 32,
    parameter int BURST_LENGTH  = 8
)(
                                                            //                  Write Mode (PHY)              //
    input logic clk, rst,                          
                                                            //          OUTPUT TO  DRAM-SIDE                  //
    output logic dqs_t, dqs_c,                              //  1. Diff. signals for DQ BUS                   //
    output logic [MEM_DATAWIDTH - 1 : 0] outdata,           //  2. Data to DQ BUS WHEN Write Processing       //
    output logic [MEM_DATAWIDTH/BURST_LENGTH-1:0] outDM,    //  3. Data Masking Bit for 64-bit data           //


                                                            //          INPUT FROM PHYCONTROLLER              //
    input logic clk2x,                                      //  1. clock 2x for generating diff. signals      //
    input logic inflag,                                     //  2. Valid for Receiving Data from WRITE BUFFER //
    input logic [MEM_DATAWIDTH-1:0] inData,                 //  3. Data Receiving from WRITE BUFFER           //
    input logic [MEM_DATAWIDTH/BURST_LENGTH-1:0] inStrb,    //  4. Data Masking bit from WRITE BUFFER         //
    input logic outflag,                                    //  5. Valid for Sending Data to DRAM-SIDE        //

                                                            //          OUTPUT TO PHYCONTROLLER               //
    output logic outACK                                     //  1. ACK to PHY Controller for Sending Data     //
);



    logic [MEM_DATAWIDTH-1:0] writeModeFIFO [PHYFIFODEPTH-1:0];     //  BURST DATA FIFO IN PHY 
    logic writeModeDMFIFO [PHYFIFODEPTH-1:0];                       //  Data Masking of BURST DATA FIFO IN PHY         
    logic [$clog2(PHYFIFODEPTH)-1:0] burst_cnt_dram;                //  Burst Count for PHY -> DRAM 
    logic [$clog2(PHYFIFODEPTH)-1:0] burst_cnt_host;                //  Burst Count for PHY <- WRITE BUFFER
    

    //---  Burst Counter & Diff. Sig Setup for DQ-BUS  (COUNTER) --//
    always @(posedge clk2x or negedge rst) begin : FIFOPOPCntAndDiff
        if(!rst) begin
            dqs_t             <= 1'b0;
            dqs_c             <= 1'b1;
            burst_cnt_dram    <= 0;
        end else begin
            if(outflag) begin
                `ifdef DISPLAY
                    $display("[%0t] PHYWriteMode | SERVING WRITE DATA : %h | Read: %d", $time, outdata, burst_cnt_dram[$clog2(BURST_LENGTH)-1:0]);
                `endif
                dqs_t <= ~dqs_t;
                dqs_c <= dqs_t;
                if(burst_cnt_dram == PHYFIFODEPTH-1)begin
                    burst_cnt_dram <= 0;
                end else begin
                    burst_cnt_dram <= burst_cnt_dram + 1;
                end
            end else begin
                dqs_t           <= '0;
                dqs_c           <= '1;
                burst_cnt_dram  <= '0;
            end
        end
    end : FIFOPOPCntAndDiff

    /////////////////////////////////////////////////////////////


    assign outACK = (burst_cnt_dram[2:0] == BURST_LENGTH-1) ? 1: 0;
    /////////////////////////////////////////////////////////////

    //-----------  Burst Data FIFO POP Process  (POP) ---------//
    assign outdata = (outflag) ? writeModeFIFO[burst_cnt_dram] : 'z; 
    `ifdef VERILATOR
        assign outDM = outflag ? ~writeModeDMFIFO[burst_cnt_dram] : '0;
    `else
        assign outDM = outflag ? ~writeModeDMFIFO[burst_cnt_dram] : 'z;
    `endif
    /////////////////////////////////////////////////////////////

    //-----------  Burst Data FIFO PUSH Process  (PUSH) ---------//
    always_ff@(posedge clk or negedge rst) begin : FIFOPUSH
        if(!rst) begin
            burst_cnt_host           <= 0;
            for(int i =0; i< PHYFIFODEPTH; i++) begin
                writeModeFIFO[i]     <= 0;
                writeModeDMFIFO[i]   <= 0;
            end
        end else begin
            if(inflag) begin
                writeModeFIFO[burst_cnt_host]     <= inData;
                writeModeDMFIFO[burst_cnt_host]   <= inStrb;
                if(burst_cnt_host == PHYFIFODEPTH-1) begin
                    burst_cnt_host                <= 0;
                end else begin
                    burst_cnt_host                <= burst_cnt_host + 1;
                end
            end else begin
                burst_cnt_host                    <= 0;
            end
        end
    end : FIFOPUSH
    /////////////////////////////////////////////////////////////


endmodule
