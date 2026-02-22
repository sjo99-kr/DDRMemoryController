`timescale 1ns / 1ps

//------------------------------------------------------------------------------
//  MemoryBFM (Top-Level DDR4 Memory Bus Functional Model)
//
//  ROLE:
//      Top-level DDR4 memory BFM that instantiates multiple independent
//      memory channels.
//
//  RESPONSIBILITIES:
//      - Act as the top abstraction of DRAM in the simulation environment.
//      - Contain per-channel DDR4 memory models (MemoryChannel).
//      - Provide a clean integration point for multi-channel ddr memory systems.
//      - Bridge external DDR4 interfaces to internal channel-level BFMs.
//
//  MODELING SCOPE:
//      - Channel-level structural composition only.
//      - No timing, scheduling, or protocol logic is implemented here (Timing, Scheduling are considered in Memory Controller-side).
//      - All DRAM protocol behavior is encapsulated inside MemoryChannel
//        and lower-level Bank/Rank BFMs.
//
//          Ch0 DDR4 IF CMD / DQ BUS   Ch1 DDR4 IF CMD / DQ BUS
//                    |   ∧                    |    ∧   
//                    V   |                    V    |
//           +-------------------------------------------+
//           |          MemoryBFM   (This module)        |
//           +-------------------------------------------+
//                  |                           |
//                  |                           |
//                  |                           |       
//              MemoryChannel_0          MemoryChannel_1
//
//  ASSUMPTIONS:
//      - Each channel operates independently (no cross-channel timing).
//      - DDR4Interface encapsulates CA/DQ signal groups per channel.
//      - The number of channels instantiated here reflects the system
//        configuration under test.
//
//  NOTES:
//      - This module is intended purely for verification and architectural
//        evaluation.
//      - Not synthesizable. (but it passes lint)
//      - Scaling to more channels only requires adding MemoryChannel instances.
//
//      Author  : Seongwon Jo
//      Created : 2026.02
//------------------------------------------------------------------------------

module MemoryBFM#(
    parameter int NUMCHANNEL    = 2,
    parameter int NUMRANK       = 4,
    parameter int IOWIDTH       = 8,
    parameter int DEVICEPERRANK = 4,
    parameter int CWIDTH        = 10,
    parameter int RWIDTH        = 15,
    parameter int BGWIDTH       = 2,
    parameter int BKWIDTH       = 2,
    parameter int COMMAND_WIDTH = 18,
    parameter int BURST_LENGTH  = 8,
    parameter int MEM_DATAWIDTH = 64,
    parameter int tCWL          = 12,
    parameter int tCL           = 16,
    parameter int tRCD          = 16,
    parameter int tRP           = 16,
    parameter int tRFC          = 256
)(
    input logic clk, rst_n, clk2x
    `ifndef VERILATOR
    ,DDR4Interface DDR4_CH0_IF,
    DDR4Interface DDR4_CH1_IF
    `endif
);
    `ifdef VERILATOR
        import MemoryController_Definitions::*;
    `endif


    `ifdef VERILATOR
    DDR4Interface #(
        .COMMAND_WIDTH(COMMAND_WIDTH),
        .MEM_DATAWIDTH(MEM_DATAWIDTH),
        .BURST_LENGTH(BURST_LENGTH),
        .BGWIDTH(BGWIDTH),
        .BKWIDTH(BKWIDTH),
        .RWIDTH(RWIDTH),
        .RKWIDTH(RKWIDTH),
        .CWIDTH(CWIDTH),
        .NUMRANK(NUMRANK)
    ) DDR4_CH0_IF(
        .clk(clk), .rst(rst_n)
    );
    DDR4Interface #(
        .COMMAND_WIDTH(COMMAND_WIDTH),
        .MEM_DATAWIDTH(MEM_DATAWIDTH),
        .BURST_LENGTH(BURST_LENGTH),
        .BGWIDTH(BGWIDTH),
        .BKWIDTH(BKWIDTH),
        .RWIDTH(RWIDTH),
        .RKWIDTH(RKWIDTH),
        .CWIDTH(CWIDTH),
        .NUMRANK(NUMRANK)
    ) DDR4_CH1_IF(
        .clk(clk), .rst(rst_n)
    );
    `endif

    //-------------------------------------------------------------------------
    //  Channel 0 Memory BFM
    //
    //  - Models one independent DDR4 memory channel.
    //  - Internally instantiates rank- and bank-level BFMs.
    //  - Handles command decoding, timing enforcement, and data bursts.
    //-------------------------------------------------------------------------
    MemoryChannel #(
        .CHANNELID(0),
        .NUMRANK(NUMRANK),
        .IOWIDTH(IOWIDTH),
        .DEVICEPERRANK(DEVICEPERRANK),
        .CWIDTH(CWIDTH),
        .RWIDTH(RWIDTH),
        .BGWIDTH(BGWIDTH),
        .BKWIDTH(BKWIDTH),
        .COMMAND_WIDTH(COMMAND_WIDTH),
        .BURST_LENGTH(BURST_LENGTH),
        .MEM_DATAWIDTH(MEM_DATAWIDTH),
        .tCWL(tCWL),
        .tCL(tCL),
        .tRCD(tRCD),
        .tRFC(tRFC),
        .tRP(tRP)
    ) MemoryChannel_Ch0(
        .clk(clk), .rst_n(rst_n), .clk2x(clk2x),
        .ddr4_cmdaddr_if(DDR4_CH0_IF.Memory_CA),
        .ddr4_dq_if(DDR4_CH0_IF.Memory_DQ)
    );

    //-------------------------------------------------------------------------
    //  Channel 1 Memory BFM
    //
    //  - Identical to Channel 0, but fully independent.
    //  - Enables multi-channel memory behavior in simulation.
    //-------------------------------------------------------------------------
    MemoryChannel #(
        .CHANNELID(1),
        .NUMRANK(NUMRANK),
        .IOWIDTH(IOWIDTH),
        .DEVICEPERRANK(DEVICEPERRANK),
        .CWIDTH(CWIDTH),
        .RWIDTH(RWIDTH),
        .BGWIDTH(BGWIDTH),
        .BKWIDTH(BKWIDTH),
        .COMMAND_WIDTH(COMMAND_WIDTH),
        .BURST_LENGTH(BURST_LENGTH),
        .MEM_DATAWIDTH(MEM_DATAWIDTH),
        .tCWL(tCWL),
        .tCL(tCL),
        .tRCD(tRCD),
        .tRFC(tRFC),
        .tRP(tRP)
    ) MemoryChannel_Ch1(
        .clk(clk), .rst_n(rst_n), .clk2x(clk2x),
        .ddr4_cmdaddr_if(DDR4_CH1_IF.Memory_CA),
        .ddr4_dq_if(DDR4_CH1_IF.Memory_DQ)
    );

endmodule
