`timescale 1ns / 1ps

//------------------------------------------------------------------------------
//  MemoryChannel (DDR4 Channel-Level BFM)
//
//  ROLE:
//      Channel-level DDR4 Bus Functional Model that aggregates multiple
//      ranks and arbitrates shared DQ/DQS signaling.
//
//  RESPONSIBILITIES:
//      - Instantiate per-rank DDR4 memory models (MemoryRank).
//      - Broadcast CA/ADDR signals to all ranks in the channel.
//      - Resolve rank-level read/write bursts onto a single channel DQ bus.
//      - Generate and drive channel-level DQS during read bursts.
//      - Model tri-state behavior of DQ/DQS for realistic DDR operation.
//
//  MODELING SCOPE:
//      - Rank-level parallelism within a single memory channel.
//      - DQ/DQS contention resolution across ranks.
//      - No scheduling, reordering, or timing arbitration logic.
//        (Handled by Memory Controller, not the DRAM model.)
//
//      Architectural Overview:
//
//            Channel DDR4 IF CMD / DQ BUS
//                    |   ∧
//                    V   |
//                +--------------+
//                | MemoryChannel|   (This module)
//                +--------------+
//                  |   |   |     |
//              ----+   |   +---- +...  ----+
//              |       |       |           |       
//        RankFSM[0] RankFSM[1]   ...  RankFSM[N]
//
//  ASSUMPTIONS:
//      - At most one rank drives read/write data at any given cycle.
//      - All ranks observe identical CA/ADDR signals (arbitration is based on cs_n signal.).
//      - Write data is broadcast and selectively consumed by target rank.
//
//  NOTES:
//      - This is a pure simulation model (BFM).
//      - Electrical accuracy is abstracted; focus is protocol-level behavior.
//      - Correctness relies on the controller obeying DDR timing rules.
//
//
//      Author  : Seongwon Jo
//      Created : 2026.02
//------------------------------------------------------------------------------

module MemoryChannel#(
        parameter int CHANNELID      = 0,
        parameter int NUMRANK        = 4,
        parameter int IOWIDTH        = 8,
        parameter int DEVICEPERRANK  = 4,
        parameter int CWIDTH         = 10,
        parameter int RWIDTH         = 15,
        parameter int BGWIDTH        = 2,
        parameter int BKWIDTH        = 2,
        parameter int COMMAND_WIDTH  = 18,
        parameter int BURST_LENGTH   = 8,
        parameter int MEM_DATAWIDTH  = 64,
        parameter int tCWL           = 12,
        parameter int tCL            = 16,
        parameter int tRCD           = 16,
        parameter int tRFC           = 256,
        parameter int tRP            = 16
    )(
        input logic clk, rst_n, clk2x,
        DDR4Interface.Memory_CA ddr4_cmdaddr_if,
        DDR4Interface.Memory_DQ ddr4_dq_if
);

    logic [MEM_DATAWIDTH-1:0] rankDQValue;
    logic [MEM_DATAWIDTH-1:0] rankRdDQ [NUMRANK-1:0];
    logic [MEM_DATAWIDTH-1:0] rankWrDQ [NUMRANK-1:0];
    //-------------------------------------------------------------------------
    //  Rank-level burst valid signals
    //
    //  - rankDQRdValid : Indicates which rank is driving read data
    //  - rankDQWrValid : Indicates which rank is consuming write data
    //-------------------------------------------------------------------------
    logic [NUMRANK-1:0] rankDQRdValid;                      //  Read Burst Valid per Rank
    logic [NUMRANK-1:0] rankDQWrValid;                      //  Write Burst Valid per Rank

    //-------------------------------------------------------------------------
    //  Channel-level DQS generation
    //
    //  - DQS is generated at the channel level and broadcast to all ranks.
    //  - Only driven during read bursts.
    //-------------------------------------------------------------------------
    logic channelDQS_t, channelDQS_c;                       //  Broadcast DQS Singal for channel
    logic channelDQRdValid, channelDQWrValid;               //  Channel Read or Write Burst Valid

    //-------------------------------------------------------------------------
    //  Per-rank memory instantiation
    //
    //  - All ranks receive identical CA/ADDR signals.
    //  - Each rank independently decides whether to drive or sample DQ.
    //-------------------------------------------------------------------------
    genvar i;
    generate
        for(i = 0; i < NUMRANK; i++) begin : genRank
            MemoryRank #(
                .RANKID(i),
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
            ) MemoryRank_ins(
                .clk(clk), .rst_n(rst_n), .clk2x(clk2x),
                `ifndef VERILATOR
                .rankDQS_t(ddr4_dq_if.dqs_t),
                .rankDQS_c(ddr4_dq_if.dqs_c),
                `endif
                `ifdef VERILATOR
                .rankDQS_t(channelDQS_t),
                .rankDQS_c(channelDQS_c),                
                `endif
                .rankRdData(rankRdDQ[i]),
                .rankWrData(rankWrDQ[i]),
                .rankDataStrb(ddr4_dq_if.dm_n),

                .ddr4_cmdaddr_if(ddr4_cmdaddr_if),

                .rankDQRdValid(rankDQRdValid[i]),
                .rankDQWrValid(rankDQWrValid[i])
            );
        end : genRank
    endgenerate

    //-------------------------------------------------------------------------
    //  Channel-level burst detection
    //
    //  - Reduction OR used to detect active read/write bursts across ranks.
    //-------------------------------------------------------------------------
    assign channelDQRdValid = |(rankDQRdValid);
    assign channelDQWrValid = |(rankDQWrValid);

    //-------------------------------------------------------------------------
    //  Channel DQS toggling logic
    //
    //  - DQS toggles on every clk2x during active read bursts.
    //  - Tri-stated when no read is in progress.
    //-------------------------------------------------------------------------
    always_ff@(posedge clk2x or negedge rst_n) begin : ChannelDQS
        if(!rst_n) begin
            channelDQS_t <= '0;
            channelDQS_c <= '1;
        end else begin
            if(channelDQRdValid) begin
                channelDQS_t <= !channelDQS_t;
                channelDQS_c <=  channelDQS_t;
            end else begin
                channelDQS_t <= '0;
                channelDQS_c <= '1;
            end
        end
    end : ChannelDQS

    `ifndef VERILATOR
    assign ddr4_dq_if.dqs_t = (channelDQRdValid) ?  channelDQS_t : 'z;
    assign ddr4_dq_if.dqs_c = (channelDQRdValid) ?  channelDQS_c : 'z;
    `endif

    `ifdef VERILATOR
    assign ddr4_dq_if.dqs_t = (channelDQRdValid) ? channelDQS_t : '0;
    assign ddr4_dq_if.dqs_c = (channelDQRdValid) ? channelDQS_c : '0;
    `endif
    //-------------------------------------------------------------------------
    //  Read data arbitration (Rank → Channel)
    //
    //  - Selects the read data from the active rank.
    //  - Assumes at most one rank asserts rankDQRdValid.
    //-------------------------------------------------------------------------

    always_comb begin : ChannelData
        rankDQValue = '0;
        for (int j = 0; j < NUMRANK; j++) begin
            if (rankDQRdValid[j]) begin
                rankDQValue = rankRdDQ[j];   
            end
        end
    end
    
    //-------------------------------------------------------------------------
    //  Per-rank DQ wiring
    //
    //  - Write data is broadcast; only the valid rank samples it.
    //  - Read data is driven onto the channel DQ bus.
    //-------------------------------------------------------------------------


    `ifndef VERILATOR
    genvar j;
    generate
    for (j = 0; j < NUMRANK; j++) begin : genDQWrite
        assign rankWrDQ[j] = rankDQWrValid[j] ? ddr4_dq_if.pin_dq : 'z;
    end
    endgenerate
    `endif

    `ifdef VERILATOR
    genvar k;
    generate 
        for(k = 0; k < NUMRANK; k++) begin : genDQWrite
            assign rankWrDQ[k] = rankDQWrValid[k] ? ddr4_dq_if.pin_dq : 0; 
        end
    endgenerate
    `endif
    
    `ifndef VERILATOR
    assign ddr4_dq_if.pin_dq = channelDQRdValid ? rankDQValue : 'z;
    `endif

    `ifdef VERILATOR
    assign ddr4_dq_if.pin_dq = channelDQRdValid ? rankDQValue : 0;
    `endif
endmodule
