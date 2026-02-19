`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////////////////////
//
//      MemoryRank  (DDR4 Rank BFM)
//
//      Role:
//          Behavioral model of a single DDR4 rank for simulation and verification.
//          Aggregates multiple bank-level BFMs and emulates rank-level command
//          decoding and DQ behavior.
//
//      BFM Scope:
//          - This module is intended for **simulation only**.
//          - NOT synthesizable.
//          - Focused on protocol correctness and timing observability,
//                  not electrical or physical accuracy.
//
//      Architectural Overview:
//
//              DDR4 IF CMD / DQ BUS
//                    |   ∧
//                    V   |
//                +-----------+
//                | MemoryRank|   (This module)
//                +-----------+
//                  |   |   |
//          +--------+   |  +----  ...  ----+
//          |            |                  |
//      BankFSM[0]   BankFSM[1]    ...  BankFSM[N]
//
//      Responsibilities:
//          1) Decode rank-level CMD/ADDR signals (CS_n, BG, BK).
//          2) Select and activate the target MemoryBankFSM.
//          3) Fan-out shared control and DQ signals to all banks.
//          4) Aggregate per-bank read/write burst activity.
//          5) Expose rank-level DQ valid signals to the controller.
//
//      Modeling Assumptions:
//          - One command targets one bank at a time.
//          - At most one bank drives the DQ bus concurrently.
//          - Timing constraints are enforced at bank-level FSMs.
//
//      Design Notes:
//          - This module does NOT model:
//              * Analog signal integrity
//              * DQS training / calibration
//              * Power-down / self-refresh electrical behavior
//          - Intended for:
//              * Memory controller verification
//              * Timing FSM validation
//              * Read/write ordering and arbitration debugging
//
//      Author  : Seongwon Jo
//      Created : 2026.02
//////////////////////////////////////////////////////////////////////////////////////////////////

module MemoryRank#(
    parameter int RANKID = 0,
    parameter int IOWIDTH = 0,
    parameter int DEVICEPERRANK = 4,
    parameter int CWIDTH = 10,
    parameter int RWIDTH = 15,
    parameter int BGWIDTH = 2,
    parameter int BKWIDTH = 2,
    parameter int COMMAND_WIDTH = 18,
    parameter int BURST_LENGTH  = 8,
    parameter int MEM_DATAWIDTH = 64,

    parameter int tCWL = 12,
    parameter int tCL  = 16,
    parameter int tRCD = 16,
    parameter int tRFC = 256,
    parameter int tRP =  16
)(
    input logic clk, rst_n, clk2x,

    `ifndef VERILATOR
    inout wire rankDQS_t, rankDQS_c,
    `endif
    `ifdef VERILATOR
    input wire rankDQS_t, rankDQS_c,
    `endif
    output logic [MEM_DATAWIDTH-1:0] rankRdData,
    input logic [MEM_DATAWIDTH-1:0] rankWrData,


    input logic [MEM_DATAWIDTH/BURST_LENGTH-1:0] rankDataStrb,

    DDR4Interface.Memory_CA ddr4_cmdaddr_if,

    output logic rankDQRdValid, rankDQWrValid
);

    localparam int NUMBANKFSM = 1 << (BGWIDTH + BKWIDTH);

    logic [NUMBANKFSM-1:0] bankCMDGranted;
    logic [NUMBANKFSM-1:0] bankDQRdGranted, bankDQWrGranted;
    logic rankSelect;

    logic [MEM_DATAWIDTH-1:0] bankRdData [NUMBANKFSM-1:0];
    logic [MEM_DATAWIDTH-1:0] bankWrData [NUMBANKFSM-1:0];



    //------------------------------------------------------------------------------
    //      Bank Command Decode (BFM)
    //
    //      - Decodes BG/BK fields from CMD/ADDR interface.
    //      - Generates one-hot bank activation vector.
    //      - Used to trigger behavioral execution in a single bank FSM.
    //
    //  NOTE:
    //      - No electrical timing or command bus contention is modeled.
    //------------------------------------------------------------------------------
    always_comb begin
        bankCMDGranted = '0;
        if(ddr4_cmdaddr_if.cke) begin
            bankCMDGranted[{ddr4_cmdaddr_if.bg, ddr4_cmdaddr_if.b}] = 1;
        end
    end
    assign rankSelect = (ddr4_cmdaddr_if.cs_n[RANKID] == 0) ? 0 : 1;


    //------------------------------------------------------------------------------
    //      Bank-Level FSM Instantiation (BFM)
    //
    //      - Each MemoryBankFSM models one DRAM bank behaviorally.
    //      - FSMs receive identical rank-level signals,
    //          but only the selected bank reacts.
    //
    //  NOTE:
    //      - Parallel bank FSMs allow verification of bank-level overlap
    //          and command interleaving behavior.
    //------------------------------------------------------------------------------
    /* verilator lint_off PINCONNECTEMPTY */
    genvar i;
    generate
        for(i = 0; i < NUMBANKFSM; i++) begin : genMemoryBankFSM
            MemoryBankFSM #(
                .BANKID(i % 4),
                .BANKGROUPID(i / 4),
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
            ) BankFSM(
                .clk(clk), .rst_n(rst_n), .clk2x(clk2x),
                
                .bankCKE(bankCMDGranted[i]),
                .bankCS_N(rankSelect),
                .bankPAR(ddr4_cmdaddr_if.par),
                .bankPIN_A(ddr4_cmdaddr_if.pin_A),
                .bankACT_N(ddr4_cmdaddr_if.act_n),

                .bankDM_N(rankDataStrb),
                .bankUDM_N(),
                .bankLDM_N(),
                .bankODT(),

                .bankRdDQ(bankRdData[i]),
                .bankWrDQ(bankWrData[i]),
                .bankDQS_t(rankDQS_t),
                .bankDQS_c(rankDQS_c),

                .ReadBurstValid(bankDQRdGranted[i]),
                .WriteBurstValid(bankDQWrGranted[i])
            );
        end : genMemoryBankFSM
    endgenerate

    //------------------------------------------------------------------------------
    //      Rank-Level DQ Activity Aggregation (BFM)
    //
    //      - OR-reduces read/write burst valid signals from all banks.
    //      - Indicates active DQ ownership at rank level.
    //
    //  BFM Assumption:
    //      - Multiple banks asserting DQ valid simultaneously is illegal.
    //------------------------------------------------------------------------------

    always_comb begin
        for (int p = 0; p < NUMBANKFSM; p++) begin
            bankWrData[p] = 0;
        end

        for(int q = 0; q < NUMBANKFSM; q++) begin
            if(bankDQRdGranted[q]) begin
                rankRdData = bankRdData[q];
            end
            if(bankDQWrGranted[q]) begin
                bankWrData[q] = rankWrData;
            end
        end
    
    end
    assign rankDQRdValid = |bankDQRdGranted;
    assign rankDQWrValid = |bankDQWrGranted;



endmodule



