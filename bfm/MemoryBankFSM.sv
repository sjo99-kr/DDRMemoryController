`timescale 1ns / 1ps

//------------------------------------------------------------------------------
//  MemoryBankFSM (DDR4 DRAM Bank-Level BFM)
//
//  ROLE:
//      Bank-level DRAM behavioral model for DDR4.
//      Each instance models one (BG, Bank) pair inside a rank (i.e., total 16 BankFSM per Rank in this modeling).
//
//  RESPONSIBILITIES:
//      - Decode DDR4 command signals broadcast from Rank-level controller in Memory Controller-side
//        (ACT, READ, WRITE, PRECHARGE, REFRESH).
//      - Model row state transitions (Closed ↔ Opened).
//      - Enforce DRAM timing constraints using a cycle-based timing counter
//        (tRCD, tCL, tCWL, tRP, tRFC, etc.).
//      - Generate burst-level read/write behavior with clk2x granularity.
//      - Maintain simplified per-bank storage for functional correctness.
//
//  MODELING SCOPE (Bank FSM in DRAM):
//      - Cycle-accurate control timing (FSM + counters).
//      - Functional data correctness (burst-level read/write).
//      - DQS and Pin_dq (i.e., bidirectional port) are abstracted
//
//  NOTES:
//      - Commands are assumed to be correctly decoded and bank-selected
//        by higher-level Rank/Channel logic.
//      - Storage is burst-granular, not full DRAM array-accurate (i.e., only has small memory space).
//      - Intended for verification & architectural validation.
//
//      Author  : Seongwon Jo
//      Created : 2026.02
//------------------------------------------------------------------------------

module MemoryBankFSM#(
    parameter int BANKID        = 0,
    parameter int BANKGROUPID   = 0,
    parameter int IOWIDTH       = 8,
    parameter int DEVICEPERRANK = 4,
    parameter int CWIDTH        = 10,
    parameter int RWIDTH        = 15,
    parameter int BGWIDTH       = 2,
    parameter int BKWIDTH       = 2,
    parameter int COMMAND_WIDTH = 18,
    parameter int BURST_LENGTH  = 8,
    parameter int MEM_DATAWIDTH = 64,
    
    parameter int tCWL = 12,
    parameter int tCL  = 16,
    parameter int tRCD = 16,
    parameter int tRFC = 256,
    parameter int tRP  = 16
)(
    input logic clk, rst_n, clk2x,
    
    //       DDR COMMAND BROADCAST TO EVERY BANKFSM         //                                                     
    input logic bankCKE,                                 
    input logic bankCS_N,                                      
    input logic bankPAR,                                                
    input logic [COMMAND_WIDTH-1:0] bankPIN_A, 
    input logic bankACT_N,
    input logic [MEM_DATAWIDTH/BURST_LENGTH - 1 : 0] bankDM_N, 
    input logic [MEM_DATAWIDTH/BURST_LENGTH - 1 : 0] bankUDM_N, 
    input logic [MEM_DATAWIDTH/BURST_LENGTH - 1 : 0] bankLDM_N, 
    input logic bankODT,

    //          INOUT  TO/FROM  Memory RANK (For DQ)        //
    input  logic [MEM_DATAWIDTH-1:0] bankWrDQ,                  
    output logic [MEM_DATAWIDTH-1:0] bankRdDQ,
    `ifndef VERILATOR
    inout  wire bankDQS_t, bankDQS_c,                          
    `endif 
    `ifdef VERILATOR
    input wire bankDQS_t, bankDQS_c,
    `endif

     //          OUTPUT  TO  Memory RANK (For READ DATA)     //
    output logic ReadBurstValid,                                
    output logic WriteBurstValid                                
);


    //////////////////////////////////////////////////////////////////////
    //////////////////// OPCODE FOR DDR4 Memory //////////////////////////
    //////////////// CKE     ACT     RAS      CAS      WE     CS   ///////
    // ACTIVATE       1       0       X        X       X       0        //
    // PRECHARGE      1       1       0       1        0       0        //
    // READ           1       1       1       0        0       0        //
    // WRITE          1       1       1       0        1       0        //
    // REFRESH        1       1       0       0        0       0        //
    // NO-OPERATION   1       1       1       1        1       0        //
    //////////////////////////////////////////////////////////////////////

    typedef enum logic [2:0] {
        rowClosed,
        Read,
        Write,
        Activate,
        rowOpened,
        Precharge,
        Refresh
    } BANKFSMState_t;

    BANKFSMState_t curr, next;

    logic [MEM_DATAWIDTH-1:0] MEM [BURST_LENGTH-1:0];  //  logic [MEM_DATAWIDTH-1:0] Memory [RankMemory/(MEM_DATAWIDTH/8) - 1:0];
    logic [$clog2(BURST_LENGTH)-1:0] burstCnt;         //  BURST COUNT FOR READ/WRITE
    logic mode;                                        //  Read Mode : 0, Write Mode : 1
    logic burstFlag;                                   //  Only VALID WHEN BURST READ OR BURST WRITE

    logic setup;                                       //  Setup Signal for DRAMCounter
    logic [5:0] load;                                  //  Load Value for DRAMCounter
    logic timeup;                                      //  Timeup Signal for FSM
    logic autoPrecharge;                               //  AutoPrecharge Reg for FSM

    //-------------------------------------------------------------------------
    //  Read / Write Mode & Burst Control
    //-------------------------------------------------------------------------
    always_ff@(posedge clk or negedge rst_n) begin : ReadWriteSetup
        if(!rst_n) begin
            mode <= 0;          //  Read/Write Mode Setup             //
            burstFlag <= 0;     //  BurstFlag for Read/Write Access   //
        end else begin
            if(curr==rowOpened) begin : ReadWriteModeSetup
                if(next == Read) mode <= 0;
                else if(next == Write) mode <= 1;
            end : ReadWriteModeSetup

            if((curr == Read) || (curr == Write)) begin : ReadWriteValid
                if(timeup) burstFlag <= 1;
            end : ReadWriteValid 
            
            if((burstFlag == 1)) begin : ReadWriteInValid
                if(burstCnt == BURST_LENGTH-1) burstFlag <= 0;
            end : ReadWriteInValid
        end
    end : ReadWriteSetup

    //-------------------------------------------------------------------------
    //  Burst Counter (clk2x domain)
    //-------------------------------------------------------------------------
    always @(posedge clk2x or negedge rst_n) begin : burstCntEvent
        if(!rst_n) begin
            burstCnt <= 0;
        end else begin
                if(burstFlag) begin
                    if(burstCnt == BURST_LENGTH-1) begin
                        burstCnt <= 0;
                    end else begin
                        burstCnt <= burstCnt + 1;
                    end
                end else begin
                    burstCnt <= 0;
                end
            end
    end : burstCntEvent


    //-------------------------------------------------------------------------
    //  Memory Data Update (Write Burst Handling)
    //-------------------------------------------------------------------------
    always @(posedge clk2x or negedge rst_n) begin : MemoryModeling
    if (!rst_n) begin
        for (int b = 0; b < BURST_LENGTH; b++)
            MEM[b] <= b;
    end else if (burstFlag && mode == 1) begin
            for (int i = 0; i < MEM_DATAWIDTH/8; i++) begin
                if (bankDM_N[i])
                    MEM[burstCnt][i*8 +: 8] <= bankWrDQ[i*8 +: 8];
            end
        end
    end : MemoryModeling


    assign bankRdDQ         =  (mode == 0)  ? MEM[burstCnt] : '0;
    assign ReadBurstValid   =  (mode == 0) && burstFlag; 
    assign WriteBurstValid  =  (mode == 1) && burstFlag;

    //-------------------------------------------------------------------------
    //  DRAM Timing Counter Interface
    //-------------------------------------------------------------------------
    DRAMTimingCounter DRAMCounter(      // Calculating the DRAM timings (e.g., tCL, tCWL, tRCD..)
        .clk(clk), .rst(rst_n),
        .setup(setup), .load(load),
        .timeUp(timeup)
    );
    
    //-------------------------------------------------------------------------
    //  Bank-level DRAM FSM
    //-------------------------------------------------------------------------
    always_ff@(posedge clk or negedge rst_n) begin : BankFSMState
        if(!rst_n) begin
            curr <= rowClosed;
        end else begin
            curr <= next;
        end
    end : BankFSMState

    //-------------------------------------------------------------------------
    //  Auto-Precharge Tracking
    //-------------------------------------------------------------------------
    always_ff@(posedge clk or negedge rst_n) begin : SetupAutoPrechargeREG
        if(!rst_n) begin
            autoPrecharge <= 0;
        end else begin
            if(curr == rowOpened) begin
                if(checkAutoRead(bankACT_N, bankCKE, bankCS_N, bankPIN_A) 
                || checkAutoWrite(bankACT_N, bankCKE, bankCS_N, bankPIN_A)) begin
                    autoPrecharge <= 1;
                end
            end
            if((curr == Read) || (curr == Write)) begin
                if((autoPrecharge == 1) && timeup) begin
                    autoPrecharge <= 0;
                end
            end
        end
    end : SetupAutoPrechargeREG

    //-------------------------------------------------------------------------
    //  Next-State Logic with Timing Enforcement
    //-------------------------------------------------------------------------
    always_comb begin : BankFSMNextState
        //      Initialization       //
        next = rowClosed;
        setup = 0;
        load = 0;
        case(curr) 
            rowClosed: begin 
                if(checkActivate(bankACT_N, bankCKE, bankCS_N, bankPIN_A)) begin
                    next = Activate;
                    setup = 1;
                    load = tRCD-2;
                end else if(checkRefresh(bankACT_N, bankCKE, bankCS_N, bankPIN_A)) begin
                    next = Refresh;
                    setup = 1;
                    load = tRFC-2;
                end else begin
                    next = rowClosed;
                end
            end
            Activate: begin
                if(timeup) begin
                    next = rowOpened;
                end else begin
                    next = Activate;
                end
            end
            rowOpened: begin
                if(checkAutoRead(bankACT_N, bankCKE, bankCS_N, bankPIN_A)) begin
                    next = Read;
                    setup = 1;
                    load = tCL-2;
                end else if(checkRead(bankACT_N, bankCKE, bankCS_N, bankPIN_A)) begin
                    next = Read;
                    setup = 1;
                    load = tCL-2;
                end else if(checkAutoWrite(bankACT_N, bankCKE, bankCS_N, bankPIN_A)) begin
                    next = Write;
                    setup = 1;
                    load = tCWL-2;
                end else if(checkWrite(bankACT_N, bankCKE, bankCS_N, bankPIN_A)) begin
                    next = Write;
                    setup = 1;
                    load = tCWL-2;
                end else if(checkPrecharge(bankACT_N, bankCKE, bankCS_N, bankPIN_A)) begin
                    next = Precharge;
                    setup = 1;
                    load = tRP-2;
                end else begin
                    next = rowOpened;
                end
            end
            Read: begin
                if(timeup) begin
                    if(autoPrecharge) begin
                        next = Precharge;
                        load = tRP-1;
                        setup = 1;
                    end else begin
                        next = rowOpened;
                    end
                end else begin
                    next = Read;
                end
            end
            Write: begin
                if(timeup) begin
                    if(autoPrecharge) begin
                        next = Precharge;
                        load = tRP-1;
                        setup = 1;
                    end else begin
                        next = rowOpened;
                    end
                end else begin
                    next = Write;
                end
            end
            Precharge : begin
                if(timeup) begin
                    next = rowClosed;
                end else begin
                    next = Precharge;
                end
            end
            Refresh: begin
                if(timeup) begin
                    next = rowClosed;
                end else begin
                    next = Refresh;
                end
            end
            default : begin
            end
        endcase
    end : BankFSMNextState
    
    //-------------------------------------------------------------------------
    //  DDR4 Command Decode Helpers
    //-------------------------------------------------------------------------
    function automatic logic checkActivate(input logic act_n, input logic cke, input logic cs_n,
                                                input logic [COMMAND_WIDTH-1:0] pin_A);
        if(act_n == 0 && cs_n == 0) begin
            if(cke == 1) begin
                $display("DRAM | IDLE -> ACTIVATION | ROW: %d | BG: %d | B: %d", 
                  pin_A[RWIDTH-1:0], BANKGROUPID, BANKID);
                return 1;
            end else begin
                return 0;
            end
        end else begin
            return 0;
        end
    endfunction
    
    function automatic logic checkRefresh(input logic act_n, input logic cke, input logic cs_n,
                                                input logic [COMMAND_WIDTH-1:0] pin_A);
        if({cke, cs_n, act_n, pin_A[16], pin_A[15], pin_A[14]} == 6'b101000) begin
            $display("DRAM | IDLE -> REFRESH");
            return 1;
        end else return 0;
    endfunction
    
    
    function automatic logic checkPrecharge(input logic act_n, input logic cke, input logic cs_n,
                                                input logic [COMMAND_WIDTH-1:0] pin_A);
        if({cke, act_n, pin_A[15], cs_n, pin_A[16], pin_A[14], pin_A[10]} == 7'b1110000) begin
            $display("DRAM | PRECHARGE | BG: %d | B: %d", BANKGROUPID, BANKID);
            return 1;
        end else return 0;
    endfunction
    
    
    function automatic logic checkAutoRead(input logic act_n, input logic cke, input logic cs_n,
                                                input logic [COMMAND_WIDTH-1:0] pin_A);
        if({cke, act_n, pin_A[16], pin_A[14], pin_A[10], cs_n, pin_A[15]} == 7'b1111100) begin
            $display("DRAM | AutoREAD | COL: %d | BG: %d | B: %d", pin_A[CWIDTH-1:0], BANKGROUPID, BANKID);
            return 1;
        end else return 0;
    endfunction
    
    function automatic logic checkRead(input logic act_n, input logic cke, input logic cs_n,
                                                input logic [COMMAND_WIDTH-1:0] pin_A);
        if({cke, act_n, pin_A[16], pin_A[14], cs_n, pin_A[15], pin_A[10]} == 7'b1111000)begin
            $display("DRAM | Read | COL: %d | BG: %d | B: %d", pin_A[CWIDTH-1:0], BANKGROUPID, BANKID);
            return 1;
        end else return 0;
    endfunction
    
    function automatic logic checkAutoWrite(input logic act_n, input logic cke, input logic cs_n,
                                                input logic [COMMAND_WIDTH-1:0] pin_A);
        if({cke, act_n, pin_A[16], pin_A[10], cs_n, pin_A[15], pin_A[14]} == 7'b1111000) begin
                $display("DRAM | AutoWrite | COL: %d | BG: %d | B: %d", pin_A[CWIDTH-1:0], BANKGROUPID, BANKID);
                return 1;
        end else return 0;
    endfunction
    
    function automatic logic checkWrite(input logic act_n, input logic cke, input logic cs_n,
                                                input logic [COMMAND_WIDTH-1:0] pin_A);
    
        if({cke, act_n, pin_A[16], cs_n, pin_A[15], pin_A[14], pin_A[10]} == 7'b1110000) begin
            $display("DRAM | Write | COL: %d | BG: %d | B: %d", pin_A[CWIDTH-1:0], BANKGROUPID, BANKID);
            return 1;
        end else return 0;
    
    endfunction
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////

endmodule





