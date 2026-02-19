`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////////////
//      ChannelController
//
//      Role:
//         Channel-level arbitration and scheduling logic for a single DDR channel.
//         This module connects MC Request from MC frontend to per-rank controllers,
//          while enforcing channel-wide timing and resource constraints.
//
//       Responsibilities:
//         1) Dispatch frontend memory requests to the corresponding RankController
//            based on rank address.
//         2) Perform channel-wide arbitration for:
//              - CMD bus (ACT / RD / WR / PRE / REF)
//              - DQ bus (read/write data timing and turnaround)
//         3) Enforce DDR timing constraints at channel scope:
//              - CMD bus : tRTR (rank-to-rank switching)
//              - DQ bus  : tCCD_S/L, tRTW, tWTR_S/L
//         4) Coordinate Auto-Precharge events between PHY, RankController,
//            and Memory Buffer.
//
//       Design Notes:
//         - One ChannelController exists per memory channel.
//         - Each channel contains multiple Rank-Controllers.
//         - Rank-level row/bank timing is handled inside RankController;
//           this module focuses on channel-level conflicts and arbitration.
//
//      Author  : Seongwon Jo
//      Created : 2026.02
//////////////////////////////////////////////////////////////////////////////////////////

module ChannelController #(
    parameter int DEVICE_CHANNEL = 0,

    parameter int NUM_BANKFSM  = 16,
    parameter int NUM_BANKFSM_BIT = 4,

    parameter int MEM_IDWIDTH = 4,
    parameter int MEM_USERWIDTH = 1,
    parameter int NUMRANK = 4, parameter int NUMBANK = 4, parameter int NUMBANKGROUP = 4,
    parameter int BGWIDTH = 2, parameter int BKWIDTH = 2,
    parameter int RWIDTH = 2, parameter int CWIDTH = 10, parameter int COMMAND_WIDTH = 18,
    parameter int READCMDQUEUEDEPTH = 8,
    parameter int WRITECMDQUEUEDEPTH = 8,
    parameter int OPENPAGELISTDEPTH = 16,
    parameter int AGINGWIDTH = 10,
    parameter int THRESHOLD = 512,
    parameter int tRP = 16, parameter int tWR = 18, parameter int tRFC = 256,
    parameter int tRTRS = 2, parameter int tCCDL = 6, parameter int tCCDS = 4,
    parameter int tRTW = 8, parameter int tWTRS = 3, parameter int tWTRL = 9,
    parameter int tREFI = 8192, parameter int tRCD = 16,
    parameter type FSMRequest = logic,    
    parameter type MemoryAddress = logic
)(
    // common
    input logic clk, rst,

                                                            /////////////////////////////////////////
                                                            //        Input from MC FrontEnd       //
    input MemoryAddress RankReqMemAddr,                     //     1. Memory Request address       // 
    input logic [MEM_IDWIDTH-1:0] RankReqId,                //     2. Memory Request ID            //
    input logic [MEM_USERWIDTH-1:0] RankReqUser,            //     3. Memory Request User          //
    input logic RankReqType,                                //     4. Memory Request Type          //
    input logic RankReqValid,                               //     5. Memory Request Valid         //
                                                     
                                                            /////////////////////////////////////////
                                                            //       Output to MC FrontEnd         //
    output logic [NUMRANK-1:0] RankReadReqReady,            //   1. Per-RankController ReadReady   //
    output logic [NUMRANK-1:0] RankWriteReqReady,           //   2. Per-RankController WriteReady  //
                                                     
                                                            /////////////////////////////////////////
    // Buffer-side                                          //        Input from MEM Buffer        //
    input logic rdBufAvailable,                             //  1. Read Buffer Available           //
    input logic [NUMRANK-1:0] rdBufRankWindowAvailable,
    input logic wrBufAvailable,                             //  2. Write Buffer Available          //

                                                            /////////////////////////////////////////
                                                            //         INPUT FROM PHYCONTROLLER    //
    input logic PHYReadModePreACK,                          //  1. Read Buffer Auto-Precharge ACK  //
    input logic PHYWriteModePreACK,                         //  2. Write Buffer Auto-Precharge ACK //
    input MemoryAddress PHYReadModePreAddr,                 //  3. Auto-Precharge Read Req ADDR    //
    input MemoryAddress PHYWriteModePreAddr,                //  4. Auto-Precharge Write Req ADDR   //

                                                            /////////////////////////////////////////
                                                            //          INPUT FROM MC BACKEND      //
    input logic channelMode,                                //  1. Channel Mode Selection          //
    output logic [$clog2(READCMDQUEUEDEPTH * NUMRANK):0] NumRdReq,
    output logic [$clog2(WRITECMDQUEUEDEPTH * NUMRANK):0] NumWrReq,
    output logic channelIdle,              
    input wire ChannelRDWRTransReady,                 

                                                            //////////////////////////////////////////
                                                            //        Output to MEM Buffer          //
    output logic [MEM_IDWIDTH-1:0] bufReadReqId,            // 1. RD Req ID for RD MEM Data RECV    //
    output logic [MEM_IDWIDTH-1:0] bufWriteReqId,           // 2. WR Req ID for WR MEM Data SEND    //
    output logic [MEM_USERWIDTH-1:0] bufReadReqUser,        // 3. RD Req User for RD MEM Data RECV  //
    output logic [MEM_USERWIDTH-1:0] bufWriteReqUser,       // 4. WR Req User for WR MEM Data SEND  //
    output logic bufReadReqACK,                             // 5. RD Req ACK for RD MEM RECV Ready  //
    output logic bufWriteReqACK,                            // 6. WR Req ACK for WR MEM SEND Ready  //
    output MemoryAddress bufReadReqACKAddr,                 // 7. Req ACK Addr for Ready to process //
    output MemoryAddress bufWriteReqACKAddr,                // 8. Req ACK Addr for Ready to process //


    output wire issuable,
                                                            //////////////////////////////////////////
                                                            //        OUTPUT TO PHY Controller      //
    output logic phyReadCMDIssuedACK,                       // 1. Read  CMD  Issued  ACK            //
    output MemoryAddress phyReadCMDIssuedAddr,              // 2. Read  CMD  Issued  ACK Address    //
    output logic phyWriteCMDIssuedACK,                      // 3. Write CMD  Issued  ACK            //
    output MemoryAddress phyWriteCMDIssuedAddr,             // 4. Write CMD  Issued  ACK Address    //

    // PHY-side                                             //////////////////////////////////////////
                                                            //         Output to PHY-SIDE           //
    DDR4Interface chDDR4_CMD_ADDR_IF                        //  1. DDR4 Command/Address Interface   //
    );
    

    typedef struct packed {                                 // MCFrontEnd-related Singals
        MemoryAddress addr;
        logic [MEM_IDWIDTH-1:0] id;
        logic [MEM_USERWIDTH-1:0] user;
        logic reqType;
        logic reqValid;
        logic reqReadReqReady;                              // 1. Per-RankController RD Ready signal : Ready when RD Requset Queue in RankController is not Full.
        logic reqWriteReqReady;                             // 2. Per-RankController WR Ready signal : Ready when WR Request Queue in RankController is not Full.
    } rankReq;

    typedef struct packed {                                 // channel Scheduler-related Signals
        logic chSchedCMDGranted;                            // 1. CMD bus granted : Only one RankController is granted for CMD Bus
        logic chSchedRdReady;                               // 2. RankController Ready signal : When there is any requests in RankController, it is ready (1).
        logic chSchedWrReady;
        logic chSchedACK;                                   // 3. RankController Req ACK signal : When RankFSM sends Read/Write signal to DRAM, it is ACK (1).
        logic chSchedGrantACK;                              // 4. CMD bus grant signal to only one RankController : When Channel Controller grants CMD bus to specific RankController, it is valid (1).
        logic chSchedFSMWait;                               // 5. RankController waits for Row timing constraints : When RankFSM is waiting for row timing constraints, it is valid (1).
        logic ccdType;                                      // 6. tCCD timing types for DQ bus timing constraints : It is 1 for tCCDS, and 0 for tCCDL.
    } chSched;

    typedef struct packed {                                 // MEMBuffer-related Signals
        logic bufReadPreACK;                                
        logic bufWritePreACK;
        logic [BKWIDTH+BGWIDTH-1:0] bufBankPre;             // 1. Bank, Bankgroup address for Auto-Precharge

        logic [MEM_IDWIDTH-1:0] bufReadReqId;
        logic [MEM_IDWIDTH-1:0] bufWriteReqId;
        logic [MEM_USERWIDTH-1:0] bufReadReqUser;
        logic [MEM_USERWIDTH-1:0] bufWriteReqUser;
        logic bufReadReqACK;
        logic bufWriteReqACK;

        MemoryAddress bufReqACKAddr;
    } memBuffer;
    

    //               Signals definition for RankControllers                         //
    rankReq rankReqVector[NUMRANK-1:0];
    chSched chSchedVector[NUMRANK-1:0];
    memBuffer memBufferVector[NUMRANK-1:0];
    
    
    logic [$clog2(READCMDQUEUEDEPTH)-1:0]  RDReqCnt [NUMRANK-1:0];
    logic [$clog2(WRITECMDQUEUEDEPTH)-1:0] WRReqCnt [NUMRANK-1:0];

    logic [$clog2(READCMDQUEUEDEPTH * NUMRANK):0]  NumRdReqCnt;
    logic [$clog2(WRITECMDQUEUEDEPTH * NUMRANK):0] NumWrReqCnt;



    always_comb begin
        NumRdReqCnt = 0;
        NumWrReqCnt = 0;
        for(int i = 0; i< NUMRANK; i++) begin
            NumRdReqCnt = RDReqCnt[i] + NumRdReqCnt;
            NumWrReqCnt = WRReqCnt[i] + NumWrReqCnt;
        end
    end

    assign NumRdReq =  NumRdReqCnt;
    assign NumWrReq =  NumWrReqCnt;

    //                   Signal definitions for Channel Scheduling                  //
    logic chRdWrAvailabe;                   // DQ bus available : accounting for tCCD      
    logic DQTurnaroundFree;                 // DQ bus available : accounting for tRTW/tWTR
    logic DQFree;                           // DQ bus Free signal : accounting for both tCCD & tRTW/tWTR
    logic rankTransition;                   // Rank transition signal : ACK for rank transition
    logic CMDTurnaroundFree;                // CMD bus available : accouting for tRT

    //                      Per-rank  CMD/ADDR INTERFACE SIGNALS                    //
    logic cke [NUMRANK-1:0];
    logic cs_n [NUMRANK-1:0];
    logic par [NUMRANK-1:0];
    logic act_n [NUMRANK-1:0];
    logic [COMMAND_WIDTH-1:0] pin_A [NUMRANK-1:0];
    logic [BGWIDTH-1:0] bg [NUMRANK-1:0];
    logic [BKWIDTH-1:0] b [NUMRANK-1:0];

    //------------------------------------------------------------------------------
    //          Auto-Precharge ACK Routing (from PHY Controller)
    //
    //          - PHY reports Auto-Precharge completion per address
    //          - ACK is forwarded only to the corresponding RankController
    //          - BankGroup/Bank information is preserved for per-bank AP tracking
    //------------------------------------------------------------------------------

    always_comb begin : AutoPreChargeACKFromPHYController
        for(int i = 0; i < NUMRANK; i++) begin
            memBufferVector[i].bufReadPreACK =0;
            memBufferVector[i].bufWritePreACK = 0;
            if(PHYReadModePreACK) begin
                if(PHYReadModePreAddr.rank == i) begin
                    memBufferVector[i].bufReadPreACK  = PHYReadModePreACK;
                    memBufferVector[i].bufWritePreACK = 0;
                    memBufferVector[i].bufBankPre = {PHYReadModePreAddr.bankgroup, PHYReadModePreAddr.bank};
                end
            end
            if(PHYWriteModePreACK) begin
                if(PHYWriteModePreAddr.rank == i) begin
                    memBufferVector[i].bufWritePreACK = PHYWriteModePreACK;
                    memBufferVector[i].bufReadPreACK  = 0;
                    memBufferVector[i].bufBankPre = {PHYWriteModePreAddr.bankgroup, PHYWriteModePreAddr.bank};
                end
            end
        end
    end : AutoPreChargeACKFromPHYController

    //------------------------------------------------------------------------------
    //      Memory Buffer and PHY Command Acknowledgment
    //
    //      - RankController issues RD/WR request acknowledgments
    //      - ChannelController multiplexes them to:
    //          * Memory Buffer (for data movement)
    //          * PHY Controller (for issued command tracking)
    //------------------------------------------------------------------------------
    always_comb begin : MEMBufferAllocationACK
        bufReadReqId       = 0; 
        bufWriteReqId      = 0;
        bufReadReqUser     = 0;
        bufWriteReqUser    = 0;
        bufReadReqACK      = 0;
        bufWriteReqACK     = 0;
        bufReadReqACKAddr  = 0;
        bufWriteReqACKAddr = 0;

        phyReadCMDIssuedACK   = '0;
        phyReadCMDIssuedAddr  = '0;
        phyWriteCMDIssuedACK  = '0;
        phyWriteCMDIssuedAddr = '0;

        for(int i =0; i<NUMRANK; i++)begin
            if(memBufferVector[i].bufReadReqACK) begin
                bufReadReqId = memBufferVector[i].bufReadReqId;
                bufReadReqUser = memBufferVector[i].bufReadReqUser;
                bufReadReqACK = memBufferVector[i].bufReadReqACK;
                bufReadReqACKAddr = memBufferVector[i].bufReqACKAddr;
                
                phyReadCMDIssuedACK  = bufReadReqACK;
                phyReadCMDIssuedAddr = bufReadReqACKAddr;
            end
            if(memBufferVector[i].bufWriteReqACK) begin
                bufWriteReqId = memBufferVector[i].bufWriteReqId;
                bufWriteReqUser = memBufferVector[i].bufWriteReqUser;
                bufWriteReqACK = memBufferVector[i].bufWriteReqACK;
                bufWriteReqACKAddr = memBufferVector[i].bufReqACKAddr;

                phyWriteCMDIssuedACK  = bufWriteReqACK;
                phyWriteCMDIssuedAddr = bufWriteReqACKAddr;
            end
        end
    end : MEMBufferAllocationACK
    
    //------------------------------------------------------------------------------
    //      Frontend Request Demultiplexing
    //
    //      - Routes a single frontend request to the target RankController
    //      - Read/Write readiness is reported back per rank
    //------------------------------------------------------------------------------
    always_comb begin : RequestDemultiplexing
        for(int i = 0; i<NUMRANK; i++) begin
            rankReqVector[i].addr = 0;
            rankReqVector[i].id = 0;
            rankReqVector[i].user = 0;
            rankReqVector[i].reqType = 0;
            rankReqVector[i].reqValid = 0;

            RankReadReqReady[i] = rankReqVector[i].reqReadReqReady;
            RankWriteReqReady[i] = rankReqVector[i].reqWriteReqReady;
            
            if(RankReqMemAddr.rank == i) begin
                rankReqVector[i].addr = RankReqMemAddr;
                rankReqVector[i].id = RankReqId;
                rankReqVector[i].user = RankReqUser;
                rankReqVector[i].reqType = RankReqType;
                rankReqVector[i].reqValid = RankReqValid;
            end
            
        end
    end : RequestDemultiplexing
    


    //////////////////////////////////////////////////////////////////////////////////////
    //      Signal Definitions for Channel Timing Scheduling
    //
    //      There are two types of channel-level scheduling: CMD Bus and DQ Bus.
    //
    //      1) CMD Bus Scheduling:
    //          - Enforces rank-to-rank turnaround timing (tRTR).
    //          - Ensures that only one RankController is granted the CMD bus at a time.
    //
    //      2) DQ Bus Scheduling:
    //          - Enforces CAS-to-CAS data timing constraints (tCCD_S / tCCD_L).
    //          - Enforces read/write turnaround timing constraints
    //              (tRTW, tWTR_S / tWTR_L).
    //////////////////////////////////////////////////////////////////////////////////////



    //------------------------------------------------------------------------------
    //      CMD Bus Arbitration (CMDGrantScheduler)
    //
    //      Policy:
    //          - Queue-depth-based priority
    //          - Random tie-break when depths are equal
    //
    //      Constraints:
    //          - Only one RankController granted per cycle
    //          - tRTR enforced via CMDTurnaroundCalculator
    //   NOTE: Current implementation assumes NUMRANK = 4.
    //------------------------------------------------------------------------------
    CMDGrantScheduler #(
        .NUMRANK(NUMRANK),
        .READCMDQUEUEDEPTH(READCMDQUEUEDEPTH),
        .WRITECMDQUEUEDEPTH(WRITECMDQUEUEDEPTH)
    ) CMDGrantScheduler(                    // CMD Bus granted scheduling 
        .clk(clk), .rst(rst), 
        .readyRdVector({chSchedVector[3].chSchedRdReady, chSchedVector[2].chSchedRdReady, chSchedVector[1].chSchedRdReady, chSchedVector[0].chSchedRdReady}), 
        .readyWrVector({chSchedVector[3].chSchedWrReady, chSchedVector[2].chSchedWrReady, chSchedVector[1].chSchedWrReady, chSchedVector[0].chSchedWrReady}),
        .fsmWaitVector({chSchedVector[3].chSchedFSMWait, chSchedVector[2].chSchedFSMWait, chSchedVector[1].chSchedFSMWait, chSchedVector[0].chSchedFSMWait}), 
        .readReqCnt(RDReqCnt),
        .writeReqCnt(WRReqCnt),
        .grantACK(|{chSchedVector[3].chSchedGrantACK, chSchedVector[2].chSchedGrantACK, chSchedVector[1].chSchedGrantACK, chSchedVector[0].chSchedGrantACK}),
        .writeMode(channelMode),
        .CMDRankTurnaround(CMDTurnaroundFree),
        .CMDGrantVector({chSchedVector[3].chSchedCMDGranted, chSchedVector[2].chSchedCMDGranted, chSchedVector[1].chSchedCMDGranted, chSchedVector[0].chSchedCMDGranted}),
        .rankTransition(rankTransition)
    );

    //------------------------------------------------------------------------------
    //      Channel-Wide Scheduling Overview
    //
    //      CMD Bus:
    //          - One RankController can issue a command at a time
    //          - Enforces tRTR (rank-to-rank turnaround)
    //
    //      DQ Bus:
    //          - Enforces CAS-to-CAS timing (tCCD_S/L)
    //          - Enforces read/write turnaround timing (tRTW, tWTR_S/L)
    //
    //      ChannelController arbitrates both domains independently
    //------------------------------------------------------------------------------
    CMDTurnaroundGrant #(
        .tRTRS(tRTRS)
    ) CMDTurnaroundGrant_Instance(    // CMD Bus tRTR timing scheduling
        .clk(clk), .rst(rst),
        .rankTransition(rankTransition), 
        .CMDTurnaroundFree(CMDTurnaroundFree)
    );
    DQTurnaroundGrant #(
        .tRTW(tRTW),
        .tWTRS(tWTRS),
        .tWTRL(tWTRL)
    ) DQTurnaroundGrant_Instance(      
        .clk(clk), .rst(rst),
        .channelMode(channelMode),
        .rankChanged(rankTransition),
        .DQTurnaroundFree(DQTurnaroundFree)
    );



    //------------------------------------------------------------------------------
    //      DQ Bus Scheduling
    //
    //      - CAS-to-CAS timing (tCCD_S/L) enforced by DQRdWrGrantArbiter
    //      - Read/Write turnaround (tRTW, tWTR_S/L) enforced by DQTurnaroundCalculator
    //      - DQFree indicates whether data transfer can proceed
    //
    //   NOTE: Current implementation assumes NUMRANK = 4.
    //------------------------------------------------------------------------------
    DQRdWrCCDGrant #(
        .tCCDS(tCCDS),
        .tCCDL(tCCDL)
    ) DQRdWrCCDGrant_Instance(                       
        .clk(clk), .rst(rst), 
        .chRdWrACK(|{chSchedVector[3].chSchedACK, chSchedVector[2].chSchedACK, chSchedVector[1].chSchedACK, chSchedVector[0].chSchedACK}),
        .CCDType(|{chSchedVector[3].ccdType, chSchedVector[2].ccdType, chSchedVector[1].ccdType, chSchedVector[0].ccdType}),
        .chRdWrAvailabe(chRdWrAvailabe)
    );
    assign DQFree = chRdWrAvailabe && DQTurnaroundFree;   


    
    //------------------------------------------------------------------------------
    //      Per-Rank Controllers
    //
    //       - Each RankController manages:
    //              * Row/Bank FSMs
    //              * Open-page tracking
    //              * Bank-level timing (tRP, tRCD, tWR, tRFC, etc.)
    //       - ChannelController only coordinates channel-wide conflicts
    //
    //   NOTE: Current implementation assumes NUMRANK = 4.
    //------------------------------------------------------------------------------

    wire [NUMRANK-1:0] rankIdle; 
    logic [NUMRANK-1:0] issuableRank;

    assign issuable = |issuableRank;

    RankController #(
    .FSM_CHANNEL(DEVICE_CHANNEL),
    .FSM_RANK(0),
    .MEM_IDWIDTH(MEM_IDWIDTH),
    .MEM_USERWIDTH(MEM_USERWIDTH),
    .READCMDQUEUEDEPTH(READCMDQUEUEDEPTH),
    .WRITECMDQUEUEDEPTH(WRITECMDQUEUEDEPTH),
    .OPENPAGELISTDEPTH(OPENPAGELISTDEPTH),
    .AGINGWIDTH(AGINGWIDTH),
    .BKWIDTH(BKWIDTH),
    .BGWIDTH(BGWIDTH),
    .RWIDTH(RWIDTH),
    .CWIDTH(CWIDTH),
    .THRESHOLD(THRESHOLD),
    .NUMBANK(NUMBANK),
    .COMMAND_WIDTH(COMMAND_WIDTH),
    .NUM_BANKFSM_BIT(NUM_BANKFSM_BIT), .NUM_BANKFSM(NUM_BANKFSM),

    .NUMBANKGROUP(NUMBANKGROUP),
    .tRP(tRP), .tWR(tWR), .tRFC(tRFC), .tREFI(tREFI), .tRCD(tRCD),
    .MemoryAddress(MemoryAddress),
    .FSMRequest(FSMRequest)
    ) Rank0(
        // common
        .clk(clk), .rst(rst),
        // FrontEnd-Side
        .RankReqMemAddr(rankReqVector[0].addr), .RankReqValid(rankReqVector[0].reqValid), 
        .RankReqId(rankReqVector[0].id), .RankReqUser(rankReqVector[0].user), .RankReqType(rankReqVector[0].reqType), 
        .RankReadReqReady(rankReqVector[0].reqReadReqReady), .RankWriteReqReady(rankReqVector[0].reqWriteReqReady),
        // ChannelScheduler + Arbiter
        .chSchedRdReady(chSchedVector[0].chSchedRdReady), .chSchedWrReady(chSchedVector[0].chSchedWrReady), .chSchedFSMWait(chSchedVector[0].chSchedFSMWait),
        .chSchedCMDGranted(chSchedVector[0].chSchedCMDGranted && CMDTurnaroundFree), .chSchedDQGranted(DQFree), 
        .chSchedWriteMode(channelMode), .ReadReqCnt(RDReqCnt[0]), .WriteReqCnt(WRReqCnt[0]),
        .chSchedRdWrACK(chSchedVector[0].chSchedACK), .chSchedCMDACK(chSchedVector[0].chSchedGrantACK),
        .chSchedCCDType(chSchedVector[0].ccdType), .chSchedRankIdle(rankIdle[0]), .chSchedTransReady(ChannelRDWRTransReady),
        // MEMBuffer-Side
        .rdBufAvailable(rdBufAvailable), .wrBufAvailable(wrBufAvailable), .rdBufRankAvailable(rdBufRankWindowAvailable[0]),
        .bufReadPreACK(memBufferVector[0].bufReadPreACK), .bufWritePreACK(memBufferVector[0].bufWritePreACK),
        .bufBankPre(memBufferVector[0].bufBankPre), 
        .bufReadReqACK(memBufferVector[0].bufReadReqACK), .bufWriteReqACK(memBufferVector[0].bufWriteReqACK),
        .bufReadReqId(memBufferVector[0].bufReadReqId), .bufWriteReqId(memBufferVector[0].bufWriteReqId),
        .bufReadReqUser(memBufferVector[0].bufReadReqUser), .bufWriteReqUser(memBufferVector[0].bufWriteReqUser),
        .bufReqACKAddr(memBufferVector[0].bufReqACKAddr),
        .issuable(issuableRank[0]),
        // DDR4 PHY-Side
        .cke(cke[0]), .cs_n(cs_n[0]), .par(par[0]), .act_n(act_n[0]),
        .pin_A(pin_A[0]), .bg(bg[0]), .b(b[0])       
    );

    RankController #(
    .FSM_CHANNEL(DEVICE_CHANNEL),
    .FSM_RANK(1),
    .MEM_IDWIDTH(MEM_IDWIDTH),
    .MEM_USERWIDTH(MEM_USERWIDTH),
    .READCMDQUEUEDEPTH(READCMDQUEUEDEPTH),
    .WRITECMDQUEUEDEPTH(WRITECMDQUEUEDEPTH),
    .OPENPAGELISTDEPTH(OPENPAGELISTDEPTH),
    .AGINGWIDTH(AGINGWIDTH),
    .BKWIDTH(BKWIDTH),
    .BGWIDTH(BGWIDTH),
    .RWIDTH(RWIDTH),
    .CWIDTH(CWIDTH),
    .COMMAND_WIDTH(COMMAND_WIDTH),
    .NUM_BANKFSM_BIT(NUM_BANKFSM_BIT), .NUM_BANKFSM(NUM_BANKFSM),

    .THRESHOLD(THRESHOLD),
    .NUMBANK(NUMBANK),
    .NUMBANKGROUP(NUMBANKGROUP),
    .tRP(tRP),  .tWR(tWR), .tRFC(tRFC), .tREFI(tREFI), .tRCD(tRCD),
    .MemoryAddress(MemoryAddress),
    .FSMRequest(FSMRequest)
    ) Rank1(
        // common
        .clk(clk), .rst(rst),
        // FrontEnd-Side
        .RankReqMemAddr(rankReqVector[1].addr), .RankReqValid(rankReqVector[1].reqValid), 
        .RankReqId(rankReqVector[1].id), .RankReqUser(rankReqVector[1].user), .RankReqType(rankReqVector[1].reqType), 
        .RankReadReqReady(rankReqVector[1].reqReadReqReady), .RankWriteReqReady(rankReqVector[1].reqWriteReqReady),
        // ChannelScheduler + Arbiter
        .chSchedRdReady(chSchedVector[1].chSchedRdReady), .chSchedWrReady(chSchedVector[1].chSchedWrReady), .chSchedFSMWait(chSchedVector[1].chSchedFSMWait),
        .chSchedCMDGranted(chSchedVector[1].chSchedCMDGranted && CMDTurnaroundFree), .chSchedDQGranted(DQFree), 
        .chSchedWriteMode(channelMode), .ReadReqCnt(RDReqCnt[1]), .WriteReqCnt(WRReqCnt[1]),
        .chSchedRdWrACK(chSchedVector[1].chSchedACK), .chSchedCMDACK(chSchedVector[1].chSchedGrantACK),
        .chSchedCCDType(chSchedVector[1].ccdType), .chSchedRankIdle(rankIdle[1]), .chSchedTransReady(ChannelRDWRTransReady),
        // MEMBuffer-Side
        .rdBufAvailable(rdBufAvailable), .wrBufAvailable(wrBufAvailable),  .rdBufRankAvailable(rdBufRankWindowAvailable[1]),
        .bufReadPreACK(memBufferVector[1].bufReadPreACK), .bufWritePreACK(memBufferVector[1].bufWritePreACK),
        .bufBankPre(memBufferVector[1].bufBankPre), 
        .bufReadReqACK(memBufferVector[1].bufReadReqACK), .bufWriteReqACK(memBufferVector[1].bufWriteReqACK),
        .bufReadReqId(memBufferVector[1].bufReadReqId), .bufWriteReqId(memBufferVector[1].bufWriteReqId),
        .bufReadReqUser(memBufferVector[1].bufReadReqUser), .bufWriteReqUser(memBufferVector[1].bufWriteReqUser),
        .bufReqACKAddr(memBufferVector[1].bufReqACKAddr),
        .issuable(issuableRank[1]),
        
        // DDR4 PHY-Side
        .cke(cke[1]), .cs_n(cs_n[1]),  .par(par[1]), .act_n(act_n[1]),
        .pin_A(pin_A[1]), .bg(bg[1]), .b(b[1])     
        );

    RankController #(
    .FSM_CHANNEL(DEVICE_CHANNEL),
    .FSM_RANK(2),
    .MEM_IDWIDTH(MEM_IDWIDTH),
    .MEM_USERWIDTH(MEM_USERWIDTH),
    .READCMDQUEUEDEPTH(READCMDQUEUEDEPTH),
    .WRITECMDQUEUEDEPTH(WRITECMDQUEUEDEPTH),
    .OPENPAGELISTDEPTH(OPENPAGELISTDEPTH),
    .AGINGWIDTH(AGINGWIDTH),
    .BKWIDTH(BKWIDTH),
    .BGWIDTH(BGWIDTH),
    .RWIDTH(RWIDTH),
    .CWIDTH(CWIDTH),
    .COMMAND_WIDTH(COMMAND_WIDTH),
    .NUM_BANKFSM_BIT(NUM_BANKFSM_BIT), .NUM_BANKFSM(NUM_BANKFSM),

    .THRESHOLD(THRESHOLD),
    .NUMBANK(NUMBANK),
    .NUMBANKGROUP(NUMBANKGROUP),
    .tRP(tRP), .tWR(tWR), .tRFC(tRFC), .tREFI(tREFI), .tRCD(tRCD),
    .MemoryAddress(MemoryAddress),
    .FSMRequest(FSMRequest)
    ) Rank2(
        // common
        .clk(clk), .rst(rst),
        // FrontEnd-Side
        .RankReqMemAddr(rankReqVector[2].addr), .RankReqValid(rankReqVector[2].reqValid), 
        .RankReqId(rankReqVector[2].id), .RankReqUser(rankReqVector[2].user), .RankReqType(rankReqVector[2].reqType), 
        .RankReadReqReady(rankReqVector[2].reqReadReqReady), .RankWriteReqReady(rankReqVector[2].reqWriteReqReady),
        // ChannelScheduler + Arbiter
        .chSchedRdReady(chSchedVector[2].chSchedRdReady), .chSchedWrReady(chSchedVector[2].chSchedWrReady), .chSchedFSMWait(chSchedVector[2].chSchedFSMWait),
        .chSchedCMDGranted(chSchedVector[2].chSchedCMDGranted && CMDTurnaroundFree), .chSchedDQGranted(DQFree), 
        .chSchedWriteMode(channelMode), .ReadReqCnt(RDReqCnt[2]), .WriteReqCnt(WRReqCnt[2]),
        .chSchedRdWrACK(chSchedVector[2].chSchedACK), .chSchedCMDACK(chSchedVector[2].chSchedGrantACK),
        .chSchedCCDType(chSchedVector[2].ccdType), .chSchedRankIdle(rankIdle[2]), .chSchedTransReady(ChannelRDWRTransReady),
        // MEMBuffer-Side
        .rdBufAvailable(rdBufAvailable), .wrBufAvailable(wrBufAvailable), .rdBufRankAvailable(rdBufRankWindowAvailable[2]),
        .bufReadPreACK(memBufferVector[2].bufReadPreACK), .bufWritePreACK(memBufferVector[2].bufWritePreACK),
        .bufBankPre(memBufferVector[2].bufBankPre), 
        .bufReadReqACK(memBufferVector[2].bufReadReqACK), .bufWriteReqACK(memBufferVector[2].bufWriteReqACK),
        .bufReadReqId(memBufferVector[2].bufReadReqId), .bufWriteReqId(memBufferVector[2].bufWriteReqId),
        .bufReadReqUser(memBufferVector[2].bufReadReqUser), .bufWriteReqUser(memBufferVector[2].bufWriteReqUser),
        .bufReqACKAddr(memBufferVector[2].bufReqACKAddr),
        .issuable(issuableRank[2]),

        // DDR4 PHY-Side
        .cke(cke[2]), .cs_n(cs_n[2]), .par(par[2]), .act_n(act_n[2]),
        .pin_A(pin_A[2]), .bg(bg[2]), .b(b[2])     
    );

    RankController #(
    .FSM_CHANNEL(DEVICE_CHANNEL),
    .FSM_RANK(3),
    .MEM_IDWIDTH(MEM_IDWIDTH),
    .MEM_USERWIDTH(MEM_USERWIDTH),
    .READCMDQUEUEDEPTH(READCMDQUEUEDEPTH),
    .WRITECMDQUEUEDEPTH(WRITECMDQUEUEDEPTH),
    .OPENPAGELISTDEPTH(OPENPAGELISTDEPTH),
    .AGINGWIDTH(AGINGWIDTH),
    .BKWIDTH(BKWIDTH),
    .BGWIDTH(BGWIDTH),
    .RWIDTH(RWIDTH),
    .CWIDTH(CWIDTH),
    .COMMAND_WIDTH(COMMAND_WIDTH),
    .NUM_BANKFSM_BIT(NUM_BANKFSM_BIT), .NUM_BANKFSM(NUM_BANKFSM),

    .THRESHOLD(THRESHOLD),
    .NUMBANK(NUMBANK),
    .NUMBANKGROUP(NUMBANKGROUP),
    .tRP(tRP), .tWR(tWR), .tRFC(tRFC), .tREFI(tREFI),.tRCD(tRCD),
    .MemoryAddress(MemoryAddress),
    .FSMRequest(FSMRequest)
    ) Rank3(
        // common
        .clk(clk), .rst(rst),
        // FrontEnd-Side
        .RankReqMemAddr(rankReqVector[3].addr), .RankReqValid(rankReqVector[3].reqValid), 
        .RankReqId(rankReqVector[3].id), .RankReqUser(rankReqVector[3].user), .RankReqType(rankReqVector[3].reqType), 
        .RankReadReqReady(rankReqVector[3].reqReadReqReady), .RankWriteReqReady(rankReqVector[3].reqWriteReqReady),
        // ChannelScheduler + Arbiter
        .chSchedRdReady(chSchedVector[3].chSchedRdReady), .chSchedWrReady(chSchedVector[3].chSchedWrReady), .chSchedFSMWait(chSchedVector[3].chSchedFSMWait),
        .chSchedCMDGranted(chSchedVector[3].chSchedCMDGranted && CMDTurnaroundFree), .chSchedDQGranted(DQFree), 
        .chSchedWriteMode(channelMode), .ReadReqCnt(RDReqCnt[3]), .WriteReqCnt(WRReqCnt[3]),
        .chSchedRdWrACK(chSchedVector[3].chSchedACK), .chSchedCMDACK(chSchedVector[3].chSchedGrantACK),
        .chSchedCCDType(chSchedVector[3].ccdType), .chSchedRankIdle(rankIdle[3]), .chSchedTransReady(ChannelRDWRTransReady),
        // MEMBuffer-Side
        .rdBufAvailable(rdBufAvailable), .wrBufAvailable(wrBufAvailable), .rdBufRankAvailable(rdBufRankWindowAvailable[3]),
        .bufReadPreACK(memBufferVector[3].bufReadPreACK), .bufWritePreACK(memBufferVector[3].bufWritePreACK),
        .bufBankPre(memBufferVector[3].bufBankPre), 
        .bufReadReqACK(memBufferVector[3].bufReadReqACK), .bufWriteReqACK(memBufferVector[3].bufWriteReqACK),
        .bufReadReqId(memBufferVector[3].bufReadReqId), .bufWriteReqId(memBufferVector[3].bufWriteReqId),
        .bufReadReqUser(memBufferVector[3].bufReadReqUser), .bufWriteReqUser(memBufferVector[3].bufWriteReqUser),
        .bufReqACKAddr(memBufferVector[3].bufReqACKAddr),
        .issuable(issuableRank[3]),

        // DDR4 PHY-Side
        .cke(cke[3]), .cs_n(cs_n[3]),  .par(par[3]), .act_n(act_n[3]),
        .pin_A(pin_A[3]), .bg(bg[3]), .b(b[3])        

    );

    assign channelIdle = &rankIdle;


    always_comb begin : COMMANDADDRSetup
        chDDR4_CMD_ADDR_IF.cke    =  '0;
        chDDR4_CMD_ADDR_IF.cs_n   =  '1;
        chDDR4_CMD_ADDR_IF.par    =  '0;
        chDDR4_CMD_ADDR_IF.pin_A  =  '0;
        chDDR4_CMD_ADDR_IF.act_n  =  '0;
        chDDR4_CMD_ADDR_IF.bg     =  '0;
        chDDR4_CMD_ADDR_IF.b      =  '0;
        for(int i = 0; i < NUMRANK; i++) begin
            if(chSchedVector[i].chSchedCMDGranted && CMDTurnaroundFree) begin
                chDDR4_CMD_ADDR_IF.cke       =  cke[i];
                chDDR4_CMD_ADDR_IF.cs_n[i]   =  cs_n[i];
                chDDR4_CMD_ADDR_IF.par       =  par[i];
                chDDR4_CMD_ADDR_IF.pin_A     =  pin_A[i];
                chDDR4_CMD_ADDR_IF.act_n     =  act_n[i];
                chDDR4_CMD_ADDR_IF.bg        =  bg[i];
                chDDR4_CMD_ADDR_IF.b         =  b[i];
            end
        end
    end:COMMANDADDRSetup


endmodule
