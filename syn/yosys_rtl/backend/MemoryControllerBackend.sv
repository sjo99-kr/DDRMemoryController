`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////////////
//      MemoryControllerBackend
//
//      Role:
//          Per-channel backend of the Memory Controller.
//          Owns all channel-local resources including:
//              - Channel scheduling (CMD/DQ)
//              - Read/Write buffering
//              - DDR timing enforcement
//              - PHY interfacing
//
//      Architectural Overview:
//
//          MemoryController (Top)
//                |      ∧
//                V      |
//      +---------------------------+
//      |   MemoryControllerBackend |
//      |   (This module)           |
//      +---------------------------+
//          |         ∧        |
//          |         |        V
//          |      ReadBuf   WriteBuf
//       Channel    Ctrl      Ctrl
//        Ctrl        ∧        |
//          |         |        V
//          |        PHYController
//          |         |        |
//          V         V        V
//  DDR IF CMD BUS   DDR4 IF DQ BUS
//
//      Responsibilities:
//          1) Decide READ / WRITE channel mode based on Number of Write Request.
//          2) Schedule DRAM commands through ChannelController.
//          3) Manage Read/Write Buffers and their interaction with PHY.
//          4) Enforce DDR timing constraints (tRP, tWR, tRTR, tWTR, etc.).
//          5) Interface with DDR4 CMD and DQ buses with Channel Controller and PHY Controller.
//
//      Design Notes:
//          - This module is strictly channel-local (no cross-channel logic).
//          - Channel-wide arbitration is handled at MemoryController (top).
//          - Bank-/rank-level timing decisions are delegated to ChannelController.
//          - PHYController abstracts DDR burst timing and data alignment.
//
//      Author  : Seongwon Jo
//      Created : 2026.02
//////////////////////////////////////////////////////////////////////////////////////////

import MemoryController_Definitions::*;

module MemoryControllerBackend(
    input logic clk, rst, clk2x,
                                                                    ///////////////////////////////////////////////////////////////////
                                                                    //     INPUT  FROM  MemoryController FrontEnd  (Request/Data)    //
    input logic [32-1:0] RankReqMemAddr,                             //  1. Memory Request Addresss (Both for CMD & DATA)             //
    input logic [MEM_IDWIDTH-1:0] RankReqId,                        //  2. Memory Request ID                                         //
    input logic [MEM_USERWIDTH-1:0] RankReqUser,                    //  3. Memory Requset User                                       //
    input logic RankReqType,                                        //  4. Memory Request Type (Read: 0; Write: 1)                   //
    input logic RankReqValid,                                       //  5. Memory Request Valid (Only for CMD Valid)                 //
    input logic [MEM_DATAWIDTH-1:0] RankData,                       //  6. Memory Request Data (Only for Write Request Data)         //
    input logic [MEM_DATAWIDTH/BURST_LENGTH -1:0] RankDataStrb,     //  7. Memory Request Data Strb (Masking bits for Write Data)    //
    input logic RankDataLast,                                       //  8. Memory Request Data Last signal (For Write Buffer)        //
    input logic RankDataValid,                                      //  9. Memory Request Data Valid (Only for Data Valid)           //
                                                                    ///////////////////////////////////////////////////////////////////

                                                                    ///////////////////////////////////////////////////////////////////
                                                                    //     INPUT  FROM  MemoryController FrontEnd  (Protocol-Ready)  //
    input logic CacheReadDataReady,                                 //  1. AXI-R Bus Ready (Valid when Cache can receive data)       //
    input logic CacheWriteDataACKReady,                             //  2. AXI-B Bus Ready (Valid when Cache can receive ACK)        //
                                                                    ///////////////////////////////////////////////////////////////////
    
                                                                    ///////////////////////////////////////////////////////////////////
                                                                    //          OUTPUT TO Memory Controller FrontEnd (Data)          //
    output logic [MEM_DATAWIDTH-1:0] CacheReadData,                 //  1. Memory Response Data (From Read Buffer)                   //
    output logic [MEM_USERWIDTH-1:0] CacheReadDataUser,             //  2. Memory Response Data User                                 //
    output logic [MEM_IDWIDTH-1:0]   CacheReadDataId,               //  3. Memory Response Data ID                                   //
    output logic CacheReadDataLast,                                 //  4. Memory Response Data Last                                 //
    output logic CacheReadDataValid,                                //  5. Memory Response Data Valid                                //
    output logic ReadBufferFull,                                    //  6. Read Buffer Full signal (For AXI-AR Bus Ready)            //
    output logic CacheWriteDataACKValid,                            //  7. Memory Response for Write Data ACK Valid (For AXI-B Bus)  //
    output logic [MEM_IDWIDTH-1:0] CacheWriteDataACKID,             //  8. Memory Response for Write Data ACK ID    (For AXI-B Bus)  //
    output logic [MEM_USERWIDTH-1:0] CacheWriteDataACKUser,         //  9. Memory Response for Write Data ACK User  (For AXI-B Bus)  //
    output logic WriteBufferFull,                                   //  10. Write Buffer Full signal (For AXI-AW/W Bus Ready)        //
    output logic [NUMRANK-1:0] RankReadReqReady,                   //  11. Channel Scheduler Ready signal (For AXI-AR Bus Ready)    //
    output logic [NUMRANK-1:0] RankWriteReqReady,                   //  12. Channel Scheduler Ready Signal (For AXI-AW/W Bus Ready)  //
                                                                    ///////////////////////////////////////////////////////////////////
                                                                    ///////////////////////////////////////////////////////////////////
                                                                    //              OUTPUT TO Memory Controller Main Module          //
    output logic [READBUFFERDEPTH-1:0] NumOfReadBufferEntry,        //  1. Read Buffer Data Available  (for Response arbitration)    //
                                                                    ///////////////////////////////////////////////////////////////////
                                                                                               
                                                                    ///////////////////////////////////////////////////////////////////
                                                                    //         INPUT/OUTPUT  FOR  DDR  INTERFACE                     //
   // DDR4DQInterface ddr4_dq_bus,                                //  1. DQ Bus for Read/Write Data                                //
   // DDR4CmdInterface ddr4_cmd_bus                                //  2. CMD Bus for Read/Write CMD/ADDR                           // 

    input logic [MEM_DATAWIDTH-1:0] pin_dq_in, 
    output wire [MEM_DATAWIDTH-1:0] pin_dq_out,
    input logic dqs_c_in, dqs_t_in,
    output logic dqs_c_out, dqs_t_out,

    output logic cke,
    output logic [NUMRANK-1:0] cs_n,
    output logic par,
    output logic act_n,
    output logic [COMMAND_WIDTH-1:0] pin_A,
    output logic [BGWIDTH-1:0] bg,
    output logic [BKWIDTH-1:0] b                                                          //////////////////////////////////////////////////////////////////
);

    //         Channel Mode Controller  (READ COMMAND PRIORITY)           //
    //  Current Scheme : Threshold-based Channel Mode Transition (Static) //
    //      (TODO) Better channel scheduling scheme?  For Dynamically?    //
    logic ChannelRDWRMode, ChannelRDWRTransReady;
    logic [$clog2(READCMDQUEUEDEPTH * NUMRANK):0]  NumRdReq;
    logic [$clog2(WRITECMDQUEUEDEPTH * NUMRANK):0] NumWrReq;

    //------------------------------------------------------------------------------
    //  Channel Read/Write Mode Controller
    //
    //  - Selects current channel mode: READ or WRITE.
    //  - Policy: Threshold-based static mode switching.
    //      * Prefer READ when read buffer has entries.
    //      * Switch to WRITE when write buffer pressure exceeds threshold.
    //  - Mode transitions are synchronized with PHY readiness.
    //
    //  NOTE:
    //      - This is a channel-wide policy (not per-rank).
    //      - More dynamic schemes (e.g., aging-based) are future work.
    //------------------------------------------------------------------------------
    always_ff@(posedge clk or negedge rst) begin
        if(!rst) begin
            ChannelRDWRMode <= 0;           // 0: Read, 1: Write
            ChannelRDWRTransReady <= 0;
        end else begin
            if(NumRdReq == 0 && NumWrReq != 0) begin
                if(ChannelRDWRMode == 0) begin
                    if(!issuable) begin
                        ChannelRDWRTransReady <= 1;
                    end
                    if(channelIdle && ModeTransitionValid && ChannelRDWRTransReady) begin
                        ChannelRDWRTransReady <= 0;
                        ChannelRDWRMode <= 1;
                    end
                end
            end else if(NumRdReq != 0 && NumWrReq == 0) begin
                if(ChannelRDWRMode == 1) begin
                    if(!issuable) begin
                        ChannelRDWRTransReady <= 1;
                    end
                    if(channelIdle && ModeTransitionValid && ChannelRDWRTransReady) begin
                        ChannelRDWRTransReady <= 0;
                        ChannelRDWRMode <= 0;
                    end
                end 
            end else if(ChannelRDWRMode == 0 && (NumWrReq > CHMODETHRESHOLD)) begin
                if(!issuable) begin
                    ChannelRDWRTransReady <= 1;
                end
                if(channelIdle && ModeTransitionValid && ChannelRDWRTransReady) begin
                    ChannelRDWRTransReady <= 0;
                    ChannelRDWRMode <= 1;
                end
            end else if(ChannelRDWRMode == 1 && (NumWrReq << CHMODETHRESHOLD)) begin
                if(!issuable) begin
                    ChannelRDWRTransReady <= 1;
                end
                if(channelIdle && ModeTransitionValid && ChannelRDWRTransReady) begin
                    ChannelRDWRTransReady <= 0;
                    ChannelRDWRMode <= 0;
                end
            end else begin
                if(ChannelRDWRMode == 1)begin
                    if(!issuable)begin
                        ChannelRDWRTransReady <= 1;
                    end
                    if(channelIdle && ModeTransitionValid && ChannelRDWRTransReady) begin
                        ChannelRDWRTransReady <= 0;
                        ChannelRDWRMode <= 0;
                    end
                end  else begin
                    if(channelIdle && ModeTransitionValid && ChannelRDWRTransReady)begin
                        ChannelRDWRMode <= 0;
                        ChannelRDWRTransReady <= 0;
                    end
                end
            end
        end
    end

    //------------------------------------------------------------------------------
    //      Inter-Module Interfaces
    //
    //      ChannelController <-> PHYController
    //          - Command issue handshake
    //          - Auto-precharge acknowledgment
    //
    //      ChannelController <-> ReadBuffer
    //          - Allocation of read response slots
    //
    //      ChannelController <-> WriteBuffer
    //          - Allocation of write data slots
    //
    //      ReadBuffer <-> PHYController
    //          - Read data return path
    //
    //      WriteBuffer <-> PHYController
    //          - Write data issue path
    //------------------------------------------------------------------------------

    //            Channel Controller <--->  PHY Controller                //
    logic PHYReadModePreACK, PHYWriteModePreACK;                //  PHY Controller     -> Channel Controller
    logic [32-1:0] PHYReadModePreAddr, PHYWriteModePreAddr;      //  PHY Controller     -> Channel Controller
    logic PHYReadCMDIssuedACK, PHYWriteCMDIssuedACK;            //  Channel Controller -> PHY Controller
    logic [32-1:0] PHYReadCMDIssuedAddr, PHYWriteCMDIssuedAddr;  //  Channel Controller -> PHY Controller
    //            Channel Controller <--->  Read Buffer                   //
    logic ReadBufferAvailable;                                  //  Read Buffer        -> Channel Controller
    logic [NUMRANK-1:0] ReadRankWindowAvailable;
    logic [MEM_IDWIDTH-1:0] ReadBufferAllocID;                  //  Channel Controller -> Read Buffer
    logic [MEM_USERWIDTH-1:0] ReadBufferAllocUser;              //  Channel Controller -> Read Buffer
    logic ReadBufferAllocValid;                                 //  Channel Controller -> Read Buffer
    logic [32-1:0] ReadBufferAllocAddr;                          //  Channel Controller -> Read Buffer
    //            Channel Controller <--->  Write Buffer                  //
    logic [NUMRANK-1:0] WriteBufferAvailable;                   //  Write Buffer       -> Channel Controller
    logic [MEM_IDWIDTH -1:0] WriteBufferAllocID;                //  Channel Controller -> Write Buffer
    logic [MEM_USERWIDTH-1:0] WriteBufferAllocUser;             //  Channel Controller -> Write Buffer
    logic WriteBufferAllocValid;                                //  Channel Controller -> Write Buffer
    logic [32-1:0] WriteBufferAllocAddr;                         //  Channel Controller -> Write Buffer
    
    //              Read Buffer     <--->     PHY Controller              //
    logic [MEM_DATAWIDTH-1:0] PHYReadModeData;                  //  PHY Controller -> Read Buffer
    logic [NUMRANK-1:0] PHYReadModeDataTag;                     //  PHY Controller -> Read Buffer
    logic PHYReadModeDataValid;                                 //  PHY Controller -> Read Buffer
    logic PHYReadModeDataLast;                                  //  PHY Controller -> Read Buffer
    logic PHYReadModeFIFOReady;                                 //  Read Buffer    -> PHY Controller

    //              Write Buffer     <--->     PHY Controller             //
    logic [MEM_DATAWIDTH-1:0] PHYWriteModeData;                 //  Write Buffer   ->  PHY Controller
    logic PHYWriteModeDataLast;                                 //  Write Buffer   ->  PHY Controller
    logic PHYWriteModeDataValid;                                //  Write Buffer   ->  PHY Controller
    logic [MEM_DATAWIDTH/BURST_LENGTH-1:0]PHYWriteModeDataStrb; //  Write Buffer   ->  PHY Controller
    logic PHYWriteModeFIFOReady;                                //  PHY Controller ->  Write Buffer
    logic WriteModeACK;

    //            Write Buffer     <--->   Memory Controller BackEnd      //
    logic [WRITEBUFFERDEPTH-1:0] NumOfWriteBufferEntry;         //  Memory Controller Backend ->  Write Buffer

    //          PHY Controller      <--->   Memory Controller BackEnd     //
    logic ModeTransitionValid;                                  //  PHY Controller  ->  Memory Controller Backend


    logic channelIdle;
    wire issuable;

    //------------------------------------------------------------------------------
    //      ChannelController
    //
    //      - Performs rank/bank-level command scheduling.
    //      - Enforces DRAM timing constraints on CMD bus.
    //      - Arbitrates among Rank FSMs for ACT/RD/WR/PRE commands.
    //      - Does NOT handle data movement (DQ).
    //------------------------------------------------------------------------------
    ChannelController ChannelController_Instance (
        .clk(clk), .rst(rst),
            //          INPUT   FROM    MemoryController FrontEnd           //
        .RankReqMemAddr(RankReqMemAddr), .RankReqId(RankReqId), .RankReqUser(RankReqUser),
        .RankReqType(RankReqType), .RankReqValid(RankReqValid),
        
            //          OUTPUT    TO    MemoryController FrontEnd             //
        .RankReadReqReady(RankReadReqReady), .RankWriteReqReady(RankWriteReqReady),

            //              INPUT   FROM    READ BUFFER                       //
        .rdBufAvailable(ReadBufferAvailable), .rdBufRankWindowAvailable(ReadRankWindowAvailable),

            //               INPUT   FROM    WRITE BUFFER                     //
        .wrBufAvailable(&WriteBufferAvailable),

            //                  OUTPUT  TO      READ BUFFER                   //
        .bufReadReqId(ReadBufferAllocID), .bufReadReqUser(ReadBufferAllocUser), 
        .bufReadReqACK(ReadBufferAllocValid), .bufReadReqACKAddr(ReadBufferAllocAddr),
        .bufWriteReqId(WriteBufferAllocID), .bufWriteReqUser(WriteBufferAllocUser), 
        .bufWriteReqACK(WriteBufferAllocValid), .bufWriteReqACKAddr(WriteBufferAllocAddr),

            //                  INPUT   FROM    PHYCONTROLLER                 //
        .PHYReadModePreACK(PHYReadModePreACK), .PHYWriteModePreACK(PHYWriteModePreACK),
        .PHYReadModePreAddr(PHYReadModePreAddr), .PHYWriteModePreAddr(PHYWriteModePreAddr),

            //                  OUTPUT  FROM   PHYCONTROLLER                  //
        .phyReadCMDIssuedACK(PHYReadCMDIssuedACK), .phyReadCMDIssuedAddr(PHYReadCMDIssuedAddr),
        .phyWriteCMDIssuedACK(PHYWriteCMDIssuedACK), .phyWriteCMDIssuedAddr(PHYWriteCMDIssuedAddr),
        
            //               INPUT FROM MC BACKEND                            //
        .channelMode(ChannelRDWRMode),.NumRdReq(NumRdReq), .NumWrReq(NumWrReq),
        .issuable(issuable),

            //                    DDR INTERFACE                               //
        .cke_t(cke), .cs_n_t(cs_n), .par_t(par), .act_n_t(act_n),
        .pin_A_t(pin_A), .bg_t(bg), .b_t(b)
    );

    //------------------------------------------------------------------------------
    //      ReadBufferController
    //
    //      - Stores read response data returned from PHY.
    //      - Tracks outstanding read transactions using directory entries.
    //      - Supplies data back to frontend when cache is ready.
    //------------------------------------------------------------------------------
    ReadBufferController ReadBufferScheduler_Instance (        
        .clk(clk), .rst(rst),
        //                  INPUT FROM MemoryController FrontEnd               //
        .readDataReady(CacheReadDataReady),        // Cache is Ready?

        //                  OUTPUT  TO MemoryController FrontEnd               //
        .readData(CacheReadData), .readDataUser(CacheReadDataUser), .readDataID(CacheReadDataId),
        .readDataLast(CacheReadDataLast), .readDataValid(CacheReadDataValid),
        .readBufferFull(ReadBufferFull),             // Buffer is ready when dir,and data is not full.

        //                  INPUT FROM PHY COTNROLLER                         //
        .writeData(PHYReadModeData), .writeDataTag(PHYReadModeDataTag), 
        .writeDataValid(PHYReadModeDataValid), .writeDataLast(PHYReadModeDataLast),
        .phyReadModeFIFOReady(PHYReadModeFIFOReady),

        //                  INPUT FROM CHANNEL SCHEDULER                       //
        .readBufferAllocID(ReadBufferAllocID), .readBufferAllocUser(ReadBufferAllocUser),
        .readBufferAllocAddr(ReadBufferAllocAddr), .readBufferAllocValid(ReadBufferAllocValid),

        //                  OUTPUT TO CHANNEL SCHEDULER                        //
        .readBufferAvailable(ReadBufferAvailable),
        .readRankWindowAvailable(ReadRankWindowAvailable),

        //                  OUTPUT TO   Memory Controller BackEnd              //
        .readBufferCnt(NumOfReadBufferEntry)
    );

    //------------------------------------------------------------------------------
    //  WriteBufferController
    //
    //  - Buffers incoming write data from frontend.
    //  - Maintains write ordering and burst assembly.
    //  - Supplies write data to PHY when scheduled.
    //  - Generates write acknowledgments (AXI-B).
    //------------------------------------------------------------------------------
    WriteBufferController  WriteBufferScheduler_Instance (
        .clk(clk), .rst(rst),
        //               INPUT    FROM    Memory Controller FrontEnd            //    
        .writeData(RankData), .writeDataStrb(RankDataStrb), .writeDataLast(RankDataLast), .writeDataValid(RankDataValid), 
        .writeDataUser(RankReqUser), .writeDataID(RankReqId), .writeDataAddr(RankReqMemAddr),
        .writeDataACKReady(CacheWriteDataACKReady),

        //                  OUTPUT TO   Memory Controller FrontEnd              //
        .writeDataACKValid(CacheWriteDataACKValid), .writeDataACKUser(CacheWriteDataACKUser),
        .writeDataACKID(CacheWriteDataACKID), .writeBufferFull(WriteBufferFull),
        
        //                  INPUT FROM  PHY Controller                          //
        .phyWriteModeFIFOReady(PHYWriteModeFIFOReady),
        .WriteModeACK(WriteModeACK),

        //                      OUTPUT TO   PHY Controller                      //
        .readData(PHYWriteModeData), .readDataLast(PHYWriteModeDataLast), 
        .readDataValid(PHYWriteModeDataValid), .readDataStrb(PHYWriteModeDataStrb), 

        //                      INPUT FROM CHANNEL SCHEDULER                    //
        .writeBufferAllocID(WriteBufferAllocID), .writeBufferAllocUser(WriteBufferAllocUser), 
        .writeBufferAllocAddr(WriteBufferAllocAddr), .writeBufferAllocValid(WriteBufferAllocValid),

        //                      OUTPUT TO CHANNEL SCHEUDLER                     //
        .writeBufferAvailable(WriteBufferAvailable),

        //                  OUTPUT TO Memory Controller BackEnd                 //
        .WriteRequestWindow(NumOfWriteBufferEntry)
    );

    //------------------------------------------------------------------------------
    //      PHYController
    //
    //      - Bridges logical DRAM transactions and physical DDR signaling.
    //      - Manages burst timing (tCL, tCWL) and FIFO alignment.
    //      - Handles DQ data path for both READ and WRITE modes.
    //      - Provides mode-transition synchronization to backend.
    //
    //  NOTE:
    //      - All DDR electrical/timing abstraction is isolated here.
    //------------------------------------------------------------------------------
    PHYController PHYController_Instance(
        .clk(clk), .rst(rst), .mode(ChannelRDWRMode),
        .clk2x(clk2x),
        //                         INPUT FROM WRITE BUFFER                     //
        .writeBufferData(PHYWriteModeData), .writeBufferDataLast(PHYWriteModeDataLast),
        .writeBufferDataValid(PHYWriteModeDataValid), .writeBufferDataStrb(PHYWriteModeDataStrb),

        //                       OUTPUT TO  WRITE BUFFER                       //
        .WriteModeFIFOReady(PHYWriteModeFIFOReady),
        .WriteModeACK(WriteModeACK),

        //                         OUTPUT TO READ BUFFER                       //
        .readBufferData(PHYReadModeData), .readBufferDataTag(PHYReadModeDataTag),
        .readBufferDataValid(PHYReadModeDataValid), .readBufferDataLast(PHYReadModeDataLast),
        .ReadModeFIFOReady(PHYReadModeFIFOReady),

        //                       INPUT FROM CHANNEL SCHEDULER                  //
        .ReadCMDIssuedACK(PHYReadCMDIssuedACK), .ReadCMDIssuedAddr(PHYReadCMDIssuedAddr),
        .WriteCMDIssuedACK(PHYWriteCMDIssuedACK), .WriteCMDIssuedAddr(PHYWriteCMDIssuedAddr),

        //                        OUTPUT TO CHANNEL SCHEDULER                  //
        .ReadAutoPrechargeACK(PHYReadModePreACK), .ReadAutoPrechargeAddr(PHYReadModePreAddr), 
        .WriteAutoPrechargeACK(PHYWriteModePreACK), .WriteAutoPrechargeAddr(PHYWriteModePreAddr),

        //                    OUTPUT TO MEMORYCONTROLLER BACKEND               //
        .ModeTransitionReady(ModeTransitionValid), 

        //                       DDR4 DQ BUS INTERFACE                         //
        .pin_dq_in(pin_dq_in), .pin_dq_out(pin_dq_out), 
        .dqs_c_in(dqs_c_in), .dqs_c_out(dqs_c_out),
        .dqs_t_in(dqs_t_in), .dqs_t_out(dqs_t_out),
    );


endmodule
