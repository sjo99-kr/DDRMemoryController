`timescale 1ns / 1ps

//------------------------------------------------------------------------------
//      RankSched
//
//      Role:
//          Rank-level request scheduler for DDR memory system.
//
//      Responsibilities:
//          - Buffers read/write requests from MC frontend.
//          - Applies FR-FCFS policy with open-page awareness.
//          - Manages refresh scheduling (tREFI).
//          - Selects next request to issue to BankFSM.
//          - Tracks open rows and request aging.
//
//      Scheduling Policy:
//          - FR-FCFS (First-Ready, First-Come-First-Serve)
//          - Priority order:
//              1) Requests beyond aging threshold
//              2) Row-hit with short tCCD
//              3) Row-hit with long tCCD
//              4) Oldest request (aging-based)
//
//      Scope & Notes:
//          - Operates at rank granularity.
//          - No command-level timing (handled by RankExecutionUnit).
//          - One request issued at a time to RankExecutionUnit.
//
//      Author  : Seongwon Jo
//      Created : 2026.02
//------------------------------------------------------------------------------

module RankSched #(
    parameter int FSM_CHANNEL        = 0,
    parameter int FSM_RANK           = 0,
    
    parameter int NUM_BANKFSM        = 16,
    parameter int NUM_BANKFSM_BIT    = 4,

    parameter int MEM_IDWIDTH        = 4,
    parameter int MEM_USERWIDTH      = 1,
    parameter int READCMDQUEUEDEPTH  = 8,
    parameter int WRITECMDQUEUEDEPTH = 8,
    parameter int OPENPAGELISTDEPTH  = 16,
    parameter int AGINGWIDTH         = 10,
    parameter int RWIDTH             = 15,
    parameter int CWIDTH             = 10,
    parameter int THRESHOLD          = 512,
    parameter int tREFI              = 8192,
    parameter type FSMRequest        = logic,
    parameter type MemoryAddress     = logic
)(
    // Common
    input logic clk, rst,

    input MemoryAddress RankReqMemAddr,                                             
    input logic [MEM_IDWIDTH-1:0] RankReqId,                                 
    input logic [MEM_USERWIDTH-1:0] RankReqUser,                            
    input logic RankReqType, 
    input logic RankReqValid,                              
                     
    output logic RankReadReqReady,                    // Valid When RD Req. Que. is not full & RD MEM Buffer is available
    output logic RankWriteReqReady,                   // Valid When WR Req. Que. is not full & WR MEM Buffer is available

                                                      //         Input from Channel Scheduler        //
    input logic chSchedCMDOnlyValid,                  //        (Input from Channel Scheduler)       //
    input logic chSchedCMDDQValid,                    //        (Input from Channel Scheduler)       //
    input wire chSchedTransReady,
    output logic chSchedAvailableCMD,                 //       (Output to RankExecutionUnit)         //
    output logic chSchedAvailableCMDDQ,               //       (Output to RankExecutionUnit)         //

    input logic WriteMode,                            

    output logic chSchedRdReady,      
    output logic chSchedWrReady,

    input logic fsmChSchedAck,                        //      (Input from RankExecutionUnit)         //    
    input logic [NUM_BANKFSM-1:0] fsmWait,            //      (Input fron RankExecutionUnit)         //
                                                      //  Valid when RankExecutionUnit is WAITING for DRAM TIMING CONSTRAINTS.    
    output logic chSchedACK,                          //       (Output to RankExecutionUnit)         //
    output logic chSchedIdle,                         //       (Output to RankExecutionUnit)         //
        
    output logic [$clog2(READCMDQUEUEDEPTH)-1:0] chSchedReadReqCnt,
    output logic [$clog2(WRITECMDQUEUEDEPTH)-1:0] chSchedWriteReqCnt,

    input logic rdBufAvailable,                       //         (Input from MEM Buffer)           //
    input logic wrBufAvailable,                       //         (Input from MEM Buffer)           //
    output logic fsmWrBufValid,                       //        (Ouput to RankExecutionUnit)       //
    output logic fsmRdBufValid,                       //        (Ouput to RankExecutionUnit)       //

    output logic [MEM_IDWIDTH-1:0] readBufReqId, 
    output logic [MEM_USERWIDTH-1:0] readBufReqUser,
    output logic readBufReqACK,
    output logic [MEM_IDWIDTH-1:0] writeBufReqId,
    output logic [MEM_USERWIDTH-1:0] writeBufReqUser,
    output logic writeBufReqACK,

    input logic [NUM_BANKFSM-1:0] fsmIdle,            // Valid when RankExecutionUnit is Idle State, Ready for receiving NEW REQUEST.
    input logic fsmRefreshAck,                        // Valid when RankExecutionUnit finished Refresh phase.
    output logic refresh,                             // Valid when the timing for tREFI is set.

    input logic [MEM_IDWIDTH-1:0] fsmBufWrReqId, fsmBufRdReqId,
    input logic [MEM_USERWIDTH-1:0] fsmBufWrReqUser, fsmBufRdReqUser, 
    input logic fsmBufWrReqIssued,                 
    input logic fsmBufRdReqIssued,

    output logic [NUM_BANKFSM-1:0] fsmIssue,           // Valid when Request Scheduler sends Request to RankExecutionUnit.
    output FSMRequest fsmIssuedReq,                    // Request for RankExecutionUnit when RankExecutionUnit is Idle.

    output logic issuable
    );

    localparam int CMDQUEUEDEPTH = (READCMDQUEUEDEPTH > WRITECMDQUEUEDEPTH) ? READCMDQUEUEDEPTH : WRITECMDQUEUEDEPTH;

    //------------------------ Interface signals Setup ----------------------//
    //                RankSched/FSM <-> ChannelSched/Arbiter                 //
    assign chSchedRdReady =  (!WriteMode) && !(&fsmIdle) && !(&fsmWait); 
    assign chSchedWrReady =   (WriteMode) && !(&fsmIdle) && !(&fsmWait);
    assign chSchedIdle = &fsmIdle;
    assign chSchedACK = fsmChSchedAck;                  

    assign chSchedAvailableCMD = chSchedCMDOnlyValid;
    assign chSchedAvailableCMDDQ = chSchedCMDDQValid;

    //                  MEMBuffer-side <-> RankSched/RankExecutionUnit                   //
    assign readBufReqId = fsmBufRdReqId;
    assign readBufReqUser = fsmBufRdReqUser;

    assign writeBufReqId = fsmBufWrReqId;
    assign writeBufReqUser = fsmBufWrReqUser;

    assign writeBufReqACK = fsmBufWrReqIssued;
    assign readBufReqACK = fsmBufRdReqIssued;

    assign fsmRdBufValid = rdBufAvailable;
    assign fsmWrBufValid = wrBufAvailable;
    //////////////////////////////////////////////////////////////////////////

    //------------------------ Struct Definition ---------------------------//
    typedef struct packed{
        MemoryAddress mem_addr;
        logic req_type;
        logic [MEM_IDWIDTH-1:0] req_id;
        logic [MEM_USERWIDTH-1:0] req_user; 
        logic [AGINGWIDTH-1:0] cnt;
    } ReqQueEntry;

    typedef struct packed {
        logic valid;
        logic [RWIDTH-1:0] RowAddr;
    }OpenRowEntry;
    //////////////////////////////////////////////////////////////////////////

    //      Memory Request Queue for Read Request and Write Request         //
    ReqQueEntry ReadRequestQueue [0: READCMDQUEUEDEPTH - 1];
    ReqQueEntry WriteRequestQueue [0: WRITECMDQUEUEDEPTH - 1];
    logic [$clog2(READCMDQUEUEDEPTH) - 1 : 0] ReadPushPtr, ReadPopPtr;
    logic [$clog2(WRITECMDQUEUEDEPTH) - 1 : 0] WritePushPtr, WritePopPtr;
    logic [READCMDQUEUEDEPTH - 1:0] ReadReqQueFree;
    logic [WRITECMDQUEUEDEPTH - 1:0] WriteReqQueFree;
    logic ReadReqQueFull, ReadReqQueEmpty; 
    logic WriteReqQueFull, WriteReqQueEmpty;

    //            OpenPagePolicy management  (OpenPagePolicy List)         //
    OpenRowEntry OpenRowList [OPENPAGELISTDEPTH - 1 : 0 ];         // OpenRowDepth -> Num. of BankGroup * Num. of Bank., One bank has one Row (Page).
    logic checkThreshold;                                          // Threshold for blocking starvation in Request Queue. (request aging)
    logic [$clog2(CMDQUEUEDEPTH) - 1 : 0] MaxValue;                // MaxValue for Oldes Entry in Request Queue.
    
    logic [1:0] PageHitT;                                          // PageHit indicates a row-hit condition within a bank.
                                                                   // In a row-hit case, READ or WRITE commands can be issued
                                                                   // without an ACT command.
                                                                   //
                                                                   // When consecutive column commands (READ/WRITE) are issued:
                                                                   // - If the commands target banks within the same bank group,
                                                                   //    tCCD_S (short) timing constraint applies.
                                                                   // - If the commands target banks in different bank groups,
                                                                   //    tCCD_L (long) timing constraint applies.

    logic PageMissT, PageEmptyT;                                   // PageMiss -> Need to Precharge on current Row(Page), and ACT the target Row.
                                                                   // PageEmpty -> Need to ACT the target Row without Precharge
    logic NextPageHitS, NextPageHitL;       
    logic [$clog2(CMDQUEUEDEPTH) -1 :0] PageHitIndexS, PageHitIndexL;

    // Refresh management
    logic [$clog2(tREFI)-1:0] refresh_cnt;                         // Refresh Interval Count
    logic refreshing;                                              // Refresh flag.
                                                                   // Normally, refresh is issued at the rank level.

    // AutoPrecharge Commnad
    logic checkAutoPrecharge;

    logic [$clog2(CMDQUEUEDEPTH) - 1 : 0] MaxValue_reg;
    logic NextPageHitS_reg, NextPageHitL_reg;
    logic checkThreshold_reg;
    logic SchedValid_2, SchedValid_3;
    logic [$clog2(CMDQUEUEDEPTH) -1 :0] PageHitIndexS_reg, PageHitIndexL_reg;

    //------------------------------------------------------------------------------
    //  Refresh Scheduler
    //
    //  - Tracks refresh interval (tREFI).
    //  - Issues rank-level refresh request when interval expires.
    //  - Refresh is only triggered when RankExecutionUnit is idle.
    //
    //------------------------------------------------------------------------------
    always_ff@(posedge clk or negedge rst)begin
        if(!rst) begin
            refresh_cnt <= 0;
            refreshing <= 0;
        end else begin
            if(refreshing == 0) begin
                refresh_cnt <= refresh_cnt + 1;
                if(refresh_cnt == (tREFI - 1)) begin        // tREFI : Refresh Interval Cycles.
                    refreshing <= 1;                        // Refresh Flag Setup.
                    refresh_cnt <= 0;
                end
            end else begin
                if(fsmRefreshAck) begin                     // ACK from RankExecutionUnit for Refreshing done.
                    refreshing <= 0; 
                end
            end
        end
    end
    assign refresh = refreshing;            // Refresh send when RankExecutionUnit is IDLE.  
                                                            // (TODO) : Refresh needs to be availble whenever the flag is set on.
    //////////////////////////////////////////////////////////////////////////


    //------------------------------------------------------------------------------
    //  Request Count Export
    //
    //  - Counts outstanding read/write requests in queues.
    //  - Used by ChannelScheduler for arbitration decisions.
    //
    //------------------------------------------------------------------------------
    always_comb begin
        chSchedReadReqCnt = 0;
        chSchedWriteReqCnt = 0;
        for(int i = 0; i < READCMDQUEUEDEPTH; i++) begin
            if(!ReadReqQueFree[i]) begin
                chSchedReadReqCnt = chSchedReadReqCnt + 1;
            end
        end
        for (int i = 0; i < WRITECMDQUEUEDEPTH; i++) begin
            if(!WriteReqQueFree[i]) begin
                chSchedWriteReqCnt = chSchedWriteReqCnt + 1;
            end
        end
    end
    //////////////////////////////////////////////////////////////////////////

    //------------------ Memory Request Queue Setup ------------------------//
    //               Set up Push Pointer based on LSB priority              //
    PriorityEncoder_LSB #(
        .vector_length(READCMDQUEUEDEPTH),
        .index_length($clog2(READCMDQUEUEDEPTH))
    ) push_ptr(     
        .vector(ReadReqQueFree),
        .index(ReadPushPtr)
    );

    PriorityEncoder_LSB #(
        .vector_length(WRITECMDQUEUEDEPTH),
        .index_length($clog2(WRITECMDQUEUEDEPTH))
    ) write_push_ptr( 
        .vector(WriteReqQueFree),
        .index(WritePushPtr)
    );



    //------------------------------------------------------------------------------
    //  Request Queue Management
    //
    //  - Pushes incoming requests from MC frontend.
    //  - Pops selected request when RankExecutionUnit is idle.
    //  - Maintains free-slot bitmap and push/pop pointers.
    //  - Includes structural hazard assertions.
    //
    //------------------------------------------------------------------------------
    always_ff@(posedge clk or negedge rst) begin
        if(!rst)begin
             // Free Queue Setup, 1 for Free slot, 0 for Allocated slot
            ReadReqQueFree <= '1;             
            WriteReqQueFree <= '1;
            //  Read/Write Request Queue Initialization (without count)
            for(int i = 0; i < READCMDQUEUEDEPTH; i++) begin
                ReadRequestQueue[i].mem_addr <= 0;
                ReadRequestQueue[i].req_type <= 0;
                ReadRequestQueue[i].req_id <= 0;
                ReadRequestQueue[i].req_user <= 0;
            end
            for(int i = 0; i < WRITECMDQUEUEDEPTH; i++) begin
                WriteRequestQueue[i].mem_addr <= 0;
                WriteRequestQueue[i].req_type <= 0;
                WriteRequestQueue[i].req_id <= 0;
                WriteRequestQueue[i].req_user <= 0;
            end
        end else begin
            //          Push Phase: Pushing Req. from MC FrontEnd           //
            if(RankReqValid && !ReadReqQueFull && (RankReqType == 0)) begin
                // For Read Request Case
                ReadRequestQueue[ReadPushPtr].mem_addr <= RankReqMemAddr;
                ReadRequestQueue[ReadPushPtr].req_type <= RankReqType;
                ReadRequestQueue[ReadPushPtr].req_id <= RankReqId;
                ReadRequestQueue[ReadPushPtr].req_user <= RankReqUser;
                ReadReqQueFree[ReadPushPtr] <= 0;
            end
            if(RankReqValid && !WriteReqQueFull && (RankReqType == 1)) begin
                // For Write Request Case
                WriteRequestQueue[WritePushPtr].mem_addr <= RankReqMemAddr;
                WriteRequestQueue[WritePushPtr].req_type <= RankReqType;
                WriteRequestQueue[WritePushPtr].req_id <= RankReqId;
                WriteRequestQueue[WritePushPtr].req_user <= RankReqUser;
                WriteReqQueFree[WritePushPtr] <= 0;
            end
            //     POP Phase: Poping Req. Request Que. and send to RankFSM   //
            if(fsmIdle[{ReadRequestQueue[ReadPopPtr].mem_addr.bankgroup, ReadRequestQueue[ReadPopPtr].mem_addr.bank}]
                 && !ReadReqQueFree[ReadPopPtr] && !WriteMode && !refreshing && !chSchedTransReady && SchedValid_3) begin 
                // For Read Request Case (Initialization for Request Queue)
                ReadRequestQueue[ReadPopPtr].mem_addr <= 0; 
                ReadRequestQueue[ReadPopPtr].req_type <= 0;
                ReadRequestQueue[ReadPopPtr].req_id <= 0;
                ReadRequestQueue[ReadPopPtr].req_user <= 0;
                ReadReqQueFree[ReadPopPtr] <= 1;
            end 
            if(fsmIdle[{WriteRequestQueue[WritePopPtr].mem_addr.bankgroup, WriteRequestQueue[WritePopPtr].mem_addr.bank}]
                && !WriteReqQueFree[WritePopPtr] && WriteMode && !refreshing && !chSchedTransReady && SchedValid_3) begin
                // For Write Request Case (Initialization for Request Queue)
                WriteRequestQueue[WritePopPtr].mem_addr <= 0; 
                WriteRequestQueue[WritePopPtr].req_type <= 0;
                WriteRequestQueue[WritePopPtr].req_id <= 0;
                WriteRequestQueue[WritePopPtr].req_user <= 0;
                WriteReqQueFree[WritePopPtr] <= 1;
            end
            ReadPopPtrOverlap : assert (
                !((RankReqValid  &&
                (ReadPushPtr == ReadPopPtr))&&
                !WriteMode &&
                !ReadReqQueFree[ReadPopPtr])
            ) else
                $fatal(2, "[%0t][ASSERT][RankSched][READ] Push/Pop overlap", $time);


            WritePopPtrOverlap : assert (
                !(RankReqValid &&
                (WritePushPtr == WritePopPtr) &&
                WriteMode &&
                !WriteReqQueFree[WritePopPtr])
            ) else
                $fatal(2, "[%0t][ASSERT][RankSched][WRITE] Push/Pop overlap", $time);
        end
    end

    assign ReadReqQueFull = !(|ReadReqQueFree);
    assign ReadReqQueEmpty = (&ReadReqQueFree);
    assign WriteReqQueFull = !(|WriteReqQueFree);
    assign WriteReqQueEmpty = (&WriteReqQueFree);

    assign RankReadReqReady =  !ReadReqQueFull && rdBufAvailable;
    assign RankWriteReqReady = !WriteReqQueFull && wrBufAvailable;


    //------------------------------------------------------------------------------
    //  Request Aging Tracker
    //
    //  - Increments per-request age counters.
    //  - Resets age when request is issued to RankExecutionUnit.
    //  - Enables starvation prevention via threshold logic.
    //
    //------------------------------------------------------------------------------
        always_ff@(posedge clk or negedge rst)begin
        // Request Count Setup for aging
        if(!rst) begin
            for(int i = 0; i < READCMDQUEUEDEPTH; i++) begin
                ReadRequestQueue[i].cnt <= 0;
            end
            for (int i = 0; i< WRITECMDQUEUEDEPTH; i++) begin
                WriteRequestQueue[i].cnt <= 0;
            end
        end else begin
            for(int i = 0; i < READCMDQUEUEDEPTH; i++)begin
                if(!ReadReqQueFree[i]) begin
                    ReadRequestQueue[i].cnt <= ReadRequestQueue[i].cnt + 1;
                end
            end
            for(int i = 0; i < WRITECMDQUEUEDEPTH; i++) begin
                if(!WriteReqQueFree[i]) begin
                    WriteRequestQueue[i].cnt <= WriteRequestQueue[i].cnt + 1; 
                end
            end
            if(fsmIdle[{ReadRequestQueue[ReadPopPtr].mem_addr.bankgroup, ReadRequestQueue[ReadPopPtr].mem_addr.bank}] 
            && !ReadReqQueFree[ReadPopPtr] && !WriteMode && !refreshing && !chSchedTransReady && SchedValid_3) begin // Condition for Sending Req. to RankFSM
                ReadRequestQueue[ReadPopPtr].cnt <= 0;
            end
            if(fsmIdle[{WriteRequestQueue[WritePopPtr].mem_addr.bankgroup, WriteRequestQueue[WritePopPtr].mem_addr.bank}] 
            && !WriteReqQueFree[WritePopPtr] && WriteMode && !refreshing && !chSchedTransReady && SchedValid_3) begin // Condition for Sending Req. to RankFSM
                WriteRequestQueue[WritePopPtr].cnt <= 0;
            end

        end
    end
    //////////////////////////////////////////////////////////////////////////
    

    //------------------------------------------------------------------------------
    //  RankExecutionUnit Issue Logic & Open-Page Update
    //
    //  - Issues selected request to RankExecutionUnit.
    //  - Updates OpenRowList on ACT / Auto-Precharge.
    //  - Clears open-page state on refresh completion.
    //
    //------------------------------------------------------------------------------
    always_ff@(posedge clk or negedge rst) begin
        if(!rst) begin
            fsmIssuedReq <= '0;                 //  Request to RankExecutionUnit
            for(int j = 0; j < NUM_BANKFSM; j++) begin
                fsmIssue[j] <= 0;               //  Valid when issuing REQUEST to RankExecutionUnit
            end
            for(int i = 0; i < OPENPAGELISTDEPTH; i++)begin
                OpenRowList[i] <= '0;           // Initialization of OpenRowList
            end  
        end else begin 
            if(fsmRefreshAck) begin             // Initialization of OpenRowList, when Refresh ACK comes on.
                                                // - Refresh is executed after precharging all pages, so we need to initialize OpenRowList.
                for(int i = 0; i < OPENPAGELISTDEPTH; i++) begin
                    OpenRowList[i] <= '0;
                end
            end
            if (!refreshing && !chSchedTransReady) begin      // Sending Request to RankExecutionUnit based on the Condition.
                if(fsmIdle[{ReadRequestQueue[ReadPopPtr].mem_addr.bankgroup, ReadRequestQueue[ReadPopPtr].mem_addr.bank}] &&
                !WriteMode && !ReadReqQueFree[ReadPopPtr] && SchedValid_3) begin 
                    fsmIssuedReq.mem_addr <= ReadRequestQueue[ReadPopPtr].mem_addr;
                    fsmIssuedReq.req_type <= ReadRequestQueue[ReadPopPtr].req_type;
                    {fsmIssuedReq.PageHit, fsmIssuedReq.PageMiss, fsmIssuedReq.PageEmpty} <= {PageHitT, PageMissT, PageEmptyT};
                    fsmIssuedReq.req_user <= ReadRequestQueue[ReadPopPtr].req_user;
                    fsmIssuedReq.req_id <= ReadRequestQueue[ReadPopPtr].req_id;
                    fsmIssue[{ReadRequestQueue[ReadPopPtr].mem_addr.bankgroup, ReadRequestQueue[ReadPopPtr].mem_addr.bank}] <= 1;

                    //           For AutoPrecharge Instruction, we invalidate AutoPrecharge Page in OpenPageList.            //
                    if(!checkAutoPrecharge) begin
                        //      If the Request is not for AutoPrecharge, then we just allocate that request in OpenPageList     //
                        OpenRowList[{ReadRequestQueue[ReadPopPtr].mem_addr.bankgroup, ReadRequestQueue[ReadPopPtr].mem_addr.bank}].valid <= 1;
                        OpenRowList[{ReadRequestQueue[ReadPopPtr].mem_addr.bankgroup, ReadRequestQueue[ReadPopPtr].mem_addr.bank}].RowAddr <=
                            ReadRequestQueue[ReadPopPtr].mem_addr.row;
                        fsmIssuedReq.AutoPreCharge <= 0;
                        `ifdef DISPLAY
                            $display("[%0t] RankSched | READ REQUEST SERVING | PAGEHit: %b | PageMiss: %b | PageEmpty: %b", $time, PageHitT, PageMissT, PageEmptyT );
                        `endif
                    end else begin
                        OpenRowList[{ReadRequestQueue[ReadPopPtr].mem_addr.bankgroup, ReadRequestQueue[ReadPopPtr].mem_addr.bank}].valid <= 0;
                        fsmIssuedReq.AutoPreCharge <= 1;
                        `ifdef DISPLAY
                            $display("[%0t] RankSched | AP-READ REQUEST SERVING | PAGEHit: %b | PageMiss: %b | PageEmpty: %b", $time, PageHitT, PageMissT, PageEmptyT );
                        `endif                        
                    end
                end else if(fsmIdle[{WriteRequestQueue[WritePopPtr].mem_addr.bankgroup, WriteRequestQueue[WritePopPtr].mem_addr.bank}] && 
                    WriteMode && !WriteReqQueFree[WritePopPtr] && SchedValid_3) begin
                    // Write Request Serving
                    fsmIssuedReq.mem_addr <= WriteRequestQueue[WritePopPtr].mem_addr;
                    fsmIssuedReq.req_type <= WriteRequestQueue[WritePopPtr].req_type;
                    {fsmIssuedReq.PageHit, fsmIssuedReq.PageMiss, fsmIssuedReq.PageEmpty} <= {PageHitT, PageMissT, PageEmptyT};
                    fsmIssuedReq.req_user <= WriteRequestQueue[WritePopPtr].req_user;
                    fsmIssuedReq.req_id <= WriteRequestQueue[WritePopPtr].req_id;
                    fsmIssue[{WriteRequestQueue[WritePopPtr].mem_addr.bankgroup, WriteRequestQueue[WritePopPtr].mem_addr.bank}] <= 1;
                    //           For AutoPrecharge Instruction, we invalidate AutoPrecharge Page in OpenPageList.            //
                    if(!checkAutoPrecharge) begin
                        OpenRowList[{WriteRequestQueue[WritePopPtr].mem_addr.bankgroup, WriteRequestQueue[WritePopPtr].mem_addr.bank}].valid <= 1;
                        OpenRowList[{WriteRequestQueue[WritePopPtr].mem_addr.bankgroup, WriteRequestQueue[WritePopPtr].mem_addr.bank}].RowAddr <=
                            WriteRequestQueue[WritePopPtr].mem_addr.row;
                        fsmIssuedReq.AutoPreCharge <= 0;
                        `ifdef DISPLAY
                            $display("[%0t] RankSched | WRITE REQUEST SERVING | PAGEHit: %b | PageMiss: %b | PageEmpty: %b", $time, PageHitT, PageMissT, PageEmptyT );
                        `endif
                    end else begin
                        OpenRowList[{WriteRequestQueue[WritePopPtr].mem_addr.bankgroup, WriteRequestQueue[WritePopPtr].mem_addr.bank}].valid <= 0;
                        fsmIssuedReq.AutoPreCharge <= 1;
                        `ifdef DISPLAY
                            $display("[%0t] RankSched | AP-WRITE REQUEST SERVING | PAGEHit: %b | PageMiss: %b | PageEmpty: %b", $time, PageHitT, PageMissT, PageEmptyT );
                        `endif
                    end
                end else begin
                    for(int i = 0; i < NUM_BANKFSM; i++) begin
                        fsmIssue[i] <= 0;
                    end
                end
            end else begin
                for(int i = 0; i < NUM_BANKFSM; i++) begin
                    fsmIssue[i] <= 0;
                end
            end
        end
    end
    /////////////////////////////////////////////////////////////////////////

    logic issuableCheck, readIssuable, writeIssuable;
    assign issuableCheck =  (!refreshing && !chSchedTransReady);
    assign readIssuable = fsmIdle[{ReadRequestQueue[ReadPopPtr].mem_addr.bankgroup, ReadRequestQueue[ReadPopPtr].mem_addr.bank}] &&
                !WriteMode && !ReadReqQueFree[ReadPopPtr] && SchedValid_3;

    assign writeIssuable = fsmIdle[{WriteRequestQueue[WritePopPtr].mem_addr.bankgroup, WriteRequestQueue[WritePopPtr].mem_addr.bank}] && 
                    WriteMode && !WriteReqQueFree[WritePopPtr] && SchedValid_3;
                

    assign issuable = (WriteMode) ? writeIssuable : readIssuable;


    //------------------------------------------------------------------------------
    //  Open-Page Policy Helpers
    //
    //  - Detect row-hit candidates for short / long tCCD cases.
    //  - Separates same-bankgroup vs cross-bankgroup conditions.
    //  - Used by FR-FCFS candidate selection logic.
    //
    //------------------------------------------------------------------------------

    function automatic logic ReadOpenPagePolicyShort(input logic [$clog2(CMDQUEUEDEPTH) -1 : 0] i);
        //////////////////////        Condition for Open for Short tCCD               /////////////////////
        // 1) Row Hit : Row of the target Request in Request Queue is SAME with the row in OpenPageList. //
        // 2) Request Valid : Request in Request Queue is valid for sending to RankExecutionUnit         //
        // 3) Same BG with Prior Req. : BankGroup (BG) of target Req is SAME with BG with Prior Req.     //
        ///////////////////////////////////////////////////////////////////////////////////////////////////
        if(ReadRequestQueue[i].mem_addr.row == OpenRowList[{ReadRequestQueue[i].mem_addr.bankgroup, ReadRequestQueue[i].mem_addr.bank}].RowAddr
        && OpenRowList[{ReadRequestQueue[i].mem_addr.bankgroup, ReadRequestQueue[i].mem_addr.bank}].valid && !ReadReqQueFree[i] && fsmIssuedReq.mem_addr.bankgroup == ReadRequestQueue[i].mem_addr.bankgroup) begin
            return 1'b1;
        end else  begin
            return 1'b0;
        end
    endfunction

        //////////////////////        Condition for Open for Short tCCD               /////////////////////
        // 1) Row Hit : Row of the target Request in Request Queue is SAME with the row in OpenPageList. //
        // 2) Request Valid : Request in Request Queue is valid for sending to RankExecutionUnit         //
        // 3) Diff BG with Prior Req. : BankGroup (BG) of target Req is NOT SAME with BG with Prior Req. //
        ///////////////////////////////////////////////////////////////////////////////////////////////////
    function automatic logic ReadOpenPagePolicyLong(input logic [$clog2(CMDQUEUEDEPTH)-1 : 0] i);
        if((ReadRequestQueue[i].mem_addr.row == OpenRowList[{ReadRequestQueue[i].mem_addr.bankgroup, ReadRequestQueue[i].mem_addr.bank}].RowAddr)
        && (OpenRowList[{ReadRequestQueue[i].mem_addr.bankgroup, ReadRequestQueue[i].mem_addr.bank}].valid) && !ReadReqQueFree[i] &&
         fsmIssuedReq.mem_addr.bankgroup != ReadRequestQueue[i].mem_addr.bankgroup) begin
            return  1'b1;
        end else  begin
            return  1'b0;
        end
    endfunction 

    //                    Write Request OpenPage Scheduling                 //
    function automatic logic WriteOpenPagePolicyShort(input logic [$clog2(CMDQUEUEDEPTH) -1 : 0] i);
        if(WriteRequestQueue[i].mem_addr.row == OpenRowList[{WriteRequestQueue[i].mem_addr.bankgroup, WriteRequestQueue[i].mem_addr.bank}].RowAddr
        && OpenRowList[{WriteRequestQueue[i].mem_addr.bankgroup, WriteRequestQueue[i].mem_addr.bank}].valid && !WriteReqQueFree[i] && 
        fsmIssuedReq.mem_addr.bankgroup == WriteRequestQueue[i].mem_addr.bankgroup) begin
            return 1'b1;
        end else  begin
            return 1'b0;
        end
    endfunction

    function automatic logic WriteOpenPagePolicyLong(input logic [$clog2(CMDQUEUEDEPTH)-1 : 0] i);
        if((WriteRequestQueue[i].mem_addr.row == OpenRowList[{WriteRequestQueue[i].mem_addr.bankgroup, WriteRequestQueue[i].mem_addr.bank}].RowAddr)
        && (OpenRowList[{WriteRequestQueue[i].mem_addr.bankgroup, WriteRequestQueue[i].mem_addr.bank}].valid) && !WriteReqQueFree[i] &&
         fsmIssuedReq.mem_addr.bankgroup != WriteRequestQueue[i].mem_addr.bankgroup) begin
            return 1'b1;
        end else  begin
            return 1'b0;
        end
    endfunction 
    //////////////////////////////////////////////////////////////////////////

    //------------------------------------------------------------------------------
    //  FR-FCFS Candidate Detection
    //
    //  - Identifies:
    //      * Short tCCD row-hits
    //      * Long tCCD row-hits
    //      * Oldest request (aging-based)
    //  - Separates read and write scheduling paths.
    //
    //------------------------------------------------------------------------------
    //        Candidate 1: Row Hit with Short-tCCD    PRIORITY: 2           //
    always_comb begin  
        NextPageHitS = 0;
        PageHitIndexS = '0;
        if(!WriteMode) begin
            // read request serving
            for(int i = READCMDQUEUEDEPTH-1; i>=0; i--)begin 
                if(ReadOpenPagePolicyShort(i) &&
                !fsmWait[{ReadRequestQueue[i].mem_addr.bankgroup, ReadRequestQueue[i].mem_addr.bank}] &&
                 fsmIdle[{ReadRequestQueue[i].mem_addr.bankgroup, ReadRequestQueue[i].mem_addr.bank}]
                 )begin
                    NextPageHitS = 1;
                    PageHitIndexS = i;
                end
            end
        end else begin
            // write request serving
            for(int i = WRITECMDQUEUEDEPTH-1; i>=0; i--) begin
                if(WriteOpenPagePolicyShort(i) &&
                !fsmWait[{WriteRequestQueue[i].mem_addr.bankgroup, WriteRequestQueue[i].mem_addr.bank}] &&
                 fsmIdle[{WriteRequestQueue[i].mem_addr.bankgroup, WriteRequestQueue[i].mem_addr.bank}]
                 ) begin
                    NextPageHitS = 1;
                    PageHitIndexS = i;
                end
            end
        end
    end
    //        Candidate 2: Row Hit with Long-tCCD     PRIORITY: 3         //
    always_comb begin 
        NextPageHitL = 0;
        PageHitIndexL = 0;
        if(!WriteMode) begin
            for(int i = READCMDQUEUEDEPTH-1; i>= 0; i--) begin
                if(ReadOpenPagePolicyLong(i) && 
                !fsmWait[{ReadRequestQueue[i].mem_addr.bankgroup, ReadRequestQueue[i].mem_addr.bank}] &&
                 fsmIdle[{ReadRequestQueue[i].mem_addr.bankgroup, ReadRequestQueue[i].mem_addr.bank}]
                 ) begin
                    NextPageHitL = 1;
                    PageHitIndexL = i[$clog2(READCMDQUEUEDEPTH)-1:0];
                end
            end
        end else begin
            for(int i = WRITECMDQUEUEDEPTH-1; i>=0; i--) begin
                if(WriteOpenPagePolicyLong(i) &&
                !fsmWait[{WriteRequestQueue[i].mem_addr.bankgroup, WriteRequestQueue[i].mem_addr.bank}] &&
                 fsmIdle[{WriteRequestQueue[i].mem_addr.bankgroup, WriteRequestQueue[i].mem_addr.bank}]
                 )begin
                    NextPageHitL = 1;
                    PageHitIndexL = i[$clog2(WRITECMDQUEUEDEPTH)-1:0];;
                end
            end
        end
    end

    //   Candidate 3: Under NO PAGEHIT, Oldest Request   PRIORITY: 4 (1)  //
    logic SchedValid_1;
    assign SchedValid_1 = (!(&ReadReqQueFree) && !WriteMode) || (!(&WriteReqQueFree) && WriteMode);

    always_comb begin 
        MaxValue = 0;
        if(!WriteMode)begin
            // Read Request serving
            for(int i =0; i < READCMDQUEUEDEPTH; i++) begin
                if(!ReadReqQueFree[i] && (ReadRequestQueue[i].cnt > ReadRequestQueue[MaxValue].cnt) &&
                 !fsmWait[{ReadRequestQueue[i].mem_addr.bankgroup, ReadRequestQueue[i].mem_addr.bank}] &&
                 fsmIdle[{ReadRequestQueue[i].mem_addr.bankgroup, ReadRequestQueue[i].mem_addr.bank}]
                 )begin
                    MaxValue = i[$clog2(CMDQUEUEDEPTH)-1:0];
                end else begin
                    MaxValue = MaxValue;
                end
            end
        end else begin
            for(int i = 0; i < WRITECMDQUEUEDEPTH; i++) begin
                if(!WriteReqQueFree[i] && (WriteRequestQueue[i].cnt > WriteRequestQueue[MaxValue].cnt) &&
                 !fsmWait[{WriteRequestQueue[i].mem_addr.bankgroup, WriteRequestQueue[i].mem_addr.bank}] &&
                 fsmIdle[{WriteRequestQueue[i].mem_addr.bankgroup, WriteRequestQueue[i].mem_addr.bank}]
                ) begin
                    MaxValue = i[$clog2(CMDQUEUEDEPTH)-1:0];
                end else begin
                    MaxValue = MaxValue;
                end
            end
        end
    end
    //  Checking Threshold in Aging, If THRESHOLD, PRIORITY chnaged to be 1   //
    always_comb begin 
        checkThreshold = 0;
        if(!WriteMode) begin
            // Read Request Serving
            for(int i = 0; i < READCMDQUEUEDEPTH; i++)begin
                if(!ReadReqQueFree[i] && (ReadRequestQueue[i].cnt > THRESHOLD) && 
                 !fsmWait[{ReadRequestQueue[i].mem_addr.bankgroup, ReadRequestQueue[i].mem_addr.bank}] &&
                 fsmIdle[{ReadRequestQueue[i].mem_addr.bankgroup, ReadRequestQueue[i].mem_addr.bank}]) begin
                    checkThreshold = 1;
                end
            end
        end else begin
            // Write Request Serving
            for(int i= 0; i < WRITECMDQUEUEDEPTH; i++) begin
                if(!WriteReqQueFree[i] && (WriteRequestQueue[i].cnt > THRESHOLD) && 
                !fsmWait[{WriteRequestQueue[i].mem_addr.bankgroup, WriteRequestQueue[i].mem_addr.bank}] &&
                fsmIdle[{WriteRequestQueue[i].mem_addr.bankgroup, WriteRequestQueue[i].mem_addr.bank}]) begin
                    checkThreshold = 1;
                end
            end
        end
    end

    //------------------------------------------------------------------------------
    //  FR-FCFS Final Selection (2-stage Pipeline )
    //      Stage 1. Calculating candidates
    //      Stage 2. Update Read/Write Pop Pointer (Candidate selection)
    //
    //  Priority Order:
    //      1) Requests beyond aging threshold
    //      2) Row-hit with short tCCD
    //      3) Row-hit with long tCCD
    //      4) Oldest request
    //
    //  - Determines final pop pointer.
    //  - Classifies PageHit / PageMiss / PageEmpty.
    //  - Decides auto-precharge behavior.
    //
    //------------------------------------------------------------------------------



    always_ff@(posedge clk or negedge rst) begin : SchedulingStage1
        if (!rst) begin
            MaxValue_reg             <= 0;
            checkThreshold_reg       <= 0;
            NextPageHitS_reg         <= 0;
            NextPageHitL_reg         <= 0;
            SchedValid_2             <= 0;
        end else begin
            MaxValue_reg         <= MaxValue;
            checkThreshold_reg   <= checkThreshold;
            NextPageHitS_reg     <= NextPageHitS;
            NextPageHitL_reg     <= NextPageHitL;
            SchedValid_2         <= SchedValid_1;
            PageHitIndexS_reg    <= PageHitIndexS;
            PageHitIndexL_reg    <= PageHitIndexL;
        end
    end : SchedulingStage1

    
    always_ff@(posedge clk or negedge rst) begin : SchedulingStage2
        if (!rst) begin
            ReadPopPtr   <= 0;
            WritePopPtr  <= 0;
            SchedValid_3 <= 0;
        end else begin
            if(!WriteMode) begin
                if(checkThreshold_reg) begin
                    ReadPopPtr   <= MaxValue_reg;
                    SchedValid_3 <= SchedValid_2;
                    if(ReadOpenPagePolicyShort(MaxValue_reg)) begin
                        {PageHitT, PageMissT, PageEmptyT} <= 4'b1000;
                    end else if(ReadOpenPagePolicyLong(MaxValue_reg)) begin
                        {PageHitT, PageMissT, PageEmptyT} <= 4'b1100;
                    end else if(OpenRowList[{ReadRequestQueue[MaxValue_reg].mem_addr.bankgroup, 
                                    ReadRequestQueue[MaxValue_reg].mem_addr.bank}].valid) begin
                        {PageHitT, PageMissT, PageEmptyT} <= 4'b0010;
                        checkAutoPrecharge                <= 1;
                    end else begin
                        {PageHitT, PageMissT, PageEmptyT} <= 4'b0001;
                    end
                end else if(NextPageHitS_reg) begin
                    ReadPopPtr                        <= PageHitIndexS_reg;
                    {PageHitT, PageMissT, PageEmptyT} <= 4'b1000;
                    SchedValid_3                      <= SchedValid_2;
                end else if(NextPageHitS_reg) begin
                    ReadPopPtr <= PageHitIndexL_reg;
                    {PageHitT, PageMissT, PageEmptyT} <= 4'b1100;
                    SchedValid_3 <= SchedValid_2;
                end else begin
                    ReadPopPtr <= MaxValue_reg;
                    SchedValid_3 <= SchedValid_2;
                    if(OpenRowList[{ReadRequestQueue[MaxValue_reg].mem_addr.bankgroup, 
                                ReadRequestQueue[MaxValue_reg].mem_addr.bank}].valid) begin
                        {PageHitT, PageMissT, PageEmptyT} <=  4'b0010;
                    end else begin
                        {PageHitT, PageMissT, PageEmptyT} <=  4'b0001;
                    end
                end
            end else begin
                if(checkThreshold_reg) begin
                    WritePopPtr  <= MaxValue_reg;
                    SchedValid_3 <= SchedValid_2;
                    if(WriteOpenPagePolicyShort(MaxValue_reg)) begin
                        {PageHitT, PageMissT, PageEmptyT} <=  4'b1000;
                    end else if(WriteOpenPagePolicyLong(MaxValue_reg)) begin
                        {PageHitT, PageMissT, PageEmptyT} <= 4'b1100;
                    end else if(OpenRowList[{WriteRequestQueue[MaxValue_reg].mem_addr.bankgroup, 
                            WriteRequestQueue[MaxValue_reg].mem_addr.bank}].valid) begin
                        checkAutoPrecharge <= 1;
                    end else begin
                    {PageHitT, PageMissT, PageEmptyT} <= 4'b0001;
                    end
                end else if(NextPageHitS_reg) begin
                    SchedValid_3 <= SchedValid_2;
                    WritePopPtr <= PageHitIndexS_reg;
                    {PageHitT, PageMissT, PageEmptyT} <= 4'b1000;
                end else if(NextPageHitL_reg) begin
                    SchedValid_3 <= SchedValid_2;
                    WritePopPtr <= PageHitIndexL_reg;
                    {PageHitT, PageMissT, PageEmptyT} <= 4'b1100;
                end else begin
                    SchedValid_3 <= SchedValid_2;
                    WritePopPtr <= MaxValue_reg;
                    if(OpenRowList[{WriteRequestQueue[MaxValue_reg].mem_addr.bankgroup, 
                        WriteRequestQueue[MaxValue_reg].mem_addr.bank}].valid) begin
                        {PageHitT, PageMissT, PageEmptyT} <=  4'b0010;
                    end else begin
                        {PageHitT, PageMissT, PageEmptyT} <=  4'b0001;
                    end
                end
            end
        end
    end : SchedulingStage2


//  ----------------------------------------------------------------------------------- //
    
endmodule
