`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////////////////////
//
//      MemoryControllerFrontend
//
//      Role:
//          Frontend of the Memory Controller.
//          Bridges a simplified AXI-like cache interface and the internal
//               Memory Controller request/response protocol.
//
//      Architectural Overview:
//
//            L2 Cache / NoC (AXI-like)
//                |          ∧    
//                V          |
//          +----------------------------+
//          |  MemoryControllerFrontend  |
//          |        (This module)       |
//          +----------------------------+
//                |          ∧
//                V          |
//          MemoryController (Channel-level Backend)
//
//      Responsibilities:
//          1) Accept cache-side read/write requests using ready/valid semantics.
//          2) Perform write burst assembly by matching AW and W channels
//                  using (ID, USER) pairs.
//          3) Translate physical addresses into internal memory addresses
//             (CH/RK/BG/BK/ROW/COL).
//          4) Perform deterministic request arbitration:
//              - Read-first policy
//              - Write-preemption when a full write burst is assembled.
//          5) Generate internal MC requests toward rank/channel FSMs.
//          6) Forward read/write responses back to cache-side interface.
//
//      Arbitration Policy:
//          - Read-first scheduling by default.
//          - Write requests are issued only when a complete write burst
//            (ADDR + DATA) is assembled.
//          - No simultaneous read/write request issuance.
//
//      Address Translation:
//          - Fixed-index address mapping:
//                Channel | Rank | BankGroup | Bank | Row | Column
//                  (Address mapping can be extend for Hash(XOR)-based (TODO))
//          - Translation is applied only on request path.
//
//      Design Notes:
//          - This module is protocol-focused and timing-agnostic.
//          - No DRAM timing constraints are enforced here.
//          - All DDR timing and scheduling are delegated to Backend logic.
//          - Simplified AXI semantics are assumed (no reordering, no interleaving).
//
//      Author  : Seongwon Jo
//      Created : 2026.02
//
//////////////////////////////////////////////////////////////////////////////////////////////////

import MemoryController_Definitions::*;

module MemoryControllerFrontend
   (
        input logic clk, 
        input logic rst_n,
                                                            //////////////////////////////////////////////
                                                            //         INPUT  FROM  Cache/MC-side       //
        input logic [157-1:0] noc_req,                             //  1. Cache-side Request (Cache-side)      //
        input logic [80-1:0] mc_resp,                               //  2. MC-side  Response  (MC-side)         //

                                                            //////////////////////////////////////////////
                                                            //         OUTPUT  TO Cache/MC-side         //
        output [80-1:0] noc_resp,                          //  1. Cache-side Response (Cache-side)     //
        output [157-1:0] mc_req                                //  2. MC-side  Request (MC-side)           //
    );

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //  I assume that the cache request/response interface is based on a simplified AXI protocol, including       //
    //  ready/valid handshaking and only the necessary signals: data, last, strobe (strb), address, user, and ID. //
    //  1) AXI - Address Read Channel                                                                             //
    //      - Ready     (Signal from MC-side)     ->  MC Resp, Cache Resp                                         //
    //      - Valid     (Singal from Cache-side)  ->  MC Req,  Cache Req                                          //
    //      - Address   (Signal from Cache-side)  ->  MC Req,  Cache Req                                          //
    //      - User & ID (Signal from Cache-side)  ->  MC Req,  Cache Req                                          //
    //  2) AXI - Read Channel                                                                                     //
    //      - Ready     (Signal from Cache-side)  ->  MC Req,  Cache Req                                          //
    //      - Valid     (Signal from MC-side)     ->  MC Resp, Cache Resp                                         //
    //      - Data      (Signal from MC-side)     ->  MC Resp, Cache Resp                                         //
    //      - LAST      (Signal from MC-side)     ->  MC Resp, Cache Resp                                         //
    //      - User & ID (Signal from MC-side)     ->  MC Resp, Cache Resp                                         //
    //  3) AXI - Address Write Channel                                                                            //
    //      - Ready     (Signal from MC-side)     ->  MC Resp, Cache Resp                                         //
    //      - Valid     (Singal from Cache-side)  ->  MC Req,  Cache Req                                          //
    //      - Address   (Signal from Cache-side)  ->  MC Req,  Cache Req                                          //
    //      - User & ID (Singal from Cache-sdie)  ->  MC Req,  Cache Req                                          //
    //  4) AXI - Write Channel                                                                                    //
    //      - Ready     (Signal from MC-side)     ->  MC Resp, Caache Resp                                        //
    //      - Valid     (Signal from Cache-side)  ->  MC Req,  Cache Req                                          //
    //      - Data      (Signal from Cache-side)  ->  MC Req,  Cache Req                                          //
    //      - LAST      (Signal from Cache-side)  ->  MC Req,  Cache Req                                          //
    //      - Strb      (Signal from Cache-side)  ->  MC Req,  Cache Req                                          //
    //      - User & ID (Signal from Cache-side)  ->  MC Req,  Cache Req                                          //
    //  5) AXI- Write Response Channel                                                                            //
    //      - Ready     (Signal from Cache-side)  ->  MC Req,  Cache Req                                          //
    //      - Valid     (Siganl from MC-side)     ->  MC Resp, Cache Resp                                         //
    //      - User & ID (Signal from MC-side)     ->  MC Resp, Cache Resp                                         //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    localparam WR_BEAT_WIDTH = AXI_DATAWIDTH + AXI_DATAWIDTH/BURST_LENGTH;      // WR_BEAT_WIDTH = 64-bit (DATA) + 8-bit (Byte Strobe)
    localparam BURSTTIMING = BURST_LENGTH - 1;                              

    //              Write Assembler-Address Queue Entry Definition             //


    logic [77-1:0] WrAddrQueue [0:ASSEMBLER_DEPTH-1];

    //              Write Assembler-Data Queue Entry Definition             //
    logic [AXI_IDWIDTH + AXI_USERWIDTH + WR_BEAT_WIDTH * BURST_LENGTH - 1 : 0] WrDataQueue [0: ASSEMBLER_DEPTH-1]; 
    logic [ASSEMBLER_DEPTH - 1 : 0] WrDataFree, WrAddrFree, WrPushPtrFree; 
    logic [ASSEMBLER_DEPTH - 1 : 0]  assemblyVector;  

    logic [$clog2(ASSEMBLER_DEPTH) - 1 :0] WrDataPushPtr, WrAddrPushPtr;

    logic WrDataQueueFull, WrAddrQueueFull;
    logic [$clog2(BURST_LENGTH) - 1:0]  WrPushCnt; 

    logic [$clog2(ASSEMBLER_DEPTH)-1:0] assembleWriteAddrIndex; 
    logic [$clog2(ASSEMBLER_DEPTH)-1:0] assembleWriteDataIndex;
    logic [$clog2(BURST_LENGTH) - 1:0] WrPopCnt; 

    //      Memory Controller Frontend reflects read-first priority          //
    logic arbitrationMode;

    // Internal signal  (MC_read Ready & MC_write Ready)
    logic  ReadRequestReceived;  // read ready signal from read buffer & queue
    logic  WriteAddrReceived, WriteDataReceived; // write ready signal from write buffer



   

    //------------------------------------------------------------------------------
    //      AddressTranslationUnit
    //
    //      - Translates AXI physical addresses into internal MemoryAddress format.
    //      - Decodes Channel / Rank / BankGroup / Bank / Row / Column fields.
    //      - Selects target Rank FSM(s) for incoming requests.
    //
    //  NOTE:
    //      - Translation is applied only on request path.
    //      - Current mapping uses fixed bit slicing.
    //------------------------------------------------------------------------------
    logic [32-1:0] translatedAddr;
    logic [NUM_FSM-1:0] FSM_vector;
    logic [NUM_FSM_BIT-1:0] FSM_index;

    AddressTranslationUnit AddressTranslationUnit_Instance(
        .readAddr(noc_req.ar.addr),
        .writeAddr(noc_req.aw.addr),
        .readReady(mc_resp.ar_ready),
        .readValid(noc_req.ar_valid),
        .writeReady(mc_resp.aw_ready),
        .writeValid(noc_req.aw_valid),
        .targetFSMVector(FSM_vector),
        .targetFSMIndex(FSM_index),
        .requestMemAddr(translatedAddr)
    );
    //-------------------------------------------------------------------//
    assign ReadRequestReceived  = noc_req.ar_valid    && noc_resp.ar_ready  &&  |(FSM_vector[FSM_index]);   
    assign WriteDataReceived    = noc_req.w_valid     && noc_resp.w_ready;
    assign WriteAddrReceived    = noc_req.aw_valid    && noc_resp.aw_ready  && |(FSM_vector[FSM_index]);
    assign arbitrationMode      = |assemblyVector;

    //----------------------- WR, WR_ADDR PTR - SETUP -----------------------//
    PriorityEncoder_LSB #(
        .vector_length(ASSEMBLER_DEPTH)
    ) WrDataQueuePushPtr(
        .vector(WrPushPtrFree),
        .index(WrDataPushPtr)
    );

    PriorityEncoder_LSB #(
        .vector_length(ASSEMBLER_DEPTH)
    ) WrAddrQueuePushPtr(
        .vector(WrAddrFree),
        .index(WrAddrPushPtr)
    );
    //-------------------------------------------------------------------//

    assign WrDataQueueFull = !(|WrDataFree);
    assign WrAddrQueueFull = !(|WrAddrFree);



    //     Assembly Vecotr Mangeemnt  && Write Data/Addr Qeueu Ptr Setup        //
    //  Assembly Vector Valid Condition :                                       //
    //    - Is there any same ID and User in Write Data and ADDR Queue??        //
    //   (TODO) : Solve Nested "for loop"                                       //
    //      Option 1. Utilize Fixed User and ID Bits. (Hash)                    //
    //          - WrAddrEntry[{User-bits, ID-bits}] = New_writeAddr, VALID bit  //


    //------------------------------------------------------------------------------
    //      Write Request Assembler (ADDR–DATA Matching)
    //
    //      - Matches AXI Write Address (AW) and Write Data (W) streams.
    //      - Uses (ID, USER) pair to associate address and burst data.
    //      - Generates assemblyVector when a full write burst is assembled.
    //
    //  NOTE:
    //      - Write requests are issued only when both ADDR and full DATA are ready.
    //      - Nested search is acceptable due to limited ASSEMBLER_DEPTH.
    //------------------------------------------------------------------------------
    always_comb begin
        assembleWriteAddrIndex  = '0;
        assembleWriteDataIndex  = '0;
        assemblyVector          = '0;
        for (int i = ASSEMBLER_DEPTH-1; i >= 0; i--) begin
            for(int j = ASSEMBLER_DEPTH-1; j >=0; j--) begin
                if (!WrDataFree[i] && !WrAddrFree[j] && 
                        (WrDataQueue[i][AXI_IDWIDTH + AXI_USERWIDTH + WR_BEAT_WIDTH * BURST_LENGTH - 1 : WR_BEAT_WIDTH * BURST_LENGTH]
                        == {WrAddrQueue[j].aw.id, WrAddrQueue[j].aw.user})) begin
                    assembleWriteDataIndex = i;
                    assembleWriteAddrIndex = j;
                    assemblyVector[i] = 1;    
                end
            end
        end
    end
    //-------------------------------------------------------------------//


    //------------------------------------------------------------------------------
    //      Write Address & Data Queue Management
    //
    //      - Buffers incoming AXI AW and W transactions independently.
    //      - Collects write data beats into burst-aligned entries.
    //      - Tracks free/occupied queue entries.
    //      - Releases entries once a write burst is issued to backend.
    //
    //  NOTE:
    //      - Partial write bursts are never forwarded.
    //      - Address and data queues are strictly decoupled.
    //------------------------------------------------------------------------------
    always_ff@(posedge clk or negedge rst_n) begin 
        if(!rst_n) begin  
            for(int i = 0; i< ASSEMBLER_DEPTH; i++)begin
                WrDataQueue[i] <= 0;
                WrAddrQueue[i] <= 0;
            end
            WrDataFree      <= '1;
            WrPushPtrFree   <= '1;
            WrAddrFree      <= '1;
            WrPushCnt <= 0;
        end else begin  
            //              Write address QUEUE Push                     //
            if(WriteAddrReceived) begin 
                WrAddrQueue[WrAddrPushPtr][77-1:45] <= translatedAddr;
                WrAddrQueue[WrAddrPushPtr][44:8] <= noc_req.aw;
                WrAddrQueue[WrAddrPushPtr][7:0] <= FSM_vector;

                WrAddrFree[WrAddrPushPtr] <= 0;
                $display("[MC Frontend] CLK: %d | AXI-Write Addr Bus | Addr: %d | ID: %d | User: %d", $time, noc_req.aw.addr, noc_req.aw.id,
                                                                            noc_req.aw.user);
            end
            // write-data setup
            if(WriteDataReceived) begin
                WrDataQueue[WrDataPushPtr][WrPushCnt * WR_BEAT_WIDTH +: WR_BEAT_WIDTH] <= {noc_req.w.strb, noc_req.w.data};
                $display("[MC Frontend] CLK: %d | AXI-Write data Bus | Data: %d | ID: %d | User: %d", $time, WrPushCnt, noc_req.w.data,
                                noc_req.w.id, noc_req.w.user);

                if(WrPushCnt == 0 )begin
                    WrDataQueue[WrDataPushPtr][AXI_IDWIDTH+AXI_USERWIDTH+WR_BEAT_WIDTH * BURST_LENGTH - 1 : WR_BEAT_WIDTH* BURST_LENGTH] <= {noc_req.w.id, noc_req.w.user};
                end 
                if (WrPushCnt == BURSTTIMING)begin
                    WrDataFree[WrDataPushPtr] <= 0;
                    WrPushPtrFree[WrDataPushPtr] <= 0;
                end
                if(WrPushCnt == BURSTTIMING) begin
                    WrPushCnt <= 0;
                end else begin
                    WrPushCnt <= WrPushCnt + 1;
                end
            end 
            // free setup 
            if(arbitrationMode && WrPopCnt == BURSTTIMING) begin 
                WrAddrFree[assembleWriteAddrIndex] <= 1;
                WrDataFree[assembleWriteDataIndex] <= 1;
                if(!WriteDataReceived) begin
                    WrPushPtrFree[assembleWriteDataIndex] <= 1;
                end else begin
                    WrPushPtrFree <= WrDataFree;
                end
                $display("[MC Frontend] %d | Write DATA Issued TO Write Buffer (ID=%d, USER=%d)",
                        $time, mc_req.mem_id, mc_req.mem_user);
            end 
        end
    end 
    


    //------------------------------------------------------------------------------
    //      Memory Controller Request Generation
    //
    //      - Converts frontend arbitration result into backend MC requests.
    //      - Issues either:
    //          * Single READ request, or
    //          * Burst-aligned WRITE request.
    //
    //  NOTE:
    //      - WRITE: req_valid asserted only at first beat.
    //      - READ : single-cycle request without pipelining.
    //------------------------------------------------------------------------------
    always_comb begin 
        // Write request for memory controller
        mc_req  = '0;
        mc_req[2]     = arbitrationMode;
        mc_req[1] = noc_req.r_ready;
        mc_req[0]  = noc_req.b_ready;

        if(arbitrationMode) begin 
            mc_req[0] = 1;
            mc_req[116:85]     = WrAddrQueue[assembleWriteAddrIndex][35:4]; 
            mc_req[153:121] = WrAddrQueue[assembleWriteAddrIndex][76:45];

            {mc_req.[121:118], mc_req[117]} = WrDataQueue[assembleWriteDataIndex][AXI_IDWIDTH+AXI_USERWIDTH+ (WR_BEAT_WIDTH * BURST_LENGTH)  - 1 : BURST_LENGTH * WR_BEAT_WIDTH];
            {mc_req[20:13], mc_req[84:21]} = WrDataQueue[assembleWriteDataIndex][WrPopCnt * WR_BEAT_WIDTH +: WR_BEAT_WIDTH];

            // Write request valid
            if(WrPopCnt == 0) begin
                mc_req[11:4] = WrAddrQueue[assembleWriteAddrIndex][44:37];
                if (mc_req[11:4]) begin
                    `ifdef ASSERTION_TURN_ON
                            $display("[MC Frontend] %d | Write Request Issued to FSM  (ID=%d, USER=%d)",
                                    $time, mc_req.mem_id, mc_req.mem_user);
                    `endif
                    end            
            end else mc_req[11:4] = 0;

            if(WrPopCnt == BURSTTIMING) begin
                mc_req[12] = 1;
            end else begin
                mc_req[12] = 0;
            end
        end 
        
        // Read request for memory controller
        else begin 
            if(ReadRequestReceived) begin
                mc_req[11:4] = (noc_req.ar_valid);
                mc_req[153:121] = translatedAddr;
                mc_req[121:118]   = noc_req.ar.id;
                mc_req[117] = noc_req.ar.user;
                mc_req[116:85]     = noc_req.ar.addr;

            end
        end 
    end 

    //------------------------------------------------------------------------------
    //      Write Burst Beat Counter
    //
    //      - Tracks current beat index within a write burst.
    //      - Controls data slicing and LAST signal generation.
    //
    //  NOTE:
    //      - Active only during WRITE arbitration phase.
    //------------------------------------------------------------------------------
    always_ff@(posedge clk or negedge rst_n)begin : WrPopCount
        if(!rst_n)begin
            WrPopCnt <= 0;
        end
        else begin
            if(arbitrationMode) begin
                WrPopCnt <= WrPopCnt + 1;
                if(WrPopCnt == BURSTTIMING)begin
                    WrPopCnt <= 0;
                end
            end else begin
                WrPopCnt <= 0;
            end
        end
    end : WrPopCount


    //------------------------------------------------------------------------------
    //      Cache / NoC Response Path
    //
    //      - Forwards backend READ and WRITE responses to cache-side interface.
    //      - Manages ready/valid handshaking for all AXI-like channels.
    //
    //  NOTE:
    //      - Read responses are stalled during write arbitration.
    //      - Backpressure is applied when assembler queues are full.
    //------------------------------------------------------------------------------
    always_comb begin : NoCResponseSetup
        noc_resp = '0;

        noc_resp.r.id = mc_resp.mem_read_id;
        noc_resp.r.user = mc_resp.mem_read_user;
        noc_resp.r.data = mc_resp.read_data;
        noc_resp.r.last = mc_resp.last;
        noc_resp.r_valid = mc_resp.r_valid;

        noc_resp.b.id = mc_resp.mem_ack_id;
        noc_resp.b.user = mc_resp.mem_ack_user;
        noc_resp.b_valid = mc_resp.b_valid;

        noc_resp.ar_ready =  (|mc_resp.ar_ready) && !arbitrationMode;
        noc_resp.aw_ready =  (|mc_resp.aw_ready) && !WrAddrQueueFull;
        noc_resp.w_ready  =  (|mc_resp.w_ready)  && !WrDataQueueFull;
    end : NoCResponseSetup




`ifdef  ASSERTION_TURN_ON
    CacheArbitrationOverlapping : assert property (@(posedge clk or negedge rst_n) disable iff(!rst_n)
        (noc_req.ar_valid |-> !(noc_req.aw_valid || noc_req.w_valid))) else
            $error("MC Frontend: Both RD and WR NoC Request comes together");
    
    WriteRequestArbitration : assert property (@(posedge clk or negedge rst_n) disable iff(!rst_n)
        (arbitrationMode |-> mc_req.write)) else 
            $error("MC Frontend: (Write Request) Arbitration Mode Error");
    ReadRequestArbitration : assert property (@(posedge clk or negedge rst_n) disable iff(!rst_n)
        (arbitrationMode |-> mc_req.write)) else 
            $error("MC Frontend: (Read Request) Arbitration Mode Error");
    
`endif


endmodule
