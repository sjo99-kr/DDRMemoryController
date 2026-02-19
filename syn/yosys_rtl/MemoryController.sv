`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////////////
//      MemoryController
//
//      Role:
//          Top-level memory controller integrating frontend request handling,
//          channel-level arbitration, and per-channel backend DDR controllers.
//
//      Architecture Overview:
//
//          Cache-side
//              |
//              v
//      +---------------------------+
//      |  MemoryControllerFrontend |
//      |  - Simple-AXI protocol    |
//      |  - Request assembly       |
//      +---------------------------+
//              |
//              v
//      +---------------------------+
//      |   MemoryController (this) |
//      |   - Channel selection     |
//      |   - Response arbitration  |
//      +---------------------------+
//           |               |
//           v               v
//      +-----------+   +-----------+
//      | Backend 0 |   | Backend 1 |
//      | (CH0)     |   | (CH1)     |
//      +-----------+   +-----------+
//           |               |
//           v               v
//        DDR4 CH0        DDR4 CH1
//
//      Responsibilities:
//          1) Dispatch memory requests to the correct Channel Controller.
//          2) Perform channel-level response (i.e., Cache-side response) arbitration to avoid starvation.
//          3) Connect frontend AXI-like interface (i.e., simple AXI bus protocol) with DDR4 backends.
//          4) Maintain fairness across channels under asymmetric load.
//
//      Design Notes:
//          - Only one request is issued at a time from the frontend.
//          - Multiple channel responses may return concurrently.
//          - Response arbitration is queue-depth-aware with starvation avoidance (i.e., Aging scheme for starvation).
//
//      Author  : Seongwon Jo
//      Created : 2026.02
//////////////////////////////////////////////////////////////////////////////////////////
// `define DISPLAY
// `define VERILATOR

import MemoryController_Definitions::*;

module MemoryController(
    // common 
    input logic clk, rst_n, clk2x,

    // Cache-side
    input logic [157-1:0] cache_req,
    output logic [80-1:0] cache_resp,

    // DDR Interface Signal
    output logic ch0_cke,
    output logic [NUMRANK-1:0] ch0_cs_n,
    output logic ch0_par,
    output logic ch0_act_n,
    output logic [COMMAND_WIDTH-1:0] ch0_pin_A,
    output logic [BGWIDTH-1:0] ch0_bg,
    output logic [BKWIDTH-1:0] ch0_b,

    input logic [MEM_DATAWIDTH-1:0] ch0_pin_dq_in,
    output wire ch0_pin_dq_out,
    input logic ch0_dqs_c_in, 
    input logic ch0_dqs_t_in,
    output logic ch0_dqs_c_out, 
    output logic ch0_dqs_t_out,

    output logic ch1_cke,
    output logic [NUMRANK-1:0] ch1_cs_n,
    output logic ch1_par,
    output logic ch1_act_n,
    output logic [COMMAND_WIDTH-1:0] ch1_pin_A,
    output logic [BGWIDTH-1:0] ch1_bg,
    output logic [BKWIDTH-1:0] ch1_b,

    input logic [MEM_DATAWIDTH-1:0] ch1_pin_dq_in,
    output wire ch1_pin_dq_out,
    input logic ch1_dqs_c_in, 
    input logic ch1_dqs_t_in,
    output logic ch1_dqs_c_out,





    output logic ch1_dqs_t_out);



    // Memory Controller-side
    logic [157-1:0] mc_req;
    logic [80-1:0] mc_resp;

    //------------------------------------------------------------------------------
    //      Memory Controller Frontend
    //
    //      - Translates cache-side (AXI-like) requests into internal MC requests.
    //      - Performs Write Request assembly for AXI-AW channel and AXI-W channel.
    //      - Ensures a single in-flight request semantics at controller entry.
    //------------------------------------------------------------------------------
    MemoryControllerFrontend MemoryControllerFrontEnd_Instance(
        .clk(clk), .rst_n(rst_n),
        .noc_req(cache_req), .noc_resp(cache_resp),
        .mc_req(mc_req), .mc_resp(mc_resp)
    );

    logic Ch0_ReadBufferFull, Ch0_WriteBufferFull;
    logic Ch1_ReadBufferFull, Ch1_WriteBufferFull;

    logic [NUMRANK-1 : 0] Ch0_RankFSMReadReady, Ch1_RankFSMReadReady;
    logic [NUMRANK-1 : 0] Ch0_RankFSMReadReady_r, Ch1_RankFSMReadReady_r;
    logic [NUMRANK-1 : 0] Ch0_RankFSMWriteReady, Ch1_RankFSMWriteReady;
    logic [NUMRANK-1 : 0] Ch0_RankFSMWriteReady_r, Ch1_RankFSMWriteReady_r;

    logic [READBUFFERDEPTH-1:0] Ch0_NumOfReadBufferEntry, Ch1_NumOfReadBufferEntry;

    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            Ch0_RankFSMReadReady_r  <= '0;
            Ch1_RankFSMReadReady_r  <= '0;
            Ch0_RankFSMWriteReady_r <= '0;
            Ch1_RankFSMWriteReady_r <= '0;
        end else begin
            Ch0_RankFSMReadReady_r <= Ch0_RankFSMReadReady;
            Ch1_RankFSMReadReady_r <= Ch1_RankFSMReadReady;
            Ch0_RankFSMWriteReady_r <= Ch0_RankFSMWriteReady;
            Ch1_RankFSMWriteReady_r <= Ch1_RankFSMWriteReady;
        end
    end

    //          Memory Request Arbitration Based on Channel Address             //
    /* verilator lint_off UNUSEDSIGNAL */
    logic [147-1:0] ch0_MCReq, ch1_MCReq;
    logic [80-1:0] ch0_MCResp, ch1_MCResp;
    /* verilator lint_on UNUSEDSIGNAL */

    //------------------------------------------------------------------------------
    //      Channel Dispatch Logic
    //
    //      - Routes MC request to channel backend based on channel address bit.
    //      - No arbitration is required since frontend issues only one request at a time.
    //              (Response can come concurrently from each backends)
    //------------------------------------------------------------------------------

    assign ch0_MCReq = (mc_req.mem_addr.channel == 0) ? mc_req : '0;
    assign ch1_MCReq = (mc_req.mem_addr.channel == 1) ? mc_req : '0;

    //      But Response need a arbitration Scheduling , because both response from channels comes together.
    //      (TODO) Current Arbitration Scheduling Method -> Queue-depth-aware fairness arbitration
    //              - For prevent starvation, we do count the serving response numbers to dominant channel.
    
    //  Arbitration state machine:
    //      - SERVE_CH0 / SERVE_CH1 indicates which channel is currently served.
    //      - SwitchWait ensures channel switching occurs only after burst completion.
    typedef enum logic {SERVE_CH0, SERVE_CH1} RespArbitrationState_t;
    RespArbitrationState_t MCRespState;
    logic  SwitchWait;
    logic [$clog2(RESPSCHEDULINGCNT) - 1:0] ServingCnt;

    //------------------------------------------------------------------------------
    //      MC Response Arbitration
    //
    //      Problem:
    //          - CH0 and CH1 backends may return read responses concurrently.
    //          - Frontend can consume only one response stream at a time.
    //
    //      Solution:
    //          - Queue-depth-aware arbitration between channel responses.
    //          - Dominant channel is periodically throttled to prevent starvation.
    //          - Channel switch is aligned with burst boundaries (last signal).
    //------------------------------------------------------------------------------
    always_ff@(posedge clk or negedge rst_n) begin : MCResponseArbitration
        if(!rst_n) begin
            MCRespState <= SERVE_CH0; 
            SwitchWait <= 0;
        end else begin
             if(SwitchWait) begin
                if(MCRespState) begin
                    if(ch1_MCResp.last) begin
                        SwitchWait <= 0;
                    end 
                end else if(!MCRespState) begin
                    if(ch0_MCResp.last)begin
                        SwitchWait <= 0;
                    end
                end
             end
             else if(ServingCnt == RESPSCHEDULINGCNT -1) begin
                if(MCRespState && (Ch0_NumOfReadBufferEntry != 0)) begin
                    MCRespState <= SERVE_CH0;
                    SwitchWait <= 1;
                end else if(!MCRespState && (Ch1_NumOfReadBufferEntry != 0)) begin
                    MCRespState <= SERVE_CH1;
                    SwitchWait <= 1;
                end
             end else if(Ch0_NumOfReadBufferEntry > Ch1_NumOfReadBufferEntry) begin
                MCRespState <= SERVE_CH0;
             end else if(Ch1_NumOfReadBufferEntry > Ch0_NumOfReadBufferEntry) begin
                MCRespState <= SERVE_CH1;
             end
        end
    end : MCResponseArbitration
    

    //------------------------------------------------------------------------------
    //      Starvation Avoidance Mechanism
    //
    //      - Counts consecutive responses served for the same channel.
    //      - Forces channel switch when SERVINGCNT threshold is reached,
    //          if the other channel has pending responses.
    //------------------------------------------------------------------------------
    always_ff@(posedge clk or negedge rst_n) begin : AvoidingRespStarvation
        if(!rst_n) begin
            ServingCnt <= 0;
        end else begin
            if(MCRespState) begin : ResponseForCH1
                if(Ch0_NumOfReadBufferEntry > Ch1_NumOfReadBufferEntry) begin 
                    ServingCnt <= 0;
                end else begin
                    if(ch1_MCResp.last)begin
                        ServingCnt <= ServingCnt + 1;
                    end 
                end
            end : ResponseForCH1 
            else if(!MCRespState) begin : ResponseForCH0
                if(Ch1_NumOfReadBufferEntry > Ch0_NumOfReadBufferEntry) begin
                    ServingCnt <= 0;
                end else begin
                    if(ch0_MCResp.last)begin
                        ServingCnt <= ServingCnt + 1;
                    end
                end
            end : ResponseForCH0
        end
    end : AvoidingRespStarvation

    //------------------------------------------------------------------------------
    //      Memory Controller Backend (Per-Channel)
    //
    //      - Handles rank/bank-level scheduling and DDR timing.
    //      - Owns read/write data buffers and PHY FIFO.
    //------------------------------------------------------------------------------


    MemoryControllerBackend CH0_MemoryControllerBackend_Instance(
        .clk(clk), .rst(rst_n), .clk2x(clk2x),
        //                          INPUT  FROM  Memory Controller Frontend              //
        .RankReqMemAddr(ch0_MCReq.mem_addr), 
        .RankReqId(ch0_MCReq.mem_id), .RankReqUser(ch0_MCReq.mem_user), .RankReqType(ch0_MCReq.write),
        .RankReqValid(|(ch0_MCReq.req_valid)), 
        .RankData(ch0_MCReq.write_data), .RankDataStrb(ch0_MCReq.write_strb), 
        .RankDataLast(ch0_MCReq.last), .RankDataValid(ch0_MCReq.req_data_valid),
        .CacheReadDataReady(ch0_MCReq.readReady && !MCRespState), .CacheWriteDataACKReady(ch0_MCReq.AckReady),
        //                          OUTPUT  TO Memory Controller Frontend                //
        .CacheReadData(ch0_MCResp.read_data), 
        .CacheReadDataUser(ch0_MCResp.mem_read_user), .CacheReadDataId(ch0_MCResp.mem_read_id),
        .CacheReadDataLast(ch0_MCResp.last), .CacheReadDataValid(ch0_MCResp.r_valid), 
        .ReadBufferFull(Ch0_ReadBufferFull),
        .CacheWriteDataACKValid(ch0_MCResp.b_valid), 
        .CacheWriteDataACKID(ch0_MCResp.mem_ack_id) , .CacheWriteDataACKUser(ch0_MCResp.mem_ack_user),
        .WriteBufferFull(Ch0_WriteBufferFull), 
        .RankReadReqReady(Ch0_RankFSMReadReady), .RankWriteReqReady(Ch0_RankFSMWriteReady),
        //                         OUTPUT TO Memory Controller                          //
        .NumOfReadBufferEntry(Ch0_NumOfReadBufferEntry),
        //                               DDR4 Interface                                 //
        .pin_dq_in(ch0_ppin_dq_in), .pin_dq_out(ch0_pin_dq_out), 
        .dqs_c_in(ch0_dqs_c_in), .dqs_c_out(ch0_dqs_c_out),
        .dqs_t_in(ch0_dqs_t_in), .dqs_t_out(ch0_dqs_t_out),

        .cke(ch0_cke), .cs_n(ch0_cs_n), .par(ch0_par), .act_n(ch0_act_n),
        .pin_A(ch0_pin_A), .bg(ch0_bg), .b(ch0_b)
    );


    
    
    MemoryControllerBackend CH1_MemoryControllerBackend_Instance(
        .clk(clk), .rst(rst_n), .clk2x(clk2x),
        //                          INPUT  FROM  Memory Controller Frontend              //
        .RankReqMemAddr(ch1_MCReq.mem_addr), 
        .RankReqId(ch1_MCReq.mem_id), .RankReqUser(ch1_MCReq.mem_user), .RankReqType(ch1_MCReq.write),
        .RankReqValid(|(ch1_MCReq.req_valid)), 
        .RankData(ch1_MCReq.write_data), .RankDataStrb(ch1_MCReq.write_strb), 
        .RankDataLast(ch1_MCReq.last), .RankDataValid(ch1_MCReq.req_data_valid),
        .CacheReadDataReady(ch1_MCReq.readReady && !MCRespState), .CacheWriteDataACKReady(ch1_MCReq.AckReady),
        //                          OUTPUT  TO Memory Controller Frontend                //
        .CacheReadData(ch1_MCResp.read_data), 
        .CacheReadDataUser(ch1_MCResp.mem_read_user), .CacheReadDataId(ch1_MCResp.mem_read_id),
        .CacheReadDataLast(ch1_MCResp.last), .CacheReadDataValid(ch1_MCResp.r_valid), 
        .ReadBufferFull(Ch1_ReadBufferFull),
        .CacheWriteDataACKValid(ch1_MCResp.b_valid), 
        .CacheWriteDataACKID(ch1_MCResp.mem_ack_id) , .CacheWriteDataACKUser(ch1_MCResp.mem_ack_user),
        .WriteBufferFull(Ch1_WriteBufferFull), 
        .RankReadReqReady(Ch1_RankFSMReadReady), .RankWriteReqReady(Ch1_RankFSMWriteReady),
        //                         OUTPUT TO Memory Controller                          //
        .NumOfReadBufferEntry(Ch0_NumOfReadBufferEntry),
        //                               DDR4 Interface                                 //
        .pin_dq_in(ch1_pin_dq_in), .pin_dq_out(ch1_pin_dq_out), 
        .dqs_c_in(ch1_dqs_c_in), .dqs_c_out(ch1_dqs_c_out),
        .dqs_t_in(ch1_dqs_t_in), .dqs_t_out(ch1_dqs_t_out),

        .cke(ch1_cke), .cs_n(ch1_cs_n), .par(ch1_par), .act_n(ch1_act_n),
        .pin_A(ch1_pin_A), .bg(ch1_bg), .b(ch1_b)
    );




    //------------------------------------------------------------------------------
    //      Final Response Multiplexing
    //
    //          - Selected channel response is forwarded to frontend.
    //          - Controlled by MCRespState FSM.
    //------------------------------------------------------------------------------
    assign mc_resp.read_data      =  MCRespState ? ch1_MCResp.read_data : ch0_MCResp.read_data;
    assign mc_resp.mem_read_id    =  MCRespState ? ch1_MCResp.mem_read_id : ch0_MCResp.mem_read_id;
    assign mc_resp.mem_read_user  =  MCRespState ? ch1_MCResp.mem_read_user : ch0_MCResp.mem_read_user;
    assign mc_resp.last           =  MCRespState ? ch1_MCResp.last : ch0_MCResp.last;
    assign mc_resp.r_valid        =  MCRespState ? ch1_MCResp.r_valid : ch0_MCResp.r_valid;
    
    assign mc_resp.mem_ack_id     =  MCRespState ? ch1_MCResp.mem_ack_id : ch0_MCResp.mem_ack_id;
    assign mc_resp.mem_ack_user   =  MCRespState ? ch1_MCResp.mem_ack_user : ch0_MCResp.mem_ack_user;
    assign mc_resp.b_valid        =  MCRespState ? ch1_MCResp.b_valid : ch0_MCResp.b_valid;
    

    assign mc_resp.ar_ready = { {4{~Ch1_ReadBufferFull}} & Ch1_RankFSMReadReady_r,   {4{~Ch0_ReadBufferFull}} & Ch0_RankFSMReadReady_r   };
    assign mc_resp.w_ready  = { {4{~Ch1_WriteBufferFull}} & Ch1_RankFSMWriteReady_r, {4{~Ch0_WriteBufferFull}} & Ch0_RankFSMWriteReady_r};
    assign mc_resp.aw_ready = { {4{~Ch1_WriteBufferFull}} & Ch1_RankFSMWriteReady_r, {4{~Ch0_WriteBufferFull}} & Ch0_RankFSMWriteReady_r};
endmodule
