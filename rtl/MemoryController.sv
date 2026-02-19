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
`ifndef VERILATOR
import MemoryController_Definitions::*;
`endif
import MemoryController_Definitions::*;

module MemoryController#(
    parameter int AXI_DATAWIDTH = 64,
    parameter int AXI_ADDRWIDTH = 32,
    parameter int AXI_IDWIDTH = 4,
    parameter int AXI_USERWIDTH = 1,
    parameter int MEM_ADDRWIDTH = 32,
    parameter int MEM_DATAWIDTH = 64,
    parameter int MEM_IDWIDTH = 4,
    parameter int MEM_USERWIDTH = 1,
    parameter int NUM_BANKFSM = 16,
    parameter int NUM_BANKFSM_BIT = 4,

    parameter int COMMAND_WIDTH = 18,
    parameter int RWIDTH = 15,
    parameter int CWIDTH = 10,
    parameter int BGWIDTH = 2,
    parameter int BKWIDTH = 2,
    parameter int RKWIDTH = 2,
    parameter int CHWIDTH = 1,
    parameter int NUM_FSM = 8,
    parameter int PHYFIFODEPTH = 32,
    parameter int READCMDQUEUEDEPTH  = 8,
    parameter int WRITECMDQUEUEDEPTH = 8,
    parameter int OPENPAGELISTDEPTH  = 16,
    parameter int NUMCHANNEL = 2,
    parameter int NUMRANK = 4,
    parameter int NUMBANKGROUP = 4,
    parameter int NUMBANK = 4,
    parameter int READBUFFERDEPTH  = 128,
    parameter int WRITEBUFFERDEPTH = 128,
    parameter int RESPSCHEDULINGCNT = 4,
    parameter int CHMODETHRESHOLD = 16,
    parameter int THRESHOLD = 512,
    parameter int AGINGWIDTH = 10,

    parameter int ASSEMBLER_DEPTH = 8,
    parameter int PHYFIFOMAXENTRY = 4,
    parameter int PHYFIFOREQUESTWINDOW = 8,
    parameter int BURST_LENGTH = 8,

    parameter int tBL   = 4,
    parameter int tCCDS = 4,
    parameter int tCCDL = 6,
    parameter int tRTRS = 2,
    parameter int tCL   = 16,
    parameter int tRCD  = 16,
    parameter int tRP   = 16,
    parameter int tCWL  = 12,
    parameter int tRTW  = 8,
    parameter int tRAS  = 39,
    parameter int tRC   = 55,
    parameter int tRTP  = 9,
    parameter int tWTRS = 3,
    parameter int tWTRL = 9,
    parameter int tWR   = 18,
    parameter int tRFC  = 256,
    parameter int tREFI = 8192
)(
    // common 
    input logic clk, rst_n, clk2x,

    // Cache-side
    input cache_side_request cache_req,
    output cache_side_response cache_resp
    `ifndef VERILATOR

        ,DDR4Interface DDR4_CH0_IF,
        DDR4Interface DDR4_CH1_IF
    `endif
    );

    `ifndef VERILATOR
    import MemoryController_Definitions::*;
    `endif

    `ifdef VERILATOR
    // DRAM-side
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



    // Memory Controller-side
    mc_side_request mc_req;
    mc_side_response mc_resp;

    //------------------------------------------------------------------------------
    //      Memory Controller Frontend
    //
    //      - Translates cache-side (AXI-like) requests into internal MC requests.
    //      - Performs Write Request assembly for AXI-AW channel and AXI-W channel.
    //      - Ensures a single in-flight request semantics at controller entry.
    //------------------------------------------------------------------------------
    MemoryControllerFrontend #(
        .AXI_ADDRWIDTH(AXI_ADDRWIDTH), .AXI_USERWIDTH(AXI_USERWIDTH),
        .AXI_IDWIDTH(AXI_IDWIDTH), .AXI_DATAWIDTH(AXI_DATAWIDTH),
        .MEM_ADDRWIDTH(MEM_ADDRWIDTH),
        .CHWIDTH(CHWIDTH), .RKWIDTH(RKWIDTH),
        .NUM_FSM(NUM_FSM), .BURST_LENGTH(BURST_LENGTH),
        .ASSEMBLER_DEPTH(ASSEMBLER_DEPTH),
        .WrAddrEntry(WrAddrEntry), .axi_aw_chan_t(axi_aw_chan_t),
        .CacheResp(cache_side_response), .CacheReq(cache_side_request),
        .MCResp(mc_side_response), .MCReq(mc_side_request),
        .MemoryAddress(mem_addr_t)
    ) MemoryControllerFrontEnd_Instance(
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

    logic [$clog2(READBUFFERDEPTH)-1:0] Ch0_NumOfReadBufferEntry, Ch1_NumOfReadBufferEntry;

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
    mc_side_request ch0_MCReq, ch1_MCReq;
    mc_side_response ch0_MCResp, ch1_MCResp;
    /* verilator lint_on UNUSEDSIGNAL */

    //------------------------------------------------------------------------------
    //      Channel Dispatch Logic
    //
    //      - Routes MC request to channel backend based on channel address bit.
    //      - No arbitration is required since frontend issues only one request at a time.
    //              (Response can come concurrently from each backends)
    //------------------------------------------------------------------------------

    assign ch0_MCReq.mem_addr       = (mc_req.mem_addr.channel == 0)  ? mc_req.mem_addr   : 0;
    assign ch0_MCReq.mem_id         = (mc_req.mem_addr.channel == 0)  ? mc_req.mem_id     : 0;
    assign ch0_MCReq.mem_user       = (mc_req.mem_addr.channel == 0)  ? mc_req.mem_user   : 0;
    assign ch0_MCReq.addr           = (mc_req.mem_addr.channel == 0)  ? mc_req.addr       : 0;
    assign ch0_MCReq.write_data     = (mc_req.mem_addr.channel == 0)  ? mc_req.write_data : 0;
    assign ch0_MCReq.write_strb     = (mc_req.mem_addr.channel == 0)  ? mc_req.write_strb : 0;
    assign ch0_MCReq.last           = (mc_req.mem_addr.channel == 0)  ? mc_req.last       : 0;

    assign ch0_MCReq.req_valid      = (mc_req.mem_addr.channel == 0) ? mc_req.req_valid      : 0;
    assign ch0_MCReq.req_data_valid = (mc_req.mem_addr.channel == 0) ? mc_req.req_data_valid : 0;
    assign ch0_MCReq.write          = (mc_req.mem_addr.channel == 0) ? mc_req.write          : 0;

    assign ch0_MCReq.readReady      = mc_req.readReady;
    assign ch0_MCReq.AckReady       = mc_req.AckReady;

    assign ch1_MCReq.mem_addr       = (mc_req.mem_addr.channel == 1) ? mc_req.mem_addr  : 0;
    assign ch1_MCReq.mem_id         = (mc_req.mem_addr.channel == 1) ? mc_req.mem_id    : 0;
    assign ch1_MCReq.mem_user       = (mc_req.mem_addr.channel == 1) ? mc_req.mem_user  : 0;
    assign ch1_MCReq.addr           = (mc_req.mem_addr.channel == 1) ? mc_req.addr      : 0;

    assign ch1_MCReq.write_data     = (mc_req.mem_addr.channel == 1) ? mc_req.write_data : 0;
    assign ch1_MCReq.write_strb     = (mc_req.mem_addr.channel == 1) ? mc_req.write_strb : 0;
    assign ch1_MCReq.last           = (mc_req.mem_addr.channel == 1) ? mc_req.last : 0;

    assign ch1_MCReq.req_valid      = (mc_req.mem_addr.channel == 1) ? mc_req.req_valid : 0;
    assign ch1_MCReq.req_data_valid = (mc_req.mem_addr.channel == 1) ? mc_req.req_data_valid : 0;
    assign ch1_MCReq.write          = (mc_req.mem_addr.channel == 1) ? mc_req.write          : 0;

    assign ch1_MCReq.readReady      =  mc_req.readReady; 
    assign ch1_MCReq.AckReady       =  mc_req.AckReady;


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
                        MCRespState <= SERVE_CH0;
                    end 
                end else if(!MCRespState) begin
                    if(ch0_MCResp.last)begin
                        SwitchWait <= 0;
                        MCRespState <= SERVE_CH1;
                    end
                end
             end
             else if(ServingCnt == RESPSCHEDULINGCNT -1) begin
                if(MCRespState && (Ch0_NumOfReadBufferEntry != 0)) begin
                    if(!ch1_MCResp.r_valid) begin
                        MCRespState <= SERVE_CH0;
                    end else begin
                        if(ch1_MCResp.r_valid && ch1_MCResp.last) begin
                            MCRespState <= SERVE_CH0;
                        end else begin
                            SwitchWait <= 1;
                        end
                    end
                end else if(!MCRespState && (Ch1_NumOfReadBufferEntry != 0)) begin
                    if(!ch0_MCResp.r_valid) begin
                        MCRespState <= SERVE_CH1;
                    end else begin
                        if(ch0_MCResp.r_valid && ch0_MCResp.last) begin
                            MCRespState <= SERVE_CH1;
                        end else begin
                            SwitchWait <= 1;
                        end
                    end
                end
             end else if(Ch0_NumOfReadBufferEntry > Ch1_NumOfReadBufferEntry) begin
                if(MCRespState) begin
                    if(ch1_MCResp.r_valid) begin
                        SwitchWait <= 1;
                        if(ch1_MCResp.r_valid && ch1_MCResp.last) begin
                            MCRespState <= SERVE_CH0;
                        end
                    end else begin
                        MCRespState <= SERVE_CH0;
                    end
                end
             end else if(Ch1_NumOfReadBufferEntry > Ch0_NumOfReadBufferEntry) begin
                if(!MCRespState) begin
                    if(ch0_MCResp.r_valid) begin
                        SwitchWait <= 1;
                        if(ch0_MCResp.r_valid && ch0_MCResp.last) begin
                            MCRespState <= SERVE_CH1;
                        end
                    end else begin
                        MCRespState <= SERVE_CH1;
                    end
                end
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
    MemoryControllerBackend #(
        .DEVICE_CHANNEL(0),
        .MEM_DATAWIDTH(MEM_DATAWIDTH), .MEM_IDWIDTH(MEM_IDWIDTH),
        .MEM_USERWIDTH(MEM_USERWIDTH), .MEM_ADDRWIDTH(MEM_ADDRWIDTH), 
        .NUM_BANKFSM(NUM_BANKFSM), .NUM_BANKFSM_BIT(NUM_BANKFSM_BIT),
        .CHMODETHRESHOLD(CHMODETHRESHOLD),
        .NUMRANK(NUMRANK), .NUMBANK(NUMBANK), .NUMBANKGROUP(NUMBANKGROUP),
        .BGWIDTH(BGWIDTH), .BKWIDTH(BKWIDTH), .RWIDTH(RWIDTH), .CWIDTH(CWIDTH),
        .COMMAND_WIDTH(COMMAND_WIDTH),
        .THRESHOLD(THRESHOLD), .AGINGWIDTH(AGINGWIDTH),
        .READCMDQUEUEDEPTH(READCMDQUEUEDEPTH), .WRITECMDQUEUEDEPTH(WRITECMDQUEUEDEPTH),
        .OPENPAGELISTDEPTH(OPENPAGELISTDEPTH),
        .BURST_LENGTH(BURST_LENGTH),
        .PHYFIFOREQUESTWINDOW(PHYFIFOREQUESTWINDOW),
        .PHYFIFOMAXENTRY(PHYFIFOMAXENTRY),
        .PHYFIFODEPTH(PHYFIFODEPTH),
        .READBUFFERDEPTH(READBUFFERDEPTH),
        .WRITEBUFFERDEPTH(WRITEBUFFERDEPTH),
        .tRP(tRP), .tWR(tWR), .tRFC(tRFC), .tRTRS(tRTRS), .tCCDL(tCCDL), .tRCD(tRCD),
        .tCCDS(tCCDS), .tRTW(tRTW), .tWTRS(tWTRS), .tWTRL(tWTRL), .tREFI(tREFI),
        .tCL(tCL), .tCWL(tCWL),
        .FSMRequest(FSMReq), .MemoryAddress(mem_addr_t), 
        .ReadBufferDataEntry(READBUFFERDATAENTRY), .ReadBufferDirEntry(ReadBufferDirEntry),
        .WriteBufferDataEntry(WRITEBUFFERDATAENTRY), .WriteBufferDirEntry(WriteBufferDirEntry)
    ) CH0_MemoryControllerBackend_Instance(
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
        .ddr4_dq_bus(DDR4_CH0_IF), 
        .ddr4_cmd_bus(DDR4_CH0_IF)
    );


    MemoryControllerBackend #(
        .DEVICE_CHANNEL(1),
        .MEM_DATAWIDTH(MEM_DATAWIDTH), .MEM_IDWIDTH(MEM_IDWIDTH),
        .MEM_USERWIDTH(MEM_USERWIDTH), .MEM_ADDRWIDTH(MEM_ADDRWIDTH),  
        .NUM_BANKFSM(NUM_BANKFSM), .NUM_BANKFSM_BIT(NUM_BANKFSM_BIT),
        .CHMODETHRESHOLD(CHMODETHRESHOLD),
        .NUMRANK(NUMRANK), .NUMBANK(NUMBANK), .NUMBANKGROUP(NUMBANKGROUP),
        .BGWIDTH(BGWIDTH), .BKWIDTH(BKWIDTH), .RWIDTH(RWIDTH), .CWIDTH(CWIDTH),
        .COMMAND_WIDTH(COMMAND_WIDTH),
        .THRESHOLD(THRESHOLD), .AGINGWIDTH(AGINGWIDTH),
        .READCMDQUEUEDEPTH(READCMDQUEUEDEPTH), .WRITECMDQUEUEDEPTH(WRITECMDQUEUEDEPTH),
        .OPENPAGELISTDEPTH(OPENPAGELISTDEPTH), .BURST_LENGTH(BURST_LENGTH),
        .PHYFIFOREQUESTWINDOW(PHYFIFOREQUESTWINDOW),
        .PHYFIFOMAXENTRY(PHYFIFOMAXENTRY),
        .PHYFIFODEPTH(PHYFIFODEPTH),
        .READBUFFERDEPTH(READBUFFERDEPTH),
        .WRITEBUFFERDEPTH(WRITEBUFFERDEPTH),
        .tRP(tRP), .tWR(tWR), .tRFC(tRFC), .tRTRS(tRTRS), .tCCDL(tCCDL), .tRCD(tRCD),
        .tCCDS(tCCDS), .tRTW(tRTW), .tWTRS(tWTRS), .tWTRL(tWTRL), .tREFI(tREFI),
        .tCL(tCL), .tCWL(tCWL),
        .FSMRequest(FSMReq), .MemoryAddress(mem_addr_t), 
        .ReadBufferDataEntry(READBUFFERDATAENTRY), .ReadBufferDirEntry(ReadBufferDirEntry),
        .WriteBufferDataEntry(WRITEBUFFERDATAENTRY), .WriteBufferDirEntry(WriteBufferDirEntry)
    ) CH1_MemoryControllerBackend_Instance(
        .clk(clk), .rst(rst_n), .clk2x(clk2x),
        //                          INPUT  FROM  Memory Controller Frontend              //
        .RankReqMemAddr(ch1_MCReq.mem_addr), 
        .RankReqId(ch1_MCReq.mem_id), .RankReqUser(ch1_MCReq.mem_user), .RankReqType(ch1_MCReq.write),
        .RankReqValid(|(ch1_MCReq.req_valid)), 
        .RankData(ch1_MCReq.write_data), .RankDataStrb(ch1_MCReq.write_strb), 
        .RankDataLast(ch1_MCReq.last), .RankDataValid(ch1_MCReq.req_data_valid),
        .CacheReadDataReady(ch1_MCReq.readReady && MCRespState), 
        .CacheWriteDataACKReady(ch1_MCReq.AckReady),
        //                          OUTPUT  TO Memory Controller Frontend                //
        .CacheReadData(ch1_MCResp.read_data), 
        .CacheReadDataUser(ch1_MCResp.mem_read_user), .CacheReadDataId(ch1_MCResp.mem_read_id),
        .CacheReadDataLast(ch1_MCResp.last), .CacheReadDataValid(ch1_MCResp.r_valid), 
        .CacheWriteDataACKValid(ch1_MCResp.b_valid), 
        .CacheWriteDataACKID(ch1_MCResp.mem_ack_id) , .CacheWriteDataACKUser(ch1_MCResp.mem_ack_user),
        .ReadBufferFull(Ch1_ReadBufferFull), .WriteBufferFull(Ch1_WriteBufferFull), 
        .RankReadReqReady(Ch1_RankFSMReadReady), .RankWriteReqReady(Ch1_RankFSMWriteReady),
        //                         OUTPUT TO Memory Controller                          //
        .NumOfReadBufferEntry(Ch1_NumOfReadBufferEntry),
        //                        DDR4 Interface                                        //
        .ddr4_dq_bus(DDR4_CH1_IF), 
        .ddr4_cmd_bus(DDR4_CH1_IF)
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
    
    assign mc_resp.mem_ack_id     =  ch1_MCResp.b_valid ? ch1_MCResp.mem_ack_id   : ch0_MCResp.b_valid ? ch0_MCResp.mem_ack_id : 0;
    assign mc_resp.mem_ack_user   =  ch1_MCResp.b_valid ? ch1_MCResp.mem_ack_user : ch0_MCResp.b_valid ? ch0_MCResp.mem_ack_user : 0;
    assign mc_resp.b_valid        =  ch1_MCResp.b_valid || ch0_MCResp.b_valid;
    

    assign mc_resp.ar_ready = { {4{~Ch1_ReadBufferFull}} & Ch1_RankFSMReadReady_r,   {4{~Ch0_ReadBufferFull}} & Ch0_RankFSMReadReady_r   };
    assign mc_resp.w_ready  = { {4{~Ch1_WriteBufferFull}} & Ch1_RankFSMWriteReady_r, {4{~Ch0_WriteBufferFull}} & Ch0_RankFSMWriteReady_r};
    assign mc_resp.aw_ready = { {4{~Ch1_WriteBufferFull}} & Ch1_RankFSMWriteReady_r, {4{~Ch0_WriteBufferFull}} & Ch0_RankFSMWriteReady_r};
endmodule
