`timescale 1ns / 1ps

import svut_if::*;


module monitor (
    ///      Cache <-> Memory Controller        ///
    input logic clk, rst_n, clk2x,

    cache_side_request cache_req,
    cache_side_response cache_resp,

    ///     Memory Controller <-> DRAM          ///
    DDR4Interface DDR4_CH0_IF,
    DDR4Interface DDR4_CH1_IF,

    //       Monitor -> ScoreBoard   (Cache_Resp, Cache Req)      ///
    // Focus on 1) Cache request and response matching;
    //          2) Cache request and Cache reponse data/addr matching;
    //          3) Cache-Request and Response-related backpressure check  et.. c..
    output cache_readReq_Issue      cacheReadReqIssue,
    output cache_writeReq_Issue     cacheWriteReqIssue,
    output cache_readResp_Receive   cacheReadRespReceive,
    output cache_writeACK_Receive   cacheWriteACKReceive,

    //       Monitor -> ScoreBoard  (DDR4 CMD / DQ BUS)     ///
    //  Focus on 1) Timing violation for DDR Interface  
    //   etc..
    output dram_cmd_Issue dramCmdIssue [$clog2(NUMCHANNEL):0],
    output dram_data_Issue dramWriteDataIssue [$clog2(NUMCHANNEL):0],
    output dram_data_Receive dramReadDataReceive  [$clog2(NUMCHANNEL):0]
);

    localparam int AWREQWIDTH = 1 << AXI_IDWIDTH;

    //      ReadRequest Monitoring      //
    integer NumCacheReadReq   = 0;
    integer NumCacheWriteReq  = 0;
    integer NumCacheReadResp  = 0;
    integer NumCacheWriteResp = 0;

    assign cacheReadReqIssue.valid =  (cache_req.ar_valid && cache_resp.ar_ready);
    assign cacheReadReqIssue.addr  =  cache_req.ar.addr;
    assign cacheReadReqIssue.id    =  cache_req.ar.id;
    assign cacheReadReqIssue.user  =  cache_req.ar.user;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            NumCacheReadReq  <= 0;
            NumCacheWriteReq <= 0;
            NumCacheReadResp <= 0;
            
        end else begin
            if (cacheReadReqIssue.valid) begin
                NumCacheReadReq <= NumCacheReadReq + 1;
                $display("Number of Cache Read Request Issued: %0d", NumCacheReadReq + 1);
            end
            if(cacheWriteReqIssue.valid) begin
                NumCacheWriteReq <= NumCacheWriteReq + 1;
                $display("Number of Cache Write Request Issued: %d", NumCacheWriteReq + 1);
            end
            if(cacheReadRespReceive.valid) begin
                NumCacheReadResp <= NumCacheReadResp + 1;
                $display("Number of Cache Read Response Received: %d", NumCacheReadResp + 1);
            end
            if(cacheWriteACKReceive.valid) begin
                NumCacheWriteResp <= NumCacheWriteResp + 1;
                $display("Number of Cache Write Response Received: %d", NumCacheWriteResp + 1);
            end
        end
    end

    logic [AXI_DATAWIDTH-1:0] AwAddrQueue [AWREQWIDTH-1:0];
    logic [AXI_IDWIDTH-1:0]   AwIdQueue   [AWREQWIDTH-1:0];
    logic [AXI_USERWIDTH-1:0] AwUserQueue [AWREQWIDTH-1:0];
    logic AwReadyQueue [AWREQWIDTH-1:0];

    logic WriteReqFlag;
    logic [$clog2(AWREQWIDTH):0] AwQueueWrPtr, AwQueueRdPtr;

    logic writeReqCombine;

    logic [$clog2(4):0] readyPtr [1:0];

    always_comb begin
        AwQueueWrPtr = 0;
        for(int i = AWREQWIDTH-1; i >= 0; i--) begin
            if(AwReadyQueue[i]) begin
                AwQueueWrPtr = i;
            end
        end
    end

    always_comb begin
        AwQueueRdPtr = 0;
        if(cache_req.w_valid && cache_resp.w_ready) begin
            for(int i = 0; i <AWREQWIDTH; i++) begin
                if(cache_req.w.id  == AwIdQueue[i]) begin
                    if(cache_req.w.user == AwUserQueue[i]) begin
                        AwQueueRdPtr = i;
                    end
                end
            end
        end
    end

    always_ff@(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            for(int i = 0; i < AWREQWIDTH; i++) begin
                AwAddrQueue[i]  <= '0;
                AwIdQueue[i]    <= '0;
                AwUserQueue[i]  <= '0;
                AwReadyQueue[i] <= '1;
            end
        end else begin
            if(writeReqCombine) begin
            end else begin 
                if(cache_req.aw_valid && cache_resp.aw_ready) begin
                    AwReadyQueue[AwQueueWrPtr] <= 0;
                    AwUserQueue[AwQueueWrPtr]  <= cache_req.aw.user;
                    AwAddrQueue[AwQueueWrPtr]  <= cache_req.aw.addr;
                    AwIdQueue[AwQueueWrPtr]    <= cache_req.aw.id;
                end
                if(cache_req.w_valid && cache_resp.w_ready) begin
                    if(cache_req.w.last) begin
                        AwReadyQueue[AwQueueRdPtr] <= 1;
                    end
                end
            end
        end
    end

    assign writeReqCombine = cache_req.aw_valid && cache_req.w_valid && cache_resp.w_ready && cache_resp.aw_ready &&
                            (cache_req.aw.id == cache_req.w.id) && (cache_req.aw.user == cache_req.w.user);

    assign cacheWriteReqIssue.valid =  (writeReqCombine) ? 1 : (cache_req.w_valid && cache_resp.w_ready) ? cache_req.w.last  : 0;
    assign cacheWriteReqIssue.addr  =  (writeReqCombine) ? cache_req.aw.addr : (cache_req.w_valid && cache_resp.w_ready) ? AwAddrQueue[AwQueueRdPtr] : 0;
    assign cacheWriteReqIssue.id    =  (writeReqCombine) ? cache_req.aw.id   : (cache_req.w_valid && cache_resp.w_ready) ? AwIdQueue[AwQueueRdPtr]   : 0;
    assign cacheWriteReqIssue.user  =  (writeReqCombine) ? cache_req.aw.user : (cache_req.w_valid && cache_resp.w_ready) ? AwUserQueue[AwQueueRdPtr] : 0;
    

    ///         RESPONSE  FINDING       // 
    logic [AXI_DATAWIDTH * BURST_LENGTH-1:0] ReadRespDataQueue;
    logic [$clog2(BURST_LENGTH)-1:0] ReadRespDataCnt;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            ReadRespDataQueue <= '0;
            ReadRespDataCnt   <= '0;
        end else begin
            if(cache_resp.r_valid) begin
                if(ReadRespDataCnt == BURST_LENGTH-1) begin
                    ReadRespDataCnt <= 0;
                end else begin
                    ReadRespDataCnt <= ReadRespDataCnt + 1;
                end
                ReadRespDataQueue[ReadRespDataCnt * AXI_DATAWIDTH +: AXI_DATAWIDTH] <= cache_resp.r.data;
            end else begin
                ReadRespDataCnt <= 0;
            end
        end
    end
    
    assign cacheReadRespReceive.valid = (cache_resp.r_valid && cache_req.r_ready) ? ((cache_resp.r.last) ? cache_resp.r_valid :  0) : 0;
    assign cacheReadRespReceive.id    = (cache_resp.r_valid && cache_req.r_ready) ? ((cache_resp.r.last) ? cache_resp.r.id    :  0) : 0;
    assign cacheReadRespReceive.user  = (cache_resp.r_valid && cache_req.r_ready) ? ((cache_resp.r.last) ? cache_resp.r.user  :  0) : 0;
    assign cacheReadRespReceive.data  = (cache_resp.r_valid && cache_req.r_ready) ? ((cache_resp.r.last) ? ReadRespDataQueue  :  0) : 0;

    assign cacheWriteACKReceive.valid = (cache_resp.b_valid && cache_req.b_ready) ? 1 : 0;
    assign cacheWriteACKReceive.id    = (cache_resp.b_valid && cache_req.b_ready) ? cache_resp.b.id   : 0;
    assign cacheWriteACKReceive.user  = (cache_resp.b_valid && cache_req.b_ready) ? cache_resp.b.user : 0;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    commandType cmdType [$clog2(NUMCHANNEL):0];
    logic [$clog2(NUMRANK)-1:0] rk_ch0, rk_ch1;
    logic [1:0] validSig;

    always_comb begin
        rk_ch0 = 0;
        rk_ch1 = 0;
        for(int i = 0; i < NUMRANK; i++) begin
            if(DDR4_CH0_IF.cs_n[i] == 0) begin
                rk_ch0 = i;  
            end
            if(DDR4_CH1_IF.cs_n[i] == 0) begin
                rk_ch1 = i;
            end
        end
    end
    logic Ch0_rankselected, Ch1_rankSelected;

    assign Ch0_rankselected = !(&(DDR4_CH0_IF.cs_n));
    assign Ch1_rankSelected = !(&(DDR4_CH1_IF.cs_n));
    
    always_comb begin
        if(!Ch0_rankselected) begin
            cmdType[0] = IDLE;
        end else begin
            if(checkActivate(DDR4_CH0_IF.cke, DDR4_CH0_IF.act_n, DDR4_CH0_IF.pin_A)) begin
                cmdType[0] = ACTIVATE;
            end else if(checkAutoPrechargeRead(DDR4_CH0_IF.cke, DDR4_CH0_IF.act_n, DDR4_CH0_IF.pin_A)) begin
                cmdType[0] = AUTOREAD;
            end else if(checkRead(DDR4_CH0_IF.cke, DDR4_CH0_IF.act_n, DDR4_CH0_IF.pin_A)) begin
                cmdType[0] = READ;
            end else if(checkWrite(DDR4_CH0_IF.cke, DDR4_CH0_IF.act_n, DDR4_CH0_IF.pin_A)) begin
                cmdType[0] = WRITE;
            end else if(checkAutoPrechargeWrite(DDR4_CH0_IF.cke, DDR4_CH0_IF.act_n, DDR4_CH0_IF.pin_A)) begin
                cmdType[0] = AUTOWRITE;
            end else if(checkRefresh(DDR4_CH0_IF.cke, DDR4_CH0_IF.act_n, DDR4_CH0_IF.pin_A)) begin
                cmdType[0] = REFRESH;
            end else if(checkPrecharge(DDR4_CH0_IF.cke, DDR4_CH0_IF.act_n, DDR4_CH0_IF.pin_A)) begin
                cmdType[0] = PRECHARGE;
            end else begin
                cmdType[0] = IDLE;
            end
        end
    end 

    always_comb begin
        if(!Ch1_rankSelected) begin
            cmdType[1] = IDLE;
        end else begin
            if(checkActivate(DDR4_CH1_IF.cke, DDR4_CH1_IF.act_n, DDR4_CH1_IF.pin_A)) begin
                cmdType[1] = ACTIVATE;
            end else if(checkAutoPrechargeRead(DDR4_CH1_IF.cke, DDR4_CH1_IF.act_n, DDR4_CH1_IF.pin_A)) begin
                cmdType[1] = AUTOREAD;
            end else if(checkRead(DDR4_CH1_IF.cke, DDR4_CH1_IF.act_n, DDR4_CH1_IF.pin_A)) begin
                cmdType[1] = READ;
            end else if(checkWrite(DDR4_CH1_IF.cke, DDR4_CH1_IF.act_n, DDR4_CH1_IF.pin_A)) begin
                cmdType[1] = WRITE;
            end else if(checkAutoPrechargeWrite(DDR4_CH1_IF.cke, DDR4_CH1_IF.act_n, DDR4_CH1_IF.pin_A)) begin
                cmdType[1] = AUTOWRITE;
            end else if(checkRefresh(DDR4_CH1_IF.cke, DDR4_CH1_IF.act_n, DDR4_CH1_IF.pin_A)) begin
                cmdType[1] = REFRESH;
            end else if(checkPrecharge(DDR4_CH1_IF.cke, DDR4_CH1_IF.act_n, DDR4_CH1_IF.pin_A)) begin
                cmdType[1] = PRECHARGE;
            end else begin
                cmdType[1] = IDLE;
            end
        end
    end 



    assign dramCmdIssue[0].cmdType    = cmdType[0];
    assign dramCmdIssue[0].valid      = (cmdType[0] != IDLE) ?  ((Ch0_rankselected) ? 1 : 0) : 0;
    assign dramCmdIssue[0].bk         = DDR4_CH0_IF.b;
    assign dramCmdIssue[0].bg         = DDR4_CH0_IF.bg;
    assign dramCmdIssue[0].rank       = rk_ch0;
    assign dramCmdIssue[0].Issue_time = (cmdType[0] != IDLE) ? $time : 0;

    assign dramCmdIssue[1].cmdType    = cmdType[1];
    assign dramCmdIssue[1].valid      = (cmdType[1] != IDLE) ? ((Ch1_rankSelected) ? 1 : 0) : 0;
    assign dramCmdIssue[1].bk         = DDR4_CH1_IF.b;
    assign dramCmdIssue[1].bg         = DDR4_CH1_IF.bg;
    assign dramCmdIssue[1].rank       = rk_ch1;
    assign dramCmdIssue[1].Issue_time = (cmdType[1] != IDLE) ? $time : 0;


    logic [$clog2(BURST_LENGTH)-1:0] dataBusCnt [$clog2(NUMCHANNEL):0];
    
    always_ff@(posedge clk2x or negedge rst_n) begin
        if(!rst_n) begin
            dataBusCnt[0] <= 0;
            dataBusCnt[1] <= 0;
        end else begin
            if(DDR4_CH0_IF.dqs_t || DDR4_CH0_IF.dqs_c) begin
                if(dataBusCnt[0] == BURST_LENGTH-1) begin
                    dataBusCnt[0] <= 0;
                end  else begin
                    dataBusCnt[0] <= dataBusCnt[0] + 1; 
                end
            end if(DDR4_CH1_IF.dqs_t || DDR4_CH1_IF.dqs_c) begin
                if(dataBusCnt[1] == BURST_LENGTH-1) begin
                    dataBusCnt[1] <= 0;
                end else begin
                    dataBusCnt[1] <= dataBusCnt[1] + 1;
                end
            end 
        end
    end



    logic [4:0] CommandQueue[NUMCHANNEL -1:0];
    logic [4:0] CommandReady[NUMCHANNEL -1:0];

    always_ff@(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
                CommandQueue[0] <= '0;
                CommandReady[0] <= '0;
        end else begin
            if(cmdType[0] == READ || cmdType[0] == AUTOREAD) begin
                CommandQueue[0] <= {1'b0, CommandQueue[0][4:1]};
                CommandReady[0] <= {1'b1, CommandReady[0][4:1]};
            end else if(cmdType[0] == WRITE || cmdType[0] == AUTOWRITE) begin
                CommandQueue[0] <= {1'b1, CommandQueue[0][4:1]};
                CommandReady[0] <= {1'b1, CommandReady[0][4:1]};
            end
            if(validSig[0]) begin
                CommandReady[0][readyPtr[0]] <= 0;
            end
        end
    end

    always_ff@(posedge clk2x or negedge rst_n) begin
        if(!rst_n) begin
            validSig <= '0;
        end else begin
            if(dramWriteDataIssue[0].valid || dramReadDataReceive[0].valid) begin
                validSig[0] <= 1;
            end else begin
                validSig[0] <= 0;
            end
            if(dramWriteDataIssue[1].valid || dramReadDataReceive[1].valid) begin
                validSig[1] <= 1;
            end else begin
                validSig[1] <= 0;
            end
        end
    end

    always_ff@(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            CommandQueue[1] <= '0;
            CommandReady[1] <= '0; 
        end else begin
            if(cmdType[1] == READ || cmdType[1] == AUTOREAD)begin
                CommandQueue[1] <= {1'b0, CommandQueue[1][4:1]};
                CommandReady[1] <= {1'b1, CommandReady[1][4:1]};
            end else if(cmdType[1] == WRITE || cmdType[1] ==AUTOWRITE) begin
                CommandQueue[1] <= {1'b1, CommandQueue[1][4:1]};
                CommandReady[1] <= {1'b1, CommandReady[1][4:1]};
            end
            if(validSig[1]) begin
                CommandReady[1][readyPtr[1]] <= 0;
            end
        end
    end

    always_comb begin
        readyPtr[0] = 0;
        readyPtr[1] = 0;
        for(int i = 4; i > 0; i--) begin
            if(CommandReady[0][i]) begin
                readyPtr[0] = i;
            end
            if(CommandReady[1][i]) begin
                readyPtr[1] = i;
            end
        end
    end


    assign dramWriteDataIssue[0].valid      = (CommandQueue[0][readyPtr[0]] && (dataBusCnt[0] == 0) && (DDR4_CH0_IF.dqs_t || DDR4_CH0_IF.dqs_c)) ? 1     : 0;
    assign dramWriteDataIssue[0].Issue_time = (CommandQueue[0][readyPtr[0]]  && (dataBusCnt[0] == 0) && (DDR4_CH0_IF.dqs_t || DDR4_CH0_IF.dqs_c)) ? $time : 0;

    assign dramWriteDataIssue[1].valid      = (CommandQueue[1][readyPtr[1]]  && (dataBusCnt[1] == 0) && (DDR4_CH1_IF.dqs_t || DDR4_CH1_IF.dqs_c)) ? 1     : 0;
    assign dramWriteDataIssue[1].Issue_time = (CommandQueue[1][readyPtr[1]]  && (dataBusCnt[1] == 0) && (DDR4_CH1_IF.dqs_t || DDR4_CH1_IF.dqs_c)) ? $time : 0;


    assign dramReadDataReceive[0].valid        = (!CommandQueue[0][readyPtr[0]]  && (dataBusCnt[0] == 0) && (DDR4_CH0_IF.dqs_t || DDR4_CH0_IF.dqs_c)) ?  1    : 0;
    assign dramReadDataReceive[0].Receive_time = (!CommandQueue[0][readyPtr[0]] && (dataBusCnt[0] == 0) && (DDR4_CH0_IF.dqs_t || DDR4_CH0_IF.dqs_c)) ? $time : 0;

    assign dramReadDataReceive[1].valid        = (!CommandQueue[1][readyPtr[1]]  && (dataBusCnt[1] == 0) && (DDR4_CH1_IF.dqs_t || DDR4_CH1_IF.dqs_c)) ?  1    : 0;
    assign dramReadDataReceive[1].Receive_time = (!CommandQueue[1][readyPtr[1]]  && (dataBusCnt[1] == 0) && (DDR4_CH1_IF.dqs_t || DDR4_CH1_IF.dqs_c)) ? $time : 0;


    function automatic logic checkActivate(
        input logic cke,
        input logic act_n,
        input logic [COMMAND_WIDTH-1:0] pin_A
    );
        if({cke, act_n} == 2'b10) begin
            return 1; 
        end else return 0;
    endfunction

    function automatic logic checkAutoPrechargeRead(
        input logic cke,
        input logic act_n,
        input logic [COMMAND_WIDTH-1:0] pin_A
    );
        if({cke, act_n, pin_A[16], pin_A[14], pin_A[10]} == 5'b11111) begin
            if(pin_A[15] == 0 )begin
                return 1;
            end  return 0;
        end
    endfunction

    function automatic logic checkRead(
        input logic cke, 
        input logic act_n,
        input logic [COMMAND_WIDTH-1:0] pin_A
    );
        if({cke, act_n, pin_A[16], pin_A[14]} == 4'b1111) begin
            if({pin_A[15], pin_A[10]} == 2'b00) begin
                return 1;
            end else return 0;
        end
        else return 0;
    endfunction

    function automatic logic checkAutoPrechargeWrite(
        input logic cke,
        input logic act_n,
        input logic [COMMAND_WIDTH-1:0] pin_A
    );
        if({cke, act_n, pin_A[16], pin_A[10]} == 4'b1111) begin
            if({pin_A[15], pin_A[14]}== 2'b00) begin
                return 1;
            end
            else return 0;
        end else return 0;    
    endfunction

    function automatic logic checkWrite(
        input logic cke,
        input logic act_n,
        input logic [COMMAND_WIDTH-1:0] pin_A    
    );
        if({cke, act_n, pin_A[16]} == 3'b111) begin
            if({pin_A[15], pin_A[14], pin_A[10]} == 3'b000) begin
                return 1;
            end else return 0;
        end return 0;
    endfunction

    function automatic logic checkRefresh(
        input logic cke,
        input logic act_n,
        input logic [COMMAND_WIDTH-1:0] pin_A
    );
        if({cke, act_n, pin_A[16], pin_A[15], pin_A[14]} == 5'b11000) begin
            return 1;
        end else return 0;
    endfunction

    function automatic logic checkPrecharge(
        input logic cke, 
        input logic act_n,
        input logic [COMMAND_WIDTH-1:0] pin_A
    );
        if({cke, act_n, pin_A[15]} == 3'b111) begin
            if({pin_A[16], pin_A[14], pin_A[10]} == 3'b000) begin
                return 1;
            end else return 0;
        end else return 0;
    endfunction

endmodule