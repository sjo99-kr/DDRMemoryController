`timescale 1ns / 1ps
import MemoryController_Definitions::*;
import svut_if::*;


`define DISPLAY
module Top;

    logic clk, rst_n, clk2x;
    
    cache_side_request CacheReq;
    cache_side_response CacheResp;

    localparam int TEST_MODE = 0; // 0 for Read Transaction  |  1 for Write Transaction


    initial begin
        clk = 1;
        clk2x = 1;
        rst_n = 0;
        #4 rst_n = 1;
    end

    always #2 clk = ~clk;
    always #1 clk2x = ~clk2x;

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
    ) DDR4Interface_CH0(
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
    ) DDR4Interface_CH1(
        .clk(clk), .rst(rst_n)
    );

    MemoryController #(
        .AXI_DATAWIDTH(AXI_DATAWIDTH), .AXI_ADDRWIDTH(AXI_ADDRWIDTH), 
        .AXI_IDWIDTH(AXI_IDWIDTH), .AXI_USERWIDTH(AXI_USERWIDTH),
        .MEM_DATAWIDTH(MEM_DATAWIDTH), .MEM_ADDRWIDTH(MEM_ADDRWIDTH),
        .MEM_IDWIDTH(MEM_IDWIDTH), .MEM_USERWIDTH(MEM_USERWIDTH),
        .COMMAND_WIDTH(COMMAND_WIDTH), 
        .CHWIDTH(CHWIDTH), .RKWIDTH(RKWIDTH), .BGWIDTH(BGWIDTH),
        .CWIDTH(CWIDTH), .RWIDTH(RWIDTH), .BKWIDTH(BKWIDTH),
        .NUMCHANNEL(NUMCHANNEL), .NUMRANK(NUMRANK),
        .NUMBANKGROUP(NUMBANKGROUP), .NUMBANK(NUMBANK),
        .NUM_FSM(NUM_FSM), 
        .THRESHOLD(THRESHOLD), .AGINGWIDTH(AGINGWIDTH),
        .READCMDQUEUEDEPTH(READCMDQUEUEDEPTH), .WRITECMDQUEUEDEPTH(WRITECMDQUEUEDEPTH),
        .OPENPAGELISTDEPTH(OPENPAGELISTDEPTH),
        .RESPSCHEDULINGCNT(RESPSCHEDULINGCNT), .ASSEMBLER_DEPTH(ASSEMBLER_DEPTH),
        .READBUFFERDEPTH(READBUFFERDEPTH), .WRITEBUFFERDEPTH(WRITEBUFFERDEPTH),
        .PHYFIFOMAXENTRY(PHYFIFOMAXENTRY), .PHYFIFODEPTH(PHYFIFODEPTH), .PHYFIFOREQUESTWINDOW(PHYFIFOREQUESTWINDOW),
        .BURST_LENGTH(BURST_LENGTH),
        .tBL(tBL), .tCCDS(tCCDS), .tCCDL(tCCDL), .tRTRS(tRTRS), .tCL(tCL), .tRCD(tRCD),
        .tRP(tRP), .tCWL(tCWL), .tRTW(tRTW), .tRAS(tRAS), .tRC(tRC), .tRTP(tRTP),
        .tWTRS(tWTRS), .tWTRL(tWTRL), .tWR(tWR), .tRFC(tRFC), .tREFI(tREFI), .CHMODETHRESHOLD(CHMODETHRESHOLD)
    ) MemoryController_Instance(
        .clk(clk), .rst_n(rst_n), .clk2x(clk2x),
        .cache_req(CacheReq), .cache_resp(CacheResp),
        .DDR4_CH0_IF(DDR4Interface_CH0), .DDR4_CH1_IF(DDR4Interface_CH1)
    );


    MemoryBFM #(
        .NUMCHANNEL(NUMCHANNEL),
        .NUMRANK(NUMRANK),
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
    ) MemoryBFM_Instance(
        .clk(clk), .rst_n(rst_n), .clk2x(clk2x),
        .DDR4_CH0_IF(DDR4Interface_CH0),
        .DDR4_CH1_IF(DDR4Interface_CH1)
    );







    
    


    //          Case for serving Requsets               //
    initial begin
        if(TEST_MODE == 0)begin
            #50
            AddressWriteRequest(CacheReq, CacheResp, 0, 0, 1, 1, 0, 4, 2, 0);
            #10
            AddressWriteRequest(CacheReq, CacheResp, 0, 1, 1, 1, 0, 4, 1, 0);
            #10
            DataWriteRequest(CacheReq, CacheResp, 1, 0);         
            #100
            DataWriteRequest(CacheReq, CacheResp, 2, 0);
            #3000 
            $finish();
        end else if(TEST_MODE == 1) begin
            #50
            ReadRequest(CacheReq, CacheResp, 0, 0, 1, 1, 0, 4, 2, 0);
            #10
            ReadRequest(CacheReq, CacheResp, 0, 1, 1, 1, 0, 2, 5, 0);
            #3000 
            $finish(); 
        end 
    end





    
    //      Task 1.     Read    Request  From    Cache-side            //
    task automatic ReadRequest(         
        ref cache_side_request CacheReq,
        ref cache_side_response CacheResp,
        input logic [CHWIDTH-1:0] channel,
        input logic [RKWIDTH-1:0] rank,
        input logic [BGWIDTH-1:0] bankgroup,
        input logic [BKWIDTH-1:0] bank,
        input logic [RWIDTH-1:0] row,
        input logic [CWIDTH-1:0] col,
        input logic [AXI_IDWIDTH-1:0] id,
        input logic [AXI_USERWIDTH-1:0] user
    );
        @(negedge clk);
        CacheReq.ar_valid = 0;

        do begin
        @(negedge clk);
        CacheReq.ar.addr = {channel, rank, bankgroup, bank, row, col};
        CacheReq.ar.id = id;
        CacheReq.ar.user = user;
        CacheReq.r_ready = 1;
        CacheReq.b_ready = 1;
        CacheReq.ar_valid = 1;
        end while(!CacheResp.ar_ready);

        @(negedge clk);
        CacheReq.ar_valid = 0;
    endtask

    //      Task 2.     Address Write  Request  From   Cache-side          //
    task automatic AddressWriteRequest(
        ref cache_side_request CacheReq,
        ref cache_side_response CacheResp,
        input logic [CHWIDTH-1:0] channel,
        input logic [RKWIDTH-1:0] rank,
        input logic [BGWIDTH-1:0] bankgroup,
        input logic [BKWIDTH-1:0] bank,
        input logic [RWIDTH-1:0] row,
        input logic [CWIDTH-1:0] col,
        input logic [AXI_IDWIDTH-1:0] id,
        input logic [AXI_USERWIDTH-1:0] user        
    );
        @(negedge clk);
        CacheReq.aw_valid = 0;
        CacheReq.b_ready = 1;
        do begin 
        @(negedge clk);
        CacheReq.aw.addr = {channel, rank, bankgroup, bank, row, col};
        CacheReq.aw.id = id;
        CacheReq.aw.user = user;
        CacheReq.aw_valid = 1; 

        end while(!CacheResp.aw_ready || !CacheResp.w_ready);

        @(negedge clk);
        CacheReq.aw_valid = 0;
    endtask

    //      Task 3. Write Data  Request  From   Cache-side              //
    task automatic DataWriteRequest(
        ref cache_side_request CacheReq,
        ref cache_side_response CacheResp,
        input logic [AXI_IDWIDTH-1:0] id,
        input logic [AXI_USERWIDTH-1:0] user    
    );
        integer burstCnt;

        @(negedge clk);
        CacheReq.w_valid = 0;
        CacheReq.w.last = 0;
        burstCnt = 0;

        while(burstCnt != BURST_LENGTH) begin
            @(negedge clk);
            CacheReq.w_valid = 1;
            CacheReq.w.id = id;
            CacheReq.w.user = user;
            CacheReq.w.data = burstCnt;
            CacheReq.w.strb = '0;
            
            if(burstCnt == BURST_LENGTH-1) CacheReq.w.last = 1;
            if(CacheResp.w_ready) burstCnt++;
        end;

        @(negedge clk);
        CacheReq.w_valid = 0;
        CacheReq.w.last = 0;
        burstCnt = 0;
    endtask

endmodule
