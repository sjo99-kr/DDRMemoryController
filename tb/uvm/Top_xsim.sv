`timescale 1ns / 1ps
import MemoryController_Definitions::*;
import svut_if::*;


`define DISPLAY
module Top_xsim;

    logic clk, rst_n, clk2x;
    
    cache_side_request CacheReq;
    cache_side_response CacheResp;

    initial begin
        clk = 1;
        clk2x = 1;
        rst_n = 0;
        #4 rst_n = 1;
    end

    always #2 clk = ~clk;
    always #1 clk2x = ~clk2x;

    `ifndef VERILATOR


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
    `endif

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

    cache_readReq_Issue monitor_cacheReadReqIssue;
    cache_writeReq_Issue monitor_cacheWriteReqIssue;
    cache_readResp_Receive monitor_cacheReadRespReceive;
    cache_writeACK_Receive monitor_cacheWriteACKReceive;
    
    dram_cmd_Issue monitor_dramCmdIssue [$clog2(NUMCHANNEL):0];
    dram_data_Issue monitor_dramDataIssue [$clog2(NUMCHANNEL):0];
    dram_data_Receive monitor_dramDataReceive [$clog2(NUMCHANNEL):0];


    logic driverEnable;
    
    axi_ar_chan_t ar_driver;
    logic ar_valid_driver, ar_ready_driver;

    axi_aw_chan_t aw_driver;
    logic aw_valid_driver, aw_ready_driver;

    axi_w_chan_t w_driver;
    logic w_valid_driver, w_ready_driver;



    
    

    driver #(
        .TIMEOUT(100),
        .AXI_DATAWIDTH(AXI_DATAWIDTH),
        .AXI_ADDRWIDTH(AXI_ADDRWIDTH),
        .AXI_USERWIDTH(AXI_USERWIDTH),
        .MODELENGTH(32),
        .BURST_LENGTH(8)
    ) driver_instance(
        .clk(clk), .rst_n(rst_n), .testEnable(driverEnable),
        .ar_id(ar_driver.id), .ar_user(ar_driver.user), .ar_addr(ar_driver.addr),
        .ar_valid(ar_valid_driver), .ar_ready(ar_ready_driver),

        .aw_id(aw_driver.id), .aw_user(aw_driver.user), .aw_addr(aw_driver.addr),
        .aw_valid(aw_valid_driver), .aw_ready(aw_ready_driver),
        
        .w_id(w_driver.id), .w_user(w_driver.user), .w_data(w_driver.data),
        .w_last(w_driver.last), .w_strb(w_driver.strb), 
        .w_valid(w_valid_driver), .w_ready(w_ready_driver)

    );

    monitor monitor_instance(
        .clk(clk), .rst_n(rst_n), .clk2x(clk2x),
        .cache_req(CacheReq), .cache_resp(CacheResp),
        .DDR4_CH0_IF(DDR4Interface_CH0), .DDR4_CH1_IF(DDR4Interface_CH1),
        .cacheReadReqIssue(monitor_cacheReadReqIssue), .cacheWriteReqIssue(monitor_cacheWriteReqIssue),
        .cacheReadRespReceive(monitor_cacheReadRespReceive), .cacheWriteACKReceive(monitor_cacheWriteACKReceive),
        .dramCmdIssue(monitor_dramCmdIssue),
        .dramWriteDataIssue(monitor_dramDataIssue),
        .dramReadDataReceive(monitor_dramDataReceive)
    );
    
    scoreboard #(
        .DEADLOCKCNT(DEADLOCKCNT),
        .AXI_DATAWIDTH(AXI_DATAWIDTH),
        .AXI_IDWIDTH(AXI_IDWIDTH),
        .AXI_USERWIDTH(AXI_USERWIDTH),
        .BURST_LENGTH(BURST_LENGTH),
        .TBREADREQWIDTH(TBREADREQWIDTH),
        .TBWRITEREQWIDTH(TBWRITEREQWIDTH),
        .ERRORCNTWIDTH(ERRORCNTWIDTH),
        .tCL(tCL), .tCWL(tCWL), .tRCD(tRCD), .tCCDS(tCCDS), .tCCDL(tCCDL)
    ) scoreboard_instance(
        .clk(clk), .rst_n(rst_n),
        .ReadReqIssue(monitor_cacheReadReqIssue), .WriteReqIssue(monitor_cacheWriteReqIssue),
        .ReadRespReceive(monitor_cacheReadRespReceive), .WriteRespReceive(monitor_cacheWriteACKReceive),
        
        .dramCmdIssue(monitor_dramCmdIssue), .dramDataIssue(monitor_dramDataIssue),
        .dramDataReceive(monitor_dramDataReceive) 
    );
    logic r_ready_driver, b_ready_driver;

    generate 
             assign CacheReq.aw       = aw_driver;
             assign CacheReq.aw_valid = aw_valid_driver;
             assign CacheReq.w        = w_driver;
             assign CacheReq.w_valid  = w_valid_driver;
             assign CacheReq.ar       = ar_driver;
             assign CacheReq.ar_valid = ar_valid_driver;
             assign CacheReq.r_ready  = r_ready_driver;
             assign CacheReq.b_ready  = b_ready_driver;

             assign aw_ready_driver   = CacheResp.aw_ready;
             assign ar_ready_driver   = CacheResp.ar_ready;
             assign w_ready_driver    = CacheResp.w_ready;
     endgenerate


    initial begin
            #50
            SVUMSetup();
            SVUMRun(driverEnable, 100);        
            SVUMFinish(500);
    end


    task automatic  SVUMSetup();
        r_ready_driver = 1;
        b_ready_driver = 1;
    endtask

    task automatic SVUMRun(ref logic UVMEnable,
                            input integer RUNTIME);
        integer i = 0;
        @(posedge clk); 
        UVMEnable = 0;

        @(posedge clk);
        UVMEnable = 1;
        
        do begin
            @(posedge clk);
            i = i + 1;
        end while(i < RUNTIME);

        UVMEnable = 0;
    endtask

    task automatic SVUMFinish(input integer ENDTIME);
        integer i = 0;
        @(posedge clk);
        do begin
            @(posedge clk);
            i = i + 1;
        end while (i < ENDTIME);
        
        $finish(); 
    endtask


endmodule
