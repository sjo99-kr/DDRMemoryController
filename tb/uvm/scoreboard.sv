`timescale 1 ns / 1ps

import svut_if::*;
module scoreboard#(
    parameter int DEADLOCKCNT = 1000,
    parameter int AXI_DATAWIDTH = 64,
    parameter int AXI_ADDRWIDTH = 32,
    parameter int AXI_IDWIDTH = 4,
    parameter int AXI_USERWIDTH = 1,
    parameter int BURST_LENGTH = 8,

    parameter int TBREADREQWIDTH = 8,
    parameter int TBWRITEREQWIDTH = 8,
    parameter int ERRORCNTWIDTH = 100,
    
    parameter int  tCL =  16,
    parameter int tCWL = 12,
    parameter int tRCD = 16,
    parameter int tCCDS = 4,
    parameter int tCCDL = 6

) (
    //  COMMON  //
    input wire clk, rst_n,

    //  MONITOR <-> SCOREBOARD   //
    input cache_readReq_Issue ReadReqIssue,
    input cache_writeReq_Issue WriteReqIssue,
    input cache_readResp_Receive ReadRespReceive,
    input cache_writeACK_Receive WriteRespReceive,

    input dram_cmd_Issue dramCmdIssue [$clog2(NUMCHANNEL):0],
    input dram_data_Issue dramDataIssue [$clog2(NUMCHANNEL):0],
    input dram_data_Receive dramDataReceive [$clog2(NUMCHANNEL):0]
);

//  SCOREBOARD 
//  1. ID,User matching between ReadReq, ReadResp
//  2. Data validation for DRAM-READ, DRAM-WRITE
//  3. DRAM TIMING CONSTRAINTS MATCHING
//  4. ID, User matching between WriteReq, Write ACK
//  5. COMMAND FSM MATCHING

///     Error Counting 

    logic [ERRORCNTWIDTH-1:0] ReadReqMatchingError, ReadReqDeadLockError;
    logic [ERRORCNTWIDTH-1:0] WriteReqMatchingError, WriteReqDeadLockError;




    logic [TBREADREQWIDTH-1:0] NumOfReadRequestIssued, NumOfReadResponseReceived;
    logic [TBWRITEREQWIDTH-1:0] NumOfWriteRequestIssued, NumOfWriteResponseReceived;


    TB_READREQENTRY readReqQueue [TBREADREQWIDTH-1:0];
    TB_WRITEREQENTRY writeReqQueue [TBWRITEREQWIDTH-1:0];
    logic [TBREADREQWIDTH-1:0]  readReqQueueFree;
    logic [TBWRITEREQWIDTH-1:0] writeReqQueueFree;
    logic [$clog2(TBREADREQWIDTH):0]  readReqReadPtr,  readReqWritePtr; 
    logic [$clog2(TBWRITEREQWIDTH):0] writeReqReadPtr, writeReqWritePtr;
    
    logic ReadReqIDUserMatching, WriteReqIDUserMatching;

    always_comb begin : RequestIDUserMatching
        readReqReadPtr   = 0;
        writeReqReadPtr  = 0;
        readReqWritePtr  = 0;
        writeReqWritePtr = 0;
        ReadReqIDUserMatching  = 0;
        WriteReqIDUserMatching = 0;

        for(int i =TBREADREQWIDTH-1; i>0; i--)begin
            if(readReqQueueFree[i]) begin
                readReqWritePtr = i;
            end
        end
        for (int i = TBWRITEREQWIDTH-1; i > 0; i--) begin
            if(writeReqQueueFree[i]) begin
                writeReqWritePtr = i;
            end
        end

        for(int i = 0; i < TBREADREQWIDTH; i++) begin
            if(({readReqQueue[i].id, readReqQueue[i].user}
                == {ReadRespReceive.id, ReadRespReceive.user}) && !readReqQueueFree[i]) begin
                    readReqReadPtr = i ;
                    ReadReqIDUserMatching = 1;
            end
        end
        for(int i = 0; i < TBWRITEREQWIDTH; i++) begin
            if(({writeReqQueue[i].id , writeReqQueue[i].user}
            == {WriteRespReceive.id, WriteRespReceive.user}) && !writeReqQueueFree[i]) begin
                writeReqReadPtr = i;
                WriteReqIDUserMatching = 1;
            end
        end
    end : RequestIDUserMatching

    always_ff@(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            for(int i = 0; i < TBREADREQWIDTH; i++) begin
                readReqQueue[i].id       <= '0;
                readReqQueue[i].user     <= '0;
                readReqQueue[i].sendTime <= '0;
            end
            for(int i = 0; i < TBWRITEREQWIDTH; i++) begin
                writeReqQueue[i].id         <= '0;
                writeReqQueue[i].user       <= '0;
                writeReqQueue[i].sendTime   <= '0;
            end
            readReqQueueFree          <= '1;
            NumOfReadRequestIssued    <= '0;
            NumOfReadResponseReceived <= '0;
        
            writeReqQueueFree           <= '1;
            NumOfWriteRequestIssued     <= '0;
            NumOfWriteResponseReceived  <= '0;
            
            ReadReqMatchingError  <= 0;
            WriteReqMatchingError <= 0;
        end else begin
            if(ReadReqIssue.valid) begin
                readReqQueue[readReqWritePtr].id        <= ReadReqIssue.id;
                readReqQueue[readReqWritePtr].user      <= ReadReqIssue.user;
                readReqQueue[readReqWritePtr].sendTime  <= $time;
                readReqQueueFree[readReqWritePtr]       <= 0;
                NumOfReadRequestIssued <= NumOfReadRequestIssued + 1;
            end
            if(WriteReqIssue.valid) begin
                writeReqQueue[writeReqWritePtr].id       <= WriteReqIssue.id;
                writeReqQueue[writeReqWritePtr].user     <= WriteReqIssue.user;
                writeReqQueue[writeReqWritePtr].sendTime <= $time;
                writeReqQueueFree[writeReqWritePtr]      <= 0;
                NumOfWriteRequestIssued <= NumOfWriteRequestIssued + 1;
            end

            if(ReadRespReceive.valid) begin
                if(ReadReqIDUserMatching) begin
                    readReqQueue[readReqReadPtr].sendTime <= 0;
                    readReqQueueFree[readReqReadPtr]      <= 1;
                    NumOfReadResponseReceived <= NumOfReadResponseReceived + 1;
                end else begin
                    ReadReqMatchingError <= ReadReqMatchingError + 1;
                    error_msg("Read Req/Resp ID/User Mismatching");
                end
            end
            if(WriteRespReceive.valid) begin
                if(WriteReqIDUserMatching) begin
                    writeReqQueue[writeReqReadPtr].sendTime <= 0;
                    writeReqQueueFree[writeReqReadPtr]      <= 1;
                    NumOfWriteResponseReceived <= NumOfWriteResponseReceived + 1; 
                end else begin
                    WriteReqMatchingError <= WriteReqMatchingError + 1;
                    error_msg("Read Req/Resp ID/User Mismatching");
                end
            end
        end
    end

    always_ff@(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            for(int i = 0; i< TBREADREQWIDTH; i++) begin
                readReqQueue[i].cnt <= 0;
            end
            for(int i = 0; i < TBWRITEREQWIDTH; i++) begin
                writeReqQueue[i].cnt <= 0;
            end
            ReadReqDeadLockError  <= 0;
            WriteReqDeadLockError <= 0;
        end else begin
            for(int i = 0; i < TBREADREQWIDTH; i++) begin
                if(!readReqQueueFree[i]) begin
                    readReqQueue[i].cnt <= readReqQueue[i].cnt  + 1;
                    if(readReqQueue[i].cnt > DEADLOCKCNT) begin
                        ReadReqDeadLockError <= ReadReqDeadLockError + 1;
                        error_msg("Read Request Deadlock Occurred");
                    end
                end
            end
            for(int i = 0; i < TBWRITEREQWIDTH; i++) begin
                if(!writeReqQueueFree[i]) begin
                    writeReqQueue[i].cnt <= writeReqQueue[i].cnt + 1;
                    if(writeReqQueue[i].cnt > DEADLOCKCNT) begin
                        WriteReqDeadLockError <= WriteReqDeadLockError + 1;
                        error_msg("Write Request Deadlock Occurred");
                    end
                end
            end
            if(ReadRespReceive.valid) begin
                if(ReadReqIDUserMatching) begin
                    readReqQueue[readReqReadPtr].cnt  <= 0;
                end
            end
            if(WriteRespReceive.valid) begin
                if(WriteReqIDUserMatching) begin
                    writeReqQueue[writeReqReadPtr].cnt <= 0;
                end
            end
        end
    end

    logic[tCL-1:0] ch0_readCmdQueue, ch1_readCmdQueue;
    logic[tCWL-1:0] ch0_writeCmdQueue, ch1_writeCmdQueue;
    logic [ERRORCNTWIDTH-1:0] Ch0_ReadDataBusTimingError, Ch1_ReadDataBusTimingError;
    logic [ERRORCNTWIDTH-1:0] Ch0_WriteDataBusTimingError, Ch1_WriteDataBusTimingError;

    always_ff @(posedge clk or negedge rst_n)begin :tCLtCWLCheck_0
        if(!rst_n) begin
            ch0_readCmdQueue        <= '0;
            ch0_writeCmdQueue       <= '0;
            ch1_readCmdQueue        <= '0;
            ch1_writeCmdQueue       <= '0;

        end  else begin
            if(dramCmdIssue[0].cmdType == READ) begin
                ch0_readCmdQueue <= {1'b1, {ch0_readCmdQueue[tCL-2:1]}};
            end else if (dramCmdIssue[0].cmdType == WRITE) begin
                ch0_writeCmdQueue <= {1'b1, {ch0_writeCmdQueue[tCWL-2:1]}};
            end else begin
                ch0_readCmdQueue  <= {1'b0, {ch0_readCmdQueue[tCL-2:1]}};
                ch0_writeCmdQueue <= {1'b0, {ch0_writeCmdQueue[tCWL-2:1]}}; 
            end

            if(dramCmdIssue[1].cmdType == READ) begin
                ch1_readCmdQueue  <= {1'b1, {ch1_readCmdQueue[tCL-2:1]}};
            end else if (dramCmdIssue[1].cmdType == WRITE) begin
                ch1_writeCmdQueue <= {1'b1, {ch1_writeCmdQueue[tCWL-2:1]}};
            end else begin
                ch1_readCmdQueue  <= {1'b0, {ch1_readCmdQueue[tCL-2:1]}}; 
                ch1_writeCmdQueue <= {1'b0, {ch1_writeCmdQueue[tCWL-2:1]}};
            end
        end
    end: tCLtCWLCheck_0

    always_ff@(posedge clk or negedge rst_n) begin :tCLtCWLCheck_1
        if(!rst_n) begin
            Ch0_ReadDataBusTimingError  <= 0;
            Ch1_ReadDataBusTimingError  <= 0;
            Ch0_WriteDataBusTimingError <= 0;
            Ch1_WriteDataBusTimingError <= 0;
        end else begin
            if(dramDataReceive[0].valid && !ch0_readCmdQueue[0]) begin
                Ch0_ReadDataBusTimingError <= Ch0_ReadDataBusTimingError + 1;
                error_msg("Channel_0 : Read Request DataBus Timing Violation (tCL)");
            end 
            if(dramDataReceive[1].valid && !ch1_readCmdQueue[0]) begin
                Ch1_ReadDataBusTimingError <= Ch1_ReadDataBusTimingError + 1;
                error_msg("Channel_1 : Read Request DataBus Timing Violation (tCL)");
            end
            if(dramDataIssue[0].valid && !ch0_writeCmdQueue[0]) begin
                Ch0_WriteDataBusTimingError <= Ch0_WriteDataBusTimingError + 1;
                error_msg("Channel_0 : Write Request DataBus Timing Violation (tCWL)");
            end
            if(dramDataIssue[1].valid && !ch1_writeCmdQueue[0]) begin
                Ch1_WriteDataBusTimingError <= Ch1_WriteDataBusTimingError + 1;
                error_msg("Channel_1 : Write Request DataBus Timing Violation (tCWL)");
            end
        end
    end : tCLtCWLCheck_1


    TB_CCDUNIT Ch0_ccdUnit , Ch1_ccdUnit;
    logic [$clog2(tCCDL)+1:0] Ch0_ccdCnt, Ch1_ccdCnt;
    logic Ch0_ccdCntOverflow, Ch1_ccdCntOverflow;

    logic [ERRORCNTWIDTH-1:0] Ch0_tCCDTimingError, Ch1_tCCDTimingError;

    always_ff@(posedge clk or negedge rst_n) begin :tCCDCheck
        if(!rst_n) begin
            Ch0_ccdUnit <= '0;
            Ch1_ccdUnit <= '0;
            Ch0_ccdCnt  <= '0;
            Ch1_ccdCnt  <= '0;
            Ch0_tCCDTimingError <= 0;
            Ch1_tCCDTimingError <= 0;
        end else begin
            if((dramCmdIssue[0].cmdType == READ) || (dramCmdIssue[0].cmdType == WRITE)) begin
                Ch0_ccdUnit.bankgroup <= dramCmdIssue[0].bg;
                Ch0_ccdUnit.valid <= 1;
                Ch0_ccdCnt <= 0;
                Ch0_ccdCntOverflow <= 0;
                if(Ch0_ccdUnit.valid) begin
                    if(dramCmdIssue[0].bg == Ch0_ccdUnit.bankgroup) begin
                        if(!Ch0_ccdCntOverflow) begin
                            if(Ch0_ccdCnt < tCCDS-1) begin
                                Ch0_tCCDTimingError <= Ch0_tCCDTimingError + 1;
                                error_msg("Channel_0 : Read/Write Timing Error (tCCDS)");
                            end 
                        end
                    end else begin
                        if(!Ch0_ccdCntOverflow) begin
                            if(Ch0_ccdCnt < tCCDL-1) begin
                                Ch0_tCCDTimingError <= Ch0_tCCDTimingError + 1;
                                error_msg("Channel_0 : Read/Write Timing Error (tCCDL)");
                            end
                        end
                    end
                end
            end else begin
                Ch0_ccdCnt <= Ch0_ccdCnt + 1;
                if(Ch0_ccdCnt > tCCDL-1) begin
                    Ch0_ccdCntOverflow <= 1;
                end 
            end
            if((dramCmdIssue[1].cmdType == READ) || (dramCmdIssue[1].cmdType == WRITE)) begin
                Ch1_ccdUnit.bankgroup <= dramCmdIssue[1].bg;
                Ch1_ccdUnit.valid     <= 1;
                Ch1_ccdCnt            <= 0;
                Ch1_ccdCntOverflow    <= 0;
                if(Ch1_ccdUnit.valid) begin
                    if(dramCmdIssue[1].bg == Ch1_ccdUnit.bankgroup)begin
                        if(!Ch1_ccdCntOverflow) begin
                            if(Ch1_ccdCnt < tCCDS-1)begin
                                Ch1_tCCDTimingError <= Ch1_tCCDTimingError + 1;
                                error_msg("Channel_1 : Read/Write Timing Error (tCCDS)");
                            end
                        end
                    end else begin
                        if(!Ch0_ccdCntOverflow) begin
                            if(Ch1_ccdCnt < tCCDL-1) begin
                                Ch1_tCCDTimingError <= Ch1_tCCDTimingError + 1;
                                error_msg("Channel_1 : Read/Write Timing Error (tCCDL)");
                            end
                        end
                    end
                end
            end else begin
                Ch1_ccdCnt <= Ch1_ccdCnt + 1;
                if(Ch1_ccdCnt > tCCDL-1) begin
                    Ch1_ccdCntOverflow <= 1;
                end
            end
        
        end
    end : tCCDCheck

    
    TB_RCDRANK Ch0_tRCDQueue [NUMRANK-1:0];
    TB_RCDRANK Ch1_tRCDQueue [NUMRANK-1:0];
    logic [ERRORCNTWIDTH-1:0] Ch0_tRCDTimingError, Ch1_tRCDTimingError;

    always_ff@(posedge clk or negedge rst_n) begin : tRCDChecking_0
        if(!rst_n) begin
            for(int i = 0; i < NUMRANK; i++) begin
                for(int j = 0; j < NUM_BANKFSM; j++) begin
                    Ch0_tRCDQueue[i].actTiming[j] <= '0;
                    Ch1_tRCDQueue[i].actTiming[j] <= '0;
                end
            end
        end else begin
            if(dramCmdIssue[0].cmdType == ACTIVATE) begin
                for(int i = 0; i < NUMRANK; i++) begin
                    for(int j = 0; j < NUM_BANKFSM; j++) begin
                        if((i == dramCmdIssue[0].rank) && (j == {dramCmdIssue[0].bg, dramCmdIssue[0].bk})) begin
                            Ch0_tRCDQueue[i].actTiming[j] <= {1'b1, Ch0_tRCDQueue[i].actTiming[j][tRCD-1:1]};
                        end else begin
                            Ch0_tRCDQueue[i].actTiming[j] <= {1'b0, Ch0_tRCDQueue[i].actTiming[j][tRCD-1:1]};
                        end
                    end
                end
            end else begin
                for(int i = 0; i< NUMRANK; i++) begin
                    for(int j = 0; j < NUM_BANKFSM; j++) begin
                        Ch0_tRCDQueue[i].actTiming[j] <= {1'b0, Ch0_tRCDQueue[i].actTiming[j][tRCD-1:1]};
                    end
                end
            end
            if(dramCmdIssue[1].cmdType == ACTIVATE) begin
                for(int i = 0; i< NUMRANK; i++) begin
                    for(int j = 0; j < NUM_BANKFSM; j++) begin
                        if((i == dramCmdIssue[1].rank) && (j == {dramCmdIssue[1].bg, dramCmdIssue[1].bk})) begin
                            Ch1_tRCDQueue[i].actTiming[j] <= {1'b1, Ch1_tRCDQueue[i].actTiming[j][tRCD-1:1]};
                        end else begin
                            Ch1_tRCDQueue[i].actTiming[j] <= {1'b0, Ch1_tRCDQueue[i].actTiming[j][tRCD-1:1]};
                        end
                    end
                end
            end else begin
                for(int i = 0; i< NUMRANK; i++) begin
                    for(int j = 0; j < NUM_BANKFSM; j++) begin
                        Ch1_tRCDQueue[i].actTiming[j] <= {1'b0, Ch1_tRCDQueue[i].actTiming[j][tRCD-1:1]};
                    end
                end
            end
        end
    end : tRCDChecking_0


    always_ff@(posedge clk or negedge rst_n) begin : tRCDChecking_1
        if(!rst_n) begin
            Ch0_tRCDTimingError <= 0;
            Ch1_tRCDTimingError <= 0;
        end else begin
            if((dramCmdIssue[0].cmdType == READ) || (dramCmdIssue[0].cmdType == WRITE)) begin
                if(|Ch0_tRCDQueue[dramCmdIssue[0].rank].actTiming[{dramCmdIssue[0].bg, dramCmdIssue[0].bk}]) begin
                    Ch0_tRCDTimingError <= Ch0_tRCDTimingError +1;
                    error_msg("Channel_0 : Read/Write Timing Error (tRCD)");
                end
            end 
            if((dramCmdIssue[1].cmdType == READ) || (dramCmdIssue[1].cmdType == WRITE)) begin
                if(|Ch1_tRCDQueue[dramCmdIssue[1].rank].actTiming[{dramCmdIssue[1].bg, dramCmdIssue[1].bk}]) begin
                    Ch1_tRCDTimingError <= Ch1_tRCDTimingError + 1;
                    error_msg("Channel_1 : Read/Write Timing Error (tRCD)");
                end
            end     
        end
    end : tRCDChecking_1



    final begin
        int totalErrors;

        totalErrors = 
            ReadReqMatchingError
            + WriteReqMatchingError
            + ReadReqDeadLockError
            + WriteReqDeadLockError
            + Ch0_ReadDataBusTimingError
            + Ch1_ReadDataBusTimingError
            + Ch0_WriteDataBusTimingError
            + Ch1_WriteDataBusTimingError
            + Ch0_tCCDTimingError
            + Ch1_tCCDTimingError
            + Ch0_tRCDTimingError
            + Ch1_tRCDTimingError;

        $display("\n====================================================================");
        $display("                        VERIFICATION SUMMARY                          ");
        $display("======================================================================");

        $display("\n[REQUEST STATISTICS]");
        $display("  Read  Requests Issued     : %0d", NumOfReadRequestIssued);
        $display("  Read  Responses Received  : %0d", NumOfReadResponseReceived);
        $display("  Write Requests Issued     : %0d", NumOfWriteRequestIssued);
        $display("  Write Responses Received  : %0d", NumOfWriteResponseReceived);

        $display("\n[REQUEST MATCHING ERRORS]");
        $display("  Read  ID/User Mismatch    : %0d", ReadReqMatchingError);
        $display("  Write ID/User Mismatch    : %0d", WriteReqMatchingError);

        $display("\n[DEADLOCK ERRORS]");
        $display("  Read  Deadlock Errors     : %0d", ReadReqDeadLockError);
        $display("  Write Deadlock Errors     : %0d", WriteReqDeadLockError);

        $display("\n[DATA BUS TIMING ERRORS]");
        $display("  Channel 0 Read  (tCL)     : %0d", Ch0_ReadDataBusTimingError);
        $display("  Channel 1 Read  (tCL)     : %0d", Ch1_ReadDataBusTimingError);
        $display("  Channel 0 Write (tCWL)    : %0d", Ch0_WriteDataBusTimingError);
        $display("  Channel 1 Write (tCWL)    : %0d", Ch1_WriteDataBusTimingError);

        $display("\n[tCCD TIMING ERRORS]");
        $display("  Channel 0                 : %0d", Ch0_tCCDTimingError);
        $display("  Channel 1                 : %0d", Ch1_tCCDTimingError);

        $display("\n[tRCD TIMING ERRORS]");
        $display("  Channel 0                 : %0d", Ch0_tRCDTimingError);
        $display("  Channel 1                 : %0d", Ch1_tRCDTimingError);

        $display("\n--------------------------------------------------------------------");

        if (totalErrors == 0)
            $display("  RESULT : PASS  ✅  (No errors detected)");
        else
            $display("  RESULT : FAIL  ❌  (Total Errors = %0d)", totalErrors);
        $display("=====================================================================\n");

    end

endmodule
