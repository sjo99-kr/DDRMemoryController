`timescale 1ns / 1ps

//------------------------------------------------------------------------------
//      WriteBufferController
//
//      Role:
//          Write data buffer between cache-side frontend and DDR PHY.
//
//      Responsibilities:
//          - Buffers write data from cache-side frontend.
//          - Tracks issued write commands from RankFSM.
//          - Streams burst write data to PHY in-order.
//          - Generates write response (ACK) for frontend.
//
//      Scope & Notes:
//          - Data-level buffering only (no DRAM timing logic).
//          - One outstanding write entry corresponds to one burst.
//          - PHY-side backpressure handled via FIFO-ready signaling.
//
//      Author  : Seongwon Jo
//      Created : 2026.02
//------------------------------------------------------------------------------

module WriteBufferController #(
    parameter int BUFFER_CHANNEL = 0,
    parameter int MEM_DATAWIDTH = 64,
    parameter int MEM_USERWIDTH = 1,
    parameter int MEM_IDWIDTH = 4,
    parameter int NUMRANK = 4,
    parameter int WRITEBUFFERDEPTH = 128,
    parameter int BURST_LENGTH = 8,
    
    parameter type WriteBufferDataEntry = logic,
    parameter type WriteBufferDirEntry = logic,
    parameter type MemoryAddress = logic

) (
    // common
    input logic clk, rst,

    // Input Cache-side
                                                                        //////////////////////////////////////////////////
                                                                        //        Input from MC FrontEnd (Cache-side)   //
    input logic [MEM_DATAWIDTH-1:0] writeData,                          // 1. Write Data for Write Request              //
    input logic [MEM_USERWIDTH-1:0] writeDataUser,                      // 2. Write Data User for Write Request         //
    input logic [MEM_IDWIDTH-1:0] writeDataID,                          // 3. Write Data ID for Write Request           //
    input logic [MEM_DATAWIDTH/BURST_LENGTH -1 : 0] writeDataStrb,      // 4. Write Data Strb for Write Request         //
    input MemoryAddress writeDataAddr,                                  // 5. Write Data Addr for Write Request         //
    input logic writeDataLast,                                          // 6. Write Data Last for Write Request         //
    input logic writeDataValid,                                         // 7. Write Data Valid for Write Request        //
    input logic writeDataACKReady,                                      // 8. Write Data ACK Ready for AXI-WRITE Resp.  //
                                                                        //////////////////////////////////////////////////

                                                                        //////////////////////////////////////////////////
                                                                        //    Output to MC FrontEnd   (Cache-side)      //
    output logic writeDataACKValid,                                     // 1. Write Data ACK Valid for AXI-WRITE Resp.  //
    output logic [MEM_USERWIDTH-1:0] writeDataACKUser,                  // 2. Write Data ACK User for AXI-WRITE Resp.   //
    output logic [MEM_IDWIDTH-1:0] writeDataACKID,                      // 3. Write Data ACK ID for AXI-WRITE Resp.     //      
    output logic writeBufferFull,                                       // 4. Write Buffer Ready for MC-FrontEnd Phase  //
                                                                        //////////////////////////////////////////////////
                                                                        
                                                                        ////////////////////////////////////////////////
                                                                        //              INPUT FROM DQ-BUS             //
    input logic phyWriteModeFIFOReady,                                  //  1. PHYWriteMode FIFO Ready from PHY-side  //
    input logic WriteModeACK,                                           //  2. PHYWriteMode ACK                       //
                                                                        ////////////////////////////////////////////////

                                                                        ////////////////////////////////////////////////
                                                                        //              Output to DQ-BUS              //
    output logic [MEM_DATAWIDTH-1:0] readData,                          // 1. Read Data to DQ-Bus (64-B)              //
    output logic readDataLast,                                          // 2. Read Data Last to DQ-Bus                //
    output logic readDataValid,                                         // 3. Read Data Valid to DQ-Bus               //
    output logic [MEM_DATAWIDTH/BURST_LENGTH -1 : 0]  readDataStrb,     // 4. Read Data Strb to DQ-BUs                //
                                                                        ////////////////////////////////////////////////

                                                                        ////////////////////////////////////////////////
                                                                        //          Input from Channel Scheduler      //
    input logic [MEM_IDWIDTH-1:0] writeBufferAllocID,                   //  1. Write Buffer Entry Allocation ID       //      
    input logic [MEM_USERWIDTH-1:0] writeBufferAllocUser,               //  2. Write Buffer Entry Allocation USER     //
    input MemoryAddress writeBufferAllocAddr,                           //  3. Write Buffer Entry Allocation Addr     //
    input logic writeBufferAllocValid,                                  //  4. Write Buffer Entry Allocation Valid    //
                                                                        ////////////////////////////////////////////////
    
                                                                        /////////////////////////////////////////////////
                                                                        //       Output to Channel Scheduler           //
    output logic [NUMRANK-1:0] writeBufferAvailable ,                   //  1. Write Buffer Available (Dir/Data Buf)   //
                                                                        /////////////////////////////////////////////////

                                                                        /////////////////////////////////////////////////
                                                                        //       Output to Top Scheduler               //
    output logic [$clog2(WRITEBUFFERDEPTH)- 1:0] WriteRequestWindow   // 1. Write Buffer Entry Count for Ch. MODE    //
                                                                        /////////////////////////////////////////////////
);

    //------------------------------------------------------------------------------
    //  Write Buffer Directory
    //
    //  - Tracks metadata for buffered write requests:
    //      * Address, ID, User, Strb
    //      * Issued / Valid state
    //  - One directory entry corresponds to one burst write.
    //  - Serves as control plane for write data buffer.
    //
    //------------------------------------------------------------------------------
    WriteBufferDirEntry writeBuffer_dirMEM [WRITEBUFFERDEPTH-1:0];        
    logic [$clog2(WRITEBUFFERDEPTH)-1:0] writeDirPtr;                       // Allocation pointer for directory        (Allocation signal comes from MCFrontEND)
    logic [$clog2(WRITEBUFFERDEPTH)-1:0] IssuedDirPtr;                      // Issue Update pointer for diretory       (Issue signal comes from RankFSM)
    logic [$clog2(WRITEBUFFERDEPTH)-1:0] readDirPtr;                        // Sending data pointer for directory/Data (Send Write Data for Issued & Valid data)
    logic [WRITEBUFFERDEPTH-1:0] writeDirFree;                              // Write Directory Free bit Vector for writeDirPtr 
    logic [WRITEBUFFERDEPTH-1:0] writeDirIssued;                            // Issued Request Tracking bit Vector for readDirPtr
    logic dir_full;                                                         // Directory Full 
    logic PHYServing;                                                       // it is only valid when there is Issued & Valid Request for write data.
    logic [$clog2(WRITEBUFFERDEPTH* BURST_LENGTH)-1:0] readDataPtr, writeDataPtr;      
    logic [$clog2(BURST_LENGTH)-1:0] readDataBurstCnt, writeDataBurstCnt;

    logic data_write;
    WriteBufferDataEntry readDataData, writeDataData;
    //--------------------------------------------------------------------//


    //------------------------------------------------------------------------------
    //  Write Request Window Counter
    //
    //  - Tracks number of outstanding write requests in buffer.
    //  - Used by top-level scheduler for channel mode decisions.
    //  - Incremented on write data enqueue.
    //  - Decremented when PHY completes write burst.
    //
    //------------------------------------------------------------------------------
    logic [$clog2(WRITEBUFFERDEPTH+1)-1:0] WriteRequestWindow_Cnt;
    always_ff@(posedge clk or negedge rst) begin
        if(!rst)begin
            WriteRequestWindow_Cnt <= '0;
        end else begin
            if(writeDataLast && !writeBufferFull && WriteModeACK) begin
                WriteRequestWindow_Cnt <= WriteRequestWindow_Cnt;
            end
            else if(writeDataLast && !writeBufferFull) begin
                WriteRequestWindow_Cnt <= WriteRequestWindow_Cnt + 1;
            end 
            else if(WriteModeACK) begin
                WriteRequestWindow_Cnt <= WriteRequestWindow_Cnt - 1;
            end
        end
    end

    assign WriteRequestWindow =  WriteRequestWindow_Cnt;


    //------------------------------------------------------------------------------
    //  Write Buffer Pointer Calculation
    //
    //  - writeDirPtr : Allocation pointer for new write requests.
    //  - readDirPtr  : Pointer for issued write request to be sent to PHY.
    //  - IssuedDirPtr: Matches write command ACK from RankFSM to directory entry.
    //
    //------------------------------------------------------------------------------
    //----------- Write Data/Directory Buffer Pointer Calculation --------//
    priority_ptr_WriteBuf #(
        .vector_length(WRITEBUFFERDEPTH),
        .vector_type(WriteBufferDirEntry)
    ) readDirPtrCalculator(
        .vector(writeBuffer_dirMEM), .index(readDirPtr)
    );

    PriorityEncoder_LSB #(
        .vector_length(WRITEBUFFERDEPTH)
    ) writeDirPtrCalculator (
        .vector(writeDirFree), .index(writeDirPtr)  
    );
    assign readDataPtr = readDirPtr * BURST_LENGTH + readDataBurstCnt;
    assign writeDataPtr = writeDirPtr * BURST_LENGTH + writeDataBurstCnt;
    //--------------------------------------------------------------------//

    //------------------------------------------------------------------------------
    //  Issued Write Request Matching
    //
    //  - Matches write command ACK (ID/User) from RankFSM.
    //  - Marks corresponding directory entry as issued.
    //  - Enables data streaming to PHY.
    //
    //------------------------------------------------------------------------------
    always_comb begin
        IssuedDirPtr = 0;
        for(int i = 0; i < WRITEBUFFERDEPTH; i++) begin
            if(writeBuffer_dirMEM[i].valid) begin
                if(writeBuffer_dirMEM[i].user == writeBufferAllocUser && writeBuffer_dirMEM[i].id == writeBufferAllocID)begin
                    IssuedDirPtr = i;
                end
            end 
        end
    end

    //------------------------------------------------------------------------------
    //  Write Buffer Directory Management
    //
    //  - Allocation   : On cache-side write data (last beat).
    //  - Issue update : On write command ACK from RankFSM.
    //  - Deallocation : After full burst is sent to PHY.
    //
    //------------------------------------------------------------------------------
    always_ff@(posedge clk or negedge rst)begin
        if(!rst) begin
            readDataStrb   <= 0;
            writeDirFree   <= '1;
            writeDirIssued <= '0;
            for(int i = 0; i < WRITEBUFFERDEPTH; i++) begin
                /* verilator lint_off BLKSEQ */
                writeBuffer_dirMEM[i] = '0;
            end
        end else begin
            if(writeBufferAllocValid) begin                         // Write Request Issued signals come from "RankFSM"            (UPDATE)
                if(writeBuffer_dirMEM[IssuedDirPtr].valid)begin
                    writeBuffer_dirMEM[IssuedDirPtr].issued <= 1;
                    writeDirIssued[IssuedDirPtr] <= 1;
                    `ifdef DISPLAY
                        $display("[%0t] WriteBufferController | WRITE BUFFER ALLOC | ID: %d | USER: %d", $time
                                ,writeBufferAllocID, writeBufferAllocUser);
                    `endif
                end
            end
            if(writeDataValid && !writeBufferFull) begin            // Write Directory Allocation, signals come from "MCFrontEnd"  (PUSH)
                if(writeDataLast) begin
                    writeDirFree[writeDirPtr] <= 0;
                    writeBuffer_dirMEM[writeDirPtr].addr <= writeDataAddr;
                    writeBuffer_dirMEM[writeDirPtr].id  <= writeDataID;
                    writeBuffer_dirMEM[writeDirPtr].user <= writeDataUser;
                    writeBuffer_dirMEM[writeDirPtr].ptr <= writeDirPtr;
                    writeBuffer_dirMEM[writeDirPtr].strb <= writeDataStrb;
                    writeBuffer_dirMEM[writeDirPtr].issued <= 0;
                    writeBuffer_dirMEM[writeDirPtr].valid <= 1;
                    `ifdef DISPLAY
                        $display("[%0t] WriteBufferController | WRITE BUFFER RECEIVING | ID: %d | USER: %d", $time
                                ,writeDataID, writeDataUser);
                    `endif
                end
            end
            if(PHYServing) begin                                    // Write Directory Serving, signals go to "PHY-side"            (POP)
                readDataStrb <= writeBuffer_dirMEM[readDirPtr].strb[readDataBurstCnt];
                if(readDataBurstCnt == BURST_LENGTH-1) begin
                    writeDirFree[readDirPtr] <= 1;
                    writeDirIssued[readDirPtr] <= 0;
                    writeBuffer_dirMEM[readDirPtr].valid <= 0;
                    writeBuffer_dirMEM[readDirPtr].issued <= 0;
                    `ifdef DISPLAY
                        $display("[%0t] WriteBufferController | WRITE BUFFER SERVING | ID: %d | USER: %d", $time
                                ,writeBuffer_dirMEM[readDirPtr].id, writeBuffer_dirMEM[readDirPtr].user);
                    `endif
                end
            end
        end
    end



    //-------------- Write Buffer ACK for Auto-Precharge -----------------//
    assign data_write = writeDataValid && !writeBufferFull;          // Write Data Allocation, data comes from "MCFrontEnd"       (PUSH)
    assign PHYServing = |writeDirIssued && phyWriteModeFIFOReady;    // Read Data from Write Data Buffer, data goes to "PHY-side" (POP)
    

    //------------------------------------------------------------------------------
    //  Write Data Buffer (Burst-Level)
    //
    //  - Stores burst write data from cache-side.
    //  - Addressed as (directory index × BURST_LENGTH).
    //  - Dual-port access:
    //      * Write : Cache-side data enqueue
    //      * Read  : PHY-side data streaming
    //
    //------------------------------------------------------------------------------
    DualPortBuffer #(
        .BufferDepth(WRITEBUFFERDEPTH * BURST_LENGTH),
        .DataEntry(WriteBufferDataEntry)
    ) writeBuffer_dataMEM (.clk(clk), .rst(rst),
    .re(PHYServing), .we(data_write), .readPtr(readDataPtr), .writePtr(writeDataPtr),
    .wdata(writeDataData), .rdata(readDataData));

    always_comb begin
        for(int i = 0; i < NUMRANK; i++) begin
            writeBufferAvailable[i] = !dir_full;
        end
    end
    assign dir_full = !(|writeDirFree);
    assign writeBufferFull = dir_full;

    //------------ Read Data Management for Write Data Buffer -----------//
    always_ff@(posedge clk or negedge rst)begin
        if(!rst) begin
            readDataValid <= 0;
            readDataBurstCnt <= 0;
            writeDataBurstCnt <=0;
        end else begin
            if(writeDataValid && !writeBufferFull) begin
                writeDataBurstCnt <= writeDataBurstCnt + 1;
                if(writeDataLast) begin
                    writeDataBurstCnt <= 0;
                end
            end
            if(PHYServing) begin
                readDataValid <= 1;
                if(readDataBurstCnt == BURST_LENGTH-1) begin
                    readDataBurstCnt <= 0;
                    readDataLast <= 1;
                end
                else begin
                    readDataBurstCnt <= readDataBurstCnt + 1;
                    readDataLast <= 0;
                end
            end else begin
                readDataValid <= 0;
                readDataLast  <= 0;
            end
        end
    end 

    assign writeDataData = writeData;
    assign readData      = readDataData;

    //--------------------------------------------------------------------//

    //------------------------------------------------------------------------------
    //  Write Response Generation
    //
    //  - Generates write response when:
    //      * Last write data beat accepted, and
    //      * Frontend ready for response.
    //  - Passes through ID and User fields.
    //
    //------------------------------------------------------------------------------
    assign writeDataACKValid = writeDataValid && writeDataLast && writeDataACKReady;
    assign writeDataACKUser = writeDataUser;
    assign writeDataACKID = writeDataID;
endmodule
