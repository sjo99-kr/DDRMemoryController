`timescale 1ns / 1ps

//------------------------------------------------------------------------------
//      ReadBufferController
//
//      Role:
//          Read data buffer between DDR PHY and Memory controller frontend.
//
//      Responsibilities:
//          - Allocates read buffer entries for incoming read commands.
//          - Matches incoming DQ data to outstanding read requests.
//          - Buffers burst data and serves it to cache in-order.
//          - Tracks per-rank outstanding reads using request windows.
//
//      Scope & Notes:
//          - Handles data-level ordering (burst granularity).
//          - No DRAM timing logic (handled by RankExecutionUnit / PHY).
//          - Uses tag-based matching for PHY read responses.
//
//      Author  : Seongwon Jo
//      Created : 2026.02
//------------------------------------------------------------------------------

module ReadBufferController#(
    parameter int BUFFER_CHANNEL        = 0,

    parameter int MEM_DATAWIDTH         = 64,
    parameter int MEM_ADDRWIDTH         = 32,
    parameter int MEM_IDWIDTH           = 4,
    parameter int MEM_USERWIDTH         = 1,
    parameter int NUMRANK               = 4,
    parameter int READBUFFERDEPTH       = 128,
    parameter int BURST_LENGTH          = 8,

    parameter type ReadBufferDataEntry  = logic,
    parameter type ReadBufferDirEntry   = logic,    
    parameter type MemoryAddress        = logic
)(
    input logic clk, rst,
                                                            ////////////////////////////////////////////////
                                                            //        Input from MC FrontEnd (Cache-side) //
    input logic readDataReady,                              //  1. Read data Ready Signal from LLC        //
                                                            ////////////////////////////////////////////////

                                                            ////////////////////////////////////////////////
                                                            //    Output to MC FrontEnd   (Cache-side)    //
    output logic [MEM_DATAWIDTH - 1:0] readData,            //  1. Read Data to MC FrontEnd               //
    output logic [MEM_USERWIDTH-1:0] readDataUser,          //  2. User of Read data to MC FrontEnd       //
    output logic [MEM_IDWIDTH-1:0] readDataID,              //  3. ID of Read data to MC FrontEnd         //
    output logic readDataLast,                              //  4. Last signal for Read Data              //
    output logic readDataValid,                             //  5. Valid signal for Read data             //
    output logic readBufferFull,                            //  6. Ready signal for Read Buffer           //
                                                            ////////////////////////////////////////////////

                                                            ////////////////////////////////////////////////
                                                            //        Input from DQ-BUS (PHY-side)        // 
    input logic [MEM_DATAWIDTH-1:0] writeData,              //  1. Write Data from DQ-Bus (64-B)          //
    input logic [NUMRANK-1:0] writeDataTag,                 //  2. Write Data Tag from PHY-side           //
    input logic writeDataValid,                             //  3. Write Data Valid from PHY-side         //
    input logic writeDataLast,                              //  4. Write Data Last from PHY-side          //
    input logic phyReadModeFIFOReady,                       //  5. PHYReadMode FIFO Ready from PHY-side   //
                                                            ////////////////////////////////////////////////

                                                            ////////////////////////////////////////////////
                                                            //          Input from Channel Scheduler      //
    input logic [MEM_IDWIDTH-1:0] readBufferAllocID,        //  1. Read Buffer Entry Allocation ID        //
    input logic [MEM_USERWIDTH-1:0] readBufferAllocUser,    //  2. Read Buffer Entry Allocation User      //
    input MemoryAddress readBufferAllocAddr,                //  3. Read Buffer Entry Allocation Addr      //
    input logic readBufferAllocValid,                       //  4. Read Buffer Entry Allocation Valid     //
                                                            ////////////////////////////////////////////////

                                                            /////////////////////////////////////////////////
                                                            //       Output to Channel Scheduler           //
    output logic readBufferAvailable,                       //  1. Read Buffer Availability (Dir/Data Buf) //
    output logic [NUMRANK-1:0] readRankWindowAvailable,
                                                            /////////////////////////////////////////////////

                                                                    /////////////////////////////////////////////////
                                                                    //       Output to Top Scheduler               //
    output logic [$clog2(READBUFFERDEPTH) - 1:0] readBufferCnt      // 1. Read Buffer Entry Count for Ch. MODE     //
                                                                    /////////////////////////////////////////////////
);

    //------------------------------------------------------------------------------
    //  Outstanding Read Request Window (Per-Rank)
    //
    //  - Tracks up to two outstanding read requests per rank.
    //  - Used to match incoming PHY read data without address info.
    //  - Maintains head pointer to preserve request ordering.
    //
    //------------------------------------------------------------------------------
                                                                        ///////////////////////////////////////////////////////////////////
    typedef struct packed {                                             //       Read Request Window : 2-entry Request Window            //
        logic [$clog2(READBUFFERDEPTH)-1:0] ptr_1;                      //      1. Entry 1 Pointer for per-Rank Request Window           // 
        logic [$clog2(READBUFFERDEPTH)-1:0] ptr_2;                      //      2. Entry 2 Pointer for per-Rank Request Window           //
        logic [1:0] received;                                           //      3. Received Tracking Signal, Valid when data received    //
        logic [1:0] valid;                                              //      4. Allocation Entry ptr Signal, Valid when ptr is valid  //
        logic head;                                                     //      5. Head pointer for Entry, (Finding oldest request)      //
    } OutstandingReadEntry;                                             ///////////////////////////////////////////////////////////////////
    
    OutstandingReadEntry ReadRequestWindow [NUMRANK-1:0];         
                                                                        //////////////////////////////////////////////////////////////////////////////
                                                                        //                  Read Request Window For Read Data Buffer                //
                                                                        //      Read request window for outstanding read requests.                  //
                                                                        //      DRAM read responses do not include explicit address information.    //
                                                                        //      Therefore, the memory controller matches incoming data to requests  //
                                                                        //      using PHY tags and per-rank outstanding read windows.               //
                                                                        //////////////////////////////////////////////////////////////////////////////


    //------------------------------------------------------------------------------
    //  Read Buffer Pointer & Free Vector Management
    //
    //  - Maintains allocation and serving pointers for directory/data buffers.
    //  - Separates cache-side (read) and PHY-side (write) access.
    //  - Uses free/received vectors to determine buffer availability.
    //
    //------------------------------------------------------------------------------
    logic [$clog2(READBUFFERDEPTH)-1:0] readDirPtr, readDataVecPtr;     //  readDirPtr  (For Serving Process with Cache-side)
    logic [$clog2(READBUFFERDEPTH)-1:0] writeDirPtr, writeDataVecPtr;   //  writeDirPtr (For Receiving Proccess with PHY-side)
    logic [READBUFFERDEPTH-1:0] readDirFree;                            
    logic [READBUFFERDEPTH-1:0] readDirReceived;   
    logic [READBUFFERDEPTH-1:0] readDataFree;                    
    ReadBufferDirEntry readDirData, writeDirData;                       //  ReadDirData/WriteDirData : 1) Request Address; 2) Request ID; 3) Request User;
    logic dir_read, dir_write;
    //-------------------------------------------------------------------//

    //-------------------- DATA Buffer-related definition -----------------//
    logic [$clog2(READBUFFERDEPTH * BURST_LENGTH)-1:0] readDataPtr;
    logic [$clog2(READBUFFERDEPTH * BURST_LENGTH)-1:0] writeDataPtr;
    ReadBufferDataEntry readDataData, writeDataData;
    
    logic [$clog2(BURST_LENGTH)-1:0] readDataBurstCnt, writeDataBurstCnt;
    //-------------------------------------------------------------------//
    logic cacheServing;                                                 // signal of cache serving Data for Read Directory and Read Data Buffer
    logic imme_status;

    assign cacheServing = readDataReady && |(~readDirFree & readDirReceived);


    logic [NUMRANK-1:0] WindowValidVector;

    always_comb begin
        for (int i = 0; i < NUMRANK; i++) begin
            WindowValidVector[i] = !(&ReadRequestWindow[i].valid);
        end
    end
    
    assign readBufferAvailable  = |WindowValidVector && phyReadModeFIFOReady;
    assign readRankWindowAvailable = WindowValidVector;
    assign readBufferFull =  !(|readDirFree);;

    
    //-------------------------------------------------------------------//

    //---------- Calculating Read Pointer (Cache) for Dir Buffer --------//
    PriorityEncoder_LSB #(                                                     // Calculating the Read Directory Pointer based on ~DirFree Vector and DirReceived Vector
        .vector_length(READBUFFERDEPTH)
    ) readDirPtrCalculator(
        .vector(~readDirFree & readDirReceived), .index(readDirPtr));  

    PriorityEncoder_LSB #(
        .vector_length(READBUFFERDEPTH)
    )  readDataPtrCalculator(
        .vector(~readDataFree), .index(readDataVecPtr)
    );
    //-------------------------------------------------------------------//

    //----- Calculating Write Pointer (Ch. Sched.) for Dir Buffer -------//
    PriorityEncoder_LSB #(                                                // Calculating the Write Directory Pointer based on DirFree Vector.
        .vector_length(READBUFFERDEPTH)
    ) writeDirPtrCalculator (
        .vector(readDirFree), .index(writeDirPtr));

    PriorityEncoder_LSB #(
        .vector_length(READBUFFERDEPTH)
    ) writeDataPtrCalculator (
        .vector(readDataFree), .index(writeDataVecPtr)  
    );
    //-------------------------------------------------------------------//

    //------------------ Read Directory Buffer setup --------------------//
    DualPortBuffer #(                                                    // Dual-port Buffer for Directory 
        .BufferDepth(READBUFFERDEPTH),                                   // Independent Read and Write port for Cache-Side / Allocation-Side
        .DataEntry(ReadBufferDirEntry)                                   // readDirPtr and writeDirPtr can not be same, avoding DATA Hazard in Dual-port Buffer
    ) readBuffer_dirMEM (.clk(clk), .rst(rst),                          
        .re(cacheServing), .we(dir_write), .readPtr(readDirPtr), .writePtr(writeDirPtr), .wdata(writeDirData), .rdata(readDirData));

    assign readDataID = readDirData.id;
    assign readDataUser = readDirData.user;

    assign dir_write = readBufferAllocValid;
    assign writeDirData.addr  = readBufferAllocAddr;
    assign writeDirData.id    = readBufferAllocID;
    assign writeDirData.user  = readBufferAllocUser;
    //-------------------------------------------------------------------//


    //////////////////////////////////////////////////////////////////////
    //              Read Directory Buffer Read/Write Case               //
    //      Read Case                                                   //
    //         - Read Buffer-driven for serving data to Cache           //
    //      Write Case                                                  //
    //         - Allocation-driven by Channel Scheduler                 //
    //////////////////////////////////////////////////////////////////////

    //------------------------------------------------------------------------------
    //  Read Directory Allocation & Free Management
    //
    //  - Allocates directory entry on read command issue.
    //  - Frees entry after full burst is served to cache.
    //  - Drives readBufferFull signal.
    //
    //------------------------------------------------------------------------------
    always_ff@(posedge clk or negedge rst)begin
        if(!rst) begin
            readDirFree <= '1;
        end else begin
            if(readBufferAllocValid)  begin
                readDirFree[writeDirPtr] <= 0;
                `ifdef DISPLAY
                    $display("[%0t] ReadBufferController | READ BUFFER ALLOC | ID: %d | USER: %d", $time
                        ,readBufferAllocID, readBufferAllocUser);
                `endif
            end  
            if(cacheServing && (readDataBurstCnt == BURST_LENGTH-1)) begin
                readDirFree[readDirPtr] <= 1;
                `ifdef DISPLAY
                    $display("[%0t] ReadBufferController | READ BUFFER SERVING | ID: %d | USER: %d", $time
                        ,readDataID, readDataUser);
                `endif
            end 
        end
    end
    //-------------------------------------------------------------------//

    //------------------------------------------------------------------------------
    //  Outstanding Read Tracking & Tag Matching
    //
    //  - Matches incoming PHY data using rank tag.
    //  - Marks request as received when last burst arrives.
    //  - Clears window entry after cache consumes data.
    //
    //------------------------------------------------------------------------------
    always_ff@(posedge clk or negedge rst) begin
        if(!rst)begin
            readDirReceived <= '0;
            for(int i =0; i < NUMRANK; i++) begin
                ReadRequestWindow[i] <= '0;                                  // ReadRequestWindow -> 2-Entry per Rank
            end
        end else begin  
            //                  Request Entry Received Set up               //
            //   Allocated Entry with WAIT state turns on Received state    //
            if(writeDataValid) begin                                        
                if(writeDataLast)begin
                    `ifdef DISPLAY
                        $display("[%0t] ReadBufferController | READ BUFFER RECEIVED | Data Tag : %d", $time, writeDataTag);
                    `endif
                    if(ReadRequestWindow[writeDataTag].head) begin                  //  WriteDataTag has information for which rank sends data.
                        if(!ReadRequestWindow[writeDataTag].received[1]) begin
                            ReadRequestWindow[writeDataTag].received[1] <= 1;
                            readDirReceived[ReadRequestWindow[writeDataTag].ptr_2] <= 1;
                        end else begin
                            if(ReadRequestWindow[writeDataTag].valid[0]) begin
                                ReadRequestWindow[writeDataTag].received[0] <= 1;
                                readDirReceived[ReadRequestWindow[writeDataTag].ptr_1] <= 1;
                            end
                        end
                    end else begin
                        if(!ReadRequestWindow[writeDataTag].received[0]) begin
                            ReadRequestWindow[writeDataTag].received[0] <= 1;
                            readDirReceived[ReadRequestWindow[writeDataTag].ptr_1] <= 1;
                        end else begin
                            if(ReadRequestWindow[writeDataTag].valid[1]) begin
                                ReadRequestWindow[writeDataTag].received[1] <= 1;
                                readDirReceived[ReadRequestWindow[writeDataTag].ptr_2] <= 1;
                            end
                        end
                    end
                end
            end
            //  Allocation phase with WAIT state in ReadRequest Window      //
            if(readBufferAllocValid) begin
                if(!(&ReadRequestWindow[readBufferAllocAddr.rank].valid)) begin
                    if(!ReadRequestWindow[readBufferAllocAddr.rank].valid[0]) begin
                        ReadRequestWindow[readBufferAllocAddr.rank].ptr_1 <= writeDirPtr;
                        ReadRequestWindow[readBufferAllocAddr.rank].valid[0] <= 1;

                        if(ReadRequestWindow[readBufferAllocAddr.rank].valid[1])begin
                            ReadRequestWindow[readBufferAllocAddr.rank].head <= 1;
                        end
                    end
                    else if(!ReadRequestWindow[readBufferAllocAddr.rank].valid[1]) begin
                        ReadRequestWindow[readBufferAllocAddr.rank].ptr_2 <= writeDirPtr;
                        ReadRequestWindow[readBufferAllocAddr.rank].valid[1] <= 1;
                        if(ReadRequestWindow[readBufferAllocAddr.rank].valid[0]) begin
                            if(cacheServing && readDataBurstCnt == BURST_LENGTH-1) begin
                                ReadRequestWindow[readBufferAllocAddr.rank].head <= 1;
                            end
                        end
                    end
                end
            end
            if(cacheServing) begin
                if(readDataBurstCnt == BURST_LENGTH-1) begin
                    readDirReceived[readDirPtr] <= 0;
                    if (ReadRequestWindow[readDirData.addr.rank].ptr_1 == readDirPtr) begin
                        ReadRequestWindow[readDirData.addr.rank].valid[0] <= 0;
                        ReadRequestWindow[readDirData.addr.rank].received[0] <= 0;
                        if(ReadRequestWindow[readDirData.addr.rank].valid[1]) begin
                            ReadRequestWindow[readDirData.addr.rank].head <= 1;
                        end
                    end else begin
                        if(ReadRequestWindow[readDirData.addr.rank].ptr_2 == readDirPtr) begin
                            ReadRequestWindow[readDirData.addr.rank].valid[1] <= 0;
                            ReadRequestWindow[readDirData.addr.rank].received[1] <= 0;
                            ReadRequestWindow[readDirData.addr.rank].head <= 0;
                        end
                    end
                end
            end
        end
    end

    //------------------------------------------------------------------------------
    //  Read Buffer Occupancy Counter
    //
    //  - Counts number of active read buffer entries.
    //  - Exported to top-level scheduler for channel mode decisions.
    //
    //------------------------------------------------------------------------------
    logic [$clog2(READBUFFERDEPTH) - 1 :0] readBufCount;
    always_comb begin
        readBufCount = 0;
        for(int i = 0; i < READBUFFERDEPTH; i++) begin
            if(!readDirFree[i]) begin
                readBufCount = readBufCount + 1;
            end
        end 
    end
    assign readBufferCnt = readBufCount;




    assign readDataPtr  = BURST_LENGTH * readDataVecPtr + readDataBurstCnt;              // Burst-level Addressing for Read Pointer
    assign writeDataPtr = BURST_LENGTH * writeDataVecPtr + writeDataBurstCnt;            // Burst-level Addressing for Write Pointer
    assign writeDataData  = writeData;
    
    assign readData = readDataData;

    //------------------------------------------------------------------------------
    //  Read Data Buffer (Burst-Level)
    //
    //  - Stores burst data from PHY.
    //  - Burst-addressed using (entry index × BURST_LENGTH).
    //  - Dual-port access:
    //      * Write : PHY-side data arrival
    //      * Read  : Cache-side consumption
    //
    //------------------------------------------------------------------------------
    DualPortBuffer #(
        .BufferDepth(READBUFFERDEPTH * BURST_LENGTH),
        .DataEntry(ReadBufferDataEntry)                 // data size : 64-bits
    ) readBuffer_dataMEM (.clk(clk), .rst(rst),
        .re(cacheServing), .we(writeDataValid), .readPtr(readDataPtr), .writePtr(writeDataPtr), .wdata(writeDataData), .rdata(readDataData));





    //------------------------------------------------------------------------------
    //  Burst Counter & Data Serving Control
    //
    //  - Tracks burst position for both PHY write and cache read.
    //  - Generates readDataValid / readDataLast signals.
    //  - Frees data buffer entry after full burst is consumed.
    //
    //------------------------------------------------------------------------------
    always_ff@(posedge clk or negedge rst) begin
        if(!rst) begin
            readDataFree <= '1;
            readDataBurstCnt  <= 0;
            writeDataBurstCnt <= 0;
        end else begin
            if(writeDataValid) begin
                writeDataBurstCnt <= writeDataBurstCnt + 1;
                if(writeDataLast)begin
                    readDataFree[writeDataVecPtr] <= 0;
                    writeDataBurstCnt <= 0;
                end
            end
            if(cacheServing) begin
                readDataBurstCnt <= readDataBurstCnt  + 1;
                readDataValid <= 1;
                readDataLast  <= 0;
                if(readDataBurstCnt == BURST_LENGTH-1) begin
                    readDataBurstCnt <= 0;
                    readDataFree[readDataVecPtr] <= 1;
                    readDataLast <= 1;
                end 
            end else begin
                readDataValid <= 0;
                readDataBurstCnt <= 0;
                readDataLast <= 0;
            end
        end
    end

endmodule
