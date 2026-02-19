`timescale 1ns / 1ps

//------------------------------------------------------------------------------
//      PHYController
//
//      Role:
//          Behavioral PHY-layer controller inside the Memory Controller.
//          This module bridges logical DRAM commands/data and rank-level
//          DDR4 DQ/DQS signaling.
//
//      Architecture Overview:
//
//          MemoryControllerBackend
//                  ∧       |
//                  |       V
//            +------------------+
//            |  PHYController   |   <-- This module
//            +------------------+
//               ∧          |
//               |          V
//        PHYReadMode   PHYWriteMode
//               ∧          |
//               |          V
//              DDR4 IF DQ BUS
//
//      Responsibilities:
//          - Manage READ / WRITE data path at PHY abstraction level.
//          - Enforce burst-level timing constraints (tCL, tCWL).
//          - Handle DQ/DQS direction control and tri-state isolation.
//          - Track in-flight READ/WRITE data inside PHY FIFOs.
//          - Provide FIFO backpressure signals to buffers and scheduler.
//          - Generate Auto-Precharge ACKs for RankFSM synchronization.
//          - Safely indicate channel mode transition readiness.
//
//      What this module DOES:
//          - Cycle-accurate modeling of DDR4 DQ behavior.
//          - Burst-length-aware data movement.
//          - Timing-aware coordination between command issue and data transfer.
//
//      What this module DOES NOT do:
//          - No DRAM command scheduling or arbitration.
//          - No bank/rank timing decision logic.
//          - No electrical-level PHY modeling.
//
//      Notes:
//          - All DDR DQ/DQS abstraction is isolated in this module.
//          - ChannelController controls when commands are issued.
//          - This module reacts to issued commands and manages data timing.
//
//      Author  : Seongwon Jo
//      Created : 2026.02
//------------------------------------------------------------------------------

module PHYController #(
    parameter int PHY_CHANNEL = 0,
    parameter int MEM_DATAWIDTH = 64,
    parameter int NUMRANK = 4,
    parameter int BURST_LENGTH = 8,
    parameter int PHYFIFOMAXENTRY = 4,
    parameter int PHYFIFOREQUESTWINDOW = 8,
    parameter int PHYFIFODEPTH = 32,
    parameter int tCL = 16,
    parameter int tCWL = 12,
    parameter type MemoryAddress = logic
)(
        // common
    input logic clk, rst, mode,                  
    input logic clk2x,                                                          //  DDR Clocking for generating DQS_t signal
           
                                                                                ////////////////////////////////////////////////////////
                                                                                //          INPUT FROM  WRITE BUFFER SIDE             // 
    input logic [MEM_DATAWIDTH-1:0] writeBufferData,                            //  1. Data from Write Buffer        (To WriteMode)   // 
    input logic writeBufferDataLast,                                            //  2. Data Last from Write Buffer   (To WriteMode)   //            
    input logic writeBufferDataValid,                                           //  3. Data Valid from Write Buffer  (To WriteMode)   //
    input logic [MEM_DATAWIDTH/BURST_LENGTH - 1 : 0] writeBufferDataStrb,       //  4. Masking Bit for Data          (To WriteMode)   //
                                                                                ////////////////////////////////////////////////////////

                                                                                ////////////////////////////////////////////////////////
                                                                                //          OUTPUT TO WRITE BUFFER SIDE               //
    output logic WriteModeFIFOReady,                                            //  1. Ready Signal from PHYWriteModeFIFO             //
    output logic WriteModeACK,                                                  //  2. ACK Signal from PHY WriteMode to WR BUFFER     //
                                                                                ////////////////////////////////////////////////////////

                                                                                ////////////////////////////////////////////////////////
                                                                                //          OUTPUT TO  RAED BUFFER SIDE               //
    output logic [MEM_DATAWIDTH-1:0] readBufferData,                            //  1. Data to Read Buffer        (From ReadMode)     //
    output logic [NUMRANK-1:0] readBufferDataTag,                               //  2. Data Tag to Read Buffer                        //
    output logic readBufferDataValid,                                           //  3. Data Valid to Read Buffer  (From ReadMode)     //
    output logic readBufferDataLast,                                            //  4. Data Last to Read Buffer   (From ReadMode)     //
    output logic ReadModeFIFOReady,                                             //  5. Ready Signal from PHYReadModeFIFO              //
                                                                                ////////////////////////////////////////////////////////

                                                                                ////////////////////////////////////////////////////////
                                                                                //          INPUT FROM CHANNEL SCHEDULER              //
    input logic ReadCMDIssuedACK,                                               //  1. READ Command Issued ACK                        //
    input MemoryAddress ReadCMDIssuedAddr,                                      //  2. READ Command Issued Address                    //
    input logic WriteCMDIssuedACK,                                              //  3. WRITE Command Issued ACK                       //
    input MemoryAddress WriteCMDIssuedAddr,                                     //  4. WRITE Command Issued Address                   //
                                                                                ////////////////////////////////////////////////////////

                                                                                ////////////////////////////////////////////////////////
                                                                                //          OUTPUT TO CHANNEL SCHEDULER               //
    output logic ReadAutoPrechargeACK,                                          //  1. ReadCMD+AP Acknowledge to RankFSM              //
    output MemoryAddress ReadAutoPrechargeAddr,                                 //  2. ReadCMD+AP ACK Address to RankFSM              //
    output logic WriteAutoPrechargeACK,                                         //  3. WriteCMD+AP Acknowledge to RankFSM             //
    output MemoryAddress WriteAutoPrechargeAddr,                                //  4. WriteCMD+AP ACK Address to RankFSM             // 
                                                                                ////////////////////////////////////////////////////////

                                                                                ////////////////////////////////////////////////////////
                                                                                //           OUTPUT TO MemoryController BackEnd       //
    output logic ModeTransitionReady,                                           //  1. Channel Mode Transition Ready                  //
                                                                                ////////////////////////////////////////////////////////

                                                                                ////////////////////////////////////////////////////////
                                                                                //          INPUT/OUTPUT FROM/TO DRAM-SIDE            //
    DDR4Interface ddr4_dataBus                                            // 1. INPUT/OUTPUT FOR DQ BUS                         //
                                                                                ////////////////////////////////////////////////////////
);


    //              From/To ReadMode                //
    logic readInFlag, readOutFlag;                          //  ReadInFlag : DRAM -> PHY, ReadOutFlag : PHY -> READ BUFFER
    logic [$clog2(BURST_LENGTH)-1:0] readBurstInCnt;        //  Cnt. of read Burst Data (PHY <- DRAM-SIDE )
    logic readReceivingACK;                                 // Only ACK when BURST_LENGTH of read data comes to READ Mode PHY.
    logic [$clog2(PHYFIFOMAXENTRY)-1:0] InflightReadModeReq;   // Num. of Read data stay in ReadMode FIFO
    logic [tCL-2:0] ReadRequestLatency;                     //  tCL timing counter for each Issued Read Request
    //////////////////////////////////////////////////

    //           From/To WriteMode                  //
    logic writeOutFlag;                                         //  WriteOutFlag  :  PHY -> DRAM 
    logic writeServingACK;                                      //  ACK signal from WriteModePHY
    logic [$clog2(PHYFIFOMAXENTRY)-1:0] TimedWriteReqCnt;          //  Num. of Write Data passed "tCWL" timing constratins in WriteMode FIFO
    logic [$clog2(PHYFIFOMAXENTRY)-1:0] InflightWriteModeReq;      //  Num. of Write Data stay in WriteMode FIFO.
    logic [tCWL-2:0] WriteRequestLatency;                       //  tCWL timing Counter for each Issued Write Request
    //////////////////////////////////////////////////
    assign WriteModeACK = writeServingACK;
            

    logic ReadModeDqs_t, ReadModeDqs_c;
    logic WriteModeDqs_t, WriteModeDqs_c, WriteModeDQS;
    logic [MEM_DATAWIDTH-1:0] ReadModeData, WriteModeData;

    assign ReadModeDqs_t = (readInFlag) ? ddr4_dataBus.dqs_t  : 'z;
    assign ReadModeDqs_c = (readInFlag) ? ddr4_dataBus.dqs_c  : 'z;
    assign ReadModeData  = (readInFlag) ? ddr4_dataBus.pin_dq : 'z;


    //              PHY READ / WRITE MODE  Instance         //
    PHYReadMode #(
        .PHY_CHANNEL(PHY_CHANNEL),
        .MEM_DATAWIDTH(MEM_DATAWIDTH),
        .PHYFIFODEPTH(PHYFIFODEPTH),
        .PHYFIFOMAXENTRY(PHYFIFOMAXENTRY),
        .BURST_LENGTH(BURST_LENGTH)
    ) PHYReadMode_Instance  (                   // Physical Read Mode Instance
        .clk(clk), .rst(rst), .clk2x(clk2x),
        .dqs_t(ReadModeDqs_t), .dqs_c(ReadModeDqs_c),
        .inData(ReadModeData),

        .inflag(readInFlag), 
        .outflag(readOutFlag),

        .readDataACK(readReceivingACK),

        .outData(readBufferData), 
        .outDataValid(readBufferDataValid), 
        .outDataLast(readBufferDataLast)
    );
    
    PHYWriteMode #(
        .PHY_CHANNEL(PHY_CHANNEL),
        .MEM_DATAWIDTH(MEM_DATAWIDTH),
        .PHYFIFODEPTH(PHYFIFODEPTH),
        .BURST_LENGTH(BURST_LENGTH)
    ) PHYWriteMode_Instance (                 // Physical Write Mode Instance
        .clk(clk), .rst(rst), 

        .dqs_t(WriteModeDqs_t), .dqs_c(WriteModeDqs_c), 
        .outdata(WriteModeData),
        .outDM(ddr4_dataBus.dm_n),

        .clk2x(clk2x), 
        .inflag(writeBufferDataValid),
        .inData(writeBufferData),
        .inStrb(writeBufferDataStrb),
        .outflag(writeOutFlag), 
        
        .outACK(writeServingACK),
        .WriteModeDQSValid(WriteModeDQS)
    );
    `ifdef VERILATOR
        assign ddr4_dataBus.dqs_t  = (readInFlag == 1) ? '0 :  (writeOutFlag) ? WriteModeDqs_t : '0;
    `else
        assign ddr4_dataBus.dqs_t  = (readInFlag == 1) ? 'z :  (writeOutFlag) ? WriteModeDqs_t : 'z;
    `endif

    `ifdef VERILATOR
        assign ddr4_dataBus.dqs_c  = (readInFlag == 1) ? '0 :  (writeOutFlag) ? WriteModeDqs_c : '0;
    `else
        assign ddr4_dataBus.dqs_c  = (readInFlag == 1) ? 'z :  (writeOutFlag) ? WriteModeDqs_c : 'z;
    `endif

    `ifdef VERILATOR
        assign ddr4_dataBus.pin_dq = (readInFlag == 1) ? '0 :  (writeOutFlag) ? WriteModeData  : '0;
    `else
        assign ddr4_dataBus.pin_dq = (readInFlag == 1) ? 'z :  (writeOutFlag) ? WriteModeData  : 'z;
    `endif

    // Channel Mode Transition is not available if there is any data on processing between PHY-side and DRAM-side.
    assign ModeTransitionReady = (!readInFlag) && (!writeOutFlag) && !(|WriteRequestLatency) && !(|ReadRequestLatency);

    // ReadMode/WriteModeFIFOReady manages RankFSM/RankSched for avoiding Buffer Spilling Out.
    assign ReadModeFIFOReady  = (InflightReadModeReq == PHYFIFOMAXENTRY-1)  ? 0 : 1;   // InflightReadModeReq reflects the number of current Read Data IN ReadMode FIFO
    assign WriteModeFIFOReady = (InflightWriteModeReq == PHYFIFOMAXENTRY-1) ? 0 : 1;   // Issued
    logic [$clog2(PHYFIFOREQUESTWINDOW)-1:0] ReadReqRdPtr, ReadReqWrPtr;
    logic [$clog2(PHYFIFOREQUESTWINDOW)-1:0] WriteReqRdPtr, WriteReqWrPtr;
    //                       READ DATA BUS MANAGEMENT                       //
    always_ff@(posedge clk or negedge rst) begin : tCLTimingCounter
        if(!rst) begin
            ReadRequestLatency <= '0;
        end else begin
            if(ReadCMDIssuedACK) begin
                ReadRequestLatency <= {{ReadRequestLatency[tCL-3 : 0]}, 1'b1};
            end else begin
                ReadRequestLatency <= {{ReadRequestLatency[tCL-3 : 0]}, 1'b0};
            end
        end
    end : tCLTimingCounter

    always_ff@(posedge clk or negedge rst) begin : tCWLTimingCounter
        if(!rst) begin
            WriteRequestLatency <= '0;
        end else begin
            if(WriteCMDIssuedACK) begin
                WriteRequestLatency <= {{WriteRequestLatency[tCWL-3:0]}, 1'b1};
            end else begin
                WriteRequestLatency <= {{WriteRequestLatency[tCWL-3:0], 1'b0}};
            end        
        end
    end : tCWLTimingCounter
    

    //                      Inflight Write Mode Request Tracker                      //
    //      Condition of Increment : Data   (Write Buffer -> WriteMode PHY FIFO)     //
    //      Condition of Decrement : Data   (WriteMode PHY FIFO -> DRAM)             //
    always_ff@(posedge clk or negedge rst) begin : InflightWriteModeReqTracker
        if(!rst) begin
            InflightWriteModeReq <= 0;
        end else begin
            if(writeBufferDataLast && writeServingACK) begin
                InflightWriteModeReq <= InflightWriteModeReq;
            end else if(writeServingACK) begin
                InflightWriteModeReq <= InflightWriteModeReq - 1;
            end else if(writeBufferDataLast) begin
                InflightWriteModeReq <= InflightWriteModeReq + 1;
            end  else begin
                InflightWriteModeReq <= InflightWriteModeReq;
            end
        end
    end : InflightWriteModeReqTracker


    //                      Timing WriteMode Requset Tracker                               //
    //  Write Buffer sends data when WRITE CMD ISSUED TO CMB/ADDR BUS.                     //
    //  HOWEVER, For DQ-BUS perspective, WE NEED TO CONSIDER FOR tCWL Timing constraints.  //
    //  So, We use double trackers one for avoiding FIFO spilling out (InflightWRModeReq), //
    // One for reflecting tCWL timing constraints (TimingWriteModeReqTracker).             //
    always_ff@(posedge clk or negedge rst) begin : TimingWriteModeReqTracker
        if(!rst) begin
            TimedWriteReqCnt <= 0;
        end else begin
                if(WriteRequestLatency[tCWL-2]==1 && writeServingACK) begin
                    TimedWriteReqCnt <= TimedWriteReqCnt;
                end else if(WriteRequestLatency[tCWL-2]) begin
                    TimedWriteReqCnt <= TimedWriteReqCnt + 1;
                end else if(writeServingACK)begin
                    TimedWriteReqCnt <= TimedWriteReqCnt - 1;
                end else begin
                    TimedWriteReqCnt <= TimedWriteReqCnt;
                end
        end
    end : TimingWriteModeReqTracker

    //               WriteMode - Data Out Valid  (PHY -> DRAM)            //
    //  In conclusion, "the data can be sent to DRAM in Write Mode" is      //
    //  1) Data is in PHY FIFO; 2) The DATA already passed tCWL constraints //
//    always_ff@(posedge clk or negedge rst) begin : WriteModeOutValid
//        if(!rst) begin
//            writeOutFlag <= 0;
//       end else begin
//            if(mode==1) begin
//                if(InflightWriteModeReq != 0 && TimedWriteReqCnt != 0) begin
//                    writeOutFlag <= 1;
//                end else begin
//                    writeOutFlag <= 0;
//                end
//            end else begin
//                writeOutFlag <= 0;
//            end
//        end
//    end : WriteModeOutValid
    assign writeOutFlag = (|InflightWriteModeReq) && (|TimedWriteReqCnt);
    
    //                   ReadMode - Data In Valid (DRAM -> PHY, tCL-aware)                 //
    //  For receving Read Data from DQ-Bus, We need to know "WHEN" the data will come?     //
    //  The timing for coming read data is based on "tCL" timing.                          //
    //  ReadModeInValid (Receiving data from DRAM) is valid for DRAM Burst_length (4-cycs) //
    //  The timing for Valid can reflect successive read data coming by initialization.    //
    always_ff@(posedge clk or negedge rst) begin : ReadModeInValid
        if(!rst) begin
            readBurstInCnt  <= 0;
            readInFlag <= 0;
        end else begin
            if(ReadRequestLatency[tCL-2]) begin
                readInFlag <= 1;
                readBurstInCnt  <= 0;       // If scucessive Read Req, Cnt needs to be initialized.
                `ifdef DISPLAY
                    $display("[%0t] PHYController | READ DATA BURST ON (tCL passed)", $time);
                `endif
            end
            else if (readBurstInCnt == BURST_LENGTH / 2 - 1) begin
                readInFlag <= 0;            // ReadBurstInCnt -> 4 cycles for receiving DATA from DRAM.
                readBurstInCnt  <= 0;
                `ifdef DISPLAY
                    $display("[%0t] PHYController | READ DATA BURST OFF", $time);
                `endif
            end 
            else begin
                if(readInFlag) begin
                    readBurstInCnt <= readBurstInCnt + 1;
                end else begin
                    readBurstInCnt <= readBurstInCnt;
                end
            end
        end
    end : ReadModeInValid

    //               ReadMode - Data Out Valid (PHY -> READ BUFFER)                               //
    //  We need to know how many data stay in our ReadMode PHY FIFO.                              //
    //  Inflight Read Mode Requests are tracked based on readReceivingACK (from ReadMode PHY).    //
    //  Condition for Increment : Data (DRAM -> READ MODE FIFO)                                   //
    //  Condition for Decrement : Data (READ MODE FIFO -> READ BUFFER)                            //
    //  readReceivingACK : Only valid when Burst-Length of read data comes to PHY FIFO            //
    //  readBufferDataLast : Only valid when Burst-Length of read data goes to Read BUffer.       //
    always_ff@(posedge clk or negedge rst ) begin : InflightReadModeReqTracker
        if(!rst) begin
            InflightReadModeReq <= 0;
        end else begin
                if(readReceivingACK && readBufferDataLast)begin
                    InflightReadModeReq <= InflightReadModeReq;
                end else if(readReceivingACK) begin
                    InflightReadModeReq <= InflightReadModeReq + 1;
                end else if(readBufferDataLast) begin
                InflightReadModeReq <= InflightReadModeReq - 1; 
                end
        end
    end : InflightReadModeReqTracker

    assign readOutFlag = (InflightReadModeReq != 0) ? 1 : 0;

    //                 Auto Precharge Acknowledgement with ACK & Addr                  //
    //  Read Data from DQ-BUS is different to Write Data to DQ-BUS.                    //
    //  For Read Request ACK, we send AutoPrecharge ACK to RankFSM,                    //
    //  when the BurstLength of DATA comes to PHY FIFO from DRAM                       //
    //  For Write Request ACK, we send AutoPrecharge ACK to RankFSM,                   //
    //  when the BurstLength of DATA goes to DRAM from PHY FIFO                        //
    assign ReadAutoPrechargeACK   = readReceivingACK;
    assign WriteAutoPrechargeACK  =  writeServingACK;

    assign readBufferDataTag      = PHYReadReqWindow[ReadReqRdPtr].addr.rank;
    assign ReadAutoPrechargeAddr  = PHYReadReqWindow[ReadReqRdPtr].addr;        
    assign WriteAutoPrechargeAddr = PHYWriteReqWindow[WriteReqRdPtr].addr;    
    ////////////////////////////////////////////////////////////////////////////////////


    //                            Request Window In PHY Controller                          //
    //  1) Read Request Window in PHY CONTROLLER                                            //
    //      -- Read CMD with Auto Precharge management.                                     //
    //      -- Recording the address of Reserved Read Data for Read Buffer (Read Data Tag)  //
    //  2) Write Request Window in PHY CONTROLLER                                           //
    //      -- Write CMD with Auto Precharge Management.                                    //
    typedef struct packed {
        MemoryAddress addr;
        logic valid;
    } PhyRequestEntry;
    
    PhyRequestEntry PHYReadReqWindow  [PHYFIFOREQUESTWINDOW-1:0];
    PhyRequestEntry PHYWriteReqWindow [PHYFIFOREQUESTWINDOW-1:0];




    always_ff@(posedge clk or negedge rst) begin : ReadReqWindowWritePointer
        if(!rst) begin
            ReadReqWrPtr <= 0;
        end else begin
            if(ReadCMDIssuedACK) begin
                if(ReadReqWrPtr == PHYFIFOREQUESTWINDOW-1)begin
                    ReadReqWrPtr <= 0;
                end else begin
                    ReadReqWrPtr <= ReadReqWrPtr + 1;
                end
            end
        end
    end : ReadReqWindowWritePointer

    always_ff@(posedge clk or negedge rst) begin : WriteReqWindowWritePointer
        if(!rst) begin
            WriteReqWrPtr <= 0;
        end else begin
            if(WriteCMDIssuedACK) begin
                if(WriteReqWrPtr == PHYFIFOREQUESTWINDOW-1) begin
                    WriteReqWrPtr <= 0;
                end else begin
                    WriteReqWrPtr <= WriteReqWrPtr + 1;
                end
            end
        end
    end : WriteReqWindowWritePointer

    always_ff@(posedge clk or negedge rst) begin : ReadReqWindowReadPointer
        if(!rst) begin
            ReadReqRdPtr <= 0;
        end else begin
            if(readBufferDataLast) begin
                if(PHYReadReqWindow[ReadReqRdPtr].valid) begin
                    if(ReadReqRdPtr == PHYFIFOREQUESTWINDOW-1) begin
                        ReadReqRdPtr <= 0; 
                    end else begin
                        ReadReqRdPtr <= ReadReqRdPtr + 1;
                    end
                end else begin
                    ReadReqRdPtr <= ReadReqRdPtr;
                end
            end
        end
    end : ReadReqWindowReadPointer

    always_ff@(posedge clk or negedge rst) begin : WriteReqWindowReadPointer
        if(!rst) begin
            WriteReqRdPtr <= 0;
        end else begin
            if(writeServingACK) begin
                if(PHYWriteReqWindow[WriteReqRdPtr].valid) begin
                    if(WriteReqRdPtr == PHYFIFOREQUESTWINDOW - 1) begin
                        WriteReqRdPtr <= 0;
                    end else begin
                        WriteReqRdPtr <= WriteReqRdPtr + 1;
                    end
                end else begin
                    WriteReqRdPtr <= WriteReqRdPtr;
                end
            end
        end
    end : WriteReqWindowReadPointer


    always_ff@(posedge clk or negedge rst) begin : ReadReqEntryPushandPOP
        if(!rst) begin
            for(int i = 0; i < PHYFIFOREQUESTWINDOW; i++)begin
                PHYReadReqWindow[i].addr  <= '0;
                PHYReadReqWindow[i].valid <= '0;
            end
        end else begin
                if(ReadCMDIssuedACK) begin
                    PHYReadReqWindow[ReadReqWrPtr].addr  <= ReadCMDIssuedAddr;
                    PHYReadReqWindow[ReadReqWrPtr].valid <= 1;
                end else if(readBufferDataLast)begin
                    PHYReadReqWindow[ReadReqRdPtr].valid <= 0;
                end
        end
    end : ReadReqEntryPushandPOP

    always_ff@(posedge clk or negedge rst) begin : WriteReqEntryPushNPOP
        if(!rst) begin
            for(int i = 0; i < PHYFIFOREQUESTWINDOW; i++) begin
                PHYWriteReqWindow[i].addr  <= '0;
                PHYWriteReqWindow[i].valid <= '0;
            end
        end else begin
                if(WriteCMDIssuedACK) begin
                    PHYWriteReqWindow[WriteReqWrPtr].addr  <= WriteCMDIssuedAddr;
                    PHYWriteReqWindow[WriteReqWrPtr].valid <= 1;
                end else if(writeServingACK) begin
                    PHYWriteReqWindow[WriteReqRdPtr].valid <= 0;    
                end
        end
    end : WriteReqEntryPushNPOP
    
endmodule
