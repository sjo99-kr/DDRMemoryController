/* verilator lint_off TIMESCALEMOD */
`timescale  1ns / 1ps

package MemoryController_Definitions;

    //                  MEMORY CONTROLLER BASE PARAMETERS                    //
    parameter int MEM_ADDRWIDTH = 32;                                        
    parameter int MEM_DATAWIDTH = 64;                                        
    parameter int MEM_IDWIDTH   = 4;                                           
    parameter int MEM_USERWIDTH = 1;                                         
    parameter int COMMAND_WIDTH = 18;                      
    parameter int BURST_LENGTH  = 8;
                     
    ///////////////////////////////////////////////////////////////////////////

    //                           MEMORY CONTROLLER ADDRESS SPECIFICATION                                   //
    parameter int RWIDTH  = 15;                             // Row address width       (15-bits)
    parameter int CWIDTH  = 10;                             // Col address width       (10-bits, 3-bits for burst-addressed)
    parameter int BGWIDTH = 2;                              // Bank address width      (2-bits, 4 banks per bank group)
    parameter int BKWIDTH = 2;                              // BankGroup address width (2-bits, 4 bankgroups per rank)
    parameter int RKWIDTH = 2;                              // Rank address width      (2-bits, 4 ranks per channel)
    parameter int CHWIDTH = 1;                              // Channel address width   (1-bits, 2 memory channels per MemoryController)
    parameter int IOWIDTH = 8;                              // I/O Width per DRAM Chip (x8)
    //////////////////////////////////////////////////////////////////////////////////////////////////////////


    //                             MEMORY CONTROLLER HARDWARE SPECIFICATION                                //
    parameter int DEVICEPERRANK = 8;



    parameter int READCMDQUEUEDEPTH  = 8;                         // Read Request Queue Depth for Rank Scheduler 
    parameter int WRITECMDQUEUEDEPTH = 8;                         // Write Request Queue Depth for Rank Scheduler
    parameter int READBUFFERDEPTH    = 128;                       // Read Data Buffer Depth (128-entry, each entry is 64-B)
    parameter int WRITEBUFFERDEPTH   = 128;                       // Write Data Buffer Depth (128-entry, each entry is 64-B)
    parameter int OPENPAGELISTDEPTH  = 1 << (BKWIDTH+BGWIDTH);    // Open Row List Depth for Rank Scheduler
    parameter int ASSEMBLER_DEPTH    = 8;                         // Depth of write assembler queue

    parameter int NUMCHANNEL    = 2;                              // Number of Memory channels in Memory Controller
    parameter int NUMRANK       = 4;                              // Number of Rank per Memory Channel in Memory Controller
    parameter int NUMBANKGROUP  = 4;                              // Number of BankGroup per Rank in Memory Controller
    parameter int NUMBANK       = 4;                              // Number of Bank per BankGroup in Memory Controller

    parameter int NUM_BANKFSM                    = NUMBANKGROUP * NUMBANK;
    parameter int NUM_BANKFSM_BIT                = $clog2(NUM_BANKFSM);
    parameter int NUM_RANKEXECUTION_UNIT         = 1 << (RKWIDTH + CHWIDTH);       
    parameter int NUM_RANKEXECUTION_UNIT_BIT     = $clog2(NUM_RANKEXECUTION_UNIT);                

    parameter int PHYFIFOMAXENTRY       = 4;                                // PHY FIFO Max number for Burst-data
    parameter int PHYFIFODEPTH          = PHYFIFOMAXENTRY * BURST_LENGTH;   // PHYFIFO FOR READ DATA PACKED
    parameter int PHYFIFOREQUESTWINDOW  = 8;                                // READ/WRITE REQUEST WINDOW FOR PHY Controller
    

    //                           MEMORY CONTROLLER SCHEDULING/MANAGEMENT SPECIFICATION                        //
    parameter int THRESHOLD         = 512;                              // Threshold for aging counting  | blocking the request starvation
    parameter int AGINGWIDTH        = 10;                              // Width of aging couting in Request Queue
    parameter int CHMODETHRESHOLD   = 16;
    parameter int RESPSCHEDULINGCNT = 4;


    //                                 MEMORY CONTROLLER-RELATED STRUCTURES                                    //
    typedef struct packed {
        logic [CHWIDTH-1:0] channel;              // 1
        logic [RKWIDTH-1:0] rank;                 // 2
        logic [BGWIDTH-1:0] bankgroup;            // 2
        logic [BKWIDTH-1:0] bank;                 // 2
        logic [RWIDTH-1:0] row;                   // 15
        logic [CWIDTH-1:0] col;                   // 10
    } mem_addr_t;                                   // MEMORY CONTROLLER REQUEST ADDRESS (32-bits)

    typedef struct packed  {
        mem_addr_t addr;
            logic [MEM_IDWIDTH-1:0] id;
            logic [MEM_USERWIDTH-1:0] user;
    } ReadBufferDirEntry;                         // READ BUFFER DIRECTORY ENTRY 

    typedef struct  packed {
        mem_addr_t addr;
        logic [MEM_IDWIDTH-1:0] id;
        logic [MEM_USERWIDTH-1:0] user;
        logic [$clog2(WRITEBUFFERDEPTH)-1:0] ptr;
        logic [MEM_DATAWIDTH/BURST_LENGTH - 1 : 0] strb;
        logic issued;
        logic valid;
    } WriteBufferDirEntry;                        // WRITE BUFFER DIRECTORY ENTRY
    
    typedef struct packed {
        mem_addr_t mem_addr;
        logic [2:0] PageHit;
        logic PageMiss;
        logic PageEmpty;
        logic AutoPreCharge;
        
        logic req_type;
        logic [MEM_USERWIDTH - 1 :0] req_user;
        logic [MEM_IDWIDTH-1:0] req_id;
    } FSMReq;                                     // RANK FINITE STATE MACHINE REQUEST STRUCTURE


    typedef logic [(MEM_DATAWIDTH)-1:0] READBUFFERDATAENTRY;    // READ BUFFER DATA ENTRY
    typedef logic [(MEM_DATAWIDTH)-1:0] WRITEBUFFERDATAENTRY;   // WRITE BUFFER DATA ENTRY




    //                               DDR-BASED MEMORY TIMING CONSTRAINTS (Based on Ramulator)                           //
    parameter int tBL   = 4;                                  // BurstLength in DDR4 Timing                                                           (DQ Bus)
    parameter int tCCDS = 4;                                // Column-to-Column for same bankgroup in DDR4 Timing                                   (DQ, CMD Bus) 
    parameter int tCCDL = 6;                                // Column-to-Column for different bankgroup in DDR4 Timing                              (DQ, CMD Bus)
    parameter int tRTRS = 2;                                // Rank-to-Rank Transition in DDR4 Timing                                               (CMD bus)
    parameter int tCL   = 16;                                 // Latency for read data between Issuing Read CMD and Receiving Read DATA               (DQ Bus)
    parameter int tRCD  = 16;                                // Latency for Activation Row                                                           (DRAM Constraints)
    parameter int tRP   = 16;                                 // Latency for Recharge                                                                 (DRAM Constraints)
    parameter int tCWL  = 12;                                // Latency for write data between Issuing Write CMD and reflecting DATA                 (DRAM Constraints)
    parameter int tRTW  = 8;                                 // Latency for Channel Mode in Memory Channel state (Read-to-Write)                     (CMD Bus)
    parameter int tRAS  = 39;                                // It can be a issue for Closed-page policy (Read-to-Precharge)                         (CMD Bus)
    parameter int tRC   = 55;                                 // tRAS + tRP                                                                           (CMD Bus)
    parameter int tRTP  = 9;                                 // Latency betweeen Read to Precharge                                                   (CMD Bus)
    parameter int tWTRS = 3;                                // Latency for Channel Mode in Memory Channel state (Write-to-Read) for same rank       (CMD Bus)
    parameter int tWTRL = 9;                                // Latency for Channel Mode in Memory Channel state (Write-to-Read) for diff rank       (CMD Bus)
    parameter int tWR   = 18;                                 // It can be a issue for Closed-page policy (Write-to-Precharge)                        (CMD Bus)
    parameter int tRFC  = 256;                               // Refresh latency in Memory Controller                                                 (CMD Bus)
    parameter int tREFI = 8192;                             // Refresh cycle in Memory Controller


   
    //                                AXI PROCOTOL BASE PARAMETERS                           //
    parameter int AXI_DATAWIDTH = 64;
    parameter int AXI_ADDRWIDTH = 32;
    parameter int AXI_IDWIDTH   = 4;
    parameter int AXI_USERWIDTH = 1;

    //                                 AXI BUS STRUCT SPECIFICATION                           //
    typedef struct packed{
        logic [AXI_IDWIDTH-1:0] id;
        logic [AXI_ADDRWIDTH-1:0] addr;
        logic [AXI_USERWIDTH - 1:0] user;
    }axi_aw_chan_t;                                 // ADDRESS-WRITE
    
    typedef struct packed {
        logic [AXI_DATAWIDTH-1:0] data;
        logic [AXI_USERWIDTH-1:0] user;
        logic [AXI_IDWIDTH-1:0] id;
        logic last;
        logic [AXI_DATAWIDTH/8 -1 :0] strb;
    } axi_w_chan_t;                                 // DATA-WRITE
    
    typedef struct packed{
        logic [AXI_ADDRWIDTH-1:0] addr;
        logic [AXI_IDWIDTH - 1 :0] id;
        logic [AXI_USERWIDTH - 1:0] user;
    } axi_ar_chan_t;                                // ADDRESS-READ
    
    typedef struct packed{
        logic [AXI_DATAWIDTH-1:0] data;
        logic [AXI_USERWIDTH-1:0] user;
        logic [AXI_IDWIDTH-1:0] id;
        logic last;
    } axi_r_chan_t;                                 // DATA-READ
    
    typedef struct packed{
        logic [AXI_USERWIDTH-1:0] user;
        logic [AXI_IDWIDTH-1:0] id;
    } axi_b_chan_t;                                 // ACK-WRITE
    

    typedef struct packed {
        mem_addr_t mem_addr;
        logic [NUM_RANKEXECUTION_UNIT-1:0] fsm;
        axi_aw_chan_t aw;
    } WrAddrEntry;                                  // MEMORY CONTROLLER WRITE-REQUEST ASSEMBLER ENTRY

    //                                        CAHCE-SIDE INTERFACE STRUCTURES                                        //
    typedef struct packed{
        axi_aw_chan_t aw;
        logic aw_valid;

        axi_w_chan_t w;
        logic w_valid;

        axi_ar_chan_t ar;
        logic ar_valid;

        logic r_ready;
        logic b_ready;
    } cache_side_request;
    
    typedef struct packed{
        logic aw_ready;
        logic ar_ready;
        logic w_ready;

        axi_r_chan_t r;
        logic r_valid;

        axi_b_chan_t b;        
        logic b_valid;
    } cache_side_response;
    
    //                                      MEMORY CONTROLLER-SIDE INTERFACE STRUCTURES                                  //
    typedef struct packed{
        
        mem_addr_t mem_addr;
        logic [MEM_IDWIDTH - 1 :0] mem_id;
        logic [MEM_USERWIDTH - 1 :0] mem_user;
        logic [AXI_ADDRWIDTH-1:0] addr;

        logic [MEM_DATAWIDTH - 1 :0] write_data;
        logic [MEM_DATAWIDTH/8 - 1:0] write_strb;
        logic last;

        logic [NUM_RANKEXECUTION_UNIT-1:0] req_valid;
        logic req_data_valid;
        logic write; // write : 1 , read: 0

        logic readReady;
        logic AckReady;
    } mc_side_request;
    
    typedef struct packed {
        logic [MEM_DATAWIDTH - 1:0] read_data;
        logic [MEM_IDWIDTH - 1 :0] mem_read_id;
        logic [MEM_USERWIDTH - 1 :0] mem_read_user;
        logic last;
        logic r_valid;

        logic [MEM_IDWIDTH - 1 : 0] mem_ack_id;
        logic [MEM_USERWIDTH - 1 :0] mem_ack_user;
        logic b_valid;

        logic [NUM_RANKEXECUTION_UNIT-1:0] ar_ready;
        logic [NUM_RANKEXECUTION_UNIT-1:0] w_ready; 
        logic [NUM_RANKEXECUTION_UNIT-1:0] aw_ready;
    } mc_side_response;
    //////////////////////////////////////////////////////////////


endpackage
/* verilator lint_on TIMESCALEMOD */
