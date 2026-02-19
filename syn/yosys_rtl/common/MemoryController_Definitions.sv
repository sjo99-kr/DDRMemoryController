/* verilator lint_off TIMESCALEMOD */

package MemoryController_Definitions;

    //                  MEMORY CONTROLLER BASE PARAMETERS                    //
    parameter int MEM_ADDRWIDTH = 32;                                        
    parameter int MEM_DATAWIDTH = 64;                                        
    parameter int MEM_IDWIDTH = 4;                                           
    parameter int MEM_USERWIDTH = 1;                                         
    parameter int COMMAND_WIDTH = 18;                                        
    parameter int BURST_LENGTH = 8;

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
    parameter int DEVICEPERRANK = 4;
    parameter int NUM_FSM = 1 << (RKWIDTH + CHWIDTH);           // Number of RankFSM in Memory Controller
    parameter int NUM_FSM_BIT = $clog2(NUM_FSM);                // Width of num. of RankFSM

    parameter int READCMDQUEUEDEPTH = 8;                        // Read Request Queue Depth for Rank Scheduler 
    parameter int WRITECMDQUEUEDEPTH = 8;                       // Write Request Queue Depth for Rank Scheduler
    parameter int READBUFFERDEPTH = 128;                        // Read Data Buffer Depth (128-entry, each entry is 64-B)
    parameter int WRITEBUFFERDEPTH = 128;                       // Write Data Buffer Depth (128-entry, each entry is 64-B)
    parameter int OPENPAGELISTDEPTH = 1 << (BKWIDTH+BGWIDTH);   // Open Row List Depth for Rank Scheduler
    parameter int ASSEMBLER_DEPTH = 8;


    parameter int NUMCHANNEL    = 2;                            // Number of Memory channels in Memory Controller
    parameter int NUMRANK       = 4;                            // Number of Rank per Memory Channel in Memory Controller
    parameter int NUMBANKGROUP  = 4;                            // Number of BankGroup per Rank in Memory Controller
    parameter int NUMBANK       = 4;                            // Number of Bank per BankGroup in Memory Controller

    parameter int PHYFIFOMAXENTRY = 4;
    parameter int PHYFIFODEPTH = PHYFIFOMAXENTRY * BURST_LENGTH;             // PHYFIFO FOR READ DATA PACKED
    parameter int PHYFIFOREQUESTWINDOW = 8;

    parameter NUM_BANKFSM = 16;
    parameter NUM_BANKFSM_BIT = 4;
    //                           MEMORY CONTROLLER SCHEDULING/MANAGEMENT SPECIFICATION                        //

    parameter int THRESHOLD = 512;                              // Threshold for aging counting  | blocking the request starvation
    parameter int AGINGWIDTH = 10;                              // Width of aging couting in Request Queue
    parameter int CHMODETHRESHOLD = 16;
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
        mem_addr_t addr;                // [36:5]
            logic [MEM_IDWIDTH-1:0] id; // [4]
            logic [MEM_USERWIDTH-1:0] user;
    } ReadBufferDirEntry;                  // 37       // READ BUFFER DIRECTORY ENTRY 

    typedef struct  packed {
        mem_addr_t addr;                // [53:22]
        logic [MEM_IDWIDTH-1:0] id;     // [21:18]
        logic [MEM_USERWIDTH-1:0] user; // [17]
        logic [$clog2(WRITEBUFFERDEPTH)-1:0] ptr; // [16:10]
        logic [MEM_DATAWIDTH/BURST_LENGTH - 1 : 0] strb; // [9:2]
        logic issued; //[1]
        logic valid;  //[0]
    } WriteBufferDirEntry;     //54                   // WRITE BUFFER DIRECTORY ENTRY

    
    typedef struct packed {
        mem_addr_t mem_addr;  // [43:12]
        logic [2:0] PageHit;  // [11:9]
        logic PageMiss;       // [8]
        logic PageEmpty;      // [7]
        logic AutoPreCharge;  // [6]
        
        logic req_type;        // [5]
        logic [MEM_USERWIDTH - 1 :0] req_user; //[4:1]
        logic [MEM_IDWIDTH-1:0] req_id; // [0] 
    } FSMReq;          //44                           // RANK FINITE STATE MACHINE REQUEST STRUCTURE


    typedef logic [(MEM_DATAWIDTH)-1:0] READBUFFERDATAENTRY;    // READ BUFFER DATA ENTRY
    typedef logic [(MEM_DATAWIDTH)-1:0] WRITEBUFFERDATAENTRY;   // WRITE BUFFER DATA ENTRY




    //                               DDR-BASED MEMORY TIMING CONSTRATINS (Based on Ramulator)                           //
    parameter int tBL = 4;                                  // BurstLength in DDR4 Timing                                                           (DQ Bus)
    parameter int tCCDS = 4;                                // Column-to-Column for same bankgroup in DDR4 Timing                                   (DQ, CMD Bus) 
    parameter int tCCDL = 6;                                // Column-to-Column for different bankgroup in DDR4 Timing                              (DQ, CMD Bus)
    parameter int tRTRS = 2;                                // Rank-to-Rank Transition in DDR4 Timing                                               (CMD bus)
    parameter int tCL = 16;                                 // Latency for read data between Issuing Read CMD and Receiving Read DATA               (DQ Bus)
    parameter int tRCD = 16;                                // Latency for Activation Row                                                           (DRAM Constraints)
    parameter int tRP = 16;                                 // Latency for Recharge                                                                 (DRAM Constraints)
    parameter int tCWL = 12;                                // Latency for write data between Issuing Write CMD and reflecting DATA                 (DRAM Constraints)
    parameter int tRTW = 8;                                 // Latency for Channel Mode in Memory Channel state (Read-to-Write)                     (CMD Bus)
    parameter int tRAS = 39;                                // It can be a issue for Closed-page policy (Read-to-Precharge)                         (CMD Bus)
    parameter int tRC = 55;                                 // tRAS + tRP                                                                           (CMD Bus)
    parameter int tRTP = 9;                                 // Latency betweeen Read to Precharge                                                   (CMD Bus)
    parameter int tWTRS = 3;                                // Latency for Channel Mode in Memory Channel state (Write-to-Read) for same rank       (CMD Bus)
    parameter int tWTRL = 9;                                // Latency for Channel Mode in Memory Channel state (Write-to-Read) for diff rank       (CMD Bus)
    parameter int tWR = 18;                                 // It can be a issue for Closed-page policy (Write-to-Precharge)                        (CMD Bus)
    parameter int tRFC = 256;                               // Refresh latency in Memory Controller                                                 (CMD Bus)
    parameter int tREFI = 8192;                             // Refresh cycle in Memory Controller




   
    //                                AXI PROCOTOL BASE PARAMETERS                           //
    parameter int AXI_DATAWIDTH = 64;
    parameter int AXI_ADDRWIDTH = 32;
    parameter int AXI_IDWIDTH = 4;
    parameter int AXI_USERWIDTH = 1;

    //                                 AXI BUS STRUCT SPECIFICATION                           //
    typedef struct packed{
        logic [AXI_IDWIDTH-1:0] id;  // [36]
        logic [AXI_ADDRWIDTH-1:0] addr; // [35:4]
        logic [AXI_USERWIDTH - 1:0] user; // [3:0]
    }axi_aw_chan_t;                   //37              // ADDRESS-WRITE
    
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
        mem_addr_t mem_addr; // 32 [76:45]
        logic [NUM_FSM-1:0] fsm; // [44:37]
        axi_aw_chan_t aw; // 37 [36:0]
    } WrAddrEntry;          //    77                    // MEMORY CONTROLLER WRITE-REQUEST ASSEMBLER ENTRY

    //                                        CAHCE-SIDE INTERFACE STRUCTURES                                        //
    typedef struct packed{
        axi_aw_chan_t aw; // 37
        logic aw_valid;  //1

        axi_w_chan_t w; // 78 
        logic w_valid;  // 1 

        axi_ar_chan_t ar; // 37
        logic ar_valid; // 1

        logic r_ready;
        logic b_ready;
    } cache_side_request; // 157
    
    typedef struct packed{
        logic aw_ready;
        logic ar_ready;
        logic w_ready;

        axi_r_chan_t r; // 70
        logic r_valid;

        axi_b_chan_t b;         // 5
        logic b_valid;
    } cache_side_response; // 80
    
    //                                      MEMORY CONTROLLER-SIDE INTERFACE STRUCTURES                                  //
    typedef struct packed{
        
        mem_addr_t mem_addr;                    // [153:121]
        logic [MEM_IDWIDTH - 1 :0] mem_id;      // [121:118]
        logic [MEM_USERWIDTH - 1 :0] mem_user;  // [117]
        logic [AXI_ADDRWIDTH-1:0] addr;         // [116:85]

        logic [MEM_DATAWIDTH - 1 :0] write_data;  // [84:21]
        logic [MEM_DATAWIDTH/8 - 1:0] write_strb; // [20:13]
        logic last; // [12]

        logic [NUM_FSM-1:0] req_valid;  // [11:4] 
        logic req_data_valid; // [3]
        logic write; // [2] write : 1 , read: 0

        logic readReady; // [1]
        logic AckReady; // [0]
    } mc_side_request;  // 154
    
    typedef struct packed {
        logic [MEM_DATAWIDTH - 1:0] read_data;
        logic [MEM_IDWIDTH - 1 :0] mem_read_id;
        logic [MEM_USERWIDTH - 1 :0] mem_read_user;
        logic last;
        logic r_valid;

        logic [MEM_IDWIDTH - 1 : 0] mem_ack_id;
        logic [MEM_USERWIDTH - 1 :0] mem_ack_user;
        logic b_valid;

        logic [NUM_FSM-1:0] ar_ready;
        logic [NUM_FSM-1:0] w_ready; 
        logic [NUM_FSM-1:0] aw_ready;
    } mc_side_response; // 80
    //////////////////////////////////////////////////////////////





endpackage
/* verilator lint_on TIMESCALEMOD */
