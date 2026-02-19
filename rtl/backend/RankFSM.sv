`timescale 1ns / 1ps

//------------------------------------------------------------------------------
//      RankFSM
//
//      Role:
//          Rank-level DDR command state machine.
//
//      Responsibilities:
//          - Executes ACT / PRE / RD / WR / REF commands for a single rank.
//          - Enforces rank- and bank-level DRAM timing constraints.
//          - Coordinates with ChannelScheduler for CMD/DQ bus availability.
//          - Issues buffer requests and handles auto-precharge semantics.
//
//      Scope & Notes:
//          - Timing-accurate at command level (tRCD, tRP, tRFC, tWR).
//          - No request reordering; serves one scheduled request at a time.
//          - Electrical behavior is abstracted (command-level only).
//
//      Author  : Seongwon Jo
//      Created : 2026.02
//------------------------------------------------------------------------------


module RankFSM #(
    parameter int FSM_CHANNEL = 0,
    parameter int FSM_RANK = 0,

    
    parameter int NUM_BANKFSM = 16,
    parameter int NUM_BANKFSM_BIT = 4,

    parameter int MEM_IDWIDTH = 4,
    parameter int MEM_USERWIDTH = 1,
    parameter int BKWIDTH = 2,
    parameter int BGWIDTH = 2,
    parameter int RWIDTH = 15,
    parameter int CWIDTH = 10,
    parameter int NUMBANK = 4,
    parameter int NUMBANKGROUP = 4,
    parameter int COMMAND_WIDTH = 18,

    parameter int tRP = 16,
    parameter int tWR = 18,
    parameter int tRFC = 256,
    parameter int tRCD = 16,

    parameter type FSMRequest = logic,
    parameter type MemoryAddress = logic
)(
    // Common
    input logic clk, 
    input logic rst,
    
                                                          //   Input from MEM Buffer    //
    input logic rbuf_available,
    input logic rbufWindowAvailable, 
    input logic wbuf_available,
    input logic ReadPreAck,
    input logic WritePreAck,
    input logic [BKWIDTH+BGWIDTH - 1 : 0] bufBankPre,

                                                          //   Output to MEM Buffer    //
    output logic bufReadReqIssued,
    output logic [MEM_IDWIDTH-1:0] bufReadReqId,
    output logic [MEM_USERWIDTH-1:0] bufReadReqUser,
    output logic bufWriteReqIssued,
    output logic [MEM_IDWIDTH-1:0] bufWriteReqId,
    output logic [MEM_USERWIDTH-1:0] bufWriteReqUser,
    output MemoryAddress bufReqACKAddr,

                                                          //   Input from RankSched    //
    input FSMRequest schedReq,
    input logic [NUM_BANKFSM-1:0] schedValid,
    input logic refresh,
                                                          //   Output to RankSched    //
    output logic schedRefACK,
    output logic [NUM_BANKFSM-1:0] schedIdle,
    output logic [NUM_BANKFSM-1:0] fsmWait,           // Valid when the FSM is waiting for loadTimer

    ////////////////////////////////////////////////////////////////////////////////////////
    // Why the FSM need to consider the timing constraints for DQ bus???                  //
    // - Unlike ACT, PRE, REFRESH, the read and write command have to aware               //
    // of timing for read data and write data.                                            //
    ////////////////////////////////////////////////////////////////////////////////////////

                                                          //   Input from ChannelSched    //
    input logic chCMDAvailable,                           // Channel Granted + tRTR  
    input logic chCMDDQAvailable,                         // tCCDL, tCCDS, tRTW, tWTRS, tWTRL
    input logic chMode,

                                                          //   Output to ChannelSched    //
    output logic CCDShort,                                // Type of tCCCD, Short? Long? //
    output logic chSchedRdWrACK,                          // ACK for Read/Write command Issuing
    output logic chSchedCMDACK,                           // ACK for all kind of CMD Issuing


    // DDR4 Interface (FSM <-> DDR4 PHY Interface)
    /* verilator lint_off UNDRIVEN */
    output logic cke, cs_n, par, act_n,
    output logic [COMMAND_WIDTH-1:0] pin_A,
    output logic [BGWIDTH-1:0] bg,
    output logic [BKWIDTH-1:0] b
    );


    logic cke_bank [NUM_BANKFSM-1:0];
    logic cs_n_bank [NUM_BANKFSM-1:0];
    logic par_bank [NUM_BANKFSM-1:0];
    logic act_n_bank [NUM_BANKFSM-1:0];
    logic [COMMAND_WIDTH-1:0]  pin_A_bank [NUM_BANKFSM-1:0];
    logic [BGWIDTH-1:0] bg_bank [NUM_BANKFSM-1:0]; 
    logic [BKWIDTH-1:0] b_bank [NUM_BANKFSM-1:0];


    // State for RANK-LEVEL Request serving FSM
    typedef enum logic [2:0] {
        Idle,       // 0                // Idle: Waiting request from RankSched.
        LoadTimer,  // 1                // LoadTimer: Calculating the DRAM Timing Constraints
        Read,       // 2    
        Write,      // 3
        Refresh,    // 4
        Precharge,  // 5
        Activate    // 6
    } state_t;
    state_t state [NUM_BANKFSM-1:0];
    state_t prev  [NUM_BANKFSM-1:0]; 
    state_t next  [NUM_BANKFSM-1:0];

    FSMRequest servingReq [NUM_BANKFSM-1:0];

    logic [NUM_BANKFSM-1:0] availBanks;
    logic [NUM_BANKFSM-1:0] targetBanks;


    always_comb begin
        availBanks  = '0;
        for(int i = 0; i < NUM_BANKFSM; i++) begin
            if(!schedIdle[i] && !fsmWait[i]) begin
                availBanks[i] = 1;
            end
        end
    end

    always_ff@(posedge clk or negedge rst) begin
        if(!rst) begin
            targetBanks <= '0; 
        end else begin
            targetBanks <= availBanks & (~availBanks + 1);
        end
    end

    
    logic [NUM_BANKFSM-1:0] ccdType;

    logic bufReadReqIssuedBank [NUM_BANKFSM-1:0];
    logic [MEM_IDWIDTH-1:0] bufReadReqIdBank [NUM_BANKFSM-1:0];
    logic [MEM_USERWIDTH-1:0] bufReadReqUserBank [NUM_BANKFSM-1:0];

    logic bufWriteReqIssuedBank [NUM_BANKFSM-1:0];
    logic [MEM_IDWIDTH-1:0] bufWriteReqIdBank [NUM_BANKFSM-1:0];
    logic [MEM_USERWIDTH-1:0] bufWriteReqUserBank [NUM_BANKFSM-1:0];

    MemoryAddress bufReqACKAddrBank [NUM_BANKFSM-1:0];

    
    logic [NUM_BANKFSM-1:0] schedRefACKBank;
    logic [NUM_BANKFSM-1:0] chSchedCMDACKBank;
    logic [NUM_BANKFSM-1:0] chSchedRdWrACKBank;

    always_comb begin
        bufReadReqIssued   = '0;
        bufReadReqId       = '0;
        bufReadReqUser     = '0;
        bufWriteReqIssued  = '0;
        bufWriteReqId      = '0;
        bufWriteReqUser    = '0;
        bufReqACKAddr      = '0;
        CCDShort           = '0;
        chSchedCMDACK      = '0;
        chSchedRdWrACK = '0;
        schedRefACK = &schedRefACKBank;
        
        for(int i = 0; i < NUM_BANKFSM; i++) begin
            if(targetBanks[i])begin
                bufReadReqIssued = bufReadReqIssuedBank[i];
                bufReadReqId = bufReadReqIdBank[i];
                bufReadReqUser = bufReadReqUserBank[i];

                bufWriteReqIssued = bufWriteReqIssuedBank[i];
                bufWriteReqId =  bufWriteReqIdBank[i];
                bufWriteReqUser = bufWriteReqUserBank[i];

                bufReqACKAddr   = bufReqACKAddrBank[i];
                CCDShort = ccdType[i];

                chSchedCMDACK  = chSchedCMDACKBank[i];
                chSchedRdWrACK = chSchedRdWrACKBank[i];
            end
        end
    end






    //--------------- Counter for DRAM Timing Constraints ------------------//
    logic [NUM_BANKFSM-1:0] TimingSet;
    logic [5:0] TimingLoad [NUM_BANKFSM-1:0];
    logic [NUM_BANKFSM-1:0] RowFree;                  // RowFree only affects on LoadTimer state.

    //------------------------------------------------------------------------------
    //  DRAM Timing Counter
    //
    //  - Centralized counter for rank-level DRAM timing constraints.
    //  - Used to serialize ACT / PRE / REF transitions via LoadTimer state.
    //  - Provides RowFree signal to unblock FSM state progression.
    //
    //------------------------------------------------------------------------------
    genvar i;
    generate 
        for(i = 0; i < NUM_BANKFSM; i++)  begin
            DRAMTimingCounter DRAMTimingCounter_Instance(             // Row Timing constraints
                .clk(clk), .rst(rst), .setup(TimingSet[i]), .load(TimingLoad[i]), .timeUp(RowFree[i])
            );
        end
    endgenerate
    //////////////////////////////////////////////////////////////////////////

    //------------ Bank State Tracker (Case for Auto-Precharge) ------------//
    logic bankState [(NUMBANK * NUMBANKGROUP) - 1 : 0];
    logic [NUM_BANKFSM-1:0] apSetup_bank;
    logic apSetup; 
    logic [$clog2(NUM_BANKFSM)-1:0] BGBKfromFSM;
    logic autoPrechargeACK;
    logic [BKWIDTH+BGWIDTH-1:0] bankIndex [NUM_BANKFSM-1 : 0];

    always_comb begin
        apSetup = 0;
        BGBKfromFSM = '0;
        for(int a = 0; a < NUM_BANKFSM; a++) begin
            if(targetBanks[a]) begin
                apSetup = apSetup_bank[a];
                BGBKfromFSM = bankIndex[a];
            end
        end
    end

    generate 
        for(i = 0; i < NUM_BANKFSM; i++ ) begin
            assign bankIndex[i] = {servingReq[i].mem_addr.bankgroup, servingReq[i].mem_addr.bank};
        end
    endgenerate

    //------------------------------------------------------------------------------
    //  Auto-Precharge & Bank State Tracker
    //
    //  - Tracks per-bank availability for safe ACT / RD / WR / PRE issuance.
    //  - Handles timing dependencies for auto-precharge (tRP, tWR).
    //  - Prevents command issue when target bank is busy.
    //
    //------------------------------------------------------------------------------
    APTimingCounter #(
        .NUMBANK(NUMBANK),
        .NUMBANKGROUP(NUMBANKGROUP),
        .tRP(tRP),
        .tWR(tWR)
    ) APTimingCounter_Instance (
        .clk(clk), .rst(rst), .mode(chMode), 
        .apSetup(apSetup), .apACk(autoPrechargeACK), 
        .BGBKfromFSM(BGBKfromFSM), 
        .BGBKfromBUF(bufBankPre), .bankState(bankState)
    );
    assign autoPrechargeACK = WritePreAck || ReadPreAck;
    //////////////////////////////////////////////////////////////////////////

    //------------------------------------------------------------------------------
    //  Scheduler Request Latch
    //
    //  - Captures one scheduled request from RankScheduler.
    //  - FSM serves exactly one request at a time.
    //  - New requests are only accepted in Idle state.
    //
    //-----------------------------------------------------------------------------
    always_ff@(posedge clk or negedge rst) begin
        if(!rst)begin
            for(int p = 0; p < NUM_BANKFSM; p++) begin
                servingReq[p] <= 0;
            end
        end else begin
            for(int p = 0; p < NUM_BANKFSM; p++) begin
                if(schedValid[p] && (state[p] == Idle)) begin
                    servingReq[p] <= schedReq;                 // The condition for receving Req : State is Idle, and Valid Signal from RankSched.
                end
            end
        end
    end



    //------------------------------------------------------------------------------
    //  RankFSM State Register
    //
    //  - Holds current / previous FSM states.
    //  - Preserves previous state during LoadTimer for timing serialization.
    //  - Ensures correct post-timing state transitions.
    //
    //------------------------------------------------------------------------------
    generate 
        for(i = 0; i < NUM_BANKFSM; i++) begin
            always_ff@(posedge clk or negedge rst) begin
                if(!rst)begin
                    state[i] <= Idle;                  // Initialization : Idle State
                    prev[i] <= Idle;
                end else begin
                    state[i] <= next[i];                  // Default: State (FF) <- Next (COMB)
                    prev[i] <= state[i];                  //          PREV (FF) <- State (FF)
                    //      If state is on LoadTimer, We have to MAINTAIN PREV state until escaping from LoadTimer.     //
                    if(state[i] == LoadTimer) begin    
                        if(prev[i] == Activate || prev[i] == Precharge || prev[i] == Refresh) begin 
                            if(!RowFree[i]) begin              
                                prev[i] <= prev[i]; 
                            end 
                        end 
                    end
                end
            end
        end
    endgenerate
    

    //------------------------------------------------------------------------------
    //  RankFSM Control Logic
    //
    //  - Determines next FSM state based on:
    //      * Page hit / miss conditions
    //      * Channel CMD / DQ availability
    //      * DRAM timing constraints
    //  - Issues:
    //      * ChannelScheduler ACKs
    //      * Buffer request handshakes
    //      * Timing counter setup signals
    //
    //------------------------------------------------------------------------------
    generate 
        for(i = 0; i < NUM_BANKFSM; i++) begin
            always_comb begin
                next[i] = Idle;

                schedIdle[i] = 0;
                fsmWait[i] = 0;

                apSetup_bank[i] = 0;

                TimingSet[i] = 0;
                TimingLoad[i] = '0;
                
                bufReadReqIssuedBank[i] = 0;
                bufWriteReqIssuedBank[i] = 0;
                bufReadReqIdBank[i] = 0;
                bufWriteReqIdBank[i] = 0;
                bufReadReqUserBank[i] = 0;
                bufWriteReqUserBank[i] = 0;
                bufReqACKAddrBank[i] = 0;
                ccdType[i] = 0;

                schedRefACKBank[i] = 0;
                chSchedCMDACKBank[i] = 0;
                chSchedRdWrACKBank[i] = 0;

                unique case(state[i])
                    Idle: begin                                                                 // Waiting request (Ready Phase)
                        schedIdle[i] = 1;
                        casez ({schedValid[i], refresh, schedReq.PageMiss, schedReq.PageEmpty})    // 4-bit standard for deciding next state.
                            4'b?1??: begin
                                next[i] = Refresh;                                                 // Next STATE: REFRESH 
                                schedIdle[i] = 0;
                                $display("[CH-%d RK-%d FSM] CLK: %d | REFRESH State", FSM_CHANNEL, FSM_RANK, $time);
                            end
                            4'b00??: begin                                                      // NO STATE CHANGE WITHOUT VALID.
                                next[i] = Idle;        
                            end
                            4'b1010: begin                                                      // Page Miss Case with Valid
                                next[i] = Precharge;                                               // We have to CLOSE the ROW first thing.
                                schedIdle[i] = 0;
                                $display("[CH-%d RK-%d BK-%d FSM] CLK: %d | PRECHARGE State", FSM_CHANNEL, FSM_RANK, i, $time);
                            end
                            4'b1001: begin                                                      // Page Empty (No need for precharge)
                                next[i] = Activate;                                             // We don't need to CLOSE the ROW.
                                                                                                // THERE IS NO OPEN ROW in the REQ. ADDR.
                                schedIdle[i] = 0;
                                $display("[CH-%d RK-%D FSM] CLK: %d | ACTIVATIVE State", FSM_CHANNEL, FSM_RANK, $time);
                            end
                            4'b1000: begin                                                      // Page Hit
                                                                                                // RTW , WTR consideration
                                                                                                // Go for WRITE and Read Command to Open Page(Row).
                                if(schedReq.req_type) begin
                                    next[i]  = Write;
                                end else begin
                                    next[i]  = Read;
                                end
                                schedIdle[i] = 0;
                            end
                            default : begin
                                next[i]  = Idle;
                            end
                        endcase
                    end
                    Activate: begin                                                              // State for loading specific row data to sense amplifiers (Row buffer)
                        if(chCMDAvailable && !bankState[bankIndex[i]] &&                       // For Activation Phase, we need to granted for memory channel.
                        targetBanks[i]) begin                        
                            next[i] = LoadTimer;                                                    // Activation can only be executed when there is no conflict with AP,
                            TimingSet[i] = 1;                                                       // and grant from channel scheduler.
                            TimingLoad[i] = tRCD-1;                                                 // We need to wait for ACTIVATION-related timing, tRCD.
                            chSchedCMDACKBank[i] = 1;                                                   // We send a signal to Channel Scheduler for acknowledging chched.
                        end else begin
                            next[i] = Activate;                                                     // With No grant and AP conditions, we need to WAIT.
                        end                                                                      // (TODO) Design flexible scheduling under AP condition.
                    end
                    LoadTimer: begin                                                             // State for Waiting DRAM TIMING CONSTRAINTS
                        fsmWait[i] = 1;
                        case(prev[i])
                            Activate: begin                                     // Need to wait for tRCD
                                if(RowFree[i]) begin                               // If the timing is end,
                                    $display("[CH-%d RK-%D FSM] CLK: %d | ACTIVATE FINISH", FSM_CHANNEL, FSM_RANK, $time);
                                    if(servingReq[i].req_type) begin               // Then go to Column operation for the requests
                                        //write == 1
                                        next[i]    = Write; 
                                        fsmWait[i] = 0;
                                    end else begin
                                        next[i] = Read;
                                        fsmWait[i] = 0;
                                    end
                                end
                                else begin 
                                    next[i] = LoadTimer;                           // Before timing is end, it requires to wait.
                                end
                            end

                            Precharge: begin                                    // Need to wait for tRP.
                                if(RowFree[i]) begin                               // If the timing is end,
                                    next[i] = Activate;                            // then go to activation the row.
                                    fsmWait[i] = 0;
                                    $display("[CH-%d RK-%D FSM] CLK: %d | PRECHARGE FINISH", FSM_CHANNEL, FSM_RANK, $time);
                                end else begin
                                    next[i] = LoadTimer;                           // Before timing is end, it requires to wait.
                                end
                            end

                            Refresh: begin                                      // Need to wait for tRFC.
                                if(RowFree[i]) begin                               // If the timing is end,
                                    next[i] = Idle;                                // then go to Idle.
                                    fsmWait[i] = 0;
                                    $display("[CH-%d RK-%D FSM] CLK: %d | REFRESH FINISH", FSM_CHANNEL, FSM_RANK, $time);
                                end else begin
                                    next[i] = LoadTimer;                           // Before timing is end, it requires to wait.
                                end
                            end
                            default: begin
                                next[i] = LoadTimer;
                            end
                        endcase
                    end
                    //////////////////////////////////////////////////////////////////////////////////////////////////////////
                    //                  Timing constraints for Read and Write Command                                       //
                    //      1) Memory channel Granted : Channel scheduler have to grant this rank for CMD Bus               //
                    //      2) tRTRs  : If there is a rank transition, it requires to wait tRTRs for CMD Bus.               //
                    //      3) tCCDs  : If there is a successive column operation, it requires to wait tCCD for DQ Bus      //
                    //      4) tRTW/tWTR : If there is a mode transition in DQ Bus, it requires to wait tRTW/WTR for DQ Bus //
                    //////////////////////////////////////////////////////////////////////////////////////////////////////////
                    //      - chCMDAvailable : Availability signal for CMD Bus, considering CMD Bus grant & tRTRs           //
                    //      - chDQAvailable  : Availability signal for DQ Bus, considering tCCD_short/long & tWTR/tRTW      //
                    //      - rbufavailable  : Availability signal from Buffer for affordability to new entry               //
                    //////////////////////////////////////////////////////////////////////////////////////////////////////////
                    Read: begin
                        if(chCMDAvailable & chCMDDQAvailable && rbuf_available && !bankState[bankIndex[i]] && rbufWindowAvailable &&
                           targetBanks[i]) begin
                            chSchedRdWrACKBank[i] = 1;
                            chSchedCMDACKBank[i] = 1;

                            bufReadReqIssuedBank[i] = 1;
                            bufReadReqIdBank[i]     = servingReq[i].req_id;
                            bufReadReqUserBank[i]   = servingReq[i].req_user;
                            bufReqACKAddrBank[i]    = servingReq[i].mem_addr;

                            if(servingReq[i].PageHit == 2'b11)begin // PageOpen Long
                                ccdType[i] = 0;               
                            end else if (servingReq[i].PageHit == 2'b10) begin // PageOpen Short
                                ccdType[i] = 1; 
                            end 
                            if(servingReq[i].AutoPreCharge) begin
                                apSetup_bank[i] = 1;
                            end
                            next[i] = Idle;
                            $display("[CH-%d RK-%D FSM] CLK: %d | READ FINISH", FSM_CHANNEL, FSM_RANK, $time);
                        end
                        else begin
                            next[i] = Read;
                        end
                    end

                    Write: begin
                        if(chCMDAvailable && chCMDDQAvailable && wbuf_available && !bankState[bankIndex[i]] &&
                            targetBanks[i]) begin
                            chSchedRdWrACKBank[i] = 1;
                            chSchedCMDACKBank[i] = 1;
                            
                            bufWriteReqIssuedBank[i] = 1;
                            bufWriteReqIdBank[i]     = servingReq[i].req_id;
                            bufWriteReqUserBank[i]   = servingReq[i].req_user;
                            bufReqACKAddrBank[i]     = servingReq[i].mem_addr;

                            if(servingReq[i].PageHit == 2'b11)begin // PageOpen Long
                                ccdType[i] = 0; // long                        
                            end else if (servingReq[i].PageHit == 2'b10) begin // PageOpen Short
                                ccdType[i] = 1; // short
                            end 
                            if(servingReq[i].AutoPreCharge) begin
                                apSetup_bank[i] = 1;
                            end
                            next[i] = Idle;
                            $display("[CH-%d RK-%D FSM] CLK: %d | WRITE FINISH", FSM_CHANNEL, FSM_RANK, $time);
                        end
                        else begin
                            next[i] = Write;
                        end
                    end

                    Refresh: begin // Rank-level Precharge + Refresh.
                        schedRefACKBank[i] = 1;
                        if(chCMDAvailable && !bankState[bankIndex[i]] && (&schedRefACKBank)) begin
                            next[i] = LoadTimer;
                            chSchedCMDACKBank[i] = 1;
                            TimingSet[i] = 1;
                            TimingLoad[i] = tRFC-1;
                        end else begin
                            next[i] = Refresh;
                        end
                    end

                    Precharge: begin // single-bank Precharge
                        if(chCMDAvailable && !bankState[bankIndex[i]] &&
                            targetBanks[i]) begin
                            next[i] = LoadTimer; // Single-bank Precharge 
                            chSchedCMDACKBank[i] = 1;
                            TimingSet[i] = 1;
                            TimingLoad[i] = tRP-1;
                        end else begin
                            next[i] = Precharge;
                        end
                    end
                    default : begin

                    end
                endcase
            end
        end
    endgenerate
    //////////////////////////////////////////////////////////////////////////


    //------------------------------------------------------------------------------
    //  DDR4 Command Interface Logic
    //
    //  - Encodes DDR4 command/address signals from FSM state.
    //  - Generates ACT / PRE / RD / WR / REF command patterns.
    //
    //------------------------------------------------------------------------------
    generate 
        for(i = 0; i < NUM_BANKFSM; i++) begin
            always_comb begin 
                pin_A_bank[i] = 0;
                // Default control signals (Deselect)
                {cke_bank[i], cs_n_bank[i], act_n_bank[i]} = 3'b111; 
                bg_bank[i] = '0;
                b_bank[i]  = '0;
                unique case(state[i])
                    Idle: begin // DES
                        {cke_bank[i], cs_n_bank[i]} = '1;
                    end
                    Activate : begin
                        if(chCMDAvailable && !bankState[bankIndex[i]] && // In Activation, the Interface of CMD is for Row operation, especially for ACT.
                        targetBanks[i]) begin  
                            {cs_n_bank[i], act_n_bank[i]} = '0;
                            cke_bank[i] = 1;
                            pin_A_bank[i] = { {(COMMAND_WIDTH-RWIDTH){1'b0}}, servingReq[i].mem_addr.row };
                            bg_bank[i] = servingReq[i].mem_addr.bankgroup;
                            b_bank[i]  = servingReq[i].mem_addr.bank;
                        end else begin
                            {cke_bank[i], cs_n_bank[i]} = '1;
                        end
                    end
                    LoadTimer: begin                        // In LoadTimer, the Interface of CMD is De-selected.
                        {cke_bank[i], cs_n_bank[i]} = '1;
                    end

                    Read: begin                             // In Read, the Interface of CMD is for Column operation, especially for READ.
                        if(chCMDAvailable && chCMDDQAvailable && rbuf_available && !bankState[bankIndex[i]] && rbufWindowAvailable
                        && targetBanks[i]) begin
                            if(servingReq[i].AutoPreCharge) begin
                                pin_A_bank[i] = {{(COMMAND_WIDTH-CWIDTH){1'b0}}, servingReq[i].mem_addr.col};
                                {cke_bank[i], act_n_bank[i], pin_A_bank[i][16], pin_A_bank[i][14], pin_A_bank[i][10]} = '1;
                                {cs_n_bank[i], pin_A_bank[i][15]} = '0;
                                bg_bank[i] = servingReq[i].mem_addr.bankgroup;
                                b_bank[i] = servingReq[i].mem_addr.bank;
                            end else begin
                                pin_A_bank[i] = {{(COMMAND_WIDTH-CWIDTH){1'b0}}, servingReq[i].mem_addr.col};
                                {cke_bank[i], act_n_bank[i], pin_A_bank[i][16], pin_A_bank[i][14]} = '1;
                                {cs_n_bank[i], pin_A_bank[i][15], pin_A_bank[i][10]} = '0;
                                bg_bank[i] = servingReq[i].mem_addr.bankgroup;
                                b_bank[i] = servingReq[i].mem_addr.bank;
                            end
                        end else begin
                            {cke_bank[i], cs_n_bank[i]} = '1;
                        end
                    end
                    Write: begin                            // In Write, the Interface of CMD is for Column operation, especially for Write.
                        if(chCMDAvailable && chCMDDQAvailable && wbuf_available && !bankState[bankIndex[i]] &&
                        targetBanks[i]) begin
                            if(servingReq[i].AutoPreCharge) begin
                                pin_A_bank[i] = {{(COMMAND_WIDTH-CWIDTH){1'b0}}, servingReq[i].mem_addr.col};
                                
                                {cke_bank[i], act_n_bank[i], pin_A_bank[i][16], pin_A_bank[i][10]} = '1;
                                {cs_n_bank[i], pin_A_bank[i][15], pin_A_bank[i][14]} = '0;
                            
                                bg_bank[i] = servingReq[i].mem_addr.bankgroup;
                                b_bank[i]  = servingReq[i].mem_addr.bank;
                            end else begin
                                pin_A_bank[i] = {{(COMMAND_WIDTH-CWIDTH){1'b0}}, servingReq[i].mem_addr.col};
                                {cke_bank[i], act_n_bank[i], pin_A_bank[i][16]} = '1;
                                {cs_n_bank[i], pin_A_bank[i][15], pin_A_bank[i][14], pin_A_bank[i][10]} = '0;
                                bg_bank[i] = servingReq[i].mem_addr.bankgroup;
                                b_bank[i]  = servingReq[i].mem_addr.bank;
                            end
                        end else begin
                            {cke_bank[i], cs_n_bank[i]} = '1;
                        end
                    end
                    Refresh: begin                          // In Refresh, the Interface of CMD is for Refresh Command.
                        if(chCMDAvailable && !bankState[bankIndex[i]] && schedRefACK) begin
                            {cke_bank[i], cs_n_bank[i], act_n_bank[i], 
                            pin_A_bank[i][16], pin_A_bank[i][15], pin_A_bank[i][14]} = 6'b101000;
                        end else begin
                            {cke_bank[i], cs_n_bank[i]} = '1;
                        end
                    end
                    Precharge: begin                        // In Precharge, the Interface of CMD is for Precharge Command.
                        if(chCMDAvailable && !bankState[bankIndex[i]] && targetBanks[i]) begin
                            {cke_bank[i], act_n_bank[i], pin_A_bank[i][15]} = '1;
                            {cs_n_bank[i], pin_A_bank[i][16], pin_A_bank[i][14], pin_A_bank[i][10]} = '0;
                            bg_bank[i] = servingReq[i].mem_addr.bankgroup;
                            b_bank[i]  =  servingReq[i].mem_addr.bank;
                        end else begin
                            {cke_bank[i], cs_n_bank[i]} = '1;
                        end
                    end
                    default : begin
                    end
                endcase
            end
        end
    endgenerate

    always_comb begin
        cke   = '1;
        cs_n  = '1;
        bg    = '0;
        b     = '0;
        act_n = '1;
        par   = '0;
        pin_A = '0;
        for(int a = 0; a< NUM_BANKFSM; a++)begin
            if(targetBanks[a]) begin
                cke   = cke_bank[a];
                cs_n  = cs_n_bank[a];
                par   = '0;
                act_n = act_n_bank[a];
                pin_A = pin_A_bank[a];
                bg    = bg_bank[a];
                b     = b_bank[a];
            end
        end
    end
    //////////////////////////////////////////////////////////////////////////
endmodule
