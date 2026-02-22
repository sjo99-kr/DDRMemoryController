`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////////////
//      CMDGrantScheduler
//
//      Role:
//          Channel-level CMD bus arbitration logic.
//          Selects exactly one RankController to issue a DDR command
//          based on request availability and queue depth.
//
//      Scheduling Policy:
//          - Queue-depth-aware priority:
//              * Rank with the deepest request queue is preferred.
//          - Random tie-breaking:
//              * When multiple ranks have equal queue depth,
//                a pseudo-random selection is applied using LFSR.
//
//      Constraints:
//          - Only one RankController can be granted per cycle.
//          - RankControllers waiting on internal timing (i.e., tRCD, tRP, tRFC) are excluded.
//          - Separate handling for READ and WRITE modes.
//
//      Outputs:
//          - CMDGrantVector : One-hot grant signal per rank.
//          - rankTransition : Asserted when CMD grant switches between ranks
//                             (used for tRTR enforcement).
//
//      Notes:
//          - This module performs arbitration only.
//          - DDR timing constraints are enforced by external turnaround logic.
//
//      Author  : Seongwon Jo
//      Created : 2026.02
//////////////////////////////////////////////////////////////////////////////////////////

module CMDGrantScheduler#(
    parameter int NUMRANK = 4,
    parameter int READCMDQUEUEDEPTH  = 8,
    parameter int WRITECMDQUEUEDEPTH = 8
)(
    input logic clk, rst,       

    input logic [NUMRANK-1:0] readyRdVector,                                    //  1. Ready Singals from RankController (READ)      //
    input logic [NUMRANK-1:0] readyWrVector,                                    //  2. Ready signals from RankController (WRITE)     //
    input logic [NUMRANK-1:0] fsmWaitVector,                                    //  3. FSMWait Signals from RankController           //
    input logic [$clog2(READCMDQUEUEDEPTH)-1:0] readReqCnt [NUMRANK-1:0],       //  4. Per-RankController Read Req Cnt               //
    input logic [$clog2(WRITECMDQUEUEDEPTH)-1:0] writeReqCnt [NUMRANK-1:0],     //  5. Per-RankController Write Req Cnt              //
    input logic grantACK,                                                       //  6. GrantACK signal from RankController           //
    input logic writeMode,                                                      //  7. Channel Mode                                  //
    input logic CMDRankTurnaround,                                              //  8. Rank Trunaround Available Signal (free tRTR)  //
    output logic [NUMRANK-1:0] CMDGrantVector,                                  //  9. Granted signal for specific RankCtrl.         //
    output logic rankTransition                                                 // 10. Rank Change signal                            //
    
    );

    logic [NUMRANK-1:0] lfsr;                                                   // Linear Feedback Shift Register, Providing Randomess
    logic [NUMRANK-1:0] masked;                                                 // Masked bits for making randomess in valid vector 
    logic tie_break;                                                            // tie_break for the case of same requests in all ranks.
    
    logic [NUMRANK-1:0] next_cmd;                                               // Next granted RankController, One-hot vector 
    logic [NUMRANK-1:0] prev_cmd;                                               // Current granted RankController, One-hot vector                       
    logic [NUMRANK-1:0] avail;                                                  // Available RankController, based on Ready to Issue CMD, FSMWait signal

    logic [$clog2(READCMDQUEUEDEPTH)-1:0] RDmaxCnt, RDminCnt;                   // Read Max Counter / Read Max Counter
    logic [$clog2(WRITECMDQUEUEDEPTH)-1:0] WRmaxCnt, WRminCnt;                  // Write Max Counter / Write Min Counter

    logic [$clog2(NUMRANK)-1:0] maxIndex;                                       // Max Req Depth Index / Min Req Depth Index


    //------------------------------------------------------------------------------
    //     Pseudo-Random Generator (LFSR)
    //
    //      - Generates a simple pseudo-random sequence
    //      - Used only for tie-breaking when multiple ranks have equal priority
    //------------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst) begin : MakingPseudoRandomLFSR
        // Pseudo random generator
        if (!rst) begin
            lfsr <= {{(NUMRANK-1){1'b0}}, 1'b1};
        end else begin
            lfsr <= {lfsr[NUMRANK-2:0],
                    lfsr[NUMRANK-1] ^ lfsr[NUMRANK-2]};
        end
    end : MakingPseudoRandomLFSR

    `ifdef DISPLAY
    //------------------------------------------------------------------------------
    //      Debug Logic for CMD Grant Observation
    //
    //      - Extracts the granted rank index from CMDGrantVector
    //      - Reports rank-to-rank transitions on CMD bus
    //------------------------------------------------------------------------------
    logic [$clog2(NUMRANK):0] granted_rank;
    always_comb begin : GrantedRankCheckForDebug
        granted_rank  = 0;
        for(int i =0; i< NUMRANK; i++)begin
            if(CMDGrantVector[i] == 1)begin
                granted_rank = i;
            end
        end
    end : GrantedRankCheckForDebug

    always_ff@(posedge clk) begin : GrantedRankDisplayForDebug
        if(rankTransition) begin
            $display("[%0t] CMDGrantScheduler | RANK TRANSITION OCCURED", $time);            
        end
    end : GrantedRankDisplayForDebug
    `endif

    //-------------------------------------------------------------------//


    //------------------------------------------------------------------------------
    //      CMD Bus Arbitration Logic
    //
    //      Policy:
    //          1) Filter available RankControllers based on:
    //              - Read/Write mode
    //              - FSM Wait & Idle state (From Rank Controllers)
    //          2) Select the rank with the deepest request queue.
    //          3) If multiple ranks have equal depth, apply random tie-breaking.
    //
    //      Result:
    //          - next_cmd is a one-hot vector indicating the selected rank
    //------------------------------------------------------------------------------
    always_comb begin : CalculatingNextTargetRank
        next_cmd = '0;
        avail = '0;
        
        if(writeMode) avail = readyWrVector & ~fsmWaitVector;    
        else avail = readyRdVector & ~ fsmWaitVector;

        masked = avail & lfsr;
        tie_break = 1;

        WRmaxCnt = 0;
        RDmaxCnt = 0;

        WRminCnt = {($clog2(WRITECMDQUEUEDEPTH)){1'b1}};
        RDminCnt = {($clog2(READCMDQUEUEDEPTH)){1'b1}};;

        maxIndex = 0;

        // If there are candidates for scheduling, check who is most deepest queue?
        if(avail != '0) begin
            // There are two types of request, write and read, which are managed separately.
            if(writeMode)begin
                for(int i = 0; i < NUMRANK; i++) begin
                    if(avail[i]) begin
                        if(WRminCnt > writeReqCnt[i]) begin
                            WRminCnt = writeReqCnt[i];
                        end
                        if(WRmaxCnt < writeReqCnt[i]) begin
                            maxIndex = i;
                            WRmaxCnt = writeReqCnt[i];
                        end
                    end
                end
                if(WRmaxCnt == WRminCnt) begin      // If the requests in ranks are same, then we need to break the tie
                    tie_break = 1;
                end else begin                      // Else, then just set the deepest target rank
                    tie_break = 0;
                    next_cmd[maxIndex] = 1;
                end
            end else begin
                for(int i =0; i < NUMRANK; i++) begin
                    if(avail[i]) begin
                        if(RDminCnt > readReqCnt[i]) begin
                            RDminCnt = readReqCnt[i];
                        end
                        if(RDmaxCnt < readReqCnt[i]) begin
                            maxIndex = i;
                            RDmaxCnt = readReqCnt[i];
                        end
                    end
                end
                if(RDmaxCnt == RDminCnt) begin
                    tie_break = 1;
                end else begin
                    tie_break = 0;
                    next_cmd[maxIndex] = 1;
                end
            end

            if(tie_break)begin              // If the tie_break is required, we utilize the pesudo random value from LFSR
                if(writeMode) begin
                    if(&masked == 0) begin         // If the masked from random value is set as zero-vector,
                        next_cmd = avail & (~avail + 1);    // Then just select "Least Significant Bit(LSB) Priority Arbiter"
                    end else begin
                        next_cmd = masked & (~masked + 1);  // If the masked is not set as zero-vector, Then just select "LSB Priority Arbiter" on that.
                    end 
                end else begin
                    if(&masked) begin
                        next_cmd = avail & (~avail +1);
                    end else begin
                        next_cmd = masked & (~masked + 1);
                    end
                end
            end
        end
    end : CalculatingNextTargetRank


    //------------------------------------------------------------------------------
    //      Grant Register Update
    //
    //      - Latches next_cmd into CMDGrantVector
    //      - Grant is updated only when:
    //          * No active grant exists (initial grant), or
    //          * Previous grant is acknowledged by RankController (grantACK)
    //------------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst) begin : Arbitration
        if (!rst) begin
            CMDGrantVector <= '0;
        end else begin
            if(CMDRankTurnaround) begin         // Change of arbitration is only valid when it is free from tRTR timing constraint.
                if(CMDGrantVector == '0 && (next_cmd != '0)) begin     
                    CMDGrantVector <= next_cmd;                   // If there is no grant in current arbitration, but there is valid in next_cmd.    
                    `ifdef DISPLAY
                        $display("[%0t] CMDGrantScheduler | Rank-%0d FSM granted for CMD Bus", $time, granted_rank);
                    `endif 
                end

                else if(grantACK)begin          // GrantACK comes from the target rank controller.
                    if(next_cmd != '0) begin    // If the grantACK comes, then we need to select new target rank for fairness.
                        CMDGrantVector <= next_cmd;
                        `ifdef DISPLAY
                            $display("[%0t] CMDGrantScheduler | Rank-%0d FSM granted for CMD Bus", $time, granted_rank);
                        `endif 
                    end else begin
                        CMDGrantVector <= '0;
                    end
                end else if(avail == '0) begin
                    CMDGrantVector <= '0;
                end else begin
                    if( |(CMDGrantVector & next_cmd) == 1) begin // if there is no grantACK yet, but the target rank is still valid, then we maintain that target rank.
                        CMDGrantVector <= CMDGrantVector;
                    end else begin
                        CMDGrantVector <= next_cmd; // but the target rank is not valid, then we change the target rank.
                    end
                end
            end
        end
    end : Arbitration

    //------------------------------------------------------------------------------
    //   Rank Transition Detection
    //
    //      - Detects changes in CMD bus ownership between cycles
    //      - Used by channel-wide timing logic to enforce tRTR constraint
    //------------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst) begin : RankTransitionDetecting
        if (!rst)
            prev_cmd <= '0;
        else begin
            prev_cmd <= CMDGrantVector;
        end
    end : RankTransitionDetecting

    assign rankTransition = (prev_cmd != CMDGrantVector);

    //-------------------------------------------------------------------//

endmodule
