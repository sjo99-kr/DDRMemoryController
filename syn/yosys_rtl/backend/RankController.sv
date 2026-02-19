`timescale 1ns / 1ps

//------------------------------------------------------------------------------
//      RankController
//
//      Role:
//          Rank-level control block inside the Memory Controller Backend.
//          Coordinates request scheduling, DRAM command generation, and
//          buffer-level handshaking for a single DRAM rank.
//
//      Position in Architecture:
//
//          MemoryController Backend
//                      |
//                      V
//        +---------------------------+
//        |     RankController        |
//        |  (per Channel, per Rank) |
//        +---------------------------+
//                      |
//                      V
//        +---------------------------+
//        |   RankSched  |  RankFSM   |
//        +---------------------------+
//                      |
//                      V
//               DDR IF CMD BUS
//
//      High-level Responsibilities:
//          1) Accept read/write requests from MC Frontend.
//          2) Queue, age, and arbitrate rank-level memory requests.
//          3) Coordinate with Channel Scheduler for CMD / DQ grants.
//          4) Manage rank-level DRAM timing constraints.
//          5) Issue DRAM commands (ACT / RD / WR / PRE / REF).
//          6) Coordinate Auto-Precharge with data-buffer completion.
//          7) Interface with Read/Write Data Buffers.
//
//      Internal Structure:
//          - RankSched:
//              * Request queueing and arbitration.
//              * Open-page awareness and aging-based prioritization.
//              * Decides which request should be issued next.
//          - RankFSM:
//              * Enforces DRAM timing constraints (tRCD, tRP, tWR, tRFC).
//              * Generates DDR command/address signals.
//              * Tracks row/bank state and refresh state.
//
//      Request Flow:
//          Frontend -> RankController -> RankSched -> RankFSM -> DRAM
//
//      Buffer Interaction:
//          - Read/Write buffers are decoupled via explicit ACK signals.
//          - RankFSM issues buffer-level ACKs when command is accepted.
//          - Auto-Precharge completion is synchronized with buffer ACKs.
//
//      Scheduling Assumptions:
//          - Channel Scheduler grants CMD / CMDDQ opportunities.
//          - RankController does not perform channel-level arbitration.
//          - Only one request is issued per cycle per rank.
//
//      Timing Model:
//          - Cycle-accurate DRAM timing abstraction.
//          - Electrical and PHY timing are handled outside (PHYController).
//
//      What this module DOES:
//          - Rank-level request coordination.
//          - Timing-aware command issuance.
//          - Buffer and scheduler synchronization.
//
//      What this module DOES NOT do:
//          - PHY-level DQ/DQS generation.
//          - Channel-level arbitration across ranks.
//          - Global memory reordering.
//
//      Design Notes:
//          - This module is performance-critical and timing-sensitive.
//          - Split between RankSched and RankFSM improves clarity and reuse.
//          - Intended to scale with NUMBANK / NUMBANKGROUP parameters.
//
//      Author  : Seongwon Jo
//      Created : 2026.02
//------------------------------------------------------------------------------
import MemoryController_Definitions::*;


module RankController (
    input logic clk, rst,
                                                          //        Input from MC FrontEnd       //
    input logic [32-1:0] RankReqMemAddr,                                            
    input logic [MEM_IDWIDTH-1:0] RankReqId,                                 
    input logic [MEM_USERWIDTH-1:0] RankReqUser,                             
    input logic RankReqType,
    input logic RankReqValid,
                                                           //        Output to MC FrontEnd       //                                       
    output logic RankReadReqReady, 
    output logic RankWriteReqReady,

                                                          //   Input from Channel Scheduler     //
    input logic chSchedCMDGranted,
    input logic chSchedDQGranted,
    input logic chSchedWriteMode,                         // Channel Mode signal for Read / Write request

                                                          //    Output to Channel Scheduler     //
    output logic chSchedRdReady,                          // Read Ready signal, Valid when there is any Read request in Request Que.
    output logic chSchedWrReady,                          // Write Ready signal, Valid when there is any Write request in Request Que.
    output logic chSchedRdWrACK,                          // RD/WR ACK signal for Read/Write request, ACK when FSM issues Read/Write CMD.
    output logic chSchedCMDACK,                           // CMD ACK signal for any CMD, ACK when FSM issues any kind of CMD.
    output logic chSchedFSMWait,                          // FSMWait signal, Valid when FSM waits for row timing constraints. (e.g., tRP, tRCD, tRFC)
    output logic chSchedCCDType,                          // CAS-to-CAS timing type, 1 for tCCD_Short , 0 for tCCD_Long
    output logic [$clog2(READCMDQUEUEDEPTH)-1:0] ReadReqCnt,  // Num. of read Requests in Request Que.
    output logic [$clog2(WRITECMDQUEUEDEPTH)-1:0] WriteReqCnt, // Num. of write Requests in Request Que.
    output wire chSchedRankIdle,
    input wire chSchedTransReady,
                                                          //        Input from MEM Buffer       //
    input rdBufAvailable,                                 // Valid when there is any empty entry in read buffer
    input logic rdBufRankAvailable,    
    input wrBufAvailable,                                 // Valid when there is any empty entry in write buffer
    input logic bufReadPreACK,                            // Valid when Read data (last) is received.
    input logic bufWritePreACK,                           // Valid when Write data (last) is sent.
    input logic [BKWIDTH + BGWIDTH - 1 : 0] bufBankPre,   // AutoPrecharge-related BankGroup, Bank Information to FSM.

                                                          //        Output to MEM Buffer        //
    output logic [MEM_IDWIDTH-1:0] bufReadReqId,          // When RankFSM sends RD Req., it sends Req. ID to RD MEM Buffer for ready to receive.
    output logic [MEM_IDWIDTH-1:0] bufWriteReqId,         // When RankFSM sends WR Req., it sends Req. ID to WR MEM Buffer for ready to send.
    output logic [MEM_USERWIDTH-1:0] bufReadReqUser,      // When RankFSM sends RD Req., it sends Req. User to RD MEM Buffer for ready to receive.
    output logic [MEM_USERWIDTH-1:0] bufWriteReqUser,     // When RankFSM sends WR Req., it sends Req. User to WR MEM Buffer for ready to send.
    output logic bufReadReqACK,                           // When RankFSM sends RD Req., it sends Req. valid to RD MEM Buffer for ready to receive.
    output logic bufWriteReqACK,                          // When RankFSM sends WR Req., it sends Req. valid to WR MEM Buffer for ready to send.    
    output logic [32-1:0] bufReqACKAddr,                   // When RankFSM sends RD/WR Req., it sends Req. Addr. to RD/WR Mem Buffer.

    output logic issuable,
    // Memory channel, PHY - side 
    output logic cke, cs_n, par, act_n,
    output logic [COMMAND_WIDTH-1:0] pin_A,
    output logic [BGWIDTH-1:0] bg,
    output logic [BKWIDTH-1:0] b
    );
    
    //----------- Internal Wires for RankSched <-> RankFSM --------------//
    logic [NUM_BANKFSM-1:0] fsmWait;
    wire  [NUM_BANKFSM-1:0] fsmIdle;
    logic [NUM_BANKFSM-1:0] fsmIssue;
    
    logic fsmRefreshACK, fsmChSchedAck;
    logic [MEM_IDWIDTH-1 :0] fsmBufWrReqId, fsmBufRdReqId;
    logic [MEM_USERWIDTH-1:0] fsmBufWrReqUser, fsmBufRdReqUser;
    logic fsmBufWrReqIssued, fsmBufRdReqIssued;

    logic fsmWrBufValid, fsmRdBufValid;
    logic refresh, chSchedAvailableCMD, chSchedAvailableCMDDQ;
    logic [44-1:0] fsmIssuedReq;


    RankSched  RankScheduler(
        .clk(clk), .rst(rst),

        .RankReqMemAddr(RankReqMemAddr), .RankReqId(RankReqId), .RankReqUser(RankReqUser),
        .RankReqType(RankReqType), .RankReqValid(RankReqValid), .RankReadReqReady(RankReadReqReady),
        .RankWriteReqReady(RankWriteReqReady),

        .chSchedCMDOnlyValid(chSchedCMDGranted), .chSchedCMDDQValid(chSchedDQGranted),
        .WriteMode(chSchedWriteMode),
        .chSchedRdReady(chSchedRdReady), .chSchedWrReady(chSchedWrReady), .chSchedACK(chSchedRdWrACK), .chSchedIdle(chSchedFSMWait),
        .chSchedReadReqCnt(ReadReqCnt), .chSchedWriteReqCnt(WriteReqCnt),

        .rdBufAvailable(rdBufAvailable), .wrBufAvailable(wrBufAvailable),

        .readBufReqId(bufReadReqId), .readBufReqUser(bufReadReqUser), .readBufReqACK(bufReadReqACK),
        .writeBufReqId(bufWriteReqId), .writeBufReqUser(bufWriteReqUser), .writeBufReqACK(bufWriteReqACK),

        .fsmIdle(fsmIdle), .fsmWait(fsmWait), .chSchedTransReady(chSchedTransReady),
        .fsmRefreshAck(fsmResfreshACK), .fsmChSchedAck(fsmChSchedAck),

        .fsmBufWrReqId(fsmBufWrReqId), .fsmBufWrReqUser(fsmBufWrReqUser), .fsmBufRdReqId(fsmBufRdReqId), .fsmBufRdReqUser(fsmBufRdReqUser), 
        .fsmBufWrReqIssued(fsmBufWrReqIssued), .fsmBufRdReqIssued(fsmBufRdReqIssued),

        .fsmWrBufValid(fsmWrBufValid), .fsmRdBufValid(fsmRdBufValid),
        .fsmIssue(fsmIssue), .fsmIssuedReq(fsmIssuedReq), .issuable(issuable),
        
        .refresh(refresh), .chSchedAvailableCMD(chSchedAvailableCMD), .chSchedAvailableCMDDQ(chSchedAvailableCMDDQ)
    );

    RankFSM  RankFiniteStateMachine (
        .clk(clk), .rst(rst), .chMode(chSchedWriteMode),
        
        .ReadPreAck(bufReadPreACK), .WritePreAck(bufWritePreACK),
        .rbuf_available(fsmRdBufValid), .wbuf_available(fsmWrBufValid), .bufBankPre(bufBankPre),
        .rbufWindowAvailable(rdBufRankAvailable),

        .bufWriteReqIssued(fsmBufWrReqIssued), .bufWriteReqId(fsmBufWrReqId), .bufWriteReqUser(fsmBufWrReqUser),
        .bufReadReqIssued(fsmBufRdReqIssued), .bufReadReqId(fsmBufRdReqId) , .bufReadReqUser(fsmBufRdReqUser),

        .schedReq(fsmIssuedReq), .schedValid(fsmIssue), .refresh(refresh), 
        .schedIdle(fsmIdle), .schedRefACK(fsmResfreshACK),

        .chCMDAvailable(chSchedAvailableCMD), .chCMDDQAvailable(chSchedAvailableCMDDQ),
        .fsmWait(fsmWait), .chSchedRdWrACK(fsmChSchedAck), .chSchedCMDACK(chSchedCMDACK),
        .CCDShort(chSchedCCDType),

        .bufReqACKAddr(bufReqACKAddr),
        .cke(cke), .cs_n(cs_n), .par(par), .act_n(act_n),
        .pin_A(pin_A), .bg(bg), .b(b)
    );
endmodule
