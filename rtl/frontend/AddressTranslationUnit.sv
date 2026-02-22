`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////////////
//      AddressTranslationUnit (ATU)                                                                                                                                        
//
//      1) Translates AXI physical address into structured DRAM address fields          
//           (CH / RK / BG / BK / ROW / COL)                                            
//                                                                                      
//      2) Generates arbitration metadata for selecting a target FSM                    
//                                                                                      
//      FSM Mapping Policy:                                                             
//          - One FSM is allocated per (Channel, Rank) pair                             
//          - targetFSMIndex = {channel, rank}                                          
//          - Bank-level parallelism is handled inside each FSM                         
//                                                                                      
//      Current Address Translation Scheme:                                             
//          - Fixed address mapping (CH/RK/BG/BK/ROW/COL)                               
//          - XOR-based or hashed address translation can be added (TODO)               
// 
//      Author  : Seongwon Jo
//      Created : 2026.02
//////////////////////////////////////////////////////////////////////////////////////////

module AddressTranslationUnit #( 
    parameter int MEM_ADDRWIDTH              = 32,
    parameter int AXI_ADDRWIDTH              = 32,
    parameter int NUM_RANKEXECUTION_UNIT     = 8,
    parameter int NUM_RANKEXECUTION_UNIT_BIT = $clog2(NUM_RANKEXECUTION_UNIT),
    parameter int CHWIDTH                    = 1,
    parameter int RKWIDTH                    = 2,
    parameter type MemoryAddress             = logic
    )(
    input logic [AXI_ADDRWIDTH - 1 :0] readAddr, 
    input logic [AXI_ADDRWIDTH - 1 :0] writeAddr, 
    input logic [NUM_RANKEXECUTION_UNIT-1:0] readReady,
    input logic readValid,
    input logic [NUM_RANKEXECUTION_UNIT-1:0] writeReady,
    input logic writeValid,

    output logic [NUM_RANKEXECUTION_UNIT - 1:0] targetFSMVector, 
    output logic [NUM_RANKEXECUTION_UNIT_BIT-1:0] targetFSMIndex,
    output MemoryAddress requestMemAddr
    );

    
    localparam FSM_WIDTH = CHWIDTH + RKWIDTH;   // Number of Total FSM.
    // FSM resides on Per

    always_comb begin
        targetFSMVector = '0;
        targetFSMIndex   = '0;
        requestMemAddr  = '0;
        if(readValid) begin
            {requestMemAddr.channel, requestMemAddr.rank, requestMemAddr.bankgroup, requestMemAddr.bank,
             requestMemAddr.row, requestMemAddr.col} = readAddr;
             if(readReady[{requestMemAddr.channel ,requestMemAddr.rank}]) begin
                targetFSMIndex = readAddr[MEM_ADDRWIDTH-1 -: FSM_WIDTH];
                targetFSMVector[targetFSMIndex] = 1'b1;
             end 
        end else if(writeValid) begin
            {requestMemAddr.channel, requestMemAddr.rank, requestMemAddr.bankgroup, requestMemAddr.bank,
             requestMemAddr.row, requestMemAddr.col} = writeAddr;
             if(writeReady[{requestMemAddr.channel ,requestMemAddr.rank}]) begin
                targetFSMIndex = writeAddr[MEM_ADDRWIDTH-1 -: FSM_WIDTH];
                targetFSMVector[targetFSMIndex] = 1'b1;
             end
        end
    end
endmodule
