`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////////////
//      DualPortBuffer
//
//      Role:
//          Simple dual-port buffer providing independent read and write access.
//          Designed to decouple producer and consumer operating on the same clock.
//
//      Functionality:
//          - One write port and one read port.
//          - Write and read addresses are supplied explicitly.
//          - Read and write can occur in the same cycle to different addresses.
//
//      Typical Usage:
//          - Write side  : PHY / Memory / Backend logic
//          - Read side   : Cache / Frontend / Consumer logic
//
//      Interface Semantics:
//          - we        : Write enable (writes wdata into mem[writePtr])
//          - re        : Read enable (captures mem[readPtr] into rdata)
//          - readPtr   : Address for read operation
//          - writePtr  : Address for write operation
//
//      Design Notes:
//          - This is NOT a FIFO:
//              * No internal pointer management
//              * No full/empty detection
//          - Pointer generation and hazard avoidance must be handled externally.
//          - Read data is registered (1-cycle latency).
//
//      Reset Behavior:
//          - Memory array is cleared on reset (simulation-friendly).
//          - rdata is reset to zero.
//
//      Author  : Seongwon Jo
//      Created : 2026.02
//////////////////////////////////////////////////////////////////////////////////////////

module DualPortBuffer #(
    parameter int BufferDepth = 8,
    parameter type DataEntry = logic

    )(
    input logic clk, rst,
    input logic re, we,
    input logic [$clog2(BufferDepth)-1:0] readPtr,
    input logic [$clog2(BufferDepth)-1:0] writePtr,
    input DataEntry wdata,
    output DataEntry rdata
);

    DataEntry mem [BufferDepth-1:0];

    
    // write port (PHY-side)
    always_ff @(posedge clk or negedge rst) begin
        if (!rst) begin
            for (int i = 0; i < BufferDepth; i++) begin
                mem[i] <= '0;
            end
        end else if (we) begin
            mem[writePtr] <= wdata;
        end
    end

    // read port (Cache-side)
    always_ff @(posedge clk or negedge rst) begin
        if (!rst) begin
            rdata <= '0;
        end else if (re) begin
            rdata <= mem[readPtr];
        end
    end

`ifdef ASSERTION
    DualPortBufferRAW : assert property ( @(posedge clk) disable iff(!rst)
        !(re && we && (readPtr == writePtr))
    ) else $error("DualPortBuffer: RAW Hazard occured.");
`endif
endmodule
