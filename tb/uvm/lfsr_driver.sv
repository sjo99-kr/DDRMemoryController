module lfsr_driver #(
    parameter int LENGTH = 4,
    parameter logic [LENGTH-1:0] KEY = '1
)(
    input  logic clk, rst_n,
    input  logic enable,
    output logic valid,
    output logic [LENGTH-1:0] random
);

    logic [LENGTH-1:0] lfsr;
    logic feedback;

    // ----------------------------
    // TAP SELECTION
    // ----------------------------
    generate
        if (LENGTH == 4)
            assign feedback = lfsr[3] ^ lfsr[2];
        else if (LENGTH == 32)
            assign feedback = lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0];
        else if (LENGTH == 64)
            assign feedback = lfsr[63] ^ lfsr[62] ^ lfsr[60] ^ lfsr[59];
        else
            initial $error("Unsupported LFSR LENGTH");
    endgenerate

    // ----------------------------
    // LFSR GENERATION
    // ----------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lfsr  <= (KEY == '0) ? '1 : KEY;
            valid <= 1'b0;
        end else if (enable) begin
            lfsr  <= {lfsr[LENGTH-2:0], feedback};
            valid <= 1'b1;
        end else begin
            valid <= 1'b0;
        end
    end

    assign random = lfsr;

endmodule