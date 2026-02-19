`timescale 1 ns / 1 ps

import MemoryController_Definitions::*;

module driver#
(
    parameter int TIMEOUT = 100,
    parameter int AXI_DATAWIDTH = 64,
    parameter int AXI_ADDRWIDTH = 32,
    parameter int AXI_IDWIDTH = 4,
    parameter int AXI_USERWIDTH = 1,
    parameter int MODELENGTH = 32,
    parameter int BURST_LENGTH = 8
)(
    //      COMMON      //
    input wire clk, rst_n, 

    //   TESTBENCH INTERFACE  //
    input wire testEnable,
    
    //      AXI BUS SIGNALS      //
    //       Read Request        //
    output wire  [AXI_IDWIDTH-1   :0]   ar_id,
    output wire  [AXI_USERWIDTH-1 :0] ar_user,
    output wire  [AXI_ADDRWIDTH-1 :0] ar_addr,
    output logic ar_valid,
    input  wire  ar_ready,

    //       Write Request       //
    output wire [AXI_IDWIDTH-1:0] aw_id,
    output wire [AXI_USERWIDTH-1:0] aw_user,
    output wire [AXI_ADDRWIDTH-1:0] aw_addr,
    output logic aw_valid,
    input wire aw_ready,

    output wire [AXI_IDWIDTH-1:0] w_id,
    output wire [AXI_USERWIDTH-1:0] w_user,
    output wire [AXI_DATAWIDTH-1:0] w_data,
    output wire w_last,
    output wire [AXI_DATAWIDTH/BURST_LENGTH -1: 0] w_strb,
    output logic w_valid,
    input wire w_ready
);

    localparam int IDVECTORWIDTH   = (1 << AXI_IDWIDTH);
    localparam int USERVECTORWIDTH = (1 << AXI_USERWIDTH); 

    logic [IDVECTORWIDTH-1:0] Ar_free_id_vector [USERVECTORWIDTH-1:0];
    logic [IDVECTORWIDTH-1:0] Aw_free_id_vector [USERVECTORWIDTH-1:0];
    logic [IDVECTORWIDTH-1:0] W_free_id_vector [USERVECTORWIDTH-1:0];


    logic [AXI_IDWIDTH-1:0] w_id_driver;
    logic [AXI_USERWIDTH-1:0] w_user_driver;
    logic [AXI_DATAWIDTH-1:0] w_data_driver;
    logic w_valid_driver;

    logic [AXI_IDWIDTH-1:0] ar_id_driver;
    logic [AXI_USERWIDTH-1:0] ar_user_driver;
    logic [AXI_ADDRWIDTH-1:0] ar_addr_driver;
    logic ar_valid_driver;

    logic [AXI_IDWIDTH-1:0] aw_id_driver;
    logic [AXI_USERWIDTH-1:0] aw_user_driver;
    logic [AXI_ADDRWIDTH-1:0] aw_addr_driver;
    logic aw_valid_driver;

    logic aw_mode;         // AW SEND OR NOT
    logic w_mode;          // W SEND OR NOT
    logic ar_mode;         // AR SEND OR NOT
    

    typedef enum logic {  
        REQUEST,
        DATABURST
    } state_t;

    state_t state;  // BURST TIMING CONSIDERATION


    logic modeEnable;
    logic modeAWValid;
    logic modeARValid;
    logic modeWValid;

    logic AwBusSent;
    logic ArBusSent;
    logic WBusSent;

    logic [MODELENGTH-1:0] modeAWRandom;
    logic [MODELENGTH-1:0] modeWRandom;
    logic [MODELENGTH-1:0] modeARRandom;


    logic awLFSRValid;
    logic wLFSRValid;
    logic arLFSRValid;
    

    logic [AXI_ADDRWIDTH-1:0] awAddrRandom;
    logic [AXI_ADDRWIDTH-1:0] arAddrRandom;
    logic [AXI_DATAWIDTH-1:0] wDataRandom;

    assign modeEnable = testEnable;

    lfsr_driver #(
        .LENGTH(MODELENGTH),
        .KEY(32'h2451_2312) 
    ) lfsr_ar_mode (
        .clk(clk), .rst_n(rst_n),
        .enable(modeEnable),
        .valid(modeARValid),
        .random(modeARRandom)
    );

    lfsr_driver #(
        .LENGTH(32),
        .KEY(32'h5351_3467)
    ) lfsr_aw_mode (
        .clk(clk), .rst_n(rst_n),
        .enable(modeEnable),
        .valid(modeAWValid),
        .random(modeAWRandom)
    );

    lfsr_driver #(
        .LENGTH(32),
        .KEY(32'h9825_5257) 
    ) lfsr_w_mode (
        .clk(clk), .rst_n(rst_n),
        .enable(modeEnable),
        .valid(modeWValid),
        .random(modeWRandom)
    );



    lfsr_driver #(
        .LENGTH(32),
        .KEY(32'h3521_1251)
    ) lfsr_ar_addr_gen(
        .clk(clk),
        .rst_n(rst_n),
        .enable(modeEnable),
        .valid(arLFSRValid),
        .random(arAddrRandom)
    );

    lfsr_driver #(
        .LENGTH(32),
        .KEY(32'h0101_9201)
    ) lfsr_aw_addr_gen(
        .clk(clk),
        .rst_n(rst_n),
        .enable(modeEnable),
        .valid(awLFSRValid),
        .random(awAddrRandom)
    );

    lfsr_driver #(
        .LENGTH(64),
        .KEY(32'h0101_9201)
    ) lfsr_w_data_gen(
        .clk(clk),
        .rst_n(rst_n),
        .enable(modeEnable),
        .valid(wLFSRValid),
        .random(wDataRandom)
    );

    always_ff@(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            w_mode  <= 0; 
            ar_mode <= 0;
            aw_mode <= 0;
        end else begin
            if(modeWValid && (state != DATABURST)) begin
                if(w_mode && w_ready) begin
                    w_mode <= w_mode;
                end else begin
                    w_mode  <= modeWRandom[0];
                end
            end 
            if(modeARValid) begin
                ar_mode <= modeARRandom[0];
            end
            if(modeAWValid) begin
                aw_mode <= modeAWRandom[0];
            end
        end
    end

    assign aw_valid_driver = testEnable ? aw_mode : 0;
    assign ar_valid_driver = testEnable ? ar_mode : 0;
    assign w_valid_driver =  testEnable ? w_mode  : 0;



    // finding LSB user, id in free id/user vector
    logic [AXI_USERWIDTH-1:0] targetArUser;
    logic [AXI_IDWIDTH-1:0] targetArId;

    logic [AXI_USERWIDTH-1:0] targetAwUser;
    logic [AXI_IDWIDTH-1:0] targetAwId;

    logic [AXI_USERWIDTH-1:0] targetWUser;
    logic [AXI_IDWIDTH-1:0] targetWId;

    logic [$clog2(BURST_LENGTH)-1:0] w_burst_cnt;

    logic ArvectorIdFull [USERVECTORWIDTH-1:0];
    logic AwvectorIdFull [USERVECTORWIDTH-1:0];
    logic WvectorIdFull  [USERVECTORWIDTH-1:0];

    always_comb begin
        for(int i = 0; i < USERVECTORWIDTH; i++) begin
            ArvectorIdFull[i] = !(|Ar_free_id_vector[i]);
            AwvectorIdFull[i] = !(|Aw_free_id_vector[i]);
            WvectorIdFull[i]  = !(|W_free_id_vector[i]);
        end
    end


    always_comb begin  : FindFreeUserId
        targetArUser = 0;
        targetAwUser = 0;
        targetWUser  = 0;
        targetArId = 0;
        targetAwId = 0;
        targetWId  = 0;
        
        for(int i = USERVECTORWIDTH; i>0; i--) begin
            if(!ArvectorIdFull[i]) begin
                targetArUser = i;
            end
            if(!AwvectorIdFull[i]) begin
                targetAwUser = i;
            end
            if(!WvectorIdFull[i]) begin
                targetWUser  = i;
            end
        end
        for(int j = IDVECTORWIDTH-1; j >= 0; j--) begin
            if(Ar_free_id_vector[targetArUser][j]) begin
                targetArId = j;
            end
            if(Aw_free_id_vector[targetAwUser][j]) begin
                targetAwId = j;
            end
            if( !(Aw_free_id_vector[targetWUser][j]) && W_free_id_vector[targetWUser][j]) begin
                targetWId = j;
            end
        end
    end : FindFreeUserId



    always_ff@(posedge clk or negedge rst_n) begin : RequestIdUserSetup
        if(!rst_n) begin
            for(int i = 0; i < USERVECTORWIDTH; i++) begin
                Ar_free_id_vector[i] <= '1;
                Aw_free_id_vector[i] <= '1;
                W_free_id_vector[i]  <= '1;
            end
        end else begin
            if(testEnable) begin
                if(AwBusSent) begin
                    Aw_free_id_vector[targetAwUser][targetAwId] <= 0;
                end 
                if(ArBusSent) begin
                    Ar_free_id_vector[targetArUser][targetArId] <= 0;
                end
                if(WBusSent)begin
                    W_free_id_vector[targetWUser][targetWId]    <= 0;
                end
            end else begin
                for(int i = 0; i < USERVECTORWIDTH; i++) begin
                    Ar_free_id_vector[i] <= '1;
                    Aw_free_id_vector[i] <= '1;
                    W_free_id_vector[i]  <= '1;
                end                
            end
        end
    end : RequestIdUserSetup


    ///         Read/Write Address-Bus Serving        ///
    always_ff @(posedge clk or negedge rst_n) begin : AwArBus_generate
        if(!rst_n) begin
            aw_addr_driver <= '0;
            ar_addr_driver <= '0;
        end else begin
            if(testEnable) begin
                if(ar_valid_driver) begin
                    ar_addr_driver <= arAddrRandom;
                end 

                if(aw_valid_driver) begin
                    aw_addr_driver <= awAddrRandom;
                end 
            end else begin

                aw_addr_driver <= '0;
                ar_addr_driver <= '0;                
            end
        end
    end : AwArBus_generate

    assign aw_id_driver   = targetAwId;
    assign ar_id_driver   = targetArId;
    assign aw_user_driver = targetAwUser;
    assign ar_user_driver = targetArUser;

    assign ar_valid       = ar_valid_driver && ar_ready;
    assign aw_valid       = aw_valid_driver && aw_ready;

    assign ArBusSent      = ar_valid;
    assign AwBusSent      = aw_valid;

    ///        Write Data-Bus Serving        ///
    always_ff@(posedge clk or negedge rst_n) begin : w_bus_stateCnt
        if(!rst_n) begin
            w_burst_cnt <= 0;
            state       <= REQUEST;
        end else begin
            if(w_valid_driver && w_ready) begin
                state <= DATABURST;
                if(state == DATABURST) begin
                    w_burst_cnt <= w_burst_cnt + 1;
                    if(w_burst_cnt == BURST_LENGTH-1) begin
                        state <= REQUEST;
                        w_burst_cnt <= 0;
                    end
                end
            end else begin
                state <= REQUEST;
                w_burst_cnt <= 0;
            end
        end
    end : w_bus_stateCnt

    always_ff@(posedge clk or negedge rst_n) begin : w_bus_generate
        if(!rst_n) begin
            w_data_driver <= '0;
            WBusSent      <= 0;
        end else begin
            if(w_valid_driver && w_ready) begin
                w_data_driver <= wDataRandom;
                if(w_last) begin
                    WBusSent <= 1;
                end else begin
                    WBusSent <= 0;
                end
            end 
        end
    end : w_bus_generate

    assign w_id_driver   = testEnable ? targetWId   : 0;
    assign w_user_driver = testEnable ? targetWUser : 0;
    assign w_valid       = (state == DATABURST) ? testEnable ? 1: 0: 0;


    ///     AXI BUS SIGNALS FOR DUT 
    assign ar_id   = testEnable ? ar_id_driver   : 0;
    assign ar_user = testEnable ? ar_user_driver : 0;
    assign ar_addr = testEnable ? ar_addr_driver : 0;

    assign aw_id   = testEnable ? aw_id_driver   : 0;
    assign aw_user = testEnable ? aw_user_driver : 0;
    assign aw_addr = testEnable ? aw_addr_driver : 0;

    assign w_id    = testEnable ? w_id_driver   : 0;
    assign w_user  = testEnable ? w_user_driver : 0;
    assign w_data  = testEnable ? w_data_driver : 0;
    assign w_last  = (w_burst_cnt == BURST_LENGTH-1) ? testEnable ? 1: 0 : 0;
    assign w_strb  = '1;


endmodule