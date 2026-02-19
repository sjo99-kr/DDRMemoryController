`timescale  1ns / 1ps

`define TESTBENCH

package svut_if;

    parameter int ERRORCNTWIDTH = 10;
    parameter int DEADLOCKCNT = 1024;
    parameter int TBREADREQWIDTH = 10;
    parameter int TBWRITEREQWIDTH = 10;



    `ifdef TESTBENCH
    localparam int TB_AXI_ADDRWIDTH = 32;
    localparam int TB_AXI_IDWIDTH   = 4;
    localparam int TB_AXI_USERWIDTH = 1;
    localparam int TB_AXI_DATAWIDTH = 64;
    localparam int TB_BURST_LENGTH  = 8;
    localparam int TB_NUMBANK       = 4;
    localparam int TB_NUMBANKGROUP  = 4;
    localparam int TB_NUMRANK       = 4;
    localparam int TB_BGWIDTH       = 2;
    localparam int TB_tRCD          = 16;
    localparam int TB_NUM_BANKFSM   = 16;
    `endif
    typedef struct packed{
        logic [TB_AXI_IDWIDTH-1:0] id;
        logic [TB_AXI_USERWIDTH-1:0] user;
        time sendTime;
        logic [$clog2(DEADLOCKCNT):0] cnt;
    } TB_READREQENTRY;

    typedef struct packed {
        logic [TB_AXI_IDWIDTH-1:0] id;
        logic [TB_AXI_USERWIDTH-1:0] user;
        time sendTime;
        logic [$clog2(DEADLOCKCNT):0] cnt;
    } TB_WRITEREQENTRY;


    typedef struct packed {
        logic [TB_BGWIDTH-1:0] bankgroup;
        logic valid;
    } TB_CCDUNIT;

    typedef struct {
        logic [TB_tRCD-1:0] actTiming [TB_NUM_BANKFSM-1:0];
    } TB_RCDRANK;

    typedef struct {
        logic valid;
        logic [TB_AXI_ADDRWIDTH-1:0] addr;
        logic [TB_AXI_IDWIDTH-1:0] id;
        logic [TB_AXI_USERWIDTH-1:0] user;
    } cache_readReq_Issue;

    typedef struct {
        logic valid;
        logic [TB_AXI_ADDRWIDTH-1:0] addr;
        logic [TB_AXI_IDWIDTH-1:0]   id;
        logic [TB_AXI_USERWIDTH-1:0] user;
    } cache_writeReq_Issue;
    

    typedef struct {
        logic valid;
        logic [TB_AXI_IDWIDTH-1:0] id;
        logic [TB_AXI_USERWIDTH-1:0] user;
        logic [TB_AXI_DATAWIDTH*TB_BURST_LENGTH-1:0] data;
    } cache_readResp_Receive;


    typedef struct {
        logic valid;
        logic [TB_AXI_IDWIDTH-1:0] id;
        logic [TB_AXI_USERWIDTH-1:0] user;
    } cache_writeACK_Receive;




    typedef enum logic [2:0] {
        IDLE,
        ACTIVATE,
        READ,
        AUTOREAD,
        WRITE,
        AUTOWRITE,
        PRECHARGE,
        REFRESH
    } commandType;

    typedef struct {
        commandType cmdType;
        logic [$clog2(TB_NUMBANK)-1:0] bk;
        logic [$clog2(TB_NUMBANKGROUP)-1:0] bg;
        logic [$clog2(TB_NUMRANK)-1:0] rank;
        logic valid;
        time Issue_time;
    } dram_cmd_Issue;

    typedef struct {
        logic valid;
        time Issue_time;
    } dram_data_Issue;

    typedef struct {
        logic valid;
        time Receive_time;
    } dram_data_Receive;


    function void error_msg(string msg);

        $display("[%0t] %s", $time, msg);

    endfunction

endpackage