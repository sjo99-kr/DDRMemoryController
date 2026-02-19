`timescale 1ns / 1ps

//////////////////////////////////////////////////////////////////////////////////////////
//      DDR4Interface
//
//      Description:
//          SystemVerilog interface modeling DDR4 command/address, data,
//              and sideband signals.
//
//      Purpose:
//          - Provides a unified connection point between:
//              * Memory Controller (MC)
//              * DDR PHY / Memory Model
//          - Separates CA, DQ, and sideband signals using modports.
//
//  NOTE:
//      This DDR4 interface definition was inspired by publicly available DDR4 
//      interface examples in github. (https://github.com/ananthbhat94/DDR4MemoryController/blob/master/DDR4Interface.sv)
//      The structure and signal grouping have been adapted and simplified
//          to fit this project's controller and memory model architecture.
//
//      Author  : Seongwon Jo
//      Created : 2026.02
//////////////////////////////////////////////////////////////////////////////////////////
import MemoryController_Definitions::*;

interface DDR4Interface(
        /* verilator lint_off UNUSEDSIGNAL */

	input logic clk, rst
);



    //Definitions and LocalParameters
    // RAS, CAS, ACT, WE, CS -> command OPCode
    // Row bits -> 15, Col bits -> 10, bg,bk -> 2 bits 
    // Pin signal requires: RAS, CAS, WE ,CS, ACT 
    `define RAS pin_A[16]	//RAS Command Input
    `define CAS pin_A[15] 	//CAS Command Input
    `define WE	pin_A[14]	//WE Command Input
    `define BC	pin_A[12] 	//Burst Chop
    `define AP 	pin_A[10]  	// Auto Precharge 
    `define HIGH '1
    localparam LOW = '0;

    /**************Define Interface Signals*********************/
    //Address Inputs
    logic [COMMAND_WIDTH-1:0] pin_A; 

    //Control Signals
    logic act_n; //Activate Command Input
    logic [BGWIDTH-1:0] bg; //Bank Address Inputs
    logic [BKWIDTH-1:0] b; //Bank Group Address Inputs

    logic cke; //Clock Enable
    logic [NUMRANK-1:0] cs_n; //Chip Select
    logic [MEM_DATAWIDTH/BURST_LENGTH - 1 : 0] dm_n; //Input Data Mask




    //////////////////////////////////////////////////////////////////////
    //////////////////// OPCODE FOR DDR4 Memory //////////////////////////
    //////////////// CKE     ACT     RAS      CAS      WE     CS   ///////
    // ACTIVATE       1       0       X        X       X       0        //
    // PRECHARGE      1       1       0       1        0       0        //
    // READ           1       1       1       0        0       0        //
    // WRITE          1       1       1       0        1       0        //
    // REFRESH        1       1       0       0        0       0        //
    // NO-OPERATION   1       1       1       1        1       0        //
    //////////////////////////////////////////////////////////////////////

    //Data Signals
    logic [MEM_DATAWIDTH-1:0] pin_dq; //Bidirectional Data Bus
    logic dbi_n, udbi_n, ldbi_n; //Data Bus Inversion
    logic dqs_t,dqs_c;   //Data Strobe pins
    /* verilator lint_on UNUSEDSIGNAL */

endinterface
