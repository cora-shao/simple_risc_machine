`define MNONE 2'b00
`define MREAD 2'b01
`define MWRITE 2'b10

module lab7bonus_top(KEY,SW,LEDR,HEX0,HEX1,HEX2,HEX3,HEX4,HEX5,CLOCK_50);
  input [3:0] KEY;
  input [9:0] SW;
  output [9:0] LEDR;
  output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;
  input CLOCK_50;

  //cpu and mem connections
  wire clk, reset, load;
  wire N,V,Z;
  wire [15:0] read_data;
  wire [15:0] write_data;
  wire [8:0] mem_addr;
  wire [1:0] mem_cmd;
  wire [15:0] dout;
  //wire led_on; // external LEDR[8] signal for HALT state

  //cpu instance
  // clk isn't negated here - is that correct
  cpu CPU (.clk(~CLOCK_50),.reset(~KEY[1]),.load(load),.N(N),.V(V),.Z(Z),
           .mem_cmd(mem_cmd), .mem_addr(mem_addr), 
           .read_data(read_data), .write_data(write_data),
           .led_on(LEDR[8]));

  //equality check module for tri_state sel
  wire msel;
  equality1 E1 (.a1(1'b0), .a2(mem_addr[8:8]), .equal(msel));

  //equality check module for MREAD
  wire eread;
  equality2 E2 (.a1(`MREAD), .a2(mem_cmd), .equal(eread));

  //equality check module for MREAD
  wire ewrite;
  equality2 E3 (.a1(`MWRITE), .a2(mem_cmd), .equal(ewrite));

  //tri-state driver instance
  tri_state TRI_DOUT (.in(dout), .enable(msel & eread), .out(read_data));

  //ram instance
  RAM MEM(.clk(~CLOCK_50),.read_address(mem_addr[7:0]),.write_address(mem_addr[7:0]),.write(msel & ewrite),.din(write_data),.dout(dout));

  //switch combinational circuit connections and instance
  wire sw_enable;

  sw_comb SW_ENABLE(.comp(9'h140),.mem_addr(mem_addr),.mem_cmd(mem_cmd),.enable(sw_enable));

  //tri-state drivers for switches

  tri_state_8 TSW1 (.in(8'h00), .out(read_data[15:8]), .enable(sw_enable));
  tri_state_8 TSW2 (.in(SW[7:0]), .out(read_data[7:0]), .enable(sw_enable));

  //led combinational circuit connections and instance
  wire led_load;

  led_comb LED_COMB (.comp(9'h100), .mem_addr(mem_addr), .mem_cmd(mem_cmd), .load(led_load));

  //led load enable reg
  vDFF_loadEnable2 LED_LOAD (.clk(~CLOCK_50), .load(led_load), .in(write_data[7:0]), .out(LEDR[7:0]));

endmodule



// To ensure Quartus uses the embedded MLAB memory blocks inside the Cyclone
// V on your DE1-SoC we follow the coding style from in Altera's Quartus II
// Handbook (QII5V1 2015.05.04) in Chapter 12, “Recommended HDL Coding Style”
//
// 1. "Example 12-11: Verilog Single Clock Simple Dual-Port Synchronous RAM 
//     with Old Data Read-During-Write Behavior" 
// 2. "Example 12-29: Verilog HDL RAM Initialized with the readmemb Command"

module RAM(clk,read_address,write_address,write,din,dout);
  parameter data_width = 16;
  parameter addr_width = 8;
  parameter filename = "data_fig4.txt";

  input clk;
  input [addr_width-1:0] read_address, write_address;
  input write;
  input [data_width-1:0] din;
  output [data_width-1:0] dout;
  reg [data_width-1:0] dout;

  reg [data_width-1:0] mem [2**addr_width-1:0];

  initial $readmemb(filename, mem);

  always @(posedge clk) begin
    if (write)
      mem[write_address] <= din;
    dout <= mem[read_address]; // dout doesn't get din in this clock cycle 
                               // (this is due to Verilog non-blocking assignment "<=")
  end 
endmodule

//tri-state driver
module tri_state (in, enable, out);
  input [15:0] in;
  input enable;
  output [15:0] out;

  assign out = enable ? in : {16{1'bz}};
endmodule

//checks for equatlity
module equality1 (a1, a2, equal);
  input a1, a2;
  output reg equal;

  always_comb begin
    if (a1 == a2)begin
      equal = 1;
    end
    else begin
      equal = 0;
    end
  end
endmodule 

module equality2 (a1, a2, equal);
  input [1: 0] a1, a2;
  output reg equal;

  always_comb begin
    if (a1 == a2)begin
      equal = 1;
    end
    else begin
      equal = 0;
    end
  end
endmodule 

module led_comb (comp, mem_addr, mem_cmd, load);
  input [8:0] comp, mem_addr;
  input [1:0] mem_cmd;
  output reg load;

  always_comb begin
    case({mem_addr,mem_cmd})
    {comp,`MWRITE}: load = 1'b1;
    default: load = 1'b0;
    endcase
  end
endmodule

module sw_comb (comp, mem_addr, mem_cmd, enable);
  input [8:0] comp, mem_addr;
  input [1:0] mem_cmd;
  output reg enable;

  always_comb begin
    case({mem_addr,mem_cmd})
    {comp,`MREAD}: enable = 1'b1;
    default: enable = 1'b0;
    endcase
  end
endmodule

module tri_state_8 (in, enable, out);
  input [7:0] in;
  input enable;
  output [7:0] out;

  assign out = enable ? in : {8{1'bz}};
endmodule

module vDFF_loadEnable2(clk, load, in, out);
  input clk;
  input load;
  input [8:0] in;
  output [8:0] out; //Why is there a bit number discrepancy?
  reg [8:0] out;

  always_ff @(posedge clk) begin
    if(load) begin
        out <= in; //should we make it non-blocking as we might use multiple registers in the topmodule 
    end
    
  end
endmodule