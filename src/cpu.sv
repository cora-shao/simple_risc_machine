module cpu(clk,reset,load,N,V,Z,mem_cmd,mem_addr,read_data,write_data,led_on);
  input clk, reset, load; //load is no longer driven 
  input [15:0] read_data;
  output [15:0] write_data;
  output [8:0] mem_addr;
  output [1:0] mem_cmd;
  output N, V, Z;
  output led_on;

  // Instruction Register connections
  wire [15:0] inst_dec;
  wire load_ir;

  // Instruction Register Instantiation
  vDFF_loadEnable #(.WIDTH(16)) instruc_reg(.clk(clk), .load(load_ir), .in(read_data), .out(inst_dec));

  // Decoder/Controller/datapath connections
  wire [2:0] opcode;
  wire [2:0] nsel;
  wire [2:0] cond;
  wire [1:0] op;
  wire [1:0] ALUop;
  wire [1:0] shift;
  wire [15:0] sximm5;
  wire [15:0] sximm8;
  wire [2:0] readnum;
  wire [2:0] writenum;

  // Decoder
  instruc_decoder DEC(.instruc_reg(inst_dec), .nsel(nsel), .opcode(opcode), .op(op), .cond(cond), .ALUop(ALUop), .shift(shift), 
                      .sximm5(sximm5), .sximm8(sximm8),
                      .readnum(readnum), .writenum(writenum));

  // Controller connections
  wire load_addr;
  wire load_pc, reset_pc;
  wire asel, bsel, addr_sel;
  wire loada, loadb, loadc, loads, write;
  wire Z, N, V;
  wire [1:0] vsel, pc_sel;
  wire led_on;

  // FSM Controller instantiation
  controller CON(.clk(clk),
                 .rst(reset),
                 .opcode(opcode), .op(op), .cond(cond),
                 .loada(loada), .loadb(loadb), .loadc(loadc), .loads(loads),
                 .vsel(vsel), .nsel(nsel), .asel(asel), .bsel(bsel), .pc_sel(pc_sel),
                 .write(write), .load_ir(load_ir), .load_pc(load_pc), .reset_pc(reset_pc),
                 .Z(Z), .N(N), .V(V),
                 .mem_cmd(mem_cmd), .load_addr(load_addr), .addr_sel(addr_sel),
                 .led_on(led_on));

  // Datapath connections
  wire [2:0] Z_out;
  wire [15:0] regfile_out;

  // Program counter load enabled reg connections
  wire [8:0] next_pc;
  wire [8:0] new_pc;
  logic [8:0] PC;

  //Datapath instantiation
  datapath DP( .clk         (clk),

                // D: register operand fetch stage
                .readnum     (readnum),
                .vsel        (vsel),
                .loada       (loada),
                .loadb       (loadb),

                // EX: computation stage (sometimes called "execute")
                .shift       (shift),
                .asel        (asel),
                .bsel        (bsel),
                .ALUop       (ALUop),
                .loadc       (loadc),
                .loads       (loads),

                // WB: set when "writing back" to register file
                .writenum    (writenum),
                .write       (write),

                // new lab 6 inputs (datapath inputs)
                .mdata(read_data),
                .PC(next_pc[7:0]),
                .sximm8(sximm8),
                .sximm5(sximm5),

                // M: outputs
                .Z_out       (Z_out),
                .regfile_data_out    (regfile_out),
                .datapath_out(write_data)
            );

  //PC reg instantiation
  vDFF_loadEnable #(.WIDTH(9)) PC_reg(.clk(clk), .load(load_pc), .in(new_pc), .out(PC));

  // Adder connections
  wire [8:0] adder_out;
  wire [8:0] adder_sximm8_out;

  // Adder instantiation
  adder PC_add(.in(PC), .add_val(9'b000_000_001), .out(adder_out)); // we can make an adder without a module by simply adding one but I just wanted to stay safe

  // Mux for pc (between next_pc and reset PC)
  Mux_2to1 #(.WIDTH(9)) PC_mux(.a0(adder_out), .a1(9'b000_000_000), .sel(reset_pc), .out(next_pc));

  // Adder for sximm8 instantiation
  adder next_PC_add(.in(next_pc), .add_val(sximm8[8:0]), .out(adder_sximm8_out)); // note that sximm8 is truncated

  // Mux to choose between next_pc, next_pc + sximm8, and R[Rd]
  Mux_3to1 PC_choose(.a0(next_pc), .a1(adder_sximm8_out), .a2(regfile_out[8:0]), .sel(pc_sel), .out(new_pc)); // PLACEHOLDER for Rd

  // Data address connections
  wire [8:0] data_addr_out;

  // Data address reg
  vDFF_loadEnable #(.WIDTH(9)) DataAddress_reg(.clk(clk), .load(load_addr), .in(write_data[8:0]), .out(data_addr_out));

  // Data-PC mux instance
  Mux_2to1 data_mux (.a0(data_addr_out),.a1(PC),.sel(addr_sel),.out(mem_addr));

  assign Z = Z_out[0];
  assign N = Z_out[1];
  assign V = Z_out[2];

endmodule

// Load enabled register
module vDFF_loadEnable #(parameter WIDTH = 16) (clk, load, in, out);
  input clk;
  input load;
  input [WIDTH-1:0] in;
  output reg [WIDTH-1:0] out;

  always_ff @(posedge clk) begin
    if(load) begin
      out <= in;
    end
  end
endmodule

module Mux_2to1 #(parameter WIDTH = 9) (a0, a1, sel, out);
  input [WIDTH-1:0] a0, a1;
  input sel;
  output reg [WIDTH-1:0] out;

  always_comb begin
    out = sel ? a1 : a0;
  end
endmodule

module Mux_3to1 (a0, a1, a2, sel, out);
  input [8:0] a0, a1, a2;
  input [1:0] sel;
  output reg [8:0] out;

  always_comb begin
    case(sel)
      2'b00: out = a0;
      2'b01: out = a1;
      2'b10: out = a2;
      default: out = a0;
    endcase
  end
endmodule

// Adder
module adder(in, add_val, out);
  input [8:0] in;
  input [8:0] add_val;
  output reg [8:0] out;

  always_comb begin
    out = in + add_val;
  end
endmodule
