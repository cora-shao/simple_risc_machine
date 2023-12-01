module datapath(input clk,
  input [1:0] vsel,
  input loada,
  input loadb,
  input loadc,
  input loads,
  input [2:0] writenum,
  input write,
  input [2:0] readnum,
  input [1:0] shift,
  input asel,
  input bsel,
  input [1:0] ALUop,
  input [7:0] PC,
  input [15:0] sximm8,
  input [15:0] sximm5,
  input [15:0] mdata,
  output wire [15:0] regfile_data_out,
  output wire [15:0] datapath_out,
  output wire [2:0] Z_out
  );

  // Regfile connections
  wire [15:0] data_in;
  wire [15:0] data_out;
  assign regfile_data_out = data_out;

  // Shifter connections
  wire [15:0] in;
  wire [15:0] sout;

  // ALU connections
  wire [15:0] Ain, Bin, ALU_out; //does it need to be signed...?
  wire [2:0] Z;

  // Other connections
  wire [15:0] A_ff_out; // "out" on the diagram

  // Register file instantiation (#1)
  regfile REGFILE(.data_in(data_in),
                  .writenum(writenum),
                  .write(write),
                  .readnum(readnum),
                  .clk(clk),
                  .data_out(data_out));

  // ALU instantiation (#2)
  ALU ALU(.Ain(Ain),
          .Bin(Bin),
          .ALUop(ALUop),
          .out(ALU_out),
          .Z(Z));

  // Shifter instantiation (#8)
  shifter shifter(.in(in), .shift(shift), .sout(sout));

  // Muxes #6, #7, #9
  Mux_2to1 #(.WIDTH(16)) mux6(.a0(A_ff_out), .a1(16'b0000_0000_0000_0000), .sel(asel), .out(Ain));

  Mux_2to1 #(.WIDTH(16)) mux7(.a0(sout), .a1(sximm5), .sel(bsel), .out(Bin));

  Mux_4to1 #(.WIDTH(16)) mux9(.a0(mdata), .a1(sximm8), .a2({8'b0, PC}), .a3(datapath_out), .sel(vsel), .b(data_in));


  // Flip-flops
  vDFF_loadEnable A_ff(.clk(clk), .load(loada), .in(data_out), .out(A_ff_out));

  vDFF_loadEnable B_ff(.clk(clk), .load(loadb), .in(data_out), .out(in));

  vDFF_loadEnable C_ff(.clk(clk), .load(loadc), .in(ALU_out), .out(datapath_out));

  vDFF_loadEnable #(.WIDTH(3)) Z_ff(.clk(clk), .load(loads), .in(Z), .out(Z_out));

endmodule


module Mux_4to1 #(parameter WIDTH = 16) (a0, a1, a2, a3, sel, b);
  input [WIDTH-1:0] a0, a1, a2, a3;
  input [1:0] sel;
  output reg [WIDTH-1:0] b;

  always_comb begin
    case (sel)
      2'b00: b = a0;
      2'b01: b = a1;
      2'b11: b = a2;
      2'b10: b = a3;
      default: b = {16{1'bx}};
    endcase

  end
endmodule