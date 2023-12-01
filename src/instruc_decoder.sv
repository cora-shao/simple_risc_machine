module instruc_decoder(instruc_reg, nsel, opcode, op, cond, ALUop, shift, 
                      sximm5, sximm8, readnum, writenum);

  input [15:0] instruc_reg;
  input [2:0] nsel;
  output [2:0] opcode;
  output [1:0] op;
  output [2:0] cond;
  output [1:0] ALUop;
  output [1:0] shift;
  output [15:0] sximm5;
  output [15:0] sximm8;
  output [2:0] readnum, writenum;

  // Intermediate wires
  wire [4:0] imm5;
  wire [7:0] imm8;
  // Mux wires
  wire [10:8] Rn;
  wire [7:5] Rd;
  wire [2:0] Rm;
  assign Rn = instruc_reg[10:8];
  assign Rd = instruc_reg[7:5];
  assign Rm = instruc_reg[2:0];


  // assign opcode to [15:13] of input
  assign opcode = instruc_reg[15:13];

  // assign op to [12:11] of input
  assign op = instruc_reg[12:11];

  // assign cond to [10:8] of input
  assign cond = instruc_reg[10:8];

  // assign ALUop to [12:11] of input
  assign ALUop =  instruc_reg[12:11];

  // assign imm5 to [4:0] of input
  assign imm5 = instruc_reg[4:0];
  // sign extend the imm5 wire: copy the MSB to the rest of the wires to extend from 5 bit -> 16 bit
  assign sximm5 = {{11{imm5[4]}}, imm5};

  // imm8 -> same as imm5
  assign imm8 = instruc_reg[7:0];
  assign sximm8 = {{8{imm8[7]}}, imm8};

  //0123456789_10_11_12_13_14_15 -> 0123456777777777
  wire [1:0] mux_out;
  mux_shift STR_shift(.a0(2'b00), .a1(instruc_reg[4:3]), .sel(opcode), .out(mux_out));
  assign shift = mux_out;

  // Mux
  logic [2:0] mux_output; // value assigned to read and writenum

  always_comb begin
    case(nsel)
      3'b001: begin
      mux_output = Rn;
      end

      3'b010: begin
      mux_output = Rd;
      end

      3'b100: begin
      mux_output = Rm;
      end

      default: begin
      mux_output = 3'b000; // I'm assuming nsel should always be valid and... not this
      end
    endcase
  end

  assign readnum = mux_output;
  assign writenum = mux_output;

endmodule

module mux_shift (a0,a1,sel,out);
  input [1:0] a0,a1;
  output reg [1:0] out;
  input [2:0] sel;

  always_comb begin
    if (sel == 3'b100) begin
      out = a0;
    end

    else begin
      out = a1;
    end
  end
endmodule