module ALU(Ain,Bin,ALUop,out,Z);
  input [15:0] Ain, Bin;
  input [1:0] ALUop;
  output [15:0] out;
  output [2:0] Z;

  reg [15:0] out;
  reg [2:0] Z;


  // assign Z = Z_out[0];
  // assign N = Z_out[1];
  // assign V = Z_out[2];

  always_comb begin
    case(ALUop)
        2'b00: begin 
                out = Ain + Bin;
                Z[2] = (Ain[15] & Bin[15] & ~out[15]) | (~Ain[15] & ~Bin[15] & out[15]);
               end
        2'b01: begin 
                out = Ain - Bin;
                Z[2] = (Ain[15] & ~Bin[15] & ~out[15]) | (~Ain[15] & Bin[15] & out[15]); // | (Ain[15] & ~Bin[15] & ~out[15])
               end
        2'b10: begin 
                out = Ain & Bin;
                Z[2] = 0;
               end
        2'b11: begin 
                out = ~Bin;
                Z[2] = 0;
               end
        default: begin 
                  out = 16'bxxxx_xxxx_xxxx_xxxx;
                  Z[2] = 0;
                 end
    endcase

    //the ALU produces a single-bit status output that is true if and only if ALU_out is equal to zero.
    //Z[0] = out ? 1'b0 : 1'b1;

    //the negative flag is 1 if the most significant bit in out is 1
    //Z[1] =  out[15] ? 1'b1 : 1'b0; 

    //the overflow 
    // case({Ain[15],Bin[15],out[15],ALUop})

    //      5'b01101: Z[2] = 1'b1;
    //      5'b10001: Z[2] = 1'b1;
    //      default: Z[2] = 1'b0;
    // endcase
  end

  always_comb begin
    Z[0] = (out == 16'd0);
    Z[1] = out[15]; 
  end

endmodule