module shifter(in,shift,sout);
  input [15:0] in;
  input [1:0] shift;
  output [15:0] sout;

  reg [15:0] sout;
  reg temp;

  assign temp = in[15];

  always @(*) begin
    case(shift)
     2'b00: sout = in;
     2'b01: sout = in << 1;
     2'b10: sout = in >> 1;
     2'b11: begin 
            sout = in >>> 1;
            sout[15] = temp;
            end
     default: sout = 16'b0000_0000_0000_0000;
    endcase
  end

endmodule