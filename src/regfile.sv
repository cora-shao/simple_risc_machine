module regfile(data_in,writenum,write,readnum,clk,data_out);
  input [15:0] data_in;
  input [2:0] writenum, readnum;  //w_addr, r_adder
  input write, clk;
  output [15:0] data_out;

  reg [15:0] r[7:0];
  wire [15:0] R0, R1, R2, R3, R4, R5, R6, R7;

  assign R0 = r[0];
  assign R1 = r[1];
  assign R2 = r[2];
  assign R3 = r[3];
  assign R4 = r[4];
  assign R5 = r[5];
  assign R6 = r[6];
  assign R7 = r[7];

  always_ff @(posedge clk) begin
    if(write) begin // If write = 1, value on 16-bit register file data_in is written into register 
                    // indicated by value on write num
        r[writenum] <= data_in;
    end
  end

  assign data_out = r[readnum];

endmodule
