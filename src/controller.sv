`define MNONE 2'b00
`define MREAD 2'b01
`define MWRITE 2'b10

typedef enum logic [4:0] {
  RST,
  IF1,
  IF2,
  UpdatePC,
  HALT,
  WriteImm,
  Get_A,
  Get_B,
  Shift,
  Add,
  Subtract,
  WriteReg,

  // For LDR
  AddRn_Sximm,
  Write_DAddr,
  ReadRAM,
  WriteRegRAM,
  // For STR
  GetRd,
  WriteRAM,

  // New Branch Instructions
  AddPC_Sximm,
  // Function Call Instructions
  BL,
  BX,
  BLX,
  GetB_BX,
  Decode,
  Decode2
} state_t;

module controller(clk, rst, opcode, op, cond,
                  loada, loadb, loadc, loads, load_ir,
                  vsel, nsel, asel, bsel, addr_sel, pc_sel,
                  write, reset_pc, load_pc, load_addr,
                  Z, N, V,
                  mem_cmd, led_on);
 input clk, rst; // Don't need s anymore, since there's no Wait state
 input Z, N, V;
 input [2:0] opcode;
 input [1:0] op;
 input [2:0] cond;
 output loada, loadb, loadc, loads, load_ir, load_pc, load_addr;
 output asel, bsel, addr_sel;
 output [2:0] nsel;
 output [1:0] vsel, pc_sel;
 output logic [1:0] mem_cmd;
 output logic write;
 output logic reset_pc;
 output logic led_on;

state_t state;

 always_ff @(posedge clk) begin
    if(rst) begin
        state = RST;
    end
    else begin
        casex({state, opcode, op, cond}) // States go back to IF1 after each instruction is completed
          {RST, 8'bxxxxx_xxx}: begin
                              state = IF1;
                            end
          {IF1, 8'bxxxxx_xxx}: begin
                              state = IF2;
                            end
          {IF2, 8'bxxxxx_xxx}: // IF2 determines next state, which is state that sets PC value
                            begin
                              state = Decode2;
                            end
          // TBD: I wonder if there's a better way to do this than IF2 -> Decode(branch) -> UpdatePC -> Decode...
          {Decode2, 8'b00100_000}:
                                begin // B
                                  state = AddPC_Sximm; // PC = PC + 1 + sx(im8)
                                end
          {Decode2, 8'b00100_011}: // BLT
                            begin
                              if(~N == V) begin // MAYBE CHANGE TO !==??
                                state = AddPC_Sximm;
                              end
                              else begin
                                state = UpdatePC;
                              end
                            end
          {Decode2, 8'b00100_001}: // BEQ
                                begin
                                  if(Z) begin
                                    state = AddPC_Sximm;
                                  end
                                  else begin
                                    state = UpdatePC;
                                  end
                                end
          {Decode2, 8'b00100_010}: // BNE
                                begin
                                  if(Z == 1'b0) begin
                                    state = AddPC_Sximm;
                                  end
                                  else begin
                                    state = UpdatePC;
                                  end
                                end
          {Decode2, 8'b00100_100}:  // BLE
                                begin
                                   if((~N == V) || Z == 1'b1) begin
                                     state = AddPC_Sximm;
                                   end
                                   else begin
                                     state = UpdatePC;
                                   end
                                end
          {Decode2, 8'b0101x_xxx}:
                                begin
                                  state = BL; // For BL and BLX
                                end
          {Decode2, 8'b01000_xxx}:
                                begin
                                  state = BX; // For BX
                                end
          {Decode2, 8'bxxxxx_xxx}: // For all other instructions
                                begin
                                  state = UpdatePC;
                                end
          {AddPC_Sximm, 8'bxxxxx_xxx}: begin
                             state = IF1;
                             end
          {UpdatePC, 8'b001xx_xxx}: begin
                              state = IF1; // Branch instructions
                            end
          {UpdatePC, 8'bxxxxx_xxx}: // TBD: Can decode instr. in UpdatePC state
                            begin
                              state = Decode;
                            end
          {Decode, 8'b111xx_xxx}: begin
                                state = HALT;
                               end
          {Decode, 8'b10101_xxx}: begin
                                state = Get_A;
                               end  
          {Decode, 8'b101x0_xxx}: begin
                                state = Get_A; // For ADD & AND
                               end
          {Decode, 8'b11000_xxx}: begin
                                state = Get_B;
                               end 
          {Decode, 8'b10111_xxx}: begin
                                state = Get_B;
                               end 
          {Decode, 8'b11010_xxx}: begin
                                state = WriteImm;
                               end
          {Decode, 8'b011xx_xxx}: begin
                                state = Get_A; // For LDR
                               end
          {Decode, 8'b100xx_xxx}: begin
                                state = Get_A; // For STR
                               end
          // {Decode, 8'b01011_xxx}: begin
          //                       state = BL; // For BL
          //                      end
          // {Decode, 8'b01000_xxx}: begin
          //                       state = Get_B; // For BX
          //                      end
          // {Decode, 8'b01010_xxx}: begin
          //                       state = BLX; // For BLX
          //                      end
          {HALT, 8'bxxxxx_xxx}: begin
                                state = HALT;
                             end
          {WriteImm, 8'bxxxxx_xxx}: begin
                                  state = IF1;
                                 end                      
          {Get_A, 8'b101xx_xxx}: begin
                               state = Get_B;
                              end 
          {Get_A, 8'b011xx_xxx}: begin
                               state = AddRn_Sximm; // For LDR
                              end 
          {Get_A, 8'b100xx_xxx}: begin
                               state = AddRn_Sximm; // For STR
                              end
          {Get_B, 8'b10101_xxx}: begin
                               state = Subtract; // For CMP
                              end
          {Get_B, 8'b101x0_xxx}: begin
                               state = Add; // For ADD & AND
                              end
          {Get_B, 8'b11000_xxx}: begin
                               state = Shift;  // For MOV instruction 2
                               end 
          {Get_B, 8'b10111_xxx}: begin
                               state = Shift;  // For MVN
                              end
          {Shift, 8'b100xx_xxx}: begin   //Hopefully, this is evaluated sequentially
                                state = WriteRAM;  // For STR
                              end
          {Shift, 8'bxxxxx_xxx}:
                                begin
                                  state = WriteReg;  // For MOV instruction 2 & MVN
                                end
          {Subtract, 8'b10101_xxx}:
                                begin
                                  state = IF1;  // For CMP
                                end
          {Add, 8'bxxxxx_xxx}:
                                begin
                                  state = WriteReg;  // For ADD & AND
                                end
          {WriteReg, 8'bxxxxx_xxx}:
                                  begin
                                    state = IF1;
                                  end
          {AddRn_Sximm, 8'bxxxxx_xxx}:
                                  begin
                                    state = Write_DAddr; // For both STR and LDR
                                  end
          {Write_DAddr, 8'b011xx_xxx}:
                                    begin
                                      state = ReadRAM;
                                    end
          {Write_DAddr, 8'b100xx_xxx}:
                                    begin
                                    state = GetRd; // For STR
                                    end
          {ReadRAM, 8'bxxxxx_xxx}:
                                    begin
                                      state = WriteRegRAM;
                                    end
          {WriteRegRAM, 8'bxxxxx_xxx}:
                                    begin
                                      state = IF1;
                                    end
          {GetRd, 8'bxxxxx_xxx}:
                            begin
                              state = Shift; // For STR
                            end
          {WriteRAM, 8'bxxxxx_xxx}:
                            begin
                              state = IF1; // For STR
                            end
          {BL, 8'b01010_xxx}:
                            begin
                              state = BX; // For BLX
                            end
          {BL, 8'bxxx11_xxx}:
                              begin
                                state = AddPC_Sximm; // For BL
                              end
          {BX, 8'bxxxxx_xxx}:
                              begin
                                state = IF1; // For BX, loads into PC
                              end
          {BLX, 8'bxxxxx_xxx}:
                              begin
                                state = Decode2; // For BLX
                              end
          default: state = state;
        endcase //end casex
    end // end else
 end // end always_ff


 reg [6:0] load; // loada, loadb, loadc, loads, load_ir, load_pc, load_addr,;
 reg [9:0] selects; // asel_bsel_vsel_nsel_pcsel

 assign loada = load[6];
 assign loadb = load[5];
 assign loadc = load[4];
 assign loads = load[3];
 assign load_ir = load[2];
 assign load_pc = load[1];
 assign load_addr = load[0];

 assign asel = selects[9];
 assign bsel = selects[8];
 assign addr_sel = selects[7];
 assign vsel = selects[6:5];
 assign nsel = selects[4:2];
 assign pc_sel = selects[1:0];

 always_comb begin
    case(state)
        RST: begin
                write = 0;
                led_on = 0;
                mem_cmd = `MNONE;
                reset_pc = 1'b1; // reset = 1
                load = 7'b0000_010; // load_pc = 1
                selects = 10'b0_0_0_00_000_00;
              end // Cycle 1
        IF1: begin
                write = 0;
                led_on = 0;
                mem_cmd = `MREAD; // mem_cmd = MREAD
                reset_pc = 1'b0;
                load = 7'b0000_000;
                selects = 10'b0_0_1_00_000_00; // addr_sel = 1
              end // Cycle 2     
        IF2: begin
                write = 0;
                led_on = 0;
                mem_cmd = `MREAD;
                reset_pc = 1'b0;
                load = 7'b0000_100; // load_ir = 1
                selects = 10'b0_0_1_00_000_00; // addr_sel = 1
                end
        Decode2: begin // Decode2 is for decoding branch instructions
                  write = 0;
                  led_on = 0;
                  mem_cmd = `MNONE;
                  reset_pc = 1'b0;
                  load = 7'b0000_000;
                  selects = 10'b0_0_0_00_000_00;
                 end
        Decode: begin
                  write = 0;
                  led_on = 0;
                  mem_cmd = `MNONE;
                  reset_pc = 1'b0;
                  load = 7'b0000_000;
                  selects = 10'b0_0_0_00_000_00;
                 end        
        UpdatePC: begin
                write = 0;
                led_on = 0;
                mem_cmd = `MNONE;
                reset_pc = 1'b0;
                load = 7'b0000_010; // load_pc = 1
                selects = 10'b0_0_0_00_000_00; 
                end
        AddPC_Sximm: begin
                  write = 0;
                  led_on = 0;
                  mem_cmd = `MNONE;
                  reset_pc = 1'b0;
                  load = 7'b0000_010; // load_pc = 1
                  selects = 10'b0_0_0_00_000_01; // pc_sel = 01
                 end
        HALT: begin
                 write = 0;
                 led_on = 1;
                 mem_cmd = `MNONE;
                 reset_pc = 1'b0;
                 load = 7'b0000_000;
                 selects = 10'b0_0_0_00_000_00;
                end         
        WriteImm: begin
                    write = 1;
                    led_on = 0;
                    mem_cmd = `MNONE;
                    reset_pc = 1'b0;
                    load = 7'b0000_000;
                    selects = 10'b0_0_0_01_001_00; // vsel = 01, nsel = 001
                   end
        Get_A: begin
                write = 0;
                led_on = 0;
                mem_cmd = `MNONE;
                reset_pc = 1'b0;
                load = 7'b1000_000; // loada = 1
                selects = 10'b0_0_0_00_001_00; // nsel = 001
                end
        Get_B: begin
                write = 0;
                led_on = 0;
                mem_cmd = `MNONE;
                reset_pc = 1'b0;
                load = 7'b0100_000; // loadb = 1
                selects = 10'b0_0_0_00_100_00; // nsel = 100
                end
        GetB_BX: begin
                write = 0;
                led_on = 0;
                mem_cmd = `MNONE;
                reset_pc = 1'b0;
                load = 7'b0100_000; // loadb = 1
                selects = 10'b0_0_0_00_010_00; // nsel = 010
                end        
        Shift: begin
                write = 0;
                led_on = 0;
                mem_cmd = `MNONE; // Note, changed MNONE -> MWRITE to account for delay
                reset_pc = 1'b0;
                load = 7'b0010_000; // loadc = 1
                selects = 10'b1_0_0_00_000_00; // asel = 1
                end
        Add: begin
                write = 0;
                led_on = 0;
                mem_cmd = `MNONE;
                reset_pc = 1'b0;
                load = 7'b0011_000; // loadc = loads = 1 // check for overflow?
                selects = 10'b0_0_0_00_000_00; 
              end
        Subtract: begin
                    write = 0;
                    led_on = 0;
                    mem_cmd = `MNONE;
                    reset_pc = 1'b0;
                    load = 7'b0011_000; // loads = loadc = 1 // Not sure if I actually need loadc...
                    selects = 10'b0_0_0_00_000_00; 
                   end
        WriteReg: begin
                    write = 1;
                    led_on = 1'b0;
                    mem_cmd = `MNONE;
                    reset_pc = 1'b0;
                    load = 7'b0000_000; 
                    selects = 10'b0_0_0_10_010_00; // vsel = 10, nsel = 010
                   end
        AddRn_Sximm: begin
                        write = 0;
                        led_on = 1'b0;
                        mem_cmd = `MNONE;
                        reset_pc = 1'b0;
                        load = 7'b0010_000; // loadc = 1
                        selects = 10'b0_1_0_00_000_00; // asel = 0, bsel = 1;
                      end      
        Write_DAddr: begin
                        write = 0;
                        led_on = 1'b0;
                        mem_cmd = `MNONE;
                        reset_pc = 1'b0;
                        load = 7'b0000_001; // load_addr = 1
                        selects = 10'b0_0_0_00_000_00; // addr_sel = 0
                       end
        ReadRAM: begin
                    write = 0;
                    led_on = 1'b0;
                    mem_cmd = `MREAD; // Reading from RAM
                    reset_pc = 1'b0;
                    load = 7'b0000_000; 
                    selects = 10'b0_0_0_00_000_00; // addr_sel = 0
                  end
        WriteRegRAM: begin
                        write = 1;
                        led_on = 0;
                        mem_cmd = `MREAD;
                        reset_pc = 1'b0;
                        load = 7'b0000_000;
                        selects = 10'b0_0_0_00_010_00; // nsel = 010, vsel = 00;
                      end
        GetRd: begin
                 write = 0;
                 led_on = 0;
                 mem_cmd = `MNONE;
                 reset_pc = 1'b0;
                 load = 7'b0100_000; // loadb = 1
                 selects = 10'b0_0_0_00_010_00; // nsel = 010;
                end 
        WriteRAM: begin
                 write = 0;
                 led_on = 0;
                 mem_cmd = `MWRITE;
                 reset_pc = 1'b0;
                 load = 7'b0000_000; 
                 selects = 10'b0_0_0_00_000_00; // addr_sel = 0;
                end
        BL: begin // R[7] = PC + 1
                 write = 1; // write = 1
                 led_on = 1'b0;
                 mem_cmd = `MNONE;
                 reset_pc = 1'b0;
                 load = 7'b0000_000; // load_pc = 0
                 selects = 10'b0_0_0_11_001_00; // nsel = 3'b001; vsel = 2'b11;
             end
        BX: begin // Loads PC with R[Rd]
                write = 0;
                led_on = 1'b0;
                mem_cmd = `MNONE;
                reset_pc = 1'b0;
                load = 7'b0000_010; // load_pc = 1
                selects = 10'b0_0_0_00_010_10; // pc_sel = 2'b10; nsel = 3'b010 for Rd;
              end
        BLX: begin
                 write = 1; // write = 1
                 led_on = 1'b0;
                 mem_cmd = `MNONE;
                 reset_pc = 1'b0;
                 load = 7'b0000_000; // load_pc = 0
                 selects = 10'b0_0_0_11_001_00; // nsel_sel = 3'b001; vsel = 2'b11;
              end
        default: begin                 
                 write = 0;
                 led_on = 1'b0;
                 reset_pc = 1'b0;
                 mem_cmd = `MNONE;
                 load = 7'b0000_000; 
                 selects = 10'b0_0_0_00_000_00;
                end
    endcase
 end

endmodule