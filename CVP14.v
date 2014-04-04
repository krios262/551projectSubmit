
module CVP14(output [15:0] Addr, output reg RD, output reg WR, output reg V,
    output reg [15:0] DataOut, input Reset, input Clk1, input Clk2, input [15:0] DataIn);

  //Parameters for opcodes
  parameter vadd = 4'b0000, vdot = 4'b0001, smul = 4'b0010, sst = 4'b0011, vld = 4'b0100,
            vst = 4'b0101,  sll = 4'b0110,  slh = 4'b0111,  j = 4'b1000,   nop = 4'b1111;

  //Parameters for states
  parameter newPC = 3'b000, fetchInst = 3'b001, startEx = 3'b010, executing = 3'b011,
            done = 3'b100, overflow = 3'b101, start = 3'b111;

  reg  [255:0] vInP;
  wire [255:0] vOutP, vOutP2;
  reg  [15:0]  sIn, vInS;
  wire [15:0]  vOutS, vOutS2, sOut;
  reg  [2:0]   sAddr, vAddr, vAddr2;
  reg sRD, sWR, sWR_l, sWR_h, vWR_p, vWR_s, vRD_p, vRD_s;
  reg updatePC, jump, setPC, updateAddr, offsetInc; //addressing module flags

  wire [15:0] PC; //program counter
  reg [15:0] instruction;
  wire [3:0] inc_offset;
  reg V_flag;

  reg [2:0] state, nextState;

  wire [255:0] AdderOut;
  wire OvF;
  reg startadd;
  wire DONE;
  reg Prevdone;
  reg [255:0] AdderIn1,AdderIn2;
  reg [15:0] vBuf;

  //Scalar and Vector registers
  sReg scalar(.DataOut(sOut), .Addr(sAddr), .Clk1(Clk1), .Clk2(Clk2), .DataIn(sIn),
              .RD(sRD), .WR(sWR), .WR_l(sWR_l), .WR_h(sWR_h));
  vReg vector(.DataOut_p(vOutP), .DataOut_s(vOutS), .Addr(vAddr), .Clk1(Clk1), .Clk2(Clk2),
              .DataIn_p(vInP), .DataIn_s(vInS), .WR_p(vWR_p), .RD_p(vRD_p), .WR_s(vWR_s),
              .RD_s(vRD_s), .DataOut2_p(vOutP2), .DataOut2_s(vOutS2), .Addr2(vAddr2));

  //Addressing
  PCunit pcu(.PC(PC), .offset(instruction[11:0]), .Clk2(Clk2), .updatePC(updatePC),
              .jump(jump), .reset(Reset), .overflow(V_flag));
  addrUnit addru(.addr(Addr), .PC(PC), .Clk1(Clk1), .imm_offset(instruction[5:0]),
              .addrBase(sOut), .setPC(setPC), .updateAddr(updateAddr), .inc_offset(inc_offset));
  offsetu osu(.Reset(Reset), .Clk2(Clk2), .offsetInc(offsetInc), .offset(inc_offset));
  
  //Operation modules
  VADD16 adder(.SumV(AdderOut),.Overflw(OvF),.Inval1(AdderIn1),.Inval2(AdderIn2),.start(startadd),.done(DONE));

  always@(posedge Clk1) begin
    //Addresses and state are set on Clk1

    if (Reset) begin
      state <= start;
      updatePC <= 1'b0;
      jump <= 1'b0;
      offsetInc <= 1'b0;
      V <= 1'b0;
    end else begin
      state <= nextState;

    case (nextState)

      start: begin
        //No action on Clk1
      end

      newPC: begin
        updatePC <= 1'b0;
        jump <= 1'b0;
        offsetInc <= 1'b0;
      end

      fetchInst: begin
        //No action on Clk1
      end

      startEx: begin

        case (instruction[15:12])
          vadd:
          begin
            vAddr <= instruction[8:6];
            vAddr2 <= instruction[5:3];
          end
          /*vdot:
          smul:
          */
          sst: begin
            sAddr <= instruction[8:6]; //get system mem dest address
          end
          vld: begin
            sAddr <= instruction[8:6]; //get system mem dest address
            vAddr <= instruction[11:9]; //vector store dest
          end
          vst: begin
            sAddr <= instruction[8:6]; //get system mem dest address
            vAddr <= instruction[11:9]; //vector store dest
          end
          sll:
          begin
            sAddr <= instruction[11:9];
          end
          slh:
          begin
            sAddr <= instruction[11:9];
          end
          j: begin
            updatePC <= 1'b1;
            jump <= 1'b1;
          end
          nop: begin
            updatePC <= 1'b1;
          end
        endcase

      end

      executing: begin
        case (instruction[15:12])
          vadd:
             vAddr <= instruction[11:9];
          sst:
            sAddr <= instruction[11:9]; //scalar value to be stored
          vld: begin
            if (updateAddr) begin
              offsetInc <= 1'b1;
            end else
              offsetInc <= 1'b0;
          end
          vst: begin
            if (updateAddr) begin
              offsetInc <= 1'b1;
            end else
              offsetInc <= 1'b0;
          end

        endcase
      end

      done: begin
        updatePC <= 1'b1;
        if (instruction[15:12] == vadd)
          V <= V_flag;
        else
          V <= V;
      end

      overflow: begin
        sAddr <= 3'b111;
      end

    endcase
    end //else

  end //always

  always@(posedge Clk2) begin
    //RD/WR flags, instruction, PC, data inputs are set on Clk2
    //Data outputs are read on Clk2

    vInS <= DataIn;

    case (state)

      start: begin
        RD <= 1'b0;
        WR <= 1'b0;
        sWR_l <= 1'b0;
        sWR <= 1'b0;
        sWR_h <= 1'b0;
        sRD <= 1'b0;
        vRD_s <= 1'b0;
        vRD_p <= 1'b0;
        vWR_s <= 1'b0;
        vWR_p <= 1'b0;
        setPC <= 1'b1;
        updateAddr <= 1'b0;
        Prevdone <= 1'b0;
        startadd <= 1'b0;
        V_flag <= 1'b0;
      end

      newPC: begin
        RD <= 1'b1;
        WR <= 1'b0;
        sWR <= 1'b0;
        V_flag <= 1'b0;
        setPC <= 1'b0;
      end

      fetchInst: begin
        RD <= 1'b0;
        vWR_s <= 1'b0;
        instruction <= DataIn;
      end

      startEx: begin

        case (instruction[15:12])
        vadd:
          begin
            vRD_p <= 1'b1;
          end
          /* vdot:sim:/t_vaddtestsynth16
          smul: */
          sst: begin
            sRD <= 1'b1;
          end
          vld: begin
            sRD <= 1'b1;
          end
          vst: begin
            sRD <= 1'b1;
          end
          sll: begin
            sIn   <= {sIn[15:8],instruction[7:0]};
            sWR_l <= 1'b1;
          end
          slh: begin
            sIn   <= {instruction[7:0],sIn[7:0]};
            sWR_h <= 1'b1;
          end
          j: begin
            setPC <= 1'b1;
          end
          nop: begin
            setPC <= 1'b1;
          end
        endcase

      end

      executing: begin
        case (instruction[15:12])
          vadd: begin
            vRD_p <= 1'b0;
            AdderIn1 <= vOutP;
            AdderIn2 <= vOutP2;
            Prevdone <= DONE;
            if(Prevdone == 1'b1)
              begin
                vInP <= AdderOut;
                V_flag <= OvF;
                vWR_p <= 1'b1;
                startadd <= 1'b0;
              end
             else
               startadd <= 1'b1;
           end
          sst: begin
            //updateAddr on first cycle, do other operations on second
            if (updateAddr) begin
              DataOut <= sOut;
              WR <= 1'b1;
              sRD <= 1'b0;
              updateAddr <= 1'b0;
            end else
              updateAddr <= 1'b1;
          end
          vld: begin
            sRD <= 1'b0;

            if (updateAddr) begin
              RD <= 1'b1;
            end else
              updateAddr <= 1'b1;
            if (RD) begin
              vWR_s <= 1'b1;
            end else
              vWR_s <= 1'b0;
          end
          vst: begin
            sRD <= 1'b0;
            vRD_s <= 1'b1;

            if (updateAddr) begin
              WR <= 1'b1;
              DataOut <= vOutS;
            end else
              updateAddr <= 1'b1;
          end

        endcase
      end

      done: begin
        //do not set RD to 0; it must stay 1 for VLD
        sWR_l <= 1'b0;
        sWR_h <= 1'b0;
        setPC <= 1'b1;
        vRD_s <= 1'b0;
        updateAddr <= 1'b0;
        vWR_p <= 1'b0;
        Prevdone <= 1'b0;
        //startadd <= 1'b0;
        case (instruction[15:12]) //for vld, RD persists
          vld: begin
            WR <= 1'b0;
            RD <= RD;
          end
          vst: begin
            WR <= WR;
            RD <= 1'b0;
            DataOut <= vOutS;
          end
          default: begin
            WR <= 1'b0;
            RD <= 1'b0;
          end
        endcase
      end

      overflow: begin
        sWR <= 1'b1;
        sIn <= instruction;
      end
    endcase

  end

  always @(state, instruction, updateAddr, inc_offset, vWR_p, V_flag) begin

    case (state)

      start: begin
        nextState = newPC;
      end

      newPC: begin
        nextState = fetchInst;
      end

      fetchInst: begin
        nextState = startEx;
      end

      startEx: begin

        case (instruction[15:12])
          vadd:
           nextState = executing;
          /*vdot:
          smul: */
          sst: begin
            nextState = executing;
          end
          vld: begin
            nextState = executing;
          end
          vst: begin
            nextState = executing;
          end
          sll: begin
            nextState = done;
          end
          slh: begin
            nextState = done;
          end
          j: begin
            nextState = newPC; //jump does not require the done state
          end
          nop: begin
            nextState = newPC; //no op does not require the done state
          end
          default:
            nextState = done;
        endcase

      end

      executing: begin

        case (instruction[15:12])
          vadd:begin
            if(vWR_p)
              nextState = done;
            else
              nextState = executing;
            end
          /*vdot:
          smul: */
          sst: begin
            if (updateAddr)
              nextState = executing;
            else
              nextState = done;
          end
          vld: begin
            if (inc_offset[3] & inc_offset [2] & inc_offset[1] & inc_offset[0])
              nextState = done;
            else
              nextState = executing;
          end
          vst: begin
            if (inc_offset[3] & inc_offset [2] & inc_offset[1] & inc_offset[0])
              nextState = done;
            else
              nextState = executing;
          end
          //j:
          //nop:
          default:
            nextState = done;
        endcase

      end

      done: begin
        if (V_flag)
          nextState = overflow;
        else
          nextState = newPC;
      end

      overflow: begin
        nextState = newPC;
      end

      default: begin
        nextState = 3'bx;
      end
    endcase

  end //combinational block

endmodule

//controls DRAM addressing
module addrUnit(output reg [15:0] addr, input Clk1, input [5:0] imm_offset,
    input [15:0] addrBase, input [15:0] PC, input setPC, input updateAddr,
    input [3:0] inc_offset);

  reg [15:0] ex_offset;
  reg [15:0] ex_inc_offset;

  always@(posedge Clk1) begin

    if (setPC)
      addr <= PC;
    else if (updateAddr)
      addr <= addrBase + ex_offset + ex_inc_offset;
    else
      addr <= addr;

  end

  always@(imm_offset, inc_offset) begin
    ex_offset = { {10{imm_offset[5]}}, imm_offset};
    ex_inc_offset = { {12{imm_offset[3]}}, inc_offset};
  end

endmodule

//Calculates address offsets
module offsetu(output reg [3:0] offset, input Reset, input Clk2,
    input offsetInc);

  always@(posedge Clk2) begin
    if (Reset)
      offset <= 4'b0;
    else
      if (offsetInc)
        offset <= offset + 1;
      else
        offset <= offset;
  end

endmodule

//Updates PC
module PCunit(output reg [15:0] PC, input [11:0] offset, input Clk2, input updatePC,
    input jump, input reset, input overflow);

  reg [15:0] ex_offset;

  always@(posedge Clk2) begin

    if (reset)
      PC <= 16'h0000;
    else if (overflow)
      PC <= 16'hfff0;
    else
      if (updatePC)

        if (jump)
          PC <= PC + ex_offset;
        else
          PC <= PC + 1;

      else
        PC <= PC;
  end

  always@(offset) begin
    ex_offset = { {4{offset[11]}}, offset};
  end

endmodule

//Eight 16-bit scalar registers
module sReg(output reg [15:0] DataOut, input [2:0] Addr, input Clk1, input Clk2,
    input [15:0] DataIn, input RD, input WR, input WR_l, input WR_h);

  reg [15:0] scalar[7:0];
  reg [2:0] address;
  wire [3:0] cmd;
  parameter read = 4'b1000, write = 4'b0100, wr_low = 4'b0010, wr_high = 4'b0001;

  assign cmd = {RD, WR, WR_l, WR_h};

  always@(posedge Clk1) begin

    case (cmd)
      read: begin
        DataOut <= scalar[address];
        scalar[address] <= scalar[address];
      end
      write: begin
        scalar[address] <= DataIn;
        DataOut <= DataOut;
      end
      wr_low: begin
        scalar[address] <= {scalar[address][15:8],DataIn[7:0]};
        DataOut <= DataOut;
      end
      wr_high: begin
        scalar[address] <= {DataIn[15:8],scalar[address][7:0]};
        DataOut <= DataOut;
      end
      default: begin
        scalar[address] <= scalar[address];
        DataOut <= DataOut;
      end
    endcase

  end

  always@(posedge Clk2) begin
    address <= Addr;
  end
endmodule


module VADD(output reg [15:0] Sum,output reg Overflow,input [15:0] A, input [15:0] B);
  
  reg [14:0] operA,operB;
  wire [14:0] operA1,operB1,operA2,operB2;
  reg [14:0] operSum;
  reg [11:0] opersum1;
  reg [10:0] opersum2;
  reg [4:0] DiffE;
  reg [5:0] i;
  reg [5:0] j;
  reg [1:0] oper;
  reg flag;
  reg flaginf;
  
  //append sticky and guard bit along with normalised one
  assign operA1 = {1'b0,1'b1,A[9:0],3'b000};
  assign operB1 = {1'b0,1'b1,B[9:0],3'b000}; 
  assign operA2 = {2'b0,A[9:0],3'b000};
  assign operB2 = {2'b0,B[9:0],3'b000};
  
  //perform floating point addition
  always @(*)
  begin
  if((A[14:10] == 5'h1F) || (B[14:10] == 5'h1F))
  begin
     if (A[14:10] == 5'h1F) 
       begin
         Sum = {A[15],A[14:10],10'h0};
         Overflow = 1'b1;
       end
     else 
       begin
         Sum = {B[15],B[14:10],10'h0};
         Overflow = 1'b1;
       end
  end
  else
  begin
  if((A[14:10] == 5'h0) && (B[14:10] == 5'h0))
  begin 
    operA = operA2;
    operB = operB2;
  end
  else if (A[14:10] == 5'h0)
  begin
    operA = operA2;
    operB = operB1;
  end
  else if(B[14:10] == 5'h0)
  begin
    operA = operA1;
    operB = operB2;
  end
  else
  begin
    operA = operA1;
    operB = operB1;
  end
  
   
  //if exponent of A > B    
  if (A[14:10] > B[14:10])
    begin
      DiffE=A[14:10]-B[14:10];//calculate the difference in exponent
      
      for(i=0;i<32;i=i+1)
      begin
        if(DiffE >0)
          begin
            if(operB[0] == 1'b1)
              begin
                operB = {1'b0,(operB[14:2]),1'b1}; // adjust the mantissa
                DiffE = DiffE-1'b1;
              end
            else
              begin
                operB = operB >> 1;
                DiffE = DiffE -1'b1 ;
              end
          end
        else
          begin
          operB = operB;
          end
      end
      
     Sum[14:10]= A[14:10];//assign final sum exponent
     if(A[15] != B[15])
       begin
       Sum[15] = A[15];  //assign sign of sum based on value of A and B         
       end
    end

//if exponent of B > A
  else if (B[14:10] > A[14:10])
    begin
      DiffE=B[14:10]-A[14:10];//calculate the difference in exponent
      
      for(i=0;i<32;i=i+1)
      begin
        if(DiffE >0)
          begin
            if(operA[0] == 1'b1)
              begin
                operA = {1'b0,(operA[14:2]),1'b1}; // adjust the mantissa
                DiffE = DiffE-1'b1;
              end
            else
              begin
                operA = operA >> 1'b1;
                DiffE = DiffE -1'b1;
              end
          end
        else
          begin
          operA = operA;
          end
      end 
      
      Sum[14:10]= B[14:10]; //assign the exponent to the sum
      if(A[15] != B[15])
        begin
        Sum[15] = B[15];  // assign the sign for the Sum         
        end       
    end

else // if exponents are equal
  begin
    operA = operA; //
    operB = operB; // no changes to A and B
    Sum[14:10] = A[14:10];// assign exponent value to output
    if(operA > operB)
      begin
        Sum[15] = A[15];// assign sign to output based on magnitude of A and B
      end
    else
      begin
        Sum[15] = B[15];// assign sign to output
      end      
  end

// based on sign of the elements perform the mathematical operation
// if sign is same perform addition
if(A[15] == B[15])
  begin
    operSum = operA + operB;
    oper = 2'b01;
  end
// if val A > B
else if (operA > operB)
  begin
    operSum = operA - operB;
    oper = 2'b10;
  end
// if val of B > A
else if (operB > operA)
  begin
    operSum = operB - operA;
    oper = 2'b11;
  end
 // if A == B and sign is opposite the answer is 0
 else if (operB == operA)
   operSum = 15'h0;


// normalising for sum
if(operSum[14] == 1'b1)
  begin
    if(Sum[14:10] != 5'h1E)
      begin
        operSum = operSum >> 1'b1;
        Sum[14:10] = Sum[14:10] + 1'b1;
        flaginf = 1'b0; 
        Overflow = 1'b0;
      end
    else
      begin
        operSum = operSum;
        Sum[14:10] = 5'h1F;
        //Sum[9:0] = 10'h0;
        flaginf = 1'b1;
        Overflow = 1'b1;
      end
  end
/* else if(Sum[14:0] == 5'b0)
  begin
    if (operSum[13] == 1'b1)
      begin*/
        
else 
   begin
     flag =1'b0;
     flaginf = 1'b0;
     Overflow = 1'b0;
     for(j=0;j<14;j=j+1)
     begin
        if((operSum[13]-j)== 1'b0)
          begin
            if(flag == 1'b0)
              begin
              operSum = operSum << 1'b1;
              Sum[14:10] = Sum[14:10] - 1'b1; 
              end
            else
              begin
              operSum = operSum;
              Sum[14:10] = Sum[14:10]; 
              end              
          end
        else
          begin
          operSum = operSum;
          Sum[14:10] = Sum[14:10];
          flag = 1'b1;
          end
    end
  end 

// rounding function based on IEEE standards after normalising
if (operSum[2] == 1'b1)
  opersum1 = operSum[14:3] + 1'b1;
else
  opersum1 = operSum[14:3];
  
  //normalising function after rounding
 if (opersum1[11] == 1'b1)
   begin
    if(flaginf != 1)    
     begin
     opersum2 = opersum1[11:1];
     Sum[14:10] = Sum[14:10] + 1'b1; 
     end 
    else
     begin
     opersum2 = opersum1[10:0];
     Sum[14:10] = Sum[14:10];
     end
   end
 else
   begin
    opersum2 = opersum1[10:0];
    Sum[14:10] = Sum[14:10];
   end
 
 // assign mantissa to the sum
 if (flaginf != 1'b1)
   begin
     Sum[9:0] = opersum2[9:0];
   end
 else
   begin
     Sum[9:0] = 10'h0;
   end
      
// if both elements are of same sign assign the sign to sum
 if(A[15] == B[15])
   begin
   Sum[15] = A[15];
   end
 end  
 end

endmodule  

module VADD16(output [255:0] SumV,output Overflw,input [255:0] Inval1,input [255:0] Inval2,input start,output done);
  
  wire [15:0] Ov;
  
  assign Overflw = Ov[0] |Ov[1] | Ov[2] | Ov[3] | Ov[4] | Ov[5] | Ov[6] | Ov[7] | Ov[8] | Ov[9] | Ov[10] | Ov[11] | Ov[12] | Ov[13] | Ov[14] | Ov[15] ; 
  assign done = start;
  
  VADD add1(SumV[15:0],Ov[0],Inval1[15:0],Inval2[15:0]);
  VADD add2(SumV[31:16],Ov[1],Inval1[31:16],Inval2[31:16]);
  VADD add3(SumV[47:32],Ov[2],Inval1[47:32],Inval2[47:32]);
  VADD add4(SumV[63:48],Ov[3],Inval1[63:48],Inval2[63:48]);
  VADD add5(SumV[79:64],Ov[4],Inval1[79:64],Inval2[79:64]);
  VADD add6(SumV[95:80],Ov[5],Inval1[95:80],Inval2[95:80]);
  VADD add7(SumV[111:96],Ov[6],Inval1[111:96],Inval2[111:96]);
  VADD add8(SumV[127:112],Ov[7],Inval1[127:112],Inval2[127:112]);
  VADD add9(SumV[143:128],Ov[8],Inval1[143:128],Inval2[143:128]);
  VADD add10(SumV[159:144],Ov[9],Inval1[159:144],Inval2[159:144]);
  VADD add11(SumV[175:160],Ov[10],Inval1[175:160],Inval2[175:160]);
  VADD add12(SumV[191:176],Ov[11],Inval1[191:176],Inval2[191:176]);
  VADD add13(SumV[207:192],Ov[12],Inval1[207:192],Inval2[207:192]);
  VADD add14(SumV[223:208],Ov[13],Inval1[223:208],Inval2[223:208]);
  VADD add15(SumV[239:224],Ov[14],Inval1[239:224],Inval2[239:224]);
  VADD add16(SumV[255:240],Ov[15],Inval1[255:240],Inval2[255:240]);
 
  
endmodule

//Eight 16x16-bit vector registers
module vReg(output reg [255:0] DataOut_p, output reg [255:0] DataOut2_p,
    output reg [15:0] DataOut_s, output reg [15:0] DataOut2_s,
    input [2:0] Addr, input [2:0] Addr2, input Clk1, input Clk2, 
    input [255:0] DataIn_p, input [15:0] DataIn_s, 
    input RD_p, input WR_p, input RD_s, input WR_s);

  reg [15:0] vector[7:0][15:0];
  integer i;  //used in for loop
  wire [3:0] cmd;
  reg [2:0] address, address2;
  parameter readp = 4'b1000, writep = 4'b0100, reads = 4'b0010, writes = 4'b0001;
  reg prev_WR_s, prev_RD_s;
  reg [3:0] select;

  assign cmd = {RD_p, WR_p, RD_s, WR_s}; 

  always@(posedge Clk1) begin

    prev_RD_s <= RD_s;
    prev_WR_s <= WR_s;

    case (cmd)
      readp: begin
      DataOut_p <= {vector[address][15], vector[address][14], 
               vector[address][13], vector[address][12], vector[address][11], 
               vector[address][10], vector[address][9], vector[address][8], 
               vector[address][7], vector[address][6], vector[address][5], 
               vector[address][4], vector[address][3], vector[address][2], 
               vector[address][1], vector[address][0]}; 
      DataOut2_p <= {vector[address2][15], vector[address2][14], 
               vector[address2][13], vector[address2][12], vector[address2][11], 
               vector[address2][10], vector[address2][9], vector[address2][8], 
               vector[address2][7], vector[address2][6], vector[address2][5], 
               vector[address2][4], vector[address2][3], vector[address2][2], 
               vector[address2][1], vector[address2][0]}; 

      for(i = 0; i < 16; i = i + 1) begin
        vector[address][i] <= vector[address][i];
      end

        DataOut_s <= DataOut_s;  
        DataOut2_s <= DataOut2_s;
      end

      writep: begin
        vector[address][0] <= DataIn_p[15:0];
        vector[address][1] <= DataIn_p[31:16];
        vector[address][2] <= DataIn_p[47:32];
        vector[address][3] <= DataIn_p[63:48];
        vector[address][4] <= DataIn_p[79:64];
        vector[address][5] <= DataIn_p[95:80];
        vector[address][6] <= DataIn_p[111:96];
        vector[address][7] <= DataIn_p[127:112];
        vector[address][8] <= DataIn_p[143:128];
        vector[address][9] <= DataIn_p[159:144];
        vector[address][10] <= DataIn_p[175:160];
        vector[address][11] <= DataIn_p[191:176];
        vector[address][12] <= DataIn_p[207:192];
        vector[address][13] <= DataIn_p[223:208];
        vector[address][14] <= DataIn_p[239:224];
        vector[address][15] <= DataIn_p[255:240];

        DataOut_p <= DataOut_p;
        DataOut2_p <= DataOut2_p;

        DataOut_s <= DataOut_s;  
        DataOut2_s <= DataOut2_s;
      end

      reads: begin
        DataOut_s <= vector[address][select];
        DataOut2_s <= vector[address2][select];

      for(i = 0; i < 16; i = i + 1) begin
        vector[address][i] <= vector[address][i];
      end
        DataOut_p <= DataOut_p;
        DataOut2_p <= DataOut2_p;
      end

      writes: begin

        DataOut_p <= DataOut_p;
        DataOut2_p <= DataOut2_p;

        for(i = 0; i < 16; i = i + 1) begin
          if (i == select)
            vector[address][select] <= DataIn_s;
          else
            vector[address][i] <= vector[address][i];
          end
        end

      default: begin
        DataOut_p <= DataOut_p;
        DataOut2_p <= DataOut2_p;

        for(i = 0; i < 16; i = i + 1) begin
          vector[address][i] <= vector[address][i];
        end

          DataOut_s <= DataOut_s;  
          DataOut2_s <= DataOut2_s;
      end
    endcase

    if ((~prev_RD_s && RD_s) || (~prev_WR_s && WR_s))
      select <= 1;
    else if (prev_RD_s || prev_WR_s)
      select <= select + 1;
    else
      select <= 0;
  end

  always@(posedge Clk2) begin
    address <= Addr;
    address2 <= Addr2;
  end

endmodule
