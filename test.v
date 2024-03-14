`timescale 1ns / 1ps

`include "ctrl_encode_def.v"
module seg7x16(
    input clk,
    input rstn,
    input disp_mode,//0,digital,1,graph
    input[63:0] i_data,
    output [7:0] o_seg,
    output [7:0] o_sel
    );
    
    reg [14:0] cnt;
    wire seg7_clk;
    always@ (posedge clk, negedge rstn)
    if(!rstn)
        cnt<=0;
    else 
        cnt<=cnt+1'b1;
    
    assign seg7_clk=cnt[14];
    
    
    reg [2:0] seg7_addr;
    
    always@(posedge seg7_clk, negedge rstn)
    if(!rstn)
        seg7_addr<=0;
    else
        seg7_addr<=seg7_addr + 1'b1;
        
    reg[7:0] o_sel_r;
    
      
    
    always @(*)
        case(seg7_addr)
            7:o_sel_r=8'b01111111;
            6:o_sel_r=8'b10111111;
            5:o_sel_r=8'b11011111;
            4:o_sel_r=8'b11101111;
            3:o_sel_r=8'b11110111;
            2:o_sel_r=8'b11111011;
            1:o_sel_r=8'b11111101;
            0:o_sel_r=8'b11111110;
        endcase
        
    reg[63:0] i_data_store;
    
    
    
    
    always @(posedge clk,negedge rstn)
    if(!rstn)
        i_data_store<=0;
    else
        i_data_store<=i_data;
        
    reg[7:0] seg_data_r;
    
    
    always@(*)
        if(disp_mode==1'b0) begin
            case(seg7_addr)
                0:seg_data_r=i_data_store[3:0];
                1:seg_data_r=i_data_store[7:4];
                2:seg_data_r=i_data_store[11:8];
                3:seg_data_r=i_data_store[15:12];
                4:seg_data_r=i_data_store[19:16];
                5:seg_data_r=i_data_store[23:20];
                6:seg_data_r=i_data_store[27:24];
                7:seg_data_r=i_data_store[31:28];
            endcase end
            else begin 
            case (seg7_addr)
                0: seg_data_r=i_data_store[7:0];
                1: seg_data_r=i_data_store[15:8];
                2: seg_data_r=i_data_store[23:16];
                3: seg_data_r=i_data_store[31:24];
                4: seg_data_r=i_data_store[39:32];
                5: seg_data_r=i_data_store[47:40];
                6: seg_data_r=i_data_store[55:48];
                7: seg_data_r=i_data_store[63:56];
                endcase end
                
            reg[7:0] o_seg_r;
            always @(posedge clk, negedge rstn)
                if(!rstn)
                    o_seg_r<=8'hff;
                    else if(disp_mode==1'b0) begin
                        case(seg_data_r)
                        4'h0: o_seg_r<=8'hC0;
                        4'h1: o_seg_r<=8'hF9;
                        4'h2: o_seg_r<=8'hA4;
                        4'h3: o_seg_r<=8'hB0;
                        4'h4: o_seg_r<=8'h99;
                        4'h5: o_seg_r<=8'h92;
                        4'h6: o_seg_r<=8'h82;
                        4'h7: o_seg_r<=8'hF8;
                        4'h8: o_seg_r<=8'h80;
                        4'h9: o_seg_r<=8'h90;
                        4'hA: o_seg_r<=8'h88;
                        4'hB: o_seg_r<=8'h83;
                        4'hC: o_seg_r<=8'hC6;
                        4'hD: o_seg_r<=8'hA1;
                        4'hE: o_seg_r<=8'h86;
                        4'hF: o_seg_r<=8'h8E;
                        default:o_seg_r<= 8'hFF;
                        endcase end
                        else begin o_seg_r<=seg_data_r;end
                        
                 assign o_sel = o_sel_r;
                 assign o_seg = o_seg_r;
                 
        endmodule         
        
module RF(input clk, input rst,input RFWr,input [15:0]sw_i,input [4:0]A1,input[4:0]A2,input[4:0] A3,input [31:0]WD,output [31:0] RD1,RD2);
        reg[31:0] rf[31:0];
        integer i;
    initial@(*)begin
        for(i=0;i<32;i=i+1)
            rf[i]<=i;
    end
       always@(posedge clk,negedge rst)
            if(!rst)begin
                for(i = 0; i <32;i = i+1)
                    rf[i] <= i;
             end
             else
                if(RFWr&&(!sw_i[1]))begin
                    rf[A3] <= WD;end
            assign RD1 = (A1!=0)?rf[A1]:0;   
            assign RD2 = (A2!=0)?rf[A2]:0;   
endmodule
       
module alu(input [15:0]sw_i ,input signed[31:0]A,B,input [4:0]ALUOP,output signed[31:0]C,output [7:0] Zero);
    reg signed [31:0] C_reg;
    reg [7:0] Zero_reg;
      always@(*)begin
        case(ALUOP)
           `ALUOp_add:C_reg=A+B;
           `ALUOp_sub:C_reg=A-B;
           `ALUOp_sll:C_reg = A << B;
           `ALUOp_srl:C_reg = A >> B;
           `ALUOp_slt:C_reg = (A < B)?1:0;
           `ALUOp_sltu :C_reg = (A < B)?1:0;
           `ALUOp_xor:C_reg = A ^ B;
           `ALUOp_sra:C_reg = A >> B;
           `ALUOp_or:C_reg = (A|B);
           `ALUOp_and:C_reg = A&B;
           default:C_reg=32'hFFFFFFFF;
        endcase
        Zero_reg=(C==0? 8'b1:8'b0);
        end
    assign C=C_reg;
    assign Zero=Zero_reg;
endmodule 

module dm( 
    input	clk,  //100MHZ CLK
    input rstn,
    input [15:0]sw_i,
    input 	DMWr,  //write signal
    input [5:0]	addr,
    input [31:0]din,
    input [2:0]DMType, 
    output reg [31:0]dout 
); 
    reg[7:0] dmem[31:0];
     integer i;
         initial@(*)begin
        for(i=0;i<32;i=i+1)
            dmem[i]<=0;
    end
        always@(posedge clk,negedge rstn)begin
           if(!rstn)begin
                for(i = 0; i <31;i = i+1)
                    dmem[i] <= 0;
                end
                else begin
                if(DMWr&&(!sw_i[1]))
                case(DMType)
                `dm_byte : dmem[addr] <= din[7:0];
                `dm_halfword :begin
                dmem[addr] <=din[7:0];
                dmem[addr+1]<=din[15:8];end
                `dm_word :begin
                dmem[addr] <=din[7:0];
                dmem[addr+1]<=din[15:8];
                dmem[addr+2]<=din[23:16];
                dmem[addr+3]<=din[31:24]; end
                endcase
                end
               end
         always@(*)begin
           case(DMType)
            `dm_byte :dout = {{24{dmem[addr][7]} },dmem[addr][7:0]};
            `dm_halfword : dout = {{16{dmem[addr+1][7]}},dmem[addr+1][7:0],dmem[addr][7:0]};
            `dm_word : dout = {dmem[addr+3][7:0],dmem[addr+2][7:0],dmem[addr+1][7:0],dmem[addr][7:0]};
            endcase
         end
            
endmodule

module ctrl(
    input [6:0] Op,  //opcode
    input [6:0] Funct7,  //funct7 
    input [2:0] Funct3,    // funct3 
    input Zero,
    output RegWrite, // control signal for register write
    output MemWrite, // control signal for memory write
    output	[5:0]EXTOp,    // control signal to signed extension
    output [4:0] ALUOp,    // ALU opertion
    //output [2:0] NPCOp,    // next pc operation
    output ALUSrc,   // ALU source for b
    output [2:0] DMType, //dm r/w type
    output [1:0]WDSel ,   // (register) write data selection  (MemtoReg)
//    output [2:0]isShamt
    output isJal,
    output itype_shamt,
    output isJalr
    );
    //rtype
wire rtype= ~Op[6]&Op[5]&Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0]; //0110011
wire i_add=rtype&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]&~Funct3[1]&~Funct3[0]; // add 0000000 000
wire i_sub=rtype&~Funct7[6]&Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]&~Funct3[1]&~Funct3[0]; // sub 0100000 000
wire i_sll=rtype&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]&~Funct3[1]&Funct3[0]; // sll 0000000 001
wire i_slt=rtype&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]&Funct3[1]&~Funct3[0]; // slt 0000000 010
wire i_sltu=rtype&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&~Funct3[2]&Funct3[1]&Funct3[0]; // sltu 0000000 011
wire i_xor=rtype&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&Funct3[2]&~Funct3[1]&~Funct3[0]; // xor 0000000 100
wire i_srl=rtype&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&Funct3[2]&~Funct3[1]&Funct3[0]; // srl 0000000 101
wire i_sra=rtype&~Funct7[6]&Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&Funct3[2]&~Funct3[1]&Funct3[0]; // srA 0100000 101
wire i_or=rtype&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&Funct3[2]&Funct3[1]&~Funct3[0]; // srA 0000000 110
wire i_and=rtype&~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0]&Funct3[2]&Funct3[1]&Funct3[0]; // srA 0000000 111
    
    //i_l type  
    wire itype_l  = ~Op[6]&~Op[5]&~Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0]; //0000011
    wire i_lb=itype_l&~Funct3[2]& ~Funct3[1]& ~Funct3[0]; //lb 000
    wire i_lh=itype_l&~Funct3[2]& ~Funct3[1]& Funct3[0];  //lh 001
    wire i_lw=itype_l&~Funct3[2]& Funct3[1]& ~Funct3[0];  //lw 010
    // i_i type
    wire itype_r  = ~Op[6]&~Op[5]&Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0]; //0010011
    wire i_addi  =  itype_r& ~Funct3[2]& ~Funct3[1]& ~Funct3[0]; // addi 000 func3
    wire i_slti  =  itype_r& ~Funct3[2]& Funct3[1]& ~Funct3[0]; //slti 010
    wire i_sltiu  =  itype_r& ~Funct3[2]& Funct3[1]& Funct3[0]; //sltiu 011
    wire i_xori  =  itype_r& Funct3[2]& ~Funct3[1]& ~Funct3[0]; //xori 100
    wire i_ori  =  itype_r& Funct3[2]& Funct3[1]& ~Funct3[0];   //ori 110
    wire i_andi  =  itype_r& Funct3[2]& Funct3[1]& Funct3[0];   //andi 111
    wire i_slli = itype_r& ~Funct3[2]& ~Funct3[1]& Funct3[0] & ~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0];  //slli 001 0000000 
    wire i_srli = itype_r& Funct3[2]& ~Funct3[1]& Funct3[0] & ~Funct7[6]&~Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0];  //srli 101 0000000
    wire i_srai = itype_r& Funct3[2]& ~Funct3[1]& Funct3[0] & ~Funct7[6]&Funct7[5]&~Funct7[4]&~Funct7[3]&~Funct7[2]&~Funct7[1]&~Funct7[0];
    // s format
    wire stype  = ~Op[6]&Op[5]&~Op[4]&~Op[3]&~Op[2]&Op[1]&Op[0];//0100011
    wire i_sw   = stype&~Funct3[2]& Funct3[1]& ~Funct3[0]; // sw 010
    wire i_sb=stype& ~Funct3[2]& ~Funct3[1]&~Funct3[0];
    wire i_sh=stype&& ~Funct3[2]&~Funct3[1]&Funct3[0];

    assign itype_shamt = i_slli | i_srli | i_srai;
    wire jtype = Op[6]&Op[5]&~Op[4]&Op[3]&Op[2]&Op[1]&Op[0];
    wire jalrtype = ~Funct3[2]&~Funct3[1]&~Funct3[0]& Op[6]&Op[5]&~Op[4]&~Op[3]&Op[2]&Op[1]&Op[0];
    //merge
    wire slt = i_slt | i_slti;
    wire sltu = i_sltu | i_sltiu;
    wire xor_ = i_xor | i_xori;
    wire or_ = i_or | i_ori;
    wire and_ = i_and | i_andi;
    wire sll = i_slli | i_sll;
    wire srl = i_srli | i_srl;
    wire sra = i_sra | i_srai;
    //操作指令生成控制信号（写、MUX选择）
    assign RegWrite   = rtype | itype_r|itype_l  ; // register write
    assign MemWrite   = stype;              // memory write
    assign ALUSrc     = itype_r | stype | itype_l |jalrtype ; // ALU B is from instruction immediate
    //mem2reg=wdsel ,WDSel_FromALU 2'b00  WDSel_FromMEM 2'b01
    assign WDSel[0] = itype_l;   
    assign WDSel[1] = 1'b0;    
    //ALUOp_nop 5'b00000
    //ALUOp_lui 5'b00001
    //ALUOp_auipc 5'b00010
    //ALUOp_add 5'b00011
    //操作指令生成运算类型aluop
    assign ALUOp[0]= i_add  | i_addi|stype|itype_l|sltu|or_|sll|sra |jalrtype;
    assign ALUOp[1]= i_add  | i_addi|stype|itype_l |slt|sltu|and_|sll|jalrtype;
    assign ALUOp[2] = i_sub | xor_|or_|and_|sll;
    assign ALUOp[3] = slt|sltu|xor_|or_|and_|sll;
    assign ALUOp[4] = srl|sra;
   
    //?操作指令生成常数扩展操作
    assign EXTOp[5] = i_slli | i_srai |i_srli;
    assign EXTOp[4]=(itype_l|jalrtype|itype_r)&&(~itype_shamt) ;
    assign EXTOp[3]=stype;
    assign EXTOp[0] = jtype;
    assign isJal = jtype;
    assign isJalr = jalrtype;
    // dm_word 3'b000
    //dm_halfword 3'b001
    //dm_halfword_unsigned 3'b010
    //dm_byte 3'b011
    //dm_byte_unsigned 3'b100
    //assign DMType[2]=i_lbu;
    //assign DMType[1]=i_lb | i_sb | i_lhu;
    //根据具体S和i_L指令生成DataMem数据操作类型编码
    assign DMType[1]=i_lb | i_sb;
    assign DMType[0]=i_lh | i_sh | i_lb | i_sb;
 endmodule
 
module EXT(
    input [4:0] iimm_shamt, //
    input [11:0]	iimm,  //instr[31:20], 12 bits
    input [11:0]	simm, //instr[31:25, 11:7], 12 bits
    input [11:0]	bimm,//instrD[31],instrD[7], instrD[30:25], instrD[11:8], 12 bits
    input [19:0]	uimm,
    input [19:0]	jimm,
    input [5:0]	 EXTOp,
    output reg [31:0] 	immout
    );
    always@(*)begin
        case (EXTOp)
		`EXT_CTRL_ITYPE_SHAMT:   immout<={27'b0,iimm_shamt[4:0]};
		`EXT_CTRL_ITYPE:	immout<={{20{iimm[11]}},iimm[11:0]};
		`EXT_CTRL_STYPE:	immout<={{20{simm[11]}},simm[11:0]};
		`EXT_CTRL_BTYPE:    immout<={{19{bimm[11]}},bimm[11:0],1'b0};
		`EXT_CTRL_UTYPE:	immout <= {uimm[19:0], 12'b0}; 
		`EXT_CTRL_JTYPE:	immout<={{11{jimm[19]}},jimm[19:0],1'b0};
		default:	        immout <= 32'b0;
	 endcase

    end
endmodule

module test(clk,rstn,sw_i,disp_seg_o,disp_an_o);
        input clk;
        input rstn;
        input [15:0] sw_i;
        output [7:0] disp_an_o,disp_seg_o;
        
        reg[31:0] clkdiv;
        wire Clk_CPU;
      always@(posedge clk or negedge rstn) begin
        if(!rstn)clkdiv<=0;
            else clkdiv<=clkdiv+1'b1;end
        assign Clk_CPU=(sw_i[15])?clkdiv[27]:clkdiv[25];
       
     reg[63:0] display_data;         //展示数据                
     reg [5:0] rom_addr;             //rom地址
     reg[3:0] alu_addr;              //alu地址
     reg[31:0] alu_disp_data;        //alu展示数据
     reg[4:0]reg_addr;               //寄存器地址
     reg[7:0]dmem_addr;              //dmem地址
     parameter IM_CODE_NUM = 22;
      parameter DM_DATA_NUM = 16;
     reg[63:0]LED_DATA[48:0];
     wire[31:0] instr;               //指令
     wire[7:0]Zero;                  //Zero信号
     reg[31:0] reg_data;            //reg数据
     reg[31:0] dmem_data;           //dm数据
     reg[31:0] WD;                  //write data
     wire[31:0] RD1,RD2;            //read data
     wire[31:0]inst_in;             //输入指令
     wire[31:0] aluout;             //alu结果
     wire[31:0] dout;               //dm输出
     wire[31:0] immout;             //扩展后数据
     wire[31:0] B ;                 //alu的第二个操作数
    //控制模块                       
    wire RegWrite;                  //reg写信号
    wire MemWrite;                  //mem写信号
    wire[5:0] EXTOp;                //扩展信号
    wire[4:0] ALUOp;                
    wire ALUSrc;                    
    wire[2:0] DMType;
    wire WDSel;                     //write data selection
    wire isJal;
    wire isJalr;
    wire itype_shamt;
      always@(sw_i)begin
      //11号开关显示alu结果
      if(sw_i[11]==1)display_data = alu_disp_data;
      else
      //12号开关显示dmem数据
      if(sw_i[12] == 1)display_data = {dmem_addr[3:0],dmem_data[27:0]};
      else 
      //13号开关显示reg的数据
        if(sw_i[13]==1)display_data = {reg_addr[3:0],reg_data[27:0]};
       //默认全0显示指令
        else display_data = inst_in;
      end

       //循环显示reg的值
      always@(posedge Clk_CPU or negedge rstn) begin 
        if(!rstn)begin  reg_addr = 5'b0;end
        else if(sw_i[13] == 1'b1)begin
        
            reg_addr = reg_addr + 1'b1;
            reg_data = U_RF.rf[reg_addr];
            if(reg_addr == 5'b10000)reg_addr = 5'b0;
            end
         end
         //循环显示alu的值
    always@(posedge Clk_CPU or negedge rstn) begin 
        if(!rstn)begin alu_addr=3'b0;end
        else if(sw_i[11]==1'b1)begin
        alu_addr=alu_addr+1'b1;
           case(alu_addr)
           3'b001:alu_disp_data=U_alu.A;
           3'b010:alu_disp_data=U_alu.B;
           3'b011:alu_disp_data=U_alu.C_reg;
           3'b100:alu_disp_data=U_alu.Zero_reg; 
              
        default:alu_disp_data=32'hFFFFFFFF;
           endcase
     end
     end
     //循环显示DM的值
      always@(posedge Clk_CPU or negedge rstn) begin 
       if(!rstn)begin dmem_addr=8'b0;end
        else if(sw_i[12]==1'b1)begin
        dmem_addr  =  dmem_addr + 1'b1;
        dmem_data = U_dm.dmem[dmem_addr];
 
        if(dmem_addr == DM_DATA_NUM)begin
        dmem_addr =  8'b0;dmem_data = 32'hFFFFFFFF;end
      end   
      end
      //地址变化

              always@(posedge Clk_CPU or negedge rstn)begin
         if(!rstn)begin  rom_addr = 32'b0;end
           else if(sw_i[14]==1'b1 && !sw_i[1]&&isJalr)begin
            rom_addr = aluout;end
         else if(sw_i[14]==1'b1 && !sw_i[1]&&!isJalr)begin
            rom_addr = rom_addr + 1'b1;
            if( rom_addr ==IM_CODE_NUM)begin  rom_addr = 6'd0;end end
            else rom_addr = rom_addr;
         end   
  
      always@(*)
        begin
            case(WDSel)
                 `WDSel_FromALU: WD<=aluout;
                `WDSel_FromMEM: WD<=dout;
                //`WDSel_FromPC: WD<=PC_out+4;
            endcase
        end
    //译码
    wire[6:0] Op = inst_in[6:0];  // op
    wire[6:0] Funct7 = inst_in[31:25]; // funct7
    wire[2:0] Funct3 = inst_in[14:12]; // funct3
    wire[4:0] rs1 = inst_in[19:15];  // rs1
    wire[4:0] rs2 = inst_in[24:20];  // rs2
    wire[4:0] rd = inst_in[11:7];  // rd
    wire[11:0] iimm=inst_in[31:20];//addi指令立即数，lw指令立即数
    wire[11:0] simm={inst_in[31:25],inst_in[11:7]}; //sw指令立即数
    wire[19:0] jimm={inst_in[31],inst_in[19:12],inst_in[20],inst_in[30:21]};
    wire [4:0] iimm_shamt = inst_in[24:20];

//       00209113
    //显示例化
     seg7x16 u_seg7x16(
     .clk(clk),
     .rstn(rstn),
     .i_data(display_data),
     .disp_mode(sw_i[0]),
     .o_seg(disp_seg_o),
     .o_sel(disp_an_o)
     );
    
     dist_mem_gen_1 U_IM(
     .a(rom_addr),
     .spo(inst_in)
          );
          
     ctrl U_ctrl(
     .Op(Op),
     .Funct7(Funct7),
     .Funct3(Funct3),
     .RegWrite(RegWrite),
     .MemWrite(MemWrite),
     .EXTOp(EXTOp),
     .ALUOp(ALUOp),
     .ALUSrc(ALUSrc),
     .DMType(DMType),
     .WDSel(WDSel),
     .isJal(isJal),
     .isJalr(isJalr)
     );
     
       EXT U_EXT(
       .iimm_shamt(iimm_shamt),
        .iimm(iimm),
        .simm(simm),
        .jimm(jimm),
        .EXTOp(EXTOp),
        .immout(immout)
        );    
     RF U_RF(.clk(Clk_CPU),.rst(rstn),.RFWr(RegWrite),.sw_i(sw_i),.A1(rs1),.A2(rs2),.A3(rd),.WD(WD),.RD1(RD1),.RD2(RD2));
     //第二个操作数来源
     assign B = (ALUSrc==1 ||itype_shamt==1 ) ? immout : RD2;

     alu U_alu(.A(RD1),.B(B),.C(aluout),.ALUOP(ALUOp),.Zero(Zero));
     dm U_dm(.clk(Clk_CPU),.rstn(rstn),.sw_i(sw_i),.DMWr(MemWrite),.addr(aluout),.din(RD2),.DMType(DMType),.dout(dout));
    endmodule
    
     
 
            
           

