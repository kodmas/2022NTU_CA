// Your code
module CHIP(clk,
            rst_n,
            // For mem_D
            mem_wen_D,
            mem_addr_D,
            mem_wdata_D,
            mem_rdata_D,
            // For mem_I
            mem_addr_I,
            mem_rdata_I
    );
    //==== I/O Declaration ========================
    input         clk, rst_n ;
    // For mem_D
    output        mem_wen_D  ;
    output [31:0] mem_addr_D ;
    output [31:0] mem_wdata_D;
    input  [31:0] mem_rdata_D;
    // For mem_I
    output [31:0] mem_addr_I ;
    input  [31:0] mem_rdata_I;

    //==== Reg/Wire Declaration ===================
    //---------------------------------------//
    // Do not modify this part!!!            //
    // Exception: You may change wire to reg //
    reg    [31:0] PC          ;              //
    reg  [31:0]  PC_nxt      ;              //
    reg          regWrite    ;              //
    wire   [ 4:0] rs1, rs2, rd;              //
    wire   [31:0] rs1_data    ;              //
    wire   [31:0] rs2_data    ;              //
    wire   [31:0] rd_data     ;              //
    //---------------------------------------//
    

    // Todo: other wire/reg
    reg [31:0] mem_wdata_D;
    reg [31:0] register [0:31];
    reg [31:0] next_register [0:31];
    reg mem_write;
    reg [31:0] mem_write_addr;            // to memory output
    reg [2:0]  ALU_ctrl ;            // ALU contorl
    reg [1:0]  ALU_op ;
    reg ALU_src	;
   
    wire [6:0]  funct7;
    wire [2:0]  funct3 ;
    wire [6:0]  opcode;
   
    reg Jal, Jalr;            // to recognize whether it is jal or jalr

   //alu
    wire [31:0] alu_in ;
    wire [31:0] alu_out ;  
    wire alu_finish   ; 

    reg [31:0] I_imm		 ; //immediate
    reg branch_control; //for beq control
    wire zero, jump	 ;            // to compute jump or not
    reg [2:0]  regWrite_src  ;            // to control register write part
    reg [20:0] tmp_Imm		 ;  // to generate extended immediate

    //assign output
    assign mem_wen_D = mem_write;
    assign mem_addr_D = mem_write_addr[31:0];
    assign mem_addr_I = PC[31:0]; 

    //func7 code
    parameter R_type = 7'b0110011;
    parameter I_type = 7'b0010011;
    parameter AUIPC_type  = 7'b0010111;
    parameter SW_type = 7'b0100011;
    parameter LW_type = 7'b0000011;
    parameter BEQ_type = 7'b1100011;
    parameter JAL_type = 7'b1101111;
    parameter JALR_type = 7'b1100111;

    //R-type instruction
    assign funct7 = mem_rdata_I[31:25];
    assign rs2 = mem_rdata_I[24:20];
    assign rs1 = mem_rdata_I[19:15];
    assign funct3 = mem_rdata_I[14:12];
    assign rd = mem_rdata_I[11:7];
    assign opcode = mem_rdata_I[6:0];
    assign zero = (register[rs1] == register[rs2])? 1 : 0;
    assign jump = (branch_control & zero) | Jal;

    //==== Submodule Connection ===================
    //---------------------------------------//
    // Do not modify this part!!!            //
    reg_file reg0(                           //
        .clk(clk),                           //
        .rst_n(rst_n),                       //
        .wen(regWrite),                      //
        .a1(rs1),                            //
        .a2(rs2),                            //
        .aw(rd),                             //
        .d(rd_data),                         //
        .q1(rs1_data),                       //
        .q2(rs2_data));                      //
    //---------------------------------------//

    // Todo: other submodules
    MUX MUX_ALUsrc(
    	.in1(register[rs2]),
    	.in2(I_imm),
    	.sig(ALU_src),
    	.out(alu_in));

    ALU ALU(
        .clk(clk),
        .rst_n(rst_n),
    	.in1(register[rs1]),
    	.in2(alu_in),
    	.ctrl(ALU_ctrl),
    	.out(alu_out),
        .finish(alu_finish));

    //==== Combinational Part =====================
    

    // Todo: any combinational/sequential circuit
    always@(*) begin
    	ALU_ctrl = 3'b111; // for ALU mode setting
    	
        ALU_src = 1'b0;
    	
        mem_write = 1'b0;
    	regWrite = 1'b0;
    	
        Jal = 1'b0;
    	Jalr = 1'b0;
    	
        branch_control = 1'b0;
    	I_imm = 32'b0;
    	
        regWrite_src = 3'b000;

        case(opcode)
            R_type:begin
                regWrite = 1'b1;
    		    ALU_src = 1'b0;
                case(funct3) 
                    3'b000: begin
                        case(funct7)
                            7'b0000000: ALU_ctrl = 3'b010; //ADD
                            7'b0100000: ALU_ctrl = 3'b011; //SUB
                            7'b0000001: begin
                                ALU_ctrl = 3'b100; //MUL
                                //$display("MUL");
                            end
                            default begin
                            end
                        endcase
                    end
                    3'b100: ALU_ctrl = 3'b001;	//XOR
                    3'b111: ALU_ctrl = 3'b000;	//AND
                    default begin
                    end
                endcase
                end
            I_type:begin
                regWrite = 1'b1;
    		    ALU_src = 1'b1;
                case(funct3)
                    3'b000:begin 							// ADDI
                        ALU_ctrl = 3'b010; 									// ADD operation
                        if (mem_rdata_I[31]) begin
                            I_imm =  {20'b11111111111111111111, mem_rdata_I[31:20]};
                        end
                        else begin
                            I_imm =  {20'b0, mem_rdata_I[31:20]};
                        end
                    end
                    3'b001:begin
                        ALU_ctrl = 3'b101; 	//SLLU
                        I_imm =  {26'b0, mem_rdata_I[25:20]};
                    end
                    3'b101:begin
                        ALU_ctrl = 3'b110; //SRLI
                        I_imm =  {26'b0, mem_rdata_I[25:20]};
                    end
                    3'b010:begin //SLTI
                        regWrite_src = 3'b100;
                        ALU_ctrl = 3'b011; 		// SUB operation
                        if (mem_rdata_I[31]) begin
                            I_imm =  {20'b11111111111111111111, mem_rdata_I[31:20]};
                        end
                        else begin
                            I_imm =  {20'b0, mem_rdata_I[31:20]};
                        end    
                    end
                    default begin
                    end
                endcase
                end
            AUIPC_type:begin
                regWrite_src = 3'b011;
                regWrite = 1'b1;
                ALU_src = 1'b1;
                I_imm = {mem_rdata_I[31:12], 12'b0};
            end
            AUIPC_type:begin
                regWrite_src = 3'b011;
                regWrite = 1'b1;
                ALU_src = 1'b1;
                I_imm = {mem_rdata_I[31:12], 12'b0};
            end
            SW_type:begin
                regWrite = 1'b0;
                ALU_ctrl = 3'b010;												 
                ALU_src = 1'b1;
                mem_write = 1'b1;
                if (mem_rdata_I[31]) begin
                    I_imm = {20'b11111111111111111111, mem_rdata_I[31:25], mem_rdata_I[11:7]};
                end
                else begin
                    I_imm = {20'b0, mem_rdata_I[31:25], mem_rdata_I[11:7]};
                end
            end
            LW_type:begin
                regWrite_src = 3'b010;
                regWrite = 1'b1;
                ALU_ctrl = 3'b010; 
                ALU_src = 1'b1;
                mem_write = 1'b0;
                if (mem_rdata_I[31]) begin
                    I_imm =  {20'b11111111111111111111, mem_rdata_I[31:20]};
                end
                else begin
                    I_imm =  {20'b0, mem_rdata_I[31:20]};
                end
            end
            BEQ_type:begin
                regWrite = 1'b0;
                ALU_ctrl = 3'b011; // SUB
                ALU_src = 1'b0;
                branch_control = 1'b1;
                if (mem_rdata_I[31]) begin
                    I_imm = {19'b1111111111111111111, mem_rdata_I[31], mem_rdata_I[7], mem_rdata_I[30:25], mem_rdata_I[11:8], 1'b0}; 
                end
                else begin
                    I_imm = {19'b0, mem_rdata_I[31], mem_rdata_I[7], mem_rdata_I[30:25], mem_rdata_I[11:8], 1'b0}; 
                end
            end
            JAL_type:begin
                regWrite = 1'b1;
                regWrite_src = 3'b001;
                Jal = 1'b1;
                Jalr = 1'b0;
                if (mem_rdata_I[31]) begin
                    I_imm = {11'b11111111111, mem_rdata_I[31], mem_rdata_I[19:12], mem_rdata_I[20], mem_rdata_I[30:21], 1'b0};
                end
                else begin
                    I_imm = {11'b0, mem_rdata_I[31], mem_rdata_I[19:12], mem_rdata_I[20], mem_rdata_I[30:21], 1'b0};
                end
            end
            JALR_type:begin
                regWrite = 1'b1;
                regWrite_src = 3'b001;
                Jal = 1'b0;
                Jalr = 1'b1;											 
                if (mem_rdata_I[31]) begin
                    I_imm =  {20'b11111111111111111111, mem_rdata_I[31:20]};
                end
                else begin
                    I_imm =  {20'b0, mem_rdata_I[31:20]};
                end
            end
            default begin
            end

        endcase
    end

    integer i;
    //handle register write
    always @(*) begin
        for(i=0;i<32;i = i+1)begin
            next_register[i] = register[i];
        end
        if(regWrite)begin
            case(regWrite_src)
                3'b000:next_register[rd] = alu_out;
                3'b001:next_register[rd] = PC + 32'd4;
                3'b010:next_register[rd] = mem_rdata_D;
                3'b011:next_register[rd] = PC + I_imm;
                3'b100: begin //SLTI
                    if (alu_out[31]) begin
                        next_register[rd] = {31'b0, 1'b1};
                    end
                    else begin
                        next_register[rd] = 32'b0;
                    end
                end
                default:begin
                end
            endcase
        end
    end

    //handle memory write
    always@(*) begin

        mem_write_addr = alu_out;
        if(mem_write) begin
        	mem_wdata_D = register[rs2];
        end
        else begin
            mem_wdata_D = 32'b0;
        end
    end 
    
    //handle next PC
    always@(*) begin
        PC_nxt = PC + 32'd4;
        if(Jalr) begin
            PC_nxt = register[rs1] + I_imm;
        end
        else begin
            if(jump) begin
                PC_nxt = PC + I_imm;
            end
            else begin
            	PC_nxt = PC + 32'd4;
            end
        end
    end

    //==== Sequential Part ========================
    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            PC <= 32'h00400000; // Do not modify this value!!!
            for(k = 0; k < 32; k = k+1) begin
                register[k] <= 32'd0;
            end
            register[32'b10] <= 32'h7fffeffc;
            register[32'b11] <= 32'h10008000;
        end
        else begin
            if (ALU_ctrl==3'b100) begin
                if (alu_finish==1) begin // multi-cycle operation
                    PC <= PC_nxt;
                    register[0] <= 32'd0;
                    for(k = 1; k < 32; k = k+1) begin
                        register[k] <= next_register[k];
                    end
                    //$display("MUL FINISH");
                end
                else begin
                    PC <= PC;
                    register[0] <= register[0];
                    for(k = 1; k < 32; k = k+1) begin
                        register[k] <= next_register[k];
                    end
                end
            end
            else begin
                PC <= PC_nxt;
                register[0] <= 32'd0;
                for(k = 1; k < 32; k = k+1) begin
                    register[k] <= next_register[k];
                end
            end  
        end
    end
endmodule






module reg_file(clk, rst_n, wen, a1, a2, aw, d, q1, q2);

    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth

    input clk, rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] d;
    input [addr_width-1:0] a1, a2, aw;

    output [BITS-1:0] q1, q2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign q1 = mem[a1];
    assign q2 = mem[a2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (aw == i)) ? d : mem[i];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem[0] <= 0; // zero: hard-wired zero
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'h7fffeffc; // sp: stack pointer
                    32'd3: mem[i] <= 32'h10008000; // gp: global pointer
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end
    end
endmodule

module MUX(in1,in2,sig,out);
    input [31:0] in1,in2;
    input sig;
    output[31:0] out;
    reg[31:0] out;
    always @(*) begin
        out =  (sig == 1'b1) ? in2 : in1;
    end
endmodule


module ALU(clk,rst_n,in1,in2,ctrl,out,finish);
    input clk,rst_n;
    input signed[31:0] in1,in2;
    input[2:0] ctrl;
    output signed[31:0] out;
    output finish;
    reg signed [32:0] reg_out;

    reg valid;
    wire ready, finish;
    wire [63:0] muldiv_out;
    wire [63:0] product;
    assign product = muldiv_out;
    reg [2:0] mode;

    parameter AND  = 3'b000;
	parameter XOR   = 3'b001;
	parameter ADD  = 3'b010;
	parameter SUB  = 3'b011;
	parameter MUL  = 3'b100;
    parameter SLLI = 3'b101;
    parameter SRLI = 3'b110;
    //parameter SLTI = 3'b111;
    
    mulDiv MULDIV(
        .clk(clk),
        .rst_n(rst_n),
        .valid(valid),
        .ready(ready),
        .mode(mode),
        .in_A(in1),
        .in_B(in2),
        .out(muldiv_out));

    
    always @(*) begin
        case(ctrl)
            AND:  reg_out = in1 & in2;
            XOR:   reg_out = in1 ^ in2;
            ADD:  reg_out = in1 + in2;
            SUB:  reg_out = in1 - in2;
            
            SLLI: reg_out = in1 << in2;
            SRLI: reg_out = in1 >> in2;

            MUL:begin 
                reg_out = product;
                mode = 3'b0;
                valid = 1;
            end
            default: begin 
                reg_out = 33'b0;
            end
        endcase
    end
    

    always @(*) begin
        case (ctrl)
            MUL: mode = 0;
            default: mode = 0;
        endcase
    end
    
    always @(*) begin
        case (ctrl)
            MUL: valid = 1;
            default: valid = 0;
        endcase
    end
    assign finish = ready;
    assign out = reg_out[31:0];
endmodule


//Below is HW2
module mulDiv(
    clk,
    rst_n,
    valid,
    ready,
    mode,
    in_A,
    in_B,
    out
);

    // Definition of ports
    input         clk, rst_n;
    input         valid;
    input  [1:0]  mode; // mode: 0: mulu, 1: divu, 2: shift, 3: avg
    output        ready;
    input  [31:0] in_A, in_B;
    output [63:0] out;

    // Definition of states
    parameter IDLE = 3'd0;
    parameter MUL  = 3'd1;
    parameter DIV  = 3'd2;
    parameter SHIFT = 3'd3;
    parameter AVG = 3'd4;
    parameter OUT  = 3'd5;

    // Todo: Wire and reg if needed
    reg  [ 2:0] state, state_nxt;
    reg  [ 4:0] counter, counter_nxt;
    reg  [63:0] shreg, shreg_nxt;
    reg  [31:0] alu_in, alu_in_nxt;
    reg  [32:0] alu_out;

    reg ready;
  
    reg ex, ex_next; //handling quotient  or other ex
    reg [32:0] temp_sum; //storage for temp sum
  
    // Todo: Instantiate any primitives if needed
    
    // Todo 5: Wire assignments
    assign out = shreg;
    // in_A = shreg[31:0];
    always @(state) begin
        if (state == OUT)
            ready = 1;
        else
            ready  = 0;
    end



    assign out = shreg;
    // Combinational always block
    // Todo 1: Next-state logic of state machine
    always @(*) begin
        case(state)
            IDLE: begin
                if(!valid) begin
                    state_nxt = IDLE;
                end
                else begin
                    case(mode)
                        2'b00 :state_nxt = MUL;
                        2'b01 :state_nxt = DIV;
                        2'b10 :state_nxt = SHIFT;
                        default:state_nxt = AVG;
                    endcase
                end
            end
            MUL : begin
                if(counter!= 31) begin 
                    state_nxt = MUL;
                end
                else begin 
                    state_nxt = OUT;
                end
            end
            DIV : begin
               if(counter!= 31) begin 
                    state_nxt = DIV;
                end
                else begin 
                    state_nxt = OUT;
                end
            end
            SHIFT : state_nxt = OUT;
            AVG : state_nxt = OUT;
            OUT : state_nxt = IDLE;
            default : state_nxt = IDLE;
        endcase
    end
    // Todo 2: Counter
    always @(*) begin
        case(state)
            MUL: begin 
                if(counter < 31) begin 
                    counter_nxt = counter + 1;

                end
                else begin
                    //counter = 0;
                    counter_nxt = 0;
                end
            end

            DIV:begin 
                if(counter < 31) begin 
                    counter_nxt = counter + 1;

                end
                else begin

                    counter_nxt = 0;
                end
            end
            default: counter_nxt = 0;
        endcase
    end

    // ALU input
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid) alu_in_nxt = in_B;
                else       alu_in_nxt = 0;
            end
            OUT : alu_in_nxt = 0;
            default: alu_in_nxt = alu_in;
        endcase
    end

    // Todo 3: ALU output
    always @(*) begin
        case(state)
            MUL: begin

                    if(shreg[0] == 0)begin
                        alu_out = shreg[63:32];

                    end
                    else begin
                        alu_out = alu_in + shreg[63:32];
                    end
 
            end

            DIV:
                begin

                    if(shreg[63:32] >= alu_in)begin
                        alu_out[31:0] =  shreg[63:32] - alu_in;
                        alu_out[32] = 1;
                        ex_next = 1;
                    end
                    else begin
                        alu_out[31:0] =  shreg[63:32]; 
                        alu_out[32] = 0;
                        ex_next = 0;
                    end
/*
                    if(shreg[62:31] >= alu_in)begin
                        alu_out[31:0] =  shreg[62:31] - alu_in;
                        alu_out[32] = 1;
                        ex_next = 1;
                    end
                    else begin
                        alu_out[31:0] =  shreg[62:31]; 
                        alu_out[32] = 0;
                        ex_next = 0;
                    end*/

            end 
            SHIFT:
                begin
                    alu_out = shreg[31:0] >> alu_in[2:0]; 
                    ex_next = 0;
  
                end
            AVG:
                begin
                    temp_sum = (shreg[31:0] + alu_in);
                    alu_out = (temp_sum >>> 1);
                    ex_next = 0;
                end
            default: begin 
                alu_out = 33'b0;
                ex_next = 0;
            end
        endcase
    end
    // Todo 4: Shift register
    always @(*) begin
        case(state)
            IDLE: begin
                if(valid)begin
                    if(state_nxt == DIV)begin
                        shreg_nxt = {31'b0,in_A,1'b0};
                    end
                    else begin 
                        shreg_nxt[31:0] = in_A;
                        shreg_nxt[63:32] = 32'b0;
                    end
                end
                else shreg_nxt = 64'b0;
            end
            MUL:begin


                shreg_nxt = {alu_out,shreg[31:1]};

                ex_next = 0;
            end
            DIV:begin
                    if(counter < 31) begin
                        shreg_nxt[63:0] = {alu_out[30:0],shreg[31:0],alu_out[32]};
                    end
                    else begin
                        shreg_nxt[63:0] = {1'b0,alu_out[30:0],shreg[30:0],alu_out[32]};
                    end

            end
            SHIFT:
                begin

                    shreg_nxt[32:0] = alu_out;
                    shreg_nxt[63:33] = 31'b0;
                    ex_next = ex;
                end
            AVG:
            begin
                shreg_nxt[32:0] = alu_out;
                shreg_nxt[63:33] = 31'b0;
                ex_next = ex;
            end
            default: begin
                shreg_nxt = shreg;
                ex_next = ex;
            end
        endcase
    end   
    // Todo: Sequential always block
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
        end
        else begin
            state <= state_nxt;
            shreg <= shreg_nxt;
            alu_in <= alu_in_nxt;
            counter <= counter_nxt;
            ex <= ex_next;
        end
    end

endmodule