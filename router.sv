/* Author: Farjad Zaim (fzaim)
 * A router transfers packets between nodes and other routers.
 */
typedef enum bit [2:0]{ZERO = 3'd0, ONE = 3'd1, TWO = 3'd2, THREE = 3'd3, NONE = 3'd4} router_port;
typedef struct packed {
  router_port tar;
} target_t;
typedef struct packed {
  router_port tar_in;
  router_port tar_out;  
  logic clr;
  logic [5:0] count;
  logic [2:0] waitTime;
} target_map;

module router(clk, rst_b,
              free_outbound, put_outbound, payload_outbound,
              free_inbound, put_inbound, payload_inbound);
  parameter ROUTERID = 0; // To differentiate between routers
  input  clk, rst_b;

  // self -> destination (sending a payload)
  input [3:0] free_outbound;
  output [3:0] put_outbound;
  output [3:0][7:0] payload_outbound;

  // source -> self (receiving a payload)
  output [3:0] free_inbound;
  input [3:0] put_inbound;
  input [3:0][7:0] payload_inbound;
  
  bit [3:0] sr_out_load;
  bit [3:0][31:0] sr_out_data_in;
  bit [3:0] sr_out_empty;
  bit [3:0] sr_in_clr, sr_in_full;
  bit [3:0][31:0] sr_in_data_out;
  bit [3:0][5:0] sr_in_count;		//Cycles Spent Full
  bit [3:0][2:0] sr_in_waitTime; //Cycles Spent Empty

  /* YOUR CODE... */
  
  
  genvar k;

  // generate shiftregs
  generate
    for(k=0; k<4; k=k+1) begin: shiftRegLoop
      shiftreg_in #(k) sr_in_inst(clk, rst_b, sr_in_clr[k], put_inbound[k],
											payload_inbound[k], sr_in_full[k], free_inbound[k],
											sr_in_count[k], sr_in_waitTime[k], sr_in_data_out[k]);
		shiftreg_out #(k) sr_out_inst(clk, rst_b, sr_out_load[k], free_outbound[k], 
												sr_out_data_in[k], put_outbound[k], sr_out_empty[k],
												payload_outbound[k]);

    end
  endgenerate
  arbiter #(ROUTERID) arbiter_inst(clk, rst_b, sr_in_full, sr_out_empty, sr_in_count, sr_in_waitTime,
										  sr_in_data_out, sr_out_load, sr_in_clr, sr_out_data_in);
  

endmodule

module arbiter (input clk, rst_b,
					 input [3:0] sr_in_rdy,  
					 input [3:0]sr_out_rdy,
					 input [3:0][5:0] count,
					 input [3:0][2:0] waitTime,
					 input [3:0][31:0] data_in,
					 output logic [3:0] load,
					 output logic [3:0] sr_in_clr,
					 output bit [3:0][31:0] data_out);
	parameter ROUTERID = 0;
	
	target_t [3:0]input_target;
	target_map[3:0] input_target_map;
	target_map temp_map;
	target_t [3:0]output_source;
	bit [3:0][3:0] to_sr_in_clr; //[output_buf #][input_buf #] 
	genvar j, k;
	generate 
		for (k = 0; k<4; k=k+1) begin: outputLoop
			router_input_target_selector #(ROUTERID, k) target_sel_inst (sr_in_rdy[k], data_in[k], input_target[k]);
			router_output_source_selector #(k) output_source_sel_inst (input_target_map, waitTime, count, sr_out_rdy[k], output_source[k]); //The logic
			
			always_comb begin
					data_out[k] = 32'd0; load[k] = 1'b1; to_sr_in_clr[k] = 4'd0;
					case(output_source[k].tar)	//More scalable or automated approach??? When using for loop, run into issue of casting to type of enum
						ZERO: begin data_out[k] = data_in[0]; to_sr_in_clr[k][0] = 1'b1; end  
						ONE: begin data_out[k] = data_in[1]; to_sr_in_clr[k][1] = 1'b1; end
						TWO: begin data_out[k] = data_in[2]; to_sr_in_clr[k][2] = 1'b1; end
						THREE: begin data_out[k] = data_in[3]; to_sr_in_clr[k][3] = 1'b1; end
						NONE: load[k] = 1'b0;
						default: load[k] = 1'b0;
					endcase
			end
		end
		for (k = 0; k<4; k=k+1) begin: inputLoop
			assign sr_in_clr[k] = to_sr_in_clr[0][k] || to_sr_in_clr[1][k] || to_sr_in_clr[2][k] || to_sr_in_clr[3][k]; 
		end
	endgenerate
	
	priority_selector priority_sel_inst (clk, rst_b, sr_in_clr, count,waitTime,input_target, input_target_map);
	

	
endmodule


module priority_selector (input clk, rst_b,
					input [3:0]sr_in_clr, 
					input[3:0][5:0] count_in,
					input[3:0][2:0] waitTime_in,
					input target_t[3:0] input_target,
					output target_map[3:0] priorityList);
	logic[3:0][1:0] sel;
	logic[3:0][3:0][1:0]nextSel, sel_stream;
	logic [3:0][3:0] nextClr, clr_stream;
	logic [3:0][5:0] nextCount, count_stream;
	logic [3:0][2:0] nextWaitTime, waitTime_stream;
	logic [3:0] clr;
	logic [5:0] count;
	logic [2:0] waitTime; 
	target_map[3:0] nextMap;
	
	const logic[3:0][1:0] SEL_INITIAL = {2'd3, 2'd2, 2'd1, 2'd0};
	assign clr = {priorityList[3].clr, priorityList[2].clr, priorityList[1].clr, priorityList[0].clr};
	assign count = {priorityList[3].count, priorityList[2].count,priorityList[1].count,priorityList[0].count};
	assign waitTime = {priorityList[3].waitTime, priorityList[2].waitTime, priorityList[1].waitTime, priorityList[0].waitTime};
	
	
	genvar j, k;
	generate
		for (k = 0;k<4; k++) begin: outputCaseLoop
			always_comb begin 
				case(sel[k])
					2'd0: begin priorityList[k].tar_in = ZERO; 		priorityList[k].tar_out = input_target[0].tar; priorityList[k].clr = sr_in_clr[0];
									priorityList[k].count = count_in[0]; priorityList[k].waitTime = waitTime_in[0]; end
					2'd1: begin priorityList[k].tar_in = ONE; 	priorityList[k].tar_out = input_target[1].tar; priorityList[k].clr = sr_in_clr[1]; 
									priorityList[k].count = count_in[1]; priorityList[k].waitTime = waitTime_in[1]; end
					2'd2:	begin priorityList[k].tar_in = TWO; 	priorityList[k].tar_out = input_target[2].tar; priorityList[k].clr = sr_in_clr[2]; 
									priorityList[k].count = count_in[2]; priorityList[k].waitTime = waitTime_in[2]; end
					2'd3:	begin priorityList[k].tar_in = THREE;	priorityList[k].tar_out = input_target[3].tar; priorityList[k].clr = sr_in_clr[3]; 
									priorityList[k].count = count_in[3]; priorityList[k].waitTime = waitTime_in[3]; end
					default: begin priorityList[k].tar_in = ZERO;priorityList[k].tar_out = input_target[0].tar; priorityList[k].clr = sr_in_clr[0]; 
									priorityList[k].count = count_in[0]; priorityList[k].waitTime = waitTime_in[0]; end
				endcase
			end
			
			logic [k:0][3:0][1:0] localSel_in, localSel_out;
			logic [k:0][3:0] localClr_in, localClr_out;
			logic [k:0][3:0][5:0] localCount_in, localCount_out;
			logic [k:0][3:0][2:0] localWaitTime_in, localWaitTime_out;
			
			always_comb begin
				if (k==0) begin
					sel_stream[k] = sel;
					clr_stream[k] = clr;
					count_stream[k] = count;
					waitTime_stream[k] = waitTime; 
				end
				else begin
					sel_stream[k] = nextSel[k-1];
					clr_stream[k] = nextClr[k-1];
					count_stream[k] = nextCount[k-1];
					waitTime_stream[k] = nextWaitTime[k-1];
				end		
				nextSel[k] = (~rst_b)? SEL_INITIAL: localSel_out[0];
				nextClr[k] = (~rst_b)? sr_in_clr: localClr_out[0];
				nextCount[k] = (~rst_b)? count_in: localCount_out[0];
				nextWaitTime[k] = (~rst_b)? waitTime_in : localWaitTime_out[0];
			end
			
			for (j = k; j>=0; j--) begin: selIteration
				always_comb begin
					if(j==k) begin localSel_in[j]=sel_stream[k]; localClr_in[j]=clr_stream[k]; localCount_in[j] = count_stream[k]; localWaitTime_in[j] = waitTime_stream[k]; end
					else begin localSel_in[j] = localSel_out[j+1]; localClr_in[j] = localClr_out[j+1]; localCount_in[j] = localCount_out[j+1]; localWaitTime_in[j] = localWaitTime_out[j+1];end
				end
					sel_iteration #(j)(localSel_in[j], localClr_in[j], localCount_in[j], localWaitTime_in[j],
											 localSel_out[j], localClr_out[j], localCount_out[j], localWaitTime_out[j]);
			end
		end
	endgenerate

	always_ff @(posedge clk, negedge rst_b) begin
		if(~rst_b)  sel <= SEL_INITIAL; 
		else sel<= nextSel[3]; 
	end
	
endmodule
					
module sel_iteration(input logic [3:0][1:0]sel_in, input logic [3:0] clr_in,
							input logic [3:0][5:0] count_in, input logic [3:0][2:0] waitTime_in,
							output logic [3:0][1:0] sel_out, output logic [3:0] clr_out,
							output logic [3:0][5:0] count_out, output logic [3:0][2:0] waitTime_out);
	parameter iterationNumber = 0;
	const logic[5:0] MAX_ACCEPTABLE_COUNT = 6'd2;
	const logic[2:0] MIN_WAITTIME = 3'd0;
	logic reducePriority;
	logic currClr, nextClr;
	logic currMinWait, nextMinWait;
	logic currMaxCount, nextMaxCount, nextGreaterCount;
	
	assign currClr = clr_in[iterationNumber];
	assign nextClr = (iterationNumber ==0)? 1'b0: clr_in[iterationNumber-1];
	assign currMinWait = (waitTime_in[iterationNumber] <= MIN_WAITTIME)? 1'b1: 1'b0;
	assign nextMinWait = (iterationNumber ==0)? 1'b0: (waitTime_in[iterationNumber-1] <= MIN_WAITTIME)? 1'b1: 1'b0;
	assign currMaxCount = (count_in[iterationNumber] >= MAX_ACCEPTABLE_COUNT)? 1'b1: 1'b0;
	assign nextMaxCount = (iterationNumber ==0) ? 1'b0: (count_in[iterationNumber-1] >= MAX_ACCEPTABLE_COUNT)? 1'b1: 1'b0;
	assign nextGreaterCount = (iterationNumber == 0)? 1'b0: (count_in[iterationNumber-1] >  count_in[iterationNumber]);
	
		always_comb begin
			clr_out = clr_in;
			sel_out = sel_in;
			count_out = count_in;
			waitTime_out = waitTime_in;
			reducePriority = 1'b0;
			if (iterationNumber != 0) begin
				if (currClr) begin
					if(nextClr) begin
						if(nextMinWait && ~currMinWait) reducePriority = 1'b1;
					end
					else begin
						if(currMinWait) begin
							if (nextMinWait) reducePriority = 1'b1;
							else if (nextMaxCount) reducePriority = 1'b1;
						end
						else reducePriority = 1'b1;
					end
				end
				else begin
					if(nextMaxCount && ~currMaxCount) reducePriority = 1'b1;
					else if(~currMinWait && nextGreaterCount) reducePriority = 1'b1;
				end
			end
			if (reducePriority) begin
				clr_out[iterationNumber] = clr_in[iterationNumber-1];
				clr_out[iterationNumber-1] = clr_in[iterationNumber];
				sel_out[iterationNumber] = sel_in[iterationNumber-1];
				sel_out[iterationNumber-1] = sel_in[iterationNumber];
				count_out[iterationNumber] = count_in[iterationNumber-1];
				count_out[iterationNumber-1] = count_in[iterationNumber];
				waitTime_out[iterationNumber] = waitTime_in[iterationNumber-1];
				waitTime_out[iterationNumber-1] = waitTime_in[iterationNumber];
			end
		end
endmodule

module router_output_source_selector(input target_map [3:0] input_target, input [3:0][2:0] waitTime, 
												 input [3:0][5:0] count, input sr_out_rdy,
												 output target_t output_source);
	parameter OUTPUTID = 0;
	
	router_port output_value;

	always_comb begin
		
		case(OUTPUTID)
			0: output_value = ZERO;
			1: output_value = ONE;
			2: output_value = TWO;
			3: output_value = THREE;
			default: output_value = NONE;
		endcase
	
		if(sr_out_rdy)begin
			if(input_target[0].tar_out == output_value) output_source = input_target[0].tar_in;
			else if (input_target[1].tar_out == output_value) output_source = input_target[1].tar_in;
			else if (input_target[2].tar_out == output_value) output_source = input_target[2].tar_in;
			else if (input_target[3].tar_out == output_value) output_source = input_target[3].tar_in;
			else output_source = NONE;
		end
		else output_source = NONE;
	end							 
endmodule


module router_input_target_selector(input sr_in_rdy,
												input[31:0] data_in,
												output target_t target);
	parameter ROUTERID = 0, SELID = 0;
	bit[3:0] dest;
	assign dest = data_in[27:24];

	always_comb begin
		
		if(sr_in_rdy)
			case(dest)
				4'd0: target.tar = (ROUTERID == 0)? ZERO: THREE;
				4'd1: target.tar = (ROUTERID == 0)? TWO: THREE;
				4'd2: target.tar = (ROUTERID == 0)? THREE: THREE;
				4'd3: target.tar = (ROUTERID == 0)? ONE: ZERO;
				4'd4: target.tar = (ROUTERID == 0)? ONE: ONE;
				4'd5: target.tar = (ROUTERID == 0)? ONE: TWO;
				default: target.tar = NONE; 
			endcase
		else target.tar = NONE;

	end
endmodule

module shiftreg_in(input clk, rst_b, clr, shift_en,
						 input [7:0] data_in,
						 output logic full, free_inbound,
						 output logic [5:0] count, 
						 output logic [2:0] waitTime, 
						 output bit [31:0] data_out);
	parameter SR_IN_ID = 0;
	typedef enum logic [2:0]{EMPTY = 3'd0, SHIFT0 = 3'd1, SHIFT1 = 3'd2, SHIFT2 = 3'd3, SHIFT3 = 3'd4,  FULL = 3'd5} sr_in_t;
	
	const bit [5:0] count_MAX = 6'b111111;
	const bit [2:0] waitTime_MAX = 3'b111;
	
	sr_in_t state, nextState;
	bit [31:0] data, nextData;
	bit [5:0]nextCount;
	logic empty, nextFull, nextEmpty;
	bit [2:0] nextWaitTime;
	
	always_comb
	begin
		nextData = {data[23:0], data_in};
		nextCount = 0;
		nextWaitTime= waitTime;
		nextFull = 0;
		nextEmpty = 0;
		nextState = EMPTY;
		free_inbound = empty;
		
		case(state)
			EMPTY: begin
				if (shift_en) begin
					nextState = SHIFT0;
					//nextData = data;
					nextEmpty = 0;			
				end
				else begin
					nextState = EMPTY;
					nextWaitTime = (waitTime < waitTime_MAX)? waitTime+ 3'd1: waitTime;
					nextEmpty = 1;
					nextData = data;
					end
				end
			SHIFT0: begin
				nextState = SHIFT1;
			end
			SHIFT1: begin
				//nextState = SHIFT2; 
				nextState = SHIFT3;
			end
			SHIFT2: begin
				nextState = SHIFT3;
			end
			SHIFT3: begin
			nextFull = 1'b1;
			nextWaitTime = 3'd0;
			nextCount = count+ 6'd1;
			nextState = FULL;
			end
			FULL: begin 
				if (clr && shift_en) begin
					nextFull = 1'b0;
					nextEmpty = 1'b0;
					nextState = SHIFT0;
					nextData = {24'd0, data_in};
					nextWaitTime = 3'd0;
					nextCount = 6'd0;
					free_inbound = 1'b0;
				end
				else if(clr) begin
					nextFull = 1'b0;
					nextEmpty = 1'b1;
					nextState = EMPTY; 
					nextData = 32'd0;
					nextWaitTime = 3'd0;
					nextCount = (count < count_MAX)? count+ 6'd1 : count;
					end
				else begin
					nextFull = full;
					nextEmpty = empty;
					nextState = state;
					nextData = data;
					end
				end
				default: nextEmpty = 1;
		endcase
	end
	
	 
	always_ff @(posedge clk, negedge rst_b) 
	begin
		if (~rst_b) begin state <= EMPTY; data <= 1'b0; empty <= 1'b1; full<= 1'b0; waitTime <= waitTime_MAX; count <= 6'd0; end
		else begin 
			if (clr) begin state <= EMPTY; data <= 1'b0; empty <= 1'b1; full <= 1'b0; waitTime <= nextWaitTime; count<= 6'd0; end
			else begin state <= nextState; data<= nextData; full <= nextFull; empty<= nextEmpty;	waitTime <= nextWaitTime; count<= nextCount; end
		end
	end	
		
	assign data_out = data;
	
endmodule

module shiftreg_out(input clk, rst_b, load, shift_en,	//Parallel In, Serial Out
						 input [31:0] data_in,
						 output logic put_outbound, empty, 
						 output logic [7:0] data_out);
	parameter SR_OUT_ID = 0;
	typedef enum logic[2:0] {EMPTY = 3'd0, FULL = 3'd1 ,SHIFT0 = 3'd2, SHIFT1 = 3'd3, SHIFT2 = 3'd4} sr_t;
	
	
	sr_t state, nextState;
	bit [31:0] data, nextData;
	logic full, nextFull, nextEmpty;
	
	
	always_comb
	begin
		nextData = {data[23:0], 8'd0};
		nextFull = 1'b0;
		nextEmpty = 1'b0;
		nextState = EMPTY; 
		data_out = data[31:24];
		put_outbound = full;
		case(state)
			EMPTY: begin
				if (load && shift_en) begin
					nextData = {data_in[23:0], 8'd0};
					data_out = data_in[31:24];
					nextState = SHIFT0;
					put_outbound = 1'b1;
				end
				else if (load) begin
					nextData = data_in;
					nextFull = 1'b1; 
					nextState = FULL; 
				end
				else begin
					nextData = 32'd0; 
					nextFull = 1'b0;
					nextState = EMPTY;
					nextEmpty = 1'b1;
				end
			end
			FULL: begin 
				if (shift_en) begin 
					nextState = SHIFT0;
					nextFull = 1'b0;
				end
				else begin
					nextState = FULL; 
					nextFull = 1'b1;
					nextData = data;
				end
			end
			SHIFT0: begin
				nextState = SHIFT1;
			end
			SHIFT1: begin
				nextState = SHIFT2;
			end
			SHIFT2: begin
				if (load) begin
					nextData = data_in;
					nextState = FULL; 
					nextFull = 1'b1;
					nextEmpty = 1'b0;
				end
				else begin
					nextData = 32'd0;
					nextState = EMPTY;
					nextFull = 1'b0;
					nextEmpty = 1'b1;
				end
			end

				default: nextEmpty = 1;
		endcase
	end
	 
	always_ff @(posedge clk, negedge rst_b) 
	begin
		if (~rst_b) begin state <= EMPTY; data <= 1'b0; empty <= 1'b1; full<= 1'b0; end
		else begin state <= nextState; data<= nextData; full <= nextFull; empty<= nextEmpty; end
	end	
		
	
endmodule
