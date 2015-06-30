/* Author: Farjad Zaim (fzaim)
 * A node communicates serially with external modules or the testbench, 
 * and receives and transmits payload packets to and from the router.
*/

module node(clk, rst_b, pkt_in, pkt_in_avail, cQ_full, pkt_out, pkt_out_avail,
            free_outbound, put_outbound, payload_outbound,
            free_inbound, put_inbound, payload_inbound);

  parameter NODEID = 0;
  input clk, rst_b;

  // Interface to TestBench
  input pkt_t pkt_in;
  input pkt_in_avail;
  output cQ_full;
  output pkt_t pkt_out;
  output logic pkt_out_avail;

  // Endpoint -> Router transaction
  input free_outbound; // Router -> Endpoint
  output logic put_outbound; // Endpoint -> Router
  output [7:0] payload_outbound;

  // Router -> Endpoint transaction
  output reg free_inbound; // Endpoint -> Router
  input put_inbound; // Router -> Endpoint
  input [7:0] payload_inbound;
  
  logic fifo_re, fifo_empty;
  logic [31:0] fifo_out;
  
  logic [31:0] sr_out_data, sr_out_nextData, sr_in_data, sr_in_nextData;
  logic next_out_avail, next_free_inbound;
  logic next_sr_out_empty, sr_out_empty;
  logic sr_out_bypass;
  
  
  typedef enum logic [2:0]{shift0 =3'd0, shift1 =3'd1, shift2 =3'd2, shift3 =3'd3, idle =3'd4} sr_t;
  sr_t sr_out_state, sr_out_nextState, sr_in_state, sr_in_nextState;
  
  fifo queue(clk, rst_b, pkt_in, pkt_in_avail, fifo_re, cQ_full, fifo_empty, fifo_out, sr_out_bypass);
  
  
  
  always_comb //Output SR Controller
  begin
		next_sr_out_empty = 0; sr_out_bypass = 0; fifo_re = 0; put_outbound = 1'b1;
		case(sr_out_state)
			shift0: begin sr_out_nextState = shift1; sr_out_nextData = {sr_out_data[23:0], 8'd0}; end  			//output 31:24
			shift1: begin sr_out_nextState = shift2; sr_out_nextData = {sr_out_data[23:0], 8'd0}; end 			//output 23:16
			shift2: begin sr_out_nextState = shift3; sr_out_nextData = {sr_out_data[23:0], 8'd0}; end 			//output 15:8
			shift3: 
			begin																											//output 7:0
				if (~fifo_empty) begin sr_out_nextState = (free_outbound)? shift0 : idle; sr_out_nextData = fifo_out; fifo_re = 1;  end
				else if (pkt_in_avail)begin sr_out_nextState = (free_outbound)? shift0 : idle; sr_out_nextData = pkt_in; sr_out_bypass = 1; end
				else begin next_sr_out_empty = 1; sr_out_nextState = idle; sr_out_nextData =  32'd0; end
			end
			idle: 
			begin
				put_outbound = 1'b0;
				if (sr_out_empty) 
					if(pkt_in_avail) begin sr_out_nextState = (free_outbound)? shift0 : idle; sr_out_nextData = pkt_in; next_sr_out_empty = 0; sr_out_bypass = 1; end 
					else begin sr_out_nextState = idle; sr_out_nextData = sr_out_data; next_sr_out_empty = 1; end
				else begin sr_out_nextState = (free_outbound)? shift0 : idle; sr_out_nextData = sr_out_data; next_sr_out_empty = 0; end
			end
			default: begin sr_out_nextState = idle; sr_out_nextData = 0; put_outbound = 1'b0; end
		endcase
		
  end
  
  always_ff @(posedge clk, negedge rst_b) //Output SR Controller
  begin
		if (~rst_b) begin sr_out_state <= idle; sr_out_data <= 0; sr_out_empty <= 1'b1; end
		else begin sr_out_state <= sr_out_nextState; sr_out_data <= sr_out_nextData; sr_out_empty <= next_sr_out_empty;	end
  end
 
  
  
    always_comb //Input SR Controller
  begin
		sr_in_nextData = {sr_in_data[23:0], payload_inbound};
		next_out_avail = 0;
		next_free_inbound = 0;
		case(sr_in_state)
			shift0: begin sr_in_nextState = shift1;  	end  			//input 31:24
			shift1: begin sr_in_nextState = shift2;	end 			//input 23:16
			//shift2: begin sr_in_nextState = idle; next_free_inbound = 1'b1; 	end 			//input 15:8
			shift2: begin sr_in_nextState = (put_inbound)? shift0: idle; next_out_avail = 1'b1; 
							  next_free_inbound = (put_inbound)? 1'b0: 1'b1;			end			//input 7:0
			idle:   begin sr_in_nextState = (put_inbound)? shift0: idle; next_out_avail = (put_inbound)?pkt_out_avail: 1'b0; 
							  next_free_inbound = (put_inbound)? 1'b0: 1'b1; 
							 // sr_in_nextData = (put_inbound)?sr_in_data: {23'd0, payload_inbound};	
							 end
			default: begin  sr_in_nextState = idle; end
		endcase
  end
  

  
  always_ff @(posedge clk, negedge rst_b) //Input SR Controller
  begin
		if (~rst_b) begin sr_in_state <= idle; sr_in_data <= 0; pkt_out_avail <= 0; free_inbound <= 1; end
		else begin sr_in_state <= sr_in_nextState; sr_in_data <= sr_in_nextData; 
					  pkt_out_avail <= next_out_avail; free_inbound <= next_free_inbound; end
  end
  
  
  //assign put_outbound = (sr_out_state ==4)? 1'b0: 1'b1;
  assign pkt_out.sourceID = sr_in_data[31:28];
  assign pkt_out.destID = sr_in_data[27:24];
  assign pkt_out.data = sr_in_data[23:0];
  assign payload_outbound = sr_out_data[31:24];
endmodule





/*  Create a fifo (First In First Out) with depth 4 using the given interface and constraints.
 *  -The fifo is initally empty.
 *  -Reads are combinational
 *  -Writes are processed on the clock edge.  
 *  -If the "we" happens to be asserted while the fifo is full, do NOT update the fifo.
 *  -Similarly, if the "re" is asserted while the fifo is empty, do NOT update the fifo. 
 */
 
 //Assert empty = fifo_empty[0], full = ~fifo_empty[3]
 //Assert re != empty
 //Todo : Bypass FIFO if empty
module fifo(clk, rst_b, data_in, we, re, full, empty, data_out, sr_bypass);
  parameter WIDTH = 32;
  input clk, rst_b;
  input [WIDTH-1:0] data_in;
  input we; //write enable
  input re; //read enable
  output full;
  output empty;
  output [WIDTH-1:0] data_out;
  input sr_bypass;
  
  
  reg[3:0][31:0] fifo_data;
  reg[3:0] fifo_empty;

  fifo_reg reg3 (clk, rst_b, we, re, empty, full, fifo_empty[2], 	1'b1,				data_in, 32'd0,			fifo_data[3], fifo_empty[3]);
  fifo_reg reg2 (clk, rst_b, we, re, empty, full, fifo_empty[1], 	fifo_empty[3], data_in, fifo_data[3], 	fifo_data[2], fifo_empty[2]);
  fifo_reg reg1 (clk, rst_b, we, re, empty, full, fifo_empty[0], 	fifo_empty[2], data_in, fifo_data[2], 	fifo_data[1], fifo_empty[1]);
  fifo_reg reg0 (clk, rst_b, we, re, empty, full, sr_bypass,		fifo_empty[1], data_in, fifo_data[1], 	fifo_data[0], fifo_empty[0]);
	
	assign data_out = fifo_data[0];
	assign empty = fifo_empty[0] && fifo_empty[1] && fifo_empty[2] && fifo_empty[3]; //Check &&, can assert that empty == empty[0]
	assign full = ~fifo_empty[0] && ~fifo_empty[1] && ~fifo_empty[2] && ~fifo_empty[3]; //Can assert that full == full[3]
endmodule

module fifo_reg (input clk, rst_b, we, re, empty, full, bypass, fifo_empty_prev,
						input[31:0] data_in, fifo_data_prev,
						output reg[31:0] fifo_data,
						output reg fifo_empty);
	//Assert fifo_empty can only be 0 if bypass is 0
	always_ff @(posedge clk, negedge rst_b)
	begin
		if(~rst_b) begin fifo_data <= 0; fifo_empty <=1; end
		else begin
			if(re && we && ~empty && ~full)
			begin
				//fifo_data <= (bypass && ~fifo_empty)? data_in: fifo_data_prev ; 
				if (fifo_empty) fifo_data <= 32'd0;
				else fifo_data <= (fifo_empty_prev)? data_in : fifo_data_prev;
				fifo_empty <= fifo_empty;
			end
			else if (re && ~empty)
			begin
				fifo_data <= fifo_data_prev; 
				fifo_empty <= fifo_empty_prev;
			end
			else if (we && ~full && ~bypass)
			begin
				fifo_data <= (fifo_empty)? data_in : fifo_data; 
				fifo_empty <= 0;
			end
			else begin fifo_data <= fifo_data; fifo_empty <= fifo_empty; end
		end
	end
	endmodule
