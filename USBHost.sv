/* Authors: Farjad Zaim (fzaim),
 *			Varsha Shetty (vshetty)
*/

//Used to transfer all necessary information between shift registers and protocol FSM
typedef struct packed {
  logic [7:0] sync;
  logic [7:0] PID;
  logic [6:0] addr;
  logic [3:0] endp;
  logic [63:0] data;
  logic [15:0] CRC16;
  logic [4:0] CRC5;
} pkt_t;
typedef enum logic [0:0] {CRC5 = 1'b0, CRC16 = 1'b1} crc_t;
typedef enum logic [2:0]{SYNC = 3'd0, PID = 3'd1, DATA = 3'd2, CRC = 3'd3, NONE = 3'd4} shift_type_t; //set to DATA for ADDR and ENDP
const logic [7:0] PID_OUT = 8'b10000111;
const logic [7:0] PID_IN = 8'b10010110;
const logic [7:0] PID_DATA = 8'b11000011;
const logic [7:0] PID_ACK = 8'b01001011;
const logic [7:0] PID_NAK = 8'b01011010;
const logic [7:0] SYNC_VAL = 8'b 00000001;
//Keeps track of shift progress and sends control signals to crc and outputConverter
module bitstreamEncoding(input logic clk, rst_L, load, stop, 
								 input pkt_t pkt_in, input logic crc5_in,crc16_in,
								 output logic shift_out,shift_en,crc5_en,crc16_en, output shift_type_t shift_type);
	const logic [6:0] last_pid_count = 7'd15;


	logic [6:0] count, nextCount;
	logic [7:0] last_data_count, last_crc_count, last_eop_count;
	logic [79:0] streamData, nextStreamData, pkt_to_stream;
	logic error_PID,crc5,crc16;
	typedef enum logic [1:0]{IDLE = 2'd0, SHIFT = 2'd1, SHIFT_CRC = 2'd3, WAIT = 2'd2} state_t;
	state_t state, nextState; 
	
	assign last_eop_count = last_crc_count +3;
	always_comb begin
		nextStreamData = streamData;
		shift_out = streamData[79];
		nextState = IDLE;
		shift_en = 1'b0;
		nextCount = count+1;
		crc5_en = 1'b0;
		crc16_en=1'b0;
		
		error_PID = 1'b0;
		case(pkt_in.PID)
			PID_OUT: begin pkt_to_stream = {pkt_in.sync, pkt_in.PID, pkt_in.addr, pkt_in.endp, 53'd0}; 
							last_data_count = 7'd26;
							last_crc_count = 7'd31;
							crc5=1'b1;
							crc16=1'b0;
						end//Token - OUT 
			PID_IN: 	begin pkt_to_stream = {pkt_in.sync, pkt_in.PID, pkt_in.addr, pkt_in.endp, 53'd0}; 
							last_data_count = 7'd26;
							last_crc_count = 7'd31;
							crc5=1'b1;
							crc16=1'b0;
						end//Token - IN
			PID_DATA:begin pkt_to_stream = {pkt_in.sync, pkt_in.PID, pkt_in.data}; 
							last_data_count = 7'd79;
							last_crc_count = 7'd95;
							crc16=1'b1;
							crc5=1'b0;
						end//Data
			PID_ACK: begin pkt_to_stream = {pkt_in.sync, pkt_in.PID, 64'd0}; 
							last_data_count = last_pid_count; 
							last_crc_count = last_pid_count;
						end//ACK
			PID_NAK: begin pkt_to_stream = {pkt_in.sync, pkt_in.PID, 64'd0}; 
							last_data_count = last_pid_count; 
							last_crc_count = last_pid_count;
						end//NAK
			default: begin pkt_to_stream = 80'd0; 
							error_PID = 1'b1; 
							last_data_count = 7'd0; 
							last_crc_count = 7'd0;
						end
		endcase
		case(state)
			IDLE: begin 
						if(load) begin nextCount = count+1; 
											nextState = SHIFT; 
											nextStreamData = {pkt_to_stream[78:0], 1'b0};
											shift_en = 1'b1;
											shift_out = pkt_to_stream[79];
											//if data packet, set crc enable
									end
						else begin nextState = IDLE;
										nextCount = 7'd0;
							end
					end
			SHIFT:begin 
						nextState = SHIFT;
						shift_en = 1'b1;
						nextStreamData = {streamData[78:0], 1'b0};
						nextCount = count+1;
						if (stop) begin
							nextCount = count;
							nextStreamData = streamData;
						end
						else if (count == last_pid_count && (pkt_in.PID == PID_ACK || pkt_in.PID == PID_NAK))
							nextState = WAIT;
						else if (count == last_pid_count+1) begin //16
								crc5_en = (crc5)? 1'b1:1'b0;
								crc16_en= (crc16)?1'b1:1'b0;
								end
						else if(count == last_data_count) //26
							nextState = SHIFT_CRC;
					end
			SHIFT_CRC: 	begin
								nextState = SHIFT_CRC;
								shift_en = 1'b1;
								crc5_en = (crc5)? 1'b1:1'b0;
								crc16_en= (crc16)?1'b1:1'b0;
								nextStreamData = streamData;
								nextCount = count+1;
								if (crc5) begin
								shift_out = crc5_in;
								end
								else if (crc16) begin
								shift_out=crc16_in;
								end
								if (stop) 
									nextCount = count;
								else if (count == last_crc_count) //31
									begin
									nextState = WAIT;
									nextCount = count;
									end
								else begin
									nextState = SHIFT_CRC;
									shift_en = 1'b1;
								end
							end
			WAIT: begin
						nextState = WAIT;
						nextCount = count+1;
						shift_en = 1'b0;
						if(count == last_eop_count-1) begin //34
							nextState = IDLE;
							nextCount = 7'd0; 
							end
					 end
			default: ;
			endcase
			
	end
	
	
	always_ff @(posedge clk, negedge rst_L)begin
		if (~rst_L) begin
			count<= 7'd0;
			streamData <= 80'd0;
			state <= IDLE;
		end
		else begin
			count <= nextCount;
			state <= nextState;
			streamData <= nextStreamData;
		end
	end
	always_comb begin
		if (count <= 7) shift_type = SYNC;
		else if (count <= last_pid_count) shift_type = PID;
		else if (count <= last_data_count) shift_type = DATA;
		else if (count <= last_crc_count) shift_type = CRC;
		else shift_type = NONE;
	end
 
  
endmodule
  
  //Inserts a 0 after 6 consecutive 1's in shift_in when shift_en is asserted
module bitStuffer (input logic clk, rst_L, shift_in, shift_en,
						 output logic shift_out, stop);
	
	logic [2:0] count, nextCount;
	
	always_comb begin
		stop = 1'b0;
		shift_out = shift_in;
		nextCount = 3'd0;
		
		if (shift_en) begin
			if (count == 6) begin
					shift_out = 1'b0;
					stop = 1'b1;
			end
			else if (shift_in)begin
				nextCount = count+1;
			end
		end
	end
	
	always_ff @(posedge clk, negedge rst_L) begin
		if (~rst_L) begin
			count <= 3'd0;
		end
		else begin
			count <= nextCount;
		end
	end
	
endmodule

//Performs NRZI and JK Conversion
module outputConverter (input logic clk, rst_L, shift_in, shift_en,
								usbWires wires, 
								output logic wires_en, done);
	logic nrzi, nrzi_prev;
	logic nextDone;
	logic DM, DP;
	typedef enum logic [1:0]{ZERO = 2'd0, ONE = 2'd1, SE0 = 2'd2, NONE = 2'd3} out_t;
	typedef enum logic [1:0]{IDLE = 2'd0, SHIFT = 2'd1, EOP1 = 2'd2, EOP2 = 2'd3} state_t;
	out_t wiresVal;
	state_t state, nextState;
	
	assign wires.DP = wires_en? DP: 1'bz;
	assign wires.DM = wires_en? DM: 1'bz;
	
	always_comb begin
		case (wiresVal)
			ZERO: begin DP = 1'b0;
							DM = 1'b1; end
			ONE: begin	DP = 1'b1;
							DM = 1'b0; end
			SE0: begin 	DP = 1'b0; 
							DM = 1'b0; end
			NONE: begin DP = 1'bz;
							DM = 1'bz; end
			default: begin DP = 1'bz; DM = 1'bz; end
		endcase
	end
	always_comb begin
		case(state)
			IDLE: 	wiresVal = (shift_en)? ZERO: NONE;
			SHIFT: 	wiresVal = (shift_en)? ((nrzi)? ONE: ZERO):  SE0;
			EOP1: 							wiresVal = SE0;
			EOP2: 							wiresVal = ONE;
			default: wiresVal = NONE;
		endcase
	end
	always_comb begin
		nrzi = 1'b1;	
		wires_en = shift_en;
		nextDone = 1'b0;
		case(state)

			IDLE: 	begin
							if (shift_en) begin
								nrzi = 1'b0;
								nextState = SHIFT;
							end
							else begin
								nextState = IDLE;
							end
						end
			SHIFT: 	begin
							if(shift_en) begin
								nrzi = (shift_in)? nrzi_prev: ~nrzi_prev ;
								nextState = SHIFT;
							end
							else begin
								wires_en = 1'b1;
								nextState = EOP1;
							end
						end
			EOP1:    begin
							wires_en = 1'b1;
							nextState = EOP2;
						end
			EOP2: 	begin
							wires_en = 1'b1;
							nextState = IDLE;
							nextDone = 1'b1;
						end
			default: begin nextState = IDLE; wires_en = 1'b0; nextDone = 1'b0; end
		endcase
	end
	
	always_ff @(posedge clk, negedge rst_L) begin
		if (~rst_L) begin
			nrzi_prev <= 1'b1;
			state <= IDLE;
			done <= 1'b0;
		end
		else begin
			nrzi_prev <= nrzi;
			state <= nextState;
			done <= nextDone;
		end
	end
endmodule

//Breaks transaction down into address transfer and data transfer. Communicates the success of each transfer with protocol_fsm
//Interfaces with usbHost tasks to communicate transaction completion and success, as well as associated data values
module rw_fsm(
				  //usbHost
				  input logic clk, rst_L, start, rw, //rw = 1 for write
				  input logic [63:0] data_in,
				  input logic [15:0] mempage,
				  output logic [63:0] data_out,
				  output logic prevDone, success,
				  //protocol_fsm
				  input logic transaction_done, transaction_success, 
				  output logic transaction_inout, transaction_start, //transaction_inout = 1 for out
				  output logic [3:0] transaction_endp,
				  input logic [63:0] transaction_data_out,
				  output logic [63:0] transaction_data_in);	  
	typedef enum logic [2:0]{IDLE = 3'd0, ADDR_OUT = 3'd1, DATA = 3'd2, NONE = 3'd3} state_t;
	state_t state, nextState;
	logic prevSuccess, done, prevTransaction_inout;
	logic [63:0] prevData_out, prevTransaction_data_in;
	logic nextTransaction_start;
	
	always_comb begin
		nextTransaction_start = 1'b0; data_out = 64'd0; success = prevSuccess;
		transaction_data_in = 64'd0; done = prevDone; transaction_inout = prevTransaction_inout;
		transaction_endp = 4'd0; 
		case(state)
			IDLE: begin 
				nextState = IDLE;
				data_out = (prevDone)? transaction_data_out: prevData_out;
				done=  1'b0;
				if (start) begin
					nextState = ADDR_OUT;
					nextTransaction_start = 1'b1;
					//transaction_data_in = {48'd0, mempage};
					
					transaction_data_in = {{<<{mempage}}, 48'd0};
					transaction_inout = 1'b1;
					transaction_endp = 4'd4;
					success = 1'b1;
				end	
			end
			
			ADDR_OUT: begin
				nextState = ADDR_OUT;
				transaction_data_in = prevTransaction_data_in;
				transaction_endp = 4'd4;
				if(transaction_done) begin
					if (transaction_success) begin
						nextState = DATA;
						nextTransaction_start = 1'b1;
						transaction_data_in = data_in;
						transaction_inout = rw;
						transaction_endp = 4'd8;
					end
					else begin
						nextState = IDLE;
						done = 1'b1;
						success = 1'b0;
						transaction_data_in = 64'b0;
					end
				end	
			end
			DATA: begin
				nextState = DATA;
				transaction_data_in = prevTransaction_data_in;
				transaction_endp = 4'd8;
				if(transaction_done) begin
					nextState = IDLE;
					done = 1'b1;
					success = transaction_success;
					data_out = transaction_data_out; //0's at this point
					transaction_endp = 4'd0;
				end
			end
		endcase
	end
	always_ff @(posedge clk, negedge rst_L) begin
		if (~rst_L) begin
			prevSuccess <= 1'b1;
			prevDone <= 1'b0;
			prevTransaction_inout <= 1'b0;
			prevData_out <= 64'd0;
			prevTransaction_data_in <= 64'd0;
			state <= IDLE;
			transaction_start <= 1'b0;
		end
		else begin
			prevSuccess <= success;
			prevDone <= done;
			prevTransaction_inout <= transaction_inout;
			prevTransaction_data_in <= transaction_data_in;
			prevData_out <= data_out;
			state <= nextState;
			transaction_start <= nextTransaction_start;
		end
	end
				  
				  
endmodule
				  
//Receives data and address transfer information from the rw_fsm, and controls the packet_shift_in and packet_shift_out modules with packet-level communications
//For each packet, the protocol_fsm enables the appropriate shift module
//With the shift_in module, the protocol_fsm receives completion and success signals, and transfers valid data packets to the rw_fsm
//Protocol_fsm also handles all packet-level error checking
module protocol_fsm	(
							 //rw_fsm
							 input logic clk, rst_L, start, transaction_inout,
							 input  logic [3:0] endp_in,
							 input logic [63:0] data_in,
							 output logic [63:0] data_out,
							 output logic done, success,
							 
							 output logic rw, //rw = 1 for write
							 //shift_in
							 input pkt_t pkt_in, 
							 input logic pkt_in_rdy,		
							 input logic pkt_in_success,
							 output logic sr_in_clr,
							 //shift_out
							 input logic sr_out_done,
							 output pkt_t pkt_out,
							 output logic pkt_out_load,
							 output crc_t crc_type);
	typedef enum logic [2:0]{IDLE = 3'd0, TOKEN = 3'd1, DATA_IN = 3'd2, DATA_OUT = 3'd3,
									 ACK_IN = 3'd4, ACK_OUT = 3'd5, NONE = 3'd6} state_t;
	state_t state, nextState;
	
	logic prevSuccess;
	logic [7:0] count_timeout, nextCount_timeout;
	logic [3:0] count_NAK,nextCount_NAK;
	logic [63:0]prevData_out;
	pkt_t prevPkt_out;
	
	always_comb begin
		done = 1'b0;
		success = 1'b1;
		pkt_out = '0;
		pkt_out_load = 1'b0;
		rw = 1'b1;
		data_out = 64'd0;
		nextState = IDLE;
		nextCount_NAK = count_NAK;
		nextCount_timeout = '0;
		sr_in_clr = 1'b0;
		crc_type = CRC5;
		case(state)
			IDLE: begin
				nextState = IDLE;
				nextCount_NAK = '0;
				success = prevSuccess;
				data_out = prevData_out;
				if (start) begin
					nextState = TOKEN;
					pkt_out.PID = (transaction_inout)? PID_OUT : PID_IN;
					pkt_out.sync = SYNC_VAL;
					pkt_out.endp = {<<{endp_in}};
					pkt_out.addr = {<<{7'd5}};
					pkt_out.data = 64'd0;
					pkt_out_load = 1'b1;
				   crc_type = CRC5;
				end
			end
			TOKEN: begin
				crc_type = CRC5;
				nextState = TOKEN;
				pkt_out = prevPkt_out;
				if (sr_out_done) begin
					if (transaction_inout) begin
						nextState = DATA_OUT;
						pkt_out.PID = PID_DATA;
						pkt_out.sync = SYNC_VAL;
						pkt_out.endp = 4'd0;
						pkt_out.data = {<<{data_in}};
						pkt_out_load = 1'b1;
						crc_type = CRC16;
					end
					else begin
						nextState = DATA_IN;
						pkt_out = '0;
						rw = 1'b0;
					end
				end
			end
			DATA_IN: begin
				nextState = DATA_IN;
				rw = 1'b0;
				nextCount_timeout = count_timeout + 1;
				if (pkt_in_rdy) begin
					sr_in_clr = 1'b1;
					nextCount_timeout = 8'd0;
					if (pkt_in_success) begin
						data_out = {<<{pkt_in.data}};
						nextState = ACK_OUT;
						rw = 1'b1;
						pkt_out.sync = SYNC_VAL;
						pkt_out.PID = PID_ACK; //Need logic for NAK for corrupted data
						pkt_out_load = 1'b1;
					end
					else begin
						nextState = ACK_OUT; //Do this for all ~pkt_in_success
						rw = 1'b1;
						pkt_out.sync = SYNC_VAL;
						pkt_out.PID = PID_NAK;
						pkt_out_load = 1'b1;
					end
				end
				else begin
					if (count_timeout == 8'd255) begin //off by one?
						nextCount_timeout = 8'd0;
						nextState = ACK_OUT;
						rw = 1'b1;
						pkt_out.sync = SYNC_VAL;
						pkt_out.PID = PID_NAK;
						pkt_out_load = 1'b1;
					end
				end
			end
			DATA_OUT: begin
				nextState = DATA_OUT;
				pkt_out = prevPkt_out;
				crc_type = CRC16;
				if (sr_out_done) begin
					nextState = ACK_IN;
					rw = 1'b0;
				end
			end
			ACK_IN: begin
				rw = 1'b0;
				nextState = ACK_IN;
				if (pkt_in_rdy) begin
					sr_in_clr = 1'b1;
					if (pkt_in_success) begin
						if (pkt_in.PID == PID_ACK) begin
							done = 1'b1;
							success = 1'b1;
							nextCount_NAK = 4'd0;
							nextState =IDLE;
						end
						else if (pkt_in.PID == PID_NAK) begin
							if (count_NAK <= 4'd8) begin //7 or 8?
								/*nextState =IDLE;
								done = 1'b1;
								success = 1'b1;
								nextCount_NAK = 4'd0;*/
								nextState = DATA_OUT;
								done = 1'b0;
								success = 1'b1;
								rw = 1'b1;
								nextCount_NAK = count_NAK+1;
								pkt_out.sync = SYNC_VAL;
								pkt_out.data = {<<{data_in}};
								pkt_out_load = 1'b1;
								pkt_out.PID = PID_DATA;
							end
							else begin
								nextCount_NAK = '0;
								nextState = IDLE; 
								done = 1'b1;
								success = 1'b0;
							end
						end
					end
					else begin	
						nextState = ACK_OUT; //Do this for all ~pkt_in_success
						rw = 1'b1;
						pkt_out.sync = SYNC_VAL;
						pkt_out.PID = PID_NAK;
						pkt_out_load = 1'b1;
						/*nextState = IDLE;
						done = 1'b1;
						success = 1'b0;
						nextCount_NAK = 4'd0;*/
					end
				end
			end
			ACK_OUT: begin
				nextState = ACK_OUT;
				pkt_out = prevPkt_out;
				if (sr_out_done) begin
					if (pkt_out.PID == PID_ACK) begin
						nextState = IDLE;
						done = 1'b1;
						success = 1'b1;
						nextCount_NAK = 4'd0;
					end
					else begin
						if (count_NAK <= 4'd8) begin //7 or 8?
							nextState = DATA_IN;
							rw = 1'b0;
							pkt_out = '0;
							nextCount_NAK = count_NAK+1;
						end
						else begin
							nextState = IDLE;
							done = 1'b1;
							success = 1'b0;
							nextCount_NAK = 4'd0;
						end
					end
				end
			end
			default: ;
		endcase
	end
	always_ff @(posedge clk, negedge rst_L) begin
		if (~rst_L) begin
			
		end
		else begin
			count_NAK <= nextCount_NAK;
			count_timeout <= nextCount_timeout;
			prevSuccess <= success;
			prevPkt_out <= pkt_out;
			prevData_out <= (pkt_in_rdy)? data_out: prevData_out;
			state <= nextState;
		end
	end
	
endmodule
//Receives packet-level communications from the protocol_fsm, and indicates completion of shift_out operations to the protocl_fsm
//Serializes the received packets and outputs them over the usbWires of usbHost when enabled by the protocol_fsm
module packet_shift_out(input logic clk, rst_L, load, input pkt_t pkt_in,
								usbWires wires, output logic done);

  
	logic stuffer_in, NRZI_in;
	logic shift_en, crc5_en,crc16_en,stuffer_en, NRZI_en;
	logic enc_out, crc_out,crc5_out,crc16_out, stuffer_out;
	logic stop, wires_en;
	logic debug_crc5_transmitting,debug_crc16_transmitting;
	shift_type_t shift_type;
	
	assign NRZI_en = shift_en;
	assign stuffer_in = enc_out;
  
	bitstreamEncoding enc(clk, rst_L, load, stop, pkt_in, crc5_out,crc16_out, enc_out, shift_en, crc5_en,crc16_en,shift_type);
	bitStuffer stuffer(clk, rst_L, stuffer_in, stuffer_en, stuffer_out, stop);
	outputConverter converter(clk, rst_L, NRZI_in, NRZI_en, wires, wires_en, done);
	crc5_check_encoding crc5(clk, rst_L, crc5_en, stop, {pkt_in.addr, pkt_in.endp}, crc5_out, debug_crc5_transmitting);
	crc16_check_encoding crc16(clk,rst_L, crc16_en, stop,{pkt_in.data},crc16_out,debug_crc16_transmitting);

	
	always_comb begin
		NRZI_in = 1'b0;
		stuffer_en = 1'b0;
		case(shift_type)
			SYNC:	begin
						stuffer_en = 1'b0;
						NRZI_in = enc_out;
					end
			PID:	begin
						stuffer_en = 1'b0;
						NRZI_in = enc_out;
					end
			DATA:	begin
						stuffer_en = 1'b1;
						NRZI_in = stuffer_out;
					end
			CRC:	begin 
						stuffer_en = 1'b1;
						NRZI_in = stuffer_out;
					end
			NONE:	;
			default: ;	
		endcase
	end
endmodule
//Receives serialized packets over usbWires, and transmits complete packets to the protocol_fsm, along with completion and success signals
//Performs bit-level error checking, including conformity of CRC values to expected CRC
 module packet_shift_in(input logic clk, rst_L, pkt_rw,usbWires wires, 
								output logic decode_done, decode_success, output pkt_t output_packet);
logic crc_in, shift_out_to_bitstreamdecoder, shift_out_to_bitunstuff,success_jk_nrzi, success_bitstreamdecoding,eop, internal_reset;		
inputconverter input_converter(clk, rst_L,pkt_rw, decode_done, eop, wires, shift_out_to_bitunstuff,success_jk_nrzi,sync_detected);
bitunstuffer unstuffer(clk, rst_L,shift_out_to_bitunstuff,pkt_rw,shift_out_to_bitstreamdecoder,reject);	
bitstreamdecoding bitstreamdecoder(clk,rst_L,shift_out_to_bitstreamdecoder, pkt_rw, reject, crc_in,crc_transmitting, sync_detected,decode_done,eop, 
												success_bitstreamdecoding,crc5_en,crc16_en,crc_out,output_packet);
success decoding_success(success_jk_nrzi,success_bitstreamdecoding,decode_done,decode_success);
crc_decoding crc_decoder (clk, rst_L, crc5_en, crc16_en, reject, crc_out, crc_transmitting, crc_in);


endmodule 

module usbHost
	(input logic clk, rst_L, 
	usbWires wires);
  	logic write_in_progress, read_in_progress;
	//RW-Top
	//RW input
	logic start, rw;
	logic [63:0] data_in;
	logic [15:0] mempage_in; 
	//RW Output
	logic done_outs, success_out;
	logic [63:0] data_out;
	
	//RW-Protocol 
	//RW Input
	logic transaction_done, transaction_success;
	logic [63:0] transaction_data_out; 
	//RW Output
	logic  transaction_inout, transaction_start;
	logic [3:0] transaction_endp;
	logic [63:0] transaction_data_in;
	
	//Protocol-SR_in
	logic pkt_rw, pkt_in_rdy, pkt_in_success, sr_in_clr;
	pkt_t pkt_in;
	
	
	//Protocol-SR_out
	logic sr_out_done, pkt_out_load;
	pkt_t pkt_out;
	crc_t crc_type;
	usbWires wires_out();
	
	//Prelab
	logic prelab, prelab_load, sr_out_load;
	pkt_t prelab_pkt_out, sr_out_pkt_out;
	
	assign wires.DM = (pkt_rw)? wires_out.DM : 1'bz;
	assign wires.DP = (pkt_rw)? wires_out.DP : 1'bz;
	assign sr_out_load = (prelab)? prelab_load : pkt_out_load;
	assign sr_out_pkt_out = (prelab)? prelab_pkt_out : pkt_out;
	
	
	rw_fsm fsm(clk, rst_L, start, rw, data_in, mempage_in, data_out, done_outs, success_out, transaction_done, transaction_success, 
				  transaction_inout, transaction_start, transaction_endp, transaction_data_out, transaction_data_in);
	protocol_fsm pfsm(clk, rst_L, transaction_start, transaction_inout, transaction_endp, transaction_data_in, transaction_data_out, 
					 transaction_done, transaction_success, pkt_rw, pkt_in, pkt_in_rdy, pkt_in_success, sr_in_clr, 
					 sr_out_done, pkt_out, pkt_out_load, crc_type);	
	packet_shift_out sr_out(clk, rst_L, sr_out_load, sr_out_pkt_out, wires_out, sr_out_done);
	packet_shift_in sr_in(clk, rst_L,~pkt_rw, wires,pkt_in_rdy, pkt_in_success, pkt_in);						  
	
 /*task tempRequest(input bit [7:0] data);
	pkt_in.sync = SYNC_VAL;
	//pkt_in.PID = 8'b11000011;
	//pkt_in.data = {<<{64'hA80000000AA05A5A}};
	pkt_in.PID = PID_ACK;
	load = 1'b1;
	wait(done);
 endtask: tempRequest*/
 
  /* Tasks needed to be finished to run testbenches */
  task prelabRequest
  // sends an OUT packet with ADDR=5 and ENDP=4
  // packet should have SYNC and EOP too
  (input bit  [7:0] data);
		logic load;
		prelab = 1'b1;
		prelab_pkt_out.sync <= SYNC_VAL;
		//prelab_pkt_out.PID <= PID_OUT;
		//prelab_pkt_out.addr <= {<<{7'd5}};//7'b1010000;
		//prelab_pkt_out.endp <= {<<{4'd4}};//4'b0010;
		
		prelab_pkt_out.PID <= PID_DATA;
		prelab_pkt_out.data <= {<<{64'h 1023456789ABCDEF}};
	   prelab_load <= 1'b1;
		@(posedge clk);
		//assert (~done);
		prelab_load <= 1'b0;
		wait (sr_out_done);
		prelab = 1'b0;
  endtask: prelabRequest 

  task readData
  // host sends memPage to thumb drive and then gets data back from it
  // then returns data and status to the caller
  (input  bit [15:0]  mempage, // Page to write
   output bit [63:0] data, // array of bytes to write
   output bit        success);
	$display("Start of ReadData");
		success <= 1'b0;
		@(posedge clk);
		read_in_progress <= 1'b1;
		prelab <= 1'b0;
		mempage_in <= mempage;
		data_in = '0;
		rw <= 1'b0;
		start <= 1'b1;
		@(posedge clk);
		start <= 1'b0;
		wait(done_outs); $display("End of ReadData");
		/*@(posedge clk);
		@(posedge clk);
		@(posedge clk);
		@(posedge clk);
		@(posedge clk);*/
		data = data_out;
		success = success_out;
		read_in_progress = 1'b0;
  endtask: readData

  
  //Needs to receive and send ACK/NAK
	task writeData
  // Host sends memPage to thumb drive and then sends data
  // then returns status to the caller
  (input  bit [15:0]  mempage, // Page to write
   input  bit [63:0] data, // array of bytes to write
   output bit        success);
		success <= 1'b0;
		@(posedge clk);
		$display("Start of WriteData");
		write_in_progress <= 1'b1;
		prelab <= 1'b0;
		mempage_in <= mempage;
		data_in = data;
		rw <= 1'b1;
		start <= 1'b1;
		@(posedge clk);
		start <= 1'b0;
		wait(done_outs); $display("End of WriteData: %d", $time); //begin
		data = data_out;
		success = success_out;
		write_in_progress = 1'b0;
		//end		
	endtask: writeData
	
  	
  // usbHost starts here!!


endmodule: usbHost


 module crc5_check_encoding
     #(parameter data_width = 11, crc_width=5)
  (input logic clk, reset_b, load, stop,
   input logic [10:0] input_packet,
   output logic crc,transmitting);


   typedef enum bit [1:0] {WAIT=2'd0,CALCULATE=2'd1, SHIFT=2'd2} states;
   states state,nextState;
   
   bit [6:0] 	count,nextcount;
   bit 		enable;
   bit [crc_width-1:0] 	crcCheckbits;
   

   always_comb begin
   case(state) 
      WAIT:if (load) begin
			  nextState = CALCULATE;
			  enable=1;	
					  if(enable && count<data_width)begin
					  nextState = CALCULATE;
					  nextcount=count+1; 
					  end
				
				end
				else  begin
					 nextState=WAIT;
					 enable=0;
					 nextcount=7'd0;
				end
      CALCULATE: 
				 if(enable && count<data_width)begin
					  nextState = CALCULATE;
					 nextcount=count+1; 
				 end
				 else if (count== data_width)begin
					  nextState = SHIFT;
					   nextcount=count+1; 
						crc = ~crcCheckbits[(data_width+crc_width-1)-count];
						//transmitting=1'b1;
					 end
      SHIFT:	  if (count<=data_width+crc_width && ~stop)begin
						  nextState=SHIFT;
							nextcount=count+1;
							crc=~crcCheckbits[(data_width+crc_width-1)-count];   
						end
				  else if (count<=data_width+crc_width && stop)begin
						  nextState=SHIFT;
							nextcount=count;
							crc=~crcCheckbits[(data_width+crc_width-1)-count];   
						end
				else if (count>data_width+crc_width)begin
							nextState=WAIT;
							//transmitting=1'b0;
						end 
				  else nextState=WAIT;

   endcase // case (state)
      
   end // always_comb begin

   always_ff @(posedge clk, negedge reset_b)
     begin

	if(~reset_b)
	  begin
	     crcCheckbits<=5'h1F;
	     transmitting<=0;
		  count<=7'd0;
		  state<=WAIT;
	     
	  end
	else if (nextState==WAIT) begin
	   crcCheckbits<=5'h1F;	
	   transmitting<=0;
		state<=nextState;
		count<=nextcount;
	   
	end	
	else if (nextState==CALCULATE) begin
	   crcCheckbits[0]<=crcCheckbits[crc_width-1]^input_packet[(data_width-1)-count];
	   crcCheckbits[1]<=crcCheckbits[0];
	   crcCheckbits[2]<=crcCheckbits[1]^crcCheckbits[crc_width-1]^input_packet[(data_width-1)-count];
	   crcCheckbits[3]<=crcCheckbits[2];
	   crcCheckbits[4]<=crcCheckbits[3];
	   transmitting<=0;
	   count<=nextcount;	
		state<=nextState;		
	end
		else if (nextState==SHIFT) begin
		   transmitting<=1;
		   count<=nextcount;
			state<=nextState;
		   
	   end
     end // always_ff @ (posedge clk, negedge reset_b)
endmodule // crc5_check_encoding


module crc16_check_encoding
     #(parameter data_width = 64, crc_width=16)
  (input logic clk, reset_b, load, stop,
   input logic [63:0] input_packet,
   output logic crc,transmitting);


   typedef enum bit [1:0] {WAIT=2'd0,CALCULATE=2'd1, SHIFT=2'd2} states;
   states state,nextState;
   
   bit [6:0] 	count,nextcount;
   bit 		enable;
   bit [crc_width-1:0] 	crcCheckbits;
   

   always_comb begin
   case(state) 
      WAIT:if (load) begin
			  nextState = CALCULATE;
			  enable=1;	
					  if(enable && count<data_width)begin
					  nextState = CALCULATE;
					  nextcount=count+1; 
					  end
				
				end
				else  begin
					 nextState=WAIT;
					 enable=0;
					 nextcount=7'd0;
				end
      CALCULATE: 
				 if(enable && count<data_width && ~stop)begin
					  nextState = CALCULATE;
					 nextcount=count+1; 
				 end
				 else if (enable && count<data_width && stop) begin
					nextState = CALCULATE;
					nextcount = count;
				 end
				 else if (count== data_width && ~stop)begin
					  nextState = SHIFT;
					   nextcount=count+1; 
						crc = ~crcCheckbits[(data_width+crc_width-1)-count];
						transmitting=1'b1;					
					 end
					  else if (count== data_width && stop)begin
					  nextState = SHIFT;
					   nextcount=count; 
						crc = ~crcCheckbits[(data_width+crc_width-1)-count];
						transmitting=1'b1;					
					 end
					 
      SHIFT:	  if (count<data_width+crc_width && ~stop)begin
						  nextState=SHIFT;
							nextcount=count+1;
							crc=~crcCheckbits[(data_width+crc_width-1)-count];   
						end
				  else if (count<data_width+crc_width && stop)begin
						  nextState=SHIFT;
							nextcount=count;
							crc=~crcCheckbits[(data_width+crc_width-1)-count];   
						end
				else if (count>=data_width+crc_width && ~stop)begin
							nextState=WAIT;	
				         transmitting=1'b0;
				         enable=1'b0;			
						end 
				else if (count>=data_width+crc_width && stop)begin
							nextState=SHIFT;							
						end 
				  else nextState=WAIT;

   endcase // case (state)
      
   end // always_comb begin

   always_ff @(posedge clk, negedge reset_b)
     begin

	if(~reset_b)
	  begin
	     crcCheckbits<=16'hFFFF;
	    // transmitting<=0;
		  count<=7'd0;
		  state<=WAIT;
	     
	  end
	else if (nextState==WAIT) begin
	   crcCheckbits<=16'hFFFF;	
	   //transmitting<=0;
		state<=nextState;
		count<=nextcount;
	   
	end	
	else if (nextState==CALCULATE && ~stop) begin
	   crcCheckbits[0]<=crcCheckbits[crc_width-1]^input_packet[(data_width-1)-count];
	   crcCheckbits[1]<=crcCheckbits[0];
	   crcCheckbits[2]<=crcCheckbits[1]^crcCheckbits[crc_width-1]^input_packet[(data_width-1)-count];
	   crcCheckbits[3]<=crcCheckbits[2];
	   crcCheckbits[4]<=crcCheckbits[3];
	   crcCheckbits[5]<=crcCheckbits[4];
	   crcCheckbits[6]<=crcCheckbits[5];
	   crcCheckbits[7]<=crcCheckbits[6];
	   crcCheckbits[8]<=crcCheckbits[7];
	   crcCheckbits[9]<=crcCheckbits[8];
	   crcCheckbits[10]<=crcCheckbits[9];
	   crcCheckbits[11]<=crcCheckbits[10];
	   crcCheckbits[12]<=crcCheckbits[11];
	   crcCheckbits[13]<=crcCheckbits[12];
	   crcCheckbits[14]<=crcCheckbits[13];
	   crcCheckbits[15]<=crcCheckbits[14]^crcCheckbits[crc_width-1]^input_packet[(data_width-1)-count];
	   //transmitting<=0;
	   count<=nextcount;   
		state<=nextState;
			end
	else if(nextState == CALCULATE && stop) begin
		crcCheckbits <= crcCheckbits;
		count <= nextcount;
		state <= nextState;
	end

		else if (nextState==SHIFT) begin
		  // transmitting<=1;
		   count<=nextcount;
			state<=nextState;
		   
	   end
     end // always_ff @ (posedge clk, negedge reset_b)
endmodule // crc16_check_encoding


module inputconverter ( input logic clk, rst_L, shift_en,done,eop, usbWires wires,
								output logic shift_out,success_jk_nrzi,sync_detected);
								
logic [1:0] shift_out_to_nrzi;
logic legal,success_nrzi;
assign success_jk_nrzi = success_nrzi&&legal;
jkconversion jk(clk, rst_L, shift_en,done,eop,sync_detected, wires,legal,shift_out_to_nrzi);
NRZI_conversion nrzi(clk,rst_L,shift_en,done,eop,shift_out_to_nrzi,shift_out,success_nrzi,sync_detected);			
			
endmodule
module jkconversion (input logic clk, rst_L, shift_en, done, eop,sync_detected,usbWires wires,
							output logic legal, output logic [1:0] shift_out);
			
typedef enum logic [1:0] {IDLE=2'd0, CONVERT=2'd1, EOP = 2'd2, DONE=2'd3} state_t;
state_t state, next_state;

//logic [1:0] count,next_count;
logic [1:0] converted_output;
logic sync_detected_latch;

always_ff @(posedge clk, negedge rst_L) begin
				if (~rst_L) begin
				//count<=2'd0;
				state<=IDLE;
				end
				else begin
				state<=next_state;
				end
				if (sync_detected)
				sync_detected_latch<=1'b1;
				else if (state==EOP)
				sync_detected_latch<=1'b0;
				else
				sync_detected_latch<=sync_detected_latch;
end//always_ff

always_comb begin

					case (state)
					IDLE: if (shift_en) begin
							next_state=CONVERT;
							shift_out = converted_output;
							end
						   else
							next_state=IDLE;
				CONVERT: if (eop) begin
								next_state = EOP;
								shift_out=converted_output;
								end
								else if (~eop) begin
								shift_out = converted_output;
								next_state=CONVERT;
								end							
					EOP: if (eop) begin
						  shift_out = converted_output;
						  next_state = EOP;
						  end
						  else begin
						  next_state = DONE;
						  end
					DONE: begin
							next_state = IDLE;
							//shift_out=2'b0;
							end
					default: next_state = IDLE;
					endcase


end//always_comb

always_comb begin
			if (shift_en) begin
			legal=1'b1;
								case({wires.DP,wires.DM})
								2'b00: converted_output = 2'b10;
								2'b01:  converted_output = 2'b00;								
								2'b10:  converted_output = 2'b01;
								2'b11:  begin legal = 1'b0;
								        converted_output=2'b11;
										  end
								default: begin 
											converted_output = 2'b0;
											legal=1'b1;
											end
								endcase
			end
			else converted_output = 2'b00;
end//always_comb			
endmodule 

module NRZI_conversion(input logic clk, rst_L,shift_en, done, eop, input logic [1:0] shift_in,
								output logic shift_out,success_nrzi,sync_detected);
								
typedef enum logic [1:0] {IDLE=2'd0, SHIFT=2'd1, EOP_CHECK=2'd2} state_t;
state_t state,next_state;

bit [1:0] count,next_count;
logic [1:0]nrzi, nrzi_prev;
logic [2:0] success;
logic [3:0]sync_count,sync_next_count;
logic [7:0] sync_detector;
bit sync_prev_detected, internal_reset;

assign nrzi = (shift_en)? ((shift_in==nrzi_prev)? 2'b01:2'b00):2'b00,
       success_nrzi = (done)? |(success) :1'b0,
		 sync_detected = (shift_en && sync_count==4'd8 && ~sync_prev_detected)?(sync_detector==8'b00000001):1'b0;

always_ff @(posedge clk, negedge rst_L) begin
	if (~rst_L || (shift_in==2'b10 && ~eop)) begin
		count<=2'd0;
		state<=IDLE;
		nrzi_prev<=2'b01;
		sync_count<=3'd0;
		sync_detector<=7'd0;
		            end
		else if (shift_en) begin
		state<=next_state;
		count<=next_count;
		sync_count<=sync_next_count;
		nrzi_prev<=(next_state == IDLE)? 2'b01: shift_in;
		            end
						
	  if (sync_count<=4'd7 && (next_state==SHIFT) && shift_en && shift_in !=2'b10)
		sync_detector <= ({sync_detector[6:0], shift_out});			
	   else begin
		sync_detector<=8'd0;
		sync_count<=4'd0;
		 end	
		
		if (sync_detected) 
		 sync_prev_detected<=1'b1;
		 else if (done)
		 sync_prev_detected<=1'b0;
		 
end//always_ff

always_comb begin
				case (state)
				IDLE: begin 						
						if (shift_en && ~eop && ~done) begin
						next_state = SHIFT;
						shift_out = nrzi[0];
						sync_next_count=(shift_in!=2'b00||shift_in!=2'b11)?sync_count+1: 7'd0;
						end
						else next_state = IDLE;
						end
				SHIFT: begin 
						 if (shift_en && ~eop) begin
						 shift_out = nrzi[0];
						 sync_next_count=(shift_in!=2'b00||shift_in!=2'b11)?sync_count+1: 7'd0;
						 end
						 else if (shift_en && eop) begin
						 next_count = count+1;
						 next_state = EOP_CHECK;
						 if (shift_in==2'b10) 
						 success[0] = 1'b1;
						 else success[0] = 1'b0;		 
						 end
						 end
				EOP_CHECK: begin
				next_count = count+1;
				sync_next_count=3'd0;
   			next_state = EOP_CHECK;
			   case (count) 
			   2'd1:		  if (shift_in!=2'b10) begin
							  success[1] = 1'b0;							  
							  end
							  else begin success[1]=1'b1;
							   end
				2'd2:		  begin 
							  next_state = IDLE;
							  next_count=2'd0;
				           if (shift_in!=2'b01)
							  success[2] = 1'b0;			
							  else 
							  success[2]=1'b1;							
							  end
		      default: next_state=IDLE;
				endcase
							  end
				default:next_state = IDLE;
				endcase
end//always_comb
								
						
endmodule 

				
module bitunstuffer
(input logic clk, rst_L, shift_in,shift_en,
 output logic shift_out,reject);

   logic [2:0] count, nextCount;
   logic [6:0] 	    total_count;
   logic 	    bit_unstuff, sync_detected_latch;

   assign bit_unstuff=(total_count>7'd15);	
always_comb begin
		shift_out = shift_in;
		nextCount = 3'd0;
		reject=1'b0;
		
		if (shift_en && bit_unstuff) begin
			if (count == 6) begin
			   reject=1'b1;			   
			end
			else if (shift_in)begin
				nextCount = count+1;
			end
		end

end//always_comb
	
	always_ff @(posedge clk, negedge rst_L) begin
		if (~rst_L) begin
			count <= 3'd0;
		   total_count<=7'd0;
		   
		end
		else begin
			count <= nextCount;
		   total_count<=(shift_en)? total_count+1: '0;
		   
		end
	end//always_ff
endmodule // bitunstuffer

module bitstreamdecoding (input logic clk, rst_L,shift_in,shift_en, reject,crc_in,crc_transmitting,sync_detected,
									output logic done, eop, success_out,crc5_en,crc16_en,crc_out,
								   output pkt_t output_packet);

	typedef enum logic [3:0] {NONE=4'd0, SYNC=4'd1, PID=4'd2, ADDR=4'd3, ENDP=4'd4, DATA=4'd5, CRC5=4'd6, CRC16=4'd7,EOP=4'd8} type_t;
   typedef enum logic [1:0] {IDLE=2'd0, SHIFT_IN=2'd1, DONE=2'd2} state_t;
   type_t current_type, next_type;
   state_t state,next_state;
	logic stream_in,sync_detected_latch,stop_crc;
	pkt_t decoded_packet;
	logic [7:0] count, next_count, last_count,crc_count,crc_next_count;
	logic [3:0] success;
	logic [4:0] crc5_calc;
	logic [15:0] crc16_calc;
	logic success_bitstreamdecoding;
	assign success_bitstreamdecoding = &success[3:1];

always_comb begin
next_type = NONE;
case (state) 
IDLE: if (shift_en && ~ reject) begin
		done=1'b0;
		next_state= SHIFT_IN;
		next_count=count;
		eop=1'b0;
		stream_in = shift_in;
		end
		else begin
		next_state=IDLE;
		next_count=count;		
		done=1'b0;
		eop=1'b0;
		end
SHIFT_IN: if (count==last_count+1)
			 next_state=DONE;
			 else next_state = SHIFT_IN;
DONE: begin
		done=1'b1;
		next_state=IDLE;
		end
default: next_state=IDLE;
endcase
if (shift_en) begin
			case (current_type)
			SYNC: begin 
					if ((count<8'd7) && ~reject && ~sync_detected) begin
					next_type=SYNC;
					next_count=count+1;
					stream_in=shift_in;
					end
					else if ((count<=8'd7) && ~reject && sync_detected) begin
					next_type=PID;
					next_count=8'd8;
					stream_in=shift_in;
					end
					else if (reject) begin
					next_type=current_type;
					//next_count=count;
					end
					else if (~sync_detected) begin
					next_type=SYNC;
					next_count=7'd0;
					end
					end
			PID: begin
			   
				   if ((count<8'd15) && ~reject) begin
					next_type=PID;
					next_count=count+1;
					stream_in=shift_in;
					end
					else if ((count==8'd15) && ~reject) begin					
					next_count=count+1;
					stream_in=shift_in;
						case(decoded_packet.PID[7:4])
						4'b1000:begin 
								  next_type = ADDR;//Token - OUT
								  last_count = 8'd34;
								  stream_in=shift_in;
								  crc_out=stream_in;
								  crc5_en=1'b1;
								  end
						4'b1001: begin 	
									next_type = ADDR; //Token - IN
									last_count=8'd34;
									stream_in=shift_in;
									crc_out=stream_in;
									crc5_en=1'b1;
									end
						4'b1100: begin 
									next_type=DATA; //Data
									last_count=8'd97;
									stream_in=shift_in;
									crc_out=stream_in;
									crc16_en=1'b1;
									end
						4'b0100: begin 
									next_type = EOP; 
									last_count=8'd17;
									eop=1'b1;
									end//ACK
						4'b0101:begin 
									next_type= EOP;
									last_count=8'd17;
									eop=1'b1;
									end //NAK
						default:begin 
								  //success_bitstreamdecoding=1'b1;	
								  end	
						endcase // case (decoded_packet.PID[7:4])	 
					end
					else if (reject) begin
					next_type=current_type;
					//next_count=count;
					end
					end					
			ADDR: begin
					crc5_en=1'b1;
					crc16_en=1'b0;
					stop_crc=1'b0;
					if ((count<8'd22) && ~reject) begin
					next_type=ADDR;
					next_count=count+1;
					stream_in=shift_in;
					crc_out=stream_in;
					end
					else if ((count==8'd22) && ~reject) begin
					next_type=ENDP;
					next_count=count+1;
					stream_in=shift_in;
					crc_out=stream_in;
					end
					else if (reject) begin
					next_type=current_type;
					stop_crc=1'b1;					
					//next_count=count;
					end
					end
			ENDP: begin
					crc5_en=1'b1;
					crc16_en=1'b0;
					stop_crc=1'b0;	
					if ((count<8'd26) && ~reject) begin
					next_type=ENDP;
					next_count=count+1;
					stream_in=shift_in;
				   crc_out=stream_in;
					end
					else if ((count==8'd26) && ~reject) begin
					next_type=CRC5;
					crc_next_count=crc_count+1;
					next_count=count+1;
					stream_in=shift_in;
					crc_out=stream_in;
					end
					else if (reject) begin
					next_type=current_type;
					stop_crc=1'b1;		
					//next_count=count;
					end
					end
			DATA: begin
			      crc16_en=1'b1;
					crc5_en=1'b0;
				   stop_crc=1'b0;
					if ((count<8'd79) && ~reject) begin
					next_type=DATA;
					next_count=count+1;
					stream_in=shift_in;
					crc_out=stream_in;
					end
					else if ((count==8'd79) && ~reject) begin
					next_type=CRC16;
					crc_next_count=crc_count+1;
					next_count=count+1;
					stream_in=shift_in;
					crc_out=stream_in;
					end
					else if (reject) begin
					next_type=current_type;
					stop_crc=1'b1;		
					//next_count=count;
					end
					end
			CRC5:begin
			      crc16_en=1'b0;
					crc5_en=1'b0;
					if ((count<8'd31) && ~reject) begin
					next_type=CRC5;
					next_count=count+1;
					stream_in=shift_in;
					end
					else if ((count==8'd31) && ~reject) begin
					next_type=EOP;
					next_count=count+1;
					stream_in=shift_in;
					end
					else if (reject) begin
					next_type=current_type;
					//next_count=count;
					end
					end
			CRC16:begin
			      crc16_en=1'b0;
					crc5_en=1'b0;
					if ((count<8'd95) && ~reject) begin
					next_type=CRC16;
					next_count=count+1;
					stream_in=shift_in;
					end
					else if ((count==8'd95) && ~reject) begin
					next_type=EOP;
					next_count=count+1;
					stream_in=shift_in;
					eop = 1'b1;
					end
					else if (reject) begin
					next_type=current_type;
					//next_count=count;
					end
					end
			EOP:  begin
					if (count<last_count+1) begin
					eop=1'b1;
					next_count=count+1;
					next_type=EOP;
					end
					else if(count==last_count+1) begin
					eop=1'b0;
					next_type=NONE;
					//next_state=DONE;
					end
					end
			NONE: begin
					eop=1'b0;
					next_count=8'd0;
					if (shift_en && ~done) begin
					next_type=SYNC;
					end
					end
			default:begin 
			        next_type=NONE;
					  next_count=7'd0;
					  next_state=IDLE;
					  end
			endcase
			end
else begin
next_count=7'd0;
next_state=IDLE;			
			end
	if (done) begin
		output_packet.sync=decoded_packet.sync;
		output_packet.PID=decoded_packet.PID;
		output_packet.addr=decoded_packet.addr;
		output_packet.endp=decoded_packet.endp;
		output_packet.data=decoded_packet.data;
		output_packet.CRC5=decoded_packet.CRC5;
		output_packet.CRC16=decoded_packet.CRC16;
    end 
	 
	case (count) inside
      8'd7: if (decoded_packet.sync!=8'd1) 
				success[0]=1'b0;	 
           	else success[0] =1'b1;
      8'd15: if (decoded_packet.PID[7:4]!=(~decoded_packet.PID[3:0])) 
	        	success[1]=1'b0;	 
           	else success[1] =1'b1;
     /* 8'd22: if ((decoded_packet.PID[7:4]==4'b1000)||(decoded_packet.PID[7:4]==4'b1001)) begin
	    	    if (decoded_packet.addr!=7'b1010000)
						success[2]=1'b0;	 
				 else success[2] =1'b1;
				 end
     // 8'd27:if ((decoded_packet.PID[7:4]==4'b1000)||(decoded_packet.PID[7:4]==4'b1001)) begin
				if (decoded_packet.endp!=5'b00010)
					success[3]=1'b0;	 
				 else success[3] =1'b1;
				 end */
      8'd32: if ((decoded_packet.PID[7:4]==4'b1000)||(decoded_packet.PID[7:4]==4'b1001)) begin
							if (decoded_packet.CRC5!=crc5_calc)
							success[2]=1'b0;	 
				 else success[2] =1'b1;
				 end
      8'd95:  if (decoded_packet.PID[7:4]==4'b1100 && ~reject) begin
					if (decoded_packet.CRC16!=crc16_calc)
							success[3]=1'b0;	 
				 else success[3] =1'b1;
				 end
      default: success=4'hF;
    endcase // case (count)

end
 //always_comb begin
     
//end
always_ff @(posedge clk, negedge rst_L) begin
if (~rst_L) begin
state<=IDLE;
current_type<=NONE;
crc5_calc<=5'd0;
crc16_calc<=16'd0;
count<=7'd0;
success_out <= 1'b1;
end
else if (shift_en)begin
state<=next_state;
current_type<=next_type;
success_out <= (success_bitstreamdecoding)? success_out : 1'b0;
end
else if (~shift_en)begin
current_type<=NONE;
state<=IDLE;
count<=7'd0;
success_out <= 1'b1;
end


if (next_type==SYNC && shift_en && ~reject) begin      
					decoded_packet.sync<={decoded_packet.sync[6:0],stream_in};
					count<=next_count;
			end
			else if (next_type==PID && shift_en && ~reject) begin      
					decoded_packet.PID<={decoded_packet.PID[6:0],stream_in};
					count<=next_count;
			end
			else if (next_type==ADDR && shift_en && ~reject) begin      
					decoded_packet.addr<={decoded_packet.addr[5:0],stream_in};
					count<=next_count;
			end
			else if (next_type==ENDP && shift_en && ~reject) begin      
					decoded_packet.endp<={decoded_packet.endp[3:0],stream_in};
					count<=next_count;
			end
			else if (next_type==DATA && shift_en && ~reject) begin      
					decoded_packet.data<={decoded_packet.data[62:0],stream_in};
					count<=next_count;
			end
			else if (next_type==CRC5 && shift_en && ~reject) begin      
					decoded_packet.CRC5<={decoded_packet.CRC5[3:0],stream_in};
					count<=next_count;
					end
			else if (next_type==CRC16 && shift_en && ~reject) begin      
					decoded_packet.CRC16<={decoded_packet.CRC16[14:0],stream_in};
					count<=next_count;		
			end
			else if (next_type ==EOP && shift_en && ~reject) begin
			count<=next_count;
			end
			else begin
			 decoded_packet<=decoded_packet;
			end
			
			
			
if (crc_transmitting &&(decoded_packet.PID[7:4]==4'b1000||decoded_packet.PID[7:4]==4'b1001) && crc_next_count<=7'd4) begin 
crc5_calc <={crc5_calc[3:0],crc_in};
crc_count<=crc_next_count;
end
else if (crc_transmitting && (decoded_packet.PID[7:4]==4'b1100) && crc_next_count<=7'd15) begin
crc16_calc<={crc16_calc[14:0],crc_in};
crc_count<=crc_next_count;
end
else if (done) begin
crc5_calc<=5'd0;
crc16_calc<=16'd0;
crc_count<=8'd0;
end

if (sync_detected)
sync_detected_latch<=1'b1;
else if (done)
sync_detected_latch<=1'b0;
else
sync_detected_latch<=sync_detected_latch;				
end


endmodule 

module crc_decoding
(input logic clk, rst_L,crc5_en, crc16_en, stop_crc, crc_in,
 output logic crc_transmitting,crc_out);
 
 logic crc_5_out,crc_16_out,crc_5_transmitting,crc_16_transmitting;
 
 assign crc_out = (crc5_en && ~crc16_en)? crc_5_out:crc_16_out,
			crc_transmitting=(crc5_en && ~crc16_en)? crc_5_transmitting:crc_16_transmitting;
 crc5_check_decoding crc_5(clk, rst_L,crc5_en,stop_crc,crc_in,crc_5_out,crc_5_transmitting);
 crc16_check_decoding crc_16(clk,rst_L,crc16_en, stop_crc,crc_in,crc_16_out,crc_16_transmitting); 
 
 endmodule 

module success(input logic success_jk_nrzi,success_bitstreamdecoding,done,
	       output logic success);
assign success = (done)? (success_jk_nrzi&&success_bitstreamdecoding):1'b1;
endmodule // success
 
  module crc5_check_decoding
     #(parameter data_width = 11, crc_width=5)
  (input logic clk, reset_b, load, stop,
   input logic  input_bit,
   output logic crc,transmitting);


     typedef enum bit [1:0] {WAIT=2'd0,CALCULATE=2'd1, SHIFT=2'd2} states;
   states state,nextState;
   
   bit [6:0] 	count,nextcount;
   bit 		enable;
   bit [crc_width-1:0] 	crcCheckbits;
   

   always_comb begin
   case(state) 
      WAIT:if (load) begin
			  nextState = CALCULATE;
			  enable=1;	
					  if(enable && count<data_width-1)begin
					  nextState = CALCULATE;
					  nextcount=count+1; 
					  end
				
				end
				else  begin
					 nextState=WAIT;
					 enable=0;
					 nextcount=7'd0;
				end
      CALCULATE: 
				 if(enable && count<data_width && ~stop)begin
					  nextState = CALCULATE;
					 nextcount=count+1; 
				 end
				 else if (enable && nextcount<data_width-2 && stop) begin
					nextState = CALCULATE;
					nextcount = count;
				 end
				 else if (count== data_width && ~stop)begin
					  nextState = SHIFT;
					   nextcount=count+1; 
						crc = ~crcCheckbits[(data_width+crc_width-1)-count];
						transmitting=1'b1;					
					 end
					  else if (count== data_width && stop)begin
					  nextState = SHIFT;
					   nextcount=count; 
						crc = ~crcCheckbits[(data_width+crc_width-1)-count];
						transmitting=1'b1;					
					 end
					 
      SHIFT:	  if (count<data_width+crc_width && ~stop)begin
						  nextState=SHIFT;
							nextcount=count+1;
							crc=~crcCheckbits[(data_width+crc_width-1)-count];   
						end
				  else if (count<data_width+crc_width && stop)begin
						  nextState=SHIFT;
							nextcount=count+1;
							crc=~crcCheckbits[(data_width+crc_width-1)-count];   
						end
				else if (count>=data_width+crc_width && ~stop)begin
							nextState=WAIT;	
				         transmitting=1'b0;
				         enable=1'b0;			
						end 
				else if (count>=data_width+crc_width && stop)begin
							nextState=SHIFT;							
						end 
				  else nextState=WAIT;

   endcase // case (state)
      
   end // always_comb begin

   always_ff @(posedge clk, negedge reset_b)
     begin

	if(~reset_b)
	  begin
	     crcCheckbits<=5'h1F;
	    // transmitting<=0;
		  count<=7'd0;
		  state<=WAIT;
	     
	  end
	else if (nextState==WAIT) begin
	   crcCheckbits<=5'h1F;	
	   //transmitting<=0;
		state<=nextState;
		count<=nextcount;
	   
	end	
	else if (nextState==CALCULATE && ~stop) begin
	   crcCheckbits[0]<=crcCheckbits[crc_width-1]^input_bit;
	   crcCheckbits[1]<=crcCheckbits[0];
	   crcCheckbits[2]<=crcCheckbits[1]^crcCheckbits[crc_width-1]^input_bit;
	   crcCheckbits[3]<=crcCheckbits[2];
	   crcCheckbits[4]<=crcCheckbits[3];
	   count<=nextcount;   
		state<=nextState;
			end
	else if(nextState == CALCULATE && stop) begin
		crcCheckbits <= crcCheckbits;
		count <= nextcount;
		state <= nextState;
	end

		else if (nextState==SHIFT) begin
		  // transmitting<=1;
		   count<=nextcount;
			state<=nextState;
		   
	   end
     end // always_ff @ (posedge clk, negedge reset_b)
endmodule 

module crc16_check_decoding
     #(parameter data_width = 64, crc_width=16)
  (input logic clk, reset_b, load, stop,
   input logic  input_bit,
   output logic crc,transmitting);


     typedef enum bit [1:0] {WAIT=2'd0,CALCULATE=2'd1, SHIFT=2'd2} states;
   states state,nextState;
   
   bit [6:0] 	count,nextcount;
   bit 		enable;
   bit [crc_width-1:0] 	crcCheckbits;
   

   always_comb begin
   case(state) 
      WAIT:if (load) begin
			  nextState = CALCULATE;
			  enable=1;	
					  if(enable && count<data_width-1)begin
					  nextState = CALCULATE;
					  nextcount=count+1; 
					  end
				
				end
				else  begin
					 nextState=WAIT;
					 enable=0;
					 nextcount=7'd0;
				end
      CALCULATE: 
				 if(enable && count<data_width && ~stop)begin
					  nextState = CALCULATE;
					 nextcount=count+1; 
				 end
				 else if (enable && nextcount<data_width-2 && stop) begin
					nextState = CALCULATE;
					nextcount = count;
				 end
				 else if (count== data_width && ~stop)begin
					  nextState = SHIFT;
					   nextcount=count+1; 
						crc = ~crcCheckbits[(data_width+crc_width-1)-count];
						transmitting=1'b1;					
					 end
					  else if (count== data_width && stop)begin
					  nextState = SHIFT;
					   nextcount=count; 
						crc = ~crcCheckbits[(data_width+crc_width-1)-count];
						transmitting=1'b1;					
					 end
					 
      SHIFT:	  if (count<data_width+crc_width && ~stop)begin
						  nextState=SHIFT;
							nextcount=count+1;
							crc=~crcCheckbits[(data_width+crc_width-1)-count];   
						end
				  else if (count<data_width+crc_width && stop)begin
						  nextState=SHIFT;
							nextcount=count+1;
							crc=~crcCheckbits[(data_width+crc_width-1)-count];   
						end
				else if (count>=data_width+crc_width && ~stop)begin
							nextState=WAIT;	
				         transmitting=1'b0;
				         enable=1'b0;			
						end 
				else if (count>=data_width+crc_width && stop)begin
							nextState=SHIFT;							
						end 
				  else nextState=WAIT;

   endcase // case (state)
      
   end // always_comb begin

   always_ff @(posedge clk, negedge reset_b)
     begin

	if(~reset_b)
	  begin
	     crcCheckbits<=16'hFFFF;
	    // transmitting<=0;
		  count<=7'd0;
		  state<=WAIT;
	     
	  end
	else if (nextState==WAIT) begin
	   crcCheckbits<=16'hFFFF;	
	   //transmitting<=0;
		state<=nextState;
		count<=nextcount;
	   
	end	
	else if (nextState==CALCULATE && ~stop) begin
	   crcCheckbits[0]<=crcCheckbits[crc_width-1]^input_bit;
	   crcCheckbits[1]<=crcCheckbits[0];
	   crcCheckbits[2]<=crcCheckbits[1]^crcCheckbits[crc_width-1]^input_bit;
	   crcCheckbits[3]<=crcCheckbits[2];
	   crcCheckbits[4]<=crcCheckbits[3];
	   crcCheckbits[5]<=crcCheckbits[4];
	   crcCheckbits[6]<=crcCheckbits[5];
	   crcCheckbits[7]<=crcCheckbits[6];
	   crcCheckbits[8]<=crcCheckbits[7];
	   crcCheckbits[9]<=crcCheckbits[8];
	   crcCheckbits[10]<=crcCheckbits[9];
	   crcCheckbits[11]<=crcCheckbits[10];
	   crcCheckbits[12]<=crcCheckbits[11];
	   crcCheckbits[13]<=crcCheckbits[12];
	   crcCheckbits[14]<=crcCheckbits[13];
	   crcCheckbits[15]<=crcCheckbits[14]^crcCheckbits[crc_width-1]^input_bit;
	   //transmitting<=0;
	   count<=nextcount;   
		state<=nextState;
			end
	else if(nextState == CALCULATE && stop) begin
		crcCheckbits <= crcCheckbits;
		count <= nextcount;
		state <= nextState;
	end

		else if (nextState==SHIFT) begin
		  // transmitting<=1;
		   count<=nextcount;
			state<=nextState;
		   
	   end
     end // always_ff @ (posedge clk, negedge reset_b)
endmodule  // crc16_check_decoding
