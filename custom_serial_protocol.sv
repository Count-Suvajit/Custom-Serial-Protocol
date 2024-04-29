
  //AM = 0x55
  //Send 5 successive AMs to assert link_stable
  //SOP = 0xFB
  //EOP = 0xFD
  //IDLE = 0x07
  
  //Packet Format: -
  //<IDLE>
  //<SOP>
  //<ID>
  //<LENGTH>
  //<COMMAND>
  //<EOP>
 
  //Command: -
  //LED0 ON = <00>
  //LED0 OFF = <01>
  //LED1 ON = <10>
  //LED1 OFF = <11>

  //<ID> is a configurable parameter in RTL.
  //RTL will not send anything on tx_serial till link_stable
  //2 RTLs are connected in daisy chain.
  //Minimum length of packet should be 1 otherwise it will lead to error.

`timescale 1ns/1ps

`include "uvm_macros.svh"
import uvm_pkg::*;

module custom_serial_protocol_top();

  reg clk; 
  reg rstn;

  reg rx_serial1;
  reg [7:0] addr1;
  reg [7:0] wr_data1;
  reg write1;
  reg valid1;
  reg tx_serial1;
  reg [7:0] read_data1;
  reg link_stable1;
  reg serial_error1;
  reg [1:0] led1;
  localparam link_id1 = 'h11;

  reg rx_serial2;
  reg [7:0] addr2;
  reg [7:0] wr_data2;
  reg write2;
  reg valid2;
  reg tx_serial2;
  reg [7:0] read_data2;
  reg link_stable2;
  reg serial_error2;
  reg [1:0] led2;
  localparam [7:0] link_id2 = 'h22;

  custom_serial_interface custom_serial_if0_tx(clk,rstn);
  custom_serial_interface custom_serial_if0_rx(clk,rstn);
  custom_serial_interface custom_serial_if1_tx(clk,rstn);
  custom_serial_interface custom_serial_if1_rx(clk,rstn);

  initial begin
	  uvm_config_db#(virtual custom_serial_interface)::set(null,"uvm_test_top.top_env.env0","custom_serial_vif0",custom_serial_if0_tx);
	  uvm_config_db#(virtual custom_serial_interface)::set(null,"uvm_test_top.top_env.env0","custom_serial_vif1",custom_serial_if0_rx);
	  uvm_config_db#(virtual custom_serial_interface)::set(null,"uvm_test_top.top_env.env1","custom_serial_vif0",custom_serial_if1_tx);
	  uvm_config_db#(virtual custom_serial_interface)::set(null,"uvm_test_top.top_env.env1","custom_serial_vif1",custom_serial_if1_rx);

	  uvm_config_db#(int)::set(null,"uvm_test_top.top_env.env0","link_id",link_id1);
	  uvm_config_db#(int)::set(null,"uvm_test_top.top_env.env1","link_id",link_id2);

          custom_serial_if0_tx.instance_name = "serial_if0_tx";
          custom_serial_if0_rx.instance_name = "serial_if0_rx";
          custom_serial_if1_tx.instance_name = "serial_if1_tx";
          custom_serial_if1_rx.instance_name = "serial_if1_rx";
  end

  initial begin
     clk=0;
     forever begin
        #0.5 clk <= ~clk;
     end
  end

  initial begin
     rstn=0;
     #2;
     rstn=1;
  end

  //UVM TEST : START
  
  initial begin
     uvm_config_db#(int)::set(null,"uvm_test_top.top_virtual_seqr","VAL",32'hFFFFFFFF);
     #1;
     uvm_config_db#(int)::set(null,"uvm_test_top.top_virtual_seqr","VAL",32'hFFFFFFFF);
  end
  initial begin
     run_test("custom_serial_test");
  end
  //UVM TEST : END

  //NON CLASS BASED TEST : START

  /*
  event test_done;

  initial begin
     drive_serial_data(32'hFFFFFFFF,32);
     drive_serial_data(32'h0,$urandom_range(32,1));
     repeat(5) begin
        drive_serial_data(8'h55,8);
     end
     drive_serial_data(8'h07,8);
     repeat(5) begin
        drive_serial_data(8'h55,8);
     end
     drive_serial_data(8'h07,8);
     drive_serial_data(8'h07,8);
     drive_serial_data(8'h07,8);
     //Packet 1 with ID=22
     drive_serial_data(8'hfb,8);
     drive_serial_data(8'h22,8);
     drive_serial_data(8'h01,8);
     drive_serial_data(8'h00,8);
     drive_serial_data(8'hfd,8);
     drive_serial_data(8'h07,8);
     drive_serial_data(8'h07,8);
     drive_serial_data(8'h07,8);
     drive_serial_data(8'h07,8);
     //Packet 2 with ID=11
     drive_serial_data(8'hfb,8);
     drive_serial_data(8'h11,8);
     drive_serial_data(8'h01,8);
     drive_serial_data(8'h10,8);
     drive_serial_data(8'hfd,8);
     drive_serial_data(8'h07,8);
     drive_serial_data(8'h07,8);
     drive_serial_data(8'h07,8);
     drive_serial_data(8'h07,8);
     drive_serial_data(8'h07,8);
     //Packet 3 with ID=11 with missing EOP
     drive_serial_data(8'hfb,8);
     drive_serial_data(8'h11,8);
     drive_serial_data(8'h01,8);
     drive_serial_data(8'h11,8);
     //Packet 4 with ID=22
     drive_serial_data(8'hfb,8);
     drive_serial_data(8'h22,8);
     drive_serial_data(8'h03,8);
     drive_serial_data(8'h01,8);
     drive_serial_data(8'h00,8);
     drive_serial_data(8'h00,8);
     drive_serial_data(8'hfd,8);
     drive_serial_data(8'h07,8);
     drive_serial_data(8'h07,8);
     drive_serial_data(8'h07,8);
     drive_serial_data(8'h07,8);
     //Packet 5 with ID=22 with invalid command
     drive_serial_data(8'hfb,8);
     drive_serial_data(8'h22,8);
     drive_serial_data(8'h01,8);
     drive_serial_data(8'h33,8);
     drive_serial_data(8'hfd,8);
     drive_serial_data(8'h07,8);
     drive_serial_data(8'h07,8);
     drive_serial_data(8'h07,8);
     drive_serial_data(8'h07,8);
     //Packet 6 with ID=11 with invalid length
     drive_serial_data(8'hfb,8);
     drive_serial_data(8'h22,8);
     drive_serial_data(8'h00,8);
     drive_serial_data(8'h33,8);
     drive_serial_data(8'hfd,8);
     drive_serial_data(8'h07,8);
     drive_serial_data(8'h07,8);
     drive_serial_data(8'h07,8);
     drive_serial_data(8'h07,8);
     //Packet 7 with ID=11 with length mismatch
     drive_serial_data(8'hfb,8);
     drive_serial_data(8'h22,8);
     drive_serial_data(8'h02,8);
     drive_serial_data(8'h33,8);
     drive_serial_data(8'h00,8);
     drive_serial_data(8'h00,8);
     drive_serial_data(8'hfd,8);
     drive_serial_data(8'h07,8);
     drive_serial_data(8'h07,8);
     drive_serial_data(8'h07,8);
     drive_serial_data(8'h07,8);
     drive_serial_data(8'h07,8);
     ->test_done;
     $display("EVENT TRIGGERED");
  end

  task automatic drive_serial_data(input bit [31:0] data, input bit [5:0] max_idx);
     for(bit [5:0] idx=0; idx<max_idx; idx+=1) begin
        @(posedge clk);
        rx_serial1 = #0 data[idx[4:0]];
     end
  endtask

  initial begin	  
     wait(test_done.triggered);
     $display("EVENT TRIGGER DETECTED");
     $finish();
  end

  */
  //NON CLASS BASED TEST : END

  initial
  begin
     $fsdbDumpfile("custom_serial_protocol_dump.fsdb");
     $fsdbDumpvars("+all");
     $fsdbDumpvars("+parameter");
     $fsdbDumpon();
  end

  serial_rtl dut1(.clk(clk),.rstn(rstn),.rx_serial(custom_serial_if0_tx.serial),.addr(addr1),.wr_data(wr_data1),.write(write1),.valid(valid1),.tx_serial(custom_serial_if0_rx.serial),.read_data(read_data1),.link_stable(link_stable1),.serial_error(serial_error1),.led(led1));
  serial_rtl dut2(.clk(clk),.rstn(rstn),.rx_serial(custom_serial_if1_tx.serial),.addr(addr2),.wr_data(wr_data2),.write(write2),.valid(valid2),.tx_serial(custom_serial_if1_rx.serial),.read_data(read_data2),.link_stable(link_stable2),.serial_error(serial_error2),.led(led2));
  defparam dut1.link_id = link_id1;
  defparam dut2.link_id = link_id2;

endmodule

module serial_rtl
#(parameter link_id)
(
  input clk,
  input rstn,
  input rx_serial,
  input [7:0] addr,
  input [7:0] wr_data,
  input write,
  input valid,
  output reg tx_serial,
  output reg [7:0] read_data,
  output link_stable,
  output serial_error,
  output reg [1:0] led
);

  reg first_am_detect;
  reg command_error;
  reg length_error;
  reg [1:0] led_r;

  reg [15:0] accumulated_data;
  reg [3:0] current_data_index;
  reg [3:0] alignment_index;

  reg [7:0] parallel_data[32];
  reg [4:0] parallel_index;
  wire [4:0] parallel_index_minus_1;
  reg [7:0] length,length_counter;

  typedef enum {LINK_DOWN,AM1,AM2,AM3,AM4,LINK_UP} link_state_e;
  link_state_e link_state;

  typedef enum {IDLE,DETECT_SOP,DETECT_ID,DETECT_LENGTH,DETECT_COMMAND,DETECT_PAYLOAD,DETECT_EOP,PACKET_ERROR} packet_state_e;
  packet_state_e packet_state;

  assign link_stable = link_state == LINK_UP ? 1 : 0;
  assign serial_error = (packet_state == PACKET_ERROR ? 1 : 0) | command_error;

  assign parallel_index_minus_1 = parallel_index - 1;

  always @(posedge clk, negedge rstn) begin
     if(rstn == 0) begin
         first_am_detect = 0;
	 current_data_index = 0;
	 alignment_index = 0;
	 accumulated_data = '0;
	 parallel_index = 0;
	 link_state = LINK_DOWN;
	 packet_state = IDLE;
	 command_error = 0;
	 led = 0;
	 length_error = 0;
     end else begin
         accumulated_data[current_data_index] = rx_serial;
	 if(current_data_index == 15) begin
            accumulated_data[7:0] = accumulated_data[15:8];
	    current_data_index = 8;
         end else begin
	    current_data_index+=1;
	 end
     end
  end

  always @(posedge clk) begin
	 if(current_data_index > 7 && first_am_detect==0) begin
		 if(accumulated_data[current_data_index-1-:8] == 8'h55) begin
                      first_am_detect = 1;
		      alignment_index = current_data_index;
		 end
	 end
  end

  always @(posedge clk) begin
         if(first_am_detect==1 && current_data_index==alignment_index) begin
                 parallel_data[parallel_index] = accumulated_data[current_data_index-1-:8];
		 parallel_index+=1;
	 end
  end

  always @(parallel_index_minus_1) begin
	  case(link_state)
		  LINK_DOWN:
		  	if(parallel_data[parallel_index_minus_1]=='h55)
                             link_state <= AM1;
		  AM1:
		  	if(parallel_data[parallel_index_minus_1]=='h55)
                             link_state <= AM2;
		        else
                             link_state <= LINK_DOWN;
		  AM2:
		  	if(parallel_data[parallel_index_minus_1]=='h55)
                             link_state <= AM3;
		        else
                             link_state <= LINK_DOWN;
		  AM3:
		  	if(parallel_data[parallel_index_minus_1]=='h55)
                             link_state <= AM4;
		        else
                             link_state <= LINK_DOWN;
		  AM4:
		  	if(parallel_data[parallel_index_minus_1]=='h55)
                             link_state <= LINK_UP;
  		        else
                             link_state <= LINK_DOWN;
	     endcase
  end

  //DETECT COMMAND
  always @(parallel_index_minus_1) begin
	  case(packet_state)
		  IDLE:
		  	if(link_state == LINK_UP)
                             packet_state <= DETECT_SOP;
		  DETECT_SOP:
		  	if(parallel_data[parallel_index_minus_1]=='hfb)
                             packet_state <= DETECT_ID;
		  DETECT_ID:
		  	if(parallel_data[parallel_index_minus_1]==link_id)
                             packet_state <= DETECT_LENGTH;
		        else
                             packet_state <= DETECT_SOP;
		  DETECT_LENGTH:
                        packet_state <= DETECT_COMMAND;
		  DETECT_COMMAND:
			if(length_error)
                            packet_state <= PACKET_ERROR;
		        else if(length_counter == 1)
                            packet_state <= DETECT_EOP;
		        else begin
			    length_counter -=1;
                            packet_state <= DETECT_PAYLOAD;
			end
		  DETECT_PAYLOAD:
			if(length_counter == 1)
                            packet_state <= DETECT_EOP;
		        else begin
			    length_counter -=1;
                            packet_state <= DETECT_PAYLOAD;
			end
		  DETECT_EOP:
			if(parallel_data[parallel_index_minus_1]=='hfd) begin
                             packet_state <= DETECT_SOP;
			     led <= led_r;
		        end
		        else
                             packet_state <= PACKET_ERROR;
		  PACKET_ERROR:
                        packet_state <= DETECT_SOP;
	     endcase
  end

  always @(parallel_index_minus_1) begin
	  command_error = 0;
	  led_r = led;
	  if(packet_state == DETECT_LENGTH) begin
		  length = parallel_data[parallel_index_minus_1];
		  length_counter = length;
		  if(length == 0) begin
			  length_error = 1;
		  end
		  else begin
			  length_error = 0;
		  end
	  end
	  if(packet_state == DETECT_COMMAND) begin
		  if(parallel_data[parallel_index_minus_1]=='h00)
			  led_r[0] = 1;
		  else if(parallel_data[parallel_index_minus_1]=='h01)
			  led_r[0] = 0;
		  else if(parallel_data[parallel_index_minus_1]=='h10)
			  led_r[1] = 1;
		  else if(parallel_data[parallel_index_minus_1]=='h11)
			  led_r[1] = 0;
		  else
			  command_error=1;
	  end
  end

  always @(parallel_index_minus_1) begin
	  if(link_state == LINK_UP) begin
		  tx_serial <= #0 parallel_data[parallel_index_minus_1][0];
		  @(posedge clk);
		  tx_serial <= #0 parallel_data[parallel_index_minus_1][1];
		  @(posedge clk);
		  tx_serial <= #0 parallel_data[parallel_index_minus_1][2];
		  @(posedge clk);
		  tx_serial <= #0 parallel_data[parallel_index_minus_1][3];
		  @(posedge clk);
		  tx_serial <= #0 parallel_data[parallel_index_minus_1][4];
		  @(posedge clk);
		  tx_serial <= #0 parallel_data[parallel_index_minus_1][5];
		  @(posedge clk);
		  tx_serial <= #0 parallel_data[parallel_index_minus_1][6];
		  @(posedge clk);
		  tx_serial <= #0 parallel_data[parallel_index_minus_1][7];
	  end
	  else begin
		  tx_serial <= #0 0;
	  end
  end

endmodule