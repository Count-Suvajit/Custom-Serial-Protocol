
`include "uvm_macros.svh"
import uvm_pkg::*;

typedef class custom_serial_base_transaction;
interface custom_serial_interface
	( input clk,
          input rstn 
	);

        reg serial;

	string instance_name="";

	semaphore sem_if;

	bit monitor_first_time;

	initial begin
		sem_if = new(1);
	end

	clocking cb_driver@(posedge clk);
           output serial;
	endclocking

	clocking cb_monitor@(posedge clk);
           input serial;
	endclocking

	function string get_inst_name();
		return instance_name;
	endfunction

	//Make sure to make interface/module/programing block methods as
	//automatic to avoid one call affecting the previous call
        task automatic drive_idle();
		bit [7:0] idle = 'h07;
		//SV Streaming operator to reverse the bits, since foreach is
		//driving from MSB bit to LSB bit for packet arrays
		idle = {<<{idle}};

                sem_if.get(1);
		foreach(idle[i]) begin
		    @cb_driver;
		    cb_driver.serial <= idle[i];
	        end
                sem_if.put(1);
	endtask

        task automatic drive_am();
		bit [7:0] am = 'h55;
		int num_am = 5;
		//For TX and RX Monitor Synchronization 2 is used.
                int num_device = 2;

		am = {<<{am}};

                sem_if.get(1);
		repeat(num_am*num_device) begin
		     foreach(am[i]) begin
		         @cb_driver;
		         cb_driver.serial <= am[i];
	             end
	        end
                sem_if.put(1);
	endtask


	task automatic drive_trans(custom_serial_base_transaction t);

		bit [7:0] sop = 'hfb;
		bit [7:0] eop = 'hfd;
		//Don't use t.print() inside uvm_info. It doesn't return a string.
		//`uvm_info(get_inst_name(), $sformatf("Received Transaction = %p", t.print()), UVM_MEDIUM)
		//Instead use t.sprint() with format specified %s
		`uvm_info(get_inst_name(), $sformatf("Received Transaction : \n%s", t.sprint()), UVM_MEDIUM)
                
		sem_if.get(1);
		for(int i=0; i< $bits(sop); i++) begin
		         @cb_driver;
		         cb_driver.serial <= sop[i];
	        end
		for(int i=0; i< $bits(t.link_id); i++) begin
		         @cb_driver;
		         cb_driver.serial <= t.link_id[i];
	        end
		for(int i=0; i< $bits(t.length); i++) begin
		         @cb_driver;
		         cb_driver.serial <= t.length[i];
	        end
		foreach(t.payload[i]) begin
			for(int j=0; j< $bits(t.payload[i]); j++) begin
		            @cb_driver;
		            cb_driver.serial <= t.payload[i][j];
		        end
	        end
		for(int i=0; i< $bits(eop); i++) begin
		         @cb_driver;
		         cb_driver.serial <= eop[i];
	        end
                sem_if.put(1);

	endtask : drive_trans

        task automatic detect_am();
		bit serial_mon[$];
		bit [7:0] am = 'h55;
		int num_am = 5;
                int num_device = 1;
		bit am_detected=0;

		forever begin
		   @cb_monitor;
		   if(serial_mon.size == $bits(am)*num_am*num_device)
			   void'(serial_mon.pop_front());
		   serial_mon.push_back(cb_monitor.serial);
		   if(serial_mon.size == $bits(am)*num_am*num_device) begin
			   am_detected=1;
			   foreach(serial_mon[i]) begin
				   if(serial_mon[serial_mon.size-1-i] != am[i%$bits(am)])
					   am_detected = 0;
			   end
		   end
		   if(am_detected) begin
			   `uvm_info(get_inst_name(),"All AMs detected", UVM_MEDIUM)
			   break;
		   end
	        end
		monitor_first_time=1;

	endtask

        task automatic collect_trans(output custom_serial_base_transaction t);
		bit [7:0] serial_mon[$];
		bit [7:0] single_cycle_data;
		bit [7:0] sop = 'hfb;
		bit [7:0] eop = 'hfd;
		bit sop_detected;
		bit eop_detected;

		t = custom_serial_base_transaction::type_id::create("t");

		//Used to give 1 cycle gap b/w am & this task to WA coding bug
		if(monitor_first_time) begin
		   @cb_monitor;
		   monitor_first_time=0;
	        end
		forever begin
			for(int i=0; i< $bits(single_cycle_data); i+=1) begin
			    @cb_monitor;
			    single_cycle_data[i] = cb_monitor.serial;
			end
			if(single_cycle_data == sop) begin
				sop_detected = 1;
			end
			else if(single_cycle_data == eop) begin
				eop_detected = 1;
			end
			else if(sop_detected) begin
				serial_mon.push_back(single_cycle_data);
			end

			if(eop_detected) begin
			    if(serial_mon.size>0) begin
                                t.link_id = serial_mon.pop_front();
				if(serial_mon.size>0) begin
                                    t.length = serial_mon.pop_front();
			            if(serial_mon.size>0) begin
				        t.payload = new[serial_mon.size];
                                        foreach(serial_mon[i]) begin
						t.payload[i] = serial_mon[i];
				        end
			            end
			        end else begin
				    `uvm_error(get_inst_name(),"Invalid size transaction received")
			        end
			    end
			    else begin
				`uvm_error(get_inst_name(),"Received EOP without SOP")
			    end
			    break;
			end
	        end

	endtask

endinterface : custom_serial_interface

typedef enum {LED0_ON=8'h00,LED0_OFF=8'h01,LED1_ON=8'h10,LED1_OFF=8'h11} serial_command;

class custom_serial_base_transaction extends uvm_sequence_item;

        rand bit [7:0] link_id;
	rand bit [7:0] length;
	rand bit [7:0] payload[];
	rand bit [7:0] command;

	serial_command serial_command_set[];
	serial_command cmd;

        `uvm_object_utils(custom_serial_base_transaction)

	function new(string name="custom_serial_base_transaction_0");
              super.new(name);
	      serial_command_set = new[cmd.num];
	      cmd = cmd.first();
	      foreach(serial_command_set[i]) begin
                   serial_command_set[i] = cmd;
		   cmd = cmd.next;
	      end
	endfunction : new

	//soft keyword is used to override the constraint for error injection.
	//Error can also be injected by overriding the same constraint in
	//extended class. Also done in post_randomize() function
        constraint payload_size {
	    soft payload.size == length;
	}

	constraint min_length {
            soft length > 0;
	}

	constraint max_length {
            length < 10;
	}

	constraint command_set {
            payload.size > 0 -> command == payload[0];
	}

	constraint valid_command {
	    soft command inside {serial_command_set};
	}

	//It's recommended to not use factory macro utils to avoid compile/run time overhead issues.
	//Define your functions using do_*
	virtual function void do_print(uvm_printer printer);
	    string idx;
	    super.do_print(printer);

	    printer.print_field_int("ID",link_id,$bits(link_id),UVM_HEX);
	    printer.print_field_int("LENGTH",length,$bits(length),UVM_HEX);
	    foreach(payload[i]) begin
   	       idx.itoa(i);
	       printer.print_field_int({"PAYLOAD[",idx,"]"},payload[i],$bits(payload[i]),UVM_HEX);
            end

        endfunction : do_print

endclass : custom_serial_base_transaction

class custom_serial_length_error_transaction extends custom_serial_base_transaction;

        `uvm_object_utils(custom_serial_length_error_transaction)

	function new(string name="custom_serial_length_error_transaction_0");
              super.new(name);
	endfunction : new

	constraint payload_length_error {
              payload.size inside {length-1,[length+1:length+5]};
	}

endclass : custom_serial_length_error_transaction

class custom_serial_zero_length_transaction extends custom_serial_base_transaction;

        `uvm_object_utils(custom_serial_zero_length_transaction)

	function new(string name="custom_serial_zero_length_transaction_0");
              super.new(name);
	endfunction : new

	constraint payload_length {
	      length == 0;
              payload.size > 0;
              payload.size < 10;
	}

endclass : custom_serial_zero_length_transaction

class custom_serial_monitor extends uvm_monitor;

        `uvm_component_utils(custom_serial_monitor)

	uvm_analysis_port#(custom_serial_base_transaction) analysis_port;
        custom_serial_base_transaction t;
	virtual custom_serial_interface serial_vif;

	function new(string name, uvm_component parent);
              super.new(name,parent);
	endfunction : new

	virtual function void build_phase(uvm_phase phase);
	      super.build_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[BUILD PHASE] Start.."), UVM_NONE)

	      //Why analysis ports are created with new() instead of type_id::create?
              analysis_port = new("analysis_port",this);

              uvm_config_db#(virtual custom_serial_interface)::get(this,"","custom_serial_vif",serial_vif);

	      `uvm_info(get_type_name(),$sformatf("[BUILD PHASE] End.."), UVM_NONE)

        endfunction : build_phase

	virtual function void connect_phase(uvm_phase phase);
	      super.connect_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[CONNECT PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[CONNECT PHASE] End.."), UVM_NONE)

        endfunction : connect_phase

	virtual function void end_of_elaboration_phase(uvm_phase phase);
	      super.end_of_elaboration_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[END OF ELABORATION PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[END OF ELABORATION PHASE] End.."), UVM_NONE)

        endfunction : end_of_elaboration_phase

	virtual function void start_of_simulation_phase(uvm_phase phase);
	      super.start_of_simulation_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[START OF SIMULATION PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[START OF SIMULATION PHASE] End.."), UVM_NONE)

        endfunction : start_of_simulation_phase

        virtual task run_phase(uvm_phase phase);
	      super.run_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[RUN PHASE] Start.."), UVM_NONE)

	      serial_vif.detect_am();
	      forever begin
	          serial_vif.collect_trans(t);
                  analysis_port.write(t);
              end

	      `uvm_info(get_type_name(),$sformatf("[RUN PHASE] End.."), UVM_NONE)

        endtask : run_phase

	virtual function void extract_phase(uvm_phase phase);
	      super.extract_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[EXTRACT PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[EXTRACT PHASE] End.."), UVM_NONE)

        endfunction : extract_phase

	virtual function void check_phase(uvm_phase phase);
	      super.check_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[CHECK PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[CHECK PHASE] End.."), UVM_NONE)

        endfunction : check_phase

	virtual function void report_phase(uvm_phase phase);
	      super.report_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[REPORT PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[REPORT PHASE] End.."), UVM_NONE)

        endfunction : report_phase

	virtual function void final_phase(uvm_phase phase);
	      super.final_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[FINAL PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[FINAL PHASE] End.."), UVM_NONE)

        endfunction : final_phase

endclass : custom_serial_monitor

class custom_serial_driver extends uvm_driver#(custom_serial_base_transaction);

        `uvm_component_utils(custom_serial_driver)

        custom_serial_base_transaction t;
	custom_serial_base_transaction t_q[$];
	virtual custom_serial_interface serial_vif;

	function new(string name, uvm_component parent);
              super.new(name,parent);
	endfunction : new

	virtual function void build_phase(uvm_phase phase);
	      super.build_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[BUILD PHASE] Start.."), UVM_NONE)

              uvm_config_db#(virtual custom_serial_interface)::get(this,"","custom_serial_vif",serial_vif);

	      `uvm_info(get_type_name(),$sformatf("[BUILD PHASE] End.."), UVM_NONE)

        endfunction : build_phase

	virtual function void connect_phase(uvm_phase phase);
	      super.connect_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[CONNECT PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[CONNECT PHASE] End.."), UVM_NONE)

        endfunction : connect_phase

	virtual function void end_of_elaboration_phase(uvm_phase phase);
	      super.end_of_elaboration_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[END OF ELABORATION PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[END OF ELABORATION PHASE] End.."), UVM_NONE)

        endfunction : end_of_elaboration_phase

	virtual function void start_of_simulation_phase(uvm_phase phase);
	      super.start_of_simulation_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[START OF SIMULATION PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[START OF SIMULATION PHASE] End.."), UVM_NONE)

        endfunction : start_of_simulation_phase

        virtual task run_phase(uvm_phase phase);
	      super.run_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[RUN PHASE] Start.."), UVM_NONE)


	      wait_reset();
	      fork 
	      begin
                  forever begin
		      seq_item_port.get_next_item(t);
                      `uvm_info(get_type_name(),$sformatf("Received Transaction %s",t.sprint()), UVM_MEDIUM)
		      t_q.push_back(t);
		      seq_item_port.item_done();
	          end
              end
              begin
		      #(1ns * $urandom_range(10,5));
		      drive_am();
		      drive_trans();
              end
              begin
		      drive_idle();
              end
              join

	      `uvm_info(get_type_name(),$sformatf("[RUN PHASE] End.."), UVM_NONE)

        endtask : run_phase

	virtual function void extract_phase(uvm_phase phase);
	      super.extract_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[EXTRACT PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[EXTRACT PHASE] End.."), UVM_NONE)

        endfunction : extract_phase

	virtual function void check_phase(uvm_phase phase);
	      super.check_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[CHECK PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[CHECK PHASE] End.."), UVM_NONE)

        endfunction : check_phase

	virtual function void report_phase(uvm_phase phase);
	      super.report_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[REPORT PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[REPORT PHASE] End.."), UVM_NONE)

        endfunction : report_phase

	virtual function void final_phase(uvm_phase phase);
	      super.final_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[FINAL PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[FINAL PHASE] End.."), UVM_NONE)

        endfunction : final_phase

	virtual task wait_reset();
              wait(serial_vif.rstn==1);
        endtask : wait_reset

	virtual task drive_idle();
	      forever begin
                      serial_vif.drive_idle();
	      end
        endtask : drive_idle

	virtual task drive_am();
              serial_vif.drive_am();
        endtask : drive_am

	virtual task drive_trans();
	      
	      custom_serial_base_transaction trans;
	      forever begin
		      wait(t_q.size>0)
		      trans = t_q.pop_front();
		      serial_vif.drive_trans(trans);
	      end
	      
        endtask : drive_trans

endclass : custom_serial_driver

class custom_serial_sequence extends uvm_sequence#(custom_serial_base_transaction);

	// The uvm_sequence_base class provides the interfaces needed to create streams
	// of sequence items and/or other sequences.
	//
	// A sequence is executed by calling its <start> method, either directly
	// or invocation of any of the `uvm_do_* macros.
	//
	// Executing sequences via <start>:
	//
	// A sequence's <start> method has a ~parent_sequence~ argument that controls
	// whether <pre_do>, <mid_do>, and <post_do> are called *in the parent*
	// sequence. It also has a ~call_pre_post~ argument that controls whether its
	// <pre_body> and <post_body> methods are called.
	// In all cases, its <pre_start> and <post_start> methods are always called.
	//
	// When <start> is called directly, you can provide the appropriate arguments
	// according to your application.
	//
	// The sequence execution flow looks like this
	//
	// User code
	//
	//| sub_seq.randomize(...); // optional
	//| sub_seq.start(seqr, parent_seq, priority, call_pre_post)
	//|
	//
	// The following methods are called, in order
	//
	//|
	//|   sub_seq.pre_start()        (task)
	//|   sub_seq.pre_body()         (task)  if call_pre_post==1
	//|     parent_seq.pre_do(0)     (task)  if parent_sequence!=null
	//|     parent_seq.mid_do(this)  (func)  if parent_sequence!=null
	//|   sub_seq.body               (task)  YOUR STIMULUS CODE
	//|     parent_seq.post_do(this) (func)  if parent_sequence!=null
	//|   sub_seq.post_body()        (task)  if call_pre_post==1
	//|   sub_seq.post_start()       (task)
	//
	//
	// Executing sub-sequences via `uvm_do macros:
	//
	// A sequence can also be indirectly started as a child in the <body> of a
	// parent sequence. The child sequence's <start> method is called indirectly
	// by invoking any of the `uvm_do macros.
	// In these cases, <start> is called with
	// ~call_pre_post~ set to 0, preventing the started sequence's <pre_body> and
	// <post_body> methods from being called. During execution of the
	// child sequence, the parent's <pre_do>, <mid_do>, and <post_do> methods
	// are called.
	//
	// The sub-sequence execution flow looks like
	//
	// User code
	//
	//|
	//| `uvm_do_with_prior(seq_seq, { constraints }, priority)
	//|
	//
	// The following methods are called, in order
	//
	//|
	//|   sub_seq.pre_start()         (task)
	//|   parent_seq.pre_do(0)        (task)
	//|   parent_req.mid_do(sub_seq)  (func)
	//|     sub_seq.body()            (task)
	//|   parent_seq.post_do(sub_seq) (func)
	//|   sub_seq.post_start()        (task)
	//|
	//
	// Remember, it is the *parent* sequence's pre|mid|post_do that are called, not
	// the sequence being executed.
	//
	//
	// Executing sequence items via <start_item>/<finish_item> or `uvm_do macros:
	//
	// Items are started in the <body> of a parent sequence via calls to
	// <start_item>/<finish_item> or invocations of any of the `uvm_do
	// macros. The <pre_do>, <mid_do>, and <post_do> methods of the parent
	// sequence will be called as the item is executed.
	//
	// The sequence-item execution flow looks like
	//
	// User code
	//
	//| parent_seq.start_item(item, priority);
	//| item.randomize(...) [with {constraints}];
	//| parent_seq.finish_item(item);
	//|
	//| or
	//|
	//| `uvm_do_with_prior(item, constraints, priority)
	//|
	//
	// The following methods are called, in order
	//
	//|
	//|   sequencer.wait_for_grant(prior) (task) \ start_item  \
	//|   parent_seq.pre_do(1)            (task) /              \
	//|                                                      `uvm_do* macros
	//|   parent_seq.mid_do(item)         (func) \              /
	//|   sequencer.send_request(item)    (func)  \finish_item /
	//|   sequencer.wait_for_item_done()  (task)  /
	//|   parent_seq.post_do(item)        (func) /
	//
	// Attempting to execute a sequence via <start_item>/<finish_item>
	// will produce a run-time error.

        `uvm_object_utils(custom_serial_sequence)

        custom_serial_base_transaction t;
	bit [7:0] link_id;
	uvm_cmdline_processor uvm_cmdline_processor_t;
	string num_packets_s;
	int num_packets;
	int scb_exp_packet_count;

	//UVM Objects do not have parent 
	function new(string name="custom_serial_sequence_0");
              super.new(name);
	      set_automatic_phase_objection(1);
	      uvm_cmdline_processor_t = uvm_cmdline_processor::get_inst();
	      if(uvm_cmdline_processor_t.get_arg_value("+num_packets=",num_packets_s))
	             num_packets = num_packets_s.atoi();
	      else
		     num_packets = 5;
	endfunction : new

	// This task is a user-definable callback that is called before the
	// optional execution of <pre_body>.
	// This method should not be called directly by the user.
	virtual task pre_start();
	      super.pre_start();

	      `uvm_info(get_type_name(),$sformatf("[PRE START] Start.."), UVM_NONE)

	      if(!uvm_config_db#(int)::get(m_sequencer,"","NUM_PACKETS_TX",scb_exp_packet_count)) begin
		      `uvm_fatal(get_type_name(),"Unable to get value of NUM_PACKETS_TX")
	      end
	      fork begin
		      forever begin
			      uvm_config_db#(int)::wait_modified(m_sequencer,"","NUM_PACKETS_TX");
			      uvm_config_db#(int)::get(m_sequencer,"","NUM_PACKETS_TX",scb_exp_packet_count);
		      end
	      end
              join_none

	      `uvm_info(get_type_name(),$sformatf("[PRE START] End.."), UVM_NONE)

        endtask : pre_start

	// This task is a user-definable callback that is called before the
	// execution of <body> ~only~ when the sequence is started with <start>.
	// If <start> is called with ~call_pre_post~ set to 0, ~pre_body~ is not
	// called.
	// This method should not be called directly by the user.	
	virtual task pre_body();
	      super.pre_body();

	      `uvm_info(get_type_name(),$sformatf("[PRE BODY] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[PRE BODY] End.."), UVM_NONE)

        endtask : pre_body

	// This task is a user-definable callback task that is called ~on the
	// parent sequence~, if any
	// sequence has issued a wait_for_grant() call and after the sequencer has
	// selected this sequence, and before the item is randomized.
	//
	// Although pre_do is a task, consuming simulation cycles may result in
	// unexpected behavior on the driver.
	//
	// This method should not be called directly by the user.	
	virtual task pre_do(bit is_item);
	      super.pre_do(is_item);

	      `uvm_info(get_type_name(),$sformatf("[PRE DO] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[PRE DO] End.."), UVM_NONE)

        endtask : pre_do

	// This function is a user-definable callback function that is called after
	// the sequence item has been randomized, and just before the item is sent
	// to the driver.  This method should not be called directly by the user.	
	virtual function void mid_do(uvm_sequence_item this_item);
	      super.mid_do(this_item);

	      `uvm_info(get_type_name(),$sformatf("[MID DO] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[MID DO] End.."), UVM_NONE)

        endfunction : mid_do

	// This is the user-defined task where the main sequence code resides.
	// This method should not be called directly by the user.	
	virtual task body();
	      super.body();

	      `uvm_info(get_type_name(),$sformatf("[BODY] Start.."), UVM_NONE)

	      uvm_config_db#(int)::get(m_sequencer,"","link_id",link_id);
	      repeat(num_packets) begin
	         t = custom_serial_base_transaction::type_id::create("t");
                 start_item(t,-1,m_sequencer);
	         if(!t.randomize() with {link_id == local::link_id;})
	            `uvm_error(get_type_name(),"Failed to randomize. Please check")
	         finish_item(t);
              end

	      `uvm_info(get_type_name(),$sformatf("[BODY] End.."), UVM_NONE)

        endtask : body

	// This function is a user-definable callback function that is called after
	// the driver has indicated that it has completed the item, using either
	// this item_done or put methods. This method should not be called directly
	// by the user.	
	virtual function void post_do(uvm_sequence_item this_item);
	      super.post_do(this_item);

	      `uvm_info(get_type_name(),$sformatf("[POST DO] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[POST DO] End.."), UVM_NONE)

        endfunction : post_do
	
	// This task is a user-definable callback task that is called after the
	// execution of <body> ~only~ when the sequence is started with <start>.
	// If <start> is called with ~call_pre_post~ set to 0, ~post_body~ is not
	// called.
	// This task is a user-definable callback task that is called after the
	// execution of the body, unless the sequence is started with call_pre_post=0.
	// This method should not be called directly by the user.	
	virtual task post_body();
	      super.post_body();

	      `uvm_info(get_type_name(),$sformatf("[POST BODY] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[POST BODY] End.."), UVM_NONE)

        endtask : post_body

	// This task is a user-definable callback that is called after the
	// optional execution of <post_body>.
	// This method should not be called directly by the user.	
	virtual task post_start();
	      super.post_start();

	      `uvm_info(get_type_name(),$sformatf("[POST START] Start.."), UVM_NONE)

	      fork begin
		      //Since disable_fork is used, to prevent if from killing
		      //other forks, this is kept inside an outer fork
		      fork
		      begin
			      wait(scb_exp_packet_count >= num_packets);
		      end
		      begin
			      #(500ns * num_packets);
			      `uvm_error(get_type_name(),$sformatf("Timeout waiting for packets. Expected : 'h%0h, Actual : 'h%0h",num_packets,scb_exp_packet_count))
		      end
		      join_any
		      disable fork;
              end
              join

	      `uvm_info(get_type_name(),$sformatf("[POST START] End.."), UVM_NONE)

        endtask : post_start

endclass : custom_serial_sequence

class custom_serial_sequencer extends uvm_sequencer#(custom_serial_base_transaction);

        `uvm_component_utils(custom_serial_sequencer)

	function new(string name, uvm_component parent);
              super.new(name,parent);
	endfunction : new

	virtual function void build_phase(uvm_phase phase);
	      super.build_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[BUILD PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[BUILD PHASE] End.."), UVM_NONE)

        endfunction : build_phase

	virtual function void connect_phase(uvm_phase phase);
	      super.connect_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[CONNECT PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[CONNECT PHASE] End.."), UVM_NONE)

        endfunction : connect_phase

        virtual task run_phase(uvm_phase phase);
	      super.run_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[RUN PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[RUN PHASE] End.."), UVM_NONE)

        endtask : run_phase

endclass : custom_serial_sequencer

class custom_serial_agent extends uvm_agent;

        `uvm_component_utils(custom_serial_agent)

        custom_serial_driver drv;
	custom_serial_monitor mon;
	custom_serial_sequencer sqr;
	bit is_active;

	function new(string name, uvm_component parent);
              super.new(name,parent);
	endfunction : new

	virtual function void build_phase(uvm_phase phase);
	      super.build_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[BUILD PHASE] Start.."), UVM_NONE)

	      if(is_active) begin
                  drv = custom_serial_driver::type_id::create("drv",this);
                  sqr = custom_serial_sequencer::type_id::create("sqr",this);
              end
              mon = custom_serial_monitor::type_id::create("mon",this);

	      `uvm_info(get_type_name(),$sformatf("[BUILD PHASE] End.."), UVM_NONE)

        endfunction : build_phase

	virtual function void connect_phase(uvm_phase phase);
	      super.connect_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[CONNECT PHASE] Start.."), UVM_NONE)

	      if(is_active) begin
                  drv.seq_item_port.connect(sqr.seq_item_export);
              end

	      `uvm_info(get_type_name(),$sformatf("[CONNECT PHASE] End.."), UVM_NONE)

        endfunction : connect_phase

	virtual function void end_of_elaboration_phase(uvm_phase phase);
	      super.end_of_elaboration_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[END OF ELABORATION PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[END OF ELABORATION PHASE] End.."), UVM_NONE)

        endfunction : end_of_elaboration_phase

	virtual function void start_of_simulation_phase(uvm_phase phase);
	      super.start_of_simulation_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[START OF SIMULATION PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[START OF SIMULATION PHASE] End.."), UVM_NONE)

        endfunction : start_of_simulation_phase

        virtual task reset_phase(uvm_phase phase);
	      super.reset_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[RESET PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[RESET PHASE] End.."), UVM_NONE)

        endtask : reset_phase
	
        virtual task configure_phase(uvm_phase phase);
	      super.configure_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[CONFUGURE PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[CONFIGURE PHASE] End.."), UVM_NONE)

        endtask : configure_phase
	
        virtual task main_phase(uvm_phase phase);
	      super.main_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[MAIN PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[MAIN PHASE] End.."), UVM_NONE)

        endtask : main_phase
	
        virtual task shutdown_phase(uvm_phase phase);
	      super.shutdown_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[SHUTDOWN PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[SHUTDOWN PHASE] End.."), UVM_NONE)

        endtask : shutdown_phase

	virtual function void extract_phase(uvm_phase phase);
	      super.extract_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[EXTRACT PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[EXTRACT PHASE] End.."), UVM_NONE)

        endfunction : extract_phase

	virtual function void check_phase(uvm_phase phase);
	      super.check_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[CHECK PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[CHECK PHASE] End.."), UVM_NONE)

        endfunction : check_phase

	virtual function void report_phase(uvm_phase phase);
	      super.report_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[REPORT PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[REPORT PHASE] End.."), UVM_NONE)

        endfunction : report_phase

	virtual function void final_phase(uvm_phase phase);
	      super.final_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[FINAL PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[FINAL PHASE] End.."), UVM_NONE)

        endfunction : final_phase

endclass : custom_serial_agent

class custom_serial_scoreboard extends uvm_scoreboard;

	`uvm_analysis_imp_decl(_tx)
	`uvm_analysis_imp_decl(_rx)

        `uvm_component_utils(custom_serial_scoreboard)

	uvm_analysis_imp_tx#(custom_serial_base_transaction,custom_serial_scoreboard) analysis_imp_tx;
	uvm_analysis_imp_rx#(custom_serial_base_transaction,custom_serial_scoreboard) analysis_imp_rx;

        custom_serial_base_transaction q_rx[$];
        custom_serial_base_transaction q_tx[$];

	int num_passing;
	int num_failing;
	int num_compared;
	int num_received_tx;
	int num_received_rx;

	function new(string name, uvm_component parent);
              super.new(name,parent);
	endfunction : new

	virtual function void build_phase(uvm_phase phase);
	      super.build_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[BUILD PHASE] Start.."), UVM_NONE)

              analysis_imp_tx = new("analysis_imp_tx",this);

	      uvm_config_db#(int)::set(this.get_parent(),"agt0.sqr","NUM_PACKETS_TX",num_received_tx);
	      uvm_config_db#(int)::set(this.get_parent(),"agt0.sqr","NUM_PACKETS_TX",num_received_rx);

              analysis_imp_rx = new("analysis_imp_rx",this);

	      `uvm_info(get_type_name(),$sformatf("[BUILD PHASE] End.."), UVM_NONE)

        endfunction : build_phase

	virtual function void connect_phase(uvm_phase phase);
	      super.connect_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[CONNECT PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[CONNECT PHASE] End.."), UVM_NONE)

        endfunction : connect_phase

	virtual function void end_of_elaboration_phase(uvm_phase phase);
	      super.end_of_elaboration_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[END OF ELABORATION PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[END OF ELABORATION PHASE] End.."), UVM_NONE)

        endfunction : end_of_elaboration_phase

	virtual function void start_of_simulation_phase(uvm_phase phase);
	      super.start_of_simulation_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[START OF SIMULATION PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[START OF SIMULATION PHASE] End.."), UVM_NONE)

        endfunction : start_of_simulation_phase

        virtual task run_phase(uvm_phase phase);
	      custom_serial_base_transaction t_tx;
	      custom_serial_base_transaction t_rx;
	      semaphore sem = new(2);
	      super.run_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[RUN PHASE] Start.."), UVM_NONE)

	      sem.get(2);
	      //In-Order comparison
	      forever begin
                 fork
		 begin
			 wait(q_tx.size > 0);
			 t_tx = q_tx.pop_front();
			 sem.put(1);
		 end
	         begin
			 wait(q_rx.size > 0);
			 t_rx = q_rx.pop_front();
			 sem.put(1);
	         end
	         begin
                         sem.get(2);
			 compare(t_tx,t_rx);
	         end
	         join
	      end

	      `uvm_info(get_type_name(),$sformatf("[RUN PHASE] End.."), UVM_NONE)

        endtask : run_phase

	virtual function void compare(custom_serial_base_transaction t_tx,t_rx);

	        bit passing = 1;
		`uvm_info(get_type_name(),$sformatf("Expected Transaction : %s", t_tx.sprint()), UVM_MEDIUM)
		`uvm_info(get_type_name(),$sformatf("Actual Transaction : %s", t_rx.sprint()), UVM_MEDIUM)
		if(t_tx.link_id == t_rx.link_id) begin
		    `uvm_info(get_type_name(),$sformatf("ID is matching : 'h%0h", t_tx.link_id), UVM_MEDIUM)
		end
		else begin
		    passing = 0;
		    `uvm_error(get_type_name(),$sformatf("ID mismatching : Expected : 'h%0h, Actual : 'h%0h", t_tx.link_id,t_rx.link_id))
		end
		if(t_tx.length == t_rx.length) begin
		    `uvm_info(get_type_name(),$sformatf("Length is matching : 'h%0h", t_tx.length), UVM_MEDIUM)
		end
		else begin
		    passing = 0;
		    `uvm_error(get_type_name(),$sformatf("Length mismatching : Expected : 'h%0h, Actual : 'h%0h", t_tx.length,t_rx.length))
		end
		if(t_tx.command == t_rx.command) begin
		    `uvm_info(get_type_name(),$sformatf("Command is matching : 'h%0h", t_tx.command), UVM_MEDIUM)
		end
		else begin
		    passing = 0;
		    `uvm_error(get_type_name(),$sformatf("Command mismatching : Expected : 'h%0h, Actual : 'h%0h", t_tx.command,t_rx.command))
		end
		if(t_tx.payload.size == t_rx.payload.size) begin

			foreach(t_tx.payload[i]) begin
				if(t_tx.payload[i] == t_rx.payload[i]) begin
		                    `uvm_info(get_type_name(),$sformatf("payload[%0d] is matching : 'h%0h", i,t_tx.payload[i]), UVM_MEDIUM)
				end
				else begin
		                    passing = 0;
		                    `uvm_error(get_type_name(),$sformatf("payload[%0d] mismatching Expected : 'h%0h, Actual : 'h%0h", i,t_tx.payload[i],t_rx.payload[i]))
				end
			end
		end
		else begin
		    passing = 0;
		    `uvm_error(get_type_name(),$sformatf("payload size mismatching : Expected : 'h%0h, Actual : 'h%0h", t_tx.payload.size,t_rx.payload.size))
		end

		if(passing)
			num_passing+=1;
		else
			num_failing+=1;
		num_compared+=1;

        endfunction

	virtual function void extract_phase(uvm_phase phase);
	      super.extract_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[EXTRACT PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[EXTRACT PHASE] End.."), UVM_NONE)

        endfunction : extract_phase

	virtual function void check_phase(uvm_phase phase);
	      super.check_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[CHECK PHASE] Start.."), UVM_NONE)

	      if(q_tx.size != 0 || q_rx.size != 0)
		      `uvm_error(get_type_name(),$sformatf("Scoreboard Queues are not clear. q_tx size = 'h%0h, q_rx size = 'h%0h",q_tx.size,q_rx.size))
	      if(num_compared == 0)
		      `uvm_error(get_type_name(),$sformatf("Scoreboard didn't compare any packets"))

	      `uvm_info(get_type_name(),$sformatf("[CHECK PHASE] End.."), UVM_NONE)

        endfunction : check_phase

	virtual function void report_phase(uvm_phase phase);
	      super.report_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[REPORT PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("================================SCOREBOARD REPORT BEGIN==============================="), UVM_NONE)

	      `uvm_info(get_type_name(),$sformatf("NUM COMPARED : 'h%0h, NUM PASSING : 'h%0h, NUM FAILING : 'h%0h", num_compared,num_passing,num_failing), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("================================SCOREBOARD REPORT END==============================="), UVM_NONE)

	      `uvm_info(get_type_name(),$sformatf("[REPORT PHASE] End.."), UVM_NONE)

        endfunction : report_phase

	virtual function void final_phase(uvm_phase phase);
	      super.final_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[FINAL PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[FINAL PHASE] End.."), UVM_NONE)

        endfunction : final_phase

        virtual function void write_tx(custom_serial_base_transaction t);
		q_tx.push_back(t);
		num_received_tx+=1;
		uvm_config_db#(int)::set(this.get_parent,"agt0.sqr","NUM_PACKETS_TX",num_received_tx);
        endfunction : write_tx

        virtual function void write_rx(custom_serial_base_transaction t);
		q_rx.push_back(t);
		num_received_rx+=1;
		uvm_config_db#(int)::set(this.get_parent,"agt0.sqr","NUM_PACKETS_RX",num_received_rx);
        endfunction : write_rx

endclass : custom_serial_scoreboard

class custom_serial_env extends uvm_env;

        `uvm_component_utils(custom_serial_env)

        custom_serial_agent agt0;
        custom_serial_agent agt1;
	custom_serial_scoreboard scb;
	virtual custom_serial_interface serial_vif0;	
	virtual custom_serial_interface serial_vif1;
	bit [7:0] link_id;

	function new(string name, uvm_component parent);
              super.new(name,parent);
	endfunction : new

	virtual function void build_phase(uvm_phase phase);
	      super.build_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[BUILD PHASE] Start.."), UVM_NONE)
              
	      agt0 = custom_serial_agent::type_id::create("agt0",this);
	      agt1 = custom_serial_agent::type_id::create("agt1",this);
	      scb = custom_serial_scoreboard::type_id::create("scb",this);

	      uvm_config_db#(virtual custom_serial_interface)::get(this,"","custom_serial_vif0",serial_vif0);
	      uvm_config_db#(virtual custom_serial_interface)::set(this,"agt0.*","custom_serial_vif",serial_vif0);

	      uvm_config_db#(virtual custom_serial_interface)::get(this,"","custom_serial_vif1",serial_vif1);
	      uvm_config_db#(virtual custom_serial_interface)::set(this,"agt1.*","custom_serial_vif",serial_vif1);

	      uvm_config_db#(int)::get(this,"","link_id",link_id);
	      uvm_config_db#(int)::set(this,"agt0.sqr","link_id",link_id);

	      agt0.is_active = 1;
	      agt1.is_active = 0;

	      `uvm_info(get_type_name(),$sformatf("[BUILD PHASE] End.."), UVM_NONE)

        endfunction : build_phase

	virtual function void connect_phase(uvm_phase phase);
	      super.connect_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[CONNECT PHASE] Start.."), UVM_NONE)

              agt0.mon.analysis_port.connect(scb.analysis_imp_tx);
              agt1.mon.analysis_port.connect(scb.analysis_imp_rx);

	      `uvm_info(get_type_name(),$sformatf("[CONNECT PHASE] End.."), UVM_NONE)

        endfunction : connect_phase

	virtual function void end_of_elaboration_phase(uvm_phase phase);
	      super.end_of_elaboration_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[END OF ELABORATION PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[END OF ELABORATION PHASE] End.."), UVM_NONE)

        endfunction : end_of_elaboration_phase

	virtual function void start_of_simulation_phase(uvm_phase phase);
	      super.start_of_simulation_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[START OF SIMULATION PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[START OF SIMULATION PHASE] End.."), UVM_NONE)

        endfunction : start_of_simulation_phase

        virtual task reset_phase(uvm_phase phase);
	      super.reset_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[RESET PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[RESET PHASE] End.."), UVM_NONE)

        endtask : reset_phase
	
        virtual task configure_phase(uvm_phase phase);
	      super.configure_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[CONFUGURE PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[CONFIGURE PHASE] End.."), UVM_NONE)

        endtask : configure_phase
	
        virtual task main_phase(uvm_phase phase);
	      super.main_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[MAIN PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[MAIN PHASE] End.."), UVM_NONE)

        endtask : main_phase
	
        virtual task shutdown_phase(uvm_phase phase);
	      super.shutdown_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[SHUTDOWN PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[SHUTDOWN PHASE] End.."), UVM_NONE)

        endtask : shutdown_phase

	virtual function void extract_phase(uvm_phase phase);
	      super.extract_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[EXTRACT PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[EXTRACT PHASE] End.."), UVM_NONE)

        endfunction : extract_phase

	virtual function void check_phase(uvm_phase phase);
	      super.check_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[CHECK PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[CHECK PHASE] End.."), UVM_NONE)

        endfunction : check_phase

	virtual function void report_phase(uvm_phase phase);
	      super.report_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[REPORT PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[REPORT PHASE] End.."), UVM_NONE)

        endfunction : report_phase

	virtual function void final_phase(uvm_phase phase);
	      super.final_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[FINAL PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[FINAL PHASE] End.."), UVM_NONE)

        endfunction : final_phase

endclass : custom_serial_env

//Fixing circular dependancy
typedef class custom_serial_top_virtual_sequencer;
class custom_serial_top_virtual_sequence extends uvm_sequence;

        `uvm_object_utils(custom_serial_top_virtual_sequence)

	`uvm_declare_p_sequencer(custom_serial_top_virtual_sequencer)

        custom_serial_sequence seq0;
        custom_serial_sequence seq1;

	//This error comes if a default value is not passed in name argument for uvm_object classes.
        //Too few arguments to function/task call. The above function/task
	//call is not done with sufficient arguments.
	
	function new(string name="custom_serial_top_virtual_sequence_0");
              super.new(name);
	      set_automatic_phase_objection(1);
	endfunction : new

	virtual task pre_start();
	      super.pre_start();

	      `uvm_info(get_type_name(),$sformatf("[PRE START] Start.."), UVM_NONE)
              seq0 = custom_serial_sequence::type_id::create("seq0");	      
              seq1 = custom_serial_sequence::type_id::create("seq1");
	      `uvm_info(get_type_name(),$sformatf("[PRE START] End.."), UVM_NONE)

        endtask : pre_start

	virtual task pre_body();
	      super.pre_body();

	      `uvm_info(get_type_name(),$sformatf("[PRE BODY] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[PRE BODY] End.."), UVM_NONE)

        endtask : pre_body

	virtual task pre_do(bit is_item);
	      super.pre_do(is_item);

	      `uvm_info(get_type_name(),$sformatf("[PRE DO] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[PRE DO] End.."), UVM_NONE)

        endtask : pre_do

	virtual function void mid_do(uvm_sequence_item this_item);
	      super.mid_do(this_item);

	      `uvm_info(get_type_name(),$sformatf("[MID DO] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[MID DO] End.."), UVM_NONE)

        endfunction : mid_do

	virtual task body();
	      int val;
	      super.body();

	      `uvm_info(get_type_name(),$sformatf("[BODY] Start.."), UVM_NONE)

	      //For TEST : BEGIN 
              if(!uvm_config_db#(int)::get(m_sequencer,"","VAL",val))
		      `uvm_fatal(get_type_name(),"Unable to get value")
	      else
		      `uvm_info(get_type_name(),$sformatf("[config_db] Value=%0h",val), UVM_MEDIUM)

              //if(!uvm_resource_db#(int)::read_by_name("uvm_test_top.top_virtual_seqr","VAL",val))
              if(!uvm_resource_db#(int)::read_by_name(m_sequencer.get_full_name(),"VAL",val))
		      `uvm_fatal(get_type_name(),"Unable to get value")
	      else
                      `uvm_info(get_type_name(),$sformatf("[resource db] Value=%0h",val), UVM_MEDIUM)
              #2;
	      //END
	      

	      fork
	      begin
                  seq0.start(p_sequencer.top_env.env0.agt0.sqr);
              end
              begin
                  seq1.start(p_sequencer.top_env.env1.agt0.sqr);
              end
              join

	      `uvm_info(get_type_name(),$sformatf("[BODY] End.."), UVM_NONE)

        endtask : body

	virtual function void post_do(uvm_sequence_item this_item);
	      super.post_do(this_item);

	      `uvm_info(get_type_name(),$sformatf("[POST DO] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[POST DO] End.."), UVM_NONE)

        endfunction : post_do

	virtual task post_body();
	      super.post_body();

	      `uvm_info(get_type_name(),$sformatf("[POST BODY] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[POST BODY] End.."), UVM_NONE)

        endtask : post_body

	virtual task post_start();
	      super.post_start();

	      `uvm_info(get_type_name(),$sformatf("[POST START] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[POST START] End.."), UVM_NONE)

        endtask : post_start

endclass : custom_serial_top_virtual_sequence

//Fixing circular dependancy using forward reference
typedef class custom_serial_top_env;
class custom_serial_top_virtual_sequencer extends uvm_sequencer;

        `uvm_component_utils(custom_serial_top_virtual_sequencer)

        int unsigned val;

        custom_serial_top_env top_env;	

	function new(string name, uvm_component parent);
              super.new(name,parent);
	endfunction : new

	virtual function void build_phase(uvm_phase phase);
	      super.build_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[BUILD PHASE] Start.."), UVM_NONE)

              if(!uvm_config_db#(int)::get(this,"","VAL",val))
		      `uvm_fatal(get_type_name(),"Unable to get value")
	      else
		      `uvm_info(get_type_name(),$sformatf("Value=%0h",val), UVM_MEDIUM)

	      `uvm_info(get_type_name(),$sformatf("[BUILD PHASE] End.."), UVM_NONE)

        endfunction : build_phase

	virtual function void connect_phase(uvm_phase phase);
	      super.connect_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[CONNECT PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[CONNECT PHASE] End.."), UVM_NONE)

        endfunction : connect_phase

        virtual task run_phase(uvm_phase phase);
	      super.run_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[RUN PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[RUN PHASE] End.."), UVM_NONE)

        endtask : run_phase

endclass : custom_serial_top_virtual_sequencer

class custom_serial_top_env extends uvm_env;

        `uvm_component_utils(custom_serial_top_env)

        custom_serial_env env0;
        custom_serial_env env1;

	function new(string name, uvm_component parent);
              super.new(name,parent);
	endfunction : new

	virtual function void build_phase(uvm_phase phase);
	      super.build_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[BUILD PHASE] Start.."), UVM_NONE)

	      env0 = custom_serial_env::type_id::create("env0",this);
	      env1 = custom_serial_env::type_id::create("env1",this);

	      `uvm_info(get_type_name(),$sformatf("[BUILD PHASE] End.."), UVM_NONE)

        endfunction : build_phase

	virtual function void connect_phase(uvm_phase phase);
	      super.connect_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[CONNECT PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[CONNECT PHASE] End.."), UVM_NONE)

        endfunction : connect_phase

	virtual function void end_of_elaboration_phase(uvm_phase phase);
	      super.end_of_elaboration_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[END OF ELABORATION PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[END OF ELABORATION PHASE] End.."), UVM_NONE)

        endfunction : end_of_elaboration_phase

	virtual function void start_of_simulation_phase(uvm_phase phase);
	      super.start_of_simulation_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[START OF SIMULATION PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[START OF SIMULATION PHASE] End.."), UVM_NONE)

        endfunction : start_of_simulation_phase

        virtual task reset_phase(uvm_phase phase);
	      super.reset_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[RESET PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[RESET PHASE] End.."), UVM_NONE)

        endtask : reset_phase
	
        virtual task configure_phase(uvm_phase phase);
	      super.configure_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[CONFUGURE PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[CONFIGURE PHASE] End.."), UVM_NONE)

        endtask : configure_phase
	
        virtual task main_phase(uvm_phase phase);
	      super.main_phase(phase);

	      //Pass the object and the number of objection. If number of
	      //objections are not passed, by default 1 objection will be raised.
	      //If you don't raise objection then the phase will start but
	      //factory will call the next phase even before this phase ends.
	      phase.raise_objection(this);
	      `uvm_info(get_type_name(),$sformatf("[MAIN PHASE] Start.."), UVM_NONE)

              #15;

	      `uvm_info(get_type_name(),$sformatf("[MAIN PHASE] End.."), UVM_NONE)
	      phase.drop_objection(this);

        endtask : main_phase
	
        virtual task shutdown_phase(uvm_phase phase);
	      super.shutdown_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[SHUTDOWN PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[SHUTDOWN PHASE] End.."), UVM_NONE)

        endtask : shutdown_phase

	virtual function void extract_phase(uvm_phase phase);
	      super.extract_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[EXTRACT PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[EXTRACT PHASE] End.."), UVM_NONE)

        endfunction : extract_phase

	virtual function void check_phase(uvm_phase phase);
	      super.check_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[CHECK PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[CHECK PHASE] End.."), UVM_NONE)

        endfunction : check_phase

	virtual function void report_phase(uvm_phase phase);
	      super.report_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[REPORT PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[REPORT PHASE] End.."), UVM_NONE)

        endfunction : report_phase

	virtual function void final_phase(uvm_phase phase);
	      super.final_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[FINAL PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[FINAL PHASE] End.."), UVM_NONE)

        endfunction : final_phase

endclass : custom_serial_top_env

class custom_serial_test extends uvm_test;

        `uvm_component_utils(custom_serial_test)

	//Constructor of SV class can't be virtual and does not have a return
	//type, not even void. If this is an extended class from some base
	//class you need to add super.new() call in 1st line. If not done, the
	//compiler will add it by default. It;s recommended that constructor
	//to not contain anything other than super.new() call.
	//Since uvm_test is a uvm_component, hence one needs to pass the name
	//and parent arguments.
	function new(string name, uvm_component parent);
              super.new(name,parent); //Added by default by the compiler if you don't add it
	endfunction : new

	custom_serial_top_env top_env;
	custom_serial_top_virtual_sequence top_virtual_seq;
	custom_serial_top_virtual_sequencer top_virtual_seqr;
	uvm_factory factory;

        //BUILD-TIME PHASES

	//Always add functions as virtual to use method override of extended
	//class. Always add a super.<method> call for phase methods.
	//Use build phase for creating the env and other components in test
	//level. This is a top down method as the top level components needs to be
	//created. Notice that when you create component using new() call, you
	//need to pass the handle of the parent. Hence the parent needs to
	//exist before the sub components. Hence build phase is top down.
	virtual function void build_phase(uvm_phase phase);
	      super.build_phase(phase);

	      //VERBOSITY ENUMS = UVM_NONE=100, UVM_LOW=200, UVM_MEDIUM=300, UVM_HIGH=400, UVM_DEBUG=500
	      //Use $sformatf to modify string instead of $psprintf as the latter is not supported 
	      //in all compilers. Use <object>.sprint() to print objects.
	      `uvm_info(get_type_name(),$sformatf("[BUILD PHASE] Start.."), UVM_NONE)

	      factory = uvm_coreservice_t::get().get_factory();
	      factory.set_type_override_by_type(custom_serial_base_transaction::get_type(),custom_serial_length_error_transaction::get_type);

              top_env = custom_serial_top_env::type_id::create("top_env",this);
              top_virtual_seq = custom_serial_top_virtual_sequence::type_id::create("top_virtual_seq");
              top_virtual_seqr = custom_serial_top_virtual_sequencer::type_id::create("top_virtual_seqr",this);

	      `uvm_info(get_type_name(),$sformatf("[BUILD PHASE] End.."), UVM_NONE)

        endfunction : build_phase

	//Q. Why phase object is passed to the function?
	//Q. Why is this bottom-up phase?
	//Connect phase is botom-up phase. Here the components are connected
	//through ports.
	virtual function void connect_phase(uvm_phase phase);
	      super.connect_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[CONNECT PHASE] Start.."), UVM_NONE)

              top_virtual_seqr.top_env = top_env;

	      `uvm_info(get_type_name(),$sformatf("[CONNECT PHASE] End.."), UVM_NONE)

        endfunction : connect_phase

	virtual function void end_of_elaboration_phase(uvm_phase phase);
	      uvm_phase main_phase = phase.find_by_name("main",0);
	      main_phase.phase_done = main_phase.get_objection(); //UVM2.0 EDIT
	      super.end_of_elaboration_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[END OF ELABORATION PHASE] Start.."), UVM_NONE)

	      main_phase.phase_done.set_drain_time(this,200);
	      uvm_top.print_topology();

	      `uvm_info(get_type_name(),$sformatf("[END OF ELABORATION PHASE] End.."), UVM_NONE)

        endfunction : end_of_elaboration_phase

	virtual function void start_of_simulation_phase(uvm_phase phase);
	      super.start_of_simulation_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[START OF SIMULATION PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[START OF SIMULATION PHASE] End.."), UVM_NONE)

        endfunction : start_of_simulation_phase

	//RUN TIME TASK PHASES
	//These phases runs in parallel in all components. Example the test
	//and env reset phases will start together in some order.
	//
	//What if objections are not raised in env but raised in test component?
	//   =>When env phase completes before test phase task, then since test
	//   raised objection, the components will not move to next phase till
	//   the test(all components) have dropped the objection.
	//   =>When env phase requires more time to complete than the test,
	//   then when test phase completes and drops the objections, then all
	//   components will move to the next phase. Hence env will move to
	//   next phase method even before the previous phase method gets to
	//   complete. This will be an issue
	//   =>Conclusion : Raise objections from all components like
	//   test/env. Exceptions could be monitor/drivers where we normally
	//   don't raise objections as we don't know when to drop the
	//   objection. These components handle transactions and should not
	//   necessarily know when all the transactions have completed(to drop
	//   objection). Use set_automatic_raise_objection(1) from
	//   base_sequence new function to control transaction related phase
	//   objections.
	
        virtual task reset_phase(uvm_phase phase);
	      super.reset_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[RESET PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[RESET PHASE] End.."), UVM_NONE)

        endtask : reset_phase
	
        virtual task configure_phase(uvm_phase phase);
	      super.configure_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[CONFUGURE PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[CONFIGURE PHASE] End.."), UVM_NONE)

        endtask : configure_phase
	
        virtual task main_phase(uvm_phase phase);
	      super.main_phase(phase);

	      phase.raise_objection(this);
	      `uvm_info(get_type_name(),$sformatf("[MAIN PHASE] Start.."), UVM_NONE)
              
	      //top_virtual_seq.start(null);
	      top_virtual_seq.start(top_virtual_seqr);

	      phase.drop_objection(this);
	      `uvm_info(get_type_name(),$sformatf("[MAIN PHASE] End.."), UVM_NONE)

        endtask : main_phase
	
        virtual task shutdown_phase(uvm_phase phase);
	      super.shutdown_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[SHUTDOWN PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[SHUTDOWN PHASE] End.."), UVM_NONE)

        endtask : shutdown_phase


	//CLEANUP PHASES

	virtual function void extract_phase(uvm_phase phase);
	      super.extract_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[EXTRACT PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[EXTRACT PHASE] End.."), UVM_NONE)

        endfunction : extract_phase

	virtual function void check_phase(uvm_phase phase);
	      super.check_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[CHECK PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[CHECK PHASE] End.."), UVM_NONE)

        endfunction : check_phase

	virtual function void report_phase(uvm_phase phase);
	      super.report_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[REPORT PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[REPORT PHASE] End.."), UVM_NONE)

        endfunction : report_phase

	virtual function void final_phase(uvm_phase phase);
	      super.final_phase(phase);

	      `uvm_info(get_type_name(),$sformatf("[FINAL PHASE] Start.."), UVM_NONE)
	      `uvm_info(get_type_name(),$sformatf("[FINAL PHASE] End.."), UVM_NONE)

        endfunction : final_phase

endclass : custom_serial_test