
`define assert(condition, message) if(!(condition)) begin $display("ERROR at time %d: %s", $time, message); $finish(1); end


// pulse(n, ack) -->
//   send <n> TESTREQ pulses
//   stores the last ACK state in <ack>
//
//   e.g. n=3, ack is the output ack state
task pulse;
	input [7:0] n;
	begin
		//$display("pulse(%d)", n);
		repeat(n) begin
			testreq=1'b1;
			#PWID;
			testreq=1'b0;
			#PGAP;
		end
	end
endtask

// pulsebreak(n, ack) -->
//   send <n> TESTREQ pulses followed by a BREAK
//   stores the last ACK state in <ack>
//
//   e.g. n=3, ack is the output ack state
task pulsebreak;
	input [7:0] n;
	begin
		pulse(n);
		//$display("pulse break");
		#BREAK;
	end
endtask

// outbyte(n) --> OUTPUT phase 2, send bits
task outbyte;
	input [7:0] byte;

	// shift register
	reg [7:0] sr;

	begin
		sr = byte;

		// send 8 bits
		repeat(8) begin
			if (sr[7]) begin
				pulsebreak(1);	// SHIFT-ONE is one pulse
			end else begin
				pulsebreak(2);	// SHIFT-ZERO is two pulses
			end

			// shift the shift register left one bit
			sr = {sr[6:0], 1'b0};
		end
	end
endtask


task spi_txn;
	input have_byte;
	input have_buffer_space;
	input [7:0] input_byte;  // byte to send to target on next input request

	output sent_byte;
	output received_byte;
	output [7:0] output_byte;  // byte output by target

	reg [15:0] sr;

	begin
		sr = {have_byte, have_buffer_space, 6'b0, input_byte};
		$display("Test SPI: start with sr=%b", sr);

		spi_cs = 1'b0;
		spi_mosi = sr[15];
		#SPIGAP;
		repeat(16) begin
			spi_sck = 1'b1;
			sr = {sr[14:0], spi_miso};
			#SPIGAP;
			spi_mosi = sr[15];
			spi_sck = 1'b0;
			#SPIGAP;
		end
		spi_cs = 1'b1;
		#SPIGAP;  // Delay so back to back spi_txn calls don't merge into one

		sent_byte = have_byte && sr[14];  // we had a byte to send, and the remote had space
		received_byte = have_buffer_space && sr[15];  // remote had a byte to send, and we had space
		output_byte = sr[7:0];  // byte output by target / sent from DUT to us over SPI

		$display("Test SPI: have_byte %d have_buffer_space %d input_byte %02x -> sent_byte %d received_byte %d output_byte %02x",
			have_byte, have_buffer_space, input_byte, sent_byte, received_byte, output_byte);
	end
endtask

// Latch the most recent TESTACK state
reg lastAck;
initial lastAck = 1'b0;
always @(posedge testreq) begin
	#100 lastAck = (testack === 1'b1) ? 1'b1 : 1'b0;
end

// Track the number of TESTREQ pulses
integer reqcount;
initial reqcount = 0;
always @(posedge testreq) reqcount = reqcount + 1;


