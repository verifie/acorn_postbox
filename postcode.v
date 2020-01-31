// TODO reset some things when reset_in==1

module postcode

#(
	// Timer timeout value
	// Refclk is 2MHz, so this is 500ns increments
	parameter TIMER_MAX = (15*2)-1,

	// Set to 1 to just use txin[0], when we're acting as an SPI slave.
	// Set to 0 when driving an LCD.
	parameter SPI_ONLY = 0
)

(
	// Clock inputs
	input refclk,							// 2MHz reference clock
	
	// Test interface
	input  testreq,							// Test REQuest (LA23)
	output testack,						    // Test ACKnowledge (TESTAK)
	
	// Data received from target (OUTPUT)
	output [7:0] rxout,					    // Received data
	input  rxready,							// 1 if interface is ready for more data from target
	output spi_clock_out,                   // SPI SCK, where MOSI == rxout[0]
	output spi_cs_out,                      // SPI CS
	output reg last_transaction_was_input,  // 1 after bit 8 of an input, 0 after bit 8 of an output
	
	// Data to send to target
	input  [7:0] txin,						// Data -> target (INPUT command)
	input  tx_pending,						// D1 if interface has data for target
	output reg want_tx,                     // Target is requesting data
	output txstrobe							// Pulses high when a byte has just been sent

);

	// SPI clock output
	reg reset_drive_sck_for_input = 1'b0;
	reg drive_sck_for_input = 1'b0;
	reg spi_clock_pre = 1'b0;
	reg spi_clock_int = 1'b1;
	assign spi_clock_out = drive_sck_for_input ? testreq : spi_clock_int;

	// SPI chip select output
	reg spi_cs_int = 1'b1;
	assign spi_cs_out = spi_cs_int;

	// Gate internal TESTACK against TESTREQ.
	// TESTACK should only ever drive high or open.
	reg testack_int;
	assign testack = (testreq & testack_int);
	
	// Monostable -- pulse train end detector
	// Reset to zero every time TESTREQ pulses high.
	// Stops on expiry.
	reg[7:0] timer;
	reg timer_expired = 1'b0;
	reg timer_expired_d = 1'b0;  // Delayed timer_expired

	always @(posedge refclk or posedge testreq) begin
		if (testreq) begin
			// TESTREQ pulse resets the timer
			timer <= 8'b0;
		end else begin
			// REFCLK pulses increment the timer until it *passes*
			// TIMER_MAX, so we get a single refclk pulse on
			// timer_expired and timer_expired_d when this happens.
			if (timer <= TIMER_MAX) begin
				timer <= timer + 8'd1;
			end
		end
	end

	/**
	 * INPUT:
	 *   Four pulses are sent.
	 *   The fourth pulse is repeated until TESTACK is asserted in response.
	 *   The following eight pulses then clock in eight data bits, MSB first.
	 *   TESTACK asserted is interpreted as a logical '1'.
	 *   If pulses continue without a break, they should be interpreted as
	 *     further polling for input and more data may be transferred without
	 *     returning to the initial four-pulse start-up.
	 *
	 * OUTPUT:
	 *   Three pulses are sent.
	 *   If TESTACK is asserted in response to the third pulse, the interface
	 *     is ready for data.
	 *   A break then occurs, and either another attempt is made or data is sent.
	 *   Data is transmitted as an eight-group sequence of either one or two pulses.
	 *     One puls is interpreted as a logical '1', two is a logical '0'.
	 *   Each sequence of eight bits is preceded by a sequence of three-pulse poll
	 *     operations to ensure the interface is ready for data.
	 *   A dummy three-pulse sequence is sent at the end of a series of bytes to
	 *     ensure that the last byte is recognised.
	 */
	
	// Receive shift register -- data from the target ("output")
	reg[7:0] rxshift = 8'b0;
	// Set by rxready input, cleared when we receive a byte
	reg rx_ready = 1'b0;
	
	// Transmit shift register -- data to the target ("input")
	reg[7:0] txshift = 8'b0;
	wire tx_done;

	// Latched version of tx_pending that is used to start an input cycle.
	reg tx_ready = 1'b0;

	// state machine
	localparam S_INITIAL		= 5'd0;		// initial state, waiting for first pulse
	localparam S_SHIFTONE	= 5'd1;		// one pulse received (shift in a '1')
	localparam S_SHIFTZERO	= 5'd2;		// two pulses received (shift in a '0')
	localparam S_OUTPUTPOLL	= 5'd3;		// three pulses received (OUTPUT)
	localparam S_INPUTPOLL	= 5'd4;		// four pulses received (INPUT)
	localparam S_START_INPUT = 5'd13;
	localparam S_INPUT_BIT7	= 5'd5;		// shift out bits MSB..LSB in response to an INPUT command
	localparam S_INPUT_BIT6	= 5'd6;
	localparam S_INPUT_BIT5	= 5'd7;
	localparam S_INPUT_BIT4	= 5'd8;
	localparam S_INPUT_BIT3	= 5'd9;
	localparam S_INPUT_BIT2	= 5'd10;
	localparam S_INPUT_BIT1	= 5'd11;
	localparam S_INPUT_BIT0	= 5'd12;

	reg[4:0] state = S_INITIAL;

	assign rxout = rxshift;
	assign txstrobe = tx_done;

	assign tx_done = (state == S_INPUT_BIT0) & testreq;
	always @(posedge tx_done) begin
		//$display("posedge tx_done (=txstrobe)");
	end

	reg [3:0] rxcounter = 4'b1000;  // count of bits received

	always @(posedge refclk) begin
		// Delayed version of timer_expired
		timer_expired_d <= timer_expired;

		// timer_expired pulses high for one clock
		timer_expired <= 1'b0;

		// spi_clock_int pulses high for one clock, starting one clock after a bit shift
		spi_clock_int <= spi_clock_pre;
		spi_clock_pre <= 1'b0;

		if (timer == TIMER_MAX - 3) begin
			if (state == S_SHIFTONE) begin
				// Timer expiry in S_SHIFTONE shifts in a '1'
				rxshift <= {rxshift[6:0], 1'b1};
				rxcounter <= rxcounter + 4'b1;
				spi_clock_pre <= 1'b1;
				//$display("Receive shift 1 (rxshift %x rxcounter %d)", rxshift, rxcounter);
			end else if (state == S_SHIFTZERO) begin
				// Timer expiry in S_SHIFTONE shifts in a '0'
				rxshift <= {rxshift[6:0], 1'b0};
				rxcounter <= rxcounter + 4'b1;
				spi_clock_pre <= 1'b1;
				//$display("Receive shift 0 (rxshift %x rxcounter %d)", rxshift, rxcounter);
			end
		end

		if (timer == TIMER_MAX) begin
			// Timer expiry -- shift in any data bits which were sent
			timer_expired <= 1'b1;
			//$display("Timer expiry in state %d", state);

			if (state == S_OUTPUTPOLL) begin
				// Three pulses received; signal start of byte
				rxcounter <= 4'b0;
				//$display("Start receive output byte from target");
			end else begin
				// Otherwise rxshift maintains its current state
				rxshift <= rxshift;
			end
		end
	end

	// Async clear spi_cs_int
	reg clear_spi_cs_int = 1'b0;
	// Async reset spi_cs_int
	reg reset_spi_cs_int = 1'b0;

	// Generate spi_cs_int
	always @(posedge refclk or posedge clear_spi_cs_int) begin
		if (clear_spi_cs_int) begin
			// Enable spi_cs whenever we're about to do I/O
			spi_cs_int <= 1'b0;
		end else if (reset_spi_cs_int) begin
			// Disable spi_cs between I/Os
			spi_cs_int <= 1'b1;
			last_transaction_was_input <= 1'b1;
		end else begin
			// posedge refclk
			if (timer == TIMER_MAX) begin
				// Deactivate spi_cs_int at the end of an input or output byte
				if (rxcounter[3] == 1'b1) begin  // Finished receiving a byte from target (output cycle)
					spi_cs_int <= 1'b1;
					last_transaction_was_input <= 1'b0;
				end
				if (state != S_SHIFTONE && state != S_SHIFTZERO) begin  // Finished sending a byte to target (input cycle)
					spi_cs_int <= 1'b1;
					last_transaction_was_input <= 1'b1;
				end
			end
			if (timer == TIMER_MAX - 3) begin
				// Activate spi_cs_int at the start of an output byte
				if (state == S_SHIFTONE || state == S_SHIFTZERO) begin
					spi_cs_int <= 1'b0;
				end
			end
		end
	end

	// Generate rx_ready; the intention here is to allow the SPI remote to pulse
	// rxready to indicate that they can read one byte, or hold it high to
	// indicate that they require no flow control.
	always @(posedge refclk or posedge rxready) begin
		if (rxready) begin
			// When external rxready signal is high, set rx_ready active
			rx_ready <= 1'b1;
		end else begin
			if (timer == TIMER_MAX) begin
				if (rxcounter[3] == 1'b1) begin
					// Clear rx_ready when we finish receiving a byte from target
					rx_ready <= 1'b0;
				end
			end
		end
	end

	always @(posedge testreq or posedge timer_expired_d) begin

		if (timer_expired_d) begin
			// A timer expiry resets the state machine one EXTCLK after
			// timer_expired. This gives the shifter logic above a chance
			// to latch the incoming data bit.
			state <= S_INITIAL;
			want_tx <= 1'b0;
		end else begin
			//$display("testreq pulse; original state %d", state);
			// State specific logic
			case (state)
				S_INITIAL:		begin
										// Rising edge of first pulse
										
										// Ack the first pulse
										testack_int <= 1'b1;

										// If this is all, the target is sending us a one
										state <= S_SHIFTONE;
									end
									
				S_SHIFTONE:		begin
										// Rising edge of second pulse

										// Ack the second pulse
										testack_int <= 1'b1;

										// If this is all, the target is sending us a zero
										state <= S_SHIFTZERO;
									end
									
				S_SHIFTZERO:	begin
										// Rising edge of third pulse
										
										// Third pulse response is "interface is ready for OUTPUT"
										// If this is zero, the interface will break (back to S_INITIAL)
										//   and try again with a fresh 3-pulse poll.
										$display("Pulse 3: set testack_int = rx_ready %d", rx_ready);
										testack_int <= rx_ready;

										// If this is all, the target is starting a byte
										state <= S_OUTPUTPOLL;

										// If NACK (rx_ready == 0, no ACK pulse) then the target will delay,
										// then send another 3-pulse POLL.
										//
										// If ACK (TESTACK pulses with TESTREQ) then the target will delay,
										// then send eight bits with SHIFT_ZERO and SHIFT_ONE commands.
										// 
										// Each sequence of eight bits is preceded by a POLL.
										//
										// At the end of a burst of bytes, a dummy 3-pulse poll burst occurs to make
										// sure the last byte was recognised.
									end

				S_OUTPUTPOLL:	begin
										// Rising edge of 4th pulse

										// The 4th pulse indicates the target wants to start an INPUT cycle and wants to know if we're ready.
										// Send the TX Ready flag then go to S_INPUTPOLL
										testack_int <= 1'b0;
										state <= S_INPUTPOLL;
										$display("Pulse 4");

										// Signal that target is requesting data
										want_tx <= 1'b1;
									end
									
				S_INPUTPOLL:	begin
										// Repeated pulse 4, or pulse 5. INPUT poll.
										// The fourth pulse is repeated until TESTACK is asserted in response.
										// After TESTACK is asserted, shift the next bit.

										$display("Repeated pulse 4; tx_ready = %d", tx_ready);
										// No data available, loop around sending poll bits
										testack_int <= 1'b0;

										reset_spi_cs_int <= 1'b0;

										if (tx_ready) begin
											state <= S_START_INPUT;
											// Clear the input request signal
											want_tx <= 1'b0;
											// Enable spi_cs
											clear_spi_cs_int <= 1'b1;
										end
									end

				S_START_INPUT:	begin
										if (testack_int == 1'b0) begin
											// We're ready to send data; ack the input poll
											testack_int <= 1'b1;
											// End pulse
											clear_spi_cs_int <= 1'b0;
										end else begin
											$display("Pulse 5: send bit 7");
											// Latch the transmit data and send the ACK
											testack_int <= SPI_ONLY ? txin[0] : txin[7];
											// Load the shift register
											txshift <= txin;
											// Advance to next state
											state <= S_INPUT_BIT7;
										end
									end
									
				S_INPUT_BIT7:	begin
										// Pulse 6: send bit 6
										testack_int <= SPI_ONLY ? txin[0] : txshift[6];
										state <= S_INPUT_BIT6;
									end

				S_INPUT_BIT6:	begin
										// Pulse 7: send bit 5
										testack_int <= SPI_ONLY ? txin[0] : txshift[5];
										state <= S_INPUT_BIT5;
									end

				S_INPUT_BIT5:	begin
										// Pulse 8: send bit 4
										testack_int <= SPI_ONLY ? txin[0] : txshift[4];
										state <= S_INPUT_BIT4;
									end

				S_INPUT_BIT4:	begin
										// Pulse 9: send bit 3
										testack_int <= SPI_ONLY ? txin[0] : txshift[3];
										state <= S_INPUT_BIT3;
									end

				S_INPUT_BIT3:	begin
										// Pulse 10: send bit 2
										testack_int <= SPI_ONLY ? txin[0] : txshift[2];
										state <= S_INPUT_BIT2;
									end

				S_INPUT_BIT2:	begin
										// Pulse 11: send bit 1
										testack_int <= SPI_ONLY ? txin[0] : txshift[1];
										state <= S_INPUT_BIT1;
									end

				S_INPUT_BIT1:	begin
										$display("Pulse 12: send bit 0");
										testack_int <= SPI_ONLY ? txin[0] : txshift[0];
										state <= S_INPUT_BIT0;
									end

				S_INPUT_BIT0:	begin
										// Pulse 13: target is requesting another byte
										// Bit 0 is on the bus.
										// Raise TX_DONE (above), prepare to send the READY flag on the next pulse, then switch to S_INPUTPOLL
										// We need to do at least one polling cycle before we can send the next byte

										// It's easier to just pretend we're not ready yet, as S_INPUTPOLL
										// has logic to set up the SCK signals.
										testack_int <= 1'b0;
										want_tx <= 1'b1;
										reset_spi_cs_int <= 1'b1;
										state <= S_INPUTPOLL;
								end
			
				default:		begin
										// Default state
										state <= S_INITIAL;
								end
			endcase
		end	
	end

	wire actually_reset_drive_sck_for_input;
	assign actually_reset_drive_sck_for_input = reset_drive_sck_for_input || timer_expired_d;

	// Assert drive_sck_for_input between the start of S_INPUT_BIT7 and the end
	// of S_INPUT_BIT0.
	always @(posedge testreq or posedge actually_reset_drive_sck_for_input) begin
		if (actually_reset_drive_sck_for_input) begin
			drive_sck_for_input <= 1'b0;
		end else begin
			// posedge testreq
			if (state == S_START_INPUT && testack_int) begin
				// About to send bit 7 to the target.
				drive_sck_for_input <= 1'b1;
			end
		end
	end

	// Support for the above block: assert reset_drive_sck_for_input between the
	// end of S_INPUT_BIT0 and the end of S_INPUTPOLL.
	always @(negedge testreq or posedge timer_expired_d) begin
		if (timer_expired_d) begin
			reset_drive_sck_for_input <= 1'b0;
		end else begin
			// negedge testreq
			if (state == S_INPUTPOLL) begin
				reset_drive_sck_for_input <= 1'b0;
			end else if (state == S_INPUT_BIT0) begin
				// End of testreq pulse while in S_INPUT_BIT0; stop driving SCK
				reset_drive_sck_for_input <= 1'b1;
			end
		end
	end

	// Set tx_ready whenever tx_pending is high, and clear it after we've sent
	// the first bit to the target.
	always @(negedge testreq or posedge tx_pending) begin
		if (tx_pending) begin
			if (!tx_ready) $display("tx_pending active: set tx_ready");
			tx_ready <= 1'b1;
		end else begin
			// falling testreq clock edge
			if (state == S_INPUT_BIT7) begin
				// Target just read bit 7; tx_ready no longer needed
				$display("sent bit 7: clear tx_ready");
				tx_ready <= 1'b0;
			end
		end
	end

endmodule
