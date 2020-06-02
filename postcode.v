// The postcode module handles communication with a target machine
// over the RISC OS POST debugging interface.  It is fully synchronous
// to refclk (48MHz), except for testack, which is combinatorial, gated
// with testreq.

module postcode

#(
    parameter REFCLK_FREQ = 48000000,

    // Timer timeout value, in refclk ticks
    parameter TIMER_MAX = 480  // 10 us
)

(
    // Clock inputs
    input refclk,                           // 48MHz reference clock

    // Test interface
    input  testreq,                         // Test REQuest (LA23)
    output testack,                         // Test ACKnowledge (TESTAK)

    // Data received from target
    output [7:0] rxout,                     // Received data (OUTPUT command)
    output rxfull,                          // High when rxout is valid
    input  rxreset,                         // Strobes to indicate remote is ready for another byte

    // Data to send to target
    input  [7:0] txin,                      // Data -> target (INPUT command)
    output txempty,                         // High when we can accept a new byte
    input  txstart                          // Strobes to indicate that there is a byte to send in txin
);

    // Synchronized version of testreq
    reg [2:0] testreq_sync;

    // Gate internal TESTACK against TESTREQ.
    // TESTACK should only ever drive high or open.
    reg testack_int = 0;
    assign testack = (testreq & testack_int & testreq_sync[2]);

    // Receive shift register -- data from the target ("output")
    reg[7:0] rxshift = 8'b0;
    assign rxout = rxshift;
    // Flag to say rxout has a byte that we're waiting for the host to receive
    reg rxbuf_full = 0;
    assign rxfull = rxbuf_full;

    // Byte ready to transmit to target, and flag to say if we have a byte in txbuf
    reg [7:0] txbuf = 0;
    reg txbuf_full = 0;
    assign txempty = !txbuf_full;

    // Monostable -- pulse train end detector
    // Reset to zero every time TESTREQ pulses high.
    // Stops on expiry.
    reg [8:0] timer;


    // Transmit shift register -- data to the target ("input")
    reg[7:0] txshift = 8'b0;

    // state machine
    localparam S_INITIAL        = 5'd0;     // initial state, waiting for first pulse
    localparam S_SHIFTONE   = 5'd1;     // one pulse received (shift in a '1')
    localparam S_SHIFTZERO  = 5'd2;     // two pulses received (shift in a '0')
    localparam S_OUTPUTPOLL = 5'd3;     // three pulses received (OUTPUT)
    localparam S_INPUTPOLL  = 5'd4;     // four pulses received (INPUT)
    localparam S_START_INPUT = 5'd13;
    localparam S_INPUT_BIT7 = 5'd5;     // shift out bits MSB..LSB in response to an INPUT command
    localparam S_INPUT_BIT6 = 5'd6;
    localparam S_INPUT_BIT5 = 5'd7;
    localparam S_INPUT_BIT4 = 5'd8;
    localparam S_INPUT_BIT3 = 5'd9;
    localparam S_INPUT_BIT2 = 5'd10;
    localparam S_INPUT_BIT1 = 5'd11;
    localparam S_INPUT_BIT0 = 5'd12;

    reg[4:0] state = S_INITIAL;
    reg [3:0] rxcounter = 4'b0;  // count of bits received

    always @(posedge refclk) begin

        // Accept new byte from host
        if (txstart && !txbuf_full) begin
            txbuf <= txin;
            txbuf_full <= 1'b1;
        end

        // The host has received a byte
        if (rxreset) begin
            rxbuf_full <= 1'b0;
            rxcounter <= 4'b0;
        end

        // Synchronize testreq
        testreq_sync <= {testreq_sync[1:0], testreq};
        if (testreq_sync[1]) begin
            timer <= 9'b0;
            if (!testreq_sync[2]) begin
                // Rising edge on testreq

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

                case (state)
                    S_INITIAL: begin
                        // Rising edge of first pulse

                        // Ack the first pulse
                        testack_int <= 1'b1;

                        // If this is all, the target is sending us a one
                        state <= S_SHIFTONE;
                    end

                    S_SHIFTONE: begin
                        // Rising edge of second pulse

                        // Ack the second pulse
                        testack_int <= 1'b1;

                        // If this is all, the target is sending us a zero
                        state <= S_SHIFTZERO;
                    end

                    S_SHIFTZERO: begin
                        // Rising edge of third pulse

                        // Third pulse response is "interface is ready for OUTPUT"
                        // If this is zero, the interface will break (back to S_INITIAL)
                        //   and try again with a fresh 3-pulse poll.
                        $display("Pulse 3: set testack_int = %d == !rxbuf_full", !rxbuf_full);
                        testack_int <= !rxbuf_full;

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

                    S_OUTPUTPOLL: begin
                        // Rising edge of 4th pulse

                        // The 4th pulse indicates the target wants to start an INPUT cycle and wants to know if we're ready.
                        // Send the TX Ready flag then go to S_INPUTPOLL
                        testack_int <= txbuf_full;
                        state <= S_INPUTPOLL;
                        $display("Pulse 4: txbuf_full = %d", txbuf_full);
                    end

                    S_INPUTPOLL: begin
                        // Repeated pulse 4, or pulse 5. INPUT poll.
                        // The fourth pulse is repeated until TESTACK is asserted in response.
                        // After TESTACK is asserted, shift the next bit.

                        $display("Repeated pulse 4; txbuf_full = %d", txbuf_full);

                        if (!testack_int) begin
                            testack_int <= txbuf_full;
                        end else begin
                            // We acked the last pulse, so we're ready
                            state <= S_INPUT_BIT7;
                            $display("Pulse 4: send bit 7");
                            // Latch the transmit data and send the ACK
                            testack_int <= txin[7];
                            // Load the shift register
                            txshift <= txin;
                            // Ready to accept another byte from the host to transmit to the target
                            txbuf_full <= 1'b0;
                            // Advance to next state
                            state <= S_INPUT_BIT7;
                        end
                    end

                    S_INPUT_BIT7: begin
                        // Pulse 6: send bit 6
                        testack_int <= txshift[6];
                        state <= S_INPUT_BIT6;
                    end

                    S_INPUT_BIT6: begin
                        // Pulse 7: send bit 5
                        testack_int <= txshift[5];
                        state <= S_INPUT_BIT5;
                    end

                    S_INPUT_BIT5: begin
                        // Pulse 8: send bit 4
                        testack_int <= txshift[4];
                        state <= S_INPUT_BIT4;
                    end

                    S_INPUT_BIT4: begin
                        // Pulse 9: send bit 3
                        testack_int <= txshift[3];
                        state <= S_INPUT_BIT3;
                    end

                    S_INPUT_BIT3: begin
                        // Pulse 10: send bit 2
                        testack_int <= txshift[2];
                        state <= S_INPUT_BIT2;
                    end

                    S_INPUT_BIT2: begin
                        // Pulse 11: send bit 1
                        testack_int <= txshift[1];
                        state <= S_INPUT_BIT1;
                    end

                    S_INPUT_BIT1: begin
                        $display("Pulse 12: send bit 0");
                        testack_int <= txshift[0];
                        state <= S_INPUT_BIT0;
                    end

                    S_INPUT_BIT0: begin
                        $display("Pulse 13: target is requesting another byte; txbuf_full=%d", txbuf_full);
                        testack_int <= txbuf_full;
                        state <= S_INPUTPOLL;
                    end

                    default: begin
                        // Default state
                        state <= S_INITIAL;
                    end
                endcase
            end
        end else begin
            if (timer <= TIMER_MAX) begin
                timer = timer + 9'd1;
            end
            if (timer == TIMER_MAX) begin
                // Timer just expired
                state <= S_INITIAL;

                if (!rxbuf_full && (state == S_SHIFTZERO || state == S_SHIFTONE)) begin
                    rxshift <= {rxshift[6:0], state == S_SHIFTZERO ? 1'b0 : 1'b1};
                    rxcounter <= rxcounter + 4'd1;
                    if (rxcounter == 4'b0111) begin
                        rxbuf_full <= 1'b1;
                    end
                end
            end
        end
    end

endmodule
