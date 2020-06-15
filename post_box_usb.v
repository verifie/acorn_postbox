// Top level module for MachXO256 FPGA on myelin's post_box_usb board

// This uses the postcode module to provides a bridge between a target Acorn
// machine and an ATSAMD21E18A microcontroller connected via SPI.

// Test connector pinout:

//  1: 5V
//  2: D0 (may need to be pulled high on older machines)
//  3: Testreq (LA21)
//  4: Testack (Romcs* / ROM nOE)
//  5: Rst*
//  6: 0V

module post_box_usb(
    // Test things
    output fpga_GPIO34,
    output fpga_GPIO35,
    
    // Connections to microcontroller
    input  fpga_clock_48mhz,    // 48 MHz clock from MCU PA10
    input  reset_in,            // 1 to reset the target machine
    output target_power_out,    // 1 when the target is powered

    input  fpga_spi_cs,         // SPI chip select input
    input  fpga_spi_sck,        // SPI clock input
    input  fpga_spi_mosi,       // SPI data input
    output fpga_spi_miso,       // SPI data output

    // Connections to buffers (and target Archimedes machine)
    input  testreq_3v,          // TESTREQ from target (buffered)
    output testack_noe,         // /OE for TESTACK (drive low to drive TESTACK high)
    output target_reset_noe,    // /OE for RESET (drive low to reset target)
    input  target_power_3v,     // Target 5V line (buffered)
    output hotswap_noe          // /OE for 74LCX125 buffer
);

// `ifdef SYNTHESIS
//  wire osc_int;
//  OSCC OSCInst0 (.OSC(osc_int));
//     assign fpga_GPIO35 = osc_int;
//     assign fpga_GPIO34 = fpga_clock_48mhz;
// `endif

    reg target_power_out_int = 1'b0;
    assign target_power_out = target_power_out_int;

    assign hotswap_noe = 1'b0;  // Enable hotswap buffer whenever FPGA is powered

    assign target_reset_noe = reset_in ? 1'b0 : 1'b1;  // reset is driven low when target_reset_noe==0

    wire testack_int;
    assign testack_noe = testack_int ? 1'b0 : 1'b1;  // testack is driven high when testack_noe==0

    // these two are ultra glitchy when scoped at the archimedes end; let's see
    // if they're any better from the POV of the FPGA
    assign fpga_GPIO35 = testreq_3v;
    assign fpga_GPIO34 = testack_int;

    reg spi_miso_int = 1'b1;
    assign fpga_spi_miso = spi_miso_int;

    // Data received from target
    wire [7:0] postcode_rxdata;
    wire postcode_rxfull;
    reg postcode_rxreset = 1'b0;
    // Data to transmit to target
    // reg [7:0] postcode_txdata = 0;  // just wiring this one up to shi_shifter
    wire postcode_txempty;
    reg postcode_txstart = 1'b0;

    reg [2:0] spi_cs_sync;
    reg [2:0] spi_sck_sync;
    reg [2:0] spi_mosi_sync;
    reg [4:0] spi_counter = 5'b0;
    reg [7:0] spi_shifter = 0;

    reg spi_we_have_a_byte_to_send = 0;
    reg spi_we_have_buffer_space = 0;
    reg spi_remote_has_a_byte_to_send = 0;
    reg spi_remote_has_buffer_space = 0;

    postcode p(
        // Connections to target Archimedes machine
        .refclk(fpga_clock_48mhz),
        .testreq(testreq_3v),
        .testack(testack_int),

        // Data received from machine under test
        .rxout(postcode_rxdata),
        .rxfull(postcode_rxfull),
        .rxreset(postcode_rxreset),

        // Data to transmit to the machine under test
        .txin(spi_shifter),
        .txempty(postcode_txempty),
        .txstart(postcode_txstart)
    );

    always @(posedge fpga_clock_48mhz) begin
        // For some reason I'm getting these warnings here; is target_power_3v unconnected externally?
        // WARNING - synthesis: I/O Port target_power_out 's net has no driver and is unused.
        // WARNING - synthesis: I/O Port target_power_3v 's net has no driver and is unused.
        target_power_out_int <= target_power_3v;

        spi_cs_sync <= {spi_cs_sync[1:0], fpga_spi_cs};
        spi_sck_sync <= {spi_sck_sync[1:0], fpga_spi_sck};
        spi_mosi_sync <= {spi_mosi_sync[1:0], fpga_spi_mosi};

        postcode_rxreset <= 1'b0;
        postcode_txstart <= 1'b0;

        if (spi_cs_sync[1] == 1'b1) begin
            // SPI inactive
            spi_counter <= 5'b0;
        end else if (spi_cs_sync[2] == 1'b1 && spi_cs_sync[1] == 1'b0) begin
            // spi_cs falling edge!
            spi_miso_int <= postcode_rxfull;
            spi_we_have_a_byte_to_send <= postcode_rxfull;
            $display("SPI start: rxfull %d txempty %d rxdata %02x", postcode_rxfull, postcode_txempty, postcode_rxdata);
        end else if (spi_sck_sync[2] == 1'b0 && spi_sck_sync[1] == 1'b1) begin
            // spi_sck rising edge!
            spi_counter <= spi_counter + 5'd1;
            if (spi_counter == 0) begin
                spi_remote_has_a_byte_to_send <= spi_mosi_sync[1];
                spi_miso_int <= postcode_txempty;
                spi_we_have_buffer_space <= postcode_txempty;
            end else if (spi_counter == 1) begin
                spi_remote_has_buffer_space <= spi_mosi_sync[1];
                // spi_miso_int <= target_power_3v;
            end else if (spi_counter == 2) begin
                // reset_in <= spi_mosi_sync[1];
            end else if (spi_counter == 6) begin
                // Set up shifter, which will
                if (spi_we_have_a_byte_to_send && spi_remote_has_buffer_space) begin
                    spi_shifter <= postcode_rxdata;
                    postcode_rxreset <= 1'b1;
                end
            end else if (spi_counter >= 7 && spi_counter < 16) begin
                spi_miso_int <= spi_shifter[7];
                spi_shifter <= {spi_shifter[6:0], spi_mosi_sync[1]};
                if (spi_counter == 15) begin
                    if (spi_remote_has_a_byte_to_send && spi_we_have_buffer_space) begin
                        postcode_txstart <= 1'b1;
                    end
                end
            end
        end
    end

endmodule
