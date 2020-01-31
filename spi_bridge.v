// SPI interface to Acorn POST connector

// This uses the postbox module to provides a bridge between a target Acorn
// machine and a host microcontroller connected via SPI.

// Test connector pinout:

//  1: 5V
//  2: D0 (may need to be pulled high on older machines)
//  3: Testreq (LA21)
//  4: Testack (Romcs* / ROM nOE)
//  5: Rst*
//  6: 0V

// CPLD pinout:

//  1: clock_in_2mhz
//  2: testreq_in
//  3: testack_out
//  5: spi_sck
//  6: spi_mosi
//  ?: spi_cs
//  ?: spi_miso
//  ?: rx_ready
//  ?: tx_pending
//  ?: want_tx
//  ?: reset_in

module spi_bridge(
    input  testreq_in,     // Test connector pin 3
    output testack_out,    // Test connector pin 4
    output reset_out,      // Test connector pin 5

    input  clock_in_2mhz,  // 2MHz clock (available on A5000 expansion bus (SK9) pin A6)
    input  rx_ready,       // 1 when SPI remote is ready to receive
    input  tx_pending,     // 1 when SPI remote has data to send
    output want_tx,        // 1 when we are requesting a byte from the SPI remote
    output last_transaction_was_input,
    input  reset_in,       // 1 to reset the target machine

    output spi_cs,         // SPI chip select output
    output spi_sck,        // SPI clock output
    input  spi_miso,       // SPI clock input
    output spi_mosi        // SPI data output
);

    defparam p.TIMER_MAX = 20;  // Should be 31, but that cuts it a bit fine on the A5000
    defparam p.SPI_ONLY = 1;  // No shift register; all comms is by SPI

    assign reset_out = reset_in ? 1'b0 : 1'bZ;

    wire testack_int;
    assign testack_out = testack_int ? 1'b1 : 1'bZ;       // Only allow TESTACK to drive high or float

    wire [7:0] rxdata;
    assign spi_mosi = rxdata[0];

    postcode p(
        // Connections to target Archimedes machine
        .refclk(clock_in_2mhz),
        .testreq(testreq_in),
        .testack(testack_int),

        // Data received from machine under test
        .rxout(rxdata),           // received data (shift register)
        .rxready(rx_ready),       // receive ready (1=ready)

        .spi_clock_out(spi_sck),  // SPI clock, for data output
        .spi_cs_out(spi_cs),      // SPI chip select, for data output
        .last_transaction_was_input(last_transaction_was_input),

        // TX data in (for INPUT command)
        .txin({7'b0, spi_miso}),
        .tx_pending(tx_pending),  // 1 when SPI remote has a byte to send
        .want_tx(want_tx)         // 1 when target machine is requesting a byte
    );

endmodule
